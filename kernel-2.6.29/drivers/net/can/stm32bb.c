/*
 * CAN bus driver for STM32 based CAN Controller
 *
 * Based on MCP251x CAN controller driver by Christian Pellegrin
 * <chripell@evolware.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <socketcan/can.h>
#include <socketcan/can/core.h>
#include <socketcan/can/dev.h>
#include <socketcan/can/platform/stm32bb.h>
#include <linux/power_supply.h>
#include "stm32bb.h"


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#error This driver does not support Kernel versions < 2.6.22
#endif


/*
 * Buffer size required for the largest SPI transfer (i.e., reading a
 * frame)
 */
#define CAN_MESSAGE_SIZE 13
#define CAN_BUFFER_SIZE 16
#define SPI_TRANSFER_BUF_LEN	(CAN_MESSAGE_SIZE * CAN_BUFFER_SIZE)

#define TX_ECHO_SKB_MAX	1

#define DEVICE_NAME "stm32bb"

static int stm32bb_enable_dma; /* Enable SPI DMA. Default: 0 (Off) */
static int stm32bb_spi_speed = 0;
module_param(stm32bb_enable_dma, int, S_IRUGO);
MODULE_PARM_DESC(stm32bb_enable_dma, "Enable SPI DMA. Default: 0 (Off)");

module_param(stm32bb_spi_speed, int, S_IRUGO);
MODULE_PARM_DESC(stm32bb_spi_speed, "Override SPI speed (kHz)");

static struct can_bittiming_const stm32bb_bittiming_const = {
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

struct stm32bb_priv {
	struct can_priv	   can;
	struct net_device *net;
	struct spi_device *spi;

	struct mutex spi_lock; /* SPI buffer lock */
	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	dma_addr_t spi_tx_dma;
	dma_addr_t spi_rx_dma;

	struct sk_buff *tx_skb;
	int tx_len;
	atomic_t data_ready;
	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct irq_work;
	wait_queue_head_t spi_wait;

	int force_quit;
	int after_suspend;
#define AFTER_SUSPEND_UP 1
#define AFTER_SUSPEND_DOWN 2
	int restart_tx;
	struct stm32bb_pwr_t pwr_data;
	struct stm32bb_iset_t pwr_iset;
};

/*
 * low level HW functions 
 */
static int stm32bb_spi_trans(struct spi_device *spi, uint8_t cmd, int len)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.cs_change = 0,
		.len = 1,
	};
	struct spi_message m;
	uint8_t tmp;
	int ret;

	spi_message_init(&m);

	if (stm32bb_enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	/* replace first byte by cmd */
	tmp = priv->spi_tx_buf[0];
	priv->spi_tx_buf[0] = cmd;

	spi_message_add_tail(&t, &m);

	/* send message */
	ret = spi_sync(spi, &m);
	if (ret) {
		dev_err(&spi->dev, "spi transfer failed: ret = %d\n", ret);
		return ret;
	}

	/* wait for transfer ready */
	wait_event_interruptible(priv->spi_wait, atomic_read(&priv->data_ready));

	if (atomic_read(&priv->data_ready) != 1)
		return -1;

	atomic_set(&priv->data_ready, 0);

	/* return saved byte */
	priv->spi_tx_buf[0] = tmp;
	t.len = len;

	spi_message_init(&m);

	if (stm32bb_enable_dma) {
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret)
		dev_err(&spi->dev, "spi transfer failed: ret = %d\n", ret);

	return ret;
}

static uint32_t stm32bb_read_reg(struct spi_device *spi, uint8_t reg, uint8_t len)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	uint32_t val = 0;
	uint8_t *buf = priv->spi_rx_buf;

	mutex_lock(&priv->spi_lock);

	stm32bb_spi_trans(spi, reg & 0x3f, len);

	/* we receive LSByte first */
	val = buf[0];
	val |= buf[1] << 8;
	val |= buf[2] << 16;
	val |= buf[3] << 24;

#if 0 
	dev_info(&spi->dev, "read_reg(0x%04x, %d) -> 0x%08x\n", reg, len, val);
#endif

	mutex_unlock(&priv->spi_lock);

	return val;
}

static void stm32bb_write_reg(struct spi_device *spi, uint8_t reg, uint32_t val, uint8_t len)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	uint8_t *buf = priv->spi_tx_buf;

	mutex_lock(&priv->spi_lock);

	buf[0] = val & 0xff;
	buf[1] = (val >> 8) & 0xff;
	buf[2] = (val >> 16) & 0xff;
	buf[3] = (val >> 24) & 0xff;

	stm32bb_spi_trans(spi, reg | CMD_WRITE, len);

#if 0
	dev_info(&spi->dev, "write_reg(0x%04x, %d) <- 0x%08x\n", reg, len, val);
#endif

	mutex_unlock(&priv->spi_lock);
}

static void stm32bb_hw_tx(struct spi_device *spi, struct can_frame *frame)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	uint32_t exide, rtr;
	uint8_t *buf = priv->spi_tx_buf;

	exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0; /* Extended ID Enable */
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0; /* Remote transmission */

	mutex_lock(&priv->spi_lock);

	/* write flag */
	buf[0] = rtr << 4 | exide << 5 | (frame->can_dlc & 0x0f);

	/* write ID - LSB first*/
	buf[1] = frame->can_id & 0xff;
	buf[2] = (frame->can_id >> 8) & 0xff;
	buf[3] = (frame->can_id >> 16) & 0xff;
	buf[4] = (frame->can_id >> 24) & 0xff;

	/* copy msg data */
	memcpy(buf + 5, frame->data, frame->can_dlc);

	/* send frame */
	stm32bb_spi_trans(spi, CAN_TX | CMD_WRITE, CAN_MESSAGE_SIZE);
	mutex_unlock(&priv->spi_lock);
}

static void stm32bb_hw_rx(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	struct sk_buff *skb;
	struct can_frame *frame;
	uint8_t *buf = priv->spi_rx_buf;
	int i;

	mutex_lock(&priv->spi_lock);

	stm32bb_spi_trans(spi, CAN_RX0, CAN_MESSAGE_SIZE * CAN_BUFFER_SIZE);

	/* Parse each valid message from received buffer */
	for (i = 0; i < CAN_BUFFER_SIZE; i++) {
		if (buf[0] & CAN_MSG_INV) {
			break;
		}

		/* allocate new skb for each message */
		skb = alloc_can_skb(priv->net, &frame);
		if (!skb) {
			dev_err(&spi->dev, "cannot allocate RX skb\n");
			priv->net->stats.rx_dropped++;
			return;
		}

		/* Extract DLC */
		frame->can_dlc = get_can_dlc(buf[0] & CAN_MSG_SIZE);

		/* Extract ID with additional flags */
		frame->can_id = buf[4] << 24 | buf[3] << 16 | buf[2] << 8 | buf[1];
		frame->can_id |= (buf[0] & CAN_MSG_RTR) ? CAN_RTR_FLAG : 0; /* rtr */
		frame->can_id |= (buf[0] & CAN_MSG_EID) ? CAN_EFF_FLAG : 0; /* Extended ID Enable */

		/* Copy data */
		memcpy(frame->data, buf + 5, frame->can_dlc);

		/* Update stats and submit */
		priv->net->stats.rx_packets++;
		priv->net->stats.rx_bytes += frame->can_dlc;
		netif_rx(skb);

		/* advance buffer */
		buf += CAN_MESSAGE_SIZE;
	}

	mutex_unlock(&priv->spi_lock);

}

static void stm32bb_get_measurement(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	uint8_t *buf = priv->spi_rx_buf;

	mutex_lock(&priv->spi_lock);

	stm32bb_spi_trans(spi, PWR_DATA, 8);

	/* Extract measurements from received buffer */
	priv->pwr_data.i_bat = buf[0] | buf[1] << 8;
	priv->pwr_data.i_sys = buf[2] | buf[3] << 8;
	priv->pwr_data.v_bat = buf[4] | buf[5] << 8;
	priv->pwr_data.v_ac  = buf[6] | buf[7] << 8;

	mutex_unlock(&priv->spi_lock);
}

static void stm32bb_set_current(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	uint8_t *buf = priv->spi_tx_buf;

	mutex_lock(&priv->spi_lock);

	buf[0] = priv->pwr_iset.i_ac & 0xff;
	buf[1] = (priv->pwr_iset.i_ac >> 8) & 0xff;

	buf[2] = priv->pwr_iset.i_bat & 0xff;
	buf[3] = (priv->pwr_iset.i_bat >> 8) & 0xff;

	stm32bb_spi_trans(spi, PWR_I_SET | CMD_WRITE, 4);
	mutex_unlock(&priv->spi_lock);
}

static void stm32bb_can_sleep(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	uint16_t reg;

	/* switch controller to init mode */
	reg = stm32bb_read_reg(spi, CAN_CTRL, 2);
	reg |= CAN_CTRL_INIT;
	stm32bb_write_reg(spi, CAN_CTRL, reg, 2);

	/* disable can interrupts */
	reg = stm32bb_read_reg(spi, SYS_INTE, 2);
	reg &= ~(SYS_INT_CANTXIF | SYS_INT_CANRX0IF | SYS_INT_CANRX1IF | SYS_INT_CANERRIF);
	stm32bb_write_reg(spi, SYS_INTE, reg, 2);

	priv->can.state = CAN_STATE_STOPPED;
}

static void stm32bb_set_normal_mode(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	unsigned long timeout;
	uint16_t reg;

	stm32bb_read_reg(spi, SYS_INTE, 2);

	/* Enable CAN interrupts */
	reg = stm32bb_read_reg(spi, SYS_INTE, 2);
	reg |= SYS_INT_CANTXIF | SYS_INT_CANRX0IF | SYS_INT_CANERRIF;
	stm32bb_write_reg(spi, SYS_INTE, reg, 2);

	reg = stm32bb_read_reg(spi, CAN_CTRL, 2);

	/* enable error reporting */
	reg |= CAN_CTRL_IERR;
	/* enable automatic bus-off managenment */
	/* reg |= CAN_CTRL_ABOM; */

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* Put device into loopback mode */
		reg |= CAN_CTRL_LOOP;
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* Put device into listen-only mode */
		reg |= CAN_CTRL_SILM;
 	} else {
		/* Put device into normal mode */
		reg |= (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT) ? CAN_CTRL_OSM : 0;
	}

	/* normal mode means init off */
	reg &= ~CAN_CTRL_INIT;
	stm32bb_write_reg(spi, CAN_CTRL, reg, 2);

	/* Wait for the device to enter normal mode */
	timeout = jiffies + HZ;
	while (stm32bb_read_reg(spi, CAN_STATUS, 2) & CAN_STAT_INAK) {
		schedule();
		if (time_after(jiffies, timeout)) {
			dev_err(&spi->dev, "Didn't enter normal mode\n");
			return;
		}
	}

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
}

static int stm32bb_do_set_bittiming(struct net_device *net)
{
	struct stm32bb_priv *priv = netdev_priv(net);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct spi_device *spi = priv->spi;
	uint32_t timing_reg = 0;
	
	/* Bit timing */
	timing_reg |= ((bt->sjw) << 8) & 0x00000f00;
	timing_reg |= ((bt->phase_seg2 - 1) << 4) & 0x000000f0;
	timing_reg |= (bt->phase_seg1 + bt->prop_seg - 1) & 0x0000000f;

	/* BRP setting */
	timing_reg <<= 16;
	timing_reg |= (bt->brp - 1) & 0x03ff;

	stm32bb_write_reg(spi, CAN_TIMING, timing_reg, 4);

	dev_info(&spi->dev, "Timing  BRP: %d SJW: %d PROP: %d TS1: %d TS2: %d\n", bt->brp, bt->sjw, bt->prop_seg, bt->phase_seg1, bt->phase_seg2);

	return 0;
}

static int stm32bb_pwr_setup(struct stm32bb_priv *priv)
{
	struct spi_device *spi = priv->spi;
	uint16_t reg;

	/* Enable power control */
	reg = stm32bb_read_reg(spi, PWR_CTRL, 2);
	reg |= PWR_CTRL_ADC | PWR_CTRL_PWM | PWR_CTRL_EN;
	stm32bb_write_reg(spi, PWR_CTRL, reg, 2);

	return 0;
}

static int stm32bb_can_setup(struct net_device *net, struct stm32bb_priv *priv,
			 struct spi_device *spi)
{
	if (priv->can.state != CAN_STATE_STOPPED) {
		dev_warn(&spi->dev, "Setup, when not in stopped state\n");
		return 1;	
	}
	stm32bb_do_set_bittiming(net);

	return 0;
}

static void stm32bb_hw_reset(struct spi_device *spi)
{
	unsigned long timeout;

	/* write reset magic to reset the device */
	stm32bb_write_reg(spi, SYS_RESET, SYS_RESET_MAGIC, 2);

	/* Wait for reset to finish
	 * The value here depends on the version of FW
	 * DEBUG version includes extra startup delay
	 */
	timeout = jiffies + HZ;
	while (1) {
		schedule();
		if (time_after(jiffies, timeout)) {
			return;
		}
	}
}

static void stm32bb_can_reset(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	unsigned long timeout;

	/* FIXME - this doesn't work very well 
	 * stm32bb_write_reg(spi, CAN_CTRL, CAN_CTRL_RST, 2);
	 * mdelay(50);
	 */

	/* lets enter init state */
	stm32bb_write_reg(spi, CAN_CTRL, CAN_CTRL_INIT, 2);

	/* Wait for the device to enter init mode */
	timeout = jiffies + HZ;
	while ((stm32bb_read_reg(spi, CAN_STATUS, 2) & CAN_STAT_INAK) == 0) {
		schedule();
		if (time_after(jiffies, timeout)) {
			dev_err(&spi->dev, "STM32 didn't enter initialization mode\n");
			return;
		}
	}
	priv->can.state = CAN_STATE_STOPPED;
}

static int stm32bb_do_set_mode(struct net_device *net, enum can_mode mode)
{
	struct stm32bb_priv *priv = netdev_priv(net);

	switch (mode) {
	case CAN_MODE_SLEEP:
	case CAN_MODE_STOP:
		netif_stop_queue(net);
		stm32bb_can_sleep(priv->spi);
		break;
	case CAN_MODE_START:
		/* We have to delay work since SPI I/O may sleep */
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
		priv->restart_tx = 1;
/*		if (priv->can.restart_ms == 0)
			priv->after_suspend = AFTER_SUSPEND_RESTART;*/
		queue_work(priv->wq, &priv->irq_work);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int stm32bb_hw_probe(struct spi_device *spi)
{
	uint16_t id;

	stm32bb_hw_reset(spi);

	id = (uint16_t)stm32bb_read_reg(spi, SYS_ID, 2);

	dev_info(&spi->dev, "System ID = 0x%04x\n", id);

	return (id == SYS_ID_MAGIC) ? 0 : 1;
}

/*
 * netdev interface functions
 */
static void stm32bb_clean(struct net_device *net)
{
	struct stm32bb_priv *priv = netdev_priv(net);

	net->stats.tx_errors++;
	if (priv->tx_skb)
		dev_kfree_skb(priv->tx_skb);
	if (priv->tx_len)
		can_free_echo_skb(priv->net, 0);
	priv->tx_skb = NULL;
	priv->tx_len = 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int stm32bb_hard_start_xmit(struct sk_buff *skb, struct net_device *net)
#else
static netdev_tx_t stm32bb_hard_start_xmit(struct sk_buff *skb,
					   struct net_device *net)
#endif
{
	struct stm32bb_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	if (priv->tx_skb || priv->tx_len) {
		dev_warn(&spi->dev, "hard_xmit called while tx busy\n");
		netif_stop_queue(net);
		return NETDEV_TX_BUSY;
	}

	if (can_dropped_invalid_skb(net, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(net);
	priv->tx_skb = skb;
	net->trans_start = jiffies;
	queue_work(priv->wq, &priv->tx_work);

	return NETDEV_TX_OK;
}

static int stm32bb_open(struct net_device *net)
{
	struct stm32bb_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	int ret;

	ret = open_candev(net);
	if (ret) {
		dev_err(&spi->dev, "unable to set initial baudrate!\n");
		return ret;
	}

	priv->force_quit = 0;
	priv->tx_skb = NULL;
	priv->tx_len = 0;

	stm32bb_can_reset(spi);
	stm32bb_can_setup(net, priv, spi);
	stm32bb_set_normal_mode(spi);

	return 0;
}

static int stm32bb_stop(struct net_device *net)
{
	struct stm32bb_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	close_candev(net);

	priv->force_quit = 1;
	flush_workqueue(priv->wq);

	if (priv->tx_skb || priv->tx_len)
		stm32bb_clean(net);

	stm32bb_can_sleep(spi);

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
static const struct net_device_ops stm32bb_netdev_ops = {
	.ndo_open = stm32bb_open,
	.ndo_stop = stm32bb_stop,
	.ndo_start_xmit = stm32bb_hard_start_xmit,
};
#endif

/*
 * workers
 */
static void stm32bb_tx_work_handler(struct work_struct *ws)
{
	struct stm32bb_priv *priv = container_of(ws, struct stm32bb_priv,
						 tx_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	struct can_frame *frame;

	if (priv->tx_skb) {
		frame = (struct can_frame *)priv->tx_skb->data;
  
		if (priv->can.state == CAN_STATE_BUS_OFF) {
			dev_warn(&spi->dev, "Trying to transmit while bus-off ! \n");
			stm32bb_clean(net);
			netif_wake_queue(net);
			return;
		}

		frame->can_dlc = get_can_dlc(frame->can_dlc);
		stm32bb_hw_tx(spi, frame);
		priv->tx_len = 1 + frame->can_dlc;
		can_put_echo_skb(priv->tx_skb, net, 0);
		priv->tx_skb = NULL;
	}
}

static void stm32bb_irq_work_handler(struct work_struct *ws)
{
	struct stm32bb_priv *priv = container_of(ws, struct stm32bb_priv,
						 irq_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	uint16_t intf;
	int can_id_res = 0;

	/* TODO: this probably wrong ! Test TEst TEST !
	 * shouldn't this be moved to separate work handler ???
	 */
	if (priv->after_suspend || priv->restart_tx) {
		mdelay(10);
		if ((priv->restart_tx) || (priv->after_suspend & AFTER_SUSPEND_UP)) {
			dev_warn(&spi->dev, "ISR: Restarting CAN hw !\n");
			stm32bb_can_reset(spi);
			stm32bb_can_setup(net, priv, spi);
			stm32bb_set_normal_mode(spi);
		} else {
			/* we were just resumed from sleep
			 * prolly never going to get here :)
			 */
			dev_warn(&spi->dev, "ISR: Got here  !\n");
			stm32bb_can_sleep(spi);
		}
		/* Clean since we lost tx buffer */
		if (priv->tx_skb || priv->tx_len) {
			stm32bb_clean(net);
		}
		if (netif_queue_stopped(net))
			netif_wake_queue(net);
		if (priv->restart_tx) {
			dev_warn(&spi->dev, "ISR: Restarts ++ !\n");
			priv->restart_tx = 0;
			can_id_res = CAN_ERR_RESTARTED;
			priv->can.can_stats.restarts++;
		}
		priv->after_suspend = 0;
	}


	while (!priv->force_quit && !freezing(current)) {
		intf = stm32bb_read_reg(spi, SYS_INTF, 2);
		if (intf == 0x0000) {
			/* only way to leave loop is to serve every interrupt !!! */
			break;
		}

		/* if there is interrupt from can, and we are not int bus-off state
		 * otherwise we are waiting for higher level to restart us
		 */ 
		if ((intf & SYS_INT_CANMASK) && (!(priv->can.restart_ms == 0 && priv->can.state == CAN_STATE_BUS_OFF))) {
			int can_id_err = 0, data1 = 0;
			uint32_t eflag;
			uint16_t can_status;
			enum can_state new_state = CAN_STATE_ERROR_ACTIVE;

			/* make snapshot of can status reg */
			/* FIXME: sometimes first read returns bogus data */
			stm32bb_read_reg(spi, CAN_STATUS, 2);
			can_status = stm32bb_read_reg(spi, CAN_STATUS, 2);
#if 0
			dev_info(&spi->dev, "ISR: intf: %04x status: %04x \n", intf, can_status);
#endif
			if (intf & SYS_INT_CANERRIF) {
				eflag = stm32bb_read_reg(spi, CAN_ERR, 4);
				dev_info(&spi->dev, "ISR: Error: 0x%04x\n", eflag);
				dev_info(&spi->dev, "ISR: Error: TEC: 0x%02x REC: 0x%02x LEC: 0x%02x flags: %s %s %s\n",
					((eflag & CAN_ERR_TEC) >> 16), ((eflag & CAN_ERR_REC) >> 24),
					((eflag & CAN_ERR_LEC) >> 4), (eflag & CAN_ERR_BOFF)?"BOFF":"",
					(eflag & CAN_ERR_EPVF)?"EOVF":"", (eflag & CAN_ERR_EWGF)?"EWGF":"");

				net->stats.tx_errors++;

				if (eflag & CAN_ERR_EWGF) {
					priv->can.can_stats.error_warning++;
					new_state = CAN_STATE_ERROR_WARNING;
					can_id_err = CAN_ERR_CRTL;
					}
				if (eflag & CAN_ERR_EPVF) {
						priv->can.can_stats.error_passive++;
						new_state = CAN_STATE_ERROR_PASSIVE;
					can_id_err = CAN_ERR_CRTL;
				}
				if (eflag & CAN_ERR_BOFF) {
					priv->can.can_stats.bus_off++;
					new_state = CAN_STATE_BUS_OFF;
					can_id_err = CAN_ERR_BUSOFF;
				}

				if (new_state == CAN_STATE_ERROR_PASSIVE) {
					if (((eflag & CAN_ERR_REC) >> 24) > 127) {
						data1 |= CAN_ERR_CRTL_RX_PASSIVE;
					}
					if (((eflag & CAN_ERR_TEC) >> 16) > 127) {
						data1 |= CAN_ERR_CRTL_TX_PASSIVE;
					}
				}
				if (new_state == CAN_STATE_ERROR_WARNING) {
					if (((eflag & CAN_ERR_REC) >> 24) >= 96){
						data1 |= CAN_ERR_CRTL_RX_WARNING;
					}
					if (((eflag & CAN_ERR_TEC) >> 16) >= 96) {
						data1 |= CAN_ERR_CRTL_TX_WARNING;
					}
				}
				/* update new state */
				priv->can.state = new_state;

				if (priv->can.state == CAN_STATE_BUS_OFF) {
					if (priv->can.restart_ms == 0) {
						can_bus_off(net);
						stm32bb_can_sleep(spi);
					}
				}
			} /* CAN_ERRIF */

			/* in case of "event" notify higher level */
			if (can_id_res != 0 || can_id_err != 0) {
				struct sk_buff *skb;
				struct can_frame *frame;

				/* Create error frame */
				skb = alloc_can_err_skb(net, &frame);
				if (skb) {
					/* Set error frame flags based on bus state */
					frame->can_id = can_id_err | can_id_res;
					frame->data[1] = data1;
					netif_rx(skb);
				} else {
					dev_info(&spi->dev, "cannot allocate error skb\n");
				}
			}

			/* Data in RX0 */
			if (intf & SYS_INT_CANRX0IF) {
				if (can_status & CAN_STAT_RXOV) {
					dev_info(&spi->dev, "Possible RX0 overflow!\n");
				}
#if 0
				dev_info(&spi->dev, "ISR: RX0 interrupt %d msgs status = %d\n", (can_status >> 8) & 0x0f, can_status);
#endif
				stm32bb_hw_rx(spi);
			}

			/* Message TX interrupt  */
			if (intf & SYS_INT_CANTXIF) {
#if 0
				dev_info(&spi->dev, "ISR: TX interrupt 0x%04x\n", can_status);
#endif
				if (can_status & CAN_STAT_ALST) {
					priv->can.can_stats.arbitration_lost++;
					dev_warn(&spi->dev, "Arbitration lost !\n");
				}
				if (can_status & CAN_STAT_TERR) {
					dev_warn(&spi->dev, "Transmit error  !\n");
				}
				if (can_status & CAN_STAT_TXOK) {
					net->stats.tx_packets++;
					net->stats.tx_bytes += priv->tx_len - 1;
					if (priv->tx_len) {
						can_get_echo_skb(net, 0);
						priv->tx_len = 0;
					}
					if (priv->can.state != CAN_STATE_BUS_OFF) {
						netif_wake_queue(net);
					}
				}
			}

			/* CAN was reset */
			if (intf & SYS_INT_CANRSTIF) {
				dev_info(&spi->dev, "ISR: CAN was reset due to error\n");

				priv->can.state = CAN_STATE_ERROR_ACTIVE;

				if (priv->tx_skb || priv->tx_len) {
					dev_info(&spi->dev, "ISR: TX was in progress, doing clean-up\n");
					stm32bb_clean(net);
					netif_wake_queue(net);
				}
			}

		}
/*
		if (intf & SYS_INT_PWRMASK) {
			uint16_t pwr_status;
			pwr_status = stm32bb_read_reg(spi, PWR_STATUS, 2);
			if (intf & SYS_INT_PWRALARM) {
				if (pwr_status & PWR_STAT_ALARM) {
					printk("ALARM ON !!\n");
				} else {
					printk("ALARM OFF !!\n");
				}
			}

			if (intf & SYS_INT_PWRAC) {
				if (pwr_status & PWR_STAT_ACPRE) {
					printk("Adapter pluged in !!\n");
				} else {
					printk("Adapter unpluged !!\n");
				}
			}
		}
*/
	}
}

static irqreturn_t stm32bb_can_isr(int irq, void *dev_id)
{
	struct net_device *net = (struct net_device *)dev_id;
	struct stm32bb_priv *priv = netdev_priv(net);

	/* Schedule bottom half */
	if (!work_pending(&priv->irq_work))
		queue_work(priv->wq, &priv->irq_work);

	return IRQ_HANDLED;
}

static irqreturn_t stm32bb_transfer_ready_isr(int irq, void *dev_id)
{
	struct spi_device *spi = (struct spi_device *)dev_id;
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);

	/* Wake-up sleeping task */
	atomic_set(&priv->data_ready, 1);
	wake_up_interruptible(&priv->spi_wait);

	return IRQ_HANDLED;
}

/*
 * POWER interface 
 */
static int stm32bb_ac_get_prop(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct stm32bb_priv *priv = dev_get_drvdata(psy->dev->parent);
	struct spi_device *spi = priv->spi;
	int ret = 0;
	uint16_t pwr_status;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		pwr_status = stm32bb_read_reg(spi, PWR_STATUS, 2);
		val->intval = !!(pwr_status & PWR_STAT_ACPRE);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		stm32bb_get_measurement(spi);
		val->intval = (int)priv->pwr_data.v_ac;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property stm32bb_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static struct power_supply stm32bb_ac = {
	.name = "stm32bb-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = stm32bb_ac_props,
	.num_properties = ARRAY_SIZE(stm32bb_ac_props),
	.get_property = stm32bb_ac_get_prop,
};

static int stm32bb_sys_get_prop(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct stm32bb_priv *priv = dev_get_drvdata(psy->dev->parent);
	struct spi_device *spi = priv->spi;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		stm32bb_get_measurement(spi);
		val->intval = (int)priv->pwr_data.i_sys;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property stm32bb_sys_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static struct power_supply stm32bb_sys = {
	.name = "stm32bb-sys",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = stm32bb_sys_props,
	.num_properties = ARRAY_SIZE(stm32bb_sys_props),
	.get_property = stm32bb_sys_get_prop,
};

static int stm32bb_bat_get_prop(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct stm32bb_priv *priv = dev_get_drvdata(psy->dev->parent);
	struct spi_device *spi = priv->spi;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1; /* present */
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = 0; /* invalid */
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		stm32bb_get_measurement(spi);
		val->intval = (int)priv->pwr_data.i_bat;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		stm32bb_get_measurement(spi);
		val->intval = (int)priv->pwr_data.v_bat;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = (int)13600;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = (int)11700;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property stm32bb_bat_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW
};

static struct power_supply stm32bb_bat = {
	.name = "stm32bb-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = stm32bb_bat_props,
	.num_properties = ARRAY_SIZE(stm32bb_bat_props),
	.get_property = stm32bb_bat_get_prop,
};

/*
 * sysfs stuff
 */
static ssize_t
show_chgcur(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stm32bb_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%u %u\n", priv->pwr_iset.i_ac, priv->pwr_iset.i_bat);
}

static ssize_t set_chgcur(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct stm32bb_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	unsigned i_ac;
	unsigned i_bat;
	int ret;

	ret = sscanf(buf, "%u %u\n", &i_ac, &i_bat);
	if (ret != 2)
		return -EINVAL;

	priv->pwr_iset.i_ac = i_ac & 0x3FFF;
	priv->pwr_iset.i_bat = i_bat & 0x3FFF;

	stm32bb_set_current(spi);

	return count;
}

static DEVICE_ATTR(charge_current, S_IRUGO | S_IWUSR, show_chgcur, set_chgcur);

static ssize_t
show_pwrctrl(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stm32bb_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	uint16_t reg;

	reg = stm32bb_read_reg(spi, PWR_CTRL, 2);

	return sprintf(buf, "%u\n", reg & 0x0f);
}

static ssize_t set_pwrctrl(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct stm32bb_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	unsigned reg;
	int ret;

	ret = sscanf(buf, "%u\n", &reg);
	if (ret != 1)
		return -EINVAL;

	reg &= 0x000f;
	stm32bb_write_reg(spi, PWR_CTRL, reg, 2);

	return count;
}

static DEVICE_ATTR(pwr_control, S_IRUGO | S_IWUSR, show_pwrctrl, set_pwrctrl);

static ssize_t
show_pwrstatus(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stm32bb_priv *priv = dev_get_drvdata(dev);
	struct spi_device *spi = priv->spi;
	uint16_t reg;

	reg = stm32bb_read_reg(spi, PWR_STATUS, 2);

	return sprintf(buf, "%u\n", reg & 0x03);
}

static DEVICE_ATTR(pwr_status, S_IRUGO, show_pwrstatus, NULL);

static struct attribute *stm32bb_sysfs_entries[] = {
	&dev_attr_charge_current.attr,
	&dev_attr_pwr_control.attr,
	&dev_attr_pwr_status.attr,
	NULL,
};

static struct attribute_group stm32bb_attr_group = {
	.name	= "power-control",			/* put in device directory */
	.attrs	= stm32bb_sysfs_entries,
};


/*
 * module functions 
 */
static int __devinit stm32bb_can_probe(struct spi_device *spi)
{
	struct net_device *net;
	struct stm32bb_priv *priv;
	struct stm32bb_platform_data *pdata = spi->dev.platform_data;
	int ret = -ENODEV;

	if (!pdata)
		/* Platform data is required for osc freq */
		goto error_out;

	/* Allocate can/net device */
	net = alloc_candev(sizeof(struct stm32bb_priv), TX_ECHO_SKB_MAX);
	if (!net) {
		ret = -ENOMEM;
		goto error_alloc;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
	net->netdev_ops = &stm32bb_netdev_ops;
#else
	net->open = stm32bb_open;
	net->stop = stm32bb_stop;
	net->hard_start_xmit = stm32bb_hard_start_xmit;
#endif
	net->flags |= IFF_ECHO;

	priv = netdev_priv(net);
	priv->can.bittiming_const = &stm32bb_bittiming_const;
	priv->can.do_set_mode = stm32bb_do_set_mode;
	priv->can.clock.freq = 36000000;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK | 
		CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_ONE_SHOT;
	priv->net = net;
	dev_set_drvdata(&spi->dev, priv);

	priv->spi = spi;
	mutex_init(&priv->spi_lock);

	/* If requested, allocate DMA buffers */
	if (stm32bb_enable_dma) {
		spi->dev.coherent_dma_mask = ~0;

		/*
		 * Minimum coherent DMA allocation is PAGE_SIZE, so allocate
		 * that much and share it between Tx and Rx DMA buffers.
		 */
		priv->spi_tx_buf = dma_alloc_coherent(&spi->dev,
						      PAGE_SIZE,
						      &priv->spi_tx_dma,
						      GFP_DMA);

		if (priv->spi_tx_buf) {
			priv->spi_rx_buf = (u8 *)(priv->spi_tx_buf +
						  (PAGE_SIZE / 2));
			priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
							(PAGE_SIZE / 2));
		} else {
			/* Fall back to non-DMA */
			stm32bb_enable_dma = 0;
		}
	}

	/* Allocate non-DMA buffers */
	if (!stm32bb_enable_dma) {
		priv->spi_tx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			ret = -ENOMEM;
			goto error_tx_buf;
		}
		priv->spi_rx_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
		if (!priv->spi_rx_buf) {
			ret = -ENOMEM;
			goto error_rx_buf;
		}
	}

	/* Call out to platform specific setup */
	if (pdata->board_specific_setup)
		pdata->board_specific_setup(spi);

	SET_NETDEV_DEV(net, &spi->dev);

	priv->wq = create_freezeable_workqueue("stm32bb_wq");

	INIT_WORK(&priv->tx_work, stm32bb_tx_work_handler);
	INIT_WORK(&priv->irq_work, stm32bb_irq_work_handler);
	init_waitqueue_head(&priv->spi_wait);
	atomic_set(&priv->data_ready, 0);

	/* Configure the SPI bus */
	spi->mode = SPI_MODE_0;
	if (stm32bb_spi_speed != 0) {
		spi->max_speed_hz = stm32bb_spi_speed * 1000;
	}

	dev_info(&spi->dev, "SPI speed = %d Hz\n", spi->max_speed_hz);
	spi->bits_per_word = 8;
	spi_setup(spi);

	ret = request_irq(pdata->transfer_ready, stm32bb_transfer_ready_isr, IRQF_TRIGGER_FALLING,
		DEVICE_NAME "_dr", spi);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d\n", pdata->transfer_ready);
		goto trans_error_probe;
	}

	ret = stm32bb_hw_probe(spi);
	if (ret) {
		dev_info(&spi->dev, "Probe failed\n");
		goto error_probe;
	}

	/* Register ISR */
	ret = request_irq(spi->irq, stm32bb_can_isr, IRQF_TRIGGER_FALLING,
		DEVICE_NAME "_irq", net);
	if (ret) {
		dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
		goto error_probe;
	}

	stm32bb_pwr_setup(priv);
	/* sync with hw */
	priv->pwr_iset.i_bat = 0;
	priv->pwr_iset.i_ac = 0;

	ret = power_supply_register(&spi->dev, &stm32bb_ac);
	if (ret)
		goto ac_failed;

	ret = power_supply_register(&spi->dev, &stm32bb_sys);
	if (ret)
		goto sys_failed;

	ret = power_supply_register(&spi->dev, &stm32bb_bat);
	if (ret)
		goto battery_failed;

	ret = sysfs_create_group(&spi->dev.kobj, &stm32bb_attr_group);
	if (ret) {
		dev_err(&spi->dev, "failed to create sysfs entries\n");
		goto sysfs_failed;
	}

	ret = register_candev(net);
	if (!ret) {
		dev_info(&spi->dev, "probed\n");
		return ret;
	}
	sysfs_remove_group(&spi->dev.kobj, &stm32bb_attr_group);
sysfs_failed:
	power_supply_unregister(&stm32bb_bat);
battery_failed:
	power_supply_unregister(&stm32bb_sys);
sys_failed:
	power_supply_unregister(&stm32bb_ac);
ac_failed:
	free_irq(spi->irq, net);
error_probe:
	free_irq(pdata->transfer_ready, spi);
trans_error_probe:
	if (!stm32bb_enable_dma)
		kfree(priv->spi_rx_buf);
error_rx_buf:
	if (!stm32bb_enable_dma)
		kfree(priv->spi_tx_buf);
error_tx_buf:
	free_candev(net);
	if (stm32bb_enable_dma)
		dma_free_coherent(&spi->dev, PAGE_SIZE,
				  priv->spi_tx_buf, priv->spi_tx_dma);
error_alloc:
	dev_err(&spi->dev, "probe failed\n");
error_out:
	return ret;
}

static int __devexit stm32bb_can_remove(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	struct stm32bb_platform_data *pdata = spi->dev.platform_data;
	struct net_device *net = priv->net;

	unregister_candev(net);
	free_candev(net);

	sysfs_remove_group(&spi->dev.kobj, &stm32bb_attr_group);
	power_supply_unregister(&stm32bb_ac);
	power_supply_unregister(&stm32bb_sys);
	power_supply_unregister(&stm32bb_bat);
	free_irq(pdata->transfer_ready, spi);
	free_irq(spi->irq, net);

	priv->force_quit = 1;
	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	if (stm32bb_enable_dma) {
		dma_free_coherent(&spi->dev, PAGE_SIZE,
				  priv->spi_tx_buf, priv->spi_tx_dma);
	} else {
		kfree(priv->spi_tx_buf);
		kfree(priv->spi_rx_buf);
	}

	return 0;
}

#ifdef CONFIG_PM
static int stm32bb_can_suspend(struct spi_device *spi, pm_message_t state)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	struct net_device *net = priv->net;

	if (netif_running(net)) {
		netif_device_detach(net);
		
		stm32bb_can_sleep(spi);
		priv->after_suspend = AFTER_SUSPEND_UP;
	} else {
		priv->after_suspend = AFTER_SUSPEND_DOWN;
	}

	return 0;
}

static int stm32bb_can_resume(struct spi_device *spi)
{
	struct stm32bb_priv *priv = dev_get_drvdata(&spi->dev);
	struct net_device *net = priv->net;

	if (priv->after_suspend & AFTER_SUSPEND_UP) {
		netif_device_attach(net);
		queue_work(priv->wq, &priv->irq_work);
	} else {
			priv->after_suspend = 0;
	}

	return 0;
}

#else
#define stm32bb_can_suspend NULL
#define stm32bb_can_resume NULL
#endif

static struct spi_driver stm32bb_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},

	.probe = stm32bb_can_probe,
	.remove = __devexit_p(stm32bb_can_remove),
	.suspend = stm32bb_can_suspend,
	.resume = stm32bb_can_resume,
};

static int __init stm32bb_can_init(void)
{
	return spi_register_driver(&stm32bb_can_driver);
}

static void __exit stm32bb_can_exit(void)
{
	spi_unregister_driver(&stm32bb_can_driver);
}

module_init(stm32bb_can_init);
module_exit(stm32bb_can_exit);

MODULE_AUTHOR("Pavol Jusko, "
	      "Michal Demin");
MODULE_DESCRIPTION("STM32-based CAN Controller");
MODULE_LICENSE("GPL v2");
