/*
 * Microchip ENC424J600 ethernet driver (MAC + PHY)
 *
 * Author: Kuba Marek <blue.cube@seznam.cz>
 * based on enc424j600.c written by Claudio Lanconelli
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#include "enc424j600_hw.h"

#define DRV_NAME	"enc424j600"
#define DRV_VERSION	"1.00"

#define ENC424J600_MSG_DEFAULT	\
	(NETIF_MSG_PROBE | NETIF_MSG_IFUP | NETIF_MSG_IFDOWN | NETIF_MSG_LINK)

#define SPI_TRANSFER_BUF_LEN	(4 + MAX_FRAMELEN)
#define TX_TIMEOUT	(4 * HZ)

/* Max TX retries in case of collision as suggested by errata datasheet */
#define MAX_TX_RETRYCOUNT	16

static int enc424j600_enable_dma; /* Enable SPI DMA. Default: 0 (Off) */

enum {
	RXFILTER_NORMAL,
	RXFILTER_MULTI,
	RXFILTER_PROMISC
};

/* Driver local data */
struct enc424j600_net {
	struct net_device *netdev;
	struct spi_device *spi;
	struct mutex lock;
	struct sk_buff *tx_skb;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct work_struct setrx_work;
	struct work_struct restart_work;
	u8 bank;		/* current register bank selected */
	u16 next_pk_ptr;	/* next packet pointer within FIFO */
	u16 max_pk_counter;	/* statistics: max packet counter */
	u16 tx_retry_count;
	bool hw_enable;
	bool full_duplex;
	bool autoneg;
	bool speed100;
	int rxfilter;
	u32 msg_enable;

	u8 *spi_rx_buf;
	u8 *spi_tx_buf;
	dma_addr_t spi_tx_dma;
	dma_addr_t spi_rx_dma;
};

/* use ethtool to change the level for any given device */
static struct {
	u32 msg_enable;
} debug = { -1 };

/*
 * SPI read buffer
 * wait for the SPI transfer and copy received data to destination
 */
static int
spi_read_buf(struct enc424j600_net *priv, int len, u8 *data)
{
#if 0
	u8 *rx_buf = priv->spi_tx_buf + 4;
	u8 *tx_buf = priv->spi_tx_buf;
	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = SPI_OPLEN + len,
	};
	struct spi_message msg;
	int ret;

	tx_buf[0] = ENC28J60_READ_BUF_MEM;
	tx_buf[1] = tx_buf[2] = tx_buf[3] = 0;	/* don't care */

	spi_message_init(&msg);
	spi_message_add_tail(&t, &msg);
	ret = spi_sync(priv->spi, &msg);
	if (ret == 0) {
		memcpy(data, &rx_buf[SPI_OPLEN], len);
		ret = msg.status;
	}
	if (ret && netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
			__func__, ret);

	return ret;
#endif
	return 0;
}

/*
 * SPI write buffer
 */
static int spi_write_buf(struct enc424j600_net *priv, int len,
			 const u8 *data)
{
#if 0
	int ret;

	if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0)
		ret = -EINVAL;
	else {
		priv->spi_tx_buf[0] = ENC28J60_WRITE_BUF_MEM;
		memcpy(&priv->spi_tx_buf[1], data, len);
		ret = spi_write(priv->spi, priv->spi_tx_buf, len + 1);
		if (ret && netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": %s() failed: ret = %d\n",
				__func__, ret);
	}
	return ret;
#endif
	return 0;
}

static int enc424j600_spi_trans(struct enc424j600_net *priv, int len)
{
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.len = len,
		.cs_change = 0,
	};
	struct spi_message m;
	int ret;

	spi_message_init(&m);

	if (enc424j600_enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(priv->spi, &m);
	if (ret)
		dev_err(&priv->spi->dev, "spi transfer failed: ret = %d\n", ret);
	return ret;
	return 0;
}

/*
 * Select the current register bank if necessary to be able to read @addr.
 */
static void enc424j600_set_bank(struct enc424j600_net *priv, u8 addr)
{
	u8 b = (addr & BANK_MASK) >> BANK_SHIFT;

	/* These registers are present in all banks, no need to switch bank */
	if (addr >= EUDASTL && addr <= ECON1H)
		return;
	if(priv->bank == b)
		return;

	priv->spi_tx_buf[0] = BXSEL(b);
	enc424j600_spi_trans(priv, 1);

	priv->bank = b;
}

/*
 * Set bits in an 8bit SFR.
 */
static void enc424j600_set_bits(struct enc424j600_net *priv, u8 addr, u8 mask)
{
	enc424j600_set_bank(priv, addr);
	priv->spi_tx_buf[0] = BFS(addr);
	priv->spi_tx_buf[1] = mask;
	enc424j600_spi_trans(priv, 2);
}

/*
 * Clear bits in an 8bit SFR.
 */
static void enc424j600_clear_bits(struct enc424j600_net *priv, u8 addr, u8 mask)
{
	enc424j600_set_bank(priv, addr);
	priv->spi_tx_buf[0] = BFC(addr);
	priv->spi_tx_buf[1] = mask;
	enc424j600_spi_trans(priv, 2);
}

/*
 * Write a 8bit special function register.
 * The @sfr parameters takes address of the register.
 * Uses banked write instruction.
 */
static int enc424j600_write_8b_sfr(struct enc424j600_net *priv, u8 sfr, u8 data)
{
	int ret;

	enc424j600_set_bank(priv, sfr);

	priv->spi_tx_buf[0] = WCR(sfr & ADDR_MASK);
	priv->spi_tx_buf[1] = data & 0xFF;
	ret = enc424j600_spi_trans(priv, 2);

	return ret;
}
/*
 * Read a 8bit special function register.
 * The @sfr parameters takes address of the register.
 * Uses banked read instruction.
 */
static int enc424j600_read_8b_sfr(struct enc424j600_net *priv, u8 sfr, u8 *data)
{
	int ret;

	enc424j600_set_bank(priv, sfr);
	priv->spi_tx_buf[0] = RCR(sfr & ADDR_MASK);
	ret = enc424j600_spi_trans(priv, 2);
	*data = priv->spi_rx_buf[1];

	return ret;
}

/*
 * Write a 16bit special function register.
 * The @sfr parameters takes address of the low byte of the register.
 * Takes care of the endiannes & buffers.
 * Uses banked write instruction.
 */
static int enc424j600_write_16b_sfr(struct enc424j600_net *priv, u8 sfr, u16 data)
{
	int ret;

	enc424j600_set_bank(priv, sfr);

	priv->spi_tx_buf[0] = WCR(sfr & ADDR_MASK);
	priv->spi_tx_buf[1] = data & 0xFF;
	priv->spi_tx_buf[2] = data >> 8;
	ret = enc424j600_spi_trans(priv, 3);

	return ret;
}

/*
 * Read a 16bit special function register.
 * The @sfr parameters takes address of the low byte of the register.
 * Takes care of the endiannes & buffers.
 * Uses banked read instruction.
 */
static int enc424j600_read_16b_sfr(struct enc424j600_net *priv, u8 sfr, u16 *data)
{
	int ret;

	enc424j600_set_bank(priv, sfr);

	priv->spi_tx_buf[0] = RCR(sfr & ADDR_MASK);
	ret = enc424j600_spi_trans(priv, 3);
	*data = priv->spi_rx_buf[1] |
		priv->spi_rx_buf[2] << (u16)8;

	return ret;
}

/*
 * Reset the enc424j600.
 * TODO: What if we get stuck on non-working spi with the initial
 * test access to EUDAST ?
 * TODO: Errors?
 */
static void enc424j600_soft_reset(struct enc424j600_net *priv)
{
	u8 estath;
	u16 eudast;

	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	do{
		enc424j600_write_16b_sfr(priv, EUDASTL, EUDAST_TEST_VAL);
		enc424j600_read_16b_sfr(priv, EUDASTL, &eudast);
	} while (eudast != EUDAST_TEST_VAL);

	do{
		enc424j600_read_8b_sfr(priv, ESTATH, &estath);
	} while (!estath & CLKRDY);

	priv->spi_tx_buf[0] = SETETHRST;
	enc424j600_spi_trans(priv, 1);

	udelay(50);

	enc424j600_read_16b_sfr(priv, EUDASTL, &eudast);
	if (netif_msg_hw(priv) && eudast != 0)
		printk(KERN_DEBUG DRV_NAME ": %s() EUDASTL is not zero!\n", __func__);

	udelay(500);
}

/*
 * Buffer memory read
 * Select the starting address and execute a SPI buffer read
 */
static void enc424j600_mem_read(struct enc424j600_net *priv,
				     u16 addr, int len, u8 *data)
{
#if 0
	mutex_lock(&priv->lock);
	nolock_regw_write(priv, ERDPTL, addr);
#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u16 reg;
		reg = nolock_regw_read(priv, ERDPTL);
		if (reg != addr)
			printk(KERN_DEBUG DRV_NAME ": %s() error writing ERDPT "
				"(0x%04x - 0x%04x)\n", __func__, reg, addr);
	}
#endif
	spi_read_buf(priv, len, data);
	mutex_unlock(&priv->lock);
#endif
}

/*
 * Write packet to enc424j600 TX buffer memory
 */
static void
enc424j600_packet_write(struct enc424j600_net *priv, int len, const u8 *data)
{
#if 0
	mutex_lock(&priv->lock);
	/* Set the write pointer to start of transmit buffer area */
	nolock_regw_write(priv, EWRPTL, TXSTART_INIT);
#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u16 reg;
		reg = nolock_regw_read(priv, EWRPTL);
		if (reg != TXSTART_INIT)
			printk(KERN_DEBUG DRV_NAME
				": %s() ERWPT:0x%04x != 0x%04x\n",
				__func__, reg, TXSTART_INIT);
	}
#endif
	/* Set the TXND pointer to correspond to the packet size given */
	nolock_regw_write(priv, ETXNDL, TXSTART_INIT + len);
	/* write per-packet control byte */
	spi_write_op(priv, ENC28J60_WRITE_BUF_MEM, 0, 0x00);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME
			": %s() after control byte ERWPT:0x%04x\n",
			__func__, nolock_regw_read(priv, EWRPTL));
	/* copy the packet into the transmit buffer */
	spi_write_buf(priv, len, data);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME
			 ": %s() after write packet ERWPT:0x%04x, len=%d\n",
			 __func__, nolock_regw_read(priv, EWRPTL), len);
	mutex_unlock(&priv->lock);
#endif
}

static unsigned long msec20_to_jiffies;

/*
 * Wait for bits in register to become equal to @readyMask, but at most 20ms.
 */
static int poll_ready(struct enc424j600_net *priv, u8 reg, u8 mask, u8 readyMask)
{
	unsigned long timeout = jiffies + msec20_to_jiffies;
	u8 value;

	/* 20 msec timeout read */
	enc424j600_read_8b_sfr(priv, reg, &value);
	while ((value & mask) != readyMask) {
		if (time_after(jiffies, timeout)) {
			if (netif_msg_drv(priv))
				dev_dbg(&priv->spi->dev,
					"reg %02x ready timeout!\n", reg);
			return -ETIMEDOUT;
		}
		cpu_relax();
		enc424j600_read_8b_sfr(priv, reg, &value);
	}

	return 0;
}

/*
 * PHY register read
 * PHY registers are not accessed directly, but through the MII
 */
static int enc424j600_phy_read(struct enc424j600_net *priv, u16 address, u16 *data)
{
	int ret;
	
	enc424j600_write_16b_sfr(priv, MIREGADRL, address | (MIREGADRH_VAL << 8));
	enc424j600_write_16b_sfr(priv, MICMDL, MIIRD);
	udelay(26);
	ret = !poll_ready(priv, MISTATL, BUSY, 0);
	enc424j600_write_16b_sfr(priv, MICMDL, 0);
	enc424j600_read_16b_sfr(priv, MIRDL, data);
	return ret;
}

static int enc424j600_phy_write(struct enc424j600_net *priv, u16 address, u16 data)
{
	enc424j600_write_16b_sfr(priv, MIREGADRL, address | (MIREGADRH_VAL << 8));
	enc424j600_write_16b_sfr(priv, MIWRL, data);
	udelay(26);
	return !poll_ready(priv, MISTATL, BUSY, 0);
}

/*
 * Read the hardware MAC address to dev->dev_addr.
 */
static int enc424j600_get_hw_macaddr(struct net_device *ndev)
{
	struct enc424j600_net *priv = netdev_priv(ndev);
	u16 maadr1;
	u16 maadr2;
	u16 maadr3;

	mutex_lock(&priv->lock);

	if (netif_msg_drv(priv))
		printk(KERN_INFO DRV_NAME
			": %s: Setting MAC address to %pM\n",
			ndev->name, ndev->dev_addr);

	enc424j600_read_16b_sfr(priv, MAADR3L, &maadr3);
	ndev->dev_addr[0] = maadr3 >> 8;
	ndev->dev_addr[1] = maadr3 & 0xff;;
	enc424j600_read_16b_sfr(priv, MAADR2L, &maadr2);
	ndev->dev_addr[2] = maadr2 >> 8;
	ndev->dev_addr[3] = maadr2 & 0xff;;
	enc424j600_read_16b_sfr(priv, MAADR1L, &maadr1);
	ndev->dev_addr[4] = maadr1 >> 8;
	ndev->dev_addr[5] = maadr1 & 0xff;;

	mutex_unlock(&priv->lock);

	return 0;
}

/*
 * Program the hardware MAC address from dev->dev_addr.
 */
static int enc424j600_set_hw_macaddr(struct net_device *ndev)
{
	struct enc424j600_net *priv = netdev_priv(ndev);


	mutex_lock(&priv->lock);

	if (priv->hw_enable) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME
				": %s() Hardware must be disabled to set "
				"Mac address\n", __func__);
		mutex_unlock(&priv->lock);
		return -EBUSY;
	}

	if (netif_msg_drv(priv))
		printk(KERN_INFO DRV_NAME
			": %s: Setting MAC address to %pM\n",
			ndev->name, ndev->dev_addr);

	enc424j600_write_16b_sfr(priv, MAADR3L, ndev->dev_addr[1] | ndev->dev_addr[0] << 8);
	enc424j600_write_16b_sfr(priv, MAADR2L, ndev->dev_addr[3] | ndev->dev_addr[2] << 8);
	enc424j600_write_16b_sfr(priv, MAADR1L, ndev->dev_addr[5] | ndev->dev_addr[4] << 8);

	mutex_unlock(&priv->lock);

	return 0;
}

/*
 * Store the new hardware address in dev->dev_addr, and update the MAC.
 */
static int enc424j600_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (netif_running(dev))
		return -EBUSY;
	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	return enc424j600_set_hw_macaddr(dev);
}

/*
 * Debug routine to dump useful register contents
 */
static void enc424j600_dump_regs(struct enc424j600_net *priv, const char *msg)
{
#if 0
	mutex_lock(&priv->lock);
	printk(KERN_DEBUG DRV_NAME " %s\n"
		"HwRevID: 0x%02x\n"
		"Cntrl: ECON1 ECON2 ESTAT  EIR  EIE\n"
		"       0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n"
		"MAC  : MACON1 MACON3 MACON4\n"
		"       0x%02x   0x%02x   0x%02x\n"
		"Rx   : ERXST  ERXND  ERXWRPT ERXRDPT ERXFCON EPKTCNT MAMXFL\n"
		"       0x%04x 0x%04x 0x%04x  0x%04x  "
		"0x%02x    0x%02x    0x%04x\n"
		"Tx   : ETXST  ETXND  MACLCON1 MACLCON2 MAPHSUP\n"
		"       0x%04x 0x%04x 0x%02x     0x%02x     0x%02x\n",
		msg, nolock_regb_read(priv, EREVID),
		nolock_regb_read(priv, ECON1), nolock_regb_read(priv, ECON2),
		nolock_regb_read(priv, ESTAT), nolock_regb_read(priv, EIR),
		nolock_regb_read(priv, EIE), nolock_regb_read(priv, MACON1),
		nolock_regb_read(priv, MACON3), nolock_regb_read(priv, MACON4),
		nolock_regw_read(priv, ERXSTL), nolock_regw_read(priv, ERXNDL),
		nolock_regw_read(priv, ERXWRPTL),
		nolock_regw_read(priv, ERXRDPTL),
		nolock_regb_read(priv, ERXFCON),
		nolock_regb_read(priv, EPKTCNT),
		nolock_regw_read(priv, MAMXFLL), nolock_regw_read(priv, ETXSTL),
		nolock_regw_read(priv, ETXNDL),
		nolock_regb_read(priv, MACLCON1),
		nolock_regb_read(priv, MACLCON2),
		nolock_regb_read(priv, MAPHSUP));
	mutex_unlock(&priv->lock);
#endif
}

/*
 * Calculate wrap around when reading beyond the end of the RX buffer
 */
static u16 rx_packet_start(u16 ptr)
{
#if 0
	if (ptr + RSV_SIZE > RXEND_INIT)
		return (ptr + RSV_SIZE) - (RXEND_INIT - RXSTART_INIT + 1);
	else
		return ptr + RSV_SIZE;
#endif
	return 0;
}

static void nolock_rxfifo_init(struct enc424j600_net *priv, u16 size)
{
#if 0
	u16 erxrdpt;

	if (start > 0x1FFF || end > 0x1FFF || start > end) {
		if (netif_msg_drv(priv))
			printk(KERN_ERR DRV_NAME ": %s(%d, %d) RXFIFO "
				"bad parameters!\n", __func__, start, end);
		return;
	}
	/* set receive buffer start + end */
	priv->next_pk_ptr = start;
	nolock_regw_write(priv, ERXSTL, start);
	erxrdpt = erxrdpt_workaround(priv->next_pk_ptr, start, end);
	nolock_regw_write(priv, ERXRDPTL, erxrdpt);
	nolock_regw_write(priv, ERXNDL, end);
#endif
}

static void nolock_txfifo_init(struct enc424j600_net *priv)
{
#if 0
	if (start > 0x1FFF || end > 0x1FFF || start > end) {
		if (netif_msg_drv(priv))
			printk(KERN_ERR DRV_NAME ": %s(%d, %d) TXFIFO "
				"bad parameters!\n", __func__, start, end);
		return;
	}
	/* set transmit buffer start + end */
	nolock_regw_write(priv, ETXSTL, start);
	nolock_regw_write(priv, ETXNDL, end);
#endif
}

/*
 * Low power mode shrinks power consumption about 100x, so we'd like
 * the chip to be in that mode whenever it's inactive.  (However, we
 * can't stay in lowpower mode during suspend with WOL active.)
 */
static void enc424j600_lowpower(struct enc424j600_net *priv, bool is_low)
{
	if (netif_msg_drv(priv))
		dev_dbg(&priv->spi->dev, "%s power...\n",
				is_low ? "low" : "high");

#if 0
	mutex_lock(&priv->lock);
	if (is_low) {
		nolock_reg_bfclr(priv, ECON1, ECON1_RXEN);
		poll_ready(priv, ESTAT, ESTAT_RXBUSY, 0);
		poll_ready(priv, ECON1, ECON1_TXRTS, 0);
		/* ECON2_VRPS was set during initialization */
		nolock_reg_bfset(priv, ECON2, ECON2_PWRSV);
	} else {
		nolock_reg_bfclr(priv, ECON2, ECON2_PWRSV);
		poll_ready(priv, ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY);
		/* caller sets ECON1_RXEN */
	}
	mutex_unlock(&priv->lock);
#endif
}


/*
 * Reset and initialize the chip, but don't enable interrupts and don't
 * start receiving yet.
 */
static int enc424j600_hw_init(struct enc424j600_net *priv)
{
	u8 eidledl;
	u16 phcon1;
	u16 macon2;

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() - %s\n", __func__,
			priv->autoneg ? "Autoneg" : (priv->full_duplex ? "FullDuplex" : "HalfDuplex"));

	mutex_lock(&priv->lock);

	priv->bank = 0;
	priv->hw_enable = false;
	priv->tx_retry_count = 0;
	priv->max_pk_counter = 0;
	priv->rxfilter = RXFILTER_NORMAL;

	enc424j600_soft_reset(priv);

	/*
	 * Check the device id and silicon revision id.
	 */
	enc424j600_read_8b_sfr(priv, EIDLEDL, &eidledl);

	if((eidledl & DEVID_MASK) >> DEVID_SHIFT != ENC424J600_DEV_ID){
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": %s() Invalid device ID: %d\n",
				__func__, (eidledl & DEVID_MASK) >> DEVID_SHIFT);
		return 0;
	}

	if (netif_msg_drv(priv))
		printk(KERN_INFO DRV_NAME ": Silicon revision ID: 0x%02x\n",
			(eidledl & REVID_MASK) >> REVID_SHIFT);
	

	/* program ERXST (rx buffer size) */
	enc424j600_write_16b_sfr(priv, ERXSTL,
		ENC424J600_SRAM_END - RX_BUFFER_SIZE + 1);
	
	/* default filter mode: (unicast OR broadcast) AND crc valid */
	enc424j600_write_16b_sfr(priv, ERXFCONL, UCEN | BCEN | CRCEN);
		
	/* PHANA */
	enc424j600_phy_write(priv, PHANA, PHANA_DEFAULT);
	
	/* PHCON1 */
	phcon1 = 0;
	if (priv->autoneg){
		/* Enable autonegotiation and renegotiate */
		phcon1 |= ANEN | RENEG;
	} else {
		if(priv->speed100)
			phcon1 |= SPD100;
		if(priv->full_duplex)
			phcon1 |= PFULDPX;
	}
	enc424j600_phy_write(priv, PHCON1, phcon1);

	/* MACON2
	 * defer transmission if collision occurs (only for half duplex)
	 * pad to 60 or 64 bytes and append CRC
	 * enable receiving huge frames (instead of limiting packet size) */
	macon2 = MACON2_DEFER | PADCFG2 | PADCFG0 | TXCRCEN | HFRMEN;

	/* If autonegotiation is enabled, we have to wait untill it finishes
	 * and set the PHYDPX bit in MACON2 correctly */
	if (priv->autoneg) {
		u16 phstat1;
		u8 estath;

		do {
			enc424j600_phy_read(priv, PHSTAT1, &phstat1);
		} while(!(phstat1 & ANDONE));

		/* read the PHYDPX bit in ESTAT and set FULDPX in MACON2 accordingly */
		enc424j600_read_8b_sfr(priv, ESTATH, &estath);
		if (estath & PHYDPX) {
			macon2 |= FULDPX;
		}
	} else if (priv->full_duplex) {
		macon2 |= FULDPX;
	}
	enc424j600_write_16b_sfr(priv, MACON2L, macon2);

	/* MAIPGL
	 * Recomended values for inter packet gaps */
	if (!priv->autoneg){
		enc424j600_write_16b_sfr(priv, MAIPGL, MAIPGL_VAL | MAIPGH_VAL << 8);
	}


	/* LED settings */
	enc424j600_write_8b_sfr(priv, EIDLEDH,
		(LED_A_SETTINGS << 4 | LED_B_SETTINGS));
	
	/* 
	 * Select enabled interrupts, but don't set the global
	 * interrupt enable flag.
	 */
	enc424j600_write_16b_sfr(priv, EIEL,
		LINKIE << 8 | PKTIE | DMAIE | TXIE |
		TXABTIE | RXABTIE);

	nolock_rxfifo_init(priv, RX_BUFFER_SIZE);
	nolock_txfifo_init(priv);

	mutex_unlock(&priv->lock);

	if (netif_msg_hw(priv))
		enc424j600_dump_regs(priv, "Hw initialized.");

	return 1;
}

static void enc424j600_hw_enable(struct enc424j600_net *priv)
{
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enabling interrupts.\n",
			__func__);

	mutex_lock(&priv->lock);

	/* Clear any pending interrupts */
	enc424j600_write_16b_sfr(priv, EIRL, 0);

	/* Enable global interrupt flag */
	enc424j600_set_bits(priv, EIEH, INTIE);

	/* enable receive logic */
	enc424j600_set_bits(priv, ECON1L, RXEN);
	priv->hw_enable = true;
	mutex_unlock(&priv->lock);
}

static void enc424j600_hw_disable(struct enc424j600_net *priv)
{
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() disabling interrupts.\n",
			__func__);

	mutex_lock(&priv->lock);

	/* disable receive logic */
	enc424j600_clear_bits(priv, ECON1L, RXEN);

	/* Disable global interrupt flag */
	enc424j600_clear_bits(priv, EIEH, INTIE);

	priv->hw_enable = false;

	mutex_unlock(&priv->lock);
}

static int
enc424j600_setlink(struct net_device *ndev, u8 autoneg, u16 speed, u8 duplex)
{
	struct enc424j600_net *priv = netdev_priv(ndev);
	int ret = 0;

	if (!priv->hw_enable) {
		/* link is in low power mode now; duplex setting
		 * will take effect on next enc424j600_hw_init().
		 */
		if (speed == SPEED_10 || speed == SPEED_100) {
			priv->autoneg = (autoneg == AUTONEG_ENABLE);
			priv->full_duplex = (duplex == DUPLEX_FULL);
			priv->speed100 = (speed == SPEED_100);
		} else {
			if (netif_msg_link(priv))
				dev_warn(&ndev->dev,
					"unsupported link setting\n");
			ret = -EOPNOTSUPP;
		}
	} else {
		if (netif_msg_link(priv))
			dev_warn(&ndev->dev, "Warning: hw must be disabled "
				"to set link mode\n");
		ret = -EBUSY;
	}
	return ret;
}

/*
 * Read the Transmit Status Vector
 */
static void enc424j600_read_tsv(struct enc424j600_net *priv, u8 tsv[TSV_SIZE])
{
#if 0
	int endptr;

	endptr = locked_regw_read(priv, ETXNDL);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": reading TSV at addr:0x%04x\n",
			 endptr + 1);
	enc424j600_mem_read(priv, endptr + 1, sizeof(tsv), tsv);
#endif
}

static void enc424j600_dump_tsv(struct enc424j600_net *priv, const char *msg,
				u8 tsv[TSV_SIZE])
{
#if 0
	u16 tmp1, tmp2;

	printk(KERN_DEBUG DRV_NAME ": %s - TSV:\n", msg);
	tmp1 = tsv[1];
	tmp1 <<= 8;
	tmp1 |= tsv[0];

	tmp2 = tsv[5];
	tmp2 <<= 8;
	tmp2 |= tsv[4];

	printk(KERN_DEBUG DRV_NAME ": ByteCount: %d, CollisionCount: %d,"
		" TotByteOnWire: %d\n", tmp1, tsv[2] & 0x0f, tmp2);
	printk(KERN_DEBUG DRV_NAME ": TxDone: %d, CRCErr:%d, LenChkErr: %d,"
		" LenOutOfRange: %d\n", TSV_GETBIT(tsv, TSV_TXDONE),
		TSV_GETBIT(tsv, TSV_TXCRCERROR),
		TSV_GETBIT(tsv, TSV_TXLENCHKERROR),
		TSV_GETBIT(tsv, TSV_TXLENOUTOFRANGE));
	printk(KERN_DEBUG DRV_NAME ": Multicast: %d, Broadcast: %d, "
		"PacketDefer: %d, ExDefer: %d\n",
		TSV_GETBIT(tsv, TSV_TXMULTICAST),
		TSV_GETBIT(tsv, TSV_TXBROADCAST),
		TSV_GETBIT(tsv, TSV_TXPACKETDEFER),
		TSV_GETBIT(tsv, TSV_TXEXDEFER));
	printk(KERN_DEBUG DRV_NAME ": ExCollision: %d, LateCollision: %d, "
		 "Giant: %d, Underrun: %d\n",
		 TSV_GETBIT(tsv, TSV_TXEXCOLLISION),
		 TSV_GETBIT(tsv, TSV_TXLATECOLLISION),
		 TSV_GETBIT(tsv, TSV_TXGIANT), TSV_GETBIT(tsv, TSV_TXUNDERRUN));
	printk(KERN_DEBUG DRV_NAME ": ControlFrame: %d, PauseFrame: %d, "
		 "BackPressApp: %d, VLanTagFrame: %d\n",
		 TSV_GETBIT(tsv, TSV_TXCONTROLFRAME),
		 TSV_GETBIT(tsv, TSV_TXPAUSEFRAME),
		 TSV_GETBIT(tsv, TSV_BACKPRESSUREAPP),
		 TSV_GETBIT(tsv, TSV_TXVLANTAGFRAME));
#endif
}

/*
 * Receive Status vector
 */
static void enc424j600_dump_rsv(struct enc424j600_net *priv, const char *msg,
			      u16 pk_ptr, int len, u16 sts)
{
#if 0
	printk(KERN_DEBUG DRV_NAME ": %s - NextPk: 0x%04x - RSV:\n",
		msg, pk_ptr);
	printk(KERN_DEBUG DRV_NAME ": ByteCount: %d, DribbleNibble: %d\n", len,
		 RSV_GETBIT(sts, RSV_DRIBBLENIBBLE));
	printk(KERN_DEBUG DRV_NAME ": RxOK: %d, CRCErr:%d, LenChkErr: %d,"
		 " LenOutOfRange: %d\n", RSV_GETBIT(sts, RSV_RXOK),
		 RSV_GETBIT(sts, RSV_CRCERROR),
		 RSV_GETBIT(sts, RSV_LENCHECKERR),
		 RSV_GETBIT(sts, RSV_LENOUTOFRANGE));
	printk(KERN_DEBUG DRV_NAME ": Multicast: %d, Broadcast: %d, "
		 "LongDropEvent: %d, CarrierEvent: %d\n",
		 RSV_GETBIT(sts, RSV_RXMULTICAST),
		 RSV_GETBIT(sts, RSV_RXBROADCAST),
		 RSV_GETBIT(sts, RSV_RXLONGEVDROPEV),
		 RSV_GETBIT(sts, RSV_CARRIEREV));
	printk(KERN_DEBUG DRV_NAME ": ControlFrame: %d, PauseFrame: %d,"
		 " UnknownOp: %d, VLanTagFrame: %d\n",
		 RSV_GETBIT(sts, RSV_RXCONTROLFRAME),
		 RSV_GETBIT(sts, RSV_RXPAUSEFRAME),
		 RSV_GETBIT(sts, RSV_RXUNKNOWNOPCODE),
		 RSV_GETBIT(sts, RSV_RXTYPEVLAN));
#endif
}

static void dump_packet(const char *msg, int len, const char *data)
{
	printk(KERN_DEBUG DRV_NAME ": %s - packet len:%d\n", msg, len);
	print_hex_dump(KERN_DEBUG, "pk data: ", DUMP_PREFIX_OFFSET, 16, 1,
			data, len, true);
}

/*
 * Hardware receive function.
 * Read the buffer memory, update the FIFO pointer to free the buffer,
 * check the status vector and decrement the packet counter.
 */
static void enc424j600_hw_rx(struct net_device *ndev)
{
#if 0
	struct enc424j600_net *priv = netdev_priv(ndev);
	struct sk_buff *skb = NULL;
	u16 erxrdpt, next_packet, rxstat;
	u8 rsv[RSV_SIZE];
	int len;

	if (netif_msg_rx_status(priv))
		printk(KERN_DEBUG DRV_NAME ": RX pk_addr:0x%04x\n",
			priv->next_pk_ptr);

	if (unlikely(priv->next_pk_ptr > RXEND_INIT)) {
		if (netif_msg_rx_err(priv))
			dev_err(&ndev->dev,
				"%s() Invalid packet address!! 0x%04x\n",
				__func__, priv->next_pk_ptr);
		/* packet address corrupted: reset RX logic */
		mutex_lock(&priv->lock);
		nolock_reg_bfclr(priv, ECON1, ECON1_RXEN);
		nolock_reg_bfset(priv, ECON1, ECON1_RXRST);
		nolock_reg_bfclr(priv, ECON1, ECON1_RXRST);
		nolock_rxfifo_init(priv, RXSTART_INIT, RXEND_INIT);
		nolock_reg_bfclr(priv, EIR, EIR_RXERIF);
		nolock_reg_bfset(priv, ECON1, ECON1_RXEN);
		mutex_unlock(&priv->lock);
		ndev->stats.rx_errors++;
		return;
	}
	/* Read next packet pointer and rx status vector */
	enc424j600_mem_read(priv, priv->next_pk_ptr, sizeof(rsv), rsv);

	next_packet = rsv[1];
	next_packet <<= 8;
	next_packet |= rsv[0];

	len = rsv[3];
	len <<= 8;
	len |= rsv[2];

	rxstat = rsv[5];
	rxstat <<= 8;
	rxstat |= rsv[4];

	if (netif_msg_rx_status(priv))
		enc424j600_dump_rsv(priv, __func__, next_packet, len, rxstat);

	if (!RSV_GETBIT(rxstat, RSV_RXOK) || len > MAX_FRAMELEN) {
		if (netif_msg_rx_err(priv))
			dev_err(&ndev->dev, "Rx Error (%04x)\n", rxstat);
		ndev->stats.rx_errors++;
		if (RSV_GETBIT(rxstat, RSV_CRCERROR))
			ndev->stats.rx_crc_errors++;
		if (RSV_GETBIT(rxstat, RSV_LENCHECKERR))
			ndev->stats.rx_frame_errors++;
		if (len > MAX_FRAMELEN)
			ndev->stats.rx_over_errors++;
	} else {
		skb = dev_alloc_skb(len + NET_IP_ALIGN);
		if (!skb) {
			if (netif_msg_rx_err(priv))
				dev_err(&ndev->dev,
					"out of memory for Rx'd frame\n");
			ndev->stats.rx_dropped++;
		} else {
			skb->dev = ndev;
			skb_reserve(skb, NET_IP_ALIGN);
			/* copy the packet from the receive buffer */
			enc424j600_mem_read(priv,
				rx_packet_start(priv->next_pk_ptr),
				len, skb_put(skb, len));
			if (netif_msg_pktdata(priv))
				dump_packet(__func__, skb->len, skb->data);
			skb->protocol = eth_type_trans(skb, ndev);
			/* update statistics */
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += len;
			netif_rx_ni(skb);
		}
	}
	/*
	 * Move the RX read pointer to the start of the next
	 * received packet.
	 * This frees the memory we just read out
	 */
	erxrdpt = erxrdpt_workaround(next_packet, RXSTART_INIT, RXEND_INIT);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() ERXRDPT:0x%04x\n",
			__func__, erxrdpt);

	mutex_lock(&priv->lock);
	nolock_regw_write(priv, ERXRDPTL, erxrdpt);
#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u16 reg;
		reg = nolock_regw_read(priv, ERXRDPTL);
		if (reg != erxrdpt)
			printk(KERN_DEBUG DRV_NAME ": %s() ERXRDPT verify "
				"error (0x%04x - 0x%04x)\n", __func__,
				reg, erxrdpt);
	}
#endif
	priv->next_pk_ptr = next_packet;
	/* we are done with this packet, decrement the packet counter */
	nolock_reg_bfset(priv, ECON2, ECON2_PKTDEC);
	mutex_unlock(&priv->lock);
#endif
}

/*
 * Calculate free space in RxFIFO
 */
static int enc424j600_get_free_rxfifo(struct enc424j600_net *priv)
{
#if 0
	int epkcnt, erxst, erxnd, erxwr, erxrd;
	int free_space;

	mutex_lock(&priv->lock);
	epkcnt = nolock_regb_read(priv, EPKTCNT);
	if (epkcnt >= 255)
		free_space = -1;
	else {
		erxst = nolock_regw_read(priv, ERXSTL);
		erxnd = nolock_regw_read(priv, ERXNDL);
		erxwr = nolock_regw_read(priv, ERXWRPTL);
		erxrd = nolock_regw_read(priv, ERXRDPTL);

		if (erxwr > erxrd)
			free_space = (erxnd - erxst) - (erxwr - erxrd);
		else if (erxwr == erxrd)
			free_space = (erxnd - erxst);
		else
			free_space = erxrd - erxwr - 1;
	}
	mutex_unlock(&priv->lock);
	if (netif_msg_rx_status(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() free_space = %d\n",
			__func__, free_space);
	return free_space;
#endif
	return 0;
}

/*
 * Access the PHY to determine link status
 */
static void enc424j600_check_link_status(struct net_device *ndev)
{
#if 0
	struct enc424j600_net *priv = netdev_priv(ndev);
	u16 reg;
	int duplex;

	reg = enc424j600_phy_read(priv, PHSTAT2);
	if (netif_msg_hw(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() PHSTAT1: %04x, "
			"PHSTAT2: %04x\n", __func__,
			enc424j600_phy_read(priv, PHSTAT1), reg);
	duplex = reg & PHSTAT2_DPXSTAT;

	if (reg & PHSTAT2_LSTAT) {
		netif_carrier_on(ndev);
		if (netif_msg_ifup(priv))
			dev_info(&ndev->dev, "link up - %s\n",
				duplex ? "Full duplex" : "Half duplex");
	} else {
		if (netif_msg_ifdown(priv))
			dev_info(&ndev->dev, "link down\n");
		netif_carrier_off(ndev);
	}
#endif
}

static void enc424j600_tx_clear(struct net_device *ndev, bool err)
{
#if 0
	struct enc424j600_net *priv = netdev_priv(ndev);

	if (err)
		ndev->stats.tx_errors++;
	else
		ndev->stats.tx_packets++;

	if (priv->tx_skb) {
		if (!err)
			ndev->stats.tx_bytes += priv->tx_skb->len;
		dev_kfree_skb(priv->tx_skb);
		priv->tx_skb = NULL;
	}
	locked_reg_bfclr(priv, ECON1, ECON1_TXRTS);
	netif_wake_queue(ndev);
#endif
}

/*
 * RX handler
 * ignore PKTIF because is unreliable! (look at the errata datasheet)
 * check EPKTCNT is the suggested workaround.
 * We don't need to clear interrupt flag, automatically done when
 * enc424j600_hw_rx() decrements the packet counter.
 * Returns how many packet processed.
 */
static int enc424j600_rx_interrupt(struct net_device *ndev)
{
#if 0
	struct enc424j600_net *priv = netdev_priv(ndev);
	int pk_counter, ret;

	pk_counter = locked_regb_read(priv, EPKTCNT);
	if (pk_counter && netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": intRX, pk_cnt: %d\n", pk_counter);
	if (pk_counter > priv->max_pk_counter) {
		/* update statistics */
		priv->max_pk_counter = pk_counter;
		if (netif_msg_rx_status(priv) && priv->max_pk_counter > 1)
			printk(KERN_DEBUG DRV_NAME ": RX max_pk_cnt: %d\n",
				priv->max_pk_counter);
	}
	ret = pk_counter;
	while (pk_counter-- > 0)
		enc424j600_hw_rx(ndev);

	return ret;
#endif
	return 0;
}

static void enc424j600_irq_work_handler(struct work_struct *work)
{
#if 0
	struct enc424j600_net *priv =
		container_of(work, struct enc424j600_net, irq_work);
	struct net_device *ndev = priv->netdev;
	int intflags, loop;

	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);
	/* disable further interrupts */
	locked_reg_bfclr(priv, EIE, EIE_INTIE);

	do {
		loop = 0;
		intflags = locked_regb_read(priv, EIR);
		/* DMA interrupt handler (not currently used) */
		if ((intflags & EIR_DMAIF) != 0) {
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intDMA(%d)\n", loop);
			locked_reg_bfclr(priv, EIR, EIR_DMAIF);
		}
		/* LINK changed handler */
		if ((intflags & EIR_LINKIF) != 0) {
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intLINK(%d)\n", loop);
			enc424j600_check_link_status(ndev);
			/* read PHIR to clear the flag */
			enc424j600_phy_read(priv, PHIR);
		}
		/* TX complete handler */
		if ((intflags & EIR_TXIF) != 0) {
			bool err = false;
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intTX(%d)\n", loop);
			priv->tx_retry_count = 0;
			if (locked_regb_read(priv, ESTAT) & ESTAT_TXABRT) {
				if (netif_msg_tx_err(priv))
					dev_err(&ndev->dev,
						"Tx Error (aborted)\n");
				err = true;
			}
			if (netif_msg_tx_done(priv)) {
				u8 tsv[TSV_SIZE];
				enc424j600_read_tsv(priv, tsv);
				enc424j600_dump_tsv(priv, "Tx Done", tsv);
			}
			enc424j600_tx_clear(ndev, err);
			locked_reg_bfclr(priv, EIR, EIR_TXIF);
		}
		/* TX Error handler */
		if ((intflags & EIR_TXERIF) != 0) {
			u8 tsv[TSV_SIZE];

			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intTXErr(%d)\n", loop);
			locked_reg_bfclr(priv, ECON1, ECON1_TXRTS);
			enc424j600_read_tsv(priv, tsv);
			if (netif_msg_tx_err(priv))
				enc424j600_dump_tsv(priv, "Tx Error", tsv);
			/* Reset TX logic */
			mutex_lock(&priv->lock);
			nolock_reg_bfset(priv, ECON1, ECON1_TXRST);
			nolock_reg_bfclr(priv, ECON1, ECON1_TXRST);
			nolock_txfifo_init(priv, TXSTART_INIT, TXEND_INIT);
			mutex_unlock(&priv->lock);
			/* Transmit Late collision check for retransmit */
			if (TSV_GETBIT(tsv, TSV_TXLATECOLLISION)) {
				if (netif_msg_tx_err(priv))
					printk(KERN_DEBUG DRV_NAME
						": LateCollision TXErr (%d)\n",
						priv->tx_retry_count);
				if (priv->tx_retry_count++ < MAX_TX_RETRYCOUNT)
					locked_reg_bfset(priv, ECON1,
							   ECON1_TXRTS);
				else
					enc424j600_tx_clear(ndev, true);
			} else
				enc424j600_tx_clear(ndev, true);
			locked_reg_bfclr(priv, EIR, EIR_TXERIF);
		}
		/* RX Error handler */
		if ((intflags & EIR_RXERIF) != 0) {
			loop++;
			if (netif_msg_intr(priv))
				printk(KERN_DEBUG DRV_NAME
					": intRXErr(%d)\n", loop);
			/* Check free FIFO space to flag RX overrun */
			if (enc424j600_get_free_rxfifo(priv) <= 0) {
				if (netif_msg_rx_err(priv))
					printk(KERN_DEBUG DRV_NAME
						": RX Overrun\n");
				ndev->stats.rx_dropped++;
			}
			locked_reg_bfclr(priv, EIR, EIR_RXERIF);
		}
		/* RX handler */
		if (enc424j600_rx_interrupt(ndev))
			loop++;
	} while (loop);

	/* re-enable interrupts */
	locked_reg_bfset(priv, EIE, EIE_INTIE);
	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() exit\n", __func__);
#endif
}

/*
 * Hardware transmit function.
 * Fill the buffer memory and send the contents of the transmit buffer
 * onto the network
 */
static void enc424j600_hw_tx(struct enc424j600_net *priv)
{
#if 0
	if (netif_msg_tx_queued(priv))
		printk(KERN_DEBUG DRV_NAME
			": Tx Packet Len:%d\n", priv->tx_skb->len);

	if (netif_msg_pktdata(priv))
		dump_packet(__func__,
			    priv->tx_skb->len, priv->tx_skb->data);
	enc424j600_packet_write(priv, priv->tx_skb->len, priv->tx_skb->data);

#ifdef CONFIG_ENC28J60_WRITEVERIFY
	/* readback and verify written data */
	if (netif_msg_drv(priv)) {
		int test_len, k;
		u8 test_buf[64]; /* limit the test to the first 64 bytes */
		int okflag;

		test_len = priv->tx_skb->len;
		if (test_len > sizeof(test_buf))
			test_len = sizeof(test_buf);

		/* + 1 to skip control byte */
		enc424j600_mem_read(priv, TXSTART_INIT + 1, test_len, test_buf);
		okflag = 1;
		for (k = 0; k < test_len; k++) {
			if (priv->tx_skb->data[k] != test_buf[k]) {
				printk(KERN_DEBUG DRV_NAME
					 ": Error, %d location differ: "
					 "0x%02x-0x%02x\n", k,
					 priv->tx_skb->data[k], test_buf[k]);
				okflag = 0;
			}
		}
		if (!okflag)
			printk(KERN_DEBUG DRV_NAME ": Tx write buffer, "
				"verify ERROR!\n");
	}
#endif
	/* set TX request flag */
	locked_reg_bfset(priv, ECON1, ECON1_TXRTS);
#endif
}

static int enc424j600_send_packet(struct sk_buff *skb, struct net_device *dev)
{
#if 0
	struct enc424j600_net *priv = netdev_priv(dev);

	if (netif_msg_tx_queued(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	/* If some error occurs while trying to transmit this
	 * packet, you should return '1' from this function.
	 * In such a case you _may not_ do anything to the
	 * SKB, it is still owned by the network queueing
	 * layer when an error is returned.  This means you
	 * may not modify any SKB fields, you may not free
	 * the SKB, etc.
	 */
	netif_stop_queue(dev);

	/* save the timestamp */
	priv->netdev->trans_start = jiffies;
	/* Remember the skb for deferred processing */
	priv->tx_skb = skb;
	schedule_work(&priv->tx_work);

	return 0;
#endif
	return 0;
}

static void enc424j600_tx_work_handler(struct work_struct *work)
{
	struct enc424j600_net *priv =
		container_of(work, struct enc424j600_net, tx_work);

	/* actual delivery of data */
	enc424j600_hw_tx(priv);
}

static irqreturn_t enc424j600_irq(int irq, void *dev_id)
{
	struct enc424j600_net *priv = dev_id;

	/*
	 * Can't do anything in interrupt context because we need to
	 * block (spi_sync() is blocking) so fire of the interrupt
	 * handling workqueue.
	 * Remember that we access enc424j600 registers through SPI bus
	 * via spi_sync() call.
	 */
	schedule_work(&priv->irq_work);

	return IRQ_HANDLED;
}

static void enc424j600_tx_timeout(struct net_device *ndev)
{
	struct enc424j600_net *priv = netdev_priv(ndev);

	if (netif_msg_timer(priv))
		dev_err(&ndev->dev, DRV_NAME " tx timeout\n");

	ndev->stats.tx_errors++;
	/* can't restart safely under softirq */
	schedule_work(&priv->restart_work);
}

/*
 * Open/initialize the board. This is called (in the current kernel)
 * sometime after booting when the 'ifconfig' program is run.
 *
 * This routine should set everything up anew at each open, even
 * registers that "should" only need to be set once at boot, so that
 * there is non-reboot way to recover if something goes wrong.
 */
static int enc424j600_net_open(struct net_device *dev)
{
	struct enc424j600_net *priv = netdev_priv(dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	if (!is_valid_ether_addr(dev->dev_addr)) {
		if (netif_msg_ifup(priv))
			dev_err(&dev->dev, "invalid MAC address %pM\n",
				dev->dev_addr);
		return -EADDRNOTAVAIL;
	}
	/* Reset the hardware here (and take it out of low power mode) */
	enc424j600_lowpower(priv, false);
	enc424j600_hw_disable(priv);
	if (!enc424j600_hw_init(priv)) {
		if (netif_msg_ifup(priv))
			dev_err(&dev->dev, "hw_reset() failed\n");
		return -EINVAL;
	}
	/* Update the MAC address (in case user has changed it) */
	enc424j600_set_hw_macaddr(dev);
	/* Enable interrupts */
	enc424j600_hw_enable(priv);
	/* check link status */
	enc424j600_check_link_status(dev);
	/* We are now ready to accept transmit requests from
	 * the queueing layer of the networking.
	 */
	netif_start_queue(dev);

	return 0;
}

/* The inverse routine to net_open(). */
static int enc424j600_net_close(struct net_device *dev)
{
	struct enc424j600_net *priv = netdev_priv(dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);

	enc424j600_hw_disable(priv);
	enc424j600_lowpower(priv, true);
	netif_stop_queue(dev);

	return 0;
}

/*
 * Set or clear the multicast filter for this adapter
 * num_addrs == -1	Promiscuous mode, receive all packets
 * num_addrs == 0	Normal mode, filter out multicast packets
 * num_addrs > 0	Multicast mode, receive normal and MC packets
 */
static void enc424j600_set_multicast_list(struct net_device *dev)
{
#if 0
	struct enc424j600_net *priv = netdev_priv(dev);
	int oldfilter = priv->rxfilter;

	if (dev->flags & IFF_PROMISC) {
		if (netif_msg_link(priv))
			dev_info(&dev->dev, "promiscuous mode\n");
		priv->rxfilter = RXFILTER_PROMISC;
	} else if ((dev->flags & IFF_ALLMULTI) || dev->mc_count) {
		if (netif_msg_link(priv))
			dev_info(&dev->dev, "%smulticast mode\n",
				(dev->flags & IFF_ALLMULTI) ? "all-" : "");
		priv->rxfilter = RXFILTER_MULTI;
	} else {
		if (netif_msg_link(priv))
			dev_info(&dev->dev, "normal mode\n");
		priv->rxfilter = RXFILTER_NORMAL;
	}

	if (oldfilter != priv->rxfilter)
		schedule_work(&priv->setrx_work);
#endif
}

static void enc424j600_setrx_work_handler(struct work_struct *work)
{
#if 0
	struct enc424j600_net *priv =
		container_of(work, struct enc424j600_net, setrx_work);

	if (priv->rxfilter == RXFILTER_PROMISC) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": promiscuous mode\n");
		locked_regb_write(priv, ERXFCON, 0x00);
	} else if (priv->rxfilter == RXFILTER_MULTI) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": multicast mode\n");
		locked_regb_write(priv, ERXFCON,
					ERXFCON_UCEN | ERXFCON_CRCEN |
					ERXFCON_BCEN | ERXFCON_MCEN);
	} else {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": normal mode\n");
		locked_regb_write(priv, ERXFCON,
					ERXFCON_UCEN | ERXFCON_CRCEN |
					ERXFCON_BCEN);
	}
#endif
}

static void enc424j600_restart_work_handler(struct work_struct *work)
{
	struct enc424j600_net *priv =
			container_of(work, struct enc424j600_net, restart_work);
	struct net_device *ndev = priv->netdev;
	int ret;

	rtnl_lock();
	if (netif_running(ndev)) {
		enc424j600_net_close(ndev);
		ret = enc424j600_net_open(ndev);
		if (unlikely(ret)) {
			dev_info(&ndev->dev, " could not restart %d\n", ret);
			dev_close(ndev);
		}
	}
	rtnl_unlock();
}

/* ......................... ETHTOOL SUPPORT ........................... */

static void
enc424j600_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info,
		dev_name(dev->dev.parent), sizeof(info->bus_info));
}

static int
enc424j600_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct enc424j600_net *priv = netdev_priv(dev);

	cmd->transceiver = XCVR_INTERNAL;
	cmd->supported	= SUPPORTED_10baseT_Half
			| SUPPORTED_10baseT_Full
			| SUPPORTED_100baseT_Half
			| SUPPORTED_100baseT_Full
			| SUPPORTED_TP;
	cmd->speed	= priv->speed100 ? SPEED_100 : SPEED_10;
	cmd->duplex	= priv->full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
	cmd->port	= PORT_TP;
	cmd->autoneg	= priv->autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE;

	return 0;
}

static int
enc424j600_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	return enc424j600_setlink(dev, cmd->autoneg, cmd->speed, cmd->duplex);
}

static u32 enc424j600_get_msglevel(struct net_device *dev)
{
	struct enc424j600_net *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void enc424j600_set_msglevel(struct net_device *dev, u32 val)
{
	struct enc424j600_net *priv = netdev_priv(dev);
	priv->msg_enable = val;
}

static const struct ethtool_ops enc424j600_ethtool_ops = {
	.get_settings	= enc424j600_get_settings,
	.set_settings	= enc424j600_set_settings,
	.get_drvinfo	= enc424j600_get_drvinfo,
	.get_msglevel	= enc424j600_get_msglevel,
	.set_msglevel	= enc424j600_set_msglevel,
};

static int enc424j600_chipset_init(struct net_device *dev)
{
	struct enc424j600_net *priv = netdev_priv(dev);

	return enc424j600_hw_init(priv);
	enc424j600_get_hw_macaddr(dev);

}

static const struct net_device_ops enc424j600_netdev_ops = {
	.ndo_open		= enc424j600_net_open,
	.ndo_stop		= enc424j600_net_close,
	.ndo_start_xmit		= enc424j600_send_packet,
	.ndo_set_multicast_list = enc424j600_set_multicast_list,
	.ndo_set_mac_address	= enc424j600_set_mac_address,
	.ndo_tx_timeout		= enc424j600_tx_timeout,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static int __devinit enc424j600_probe(struct spi_device *spi)
{
	struct net_device *dev;
	struct enc424j600_net *priv;
	int ret = 0;

	if (netif_msg_drv(&debug))
		dev_info(&spi->dev, DRV_NAME " Ethernet driver %s loaded\n",
			DRV_VERSION);

	dev = alloc_etherdev(sizeof(struct enc424j600_net));
	if (!dev) {
		if (netif_msg_drv(&debug))
			dev_err(&spi->dev, DRV_NAME
				": unable to alloc new ethernet\n");
		ret = -ENOMEM;
		goto error_alloc;
	}
	priv = netdev_priv(dev);

	priv->netdev = dev;	/* priv to netdev reference */
	priv->spi = spi;	/* priv to spi reference */
	priv->msg_enable = netif_msg_init(debug.msg_enable,
						ENC424J600_MSG_DEFAULT);
	mutex_init(&priv->lock);
	INIT_WORK(&priv->tx_work, enc424j600_tx_work_handler);
	INIT_WORK(&priv->setrx_work, enc424j600_setrx_work_handler);
	INIT_WORK(&priv->irq_work, enc424j600_irq_work_handler);
	INIT_WORK(&priv->restart_work, enc424j600_restart_work_handler);
	dev_set_drvdata(&spi->dev, priv);	/* spi to priv reference */
	SET_NETDEV_DEV(dev, &spi->dev);

	/* If requested, allocate DMA buffers */
	if (enc424j600_enable_dma) {
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
			enc424j600_enable_dma = 0;
		}
	}

	/* Allocate non-DMA buffers */
	if (!enc424j600_enable_dma) {
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

	if (!enc424j600_chipset_init(dev)) {
		if (netif_msg_probe(priv))
			dev_info(&spi->dev, DRV_NAME " chip not found\n");
		ret = -EIO;
		goto error_irq;
	}

	/* Board setup must set the relevant edge trigger type;
	 * level triggers won't currently work.
	 */
	ret = request_irq(spi->irq, enc424j600_irq, 0, DRV_NAME, priv);
	if (ret < 0) {
		if (netif_msg_probe(priv))
			dev_err(&spi->dev, DRV_NAME ": request irq %d failed "
				"(ret = %d)\n", spi->irq, ret);
		goto error_irq;
	}

	dev->if_port = IF_PORT_10BASET; /* TODO: ?? */
	dev->irq = spi->irq;
	dev->netdev_ops = &enc424j600_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	SET_ETHTOOL_OPS(dev, &enc424j600_ethtool_ops);

	enc424j600_lowpower(priv, true);

	ret = register_netdev(dev);
	if (ret) {
		if (netif_msg_probe(priv))
			dev_err(&spi->dev, "register netdev " DRV_NAME
				" failed (ret = %d)\n", ret);
		goto error_register;
	}
	dev_info(&dev->dev, DRV_NAME " driver registered\n");

	return 0;

error_register:
	free_irq(spi->irq, priv);
error_irq:
	free_netdev(dev);
	if (!enc424j600_enable_dma)
		kfree(priv->spi_rx_buf);
error_rx_buf:
	if (!enc424j600_enable_dma)
		kfree(priv->spi_tx_buf);
error_tx_buf:
	if (enc424j600_enable_dma)
		dma_free_coherent(&spi->dev, PAGE_SIZE,
				  priv->spi_tx_buf, priv->spi_tx_dma);
error_alloc:
	return ret;
}

static int __devexit enc424j600_remove(struct spi_device *spi)
{
	struct enc424j600_net *priv = dev_get_drvdata(&spi->dev);

	if (netif_msg_drv(priv))
		printk(KERN_DEBUG DRV_NAME ": remove\n");

	unregister_netdev(priv->netdev);
	free_irq(spi->irq, priv);
	free_netdev(priv->netdev);

	return 0;
}

static struct spi_driver enc424j600_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
	 },
	.probe = enc424j600_probe,
	.remove = __devexit_p(enc424j600_remove),
};

static int __init enc424j600_init(void)
{
	msec20_to_jiffies = msecs_to_jiffies(20);

	return spi_register_driver(&enc424j600_driver);
}

module_init(enc424j600_init);

static void __exit enc424j600_exit(void)
{
	spi_unregister_driver(&enc424j600_driver);
}

module_exit(enc424j600_exit);

MODULE_DESCRIPTION(DRV_NAME " ethernet driver");
MODULE_AUTHOR("Kuba Marek <blue.cube@seznam.cz>");
MODULE_LICENSE("GPL");
module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., ffff=all)");
module_param(enc424j600_enable_dma, int, S_IRUGO);
MODULE_PARM_DESC(enc424j600_enable_dma, "Enable SPI DMA. Default: 0 (Off)");
