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
#define DEBUG 1
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
}

/*
 * Read data from chip SRAM.
 * Returns 0 on success, negative error code otherwise.
 */
static int enc424j600_read_sram(struct enc424j600_net *priv,
			 u8 *dst, int len, u16 srcaddr)
{
	int ret;

	if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0) {
		return -EINVAL;
	}

	/* First set the general purpose read pointer */
	priv->spi_tx_buf[0] = WGPRDPT;
	priv->spi_tx_buf[1] = srcaddr & 0xFF;
	priv->spi_tx_buf[2] = srcaddr >> 8;
	enc424j600_spi_trans(priv, 3);

	/* Transfer the data */
	/* TODO: Does this work? */
	priv->spi_tx_buf[0] = RGPDATA;
	ret = enc424j600_spi_trans(priv, len + 1);

	/* Copy the data to the tx buffer */
	memcpy(dst, &priv->spi_rx_buf[1], len);

	return ret;
}

/*
 * Write data to chip SRAM.
 * Returns 0 on success, negative error code otherwise.
 */
static int enc424j600_write_sram(struct enc424j600_net *priv,
			 const u8 *src, int len, u16 dstaddr)
{
	int ret;

	if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0) {
		return -EINVAL;
	}

	/* First set the general purpose write pointer */
	priv->spi_tx_buf[0] = WGPWRPT;
	priv->spi_tx_buf[1] = dstaddr & 0xFF;
	priv->spi_tx_buf[2] = dstaddr >> 8;	
	enc424j600_spi_trans(priv, 3);

	/* Copy the data to the tx buffer */
	memcpy(&priv->spi_tx_buf[1], src, len);

	/* Transfer the data */
	/* TODO: Does this work? */
	priv->spi_tx_buf[0] = WGPDATA;
	ret = enc424j600_spi_trans(priv, len + 1);

#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		/* We're only checking the first few bytes. */
		u8 verify_buffer[64];
		int verify_len;
		int k;
		
		if (len > sizeof(verify_buffer))
			verify_len = sizeof(verify_buffer);
		else
			verify_len = len;
		
		enc424j600_read_sram(priv, verify_buffer, verify_len, dstaddr);

		for (k = 0; k < verify_len; k++) {
                        if (src[k] != verify_buffer[k]) {
                                printk(KERN_DEBUG DRV_NAME
                                         ": RAM write verify error, %d location "
					 "differs: 0x%02x-0x%02x\n", k,
                                         src[k], verify_buffer[k]);
                        }
                }
	}
#endif

	return ret;
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

#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u8 val;

		enc424j600_read_8b_sfr(priv, sfr, &val);

		if (val != data)
			printk(KERN_DEBUG DRV_NAME
				 ": 8 bit sfr write verify error, values differ: "
				 "0x%02x - 0x%02x\n", val, data);
	}
#endif

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

#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u16 val;

		enc424j600_read_16b_sfr(priv, sfr, &val);

		if (val != data)
			printk(KERN_DEBUG DRV_NAME
				 ": 8 bit sfr write verify error, values differ: "
				 "0x%02x - 0x%02x\n", val, data);
	}
#endif

	return ret;
}

/*
 * Set bits in an 8bit SFR.
 */
static int enc424j600_set_bits(struct enc424j600_net *priv, u8 addr, u8 mask)
{
	int ret;

	enc424j600_set_bank(priv, addr);
	priv->spi_tx_buf[0] = BFS(addr);
	priv->spi_tx_buf[1] = mask;
	ret = enc424j600_spi_trans(priv, 2);

#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u8 val;

		enc424j600_read_8b_sfr(priv, addr, &val);

		if (val & mask != mask)
			printk(KERN_DEBUG DRV_NAME
				 ": set_bits verify error, all bits are not set: "
				 "0x%02x; mask: 0x%02x\n", val, mask);
	}
#endif
	
	return ret;
}

/*
 * Clear bits in an 8bit SFR.
 */
static int enc424j600_clear_bits(struct enc424j600_net *priv, u8 addr, u8 mask)
{
	int ret;

	enc424j600_set_bank(priv, addr);
	priv->spi_tx_buf[0] = BFC(addr);
	priv->spi_tx_buf[1] = mask;
	ret = enc424j600_spi_trans(priv, 2);

#ifdef CONFIG_ENC28J60_WRITEVERIFY
	if (netif_msg_drv(priv)) {
		u8 val;

		enc424j600_read_8b_sfr(priv, addr, &val);

		if (val & mask != 0)
			printk(KERN_DEBUG DRV_NAME
				": set_bits verify error, all bits are not "
				"cleared: 0x%02x; mask: 0x%02x\n", val, mask);
	}
#endif

	return ret;
}

/*
 * Read memory from the wrapped RX area.
 * Handles srcaddr that is behind the rx area end (this is wrapped
 * as well).
 */
static int enc424j600_read_rx_area(struct enc424j600_net *priv,
			 u8 *dst, int len, u16 srcaddr)
{
	int ret;
	int split;

	if (srcaddr >= SRAM_SIZE) {
		srcaddr -= RX_BUFFER_SIZE;
	}

	if (srcaddr + len < SRAM_SIZE) {
		return enc424j600_read_sram(priv, dst, len, srcaddr);
	} else {
		// length of the first half
		split = SRAM_SIZE - srcaddr + 1;
		ret = enc424j600_read_sram(priv, dst, split, srcaddr);
		if (ret)
			return ret;

		return enc424j600_read_sram(priv, dst + split, len - split,
			ERXST_VAL);
	}
}

/*
 * Reset the enc424j600.
 * (Datasheet: 8.1)
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
		printk(KERN_DEBUG DRV_NAME ": %s() EUDAST is not zero!\n", __func__);

	udelay(500);
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
 * Set the filters in the chip according to priv->rxfilter .
 */
static void enc424j600_set_hw_filters(struct enc424j600_net *priv)
{
	u8 erxfconl = CRCEN | RUNTEN | UCEN | BCEN;

	if (priv->rxfilter == RXFILTER_PROMISC) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": promiscuous mode\n");
		erxfconl |= NOTMEEN | MCEN;
	} else if (priv->rxfilter == RXFILTER_MULTI) {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": multicast mode\n");
		erxfconl |= MCEN;
	} else {
		if (netif_msg_drv(priv))
			printk(KERN_DEBUG DRV_NAME ": normal mode\n");
	}

	enc424j600_write_8b_sfr(priv, ERXFCONL, erxfconl);
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

	enc424j600_read_16b_sfr(priv, MAADR1L, &maadr1);
	ndev->dev_addr[0] = maadr1 & 0xff;;
	ndev->dev_addr[1] = maadr1 >> 8;
	enc424j600_read_16b_sfr(priv, MAADR2L, &maadr2);
	ndev->dev_addr[2] = maadr2 & 0xff;;
	ndev->dev_addr[3] = maadr2 >> 8;
	enc424j600_read_16b_sfr(priv, MAADR3L, &maadr3);
	ndev->dev_addr[4] = maadr3 & 0xff;;
	ndev->dev_addr[5] = maadr3 >> 8;

	if (netif_msg_drv(priv))
		printk(KERN_INFO DRV_NAME
			": %s: Setting MAC is %pM\n",
			ndev->name, ndev->dev_addr);

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

	enc424j600_write_16b_sfr(priv, MAADR1L, ndev->dev_addr[0] | ndev->dev_addr[1] << 8);
	enc424j600_write_16b_sfr(priv, MAADR2L, ndev->dev_addr[2] | ndev->dev_addr[3] << 8);
	enc424j600_write_16b_sfr(priv, MAADR3L, ndev->dev_addr[4] | ndev->dev_addr[5] << 8);

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
 * TODO
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
 * Low power mode shrinks power consumption about 100x, so we'd like
 * the chip to be in that mode whenever it's inactive.  (However, we
 * can't stay in lowpower mode during suspend with WOL active.)
 * TODO
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

/* Waits for autonegotiation to complete and sets FULDPX bit in macon2. */
static void enc424j600_wait_for_autoneg(struct enc424j600_net *priv)
{
	u16 phstat1;

	do {
		enc424j600_phy_read(priv, PHSTAT1, &phstat1);
	} while(!(phstat1 & ANDONE));
}

/* Sets the protected area in rx buffer to be 2 bytes long
 * (smallest allowed value)
 * Takes care of the rx buffer wrapping */
static void enc424j600_clear_unprocessed_rx_area(struct enc424j600_net *priv)
{
	u16 tail = priv->next_pk_ptr - 2;

	if (tail < ERXST_VAL)
		tail = SRAM_SIZE - 2;
	
	enc424j600_write_16b_sfr(priv, ERXTAILL, tail);
}

/* Prepare the receive buffer in the chip
 * Datasheet: 8.3, 9.2.1 */
static void enc424j600_prepare_rx_buffer(struct enc424j600_net *priv)
{
	/* ERXST (start of the rx buffer => its size) */
	enc424j600_write_16b_sfr(priv, ERXSTL, ERXST_VAL);

	/* Where the next frame should be read. */
	priv->next_pk_ptr = ERXST_VAL;

	/* ERXTAIL (end of the unprocessed block) */
	enc424j600_clear_unprocessed_rx_area(priv);
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
	

	/* Prepare the receive buffer */
	enc424j600_prepare_rx_buffer(priv);

	enc424j600_set_hw_filters(priv);
	
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


	// TODO: First PHY, then MAC ?

	/* MACON2
	 * defer transmission if collision occurs (only for half duplex)
	 * pad to 60 or 64 bytes and append CRC
	 * enable receiving huge frames (instead of limiting packet size) */
	macon2 = MACON2_DEFER | PADCFG2 | PADCFG0 | TXCRCEN | HFRMEN;

	/* If autonegotiation is enabled, we have to wait untill it finishes
	 * and set the PHYDPX bit in MACON2 correctly */
	if (priv->autoneg) {
		u8 estath;
		enc424j600_wait_for_autoneg(priv);

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
 * Receive Status vector
 */
static void enc424j600_dump_rsv(struct enc424j600_net *priv, const char *msg,
			      u16 pk_ptr, int len, u32 rxstat)
{
	printk(KERN_DEBUG DRV_NAME ": %s - NextPk: 0x%04x - RSV:\n",
		msg, pk_ptr);
	printk(KERN_DEBUG DRV_NAME ": Byte count: %d\n", len);

#define PRINT_RSV_BIT(value, desc) \
	printk(KERN_DEBUG DRV_NAME ": " desc ": %d\n", RSV_GETBIT(rxstat, (value)))

	PRINT_RSV_BIT(RSV_UNICAST_FILTER, "Unicast Filter Match");
	PRINT_RSV_BIT(RSV_PATTERN_FILTER, "Pattern Match Filter Match");
	PRINT_RSV_BIT(RSV_MAGIC_FILTER, "Magic Packet Filter Match");
	PRINT_RSV_BIT(RSV_HASH_FILTER, "Hash Filter Match");
	PRINT_RSV_BIT(RSV_NOT_ME_FILTER, "Not-Me Filter Match");
	PRINT_RSV_BIT(RSV_RUNT_FILTER, "Runt Filter Match");
	PRINT_RSV_BIT(RSV_VLAN, "Receive VLAN Type Detected");
	PRINT_RSV_BIT(RSV_UNKNOWN_OPCODE, "Receive Unknown Opcode");
	PRINT_RSV_BIT(RSV_PAUSE_CONTROL_FRAME, "Receive Pause Control Frame");
	PRINT_RSV_BIT(RSV_CONTROL_FRAME, "Receive Control Frame");
	PRINT_RSV_BIT(RSV_DRIBBLE_NIBBLE, "Dribble Nibble");
	PRINT_RSV_BIT(RSV_BROADCAST, "Receive Broadcast Packet");
	PRINT_RSV_BIT(RSV_MULTICAST, "Receive Multicast Packet");
	PRINT_RSV_BIT(RSV_RXOK, "Received Ok");
	PRINT_RSV_BIT(RSV_LENGTH_OUT_OF_RANGE, "Length Out of Range");
	PRINT_RSV_BIT(RSV_LENGTH_CHECK_ERROR, "Length Check Error");
	PRINT_RSV_BIT(RSV_CRC_ERROR, "CRC Error");
	PRINT_RSV_BIT(RSV_CARRIER_EVENT, "Carrier Event Previously Seen");
	PRINT_RSV_BIT(RSV_PREVIOUSLY_IGNORED, "Packet Previously Ignored");

#undef PRINT_RSV_BIT
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
 * TODO: Locking?
 */
static void enc424j600_hw_rx(struct net_device *ndev)
{
	struct enc424j600_net *priv = netdev_priv(ndev);
	struct sk_buff *skb = NULL;
	u16 next_packet;
	u32 rxstat;
	u8 rsv[RSV_SIZE];
	int len;

	if (netif_msg_rx_status(priv))
		printk(KERN_DEBUG DRV_NAME ": RX pk_addr:0x%04x\n",
			priv->next_pk_ptr);
	
	// TODO corrupted packet addresses
	#if 0
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
	#endif
	
	/* Read next packet pointer and rx status vector */
	enc424j600_read_rx_area(priv, rsv, sizeof(rsv), priv->next_pk_ptr);

	next_packet = rsv[0] | (rsv[1] << 8);
	len = rsv[2] | (rsv[3] << 8);
	rxstat = rsv[4] | (rsv[5] << 8) | (rsv[6] << 16);

	if (netif_msg_rx_status(priv))
		enc424j600_dump_rsv(priv, __func__, next_packet, len, rxstat);

	if (!RSV_GETBIT(rxstat, RSV_RXOK) || len > MAX_FRAMELEN) {
		if (netif_msg_rx_err(priv))
			dev_err(&ndev->dev, "Rx Error (%04x)\n", rxstat);
		ndev->stats.rx_errors++;
		if (RSV_GETBIT(rxstat, RSV_CRC_ERROR))
			ndev->stats.rx_crc_errors++;
		if (RSV_GETBIT(rxstat, RSV_LENGTH_CHECK_ERROR))
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
			enc424j600_read_rx_area(priv, skb_put(skb, len), len,
				priv->next_pk_ptr + RSV_SIZE);
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
	mutex_lock(&priv->lock);

	priv->next_pk_ptr = next_packet;

	/* unprotect the area that was used by this packet. */
	enc424j600_clear_unprocessed_rx_area(priv);

	/* we are done with this packet, decrement the packet counter */
	enc424j600_set_bits(priv, ECON1H, PKTDEC);
	mutex_unlock(&priv->lock);
}

#if 0
/*
 * Calculate free space in RxFIFO
 * TODO: This function may come handy.
 */
static int enc424j600_get_free_rxfifo(struct enc424j600_net *priv)
{
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
	return 0;
}
#endif

/*
 * Access the PHY to determine link status
 */
static void enc424j600_check_link_status(struct enc424j600_net *priv)
{
	u8 estath;

	enc424j600_read_8b_sfr(priv, ESTATH, &estath);
	if (estath & PHYLNK) {
		if (priv->autoneg) {
			enc424j600_wait_for_autoneg(priv);
			if (estath & PHYDPX) {
				u16 macon2;
				enc424j600_read_16b_sfr(
					priv, MACON2L, &macon2);
				macon2 |= FULDPX;
				enc424j600_write_16b_sfr(
					priv, MACON2L, macon2);
			}

		}
		netif_carrier_on(priv->netdev);
		if (netif_msg_ifup(priv))
			dev_info(&(priv->netdev->dev), "link up\n");
	} else {
		if (netif_msg_ifdown(priv))
			dev_info(&(priv->netdev->dev), "link down\n");
		netif_carrier_off(priv->netdev);
	}
}

static void enc424j600_tx_clear(struct enc424j600_net *priv, bool err)
{
	struct net_device *ndev = priv->netdev;

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

	netif_wake_queue(ndev);
}

static void enc424j600_int_rx_abbort_handler(struct enc424j600_net *priv)
{
	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME
			": intRXAbt\n");
	mutex_lock(&priv->lock);
	priv->netdev->stats.rx_dropped++;
	mutex_unlock(&priv->lock);
}

static void enc424j600_int_link_handler(struct enc424j600_net *priv)
{
	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME
			": intLINK\n");

	/* we check more than is necessary here --
	 * only PHYLNK would be needed. */
	enc424j600_check_link_status(priv);
}

static void enc424j600_int_tx_handler(struct enc424j600_net *priv)
{
	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME
			": intTX\n");
	
	mutex_lock(&priv->lock);
	enc424j600_tx_clear(priv, false);
	mutex_unlock(&priv->lock);
}

static void enc424j600_int_tx_err_handler(struct enc424j600_net *priv)
{
	u16 etxstat;
	
	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME
			": intTXErr\n");

	mutex_lock(&priv->lock);

	enc424j600_read_16b_sfr(priv, ETXSTATL, &etxstat);

	if (etxstat & LATECOL) {
		if (netif_msg_tx_err(priv))
			printk(KERN_DEBUG DRV_NAME
				": Late collision TXErr (%d)\n",
				priv->tx_retry_count);
		if (priv->tx_retry_count++ < MAX_TX_RETRYCOUNT)
			enc424j600_set_bits(priv, ECON1L, TXRTS);
		else
			enc424j600_tx_clear(priv, true);
	} else if (etxstat & MAXCOL) {
		if (netif_msg_tx_err(priv))
			printk(KERN_DEBUG DRV_NAME
				": Max collisions TXErr\n");
		enc424j600_tx_clear(priv, true);
	} else {
		enc424j600_tx_clear(priv, true);
	}

	mutex_unlock(&priv->lock);
}

static int enc424j600_int_received_packet_handler(struct enc424j600_net *priv)
{
	uint8_t pk_counter;
	int ret;

	enc424j600_read_8b_sfr(priv, ESTATL, &pk_counter);

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
		enc424j600_hw_rx(priv->netdev);

	return ret;
}


static void enc424j600_irq_work_handler(struct work_struct *work)
{
	struct enc424j600_net *priv =
		container_of(work, struct enc424j600_net, irq_work);
	u16 eir;

	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() enter\n", __func__);
	
	/* disable further interrupts */
	enc424j600_clear_bits(priv, EIEH, INTIE);

	do {
		u8 eirh;
		u8 eirl;

		enc424j600_read_16b_sfr(priv, EIRL, &eir);
		/* TODO: Possible race condition here! Is it even possible to avoid this? */
		eirh = eir >> 8;
		eirl = eir & 0xff;
		enc424j600_clear_bits(priv, EIRL, eirl);
		enc424j600_clear_bits(priv, EIRH, eirh);


		/* Unused interrupts:
		 * 	Modular exponentiation complete
		 * 	Hash complete
		 * 	AES complete
		 * 	DMA complete
		 * 	Receive packet counter full
		 */

		/* link changed handler */
		if ((eirh & LINKIF) != 0) {
			enc424j600_int_link_handler(priv);
		}

		/* TX complete handler */
		if ((eirl & TXIF) != 0) {
			enc424j600_int_tx_handler(priv);
		}

		/* TX Error handler */
		if ((eirl & TXABTIF) != 0) {
			enc424j600_int_tx_err_handler(priv);
		}

		/* RX Error handler */
		if ((eirl & RXABTIF) != 0) {
			enc424j600_int_rx_abbort_handler(priv);
		}

		/* RX handler */
		if ((eirl & PKTIF) != 0) {
			enc424j600_int_received_packet_handler(priv);
		}
	} while (eir);

	/* re-enable interrupts */
	enc424j600_set_bits(priv, EIEH, INTIE);
	if (netif_msg_intr(priv))
		printk(KERN_DEBUG DRV_NAME ": %s() exit\n", __func__);
}

/*
 * Hardware transmit function.
 * Fill the buffer memory and send the contents of the transmit buffer
 * onto the network
 */
static void enc424j600_hw_tx(struct enc424j600_net *priv)
{
	if (netif_msg_tx_queued(priv))
		printk(KERN_DEBUG DRV_NAME
			": Tx Packet Len:%d\n", priv->tx_skb->len);

	if (netif_msg_pktdata(priv))
		dump_packet(__func__,
			priv->tx_skb->len, priv->tx_skb->data);
	
	mutex_lock(&priv->lock);

	/* Copy the packet into the transmit buffer */
	enc424j600_write_sram(priv,
		priv->tx_skb->data, priv->tx_skb->len,
		SRAM_GP_START);

	/* Set the tx pointer to start of general purpose SRAM area */
	enc424j600_write_16b_sfr(priv, ETXSTL, SRAM_GP_START);

	/* Write the transfer length */
	enc424j600_write_16b_sfr(priv, ETXLENL, priv->tx_skb->len);
	
	/* set TX request flag */
	enc424j600_set_bits(priv, ECON1L, TXRTS);

	mutex_unlock(&priv->lock);

}

static int enc424j600_send_packet(struct sk_buff *skb, struct net_device *dev)
{
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
	enc424j600_check_link_status(priv);
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
 */
static void enc424j600_set_multicast_list(struct net_device *dev)
{
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
}

static void enc424j600_setrx_work_handler(struct work_struct *work)
{
	struct enc424j600_net *priv =
		container_of(work, struct enc424j600_net, setrx_work);

	enc424j600_set_hw_filters(priv);
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
	int ret;

	ret = enc424j600_hw_init(priv);
	enc424j600_get_hw_macaddr(dev);

	return ret;
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
#if SPI_TRANSFER_BUF_LEN > PAGE_SIZE / 2
#error "A problem in DMA buffer allocation"
#endif
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
	ret = request_irq(spi->irq, enc424j600_irq, IRQF_TRIGGER_FALLING, DRV_NAME, priv);
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
