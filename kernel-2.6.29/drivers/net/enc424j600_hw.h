/*
 * enc424j600_hw.h: Register definitions
 *
 */

#ifndef _ENC424J600_HW_H
#define _ENC424J600_HW_H

/*
 * ENC424J600 Control Registers
 * Control register definitions are a combination of address
 * and bank number
 * - Register address	(bits 0-4)
 * - Bank number	(bits 5-6)
 * Most of the registers are 16bit and only the low bytes are listed here
 * (high byte is always +1)
 */
#define ADDR_MASK	0x1F
#define BANK_MASK	0x60

/* All-bank registers */
#define EUDAST		0x16
#define EUDAND		0x18
#define ESTAT		0x1A
#define EIR		0x1C
#define ECON1		0x1E

/* Bank 0 registers */
#define ETXSTL		(0x00 | 0x00)
#define ETXLENL		(0x02 | 0x00)
#define ERXSTL		(0x04 | 0x00)
#define ERXTAILL	(0x06 | 0x00)
#define ERXHEADL	(0x08 | 0x00)
#define EDMASTL		(0x0A | 0x00)
#define EDMALENL	(0x0C | 0x00)
#define EDMADSTL	(0x0E | 0x00)
#define EDMACSL		(0x10 | 0x00)
#define ETXSTATL	(0x12 | 0x00)
#define ETXWIREL	(0x14 | 0x00)

/* Bank 1 registers */
#define EHT1L		(0x00 | 0x20)
#define EHT2L		(0x02 | 0x20)
#define EHT3L		(0x04 | 0x20)
#define EHT4L		(0x06 | 0x20)
#define EPMM1L		(0x08 | 0x20)
#define EPMM2L		(0x0A | 0x20)
#define EPMM3L		(0x0C | 0x20)
#define EPMM4L		(0x0E | 0x20)
#define EPMCSL		(0x10 | 0x20)
#define EPMOL		(0x12 | 0x20)
#define ERXFCONL	(0x14 | 0x20)

/* Bank 2 registers */
#define MACON1L		(0x00 | 0x40)
#define MACON2L		(0x02 | 0x40)
#define MABBIPGL	(0x04 | 0x40)
#define MAIPGL		(0x06 | 0x40)
#define MACLCONL	(0x08 | 0x40)
#define MAMXFLL		(0x0A | 0x40)
#define MICMDL		(0x12 | 0x40)
#define MIREGADRL	(0x14 | 0x40)

/* Bank 3 registers */
#define MAADR3L		(0x00 | 0x60)
#define MAADR2L		(0x02 | 0x60)
#define MAADR1L		(0x04 | 0x60)
#define MIWRL		(0x06 | 0x60)
#define MIRDL		(0x08 | 0x60)
#define MISTATL		(0x0A | 0x60)
#define EPAUSL		(0x0C | 0x60)
#define ECON2L		(0x0E | 0x60)
#define ERXWML		(0x10 | 0x60)
#define EIEL		(0x12 | 0x60)
#define EIDLEDL		(0x14 | 0x60)

/* Unbanked registers */
#define EGPDATA		(0x00 | 0x80)
#define ERXDATA		(0x02 | 0x80)
#define EUDADATA	(0x04 | 0x80)
#define EGPRDPTL	(0x06 | 0x80)
#define EGPWRPTL	(0x08 | 0x80)
#define ERXRDPTL	(0x0A | 0x80)
#define ERXWRPTL	(0x0C | 0x80)
#define EUDARDPTL	(0x0E | 0x80)
#define EUDAWRPTL	(0x10 | 0x80)

/* PHY registers */
#define PHCON1		0x00
#define PHSTAT1		0x01
#define PHANA		0x04
#define PHANLPA		0x05
#define PHANE		0x06
#define PHCON2		0x11
#define PHSTAT2		0x1B
#define PHSTAT3		0x1F

/* Single-byte nstructions */
#define B0SEL		0xC0	/* Bank 0 Select */
#define B1SEL		0xC2	/* Bank 1 Select */
#define B2SEL		0xC4	/* Bank 2 Select */
#define B3SEL		0xC6	/* Bank 3 Select */
#define SETETHRST	0xCA	/* System Reset */
#define FCDISABLE	0xE0	/* Flow Control Disable */
#define FCSINGLE	0xE2	/* Flow Control Single */
#define FCMULTIPLE	0xE4	/* Flow Control Multiple */
#define FCCLEAR		0xE6	/* Flow Control Clear */
#define SETPKTDEC	0xCC	/* Decrement Packet Counter */
#define DMASTOP		0xD2	/* DMA Stop */
#define DMACKSUM	0xD8	/* DMA Start Checksum */
#define DMACKSUMS	0xDA	/* DMA Start Checksum with Seed */
#define DMACOPY		0xDC	/* DMA Start Copy */
#define DMACOPYS	0xDE	/* DMA Start Copy and Checksum with Seed */
#define SETTXRTS	0xD4	/* Request Packet Transmission */
#define ENABLERX	0xE8	/* Enable RX */
#define DISABLERX	0xEA	/* Disable RX */
#define SETEIE		0xEC	/* Enable Interrupts */
#define CLREIE		0xEE	/* Disable Interrupts */

/* Two byte instructions */
#define RBSEL		0xC8	/* Read Bank Select */

/* Three byte instructions */
#define WGPRDPT		0x60	/* Write EGPRDPT */
#define RGPRDPT		0x62	/* Read EGPRDPT */
#define WRXRDPT		0x64	/* Write ERXRDPT */
#define RRXRDPT		0x66	/* Read ERXRDPT */
#define WUDARDPT	0x68	/* Write EUDARDPT */
#define RUDARDPT	0x6A	/* Read EUDARDPT */
#define WGPWRPT		0x6C	/* Write EGPWRPT */
#define RGPWRPT		0x6E	/* Read EGPWRPT */
#define WRXWRPT		0x70	/* Write ERXWRPT */
#define RRXWRPT		0x72	/* Read ERXWRPT */
#define WUDAWRPT	0x74	/* Write EUDAWRPT */
#define RUDAWRPT	0x76	/* Read EUDAWRPT */

/* n byte instructions */
#define RCR(addr)	(0x00 | (addr & ADDR_MASK))	/* Read Control Register */
#define WCR(addr)	(0x40 | (addr & ADDR_MASK))	/* Write Control Register */
#define RCRU		0x20	/* Read Control Register Unbanked */
#define WCRU		0x22	/* Write Control Register Unbanked */
#define BFS(addr)	(0x80 | (addr & ADDR_MASK))	/* Bit Field Set */
#define BFC(addr)	(0xA0 | (addr & ADDR_MASK))	/* Bit Field Clear */
#define BFSU		0x24	/* Bit Field Set Unbanked */
#define BFCU		0x26	/* Bit Field Clear Unbanked */
#define RGPDATA		0x28	/* Read EGPDATA */
#define WGPDATA		0x2A	/* Write EGPDATA */
#define RRXDATA		0x2C	/* Read ERXDATA */
#define WRXDATA		0x2E	/* Write ERXDATA */
#define RUDADATA	0x30	/* Read EUDADATA */
#define WUDADATA	0x32	/* Write EUDADATA */

/* Register bit definitions */
/* MIREGADR */
#define MIREGADRH_WRITE	0x01
#define PHREG_SHIFT	0
#define PHREG_MASK	(0x1F << PHREG_SHIFT)

/* ECON2 */
#define ETHEN		(1 << 15)
#define STRCH		(1 << 14)
#define TXMAC		(1 << 13)
#define SHA1MD5		(1 << 12)
#define COCON3		(1 << 11)
#define COCON2		(1 << 10)
#define COCON1		(1 << 9)
#define COCON0		(1 << 8)
#define AUTOFC		(1 << 7)
#define TXRST		(1 << 6)
#define RXRST		(1 << 5)
#define ETHRST		(1 << 4)
#define MODLEN1		(1 << 3)
#define MODLEN0		(1 << 2)
#define AESLEN1		(1 << 1)
#define AESLEN0		(1 << 0)

/* EIDLED */
#define LACFG3		(1 << 15)
#define LACFG2		(1 << 14)
#define LACFG1		(1 << 13)
#define LACFG0		(1 << 12)
#define LBCFG3		(1 << 11)
#define LBCFG2		(1 << 10)
#define LBCFG1		(1 << 9)
#define LBCFG0		(1 << 8)
#define DEVID_SHIFT	5
#define DEVID_MASK	(0x7 << DEVID_SHIFT)
#define REVID_SHIFT	0
#define REVID_MASK	(0x1F << REVID_SHIFT)

/* MACON2 */
#define DEFER		(1 << 14)
#define BPEN		(1 << 13)
#define NOBKOFF		(1 << 12)
#define PADCFG2		(1 << 7)
#define PADCFG1		(1 << 6)
#define PADCFG0		(1 << 5)
#define TXCRCEN		(1 << 4)
#define PHDREN		(1 << 3)
#define HFRMEN		(1 << 2)
#define FULDPX		(1 << 0)

/* MABBIPG */
#define BBIPG_SHIFT	0
#define BBIPG_MASK	(0x3F << BBIPG_SHIFT)

/* MAIPG */
#define MAIPGH_WRITE	0x0C
#define IPG_SHIFT	0
#define IPG_MASK	(0x3F << IPG_SHIFT)

/* MALCON */
#define MALCONH_WRITE	0x37
#define MAXRET_SHIFT	0
#define MAXRET_MASK	(0xF << MAXRET_SHIFT)

#endif
