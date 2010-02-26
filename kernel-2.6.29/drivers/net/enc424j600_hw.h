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
 */
#define ADDR_MASK	0x1F
#define BANK_MASK	0x60

/* All-bank registers */
#define EUDASTL		0x16
#define EUDASTH		0x17
#define EUDANDL		0x18
#define EUDANDH		0x19
#define ESTATL		0x1A
#define ESTATH		0x1B
#define EIRL		0x1C
#define EIRH		0x1D
#define ECON1L		0x1E
#define ECON1H		0x1F

/* Bank 0 registers */
#define ETXSTL		(0x00 | 0x00)
#define ETXSTH		(0x01 | 0x00)
#define ETXLENL		(0x02 | 0x00)
#define ETXLENH		(0x03 | 0x00)
#define ERXSTL		(0x04 | 0x00)
#define ERXSTH		(0x05 | 0x00)
#define ERXTAILL	(0x06 | 0x00)
#define ERXTAILH	(0x07 | 0x00)
#define ERXHEADL	(0x08 | 0x00)
#define ERXHEADH	(0x09 | 0x00)
#define EDMASTL		(0x0A | 0x00)
#define EDMASTH		(0x0B | 0x00)
#define EDMALENL	(0x0C | 0x00)
#define EDMALENH	(0x0D | 0x00)
#define EDMADSTL	(0x0E | 0x00)
#define EDMADSTH	(0x0F | 0x00)
#define EDMACSL		(0x10 | 0x00)
#define EDMACSH		(0x11 | 0x00)
#define ETXSTATL	(0x12 | 0x00)
#define ETXSTATH	(0x13 | 0x00)
#define ETXWIREL	(0x14 | 0x00)
#define ETXWIREH	(0x15 | 0x00)

/* Bank 1 registers */
#define EHT1L		(0x00 | 0x20)
#define EHT1H		(0x01 | 0x20)
#define EHT2L		(0x02 | 0x20)
#define EHT2H		(0x03 | 0x20)
#define EHT3L		(0x04 | 0x20)
#define EHT3H		(0x05 | 0x20)
#define EHT4L		(0x06 | 0x20)
#define EHT4H		(0x07 | 0x20)
#define EPMM1L		(0x08 | 0x20)
#define EPMM1H		(0x09 | 0x20)
#define EPMM2L		(0x0A | 0x20)
#define EPMM2H		(0x0B | 0x20)
#define EPMM3L		(0x0C | 0x20)
#define EPMM3H		(0x0D | 0x20)
#define EPMM4L		(0x0E | 0x20)
#define EPMM4H		(0x0F | 0x20)
#define EPMCSL		(0x10 | 0x20)
#define EPMCSH		(0x11 | 0x20)
#define EPMOL		(0x12 | 0x20)
#define EPMOH		(0x13 | 0x20)
#define ERXFCONL	(0x14 | 0x20)
#define ERXFCONH	(0x15 | 0x20)

/* Bank 2 registers */
#define MACON1L		(0x00 | 0x40)
#define MACON1H		(0x01 | 0x40)
#define MACON2L		(0x02 | 0x40)
#define MACON2H		(0x03 | 0x40)
#define MABBIPGL	(0x04 | 0x40)
#define MABBIPGH	(0x05 | 0x40)
#define MAIPGL		(0x06 | 0x40)
#define MAIPGH		(0x07 | 0x40)
#define MACLCONL	(0x08 | 0x40)
#define MACLCONH	(0x09 | 0x40)
#define MAMXFLL		(0x0A | 0x40)
#define MAMXFLH		(0x0B | 0x40)
#define MICMDL		(0x12 | 0x40)
#define MICMDH		(0x13 | 0x40)
#define MIREGADRL	(0x14 | 0x40)
#define MIREGADRH	(0x15 | 0x40)

/* Bank 3 registers */
#define MAADR3L		(0x00 | 0x60)
#define MAADR3H		(0x01 | 0x60)
#define MAADR2L		(0x02 | 0x60)
#define MAADR2H		(0x03 | 0x60)
#define MAADR1L		(0x04 | 0x60)
#define MAADR1H		(0x05 | 0x60)
#define MIWRL		(0x06 | 0x60)
#define MIWRH		(0x07 | 0x60)
#define MIRDL		(0x08 | 0x60)
#define MIRDH		(0x09 | 0x60)
#define MISTATL		(0x0A | 0x60)
#define MISTATH		(0x0B | 0x60)
#define EPAUSL		(0x0C | 0x60)
#define EPAUSH		(0x0D | 0x60)
#define ECON2L		(0x0E | 0x60)
#define ECON2H		(0x0F | 0x60)
#define ERXWML		(0x10 | 0x60)
#define ERXWMH		(0x11 | 0x60)
#define EIEL		(0x12 | 0x60)
#define EIEH		(0x13 | 0x60)
#define EIDLEDL		(0x14 | 0x60)
#define EIDLEDH		(0x15 | 0x60)

/* Unbanked registers */
#define EGPDATA		(0x00 | 0x80)
#define ERXDATA		(0x02 | 0x80)
#define EUDADATA	(0x04 | 0x80)
#define EGPRDPTL	(0x06 | 0x80)
#define EGPRDPTH	(0x07 | 0x80)
#define EGPWRPTL	(0x08 | 0x80)
#define EGPWRPTH	(0x09 | 0x80)
#define ERXRDPTL	(0x0A | 0x80)
#define ERXRDPTH	(0x0B | 0x80)
#define ERXWRPTL	(0x0C | 0x80)
#define ERXWRPTH	(0x0D | 0x80)
#define EUDARDPTL	(0x0E | 0x80)
#define EUDARDPTH	(0x0F | 0x80)
#define EUDAWRPTL	(0x10 | 0x80)
#define EUDAWRPTH	(0x11 | 0x80)

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
/* ESTATH */
#define INT		(1 << 7)
#define FCIDLE		(1 << 6)
#define RXBUSY		(1 << 5)
#define CLOCKRDY	(1 << 4)
#define PHYDPX		(1 << 2)
#define PHYLNK		(1 << 0)

/* EIRH */
#define CRYPTEN		(1 << 7)
#define MODEXIF		(1 << 6)
#define HASHIF		(1 << 5)
#define AESIF		(1 << 4)
#define LINKIF		(1 << 3)

/* EIRL */
#define PKTIF		(1 << 6)
#define DMAIF		(1 << 5)
#define TXIF		(1 << 3)
#define TXABTIF		(1 << 2)
#define RXABTIF		(1 << 1)
#define PCFULIF		(1 << 0)

/* ECON1H */
#define MODEXST		(1 << 7)
#define HASHEN		(1 << 6)
#define HASHOP		(1 << 5)
#define HASHLST		(1 << 4)
#define AESST		(1 << 3)
#define AESOP1		(1 << 2)
#define AESOP0		(1 << 1)
#define PKTDEC		(1 << 0)

/* ECON1L */
#define FCOP1		(1 << 7)
#define FCOP0		(1 << 6)
#define DMAST		(1 << 5)
#define DMACPY		(1 << 4)
#define DMACSSD		(1 << 3)
#define DMANOCS		(1 << 2)
#define TXRTS		(1 << 1)
#define RXEN		(1 << 0)

/* ETXSTATH */
#define LATECOL		(1 << 2)
#define MAXCOL		(1 << 1)
#define EXDEFER		(1 << 0)

/* ETXSTATL */
#define DEFER		(1 << 7)
#define CRCBAD		(1 << 4)
#define COLCNT_MASK	0xF

/* ERXFCONH */
#define HTEN		(1 << 7)
#define MPEN		(1 << 6)
#define NOTPM		(1 << 4)
#define PMEN3		(1 << 3)
#define PMEN2		(1 << 2)
#define PMEN1		(1 << 1)
#define PMEN0		(1 << 0)

/* ERXFCONL */
#define CRCEEN		(1 << 7)
#define CRCEN		(1 << 6)
#define RUNTEEN		(1 << 5)
#define RUNTEN		(1 << 4)
#define UCEN		(1 << 3)
#define NOTMEEN		(1 << 2)
#define MCEN		(1 << 1)
#define BCEN		(1 << 0)

/* MACON1L */
#define LOOPBK		(1 << 4)
#define RXPAUS		(1 << 2)
#define PASSALL		(1 << 1)

/* MACON2H */
#define DEFER		(1 << 6)
#define BPEN		(1 << 5)
#define NOBKOFF		(1 << 4)

/* MACON2L */
#define PADCFG2		(1 << 7)
#define PADCFG1		(1 << 6)
#define PADCFG0		(1 << 5)
#define TXCRCEN		(1 << 4)
#define PHDREN		(1 << 3)
#define HFRMEN		(1 << 2)
#define FULDPX		(1 << 0)

/* MICMDL */
#define MIISCAN		(1 << 1)
#define MIIRD		(1 << 0)

/* MISTATL */
#define NVALID		(1 << 2)
#define SCAN		(1 << 1)
#define BUSY		(1 << 0)

/* ECON2H */
#define ETHEN		(1 << 7)
#define STRCH		(1 << 6)
#define TXMAC		(1 << 5)
#define SHA1MD5		(1 << 4)
#define COCON3		(1 << 3)
#define COCON2		(1 << 2)
#define COCON1		(1 << 1)
#define COCON0		(1 << 0)

/* ECON2L */
#define AUTOFC		(1 << 7)
#define TXRST		(1 << 6)
#define RXRST		(1 << 5)
#define ETHRST		(1 << 4)
#define MODLEN1		(1 << 3)
#define MODLEN0		(1 << 2)
#define AESLEN1		(1 << 1)
#define AESLEN0		(1 << 0)

/* EIEH */
#define INTIE		(1 << 7)
#define MODEXIE		(1 << 6)
#define HASHIE		(1 << 5)
#define AESIE		(1 << 4)
#define LINKIE		(1 << 3)

/* EIEL */
#define PKTIE		(1 << 6)
#define DMAIE		(1 << 5)
#define TXIE		(1 << 3)
#define TXABTIE		(1 << 2)
#define RXABTIE		(1 << 1)
#define PCFULIE		(1 << 0)

/* EIDLEDH */
#define LACFG3		(1 << 7)
#define LACFG2		(1 << 6)
#define LACFG1		(1 << 5)
#define LACFG0		(1 << 4)
#define LBCFG3		(1 << 3)
#define LBCFG2		(1 << 2)
#define LBCFG1		(1 << 1)
#define LBCFG0		(1 << 0)

/* EIDLEDL */
#define DEVID_SHIFT	5
#define DEVID_MASK	(0x7 << DEVID_SHIFT)
#define REVID_SHIFT	0
#define REVID_MASK	(0x1F << REVID_SHIFT)


/* SRAM size */
#define ENC424J600_SRAM_END	0x5FFF

/* Configuration */

/* maximum ethernet frame length */
#define MAX_FRAMELEN		1518

#endif
