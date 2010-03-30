/*
 * SPI Slave Commands
 *
 * 2010 Michal Demin
 *
 */

#ifndef STM32BB_H_
#define STM32BB_H_

/* REG numbers 0x00-0x7f */
#define CMD_WRITE  0x80

/* global regs */
#define SYS_INTE   0x01 /* RW - 2 bytes */
#define SYS_INTF   0x02 /* RO - 2 bytes */
#define SYS_ID     0x03 /* RO - 2 bytes */
#define SYS_RESET  0x04 /* WO - 2 bytes */

/* can related regs */
#define CAN_STATUS 0x10 /* RW - 2 bytes */
#define CAN_CTRL   0x11 /* RW - 2 bytes */
#define CAN_TIMING 0x12 /* RW - 4 bytes */
#define CAN_TX     0x13 /* WO - 13bytes ?*/
#define CAN_RX0    0x14 /* RO - 13bytes ?*/
#define CAN_RX1    0x15 /* RO - 13bytes ?*/
#define CAN_ERR    0x16

/* power related regs*/
#define PWR_STATUS 0x20 /* RW - 2 bytes */
#define PWR_CTRL   0x21 /* RW - 2 bytes */
#define PWR_I_SET  0x22 /* RW - 4 bytes */ 
#define PWR_DATA   0x23 /* RO - 8 bytes */


#define SYS_INT_CANMASK  0x00ff
#define SYS_INT_CANTXIF  0x0001
#define SYS_INT_CANRX0IF 0x0002
#define SYS_INT_CANRX1IF 0x0004
#define SYS_INT_CANERRIF 0x0008
#define SYS_INT_CANRSTIF 0x0010

#define SYS_INT_PWRMASK  0x0f00
#define SYS_INT_PWRALARM 0x0100
#define SYS_INT_PWRAC    0x0200

#define SYS_RESET_MAGIC  0xBABE
#define SYS_ID_MAGIC     0xCAFE


/* control bit fields */
#define CAN_CTRL_INIT  0x0001 /* enter init mode */
#define CAN_CTRL_LOOP  0x0002 /* enable loopback mode */
#define CAN_CTRL_OSM   0x0004 /* enable One Shot Mode */
#define CAN_CTRL_SILM  0x0008 /* Listen only - silent mode */
#define CAN_CTRL_RST   0x0010 /* bxCan master reset */
#define CAN_CTRL_ABOM  0x0020 /* Automatic Bus-Off Management*/

/* status bit fields */
#define CAN_STAT_TXF   0x0001 /* TX fifo Full */
#define CAN_STAT_ALST  0x0004 /* Arbitration lost for last message */
#define CAN_STAT_TERR  0x0008 /* Transmit error for last message */
#define CAN_STAT_INAK  0x0010 /* Initialization ACK */
#define CAN_STAT_TXOK  0x0020 /* Transmit OK */
#define CAN_STAT_RX0   0x0F00 /* no of messages queued in RX0 */
#define CAN_STAT_RX1   0xF000 /* no of messages queued in RX1 */

/* error bit fields */
#define CAN_ERR_REC    0xFF000000 /* Receive error count */
#define CAN_ERR_TEC    0x00FF0000 /* Transmit error count */
#define CAN_ERR_LEC    0x00000070 /* Last error code */
#define CAN_ERR_BOFF   0x00000004 /* Bus-off */
#define CAN_ERR_EPVF   0x00000002 /* Error passive */
#define CAN_ERR_EWGF   0x00000001 /* Error warning */

/* LEC field */
#define CAN_LEC_NOERR  0x00 /* No Error */
#define CAN_LEC_STUFF  0x01 /* Stuff Error */
#define CAN_LEC_FORM   0x02 /* Form Error */
#define CAN_LEC_ACK    0x03 /* Ack Error */
#define CAN_LEC_BRE    0x04 /* Bit recessive Error */
#define CAN_LEC_BDE    0x05 /* Bit dominant Error */
#define CAN_LEC_CRC    0x06 /* CRC Error */
#define CAN_LEC_SW     0x07 /* Set by software */

#define CAN_MSG_SIZE	0x0F // DLC[0:3]
#define CAN_MSG_RTR   0x10 // RTR[4]
#define CAN_MSG_EID   0x20 // EID[5]
#define CAN_MSG_INV   0x40 // invalid message flag


#define PWR_CTRL_ADC 0x0001 /* Enable ADC sampling */
#define PWR_CTRL_PWM 0x0002 /* Enable PWM output for Current setting */
#define PWR_CTRL_EN  0x0004 /* Charger enable 1 = ENABLED */
#define PWR_CTRL_ACS 0x0008 /* AC select 1 = AC, 0 = BAT */

#define PWR_STAT_ALARM 0x0001 /* ALARM state */
#define PWR_STAT_ACPRE 0x0002 /* AC present */

struct stm32bb_pwr_t {
	uint16_t i_bat;
	uint16_t i_sys;
	uint16_t v_bat;
	uint16_t v_ac;
};

struct stm32bb_iset_t {
	uint16_t i_ac;
	uint16_t i_bat;
};
							 
#endif

