/*
 * SPI Slave Commands
 *
 * 2010 Michal Demin
 *
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

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
#define CAN_TX     0x13 /* WO - 13 bytes */
#define CAN_RX0    0x14 /* RO - 13 bytes */
//#define CAN_RX1    0x15 /* RO - 13 bytes */
#define CAN_ERR    0x16 /* RO - 4 bytes*/

/* power related regs*/
#define PWR_STATUS 0x20 /* RW - 2 bytes */
#define PWR_CTRL   0x21 /* RW - 2 bytes */
#define PWR_I_SET  0x22 /* RW - 4 bytes */
#define PWR_DATA   0x23 /* RO - 8 bytes */

#endif

