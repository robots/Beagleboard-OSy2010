/*
 * SPI Slave Commands
 *
 * 2010 Michal Demin
 *
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

/* REG numbers 0x00-0x7f */
/** Write cmd flag */
#define CMD_WRITE  0x80

/* global regs */
/** Interrupt enable - RW (2bytes) */
#define SYS_INTE   0x01
/** Interrupt pending flag - RO (2bytes) - read clears */
#define SYS_INTF   0x02
/** System ID - RO (2bytes)*/
#define SYS_ID     0x03
/** System reset - WO (2bytes) */
#define SYS_RESET  0x04

/* can related regs */
/** CAN status register - RO (2bytes) */
#define CAN_STATUS 0x10
/** CAN control register - RW (2bytes) */
#define CAN_CTRL   0x11
/** CAN timing register - RW (4bytes) */
#define CAN_TIMING 0x12
/** CAN Tx mailbox - WO (13bytes) */
#define CAN_TX     0x13
/** CAN RX mailbox - RO (16*13bytes) */
#define CAN_RX0    0x14 
/** CAN Error register - RO (4bytes) */
#define CAN_ERR    0x16

/* power related regs*/
/** Power status register - RW (2bytes) */
#define PWR_STATUS 0x20
/** Power control register - RW (2bytes) */
#define PWR_CTRL   0x21
/** Current setting register - RW (4bytes) */
#define PWR_I_SET  0x22
/** Measurement data register - RO (8bytes) */
#define PWR_DATA   0x23

#endif
