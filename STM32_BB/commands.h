#ifndef COMMANDS_H_
#define COMMANDS_H_

/* REG numbers 0x00-0x3f */

/* global regs */
#define SYS_INTE   0x01 /* RW */
#define SYS_INTF   0x02 /* RO */

/* can related regs */
#define CAN_STATUS 0x10 /* RW */
#define CAN_CTRL   0x11 /* RW */
#define CAN_TIMING 0x12 /* RW */
#define CAN_TX     0x13 /* WO */
#define CAN_RX0    0x14 /* RO */
#define CAN_RX1    0x15 /* RO */
#define CAN_ERR    0x16

/* power related regs*/
#define PWR_STATUS 0x20 /* RW */
#define PWR_CTRL   0x21 /* RW */
#define PWR_I_SET  0x22 /* RW */ 
#define PWR_DATA   0x23 /* RO */

#endif

