/*
 * bxCAN driver for STM32 family processors
 *
 * 2010 Michal Demin
 *
 */

#ifndef CANCONTROLLER_H_
#define CANCONTROLLER_H_

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



struct can_timing_t {
	uint16_t brp; // brp[0:9]
	uint16_t ts; // res[15] lbkm[14] res[13:10] swj[9:8] res[7] ts2[6:4] ts1[3:0]
} __attribute__ ((packed));

#define CAN_MSG_SIZE	0x0F // DLC[0:3]
#define CAN_MSG_RTR   0x10 // RTR[4]
#define CAN_MSG_EID   0x20 // EID[5]

struct can_message_t {
	uint8_t flags;
	uint32_t id;
	uint8_t data[8];
} __attribute__ ((packed));

void CANController_Init(void);
void CANController_Rx1Handle(void);
void CANController_Rx0Handle(void);
void CANController_TxHandle(void);
void CANController_ControlHandle(void);
void CANController_TimingHandle(void);

extern volatile uint32_t CANController_Error; 
extern volatile uint16_t CANController_Status; 
extern volatile uint16_t CANController_Control; 

extern volatile struct can_timing_t CANController_Timing;

extern volatile struct can_message_t *CANController_RX0;
extern volatile struct can_message_t *CANController_RX1;
extern volatile struct can_message_t *CANController_TX;

#endif

