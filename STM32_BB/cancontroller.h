#ifndef CANCONTROLLER_H_
#define CANCONTROLLER_H_

/* status bit fields */
#define CAN_STATUS_TXE   0x0001
#define CAN_STATUS_RXNE  0x0002

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

void CANController_Rx1Handle(void);
void CANController_Rx0Handle(void);
void CANController_TxHandle(void);
void CANController_ControlHandle(void);
void CANController_TimingHandle(void);

extern uint16_t CANController_Status; 

extern struct can_timing_t CANController_Timing;

extern struct can_message_t *CANController_RX0;
extern struct can_message_t *CANController_RX1;
extern struct can_message_t *CANController_TX;

#endif

