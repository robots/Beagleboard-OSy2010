#ifndef CANCONTROLLER_H_
#define CANCONTROLLER_H_


struct can_timing_t {
	uint16_t brp;
	uint16_t ts; //swj[9:8] ts2[6:4] ts1[3:0]
} __attribute__ ((packed));

#define CAN_MSG_SIZE	0x0B
#define CAN_MSG_RTR   0x08
#define CAN_MSG_EID   0x10

struct can_message_t {
	uint8_t flags;
	uint8_t id[4];
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

