#ifndef CANCONTROLLER_H_
#define CANCONTROLLER_H_


#define CAN_CMD_WRITE 0x40

// commands
#define CAN_STATUS 0x01 // r, 2
#define CAN_CTRL   0x02 // w, 2
#define CAN_TIMING 0x03 // rw, 4
#define CAN_INTE   0x04 // rw, 2
#define CAN_INTF   0x05 // r, 2
#define CAN_TX     0x06 // w, 13
#define CAN_RX0    0x07 // r, 13
#define CAN_RX1    0x08 // r, 13

// TODO: filters

struct can_timing_t {
	uint16_t brp;
	uint16_t ts; //swj[9:8] ts2[6:4] ts1[3:0]
}


void CANController_ISR(uint8_t);

extern struct can_timing_t CANController_Timing;

#endif

