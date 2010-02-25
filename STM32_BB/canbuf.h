#ifndef CANBUF_H_
#define CANBUF_H_

#include "cancontroller.h"

#define CAN_BUFFER_SIZE 10

struct can_buffer_t {
	struct can_message_t msgs[CAN_BUFFER_SIZE];
	uint16_t read;
	uint16_t write;
};

void CANBuf_Init(struct can_buffer_t *);

uint8_t CANBuf_Empty(struct can_buffer_t *b) {
uint16_t CANBuf_GetAvailable(struct can_buffer_t *b);

struct can_message_t *CANBuf_GetReadAddr(struct can_buffer_t *b);
void CANBuf_ReadDone(struct can_buffer_t *b);

struct can_message_t *CANBuf_GetNextWriteAddr(struct can_buffer_t *b);
void CANBuf_Written(struct can_buffer_t *b);


#endif

