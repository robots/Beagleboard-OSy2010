/*#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "platform.h"
#include "stm32f10x.h"
*/

#include "canbuf.h"

void CANBuf_Init(struct can_buffer_t *b) {
	b->write = 0;
	b->read = 0;
}

uint16_t CANBuf_GetAvailable(struct can_buffer_t *b) {
	int32_t out = b->write - b->read;

	if (out < 0) {
		out += CAN_BUFFER_SIZE;
	}
	return (uint16_t)out;
}

struct can_message_t *CANBuf_GetReadAddr(struct can_buffer_t *b) {
	return &b->msgs[read];
}

/* this should be called after message is read */
void CANBuf_ReadDone(struct can_buffer_t *b) {
	/* if there is something more to read -> advance */
	if (b->read < b->write) { 
		b->read++;
	}
}

struct can_message_t *CANBuf_GetNextWriteAddr(struct can_buffer_t *b) {
	return &b->msgs[write];
}

void CANBuf_Written(struct can_buffer_t *b) {
	b->write ++;
	b->write %= CAN_BUFFER_SIZE;

	/* we are dropping old messages in favior of new ones */
	if (b->write == b->read) {
		b->read++;
	}
}

