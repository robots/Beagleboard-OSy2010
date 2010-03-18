/*
 * Ring buffer for CAN messages
 *
 * 2010 Michal Demin
 *
 */

#include "FreeRTOS.h"
/*#include "task.h"
#include "queue.h"
#include "semphr.h"
*/
#include "platform.h"
#include "stm32f10x.h"


#include "canbuf.h"

void CANBuf_Init(struct can_buffer_t *b) {
	b->write = 0;
	b->read = 0;
}

uint8_t CANBuf_Empty(struct can_buffer_t *b) {
	if ( b->write == b->read) {
		return 1;
	}
	return 0;
}

uint16_t CANBuf_GetAvailable(struct can_buffer_t *b) {
	uint16_t out;

	if (b->read > b->write) {
		out = b->read - b->write;
		out = CAN_BUFFER_SIZE - out;
	} else {
		out = b->write - b->read;
	}
	return (uint16_t)out;
}

struct can_message_t *CANBuf_GetReadAddr(struct can_buffer_t *b) {
	return &b->msgs[b->read];
}

/* this should be called after message is read */
void CANBuf_ReadDone(struct can_buffer_t *b) {
	// if there is something more to read -> advance
	if (CANBuf_GetAvailable(b)>0) { 
		b->read++;
		b->read %= CAN_BUFFER_SIZE;
	}
}

struct can_message_t *CANBuf_GetNextWriteAddr(struct can_buffer_t *b) {
	return &b->msgs[b->write];
}

void CANBuf_Written(struct can_buffer_t *b) {
/*	b->write ++;
	b->write %= CAN_BUFFER_SIZE;

	// we are dropping old messages in favior of new ones
	if (b->write == b->read) {
		b->read++;
		b->read %= CAN_BUFFER_SIZE;
	}
*/
	// when the buffer is full, do not advance "writer head"
	// this avoids the need to update CanController_RXx pointer (possible race)
	if (((b->write + 1) % CAN_BUFFER_SIZE) == b->read) {
		return;
	}

	b->write ++;
	b->write %= CAN_BUFFER_SIZE;
}

