/*
 * Ring buffer for CAN messages
 *
 * 2010 Michal Demin
 *
 */

#ifndef CANBUF_H_
#define CANBUF_H_

#include "cancontroller.h"

/* DO NOT CHANGE !!!! CAN_STATUS depends on this !!!! */
#define CAN_BUFFER_SIZE 16

struct can_buffer_t {
	struct can_message_t msgs[CAN_BUFFER_SIZE];
	uint16_t read;
	uint16_t write;
} __attribute__ ((packed));


void CANBuf_Init(struct can_buffer_t *);

uint8_t CANBuf_Empty(struct can_buffer_t *b);
uint16_t CANBuf_GetAvailable(struct can_buffer_t *b);

struct can_message_t *CANBuf_GetReadAddr(struct can_buffer_t *b);
void CANBuf_ReadDone(struct can_buffer_t *b);

struct can_message_t *CANBuf_GetWriteAddr(struct can_buffer_t *b);
void CANBuf_WriteDone(struct can_buffer_t *b);


#endif

