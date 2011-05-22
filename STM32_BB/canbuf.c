/*
 * Ring buffer for CAN messages
 *
 * 2010 Michal Demin
 *
 */

#include "platform.h"
#include "stm32f10x.h"

#include "canbuf.h"

/**
 * Initialize CAN message buffer
 * \param b pointer to buffer structure to be initialized
 */
void CANBuf_Init(struct can_buffer_t *b)
{
	uint16_t i;
	b->write = 0;
	b->read = 0;

	for (i = 0; i < CAN_BUFFER_SIZE; i++ ) {
		b->msgs[i].flags = CAN_MSG_INV;
	}
}

/**
 * Checks whether CAN message buffer is empty.
 * \param b pointer to buffer structure.
 * \return 1 for empty, 0 otherwise.
 */
uint8_t CANBuf_Empty(struct can_buffer_t *b)
{
	if ( b->write == b->read) {
		return 1;
	}
	return 0;
}

/**
 * Returns available messages in CAN message buffer.
 * \param b pointer to buffer structure.
 * \return number of messages available.
 */
uint16_t CANBuf_GetAvailable(struct can_buffer_t *b)
{
	uint16_t out;

	if (b->read > b->write) {
		out = b->read - b->write;
		out = CAN_BUFFER_SIZE - out;
	} else {
		out = b->write - b->read;
	}
	return out;
}

/**
 * Returns address of cell pointed to by read pointer.
 * \param b pointer to buffer structure.
 * \return pointer to can_message_t structure.
 */
struct can_message_t *CANBuf_GetReadAddr(struct can_buffer_t *b)
{
	return &b->msgs[b->read];
}

/**
 * Returns address of cell pointed to by write pointer.
 * \param b pointer to buffer structure.
 * \return pointer to can_message_t structure.
 */
struct can_message_t *CANBuf_GetWriteAddr(struct can_buffer_t *b)
{
	return &b->msgs[b->write];
}

/**
 * Advances read pointer in CAN buffer. Should be called after message is read
 * \param b pointer to buffer structure.
 */
void CANBuf_ReadDone(struct can_buffer_t *b)
{
	// if there is something more to read -> advance
	if (!CANBuf_Empty(b)) { 
		b->read++;
		b->read %= CAN_BUFFER_SIZE;
	}
}

/**
 * Advances write pointer in CAN buffer. Should be called after message is read
 * \param b pointer to buffer structure.
 */
void CANBuf_WriteDone(struct can_buffer_t *b)
{
	// when the buffer is full, do not advance "writer head"
	// this avoids the need to update CanController_RXx pointer (possible race)
	if (((b->write + 1) % CAN_BUFFER_SIZE) == b->read) {
		return;
	}

	b->write ++;
	b->write %= CAN_BUFFER_SIZE;
}
