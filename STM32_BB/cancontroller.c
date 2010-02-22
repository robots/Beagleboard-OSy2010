#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "platform.h"
#include "stm32f10x.h"

#include "cancontroller.h"

uint16_t CANController_Status; 

struct can_timing_t CANController_Timing;

struct can_message_t *CANController_RX0;
struct can_message_t *CANController_RX1;
struct can_message_t *CANController_TX;

struct can_buffer_t CANController_RX0Buffer;
struct can_buffer_t CANController_RX1Buffer;
struct can_buffer_t CANController_TXBuffer;


void CANController_Init(void) {
	CANBuf_Init(&CANController_RX0Buffer);
	CANBuf_Init(&CANController_RX1Buffer);
	CANBuf_Init(&CANController_TXBuffer);

	CANController_RX0 = CANBuf_ReadAddr(&CANController_RX0Buffer);
	CANController_RX1 = CANBuf_ReadAddr(&CANController_RX1Buffer);
	CANController_TX = CANBuf_GetNextWriteAddr(&CANController_TXBuffer);
}

/* message has been read from RX0, shift the buffer */
void CANController_Rx0Handle(void) {
	CANBuf_ReadDone(CANController_RX0);
	CANController_RX0 = CANBuf_ReadAddr(&CANController_RX0Buffer);
}

/* message has been read from RX1, shift the buffer */
void CANController_Rx1Handle(void) {
	CANBuf_ReadDone(CANController_RX1);
	CANController_RX1 = CANBuf_ReadAddr(&CANController_RX1Buffer);
}

/* new message to be transmitted */
void CANController_TxHandle(void) {
	/* Move the ring buffer */
	CANBuf_Written(&CANController_TXBuffer);
	CANController_TX = CANBuf_GetNextWriteAddr(&CANController_TXBuffer);

	/* Wake up CAN worker */
	xSemaphoreGive(xCAN_Sem);

}

void CANController_ControlHandle(void) {

}

/* Changes the timming in the bxCAN module */
void CANController_TimingHandle(void) {

}

