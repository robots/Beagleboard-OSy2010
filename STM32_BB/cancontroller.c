#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "platform.h"
#include "stm32f10x.h"

#include "cancontroller.h"


uint16_t CANController_Status = 0x00;
uint16_t CANController_Status_Last = 0x00; 
uint16_t CANController_Control = 0x00 ;
uint16_t CANController_Control_Last = 0x00;
uint32_t CANController_Error = 0x00;

struct can_timing_t CANController_Timing;

struct can_message_t *CANController_RX0;
struct can_message_t *CANController_RX1;
struct can_message_t CANController_TXBuffer;
struct can_message_t *CANController_TX;

struct can_buffer_t CANController_RX0Buffer;
struct can_buffer_t CANController_RX1Buffer;

static void CANController_Rx0Receive(void) __attribute__ ((naked));
static void CANController_Rx1Receive(void) __attribute__ ((naked));

void CANController_Init(void) {
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	CANBuf_Init(&CANController_RX0Buffer);
	CANBuf_Init(&CANController_RX1Buffer);

	CANController_RX0 = CANBuf_ReadAddr(&CANController_RX0Buffer);
	CANController_RX1 = CANBuf_ReadAddr(&CANController_RX1Buffer);
	CANController_TX = &CANController_TXBuffer;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN pin: TX */   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	CAN_DeInit();
}

/* message has been read from RX0, shift the buffer */
void CANController_Rx0Handle(void) {
	CANBuf_ReadDone(&CANController_RX0Buffer);
	CANController_RX0 = CANBuf_ReadAddr(&CANController_RX0Buffer);
}

/* should be called from ISR */
static void CANController_Rx0Receive(void) {
	static int offset = CANBuf_GetNextWriteAddr(&CANController_RX0Buffer); 
	static struct can_message_t *RX0 = &CANController_RX0Buffer[offset];

	RX0->flags = 0x00;

	/* check RTR */
	if (CAN1->sFIFOMailBox[0].RIR & 0x02) {
		RX0->flags |= CAN_MSG_RTR;
	}

	/* update DLC */
	RX0->flags |= CAN1->sFIFOMailBox[0].RIR & 0x0F;

	/* copy msg ID */
	if (CAN1->sFIFOMailBox[0].RIR & 0x04) { // extended id
		RX0->flags |= CAN_MSG_EID;
		RX0->id = (uint32_t)0x1FFFFFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
	} else { // standard id
		RX0->id = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
	}

	/* copy data bytes */
  RX0->data[0] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR);
  RX0->data[1] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  RX0->data[2] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  RX0->data[3] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 24);
  RX0->data[4] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR);
  RX0->data[5] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  RX0->data[6] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  RX0->data[7] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 24);

	/* release fifo */
	CAN1->RF0R = RF0R_RFOM0;

	if (CANBuf_Empty(&CANController_RX0Buffer) == 1) {
		/* Move the ring buffer */
		CANBuf_Written(&CANController_RX0Buffer);
		
		CANController_RX0 = RX0;

		/* notify host by interrupt */
		SYS_ChangeIntFlag(SYS_INT_CANRX1IF);

	} else {
		/* Move the ring buffer */
		CANBuf_Written(&CANController_RX0Buffer);
		SYS_ChangeIntFlag(SYS_INT_CANRX1IF);
	}
}

/* message has been read from RX1, shift the buffer */
void CANController_Rx1Handle(void) {
	CANBuf_ReadDone(&CANController_RX1Buffer);
	CANController_RX1 = CANBuf_ReadAddr(&CANController_RX1Buffer);
}

/* should be called from ISR */
static void CANController_Rx1Receive(void) {
	static int offset = CANBuf_GetNextWriteAddr(&CANController_RX1Buffer); 
	static struct can_message_t *RX1 = &CANController_RX1Buffer[offset];

	RX1->flags = 0x00;

	/* check RTR */
	if (CAN1->sFIFOMailBox[1].RIR & 0x02) {
		RX1->flags |= CAN_MSG_RTR;
	}

	/* update DLC */
	RX1->flags |= CAN1->sFIFOMailBox[1].RIR & 0x0F;

	/* copy msg ID */
	if (CAN1->sFIFOMailBox[1].RIR & 0x04) { // extended id
		RX1->flags |= CAN_MSG_EID;
		RX1->id = (uint32_t)0x1FFFFFFF & (CAN1->sFIFOMailBox[1].RIR >> 3);
	} else { // standard id
		RX1->id = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[1].RIR >> 21);
	}

	/* copy data bytes */
  RX1->data[0] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDLR);
  RX1->data[1] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDLR >> 8);
  RX1->data[2] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDLR >> 16);
  RX1->data[3] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDLR >> 24);
  RX1->data[4] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDHR);
  RX1->data[5] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDHR >> 8);
  RX1->data[6] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDHR >> 16);
  RX1->data[7] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[1].RDHR >> 24);

	/* release fifo */
	CAN1->RF1R = RF1R_RFOM1;

	if (CANBuf_Empty(&CANController_RX1Buffer) == 1) {
		/* Move the ring buffer */
		CANBuf_Written(&CANController_RX1Buffer);
		
		CANController_RX1 = RX1;

		/* notify host by interrupt */
		SYS_ChangeIntFlag(SYS_INT_CANRX1IF);
	} else {
		/* Move the ring buffer */
		CANBuf_Written(&CANController_RX1Buffer);
		SYS_ChangeIntFlag(SYS_INT_CANRX1IF);
	}
}
/* new message to be transmitted */
void CANController_TxHandle(void) {
	static uint16_t mailbox = 4;
	if (CAN1->TSR & TSR_TME0) {
		mailbox = 0;
	} else if (CAN1->TSR & TSR_TME1) {
		mailbox = 1;
	} else if (CAN1->TSR & TSR_TME2) {
		mailbox = 2;
	} else {
		/* nothing empty ? :( this should not happen ! */
		return;
	}

	/* clear message */
	CAN1->sTxMailBox[mailbox].TIR &= TMIDxR_TXRQ;
	
	/* add IDE and RTR fields */
	CAN1->sTxMailBox[mailbox].TIR |= (CANController_TX->flags >> 3) & 0x06;
	
	/* setup the DLC field */
	CAN1->sTxMailBox[mailbox].TDTR &= 0xFFFFFFF0;
	CAN1->sTxMailBox[mailbox].TDTR |= (CANController_TX->flags & CAN_MSG_SIZE);
	

	/* Set up the data fields */
	CAN1->sTxMailBox[mailbox].TDLR = (((uint32_t)CANController_TX->data[3] << 24) | 
																		((uint32_t)CANController_TX->data[2] << 16) |
																		((uint32_t)CANController_TX->data[1] << 8) | 
																		((uint32_t)CANController_TX->data[0]));
	CAN1->sTxMailBox[mailbox].TDHR = (((uint32_t)CANController_TX->data[7] << 24) | 
																		((uint32_t)CANController_TX->data[6] << 16) |
																		((uint32_t)CANController_TX->data[5] << 8) |
																		((uint32_t)CANController_TX->data[4]));

	/* mark message for transmission */
	CAN1->sTxMailBox[mailbox].TIR |= TMIDxR_TXRQ;

	/* if no TX mailbox empty signal host */
	if (!(CAN1->TSR & (TSR_TME0 | TSR_RME1 | TSR_TME2))) {
		CANController_Status |= CAN_STAT
	}
}

void CANController_ControlHandle(void) {
	/* select the changed bits */
	uint16_t change = CANController_Control_Last ^ CANController_Control;

	if (change & CAN_CTRL_INIT) {
		if (CANController_Control & CAN_CTRL_INIT) {
			CAN1->MCR |= MCR_INRQ;
		} else {
			CAN1->MCR &= ~MCR_INRQ;
		}
	} else if (change & CAN_CTRL_LOOP) {

	}

	CANController_Control_Last = CANController_Control;
}

/* Changes the timming in the bxCAN module */
void CANController_TimingHandle(void) {

	/* Test if CAN1 is in initialization mode */
	if ((CAN1->MCR & MCR_INRQ) && (CAN1->MSR & MSR_INAK)) {
		/* apply the settings */
		CANController_Timing->ts &= 0x437F; // this removes any junk, reserved/not used
		CANController_Timing->brp &= 0x3FF; // this removes any junk, reserved/not used

		CAN1->BTR = (CANController_Timing->ts << 16) | CANController_Timing->brp;
	}
}

/* TX interrupt */
void USB_HP_CAN1_TX_IRQHandler(void) {
	if (CAN1->TSR & TSR_RQCP2) {
		CAN1->TSR = TSR_RQCP2;
	}
	if (CAN1->TSR & TSR_RQCP1) {
		CAN1->TSR = TSR_RQCP1;
	}
	if (CAN1->TSR & TSR_RQCP0) {
		CAN1->TSR = TSR_RQCP0;
	}
	/* notify host that message was sent ! */
	SYS_ChangeIntFlag(SYS_INT_CANTXIF);
}

/* RX0 fifo interrupt */
void USB_LP_CAN1_RX0_IRQHandler(void) {
	CANController_Rx0Receive();
}

/* RX1 fifo interrupt */
void CAN1_RX1_IRQHandler(void) {
	CANController_Rx1Receive();
	SYS_ChangeIntFlag(SYS_INT_CANRX1IF);
}

void CAN1_SCE_IRQHandler(void) {

	if (CAN1->ESR & (ESR_EWGF | ESR_EPVF | ESR_BOFF)) {
		/* if error happened, copy the state */
		CANController_Error =	CAN1->ESR;

		/* clean flag */
		CANx->ESR &= ~ (ESR_EWGF | ESR_EPVF | ESR_BOFF);

		/* notify host */
		SYS_ChangeIntFlag(SYS_INT_CANERRIF);
	}
}


