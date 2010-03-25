/*
 * bxCAN driver for STM32 family processors
 *
 * 2010 Michal Demin
 *
 */

#include "platform.h"
#include "stm32f10x.h"

#include "sys.h"
#include "canbuf.h"

#include "cancontroller.h"


volatile uint16_t CANController_Status = 0x0000;
volatile uint16_t CANController_Control = 0x0000;
volatile uint16_t CANController_Control_Last = 0x0000;
volatile uint32_t CANController_Error = 0x0000;

volatile struct can_timing_t CANController_Timing;

volatile struct can_message_t *CANController_RX0;
volatile struct can_message_t *CANController_RX1;
volatile struct can_message_t CANController_TXBuffer;
volatile struct can_message_t *CANController_TX;

struct can_buffer_t CANController_RX0Buffer;

void CANController_Init(void) {
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	CANBuf_Init(&CANController_RX0Buffer);
	CANController_RX0 = CANBuf_GetReadAddr(&CANController_RX0Buffer);

	CANController_TX = &CANController_TXBuffer;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	// Configure CAN pin: RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure CAN pin: TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SYS_ClrIntFlag(SYS_INT_CANMASK);

	// Reset CAN1
	CAN_DeInit(CAN1);

	// CAN filter init - all messages to FIFO0
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);


	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	// enable intertrupts, TODO: less magic values !
	CAN1->IER = 0x00008F13; // enable all interrupts (except FIFOx full/overrun, sleep/wakeup)

	CANController_Control_Last = 0;
	CANController_Control = 0;
	CANController_Status = 0;
}

/* This should be periodicaly called, using timer */
void CANController_Worker() {
	if ((CAN1->MSR & CAN_MSR_INAK) && (CAN1->MCR & CAN_MCR_INRQ)) {
		CANController_Status |= CAN_STAT_INAK;
	} 
}

/* message has been read from RX0, shift the buffer */
void CANController_Rx0Handle(void) {
	uint16_t rx0_count;

	CANController_RX0->flags |= CAN_MSG_INV;
	CANBuf_ReadDone(&CANController_RX0Buffer);
	CANController_RX0 = CANBuf_GetReadAddr(&CANController_RX0Buffer);
/*	rx0_count = CANBuf_GetAvailable(&CANController_RX0Buffer);

	//NOO takes lot of time :)
	//rx0_count <<= 8;
	//CANController_Status = (CANController_Status & ~CAN_STAT_RX0) | rx0_count;

	if (rx0_count) {
		SYS_SetIntFlag(SYS_INT_CANRX0IF);
	}*/
}

/* new message to be transmitted */
void CANController_TxHandle(void) {
	static uint16_t mailbox = 4;
	if (CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) {
		mailbox = (CAN1->TSR & CAN_TSR_CODE) >> 24;
	} else {
		// nothing empty ? :( this should not happen !
		// TODO: should we interrupt host with Error ?
		return;
	}

	// clear message
	CAN1->sTxMailBox[mailbox].TIR &= CAN_TI0R_TXRQ;

	// add IDE and RTR fields
	CAN1->sTxMailBox[mailbox].TIR |= (CANController_TX->flags >> 3) & 0x06;

	// add msg ID
	if (CANController_TX->flags & CAN_MSG_EID) {
		CAN1->sTxMailBox[mailbox].TIR |= CANController_TX->id << 3;
	} else {
		CAN1->sTxMailBox[mailbox].TIR |= CANController_TX->id << 21;
	}

	// setup the DLC field
	CAN1->sTxMailBox[mailbox].TDTR &= 0xFFFFFFF0;
	CAN1->sTxMailBox[mailbox].TDTR |= (CANController_TX->flags & CAN_MSG_SIZE);

	// Set up the data fields
	CAN1->sTxMailBox[mailbox].TDLR = (((uint32_t)CANController_TX->data[3] << 24) | ((uint32_t)CANController_TX->data[2] << 16) | ((uint32_t)CANController_TX->data[1] << 8) | ((uint32_t)CANController_TX->data[0]));
	CAN1->sTxMailBox[mailbox].TDHR = (((uint32_t)CANController_TX->data[7] << 24) | ((uint32_t)CANController_TX->data[6] << 16) | ((uint32_t)CANController_TX->data[5] << 8) | ((uint32_t)CANController_TX->data[4]));

	// mark message for transmission
	CAN1->sTxMailBox[mailbox].TIR |= CAN_TI1R_TXRQ;

	// if no TX mailbox empty signal host
	if (!(CAN1->TSR & CAN_TSR_TME)) {
		CANController_Status |= CAN_STAT_TXF;
	}
}

void CANController_ControlHandle(void) {
	// select the changed bits
	uint16_t change = CANController_Control_Last ^ CANController_Control;

	// bxCAN HW reset !
	if (change & CAN_CTRL_RST) {
		if (CANController_Control & CAN_CTRL_RST) {
			CAN1->MCR |= CAN_MCR_RESET;
			// FIXME
			// call init, it resets can Hw to default state
			//CANController_Init();
			return;
		}
	}

	if ((CAN1->MCR & CAN_MCR_INRQ) && (CAN1->MSR & CAN_MSR_INAK)) {
		// Automatic Bus-off managenment
		if (change & CAN_CTRL_ABOM) {
			if (CANController_Control & CAN_CTRL_ABOM) {
				CAN1->MCR |= CAN_MCR_ABOM;
			} else {
				CAN1->MCR &= ~CAN_MCR_ABOM;
			}
		}

		// One-shot mode
		if (change & CAN_CTRL_OSM) {
			if (CANController_Control & CAN_CTRL_OSM) {
				CAN1->MCR |= CAN_MCR_NART;
			} else {
				CAN1->MCR &= ~CAN_MCR_NART;
			}
		}

		// Loopback mode
		if (change & CAN_CTRL_LOOP) {
			if (CANController_Control & CAN_CTRL_LOOP) {
				CAN1->BTR |= CAN_BTR_LBKM;
			} else {
				CAN1->BTR &= ~CAN_BTR_LBKM;
			}
		}

		// Silent mode
		if (change & CAN_CTRL_SILM) {
			if (CANController_Control & CAN_CTRL_SILM) {
				CAN1->BTR |= CAN_BTR_SILM;
			} else {
				CAN1->BTR &= ~CAN_BTR_SILM;
			}
		}
	}

	// Initialization mode request
	if (change & CAN_CTRL_INIT) {
		if (CANController_Control & CAN_CTRL_INIT) {
			CAN1->MCR |= CAN_MCR_INRQ;
		} else {
			CAN1->MCR |= CAN_MCR_TXFP | CAN_MCR_RFLM | CAN_MCR_AWUM;// | CAN_MCR_ABOM;
			CAN1->MCR &= ~(CAN_MCR_SLEEP | 0x10000); // we don't support sleep, no debug-freeze
			CAN1->MCR &= ~CAN_MCR_INRQ; // leave init mode
			CANController_Status &= ~CAN_STAT_INAK;
		}
	}

	CANController_Control_Last = CANController_Control;
}

/* Changes the timming in the bxCAN module
 * Side-effect: silent and loopback is disabled
 */
void CANController_TimingHandle(void) {

	// Test if CAN1 is in initialization mode
	if ((CAN1->MCR & CAN_MCR_INRQ) && (CAN1->MSR & CAN_MSR_INAK)) {
		// apply the settings
		CANController_Timing.ts &= 0x437F; // clears loopback and silent bit
		CANController_Timing.brp &= 0x3FF; // removes any junk

		CAN1->BTR = (CANController_Timing.ts << 16) | CANController_Timing.brp;
	}
}

/* TX interrupt */
void USB_HP_CAN1_TX_IRQHandler(void) {
	CANController_Status &= ~(CAN_STAT_ALST | CAN_STAT_TERR);

	if (CAN1->TSR & CAN_TSR_RQCP2) {
		CAN1->TSR |= CAN_TSR_RQCP2;
		CANController_Status |= (CAN1->TSR & CAN_TSR_ALST0)?CAN_STAT_ALST:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TERR0)?CAN_STAT_TERR:0;
	}
	if (CAN1->TSR & CAN_TSR_RQCP1) {
		CAN1->TSR |= CAN_TSR_RQCP1;
		CANController_Status |= (CAN1->TSR & CAN_TSR_ALST1)?CAN_STAT_ALST:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TERR1)?CAN_STAT_TERR:0;
	}
	if (CAN1->TSR & CAN_TSR_RQCP0) {
		CAN1->TSR |= CAN_TSR_RQCP0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_ALST2)?CAN_STAT_ALST:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TERR2)?CAN_STAT_TERR:0;
	}

	CANController_Status &= ~CAN_STAT_TXF;

	// notify host that message was sent !
	SYS_SetIntFlag(SYS_INT_CANTXIF);
}

/* RX0 fifo interrupt 
 * We cannot eat the whole FIFO, instead we let NVIC process higher prio
 * interrupt and return here later.
 * This is necessary for SPI_Slave to work !!!
 */
void USB_LP_CAN1_RX0_IRQHandler(void) {
	static struct can_message_t *RX0;
	static uint16_t rx0_count;

	RX0 = CANBuf_GetWriteAddr(&CANController_RX0Buffer);
	RX0->flags = 0x00;

	// check RTR
	if (CAN1->sFIFOMailBox[0].RIR & 0x02) {
		RX0->flags |= CAN_MSG_RTR;
	}

	// update DLC
	RX0->flags |= CAN1->sFIFOMailBox[0].RDTR & CAN_MSG_SIZE;

	// copy msg ID
	if (CAN1->sFIFOMailBox[0].RIR & 0x04) {
		// extended id
		RX0->flags |= CAN_MSG_EID;
		RX0->id = (uint32_t)0x1FFFFFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
	} else {
		// standard id
		RX0->id = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
	}

	// copy data bytes
	RX0->data[0] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR);
	RX0->data[1] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
	RX0->data[2] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
	RX0->data[3] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDLR >> 24);
	RX0->data[4] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR);
	RX0->data[5] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
	RX0->data[6] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
	RX0->data[7] = (uint8_t)0xFF & (CAN1->sFIFOMailBox[0].RDHR >> 24);

	// release fifo
	CAN1->RF0R = CAN_RF0R_RFOM0;

	if (CANBuf_Empty(&CANController_RX0Buffer)) {
		CANController_RX0 = RX0;
	}

	// Advance the ring buffer
	CANBuf_WriteDone(&CANController_RX0Buffer);
		
	// update Status 
	rx0_count = CANBuf_GetAvailable(&CANController_RX0Buffer);
	rx0_count <<= 8;
	CANController_Status = (CANController_Status & ~CAN_STAT_RX0) | (rx0_count & CAN_STAT_RX0);

	// notify host by interrupt
	SYS_SetIntFlag(SYS_INT_CANRX0IF);

}

/* status change and error
 * This ISR is called periodicaly while error condition persists !
 */
void CAN1_SCE_IRQHandler(void) {

	if (CAN1->ESR & (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)) {
		// if error happened, copy the state
		CANController_Error = CAN1->ESR;

		// clean flag
		CAN1->ESR &= ~ (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF);
		CAN1->ESR |= CAN_ESR_LEC;

		// clear interrupt flag
		CAN1->MSR &= ~CAN_MSR_ERRI;

		// notify host
		SYS_SetIntFlag(SYS_INT_CANERRIF);
	}
}

