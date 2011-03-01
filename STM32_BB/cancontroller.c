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
volatile uint32_t CANController_Error_Last = 0x0000;

volatile struct can_timing_t CANController_Timing;

volatile uint16_t RX0_Count = 0;
volatile struct can_message_t CANController_TXBuffer;
volatile struct can_message_t *CANController_TX;

struct can_buffer_t *CANController_RX0Buffer;
struct can_buffer_t CANController_RX0Buffer0;
struct can_buffer_t CANController_RX0Buffer1;
uint8_t CANController_RX0Active = 0;

static CAN_FilterInitTypeDef CAN_FilterInitStructure = {
	.CAN_FilterNumber = 0,
	.CAN_FilterMode = CAN_FilterMode_IdMask,
	.CAN_FilterScale = CAN_FilterScale_32bit,
	.CAN_FilterIdHigh = 0x0000,
	.CAN_FilterIdLow = 0x0000,
	.CAN_FilterMaskIdHigh = 0x0000,
	.CAN_FilterMaskIdLow = 0x0000,
	.CAN_FilterFIFOAssignment = 0,
	.CAN_FilterActivation = ENABLE,
};
static NVIC_InitTypeDef CAN_Int;

static void CANController_HW_Reinit(int first);

void CANController_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure = {
		.GPIO_Speed = GPIO_Speed_50MHz,
	};

	CANBuf_Init(&CANController_RX0Buffer0);
	CANBuf_Init(&CANController_RX0Buffer1);
	CANController_RX0Active = 0;
	CANController_RX0Buffer = &CANController_RX0Buffer0;

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

	CAN_Int.NVIC_IRQChannelPreemptionPriority = 0;
	CAN_Int.NVIC_IRQChannelSubPriority = 5;

	CANController_Control_Last = 0;
	CANController_Control = 0;
	CANController_Status = 0;

	CANController_HW_Reinit(1);
}

/* Initialization routine with workaround for the error managenment bug in the bxCAN */
static void CANController_HW_Reinit(int first) {
	CAN_Int.NVIC_IRQChannelCmd = DISABLE;
	CAN_Int.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_Init(&CAN_Int);
	CAN_Int.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_Init(&CAN_Int);
	CAN_Int.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_Init(&CAN_Int);

	// Reset CAN1 - clears the error state
	CAN_DeInit(CAN1);

	SYS_ClrIntFlag(SYS_INT_CANMASK);

	// setup filter to receive everything
	CAN_FilterInit(&CAN_FilterInitStructure);

	// enable intertrupts
	//CAN1->IER = 0x00008F13; // enable all interrupts (except FIFOx full/overrun, sleep/wakeup)
	CAN1->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0;
	CAN1->IER |= CAN_IER_EWGIE | CAN_IER_EPVIE | CAN_IER_BOFIE | CAN_IER_LECIE;
//	CAN1->IER |= CAN_IER_ERRIE;  // TODO: test me

	if (!first) {
		// enter the init mode
		CAN1->MCR &= ~CAN_MCR_SLEEP;
		CAN1->MCR |= CAN_MCR_INRQ;

		// wait for it !
		while ((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

		// force reinitialization !
		// tell the ControlHandler that we are in init mode
		CANController_Control_Last = 1;
		CANController_Error_Last = 0;

		// apply the same setting
		CANController_TimingHandle();
		CANController_ControlHandle();
	}

	CAN_Int.NVIC_IRQChannelCmd = ENABLE;
	CAN_Int.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_Init(&CAN_Int);
	CAN_Int.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_Init(&CAN_Int);
	CAN_Int.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_Init(&CAN_Int);

}

void CANController_StatusHandle() {
	if ((CAN1->MSR & CAN_MSR_INAK) && (CAN1->MCR & CAN_MCR_INRQ)) {
		CANController_Status |= CAN_STAT_INAK;
	}
	CANController_Status |= CAN_STAT_RX1;
}

uint8_t CANController_Rx0Handle(void) {
	struct can_buffer_t *canbuf;

	if (CANController_RX0Active == 0) {
		canbuf = &CANController_RX0Buffer1;
	} else {
		canbuf = &CANController_RX0Buffer0;
	}
	CANController_RX0Active = !CANController_RX0Active;

	// invalidate buffer
	CANBuf_Init(canbuf);

	CANController_RX0Buffer = canbuf;
	RX0_Count = 0;
	CANController_Status &= ~(CAN_STAT_RX0 | CAN_STAT_RXOV);

	return CANController_RX0Active; 
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
			// Setting the reset bit doesn't reset fauly interrupt
			// instead we reset the whole bxCan HW
			//CAN1->MCR |= CAN_MCR_RESET;
			CANController_HW_Reinit(0);
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

		// Error interrupt enable
		if (change & CAN_CTRL_IERR) {
			if (CANController_Control & CAN_CTRL_IERR) {
				CAN1->IER |= CAN_IER_ERRIE; 
			} else {
				CAN1->IER &= ~CAN_IER_ERRIE; 
			}
		}
	}

	// Initialization mode request
	if (change & CAN_CTRL_INIT) {
		if (CANController_Control & CAN_CTRL_INIT) {
			CAN1->MCR |= CAN_MCR_INRQ;
		} else {
			CAN1->MCR |= CAN_MCR_TXFP | CAN_MCR_RFLM | CAN_MCR_AWUM; // automatic wakeup, tx round-robin mode
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
	CANController_Status &= ~(CAN_STAT_ALST | CAN_STAT_TERR | CAN_STAT_TXOK);

	if (CAN1->TSR & CAN_TSR_RQCP0) {
		CANController_Status |= (CAN1->TSR & CAN_TSR_ALST0)?CAN_STAT_ALST:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TERR0)?CAN_STAT_TERR:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TXOK0)?CAN_STAT_TXOK:0;
		CAN1->TSR |= CAN_TSR_RQCP2;
	}
	if (CAN1->TSR & CAN_TSR_RQCP1) {
		CANController_Status |= (CAN1->TSR & CAN_TSR_ALST1)?CAN_STAT_ALST:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TERR1)?CAN_STAT_TERR:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TXOK1)?CAN_STAT_TXOK:0;
		CAN1->TSR |= CAN_TSR_RQCP1;
	}
	if (CAN1->TSR & CAN_TSR_RQCP2) {
		CANController_Status |= (CAN1->TSR & CAN_TSR_ALST2)?CAN_STAT_ALST:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TERR2)?CAN_STAT_TERR:0;
		CANController_Status |= (CAN1->TSR & CAN_TSR_TXOK2)?CAN_STAT_TXOK:0;
		CAN1->TSR |= CAN_TSR_RQCP0;
	}

	// Check for abort ack
	if (CANController_Status & (CAN_STAT_ALST | CAN_STAT_TERR | CAN_STAT_TXOK)) {
		CANController_Status &= ~CAN_STAT_TXF;
		// notify host that message was sent !
		SYS_SetIntFlag(SYS_INT_CANTXIF);
	}
}

/* RX0 fifo interrupt
 * We cannot eat the whole FIFO, instead we let NVIC process higher prio
 * interrupt and return here later.
 * This is necessary for SPI_Slave to work !!!
 */
void USB_LP_CAN1_RX0_IRQHandler(void) {
	static struct can_message_t *RX0;

	RX0 = CANBuf_GetWriteAddr(CANController_RX0Buffer);
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

	// Advance the ring buffer
	CANBuf_WriteDone(CANController_RX0Buffer);

	// update Status
	if (RX0_Count < CAN_BUFFER_SIZE) {
		RX0_Count += 1;
	} else {
		CANController_Status |= CAN_STAT_RXOV;
	}

	CANController_Status &= ~CAN_STAT_RX0;
	CANController_Status |= (RX0_Count << 8) & CAN_STAT_RX0;

	// notify host by interrupt
	SYS_SetIntFlag(SYS_INT_CANRX0IF);

}

/* status change and error
 * This ISR is called periodicaly while error condition persists !
 *
 * FIXME: This is very much broken !!!
 */
void CAN1_SCE_IRQHandler(void) {
	static uint16_t count = 0;

	if (CAN1->ESR & (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)) {
		// if error happened, copy the state
		CANController_Error = CAN1->ESR;

		// abort msg transmission on Bus-Off
		if (CAN1->ESR & CAN_ESR_BOFF) {
			CAN1->TSR |= (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2);
		}
		// clean flag - not working at all :(
		CAN1->ESR &= ~ (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF);

		// clear last error code
		CAN1->ESR |= CAN_ESR_LEC;

		// clear interrupt flag
		CAN1->MSR &= ~CAN_MSR_ERRI;

		// work around the bug in HW
		// notify only on "new" error, otherwise reset can controller
		if (CANController_Error ^ CANController_Error_Last) {
			count = 0;
			// notify host
			SYS_SetIntFlag(SYS_INT_CANERRIF);
		} else {
			count ++;
			if (count > 5) {
				count = 0;
				CANController_HW_Reinit(0);
				SYS_SetIntFlag(SYS_INT_CANRSTIF);
			}
		}
		CANController_Error_Last = CANController_Error;
	}
}

