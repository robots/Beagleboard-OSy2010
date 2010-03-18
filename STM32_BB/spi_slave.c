/*
 * SPI driver for STM32 family processors
 *
 * 2009-2010 Michal Demin
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "platform.h"
#include "stm32f10x.h"

#include "cancontroller.h"
#include "power.h"
#include "sys.h"

#include "commands.h"
#include "spi_slave.h"

typedef void (*DMA_Callback_t)();

/* number of pending handlers to be executed */
#define DMA_QUEUE_SIZE      (5)
/* needs to be highest priority */
#define DMA_WORKER_PRIORITY (1)


enum {
	SPI1_CMD,
	SPI1_DATA
};

/* specifies what dma transfer is finishing next */
uint8_t SPI1_DMA_Type = SPI1_CMD;

/* command received */
uint8_t SPI1_Cmd;
static uint8_t dummy;

/* queue for worker thread */
xQueueHandle xDMAQueue = NULL;

/* callback */
DMA_Callback_t DMA_Callback;

static void taskDMAWorker( void *pvParameters );


void SPI1_Slave_Init() {

	SPI_InitTypeDef  SPIConf;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	xDMAQueue = xQueueCreate(DMA_QUEUE_SIZE, sizeof(DMA_Callback_t));
	xTaskCreate( taskDMAWorker, ( signed char * ) "DMAWork", configMINIMAL_STACK_SIZE, NULL, DMA_WORKER_PRIORITY, NULL );

	// SPI module enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	// Configure SPI1 pins
	// MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // input
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// MISO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // alt. fnc. push-pull
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// CLK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // input 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // input
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPIConf.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPIConf.SPI_Mode = SPI_Mode_Slave;
	SPIConf.SPI_DataSize = SPI_DataSize_8b;
	SPIConf.SPI_CPOL = SPI_CPOL_Low;
	SPIConf.SPI_CPHA = SPI_CPHA_1Edge;
	SPIConf.SPI_NSS = SPI_NSS_Hard;
	SPIConf.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //not used in slave
	SPIConf.SPI_FirstBit = SPI_FirstBit_MSB;
	SPIConf.SPI_CRCPolynomial = 7;


	SPI_Init(SPI1, &SPIConf);
	SPI_SSOutputCmd(SPI1, DISABLE);
	SPI_Cmd(SPI1, ENABLE);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

	// Init DMA structure
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	// Init DMA Channel (MEM->SPI)
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

	// Init DMA Channel (SPI->MEM)
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;

	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);


	// enable RX DMA - bootstrap
	DMA1_Channel2->CMAR = (uint32_t)&SPI1_Cmd;
	DMA1_Channel2->CNDTR = sizeof(SPI1_Cmd);

	DMA_Cmd(DMA1_Channel2, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	// spi1 ISR does not rely on FreeRTOS api - can be higher priority
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);

}

static void taskDMAWorker( void *pvParameters ) {
	(void)pvParameters; // silent :)
	DMA_Callback_t tmpCallback;

	while (1) {
		xQueueReceive( xDMAQueue, &tmpCallback, portMAX_DELAY);
		if (tmpCallback != NULL) {
			tmpCallback();
		}
	}
}

void SPI1_IRQHandler(void) {
	// clear the interrupt pending flag
	SPI1->SR = (uint16_t)~((1 << SPI_IT_RXNE) | (1 << SPI_IT_TXE));
	// disable interrupt
	SPI1->CR2 &= (uint16_t)~((1 << SPI_IT_RXNE) | (1 << SPI_IT_TXE));

	// clear the TXE bit if set
	dummy = SPI1->DR;

	if (SPI1_Cmd & CMD_WRITE) {
		// enable RX DMA
		DMA1_Channel2->CCR |= CCR_ENABLE_Set;
	} else {
		// enable TX DMA
		DMA1_Channel3->CCR |= CCR_ENABLE_Set;
	}
}

/* ISR for DMA1 Channel2 */ 
void DMA1_Channel2_IRQHandler(void) {
	static portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	// disable DMA
	DMA1_Channel2->CCR &= ~CCR_ENABLE_Set;
	// clear int pending bit on DMA2
	DMA1->IFCR = DMA1_IT_GL2;

	if (SPI1_DMA_Type == SPI1_DATA) {
		xQueueSendFromISR( xDMAQueue, &DMA_Callback, &pxHigherPriorityTaskWoken);
		DMA_Callback = NULL;

		DMA1_Channel2->CMAR = (uint32_t)&SPI1_Cmd;
		DMA1_Channel2->CNDTR = 1;
		DMA1_Channel2->CCR |= CCR_ENABLE_Set;

		SPI1_DMA_Type = SPI1_CMD; // next is command
		portEND_SWITCHING_ISR( pxHigherPriorityTaskWoken );
	} else {
		SPI1_DMA_Type = SPI1_DATA; // data is next

		// we have time to setup another trancation
		if (SPI1_Cmd & CMD_WRITE) { // WRITE 
			// read - to clear RXNE flag
			dummy = SPI1->DR;
			SPI1->CR2 |= (uint16_t)(1 << SPI_IT_RXNE);
			switch (SPI1_Cmd & 0x7F) {
				case SYS_INTE:
					DMA1_Channel2->CMAR = (uint32_t)&SYS_InterruptEnable;
					DMA1_Channel2->CNDTR = sizeof(SYS_InterruptEnable);
					break;
				case SYS_INTF:
					DMA_Callback = SYS_IntFlagWriteHandle;
					DMA1_Channel2->CMAR = (uint32_t)&SYS_InterruptFlag;
					DMA1_Channel2->CNDTR = sizeof(SYS_InterruptFlag);
					break;
				case SYS_RESET:
					DMA_Callback = SYS_ResetHandler;
					DMA1_Channel2->CMAR = (uint32_t)&SYS_Reset;
					DMA1_Channel2->CNDTR = sizeof(SYS_Reset);
					break;
				case CAN_TIMING:
					DMA_Callback = CANController_TimingHandle;
					DMA1_Channel2->CMAR = (uint32_t)&CANController_Timing;
					DMA1_Channel2->CNDTR = sizeof(struct can_timing_t);
					break;
				case CAN_CTRL:
					DMA_Callback = CANController_ControlHandle;
					DMA1_Channel2->CMAR = (uint32_t)&CANController_Control;
					DMA1_Channel2->CNDTR = sizeof(CANController_Control);
					break;
				case CAN_TX:
					DMA_Callback = CANController_TxHandle;
					DMA1_Channel2->CMAR = (uint32_t)CANController_TX;
					DMA1_Channel2->CNDTR = sizeof(struct can_message_t);
					break;
				case PWR_CTRL:
					DMA_Callback = PWR_ControlHandle;
					DMA1_Channel2->CMAR = (uint32_t)&PWR_Control;
					DMA1_Channel2->CNDTR = sizeof(PWR_Control);
					break;
				case PWR_I_SET:
					DMA_Callback = PWR_CurrentHandle;
					DMA1_Channel2->CMAR = (uint32_t)&PWR_I_Set;
					DMA1_Channel2->CNDTR = sizeof(PWR_I_Set);
					break;
				default:
					// fault ... receive command byte again
					SPI1_DMA_Type = SPI1_CMD;
					DMA1_Channel2->CMAR = (uint32_t)&SPI1_Cmd;
					DMA1_Channel2->CNDTR = 1;
					DMA1_Channel2->CCR |= CCR_ENABLE_Set;
					SPI1->CR2 &= (uint16_t)~((1 << SPI_IT_RXNE) | (1 << SPI_IT_TXE));
					break;
			}
		} else { // READ
			// put something in the tx fifo, this ensures that TXE flag is cleared
			SPI1->DR = 0x55;
			// enable SPI tx empty interrupt
			SPI1->CR2 |= (uint16_t)(1 << SPI_IT_TXE);
			DMA_Callback = NULL;
			switch (SPI1_Cmd) {
				case SYS_INTE:
					DMA1_Channel3->CMAR = (uint32_t)&SYS_InterruptEnable;
					DMA1_Channel3->CNDTR = sizeof(SYS_InterruptEnable);
					break;
				case SYS_INTF:
					DMA1_Channel3->CMAR = (uint32_t)&SYS_InterruptFlag;
					DMA1_Channel3->CNDTR = sizeof(SYS_InterruptFlag);
					break;
				case SYS_ID:
					DMA1_Channel3->CMAR = (uint32_t)&SYS_Identifier;
					DMA1_Channel3->CNDTR = sizeof(SYS_Identifier);
					break;
				case CAN_ERR:
					DMA1_Channel3->CMAR = (uint32_t)&CANController_Error;
					DMA1_Channel3->CNDTR = sizeof(CANController_Error);
					break;
				case CAN_STATUS:
					DMA1_Channel3->CMAR = (uint32_t)&CANController_Status;
					DMA1_Channel3->CNDTR = sizeof(CANController_Status);
					break;
				case CAN_CTRL:
					DMA1_Channel3->CMAR = (uint32_t)&CANController_Control;
					DMA1_Channel3->CNDTR = sizeof(CANController_Control);
					break;
				case CAN_TIMING:
					DMA1_Channel3->CMAR = (uint32_t)&CANController_Timing;
					DMA1_Channel3->CNDTR = sizeof(struct can_timing_t);
					break;
				case CAN_RX0:
					DMA_Callback = CANController_Rx0Handle;
					DMA1_Channel3->CMAR = (uint32_t)CANController_RX0;
					DMA1_Channel3->CNDTR = sizeof(struct can_message_t);
					break;
				case CAN_RX1:
					DMA_Callback = CANController_Rx1Handle;
					DMA1_Channel3->CMAR = (uint32_t)CANController_RX1;
					DMA1_Channel3->CNDTR = sizeof(struct can_message_t);
					break;
				case PWR_STATUS:
					DMA1_Channel3->CMAR = (uint32_t)&PWR_Status;
					DMA1_Channel3->CNDTR = sizeof(PWR_Status);
					break;
				case PWR_CTRL:
					DMA1_Channel3->CMAR = (uint32_t)&PWR_Control;
					DMA1_Channel3->CNDTR = sizeof(PWR_Control);
					break;
				case PWR_I_SET:
					DMA1_Channel3->CMAR = (uint32_t)&PWR_I_Set;
					DMA1_Channel3->CNDTR = sizeof(PWR_I_Set);
					break;
				case PWR_DATA:
					DMA1_Channel3->CMAR = (uint32_t)&PWR_Measurement_Data;
					DMA1_Channel3->CNDTR = sizeof(PWR_Measurement_Data);
					break;
				default:
					// fault ... receive command byte again
					SPI1_DMA_Type = SPI1_CMD;
					DMA1_Channel2->CMAR = (uint32_t)&SPI1_Cmd;
					DMA1_Channel2->CNDTR = 1;
					DMA1_Channel2->CCR |= CCR_ENABLE_Set;
					SPI1->CR2 &= (uint16_t)~((1 << SPI_IT_RXNE) | (1 << SPI_IT_TXE));
					break;
			}
		}
	}
}

/* ISR for DMA1 Channel3 */
void DMA1_Channel3_IRQHandler(void) {
	static portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	// disable DMA Channel 
	DMA1_Channel3->CCR &= ~CCR_ENABLE_Set;
	// clear int pending bit
	DMA1->IFCR = DMA1_IT_GL3;

	// read - to clear RXNE flag
	// enabled DMA would read this data, and we don't like that
	dummy = SPI1->DR;

	// send data to worker thread
	xQueueSendFromISR( xDMAQueue, &DMA_Callback, &pxHigherPriorityTaskWoken);
	DMA_Callback = NULL;

	// awaiting command in next transfer
	SPI1_DMA_Type = SPI1_CMD;

	// enable dma
	DMA1_Channel2->CMAR = (uint32_t)&SPI1_Cmd;
	DMA1_Channel2->CNDTR = 1;
	DMA1_Channel2->CCR |= CCR_ENABLE_Set;

	portEND_SWITCHING_ISR( pxHigherPriorityTaskWoken );
}

