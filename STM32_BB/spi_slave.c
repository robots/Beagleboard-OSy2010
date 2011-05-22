/*
 * SPI Slave driver for STM32 family processors
 *
 * 2010-2011 Michal Demin
 *
 */

#include "platform.h"
#include "stm32f10x.h"

#include "cancontroller.h"
#include "power.h"
#include "sys.h"
#include "canbuf.h"

#include "commands.h"
#include "spi_slave.h"

/** Temporary buffer to save interrupt flag and others */
volatile uint16_t SPI1_TX_Tmp;

/** Dummy byte */
volatile uint8_t dummy;

/** Pointer to callback function */
DMA_Callback_t DMA_Callback = NULL;

/**
 * Initialization of SPI slave.
 */
void SPI1_Slave_Init()
{
	SPI_InitTypeDef  SPIConf;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	// Reset SPI1
	SPI_I2S_DeInit(SPI1);

	// Reset DMA Channels
	DMA_DeInit(DMA1_Channel2);
	DMA_DeInit(DMA1_Channel3);

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

	// DR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // out open drain
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
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	// Init DMA Channel (MEM->SPI)
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

	// Init DMA Channel (SPI->MEM)
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;

	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	// clear flag
	SPI_DR_WRITE(Bit_SET);


	// spi1 ISR
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	// enable interrupt
	SPI1RXNEI = 1;
	SPI1TXEI = 0;

}

/**
 * Sets-up DMA transfer (rx or tx) according to command received
 * \param cmd Command received from host
 */
static uint8_t setup_transfer(uint8_t cmd)
{
	uint8_t tmp;

	if (cmd & CMD_WRITE) { // WRITE
		switch (cmd & 0x7F) {
			case SYS_INTE:
				DMA1_Channel2->CMAR = (uint32_t)&SYS_InterruptEnable;
				DMA1_Channel2->CNDTR = sizeof(SYS_InterruptEnable);
				break;
/*			case SYS_INTF:
				DMA_Callback = SYS_IntFlagWriteHandle;
				DMA1_Channel2->CMAR = (uint32_t)&SYS_InterruptFlag;
				DMA1_Channel2->CNDTR = sizeof(SYS_InterruptFlag);
				break;*/
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
				// fault - enable spi interrupt
				SPI1RXNEI = 1;
				return 0;
				break;
		}
	} else { // READ
		switch (cmd) {
			case SYS_INTE:
				DMA1_Channel3->CMAR = (uint32_t)&SYS_InterruptEnable;
				DMA1_Channel3->CNDTR = sizeof(SYS_InterruptEnable);
				break;
			case SYS_INTF:
				/* interrupt "read and clear" atomic */
				SPI1_TX_Tmp = SYS_InterruptFlag;
				SYS_ClrIntFlag(0xffff);
				/*DMA1_Channel3->CMAR = (uint32_t)&SYS_InterruptFlag;
				DMA1_Channel3->CNDTR = sizeof(SYS_InterruptFlag);*/
				DMA1_Channel3->CMAR = (uint32_t)&SPI1_TX_Tmp;
				DMA1_Channel3->CNDTR = sizeof(SPI1_TX_Tmp);
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
				CANController_StatusHandle();
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
				tmp = CANController_Rx0Handle();
				if (tmp == 1) {
					DMA1_Channel3->CMAR = (uint32_t)&CANController_RX0Buffer0;
				} else {
					DMA1_Channel3->CMAR = (uint32_t)&CANController_RX0Buffer1;
				}
				DMA1_Channel3->CNDTR = sizeof(struct can_message_t) * CAN_BUFFER_SIZE;
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
				// fault - enable spi interrupt
				SPI1RXNEI = 1;
				return 0;
				break;
		}
	}
	return 1;
}

/**
 * Spi Interrupt handler.
 * Called after command byte is received.
 */
void SPI1_IRQHandler(void)
{
	static uint8_t cmd;
	static uint8_t ret;

	cmd = SPI1->DR;

	// disable spi interrupt
	SPI1RXNEI = 0;

	//setup dma transfer
	ret = setup_transfer(cmd);
	if (ret) {
		if (cmd & CMD_WRITE) {
			// enable RX DMA
			DMA1CH2EN = 1;
		} else {
			// enable TX DMA
			DMA1CH3EN = 1;
		}

		// tell host that transfer is ready
		SPI_DR_WRITE(Bit_RESET);
	}
}

/**
 * DMA1 Channel2 interrupt handler
 */
void DMA1_Channel2_IRQHandler(void)
{
	// disable DMA
	DMA1CH2EN = 0;
	// clear int pending bit on DMA2
	DMA1CH2GL = 1;

	// enable SPI rx interrupt
	dummy = SPI1->DR;
	SPI1RXNEI = 1;

	// clear flag
	SPI_DR_WRITE(Bit_SET);

	if (DMA_Callback) {
		DMA_Callback();
		DMA_Callback = NULL;
	}
}

/**
 * DMA1 Channel3 interrupt handler
 */
void DMA1_Channel3_IRQHandler(void)
{
	// disable DMA Channel
	DMA1CH3EN = 0;
	// clear int pending bit
	DMA1CH3GL = 1;

	// enable SPI rx interrupt
	dummy = SPI1->DR;
	SPI1RXNEI = 1;

	// clear flag
	SPI_DR_WRITE(Bit_SET);

	if (DMA_Callback) {
		DMA_Callback();
		DMA_Callback = NULL;
	}
}
