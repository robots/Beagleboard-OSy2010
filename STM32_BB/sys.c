/*
 * External interrupt and system managenment
 *
 * 2010 Michal Demin
 *
 */
/*
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
*/
#include "platform.h"
#include "stm32f10x.h"

#include "sys.h"

volatile uint16_t SYS_InterruptEnable = 0x0000;
volatile uint16_t SYS_InterruptFlag = 0x0000;
const uint16_t SYS_Identifier = 0xCAFE;
volatile uint16_t SYS_Reset = 0x0000;

void SYS_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	// SPI Int as OD output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SYS_InterruptEnable = 0x0000;
	SYS_InterruptFlag = 0x0000;

	// leave the pin in Hi-Z
	SPI_INT_WRITE(Bit_SET);
}

void SYS_ChangeIntFlag(uint16_t in) {
	SYS_InterruptFlag |= in;
	if (SYS_InterruptEnable & in) {
		SPI_INT_WRITE(Bit_RESET);
		LED_YELLOW(Bit_RESET);
	}
}

void SYS_IntFlagWriteHandle(void) {
	if ((SYS_InterruptFlag & SYS_InterruptEnable) == 0x0000) {
		SPI_INT_WRITE(Bit_SET);
		LED_YELLOW(Bit_SET);
	}
}

void SYS_ResetHandler(void) {
	if (SYS_Reset == SYS_RESET_MAGIC) {
		SYS_Reset = 0x0000;

		// do reset
		// TODO: add RCC reset to main, to reset peripherals !
		NVIC_SystemReset();
	}
}

