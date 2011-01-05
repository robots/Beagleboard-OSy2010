/*
 * External interrupt and system managenment
 *
 * 2010 Michal Demin
 *
 */

#include "platform.h"
#include "stm32f10x.h"

#include "sys.h"

volatile uint16_t SYS_InterruptEnable = 0x0000;
volatile uint16_t SYS_InterruptFlag = 0x0000;
const uint16_t SYS_Identifier = 0xCAFE;
volatile uint16_t SYS_Reset = 0x0000;

void SYS_Init() {
	// SPI Int as OD output
	GPIO_InitTypeDef GPIO_InitStructure = {
		.GPIO_Pin = GPIO_Pin_8,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_Mode = GPIO_Mode_Out_OD,
	};

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// leave the pin in Hi-Z
	SPI_INT_WRITE(Bit_SET);
	LED_YELLOW(Bit_SET);

	SYS_InterruptEnable = 0x0000;
	SYS_InterruptFlag = 0x0000;
}

void SYS_SetIntFlag(uint16_t in) {
	SYS_InterruptFlag |= in;
	if (SYS_InterruptEnable & SYS_InterruptFlag) {
	// we need to check global flag
	// otherwise we can loose interrupt
	//if (SYS_InterruptEnable & in) {
		SPI_INT_WRITE(Bit_RESET);
		LED_YELLOW(Bit_RESET);
	}
}

void SYS_ClrIntFlag(uint16_t in) {
	SYS_InterruptFlag &= ~in;
	if ((SYS_InterruptEnable & SYS_InterruptFlag) == 0x0000) {
	// we need to check global flag
	// otherwise we can loose interrupt
	//if ((SYS_InterruptEnable & in) == 0x0000) {
		SPI_INT_WRITE(Bit_SET);
		LED_YELLOW(Bit_SET);
	}
}

void SYS_IntFlagWriteHandle(void) {
	if ((SYS_InterruptFlag & SYS_InterruptEnable) == 0x0000) {
		SPI_INT_WRITE(Bit_SET);
		LED_YELLOW(Bit_SET);
	} else if ((SYS_InterruptFlag & SYS_InterruptEnable)) {
		SPI_INT_WRITE(Bit_RESET);
		LED_YELLOW(Bit_RESET);
	}
}

extern volatile uint16_t DEBUG_ON;

void SYS_ResetHandler(void) {
	if (DEBUG_ON == 1)
		return;

	if (SYS_Reset == SYS_RESET_MAGIC) {
		__disable_irq();

		// clear all interrupts
		SYS_ClrIntFlag(0xffff);

		// reset magic (it should be done by C startup, but just to be sure
		SYS_Reset = 0x0000;

		// show the world we are down
		LED_GREEN(Bit_SET);

		// do reset
		NVIC_SystemReset();
	}
}

