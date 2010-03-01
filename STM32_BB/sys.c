#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "platform.h"
#include "stm32f10x.h"

#include "sys.h"

uint16_t SYS_InterruptEnable = 0x0000;
uint16_t SYS_InterruptFlag = 0x0000;
uint16_t SYS_Identifier = 0xCAFE;
uint16_t SYS_Reset = 0x0000;

void SYS_Init() {
	SYS_InterruptEnable = 0x0000;
	SYS_InterruptFlag = 0x0000;
	SPI_INT_WRITE(Bit_SET);
}

void SYS_ChangeIntFlag(uint16_t in) {
	SYS_InterruptFlag |= in;
	if (SYS_InterruptEnable & in) {
		SPI_INT_WRITE(Bit_RESET);
	}
}

void SYS_IntFlagWriteHandle(void) {
	if ((SYS_InterruptFlag & SYS_InterruptEnable) == 0x0000) {
		SPI_INT_WRITE(Bit_SET);
	}
}

void SYS_ResetHandler(void) {
	if (SYS_Reset == SYS_RESET_MAGIC) {
		SYS_Reset = 0x0000;

		/* do reset */
		NVIC_SystemReset();
	}
}
