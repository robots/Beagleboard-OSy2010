#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "platform.h"
#include "stm32f10x.h"

#include "sys.h"

uint16_t SYS_InterruptEnable = 0x00;
uint16_t SYS_InterruptFlag = 0x00;


void SYS_ChangeIntFlag(uint16_t in) {
	SYS_InterruptFlag |= in;
	if (SYS_InterruptEnable & in) {
					
	}
}

void SYS_IntFlagReadHandle(void) {

}

void SYS_IntFlagWriteHandle(void) {
	
}

