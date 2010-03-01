#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f10x.h"

#include "platform.h"

#ifdef VECT_TAB_RAM
/* vector-offset (TBLOFF) from bottom of SRAM. defined in linker script */
extern uint32_t _isr_vectorsram_offs;
void NVIC_Configuration(void)
{
	/* Set the Vector Table base location at 0x20000000+_isr_vectorsram_offs */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, (uint32_t)&_isr_vectorsram_offs);
}
#else
extern uint32_t _isr_vectorsflash_offs;
void NVIC_Configuration(void)
{
	/* Set the Vector Table base location at 0x08000000+_isr_vectorsflash_offs */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, (uint32_t)&_isr_vectorsflash_offs);
}
#endif /* VECT_TAB_RAM */

void RCC_Configuration(void)
{
	SystemInit();

	/* Enable GPIO for led */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// CK_RTC clock selection
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* disable JTAG !!! */
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	/* set LED, SPI int as Open-drain */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* set LED, SPI int as Open-drain */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/*TODO:  should this be moved to power.c ? */
	/* PB6 - ACSEL, PB7 - ENABLE */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PB2 - ACPRES, PB5 - ALARM */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* pins to HiZ state */
	LED_YELLOW(Bit_SET);
	LED_GREEN(Bit_SET);
	LED_RED(Bit_SET);
	SPI_INT_WRITE(Bit_SET);

	PWR_ENABLE(Bit_RESET);
	PWR_ACSEL(Bit_RESET);

}

int main(void)
{
	/* System Clocks Configuration */
	RCC_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* Configure the GPIO ports */
	GPIO_Configuration();

	SYS_Init();
	PWR_Init();
	CANController_Init();
	SPI_Slave_Init();

	vTaskStartScheduler();

	while (1);
}

#define PERIOD ((portTickType) 1000 / portTICK_RATE_MS)
void vApplicationTickHook( void ) {
/*DISABLED*/
	static uint32_t uiTickCount = 0;
	static uint32_t b = 0xCAFEBABE;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (uiTickCount >= PERIOD) {
		uiTickCount = 0;
		xHigherPriorityTaskWoken = pdFALSE;
		b++;
	}
	uiTickCount ++;

}

void vApplicationIdleHook( void ) {
	__WFI();
}

