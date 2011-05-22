/*
 * Main file
 *
 * 2010 Michal Demin
 *
 */

#include "stm32f10x.h"
#include "spi_slave.h"
#include "sys.h"
#include "cancontroller.h"
#include "power.h"

#include "platform.h"

/** Variable to be changed by GDB to enable debug mode */
volatile uint32_t DEBUG_ON = 1;

#ifdef VECT_TAB_RAM
/* vector-offset (TBLOFF) from bottom of SRAM. defined in linker script */
extern uint32_t _isr_vectorsram_offs;
void NVIC_Configuration(void)
{
	// Set the Vector Table base location at 0x20000000+_isr_vectorsram_offs
	NVIC_SetVectorTable(NVIC_VectTab_RAM, (uint32_t)&_isr_vectorsram_offs);
}
#else
extern uint32_t _isr_vectorsflash_offs;
void NVIC_Configuration(void)
{
	// Set the Vector Table base location at 0x08000000+_isr_vectorsflash_offs
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, (uint32_t)&_isr_vectorsflash_offs);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
#endif /* VECT_TAB_RAM */

void RCC_Configuration(void)
{
	SystemInit();

	// Enable GPIO for led
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// JTAG enabled in debug mode !!! (see also .gdbinit)
	if (DEBUG_ON != 1) {
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	}

	// set PA[0-4] as analog inputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// set LEDs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// pins to default state
	LED_RED(Bit_SET); // off
	LED_GREEN(Bit_SET); // off
	LED_YELLOW(Bit_SET); // off
}

int main(void)
{
	// System Clocks Configuration
	RCC_Configuration();

	// NVIC configuration
	NVIC_Configuration();

	// Configure the GPIO ports
	GPIO_Configuration();

	__disable_irq();
	PWR_Init();
	CANController_Init();
	SPI1_Slave_Init();
	SYS_Init();

	LED_GREEN(Bit_RESET); // on
	__enable_irq();

	while (1) {
	}
}
