#include "stm32f10x.h"

#include "platform.h"


void Delay_ (unsigned long a) { while (--a!=0); }

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
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* pins to HiZ state */
	LED_YELLOW(Bit_SET);
	LED_GREEN(Bit_SET);
	LED_RED(Bit_SET);


}

int main(void)
{
	uint32_t dly;

	/* System Clocks Configuration */
	RCC_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* Configure the GPIO ports */
	GPIO_Configuration();
	LED_YELLOW(Bit_RESET);
	RTCInit();


	while (1);
}

