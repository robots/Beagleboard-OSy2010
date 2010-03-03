#include "FreeRTOS.h"
/*#include "task.h"
#include "queue.h"
#include "semphr.h"
*/
#include "platform.h"
#include "stm32f10x.h"

#include "sys.h"

#include "power.h"

struct pwr_data_t PWR_Measurement_Data = {0, 0, 0, 0};
struct pwr_i_set_t PWR_I_Set = {0, 0};

uint16_t PWR_Control = 0x0000;
uint16_t PWR_Control_Last = 0x0000;
uint16_t PWR_Status = 0x0000;

void PWR_Init() {
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	PWR_Control = 0;
	PWR_Control_Last = 0;
	PWR_Status = 0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	DMA_DeInit(DMA1_Channel1);
	ADC_DeInit(ADC1);

	/* Init DMA structure */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&PWR_Measurement_Data;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	/* Init DMA Channel (MEM->SPI) */
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	//DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 4;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* regular channel group */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);

	/* calibrate ADC */
/*TODO: fix me !!!
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
*/

/* setup external interrupts */
	/* PB2 - ACPRES, PB5 - ALARM */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* switch exti to port B */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
	
	/* setup EXTI trigger mode */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_Init(&EXTI_InitStructure);
	
	/* enable interrupt at NVIC */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	/* setup PWR_Status to show actual state of EXTI pins ? */
	if (PWR_ACPRES() == Bit_SET) {
		PWR_Status |= PWR_STAT_ACPRE;
	}
	if (PWR_ALARM() == Bit_SET) {
		PWR_Status |= PWR_STAT_ALARM;
	}

/* setup PWM output */
	/* setup PB0 PB1 as AF_PP*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* setup TIM3 to generate PWM  on PB[01] */
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0x3FFF; // 36MHz / 16368 = ~2.5KHz
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	
/* setup output pins */
	/* PB6 - ACSEL, PB7 - ENABLE */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	PWR_ENABLE(Bit_RESET);
	PWR_ACSEL(Bit_RESET);
}

void PWR_ControlHandle() {
	int16_t change = PWR_Control_Last ^ PWR_Control;

	if (change & PWR_CTRL_ADC) {
		if (PWR_Control & PWR_CTRL_ADC) {
			DMA_Cmd(DMA1_Channel1, ENABLE);
			ADC_DMACmd(ADC1, ENABLE);
			ADC_Cmd(ADC1, ENABLE);
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		} else {
			DMA_Cmd(DMA1_Channel1, DISABLE);
			ADC_Cmd(ADC1, DISABLE);
		}
	}

	if (change & PWR_CTRL_PWM) {
		if (PWR_Control & PWR_CTRL_PWM) {
			TIM_Cmd(TIM3, ENABLE);
		} else {
			TIM_Cmd(TIM3, DISABLE);
		}
	}

	if (change & PWR_CTRL_EN) {
		if (PWR_Control & PWR_CTRL_EN) {
			PWR_ENABLE(Bit_SET);
		} else {
			PWR_ENABLE(Bit_RESET);
		}
	}

	if (change & PWR_CTRL_ACS) {
		if (PWR_Control & PWR_CTRL_ACS) {
			PWR_ACSEL(Bit_SET);
		} else {
			PWR_ACSEL(Bit_RESET);
		}
	}

	PWR_Control_Last = PWR_Control;
}

void PWR_CurrentHandle() {
	/* setup PWM compare registers */
	TIM3->CCR3 = PWR_I_Set.i_bat & 0x3FFF;
	TIM3->CCR4 = PWR_I_Set.i_ac & 0x3FFF;
	TIM3->EGR |= TIM_EGR_UG;
}

/* Line 2 - ACPRES */
void EXTI2_IRQHandler(void) {
	if (PWR_ACPRES() == Bit_SET) {
		PWR_Status |= PWR_STAT_ACPRE;
	} else {
		PWR_Status &= ~PWR_STAT_ACPRE;
	}

	/* notify host */
	SYS_ChangeIntFlag(SYS_INT_PWRAC);
	
	/* Clear the EXTI line 9 pending bit */
	EXTI_ClearITPendingBit(EXTI_Line2);
}

/* Line5 - ALARM */
void EXTI9_5_IRQHandler(void) { 
	if (PWR_ALARM() == Bit_SET) {
		PWR_Status |= PWR_STAT_ALARM;
	} else {
		PWR_Status &= ~PWR_STAT_ALARM;
	}

	/* notify host */
	SYS_ChangeIntFlag(SYS_INT_PWRALARM);
	
	/* Clear the EXTI line 9 pending bit */
	EXTI_ClearITPendingBit(EXTI_Line5);
} 

