
#include "power.h"

struct pwr_data_t PWR_Measurement_Data;
struct pwr_i_set_t PWR_I_Set;
uint16_t PWR_Control = 0x0000;
uint16_t PWR_Control_Last = 0x0000;
uint16_t PWR_Status;

void PWR_Init() {
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	/* Init DMA structure */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&PWR_Measurement_Data;
	DMA_InitStructure.DMA_BufferSize = sizeof(struct pwr_data_t);
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
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

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);

	/* calibrate ADC */
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET); 

/* let host enable this
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
*/

	/* setup timer for pwm output */
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
		if (PWR_Control & PWE_CTRL_PWM) {
			// TIM3 enable
		} else {
			// tim3 disable
		}
	}

	PWR_Control_Last = PWR_Control;
}

void PWR_CurrentHandle() {
	/* setup PWM comare registers */
	
}
