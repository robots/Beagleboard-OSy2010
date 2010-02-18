/*
 * CAN driver for STM32 family processors
 *
 * 2009-2010 Michal Demin
 *
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "platform.h"
#include "stm32f10x.h"

#include "can.h"

void CAN_Init() {
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN pin: TX */   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	CAN_DeInit();
	CAN_StructInit(&CAN_InitStructure);
	
}

/* TX interrupt */
void USB_HP_CAN1_TX_IRQHandler(void) {

}

/* RX0 fifo interrupt */
void USB_LP_CAN1_RX0_IRQHandler(void) {

}

/* RX1 fifo interrupt */
void CAN1_RX1_IRQHandler(void) {

}

void CAN1_SCE_IRQHandler(void) {

}

