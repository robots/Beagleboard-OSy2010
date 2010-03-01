/*
 * SPI driver for STM32 family processors
 *
 * 2009-2010 Michal Demin
 *
 */

#ifndef SPI_H_
#define SPI_H_

/* DMA enable bit*/
#define CCR_ENABLE_Set      ((uint32_t)0x00000001)

#define SPI_IT_RXNE         (6)
#define SPI_IT_TXE          (7)

void SPI1_Slave_Init();

#endif

