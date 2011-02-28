/*
 * SPI Slave driver for STM32 family processors
 *
 * 2010 Michal Demin
 *
 */

#ifndef SPI_H_
#define SPI_H_

#ifndef NULL
#define NULL 0
#endif

/* DMA enable bit*/
#define CCR_ENABLE_Set      ((uint32_t)0x00000001)

#define SPI_IT_RXNE         (6)
#define SPI_IT_TXE          (7)

// bit-band regions
#define DMA1CH2EN (*((volatile unsigned long *) 0x42400380))
#define DMA1CH2GL (*((volatile unsigned long *) 0x42400090))
#define DMA1CH3EN (*((volatile unsigned long *) 0x42400600))
#define DMA1CH3GL (*((volatile unsigned long *) 0x424000A0))

#define SPI1TXEI  (*((volatile unsigned long *) 0x4226009C))
#define SPI1RXNEI (*((volatile unsigned long *) 0x42260098))

typedef void (*DMA_Callback_t)();

void SPI1_Slave_Init();

#endif

