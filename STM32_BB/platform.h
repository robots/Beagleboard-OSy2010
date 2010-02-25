#ifndef PLATFORM_H_
#define PLATFORM_H_

#define LED_YELLOW_PIN     GPIO_Pin_13
#define LED_YELLOW_GPIO    GPIOA

#define LED_GREEN_PIN      GPIO_Pin_14
#define LED_GREEN_GPIO     GPIOA

#define LED_RED_PIN        GPIO_Pin_15
#define LED_RED_GPIO       GPIOA

#define SPI_INT_PIN        GPIO_Pin_8
#define SPI_INT_GPIO       GPIOA


#define LED_YELLOW(x)      GPIO_WriteBit(LED_YELLOW_GPIO, LED_YELLOW_PIN, x)
#define LED_GREEN(x)       GPIO_WriteBit(LED_GREEN_GPIO, LED_GREEN_PIN, x)
#define LED_RED(x)         GPIO_WriteBit(LED_RED_GPIO, LED_RED_PIN, x)

#define SPI_INT_WRITE(x)   GPIO_WriteBit(SPI_INT_GPIO, SPI_INT_PIN, x)

#define TAMP_READ          GPIO_ReadInputDataBit(TAMP_GPIO, TAMP_PIN)
#endif

