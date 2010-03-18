/*
 * Platform specific file
 *
 * 2010 Michal Demin
 *
 */
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

#define PWR_ENABLE_PIN     GPIO_Pin_7
#define PWR_ENABLE_GPIO    GPIOB

#define PWR_ACSEL_PIN      GPIO_Pin_6
#define PWR_ACSEL_GPIO     GPIOB

#define PWR_ALARM_PIN      GPIO_Pin_5
#define PWR_ALARM_GPIO     GPIOB

#define PWR_ACPRES_PIN     GPIO_Pin_2
#define PWR_ACPRES_GPIO    GPIOB

#define LED_YELLOW(x)      GPIO_WriteBit(LED_YELLOW_GPIO, LED_YELLOW_PIN, x)
#define LED_GREEN(x)       GPIO_WriteBit(LED_GREEN_GPIO, LED_GREEN_PIN, x)
#define LED_RED(x)         GPIO_WriteBit(LED_RED_GPIO, LED_RED_PIN, x)

#define SPI_INT_WRITE(x)   GPIO_WriteBit(SPI_INT_GPIO, SPI_INT_PIN, x)

#define PWR_ENABLE(x)      GPIO_WriteBit(PWR_ENABLE_GPIO, PWR_ENABLE_PIN, x)
#define PWR_ACSEL(x)       GPIO_WriteBit(PWR_ACSEL_GPIO, PWR_ACSEL_PIN, x)

#define PWR_ALARM()        GPIO_ReadInputDataBit(PWR_ALARM_GPIO, PWR_ALARM_PIN)
#define PWR_ACPRES()       GPIO_ReadInputDataBit(PWR_ACPRES_GPIO, PWR_ACPRES_PIN)
#endif

