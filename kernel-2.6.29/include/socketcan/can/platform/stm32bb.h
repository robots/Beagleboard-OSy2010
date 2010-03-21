#ifndef __CAN_PLATFORM_STM32BB_H__
#define __CAN_PLATFORM_STM32BB_H__


#include <linux/spi/spi.h>

/**
 * struct stm32bb_platform_data - STM32BB SPI CAN controller platform data
 * @board_specific_setup:       - called before probing the chip (power,reset)
 * @transceiver_enable:         - called to power on/off the transceiver
 * @power_enable:               - called to power on/off the mcp *and* the
 *                                transceiver
 *
 * Please note that you should define power_enable or transceiver_enable or
 * none of them. Defining both of them is no use.
 *
 */

struct stm32bb_platform_data {
	int (*board_specific_setup)(struct spi_device *spi);
	int (*transceiver_enable)(int enable);
	int (*power_enable) (int enable);
};

#endif /* __CAN_PLATFORM_MCP251X_H__ */
