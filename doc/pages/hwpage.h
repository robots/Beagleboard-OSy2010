/**
\file This file contains only documentation page for doxygen 

\page hwpage Hardware description

\image html main_small.jpg "From top to bottom: USB hub, expansion board and Beagleboard."


\section hwpagebeagle Beagleboard
The system is based on Beagleboard. Low cost OMAP3530 development board. More about this board can be found here http://beagleboard.org
Beagleboard provides expansion connector with multiple processor peripherals exposed - SPI, MMC, GPIO, etc.

\section hwpageexpansion Expansion Board

Our custom expansion board contains \ref hwpageenc "ethernet controller chip", STM32 mircocontroller,
battery managenment. This board provides CAN bus and Ethernet connectivity, battery charging and
power managenment for the whole system.

- level translator

- popisat ako funguje stm32

- popisat ako funguje nabijacka

Our expansion board connects to the epansion header of Beagleboard, adds
several peripherials and exposes more UART and I2C interfaces of Beagleboard.

- Real time clock with battery
- \ref hwpageenc "10/100Mbit Ethernet"
- STM32 MCU (Cortex-M3 based)
	- CAN bus
	- Battery/power management
Exposes more BB peripherals - UART, I2C

\image html system.svg "Expansion Board Connections"

\subsection hwpageenc Microchip ENC424j600
Microchip ENC424j600 is an fast ethernet controller with integrated MAC and PHY.
It is designed to be easily added to a microcontroller only with a minimum external
components.

Its features include:
- 24kB SRAM for TX and RX buffers
- HW crypto engines (AES, MD5, SHA1, ...)
- SPI and parallel interface

The chip is connected via the SPI interface and uses one interrupt channel.

*/
