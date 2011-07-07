/**
\file This file contains only documentation page for doxygen 

\page hwpage Hardware description

\section hwpagebeagle Beagleboard
The system is based on Beagleboard. Low cost OMAP3530 development board. More about this board can be found here http://beagleboard.org
Beagleboard provides expansion connector with multiple processor peripherals exposed - SPI, MMC, GPIO, etc.

Our custom expansion board contains ethernet controller chip (with embedded MAC + PHY), STM32 mircocontroller,
battery managenment. This board provides CAN bus and Ethernet connectivity, battery charging and
power managenment for the whole system.

- level translator

- popisat ako funguje stm32

- popisat ako funguje nabijacka

\section hwpageenc Microchip ENC424j600
Microchip ENC424j600 is an fast ethernet controller with integrated MAC and PHY.
It is designed to be easily added to a microcontroller only with a minimum external
components.

Its features include:
- 24kB SRAM for TX and RX buffers
- HW crypto engines (AES, MD5, SHA1, ...)
- SPI and parallel interface

The chip is connected via the SPI interface and uses one interrupt channel.


\image html system.svg "Expansion Board Connections"

*/
