/**
\file This file contains only documentation page for doxygen 

\page encdriverpage Linux driver for Microchip ENC424j600 SPI Ethernet controller

This driver was based on enc28j60 driver by Claudio Lanconelli.

- Using banked reads / writes.
	- Several registers can be accessed from all banks.
- Write verification
	CONFIG_ENC424J600_WRITEVERIFY
- RX buffer
	- Rx buffer wrapping
	- Protected RX area
- PHY registers
- Locking
- Low power
- Autonegotiation
- Huge frames
- Link settings (enc424j600_setlink)
- Tx errors

\section encint Interrupts & work queues
- Interrupt work queue.
- Tx work queue

*/
