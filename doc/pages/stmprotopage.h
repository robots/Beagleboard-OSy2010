/**
\file This file contains only documentation page for doxygen 

\page stmprotopage Communication protocol for STM32 firmware


Communication protocol is built on top of SPI with 2 dedicated interrupt lines: interrupt, data ready.

First interrupt line serves as interrupt notification to host, that interrupt condition was met.
Second interrupt line servers as data ready line. Indicates that host can continue with transaction. (Will be explaned later)

There are 2 commands implemented: read and write. Host can choose which data register is read from or written to. This is selected
by the MSB in the first byte. Rest of the byte (7bits) choose which register is being act upon.

Registers are fixed sizes, with different access modes implemented. RO, WO, RW - Read only, Write only, Read-write.

List of valid registers with their respective sizes and access mode is available in \ref STM32_BB/command.h

\section Reading from register

Host begins by sending first byte to device. MSB is 0 and rest contains register which should be read. After this byte has been shifted out
on the SPI bus, host needs to wait for "data ready" interrupt to be triggered. This signalises that device is ready to accept further communication.
After the "data ready" interrupt triggers host may read the register by sending dummy bytes on the bus. The number of dummy bytes is equal to size
of the register.

\section Writing to register.

Writing to register is similar to reading from it, except the MSB of first byte is 1. Instead of sending dummy bytes host sends actual content to be written to register.

*/
