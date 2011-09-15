/**
\file This file contains only documentation page for doxygen 

\page hwpage Hardware description

\image html main_small.jpg "From top to bottom: USB hub, expansion board and Beagleboard."


\section hwpagebeagle Beagleboard
The system is based on Beagleboard. Low cost OMAP3530 development board. More about this board can be found here http://beagleboard.org
Beagleboard provides expansion connector with multiple processor peripherals exposed - SPI, MMC, GPIO, etc.

There are few downsides to this board. It is missing battery backed RTC, ethernet connectivity. These features are
added by our custom expansion board. 

This custom expansion board contains ethernet controller chip (with embedded MAC + PHY), STM32 microcontroller,
battery managenment and RTC (real time clock). This board provides CAN bus and Ethernet connectivity, battery charging and
power managenment for the whole system. Also additional peripherals from OMAP processor are exposed.

As the OMAP cpu works with 1.8V logic most connections to the expansion board are driven through level translators to protect 
the CPU from over-voltage.

\section hwpagecharger Power management

The battery charger is based on BQ24702 battery management chip. This chip provides charging capabilities for external battery. Either
Pb chemistry, or with little circuit modifications also different chemistries are supported (Li-ion, Li-pol, NiMH). This chip also 
information about external AC adaptor connection/disconnection, battery voltage monitoring, charging current monitoring, and dynamic
power supply handover in case battery voltage is critical. More can be found in datasheet (href bq24702.pdf)

\section hwpagestm STM32

STM32 MCU is STM32F103 chip with 32kB flash memory and 20kB SRAM with lot's of peripherals, such as CAN controller, 4 Timers (each capable of PWM
generation), SPI (master/slave capable). This MCU is connected to the OMAP3530 using SPI bus (SPI #3 CS0), with 2 dedicated interrupt lines (general
purpose interrupt and data ready signal). The maximal SPI clock speed is ~36MHz, resulting in 36Mbit/s transfer rate.
This chip is in charge of the controlling of the battery charger and also provides CAN bus connection.

\subsection hwpagecan CAN

The internal CAN controller interface is exposed, and adds some modifications: 16 message buffer for incoming messages. This
interface is available as set of registers, which can be modified/read/written to by the SPI bus.
For more information about the Can controller, please refer to (STM32 Cortex-M3 family reference manual, section x - bxCan controller)

The charger's interface consists of data measurement registers, current setting registers and control registers. Data measurement register is read-only
register and provides host with actual measurements of battery charging current, system drain current, battery voltage and AC adaptor voltage. The
current setting register sets maximal charging current for the battery and maximal current that the AC adaptor is able to handle.

\subsection hwpageinterrupts interrupts

The MCU also provides interrupts multiplexing. Various interrupts such as AC plugged/unplugged, Battery alarm, CAN TX, CAN RX interrupts are
received and saved as pending interrupts. Host is able to configure upon which interrupt should the external interrupt be triggered.
For complete list of interrupts available see (reference to sys.h)

\subsection hwpageenc Microchip ENC424j600
Microchip ENC424j600 is an fast ethernet controller with integrated MAC and PHY.
It is designed to be easily added to a microcontroller only with a minimum external
components.

Its features include:
- 24kB SRAM for TX and RX buffers
- HW crypto engines (AES, MD5, SHA1, ...)
- SPI and parallel interface

The chip is connected via the SPI interface and uses one interrupt channel.

\section hwpageethernet Ethernet

Ethernet connectivity is provided by ENC424J600 chip. This chip integrates ethernet MAC and PHY in one package, this fact simplifies the design a lot.
ENCxx provides multiple interfaces to communicate with. For ease of design we choose SPI to connect to the OMAP3530 CPU. (SPI4 negCS0). Maximal clock speed
for this chip is ~14MHz, resulting in 14Mbit/s transfer rate.

\section hwpagertc RTC

RTC is PCF8536, with Super CAP backup. The theoretical calculated backup time is >1week without power supply. This chip it connected to the I2C #2
bus on the OMAP3530.

\subsection hwpagecharger Charger and Power Management
\todo charger

*/
