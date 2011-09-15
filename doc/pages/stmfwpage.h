/**
\file This file contains only documentation page for doxygen 

\page stmfwpage Firmware for embedded STM32 MCU

Firmware in STM32 chip takes care of several tasks:
 - Battery charging/power managenment
 - CAN
 - SPI slave
 - External interrupt handling

\section Battery charging and power managenment

Battery charging is managed by external chip. The charging process itself cannot be controled from
the firmware. What we control is:
 - Charging current
 - Maximal current drawn from AC adaptor
 - Selection of power source (battery/AC)
 - Enable/Disable charging

All of these features are configurable through expodes registers via SPI interface.

Selection of power source has built in fallback in case battery voltage drops below 10.7V. In case
of such event ALARM interrupt is generated to inform host. If there is no fallback (AC adaptor missing)
user should take care to provide power through AC adaptor.

In case of plug-in/out of AC adapor ACPRES interrupt is generated.

Both current settings are controller using 1 timer serving as PWM generator.

On the current HW we monitor:
 - Charging current
 - System draw current
 - AC adaptor voltage
 - Battery voltage

We are using ADC with DMA controller to sample these values and to write them to the correct place in the memory.
By using the DMA we offload the CPU.

\section CAN - Controller area network

This part of firmware is a wrapper around physical CAN controller available in STM32-family MCU. 
It features 16message circular buffer to store incomming messages.

Circular buffer has been implemented to prevent messages being lost during burst (on SYNC, etc) as the SPI interface in Linux has some latency issues
(due to scheduling)

During the writing of the firmware we discovered HW bug. The error flag in the can controller is not reset after the error condition has been removed
and error interrupts keep occuring. We have implemented workaround, that resets the CAN controller hardware in case of error, and reinitializes it 
into state before the reset condition appeared.


\section SPI Slave

- spi slave , popisat ako funguje, a dovod data ready

- interrupty 
 - odpojenie AC, vybita baterka, ...
 - can recv, can error, can TX ok/nok....


\ref stmprotopage "Pouzity protokol"

*/
