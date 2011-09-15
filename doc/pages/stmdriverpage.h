/**
\file This file contains only documentation page for doxygen 

\page stmdriverpage Linux Driver - Counterpart For \ref stmfwpage "STM32 Firmware"

Communication with the MCU is done through SPI. SPI is synchronous multi-slave bus. This bus is reachable from linux kernel through SPI api.

During the device probing 2 interrupts are requested for this driver. One is bound to the SPI part of the device. And the other one is bound
to the netdev part. The first interrupt servers as "data ready" from the device, and notifies the driver that it can continute with transaction, 
second is generic interrupt that informs the host that some interrupt event occured.

\section stmdriverspi SPI communication

As the MCU has many other tasks to take into account, certain protocol was needed to be implemented on top of the SPI for the communication to work
reliably. This protocol is described \ref stmprotopage "here".

The way how SPI api is implemented prohibits to use SPI api directly from interrupt routine and workqueue has to be used. This adds some delay
between when the event of interrupt occuring and when the actual interrupt in serviced. This is accounted for loss of performance in this project.

\section stmdriverpower Power management

The power management of the expansion board is exposed through Power Supply api in kernel. This api allows defining different type of powersupplies.
Implemented are Battery and AC adaptor, System. For battery Charging current and voltage can be monitored. For AC adaptor voltage can be monitored,
and for system drain current can be monitored. This api does not support features like setting charging current or controlling the charger. These 
had to be implemented as Sysfs entries (placed in "power-control" directory of the device).

\section stmdrivercan Can controller

Can controller is implemented as SocketCAN device and uses netdev api. This makes CAN available through sockets to the userspace. As there is a bug
in the SocketCan timming calculation, all the bittimings need to be set manually for the communication to work. Reference clock is set to 36Mhz.

This driver works by sending 1 message at a time and waiting for send confirmation from the MCU. During this time the whole the network queue
is stoped. Upon correct sending it will send next available message. When error occures in the MCU, MCU's CAN controller is restarted to bring it
into known state.

When receiving messages host will read 16message buffer from the device, and pass messages to the netdev layer, stripping all invalid ones.

Various error statictics are captured in the driver and presented through SocketCAN api - arbitration lost, TX error, RX error, etc.


\ref stmprotopage "SPI protocol"

*/
