/**
\file enc424j600.c
*/

/**
\page encdriverpage Linux Driver For Microchip ENC424j600 Ethernet Controller

Our enc424j600 driver is based on enc28j60 driver by Claudio Lanconelli.

\section encdriverfeatures Supported Features
- 10/100Mb
- Full/half duplex
- Low-power mode
- Autonegotiation


\section encdriverunsupported Unsupported Functionality
- Cryptographic engines
- DMA (within enc424j600's SRAM)


\section encdriverspi SPI Communication
The driver uses chip's SPI interface.
The lowest level of this communication is handled in function enc424j600_spi_trans().
SPI communication may use DMA (in the host computer),
if it is enabled using the enc424j600_enable_dma module parameter.

\subsection encdriverprotocol Protocol
The SPI protocol uses commands consisting of command identifier sent to the chip and
optional data (sent or received).
This protocol is one-directional and doesn't use the SPI feature of transmitting and
receiving at the same time (other direction is always ignored).


\section encdriverreg Registers
Apart from moving packets to and from the chip, most SPI communication consists of
setting and reading special function registers of enc424j600.

Registers are 8 or 16 bits wide, divided into four 32 byte long banks, and a group of unbanked registers.

Bank is selected by issuing a special command before the register access.
Additionally all registers may be accessed using unbanked commands with longer addressing.

Our driver doesn't need the unbanked registers, so all registers can work with
the banked addressing.
To achieve this the driver remembers last accessed bank and sends the change
bank command if this is different from the currently used register.

Register access commands don't have a fixed payload length, only a starting register
address is specified.
Reading 16 bit registers thus means using a read command with the address of the lower
half of the register and size 2.

We have chosen to support both 8 bit and 16 bit access to 16 bit registers.
This means that for example to read ECON1 register, ECON1L must be used as an argument
to enc424j600_read_16b_sfr().

Functions for manipulation with registers:
- enc424j600_read_8b_sfr()
- enc424j600_write_8b_sfr()
- enc424j600_read_16b_sfr()
- enc424j600_write_16b_sfr()
- enc424j600_set_bits()
- enc424j600_clear_bits()

\subsection encdrivermacphy MAC and PHY Registers
The above properties of the registers only hold for eth registers
(with names starting in "E").

MAC registers with names starting with "M" must be accessed as whole 16 bit at a time
and can't be targets of the bit set/clear commands or 8 bit reads and writes.

PHY (names with "PH") registers cannot be accessed with regular commands for reading
and writing registers.
Access a PHY register consists of several reads and writes of MAC registers,
these are contained in functions enc424j600_phy_read() and enc424j600_phy_write().


\section encdriversram SRAM

ENC424j600 has 24kB of SRAM.
There is an area at the end of the memory used for circular buffer for received packets,
the rest of the memory can be used as transmit buffers or general purpose storage.

Commands for reading and writing SRAM are in functions enc424j600_read_sram() and
enc424j600_write_sram().

\subsection encdriverrxbuffer RX buffer
See \ref encdriverrx

Hardware appends received packets to the end of the buffer, wrapping around when the end is reached.
Part of the reveive buffer is marked as protected to keep unprocessed received packets
from being overwritten.

Size of the RX area is set during chip configuration.
Our driver sets it to 15 * ::MAX_FRAMELEN bytes.

enc424j600_read_rx_area() reads packets from the receive buffer and handles
wrapping at its end, enc424j600_clear_unprocessed_rx_area() sets the protected area
to be 2 bytes large (the smallest possible value).


\section encdriverwriteverify Write Verification
The module configuration option CONFIG_ENC424J600_WRITEVERIFY enables write verification.

With write verification all register writes and first 64 bytes
of every SRAM write are read back and verified.

This feature is obviously useful only when debugging the module, chip connection, etc.


\section encdriverinit Chip Initialization and Reset

The function enc424j600_hw_init() brings the chip to a well defined
initial state, with disabled interrupts.

After this function is called for a first time, it is necessary to 
read MAC address burned into the chip.
This is done using enc424j600_get_hw_macaddr().


\section Power Modes
Enc424j600 is capable of switching the low power mode.
In this mode MAC and PHY are disabled.

Low power mode is entered after the first chip reset and every time
the network device is stopped (ifconfig eth0 down).


\section Packet Filters and Promiscuous Mode
Enc424j600 supports several packet filters organized into a filtering
pipeline, each of them can be individually enabled or disabled.

Packets rejected by these filters are ignored.

Our driver uses only three combinations of these filters:
Normal mode in which only correct unicast packets to this network interface
are received, Multicast mode that adds multicast packets and promiscuous mode
that receives all correct packets.

Function enc424j600_set_hw_filters() immediately sets the value to the
chip, function enc424j600_set_multicast_list() is a part of the netdev interface
that wraps around the former function (Schedules a workqueue to write the values).


\section encdrivermac MAC Address
The chip comes with preprogrammed MAC address.
After the chip is initialized for a first time, this address is read
into a driver's data structure.
This is done in function enc424j600_get_hw_macaddr().

Function enc424j600_set_mac_address() is a part of netdev interface that
changes the address, the actual writing of the address is done in
enc424j600_set_hw_macaddr().


\section encdrivertx Transmitting
Function enc424j600_send_packet() is a part of netdev interface.

The actual transmission of packets is done in the tx_work work queue,
by the function enc424j600_hw_tx().

We allow only a single packet to be waiting for TX.
When a packet is to be transmitted, we stop the net queue and start it
when the packet transmission is (successfully or not) over.
This makes it possible to use only a small part of the SRAM as a TX buffer.

In case of TX timeout, the driver restarts the chip.

\subsection encdrivertximprovement Potential Performance Improvement
Performance of packet transmission could be marginally improved by
increasing the size of TX buffer and preloading more packets into the
chip's SRAM while a packet is being transmitted to the wire.

However since ethernet is about 10x faster than our SPI connection,
this would yield only about 10% speed improvement in the best case,
so we chose to not use this.


\section encdriverrx Receiving
When an packet pending interrupt is serviced, we read the packet counter register.
This register contains number of unprocessed packets and when it is nonzero it
triggers the interrupt.

Function enc424j600_hw_rx() reads individual packets from RX buffer.

We keep an pointer into chip SRAM where next packet will be placed.
At the beginning of a received packet there is a Receive status vector,
that contains information about the packet, including status bits, length,
pointer to next packet, ...
These are used to check correctness of the packet and update statistics.

If the received packet is OK, we allocate a new skb, read the packet 
data into it and hand it to OS.

To finalize each receive, the memory occupied by the packet is unprotected
(enc424j600_clear_unprocessed_rx_area()) and packet counter is decremented.


\section encdriverirq Interrupts
In addition to SPI there is one interrupt line between enc424j600 and the host system.
Handler of this IRQ (enc424j600_irq()) only schedules the work queue that later does
some actual work.

Function enc424j600_irq_work_handler() turns off interrupts of enc424j600 and checks 
individual interrupt flags.

\subsection encdriverirqlink Link Change
This interrupts occurs when PHY detects change in link status.
See \ref encdriverlink.

\subsection encdriverirqtx TX Complete
TX complete interrupt is raised on successfull transmit of a packet.
See \ref encdrivertx and enc424j600_tx_clear().

\subsection encdriverirqtxabt TX Abort
Failed transmit raises this interrupt.
Handling is similar to the previous case, only here reason of the failure
is examined and counted in addition to simply clearing TX.

\subsection encdriverirqrxabt RX Abort
This interrupt happens on unsuccessful receive, when a receive tries to
overwrite a \ref encdriverrxbuffer "protected area" in a RX buffer, or
when packet counter would overflow.
These situations are only counted.

\subsection encdriverirqrx Receive Packet Pending
This is by far the most used interrupt.

It is serviced by reading the packet counter (number of unprocessed
packets), and calling function enc424j600_hw_rx() for each of them.
See \ref encdriverrx.


\section encdriverlink Link Detection and Autonegotiation

Core of link detection is in the function enc424j600_check_link_status().
It gets called after a link change interrupt and after opening the net device.

This function reads PHYLNK bit, which tells us if we have link signal.
If autonegotiation is enabled the driver has to wait for the autonegotiation to
finish and then update the status fields for ethtool according to the real
settings.


*/
