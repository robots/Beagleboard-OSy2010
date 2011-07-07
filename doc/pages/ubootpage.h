/**
\file This file contains only documentation page for doxygen 

\page ubootpage U-boot modification

Out \ref hwpage "custom hardware" uses SPI to exchange information with the beagleboard.
Stock configuration does not enable SPI peripheral on the expansion connector.

\section ubootpinmux Pin multiplexing on ARM processor.
ARM processors provide much more peripherals than they have available connectors. These 
peripherals share multiple pins - so called pin multiplexing. 

Pin multiplexing is usually part of GPIO peripheral, which chooses which peripheral will have access
to physical pin. This is usually controlled by running software and setup during the boot process.
Except in some special cases there is no need to reconfigure the pin muxing during runtime as the HW
does not change at all.

\section ubootbbmux Beagleboard specific files

On beagleboard U-boot is in charge of setting the pin muxing correctly. Beagleboard specific configuration
can be found in:

u-boot-bb/board/omap3/beagle/

The pin muxing configuration is in: 

u-boot-bb/board/omap3/beagle/beagle.h

Complete documentation to pin multiplexing on beagleboard can be found in Technical Reference Manual (TRM) on this page http://focus.ti.com/docs/prod/folders/print/omap3530.html

\section ubootchange Our change to the u-boot

Lines 204-216 in beagle.h have been changed to SPI3, and GPIO. blah

*/
