#
#  Makefile for the Linux Controller Area Network drivers.
#

obj-$(CONFIG_CAN_VCAN)		+= vcan.o

obj-$(CONFIG_CAN_DEV)		+= can-dev.o
can-dev-y			:= dev.o
can-dev-$(CONFIG_CAN_DEV_SYSFS) += sysfs.o

obj-$(CONFIG_CAN_STM32BB)	+= stm32bb.o
obj-$(CONFIG_CAN_SJA1000)	+= sja1000/
obj-$(CONFIG_CAN_MSCAN)		+= mscan/

obj-$(CONFIG_CAN_MCP251X)	+= mcp251x.o

ccflags-$(CONFIG_CAN_DEBUG_DEVICES) := -DDEBUG
