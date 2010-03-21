#
#  Makefile for the Linux Controller Area Network core.
#

obj-$(CONFIG_CAN)	+= can.o
can-objs		:= af_can.o proc.o

obj-$(CONFIG_CAN_RAW)	+= can-raw.o
can-raw-objs		:= raw.o

obj-$(CONFIG_CAN_BCM)	+= can-bcm.o
can-bcm-objs		:= bcm.o

obj-$(CONFIG_CAN_ISOTP)	+= can-isotp.o
can-isotp-objs		:= isotp.o

obj-$(CONFIG_CAN_GW)	+= can-gw.o
can-gw-objs		:= gw.o
