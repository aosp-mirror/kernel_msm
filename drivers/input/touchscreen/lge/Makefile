#
# Makefile for lge touch core driver
#

# Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_core_base.o

touch_core_base-objs			+= touch_core.o \
					   touch_core_sysfs.o \
					   touch_common.o \
					   touch_i2c.o \
					   touch_spi.o \
					   touch_model.o \
					   touch_notify.o \
					   touch_hwif.o

obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= lgsic/
