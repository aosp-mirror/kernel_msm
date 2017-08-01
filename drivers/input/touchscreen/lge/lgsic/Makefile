#
## Makefile for the touchscreen drivers.
#
#
## Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49408)     += sw49408.o
sw49408-objs				  += touch_sw49408.o \
					     touch_sw49408_abt.o

obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49408_PRD) += touch_sw49408_prd.o
#obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49408)    += touch_filter.o

#TODO: remove me b/62058353
subdir-ccflags-y += \
	-Wno-duplicate-decl-specifier
