#
## Makefile for the touchscreen drivers.
#
#
## Each configuration option enables a list of files.

# Touch Core Driver Common Header Code..!!
ccflags-y += -I$(srctree)/drivers/input/touchscreen/lge/

obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49408)           += touch_sw49408.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49408)           += touch_sw49408_prd.o
obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49408)           += touch_sw49408_abt.o
#obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49407_LUCYE)           += touch_sw49407_asc_lucye.o
#obj-$(CONFIG_LGE_TOUCH_LGSIC_SW49408)           += touch_filter.o
