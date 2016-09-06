#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
obj-$(CONFIG_TOUCHSCREEN_FTS) += fts_ts.o 
fts_ts-y += focaltech_core.o focaltech_ctl.o focaltech_ex_fun.o focaltech_gesture.o focaltech_flash.o 
extlibs :=$(PWD)/drivers/input/touchscreen/ft_gesture_32bit_lib_v1.0_20140820.a 
ldflagobj-y += fts_ts.o 
obj-y s-y +=--whole-archive $(extlibs) --no-whole-archive



obj-y	+=  focaltech_ctl.o
obj-y	+=  focaltech_ex_fun.o
obj-y	+=  focaltech_flash.o
obj-y	+=  focaltech_gesture.o
obj-y	+=  focaltech_core.o
obj-y	+=  focaltech_esd_protection.o
