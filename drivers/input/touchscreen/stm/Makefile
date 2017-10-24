# FTM5 support
obj-$(CONFIG_TOUCHSCREEN_FTS)	+= fts.o fts_proc.o fts_lib/

# Legacy driver
obj-$(CONFIG_TOUCHSCREEN_FTM4)	+= ftm4.o
ftm4-objs			+= ftm4_ts.o ftm4_fwu.o ftm4_pdc.o

# TODO: remove me b/62057517
subdir-ccflags-y += \
	-Wno-strncat-size \
	-Wno-typedef-redefinition \
	-Wno-parentheses-equality \
