# FTM5 support
obj-$(CONFIG_TOUCHSCREEN_FTS)	+= ftm5.o
ftm5-objs = fts.o fts_proc.o
ftm5-objs += \
	fts_lib/ftsCompensation.o fts_lib/ftsCore.o fts_lib/ftsError.o \
	fts_lib/ftsFrame.o fts_lib/ftsIO.o fts_lib/ftsTest.o fts_lib/ftsTime.o \
	fts_lib/ftsTool.o fts_lib/ftsFlash.o fts_lib/ftsGesture.o

# TODO: remove me b/62057517
subdir-ccflags-y += \
	-Wno-strncat-size \
	-Wno-typedef-redefinition \
	-Wno-parentheses-equality \
