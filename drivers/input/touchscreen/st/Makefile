# SPDX-License-Identifier: GPL-2.0-only
#
## Makefile for the STMicroelectronics touchscreen driver.
#

obj-$(CONFIG_TOUCHSCREEN_ST) += stm-fts.o
stm-fts-objs += fts.o fts_gui.o fts_driver_test.o fts_aoi_event.o
stm-fts-objs += $(patsubst %,fts_lib/%,ftsCompensation.o \
	ftsCrossCompile.o ftsError.o ftsFrame.o ftsIO.o ftsTest.o \
	ftsTime.o ftsTool.o ftsFlash.o ftsGesture.o)

