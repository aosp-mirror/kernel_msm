/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_DDR_H_
#define _AIRBRUSH_DDR_H_

#include <linux/airbrush-sm-ctrl.h>

#define DDR_SUCCESS	(0)
#define DDR_FAIL	(-1)

#define DDR_TRAIN_STARTED		(0x1 << 0)
#define DDR_TRAIN_INIT_DONE		(0x1 << 1)
#define DDR_TRAIN_VREF_DONE		(0x1 << 2)
#define DDR_TRAIN_REPEAT_DONE		(0x1 << 3)
#define DDR_TRAIN_COMPLETE		(0x1 << 4)
#define DDR_TRAIN_RESTORE_STARTED	(0x1 << 5)
#define DDR_TRAIN_RESTORE_COMPLETE	(0x1 << 6)
#define DDR_TRAIN_SAVE_STARTED		(0x1 << 7)
#define DDR_TRAIN_SAVE_COMPLETE		(0x1 << 8)
#define DDR_TRAIN_FAIL			(0x1 << 9)

#define DDR_BOOT_TEST_WRITE		(0x1 << 0)
#define DDR_BOOT_TEST_READ		(0x1 << 1)
#define DDR_TEST_MEMTESTER		(0x1 << 4)
#define DDR_TEST_PCIE_DMA		(0x1 << 5)
#define DDR_TEST_NOPRINT		(0x1 << 31)
/* size should be multiple of 1MB */
#define DDR_TEST_PCIE_DMA_SIZE_MB(x)	(((x) & (0x3ff << 8)) >> 8)
#define DDR_TEST_EYE_MARGIN_SAMPLES(x)	(((x) & (0xff << 20)) >> 20)
#define DDR_BOOT_TEST_READ_WRITE (DDR_BOOT_TEST_READ | DDR_BOOT_TEST_WRITE)

#define DDR_TEST_PCIE_DMA_READ(x)	\
	(DDR_TEST_PCIE_DMA | DDR_BOOT_TEST_READ | (x << 8))

#define DDR_TEST_PCIE_DMA_WRITE(x)	\
	(DDR_TEST_PCIE_DMA | DDR_BOOT_TEST_WRITE | (x << 8))

#define DDR_TEST_PCIE_DMA_READ_WRITE(x)	\
	(DDR_TEST_PCIE_DMA_READ(x) | DDR_TEST_PCIE_DMA_WRITE(x))

int32_t ab_ddr_train_gpio(struct ab_state_context *sc);
int32_t ab_ddr_train_sysreg(struct ab_state_context *sc);

int ab_ddr_init(struct ab_state_context *sc);
int ab_ddr_suspend(struct ab_state_context *sc);
int ab_ddr_resume(struct ab_state_context *sc);
int ab_ddr_selfrefresh_enter(struct ab_state_context *sc);
int ab_ddr_selfrefresh_exit(struct ab_state_context *sc);
int ab_ddr_setup(struct ab_state_context *sc);
int ab_ddr_read_write_test(unsigned int read_write);
int ab_ddr_freq_change(struct ab_state_context *sc, int val);
int ab_ddr_measure_eye(struct ab_state_context *sc, unsigned int test_data);

#endif /* _AIRBRUSH_DDR_H_ */
