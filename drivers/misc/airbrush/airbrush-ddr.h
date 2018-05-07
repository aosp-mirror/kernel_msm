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

#include "airbrush-sm-ctrl.h"

#define DDR_SUCCESS	( 0)
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

void ab_ddr_train_gpio(void);
void ab_ddr_train_sysreg(void);
void ab_ddr_train_gpio_interrupt(void);
void ab_ddr_train_sysreg_interrupt(void);
void ab_ddr_save_training_results(void);
void ab_ddr_restore_training_results(uint32_t sram_address);
int32_t ab_ddr_init(struct ab_state_context *sc);

#endif /* _AIRBRUSH_DDR_H_ */
