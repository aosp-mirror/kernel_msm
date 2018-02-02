/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * Airbrush State Manager Control driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_SM_CTRL_H
#define _AIRBRUSH_SM_CTRL_H

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define __DEBUG_FS	/* Comment this if debugfs entry is not needed */
#define __GPIO_ENABLE	0x1
#define __GPIO_DISABLE	0x0

enum ab_sm_state {
	AB_SM_S0 = 0,	/* active high perf */
	AB_SM_S1 = 1,	/* active medium perf */
	AB_SM_S2 = 2,	/* active low perf */
	AB_SM_S3 = 3,	/* active background perf */
	AB_SM_S4 = 4,	/* suspend */
	AB_SM_S5 = 5,	/* off */
};

enum ab_sm_event {
	AB_SM_EV_THERMAL_MONITOR,	/* Thermal event */
	AB_SM_EV_DEVICE_ERROR,		/* Other device fail */
	AB_SM_EV_LINK_ERROR,		/* ... */
	// ...
};

typedef int (*ab_sm_callback_t)(enum ab_sm_event, uintptr_t data, void *cookie);

#define AB_DEV_SOC	0x0
#define AB_DEV_IPU	0x1
#define AB_DEV_TPU	0x2
#define AB_DEV_ALL	0xff

struct ab_state_context {
	struct platform_device *pdev;
	struct device *dev;
	int state;

	/* mutex for synchronization */
	struct mutex lock;

	/* pins used in bootsequence and state transitions */
	struct gpio_desc *soc_pwrgood;	/* output */
	struct gpio_desc *fw_patch_en;	/* output */
	struct gpio_desc *ab_ready;	/* input  */
	struct gpio_desc *ddr_sr;	/* output */
	struct gpio_desc *ddr_train;	/* output */
	struct gpio_desc *cke_in;	/* output */
	struct gpio_desc *cke_in_sense;	/* output */

	unsigned int ab_ready_irq; /* ab_ready_gpio irq */

	int otp_fw_patch_dis;	/* OTP info from Airbrush (DT property) */

	ab_sm_callback_t cb_event;	/* Event callback registered by the SM
					 */
	void		*cb_cookie;	/* Private data sent by SM while
					 *  registering event callback
					 */
	enum ab_sm_state cur_state;	/* current state of airbrush device */
#ifdef __DEBUG_FS
	struct dentry *d_entry;
#endif
};

struct ab_state_context *ab_sm_init(struct platform_device *pdev);
int ab_sm_set_state(struct ab_state_context *sc,
			uint32_t device, enum ab_sm_state s);
enum ab_sm_state ab_sm_get_state(struct ab_state_context *sc, uint32_t device);
int ab_sm_register_callback(struct ab_state_context *sc,
				ab_sm_callback_t cb, void *cookie);
int ab_bootsequence(struct ab_state_context *ab_ctx, bool patch_fw);
int ab_interrupt_M0(int tar_dev);

#endif /* _AIRBRUSH_SM_CTRL_H */
