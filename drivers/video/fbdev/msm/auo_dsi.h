/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */



#ifndef AUO_DSI_H
#define AUO_DSI_H

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>

enum {
	ESD_NA = 0,
	ESD_AUO_U128BLX
};

struct mdss_dsi_ctrl_pdata;
struct mdss_panel_data;

ssize_t mdss_fb_set_boost_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
extern int dsi_auo_read_id_code(struct mdss_dsi_ctrl_pdata *ctrl);

ssize_t mdss_fb_get_boost_mode(struct device *dev,
		struct device_attribute *attr, char *buf);

extern int mdss_dsi_raydium_parse_dt(struct device_node *np,
		struct mdss_dsi_ctrl_pdata *ctrl_pdata);
extern int mdss_dsi_raydium_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl,
		char page, char addr, void (*fxn)(int), char *rbuf, int len);


extern void mdss_dsi_brightness_boost_on(struct mdss_dsi_ctrl_pdata *ctrl);

extern void mdss_dsi_brightness_boost_off(struct mdss_dsi_ctrl_pdata *ctrl);

extern int mdss_dsi_buck_boost_enable(struct mdss_panel_data*, int);

extern int dsi_auo_set_boost_mode(struct mdss_dsi_ctrl_pdata*, int);

extern void mdss_dsi_parse_esd_check_model(struct device_node *np,
		struct mdss_dsi_ctrl_pdata *ctrl);

extern void __mdss_dsi_check_esd_work(struct work_struct *work);

extern void mdss_dsi_raydium_panel_reset(struct mdss_panel_data *pdata,
		int enable);

#endif /*AUO_DSI_H */

