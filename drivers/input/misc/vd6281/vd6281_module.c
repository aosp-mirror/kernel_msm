/*
 *  vd6281_module.c - Linux kernel modules for rainbow sensor
 *
 *  Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
/*
 * API includes
 */

#include "vd6281.h"

/*
 * Global data
 */

static struct vd6281_module_fn_t vd6281_module_func_tbl = {
	.init = vd6281_init_i2c,
	.deinit = vd6281_exit_i2c,
	.power_up = vd6281_power_up_i2c,
	.power_down = vd6281_power_down_i2c,
};

static int vd6281_init_client(struct vd6281_data *data);
static int vd6281_start(struct vd6281_data *data, u8 scaling,
			int mode);
static int vd6281_stop(struct vd6281_data *data);

/*
 * Initialization function
 */
static int vd6281_init_client(struct vd6281_data *data)
{

	struct vd6281_data *vd6281_dev = data;

	vd6281_dev->I2cDevAddr      = 0x40;
	vd6281_dev->comms_type      =  1;
	vd6281_dev->comms_speed_khz =  400;

	/* Test */
	vd6281_start(data, 0, 0);

	vd6281_stop(data);

	return 0;
}


static int vd6281_start(struct vd6281_data *data, u8 scaling,
	int mode)
{
	int rc = 0;

	/* Power up */
	rc = vd6281_module_func_tbl.power_up(data->client_object, &data->reset);
	if (rc) {
		pr_err("%s: %d,error rc %d\n", __func__, __LINE__, rc);
		return rc;
	}
	/* init */
	rc = vd6281_init_client(data);
	if (rc) {
		pr_err("%s: %d, error rc %d\n", __func__, __LINE__, rc);
		vd6281_module_func_tbl.power_down(data->client_object);
		return rc;
	}

	return rc;
}

static int vd6281_stop(struct vd6281_data *data)
{
	int rc = 0;

	/* power down */
	rc = vd6281_module_func_tbl.power_down(data->client_object);
	if (rc)
		pr_err("%s: %d failed with %d\n", __func__, __LINE__, rc);

	return rc;
}

/*
 * I2C init/probing/exit functions
 */
static const struct file_operations vd6281_fops = {
	.owner =			THIS_MODULE,
};

static int __init vd6281_init(void)
{
	int ret = -1;

	/* client specific init function */
	ret = vd6281_module_func_tbl.init();

	if (ret)
		pr_err("%s: %d failed with %d\n", __func__, __LINE__, ret);

	return ret;
}

static void __exit vd6281_exit(void)
{
}

MODULE_DESCRIPTION("ST rainbow sensor");
MODULE_LICENSE("GPL");

module_init(vd6281_init);
module_exit(vd6281_exit);

