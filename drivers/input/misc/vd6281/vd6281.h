/*
 *  vd6281.h - Linux kernel modules for rainbow sensor
 *
 *  Copyright (C) 2017 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
/*
 * Defines
 */
#ifndef VD6281_H
#define VD6281_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>

#define VD6281_DRV_NAME	"vd6281"

/*
 *  IOCTL register data structs
 */
struct vd6281_register {
	u32 is_read; /*1: read 0: write*/
	u32 reg_index;
	u32 reg_bytes;
	u32 reg_data;
	s32 status;
};


/*
 *  driver data structs
 */
struct vd6281_data {

	u8   I2cDevAddr;
	u8   comms_type;
	u16  comms_speed_khz;

	struct i2c_data *client_object;

	struct regulator *vdd;

	const char *dev_name;

	/* misc device */
	struct miscdevice miscdev;

	int irq;
	int irq_gpio;
	unsigned int reset;

	struct mutex work_mutex;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_active;

	/* Debug */
	unsigned int enableDebug;
	u8 interrupt_received;
};

/*
 *  function pointer structs
 */
struct vd6281_module_fn_t {
	int (*init)(void);
	void (*deinit)(void *);
	int (*power_up)(void *, unsigned int *);
	int (*power_down)(void *);
	int (*query_power_status)(void *);
};

struct i2c_data {
	struct i2c_client *client;
	struct regulator *vdd;
	u8 power_up;
};
int vd6281_init_i2c(void);
void vd6281_exit_i2c(void *i2c_object);
int vd6281_power_up_i2c(void *i2c_object, unsigned int *preset_flag);
int vd6281_power_down_i2c(void *i2c_object);


#endif /* VD6281_H */
