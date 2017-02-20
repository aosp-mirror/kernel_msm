/*
 * touch_i2c.h
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef TOUCH_I2C_H
#define TOUCH_I2C_H

extern int touch_i2c_read(struct i2c_client *, struct touch_bus_msg *msg);
extern int touch_i2c_write(struct i2c_client *, struct touch_bus_msg *msg);
extern int touch_i2c_device_init(struct touch_hwif *hwif, void *driver);
extern void touch_i2c_device_exit(struct touch_hwif *hwif);

#endif
