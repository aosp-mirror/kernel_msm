/* Copyright (c) 2016, The Linux Foundation. All rightsreserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PN551_H_
#define _PN551_H_

#define PN551_I2C_NAME "pn551"

#define PN551_MAGIC	0xE9
#define PN551_SET_PWR	_IOW(PN551_MAGIC, 0x01, unsigned int)
#define IO_WAKE_LOCK_TIMEOUT (2*HZ)

struct pn551_i2c_platform_data {
	void (*gpio_init) (void);
	unsigned int irq_gpio;
	uint32_t irq_gpio_flags;
	unsigned int ven_gpio;
	uint32_t ven_gpio_flags;
	unsigned int firm_gpio;
	uint32_t firm_gpio_flags;
	unsigned int pvdd_en;
	uint32_t pvdd_en_flags;
	unsigned int ven_isinvert;
};

#endif
