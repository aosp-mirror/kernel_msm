/*
 * fan49103.h - Fairchild Regulator FAN49103 Driver
 *
 * Copyright (C), 2008-2019, OPPO Mobile Comm Corp., Ltd.
 * Author: Zeng Zhaoxiu <zengzhaoxiu@oppo.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __FAN49103_H__
#define __FAN49103_H__

struct fan49103_platform_data {
	struct regulator_init_data *regulator;
	unsigned int init_uv;
	int gpio_en;
	int gpio_pg;
};

#endif /* __FAN49103_H__ */
