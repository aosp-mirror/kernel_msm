/*
 * linux/platform_data/cs40l20.h -- Platform data for CS40L20
 *
 * Copyright (c) 2017 Cirrus Logic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CS40L20_H
#define __CS40L20_H

struct cs40l20_platform_data {
	unsigned int boost_ind;
	unsigned int boost_cap;
	unsigned int boost_ipk;
	bool refclk_gpio2;
};

#endif /* __CS40L20_H */
