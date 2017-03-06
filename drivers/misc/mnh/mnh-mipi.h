/*
 *
 * MNH State Manager MIPI Driver
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef __MNH_MIPI
#define __MNH_MIPI

#include <uapi/linux/mnh-sm.h>

int mnh_mipi_config(struct device *dev, struct mnh_mipi_config cfg);
void mnh_mipi_stop_device(struct device *dev, int txdev);
void mnh_mipi_stop_host(struct device *dev, int rxdev);

#endif /* __MNH_MIPI */
