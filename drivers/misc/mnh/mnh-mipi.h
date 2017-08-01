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

#include "mnh-sm.h"

/* Interrupt status registers, R/C=read-clear, W/C=write-clear */
struct mipi_device_irq_st {
	int dev;
	/* status of individual interrupt sources (R/C) */
	uint32_t main;
	/* interrupt group caused by video pattern gen. (R/C) */
	uint32_t vpg;
	/* interrupt group caused by IDI interface (R/C) */
	uint32_t idi;
	/* interrupt group caused by the PHY (R/C) */
	uint32_t phy;
	/* buffer overflow indicator in the bypass FIFOs in MIPI_TOP (W/C=1) */
	uint32_t fifo_overflow;
};

struct mipi_host_irq_st {
	int dev;
	/* status of individual interrupt sources (R/C) */
	uint32_t main;
	/* fatal interruption caused by PHY (R/C) */
	uint32_t phy_fatal;
	/* fatal interruption related to packet construction (R/C) */
	uint32_t pkt_fatal;
	/* fatal interruption related to frame construction (R/C) */
	uint32_t frame_fatal;
	/* interruption caused by PHY (R/C) */
	uint32_t phy;
	/* interruption related to packet construction (R/C) */
	uint32_t pkt;
	/* interruption related to line construction (R/C) */
	uint32_t line;
};

int mnh_mipi_config(struct device *dev, struct mnh_mipi_config cfg);
int mnh_mipi_stop(struct device *dev, struct mnh_mipi_config cfg);
void mnh_mipi_stop_device(struct device *dev, int txdev);
void mnh_mipi_stop_host(struct device *dev, int rxdev);
void mnh_mipi_set_debug(int val);
int mnh_mipi_get_device_interrupts(struct device *dev,
				   struct mipi_device_irq_st *int_status);
int mnh_mipi_get_host_interrupts(struct device *dev,
				 struct mipi_host_irq_st *int_status);

#endif /* __MNH_MIPI */
