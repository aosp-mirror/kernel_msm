/*
 *
 * MNH State Manager Configuration.
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

#ifndef __MNH_SM_CONFIG_A
#define __MNH_SM_CONFIG_A
#include "mnh-ddr-regconfig-100.h"
#include "mnh-ddr-regconfig-400.h"
#include "mnh-ddr-regconfig-600.h"

static const struct mnh_mipi_conf mnh_mipi_configs[] = {
	{
	.freq = 640,
	.is_gen3 = 1,
	},
	{
	.freq = 1296,
	.is_gen3 = 1,
	},
	{
	.freq = 2100,
	.is_gen3 = 1,
	},
};

static struct mnh_tx_conf mnh_tx_configs[] = {
	{
	.rxdev = MNH_MUX_DEVICE_RX0,
	.conf_sel = 0,
	},
	{
	.rxdev = MNH_MUX_DEVICE_RX1,
	.conf_sel = 0,
	},

};

static struct mnh_ddr_state mnh_ddr_po_config = {
	.bases = {
		HWIO_DDR_CTL_BASE_ADDR,
		HWIO_DDR_PHY_BASE_ADDR,
		HWIO_DDR_PI_BASE_ADDR
	},
	.fsps = {
		MNH_LP4_FREQ_BOOT,
		MNH_LP4_FREQ_2400,
		MNH_LP4_FREQ_2400,
		MNH_LP4_FREQ_2400
	},
	.configs = {
		&mnh_ddr_600,
		&mnh_ddr_600,
		&mnh_ddr_600,
		&mnh_ddr_600,
	}
};

struct mnh_sm_configuration sm_config_1 = {
	.mipi_items = ARRAY_SIZE(mnh_mipi_configs),
	.mipi_configs = mnh_mipi_configs,
	.tx_configs = mnh_tx_configs,
	.ddr_config = &mnh_ddr_po_config,
};

#endif /* __MNH_SM_CONFIG_A */
