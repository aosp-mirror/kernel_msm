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

#include "mnh-ddr-600.h"
#include "mnh-ddr-33-300-600-400.h"

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
	.conf_sel = 1,
	},
	{
	.rxdev = MNH_MUX_DEVICE_RX1,
	.conf_sel = 1,
	},

};

static struct mnh_ddr_state mnh_ddr_po_config = {
	.bases = {
		HWIO_DDR_CTL_BASE_ADDR,
		HWIO_DDR_PHY_BASE_ADDR,
		HWIO_DDR_PI_BASE_ADDR
	},
#ifndef MNH_DDR_DO_FSP
	.fsps = {
		0x100B007D,
		0x0311007D,
		0x0311007D,
		0x0311007D
	},
	&mnh_ddr_600,
#else
	.fsps = {
		0x100B007D,
		0x0311007D,
		0x0311007D,
		0x0311007D
	},
	&mnh_ddr_33_300_600_400,
#endif
};

struct mnh_sm_configuration sm_config_1 = {
	.mipi_items = ARRAY_SIZE(mnh_mipi_configs),
	.mipi_configs = mnh_mipi_configs,
	.tx_configs = mnh_tx_configs,
	.ddr_config = &mnh_ddr_po_config,
};

#endif /* __MNH_SM_CONFIG_A */
