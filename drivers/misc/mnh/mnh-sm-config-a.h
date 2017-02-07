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

#include "mnh-ddr-33-300-600-400.h"

static struct mnh_mipi_config mnh_mipi_configs[] = {
	{
		.rxdev = MNH_MUX_DEVICE_RX0,
		.rx_rate = 1296,
		.tx_rate = 1296,
		.vc_en_mask = MNH_MIPI_VC_ALL_EN_MASK,
		.is_gen3 = 1,
	},
	{
		.rxdev = MNH_MUX_DEVICE_RX1,
		.rx_rate = 648,
		.tx_rate = 1296,
		.vc_en_mask = MNH_MIPI_VC_ALL_EN_MASK,
		.is_gen3 = 1,
	},
};

static struct mnh_ddr_state mnh_ddr_po_config = {
	.bases = {
		HWIO_DDR_CTL_BASE_ADDR,
		HWIO_DDR_PHY_BASE_ADDR,
		HWIO_DDR_PI_BASE_ADDR
	},
	.fsps = {
		0x100B007D,
		0x0433007D,
		0x0311007D,
		0x0422007D
	},
	&mnh_ddr_33_300_600_400,
};

struct mnh_sm_configuration sm_config_default = {
	.mipi_configs = mnh_mipi_configs,
	.ddr_config = &mnh_ddr_po_config,
};

#endif /* __MNH_SM_CONFIG_A */
