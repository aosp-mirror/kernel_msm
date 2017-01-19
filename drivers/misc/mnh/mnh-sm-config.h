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

#ifndef __MNH_SM_CONFIG
#define __MNH_SM_CONFIG

#include "mnh-hwio-bases.h"
#include "mnh-pwr.h"
#include "mnh-ddr.h"

#include <linux/init.h>

#define MNH_MUX_DEVICE_TX_MAX	2
#define MNH_MUX_DEVICE_TX0	0
#define MNH_MUX_DEVICE_TX1	1
#define MNH_MUX_DEVICE_RX0	0
#define MNH_MUX_DEVICE_RX1	1
#define MNH_MUX_DEVICE_RX2	2

/** Firmware download image state */
enum fw_image_state {
	FW_IMAGE_NONE = 0,
	FW_IMAGE_DOWNLOADING,
	FW_IMAGE_DOWNLOAD_SUCCESS,
	FW_IMAGE_DOWNLOAD_FAIL
};

struct mnh_sm_register_write_rep {
	u16 address;
	u8 len;
	u32 val;
	u16 dev_i2c_addr;
};

struct mnh_sm_register_read_rep {
	u16 address;
	u8 len;
	u32 mask;
	u16 dev_i2c_addr;
};

struct mnh_sm_power_seq_entity {
	char ent_name[12];
	int ent_number;
	u16 address;
	unsigned int val;
	unsigned int undo_val; /* Undo value if any previous step failed */
	unsigned int delay; /* delay in micro seconds */
};

struct mnh_mipi_conf {
	uint32_t freq;
	int      is_gen3;
};

struct mnh_tx_conf {
	/* Rx dev MNH_MUX_DEVICE_RX* */
	int		rxdev;
	/* The conf sel is the index in list of mnh_mipi_conf */
	int		conf_sel;
};

struct mnh_sm_configuration {
	const unsigned int mipi_items;
	const struct mnh_mipi_conf *mipi_configs;
	struct mnh_tx_conf *tx_configs;
	struct mnh_ddr_state *ddr_config;
	unsigned int cur_mipi_config;
};

#endif /* __MNH_SM_CONFIG */
