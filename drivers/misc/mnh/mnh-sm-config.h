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


struct mnh_sm_configuration {

	const unsigned int power_items;
	const struct mnh_sm_power_seq_entity *power_entities;
	const unsigned int power_delay; /* in micro seconds */

	const unsigned int powerup_regs_items;
	const struct mnh_sm_register_write_rep *powerup_regs;

	const unsigned int poweroff_regs_items;
	const struct mnh_sm_register_write_rep *poweroff_regs;

	const unsigned int ddr_suspend_regs_items;
	const struct mnh_sm_register_write_rep *ddr_suspend_regs;

	const unsigned int ddr_resume_regs_items;
	const struct mnh_sm_register_write_rep *ddr_resume_regs;

	const unsigned int mipi_powerup_regs_items;
	const struct mnh_sm_register_write_rep *mipi_powerup_regs;
};

#endif /* __MNH_SM_CONFIG */
