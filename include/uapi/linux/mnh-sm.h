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

#ifndef _UAPI__MNH_SM_H
#define _UAPI__MNH_SM_H

#define MIPI_TX0    0
#define MIPI_TX1    1
#define MIPI_TX_IPU 2
#define MIPI_RX0    0
#define MIPI_RX1    1
#define MIPI_RX2    2
#define MIPI_RX_IPU 3

#define MIPI_MODE_BYPASS       0
#define MIPI_MODE_BYPASS_W_IPU 1
#define MIPI_MODE_FUNCTIONAL   2

#define MNH_MIPI_VC0_EN_MASK	0x1
#define MNH_MIPI_VC1_EN_MASK	0x2
#define MNH_MIPI_VC2_EN_MASK	0x4
#define MNH_MIPI_VC3_EN_MASK	0x8
#define MNH_MIPI_VC_ALL_EN_MASK 0xf

#define MNH_SM_IOC_MAGIC 'T'
#define MNH_SM_MAX 8

#define MNH_ION_BUFFER_SIZE SZ_64M

#define MNH_SM_IOC_GET_STATE \
	_IOR(MNH_SM_IOC_MAGIC, 1, int *)
#define MNH_SM_IOC_SET_STATE \
	_IOW(MNH_SM_IOC_MAGIC, 2, int)
#define MNH_SM_IOC_WAIT_FOR_STATE \
	_IOW(MNH_SM_IOC_MAGIC, 3, int)
#define MNH_SM_IOC_CONFIG_MIPI \
	_IOW(MNH_SM_IOC_MAGIC, 4, struct mnh_mipi_config *)
#define MNH_SM_IOC_STOP_MIPI \
	_IOW(MNH_SM_IOC_MAGIC, 5, struct mnh_mipi_config *)
#define MNH_SM_IOC_WAIT_FOR_POWER \
	_IO(MNH_SM_IOC_MAGIC, 6)
#define MNH_SM_IOC_GET_UPDATE_BUF \
	_IOR(MNH_SM_IOC_MAGIC, 7, int *)
#define MNH_SM_IOC_POST_UPDATE_BUF \
	_IOW(MNH_SM_IOC_MAGIC, 8, struct mnh_update_configs *)

enum mnh_sm_state {
	MNH_STATE_OFF, /* powered off */
	MNH_STATE_ACTIVE, /* powered on and booted */
	MNH_STATE_SUSPEND, /* suspended, ddr in self-refresh */
	MNH_STATE_MAX,
};

enum mnh_fw_slot {
	MNH_FW_SLOT_SBL = 0,
	MNH_FW_SLOT_KERNEL,
	MNH_FW_SLOT_DTB,
	MNH_FW_SLOT_RAMDISK,
	MAX_NR_MNH_FW_SLOTS
};

struct mnh_mipi_config {
	/* Tx dev (MIPI sink) MNH_MUX_DEVICE_TX* */
	int txdev;
	/* Rx dev (MIPI source) MNH_MUX_DEVICE_RX* */
	int rxdev;
	/* RX MIPI transfer rate */
	int rx_rate;
	/* TX MIPI transfer rate */
	int tx_rate;
	/* Mux mode */
	int mode;
	/* virtual channel enable mask */
	int vc_en_mask;
};

struct mnh_update_config {
	/* slot type (dtb, kernel, ramdisk) */
	enum mnh_fw_slot slot_type;
	/* slot offset in the ion buffer */
	unsigned long offset;
	/* slot size */
	size_t size;
};

struct mnh_update_configs {
	struct mnh_update_config config[MAX_NR_MNH_FW_SLOTS];
};

#endif /* _UAPI__MNH_SM_H */
