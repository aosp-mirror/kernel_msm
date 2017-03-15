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

#define MNH_MUX_DEVICE_TX_MAX	2
#define MNH_MUX_DEVICE_TX0	0
#define MNH_MUX_DEVICE_TX1	1
#define MNH_MUX_DEVICE_RX0	0
#define MNH_MUX_DEVICE_RX1	1
#define MNH_MUX_DEVICE_RX2	2

#define MNH_MIPI_VC0_EN_MASK	0x1
#define MNH_MIPI_VC1_EN_MASK	0x2
#define MNH_MIPI_VC2_EN_MASK	0x4
#define MNH_MIPI_VC3_EN_MASK	0x8
#define MNH_MIPI_VC_ALL_EN_MASK 0xf

#define MNH_SM_IOC_MAGIC 'T'
#define MNH_SM_MAX 8

#define MNH_SM_IOC_POWERON \
	_IO(MNH_SM_IOC_MAGIC, 1)
#define MNH_SM_IOC_POWEROFF \
	_IO(MNH_SM_IOC_MAGIC, 2)
#define MNH_SM_IOC_CONFIG_MIPI \
	_IOW(MNH_SM_IOC_MAGIC, 3, struct mnh_mipi_config *)
#define MNH_SM_IOC_CONFIG_DDR \
	_IO(MNH_SM_IOC_MAGIC, 4)
#define MNH_SM_IOC_GET_STATE \
	_IOR(MNH_SM_IOC_MAGIC, 5, int *)
#define MNH_SM_IOC_SET_STATE \
	_IOW(MNH_SM_IOC_MAGIC, 6, int)
#define MNH_SM_IOC_DOWNLOAD \
	_IO(MNH_SM_IOC_MAGIC, 7)
#define MNH_SM_IOC_SUSPEND \
	_IO(MNH_SM_IOC_MAGIC, 8)
#define MNH_SM_IOC_RESUME \
	_IO(MNH_SM_IOC_MAGIC, 9)

enum mnh_sm_state {
	MNH_STATE_OFF, /* powered off */
	MNH_STATE_INIT, /* powered on, unconfigured */
	MNH_STATE_CONFIG_MIPI, /* powered on, mipi configured */
	MNH_STATE_CONFIG_DDR, /* powered on, ddr configured */
	MNH_STATE_ACTIVE, /* powered on and booted */
	MNH_STATE_SUSPEND_SELF_REFRESH, /* suspended, ddr in self-refresh */
	MNH_STATE_SUSPEND_HIBERNATE, /* suspended, kernel image in AP DRAM */
	MNH_STATE_BYPASS, /* CPU and DDR powered on, DDR in self-refresh */
	MNH_STATE_MAX,
};

struct mnh_mipi_config {
	/* Tx dev MNH_MUX_DEVICE_TX* */
	int		txdev;
	/* Rx dev MNH_MUX_DEVICE_RX* */
	int		rxdev;
	/* RX MIPI transfer rate */
	int		rx_rate;
	/* TX MIPI transfer rate */
	int		tx_rate;
	/* virtual channel enable mask */
	int		vc_en_mask;
	/* MIPI gen3 ports? */
	int		is_gen3;
};

#endif /* _UAPI__MNH_SM_H */
