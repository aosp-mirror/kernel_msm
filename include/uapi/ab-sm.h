/*
 * Driver interface for the AB State Manager
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __UAPI_AB_SM_H__
#define __UAPI_AB_SM_H__

#include <linux/ioctl.h>


#define AB_SM_IOCTL_MAGIC	'a'

/* First call after open will immediately return the current
 * state. Subsequent calls will block until next state change,
 * then return the new state.
 *
 * Parameter int *:
 *	Pass pointer to integer to be filled in with new state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_ASYNC_NOTIFY	_IOR(AB_SM_IOCTL_MAGIC, 0, int *)

/*
 * Parameter int:
 *	Pass new state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_STATE		_IOW(AB_SM_IOCTL_MAGIC, 1, int)

/*
 * Parameter int *:
 *	Pass pointer to integer to be filled in with state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_GET_STATE		_IOR(AB_SM_IOCTL_MAGIC, 2, int *)

/* On success will return 0, otherwise will return error < 0. */
#define AB_SM_ENTER_EL2		_IO(AB_SM_IOCTL_MAGIC, 3)

/* On success will return 0, otherwise will return error < 0. */
#define AB_SM_EXIT_EL2		_IO(AB_SM_IOCTL_MAGIC, 4)

/*
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_IPU_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 5, int)

/*
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_TPU_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 6, int)

/*
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_DDR_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 7, int)

/*
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_PCIE_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 8, int)

/*
 * Parameter int:
 *	Pass new frequency value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_AON_FREQUENCY	_IOW(AB_SM_IOCTL_MAGIC, 9, int)

/*
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_IPU_STATE	_IOW(AB_SM_IOCTL_MAGIC, 10, int)

/*
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_TPU_STATE	_IOW(AB_SM_IOCTL_MAGIC, 11, int)

/*
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_DDR_STATE	_IOW(AB_SM_IOCTL_MAGIC, 12, int)

/*
 * Parameter int:
 *	Pass new low power state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_PCIE_STATE	_IOW(AB_SM_IOCTL_MAGIC, 13, int)

/*
 * Parameter struct new_block_props *:
 *	Pass new ipu state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_IPU_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 14, \
		struct new_block_props *)

/*
 * Parameter struct new_block_props *:
 *	Pass new tpu state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_TPU_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 15, \
		struct new_block_props *)

/*
 * Parameter struct new_block_props *:
 *	Pass new dram state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_DRAM_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 16, \
		struct new_block_props *)

/*
 * Parameter struct new_block_props *:
 *	Pass new mif state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_MIF_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 17, \
		struct new_block_props *)

/*
 * Parameter struct new_block_props *:
 *	Pass new fsys state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_FSYS_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 18, \
		struct new_block_props *)

/*
 * Parameter struct new_block_props *:
 *	Pass new aon state properties
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_UPDATE_AON_STATE_PROPERTIES	_IOW(AB_SM_IOCTL_MAGIC, 19, \
		struct new_block_props *)

/*
 * Parameter int *:
 *	Pass pointer to integer to be filled in with state value
 *	On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_GET_EL2_MODE	_IOR(AB_SM_IOCTL_MAGIC, 20, int *)

/* First call after open will immediately return the current
 * state. Subsequent calls will block until next state change,
 * then return the new state.
 *
 * Parameter int *:
 *	Pass pointer to integer to be filled in with mapped state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_MAPPED_ASYNC_NOTIFY	_IOR(AB_SM_IOCTL_MAGIC, 21, int *)

/*
 * Parameter int:
 *	Pass mapped state value to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_MAPPED_SET_STATE		_IOW(AB_SM_IOCTL_MAGIC, 22, int)

/*
 * Parameter int *:
 *	Pass pointer to integer to be filled in with mapped state value
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_MAPPED_GET_STATE		_IOR(AB_SM_IOCTL_MAGIC, 23, int *)

/*
 * Parameter int:
 *	Pass new L1ss entry delay to set
 * On success will return 0, otherwise will return error < 0.
 */
#define AB_SM_SET_PCIE_L1SS_DELAY  _IOW(AB_SM_IOCTL_MAGIC, 24, int)

#define AB_CHIP_ID_UNKNOWN	-1
#define AB_CHIP_ID_A0		0
#define AB_CHIP_ID_B0		1

#define UAPI_BLK_(num, pmu, rail, v, clk, freq, pwr, used, tiles, dr) \
	{					\
		UAPI_BLOCK_STATE_ ## num,	\
		(enum uapi_pmu_state)pmu,	\
		uapi_ ## rail,			\
		UAPI_VOLTAGE_ ## v,		\
		uapi_ ## clk,			\
		(__u64)(1000000. * freq),	\
		pwr,				\
		used,				\
		tiles,				\
		dr,				\
	}

/* Keep in sync with enum block_state in airbrush-sm-ctrl.h. */
#define AB_SM_UAPI_NUM_BLOCK_STATES	12
enum uapi_block_state {
	UAPI_BLOCK_STATE_0 = 0,
	UAPI_BLOCK_STATE_100 = 100,
	UAPI_BLOCK_STATE_101,
	UAPI_BLOCK_STATE_200 = 200,
	UAPI_BLOCK_STATE_201,
	UAPI_BLOCK_STATE_202,
	UAPI_BLOCK_STATE_300 = 300,
	UAPI_BLOCK_STATE_301,
	UAPI_BLOCK_STATE_302,
	UAPI_BLOCK_STATE_303,
	UAPI_BLOCK_STATE_304,
	UAPI_BLOCK_STATE_305,
};

/* Keep in sync with enum pmu_states in airbrush_sm_ctrl.h. */
enum uapi_pmu_state {
	UAPI_PMU_STATE_ON = 0,
	UAPI_PMU_STATE_SLEEP,
	UAPI_PMU_STATE_DEEP_SLEEP,
	UAPI_PMU_STATE_OFF
};

/* Keep in sync with enum states in airbrush-sm-ctrl.h. */
enum uapi_state {
	uapi_off = 0,
	uapi_on = 1,
};

/* Keep in sync with enum logic_voltage in airbrush-sm-ctrl.h. */
enum uapi_logic_voltage {
	UAPI_VOLTAGE_0_0,
	UAPI_VOLTAGE_0_60,
	UAPI_VOLTAGE_0_75,
	UAPI_VOLTAGE_0_85,
};

/**
 * Stores information of the soc block's operating state.
 * Similar to struct block_property in airbrush-sm-ctrl.h.
 */
struct uapi_block_properties {
	enum uapi_block_state id;
	enum uapi_pmu_state pmu;
	enum uapi_state rail_en;
	enum uapi_logic_voltage logic_voltage;
	enum uapi_state clk_status;
	__u64 clk_frequency;
	__u32 num_powered_cores;
	__u32 num_computing_cores;
	__u32 num_powered_tiles;
	__u32 data_rate;
};

struct new_block_props {
	struct uapi_block_properties table[AB_SM_UAPI_NUM_BLOCK_STATES];
};


#endif /* __UAPI_AB_SM_H__ */
