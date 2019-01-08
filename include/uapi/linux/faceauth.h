/*
 * FaceAuth coordinator driver
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

#ifndef _UAPI_LINUX_FACEAUTH_H
#define _UAPI_LINUX_FACEAUTH_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/time.h>

/* Keep it in sync with faceauth firmware */
#define FACEAUTH_OP_ERASE 0
#define FACEAUTH_OP_ENROLL 1
#define FACEAUTH_OP_VALIDATE 2
#define FACEAUTH_OP_ENROLL_COMPLETE 3

/* Keep it in sync with faceauth firmware */
#define FACEAUTH_RESULT_SUCCESS 0
#define FACEAUTH_RESULT_FAILURE 1

#define FACEAUTH_ERROR_NO_ERROR 0

#define FACEAUTH_MAX_TASKS 32
#define FACEAUTH_DEBUG_REGISTER_COUNT (24)

/* This struct is written by userspace and read by kernel */
struct faceauth_start_data {
	/*
	 * Operation requested by user, see FACEAUTH_OP_*
	 */
	__u8 profile_id;
	__u8 operation;

	__u8 __user *image_dot_left;
	__u32 image_dot_left_size;

	__u8 __user *image_dot_right;
	__u32 image_dot_right_size;

	__u8 __user *image_flood;
	__u32 image_flood_size;

	void __user *calibration;
	__u32 calibration_size;

	/* Output parameters */
	__u8 result; /* result code from AB */
	__u32 bin_bitmap;
	__s32 error_code; /* ab-faceauth error code */
	__u32 fw_version; /* ab-faceauth firmware version */
} __attribute__((packed));

#define GET_DEBUG_DATA_FROM_CACHE (0)
#define GET_DEBUG_DATA_FROM_AB_DRAM (1)

/* This struct contains a user supplied buffer that is written by kernel */
struct faceauth_debug_data {
	union {
		__u8 __user *debug_buffer;
		__u8 __user *print_buffer;
	};
	union {
		__u32 debug_buffer_size;
		__u32 print_buffer_size;
	};
	__u32 flags;
} __attribute__((packed));

struct faceauth_workload_control {
	__u32 workload_state;
	__u32 run_count;
	__u32 run_time_ms;
	__u32 run_time_us;
	__u32 status;
} __attribute__((packed));

struct faceauth_debug_register {
	__u64 address;
	__u64 value;
} __attribute__((packed));

struct faceauth_airbrush_state {
	__u32 faceauth_version;
	__s32 error_code;
	__u32 internal_state_size;
	__u32 command;
	__u32 num_tasks;
	__u32 register_list_length;
	__u32 saved_register_count;
	struct faceauth_workload_control control_list[FACEAUTH_MAX_TASKS];
	struct faceauth_debug_register
		debug_registers[FACEAUTH_DEBUG_REGISTER_COUNT];
} __attribute__((packed));

struct faceauth_debug_image {
	__u32 offset_to_image;
	__u32 image_size;
};

struct faceauth_debug_entry {
	struct timeval timestamp;
	struct faceauth_debug_image left_dot;
	struct faceauth_debug_image right_dot;
	struct faceauth_debug_image flood;
	struct faceauth_airbrush_state ab_state;
} __attribute__((packed));

/*
 * Prepare AP and AB for faceauth workflow. This step might include slow
 * operations like reading firmware from filesystem and copying to AB memory.
 */
#define FACEAUTH_DEV_IOC_INIT _IO('f', 1)
#define FACEAUTH_DEV_IOC_START _IOWR('f', 2, struct faceauth_start_data)
#define FACEAUTH_DEV_IOC_CLEANUP _IO('f', 4)
#define FACEAUTH_DEV_IOC_DEBUG _IOR('f', 5, struct faceauth_debug_data)
#define FACEAUTH_DEV_IOC_DEBUG_DATA _IOR('f', 6, struct faceauth_debug_data)

//TODO: might be useful to be able to pull debug data either from the circular
//log, or from ab_dram directly

#endif /* _UAPI_LINUX_FACEAUTH_H */
