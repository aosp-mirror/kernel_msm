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

/* Keep it in sync with faceauth firmware */
#define FACEAUTH_OP_ERASE 0
#define FACEAUTH_OP_ENROLL 1
#define FACEAUTH_OP_VALIDATE 2

/* Keep it in sync with faceauth firmware */
#define FACEAUTH_RESULT_SUCCESS 0
#define FACEAUTH_RESULT_FAILURE 1

/* This struct is written by userspace and read by kernel */
struct faceauth_start_data {
	/*
	 * Operation requested by user, see FACEAUTH_OP_*
	 */
	__u32 operation;

	__u8 __user *image_dot_left;
	__u32 image_dot_left_size;

	__u8 __user *image_dot_right;
	__u32 image_dot_right_size;

	__u8 __user *image_flood;
	__u32 image_flood_size;
} __attribute__((packed));

/* This struct is written by kernel */
struct faceauth_continue_data {
	__u8 completed; /* is faceauth process completed? */
	__u8 result; /* FACEAUTH_RESULT_* */
} __attribute__((packed));

/* This struct contains a user supplied buffer that is written by kernel */
struct faceauth_debug_data {
	__u8 __user *print_buffer;
	__u32 print_buffer_size;
} __attribute__((packed));

/*
 * Prepare AP and AB for faceauth workflow. This step might include slow
 * operations like reading firmware from filesystem and copying to AB memory.
 */
#define FACEAUTH_DEV_IOC_INIT _IO('f', 1)
#define FACEAUTH_DEV_IOC_START _IOW('f', 2, struct faceauth_start_data)
#define FACEAUTH_DEV_IOC_CONTINUE _IOR('f', 3, struct faceauth_continue_data)
#define FACEAUTH_DEV_IOC_CLEANUP _IO('f', 4)
#define FACEAUTH_DEV_IOC_DEBUG _IOR('f', 5, struct faceauth_debug_data)

#endif /* _UAPI_LINUX_FACEAUTH_H */
