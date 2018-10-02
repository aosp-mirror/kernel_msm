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

/* This struct is written by userspace and read by kernel */
struct faceauth_start_data {
	__u8 __user *image_dot_left;
	__u32 image_dot_left_size;

	__u8 __user *image_dot_right;
	__u32 image_dot_right_size;

	__u8 __user *image_flood;
	__u32 image_flood_size;

	/*
	 * This is data to feed individual TPU stages, for debug and demo
	 * TODO: To be removed
	 */
	__u8 __user *facenet_input;
	__u32 facenet_input_size;

	__u8 __user *gazenet_input;
	__u32 gazenet_input_size;

	__u8 __user *depthid_input;
	__u32 depthid_input_size;

	__u8 __user *depth_output;
	__u32 depth_output_size;

	__u8 __user *affine_output;
	__u32 affine_output_size;

	__u8 __user *fssd_output;
	__u32 fssd_output_size;

	__u8 __user *facenet_output;
	__u32 facenet_output_size;

	__u8 __user *gazenet_output;
	__u32 gazenet_output_size;

	__u8 __user *depthid_output;
	__u32 depthid_output_size;

};

/* This struct is written by kernel */
struct faceauth_continue_data {
	__u8 completed; /* is faceauth process completed? */
	__u8 success; /* is faceauth process successful? */

};

#define FACEAUTH_DEV_IOC_INIT _IO('f', 1)
#define FACEAUTH_DEV_IOC_START _IOW('f', 2, struct faceauth_start_data)
#define FACEAUTH_DEV_IOC_CONTINUE _IOR('f', 3, struct faceauth_continue_data)
#define FACEAUTH_DEV_IOC_CLEANUP _IO('f', 4)

#endif /* _UAPI_LINUX_FACEAUTH_H */
