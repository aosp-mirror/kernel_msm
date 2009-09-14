/* include/linux/msm_hw3d.h
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#ifndef _MSM_HW3D_H_
#define _MSM_HW3D_H_

#include <linux/fs.h>
#include <linux/ioctl.h>

struct hw3d_region;

#define HW3D_IOCTL_MAGIC		'h'
#define HW3D_WAIT_FOR_REVOKE		_IO(HW3D_IOCTL_MAGIC, 0x80)
#define HW3D_WAIT_FOR_INTERRUPT		_IO(HW3D_IOCTL_MAGIC, 0x81)
#define HW3D_GET_REGIONS		\
			_IOR(HW3D_IOCTL_MAGIC, 0x82, struct hw3d_region *)

#define HW3D_REGION_OFFSET(id)		((((uint32_t)(id)) & 0xf) << 28)
#define HW3D_REGION_ID(addr)		(((uint32_t)(addr) >> 28) & 0xf)
#define HW3D_OFFSET_IN_REGION(addr)	((uint32_t)(addr) & ~(0xfUL << 28))

enum {
	HW3D_EBI		= 0,
	HW3D_SMI		= 1,
	HW3D_REGS		= 2,

	HW3D_NUM_REGIONS	= HW3D_REGS + 1,
};

struct hw3d_region {
	unsigned long		phys;
	unsigned long		map_offset;
	unsigned long		len;
};

#ifdef CONFIG_MSM_HW3D
int get_msm_hw3d_file(int fd, uint32_t *offs, unsigned long *pbase,
		      unsigned long *len, struct file **filp);
void put_msm_hw3d_file(struct file *file);
bool is_msm_hw3d_file(struct file *file);
#else
int get_msm_hw3d_file(int fd, uint32_t *offs, unsigned long *pbase,
		      unsigned long *len, struct file **filp) { return -1; }
void put_msm_hw3d_file(struct file *file) {}
bool is_msm_hw3d_file(struct file *file) { return false; }
#endif

#endif /* _MSM_HW3D_H_ */
