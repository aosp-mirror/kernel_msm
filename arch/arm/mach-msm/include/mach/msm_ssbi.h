/* arch/arm/mach-msm/include/mach/msm_ssbi.h
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Dima Zavin <dima@android.com>
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

#ifndef __ASM_ARCH_MSM_SSBI_H
#define __ASM_ARCH_MSM_SSBI_H

#include <linux/types.h>

struct msm_ssbi_slave_info {
	const char	*name;
	int		irq;
	void		*platform_data;
};

struct msm_ssbi_platform_data {
	struct msm_ssbi_slave_info	slave;
	const char			*rspinlock_name;
};

int msm_ssbi_write(struct device *dev, u16 addr, u8 *buf, int len);
int msm_ssbi_read(struct device *dev, u16 addr, u8 *buf, int len);
#endif
