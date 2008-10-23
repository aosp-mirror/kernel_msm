/* arch/arm/mach-msm/pm.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
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

#ifndef __ARCH_ARM_MACH_MSM_PM_H
#define __ARCH_ARM_MACH_MSM_PM_H

#include <asm/arch/msm_iomap.h>

#define A11S_CLK_SLEEP_EN_ADDR MSM_CSR_BASE + 0x11c

#define CLK_SLEEP_EN_ARM11_CORE	0x01
#define CLK_SLEEP_EN_ARM11_AHB	0x02
#define CLK_SLEEP_EN_ID_BRIDGE	0x04
#define CLK_SLEEP_EN_DMA_BRIDGE	0x08
#define CLK_SLEEP_EN_PBUS	0x10
#define CLK_SLEEP_EN_DEBUG_TIME	0x20
#define CLK_SLEEP_EN_GP_TIMER	0x40
#endif
