/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _AIRBRUSH_TMU_HW_
#define _AIRBRUSH_TMU_HW_

#include <linux/types.h>

struct ab_tmu_hw;
struct device;

struct ab_tmu_hw *devm_ab_tmu_hw_create(struct device *dev, u32 base);
void devm_ab_tmu_hw_destroy(struct ab_tmu_hw *hw);

bool ab_tmu_hw_pcie_link_lock(struct ab_tmu_hw *hw);
void ab_tmu_hw_pcie_link_unlock(struct ab_tmu_hw *hw);

u32 ab_tmu_hw_read(struct ab_tmu_hw *hw, u32 offset);
void ab_tmu_hw_write(struct ab_tmu_hw *hw, u32 offset, u32 value);

u32 ab_tmu_hw_read_current_temp(struct ab_tmu_hw *hw, int id);

/* Exynos generic registers */
#define AIRBRUSH_TMU_REG_TRIMINFO	0x0
#define AIRBRUSH_TMU_REG_TRIMINFO_P(n)	((0x0) + ((n)*4))
#define AIRBRUSH_TMU_REG_CONTROL	0x20
#define AIRBRUSH_TMU_REG_CONTROL1	0x24
#define AIRBRUSH_TMU_REG_STATUS		0x28
#define AIRBRUSH_TMU_SAMPLING_INTERVAL	0x02c
#define AIRBRUSH_TMU_COUNTER_VALUE0	0x030
#define AIRBRUSH_TMU_COUNTER_VALUE1	0x034
#define AIRBRUSH_TMU_AVG_CONTROL	0x034
#define AIRBRUSH_TMU_REG_CURRENT_TEMP0_1	0x040
#define AIRBRUSH_TMU_REG_CURRENT_TEMP2_4	0x044
#define AIRBRUSH_TMU_REG_CURRENT_TEMP5_7	0x048
#define AIRBRUSH_THD_TEMP_RISE7_6_P(n) \
		((n == 0) ? (0x050) : (0x170+(n-1)*0x20))
#define AIRBRUSH_THD_TEMP_RISE5_4		0x054
#define AIRBRUSH_THD_TEMP_RISE3_2		0x058
#define AIRBRUSH_THD_TEMP_RISE1_0		0x05C
#define AIRBRUSH_THD_TEMP_FALL7_6_P(n) \
		((n == 0) ? (0x060) : (0x180+(n-1)*0x20))
#define AIRBRUSH_TMU_REG_INTEN_P(n) \
		((n <= 4) ? (0x110+n*0x10) : (0x310+((n-5)*0x10)))
#define AIRBRUSH_TMU_REG_INTSTAT_P(n) \
		((n <= 4) ? (0x114+n*0x10) : (0x314+((n-5)*0x10)))
#define AIRBRUSH_TMU_REG_INTPEND_P(n) \
		((n <= 4) ? (0x118+n*0x10) : (0x318+((n-5)*0x10)))

#define AIRBRUSH_TMU_INT_EN			0xff01ff

#define AIRBRUSH_TMU_INTEN_FALL0_SHIFT		16

#define AIRBRUSH_TMU_CAL_MASK		0x3
#define AIRBRUSH_TMU_TEMP_MASK		0x1ff
#define AIRBRUSH_TMU_BUF_SLOPE_SEL_MASK	0xf
#define AIRBRUSH_TMU_BUF_SLOPE_SEL_SHIFT	8
#define AIRBRUSH_TMU_CORE_EN_SHIFT	0
#define AIRBRUSH_TMU_EN_TRIP_SHIFT	12
#define AIRBRUSH_TMU_TRIP_MODE_SHIFT	13

#define AIRBRUSH_EMUL_CON		0x160
#define AIRBRUSH_DEBUG_CURRENT_TEMP	0x164
#define AIRBRUSH_EMUL_DATA_SHIFT	7
#define AIRBRUSH_EMUL_DATA_MASK		0x1ff
#define AIRBRUSH_EMUL_NEXTTIME_SHIFT	16
#define AIRBRUSH_EMUL_NEXTTIME_VAL	0x1
#define AIRBRUSH_EMUL_ENABLE_SHIFT	0
#define AIRBRUSH_EMUL_ENABLE	0x1

#define AIRBRUSH_TRIMINFO_25_SHIFT	0
#define AIRBRUSH_TRIMINFO_85_SHIFT	8
#define AIRBRUSH_TMU_TRIP_MODE_SHIFT	13
#define AIRBRUSH_TMU_TRIP_MODE_MASK	0x7
#define AIRBRUSH_TMU_THERM_TRIP_EN_SHIFT	12

#define AIRBRUSH_TMU_INTEN_RISE0_SHIFT		0
#define AIRBRUSH_TMU_INTEN_RISE1_SHIFT		1
#define AIRBRUSH_TMU_INTEN_RISE2_SHIFT		2
#define AIRBRUSH_TMU_INTEN_RISE3_SHIFT		3
#define AIRBRUSH_TMU_INTEN_RISE4_SHIFT		4
#define AIRBRUSH_TMU_INTEN_RISE5_SHIFT		5
#define AIRBRUSH_TMU_INTEN_RISE6_SHIFT		6
#define AIRBRUSH_TMU_INTEN_RISE7_SHIFT		7

#define AIRBRUSH_NO_TRIMMING	0
#define AIRBRUSH_ONE_POINT_TRIMMING	1
#define AIRBRUSH_TWO_POINT_TRIMMING	2

#define AIRBRUSH_TEMP_PROBE_MAIN		0
#define AIRBRUSH_TEMP_PROBE_IPU0		1
#define AIRBRUSH_TEMP_PROBE_IPU1		2
#define AIRBRUSH_TEMP_PROBE_IPU2		3
#define AIRBRUSH_TEMP_PROBE_IPU_TPU		4
#define AIRBRUSH_TEMP_PROBE_TPU0		5
#define AIRBRUSH_TEMP_PROBE_TPU1		6
#define AIRBRUSH_NUM_REMOTE_PROBE		0x6
#define AIRBRUSH_REMOTE_PROBE_SHIFT		16
#define AIRBRUSH_NUM_ALL_PROBE			7
#define AIRBRUSH_TMU_CAL_SHIFT			18
#define AIRBRUSH_TMU_TEMP_SHIFT			9

#endif /* _AIRBRUSH_TMU_HW_ */
