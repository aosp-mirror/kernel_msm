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

/* These should be hided after all dependents are moved to here. */
void ab_tmu_hw_control(struct ab_tmu_hw *hw, bool on);
void ab_tmu_hw_set_irqs(struct ab_tmu_hw *hw, bool enable);
void ab_tmu_hw_clear_irqs(struct ab_tmu_hw *hw);

u32 ab_tmu_hw_read_current_temp(struct ab_tmu_hw *hw, int id);

/* Airbrush TMU registers */
#define AB_TMU_TRIMINFO0		0x0
#define AB_TMU_TRIMINFO(n)		(AB_TMU_TRIMINFO0 + ((n) * 4))
#define AB_TMU_CONTROL			0x20
#define AB_TMU_CONTROL1			0x24
#define AB_TMU_STATUS			0x28
#define AB_TMU_SAMPLING_INTERVAL	0x2c
#define AB_TMU_COUNTER_VALUE0		0x30
#define AB_TMU_COUNTER_VALUE1		0x34
#define AB_TMU_AVG_CONTROL		0x34
#define AB_TMU_CURRENT_TEMP0_1		0x040
#define AB_TMU_CURRENT_TEMP2_4		0x044
#define AB_TMU_CURRENT_TEMP5_7		0x048
#define AB_TMU_THD_TEMP_RISE7_6_P(n) \
	((n == 0) ? (0x050) : (0x170+(n-1)*0x20))
#define AB_TMU_THD_TEMP_RISE5_4		0x054
#define AB_TMU_THD_TEMP_RISE3_2		0x058
#define AB_TMU_THD_TEMP_RISE1_0		0x05C
#define AB_TMU_THD_TEMP_FALL7_6_P(n) \
	((n == 0) ? (0x060) : (0x180+(n-1)*0x20))

#define AB_TMU_INTEN0			0x110
#define AB_TMU_INTSTAT0			0x114
#define AB_TMU_INTPEND0			0x118
#define AB_TMU_INTEN5			0x310
#define AB_TMU_INTSTAT5			0x314
#define AB_TMU_INTPEND5			0x318

#define AB_TMU_INTX_STEP			0x10
#define AB_TMU_INTX(x, n) \
({ \
	const unsigned int _n = (n); \
	(_n <= 4) ? \
		AB_TMU_INT##x##0 + _n * AB_TMU_INTX_STEP : \
		AB_TMU_INT##x##5 + ((_n - 5) * AB_TMU_INTX_STEP); \
})
#define AB_TMU_INTEN(n)			AB_TMU_INTX(EN, n)
#define AB_TMU_INTSTAT(n)		AB_TMU_INTX(STAT, n)
#define AB_TMU_INTPEND(n)		AB_TMU_INTX(PEND, n)

#define AB_TMU_INTX_RISE_SHIFT(n)	(n)
#define AB_TMU_INTX_FALL_SHIFT(n)	((n) + 16)
#define AB_TMU_INTEN_ALL		0xff01ff

#define AB_TMU_CAL_MASK			0x3
#define AB_TMU_TEMP_MASK		0x1ff
#define AB_TMU_BUF_SLOPE_SEL_MASK	0xf
#define AB_TMU_BUF_SLOPE_SEL_SHIFT	8
#define AB_TMU_CORE_EN_SHIFT		0
#define AB_TMU_EN_TRIP_SHIFT		12
#define AB_TMU_TRIP_MODE_SHIFT		13

#define AB_TMU_EMUL_CON			0x160
#define AB_TMU_DEBUG_CURRENT_TEMP	0x164
#define AB_TMU_EMUL_DATA_SHIFT		7
#define AB_TMU_EMUL_DATA_MASK		0x1ff
#define AB_TMU_EMUL_NEXTTIME_SHIFT	16
#define AB_TMU_EMUL_NEXTTIME_VAL	0x1
#define AB_TMU_EMUL_ENABLE_SHIFT	0
#define AB_TMU_EMUL_ENABLE		0x1

#define AB_TMU_TRIMINFO_25_SHIFT	0
#define AB_TMU_TRIMINFO_85_SHIFT	8
#define AB_TMU_TRIP_MODE_SHIFT		13
#define AB_TMU_TRIP_MODE_MASK		0x7
#define AB_TMU_THERM_TRIP_EN_SHIFT	12

#define AB_TMU_NO_TRIMMING		0
#define AB_TMU_ONE_POINT_TRIMMING	1
#define AB_TMU_TWO_POINT_TRIMMING	2

#define AB_TMU_TEMP_PROBE_MAIN		0
#define AB_TMU_TEMP_PROBE_IPU0		1
#define AB_TMU_TEMP_PROBE_IPU1		2
#define AB_TMU_TEMP_PROBE_IPU2		3
#define AB_TMU_TEMP_PROBE_IPU_TPU	4
#define AB_TMU_TEMP_PROBE_TPU0		5
#define AB_TMU_TEMP_PROBE_TPU1		6
#define AB_TMU_NUM_REMOTE_PROBE		0x6
#define AB_TMU_REMOTE_PROBE_SHIFT	16
#define AB_TMU_NUM_ALL_PROBE		7
#define AB_TMU_CAL_SHIFT		18
#define AB_TMU_TEMP_SHIFT		9

#endif /* _AIRBRUSH_TMU_HW_ */
