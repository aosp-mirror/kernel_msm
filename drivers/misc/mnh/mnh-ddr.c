/*
*
* MNH DDR Driver
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

#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-ddr-ctl.h"
#include "mnh-hwio-ddr-pi.h"
#include "mnh-hwio-ddr-phy.h"
#include "mnh-hwio-scu.h"
#include "mnh-ddr.h"
#include "mnh-pcie.h"
#include "mnh-pll.h"

#define MNH_DDR_CTL_IN(reg) \
	HW_IN(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg)
#define MNH_DDR_CTL_INf(reg, fld) \
	HW_INf(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg, fld)
#define MNH_DDR_CTL_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_DDR_CTL_BASE_ADDR, DDR_CTL, reg, fld, val)

#define MNH_DDR_PI_INf(reg, fld) \
	HW_INf(HWIO_DDR_PI_BASE_ADDR, DDR_PI, reg, fld)
#define MNH_DDR_PI_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_DDR_PI_BASE_ADDR, DDR_PI, reg, fld, val)
#define MNH_DDR_PI_OUT(reg, val) \
	HW_OUT(HWIO_DDR_PI_BASE_ADDR, DDR_PI, reg, val)

#define MNH_DDR_PHY_INf(reg, fld) \
	HW_INf(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, reg, fld)
#define MNH_DDR_PHY_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, reg, fld, val)
#define MNH_DDR_PHY_OUT(reg, val) \
	HW_OUT(HWIO_DDR_PHY_BASE_ADDR, DDR_PHY, reg, val)

#define MNH_SCU_INf(reg, fld) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, reg, fld)
#define MNH_SCU_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, reg, fld, val)
#define MNH_SCU_OUT(reg, val) \
	HW_OUT(HWIO_SCU_BASE_ADDR, SCU, reg, val)
#define MNH_SCU_OUTx(reg, inst, val) \
	HW_OUTx(HWIO_SCU_BASE_ADDR, SCU, reg, inst, val)

#define MNH_RSTC_INf(fld) \
	HW_INf(HWIO_SCU_BASE_ADDR, SCU, RSTC, fld)
#define MNH_RSTC_OUTf(fld, val) \
	HW_OUTf(HWIO_SCU_BASE_ADDR, SCU, RSTC, fld, val)

const u32 ELEM_VAL   = MNH_DDR_REG_CONFIG_ELEM_VAL;
const u32 ELEM_FLAGS = MNH_DDR_REG_CONFIG_ELEM_FLAGS;

int mnh_ddr_po_init(struct device *dev,
		    struct mnh_ddr_state *state)
{
	int index;

#define WRITE_DDR_REG_CONFIG(fsp, ddrblock, regindex) do { \
		if ((state->configs[fsp]->ddrblock[regindex][ELEM_FLAGS] & \
			MNH_DDR_REG_CONFIG_FLAG_SKIP_PUSH) == 0) { \
			mnh_reg_write(state->bases.ddrblock##_base + \
				      (regindex * sizeof(u32)), \
				       state->configs[fsp]->ddrblock[regindex][ELEM_VAL]); \
		} \
	} while (0)

	/* MNH_PLL_PASSCODE_SET */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x4CD9);

	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, PD, 0);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 0);
	MNH_SCU_OUTf(LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);

	MNH_SCU_OUTf(LPDDR4_LOW_POWER_CFG, LP4_FSP_SW_OVERRIDE, 0);

	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 0, 0x100B007D);

	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 1, 0x0411007D);
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 2, 0x0411007D);
	MNH_SCU_OUTx(LPDDR4_FSP_SETTING, 3, 0x0411007D);

	/* MNH_PLL_PASSCODE_CLR */
	MNH_SCU_OUTf(PLL_PASSCODE, PASSCODE, 0x0);

	for (index = 0; index < MNH_DDR_NUM_CTL_REG; index++)
		WRITE_DDR_REG_CONFIG(2, ctl, index);

	for (index = 0; index < MNH_DDR_NUM_PI_REG; index++)
		WRITE_DDR_REG_CONFIG(2, pi, index);

	MNH_DDR_PHY_OUTf(1024, PHY_FREQ_SEL, 2);
	MNH_DDR_PHY_OUTf(1025, PHY_FREQ_SEL_MULTICAST_EN, 1);

	for (index = 0; index < MNH_DDR_NUM_PHY_REG; index++)
		WRITE_DDR_REG_CONFIG(2, phy, index);

	MNH_DDR_CTL_OUTf(493, DISABLE_MEMORY_MASKED_WRITE, 1);
	MNH_DDR_CTL_OUTf(164, MR13_DATA_0, 0xC0);
	MNH_DDR_CTL_OUTf(508, WR_ORDER_REQ, 0x3);

	MNH_DDR_PI_OUTf(00, PI_START, 1);
	MNH_DDR_CTL_OUTf(00, START, 1);

	while (!(MNH_DDR_CTL_IN(227) & 0x00000010))
		udelay(1000);
	return 0;
}
EXPORT_SYMBOL(mnh_ddr_po_init);
