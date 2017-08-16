/*
 *
 * MNH eFuse Driver
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

#include <linux/delay.h>
#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-scu.h"
#include "mnh-efuse.h"

static uint32_t initialized;
static uint32_t rows[MNH_EFUSE_ROWS] = { 0, 0, 0, 0, 0, 0, 0, 0 };

static const uintptr_t scu_base = HWIO_SCU_BASE_ADDR;
#define SCU_IN(reg)             HW_IN(scu_base, SCU, reg)
#define SCU_INf(reg, fld)       HW_INf(scu_base, SCU, reg, fld)
#define SCU_OUT(reg, val)       HW_OUT(scu_base, SCU, reg, val)
#define SCU_OUTf(reg, fld, val) HW_OUTf(scu_base, SCU, reg, fld, val)

#define EFUSE_IN(reg)             HW_IN(scu_base, SCU, EFUSE_##reg)
#define EFUSE_INf(reg, fld)            HW_INf(scu_base, SCU, EFUSE_##reg, fld)
#define EFUSE_OUTf(reg, fld, val) HW_OUTf(scu_base, SCU, EFUSE_##reg, fld, val)

#define MNH_EFUSE_T_SUR_PD	800
#define MNH_EFUSE_T_RD		100
#define MNH_EFUSE_T_HR_PS_CS	50

static void mnh_efuse_read_raw(void)
{
#ifdef MNH_FULL_EMULATION
	/* populate dummy fuse data */
	rows[0] = 0x12345678;
	rows[1] = 0x12345678;
	rows[2] = 0x9abcdef0;
	rows[3] = 0x9abcdef0;
	rows[4] = 0x11223344;
	rows[5] = 0x11223344;
	rows[6] = 0x55667788;
	rows[7] = 0x55667788;
#else
	uint32_t row = 0;

	EFUSE_OUTf(PASSCODE, PASSCODE, 0x9DC4);
	EFUSE_OUTf(CTRL, PD, 0);
	/*
	 * PS should always be low except for write/programming mode
	 * according to efuse datasheet
	 */
	EFUSE_OUTf(CTRL, PS, 0);
	ndelay(MNH_EFUSE_T_SUR_PD);
	/* looking at the efuse datasheet, I reordered these from HAS */
	EFUSE_OUTf(ACC, CSB, 0);
	EFUSE_OUTf(CTRL, PGENB, 1);
	EFUSE_OUTf(CTRL, LOAD, 1);

	for (row = 0; row < MNH_EFUSE_ROWS; row++) {
		EFUSE_OUTf(ACC, ZEROED, 0);
		EFUSE_OUTf(ACC, ROW, row);
		EFUSE_OUTf(ACC, STROBE, 1);
		ndelay(MNH_EFUSE_T_RD);
		EFUSE_OUTf(ACC, STROBE, 0);
		ndelay(MNH_EFUSE_T_RD/2);
		rows[row] = EFUSE_IN(READ_DATA);
	}

	EFUSE_OUTf(ACC, CSB, 1);
	ndelay(MNH_EFUSE_T_HR_PS_CS);

	EFUSE_OUTf(CTRL, PD, 1);
	EFUSE_OUTf(PASSCODE, PASSCODE, 0);
#endif /* MNH_FULL_EMULATION */
	initialized = 1;
}

uint32_t mnh_efuse_read(const struct mnh_efuse_addr *addr)
{
	uint32_t high, low, row1, row2, mask, value = 0;

	if (!initialized)
		mnh_efuse_read_raw();

	if (addr) {
		high = addr->bit_high;
		low  = addr->bit_low;
		row1 = addr->row1;
		row2 = addr->row2;

		if ((high < MNH_EFUSE_ROW_WIDTH) &&
		(low <= high) &&
		(row1 < MNH_EFUSE_ROWS) &&
		(row2 < MNH_EFUSE_ROWS)) {
			mask = (1 << ((high - low) + 1)) - 1;
			value = ((rows[row1] | rows[row2]) >> low) & mask;
		} else {
			pr_err("%s: address is malformed!\n", __func__);
		}
	}
	return value;
}
