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

#ifndef __MNH_EFUSE_H__
#define __MNH_EFUSE_H__


#define MNH_EFUSE_ROWS 8
#define MNH_EFUSE_ROW_WIDTH 32

struct mnh_efuse_addr {
	uint32_t row1;
	uint32_t row2;
	/* zero based, inclusive */
	uint32_t bit_high;
	uint32_t bit_low;
};

uint32_t mnh_efuse_read(const struct mnh_efuse_addr *addr);

#endif /* __MNH_EFUSE_H__ */
