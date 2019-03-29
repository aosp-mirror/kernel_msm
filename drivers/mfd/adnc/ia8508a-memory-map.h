/*
 * ia8508a-memory-map.h -- IA8508A memory map header file
 *
 * Copyright 2019 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef __IA8508A_MEMORY_MAP_H__
#define __IA8508A_MEMORY_MAP_H__

#include <linux/kernel.h>
/******************* ROM START ADDR *********************/
#define IAXXX_HMD_DROM_SYS_START	0xA6200000
#define IAXXX_HMD_IROM_SYS_START	0xA6600000
#define IAXXX_DMX_DROM_SYS_START	0xA8200000
#define IAXXX_DMX_IROM_SYS_START	0xA8600000

#define IAXXX_ROM_ADDR_MAPPING { \
	IAXXX_HMD_DROM_SYS_START, \
	IAXXX_HMD_IROM_SYS_START, \
	IAXXX_DMX_DROM_SYS_START, \
	IAXXX_DMX_IROM_SYS_START, \
}

#define MAX_ADDR_ROM_SIZE	(64 * 1024)

#define IS_WITHIN_ROM_ADDRESS_RANGE(addr, start, size) \
	(((addr) >= (start)) && ((addr) < ((start) + (size))))

#define IS_PHY_START_ADDR_IN_ROM_ADDRESS_RANGE(addr, start) \
	(((addr) <= (start)) && ((addr + MAX_ADDR_ROM_SIZE) > start))
/******************* ROM START ADDR END *******************/

void rom_phy_address_range_check_and_update(uint32_t *phy_addr,
		uint32_t in_bytes, uint32_t *phy_size1,
		uint32_t *phy_addr2, uint32_t *phy_size2);

#endif /* __IA8508A_MEMORY_MAP_H__ */
