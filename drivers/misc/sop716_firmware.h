/* Copyright (c) 2017, LGE Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SOP716_FIRMWARE__
#define __SOP716_FIRMWARE__

#include <linux/i2c.h>

#define SOP716FW_MAGIC      "SOP716FW"
#define SOP716FW_MAGIC_SIZE 8


struct sop716fw_header {
	uint8_t magic[SOP716FW_MAGIC_SIZE];
	uint32_t section_num;
	uint32_t section_offset;
	uint32_t firmware_offset;
	uint32_t firmware_size;
	uint8_t reserved1[488];
};

struct sop716fw_section {
	uint32_t start;
	uint32_t length;
};

void sop716fw_validate_firmware(const uint8_t *fw,
		uint8_t *major, uint8_t *minor);
int sop716fw_update_firmware(struct i2c_client *client, const uint8_t *fw);

#endif
