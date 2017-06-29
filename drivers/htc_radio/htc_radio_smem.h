/*
 * Copyright (c) 2016 HTC Corporation.
 *
 *     @file   /kernel/drivers/htc_radio/htc_radio_smem.h
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

#ifndef _HTC_RADIO_SMEM_H
#define _HTC_RADIO_SMEM_H

#include <linux/types.h>

/*hTC radio smem driver version */
#define HTC_RADIO_SMEM_VERSION	0x20170119

struct htc_smem_type {
	uint32_t	version;
	uint32_t	struct_size;
	uint32_t	htc_smem_pid;
	uint32_t	htc_smem_app_run_mode;
	uint8_t		htc_rom_ver[16];
	uint8_t		htc_smem_skuid[48];
	uint32_t	htc_smem_factory_reset;
	uint32_t	htc_smem_flag;
	uint8_t		reserved[1960];
	/* totally 2048 bytes */
};

#define htc_smem_addr(smem, field) \
	({ \
		volatile void __iomem *__p = (smem); \
		__p += offsetof(typeof(*(smem)), field); \
		__p; \
	})

#define htc_smem_set_u32(p, field, value) \
	writel((value), htc_smem_addr((p), field))

#define htc_smem_copy(p, field, src) \
	memcpy_toio(htc_smem_addr((p), field), (src), sizeof((p)->field))

#endif /* end of _HTC_RADIO_SMEM_H */
