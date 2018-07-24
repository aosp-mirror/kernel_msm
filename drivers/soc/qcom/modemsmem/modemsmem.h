/*
 * Copyright (c) 2018 Google Inc.
 *
 *     @file   /kernel/drivers/soc/qcom/modemsmem/modemsmem.h
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

#ifndef _MODEM_SMEM_H
#define _MODEM_SMEM_H

#include <linux/types.h>

/* Modem smem driver version */
#define MODEM_SMEM_VERSION	0x0
#define MODEM_FTM_MAGIC		0x6846544D

struct modem_smem_type {
	uint32_t	version;
	uint32_t	modem_flag;
	uint32_t	major_id;
	uint32_t	minor_id;
	uint32_t	subtype;
	uint32_t	platform;
	uint32_t	efs_magic;
	uint32_t	ftm_magic;
};

#define modem_smem_addr(smem, field) \
	({ \
		volatile void __iomem *__p = (smem); \
		__p += offsetof(typeof(*(smem)), field); \
		__p; \
	})

#define modem_smem_set_u32(p, field, value) \
	writel_relaxed((value), modem_smem_addr((p), field))

#define modem_smem_copy(p, field, src) \
	memcpy_toio(modem_smem_addr((p), field), (src), sizeof((p)->field))

#endif /* end of _MODEM_SMEM_H */
