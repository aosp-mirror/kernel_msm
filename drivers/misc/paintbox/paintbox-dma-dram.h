/*
 * DMA DRAM support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PAINTBOX_DMA_DRAM_H__
#define __PAINTBOX_DMA_DRAM_H__

#include <linux/types.h>

#include "paintbox-common.h"

/* The caller to this function must hold pb->lock */
int ipu_dma_attach_buffer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_dram_config *config, enum dma_data_direction dir);

/* The caller to this function must hold pb->lock */
void ipu_dma_release_buffer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer);

/* The caller to this function must hold pb->lock */
int ipu_dma_release_and_copy_buffer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer, void __user *buf,
		size_t len_bytes);

#endif  /* __PAINTBOX_DMA_DRAM_H__ */
