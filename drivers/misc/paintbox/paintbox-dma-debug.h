/*
 * DMA debug support for the Paintbox programmable IPU
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

#ifndef __PAINTBOX_DMA_DEBUG_H__
#define __PAINTBOX_DMA_DEBUG_H__

#include <linux/types.h>

#include "paintbox-common.h"
#include "paintbox-regs.h"

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 2
#define DMA_DEBUG_BUFFER_SIZE ((DMA_TOP_NUM_REGS + DMA_GRP_NUM_REGS) * \
		REG_DEBUG_BUFFER_SIZE)
#else
#define DMA_DEBUG_BUFFER_SIZE (DMA_NUM_REGS * REG_DEBUG_BUFFER_SIZE)
#endif

#ifdef VERBOSE_DEBUG
void paintbox_log_dma_registers(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, const char *msg);

#define LOG_DMA_REGISTERS(pb, channel)		\
	paintbox_log_dma_registers(pb, channel, __func__)

#else
#define LOG_DMA_REGISTERS(pb, channel)		\
do { } while (0)
#endif

#ifdef DEBUG
void paintbox_log_dma_dram_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);
void paintbox_log_dma_lbp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);
void paintbox_log_dma_dram_to_stp_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config *config);

#define LOG_DMA_DRAM_TO_LBP_TRANSFER(pb, channel, transfer, config)	\
	paintbox_log_dma_dram_to_lbp_transfer(pb, channel, transfer, config)
#define LOG_DMA_LBP_TO_DRAM_TRANSFER(pb, channel, transfer, config)	\
	paintbox_log_dma_lbp_to_dram_transfer(pb, channel, transfer, config)
#define LOG_DMA_DRAM_TO_STP_TRANSFER(pb, channel, transfer, config)	\
	paintbox_log_dma_dram_to_stp_transfer(pb, channel, transfer, config)
#else
#define LOG_DMA_DRAM_TO_LBP_TRANSFER(pb, channel, transfer, config)	\
do { } while (0)
#define LOG_DMA_LBP_TO_DRAM_TRANSFER(pb, channel, transfer, config)	\
do { } while (0)
#define LOG_DMA_DRAM_TO_STP_TRANSFER(pb, channel, transfer, config)	\
do { } while (0)
#endif

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_dma_registers(struct paintbox_debug *debug, char *buf,
		size_t len);
int paintbox_dump_dma_channel_registers(struct paintbox_debug *debug, char *buf,
		size_t len);

void paintbox_dma_debug_init(struct paintbox_data *pb);
void paintbox_dma_debug_remove(struct paintbox_data *pb);
void paintbox_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);
void paintbox_dma_channel_debug_remove(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);
#else
static inline void paintbox_dma_debug_init(struct paintbox_data *pb) { }
static inline void paintbox_dma_debug_remove(struct paintbox_data *pb) { }
static inline void paintbox_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel) { }
static inline void paintbox_dma_channel_debug_remove(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel) { }
#endif

#endif  /* __PAINTBOX_DMA_DEBUG_H__ */
