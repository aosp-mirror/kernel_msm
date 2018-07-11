/*
 * DMA LBP support for the Paintbox programmable IPU
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

#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <uapi/paintbox.h>

#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-common.h"
#include "paintbox-dma-debug.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-lbp.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

/* The caller to this function must hold pb->lock */
static int set_dma_lbp_parameters(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_lbp_config_v2 *config)
{
	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	struct paintbox_lb *lb;
	int ret;

	lb = get_lb(pb, session, config->base.lbp_id, config->base.lb_id, &ret);
	if (ret < 0)
		return ret;

	if (config->base.read_ptr_id >= lb->num_read_ptrs) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid rptr id, %u, num read ptrs %u\n",
				__func__, channel->channel_id,
				config->base.read_ptr_id, lb->num_read_ptrs);
		return -EINVAL;
	}

	if (config->base.start_x_pixels < DMA_CHAN_IMG_POS_LB_START_MIN ||
			config->base.start_x_pixels >
			DMA_CHAN_IMG_POS_LB_START_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u lb_start x out of bounds, %d min %d max %d\n",
				__func__, channel->channel_id,
				config->base.start_x_pixels,
				DMA_CHAN_IMG_POS_LB_START_MIN,
				DMA_CHAN_IMG_POS_LB_START_MAX);
		return -ERANGE;
	}

	if (config->base.start_y_pixels < DMA_CHAN_IMG_POS_LB_START_MIN ||
			config->base.start_y_pixels >
			DMA_CHAN_IMG_POS_LB_START_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u lb_start y out of bounds, %d min %d max %d\n",
				 __func__, channel->channel_id,
				config->base.start_y_pixels,
				DMA_CHAN_IMG_POS_LB_START_MIN,
				DMA_CHAN_IMG_POS_LB_START_MAX);
		return -ERANGE;
	}
#endif

	/* Set the LBP node configuration */
	transfer->chan_node = config->base.lbp_id;
	transfer->chan_node |= config->base.lb_id << DMA_CHAN_NODE_LB_ID_SHIFT;
	transfer->chan_node |= config->base.read_ptr_id <<
			DMA_CHAN_NODE_RPTR_ID_SHIFT;
	transfer->chan_node |= config->noc_port_id <<
			DMA_CHAN_NODE_NOC_PORT_SHIFT;
	transfer->chan_node |= (uint64_t)config->slice_id_width <<
			DMA_CHAN_NODE_SLICE_ID_WIDTH_SHIFT;
	transfer->chan_node |= (uint64_t)config->slice_id_lsb <<
			DMA_CHAN_NODE_SLICE_ID_LSB_SHIFT;

	/* Set the line buffer image position */
	paintbox_dma_set_lb_start(transfer,
			(uint64_t)config->base.start_x_pixels,
			(uint64_t)config->base.start_y_pixels);

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{
	int ret;

	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->src.dram.len_bytes > DMA_CHAN_VA_BDRY_LEN_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u transfer too large, %llu max %llu bytes",
				__func__, channel->channel_id,
				config->dst.dram.len_bytes,
				DMA_CHAN_VA_BDRY_LEN_MAX);
		return -ERANGE;
	}
#endif

	ret = ipu_dma_attach_buffer(pb, channel, transfer, &config->src.dram,
			DMA_TO_DEVICE);
	if (ret < 0)
		return ret;

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_start_time = ktime_get_boottime();

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_DRAM, DMA_CHAN_MODE_DST_LBP,
			config->dst.lbp.base.gather);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->dst.lbp);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	LOG_DMA_DRAM_TO_LBP_TRANSFER(pb, channel, transfer, config);

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_finish_time =
				ktime_get_boottime();

	return 0;

err_exit:
	ipu_dma_release_buffer(pb, transfer);

	return ret;
}

/* The caller to this function must hold pb->lock */
int dma_setup_lbp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{
	int ret;

	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->dst.dram.len_bytes > DMA_CHAN_VA_BDRY_LEN_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u transfer too large, %llu max %llu bytes",
				__func__, channel->channel_id,
				config->dst.dram.len_bytes,
				DMA_CHAN_VA_BDRY_LEN_MAX);
		return -ERANGE;
	}
#endif

	ret = ipu_dma_attach_buffer(pb, channel, transfer, &config->dst.dram,
			DMA_FROM_DEVICE);
	if (ret < 0)
		return ret;

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_start_time = ktime_get_boottime();

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_LBP, DMA_CHAN_MODE_DST_DRAM,
			config->src.lbp.base.gather);

	ret = set_dma_lbp_parameters(pb, session, channel, transfer,
			&config->src.lbp);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	LOG_DMA_LBP_TO_DRAM_TRANSFER(pb, channel, transfer, config);

	if (channel->stats.time_stats_enabled)
		channel->stats.non_dram_setup_finish_time =
				ktime_get_boottime();

	return 0;

err_exit:
	ipu_dma_release_buffer(pb, transfer);

	return ret;
}
