/*
 * DMA STP driver support for the Paintbox programmable IPU
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

#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-common.h"
#include "paintbox-dma-debug.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-stp.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

/* The caller to this function must hold pb->lock */
static int set_dma_stp_parameters(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_dram_config *dram_config,
		struct dma_stp_config_v2 *stp_config)
{
	struct paintbox_stp *stp;
	int ret = 0;

	stp = get_stp(pb, session, stp_config->base.stp_id, &ret);
	if (ret < 0)
		return ret;

	/* For STP DMA transfers the SRAM address and target are encoded into
	 * the LB_START_X and LB_START_Y fields.
	 */
	switch (stp_config->base.sram_target) {
	case SRAM_TARGET_STP_INSTRUCTION_RAM:
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->base.sram_addr,
				DMA_CHAN_LB_START_Y_STP_IRAM);
		break;
	case SRAM_TARGET_STP_CONSTANT_RAM:
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->base.sram_addr,
				DMA_CHAN_LB_START_Y_STP_CRAM);
		break;
	case SRAM_TARGET_STP_SCALAR_RAM:
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->base.sram_addr,
				DMA_CHAN_LB_START_Y_STP_DRAM);
		break;
	case SRAM_TARGET_STP_VECTOR_RAM:
		/* TODO:  Add parameter checks for vector b/30969166
		 */
		paintbox_dma_set_lb_start(transfer,
				(uint64_t)stp_config->base.sram_addr,
				stp_config->base.include_halo ?
				DMA_CHAN_LB_START_Y_STP_ARRAY_32x32 :
				DMA_CHAN_LB_START_Y_STP_ARRAY_16x16);
		break;
	default:
		dev_err(pb->dev,
				"%s: dma channel%u invalid STP SRAM type, %u\n",
				__func__, channel->channel_id,
				stp_config->base.sram_target);
		return -EINVAL;
	};

	/* Set the STP node configuration */
	transfer->chan_node = stp_config->base.stp_id;
	transfer->chan_node |= stp_config->noc_port_id <<
			DMA_CHAN_NODE_NOC_PORT_SHIFT;

	return 0;
}

/* The caller to this function must hold pb->lock */
int dma_setup_dram_to_stp_transfer(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{
	int ret;

	/* TODO:  Temporarily make stp dma configuration validation a
	 * debug only operation.  b/62353362
	 */
#ifdef DEBUG
	if (config->src.dram.len_bytes > DMA_CHAN_VA_BDRY_LEN_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u: transfer too large, %llu max %llu bytes",
				__func__, channel->channel_id,
				config->dst.dram.len_bytes,
				DMA_CHAN_VA_BDRY_LEN_MAX);
		return -ERANGE;
	}

	/* Verify that the target STP is part of the session. */
	ret = validate_stp(pb, session, config->dst.stp.base.stp_id);
	if (ret < 0) {
		dev_err(pb->dev,
				"%s: dma channel%u: stp%u is not part of the session, err %d\n",
				__func__, channel->channel_id,
				config->dst.stp.base.stp_id, ret);
		return ret;
	}
#endif

	ret = ipu_dma_attach_buffer(pb, channel, transfer, &config->src.dram,
			DMA_TO_DEVICE);
	if (ret < 0)
		return ret;

	paintbox_dma_set_channel_mode(pb, session, channel, transfer,
			DMA_CHAN_MODE_SRC_DRAM, DMA_CHAN_MODE_DST_STP, false);

	ret = set_dma_stp_parameters(pb, session, channel, transfer,
			&config->src.dram, &config->dst.stp);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_image_parameters(pb, channel, transfer, &config->img);
	if (ret < 0)
		goto err_exit;

	ret = set_dma_transfer_region_parameters(pb, channel, transfer, config);
	if (ret < 0)
		goto err_exit;

	LOG_DMA_DRAM_TO_STP_TRANSFER(pb, channel, transfer, config);

	return 0;

err_exit:
	ipu_dma_release_buffer(pb, transfer);

	return ret;
}
