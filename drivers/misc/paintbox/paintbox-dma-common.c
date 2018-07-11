/*
 * DMA common support for the Paintbox programmable IPU
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
#include "paintbox-common.h"
#include "paintbox-dma-common.h"
#include "paintbox-dma-debug.h"
#include "paintbox-io.h"
#include "paintbox-lbp.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

/* TODO:  Temporarily make DMA configuration validation a debug
 * only operation.
 */
#ifdef DEBUG
/* The caller to this function must hold pb->lock */
static int validate_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct dma_image_config *config)
{
	/* Image Size */
	if (config->width_pixels > DMA_CHAN_IMG_SIZE_WIDTH_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u image width too large, %u max %llu\n",
				__func__, channel->channel_id,
				config->width_pixels,
				DMA_CHAN_IMG_SIZE_WIDTH_MAX);
		return -ERANGE;
	}

	if (config->height_pixels > DMA_CHAN_IMG_SIZE_HEIGHT_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u image height too large, %u max %llu\n",
				__func__, channel->channel_id,
				config->height_pixels,
				DMA_CHAN_IMG_SIZE_HEIGHT_MAX);
		return -ERANGE;
	}

	/* Image Position */
	if (config->start_x_pixels < DMA_CHAN_IMG_POS_START_MIN ||
			config->start_x_pixels > DMA_CHAN_IMG_POS_START_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u start x out of bounds, %d min %d max %d\n",
				__func__, channel->channel_id,
				config->start_x_pixels,
				DMA_CHAN_IMG_POS_START_MIN,
				DMA_CHAN_IMG_POS_START_MAX);
		return -ERANGE;
	}

	if (config->start_y_pixels < DMA_CHAN_IMG_POS_START_MIN ||
			config->start_y_pixels > DMA_CHAN_IMG_POS_START_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u start y out of bounds, %d min %d max %d\n",
				__func__, channel->channel_id,
				config->start_y_pixels,
				DMA_CHAN_IMG_POS_START_MIN,
				DMA_CHAN_IMG_POS_START_MAX);
		return -ERANGE;
	}

	/* Image Format */
	if (config->components < DMA_CHAN_IMG_FORMAT_COMPONENTS_MIN ||
			config->components >
			DMA_CHAN_IMG_FORMAT_COMPONENTS_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid number of components, %u min %u max %llu\n",
				__func__, channel->channel_id,
				config->components,
				DMA_CHAN_IMG_FORMAT_COMPONENTS_MIN,
				DMA_CHAN_IMG_FORMAT_COMPONENTS_MAX);
		return -EINVAL;
	}

	if (config->planes < DMA_CHAN_IMG_FORMAT_PLANES_MIN ||
			config->planes > DMA_CHAN_IMG_FORMAT_PLANES_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid number of planes, %u min %u max %llu)\n",
				__func__, channel->channel_id,
				config->planes, DMA_CHAN_IMG_FORMAT_PLANES_MIN,
				DMA_CHAN_IMG_FORMAT_PLANES_MAX);
		return -EINVAL;
	}

	/* Image Layout */
	if (config->row_stride_bytes > DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid row stride, %u max %llu\n",
				__func__, channel->channel_id,
				config->row_stride_bytes,
				DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_MAX);
		return -EINVAL;
	}

	if (config->plane_stride_bytes > DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid plane stride, %llu max %llu\n",
				__func__, channel->channel_id,
				config->plane_stride_bytes,
				DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_MAX);
		return -EINVAL;
	}

	return 0;
}
#endif

/* The caller to this function must hold pb->lock */
int set_dma_image_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_image_config *config)
{
	/* TODO:  Temporarily make LBP DMA configuration validation a
	 * debug only operation.
	 */
#ifdef DEBUG
	int ret;

	ret = validate_dma_image_parameters(pb, channel, config);
	if (ret < 0)
		return ret;
#endif

	/* Image Start Position */
	paintbox_dma_set_img_start(transfer, (uint64_t)config->start_x_pixels,
			(uint64_t)config->start_y_pixels);

	/* Image Size */
	transfer->chan_img_size = config->width_pixels;
	transfer->chan_img_size |= config->height_pixels <<
			DMA_CHAN_IMG_SIZE_IMG_HEIGHT_SHIFT;

	/* Image Format */
	transfer->chan_img_format = config->components - 1;
	transfer->chan_img_format |= (config->planes - 1) <<
			DMA_CHAN_IMG_FORMAT_PLANES_SHIFT;

	switch (config->bit_depth) {
	case 8:
		transfer->chan_img_format |= DMA_CHAN_IMG_FORMAT_BIT_DEPTH8 <<
				DMA_CHAN_IMG_FORMAT_BIT_DEPTH_SHIFT;
		break;
	case 10:
		transfer->chan_img_format |= DMA_CHAN_IMG_FORMAT_BIT_DEPTH10 <<
				DMA_CHAN_IMG_FORMAT_BIT_DEPTH_SHIFT;
		break;
	case 12:
		transfer->chan_img_format |= DMA_CHAN_IMG_FORMAT_BIT_DEPTH12 <<
				DMA_CHAN_IMG_FORMAT_BIT_DEPTH_SHIFT;
		break;
	case 14:
		transfer->chan_img_format |= DMA_CHAN_IMG_FORMAT_BIT_DEPTH14 <<
				DMA_CHAN_IMG_FORMAT_BIT_DEPTH_SHIFT;
		break;
	case 16:
		transfer->chan_img_format |= DMA_CHAN_IMG_FORMAT_BIT_DEPTH16 <<
				DMA_CHAN_IMG_FORMAT_BIT_DEPTH_SHIFT;
		break;
	default:
		dev_err(pb->dev, "%s: dma channel%u unsupported bit depth %u\n",
			__func__, channel->channel_id, config->bit_depth);
		return -EINVAL;
	};

	switch (config->rgba_format) {
	case RGBA_FORMAT_DISABLED:
#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
		transfer->chan_img_format |=
				DMA_CHAN_IMG_FORMAT_RGBA_FORMAT_DISABLED <<
				DMA_CHAN_IMG_FORMAT_RGBA_FORMAT_SHIFT;
		break;
	case RGBA_FORMAT_RGBA:
		transfer->chan_img_format |=
				DMA_CHAN_IMG_FORMAT_RGBA_FORMAT_RGBA <<
				DMA_CHAN_IMG_FORMAT_RGBA_FORMAT_SHIFT;
		break;
	case RGBA_FORMAT_ARGB:
		transfer->chan_img_format |=
				DMA_CHAN_IMG_FORMAT_RGBA_FORMAT_ARGB <<
				DMA_CHAN_IMG_FORMAT_RGBA_FORMAT_SHIFT;
#endif
		break;
	default:
		dev_err(pb->dev, "%s: dma channel%u: invalid RGBA format %u",
			__func__, channel->channel_id, config->rgba_format);
		return -EINVAL;
	};

	if (config->block4x4)
		transfer->chan_img_format |= DMA_CHAN_IMG_FORMAT_BLOCK_4X4_MASK;

	if (config->mipi_raw_format)
		transfer->chan_img_format |=
				DMA_CHAN_IMG_FORMAT_MIPI_RAW_FORMAT_MASK;

	/* Image Layout */
	transfer->chan_img_layout = ((uint64_t)config->row_stride_bytes <<
			DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_SHIFT) &
			DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_MASK;
	transfer->chan_img_layout |= (config->plane_stride_bytes <<
			DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_SHIFT) &
			DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_MASK;

	return 0;
}

#define IPU_SSP_DATA_WIDTH 128
#define IPU_SSP_SEG_WORDS 32
#define IPU_LBP_PIXEL_BURST_WIDTH 9
#define IPU_LBP_PIXEL_BURST_MSB (IPU_LBP_PIXEL_BURST_WIDTH - 1)

static int paintbox_dma_set_ssp_config(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{
	unsigned int pix_per_loc_int, pixels_per_location, locations_per_row;
	bool is_mipi = config->transfer_type == DMA_MIPI_TO_LBP ||
			config->transfer_type == DMA_MIPI_TO_DRAM;

	switch (config->img.bit_depth) {
	case 8:
	case 16:
		pix_per_loc_int = IPU_SSP_DATA_WIDTH / config->img.bit_depth;
		break;
	case 10:
		pix_per_loc_int = IPU_SSP_DATA_WIDTH / (is_mipi ? 10 : 16);
		break;
	case 12:
		pix_per_loc_int = IPU_SSP_DATA_WIDTH / (is_mipi ? 12 : 16);
		break;
	case 14:
		pix_per_loc_int = IPU_SSP_DATA_WIDTH / 16;
		break;
	default:
		dev_err(pb->dev,
				"%s: dma channel%u bit depth %u is not supported\n",
				__func__, channel->channel_id,
				config->img.bit_depth);
		return -EINVAL;
	};

	pixels_per_location = (config->img.block4x4 || (is_mipi &&
			config->img.components == 2)) ? pix_per_loc_int / 2 :
			pix_per_loc_int;

	locations_per_row = (config->sheet_width / pixels_per_location) +
			!!(config->sheet_width % pixels_per_location);

	/* FIFO depth is currently set to the default value.
	 * TODO(ahampson):  In the future we may want to assign priorities to
	 * certain channels.
	 */
	transfer->chan_ssp_config = DMA_CHAN_SSP_CFG_DEF &
			DMA_CHAN_SSP_CFG_PTRS_FIFO_DEPTH_MASK;

	transfer->chan_ssp_config |= pixels_per_location &
			DMA_CHAN_SSP_CFG_PIX_PER_LOC_MASK;

	transfer->chan_ssp_config |= (locations_per_row &
			DMA_CHAN_SSP_CFG_LOCS_PER_ROW_M) <<
			DMA_CHAN_SSP_CFG_LOCS_PER_ROW_SHIFT;

	if (is_mipi && config->img.components == 1 &&
			(pixels_per_location * IPU_SSP_SEG_WORDS) >
			(1 << IPU_LBP_PIXEL_BURST_MSB))
		transfer->chan_ssp_config |= DMA_CHAN_SSP_CFG_MULT_SHEETS_MASK;

	if (locations_per_row * config->sheet_height > IPU_SSP_SEG_WORDS)
		transfer->chan_ssp_config |= DMA_CHAN_SSP_CFG_MULT_SEGS_MASK;

	return 0;
}

/* The caller to this function must hold pb->lock */
int set_dma_transfer_region_parameters(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{

/* TODO:  Temporarily make DMA configuration validation a debug
 * only operation.
 */
#ifdef DEBUG
	if (config->stripe_height > DMA_CHAN_BIF_XFER_STRIPE_HEIGHT_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid stripe height %u, max %u\n",
				__func__, channel->channel_id,
				config->stripe_height,
				DMA_CHAN_BIF_XFER_STRIPE_HEIGHT_MAX);
		return -EINVAL;
	}

	if (config->sheet_width > DMA_CHAN_NOC_XFER_SHEET_WIDTH_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid sheet width %u, max %u",
				__func__, channel->channel_id,
				config->sheet_width,
				DMA_CHAN_NOC_XFER_SHEET_WIDTH_MAX);
		return -EINVAL;
	}

	if (config->sheet_height > DMA_CHAN_NOC_XFER_SHEET_HEIGHT_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid sheet height %u, max %u\n",
				__func__, channel->channel_id,
				config->sheet_height,
				DMA_CHAN_NOC_XFER_SHEET_HEIGHT_MAX);
		return -EINVAL;
	}

	if (config->noc_outstanding < DMA_CHAN_NOC_XFER_OUTSTANDING_MIN ||
		config->noc_outstanding > DMA_CHAN_NOC_XFER_OUTSTANDING_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid NOC outstanding value %u, min %d max %d\n",
				__func__, channel->channel_id,
				config->noc_outstanding,
				DMA_CHAN_NOC_XFER_OUTSTANDING_MIN,
				DMA_CHAN_NOC_XFER_OUTSTANDING_MAX);
		return -EINVAL;
	}

	if (config->retry_interval < DMA_CHAN_NOC_XFER_RETRY_INTERVAL_MIN ||
			config->retry_interval >
			DMA_CHAN_NOC_XFER_RETRY_INTERVAL_MAX) {
		dev_err(pb->dev,
				"%s: dma channel%u invalid retry interval value %u, min %u max %llu\n",
				__func__, channel->channel_id,
				config->retry_interval,
				DMA_CHAN_NOC_XFER_RETRY_INTERVAL_MIN,
				DMA_CHAN_NOC_XFER_RETRY_INTERVAL_MAX);
		return -EINVAL;
	}
#endif

	transfer->chan_bif_xfer = config->stripe_height;
	transfer->chan_bif_xfer |= (pb->dma.bif_outstanding - 1) <<
			DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT;
	transfer->chan_noc_xfer = config->sheet_width;
	transfer->chan_noc_xfer |= config->sheet_height <<
			DMA_CHAN_NOC_XFER_SHEET_HEIGHT_SHIFT;
	transfer->chan_noc_xfer |= config->noc_outstanding <<
			DMA_CHAN_NOC_XFER_OUTSTANDING_SHIFT;
	transfer->chan_noc_xfer |= (uint64_t)config->retry_interval <<
			DMA_CHAN_NOC_XFER_RETRY_INTERVAL_SHIFT;

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	/* TODO:  DMA_CHAN_NOC_XFER_DYN_OUTSTANDING_MASK is currently
	 * set for all DMA transfers at this time.  This may change in the
	 * future to give priority to MIPI transfers.
	 */
	transfer->chan_noc_xfer |= DMA_CHAN_NOC_XFER_DYN_OUTSTANDING_MASK;
#endif

	return paintbox_dma_set_ssp_config(pb, channel, transfer, config);
}

void paintbox_dma_set_channel_mode(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, uint64_t src_type,
		uint64_t dst_type, bool gather)
{
	unsigned long irq_flags;

	transfer->chan_mode = src_type << DMA_CHAN_MODE_SRC_SHIFT;
	transfer->chan_mode |= dst_type << DMA_CHAN_MODE_DST_SHIFT;
	transfer->chan_mode |= DMA_CHAN_MODE_CHAN_ENA_MASK;

	if (gather) {
		transfer->chan_mode |= DMA_CHAN_MODE_GATHER_MASK;

		spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);
		session->dma_gather_transfer_count++;
		session->dma_gather_channel_mask |= 1 << channel->channel_id;
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
	}
}
