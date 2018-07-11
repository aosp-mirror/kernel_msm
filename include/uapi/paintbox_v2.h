/*
 * Driver interface for the Paintbox Image Processing Unit
 *
 * Copyright (C) 2017 Google, Inc.
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

#ifndef __PAINTBOX_V2_H__
#define __PAINTBOX_V2_H__

#include <linux/ioctl.h>
#include <uapi/paintbox.h>

/* TODO(ahampson):  This is a temporary Airbrush version of setup line buffer
 * used to exercise Airbrush LBP features.  It will be removed and replace with
 * a job queue command later on.
 */
struct line_buffer_config_v2 {
	struct line_buffer_config base;
	bool enable_chaining;
	bool addr_mode_is_physical;
};

struct dma_lbp_config_v2 {
	struct dma_lbp_config base;
	uint32_t noc_port_id;
	uint32_t slice_id_lsb;
	uint32_t slice_id_width;
};

struct dma_stp_config_v2 {
	struct dma_stp_config base;
	uint32_t noc_port_id;
};

struct dma_transfer_config_v2 {
	uint32_t channel_id;
	enum dma_transfer_type transfer_type;
	struct dma_image_config img;
	union {
		/* MIPI transfers do not require any additional settings */
		struct dma_stp_config_v2 stp;
		struct dma_dram_config dram;
		struct dma_lbp_config_v2 lbp;
	} src;
	union {
		/* MIPI transfers do not require any additional settings */
		struct dma_stp_config_v2 stp;
		struct dma_dram_config dram;
		struct dma_lbp_config_v2 lbp;
	} dst;
	uint32_t sheet_width;
	uint32_t sheet_height;
	uint32_t stripe_height;
	uint32_t noc_outstanding;
	uint32_t retry_interval;

	/* Set to true when the runtime will be waiting for a completion
	 * notification.
	 */
	bool notify_on_completion;

	/* When auto_start_transfer is set to true the transfer will begin
	 * immediately if there are no pending transfers ahead of it and there
	 * is space in the active queue.  If the transfer is unable to start
	 * then it is placed on the pending queue.
	 *
	 * When auto_start_transfer is set to false the transfer will be placed
	 * on the pending queue and PB_START_DMA_TRANSFER will need to be
	 * called to start the transfer.
	 */
	bool auto_start_transfer;
};

/* TODO(ahampson):  These are temporary Airbrush versions of ioctls that will
 * later be replaced by JQS buffers.
 */
#define PB_SETUP_LINE_BUFFER_V2 _IOW('a', 1, struct line_buffer_config_v2)
#define PB_SETUP_DMA_TRANSFER_V2 _IOW('a', 2, struct dma_transfer_config_v2)

#endif /* _PAINTBOX_V2_H_ */
