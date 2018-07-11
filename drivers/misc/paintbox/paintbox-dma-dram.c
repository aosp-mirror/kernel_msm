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

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-debug.h"
#include "paintbox-dma-dram.h"

static int ipu_copy_user_to_dma_buffer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer,
		struct dma_dram_config *config)
{
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time = ktime_get_boottime();
#endif
	WARN_ON(transfer->buf_vaddr);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		start_time = ktime_get_boottime();
#endif

	transfer->buf_vaddr = dma_alloc_coherent(pb->dev,
			config->len_bytes, &transfer->dma_addr, GFP_KERNEL);
	if (!transfer->buf_vaddr) {
		dev_err(pb->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	/* Copy the entire user buffer into the transfer buffer for both
	 * directions in case the actual transfer is just a stripe.  This will
	 * ensure that portions of the buffer outside of the stripe will be
	 * consistent between the user buffer and the transfer buffer.
	 */
	if (copy_from_user(transfer->buf_vaddr, config->host_vaddr,
			config->len_bytes)) {
		dma_free_coherent(pb->dev, config->len_bytes,
				transfer->buf_vaddr, transfer->dma_addr);
		return -EFAULT;
	}

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb, PB_STATS_DMA_MALLOC,
				start_time, ktime_get_boottime(),
				(size_t)config->len_bytes);
#endif

	return 0;
}

static void ipu_release_kernel_dma_buffer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer)
{
	dev_dbg(pb->dev, "%s: len %lu\n", __func__, transfer->len_bytes);
	dev_dbg(pb->dev, "\tva %p dma addr %pad\n", transfer->buf_vaddr,
			&transfer->dma_addr);

	dma_free_coherent(pb->dev, transfer->len_bytes,
			transfer->buf_vaddr, transfer->dma_addr);
}

static int ipu_import_dma_buf(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_dram_config *config)
{
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time = ktime_get_boottime();
#endif
	int ret;

	transfer->dma_buf = dma_buf_get(config->dma_buf.fd);
	if (IS_ERR(transfer->dma_buf))
		return PTR_ERR(transfer->dma_buf);

	transfer->attach = dma_buf_attach(transfer->dma_buf, pb->dev);
	if (IS_ERR(transfer->attach)) {
		ret = PTR_ERR(transfer->attach);
		dev_err(pb->dev, "%s: failed to attach dma_buf, err %d\n",
				__func__, ret);
		goto err_put;
	}

	transfer->sg_table = dma_buf_map_attachment(transfer->attach,
			transfer->dir);
	if (IS_ERR(transfer->sg_table)) {
		ret = PTR_ERR(transfer->sg_table);
		dev_err(pb->dev, "%s: failed to map dma_buf, err %d\n",
				__func__, ret);
		goto err_detach;
	}

#ifdef CONFIG_PAINTBOX_DEBUG
	if (channel->stats.time_stats_enabled || pb->stats.ioctl_time_enabled) {
		ktime_t end_time = ktime_get_boottime();

		if (channel->stats.time_stats_enabled) {
			channel->stats.dma_buf_map_start_time = start_time;
			channel->stats.dma_buf_map_finish_time = end_time;
		}

		if (pb->stats.ioctl_time_enabled)
			paintbox_debug_log_non_ioctl_stats(pb,
					PB_STATS_CACHE_OP, start_time, end_time,
					0);
	}
#endif

	/* TODO:  dma_buf_offset_bytes + config->len_bytes should be
	 * less than or equal sg_dma_len(transfer->sg_table->sgl).  Currently
	 * the config->len_bytes value supplied by the runtime is not the
	 * actual transfer boundary.  b/35243756
	 */

	/* Map the scatter list into the IOVA space. */
	if (iommu_present(&paintbox_bus_type)) {
		ret = dma_map_sg_attrs(pb->dev, transfer->sg_table->sgl,
				transfer->sg_table->nents, transfer->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
		if (ret == 0) {
			dev_err(pb->dev,
					"%s: DMA map error, nents %d requested",
					__func__, transfer->sg_table->nents);
			ret = -EINVAL;
			goto err_unmap;
		}

	}

	/* TODO:  The buffer offset should be factored in earlier so
	 * we don't map pages we don't need into the IOVA.
	 */
	transfer->dma_addr = sg_dma_address(transfer->sg_table->sgl) +
			config->dma_buf.offset_bytes;

	return 0;

err_unmap:
	dma_buf_unmap_attachment(transfer->attach, transfer->sg_table,
			transfer->dir);
err_detach:
	dma_buf_detach(transfer->dma_buf, transfer->attach);
err_put:
	dma_buf_put(transfer->dma_buf);

	return ret;
}

static void ipu_release_dma_buf(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer)
{
	if (iommu_present(&paintbox_bus_type)) {
		dma_unmap_sg_attrs(pb->dev, transfer->sg_table->sgl,
				transfer->sg_table->nents, transfer->dir,
				DMA_ATTR_SKIP_CPU_SYNC);
	}

	dma_buf_unmap_attachment(transfer->attach, transfer->sg_table,
			transfer->dir);
	dma_buf_detach(transfer->dma_buf, transfer->attach);
	dma_buf_put(transfer->dma_buf);
}

/* The caller to this function must hold pb->lock */
int ipu_dma_attach_buffer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_dram_config *config, enum dma_data_direction dir)
{
	int ret;

	transfer->buffer_type = config->buffer_type;
	transfer->dir = dir;
	transfer->len_bytes = config->len_bytes;

	switch (config->buffer_type) {
	case DMA_DRAM_BUFFER_USER:
		ret = ipu_copy_user_to_dma_buffer(pb, transfer, config);
		break;
	case DMA_DRAM_BUFFER_DMA_BUF:
		ret = ipu_import_dma_buf(pb, channel, transfer, config);
		break;
	case DMA_DRAM_BUFFER_UNUSED:
	default:
		dev_err(pb->dev, "%s: invalid buffer type\n", __func__);
		ret = -EINVAL;
	};

	if (ret < 0)
		return ret;

	transfer->chan_va = (uint64_t)transfer->dma_addr;

	/* VA_BDRY is the virtual address boundary for the DMA transfer.  The
	 * memory transferred by DMA is [VA, VA + VA_BDRY].
	 */
	transfer->chan_va_bdry = (uint64_t)transfer->len_bytes;

	return 0;
}

/* The caller to this function must hold pb->lock */
void ipu_dma_release_buffer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer)
{
	switch (transfer->buffer_type) {
	case DMA_DRAM_BUFFER_USER:
		ipu_release_kernel_dma_buffer(pb, transfer);
		break;
	case DMA_DRAM_BUFFER_DMA_BUF:
		ipu_release_dma_buf(pb, transfer);
		break;
	case DMA_DRAM_BUFFER_UNUSED:
	default:
		break;
	};
}

/* The caller to this function must hold pb->lock */
int ipu_dma_release_and_copy_buffer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer, void __user *buf,
		size_t len_bytes)
{
	if (transfer->dir == DMA_FROM_DEVICE &&
			transfer->buffer_type == DMA_DRAM_BUFFER_USER &&
			buf != NULL) {
		if (WARN_ON(transfer->buf_vaddr == NULL))
			return -EINVAL;

		if (copy_to_user(buf, transfer->buf_vaddr,
				min(transfer->len_bytes, len_bytes)))
			return -EFAULT;
	}

	ipu_dma_release_buffer(pb, transfer);

	return 0;
}
