/*
 * DMA debug support V2 for the Paintbox programmable IPU
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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <uapi/paintbox.h>
#include <uapi/paintbox_v2.h>

#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-debug.h"
#include "paintbox-regs.h"

static const char *dma_top_reg_names[DMA_TOP_NUM_REGS] = {
	REG_NAME_ENTRY(DMA_CTRL),
	REG_NAME_ENTRY(DMA_IRQ_ISR),
	REG_NAME_ENTRY(DMA_IRQ_VA_ERR_ISR),
	REG_NAME_ENTRY(DMA_IRQ_SSP_ERR_ISR),
	REG_NAME_ENTRY(DMA_IRQ_ITR),
	REG_NAME_ENTRY(DMA_IRQ_VA_ERR_ITR),
	REG_NAME_ENTRY(DMA_IRQ_SSP_ERR_ITR),
	REG_NAME_ENTRY(DMA_IRQ_IER),
	REG_NAME_ENTRY(DMA_IRQ_VA_ERR_IER),
	REG_NAME_ENTRY(DMA_IRQ_SSP_ERR_IER),
	REG_NAME_ENTRY(DMA_IRQ_IMR),
	REG_NAME_ENTRY(DMA_IRQ_VA_ERR_IMR),
	REG_NAME_ENTRY(DMA_IRQ_SSP_ERR_IMR),
	REG_NAME_ENTRY(DMA_IRQ_ISR_OVF),
	REG_NAME_ENTRY(DMA_IRQ_VA_ERR_ISR_OVF),
	REG_NAME_ENTRY(DMA_IRQ_SSP_ERR_ISR_OVF),
	REG_NAME_ENTRY(DMA_PMON_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_0_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_0),
	REG_NAME_ENTRY(DMA_PMON_CNT_0_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_0_STS),
	REG_NAME_ENTRY(DMA_PMON_CNT_1_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_1),
	REG_NAME_ENTRY(DMA_PMON_CNT_1_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_1_STS),
	REG_NAME_ENTRY(DMA_PMON_CNT_2_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_2),
	REG_NAME_ENTRY(DMA_PMON_CNT_2_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_2_STS),
	REG_NAME_ENTRY(DMA_PMON_CNT_3_CFG),
	REG_NAME_ENTRY(DMA_PMON_CNT_3),
	REG_NAME_ENTRY(DMA_PMON_CNT_3_STS_ACC),
	REG_NAME_ENTRY(DMA_PMON_CNT_3_STS),
	REG_NAME_ENTRY(DMA_STAT_CTRL),
	REG_NAME_ENTRY(DMA_STAT_STATE),
	REG_NAME_ENTRY(DMA_STAT_PTR),
	REG_NAME_ENTRY(DMA_STAT_ADDR),
	REG_NAME_ENTRY(DMA_SPARE)
};

static const char *dma_grp_reg_names[DMA_GRP_NUM_REGS] = {
	REG_NAME_ENTRY(DMA_SEL),
	REG_NAME_ENTRY(DMA_CHAN_CTRL),
	REG_NAME_ENTRY(DMA_CHAN_MODE),
	REG_NAME_ENTRY(DMA_CHAN_IMG_FORMAT),
	REG_NAME_ENTRY(DMA_CHAN_IMG_SIZE),
	REG_NAME_ENTRY(DMA_CHAN_IMG_POS),
	REG_NAME_ENTRY(DMA_CHAN_IMG_LAYOUT),
	REG_NAME_ENTRY(DMA_CHAN_BIF_XFER),
	REG_NAME_ENTRY(DMA_CHAN_VA),
	REG_NAME_ENTRY(DMA_CHAN_VA_BDRY),
	REG_NAME_ENTRY(DMA_CHAN_NOC_XFER),
	REG_NAME_ENTRY(DMA_CHAN_SSP_CFG),
	REG_NAME_ENTRY(DMA_CHAN_NODE),
	REG_NAME_ENTRY(DMA_CHAN_MODE_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_FORMAT_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_SIZE_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_POS_RO),
	REG_NAME_ENTRY(DMA_CHAN_IMG_LAYOUT_RO),
	REG_NAME_ENTRY(DMA_CHAN_BIF_XFER_RO),
	REG_NAME_ENTRY(DMA_CHAN_VA_RO),
	REG_NAME_ENTRY(DMA_CHAN_VA_BDRY_RO),
	REG_NAME_ENTRY(DMA_CHAN_NOC_XFER_RO),
	REG_NAME_ENTRY(DMA_CHAN_SSP_CFG_RO),
	REG_NAME_ENTRY(DMA_CHAN_NODE_RO),
	REG_NAME_ENTRY(DMA_GRP_SPARE)
};

static void paintbox_log_dma_common_transfer(struct paintbox_data *pb,
		struct dma_transfer_config_v2 *config)
{
	dev_info(pb->dev,
			"\twidth %upx height %upx planes %u components %u bit depth %u bits\n",
			config->img.width_pixels, config->img.height_pixels,
			config->img.planes, config->img.components,
			config->img.bit_depth);
	dev_info(pb->dev,
			"\tplane stride %llu bytes row stride %u bytes\n",
			config->img.plane_stride_bytes,
			config->img.row_stride_bytes);
	dev_info(pb->dev,
			"\tstart X %dpx start Y %dpx block4x4 %d mipi raw format %d rgba format %d\n",
			config->img.start_x_pixels,
			config->img.start_y_pixels, config->img.block4x4,
			config->img.mipi_raw_format, config->img.rgba_format);
	dev_info(pb->dev,
			"\tsheet width %upx sheet height %u x stripe height %u rows\n",
			config->sheet_width, config->sheet_height,
			config->stripe_height);
	dev_info(pb->dev,
			"\tnoc outstanding %u retry interval %u\n",
			config->noc_outstanding, config->retry_interval);
	dev_info(pb->dev, "\tnotify on completion %d auto start %d\n",
			config->notify_on_completion,
			config->auto_start_transfer);
}

/* The caller to this function must hold pb->lock */
void paintbox_log_dma_dram_to_lbp_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{
	dev_info(pb->dev, "dma channel%u setup transfer %p\n",
			channel->channel_id, transfer);
	if (config->src.dram.buffer_type == DMA_DRAM_BUFFER_USER) {
		dev_info(pb->dev,
				"\tva %p dma addr %pad %llu bytes -> lbp%u lb%u rptr id %u\n",
				config->src.dram.host_vaddr,
				&transfer->dma_addr, config->src.dram.len_bytes,
				config->dst.lbp.base.lbp_id,
				config->dst.lbp.base.lb_id,
				config->dst.lbp.base.read_ptr_id);
	} else {
		dev_info(pb->dev,
				"\tdma buf fd %d offset %zu bytes dma_addr %pad %llu bytes -> lbp%u lb%u rptr id %u\n",
				config->src.dram.dma_buf.fd,
				config->src.dram.dma_buf.offset_bytes,
				&transfer->dma_addr, config->src.dram.len_bytes,
				config->dst.lbp.base.lbp_id,
				config->dst.lbp.base.lb_id,
				config->dst.lbp.base.read_ptr_id);
	}

	dev_info(pb->dev,
				"\tlb start X %dpx lb start Y %dpx gather %d\n",
				config->dst.lbp.base.start_x_pixels,
				config->dst.lbp.base.start_y_pixels,
				config->dst.lbp.base.gather);

	paintbox_log_dma_common_transfer(pb, config);
}

/* The caller to this function must hold pb->lock */
void paintbox_log_dma_lbp_to_dram_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{
	dev_info(pb->dev, "dma channel%u setup transfer %p\n",
			channel->channel_id, transfer);
	if (config->dst.dram.buffer_type == DMA_DRAM_BUFFER_USER) {
		dev_info(pb->dev,
				"\tlbp%u lb%u rptr id %u -> va %p dma addr %pad %llu bytes\n",
				config->src.lbp.base.lbp_id,
				config->src.lbp.base.lb_id,
				config->src.lbp.base.read_ptr_id,
				config->dst.dram.host_vaddr,
				&transfer->dma_addr,
				config->dst.dram.len_bytes);
	} else {
		dev_info(pb->dev,
				"\tlbp%u lb%u rptr id %u -> dma buf fd %d offset %zu bytes dma_addr %pad %llu bytes\n",
				config->src.lbp.base.lbp_id,
				config->src.lbp.base.lb_id,
				config->src.lbp.base.read_ptr_id,
				config->dst.dram.dma_buf.fd,
				config->dst.dram.dma_buf.offset_bytes,
				&transfer->dma_addr,
				config->dst.dram.len_bytes);
	}

	dev_info(pb->dev,
				"\tlb start X %dpx lb start Y %dpx gather %d\n",
				config->src.lbp.base.start_x_pixels,
				config->src.lbp.base.start_y_pixels,
				config->src.lbp.base.gather);

	paintbox_log_dma_common_transfer(pb, config);
}

/* The caller to this function must hold pb->lock */
void paintbox_log_dma_dram_to_stp_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer,
		struct dma_transfer_config_v2 *config)
{
	dev_info(pb->dev, "dma channel%u setup transfer %p\n",
			channel->channel_id, transfer);

	if (config->src.dram.buffer_type == DMA_DRAM_BUFFER_USER) {
		dev_info(pb->dev,
				"\tva %p dma addr %pad %llu bytes -> stp%u sram addr 0x%08x\n",
				config->src.dram.host_vaddr,
				&transfer->dma_addr, config->src.dram.len_bytes,
				config->dst.stp.base.stp_id,
				config->dst.stp.base.sram_addr);
	} else {
		dev_info(pb->dev,
				"\tdma buf fd %d offset %zu bytes dma_addr %pad %llu bytes -> stp%u sram addr 0x%08x\n",
				config->src.dram.dma_buf.fd,
				config->src.dram.dma_buf.offset_bytes,
				&transfer->dma_addr, config->src.dram.len_bytes,
				config->dst.stp.base.stp_id,
				config->dst.stp.base.sram_addr);
	}

	paintbox_log_dma_common_transfer(pb, config);
}

#ifdef CONFIG_PAINTBOX_DEBUG
static uint64_t paintbox_dma_reg_entry_read(
			struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	uint64_t val;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	val = paintbox_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
			reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	return val;
}

static void paintbox_dma_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	paintbox_writeq(pb->dev, val, IPU_CSR_DMA_TOP_OFFSET +
			reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

static uint64_t paintbox_dma_channel_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	uint64_t val;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	paintbox_dma_select_channel(pb, channel->channel_id);

	val = paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return val;
}

static void paintbox_dma_channel_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	paintbox_dma_select_channel(pb, channel->channel_id);

	paintbox_writeq(pb->dev, val, IPU_CSR_DMA_GRP_OFFSET +
			reg_entry->reg_offset);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);
}
#endif

static inline int dump_dma_top_reg(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len)
{
	const char *reg_name = dma_top_reg_names[REG_INDEX(reg_offset)];

	return dump_ipu_register_with_value(pb, IPU_CSR_DMA_TOP_OFFSET,
			reg_offset, reg_value, reg_name, buf, written, len);
}

static inline int dump_dma_grp_reg(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len)
{
	const char *reg_name = dma_grp_reg_names[REG_INDEX(reg_offset)];

	return dump_ipu_register_with_value(pb, IPU_CSR_DMA_GRP_OFFSET,
			reg_offset, reg_value, reg_name, buf, written, len);
}

static int dump_dma_top_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_dma_top_reg(pb, reg_offset, reg_value, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static int dump_dma_grp_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = dump_dma_grp_reg(pb, reg_offset, reg_value, buf, written, len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = dump_ipu_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static inline const char *dma_swizzle_to_str(uint64_t val)
{
	switch ((val & DMA_CTRL_AXI_SWIZZLE_MASK) >>
			DMA_CTRL_AXI_SWIZZLE_SHIFT) {
	case DMA_CTRL_AXI_SWIZZLE_NONE:
		return "NONE";
	case DMA_CTRL_AXI_SWIZZLE_BIG_ENDIAN:
		return "BIG ENDIAN";
	case DMA_CTRL_AXI_SWIZZLE_NEIGHBOR_BYTES:
		return "NEIGHBOR BYTES";
	default:
		return "UNKNOWN";
	};
}

/* The caller to this function must hold pb->lock */
int paintbox_dump_dma_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	uint64_t dma_top_registers[DMA_TOP_NUM_REGS];
	struct paintbox_data *pb = debug->pb;
	uint64_t val;
	uint32_t dma_sel_val;
	unsigned long irq_flags;
	unsigned int i, reg_offset;
	int ret, written = 0;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	dma_sel_val = paintbox_readl(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_SEL);

	for (reg_offset = DMA_CTRL; reg_offset < DMA_TOP_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {
		if (!dma_top_reg_names[REG_INDEX(reg_offset)])
			continue;

		dma_top_registers[REG_INDEX(reg_offset)] =
				paintbox_readq(pb->dev,
				IPU_CSR_DMA_TOP_OFFSET + reg_offset);
	}

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	ret = dump_dma_grp_reg_verbose(pb, DMA_SEL, dma_sel_val, buf, &written,
			len, "\tGRP_SEL %u CHAN_SEL %u\n",
			(dma_sel_val & DMA_SEL_GRP_SEL_MASK) >>
			DMA_SEL_GRP_SEL_SHIFT,
			dma_sel_val & DMA_SEL_CHAN_SEL_MASK);
	if (ret < 0)
		goto err_exit;

	val = dma_top_registers[REG_INDEX(DMA_CTRL)];
	ret = dump_dma_top_reg_verbose(pb, DMA_CTRL, val, buf, &written, len,
			"\tAXI_SWIZZLE %s RESET %d\n", dma_swizzle_to_str(val),
			val & DMA_CTRL_DMA_RESET_MASK);
	if (ret < 0)
		goto err_exit;

	for (i = REG_INDEX(DMA_IRQ_ISR); i <= REG_INDEX(DMA_SPARE); i++) {
		if (dma_top_reg_names[i] != NULL) {
			val = dma_top_registers[i];
			ret = dump_dma_top_reg(pb, i * IPU_REG_WIDTH, val, buf,
					&written, len);
			if (ret < 0)
				goto err_exit;
		}
	}

	return written;

err_exit:
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}

static inline unsigned int get_bit_depth(uint64_t val)
{
	switch ((val & DMA_CHAN_IMG_FORMAT_BIT_DEPTH_MASK) >>
			DMA_CHAN_IMG_FORMAT_BIT_DEPTH_SHIFT) {
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH8:
		return 8;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH10:
		return 10;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH12:
		return 12;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH14:
		return 14;
	case DMA_CHAN_IMG_FORMAT_BIT_DEPTH16:
		return 16;
	default:
		return 0;
	};
}

static inline const char *dma_src_to_str(uint64_t val)
{
	switch ((val & DMA_CHAN_MODE_SRC_MASK) >> DMA_CHAN_MODE_SRC_SHIFT) {
	case DMA_CHAN_MODE_SRC_DRAM:
		return "DRAM";
	case DMA_CHAN_MODE_SRC_LBP:
		return "LBP";
	case DMA_CHAN_MODE_SRC_STP:
		return "STP";
	case DMA_CHAN_MODE_SRC_MIPI_IN:
		return "MIPI IN";
	default:
		return "UNKNOWN";
	};
}

static inline const char *dma_dst_to_str(uint64_t val)
{
	switch ((val & DMA_CHAN_MODE_DST_MASK) >> DMA_CHAN_MODE_DST_SHIFT) {
	case DMA_CHAN_MODE_DST_DRAM:
		return "DRAM";
	case DMA_CHAN_MODE_DST_LBP:
		return "LBP";
	case DMA_CHAN_MODE_DST_STP:
		return "STP";
	case DMA_CHAN_MODE_DST_MIPI_OUT:
		return "MIPI OUT";
	default:
		return "UNKNOWN";
	};
}

static int dump_chan_mode_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tGATHER %u ADDR_MODE %s DST %s SRC %s ENA %u\n",
			!!(val & DMA_CHAN_MODE_GATHER_MASK),
			val & DMA_CHAN_MODE_ADDR_MODE_MASK ? "PHYSICAL" :
			"ABSTRACT",
			dma_dst_to_str(val), dma_src_to_str(val),
			!!(val & DMA_CHAN_MODE_CHAN_ENA_MASK));
}

static int dump_chan_img_format_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tBLOCK_4x4 %u MIPI_RAW_FORMAT %d BIT DEPTH %u PLANES %u COMPONENTS %u\n",
			!!(val & DMA_CHAN_IMG_FORMAT_BLOCK_4X4_MASK),
			!!(val & DMA_CHAN_IMG_FORMAT_MIPI_RAW_FORMAT_MASK),
			get_bit_depth(val),
			((val & DMA_CHAN_IMG_FORMAT_PLANES_MASK) >>
					DMA_CHAN_IMG_FORMAT_PLANES_SHIFT) + 1,
			(val & DMA_CHAN_IMG_FORMAT_COMPONENTS_MASK) + 1);
}

static int dump_chan_img_size_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tIMG_HEIGHT %u IMG_WIDTH %u\n",
			(val & DMA_CHAN_IMG_SIZE_IMG_HEIGHT_MASK) >>
					DMA_CHAN_IMG_SIZE_IMG_HEIGHT_SHIFT,
			val & DMA_CHAN_IMG_SIZE_IMG_WIDTH_MASK);
}

static int dump_chan_img_position_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tLB_START_Y %d LB_START_X %d START_Y %u START_X %u\n",
			(int16_t)((val & DMA_CHAN_IMG_POS_LB_START_Y_MASK) >>
					DMA_CHAN_IMG_POS_LB_START_Y_SHIFT),
			(int16_t)((val & DMA_CHAN_IMG_POS_LB_START_X_MASK) >>
					DMA_CHAN_IMG_POS_LB_START_X_SHIFT),
			(int16_t)((val & DMA_CHAN_IMG_POS_START_Y_MASK) >>
					DMA_CHAN_IMG_POS_START_Y_SHIFT),
			(int16_t)(val & DMA_CHAN_IMG_POS_START_X_MASK));
}

static int dump_chan_img_layout_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tPLANE_STRIDE %llu ROW_STRIDE %u\n",
			(val & DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_MASK) >>
			DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_SHIFT,
			(val & DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_MASK) >>
			DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_SHIFT);
}

static int dump_chan_bif_transfer_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tOUTSTANDING %u STRIPE_HEIGHT %u\n",
			(val & DMA_CHAN_BIF_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT,
			val & DMA_CHAN_BIF_XFER_STRIPE_HEIGHT_MASK);
}

static int dump_chan_va_register(struct paintbox_data *pb, uint32_t reg_offset,
		uint64_t val, char *buf, int *written, size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tVA 0x%016llx\n", val & DMA_CHAN_VA_BASE_MASK);
}

static int dump_chan_va_bdry_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tVA BDRY %llu\n", val & DMA_CHAN_VA_BDRY_LEN_MASK);
}

static int dump_chan_noc_transfer_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tRETRY_INTERVAL %u OUTSTANDING %u SHEET_HEIGHT %u SHEET_WIDTH %u\n",
			(val & DMA_CHAN_NOC_XFER_RETRY_INTERVAL_MASK) >>
			DMA_CHAN_NOC_XFER_RETRY_INTERVAL_SHIFT,
			(val & DMA_CHAN_NOC_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_NOC_XFER_OUTSTANDING_SHIFT,
			(val & DMA_CHAN_NOC_XFER_SHEET_HEIGHT_MASK) >>
			DMA_CHAN_NOC_XFER_SHEET_HEIGHT_SHIFT,
			val & DMA_CHAN_NOC_XFER_SHEET_WIDTH_MASK);
}


static int dump_chan_ssp_config_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tMULT_SEGS %d MULT_SHEETS %d LOCS_PER_ROW %u PIX_PER_LOC %u\n",
			!!(val & DMA_CHAN_SSP_CFG_MULT_SEGS_MASK),
			!!(val & DMA_CHAN_SSP_CFG_MULT_SHEETS_MASK),
			(val & DMA_CHAN_SSP_CFG_LOCS_PER_ROW_MASK) >>
			DMA_CHAN_SSP_CFG_LOCS_PER_ROW_SHIFT,
			val & DMA_CHAN_SSP_CFG_PIX_PER_LOC_MASK);
}

static int dump_chan_node_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return dump_dma_grp_reg_verbose(pb, reg_offset, val, buf, written, len,
			"\tCORE_ID %u LB_ID %u RPTR_ID %u\n",
			val & DMA_CHAN_NODE_CORE_ID_MASK,
			(val & DMA_CHAN_NODE_LB_ID_MASK) >>
			DMA_CHAN_NODE_LB_ID_SHIFT,
			(val & DMA_CHAN_NODE_RPTR_ID_MASK) >>
			DMA_CHAN_NODE_RPTR_ID_SHIFT);
}

static inline uint64_t get_reg_value(uint64_t *reg_values, uint32_t reg_offset)
{
	return reg_values[REG_INDEX(reg_offset)];
}

/* The caller to this function must hold pb->dma.dma_lock. */
void paintbox_log_dma_registers(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, const char *msg)
{
	dev_info(pb->dev, "%s\n", msg);
	dump_chan_mode_register(pb, DMA_CHAN_MODE_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_MODE_RO), NULL, NULL, 0);

	dump_chan_img_format_register(pb, DMA_CHAN_IMG_FORMAT_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_IMG_FORMAT_RO), NULL, NULL, 0);

	dump_chan_img_size_register(pb, DMA_CHAN_IMG_SIZE_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_IMG_SIZE_RO), NULL, NULL, 0);

	dump_chan_img_position_register(pb, DMA_CHAN_IMG_POS_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_IMG_POS_RO), NULL, NULL, 0);

	dump_chan_img_layout_register(pb, DMA_CHAN_IMG_LAYOUT_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_IMG_LAYOUT_RO), NULL, NULL, 0);

	dump_chan_bif_transfer_register(pb, DMA_CHAN_BIF_XFER_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_BIF_XFER_RO), NULL, NULL, 0);

	dump_chan_va_register(pb, DMA_CHAN_VA_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_VA_RO), NULL, NULL, 0);

	dump_chan_va_bdry_register(pb, DMA_CHAN_VA_BDRY_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_VA_BDRY_RO), NULL, NULL, 0);

	dump_chan_noc_transfer_register(pb, DMA_CHAN_NOC_XFER_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_NOC_XFER_RO), NULL, NULL, 0);

	dump_chan_ssp_config_register(pb, DMA_CHAN_SSP_CFG_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_SSP_CFG_RO), NULL, NULL, 0);

	dump_chan_node_register(pb, DMA_CHAN_NODE_RO,
			paintbox_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_NODE_RO), NULL, NULL, 0);
}

#ifdef CONFIG_PAINTBOX_DEBUG
int paintbox_dump_dma_channel_registers(struct paintbox_debug *debug, char *buf,
			size_t len)
{
	uint64_t reg_values[DMA_GRP_NUM_REGS];
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned long irq_flags;
	unsigned int reg_offset;
	int ret, written = 0;
	uint64_t val;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	paintbox_dma_select_channel(pb, channel->channel_id);

	for (reg_offset = 0; reg_offset < DMA_GRP_BLOCK_END; reg_offset +=
			IPU_REG_WIDTH) {
		if (!dma_grp_reg_names[REG_INDEX(reg_offset)])
			continue;

		reg_values[REG_INDEX(reg_offset)] = paintbox_readq(pb->dev,
				IPU_CSR_DMA_GRP_OFFSET + reg_offset);
	}

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	val = get_reg_value(reg_values, DMA_CHAN_CTRL);
	ret = dump_dma_grp_reg_verbose(pb, DMA_CHAN_CTRL, val, buf, &written,
			len,
			"\tSTOP %d CONTINUOUS %d CHAN_RESET %d\n",
			(val & DMA_CHAN_CTRL_STOP_MASK) >>
					DMA_CHAN_CTRL_STOP_SHIFT,
			(val & DMA_CHAN_CTRL_CONTINUOUS_MASK) >>
					DMA_CHAN_CTRL_CONTINUOUS_SHIFT,
			val & DMA_CHAN_CTRL_CHAN_RESET_MASK);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_mode_register(pb, DMA_CHAN_MODE,
			get_reg_value(reg_values, DMA_CHAN_MODE), buf, &written,
			len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_format_register(pb, DMA_CHAN_IMG_FORMAT,
			get_reg_value(reg_values, DMA_CHAN_IMG_FORMAT),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_size_register(pb, DMA_CHAN_IMG_SIZE,
			get_reg_value(reg_values, DMA_CHAN_IMG_SIZE), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_position_register(pb, DMA_CHAN_IMG_POS,
			get_reg_value(reg_values, DMA_CHAN_IMG_POS),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_layout_register(pb, DMA_CHAN_IMG_LAYOUT,
			get_reg_value(reg_values, DMA_CHAN_IMG_LAYOUT), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_bif_transfer_register(pb, DMA_CHAN_BIF_XFER,
			get_reg_value(reg_values, DMA_CHAN_BIF_XFER), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_va_register(pb, DMA_CHAN_VA,
			get_reg_value(reg_values, DMA_CHAN_VA), buf, &written,
			len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_va_bdry_register(pb, DMA_CHAN_VA_BDRY,
			get_reg_value(reg_values, DMA_CHAN_VA_BDRY), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_noc_transfer_register(pb, DMA_CHAN_NOC_XFER,
			get_reg_value(reg_values, DMA_CHAN_NOC_XFER), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_ssp_config_register(pb, DMA_CHAN_SSP_CFG,
			get_reg_value(reg_values, DMA_CHAN_SSP_CFG), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_node_register(pb, DMA_CHAN_NODE,
			get_reg_value(reg_values, DMA_CHAN_NODE), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_mode_register(pb, DMA_CHAN_MODE_RO,
			get_reg_value(reg_values, DMA_CHAN_MODE_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_format_register(pb, DMA_CHAN_IMG_FORMAT_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_FORMAT_RO),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_size_register(pb, DMA_CHAN_IMG_SIZE_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_SIZE_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_position_register(pb, DMA_CHAN_IMG_POS_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_POS_RO),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_img_layout_register(pb, DMA_CHAN_IMG_LAYOUT_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_LAYOUT_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_bif_transfer_register(pb, DMA_CHAN_BIF_XFER_RO,
			get_reg_value(reg_values, DMA_CHAN_BIF_XFER_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_va_register(pb, DMA_CHAN_VA_RO,
			get_reg_value(reg_values, DMA_CHAN_VA_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_va_bdry_register(pb, DMA_CHAN_VA_BDRY_RO,
			get_reg_value(reg_values, DMA_CHAN_VA_BDRY_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_noc_transfer_register(pb, DMA_CHAN_NOC_XFER_RO,
			get_reg_value(reg_values, DMA_CHAN_NOC_XFER_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_ssp_config_register(pb, DMA_CHAN_SSP_CFG_RO,
			get_reg_value(reg_values, DMA_CHAN_SSP_CFG_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = dump_chan_node_register(pb, DMA_CHAN_NODE_RO,
			get_reg_value(reg_values, DMA_CHAN_NODE_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	return written;

err_exit:
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}

int paintbox_dump_dma_channel_stats(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;
	int written;

	written = snprintf(buf, len,
			"interrupts EOF %u VA %u IRQ activations %u\n",
			channel->stats.eof_interrupts,
			channel->stats.va_interrupts,
			channel->stats.irq_activations);

	written += snprintf(buf + written, len - written,
			"\ttransfers reported: completed %u discarded %u\n",
			channel->stats.reported_completions,
			channel->stats.reported_discards);

	written += snprintf(buf + written, len - written,
			"\tqueue counts: pending %u active %u completed %u discarded %u free %u\n",
			channel->pending_count, channel->active_count,
			channel->completed_count, pb->dma.discard_count,
			pb->dma.free_count);
	written += snprintf(buf + written, len - written,
			"\tstop request pending: %u\n", channel->stop_request);
	if (channel->stats.time_stats_enabled) {
		written += snprintf(buf + written, len - written,
				"\tlast transfer time %lldus\n",
				channel->stats.last_transfer_time_us);

		written += snprintf(buf + written, len - written,
				"\ttotal setup time %lldus\n",
				ktime_to_us(ktime_sub(
				channel->stats.setup_finish_time,
				channel->stats.setup_start_time)));
		written += snprintf(buf + written, len - written,
				"\tnon-dram setup time %lldus\n",
				ktime_to_us(ktime_sub(
				channel->stats.non_dram_setup_finish_time,
				channel->stats.non_dram_setup_start_time)));

		written += snprintf(buf + written, len - written,
				"\tdma buf map time %lldus\n",
				ktime_to_us(ktime_sub(
				channel->stats.dma_buf_map_finish_time,
				channel->stats.dma_buf_map_start_time)));

#ifdef CONFIG_PAINTBOX_IOMMU
		written += snprintf(buf + written, len - written,
				"\tiommu map time %lldus\n",
				ktime_to_us(ktime_sub(
				channel->stats.iommu_map_finish_time,
				channel->stats.iommu_map_start_time)));
#endif
	}

	return written;
}

static int paintbox_dma_channel_time_stats_enable_show(struct seq_file *s,
		void *p)
{
	struct paintbox_dma_channel *channel = s->private;

	seq_printf(s, "%u\n", channel->stats.time_stats_enabled);
	return 0;
}

static int paintbox_dma_channel_time_stats_enable_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, paintbox_dma_channel_time_stats_enable_show,
			inode->i_private);
}

static ssize_t paintbox_dma_channel_time_stats_enable_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_dma_channel *channel = s->private;
	struct paintbox_data *pb = channel->debug.pb;
	unsigned int val;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		channel->stats.time_stats_enabled = !!val;
		return count;
	}

	dev_err(pb->dev, "%s: invalid value, err = %d", __func__, ret);

	return ret;
}

static const struct file_operations dma_channel_time_stats_enable_fops = {
	.open = paintbox_dma_channel_time_stats_enable_open,
	.write = paintbox_dma_channel_time_stats_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int paintbox_dma_bif_oustanding_show(struct seq_file *s, void *p)
{
	struct paintbox_data *pb = s->private;

	seq_printf(s, "%u\n", pb->dma.bif_outstanding);
	return 0;
}

static int paintbox_dma_bif_outstanding_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, paintbox_dma_bif_oustanding_show,
			inode->i_private);
}

static ssize_t paintbox_dma_bif_outstanding_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_data *pb = s->private;
	unsigned int val;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		if (val < DMA_CHAN_BIF_XFER_OUTSTANDING_MIN ||
				val > DMA_CHAN_BIF_XFER_OUTSTANDING_MAX) {
			dev_err(pb->dev,
					"%s: invalid BIF outstanding value %u min %u max %u\n",
					__func__, val,
					DMA_CHAN_BIF_XFER_OUTSTANDING_MIN,
					DMA_CHAN_BIF_XFER_OUTSTANDING_MAX);
			return -ERANGE;
		}

		pb->dma.bif_outstanding = val;
		return count;
	}

	dev_err(pb->dev, "%s: invalid value, err = %d", __func__, ret);

	return ret;
}

static const struct file_operations dma_bif_outstanding_fops = {
	.open = paintbox_dma_bif_outstanding_open,
	.write = paintbox_dma_bif_outstanding_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

void paintbox_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	paintbox_debug_create_entry(pb, &channel->debug,
			pb->dma.debug.debug_dir, "channel", channel->channel_id,
			paintbox_dump_dma_channel_registers,
			paintbox_dump_dma_channel_stats,
			channel);

	paintbox_debug_create_reg_entries(pb, &channel->debug,
			&dma_grp_reg_names[REG_INDEX(DMA_CHAN_CTRL)],
			DMA_GRP_NUM_REGS, paintbox_dma_channel_reg_entry_write,
			paintbox_dma_channel_reg_entry_read);

	channel->time_stats_enable_dentry = debugfs_create_file(
			"time_stats_enable", 0640, channel->debug.debug_dir,
			channel, &dma_channel_time_stats_enable_fops);
	if (IS_ERR(channel->time_stats_enable_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(channel->time_stats_enable_dentry));
		return;
	}
}

void paintbox_dma_channel_debug_remove(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	debugfs_remove(channel->time_stats_enable_dentry);
	paintbox_debug_free_reg_entries(&channel->debug);
	paintbox_debug_free_entry(&channel->debug);
}

void paintbox_dma_debug_init(struct paintbox_data *pb)
{
	unsigned int i, reg_index;
	size_t reg_count = DMA_TOP_NUM_REGS;

	int ret;

	paintbox_debug_create_entry(pb, &pb->dma.debug, pb->debug_root,
			"dma", -1, paintbox_dump_dma_registers, NULL, &pb->dma);

	ret = paintbox_debug_alloc_reg_entries(pb, &pb->dma.debug, reg_count);

	pb->dma.bif_outstanding_dentry = debugfs_create_file("bif_outstanding",
			0640, pb->dma.debug.debug_dir, pb,
			&dma_bif_outstanding_fops);
	if (IS_ERR(pb->dma.bif_outstanding_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(pb->dma.bif_outstanding_dentry));
		return;
	}

	for (i = 0, reg_index = 0; i < DMA_TOP_NUM_REGS &&
			reg_index < REG_INDEX(DMA_TOP_BLOCK_LEN);
			reg_index++, i++) {
		if (!dma_top_reg_names[i])
			continue;

		ret = paintbox_debug_create_reg_entry(pb, &pb->dma.debug, i,
				dma_top_reg_names[reg_index],
				reg_index * IPU_REG_WIDTH,
				paintbox_dma_reg_entry_write,
				paintbox_dma_reg_entry_read);
		if (ret < 0) {
			paintbox_debug_free_reg_entries(&pb->dma.debug);
			return;
		}
	}

	for (reg_index = REG_INDEX(DMA_CHAN_CTRL); i < reg_count &&
			reg_index < REG_INDEX(DMA_CHAN_NODE_RO); reg_index++,
			i++) {
		if (!dma_grp_reg_names[reg_index])
			continue;

		ret = paintbox_debug_create_reg_entry(pb, &pb->dma.debug,
				i, dma_grp_reg_names[reg_index],
				reg_index * IPU_REG_WIDTH,
				paintbox_dma_channel_reg_entry_write,
				paintbox_dma_channel_reg_entry_read);
		if (ret < 0) {
			paintbox_debug_free_reg_entries(&pb->dma.debug);
			return;
		}
	}
}

void paintbox_dma_debug_remove(struct paintbox_data *pb)
{
	debugfs_remove(pb->dma.bif_outstanding_dentry);
	paintbox_debug_free_reg_entries(&pb->dma.debug);
	paintbox_debug_free_entry(&pb->dma.debug);
}
#endif
