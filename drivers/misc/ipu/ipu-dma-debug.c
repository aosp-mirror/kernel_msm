/*
 * DMA debug support for the Paintbox programmable IPU
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

#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-dma.h"
#include "ipu-dma-debug.h"
#include "ipu-regs.h"

static const char *ipu_dma_top_reg_names[DMA_TOP_NUM_REGS] = {
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
	REG_NAME_ENTRY(SSP_STATUS),
	REG_NAME_ENTRY(DMA_SPARE)
};

static const char *ipu_dma_grp_reg_names[DMA_GRP_NUM_REGS] = {
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

/* The caller to this function must hold pb->lock */
static inline void ipu_dma_select_channel(struct paintbox_data *pb,
		unsigned int channel_id)
{
	ipu_writel(pb->dev, channel_id & (DMA_SEL_GRP_SEL_MASK |
			DMA_SEL_CHAN_SEL_MASK),
			IPU_CSR_DMA_GRP_OFFSET + DMA_SEL);
}

/* The caller to this function must hold pb->lock */
static uint64_t ipu_dma_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	return ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
			reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static void ipu_dma_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_data *pb = debug->pb;

	ipu_writeq(pb->dev, val, IPU_CSR_DMA_TOP_OFFSET +
			reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static uint64_t ipu_dma_channel_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;

	ipu_dma_select_channel(pb, channel->channel_id);

	return ipu_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			reg_entry->reg_offset);
}

/* The caller to this function must hold pb->lock */
static void ipu_dma_channel_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;

	ipu_dma_select_channel(pb, channel->channel_id);

	ipu_writeq(pb->dev, val, IPU_CSR_DMA_GRP_OFFSET +
			reg_entry->reg_offset);
}

static inline int ipu_dma_dump_top_reg(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len)
{
	const char *reg_name = ipu_dma_top_reg_names[REG_INDEX(reg_offset)];

	return ipu_debug_dump_register_with_value(pb, IPU_CSR_DMA_TOP_OFFSET,
			reg_offset, reg_value, reg_name, buf, written, len);
}

static inline int ipu_dma_dump_grp_reg(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len)
{
	const char *reg_name = ipu_dma_grp_reg_names[REG_INDEX(reg_offset)];

	return ipu_debug_dump_register_with_value(pb, IPU_CSR_DMA_GRP_OFFSET,
			reg_offset, reg_value, reg_name, buf, written, len);
}

static int ipu_dma_dump_top_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = ipu_dma_dump_top_reg(pb, reg_offset, reg_value, buf, written,
			len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = ipu_debug_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static int ipu_dma_dump_grp_reg_verbose(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t reg_value, char *buf,
		int *written, size_t len, const char *format, ...)
{
	va_list args;
	int ret;

	ret = ipu_dma_dump_grp_reg(pb, reg_offset, reg_value, buf, written,
			len);
	if (ret < 0)
		return ret;

	va_start(args, format);

	ret = ipu_debug_vprintf(pb, buf, written, len, format, args);

	va_end(args);

	return ret;
}

static inline const char *ipu_dma_swizzle_to_str(uint64_t val)
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

static int ipu_dma_dump_stat_control_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_top_reg_verbose(pb, reg_offset, val, buf, written,
			len,
			"\tNOC_MNGR_PORT %d DST_BIF_SEL_SSP %d ADDR_MODE %d ENABLE %d\n",
			!!(val & DMA_STAT_CTRL_NOC_MNGR_PORT_MASK),
			!!(val & DMA_STAT_CTRL_DST_BIF_SEL_SSP_MASK),
			!!(val & DMA_STAT_CTRL_ADDR_MODE_MASK),
			!!(val & DMA_STAT_CTRL_ENABLE_MASK));
}

static const char *ipu_dma_state_src_mode_to_str(uint64_t dma_stat_state_val,
		uint32_t chan_mode_val)
{
	uint32_t src;

	src = (chan_mode_val & DMA_CHAN_MODE_SRC_MASK) >>
			DMA_CHAN_MODE_SRC_SHIFT;

	if (src == DMA_CHAN_MODE_SRC_DRAM) {
		switch (dma_stat_state_val & DMA_STAT_STATE_SRC_MASK) {
		case 0:
			return "Idle";
		case 1:
			return "Get Gather Coord";
		case 3:
			return "Wait Segment Grant";
		case 4:
			return "Add Calc Delay";
		case 5:
			return "Write Ctl";
		case 6:
			return "Request Data From BIF";
		case 7:
			return "Reset State";
		default:
			return "UNKNOWN";
		};
	} else if (src == DMA_CHAN_MODE_SRC_LBP) {
		switch (dma_stat_state_val & DMA_STAT_STATE_SRC_MASK) {
		case 0:
			return "Cmd Idle";
		case 1:
			return "Request Segments";
		case 2:
			return "Wait Segment Grant";
		case 3:
			return "Cmd";
		case 4:
			return "Cmd Wait";
		case 5:
			return "Retry Cmd";
		case 6:
			return "Ack Wait";
		case 7:
			return "Check Avail";
		default:
			return "UNKNOWN";
		};
	} else {
		return "UNKNOWN";
	}
}

static const char *ipu_dma_state_dst_mode_to_str(uint64_t dma_stat_state_val,
		uint64_t chan_mode_val)
{
	uint32_t dst;

	dst = (chan_mode_val & DMA_CHAN_MODE_DST_MASK) >>
			DMA_CHAN_MODE_DST_SHIFT;

	if (dst == DMA_CHAN_MODE_DST_DRAM) {
		switch ((dma_stat_state_val & DMA_STAT_STATE_DST_MASK) >>
				DMA_STAT_STATE_DST_SHIFT) {
		case 0:
			return "Idle";
		case 1:
			return "Wait Segment";
		case 2:
			return "SSP Read Reqs";
		case 3:
			return "Load Address 0";
		case 4:
			return "Load Address 1";
		case 5:
			return "Load Address 2";
		case 6:
			return "Load Ptrs";
		case 7:
			return "DRAM Write Req";
		case 8:
			return "Wait Ptr";
		case 9:
			return "DRAM Pull Wait";
		case 10:
			return "Check If Last";
		case 11:
			return "DRAM Write Ack Wait";
		default:
			return "UNKNOWN";
		};
	} else if (dst == DMA_CHAN_MODE_DST_LBP) {
		switch ((dma_stat_state_val & DMA_STAT_STATE_DST_MASK) >>
				DMA_STAT_STATE_DST_SHIFT) {
		case 0:
			return "Cmd Idle";
		case 1:
			return "Cmd";
		case 2:
			return "Ack Wait";
		case 3:
			return "Retry Cmd";
		case 4:
			return "Load Ptrs";
		case 5:
			return "Cmd EOF";
		case 6:
			return "Check Avail";
		default:
			return "UNKNOWN";
		};
	} else {
		return "UNKNOWN";
	}
}

static int ipu_dma_dump_dma_stat_state_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, uint32_t mode_val, char *buf,
		int *written, size_t len)
{
	int ret;

	ret = ipu_dma_dump_top_reg(pb, reg_offset, val, buf, written, len);
	if (ret < 0)
		return ret;

	ret = ipu_debug_printf(pb, buf, written, len,
			"\tWR_MNGR: CHAN %u MODE %d STATE %d\n",
			(val & DMA_STAT_STATE_WR_MNGR_CHAN_MASK) >>
			DMA_STAT_STATE_WR_MNGR_CHAN_SHIFT,
			!!(val & DMA_STAT_STATE_WR_MNGR_STATE_MODE_MASK),
			(val & DMA_STAT_STATE_WR_MNGR_STATE_MASK) >>
			DMA_STAT_STATE_WR_MNGR_STATE_SHIFT);
	if (ret < 0)
		return ret;

	ret = ipu_debug_printf(pb, buf, written, len,
			"\tRD_MNGR: CHAN %u MODE %d\n",
			(val & DMA_STAT_STATE_RD_MNGR_CHAN_MASK) >>
			DMA_STAT_STATE_RD_MNGR_CHAN_SHIFT,
			!!(val & DMA_STAT_STATE_RD_MNGR_CHAN_MODE_MASK));
	if (ret < 0)
		return ret;

	return ipu_debug_printf(pb, buf, written, len,
			"\tDST_MODE %d DST %u (%s) SRC_MODE %d SRC %u (%s)\n",
			!!(val & DMA_STAT_STATE_DST_MODE_MASK),
			(val & DMA_STAT_STATE_DST_MASK) >>
			DMA_STAT_STATE_DST_SHIFT,
			ipu_dma_state_dst_mode_to_str(val, mode_val),
			!!(val & DMA_STAT_STATE_SRC_MODE_MASK),
			val & DMA_STAT_STATE_SRC_MASK,
			ipu_dma_state_src_mode_to_str(val, mode_val));
}

static int ipu_dma_dump_dma_stat_ptr_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_top_reg_verbose(pb, reg_offset, val, buf, written,
			len,
			"\tPTR_MODE %d SHEET_HEIGHT %u SHEET WIDTH %u Y %u X %u\n",
			!!(val & DMA_STAT_PTR_MODE_MASK),
			(val & DMA_STAT_PTR_SHEET_HEIGHT_MASK) >>
			DMA_STAT_PTR_SHEET_HEIGHT_SHIFT,
			(val & DMA_STAT_PTR_SHEET_WIDTH_MASK) >>
			DMA_STAT_PTR_SHEET_WIDTH_SHIFT,
			(val & DMA_STAT_PTR_Y_MASK) >> DMA_STAT_PTR_Y_SHIFT,
			val & DMA_STAT_PTR_X_MASK);
}

static int ipu_dma_dump_dma_stat_address_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_top_reg_verbose(pb, reg_offset, val, buf, written,
			len, "\tADDR 0x%016llx\n", val);
}

static int ipu_dma_dump_dma_ssp_status_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_top_reg_verbose(pb, reg_offset, val, buf, written,
			len, "\tSEG_AVAIL %u\n", val);
}

/* The caller to this function must hold pb->lock */
int ipu_dma_dump_registers(struct paintbox_debug *debug, char *buf, size_t len)
{
	uint64_t dma_top_registers[DMA_TOP_NUM_REGS];
	struct paintbox_data *pb = debug->pb;
	uint64_t val;
	uint32_t dma_sel_val;
	unsigned int i, reg_offset;
	int ret, written = 0;

	dma_sel_val = ipu_readl(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_SEL);

	for (reg_offset = DMA_CTRL; reg_offset < DMA_TOP_BLOCK_END;
			reg_offset += IPU_REG_WIDTH) {
		if (!ipu_dma_top_reg_names[REG_INDEX(reg_offset)])
			continue;

		dma_top_registers[REG_INDEX(reg_offset)] =
				ipu_readq(pb->dev,
				IPU_CSR_DMA_TOP_OFFSET + reg_offset);
	}

	ret = ipu_dma_dump_grp_reg_verbose(pb, DMA_SEL, dma_sel_val, buf,
			&written, len, "\tGRP_SEL %u CHAN_SEL %u\n",
			(dma_sel_val & DMA_SEL_GRP_SEL_MASK) >>
			DMA_SEL_GRP_SEL_SHIFT,
			dma_sel_val & DMA_SEL_CHAN_SEL_MASK);
	if (ret < 0)
		goto err_exit;

	val = dma_top_registers[REG_INDEX(DMA_CTRL)];
	ret = ipu_dma_dump_top_reg_verbose(pb, DMA_CTRL, val, buf, &written,
			len, "\tAXI_SWIZZLE %s RESET %d\n",
			ipu_dma_swizzle_to_str(val),
			val & DMA_CTRL_DMA_RESET_MASK);
	if (ret < 0)
		goto err_exit;

	for (i = REG_INDEX(DMA_IRQ_ISR); i <= REG_INDEX(DMA_PMON_CNT_3_STS);
			i++) {
		if (ipu_dma_top_reg_names[i] != NULL) {
			val = dma_top_registers[i];
			ret = ipu_dma_dump_top_reg(pb, i * IPU_REG_WIDTH, val,
					buf, &written, len);
			if (ret < 0)
				goto err_exit;
		}
	}

	val = dma_top_registers[REG_INDEX(DMA_STAT_CTRL)];
	ret = ipu_dma_dump_stat_control_register(pb, DMA_STAT_CTRL, val,
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	val = dma_top_registers[REG_INDEX(DMA_STAT_STATE)];
	ret = ipu_dma_dump_dma_stat_state_register(pb, DMA_STAT_STATE, val,
			ipu_readq(pb->dev, IPU_CSR_DMA_GRP_OFFSET +
			DMA_CHAN_MODE_RO), buf, &written, len);
	if (ret < 0)
		goto err_exit;

	val = dma_top_registers[REG_INDEX(DMA_STAT_PTR)];
	ret = ipu_dma_dump_dma_stat_ptr_register(pb, DMA_STAT_PTR, val,
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	val = dma_top_registers[REG_INDEX(DMA_STAT_ADDR)];
	ret = ipu_dma_dump_dma_stat_address_register(pb, DMA_STAT_ADDR, val,
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	val = dma_top_registers[REG_INDEX(SSP_STATUS)];
	ret = ipu_dma_dump_dma_ssp_status_register(pb, SSP_STATUS, val,
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	return written;

err_exit:
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}

static inline unsigned int ipu_dma_get_bit_depth(uint64_t val)
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

static inline const char *ipu_dma_src_to_str(uint64_t val)
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

static inline const char *ipu_dma_dst_to_str(uint64_t val)
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

static int ipu_dma_dump_chan_mode_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len,
			"\tGATHER %u ADDR_MODE %s DST %s SRC %s ENA %u\n",
			!!(val & DMA_CHAN_MODE_GATHER_MASK),
			val & DMA_CHAN_MODE_ADDR_MODE_MASK ? "PHYSICAL" :
			"ABSTRACT",
			ipu_dma_dst_to_str(val), ipu_dma_src_to_str(val),
			!!(val & DMA_CHAN_MODE_CHAN_ENA_MASK));
}

static int ipu_dma_dump_chan_img_format_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len,
			"\tBLOCK_4x4 %u MIPI_RAW_FORMAT %d BIT DEPTH %u PLANES %u COMPONENTS %u\n",
			!!(val & DMA_CHAN_IMG_FORMAT_BLOCK_4X4_MASK),
			!!(val & DMA_CHAN_IMG_FORMAT_MIPI_RAW_FORMAT_MASK),
			ipu_dma_get_bit_depth(val),
			((val & DMA_CHAN_IMG_FORMAT_PLANES_MASK) >>
					DMA_CHAN_IMG_FORMAT_PLANES_SHIFT) + 1,
			(val & DMA_CHAN_IMG_FORMAT_COMPONENTS_MASK) + 1);
}

static int ipu_dma_dump_chan_img_size_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len, "\tIMG_HEIGHT %u IMG_WIDTH %u\n",
			(val & DMA_CHAN_IMG_SIZE_IMG_HEIGHT_MASK) >>
					DMA_CHAN_IMG_SIZE_IMG_HEIGHT_SHIFT,
			val & DMA_CHAN_IMG_SIZE_IMG_WIDTH_MASK);
}

static int ipu_dma_dump_chan_img_position_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len,
			"\tLB_START_Y %d LB_START_X %d START_Y %d START_X %d\n",
			(int16_t)((val & DMA_CHAN_IMG_POS_LB_START_Y_MASK) >>
					DMA_CHAN_IMG_POS_LB_START_Y_SHIFT),
			(int16_t)((val & DMA_CHAN_IMG_POS_LB_START_X_MASK) >>
					DMA_CHAN_IMG_POS_LB_START_X_SHIFT),
			(int16_t)((val & DMA_CHAN_IMG_POS_START_Y_MASK) >>
					DMA_CHAN_IMG_POS_START_Y_SHIFT),
			(int16_t)(val & DMA_CHAN_IMG_POS_START_X_MASK));
}

static int ipu_dma_dump_chan_img_layout_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len, "\tPLANE_STRIDE %llu ROW_STRIDE %u\n",
			(val & DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_MASK) >>
			DMA_CHAN_IMG_LAYOUT_PLANE_STRIDE_SHIFT,
			(val & DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_MASK) >>
			DMA_CHAN_IMG_LAYOUT_ROW_STRIDE_SHIFT);
}

static int ipu_dma_dump_chan_bif_transfer_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len, "\tOUTSTANDING %u STRIPE_HEIGHT %u\n",
			(val & DMA_CHAN_BIF_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT,
			val & DMA_CHAN_BIF_XFER_STRIPE_HEIGHT_MASK);
}

static int ipu_dma_dump_chan_va_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len, "\tVA 0x%016llx\n", val & DMA_CHAN_VA_BASE_MASK);
}

static int ipu_dma_dump_chan_va_bdry_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len, "\tVA BDRY %llu\n", val &
			DMA_CHAN_VA_BDRY_LEN_MASK);
}

static int ipu_dma_dump_chan_noc_transfer_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len,
			"\tRETRY_INTERVAL %u OUTSTANDING %u SHEET_HEIGHT %u SHEET_WIDTH %u\n",
			(val & DMA_CHAN_NOC_XFER_RETRY_INTERVAL_MASK) >>
			DMA_CHAN_NOC_XFER_RETRY_INTERVAL_SHIFT,
			(val & DMA_CHAN_NOC_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_NOC_XFER_OUTSTANDING_SHIFT,
			(val & DMA_CHAN_NOC_XFER_SHEET_HEIGHT_MASK) >>
			DMA_CHAN_NOC_XFER_SHEET_HEIGHT_SHIFT,
			val & DMA_CHAN_NOC_XFER_SHEET_WIDTH_MASK);
}


static int ipu_dma_dump_chan_ssp_config_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len,
			"\tMULT_SEGS %d MULT_SHEETS %d PTRS_FIFO_DEPTH %u LOCS_PER_ROW %u PIX_PER_LOC %u\n",
			!!(val & DMA_CHAN_SSP_CFG_MULT_SEGS_MASK),
			!!(val & DMA_CHAN_SSP_CFG_MULT_SHEETS_MASK),
			(val & DMA_CHAN_SSP_CFG_PTRS_FIFO_DEPTH_MASK) >>
			DMA_CHAN_SSP_CFG_PTRS_FIFO_DEPTH_SHIFT,
			(val & DMA_CHAN_SSP_CFG_LOCS_PER_ROW_MASK) >>
			DMA_CHAN_SSP_CFG_LOCS_PER_ROW_SHIFT,
			val & DMA_CHAN_SSP_CFG_PIX_PER_LOC_MASK);
}

static int ipu_dma_dump_chan_node_register(struct paintbox_data *pb,
		uint32_t reg_offset, uint64_t val, char *buf, int *written,
		size_t len)
{
	return ipu_dma_dump_grp_reg_verbose(pb, reg_offset, val, buf, written,
			len,
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

/* The caller to this function must hold pb->lock */
int ipu_dma_dump_channel_registers(struct paintbox_debug *debug, char *buf,
			size_t len)
{
	uint64_t reg_values[DMA_GRP_NUM_REGS];
	struct paintbox_dma_channel *channel = container_of(debug,
			struct paintbox_dma_channel, debug);
	struct paintbox_data *pb = debug->pb;
	unsigned int reg_offset;
	int ret, written = 0;
	uint64_t val;

	ipu_dma_select_channel(pb, channel->channel_id);

	for (reg_offset = 0; reg_offset < DMA_GRP_BLOCK_END; reg_offset +=
			IPU_REG_WIDTH) {
		if (!ipu_dma_grp_reg_names[REG_INDEX(reg_offset)])
			continue;

		reg_values[REG_INDEX(reg_offset)] = ipu_readq(pb->dev,
				IPU_CSR_DMA_GRP_OFFSET + reg_offset);
	}

	val = get_reg_value(reg_values, DMA_CHAN_CTRL);
	ret = ipu_dma_dump_grp_reg_verbose(pb, DMA_CHAN_CTRL, val, buf,
			&written, len,
			"\tSTOP %d CONTINUOUS %d CHAN_RESET %d\n",
			(val & DMA_CHAN_CTRL_STOP_MASK) >>
					DMA_CHAN_CTRL_STOP_SHIFT,
			(val & DMA_CHAN_CTRL_CONTINUOUS_MASK) >>
					DMA_CHAN_CTRL_CONTINUOUS_SHIFT,
			val & DMA_CHAN_CTRL_CHAN_RESET_MASK);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_mode_register(pb, DMA_CHAN_MODE,
			get_reg_value(reg_values, DMA_CHAN_MODE), buf, &written,
			len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_format_register(pb, DMA_CHAN_IMG_FORMAT,
			get_reg_value(reg_values, DMA_CHAN_IMG_FORMAT),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_size_register(pb, DMA_CHAN_IMG_SIZE,
			get_reg_value(reg_values, DMA_CHAN_IMG_SIZE), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_position_register(pb, DMA_CHAN_IMG_POS,
			get_reg_value(reg_values, DMA_CHAN_IMG_POS),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_layout_register(pb, DMA_CHAN_IMG_LAYOUT,
			get_reg_value(reg_values, DMA_CHAN_IMG_LAYOUT), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_bif_transfer_register(pb, DMA_CHAN_BIF_XFER,
			get_reg_value(reg_values, DMA_CHAN_BIF_XFER), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_va_register(pb, DMA_CHAN_VA,
			get_reg_value(reg_values, DMA_CHAN_VA), buf, &written,
			len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_va_bdry_register(pb, DMA_CHAN_VA_BDRY,
			get_reg_value(reg_values, DMA_CHAN_VA_BDRY), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_noc_transfer_register(pb, DMA_CHAN_NOC_XFER,
			get_reg_value(reg_values, DMA_CHAN_NOC_XFER), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_ssp_config_register(pb, DMA_CHAN_SSP_CFG,
			get_reg_value(reg_values, DMA_CHAN_SSP_CFG), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_node_register(pb, DMA_CHAN_NODE,
			get_reg_value(reg_values, DMA_CHAN_NODE), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_mode_register(pb, DMA_CHAN_MODE_RO,
			get_reg_value(reg_values, DMA_CHAN_MODE_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_format_register(pb, DMA_CHAN_IMG_FORMAT_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_FORMAT_RO),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_size_register(pb, DMA_CHAN_IMG_SIZE_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_SIZE_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_position_register(pb, DMA_CHAN_IMG_POS_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_POS_RO),
			buf, &written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_img_layout_register(pb, DMA_CHAN_IMG_LAYOUT_RO,
			get_reg_value(reg_values, DMA_CHAN_IMG_LAYOUT_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_bif_transfer_register(pb, DMA_CHAN_BIF_XFER_RO,
			get_reg_value(reg_values, DMA_CHAN_BIF_XFER_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_va_register(pb, DMA_CHAN_VA_RO,
			get_reg_value(reg_values, DMA_CHAN_VA_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_va_bdry_register(pb, DMA_CHAN_VA_BDRY_RO,
			get_reg_value(reg_values, DMA_CHAN_VA_BDRY_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_noc_transfer_register(pb, DMA_CHAN_NOC_XFER_RO,
			get_reg_value(reg_values, DMA_CHAN_NOC_XFER_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_ssp_config_register(pb, DMA_CHAN_SSP_CFG_RO,
			get_reg_value(reg_values, DMA_CHAN_SSP_CFG_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	ret = ipu_dma_dump_chan_node_register(pb, DMA_CHAN_NODE_RO,
			get_reg_value(reg_values, DMA_CHAN_NODE_RO), buf,
			&written, len);
	if (ret < 0)
		goto err_exit;

	if (pb->dma.debug_enabled) {
		ret = ipu_dma_dump_stat_control_register(pb, DMA_STAT_CTRL,
				ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
				DMA_STAT_CTRL), buf, &written, len);
		if (ret < 0)
			goto err_exit;

		ret = ipu_dma_dump_dma_stat_state_register(pb, DMA_STAT_STATE,
				ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
				DMA_STAT_STATE), get_reg_value(reg_values,
				DMA_CHAN_MODE_RO), buf, &written, len);
		if (ret < 0)
			goto err_exit;

		ret = ipu_dma_dump_dma_stat_ptr_register(pb, DMA_STAT_PTR,
				ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
				DMA_STAT_PTR), buf, &written, len);
		if (ret < 0)
			goto err_exit;

		ret = ipu_dma_dump_dma_stat_address_register(pb, DMA_STAT_ADDR,
				ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
				DMA_STAT_ADDR), buf, &written, len);
		if (ret < 0)
			goto err_exit;

		ret = ipu_dma_dump_dma_ssp_status_register(pb, SSP_STATUS,
				ipu_readq(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
				SSP_STATUS), buf, &written, len);
		if (ret < 0)
			goto err_exit;
	}


	return written;

err_exit:
	dev_err(pb->dev, "%s: register dump error, err = %d", __func__, ret);
	return ret;
}

static int ipu_dma_debug_enable_show(struct seq_file *s, void *p)
{
	struct paintbox_data *pb = s->private;

	seq_printf(s, "%u\n", pb->dma.debug_enabled);
	return 0;
}

static int ipu_dma_debug_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, ipu_dma_debug_enable_show,
			inode->i_private);
}

static ssize_t ipu_dma_debug_enable_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct paintbox_data *pb = s->private;
	unsigned int val;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 0, &val);
	if (ret == 0) {
		uint32_t reg_val;

		reg_val = ipu_readl(pb->dev, IPU_CSR_DMA_TOP_OFFSET +
					DMA_STAT_CTRL);
		if (val != 0)
			reg_val |= DMA_STAT_CTRL_ENABLE_MASK;
		else
			reg_val &= ~DMA_STAT_CTRL_ENABLE_MASK;

		ipu_writel(pb->dev, reg_val, IPU_CSR_DMA_TOP_OFFSET +
				DMA_STAT_CTRL);

		pb->dma.debug_enabled = !!val;

		return count;
	}

	dev_err(pb->dev, "%s: invalid value, err = %d", __func__, ret);

	return ret;
}

static const struct file_operations ipu_dma_debug_enable_fops = {
	.open = ipu_dma_debug_enable_open,
	.write = ipu_dma_debug_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

void ipu_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	ipu_debug_create_entry(pb, &channel->debug, pb->dma.debug.debug_dir,
			"channel", channel->channel_id,
			ipu_dma_dump_channel_registers, NULL, channel);

	ipu_debug_create_reg_entries(pb, &channel->debug,
			&ipu_dma_grp_reg_names[REG_INDEX(DMA_CHAN_CTRL)],
			DMA_GRP_NUM_REGS, ipu_dma_channel_reg_entry_write,
			ipu_dma_channel_reg_entry_read);
}

void ipu_dma_channel_debug_remove(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{

	ipu_debug_free_reg_entries(&channel->debug);
	ipu_debug_free_entry(&channel->debug);
}

void ipu_dma_debug_init(struct paintbox_data *pb)
{
	unsigned int i, reg_index;
	size_t reg_count = DMA_TOP_NUM_REGS;

	int ret;

	ipu_debug_create_entry(pb, &pb->dma.debug, pb->debug_root, "dma", -1,
			ipu_dma_dump_registers, NULL, &pb->dma);

	ret = ipu_debug_alloc_reg_entries(pb, &pb->dma.debug, reg_count);

	pb->dma.debug_enable_dentry = debugfs_create_file("debug_en", 0640,
			pb->dma.debug.debug_dir, pb,
			&ipu_dma_debug_enable_fops);
	if (IS_ERR(pb->dma.debug_enable_dentry)) {
		dev_err(pb->dev, "%s: err = %ld", __func__,
				PTR_ERR(pb->dma.debug_enable_dentry));
		return;
	}

	for (i = 0, reg_index = 0; i < DMA_TOP_NUM_REGS &&
			reg_index < REG_INDEX(DMA_TOP_BLOCK_LEN);
			reg_index++, i++) {
		if (!ipu_dma_top_reg_names[i])
			continue;

		ret = ipu_debug_create_reg_entry(pb, &pb->dma.debug, i,
				ipu_dma_top_reg_names[reg_index],
				reg_index * IPU_REG_WIDTH,
				ipu_dma_reg_entry_write,
				ipu_dma_reg_entry_read);
		if (ret < 0) {
			ipu_debug_free_reg_entries(&pb->dma.debug);
			return;
		}
	}

	for (reg_index = REG_INDEX(DMA_CHAN_CTRL); i < reg_count &&
			reg_index < REG_INDEX(DMA_CHAN_NODE_RO); reg_index++,
			i++) {
		if (!ipu_dma_grp_reg_names[reg_index])
			continue;

		ret = ipu_debug_create_reg_entry(pb, &pb->dma.debug, i,
				ipu_dma_grp_reg_names[reg_index], reg_index *
				IPU_REG_WIDTH, ipu_dma_channel_reg_entry_write,
				ipu_dma_channel_reg_entry_read);
		if (ret < 0) {
			ipu_debug_free_reg_entries(&pb->dma.debug);
			return;
		}
	}
}

void ipu_dma_debug_remove(struct paintbox_data *pb)
{
	ipu_debug_free_reg_entries(&pb->dma.debug);
	ipu_debug_free_entry(&pb->dma.debug);
	debugfs_remove(pb->dma.debug_enable_dentry);
}
