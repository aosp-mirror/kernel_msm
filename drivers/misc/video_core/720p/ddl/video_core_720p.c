/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/unistd.h>

#include "video_core_type.h"
#include "video_core_720p.h"

#define VIDC_720P_VERSION_STRING "VIDC_V1.0"

unsigned long vid_c_base_addr;

void vidc_720p_set_device_virtual_base(void *virt_addr)
{
	vid_c_base_addr = (unsigned long)virt_addr;
}

static inline void vidc_720p_write(unsigned long offset, unsigned long val)
{
	pr_debug("REG 0x%08lx: write 0x%08lx\n", offset, val);
	mb();
	iowrite32(val, vid_c_base_addr + offset);
}

static inline unsigned long vidc_720p_read(unsigned long offset)
{
	unsigned long val;
	mb();
	val = ioread32(vid_c_base_addr + offset);
	pr_debug("REG 0x%08lx: read 0x%08lx\n", offset, val);
	return val;
}

void vidc_720p_init(char **ppsz_version, size_t sz, phys_addr_t phys_addr,
	enum vidc_720p_endian_type dma_endian, u32 interrupt_off,
	enum vidc_720p_interrupt_level_selection_type interrupt_sel,
	u32 interrupt_mask)
{
	if (ppsz_version)
		*ppsz_version = VIDC_720P_VERSION_STRING;

	if (interrupt_sel == VIDC_720P_INTERRUPT_LEVEL_SEL)
		vidc_720p_write(0x0504, 0);
	else
		vidc_720p_write(0x0504, 1);

	if (interrupt_off)
		vidc_720p_write(0x0500, 1);
	else
		vidc_720p_write(0x0500, 0);

	vidc_720p_write(0x0508, 1);

	vidc_720p_write(0x0518, 0);

	vidc_720p_write(0x0518, interrupt_mask);

	vidc_720p_write(0x0044, dma_endian);

	vidc_720p_write(0x0138, 0);

	vidc_720p_write(0x0110, 1);

	vidc_720p_write(0x000C, sz / 4);  /* word size */

	vidc_720p_write(0x0014, phys_addr);

	vidc_720p_write(0x0020, 0);

	vidc_720p_write(0x0000, 1);
}

u32 vidc_720p_do_sw_reset(void)
{
	u32 fw_start;
	udelay(5);
	vidc_720p_write(0x0108, 0);
	udelay(5);
	vidc_720p_write(0x0134, 0);
	udelay(5);
	vidc_720p_write(0x0130, 1);
	udelay(15);
	vidc_720p_write(0x0130, 0);
	udelay(5);
	fw_start = vidc_720p_read(0x0134);

	if (!fw_start) {
		pr_debug("VIDC-SW-RESET-FAILS!\n");
		return false;
	}
	return true;
}

u32 vidc_720p_reset_is_success()
{
	u32 stagecounter;
	stagecounter = vidc_720p_read(0x0414);
	stagecounter &= 0xff;
	if (stagecounter != 0xe5) {
		pr_debug("VIDC-CPU_RESET-FAILS!\n");
		vidc_720p_write(0x0108, 0);
		msleep(10);
		return false;
	}
	return true;
}

void vidc_720p_start_cpu(enum vidc_720p_endian_type dma_endian,
	phys_addr_t icontext_bufferstart, phys_addr_t debug_core_dump_addr,
	size_t debug_buffer_size)
{
	u32 dbg_info_input0_reg = 0x1;

	vidc_720p_write(0x0110, 0);
	vidc_720p_write(0x0230, icontext_bufferstart);
	vidc_720p_write(0x0044, dma_endian);
	if (debug_buffer_size) {
		dbg_info_input0_reg = (debug_buffer_size << 0x10)
			| (0x2 << 1) | 0x1;
		vidc_720p_write(0x0D10, debug_core_dump_addr);
	}
	vidc_720p_write(0x0D0C, dbg_info_input0_reg);
	vidc_720p_write(0x0108, 1);
}

u32 vidc_720p_cpu_start()
{
	u32 fw_status;

	fw_status = vidc_720p_read(0x0C14);
	if (fw_status != 0x02)
		return false;
	return true;
}


void vidc_720p_stop_fw(void)
{
	vidc_720p_write(0x0134, 0);
	vidc_720p_write(0x0108, 0);
}

void vidc_720p_get_interrupt_status(u32 *interrupt_status,
		u32 *cmd_err_status, u32 *disp_pic_err_status, u32 *op_failed)
{
	u32 err_status;

	*interrupt_status = vidc_720p_read(0x0514);
	err_status = vidc_720p_read(0x0E9C);
	*cmd_err_status = err_status & 0xffff;
	*disp_pic_err_status = (err_status & 0xffff0000) >> 16;
	*op_failed = (vidc_720p_read(0x0EC0) & 0x2) >> 1;
}

void vidc_720p_interrupt_done_clear(void)
{
	vidc_720p_write(0x0508, 1);
	vidc_720p_write(0x0104, 4);
}

void vidc_720p_submit_command(u32 ch_id, u32 cmd_id)
{
	u32 fw_status;

	vidc_720p_write(0x0104, ch_id);
	vidc_720p_write(0x0D00, cmd_id);

	fw_status = vidc_720p_read(0x0C14);
	vidc_720p_write(0x0D1C, fw_status);
}

u32 vidc_720p_engine_reset(u32 ch_id, enum vidc_720p_endian_type dma_endian,
	enum vidc_720p_interrupt_level_selection_type interrupt_sel,
	u32 interrupt_mask)
{
	u32 op_done;
	u32 counter = 0;

	pr_debug("ENG-RESET!!\n");
	/* issue the engine reset command */
	vidc_720p_submit_command(ch_id, VIDC_720P_CMD_MFC_ENGINE_RESET);

	do {
		udelay(20);
		op_done = vidc_720p_read(0x050C);
		counter++;
	} while (!op_done && counter < 10);

	if (!op_done)
		return false;  /* reset fails */

	/* write invalid channel id */
	vidc_720p_write(0x0104, 4);

	/* Set INT_PULSE_SEL */
	if (interrupt_sel == VIDC_720P_INTERRUPT_LEVEL_SEL)
		vidc_720p_write(0x0504, 0);
	else
		vidc_720p_write(0x0504, 1);

	if (!interrupt_mask) {
		/* Disable interrupt */
		vidc_720p_write(0x0500, 1);
	} else {
	  /* Enable interrupt */
		vidc_720p_write(0x0500, 0);
	}

	/* Clear any pending interrupt */
	vidc_720p_write(0x0508, 1);

	/* Set INT_ENABLE_REG */
	vidc_720p_write(0x0518, interrupt_mask);

	/* Sets the DMA endianness */
	vidc_720p_write(0x0044, dma_endian);

	/* return engine reset success */
	return true ;
}

void vidc_720p_set_channel(u32 ch_id, enum vidc_720p_enc_dec_selection_type
	enc_dec_sel, enum vidc_720p_codec_type codec, phys_addr_t pi_fw,
	size_t firmware_size)
{
	u32 std_sel = codec;

	vidc_720p_write(0x012C, 0);

	if (enc_dec_sel)
		std_sel |= 0x10;

	vidc_720p_write(0x0100, std_sel);

	switch (codec) {
	default:
	case VIDC_720P_DIVX:
	case VIDC_720P_XVID:
	case VIDC_720P_MPEG4:
		if (enc_dec_sel == VIDC_720P_ENCODER)
			vidc_720p_write(0x0200, pi_fw);
		else
			vidc_720p_write(0x0204, pi_fw);
		break;
	case VIDC_720P_H264:
		if (enc_dec_sel == VIDC_720P_ENCODER)
			vidc_720p_write(0x0208, pi_fw);
		else
			vidc_720p_write(0x020C, pi_fw);
		break;
	case VIDC_720P_H263:
		if (enc_dec_sel == VIDC_720P_ENCODER)
			vidc_720p_write(0x0200, pi_fw);
		else
			vidc_720p_write(0x0218, pi_fw);
		break;
	case VIDC_720P_VC1:
		vidc_720p_write(0x0210, pi_fw);
		break;
	case VIDC_720P_MPEG2:
		vidc_720p_write(0x40293, pi_fw);
		break;
	}
	vidc_720p_write(0x000C, firmware_size / 4);  /* word size */

	vidc_720p_submit_command(ch_id, VIDC_720P_CMD_CHSET);
}

void vidc_720p_encode_set_profile(u32 profile, u32 level)
{
	u32 profile_level = profile|(level << 0x8);

	vidc_720p_write(0x0300, profile_level);
}

void vidc_720p_set_frame_size(u32 size_x, u32 size_y)
{
	vidc_720p_write(0x0118, size_x);

	vidc_720p_write(0x011C, size_y);
}

void vidc_720p_encode_set_fps(u32 rc_frame_rate)
{
	vidc_720p_write(0x0D14, rc_frame_rate);
}

void vidc_720p_encode_set_short_header(u32 short_header)
{
	vidc_720p_write(0x0318, short_header);
}

void vidc_720p_encode_set_vop_time(u32 vop_time_resolution,
		u32 vop_time_increment)
{
	u32 enable_vop, vop_timing_reg;

	if (!vop_time_resolution)
		vidc_720p_write(0x0E00, 0x0);
	else {
		enable_vop = 0x1;
		vop_timing_reg = (enable_vop << 0x1f) |
		(vop_time_resolution << 0x10) | vop_time_increment;
		vidc_720p_write(0x0E00, vop_timing_reg);
	}
}

void vidc_720p_encode_set_hec_period(u32 hec_period)
{
	vidc_720p_write(0x0EB0, hec_period);
}

void vidc_720p_encode_set_qp_params(u32 max_qp, u32 min_qp)
{
	u32 qp = min_qp | (max_qp << 0x8);

	vidc_720p_write(0x0A0C, qp);
}

void vidc_720p_encode_set_rc_config(u32 enable_frame_level_rc,
	u32 enable_mb_level_rc_flag, u32 iframe_qp, u32 pframe_qp)
{
	u32 rc_config = iframe_qp;

	if (enable_frame_level_rc)
		rc_config |= (0x1 << 0x9);

	if (enable_mb_level_rc_flag)
		rc_config |= (0x1 << 0x8);

	vidc_720p_write(0x0A00, rc_config);
	vidc_720p_write(0x0A04, pframe_qp);
}

void vidc_720p_encode_set_bit_rate(u32 target_bitrate)
{
	vidc_720p_write(0x0A08, target_bitrate);
}

void vidc_720p_encoder_set_param_change(u32 enc_param_change)
{
	vidc_720p_write(0x0E08, enc_param_change);
}

void vidc_720p_encode_set_control_param(u32 param_val)
{
	vidc_720p_write(0x0EC8, param_val);
}

void vidc_720p_encode_set_frame_level_rc_params(u32 reaction_coeff)
{
	vidc_720p_write(0x0A10, reaction_coeff);
}

void vidc_720p_encode_set_mb_level_rc_params(u32 dark_region_as_flag,
		u32 smooth_region_as_flag, u32 static_region_as_flag,
		u32 activity_region_flag)
{
	u32 mb_level_rc = 0x0;

	if (activity_region_flag)
		mb_level_rc |= 0x1;
	if (static_region_as_flag)
		mb_level_rc |= (0x1 << 0x1);
	if (smooth_region_as_flag)
		mb_level_rc |= (0x1 << 0x2);
	if (dark_region_as_flag)
		mb_level_rc |= (0x1 << 0x3);
	/* Write MB level rate control */
	vidc_720p_write(0x0A14, mb_level_rc);
}

void vidc_720p_encode_set_entropy_control(enum vidc_720p_entropy_sel_type
		entropy_sel, enum vidc_720p_cabac_model_type cabac_model_number)
{
	u32 num;
	u32 entropy_params = entropy_sel;

	/* Set Model Number */
	if (entropy_sel == VIDC_720P_ENTROPY_SEL_CABAC) {
		num = (u32)cabac_model_number;
		entropy_params |= (num << 0x2);
	}
	/* Set Entropy parameters */
	vidc_720p_write(0x0310, entropy_params);
}

void vidc_720p_encode_set_db_filter_control(enum vidc_720p_DBConfig_type
		db_config, u32 slice_alpha_offset, u32 slice_beta_offset)
{
	u32 deblock_params;

	deblock_params = db_config;
	deblock_params |=
		(slice_beta_offset << 0x2) | (slice_alpha_offset << 0x7);

	/* Write deblocking control settings */
	vidc_720p_write(0x0314, deblock_params);
}

void vidc_720p_encode_set_intra_refresh_mb_number(u32 cir_mb_number)
{
	vidc_720p_write(0x0810, cir_mb_number);
}

void vidc_720p_encode_set_multi_slice_info(enum vidc_720p_MSlice_selection_type
		m_slice_sel, u32 multi_slice_size)
{
	switch (m_slice_sel) {
	case VIDC_720P_MSLICE_BY_MB_COUNT:
		vidc_720p_write(0x0EA8, 0x1);
		vidc_720p_write(0x1517, m_slice_sel);
		vidc_720p_write(0x0324, multi_slice_size);
		break;
	case VIDC_720P_MSLICE_BY_BYTE_COUNT:
		vidc_720p_write(0x0EA8, 0x1);
		vidc_720p_write(0x1517, m_slice_sel);
		vidc_720p_write(0x0328, multi_slice_size);
		break;
	case VIDC_720P_MSLICE_BY_GOB:
		vidc_720p_write(0x0EA8, 0x1);
		break;
	default:
	case VIDC_720P_MSLICE_OFF:
		vidc_720p_write(0x0EA8, 0x0);
		break;
	}
}

void vidc_720p_encode_set_dpb_buffer(dma_addr_t pi_enc_dpb_addr,
	size_t alloc_len)
{
	vidc_720p_write(0x080C, pi_enc_dpb_addr);
	vidc_720p_write(0x0ED4, alloc_len);
}

void vidc_720p_encode_set_i_period(u32 period)
{
	vidc_720p_write(0x0308, period);
}

void vidc_720p_encode_init_codec(u32 ch_id,
	enum vidc_720p_memory_access_method_type memory_access_model)
{
	vidc_720p_write(0x0600, memory_access_model);
	vidc_720p_submit_command(ch_id, VIDC_720P_CMD_INITCODEC);
}

void vidc_720p_encode_unalign_bitstream(u32 upper_unalign_word,
	u32 lower_unalign_word)
{
	vidc_720p_write(0x0EA0, upper_unalign_word);
	vidc_720p_write(0x0EA4, lower_unalign_word);
}

void vidc_720p_encode_set_seq_header_buffer(phys_addr_t ext_buffer_start,
	phys_addr_t ext_buffer_end, u32 start_byte_num)
{
	vidc_720p_write(0x0018, ext_buffer_start);

	vidc_720p_write(0x0024, ext_buffer_start);

	vidc_720p_write(0x001C, ext_buffer_end);

	vidc_720p_write(0x005C, start_byte_num);
}

void vidc_720p_encode_frame(u32 ch_id, phys_addr_t ext_buffer_start,
	phys_addr_t ext_buffer_end, u32 start_byte_number, phys_addr_t y_addr,
	phys_addr_t c_addr)
{
	vidc_720p_write(0x0018, ext_buffer_start);

	vidc_720p_write(0x001C, ext_buffer_end);

	vidc_720p_write(0x0024, ext_buffer_start);

	vidc_720p_write(0x005C, start_byte_number);

	vidc_720p_write(0x99105, y_addr);

	vidc_720p_write(0x0804, c_addr);

	vidc_720p_submit_command(ch_id, VIDC_720P_CMD_FRAMERUN);
}

void vidc_720p_encode_get_header(u32 *pi_enc_header_size)
{
	*pi_enc_header_size = vidc_720p_read(0x0060);
}

void vidc_720p_enc_frame_info(struct vidc_720p_enc_frame_info *enc_frame_info)
{
	enc_frame_info->enc_size = vidc_720p_read(0x0058);

	enc_frame_info->frame_type = vidc_720p_read(0x0EBC);

	enc_frame_info->frame_type &= 0x03;

	enc_frame_info->metadata_exists = vidc_720p_read(0x0EB8);
}

void vidc_720p_decode_bitstream_header(u32 ch_id, u32 dec_unit_size,
		u32 start_byte_num, u32 ext_buffer_start, u32 ext_buffer_end,
		enum vidc_720p_memory_access_method_type memory_access_model)
{
	vidc_720p_write(0x0E04, 0x0);

	vidc_720p_write(0x0018, ext_buffer_start);

	vidc_720p_write(0x001C, ext_buffer_end);

	vidc_720p_write(0x0024, ext_buffer_end);

	vidc_720p_write(0x0054, dec_unit_size);

	vidc_720p_write(0x005C, start_byte_num);

	vidc_720p_write(0x0600, memory_access_model);

	vidc_720p_submit_command(ch_id, VIDC_720P_CMD_INITCODEC);
}

void vidc_720p_decode_get_seq_hdr_info(struct vidc_720p_seq_hdr_info_type
		*seq_hdr_info)
{
	unsigned long tmp;

	seq_hdr_info->img_size_x = vidc_720p_read(0x0118);

	seq_hdr_info->img_size_y = vidc_720p_read(0x011C);

	seq_hdr_info->min_num_dpb = vidc_720p_read(0x0E10);

	seq_hdr_info->min_dpb_size = vidc_720p_read(0x0C10);

	seq_hdr_info->dec_frm_size = vidc_720p_read(0x0C08);

	tmp = vidc_720p_read(0x0C0C);
	seq_hdr_info->profile = tmp & 0x1f;
	seq_hdr_info->level = (tmp & 0xff00) >> 8;

	tmp = vidc_720p_read(0x0408);
	seq_hdr_info->progressive = (tmp & 0x4) >> 2;
	/* bit 3 is for crop existence */
	seq_hdr_info->crop_exists = (tmp & 0x8) >> 3;

	if (seq_hdr_info->crop_exists) {
		/* read the cropping information */
		tmp = vidc_720p_read(0x0C00);
		seq_hdr_info->crop_right_offset = (tmp & 0xffff0000) >> 0x10;
		seq_hdr_info->crop_left_offset = tmp & 0xffff;
		tmp = vidc_720p_read(0x0C04);
		seq_hdr_info->crop_bottom_offset = (tmp & 0xffff0000) >> 0x10;
		seq_hdr_info->crop_top_offset = tmp & 0xffff;
	}
	/* Read the MPEG4 data partitioning indication */
	seq_hdr_info->data_partitioned = (vidc_720p_read(0x0EBC) & 0x8) >> 3;
}

void vidc_720p_decode_set_dpb_release_buffer_mask(u32 dpb_release_buffer_mask)
{
	vidc_720p_write(0x0E98, dpb_release_buffer_mask);
}

void vidc_720p_decode_set_dpb_buffers(u32 i, phys_addr_t dpb_buffer)
{
	vidc_720p_write(0x0E18 + sizeof(i) * i, dpb_buffer);
}

void vidc_720p_decode_set_comv_buffer(phys_addr_t pi_dpb_comv_buffer,
	size_t alloc_len)
{
	vidc_720p_write(0x0904, pi_dpb_comv_buffer);

	vidc_720p_write(0x0D08, alloc_len);
}

void vidc_720p_decode_set_dpb_details(u32 num_dpb, size_t alloc_len,
	phys_addr_t ref_buffer)
{
	vidc_720p_write(0x0900, ref_buffer);

	vidc_720p_write(0x0908, 0);

	vidc_720p_write(0x0E14, num_dpb);

	vidc_720p_write(0x0ED4, alloc_len);
}

void vidc_720p_decode_set_mpeg4Post_filter(u32 enable_post_filter)
{
	if (enable_post_filter)
		vidc_720p_write(0x0124, 0x1);
	else
		vidc_720p_write(0x0124, 0x0);
}

void vidc_720p_decode_set_error_control(u32 enable_error_control)
{
	if (enable_error_control)
		vidc_720p_write(0x013C, 0);
	else
		vidc_720p_write(0x013C, 1);
}

void vidc_720p_set_deblock_line_buffer(dma_addr_t pi_deblock_line_buffer_start,
					size_t alloc_len)
{
	vidc_720p_write(0x0234, pi_deblock_line_buffer_start);

	vidc_720p_write(0x0D04, alloc_len);
}

void vidc_720p_decode_set_mpeg4_data_partitionbuffer(dma_addr_t vsp_buf_start)
{
	vidc_720p_write(0x0230, vsp_buf_start);
}

void vidc_720p_decode_setH264VSPBuffer(dma_addr_t pi_vsp_temp_buffer_start)
{
	vidc_720p_write(0x0230, pi_vsp_temp_buffer_start);
}

void vidc_720p_decode_frame(u32 ch_id, phys_addr_t ext_buffer_start,
	phys_addr_t ext_buffer_end, size_t dec_unit_size, u32 start_byte_num,
	u32 input_frame_tag)
{
	vidc_720p_write(0x0018, ext_buffer_start);

	vidc_720p_write(0x001C, ext_buffer_end);

	vidc_720p_write(0x0024, ext_buffer_end);

	vidc_720p_write(0x005C, start_byte_num);

	vidc_720p_write(0x0EE0, input_frame_tag);

	vidc_720p_write(0x0054, dec_unit_size);

	vidc_720p_submit_command(ch_id, VIDC_720P_CMD_FRAMERUN);
}

void vidc_720p_issue_eos(u32 ch_id)
{
	vidc_720p_write(0x0028, 0x1);

	vidc_720p_write(0x0054, 0);

	vidc_720p_submit_command(ch_id, VIDC_720P_CMD_FRAMERUN);
}

void vidc_720p_eos_info(u32 *disp_status)
{
	*disp_status = vidc_720p_read(0x0408) & 0x3;
}

void vidc_720p_decode_display_info(struct vidc_720p_dec_disp_info *disp_info)
{
	unsigned long tmp;

	tmp = vidc_720p_read(0x0408);

	disp_info->disp_status = (enum vidc_720p_display_status_type)
		(tmp & 0x3);

	disp_info->disp_is_interlace = (tmp & 0x4) >> 2;
	disp_info->crop_exists = (tmp & 0x8) >> 3;

	disp_info->resl_change = (tmp & 0x30) >> 4;

	disp_info->reconfig_flush_done = vidc_720p_read(0x0EC0) & 0x1;

	disp_info->img_size_x = vidc_720p_read(0x0118);
	disp_info->img_size_y = vidc_720p_read(0x011C);
	disp_info->y_addr = vidc_720p_read(0x0400);
	disp_info->c_addr = vidc_720p_read(0x0404);
	disp_info->tag_top = vidc_720p_read(0x0EA8);
	disp_info->tag_bottom = vidc_720p_read(0x0EE4);
	disp_info->pic_time_top = vidc_720p_read(0x0ED8);
	disp_info->pic_time_bottom = vidc_720p_read(0x0EDC);

	if (disp_info->crop_exists) {
		tmp = vidc_720p_read(0x0C00);
		disp_info->crop_right_offset = (tmp & 0xffff0000) >> 0x10;
		disp_info->crop_left_offset = tmp & 0xffff;
		tmp = vidc_720p_read(0x0C04);
		disp_info->crop_bottom_offset = (tmp & 0xffff0000) >> 0x10;
		disp_info->crop_top_offset = tmp & 0xffff;
	}
	disp_info->metadata_exists = vidc_720p_read(0x0EB8);

	disp_info->input_bytes_consumed = vidc_720p_read(0x0C08);

	disp_info->input_frame_num = vidc_720p_read(0x0410);

	disp_info->input_frame_type = vidc_720p_read(0x0EBC) & 0x7;

	disp_info->input_is_interlace = (disp_info->input_frame_type & 0x4) >>
		2;

	disp_info->input_frame_type &= 0x3;
}

void vidc_720p_decode_skip_frm_details(phys_addr_t *free_luma_dpb)
{
	u32 disp_frm_type;

	disp_frm_type = vidc_720p_read(0x0EB4);

	if (disp_frm_type == VIDC_720P_NOTCODED)
		*free_luma_dpb = vidc_720p_read(0x0C18);
}

void vidc_720p_metadata_enable(u32 flag, phys_addr_t input_buffer)
{
	vidc_720p_write(0x0EC4, flag);
	vidc_720p_write(0x0ED0, input_buffer);
}

void vidc_720p_decode_dynamic_req_reset(void)
{
	vidc_720p_write(0x0EE8, 0x0);
	vidc_720p_write(0x0EAC, 0x0);
	vidc_720p_write(0x0028, 0x0);
}

void vidc_720p_decode_dynamic_req_set(u32 property)
{
	if (property == VIDC_720P_FLUSH_REQ)
		vidc_720p_write(0x0EE8, 0x1);
	else if (property == VIDC_720P_EXTRADATA)
		vidc_720p_write(0x0EAC, 0x1);
}

void vidc_720p_decode_setpassthrough_start(phys_addr_t pass_startaddr)
{
	vidc_720p_write(0x0D18, pass_startaddr);
}
