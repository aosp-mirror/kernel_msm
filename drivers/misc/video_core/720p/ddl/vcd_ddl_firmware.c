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

#include "video_core_type.h"

#include "vcd_ddl_firmware.h"
#include "vcd_ddl_utils.h"

#define VCDFW_TOTALNUM_IMAGE  7
#define VCDFW_MAX_NO_IMAGE    2

struct vcd_firmware_type {
	u32 a_active_fw_img[VCDFW_TOTALNUM_IMAGE];
	struct ddl_buf_addr_type boot_code;

	struct ddl_buf_addr_type enc_mpeg4;
	struct ddl_buf_addr_type encH264;

	struct ddl_buf_addr_type dec_mpeg4;
	struct ddl_buf_addr_type decH264;
	struct ddl_buf_addr_type decH263;
	struct ddl_buf_addr_type dec_mpeg2;
	struct ddl_buf_addr_type dec_vc1;
};

static struct vcd_firmware_type vcd_firmware;


static void vcd_fw_change_endian(unsigned char *fw, u32 n_fw_size)
{
	u32 i = 0;
	unsigned char temp;
	for (i = 0; i < n_fw_size; i = i + 4) {
		temp = fw[i];
		fw[i] = fw[i + 3];
		fw[i + 3] = temp;

		temp = fw[i + 1];
		fw[i + 1] = fw[i + 2];
		fw[i + 2] = temp;
	}
	return;
}

static u32 vcd_fw_prepare(struct ddl_buf_addr_type *fw_details,
			 const unsigned char fw_array[],
			 const unsigned int fw_array_size, u32 change_endian)
{
	u32 *buffer;

	ddl_pmem_alloc(fw_details, fw_array_size,
		       DDL_LINEAR_BUFFER_ALIGN_BYTES);
	if (!fw_details->p_virtual_base_addr)
		return FALSE;

	fw_details->n_buffer_size = fw_array_size / 4;

	buffer = fw_details->p_align_virtual_addr;

	memcpy(buffer, fw_array, fw_array_size);
	if (change_endian)
		vcd_fw_change_endian((unsigned char *)buffer, fw_array_size);
	return TRUE;
}

u32 vcd_fw_init(void)
{
	u32 b_status = FALSE;

	b_status = vcd_fw_prepare(&vcd_firmware.boot_code,
				vid_c_command_control_fw,
				vid_c_command_control_fw_size, FALSE);

	if (b_status) {
		b_status = vcd_fw_prepare(&vcd_firmware.dec_mpeg4,
					vid_c_mpg4_dec_fw,
					vid_c_mpg4_dec_fw_size, TRUE);
	}

	if (b_status) {
		b_status = vcd_fw_prepare(&vcd_firmware.decH264,
					vid_c_h264_dec_fw,
					vid_c_h264_dec_fw_size, TRUE);
	}

	if (b_status) {
		b_status = vcd_fw_prepare(&vcd_firmware.decH263,
					vid_c_h263_dec_fw,
					vid_c_h263_dec_fw_size, TRUE);
	}

	if (b_status) {
		b_status = vcd_fw_prepare(&vcd_firmware.enc_mpeg4,
					vid_c_mpg4_enc_fw,
					vid_c_mpg4_enc_fw_size, TRUE);
	}

	if (b_status) {
		b_status = vcd_fw_prepare(&vcd_firmware.encH264,
					vid_c_h264_enc_fw,
					vid_c_h264_enc_fw_size, TRUE);
	}

	return b_status;
}


static u32 get_dec_fw_image(struct vcd_fw_details_type *p_fw_details)
{
	u32 b_return = TRUE;
	switch (p_fw_details->e_codec) {
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
	case VCD_CODEC_MPEG4:
		{
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.dec_mpeg4.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.dec_mpeg4.n_buffer_size;
			break;
		}
	case VCD_CODEC_H264:
		{
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.decH264.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.decH264.n_buffer_size;
			break;
		}
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		{
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.dec_vc1.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.dec_vc1.n_buffer_size;
			break;
		}
	case VCD_CODEC_MPEG2:
		{
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.dec_mpeg2.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.dec_mpeg2.n_buffer_size;
			break;
		}
	case VCD_CODEC_H263:
		{
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.decH263.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.decH263.n_buffer_size;
			break;
		}
	default:
		{
			b_return = FALSE;
			break;
		}
	}
	return b_return;
}

static u32 get_enc_fw_image(struct vcd_fw_details_type *p_fw_details)
{
	u32 b_return = TRUE;
	switch (p_fw_details->e_codec) {
	case VCD_CODEC_H263:
	case VCD_CODEC_MPEG4:
		{
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.enc_mpeg4.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.enc_mpeg4.n_buffer_size;
			break;
		}
	case VCD_CODEC_H264:
		{
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.encH264.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.encH264.n_buffer_size;
			break;
		}
	default:
		{
			b_return = FALSE;
			break;
		}
	}
	return b_return;
}

u32 vcd_get_fw_property(u32 prop_id, void *prop_details)
{
	u32 b_return = TRUE;
	struct vcd_fw_details_type *p_fw_details;
	switch (prop_id) {
	case VCD_FW_ENDIAN:
		{
			*(u32 *) prop_details = VCD_FW_BIG_ENDIAN;
			break;
		}
	case VCD_FW_BOOTCODE:
		{
			p_fw_details =
			    (struct vcd_fw_details_type *)prop_details;
			p_fw_details->p_fw_buffer_addr =
			    vcd_firmware.boot_code.p_align_physical_addr;
			p_fw_details->n_fw_size =
			    vcd_firmware.boot_code.n_buffer_size;
			break;
		}
	case VCD_FW_DECODE:
		{
			p_fw_details =
			    (struct vcd_fw_details_type *)prop_details;
			b_return = get_dec_fw_image(p_fw_details);
			break;
		}
	case VCD_FW_ENCODE:
		{
			p_fw_details =
			    (struct vcd_fw_details_type *)prop_details;
			b_return = get_enc_fw_image(p_fw_details);
			break;
		}
	default:
		{
			b_return = FALSE;
			break;
		}
	}
	return b_return;
}

u32 vcd_fw_transact(u32 b_add, u32 b_decoding, enum vcd_codec_type e_codec)
{
	u32 b_return = TRUE;
	u32 n_index = 0, n_active_fw = 0, n_loop_count;

	if (b_decoding) {
		switch (e_codec) {
		case VCD_CODEC_DIVX_4:
		case VCD_CODEC_DIVX_5:
		case VCD_CODEC_DIVX_6:
		case VCD_CODEC_XVID:
		case VCD_CODEC_MPEG4:
			{
				n_index = 0;
				break;
			}
		case VCD_CODEC_H264:
			{
				n_index = 1;
				break;
			}
		case VCD_CODEC_H263:
			{
				n_index = 2;
				break;
			}
		case VCD_CODEC_MPEG2:
			{
				n_index = 3;
				break;
			}
		case VCD_CODEC_VC1:
		case VCD_CODEC_VC1_RCV:
			{
				n_index = 4;
				break;
			}
		default:
			{
				b_return = FALSE;
				break;
			}
		}
	} else {
		switch (e_codec) {
		case VCD_CODEC_H263:
		case VCD_CODEC_MPEG4:
			{
				n_index = 5;
				break;
			}
		case VCD_CODEC_H264:
			{
				n_index = 6;
				break;
			}
		default:
			{
				b_return = FALSE;
				break;
			}
		}
	}

	if (!b_return)
		return b_return;

	if (!b_add &&
	    vcd_firmware.a_active_fw_img[n_index]
	    ) {
		--vcd_firmware.a_active_fw_img[n_index];
		return b_return;
	}

	for (n_loop_count = 0; n_loop_count < VCDFW_TOTALNUM_IMAGE;
	     ++n_loop_count) {
		if (vcd_firmware.a_active_fw_img[n_loop_count])
			++n_active_fw;
	}

	if (n_active_fw < VCDFW_MAX_NO_IMAGE ||
	    vcd_firmware.a_active_fw_img[n_index] > 0) {
		++vcd_firmware.a_active_fw_img[n_index];
	} else {
		b_return = FALSE;
	}
	return b_return;
}

void vcd_fw_release(void)
{
	ddl_pmem_free(vcd_firmware.boot_code);
	ddl_pmem_free(vcd_firmware.enc_mpeg4);
	ddl_pmem_free(vcd_firmware.encH264);
	ddl_pmem_free(vcd_firmware.dec_mpeg4);
	ddl_pmem_free(vcd_firmware.decH264);
	ddl_pmem_free(vcd_firmware.decH263);
	ddl_pmem_free(vcd_firmware.dec_mpeg2);
	ddl_pmem_free(vcd_firmware.dec_vc1);
}
