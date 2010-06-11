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

#include <linux/firmware.h>

#include "video_core_type.h"
#include "vcd_ddl_firmware.h"

struct vcd_firmware_table {
	bool prepared;
	struct device *dev;
	struct vcd_firmware fw[6];
};

//TODO max_sz is kinda sucky, a better way?
static struct vcd_firmware_table vcd_fw_table = {
	.prepared = false,
	.fw[0] = {
		.filename = "vidc_720p_command_control.fw",
		.change_endian = false,
		.max_sz = 12288,
	},
	.fw[1] = {
		.filename = "vidc_720p_mp4_dec_mc.fw",
		.change_endian = true,
		.max_sz = 32768,
	},
	.fw[2] = {
		.filename = "vidc_720p_h263_dec_mc.fw",
		.change_endian = true,
		.max_sz = 24576,
	},
	.fw[3] = {
		.filename = "vidc_720p_h264_dec_mc.fw",
		.change_endian = true,
		.max_sz = 45056,
	},
	.fw[4] = {
		.filename = "vidc_720p_mp4_enc_mc.fw",
		.change_endian = true,
		.max_sz = 32768,
	},
	.fw[5] = {
		.filename = "vidc_720p_h264_enc_mc.fw",
		.change_endian = true,
		.max_sz = 36864,
	},

};

static void vcd_fw_change_endian(struct vcd_firmware *vcd_fw)
{
	size_t i;
	u8 tmp;
	u8 *fw = vcd_fw->virt_addr;
	for (i = 0; i < vcd_fw->sz; i += 4) {
		tmp = fw[i];
		fw[i] = fw[i + 3];
		fw[i + 3] = tmp;

		tmp = fw[i + 1];
		fw[i + 1] = fw[i + 2];
		fw[i + 2] = tmp;
	}
}

static int vcd_fw_prepare(struct vcd_firmware *vcd_fw)
{
	int rc;
	const struct firmware *fw;

	rc = request_firmware(&fw, vcd_fw->filename, vcd_fw_table.dev);
	if (rc) {
		pr_err("request_firmware(%s) failed %d\n", vcd_fw->filename,
			rc);
		return rc;
	}

	if (fw->size > vcd_fw->max_sz) {
		pr_err("firmware %s is larger than allocated size (%u > %u)\n",
			vcd_fw->filename, fw->size, vcd_fw->max_sz);
		rc = -ENOMEM;
		goto out;
	}
	vcd_fw->sz = fw->size;
	memcpy(vcd_fw->virt_addr, fw->data, fw->size);

	if (vcd_fw->change_endian)
		vcd_fw_change_endian(vcd_fw);

	pr_info("prepared firmware %s\n", vcd_fw->filename);

out:
	release_firmware(fw);
	return rc;
}

int vcd_fw_prepare_all()
{
	int i;
	int rc = 0;

	if (vcd_fw_table.prepared)
		goto out;

	for (i = 0; i < ARRAY_SIZE(vcd_fw_table.fw); i++) {
		rc = vcd_fw_prepare(&vcd_fw_table.fw[i]);
		if (rc)
			goto out;
	}
	vcd_fw_table.prepared = true;

out:
	return rc;
}

int vcd_fw_init(struct device *dev) {
	int i;
	vcd_fw_table.dev = dev;
	for (i = 0; i < ARRAY_SIZE(vcd_fw_table.fw); i++) {
		struct vcd_firmware *fw = &vcd_fw_table.fw[i];
		fw->virt_addr = dma_alloc_coherent(NULL, fw->max_sz,
			&fw->phys_addr, GFP_KERNEL);
		if (!fw->virt_addr) {
			pr_err("failed to allocate %d for %s\n", fw->max_sz,
				fw->filename);
			vcd_fw_exit();
			return -ENOMEM;
		}
	}
	return 0;
}

void vcd_fw_exit(void) {
	int i;
	vcd_fw_table.prepared = false;
	for (i = 0; i < ARRAY_SIZE(vcd_fw_table.fw); i++) {
		struct vcd_firmware *fw = &vcd_fw_table.fw[i];
		if (!fw->virt_addr)
			continue;
		dma_free_coherent(NULL, fw->max_sz, fw->virt_addr,
			fw->phys_addr);
	}
}

struct vcd_firmware *vcd_fw_get_boot_fw(void)
{
	if (!vcd_fw_table.prepared)
		return NULL;
	return &vcd_fw_table.fw[0];
}

struct vcd_firmware *vcd_fw_get_fw(bool is_decode, enum vcd_codec codec)
{
	if (!vcd_fw_table.prepared)
		return NULL;

	if (is_decode) {
		switch (codec) {
		case VCD_CODEC_DIVX_4:
		case VCD_CODEC_DIVX_5:
		case VCD_CODEC_DIVX_6:
		case VCD_CODEC_XVID:
		case VCD_CODEC_MPEG4:
			return &vcd_fw_table.fw[1];
		case VCD_CODEC_H264:
			return &vcd_fw_table.fw[3];
		case VCD_CODEC_VC1:
		case VCD_CODEC_VC1_RCV:
			/* vidc_720p_vc1_dec_mc.fw - untested */
			break;
		case VCD_CODEC_MPEG2:
			/* vidc_720p_mp2_dec_mc.fw - untested */
			break;
		case VCD_CODEC_H263:
			return &vcd_fw_table.fw[2];
		default:
			break;
		}
	} else {
		switch (codec) {
		case VCD_CODEC_H263:
		case VCD_CODEC_MPEG4:
			return &vcd_fw_table.fw[4];
		case VCD_CODEC_H264:
			return &vcd_fw_table.fw[5];
		default:
			break;
		}
	}
	return NULL;
}

bool vcd_fw_is_codec_supported(bool is_decode, enum vcd_codec codec)
{
	return vcd_fw_get_fw(is_decode, codec) != NULL;
}
