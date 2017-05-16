/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name                   : ftm4_fwu.c
* Authors                      : AMS(Analog Mems Sensor) Team
* Description     : FTS Capacitive touch screen controller (FingerTipS)
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include "ftm4_ts.h"

#define FTS64FILE_SIGNATURE 0xaaaa5555

enum {
	BUILT_IN = 0,
	UMS,
	NONE,
	FFU,
};

struct fts64_header {
	unsigned int signature;
	unsigned short fw_ver;
	unsigned char fw_id;
	unsigned char reserved1;
	unsigned char internal_ver[8];
	unsigned char released_ver[8];
	unsigned int reserved2;
	unsigned int checksum;
};

bool get_pure_autotune_status(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char addrs[3];
	unsigned char buf[5];
	bool ret = false;
	int doffset = 1;

	if (info->digital_rev == FTS_DIGITAL_REV_1)
		doffset = 0;

	addrs[0] = 0xd0;
	addrs[1] = 0x00;
	addrs[2] = 0x4E;

	rc = info->fts_read_reg(info, addrs, 3, buf, 4);
	if (rc < 0) {
		tsp_debug_err(info->dev,
			"%s: PureAutotune Information Read Fail!!"
			"[Data : %2X%2X]\n",
			__func__, buf[0 + doffset],
			buf[1 + doffset]);
	} else {
		if ((buf[0 + doffset] == 0xA5) && (buf[1 + doffset] == 0x96))
			ret = 1;
		tsp_debug_info(info->dev,
			"%s: PureAutotune Information !! "
			"[Data : %2X%2X]\n", __func__,
			buf[0 + doffset],
			buf[1 + doffset]);
	}
	return ret;
}
EXPORT_SYMBOL(get_pure_autotune_status);

static bool get_afe_status(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char addrs[3];
	unsigned char buf[5];
	bool ret = false;
	int doffset = 1;

	if (info->digital_rev == FTS_DIGITAL_REV_1)
		doffset = 0;

	addrs[0] = 0xd0;
	addrs[1] = 0x00;
	addrs[2] = 0x52;

	rc = info->fts_read_reg(info, addrs, 3, buf, 4);
	if (rc < 0) {
		tsp_debug_err(info->dev,
			"%s: Read Fail - Final AFE [Data :"
			" %2X] AFE Ver [Data : %2X]\n",
			__func__,
			buf[0 + doffset],
			buf[1 + doffset]);
		return rc;
	}

	if (buf[0 + doffset])
		ret = true;

	tsp_debug_info(info->dev,
		"%s: Final AFE [Data : %2X] AFE Ver "
		"[Data : %2X]\n",
		__func__,
		buf[0 + doffset],
		buf[1 + doffset]);

	return ret;
}

int fts_fw_wait_for_specific_event(struct fts_ts_info *info,
		unsigned char eid0, unsigned char eid1, unsigned char eid2)
{
	int rc = 0;
	unsigned char addrs;
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	addrs = READ_ONE_EVENT;

	while (info->fts_read_reg(info, &addrs, 1, (unsigned char *)data,
				FTS_EVENT_SIZE)) {
		if (data[0]) {
			if ((data[0] == eid0) && (data[1] == eid1) &&
					(data[2] == eid2)) {
				rc = 0;
				break;
			} else {
				tsp_debug_dbg(info->dev,
					"%s: %2X, %2X, %2X, %2X\n",
					__func__, data[0], data[1],
					data[2], data[3]);
			}
		}
		if (retry++ > FTS_RETRY_COUNT * 15) {
			rc = -1;
			tsp_debug_err(info->dev,
				"%s: Time Over ( %2X, %2X, %2X, %2X )\n",
				__func__, data[0], data[1],
				data[2], data[3]);
			break;
		}
		fts_delay(20);
	}

	return rc;
}

int fts_fw_wait_for_event(struct fts_ts_info *info, unsigned char eid0,
		unsigned char eid1)
{
	int rc = 0;
	unsigned char addrs;
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	addrs = READ_ONE_EVENT;

	while (info->fts_read_reg(info, &addrs, 1, (unsigned char *)data,
				FTS_EVENT_SIZE)) {
		if ((data[0] == EVENTID_STATUS_EVENT) ||
		    (data[0] == EVENTID_ERROR)) {
			if ((data[0] == EVENTID_STATUS_EVENT) &&
			    (data[1] == eid0) && (data[2] == eid1)) {
				break;
			} else if ((data[0] == EVENTID_STATUS_EVENT) &&
				 (data[1] == STATUS_EVENT_FORCE_CAL_DONE)) {
				break;
			} else {
				tsp_debug_dbg(info->dev,
					"%s: %2X,%2X,%2X,%2X\n",
					__func__,
					data[0],
					data[1],
					data[2],
					data[3]);
			}
		}

		if (retry++ > FTS_RETRY_COUNT * 15) {
			rc = -1;
			tsp_debug_err(info->dev,
				"%s: Time Over (%2X,%2X,%2X,%2X)\n",
				__func__,
				data[0],
				data[1],
				data[2],
				data[3]);
			break;
		}
		fts_delay(20);
	}

	return rc;
}

void fts_execute_autotune(struct fts_ts_info *info)
{
	int ret = 0;
	unsigned char regData[4]; /* {0xC1, 0x0E}; */
	bool bFinalAFE = false;
	bool NoNeedAutoTune = false; /* default for factory */

	bFinalAFE = get_afe_status(info);

	/* Check flag and decide cx_tune */
	NoNeedAutoTune = get_pure_autotune_status(info);

	tsp_debug_info(info->dev,
		"%s: AFE(%d), NoNeedAutoTune(%d)\n", __func__,
		bFinalAFE, NoNeedAutoTune);

	if ((!NoNeedAutoTune) || (info->o_afe_ver != info->afe_ver)) {
		info->fts_command(info, CX_TUNNING);
		fts_delay(300);
		fts_fw_wait_for_event(info, STATUS_EVENT_MUTUAL_AUTOTUNE_DONE,
				0x00);

		info->fts_command(info, SELF_AUTO_TUNE);
		fts_delay(300);
		fts_fw_wait_for_event(info, STATUS_EVENT_SELF_AUTOTUNE_DONE,
				0x00);

		if (NoNeedAutoTune) {
			tsp_debug_info(info->dev,
				"%s: AFE_status(%d) write ( C8 01 )\n",
				__func__, bFinalAFE);

			regData[0] = 0xC8;
			regData[1] = 0x01;
			ret = info->fts_write_reg(info, regData, 2);
			if (ret < 0) {
				tsp_debug_err(info->dev,
				"%s: Flash Back up PureAutotune"
				"Fail (Clear)\n", __func__);
			}

			fts_delay(20);
			fts_fw_wait_for_event(info,
				STATUS_EVENT_PURE_AUTOTUNE_FLAG_CLEAR_FINISH,
				0x00);
		}

		info->fts_command(info, FTS_CMD_SAVE_CX_TUNING);
		fts_delay(230);
		fts_fw_wait_for_event(info,
				STATUS_EVENT_FLASH_WRITE_CXTUNE_VALUE, 0x00);

		info->fts_command(info, FTS_CMD_SAVE_FWCONFIG);
		fts_delay(230);
		fts_fw_wait_for_event(info, STATUS_EVENT_FLASH_WRITE_CONFIG,
				0x00);

		/* Reset FTS */
		info->fts_systemreset(info);
		fts_delay(20);
		/* wait for ready event */
		info->fts_wait_for_ready(info);
	}
}

#define FW_IMAGE_NAME_D2_TB_INTEG	"tsp_stm/stm_tb_integ.fw"
#define FW_IMAGE_NAME_D2_Z2A		"tsp_stm/stm_z2a.fw"
#define FW_IMAGE_NAME_D2_Z2I		"tsp_stm/stm_z2i.fw"
#define CONFIG_ID_D1_S			0x2C
#define CONFIG_ID_D2_TR			0x2E
#define CONFIG_ID_D2_TB			0x30
#define CONFIG_OFFSET_BIN_D1		0xf822
#define CONFIG_OFFSET_BIN_D2		0x1E822
#define RX_OFFSET_BIN_D2		0x1E834
#define TX_OFFSET_BIN_D2		0x1E835

#define FW_IMAGE_SIZE_D3		(256 * 1024)
#define SIGNEDKEY_SIZE			(256)

int wait_for_flash_ready(struct fts_ts_info *info, uint8_t type)
{
	uint8_t cmd[2] = {FLASH_CMD_READ_REGISTER, type};
	uint8_t readData;
	int i, res = -1;

	tsp_debug_info(info->dev, "[wait_for_flash_ready"
		" Waiting for flash ready\n");

	for (i = 0; i < 1000 && res != 0; i++) {
		info->fts_read_reg(info, cmd, sizeof(cmd), &readData, 1);
		res = readData & 0x80;
		fts_delay(50);
	}

	if (i >= 1000 && res != 0) {
		tsp_debug_err(info->dev, "[wait_for_flash_ready]"
			" Wait for flash TIMEOUT! ERROR\n");
		return 0;
	}

	tsp_debug_info(info->dev, "[wait_for_flash_ready]"
		" Flash READY!\n");

	return 1;
}

int start_flash_dma(struct fts_ts_info *info)
{
	int status;
	uint8_t cmd[3] = {FLASH_CMD_WRITE_REGISTER,
		FLASH_DMA_CODE0, FLASH_DMA_CODE1};

	tsp_debug_info(info->dev,
			"[start_flash_dma] Command flash DMA ...\n");
	info->fts_write_reg(info, cmd, sizeof(cmd));

	status = wait_for_flash_ready(info, FLASH_DMA_CODE0);

	if (status != true) {
		tsp_debug_err(info->dev,
				"[start_flash_dma] start_flash_dma: ERROR\n");
		return false;
	}
	tsp_debug_info(info->dev, "[start_flash_dma] flash DMA DONE!\n");

	return true;
}

int fillFlash(struct fts_ts_info *info, uint32_t address, uint8_t *data,
		int size)
{
	int remaining;
	int toWrite = 0;
	int byteBlock = 0;
	int wheel = 0;
	uint32_t addr = 0;
	int res;
	int delta;

	uint8_t buff[DMA_CHUNK + 3] = {0};

	remaining = size;
	while (remaining > 0) {
		byteBlock = 0;
		addr = 0;
		tsp_debug_info(info->dev,
				"[fillFlash] [%d] Write data to memory.\n",
				wheel);
		while (byteBlock < FLASH_CHUNK && remaining > 0) {
			buff[0] = FLASH_CMD_WRITE_64K;
			if (remaining >= DMA_CHUNK) {
				if ((byteBlock + DMA_CHUNK) <= FLASH_CHUNK) {
					toWrite = DMA_CHUNK;
					remaining -= DMA_CHUNK;
					byteBlock += DMA_CHUNK;
				} else {
					delta = FLASH_CHUNK - byteBlock;
					toWrite = delta;
					remaining -= delta;
					byteBlock += delta;
				}
			} else {
				if ((byteBlock + remaining) <= FLASH_CHUNK) {
					toWrite = remaining;
					byteBlock += remaining;
					remaining = 0;

				} else {
					delta = FLASH_CHUNK - byteBlock;
					toWrite = delta;
					remaining -= delta;
					byteBlock += delta;
				}
			}

			buff[1] = (uint8_t) ((addr & 0x0000FF00) >> 8);
			buff[2] = (uint8_t) (addr & 0x000000FF);
			memcpy(&buff[3], data, toWrite);
			info->fts_write_reg(info, buff, 3 + toWrite);

			addr += toWrite;
			data += toWrite;
		}

		/* configuring the DMA */
		tsp_debug_info(info->dev,
				"[fillFlash] [%d] Configure DMA\n", wheel);
		byteBlock = byteBlock / 4 - 1;

		buff[0] = FLASH_CMD_WRITE_REGISTER;
		buff[1] = FLASH_DMA_CONFIG;
		buff[2] = 0x00;
		buff[3] = 0x00;

		addr = address + ((wheel * FLASH_CHUNK)/4);
		buff[4] = (uint8_t) ((addr & 0x000000FF));
		buff[5] = (uint8_t) ((addr & 0x0000FF00) >> 8);
		buff[6] = (uint8_t) (byteBlock & 0x000000FF);
		buff[7] = (uint8_t) ((byteBlock & 0x0000FF00) >> 8);
		buff[8] = 0x00;

		info->fts_write_reg(info, buff, 9);
		fts_delay(10);

		tsp_debug_info(info->dev,
				"[fillFlash] [%d] Start flash DMA\n", wheel);
		res = start_flash_dma(info);
		if (res < true) {
			tsp_debug_err(info->dev,
				"[fillFlash] Error during flashing DMA! ERROR\n");
			return false;
		}
		tsp_debug_info(info->dev,
				"[fillFlash] [%d] DMA done\n", wheel);

		wheel++;
	}
	return true;
}

uint32_t convU8toU32(uint8_t *src)
{
	uint32_t tmpData;

	tmpData = (uint32_t) (((src[3] & 0xFF) << 24) +
		((src[2] & 0xFF) << 16) +
		((src[1] & 0xFF) << 8) +
		(src[0] & 0xFF));

	return tmpData;
}

int parseBinFile(struct fts_ts_info *info, uint8_t *data,
	int fw_size,
	struct FW_FTB_HEADER *fw_header,
	int keep_cx)
{
	int dimension, index;
	uint32_t temp;
	int file_type;

	/* start the parsing */
	index = 0;
	fw_header->signature = convU8toU32(&data[index]);
	if (fw_header->signature == FW_HEADER_FTB_SIGNATURE) {
		tsp_debug_info(info->dev,
			"[parseBinFile] FW Signature - ftb file\n");
		file_type = BIN_FTB;
	} else {
		tsp_debug_info(info->dev,
			"[parseBinFile] FW Signature - ftsxxx file. %08X\n",
			fw_header->signature);
		file_type = BIN_FTS256;
		return file_type;
	}

	index += FW_BYTES_ALLIGN;
#ifdef FTS_USE_FTB_1
	fw_header->ftb_ver = convU8toU32(&data[index]);
	if (fw_header->ftb_ver != FW_FTB_VER) {
		 tsp_debug_err(info->dev,
			 "[parseBinFile] Wrong" " ftb_version %08X ... ERROR\n",
			 fw_header->ftb_ver);
		return false;
	}
#endif /* FTS_USE_FTB_1 */

	index += FW_BYTES_ALLIGN;
	fw_header->target = convU8toU32(&data[index]);
	if (fw_header->target != 0x00007036) {
		tsp_debug_err(info->dev,
			"[parseBinFile] Wrong target version %08X ... ERROR\n",
			fw_header->target);
		return false;
	}

	index += FW_BYTES_ALLIGN;
	fw_header->fw_id = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->fw_ver = convU8toU32(&data[index]);
	info->fw_version_of_bin = fw_header->fw_ver;

	index += FW_BYTES_ALLIGN;
	fw_header->cfg_id = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->cfg_ver = convU8toU32(&data[index]);
	info->config_version_of_bin = fw_header->cfg_ver;

	index += FW_BYTES_ALLIGN * 3; /* skip 2 reserved data */
	fw_header->bl_fw_ver = convU8toU32(&data[index]);
	index += FW_BYTES_ALLIGN;

	fw_header->ext_ver = convU8toU32(&data[index]);

	tsp_debug_info(info->dev,
		"[parseBinFile] Version : External"
		" = %04X, FW = %04X, CFG = %04X\n",
		fw_header->ext_ver,
		fw_header->fw_ver,
		fw_header->cfg_ver);

	index += FW_BYTES_ALLIGN;
	fw_header->sec0_size = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->sec1_size = convU8toU32(&data[index]);

	tsp_debug_info(info->dev,
		"[parseBinFile] sec0_size = %08X"
		" (%d bytes), sec1_size = %08X (%d bytes)\n",
		fw_header->sec0_size,
		fw_header->sec0_size,
		fw_header->sec1_size,
		fw_header->sec1_size);

	index += FW_BYTES_ALLIGN;
	fw_header->sec2_size = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->sec3_size = convU8toU32(&data[index]);

	tsp_debug_info(info->dev,
		"[parseBinFile] sec2_size = %08X"
		" (%d bytes), sec3_size = %08X (%d bytes)\n",
		fw_header->sec2_size,
		fw_header->sec2_size,
		fw_header->sec3_size,
		fw_header->sec3_size);

	index += FW_BYTES_ALLIGN;
	fw_header->hdr_crc = convU8toU32(&data[index]);

	if (!keep_cx) {
		dimension = fw_header->sec0_size + fw_header->sec1_size
			+ fw_header->sec2_size + fw_header->sec3_size;

		temp = fw_size;
	} else {
		/* sec2 may contain cx data (future implementation)
		 * sec3 atm not used */
		dimension = fw_header->sec0_size + fw_header->sec1_size;
		temp = fw_size - fw_header->sec2_size - fw_header->sec3_size;
	}

	if (dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN != temp) {
		tsp_debug_info(info->dev,
			"[parseBinFile] Read only %d"
			" instead of %d... ERROR\n",
			fw_size, dimension
			+ FW_HEADER_SIZE
			+ FW_BYTES_ALLIGN);

		return false;
	}

	return file_type;
}

static int fts_check_erase_done(struct fts_ts_info *info)
{
	int timeout = 60;  /* 3 sec timeout */
	unsigned char addrs[2] = {0xF9, 0x02};
	unsigned char val[1];
	int rc = 0;

	do {
		info->fts_read_reg(info, &addrs[0], 2, (unsigned char *)val, 1);

		if ((val[0] & 0x80) != 0x80)
			break;

		fts_delay(50);
		timeout--;
	} while (timeout != 0);

	if (timeout == 0)
		rc = -1;

	return rc;
}

int fw_download(struct fts_ts_info *info, uint8_t *pFilename,
		struct FW_FTB_HEADER *fw_Header, int8_t block_type)
{
	uint32_t FTS_TOTAL_SIZE = (256 * 1024); /* Total 256kB */
	int HEADER_DATA_SIZE = 32;

	int res = 0, rc = 0, i = 0;
	uint8_t addrs[8] = {0};

	/* System reset */
	/* System Reset ==> F7 52 34 */

	addrs[0] = 0xF7;
	addrs[1] = 0x52;
	addrs[2] = 0x34;
	info->fts_write_reg(info, &addrs[0], 3);
	fts_delay(30);

	/* Unlock Flash */
	/* Unlock Flash Command ==> F7 74 45 */
	addrs[0] = 0xF7;
	addrs[1] = 0x74;
	addrs[2] = 0x45;
	info->fts_write_reg(info, &addrs[0], 3);
	fts_delay(100);

	/* Unlock Erase Operation */
	addrs[0] = 0xFA;
	addrs[1] = 0x72;
	addrs[2] = 0x01;
	info->fts_write_reg(info, &addrs[0], 3);
	fts_delay(30);

	/* Erase Partial Flash */
	for (i = 0; i < 64; i++) {
		/* skip CX2 area (page 61 and page 62) */
		if ((i == 61) || (i == 62))
			continue;

		addrs[0] = 0xFA;
		addrs[1] = 0x02;
		addrs[2] = (0x80 + i) & 0xFF;
		info->fts_write_reg(info, &addrs[0], 3);
		rc = fts_check_erase_done(info);
		if (rc < 0)
			return rc;
	}

	/* Unlock Programming operation */
	addrs[0] = 0xFA;
	addrs[1] = 0x72;
	addrs[2] = 0x02;
	info->fts_write_reg(info, &addrs[0], 3);
	fts_delay(100);

	/* Write to FLASH */
	if (block_type == BIN_FTB) {
		tsp_debug_info(info->dev,
			"[fw_download] Start sec0 program\n");

		res = fillFlash(info,
			FLASH_ADDR_CODE,
			&pFilename[FW_HEADER_SIZE],
			fw_Header->sec0_size);

		if (res != true) {
			tsp_debug_err(info->dev,
				"[fw_download] Error - load sec0 program\n");
			return false;
		}
		tsp_debug_info(info->dev,
			"[fw_download] load sec0 program DONE!\n");
		tsp_debug_info(info->dev,
			"[fw_download] Start sec1 program\n");

		res = fillFlash(info,
			FLASH_ADDR_CONFIG,
			&pFilename[FW_HEADER_SIZE +
			fw_Header->sec0_size],
			fw_Header->sec1_size);

		if (res != true) {
			tsp_debug_err(info->dev,
				"[fw_download] Error - load sec1 program\n");
			return false;
		}
		tsp_debug_info(info->dev,
				"[fw_download] load sec1 program DONE!\n");

		tsp_debug_info(info->dev,
				"[fw_download] Flash burn COMPLETED!\n");
	} else {
		tsp_debug_info(info->dev,
				"[fw_download] Start firmware downloading\n");
		res = fillFlash(info, FLASH_ADDR_CODE,
				&pFilename[HEADER_DATA_SIZE], FTS_TOTAL_SIZE);
		if (res != true) {
			tsp_debug_err(info->dev,
				"[fw_download] Error - load sec0 program\n");
			return false;
		}
	}

	/* System reset  */
	addrs[0] = 0xF7;
	addrs[1] = 0x52;
	addrs[2] = 0x34;
	info->fts_write_reg(info, &addrs[0], 3);
	if (fts_cmd_completion_check(info, 0x10, 0x00, 0x00) < 0) {
		tsp_debug_err(info->dev,
			"[fw_download] Error - System Reset FAILED\n");
		return false;
	}

	return true;
}

static int fts_fw_compare(struct fts_ts_info *info, const struct firmware *fw)
{
	u32 bin_fw_ver_addr_1 = 0;
	u32 bin_fw_ver_addr_2 = 0;
	u32 bin_fw_ver_offset = 24;
	u8 buf[2] = {0};
	struct fts_version *binary = NULL;
	struct fts_version *device = &info->ic_fw_ver;
	int update = 0;

	if ((u32)fw->size < bin_fw_ver_offset) {
		tsp_debug_err(info->dev,
			" fw->size(0x%08X) < bin_fw_ver_offset(0x%08X)\n",
			(u32)fw->size, bin_fw_ver_offset);
		update = 0;
		goto error;
	}

	bin_fw_ver_addr_1 = (u32)fw->size - bin_fw_ver_offset;
	bin_fw_ver_addr_2 = bin_fw_ver_addr_1 + 1;
	tsp_debug_info(info->dev,
		"%s: bin_fw_ver_addr_1 = 0x%08X , bin_fw_ver_addr_2 = 0x%08X\n",
		__func__, bin_fw_ver_addr_1, bin_fw_ver_addr_2);

	binary = kzalloc(sizeof(struct fts_version), GFP_KERNEL);
	if (binary == NULL) {
		tsp_debug_err(info->dev, "failed to kzalloc binary\n");
		update = 0;
		goto error;
	}

	buf[0] = fw->data[bin_fw_ver_addr_1];
	buf[1] = fw->data[bin_fw_ver_addr_2];

	binary->build = (buf[0] >> 4) & 0x0F;
	binary->major = buf[0] & 0x0F;
	binary->minor = buf[1];

	if (binary->major != device->major) {
		update = 1;
	} else {
		if (binary->minor != device->minor)
			update = 1;
		else if (binary->build > device->build)
			update = 1;
	}

	tsp_debug_info(info->dev,
			"%s : binary[%d.%02d.%d] device[%d.%02d.%d]"
			" -> update: %d\n", __func__,
			binary->major, binary->minor, binary->build,
			device->major, device->minor, device->build,
			update);

error:
	if (binary)
		kfree(binary);

	return update;
}

void fts_fw_init(struct fts_ts_info *info)
{
	tsp_debug_info(info->dev, "%s\n", __func__);

	info->fts_command(info, FTS_CMD_TRIM_LOW_POWER_OSCILLATOR);
	fts_delay(200);
	info->fts_command(info, FTS_CMD_SAVE_CX_TUNING);
	fts_delay(230);
	fts_fw_wait_for_event(info, STATUS_EVENT_FLASH_WRITE_CXTUNE_VALUE, 0x00);

	fts_get_afe_info(info);

	fts_execute_autotune(info);

	info->fts_command(info, SENSEON);

	fts_fw_wait_for_event(info, STATUS_EVENT_FORCE_CAL_DONE, 0x00);

	info->fts_interrupt_set(info, INT_ENABLE);
}

static int fts_fw_check(struct fts_ts_info *info)
{
	int retval = 0;

	retval = fts_systemreset(info);
	if (retval < 0)
		return retval;

	retval = fts_wait_for_ready(info);
	if (retval < 0)
		return retval;

	retval = fts_read_chip_id(info);
	if (retval < 0)
		return retval;

	return retval;
}

int fts_fw_update(struct fts_ts_info *info)
{
	const struct firmware *fw_entry = NULL;
	unsigned char *fw_data = NULL;
	char fw_path[FTS_MAX_FW_PATH];
	const struct FW_FTB_HEADER *header;
	int fw_size;
	int fw_type;
	int keep_cx;
	int retval = 0;
	struct FW_FTB_HEADER fw_ftbHeader;

	if (info->fts_power_state != FTS_POWER_STATE_ACTIVE) {
		tsp_debug_err(info->dev,
			"%s : FTS_POWER_STATE is not ACTIVE\n", __func__);
		return -EPERM;
	}

	if (info->test_fwpath[0]) {
		strlcpy(fw_path, &info->test_fwpath[0], sizeof(fw_path));
	} else if(info->board->firmware_name) {
		/* A pointer and size of buffer for binary file */
		strlcpy(fw_path, &info->board->firmware_name[0], sizeof(fw_path));
	} else {
		tsp_debug_err(info->dev, "%s : no firmware file\n", __func__);
		return -EPERM;
	}

	tsp_debug_info(info->dev,
		"%s : firmware name : %s\n", __func__, fw_path);

	retval = request_firmware(&fw_entry, fw_path, info->dev);
	if (retval) {
		tsp_debug_err(info->dev,
			"%s : Firmware image %s not available\n", __func__,
			fw_path);
		return retval;
	}

	if (!fts_fw_compare(info, fw_entry)) {
		tsp_debug_info(info->dev,
			"%s : skip fw_upgrade(ic_fw_ver == bin_fw_ver)\n",
			__func__);
		goto out;
	}

	fw_size = fw_entry->size;
	fw_data = (unsigned char *)fw_entry->data;
	header = (struct FW_FTB_HEADER *)fw_data;

	info->fw_version_of_bin = header->fw_ver;
	info->config_version_of_bin = header->cfg_ver;
	 /* saver previous afe version before downloading */
	info->o_afe_ver = info->afe_ver;
#ifdef FTS_FTB_STYLE_2
	info->fw_main_version_of_bin =
	((header->ext_ver & 0xff)<<8) +
	((header->ext_ver >> 8) & 0xff);

	tsp_debug_info(info->dev,
				"Bin Firmware Version : 0x%04X "
				"Bin Config Version : 0x%04X "
				"Bin Main Firmware Version : 0x%04X ",
				info->fw_version_of_bin,
				info->config_version_of_bin,
				info->fw_main_version_of_bin);
#else /* FTS_FTB_STYLE_2 */
	tsp_debug_info(info->dev,
				"Bin Firmware Version : 0x%04X "
				"Bin Config Version : 0x%04X ",
				info->fw_version_of_bin,
				info->config_version_of_bin);
#endif
	memcpy(&fw_ftbHeader, fw_data, sizeof(struct FW_FTB_HEADER));

	tsp_debug_info(info->dev,
			"[flashProcedure] Firmware size : %d\n", fw_size);
	keep_cx = 0;

	fw_type = parseBinFile(info, fw_data, fw_size, &fw_ftbHeader, keep_cx);
	if (fw_type == false) {
		tsp_debug_err(info->dev,
			"[flashProcedure] Error - FW is not appreciate\n");
		retval = -EINVAL;
		goto out;
	}

	retval = fw_download(info, fw_data, &fw_ftbHeader, fw_type);
	if (retval == 0) {
		tsp_debug_err(info->dev,
			"[flashProcedure] Error - Firmware update is not completed.\n");
		retval = -EIO;
		goto out;
	}

	fts_fw_init(info);
	retval = fts_fw_check(info);
	if (retval < 0 ||
		info->flash_corruption_info.fw_broken ||
		info->flash_corruption_info.cfg_broken ||
		info->flash_corruption_info.cx_broken) {
		retval = -EIO;
		goto out;
	}

	fts_get_version_info(info);
	if (fts_fw_compare(info, fw_entry)) {
		tsp_debug_err(info->dev,
			"[flashProcedure] Firmware update failed\n");
		retval = -EIO;
		goto out;
	}

	tsp_debug_info(info->dev,
			"[flashProcedure] Firmware update is done successfully.\n");
	retval = 0;
out:
	if (fw_entry)
		release_firmware(fw_entry);
	return retval;
}
EXPORT_SYMBOL(fts_fw_update);

int fts_fw_verify_update(struct fts_ts_info *info)
{
	int retry = 0;

	info->fts_irq_enable(info, false);
	while (retry++ < FTS_FW_UPDATE_RETRY) {
		tsp_debug_info(info->dev,
			"[fw_update] try:%d\n", retry);
		if (0 == fts_fw_update(info)) {
			info->fts_irq_enable(info, true);
			return 0;
		}
	}
	info->fts_irq_enable(info, true);
	return -EIO;
}
EXPORT_SYMBOL(fts_fw_verify_update);
