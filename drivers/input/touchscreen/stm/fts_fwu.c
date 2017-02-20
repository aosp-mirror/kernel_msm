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

#include "fts_ts.h"

#define FTS_DEFAULT_FW "/sdcard/stm.fw"
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

int fts_fw_wait_for_specific_event(struct fts_ts_info *info, unsigned char eid0, unsigned char eid1, unsigned char eid2)
{
	int rc;
	unsigned char regAdd;
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	regAdd = READ_ONE_EVENT;
	rc = -1;
	while (info->fts_read_reg(info, &regAdd, 1, (unsigned char *)data, FTS_EVENT_SIZE)) {
		if (data[0]) {
			if ((data[0] == eid0) && (data[1] == eid1) && (data[2] == eid2)) {
				rc = 0;
				break;
			} else {
				tsp_debug_info(true, info->dev, "%s: %2X,%2X,%2X,%2X\n", __func__, data[0],data[1],data[2],data[3]);
			}
		}
		if (retry++ > FTS_RETRY_COUNT * 15) {
			rc = -1;
			tsp_debug_info(true, info->dev, "%s: Time Over ( %2X,%2X,%2X,%2X )\n", __func__, data[0],data[1],data[2],data[3]);
			break;
		}
		msleep(20);
	}

	return rc;
}

int fts_fw_wait_for_event(struct fts_ts_info *info, unsigned char eid)
{
	int rc;
	unsigned char regAdd;
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	regAdd = READ_ONE_EVENT;
	rc = -1;
	while (info->fts_read_reg(info, &regAdd, 1, (unsigned char *)data, FTS_EVENT_SIZE)) {
		if ((data[0] == EVENTID_STATUS_EVENT) &&
			(data[1] == eid)) {
			rc = 0;
			break;
		}

		if (retry++ > FTS_RETRY_COUNT * 15) {
			rc = -1;
			tsp_debug_info(true, info->dev, "%s: Time Over\n", __func__);
			break;
		}
		msleep(20);
	}

	return rc;
}

void fts_execute_autotune(struct fts_ts_info *info)
{
	info->fts_command(info, CX_TUNNING);
	msleep(300);
	fts_fw_wait_for_event(info, STATUS_EVENT_MUTUAL_AUTOTUNE_DONE);

#ifdef FTS_SUPPORT_WATER_MODE
	fts_fw_wait_for_event (info, STATUS_EVENT_WATER_SELF_AUTOTUNE_DONE);
	fts_fw_wait_for_event(info, STATUS_EVENT_SELF_AUTOTUNE_DONE);
#endif
#ifdef FTS_SUPPORT_SELF_MODE
	info->fts_command(info, SELF_AUTO_TUNE);
	msleep(300);
	fts_fw_wait_for_event(info, STATUS_EVENT_SELF_AUTOTUNE_DONE);
#endif

	info->fts_command(info, FTS_CMD_SAVE_CX_TUNING);
	msleep(230);
	fts_fw_wait_for_event(info, STATUS_EVENT_FLASH_WRITE_CXTUNE_VALUE);
}

#define FW_IMAGE_NAME_D2_TB_INTEG			"tsp_stm/stm_tb_integ.fw"
#define FW_IMAGE_NAME_D2_Z2A			"tsp_stm/stm_z2a.fw"
#define FW_IMAGE_NAME_D2_Z2I			"tsp_stm/stm_z2i.fw"
#define CONFIG_ID_D1_S				0x2C
#define CONFIG_ID_D2_TR				0x2E
#define CONFIG_ID_D2_TB				0x30
#define CONFIG_OFFSET_BIN_D1			0xf822
#define CONFIG_OFFSET_BIN_D2			0x1E822
#define RX_OFFSET_BIN_D2				0x1E834
#define TX_OFFSET_BIN_D2				0x1E835

#define	FW_IMAGE_SIZE_D3	(256 * 1024)
#define	SIGNEDKEY_SIZE		(256)

#define FTS_FIFO_ADDR						0x85

static uint8_t fts_fifo_addr[2] = {FTS_FIFO_ADDR, 0};

int fts_cmd_completion_check(struct fts_ts_info *info, uint8_t event1, uint8_t event2, uint8_t event3)
{
	uint8_t val[8];
	int retry = 100;

	while (retry--) {
		msleep(10);

		info->fts_read_reg(info, fts_fifo_addr, 1, (uint8_t *) val, FTS_EVENT_SIZE);
		if ((val[0] == event1) && (val[1] == event2) && (val[2] == event3))
		{
			tsp_debug_info(true, info->dev, "\n\r[fts_cmd_completion_check] OK [%02x][%02x][%02x]", val[0], val[1], val[2]);
			return 1;
		}
		else if (val[0] == 0x0F)
		{
			tsp_debug_info(true, info->dev, "\n\r[fts_cmd_completion_check] Error - [%02x][%02x][%02x]", val[0], val[1], val[2]);
		}
	}
	if (retry <= 0)
		tsp_debug_info(true, info->dev, "\n\r[fts_cmd_completion_check] Error - Time Over [%02x][%02x][%02x]", event1, event2, event3);

	return 0;
}

int wait_for_flash_ready(struct fts_ts_info *info, uint8_t type)
{
	uint8_t	cmd[2] = {FLASH_CMD_READ_REGISTER, type};
	uint8_t	readData;
	int	i, res = -1;

	tsp_debug_info(true, info->dev, "\n\r[wait_for_flash_ready] Waiting for flash ready");
	for (i = 0; i < 1000 && res != 0; i++) {
		info->fts_read_reg(info, cmd, sizeof(cmd), &readData, 1);
		res = readData & 0x80;
		msleep(50);
	}

	if (i >= 1000 && res != 0) {
		tsp_debug_info(true, info->dev, "\n\r[wait_for_flash_ready] Wait for flash TIMEOUT! ERROR");
		return	0;
	}

	tsp_debug_info(true, info->dev, "\n\r[wait_for_flash_ready] Flash READY!");
	return	1;
}

int start_flash_dma(struct fts_ts_info *info)
{
	int	status;
	uint8_t	cmd[3] = {FLASH_CMD_WRITE_REGISTER, FLASH_DMA_CODE0, FLASH_DMA_CODE1};

	tsp_debug_info(true, info->dev, "\n\r[start_flash_dma] Command flash DMA ...");
	info->fts_write_reg(info, cmd, sizeof(cmd));

	status = wait_for_flash_ready(info, FLASH_DMA_CODE0);

	if (status != true) {
		tsp_debug_info(true, info->dev, "\n\r[start_flash_dma] start_flash_dma: ERROR");
		return	false;
	}
	tsp_debug_info(true, info->dev, "\n\r[start_flash_dma] flash DMA DONE!");

	return	true;
}

int fillFlash(struct fts_ts_info *info, uint32_t address, uint8_t *data, int size)
{
	int		remaining;
	int		toWrite = 0;
	int		byteBlock = 0;
	int		wheel = 0;
	uint32_t	addr = 0;
	int		res;
	int		delta;

	uint8_t		buff[DMA_CHUNK + 3] = {0};

	remaining = size;
	while (remaining > 0)
	{
		byteBlock = 0;
		addr =0;
		tsp_debug_info(true, info->dev, "\n\r[fillFlash] [%d] Write data to memory.", wheel);
		while (byteBlock < FLASH_CHUNK && remaining > 0)
		{
			buff[0] = FLASH_CMD_WRITE_64K;
			if (remaining >= DMA_CHUNK)
			{
				if ((byteBlock + DMA_CHUNK) <= FLASH_CHUNK)
				{
					toWrite = DMA_CHUNK;
					remaining -= DMA_CHUNK;
					byteBlock += DMA_CHUNK;
				}
				else
				{
					delta = FLASH_CHUNK - byteBlock;
					toWrite = delta;
					remaining -= delta;
					byteBlock += delta;
				}
			}
			else
			{
				if ((byteBlock + remaining) <= FLASH_CHUNK)
				{
					toWrite = remaining;
					byteBlock += remaining;
					remaining = 0;

				}
				else
				{
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

		//configuring the DMA
		tsp_debug_info(true, info->dev, "\n\r[fillFlash] [%d] Configure DMA", wheel);
		byteBlock = byteBlock / 4 - 1;

		buff[0] = FLASH_CMD_WRITE_REGISTER;
		buff[1] = FLASH_DMA_CONFIG;
		buff[2] = 0x00;
		buff[3] = 0x00;

		addr = address + ((wheel * FLASH_CHUNK)/4);
		buff[4] = (uint8_t) ((addr & 0x000000FF));
		buff[5] = (uint8_t) ((addr & 0x0000FF00) >> 8);
		buff[6] = (uint8_t) (byteBlock & 0x000000FF);
		buff[7] = (uint8_t) ((byteBlock & 0x0000FF00)>> 8);
		buff[8] = 0x00;

		info->fts_write_reg(info, buff, 9);
		msleep(10);

		tsp_debug_info(true, info->dev, "\n\r[fillFlash] [%d] Start flash DMA", wheel);
		res = start_flash_dma(info);
		if (res < true) {
			tsp_debug_info(true, info->dev, "\n\r[fillFlash] Error during flashing DMA! ERROR");
			return	false;
		}
		tsp_debug_info(true, info->dev, "\n\r[fillFlash] [%d] DMA done", wheel);

		wheel++;
	}
	return	true;
}

uint32_t convU8toU32(uint8_t *src)
{
	uint32_t	tmpData;

	tmpData = (uint32_t) (((src[3] & 0xFF) << 24) + ((src[2] & 0xFF) << 16) + ((src[1] & 0xFF) << 8) + (src[0] & 0xFF));

	return	tmpData;
}

int parseBinFile(struct fts_ts_info *info, uint8_t *data, int fw_size,struct FW_FTB_HEADER *fw_header, int keep_cx)
{
	int			dimension, index;
	uint32_t	temp;
	int			file_type;

	/* start the parsing */
	index = 0;
	fw_header->signature = convU8toU32(&data[index]);
	if (fw_header->signature == FW_HEADER_FTB_SIGNATURE)
	{
		tsp_debug_info(true, info->dev, "\n\r[parseBinFile] FW Signature - ftb file");
		file_type = BIN_FTB;
	}
	else
	{
		tsp_debug_info(true, info->dev, "\n\r[parseBinFile] FW Signature - ftsxxx file. %08X", fw_header->signature);
		file_type = BIN_FTS256;
		return	file_type;
	}

	index += FW_BYTES_ALLIGN;
	fw_header->ftb_ver = convU8toU32(&data[index]);
	if (fw_header->ftb_ver != FW_FTB_VER)
	{
		 tsp_debug_info(true, info->dev, "\n\r[parseBinFile] Wrong ftb_version %08X ... ERROR", fw_header->ftb_ver);
		return	false;
	}

	index += FW_BYTES_ALLIGN;
	fw_header->target = convU8toU32(&data[index]);
	if (fw_header->target != 0x00007036)
	{
		tsp_debug_info(true, info->dev, "\n\r[parseBinFile] Wrong target version %08X ... ERROR", fw_header->target);
		return	false;
	}

	index += FW_BYTES_ALLIGN;
	fw_header->fw_id = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->fw_ver = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->cfg_id = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->cfg_ver = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN * 3;			// skip 2 reserved data

	fw_header->ext_ver = convU8toU32(&data[index]);
	tsp_debug_info(true, info->dev, "\n\r[parseBinFile] Version : External = %04X, FW = %04X, CFG = %04X", fw_header->ext_ver, fw_header->fw_ver, fw_header->cfg_ver);

	index += FW_BYTES_ALLIGN * 2;
	fw_header->sec0_size = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->sec1_size = convU8toU32(&data[index]);
	tsp_debug_info(true, info->dev, "\n\r[parseBinFile] sec0_size = %08X (%d bytes), sec1_size = %08X (%d bytes), ", fw_header->sec0_size, fw_header->sec0_size, fw_header->sec1_size, fw_header->sec1_size);

	index += FW_BYTES_ALLIGN;
	fw_header->sec2_size = convU8toU32(&data[index]);

	index += FW_BYTES_ALLIGN;
	fw_header->sec3_size = convU8toU32(&data[index]);
	tsp_debug_info(true, info->dev, "\n\r[parseBinFile] sec2_size = %08X (%d bytes), sec3_size = %08X (%d bytes)", fw_header->sec2_size, fw_header->sec2_size, fw_header->sec3_size, fw_header->sec3_size);

	index += FW_BYTES_ALLIGN;
	fw_header->hdr_crc = convU8toU32(&data[index]);

	if (!keep_cx)
	{
		dimension = fw_header->sec0_size + fw_header->sec1_size + fw_header->sec2_size + fw_header->sec3_size;
		temp = fw_size;
	}
	else
	{
		//sec2 may contain cx data (future implementation) sec3 atm not used
		dimension = fw_header->sec0_size + fw_header->sec1_size;
		temp = fw_size - fw_header->sec2_size - fw_header->sec3_size;
	}

	if (dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN != temp)
	{
		tsp_debug_info(true, info->dev, "\n\r[parseBinFile] Read only %d instead of %d... ERROR", fw_size, dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN);
		return	false;
	}

	return	file_type;
}

int fw_download(struct fts_ts_info *info, uint8_t *pFilename,struct FW_FTB_HEADER *fw_Header, int8_t block_type)
{
	uint32_t	FTS_TOTAL_SIZE = (256 * 1024);	// Total 256kB
	int			HEADER_DATA_SIZE = 32;

	int			res;
	uint8_t		regAdd[8] = {0};

	//==================== System reset ====================
	regAdd[0] = 0xF7;		regAdd[1] = 0x52;		regAdd[2] = 0x34;
	info->fts_write_reg(info, &regAdd[0],3);
	msleep(30);

	//==================== Warm Boot ====================
	regAdd[0] = 0xB6;	regAdd[1] = (ADDR_WARM_BOOT >> 8) & 0xFF;	regAdd[2] = ADDR_WARM_BOOT & 0xFF;	regAdd[3] = WARM_BOOT_VALUE;
	info->fts_write_reg(info, &regAdd[0],4);
	msleep(30);

	//==================== Unlock Flash ====================
	regAdd[0] = 0xF7;		regAdd[1] = 0x74;		regAdd[2] = 0x45;
	info->fts_write_reg(info, &regAdd[0],3);
	msleep(30);

	//==================== Unlock Erase & Programming Operation ====================
	regAdd[0] = 0xFA;		regAdd[1] = 0x72;		regAdd[2] = 0x03;
	info->fts_write_reg(info, &regAdd[0],3);
	msleep(30);

	//==================== Erase full Flash ====================
	regAdd[0] = 0xFA;		regAdd[1] = 0x02;		regAdd[2] = 0xC0;
	info->fts_write_reg(info, &regAdd[0],3);
	msleep(200);

	//========================== Write to FLASH ==========================
	if (block_type == BIN_FTB)
	{
		tsp_debug_info(true, info->dev, "\n\r[fw_download] Start sec0 program");
		res = fillFlash(info, FLASH_ADDR_CODE, &pFilename[FW_HEADER_SIZE], fw_Header->sec0_size);
		if (res != true)
		{
			tsp_debug_info(true, info->dev, "\n\r[fw_download] Error - load sec0 program");
			return	false;
		}
		tsp_debug_info(true, info->dev, "\n\r[fw_download] load sec0 program DONE!");
		tsp_debug_info(true, info->dev, "\n\r[fw_download] Start sec1 program");
		res = fillFlash(info, FLASH_ADDR_CONFIG, &pFilename[FW_HEADER_SIZE + fw_Header->sec0_size], fw_Header->sec1_size);
		if (res != true)
		{
			tsp_debug_info(true, info->dev, "\n\r[fw_download] Error - load sec1 program");
			return	false;
		}
		tsp_debug_info(true, info->dev, "\n\r[fw_download] load sec1 program DONE!");

		tsp_debug_info(true, info->dev, "\n\r[fw_download] Flash burn COMPLETED!");
	}
	else
	{
		tsp_debug_info(true, info->dev, "\n\r[fw_download] Start firmware downloading");
		res = fillFlash(info, FLASH_ADDR_CODE, &pFilename[HEADER_DATA_SIZE], FTS_TOTAL_SIZE);
		if (res != true)
		{
			tsp_debug_info(true, info->dev, "\n\r[fw_download] Error - load sec0 program");
			return	false;
		}
	}

	//==================== System reset ====================
	regAdd[0] = 0xF7;		regAdd[1] = 0x52;		regAdd[2] = 0x34;
	info->fts_write_reg(info, &regAdd[0],3);
	if (fts_cmd_completion_check(info, 0x10, 0x00, 0x00) == false)
	{
		tsp_debug_info(true, info->dev, "\n\r[fw_download] Error - System Reset FAILED");
		return	false;
	}

	return true;
}


int fts_fw_update(struct fts_ts_info *info)
{
	const struct firmware *fw_entry = NULL;
	unsigned char *fw_data	= NULL;
	char fw_path[FTS_MAX_FW_PATH];
	//uint8_t			*pFilename = NULL;
	int				fw_size;
	int				status, fw_type;
	int 			keep_cx;
	int 			retval = 0;

	struct FW_FTB_HEADER	fw_ftbHeader;

	if(info->fts_power_state != FTS_POWER_STATE_ACTIVE) {
		tsp_debug_info(true, info->dev,"%s : FTS_POWER_STATE is not ACTIVE \n", __func__);

		return false;
	}
	if(info->force_update)
		info->firmware_name = info->test_fwpath;
	else{
		/* A pointer and size of buffer for binary file */
		if (info->board->firmware_name)
			info->firmware_name = info->board->firmware_name;
		else
			info->firmware_name = FTS_DEFAULT_FW;
	}

	snprintf(fw_path, FTS_MAX_FW_PATH, "%s", info->firmware_name);
	tsp_debug_info(true, info->dev, " %s : firmware name : %s \n", __func__, info->firmware_name);

	retval = request_firmware(&fw_entry, fw_path, info->dev);

	if(retval){
		tsp_debug_info(true, info->dev,"%s : Firmware image %s not available \n", __func__, fw_path);
		return false;
	}

	fw_size = fw_entry->size;
	fw_data = (unsigned char *)fw_entry->data;
	memcpy(&fw_ftbHeader, fw_data, sizeof(struct FW_FTB_HEADER));

	tsp_debug_info(true, info->dev, "\n\r[flashProcedure] Firmware size : %d", fw_size);
	keep_cx = 0;

	fw_type = parseBinFile(info, fw_data, fw_size, &fw_ftbHeader, keep_cx);
	if (fw_type == false)
	{
		tsp_debug_info(true, info->dev, "\n\r[flashProcedure] Error - FW is not appreciate");
		return	false;
	}

	status = fw_download(info, fw_data, &fw_ftbHeader, fw_type);
	if (status == false)
	{
		tsp_debug_info(true, info->dev, "\n\r[flashProcedure] Error - Firmware update is not completed.");
		return	false;
	}

	tsp_debug_info(true, info->dev, "\n\r[flashProcedure] Firmware update is done successfully.");

	return status;
}
EXPORT_SYMBOL(fts_fw_update);
