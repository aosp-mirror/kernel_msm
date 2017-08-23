/* drivers/input/touchscreen/it7258_ts_i2c.c
 *
 * Copyright (C) 2014 ITE Tech. Inc.
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include <linux/input/mt.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#ifdef	CONFIG_BEZEL_SUPPORT
#include "input_bezel.h"
#include "it7259_ts_i2c.h"
#endif

#define MAX_BUFFER_SIZE			144
#define DEVICE_NAME			"it7259"
#define SCREEN_X_RESOLUTION		320
#define SCREEN_Y_RESOLUTION		320
#define DEBUGFS_DIR_NAME		"ts_debug"
#define FW_NAME				"it7259_fw.bin"
#define CFG_NAME			"it7259_cfg.bin"
#define IT7259_CFG_PATH                 "/system/etc/firmware/it7259.cfg"
#define IT7259_FW_PATH                  "/system/etc/firmware/it7259.fw"
#define VER_BUFFER_SIZE			4
#define IT_FW_CHECK(x, y) \
	(((x)[0] < (y)->data[8]) || ((x)[1] < (y)->data[9]) || \
	((x)[2] < (y)->data[10]) || ((x)[3] < (y)->data[11]))
#define IT_CFG_CHECK(x, y) \
	(((x)[0] < (y)->data[(y)->size - 8]) || \
	((x)[1] < (y)->data[(y)->size - 7]) || \
	((x)[2] < (y)->data[(y)->size - 6]) || \
	((x)[3] < (y)->data[(y)->size - 5]))
#define it7259_COORDS_ARR_SIZE		4

/* all commands writes go to this idx */
#define BUF_COMMAND			0x20
#define BUF_SYS_COMMAND			0x40
/*
 * "device ready?" and "wake up please" and "read touch data" reads
 * go to this idx
 */
#define BUF_QUERY			0x80
/* most command response reads go to this idx */
#define BUF_RESPONSE			0xA0
#define BUF_SYS_RESPONSE		0xC0
/* reads of "point" go through here and produce 14 bytes of data */
#define BUF_POINT_INFO			0xE0

/*
 * commands and their subcommands. when no subcommands exist, a zero
 * is send as the second byte
 */
#define CMD_IDENT_CHIP			0x00
/* VERSION_LENGTH bytes of data in response */
#define CMD_READ_VERSIONS		0x01
#define SUB_CMD_READ_FIRMWARE_VERSION	0x00
#define SUB_CMD_READ_CONFIG_VERSION	0x06
#define VERSION_LENGTH			10
/* subcommand is zero, next byte is power mode */
#define CMD_PWR_CTL			0x04
/* active mode */
#define PWR_CTL_ACTIVE_MODE		0x00
/* idle mode */
#define PWR_CTL_LOW_POWER_MODE		0x01
/* sleep mode */
#define PWR_CTL_SLEEP_MODE		0x02
#define WAIT_CHANGE_MODE		50
/* command is not documented in the datasheet v1.0.0.7 */
#define CMD_UNKNOWN_7			0x07
#define CMD_FIRMWARE_REINIT_C		0x0C
/* needs to be followed by 4 bytes of zeroes */
#define CMD_CALIBRATE			0x13
#define CMD_FIRMWARE_UPGRADE		0x60
#define SUB_CMD_ENTER_FW_UPGRADE_MODE	0x00
#define SUB_CMD_EXIT_FW_UPGRADE_MODE	0x80
/* address for FW read/write */
#define CMD_SET_START_OFFSET		0x61
/* subcommand is number of bytes to write */
#define CMD_FW_WRITE			0x62
/* subcommand is number of bytes to read */
#define CMD_FW_READ			0x63
#define CMD_FIRMWARE_REINIT_6F		0x6F

#define FW_WRITE_CHUNK_SIZE		128
#define FW_WRITE_RETRY_COUNT		4
#define CHIP_FLASH_SIZE			0x8000
#define DEVICE_READY_COUNT_MAX		500
#define DEVICE_READY_COUNT_20		20
#define IT_I2C_WAIT_10MS		10
#define IT_I2C_READ_RET			2
#define IT_I2C_WRITE_RET		1

/* result of reading with BUF_QUERY bits */
#define CMD_STATUS_BITS			0x07
#define CMD_STATUS_DONE			0x00
#define CMD_STATUS_BUSY			0x01
#define CMD_STATUS_ERROR		0x02
#define CMD_STATUS_NO_CONN		0x07
#define PT_INFO_BITS			0xF8
#define PT_INFO_YES			0x80

#define PD_FLAGS_DATA_TYPE_BITS		0xF0
/* other types (like chip-detected gestures) exist but we do not care */
#define PD_FLAGS_DATA_TYPE_TOUCH	0x00
#define PD_FLAGS_IDLE_TO_ACTIVE		0x10
/* a bit for each finger data that is valid (from lsb to msb) */
#define PD_FLAGS_HAVE_FINGERS		0x07
#define PD_PALM_FLAG_BIT		0x01
#define FD_PRESSURE_BITS		0x0F
#define FD_PRESSURE_NONE		0x00
#define FD_PRESSURE_LIGHT		0x01

#define IT_VTG_MIN_UV		1800000
#define IT_VTG_MAX_UV		1800000
#define IT_ACTIVE_LOAD_UA	15000
#define IT_I2C_VTG_MIN_UV	2600000
#define IT_I2C_VTG_MAX_UV	3300000
#define IT_I2C_ACTIVE_LOAD_UA	10000
#define DELAY_VTG_REG_EN	170
#define DELAY_I2C_TRANSATION	200

#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"

int download;
#define COMMAND_SUCCESS         0x0000
#define COMMAND_ERROR     0x0200
#define ERROR_QUERY_TIME_OUT    0x0800

#define HOVER_Z_MAX (255)

struct finger_data {
	u8 xLo;
	u8 hi;
	u8 yLo;
	u8 pressure;
}  __packed;

struct point_data {
	u8 flags;
	u8 gesture_id;
	struct finger_data fd[3];
}  __packed;

struct it7259_ts_platform_data {
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	bool wakeup;
	bool palm_detect_en;
	u16 palm_detect_keycode;
	const char *fw_name;
	const char *cfg_name;
	unsigned int panel_minx;
	unsigned int panel_miny;
	unsigned int panel_maxx;
	unsigned int panel_maxy;
	unsigned int disp_minx;
	unsigned int disp_miny;
	unsigned int disp_maxx;
	unsigned int disp_maxy;
	unsigned num_of_fingers;
	unsigned int reset_delay;
	unsigned int avdd_lpm_cur;
	bool low_reset;
};

struct it7259_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct it7259_ts_platform_data *pdata;
#ifdef	CONFIG_BEZEL_SUPPORT
	struct bezel_data *bdata;
#endif
	struct regulator *vdd;
	struct regulator *avdd;
	bool in_low_power_mode;
	bool suspended;
	bool fw_upgrade_result;
	bool cfg_upgrade_result;
	bool fw_cfg_uploading;
	struct work_struct work_pm_relax;
	bool calibration_success;
	bool had_finger_down;
	char fw_name[MAX_BUFFER_SIZE];
	char cfg_name[MAX_BUFFER_SIZE];
	struct mutex fw_cfg_mutex;
	u8 fw_ver[VER_BUFFER_SIZE];
	u8 cfg_ver[VER_BUFFER_SIZE];
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
	struct dentry *dir;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
	bool palm_pressed;
	bool fw_active;
};

/* Function declarations */
static int fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data);
static int it7259_ts_resume(struct device *dev);
static int it7259_ts_suspend(struct device *dev);

static int it7259_debug_suspend_set(void *_data, u64 val)
{
	struct it7259_ts_data *ts_data = _data;

	if (val)
		it7259_ts_suspend(&ts_data->client->dev);
	else
		it7259_ts_resume(&ts_data->client->dev);

	return 0;
}

static int it7259_debug_suspend_get(void *_data, u64 *val)
{
	struct it7259_ts_data *ts_data = _data;

	mutex_lock(&ts_data->input_dev->mutex);
	*val = ts_data->suspended;
	mutex_lock(&ts_data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, it7259_debug_suspend_get,
				it7259_debug_suspend_set, "%lld\n");

/* internal use func - does not make sure chip is ready before read */
static int it7259_i2c_read_no_ready_check(struct it7259_ts_data *ts_data,
			uint8_t buf_index, uint8_t *buffer, uint16_t buf_len)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = ts_data->client->addr,
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = &buf_index
		},
		{
			.addr = ts_data->client->addr,
			.flags = I2C_M_RD,
			.len = buf_len,
			.buf = buffer
		}
	};

	memset(buffer, 0xFF, buf_len);

	ret = i2c_transfer(ts_data->client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&ts_data->client->dev, "i2c read failed %d\n", ret);

	return ret;
}

static int it7259_i2c_write_no_ready_check(struct it7259_ts_data *ts_data,
		uint8_t buf_index, const uint8_t *buffer, uint16_t buf_len)
{
	uint8_t txbuf[257];
	int ret;
	struct i2c_msg msg = {
		.addr = ts_data->client->addr,
		.flags = 0,
		.len = buf_len + 1,
		.buf = txbuf
	};

	/* just to be careful */
	if (buf_len > sizeof(txbuf) - 1) {
		dev_err(&ts_data->client->dev, "buf length is out of limit\n");
		return false;
	}

	txbuf[0] = buf_index;
	memcpy(txbuf + 1, buffer, buf_len);

	ret = i2c_transfer(ts_data->client->adapter, &msg, 1);
	if (ret < 0)
		dev_err(&ts_data->client->dev, "i2c write failed %d\n", ret);

	return ret;
}


/*
 * Device is apparently always ready for I2C communication but not for
 * actual register reads/writes. This function checks if it is ready
 * for that too. The results of this call often were ignored.
 * If forever is set to TRUE, then check the device's status until it
 * becomes ready with 500 retries at max. Otherwise retry 25 times only.
 * If slowly is set to TRUE, then add sleep of 50 ms in each retry,
 * otherwise don't sleep.
 */
static int it7259_wait_device_ready(struct it7259_ts_data *ts_data,
					bool forever, bool slowly)
{
	uint8_t query;
	uint32_t count = DEVICE_READY_COUNT_20;
	int ret;

	if (ts_data->fw_cfg_uploading || forever)
		count = DEVICE_READY_COUNT_MAX;

	do {
		ret = it7259_i2c_read_no_ready_check(ts_data, BUF_QUERY, &query,
						sizeof(query));
		if (ret < 0 && ((query & CMD_STATUS_BITS)
						== CMD_STATUS_NO_CONN))
			continue;

		if ((query & CMD_STATUS_BITS) == CMD_STATUS_DONE)
			break;

		query = CMD_STATUS_BUSY;
		if (slowly)
			msleep(IT_I2C_WAIT_10MS);
	} while (--count);

	return ((!(query & CMD_STATUS_BITS)) ? 0 : -ENODEV);
}

static int it7259_i2c_write(struct it7259_ts_data *ts_data, uint8_t buf_index,
                        const uint8_t *buffer, uint16_t buf_len)
{
        int ret;

        ret = it7259_wait_device_ready(ts_data, false, false);
        if (ret < 0)
                return ret;

        return it7259_i2c_write_no_ready_check(ts_data, buf_index,
                                        buffer, buf_len);
}


static int it7259_ts_chip_low_power_mode(struct it7259_ts_data *ts_data,
					const u8 sleep_type)
{
	const u8 cmd_sleep[] = {CMD_PWR_CTL, 0x00, sleep_type};
	u8 dummy;
	int ret;

	if (sleep_type) {
		ret = it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND,
					cmd_sleep, sizeof(cmd_sleep));
		if (ret != IT_I2C_WRITE_RET)
			dev_err(&ts_data->client->dev,
				"Can't go to sleep or low power mode(%d) %d\n",
				sleep_type, ret);
		else
			ret = 0;
	} else {
		ret = it7259_i2c_read_no_ready_check(ts_data, BUF_QUERY, &dummy,
						sizeof(dummy));
		if (ret != IT_I2C_READ_RET)
			dev_err(&ts_data->client->dev,
				"Can't go to active mode %d\n", ret);
		else
			ret = 0;
	}

	msleep(WAIT_CHANGE_MODE);
	return ret;
}





static int i2cInternalWriteToIT7259(struct it7259_ts_data *ts_data, int wAddress,
                unsigned char const dataBuffer[], unsigned short dataLength)
{
        unsigned char buffer4Write[1024];
        struct i2c_msg msgs[1] = { { .addr = ts_data->client->addr, .flags = 0, .len =
                        dataLength + 3, .buf = buffer4Write } };
        //printk("====in internal write function===\n");

        buffer4Write[0] = 0x70;
        buffer4Write[1] = (unsigned char)(wAddress &0xFF);

        memcpy(&(buffer4Write[2]), dataBuffer, dataLength);
        return i2c_transfer(ts_data->client->adapter, msgs, 1);
}

static int i2cDirectReadFromIT7259(struct it7259_ts_data *ts_data, int wAddress,
                unsigned char readDataBuffer[], unsigned short readDataLength)
{
	int ret;
	unsigned char buffer4Write[1024];
	struct i2c_msg msgs[2] = { { .addr = ts_data->client->addr, .flags = 0,
		.len = 4, .buf = buffer4Write }, { .addr = ts_data->client->addr, .flags =
			I2C_M_RD, .len = readDataLength, .buf = readDataBuffer } };
	//printk("====in Direct read function===\n");

	buffer4Write[0] = 0x90;
	buffer4Write[1] = 0x00;
	buffer4Write[2] = (unsigned char)((wAddress &0xFF00) >> 8);
	buffer4Write[3] = (unsigned char)(wAddress &0xFF);

	memset(readDataBuffer, 0xFF, readDataLength);
	ret = i2c_transfer(ts_data->client->adapter, msgs, 2);
	return ret;
}

static int i2cDirectWriteToIT7259(struct it7259_ts_data *ts_data, int wAddress,
                unsigned char const dataBuffer[], unsigned short dataLength)
{
        unsigned char buffer4Write[1024];
        int nRetryCount = 0;
        int nRet = 0;
        struct i2c_msg msgs[1] = { { .addr = ts_data->client->addr, .flags = 0, .len =
                        dataLength + 4, .buf = buffer4Write } };
        //printk("====in Direct write function===\n");

        buffer4Write[0] = 0x10;
        buffer4Write[1] = 0x00;
        buffer4Write[2] = (unsigned char)((wAddress &0xFF00) >> 8);
        buffer4Write[3] = (unsigned char)(wAddress &0xFF);
        memcpy(&(buffer4Write[4]), dataBuffer, dataLength);

        do
        {
                nRet = i2c_transfer(ts_data->client->adapter, msgs, 1);

        } while((nRet <= 0) && (nRetryCount++ < 10));

        return  nRet;
}


static bool gfnIT7259_SPIFCRReady(struct it7259_ts_data *ts_data)
{
        int nReadCount=0;
        unsigned char ucBuffer[2];

        do {
                i2cDirectReadFromIT7259(ts_data, 0xF400, ucBuffer, 2);//gfnIT7259_DirectReadMemoryRegister
        }while(((ucBuffer[1]& 0x01)!=0x00) && ++nReadCount<20 ); //nReadCount 3000

        if(nReadCount >=20) //nReadCount 3000
                return false;

        return true;
}



static int gfnIT7259_DirectReadFlash(struct it7259_ts_data *ts_data, int wFlashAddress, unsigned int readLength, unsigned char* pData)
{
        int nSector = 0;
        unsigned char pucCommandBuffer[1024];
        int wTmp;
        int wAddress;
        unsigned int /*AddrOffset, */LenOffset;
        unsigned char bufTemp[4];
        int wOffset;
        int i;

        nSector = wFlashAddress/0x0400;
        pucCommandBuffer[0] = nSector;

        //Select Sector
        wAddress = 0xF404;
        wTmp = i2cDirectWriteToIT7259(ts_data,wAddress,pucCommandBuffer,1);
        if(wTmp <= 0)
        {
                return COMMAND_ERROR;
        }

        //Wait SPIFCR
        if(!gfnIT7259_SPIFCRReady(ts_data))
        {
                return ERROR_QUERY_TIME_OUT;
        }

        //Read flash
        wOffset = wFlashAddress - (nSector*0x0400);
        wAddress = 0x3000 + wOffset;
        //printk("======= gfnIT7259_DirectReadFlash 8 byte limit =======\n");
        for(LenOffset=0; LenOffset < readLength; LenOffset+=4)  //for 8 byte limit
        {
                wTmp = i2cDirectReadFromIT7259(ts_data, wAddress, bufTemp, 4);
                if(wTmp <= 0)
                {
                        return COMMAND_ERROR;
                }

                for(i = 0; i < 4; i++)
                {
                        pucCommandBuffer[LenOffset + i] = bufTemp[i] ;
                }
                wAddress = wAddress + 4;
        }

        //Wait SPIFCR
        if(!gfnIT7259_SPIFCRReady(ts_data))
        {
                return ERROR_QUERY_TIME_OUT;
        }

        memcpy((unsigned char*)pData, pucCommandBuffer, readLength * sizeof(unsigned char));

        return COMMAND_SUCCESS;
}

static int i2cInternalReadFromIT7259(struct it7259_ts_data *ts_data,
				int wAddress, unsigned char readDataBuffer[],
				unsigned short readDataLength)
{
	int ret;
	unsigned char buffer4Write[1024];
	struct i2c_msg msgs[2] = { { .addr = ts_data->client->addr, .flags = 0,
		.len = 2, .buf = buffer4Write }, { .addr = ts_data->client->addr, .flags =
			I2C_M_RD, .len = readDataLength, .buf = readDataBuffer } };
	//printk("====in internal read function===\n");

	buffer4Write[0] = 0x70;
	buffer4Write[1] = (unsigned char)(wAddress &0xFF);

	memset(readDataBuffer, 0xFF, readDataLength);
	ret = i2c_transfer(ts_data->client->adapter, msgs, 2);
	return ret;

}



static int gfnIT7259_DirectEraseFlash(struct it7259_ts_data *ts_data, unsigned char ucEraseType, int wFlashAddress)
{
        int nSector = 0;
        unsigned char pucCommandBuffer[1024];
        int wTmp;
        int wAddress;
        nSector = wFlashAddress/0x0400;
        pucCommandBuffer[0] = nSector;

        //Select Sector
        wAddress = 0xF404;
        wTmp = i2cDirectWriteToIT7259(ts_data,wAddress,pucCommandBuffer,1);
        if(wTmp <= 0)
        {
                return COMMAND_ERROR;
        }

        //Wait SPIFCR
        if(!gfnIT7259_SPIFCRReady(ts_data))
        {
                return ERROR_QUERY_TIME_OUT;
        }

        //Read flash
        wAddress = 0xF402;
        pucCommandBuffer[0] = ucEraseType;
        wTmp = i2cDirectWriteToIT7259(ts_data,wAddress,pucCommandBuffer,1);

       //Wait SPIFCR
        if(!gfnIT7259_SPIFCRReady(ts_data))
        {
                return ERROR_QUERY_TIME_OUT;
        }

        return COMMAND_SUCCESS;
}


static int gfnIT7259_DMAModeWriteFlash(struct it7259_ts_data *ts_data, int wFlashAddress, int wSRAMAddress,
                        unsigned int dataLength, unsigned char* pData,  bool bPollingWait)
{
	int nSector = 0;
	int wAddress;
	int wTmp;
	int i;
	unsigned char pucCommandBuffer[1024];
	unsigned char pucReadData[2];
	unsigned int LenOffset;
	unsigned char bufTemp[4];
	unsigned int wStartAddress;

	//write  to address 0x0000 (SRAM only 6K)
	memset(bufTemp, 0xFF, 4); //for 8 byte limit
	wAddress = wSRAMAddress;
	printk("###write  to address 0x0000 (wSRAMAddress = %04x, dataLength = %02x)\n",
			wSRAMAddress,dataLength);
	for(LenOffset=0; LenOffset < dataLength; LenOffset+=4) {
		for(i = 0; i < 4; i++) {
			bufTemp[i] = pData[LenOffset + i];
		}
		wTmp = i2cDirectWriteToIT7259(ts_data,wAddress,bufTemp,4);
		if(wTmp <= 0) {
			//printk("###write  to address 0x0000 fail!\n");                
			return COMMAND_ERROR;
		}
		wAddress = wAddress + 4;
		//printk(" ======== wAddress = %04x , LenOffset = %02x ========\n",wAddress,LenOffset);
	}

	//Select Sector 
	memset(pucCommandBuffer, 0xFF, 1024);
	nSector = wFlashAddress/0x0400;
	pucCommandBuffer[0] = (unsigned char)(nSector & 0xFF);

	wAddress = 0xF404;
	wTmp = i2cDirectWriteToIT7259(ts_data,wAddress,pucCommandBuffer,1);//DirectWriteMemoryRegister
	if(wTmp <= 0) {
		printk("###Select Sector fail!");
		return COMMAND_ERROR;
	}
	//Wait SPIFCR
	//printk("###Wait SPIFCR\n");
	if(!gfnIT7259_SPIFCRReady(ts_data)) {
		return ERROR_QUERY_TIME_OUT;
	}

	//Write Flash strat address
	//printk("###Write Flash strat address\n");
	memset(pucCommandBuffer, 0xFF, 1024);
	wAddress = 0xF41A;
	wStartAddress = wFlashAddress - (nSector*0x0400);
	pucCommandBuffer[0] =  wStartAddress & 0x00FF;
	pucCommandBuffer[1] =  (wStartAddress & 0xFF00) >> 8 ;
	wTmp = i2cDirectWriteToIT7259(ts_data, wAddress, pucCommandBuffer, 2);//DirectWriteMemoryRegister  
	if(wTmp <= 0) {
		printk("###Write Flash strat address fail!\n");
		return COMMAND_ERROR;
	}

	//Write SARM strat address
	wAddress = 0xF41C;
	memset(pucCommandBuffer, 0xFF, 1024);
	pucCommandBuffer[0] =  wSRAMAddress & 0xFF;
	pucCommandBuffer[1] =  (wSRAMAddress & 0xFF00) >> 8 ;
	wTmp = i2cDirectWriteToIT7259(ts_data, wAddress, pucCommandBuffer, 2);//DirectWriteMemoryRegister
	if(wTmp <= 0)
	{
		printk("###Write SARM strat address fail!\n");
		return COMMAND_ERROR;
	}

	//write DMA transfer length
	wAddress = 0xF41E;
	pucCommandBuffer[0] =  dataLength & 0xFF;
	pucCommandBuffer[1] =  (dataLength & 0xFF00) >> 8 ;
	wTmp = i2cDirectWriteToIT7259(ts_data, wAddress, pucCommandBuffer, 2);//DirectWriteMemoryRegister
	if(wTmp <= 0)
	{
		printk("###write DMA transfer length fail!\n");
		return COMMAND_ERROR;
	}

	//Write DMA_DIR and DMAEN
	wAddress = 0xF418;
	pucCommandBuffer[0] = 0x0B; //auto erase
	pucCommandBuffer[1] = 0x00;
	wTmp = i2cDirectWriteToIT7259(ts_data, wAddress, pucCommandBuffer, 2);//DirectWriteMemoryRegister
	if(wTmp <= 0)
	{
		printk("###Write DMA_DIR and DMAEN fail!\n");
		return COMMAND_ERROR;
	}
	if(bPollingWait) {
		//polling bit 0, until value of bit 0 = 0
		wAddress = 0xF418;
		do {
			wTmp = i2cDirectReadFromIT7259(ts_data, wAddress, pucReadData, 2);//gfnIT7259_DirectReadMemoryRegister
			if(wTmp <= 0) {
				break;
				return COMMAND_ERROR;
			}
		}while((pucReadData[0] & 0x01)!= 0x00);

		//Wait SPIFCR
		if(!gfnIT7259_SPIFCRReady(ts_data)) {
			return ERROR_QUERY_TIME_OUT;
		}

	}
	return COMMAND_SUCCESS;
}

static unsigned int gfnIT7259_GetFWSize(struct it7259_ts_data *ts_data)
{
        int wAddress;
        unsigned char arucBuffer[1024];
        unsigned int unRet = 0;

        printk("###Entry gfnIT7259_GetFWSize()\n");
        wAddress = 0;
        gfnIT7259_DirectReadFlash(ts_data, wAddress, 0x0400, arucBuffer);

        unRet = arucBuffer[0x80+12] + (arucBuffer[0x80+13] << 8);

        return unRet;
}


static bool gfnIT7259_SwitchCPUClock(struct it7259_ts_data *ts_data, unsigned char ucMode)
{
        unsigned char ucCommandBuffer[1];
        unsigned char ucRetCommandBuffer[1];
        int nErrCount = 0;
        int dwAddress = 0x0023;

        ucCommandBuffer[0] = ucMode;

        do
        {
                i2cInternalWriteToIT7259(ts_data, dwAddress,
						ucCommandBuffer, 1);

                i2cInternalReadFromIT7259(ts_data, dwAddress,
						ucRetCommandBuffer, 1);

                nErrCount++;
        } while(((ucRetCommandBuffer[0] & 0x0F )  != ucMode) && nErrCount <= 1000);

        if(nErrCount>1000) {
                return false;
        }

        return true;

}


static int gfnIT7259_DirectWriteFlash(struct it7259_ts_data *ts_data, int wFlashAddress, unsigned int wWriteLength, unsigned char* pData)
{
        //printk("###Entry gfnIT7259_DirectWriteFlash()\n");    
        //struct IT7259_ts_data *ts = gl_ts;    
        //struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
        int nSector = 0;
        unsigned char pucCommandBuffer[1024];
        int wTmp;
        int wAddress;
        int wOffset;
        nSector = wFlashAddress/0x0400;
        pucCommandBuffer[0] = nSector;

        //Select Sector
        wAddress = 0xF404;
        wTmp = i2cDirectWriteToIT7259(ts_data,wAddress,pucCommandBuffer,1);
        if(wTmp <= 0)
        {
                return COMMAND_ERROR;
        }

        //Wait SPIFCR
        if(!gfnIT7259_SPIFCRReady(ts_data))
        {
                return ERROR_QUERY_TIME_OUT;
        }

        //write flash
        wOffset = wFlashAddress - (nSector*0x0400);
        wAddress = 0x3000 + wOffset;
        memcpy(pucCommandBuffer, (unsigned char*)pData, wWriteLength * sizeof(unsigned char));
        //wTmp = gfnIT7259_DirectWriteMemoryRegister(0x01,wAddress,0x0000,wWriteLength, pucCommandBuffer);
        wTmp = i2cDirectWriteToIT7259(ts_data,wAddress,pucCommandBuffer,wWriteLength);

        if(wTmp <= 0)
        {
                return COMMAND_ERROR;
        }

        //Wait SPIFCR
        if(!gfnIT7259_SPIFCRReady(ts_data))
        {
                return ERROR_QUERY_TIME_OUT;
        }

        return COMMAND_SUCCESS;
}


static bool gfnIT7259_FirmwareDownload(struct it7259_ts_data *ts_data, unsigned int unFirmwareLength,
                        unsigned char arucFW[], unsigned int unConfigLength, unsigned char arucConfig[])
{
	int dwAddress;
	unsigned char RetDATABuffer[10];
	unsigned char DATABuffer[10];
	int nSector = 0;
	unsigned int nFillSize = 0;
	int wTmp;
	unsigned int unTmp;
	unsigned int nConfigSize;
	unsigned long dwFlashSize = 0x10000;
	unsigned int nEndFwSector;
	unsigned int nStartCFGSector;
	unsigned char putFWBuffer[1024];
	int wAddress;
	int wRemainderFWAddress;
	int wConfigAddress = 0;
	unsigned int nRemainderFwSize = 0;
	unsigned int i = 0;
	int nConfigCount = 0;
	int nSize = 0;
	int Tmp = 0;

	if((unFirmwareLength == 0) && (unConfigLength == 0)) {
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}


	disable_irq(ts_data->client->irq);
	//trun off CPU data clock
	if (!gfnIT7259_SwitchCPUClock(ts_data, 0x01))
	{
		printk("###002 gfnIT7259_SwitchCPUClock(0x01) fail!\n");
		return false;
	}


	//Wait SPIFCR
	//printk("###003 Wait SPIFCR\n");       
	dwAddress = 0xF400;
	do
	{
		i2cDirectReadFromIT7259(ts_data,dwAddress,RetDATABuffer,2);
	}
	while((RetDATABuffer[1] & 0x01 )  != 0x00);
	//printk("###003 End SPIFCR\n");
	//Erase signature
	//printk("###004 Erase signature\n");
	dwAddress = 0xF404;
	DATABuffer[0] = 0x3F;
	i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,1);

	dwAddress = 0xF402;
	DATABuffer[0] = 0xD7;
	i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,1);

	//Wait SPIFCR
	//printk("###005 Wait SPIFCR\n");
	dwAddress = 0xF400;
	do
	{
		i2cDirectReadFromIT7259(ts_data,dwAddress,RetDATABuffer,2);
	}
	while((RetDATABuffer[1] & 0x01 )  != 0x00);
	//printk("###005 End SPIFCR\n");
	if((download == 2)||(download == 3)) {
		//Download FW
		printk("###006 Download FW\n");
		for(i  = 0 ; i < unFirmwareLength ; i+=0x0400) {
			if(( unFirmwareLength - i) >= 0x0400)//0x0400
				nFillSize = 0x0400;//0x0400
			else
				nFillSize = unFirmwareLength - i ;

			Tmp = gfnIT7259_DMAModeWriteFlash(ts_data, i, 0x0000,
							nFillSize,arucFW+i, true);

			mdelay(100);//for test
			if(wTmp != COMMAND_SUCCESS) {
				//Write Firmware Flash error
				printk("###DMA ModeWrite Firmware Flash error(FlashAddress:%04x)\n", i);
				return false;
			}
		}

		//3. Fw CRC Check       
		//check FW CRC
		//printk("###007 check FW CRC\n");
		//write start address

		dwAddress = 0xF40A;
		DATABuffer[0] = 0x00;
		DATABuffer[1] = 0x00;
		i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,2);

		//write end address
		dwAddress = 0xF40C;
		DATABuffer[0] = (unFirmwareLength-3) & 0x00ff ;
		DATABuffer[1] = ((unFirmwareLength-3) & 0xff00)>>8;
		i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,2);

		//write CRCCR
		dwAddress = 0xF408;
		DATABuffer[0] = 0x01 ;
		i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,1);

		//wait CRCCR
		dwAddress = 0xF408;
		do
		{
			i2cDirectReadFromIT7259(ts_data,dwAddress,RetDATABuffer,2);
		}
		while((RetDATABuffer[0] & 0x01 )  != 0x00);

		//read CRC
		dwAddress = 0xF40E;
		i2cDirectReadFromIT7259(ts_data,dwAddress,RetDATABuffer,2);

		//compare FW CRC
		//printk("###008 compare FW CRC\n");

		if (RetDATABuffer[0]!= arucFW[unFirmwareLength - 2] || RetDATABuffer[1]!= arucFW[unFirmwareLength - 1])
		{
			printk("###008 FW CRC check fail\n");
			printk("RetDATABuffer[0]:%02x,RetDATABuffer[1]:%02x,FW[Length-2]:%02x,FW[Length-1]:%02x\n",
				RetDATABuffer[0], RetDATABuffer[1],
				arucFW[unFirmwareLength - 2],
				arucFW[unFirmwareLength - 1]);
			return false;//FW CRC check fail
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if((download == 1) || (download == 3)) {
		//download config
		printk("###009 start to download config\n");
		unTmp = gfnIT7259_GetFWSize(ts_data);
		nConfigSize = unConfigLength;

		//3.7 get address for writing config (in flash)
		//check whether fw and config are in the same sector or not
		//set flash size
		nEndFwSector = (unTmp-1) / 1024;
		nStartCFGSector = 62 - (unConfigLength-1)/1024;
		nRemainderFwSize = 0;

		if(nEndFwSector == nStartCFGSector) {
			nRemainderFwSize = unTmp - nEndFwSector*1024;
			wAddress = nEndFwSector*0x0400;
			//printk(" =========== nRemainderFwSize = %4x ===========\n",nRemainderFwSize);
			gfnIT7259_DirectReadFlash(ts_data, wAddress, nRemainderFwSize, putFWBuffer);
		}

		//get config start address      
		wTmp = dwFlashSize -1024 - unConfigLength;
		//printk("###010 get config start address(%04x)\n",wTmp);

		for(i = wTmp ; i<(dwFlashSize -1024) ; i+=0x0400) {
			nSector = i/0x0400;

			if((nRemainderFwSize!=0) && (nSector == nStartCFGSector)) {
				wRemainderFWAddress = nStartCFGSector*0x0400;
				nFillSize = nRemainderFwSize;
				gfnIT7259_DMAModeWriteFlash(ts_data, wRemainderFWAddress,0x0000,nFillSize,putFWBuffer,true);
			}
			//write config
			nSize = (unConfigLength - (62-nSector)*1024);

			if( nSize >=1024) {
				wConfigAddress = nSector * 0x0400;
				nFillSize = 1024;
			} else {
				wConfigAddress = i;
				nFillSize = nSize;
			}

			wTmp = gfnIT7259_DMAModeWriteFlash(ts_data,
					wConfigAddress, 0x0000, nFillSize,
					arucConfig + nConfigCount , true);

			if(wTmp != COMMAND_SUCCESS)
				return false;

			nConfigCount += nFillSize;
		}

		//Config CRC Check
		//printk("###011 Config CRC Check\n");
		//write start address
		dwAddress = 0xF40A;
		DATABuffer[0] = (0x10000 - unConfigLength -1024) & 0x00ff;
		DATABuffer[1] = ((0x10000- unConfigLength -1024) & 0xff00)>>8;
		i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,2);

		//write end address
		dwAddress = 0xF40C;
		DATABuffer[0] = (0x10000 -1024 -3)& 0x00ff;
		DATABuffer[1] = ((0x10000-1024 -3) & 0xff00)>>8;
		i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,2);

		//write CRCCR
		dwAddress = 0xF408;
		DATABuffer[0] = 0x01 ;
		i2cDirectWriteToIT7259(ts_data,dwAddress,DATABuffer,1);

		//wait CRCCR
		dwAddress = 0xF408;
		do
		{
			i2cDirectReadFromIT7259(ts_data, dwAddress,
						RetDATABuffer, 2);
		}
		while((RetDATABuffer[0] & 0x01 )  != 0x00);

		//read CRC
		dwAddress = 0xF40E;
		i2cDirectReadFromIT7259(ts_data, dwAddress,
					RetDATABuffer, 2);

		//compare Config CRC
		if ((RetDATABuffer[0]!= arucConfig[unConfigLength - 2]) ||
			(RetDATABuffer[1]!= arucConfig[unConfigLength - 1])) {
			printk("###011 config CRC Check Error\n");
			printk("RetDATABuffer[0]:%02x,RetDATABuffer[1]:%02x,CFG[Length-2]:%02x,CFG[Length-1]:%02x\n",
				RetDATABuffer[0], RetDATABuffer[1],
				arucConfig[unConfigLength - 2],
				arucConfig[unConfigLength - 1]);
			return false;//config CRC Check Error
		}
	}

	//write signature
	DATABuffer[0] = 0x59;
	DATABuffer[1] = 0x72;

	gfnIT7259_DirectEraseFlash(ts_data, 0xD7,(dwFlashSize -1024));
	gfnIT7259_DirectWriteFlash(ts_data, (dwFlashSize -1024),2,DATABuffer);

	DATABuffer[0] = 0x00;
	DATABuffer[1] = 0x00;
	i2cDirectReadFromIT7259(ts_data,dwAddress,DATABuffer,2);

	//trun on CPU data clock
	//printk("###012 trun on CPU data clock\n");    
	if(!gfnIT7259_SwitchCPUClock(ts_data, 0x04))
	{
		printk("###012 trun on CPU data clock fail\n");
		return false;
	}

	printk("###gfnIT7259_FirmwareDownload() end.\n");

	enable_irq(ts_data->client->irq);
	return true;
}

static int Force_Upgrade(struct it7259_ts_data *ts_data)
{
	unsigned int fw_size = 0;
	unsigned int config_size = 0;

	printk("Execute force upgrade()\n");
	fw_size = sizeof(fw_buf);
	config_size = sizeof(config_buf);

	printk("File Config version : %02x %02x %02x %02x\n",
			config_buf[config_size-8], config_buf[config_size-7],
			config_buf[config_size-6], config_buf[config_size-5]);

	download = 3;
	if (gfnIT7259_FirmwareDownload(ts_data, fw_size, fw_buf, config_size, config_buf) == false) {
		//fail
		return 1;
	}else{
		//success
		return 0;
	}
}

static int Upgrade_FW_CFG(struct it7259_ts_data *ts_data)
{
	unsigned int fw_size = 0;
	unsigned int config_size = 0;

	struct file *fw_fd = NULL;
	struct file *config_fd = NULL;
	mm_segment_t fs;

	unsigned char *fw_buf = kzalloc(0x10000, GFP_KERNEL);
	unsigned char *config_buf = kzalloc(0x500, GFP_KERNEL);

	printk("Execute Upgrade_FW_CFG()\n");
	if ( fw_buf  == NULL || config_buf == NULL  ) {
		printk("kzalloc failed\n");
	}

	fs = get_fs();
	set_fs(get_ds());
	//load fw file
	fw_fd = filp_open(IT7259_FW_PATH, O_RDONLY, 0);

	if (fw_fd < 0){
		printk("open %s failed \n", IT7259_FW_PATH);
		return 1;
	}

	fw_size = fw_fd->f_op->read(fw_fd, fw_buf, 0x10000, &fw_fd->f_pos);
	printk("File Firmware version : %02x %02x %02x %02x\n",
		fw_buf[136], fw_buf[137], fw_buf[138], fw_buf[139]);
	//printk("--------------------- fw_size = %x\n", fw_size);

	//load config file
	config_fd = filp_open(IT7259_CFG_PATH, O_RDONLY, 0);

	if(config_fd < 0){
		printk("open %s failed \n", IT7259_CFG_PATH);
		return 1;
	}

	config_size = config_fd->f_op->read(config_fd, config_buf, 0x500, &config_fd->f_pos);
	printk("File Config version : %02x %02x %02x %02x\n",
		config_buf[config_size-8], config_buf[config_size-7],
		config_buf[config_size-6], config_buf[config_size-5]);

	set_fs(fs);
	filp_close(fw_fd,NULL);
	filp_close(config_fd,NULL);

	printk("Chip firmware version : %02x %02x %02x %02x\n",
		ts_data->fw_ver[0], ts_data->fw_ver[1],
		ts_data->fw_ver[2], ts_data->fw_ver[3]);
	printk("Chip config version : %02x %02x %02x %02x\n",
		ts_data->cfg_ver[0], ts_data->cfg_ver[1],
		ts_data->cfg_ver[2], ts_data->cfg_ver[3]);
	download = 0;
	if ((ts_data->cfg_ver[0] != config_buf[config_size-8]) ||
		(ts_data->cfg_ver[1] != config_buf[config_size-7]) ||
		(ts_data->cfg_ver[2] != config_buf[config_size-6])
			|| (ts_data->cfg_ver[3] != config_buf[config_size-5]))
		download += 1;

	if ((ts_data->fw_ver[0] != fw_buf[136]) ||
		(ts_data->fw_ver[1] != fw_buf[137]) ||
		(ts_data->fw_ver[2] != fw_buf[138]) ||
		(ts_data->fw_ver[3] != fw_buf[139]))
		download += 2;

	printk("%s, print download = %d\n", __func__, download);

	if(download == 0){
		printk("Do not need to upgrade\n");
		return 0;
	} else {
		if (gfnIT7259_FirmwareDownload(ts_data, fw_size, fw_buf, config_size, config_buf) == false) {
			//fail          
			return 1;
		}else{
			//success               
			return 0;
		}
	}
}


static ssize_t sysfs_point_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
        uint8_t pt_data[sizeof(struct point_data)];
        int readSuccess;
        ssize_t ret;

        readSuccess = it7259_i2c_read_no_ready_check(ts_data, BUF_POINT_INFO,
                                        pt_data, sizeof(pt_data));

        if (readSuccess == IT_I2C_READ_RET) {
                ret = scnprintf(buf, MAX_BUFFER_SIZE,
                        "point_show read ret[%d]--point[%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x]\n",
                        readSuccess, pt_data[0], pt_data[1],
                        pt_data[2], pt_data[3], pt_data[4],
                        pt_data[5], pt_data[6], pt_data[7],
                        pt_data[8], pt_data[9], pt_data[10],
                        pt_data[11], pt_data[12], pt_data[13]);
        } else {
                 ret = scnprintf(buf, MAX_BUFFER_SIZE,
                        "failed to read point data\n");
        }
        dev_dbg(dev, "%s", buf);

        return ret;
}


static ssize_t sysfs_upgrade_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

        printk("%s():\n", __func__);
        if (ts_data->suspended) {
                dev_err(dev, "Device is suspended, can't upgrade FW/CFG !!!\n");
                return -EBUSY;
        }

        mutex_lock(&ts_data->fw_cfg_mutex);
        if(Upgrade_FW_CFG(ts_data)) {
                printk("IT7259_upgrade_failed\n");
                return -1;

        } else {
                printk("IT7259_upgrade_OK\n\n");
                return 0;
        }
        mutex_unlock(&ts_data->fw_cfg_mutex);

        return count;
}

static ssize_t sysfs_upgrade_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

        return scnprintf(buf, MAX_BUFFER_SIZE, "%d\n",
                                ts_data->cfg_upgrade_result);
}

static ssize_t sysfs_GetCSEL_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	static const u8 Current_Mode[] = {0x1A, 0x05, 0x00};
	static const u8 Ground_Mode[] = {0x1A, 0x05, 0x01};
	static const u8 Shielding_Mode[] = {0x1A, 0x05, 0x02};
	u8 Current_CSEL[56] = {0,};
	u8 Ground_CSEL[56] = {0,};
	u8 Shielding_CSEL[56] = {0,};
	int ret;

	ret = it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND, Current_Mode,
			sizeof(Current_Mode));
	if (ret != IT_I2C_WRITE_RET) {
		dev_err(&ts_data->client->dev, "failed to write CMD_IDENT_CHIP\n");
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to write CMD_IDENT_CHIP\n");
	}

	ret = it7259_wait_device_ready(ts_data, true, false);
	if (ret < 0) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip status\n");
	}
	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE, Current_CSEL,
			sizeof(Current_CSEL));
	if (ret != IT_I2C_READ_RET) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip-id\n");
	}

	return scnprintf(buf, MAX_BUFFER_SIZE,
			"it7259 Current_CSEL : 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n  0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n  0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n  0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n  0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n ===========================================================================\n",
			Current_CSEL[0],Current_CSEL[1],Current_CSEL[2],Current_CSEL[3],Current_CSEL[4],Current_CSEL[5],Current_CSEL[6],Current_CSEL[7],
			Current_CSEL[8],Current_CSEL[9],Current_CSEL[10],Current_CSEL[11],Current_CSEL[12],Current_CSEL[13],Current_CSEL[14],Current_CSEL[15],
			Current_CSEL[16],Current_CSEL[17],Current_CSEL[18],Current_CSEL[19],Current_CSEL[20],Current_CSEL[21],Current_CSEL[22],Current_CSEL[23],
			Current_CSEL[24],Current_CSEL[25],Current_CSEL[26],Current_CSEL[27],Current_CSEL[28],Current_CSEL[29],Current_CSEL[30],Current_CSEL[31],
			Current_CSEL[32],Current_CSEL[33],Current_CSEL[34],Current_CSEL[35],Current_CSEL[36],Current_CSEL[37],Current_CSEL[38],Current_CSEL[39],
			Current_CSEL[40],Current_CSEL[41],Current_CSEL[42],Current_CSEL[43],Current_CSEL[44],Current_CSEL[45],Current_CSEL[46],Current_CSEL[47],
			Current_CSEL[48],Current_CSEL[49],Current_CSEL[50],Current_CSEL[51],Current_CSEL[52],Current_CSEL[53],Current_CSEL[54],Current_CSEL[55]);

	ret = it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND, Ground_Mode,
			sizeof(Ground_Mode));
	if (ret != IT_I2C_WRITE_RET) {
		dev_err(&ts_data->client->dev,"failed to write CMD_IDENT_CHIP\n");
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to write CMD_IDENT_CHIP\n");
	}
	ret = it7259_wait_device_ready(ts_data, true, false);
	if (ret < 0) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip status\n");
	}

	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE, Ground_CSEL,
			sizeof(Ground_CSEL));
	if (ret != IT_I2C_READ_RET) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip-id\n");
	}

	return scnprintf(buf, MAX_BUFFER_SIZE,
			"it7259 Ground_CSEL  : 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n  0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n  ===========================================================================\n",
			Ground_CSEL[0],Ground_CSEL[1],Ground_CSEL[2],Ground_CSEL[3],Ground_CSEL[4],Ground_CSEL[5],Ground_CSEL[6],Ground_CSEL[7],
			Ground_CSEL[8],Ground_CSEL[9],Ground_CSEL[10],Ground_CSEL[11],Ground_CSEL[12],Ground_CSEL[13],Ground_CSEL[14],Ground_CSEL[15],
			Ground_CSEL[16],Ground_CSEL[17],Ground_CSEL[18],Ground_CSEL[19],Ground_CSEL[20],Ground_CSEL[21],Ground_CSEL[22],Ground_CSEL[23],
			Ground_CSEL[24],Ground_CSEL[25],Ground_CSEL[26],Ground_CSEL[27],Ground_CSEL[28],Ground_CSEL[29],Ground_CSEL[30],Ground_CSEL[31],
			Ground_CSEL[32],Ground_CSEL[33],Ground_CSEL[34],Ground_CSEL[35],Ground_CSEL[36],Ground_CSEL[37],Ground_CSEL[38],Ground_CSEL[39],
			Ground_CSEL[40],Ground_CSEL[41],Ground_CSEL[42],Ground_CSEL[43],Ground_CSEL[44],Ground_CSEL[45],Ground_CSEL[46],Ground_CSEL[47],
			Ground_CSEL[48],Ground_CSEL[49],Ground_CSEL[50],Ground_CSEL[51],Ground_CSEL[52],Ground_CSEL[53],Ground_CSEL[54],Ground_CSEL[55]);

	ret = it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND, Shielding_Mode,
			sizeof(Shielding_Mode));
	if (ret != IT_I2C_WRITE_RET) {
		dev_err(&ts_data->client->dev, "failed to write CMD_IDENT_CHIP\n");
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to write CMD_IDENT_CHIP\n");
	}

	ret = it7259_wait_device_ready(ts_data, true, false);
	if (ret < 0) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip status\n");
	}

	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE, Shielding_CSEL,
			sizeof(Shielding_CSEL));
	if (ret != IT_I2C_READ_RET) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip-id\n");
	}
	return scnprintf(buf, MAX_BUFFER_SIZE,
			"it7259 Shielding_CSEL  : 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n ===========================================================================\n",
			Shielding_CSEL[0],Shielding_CSEL[1],Shielding_CSEL[2],Shielding_CSEL[3],Shielding_CSEL[4],Shielding_CSEL[5],Shielding_CSEL[6],Shielding_CSEL[7],
			Shielding_CSEL[8],Shielding_CSEL[9],Shielding_CSEL[10],Shielding_CSEL[11],Shielding_CSEL[12],Shielding_CSEL[13],Shielding_CSEL[14],Shielding_CSEL[15],
			Shielding_CSEL[16],Shielding_CSEL[17],Shielding_CSEL[18],Shielding_CSEL[19],Shielding_CSEL[20],Shielding_CSEL[21],Shielding_CSEL[22],Shielding_CSEL[23],
			Shielding_CSEL[24],Shielding_CSEL[25],Shielding_CSEL[26],Shielding_CSEL[27],Shielding_CSEL[28],Shielding_CSEL[29],Shielding_CSEL[30],Shielding_CSEL[31],
			Shielding_CSEL[32],Shielding_CSEL[33],Shielding_CSEL[34],Shielding_CSEL[35],Shielding_CSEL[36],Shielding_CSEL[37],Shielding_CSEL[38],Shielding_CSEL[39],
			Shielding_CSEL[40],Shielding_CSEL[41],Shielding_CSEL[42],Shielding_CSEL[43],Shielding_CSEL[44],Shielding_CSEL[45],Shielding_CSEL[46],Shielding_CSEL[47],
			Shielding_CSEL[48],Shielding_CSEL[49],Shielding_CSEL[50],Shielding_CSEL[51],Shielding_CSEL[52],Shielding_CSEL[53],Shielding_CSEL[54],Shielding_CSEL[55]);

}

static ssize_t sysfs_readstage_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	static const u8 read_stage[] = {0x1A, 0x00, 0x01, 0x05};
	u8 stage_CDC[10] = {0,};
	int ret;

	ret = it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND, read_stage,
			sizeof(read_stage));
	if (ret != IT_I2C_WRITE_RET) {
		dev_err(&ts_data->client->dev, "failed to write CMD_IDENT_CHIP\n");
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to write CMD_IDENT_CHIP\n");
	}

	ret = it7259_wait_device_ready(ts_data, true, false);
	if (ret < 0) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip status\n");
	}

	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE, stage_CDC,
			sizeof(stage_CDC));
	if (ret != IT_I2C_READ_RET) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip-id\n");
	}

	return scnprintf(buf, MAX_BUFFER_SIZE,
			"it7259 stage CDC : 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n",
			stage_CDC[0], stage_CDC[1], stage_CDC[2], stage_CDC[3], stage_CDC[4],
			stage_CDC[5], stage_CDC[6], stage_CDC[7], stage_CDC[8], stage_CDC[9]);

}


static ssize_t sysfs_chipID_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	static const uint8_t cmd_ident[] = {CMD_IDENT_CHIP};
	uint8_t chip_id[10] = {0,};
	int ret;

	ret = it7259_wait_device_ready(ts_data, false, true);
	if (ret < 0) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip status\n");
	}

	ret = it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND, cmd_ident,
			sizeof(cmd_ident));
	if (ret != IT_I2C_WRITE_RET) {
		dev_err(&ts_data->client->dev, "failed to write CMD_IDENT_CHIP\n");
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to write CMD_IDENT_CHIP\n");
	}

	ret = it7259_wait_device_ready(ts_data, true, false);
	if (ret < 0) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip status\n");
	}
	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE, chip_id,
			sizeof(chip_id));
	if (ret != IT_I2C_READ_RET) {
		return scnprintf(buf, MAX_BUFFER_SIZE,
				"failed to read chip-id\n");
	}

	return scnprintf(buf, MAX_BUFFER_SIZE,
			"it7259_ts_chip_identify read id: %02X %c%c%c%c%c%c%c %c%c\n",
			chip_id[0], chip_id[1], chip_id[2], chip_id[3], chip_id[4],
			chip_id[5], chip_id[6], chip_id[7], chip_id[8], chip_id[9]);
}

static ssize_t sysfs_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

	return scnprintf(buf, MAX_BUFFER_SIZE,
			"fw{%X.%X.%X.%X} cfg{%X.%X.%X.%X}\n",
			ts_data->fw_ver[0], ts_data->fw_ver[1],
			ts_data->fw_ver[2], ts_data->fw_ver[3],
			ts_data->cfg_ver[0], ts_data->cfg_ver[1],
			ts_data->cfg_ver[2], ts_data->cfg_ver[3]);
}

static ssize_t sysfs_cfg_name_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	char *strptr;

	if (count >= MAX_BUFFER_SIZE) {
		dev_err(dev, "Input over %d chars long\n", MAX_BUFFER_SIZE);
		return -EINVAL;
	}

	strptr = strnstr(buf, ".bin", count);
	if (!strptr) {
		dev_err(dev, "Input is invalid cfg file\n");
		return -EINVAL;
	}

	strlcpy(ts_data->cfg_name, buf, count);

	return count;
}

static ssize_t sysfs_cfg_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

	if (strnlen(ts_data->cfg_name, MAX_BUFFER_SIZE) > 0)
		return scnprintf(buf, MAX_BUFFER_SIZE, "%s\n",
				ts_data->cfg_name);
	else
		return scnprintf(buf, MAX_BUFFER_SIZE,
			"No config file name given\n");
}

static ssize_t sysfs_fw_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	char *strptr;

	if (count >= MAX_BUFFER_SIZE) {
		dev_err(dev, "Input over %d chars long\n", MAX_BUFFER_SIZE);
		return -EINVAL;
	}

	strptr = strnstr(buf, ".bin", count);
	if (!strptr) {
		dev_err(dev, "Input is invalid fw file\n");
		return -EINVAL;
	}

	strlcpy(ts_data->fw_name, buf, count);
	return count;
}

static ssize_t sysfs_fw_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

	if (strnlen(ts_data->fw_name, MAX_BUFFER_SIZE) > 0)
		return scnprintf(buf, MAX_BUFFER_SIZE, "%s\n",
			ts_data->fw_name);
	else
		return scnprintf(buf, MAX_BUFFER_SIZE,
			"No firmware file name given\n");
}

#ifdef	CONFIG_BEZEL_SUPPORT
static ssize_t sysfs_bezel_inset_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

        return scnprintf(buf, MAX_BUFFER_SIZE, "%d\n",
                                ts_data->bdata->inset);
}

static ssize_t sysfs_bezel_inset_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	int inset, rc;

	rc = kstrtoint(buf, 0, &inset);
	if (rc != 0)
		return -EINVAL;
	ts_data->bdata->inset = inset;
	return count;
}
static ssize_t sysfs_bezel_ang_dis_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

        return scnprintf(buf, MAX_BUFFER_SIZE, "%d\n",
                                ts_data->bdata->angular_dist_thresh);
}

static ssize_t sysfs_bezel_ang_dis_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	int ang_dis, rc;

	rc = kstrtoint(buf, 0, &ang_dis);
	if (rc != 0)
		return -EINVAL;
	ts_data->bdata->angular_dist_thresh = ang_dis;
	return count;
}
static ssize_t sysfs_bezel_thickness_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);

        return scnprintf(buf, MAX_BUFFER_SIZE, "%d\n",
                                ts_data->bdata->thickness);
}

static ssize_t sysfs_bezel_thickness_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	int thickness, rc;

	rc = kstrtoint(buf, 0, &thickness);
	if (rc != 0)
		return -EINVAL;
	ts_data->bdata->thickness = thickness;
	return count;
}
#endif

static DEVICE_ATTR(version, S_IRUGO | S_IWUSR,
		sysfs_version_show, NULL);
static DEVICE_ATTR(chipID, S_IRUGO | S_IWUSR,
                        sysfs_chipID_show, NULL);
static DEVICE_ATTR(readstage, S_IRUGO | S_IWUSR,
                        sysfs_readstage_show, NULL);
static DEVICE_ATTR(GetCSEL, S_IRUGO | S_IWUSR,
                        sysfs_GetCSEL_show, NULL);
static DEVICE_ATTR(upgrade, S_IRUGO | S_IWUSR,
                        sysfs_upgrade_show, sysfs_upgrade_store);
static DEVICE_ATTR(point, S_IRUGO | S_IWUSR,
                        sysfs_point_show, NULL);
static DEVICE_ATTR(fw_name, S_IRUGO | S_IWUSR,
                        sysfs_fw_name_show, sysfs_fw_name_store);
static DEVICE_ATTR(cfg_name, S_IRUGO | S_IWUSR,
                        sysfs_cfg_name_show, sysfs_cfg_name_store);
#ifdef CONFIG_BEZEL_SUPPORT
static DEVICE_ATTR(bezel_inset, S_IRUGO | S_IWUSR,
                        sysfs_bezel_inset_show, sysfs_bezel_inset_store);
static DEVICE_ATTR(bezel_ang_dis, S_IRUGO | S_IWUSR,
                        sysfs_bezel_ang_dis_show, sysfs_bezel_ang_dis_store);
static DEVICE_ATTR(bezel_thickness, S_IRUGO | S_IWUSR,
                        sysfs_bezel_thickness_show, sysfs_bezel_thickness_store);
#endif

static struct attribute *it7259_attributes[] = {
        &dev_attr_version.attr,
        &dev_attr_upgrade.attr,
        &dev_attr_point.attr,
        &dev_attr_fw_name.attr,
        &dev_attr_cfg_name.attr,
        &dev_attr_chipID.attr,
        &dev_attr_readstage.attr,
        &dev_attr_GetCSEL.attr,
#ifdef CONFIG_BEZEL_SUPPORT
        &dev_attr_bezel_inset.attr,
        &dev_attr_bezel_ang_dis.attr,
        &dev_attr_bezel_thickness.attr,
#endif
                NULL
};

static const struct attribute_group it7259_attr_group = {
	.attrs = it7259_attributes,
};

static void it7259_ts_release_all(struct it7259_ts_data *ts_data)
{
	int finger;

	for (finger = 0; finger < ts_data->pdata->num_of_fingers; finger++) {
		input_mt_slot(ts_data->input_dev, finger);
		input_mt_report_slot_state(ts_data->input_dev,
				MT_TOOL_FINGER, 0);
	}

	input_report_key(ts_data->input_dev, BTN_TOUCH, 0);
	input_sync(ts_data->input_dev);
}


static irqreturn_t it7259_ts_threaded_handler(int irq, void *devid)
{
	struct point_data pt_data;
	struct it7259_ts_data *ts_data = devid;
	struct input_dev *input_dev = ts_data->input_dev;
	u8 dev_status, finger, touch_count = 0, finger_status;
	u8 pressure = FD_PRESSURE_NONE;
	u16 x, y;
	bool palm_detected;
	int ret;
#ifdef	CONFIG_BEZEL_SUPPORT
	u8 bezel_count = 0;
	static u8 transit_from_disp_to_bezel = 0;
	u8 disp_radius = 0;
	u16 x_center = 0, y_center = 0, x_position = 0, y_position = 0;
	u16 x_thickness_center = 0, y_thickness_center = 0;
	u32 virtual_radius = 0, panel_radius = 0, sq_rad_position = 0, sq_disp_position = 0;
#endif

	/* verify there is point data to read & it is readable and valid */
	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_QUERY, &dev_status,
						sizeof(dev_status));
	if (ret == IT_I2C_READ_RET)
		if (!((dev_status & PT_INFO_BITS) & PT_INFO_YES))
			return IRQ_HANDLED;
	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_POINT_INFO,
				(void *)&pt_data, sizeof(pt_data));
	if (ret != IT_I2C_READ_RET) {
		dev_err(&ts_data->client->dev,
			"failed to read point data buffer\n");
		return IRQ_HANDLED;
	}

	/* Check if controller moves from idle to active state */
	if ((pt_data.flags & PD_FLAGS_DATA_TYPE_BITS) !=
					PD_FLAGS_DATA_TYPE_TOUCH) {
		/*
		 * This code adds the touch-to-wake functionality to the ITE
		 * tech driver. When user puts a finger on touch controller in
		 * idle state, the controller moves to active state and driver
		 * sends the KEY_WAKEUP event to wake the device. The
		 * pm_stay_awake() call tells the pm core to stay awake until
		 * the CPU cores are up already. The schedule_work() call
		 * schedule a work that tells the pm core to relax once the CPU
		 * cores are up.
		 */
		if ((pt_data.flags & PD_FLAGS_DATA_TYPE_BITS) ==
				PD_FLAGS_IDLE_TO_ACTIVE &&
				pt_data.gesture_id == 0) {
			pm_stay_awake(&ts_data->client->dev);
			input_report_key(input_dev, KEY_WAKEUP, 1);
			input_sync(input_dev);
			input_report_key(input_dev, KEY_WAKEUP, 0);
			input_sync(input_dev);
			ts_data->fw_active = true;
			schedule_work(&ts_data->work_pm_relax);
		} else {
			dev_dbg(&ts_data->client->dev,
				"Ignore the touch data\n");
		}
		return IRQ_HANDLED;
	}

	/*
	 * Check if touch data also includes any palm gesture or not.
	 * If palm gesture is detected, then send the keycode parsed
	 * from the DT.
	 */
	palm_detected = pt_data.gesture_id & PD_PALM_FLAG_BIT;
	if ((!ts_data->palm_pressed) && palm_detected &&
			ts_data->pdata->palm_detect_en) {
		input_report_key(input_dev,
				ts_data->pdata->palm_detect_keycode, 1);
		input_sync(input_dev);
		ts_data->palm_pressed = true;
	}

#ifdef	CONFIG_BEZEL_SUPPORT
        /* Center co-odinate of panel */
	x_center = ts_data->pdata->panel_maxx/2;
	y_center = ts_data->pdata->panel_maxy/2;

	x_thickness_center = x_center - ts_data->bdata->thickness;
	y_thickness_center = y_center - ts_data->bdata->thickness;
	/* Radius of the panel */
	panel_radius = x_center;

	/* Radius of the display */
	disp_radius = (ts_data->pdata->disp_maxx - ts_data->pdata->disp_minx)/2;
	virtual_radius = disp_radius;
#endif

	for (finger = 0; finger < ts_data->pdata->num_of_fingers; finger++) {
		finger_status = pt_data.flags & (0x01 << finger);

		input_mt_slot(input_dev, finger);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
					finger_status != 0);

		x = pt_data.fd[finger].xLo +
			(((u16)(pt_data.fd[finger].hi & 0x0F)) << 8);
		y = pt_data.fd[finger].yLo +
			(((u16)(pt_data.fd[finger].hi & 0xF0)) << 4);

		pressure = pt_data.fd[finger].pressure & FD_PRESSURE_BITS;

		if (finger_status) {
			if (pressure >= FD_PRESSURE_LIGHT) {
#ifdef	CONFIG_BEZEL_SUPPORT
                                /* Distance from present co-ordinate to center co-ordinate */
				x_position = ABS(x, x_center);
				y_position = ABS(y, y_center);

				sq_rad_position = (u32)((x_position * x_position) + (y_position * y_position));
				ts_data->bdata->sq_rad_position = sq_rad_position;

				if(ts_data->bdata->bezel_touch_status)
					virtual_radius = disp_radius - ts_data->bdata->inset;

				sq_disp_position = (u32)(virtual_radius * virtual_radius);

				if(sq_rad_position > sq_disp_position) {
					bezel_report_wheel(x, y, ts_data->bdata);
					if(ts_data->bdata->step_threshold2 ==
						ts_data->bdata->angular_dist_thresh)
					ts_data->bdata->bezel_touch_status = true;
                                        bezel_count++;
				} else
#endif
				{
				        input_report_key(input_dev, BTN_TOUCH, 1);
				        input_report_abs(input_dev,
							ABS_MT_POSITION_X, x);
				        input_report_abs(input_dev,
							ABS_MT_POSITION_Y, y);
					transit_from_disp_to_bezel = true;
				        touch_count++;
                                }
			}
		}
	}

	if(!ts_data->bdata->bezel_touch_status) {
		input_report_key(input_dev, BTN_TOUCH, touch_count > 0);
		if (ts_data->palm_pressed && ts_data->pdata->palm_detect_en &&
				(!palm_detected)) {
			input_report_key(input_dev,
					ts_data->pdata->palm_detect_keycode, 0);
			ts_data->palm_pressed = false;
		}
		input_sync(input_dev);
	} else if (transit_from_disp_to_bezel) {
		transit_from_disp_to_bezel = false;
		input_report_key(input_dev, BTN_TOUCH, touch_count > 0);
		input_sync(input_dev);
	}
#ifdef	CONFIG_BEZEL_SUPPORT
        /*input_report_rel(input_dev, BTN_TOOL_FINGER, bezel_count > 0);*/
	if(!(bezel_count > 0)) {
		bezel_reset();
		ts_data->bdata->bezel_touch_status = false;
	}
#endif

	return IRQ_HANDLED;
}

static void it7259_ts_work_func(struct work_struct *work)
{
	struct it7259_ts_data *ts_data = container_of(work,
				struct it7259_ts_data, work_pm_relax);

	pm_relax(&ts_data->client->dev);
}

static int it7259_ts_chip_identify(struct it7259_ts_data *ts_data)
{
	static const uint8_t cmd_ident[] = {CMD_IDENT_CHIP};
	static const uint8_t expected_id[] = {0x0A, 'I', 'T', 'E', '7',
							'2', '5', '9'};
	uint8_t chip_id[10] = {0,};
	int ret;

	/*
	 * Sometimes, the controller may not respond immediately after
	 * writing the command, so wait for device to get ready.
	 * FALSE means to retry 20 times at max to read the chip status.
	 * TRUE means to add delay in each retry.
	 */
	ret = it7259_wait_device_ready(ts_data, false, true);
	if (ret < 0) {
		dev_err(&ts_data->client->dev,
			"failed to read chip status %d\n", ret);
		return ret;
	}

	ret = it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND, cmd_ident,
							sizeof(cmd_ident));
	if (ret != IT_I2C_WRITE_RET) {
		dev_err(&ts_data->client->dev,
			"failed to write CMD_IDENT_CHIP %d\n", ret);
		return ret;
	}

	/*
	 * Sometimes, the controller may not respond immediately after
	 * writing the command, so wait for device to get ready.
	 * TRUE means to retry 500 times at max to read the chip status.
	 * FALSE means to avoid unnecessary delays in each retry.
	 */
	ret = it7259_wait_device_ready(ts_data, true, false);
	if (ret < 0) {
		dev_err(&ts_data->client->dev,
			"failed to read chip status %d\n", ret);
		return ret;
	}


	ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE, chip_id,
							sizeof(chip_id));
	if (ret != IT_I2C_READ_RET) {
		dev_err(&ts_data->client->dev,
			"failed to read chip-id %d\n", ret);
		return ret;
	}
	dev_info(&ts_data->client->dev,
		"it7259_ts_chip_identify read id: %02X %c%c%c%c%c%c%c %c%c\n",
		chip_id[0], chip_id[1], chip_id[2], chip_id[3], chip_id[4],
		chip_id[5], chip_id[6], chip_id[7], chip_id[8], chip_id[9]);

	if (memcmp(chip_id, expected_id, sizeof(expected_id)))
		return -EINVAL;

/*	if (chip_id[8] == '5' && chip_id[9] == '6')
		dev_info(&ts_data->client->dev, "rev BX3 found\n");
	else if (chip_id[8] == '6' && chip_id[9] == '6')
		dev_info(&ts_data->client->dev, "rev BX4 found\n");
	else
		dev_info(&ts_data->client->dev, "unknown revision (0x%02X 0x%02X) found\n",
						chip_id[8], chip_id[9]);
*/
	return 0;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int it7259_regulator_configure(struct it7259_ts_data *ts_data, bool on)
{
	int retval;

	if (on == false)
		goto hw_shutdown;

	ts_data->vdd = devm_regulator_get(&ts_data->client->dev, "vdd");
	if (IS_ERR(ts_data->vdd)) {
		dev_err(&ts_data->client->dev,
				"%s: Failed to get vdd regulator\n", __func__);
		return PTR_ERR(ts_data->vdd);
	}

	if (regulator_count_voltages(ts_data->vdd) > 0) {
		retval = regulator_set_voltage(ts_data->vdd,
			IT_VTG_MIN_UV, IT_VTG_MAX_UV);
		if (retval) {
			dev_err(&ts_data->client->dev,
				"regulator set_vtg failed retval =%d\n",
				retval);
			goto err_set_vtg_vdd;
		}
	}

	ts_data->avdd = devm_regulator_get(&ts_data->client->dev, "avdd");
	if (IS_ERR(ts_data->avdd)) {
		dev_err(&ts_data->client->dev,
				"%s: Failed to get i2c regulator\n", __func__);
		retval = PTR_ERR(ts_data->avdd);
		goto err_get_vtg_i2c;
	}

	if (regulator_count_voltages(ts_data->avdd) > 0) {
		retval = regulator_set_voltage(ts_data->avdd,
			IT_I2C_VTG_MIN_UV, IT_I2C_VTG_MAX_UV);
		if (retval) {
			dev_err(&ts_data->client->dev,
				"reg set i2c vtg failed retval =%d\n",
				retval);
		goto err_set_vtg_i2c;
		}
	}

	return 0;

err_set_vtg_i2c:
err_get_vtg_i2c:
	if (regulator_count_voltages(ts_data->vdd) > 0)
		regulator_set_voltage(ts_data->vdd, 0, IT_VTG_MAX_UV);
err_set_vtg_vdd:
	return retval;

hw_shutdown:
	if (regulator_count_voltages(ts_data->vdd) > 0)
		regulator_set_voltage(ts_data->vdd, 0, IT_VTG_MAX_UV);
	if (regulator_count_voltages(ts_data->avdd) > 0)
		regulator_set_voltage(ts_data->avdd, 0, IT_I2C_VTG_MAX_UV);
	return 0;
};

static int it7259_power_on(struct it7259_ts_data *ts_data, bool on)
{
	int retval;

	if (on == false)
		goto power_off;

	retval = reg_set_optimum_mode_check(ts_data->vdd,
		IT_ACTIVE_LOAD_UA);
	if (retval < 0) {
		dev_err(&ts_data->client->dev,
			"Regulator vdd set_opt failed rc=%d\n",
			retval);
		return retval;
	}

	retval = regulator_enable(ts_data->vdd);
	if (retval) {
		dev_err(&ts_data->client->dev,
			"Regulator vdd enable failed rc=%d\n",
			retval);
		goto error_reg_en_vdd;
	}

	retval = reg_set_optimum_mode_check(ts_data->avdd,
		IT_I2C_ACTIVE_LOAD_UA);
	if (retval < 0) {
		dev_err(&ts_data->client->dev,
			"Regulator avdd set_opt failed rc=%d\n",
			retval);
		goto error_reg_opt_i2c;
	}

	retval = regulator_enable(ts_data->avdd);
	if (retval) {
		dev_err(&ts_data->client->dev,
			"Regulator avdd enable failed rc=%d\n",
			retval);
		goto error_reg_en_avdd;
	}

	return 0;

error_reg_en_avdd:
	reg_set_optimum_mode_check(ts_data->avdd, 0);
error_reg_opt_i2c:
	regulator_disable(ts_data->vdd);
error_reg_en_vdd:
	reg_set_optimum_mode_check(ts_data->vdd, 0);
	return retval;

power_off:
	reg_set_optimum_mode_check(ts_data->vdd, 0);
	regulator_disable(ts_data->vdd);
	reg_set_optimum_mode_check(ts_data->avdd, 0);
	regulator_disable(ts_data->avdd);

	return 0;
}

static int it7259_gpio_configure(struct it7259_ts_data *ts_data, bool on)
{
	int retval = 0;

	if (on) {
		if (gpio_is_valid(ts_data->pdata->irq_gpio)) {
			/* configure touchscreen irq gpio */
			retval = gpio_request(ts_data->pdata->irq_gpio,
					"ite_irq_gpio");
			if (retval) {
				dev_err(&ts_data->client->dev,
					"unable to request irq gpio [%d]\n",
					retval);
				goto err_irq_gpio_req;
			}

			retval = gpio_direction_input(ts_data->pdata->irq_gpio);
			if (retval) {
				dev_err(&ts_data->client->dev,
					"unable to set direction for irq gpio [%d]\n",
					retval);
				goto err_irq_gpio_dir;
			}
		} else {
			dev_err(&ts_data->client->dev,
				"irq gpio not provided\n");
				goto err_irq_gpio_req;
		}

		if (gpio_is_valid(ts_data->pdata->reset_gpio)) {
			/* configure touchscreen reset out gpio */
			retval = gpio_request(ts_data->pdata->reset_gpio,
					"ite_reset_gpio");
			if (retval) {
				dev_err(&ts_data->client->dev,
					"unable to request reset gpio [%d]\n",
					retval);
					goto err_reset_gpio_req;
			}

			retval = gpio_direction_output(
					ts_data->pdata->reset_gpio, 1);
			if (retval) {
				dev_err(&ts_data->client->dev,
					"unable to set direction for reset gpio [%d]\n",
					retval);
				goto err_reset_gpio_dir;
			}
			if (ts_data->pdata->low_reset)
				gpio_set_value(ts_data->pdata->reset_gpio, 0);
			else
				gpio_set_value(ts_data->pdata->reset_gpio, 1);
			msleep(ts_data->pdata->reset_delay);
		} else {
			dev_err(&ts_data->client->dev,
				"reset gpio not provided\n");
				goto err_reset_gpio_req;
		}
	} else {
		if (gpio_is_valid(ts_data->pdata->irq_gpio))
			gpio_free(ts_data->pdata->irq_gpio);
		if (gpio_is_valid(ts_data->pdata->reset_gpio)) {
			/*
			 * This is intended to save leakage current
			 * only. Even if the call(gpio_direction_input)
			 * fails, only leakage current will be more but
			 * functionality will not be affected.
			 */
			retval = gpio_direction_input(
					ts_data->pdata->reset_gpio);
			if (retval) {
				dev_err(&ts_data->client->dev,
					"unable to set direction for gpio reset [%d]\n",
					retval);
			}
			gpio_free(ts_data->pdata->reset_gpio);
		}
	}

	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);
err_reset_gpio_req:
err_irq_gpio_dir:
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
err_irq_gpio_req:
	return retval;
}

#ifdef	CONFIG_BEZEL_SUPPORT
static void it7259_fill_bezeldata(struct device *dev,
				struct bezel_data *bdata)
{
	bdata->inset = BEZEL_INSET_POS;
	bdata->angular_dist_thresh = ANGULAR_DISTANCE_THRESHOLD;
	bdata->thickness = 0;
	bdata->bezel_touch_status = false;
}
#endif

#if CONFIG_OF
static int it7259_get_dt_coords(struct device *dev, char *name,
				struct it7259_ts_platform_data *pdata)
{
	u32 coords[it7259_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != it7259_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (strcmp(name, "ite,panel-coords") == 0) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];

		if (pdata->panel_maxx == 0 || pdata->panel_minx > 0)
			rc = -EINVAL;
		else if (pdata->panel_maxy == 0 || pdata->panel_miny > 0)
			rc = -EINVAL;

		if (rc) {
			dev_err(dev, "Invalid panel resolution %d\n", rc);
			return rc;
		}
	} else if (strcmp(name, "ite,display-coords") == 0) {
		pdata->disp_minx = coords[0];
		pdata->disp_miny = coords[1];
		pdata->disp_maxx = coords[2];
		pdata->disp_maxy = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int it7259_parse_dt(struct device *dev,
				struct it7259_ts_platform_data *pdata)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np,
			"ite,reset-gpio", 0, &pdata->reset_gpio_flags);
	pdata->irq_gpio = of_get_named_gpio_flags(np,
			"ite,irq-gpio", 0, &pdata->irq_gpio_flags);

	rc = of_property_read_u32(np, "ite,num-fingers", &temp_val);
	if (!rc)
		pdata->num_of_fingers = temp_val;
	else if (rc != -EINVAL) {
		dev_err(dev, "Unable to read reset delay\n");
		return rc;
	}

	pdata->wakeup = of_property_read_bool(np, "ite,wakeup");
	pdata->palm_detect_en = of_property_read_bool(np, "ite,palm-detect-en");
	if (pdata->palm_detect_en) {
		rc = of_property_read_u32(np, "ite,palm-detect-keycode",
							&temp_val);
		if (!rc) {
			pdata->palm_detect_keycode = temp_val;
		} else {
			dev_err(dev, "Unable to read palm-detect-keycode\n");
			return rc;
		}
	}

	rc = of_property_read_string(np, "ite,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw image name %d\n", rc);
		return rc;
	}

	rc = of_property_read_string(np, "ite,cfg-name", &pdata->cfg_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read cfg image name %d\n", rc);
		return rc;
	}

	snprintf(ts_data->fw_name, MAX_BUFFER_SIZE, "%s",
		(pdata->fw_name != NULL) ? pdata->fw_name : FW_NAME);
	snprintf(ts_data->cfg_name, MAX_BUFFER_SIZE, "%s",
		(pdata->cfg_name != NULL) ? pdata->cfg_name : CFG_NAME);

	rc = of_property_read_u32(np, "ite,reset-delay", &temp_val);
	if (!rc)
		pdata->reset_delay = temp_val;
	else if (rc != -EINVAL) {
		dev_err(dev, "Unable to read reset delay\n");
		return rc;
	}

	rc = of_property_read_u32(np, "ite,avdd-lpm-cur", &temp_val);
	if (!rc) {
		pdata->avdd_lpm_cur = temp_val;
	} else if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read avdd lpm current value %d\n", rc);
		return rc;
	}

	pdata->low_reset = of_property_read_bool(np, "ite,low-reset");

	rc = it7259_get_dt_coords(dev, "ite,display-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = it7259_get_dt_coords(dev, "ite,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	return 0;
}
#else
static inline int it7259_ts_parse_dt(struct device *dev,
				struct it7259_ts_platform_data *pdata)
{
	return 0;
}
#endif

static int it7259_ts_pinctrl_init(struct it7259_ts_data *ts_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts_data->ts_pinctrl = devm_pinctrl_get(&(ts_data->client->dev));
	if (IS_ERR_OR_NULL(ts_data->ts_pinctrl)) {
		retval = PTR_ERR(ts_data->ts_pinctrl);
		dev_dbg(&ts_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	ts_data->pinctrl_state_active
		= pinctrl_lookup_state(ts_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ts_data->pinctrl_state_active)) {
		retval = PTR_ERR(ts_data->pinctrl_state_active);
		dev_err(&ts_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	ts_data->pinctrl_state_suspend
		= pinctrl_lookup_state(ts_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ts_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(ts_data->pinctrl_state_suspend);
		dev_err(&ts_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	ts_data->pinctrl_state_release
		= pinctrl_lookup_state(ts_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(ts_data->pinctrl_state_release)) {
		retval = PTR_ERR(ts_data->pinctrl_state_release);
		dev_dbg(&ts_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ts_data->ts_pinctrl);
err_pinctrl_get:
	ts_data->ts_pinctrl = NULL;
	return retval;
}



/*
 * this code to get versions from the chip via i2c transactions, and save
 * them in driver data structure.
 */
static void it7259_get_chip_versions(struct it7259_ts_data *ts_data)
{
        static const u8 cmd_read_fw_ver[] = {CMD_READ_VERSIONS,
                                                SUB_CMD_READ_FIRMWARE_VERSION};
        static const u8 cmd_read_cfg_ver[] = {CMD_READ_VERSIONS,
                                                SUB_CMD_READ_CONFIG_VERSION};
        u8 ver_fw[VERSION_LENGTH], ver_cfg[VERSION_LENGTH];
        int ret;

        ret = it7259_i2c_write(ts_data, BUF_COMMAND, cmd_read_fw_ver,
                                        sizeof(cmd_read_fw_ver));
        if (ret == IT_I2C_WRITE_RET) {
                /*
                 * Sometimes, the controller may not respond immediately after
                 * writing the command, so wait for device to get ready.
                 */
                ret = it7259_wait_device_ready(ts_data, true, false);
                if (ret < 0)
                        dev_err(&ts_data->client->dev,
                                "failed to read chip status %d\n", ret);

                ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE,
                                        ver_fw, VERSION_LENGTH);
                if (ret == IT_I2C_READ_RET)
                        memcpy(ts_data->fw_ver, ver_fw + (5 * sizeof(u8)),
                                        VER_BUFFER_SIZE * sizeof(u8));
                else
                        dev_err(&ts_data->client->dev,
                                "failed to read fw-ver from chip %d\n", ret);
        } else {
                dev_err(&ts_data->client->dev,
                                "failed to write fw-read command %d\n", ret);
        }

        ret = it7259_i2c_write(ts_data, BUF_COMMAND, cmd_read_cfg_ver,
                                        sizeof(cmd_read_cfg_ver));
        if (ret == IT_I2C_WRITE_RET) {
                /*
                 * Sometimes, the controller may not respond immediately after
                 * writing the command, so wait for device to get ready.
                 */
                ret = it7259_wait_device_ready(ts_data, true, false);
                if (ret < 0)
                        dev_err(&ts_data->client->dev,
                                "failed to read chip status %d\n", ret);

                ret = it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE,
                                        ver_cfg, VERSION_LENGTH);
                if (ret == IT_I2C_READ_RET)
                        memcpy(ts_data->cfg_ver, ver_cfg + (1 * sizeof(u8)),
                                        VER_BUFFER_SIZE * sizeof(u8));
                else
                        dev_err(&ts_data->client->dev,
                                "failed to read cfg-ver from chip %d\n", ret);
        } else {
                dev_err(&ts_data->client->dev,
                                "failed to write cfg-read command %d\n", ret);
        }

        dev_info(&ts_data->client->dev, "Current fw{%X.%X.%X.%X} cfg{%X.%X.%X.%X}\n",
                ts_data->fw_ver[0], ts_data->fw_ver[1], ts_data->fw_ver[2],
                ts_data->fw_ver[3], ts_data->cfg_ver[0], ts_data->cfg_ver[1],
                ts_data->cfg_ver[2], ts_data->cfg_ver[3]);
}


static int it7259_ts_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	static const uint8_t cmd_start[] = {CMD_UNKNOWN_7};
	struct it7259_ts_data *ts_data;
	struct it7259_ts_platform_data *pdata;
#ifdef	CONFIG_BEZEL_SUPPORT
	struct bezel_data *bdata;
#endif
	uint8_t rsp[2];
	int ret = -1, err;
	struct dentry *temp;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	ts_data = devm_kzalloc(&client->dev, sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data)
		return -ENOMEM;

	ts_data->client = client;
	i2c_set_clientdata(client, ts_data);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		ret = it7259_parse_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		dev_err(&client->dev, "No platform data found\n");
		return -ENOMEM;
	}

	ts_data->palm_pressed = false;

#ifdef	CONFIG_BEZEL_SUPPORT
	bdata = devm_kzalloc(&client->dev, sizeof(*bdata), GFP_KERNEL);
	if (!bdata)
		return -ENOMEM;
	it7259_fill_bezeldata(&client->dev, bdata);
	ts_data->bdata = bdata;
#endif

	ts_data->pdata = pdata;

	ret = it7259_regulator_configure(ts_data, true);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to configure regulators\n");
		goto err_reg_configure;
	}

	ret = it7259_power_on(ts_data, true);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to power on\n");
		goto err_power_device;
	}

	/*
	 * After enabling regulators, controller needs a delay to come to
	 * an active state.
	 */
	msleep(DELAY_VTG_REG_EN);

	ret = it7259_ts_pinctrl_init(ts_data);
	if (!ret && ts_data->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret = pinctrl_select_state(ts_data->ts_pinctrl,
					ts_data->pinctrl_state_active);
		if (ret < 0) {
			dev_err(&ts_data->client->dev,
				"failed to select pin to active state %d",
				ret);
		}
	} else {
		ret = it7259_gpio_configure(ts_data, true);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to configure gpios\n");
			goto err_gpio_config;
		}
	}

	ret = it7259_ts_chip_identify(ts_data);
	if (ret) {
		dev_err(&client->dev, "Failed to identify chip %d!!!", ret);
		if(Force_Upgrade(ts_data))
			printk("IT7259 force upgrade fail, please check hardware\n");
		else
			printk("IT7259 force upgrade success!!\n");
	}

	msleep(DELAY_I2C_TRANSATION);
	it7259_get_chip_versions(ts_data);

	ts_data->input_dev = input_allocate_device();
	if (!ts_data->input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	/* Initialize mutex for fw and cfg upgrade */
	mutex_init(&ts_data->fw_cfg_mutex);

	ts_data->fw_active = true;
	ts_data->input_dev->name = DEVICE_NAME;
	ts_data->input_dev->phys = "I2C";
	ts_data->input_dev->id.bustype = BUS_I2C;
	ts_data->input_dev->id.vendor = 0x0001;
	ts_data->input_dev->id.product = 0x7259;
	set_bit(EV_SYN, ts_data->input_dev->evbit);
	set_bit(EV_KEY, ts_data->input_dev->evbit);
	set_bit(EV_ABS, ts_data->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, ts_data->input_dev->propbit);
	set_bit(BTN_TOUCH, ts_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, ts_data->input_dev->keybit);
	input_set_abs_params(ts_data->input_dev, ABS_MT_POSITION_X,
		ts_data->pdata->disp_minx, ts_data->pdata->disp_maxx, 0, 0);
	input_set_abs_params(ts_data->input_dev, ABS_MT_POSITION_Y,
		ts_data->pdata->disp_miny, ts_data->pdata->disp_maxy, 0, 0);
        input_set_abs_params(ts_data->input_dev, ABS_DISTANCE, 0,
                        HOVER_Z_MAX, 0, 0);
	input_mt_init_slots(ts_data->input_dev,
					ts_data->pdata->num_of_fingers, 0);

	input_set_drvdata(ts_data->input_dev, ts_data);

	if (pdata->wakeup) {
		set_bit(KEY_WAKEUP, ts_data->input_dev->keybit);
		INIT_WORK(&ts_data->work_pm_relax, it7259_ts_work_func);
		device_init_wakeup(&client->dev, pdata->wakeup);
	}

	if (pdata->palm_detect_en)
		set_bit(ts_data->pdata->palm_detect_keycode,
					ts_data->input_dev->keybit);

	if (input_register_device(ts_data->input_dev)) {
		dev_err(&client->dev, "failed to register input device\n");
		goto err_input_register;
	}

#ifdef	CONFIG_BEZEL_SUPPORT
	if (bezel_register_device(&client->dev, ts_data->pdata->panel_maxx)) {
		dev_err(&client->dev, "failed to register Bezel\n");
		goto err_irq_reg;
	}
#endif

	if (request_threaded_irq(client->irq, NULL, it7259_ts_threaded_handler,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ts_data)) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_irq_reg;
	}

	if (sysfs_create_group(&(client->dev.kobj), &it7259_attr_group)) {
		dev_err(&client->dev, "failed to register sysfs #2\n");
		goto err_sysfs_grp_create;
	}

#if defined(CONFIG_FB)
	ts_data->fb_notif.notifier_call = fb_notifier_callback;

	ret = fb_register_client(&ts_data->fb_notif);
	if (ret)
		dev_err(&client->dev, "Unable to register fb_notifier %d\n",
					ret);
#endif
	
	it7259_i2c_write_no_ready_check(ts_data, BUF_COMMAND, cmd_start,
							sizeof(cmd_start));
	msleep(pdata->reset_delay);
	it7259_i2c_read_no_ready_check(ts_data, BUF_RESPONSE, rsp, sizeof(rsp));
	msleep(pdata->reset_delay);

	ts_data->dir = debugfs_create_dir(DEBUGFS_DIR_NAME, NULL);
	if (ts_data->dir == NULL || IS_ERR(ts_data->dir)) {
		dev_err(&client->dev,
			"%s: Failed to create debugfs directory, ret = %ld\n",
			__func__, PTR_ERR(ts_data->dir));
		ret = PTR_ERR(ts_data->dir);
		goto err_create_debugfs_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, ts_data->dir,
					ts_data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		dev_err(&client->dev,
			"%s: Failed to create suspend debugfs file, ret = %ld\n",
			__func__, PTR_ERR(temp));
		ret = PTR_ERR(temp);
		goto err_create_debugfs_file;
	}

	return 0;

err_create_debugfs_file:
	debugfs_remove_recursive(ts_data->dir);
err_create_debugfs_dir:
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts_data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif
	sysfs_remove_group(&(client->dev.kobj), &it7259_attr_group);

err_sysfs_grp_create:
	free_irq(client->irq, ts_data);

err_irq_reg:
	input_unregister_device(ts_data->input_dev);

err_input_register:
	if (pdata->wakeup) {
		cancel_work_sync(&ts_data->work_pm_relax);
		device_init_wakeup(&client->dev, false);
	}
	if (ts_data->input_dev)
		input_free_device(ts_data->input_dev);
	ts_data->input_dev = NULL;

err_input_alloc:
	if (ts_data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(ts_data->pinctrl_state_release)) {
			devm_pinctrl_put(ts_data->ts_pinctrl);
			ts_data->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(ts_data->ts_pinctrl,
					ts_data->pinctrl_state_release);
			if (err)
				dev_err(&ts_data->client->dev,
					"failed to select relase pinctrl state %d\n",
					err);
		}
	} else {
		if (gpio_is_valid(pdata->reset_gpio))
			gpio_free(pdata->reset_gpio);
		if (gpio_is_valid(pdata->irq_gpio))
			gpio_free(pdata->irq_gpio);
	}

err_gpio_config:
	it7259_power_on(ts_data, false);

err_power_device:
	it7259_regulator_configure(ts_data, false);

err_reg_configure:
	return ret;
}

static int it7259_ts_remove(struct i2c_client *client)
{
	struct it7259_ts_data *ts_data = i2c_get_clientdata(client);
	int ret;

	debugfs_remove_recursive(ts_data->dir);
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts_data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif
	sysfs_remove_group(&(client->dev.kobj), &it7259_attr_group);
	free_irq(client->irq, ts_data);
	input_unregister_device(ts_data->input_dev);
	if (ts_data->input_dev)
		input_free_device(ts_data->input_dev);
	ts_data->input_dev = NULL;
	if (ts_data->pdata->wakeup) {
		cancel_work_sync(&ts_data->work_pm_relax);
		device_init_wakeup(&client->dev, false);
	}
	if (ts_data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(ts_data->pinctrl_state_release)) {
			devm_pinctrl_put(ts_data->ts_pinctrl);
			ts_data->ts_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts_data->ts_pinctrl,
					ts_data->pinctrl_state_release);
			if (ret)
				dev_err(&ts_data->client->dev,
					"failed to select relase pinctrl state %d\n",
					ret);
		}
	} else {
		if (gpio_is_valid(ts_data->pdata->reset_gpio))
			gpio_free(ts_data->pdata->reset_gpio);
		if (gpio_is_valid(ts_data->pdata->irq_gpio))
			gpio_free(ts_data->pdata->irq_gpio);
	}
	it7259_power_on(ts_data, false);
	it7259_regulator_configure(ts_data, false);

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct it7259_ts_data *ts_data = container_of(self,
					struct it7259_ts_data, fb_notif);
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && ts_data && ts_data->client) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK)
				it7259_ts_resume(&(ts_data->client->dev));
			else if (*blank == FB_BLANK_POWERDOWN ||
					*blank == FB_BLANK_VSYNC_SUSPEND)
				it7259_ts_suspend(&(ts_data->client->dev));
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_PM
static int it7259_ts_resume(struct device *dev)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	int retval;

	if (device_may_wakeup(dev)) {
		if (ts_data->in_low_power_mode) {
			/* Set active current for the avdd regulator */
			if (ts_data->pdata->avdd_lpm_cur) {
				retval = reg_set_optimum_mode_check(
						ts_data->avdd,
						IT_I2C_ACTIVE_LOAD_UA);
				if (retval < 0)
					dev_err(dev, "Regulator avdd set_opt failed at resume rc=%d\n",
					retval);
			}

			ts_data->in_low_power_mode = false;
			disable_irq_wake(ts_data->client->irq);
		}
		return 0;
	}

	if (ts_data->ts_pinctrl) {
		retval = pinctrl_select_state(ts_data->ts_pinctrl,
				ts_data->pinctrl_state_active);
		if (retval < 0) {
			dev_err(dev, "Cannot get default pinctrl state %d\n",
				retval);
			goto err_pinctrl_select_suspend;
		}
	}

	enable_irq(ts_data->client->irq);
	ts_data->suspended = false;
	return 0;

err_pinctrl_select_suspend:
	return retval;
}

static int it7259_ts_suspend(struct device *dev)
{
	struct it7259_ts_data *ts_data = dev_get_drvdata(dev);
	int retval;

	if (ts_data->fw_cfg_uploading) {
		dev_dbg(dev, "Fw/cfg uploading. Can't go to suspend.\n");
		return -EBUSY;
	}

	if (device_may_wakeup(dev)) {
		if (!ts_data->in_low_power_mode) {
			if(ts_data->fw_active) {
				/* put the device in low power idle mode */
				retval = it7259_ts_chip_low_power_mode(ts_data,
						PWR_CTL_LOW_POWER_MODE);
				if (retval)
					dev_err(dev, "Can't go to low power mode %d\n",
							retval);
				ts_data->fw_active = false;
			}

			/* Set lpm current for avdd regulator */
			if (ts_data->pdata->avdd_lpm_cur) {
				retval = reg_set_optimum_mode_check(
						ts_data->avdd,
						ts_data->pdata->avdd_lpm_cur);
				if (retval < 0)
					dev_err(dev, "Regulator avdd set_opt failed at suspend rc=%d\n",
						retval);
			}

			ts_data->in_low_power_mode = true;
			enable_irq_wake(ts_data->client->irq);
		}
		return 0;
	}

	disable_irq(ts_data->client->irq);

	it7259_ts_release_all(ts_data);

	if (ts_data->ts_pinctrl) {
		retval = pinctrl_select_state(ts_data->ts_pinctrl,
				ts_data->pinctrl_state_suspend);
		if (retval < 0) {
			dev_err(dev, "Cannot get idle pinctrl state %d\n",
				retval);
			goto err_pinctrl_select_suspend;
		}
	}

	ts_data->suspended = true;

	return 0;

err_pinctrl_select_suspend:
	return retval;
}

static const struct dev_pm_ops it7259_ts_dev_pm_ops = {
	.suspend = it7259_ts_suspend,
	.resume  = it7259_ts_resume,
};
#else
static int it7259_ts_resume(struct device *dev)
{
	return 0;
}

static int it7259_ts_suspend(struct device *dev)
{
	return 0;
}
#endif

static const struct i2c_device_id it7259_ts_id[] = {
	{ DEVICE_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, it7259_ts_id);

static const struct of_device_id it7259_match_table[] = {
	{ .compatible = "ite,it7259_ts",},
	{},
};

static struct i2c_driver it7259_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
		.of_match_table = it7259_match_table,
#ifdef CONFIG_PM
		.pm = &it7259_ts_dev_pm_ops,
#endif
	},
	.probe = it7259_ts_probe,
	.remove = it7259_ts_remove,
	.id_table = it7259_ts_id,
};

module_i2c_driver(it7259_ts_driver);

MODULE_DESCRIPTION("it7259 Touchscreen Driver");
MODULE_LICENSE("GPL v2");
