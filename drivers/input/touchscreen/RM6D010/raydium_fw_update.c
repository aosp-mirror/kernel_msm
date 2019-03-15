/* drivers/input/touchscreen/raydium_burn_ts.c
 *
 * Raydium TouchScreen driver.
 *
 * Copyright (c) 2010  Raydium tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <asm/traps.h>
#include <linux/firmware.h>
#include "raydium_driver.h"
#include "rad_fw_image_10.h"
#include "rad_fw_image_11.h"
#include "rad_fw_image_20.h"
#include "rad_fw_image_21.h"

void raydium_mem_table_init(unsigned short u16_id)
{
	pr_info("[touch]Raydium table init 0x%x\n", u16_id);

	switch (u16_id) {
	case RAD_10:
	case RAD_11:
		g_rad_boot_image = kzalloc(RAD_BOOT_1X_SIZE + RAD_INIT_1X_SIZE,
			GFP_KERNEL);
		g_rad_init_image = kzalloc(RAD_INIT_1X_SIZE, GFP_KERNEL);
		g_rad_fw_image = kzalloc(RAD_FW_1X_SIZE, GFP_KERNEL);
		g_rad_para_image = kzalloc(RAD_PARA_1X_SIZE, GFP_KERNEL);
		g_rad_testfw_image = kzalloc(RAD_TESTFW_1X_SIZE, GFP_KERNEL);
		g_rad_testpara_image = kzalloc(RAD_PARA_1X_SIZE, GFP_KERNEL);
		break;
	case RAD_20:
	case RAD_21:
		g_rad_boot_image = kzalloc(RAD_BOOT_2X_SIZE, GFP_KERNEL);
		g_rad_init_image = kzalloc(RAD_INIT_2X_SIZE, GFP_KERNEL);
		g_rad_fw_image = kzalloc(RAD_FW_2X_SIZE, GFP_KERNEL);
		g_rad_para_image = kzalloc(RAD_PARA_2X_SIZE, GFP_KERNEL);
		g_rad_testfw_image = kzalloc(RAD_TESTFW_2X_SIZE, GFP_KERNEL);
		g_rad_testpara_image = kzalloc(RAD_PARA_2X_SIZE, GFP_KERNEL);
		break;
	}
}


void raydium_mem_table_setting(unsigned short u16_id)
{
	pr_info("[touch]Raydium ID is 0x%x\n", u16_id);

	switch (u16_id) {
	case RAD_10:
		memcpy(g_rad_boot_image,
			u8_rad_boot_10, RAD_BOOT_1X_SIZE);
		memcpy(g_rad_boot_image + RAD_BOOT_1X_SIZE,
			u8_rad_init_10, 0x1FC);
		memcpy(g_rad_init_image,
			u8_rad_init_10, RAD_INIT_1X_SIZE);
		memcpy(g_rad_fw_image,
			u8_rad_fw_10 + 0x1000, RAD_FW_1X_SIZE);
		memcpy(g_rad_para_image,
			u8_rad_fw_10 + 0xF1C, RAD_PARA_1X_SIZE);
		memcpy(g_rad_testfw_image,
			u8_rad_testfw_10 + 0xA00, RAD_TESTFW_1X_SIZE);
		memcpy(g_rad_testpara_image,
			u8_rad_testfw_10 + 0xF1C, RAD_PARA_1X_SIZE);
		break;
	case RAD_11:
		memcpy(g_rad_boot_image,
			u8_rad_boot_11, RAD_BOOT_1X_SIZE);
		memcpy(g_rad_boot_image + RAD_BOOT_1X_SIZE,
			u8_rad_init_11, 0x1FC);
		memcpy(g_rad_init_image,
			u8_rad_init_11, RAD_INIT_1X_SIZE);
		memcpy(g_rad_fw_image,
			u8_rad_fw_11 + 0x1000, RAD_FW_1X_SIZE);
		memcpy(g_rad_para_image,
			u8_rad_fw_11 + 0xF1C, RAD_PARA_1X_SIZE);
		memcpy(g_rad_testfw_image,
			u8_rad_testfw_11 + 0xA00, RAD_TESTFW_1X_SIZE);
		memcpy(g_rad_testpara_image,
			u8_rad_testfw_11 + 0xF1C, RAD_PARA_1X_SIZE);
		break;
	case RAD_20:
		memcpy(g_rad_boot_image,
			u8_rad_boot_20, RAD_BOOT_2X_SIZE);
		memcpy(g_rad_init_image,
			u8_rad_init_20, RAD_INIT_2X_SIZE);
		memcpy(g_rad_fw_image,
			u8_rad_fw_20, RAD_FW_2X_SIZE);
		memcpy(g_rad_para_image,
			u8_rad_para_20, RAD_PARA_2X_SIZE);
		memcpy(g_rad_testfw_image,
			u8_rad_testfw_20, RAD_FW_2X_SIZE);
		memcpy(g_rad_testfw_image + RAD_FW_2X_SIZE,
			u8_rad_testpara_20, RAD_PARA_2X_SIZE);
		memcpy(g_rad_testpara_image,
			u8_rad_testpara_20, RAD_PARA_2X_SIZE);
		break;
	case RAD_21:
		memcpy(g_rad_boot_image,
			u8_rad_boot_21, RAD_BOOT_2X_SIZE);
		memcpy(g_rad_init_image,
			u8_rad_init_21, RAD_INIT_2X_SIZE);
		memcpy(g_rad_fw_image,
			u8_rad_fw_21, RAD_FW_2X_SIZE);
		memcpy(g_rad_para_image,
			u8_rad_para_21, RAD_PARA_2X_SIZE);
		memcpy(g_rad_testfw_image,
			u8_rad_testfw_21, RAD_FW_2X_SIZE);
		memcpy(g_rad_testfw_image + RAD_FW_2X_SIZE,
			u8_rad_testpara_21, RAD_PARA_2X_SIZE);
		memcpy(g_rad_testpara_image,
			u8_rad_testpara_21, RAD_PARA_2X_SIZE);
		break;
	}

	g_u8_table_setting = 0;
}

static unsigned int bits_reverse(unsigned int u32_num, unsigned int bit_num)
{
	unsigned int reverse = 0, u32_i;

	for (u32_i = 0; u32_i < bit_num; u32_i++) {
		if (u32_num & (1 << u32_i))
			reverse |= 1 << ((bit_num - 1) - u32_i);
	}
	return reverse;
}

static unsigned int rc_crc32(const char *buf, unsigned int u32_len,
			     unsigned int u32_crc)
{
	unsigned int u32_i;
	unsigned char u8_flash_byte, u8_current, u8_j;

	for (u32_i = 0; u32_i < u32_len; u32_i++) {
		u8_flash_byte = buf[u32_i];
		u8_current = (unsigned char)bits_reverse(u8_flash_byte, 8);
		for (u8_j = 0; u8_j < 8; u8_j++) {
			if ((u32_crc ^ u8_current) & 0x01)
				u32_crc = (u32_crc >> 1) ^ 0xedb88320;
			else
				u32_crc >>= 1;
			u8_current >>= 1;
		}
	}
	return u32_crc;
}

int wait_fw_state(struct i2c_client *client, unsigned int u32_addr,
			 unsigned int u32_state, unsigned long u32_delay_us,
			 unsigned short u16_retry)
{
	unsigned char u8_buf[4];
	unsigned int u32_read_data;
	unsigned int u32_min_delay_us = u32_delay_us - 500;
	unsigned int u32_max_delay_us = u32_delay_us + 500;

	do {
		if (raydium_i2c_pda_read(client, u32_addr, u8_buf, 4) == ERROR)
			return ERROR;

		memcpy(&u32_read_data, u8_buf, 4);
		u16_retry--;
		usleep_range(u32_min_delay_us, u32_max_delay_us);
	} while ((u32_read_data != u32_state) && (u16_retry != 0));

	if (u32_read_data != u32_state) {
		pr_err("[touch]confirm data error : 0x%x\n", u32_read_data);
		return ERROR;
	}

	return SUCCESS;
}

int wait_irq_state(struct i2c_client *client, unsigned int retry_time,
				unsigned int u32_delay_us)
{
	int i32_ret = SUCCESS;
	unsigned int u32_retry;
	unsigned int u32_irq_value;
	unsigned int u32_min_delay_us = u32_delay_us - 500;
	unsigned int u32_max_delay_us = u32_delay_us + 500;
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);

	u32_retry = retry_time;
	while (u32_retry != 0 && (g_u8_raydium_flag & INT_FLAG) != INT_FLAG) {
		usleep_range(u32_min_delay_us, u32_max_delay_us);
		u32_retry--;
	}

	if ((g_u8_raydium_flag & INT_FLAG) == INT_FLAG) {
		u32_retry = retry_time;
		u32_irq_value = 0;
		while (u32_retry != 0 && u32_irq_value != 1) {
			u32_irq_value = gpio_get_value(ts->irq_gpio);
			usleep_range(u32_min_delay_us, u32_max_delay_us);
			u32_retry--;
		}
		pr_info("[touch]irq_value is %d\n", u32_irq_value);
		g_u8_raydium_flag &= ~INT_FLAG;
	}

	if (u32_retry == 0) {
		pr_err("[touch]%s, FW not ready, retry error!\n", __func__);
		i32_ret = ERROR;
	}

	return i32_ret;
}

int raydium_do_software_reset(struct i2c_client *client)
{
	int i32_ret = SUCCESS;

	unsigned char u8_buf[4];
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);

	if (ts->id == RAD_10 || ts->id == RAD_11)
		raydium_irq_control(ts, ENABLE);

	/*SW reset*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	pr_info("[touch]SW reset\n");
	i32_ret = raydium_i2c_pda_write(client, 0x40000004, u8_buf, 4);
	if (i32_ret < 0)
		goto exit;

	if (ts->id == RAD_20 || ts->id == RAD_21)
		msleep(25);
	else if (ts->id == RAD_10 || ts->id == RAD_11)
		i32_ret = wait_irq_state(client, 1000, 2000);

exit:
	return i32_ret;
}

static int raydium_check_fw_ready(struct i2c_client *client)
{
	int i32_ret = SUCCESS;
	unsigned int u32_retry = 400;
	unsigned char u8_buf[4];

	u8_buf[1] = 0;
	while (u8_buf[1] != 0x40 && u32_retry != 0) {
		i32_ret = raydium_i2c_pda_read(client, 0x50000918, u8_buf, 4);
		if (i32_ret < 0)
			goto exit;

		u32_retry--;
		usleep_range(4500, 5500);
	}

	if (u32_retry == 0) {
		pr_err("[touch]%s, FW not ready, retry error!\n", __func__);
		i32_ret = ERROR;
	} else {
		pr_info("[touch]%s, FW is ready!!\n", __func__);
		usleep_range(4500, 5500);
	}

exit:
	return i32_ret;
}

int set_skip_load(struct i2c_client *client)
{
	int i32_ret = SUCCESS;
	unsigned int u32_retry_time = 1000;
	unsigned char u8_buf[4];

	/*Skip load*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x10;
	u8_buf[1] = 0x08;
	i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	i32_ret = raydium_do_software_reset(client);
	if (i32_ret < 0)
		pr_err("[touch]%s, SW reset error!\n", __func__);

	i32_ret = wait_fw_state(client, 0x20000214, 0x82, 2000, u32_retry_time);
	if (i32_ret < 0)
		pr_err("[touch]%s, wait_fw_state error!\n", __func__);

exit_upgrade:
	return i32_ret;
}

/* upgrade firmware with image file */
static int raydium_fw_upgrade_with_image(struct i2c_client *client,
		unsigned int u32_fw_addr,
		unsigned char u8_type)
{
	int i32_ret = ERROR;
	unsigned int u32_fw_size = 0;
	unsigned char *p_u8_firmware_data = NULL;
	unsigned int u32_write_offset = 0;
	unsigned short u16_write_length = 0;
	unsigned int u32_checksum = 0xFFFFFFFF;
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);

	switch (u8_type) {
	case RAYDIUM_INIT:
		u32_fw_size = 0x1fc;
		p_u8_firmware_data = g_rad_init_image;
		break;
	case RAYDIUM_PARA:
		if (ts->id == RAD_20 || ts->id == RAD_21)
			u32_fw_size = 0x158;
		else if (ts->id == RAD_10 || ts->id == RAD_11)
			u32_fw_size = 0xE0;
		p_u8_firmware_data = g_rad_para_image;
		break;
	case RAYDIUM_FIRMWARE:
		if (ts->id == RAD_20 || ts->id == RAD_21)
			u32_fw_size = 0x61fc;
		else if (ts->id == RAD_10 || ts->id == RAD_11)
			u32_fw_size = 0x4FFC;

		p_u8_firmware_data = g_rad_fw_image;
		break;
	case RAYDIUM_BOOTLOADER:
		if (ts->id == RAD_20 || ts->id == RAD_21)
			u32_fw_size = 0x7FC;
		else if (ts->id == RAD_10 || ts->id == RAD_11)
			u32_fw_size = 0x9FC;

		p_u8_firmware_data = g_rad_boot_image;
		break;
	case RAYDIUM_TEST_FW:
		if (ts->id == RAD_20 || ts->id == RAD_21)
			u32_fw_size = 0x635C;
		else if (ts->id == RAD_10 || ts->id == RAD_11)
			u32_fw_size = 0x55FC;

		p_u8_firmware_data = g_rad_testfw_image;
		break;
	}

	if ((ts->id == RAD_10 || ts->id == RAD_11) &&
		(u8_type == RAYDIUM_TEST_FW)) {
		pr_info("[touch]CRC 0x%08X\n",
			*(unsigned int *)(p_u8_firmware_data));
		u32_checksum = rc_crc32((p_u8_firmware_data + 4), u32_fw_size,
				       u32_checksum);
		u32_checksum = bits_reverse(u32_checksum, 32);
		memcpy(p_u8_firmware_data, &u32_checksum, 4);
	} else {
		pr_info("[touch]CRC 0x%08X\n",
			*(unsigned int *)(p_u8_firmware_data + u32_fw_size));

		u32_checksum = rc_crc32(p_u8_firmware_data,
			u32_fw_size, u32_checksum);
		u32_checksum = bits_reverse(u32_checksum, 32);
		memcpy((p_u8_firmware_data + u32_fw_size), &u32_checksum, 4);
	}

	pr_info("[touch]CRC result 0x%08X\n", u32_checksum);
	u32_fw_size += 4;

	u32_write_offset = 0;
	while (u32_write_offset < u32_fw_size) {
		if ((u32_write_offset + MAX_WRITE_PACKET_SIZE) < u32_fw_size)
			u16_write_length = MAX_WRITE_PACKET_SIZE;
		else
			u16_write_length =
				(unsigned short)
				(u32_fw_size - u32_write_offset);

		i32_ret = raydium_i2c_pda_write(
			      client,
			      (u32_fw_addr + u32_write_offset),
			      (p_u8_firmware_data + u32_write_offset),
			      u16_write_length);
		if (i32_ret < 0)
			goto exit_upgrate;

		u32_write_offset += (unsigned long)u16_write_length;
	}
	u32_fw_addr += u32_write_offset;

exit_upgrate:
	if (i32_ret < 0) {
		pr_err("[touch]upgrade failed\n");
		return i32_ret;
	} else {
		pr_info("[touch]upgrade success\n");
		return 0;
	}
}

static int raydium_boot_upgrade_1X(struct i2c_client *client)
{
	int i32_ret = SUCCESS;
	unsigned char u8_buf[4];

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x10;
	u8_buf[1] = 0x08;
	raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	raydium_i2c_pda_write(client, 0x40000004, u8_buf, 4);
	msleep(50);

	/*set mcu hold*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x20;
	raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	u8_buf[0] = 0x01;
	raydium_i2c_pda_write(client, 0x40000004, u8_buf, 4);
	msleep(50);

	/*WRT boot-loader to PRAM first*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x50000900, u8_buf, 4);

	/*Sending bootloader*/
	i32_ret = raydium_fw_upgrade_with_image(client, 0x0000,
					    RAYDIUM_BOOTLOADER);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = raydium_i2c_pda_read(client, 0x40000000, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	u8_buf[3] |= 0x40;
	i32_ret = raydium_i2c_pda_write(client, 0x40000000, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = raydium_i2c_pda_read(client, 0x40000014, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	u8_buf[0] |= 0x04;
	u8_buf[1] |= 0x04;
	i32_ret = raydium_i2c_pda_write(client, 0x40000014, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*Skip load*/
	i32_ret = set_skip_load(client);
	if (i32_ret < 0)
		pr_err("[touch]%s, set skip_load error!\n", __func__);

exit_upgrade:
	return i32_ret;
}

static int raydium_boot_upgrade_2X(struct i2c_client *client)
{
	int i32_ret = SUCCESS;
	unsigned char u8_buf[4];
	unsigned int u32_read_data, u32_crc_result;

	/*set mcu hold*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x20;
	i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	u8_buf[0] = 0x01;
	i32_ret = raydium_i2c_pda_write(client, 0x40000004, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	msleep(25);

	/*WRT boot-loader to PRAM first*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x50000900, u8_buf, 4);

	/*Sending bootloader*/
	i32_ret = raydium_fw_upgrade_with_image(client, 0x0000,
					    RAYDIUM_BOOTLOADER);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*check pram bootloader crc*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[2] = 0xFB;
	u8_buf[3] = 0x07;
	i32_ret = raydium_i2c_pda_write(client, 0x50000974, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	i32_ret = raydium_i2c_pda_read(client, 0x5000094C, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	u8_buf[3] |= 0x81;
	i32_ret = raydium_i2c_pda_write(client, 0x5000094C, u8_buf, 4);
	i32_ret = raydium_i2c_pda_read(client, 0x50000978, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	memcpy(&u32_crc_result, u8_buf, 4);
	i32_ret = raydium_i2c_pda_read(client, 0x7FC, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	memcpy(&u32_read_data, u8_buf, 4);
	if (u32_read_data != u32_crc_result) {
		pr_err("[touch]check pram bootloader crc fail!!\n");
		pr_err("[touch]u32_crc_result 0x%x\n", u32_crc_result);
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*release mcu hold*/
	/*Skip load*/
	i32_ret = set_skip_load(client);
	if (i32_ret < 0)
		pr_err("[touch]%s, set skip_load error!\n", __func__);

exit_upgrade:
	return i32_ret;
}

/* Raydium fireware upgrade flow */
static int raydium_fw_upgrade_1X(struct i2c_client *client,
			      unsigned char u8_type,
			      unsigned char u8_check_crc)
{
	int i32_ret = 0;
	unsigned char u8_buf[4];
	unsigned short u16_retry = 1000;
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);

	/*##### wait for boot-loader start #####*/
	pr_info("[touch]Type is %x\n", u8_type);
	/*#set main state as burning mode, normal init state*/
	/* #sync_data:200h
	 * main_state:204h
	 * normal_state:208h
	 * burning_state:20Ch
	 */
	/* #sync_data:210h
	 * cmd_type:210h
	 * ret_data:214h
	 * test_mode:218h
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	i32_ret = raydium_i2c_pda_write(client, 0x20000204, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000208, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x2000020C, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	i32_ret = raydium_i2c_pda_write(client, 0x20000218, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#confirm in burn mode*/
	if (wait_fw_state(client, 0x50000900, 63, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, confirm in burn mode\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*Clear BL_CRC*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x10;
	i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#write PRAM relative data*/
	/*#set PRAM type (at 0x50000904), wrt param code*/
	/* #init_code:0x01,
	 * baseline:0x02
	 * COMP:0x04
	 * param:0x08
	 * FW:0x10
	 * bootloader:0x20
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = u8_type;
	i32_ret = raydium_i2c_pda_write(client, 0x50000904, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#set PRAM addr (at 'h5000_0908)*/
	/* #init_code:0x800
	 * Baseline:0xA00
	 * COMP:0xCD4
	 * para:0xF1C
	 * FW:0x1000
	 * BOOT:0x5000
	*/
	/*#set PRAM length (at 'h5000_090C)*/
	if (u8_type == RAYDIUM_INIT) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x08;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x02;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_BASELINE) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x04;
		u8_buf[1] = 0x0a;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0xd0;
		u8_buf[1] = 0x02;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_COMP) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0xd4;
		u8_buf[1] = 0x0c;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x48;
		u8_buf[1] = 0x02;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_PARA) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x1c;
		u8_buf[1] = 0x0f;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0xe4;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_FIRMWARE) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x1C;
		u8_buf[1] = 0x0F;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0xE4;
		u8_buf[1] = 0x50;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_BOOTLOADER) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x40;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x0A;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	/*#set sync_data(0x20000200) = 0 as WRT data finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#Wait bootloader check addr and PRAM unlock*/
	/*#Confirm g_u8_sync_data.ret_data at 0x20000214 is SET_ADDR_READY*/
	if (wait_fw_state(client, 0x20000214, 161, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, SET_ADDR_READY\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#Confirm g_u8_sync_data.cmd_type at 0x20000210 is WRT_PRAM_DATA*/
	if (wait_fw_state(client, 0x20000210, 163, 1000, u16_retry) == ERROR) {
		pr_err("[touch]Error, WRT_PRAM_DATA\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#start write data to PRAM*/
	if (u8_type == RAYDIUM_INIT) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0x800,
						    RAYDIUM_INIT);
		if (i32_ret < 0)
			goto exit_upgrade;
	}  else if (u8_type == RAYDIUM_PARA) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0xF1C,
						    RAYDIUM_PARA);
		if (i32_ret < 0)
			goto exit_upgrade;
	} else if (u8_type == RAYDIUM_FIRMWARE) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0xF1C,
						    RAYDIUM_PARA);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_fw_upgrade_with_image(client, 0x1000,
						    RAYDIUM_FIRMWARE);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_BOOTLOADER) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0x4000,
						    RAYDIUM_BOOTLOADER);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	/*
	 *set sync_data(0x20000200) = 0 as WRT data finish
	 *bootloader check checksum
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*
	 * wait(checksum okay) ACK cmd
	 * (gu8I2CSyncData.cmd_type=0xa5 at 0x20000210)
	 */
	if (wait_fw_state(client, 0x20000210, 165, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, WRT_CHECKSUM_OK\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#confirm ACK cmd result(gu8I2CSyncData.ret_data=0xa5 at 0x20000214)*/
	if (wait_fw_state(client, 0x20000214, 165, 1000, u16_retry) == ERROR) {
		pr_err("[touch]Error, confirm ACK cmd result\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*
	 * set ACK return data = 0x5A
	 * adb shell "echo 20000210 1 A5 > /sys/bus/i2c/drivers/raydium_ts/
	 * 1-0039 raydium_i2c_pda_access"
	 * above command can be ignored, due to previous while loop has check
	 * its value.
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x5a;
	i32_ret = raydium_i2c_pda_write(client, 0x20000214, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#clr sync_data(0x20000200) = 0 as finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#wait for input unlock key*/
	if (wait_fw_state(client, 0x20000210, 168, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, wait for input unlock key\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#unlock key*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xd7;
	i32_ret = raydium_i2c_pda_write(client, 0x50000938, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa5;
	i32_ret = raydium_i2c_pda_write(client, 0x50000934, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x50000934, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa5;
	i32_ret = raydium_i2c_pda_write(client, 0x50000934, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x50000938, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#wrt return data as unlock value*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa8;
	i32_ret = raydium_i2c_pda_write(client, 0x20000214, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*raydium_irq_control(ts, ENABLE);*/

	/*#clr sync_data(0x20000200) = 0 as finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	/*
	i32_ret = wait_irq_state(client, 200, 10000);
	if (i32_ret == ERROR) {
		pr_err("[touch]Error, wait flash irq\n");
		goto exit_upgrade;
	}*/
	msleep(500);

	/* wait erase/wrt finish
	 * confirm burning_state result (gu8I2CSyncData.burning_state =
	 * BURNING_WRT_FLASH_FINISH at 0x2000020C)
	 */
	if (wait_fw_state(client, 0x2000020c, 6, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, wait erase/wrt finish\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}
	pr_info("[touch]Burn flash ok\n");

	/*############## Use Interrupt to wait sofware reset ##############*/
	/*#clr sync_data(0x20000200) = 0 as finish*/
	/*#Set bootloader flag*/
	/*#RAYDIUM_INTERRUPT_FLAG = 0x01*/
	/*#RAYDIUM_ENGINEER_MODE = 0x02*/
	/*g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;*/
	/*g_uc_raydium_flag |= RAYDIUM_ENGINEER_MODE;*/
	/*#enable INT*/
	/*dev_info(&ts->client->dev,
		 "[touch]ready to software reset => enable INT\n");*/
	raydium_irq_control(ts, ENABLE);

	if (u8_type == RAYDIUM_BOOTLOADER) {
		i32_ret = raydium_i2c_pda_read(client, 0x50000918, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		u8_buf[0] |= 0x10;
		i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	}

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#wait software reset finish*/
	/* confirm RAYDIUM_INTERRUPT_FLAG result (if raydium_flag = 1 =>
	 * pass)
	 */
	i32_ret = wait_irq_state(client, 200, 10000);
	if (i32_ret == ERROR)
		goto exit_upgrade;

	/* wait sw reset finished 0x20000214 = 0x82 */
	if (wait_fw_state(client, 0x20000214, 130, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, wait sw reset finished\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	if (u8_type == RAYDIUM_BASELINE || u8_type == RAYDIUM_INIT ||
	    u8_check_crc == 1) {
		/*#set test_mode = 1*/
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x01;
		i32_ret = raydium_i2c_pda_write(client, 0x20000218, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		/*#wait crc check finish*/
		if (wait_fw_state(client, 0x20000208, 2, 2000,
			u16_retry) == ERROR) {
			pr_err("[touch]Error, wait crc check finish\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}

		/*#crc check pass 0x20000214 = 0x81*/
		if (wait_fw_state(client, 0x20000214, 0x81, 1000,
			u16_retry) == ERROR) {
			pr_err("[touch]Error, confirm crc result\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}
	}

	/*#run to next step*/
	pr_info("[touch]Type 0x%x => Pass\n", u8_type);

	if (u8_check_crc) {
		/*#clr sync para*/
		memset(u8_buf, 0, sizeof(u8_buf));
		i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_i2c_pda_write(client, 0x20000210, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_i2c_pda_write(client, 0x20000214, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_i2c_pda_write(client, 0x20000218, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		/*wait bootloader to jump fw*/
		raydium_irq_control(ts, ENABLE);
		u8_buf[0] = 0x02;
		i32_ret = raydium_i2c_pda_write(client, 0x20000204, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = wait_irq_state(client, 200, 10000);
		if (i32_ret == ERROR)
			goto exit_upgrade;


		usleep_range(4500, 5500);
		raydium_i2c_pda_read(client, RAD_PDA2_CTRL_CMD, u8_buf, 4);
		u8_buf[0] |= RAD_ENABLE_PDA2 | RAD_ENABLE_SI2;
		raydium_i2c_pda_write(client, RAD_PDA2_CTRL_CMD, u8_buf, 4);
		raydium_i2c_pda_set_address(ts, 0x50000628, DISABLE);

		g_u8_i2c_mode = PDA2_MODE;

		pr_info("[touch]Burn FW finish!\n");
	}

exit_upgrade:
	return i32_ret;
}

/* Raydium fireware upgrade flow */
static int raydium_fw_upgrade_2X(struct i2c_client *client,
			      unsigned char u8_type,
			      unsigned char u8_check_crc)
{
	int i32_ret = 0;
	unsigned char u8_buf[4];
	unsigned short u16_retry = 1000;
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);

	/*##### wait for boot-loader start #####*/
	pr_info("[touch]Type is %x\n", u8_type);
	/*#set main state as burning mode, normal init state*/
	/* #sync_data:200h
	 * main_state:204h
	 * normal_state:208h
	 * burning_state:20Ch
	 */
	/* #sync_data:210h
	 * cmd_type:210h
	 * ret_data:214h
	 * test_mode:218h
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	i32_ret = raydium_i2c_pda_write(client, 0x20000204, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000208, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x2000020C, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	i32_ret = raydium_i2c_pda_write(client, 0x20000218, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#confirm in burn mode*/
	if (wait_fw_state(client, 0x50000900, 63, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, confirm in burn mode\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*Clear BL_CRC*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x10;
	i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#write PRAM relative data*/
	/*#set PRAM type (at 0x50000904), wrt param code*/
	/* #init_code:0x01,
	 * baseline:0x02
	 * COMP:0x04
	 * param:0x08
	 * FW:0x10
	 * bootloader:0x20
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = u8_type;
	i32_ret = raydium_i2c_pda_write(client, 0x50000904, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#set PRAM addr (at 'h5000_0908)*/
	/* #init_code:0x800
	 * Baseline:0xA00
	 * COMP:0xCD4
	 * para:0xF1C
	 * FW:0x1000
	 * BOOT:0x5000
	*/
	/*#set PRAM length (at 'h5000_090C)*/
	if (u8_type == RAYDIUM_INIT) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x6e;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x02;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_BASELINE) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0xcc;
		u8_buf[1] = 0x6c;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x30;
		u8_buf[1] = 0x01;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_COMP) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x60;
		u8_buf[1] = 0x6b;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x9c;
		u8_buf[1] = 0x02;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_PARA) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x6a;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x5c;
		u8_buf[1] = 0x01;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_FIRMWARE) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x08;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x5c;
		u8_buf[1] = 0x63;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_BOOTLOADER) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x08;
		i32_ret = raydium_i2c_pda_write(client, 0x50000908, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x0A;
		i32_ret = raydium_i2c_pda_write(client, 0x5000090C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	/*#set sync_data(0x20000200) = 0 as WRT data finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#Wait bootloader check addr and PRAM unlock*/
	/*#Confirm g_u8_sync_data.ret_data at 0x20000214 is SET_ADDR_READY*/
	if (wait_fw_state(client, 0x20000214, 161, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, SET_ADDR_READY\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#Confirm g_u8_sync_data.cmd_type at 0x20000210 is WRT_PRAM_DATA*/
	if (wait_fw_state(client, 0x20000210, 163, 1000, u16_retry) == ERROR) {
		pr_err("[touch]Error, WRT_PRAM_DATA\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#start write data to PRAM*/
	if (u8_type == RAYDIUM_INIT) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0x6E00,
						    RAYDIUM_INIT);
		if (i32_ret < 0)
			goto exit_upgrade;
	}  else if (u8_type == RAYDIUM_PARA) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0x6a00,
						    RAYDIUM_PARA);
		if (i32_ret < 0)
			goto exit_upgrade;
	} else if (u8_type == RAYDIUM_FIRMWARE) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0x800,
						    RAYDIUM_FIRMWARE);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_fw_upgrade_with_image(client, 0x6a00,
						    RAYDIUM_PARA);
		if (i32_ret < 0)
			goto exit_upgrade;


	} else if (u8_type == RAYDIUM_BOOTLOADER) {
		i32_ret = raydium_fw_upgrade_with_image(client, 0x0800,
						    RAYDIUM_BOOTLOADER);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_fw_upgrade_with_image(client, 0x1000,
						    RAYDIUM_INIT);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	/*
	 *set sync_data(0x20000200) = 0 as WRT data finish
	 *bootloader check checksum
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*
	 * wait(checksum okay) ACK cmd
	 * (gu8I2CSyncData.cmd_type=0xa5 at 0x20000210)
	 */
	if (wait_fw_state(client, 0x20000210, 165, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, WRT_CHECKSUM_OK\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#confirm ACK cmd result(gu8I2CSyncData.ret_data=0xa5 at 0x20000214)*/
	if (wait_fw_state(client, 0x20000214, 165, 1000, u16_retry) == ERROR) {
		pr_err("[touch]Error, confirm ACK cmd result\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*
	 * set ACK return data = 0x5A
	 * adb shell "echo 20000210 1 A5 > /sys/bus/i2c/drivers/raydium_ts/
	 * 1-0039 raydium_i2c_pda_access"
	 * above command can be ignored, due to previous while loop has check
	 * its value.
	 */
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa5;
	i32_ret = raydium_i2c_pda_write(client, 0x20000210, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x5a;
	i32_ret = raydium_i2c_pda_write(client, 0x20000214, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#clr sync_data(0x20000200) = 0 as finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#wait for input unlock key*/
	if (wait_fw_state(client, 0x20000210, 168, 1000, u16_retry) == ERROR) {
		pr_err("[touch]Error, wait for input unlock key\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#unlock key*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xd7;
	i32_ret = raydium_i2c_pda_write(client, 0x50000938, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa5;
	i32_ret = raydium_i2c_pda_write(client, 0x50000934, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x50000934, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa5;
	i32_ret = raydium_i2c_pda_write(client, 0x50000934, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x50000938, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x50000624, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#wrt return data as unlock value*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa8;
	i32_ret = raydium_i2c_pda_write(client, 0x20000214, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*pr_info("[touch]ready burn flash\n");*/

	/*#clr sync_data(0x20000200) = 0 as finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/* wait erase/wrt finish
	 * confirm burning_state result (gu8I2CSyncData.burning_state =
	 * BURNING_WRT_FLASH_FINISH at 0x2000020C)
	 */
	if (wait_fw_state(client, 0x2000020c, 6, 2000, u16_retry) == ERROR) {
		dev_err(&ts->client->dev,
			"[touch]Error, wait erase/wrt finish\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}
	pr_info("[touch]Burn flash ok\n");


	if (u8_type == RAYDIUM_BOOTLOADER) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x10;
		u8_buf[1] = 0x08;
		i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#wait software reset finish*/
	msleep(25);

	/* wait sw reset finished 0x20000214 = 0x82 */
	if (wait_fw_state(client, 0x20000214, 130, 2000, u16_retry) == ERROR) {
		pr_err("[touch]Error, wait sw reset finished\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	if (u8_type == RAYDIUM_BASELINE || u8_type == RAYDIUM_COMP ||
	    u8_type == RAYDIUM_FIRMWARE || u8_check_crc == 1) {
		/*#set test_mode = 1*/
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x01;
		i32_ret = raydium_i2c_pda_write(client, 0x20000218, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		/*#wait crc check finish*/
		if (wait_fw_state(client, 0x20000208, 2, 2000, u16_retry)
				== ERROR) {
			pr_err("[touch]Error, wait crc check finish\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}

		/*#crc check pass 0x20000214 = 0x81*/
		if (wait_fw_state(client, 0x20000214, 0x81, 2000, u16_retry)
				== ERROR) {
			pr_err("[touch]Error, confirm crc result\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}
	}

	/*#run to next step*/
	pr_info("[touch]Type 0x%x => Pass\n", u8_type);

	if (u8_check_crc) {
		/*#clr sync para*/
		memset(u8_buf, 0, sizeof(u8_buf));
		i32_ret = raydium_i2c_pda_write(client, 0x20000210, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_i2c_pda_write(client, 0x20000214, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_i2c_pda_write(client, 0x20000218, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;


		i32_ret = raydium_i2c_pda_write(client, 0x20000200, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		usleep_range(4500, 5500);
		raydium_i2c_pda_set_address(ts, 0x50000628, DISABLE);

		g_u8_i2c_mode = PDA2_MODE;

		pr_info("[touch]Burn FW finish!\n");
	}

exit_upgrade:
	return i32_ret;
}

int raydium_burn_fw(struct i2c_client *client)
{
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);
	int i32_ret = 0;

	if (ts->id == RAD_20 || ts->id == RAD_21) {
		pr_info("[touch]start burn function!\n");
		i32_ret = raydium_boot_upgrade_2X(client);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_fw_upgrade_2X(client, RAYDIUM_BOOTLOADER, 0);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_fw_upgrade_2X(client, RAYDIUM_FIRMWARE, 1);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (ts->id == RAD_10 || ts->id == RAD_11) {
		pr_info("[touch]start burn function!\n");
		i32_ret = raydium_boot_upgrade_1X(client);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_fw_upgrade_1X(client, RAYDIUM_BOOTLOADER, 0);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_fw_upgrade_1X(client, RAYDIUM_FIRMWARE, 1);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

exit_upgrade:
	return i32_ret;
}

int raydium_fw_update_check(struct raydium_ts_data *ts,
			unsigned short u16_i2c_data)
{

	unsigned char u8_rbuffer[4];

	unsigned int u32_fw_version, u32_image_version;
	int i32_ret = ERROR;
#ifdef FW_UPDATE_EN
	unsigned char u8_mode_change;
#endif
	mutex_lock(&ts->lock);
	i32_ret = raydium_i2c_pda2_set_page(ts->client,
				ts->is_suspend,
				RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0)
		goto exit_error;

	i32_ret = raydium_i2c_pda2_read(ts->client,
				    RAYDIUM_PDA2_FW_VERSION_ADDR,
				    u8_rbuffer,
				    4);
	if (i32_ret < 0)
		goto exit_error;

	mutex_unlock(&ts->lock);

	ts->id = ((u16_i2c_data & 0xF) << 12) |
		((u8_rbuffer[0] & 0xF) << 8) | u8_rbuffer[1];

	raydium_mem_table_init(ts->id);
	raydium_mem_table_setting(ts->id);

	u32_fw_version = (u8_rbuffer[0] << 24) | (u8_rbuffer[1] << 16) |
		(u8_rbuffer[3] << 8) | u8_rbuffer[2];
	pr_info("[touch]RAD FW ver 0x%.8x\n", u32_fw_version);

	ts->fw_version = u32_fw_version;

	u32_image_version = (g_rad_para_image[0x0004] << 24) |
			(g_rad_para_image[0x0005] << 16) |
			(g_rad_para_image[0x0007] << 8) |
			g_rad_para_image[0x0006];

	pr_info("[touch]RAD Image FW ver : 0x%x\n", u32_image_version);

#ifdef FW_UPDATE_EN
	if (u32_fw_version != u32_image_version) {
		pr_info("[touch]FW need update.\n");
		raydium_irq_control(ts, DISABLE);
		if ((g_u8_raydium_flag & ENG_MODE) == 0) {
			g_u8_raydium_flag |= ENG_MODE;
			u8_mode_change = 1;
		}
		i32_ret = raydium_burn_fw(ts->client);
		if (i32_ret < 0)
			pr_err("[touch]FW update fail:%d\n", i32_ret);

		if (u8_mode_change) {
			g_u8_raydium_flag &= ~ENG_MODE;
			u8_mode_change = 0;
		}
		raydium_irq_control(ts, ENABLE);
		mutex_lock(&ts->lock);
		i32_ret = raydium_i2c_pda2_set_page(ts->client,
					ts->is_suspend,
					RAYDIUM_PDA2_PAGE_0);
		if (i32_ret < 0)
			goto exit_error;

		i32_ret = raydium_i2c_pda2_read(ts->client,
					    RAYDIUM_PDA2_FW_VERSION_ADDR,
					    u8_rbuffer,
					    4);
		if (i32_ret < 0)
			goto exit_error;

		mutex_unlock(&ts->lock);
		u32_fw_version = (u8_rbuffer[0] << 24) |
			     (u8_rbuffer[1] << 16) |
			     (u8_rbuffer[3] << 8) |
			     u8_rbuffer[2];
		pr_info("[touch]RAD FW ver is 0x%x\n",
			 u32_fw_version);
		ts->fw_version = u32_fw_version;
	} else
		pr_info("[touch]FW is the latest version.\n");
#endif

	return i32_ret;

exit_error:
	mutex_unlock(&ts->lock);
	return i32_ret;
}

/*static int raydium_i2c_pda_loadfw(struct i2c_client *client,
				  unsigned int u32_addr,
				  const unsigned char *p_u8_w_data,
				  unsigned short u16_length)
{
	int i32_retval;
	unsigned char u8_retry;
	unsigned char u8_mode = 0x00;
	unsigned char u8_buf[MAX_WRITE_PACKET_SIZE + 1];
	struct raydium_ts_data *ts = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = u16_length + 1,
			.buf = u8_buf,
		},
	};

	if (u16_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;

	if (u16_length == 4)
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			RAD_I2C_PDA_2_MODE_DISABLE |
			RAD_I2C_PDA_MODE_WORD_MODE;
	else
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			RAD_I2C_PDA_2_MODE_DISABLE;

	u8_buf[0] = u32_addr & MASK_8BIT;
	memcpy(&u8_buf[1], p_u8_w_data, u16_length);

	i32_retval = raydium_i2c_pda_set_address(ts, u32_addr, u8_mode);
	if (i32_retval != RAYDIUM_I2C_PDA_ADDRESS_LENGTH)
		goto exit;
	usleep_range(50, 80);

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			i32_retval = u16_length;
			break;
		}
		pr_err("[touch]%s: I2C retry %d\n", __func__, u8_retry + 1);
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		pr_err("[touch]%s: I2C write over retry limit\n", __func__);
		i32_retval = -EIO;
	}
exit:
	return i32_retval;
}*/

int raydium_burn_comp(struct i2c_client *client)
{
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);
	int i32_ret = FAIL;

	i32_ret = set_skip_load(client);
	if (i32_ret < 0)
		goto exit_upgrade;


	if (ts->id == RAD_20 || ts->id == RAD_21) {
		i32_ret = raydium_fw_upgrade_2X(client, RAYDIUM_COMP, 1);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (ts->id == RAD_10 || ts->id == RAD_11) {
		i32_ret = raydium_fw_upgrade_1X(client, RAYDIUM_COMP, 1);
		if (i32_ret < 0)
			goto exit_upgrade;
	}
	i32_ret = SUCCESS;

exit_upgrade:
	return i32_ret;
}

int raydium_load_test_fw(struct i2c_client *client)
{
	int i32_ret = SUCCESS;
	unsigned char u8_buf[4];
	unsigned int u32_crc_result, u32_read_data;
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);

	/*set mcu hold*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x20;
	raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	raydium_i2c_pda_read(client, 0x40000004, u8_buf, 4);
	u8_buf[0] |= 0x01;
	raydium_i2c_pda_write(client, 0x40000004, u8_buf, 4);
	msleep(25);


	i32_ret = raydium_i2c_pda_read(client, 0x40000000, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	u8_buf[3] |= 0x40;
	i32_ret = raydium_i2c_pda_write(client, 0x40000000, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = raydium_i2c_pda_read(client, 0x40000014, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	u8_buf[0] |= 0x04;
	u8_buf[1] |= 0x04;
	i32_ret = raydium_i2c_pda_write(client, 0x40000014, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	pr_info("[touch]Raydium WRT test_fw to PRAM\n");

	i32_ret = raydium_i2c_pda_write(client, 0x50000900, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*Sending test fw*/
	if (ts->id == RAD_20 || ts->id == RAD_21) {
		i32_ret = raydium_fw_upgrade_with_image(client,
			0x800, RAYDIUM_TEST_FW);
		if (i32_ret < 0)
			goto exit_upgrade;
	} else if (ts->id == RAD_10 || ts->id == RAD_11) {
		i32_ret = raydium_fw_upgrade_with_image(client,
			0xA00, RAYDIUM_TEST_FW);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	/*check pram crc data*/
	if (ts->id == RAD_20 || ts->id == RAD_21) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[1] = 0x08;
		u8_buf[2] = 0x5B;
		u8_buf[3] = 0x6B;
		i32_ret = raydium_i2c_pda_write(client, 0x50000974, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_i2c_pda_read(client, 0x5000094C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		u8_buf[3] |= 0x81;
		i32_ret = raydium_i2c_pda_write(client, 0x5000094C, u8_buf, 4);
		usleep_range(9500, 10500);
		i32_ret = raydium_i2c_pda_read(client, 0x50000978, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		memcpy(&u32_crc_result, u8_buf, 4);
		i32_ret = raydium_i2c_pda_read(client, 0x6B5C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		memcpy(&u32_read_data, u8_buf, 4);
		if (u32_read_data != u32_crc_result) {
			pr_err("[touch]check pram fw crc fail!!\n");
			pr_err("[touch]u32_crc_result 0x%x\n", u32_crc_result);
			i32_ret = ERROR;
			goto exit_upgrade;
		}

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x60;
		u8_buf[1] = 0x6B;
		u8_buf[2] = 0xFB;
		u8_buf[3] = 0x6D;
		i32_ret = raydium_i2c_pda_write(client, 0x50000974, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_i2c_pda_read(client, 0x5000094C, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		u8_buf[3] |= 0x81;
		i32_ret = raydium_i2c_pda_write(client, 0x5000094C, u8_buf, 4);
		usleep_range(1000, 2000);
		i32_ret = raydium_i2c_pda_read(client, 0x50000978, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		memcpy(&u32_crc_result, u8_buf, 4);
		i32_ret = raydium_i2c_pda_read(client, 0x6DFC, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		memcpy(&u32_read_data, u8_buf, 4);
		if (u32_read_data != u32_crc_result) {
			pr_err("[touch]check pram CB crc fail!!\n");
			pr_err("[touch]u32_crc_result 0x%x\n", u32_crc_result);
			i32_ret = ERROR;
			goto exit_upgrade;
		}

		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[1] = 0x04;
		i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	/*Skip load*/
	pr_info("[touch]Raydium skip load\n");
	i32_ret = raydium_i2c_pda_read(client, 0x50000918, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	u8_buf[0] = 0x10;
	i32_ret = raydium_i2c_pda_write(client, 0x50000918, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = raydium_do_software_reset(client);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = raydium_check_fw_ready(client);

exit_upgrade:
	return i32_ret;
}
