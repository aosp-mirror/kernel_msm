/* Copyright (c) 2017, LGE Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "sop716_firmware.h"

#define CRC16_POLY               0x1021
#define SOP716FW_BUFFER_SIZE     1024
#define SOP716_RESET_VECTOR_ADDR 0xFFFE

/* Defines for BSL Commands */
#define SOP716_I2C_SENDDATA     0x10
#define SOP716_I2C_BSLPASSWORD  0x11
#define SOP716_I2C_MASSERASE    0x15
#define SOP716_I2C_SETPC        0x17
#define SOP716_I2C_READDATA     0x18
#define SOP716_I2C_RESET        0x1C
#define SOP716_I2C_CMD_RESP     0x3A
#define SOP716_I2C_MESSAGE_RESP 0x3B

#define SOP716_I2C_HEADER       0x80

/* Defines for response codes */
#define SOP716_STATUS_OPERATION_OK  0x00


struct sop716fw_info {
	struct i2c_client *client;
	char *xfer_buffer;
};

struct sop716fw_playload {
	uint8_t cmd;
	uint8_t addr[3]; /* 0: lsb, 2: msb */
	uint8_t *data;
};

struct sop716fw_packet {
	uint8_t header;
	uint16_t length;
	struct sop716fw_playload payload;
	uint16_t checksum;
};

static uint16_t CRC16(uint8_t *data, uint32_t len)
{
	int i, j;
	uint16_t msg;
	uint16_t crc = 0xffff;
	uint8_t *p = data;

	for (i = 0; i < len; i++) {
		msg = p[i] << 8;
		for (j = 0; j < 8; j++) {
			if ((msg ^ crc) >> 15)
				crc = (crc << 1) ^ CRC16_POLY;
			else
				crc <<= 1;
			msg <<= 1;
		}
	}

	return crc;
}

static int sop716fw_i2c_write(struct sop716fw_info *sfi, uint8_t *buf, int len)
{
	struct i2c_client *client = sfi->client;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len,
			.buf = buf
		},
	};

	if ((i2c_transfer(client->adapter, msg, 1)) != 1)
		return -EIO;

	return 0;
}

static int sop716fw_send_packet(struct sop716fw_info *sfi,
		struct sop716fw_packet *packet, int len)
{
	struct i2c_client *client = sfi->client;
	uint16_t crc;
	uint16_t offset = 0;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD
		},
	};

	/* xfer buffer */
	sfi->xfer_buffer[0] = packet->header;
	sfi->xfer_buffer[1] = packet->length & 0xff;
	sfi->xfer_buffer[2] = (packet->length >> 8) & 0xff;
	sfi->xfer_buffer[3] = packet->payload.cmd;

	if (packet->payload.addr[2] != 0xff) {
		offset = sizeof(packet->payload.addr);
		memcpy(&sfi->xfer_buffer[4], packet->payload.addr, offset);
	}

	if ((packet->length - offset - 1) > 0)
		memcpy(sfi->xfer_buffer + 4 + offset,
				packet->payload.data,
				packet->length - offset - 1);

	/* CRC */
	crc = CRC16(sfi->xfer_buffer + 3, packet->length);
	sfi->xfer_buffer[packet->length + 3] = crc & 0xff;
	sfi->xfer_buffer[packet->length + 4] = (crc >> 8) & 0xff;


	/* i2c msg */
	msg[0].len = packet->length + 5;
	msg[0].buf = sfi->xfer_buffer;

	msg[1].len = len + 7;
	msg[1].buf = sfi->xfer_buffer;

	if ((i2c_transfer(client->adapter, msg, 2)) != 2)
		return -EIO;

	return 0;
}

static int __check_resp_header(struct sop716fw_info *sfi, uint16_t size)
{
	uint16_t len, crc;

	/* check header */
	if (sfi->xfer_buffer[0] || (sfi->xfer_buffer[1] != SOP716_I2C_HEADER)) {
		pr_err("%s: invalid header\n", __func__);
		return -EINVAL;
	}

	/* check length */
	len = sfi->xfer_buffer[2] | (sfi->xfer_buffer[3] << 8);
	if (len != (size + 1)) {
		pr_err("%s: invalid length %u\n", __func__, len);
		return -EINVAL;
	}

	/* CRC */
	crc = sfi->xfer_buffer[len + 4] | (sfi->xfer_buffer[len + 5] << 8);
	if (crc != CRC16(sfi->xfer_buffer + 4, len)) {
		pr_err("%s: CRC error\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int sop716fw_check_msg_resp(struct sop716fw_info *sfi)
{
	if (__check_resp_header(sfi, 1))
		return -EINVAL;

	/* check response */
	if (sfi->xfer_buffer[4] != SOP716_I2C_MESSAGE_RESP ||
	    sfi->xfer_buffer[5] != SOP716_STATUS_OPERATION_OK) {
		pr_debug("%s: wrong resp %x, error %x\n", __func__,
				sfi->xfer_buffer[4],
				sfi->xfer_buffer[5]);
		return -EINVAL;
	}

	return 0;
}

static int sop716fw_check_cmd_resp(struct sop716fw_info *sfi, uint16_t size)
{
	if (__check_resp_header(sfi, size))
		return -EINVAL;

	/* check response */
	if (sfi->xfer_buffer[4] != SOP716_I2C_CMD_RESP) {
		pr_debug("%s: wrong resp %x\n", __func__, sfi->xfer_buffer[4]);
		return -EINVAL;
	}

	return 0;
}

static int sop716fw_write_data(struct sop716fw_info *sfi,
		uint8_t *data, uint32_t addr, uint32_t size)
{
	struct sop716fw_packet packet;

	/* allow for max 20bits address */
	packet.payload.addr[0] = addr & 0xff;
	packet.payload.addr[1] = (addr >> 8) & 0xff;
	packet.payload.addr[2] = (addr >> 16) & 0x0f;

	packet.header = SOP716_I2C_HEADER;
	packet.length = size + 4;
	packet.payload.cmd = SOP716_I2C_SENDDATA;
	packet.payload.data = data;

	if (sop716fw_send_packet(sfi, &packet, 1)) {
		pr_err("%s: cannot send packet\n", __func__);
		return -EIO;
	}

	return sop716fw_check_msg_resp(sfi);
}

static int sop716fw_read_data(struct sop716fw_info *sfi,
		uint8_t *data, uint32_t addr, uint32_t size)
{
	struct sop716fw_packet packet;
	uint8_t buffer[2];
	int err;

	packet.header = SOP716_I2C_HEADER;
	packet.payload.cmd = SOP716_I2C_READDATA;
	packet.payload.data = buffer;
	packet.payload.addr[0] = addr & 0xff;
	packet.payload.addr[1] = (addr >> 8) & 0xff;
	packet.payload.addr[2] = (addr >> 16) & 0x0f;
	packet.payload.data[0] = size & 0xff;
	packet.payload.data[1] = (size >> 8) & 0xff;
	packet.length = 6;

	if (sop716fw_send_packet(sfi, &packet, size)) {
		pr_err("%s: cannot send packet\n", __func__);
		return -EIO;
	}

	err = sop716fw_check_cmd_resp(sfi, size);
	if (err)
		return err;

	memcpy(data, sfi->xfer_buffer + 5, size);
	return 0;
}

static int sop716fw_invoke_bsl(struct sop716fw_info *sfi)
{
	uint8_t data[] = {0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE};
	int err;

	err = sop716fw_i2c_write(sfi, data, sizeof(data));
	if (err)
		return err;

	return 0;
}

static int sop716fw_erase(struct sop716fw_info *sfi)
{
	uint16_t crc;
	int err;

	sfi->xfer_buffer[0] = SOP716_I2C_HEADER;
	sfi->xfer_buffer[1] = 1;
	sfi->xfer_buffer[2] = 0;
	sfi->xfer_buffer[3] = SOP716_I2C_MASSERASE;

	crc = CRC16(sfi->xfer_buffer + 3, 1);
	sfi->xfer_buffer[4] = crc & 0xff;
	sfi->xfer_buffer[5] = (crc >> 8) & 0xff;

	err = sop716fw_i2c_write(sfi, sfi->xfer_buffer, 6);
	if (err)
		return err;

	return 0;
}

static int sop716fw_set_programcounter(struct sop716fw_info *sfi,
		uint16_t addr)
{
	uint16_t crc;
	int err;

	sfi->xfer_buffer[0] = SOP716_I2C_HEADER;
	sfi->xfer_buffer[1] = 4;
	sfi->xfer_buffer[2] = 0;
	sfi->xfer_buffer[3] = SOP716_I2C_SETPC;

	sfi->xfer_buffer[4] = addr & 0xff;
	sfi->xfer_buffer[5] = (addr >> 8) & 0xff;
	sfi->xfer_buffer[6] = 0;

	crc = CRC16(sfi->xfer_buffer + 3, 4);
	sfi->xfer_buffer[7] = crc & 0xff;
	sfi->xfer_buffer[8] = (crc >> 8) & 0xff;

	err = sop716fw_i2c_write(sfi, sfi->xfer_buffer, 9);
	if (err)
		return err;

	return 0;
}

static int sop716fw_reset(struct sop716fw_info *sfi)
{
	uint8_t reset_vector[2];
	uint16_t value;
	int err;

	err = sop716fw_read_data(sfi,
			reset_vector, SOP716_RESET_VECTOR_ADDR, 2);
	if (err) {
		pr_err("%s: cannot read reset vector address\n", __func__);
		return err;
	}

	value = (reset_vector[1] << 8) | reset_vector[0];
	pr_debug("sop716: reset vector addr 0x%x\n", value);
	err = sop716fw_set_programcounter(sfi, value);
	if (err)
		return err;

	return 0;
}

static int sop716fw_unlock(struct sop716fw_info *sfi)
{
	uint8_t password[32];
	struct sop716fw_packet packet;
	int err;

	memset(password, 0xff, sizeof(password));
	packet.payload.addr[0] = 0;
	packet.payload.addr[1] = 0;
	packet.payload.addr[2] = 0xff;

	packet.header = SOP716_I2C_HEADER;
	packet.length = 33;
	packet.payload.cmd = SOP716_I2C_BSLPASSWORD;
	packet.payload.data = password;

	err = sop716fw_send_packet(sfi, &packet, 1);
	if (err) {
		pr_err("%s: cannot unlock\n", __func__);
		return err;
	}

	return sop716fw_check_msg_resp(sfi);
}

static int sop716fw_program_section(struct sop716fw_info *sfi,
	   uint8_t *data, uint32_t addr, uint32_t size, bool no_check)
{
	const uint32_t chunk_size = 128;
	uint32_t i = 0;
	int err;

	pr_debug("sop716: programming section @ 0x%x\n", addr);
	while (i < size) {
		if ((size - i) > chunk_size) {
			err = sop716fw_write_data(sfi,
					data + i, addr + i, chunk_size);
			i += chunk_size;
		} else {
			err = sop716fw_write_data(sfi,
					data + i, addr + i, size - i);
			i = size;
		}

		if (!no_check && err) {
			pr_err("%s: cannot program section @ 0x%x\n",
					__func__, addr + i);
			break;
		}
	}
	return 0;
}

void sop716fw_validate_firmware(const uint8_t *fw,
		uint8_t *major, uint8_t *minor)
{
	struct sop716fw_header *fhdr;
	struct sop716fw_section *fsec;
	uint8_t *fw_data;
	uint32_t size = 0;
	int section_num;
	int i;

	*major = 0;
	*minor = 0;

	fhdr = (struct sop716fw_header *)fw;
	section_num = fhdr->section_num;
	fsec = (struct sop716fw_section *)(fw + fhdr->section_offset);
	fw_data = (uint8_t *)(fw + fhdr->firmware_offset);

	/* check header */
	if (memcmp(fhdr->magic, SOP716FW_MAGIC, SOP716FW_MAGIC_SIZE)) {
		pr_err("%s: wrong firmware file\n", __func__);
		return;
	}

	/* check firmware size */
	for (i = 0; i < section_num; i++)
		size +=fsec[i].length;
	if (size != fhdr->firmware_size) {
		pr_err("%s: firmware size not matched (%u %u)\n", __func__,
				fhdr->firmware_size, size);
		return;
	}

	/* read version */
	*minor = fw_data[4];
	*major = fw_data[5];
}

int sop716fw_update_firmware(struct i2c_client *client, const uint8_t *fw)
{
	struct sop716fw_header *fhdr;
	struct sop716fw_section *fsec;
	struct sop716fw_info sfi;
	uint8_t *fw_data;
	int section_num;
	int retry = 5;
	int err;

	memset(&sfi, 0, sizeof(struct sop716fw_info));
	sfi.client = client;

	pr_info("sop716: updating firmware\n");

	/* alloc buffers */
	sfi.xfer_buffer = kzalloc(SOP716FW_BUFFER_SIZE, GFP_KERNEL);
	if (!sfi.xfer_buffer) {
		pr_err("%s: no mem for xfer_buffer\n", __func__);
		return -ENOMEM;
	}

	fhdr = (struct sop716fw_header *)fw;
	section_num = fhdr->section_num;
	fsec = (struct sop716fw_section *)(fw + fhdr->section_offset);
	fw_data = (uint8_t *)(fw + fhdr->firmware_offset);

	/* invoke the BSL */
	pr_info("sop716: Invoking BSL\n");
	err = sop716fw_invoke_bsl(&sfi);
	if (err) {
		pr_err("%s: cannot invoke BSL\n", __func__);
		goto out;
	}

	msleep(100);

	/* erase */
	pr_info("sop716: erasing\n");
	err = sop716fw_erase(&sfi);
	if (err) {
		pr_err("%s: cannot erase\n", __func__);
		goto out;
	}

	msleep(100);

	/* unlock */
	pr_info("sop716: unlock device\n");
	err = sop716fw_unlock(&sfi);
	if (err) {
		pr_err("%s: cannot unlock\n", __func__);
		goto out;
	}

	/* programming */
	pr_info("sop716: programming firmware\n");
	while (retry--) {
		int i;
		int offset = 0;

		for (i = 0; i < section_num; i++) {
			uint8_t *sec_data = fw_data + offset;
			uint32_t sec_length = fsec[i].length;
			uint32_t sec_addr = fsec[i].start;

			/*
			 * section 0, 1 and 2 are firmware version info
			 * don't need to check error code
			 */
			err = sop716fw_program_section(&sfi,
					sec_data, sec_addr, sec_length,
					i < 3? true : false);
			if (err)
				break;

			offset += sec_length;
		}
		if (!err)
			break;
	}

	if (err) {
		pr_err("%s: cannot programming firmware\n", __func__);
		goto out;
	}

	/* reset */
	err = sop716fw_reset(&sfi);
	if (err) {
		pr_err("%s: cannot reset device\n", __func__);
		goto out;
	}

	pr_info("sop716: firmware updated successfully!\n");

	err = 0;
out:
	kfree(sfi.xfer_buffer);

	return err;
}
