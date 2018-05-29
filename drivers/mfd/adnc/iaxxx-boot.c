/*
 * iaxxx-boot.c -- IAxxx boot firmware to application mode
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include "iaxxx.h"
#include <linux/mfd/adnc/iaxxx-core.h>

#define IAXXX_CHUNK_SIZE 32768

/* Firmware and hardware configuration files */
static const char *iaxxx_fw_filename = "audience/iaxxx/RomeApp.bin";

void iaxxx_copy_le32_to_cpu(void *dst, const void *src, size_t nbytes)
{
	int i;
	int num_words = nbytes / sizeof(uint32_t);
	uint32_t *p_dst = dst;
	const uint32_t *p_src = src;

	if (WARN_ON(!src) || WARN_ON(!dst) || WARN_ON(nbytes & 0x3))
		return;

	for (i = 0; i < num_words; ++i, ++p_dst, ++p_src)
		*p_dst = le32_to_cpu(*p_src);
}

/**
 * iaxxx_download_section - downloads a firmware text or data section
 *
 * @priv    : iaxxx private data
 * @data    : the section data to be downloaded
 * @section : pointer to the section data (section address, length, etc).
 */
int iaxxx_download_section(struct iaxxx_priv *priv, const uint8_t *data,
				const struct firmware_section_header *section)
{
	int rc, i = 0;
	struct device *dev = priv->dev;
	uint32_t iaxxx_chunk_size = (priv->bus == IAXXX_I2C)
				? IAXXX_CHUNK_SIZE/64 : IAXXX_CHUNK_SIZE;
	int rem_bytes = section->length % (iaxxx_chunk_size);
	int temp_len = section->length / (iaxxx_chunk_size);
	int chunk_word_size = iaxxx_chunk_size * 4;


	dev_err(dev, "Writing section at 0x%.08X, %d words(s)\n",
				section->start_address, section->length);

	/* Write the section data directly to the device memory */
	for (i = 0; i < temp_len; i++) {

		rc = regmap_bulk_write(priv->regmap,
			(section->start_address + ((i * chunk_word_size))),
			data + (i * chunk_word_size), iaxxx_chunk_size);
		if (rc) {
			dev_err(dev, "%s: regmap_write failed, rc=%d\n",
						__func__, rc);
			return rc;
		}
	}
	if (rem_bytes) {
		rc = regmap_bulk_write(priv->regmap,
			(section->start_address + ((i * chunk_word_size))),
			data + (i * chunk_word_size), rem_bytes);
		if (rc) {
			dev_err(dev, "%s: regmap_write failed, rc=%d\n",
						__func__, rc);
			return rc;
		}
	}

	return 0;
}

int iaxxx_verify_fw_header(struct device *dev,
			struct firmware_file_header *header)
{
	const uint32_t SIG1 = 0x49445541;	/* "IDUA */
	const uint32_t SIG2 = 0x45434E45;	/* "ECNE */

	/* Verify the file signature */
	if (header->signature[0] != SIG1 || header->signature[1] != SIG2) {
		dev_err(dev, "Bad firmware file signature\n");
		return -EINVAL;
	}

	/* The file must contain at least 1 section */
	if (!header->number_of_sections) {
		dev_err(dev, "Missing sections in firmware file\n");
		return -EINVAL;
	}

	return 0;
}

static int
iaxxx_download_firmware(struct iaxxx_priv *priv, const struct firmware *fw)
{
	int i, rc = 0;
	int retries = 0;
	const int max_retries = 5;
	const uint8_t *data;
	struct device *dev = priv->dev;

	/* Checksum variable */
	unsigned int sum1 = 0xffff;
	unsigned int sum2 = 0xffff;

	size_t file_section_bytes;
	struct firmware_file_header header;
	struct firmware_section_header file_section = { 0x0, 0x0 };

	/* File header */
	if (sizeof(header) > fw->size) {
		dev_err(dev, "Bad Firmware image file (too small)\n");
		goto out;
	}
	iaxxx_copy_le32_to_cpu(&header, fw->data, sizeof(header));
	data = fw->data + sizeof(header);

	/* Verify the file header */
	rc = iaxxx_verify_fw_header(dev, &header);
	if (rc) {
		dev_err(dev, "Bad firmware image file\n");
		goto out;
	}

	/* Include file header fields as part of the checksum */
	CALC_FLETCHER16(header.number_of_sections, sum1, sum2);
	CALC_FLETCHER16(header.entry_point, sum1, sum2);

	/* Download each memory section */
	for (i = 0; i < header.number_of_sections; ++i) {
		/* Load the next data section */
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);

		if (file_section.length) {
			file_section_bytes = file_section.length * sizeof(u32);

			/* Include section header fields in the checksum */
			CALC_FLETCHER16(file_section.length, sum1, sum2);
			CALC_FLETCHER16(file_section.start_address, sum1, sum2);

			do {
				rc = iaxxx_download_section(priv, data,
								&file_section);
			} while (rc && ++retries < max_retries);

			if (rc) {
				dev_err(dev, "Failed to load firmware section\n");
				goto out;
			}

			/* Include checksum for this section */
			rc = iaxxx_checksum_request(priv,
					file_section.start_address,
					file_section.length, &sum1, &sum2);
			if (rc) {
				dev_err(dev, "Checksum request error\n");
				goto out;
			}

			/* Next section */
			data += file_section_bytes;
			WARN_ON((data - fw->data) > fw->size);
		}
	}

	/* If the last section length is 0, then verify the checksum */
	if (file_section.length == 0) {
		uint32_t checksum = (sum2 << 16) | sum1;

		dev_err(dev, "Expected checksum = 0x%.08X\n", checksum);

		if (checksum != file_section.start_address) {
			rc = -EINVAL;
			dev_err(dev, "%s(): Checksum mismatch 0x%.08X != 0x%.08X\n",
				__func__, checksum, file_section.start_address);
		}
	}

out:
	return rc;
}

/**
 * iaxxx_get_boot_complete_event - Checks for boot complete event in queue
 *
 * @priv    : iaxxx private data
 */
static int iaxxx_get_boot_complete_event(struct iaxxx_priv *priv)
{
	int rc;
	bool boot_completed = false;
	struct device *dev = priv->dev;

#if defined(CONFIG_MFD_IAXXX_FIRMWARE_EVENT_MANAGER)
	uint32_t count;
	struct iaxxx_event event;

	/* Read the count of available events */
	rc = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_COUNT_ADDR, &count);
	if (rc) {
		dev_err(dev, "Failed to read EVENT_COUNT, rc = %d\n", rc);
		goto out;
	}

	/* There should only be one event in the queue */
	WARN_ON(count != 1);

	while (count--) {
		rc = iaxxx_next_event_request(priv, &event);
		if (rc) {
			dev_err(dev, "Failed to read event, rc = %d\n", rc);
			goto out;
		}

		if (!boot_completed && event.event_src == 0 &&
		    event.event_id == SYS_EVENT_BOOT_COMPLETE)
			boot_completed = true;
		else
			dev_err(dev, "Unexpected event: src:0x%.04x, id:0x%.04X\n",
				event.event_src, event.event_id);
	}
#else
	uint8_t mode;
	uint32_t status;
	uint32_t reg;

	/* In absence of events, poll SYSTEM_STATUS for Application Mode */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &status);
	if (rc) {
		dev_err(dev, "Failed to read SYSTEM_STATUS, rc = %d\n", rc);
		goto out;
	}

	mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
	dev_dbg(dev, "System status 0x%.08x, mode = %d\n", status, mode);

	/* Get and log the Device ID */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_DEVICE_ID_ADDR, &reg);
	if (rc)
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);

	dev_dbg(dev, "Device ID after download: 0x%.08X\n", reg);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_ROM_VER_NUM_ADDR, &reg);
	if (rc)
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);

	dev_dbg(dev, "ROM Version after download: 0x%.08X\n", reg);

	if (mode == SYSTEM_STATUS_MODE_APPS)
		boot_completed = true;
	else
		return -EAGAIN;	/* Try again */
#endif

out:
	if (!boot_completed) {
		dev_err(dev, "Missing completion event on apps boot\n");
		return -ENXIO;
	}

	return rc;
}

/**
 * iaxxx_wait_apps_ready - Wait for firmware application mode to be ready
 *
 * @priv: iaxxx private data
 *
 * Returns 0 on success or -ETIMEDOUT if the device wasn't ready within the
 * expected time.
 */
static int iaxxx_wait_apps_ready(struct iaxxx_priv *priv)
{
	int i, rc = 0;
	unsigned int count;

	const unsigned int IAXXX_APPS_MODE_POLL_MSEC = 20;	/* 20 ms */
	const unsigned int IAXXX_APPS_MODE_WAIT_MSEC = 200;	/* 200 ms */

	count = IAXXX_APPS_MODE_WAIT_MSEC / IAXXX_APPS_MODE_POLL_MSEC;

	/* Apps mode is expected to be ready within 50 msecs */
	for (i = 0; i < count; ++i) {
		rc = iaxxx_get_boot_complete_event(priv);
		if (rc != -EAGAIN)
			break;
		msleep(IAXXX_APPS_MODE_POLL_MSEC);
	}

	return (rc == -EAGAIN) ? -ETIMEDOUT : rc;
}

/**
 * iaxxx_bootup - Boots the chip hardware
 *
 * @priv: iaxxx private data
 *
 * Downloads any necessary hardware configuration and RAM patches.
 */
int iaxxx_bootup(struct iaxxx_priv *priv)
{
	int rc;
	uint8_t mode;
	uint32_t status;
	uint32_t reg;
	const struct firmware *fw;
	struct device *dev = priv->dev;
	const struct firmware_file_header *fw_header;

	/* Request the firmware image */
	rc = request_firmware(&fw, iaxxx_fw_filename, dev);
	if (rc) {
		dev_err(dev, "Firmware file %s not found rc = %d\n",
						iaxxx_fw_filename, rc);
		return rc;
	}
	/* Download the firmware to device memory */
	rc = iaxxx_download_firmware(priv, fw);
	if (rc) {
		dev_err(dev, "Firmware download failed, rc = %d\n", rc);
		goto out;
	}

	fw_header = (const struct firmware_file_header *)fw->data;

	/* Get and log the Device ID */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_DEVICE_ID_ADDR, &reg);
	if (rc)
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);

	dev_dbg(dev, "Device ID before jump to ram: 0x%.08X\n", reg);

	/* Get and log the ROM version */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_ROM_VER_NUM_ADDR, &reg);
	if (rc)
		dev_err(dev, "regmap_read failed, rc = %d\n", rc);

	dev_dbg(dev, "ROM Version before jump to ram: 0x%.08X\n", reg);

	/* Boot device into application mode */
	rc = iaxxx_jump_to_request(priv, le32_to_cpu(fw_header->entry_point));
	if (rc) {
		dev_err(dev, "Failed to boot firmware, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the application mode to be up and running */
	rc = iaxxx_wait_apps_ready(priv);
	if (rc) {
		dev_err(dev, "Timed out waiting for apps mode, rc = %d\n", rc);
		goto out;
	}

	/* Read SYSTEM_STATUS to ensure that device is in Application Mode */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &status);
	if (rc) {
		dev_err(dev, "Failed to read SYSTEM_STATUS, rc = %d\n", rc);
		goto out;
	}

	mode = status & IAXXX_SRB_SYS_STATUS_MODE_MASK;
	if (mode != SYSTEM_STATUS_MODE_APPS) {
		dev_err(dev, "Not in application mode, mode = %d\n", mode);
		rc = -ENXIO;
		goto out;
	}
	/*
	 * device running in application mode
	 */

	dev_dbg(dev, "Firmware running in application mode\n");

out:
	release_firmware(fw);
	return rc;
}
