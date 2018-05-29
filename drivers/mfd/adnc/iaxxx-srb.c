/*
 * iaxxx-srb.c -- IAxxx System Register Block Requests
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


#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-plugin-instance-header.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include "iaxxx.h"

#define BIT_FNAME(N, T)  N ## _ ## T
#define GET_BITS(R, N)	(((R) & BIT_FNAME(N, MASK)) >> BIT_FNAME(N, POS))

#define SET_BITS(R, N, V) ((R) = (((R) & ~(BIT_FNAME(N, MASK))) | \
	((V) << (BIT_FNAME(N, POS)) & (BIT_FNAME(N, MASK)))))

#define GET_SYS_STATUS_BLK_UPDATE_CODE(S) \
	GET_BITS(S, IAXXX_SRB_SYS_BLK_UPDATE_ERR_CODE)

#define GET_SYS_STATUS_BLK_UPDATE_RESULT(S) \
	GET_BITS(S, IAXXX_SRB_SYS_BLK_UPDATE_RES)

#define IAXXX_MAX_CORES	3

#define IAXXX_READ_DELAY        10	/* 10 us delay before SPI read */
#define IAXXX_READ_DELAY_RANGE  10	/* 10 us range */

/**
 * iaxxx_regmap_wait_clear - waits for the masked bits of a register to clear
 *
 * @priv: iaxxx private data
 * @reg : the register to be monitored
 * @mask: the mask of the bits to be tested
 *
 * This functions polls a register until all of the mask bits are cleared.
 * Returns 0 on success or -ETIMEDOUT if the bits didn't clear.
 */
static int
iaxxx_regmap_wait_clear(struct iaxxx_priv *priv, uint32_t reg, uint32_t mask)
{
	uint32_t req;
	int rc, retries;
	const int max_retries = 10;
	struct device *dev = priv->dev;

	for (retries = 0; retries < max_retries; ++retries) {
		if (priv->bus == IAXXX_I2C)
			msleep(200);
		else
			/* Give some time to device before read */
			usleep_range(IAXXX_READ_DELAY,
				IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

		rc = regmap_read(priv->regmap, reg, &req);
		if (rc) {
			/* We should not return here until we are done
			 * with our retries
			 */
			dev_err(dev, "regmap_read failed, rc = %d\n", rc);
		}

		/* Has the update bit been cleared */
		if (!(req & mask))
			return 0;

		if (priv->bus == IAXXX_SPI) {
			if (priv->is_application_mode)
				usleep_range(10000, 10005);
			else
				msleep(20);
		}
	}

	return -ETIMEDOUT;
}

static inline int
iaxxx_regmap_clear_bits(struct iaxxx_priv *priv, uint32_t reg, uint32_t mask)
{
	return regmap_update_bits(priv->regmap, reg, mask, 0);
}

static int iaxxx_read_error_register(struct iaxxx_priv *priv)
{
	int rc;
	int i;

	rc = regmap_read(priv->regmap, IAXXX_PLUGIN_HDR_ERROR_BLOCK_0_ADDR,
			&priv->iaxxx_state->err.error[IAXXX_BLOCK_0]);
	if (rc) {
		dev_err(priv->dev,
				"Read Error Block 0 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_0_ADDR,
			&priv->iaxxx_state->err.plg_inst_id[IAXXX_BLOCK_0]);
	if (rc) {
		dev_err(priv->dev,
				"Read Inst id Block 0 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(priv->regmap, IAXXX_PLUGIN_HDR_ERROR_BLOCK_1_ADDR,
			&priv->iaxxx_state->err.error[IAXXX_BLOCK_1]);
	if (rc) {
		dev_err(priv->dev,
				"Read Error Block 1 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_1_ADDR,
			&priv->iaxxx_state->err.plg_inst_id[IAXXX_BLOCK_1]);
	if (rc) {
		dev_err(priv->dev,
				"Read Inst id Block 1 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(priv->regmap, IAXXX_PLUGIN_HDR_ERROR_BLOCK_2_ADDR,
			&priv->iaxxx_state->err.error[IAXXX_BLOCK_2]);
	if (rc) {
		dev_err(priv->dev,
				"Read Error Block 2 addr fail %d\n", rc);
		goto out;
	}
	rc = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_2_ADDR,
			&priv->iaxxx_state->err.plg_inst_id[IAXXX_BLOCK_2]);
	if (rc) {
		dev_err(priv->dev,
				"Read Inst id Block 2 addr failed %d\n", rc);
		goto out;
	}
	for (i = 0 ; i < IAXXX_MAX_CORES; i++)
		dev_info(priv->dev, "Block id %d : Err 0x%x Inst id 0x%x\n", i,
		priv->iaxxx_state->err.error[i],
		priv->iaxxx_state->err.plg_inst_id[i] == PLUGIN_INST_NONE ? 0
			: priv->iaxxx_state->err.plg_inst_id[i]);
out:
	return rc;
}

/**
 * iaxxx_update_block_request - sends Update Block Request & waits for response
 *
 * @priv   : iaxxx private data
 * @status : pointer for the returned system status
 *
 * Core lock must be held by the caller.
 *
 * All requests are expected to finish within a few milliseconds and therefore
 * polling is used rather than using event interrupts to signal completion.
 */

static int iaxxx_update_block_request(struct iaxxx_priv *priv, uint32_t *status)
{
	int rc;
	int ret;
	struct device *dev = priv->dev;

	if (!status)
		return -EINVAL;

	dev_dbg(dev, "%s()\n", __func__);

	/* To protect concurrent update blocks requests*/
	mutex_lock(&priv->update_block_lock);

	/* Set the UPDATE_BLOCK_REQUEST bit in the SRB_UPDATE_CTRL register */
	rc = regmap_write(priv->regmap, IAXXX_SRB_SYS_BLK_UPDATE_ADDR,
			       IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "Failed to set UPDATE_BLOCK_REQUEST bit\n");
		goto out;
	}

	/*
	 * Note that in application mode, events can be used instead of polling.
	 * Make a wait for event like task that either polls or waits for an
	 * incoming event and wakes us up when the block update has completed.
	 */

	/* Poll waiting for the UPDATE_BLOCK_REQUEST bit to clear */
	rc = iaxxx_regmap_wait_clear(priv, IAXXX_SRB_SYS_BLK_UPDATE_ADDR,
				     IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "BLOCK_UPDATE failed, rc = %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* Read SYSTEM_STATUS to check for any errors */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_BLK_UPDATE_ADDR, status);
	if (rc) {
		dev_err(dev, "Failed to read SYSTEM_STATUS, rc = %d\n", rc);
		goto out;
	}

	/*
	 * If the BLOCK_UPDATE_RESULT is non-zero, then some update failed. The
	 * caller(s) will need to determine how to handle the failure.
	 */
	if (*status) {
		dev_err(dev, "%s status 0x%x\n", __func__, *status);
		rc = -ENXIO;
	}

out:
	mutex_unlock(&priv->update_block_lock);
	if (rc) {
		ret = iaxxx_read_error_register(priv);
		if (ret)
			dev_err(dev, "Read error register failed %d\n", ret);
	}

	return rc;
}

/**
 * iaxxx_update_block1_request - sends Update Block Request & waits for response
 *
 * @priv   : iaxxx private data
 * @status : pointer for the returned system status
 *
 * Core lock must be held by the caller.
 *
 * All requests are expected to finish within a few milliseconds and therefore
 * polling is used rather than using event interrupts to signal completion.
 */

static int iaxxx_update_block1_request(struct iaxxx_priv *priv,
						uint32_t *status)
{
	int rc;
	int ret;
	struct device *dev = priv->dev;

	if (!status)
		return -EINVAL;

	dev_dbg(dev, "%s()\n", __func__);

	/* To protect concurrent update blocks requests*/
	mutex_lock(&priv->update_block_lock);

	/* Set the UPDATE_BLOCK_REQUEST bit in the SRB_UPDATE_CTRL register */
	rc = regmap_write(priv->regmap, IAXXX_SRB_SYS_BLK_UPDATE_1_ADDR,
			       IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "Failed to set UPDATE_BLOCK_REQUEST bit\n");
		goto out;
	}

	/*
	 * Note that in application mode, events can be used instead of polling.
	 * Make a wait for event like task that either polls or waits for an
	 * incoming event and wakes us up when the block update has completed.
	 */

	/* Poll waiting for the UPDATE_BLOCK_REQUEST bit to clear */
	rc = iaxxx_regmap_wait_clear(priv, IAXXX_SRB_SYS_BLK_UPDATE_1_ADDR,
				     IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "BLOCK_UPDATE failed, rc = %d\n", rc);
		goto out;
	}

	/* Read SYSTEM_STATUS to check for any errors */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_BLK_UPDATE_1_ADDR, status);
	if (rc) {
		dev_err(dev, "Failed to read SYSTEM_STATUS, rc = %d\n", rc);
		goto out;
	}

	/*
	 * If the BLOCK_UPDATE_RESULT is non-zero, then some update failed. The
	 * caller(s) will need to determine how to handle the failure.
	 */
	if (*status) {
		dev_err(dev, "%s status 0x%x\n", __func__, *status);
		rc = -ENXIO;
	}

out:
	mutex_unlock(&priv->update_block_lock);
	if (rc) {
		ret = iaxxx_read_error_register(priv);
		if (ret)
			dev_err(dev, "Read error register failed %d\n", ret);
	}
	return rc;
}
/**
 * iaxxx_update_block2_request - sends Update Block Request & waits for response
 *
 * @priv   : iaxxx private data
 * @status : pointer for the returned system status
 *
 * Core lock must be held by the caller.
 *
 * All requests are expected to finish within a few milliseconds and therefore
 * polling is used rather than using event interrupts to signal completion.
 */

static int iaxxx_update_block2_request(struct iaxxx_priv *priv,
						uint32_t *status)
{
	int rc;
	int ret;
	struct device *dev = priv->dev;

	if (!status)
		return -EINVAL;

	dev_dbg(dev, "%s()\n", __func__);

	/* To protect concurrent update blocks requests*/
	mutex_lock(&priv->update_block_lock);

	/* Set the UPDATE_BLOCK_REQUEST bit in the SRB_UPDATE_CTRL register */
	rc = regmap_write(priv->regmap, IAXXX_SRB_SYS_BLK_UPDATE_2_ADDR,
			       IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "Failed to set UPDATE_BLOCK_REQUEST bit\n");
		goto out;
	}

	/*
	 * Note that in application mode, events can be used instead of polling.
	 * Make a wait for event like task that either polls or waits for an
	 * incoming event and wakes us up when the block update has completed.
	 */

	/* Poll waiting for the UPDATE_BLOCK_REQUEST bit to clear */
	rc = iaxxx_regmap_wait_clear(priv, IAXXX_SRB_SYS_BLK_UPDATE_2_ADDR,
				     IAXXX_SRB_SYS_BLK_UPDATE_REQ_MASK);
	if (rc) {
		dev_err(dev, "BLOCK_UPDATE failed, rc = %d\n", rc);
		goto out;
	}

	/* Read SYSTEM_STATUS to check for any errors */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_BLK_UPDATE_2_ADDR, status);
	if (rc) {
		dev_err(dev, "Failed to read SYSTEM_STATUS, rc = %d\n", rc);
		goto out;
	}

	/*
	 * If the BLOCK_UPDATE_RESULT is non-zero, then some update failed. The
	 * caller(s) will need to determine how to handle the failure.
	 */
	if (*status) {
		dev_err(dev, "%s status 0x%x\n", __func__, *status);
		rc = -ENXIO;
	}

out:
	mutex_unlock(&priv->update_block_lock);
	if (rc) {
		ret = iaxxx_read_error_register(priv);
		if (ret)
			dev_err(dev, "Read error register failed %d\n", ret);
	}
	return rc;
}


/*===========================================================================
 * Bootloader
 *===========================================================================
 */

static int iaxxx_bootloader_request(struct iaxxx_priv *priv, u32 mask, u32 val)
{
	int rc;
	uint32_t status;
	struct device *dev = priv->dev;

	/* Set the request bit in BOOTLOADER_REQUEST register */
	rc = regmap_update_bits(priv->regmap,
				IAXXX_SRB_BOOT_REQ_ADDR, mask, val);
	if (rc) {
		dev_err(dev, "Failed to set BOOTLOADER_REQUEST, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_update_block_request(priv, &status);
	if (rc) {
		dev_err(dev, "BOOTLOADER_REQUEST status = 0x%.08X\n", status);
		dev_err(dev, "BOOTLOADER_REQUEST failed, rc = %d\n", rc);

		/* 0x01 - SRB block update failure */
		WARN_ON(GET_SYS_STATUS_BLK_UPDATE_RESULT(status) != 0x01);

		if (mask == IAXXX_SRB_BOOT_REQ_JUMP_REQ_MASK) {
			/* 0x01 - Invalid Jump Address */
			WARN_ON(GET_SYS_STATUS_BLK_UPDATE_CODE(status) != 0x01);
		}

		goto out;
	}

	/* Special case, give extra time on jump-to-RAM requests */
	if (mask & IAXXX_SRB_BOOT_REQ_JUMP_REQ_MASK)
		msleep(200);

	/* The bit should have been cleared by firmware */
	rc = regmap_read(priv->regmap, IAXXX_SRB_BOOT_REQ_ADDR, &status);
	if (rc) {
		dev_err(dev, "Failed to read BOOTLOADR_REQUEST, rc = %d\n", rc);
		goto out;
	}

	if (WARN_ON(status & mask)) {
		/* Clear the request bit */
		iaxxx_regmap_clear_bits(priv, IAXXX_SRB_BOOT_REQ_ADDR, mask);
	}

out:
	return rc;
}

int iaxxx_jump_to_request(struct iaxxx_priv *priv, uint32_t address)
{
	int rc;
	struct device *dev = priv->dev;

	/* Program BOOTLOADER_JUMP_ADDRESS with the start address */
	rc = regmap_write(priv->regmap, IAXXX_SRB_BOOT_JUMP_ADDR_ADDR, address);
	if (rc) {
		dev_err(dev, "Failed to set JUMP_ADDRESS, rc = %d\n", rc);
		goto out;
	}

	/* Send the request */
	rc = iaxxx_bootloader_request(priv,
				IAXXX_SRB_BOOT_REQ_JUMP_REQ_MASK,
				0x1 << IAXXX_SRB_BOOT_REQ_JUMP_REQ_POS);
	if (rc) {
		dev_err(dev, "JUMP_REQUEST failed, rc = %d\n", rc);
		goto out;
	}

out:
	return rc;
}

int iaxxx_calibrate_oscillator_request(struct iaxxx_priv *priv, uint32_t delay)
{
	return -ENOTSUPP;
}

/*===========================================================================
 * Checksum
 *===========================================================================
 */

int iaxxx_checksum_request(struct iaxxx_priv *priv, uint32_t address,
			uint32_t length, uint32_t *sum1, uint32_t *sum2)
{
	int rc;
	uint32_t status;
	uint32_t request = 0;
	struct device *dev = priv->dev;

	if (!sum1 || !sum2)
		return -EINVAL;

	/* Set the CHECKSUM_VALUE */
	rc = regmap_write(priv->regmap, IAXXX_SRB_CHKSUM_VAL1_ADDR, *sum1);
	if (rc) {
		dev_err(dev, "Unable to clear CHECKSUM_VALUE1, rc = %d\n", rc);
		goto out;
	}

	rc = regmap_write(priv->regmap, IAXXX_SRB_CHKSUM_VAL2_ADDR, *sum2);
	if (rc) {
		dev_err(dev, "Unable to clear CHECKSUM_VALUE2, rc = %d\n", rc);
		goto out;
	}

	/* Program the CHECKSUM_ADDRESS with first memory address */
	rc = regmap_write(priv->regmap,
			  IAXXX_SRB_CHKSUM_BUFFER_ADDR_ADDR, address);
	if (rc) {
		dev_err(dev, "Failed to set CHECKSUM_ADDRESS, rc = %d\n", rc);
		goto out;
	}

	/* Program the CHECKSUM_LENGTH with the buffer length (in bytes) */
	rc = regmap_write(priv->regmap,
			  IAXXX_SRB_CHKSUM_LENGTH_ADDR, length);
	if (rc) {
		dev_err(dev, "Failed to set CHECKSUM_LENGTH, rc = %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* Set the CHECKSUM_REQUEST bit in CHECKSUM_REQUEST */
	rc = regmap_update_bits(priv->regmap, IAXXX_SRB_CHKSUM_ADDR,
			       IAXXX_SRB_CHKSUM_REQ_MASK,
			       0x1 << IAXXX_SRB_CHKSUM_REQ_POS);
	if (rc) {
		dev_err(dev, "Failed to set CHECKSUM_REQUEST, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_update_block_request(priv, &status);
	if (rc) {
		dev_err(dev, "CHECKSUM_REQUEST failed, rc = %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* The CHECKSUM_REQUEST bit should have been cleared by firmware */
	rc = regmap_read(priv->regmap, IAXXX_SRB_CHKSUM_ADDR, &request);
	if (rc) {
		dev_err(dev, "Failed to read CHECKSUM_REQUEST, rc = %d\n", rc);
		goto out;
	}

	WARN_ON(request & IAXXX_SRB_CHKSUM_REQ_MASK);

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	/* Read the checksum from CHECKSUM_VALUE */
	rc = regmap_read(priv->regmap, IAXXX_SRB_CHKSUM_VAL1_ADDR, sum1);
	if (rc) {
		dev_err(dev, "Failed to read CHECKSUM_VALUE1, rc = %d\n", rc);
		goto out;
	}

	/* Give some time to device before read */
	usleep_range(IAXXX_READ_DELAY,
			IAXXX_READ_DELAY + IAXXX_READ_DELAY_RANGE);

	rc = regmap_read(priv->regmap, IAXXX_SRB_CHKSUM_VAL2_ADDR, sum2);
	if (rc) {
		dev_err(dev, "Failed to read CHECKSUM_VALUE2, rc = %d\n", rc);
		goto out;
	}

out:
	return rc;
}

/*===========================================================================
 * Event Subscription
 *===========================================================================
 */

static int iaxxx_set_event_identification(struct iaxxx_priv *priv,
				u16 event_id, u16 event_src, u16 event_dst)
{
	int rc;
	struct device *dev = priv->dev;

	/* Set the system IDs of the source in EVENT_SUB_SRC_DST */
	rc = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR,
			IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_MASK,
			event_src << IAXXX_EVT_MGMT_EVT_SUB_SRC_ID_POS);
	if (rc) {
		dev_err(dev, "Failed to set EVENT_SUB_SRC, rc = %d\n", rc);
		goto out;
	}

	/* Set the system IDs of the destination in EVENT_SUB_SRC_DST */
	rc = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_SUB_ADDR,
			IAXXX_EVT_MGMT_EVT_SUB_DST_ID_MASK,
			event_dst << IAXXX_EVT_MGMT_EVT_SUB_DST_ID_POS);
	if (rc) {
		dev_err(dev, "Failed to set EVENT_SUB_DST, rc = %d\n", rc);
		goto out;
	}

	/* Set the EVENT_ID field of the EVENT_SUB_ID register */
	rc = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ID_ADDR,
			IAXXX_EVT_MGMT_EVT_ID_REG_MASK,
			event_id << IAXXX_EVT_MGMT_EVT_ID_REG_POS);
	if (rc) {
		dev_err(dev, "Failed to EVENT_ID, rc = %d\n", rc);
		goto out;
	}

out:
	return rc;
}

/**
 * iaxxx_subscribe_request - Sends an event subscription request to the device
 *
 * @priv	: iaxxx private data
 * @event_id	:
 * @event_src	:
 * @event_dst	:
 * @action_id	:
 * @opaque_data	:
 */
int iaxxx_subscribe_request(struct iaxxx_priv *priv, u16 event_id,
				u16 event_src, u16 event_dst, u32 opaque_data)
{
	int rc;
	uint32_t status;
	struct device *dev = priv->dev;

	rc = iaxxx_set_event_identification(priv,
					    event_id, event_src, event_dst);
	if (rc) {
		dev_err(dev, "Failed to set event ident, rc = %d\n", rc);
		goto out;
	}

	/* Set the EVENT_DST_OPAQUE in the EVENT_SUB_DST_OPAQUE register */
	rc = regmap_write(priv->regmap,
			IAXXX_EVT_MGMT_EVT_SUB_DST_OPAQUE_ADDR, opaque_data);
	if (rc) {
		dev_err(dev, "Failed to set EVENT_DST_OPAQUE, rc = %d\n", rc);
		goto out;
	}

	/* Set the EVENT_SUB_REQUEST bit in EVENT_SUB_REQUEST */
	rc = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
			IAXXX_EVT_MGMT_EVT_SUB_REQ_MASK,
			0x1 << IAXXX_EVT_MGMT_EVT_SUB_REQ_POS);
	if (rc) {
		dev_err(dev, "Failed to EVENT_SUB_REQUEST, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_update_block_request(priv, &status);
	if (rc) {
		dev_err(dev, "EVENT_SUB_REQUEST failed, rc = %d\n", rc);

		/* Request is serialized so the error code should match */
		WARN_ON(GET_SYS_STATUS_BLK_UPDATE_CODE(status) != 0x08);
		WARN_ON(GET_SYS_STATUS_BLK_UPDATE_RESULT(status) != 0x01);

		/* Clear the request bit */
		iaxxx_regmap_clear_bits(priv, IAXXX_EVT_MGMT_EVT_ADDR,
				IAXXX_EVT_MGMT_EVT_SUB_REQ_MASK);

		goto out;
	}

	/* The EVENT_SUB_REQUEST bit should have been cleared by firmware */
	rc = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR, &status);
	if (rc) {
		dev_err(dev, "Failed to read EVENT_SUB_REQUEST, rc = %d\n", rc);
		goto out;
	}

	WARN_ON(status & IAXXX_EVT_MGMT_EVT_SUB_REQ_MASK);

out:
	return rc;
}

/**
 * iaxxx_unsubscribe_request - Sends an event unsubscribe request to the device
 *
 * @priv	: iaxxx private data
 * @event_id	:
 * @event_src	:
 * @event_dst	:
 * @action_id	:
 */
int iaxxx_unsubscribe_request(struct iaxxx_priv *priv,
				u16 event_id, u16 event_src, u16 event_dst)
{
	int rc;
	uint32_t status;
	struct device *dev = priv->dev;

	rc = iaxxx_set_event_identification(priv,
					    event_id, event_src, event_dst);
	if (rc) {
		dev_err(dev, "Failed to set event ident, rc = %d\n", rc);
		goto out;
	}

	/* Set the EVENT_UNSUB_REQUEST bit in EVENT_SUB_REQUEST */
	rc = regmap_update_bits(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR,
			IAXXX_EVT_MGMT_EVT_UNSUB_REQ_MASK,
			0x1 << IAXXX_EVT_MGMT_EVT_UNSUB_REQ_POS);
	if (rc) {
		dev_err(dev, "Failed to EVENT_SUB_REQUEST, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_update_block_request(priv, &status);
	if (rc) {
		dev_err(dev, "EVENT_UNSUB_REQUEST failed, rc = %d\n", rc);

		/* Request is serialized so the error code should match */
		WARN_ON(GET_SYS_STATUS_BLK_UPDATE_CODE(status) != 0x0A);
		WARN_ON(GET_SYS_STATUS_BLK_UPDATE_RESULT(status) != 0x01);

		/* Clear the request bit */
		iaxxx_regmap_clear_bits(priv, IAXXX_EVT_MGMT_EVT_ADDR,
				IAXXX_EVT_MGMT_EVT_UNSUB_REQ_MASK);

		goto out;
	}

	/* The EVENT_UNSUB_REQUEST bit should have been cleared by firmware */
	rc = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_ADDR, &status);
	if (rc) {
		dev_err(dev, "Failed to read EVENT_SUB_REQUEST, rc = %d\n", rc);
		goto out;
	}

	WARN_ON(status & IAXXX_EVT_MGMT_EVT_UNSUB_REQ_MASK);

out:
	return rc;
}

/*===========================================================================
 * Event Notification
 *===========================================================================
 */

int iaxxx_next_event_request(struct iaxxx_priv *priv, struct iaxxx_event *ev)
{
	int rc;
	uint32_t status, data[3];
	struct device *dev = priv->dev;

	/* Set the next event notification request */
	rc = regmap_update_bits(priv->regmap,
			IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
			IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK,
			0x1 << IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_POS);
	if (rc) {
		dev_err(dev, "Failed to set EVT_NEXT_REQ, rc = %d\n", rc);
		goto out;
	}

	/* Wait for the request to complete */
	rc = iaxxx_update_block_request(priv, &status);
	if (rc) {
		dev_err(dev, "EVT_NEXT_REQ failed, rc = %d\n", rc);

		/* Clear the request bit */
		iaxxx_regmap_clear_bits(priv,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
				IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK);

		goto out;
	}

	/* The EVT_NEXT_REQ bit should have been cleared by firmware */
	rc = regmap_read(priv->regmap, IAXXX_EVT_MGMT_EVT_NEXT_REQ_ADDR,
			&status);
	if (rc) {
		dev_err(dev, "Failed to read EVT_NEXT_REQ, rc = %d\n", rc);
		goto out;
	}

	WARN_ON(status & IAXXX_EVT_MGMT_EVT_NEXT_REQ_NOT_MASK);
	rc = regmap_bulk_read(priv->regmap,
			IAXXX_EVT_MGMT_EVT_SRC_INFO_ADDR, data,
			ARRAY_SIZE(data));
	if (rc) {
		dev_err(dev, "Failed to read EVENT_INFO, rc = %d\n", rc);
		goto out;
	}

	ev->event_src = GET_BITS(data[0],
			IAXXX_EVT_MGMT_EVT_SRC_INFO_SYS_ID);
	ev->event_id  = GET_BITS(data[0],
			IAXXX_EVT_MGMT_EVT_SRC_INFO_EVT_ID);
	ev->src_opaque = data[1];
	ev->dst_opaque = data[2];

out:
	return rc;
}

/*===========================================================================
 * Script
 *===========================================================================
 */

/*===========================================================================
 * Exported APIs
 *===========================================================================
 */

/**
 * iaxxx_send_update_block_request - sends Update Block Request to the device
 *
 * @dev    : MFD core device pointer
 * @status : pointer for the returned system status
 * @id     : Block ID/ Proc ID
 *
 * This call will be blocked until either the device responds to the request
 * or a timeout occurs while waiting for the response.
 */
int iaxxx_send_update_block_request(struct device *dev, uint32_t *status,
						int id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = 0;
	uint32_t err = 0;

	/* WARN_ON(!mutex_is_locked(&priv->srb_lock)); */

	if (atomic_read(&priv->power_state) != IAXXX_NORMAL)
		return -ENXIO;

	if (!priv)
		return -EINVAL;
	if (id == IAXXX_BLOCK_0)
		rc = iaxxx_update_block_request(priv, status);
	else if (id == IAXXX_BLOCK_1)
		rc = iaxxx_update_block1_request(priv, status);
	else if (id == IAXXX_BLOCK_2)
		rc = iaxxx_update_block2_request(priv, status);

	if (rc == -ETIMEDOUT) {
		dev_err(dev, "No update block response from FW and host timedout:%d\n",
					-ETIMEDOUT);
	} else if (rc == -ENXIO) {
		err = *status & IAXXX_SRB_SYS_BLK_UPDATE_ERR_CODE_MASK;
		if (err == SYSRC_FAIL)
			dev_err(dev, "FW SYSRC_FAIL:0x%x\n", *status);
		else if (err == SYSRC_ERR_MEM)
			dev_err(dev, "FW NOMEM err:0x%x\n", *status);
		else if (err) {
			dev_err(dev, "FW general err:0x%x\n", *status);
			return -EINVAL;
		}
	} else
		return rc;

	if (priv->iaxxx_state->fw_state == FW_APP_MODE) {
		priv->iaxxx_state->fw_state = FW_CRASH;
		schedule_work(&priv->crash_recover_work);
	}

	return rc;
}
EXPORT_SYMBOL(iaxxx_send_update_block_request);
