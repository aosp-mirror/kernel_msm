/*
 * iaxxx-module.c -- IAxxx module interface for managers
 *
 * Copyright 2017 Knowles Corporation
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
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-sensor-header.h>
#include <linux/mfd/adnc/iaxxx-sensor-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include "iaxxx.h"


/*****************************************************************************
 * iaxxx_core_change_sensor_state()
 * @brief Change sensor state to enable/disable
 *
 * @inst_id	Sensor Instance Id
 * @is_enable	0 - Disable, 1 - Enable
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_sensor_change_state(struct device *dev, uint32_t inst_id,
			uint8_t is_enable, uint8_t block_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s()\n", __func__);
	/* protect this sensor operation */
	mutex_lock(&priv->module_lock);
	/* Set enable bit in sensor inst enable header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_SENSOR_HDR_SENSOR_ENABLE_ADDR,
		1 << inst_id,
		is_enable << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_change_sensor_state_err;
	}
	/* update on sensor enable and disable */
	priv->sensor_en[inst_id] = is_enable;

core_change_sensor_state_err:
	mutex_unlock(&priv->module_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_change_state);

/*****************************************************************************
 * iaxxx_core_sensor_set_param_by_inst()
 * @brief Set a param in a sensor instance
 *
 * @id		Sensor Instance Id
 * @param_id	Param Id
 * @param_val	Param value
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_sensor_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	inst_id &= IAXXX_SENSR_ID_MASK;
	/* protect this sensor operation */
	mutex_lock(&priv->module_lock);

	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_ID_REG(inst_id), param_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_REG(inst_id), param_val);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
			IAXXX_SENSOR_HDR_SET_PARAM_REQ_ADDR,
			1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "update bit failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

sensor_set_param_inst_err:
	mutex_unlock(&priv->module_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_set_param_by_inst);

/*****************************************************************************
 * iaxxx_core_sensor_get_param_by_inst()
 * @brief get a param in a sensor instance
 *
 * @id		Sensor Instance Id
 * @param_id	Param Id
 * @param_val	return param value
 * @block_id	Update block id
 *
 * @ret 0 in case of success, -EINVAL in case of error.
 ****************************************************************************/
int iaxxx_core_sensor_get_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t *param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	inst_id &= IAXXX_SENSR_ID_MASK;
	dev_dbg(dev, "%s()\n", __func__);
	/* protect this sensor operation */
	mutex_lock(&priv->module_lock);

	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_ID_REG(inst_id), param_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
			IAXXX_SENSOR_HDR_GET_PARAM_REQ_ADDR,
			1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

	ret = regmap_read(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_REG(inst_id), param_val);
	if (ret) {
		dev_err(dev, "read failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

sensor_get_param_inst_err:
	mutex_unlock(&priv->module_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_get_param_by_inst);
