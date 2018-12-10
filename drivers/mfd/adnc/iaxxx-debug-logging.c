/*
 * iaxxx-debug-logging.c -- IAxxx debug interface for logging
 *
 * Copyright 2018 Knowles Corporation
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
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-debug-intf.h>
#include "iaxxx.h"
#include <linux/mfd/adnc/iaxxx-register-defs-debuglog.h>

#define IAXXX_DEBUGLOG_DEBUG_LEVEL_ADDR(I) \
	(IAXXX_DEBUGLOG_DEBUG_LEVEL_0_ADDR + (4 * (I)))

#define IAXXX_DBG_LVL_NUM_BITS	0x4
#define IAXXX_NO_OF_MODULES_IN_EACH_REGISTER \
		(32 / IAXXX_DBG_LVL_NUM_BITS)

/*****************************************************************************
 * iaxxx_set_debug_log_level()
 * @brief Set debug log level for module id
 *
 * @module_id set log level to Module id
 * @log_level set log level
 *
 * @ret 0 on success, ret in case of error
 ****************************************************************************/
int iaxxx_set_debug_log_level(struct device *dev,
			uint32_t module_id, uint32_t log_level)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t mod_id;
	uint32_t mod_id_pos;
	uint32_t mod_mask;
	uint32_t mod_log_val;
	int ret = 0;
	uint32_t status;

	mod_id = module_id / IAXXX_NO_OF_MODULES_IN_EACH_REGISTER;
	mod_id_pos = module_id % IAXXX_NO_OF_MODULES_IN_EACH_REGISTER;
	mod_mask =
		(IAXXX_DEBUGLOG_DEBUG_LEVEL_0_MOD_0_DEBUG_LEVEL_MASK <<
		(mod_id_pos * IAXXX_DBG_LVL_NUM_BITS));
	mod_log_val = (log_level << (mod_id_pos * IAXXX_DBG_LVL_NUM_BITS));

	ret = regmap_update_bits(priv->regmap,
			IAXXX_DEBUGLOG_DEBUG_LEVEL_ADDR(mod_id),
			mod_mask, mod_log_val);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto out;
	}

	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "%s() Update blk failed %d\n", __func__, ret);
out:
	return ret;
}
EXPORT_SYMBOL(iaxxx_set_debug_log_level);

/*****************************************************************************
 * iaxxx_get_debug_log_level()
 * @brief Get debug log level for module id
 *
 * @module_id get log level to Module id
 * @log_level log level read for module id
 *
 * @ret 0 on success, ret in case of error
 ****************************************************************************/
int iaxxx_get_debug_log_level(struct device *dev,
			uint32_t module_id, uint32_t *log_level)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t mod_id;
	uint32_t mod_id_pos;
	uint32_t mod_log_val;
	int ret = 0;

	mod_id = module_id / IAXXX_NO_OF_MODULES_IN_EACH_REGISTER;

	ret = regmap_read(priv->regmap,
			IAXXX_DEBUGLOG_DEBUG_LEVEL_ADDR(mod_id),
			&mod_log_val);
	if (ret) {
		dev_err(dev, "%s() failed %d\n", __func__, ret);
		return ret;
	}

	mod_id_pos = module_id % IAXXX_NO_OF_MODULES_IN_EACH_REGISTER;
	*log_level = ((mod_log_val >>
		(mod_id_pos * IAXXX_DBG_LVL_NUM_BITS)) &
		IAXXX_DEBUGLOG_DEBUG_LEVEL_0_MOD_0_DEBUG_LEVEL_MASK);

	return ret;
}
EXPORT_SYMBOL(iaxxx_get_debug_log_level);

/*****************************************************************************
 * iaxxx_set_debug_log_mode()
 * @brief set debug log mode for each processor
 *
 * @mode debug from memory or endpoint
 * @proc_id processor id
 *
 * @ret 0 on success, ret in case of error
 ****************************************************************************/
int iaxxx_set_debug_log_mode(struct device *dev,
				bool mode, uint8_t proc_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret = 0;
	uint32_t status;

	ret = regmap_update_bits(priv->regmap,
				IAXXX_DEBUGLOG_DEBUG_LOG_MODE_ADDR,
				1 << proc_id, mode << proc_id);
	if (ret) {
		dev_err(dev, "write set dbg log mode failed %s()\n", __func__);
		goto out;
	}

	ret = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (ret)
		dev_err(dev, "%s() Update blk failed %d\n", __func__, ret);

out:
	return ret;
}
EXPORT_SYMBOL(iaxxx_set_debug_log_mode);

/*****************************************************************************
 * iaxxx_get_debug_log_mode()
 * @brief get debug log mode for each processor
 *
 * @mode return mode read from chip
 * @proc_id processor id
 *
 * @ret 0 on success, ret in case of error
 ****************************************************************************/
int iaxxx_get_debug_log_mode(struct device *dev,
				bool *mode, uint8_t proc_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t read_mode = 0;
	int ret = 0;

	ret = regmap_read(priv->regmap,
			IAXXX_DEBUGLOG_DEBUG_LOG_MODE_ADDR, &read_mode);
	if (ret) {
		dev_err(dev, "%s() failed %d\n", __func__, ret);
		return ret;
	}

	*mode = (read_mode & (1 << proc_id));
	return ret;
}
EXPORT_SYMBOL(iaxxx_get_debug_log_mode);
