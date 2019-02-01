/*
 * iaxxx-sysfs.c -- IAxxx Sysfs attributes
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
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-odsp.h>
#include <linux/mfd/adnc/iaxxx-module.h>
#include "iaxxx.h"

#define IAXXX_PWR_DWN_VAL 0x01C00050
#define IAXXX_PWR_ON_VAL 0x845
#define IAXXX_PWR_STATE_RETRY	0x5

#define IAXXX_PM_AUTOSUSPEND_DELAY 3000

#define IAXXX_PWR_MAX_SPI_SPEED   25000000

void iaxxx_pm_enable(struct iaxxx_priv *priv)
{
#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	int ret = 0;

	priv->in_suspend = 0;
	priv->in_resume = 0;
	ret = pm_runtime_set_active(priv->dev);
	if (ret < 0)
		pr_err("pm_runtime_set_active fail %d\n", ret);
	pm_runtime_enable(priv->dev);

	pm_runtime_set_autosuspend_delay(priv->dev, IAXXX_PM_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(priv->dev);
	pm_runtime_mark_last_busy(priv->dev);
#endif
}

int iaxxx_pm_get_sync(struct device *dev)
{
	int ret = 0;

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (priv == NULL) {
		dev_err(dev, "%s dev is NULL here\n", __func__);
		return -EINVAL;
	}

	if (!pm_runtime_enabled(dev)) {
		dev_dbg(dev, "%s Run time PM not enabled\n", __func__);
		return 0;
	}

	if (mutex_trylock(&priv->pm_mutex))
		ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		dev_dbg(dev, "%s() Fail. %d\n", __func__, ret);

	mutex_unlock(&priv->pm_mutex);
#endif
	return ret;
}

int iaxxx_pm_put_autosuspend(struct device *dev)
{
	int ret = 0;

#ifndef CONFIG_MFD_IAXXX_DISABLE_RUNTIME_PM
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (priv == NULL) {
		dev_err(dev, "%s dev is NULL here\n", __func__);
		return -EINVAL;
	}

	if (!pm_runtime_enabled(dev)) {
		dev_dbg(dev, "%s Run time PM not enabled\n", __func__);
		return 0;
	}

	if (mutex_trylock(&priv->pm_mutex)) {
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_sync_autosuspend(dev);
		if (ret && ret != -EBUSY)
			dev_dbg(dev, "%s(): fail %d\n", __func__, ret);
		mutex_unlock(&priv->pm_mutex);
	}

	if (ret == -EBUSY)
		ret = 0;
#endif
	return ret;
}

int iaxxx_wakeup_chip(struct iaxxx_priv *priv)
{
	int rc, reg_val;

	/* Enable external clock */
	if (priv->iaxxx_state->power_state == IAXXX_SLEEP_MODE) {
		if (priv->iaxxx_mclk_cb)
			priv->iaxxx_mclk_cb(priv, 1);
	}

	/* SPI_CS would act as wake up source. So making an
	 * SPI transaction to wake up the chip and then wait for 40ms
	 */
	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_STATUS_ADDR,
			&reg_val);

	msleep(40);

	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_STATUS_ADDR,
			&reg_val);
	rc = regmap_read(priv->regmap_no_pm, IAXXX_SRB_SYS_STATUS_ADDR,
			&reg_val);

	/* if read failed or SYS mode is not in APP mode, flag error*/
	reg_val &= IAXXX_SRB_SYS_STATUS_MODE_MASK;
	dev_dbg(priv->dev, "%s chip wake up reg_val%d\n", __func__, reg_val);
	if (rc || (reg_val != SYSTEM_STATUS_MODE_APPS)) {
		dev_err(priv->dev, "%s chip wake up failed %d rc %d\n",
				__func__, reg_val, rc);
		return -EIO;
	}

	if (priv->iaxxx_state->power_state == IAXXX_SLEEP_MODE) {
		priv->iaxxx_state->power_state = IAXXX_NORMAL_MODE;
		dev_info(priv->dev, "%s set to normal power mode done\n",
			__func__);
	}

	dev_info(priv->dev, "%s() Success\n", __func__);
	return 0;
}

int iaxxx_suspend_chip(struct iaxxx_priv *priv)
{
	int rc;

	/* set up the SPI speed thats expected when the system is wake up
	 * Set the SPI Speed to maximum so system will be awake with max
	 * clock speed.
	 */
	rc = regmap_write(priv->regmap_no_pm,
			IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_ADDR,
			IAXXX_PWR_MAX_SPI_SPEED);

	if (rc) {
		dev_err(priv->dev,
			"Failed to set Max SPI speed in %s\n", __func__);
		return rc;
	}

	/* Note: Control interface of second host should also be disabled
	 * to achieve optimal or sleep mode.
	 */
	/* SLEEP MODE: If plugin count is zero and route is inactive */
	if (list_empty_careful(&priv->iaxxx_state->plugin_head_list)
						&& !priv->route_status) {
		/* Enable external clock */
		if (priv->iaxxx_mclk_cb)
			priv->iaxxx_mclk_cb(priv, 1);

		/* Switch to external oscillator */
		rc = iaxxx_set_mpll_source_no_pm(priv, IAXXX_EXT_OSC);
		if (rc) {
			dev_err(priv->dev,
			"%s() Failed to set MPLL Clk Src to external\n",
							__func__);
			return rc;
		}

		/* Issue sleep power mode command */
		rc = regmap_update_bits(priv->regmap_no_pm,
				IAXXX_SRB_SYS_POWER_CTRL_ADDR,
				IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_MASK,
				(IAXXX_SLEEP_MODE <<
				IAXXX_SRB_SYS_POWER_CTRL_SET_POWER_MODE_POS));

		if (rc) {
			dev_err(priv->dev, "%s() Fail\n", __func__);
			return rc;
		}
		iaxxx_send_update_block_no_wait_no_pm(priv->dev, IAXXX_HOST_0);
		msleep(20);

		/* Disable external clock */
		if (priv->iaxxx_mclk_cb)
			priv->iaxxx_mclk_cb(priv, 0);

		priv->iaxxx_state->power_state = IAXXX_SLEEP_MODE;
		dev_info(priv->dev, "%s() chip put into sleep power mode\n",
								__func__);
	} else {
	/* OPTIMAL MODE: If plugin count is not zero or route is still active */

		/* Issue optimal power mode command */
		rc = regmap_update_bits(priv->regmap_no_pm,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK,
			(IAXXX_OPTIMAL_MODE <<
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_POS));

		if (rc) {
			dev_err(priv->dev, "%s() Fail\n", __func__);
			return rc;
		}

		iaxxx_send_update_block_no_wait_no_pm(priv->dev, IAXXX_HOST_0);
		msleep(20);

		priv->iaxxx_state->power_state = IAXXX_OPTIMAL_MODE;
		dev_info(priv->dev, "%s() chip put into optimal power mode\n",
								__func__);
	}
	dev_info(priv->dev, "%s() Success\n", __func__);

	return 0;
}

/*
 * iaxxx_pm_set_aclk, input value as follows.
 * 0 - 3.072 MHz
 * 1 - 6.144 MHz
 * 2 - 12.288 MHz
 * 3 - 24.576 MHz
 * 4 - 49.152 MHz
 * 5 - 98.304 MHz
 * 6 - 368.640 MHz
 * otherwise - Invalid
 */
int iaxxx_pm_set_aclk(struct device *dev, int clk_freq)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc, status;

	rc = regmap_update_bits(priv->regmap,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_MASK,
			clk_freq <<
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_OUT_FREQ_POS);
	if (!rc)
		rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK);

	if (!rc)
		rc = iaxxx_send_update_block_request(dev, &status,
				IAXXX_BLOCK_0);

	if (rc)
		dev_info(dev, "%s() Fail\n", __func__);
	else
		dev_info(dev, "%s() Success\n", __func__);
	return rc;
}

/*
 * iaxxx_pm_set_optimal_power_mode (host0)
 * Need to do wake up to come out
 */
int iaxxx_pm_set_optimal_power_mode_host0(struct device *dev)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;

	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = regmap_write(priv->regmap, IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_ADDR,
			priv->spi_app_speed);

	rc = regmap_update_bits(priv->regmap, IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_MASK,
			0x1 <<
			IAXXX_SRB_SYS_POWER_CTRL_DISABLE_CTRL_INTERFACE_POS);
	if (rc) {
		dev_info(dev, "%s() Fail\n", __func__);
		return rc;
	}

	iaxxx_send_update_block_no_wait(dev, IAXXX_HOST_0);

	msleep(20);
	priv->iaxxx_state->power_state = IAXXX_OPTIMAL_MODE;
	return rc;
}

/*
 * iaxxx_pm_set_optimal_power_mode (host1)
 * Need to do wake up to come out
 */
int iaxxx_pm_set_optimal_power_mode_host1(struct device *dev)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;

	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = regmap_write(priv->regmap, IAXXX_PWR_MGMT_MAX_SPI_SPEED_REQ_1_ADDR,
	    priv->spi_app_speed);
	if (!rc)
		rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_1_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_MASK,
			0x1 <<
			IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_POS);
	else {
		dev_info(dev, "%s() Fail\n", __func__);
		return rc;
	}

	iaxxx_send_update_block_no_wait(dev, IAXXX_HOST_1);
	msleep(20);
	return rc;
}

/*
 * iaxxx_set_speed_spi2
 */
int iaxxx_set_spi2_master_speed(struct device *dev, int spi_speed)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc, status;

	/* Write speed for SPI2 interface */
	rc = regmap_write(priv->regmap,
			IAXXX_PWR_MGMT_MAX_SPI2_MASTER_SPEED_REQ_ADDR,
			spi_speed);

	if (rc) {
		dev_info(dev, "%s() Fail rc = %d\n", __func__, rc);
		return rc;
	}

	rc = iaxxx_send_update_block_request(dev, &status,
			IAXXX_BLOCK_0);

	return rc;
}

int iaxxx_set_mpll_source(struct iaxxx_priv *priv, int source)
{
	int rc;

	rc = regmap_update_bits(priv->regmap, IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
			IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_MASK,
			source << IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_POS);
	if (!rc)
		rc = regmap_update_bits(priv->regmap,
			IAXXX_SRB_SYS_POWER_CTRL_ADDR,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_MASK,
			IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_MASK);

	if (!rc)
		rc = iaxxx_send_update_block_no_wait_no_pm(priv->dev,
				IAXXX_HOST_0);

	if (rc) {
		dev_err(priv->dev, "%s failed error code = %d\n", __func__, rc);
		return rc;
	}

	msleep(20);
	return rc;
}

int iaxxx_set_mpll_source_no_pm(struct iaxxx_priv *priv, int source)
{
	int rc;

	/* Disable control interface 1 */
	rc = regmap_update_bits(priv->regmap_no_pm,
		IAXXX_SRB_SYS_POWER_CTRL_1_ADDR,
		IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_MASK,
		(1 <<
		IAXXX_SRB_SYS_POWER_CTRL_1_DISABLE_CTRL_INTERFACE_POS));

	if (rc) {
		dev_err(priv->dev,
			"%s() disabling controle interface 1 Fail\n", __func__);
			return rc;
	}

	iaxxx_send_update_block_no_wait_no_pm(priv->dev, IAXXX_HOST_1);

	msleep(20);

	rc = regmap_update_bits(priv->regmap_no_pm,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_MASK,
		(source << IAXXX_PWR_MGMT_SYS_CLK_CTRL_MPLL_SRC_POS));

	if (rc) {
		dev_err(priv->dev, "%s() Failed to set MPLL err = %d\n",
							__func__, rc);
		return rc;
	}

	rc = regmap_update_bits(priv->regmap_no_pm,
		IAXXX_SRB_SYS_POWER_CTRL_ADDR,
		IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_MASK,
		(1 << IAXXX_SRB_SYS_POWER_CTRL_CONFIG_MPLL_POS));

	if (rc) {
		dev_err(priv->dev, "%s() Fail err = %d\n", __func__, rc);
		return rc;
	}

	rc = iaxxx_send_update_block_no_wait_no_pm(priv->dev, IAXXX_HOST_0);
	if (rc) {
		dev_err(priv->dev, "%s() Fail err = %d\n", __func__, rc);
		return rc;
	}

	msleep(20);
	return rc;
}

int iaxxx_set_apll_source(struct iaxxx_priv *priv, int source)
{
	int rc;
	u32 efuse_trim_value, status = 0;

	rc = regmap_update_bits(priv->regmap, IAXXX_PWR_MGMT_SYS_CLK_CTRL_ADDR,
		IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_SRC_MASK,
		source << IAXXX_PWR_MGMT_SYS_CLK_CTRL_APLL_SRC_POS);

	if (!rc)
		rc = regmap_update_bits(priv->regmap,
				IAXXX_SRB_SYS_POWER_CTRL_ADDR,
				IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK,
				IAXXX_SRB_SYS_POWER_CTRL_CONFIG_APLL_MASK);

	regmap_read(priv->regmap, IAXXX_AO_EFUSE_BOOT_ADDR, &efuse_trim_value);
	efuse_trim_value = (efuse_trim_value >> 25) & 0x7F;

	if (efuse_trim_value) {
		rc = regmap_update_bits(priv->regmap, IAXXX_AO_OSC_CTRL_ADDR,
			IAXXX_AO_OSC_CTRL_ADJ_MASK, efuse_trim_value);
	}

	if (!rc)
		rc = iaxxx_send_update_block_request(priv->dev,
				&status, IAXXX_BLOCK_0);
	if (rc)
		pr_err("%s failed to set up apll source err = %0x\n",
					__func__, rc);
	return rc;

}

/*
 * iaxxx_get_max_spi_speed
 * Gets the Max SPI Speed supported in the optimal power mode
 * max_spi_speed   Maximum SPI speed
 */
int iaxxx_get_max_spi_speed(struct device *dev, uint32_t *max_spi_speed)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;

	rc = regmap_read(priv->regmap,
			IAXXX_PWR_MGMT_MAX_SPI_SPEED_ADDR,
			max_spi_speed);
	if (rc) {
		dev_err(dev,
		"%s() Fail to read PWR_MGMT_MAX_SPI_SPEED_ADDR rc = %d\n",
								__func__, rc);
	}

	return rc;
}

/*****************************************************************************
 * iaxxx_core_get_pwr_stats()
 * @brief get power transition count for each xclk and aclk values.
 *
 * @pwr_stats power statistics for mpll and apll values
 *
 * @ret n number of words read, ret in case of error
 ****************************************************************************/
int iaxxx_core_get_pwr_stats(struct device *dev,
			struct iaxxx_pwr_stats *pwr_stats)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t pwr_stats_addr = 0;
	uint32_t pwr_stats_size = 0;
	int ret = 0;

	ret = regmap_read(priv->regmap,
		IAXXX_PWR_MGMT_PWR_MGMT_STATS_PTR_ADDR, &pwr_stats_addr);
	if (ret) {
		dev_err(dev, "%s() failed %d\n", __func__, ret);
		goto exit;
	}

	ret = regmap_read(priv->regmap,
		IAXXX_PWR_MGMT_PWR_MGMT_STATS_SIZE_ADDR, &pwr_stats_size);
	if (ret) {
		dev_err(dev, "%s() failed %d\n", __func__, ret);
		goto exit;
	}

	if (pwr_stats_size != (sizeof(struct iaxxx_pwr_stats) >> 2)) {
		dev_err(dev,
			"%s() pwr_stats_size %d != struct size %zu\n",
			__func__, pwr_stats_size,
			sizeof(struct iaxxx_pwr_stats) >> 2);
		ret = -EINVAL;
		goto exit;
	}

	ret = priv->bulk_read(priv->dev,
		pwr_stats_addr, pwr_stats, pwr_stats_size);
	if (ret != pwr_stats_size) {
		dev_err(priv->dev, "Not able to read pwr stats %d\n",
				ret);
		goto exit;
	}

	dev_dbg(priv->dev, "read pwr stats successfully in words %u\n",
			pwr_stats_size);
exit:
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_get_pwr_stats);
