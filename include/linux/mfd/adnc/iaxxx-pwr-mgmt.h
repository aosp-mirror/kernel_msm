/*
 * iaxxx-pwr-mgmt.h -- iaxxx power management
 *
 * Copyright 2018 Knowles Corporation
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

#ifndef _IAXXX_PWR_MGMT_H
#define _IAXXX_PWR_MGMT_H

int iaxxx_wakeup_chip(struct iaxxx_priv *priv);
int iaxxx_suspend_chip(struct iaxxx_priv *priv);
int iaxxx_pm_get_sync(struct device *dev);
int iaxxx_pm_put_autosuspend(struct device *dev);
int iaxxx_pm_put_sync_suspend(struct device *dev);
void iaxxx_pm_enable(struct iaxxx_priv *priv);
int iaxxx_pm_set_aclk(struct device *dev, int clk_freq);
int iaxxx_pm_set_optimal_power_mode_host0(struct device *dev);
int iaxxx_pm_set_optimal_power_mode_host1(struct device *dev);
int iaxxx_set_spi2_master_speed(struct device *dev, int spi_speed);
int iaxxx_set_mpll_source(struct iaxxx_priv *priv, int source);
int iaxxx_set_mpll_source_no_pm(struct iaxxx_priv *priv, int source);
int iaxxx_set_apll_source(struct iaxxx_priv *priv, int source);
int iaxxx_get_max_spi_speed(struct device *dev, uint32_t *max_spi_speed);

#endif /* _IAXXX_PWR_MGMT_H */
