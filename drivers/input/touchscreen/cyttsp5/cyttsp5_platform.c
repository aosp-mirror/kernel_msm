/*
 * cyttsp5_platform.c
 * Parade TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2013-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 *
 */

#include "cyttsp5_regs.h"
#include <linux/cyttsp5_platform.h>

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "cyttsp5_fw_pid00.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid00 = {
	.img = cyttsp4_img_pid00,
	.size = ARRAY_SIZE(cyttsp4_img_pid00),
	.ver = cyttsp4_ver_pid00,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "cyttsp5_fw_pid01.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid01 = {
	.img = cyttsp4_img_pid01,
	.size = ARRAY_SIZE(cyttsp4_img_pid01),
	.ver = cyttsp4_ver_pid01,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "cyttsp5_params_pid00.h"
static struct touch_settings cyttsp5_sett_param_regs_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid00),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid00 = {
	.param_regs = &cyttsp5_sett_param_regs_pid00,
	.param_size = &cyttsp5_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "cyttsp5_params_pid01.h"
static struct touch_settings cyttsp5_sett_param_regs_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid01),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid01 = {
	.param_regs = &cyttsp5_sett_param_regs_pid01,
	.param_size = &cyttsp5_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct cyttsp5_touch_firmware *cyttsp5_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	&cyttsp5_firmware_pid00,
	&cyttsp5_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct cyttsp5_touch_config *cyttsp5_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	&cyttsp5_ttconfig_pid00,
	&cyttsp5_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.fws = cyttsp5_firmwares,
	.ttconfigs = cyttsp5_ttconfigs,
	.flags = CY_LOADER_FLAG_NONE,
};

int cyttsp5_pinctrl_configure(struct device *dev,
		struct pinctrl *pinctrl, const char *pin_state)
{
	struct pinctrl_state *state;
	int rc;

	state = pinctrl_lookup_state(pinctrl, pin_state);
	if (IS_ERR(state)) {
		dev_err(dev, "cannot get ts pinctrl %s state\n", pin_state);
		return PTR_ERR(state);
	}

	rc = pinctrl_select_state(pinctrl, state);
	if (rc < 0) {
		dev_err(dev, "cannot set ts pinctrl state\n");
		return rc;
	}

	return 0;
}

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = -ENODEV;

	if (gpio_is_valid(rst_gpio)) {
		rc = 0;
//		gpio_set_value(rst_gpio, 1);
//		msleep(20);
		gpio_set_value(rst_gpio, 0);
		msleep(20);
		gpio_set_value(rst_gpio, 1);
		msleep(4);
	}
	dev_info(dev, "%s: RESET CYTTSP gpio=%d r=%d\n",
		__func__, rst_gpio, rc);
	return rc;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (on) {
		if (gpio_is_valid(rst_gpio)) {
			rc = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH, "tp-reset");
			if (rc < 0) {
				dev_err(dev, "%s: Fail request rst_gpio=%d\n",
						__func__, rst_gpio);
				pdata->rst_gpio = -1;
			}
		}

		rc = devm_gpio_request_one(dev, irq_gpio, GPIOF_DIR_IN, "tp-irq");
		if (rc < 0) {
			dev_err(dev, "%s: Fail request irq_gpio=%d\n",
					__func__, irq_gpio);
			pdata->irq_gpio = -1;
		}
	}

	dev_info(dev, "%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}

static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int rc;

	if (pdata->vdd_reg != NULL) {
		rc = regulator_enable(pdata->vdd_reg);
		if (rc < 0) {
			dev_err(dev, "%s: VDD Power up fails r=%d\n", __func__, rc);
			return rc;
		}
	}

	if (pdata->vddio_reg != NULL) {
		rc = regulator_enable(pdata->vddio_reg);
		if (rc < 0) {
			dev_err(dev, "%s: VDD-IO Power up fails r=%d\n", __func__, rc);
			return rc;
		}
	}

	if (pdata->bus_pullup_reg != NULL) {
		rc = regulator_enable(pdata->bus_pullup_reg);
		if (rc < 0) {
			dev_err(dev, "%s: VDD-BUS-PULL-UP Power up fails r=%d\n", __func__, rc);
			return rc;
		}
	}

	if (pdata->pinctrl != NULL) {
		rc = cyttsp5_pinctrl_configure(dev, pdata->pinctrl, "pmx_ts_active");
		if (rc < 0) {
			dev_err(dev, "cannot set ts pinctrl active state\n");
			return rc;
		}
	}

#ifdef CONFIG_OPPO
    /* WSW.BSP.Kernel, 2020-3-15 repower and reset the TP*/
    cyttsp5_xres(pdata,dev);
#endif

	return 0;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int rc;

	if (pdata->pinctrl != NULL) {
		rc = cyttsp5_pinctrl_configure(dev, pdata->pinctrl, "pmx_ts_release");
		if (rc < 0) {
			dev_err(dev, "cannot set ts pinctrl sleep state\n");
			return rc;
		}
	}

	if (pdata->vddio_reg != NULL) {
		rc = regulator_disable(pdata->vddio_reg);
		if (rc < 0) {
			dev_err(dev, "%s: VDD-IO Power down fails r=%d\n",
						__func__, rc);
			return rc;
		}
	}

	if (pdata->vdd_reg != NULL) {
		rc = regulator_disable(pdata->vdd_reg);
		if (rc < 0) {
			dev_err(dev, "%s: VDD Power down fails r=%d\n",
						__func__, rc);
			return rc;
		}
	}

	if (pdata->bus_pullup_reg != NULL) {
		rc = regulator_disable(pdata->bus_pullup_reg);
		if (rc < 0) {
			dev_err(dev, "%s: VDD-BUS-PULL-UP Power down fails r=%d\n", __func__, rc);
			return rc;
		}
	}

	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef CYTTSP5_DETECT_HW
int cyttsp5_detect(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		parade_debug(dev, DEBUG_LEVEL_2, "%s: Performing a reset\n",
			__func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		parade_debug(dev, DEBUG_LEVEL_2, "%s: Read unsuccessful, try=%d\n",
			__func__, 3 - retry);
	}

	return rc;
}
#endif
