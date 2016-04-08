/*
 * cyttsp5_core.h
 * Cypress TrueTouch(TM) Standard Product V5 Core Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2014 Cypress Semiconductor
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
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#ifndef _LINUX_CYTTSP5_CORE_H
#define _LINUX_CYTTSP5_CORE_H

#include <linux/stringify.h>

#define CYTTSP5_I2C_NAME "cyttsp5_i2c_adapter"
#define CYTTSP5_SPI_NAME "cyttsp5_spi_adapter"

#define CYTTSP5_CORE_NAME "cyttsp5_core"
#define CYTTSP5_MT_NAME "cyttsp5_mt"
#define CYTTSP5_BTN_NAME "cyttsp5_btn"
#define CYTTSP5_PROXIMITY_NAME "cyttsp5_proximity"

#define CY_DRIVER_NAME TTDA
#define CY_DRIVER_MAJOR 03
#define CY_DRIVER_MINOR 04

#define CY_DRIVER_REVCTRL 721053
/* if wakeup fail, we will try #CY_WAKEUP_RETRY_TIMES to wake */
#define CY_WAKEUP_RETRY_TIMES 2

#define CY_DRIVER_VERSION		    \
__stringify(CY_DRIVER_NAME)		    \
"." __stringify(CY_DRIVER_MAJOR)	    \
"." __stringify(CY_DRIVER_MINOR)	    \
"." __stringify(CY_DRIVER_REVCTRL)

#define CY_DRIVER_DATE "20141006"	/* YYYYMMDD */

/* abs settings */
#define CY_IGNORE_VALUE             -1

#define FW_UPDATE_TO_HIGH 0
#define FW_UPDATE_DIFF 1
enum cyttsp5_core_platform_flags {
	CY_CORE_FLAG_NONE,
	CY_CORE_FLAG_WAKE_ON_GESTURE = 0x01,
	CY_CORE_FLAG_POWEROFF_ON_SLEEP = 0x02,
	CY_CORE_FLAG_RESTORE_PARAMETERS = 0x04,
};

enum cyttsp5_core_platform_easy_wakeup_gesture {
	CY_CORE_EWG_NONE,
	CY_CORE_EWG_TAP_TAP,
	CY_CORE_EWG_TWO_FINGER_SLIDE,
	CY_CORE_EWG_RESERVED,
	CY_CORE_EWG_WAKE_ON_INT_FROM_HOST = 0xFF,
};

enum cyttsp5_loader_platform_flags {
	CY_LOADER_FLAG_NONE,
	CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE,
	/* Use CONFIG_VER field in TT_CFG to decide TT_CFG update */
	CY_LOADER_FLAG_CHECK_TTCONFIG_VERSION,
	CY_LOADER_FLAG_CALIBRATE_AFTER_TTCONFIG_UPGRADE,
};

enum cyttsp5_power_type {
	CY_POWER_NONE,
	CY_POWER_GPIO,
	CY_POWER_PMU,
};

struct cyttsp5_power_config {
	u8 vdd_type;
	u8 vbus_type;
	int vdd_en_gpio;
	int vbus_en_gpio;
	struct regulator* vdd_reg;
	struct regulator* vbus_reg;
	char const* vdd_name;
	char const* vbus_name;
};

struct touch_settings {
	const uint8_t   *data;
	uint32_t         size;
	uint8_t         tag;
};

struct cyttsp5_touch_firmware {
	const uint8_t *img;
	uint32_t size;
	const uint8_t *ver;
	uint8_t vsize;
	uint8_t panel_id;
};

struct cyttsp5_touch_config {
	struct touch_settings *param_regs;
	struct touch_settings *param_size;
	const uint8_t *fw_ver;
	uint8_t fw_vsize;
	uint8_t panel_id;
	char *  product_name;
};

struct cyttsp5_loader_platform_data {
	struct cyttsp5_touch_firmware *fw;
	struct cyttsp5_touch_config *ttconfig;
	struct cyttsp5_touch_firmware **fws;
	struct cyttsp5_touch_config **ttconfigs;
	u32 flags;
};

struct cyttsp5_wakeup_keys {
	const uint16_t  *keys;
	uint8_t         size;
	uint8_t         enable_wkeys;	/*just padding, not used*/
};

enum cyttsp5_gesture_num {
	GESTURE_DOUBLE_CLICK = 0,
	GESTURE_SLIDE_L2R,
	GESTURE_SLIDE_R2L,
	GESTURE_SLIDE_T2B,
	GESTURE_SLIDE_B2T,
	GESTURE_SLIDE_T2B2,
	GESTURE_CIRCLE_SLIDE,
	GESTURE_LETTER_c,
	GESTURE_LETTER_e,
	GESTURE_LETTER_m,
	GESTURE_LETTER_w,
	GESTURE_PALM_COVERED,
	GESTURE_MAX,
};

#define REPORT_GESTURE_LOCUS 1
#define GESTURE_TIMER_INTERVAL  HZ //jiffies 1S


#define CYTTSP5_SIZE_OF_LETTER_DATA 30
#define CYTTSP5_SIZE_OF_CLICK_DATA 14
#define CYTTSP5_LETTER_LOCUS_NUM 6
#define CYTTSP5_CLICK_LOCUS_NUM 2
#define IS_GESTURE_ENABLE(gesture, var) (APP_ENABLE_GESTURE((gesture)) & (var))

#define APP_ENABLE_GESTURE(x)  ((u32)(1<<(x)))

typedef int (*cyttsp5_platform_read) (struct device *dev, void *buf, int size);

#define CY_TOUCH_SETTINGS_MAX 32

struct cyttsp5_core_platform_data {
	int irq_gpio;
	int rst_gpio;
	int level_irq_udelay;
	u16 hid_desc_register;
	u16 vendor_id;
	u16 product_id;
	const char *product_name;
	const char *chip_name;

	struct cyttsp5_power_config * power_config;

	int (*xres)(struct cyttsp5_core_platform_data *pdata,
		struct device *dev);
	int (*init)(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev);
	int (*power)(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq);
	int (*detect)(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read);
	int (*irq_stat)(struct cyttsp5_core_platform_data *pdata,
		struct device *dev);
	struct touch_settings *sett[CY_TOUCH_SETTINGS_MAX];
	u32 flags;
	unsigned int easy_wakeup_gesture;
	unsigned int easy_wakeup_supported_gestures;
	unsigned int double_tap_enabled;
	int dtz_x0;
	int dtz_y0;
	int dtz_x1;
	int dtz_y1;
	struct cyttsp5_wakeup_keys *wakeup_keys;

	int fw_upgrade_start_ver;
	unsigned int glove_support;
	unsigned int holster_support;
	int fw_update_logic;
	int input_register_early;
	unsigned int mmi_test_support;
};

struct touch_framework {
	const int16_t  *abs;
	uint8_t         size;
	uint8_t         enable_vkeys;
} __packed;

enum cyttsp5_mt_platform_flags {
	CY_MT_FLAG_NONE,
	CY_MT_FLAG_HOVER = 0x04,
	CY_MT_FLAG_FLIP = 0x08,
	CY_MT_FLAG_INV_X = 0x10,
	CY_MT_FLAG_INV_Y = 0x20,
	CY_MT_FLAG_VKEYS = 0x40,
	CY_MT_FLAG_NO_TOUCH_ON_LO = 0x80,
};

struct cyttsp5_mt_platform_data {
	struct touch_framework *frmwrk;
	unsigned short flags;
	char const *inp_dev_name;
	int vkeys_x;
	int vkeys_y;
	int x_max;
	int y_max;
	int p_max;
	int slots_max;
};

struct cyttsp5_btn_platform_data {
	char const *inp_dev_name;
};

struct cyttsp5_proximity_platform_data {
	struct touch_framework *frmwrk;
	char const *inp_dev_name;
};

struct cyttsp5_platform_data {
	struct cyttsp5_core_platform_data *core_pdata;
	struct cyttsp5_mt_platform_data *mt_pdata;
	struct cyttsp5_btn_platform_data *btn_pdata;
	struct cyttsp5_proximity_platform_data *prox_pdata;
	struct cyttsp5_loader_platform_data *loader_pdata;
};

static inline struct cyttsp5_platform_data *cyttsp5_get_platform_data(struct device *dev)
{
	return (struct cyttsp5_platform_data *)dev->platform_data;
}

int cyttsp5_command_response(struct device *dev,u8 *buf);
int cyttsp5_send_command(struct device *dev,char *buf);

#endif /* _LINUX_CYTTSP5_CORE_H */
