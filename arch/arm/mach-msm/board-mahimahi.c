/* linux/arch/arm/mach-msm/board-mahimahi.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
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

#include <linux/cy8c_tmg_ts.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>

#include <linux/android_pmem.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/a1026.h>
#include <linux/capella_cm3602.h>
#include <linux/akm8973.h>
#include <linux/regulator/machine.h>
#include <linux/ds2784_battery.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#include <../../../drivers/w1/w1.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>
#include <mach/msm_serial_hs.h>
#include <mach/bcm_bt_lpm.h>
#include <mach/msm_smd.h>

#include "board-mahimahi.h"
#include "devices.h"
#include "proc_comm.h"
#include "board-mahimahi-flashlight.h"
#include "board-mahimahi-tpa2018d1.h"
#include "board-mahimahi-smb329.h"

static uint debug_uart;

module_param_named(debug_uart, debug_uart, uint, 0);

extern void notify_usb_connected(int);
extern void msm_init_pmic_vibrator(void);
extern void __init mahimahi_audio_init(void);

extern int microp_headset_has_mic(void);

static void config_gpio_table(uint32_t *table, int len);

static int mahimahi_phy_init_seq[] = {
	0x0C, 0x31,
	0x31, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1 };

static void mahimahi_usb_phy_reset(void)
{
	u32 id;
	int ret;

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}

	msleep(1);

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}

static void mahimahi_usb_hw_reset(bool enable)
{
	u32 id;
	int ret;
	u32 func;

	id = PCOM_CLKRGM_APPS_RESET_USBH;
	if (enable)
		func = PCOM_CLK_REGIME_SEC_RESET_ASSERT;
	else
		func = PCOM_CLK_REGIME_SEC_RESET_DEASSERT;
	ret = msm_proc_comm(func, &id, NULL);
	if (ret)
		pr_err("%s: Cannot set reset to %d (%d)\n", __func__, enable,
		       ret);
}


static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= mahimahi_phy_init_seq,
	.phy_reset		= mahimahi_usb_phy_reset,
	.hw_reset		= mahimahi_usb_hw_reset,
	.usb_connected		= notify_usb_connected,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x4e11,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x4e12,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x4e13,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x4e14,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x4e17,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
#endif
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Google, Inc.",
	.product	= "Nexus One",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x18d1,
	.vendorDescr	= "Google, Inc.",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x18d1,
	.product_id	= 0x4e11,
	.version	= 0x0100,
	.product_name		= "Nexus One",
	.manufacturer_name	= "Google, Inc.",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static struct platform_device mahimahi_rfkill = {
	.name = "mahimahi_rfkill",
	.id = -1,
};

static struct resource msm_kgsl_resources[] = {
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_phys_memory",
		.start	= MSM_GPU_MEM_BASE,
		.end	= MSM_GPU_MEM_BASE + MSM_GPU_MEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

#define PWR_RAIL_GRP_CLK		8
static int mahimahi_kgsl_power_rail_mode(int follow_clk)
{
	int mode = follow_clk ? 0 : 1;
	int rail_id = PWR_RAIL_GRP_CLK;

	return msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
}

static int mahimahi_kgsl_power(bool on)
{
	int cmd;
	int rail_id = PWR_RAIL_GRP_CLK;

	cmd = on ? PCOM_CLKCTL_RPC_RAIL_ENABLE : PCOM_CLKCTL_RPC_RAIL_DISABLE;
	return msm_proc_comm(cmd, &rail_id, NULL);
}

static struct platform_device msm_kgsl_device = {
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.start		= MSM_PMEM_MDP_BASE,
	.size		= MSM_PMEM_MDP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name		= "pmem_camera",
	.start		= MSM_PMEM_CAMERA_BASE,
	.size		= MSM_PMEM_CAMERA_SIZE,
	.no_allocator	= 1,
	.cached		= 1,
};

static struct platform_device android_pmem_mdp_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_camera_device = {
	.name		= "android_pmem",
	.id		= 2,
	.dev		= {
		.platform_data = &android_pmem_camera_pdata,
	},
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int mahimahi_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		/* level shifter should be off */
		gpio_set_value(MAHIMAHI_GPIO_TP_EN, 1);
		msleep(120);
		/* enable touch panel level shift */
		gpio_set_value(MAHIMAHI_GPIO_TP_LS_EN, 1);
		msleep(3);
	} else {
		gpio_set_value(MAHIMAHI_GPIO_TP_LS_EN, 0);
		gpio_set_value(MAHIMAHI_GPIO_TP_EN, 0);
		udelay(50);
	}

	return 0;
}

struct cy8c_i2c_platform_data mahimahi_cy8c_ts_data = {
	.version = 0x0001,
	.abs_x_min = 0,
	.abs_x_max = 479,
	.abs_y_min = 0,
	.abs_y_max = 799,
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
	.abs_width_min = 0,
	.abs_width_max = 10,
	.power = mahimahi_ts_power,
};

static struct synaptics_i2c_rmi_platform_data mahimahi_synaptics_ts_data[] = {
	{
		.version = 0x105,
		.power = mahimahi_ts_power,
		.flags = SYNAPTICS_FLIP_Y,
		.inactive_left = -15 * 0x10000 / 480,
		.inactive_right = -15 * 0x10000 / 480,
		.inactive_top = -15 * 0x10000 / 800,
		.inactive_bottom = -50 * 0x10000 / 800,
		.sensitivity_adjust = 9,
	},
	{
		.flags = SYNAPTICS_FLIP_Y,
		.inactive_left = -15 * 0x10000 / 480,
		.inactive_right = -15 * 0x10000 / 480,
		.inactive_top = -15 * 0x10000 / 800,
		.inactive_bottom = -40 * 0x10000 / 800,
		.sensitivity_adjust = 12,
	},
};

static struct a1026_platform_data a1026_data = {
	.gpio_a1026_micsel = MAHIMAHI_AUD_MICPATH_SEL,
	.gpio_a1026_wakeup = MAHIMAHI_AUD_A1026_WAKEUP,
	.gpio_a1026_reset = MAHIMAHI_AUD_A1026_RESET,
	.gpio_a1026_clk = MAHIMAHI_AUD_A1026_CLK,
	/*.gpio_a1026_int = MAHIMAHI_AUD_A1026_INT,*/
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = MAHIMAHI_LAYOUTS,
	.project_name = MAHIMAHI_PROJECT_NAME,
	.reset = MAHIMAHI_GPIO_COMPASS_RST_N,
	.intr = MAHIMAHI_GPIO_COMPASS_INT_N,
};

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] = {
	{
		.supply = "acpu_vcore",
	},
};

static struct regulator_init_data tps65023_data[5] = {
	{
		.constraints = {
			.name = "dcdc1", /* VREG_MSMC2_1V29 */
			.min_uV = 1000000,
			.max_uV = 1300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		},
		.consumer_supplies = tps65023_dcdc1_supplies,
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
	},
	/* dummy values for unused regulators to not crash driver: */
	{
		.constraints = {
			.name = "dcdc2", /* VREG_MSMC1_1V26 */
			.min_uV = 1260000,
			.max_uV = 1260000,
		},
	},
	{
		.constraints = {
			.name = "dcdc3", /* unused */
			.min_uV = 800000,
			.max_uV = 3300000,
		},
	},
	{
		.constraints = {
			.name = "ldo1", /* unused */
			.min_uV = 1000000,
			.max_uV = 3150000,
		},
	},
	{
		.constraints = {
			.name = "ldo2", /* V_USBPHY_3V3 */
			.min_uV = 3300000,
			.max_uV = 3300000,
		},
	},
};


static void ds2482_set_slp_n(unsigned n)
{
	gpio_direction_output(MAHIMAHI_GPIO_DS2482_SLP_N, n);
}

static struct tpa2018d1_platform_data tpa2018_data = {
	.gpio_tpa2018_spk_en = MAHIMAHI_CDMA_GPIO_AUD_SPK_AMP_EN,
};

static struct i2c_board_info base_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ds2482", 0x30 >> 1),
		.platform_data = ds2482_set_slp_n,
	},
	{
		I2C_BOARD_INFO("cy8c-tmg-ts", 0x34),
		.platform_data = &mahimahi_cy8c_ts_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_TP_INT_N),
	},
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x40),
		.platform_data = mahimahi_synaptics_ts_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_TP_INT_N)
	},
	{
		I2C_BOARD_INFO("mahimahi-microp", 0x66),
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
};

static struct i2c_board_info rev0_i2c_devices[] = {
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_REV0_GPIO_COMPASS_INT_N),
	},
};

static struct i2c_board_info rev1_i2c_devices[] = {
	{
		I2C_BOARD_INFO("audience_a1026", 0x3E),
		.platform_data = &a1026_data,
		/*.irq = MSM_GPIO_TO_INT(MAHIMAHI_AUD_A1026_INT)*/
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_COMPASS_INT_N),
	},
};

static struct i2c_board_info rev_CX_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tpa2018d1", 0x58),
		.platform_data = &tpa2018_data,
	},
	{
		I2C_BOARD_INFO("smb329", 0x6E >> 1),
	},
};

static void config_gpio_table(uint32_t *table, int len);

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
};

void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		 INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name = "s5k3e2fx",
	.sensor_reset = 144, /* CAM1_RST */
	.sensor_pwd = 143,  /* CAM1_PWDN, enabled in a9 */
	/*.vcm_pwd = 31, */  /* CAM1_VCM_EN, enabled in a9 */
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.camera_flash = flashlight_control,
	.num_flash_levels = FLASHLIGHT_NUM,
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev      = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};

static int capella_cm3602_power(int on)
{
	/* TODO eolsen Add Voltage reg control */
	if (on) {
		gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 0);
	} else {
		gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 1);
	}

	return 0;
}


static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_out = MAHIMAHI_GPIO_PROXIMITY_INT_N
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};

static uint32_t flashlight_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
};

static uint32_t flashlight_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(MAHIMAHI_CDMA_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
};


static int config_mahimahi_flashlight_gpios(void)
{
	if (is_cdma_version(system_rev)) {
		config_gpio_table(flashlight_gpio_table_rev_CX,
			ARRAY_SIZE(flashlight_gpio_table_rev_CX));
	} else {
		config_gpio_table(flashlight_gpio_table,
			ARRAY_SIZE(flashlight_gpio_table));
	}
	return 0;
}

static struct flashlight_platform_data mahimahi_flashlight_data = {
	.gpio_init  = config_mahimahi_flashlight_gpios,
	.torch = MAHIMAHI_GPIO_FLASHLIGHT_TORCH,
	.flash = MAHIMAHI_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device mahimahi_flashlight_device = {
	.name = "flashlight",
	.dev = {
		.platform_data  = &mahimahi_flashlight_data,
	},
};
static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = MAHIMAHI_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device mahimahi_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = -1,
	.inject_rx_on_wakeup = 0,
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = MAHIMAHI_GPIO_BT_WAKE,
	.gpio_host_wake = MAHIMAHI_GPIO_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off_locked,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};

static int ds2784_charge(int on, int fast)
{
	if (is_cdma_version(system_rev)) {
		if (!on)
			smb329_set_charger_ctrl(SMB329_DISABLE_CHG);
		else
			smb329_set_charger_ctrl(fast ? SMB329_ENABLE_FAST_CHG : SMB329_ENABLE_SLOW_CHG);
	}
	else
		gpio_direction_output(MAHIMAHI_GPIO_BATTERY_CHARGER_CURRENT, !!fast);
	gpio_direction_output(MAHIMAHI_GPIO_BATTERY_CHARGER_EN, !on);
	return 0;
}

static int w1_ds2784_add_slave(struct w1_slave *sl)
{
	struct dd {
		struct platform_device pdev;
		struct ds2784_platform_data pdata;
	} *p;

	int rc;

	p = kzalloc(sizeof(struct dd), GFP_KERNEL);
	if (!p) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	rc = gpio_request(MAHIMAHI_GPIO_BATTERY_CHARGER_EN, "charger_en");
	if (rc < 0) {
		pr_err("%s: gpio_request(%d) failed: %d\n", __func__,
			MAHIMAHI_GPIO_BATTERY_CHARGER_EN, rc);
		kfree(p);
		return rc;
	}

	if (!is_cdma_version(system_rev)) {
		rc = gpio_request(MAHIMAHI_GPIO_BATTERY_CHARGER_CURRENT, "charger_current");
		if (rc < 0) {
			pr_err("%s: gpio_request(%d) failed: %d\n", __func__,
				MAHIMAHI_GPIO_BATTERY_CHARGER_CURRENT, rc);
			gpio_free(MAHIMAHI_GPIO_BATTERY_CHARGER_EN);
			kfree(p);
			return rc;
		}
	}

	p->pdev.name = "ds2784-battery";
	p->pdev.id = -1;
	p->pdev.dev.platform_data = &p->pdata;
	p->pdata.charge = ds2784_charge;
	p->pdata.w1_slave = sl;

	platform_device_register(&p->pdev);

	return 0;
}

static struct w1_family_ops w1_ds2784_fops = {
	.add_slave = w1_ds2784_add_slave,
};

static struct w1_family w1_ds2784_family = {
	.fid = W1_FAMILY_DS2784,
	.fops = &w1_ds2784_fops,
};

static int __init ds2784_battery_init(void)
{
	return w1_register_family(&w1_ds2784_family);
}

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
	&bcm_bt_lpm_device,
	&msm_device_uart_dm1,
	&ram_console_device,
	&mahimahi_rfkill,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_hsusb,
	&usb_mass_storage_device,
#ifdef CONFIG_USB_ANDROID_RNDIS
	&rndis_device,
#endif
	&android_usb_device,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
	&android_pmem_camera_device,
	&msm_kgsl_device,
	&msm_device_i2c,
	&capella_cm3602,
	&msm_camera_sensor_s5k3e2fx,
	&mahimahi_flashlight_device,
};


static uint32_t bt_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static uint32_t bt_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_CDMA_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static uint32_t misc_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_LCD_RST_N, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_LED_3V3_EN, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_DOCK, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t key_int_shutdown_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_35MM_KEY_INT_SHUTDOWN, 0, GPIO_OUTPUT,
		      GPIO_NO_PULL, GPIO_2MA),
};

static void mahimahi_headset_init(void)
{
	if (is_cdma_version(system_rev))
		return;
	config_gpio_table(key_int_shutdown_gpio_table,
			ARRAY_SIZE(key_int_shutdown_gpio_table));
	gpio_set_value(MAHIMAHI_GPIO_35MM_KEY_INT_SHUTDOWN, 0);
}

#define ATAG_BDADDR 0x43294329  /* mahimahi bluetooth address tag */
#define ATAG_BDADDR_SIZE 4
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

static int __init parse_tag_bdaddr(const struct tag *tag)
{
	unsigned char *b = (unsigned char *)&tag->u;

	if (tag->hdr.size != ATAG_BDADDR_SIZE)
		return -EINVAL;

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
			b[0], b[1], b[2], b[3], b[4], b[5]);

        return 0;
}

__tagtable(ATAG_BDADDR, parse_tag_bdaddr);

static int __init board_serialno_setup(char *serialno)
{
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
#endif

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static struct msm_acpu_clock_platform_data mahimahi_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
	.mpll_khz		= 245000
};

static struct msm_acpu_clock_platform_data mahimahi_cdma_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 235930,
	.wait_for_irq_khz	= 235930,
	.mpll_khz		= 235930
};

static ssize_t mahimahi_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	if (system_rev > 2 && system_rev != 0xC0) {
		/* center: x: back: 55, menu: 172, home: 298, search 412, y: 835 */
		return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":55:835:90:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":172:835:125:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":298:835:115:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":412:835:95:55"
		   "\n");
	} else {
		/* center: x: home: 55, menu: 185, back: 305, search 425, y: 835 */
		return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":55:835:70:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":185:835:100:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":305:835:70:55"
		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":425:835:70:55"
		   "\n");
	}
}

static struct kobj_attribute mahimahi_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mahimahi_virtual_keys_show,
};

static struct attribute *mahimahi_properties_attrs[] = {
	&mahimahi_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group mahimahi_properties_attr_group = {
	.attrs = mahimahi_properties_attrs,
};

static void mahimahi_reset(void)
{
	gpio_set_value(MAHIMAHI_GPIO_PS_HOLD, 0);
}

int mahimahi_init_mmc(int sysrev, unsigned debug_uart);

static const struct smd_tty_channel_desc smd_cdma_default_channels[] = {
	{ .id = 0, .name = "SMD_DS" },
	{ .id = 19, .name = "SMD_DATA3" },
	{ .id = 27, .name = "SMD_GPSNMEA" }
};

static void __init mahimahi_init(void)
{
	int ret;
	struct kobject *properties_kobj;

	printk("mahimahi_init() revision=%d\n", system_rev);

	if (is_cdma_version(system_rev))
		smd_set_channel_list(smd_cdma_default_channels,
					ARRAY_SIZE(smd_cdma_default_channels));

	msm_hw_reset_hook = mahimahi_reset;

	if (is_cdma_version(system_rev))
		msm_acpu_clock_init(&mahimahi_cdma_clock_data);
	else
		msm_acpu_clock_init(&mahimahi_clock_data);

	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      &msm_device_uart1.dev, 1, MSM_GPIO_TO_INT(139));

	config_gpio_table(misc_gpio_table, ARRAY_SIZE(misc_gpio_table));

	if (is_cdma_version(system_rev)) {
		bcm_bt_lpm_pdata.gpio_wake = MAHIMAHI_CDMA_GPIO_BT_WAKE;
		mahimahi_flashlight_data.torch = MAHIMAHI_CDMA_GPIO_FLASHLIGHT_TORCH;
		config_gpio_table(bt_gpio_table_rev_CX, ARRAY_SIZE(bt_gpio_table_rev_CX));
	} else {
		config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));
	}

	gpio_request(MAHIMAHI_GPIO_TP_LS_EN, "tp_ls_en");
	gpio_direction_output(MAHIMAHI_GPIO_TP_LS_EN, 0);
	gpio_request(MAHIMAHI_GPIO_TP_EN, "tp_en");
	gpio_direction_output(MAHIMAHI_GPIO_TP_EN, 0);
	gpio_request(MAHIMAHI_GPIO_PROXIMITY_EN, "proximity_en");
	gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 1);
	gpio_request(MAHIMAHI_GPIO_COMPASS_RST_N, "compass_rst");
	gpio_direction_output(MAHIMAHI_GPIO_COMPASS_RST_N, 1);
	gpio_request(MAHIMAHI_GPIO_COMPASS_INT_N, "compass_int");
	gpio_direction_input(MAHIMAHI_GPIO_COMPASS_INT_N);

	gpio_request(MAHIMAHI_GPIO_DS2482_SLP_N, "ds2482_slp_n");

	/* set the gpu power rail to manual mode so clk en/dis will not
	 * turn off gpu power, and hang it on resume */
	mahimahi_kgsl_power_rail_mode(0);
	mahimahi_kgsl_power(true);

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;

	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(0, base_i2c_devices,
		ARRAY_SIZE(base_i2c_devices));

	if (system_rev == 0) {
		/* Only board after XB with Audience A1026 */
		i2c_register_board_info(0, rev0_i2c_devices,
			ARRAY_SIZE(rev0_i2c_devices));
	}

	if (system_rev > 0) {
		/* Only board after XB with Audience A1026 */
		i2c_register_board_info(0, rev1_i2c_devices,
			ARRAY_SIZE(rev1_i2c_devices));
	}

	if (is_cdma_version(system_rev)) {
		/* Only CDMA version with TI TPA2018D1 Speaker Amp. */
		i2c_register_board_info(0, rev_CX_i2c_devices,
			ARRAY_SIZE(rev_CX_i2c_devices));
		if ((system_rev & 0x0F) == 0x00) {
			a1026_data.gpio_a1026_clk = MAHIMAHI_CDMA_XA_AUD_A1026_CLK;
		} else if ((system_rev & 0x0F) >= 0x01) {
			a1026_data.gpio_a1026_wakeup = MAHIMAHI_CDMA_XB_AUD_A1026_WAKEUP;
			a1026_data.gpio_a1026_reset = MAHIMAHI_CDMA_XB_AUD_A1026_RESET;
			a1026_data.gpio_a1026_clk = MAHIMAHI_CDMA_XB_AUD_A1026_CLK;
		}
	}

	ret = mahimahi_init_mmc(system_rev, debug_uart);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					 &mahimahi_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");

	mahimahi_audio_init();
	mahimahi_headset_init();

	if (system_rev > 0)
		platform_device_register(&mahimahi_timed_gpios);
	else
		msm_init_pmic_vibrator();

	ds2784_battery_init();
}

static void __init mahimahi_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = (219*1024*1024);
	mi->bank[1].start = MSM_HIGHMEM_BASE;
	mi->bank[1].node = PHYS_TO_NID(MSM_HIGHMEM_BASE);
	mi->bank[1].size = MSM_HIGHMEM_SIZE;
}

static void __init mahimahi_map_io(void)
{
	msm_map_qsd8x50_io();
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
}

extern struct sys_timer msm_timer;

MACHINE_START(MAHIMAHI, "mahimahi")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x20000100,
	.fixup		= mahimahi_fixup,
	.map_io		= mahimahi_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= mahimahi_init,
	.timer		= &msm_timer,
MACHINE_END
