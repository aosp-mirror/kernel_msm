/* arch/arm/mach-msm/board-trout.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/keyreset.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/akm8976.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>

#include <linux/delay.h>

#include <asm/gpio.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <mach/system.h>
#include <mach/vreg.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/msm_audio.h>

#include "board-trout.h"

#include "gpio_chip.h"

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_debugger.h>
#include <mach/msm_serial_hs.h>
#include <mach/htc_pwrsink.h>
#ifdef CONFIG_HTC_HEADSET
#include <mach/htc_headset.h>
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wifi_tiwlan.h>
#endif

#include "proc_comm.h"
#include "devices.h"

void msm_init_irq(void);
void msm_init_gpio(void);

extern int trout_init_mmc(unsigned int);
#ifdef CONFIG_WIFI_CONTROL_FUNC
#ifdef CONFIG_WIFI_MEM_PREALLOC
extern int trout_init_wifi_mem(void);
#endif
extern struct wifi_platform_data trout_wifi_control;
#endif

struct trout_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
};
static bool nav_just_on;
static int nav_on_jiffies;

uint16_t trout_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct trout_axis_info *ai = container_of(info, struct trout_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if((ai->in_state ^ in) & 1)
		out--;
	if((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	return out;
}

int trout_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	gpio_set_value(TROUT_GPIO_JOG_EN, on);
	if (on) {
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	}
	return 0;
}

static uint32_t trout_4_x_axis_gpios[] = {
	TROUT_4_BALL_LEFT_0, TROUT_4_BALL_RIGHT_0
};
static uint32_t trout_5_x_axis_gpios[] = {
	TROUT_5_BALL_LEFT_0, TROUT_5_BALL_RIGHT_0
};

static struct trout_axis_info trout_x_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(trout_5_x_axis_gpios),
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(trout_5_x_axis_gpios),
		.map = trout_axis_map,
		.gpio = trout_5_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */
	}
};

static uint32_t trout_4_y_axis_gpios[] = {
	TROUT_4_BALL_UP_0, TROUT_4_BALL_DOWN_0
};
static uint32_t trout_5_y_axis_gpios[] = {
	TROUT_5_BALL_UP_0, TROUT_5_BALL_DOWN_0
};

static struct trout_axis_info trout_y_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(trout_5_y_axis_gpios),
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(trout_5_y_axis_gpios),
		.map = trout_axis_map,
		.gpio = trout_5_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */
	}
};

static struct gpio_event_direct_entry trout_nav_buttons[] = {
	{ TROUT_GPIO_NAVI_ACT_N, BTN_MOUSE }
};

static struct gpio_event_input_info trout_nav_button_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = trout_nav_buttons,
	.keymap_size = ARRAY_SIZE(trout_nav_buttons)
};

static struct gpio_event_info *trout_nav_info[] = {
	&trout_x_axis.info.info,
	&trout_y_axis.info.info,
	&trout_nav_button_info.info
};

static struct gpio_event_platform_data trout_nav_data = {
	.name = "trout-nav",
	.info = trout_nav_info,
	.info_count = ARRAY_SIZE(trout_nav_info),
	.power = trout_nav_power,
};

static struct platform_device trout_nav_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 2,
	.dev = {
		.platform_data = &trout_nav_data,
	},
};

static int trout_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data trout_reset_keys_pdata = {
	.keys_up = trout_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_MENU,
		KEY_END,
		0
	},
};

struct platform_device trout_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &trout_reset_keys_pdata,
};

static int trout_ts_power(int on)
{
	int tp_ls_gpio = system_rev < 5 ? TROUT_4_TP_LS_EN : TROUT_5_TP_LS_EN;
	if (on) {
		gpio_set_value(TROUT_GPIO_TP_I2C_PULL, 1);
		gpio_set_value(TROUT_GPIO_TP_EN, 1);
		/* touchscreen must be powered before we enable i2c pullup */
		msleep(2);
		/* enable touch panel level shift */
		gpio_set_value(tp_ls_gpio, 1);
		msleep(2);
	}
	else {
		gpio_set_value(tp_ls_gpio, 0);
		udelay(50);
		gpio_set_value(TROUT_GPIO_TP_EN, 0);
		gpio_set_value(TROUT_GPIO_TP_I2C_PULL, 0);
	}
	return 0;
}

static struct synaptics_i2c_rmi_platform_data trout_ts_data[] = {
	{
		.version = 0x010c,
		.power = trout_ts_power,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -100 * 0x10000 / 4334,
		.inactive_right = -100 * 0x10000 / 4334,
		.inactive_top = -40 * 0x10000 / 6696,
		.inactive_bottom = -40 * 0x10000 / 6696,
		.snap_left_on = 300 * 0x10000 / 4334,
		.snap_left_off = 310 * 0x10000 / 4334,
		.snap_right_on = 300 * 0x10000 / 4334,
		.snap_right_off = 310 * 0x10000 / 4334,
		.snap_top_on = 100 * 0x10000 / 6696,
		.snap_top_off = 110 * 0x10000 / 6696,
		.snap_bottom_on = 100 * 0x10000 / 6696,
		.snap_bottom_off = 110 * 0x10000 / 6696,
	},
	{
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_right = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_top = ((6946 - 6696) / 2) * 0x10000 / 6696,
		.inactive_bottom = ((6946 - 6696) / 2) * 0x10000 / 6696,
	}
};

static struct akm8976_platform_data compass_platform_data = {
	.reset = TROUT_GPIO_COMPASS_RST_N,
	.clk_on = TROUT_GPIO_COMPASS_32K_EN,
	.intr = TROUT_GPIO_COMPASS_IRQ,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.platform_data = trout_ts_data,
		.irq = TROUT_GPIO_TO_INT(TROUT_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO("elan-touch", 0x10),
		.irq = TROUT_GPIO_TO_INT(TROUT_GPIO_TP_ATT_N),
	},
	{
		I2C_BOARD_INFO("akm8976", 0x1C),
		.platform_data = &compass_platform_data,
		.irq = TROUT_GPIO_TO_INT(TROUT_GPIO_COMPASS_IRQ),
	},
	{
		I2C_BOARD_INFO("pca963x", 0x62),
	},
#if defined(CONFIG_MSM_CAMERA) && defined(CONFIG_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
#ifdef CONFIG_SENSORS_MT9T013
	{
		I2C_BOARD_INFO("mt9t013", 0x6C >> 1),
	},
#endif
};

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = TROUT_GPIO_HAPTIC_PWM,
		.max_timeout = 15000,
	},
	{
		.name = "flash",
		.gpio = TROUT_GPIO_FLASH_EN,
		.max_timeout = 400,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device android_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

static struct gpio_led android_led_list[] = {
	{
		.name = "spotlight",
		.gpio = TROUT_GPIO_SPOTLIGHT_EN,
	},
	{
		.name = "keyboard-backlight",
		.gpio = TROUT_GPIO_QTKEY_LED_EN,
	},
	{
		.name = "button-backlight",
		.gpio = TROUT_GPIO_UI_LED_EN,
	},
};

static struct gpio_led_platform_data android_leds_data = {
	.num_leds	= ARRAY_SIZE(android_led_list),
	.leds		= android_led_list,
};

static struct platform_device android_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &android_leds_data,
	},
};

static struct gpio_switch_platform_data sd_door_switch_data = {
	.name		= "sd-door",
	.gpio		= TROUT_GPIO_SD_DOOR_N,
	.state_on	= "open",
	.state_off	= "closed",
};

static struct platform_device sd_door_switch = {
	.name		= "switch-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &sd_door_switch_data,
	},
};

#ifdef CONFIG_HTC_HEADSET
static void h2w_config_cpld(int route)
{
	switch (route) {
	case H2W_UART3:
		gpio_set_value(TROUT_GPIO_H2W_SEL0, 0);
		gpio_set_value(TROUT_GPIO_H2W_SEL1, 1);
		break;
	case H2W_GPIO:
		gpio_set_value(TROUT_GPIO_H2W_SEL0, 0);
		gpio_set_value(TROUT_GPIO_H2W_SEL1, 0);
		break;
	}
}

static void h2w_init_cpld(void)
{
	h2w_config_cpld(H2W_UART3);
	gpio_set_value(TROUT_GPIO_H2W_CLK_DIR, 0);
	gpio_set_value(TROUT_GPIO_H2W_DAT_DIR, 0);
}

static struct h2w_platform_data trout_h2w_data = {
	.cable_in1		= TROUT_GPIO_CABLE_IN1,
	.cable_in2		= TROUT_GPIO_CABLE_IN2,
	.h2w_clk		= TROUT_GPIO_H2W_CLK_GPI,
	.h2w_data		= TROUT_GPIO_H2W_DAT_GPI,
	.debug_uart 		= H2W_UART3,
	.config_cpld 		= h2w_config_cpld,
	.init_cpld 		= h2w_init_cpld,
};

static struct platform_device trout_h2w = {
	.name		= "h2w",
	.id		= -1,
	.dev		= {
		.platform_data	= &trout_h2w_data,
	},
};
#endif

static void trout_phy_reset(void)
{
	gpio_set_value(TROUT_GPIO_USB_PHY_RST_N, 0);
	mdelay(10);
	gpio_set_value(TROUT_GPIO_USB_PHY_RST_N, 1);
	mdelay(10);
}

static void config_camera_on_gpios(void);
static void config_camera_off_gpios(void);

#ifdef CONFIG_MSM_CAMERA
static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 108,
	.sensor_pwd     = 85,
	.vcm_pwd        = TROUT_GPIO_VCM_PWDN,
	.pdata          = &msm_camera_device_data,
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name           = "msm_camera_mt9t013",
	.dev            = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif
#endif

#ifdef CONFIG_SENSORS_MT9T013
static struct msm_camera_legacy_device_platform_data msm_camera_device_mt9t013 = {
	.sensor_reset	= 108,
	.sensor_pwd	= 85,
	.vcm_pwd	= TROUT_GPIO_VCM_PWDN,
	.config_gpio_on = config_camera_on_gpios,
	.config_gpio_off = config_camera_off_gpios,
};

static struct platform_device trout_camera = {
	.name           = "camera",
	.dev            = {
		.platform_data = &msm_camera_device_mt9t013,
	},
};
#endif

static struct pwr_sink trout_pwrsink_table[] = {
	{
		.id	= PWRSINK_AUDIO,
		.ua_max	= 90000,
	},
	{
		.id	= PWRSINK_BACKLIGHT,
		.ua_max	= 128000,
	},
	{
		.id	= PWRSINK_LED_BUTTON,
		.ua_max	= 17000,
	},
	{
		.id	= PWRSINK_LED_KEYBOARD,
		.ua_max	= 22000,
	},
	{
		.id	= PWRSINK_GP_CLK,
		.ua_max	= 30000,
	},
	{
		.id	= PWRSINK_BLUETOOTH,
		.ua_max	= 15000,
	},
	{
		.id	= PWRSINK_CAMERA,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_SDCARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_VIDEO,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id	= PWRSINK_SYSTEM_LOAD,
		.ua_max	= 100000,
		.percent_util = 38,
	},
};

static struct pwr_sink_platform_data trout_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(trout_pwrsink_table),
	.sinks		= trout_pwrsink_table,
	.suspend_late	= NULL,
	.resume_early	= NULL,
	.suspend_early	= NULL,
	.resume_late	= NULL,
};

static struct platform_device trout_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev	= {
		.platform_data = &trout_pwrsink_data,
	},
};

static struct platform_device trout_rfkill = {
	.name = "trout_rfkill",
	.id = -1,
};

static struct msm_pmem_setting pmem_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct platform_device trout_wifi = {
	.name		= "msm_wifi",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
	.dev		= {
		.platform_data = &trout_wifi_control,
	},
};
#endif

#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(3, "BT"),
	SND(44, "BT_EC_OFF"),
	SND(10, "HEADSET_AND_SPEAKER"),
	SND(256, "CURRENT"),

	/* Bluetooth accessories. */

	SND(12, "HTC BH S100"),
	SND(13, "HTC BH M100"),
	SND(14, "Motorola H500"),
	SND(15, "Nokia HS-36W"),
	SND(16, "PLT 510v.D"),
	SND(17, "M2500 by Plantronics"),
	SND(18, "Nokia HDW-3"),
	SND(19, "HBH-608"),
	SND(20, "HBH-DS970"),
	SND(21, "i.Tech BlueBAND"),
	SND(22, "Nokia BH-800"),
	SND(23, "Motorola H700"),
	SND(24, "HTC BH M200"),
	SND(25, "Jabra JX10"),
	SND(26, "320Plantronics"),
	SND(27, "640Plantronics"),
	SND(28, "Jabra BT500"),
	SND(29, "Motorola HT820"),
	SND(30, "HBH-IV840"),
	SND(31, "6XXPlantronics"),
	SND(32, "3XXPlantronics"),
	SND(33, "HBH-PV710"),
	SND(34, "Motorola H670"),
	SND(35, "HBM-300"),
	SND(36, "Nokia BH-208"),
	SND(37, "Samsung WEP410"),
	SND(38, "Jabra BT8010"),
	SND(39, "Motorola S9"),
	SND(40, "Jabra BT620s"),
	SND(41, "Nokia BH-902"),
	SND(42, "HBH-DS220"),
	SND(43, "HBH-DS980"),
};
#undef SND

static struct msm_snd_endpoints trout_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device trout_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev	= {
		.platform_data = &trout_snd_endpoints,
	},
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_uart1,
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER) && !defined(CONFIG_TROUT_H2W)
	&msm_device_uart3,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
	&trout_nav_device,
	&trout_reset_keys_device,
	&android_leds,
	&sd_door_switch,
	&android_timed_gpios,
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_SENSORS_MT9T013
	&trout_camera,
#endif
	&trout_rfkill,
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&trout_wifi,
#endif
#ifdef CONFIG_HTC_HEADSET
	&trout_h2w,
#endif
#ifdef CONFIG_HTC_PWRSINK
	&trout_pwr_sink,
#endif
	&trout_snd,
};

extern struct sys_timer msm_timer;

static void __init trout_init_irq(void)
{
	printk("trout_init_irq()\n");
	msm_init_irq();
}

static uint opt_disable_uart3;

module_param_named(disable_uart3, opt_disable_uart3, uint, 0);

static void trout_reset(void)
{
	gpio_set_value(TROUT_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
	/* BLUETOOTH */
#ifdef CONFIG_SERIAL_MSM_HS
	PCOM_GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RTS */
	PCOM_GPIO_CFG(44, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* CTS */
	PCOM_GPIO_CFG(45, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RX */
	PCOM_GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* TX */
#else
	PCOM_GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RTS */
	PCOM_GPIO_CFG(44, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* CTS */
	PCOM_GPIO_CFG(45, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* RX */
	PCOM_GPIO_CFG(46, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* TX */
#endif
};


static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
	config_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data trout_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(45),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

static void __init trout_init(void)
{
	int rc;

	printk("trout_init() revision=%d\n", system_rev);

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	msm_hw_reset_hook = trout_reset;

	gpio_direction_output(system_rev < 5 ?
			TROUT_4_TP_LS_EN : TROUT_5_TP_LS_EN, 0);

	msm_acpu_clock_init(&trout_clock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				      &msm_device_uart3.dev, 1,
				      MSM_GPIO_TO_INT(86));
#endif

	/* gpio_configure(108, IRQF_TRIGGER_LOW); */

	/* put the AF VCM in powerdown mode to avoid noise */
	gpio_set_value(TROUT_GPIO_VCM_PWDN, 1);
	mdelay(100);

	if (system_rev < 5) {
		trout_x_axis.info.gpio = trout_4_x_axis_gpios;
		trout_y_axis.info.gpio = trout_4_y_axis_gpios;
	}

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
	msm_add_usb_devices(trout_phy_reset);

	msm_add_mem_devices(&pmem_setting);

	rc = trout_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

#ifdef CONFIG_WIFI_MEM_PREALLOC
	rc = trout_init_wifi_mem();
	if (rc)
		printk(KERN_CRIT "%s: WiFi Memory init failure (%d)\n", __func__, rc);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	/* SD card door should wake the device */
	set_irq_wake(TROUT_GPIO_TO_INT(TROUT_GPIO_SD_DOOR_N), 1);
}

static struct map_desc trout_io_desc[] __initdata = {
	{
		.virtual = TROUT_CPLD_BASE,
		.pfn     = __phys_to_pfn(TROUT_CPLD_START),
		.length  = TROUT_CPLD_SIZE,
		.type    = MT_DEVICE_NONSHARED
	}
};

static void __init trout_fixup(struct machine_desc *desc, struct tag *tags,
				char **cmdline, struct meminfo *mi)
{
	mi->nr_banks=1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = (101*1024*1024);
}

static void __init trout_map_io(void)
{
	msm_map_common_io();
	iotable_init(trout_io_desc, ARRAY_SIZE(trout_io_desc));
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

MACHINE_START(TROUT, "trout")
/* Maintainer: Brian Swetland <swetland@google.com> */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x10000100,
	.fixup          = trout_fixup,
	.map_io         = trout_map_io,
	.init_irq       = trout_init_irq,
	.init_machine   = trout_init,
	.timer          = &msm_timer,
MACHINE_END
