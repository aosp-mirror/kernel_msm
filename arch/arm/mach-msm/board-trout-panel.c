/* linux/arch/arm/mach-msm/board-trout-mddi.c
** Author: Brian Swetland <swetland@google.com>
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>

#include "board-trout.h"
#include "proc_comm.h"
#include "devices.h"

#define TROUT_DEFAULT_BACKLIGHT_BRIGHTNESS 255
#define VSYNC_GPIO 97

static struct clk *gp_clk;
static int trout_backlight_off;
static int trout_backlight_brightness = TROUT_DEFAULT_BACKLIGHT_BRIGHTNESS;
static int trout_new_backlight = 1;
static uint8_t trout_backlight_last_level = 33;
static DEFINE_MUTEX(trout_backlight_lock);

static void trout_set_backlight_level(uint8_t level)
{
	unsigned percent = ((int)level * 100) / 255;

	if (trout_new_backlight) {
		unsigned long flags;
		int i = 0;
		level = (int)level * 34 / 256;

		if (trout_backlight_last_level == level)
			return;

		if (level == 0) {
			gpio_set_value(27, 0);
			msleep(2);
		} else {
			local_irq_save(flags);
			if (trout_backlight_last_level == 0) {
				gpio_set_value(27, 1);
				udelay(40);
				trout_backlight_last_level = 33;
			}
			i = (trout_backlight_last_level - level + 33) % 33;
			while (i-- > 0) {
				gpio_set_value(27, 0);
				udelay(1);
				gpio_set_value(27, 1);
				udelay(1);
			}
			local_irq_restore(flags);
		}
		trout_backlight_last_level = level;
	}
	else {
		if(level) {
			clk_enable(gp_clk);
			writel((1U << 16) | (~level & 0xffff),
			       MSM_CLK_CTL_BASE + 0x58);
			/* Going directly to a 100% duty cycle does not
			 *  seem to work */
			if(level == 255) {
				writel((~127 << 16) | 0xb20,
				       MSM_CLK_CTL_BASE + 0x5c);
				udelay(1);
			}
			writel((~127 << 16) | 0xb58, MSM_CLK_CTL_BASE + 0x5c);
		}
		else {
			writel(0x0, MSM_CLK_CTL_BASE + 0x5c);
			clk_disable(gp_clk);
		}
	}
	htc_pwrsink_set(PWRSINK_BACKLIGHT, percent);
}

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define SPI_BLOCK_BASE         0x120000
#define I2C_BLOCK_BASE         0x130000
#define PWM_BLOCK_BASE         0x140000
#define GPIO_BLOCK_BASE        0x150000
#define SYSTEM_BLOCK1_BASE     0x160000
#define SYSTEM_BLOCK2_BASE     0x170000


#define	DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define	SYSCLKENA   (MDDI_CLIENT_CORE_BASE|0x2C)
#define	PWM0OFF	      (PWM_BLOCK_BASE|0x1C)

#define V_VDDE2E_VDD2_GPIO 0
#define MDDI_RST_N 82

#define	MDDICAP0    (MDDI_CLIENT_CORE_BASE|0x00)
#define	MDDICAP1    (MDDI_CLIENT_CORE_BASE|0x04)
#define	MDDICAP2    (MDDI_CLIENT_CORE_BASE|0x08)
#define	MDDICAP3    (MDDI_CLIENT_CORE_BASE|0x0C)
#define	MDCAPCHG    (MDDI_CLIENT_CORE_BASE|0x10)
#define	MDCRCERC    (MDDI_CLIENT_CORE_BASE|0x14)
#define	TTBUSSEL    (MDDI_CLIENT_CORE_BASE|0x18)
#define	DPSET0      (MDDI_CLIENT_CORE_BASE|0x1C)
#define	DPSET1      (MDDI_CLIENT_CORE_BASE|0x20)
#define	DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define	DPRUN       (MDDI_CLIENT_CORE_BASE|0x28)
#define	SYSCKENA    (MDDI_CLIENT_CORE_BASE|0x2C)
#define	TESTMODE    (MDDI_CLIENT_CORE_BASE|0x30)
#define	FIFOMONI    (MDDI_CLIENT_CORE_BASE|0x34)
#define	INTMONI     (MDDI_CLIENT_CORE_BASE|0x38)
#define	MDIOBIST    (MDDI_CLIENT_CORE_BASE|0x3C)
#define	MDIOPSET    (MDDI_CLIENT_CORE_BASE|0x40)
#define	BITMAP0     (MDDI_CLIENT_CORE_BASE|0x44)
#define	BITMAP1     (MDDI_CLIENT_CORE_BASE|0x48)
#define	BITMAP2     (MDDI_CLIENT_CORE_BASE|0x4C)
#define	BITMAP3     (MDDI_CLIENT_CORE_BASE|0x50)
#define	BITMAP4     (MDDI_CLIENT_CORE_BASE|0x54)

#define	SRST        (LCD_CONTROL_BLOCK_BASE|0x00)
#define	PORT_ENB    (LCD_CONTROL_BLOCK_BASE|0x04)
#define	START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define	PORT        (LCD_CONTROL_BLOCK_BASE|0x0C)
#define	CMN         (LCD_CONTROL_BLOCK_BASE|0x10)
#define	GAMMA       (LCD_CONTROL_BLOCK_BASE|0x14)
#define	INTFLG      (LCD_CONTROL_BLOCK_BASE|0x18)
#define	INTMSK      (LCD_CONTROL_BLOCK_BASE|0x1C)
#define	MPLFBUF     (LCD_CONTROL_BLOCK_BASE|0x20)
#define	HDE_LEFT    (LCD_CONTROL_BLOCK_BASE|0x24)
#define	VDE_TOP     (LCD_CONTROL_BLOCK_BASE|0x28)
#define	PXL         (LCD_CONTROL_BLOCK_BASE|0x30)
#define	HCYCLE      (LCD_CONTROL_BLOCK_BASE|0x34)
#define	HSW         (LCD_CONTROL_BLOCK_BASE|0x38)
#define	HDE_START   (LCD_CONTROL_BLOCK_BASE|0x3C)
#define	HDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x40)
#define	VCYCLE      (LCD_CONTROL_BLOCK_BASE|0x44)
#define	VSW         (LCD_CONTROL_BLOCK_BASE|0x48)
#define	VDE_START   (LCD_CONTROL_BLOCK_BASE|0x4C)
#define	VDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x50)
#define	WAKEUP      (LCD_CONTROL_BLOCK_BASE|0x54)
#define	WSYN_DLY    (LCD_CONTROL_BLOCK_BASE|0x58)
#define	REGENB      (LCD_CONTROL_BLOCK_BASE|0x5C)
#define	VSYNIF      (LCD_CONTROL_BLOCK_BASE|0x60)
#define	WRSTB       (LCD_CONTROL_BLOCK_BASE|0x64)
#define	RDSTB       (LCD_CONTROL_BLOCK_BASE|0x68)
#define	ASY_DATA    (LCD_CONTROL_BLOCK_BASE|0x6C)
#define	ASY_DATB    (LCD_CONTROL_BLOCK_BASE|0x70)
#define	ASY_DATC    (LCD_CONTROL_BLOCK_BASE|0x74)
#define	ASY_DATD    (LCD_CONTROL_BLOCK_BASE|0x78)
#define	ASY_DATE    (LCD_CONTROL_BLOCK_BASE|0x7C)
#define	ASY_DATF    (LCD_CONTROL_BLOCK_BASE|0x80)
#define	ASY_DATG    (LCD_CONTROL_BLOCK_BASE|0x84)
#define	ASY_DATH    (LCD_CONTROL_BLOCK_BASE|0x88)
#define	ASY_CMDSET  (LCD_CONTROL_BLOCK_BASE|0x8C)

#define	SSICTL      (SPI_BLOCK_BASE|0x00)
#define	SSITIME     (SPI_BLOCK_BASE|0x04)
#define	SSITX       (SPI_BLOCK_BASE|0x08)
#define	SSIRX       (SPI_BLOCK_BASE|0x0C)
#define	SSIINTC     (SPI_BLOCK_BASE|0x10)
#define	SSIINTS     (SPI_BLOCK_BASE|0x14)
#define	SSIDBG1     (SPI_BLOCK_BASE|0x18)
#define	SSIDBG2     (SPI_BLOCK_BASE|0x1C)
#define	SSIID       (SPI_BLOCK_BASE|0x20)

#define	WKREQ       (SYSTEM_BLOCK1_BASE|0x00)
#define	CLKENB      (SYSTEM_BLOCK1_BASE|0x04)
#define	DRAMPWR     (SYSTEM_BLOCK1_BASE|0x08)
#define	INTMASK     (SYSTEM_BLOCK1_BASE|0x0C)
#define	GPIOSEL     (SYSTEM_BLOCK2_BASE|0x00)

#define	GPIODATA    (GPIO_BLOCK_BASE|0x00)
#define	GPIODIR     (GPIO_BLOCK_BASE|0x04)
#define	GPIOIS      (GPIO_BLOCK_BASE|0x08)
#define	GPIOIBE     (GPIO_BLOCK_BASE|0x0C)
#define	GPIOIEV     (GPIO_BLOCK_BASE|0x10)
#define	GPIOIE      (GPIO_BLOCK_BASE|0x14)
#define	GPIORIS     (GPIO_BLOCK_BASE|0x18)
#define	GPIOMIS     (GPIO_BLOCK_BASE|0x1C)
#define	GPIOIC      (GPIO_BLOCK_BASE|0x20)
#define	GPIOOMS     (GPIO_BLOCK_BASE|0x24)
#define	GPIOPC      (GPIO_BLOCK_BASE|0x28)
#define	GPIOID      (GPIO_BLOCK_BASE|0x30)

#define SPI_WRITE(reg, val) \
	{ SSITX,        0x00010000 | (((reg) & 0xff) << 8) | ((val) & 0xff) }, \
	{ 0, 5 },

#define SPI_WRITE1(reg) \
	{ SSITX,        (reg) & 0xff }, \
	{ 0, 5 },

struct mddi_table {
	uint32_t reg;
	uint32_t value;
};
static struct mddi_table mddi_toshiba_init_table[] = {
	{ DPSET0,       0x09e90046 },
	{ DPSET1,       0x00000118 },
	{ DPSUS,        0x00000000 },
	{ DPRUN,        0x00000001 },
	{ 1,            14         }, /* msleep 14 */
	{ SYSCKENA,     0x00000001 },
	//{ CLKENB,       0x000000EF },
	{ CLKENB,       0x0000A1EF },  /*    # SYS.CLKENB  # Enable clocks for each module (without DCLK , i2cCLK) */
	//{ CLKENB,       0x000025CB }, /* Clock enable register */

	{ GPIODATA,     0x02000200 },  /*   # GPI .GPIODATA  # GPIO2(RESET_LCD_N) set to 0 , GPIO3(eDRAM_Power) set to 0 */
	{ GPIODIR,      0x000030D  },  /* 24D   # GPI .GPIODIR  # Select direction of GPIO port (0,2,3,6,9 output) */
	{ GPIOSEL,      0/*0x00000173*/},  /*   # SYS.GPIOSEL  # GPIO port multiplexing control */
	{ GPIOPC,       0x03C300C0 },  /*   # GPI .GPIOPC  # GPIO2,3 PD cut */
	{ WKREQ,        0x00000000 },  /*   # SYS.WKREQ  # Wake-up request event is VSYNC alignment */

	{ GPIOIBE,      0x000003FF },
	{ GPIOIS,       0x00000000 },
	{ GPIOIC,       0x000003FF },
	{ GPIOIE,       0x00000000 },

	{ GPIODATA,     0x00040004 },  /*   # GPI .GPIODATA  # eDRAM VD supply */
	{ 1,            1          }, /* msleep 1 */
	{ GPIODATA,     0x02040004 },  /*   # GPI .GPIODATA  # eDRAM VD supply */
	{ DRAMPWR,      0x00000001 }, /* eDRAM power */
};

static struct mddi_table mddi_toshiba_panel_init_table[] = {
	{ SRST,         0x00000003 }, /* FIFO/LCDC not reset */
	{ PORT_ENB,     0x00000001 }, /* Enable sync. Port */
	{ START,        0x00000000 }, /* To stop operation */
	//{ START,        0x00000001 }, /* To start operation */
	{ PORT,         0x00000004 }, /* Polarity of VS/HS/DE. */
	{ CMN,          0x00000000 },
	{ GAMMA,        0x00000000 }, /* No Gamma correction */
	{ INTFLG,       0x00000000 }, /* VSYNC interrupt flag clear/status */
	{ INTMSK,       0x00000000 }, /* VSYNC interrupt mask is off. */
	{ MPLFBUF,      0x00000000 }, /* Select frame buffer's base address. */
	{ HDE_LEFT,     0x00000000 }, /* The value of HDE_LEFT. */
	{ VDE_TOP,      0x00000000 }, /* The value of VDE_TPO. */
	{ PXL,          0x00000001 }, /* 1. RGB666 */
	                              /* 2. Data is valid from 1st frame of beginning. */
	{ HDE_START,    0x00000006 }, /* HDE_START= 14 PCLK */
	{ HDE_SIZE,     0x0000009F }, /* HDE_SIZE=320 PCLK */
	{ HSW,          0x00000004 }, /* HSW= 10 PCLK */
	{ VSW,          0x00000001 }, /* VSW=2 HCYCLE */
	{ VDE_START,    0x00000003 }, /* VDE_START=4 HCYCLE */
	{ VDE_SIZE,     0x000001DF }, /* VDE_SIZE=480 HCYCLE */
	{ WAKEUP,       0x000001e2 }, /* Wakeup position in VSYNC mode. */
	{ WSYN_DLY,     0x00000000 }, /* Wakeup position in VSIN mode. */
	{ REGENB,       0x00000001 }, /* Set 1 to enable to change the value of registers. */
	{ CLKENB,       0x000025CB }, /* Clock enable register */

	{ SSICTL,       0x00000170 }, /* SSI control register */
	{ SSITIME,      0x00000250 }, /* SSI timing control register */
	{ SSICTL,       0x00000172 }, /* SSI control register */
};


static struct mddi_table mddi_sharp_init_table[] = {
	{ VCYCLE,       0x000001eb },
	{ HCYCLE,       0x000000ae },
	{ REGENB,       0x00000001 }, /* Set 1 to enable to change the value of registers. */
	{ GPIODATA,     0x00040000 }, /* GPIO2 low */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ 1,            1          }, /* msleep 1 */
	{ GPIODATA,     0x00040004 }, /* GPIO2 high */
	{ 1,            10         }, /* msleep 10 */
	SPI_WRITE(0x5f, 0x01)
	SPI_WRITE1(0x11)
	{ 1,            200        }, /* msleep 200 */
	SPI_WRITE1(0x29)
	SPI_WRITE1(0xde)
	{ START,        0x00000001 }, /* To start operation */
};

static struct mddi_table mddi_sharp_deinit_table[] = {
	{ 1,            200        }, /* msleep 200 */
	SPI_WRITE(0x10, 0x1)
	{ 1,            100        }, /* msleep 100 */
	{ GPIODATA,     0x00040004 }, /* GPIO2 high */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ GPIODATA,     0x00040000 }, /* GPIO2 low */
	{ 1,            10         }, /* msleep 10 */
};

static struct mddi_table mddi_tpo_init_table[] = {
	{ VCYCLE,       0x000001e5 },
	{ HCYCLE,       0x000000ac },
	{ REGENB,       0x00000001 }, /* Set 1 to enable to change the value of registers. */
	{ 0,            20         }, /* udelay 20 */
	{ GPIODATA,     0x00000004 }, /* GPIO2 high */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ 0,            20         }, /* udelay 20 */

	SPI_WRITE(0x08, 0x01)
	{ 0,            500        }, /* udelay 500 */
	SPI_WRITE(0x08, 0x00)
	SPI_WRITE(0x02, 0x00)
	SPI_WRITE(0x03, 0x04)
	SPI_WRITE(0x04, 0x0e)
	SPI_WRITE(0x09, 0x02)
	SPI_WRITE(0x0b, 0x08)
	SPI_WRITE(0x0c, 0x53)
	SPI_WRITE(0x0d, 0x01)
	SPI_WRITE(0x0e, 0xe0)
	SPI_WRITE(0x0f, 0x01)
	SPI_WRITE(0x10, 0x58)
	SPI_WRITE(0x20, 0x1e)
	SPI_WRITE(0x21, 0x0a)
	SPI_WRITE(0x22, 0x0a)
	SPI_WRITE(0x23, 0x1e)
	SPI_WRITE(0x25, 0x32)
	SPI_WRITE(0x26, 0x00)
	SPI_WRITE(0x27, 0xac)
	SPI_WRITE(0x29, 0x06)
	SPI_WRITE(0x2a, 0xa4)
	SPI_WRITE(0x2b, 0x45)
	SPI_WRITE(0x2c, 0x45)
	SPI_WRITE(0x2d, 0x15)
	SPI_WRITE(0x2e, 0x5a)
	SPI_WRITE(0x2f, 0xff)
	SPI_WRITE(0x30, 0x6b)
	SPI_WRITE(0x31, 0x0d)
	SPI_WRITE(0x32, 0x48)
	SPI_WRITE(0x33, 0x82)
	SPI_WRITE(0x34, 0xbd)
	SPI_WRITE(0x35, 0xe7)
	SPI_WRITE(0x36, 0x18)
	SPI_WRITE(0x37, 0x94)
	SPI_WRITE(0x38, 0x01)
	SPI_WRITE(0x39, 0x5d)
	SPI_WRITE(0x3a, 0xae)
	SPI_WRITE(0x3b, 0xff)
	SPI_WRITE(0x07, 0x09)
	{ 0,            10         }, /* udelay 10 */
	{ START,        0x00000001 }, /* To start operation */
};

static struct mddi_table mddi_tpo_deinit_table[] = {
	SPI_WRITE(0x07, 0x19)
	{ START,        0x00000000 }, /* To stop operation */
	{ GPIODATA,     0x00040004 }, /* GPIO2 high */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ GPIODATA,     0x00040000 }, /* GPIO2 low */
	{ 0,            5        }, /* usleep 5 */
};


#define GPIOSEL_VWAKEINT (1U << 0)
#define INTMASK_VWAKEOUT (1U << 0)

static void trout_process_mddi_table(struct msm_mddi_client_data *client_data,
				     struct mddi_table *table, size_t count)
{
	int i;
	for(i = 0; i < count; i++) {
		uint32_t reg = table[i].reg;
		uint32_t value = table[i].value;

		if (reg == 0)
			udelay(value);
		else if (reg == 1)
			msleep(value);
		else
			client_data->remote_write(client_data, value, reg);
	}
}

static struct vreg *vreg_mddi_1v5;
static struct vreg *vreg_lcm_2v85;

static void trout_mddi_power_client(struct msm_mddi_client_data *client_data,
				    int on)
{
    unsigned id, on_off;
	if(on) {
		on_off = 0;
		id = PM_VREG_PDOWN_MDDI_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_mddi_1v5);
		mdelay(5); // delay time >5ms and <10ms
		gpio_set_value(V_VDDE2E_VDD2_GPIO, 1);
		gpio_set_value(TROUT_GPIO_MDDI_32K_EN, 1);
		msleep(3);
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);
		msleep(3);
		gpio_set_value(MDDI_RST_N, 1);
		msleep(10);
	} else {
		gpio_set_value(TROUT_GPIO_MDDI_32K_EN, 0);
		gpio_set_value(MDDI_RST_N, 0);
		msleep(10);
		vreg_disable(vreg_lcm_2v85);
		on_off = 1;
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		msleep(5);
		gpio_set_value(V_VDDE2E_VDD2_GPIO, 0);
		msleep(200);
		vreg_disable(vreg_mddi_1v5);
		id = PM_VREG_PDOWN_MDDI_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
	}
}

static int trout_mddi_toshiba_client_init(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int panel_id;

	client_data->auto_hibernate(client_data, 0);
	trout_process_mddi_table(client_data, mddi_toshiba_init_table,
				 ARRAY_SIZE(mddi_toshiba_init_table));
	client_data->auto_hibernate(client_data, 1);
	panel_id = (client_data->remote_read(client_data, GPIODATA) >> 4) & 3;
	if (panel_id > 1) {
		printk("unknown panel id at mddi_enable\n");
		return -1;
	}
	return 0;
}

static int trout_mddi_toshiba_client_uninit(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	return 0;
}

static int trout_mddi_panel_unblank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{

	int panel_id, ret = 0;
	
	trout_set_backlight_level(0);
	client_data->auto_hibernate(client_data, 0);
	trout_process_mddi_table(client_data, mddi_toshiba_panel_init_table,
		ARRAY_SIZE(mddi_toshiba_panel_init_table));
	panel_id = (client_data->remote_read(client_data, GPIODATA) >> 4) & 3;
	switch(panel_id) {
	 case 0:
		printk("init sharp panel\n");
		trout_process_mddi_table(client_data,
					 mddi_sharp_init_table,
					 ARRAY_SIZE(mddi_sharp_init_table));
		break;
	case 1:
		printk("init tpo panel\n");
		trout_process_mddi_table(client_data,
					 mddi_tpo_init_table,
					 ARRAY_SIZE(mddi_tpo_init_table));
		break;
	default:
		printk("unknown panel_id: %d\n", panel_id);
		ret = -1;
	};
	mutex_lock(&trout_backlight_lock);
	trout_set_backlight_level(trout_backlight_brightness);
	trout_backlight_off = 0;
	mutex_unlock(&trout_backlight_lock);
	client_data->auto_hibernate(client_data, 1);
	client_data->remote_write(client_data, GPIOSEL_VWAKEINT, GPIOSEL);
	client_data->remote_write(client_data, INTMASK_VWAKEOUT, INTMASK);
	return ret;

}

static int trout_mddi_panel_blank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int panel_id, ret = 0;

	panel_id = (client_data->remote_read(client_data, GPIODATA) >> 4) & 3;
	client_data->auto_hibernate(client_data, 0);
	switch(panel_id) {
	case 0:
		printk("deinit sharp panel\n");
		trout_process_mddi_table(client_data,
					 mddi_sharp_deinit_table,
					 ARRAY_SIZE(mddi_sharp_deinit_table));
		break;
	case 1:
		printk("deinit tpo panel\n");
		trout_process_mddi_table(client_data,
					 mddi_tpo_deinit_table,
					 ARRAY_SIZE(mddi_tpo_deinit_table));
		break;
	default:
		printk("unknown panel_id: %d\n", panel_id);
		ret = -1;
	};
	client_data->auto_hibernate(client_data, 1);
	mutex_lock(&trout_backlight_lock);
	trout_set_backlight_level(0);
	trout_backlight_off = 1;
	mutex_unlock(&trout_backlight_lock);
	client_data->remote_write(client_data, 0, SYSCLKENA);
	client_data->remote_write(client_data, 1, DPSUS);
	return ret;
}

static void trout_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	mutex_lock(&trout_backlight_lock);
	trout_backlight_brightness = value;
	if(!trout_backlight_off)
		trout_set_backlight_level(trout_backlight_brightness);
	mutex_unlock(&trout_backlight_lock);
}

static struct led_classdev trout_backlight_led = {
	.name			= "lcd-backlight",
	.brightness = TROUT_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = trout_brightness_set,
};

static int trout_backlight_probe(struct platform_device *pdev)
{
	led_classdev_register(&pdev->dev, &trout_backlight_led);
	return 0;
}

static int trout_backlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&trout_backlight_led);
	return 0;
}

static struct platform_driver trout_backlight_driver = {
	.probe		= trout_backlight_probe,
	.remove		= trout_backlight_remove,
	.driver		= {
		.name		= "trout-backlight",
		.owner		= THIS_MODULE,
	},
};

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE,
		.flags = IORESOURCE_MEM,
	},
};

struct msm_mddi_bridge_platform_data toshiba_client_data = {
	.init = trout_mddi_toshiba_client_init,
	.uninit = trout_mddi_toshiba_client_uninit,
	.blank = trout_mddi_panel_blank,
	.unblank = trout_mddi_panel_unblank,
	.fb_data = {
		.xres = 320,
		.yres = 480,
		.width = 45,
		.height = 67,
		.output_format = 0,
	},
};

static struct msm_mddi_platform_data mddi_pdata = {
	.clk_rate = 122880000,
	.power_client = trout_mddi_power_client,
	.vsync_irq = MSM_GPIO_TO_INT(VSYNC_GPIO),
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0xd263 << 16 | 0),
			.name = "mddi_c_d263_0000",
			//.name = "mddi_c_dummy",
			.id = 0,
			.client_data = &toshiba_client_data,
			//.client_data = &toshiba_client_data.fb_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_device trout_backlight = {
	.name = "trout-backlight",
};

int __init trout_init_panel(void)
{
	int rc;

        if (!machine_is_trout())
                return 0;
	vreg_mddi_1v5 = vreg_get(0, "gp2");
	if (IS_ERR(vreg_mddi_1v5))
		return PTR_ERR(vreg_mddi_1v5);
	vreg_lcm_2v85 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	trout_new_backlight = system_rev >= 5;
	if (trout_new_backlight) {
		uint32_t config = PCOM_GPIO_CFG(27, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	}
	else {
		uint32_t config = PCOM_GPIO_CFG(27, 1, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

		gp_clk = clk_get(NULL, "gp_clk");
		if (IS_ERR(gp_clk)) {
			printk(KERN_ERR "trout_init_panel: could not get gp"
			       "clock\n");
			gp_clk = NULL;
		}
		rc = clk_set_rate(gp_clk, 19200000);
		if (rc)
			printk(KERN_ERR "trout_init_panel: set clock rate "
			       "failed\n");
	}

	rc = gpio_request(VSYNC_GPIO, "vsync");
	if (rc)
		return rc;
	rc = gpio_direction_input(VSYNC_GPIO);
	if (rc)
		return rc;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;
	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;
	platform_device_register(&trout_backlight);
	return platform_driver_register(&trout_backlight_driver);
}

device_initcall(trout_init_panel);
