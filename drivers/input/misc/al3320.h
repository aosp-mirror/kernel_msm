/*
 * This file is part of the AL3320 sensor driver.
 * AL3320 is a ambient light sensor.
 *
 * Contact: YC Hou <yc.hou@liteonsemi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: al3320.h
 *
 * Summary:
 *      AL3320 sensor dirver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 12/26/12 YC       Original Creation (Test version:1.0)
 * 01/07/13 YC       Add a-dummy and the recommand settings in intial fuction.
 * 05/08/13 YC       Update the range according to datasheet rev 1.17.
 *                   Change to v1.02.
 * 05/20/13 YC       1. Move up timer initial function to avoid fatal error.
 *                   2. Correct the polling condition in initial.
 *                   3. Move up reset action to fix the always reset error.
 *                   Change to v1.03.
 * 06/06/13 YC       Add functions for set delay of HAL.
 * 31/07/15 Jennifer Delete Ext_Gain.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/ktime.h>
#include <asm/div64.h>
#include <linux/wakelock.h>

#define AL3320_DRV_NAME		"al3320"
#define DRIVER_VERSION		"1.04"

#define AL3320_NUM_CACHABLE_REGS	16

#define AL3320_MODE_COMMAND	0x00
#define AL3320_MODE_SHIFT	(0)
#define AL3320_MODE_MASK	0x01

#define AL3320_INT_COMMAND	0x01
#define AL3320_INT_SHIFT	(0)
#define AL3320_INT_MASK		0x01

#define AL3320_INT_ENABLE	0x02
#define AL3320_INT_ENABLE_SHIFT	(3)
#define AL3320_INT_ENABLE_MASK	0x08
#define AL3320_SUS_ENABLE_SHIFT	(2)
#define AL3320_SUS_ENABLE_MASK	0x04

#define AL3320_ID		0x04

#define AL3320_WAITING_TIME	0x06
#define AL3320_WAITING_SHIFT	(0)
#define AL3320_WAITING_MASK	0xff

#define AL3320_RAN_COMMAND	0x07
#define AL3320_RAN_MASK		0x06
#define AL3320_RAN_SHIFT	(1)

#define AL3320_ALS_PERSIST		0x08
#define AL3320_PERSIST_SHIFT	(0)
#define AL3320_PERSIST_MASK		0x3f

#define AL3320_ALS_MEANTIME		0x09
#define AL3320_MEANTIME_SHIFT	(0)
#define AL3320_MEANTIME_MASK	0x0f

#define AL3320_ALS_ADUMMY		0x0a
#define AL3320_ADUMMY_SHIFT		(0)
#define AL3320_ADUMMY_MASK		0xff

#define	AL3320_ADC_LSB		0x22
#define	AL3320_ADC_MSB		0x23

#define AL3320_ALS_LTHL			0x30
#define AL3320_ALS_LTHL_SHIFT	(0)
#define AL3320_ALS_LTHL_MASK	0xff

#define AL3320_ALS_LTHH			0x31
#define AL3320_ALS_LTHH_SHIFT	(0)
#define AL3320_ALS_LTHH_MASK	0xff

#define AL3320_ALS_HTHL			0x32
#define AL3320_ALS_HTHL_SHIFT	(0)
#define AL3320_ALS_HTHL_MASK	0xff

#define AL3320_ALS_HTHH			0x33
#define AL3320_ALS_HTHH_SHIFT	(0)
#define AL3320_ALS_HTHH_MASK	0xff

#define ALS_ACTIVE    0x01
#define ALS_DEACTIVE    0x00
#define ALS_RESET    0x04

#define ENABLE	0X01
#define DISABLE	0X00

#define ALS_NO_WAITING    0x00

#define ALS_RAN_0	0x00
#define ALS_RAN_1	0x01
#define ALS_RAN_2	0x02
#define ALS_RAN_3	0x03

/* #define ALS_LOW_GAIN		0x00
 * #define ALS_HIGH_GAIN	0x01 */

#define ALS_ADUMMY_0	(53)
#define ALS_ADUMMY_1	(43)
#define ALS_ADUMMY_2	(3)
#define ALS_ADUMMY_3	(2)

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s, args...) {pr_info("LDBG: func [%s], line [%d], ", __func__, \
	__LINE__); pr_info(s, ## args); }
#else
#define LDBG(s, args...) {}
#endif

#define ADD_TO_IDX(addr, idx) {				\
	int i;						\
	for (i = 0; i < AL3320_NUM_CACHABLE_REGS; i++) {\
		if (addr == al3320_reg[i]) {		\
			idx = i;			\
			break;				\
		}					\
	}						\
	}

struct al3320_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[AL3320_NUM_CACHABLE_REGS];
	int irq;
	struct pinctrl *al3320_pinctrl;

	struct workqueue_struct *wq;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	struct input_dev *light_input_dev;
	struct work_struct work_light;
	struct hrtimer light_timer;
	ktime_t light_poll_delay;
};

/* AL3320 register */
static u8 al3320_reg[AL3320_NUM_CACHABLE_REGS] = {
			0x00, 0x01, 0x02, 0x04, 0x06,
			0x07, 0x08, 0x09, 0x0a, 0x22,
			0x23, 0x30, 0x31, 0x32, 0x33,
			0x34};

/* AL3320 range */
static long al3320_range[4] = {33280, 8320, 2080, 650};

int cali = 100;
u8 suspend_mode;

/* ========================================================================== */
/* User settings. Customer can change the settings here. */

u8 als_polling = 0;			/* Set the polling mode or int mode here. 0 = interrupt mode, 1 = polling mode */
u16 als_poll_delay = 200;		/* Set polling delay (ms) for polling mode */
u8 als_range = ALS_RAN_1;		/* Set range of ALS */
u8 als_meantime = 0x0f;			/* Set meantime of ALS */
u16 als_low_threshold = 0xffff;		/* Set low threshold of ALS */
u16 als_high_threshold = 0x0000;	/* Set high threshold of ALS */
u8 als_persist = 0x1;				/* Set persist of ALS */
u16 als_tolerance = 10;

/* ========================================================================== */

static int al3320_set_adummy(struct i2c_client *client, int adummy);
