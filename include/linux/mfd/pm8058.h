/* include/linux/mfd/pm8058.h
 *
 * Copyright (C) 2010 Google, Inc.
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

#ifndef __LINUX_MFD_PM8058_CORE_H
#define __LINUX_MFD_PM8058_CORE_H

#include <mach/irqs.h>

#define PM8058_NUM_GPIO_IRQS		40
#define PM8058_NUM_MPP_IRQS		12
#define PM8058_NUM_KEYPAD_IRQS		2
#define PM8058_NUM_CHARGER_IRQS		7
#define PM8058_NUM_IRQS			(PM8058_NUM_GPIO_IRQS + \
					 PM8058_NUM_MPP_IRQS + \
					 PM8058_NUM_KEYPAD_IRQS + \
					 PM8058_NUM_CHARGER_IRQS)

/* be careful if you change this since this is used to map irq <-> gpio */
#define PM8058_FIRST_GPIO_IRQ		0
#define PM8058_FIRST_MPP_IRQ		(PM8058_FIRST_GPIO_IRQ + \
					 PM8058_NUM_GPIO_IRQS)
#define PM8058_FIRST_KEYPAD_IRQ		(PM8058_FIRST_MPP_IRQ + \
					 PM8058_NUM_MPP_IRQS)
#define PM8058_FIRST_CHARGER_IRQ	(PM8058_FIRST_KEYPAD_IRQ + \
					 PM8058_NUM_KEYPAD_IRQS)

#define PM8058_KEYPAD_IRQ		(PM8058_FIRST_KEYPAD_IRQ + 0)
#define PM8058_KEYPAD_STUCK_IRQ		(PM8058_FIRST_KEYPAD_IRQ + 1)

#define PM8058_CHGVAL_IRQ		(PM8058_FIRST_CHARGER_IRQ + 0)
#define PM8058_CHGEND_IRQ		(PM8058_FIRST_CHARGER_IRQ + 1)
#define PM8058_FASTCHG_IRQ		(PM8058_FIRST_CHARGER_IRQ + 2)
#define PM8058_CHGFAIL_IRQ		(PM8058_FIRST_CHARGER_IRQ + 5)
#define PM8058_CHGDONE_IRQ		(PM8058_FIRST_CHARGER_IRQ + 6)

#define PM8058_GPIO_TO_IRQ(base,gpio)	(PM8058_FIRST_GPIO_IRQ + \
					 (base) + (gpio))

/* these need to match the irq counts/offsets above above */
#define PM8058_FIRST_GPIO		PM8058_FIRST_GPIO_IRQ
#define PM8058_NUM_GPIOS		PM8058_NUM_GPIO_IRQS
#define PM8058_FIRST_MPP		PM8058_FIRST_MPP_IRQ
#define PM8058_NUM_MPP			PM8058_NUM_MPP_IRQS

#define PM8058_GPIO(base,gpio)		((base) + (gpio) + PM8058_FIRST_GPIO)
/*#define PM8058_MPP(base,mpp)		((base) + (mpp) + PM8058_FIRST_MPP)*/

struct pm8058_keypad_platform_data {
	const char		*name;
	int			num_drv;
	int			num_sns;
	/* delay in ms = 1 << scan_delay_shift, 0-7 */
	int			scan_delay_shift;
	/* # of 32kHz clock cycles, 1-4 */
	int			drv_hold_clks;
	/* in increments of 5ms, max 20ms */
	int			debounce_ms;

	/* size must be num_drv * num_sns
	 * index is (drv * num_sns + sns) */
	const unsigned short	*keymap;

	int			(*init)(struct device *dev);
};

struct pm8058_charger_platform_data {
	/* function to call on vbus detect */
	void			(*vbus_present)(bool present);

	int			(*charge)(u32 max_current, bool is_ac);

	char			**supplied_to;
	int			num_supplicants;
};

struct pm8058_platform_data {
	unsigned int				irq_base;
	unsigned int				gpio_base;
	int					(*init)(struct device *dev);

	/* child devices */
	struct pm8058_keypad_platform_data	*keypad_pdata;
	struct pm8058_charger_platform_data	*charger_pdata;
};

#define PM8058_GPIO_VIN_SRC_VPH_PWR	0x0 /* VDD_L6_L7 */
#define PM8058_GPIO_VIN_SRC_VREG_BB	0x1 /* VDD_L3_L4_L5 */
#define PM8058_GPIO_VIN_SRC_VREG_S3	0x2 /* VDD_L0_L1_LVS, 1.8V */
#define PM8058_GPIO_VIN_SRC_VREG_L3	0x3 /* 1.8V or 2.85 */
#define PM8058_GPIO_VIN_SRC_VREG_L7	0x4 /* 1.8V */
#define PM8058_GPIO_VIN_SRC_VREG_L6	0x5 /* 3.3V */
#define PM8058_GPIO_VIN_SRC_VREG_L5	0x6 /* 2.85V */
#define PM8058_GPIO_VIN_SRC_VREG_L2	0x7 /* 2.6V */

#define PM8058_GPIO_INPUT		0x01
#define PM8058_GPIO_OUTPUT		0x02
#define PM8058_GPIO_OUTPUT_HIGH 	0x04

#define PM8058_GPIO_STRENGTH_OFF	0x0
#define PM8058_GPIO_STRENGTH_HIGH	0x1
#define PM8058_GPIO_STRENGTH_MED	0x2
#define PM8058_GPIO_STRENGTH_LOW	0x3

#define	PM8058_GPIO_PULL_UP_30		0x0
#define	PM8058_GPIO_PULL_UP_1P5		0x1
#define	PM8058_GPIO_PULL_UP_31P5	0x2
#define	PM8058_GPIO_PULL_UP_1P5_30	0x3
#define	PM8058_GPIO_PULL_DOWN		0x4
#define	PM8058_GPIO_PULL_NONE		0x5

#define PM8058_GPIO_FUNC_NORMAL		0x0
#define PM8058_GPIO_FUNC_PAIRED		0x1
#define PM8058_GPIO_FUNC_1		0x2
#define PM8058_GPIO_FUNC_2		0x3

/* gpio pin flags */
#define PM8058_GPIO_OPEN_DRAIN		0x10
#define PM8058_GPIO_HIGH_Z		0x20
#define PM8058_GPIO_INV_IRQ_POL		0x40
#define PM8058_GPIO_CONFIGURED		0x80 /* FOR INTERNAL USE ONLY */

struct pm8058_pin_config {
	u8		vin_src;
	u8		dir;
	u8		pull_up;
	u8		strength;
	u8		func;
	u8		flags;
};

#define PM8058_GPIO_PIN_CONFIG(v,d,p,s,fn,fl) \
	{ \
		.vin_src	= (v), \
		.dir		= (d), \
		.pull_up	= (p), \
		.strength	= (s), \
		.func		= (fn), \
		.flags		= (fl), \
	}

#ifdef CONFIG_PM8058
int pm8058_readb(struct device *dev, u16 addr, u8 *val);
int pm8058_writeb(struct device *dev, u16 addr, u8 val);
int pm8058_write_buf(struct device *dev, u16 addr, u8 *buf, int cnt);
int pm8058_read_buf(struct device *dev, u16 addr, u8 *buf, int cnt);
int pm8058_gpio_mux_cfg(struct device *dev, unsigned int gpio,
			struct pm8058_pin_config *cfg);
int pm8058_gpio_mux(unsigned int gpio, struct pm8058_pin_config *cfg);
int pm8058_irq_get_status(struct device *dev, unsigned int irq);
#else
static inline int pm8058_readb(struct device *dev, u16 addr, u8 *val)
{ return 0; }
static inline int pm8058_writeb(struct device *dev, u16 addr, u8 val)
{ return 0; }
static inline int pm8058_write_buf(struct device *dev, u16 addr, u8 *buf,
				   int cnt) { return 0; }
static inline int pm8058_read_buf(struct device *dev, u16 addr, u8 *buf,
				  int cnt) { return 0; }
static inline int pm8058_gpio_mux_cfg(struct device *dev, unsigned int gpio,
			struct pm8058_pin_config *cfg) { return 0; }
static inline int pm8058_gpio_mux(unsigned int gpio,
			struct pm8058_pin_config *cfg) { return 0; }
static inline int pm8058_irq_get_status(struct device *dev, unsigned int irq)
{ return 0; }
#endif

#ifdef CONFIG_CHARGER_PM8058
void pm8058_notify_charger_connected(int status);
#else
static inline void pm8058_notify_charger_connected(int status) {}
#endif

#endif
