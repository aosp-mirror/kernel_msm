/* drivers/input/keyboard/pm8058-keypad.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2009 Code Aurora Forum
 *
 * Author: Dima Zavin <dima@android.com>
 *    - Heavily based on the driver from the Code Aurora Forum.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/pm8058.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/wait.h>

enum {
	DEBUG_IRQ = 1U << 0,
	DEBUG_KEYS = 1U << 1,
};
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define REG_KEYP_CTRL		0x148
#define REG_KEYP_SCAN		0x149
#define REG_KEYP_TEST		0x14a
#define REG_KEYP_NEW_DATA	0x14b
#define REG_KEYP_OLD_DATA	0x14c

#define KP_SNS_MIN		5
#define KP_SNS_MAX		8
#define KP_DRV_MIN		5
#define KP_DRV_MAX		18

#define KP_CLOCK_FREQ		32768

#define KPF_HAS_SYNC_READ	0x00000001

struct pm8058_keypad {
	struct device		*dev;
	struct input_dev	*input_dev;
	int			sense_irq;
	int			stuck_irq;

	int			num_sns;
	int			num_drv;
	const unsigned short	*keymap;

	u8			key_state[KP_DRV_MAX];
	u8			stuck_state[KP_DRV_MAX];

	u32			flags;
};

/* convenience wrapers */
static inline int kp_writeb(struct pm8058_keypad *kp, u16 addr, u8 val)
{
	return pm8058_writeb(kp->dev->parent, addr, val);
}

static inline int kp_readb(struct pm8058_keypad *kp, u16 addr, u8 *val)
{
	return pm8058_readb(kp->dev->parent, addr, val);
}

static inline int kp_read_buf(struct pm8058_keypad *kp, u16 addr,
				     u8 *buf, int cnt)
{
	return pm8058_read_buf(kp->dev->parent, addr, buf, cnt);
}

static int kp_hw_init(struct pm8058_keypad *kp,
		      struct pm8058_keypad_platform_data *pdata)
{
	int ret;
	u8 val;
	u8 drv_bits[] = {
		0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7,
	};
	u8 sns_bits[] = {
		0, 0, 0, 0, 0, 0, 1, 2, 3,
	};

	val = ((drv_bits[pdata->num_drv] << 2) |
	       (sns_bits[pdata->num_sns] << 5));
	ret = kp_writeb(kp, REG_KEYP_CTRL, val);
	if (ret) {
		pr_err("%s: can't write kp ctrl\n", __func__);
		goto out;
	}

	val = ((((pdata->drv_hold_clks - 1) & 0x3) << 6) |
	       ((pdata->scan_delay_shift & 0x7) << 3) |
	       ((((pdata->debounce_ms / 5) & 0x3)) << 1));
	ret = kp_writeb(kp, REG_KEYP_SCAN, val);
	if (ret) {
		pr_err("%s: can't write kp scan\n", __func__);
		goto out;
	}

out:
	return ret;
}

static int kp_get_scan_data(struct pm8058_keypad *kp, u8 *old, u8 *new)
{
	int ret;
	u8 val;

	/* XXX: B0 only? */
	if (kp->flags & KPF_HAS_SYNC_READ) {
		ret = kp_readb(kp, REG_KEYP_SCAN, &val);
		if (ret)
			goto err;
		ret = kp_writeb(kp, REG_KEYP_SCAN, val | 1);
		if (ret)
			goto err;
		/* 2 * 32KHz clocks */
		udelay((2 * USEC_PER_SEC / KP_CLOCK_FREQ) + 1);
	}

	if (old) {
		ret = kp_read_buf(kp, REG_KEYP_OLD_DATA, old, kp->num_drv);
		if (ret)
			goto done;
	}

	ret = kp_read_buf(kp, REG_KEYP_NEW_DATA, new, kp->num_drv);
	if (ret)
		goto done;

done:
	if (kp->flags & KPF_HAS_SYNC_READ) {
		/* 4 * 32KHz clocks */
		udelay((4 * USEC_PER_SEC / KP_CLOCK_FREQ) + 1);
		ret = kp_readb(kp, REG_KEYP_SCAN, &val);
		if (ret)
			goto err;
		ret = kp_writeb(kp, REG_KEYP_SCAN, val & (~0x1));
		if (ret)
			goto err;
	}

err:
	if (ret)
		pr_err("%s: can't get scan data\n", __func__);
	return ret;
}

static int kp_process_scan_data(struct pm8058_keypad *kp, u8 *old, u8 *new)
{
	int drv;
	int sns;

	for (drv = 0; drv < kp->num_drv; ++drv) {
		unsigned long bits_changed = (new[drv] ^ old[drv]) & 0xff;

		for_each_set_bit(sns, &bits_changed, kp->num_sns) {
			int key_idx = drv * kp->num_sns + sns;
			unsigned int code = kp->keymap[key_idx] ?: KEY_UNKNOWN;
			int down = !(new[drv] & (1 << sns));

			if (debug_mask & DEBUG_KEYS)
				pr_info("%s: key [%d:%d] %s\n", __func__,
					drv, sns, down ? "down" : "up");
			input_event(kp->input_dev, EV_MSC, MSC_SCAN, key_idx);
			input_report_key(kp->input_dev, code, down);
			input_sync(kp->input_dev);
		}
	}

	return 0;
}

/*
 * NOTE: We are reading recent and old data registers blindly
 * whenever key-stuck interrupt happens, because events counter doesn't
 * get updated when this interrupt happens due to key stuck doesn't get
 * considered as key state change.
 *
 * We are not using old data register contents after they are being read
 * because it might report the key which was pressed before the key being stuck
 * as stuck key because it's pressed status is stored in the old data
 * register.
 */
static irqreturn_t kp_stuck_irq_handler(int irq, void *dev_id)
{
	struct pm8058_keypad *kp = dev_id;
	u8 old[KP_DRV_MAX];
	u8 new[KP_DRV_MAX];
	int ret;

	if (debug_mask & DEBUG_IRQ)
		pr_info("%s: key stuck!\n", __func__);

	ret = kp_get_scan_data(kp, old, new);
	if (ret) {
		pr_err("%s: couldn't get scan data\n", __func__);
		goto out;
	}
	kp_process_scan_data(kp, kp->stuck_state, new);

out:
	return IRQ_HANDLED;
}

static irqreturn_t kp_sense_irq_handler(int irq, void *dev_id)
{
	struct pm8058_keypad *kp = dev_id;
	int ret;
	u8 old[KP_DRV_MAX];
	u8 new[KP_DRV_MAX];
	u8 val;

	if (debug_mask & DEBUG_IRQ)
		pr_info("%s: key event!!\n", __func__);
	ret = kp_readb(kp, REG_KEYP_CTRL, &val);
	if (ret) {
		pr_err("%s: can't read events\n", __func__);
		goto out;
	}

	/* events counter is gray coded */
	switch(val & 0x3) {
	case 0x1:
		ret = kp_get_scan_data(kp, NULL, new);
		if (ret)
			goto out;
		kp_process_scan_data(kp, kp->key_state, new);
		memcpy(kp->key_state, new, sizeof(new));
		break;

	case 0x2:
		pr_debug("%s: some key events were missed\n", __func__);
	case 0x3:
		ret = kp_get_scan_data(kp, old, new);
		if (ret)
			goto out;
		/* first process scan data in relation to last known
		 * key state */
		kp_process_scan_data(kp, kp->key_state, old);
		kp_process_scan_data(kp, old, new);
		memcpy(kp->key_state, new, sizeof(new));
		break;
		
	case 0x0:
		pr_warning("%s: interrupt without any events?!\n", __func__);
		break;
	}

out:
	if (ret)
		pr_err("%s: couldn't get scan data\n", __func__);

	return IRQ_HANDLED;
}

static int pm8058_keypad_probe(struct platform_device *pdev)
{
	struct pm8058_keypad_platform_data *pdata = pdev->dev.platform_data;
	struct pm8058_keypad *kp;
	int sense_irq;
	int stuck_irq;
	int ret;
	int i;
	u8 val;

	sense_irq = platform_get_irq_byname(pdev, "kp_sense");
	stuck_irq = platform_get_irq_byname(pdev, "kp_stuck");

	if (!pdata || sense_irq < 0 || stuck_irq < 0) {
		pr_err("%s: missing platform data/resources\n", __func__);
		return -EINVAL;
	}

	if (pdata->num_sns > KP_SNS_MAX || pdata->num_drv > KP_DRV_MAX ||
	    (pdata->drv_hold_clks == 0) || !pdata->keymap) {
		pr_err("%s: invalid plaform data\n", __func__);
		return -EINVAL;
	}

	kp = kzalloc(sizeof(*kp), GFP_KERNEL);
	if (!kp) {
		pr_err("%s: can't allocate memory for kp struct\n", __func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, kp);
	kp->dev = &pdev->dev;
	kp->num_sns = pdata->num_sns;
	kp->num_drv = pdata->num_drv;
	kp->keymap = pdata->keymap;
	kp->sense_irq = sense_irq;
	kp->stuck_irq = stuck_irq;

	memset(kp->key_state, 0xff, sizeof(kp->key_state));
	memset(kp->stuck_state, 0xff, sizeof(kp->stuck_state));

	/* b0 and up have sync_read support */
	kp->flags = KPF_HAS_SYNC_READ;

	kp->input_dev = input_allocate_device();
	if (!kp->input_dev) {
		ret = -ENOMEM;
		pr_err("%s: Failed to allocate input device\n", __func__);
		goto err_input_dev_alloc;
	}

	kp->input_dev->name = pdata->name;
	input_set_capability(kp->input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(kp->input_dev, kp);

	for (i = 0; i < kp->num_drv * kp->num_sns; ++i) {
		unsigned short keycode = kp->keymap[i];
		BUG_ON(keycode && keycode > KEY_MAX);
		if (keycode)
			input_set_capability(kp->input_dev, EV_KEY, keycode);
	}

	ret = input_register_device(kp->input_dev);
	if (ret) {
		pr_err("%s: can't register input device '%s'\n", __func__,
		       pdata->name);
		goto err_input_dev_reg;
	}

	ret = kp_hw_init(kp, pdata);
	if (ret) {
		pr_err("%s: can't initialize keypad hardware\n", __func__);
		goto err_kp_hw_init;
	}

	if (pdata->init) {
		ret = pdata->init(kp->dev);
		if (ret) {
			pr_err("%s: can't call board's init\n", __func__);
			goto err_pdata_init;
		}
	}

	ret = request_threaded_irq(kp->sense_irq, NULL, kp_sense_irq_handler,
			  IRQF_TRIGGER_RISING, "pm8058-keypad-sense", kp);
	if (ret) {
		pr_err("%s: can't request sense_irq\n", __func__);
		goto err_req_sense_irq;
	}
	ret = request_threaded_irq(kp->stuck_irq, NULL, kp_stuck_irq_handler,
			  IRQF_TRIGGER_RISING, "pm8058-keypad-stuck", kp);
	if (ret) {
		pr_err("%s: can't request stuck\n", __func__);
		goto err_req_stuck_irq;
	}

	enable_irq_wake(kp->sense_irq);

	ret = kp_readb(kp, REG_KEYP_CTRL, &val);
	if (ret) {
		pr_err("%s: can't read kp ctrl\n", __func__);
		goto err_read_kp_ctrl;
	}
	val |= 1 << 7;
	ret = kp_writeb(kp, REG_KEYP_CTRL, val);
	if (ret) {
		pr_err("%s: can't enable kp\n", __func__);
		goto err_kp_enable;
	}

	pr_info("%s: %dx%d matrix keypad '%s' registered\n", __func__,
		kp->num_drv, kp->num_sns, pdata->name);

	return 0;

err_kp_enable:
err_read_kp_ctrl:
	disable_irq_wake(kp->sense_irq);
	free_irq(kp->stuck_irq, kp);
err_req_stuck_irq:
	free_irq(kp->sense_irq, kp);
err_req_sense_irq:
err_pdata_init:
err_kp_hw_init:
	input_unregister_device(kp->input_dev);
err_input_dev_reg:
	input_free_device(kp->input_dev);
err_input_dev_alloc:
	platform_set_drvdata(pdev, NULL);
	kfree(kp);
	return ret;
}

static struct platform_driver pm8058_keypad_driver = {
	.probe		= pm8058_keypad_probe,
	.driver		= {
		.name	= "pm8058-keypad",
		.owner	= THIS_MODULE,
	},
};

static int __init pm8058_keypad_init(void)
{
	return platform_driver_register(&pm8058_keypad_driver);
}
device_initcall(pm8058_keypad_init);
