/*
 * Copyright (C) 2007-2008 HTC Corporation.
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

#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/elan_i2c.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>


static const char EKT8232NAME[]	= "elan-touch";

#define ELAN_TS_FUZZ 		0
#define ELAN_TS_FLAT 		0
#define IDX_PACKET_SIZE		9

enum {
	STATE_DEEP_SLEEP	= 0,
	STATE_NORMAL		= 1U,
	STATE_MASK		= 0x08,
	cmd_reponse_packet	= 0x52,
	read_cmd_packet		= 0x53,
	write_cmd_packet	= 0x54,
	hello_packet 		= 0x55,
	enable_int		= 0xa6,
	disable_int		= 0x56,
	idx_coordinate_packet 	= 0x5a,
};

enum {
	idx_finger_width = 7,
	idx_finger_state = 8,
};

static struct workqueue_struct *elan_wq;

static struct ekt8232_data {
	int intr_gpio;
	int use_irq;
	/* delete when finish migration */
	int fw_ver;
	struct hrtimer timer;
	struct work_struct work;
	struct i2c_client *client;
	struct input_dev *input;
	wait_queue_head_t wait;
	int (*power)(int on);
	struct early_suspend early_suspend;
} ekt_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ts_early_suspend(struct early_suspend *h);
static void elan_ts_late_resume(struct early_suspend *h);
#endif

static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s_%#x\n", EKT8232NAME, ekt_data.fw_ver);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_INFO
		       "touch_sysfs_init: subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_INFO
		       "touch_sysfs_init: sysfs_create_group failed\n");
		goto err4;
	}

	return 0 ;
err4:
	kobject_del(android_touch_kobj);
err:
	return ret ;
}

static int ekt8232_detect_int_level(void)
{
	unsigned v;
	v = gpio_get_value(ekt_data.intr_gpio);
	/* printk("ekt8232_detect_int_level: v = %0x\n", v); */
	return v;
}

static int __ekt8232_poll(struct i2c_client *client)
{
	int status = 0, retry = 10;

	do {
		status = ekt8232_detect_int_level();
		dev_dbg(&client->dev, "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	dev_dbg(&client->dev, "%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int ekt8232_poll(struct i2c_client *client)
{
	return __ekt8232_poll(client);
}

static int ekt8232_get_data(struct i2c_client *client, uint8_t *cmd,
			    uint8_t *buf, size_t size, int sleep)
{
	int rc;
	unsigned time_out = msecs_to_jiffies(10);

	dev_dbg(&client->dev, "%s: enter.\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	if (sleep == 1) {
		rc = wait_event_timeout(ekt_data.wait,
				i2c_master_recv(client, buf, size) == size &&
				buf[0] == cmd_reponse_packet, time_out);
		if (rc == 0) {
			dev_err(&client->dev,
				"%s: i2c_master_recv failed\n", __func__);
			return -ETIMEDOUT;
		}
	} else {
		rc = ekt8232_poll(client);
		if (rc < 0)
			return -EINVAL;
		else {
			if (i2c_master_recv(client, buf, size) != size ||
			    buf[0] != cmd_reponse_packet)
				return -EINVAL;
		}
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[4] = { 0 };

	rc = ekt8232_poll(client);
	if (rc < 0) {
		dev_err(&client->dev, "%s: failed!\n", __func__);
		return -EINVAL;
	}

	rc = i2c_master_recv(client, buf_recv, 4);
	if (rc != 4) {
		dev_err(&client->dev,
			"%s: get hello packet failed!, rc = %d\n",
			__func__, rc);
		return rc;
	} else {
		int i;
		dev_dbg(&client->dev,
			"dump hello packet: %0x, %0x, %0x, %0x\n",
			buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

		for (i = 0; i < 4; i++)
			if (buf_recv[i] != hello_packet)
				return -EINVAL;
	}

	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	int rc;
	int major, minor;
	uint8_t cmd[] = { read_cmd_packet, 0x00, 0x00, 0x01 };
	uint8_t buf_recv[4] = { 0 };

	rc = ekt8232_get_data(client, cmd, buf_recv, 4, 0);
	if (rc < 0)
		return rc;

	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);

	/* delete after migration */
	ekt_data.fw_ver = major << 8 | minor;

	printk(KERN_INFO "%s: firmware version: 0x%x\n",
			__func__, ekt_data.fw_ver);
	return 0;
}

static int __set_report_type(struct i2c_client *client)
{
	return 0;
}

static inline int ekt8232_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

/*	ekt8232_ts_init -- hand shaking with touch panel
 *
 *	1. recv hello packet
 *	2. check its' firmware version
 *	3. set up sensitivity, report rate, ...
 */
static int ekt8232_ts_init(struct i2c_client *client)
{
	int rc;

	rc = __hello_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev, "%s: hello packet got.\n", __func__);

	rc = __fw_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev, "%s: firmware checking done.\n", __func__);

	rc = __set_report_type(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev,
		"%s: channging operating mode done.\n", __func__);

	if (ekt_data.fw_ver == 0x103) {
		uint8_t cmd[4] = {0x5c, 0x10, 0x00, 0x01};
		if ((i2c_master_send(client, cmd, 4)) != 4) {
			dev_err(&client->dev,
				"%s: set adc failed\n", __func__);
		}
		cmd[0] = 0x54;
		cmd[0] = 0x43;
		cmd[0] = 0x00;
		cmd[0] = 0x01;
		if ((i2c_master_send(client, cmd, 4)) != 4) {
			dev_err(&client->dev,
				"%s: set gain failed\n", __func__);
		}
	}

hand_shake_failed:
	return rc;
}

static int ekt8232_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {write_cmd_packet, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "%s: enter.\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int ekt8232_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = { read_cmd_packet, 0x50, 0x00, 0x01 };
	uint8_t buf[4], power_state;

	rc = ekt8232_get_data(client, cmd, buf, 4, 0);
	if (rc)
		return rc;
	else {
		power_state = buf[1];
		dev_dbg(&client->dev, "dump repsponse: %0x\n", power_state);

		power_state = (power_state & STATE_MASK) >> 3;
		dev_dbg(&client->dev, "power state = %s\n",
			power_state == STATE_DEEP_SLEEP ?
			"Deep Sleep" : "Normal/Idle");
		return power_state;
	}
}

static int ekt8232_recv_data(struct i2c_client *client, uint8_t *buf)
{
	int rc, bytes_to_recv = IDX_PACKET_SIZE;
	int retry = 5;

	if (ekt_data.fw_ver == 0x101)
		bytes_to_recv = 8;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	rc = i2c_master_recv(client, buf, bytes_to_recv);

	if (rc != bytes_to_recv) {
		dev_err(&client->dev,
			"%s: i2c_master_recv error?! \n", __func__);
		/* power off level shift */
		ekt_data.power(0);
		msleep(5);
		/* power on level shift */
		ekt_data.power(1);
		/* re-initial */
		if (ekt_data.fw_ver > 0x101) {
			msleep(100);
			rc = ekt8232_ts_init(client);
		} else {
			do {
				rc = ekt8232_set_power_state(client,
							     STATE_NORMAL);

				rc = ekt8232_get_power_state(client);
				if (rc != STATE_NORMAL)
					dev_err(&client->dev,
						"%s: wake up tp failed! \
						err = %d\n",
						__func__, rc);
				else
					break;
			} while (--retry);
		}
		if (ekt8232_detect_int_level() == 0)
			queue_work(elan_wq, &ekt_data.work);
		return -EINVAL;
	}

	return rc;
}

static inline void ekt8232_parse_width(uint8_t data, uint8_t *w1, uint8_t *w2)
{
	*w1 = *w2 = 0;
	*w1 = (data & 0xf0) >> 4;
	*w2 = data & 0x0f;
}

static void ekt8232_report_data(struct i2c_client *client, uint8_t *buf)
{
	static unsigned report_time;
	unsigned report_time2;

	switch (buf[0]) {
	case idx_coordinate_packet: {
		uint16_t x1, x2, y1, y2;
		uint8_t finger_stat, w1 = 1, w2 = 1;

		ekt8232_parse_xy(&buf[1], &x1, &y1);
		if (ekt_data.fw_ver == 0x101) {
			finger_stat = buf[7] >> 1;
		} else {
			ekt8232_parse_width(buf[idx_finger_width], &w1, &w2);
			finger_stat = buf[idx_finger_state] >> 1;
		}

		if (finger_stat != 0) {
			input_report_abs(ekt_data.input, ABS_X, x1);
			if (ekt_data.fw_ver == 0x101)
				input_report_abs(ekt_data.input, ABS_Y,
						 (544 - 1) - y1);
			else
				input_report_abs(ekt_data.input, ABS_Y, y1);
			/* only report finger width at y */
			input_report_abs(ekt_data.input, ABS_TOOL_WIDTH, w2);
		}

		dev_dbg(&client->dev,
			"x1 = %d, y1 = %d, \
			 w1 = %d, w2 = %d, finger status = %d\n",
			x1, y1, w1, w2, finger_stat);

		input_report_abs(ekt_data.input, ABS_PRESSURE, 100);
		input_report_key(ekt_data.input, BTN_TOUCH, finger_stat);
		input_report_key(ekt_data.input, BTN_2, finger_stat == 2);

		if (finger_stat > 1) {
			ekt8232_parse_xy(&buf[4], &x2, &y2);
			dev_dbg(&client->dev, "x2 = %d, y2 = %d\n", x2, y2);
			input_report_abs(ekt_data.input, ABS_HAT0X, x2);
			input_report_abs(ekt_data.input, ABS_HAT0Y, y2);
		}
		input_sync(ekt_data.input);
		break;
	}
	default:
		dev_err(&client->dev,
			"%s: Unknown packet type: %0x\n", __func__, buf[0]);
		break;
	}

	report_time2 = jiffies;
	dev_dbg(&client->dev,
		"report time = %d\n",
		jiffies_to_msecs(report_time2 - report_time));

	report_time = report_time2;

}

static void ekt8232_work_func(struct work_struct *work)
{
	int rc;
	uint8_t buf[IDX_PACKET_SIZE] = { 0 };
	struct i2c_client *client = ekt_data.client;

	/* dev_dbg(&client->dev, "%s: enter. \n", __func__); */

	/* this means that we have already serviced it */
	if (ekt8232_detect_int_level())
		return;

	rc = ekt8232_recv_data(client, buf);
	if (rc < 0)
		return;

	ekt8232_report_data(client, buf);
}

static irqreturn_t ekt8232_ts_interrupt(int irq, void *dev_id)
{
	/* the queue_work has spin_lock protection */
	/* disable_irq(irq); */
	queue_work(elan_wq, &ekt_data.work);

	return IRQ_HANDLED;
}

static enum hrtimer_restart ekt8232_ts_timer_func(struct hrtimer *timer)
{
	queue_work(elan_wq, &ekt_data.work);
	hrtimer_start(&ekt_data.timer,
		      ktime_set(0, 12500000),
		      HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static int ekt8232_register_interrupt(struct i2c_client *client)
{
	int err = 0;

	if (client->irq) {
		ekt_data.use_irq = 1;

		err = request_irq(client->irq, ekt8232_ts_interrupt, 0,
				  EKT8232NAME, &ekt_data);
		if (err < 0) {
			dev_err(&client->dev,
				"%s(%s): Can't allocate irq %d\n",
				__FILE__, __func__, client->irq);
			ekt_data.use_irq = 0;
		}
	}

	if (!ekt_data.use_irq) {
		hrtimer_init(&ekt_data.timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ekt_data.timer.function = ekt8232_ts_timer_func;
		hrtimer_start(&ekt_data.timer, ktime_set(1, 0),
			      HRTIMER_MODE_REL);
	}

	dev_dbg(&client->dev,
		"elan starts in %s mode.\n",
		ekt_data.use_irq == 1 ? "interrupt":"polling");
	return 0;
}

static int ekt8232_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct elan_i2c_platform_data *pdata;
	int x_max, y_max;
	uint8_t x_resolution_cmd[] = { read_cmd_packet, 0x60, 0x00, 0x01 };
	uint8_t y_resolution_cmd[] = { read_cmd_packet, 0x63, 0x00, 0x01 };
	uint8_t buf_recv[4] = { 0 };

	elan_wq = create_singlethread_workqueue("elan_wq");
	if (!elan_wq) {
		err = -ENOMEM;
		goto fail;
	}

	printk(KERN_INFO "ekt8232_probe enter.\n");
	dev_dbg(&client->dev, "ekt8232_probe enter.\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"No supported i2c func what we need?!!\n");
		err = -ENOTSUPP;
		goto fail;
	}

	ekt_data.client = client;
	strlcpy(client->name, EKT8232NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &ekt_data);
	INIT_WORK(&ekt_data.work, ekt8232_work_func);
	init_waitqueue_head(&ekt_data.wait);

	ekt_data.input = input_allocate_device();
	if (ekt_data.input == NULL) {
		err = -ENOMEM;
		goto fail;
	}

	pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		ekt_data.intr_gpio =
			((struct elan_i2c_platform_data *)pdata)->intr_gpio;
		ekt_data.power =
			((struct elan_i2c_platform_data *)pdata)->power;
		ekt_data.power(1);
		dev_info(&client->dev, "touch panel is powered on. \n");
		mdelay(500);	/* elan will be ready after about 500 ms */
	} else {
		dev_err(&client->dev, "without platform data??!!\n");
	}

	err = ekt8232_ts_init(client);
	if (err < 0) {
		printk(KERN_INFO "looks like it's not Elan, so..i'll quit\n");
		err = -ENODEV;
		goto fail;
	}

	if (pdata) {
		while (pdata->version > ekt_data.fw_ver) {
			printk(KERN_INFO "ekt8232_probe: old tp detected, "
					"panel version = 0x%x\n",
					ekt_data.fw_ver);
			pdata++;
		}
	}
	printk(KERN_INFO "ekt8232_register_input\n");

	ekt_data.input->name = EKT8232NAME;
	ekt_data.input->id.bustype = BUS_I2C;
	set_bit(EV_SYN, ekt_data.input->evbit);
	set_bit(EV_KEY, ekt_data.input->evbit);
	set_bit(BTN_TOUCH, ekt_data.input->keybit);
	set_bit(BTN_2, ekt_data.input->keybit);
	set_bit(EV_ABS, ekt_data.input->evbit);

	if (ekt_data.fw_ver >= 0x104) {
		err = ekt8232_get_data(ekt_data.client, x_resolution_cmd,
				       buf_recv, 4, 0);
		if (err < 0) {
			dev_err(&client->dev,
				"%s: get x resolution failed, err = %d\n",
				__func__, err);
			goto fail;
		}

		x_max = ((buf_recv[3] & 0xf0) << 4) | ((buf_recv[2] & 0xff));
		printk(KERN_INFO "ekt8232_probe: x_max: %d\n", x_max);

		err = ekt8232_get_data(ekt_data.client, y_resolution_cmd,
				      buf_recv, 4, 0);
		if (err < 0) {
			dev_err(&client->dev,
				"%s: get y resolution failed, err = %d\n",
				__func__, err);
			goto fail;
		}

		y_max = ((buf_recv[3] & 0xf0) << 4) | ((buf_recv[2] & 0xff));
		printk(KERN_INFO "ekt8232_probe: y_max: %d\n", y_max);
		input_set_abs_params(ekt_data.input, ABS_X,
				     pdata->abs_x_min, x_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_Y,
				     pdata->abs_y_min, y_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_HAT0X,
				     pdata->abs_x_min, x_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_HAT0Y,
				     pdata->abs_y_min, y_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
	} else {
		input_set_abs_params(ekt_data.input, ABS_X,
				     pdata->abs_x_min,  pdata->abs_x_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_Y,
				     pdata->abs_y_min,  pdata->abs_y_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_HAT0X,
				     pdata->abs_x_min,  pdata->abs_x_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_HAT0Y,
				     pdata->abs_y_min,  pdata->abs_y_max,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_PRESSURE, 0, 255,
				     ELAN_TS_FUZZ, ELAN_TS_FLAT);
		input_set_abs_params(ekt_data.input, ABS_TOOL_WIDTH, 1, 8,
				     1, ELAN_TS_FLAT);
	}

	err = input_register_device(ekt_data.input);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: input_register_device failed, err = %d\n",
			__func__, err);
		goto fail;
	}

	ekt8232_register_interrupt(ekt_data.client);

	/* checking the interrupt to avoid missing any interrupt */
	if (ekt8232_detect_int_level() == 0)
		ekt8232_ts_interrupt(client->irq, NULL);
#ifdef CONFIG_HAS_EARLYSUSPEND
	ekt_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ekt_data.early_suspend.suspend = elan_ts_early_suspend;
	ekt_data.early_suspend.resume = elan_ts_late_resume;
	register_early_suspend(&ekt_data.early_suspend);
#endif
	touch_sysfs_init();
	return 0;

fail:
	input_free_device(ekt_data.input);
	if (elan_wq)
		destroy_workqueue(elan_wq);
	return err;
}

static int ekt8232_remove(struct i2c_client *client)
{
	struct ekt8232_data *tp = i2c_get_clientdata(client);

	if (elan_wq)
		destroy_workqueue(elan_wq);

	dev_dbg(&client->dev, "%s: enter.\n", __func__);

	input_unregister_device(tp->input);

	if (ekt_data.use_irq)
		free_irq(client->irq, tp);
	else
		hrtimer_cancel(&ekt_data.timer);
	return 0;
}

static int ekt8232_suspend(struct i2c_client *client, pm_message_t mesg)
{
	uint8_t cmd[4];
	int rc = 0;

	dev_dbg(&client->dev, "%s: enter. irq = %d\n", __func__, client->irq);

	cancel_work_sync(&ekt_data.work);

	rc = ekt8232_set_power_state(client, STATE_DEEP_SLEEP);
/*
	rc = ekt8232_get_power_state(client);
	if (rc < 0 || rc != STATE_DEEP_SLEEP)
		dev_err(&client->dev,
			"%s: put tp into sleep failed, err = %d!\n",
			__func__, rc);
*/
	/* disable tp interrupt */
	if (ekt_data.fw_ver > 0x101) {
		memset(cmd, disable_int, 4);
		if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd))
			dev_err(&client->dev,
				"%s: tp disable interrupt failed\n", __func__);
	}

	/* power off level shift */
	ekt_data.power(0);

	return 0;
}

static int ekt8232_resume(struct i2c_client *client)
{
	int rc = 0, retry = 5;

	dev_dbg(&client->dev,
		"%s: enter. irq = %d\n", __func__, client->irq);

	disable_irq(client->irq);

	/* power on level shift */
	ekt_data.power(1);

	/* re-initial */
	if (ekt_data.fw_ver > 0x101) {
		msleep(500);
		rc = ekt8232_ts_init(client);
	} else {
		do {
			rc = ekt8232_set_power_state(client, STATE_NORMAL);
			rc = ekt8232_get_power_state(client);
			if (rc != STATE_NORMAL)
				dev_err(&client->dev,
					"%s: wake up tp failed! err = %d\n",
					__func__, rc);
			else
				break;
		} while (--retry);
	}

	enable_irq(client->irq);

	if (ekt8232_detect_int_level() == 0)
		ekt8232_ts_interrupt(client->irq, NULL);

	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ts_early_suspend(struct early_suspend *h)
{
	struct i2c_client *client = ekt_data.client;

	dev_dbg(&client->dev, "%s enter.\n", __func__);
	ekt8232_suspend(client, PMSG_SUSPEND);
}

static void elan_ts_late_resume(struct early_suspend *h)
{
	struct i2c_client *client = ekt_data.client;

	dev_dbg(&client->dev, "%s enter.\n", __func__);
	ekt8232_resume(client);
}
#endif

/* -------------------------------------------------------------------- */
static const struct i2c_device_id ekt8232_ts_id[] = {
	{ ELAN_8232_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver ekt8232_driver = {
	.probe		= ekt8232_probe,
	.remove		= ekt8232_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= ekt8232_suspend,
	.resume		= ekt8232_resume,
#endif
	.id_table	= ekt8232_ts_id,
	.driver		= {
		.name = ELAN_8232_I2C_NAME,
	},
};

static int __init ekt8232_init(void)
{
	return i2c_add_driver(&ekt8232_driver);
}

static void __exit ekt8232_exit(void)
{
	i2c_del_driver(&ekt8232_driver);
}

module_init(ekt8232_init);
module_exit(ekt8232_exit);

MODULE_AUTHOR("Shan-Fu Chiou <sfchiou@gmail.com>, "
	      "Jay Tu <jay_tu@htc.com>");
MODULE_DESCRIPTION("ELAN ekt8232 driver");
MODULE_LICENSE("GPL");
