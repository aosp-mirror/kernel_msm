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

#define EKT8232NAME "elan-touch"

#define ELAN_TS_ABS_X_MIN 	32
#define ELAN_TS_ABS_X_MAX 	352
#define ELAN_TS_ABS_Y_MIN 	32
#define ELAN_TS_ABS_Y_MAX 	544
#define ELAN_TS_FUZZ 		0
#define ELAN_TS_FLAT 		0

enum {
	STATE_DEEP_SLEEP	= 0,
	STATE_NORMAL		= 1U,
	STATE_MASK		= 0x08,
	cmd_reponse_packet	= 0x52,
	read_cmd_packet		= 0x53,
	write_cmd_packet	= 0x54,
	hello_packet 		= 0x55,
	idx_coordinate_packet 	= 0x5a,
};

static struct ekt8232_data {
	int use_irq;
	struct hrtimer timer;
	struct work_struct work;
	struct i2c_client *client;
	struct input_dev *input;
	wait_queue_head_t wait;
} ekt_data;

/* Though the writing-clients suggest to use SMBus level
 * communication instead of plain i2c communication.
 * But msm_i2c_algo registers master_xfer only.
 */

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[4] = { 0 };

	rc = i2c_master_recv(client, buf_recv, 4);
	if (rc != 4) {
		dev_err(&client->dev,
			"%s: get hello packet failed!, rc = %d\n",
			__FUNCTION__, rc);
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

	rc = i2c_master_send(client, cmd, sizeof(cmd));
	if (rc != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __FUNCTION__);
		return -EINVAL;
	}
	msleep(50);

	rc = i2c_master_recv((struct i2c_client *)client, buf_recv, 4);
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	dev_dbg(&client->dev,
		"%s: firmware version: %d.%d\n", __FUNCTION__, major, minor);

	return 0;
}

static int __set_report_type(struct i2c_client *client)
{
	return 0;
}

static int __parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
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
	dev_dbg(&client->dev, "%s: hello packet got.\n", __FUNCTION__);

	rc = __fw_packet_handler(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev, "%s: firmware checking done.\n", __FUNCTION__);

	rc = __set_report_type(client);
	if (rc < 0)
		goto hand_shake_failed;
	dev_dbg(&client->dev,
		"%s: channging operating mode done.\n", __FUNCTION__);

hand_shake_failed:
	return rc;
}

static void ekt8232_work_func(struct work_struct *work)
{
	int rc;
	uint8_t buf[8] = { 0 };
	struct i2c_client *client = ekt_data.client;

	rc = i2c_master_recv(client, buf, 8);
	if (rc != 8) {
		dev_err(&client->dev,
			"%s: i2c_master_recv error?! \n", __FUNCTION__);
		goto done;
	}

	switch (buf[0]) {
	case idx_coordinate_packet: {
		uint16_t x1, x2, y1, y2;
		uint8_t finger_stat;

		__parse_xy(&buf[1], &x1, &y1);
		finger_stat = buf[7] >> 1;
		dev_dbg(&client->dev,
			"x1 = %d, y1 = %d, finger status = %d\n",
			x1, y1, finger_stat);
		if (finger_stat != 0) {
			input_report_abs(ekt_data.input, ABS_X, x1);
			input_report_abs(ekt_data.input, ABS_Y,
					 ELAN_TS_ABS_Y_MAX - y1);
		}
		input_report_key(ekt_data.input, BTN_TOUCH, finger_stat);
		input_report_key(ekt_data.input, BTN_2, finger_stat == 2);

		if (finger_stat > 1) {
			__parse_xy(&buf[4], &x2, &y2);
			dev_dbg(&client->dev, "x2 = %d, y2 = %d\n", x2, y2);
			input_report_abs(ekt_data.input, ABS_HAT0X, x2);
			input_report_abs(ekt_data.input, ABS_HAT0Y,
					 ELAN_TS_ABS_Y_MAX - y2);
		}
		input_sync(ekt_data.input);
		break;
	}
	default:
		dev_err(&client->dev,
			"%s: Unknown packet type: %0x\n",
			__FUNCTION__, buf[0]);
		break;
	}

done:
	if (ekt_data.use_irq)
		enable_irq(ekt_data.client->irq);
}

static irqreturn_t ekt8232_ts_interrupt(int irq, void *dev_id)
{
	disable_irq(irq);
	schedule_work(&ekt_data.work);

	return IRQ_HANDLED;
}

static enum hrtimer_restart ekt8232_ts_timer_func(struct hrtimer *timer)
{
	schedule_work(&ekt_data.work);
	hrtimer_start(&ekt_data.timer, ktime_set(0, 12500000),
		      HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static int __init ekt8232_register_input(struct input_dev *input)
{
	dev_dbg(&input->dev, "%s: enter\n", __FUNCTION__);

	input->name = EKT8232NAME;
	input->id.bustype = BUS_I2C;

	input->evbit[0] =
		BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input, ABS_X,
			     ELAN_TS_ABS_X_MIN, ELAN_TS_ABS_X_MAX,
			     ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(input, ABS_Y,
			     ELAN_TS_ABS_Y_MIN, ELAN_TS_ABS_Y_MAX,
			     ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(input, ABS_HAT0X,
			     ELAN_TS_ABS_X_MIN, ELAN_TS_ABS_X_MAX,
			     ELAN_TS_FUZZ, ELAN_TS_FLAT);
	input_set_abs_params(input, ABS_HAT0Y,
			     ELAN_TS_ABS_Y_MIN, ELAN_TS_ABS_Y_MAX,
			     ELAN_TS_FUZZ, ELAN_TS_FLAT);

	return input_register_device(input);
}

static int ekt8232_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0, retry = 10;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"No supported i2c func what we need?!!\n");
		return -ENOTSUPP;
	}

	ekt_data.client = client;
	strlcpy(client->name, EKT8232NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &ekt_data);
	INIT_WORK(&ekt_data.work, ekt8232_work_func);
	init_waitqueue_head(&ekt_data.wait);

	ekt_data.input = input_allocate_device();
	if (ekt_data.input == NULL)
		return -ENOMEM;

	/* Actually, we are missing the first time interrupt here.
	 * So, we need to fake we are getting the interrupt for hello packet.
	 */
	do {
		err = ekt8232_ts_init(client);
		if (err >= 0)
			break;
		msleep(100);
	} while (retry--);

	if (err < 0) {
		dev_dbg(&client->dev,
			"looks like it's not Elan, so..i'll quit\n");
		err = -ENODEV;
		goto fail;
	}

	err = ekt8232_register_input(ekt_data.input);
	if (err < 0) {
		dev_err(&client->dev,
			"%s: register to input system failed, err = %d\n",
			__FUNCTION__, err);
		goto fail;
	}
	/* CPLD does not accept configuring interrupt type */
	if (client->irq) {
		err = request_irq(client->irq, ekt8232_ts_interrupt, 0,
				  EKT8232NAME, &ekt_data);
		if (err < 0) {
			dev_err(&client->dev,
				"%s(%s): Can't allocate irq %d\n",
				__FILE__, __FUNCTION__, client->irq);
			ekt_data.use_irq = 0;
			free_irq(client->irq, &ekt_data);
		} else {
			ekt_data.use_irq = 1;
		}
	}
	if (!ekt_data.use_irq) {
		hrtimer_init(&ekt_data.timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ekt_data.timer.function = ekt8232_ts_timer_func;
		hrtimer_start(&ekt_data.timer, ktime_set(1, 0),
			      HRTIMER_MODE_REL);
	}
	return 0;

fail:
	input_free_device(ekt_data.input);
	return err;
}

static int ekt8232_remove(struct i2c_client *client)
{
	struct ekt8232_data *tp = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s: enter.\n", __FUNCTION__);

	input_unregister_device(tp->input);

	free_irq(client->irq, tp);

	return 0;
}
static int ekt8232_get_power_state(struct i2c_client *client)
{
	uint8_t cmd[] = { read_cmd_packet, 0x50, 0x00, 0x01 };
	uint8_t power_state, buf_recv[4] = { 0 };

	dev_dbg(&client->dev, "%s: enter.\n", __FUNCTION__);

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __FUNCTION__);
		return -EINVAL;
	}
	msleep(50);

	if ((i2c_master_recv(client, buf_recv, 4)) != 4) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __FUNCTION__);
		return -EINVAL;
	}

	if (buf_recv[0] != cmd_reponse_packet) {
		dev_err(&client->dev,
			"%s: unknown packet got ?!\n", __FUNCTION__);
		return -EINVAL;
	} else {
		power_state = buf_recv[1];
		dev_dbg(&client->dev,
			"dump repsponse: %0x\n", power_state);

		power_state = (power_state & STATE_MASK) >> 3;
		dev_dbg(&client->dev, "power state = %s\n",
			power_state == STATE_DEEP_SLEEP ?
			"Deep Sleep" : "Normal/Idle");

		return power_state;
	}
}

static int ekt8232_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = { write_cmd_packet, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "%s: enter.\n", __FUNCTION__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_PM
/*
 *	ekt8232_suspend -- suspend the ekt8232 panel
 *
 *	1. Send Power State Packet to get the present state
 *	2. If it is not in *Deep Sleep*, send Power State Packet
 *	to change that.
 */
static int ekt8232_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc = 0;

	dev_dbg(&client->dev,
		"%s: enter. irq = %d\n", __FUNCTION__, client->irq);

	cancel_work_sync(&ekt_data.work);

	disable_irq(client->irq);

	rc = ekt8232_set_power_state(client, STATE_DEEP_SLEEP);

	return rc;
}

static int ekt8232_resume(struct i2c_client *client)
{
	int rc = 0;

	dev_dbg(&client->dev,
		"%s: enter. irq = %d\n", __FUNCTION__, client->irq);

	ekt8232_set_power_state(client, STATE_NORMAL);

	rc = wait_event_timeout(ekt_data.wait,
				((rc = ekt8232_get_power_state(client)) == STATE_NORMAL), HZ);
	if (rc <= 0) {
		dev_err(&client->dev, "Oops, can not wake touch panel up!!!\n");
		return -ETIMEDOUT;
	}

	dev_dbg(&client->dev,
		"%s: done. enable irq = %d\n", __FUNCTION__, client->irq);

	enable_irq(client->irq);

	return rc;
}
#else
#define ekt8232_suspend		NULL
#define ekt8232_resume		NULL
#endif

static const struct i2c_device_id ekt8232_id[] = {
	{ EKT8232NAME, 0 },
	{ }
};

static struct i2c_driver ekt8232_driver = {
	.probe		= ekt8232_probe,
	.remove		= ekt8232_remove,
	.suspend	= ekt8232_suspend,
	.resume		= ekt8232_resume,
	.id_table	= ekt8232_id,
	.driver		= {
		.name = EKT8232NAME,
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
