/*
 * This file is part of the AL3320 sensor driver.
 * AL3320 is an ambient light sensor.
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
 * Filename: al3320.c
 *
 * Summary:
 *           AL3320 sensor dirver.
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
 * 09/06/15 Templeton Tsai      Porting to device tree version of platform.
 * 31/07/15 Jennifer Delete Ext_Gain.
 */

#include "al3320.h"

/*
 * register access helpers
 */
static struct wake_lock al3320b_wl;
static struct al3320_data *al3320_data_g;

static int __al3320_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct al3320_data *data = i2c_get_clientdata(client);
	u8 idx = 0xff;

	ADD_TO_IDX(reg, idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __al3320_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct al3320_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;
	u8 idx = 0xff;

	ADD_TO_IDX(reg, idx)
	if (idx >= AL3320_NUM_CACHABLE_REGS)
		return -EINVAL;

	tmp = data->reg_cache[idx];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[idx] = tmp;

	return ret;
}

/*
 * internally used functions
 */

/* mode */
static int al3320_get_mode(struct i2c_client *client)
{
	int ret;

	ret = __al3320_read_reg(client, AL3320_MODE_COMMAND,
			AL3320_MODE_MASK, AL3320_MODE_SHIFT);
	return ret;
}

static int al3320_set_mode(struct i2c_client *client, int mode)
{
	struct al3320_data *data = i2c_get_clientdata(client);

	if (mode != al3320_get_mode(client)) {
		__al3320_write_reg(client, AL3320_MODE_COMMAND,
		AL3320_MODE_MASK, AL3320_MODE_SHIFT, mode);

		if (als_polling) {
			/* Enable/Disable ALS */
			if (ALS_ACTIVE & mode) {
				hrtimer_start(&data->light_timer,
				data->light_poll_delay, HRTIMER_MODE_REL);
			} else {
				hrtimer_cancel(&data->light_timer);
				cancel_work_sync(&data->work_light);
			}
		}
	}
	return 0;
}

/* waiting time */
#if 0
static int al3320_get_waiting_time(struct i2c_client *client)
{
	return __al3320_read_reg(client, AL3320_WAITING_TIME,
		AL3320_WAITING_MASK, AL3320_WAITING_SHIFT);
}
#endif

static int al3320_set_waiting_time(struct i2c_client *client, int wait_time)
{
	int ret = __al3320_write_reg(client, AL3320_WAITING_TIME,
		AL3320_WAITING_MASK, AL3320_WAITING_SHIFT, wait_time);

	return ret;
}

/* INT enable */
#if 0
static int al3320_get_int_enable(struct i2c_client *client)
{
	return __al3320_read_reg(client, AL3320_INT_ENABLE,
		AL3320_INT_ENABLE_MASK, AL3320_INT_ENABLE_SHIFT);
}

#endif
static int al3320_set_int_enable(struct i2c_client *client, int flag)
{
	int ret = __al3320_write_reg(client, AL3320_INT_ENABLE,
		AL3320_INT_ENABLE_MASK, AL3320_INT_ENABLE_SHIFT, flag);

	return ret;
}

/* suspend enable */
#if 0
static int al3320_get_sus_enable(struct i2c_client *client)
{
	return __al3320_read_reg(client, AL3320_INT_ENABLE,
		AL3320_SUS_ENABLE_MASK, AL3320_SUS_ENABLE_SHIFT);
}
#endif

static int al3320_set_sus_enable(struct i2c_client *client, int flag)
{
	int ret = __al3320_write_reg(client, AL3320_INT_ENABLE,
		AL3320_SUS_ENABLE_MASK, AL3320_SUS_ENABLE_SHIFT, flag);

	return ret;
}

/* range */
static long al3320_get_range(struct i2c_client *client)
{
	u8 idx;

	idx = __al3320_read_reg(client, AL3320_RAN_COMMAND,
		AL3320_RAN_MASK, AL3320_RAN_SHIFT);

	return al3320_range[idx];
}

static int al3320_set_range(struct i2c_client *client, int range)
{
	int adummy, ret;

	switch (range) {
	case ALS_RAN_0:
		adummy = ALS_ADUMMY_0;
		break;
	case ALS_RAN_1:
		adummy = ALS_ADUMMY_1;
		break;
	case ALS_RAN_2:
		adummy = ALS_ADUMMY_2;
		break;
	case ALS_RAN_3:
		adummy = ALS_ADUMMY_3;
		break;
	default:
		adummy = 0;
	}

	ret = al3320_set_adummy(client, adummy);

	if (ret)
		return ret;

	return __al3320_write_reg(client, AL3320_RAN_COMMAND,
		AL3320_RAN_MASK, AL3320_RAN_SHIFT, range);
}
/* persist */
#if 0
static int al3320_get_persist(struct i2c_client *client)
{
	return __al3320_read_reg(client, AL3320_ALS_PERSIST,
		AL3320_PERSIST_MASK, AL3320_PERSIST_SHIFT);
}
#endif

static int al3320_set_persist(struct i2c_client *client, int persist)
{
	return __al3320_write_reg(client, AL3320_ALS_PERSIST,
		AL3320_PERSIST_MASK, AL3320_PERSIST_SHIFT, persist);
}

/* meantime */
#if 0
static int al3320_get_meantime(struct i2c_client *client)
{
	return __al3320_read_reg(client, AL3320_ALS_MEANTIME,
		AL3320_MEANTIME_MASK, AL3320_MEANTIME_SHIFT);
}
#endif

static int al3320_set_meantime(struct i2c_client *client, int meantime)
{
	return __al3320_write_reg(client, AL3320_ALS_MEANTIME,
		AL3320_MEANTIME_MASK, AL3320_MEANTIME_SHIFT, meantime);
}

/* a-dummy */
#if 0
static int al3320_get_adummy(struct i2c_client *client)
{
	return __al3320_read_reg(client, AL3320_ALS_ADUMMY,
		AL3320_ADUMMY_MASK, AL3320_ADUMMY_SHIFT);
}
#endif

static int al3320_set_adummy(struct i2c_client *client, int adummy)
{
	return __al3320_write_reg(client, AL3320_ALS_ADUMMY,
		AL3320_ADUMMY_MASK, AL3320_ADUMMY_SHIFT, adummy);
}

/* ALS low threshold */
static int al3320_get_althres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __al3320_read_reg(client, AL3320_ALS_LTHL,
				AL3320_ALS_LTHL_MASK, AL3320_ALS_LTHL_SHIFT);
	msb = __al3320_read_reg(client, AL3320_ALS_LTHH,
				AL3320_ALS_LTHH_MASK, AL3320_ALS_LTHH_SHIFT);
	return (msb << 8) | lsb;
}

static int al3320_set_althres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & AL3320_ALS_LTHL_MASK;

	err = __al3320_write_reg(client, AL3320_ALS_LTHL,
		AL3320_ALS_LTHL_MASK, AL3320_ALS_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __al3320_write_reg(client, AL3320_ALS_LTHH,
		AL3320_ALS_LTHH_MASK, AL3320_ALS_LTHH_SHIFT, msb);

	return err;
}

/* ALS high threshold */
static int al3320_get_ahthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __al3320_read_reg(client, AL3320_ALS_HTHL,
				AL3320_ALS_HTHL_MASK, AL3320_ALS_HTHL_SHIFT);
	msb = __al3320_read_reg(client, AL3320_ALS_HTHH,
				AL3320_ALS_HTHH_MASK, AL3320_ALS_HTHH_SHIFT);
	return (msb << 8) | lsb;
}

static int al3320_set_ahthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & AL3320_ALS_HTHL_MASK;

	err = __al3320_write_reg(client, AL3320_ALS_HTHL,
		AL3320_ALS_HTHL_MASK, AL3320_ALS_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __al3320_write_reg(client, AL3320_ALS_HTHH,
		AL3320_ALS_HTHH_MASK, AL3320_ALS_HTHH_SHIFT, msb);

	return err;
}

static int al3320_get_adc_value(struct i2c_client *client, int lock)
{
	struct al3320_data *data = i2c_get_clientdata(client);
	unsigned int lsb, msb;
	unsigned long tmp, range;
	int lthreshold, hthreshold;

	if (!lock)
		mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, AL3320_ADC_LSB);

	if (lsb < 0) {
		if (!lock)
			mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3320_ADC_MSB);
	if (!lock)
		mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	if (!als_polling) {
		tmp = (msb << 8) | lsb;
		lthreshold = tmp * (100 - als_tolerance) / 100;
		if (lthreshold < 0)
			lthreshold = 0;
		hthreshold = tmp * (100 + als_tolerance) / 100;
		if (hthreshold > 0xffff)
			hthreshold = 0xffff;
		al3320_set_althres(data->client, lthreshold);
		al3320_set_ahthres(data->client, hthreshold);
		pr_debug("adc value: %ld, lthreshold:%d hthreshold:%d\n", tmp, lthreshold, hthreshold);
	}

	range = al3320_get_range(client);

	tmp = (((msb << 8) | lsb) * range) >> 16;
	tmp *= cali;

	return tmp / 100;
}

static int al3320_clean_int(struct i2c_client *client)
{
	int err;

	err = __al3320_write_reg(client, AL3320_INT_COMMAND,
		AL3320_INT_MASK, AL3320_INT_SHIFT, 0);

	return err;
}

/*
 * sysfs layer
 */

/* als_poll_delay */
static ssize_t al3320_show_als_poll_delay(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct al3320_data *data = al3320_data_g;

	return snprintf(buf, 20, "%d\n",
		do_div(ktime_to_ns(data->light_poll_delay), 1000));
}

static ssize_t al3320_store_als_poll_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct al3320_data *data = al3320_data_g;
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	data->light_poll_delay = ns_to_ktime(val*1000);

	return count;
}


/* range */
static ssize_t al3320_show_range(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct al3320_data *data = al3320_data_g;
	return snprintf(buf, 20,
		"%ld\n", al3320_get_range(data->client));
}

static ssize_t al3320_store_range(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct al3320_data *data = al3320_data_g;
	unsigned long val;
	int ret;

	if ((kstrtoul(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;

	ret = al3320_set_range(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

/* mode */
static ssize_t al3320_show_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct al3320_data *data = al3320_data_g;

	return snprintf(buf, 20,
		"%d\n", al3320_get_mode(data->client));
}

static ssize_t al3320_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct al3320_data *data = al3320_data_g;
	unsigned long val;
	int ret;

	if ((kstrtoul(buf, 10, &val) < 0) || (val > 4))
		return -EINVAL;

	ret = al3320_set_mode(data->client, val);

	if (ret < 0)
		return ret;
	return count;
}



/* lux */
static ssize_t al3320_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct al3320_data *data = al3320_data_g;

	/* No LUX data if power down */
	if (al3320_get_mode(data->client) == 0x00) {
		wake_lock_timeout(&al3320b_wl, 10*HZ);
		return snprintf((char *) buf, 20,
			"%s\n", "Please power up first!");
	}

	return snprintf(buf, 20, "%d\n",
		al3320_get_adc_value(data->client, 0));
}



/* ALS low threshold */
static ssize_t al3320_show_althres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct al3320_data *data = al3320_data_g;
	return snprintf(buf, 20,
		"%d\n", al3320_get_althres(data->client));
}

static ssize_t al3320_store_althres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct al3320_data *data = al3320_data_g;
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	ret = al3320_set_althres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}



/* ALS high threshold */
static ssize_t al3320_show_ahthres(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct al3320_data *data = al3320_data_g;
	return snprintf(buf, 20, "%d\n",
		al3320_get_ahthres(data->client));
}

static ssize_t al3320_store_ahthres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct al3320_data *data = al3320_data_g;
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	ret = al3320_set_ahthres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}



/* calibration */
static ssize_t al3320_show_calibration_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, 20, "%d\n", cali);
}

static ssize_t al3320_store_calibration_state(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct al3320_data *data = al3320_data_g;
	int stdls, lux;
	char tmp[10];

	/* No LUX data if not operational */
	if (al3320_get_mode(data->client) == 0x00) {
		pr_err("Please power up first!");
		return -EINVAL;
	}

	cali = 100;
	sscanf(buf, "%d %s", &stdls, tmp);

	if (!strncmp(tmp, "-setcv", 6)) {
		cali = stdls;
		return -EBUSY;
	}

	if (stdls < 0) {
		pr_err("Std light source: [%d] < 0 !!!\n", stdls);
		pr_err("Check again, please.\n");
		pr_err("Set calibration factor to 100.\n");
		return -EBUSY;
	}

	lux = al3320_get_adc_value(data->client, 0);
	cali = stdls * 100 / lux;

	return -EBUSY;
}

static ssize_t al3320_check_id(struct i2c_client *client)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, AL3320_ID);
	if (ret != 0x07) {
		pr_err("%s: unknown chip ID 0x%x\n", __func__, ret);
		return -EIO;
	}
	pr_info("%s: Chip ID 0x%x confirmed\n", __func__, ret);
	return 0;
}

#ifdef LSC_DBG
/* engineer mode */
static ssize_t al3320_em_read(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct al3320_data *data = al3320_data_g;
	int i;
	u8 tmp;

	for (i = 0; i < AL3320_NUM_CACHABLE_REGS; i++) {
		mutex_lock(&data->lock);
		tmp = i2c_smbus_read_byte_data(data->client, al3320_reg[i]);
		mutex_unlock(&data->lock);

		pr_info("Reg[0x%02x] Val[0x%02x]\n", al3320_reg[i], tmp);
	}

	return 0;
}

static ssize_t al3320_em_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct al3320_data *data = al3320_data_g;
	u32 addr, val, idx = 0;
	int ret = 0;

	sscanf(buf, "%x%x", &addr, &val);

	pr_info("Write [%x] to Reg[%x]...\n", val, addr);
	mutex_lock(&data->lock);

	ret = i2c_smbus_write_byte_data(data->client, addr, val);
	ADD_TO_IDX(addr, idx)
	if (!ret)
		data->reg_cache[idx] = val;

	mutex_unlock(&data->lock);

	return count;
}
#endif


static DEVICE_ATTR(als_poll_delay, S_IWUSR | S_IWGRP | S_IRUGO,
	al3320_show_als_poll_delay, al3320_store_als_poll_delay);
static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
	al3320_show_range, al3320_store_range);

static DEVICE_ATTR(mode, S_IWUSR | S_IWGRP | S_IRUGO,
	al3320_show_mode, al3320_store_mode);
static DEVICE_ATTR(lux, S_IRUGO, al3320_show_lux, NULL);
static DEVICE_ATTR(althres, S_IWUSR | S_IRUGO,
	al3320_show_althres, al3320_store_althres);
static DEVICE_ATTR(ahthres, S_IWUSR | S_IRUGO,
	al3320_show_ahthres, al3320_store_ahthres);
static DEVICE_ATTR(calibration, S_IWUSR | S_IRUGO,
	al3320_show_calibration_state, al3320_store_calibration_state);
#ifdef LSC_DBG
static DEVICE_ATTR(em, S_IWUSR | S_IRUGO,
	al3320_em_read, al3320_em_write);
#endif

static struct attribute *al3320_attrs[] = {
	&dev_attr_als_poll_delay.attr,
	&dev_attr_range.attr,
	&dev_attr_mode.attr,
	&dev_attr_lux.attr,
	&dev_attr_althres.attr,
	&dev_attr_ahthres.attr,
	&dev_attr_calibration.attr,
	&dev_attr_em.attr,
	NULL
};

static const struct attribute_group al3320_attribute_group = {
	.attrs = al3320_attrs,
};

static int al3320_init_client(struct i2c_client *client)
{
	struct al3320_data *data = i2c_get_clientdata(client);
	int i;

	/* reset */
	al3320_set_mode(client, ALS_RESET);
	mdelay(15);

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < AL3320_NUM_CACHABLE_REGS; i++) {
		int v = i2c_smbus_read_byte_data(client, al3320_reg[i]);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */

	/* ALS waiting time */
	al3320_set_waiting_time(client, ALS_NO_WAITING);
	mdelay(1);

	/* ALS gain */
	al3320_set_range(client, als_range);
	mdelay(1);


	/* ALS meantime */
	al3320_set_meantime(client, als_meantime);
	mdelay(1);

	/* interrupt, suspend settings */
	al3320_set_sus_enable(client, DISABLE);
	mdelay(1);

	if (als_polling)
		al3320_set_int_enable(client, DISABLE);
	else {
		al3320_set_althres(client, als_low_threshold);
		al3320_set_ahthres(client, als_high_threshold);
		mdelay(1);

		/* ALS persist */
		al3320_set_persist(client, als_persist);
	}

	mdelay(1);

	return 0;
}

static void al3320_work_func_light(struct work_struct *work)
{
	struct al3320_data *data = container_of(work, struct al3320_data,
						work_light);
	int Aval;

	mutex_lock(&data->lock);

	Aval = al3320_get_adc_value(data->client, 1);
	pr_debug("ALS lux value: %u\n", Aval);

	mutex_unlock(&data->lock);

	input_report_abs(data->light_input_dev, ABS_MISC, Aval);
	input_sync(data->light_input_dev);
}

static enum hrtimer_restart al3320_light_timer_func(struct hrtimer *timer)
{
	struct al3320_data *data = container_of(timer, struct al3320_data,
						light_timer);
	queue_work(data->wq, &data->work_light);
	hrtimer_forward_now(&data->light_timer, data->light_poll_delay);
	return HRTIMER_RESTART;
}

static void al3320_timer_init(struct al3320_data *data)
{
	if (als_polling) {
		/* light hrtimer settings. */
		hrtimer_init(&data->light_timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		data->light_poll_delay = ns_to_ktime(
						als_poll_delay * NSEC_PER_MSEC);
		data->light_timer.function = al3320_light_timer_func;
	}
}

static int al3320_input_init(struct al3320_data *data)
{
	struct input_dev *input_dev;
	int ret;

	/* allocate light input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		LDBG("could not allocate input device\n");
		goto err_light_all;
	}
	input_set_drvdata(input_dev, data);
	input_dev->name = "light";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

	LDBG("registering light sensor input device\n");
	ret = input_register_device(input_dev);
	if (ret < 0) {
		LDBG("could not register input device\n");
		goto err_light_reg;
	}
	data->light_input_dev = input_dev;

	return 0;

err_light_reg:
	input_free_device(input_dev);
err_light_all:

	return -EBUSY;
}

static void al3320_input_fini(struct al3320_data *data)
{
	struct input_dev *dev = data->light_input_dev;

	input_unregister_device(dev);
	input_free_device(dev);
}

/*
 * I2C layer
 */

static irqreturn_t al3320_irq(int irq, void *data_)
{
	struct al3320_data *data = data_;

	mutex_lock(&data->lock);

	/* ALS int */
	queue_work(data->wq, &data->work_light);

	/* clean interrupt flag */
	al3320_clean_int(data->client);

	mutex_unlock(&data->lock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void al3320_early_suspend(struct early_suspend *h)
{
	struct al3320_data *data = container_of(h, struct al3320_data,
						early_suspend);

	LDBG("early suspend\n");

	if ((al3320_get_mode(data->client) & ALS_ACTIVE))
		suspend_mode = ALS_ACTIVE;

	al3320_set_mode(data->client, ALS_DEACTIVE);
}

static void al3320_late_resume(struct early_suspend *h)
{
	struct al3320_data *data = container_of(h, struct al3320_data,
						early_suspend);

	LDBG("late resume\n");

	if (suspend_mode) {
		al3320_set_mode(data->client, ALS_ACTIVE);
		suspend_mode = ALS_DEACTIVE;
	}
}
#endif
static int al3320_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct al3320_data *data;
	int err = 0;
	struct device *dev = &client->dev;
	struct pinctrl_state *set_state;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	wake_lock_init(&al3320b_wl, WAKE_LOCK_SUSPEND, "al3320b");
	data = kzalloc(sizeof(struct al3320_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	al3320_timer_init(data);

	err = al3320_check_id(client);
	if (err)
		return -EIO;

	err = sysfs_create_group(&client->dev.kobj, &al3320_attribute_group);
	if (err)
		goto exit_sysfs;

	/* initialize the AL3320 chip */
	err = al3320_init_client(client);
	if (err)
		goto exit_kfree;

	err = al3320_input_init(data);
	if (err)
		goto exit_kfree;

	if (!als_polling) {
		data->al3320_pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(data->al3320_pinctrl)) {
			if (PTR_ERR(data->al3320_pinctrl) == -EPROBE_DEFER)
				return -EPROBE_DEFER;

			pr_info("Target does not use pinctrl\n");
			data->al3320_pinctrl = NULL;
		}

		set_state = pinctrl_lookup_state(data->al3320_pinctrl,
						"al3320_int_pin_default");
		pinctrl_select_state(data->al3320_pinctrl, set_state);

		data->irq = client->irq;
		err = request_threaded_irq(client->irq, NULL, al3320_irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"al3320", data);

		if (err) {
			dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",
				err, client->irq);
				goto exit_irq;
		}
	}

	INIT_WORK(&data->work_light, al3320_work_func_light);

	data->wq = create_singlethread_workqueue("al3320_wq");
	if (!data->wq) {
		LDBG("could not create workqueue\n");
		goto exit_work;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	data->early_suspend.suspend = al3320_early_suspend;
	data->early_suspend.resume = al3320_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	al3320_data_g = data;
	dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
	return 0;

exit_work:
	destroy_workqueue(data->wq);

exit_irq:
	al3320_input_fini(data);

exit_kfree:
	mutex_destroy(&data->lock);
	kfree(data);

exit_sysfs:
	sysfs_remove_group(&client->dev.kobj, &al3320_attribute_group);

	return err;
}

static int al3320_remove(struct i2c_client *client)
{
	struct al3320_data *data = i2c_get_clientdata(client);

	if (!als_polling)
		free_irq(data->irq, data);
	input_unregister_device(data->light_input_dev);

	if (data->reg_cache[0] & ALS_ACTIVE) {
			hrtimer_cancel(&data->light_timer);
			cancel_work_sync(&data->work_light);
	}

	destroy_workqueue(data->wq);
	mutex_destroy(&data->lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	kfree(data);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id al3320_match_table[] = {
	{ .compatible = "dynaimage,al3320",},
	{ },
};
#else
#define al3320_match_table NULL
#endif

static const struct i2c_device_id al3320_id[] = {
	{ "al3320", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3320_id);

static struct i2c_driver al3320_driver = {
	.driver = {
		.name = AL3320_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = al3320_match_table,
	},
	.probe = al3320_probe,
	.remove = al3320_remove,
	.id_table = al3320_id,
};

static int __init al3320_init(void)
{
	int ret;
	ret = i2c_add_driver(&al3320_driver);

	return ret;
}

static void __exit al3320_exit(void)
{
	i2c_del_driver(&al3320_driver);
}

MODULE_AUTHOR("templeton.tsai@dyna-image.com");
MODULE_DESCRIPTION("AL3320 driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3320_init);
module_exit(al3320_exit);

