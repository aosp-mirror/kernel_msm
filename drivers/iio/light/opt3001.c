/**
 * opt3001.c - Texas Instruments OPT3001 Light Sensor
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Andreas Dannenberg <dannenberg@ti.com>
 * Based on previous work from: Felipe Balbi <balbi@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 of the License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#define OPT3001_RESULT		0x00
#define OPT3001_CONFIGURATION	0x01
#define OPT3001_LOW_LIMIT	0x02
#define OPT3001_HIGH_LIMIT	0x03
#define OPT3001_MANUFACTURER_ID	0x7e
#define OPT3001_DEVICE_ID	0x7f

#define OPT3001_CONFIGURATION_RN_MASK	(0xf << 12)
#define OPT3001_CONFIGURATION_RN_AUTO	(0xc << 12)

#define OPT3001_CONFIGURATION_CT	BIT(11)

#define OPT3001_CONFIGURATION_M_MASK	(3 << 9)
#define OPT3001_CONFIGURATION_M_SHUTDOWN (0 << 9)
#define OPT3001_CONFIGURATION_M_SINGLE	(1 << 9)
#define OPT3001_CONFIGURATION_M_CONTINUOUS (2 << 9) /* also 3 << 9 */

#define OPT3001_CONFIGURATION_OVF	BIT(8)
#define OPT3001_CONFIGURATION_CRF	BIT(7)
#define OPT3001_CONFIGURATION_FH	BIT(6)
#define OPT3001_CONFIGURATION_FL	BIT(5)
#define OPT3001_CONFIGURATION_L		BIT(4)
#define OPT3001_CONFIGURATION_POL	BIT(3)
#define OPT3001_CONFIGURATION_ME	BIT(2)

#define OPT3001_CONFIGURATION_FC_MASK	(3 << 0)

/* The end-of-conversion enable is located in the low-limit register */
#define OPT3001_LOW_LIMIT_EOC_ENABLE	0xc000

#define OPT3001_REG_EXPONENT(n)		((n) >> 12)
#define OPT3001_REG_MANTISSA(n)		((n) & 0xfff)

#define OPT3001_INT_TIME_LONG		800000
#define OPT3001_INT_TIME_SHORT		100000

/*
 * Time to wait for conversion result to be ready. The device datasheet
 * sect. 6.5 states results are ready after total integration time plus 3ms.
 * This results in worst-case max values of 113ms or 883ms, respectively.
 * Add some slack to be on the safe side.
 */
#define OPT3001_RESULT_READY_SHORT	150
#define OPT3001_RESULT_READY_LONG	1000

/* OPT3001 polling rate in ms */
#define OPT3001_LS_MIN_POLL_DELAY       1
#define OPT3001_LS_MAX_POLL_DELAY       1000
#define OPT3001_LS_DEFAULT_POLL_DELAY   100

#define CAL_LUX	600
#define CAL_FILE  "/persist/sensors/lightsensor_calibration.dat"

void opt3001_get_cal_data(struct iio_dev *iio);
int opt3001_get_pure_lux(struct iio_dev *iio);

struct opt3001_info {
	bool enabled;
	int poll_delay;
};
struct opt3001_setting {
	int caliberated_value;
};

struct opt3001 {
	struct i2c_client	*client;
	struct device		*dev;

	struct mutex		lock;
	bool			ok_to_ignore_lock;
	bool			result_ready;
	wait_queue_head_t	result_ready_queue;
	u16			result;

	u32			int_time;
	u32			mode;

	u16			high_thresh_mantissa;
	u16			low_thresh_mantissa;

	u8			high_thresh_exp;
	u8			low_thresh_exp;

	bool			use_irq;
	struct sensors_classdev	als_cdev;
	struct opt3001_info	*pdata;
	struct mutex		io_lock;
	struct input_dev	*ls_input_dev;
	struct hrtimer		als_timer;
	struct work_struct	als_work;
	struct workqueue_struct	*als_wq;
	struct opt3001_setting  setting;
	int flush_count;
	bool cal_status;

};

struct opt3001_scale {
	int	val;
	int	val2;
};

static const struct opt3001_scale opt3001_scales[] = {
	{
		.val = 40,
		.val2 = 950000,
	},
	{
		.val = 81,
		.val2 = 900000,
	},
	{
		.val = 163,
		.val2 = 800000,
	},
	{
		.val = 327,
		.val2 = 600000,
	},
	{
		.val = 655,
		.val2 = 200000,
	},
	{
		.val = 1310,
		.val2 = 400000,
	},
	{
		.val = 2620,
		.val2 = 800000,
	},
	{
		.val = 5241,
		.val2 = 600000,
	},
	{
		.val = 10483,
		.val2 = 200000,
	},
	{
		.val = 20966,
		.val2 = 400000,
	},
	{
		.val = 83865,
		.val2 = 600000,
	},
};

static struct sensors_classdev sensors_light_cdev = {
	.name = "opt3001-light",
	.vendor = "TI",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "800",
	.resolution = "0.0125",
	.sensor_power = "0.15",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = OPT3001_LS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int opt3001_find_scale(const struct opt3001 *opt, int val,
		int val2, u8 *exponent)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(opt3001_scales); i++) {
		const struct opt3001_scale *scale = &opt3001_scales[i];

		/*
		 * Combine the integer and micro parts for comparison
		 * purposes. Use milli lux precision to avoid 32-bit integer
		 * overflows.
		 */
		if ((val * 1000 + val2 / 1000) <=
				(scale->val * 1000 + scale->val2 / 1000)) {
			*exponent = i;
			return 0;
		}
	}

	return -EINVAL;
}

static void opt3001_to_iio_ret(struct opt3001 *opt, u8 exponent,
		u16 mantissa, int *val, int *val2)
{
	int lux;

	lux = 10 * (mantissa << exponent);
	*val = lux / 1000;
	*val2 = (lux - (*val * 1000)) * 1000;
}

static void opt3001_set_mode(struct opt3001 *opt, u16 *reg, u16 mode)
{
	*reg &= ~OPT3001_CONFIGURATION_M_MASK;
	*reg |= mode;
	opt->mode = mode;
}

static IIO_CONST_ATTR_INT_TIME_AVAIL("0.1 0.8");

static ssize_t opt3001_als_trim_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
        struct iio_dev *iio = dev_to_iio_dev(dev);
	struct opt3001 *als_data = iio_priv(iio);

        return sprintf(buf, "%d\n", als_data->setting.caliberated_value);
}

static ssize_t opt3001_als_trim_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t len)
{
        struct iio_dev *iio = dev_to_iio_dev(dev);
	struct opt3001 *als_data = iio_priv(iio);
        int value;

        if (kstrtoint(buf, 0, &value))
                return -EINVAL;

        if (value)
                als_data->setting.caliberated_value = value;

        return len;
}

static ssize_t opt3001_do_calibrate(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t len)
{
	struct iio_dev *iio = dev_to_iio_dev(dev);
	struct opt3001 *opt = iio_priv(iio);
	int value;
	int ret;
	u16 reg;

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_CONFIGURATION);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x with 800ms, hence fall back to 100ms\n",
				OPT3001_CONFIGURATION);
	}

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if (value == 1) {
		/* Set time as 800ms for proper caliberation data */
		reg = ret;
		reg |= OPT3001_CONFIGURATION_CT;
		ret = i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION, reg);
		if(ret < 0)
			pr_err("Time set for 800ms failed\n");
		opt->int_time = OPT3001_INT_TIME_LONG;
		msleep(100);
		opt3001_get_cal_data(iio);
		/* Set time to 100ms read lux value */
		reg &= ~OPT3001_CONFIGURATION_CT;
		i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION, reg);
		ret = i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION, reg);
		if(ret < 0)
			pr_err("Time set for 100ms failed\n");
		opt->int_time = OPT3001_INT_TIME_SHORT;
	}

	return len;
}

static ssize_t opt3001_lux_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
        struct iio_dev *iio = dev_to_iio_dev(dev);

        return sprintf(buf, "%d\n", opt3001_get_pure_lux(iio));
}

static DEVICE_ATTR(illuminance0_input, S_IRUGO, opt3001_lux_show, NULL);
static DEVICE_ATTR(illuminance0_calibrate, S_IWUSR, NULL, opt3001_do_calibrate);
static DEVICE_ATTR(illuminance0_calibbias, S_IRUGO | S_IWUSR,
			opt3001_als_trim_show, opt3001_als_trim_store);

static struct attribute *opt3001_attributes[] = {
	&iio_const_attr_integration_time_available.dev_attr.attr,
	&dev_attr_illuminance0_calibrate.attr,
	&dev_attr_illuminance0_calibbias.attr,
	&dev_attr_illuminance0_input.attr,
	NULL
};

static const struct attribute_group opt3001_attribute_group = {
	.attrs = opt3001_attributes,
};

static const struct iio_event_spec opt3001_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec opt3001_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_INT_TIME),
		.event_spec = opt3001_event_spec,
		.num_event_specs = ARRAY_SIZE(opt3001_event_spec),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static int opt3001_get_lux(struct opt3001 *opt, int *val, int *val2)
{
	int ret;
	u16 mantissa;
	u16 reg;
	u8 exponent;
	u16 value;
	long timeout;

	if (opt->use_irq) {
		/*
		 * Enable the end-of-conversion interrupt mechanism. Note that
		 * doing so will overwrite the low-level limit value however we
		 * will restore this value later on.
		 */
		ret = i2c_smbus_write_word_swapped(opt->client,
				OPT3001_LOW_LIMIT,
				OPT3001_LOW_LIMIT_EOC_ENABLE);
		if (ret < 0) {
			dev_err(opt->dev, "failed to write register %02x\n",
					OPT3001_LOW_LIMIT);
			return ret;
		}

		/* Allow IRQ to access the device despite lock being set */
		opt->ok_to_ignore_lock = true;
	}

	/* Reset data-ready indicator flag */
	opt->result_ready = false;

	/* Configure for single-conversion mode and start a new conversion */
	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_CONFIGURATION);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_CONFIGURATION);
		goto err;
	}

	reg = ret;
	opt3001_set_mode(opt, &reg, OPT3001_CONFIGURATION_M_SINGLE);

	ret = i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION,
			reg);
	if (ret < 0) {
		dev_err(opt->dev, "failed to write register %02x\n",
				OPT3001_CONFIGURATION);
		goto err;
	}

	if (opt->use_irq) {
		/* Wait for the IRQ to indicate the conversion is complete */
		ret = wait_event_timeout(opt->result_ready_queue,
				opt->result_ready,
				msecs_to_jiffies(OPT3001_RESULT_READY_LONG));
	} else {
		/* Sleep for result ready time */
		timeout = (opt->int_time == OPT3001_INT_TIME_SHORT) ?
			OPT3001_RESULT_READY_SHORT : OPT3001_RESULT_READY_LONG;
		msleep(timeout);

		/* Check result ready flag */
		ret = i2c_smbus_read_word_swapped(opt->client,
				OPT3001_CONFIGURATION);
		if (ret < 0) {
			dev_err(opt->dev, "failed to read register %02x\n",
					OPT3001_CONFIGURATION);
			goto err;
		}

		if (!(ret & OPT3001_CONFIGURATION_CRF)) {
			ret = -ETIMEDOUT;
			goto err;
		}

		/* Obtain value */
		ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_RESULT);
		if (ret < 0) {
			dev_err(opt->dev, "failed to read register %02x\n",
					OPT3001_RESULT);
			goto err;
		}
		opt->result = ret;
		opt->result_ready = true;
	}

err:
	if (opt->use_irq)
		/* Disallow IRQ to access the device while lock is active */
		opt->ok_to_ignore_lock = false;

	if (ret == 0)
		return -ETIMEDOUT;
	else if (ret < 0)
		return ret;

	if (opt->use_irq) {
		/*
		 * Disable the end-of-conversion interrupt mechanism by
		 * restoring the low-level limit value (clearing
		 * OPT3001_LOW_LIMIT_EOC_ENABLE). Note that selectively clearing
		 * those enable bits would affect the actual limit value due to
		 * bit-overlap and therefore can't be done.
		 */
		value = (opt->low_thresh_exp << 12) | opt->low_thresh_mantissa;
		ret = i2c_smbus_write_word_swapped(opt->client,
				OPT3001_LOW_LIMIT,
				value);
		if (ret < 0) {
			dev_err(opt->dev, "failed to write register %02x\n",
					OPT3001_LOW_LIMIT);
			return ret;
		}
	}

	exponent = OPT3001_REG_EXPONENT(opt->result);
	mantissa = OPT3001_REG_MANTISSA(opt->result);

	opt3001_to_iio_ret(opt, exponent, mantissa, val, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

void opt3001_get_cal_data(struct iio_dev *iio)
{
	struct opt3001 *als_data = iio_priv(iio);
	int als_lux_msb = 0;
	int als_lux_lsb = 0;
	int cal_lux = 0;

	if(als_data->pdata->enabled)
		hrtimer_cancel(&als_data->als_timer);

	mutex_lock(&als_data->io_lock);
	opt3001_get_lux(als_data, &als_lux_lsb, &als_lux_msb);
	cal_lux = (als_lux_lsb * 100) + (als_lux_msb / 10000);
	als_data->setting.caliberated_value = cal_lux;
	mutex_unlock(&als_data->io_lock);
	if(als_data->pdata->enabled)
		hrtimer_start(&als_data->als_timer,
				ns_to_ktime(als_data->pdata->poll_delay * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
}

int opt3001_get_pure_lux(struct iio_dev *iio)
{
	struct opt3001 *ps_data = iio_priv(iio);
        int als_lux_msb = 0;
        int als_lux_lsb = 0;
        int actual_lux = 0;
        int als_lux = 0;

        mutex_lock(&ps_data->io_lock);
        opt3001_get_lux(ps_data, &als_lux_lsb, &als_lux_msb);
        actual_lux = (als_lux_lsb * 100) + (als_lux_msb / 10000);
        /* Caliberated lux = ((Actual Lux * 600) / Lux value received in 600 lux) */
        if(ps_data->setting.caliberated_value)
                als_lux = ((actual_lux * 600) / ps_data->setting.caliberated_value);
        else
                als_lux = actual_lux;

        mutex_unlock(&ps_data->io_lock);

	return als_lux;
}

static int opt3001_get_int_time(struct opt3001 *opt, int *val, int *val2)
{
	*val = 0;
	*val2 = opt->int_time;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int opt3001_set_int_time(struct opt3001 *opt, int time)
{
	int ret;
	u16 reg;

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_CONFIGURATION);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_CONFIGURATION);
		return ret;
	}

	reg = ret;

	switch (time) {
		case OPT3001_INT_TIME_SHORT:
			reg &= ~OPT3001_CONFIGURATION_CT;
			opt->int_time = OPT3001_INT_TIME_SHORT;
			break;
		case OPT3001_INT_TIME_LONG:
			reg |= OPT3001_CONFIGURATION_CT;
			opt->int_time = OPT3001_INT_TIME_LONG;
			break;
		default:
			return -EINVAL;
	}

	return i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION,
			reg);
}

static int opt3001_read_raw(struct iio_dev *iio,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	struct opt3001 *opt = iio_priv(iio);
	int ret;

	if (opt->mode == OPT3001_CONFIGURATION_M_CONTINUOUS)
		return -EBUSY;

	if (chan->type != IIO_LIGHT)
		return -EINVAL;

	mutex_lock(&opt->lock);

	switch (mask) {
		case IIO_CHAN_INFO_PROCESSED:
			ret = opt3001_get_lux(opt, val, val2);
			break;
		case IIO_CHAN_INFO_INT_TIME:
			ret = opt3001_get_int_time(opt, val, val2);
			break;
		default:
			ret = -EINVAL;
	}

	mutex_unlock(&opt->lock);

	return ret;
}

static int opt3001_write_raw(struct iio_dev *iio,
		struct iio_chan_spec const *chan, int val, int val2,
		long mask)
{
	struct opt3001 *opt = iio_priv(iio);
	int ret;

	if (opt->mode == OPT3001_CONFIGURATION_M_CONTINUOUS)
		return -EBUSY;

	if (chan->type != IIO_LIGHT)
		return -EINVAL;

	if (mask != IIO_CHAN_INFO_INT_TIME)
		return -EINVAL;

	if (val != 0)
		return -EINVAL;

	mutex_lock(&opt->lock);
	ret = opt3001_set_int_time(opt, val2);
	mutex_unlock(&opt->lock);

	return ret;
}

static int opt3001_read_event_value(struct iio_dev *iio,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int *val, int *val2)
{
	struct opt3001 *opt = iio_priv(iio);
	int ret = IIO_VAL_INT_PLUS_MICRO;

	mutex_lock(&opt->lock);

	switch (dir) {
		case IIO_EV_DIR_RISING:
			opt3001_to_iio_ret(opt, opt->high_thresh_exp,
					opt->high_thresh_mantissa, val, val2);
			break;
		case IIO_EV_DIR_FALLING:
			opt3001_to_iio_ret(opt, opt->low_thresh_exp,
					opt->low_thresh_mantissa, val, val2);
			break;
		default:
			ret = -EINVAL;
	}

	mutex_unlock(&opt->lock);

	return ret;
}

static int opt3001_write_event_value(struct iio_dev *iio,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int val, int val2)
{
	struct opt3001 *opt = iio_priv(iio);
	int ret;

	u16 mantissa;
	u16 value;
	u16 reg;

	u8 exponent;

	if (val < 0)
		return -EINVAL;

	mutex_lock(&opt->lock);

	ret = opt3001_find_scale(opt, val, val2, &exponent);
	if (ret < 0) {
		dev_err(opt->dev, "can't find scale for %d.%06u\n", val, val2);
		goto err;
	}

	mantissa = (((val * 1000) + (val2 / 1000)) / 10) >> exponent;
	value = (exponent << 12) | mantissa;

	switch (dir) {
		case IIO_EV_DIR_RISING:
			reg = OPT3001_HIGH_LIMIT;
			opt->high_thresh_mantissa = mantissa;
			opt->high_thresh_exp = exponent;
			break;
		case IIO_EV_DIR_FALLING:
			reg = OPT3001_LOW_LIMIT;
			opt->low_thresh_mantissa = mantissa;
			opt->low_thresh_exp = exponent;
			break;
		default:
			ret = -EINVAL;
			goto err;
	}

	ret = i2c_smbus_write_word_swapped(opt->client, reg, value);
	if (ret < 0) {
		dev_err(opt->dev, "failed to write register %02x\n", reg);
		goto err;
	}

err:
	mutex_unlock(&opt->lock);

	return ret;
}

static int opt3001_read_event_config(struct iio_dev *iio,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir)
{
	struct opt3001 *opt = iio_priv(iio);

	return opt->mode == OPT3001_CONFIGURATION_M_CONTINUOUS;
}

static int opt3001_write_event_config(struct iio_dev *iio,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, int state)
{
	struct opt3001 *opt = iio_priv(iio);
	int ret;
	u16 mode;
	u16 reg;

	if (state && opt->mode == OPT3001_CONFIGURATION_M_CONTINUOUS)
		return 0;

	if (!state && opt->mode == OPT3001_CONFIGURATION_M_SHUTDOWN)
		return 0;

	mutex_lock(&opt->lock);

	mode = state ? OPT3001_CONFIGURATION_M_CONTINUOUS
		: OPT3001_CONFIGURATION_M_SHUTDOWN;

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_CONFIGURATION);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_CONFIGURATION);
		goto err;
	}

	reg = ret;
	opt3001_set_mode(opt, &reg, mode);

	ret = i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION,
			reg);
	if (ret < 0) {
		dev_err(opt->dev, "failed to write register %02x\n",
				OPT3001_CONFIGURATION);
		goto err;
	}

err:
	mutex_unlock(&opt->lock);

	return ret;
}

static const struct iio_info opt3001_info = {
	.driver_module = THIS_MODULE,
	.attrs = &opt3001_attribute_group,
	.read_raw = opt3001_read_raw,
	.write_raw = opt3001_write_raw,
	.read_event_value = opt3001_read_event_value,
	.write_event_value = opt3001_write_event_value,
	.read_event_config = opt3001_read_event_config,
	.write_event_config = opt3001_write_event_config,
};

static int opt3001_read_id(struct opt3001 *opt)
{
	char manufacturer[2];
	u16 device_id;
	int ret;

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_MANUFACTURER_ID);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_MANUFACTURER_ID);
		return ret;
	}

	manufacturer[0] = ret >> 8;
	manufacturer[1] = ret & 0xff;

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_DEVICE_ID);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_DEVICE_ID);
		return ret;
	}

	device_id = ret;

	dev_info(opt->dev, "Found %c%c OPT%04x\n", manufacturer[0],
			manufacturer[1], device_id);

	return 0;
}

static int opt3001_configure(struct opt3001 *opt)
{
	int ret;
	u16 reg;

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_CONFIGURATION);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_CONFIGURATION);
		return ret;
	}

	reg = ret;

	/* Enable automatic full-scale setting mode */
	reg &= ~OPT3001_CONFIGURATION_RN_MASK;
	reg &= ~OPT3001_CONFIGURATION_CT;
	reg |= OPT3001_CONFIGURATION_RN_AUTO;

	/* Reflect status of the device's integration time setting */
	if (reg & OPT3001_CONFIGURATION_CT)
		opt->int_time = OPT3001_INT_TIME_LONG;
	else
		opt->int_time = OPT3001_INT_TIME_SHORT;

	/* Ensure device is in shutdown initially */
	opt3001_set_mode(opt, &reg, OPT3001_CONFIGURATION_M_SHUTDOWN);

	/* Configure for latched window-style comparison operation */
	reg |= OPT3001_CONFIGURATION_L;
	reg &= ~OPT3001_CONFIGURATION_POL;
	reg &= ~OPT3001_CONFIGURATION_ME;
	reg &= ~OPT3001_CONFIGURATION_FC_MASK;

	ret = i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION,
			reg);
	if (ret < 0) {
		dev_err(opt->dev, "failed to write register %02x\n",
				OPT3001_CONFIGURATION);
		return ret;
	}

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_LOW_LIMIT);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_LOW_LIMIT);
		return ret;
	}

	opt->low_thresh_mantissa = OPT3001_REG_MANTISSA(ret);
	opt->low_thresh_exp = OPT3001_REG_EXPONENT(ret);

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_HIGH_LIMIT);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_HIGH_LIMIT);
		return ret;
	}

	opt->high_thresh_mantissa = OPT3001_REG_MANTISSA(ret);
	opt->high_thresh_exp = OPT3001_REG_EXPONENT(ret);

	return 0;
}

static irqreturn_t opt3001_irq(int irq, void *_iio)
{
	struct iio_dev *iio = _iio;
	struct opt3001 *opt = iio_priv(iio);
	int ret;

	if (!opt->ok_to_ignore_lock)
		mutex_lock(&opt->lock);

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_CONFIGURATION);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_CONFIGURATION);
		goto out;
	}

	if ((ret & OPT3001_CONFIGURATION_M_MASK) ==
			OPT3001_CONFIGURATION_M_CONTINUOUS) {
		if (ret & OPT3001_CONFIGURATION_FH)
			iio_push_event(iio,
					IIO_UNMOD_EVENT_CODE(IIO_LIGHT, 0,
						IIO_EV_TYPE_THRESH,
						IIO_EV_DIR_RISING),
					iio_get_time_ns());
		if (ret & OPT3001_CONFIGURATION_FL)
			iio_push_event(iio,
					IIO_UNMOD_EVENT_CODE(IIO_LIGHT, 0,
						IIO_EV_TYPE_THRESH,
						IIO_EV_DIR_FALLING),
					iio_get_time_ns());
	} else if (ret & OPT3001_CONFIGURATION_CRF) {
		ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_RESULT);
		if (ret < 0) {
			dev_err(opt->dev, "failed to read register %02x\n",
					OPT3001_RESULT);
			goto out;
		}
		opt->result = ret;
		opt->result_ready = true;
		wake_up(&opt->result_ready_queue);
	}

out:
	if (!opt->ok_to_ignore_lock)
		mutex_unlock(&opt->lock);

	return IRQ_HANDLED;
}

static enum hrtimer_restart opt3001_als_timer_func(struct hrtimer *timer)
{
        struct opt3001 *ps_data = container_of(timer, struct opt3001, als_timer);
        queue_work(ps_data->als_wq, &ps_data->als_work);
        hrtimer_forward_now(&ps_data->als_timer, ns_to_ktime(ps_data->pdata->poll_delay * NSEC_PER_MSEC));
        return HRTIMER_RESTART;
}

static void opt3001_als_work_func(struct work_struct *work)
{
	struct opt3001 *ps_data = container_of(work, struct opt3001, als_work);
	int als_lux_msb = 0;
	int als_lux_lsb = 0;
	int actual_lux = 0;
	int als_lux = 0;
	ktime_t timestamp;

	timestamp = ktime_get_boottime();

	mutex_lock(&ps_data->io_lock);
	opt3001_get_lux(ps_data, &als_lux_lsb, &als_lux_msb);
	actual_lux = (als_lux_lsb * 100) + (als_lux_msb / 10000);
	/* Caliberated lux = ((Actual Lux * 600) / Lux value received in 600 lux) */
	if(ps_data->setting.caliberated_value)
		als_lux = ((actual_lux * 600) / ps_data->setting.caliberated_value);
	else
		als_lux = actual_lux;

	input_report_abs(ps_data->ls_input_dev, ABS_MISC, als_lux);
	input_event(ps_data->ls_input_dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(timestamp).tv_sec);
	input_event(ps_data->ls_input_dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(timestamp).tv_nsec);
	input_sync(ps_data->ls_input_dev);
	mutex_unlock(&ps_data->io_lock);
}


static int opt3001_als_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct opt3001 *als_data = container_of(sensors_cdev,
			struct opt3001, als_cdev);

	if ((delay_msec < OPT3001_LS_MIN_POLL_DELAY) ||
			(delay_msec > OPT3001_LS_MAX_POLL_DELAY))
		return -EINVAL;
	als_data->pdata->poll_delay = delay_msec;

	return 0;
}


static int opt3001_als_flush(struct sensors_classdev *sensors_cdev)
{
	struct opt3001 *als_data = container_of(sensors_cdev,
			struct opt3001, als_cdev);

        input_event(als_data->ls_input_dev, EV_SYN, SYN_CONFIG,
                                                als_data->flush_count++);
        input_sync(als_data->ls_input_dev);

        return 0;
}

static int opt3001_als_get_cal_data(struct opt3001 *als_data)
{
        struct file *filp;
        char read_data[8];
        unsigned int read_size = 0;
        int value;
        mm_segment_t fs;

        fs = get_fs();
        set_fs(get_ds());
        filp=filp_open(CAL_FILE, O_RDONLY, 0);
        if(IS_ERR(filp)) {
                pr_err("Failed to open %s\n", CAL_FILE);
                return -EIO;
        }

        memset(read_data, 0, sizeof(read_data));

        read_size = filp->f_op->read(filp, read_data, 8, &filp->f_pos);
        if (kstrtoint(read_data, 0, &value))
                return -EINVAL;

        if (value)
                als_data->setting.caliberated_value = value;

        set_fs(fs);
        filp_close(filp, NULL);

        return 0;
}

static int opt3001_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct opt3001 *als_data = container_of(sensors_cdev,
			struct opt3001, als_cdev);
	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	if(!als_data->cal_status) {
		/* Read Caliberated data and set */
		if(opt3001_als_get_cal_data(als_data))
			pr_err("lux calibration failed\n");

		als_data->cal_status = true;
	}

	if (enable) {
		als_data->pdata->enabled = true;
		hrtimer_start(&als_data->als_timer, ns_to_ktime(als_data->pdata->poll_delay * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	} else {
		als_data->pdata->enabled = false;
		hrtimer_cancel(&als_data->als_timer);
	}

	return 0;
}


static int lightsensor_setup(struct opt3001 *opt)
{
        int ret;

        opt->ls_input_dev = devm_input_allocate_device(&opt->client->dev);
        if (!opt->ls_input_dev) {
                pr_err(
                        "%s: could not allocate ls input device\n",
                        __func__);
                return -ENOMEM;
        }
        opt->ls_input_dev->name = "opt3001-light";
        opt->ls_input_dev->id.bustype = BUS_I2C;
        set_bit(EV_ABS, opt->ls_input_dev->evbit);

        input_set_abs_params(opt->ls_input_dev, ABS_MISC, 0, 65535, 0, 0);

        ret = input_register_device(opt->ls_input_dev);
        if (ret < 0) {
                pr_err("%s: can not register ls input device\n",
                                __func__);
                goto err_free_ls_input_device;
        }

        input_set_drvdata(opt->ls_input_dev, opt);

err_free_ls_input_device:
        return ret;
}

static int opt3001_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;

	struct iio_dev *iio;
	struct opt3001 *opt;
	struct opt3001_info *plat_data;
	int irq = client->irq;
	int ret;

	iio = devm_iio_device_alloc(dev, sizeof(*opt));
	if (!iio)
		return -ENOMEM;

	opt = iio_priv(iio);
	opt->client = client;
	opt->dev = dev;

	mutex_init(&opt->lock);
	init_waitqueue_head(&opt->result_ready_queue);
	i2c_set_clientdata(client, iio);

	ret = opt3001_read_id(opt);
	if (ret)
		return ret;

	ret = opt3001_configure(opt);
	if (ret)
		return ret;

	iio->name = client->name;
	iio->channels = opt3001_channels;
	iio->num_channels = ARRAY_SIZE(opt3001_channels);
	iio->dev.parent = dev;
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &opt3001_info;

	ret = devm_iio_device_register(dev, iio);
	if (ret) {
		dev_err(dev, "failed to register IIO device\n");
		return ret;
	}

	/* Make use of INT pin only if valid IRQ no. is given */
	if (irq > 0) {
		ret = request_threaded_irq(irq, NULL, opt3001_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"opt3001", iio);
		if (ret) {
			dev_err(dev, "failed to request IRQ #%d\n", irq);
			return ret;
		}
		opt->use_irq = true;
	} else {
		dev_dbg(opt->dev, "enabling interrupt-less operation\n");
	}

	plat_data = devm_kzalloc(dev,
			sizeof(struct opt3001_info), GFP_KERNEL);
	if (!plat_data) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	plat_data->enabled = 0;
	plat_data->poll_delay = OPT3001_LS_DEFAULT_POLL_DELAY;
	opt->pdata = plat_data;

	mutex_init(&opt->io_lock);

	ret = lightsensor_setup(opt);
	if (ret < 0) {
		pr_err("%s: lightsensor_setup error!!\n", __func__);
		return -1;
	}


	opt->setting.caliberated_value = 0;
	opt->als_cdev = sensors_light_cdev;
	opt->als_cdev.sensors_enable = opt3001_als_set_enable;
	opt->als_cdev.sensors_poll_delay = opt3001_als_poll_delay_set;
	opt->als_cdev.sensors_flush = opt3001_als_flush;
	opt->als_wq = create_singlethread_workqueue("als_wq");
	INIT_WORK(&opt->als_work, opt3001_als_work_func);
	hrtimer_init(&opt->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	opt->als_timer.function = opt3001_als_timer_func;

	ret = sensors_classdev_register(&opt->ls_input_dev->dev,
			&opt->als_cdev);
	if (ret) {
		pr_err("%s: ERROR: Failed to register sensor class\n", __func__);
		return -1;
	}

	dev_info(opt->dev, "OPT3001 Light sensor driver probe: Success ");
	return 0;
}

static int opt3001_remove(struct i2c_client *client)
{
	struct iio_dev *iio = i2c_get_clientdata(client);
	struct opt3001 *opt = iio_priv(iio);
	int ret;
	u16 reg;

	if (opt->use_irq)
		free_irq(client->irq, iio);

	ret = i2c_smbus_read_word_swapped(opt->client, OPT3001_CONFIGURATION);
	if (ret < 0) {
		dev_err(opt->dev, "failed to read register %02x\n",
				OPT3001_CONFIGURATION);
		return ret;
	}

	reg = ret;
	opt3001_set_mode(opt, &reg, OPT3001_CONFIGURATION_M_SHUTDOWN);

	ret = i2c_smbus_write_word_swapped(opt->client, OPT3001_CONFIGURATION,
			reg);
	if (ret < 0) {
		dev_err(opt->dev, "failed to write register %02x\n",
				OPT3001_CONFIGURATION);
		return ret;
	}

	return 0;
}

static const struct i2c_device_id opt3001_id[] = {
	{ "opt3001", 0 },
	{ } /* Terminating Entry */
};
MODULE_DEVICE_TABLE(i2c, opt3001_id);

static const struct of_device_id opt3001_of_match[] = {
	{ .compatible = "ti,opt3001" },
	{ }
};
MODULE_DEVICE_TABLE(of, opt3001_of_match);

static struct i2c_driver opt3001_driver = {
	.probe = opt3001_probe,
	.remove = opt3001_remove,
	.id_table = opt3001_id,

	.driver = {
		.name = "opt3001",
		.of_match_table = of_match_ptr(opt3001_of_match),
	},
};

module_i2c_driver(opt3001_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andreas Dannenberg <dannenberg@ti.com>");
MODULE_DESCRIPTION("Texas Instruments OPT3001 Light Sensor Driver");
