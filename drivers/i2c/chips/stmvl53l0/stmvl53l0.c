/* stmvl53l0.c - driver file for Laser Sensor
 *
 * Copyright (C) 2014 HTC Ltd.
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
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/vibtrig.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/irq_work.h>

#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>

#include <linux/of_gpio.h>

#include "inc/stmvl53l0.h"
#include "inc/vl53l0_api.h"

#define D(x...) pr_debug("[Laser] " x)
#define I(x...) pr_info("[Laser] " x)
#define W(x...) pr_warn("[Laser] " x)
#define E(x...) pr_err("[Laser] " x)

#define VL53L0_MODEL_ID        0xEE
#define VL53L0_MODULE_ID_1_1   0x10

#define VL53L0_MAGIC 'A'
#define VL53L0_IOCTL_GET_DATA                  _IOR(VL53L0_MAGIC,      0x01, VL53L0_RangingMeasurementData_t)
#define VL53L0_IOCTL_OFFSET_CALI               _IOR(VL53L0_MAGIC,      0x02, int)
#define VL53L0_IOCTL_XTALK_CALI                _IOR(VL53L0_MAGIC,      0x03, int)
#define VL53L0_IOCTL_SET_XTALK_CALI_DISTANCE   _IOR(VL53L0_MAGIC,      0x04, int)

#ifndef ABS
#define ABS(x)              (((x) > 0) ? (x) : (-(x)))
#endif

#define CALIBRATION_DATA_PATH "/calibration_data"
#define SENSOR_FLASH_DATA "gyro_flash"

#define OFFSET_CALI_TARGET_DISTANCE   100 // Target: 100 mm
#define RANGE_MEASUREMENT_TIMES        10
#define RANGE_MEASUREMENT_RETRY_TIMES  13
#define RANGE_MEASUREMENT_OVERFLOW     8100

#define POLLING_DELAY_MIN 10
#define POLLING_DELAY_MAX 1000
#define REPORT_EVENT_COMMON_LEN 5

struct laser_device_data {
	u32 pwdn_gpio;
	u32 power_gpio;
	struct regulator *camio_1v8;
	struct class *laser_class;
	struct device *laser_dev;
	struct device *sensor_dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_init;
	bool laser_power;
	FixPoint1616_t cali_distance;
	int offset_kvalue;
	FixPoint1616_t xtalk_kvalue;
	u8 cali_status;

	struct iio_trigger	*trig;
	atomic_t		pseudo_irq_enable;
	s32			iio_data[6];
	struct iio_dev		*indio_dev;
	struct irq_work		iio_irq_work;
	struct delayed_work	work;
	struct work_struct	one_shot_work;
	struct workqueue_struct *laser_wq;
	atomic_t delay;
	struct mutex lock;

	bool	enabled;
	u32	pending_flush;

	bool	wk_report_meta;
};

struct laser_device_data *laser_data;
static struct i2c_client *g_pstLaser_I2Cclient = NULL;
VL53L0_Dev_t MyDevice;



int Laser_RegWriteByte(u8 addr, u8 data)
{
	int ret = 0;
	u8 SendCmd[2] = {	(u8)( addr       &0xFF),
				(u8)( data       &0xFF)};

	ret = i2c_master_send(g_pstLaser_I2Cclient, SendCmd , 2);
	if (ret < 0)
	{
		E("%s: I2C write data failed (%d)!!\n", __func__, ret);
		return -1;
	}

	return 0;
}

int Laser_RegReadByte(u8 addr, u8 *data)
{
	int ret = 0;
	u8 SendCmd = (u8)(addr & 0xFF);

	ret = i2c_master_send(g_pstLaser_I2Cclient, &SendCmd, 1);
	if (ret < 0)
	{
		E("%s: I2C send addr failed (%d)!!\n", __func__, ret);
		return -1;
	}

	ret = i2c_master_recv(g_pstLaser_I2Cclient, data , 1);
	if (ret < 0)
	{
		E("%s: I2C read data failed (%d)!!\n", __func__, ret);
		return -1;
	}

	return 0;
}

int Laser_RegWriteWord(u8 addr, u16 data)
{
	int ret = 0;
	u8 SendCmd[3] = {	(u8)( addr       &0xFF),
				(u8)((data >>  8)&0xFF),
				(u8)( data       &0xFF)};

	ret = i2c_master_send(g_pstLaser_I2Cclient, SendCmd , 3);
	if (ret < 0)
	{
		E("%s: I2C write data failed (%d)!!\n", __func__, ret);
		return -1;
	}

	return 0;
}

int Laser_RegReadWord(u8 addr, u16 *data)
{
	int ret = 0;
	u8 SendCmd = (u8)(addr & 0xFF);
	u16 vData = 0;

	ret = i2c_master_send(g_pstLaser_I2Cclient, &SendCmd, 1);
	if (ret < 0)
	{
		E("%s: I2C send addr failed (%d)!!\n", __func__, ret);
		return -1;
	}

	ret = i2c_master_recv(g_pstLaser_I2Cclient, (char *)data , 2);
	if (ret < 0)
	{
		E("%s: I2C read data failed (%d)!!\n", __func__, ret);
		return -1;
	}

	vData = *data;
	*data = ( (vData & 0xFF) << 8 ) + ( (vData >> 8) & 0xFF );

	return 0;
}

int Laser_RegWriteDWord(u8 addr, u32 data)
{
	int ret = 0;
	u8 SendCmd[5] = {	(u8)( addr       &0xFF),
				(u8)((data >> 24)&0xFF),
				(u8)((data >> 16)&0xFF),
				(u8)((data >>  8)&0xFF),
				(u8)( data       &0xFF)};

	ret = i2c_master_send(g_pstLaser_I2Cclient, SendCmd , 5);
	if (ret < 0)
	{
		E("%s: I2C write data failed (%d)!!\n", __func__, ret);
		return -1;
	}

	return 0;
}

int Laser_RegReadDWord(u8 addr, u32 *data)
{
	int ret = 0;
	u8 SendCmd = (u8)(addr & 0xFF);
	u32 vData = 0;

	ret = i2c_master_send(g_pstLaser_I2Cclient, &SendCmd, 1);
	if (ret < 0)
	{
		E("%s: I2C send addr failed (%d)!!\n", __func__, ret);
		return -1;
	}

	ret = i2c_master_recv(g_pstLaser_I2Cclient, (char *)data , 4);
	if (ret < 0)
	{
		E("%s: I2C read data failed (%d)!!\n", __func__, ret);
		return -1;
	}

	vData = *data;
	*data= (	(vData        &0xFF) <<24) +
			(((vData>> 8)&0xFF) <<16) +
			(((vData>>16)&0xFF) << 8) +
			(((vData>>24)&0xFF) );

	return 0;
}

int Laser_RegReadMulti(u8 addr, u8 *data, uint32_t count)
{
	int ret = 0;
	u8 SendCmd = (u8)(addr & 0xFF);

	ret = i2c_master_send(g_pstLaser_I2Cclient, &SendCmd, 1);
	if (ret < 0)
	{
		E("%s: I2C send addr failed (%d)!!\n", __func__, ret);
		return -1;
	}

	ret = i2c_master_recv(g_pstLaser_I2Cclient, (char *)data , count);
	if (ret < 0)
	{
		E("%s: I2C read data failed (%d)!!\n", __func__, ret);
		return -1;
	}

	return 0;
}

static int Laser_poweron(void)
{
	int ret = 0;
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	ret = regulator_enable(laser_data->camio_1v8);
	if (ret) {
		E("%s: Failed to enable CAMIO_1v8\n", __func__);
		return ret;
	}

	ret = gpio_direction_output(laser_data->power_gpio, 1);
	if (ret) {
		E("%s: Failed to pull up power_gpio\n", __func__);
		return ret;
	}
	msleep(1);

	ret = gpio_direction_output(laser_data->pwdn_gpio, 1);
	if (ret) {
		E("%s: Failed to pull up pwdn_gpio\n", __func__);
		return ret;
	}
	msleep(2);

	// data initialization / device initializtion
	Status = VL53L0_DataInit(&MyDevice);
	Status |= VL53L0_StaticInit(&MyDevice);
	if (Status != VL53L0_ERROR_NONE) {
		E("%s: Initialization failed with Status = %d\n", __func__, Status);
		return Status;
	}

	// Set calibration data to register
	if (laser_data->offset_kvalue != 0) {
		Status = VL53L0_SetOffsetCalibrationDataMicroMeter(&MyDevice, (laser_data->offset_kvalue*1000));
		if (Status != VL53L0_ERROR_NONE) {
			E("%s: Failed to set offset calibration data to register\n", __func__);
		}
	}

	if (laser_data->xtalk_kvalue != 0) {
		Status = VL53L0_SetXTalkCompensationRateMegaCps(&MyDevice, laser_data->xtalk_kvalue);

		if (Status != VL53L0_ERROR_NONE) {
			E("%s: Failed to set xtalk calibration data to register\n", __func__);
		}
	}

	// Enable xtalk compensation
	Status = VL53L0_SetXTalkCompensationEnable(&MyDevice, 1);
	if (Status != VL53L0_ERROR_NONE) {
		E("%s: Failed to enable xtalk compensation\n", __func__);
	}

	laser_data->laser_power = 1;

	return ret;
}

static int Laser_poweron_without_init(void)
{
	int ret = 0;

	ret = regulator_enable(laser_data->camio_1v8);
	if (ret) {
		E("%s: Failed to enable CAMIO_1v8\n", __func__);
		return ret;
	}

	ret = gpio_direction_output(laser_data->power_gpio, 1);
	if (ret) {
		E("%s: Failed to pull up power_gpio\n", __func__);
		return ret;
	}
	msleep(1);

	ret = gpio_direction_output(laser_data->pwdn_gpio, 1);
	if (ret) {
		E("%s: Failed to pull up pwdn_gpio\n", __func__);
		return ret;
	}
	msleep(2);

	laser_data->laser_power = 1;

	return ret;
}

static int Laser_poweroff(void)
{
	int ret = 0;

	ret = gpio_direction_output(laser_data->power_gpio, 0);
	if (ret) {
		E("%s: Failed to pull down power_gpio\n", __func__);
		return ret;
	}
	msleep(1);

	ret = gpio_direction_output(laser_data->pwdn_gpio, 0);
	if (ret) {
		E("%s: Failed to pull down pwdn_gpio\n", __func__);
		return ret;
	}

	ret = regulator_disable(laser_data->camio_1v8);
	if (ret) {
		E("%s: Failed to disable CAMIO_1v8\n", __func__);
		return ret;
	}

	laser_data->laser_power = 0;

	return ret;
}

static int Laser_pinctrl_init(void)
{
	int retval = 0;
	int ret = 0;

	I("Laser_pinctrl_init\n");

	laser_data->pinctrl = devm_pinctrl_get(laser_data->sensor_dev);
	if (IS_ERR_OR_NULL(laser_data->pinctrl)) {
		E("Target does not use pinctrl\n");
		retval = PTR_ERR(laser_data->pinctrl);
		laser_data->pinctrl = NULL;
		return retval;
	}

	laser_data->gpio_state_init = pinctrl_lookup_state(laser_data->pinctrl, "laser_gpio_init");
	if (IS_ERR_OR_NULL(laser_data->gpio_state_init)) {
		E("Can not get ts default pinstate\n");
		retval = PTR_ERR(laser_data->gpio_state_init);
		laser_data->pinctrl = NULL;
		return retval;
	}

	ret = pinctrl_select_state(laser_data->pinctrl, laser_data->gpio_state_init);
	if (ret) {
		E("can not init gpio\n");
		return ret;
	}

	return 0;
}

static int laser_send_event(struct laser_device_data *laser_data, u8 id, u32 *data,
			    s64 timestamp)
{
	u8 event[21];/* Sensor HAL uses fixed 21 bytes */
	u16 range[2];
	u32 rate[2];
	u8 status;

	range[0] = (u16) data[0];
	range[1] = (u16) data[1];
	rate[0] = data[2];
	rate[1] = data[3];
	status = (u8) data[4];

	event[0] = id;
	memcpy(&event[1], range, sizeof(u16)*2);
	memcpy(&event[5], rate, sizeof(u32)*2);
	memcpy(&event[13], &status, sizeof(u8));
	memset(&event[14], 0, sizeof(u8)*7);

	D(
	  "%s: active_scan_mask = 0x%p, masklength = %u, data(x, y, z) ="
	  "(%d, %d, %d)\n",
	  __func__, laser_data->indio_dev->active_scan_mask,
	  laser_data->indio_dev->masklength,
	  *(s16 *)&event[1], *(s16 *)&event[3], *(s16 *)&event[5]);

	if (laser_data->indio_dev && laser_data->indio_dev->active_scan_mask &&
	    (!bitmap_empty(laser_data->indio_dev->active_scan_mask,
			   laser_data->indio_dev->masklength))) {
		mutex_lock(&laser_data->lock);
		if (atomic_read(&laser_data->pseudo_irq_enable))
			iio_push_to_buffers(laser_data->indio_dev, event);
		else {
			D(
			  "%s: Drop data(0, 1, 2, 3) = "
			  "(0x%x, 0x%x, 0x%x, 0x%x)\n", __func__,
			  data[0], data[1], data[2], data[3]);
		}
		mutex_unlock(&laser_data->lock);

		if (!laser_data->indio_dev)
			E("%s: laser_data->indio_dev == NULL!!\n", __func__);

		return 0;
	} else
		D("%s: Event might be missing\n", __func__);

	return -EIO;
}

static void report_laser(struct laser_device_data *laser_data)
{
	u32 data[REPORT_EVENT_COMMON_LEN] = {0};
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	VL53L0_RangingMeasurementData_t RangingMeasurementData;

	if (laser_data->enabled) {
		Status = VL53L0_PerformSingleRangingMeasurement(&MyDevice, &RangingMeasurementData);
		if (Status == VL53L0_ERROR_NONE) {
			/*I("RangeMilliMeter = %d,  SignalRateRtnMegaCps = 0x%x,  RangeStatus = %d\n",
				RangingMeasurementData.RangeMilliMeter,
				RangingMeasurementData.SignalRateRtnMegaCps,
				RangingMeasurementData.RangeStatus);*/

			data[0] = RangingMeasurementData.RangeMilliMeter;
			data[1] = RangingMeasurementData.RangeDMaxMilliMeter;
			data[2] = RangingMeasurementData.SignalRateRtnMegaCps;
			data[3] = RangingMeasurementData.AmbientRateRtnMegaCps;
			data[4] = RangingMeasurementData.RangeStatus;

			laser_send_event(laser_data, LASER_RANGE_DATA, data, 0);
		}
	}
}

/*=======iio device reg=========*/
static void iio_trigger_work(struct irq_work *work)
{
	struct laser_device_data *laser_data = container_of(
					(struct irq_work *)work,
					struct laser_device_data, iio_irq_work);

	iio_trigger_poll(laser_data->trig);
}

static irqreturn_t laser_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct laser_device_data *laser_data = iio_priv(indio_dev);

	report_laser(laser_data);

	mutex_lock(&laser_data->lock);
	iio_trigger_notify_done(laser_data->indio_dev->trig);
	mutex_unlock(&laser_data->lock);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops laser_buffer_setup_ops = {
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static int laser_pseudo_irq_enable(struct iio_dev *indio_dev)
{
	struct laser_device_data *laser_data = iio_priv(indio_dev);

	if (!atomic_cmpxchg(&laser_data->pseudo_irq_enable, 0, 1)) {
		I("%s:\n", __func__);
		cancel_delayed_work_sync(&laser_data->work);
		queue_delayed_work(laser_data->laser_wq, &laser_data->work, 0);
	}

	return 0;
}

static int laser_pseudo_irq_disable(struct iio_dev *indio_dev)
{
	struct laser_device_data *laser_data = iio_priv(indio_dev);

	if (atomic_cmpxchg(&laser_data->pseudo_irq_enable, 1, 0)) {
		cancel_delayed_work_sync(&laser_data->work);
		I("%s:\n", __func__);
	}
	return 0;
}

static int laser_set_pseudo_irq(struct iio_dev *indio_dev, int enable)
{
	if (enable)
		laser_pseudo_irq_enable(indio_dev);
	else
		laser_pseudo_irq_disable(indio_dev);

	return 0;
}

static int laser_data_rdy_trigger_set_state(struct iio_trigger *trig,
					    bool state)
{
	struct iio_dev *indio_dev =
			(struct iio_dev *)iio_trigger_get_drvdata(trig);
	struct laser_device_data *laser_data = iio_priv(indio_dev);

	mutex_lock(&laser_data->lock);
	laser_set_pseudo_irq(indio_dev, state);
	mutex_unlock(&laser_data->lock);

	return 0;
}

static const struct iio_trigger_ops laser_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &laser_data_rdy_trigger_set_state,
};

static int laser_probe_trigger(struct iio_dev *iio_dev)
{
	struct laser_device_data *laser_data = iio_priv(iio_dev);
	int ret;

	iio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time,
			&laser_trigger_handler, IRQF_ONESHOT, iio_dev,
			"%s_consumer%d", iio_dev->name, iio_dev->id);
	if (iio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	laser_data->trig = iio_trigger_alloc("%s-dev%d",
			iio_dev->name,
			iio_dev->id);
	if (!laser_data->trig) {
		ret = -ENOMEM;
		goto error_dealloc_pollfunc;
	}
	laser_data->trig->dev.parent = &g_pstLaser_I2Cclient->dev;
	laser_data->trig->ops = &laser_trigger_ops;
	iio_trigger_set_drvdata(laser_data->trig, iio_dev);

	ret = iio_trigger_register(laser_data->trig);
	if (ret)
		goto error_free_trig;

	return 0;

error_free_trig:
	iio_trigger_free(laser_data->trig);
error_dealloc_pollfunc:
	iio_dealloc_pollfunc(iio_dev->pollfunc);
error_ret:
	return ret;
}

static int laser_probe_buffer(struct iio_dev *iio_dev)
{
	int ret;
	struct iio_buffer *buffer;

	buffer = iio_kfifo_allocate(iio_dev);
	if (!buffer) {
		ret = -ENOMEM;
		goto error_ret;
	}

	buffer->scan_timestamp = true;
	iio_dev->buffer = buffer;
	iio_dev->setup_ops = &laser_buffer_setup_ops;
	iio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_buffer_register(iio_dev, iio_dev->channels,
				  iio_dev->num_channels);
	if (ret)
		goto error_free_buf;

	iio_scan_mask_set(iio_dev, iio_dev->buffer, LASER_SCAN_ID);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, LASER_SCAN_X);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, LASER_SCAN_Y);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, LASER_SCAN_Z);
	return 0;

error_free_buf:
	iio_kfifo_free(iio_dev->buffer);
error_ret:
	return ret;
}


static int laser_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val,
			  int *val2,
			  long mask) {
	struct laser_device_data *laser_data = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = laser_data->iio_data[chan->channel2 - IIO_MOD_X];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		/* Gain : counts / uT = 1000 [nT] */
		/* Scaling factor : 1000000 / Gain = 1000 */
		*val = 0;
		*val2 = 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	}

	return ret;
}

#define LASER_CHANNEL(axis)			\
{						\
	.type = IIO_ACCEL,			\
	.modified = 1,				\
	.channel2 = axis+1,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = axis,			\
	.scan_type = {				\
			.sign = 'u',		\
			.realbits = 32,		\
			.storagebits = 32,	\
			.shift = 0,		\
			.endianness = IIO_BE,	\
		}, \
}
	/*.scan_type = IIO_ST('u', 32, 32, 0) */

static const struct iio_chan_spec laser_channels[] = {
	LASER_CHANNEL(LASER_SCAN_ID),
	LASER_CHANNEL(LASER_SCAN_X),
	LASER_CHANNEL(LASER_SCAN_Y),
	LASER_CHANNEL(LASER_SCAN_Z),
	IIO_CHAN_SOFT_TIMESTAMP(LASER_SCAN_TIMESTAMP)
};

static const struct iio_info laser_info = {
	.read_raw = &laser_read_raw,
	.driver_module = THIS_MODULE,
};

static ssize_t active_set(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct laser_device_data *laser_data = dev_get_drvdata(dev);
	long enabled = 0;
	int rc;

	rc = kstrtoul(buf, 10, &enabled);
	if (rc) {
		E("%s: kstrtoul fails, rc = %d\n", __func__, rc);
		return rc;
	}

	if (enabled != laser_data->enabled) {
		if (enabled == 1) {
			rc = Laser_poweron();
		} else {
			rc = Laser_poweroff();
		}
		if (rc)
			return rc;
	}

	laser_data->enabled = !!enabled;

	I("%s: enabled = %d\n", __func__, laser_data->enabled);

	return count;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct laser_device_data *laser_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "enabled = %d\n",
			 laser_data->enabled);
}


static ssize_t batch_set(struct device *dev,
		     struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct laser_device_data *laser_data = dev_get_drvdata(dev);
	int delay_ms = 0;
	int rc;
	long input_val;

	rc = kstrtoul(buf, 10, &input_val);
	if (rc) {
		E("%s: kstrtoul fails, rc = %d\n", __func__, rc);
		return rc;
	}

	delay_ms = input_val;

	if ((delay_ms >= POLLING_DELAY_MIN) && ((delay_ms < POLLING_DELAY_MAX)))
		atomic_set(&laser_data->delay, delay_ms);
	else
		E("%s: Invalid delay_ms = %d\n", __func__, delay_ms);

	I("%s--: laser_data->delay = %d, delay_ms = %d\n", __func__,
	  atomic_read(&laser_data->delay), delay_ms);

	return (rc) ? rc : count;
}

static ssize_t batch_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct laser_device_data *laser_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "delay = %dms",
			 atomic_read(&laser_data->delay));
}



static ssize_t flush_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct laser_device_data *laser_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "pending_flush = %d\n",
			 laser_data->pending_flush);
}

static void laser_send_meta(struct laser_device_data *laser_data)
{
	u8 type = LASER_META_DATA;
	u32 data[REPORT_EVENT_COMMON_LEN] = {0};
	s64 timestamp = 0;
	int rc;

	I("%s: pending_flush = %d\n", __func__, laser_data->pending_flush);

	rc = laser_send_event(laser_data, type, data, timestamp);
	if (rc < 0)
		E("%s: send_event fails, rc = %d\n", __func__, rc);
	else
		laser_data->pending_flush--;

	if (laser_data->pending_flush) {

		I("%s: pending_flush = %d, send meta again\n",
		  __func__, laser_data->pending_flush);

		laser_data->wk_report_meta = true;
		queue_work(laser_data->laser_wq, &laser_data->one_shot_work);
	}
}

static ssize_t flush_set(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct laser_device_data *laser_data = dev_get_drvdata(dev);

	laser_data->pending_flush++;

	I("%s: laser_data->pending_flush = %d\n", __func__,
	  laser_data->pending_flush);

	laser_data->wk_report_meta = true;
	queue_work(laser_data->laser_wq, &laser_data->one_shot_work);

	return count;
}

static int Laser_offset_calibrate(int32_t *offset)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	Status = VL53L0_SetOffsetCalibrationDataMicroMeter(&MyDevice, 0);
	Status |= VL53L0_SetXTalkCompensationRateMegaCps(&MyDevice, 0);
	if (Status == VL53L0_ERROR_NONE)
	{
		Status = VL53L0_PerformOffsetCalibration(&MyDevice, (OFFSET_CALI_TARGET_DISTANCE << 16), offset);
		if (Status == VL53L0_ERROR_NONE) {
			// Update to offset_kvalue
			*offset = *offset / 1000;
			laser_data->offset_kvalue = *offset;
			return 0;
		} else {
			E("%s: Failed in offset calibration\n", __func__);
			return -1;
		}
	}
	else
	{
		E("%s: Set default offset/xtalk value failed\n", __func__);
		return -1;
	}

	return 0;
}

static int Laser_xtalk_calibrate(FixPoint1616_t *XTalkCompensationRateMegaCps)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	Status = VL53L0_SetXTalkCompensationRateMegaCps(&MyDevice, 0);
	if (Status == VL53L0_ERROR_NONE)
	{
		if (laser_data->cali_distance != 0)
		{
			Status = VL53L0_PerformXTalkCalibration(&MyDevice, laser_data->cali_distance, XTalkCompensationRateMegaCps);
			if (Status == VL53L0_ERROR_NONE)
			{
				I("%s: Xtalk calibration finished. XTalkCompensationRateMegaCps = 0x%X with distance = %d mm\n", __func__, *XTalkCompensationRateMegaCps, (laser_data->cali_distance>>16));
				// Update to xtalk_kvalue
				laser_data->xtalk_kvalue = *XTalkCompensationRateMegaCps;
				return 0;
			}
			else
			{
				E("%s: Failed in xtalk calibration\n", __func__);
				return -1;
			}
		}
		else
		{
			E("%s: Please set distance before xtalk calibration\n", __func__);
			return -1;
		}
	}
	else
	{
		E("%s: Set default xtalk value failed\n", __func__);
		return -1;
	}

	return 0;
}

static int Laser_parse_dt(struct device *dev, struct laser_device_data *pdata)
{
	struct device_node *dt = dev->of_node;
	struct device_node *sensor_offset;
	char *sensor_cali_data = NULL;
	int sensor_cali_size = 0;
	int i = 0;

	pdata->pwdn_gpio = of_get_named_gpio(dt, "laser,pwdn-gpio", 0);
	if (!gpio_is_valid(pdata->pwdn_gpio))
		E("%s: pwdn_gpio value is not valid\n", __func__);
	else
		I("%s: pwdn_gpio = %d\n", __func__, pdata->pwdn_gpio);

	pdata->power_gpio = of_get_named_gpio(dt, "laser,power-2v8", 0);
	if (!gpio_is_valid(pdata->power_gpio))
		E("%s: power_gpio value is not valid\n", __func__);
	else
		I("%s: power_gpio = %d\n", __func__, pdata->power_gpio);

        pdata->camio_1v8 = devm_regulator_get(&g_pstLaser_I2Cclient->dev, "CAMIO_1v8");
        if (IS_ERR(pdata->camio_1v8)) {
                pdata->camio_1v8 = NULL;
                E("%s: Unable to get CAMIO_1v8\n", __func__);
        }

	// get calibration data
	sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	if (sensor_offset) {
		sensor_cali_data = (char *)
				of_get_property(sensor_offset,
						SENSOR_FLASH_DATA,
						&sensor_cali_size);
		I("%s: sensor cali_size = %d\n", __func__, sensor_cali_size);

		if (sensor_cali_data) {
			for (i = 100; (i < sensor_cali_size) && (i < 108); i++) {
				I("gyro sensor cali_data[%d] = %02x\n", i, sensor_cali_data[i]);
			}

			laser_data->offset_kvalue = (s8) sensor_cali_data[103];
			I("%s: Update offset_kvalue = %d\n",__func__, laser_data->offset_kvalue);
			if (laser_data->offset_kvalue != 0)
				laser_data->cali_status = 0x10;

			laser_data->xtalk_kvalue = (sensor_cali_data[107]<<8) | sensor_cali_data[106];
			I("%s: Update xtalk_kvalue = 0x%X\n", __func__, laser_data->xtalk_kvalue);
			if (laser_data->xtalk_kvalue != 0)
				laser_data->cali_status |= 0x1;
		}
	} else {
		E("%s: Sensor Calibration data offset not found\n", __func__);
	}

        return 0;
}

static ssize_t laser_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "laser_power = %d\n", laser_data->laser_power);
}

static ssize_t laser_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value = 0;
	int err = 0;

	err = kstrtoul(buf, 10, &value);
	if (err) {
		E("%s: kstrtoul fails, error = %d\n", __func__, err);
		return err;
	}

	if (value == 1)
		err = Laser_poweron();
	else
		err = Laser_poweroff();

	if (err)
		return -1;

	return count;
}

static ssize_t laser_hwid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 model_id = 0;
	u8 revisin_id = 0;
	u8 module_id = 0;
	u16 rangeA_timeout = 0;
	u16 rangeB1_timeout = 0;

	ret = Laser_RegReadByte(VL53L0_REG_IDENTIFICATION_MODEL_ID, &model_id);
	ret += Laser_RegReadByte(VL53L0_REG_IDENTIFICATION_REVISION_ID, &revisin_id);
	ret += Laser_RegReadByte(VL53L0_REG_IDENTIFICATION_MODULE_ID, &module_id);
//	ret += Laser_RegReadWord(VL53L0_REG_RNGA_TIMEOUT_MSB, &rangeA_timeout);
//	ret += Laser_RegReadWord(VL53L0_REG_RNGB1_TIMEOUT_MSB, &rangeB1_timeout);

	if (ret == 0)
		return scnprintf(buf, PAGE_SIZE, "0x%X 0x%X 0x%X 0x%X 0x%X\n", model_id, revisin_id, module_id, rangeA_timeout, rangeB1_timeout);
	else
		return scnprintf(buf, PAGE_SIZE, "0 0 0 0 0\n");
}

static ssize_t laser_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	VL53L0_RangingMeasurementData_t RangingMeasurementData;

	if (laser_data->laser_power == 1) {
		Status = VL53L0_PerformSingleRangingMeasurement(&MyDevice, &RangingMeasurementData);
		if (Status == VL53L0_ERROR_NONE) {
			I("RangeMilliMeter = %d,  SignalRateRtnMegaCps = 0x%x\n", RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
		}

		return scnprintf(buf, PAGE_SIZE, "RangeMilliMeter = %d,  SignalRateRtnMegaCps = 0x%x\n", RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
	} else {
		return scnprintf(buf, PAGE_SIZE, "RangeMilliMeter = 0,  SignalRateRtnMegaCps = 0\n");
	}
}

static ssize_t laser_compensation_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	char compensation_enable = 0;

	Status = VL53L0_GetXTalkCompensationEnable(&MyDevice, &compensation_enable);
	if (Status == VL53L0_ERROR_NONE)
	{
		return scnprintf(buf, PAGE_SIZE, "compensation_enable = %d \n", compensation_enable);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in VL53L0_GetXTalkCompensationEnable\n");
	}
}

static ssize_t laser_compensation_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	unsigned long value = 0;
	int err = 0;

	err = kstrtoul(buf, 10, &value);
	if (err) {
		E("%s: kstrtoul fails, error = %d\n", __func__, err);
		return err;
	}

	Status = VL53L0_SetXTalkCompensationEnable(&MyDevice, value);
	if (Status == VL53L0_ERROR_NONE)
	{
		I("%s: Set compensation_enable = %lu\n", __func__, value);
	}
	else
	{
		E("%s: Failed in VL53L0_SetXTalkCompensationEnable\n", __func__);
		return -1;
	}

	return count;
}

static ssize_t laser_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	int offset_value = 0;

	Status = VL53L0_GetOffsetCalibrationDataMicroMeter(&MyDevice, &offset_value);
	offset_value = (offset_value / 1000);
	if (Status == VL53L0_ERROR_NONE)
	{
		return scnprintf(buf, PAGE_SIZE, "Offset = %d mm\n", offset_value);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in VL53L0_GetOffsetCalibrationDataMicroMeter\n");
	}
}

static ssize_t laser_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	int value = 0;
	int err = 0;

	err = kstrtoint(buf, 10, &value);
	if (err) {
		E("%s: kstrtoint fails, error = %d\n", __func__, err);
		return err;
	}

	Status = VL53L0_SetOffsetCalibrationDataMicroMeter(&MyDevice, (value*1000));
	if(Status == VL53L0_ERROR_NONE)
	{
		I("%s: Set offset = %d mm\n", __func__, value);
		// Update to offset_kvalue
		laser_data->offset_kvalue = value;
	}
	else
	{
		E("%s: Failed in VL53L0_SetOffsetCalibrationDataMicroMeter\n", __func__);
		return -1;
	}

	return count;
}

static ssize_t laser_xtalk_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	FixPoint1616_t XTalkCompensationRateMegaCps;

	Status = VL53L0_GetXTalkCompensationRateMegaCps(&MyDevice, &XTalkCompensationRateMegaCps);
	if (Status == VL53L0_ERROR_NONE)
	{
		return scnprintf(buf, PAGE_SIZE, "Xtalk = 0x%X\n", XTalkCompensationRateMegaCps);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in VL53L0_GetXTalkCompensationRateMegaCps\n");
	}
}

static ssize_t laser_xtalk_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	unsigned long value = 0;
	int err = 0;

	err = kstrtoul(buf, 10, &value);
	if (err) {
		E("%s: kstrtoul fails, error = %d\n", __func__, err);
		return err;
	}

	Status = VL53L0_SetXTalkCompensationRateMegaCps(&MyDevice, value);
	if(Status == VL53L0_ERROR_NONE)
	{
		I("%s: Set xtalk = %lu\n", __func__, value);
		// Update to xtalk_kvalue
		laser_data->xtalk_kvalue = value;
	}
	else
	{
		E("%s: Failed in VL53L0_SetXTalkCompensationRateMegaCps\n", __func__);
		return -1;
	}

	return count;
}

static ssize_t laser_offset_calibrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int offset_value = 0;

	ret = Laser_offset_calibrate(&offset_value);
	if (ret == 0)
	{
		return scnprintf(buf, PAGE_SIZE, "Offset calibration finished. Offset = %d mm\n", offset_value);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in offset calibration\n");
	}

}

static ssize_t laser_xtalk_calibrate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value = 0;
	int err = 0;

	err = kstrtoul(buf, 10, &value);
	if (err) {
		E("%s: kstrtoul fails, error = %d\n", __func__, err);
		return err;
	}

	laser_data->cali_distance = value << 16;

	return count;
}

static ssize_t laser_xtalk_calibrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	FixPoint1616_t XTalkCompensationRateMegaCps = 0;

	if (laser_data->cali_distance != 0)
	{
		ret = Laser_xtalk_calibrate(&XTalkCompensationRateMegaCps);
		if (ret == 0)
		{
			return scnprintf(buf, PAGE_SIZE, "Xtalk calibration finished. XTalkCompensationRateMegaCps = 0x%X with distance = %d mm\n", XTalkCompensationRateMegaCps, (laser_data->cali_distance>>16));
		}
		else
		{
			return scnprintf(buf, PAGE_SIZE, "Failed in xtalk calibration\n");
		}
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Please set distance before xtalk calibration\n");
	}
}

static ssize_t laser_cali_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "MFG calibration status = 0x%x\n",laser_data->cali_status);
}

static struct device_attribute attributes[] = {
	__ATTR(enable, 0660, active_show, active_set),
	__ATTR(batch, 0660, batch_show, batch_set),
	__ATTR(flush, 0660, flush_show, flush_set),
	__ATTR(laser_power, 0660, laser_power_show, laser_power_store),
	__ATTR(laser_hwid, 0440, laser_hwid_show, NULL),
	__ATTR(laser_range, 0440, laser_range_show, NULL),
	__ATTR(laser_compensation, 0660, laser_compensation_show, laser_compensation_store),
	__ATTR(laser_offset, 0660, laser_offset_show, laser_offset_store),
	__ATTR(laser_xtalk, 0660, laser_xtalk_show, laser_xtalk_store),
	__ATTR(laser_offset_calibrate, 0440, laser_offset_calibrate_show, NULL),
	__ATTR(laser_xtalk_calibrate, 0660, laser_xtalk_calibrate_show, laser_xtalk_calibrate_store),
	__ATTR(laser_cali_status, 0440, laser_cali_status_show, NULL),
};

static int Laser_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int Laser_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long Laser_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	I("%s: cmd = 0x%x\n", __func__, cmd);

	if (Laser_poweron()) {
		E("%s: Failed to do Laser_poweron\n", __func__);
		return -1;
	}

	switch (cmd) {
	case VL53L0_IOCTL_GET_DATA:
		{
			VL53L0_RangingMeasurementData_t RangingMeasurementData;
			VL53L0_Error Status = VL53L0_ERROR_NONE;
			uint32_t RangeSum = 0;
			uint32_t RateSum = 0;
			int i = 0;
			int measure_times = 0;

			for (i = 0; i < RANGE_MEASUREMENT_TIMES;)
			{
				Status = VL53L0_PerformSingleRangingMeasurement(&MyDevice, &RangingMeasurementData);
				if (Status == VL53L0_ERROR_NONE && RangingMeasurementData.RangeMilliMeter < RANGE_MEASUREMENT_OVERFLOW) {
					i++;
					RangeSum += RangingMeasurementData.RangeMilliMeter;
					RateSum += RangingMeasurementData.SignalRateRtnMegaCps;
				}
				measure_times++;
				if (measure_times > RANGE_MEASUREMENT_RETRY_TIMES) {
					E("Failed to get measurement data\n");
					return -1;
				}
			}

			// Return avg, data
			I("Sum of measurement data = (0x%X , 0x%X)", RangeSum, RateSum);
			RangingMeasurementData.RangeMilliMeter = RangeSum / RANGE_MEASUREMENT_TIMES;
			RangingMeasurementData.SignalRateRtnMegaCps = RateSum / RANGE_MEASUREMENT_TIMES;		

			if (copy_to_user(argp , &RangingMeasurementData , sizeof(VL53L0_RangingMeasurementData_t)))
			{
				E("copy to user failed in VL53L0_IOCTL_GET_DATA\n");
				return -1;
			}
		}
		break;

	case VL53L0_IOCTL_OFFSET_CALI:
		{
			int offset_value = 0;
			int ret = 0;

			ret = Laser_offset_calibrate(&offset_value);
			if (ret == 0)
			{
				I("Offset calibration finished. Offset = %d mm\n", offset_value);
			}
			else
			{
				E("Failed in offset calibration\n");
				return -1;
			}

			if(copy_to_user(argp , &offset_value , sizeof(int))) {
				E("copy to user failed in VL53L0_IOCTL_OFFSET_CALI\n");
				return -1;
			}
		}
		break;

	case VL53L0_IOCTL_XTALK_CALI:
		{
			FixPoint1616_t XTalkCompensationRateMegaCps = 0;
			int ret = 0;
			VL53L0_Error Status = VL53L0_ERROR_NONE;

			ret = Laser_xtalk_calibrate(&XTalkCompensationRateMegaCps);
			if (ret == 0)
			{
				I("Xtalk calibration finished. XTalkCompensationRateMegaCps = 0x%X with distance = %d mm\n", XTalkCompensationRateMegaCps, (laser_data->cali_distance>>16));
			}
			else
			{
				E("Failed in xtalk calibration\n");
				return -1;
			}

			// read out xtalk value and return
			Status = VL53L0_GetXTalkCompensationRateMegaCps(&MyDevice, &XTalkCompensationRateMegaCps);
			if (Status != VL53L0_ERROR_NONE)
			{
				E("Failed in VL53L0_GetXTalkCompensationRateMegaCps\n");
				return -1;
			}

			if(copy_to_user(argp , &XTalkCompensationRateMegaCps , sizeof(int))) {
				E("copy to user failed in VL53L0_IOCTL_XTALK_CALI\n");
				return -1;
			}
		}
		break;

	case VL53L0_IOCTL_SET_XTALK_CALI_DISTANCE:
		{
			int distance_mm = 0;
			if(copy_from_user(&distance_mm , argp , sizeof(int))) {
				E("copy from user failed in VL53L0_IOCTL_SET_XTALK_DISTANCE\n");
				return -1;
			}

			laser_data->cali_distance = distance_mm << 16;
			I("Set distance for xtalk calibration to %d mm\n", distance_mm);
		}
		break;
	}

	Laser_poweroff();

	return 0;
}

static const struct file_operations laser_fops =
{
	.owner = THIS_MODULE,
	.open = Laser_open,
	.release = Laser_release,
	.unlocked_ioctl = Laser_ioctl,
};

static struct miscdevice laser_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "laser_stmvl53l0",
	.fops = &laser_fops,
};

static void laser_one_shot(struct work_struct *work)
{
	struct laser_device_data *laser_data = container_of(
						(struct work_struct *)work,
						struct laser_device_data,
						one_shot_work);

	if (laser_data->wk_report_meta == true) {
		laser_data->wk_report_meta = false;
		laser_send_meta(laser_data);
	}

}

static void laser_work_report(struct work_struct *work)
{
	struct laser_device_data *laser_data =
			container_of((struct delayed_work *)work,
				     struct laser_device_data, work);

	if (atomic_read(&laser_data->pseudo_irq_enable)) {
		unsigned long jiff;

		jiff = msecs_to_jiffies(atomic_read(&laser_data->delay));
		if (!jiff)
			jiff = 1;
		irq_work_queue(&laser_data->iio_irq_work);
		queue_delayed_work(laser_data->laser_wq, &laser_data->work,
				   jiff);
	}
}

static void laser_remove_trigger(struct iio_dev *indio_dev)
{
	struct laser_device_data *laser_data = iio_priv(indio_dev);

	iio_trigger_unregister(laser_data->trig);
	iio_trigger_free(laser_data->trig);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
}
static void laser_remove_buffer(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
	iio_kfifo_free(indio_dev->buffer);
}

static int Laser_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	int err = 0;
	int i = 0;
	u8 model_id = 0;
	u8 module_id = 0;

	I("Laser_probe++\n");

	indio_dev = iio_device_alloc(sizeof(*laser_data));
	if (!indio_dev) {
		I("%s: iio_device_alloc failed\n", __func__);
		return -ENOMEM;
	}

	err = misc_register(&laser_device);
	if (err) {
		E("Failed to register misc device for '%s'!\n", laser_device.name);
		goto exit_err;
	}

	i2c_set_clientdata(client, indio_dev);

	indio_dev->name = STMVL53L0_DRV_NAME;
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &laser_info;
	indio_dev->channels = laser_channels;
	indio_dev->num_channels = ARRAY_SIZE(laser_channels);
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	laser_data = iio_priv(indio_dev);
	laser_data->indio_dev = indio_dev;


	g_pstLaser_I2Cclient = client;
	laser_data->sensor_dev = &client->dev;
	MyDevice.I2cDevAddr = g_pstLaser_I2Cclient->addr;
	MyDevice.comms_type = 1;
	MyDevice.comms_speed_khz = 400;

	mutex_init(&laser_data->lock);

	INIT_DELAYED_WORK(&laser_data->work, laser_work_report);
	INIT_WORK(&laser_data->one_shot_work, laser_one_shot);

	init_irq_work(&laser_data->iio_irq_work, iio_trigger_work);

	laser_data->laser_wq = create_singlethread_workqueue("htc_laser");

	// parse device tree
	if (client->dev.of_node) {
		Laser_parse_dt(&client->dev, laser_data);
	}

	err = laser_probe_buffer(indio_dev);
	if (err) {
		E("%s: iio laser_probe_buffer failed\n", __func__);
		goto error_free_dev;
	}
	err = laser_probe_trigger(indio_dev);
	if (err) {
		E("%s: iio laser_probe_trigger failed\n", __func__);
		goto error_remove_buffer;
	}
	err = iio_device_register(indio_dev);
	if (err) {
		E("%s: iio iio_device_register failed\n", __func__);
		goto error_remove_trigger;
	}


	laser_data->laser_class = class_create(THIS_MODULE, "htc_laser");
	if (IS_ERR(laser_data->laser_class)) {
		goto exit_err_class_create;
	}

	laser_data->laser_dev = device_create(laser_data->laser_class, NULL, 0, "%s", "laser");
	if (IS_ERR(laser_data->laser_dev)) {
		goto exit_err_device_create;
	}

	dev_set_drvdata(laser_data->laser_dev, laser_data);

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		if (device_create_file(laser_data->laser_dev, attributes + i))
			goto exit_err_file_create;
	}

	err = sysfs_create_link(&laser_data->laser_dev->kobj,
				&laser_data->indio_dev->dev.kobj, "iio");
	if (err < 0) {
		E("%s: sysfs_create_link() fails, err = %d\n", __func__, err);
		goto exit_err_create_link;
	}

	atomic_set(&laser_data->delay, 200);

	// gpio pin init
	Laser_pinctrl_init();

	// Make sure that camio_1v8 is disable
	err = regulator_disable(laser_data->camio_1v8);
        if (err)
                E("%s: Failed to disable CAMIO_1v8.\n", __func__);

	// read out model ID for check
	err = Laser_poweron_without_init();
	if (!err) {
		Laser_RegReadByte(VL53L0_REG_IDENTIFICATION_MODEL_ID, &model_id);
		if (model_id != VL53L0_MODEL_ID) {
			E("Model ID doesn't match!\n");
			err = Laser_poweroff();
			err = -ENOENT;
			goto exit_err_create_link;
		}
		Laser_RegReadByte(VL53L0_REG_IDENTIFICATION_MODULE_ID, &module_id);
		if ( module_id != VL53L0_MODULE_ID_1_1) {
			E("module ID doesn't match!\n");
			err = Laser_poweroff();
			err = -ENOENT;
			goto exit_err_create_link;
		}
	}
	err = Laser_poweroff();

	I("%s: Successful\n", __func__);

	return 0;

exit_err_create_link:
exit_err_file_create:
	while (--i >= 0)
		device_remove_file(laser_data->laser_dev, attributes + i);
	device_unregister(laser_data->laser_dev);
exit_err_device_create:
	class_destroy(laser_data->laser_class);
exit_err_class_create:
	if (indio_dev)
		iio_device_unregister(indio_dev);
error_remove_trigger:
	if (indio_dev)
		laser_remove_trigger(indio_dev);
error_remove_buffer:
	if (indio_dev)
		laser_remove_buffer(indio_dev);
error_free_dev:
exit_err:
	return err;

}

static int Laser_remove(struct i2c_client *client)
{
	E("Laser_remove++\n");
	return 0;
}

static const struct i2c_device_id stmvl53l0_id[] = {
	{ STMVL53L0_DRV_NAME, 0 },
	{ }
};

static struct of_device_id Laser_match_table[] = {
        {.compatible = "htc_laser" },
        {},
};

MODULE_DEVICE_TABLE(i2c, stmvl53l0_id);

static struct i2c_driver stLaser_Driver = {
	.driver = {
		.name = STMVL53L0_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = Laser_match_table,
	},
	.probe    = Laser_probe,
	.remove   = Laser_remove,
	.id_table = stmvl53l0_id,
};

static int __init STMVL53L0_i2c_init(void)
{
	I("STMVL53L0_i2c_init\n");
	return i2c_add_driver(&stLaser_Driver);
}
module_init(STMVL53L0_i2c_init);

static void __exit STMVL53L0_i2c_exit(void)
{
	I("STMVL53L0_i2c_exit\n");
	i2c_del_driver(&stLaser_Driver);
}
module_exit(STMVL53L0_i2c_exit);

MODULE_DESCRIPTION("VL53L0 Laser Sensor Driver V1.0");
MODULE_LICENSE("GPL");
