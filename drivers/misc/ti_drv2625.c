/*
 * Copyright (c) 2016  Texas Instruments Inc.
 * Copyright (c) 2016  LGE Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c/ti_drv2625.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>


#define DRV2625_SEQ_MAX_NUM        8
#define DRV2625_AUTOCAL_WAIT_COUNT 5

struct drv2625_platform_data {
	int mnGpioNRST;
	enum loop_type meLoop;
	struct actuator_data msActuator;
	bool autocal_enabled;
};

struct drv2625_data {
	/* led_dev should be first because it is referenced by power_on */
	struct led_classdev led_dev;
	struct drv2625_platform_data msPlatData;
	unsigned char mnDeviceID;
	struct device *dev;
	struct regmap *mpRegmap;
	struct i2c_client* client;
	unsigned char mnIntStatus;
	struct drv2625_waveform_sequencer msWaveformSequencer;
	unsigned char mnFileCmd;
	volatile int mnVibratorPlaying;
	volatile char mnWorkMode;
	unsigned char mnCurrentReg;
	struct wake_lock wklock;
	struct mutex lock;
	struct work_struct vibrator_work;
	struct work_struct haptics_work;
	struct regulator *vdd_reg;
	struct dentry *dent;
};


static int drv2625_reg_read(struct drv2625_data *pDrv2625data,
		unsigned char reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pDrv2625data->mpRegmap, reg, &val);
	if (ret < 0){
		dev_err(pDrv2625data->dev,
			"%s reg=0x%x error %d\n", __func__, reg, ret);
		return ret;
	}

	return val;
}

static int drv2625_reg_write(struct drv2625_data *pDrv2625data,
	unsigned char reg, unsigned char val)
{
	int ret;

	ret = regmap_write(pDrv2625data->mpRegmap, reg, val);
	if (ret < 0){
		dev_err(pDrv2625data->dev,
			"%s reg=0x%x, value=0%x error %d\n",
			__func__, reg, val, ret);
	}

	return ret;
}


static int drv2625_bulk_write(struct drv2625_data *pDrv2625data,
	unsigned char reg, unsigned int count, const u8 *buf)
{
	int ret;
	ret = regmap_bulk_write(pDrv2625data->mpRegmap, reg, buf, count);
	if (ret < 0){
		dev_err(pDrv2625data->dev,
			"%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, ret);
	}

	return ret;
}


static void drv2625_enable_irq(struct drv2625_data *pDrv2625data)
{
	drv2625_reg_read(pDrv2625data, DRV2625_REG_STATUS);
	drv2625_reg_write(pDrv2625data,
			  DRV2625_REG_INT_ENABLE, INT_ENABLE_ALL);
	enable_irq(pDrv2625data->client->irq);
}

static void drv2625_disable_irq(struct drv2625_data *pDrv2625data)
{
	disable_irq(pDrv2625data->client->irq);
	drv2625_reg_write(pDrv2625data, DRV2625_REG_INT_ENABLE, INT_MASK_ALL);
}

static int drv2625_set_bits(struct drv2625_data *pDrv2625data,
	unsigned char reg, unsigned char mask, unsigned char val)
{
	int ret;
	ret = regmap_update_bits(pDrv2625data->mpRegmap, reg, mask, val);
	if (ret < 0){
		dev_err(pDrv2625data->dev,
			"%s reg=%x, mask=0x%x, value=0x%x error %d\n",
			__func__, reg, mask, val, ret);
	}

	return ret;
}

static int drv2625_set_go_bit(struct drv2625_data *pDrv2625data,
		unsigned char val)
{
	int ret = 0;

	val &= 0x01;
	dev_info(pDrv2625data->dev, "set_go_bit %d\n", val);
	ret = drv2625_reg_write(pDrv2625data, DRV2625_REG_GO, val);
	if (ret < 0)
		return ret; /* Error writing to GO bit */

	if (val == 1) {
		usleep_range(1000, 2000);
		ret = drv2625_reg_read(pDrv2625data, DRV2625_REG_GO);
		if (ret != 1) {
			dev_err(pDrv2625data->dev,
				"setting GO bit failed, stop action\n");
			ret = -1;
		}
	} else {
		/* After writing 0 to the GO bit, wait until the bit
		 * reads back as 0, indicating that the auto-break
		 * cycle has finished.
		 */
		int poll_ready = 20;

		while (1) {
			ret = drv2625_reg_read(pDrv2625data, DRV2625_REG_GO);
			if (ret == 0)
				break; /* Success */

			/* Try reading again in 1-2 ms*/
			poll_ready--;
			if (poll_ready <= 0) {
				dev_err(pDrv2625data->dev,
					"clearing GO bit failed\n");
				ret = -1;
				break;
			}
			usleep_range(1000, 2000);
		}
	}
	return ret;
}

static void drv2625_change_mode(struct drv2625_data *pDrv2625data,
		unsigned char work_mode)
{
	drv2625_set_bits(pDrv2625data, DRV2625_REG_MODE, DRV2625_MODE_MASK , work_mode);
}


static void drv2625_stop(struct drv2625_data *pDrv2625data)
{
	if (pDrv2625data->mnVibratorPlaying == YES) {
		drv2625_disable_irq(pDrv2625data);
		drv2625_set_go_bit(pDrv2625data, STOP);
		pDrv2625data->mnVibratorPlaying = NO;
		wake_unlock(&pDrv2625data->wklock);
	}
}

static void drv2625_haptics_work(struct work_struct *work)
{
	struct drv2625_data *pDrv2625data =
		container_of(work, struct drv2625_data, haptics_work);
	int ret = 0;
	const int state = pDrv2625data->led_dev.brightness;

	mutex_lock(&pDrv2625data->lock);
	pDrv2625data->mnWorkMode = WORK_IDLE;
	drv2625_stop(pDrv2625data);
	if (state != LED_OFF) {
		wake_lock(&pDrv2625data->wklock);
		pDrv2625data->mnVibratorPlaying = YES;
		drv2625_enable_irq(pDrv2625data);
		ret = drv2625_set_go_bit(pDrv2625data, GO);
		if (ret < 0) {
			wake_unlock(&pDrv2625data->wklock);
			pDrv2625data->mnVibratorPlaying = NO;
			drv2625_disable_irq(pDrv2625data);
		} else {
			pDrv2625data->mnWorkMode |= WORK_VIBRATOR;
		}
	}
	mutex_unlock(&pDrv2625data->lock);
}


static void vibrator_enable(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct drv2625_data *pDrv2625data =
		container_of(led_cdev, struct drv2625_data, led_dev);
	dev_info(pDrv2625data->dev, "vibrator_enable brightness=%d\n", value);
	led_cdev->brightness = value;
	schedule_work(&pDrv2625data->haptics_work);
}


static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2625_data *pDrv2625data =
		container_of(work, struct drv2625_data, vibrator_work);
	unsigned char mode;

	mutex_lock(&pDrv2625data->lock);
	if (pDrv2625data->mnWorkMode & WORK_IRQ) {
		unsigned char status = pDrv2625data->mnIntStatus;
		drv2625_disable_irq(pDrv2625data);

		if (status & OVERCURRENT_MASK){
			dev_err(pDrv2625data->dev,
				"ERROR, Over Current detected!!\n");
		}

		if (status & OVERTEMPRATURE_MASK){
			dev_err(pDrv2625data->dev,
				"ERROR, Over Temperature detected!!\n");
		}

		if (status & ULVO_MASK){
			dev_err(pDrv2625data->dev,
				"ERROR, VDD drop observed!!\n");
		}

		if (status & PRG_ERR_MASK){
			dev_err(pDrv2625data->dev,
				"ERROR, PRG error!!\n");
		}

		if (status & PROCESS_DONE_MASK){
			mode = drv2625_reg_read(pDrv2625data,
					DRV2625_REG_MODE) & DRV2625_MODE_MASK;
			if (mode == DRV2625_MODE_CALIBRATION){
				if ((status&DIAG_MASK) != DIAG_SUCCESS){
					dev_err(pDrv2625data->dev,
							"Calibration fail\n");
				} else {
					unsigned char calComp =
						drv2625_reg_read(pDrv2625data,
							DRV2625_REG_CAL_COMP);
					unsigned char calBemf =
						drv2625_reg_read(pDrv2625data,
							DRV2625_REG_CAL_BEMF);
					unsigned char calBemfGain =
						drv2625_reg_read(pDrv2625data,
							DRV2625_REG_LOOP_CONTROL) &
							BEMFGAIN_MASK;
					dev_info(pDrv2625data->dev,
						"AutoCal : Comp=0x%x, Bemf=0x%x, Gain=0x%x\n",
						calComp, calBemf, calBemfGain);
				}
			} else if (mode == DRV2625_MODE_DIAGNOSTIC){
				if ((status&DIAG_MASK) != DIAG_SUCCESS) {
					dev_err(pDrv2625data->dev,
							"Diagnostic fail\n");
				} else {
					unsigned char diagZ =
						drv2625_reg_read(pDrv2625data,
							DRV2625_REG_DIAG_Z);
					unsigned char diagK =
						drv2625_reg_read(pDrv2625data,
							DRV2625_REG_DIAG_K);
					dev_info(pDrv2625data->dev,
						"Diag : ZResult=0x%x, CurrentK=0x%x\n",
						diagZ, diagK);
				}
			} else if (mode == DRV2625_MODE_WAVEFORM_SEQUENCER) {
				dev_info(pDrv2625data->dev,
					"Waveform Sequencer Playback finished\n");
			}

			if (pDrv2625data->mnVibratorPlaying == YES) {
				pDrv2625data->mnVibratorPlaying = NO;
				wake_unlock(&pDrv2625data->wklock);
			}
		}

		pDrv2625data->mnWorkMode &= ~WORK_IRQ;
	}

	if (pDrv2625data->mnWorkMode & WORK_VIBRATOR) {
		drv2625_stop(pDrv2625data);
		pDrv2625data->mnWorkMode &= ~WORK_VIBRATOR;
	}

	mutex_unlock(&pDrv2625data->lock);
}


static void drv2625_auto_cal(struct drv2625_data *pDrv2625data)
{
	struct drv2625_platform_data *pDrv2625Platdata =
		&pDrv2625data->msPlatData;
	struct actuator_data actuator = pDrv2625Platdata->msActuator;
	unsigned char value_temp = 0;
	unsigned char mask_temp = 0;
	unsigned char DriveTime = 0;
	unsigned char calComp = 0;
	unsigned char calBemf = 0;
	unsigned char calBemfGain = 0;
	int cnt = 0;

	drv2625_change_mode(pDrv2625data, DRV2625_MODE_CALIBRATION);

	drv2625_set_bits(pDrv2625data, DRV2625_REG_CONTROL1, ACTUATOR_MASK,
			(actuator.meActuatorType << ACTUATOR_SHIFT));

	mask_temp = FB_BRK_FACTOR_MASK | LOOP_GAIN_MASK;
	value_temp = (3 << FB_BRK_FACTOR_SHIFT) | (2 << LOOP_GAIN_SHIFT);
	drv2625_set_bits(pDrv2625data, DRV2625_REG_LOOP_CONTROL,
			mask_temp, value_temp);

	drv2625_reg_write(pDrv2625data,
		DRV2625_REG_RATED_VOLTAGE, actuator.mnRatedVoltage);

	drv2625_reg_write(pDrv2625data,
		DRV2625_REG_OVERDRIVE_CLAMP, actuator.mnOverDriveClampVoltage);

	drv2625_reg_write(pDrv2625data, DRV2625_REG_AUTO_CAL_TIME, 3);

	DriveTime = 5*(1000 - actuator.mnLRAFreq)/actuator.mnLRAFreq;
	drv2625_set_bits(pDrv2625data, DRV2625_REG_DRIVE_TIME,
			DRIVE_TIME_MASK, DriveTime);

	mask_temp = BRK_TIME_MASK | IDISS_TIME_MASK;
	value_temp = (1 << BRK_TIME_SHIFT) | 0x1;
	drv2625_set_bits(pDrv2625data, DRV2625_REG_BRK_TIME,
			mask_temp, value_temp);

	mask_temp = SAMPLE_TIME_MASK | ZC_DET_TIME_MASK;
	value_temp = (3 << SAMPLE_TIME_SHIFT);
	drv2625_set_bits(pDrv2625data, DRV2625_REG_OD_CLAMP_TIME,
			mask_temp, value_temp);

	drv2625_set_go_bit(pDrv2625data, GO);

	calComp = drv2625_reg_read(pDrv2625data, DRV2625_REG_CAL_COMP);
	calBemf = drv2625_reg_read(pDrv2625data, DRV2625_REG_CAL_BEMF);
	calBemfGain = drv2625_reg_read(pDrv2625data, DRV2625_REG_LOOP_CONTROL) &
			BEMFGAIN_MASK;
	dev_dbg(pDrv2625data->dev,
		"AutoCal(before) : Comp=0x%x, Bemf=0x%x, Gain=0x%x\n",
		calComp, calBemf, calBemfGain);
	dev_dbg(pDrv2625data->dev,
		"%s: vibrator is auto calibrating(1000ms)...\n",__func__);
	msleep(1000);

	drv2625_set_go_bit(pDrv2625data, STOP);

	do {
		pDrv2625data->mnIntStatus =
			drv2625_reg_read(pDrv2625data,DRV2625_REG_STATUS);
		if (pDrv2625data->mnIntStatus & PROCESS_DONE_MASK)
			break;

		msleep(20);
	} while (cnt++ < DRV2625_AUTOCAL_WAIT_COUNT);

	dev_dbg(pDrv2625data->dev,
			"%s: vibrator auto calibration waiting(%dms)\n",
			__func__, cnt * 20);

	if (cnt > DRV2625_AUTOCAL_WAIT_COUNT) {
		dev_warn(pDrv2625data->dev,
			"%s: vibrator auto calibration is failed(timeout)\n",
			__func__);
		return;
	}

	pDrv2625data->mnWorkMode |= WORK_IRQ;
	schedule_work(&pDrv2625data->vibrator_work);
}

static void dev_init_platform_data(struct drv2625_data *pDrv2625data)
{
	struct drv2625_platform_data *pDrv2625Platdata =
		&pDrv2625data->msPlatData;
	struct actuator_data actuator = pDrv2625Platdata->msActuator;
	unsigned char value_temp = 0;
	unsigned char mask_temp = 0;

	drv2625_set_bits(pDrv2625data,
		DRV2625_REG_INT_ENABLE, INT_MASK_ALL, INT_ENABLE_ALL);

	drv2625_set_bits(pDrv2625data,
		DRV2625_REG_MODE, PINFUNC_MASK, (PINFUNC_INT<<PINFUNC_SHIFT));

	if ((actuator.meActuatorType == ERM)||
		(actuator.meActuatorType == LRA)) {
		mask_temp |= ACTUATOR_MASK;
		value_temp |= (actuator.meActuatorType << ACTUATOR_SHIFT);
	}

	if ((pDrv2625Platdata->meLoop == CLOSE_LOOP)||
		(pDrv2625Platdata->meLoop == OPEN_LOOP)) {
		mask_temp |= LOOP_MASK;
		value_temp |= (pDrv2625Platdata->meLoop << LOOP_SHIFT);
	}

	if (value_temp != 0) {
		drv2625_set_bits(pDrv2625data,
			DRV2625_REG_CONTROL1,
			mask_temp|AUTOBRK_OK_MASK, value_temp|AUTOBRK_OK_ENABLE);
	}

	value_temp = 0;
	if(actuator.meActuatorType == ERM)
		value_temp = LIB_ERM;
	else if(actuator.meActuatorType == LRA)
		value_temp = LIB_LRA;
	if (value_temp != 0){
		drv2625_set_bits(pDrv2625data,
			DRV2625_REG_CONTROL2, LIB_MASK, value_temp<<LIB_SHIFT);
	}

	/* Set PLAYBACK_INTERVAL to 1ms. This shortens auto-break to 10ms */
	drv2625_set_bits(pDrv2625data,
			DRV2625_REG_CONTROL2, INTERVAL_MASK, 1<<INTERVAL_SHIFT);

	if (actuator.mnRatedVoltage != 0){
		drv2625_reg_write(pDrv2625data,
			DRV2625_REG_RATED_VOLTAGE, actuator.mnRatedVoltage);
	} else {
		dev_err(pDrv2625data->dev,
			"%s, ERROR Rated ZERO\n", __func__);
	}

	if (actuator.mnOverDriveClampVoltage != 0){
		drv2625_reg_write(pDrv2625data,
			DRV2625_REG_OVERDRIVE_CLAMP, actuator.mnOverDriveClampVoltage);
	} else {
		dev_err(pDrv2625data->dev,
			"%s, ERROR OverDriveVol ZERO\n", __func__);
	}

	if (actuator.meActuatorType == LRA) {
		unsigned char DriveTime = 0;
		unsigned short openLoopPeriod = 0;
		if (actuator.mnLRAFreq) {
		    DriveTime = 5*(1000 - actuator.mnLRAFreq)/actuator.mnLRAFreq;
		    openLoopPeriod = (unsigned short)((unsigned int)1000000000 /
				    (24619 * actuator.mnLRAFreq));
		}

		if (actuator.mnLRAFreq < 125)
			DriveTime |= (MINFREQ_SEL_45HZ << MINFREQ_SEL_SHIFT);
		drv2625_set_bits(pDrv2625data,
			DRV2625_REG_DRIVE_TIME,
			DRIVE_TIME_MASK | MINFREQ_SEL_MASK, DriveTime);
		drv2625_set_bits(pDrv2625data,
			DRV2625_REG_OL_PERIOD_H, 0x03, (openLoopPeriod&0x0300)>>8);
		drv2625_reg_write(pDrv2625data,
			DRV2625_REG_OL_PERIOD_L, (openLoopPeriod&0x00ff));

		dev_info(pDrv2625data->dev,
			"%s, LRA = %d, DriveTime=0x%x\n",
			__func__, actuator.mnLRAFreq, DriveTime);
	}
}

static irqreturn_t drv2625_irq_handler(int irq, void *dev_id)
{
	struct drv2625_data *pDrv2625data = (struct drv2625_data *)dev_id;

	pDrv2625data->mnIntStatus =
		drv2625_reg_read(pDrv2625data, DRV2625_REG_STATUS);
	if (pDrv2625data->mnIntStatus & INT_MASK) {
		pDrv2625data->mnWorkMode |= WORK_IRQ;
		schedule_work(&pDrv2625data->vibrator_work);
	}
	return IRQ_HANDLED;
}

static struct regmap_config drv2625_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

#ifdef CONFIG_OF
static int drv2625_parse_dt(struct device *dev,
		struct drv2625_data *pDrv2625data)
{
	struct device_node *node = dev->of_node;
	struct drv2625_platform_data *pDrv2625Platdata =
		&pDrv2625data->msPlatData;
	int err;
	u32 prop_val;

	pDrv2625Platdata->mnGpioNRST = of_get_named_gpio(node,
			"enable-gpio", 0);
	if (pDrv2625Platdata->mnGpioNRST < 0) {
		dev_err(dev, "%s: No entry for enalbe-gpio\n", __func__);
		return pDrv2625Platdata->mnGpioNRST;
	}
	dev_info(dev, "RESET GPIO %d\n", pDrv2625Platdata->mnGpioNRST);

	err = of_property_read_u32(node, "loop", &pDrv2625Platdata->meLoop);
	if (err) {
		dev_err(dev, "%s: No entry for loop type(%d)\n", __func__,err);
		return err;
	}
	dev_info(dev, "LOOP type: %d\n", pDrv2625Platdata->meLoop);

	node = of_get_child_by_name(node, "actuator");
	if (node) {
		err =  of_property_read_u32(node, "actuator-type",
				&pDrv2625Platdata->msActuator.meActuatorType);
		if (err) {
			dev_err(dev, "%s: No entry for actuator-type(%d)\n",
					__func__, err);
			return err;
		}
		dev_info(dev, "Actuator Type: %d\n",
				pDrv2625Platdata->msActuator.meActuatorType);

		err =  of_property_read_u32(node, "rated-volt", &prop_val);
		if (err) {
			dev_err(dev, "%s: No entry for rated-voltage(%d)\n",
					__func__, err);
			return err;
		}
		pDrv2625Platdata->msActuator.mnRatedVoltage = prop_val;
		dev_info(dev, "Actuator Rated Voltage: %d\n",
				pDrv2625Platdata->msActuator.mnRatedVoltage);

		err =  of_property_read_u32(node, "od-clamp-volt", &prop_val);
		if (err) {
			dev_err(dev, "%s: No entry for overdrive clamp voltage(%d)\n",
					__func__, err);
			return err;
		}
		pDrv2625Platdata->msActuator.mnOverDriveClampVoltage = prop_val;
		dev_info(dev, "Actuator OverDriveClampVoltage: %d\n",
			pDrv2625Platdata->msActuator.mnOverDriveClampVoltage);

		err =  of_property_read_u32(node, "lra-freq", &prop_val);
		if (err) {
			dev_err(dev, "%s: No entry for LRA frequnce(%d)\n",
					__func__, err);
			return err;
		}
		pDrv2625Platdata->msActuator.mnLRAFreq = prop_val;
		dev_info(dev, "Actuator LRA Freq: %d\n",
			pDrv2625Platdata->msActuator.mnLRAFreq);

		pDrv2625Platdata->autocal_enabled =
			of_property_read_bool(node, "autocal-enabled");

		dev_info(dev, "Actuator Auto Cal enabled: %d\n",
			pDrv2625Platdata->autocal_enabled);
	} else {
		dev_err(dev, "%s: No node for actuator\n", __func__);
		return -ENODEV;
	}

	return 0;
}
#else
static int drv2625_parse_dt(struct device *dev,
		struct drv2625_data *pDrv2625data)
{
	dev_err(dev, "no platform data defined\n");
	return -EINVAL;
}
#endif


static ssize_t rtp_input_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int rtp_input;

	rtp_input = drv2625_reg_read(pDrv2625data, DRV2625_REG_RTP_INPUT);
	return snprintf(buf, PAGE_SIZE, "%d\n", rtp_input);
}

static ssize_t rtp_input_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;
	char rtp_input;

	dev_info(pDrv2625data->dev, "store rtp amplitude: %s\n", buf);
	ret = kstrtos8(buf, 10, &rtp_input);
	if (ret) {
		dev_err(dev, "Invalid input for rtp_input: ret = %d\n", ret);
		return ret;
	}
	drv2625_reg_write(pDrv2625data, DRV2625_REG_RTP_INPUT, rtp_input);
	return count;
}

static const char * const drv2625_modes[] = {
	"rtp",
	"waveform",
	"diag",
	"autocal",
};

static ssize_t mode_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int mode = drv2625_reg_read(pDrv2625data, DRV2625_REG_MODE)
			& DRV2625_MODE_MASK;

	if (mode >= ARRAY_SIZE(drv2625_modes) || mode < 0) {
		dev_err(dev, "Unexpected mode: mode = %d\n", mode);
		return snprintf(buf, PAGE_SIZE, "%d\n", mode);
	}
	return snprintf(buf, strlen(drv2625_modes[mode]) + 2, "%s\n",
			drv2625_modes[mode]);
}

static ssize_t mode_store(struct device *dev,
			  struct device_attribute *attr, const char *buf,
			  size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	char mode_name[25];
	size_t len;
	unsigned char new_mode;

	mode_name[sizeof(mode_name) - 1] = '\0';
	strlcpy(mode_name, buf, sizeof(mode_name) - 1);
	len = strlen(mode_name);
	if (len && mode_name[len - 1] == '\n')
		mode_name[len - 1] = '\0';

	dev_info(dev, "store mode value: %s\n", mode_name);
	for (new_mode = 0; new_mode < ARRAY_SIZE(drv2625_modes); new_mode++) {
		if (!strcmp(mode_name, drv2625_modes[new_mode]))
			drv2625_change_mode(pDrv2625data, new_mode);
	}
	return count;
}

static ssize_t loop_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int loop;

	loop = drv2625_reg_read(pDrv2625data, DRV2625_REG_MAIN_LOOP) &
		MAIN_LOOP_MASK;
	return snprintf(buf, PAGE_SIZE, "%d\n", loop);
}

static ssize_t loop_store(struct device *dev,
			  struct device_attribute *attr, const char *buf,
			  size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;
	int loop;

	ret = kstrtoint(buf, 10, &loop);
	if (ret) {
		dev_err(dev, "Invalid input for loop: ret = %d\n", ret);
		return ret;
	}
	if (loop < 0 || loop > MAIN_LOOP_MASK) {
		dev_err(dev, "Invalid value (%d) for loop, must be [0..7]\n",
		       loop);
		return -EINVAL;
	}
	drv2625_reg_write(pDrv2625data, DRV2625_REG_MAIN_LOOP, loop);
	return count;
}


static ssize_t interval_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int interval;

	interval = drv2625_reg_read(pDrv2625data, DRV2625_REG_CONTROL2);
	interval = ((interval & INTERVAL_MASK) >> INTERVAL_SHIFT);
	return snprintf(buf, PAGE_SIZE, "%d\n", interval);
}

static ssize_t interval_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;
	int interval;

	ret = kstrtoint(buf, 10, &interval);
	if (ret) {
		dev_err(dev, "Invalid input for loop: ret = %d\n", ret);
		return ret;
	}
	if (interval < 0 || interval > 1) {
		dev_err(dev,
			"Invalid value (%d) for interval, must be 0 or 1\n",
		       interval);
		return -EINVAL;
	}
	drv2625_set_bits(pDrv2625data, DRV2625_REG_CONTROL2, INTERVAL_MASK,
			 interval << INTERVAL_SHIFT);
	return count;
}

static int drv2625_set_waveform(struct drv2625_data *pDrv2625data,
				unsigned char effect[],
				unsigned char loop[])
{
	int ret = 0;
	int i = 0;
	unsigned char seq_loop[2] = { 0 };

	for (i = 0; i < DRV2625_SEQUENCER_SIZE && (effect[i] != 0) ; i++) {
		if (i < 4)
			seq_loop[0] |= (loop[i] << (2 * i));
		else
			seq_loop[1] |= (loop[i] << (2 * (i - 4)));
	}
	if (i == 0)
		ret = drv2625_reg_write(pDrv2625data,
					DRV2625_REG_SEQUENCER_1, 0);
	else
		ret = drv2625_bulk_write(pDrv2625data,
					 DRV2625_REG_SEQUENCER_1,
					 i, effect);

	if (ret < 0) {
		dev_err(pDrv2625data->dev, "error writing effects sequence\n");
		return ret;
	}

	ret = drv2625_reg_write(pDrv2625data,
				DRV2625_REG_SEQ_LOOP_1, seq_loop[0]);
	if (ret < 0)
		dev_err(pDrv2625data->dev, "error writing SEQ_LOOP_1\n");

	ret = drv2625_reg_write(pDrv2625data,
				DRV2625_REG_SEQ_LOOP_2, seq_loop[1]);
	if (ret < 0)
		dev_err(pDrv2625data->dev, "error writing SEQ_LOOP_2\n");

	return ret;
}

static ssize_t scale_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int interval;

	interval = drv2625_reg_read(pDrv2625data, DRV2625_REG_CONTROL2);
	interval = interval & SCALE_MASK;
	return snprintf(buf, PAGE_SIZE, "%d\n", interval);
}

static ssize_t scale_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;
	int scale;

	ret = kstrtoint(buf, 10, &scale);
	if (ret) {
		dev_err(dev, "invalid input for scale: ret = %d\n", ret);
		return ret;
	}
	if (scale < 0 || scale > 3) {
		dev_err(dev, "invalid value (%d) for scale, must be [0..3]\n",
			scale);
		return -EINVAL;
	}
	drv2625_set_bits(pDrv2625data,
			 DRV2625_REG_CONTROL2, SCALE_MASK, scale);
	return count;
}

static ssize_t ctrl_loop_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ctrl_loop;

	ctrl_loop = drv2625_reg_read(pDrv2625data, DRV2625_REG_CONTROL1);
	ctrl_loop = (ctrl_loop & LOOP_MASK) >> LOOP_SHIFT;
	return snprintf(buf, PAGE_SIZE, "%d\n", ctrl_loop);
}

static ssize_t ctrl_loop_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;
	int ctrl_loop;

	ret = kstrtoint(buf, 10, &ctrl_loop);
	if (ret) {
		dev_err(dev, "Invalid input for ctrl_loop: ret = %d\n", ret);
		return ret;
	}
	if (ctrl_loop < 0 || ctrl_loop > 1) {
		dev_err(dev,
			"Invalid input (%d) for ctrl_loop, must be 0 or 1\n",
			ctrl_loop);
		return -EINVAL;
	}
	drv2625_set_bits(pDrv2625data, DRV2625_REG_CONTROL1, LOOP_MASK,
			 ctrl_loop << LOOP_SHIFT);
	return count;
}


static ssize_t set_sequencer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	unsigned char effect[DRV2625_SEQUENCER_SIZE] = { 0 };
	unsigned char loop[DRV2625_SEQUENCER_SIZE] = { 0 };
	int n;

	n = sscanf(buf,
		   "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu "
		   "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
		   &effect[0], &loop[0],
		   &effect[1], &loop[1],
		   &effect[2], &loop[2],
		   &effect[3], &loop[3],
		   &effect[4], &loop[4],
		   &effect[5], &loop[5],
		   &effect[6], &loop[6],
		   &effect[7], &loop[7]);
	if (n > DRV2625_SEQUENCER_SIZE * 2)
		return -EINVAL;

	drv2625_set_waveform(pDrv2625data, effect, loop);
	return count;
}

static ssize_t set_play_waveform(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;

	ret = set_sequencer_store(dev, attr, buf, count);
	if (ret < 0)
		return ret;
	drv2625_change_mode(pDrv2625data, DRV2625_MODE_WAVEFORM_SEQUENCER);
	vibrator_enable(&pDrv2625data->led_dev, LED_FULL);
	return count;
}

static ssize_t od_clamp_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int od_clamp;

	od_clamp = drv2625_reg_read(pDrv2625data, DRV2625_REG_OVERDRIVE_CLAMP);
	return snprintf(buf, PAGE_SIZE, "%d\n", od_clamp);
}

static ssize_t od_clamp_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;
	unsigned char od_clamp;

	ret = kstrtou8(buf, 10, &od_clamp);
	if (ret) {
		dev_err(dev, "Invalid input for od_clamp: ret = %d\n", ret);
		return ret;
	}
	drv2625_reg_write(pDrv2625data, DRV2625_REG_OVERDRIVE_CLAMP, od_clamp);
	return count;
}

static ssize_t diag_result_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int diag_z, diag_k;

	diag_z = drv2625_reg_read(pDrv2625data, DRV2625_REG_DIAG_Z);
	diag_k = drv2625_reg_read(pDrv2625data, DRV2625_REG_DIAG_K);
	return snprintf(buf, PAGE_SIZE, "z=%d k=%d\n", diag_z, diag_k);
}

static ssize_t autocal_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	u32 autocal_comp, autocal_bemf, autocal_gain;

	autocal_comp = drv2625_reg_read(pDrv2625data, DRV2625_REG_CAL_COMP);
	autocal_bemf = drv2625_reg_read(pDrv2625data, DRV2625_REG_CAL_BEMF);
	autocal_gain =
		drv2625_reg_read(pDrv2625data, DRV2625_REG_LOOP_CONTROL) &
		BEMFGAIN_MASK;
	return snprintf(buf, PAGE_SIZE, "comp=%d bemf=%d gain=%d\n",
			autocal_comp, autocal_bemf, autocal_gain);
}

static ssize_t autocal_store(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int n;
	unsigned char comp, bemf, bemfgain;

	n = sscanf(buf, "%hhu %hhu %hhu", &comp, &bemf, &bemfgain);
	if (n != 3) {
		dev_err(dev, "Invalid input, expected: comp bemf gain\n");
		return -EINVAL;
	}
	drv2625_reg_write(pDrv2625data, DRV2625_REG_CAL_COMP, comp);
	drv2625_reg_write(pDrv2625data, DRV2625_REG_CAL_BEMF, bemf);
	drv2625_set_bits(pDrv2625data, DRV2625_REG_LOOP_CONTROL,
			 BEMFGAIN_MASK, bemfgain);
	return count;
}

static ssize_t lra_period_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	u32 hi, lo, lra_period;

	hi = drv2625_reg_read(pDrv2625data, DRV2625_REG_LRA_PERIOD_H);
	lo = drv2625_reg_read(pDrv2625data, DRV2625_REG_LRA_PERIOD_L);
	lra_period = ((hi & 0x03) << 8) | lo;
	return snprintf(buf, PAGE_SIZE, "%d\n", lra_period);
}

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	u32 status;

	status = drv2625_reg_read(pDrv2625data, DRV2625_REG_STATUS);
	return snprintf(buf, PAGE_SIZE,
			"DIAG_RES=%d, PROC_DONE=%d, UVLO=%d, OVER_TEMP=%d, OC_DETECT=%d\n",
			status & (1<<7),
			status & (1<<3),
			status & (1<<2),
			status & (1<<1),
			status & 0x01);
}

static ssize_t ol_lra_period_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	u32 hi, lo, ol_lra_period;

	hi = drv2625_reg_read(pDrv2625data, DRV2625_REG_OL_PERIOD_H);
	lo = drv2625_reg_read(pDrv2625data, DRV2625_REG_OL_PERIOD_L);
	ol_lra_period = ((hi & 0x03) << 8) | lo;
	return snprintf(buf, PAGE_SIZE, "%d\n", ol_lra_period);
}

static ssize_t ol_lra_period_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct drv2625_data *pDrv2625data = dev_get_drvdata(dev);
	int ret;
	u32 ol_lra_period;

	ret = kstrtou32(buf, 10, &ol_lra_period);
	if (ret) {
		dev_err(dev,
			"Invalid input for ol_lra_period: ret = %d\n", ret);
		return ret;
	}
	drv2625_reg_write(pDrv2625data, DRV2625_REG_OL_PERIOD_H,
			  (ol_lra_period >> 8) & 0x03);
	drv2625_reg_write(pDrv2625data, DRV2625_REG_OL_PERIOD_L,
			  ol_lra_period & 0xFF);
	return count;
}

static DEVICE_ATTR(rtp_input, 0660, rtp_input_show, rtp_input_store);
static DEVICE_ATTR(mode, 0660, mode_show, mode_store);
static DEVICE_ATTR(loop, 0660, loop_show, loop_store);
static DEVICE_ATTR(interval, 0660, interval_show, interval_store);
static DEVICE_ATTR(scale, 0660, scale_show, scale_store);
static DEVICE_ATTR(ctrl_loop, 0660, ctrl_loop_show, ctrl_loop_store);
static DEVICE_ATTR(set_sequencer, 0660, NULL, set_sequencer_store);
static DEVICE_ATTR(play_seq, 0660, NULL, set_play_waveform);
static DEVICE_ATTR(od_clamp, 0660, od_clamp_show, od_clamp_store);
static DEVICE_ATTR(diag_result, 0600, diag_result_show, NULL);
static DEVICE_ATTR(autocal, 0660, autocal_show, autocal_store);
static DEVICE_ATTR(lra_period, 0600, lra_period_show, NULL);
static DEVICE_ATTR(status, 0600, status_show, NULL);
static DEVICE_ATTR(ol_lra_period, 0660, ol_lra_period_show,
		   ol_lra_period_store);

static struct attribute *drv2625_fs_attrs[] = {
	&dev_attr_rtp_input.attr,
	&dev_attr_mode.attr,
	&dev_attr_loop.attr,
	&dev_attr_interval.attr,
	&dev_attr_scale.attr,
	&dev_attr_ctrl_loop.attr,
	&dev_attr_set_sequencer.attr,
	&dev_attr_play_seq.attr,
	&dev_attr_od_clamp.attr,
	&dev_attr_diag_result.attr,
	&dev_attr_autocal.attr,
	&dev_attr_lra_period.attr,
	&dev_attr_status.attr,
	&dev_attr_ol_lra_period.attr,
	NULL
};

struct attribute_group drv2625_fs_attr_group = {
	.attrs = drv2625_fs_attrs,
};

static int drv2625_probe(struct i2c_client* client,
		const struct i2c_device_id* id)
{
	struct drv2625_data *pDrv2625data;
	struct drv2625_platform_data *pDrv2625Platdata =
		dev_get_platdata(&client->dev);
	int err = 0;
	unsigned char chipid = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: I2C check failed\n", __func__);
		return -ENODEV;
	}

	pDrv2625data = devm_kzalloc(&client->dev,
			sizeof(struct drv2625_data), GFP_KERNEL);
	if (!pDrv2625data) {
		dev_err(&client->dev, "%s: No memory\n", __func__);
		return -ENOMEM;
	}

	if (IS_ENABLED(CONFIG_OF)) {
		err = drv2625_parse_dt(&client->dev, pDrv2625data);
		if (err) {
			dev_err(&client->dev, "%s: device tree parsing is failed(%d)\n"
					, __func__, err);
			return err;
		}
	} else if (pDrv2625Platdata) {
		memcpy(&pDrv2625data->msPlatData, pDrv2625Platdata,
				sizeof(struct drv2625_platform_data));
	} else {
		dev_err(&client->dev, "%s: Platform data not set\n", __func__);
		err = -ENODEV;
	}

	pDrv2625data->dev = &client->dev;
	pDrv2625data->mpRegmap = devm_regmap_init_i2c(client,
			&drv2625_i2c_regmap);
	if (IS_ERR(pDrv2625data->mpRegmap)) {
		err = PTR_ERR(pDrv2625data->mpRegmap);
		dev_err(pDrv2625data->dev,
			"%s:Failed to allocate register map: %d\n",
			__func__, err);
		return err;
	}

	i2c_set_clientdata(client, pDrv2625data);
	dev_set_drvdata(&client->dev, pDrv2625data);
	pDrv2625data->client = client;

	pDrv2625data->vdd_reg = devm_regulator_get(pDrv2625data->dev,
			"vdd");
	if (IS_ERR(pDrv2625data->vdd_reg)) {
		dev_warn(pDrv2625data->dev, "regulator: VDD request failed\n");
		pDrv2625data->vdd_reg = NULL;
	}
	if (pDrv2625data->vdd_reg) {
		if (regulator_enable(pDrv2625data->vdd_reg))
			dev_warn(pDrv2625data->dev,
					"regulator: VDD enable failed\n");
	}

	if (gpio_is_valid(pDrv2625data->msPlatData.mnGpioNRST)) {
		err = gpio_request_one(pDrv2625data->msPlatData.mnGpioNRST,
				GPIOF_OUT_INIT_HIGH,
				HAPTICS_DEVICE_NAME"NRST");
		if (err < 0) {
			dev_err(pDrv2625data->dev,
				"%s: GPIO %d request RESET error\n",
				__func__, pDrv2625data->msPlatData.mnGpioNRST);
			return err;
		}
		usleep_range(1000, 1100);
	}

	err = drv2625_reg_read(pDrv2625data, DRV2625_REG_ID);
	if (err < 0) {
		dev_err(pDrv2625data->dev,
			"%s, i2c bus fail (%d)\n", __func__, err);
		goto err_gpio_request;
	}
	dev_info(pDrv2625data->dev, "Device ID: 0x%x\n", err);
	pDrv2625data->mnDeviceID = err;

	chipid = pDrv2625data->mnDeviceID & DRV2625_CHIPID_MASK;
	if (chipid != DRV2625_CHIPID){
		dev_err(pDrv2625data->dev,
			"%s, CHIP ID is not matched (0x%x : 0x%x)",
			__func__, chipid, DRV2625_CHIPID);
		goto err_gpio_request;
	}

	dev_init_platform_data(pDrv2625data);

	if (!client->irq) {
		dev_err(pDrv2625data->dev,
			"%s: no irq allocated\n", __func__);
		goto err_gpio_request;
	}
	err = devm_request_threaded_irq(
		pDrv2625data->dev,
		client->irq,
		drv2625_irq_handler, NULL,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		client->name, pDrv2625data);
	if (err < 0) {
		dev_err(pDrv2625data->dev,
			"%s: request_irq failed\n", __func__);
		goto err_gpio_request;
	}
	drv2625_disable_irq(pDrv2625data);

	pDrv2625data->led_dev.name = "vibrator";
	pDrv2625data->led_dev.max_brightness = LED_FULL;
	pDrv2625data->led_dev.brightness_set = vibrator_enable;
	err = led_classdev_register(pDrv2625data->dev, &pDrv2625data->led_dev);
	if (err) {
		dev_err(pDrv2625data->dev,
			"drv2625: fail to create led classdev\n");
		goto err_gpio_request;
	}

	INIT_WORK(&pDrv2625data->vibrator_work, vibrator_work_routine);
	INIT_WORK(&pDrv2625data->haptics_work, drv2625_haptics_work);

	wake_lock_init(&pDrv2625data->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&pDrv2625data->lock);

	err = sysfs_create_group(&pDrv2625data->dev->kobj,
				 &drv2625_fs_attr_group);
	if (err)
		goto err_gpio_request;

	if (pDrv2625data->msPlatData.autocal_enabled)
		drv2625_auto_cal(pDrv2625data);

	dev_info(pDrv2625data->dev, "probe succeeded\n");
	return 0;

err_gpio_request:
	if (gpio_is_valid(pDrv2625data->msPlatData.mnGpioNRST))
		gpio_free(pDrv2625data->msPlatData.mnGpioNRST);

	dev_err(pDrv2625data->dev, "%s failed, err=%d\n",
		__func__, err);
	return err;
}

static int drv2625_remove(struct i2c_client* client)
{
	struct drv2625_data *pDrv2625data = i2c_get_clientdata(client);

	cancel_work_sync(&pDrv2625data->vibrator_work);
	cancel_work_sync(&pDrv2625data->haptics_work);

	debugfs_remove_recursive(pDrv2625data->dent);
	wake_lock_destroy(&pDrv2625data->wklock);
	mutex_destroy(&pDrv2625data->lock);

	if (gpio_is_valid(pDrv2625data->msPlatData.mnGpioNRST))
		gpio_free(pDrv2625data->msPlatData.mnGpioNRST);

	return 0;
}

static struct i2c_device_id drv2625_id_table[] =
{
	{ HAPTICS_DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, drv2625_id_table);

#ifdef CONFIG_OF
static const struct of_device_id drv2625_of_match[] = {
	{ .compatible = "ti,drv2625", },
	{ }
};
MODULE_DEVICE_TABLE(of, drv2625_of_match);
#endif

static struct i2c_driver drv2625_driver =
{
	.driver = {
		.name = HAPTICS_DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(drv2625_of_match),
	},
	.id_table = drv2625_id_table,
	.probe = drv2625_probe,
	.remove = drv2625_remove,
};

static int __init drv2625_init(void)
{
	return i2c_add_driver(&drv2625_driver);
}

static void __exit drv2625_exit(void)
{
	i2c_del_driver(&drv2625_driver);
}

module_init(drv2625_init);
module_exit(drv2625_exit);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
