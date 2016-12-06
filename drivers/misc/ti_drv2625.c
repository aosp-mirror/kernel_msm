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
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
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
#include "../staging/android/timed_output.h"


#define DRV2625_SEQ_MAX_NUM        8
#define DRV2625_AUTOCAL_WAIT_COUNT 5

struct drv2625_platform_data {
	int mnGpioNRST;
	enum loop_type meLoop;
	struct actuator_data msActuator;
	bool autocal_enabled;
};

struct drv2625_data {
	/* to_dev should be first because it is referenced by power_on */
	struct timed_output_dev to_dev;
	struct drv2625_platform_data msPlatData;
	unsigned char mnDeviceID;
	struct device *dev;
	struct regmap *mpRegmap;
	unsigned char mnIntStatus;
	struct drv2625_waveform_sequencer msWaveformSequencer;
	unsigned char mnFileCmd;
	volatile int mnVibratorPlaying;
	volatile char mnWorkMode;
	unsigned char mnCurrentReg;
	struct wake_lock wklock;
	struct hrtimer timer;
	struct mutex lock;
	struct work_struct vibrator_work;
	struct regulator *vdd_reg;
	struct dentry *dent;
};

static struct drv2625_data *g_DRV2625data = NULL;

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

static int drv2625_bulk_read(struct drv2625_data *pDrv2625data,
	unsigned char reg, unsigned int count, u8 *buf)
{
	int ret;
	ret = regmap_bulk_read(pDrv2625data->mpRegmap, reg, buf, count);
	if (ret < 0){
		dev_err(pDrv2625data->dev,
			"%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, ret);
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
	return drv2625_reg_write(pDrv2625data, DRV2625_REG_GO, (val&0x01));
}

static void drv2625_change_mode(struct drv2625_data *pDrv2625data,
		unsigned char work_mode)
{
	drv2625_set_bits(pDrv2625data, DRV2625_REG_MODE, DRV2625_MODE_MASK , work_mode);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct drv2625_data *pDrv2625data =
		container_of(dev, struct drv2625_data, to_dev);

	if (hrtimer_active(&pDrv2625data->timer)) {
		ktime_t r = hrtimer_get_remaining(&pDrv2625data->timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void drv2625_stop(struct drv2625_data *pDrv2625data)
{
	if (pDrv2625data->mnVibratorPlaying == YES){
		hrtimer_cancel(&pDrv2625data->timer);
        	drv2625_change_mode(pDrv2625data, DRV2625_MODE_WAVEFORM_SEQUENCER);
		drv2625_set_go_bit(pDrv2625data, STOP);
		pDrv2625data->mnVibratorPlaying = NO;
		wake_unlock(&pDrv2625data->wklock);
	}
}

static void vibrator_enable( struct timed_output_dev *dev, int value)
{
	struct drv2625_data *pDrv2625data =
		container_of(dev, struct drv2625_data, to_dev);

	mutex_lock(&pDrv2625data->lock);

	pDrv2625data->mnWorkMode = WORK_IDLE;
	drv2625_stop(pDrv2625data);

	if (value > 0) {
		dev_info(pDrv2625data->dev, "vibe for %dms", value);

		wake_lock(&pDrv2625data->wklock);

		drv2625_change_mode(pDrv2625data, DRV2625_MODE_RTP);
		pDrv2625data->mnVibratorPlaying = YES;
		drv2625_set_go_bit(pDrv2625data, GO);
		hrtimer_start(&pDrv2625data->timer,
			ns_to_ktime((u64)value * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);
	}

	mutex_unlock(&pDrv2625data->lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct drv2625_data *pDrv2625data =
		container_of(timer, struct drv2625_data, timer);

	pDrv2625data->mnWorkMode |= WORK_VIBRATOR;
	schedule_work(&pDrv2625data->vibrator_work);

	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2625_data *pDrv2625data =
		container_of(work, struct drv2625_data, vibrator_work);
	unsigned char mode;

	mutex_lock(&pDrv2625data->lock);

	if( pDrv2625data->mnWorkMode & WORK_IRQ){
		unsigned char status = pDrv2625data->mnIntStatus;
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

static int drv2625_set_seq_loop(struct drv2625_data *pDrv2625data,
		unsigned long arg)
{
	int ret = 0, i;
	struct drv2625_seq_loop seqLoop;
	unsigned char halfSize = DRV2625_SEQUENCER_SIZE / 2;
	unsigned char loop[2] = {0, 0};

	if (copy_from_user(&seqLoop,
		(void __user *)arg, sizeof(struct drv2625_seq_loop)))
		return -EFAULT;

	for (i=0; i < DRV2625_SEQUENCER_SIZE; i++) {
		if (i < halfSize)
			loop[0] |= (seqLoop.mpLoop[i] << (i*2));
		else
			loop[1] |= (seqLoop.mpLoop[i] << ((i-halfSize)*2));
	}

	ret = drv2625_bulk_write(pDrv2625data, DRV2625_REG_SEQ_LOOP_1, 2, loop);

	return ret;
}

static int drv2625_set_main(struct drv2625_data *pDrv2625data, unsigned long arg)
{
	int ret = 0;
	struct drv2625_wave_setting mainSetting;
	unsigned char control = 0;

	if (copy_from_user(&mainSetting,
		(void __user *)arg, sizeof(struct drv2625_wave_setting)))
		return -EFAULT;

	control |= mainSetting.meScale;
	control |= (mainSetting.meInterval << INTERVAL_SHIFT);
	drv2625_set_bits(pDrv2625data,
		DRV2625_REG_CONTROL2,
		SCALE_MASK | INTERVAL_MASK,
		control);

	drv2625_set_bits(pDrv2625data,
		DRV2625_REG_MAIN_LOOP,
		0x07, mainSetting.meLoop);

	return ret;
}

static int drv2625_set_wave_seq(struct drv2625_data *pDrv2625data,
		unsigned long arg)
{
	int ret = 0;
	struct drv2625_wave_seq waveSeq;

	if (copy_from_user(&waveSeq,
		(void __user *)arg, sizeof(struct drv2625_wave_seq)))
		return -EFAULT;

	ret = drv2625_bulk_write(pDrv2625data,
		DRV2625_REG_SEQUENCER_1, DRV2625_SEQUENCER_SIZE,
		waveSeq.mpWaveIndex);

	return ret;
}

static int drv2625_get_diag_result(struct drv2625_data *pDrv2625data,
		unsigned long arg)
{
	int ret = 0;
	struct drv2625_diag_result diagResult;
	unsigned char mode, go;

	memset(&diagResult, 0, sizeof(struct drv2625_diag_result));

	mode = drv2625_reg_read(pDrv2625data, DRV2625_REG_MODE) & DRV2625_MODE_MASK;
	if (mode != DRV2625_MODE_DIAGNOSTIC) {
		diagResult.mnFinished = -EFAULT;
		return ret;
	}

	go = drv2625_reg_read(pDrv2625data, DRV2625_REG_GO) & 0x01;
	if (go) {
		diagResult.mnFinished = NO;
	} else {
		diagResult.mnFinished = YES;
		diagResult.mnResult =
			((pDrv2625data->mnIntStatus & DIAG_MASK) >> DIAG_SHIFT);
		diagResult.mnDiagZ = drv2625_reg_read(pDrv2625data, DRV2625_REG_DIAG_Z);
		diagResult.mnDiagK= drv2625_reg_read(pDrv2625data, DRV2625_REG_DIAG_K);
	}

	if (copy_to_user((void __user *)arg, &diagResult, sizeof(struct drv2625_diag_result)))
		return -EFAULT;

	return ret;
}

static int drv2625_get_autocal_result(struct drv2625_data *pDrv2625data,
		unsigned long arg)
{
	int ret = 0;
	struct drv2625_autocal_result autocalResult;
	unsigned char mode, go;

	memset(&autocalResult, 0, sizeof(struct drv2625_autocal_result));

	mode = drv2625_reg_read(pDrv2625data, DRV2625_REG_MODE) & DRV2625_MODE_MASK;
	if (mode != DRV2625_MODE_CALIBRATION) {
		autocalResult.mnFinished = -EFAULT;
		return ret;
	}

	go = drv2625_reg_read(pDrv2625data, DRV2625_REG_GO) & 0x01;
	if (go) {
		autocalResult.mnFinished = NO;
	} else {
		autocalResult.mnFinished = YES;
		autocalResult.mnResult =
			((pDrv2625data->mnIntStatus & DIAG_MASK) >> DIAG_SHIFT);
		autocalResult.mnCalComp = drv2625_reg_read(pDrv2625data, DRV2625_REG_CAL_COMP);
		autocalResult.mnCalBemf = drv2625_reg_read(pDrv2625data, DRV2625_REG_CAL_BEMF);
		autocalResult.mnCalGain =
			drv2625_reg_read(pDrv2625data, DRV2625_REG_CAL_COMP) & BEMFGAIN_MASK;
	}

	if (copy_to_user((void __user *)arg, &autocalResult, sizeof(struct drv2625_autocal_result)))
		return -EFAULT;

	return ret;
}

static int drv2625_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE)) return -ENODEV;

	file->private_data = (void*)g_DRV2625data;
	return 0;
}

static int drv2625_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void*)NULL;
	module_put(THIS_MODULE);

	return 0;
}

static long drv2625_file_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct drv2625_data *pDrv2625data = file->private_data;
	int ret = 0;

	dev_dbg(pDrv2625data->dev, "%s : cmd = %d, arg = %ld\n",
			__func__, cmd, arg);

	mutex_lock(&pDrv2625data->lock);
	switch (cmd) {
		case DRV2625_SET_SEQ_LOOP:
			ret = drv2625_set_seq_loop(pDrv2625data, arg);
		break;

		case DRV2625_SET_MAIN:
			ret = drv2625_set_main(pDrv2625data, arg);
		break;

		case DRV2625_SET_WAV_SEQ:
			ret = drv2625_set_wave_seq(pDrv2625data, arg);
		break;

		case DRV2625_WAVSEQ_PLAY:
		{
			drv2625_stop(pDrv2625data);

			wake_lock(&pDrv2625data->wklock);
			pDrv2625data->mnVibratorPlaying = YES;
			drv2625_change_mode(pDrv2625data, DRV2625_MODE_WAVEFORM_SEQUENCER);
			drv2625_set_go_bit(pDrv2625data, GO);
		}
		break;

		case DRV2625_STOP:
		{
			drv2625_stop(pDrv2625data);
		}
		break;

		case DRV2625_RUN_DIAGNOSTIC:
		{
			drv2625_stop(pDrv2625data);

			wake_lock(&pDrv2625data->wklock);
			pDrv2625data->mnVibratorPlaying = YES;
			drv2625_change_mode(pDrv2625data, DRV2625_MODE_DIAGNOSTIC);
			drv2625_set_go_bit(pDrv2625data, GO);
		}
		break;

		case DRV2625_GET_DIAGRESULT:
			ret = drv2625_get_diag_result(pDrv2625data, arg);
		break;

		case DRV2625_RUN_AUTOCAL:
		{
			drv2625_stop(pDrv2625data);

			wake_lock(&pDrv2625data->wklock);
			pDrv2625data->mnVibratorPlaying = YES;
			drv2625_change_mode(pDrv2625data, DRV2625_MODE_CALIBRATION);
			drv2625_set_go_bit(pDrv2625data, GO);
		}
		break;

		case DRV2625_GET_CALRESULT:
			ret = drv2625_get_autocal_result(pDrv2625data, arg);
		break;
	}

	mutex_unlock(&pDrv2625data->lock);

	return ret;
}

static ssize_t drv2625_file_read(struct file* filp, char* buff, size_t length,
		loff_t* offset)
{
	struct drv2625_data *pDrv2625data =
		(struct drv2625_data *)filp->private_data;
	int ret = 0;
	unsigned char value = 0;
	unsigned char *p_kBuf = NULL;

	dev_dbg(pDrv2625data->dev, "%s : cmd = %d\n",
			__func__, pDrv2625data->mnFileCmd);

	mutex_lock(&pDrv2625data->lock);
	switch(pDrv2625data->mnFileCmd) {
	case HAPTIC_CMDID_REG_READ:
		if (length == 1) {
			ret = drv2625_reg_read(pDrv2625data, pDrv2625data->mnCurrentReg);
			if (0 > ret) {
				dev_err(pDrv2625data->dev, "dev read fail %d\n", ret);
				ret = -EFAULT;
				break;
			}
			value = ret;

			ret = copy_to_user(buff, &value, 1);
			if (0 != ret) {
				/* Failed to copy all the data, exit */
				dev_err(pDrv2625data->dev, "copy to user fail %d\n", ret);
				ret = -EFAULT;
				break;
			}

			ret = length;
		} else if(length > 1) {
			p_kBuf = (unsigned char *)kzalloc(length, GFP_KERNEL);
			if (p_kBuf != NULL) {
				ret = drv2625_bulk_read(pDrv2625data,
					pDrv2625data->mnCurrentReg, length, p_kBuf);
				if (0 > ret) {
					dev_err(pDrv2625data->dev, "dev bulk read fail %d\n", ret);
					ret = -EFAULT;
				} else {
					ret = copy_to_user(buff, p_kBuf, length);
					if (0 != ret) {
						/* Failed to copy all the data, exit */
						dev_err(pDrv2625data->dev, "copy to user fail %d\n", ret);
						ret = -EFAULT;
					} else {
						ret = length;
					}
				}

				kfree(p_kBuf);
			} else {
				dev_err(pDrv2625data->dev, "read no mem\n");
				ret = -ENOMEM;
			}
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		pDrv2625data->mnFileCmd = 0;
		break;
	}
	mutex_unlock(&pDrv2625data->lock);

    return ret;
}

static ssize_t drv2625_file_write(struct file* filp, const char* buff,
		size_t len, loff_t* off)
{
	int ret = len;
	struct drv2625_data *pDrv2625data =
		(struct drv2625_data *)filp->private_data;
	int i;

	if (len < 1)
		return -EINVAL;

	dev_dbg(pDrv2625data->dev, "%s : buff = %s\n", __func__, buff);

	mutex_lock(&pDrv2625data->lock);

	pDrv2625data->mnFileCmd = buff[0];

	switch(pDrv2625data->mnFileCmd) {
	case HAPTIC_CMDID_REG_READ:
		if(len == 2){
			pDrv2625data->mnCurrentReg = buff[1];
		} else {
			dev_err(pDrv2625data->dev,
				" read cmd len %d err\n", len);
			ret = -EINVAL;
		}
		break;

	case HAPTIC_CMDID_REG_WRITE:
		if ((len-1) == 2) {
			drv2625_reg_write(pDrv2625data, buff[1], buff[2]);
		} else if((len-1)>2) {
			unsigned char *data =
				(unsigned char *)kzalloc(len-2, GFP_KERNEL);
			if(data != NULL){
				if (copy_from_user(data, &buff[2], len-2) != 0) {
					dev_err(pDrv2625data->dev,
						"%s, reg copy err\n", __func__);
					ret = -EFAULT;
				} else {
					drv2625_bulk_write(pDrv2625data, buff[1], len-2, data);
				}
				kfree(data);
			} else {
				dev_err(pDrv2625data->dev, "memory fail\n");
				ret = -ENOMEM;
			}
		} else {
			dev_err(pDrv2625data->dev,
				"%s, reg_write len %d error\n", __func__, len);
			ret = -EINVAL;
		}
		break;

	case HAPTIC_CMDID_REG_SETBIT:
		for (i = 1; i < len; ) {
			drv2625_set_bits(pDrv2625data, buff[i], buff[i+1],
					buff[i+2]);
			i += 3;
		}
		break;
	default:
		dev_err(pDrv2625data->dev,
			"%s, unknown cmd\n", __func__);
		break;
	}

	mutex_unlock(&pDrv2625data->lock);

	return ret;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.read = drv2625_file_read,
	.write = drv2625_file_write,
	.unlocked_ioctl = drv2625_file_unlocked_ioctl,
	.open = drv2625_file_open,
	.release = drv2625_file_release,
};

static struct miscdevice drv2625_misc =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = HAPTICS_DEVICE_NAME,
	.fops = &fops,
};

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
		drv2625_reg_read(pDrv2625data,DRV2625_REG_STATUS);
	if(pDrv2625data->mnIntStatus & INT_MASK){
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

static int drv2625_debugfs_set_go(void *data, u64 val)
{
	struct drv2625_data *pDrv2625data = data;

	return drv2625_set_go_bit(pDrv2625data, (unsigned char)val);
}

static int drv2625_debugfs_get_go(void *data, u64 *val)
{
	int ret;
	struct drv2625_data *pDrv2625data = data;

	ret = drv2625_reg_read(pDrv2625data, DRV2625_REG_GO) & 0x01;
	if (ret >= 0) {
		*val = (u64)ret;
		ret = 0;
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(drv2625_debugfs_go_fops,
		drv2625_debugfs_get_go,
		drv2625_debugfs_set_go,
		"%llu\n");

static int drv2625_debugfs_set_mode(void *data, u64 val)
{
	struct drv2625_data *pDrv2625data = data;

	drv2625_change_mode(pDrv2625data, (unsigned char)val);

	return 0;
}

static int drv2625_debugfs_get_mode(void *data, u64 *val)
{
	int ret;
	struct drv2625_data *pDrv2625data = data;

	ret = drv2625_reg_read(pDrv2625data, DRV2625_REG_MODE) &
		DRV2625_MODE_MASK;
	if (ret >= 0) {
		*val = (u64)ret;
		ret = 0;
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(drv2625_debugfs_mode_fops,
		drv2625_debugfs_get_mode,
		drv2625_debugfs_set_mode,
		"%llu\n");

static int drv2625_debugfs_seq_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return nonseekable_open(inode, file);
}

static ssize_t drv2625_debugfs_seq_write(struct file *file, const char *buf,
		size_t len, loff_t *off)
{
	struct drv2625_data *pDrv2625data =
		(struct drv2625_data *)file->private_data;
	unsigned char args[2] = {0, }; /* seq_num, effect_num */
	char *temp, *token;
	size_t size;
	int i = 0;
	int ret;

	size = len + 1;
	size = min(size, (size_t)PAGE_SIZE);
	temp = kmemdup(buf, size, GFP_KERNEL);
	if (!temp) {
		dev_err(pDrv2625data->dev, "%s: no mem\n", __func__);
		return -ENOMEM;
	}
	temp[size-1] = '\0';

	while (temp) {
		token = strsep(&temp, " ");
		if (!isalnum(*token))
			continue;

		if (i >= 2) {
			dev_err(pDrv2625data->dev, "%s: too many arguments\n",
					__func__);
			ret = -EINVAL;
			goto error;
		}

		ret = kstrtou8(token, 0, &args[i++]);
		if (ret < 0) {
			dev_err(pDrv2625data->dev, "%s: invalid value\n",
					__func__);
			goto error;
		}
	}

	if (i != 2 || args[0] == 0 || args[0] > DRV2625_SEQ_MAX_NUM) {
		dev_err(pDrv2625data->dev, "%s: invalid arguments\n", __func__);
		ret = -EINVAL;
		goto error;
	}

	ret = drv2625_reg_write(pDrv2625data,
			DRV2625_REG_SEQUENCER_1 + (args[0] - 1),
			args[1]);
	if (ret)
		goto error;

	dev_info(pDrv2625data->dev, "WAV_FRM_SEQ%u => %u\n",
			args[0], args[1]);
	kfree(temp);
	return len;

error:
	kfree(temp);
	return ret;
}

static int drv2625_debugfs_seq_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations drv2625_debugfs_seq_fops = {
	.open    = drv2625_debugfs_seq_open,
	.release = drv2625_debugfs_seq_release,
	.write   = drv2625_debugfs_seq_write,
	.llseek  = no_llseek,
};

static int drv2625_debugfs_set_amp(void *data, u64 val)
{
	struct drv2625_data *pDrv2625data = data;
	int ret;

	if (val > 127) {
		dev_err(pDrv2625data->dev, "Invalid amplitude supplied\n");
		return -EINVAL;
	}

	ret = drv2625_reg_write(pDrv2625data, DRV2625_REG_RTP_INPUT,
		(uint8_t)val);
	if (ret)
		dev_err(pDrv2625data->dev, "Error writing amplitude\n");

	return ret;
}

static int drv2625_debugfs_get_amp(void *data, u64 *val)
{
	int ret;
	struct drv2625_data *pDrv2625data = data;

	ret = drv2625_reg_read(pDrv2625data, DRV2625_REG_RTP_INPUT);
	if (ret >= 0) {
		*val = (u64)ret;
		ret = 0;
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(drv2625_debugfs_amp_fops,
		drv2625_debugfs_get_amp,
		drv2625_debugfs_set_amp,
		"%llu\n");

static void drv2625_create_debugfs_entries(
		struct drv2625_data *pDrv2625data)
{
	struct dentry *file;

	pDrv2625data->dent = debugfs_create_dir(HAPTICS_DEVICE_NAME, NULL);
	if (IS_ERR(pDrv2625data->dent)) {
		dev_err(pDrv2625data->dev,
				"%s: %s driver couldn't create debugfs\n",
				__func__, HAPTICS_DEVICE_NAME);
		return;
	}

	file = debugfs_create_file("go", S_IRUSR | S_IWUSR, pDrv2625data->dent,
			(void *)pDrv2625data, &drv2625_debugfs_go_fops);
	if (IS_ERR(file)) {
		dev_err(pDrv2625data->dev, "%s: %s couldn't create go node\n",
				 __func__, HAPTICS_DEVICE_NAME);
		return;
	}

	file = debugfs_create_file("mode", S_IRUSR | S_IWUSR,
			pDrv2625data->dent, (void *)pDrv2625data,
			&drv2625_debugfs_mode_fops);
	if (IS_ERR(file)) {
		dev_err(pDrv2625data->dev, "%s: %s couldn't create mode node\n",
				 __func__, HAPTICS_DEVICE_NAME);
		return;
	}

	file = debugfs_create_file("seq", S_IWUSR, pDrv2625data->dent,
			(void *)pDrv2625data, &drv2625_debugfs_seq_fops);
	if (IS_ERR(file)) {
		dev_err(pDrv2625data->dev, "%s: %s couldn't create seq node\n",
				 __func__, HAPTICS_DEVICE_NAME);
		return;
	}

	file = debugfs_create_file("amp", S_IRUSR | S_IWUSR,
			pDrv2625data->dent, (void *)pDrv2625data,
			&drv2625_debugfs_amp_fops);
	if (IS_ERR(file)) {
		dev_err(pDrv2625data->dev, "%s: %s couldn't create amp node\n",
				 __func__, HAPTICS_DEVICE_NAME);
		return;
	}
}

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

	g_DRV2625data = pDrv2625data;

	/* register timed_out device */
	pDrv2625data->to_dev.name = "vibrator";
	pDrv2625data->to_dev.get_time = vibrator_get_time;
	pDrv2625data->to_dev.enable = vibrator_enable;
	err = timed_output_dev_register(&pDrv2625data->to_dev);
	if (err < 0) {
		dev_err(pDrv2625data->dev,
			"%s: fail to create timed output dev\n", __func__);
		goto err_gpio_request;
	}

	err = misc_register(&drv2625_misc);
	if (err) {
		dev_err(pDrv2625data->dev,
			"drv2625 misc fail: %d\n", err);
		goto err_misc_register;
	}

	hrtimer_init(&pDrv2625data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pDrv2625data->timer.function = vibrator_timer_func;
	INIT_WORK(&pDrv2625data->vibrator_work, vibrator_work_routine);

	wake_lock_init(&pDrv2625data->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&pDrv2625data->lock);

	drv2625_create_debugfs_entries(pDrv2625data);

	if (pDrv2625data->msPlatData.autocal_enabled)
		drv2625_auto_cal(pDrv2625data);

	dev_info(pDrv2625data->dev, "probe succeeded\n");
	return 0;

err_misc_register:
	timed_output_dev_unregister(&pDrv2625data->to_dev);
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

	debugfs_remove_recursive(pDrv2625data->dent);
	wake_lock_destroy(&pDrv2625data->wklock);
	mutex_destroy(&pDrv2625data->lock);
	timed_output_dev_unregister(&pDrv2625data->to_dev);

	misc_deregister(&drv2625_misc);

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
