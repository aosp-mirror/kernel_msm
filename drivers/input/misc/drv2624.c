/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** File:
**     drv2624.c
**
** Description:
**     DRV2624 chip driver
**
** =============================================================================
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include "drv2624.h"

static struct drv2624_data *drv2624_plat_data;

static int drv2624_reg_read(struct drv2624_data *drv2624, unsigned char reg)
{
	unsigned int val;
	int ret;

	mutex_lock(&drv2624->dev_lock);
	ret = regmap_read(drv2624->regmap, reg, &val);
	mutex_unlock(&drv2624->dev_lock);

	if (ret < 0) {
		dev_err(drv2624->dev,
			"%s reg=0x%x error %d\n", __func__, reg, ret);
		return ret;
	}

	dev_dbg(drv2624->dev, "%s, Reg[0x%x]=0x%x\n", __func__, reg, val);

	return val;
}

static int drv2624_reg_write(struct drv2624_data *drv2624,
			     unsigned char reg, unsigned char val)
{
	int ret;

	mutex_lock(&drv2624->dev_lock);
	ret = regmap_write(drv2624->regmap, reg, val);
	mutex_unlock(&drv2624->dev_lock);

	if (ret < 0) {
		dev_err(drv2624->dev,
			"%s reg=0x%x, value=0%x error %d\n",
			__func__, reg, val, ret);
	} else {
		dev_dbg(drv2624->dev, "%s, Reg[0x%x]=0x%x\n",
			__func__, reg, val);
	}

	return ret;
}

static int drv2624_bulk_read(struct drv2624_data *drv2624,
			     unsigned char reg, u8 *buf, unsigned int count)
{
	int ret;

	mutex_lock(&drv2624->dev_lock);
	ret = regmap_bulk_read(drv2624->regmap, reg, buf, count);
	mutex_unlock(&drv2624->dev_lock);

	if (ret < 0) {
		dev_err(drv2624->dev,
			"%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, ret);
	}

	return ret;
}

static int drv2624_bulk_write(struct drv2624_data *drv2624,
			      unsigned char reg, const u8 *buf,
			      unsigned int count)
{
	int ret, i;

	mutex_lock(&drv2624->dev_lock);
	ret = regmap_bulk_write(drv2624->regmap, reg, buf, count);
	mutex_unlock(&drv2624->dev_lock);

	if (ret < 0) {
		dev_err(drv2624->dev,
			"%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, ret);
	} else {
		for (i = 0; i < count; i++)
			dev_dbg(drv2624->dev, "%s, Reg[0x%x]=0x%x\n",
				__func__, reg + i, buf[i]);
	}
	return ret;
}

static int drv2624_set_bits(struct drv2624_data *drv2624,
			    unsigned char reg, unsigned char mask,
			    unsigned char val)
{
	int ret;

	mutex_lock(&drv2624->dev_lock);
	ret = regmap_update_bits(drv2624->regmap, reg, mask, val);
	mutex_unlock(&drv2624->dev_lock);

	if (ret < 0) {
		dev_err(drv2624->dev,
			"%s reg=%x, mask=0x%x, value=0x%x error %d\n",
			__func__, reg, mask, val, ret);
	} else {
		dev_dbg(drv2624->dev, "%s, Reg[0x%x]:M=0x%x, V=0x%x\n",
			__func__, reg, mask, val);
	}

	return ret;
}

static void drv2624_enable_irq(struct drv2624_data *drv2624, bool rtp)
{
	unsigned char mask = INT_ENABLE_CRITICAL;

	if (rtp)
		mask = INT_ENABLE_ALL;

	drv2624_reg_read(drv2624, DRV2624_REG_STATUS);
	drv2624_reg_write(drv2624, DRV2624_REG_INT_ENABLE, mask);

	enable_irq(drv2624->irq);
}

static void drv2624_disable_irq(struct drv2624_data *drv2624)
{
	disable_irq(drv2624->irq);
	drv2624_reg_write(drv2624, DRV2624_REG_INT_ENABLE, INT_MASK_ALL);
}

static int drv2624_set_go_bit(struct drv2624_data *drv2624, unsigned char val)
{
	int ret = 0, value = 0;
	int retry = 10; /* to finish auto-brake */

	val &= 0x01;
	ret = drv2624_reg_write(drv2624, DRV2624_REG_GO, val);
	if (ret >= 0) {
		if (val == 1) {
			usleep_range(1000, 2000);
			value = drv2624_reg_read(drv2624, DRV2624_REG_GO);
			if (value < 0) {
				ret = value;
			} else if (value != 1) {
				ret = -1;
				dev_warn(drv2624->dev,
					 "%s, GO fail, stop action\n",
					 __func__);
			}
		} else {
			while (retry > 0) {
				value =
				    drv2624_reg_read(drv2624, DRV2624_REG_GO);
				if (value < 0) {
					ret = value;
					break;
				}

				if (value == 0)
					break;
				usleep_range(10000, 20000);
				retry--;
			}

			if (retry == 0) {
				dev_err(drv2624->dev,
					"%s, ERROR: clear GO fail\n",
					__func__);
			}
		}
	}

	return ret;
}

static void drv2624_change_mode(struct drv2624_data *drv2624,
				unsigned char work_mode)
{
	drv2624_set_bits(drv2624, DRV2624_REG_MODE, WORKMODE_MASK, work_mode);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct drv2624_data *drv2624 =
	    container_of(dev, struct drv2624_data, to_dev);

	if (hrtimer_active(&drv2624->timer)) {
		ktime_t r = hrtimer_get_remaining(&drv2624->timer);

		return ktime_to_ms(r);
	}

	return 0;
}

static void drv2624_stop(struct drv2624_data *drv2624)
{
	if (drv2624->vibrator_playing) {
		dev_dbg(drv2624->dev, "%s\n", __func__);
		drv2624_disable_irq(drv2624);
		hrtimer_cancel(&drv2624->timer);
		drv2624_set_go_bit(drv2624, STOP);
		drv2624->vibrator_playing = false;
		wake_unlock(&drv2624->wklock);
	}
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	int ret = 0;
	struct drv2624_data *drv2624 =
	    container_of(dev, struct drv2624_data, to_dev);

	dev_dbg(drv2624->dev, "%s, value=%d\n", __func__, value);

	mutex_lock(&drv2624->lock);

	drv2624->work_mode = WORK_IDLE;
	dev_dbg(drv2624->dev, "%s, afer mnWorkMode=0x%x\n",
		__func__, drv2624->work_mode);

	drv2624_stop(drv2624);

	if (value > 0) {
		wake_lock(&drv2624->wklock);

		drv2624_change_mode(drv2624, MODE_RTP);
		drv2624->vibrator_playing = true;
		drv2624_enable_irq(drv2624, true);
		ret = drv2624_set_go_bit(drv2624, GO);
		if (ret < 0) {
			dev_warn(drv2624->dev, "Start RTP failed\n");
			wake_unlock(&drv2624->wklock);
			drv2624->vibrator_playing = false;
			drv2624_disable_irq(drv2624);
		} else {
			value = (value > MAX_TIMEOUT) ? MAX_TIMEOUT : value;
			hrtimer_start(&drv2624->timer,
				      ns_to_ktime((u64)value * NSEC_PER_MSEC),
				      HRTIMER_MODE_REL);
		}
	}

	mutex_unlock(&drv2624->lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct drv2624_data *drv2624 =
	    container_of(timer, struct drv2624_data, timer);

	dev_dbg(drv2624->dev, "%s\n", __func__);
	drv2624->work_mode |= WORK_VIBRATOR;
	schedule_work(&drv2624->vibrator_work);
	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2624_data *drv2624 =
	    container_of(work, struct drv2624_data, vibrator_work);
	unsigned char mode = MODE_RTP;
	unsigned char status;
	int ret = 0;

	mutex_lock(&drv2624->lock);

	dev_dbg(drv2624->dev, "%s, afer mnWorkMode=0x%x\n",
		__func__, drv2624->work_mode);

	if (drv2624->work_mode & WORK_IRQ) {
		ret = drv2624_reg_read(drv2624, DRV2624_REG_STATUS);
		if (ret >= 0)
			drv2624->int_status = ret;

		drv2624_disable_irq(drv2624);

		if (ret < 0) {
			dev_err(drv2624->dev,
				"%s, reg read error\n", __func__);
			goto err;
		}

		status = drv2624->int_status;
		dev_dbg(drv2624->dev, "%s, status=0x%x\n",
			__func__, drv2624->int_status);

		if (status & OVERCURRENT_MASK) {
			dev_err(drv2624->dev,
				"ERROR, Over Current detected!!\n");
		}

		if (status & OVERTEMPRATURE_MASK) {
			dev_err(drv2624->dev,
				"ERROR, Over Temperature detected!!\n");
		}

		if (status & ULVO_MASK)
			dev_err(drv2624->dev, "ERROR, VDD drop observed!!\n");

		if (status & PRG_ERR_MASK)
			dev_err(drv2624->dev, "ERROR, PRG error!!\n");

		if (status & PROCESS_DONE_MASK) {
			ret = drv2624_reg_read(drv2624, DRV2624_REG_MODE);
			if (ret < 0) {
				dev_err(drv2624->dev,
					"%s, reg read error\n", __func__);
				goto err;
			}

			mode = ret & WORKMODE_MASK;
			if (mode == MODE_CALIBRATION) {
				drv2624->auto_cal_result.result = status;
				if ((status & DIAG_MASK) != DIAG_SUCCESS) {
					dev_err(drv2624->dev,
						"Calibration fail\n");
				} else {
					drv2624->auto_cal_result.cal_comp =
					    drv2624_reg_read(drv2624,
							     DRV2624_REG_CAL_COMP);
					drv2624->auto_cal_result.cal_bemf =
					    drv2624_reg_read(drv2624,
							     DRV2624_REG_CAL_BEMF);
					drv2624->auto_cal_result.cal_gain =
					    drv2624_reg_read(drv2624,
							     DRV2624_REG_CAL_COMP)
					    & BEMFGAIN_MASK;
					dev_dbg(drv2624->dev,
						"AutoCal : Comp=0x%x, Bemf=0x%x, Gain=0x%x\n",
						drv2624->auto_cal_result.cal_comp,
						drv2624->auto_cal_result.cal_bemf,
						drv2624->auto_cal_result.cal_gain);
				}
			} else if (mode == MODE_DIAGNOSTIC) {
				drv2624->diag_result.result = status;
				if ((status & DIAG_MASK) != DIAG_SUCCESS) {
					dev_err(drv2624->dev,
						"Diagnostic fail\n");
				} else {
					drv2624->diag_result.diagz =
					    drv2624_reg_read(drv2624,
							     DRV2624_REG_DIAG_Z);
					drv2624->diag_result.diagk =
					    drv2624_reg_read(drv2624,
							     DRV2624_REG_DIAG_K);
					dev_dbg(drv2624->dev,
						"Diag : ZResult=0x%x, CurrentK=0x%x\n",
						drv2624->diag_result.diagz,
						drv2624->diag_result.diagk);
				}
			} else if (mode == MODE_WAVEFORM_SEQUENCER) {
				dev_dbg(drv2624->dev,
					"Waveform Sequencer Playback finished\n");
			} else if (mode == MODE_RTP) {
				dev_dbg(drv2624->dev, "RTP IRQ\n");
			}
		}

		if ((mode != MODE_RTP) && drv2624->vibrator_playing) {
			dev_info(drv2624->dev, "release wklock\n");
			drv2624->vibrator_playing = false;
			wake_unlock(&drv2624->wklock);
		}

		drv2624->work_mode &= ~WORK_IRQ;
	}

	if (drv2624->work_mode & WORK_VIBRATOR) {
		drv2624_stop(drv2624);
		drv2624->work_mode &= ~WORK_VIBRATOR;
	}
err:

	mutex_unlock(&drv2624->lock);
}

static int dev_auto_calibrate(struct drv2624_data *drv2624)
{
	int ret = 0;

	dev_info(drv2624->dev, "%s\n", __func__);

	wake_lock(&drv2624->wklock);
	drv2624->vibrator_playing = true;
	drv2624_change_mode(drv2624, MODE_CALIBRATION);

	ret = drv2624_set_go_bit(drv2624, GO);
	if (ret < 0) {
		dev_warn(drv2624->dev, "calibration start fail\n");
		wake_unlock(&drv2624->wklock);
		drv2624->vibrator_playing = false;
	} else {
		dev_dbg(drv2624->dev, "calibration start\n");
	}

	return ret;
}

static int dev_run_diagnostics(struct drv2624_data *drv2624)
{
	int ret = 0;

	dev_info(drv2624->dev, "%s\n", __func__);

	wake_lock(&drv2624->wklock);
	drv2624->vibrator_playing = true;
	drv2624_change_mode(drv2624, MODE_DIAGNOSTIC);

	ret = drv2624_set_go_bit(drv2624, GO);
	if (ret < 0) {
		dev_warn(drv2624->dev, "Diag start fail\n");
		wake_unlock(&drv2624->wklock);
		drv2624->vibrator_playing = false;
	} else {
		dev_dbg(drv2624->dev, "Diag start\n");
	}

	return ret;
}

static int drv2624_play_effect(struct drv2624_data *drv2624)
{
	int ret = 0;

	dev_info(drv2624->dev, "%s\n", __func__);

	wake_lock(&drv2624->wklock);
	drv2624->vibrator_playing = true;
	drv2624_change_mode(drv2624, MODE_WAVEFORM_SEQUENCER);

	ret = drv2624_set_go_bit(drv2624, GO);
	if (ret < 0) {
		dev_warn(drv2624->dev, "effects start fail\n");
		wake_unlock(&drv2624->wklock);
		drv2624->vibrator_playing = false;
	} else {
		dev_dbg(drv2624->dev, "effects start\n");
	}

	return ret;
}

static int drv2624_config_waveform(struct drv2624_data *drv2624,
				   struct drv2624_wave_setting *psetting)
{
	int ret = 0;
	int value = 0;

	ret = drv2624_reg_write(drv2624,
				DRV2624_REG_MAIN_LOOP, psetting->loop & 0x07);
	if (ret >= 0) {
		value |= ((psetting->interval & 0x01) << INTERVAL_SHIFT);
		value |= (psetting->scale & 0x03);
		drv2624_set_bits(drv2624, DRV2624_REG_CONTROL2,
				 INTERVAL_MASK | SCALE_MASK, value);
	}
	return ret;
}

static int drv2624_set_waveform(struct drv2624_data *drv2624,
				struct drv2624_waveform_sequencer *sequencer)
{
	int ret = 0;
	int i = 0;
	unsigned char loop[2] = { 0 };
	unsigned char effects[DRV2624_SEQUENCER_SIZE] = { 0 };
	unsigned char len = 0;

	for (i = 0; i < DRV2624_SEQUENCER_SIZE; i++) {
		len++;
		if (sequencer->waveform[i].effect != 0) {
			if (i < 4)
				loop[0] |=
				    (sequencer->waveform[i].loop << (2 * i));
			else
				loop[1] |=
				    (sequencer->waveform[i].loop << (2 * (i - 4)));

			effects[i] = sequencer->waveform[i].effect;
		} else {
			break;
		}
	}

	if (len == 1)
		ret = drv2624_reg_write(drv2624, DRV2624_REG_SEQUENCER_1, 0);
	else
		ret =
		    drv2624_bulk_write(drv2624, DRV2624_REG_SEQUENCER_1,
				       effects, len);

	if (ret < 0)
		dev_err(drv2624->dev, "sequence error\n");

	if ((ret >= 0) && (len > 1)) {
		if ((len - 1) <= 4)
			drv2624_reg_write(drv2624,
					  DRV2624_REG_SEQ_LOOP_1, loop[0]);
		else
			drv2624_bulk_write(drv2624,
					   DRV2624_REG_SEQ_LOOP_1, loop, 2);
	}

	return ret;
}

static int fw_chksum(const struct firmware *fw)
{
	int sum = 0;
	int i = 0;
	int size = fw->size;
	const unsigned char *buffer = fw->data;

	for (i = 0; i < size; i++) {
		if (!((i > 11) && (i < 16)))
			sum += buffer[i];
	}

	return sum;
}

/* drv2624_firmware_load:   This function is called by the
 *		request_firmware_nowait function as soon
 *		as the firmware has been loaded from the file.
 *		The firmware structure contains the data and$
 *		the size of the firmware loaded.
 * @fw: pointer to firmware file to be dowloaded
 * @context: pointer variable to drv2624 data
 *
 *
 */
static void drv2624_firmware_load(const struct firmware *fw, void *context)
{
	struct drv2624_data *drv2624 = context;
	int size = 0, fwsize = 0, i = 0;
	const unsigned char *buffer = NULL;

	if (!context)
		return;

	mutex_lock(&drv2624->lock);

	if (fw) {
		buffer = fw->data;
		size = fw->size;
		if (size > 1024) {
			dev_err(drv2624->dev,
				"%s, ERROR!! firmware size %d too big\n",
				__func__, size);
		} else {
			memcpy(&drv2624->fw_header, buffer,
			       sizeof(struct drv2624_fw_header));
			if ((drv2624->fw_header.fw_magic != DRV2624_MAGIC) ||
			    (drv2624->fw_header.fw_size != size) ||
			    (drv2624->fw_header.fw_chksum != fw_chksum(fw))) {
				dev_err(drv2624->dev,
					"%s, ERROR!! firmware not right:Magic=0x%x,Size=%d,chksum=0x%x\n",
					__func__,
					drv2624->fw_header.fw_magic,
					drv2624->fw_header.fw_size,
					drv2624->fw_header.fw_chksum);
			} else {
				dev_info(drv2624->dev, "%s, firmware good\n",
					 __func__);

				buffer += sizeof(struct drv2624_fw_header);

				drv2624_reg_write(drv2624,
						  DRV2624_REG_RAM_ADDR_UPPER,
						  0);
				drv2624_reg_write(drv2624,
						  DRV2624_REG_RAM_ADDR_LOWER,
						  0);

				fwsize =
				    size - sizeof(struct drv2624_fw_header);

				for (i = 0; i < fwsize; i++) {
					drv2624_reg_write(drv2624,
							  DRV2624_REG_RAM_DATA,
							  buffer[i]);
				}
			}
		}
	} else {
		dev_err(drv2624->dev, "%s, ERROR!! firmware not found\n",
			__func__);
	}

	release_firmware(fw);

	mutex_unlock(&drv2624->lock);
}

static int drv2624_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)drv2624_plat_data;
	return 0;
}

static int drv2624_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;
	module_put(THIS_MODULE);
	return 0;
}

static long drv2624_file_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	struct drv2624_data *drv2624 = file->private_data;
	int ret = 0;

	mutex_lock(&drv2624->lock);

	dev_dbg(drv2624->dev, "ioctl 0x%x\n", cmd);

	mutex_unlock(&drv2624->lock);

	return ret;
}

static ssize_t drv2624_file_read(struct file *filp, char *buff, size_t length,
				 loff_t *offset)
{
	struct drv2624_data *drv2624 =
	    (struct drv2624_data *)filp->private_data;
	int ret = 0;
	unsigned char value = 0;
	unsigned char *buffer = NULL;

	mutex_lock(&drv2624->lock);

	switch (drv2624->file_cmd) {
	case HAPTIC_CMDID_REG_READ:
		{
			if (length == 1) {
				ret =
				    drv2624_reg_read(drv2624,
						     drv2624->current_reg);
				if (ret < 0) {
					dev_err(drv2624->dev,
						"dev read fail %d\n", ret);
				} else {
					value = ret;
					ret = copy_to_user(buff, &value, 1);
					if (ret) {
						/* Failed to copy all the data, exit */
						dev_err(drv2624->dev,
							"copy to user fail %d\n",
							ret);
					}
				}
			} else if (length > 1) {
				buffer = kzalloc(length, GFP_KERNEL);
				if (buffer) {
					ret = drv2624_bulk_read(drv2624,
								drv2624->current_reg,
								buffer, length);
					if (ret < 0) {
						dev_err(drv2624->dev,
							"dev bulk read fail %d\n",
							ret);
					} else {
						ret =
						    copy_to_user(buff, buffer,
								 length);
						if (ret) {
							/* Failed to copy all the data, exit */
							dev_err(drv2624->dev,
								"copy to user fail %d\n",
								ret);
						}
					}

					kfree(buffer);
				} else {
					dev_err(drv2624->dev, "read no mem\n");
					ret = -ENOMEM;
				}
			}
			break;
		}

	case HAPTIC_CMDID_RUN_DIAG:
		{
			if (drv2624->vibrator_playing) {
				length = 0;
			} else {
				unsigned char buf[3];

				buf[0] = drv2624->diag_result.result;
				buf[1] = drv2624->diag_result.diagz;
				buf[2] = drv2624->diag_result.diagk;

				ret = copy_to_user(buff, buf, 3);
				if (ret) {
					/* Failed to copy all the data, exit */
					dev_err(drv2624->dev,
						"copy to user fail %d\n", ret);
				}
			}
			break;
		}

	case HAPTIC_CMDID_RUN_CALIBRATION:
		{
			if (drv2624->vibrator_playing) {
				length = 0;
			} else {
				unsigned char buf[4];

				buf[0] = drv2624->auto_cal_result.result;
				buf[1] = drv2624->auto_cal_result.cal_comp;
				buf[2] = drv2624->auto_cal_result.cal_bemf;
				buf[3] = drv2624->auto_cal_result.cal_gain;

				ret = copy_to_user(buff, buf, 4);
				if (ret) {
					/* Failed to copy all the data, exit */
					dev_err(drv2624->dev,
						"copy to user fail %d\n", ret);
				}
			}
			break;
		}

	case HAPTIC_CMDID_CONFIG_WAVEFORM:
		{
			if (length == sizeof(struct drv2624_wave_setting)) {
				struct drv2624_wave_setting wavesetting;
				value =
				    drv2624_reg_read(drv2624,
						     DRV2624_REG_CONTROL2);
				wavesetting.loop =
				    drv2624_reg_read(drv2624,
						     DRV2624_REG_MAIN_LOOP) &
				    0x07;
				wavesetting.interval =
				    ((value & INTERVAL_MASK) >> INTERVAL_SHIFT);
				wavesetting.scale = (value & SCALE_MASK);
				ret = copy_to_user(buff, &wavesetting, length);
				if (ret) {
					/* Failed to copy all the data, exit */
					dev_err(drv2624->dev,
						"copy to user fail %d\n", ret);
				}
			}

			break;
		}

	case HAPTIC_CMDID_SET_SEQUENCER:
		{
			if (length == sizeof(struct drv2624_waveform_sequencer)) {
				struct drv2624_waveform_sequencer sequencer;
				unsigned char effects[DRV2624_SEQUENCER_SIZE] = {
					0 };
				unsigned char loop[2] = { 0 };
				int i;

				ret = drv2624_bulk_read(drv2624,
							DRV2624_REG_SEQUENCER_1,
							effects,
							DRV2624_SEQUENCER_SIZE);
				if (ret < 0) {
					dev_err(drv2624->dev,
						"bulk read error %d\n", ret);
					break;
				}

				ret = drv2624_bulk_read(drv2624,
							DRV2624_REG_SEQ_LOOP_1,
							loop, 2);

				for (i = 0; i < DRV2624_SEQUENCER_SIZE; i++) {
					sequencer.waveform[i].effect =
					    effects[i];
					if (i < 4)
						sequencer.waveform[i].loop =
						    ((loop[0] >> (2 * i)) &
						     0x03);
					else
						sequencer.waveform[i].loop =
						    ((loop[1] >> (2 * (i - 4)))
						     & 0x03);
				}

				ret = copy_to_user(buff, &sequencer, length);
				if (ret) {
					/* Failed to copy all the data, exit */
					dev_err(drv2624->dev,
						"copy to user fail %d\n", ret);
				}
			}

			break;
		}

	case HAPTIC_CMDID_READ_FIRMWARE:
		{
			if (length > 0) {
				int i = 0;

				buffer = kzalloc(length, GFP_KERNEL);
				if (buffer) {
					drv2624_reg_write(drv2624,
							  DRV2624_REG_RAM_ADDR_UPPER,
							  drv2624->ram_msb);
					drv2624_reg_write(drv2624,
							  DRV2624_REG_RAM_ADDR_LOWER,
							  drv2624->ram_lsb);

					for (i = 0; i < length; i++)
						buffer[i] =
						    drv2624_reg_read(drv2624,
								     DRV2624_REG_RAM_DATA);

					if (ret < 0) {
						dev_err(drv2624->dev,
							"dev read fail %d\n",
							ret);
					} else {
						ret =
						    copy_to_user(buff, buffer,
								 length);
						if (ret) {
							/* Failed to copy all the data, exit */
							dev_err(drv2624->dev,
								"copy to user fail %d\n",
								ret);
						}
					}

					kfree(buffer);
				} else {
					dev_err(drv2624->dev, "read no mem\n");
					ret = -ENOMEM;
				}
			}
			break;
		}

	default:
		drv2624->file_cmd = 0;
		break;
	}

	mutex_unlock(&drv2624->lock);

	return length;
}

static ssize_t drv2624_file_write(struct file *filp, const char *buff,
				  size_t len, loff_t *off)
{
	struct drv2624_data *drv2624 =
	    (struct drv2624_data *)filp->private_data;
	unsigned char *buffer;
	int ret;

	mutex_lock(&drv2624->lock);

	buffer = kzalloc(len, GFP_KERNEL);
	if (!buffer)
		goto err;

	ret = copy_from_user(buffer, buff, len);
	if (ret) {
		dev_err(drv2624->dev, "copy_from_user failed.\n");
		goto err;
	}

	drv2624->file_cmd = buffer[0];

	switch (drv2624->file_cmd) {
	case HAPTIC_CMDID_REG_READ:
		{
			if (len == 2) {
				drv2624->current_reg = buffer[1];
			} else {
				dev_err(drv2624->dev,
					" read cmd len %zu err\n", len);
			}
			break;
		}

	case HAPTIC_CMDID_REG_WRITE:
		{
			if ((len - 1) == 2) {
				drv2624_reg_write(drv2624, buffer[1],
						  buffer[2]);
			} else if ((len - 1) > 2) {
				drv2624_bulk_write(drv2624, buffer[1],
						   &buffer[2], len - 2);
			} else {
				dev_err(drv2624->dev,
					"%s, reg_write len %zu error\n",
					__func__, len);
			}
			break;
		}

	case HAPTIC_CMDID_REG_SETBIT:
		{
			if (len == 4) {
				drv2624_set_bits(drv2624, buffer[1], buffer[2],
						 buffer[3]);
			} else {
				dev_err(drv2624->dev,
					"setbit len %zu error\n", len);
			}
			break;
		}

	case HAPTIC_CMDID_RUN_DIAG:
		{
			drv2624_stop(drv2624);
			drv2624_enable_irq(drv2624, false);
			ret = dev_run_diagnostics(drv2624);
			if (ret < 0)
				drv2624_disable_irq(drv2624);
			break;
		}

	case HAPTIC_CMDID_UPDATE_FIRMWARE:
		{
			ret = request_firmware_nowait(THIS_MODULE,
						      FW_ACTION_HOTPLUG,
						      "drv2624.bin",
						      drv2624->dev,
						      GFP_KERNEL,
						      drv2624,
						      drv2624_firmware_load);
			break;
		}

	case HAPTIC_CMDID_READ_FIRMWARE:
		{
			if (len == 3) {
				drv2624->ram_msb = buff[1];
				drv2624->ram_lsb = buff[2];
			} else {
				dev_err(drv2624->dev,
					"%s, read fw len error\n",
					__func__);
			}
			break;
		}

	case HAPTIC_CMDID_RUN_CALIBRATION:
		{
			drv2624_stop(drv2624);
			drv2624_enable_irq(drv2624, false);
			ret = dev_auto_calibrate(drv2624);
			if (ret < 0)
				drv2624_disable_irq(drv2624);
			break;
		}

	case HAPTIC_CMDID_CONFIG_WAVEFORM:
		{
			if (len == (1 + sizeof(struct drv2624_wave_setting))) {
				struct drv2624_wave_setting wavesetting;

				memcpy(&wavesetting, &buffer[1],
				       sizeof(struct drv2624_wave_setting));
				ret =
				    drv2624_config_waveform(drv2624,
							    &wavesetting);
			} else {
				dev_dbg(drv2624->dev,
					"pass cmd, prepare for read\n");
			}
		}
		break;

	case HAPTIC_CMDID_SET_SEQUENCER:
		{
			if (len ==
			    (1 + sizeof(struct drv2624_waveform_sequencer))) {
				struct drv2624_waveform_sequencer sequencer;

				memcpy(&sequencer, &buffer[1],
				       sizeof(struct
					      drv2624_waveform_sequencer));
				ret =
				    drv2624_set_waveform(drv2624, &sequencer);
			} else {
				dev_dbg(drv2624->dev,
					"pass cmd, prepare for read\n");
			}
		}
		break;

	case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE:
		{
			drv2624_stop(drv2624);
			drv2624_enable_irq(drv2624, false);
			ret = drv2624_play_effect(drv2624);
			if (ret < 0)
				drv2624_disable_irq(drv2624);
			break;
		}

	default:
		dev_err(drv2624->dev, "%s, unknown cmd\n", __func__);
		break;
	}

err:
	kfree(buffer);

	mutex_unlock(&drv2624->lock);

	return len;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = drv2624_file_read,
	.write = drv2624_file_write,
	.unlocked_ioctl = drv2624_file_unlocked_ioctl,
	.open = drv2624_file_open,
	.release = drv2624_file_release,
};

static struct miscdevice drv2624_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = HAPTICS_DEVICE_NAME,
	.fops = &fops,
};

static int haptics_init(struct drv2624_data *drv2624)
{
	int ret;

	drv2624->to_dev.name = "vibrator";
	drv2624->to_dev.get_time = vibrator_get_time;
	drv2624->to_dev.enable = vibrator_enable;
	drv2624->vibrator_playing = false;

	ret = timed_output_dev_register(&drv2624->to_dev);
	if (ret < 0) {
		dev_err(drv2624->dev,
			"drv2624: fail to create timed output dev\n");
		return ret;
	}

	ret = misc_register(&drv2624_misc);
	if (ret) {
		dev_err(drv2624->dev, "drv2624 misc fail: %d\n", ret);
		return ret;
	}

	hrtimer_init(&drv2624->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv2624->timer.function = vibrator_timer_func;
	INIT_WORK(&drv2624->vibrator_work, vibrator_work_routine);

	wake_lock_init(&drv2624->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&drv2624->lock);

	return 0;
}

static void dev_init_platform_data(struct drv2624_data *drv2624)
{
	struct drv2624_platform_data *pdata = &drv2624->plat_data;
	struct actuator_data actuator = pdata->actuator;
	unsigned char value_temp = 0;
	unsigned char mask_temp = 0;

	drv2624_set_bits(drv2624,
			 DRV2624_REG_MODE, PINFUNC_MASK,
			 (PINFUNC_INT << PINFUNC_SHIFT));

	if ((actuator.actuator_type == ERM) ||
	    (actuator.actuator_type == LRA)) {
		mask_temp |= ACTUATOR_MASK;
		value_temp |= (actuator.actuator_type << ACTUATOR_SHIFT);
	}

	if ((pdata->loop == CLOSE_LOOP) ||
	    (pdata->loop == OPEN_LOOP)) {
		mask_temp |= LOOP_MASK;
		value_temp |= (pdata->loop << LOOP_SHIFT);
	}

	if (value_temp != 0) {
		drv2624_set_bits(drv2624,
				 DRV2624_REG_CONTROL1,
				 mask_temp | AUTOBRK_OK_MASK,
				 value_temp | AUTOBRK_OK_ENABLE);
	}

	value_temp = 0;
	if (actuator.actuator_type == ERM)
		value_temp = LIB_ERM;
	else if (actuator.actuator_type == LRA)
		value_temp = LIB_LRA;
	if (value_temp != 0) {
		drv2624_set_bits(drv2624,
				 DRV2624_REG_CONTROL2, LIB_MASK,
				 value_temp << LIB_SHIFT);
	}

	if (actuator.rated_voltage != 0) {
		drv2624_reg_write(drv2624,
				  DRV2624_REG_RATED_VOLTAGE,
				  actuator.rated_voltage);
	} else {
		dev_err(drv2624->dev, "%s, ERROR Rated ZERO\n", __func__);
	}

	if (actuator.over_drive_clamp_voltage != 0) {
		drv2624_reg_write(drv2624,
				  DRV2624_REG_OVERDRIVE_CLAMP,
				  actuator.over_drive_clamp_voltage);
	} else {
		dev_err(drv2624->dev,
			"%s, ERROR OverDriveVol ZERO\n", __func__);
	}

	if (actuator.actuator_type == LRA) {
		unsigned char drive_time =
		    5 * (1000 - actuator.lra_freq) / actuator.lra_freq;
		unsigned short open_loop_period =
		    (unsigned short)((unsigned int)1000000000 /
				     (24619 * actuator.lra_freq));

		if (actuator.lra_freq < 125)
			drive_time |= (MINFREQ_SEL_45HZ << MINFREQ_SEL_SHIFT);
		drv2624_set_bits(drv2624,
				 DRV2624_REG_DRIVE_TIME,
				 DRIVE_TIME_MASK | MINFREQ_SEL_MASK, drive_time);
		drv2624_set_bits(drv2624,
				 DRV2624_REG_OL_PERIOD_H, 0x03,
				 (open_loop_period & 0x0300) >> 8);
		drv2624_reg_write(drv2624, DRV2624_REG_OL_PERIOD_L,
				  (open_loop_period & 0x00ff));

		dev_info(drv2624->dev,
			 "%s, LRA = %d, drive_time=0x%x\n",
			 __func__, actuator.lra_freq, drive_time);
	}
}

static irqreturn_t drv2624_irq_handler(int irq, void *dev_id)
{
	struct drv2624_data *drv2624 = (struct drv2624_data *)dev_id;

	drv2624->work_mode |= WORK_IRQ;
	schedule_work(&drv2624->vibrator_work);
	return IRQ_HANDLED;
}

static int drv2624_parse_dt(struct device *dev, struct drv2624_data *drv2624)
{
	struct device_node *np = dev->of_node;
	struct drv2624_platform_data *pdata = &drv2624->plat_data;
	int rc = 0, ret = 0;
	unsigned int value;

	pdata->gpio_nrst = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (pdata->gpio_nrst < 0) {
		dev_err(drv2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, pdata->gpio_nrst);
		ret = -1;
	} else {
		dev_dbg(drv2624->dev, "ti,reset-gpio=%d\n",
			pdata->gpio_nrst);
	}

	if (ret >= 0) {
		pdata->gpio_int = of_get_named_gpio(np, "ti,irq-gpio", 0);
		if (pdata->gpio_int < 0) {
			dev_err(drv2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,irq-gpio", np->full_name,
				pdata->gpio_int);
			ret = -1;
		} else {
			dev_dbg(drv2624->dev, "ti,irq-gpio=%d\n",
				pdata->gpio_int);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,smart-loop", &value);
		if (rc) {
			dev_err(drv2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,smart-loop", np->full_name, rc);
			ret = -2;
		} else {
			pdata->loop = value & 0x01;
			dev_dbg(drv2624->dev,
				"ti,smart-loop=%d\n", pdata->loop);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,actuator", &value);
		if (rc) {
			dev_err(drv2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,actuator", np->full_name, rc);
			ret = -2;
		} else {
			pdata->actuator.actuator_type = value & 0x01;
			dev_dbg(drv2624->dev,
				"ti,actuator=%d\n",
				pdata->actuator.actuator_type);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,rated-voltage", &value);
		if (rc) {
			dev_err(drv2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,rated-voltage", np->full_name, rc);
			ret = -2;
		} else {
			pdata->actuator.rated_voltage = value;
			dev_dbg(drv2624->dev,
				"ti,rated-voltage=0x%x\n",
				pdata->actuator.rated_voltage);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,odclamp-voltage", &value);
		if (rc) {
			dev_err(drv2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,odclamp-voltage", np->full_name, rc);
			ret = -2;
		} else {
			pdata->actuator.over_drive_clamp_voltage = value;
			dev_dbg(drv2624->dev,
				"ti,odclamp-voltage=0x%x\n",
				pdata->actuator.over_drive_clamp_voltage);
		}
	}

	if ((ret >= 0) && (pdata->actuator.actuator_type == LRA)) {
		rc = of_property_read_u32(np, "ti,lra-frequency", &value);
		if (rc) {
			dev_err(drv2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,lra-frequency", np->full_name, rc);
			ret = -3;
		} else {
			if ((value >= 45) && (value <= 300)) {
				pdata->actuator.lra_freq = value;
				dev_dbg(drv2624->dev,
					"ti,lra-frequency=%d\n",
					pdata->actuator.lra_freq);
			} else {
				ret = -1;
				dev_err(drv2624->dev,
					"ERROR, ti,lra-frequency=%d, out of range\n",
					pdata->actuator.lra_freq);
			}
		}
	}

	return ret;
}

static struct regmap_config drv2624_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int drv2624_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct drv2624_data *drv2624;
	int err = 0;

	dev_info(&client->dev, "%s enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s:I2C check failed\n", __func__);
		return -ENODEV;
	}

	drv2624 =
	    devm_kzalloc(&client->dev, sizeof(struct drv2624_data), GFP_KERNEL);
	if (!drv2624) {
		dev_err(&client->dev, "%s:no memory\n", __func__);
		return -ENOMEM;
	}

	drv2624->dev = &client->dev;
	i2c_set_clientdata(client, drv2624);
	dev_set_drvdata(&client->dev, drv2624);

	drv2624->regmap = devm_regmap_init_i2c(client, &drv2624_i2c_regmap);
	if (IS_ERR(drv2624->regmap)) {
		err = PTR_ERR(drv2624->regmap);
		dev_err(drv2624->dev,
			"%s:Failed to allocate register map: %d\n",
			__func__, err);
		return err;
	}

	if (client->dev.of_node) {
		dev_dbg(drv2624->dev, "of node parse\n");
		err = drv2624_parse_dt(&client->dev, drv2624);
	} else if (client->dev.platform_data) {
		dev_dbg(drv2624->dev, "platform data parse\n");
		memcpy(&drv2624->plat_data,
		       client->dev.platform_data,
		       sizeof(struct drv2624_platform_data));
	} else {
		dev_err(drv2624->dev, "%s: ERROR no platform data\n", __func__);
		return -ENODEV;
	}

	if ((err < 0) || (drv2624->plat_data.gpio_nrst <= 0) ||
	    (drv2624->plat_data.gpio_int <= 0)) {
		dev_err(drv2624->dev, "%s: platform data error\n", __func__);
		return -ENODEV;
	}

	if (drv2624->plat_data.gpio_nrst) {
		err = gpio_request(drv2624->plat_data.gpio_nrst, "DRV2624-NRST");
		if (err < 0) {
			dev_err(drv2624->dev,
				"%s: GPIO %d request NRST error\n",
				__func__, drv2624->plat_data.gpio_nrst);
			return err;
		}

		gpio_direction_output(drv2624->plat_data.gpio_nrst, 0);
		usleep_range(1000, 2000);
		gpio_direction_output(drv2624->plat_data.gpio_nrst, 1);
		usleep_range(1000, 2000); /* t(on) = 1ms */
	}

	mutex_init(&drv2624->dev_lock);
	err = drv2624_reg_read(drv2624, DRV2624_REG_ID);
	if (err < 0) {
		dev_err(drv2624->dev, "%s, i2c bus fail (%d)\n", __func__, err);
		goto exit_gpio_request_failed1;
	} else {
		dev_info(drv2624->dev, "%s, ID status (0x%x)\n", __func__, err);
		drv2624->device_id = err;
	}

	if ((drv2624->device_id & 0xf0) != DRV2624_ID) {
		dev_err(drv2624->dev, "%s, device_id(0x%x) fail\n",
			__func__, drv2624->device_id);
		goto exit_gpio_request_failed1;
	}

	dev_init_platform_data(drv2624);

	if (drv2624->plat_data.gpio_int) {
		err = gpio_request(drv2624->plat_data.gpio_int, "DRV2624-IRQ");
		if (err < 0) {
			dev_err(drv2624->dev,
				"%s: GPIO %d request INT error\n",
				__func__, drv2624->plat_data.gpio_int);
			goto exit_gpio_request_failed1;
		}

		gpio_direction_input(drv2624->plat_data.gpio_int);

		drv2624->irq = gpio_to_irq(drv2624->plat_data.gpio_int);
		dev_dbg(drv2624->dev, "irq = %d\n", drv2624->irq);

		err = request_threaded_irq(drv2624->irq, drv2624_irq_handler,
					   NULL,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   client->name, drv2624);

		if (err < 0) {
			dev_err(drv2624->dev, "request_irq failed, %d\n", err);
			goto exit_gpio_request_failed2;
		}
		drv2624_disable_irq(drv2624);
	}

	drv2624_plat_data = drv2624;

	haptics_init(drv2624);

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "drv2624.bin",
				&client->dev, GFP_KERNEL, drv2624,
				drv2624_firmware_load);

	dev_info(drv2624->dev, "drv2624 probe succeeded\n");

	return 0;

exit_gpio_request_failed2:
	if (drv2624->plat_data.gpio_int > 0)
		gpio_free(drv2624->plat_data.gpio_int);

exit_gpio_request_failed1:
	if (drv2624->plat_data.gpio_nrst > 0)
		gpio_free(drv2624->plat_data.gpio_nrst);

	mutex_destroy(&drv2624->dev_lock);

	dev_err(drv2624->dev, "%s failed, err=%d\n", __func__, err);
	return err;
}

static int drv2624_i2c_remove(struct i2c_client *client)
{
	struct drv2624_data *drv2624 = i2c_get_clientdata(client);

	if (drv2624->plat_data.gpio_nrst)
		gpio_free(drv2624->plat_data.gpio_nrst);

	if (drv2624->plat_data.gpio_int)
		gpio_free(drv2624->plat_data.gpio_int);

	misc_deregister(&drv2624_misc);

	mutex_destroy(&drv2624->lock);
	mutex_destroy(&drv2624->dev_lock);

	return 0;
}

static const struct i2c_device_id drv2624_i2c_id[] = {
	{"drv2624", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, drv2624_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id drv2624_of_match[] = {
	{.compatible = "ti,drv2624"},
	{},
};

MODULE_DEVICE_TABLE(of, drv2624_of_match);
#endif

static struct i2c_driver drv2624_i2c_driver = {
	.driver = {
		   .name = "drv2624",
		   .owner = THIS_MODULE,
#if defined(CONFIG_OF)
		   .of_match_table = of_match_ptr(drv2624_of_match),
#endif
		   },
	.probe = drv2624_i2c_probe,
	.remove = drv2624_i2c_remove,
	.id_table = drv2624_i2c_id,
};

module_i2c_driver(drv2624_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("DRV2624 I2C Smart Haptics driver");
MODULE_LICENSE("GPL v2");
