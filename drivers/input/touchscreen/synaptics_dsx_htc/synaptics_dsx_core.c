/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/input/synaptics_dsx_htc.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
#include <linux/sensor_hub.h>
#endif
#ifdef CONFIG_SYNC_TOUCH_STATUS
#include <linux/CwMcuSensor.h>
#endif
#include "synaptics_dsx_core.h"
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif
#ifdef CONFIG_AK8789_HALLSENSOR
#include <linux/hall_sensor.h>
#endif

#define INPUT_PHYS_NAME "synaptics_dsx/touch_input"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
#define WAKEUP_GESTURE true
#else
#define WAKEUP_GESTURE false
#endif

#define NO_0D_WHILE_2D
#define REPORT_2D_Z
#define REPORT_2D_W

#define F12_DATA_15_WORKAROUND

#define IGNORE_FN_INIT_FAILURE

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 500 /* ms */
#define MAX_F11_TOUCH_WIDTH 15

#define CHECK_STATUS_TIMEOUT_MS 100

#define F01_STD_QUERY_LEN 21
#define F01_CHIP_ID_OFFSET 17
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define SENSOR_SLEEP_NO_CAL (1 << 1)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

#define F11_CONTINUOUS_MODE 0x00
#define F11_WAKEUP_GESTURE_MODE 0x04
#define F12_CONTINUOUS_MODE 0x00
#define F12_WAKEUP_GESTURE_MODE 0x02

#define V5V6_CONFIG_ID_SIZE 4
#define V7_CONFIG_ID_SIZE 32

#define SHIFT_BITS (10)
static uint32_t debug_mask = 0x00000000;
//extern char *htc_get_bootmode(void);
static DECLARE_WAIT_QUEUE_HEAD(syn_data_ready_wq);
static unsigned int support_htc_event_flag = 0;
static unsigned int tamper_flag = 0;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
static uint8_t offmode_charging_flag = 0;
#endif

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28);

static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data);
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_set_status(struct synaptics_rmi4_data *rmi4_data, int value);
static int synaptics_rmi4_set_edge_filter(struct synaptics_rmi4_data *rmi4_data, int value);
static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_hw_reset_device(struct synaptics_rmi4_data *rmi4_data);
static irqreturn_t synaptics_rmi4_irq(int irq, void *data);
#ifdef MTK_PLATFORM
static int irq_registration(unsigned int *irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev);
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
extern void incell_driver_ready(void (*fn));
void notify_from_disp_cont_splash(int enable);
//void synaptics_rmi4_touch_enable(int enable);
#endif

static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void synaptics_rmi4_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_FB
static void synaptics_rmi4_early_suspend(struct device *dev);

static void synaptics_rmi4_late_resume(struct device *dev);
#endif

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

#ifdef CONFIG_FB
static ssize_t synaptics_rmi4_early_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
static int facedown_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused);
#endif
#ifdef CONFIG_SYNC_TOUCH_STATUS
static void switch_sensor_hub(struct synaptics_rmi4_data *rmi4_data, int mode);
#endif

struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
			struct {
				unsigned char data8_is_present:1;
				unsigned char data9_is_present:1;
				unsigned char data10_is_present:1;
				unsigned char data11_is_present:1;
				unsigned char data12_is_present:1;
				unsigned char data13_is_present:1;
				unsigned char data14_is_present:1;
				unsigned char data15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
		};
		unsigned char data[14];
	};
};

struct synaptics_rmi4_f12_ctrl_23 {
	union {
		struct {
			unsigned char obj_type_enable;
			unsigned char max_reported_objects;
		};
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
#ifdef REPORT_2D_Z
	unsigned char z;
#endif
#ifdef REPORT_2D_W
	unsigned char wx;
	unsigned char wy;
#endif
};

struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char button_int_enable;
	unsigned char multi_button;
	unsigned char *txrx_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char max_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_f54_query {
	union {
		struct {
			/* query 0 */
			unsigned char num_of_rx_electrodes;

			/* query 1 */
			unsigned char num_of_tx_electrodes;

			/* query 2 */
			unsigned char f54_query2_b0__1:2;
			unsigned char has_baseline:1;
			unsigned char has_image8:1;
			unsigned char f54_query2_b4__5:2;
			unsigned char has_image16:1;
			unsigned char f54_query2_b7:1;

			/* queries 3.0 and 3.1 */
			unsigned short clock_rate;

			/* query 4 */
			unsigned char touch_controller_family;

			/* query 5 */
			unsigned char has_pixel_touch_threshold_adjustment:1;
			unsigned char f54_query5_b1__7:7;

			/* query 6 */
			unsigned char has_sensor_assignment:1;
			unsigned char has_interference_metric:1;
			unsigned char has_sense_frequency_control:1;
			unsigned char has_firmware_noise_mitigation:1;
			unsigned char has_ctrl11:1;
			unsigned char has_two_byte_report_rate:1;
			unsigned char has_one_byte_report_rate:1;
			unsigned char has_relaxation_control:1;

			/* query 7 */
			unsigned char curve_compensation_mode:2;
			unsigned char f54_query7_b2__7:6;

			/* query 8 */
			unsigned char f54_query8_b0:1;
			unsigned char has_iir_filter:1;
			unsigned char has_cmn_removal:1;
			unsigned char has_cmn_maximum:1;
			unsigned char has_touch_hysteresis:1;
			unsigned char has_edge_compensation:1;
			unsigned char has_per_frequency_noise_control:1;
			unsigned char has_enhanced_stretch:1;

			/* query 9 */
			unsigned char has_force_fast_relaxation:1;
			unsigned char has_multi_metric_state_machine:1;
			unsigned char has_signal_clarity:1;
			unsigned char has_variance_metric:1;
			unsigned char has_0d_relaxation_control:1;
			unsigned char has_0d_acquisition_control:1;
			unsigned char has_status:1;
			unsigned char has_slew_metric:1;

			/* query 10 */
			unsigned char has_h_blank:1;
			unsigned char has_v_blank:1;
			unsigned char has_long_h_blank:1;
			unsigned char has_startup_fast_relaxation:1;
			unsigned char has_esd_control:1;
			unsigned char has_noise_mitigation2:1;
			unsigned char has_noise_state:1;
			unsigned char has_energy_ratio_relaxation:1;

			/* query 11 */
			unsigned char has_excessive_noise_reporting:1;
			unsigned char has_slew_option:1;
			unsigned char has_two_overhead_bursts:1;
			unsigned char has_query13:1;
			unsigned char has_one_overhead_burst:1;
			unsigned char f54_query11_b5:1;
			unsigned char has_ctrl88:1;
			unsigned char has_query15:1;

			/* query 12 */
			unsigned char number_of_sensing_frequencies:4;
			unsigned char f54_query12_b4__7:4;
		} __packed;
		unsigned char data[14];
	};
};

struct synaptics_rmi4_f54_query_13 {
	union {
		struct {
			unsigned char has_ctrl86:1;
			unsigned char has_ctrl87:1;
			unsigned char has_ctrl87_sub0:1;
			unsigned char has_ctrl87_sub1:1;
			unsigned char has_ctrl87_sub2:1;
			unsigned char has_cidim:1;
			unsigned char has_noise_mitigation_enhancement:1;
			unsigned char has_rail_im:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_exp_fhandler {
	struct synaptics_rmi4_exp_fn *exp_fn;
	bool insert;
	bool remove;
	struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;

static struct device_attribute attrs[] = {
	__ATTR(full_pm_cycle, (S_IRUGO | S_IWUSR),
			synaptics_rmi4_full_pm_cycle_show,
			synaptics_rmi4_full_pm_cycle_store),
	__ATTR(reset, S_IWUSR,
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, S_IRUGO,
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(buildid, S_IRUGO,
			synaptics_rmi4_f01_buildid_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, S_IRUGO,
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(0dbutton, (S_IRUGO | S_IWUSR),
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
	__ATTR(suspend, S_IWUSR,
			synaptics_rmi4_show_error,
			synaptics_rmi4_suspend_store),
#ifdef CONFIG_FB
	__ATTR(early_suspend, S_IWUSR,
			synaptics_rmi4_show_error,
			synaptics_rmi4_early_suspend_store),
#endif
	__ATTR(wake_gesture, (S_IRUGO | S_IWUSR),
			synaptics_rmi4_wake_gesture_show,
			synaptics_rmi4_wake_gesture_store),
};

static char state2char(int status)
{
	switch (status) {
		case F12_FINGER_STATUS:
			return 'F';
		case F12_GLOVED_FINGER_STATUS:
			return 'G';
		default:
			return 'F';
	}
}

#ifdef CONFIG_AK8789_HALLSENSOR
static DEFINE_MUTEX(synaptics_block_mutex);
static void synaptics_block_touch(struct synaptics_rmi4_data *rmi4_data, int enable)
{
	mutex_lock(&synaptics_block_mutex);
	rmi4_data->hall_block_touch_event = enable;
	mutex_unlock(&synaptics_block_mutex);
	pr_info("%s: %d\n", __func__, rmi4_data->hall_block_touch_event);
}

static void synaptics_block_touch_work_func(struct work_struct *dummy)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	synaptics_block_touch(rmi4_data, 0);
}
static DECLARE_DELAYED_WORK(synaptics_block_touch_work, synaptics_block_touch_work_func);

static void synaptics_handle_block_touch(struct synaptics_rmi4_data *rmi4_data, int enable)
{
	int ret;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	if (rmi4_data->hall_block_touch_event) {
		ret = cancel_delayed_work(&synaptics_block_touch_work);
		synaptics_block_touch(rmi4_data, 0);
	}
	if (enable) {
		pr_info("%s: %d\n", __func__, bdata->hall_block_touch_time);
		ret = schedule_delayed_work(&synaptics_block_touch_work, HZ*bdata->hall_block_touch_time/1000);
		synaptics_block_touch(rmi4_data, 1);
	}
}
#endif

static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->full_pm_cycle);
}

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->full_pm_cycle = input > 0 ? 1 : 0;

	return count;
}

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->firmware_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (list_empty(&rmi->support_fn_list))
		return -ENODEV;

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
			ii = fhandler->intr_reg_num;

			retval = synaptics_rmi4_reg_read(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;

			if (input == 1)
				intr_enable |= fhandler->intr_mask;
			else
				intr_enable &= ~fhandler->intr_mask;

			retval = synaptics_rmi4_reg_write(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		synaptics_rmi4_suspend(dev);
	else if (input == 0)
		synaptics_rmi4_resume(dev);
	else
		return -EINVAL;

	return count;
}

#ifdef CONFIG_FB
static ssize_t synaptics_rmi4_early_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		synaptics_rmi4_early_suspend(dev);
	else if (input == 0)
		synaptics_rmi4_late_resume(dev);
	else
		return -EINVAL;

	return count;
}
#endif

static ssize_t synaptics_rmi4_wake_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	buf[0] = '0' + rmi4_data->enable_wakeup_gesture;
	buf[1] = '\n';
	buf[2] = 0;
	return 2;
}

static ssize_t synaptics_rmi4_wake_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	switch(buf[0]) {
	case '0':
		input = 0;
		break;
	case '1':
		input = 1;
		break;
	default:
		return -EINVAL;
	}

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture)
		rmi4_data->enable_wakeup_gesture = input;

	return count;
}

static ssize_t synaptics_debug_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%08X\n", debug_mask);
}

static ssize_t synaptics_debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (sscanf(buf, "%ux", &debug_mask) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO), synaptics_debug_show, synaptics_debug_store);

static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	ret = snprintf(buf, PAGE_SIZE, "synaptics-%d", rmi4_data->chip_id);
	if (bdata->tw_pin_mask != 0)
		ret += scnprintf(buf+ret, PAGE_SIZE-ret, "_twID-0x%x", rmi4_data->tw_vendor);
	if (strlen(rmi4_data->lcm_vendor))
		ret += scnprintf(buf+ret, PAGE_SIZE-ret, "_LCM-%s", rmi4_data->lcm_vendor);
	ret += scnprintf(buf+ret, PAGE_SIZE-ret, "_PR: %d\n", rmi4_data->firmware_id);

	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, touch_vendor_show, NULL);

static ssize_t touch_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	ret = snprintf(buf, PAGE_SIZE, "config_version:%s\n", rmi4_data->config_version);

	return ret;
}

static DEVICE_ATTR(config, S_IRUGO, touch_config_show, NULL);

static ssize_t synaptics_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (!bdata->irq_gpio)
		return ret;

	ret = gpio_get_value(bdata->irq_gpio);
	printk(KERN_DEBUG "[TP] GPIO_TP_INT_N=%d\n", ret);
	snprintf(buf, PAGE_SIZE, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;

	return ret;
}
static DEVICE_ATTR(gpio, S_IRUGO, synaptics_gpio_show, NULL);

static ssize_t synaptics_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = 0;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	unsigned int reset;

	pr_info(" %s\n", __func__);
	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	ret = synaptics_rmi4_hw_reset_device(rmi4_data);
	if (ret < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command, error = %ld\n",
				__func__, ret);
		return ret;
	}

	return count;
}

static DEVICE_ATTR(reset, S_IWUSR, NULL, synaptics_reset_store);

static ssize_t synaptics_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	size_t count = 0;
	unsigned char cmd = 0x01;
	int retval = 0;
	uint16_t i, j;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f54_data_base_addr,
			&rmi4_data->diag_command,
			sizeof(rmi4_data->diag_command));
	if (retval < 0){
		dev_info(rmi4_data->pdev->dev.parent," %s write error:1", __func__);
		return retval;
	}

	atomic_set(&rmi4_data->data_ready, 0);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f54_cmd_base_addr,
			&cmd,
			sizeof(cmd));
	if (retval < 0) {
		atomic_set(&rmi4_data->data_ready, 1);
		dev_err(rmi4_data->pdev->dev.parent," %s write error:2", __func__);
		return retval;
	}

	wait_event_interruptible_timeout(syn_data_ready_wq,
			atomic_read(&rmi4_data->data_ready), 50);

	for (i = 0; i < rmi4_data->num_of_tx; i++) {
		for (j = 0; j < rmi4_data->num_of_rx; j++) {
			switch (rmi4_data->chip_id) {
				case 2200:
				case 3202:
				case 7508:
					count += snprintf(buf + count, PAGE_SIZE, "%5d", rmi4_data->report_data[i + j*rmi4_data->num_of_tx]);
					break;
				//case 3201:
				//case 3508:
				//case 3528:
				//case 3351:
				default:
					count += snprintf(buf + count, PAGE_SIZE, "%5d", rmi4_data->report_data[i*rmi4_data->num_of_rx + j]);
					break;
			}
		}
		count += snprintf(buf + count, PAGE_SIZE, "\n");
	}

	return count;
}

static ssize_t synaptics_diag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	if (buf[0] == '1')
		rmi4_data->diag_command = 2;
	else if (buf[0] == '2')
		rmi4_data->diag_command = 3;

	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO), synaptics_diag_show, synaptics_diag_store);

static ssize_t synaptics_rmi4_cover_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	size_t count = 0;

	if (bdata->support_cover) {
		count += snprintf(buf + count, PAGE_SIZE, "%d\n", rmi4_data->cover_mode);
	} else {
		count += snprintf(buf + count, PAGE_SIZE, "0\n");
	}

	return count;
}

static ssize_t synaptics_rmi4_cover_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int value;

	if (!bdata->support_cover)
		return count;

	if (sysfs_streq(buf, "0")) {
		value = 0;
	}
	else if (sysfs_streq(buf, "1")) {
		value = 1;
	}
	else
		return -EINVAL;

	rmi4_data->cover_mode = value;
	pr_info("%s: cover_mode change to %d\n", __func__, rmi4_data->cover_mode);

	synaptics_rmi4_set_edge_filter(rmi4_data, rmi4_data->cover_mode);
	synaptics_rmi4_set_status(rmi4_data, rmi4_data->cover_mode << 1 | rmi4_data->glove_enable);

	return count;
}
static DEVICE_ATTR(cover, (S_IWUSR|S_IRUGO),
	synaptics_rmi4_cover_show, synaptics_rmi4_cover_store);

static ssize_t synaptics_rmi4_glove_setting_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	size_t count = 0;

	if (bdata->support_glove) {
		count += snprintf(buf + count, PAGE_SIZE, "%d\n", rmi4_data->glove_setting);
	} else {
		count += snprintf(buf + count, PAGE_SIZE, "0\n");
	}

	return count;
}

static ssize_t synaptics_rmi4_glove_setting_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int value;

	if (!bdata->support_glove)
		return count;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("bad parameter\n");
		return -EINVAL;
	}

	if (value > 3 || value < 0) {
		pr_info("%s: wrong parameter\n", __func__);
		return -EINVAL;
	}

	rmi4_data->glove_setting = value;
	pr_info("%s: glove_setting change to %d\n", __func__, rmi4_data->glove_setting);
	value = value & 0x01;

	if (rmi4_data->cover_mode) {
		pr_info("%s: Under cover mode, will not change glove_mode\n", __func__);
		return count;
	}

	if (!rmi4_data->i2c_to_mcu) {
		if (synaptics_rmi4_set_status(rmi4_data, value)) {
			pr_err("set glove mode error\n");
			return -EINVAL;
		}
	} else
		pr_info("%s: I2C already switch to MCU side\n", __func__);

	rmi4_data->glove_enable = value;
	pr_info("%s: glove_mode change to %d\n", __func__, rmi4_data->glove_enable);

	return count;
}
static DEVICE_ATTR(glove_setting, (S_IWUSR|S_IRUGO),
	synaptics_rmi4_glove_setting_show, synaptics_rmi4_glove_setting_store);

static ssize_t int_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	size_t count = 0;

	count += snprintf(buf + count, PAGE_SIZE, "%d ", rmi4_data->irq_enabled);
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t int_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int value, ret=0;

	if (sysfs_streq(buf, "0"))
		value = false;
	else if (sysfs_streq(buf, "1"))
		value = true;
	else
		return -EINVAL;

	if (value) {
#ifdef MTK_PLATFORM
		ret = irq_registration(&rmi4_data->irq,
				synaptics_rmi4_irq, bdata->irq_flags,
				PLATFORM_DRIVER_NAME, rmi4_data);
#else
		ret = request_threaded_irq(rmi4_data->irq, NULL,
				synaptics_rmi4_irq, bdata->irq_flags,
				PLATFORM_DRIVER_NAME, rmi4_data);
#endif
		if (ret == 0) {
			rmi4_data->irq_enabled = true;
			pr_info("%s: interrupt enable: %x\n", __func__, rmi4_data->irq_enabled);
		}
	} else {
		disable_irq(rmi4_data->irq);
		free_irq(rmi4_data->irq, rmi4_data);
		rmi4_data->irq_enabled = false;
		pr_info("%s: interrupt disable: %x\n", __func__, rmi4_data->irq_enabled);
	}

	return count;
}
static DEVICE_ATTR(enabled, (S_IWUSR|S_IRUGO), int_status_show, int_status_store);

enum SR_REG_STATE{
	ALLOCATE_DEV_FAIL = -2,
	REGISTER_DEV_FAIL,
	SUCCESS,
};

static char *vk_name = "virtualkeys.sr_touchscreen";
static struct kobj_attribute vk_dev;

static int register_sr_touch_device(void)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int ret = 0;

	rmi4_data->sr_input_dev = input_allocate_device();

	if (rmi4_data->sr_input_dev == NULL) {
		printk(KERN_ERR "[TP][TOUCH_ERR]%s: Failed to allocate SR input device\n", __func__);
		return ALLOCATE_DEV_FAIL;
	}

	if (bdata->vk_obj) {
		memcpy(&vk_dev, bdata->vk2Use, sizeof(struct kobj_attribute));
		vk_dev.attr.name = vk_name;
		ret = sysfs_create_file(bdata->vk_obj, &(vk_dev.attr));
	}

	rmi4_data->sr_input_dev->name = "sr_touchscreen";
	set_bit(EV_SYN, rmi4_data->sr_input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->sr_input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->sr_input_dev->evbit);

	set_bit(KEY_BACK, rmi4_data->sr_input_dev->keybit);
	set_bit(KEY_HOME, rmi4_data->sr_input_dev->keybit);
	set_bit(KEY_MENU, rmi4_data->sr_input_dev->keybit);
	set_bit(KEY_SEARCH, rmi4_data->sr_input_dev->keybit);
	set_bit(BTN_TOUCH, rmi4_data->sr_input_dev->keybit);
	set_bit(KEY_APPSELECT, rmi4_data->sr_input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, rmi4_data->sr_input_dev->propbit);
	input_set_abs_params(rmi4_data->sr_input_dev, ABS_MT_TRACKING_ID,
		0, rmi4_data->num_of_fingers, 0, 0);
	pr_info("[SR]input_set_abs_params: mix_x %d, max_x %d,"
		" min_y %d, max_y %d", 0,
		 rmi4_data->sensor_max_x, 0, rmi4_data->sensor_max_y);

	input_set_abs_params(rmi4_data->sr_input_dev, ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->sr_input_dev, ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
	input_set_abs_params(rmi4_data->sr_input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255, 0, 0);
	input_set_abs_params(rmi4_data->sr_input_dev, ABS_MT_PRESSURE,
		0, 30, 0, 0);
	input_set_abs_params(rmi4_data->sr_input_dev, ABS_MT_WIDTH_MAJOR,
		0, 30, 0, 0);

	if (input_register_device(rmi4_data->sr_input_dev)) {
		input_free_device(rmi4_data->sr_input_dev);
		pr_info("[SR][TOUCH_ERR]%s: Unable to register %s input device\n",
			__func__, rmi4_data->sr_input_dev->name);
		return REGISTER_DEV_FAIL;
	}
	return SUCCESS;
}

static ssize_t set_en_sr(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	if (buf[0]) {
		if (rmi4_data->sr_input_dev)
			printk(KERN_INFO "[TP]%s: SR device already exist!\n", __func__);
		else
			printk(KERN_INFO "[TP]%s: SR touch device enable result:%X\n", __func__, register_sr_touch_device());
	}
	return count;
}

static ssize_t get_en_sr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	if (rmi4_data->sr_input_dev)
		return snprintf(buf, PAGE_SIZE, "%s \n", rmi4_data->sr_input_dev->name);
	else
		return snprintf(buf, PAGE_SIZE, "0\n");
}
static DEVICE_ATTR(sr_en, (S_IWUSR|S_IRUGO), get_en_sr, set_en_sr);

static struct kobject *android_touch_kobj;

#ifdef CONFIG_AK8789_HALLSENSOR
static int hallsensor_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	pole_value = status & 0x01;
	pole = (status & 0x02) >> HALL_POLE_BIT;
	pr_info("[HL] %s[%s]\n", pole? "att_s" : "att_n", pole_value ? "Near" : "Far");

	if ((pole == HALL_POLE_S) && bdata->support_cover) {
		if (pole_value == HALL_FAR) {
			rmi4_data->cover_mode = 0;
			if (bdata->hall_block_touch_time > 1)
				synaptics_handle_block_touch(rmi4_data, 0);
		}
		else {
			rmi4_data->cover_mode = 1;
			if (bdata->hall_block_touch_time > 1)
				synaptics_handle_block_touch(rmi4_data, 1);
		}

		if (!rmi4_data->i2c_to_mcu) {
			synaptics_rmi4_set_edge_filter(rmi4_data, rmi4_data->cover_mode);
			if (rmi4_data->cover_mode) {
				synaptics_rmi4_set_status(rmi4_data, rmi4_data->cover_mode << 1);
			}
			else if (rmi4_data->glove_setting & 0x01) {
				synaptics_rmi4_set_status(rmi4_data, 1);
				rmi4_data->glove_enable = 1;
				pr_info("%s: glove_enable = %d\n", __func__, rmi4_data->glove_enable);
			}
			else {
				synaptics_rmi4_set_status(rmi4_data, 0);
				rmi4_data->glove_enable = 0;
				pr_info("%s: glove_enable = %d\n", __func__, rmi4_data->glove_enable);
			}
		}

		pr_info("[HL] %s: cover_enable = %d\n", __func__, rmi4_data->cover_mode);
	}

	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_status_handler_func,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
static int synaptics_rmi4_set_status(struct synaptics_rmi4_data *rmi4_data, int value)
{
	int retval;
	unsigned char ctrl_9_offset;
	unsigned char ctrl_10_offset;
	unsigned char ctrl_11_offset;
	unsigned char ctrl_15_offset;
	unsigned char ctrl_23_offset;
	unsigned char report_control[18];
	unsigned char *control_data;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	if (rmi4_data->cover_setting_size == 0) {
		pr_info("%s: No cover_setting\n", __func__);
		return 0;
	}

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
			break;
	}

	if (fhandler->fn_number != SYNAPTICS_RMI4_F12) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: No F12 function\n",
				__func__);
		return -1;
	}

	dev_info(rmi4_data->pdev->dev.parent, " %s: %d\n", __func__, value);
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	ctrl_9_offset = extra_data->ctrl9_offset;
	ctrl_10_offset = extra_data->ctrl10_offset;
	ctrl_11_offset = extra_data->ctrl11_offset;
	ctrl_15_offset = extra_data->ctrl15_offset;
	ctrl_23_offset = extra_data->ctrl23_offset;

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			report_control,
			sizeof(uint8_t) * 1);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change report mode\n",
				__func__);
		return -1;
	}

	if (value)
		report_control[0] |= BIT(5);
	else
		report_control[0] &= ~BIT(5);

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			report_control,
			sizeof(uint8_t) * 1);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change report mode\n",
				__func__);
		return -1;
	}

	if (value & 0x2) {
		pr_info("%s: cover\n", __func__);
		control_data = rmi4_data->cover_setting;
	}
	else if (value & 0x01) {
		pr_info("%s: glove\n", __func__);
		control_data = rmi4_data->glove_mode_setting;
	}
	else {
		pr_info("%s: normal\n", __func__);
		control_data = rmi4_data->uncover_setting;
	}

	// F12_CTRL_09(03)00 [16th byte] F12_CTRL_09(04)01 [18th byte]
	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_9_offset,
			report_control,
			sizeof(uint8_t) * 18);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F12_CTRL_09\n",
				__func__);
		return -1;
	}

	report_control[15] = control_data[0];
	report_control[17] = control_data[1];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_9_offset,
			report_control,
			sizeof(uint8_t) * 18);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F12_CTRL_09\n",
				__func__);
		return -1;
	}

	// F12_CTRL_10(00)00 [1st byte] F12_CTRL_10(00)01 [2nd byte] F12_CTRL_10(00)02 [3rd byte]
/*	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_10_offset,
			report_control,
			sizeof(uint8_t) * 3);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F12_CTRL_10\n",
				__func__);
		return -1;
	}
*/
	report_control[0] = control_data[2];
	report_control[1] = control_data[3];
	report_control[2] = control_data[4];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_10_offset,
			report_control,
			sizeof(uint8_t) * 3);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F12_CTRL_10\n",
				__func__);
		return -1;
	}

	//F12_CTRL_11(01)05 [8th byte]
	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_11_offset,
			report_control,
			sizeof(uint8_t) * 8);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F12_CTRL_11\n",
				__func__);
		return -1;
	}

	report_control[7] = control_data[5];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_11_offset,
			report_control,
			sizeof(uint8_t) * 8);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F12_CTRL_11\n",
				__func__);
		return -1;
	}

	// F12_CTRL_15(00)00 [1st byte] F12_CTRL_15(00)01 [2nd byte] F12_CTRL_15(00)02 [3rd byte]
/*	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_15_offset,
			report_control,
			sizeof(uint8_t) * 3);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F12_CTRL_15\n",
				__func__);
		return -1;
	}
*/
	report_control[0] = control_data[6];
	report_control[1] = control_data[7];
	report_control[2] = control_data[8];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_15_offset,
			report_control,
			sizeof(uint8_t) * 3);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F12_CTRL_15\n",
				__func__);
		return -1;
	}

	// F54_CTRL_02.00
/*	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f54_ctrl_base_addr + 1,
			report_control,
			sizeof(uint8_t) * 1);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F54_CTRL_02\n",
				__func__);
		return -1;
	}
*/
	report_control[0] = control_data[9];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f54_ctrl_base_addr + 1,
			report_control,
			sizeof(uint8_t) * 1);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F54_CTRL_02\n",
				__func__);
		return -1;
	}

	dev_info(rmi4_data->pdev->dev.parent, " %s: done\n", __func__);
	return 0;
}
#else
static int synaptics_rmi4_set_status(struct synaptics_rmi4_data *rmi4_data, int value)
{
	int retval;
	unsigned char ctrl_10_offset;
	unsigned char ctrl_15_offset;
	unsigned char ctrl_23_offset;
	unsigned char report_control[18];
	unsigned char *control_data;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	if (rmi4_data->cover_setting_size == 0) {
		pr_info("%s: No cover_setting\n", __func__);
		return 0;
	}

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
			break;
	}

	if (fhandler->fn_number != SYNAPTICS_RMI4_F12) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: No F12 function\n",
				__func__);
		return -1;
	}

	dev_info(rmi4_data->pdev->dev.parent, " %s: %d\n", __func__, value);
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	ctrl_10_offset = extra_data->ctrl10_offset;
	ctrl_15_offset = extra_data->ctrl15_offset;
	ctrl_23_offset = extra_data->ctrl23_offset;

	if (value & 0x2) {
		pr_info("%s: cover\n", __func__);
		control_data = rmi4_data->cover_setting;
	}
	else if (value & 0x01) {
		pr_info("%s: glove\n", __func__);
		control_data = rmi4_data->glove_mode_setting;
	}
	else {
		pr_info("%s: normal\n", __func__);
		control_data = rmi4_data->uncover_setting;
	}

	//F12_CTRL_10(00)01 [2nd byte]
	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_10_offset,
			report_control,
			sizeof(uint8_t) * 2);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F12_CTRL_10\n",
				__func__);
		return -1;
	}

	report_control[0] = control_data[4];
	report_control[1] = control_data[0];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_10_offset,
			report_control,
			sizeof(uint8_t) * 2);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F12_CTRL_10\n",
				__func__);
		return -1;
	}

	// F12_CTRL_15(00)00 [1st byte] F12_CTRL_15(00)01 [2nd byte]
	report_control[0] = control_data[1];
	report_control[1] = control_data[2];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_15_offset,
			report_control,
			sizeof(uint8_t) * 2);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F12_CTRL_15\n",
				__func__);
		return -1;
	}

	//F12_CTRL_23(00)00 [1st byte]
	report_control[0] = control_data[3];

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to MCU\n", __func__);
		return 0;
	}
	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			report_control,
			sizeof(uint8_t) * 1);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F12_CTRL_23\n",
				__func__);
		return -1;
	}

	dev_info(rmi4_data->pdev->dev.parent, " %s: done\n", __func__);
	return 0;
}
#endif

static int synaptics_rmi4_set_edge_filter(struct synaptics_rmi4_data *rmi4_data, int value)
{
	int retval;
	unsigned char command;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F51)
			break;
	}

	if (fhandler->fn_number != SYNAPTICS_RMI4_F51) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: No F51 function\n",
				__func__);
		return -1;
	}

	dev_info(rmi4_data->pdev->dev.parent, " %s: %d\n", __func__, value);

	if (value)
		command = 0x03;
	else
		command = 0x00;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f51_ctrl_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f51_ctrl_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;
	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
static int synaptics_rmi4_querry_f51_data(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char report;
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f51_data_base_addr,
			&report,
			sizeof(report));
	if (retval < 0)
		return retval;

	pr_info("%s: 0x%02x\n", __func__, report);
	return report;
}
#endif

static int synaptics_rmi4_sysfs_init(struct synaptics_rmi4_data *rmi4_data, bool enable)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (enable) {
		android_touch_kobj = kobject_create_and_add("android_touch", NULL);
		if (android_touch_kobj == NULL) {
			dev_err(rmi4_data->pdev->dev.parent,"%s: subsystem_register failed\n", __func__);
			return -ENOMEM;
		}

		if (sysfs_create_link(android_touch_kobj, &rmi4_data->input_dev->dev.kobj, "synaptics_rmi4_dsx") < 0) {
			dev_err(rmi4_data->pdev->dev.parent, "%s: failed to create link\n", __func__);
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [debug_level]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [vendor]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_config.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [config]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [gpio]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [reset]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [diag]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_enabled.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [enabled]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_sr_en.attr)) {
			dev_err(rmi4_data->pdev->dev.parent,"failed to create sysfs file [sr_en]");
			return -ENOMEM;
		}

		if (sysfs_create_file(android_touch_kobj, &dev_attr_cover.attr) < 0) {
			pr_err("failed to create sysfs file [cover]");
			return -ENOMEM;
		}

		if (bdata->support_glove) {
			if (sysfs_create_file(android_touch_kobj, &dev_attr_glove_setting.attr) < 0) {
				pr_err("failed to create sysfs file [glove_setting]");
				return -ENOMEM;
			}
		}
	} else {
		sysfs_remove_link(android_touch_kobj, "synaptics_rmi4_dsx");
		if (bdata->support_glove)
			sysfs_remove_file(android_touch_kobj, &dev_attr_glove_setting.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_cover.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_sr_en.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_enabled.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_config.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
		sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
		kobject_del(android_touch_kobj);
	}
	return 0;
}

static void synaptics_rmi4_set_chip_mode(struct synaptics_rmi4_data *rmi4_data)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	synaptics_rmi4_querry_f51_data(rmi4_data);
#endif
	if (!bdata->support_glove && !bdata->support_cover) {
		return;
	}

	if (rmi4_data->i2c_to_mcu) {
		pr_info("%s: switch to i2c\n", __func__);
		return;
	}

	synaptics_rmi4_set_edge_filter(rmi4_data, rmi4_data->cover_mode);
	if (rmi4_data->cover_mode) {
		synaptics_rmi4_set_status(rmi4_data, rmi4_data->cover_mode << 1);
	}
	else if (rmi4_data->glove_setting & 0x01) {
		synaptics_rmi4_set_status(rmi4_data, 1);
		rmi4_data->glove_enable = 1;
	}
	else {
		synaptics_rmi4_set_status(rmi4_data, 0);
		rmi4_data->glove_enable = 0;
	}
}

static int synaptics_rmi4_get_noise_state(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	uint8_t data[2], ns = 0;
	uint16_t im = 0, cidim = 0, freq = 0;
	struct synaptics_rmi4_noise_state noise_state;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f54_data_base_addr + rmi4_data->f54_im_offset,
			data, sizeof(data));
	if (retval < 0)
		return retval;
	im = (data[1] << 8) | data[0];

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f54_data_base_addr + rmi4_data->f54_ns_offset,
			&ns, sizeof(ns));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f54_data_base_addr + rmi4_data->f54_cidim_offset,
			data, sizeof(data));
	if (retval < 0)
		return retval;
	cidim = (data[1] << 8) | data[0];

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f54_data_base_addr + rmi4_data->f54_freq_offset,
			data, sizeof(data));
	if (retval < 0)
		return retval;
	freq = (data[1] << 8) | data[0];

	noise_state.im = im;
	noise_state.cidim = cidim;
	noise_state.freq = freq;
	noise_state.ns = ns;
	if((noise_state.freq != rmi4_data->noise_state.freq) || (noise_state.ns != rmi4_data->noise_state.ns))
		pr_info("[NS]: IM:%d, CIDIM:%d, Freq:%d, NS:%d\n", im, cidim, freq, ns);

	memcpy(&rmi4_data->noise_state, &noise_state, sizeof(noise_state));

	return 0;
}

static unsigned short synaptics_sqrt(unsigned int num)
{
	unsigned short root, remainder, place;

	root = 0;
	remainder = num;
	place = 0x4000;

	while (place > remainder)
		place = place >> 2;
	while (place)
	{
		if (remainder >= root + place)
		{
			remainder = remainder - root - place;
			root = root + (place << 1);
		}
		root = root >> 1;
		place = place >> 2;
	}

	return root;
}

static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char data_reg_blk_size;
	unsigned char finger_status_reg[3];
	unsigned char data[F11_STD_DATA_LEN];
	unsigned char detected_gestures;
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
	int z;
	int wx;
	int wy;
	int temp;
	struct synaptics_rmi4_f11_extra_data *extra_data;

	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;
	data_reg_blk_size = fhandler->size_of_data_register_block;
	extra_data = (struct synaptics_rmi4_f11_extra_data *)fhandler->extra;

	if (rmi4_data->suspend && rmi4_data->enable_wakeup_gesture) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data38_offset,
				&detected_gestures,
				sizeof(detected_gestures));
		if (retval < 0)
			return 0;

		if (detected_gestures) {
			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 0);
			input_sync(rmi4_data->input_dev);
			rmi4_data->suspend = false;
		}

		return 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr,
			finger_status_reg,
			num_of_finger_status_regs);
	if (retval < 0)
		return 0;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif

		if (finger_status) {
			data_offset = data_addr +
					num_of_finger_status_regs +
					(finger * data_reg_blk_size);
			retval = synaptics_rmi4_reg_read(rmi4_data,
					data_offset,
					data,
					data_reg_blk_size);
			if (retval < 0) {
				touch_count = 0;
				goto exit;
			}

			x = (data[0] << 4) | (data[2] & MASK_4BIT);
			y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
			wx = (data[3] & MASK_4BIT);
			wy = (data[3] >> 4) & MASK_4BIT;
#ifdef REPORT_2D_Z
			z = data[4];
#endif

			if (rmi4_data->hw_if->board_data->swap_axes) {
				temp = x;
				x = y;
				y = temp;
				temp = wx;
				wx = wy;
				wy = temp;
			}

			if (rmi4_data->hw_if->board_data->x_flip)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->hw_if->board_data->y_flip)
				y = rmi4_data->sensor_max_y - y;

			if (rmi4_data->support_htc_event) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
				input_report_abs(rmi4_data->input_dev, ABS_MT_AMPLITUDE,
					(z << 16) | (wx + wy));
				input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION,
					((finger_status == 0) << 31) | (x << 16) | y);
#endif
			}

			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_Z
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_PRESSURE, z);
#endif
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, synaptics_sqrt(wx*wx + wy*wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif

			if (debug_mask & BIT(1)) {
				dev_info(rmi4_data->pdev->dev.parent,
					"%s: Finger %d: "
					"status = 0x%02x "
					"x = %d "
					"y = %d "
					"z = %d "
					"wx = %d "
					"wy = %d\n",
					__func__, finger,
					finger_status,
					x, y, z, wx, wy);
			}
			if (debug_mask & BIT(1)) {
				dev_info(rmi4_data->pdev->dev.parent,
					"Finger %d=> X:%d Y:%d Z:%d\n",
					finger+1, x, y, z);
			}


			touch_count++;
		}
	}

	if (touch_count == 0) {
		if (rmi4_data->support_htc_event) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
			input_report_abs(rmi4_data->input_dev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION,	(1 << 31));
#endif
		}

#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}

	input_sync(rmi4_data->input_dev);

exit:
	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return touch_count;
}

static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char finger;
	unsigned char fingers_to_process;
	unsigned char finger_status;
	unsigned char size_of_2d_data;
	unsigned char detected_gestures;
	unsigned short data_addr = 0;
	unsigned short glove_status = 0;
	int x = 0;
	int y = 0;
	int z = 0;
	int wx = 0;
	int wy = 0;
	int temp = 0;
	int state = 0;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_finger_data *data;
	struct synaptics_rmi4_f12_finger_data *finger_data;
#ifdef F12_DATA_15_WORKAROUND
	static unsigned char fingers_already_present;
#endif

	fingers_to_process = fhandler->num_of_data_points;
	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	if (rmi4_data->suspend && rmi4_data->enable_wakeup_gesture) {
		dev_dbg(rmi4_data->pdev->dev.parent, " %s, enable_wakeup_gesture\n", __func__);
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data4_offset,
				&detected_gestures,
				sizeof(detected_gestures));
		if (retval < 0)
			return 0;

		if (detected_gestures) {
			dev_dbg(rmi4_data->pdev->dev.parent, " %s, detected_gestures\n", __func__);
			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_WAKEUP, 0);
			input_sync(rmi4_data->input_dev);
			rmi4_data->suspend = false;
		}

		return 0;
	}

	/* Determine the total number of fingers to process */
	if (extra_data->data15_size) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				data_addr + extra_data->data15_offset,
				extra_data->data15_data,
				extra_data->data15_size);
		if (retval < 0)
			return 0;

		/* Start checking from the highest bit */
		temp = extra_data->data15_size - 1; /* Highest byte */
		if (temp >= (F12_FINGERS_TO_SUPPORT + 7) / 8)
			temp = (F12_FINGERS_TO_SUPPORT + 7) / 8 - 1;
		finger = (fingers_to_process - 1) % 8; /* Highest bit */
		do {
			if (extra_data->data15_data[temp] & (1 << finger))
				break;

			if (finger) {
				finger--;
			} else {
				temp--; /* Move to the next lower byte */
				finger = 7;
			}

			fingers_to_process--;
		} while (fingers_to_process);

		dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Number of fingers to process = %d\n",
			__func__, fingers_to_process);
	}

#ifdef F12_DATA_15_WORKAROUND
	fingers_to_process = max(fingers_to_process, fingers_already_present);
#endif

	if (!fingers_to_process) {
		synaptics_rmi4_free_fingers(rmi4_data);
		return 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr + extra_data->data1_offset,
			(unsigned char *)fhandler->data,
			fingers_to_process * size_of_2d_data);
	if (retval < 0)
		return 0;

	data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;

	synaptics_rmi4_get_noise_state(rmi4_data);

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_data = data + finger;
		finger_status = finger_data->object_type_and_status;

		switch (finger_status) {
		case F12_GLOVED_FINGER_STATUS:
			if (rmi4_data->glove_enable)
				glove_status |= 1 << finger;
		case F12_FINGER_STATUS:
#ifdef TYPE_B_PROTOCOL
		if (rmi4_data->hall_block_touch_event == 0) {
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 1);
			if (glove_status & (1 << finger))
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_GLOVE, 1);
			else
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_GLOVE, 0);
		}
#endif

#ifdef F12_DATA_15_WORKAROUND
			fingers_already_present = finger + 1;
#endif

			x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
			y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
#ifdef REPORT_2D_Z
			z = finger_data->z;
#endif
#ifdef REPORT_2D_W
			wx = finger_data->wx;
			wy = finger_data->wy;
#endif

			if (rmi4_data->hw_if->board_data->swap_axes) {
				temp = x;
				x = y;
				y = temp;
				temp = wx;
				wx = wy;
				wy = temp;
			}

			if (rmi4_data->hw_if->board_data->x_flip)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->hw_if->board_data->y_flip)
				y = rmi4_data->sensor_max_y - y;

		if (rmi4_data->hall_block_touch_event == 0) {
			if (rmi4_data->support_htc_event) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
				input_report_abs(rmi4_data->input_dev, ABS_MT_AMPLITUDE,
					(z << 16) | (wx + wy));
				input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION,
					((finger_status == 0) << 31) | (x << 16) | y);
#endif
			}

			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_Z
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_PRESSURE, z);
#endif
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, synaptics_sqrt(wx*wx + wy*wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
		}

			if (debug_mask & BIT(8)) {
				dev_info(rmi4_data->pdev->dev.parent,
					"%s: Finger %d: "
					"status = 0x%02x "
					"x = %d "
					"y = %d "
					"z = %d "
					"wx = %d "
					"wy = %d\n",
					__func__, finger,
					finger_status,
					x, y, z, wx, wy);
			}
			if (debug_mask & BIT(1)) {
				dev_info(rmi4_data->pdev->dev.parent,
					"Finger %d=> X:%d Y:%d Wx:%d Wy:%d Z:%d\n",
					finger+1, x, y, wx, wy, z);
			}

			state = 1;
			rmi4_data->report_points[finger].x = x;
			rmi4_data->report_points[finger].y = y;
			if (rmi4_data->report_points[finger].state != state) {
				if (debug_mask & BIT(3)) {
					if (rmi4_data->width_factor && rmi4_data->height_factor) {
						pr_info("Screen:%c[%02d]:Down, X=%d, Y=%d, Wx=%d, Wy=%d, Z=%d, IM=%d, CIDIM=%d, Freq=%d, NS=%d\n",
							state2char(finger_status),
							finger+1, (rmi4_data->report_points[finger].x*rmi4_data->width_factor)>>SHIFT_BITS,
							(rmi4_data->report_points[finger].y*rmi4_data->height_factor)>>SHIFT_BITS,wx,wy,z,
							rmi4_data->noise_state.im, rmi4_data->noise_state.cidim,
							rmi4_data->noise_state.freq, rmi4_data->noise_state.ns);
					}
					else {
						pr_info("Raw:%c[%02d]:Down, X=%d, Y=%d, Wx=%d, Wy=%d, Z=%d, IM=%d, CIDIM=%d, Freq=%d, NS=%d\n",
						state2char(finger_status),
						finger+1, rmi4_data->report_points[finger].x, rmi4_data->report_points[finger].y,wx,wy,z,
						rmi4_data->noise_state.im, rmi4_data->noise_state.cidim,
						rmi4_data->noise_state.freq, rmi4_data->noise_state.ns);
					}
				}
			}
			rmi4_data->report_points[finger].state = state;

			touch_count++;
			break;
		default:
#ifdef TYPE_B_PROTOCOL
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, 0);
			if (rmi4_data->glove_status & (1 << finger))
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_GLOVE, 1);
			else
				input_report_abs(rmi4_data->input_dev,
						ABS_MT_GLOVE, 0);
#endif
			if (rmi4_data->support_htc_event) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
				input_report_abs(rmi4_data->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION,	(1 << 31));
#endif
			}

			state = 0;
			if (rmi4_data->report_points[finger].state != state) {
				if (debug_mask & BIT(3)) {
					if (rmi4_data->glove_status & (1 << finger)) {
						finger_status = F12_GLOVED_FINGER_STATUS;
					}
					else
						finger_status = F12_FINGER_STATUS;
					if (rmi4_data->width_factor && rmi4_data->height_factor) {
						pr_info("Screen:%c[%02d]:Up, X=%d, Y=%d, Wx=%d, Wy=%d, Z=%d, IM=%d, CIDIM=%d, Freq=%d, NS=%d\n",
							state2char(finger_status),
							finger+1, (rmi4_data->report_points[finger].x*rmi4_data->width_factor)>>SHIFT_BITS,
							(rmi4_data->report_points[finger].y*rmi4_data->height_factor)>>SHIFT_BITS,wx,wy,z,
							rmi4_data->noise_state.im, rmi4_data->noise_state.cidim,
							rmi4_data->noise_state.freq, rmi4_data->noise_state.ns);
					}
					else {
						pr_info("Raw:%c[%02d]:Up, X=%d, Y=%d, Wx=%d, Wy=%d, Z=%d, IM=%d, CIDIM=%d, Freq=%d, NS=%d\n",
						state2char(finger_status),
						finger+1, rmi4_data->report_points[finger].x, rmi4_data->report_points[finger].y,wx,wy,z,
						rmi4_data->noise_state.im, rmi4_data->noise_state.cidim,
						rmi4_data->noise_state.freq, rmi4_data->noise_state.ns);
					}
				}
			}
			rmi4_data->report_points[finger].state = state;

			break;
		}
	}

	if (touch_count == 0) {
#ifdef F12_DATA_15_WORKAROUND
		fingers_already_present = 0;
#endif
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
		if (debug_mask & BIT(1)) {
			dev_info(rmi4_data->pdev->dev.parent,
				"Finger leave\n");
		}
		for (finger = 0; finger < fingers_to_process; finger++) {
			state = 0;
			if (rmi4_data->report_points[finger].state != state) {
				if (debug_mask & BIT(3)) {
					if (rmi4_data->glove_status & (1 << finger)) {
						finger_status = F12_GLOVED_FINGER_STATUS;
					}
					else
						finger_status = F12_FINGER_STATUS;
					if (rmi4_data->width_factor && rmi4_data->height_factor) {
						pr_info("Screen:%c[%02d]:Up, X=%d, Y=%d, IM=%d, CIDIM=%d, Freq=%d, NS=%d\n",
							state2char(finger_status),
							finger+1, (rmi4_data->report_points[finger].x*rmi4_data->width_factor)>>SHIFT_BITS,
							(rmi4_data->report_points[finger].y*rmi4_data->height_factor)>>SHIFT_BITS,
							rmi4_data->noise_state.im, rmi4_data->noise_state.cidim,
							rmi4_data->noise_state.freq, rmi4_data->noise_state.ns);
					}
					else {
						pr_info("Raw:%c[%02d]:Up, X=%d, Y=%d, IM=%d, CIDIM=%d, Freq=%d, NS=%d\n",
							state2char(finger_status),
							finger+1, rmi4_data->report_points[finger].x, rmi4_data->report_points[finger].y,
							rmi4_data->noise_state.im, rmi4_data->noise_state.cidim,
							rmi4_data->noise_state.freq, rmi4_data->noise_state.ns);
					}
				}
			}
			rmi4_data->report_points[finger].state = state;
		}
	}

	input_sync(rmi4_data->input_dev);

	rmi4_data->glove_status = glove_status;

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	static unsigned char do_once = 1;
	static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
	static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

	if (do_once) {
		memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
		memset(before_2d_status, 0, sizeof(before_2d_status));
		memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
		do_once = 0;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			data_addr,
			f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read button data registers\n",
				__func__);
		return;
	}

	data = f1a->button_data_buffer;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

		if (current_status[button] == status)
			continue;
		else
			current_status[button] = status;

		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Button %d (code %d) ->%d\n",
				__func__, button,
				f1a->button_map[button],
				status);
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				before_2d_status[button] = 1;
			} else {
				if (while_2d_status[button] == 1) {
					while_2d_status[button] = 0;
					continue;
				} else {
					before_2d_status[button] = 0;
				}
			}
			touch_count++;
			input_report_key(rmi4_data->input_dev,
					f1a->button_map[button],
					status);
		} else {
			if (before_2d_status[button] == 1) {
				before_2d_status[button] = 0;
				touch_count++;
				input_report_key(rmi4_data->input_dev,
						f1a->button_map[button],
						status);
			} else {
				if (status == 1)
					while_2d_status[button] = 1;
				else
					while_2d_status[button] = 0;
			}
		}
#else
		touch_count++;
		input_report_key(rmi4_data->input_dev,
				f1a->button_map[button],
				status);
#endif
	}

	if (touch_count)
		input_sync(rmi4_data->input_dev);

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	return;
}

static void synaptics_rmi4_f54_report(struct synaptics_rmi4_data *rmi4_data)
{
	int ret, size;
	uint8_t data[2] = {0};

	ret = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f54_data_base_addr + 1,
			data,
			sizeof(data));
	if (ret < 0) {
		dev_info(rmi4_data->pdev->dev.parent," %s write error\n", __func__);
	}
	else {
		size = rmi4_data->num_of_tx * rmi4_data->num_of_rx * 2;
		ret = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f54_data_base_addr + 3,
				rmi4_data->temp_report_data,
				size);

		if (ret >= 0) {
			memcpy(rmi4_data->report_data, rmi4_data->temp_report_data, size);
		}
		else {
			memset(rmi4_data->report_data, 0x0, (4 * rmi4_data->num_of_tx * rmi4_data->num_of_rx));
			dev_info(rmi4_data->pdev->dev.parent," %s Read error\n", __func__);
		}
	}

	atomic_set(&rmi4_data->data_ready, 1);
	wake_up(&syn_data_ready_wq);
}

static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	unsigned char touch_count_2d;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x reporting\n",
			__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F12:
		touch_count_2d = synaptics_rmi4_f12_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F1A:
		synaptics_rmi4_f1a_report(rmi4_data, fhandler);
		break;
	case SYNAPTICS_RMI4_F54:
		synaptics_rmi4_f54_report(rmi4_data);
		break;
	default:
		break;
	}

	return;
}

static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char data[MAX_INTR_REGISTERS + 1];
	unsigned char *intr = &data[1];
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			data,
			rmi4_data->num_of_intr_regs + 1);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read interrupt status\n",
				__func__);
		return;
	}

	status.data[0] = data[0];
	if (status.unconfigured && !status.flash_prog) {
		pr_notice("%s: spontaneous reset detected\n", __func__);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to reinit device\n",
					__func__);
		}
	}

	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
							fhandler);
				}
			}
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (!exp_fhandler->insert &&
					!exp_fhandler->remove &&
					(exp_fhandler->exp_fn->attn != NULL))
				exp_fhandler->exp_fn->attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}

#ifdef MTK_PLATFORM
static struct timespec time_start, time_end, time_delta;
static void synaptics_rmi4_irq_wq(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data = 
			container_of(work, struct synaptics_rmi4_data, work_irq);

	synaptics_rmi4_sensor_report(rmi4_data);

	if (debug_mask & BIT(2)) {
		getnstimeofday(&time_end);
		time_delta.tv_nsec = (time_end.tv_sec*1000000000+time_end.tv_nsec)
			-(time_start.tv_sec*1000000000+time_start.tv_nsec);
		pr_info("Touch latency = %ld us\n", time_delta.tv_nsec/1000);
	}
}

static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (debug_mask & BIT(2)) {
		getnstimeofday(&time_start);
	}
	if (gpio_get_value(bdata->irq_gpio) != bdata->irq_on_state)
		goto exit;

	queue_work(rmi4_data->wq, &rmi4_data->work_irq);
exit:
	return IRQ_HANDLED;
}
#else
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	struct timespec time_start, time_end, time_delta;

	if (debug_mask & BIT(2)) {
		getnstimeofday(&time_start);
	}
	if (gpio_get_value(bdata->irq_gpio) != bdata->irq_on_state)
		goto exit;

	synaptics_rmi4_sensor_report(rmi4_data);

	if (debug_mask & BIT(2)) {
		getnstimeofday(&time_end);
		time_delta.tv_nsec = (time_end.tv_sec*1000000000+time_end.tv_nsec)
			-(time_start.tv_sec*1000000000+time_start.tv_nsec);
		pr_info("Touch latency = %ld us\n", time_delta.tv_nsec/1000);
	}
exit:
	return IRQ_HANDLED;
}
#endif

static int synaptics_rmi4_int_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char ii;
	unsigned char zero = 0x00;
	unsigned char *intr_mask;
	unsigned short intr_addr;
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: %d\n", __func__, enable);

	intr_mask = rmi4_data->intr_mask;

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (intr_mask[ii] != 0x00) {
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			if (enable) {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						intr_addr,
						&(intr_mask[ii]),
						sizeof(intr_mask[ii]));
				if (retval < 0)
					return retval;
			} else {
				retval = synaptics_rmi4_reg_write(rmi4_data,
						intr_addr,
						&zero,
						sizeof(zero));
				if (retval < 0)
					return retval;
			}
		}
	}

	return retval;
}

static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable, bool attn_only)
{
	int retval = 0;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	if (offmode_charging_flag) {
		pr_info("%s, Offmode charging, skip irq disable/enable\n", __func__);
		return -1;
	}
#endif
	if (attn_only) {
		retval = synaptics_rmi4_int_enable(rmi4_data, enable);
		return retval;
	}

	if (enable) {
		if (rmi4_data->irq_enabled)
			return retval;

		retval = synaptics_rmi4_int_enable(rmi4_data, false);
		if (retval < 0)
			return retval;

		/* Process and clear interrupts */
		synaptics_rmi4_sensor_report(rmi4_data);

		enable_irq(rmi4_data->irq);

		retval = synaptics_rmi4_int_enable(rmi4_data, true);
		if (retval < 0)
			return retval;

		rmi4_data->irq_enabled = true;
	} else {
		if (rmi4_data->irq_enabled) {
			disable_irq(rmi4_data->irq);
			rmi4_data->irq_enabled = false;
		}
	}

	return retval;
}

static void synaptics_rmi4_set_intr_mask(struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	unsigned char ii;
	unsigned char intr_offset;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	return;
}

static int synaptics_rmi4_f01_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f01_query_base_addr = fd->query_base_addr;
	rmi4_data->f01_ctrl_base_addr = fd->ctrl_base_addr;
	rmi4_data->f01_data_base_addr = fd->data_base_addr;
	rmi4_data->f01_cmd_base_addr = fd->cmd_base_addr;

	return 0;
}

static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int retval;
	unsigned char abs_data_size;
	unsigned char abs_data_blk_size;
	unsigned char query[F11_STD_QUERY_LEN];
	unsigned char control[F11_STD_CTRL_LEN];

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base,
			query,
			sizeof(query));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if ((query[1] & MASK_3BIT) <= 4)
		fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
	else if ((query[1] & MASK_3BIT) == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			control,
			sizeof(control));
	if (retval < 0)
		return retval;

	/* Maximum x, y and z */
	rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
			((control[7] & MASK_4BIT) << 8);
	rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
			((control[9] & MASK_4BIT) << 8);
#ifdef REPORT_2D_Z
	rmi4_data->sensor_max_z = 255;
#endif
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Function %02x max x = %d max y = %d max z = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y,
			rmi4_data->sensor_max_z);

	if (bdata->display_width && bdata->display_height
		&& rmi4_data->sensor_max_x && rmi4_data->sensor_max_y) {

		dev_info(rmi4_data->pdev->dev.parent, "%s Load display resolution: %dx%d\n",
				__func__, bdata->display_width, bdata->display_height);
		rmi4_data->width_factor = (bdata->display_width<<SHIFT_BITS)/rmi4_data->sensor_max_x;
		rmi4_data->height_factor = (bdata->display_height<<SHIFT_BITS)/rmi4_data->sensor_max_y;
	}

	rmi4_data->max_touch_width = synaptics_sqrt(
			MAX_F11_TOUCH_WIDTH * MAX_F11_TOUCH_WIDTH * 2);

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	abs_data_size = query[5] & MASK_2BIT;
	abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
	fhandler->size_of_data_register_block = abs_data_blk_size;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28)
{
	int retval;
	static unsigned short ctrl_28_address;

	if (ctrl28)
		ctrl_28_address = ctrl28;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			ctrl_28_address,
			&rmi4_data->report_enable,
			sizeof(rmi4_data->report_enable));
	if (retval < 0)
		return retval;

	return retval;
}

static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int retval;
	unsigned char size_of_2d_data;
	unsigned char size_of_query8;
	unsigned char ctrl_8_offset;
	unsigned char ctrl_9_offset;
	unsigned char ctrl_10_offset;
	unsigned char ctrl_11_offset;
	unsigned char ctrl_15_offset;
	unsigned char ctrl_20_offset;
	unsigned char ctrl_23_offset;
	unsigned char ctrl_28_offset;
	unsigned char num_of_fingers;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_query_5 query_5;
	struct synaptics_rmi4_f12_query_8 query_8;
	struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
	struct synaptics_rmi4_f12_ctrl_23 ctrl_23;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	if (!fhandler->extra) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 5,
			query_5.data,
			sizeof(query_5.data));
	if (retval < 0)
		return retval;

	ctrl_8_offset = query_5.ctrl0_is_present +
			query_5.ctrl1_is_present +
			query_5.ctrl2_is_present +
			query_5.ctrl3_is_present +
			query_5.ctrl4_is_present +
			query_5.ctrl5_is_present +
			query_5.ctrl6_is_present +
			query_5.ctrl7_is_present;

	ctrl_9_offset = ctrl_8_offset +
			query_5.ctrl8_is_present;

	ctrl_10_offset = ctrl_9_offset +
			query_5.ctrl9_is_present;

	ctrl_11_offset = ctrl_10_offset +
			query_5.ctrl10_is_present;

	ctrl_15_offset = ctrl_11_offset +
			query_5.ctrl11_is_present +
			query_5.ctrl12_is_present +
			query_5.ctrl13_is_present +
			query_5.ctrl14_is_present;

	ctrl_20_offset = ctrl_15_offset +
			query_5.ctrl15_is_present +
			query_5.ctrl16_is_present +
			query_5.ctrl17_is_present +
			query_5.ctrl18_is_present +
			query_5.ctrl19_is_present;

	ctrl_23_offset = ctrl_20_offset +
			query_5.ctrl20_is_present +
			query_5.ctrl21_is_present +
			query_5.ctrl22_is_present;

	ctrl_28_offset = ctrl_23_offset +
			query_5.ctrl23_is_present +
			query_5.ctrl24_is_present +
			query_5.ctrl25_is_present +
			query_5.ctrl26_is_present +
			query_5.ctrl27_is_present;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			ctrl_23.data,
			sizeof(ctrl_23.data));
	if (retval < 0)
		return retval;

	extra_data->ctrl23_offset = ctrl_23_offset;
	/* Maximum number of fingers supported */
	fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
			(unsigned char)F12_FINGERS_TO_SUPPORT);

	num_of_fingers = fhandler->num_of_data_points;
	rmi4_data->num_of_fingers = num_of_fingers;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base + 8,
			query_8.data,
			sizeof(query_8.data));
	if (retval < 0)
		return retval;

	/* Determine the presence of the Data0 register */
	extra_data->data1_offset = query_8.data0_is_present;

	if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
		extra_data->data15_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present +
				query_8.data4_is_present +
				query_8.data5_is_present +
				query_8.data6_is_present +
				query_8.data7_is_present +
				query_8.data8_is_present +
				query_8.data9_is_present +
				query_8.data10_is_present +
				query_8.data11_is_present +
				query_8.data12_is_present +
				query_8.data13_is_present +
				query_8.data14_is_present;
		extra_data->data15_size = (num_of_fingers + 7) / 8;
	} else {
		extra_data->data15_size = 0;
	}

	rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
	rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
	rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

	retval = synaptics_rmi4_f12_set_enables(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_28_offset);
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_8_offset,
			ctrl_8.data,
			sizeof(ctrl_8.data));
	if (retval < 0)
		return retval;

	/* Maximum x, y and z */
	rmi4_data->sensor_max_x =
			((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_x_coord_msb << 8);
	rmi4_data->sensor_max_y =
			((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_y_coord_msb << 8);
#ifdef REPORT_2D_Z
	rmi4_data->sensor_max_z = 255;
#endif
	rmi4_data->num_of_rx = ctrl_8.num_of_rx;
	rmi4_data->num_of_tx = ctrl_8.num_of_tx;
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Function %02x max x = %d max y = %d max z = %d Rx: %d Tx: %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y,
			rmi4_data->sensor_max_z,
			rmi4_data->num_of_rx,
			rmi4_data->num_of_tx);

	if (bdata->display_width && bdata->display_height
		&& rmi4_data->sensor_max_x && rmi4_data->sensor_max_y) {

		dev_info(rmi4_data->pdev->dev.parent, "%s Load display resolution: %dx%d\n",
				__func__, bdata->display_width, bdata->display_height);
		rmi4_data->width_factor = (bdata->display_width<<SHIFT_BITS)/rmi4_data->sensor_max_x;
		rmi4_data->height_factor = (bdata->display_height<<SHIFT_BITS)/rmi4_data->sensor_max_y;
	}

	rmi4_data->max_touch_width = synaptics_sqrt(
			rmi4_data->num_of_rx*rmi4_data->num_of_rx +
			rmi4_data->num_of_tx*rmi4_data->num_of_tx);

	rmi4_data->f12_wakeup_gesture = query_5.ctrl27_is_present;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	extra_data->ctrl20_offset = ctrl_20_offset;
#endif
	if (rmi4_data->f12_wakeup_gesture) {
		extra_data->ctrl20_offset = ctrl_20_offset;
		extra_data->data4_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present;
	}

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	/* Allocate memory for finger data storage space */
	fhandler->data_size = num_of_fingers * size_of_2d_data;
	fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);
	if (!fhandler->data) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	if (bdata->support_cover || bdata->support_glove) {
		extra_data->ctrl9_offset = ctrl_9_offset;
		extra_data->ctrl10_offset = ctrl_10_offset;
		extra_data->ctrl11_offset = ctrl_11_offset;
		extra_data->ctrl15_offset = ctrl_15_offset;
	}

	if (rmi4_data->temp_report_data != NULL)
		kfree(rmi4_data->temp_report_data);
	if (rmi4_data->report_data != NULL)
		kfree(rmi4_data->report_data);
	rmi4_data->temp_report_data = kzalloc(4 * rmi4_data->num_of_tx * rmi4_data->num_of_rx, GFP_KERNEL);
	rmi4_data->report_data = kzalloc(4 * rmi4_data->num_of_tx * rmi4_data->num_of_rx, GFP_KERNEL);
	if(rmi4_data->temp_report_data == NULL || rmi4_data->report_data == NULL) {
		dev_err(rmi4_data->pdev->dev.parent," %s report data init fail\n", __func__);
		return -1;
	}

	dev_info(rmi4_data->pdev->dev.parent," %s report data init done\n",__func__);

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for function handle\n",
				__func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;
	fhandler->extra = NULL;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.query_base,
			f1a->button_query.data,
			sizeof(f1a->button_query.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read query registers\n",
				__func__);
		return retval;
	}

	f1a->max_count = f1a->button_query.max_button_count + 1;

	f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
	if (!f1a->button_control.txrx_map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for tx rx mapping\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
			sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for data buffer\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->max_count,
			sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for button map\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char ii;
	unsigned char mapping_offset = 0;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	mapping_offset = f1a->button_query.has_general_control +
			f1a->button_query.has_interrupt_enable +
			f1a->button_query.has_multibutton_select;

	if (f1a->button_query.has_tx_rx_map) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				fhandler->full_addr.ctrl_base + mapping_offset,
				f1a->button_control.txrx_map,
				sizeof(f1a->button_control.txrx_map));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read tx rx mapping\n",
					__func__);
			return retval;
		}

		rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
	}

	if (!bdata->cap_button_map) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: cap_button_map is NULL in board file\n",
				__func__);
		return -ENODEV;
	} else if (!bdata->cap_button_map->map) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Button map is missing in board file\n",
				__func__);
		return -ENODEV;
	} else {
		if (bdata->cap_button_map->nbuttons != f1a->max_count) {
			f1a->valid_button_count = min(f1a->max_count,
					bdata->cap_button_map->nbuttons);
		} else {
			f1a->valid_button_count = f1a->max_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] = bdata->cap_button_map->map[ii];
	}

	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_control.txrx_map);
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}

static int synaptics_rmi4_f34_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count,
		unsigned int page_number)
{
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f34_query_base_addr =
		(fd->query_base_addr | (page_number << 8));
	rmi4_data->f34_ctrl_base_addr =
		(fd->ctrl_base_addr | (page_number << 8));
	rmi4_data->f34_data_base_addr =
		(fd->data_base_addr | (page_number << 8));
	rmi4_data->f34_cmd_base_addr =
		(fd->cmd_base_addr | (page_number << 8));

	return 0;
}

static int synaptics_rmi4_f54_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count,
		unsigned int page_number)
{
	struct synaptics_rmi4_f54_query f54_query;
	struct synaptics_rmi4_f54_query_13 query_13;
	unsigned char data_offset = 3;
	unsigned char offset;
	int retval;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f54_query_base_addr =
		(fd->query_base_addr | (page_number << 8));
	rmi4_data->f54_ctrl_base_addr =
		(fd->ctrl_base_addr | (page_number << 8));
	rmi4_data->f54_data_base_addr =
		 (fd->data_base_addr | (page_number << 8));
	rmi4_data->f54_cmd_base_addr =
		 (fd->cmd_base_addr | (page_number << 8));

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f54_query_base_addr,
			f54_query.data,
			sizeof(f54_query));
	if (retval < 0)
		return retval;

	offset = sizeof(f54_query.data);

	/* query 12 */
	if (f54_query.has_sense_frequency_control == 0)
		offset -= 1;

	/* query 13 */
	if (f54_query.has_query13) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f54_query_base_addr + offset,
				query_13.data,
				sizeof(query_13.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* data 4 */
	if (f54_query.has_sense_frequency_control == 1)
		data_offset += 1;

	/* data 6 */
	if (f54_query.has_interference_metric == 1) {
		rmi4_data->f54_im_offset = data_offset + 1;
		data_offset += 2;
	}

	/* data 7.0 */
	if ((f54_query.has_two_byte_report_rate == 1) | (f54_query.has_one_byte_report_rate == 1))
		data_offset += 1;

	/* data 7.1 */
	if (f54_query.has_two_byte_report_rate == 1)
		data_offset += 1;

	/* data 8 */
	if (f54_query.has_variance_metric == 1)
		data_offset += 2;

	/* data 9 */
	if (f54_query.has_multi_metric_state_machine == 1)
		data_offset += 2;

	/* data 10 */
	if ((f54_query.has_multi_metric_state_machine == 1) | (f54_query.has_noise_state == 1)) {
		data_offset += 1;
		rmi4_data->f54_ns_offset = data_offset;
	}

	/* data 11 */
	if (f54_query.has_status == 1)
		data_offset += 1;

	/* data 12 */
	if (f54_query.has_slew_metric == 1)
		data_offset += 2;

	/* data 13 */
	if (f54_query.has_multi_metric_state_machine == 1)
		data_offset += 2;

	/* data 14 */
	if (query_13.has_cidim == 1) {
		data_offset += 1;
		rmi4_data->f54_cidim_offset = data_offset;
	}

	/* data 15 */
	if (query_13.has_rail_im == 1)
		data_offset += 1;

	/* data 16 */
	if (query_13.has_noise_mitigation_enhancement == 1) {
		data_offset += 1;
		rmi4_data->f54_freq_offset = data_offset;
	}

	return 0;
}

static int synaptics_rmi4_f51_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count,
		unsigned int page_number)
{
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f51_query_base_addr =
		(fd->query_base_addr | (page_number << 8));
	rmi4_data->f51_ctrl_base_addr =
		(fd->ctrl_base_addr | (page_number << 8));
	rmi4_data->f51_data_base_addr =
		 (fd->data_base_addr | (page_number << 8));
	rmi4_data->f51_cmd_base_addr =
		 (fd->cmd_base_addr | (page_number << 8));

	return 0;
}

static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_fn *fhandler_temp;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry_safe(fhandler,
				fhandler_temp,
				&rmi->support_fn_list,
				link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			list_del(&fhandler->link);
			kfree(fhandler);
		}
	}
	INIT_LIST_HEAD(&rmi->support_fn_list);

	return;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
		bool *was_in_bl_mode)
{
	int retval;
	int timeout = CHECK_STATUS_TIMEOUT_MS;
	unsigned char intr_status;
	struct synaptics_rmi4_f01_device_status status;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status.data,
			sizeof(status.data));
	if (retval < 0)
		return retval;

	while (status.status_code == STATUS_CRC_IN_PROGRESS) {
		if (timeout > 0)
			msleep(20);
		else
			return -1;

		retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (retval < 0)
			return retval;

		timeout -= 20;
	}

	if (timeout != CHECK_STATUS_TIMEOUT_MS)
		*was_in_bl_mode = true;

	if (status.flash_prog == 1) {
		rmi4_data->flash_prog_mode = true;
		pr_notice("%s: In flash prog mode, status = 0x%02x\n",
				__func__,
				status.status_code);
	} else {
		rmi4_data->flash_prog_mode = false;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			&intr_status,
			sizeof(intr_status));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read interrupt status\n",
				__func__);
		return retval;
	}

	return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set configured\n",
				__func__);
		return;
	}

	rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
	device_ctrl |= CONFIGURED;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to set configured\n",
				__func__);
	}

	return;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));

	return 0;
}

static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char page_number;
	unsigned char intr_count;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	unsigned short pdt_entry_addr;
	unsigned char config_id_size;
	bool f01found;
	bool was_in_bl_mode;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
	uint8_t data[32]={0};
	char tmp_buf[5];
	unsigned char ii;

	rmi = &(rmi4_data->rmi4_mod_info);

rescan_pdt:
	f01found = false;
	was_in_bl_mode = false;
	intr_count = 0;
	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_reg_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			pdt_entry_addr &= ~(MASK_8BIT << 8);

			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				dev_dbg(rmi4_data->pdev->dev.parent,
						"%s: Reached end of PDT\n",
						__func__);
				break;
			}

			dev_info(rmi4_data->pdev->dev.parent,
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				if (rmi_fd.intr_src_count == 0)
					break;

				f01found = true;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f01_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;

				retval = synaptics_rmi4_check_status(rmi4_data,
						&was_in_bl_mode);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to check status\n",
							__func__);
					return retval;
				}

				if (was_in_bl_mode) {
					kfree(fhandler);
					fhandler = NULL;
					goto rescan_pdt;
				}

				if (rmi4_data->flash_prog_mode)
					goto flash_prog_mode;

				break;
			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F12:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f12_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0) {
#ifdef IGNORE_FN_INIT_FAILURE
					kfree(fhandler);
					fhandler = NULL;
#else
					return retval;
#endif
				}
				break;
			case SYNAPTICS_RMI4_F34:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}
				retval = synaptics_rmi4_f34_init(rmi4_data,
						fhandler, &rmi_fd, intr_count,page_number);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F54:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}
				retval = synaptics_rmi4_f54_init(rmi4_data,
						fhandler, &rmi_fd, intr_count,page_number);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F51:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}
				retval = synaptics_rmi4_f51_init(rmi4_data,
						fhandler, &rmi_fd, intr_count,page_number);
				if (retval < 0)
					return retval;
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

	if (!f01found) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to find F01\n",
				__func__);
		return -EINVAL;
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2];
	rmi->product_info[1] = f01_query[3];
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Non-Synaptics device found, manufacturer ID = %d\n",
				__func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_CHIP_ID_OFFSET,
			rmi->package_id,
			sizeof(rmi->package_id));
	if (retval < 0)
		return retval;

	rmi4_data->chip_id = (unsigned int)rmi->package_id[0] +
			(unsigned int)rmi->package_id[1] * 0x100;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (retval < 0)
		return retval;

	rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] +
			(unsigned int)rmi->build_id[1] * 0x100 +
			(unsigned int)rmi->build_id[2] * 0x10000;

	dev_info(rmi4_data->pdev->dev.parent, "%s: chip_id:%d, firmware_id:%d\n",
				__func__, rmi4_data->chip_id, rmi4_data->firmware_id);

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	switch (rmi4_data->chip_id) {
		case 3708:
			config_id_size = V7_CONFIG_ID_SIZE;
			break;
		default:
			config_id_size = V5V6_CONFIG_ID_SIZE;
			break;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f34_ctrl_base_addr,
			data,
			sizeof(uint8_t) * config_id_size);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent," %s err r:4\n",__func__);
		return retval;
	}

	memset(rmi4_data->config_version, 0, sizeof(rmi4_data->config_version));
	for (ii = 0; ii < config_id_size; ii++) {
		snprintf(tmp_buf, 3, "%02x ", data[ii]);
		strlcat(rmi4_data->config_version, tmp_buf, sizeof(rmi4_data->config_version));
	}

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: config_version: %s\n",
			__func__,
			rmi4_data->config_version);

	//rmi4_data->config_version = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	//dev_info(rmi4_data->pdev->dev.parent, "%s config_version: %08X\n", __func__, rmi4_data->config_version);

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
						fhandler->intr_mask;
			}
		}
	}

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture)
		rmi4_data->enable_wakeup_gesture = WAKEUP_GESTURE;
	else
		rmi4_data->enable_wakeup_gesture = false;

	synaptics_rmi4_set_configured(rmi4_data);

	return 0;
}

static int synaptics_rmi4_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	char buf[16] = {0};

	if (config) {
		snprintf(buf, 16, "dsx_gpio_%u", gpio);
		retval = gpio_request(gpio, buf);
		if (retval) {
			pr_info("%s: Failed to get gpio %d (code: %d)",
					__func__, gpio, retval);
			return retval;
		}

#ifdef MTK_PLATFORM
		gpio |= 0x80000000;
		if (dir == 0) {
			mt_set_gpio_mode(gpio, GPIO_CTP_EINT_PIN_M_EINT);
			mt_set_gpio_dir(gpio, GPIO_DIR_IN);
			mt_set_gpio_pull_enable(gpio, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(gpio, 1);
			mdelay(50);
		} else {
			mt_set_gpio_mode(gpio, GPIO_CTP_RST_PIN_M_GPIO);
			mt_set_gpio_dir(gpio, GPIO_DIR_OUT);
			mt_set_gpio_out(gpio, state);
			mdelay(50);
		}
#else
		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
#endif
	} else {
		gpio_free(gpio);
	}

	return retval;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
#ifdef REPORT_2D_Z
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_PRESSURE, 0,
			rmi4_data->sensor_max_z, 0, 0);
#endif
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
	if (rmi4_data->input_dev->mt &&
		rmi4_data->input_dev->mt->num_slots != rmi4_data->num_of_fingers)
		input_mt_destroy_slots(rmi4_data->input_dev);
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers, 0);
	if (bdata->support_glove) {
		input_set_abs_params(rmi4_data->input_dev, ABS_MT_GLOVE, 0, 1, 0, 0);
	}
#endif

	if (rmi4_data->support_htc_event) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
		input_set_abs_params(rmi4_data->input_dev, ABS_MT_AMPLITUDE,
			0, ((rmi4_data->sensor_max_z << 16) | (rmi4_data->max_touch_width + rmi4_data->max_touch_width)), 0, 0);
		input_set_abs_params(rmi4_data->input_dev, ABS_MT_POSITION,
			0, ((1 << 31) | (rmi4_data->sensor_max_x << 16) | rmi4_data->sensor_max_y), 0, 0);
#endif
	}

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
	}

	if (rmi4_data->f11_wakeup_gesture || rmi4_data->f12_wakeup_gesture) {
		set_bit(KEY_WAKEUP, rmi4_data->input_dev->keybit);
		input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_WAKEUP);
	}

	return;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to allocate input device\n",
				__func__);
		retval = -ENOMEM;
		goto err_input_device;
	}

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to query device\n",
				__func__);
		goto err_query_device;
	}

	rmi4_data->input_dev->name = PLATFORM_DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->dev.parent = rmi4_data->pdev->dev.parent;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	if (rmi4_data->hw_if->board_data->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	synaptics_rmi4_set_params(rmi4_data);

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to register input device\n",
				__func__);
		goto err_register_input;
	}

	return 0;

err_register_input:
err_query_device:
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_free_device(rmi4_data->input_dev);

err_input_device:
	return retval;
}

#ifndef MTK_PLATFORM
static int synaptics_dsx_pinctrl_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	rmi4_data->ts_pinctrl = devm_pinctrl_get((rmi4_data->pdev->dev.parent));
	if (IS_ERR_OR_NULL(rmi4_data->ts_pinctrl)) {
		dev_info(rmi4_data->pdev->dev.parent,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(rmi4_data->ts_pinctrl);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	rmi4_data->gpio_state_active
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(rmi4_data->gpio_state_active)) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(rmi4_data->gpio_state_active);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	rmi4_data->gpio_state_suspend
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(rmi4_data->gpio_state_suspend)) {
		dev_dbg(rmi4_data->pdev->dev.parent,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(rmi4_data->gpio_state_suspend);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static void synaptics_dsx_pinctrl_deinit(struct synaptics_rmi4_data *rmi4_data)
{
	/* Put pinctrl if target uses pinctrl */
	if (rmi4_data->ts_pinctrl != NULL) {
		rmi4_data->gpio_state_active = NULL;
		rmi4_data->gpio_state_suspend = NULL;
		devm_pinctrl_put(rmi4_data->ts_pinctrl);
	}
}

static int synaptics_dsx_pinctrl_select(struct synaptics_rmi4_data *rmi4_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? rmi4_data->gpio_state_active
		: rmi4_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(rmi4_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_info(rmi4_data->pdev->dev.parent,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		dev_err(rmi4_data->pdev->dev.parent,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");

	return 0;
}
#endif

static int synaptics_rmi4_set_gpio(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	retval = synaptics_rmi4_gpio_setup(
			bdata->irq_gpio,
			true, 0, 0);
	if (retval < 0) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Failed to configure attention GPIO\n",
				__func__);
		goto err_gpio_irq;
	}

	if (bdata->power_gpio >= 0) {
		retval = synaptics_rmi4_gpio_setup(
				bdata->power_gpio,
				true, 1, bdata->power_on_state);
		if (retval == -EBUSY)
		{
			rmi4_data->hw_if->board_data->power_gpio = -1;
			dev_info(rmi4_data->pdev->dev.parent,
					"%s: power GPIO has been requested, by pass.\n",
					__func__);
		}
		else if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to configure power GPIO\n",
					__func__);
			goto err_gpio_power;
		}
	}

	if (bdata->power_gpio_1v8 >= 0) {
		retval = synaptics_rmi4_gpio_setup(
				bdata->power_gpio_1v8,
				true, 1, bdata->power_on_state);
		if (retval == -EBUSY)
		{
			rmi4_data->hw_if->board_data->power_gpio_1v8 = -1;
			dev_info(rmi4_data->pdev->dev.parent,
					"%s: power-1v8 GPIO has been requested, by pass.\n",
					__func__);
		}
		else if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to configure power-1v8 GPIO\n",
					__func__);
			goto err_gpio_power_1v8;
		}
	}

	if (bdata->reset_gpio >= 0) {
		retval = synaptics_rmi4_gpio_setup(
				bdata->reset_gpio,
				true, 1, !bdata->reset_on_state);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to configure reset GPIO\n",
					__func__);
			goto err_gpio_reset;
		}
	}

	if (bdata->switch_gpio >= 0) {
		retval = synaptics_rmi4_gpio_setup(
				bdata->switch_gpio,
				true, 1, 0);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to configure switch GPIO\n",
					__func__);
			goto err_gpio_switch;
		}
	}

	if (bdata->power_gpio >= 0) {
		gpio_set_value(bdata->power_gpio, bdata->power_on_state);
		msleep(bdata->power_delay_ms);
	}

	if (bdata->power_gpio_1v8 >= 0) {
		gpio_set_value(bdata->power_gpio_1v8, bdata->power_on_state);
		msleep(bdata->power_delay_ms);
	}

	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		msleep(bdata->reset_delay_ms);
	}

	return 0;

err_gpio_switch:
	if (bdata->switch_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->switch_gpio, false, 0, 0);
err_gpio_reset:
	if (bdata->power_gpio_1v8 >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio_1v8, false, 0, 0);
err_gpio_power_1v8:
	if (bdata->power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio, false, 0, 0);

err_gpio_power:
	synaptics_rmi4_gpio_setup(bdata->irq_gpio, false, 0, 0);

err_gpio_irq:
	return retval;
}

static int synaptics_rmi4_get_reg(struct synaptics_rmi4_data *rmi4_data,
		bool get)
{
	int retval;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if ((bdata->pwr_reg_name != NULL) && (*bdata->pwr_reg_name != 0)) {
		rmi4_data->pwr_reg = regulator_get(rmi4_data->pdev->dev.parent,
				bdata->pwr_reg_name);
		if (IS_ERR(rmi4_data->pwr_reg)) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to get power regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->pwr_reg);
			goto regulator_put;
		}
	}

	if ((bdata->bus_reg_name != NULL) && (*bdata->bus_reg_name != 0)) {
		rmi4_data->bus_reg = regulator_get(rmi4_data->pdev->dev.parent,
				bdata->bus_reg_name);
		if (IS_ERR(rmi4_data->bus_reg)) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to get bus pullup regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->bus_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (rmi4_data->pwr_reg) {
		regulator_put(rmi4_data->pwr_reg);
		rmi4_data->pwr_reg = NULL;
	}

	if (rmi4_data->bus_reg) {
		regulator_put(rmi4_data->bus_reg);
		rmi4_data->bus_reg = NULL;
	}

	return retval;
}

static int synaptics_rmi4_enable_reg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	if (!enable) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (rmi4_data->bus_reg) {
		retval = regulator_enable(rmi4_data->bus_reg);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable bus pullup regulator\n",
					__func__);
			goto exit;
		}
	}

	if (rmi4_data->pwr_reg) {
		retval = regulator_enable(rmi4_data->pwr_reg);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable power regulator\n",
					__func__);
			goto disable_bus_reg;
		}
		msleep(bdata->power_delay_ms);
	}

	return 0;

disable_pwr_reg:
	if (rmi4_data->pwr_reg)
		regulator_disable(rmi4_data->pwr_reg);

disable_bus_reg:
	if (rmi4_data->bus_reg)
		regulator_disable(rmi4_data->bus_reg);

exit:
	return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;

	mutex_lock(&(rmi4_data->rmi4_report_mutex));

#ifdef TYPE_B_PROTOCOL
	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);
		if (rmi4_data->glove_status & (1 << ii))
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_GLOVE, 1);
		else
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_GLOVE, 0);

		if (rmi4_data->support_htc_event) {
#if defined(ABS_MT_AMPLITUDE) && defined(ABS_MT_POSITION)
			input_report_abs(rmi4_data->input_dev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION,	(1 << 31));
#endif
		}
		rmi4_data->report_points[ii].state = 0;
	}
	rmi4_data->glove_status = 0;
#endif
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(rmi4_data->input_dev);
#endif
	input_sync(rmi4_data->input_dev);

	mutex_unlock(&(rmi4_data->rmi4_report_mutex));

	rmi4_data->fingers_on_2d = false;

	return 0;
}

static int synaptics_rmi4_force_cal(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char command = 0x02;

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f54_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
int synaptics_rmi4_reset(void)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	return synaptics_rmi4_hw_reset_device(rmi4_data);
}
EXPORT_SYMBOL(synaptics_rmi4_reset);
#endif

static int synaptics_rmi4_hw_reset(struct synaptics_rmi4_data *rmi4_data)
{
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	int retval;

	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
	}

	if (rmi4_data->hw_if->ui_hw_init) {
		retval = rmi4_data->hw_if->ui_hw_init(rmi4_data);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int synaptics_rmi4_sw_reset(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char command = 0x01;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	msleep(rmi4_data->hw_if->board_data->reset_delay_ms);

	if (rmi4_data->hw_if->ui_hw_init) {
		retval = rmi4_data->hw_if->ui_hw_init(rmi4_data);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	synaptics_rmi4_free_fingers(rmi4_data);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F12) {
				synaptics_rmi4_f12_set_enables(rmi4_data, 0);
				break;
			}
		}
	}

	synaptics_rmi4_empty_fn_list(rmi4_data);

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to query device\n",
				__func__);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	if (rmi4_data->hw_if->board_data->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	synaptics_rmi4_set_params(rmi4_data);

	retval = synaptics_rmi4_int_enable(rmi4_data, true);
	if (retval < 0)
		goto exit;

	rmi4_data->sensor_sleep = false;
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reinit != NULL)
				exp_fhandler->exp_fn->reinit(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	synaptics_rmi4_set_configured(rmi4_data);

	if (rmi4_data->cover_mode || (rmi4_data->glove_setting & 0x01)) {
		synaptics_rmi4_set_chip_mode(rmi4_data);
	}
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	else
		synaptics_rmi4_querry_f51_data(rmi4_data);
#endif
	retval = 0;

exit:
	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	return retval;
}

static int synaptics_rmi4_hw_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	pr_info(" %s\n", __func__);

	retval = synaptics_rmi4_hw_reset(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command\n",
				__func__);
		return retval;
	}

	return 0;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	pr_info(" %s\n", __func__);
	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	synaptics_rmi4_irq_enable(rmi4_data, false, false);

	retval = synaptics_rmi4_sw_reset(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to issue reset command\n",
				__func__);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	synaptics_rmi4_free_fingers(rmi4_data);

	synaptics_rmi4_empty_fn_list(rmi4_data);

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to query device\n",
				__func__);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	if (rmi4_data->hw_if->board_data->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	synaptics_rmi4_set_params(rmi4_data);

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reset != NULL)
				exp_fhandler->exp_fn->reset(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	if (rmi4_data->cover_mode || (rmi4_data->glove_setting & 0x01)) {
		synaptics_rmi4_set_chip_mode(rmi4_data);
	}
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	else
		synaptics_rmi4_querry_f51_data(rmi4_data);
#endif

	synaptics_rmi4_sensor_wake(rmi4_data);

	synaptics_rmi4_irq_enable(rmi4_data, true, false);

	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

	return 0;
}

static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler_temp;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	mutex_lock(&rmi4_data->rmi4_exp_init_mutex);
	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
				exp_fhandler_temp,
				&exp_data.list,
				link) {
			if ((exp_fhandler->exp_fn->init != NULL) &&
					exp_fhandler->insert) {
				exp_fhandler->exp_fn->init(rmi4_data);
				exp_fhandler->insert = false;
			} else if ((exp_fhandler->exp_fn->remove != NULL) &&
					exp_fhandler->remove) {
				exp_fhandler->exp_fn->remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);
	mutex_unlock(&rmi4_data->rmi4_exp_init_mutex);

	return;
}

void synaptics_rmi4_new_function(struct synaptics_rmi4_exp_fn *exp_fn,
		bool insert)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

	mutex_lock(&exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			pr_err("%s: Failed to alloc mem for expansion function\n",
					__func__);
			goto exit;
		}
		exp_fhandler->exp_fn = exp_fn;
		exp_fhandler->insert = true;
		exp_fhandler->remove = false;
		list_add_tail(&exp_fhandler->link, &exp_data.list);
	} else if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->exp_fn->fn_type == exp_fn->fn_type) {
				exp_fhandler->insert = false;
				exp_fhandler->remove = true;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&exp_data.mutex);

	if (exp_data.queue_work) {
		queue_delayed_work(exp_data.workqueue,
				&exp_data.work,
				msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	}

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
static struct notifier_block facedown_status_handler = {
	.notifier_call = facedown_status_handler_func,
};
#endif

#ifdef MTK_PLATFORM
static int irq_registration(unsigned int *irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if (node) {
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		*irq = irq_of_parse_and_map(node, 0);

		ret = request_irq(*irq, handler, flags, "TOUCH_PANEL-eint", dev);
		if (ret != 0) {
			ret = -1;
			pr_err("request_irq IRQ LINE NOT AVAILABLE!.");
		}
	} else {
		pr_err("request_irq can not find touch eint device node!.");
		ret = -1;
	}

	pr_info("[%s]irq:%d, debounce:%d-%d:", __FUNCTION__, *irq, ints[0], ints[1]);

	return ret;
}
#endif

static int check_chip_exist(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char data;
	int retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			(PDT_PROPS-1),
			&data,
			sizeof(data));
	if (retval > 0 && (data == SYNAPTICS_RMI4_F34)) {
		dev_info(rmi4_data->pdev->dev.parent, "%s: Synaptics chip exist\n", __func__);
		return 0;
	}

	return -1;
}

static int synaptics_rmi4_probe(struct platform_device *pdev)
{
	int retval;
	int attr_count;
	struct synaptics_rmi4_data *rmi4_data;
	const struct synaptics_dsx_hw_interface *hw_if;
	const struct synaptics_dsx_board_data *bdata;
	pr_info(" %s\n", __func__);

	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		dev_err(&pdev->dev,
				"%s: No hardware interface found\n",
				__func__);
		return -EINVAL;
	}

	bdata = hw_if->board_data;
	if (!bdata) {
		dev_err(&pdev->dev,
				"%s: No board data found\n",
				__func__);
		return -EINVAL;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&pdev->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi4_data->pdev = pdev;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->hw_if = hw_if;
	rmi4_data->sensor_sleep = false;
	rmi4_data->suspend = false;
	rmi4_data->irq_enabled = false;
	rmi4_data->fingers_on_2d = false;
	rmi4_data->f11_wakeup_gesture = false;
	rmi4_data->f12_wakeup_gesture = false;

	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;
	rmi4_data->hall_block_touch_event = 0;

	mutex_init(&(rmi4_data->rmi4_reset_mutex));
	mutex_init(&(rmi4_data->rmi4_report_mutex));
	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
	mutex_init(&(rmi4_data->rmi4_exp_init_mutex));

	platform_set_drvdata(pdev, rmi4_data);

#ifndef MTK_PLATFORM
	retval = synaptics_rmi4_get_reg(rmi4_data, true);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to get regulators\n",
				__func__);
		goto err_get_reg;
	}

	retval = synaptics_rmi4_enable_reg(rmi4_data, true);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to enable regulators\n",
				__func__);
		goto err_enable_reg;
	}

	retval = synaptics_dsx_pinctrl_init(rmi4_data);
	if (!retval && (rmi4_data->ts_pinctrl != NULL)) {
		retval = synaptics_dsx_pinctrl_select(rmi4_data, true);
		if (retval < 0)
			goto err_config_gpio;
	}
#endif

	retval = synaptics_rmi4_set_gpio(rmi4_data);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to set up GPIO's\n",
				__func__);
		goto err_set_gpio;
	}

	if (hw_if->ui_hw_init) {
		retval = hw_if->ui_hw_init(rmi4_data);
		if (retval < 0) {
			dev_err(&pdev->dev,
					"%s: Failed to initialize hardware interface\n",
					__func__);
			goto err_ui_hw_init;
		}
	}

	if (check_chip_exist(rmi4_data) < 0) {
		dev_info(&pdev->dev, "%s: No Synaptics chip\n", __func__);
		retval = -1;
		goto err_set_input_dev;
	}

	if (tamper_flag==0) {
		debug_mask |= BIT(3);
		pr_info("Debug level=0x%08X\n", debug_mask);
	}

	retval = synaptics_rmi4_set_input_dev(rmi4_data);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to set up input device\n",
				__func__);
		goto err_set_input_dev;
	}
/*
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	if (strcmp(htc_get_bootmode(), "recovery") == 0) {
		pr_info("Recovery mode. Disable touch\n");
		retval = -ENODEV;
		goto err_off_mode;
	}

	if (strcmp(htc_get_bootmode(), "offmode_charging") == 0) {
		pr_info("Offmode charging. Disable touch interrupts\n");
		offmode_charging_flag = 1;
	}
#else
	if ((strcmp(htc_get_bootmode(), "offmode_charging") == 0)
		|| (strcmp(htc_get_bootmode(), "recovery") == 0)) {
		pr_info("%s mode. Set touch chip to sleep mode and skip touch driver probe\n", htc_get_bootmode());
		synaptics_rmi4_sensor_sleep(rmi4_data);
		retval = -ENODEV;
		goto err_off_mode;
	}
#endif
*/
	if (bdata->support_cover) {
		rmi4_data->cover_mode = 0;
		if (bdata->config_table->cover_setting_size != 0) {
			dev_info(&pdev->dev,"%s cover_setting_size = %d\n",
				__func__, bdata->config_table->cover_setting_size);
			memcpy(rmi4_data->cover_setting, bdata->config_table->cover_setting,
				sizeof(rmi4_data->cover_setting));
			memcpy(rmi4_data->uncover_setting, bdata->config_table->uncover_setting,
				sizeof(rmi4_data->uncover_setting));
			if (bdata->support_glove)
				memcpy(rmi4_data->glove_mode_setting, bdata->config_table->glove_mode_setting,
					sizeof(rmi4_data->glove_mode_setting));
			rmi4_data->cover_setting_size = bdata->config_table->cover_setting_size;
		}
	}

	if (bdata->support_glove) {
		synaptics_rmi4_set_status(rmi4_data, 0);
		rmi4_data->glove_enable = 0;
		rmi4_data->glove_setting = 0;
		dev_info(&pdev->dev,"%s glove mode default off\n", __func__);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
	rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
	register_early_suspend(&rmi4_data->early_suspend);
#endif
#ifdef CONFIG_FB
	rmi4_data->fb_notifier.notifier_call = fb_notifier_callback;
	fb_register_client(&rmi4_data->fb_notifier);
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
	retval = register_notifier_by_facedown(&facedown_status_handler);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to register facedown notifier\n",
				__func__);
		goto err_register_notifier;
	}
#endif
	rmi4_data->face_down = 0;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

#ifdef MTK_PLATFORM
	/* Create singlethread workqueue */
	rmi4_data->wq = create_singlethread_workqueue("synaptics_rmi4_wq");
	if (rmi4_data->wq == NULL) {
		pr_err("Not able to create workqueue\n");
		retval = -ENOMEM;
		goto err_device_init_wq;
	}
	INIT_WORK(&rmi4_data->work_irq, synaptics_rmi4_irq_wq);
#endif

	rmi4_data->irq = gpio_to_irq(bdata->irq_gpio);

#ifdef MTK_PLATFORM
	retval = irq_registration(&rmi4_data->irq,
			synaptics_rmi4_irq, bdata->irq_flags,
			PLATFORM_DRIVER_NAME, rmi4_data);
#else
	retval = request_threaded_irq(rmi4_data->irq, NULL,
			synaptics_rmi4_irq, bdata->irq_flags,
			PLATFORM_DRIVER_NAME, rmi4_data);
#endif
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create irq thread\n",
				__func__);
		goto err_request_irq;
	}

	rmi4_data->irq_enabled = true;

	if (rmi4_data->enable_wakeup_gesture)
		irq_set_irq_wake(rmi4_data->irq, 1);

	if(support_htc_event_flag == 1) {
		rmi4_data->support_htc_event = 1;
		pr_info("support_htc_event = %d\n", rmi4_data->support_htc_event);
	}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	if (offmode_charging_flag == 1) {
		disable_irq(rmi4_data->irq);
		rmi4_data->irq_enabled = false;
	} else {
#endif
		retval = synaptics_rmi4_irq_enable(rmi4_data, true, false);
		if (retval < 0) {
			dev_err(&pdev->dev,
					"%s: Failed to enable attention interrupt\n",
					__func__);
			goto err_enable_irq;
		}

		dev_info(&pdev->dev,"%s: enable irq done\n", __func__);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	}
#endif

	retval = synaptics_rmi4_sysfs_init(rmi4_data, true);
	if (retval < 0) {
		dev_err(&pdev->dev,
				"%s: Failed to create sysfs\n", __func__);
		goto err_sysfs_init;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&pdev->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			goto err_sysfs;
		}
	}

	init_waitqueue_head(&syn_data_ready_wq);
	exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
	exp_data.rmi4_data = rmi4_data;
	exp_data.queue_work = true;
	queue_delayed_work(exp_data.workqueue,
			&exp_data.work,
			0);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
//	incell_driver_ready(synaptics_rmi4_touch_enable);
	incell_driver_ready(notify_from_disp_cont_splash);
	pr_info("Register to display driver\n");
#endif

#ifdef CONFIG_SYNC_TOUCH_STATUS
	touch_solution(1);
	pr_info("Register to sensor_hub driver\n");
#endif

#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_register_notifier(&hallsensor_status_handler);
#endif
	pr_info(" %s: Done\n", __func__);
	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

err_sysfs_init:
	synaptics_rmi4_sysfs_init(rmi4_data, false);
	synaptics_rmi4_irq_enable(rmi4_data, false, false);
	free_irq(rmi4_data->irq, rmi4_data);

err_enable_irq:
	free_irq(rmi4_data->irq, rmi4_data);
err_request_irq:
#ifdef MTK_PLATFORM
	flush_workqueue(rmi4_data->wq);
	destroy_workqueue(rmi4_data->wq);
err_device_init_wq:
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif
#ifdef CONFIG_FB
	fb_unregister_client(&rmi4_data->fb_notifier);
#endif
/*
#ifndef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
err_off_mode:
#endif
*/
	synaptics_rmi4_empty_fn_list(rmi4_data);
	if (rmi4_data->temp_report_data != NULL)
		kfree(rmi4_data->temp_report_data);
	if (rmi4_data->report_data != NULL)
		kfree(rmi4_data->report_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
err_register_notifier:
#endif
err_set_input_dev:
	synaptics_rmi4_gpio_setup(bdata->irq_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->reset_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio, false, 0, 0);

	if (bdata->power_gpio_1v8 >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio_1v8, false, 0, 0);

	if (bdata->switch_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->switch_gpio, false, 0, 0);

err_ui_hw_init:
err_set_gpio:
#ifndef MTK_PLATFORM
	synaptics_dsx_pinctrl_deinit(rmi4_data);
err_config_gpio:
#endif
	synaptics_rmi4_enable_reg(rmi4_data, false);
err_enable_reg:
	synaptics_rmi4_get_reg(rmi4_data, false);

err_get_reg:
	kfree(rmi4_data);

	return retval;
}

static int synaptics_rmi4_remove(struct platform_device *pdev)
{
	int attr_count;
	struct synaptics_rmi4_data *rmi4_data = platform_get_drvdata(pdev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_unregister_notifier(&hallsensor_status_handler);
#endif

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	synaptics_rmi4_sysfs_init(rmi4_data, false);
	synaptics_rmi4_irq_enable(rmi4_data, false, false);
	free_irq(rmi4_data->irq, rmi4_data);
#ifdef MTK_PLATFORM
	flush_workqueue(rmi4_data->wq);
	destroy_workqueue(rmi4_data->wq);
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
	unregister_notifier_by_facedown(&facedown_status_handler);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif
#ifdef CONFIG_FB
	fb_unregister_client(&rmi4_data->fb_notifier);
#endif

	if (rmi4_data->temp_report_data != NULL)
		kfree(rmi4_data->temp_report_data);
	if (rmi4_data->report_data != NULL)
		kfree(rmi4_data->report_data);
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

	synaptics_rmi4_gpio_setup(bdata->irq_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->reset_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio, false, 0, 0);

	if (bdata->power_gpio_1v8 >= 0)
		synaptics_rmi4_gpio_setup(bdata->power_gpio_1v8, false, 0, 0);

	if (bdata->switch_gpio >= 0)
		synaptics_rmi4_gpio_setup(bdata->switch_gpio, false, 0, 0);

#ifndef MTK_PLATFORM
	synaptics_dsx_pinctrl_deinit(rmi4_data);
#endif
	synaptics_rmi4_enable_reg(rmi4_data, false);
	synaptics_rmi4_get_reg(rmi4_data, false);

	kfree(rmi4_data);

	return 0;
}

#ifdef CONFIG_PM
static void synaptics_rmi4_f11_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char reporting_control;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F11)
			break;
	}

	if (fhandler->fn_number != SYNAPTICS_RMI4_F11) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: No F11 function\n",
				__func__);
		return;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	reporting_control = (reporting_control & ~MASK_3BIT);
	if (enable)
		reporting_control |= F11_WAKEUP_GESTURE_MODE;
	else
		reporting_control |= F11_CONTINUOUS_MODE;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base,
			&reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	return;
}

static void synaptics_rmi4_f12_wg(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval;
	unsigned char offset;
	unsigned char reporting_control[3];
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	unsigned char device_ctrl;
#endif
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	pr_info(" %s\n", __func__);
	rmi = &(rmi4_data->rmi4_mod_info);

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F12)
			break;
	}

	if (fhandler->fn_number != SYNAPTICS_RMI4_F12) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: No F12 function\n",
				__func__);
		return;
	}

	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	offset = extra_data->ctrl20_offset;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fhandler->full_addr.ctrl_base + offset,
			reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

	if (enable)
		reporting_control[2] = F12_WAKEUP_GESTURE_MODE;
	else
		reporting_control[2] = F12_CONTINUOUS_MODE;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fhandler->full_addr.ctrl_base + offset,
			reporting_control,
			sizeof(reporting_control));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to change reporting mode\n",
				__func__);
		return;
	}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	if (enable) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f01_ctrl_base_addr,
				&device_ctrl,
				sizeof(device_ctrl));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to set sleep mode\n",
					__func__);
			return;
		}

		device_ctrl = (device_ctrl & ~MASK_3BIT);
		device_ctrl = (device_ctrl | CONFIGURED | NO_SLEEP_ON);

		retval = synaptics_rmi4_reg_write(rmi4_data,
				rmi4_data->f01_ctrl_base_addr,
				&device_ctrl,
				sizeof(device_ctrl));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to set sleep mode\n",
					__func__);
			return;
		}
	}
#endif

	return;
}

static void synaptics_rmi4_wakeup_gesture(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	dev_dbg(rmi4_data->pdev->dev.parent, " %s:%d\n", __func__, enable);
	if (rmi4_data->f11_wakeup_gesture)
		synaptics_rmi4_f11_wg(rmi4_data, enable);
	else if (rmi4_data->f12_wakeup_gesture)
		synaptics_rmi4_f12_wg(rmi4_data, enable);

	return;
}

static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enter sleep mode\n",
				__func__);
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	device_ctrl = (device_ctrl | CONFIGURED);
#endif
	device_ctrl = (device_ctrl | SENSOR_SLEEP);
	dev_info(rmi4_data->pdev->dev.parent, " %s: %x\n", __func__, device_ctrl);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enter sleep mode\n",
				__func__);
		return;
	}

	rmi4_data->sensor_sleep = true;

	return;
}

static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to wake from sleep mode\n",
				__func__);
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	device_ctrl = (device_ctrl | CONFIGURED | NO_SLEEP_ON);
#endif
	dev_info(rmi4_data->pdev->dev.parent, " %s: %x\n", __func__, device_ctrl);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to wake from sleep mode\n",
				__func__);
		return;
	}

	rmi4_data->sensor_sleep = false;

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data, early_suspend);

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	if (rmi4_data->stay_awake)
		return;

	if (rmi4_data->enable_wakeup_gesture && !rmi4_data->face_down) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: gesture mode\n", __func__);
		synaptics_rmi4_wakeup_gesture(rmi4_data, true);
		goto exit;
	}

	if (!rmi4_data->suspend) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: sleep mode\n", __func__);
		synaptics_rmi4_irq_enable(rmi4_data, false, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->early_suspend != NULL)
				exp_fhandler->exp_fn->early_suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));

#ifdef CONFIG_SYNC_TOUCH_STATUS
	switch_sensor_hub(rmi4_data, 1);
#endif
exit:
	rmi4_data->suspend = true;

	return;
}

static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data, early_suspend);

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	if (rmi4_data->stay_awake)
		return;

	if (rmi4_data->enable_wakeup_gesture && !rmi4_data->face_down) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: wake up from gesture mode\n", __func__);
		synaptics_rmi4_wakeup_gesture(rmi4_data, false);
		synaptics_rmi4_force_cal(rmi4_data);
		goto exit;
	}

#ifdef CONFIG_SYNC_TOUCH_STATUS
	switch_sensor_hub(rmi4_data, 0);
#endif

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));

	if (rmi4_data->suspend) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: wake up\n", __func__);
		synaptics_rmi4_free_fingers(rmi4_data);
#if 1		/* Enable HW reset fot all projects */
		synaptics_rmi4_irq_enable(rmi4_data, true, false);
		synaptics_rmi4_hw_reset_device(rmi4_data);
		rmi4_data->sensor_sleep = false;
#else
		synaptics_rmi4_sensor_wake(rmi4_data);
	#ifdef CONFIG_SYNC_TOUCH_STATUS
		synaptics_rmi4_force_cal(rmi4_data);
	#endif
		synaptics_rmi4_irq_enable(rmi4_data, true, false);
		synaptics_rmi4_set_chip_mode(rmi4_data);
#endif
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->late_resume != NULL)
				exp_fhandler->exp_fn->late_resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

exit:
	rmi4_data->suspend = false;
	rmi4_data->face_down = 0;

	return;
}
#endif

#ifdef CONFIG_FB
static void synaptics_rmi4_early_suspend(struct device *dev)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	if (rmi4_data->stay_awake)
		return;

	if (rmi4_data->enable_wakeup_gesture && !rmi4_data->face_down) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: gesture mode\n", __func__);
		synaptics_rmi4_wakeup_gesture(rmi4_data, true);
		goto exit;
	}

	if (!rmi4_data->suspend) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: sleep mode\n", __func__);
		synaptics_rmi4_irq_enable(rmi4_data, false, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->early_suspend != NULL)
				exp_fhandler->exp_fn->early_suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));

exit:
	rmi4_data->suspend = true;

	return;
}

static void synaptics_rmi4_late_resume(struct device *dev)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	if (rmi4_data->stay_awake)
		return;

	if (rmi4_data->enable_wakeup_gesture && !rmi4_data->face_down) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: wake up from gesture mode\n", __func__);
		synaptics_rmi4_wakeup_gesture(rmi4_data, false);
		synaptics_rmi4_force_cal(rmi4_data);
		goto exit;
	}

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));

	if (rmi4_data->suspend) {
		dev_info(rmi4_data->pdev->dev.parent, " %s: wake up\n", __func__);
		synaptics_rmi4_free_fingers(rmi4_data);
#if 1		/* Enable HW reset fot all projects */
		synaptics_rmi4_irq_enable(rmi4_data, true, false);
		synaptics_rmi4_hw_reset_device(rmi4_data);
		rmi4_data->sensor_sleep = false;
#else
		synaptics_rmi4_sensor_wake(rmi4_data);
	#ifdef CONFIG_SYNC_TOUCH_STATUS
		synaptics_rmi4_force_cal(rmi4_data);
	#endif
		synaptics_rmi4_irq_enable(rmi4_data, true, false);
		synaptics_rmi4_set_chip_mode(rmi4_data);
#endif
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->late_resume != NULL)
				exp_fhandler->exp_fn->late_resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

exit:
	rmi4_data->suspend = false;
	rmi4_data->face_down = 0;

	return;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_WAKEUP_GESTURE
static int facedown_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	dev_dbg(rmi4_data->pdev->dev.parent, " %s: status: %lu\n", __func__, status);

	if (rmi4_data->stay_awake || (!rmi4_data->suspend))
		return 0;
	if (rmi4_data->face_down == status)
		return 0;

	if (!status) {
		dev_dbg(rmi4_data->pdev->dev.parent, " %s: gesture mode\n", __func__);
		if (rmi4_data->enable_wakeup_gesture) {
			synaptics_rmi4_sensor_wake(rmi4_data);
			synaptics_rmi4_irq_enable(rmi4_data, true, false);
			synaptics_rmi4_wakeup_gesture(rmi4_data, true);
		}
		rmi4_data->face_down = 0;
	}
	else {
		dev_dbg(rmi4_data->pdev->dev.parent, " %s: sleep mode\n", __func__);
		if (rmi4_data->enable_wakeup_gesture)
			synaptics_rmi4_wakeup_gesture(rmi4_data, false);
		synaptics_rmi4_irq_enable(rmi4_data, false, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
		rmi4_data->face_down = 1;
	}
	return 0;
}
#endif

static int synaptics_rmi4_suspend(struct device *dev)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	if (rmi4_data->stay_awake)
		return 0;

	if (rmi4_data->enable_wakeup_gesture && !rmi4_data->face_down) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
		if (rmi4_data->irq_enabled && !offmode_charging_flag) {
#else
		if (rmi4_data->irq_enabled) {
#endif
			dev_dbg(rmi4_data->pdev->dev.parent, " %s, irq disabled\n", __func__);
			disable_irq(rmi4_data->irq);
			rmi4_data->irq_enabled = false;
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->suspend != NULL)
				exp_fhandler->exp_fn->suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	if (rmi4_data->pwr_reg)
		regulator_disable(rmi4_data->pwr_reg);

	return 0;
}

static int synaptics_rmi4_resume(struct device *dev)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;

	dev_info(rmi4_data->pdev->dev.parent, " %s\n", __func__);
	if (rmi4_data->stay_awake)
		return 0;

	if (rmi4_data->pwr_reg) {
		retval = regulator_enable(rmi4_data->pwr_reg);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable power regulator\n",
					__func__);
		}
		msleep(bdata->power_delay_ms);
		rmi4_data->current_page = MASK_8BIT;
		if (rmi4_data->hw_if->ui_hw_init)
			rmi4_data->hw_if->ui_hw_init(rmi4_data);
	}

	if (rmi4_data->enable_wakeup_gesture && !rmi4_data->face_down) {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
		if (!rmi4_data->irq_enabled && !offmode_charging_flag) {
#else
		if (rmi4_data->irq_enabled) {
#endif
			dev_dbg(rmi4_data->pdev->dev.parent, " %s, irq enabled\n", __func__);
			enable_irq(rmi4_data->irq);
			rmi4_data->irq_enabled = true;
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->resume != NULL)
				exp_fhandler->exp_fn->resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	return 0;
}

#ifdef CONFIG_SYNC_TOUCH_STATUS
static void switch_sensor_hub(struct synaptics_rmi4_data *rmi4_data, int mode)
{
	const struct synaptics_dsx_board_data *bdata =
				rmi4_data->hw_if->board_data;
	int mask = 0;

	pr_info("%s: %d\n", __func__, mode);
	if (bdata->support_glove){
		mask = (rmi4_data->glove_setting & 0x01) << 1;
	}

	if (gpio_is_valid(bdata->switch_gpio)) {
		switch(mode) {
		case 0:
			touch_status(0 | mask);
#ifdef MTK_PLATFORM
			gpio_set_value(bdata->switch_gpio, 0);
#else
			gpio_direction_output(bdata->switch_gpio, 0);
#endif
			rmi4_data->i2c_to_mcu = 0;
			pr_info("[SensorHub] Switch touch i2c to CPU\n");
			break;
		case 1:
#ifdef MTK_PLATFORM
			gpio_set_value(bdata->switch_gpio, 1);
#else
			gpio_direction_output(bdata->switch_gpio, 1);
#endif
			rmi4_data->i2c_to_mcu = 1;
			pr_info("[SensorHub] Switch touch i2c to MCU\n");
			touch_status(1 | mask);
			break;
		}
	}
}
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
/*
void synaptics_rmi4_touch_enable(int enable)
{
	//struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	pr_info("%s:%d\n", __func__, enable);
	if (enable == 0) {
		//synaptics_rmi4_early_suspend((&rmi4_data->input_dev->dev));
	}
	else if (enable == 1) {
		//synaptics_rmi4_late_resume(&(rmi4_data->input_dev->dev));
	}
}
*/
void notify_from_disp_cont_splash(int enable)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;
	pr_info("%s:%d\n", __func__, enable);
	rmi4_data->cont_splash_enable =  enable;
}
#endif

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
					 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	static int firstBoot = 2;
	struct synaptics_rmi4_data *rmi4_data =
		container_of(self, struct synaptics_rmi4_data, fb_notifier);

	dev_info(rmi4_data->pdev->dev.parent, "%s, event = %ld\n", __func__, event);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SUPPORT_INCELL
	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK && (!rmi4_data->cont_splash_enable || !firstBoot)) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
	#ifdef CONFIG_SYNC_TOUCH_STATUS
			switch_sensor_hub(rmi4_data, 0);
			synaptics_rmi4_f12_wg(rmi4_data, 0);
	#endif
			synaptics_rmi4_querry_f51_data(rmi4_data);
			synaptics_rmi4_late_resume(&(rmi4_data->input_dev->dev));
			break;
		case FB_BLANK_POWERDOWN:
			synaptics_rmi4_querry_f51_data(rmi4_data);
			synaptics_rmi4_early_suspend((&rmi4_data->input_dev->dev));
			break;
		}
	}

	if (evdata && evdata->data && event == FB_EVENT_BLANK && (!rmi4_data->cont_splash_enable || !firstBoot)) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_POWERDOWN:
	#ifdef CONFIG_SYNC_TOUCH_STATUS
			synaptics_rmi4_querry_f51_data(rmi4_data);
			//synaptics_rmi4_f12_wg(rmi4_data, 1);
			switch_sensor_hub(rmi4_data, 1);
	#endif
			break;
		}
	}

	if (firstBoot > 0)
		firstBoot--;
	else
		firstBoot = 0;
#else
	if (firstBoot > 0) {
		pr_info("%s: ignore first boot:%d\n", __func__, firstBoot);
		firstBoot--;
		return 0;
	} else
		firstBoot = 0;

	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
	#ifdef CONFIG_SYNC_TOUCH_STATUS
			switch_sensor_hub(rmi4_data, 0);
	#endif
			synaptics_rmi4_late_resume(&(rmi4_data->input_dev->dev));
			break;
		case FB_BLANK_POWERDOWN:
			synaptics_rmi4_early_suspend((&rmi4_data->input_dev->dev));
	#ifdef CONFIG_SYNC_TOUCH_STATUS
			switch_sensor_hub(rmi4_data, 1);
	#endif
			break;
		}
	}
#endif

	return 0;
}
#endif

static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
	.suspend = synaptics_rmi4_suspend,
	.resume  = synaptics_rmi4_resume,
};
#endif

static struct platform_driver synaptics_rmi4_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &synaptics_rmi4_dev_pm_ops,
#endif
	},
	.probe = synaptics_rmi4_probe,
	.remove = synaptics_rmi4_remove,
};

static int __init synaptics_rmi4_init(void)
{
	int retval;

	retval = synaptics_rmi4_bus_init();
	if (retval)
		return retval;

	return platform_driver_register(&synaptics_rmi4_driver);
}

static void __exit synaptics_rmi4_exit(void)
{
	platform_driver_unregister(&synaptics_rmi4_driver);

	synaptics_rmi4_bus_exit();

	return;
}

late_initcall(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

static int __init get_tamper_flag(char *str)
{
	int ret = kstrtouint(str, 0, &tamper_flag);
	pr_info(" %d: %d from %s",
			ret, tamper_flag, str);
	return ret;
} early_param("td.sf", get_tamper_flag);

static int __init get_htc_event_support_flag(char *str)
{
	int ret = kstrtouint(str, 0, &support_htc_event_flag);
	pr_info("androidtouch.htc_event %d: %d from %s",
			ret, support_htc_event_flag, str);
	return ret;
} early_param("androidtouch.htc_event", get_htc_event_support_flag);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX Touch Driver");
MODULE_LICENSE("GPL v2");
