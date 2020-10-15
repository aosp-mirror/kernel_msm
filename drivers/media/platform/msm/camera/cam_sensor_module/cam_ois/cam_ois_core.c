/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "../cam_fw_update/fw_update.h"

static struct cam_ois_timer_t ois_timer;
DEFINE_MSM_MUTEX(ois_shift_mutex);

int cam_ois_calibration(struct cam_ois_ctrl_t *o_ctrl,
	stReCalib *cal_result)
{
	int rc;

	rc = GyroReCalib(&o_ctrl->io_master_info, cal_result);
	if (rc != 0)
		CAM_ERR(CAM_OIS,
			"[OISCali] ReCalib FAIL, rc = %d", rc);
	else {
		rc = WrGyroOffsetData();
		if (rc != 0)
			CAM_ERR(CAM_OIS,
				"[OISCali] WrGyro FAIL, rc = %d", rc);
		else
			CAM_INFO(CAM_OIS,
				"[OISCali] SUCCESS");
	}

	return rc;
}

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 2;
	power_info->power_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting) *
			power_info->power_setting_size,
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VIO;
	power_info->power_setting[0].seq_val = CAM_VIO;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 0;
	power_info->power_setting[1].seq_type = SENSOR_VAF;
	power_info->power_setting[1].seq_val = CAM_VAF;
	power_info->power_setting[1].config_val = 1;
	power_info->power_setting[1].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting) *
			power_info->power_down_setting_size,
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, (void __user *) cmd->handle,
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user((void __user *) cmd->handle, &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc)
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = msm_camera_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 1;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)(
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	page = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)(
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	release_firmware(fw);

	return rc;
}

/**
 * cam_ois_shift_data_enqueue - enqueue shift data to ring buffer
 * @time_readout:		ctrl structure
 * @shift_x:            shift in x
 * @shift_y:            shift in y
 * @buffer:             rint buffer
 *
 * Returns success or failure
 */
static int cam_ois_shift_data_enqueue(int64_t time_readout, int16_t shift_x,
	int16_t shift_y, struct cam_ois_shift_buffer *buffer)
{
	int rc = 0;

	mutex_lock(&ois_shift_mutex);
	if (buffer->write_pos >= CAM_OIS_SHIFT_DATA_BUFFER_SIZE ||
		buffer->write_pos < 0) {
		CAM_ERR(CAM_OIS,
			"invalid OIS shift buffer index: %d",
			buffer->write_pos);
		rc = -EFAULT;
	} else {
		struct cam_ois_shift *pb = &buffer->buffer[buffer->write_pos];

		pb->time_readout = time_readout;
		pb->ois_shift_x = shift_x;
		pb->ois_shift_y = shift_y;
		buffer->write_pos++;
		if (buffer->write_pos == CAM_OIS_SHIFT_DATA_BUFFER_SIZE) {
			buffer->write_pos = 0;
			buffer->is_full = true;
		}
		rc = 0;
	}
	mutex_unlock(&ois_shift_mutex);
	return rc;
}

/**
 * cam_ois_read_work - worker function of read timer
 * @work:       work
 *
 * Returns success or failure
 */
static void cam_ois_read_work(struct work_struct *work)
{
	uint8_t buf[8] = { 0 };
	int32_t rc = 0;
	int16_t shift_x, shift_y;
#ifdef CONFIG_BOARD_BONITO
	uint16_t raw_x, raw_y, raw_z;
#endif
	struct timespec ts;
	int64_t time_readout;
	struct cam_ois_timer_t *ois_timer_in;

	ois_timer_in = container_of(work, struct cam_ois_timer_t, g_work);
	get_monotonic_boottime(&ts);
	rc = camera_io_dev_read_seq(&ois_timer_in->o_ctrl->io_master_info,
		0xE001, &buf[0], CAMERA_SENSOR_I2C_TYPE_WORD, 6);

	if (rc != 0) {
		ois_timer.i2c_fail_count++;
		CAM_ERR(CAM_OIS,
			"read seq fail. cnt = %d", ++ois_timer.i2c_fail_count);
		if (ois_timer.i2c_fail_count >= MAX_FAIL_CNT) {
			CAM_ERR(CAM_OIS, "Too many i2c failed. Stop timer.");
			ois_timer.ois_timer_state = CAM_OIS_TIME_ERROR;
		}
	} else {
		ois_timer.i2c_fail_count = 0;
		time_readout = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
		shift_x = (int16_t)(((uint16_t)buf[0] << 8) + (uint16_t)buf[1]);
		shift_y = (int16_t)(((uint16_t)buf[2] << 8) + (uint16_t)buf[3]);

		rc = cam_ois_shift_data_enqueue(time_readout, shift_x,
			shift_y, &ois_timer_in->o_ctrl->buf);
		if (rc != 0)
			CAM_ERR(CAM_OIS, "OIS shift data enqueue failed");
	}
#ifdef CONFIG_BOARD_BONITO
	if (ois_timer.ois_factory_read & (1 << 0)) {
		rc = camera_io_dev_read_seq(
			&ois_timer_in->o_ctrl->io_master_info, 0xE003,
			&buf[0], CAMERA_SENSOR_I2C_TYPE_WORD, 8);
		if (rc != 0)
			CAM_ERR(CAM_OIS, "0xE003, read seq fail.");
		else {
			raw_x =	(((uint16_t)buf[0] << 8) + (uint16_t)buf[1]);
			raw_y =	(((uint16_t)buf[2] << 8) + (uint16_t)buf[3]);
			raw_z =	(((uint16_t)buf[4] << 8) + (uint16_t)buf[5]);
			CAM_INFO(CAM_OIS,
				"Readout 0xE003 [x:0x%04X, y:0x%04X, z:0x%04X]",
				raw_x, raw_y, raw_z);
		}
	}

	if (ois_timer.ois_factory_read & (1 << 1)) {
		rc = camera_io_dev_read_seq(
			&ois_timer_in->o_ctrl->io_master_info, 0xE005,
			&buf[0], CAMERA_SENSOR_I2C_TYPE_WORD, 8);
		if (rc != 0)
			CAM_ERR(CAM_OIS, "0xE005, read seq fail.");
		else {
			raw_x =	(((uint16_t)buf[0] << 8) + (uint16_t)buf[1]);
			raw_y =	(((uint16_t)buf[2] << 8) + (uint16_t)buf[3]);
			raw_z =	(((uint16_t)buf[4] << 8) + (uint16_t)buf[5]);
			CAM_INFO(CAM_OIS,
				"Readout 0xE005 [x:0x%04X, y:0x%04X, z:0x%04X]",
				raw_x, raw_y, raw_z);
		}
	}
#endif
}

/**
 * cam_ois_shift_timer - ois shift reader timer expiracy callback func
 * @timer:			pointer to the hrtimer struct
 *
 * Returns hrtimer_restart state
 */
static enum hrtimer_restart cam_ois_shift_timer(struct hrtimer *timer)
{
	ktime_t currtime, interval;
	struct cam_ois_timer_t *ois_timer_in;

	ois_timer_in = container_of(timer, struct cam_ois_timer_t, hr_timer);
	if (ois_timer.ois_timer_state != CAM_OIS_TIME_ERROR) {
		queue_work(ois_timer_in->ois_wq, &ois_timer_in->g_work);
		currtime = ktime_get();
		interval = ktime_set(0, READ_OUT_TIME);
		hrtimer_forward(timer, currtime, interval);

		return HRTIMER_RESTART;
	}

	CAM_ERR(CAM_OIS, "HRTIMER_NORESTART");
	return HRTIMER_NORESTART;
}

/**
 * cam_ois_stop_offset_reader_thread - stop shift reader
 *
 * Returns success or failure
 */
static int cam_ois_stop_shift_reader(void)
{
	if (ois_timer.ois_timer_state == CAM_OIS_TIME_ACTIVE ||
		ois_timer.ois_timer_state == CAM_OIS_TIME_ERROR) {
		hrtimer_cancel(&ois_timer.hr_timer);
		destroy_workqueue(ois_timer.ois_wq);
		ois_timer.ois_timer_state = CAM_OIS_TIME_INACTIVE;
		CAM_INFO(CAM_OIS, "Successfully stopped OIS shift reader.");
	} else {
		CAM_ERR(CAM_OIS,
			"Invalid ois timer state:%d",
			ois_timer.ois_timer_state);
	}
	return 0;
}

/**
 * cam_ois_start_shift_reader_thread - start shift reader
 * @o_ctrl:			ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_start_shift_reader(struct cam_ois_ctrl_t *o_ctrl)
{
	ktime_t ktime;
	int rc = 0;

	if (ois_timer.ois_timer_state == CAM_OIS_TIME_ERROR) {
		CAM_ERR(CAM_OIS, "OIS Timer Error.");
		cam_ois_stop_shift_reader();
		return -EFAULT;
	}
	ois_timer.i2c_fail_count = 0;
	if (ois_timer.ois_timer_state != CAM_OIS_TIME_ACTIVE) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			I2C_FAST_PLUS_MODE;
		ois_timer.o_ctrl = o_ctrl;

		// set worker function and work queue
		INIT_WORK(&ois_timer.g_work, cam_ois_read_work);
		ois_timer.ois_wq = alloc_workqueue("ois_wq", WQ_HIGHPRI, 1);
		if (!ois_timer.ois_wq) {
			CAM_ERR(CAM_OIS, "ois_wq create failed.");
			return -EFAULT;
		}

		// set timer
		ktime = ktime_set(0, READ_OUT_TIME);
		hrtimer_init(&ois_timer.hr_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
		ois_timer.hr_timer.function = &cam_ois_shift_timer;
		hrtimer_start(&ois_timer.hr_timer, ktime,
			HRTIMER_MODE_REL);
		ois_timer.ois_timer_state = CAM_OIS_TIME_ACTIVE;
	} else {
		CAM_ERR(CAM_OIS,
			"invalid timer state = %d", ois_timer.ois_timer_state);
		return -EFAULT;
	}

	mutex_lock(&ois_shift_mutex);
	o_ctrl->buf.write_pos = 0;
	o_ctrl->buf.is_full = false;
	mutex_unlock(&ois_shift_mutex);

	CAM_INFO(CAM_OIS, "Successfully started OIS shift reader.");
	return rc;
}

/**
 * cam_ois_get_shift - write ois shift data to user space
 * @o_ctrl:             ctrl structure
 * @query_size_handle:  handle of query_size in user space
 * @shift_data_handle:  handle of shift_data in user space
 *
 * Returns success or failure
 */
static int cam_ois_get_shift(struct cam_ois_ctrl_t *o_ctrl,
	uint64_t query_size_handle, uint64_t shift_data_handle)
{
	int rc = 0;
	struct cam_ois_shift buf[CAM_OIS_SHIFT_DATA_BUFFER_SIZE];
	uint8_t query_size = 0;
	int32_t read_pos = 0;
	int32_t write_pos = 0;

	// copy from ring buffer to local continuous buffer
	mutex_lock(&ois_shift_mutex);
	write_pos = o_ctrl->buf.write_pos;
	if (o_ctrl->buf.is_full) {
		read_pos = write_pos;
		do {
			buf[query_size++] = o_ctrl->buf.buffer[read_pos++];
			if (read_pos == CAM_OIS_SHIFT_DATA_BUFFER_SIZE)
				read_pos = 0;
		} while (read_pos != write_pos);
	} else {
		read_pos = 0;
		while (read_pos != write_pos)
			buf[query_size++] = o_ctrl->buf.buffer[read_pos++];
	}
	// reset ring buffer
	o_ctrl->buf.write_pos = 0;
	o_ctrl->buf.is_full = false;
	mutex_unlock(&ois_shift_mutex);

	// copy to user space
	if (copy_to_user((void __user *) query_size_handle, &query_size,
		sizeof(uint8_t))) {
		CAM_ERR(CAM_OIS, "ois_get_shift: copy to user failed!");
		return -EFAULT;
	}
	if (copy_to_user((void __user *) shift_data_handle, buf,
		sizeof(struct cam_ois_shift) * query_size)) {
		CAM_ERR(CAM_OIS, "ois_get_shift: copy to user failed!");
		return -EFAULT;
	}

	return rc;
}

/**
 * cam_ois_read_reg - read register data and copy to user space
 * @o_ctrl:           ctrl structure
 * @cmd_get_ois:      handle of shift_data in user space
 *
 * Returns success or failure
 */
static int cam_ois_read_reg(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_cmd_get_ois_data *cmd_get_ois)
{
	uint8_t buf[8] = { 0 };
	int32_t rc = 0;
	uint32_t addr = cmd_get_ois->reg_addr;
	int32_t num_bytes = cmd_get_ois->reg_data;

	if (addr <= 0 || addr > 0xFFFF) {
		CAM_ERR(CAM_OIS,
			"Invalid addr while read OIS data: %x", addr);
		return -EINVAL;
	}

	if (num_bytes <= 0 || num_bytes > 8) {
		CAM_ERR(CAM_OIS,
			"Invalid read size while read OIS data: %d", num_bytes);
		return -EINVAL;
	}

	rc = camera_io_dev_read_seq(&o_ctrl->io_master_info,
		addr, buf, CAMERA_SENSOR_I2C_TYPE_WORD, num_bytes);

	if (rc) {
		CAM_ERR(CAM_OIS, "camera_io_dev_read_seq failed!");
		return rc;
	}

	if (copy_to_user((void __user *) cmd_get_ois->query_data_handle,
		buf, sizeof(uint8_t) * num_bytes)) {
		CAM_ERR(CAM_OIS, "ois_read_reg: copy to user failed!");
	}

	return rc;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uint64_t                        generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uint64_t                        generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;
	struct cam_cmd_get_ois_data     *cmd_get_ois = NULL;
	int32_t                         *cal_rc;
	stReCalib                       *cal_result;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config, (void __user *) ioctl_ctrl->handle,
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		(uint64_t *)&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"offset is out of bound: off: %lld len: %zu",
			dev_config.offset, pkt_len);
		return -EINVAL;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_OIS, "Invalid packet params");
		return -EINVAL;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				(uint64_t *)&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf");
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				return -EINVAL;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				return -EINVAL;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					return rc;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					return rc;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					return rc;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					return rc;
				}
			}
			break;
			}
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				return rc;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply Init settings");
			goto pwr_dwn;
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0)
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
		break;
	case CAM_OIS_PACKET_OPCODE_SHIFT_READER_START:
#ifdef CONFIG_BOARD_BONITO
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_mem_get_cpu_buf(cmd_desc[0].mem_handle,
			(uint64_t *)&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed to get cpu buf");
			return rc;
		}
		cmd_buf = (uint32_t *)generic_ptr;
		if (!cmd_buf) {
			CAM_ERR(CAM_OIS, "invalid cmd buf");
			return -EINVAL;
		}
		cmd_buf += cmd_desc->offset / sizeof(uint32_t);
		ois_timer.ois_factory_read = *(uint8_t *)cmd_buf;
#endif
		rc = cam_ois_start_shift_reader(o_ctrl);
		break;
	case CAM_OIS_PACKET_OPCODE_SHIFT_READER_STOP:
		rc = cam_ois_stop_shift_reader();
		break;
	case CAM_OIS_PACKET_OPCODE_SHIFT_GET:
	case CAM_OIS_PACKET_OPCODE_READ:
		if (csl_packet->num_cmd_buf != 1) {
			CAM_ERR(CAM_OIS,
				"More than one cmd buf found in shift_get");
			return -EINVAL;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		total_cmd_buf_in_bytes = cmd_desc->length;
		if (!total_cmd_buf_in_bytes) {
			CAM_ERR(CAM_OIS,
				"Empty cmd buf found in shift_get");
			return -EINVAL;
		}

		rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
			(uint64_t *)&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed to get cpu buf");
			return rc;
		}
		cmd_buf = (uint32_t *)generic_ptr;
		if (!cmd_buf) {
			CAM_ERR(CAM_OIS, "invalid cmd buf");
			return -EINVAL;
		}
		cmd_buf += cmd_desc->offset / sizeof(uint32_t);
		cmd_get_ois = (struct cam_cmd_get_ois_data *)cmd_buf;

		if ((csl_packet->header.op_code & 0xFFFFFF) ==
			CAM_OIS_PACKET_OPCODE_SHIFT_GET) {
			rc = cam_ois_get_shift(o_ctrl,
				cmd_get_ois->query_size_handle,
				cmd_get_ois->query_data_handle);
		} else if ((csl_packet->header.op_code & 0xFFFFFF) ==
			CAM_OIS_PACKET_OPCODE_READ) {
			rc = cam_ois_read_reg(o_ctrl, cmd_get_ois);
		}
		break;
	case CAM_OIS_PACKET_OPCODE_CALIBRATION:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_mem_get_cpu_buf(cmd_desc[0].mem_handle,
			(uint64_t *)&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed to get cpu buf");
			return rc;
		}
		cmd_buf = (uint32_t *)generic_ptr;
		if (!cmd_buf) {
			CAM_ERR(CAM_OIS, "invalid cmd buf");
			return -EINVAL;
		}
		cmd_buf += cmd_desc->offset / sizeof(uint32_t);
		cal_rc = (int32_t *)cmd_buf;
		cal_result = (stReCalib *)(cmd_buf + 1);
		*cal_rc = cam_ois_calibration(o_ctrl, cal_result);
		break;
	default:
		break;
	}
	return rc;
pwr_dwn:
	cam_ois_power_down(o_ctrl);
	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user((void __user *) cmd->handle,
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;
		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
