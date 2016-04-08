/*
 * cyttsp5_device_access.c
 * Cypress TrueTouch(TM) Standard Product V5 Device Access Module.
 * Configuration and Test command/status user interface.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2014 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include "cyttsp5_regs.h"

#define CYTTSP5_DEVICE_ACCESS_NAME "cyttsp5_device_access"
#define CYTTSP5_INPUT_ELEM_SZ (sizeof("0xHH") + 1)

#define STATUS_SUCCESS	0
#define STATUS_FAIL	-1
#define PIP_CMD_MAX_LENGTH ((1 << 16) - 1)

extern int cyttsp5_hid_output_write_mfg_val0(struct cyttsp5_core_data *cd,u8 val);
extern int cyttsp5_read_mfg_val0_(struct cyttsp5_core_data *cd);
static struct cyttsp5_core_commands *cmd;
static struct kobject *touch_screen_kobject_ts=NULL;
static struct kobject *cyttsp5_kobject =NULL;
static struct device *cyttsp5_dev_ptr = NULL;

struct cyttsp5_attribute {
	struct attribute attr;

	ssize_t (*show)(struct device *dev, struct cyttsp5_attribute *attr,
			char *buf);
	ssize_t (*store)(struct device *dev, struct cyttsp5_attribute *attr,
			const char *buf, size_t count);
};

#define CY_ATTR(_name, _mode, _show, _store)  \
	struct cyttsp5_attribute cy_attr_##_name = \
		__ATTR(_name, _mode, _show, _store)

static inline int cyttsp5_create_file(struct device *dev,
		const struct cyttsp5_attribute *attr)
{
	struct cyttsp5_device_access_data *dad;
	int error = 0;

	if (dev) {
		dad = cyttsp5_get_device_access_data(dev);
		error = sysfs_create_file(&dad->mfg_test, &attr->attr);
	}

	return error;
}

static inline void cyttsp5_remove_file(struct device *dev,
		const struct cyttsp5_attribute *attr)
{
	struct cyttsp5_device_access_data *dad;

	if (dev) {
		dad = cyttsp5_get_device_access_data(dev);
		sysfs_remove_file(&dad->mfg_test, &attr->attr);
	}
}

static ssize_t cyttsp5_attr_show(struct kobject *kobj, struct attribute *attr,
		char *buf)
{
	struct cyttsp5_attribute *cyttsp5_attr = container_of(attr,
			struct cyttsp5_attribute, attr);
	struct device *dev = container_of(kobj->parent, struct device, kobj);
	ssize_t ret = -EIO;

	if (cyttsp5_attr->show)
		ret = cyttsp5_attr->show(dev, cyttsp5_attr, buf);
	return ret;
}

static ssize_t cyttsp5_attr_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t count)
{
	struct cyttsp5_attribute *cyttsp5_attr = container_of(attr,
			struct cyttsp5_attribute, attr);
	struct device *dev = container_of(kobj->parent, struct device, kobj);
	ssize_t ret = -EIO;

	if (cyttsp5_attr->store)
		ret = cyttsp5_attr->store(dev, cyttsp5_attr, buf, count);
	return ret;
}

static const struct sysfs_ops cyttsp5_sysfs_ops = {
	.show = cyttsp5_attr_show,
	.store = cyttsp5_attr_store,
};

static struct kobj_type cyttsp5_ktype = {
	.sysfs_ops = &cyttsp5_sysfs_ops,
};
/*check TP device info, if device is wrong status,return error code*/
static int cyttsp5_tp_check_device_info(void)
{
	struct device *cdev = cyttsp5_dev_ptr;
	struct cyttsp5_core_data *cd;
	int ret = -EINVAL;

	tp_log_warning("%s: begin \n", __func__);
	if(!cdev){
		tp_log_err("%s: device is null \n", __func__);
		return ret;
	}

	cd = dev_get_drvdata(cdev);
	if(!cd){
		tp_log_err("%s: cyttsp5 core data is null \n", __func__);
		return ret;
	}

	/*read the tp_color value from IC*/
	ret = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (ret < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return ret;
	}
	ret = cyttsp5_read_mfg_val0_(cd);
	if (release_exclusive(cd, cd->dev) < 0)
		/* Don't return fail code, mode is already changed. */
		tp_log_err( "%s: fail to release exclusive\n", __func__);
	else
		tp_log_debug("%s: pass release exclusive\n", __func__);
	if (ret< 0) {
		tp_log_err("%s: driver could not read tp color data\n", __func__);
		return ret;
	}
	return ret;
}
static ssize_t hw_cyttsp5_tp_color_info_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	u8 tp_color_info = 0;
	enum cyttsp5_sleep_state uSleep = SS_SLEEP_OFF;

	struct device *cdev = cyttsp5_dev_ptr;
	struct cyttsp5_core_data *cd;

	int ret;
	tp_log_warning("%s: begin \n", __func__);
	if(0 == tp_color_data){
		/*if tp have color value 0, we will first check if tp is power on, then read mfg_val0*/
		cd = dev_get_drvdata(cdev);
		mutex_lock(&cd->system_lock);
		uSleep = cd->sleep_state;
		mutex_unlock(&cd->system_lock);
		if(uSleep == SS_SLEEP_OFF) {
			/*if color is 0,when every wakeup, ALS sensor will first wake to read color from tp.
			but tp has not power on yet, so the command will timeout. will delay 200ms during wakeup*/
			ret = cyttsp5_tp_check_device_info();
			if(ret < 0){
				tp_log_err("%s: driver check device info fail\n", __func__);
			}
		}
	}

	tp_color_info = tp_color_data;
	if(0 == tp_color_info){
		tp_log_warning("%s: tp color info is invalid \n", __func__);
	}

	switch(tp_color_info)
	{
	case WHITE:
		strncpy(buf, "white", sizeof("white"));
		break;
	case BLACK:
		strncpy(buf, "black", sizeof("black"));
		break;
	case GOLD:
		strncpy(buf, "gold", sizeof("gold"));
		break;
	default:
		strncpy(buf, "fail", sizeof("fail"));
		break;
	}
	tp_log_warning("%s: tp color : %s \n", __func__, buf);

	return strlen(buf);
}
static ssize_t hw_cyttsp5_tp_color_info_store(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct device *cdev = cyttsp5_dev_ptr;
	struct cyttsp5_core_data *cd;
	unsigned long value;
	int ret = -EINVAL;

	tp_log_warning("%s: begin \n", __func__);
	if(!cdev){
		tp_log_err("%s: device is null \n", __func__);
		return ret;
	}

	cd = dev_get_drvdata(cdev);
	if(!cd){
		tp_log_err("%s: cyttsp5 core data is null \n", __func__);
		return ret;
	}

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		tp_log_err("%s %d:Parse input value fail, value: %s, ret:%d\n",
					__func__, __LINE__, buf, ret);
		return -EINVAL;
	}

	ret = cyttsp5_hid_output_write_mfg_val0(cd, value);
	if(ret){
		tp_log_err("%s faied to write tp color value, ret:%d\n", __func__, ret);
		return ret;
	}

	return size;
}
static struct kobj_attribute tp_color_info = {
	.attr = {.name = "tp_color_info", .mode = (S_IWUSR|S_IRUSR|S_IRGRP|S_IROTH)},
	.show = hw_cyttsp5_tp_color_info_show,
	.store = hw_cyttsp5_tp_color_info_store,
};
static ssize_t cyttsp5_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	u8 val;

	mutex_lock(&dad->sysfs_lock);
	val = dad->status;
	mutex_unlock(&dad->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE, "%d\n", val);
}
static DEVICE_ATTR(status, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_status_show, NULL);
static ssize_t hw_cyttsp5_status_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct device *cdev=cyttsp5_dev_ptr;
	if(!cdev)
	{
		tp_log_err("%s: device is null \n", __func__);
		return -EINVAL;
	}
	return cyttsp5_status_show(cdev,NULL,buf);
}
static struct kobj_attribute hw_status_node = {
	.attr = {.name = "status", .mode = (S_IRUSR | S_IRGRP | S_IROTH)},
	.show = hw_cyttsp5_status_show,
	.store = NULL,
};
static ssize_t cyttsp5_response_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int i;
	ssize_t num_read;
	int index;

	mutex_lock(&dad->sysfs_lock);
	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);
	if (!dad->status)
		goto error;

	num_read = dad->response_length;

	for (i = 0; i < num_read; i++)
		index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
				"0x%02X\n", dad->response_buf[i]);

	index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
			"(%zd bytes)\n", num_read);

error:
	mutex_unlock(&dad->sysfs_lock);
	return index;
}

static DEVICE_ATTR(response, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_response_show, NULL);
static ssize_t hw_cyttsp5_response_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct device *cdev=cyttsp5_dev_ptr;
	if(!cdev)
	{
		tp_log_err("%s: device is null \n", __func__);
		return -EINVAL;
	}
	return cyttsp5_response_show(cdev,NULL,buf);
}

static struct kobj_attribute hw_response = {
	.attr = {.name = "response", .mode = (S_IRUSR | S_IRGRP | S_IROTH)},
	.show = hw_cyttsp5_response_show,
	.store = NULL,
};
/*
 * Gets user input from sysfs and parse it
 * return size of parsed output buffer
 */
int cyttsp5_ic_parse_input(struct device *dev, const char *buf,
		size_t buf_size, u8 *ic_buf, size_t ic_buf_size)
{
	const char *pbuf = buf;
	unsigned long value;
	char scan_buf[CYTTSP5_INPUT_ELEM_SZ];
	u32 i = 0;
	u32 j;
	int last = 0;
	int ret;

	tp_log_debug( "%s: pbuf=%p buf=%p size=%zu %s=%zu buf=%s\n", __func__,
			pbuf, buf, buf_size, "scan buf size",
			CYTTSP5_INPUT_ELEM_SZ, buf);

	while (pbuf <= (buf + buf_size)) {
		if (i >= CY_MAX_CONFIG_BYTES) {
			tp_log_err( "%s: %s size=%d max=%d\n", __func__,
					"Max cmd size exceeded", i,
					CY_MAX_CONFIG_BYTES);
			return -EINVAL;
		}
		if (i >= ic_buf_size) {
			tp_log_err( "%s: %s size=%d buf_size=%zu\n", __func__,
					"Buffer size exceeded", i, ic_buf_size);
			return -EINVAL;
		}
		while (((*pbuf == ' ') || (*pbuf == ','))
				&& (pbuf < (buf + buf_size))) {
			last = *pbuf;
			pbuf++;
		}

		if (pbuf >= (buf + buf_size))
			break;

		memset(scan_buf, 0, CYTTSP5_INPUT_ELEM_SZ);
		if ((last == ',') && (*pbuf == ',')) {
			tp_log_err( "%s: %s \",,\" not allowed.\n", __func__,
					"Invalid data format.");
			return -EINVAL;
		}
		for (j = 0; j < (CYTTSP5_INPUT_ELEM_SZ - 1)
				&& (pbuf < (buf + buf_size))
				&& (*pbuf != ' ')
				&& (*pbuf != ','); j++) {
			last = *pbuf;
			scan_buf[j] = *pbuf++;
		}

		ret = kstrtoul(scan_buf, 16, &value);
		if (ret < 0) {
			tp_log_err( "%s: %s '%s' %s%s i=%d r=%d\n", __func__,
					"Invalid data format. ", scan_buf,
					"Use \"0xHH,...,0xHH\"", " instead.",
					i, ret);
			return ret;
		}

		ic_buf[i] = value;
		i++;
	}

	return i;
}

static ssize_t cyttsp5_command_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	ssize_t length;
	int rc;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;
	length = cyttsp5_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		tp_log_err( "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto exit;
	}

	/* write ic_buf to log */
	cyttsp5_pr_buf(dev, dad->ic_buf, length, "ic_buf");

	pm_runtime_get_sync(dev);
	rc = cmd->nonhid_cmd->user_cmd(dev, 1, CY_MAX_PRBUF_SIZE,
			dad->response_buf, length, dad->ic_buf,
			&dad->response_length);
	pm_runtime_put(dev);
	if (rc) {
		dad->response_length = 0;
		tp_log_err( "%s: Failed to store command\n", __func__);
	} else {
		dad->status = 1;
	}

exit:
	mutex_unlock(&dad->sysfs_lock);
	tp_log_debug("%s: return size=%zu\n", __func__, size);
	return size;
}

static DEVICE_ATTR(command, S_IWUSR | S_IWGRP, NULL, cyttsp5_command_store);
static ssize_t hw_cyttsp5_command_store(struct kobject *dev,
		struct kobj_attribute *attr,const char *buf,size_t size)
{
	struct device *cdev=cyttsp5_dev_ptr;
	if(!cdev)
	{
		tp_log_err("%s: device is null \n", __func__);
		return -EINVAL;
	}
	return cyttsp5_command_store(cdev,NULL,buf,size);
}

static struct kobj_attribute hw_command = {
	.attr = {.name = "command", .mode = (S_IWUSR | S_IWGRP)},
	.show = NULL,
	.store = hw_cyttsp5_command_store,
};
/*
 * Suspend scan command
 */
static int cyttsp5_suspend_scan_cmd_(struct device *dev)
{
	int rc;

	rc = cmd->nonhid_cmd->suspend_scanning(dev, 0);
	if (rc < 0)
		tp_log_err( "%s: Suspend scan failed r=%d\n",
			__func__, rc);
	return rc;
}

/*
 * Resume scan command
 */
static int cyttsp5_resume_scan_cmd_(struct device *dev)
{
	int rc;

	rc = cmd->nonhid_cmd->resume_scanning(dev, 0);
	if (rc < 0)
		tp_log_err( "%s: Resume scan failed r=%d\n",
			__func__, rc);
	return rc;
}

/*
 * Execute scan command
 */
static int cyttsp5_exec_scan_cmd_(struct device *dev)
{
	int rc;

	rc = cmd->nonhid_cmd->exec_panel_scan(dev, 0);
	if (rc < 0)
		tp_log_err( "%s: Heatmap start scan failed r=%d\n",
			__func__, rc);
	return rc;
}

/*
 * Retrieve panel data command
 */
static int cyttsp5_ret_scan_data_cmd_(struct device *dev, u16 read_offset,
		u16 read_count, u8 data_id, u8 *response, u8 *config,
		u16 *actual_read_len, u8 *return_buf)
{
	int rc;

	rc = cmd->nonhid_cmd->retrieve_panel_scan(dev, 0, read_offset,
			read_count, data_id, response, config, actual_read_len,
			return_buf);
	if (rc < 0)
		tp_log_err( "%s: Retrieve scan data failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Get data structure command
 */
static int cyttsp5_get_data_structure_cmd_(struct device *dev, u16 read_offset,
		u16 read_length, u8 data_id, u8 *status, u8 *data_format,
		u16 *actual_read_len, u8 *data)
{
	int rc;

	rc = cmd->nonhid_cmd->get_data_structure(dev, 0, read_offset,
			read_length, data_id, status, data_format,
			actual_read_len, data);
	if (rc < 0)
		tp_log_err( "%s: Get data structure failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Run self test command
 */
static int cyttsp5_run_selftest_cmd_(struct device *dev, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;

	rc = cmd->nonhid_cmd->run_selftest(dev, 0, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);
	if (rc < 0)
		tp_log_err( "%s: Run self test failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Get self test result command
 */
static int cyttsp5_get_selftest_result_cmd_(struct device *dev,
		u16 read_offset, u16 read_length, u8 test_id, u8 *status,
		u16 *actual_read_len, u8 *data)
{
	int rc;

	rc = cmd->nonhid_cmd->get_selftest_result(dev, 0, read_offset,
			read_length, test_id, status, actual_read_len, data);
	if (rc < 0)
		tp_log_err( "%s: Get self test result failed r=%d\n",
				__func__, rc);
	return rc;
}

/*
 * Calibrate IDACs command
 */
static int _cyttsp5_calibrate_idacs_cmd(struct device *dev,
		u8 sensing_mode, u8 *status)
{
	int rc;

	rc = cmd->nonhid_cmd->calibrate_idacs(dev, 0, sensing_mode, status);
	return rc;
}

/*
 * Initialize Baselines command
 */
static int _cyttsp5_initialize_baselines_cmd(struct device *dev,
		u8 sensing_mode, u8 *status)
{
	int rc;

	rc = cmd->nonhid_cmd->initialize_baselines(dev, 0, sensing_mode,
			status);
	return rc;
}

static int prepare_print_buffer(int status, u8 *in_buf, int length,
		u8 *out_buf)
{
	int index = 0;
	int i;

	index += scnprintf(out_buf, CY_MAX_PRBUF_SIZE, "status %d\n", status);

	for (i = 0; i < length; i++) {
		index += scnprintf(&out_buf[index],
				CY_MAX_PRBUF_SIZE - index,
				"%02X\n", in_buf[i]);
	}

	return index;
}

static ssize_t cyttsp5_panel_scan_show(struct device *dev,
		struct cyttsp5_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int status = STATUS_FAIL;
	u8 config;
	u16 actual_read_len;
	int length = 0;
	u8 element_size = 0;
	u8 *buf_offset;
	int elem_offset = 0;
	int size;
	int rc;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = cyttsp5_exec_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error on execute panel scan r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	/* Set length to max to read all */
	rc = cyttsp5_ret_scan_data_cmd_(dev, 0, 0xFFFF,
			dad->panel_scan_data_id, dad->ic_buf, &config,
			&actual_read_len, NULL);
	if (rc < 0) {
		tp_log_err( "%s: Error on retrieve panel scan r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;
	element_size = config & 0x07;
	elem_offset = actual_read_len;
	while (actual_read_len > 0) {
		rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, 0xFFFF,
				dad->panel_scan_data_id, NULL, &config,
				&actual_read_len, buf_offset);
		if (rc < 0)
			goto resume_scan;

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem_offset += actual_read_len;
	}
	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);

	/* Do not print command header */
	length -= 5;

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	size = prepare_print_buffer(status, &dad->ic_buf[5], length, buf);

	mutex_unlock(&dad->sysfs_lock);

	return size;
}

static ssize_t cyttsp5_panel_scan_store(struct device *dev,
		struct cyttsp5_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	ssize_t length;
	int rc = 0;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length != 1) {
		tp_log_err( "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->panel_scan_data_id = dad->ic_buf[0];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return size;
}

static CY_ATTR(panel_scan, S_IRUSR | S_IWUSR,
		cyttsp5_panel_scan_show, cyttsp5_panel_scan_store);

static ssize_t cyttsp5_get_idac_show(struct device *dev,
		struct cyttsp5_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int status = STATUS_FAIL;
	u8 cmd_status = 0;
	u8 data_format = 0;
	u16 act_length = 0;
	int length = 0;
	int size;
	int rc;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = cyttsp5_get_data_structure_cmd_(dev, 0, PIP_CMD_MAX_LENGTH,
			dad->get_idac_data_id, &cmd_status, &data_format,
			&act_length, &dad->ic_buf[5]);
	if (rc < 0) {
		tp_log_err( "%s: Error on get data structure r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	dad->ic_buf[0] = cmd_status;
	dad->ic_buf[1] = dad->get_idac_data_id;
	dad->ic_buf[2] = LOW_BYTE(act_length);
	dad->ic_buf[3] = HI_BYTE(act_length);
	dad->ic_buf[4] = data_format;

	length = 5 + act_length;

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	size = prepare_print_buffer(status, dad->ic_buf, length, buf);

	mutex_unlock(&dad->sysfs_lock);

	return size;
}

static ssize_t cyttsp5_get_idac_store(struct device *dev,
		struct cyttsp5_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	ssize_t length;
	int rc = 0;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length != 1) {
		tp_log_err( "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->get_idac_data_id = dad->ic_buf[0];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return size;
}

static CY_ATTR(get_idac, S_IRUSR | S_IWUSR,
		cyttsp5_get_idac_show, cyttsp5_get_idac_store);

static ssize_t cyttsp5_auto_shorts_show(struct device *dev,
		struct cyttsp5_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int status = STATUS_FAIL;
	u8 cmd_status = 0;
	u8 summary_result = 0;
	u16 act_length = 0;
	int length = 0;
	int size;
	int rc;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = cyttsp5_run_selftest_cmd_(dev, CY_ST_ID_AUTOSHORTS, 0,
			&cmd_status, &summary_result, NULL);
	if (rc < 0) {
		tp_log_err( "%s: Error on run self test r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	/* Form response buffer */
	dad->ic_buf[0] = cmd_status;
	dad->ic_buf[1] = summary_result;

	length = 2;

	/* Get data unless test result is success */
	if (cmd_status == CY_CMD_STATUS_SUCCESS
			&& summary_result == CY_ST_RESULT_PASS)
		goto status_success;

	/* Set length to 255 to read all */
	rc = cyttsp5_get_selftest_result_cmd_(dev, 0, 255,
			CY_ST_ID_AUTOSHORTS, &cmd_status,
			&act_length, &dad->ic_buf[6]);
	if (rc < 0) {
		tp_log_err( "%s: Error on get self test result r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	dad->ic_buf[2] = cmd_status;
	dad->ic_buf[3] = CY_ST_ID_AUTOSHORTS;
	dad->ic_buf[4] = LOW_BYTE(act_length);
	dad->ic_buf[5] = HI_BYTE(act_length);

	length = 6 + act_length;

status_success:
	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	size = prepare_print_buffer(status, dad->ic_buf, length, buf);

	mutex_unlock(&dad->sysfs_lock);

	return size;
}

static CY_ATTR(auto_shorts, S_IRUSR, cyttsp5_auto_shorts_show, NULL);

static ssize_t cyttsp5_opens_show(struct device *dev,
		struct cyttsp5_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int status = STATUS_FAIL;
	u8 cmd_status = 0;
	u8 summary_result = 0;
	u16 act_length = 0;
	int length = 0;
	int size;
	int rc;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = cyttsp5_run_selftest_cmd_(dev, CY_ST_ID_OPENS, 0,
			&cmd_status, &summary_result, NULL);
	if (rc < 0) {
		tp_log_err( "%s: Error on run self test r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	/* Form response buffer */
	dad->ic_buf[0] = cmd_status;
	dad->ic_buf[1] = summary_result;

	length = 2;

	/* Get data unless test result is success */
	if (cmd_status == CY_CMD_STATUS_SUCCESS
			&& summary_result == CY_ST_RESULT_PASS)
		goto status_success;

	/* Set length to PIP_CMD_MAX_LENGTH to read all */
	rc = cyttsp5_get_selftest_result_cmd_(dev, 0, PIP_CMD_MAX_LENGTH,
			CY_ST_ID_OPENS, &cmd_status,
			&act_length, &dad->ic_buf[6]);
	if (rc < 0) {
		tp_log_err( "%s: Error on get self test result r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	dad->ic_buf[2] = cmd_status;
	dad->ic_buf[3] = CY_ST_ID_OPENS;
	dad->ic_buf[4] = LOW_BYTE(act_length);
	dad->ic_buf[5] = HI_BYTE(act_length);

	length = 6 + act_length;

status_success:
	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	size = prepare_print_buffer(status, dad->ic_buf, length, buf);

	mutex_unlock(&dad->sysfs_lock);

	return size;
}

static CY_ATTR(opens, S_IRUSR, cyttsp5_opens_show, NULL);

static ssize_t cyttsp5_calibrate_show(struct device *dev,
		struct cyttsp5_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int status = STATUS_FAIL;
	int length = 0;
	int size;
	int rc;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = _cyttsp5_calibrate_idacs_cmd(dev, dad->calibrate_sensing_mode,
			&dad->ic_buf[0]);
	if (rc < 0) {
		tp_log_err( "%s: Error on calibrate idacs r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	length = 1;

	/* Check if baseline initialization is requested */
	if (dad->calibrate_initialize_baselines) {
		/* Perform baseline initialization for all modes */
		rc = _cyttsp5_initialize_baselines_cmd(dev, CY_IB_SM_MUTCAP |
				CY_IB_SM_SELFCAP | CY_IB_SM_BUTTON,
				&dad->ic_buf[length]);
		if (rc < 0) {
			tp_log_err( "%s: Error on initialize baselines r=%d\n",
					__func__, rc);
			goto resume_scan;
		}

		length++;
	}

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	size = prepare_print_buffer(status, dad->ic_buf, length, buf);

	mutex_unlock(&dad->sysfs_lock);

	return size;
}

static ssize_t cyttsp5_calibrate_store(struct device *dev,
		struct cyttsp5_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	ssize_t length;
	int rc = 0;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length != 2) {
		tp_log_err( "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->calibrate_sensing_mode = dad->ic_buf[0];
	dad->calibrate_initialize_baselines = dad->ic_buf[1];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return size;
}

static CY_ATTR(calibrate, S_IRUSR | S_IWUSR,
		cyttsp5_calibrate_show, cyttsp5_calibrate_store);

static ssize_t hw_cyttsp5_calibrate_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct device *cdev=cyttsp5_dev_ptr;
	if(!cdev)
	{
		tp_log_err("%s: device is null \n", __func__);
		return -EINVAL;
	}
	return cyttsp5_calibrate_show(cdev,NULL,buf);
}

static ssize_t hw_cyttsp5_calibrate_store(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct device *cdev=cyttsp5_dev_ptr;
	if(!cdev)
	{
		tp_log_err("%s: device is null \n", __func__);
		return -EINVAL;
	}
	return cyttsp5_calibrate_store(cdev,NULL,buf,size);
}
static struct kobj_attribute hw_calibrate = {
	.attr = {.name = "calibrate", .mode = (S_IRUSR | S_IWUSR)},
	.show = hw_cyttsp5_calibrate_show,
	.store = hw_cyttsp5_calibrate_store,
};
static ssize_t cyttsp5_baseline_show(struct device *dev,
		struct cyttsp5_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int status = STATUS_FAIL;
	int length = 0;
	int size;
	int rc;

	mutex_lock(&dad->sysfs_lock);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_suspend_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error on suspend scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	rc = _cyttsp5_initialize_baselines_cmd(dev, dad->baseline_sensing_mode,
			&dad->ic_buf[0]);
	if (rc < 0) {
		tp_log_err( "%s: Error on initialize baselines r=%d\n",
				__func__, rc);
		goto resume_scan;
	}

	length = 1;

	status = STATUS_SUCCESS;

resume_scan:
	cyttsp5_resume_scan_cmd_(dev);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	if (status == STATUS_FAIL)
		length = 0;

	size = prepare_print_buffer(status, dad->ic_buf, length, buf);

	mutex_unlock(&dad->sysfs_lock);

	return size;
}

static ssize_t cyttsp5_baseline_store(struct device *dev,
		struct cyttsp5_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	ssize_t length;
	int rc = 0;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp5_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length != 1) {
		tp_log_err( "%s: Malformed input\n", __func__);
		rc = -EINVAL;
		goto exit_unlock;
	}

	dad->baseline_sensing_mode = dad->ic_buf[0];

exit_unlock:
	mutex_unlock(&dad->sysfs_lock);

	if (rc)
		return rc;

	return size;
}

static CY_ATTR(baseline, S_IRUSR | S_IWUSR,
		cyttsp5_baseline_show, cyttsp5_baseline_store);

/* add driver interfaces for MMI rawdata test*/
static ssize_t cyttsp5_get_hid_desc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	u16 hid_desc_register = 1;
	ssize_t length;
	int rc;
	int i;
	ssize_t num_read;
	int index = 0;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;

	length = sizeof(hid_desc_register);

	/* Set HID descriptor register */
	memcpy(dad->ic_buf, &hid_desc_register, length);

	/* write ic_buf to log */
	cyttsp5_pr_buf(dev, dad->ic_buf, length, "ic_buf");

	pm_runtime_get_sync(dev);
	rc = cmd->nonhid_cmd->user_cmd(dev, 1, CY_MAX_PRBUF_SIZE,
			dad->response_buf, length, dad->ic_buf,
			&dad->response_length);
	pm_runtime_put(dev);
	if (rc) {
		dad->response_length = 0;
		tp_log_err("%s: Failed to store command\n", __func__);
	} else {
		dad->status = 1;
	}

	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);
	if (!dad->status)
		goto error;

	num_read = dad->response_length;

	for (i = 0; i < num_read; i++)
		index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
				"0x%02X\n", dad->response_buf[i]);

	index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
			"(%zd bytes)\n", num_read);

error:
	mutex_unlock(&dad->sysfs_lock);
	return index;
}

static const u8 get_sys_info_cmd[] = {
	0x04, 0x00, 0x05, 0x00, 0x2F, 0x00, 0x02
};
static const u8 suspend_scan_cmd[] = {
	0x04, 0x00, 0x05, 0x00, 0x2F, 0x00, 0x03
};
static const u8 resume_scan_cmd[] = {
	0x04, 0x00, 0x05, 0x00, 0x2F, 0x00, 0x04
};

static const u8 enable_single_tx_cmd[] = {
	0x04, 0x00, 0x08, 0x00, 0x2F, 0x00, 0x06, 0x1F, 0x01, 0x01
};

static const u8 disable_single_tx_cmd[] = {
	0x04, 0x00, 0x08, 0x00, 0x2F, 0x00, 0x06, 0x1F, 0x01, 0x00
};

static const u8 retrieve_data_structure_cmd[] = {
	0x04, 0x00, 0x0A, 0x00, 0x2F, 0x00, 0x24, 0x00, 0x00, 0x0A, 0x00,0x00
};
static const u8 short_test_cmd[] = {
	0x04, 0x00, 0x06, 0x00, 0x2F, 0x00, 0x26, 0x04
};
static const u8 calibrate_cmd[] = {
	0x04, 0x00, 0x06, 0x00, 0x2F, 0x00, 0x28, 0x00
};

static const u8 panel_scan_cmd[] = {
	0x04, 0x00, 0x05, 0x00, 0x2F, 0x00, 0x2A
};


static int  cyttsp5_cmcp_command_(struct device *dev, const void *in_buf, ssize_t length, u8 *out_buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);

	int rc;
	int index = 0;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;

	memcpy(dad->ic_buf, in_buf, length);

	/* write ic_buf to log */
	cyttsp5_pr_buf(dev, dad->ic_buf, length, "ic_buf");

	pm_runtime_get_sync(dev);
	rc = cmd->nonhid_cmd->user_cmd(dev, 1, CY_MAX_PRBUF_SIZE,
			dad->response_buf, length, dad->ic_buf,
			&dad->response_length);
	pm_runtime_put(dev);
	if (rc) {
		dad->response_length = 0;
		tp_log_err("%s: Failed to store command\n", __func__);
	} else {
		dad->status = 1;
	}

	index = scnprintf(out_buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);

	mutex_unlock(&dad->sysfs_lock);
	return index;
}



static ssize_t cyttsp5_get_sys_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(get_sys_info_cmd);
	index = cyttsp5_cmcp_command_(dev, get_sys_info_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_suspend_scan_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(suspend_scan_cmd);
	index = cyttsp5_cmcp_command_(dev, suspend_scan_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_resume_scan_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(resume_scan_cmd);
	index = cyttsp5_cmcp_command_(dev, resume_scan_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_enable_sigle_tx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(enable_single_tx_cmd);
	index = cyttsp5_cmcp_command_(dev, enable_single_tx_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_disable_sigle_tx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(disable_single_tx_cmd);
	index = cyttsp5_cmcp_command_(dev, disable_single_tx_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_short_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(short_test_cmd);
	index = cyttsp5_cmcp_command_(dev, short_test_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_calibrate_idacs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(calibrate_cmd);
	index = cyttsp5_cmcp_command_(dev, calibrate_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_retrieve_data_structure_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	int index = 0;

	length = sizeof(retrieve_data_structure_cmd);
	index = cyttsp5_cmcp_command_(dev, retrieve_data_structure_cmd, length, buf);

	return index;
}

static ssize_t cyttsp5_read_data_block_row_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);

	ssize_t length;
	int index = 0;
	u8 read_data_block_row_cmd[] = {
		0x04, 0x00, 0x0A, 0x00, 0x2F, 0x00, 0x22, 0x00, 0x00, 0x80, 0x00, 0x00
	};
	int rc;
	int i;
	ssize_t num_read = 0;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;

	length = sizeof(read_data_block_row_cmd);
	memcpy(dad->ic_buf, read_data_block_row_cmd, length);

	pm_runtime_get_sync(dev);

	for(i=0; i<6; i++) {
		dad->ic_buf[7] = i;

		if(i == 5)
			dad->ic_buf[9] = 0x64;

		/* write ic_buf to log */
		cyttsp5_pr_buf(dev, dad->ic_buf, length, "ic_buf");
		rc = cmd->nonhid_cmd->user_cmd(dev, 1, CY_MAX_PRBUF_SIZE,
			dad->response_buf+num_read, length, dad->ic_buf,
			&dad->response_length);
		tp_log_debug("%s: dad->response_length = %d\n", __func__,
				dad->response_length);
		if (rc) {
			dad->response_length = 0;
			tp_log_err("%s: Failed to store command\n", __func__);
			break;
		} else {
			dad->status = 1;
			num_read += dad->response_length;
		}
	}
	pm_runtime_put(dev);

	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);

	dad->response_length = num_read;

	mutex_unlock(&dad->sysfs_lock);
	return index;
}

static ssize_t cyttsp5_panel_scan_mut_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	u8 config;
	u16 actual_read_len;
	int length = 0;
	u8 element_size = 0;
	u8 *buf_offset;
	int elem_offset = 0;
	int rc;
	int index = 0;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err("%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_exec_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err("%s: Error on execute panel scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	dad->panel_scan_data_id = 0;

	/* Set length to max to read all */
	rc = cyttsp5_ret_scan_data_cmd_(dev, 0, 0xFFFF,
			dad->panel_scan_data_id, dad->ic_buf, &config,
			&actual_read_len, NULL);
	if (rc < 0) {
		tp_log_err("%s: Error on retrieve panel scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;
	element_size = config & 0x07;
	elem_offset = actual_read_len;
	while (actual_read_len > 0) {
		rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, 0xFFFF,
				dad->panel_scan_data_id, NULL, &config,
				&actual_read_len, buf_offset);
		if (rc < 0) {
			dad->status = 0;
			goto release_exclusive;
		}

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem_offset += actual_read_len;
	}
	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);
	dad->status = 1;
	dad->response_length = length;
	memcpy(dad->response_buf, dad->ic_buf, length);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);

	mutex_unlock(&dad->sysfs_lock);

	return index;
}

static ssize_t cyttsp5_panel_scan_self_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	u8 config;
	u16 actual_read_len;
	int length = 0;
	u8 element_size = 0;
	u8 *buf_offset;
	int elem_offset = 0;
	int rc;
	int index = 0;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err("%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_exec_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err("%s: Error on execute panel scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	dad->panel_scan_data_id = 3;

	/* Set length to max to read all */
	rc = cyttsp5_ret_scan_data_cmd_(dev, 0, 0xFFFF,
			dad->panel_scan_data_id, dad->ic_buf, &config,
			&actual_read_len, NULL);
	if (rc < 0) {
		tp_log_err("%s: Error on retrieve panel scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;
	element_size = config & 0x07;
	elem_offset = actual_read_len;
	while (actual_read_len > 0) {
		rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, 0xFFFF,
				dad->panel_scan_data_id, NULL, &config,
				&actual_read_len, buf_offset);
		if (rc < 0)
			goto release_exclusive;

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem_offset += actual_read_len;
	}
	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);
	dad->status = 1;
	dad->response_length = length;
	memcpy(dad->response_buf, dad->ic_buf, length);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);

	mutex_unlock(&dad->sysfs_lock);

	return index;
}

static ssize_t cyttsp5_panel_scan_button_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	u8 config;
	u16 actual_read_len;
	int length = 0;
	u8 element_size = 0;
	u8 *buf_offset;
	int elem_offset = 0;
	int rc;
	int index = 0;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err("%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto put_pm_runtime;
	}

	rc = cyttsp5_exec_scan_cmd_(dev);
	if (rc < 0) {
		tp_log_err("%s: Error on execute panel scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	dad->panel_scan_data_id = 9;

	/* Set length to max to read all */
	rc = cyttsp5_ret_scan_data_cmd_(dev, 0, 0xFFFF,
			dad->panel_scan_data_id, dad->ic_buf, &config,
			&actual_read_len, NULL);
	if (rc < 0) {
		tp_log_err("%s: Error on retrieve panel scan r=%d\n",
				__func__, rc);
		goto release_exclusive;
	}

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;
	element_size = config & 0x07;
	elem_offset = actual_read_len;
	while (actual_read_len > 0) {
		rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, 0xFFFF,
				dad->panel_scan_data_id, NULL, &config,
				&actual_read_len, buf_offset);
		if (rc < 0)
			goto release_exclusive;

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem_offset += actual_read_len;
	}
	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);
	dad->status = 1;
	dad->response_length = length;
	memcpy(dad->response_buf, dad->ic_buf, length);

release_exclusive:
	cmd->release_exclusive(dev);

put_pm_runtime:
	pm_runtime_put(dev);

	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);

	mutex_unlock(&dad->sysfs_lock);

	return index;
}
static struct device_attribute attributes[] = {
	__ATTR(get_hid_desc, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_get_hid_desc_show, NULL),
	__ATTR(get_sys_info, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_get_sys_info_show, NULL),
	__ATTR(suspend_scan, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_suspend_scan_show, NULL),
	__ATTR(resume_scan, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_resume_scan_show, NULL),
	__ATTR(enable_single_tx, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_enable_sigle_tx_show, NULL),
	__ATTR(disable_single_tx, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_disable_sigle_tx_show, NULL),
	__ATTR(short_test, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_short_test_show, NULL),
	__ATTR(calibrate_idacs, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_calibrate_idacs_show, NULL),
	__ATTR(retrieve_data_structure, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_retrieve_data_structure_show, NULL),
	__ATTR(read_data_block_row, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_read_data_block_row_show, NULL),
	__ATTR(panel_scan_mut_raw, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_panel_scan_mut_raw_show, NULL),
	__ATTR(panel_scan_self_raw, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_panel_scan_self_raw_show, NULL),
	__ATTR(panel_scan_button, S_IRUSR | S_IRGRP | S_IROTH, cyttsp5_panel_scan_button_show, NULL),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	tp_log_err("%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}
#ifdef TTHE_TUNER_SUPPORT
static ssize_t tthe_get_panel_data_debugfs_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;
	struct device *dev;
	u8 config;
	u16 actual_read_len;
	u16 length = 0;
	u8 element_size = 0;
	u8 *buf_offset;
	u8 *buf_out;
	int elem;
	int elem_offset = 0;
	int print_idx = 0;
	int rc;
	int rc1;
	int i;

	mutex_lock(&dad->debugfs_lock);
	dev = dad->dev;
	buf_out = dad->tthe_get_panel_data_buf;
	if (!buf_out)
		goto release_mutex;

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto put_runtime;

	if (dad->heatmap.scan_start) {
		/* Start scan */
		rc = cyttsp5_exec_scan_cmd_(dev);
		if (rc < 0)
			goto release_exclusive;
	}

	elem = dad->heatmap.num_element;
	rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, elem,
			dad->heatmap.data_type, dad->ic_buf, &config,
			&actual_read_len, NULL);
	if (rc < 0)
		goto release_exclusive;

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;

	element_size = config & CY_CMD_RET_PANEL_ELMNT_SZ_MASK;

	elem -= actual_read_len;
	elem_offset = actual_read_len;
	while (elem > 0) {
		rc = cyttsp5_ret_scan_data_cmd_(dev, elem_offset, elem,
				dad->heatmap.data_type, NULL, &config,
				&actual_read_len, buf_offset);
		if (rc < 0)
			goto release_exclusive;

		if (!actual_read_len)
			break;

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem -= actual_read_len;
		elem_offset += actual_read_len;
	}

	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);

release_exclusive:
	rc1 = cmd->release_exclusive(dev);
put_runtime:
	pm_runtime_put(dev);

	if (rc < 0)
		goto release_mutex;

	print_idx += scnprintf(buf_out, TTHE_TUNER_MAX_BUF, "CY_DATA:");
	for (i = 0; i < length; i++)
		print_idx += scnprintf(buf_out + print_idx,
				TTHE_TUNER_MAX_BUF - print_idx,
				"%02X ", dad->ic_buf[i]);
	print_idx += scnprintf(buf_out + print_idx,
			TTHE_TUNER_MAX_BUF - print_idx,
			":(%d bytes)\n", length);
	rc = simple_read_from_buffer(buf, count, ppos, buf_out, print_idx);
	print_idx = rc;

release_mutex:
	mutex_unlock(&dad->debugfs_lock);
	return print_idx;
}

static ssize_t tthe_get_panel_data_debugfs_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;
	struct device *dev = dad->dev;
	ssize_t length;
	int max_read;
	u8 *buf_in = dad->tthe_get_panel_data_buf;
	int ret;

	mutex_lock(&dad->debugfs_lock);
	ret = copy_from_user(buf_in + (*ppos), buf, count);
	if (ret)
		goto exit;
	buf_in[count] = 0;

	length = cyttsp5_ic_parse_input(dev, buf_in, count, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		tp_log_err( "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto exit;
	}

	/* update parameter value */
	dad->heatmap.num_element = get_unaligned_le16(&dad->ic_buf[3]);
	dad->heatmap.data_type = dad->ic_buf[5];

	if (dad->ic_buf[6] > 0)
		dad->heatmap.scan_start = true;
	else
		dad->heatmap.scan_start = false;

	/* elem can not be bigger then buffer size */
	max_read = CY_CMD_RET_PANEL_HDR;
	max_read += dad->heatmap.num_element * CY_CMD_RET_PANEL_ELMNT_SZ_MAX;

	if (max_read >= CY_MAX_PRBUF_SIZE) {
		dad->heatmap.num_element =
			(CY_MAX_PRBUF_SIZE - CY_CMD_RET_PANEL_HDR)
			/ CY_CMD_RET_PANEL_ELMNT_SZ_MAX;
		tp_log_err( "%s: Will get %d element\n", __func__,
				dad->heatmap.num_element);
	}

exit:
	mutex_unlock(&dad->debugfs_lock);
	tp_log_debug( "%s: return count=%zu\n", __func__, count);
	return count;
}

static int tthe_get_panel_data_debugfs_open(struct inode *inode,
		struct file *filp)
{
	struct cyttsp5_device_access_data *dad = inode->i_private;

	mutex_lock(&dad->debugfs_lock);

	if (dad->tthe_get_panel_data_is_open) {
		mutex_unlock(&dad->debugfs_lock);
		return -EBUSY;
	}

	filp->private_data = inode->i_private;

	dad->tthe_get_panel_data_is_open = 1;
	mutex_unlock(&dad->debugfs_lock);
	return 0;
}

static int tthe_get_panel_data_debugfs_close(struct inode *inode,
		struct file *filp)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;

	mutex_lock(&dad->debugfs_lock);
	filp->private_data = NULL;
	dad->tthe_get_panel_data_is_open = 0;
	mutex_unlock(&dad->debugfs_lock);

	return 0;
}

static const struct file_operations tthe_get_panel_data_fops = {
	.open = tthe_get_panel_data_debugfs_open,
	.release = tthe_get_panel_data_debugfs_close,
	.read = tthe_get_panel_data_debugfs_read,
	.write = tthe_get_panel_data_debugfs_write,
};
#endif

static int cyttsp5_setup_sysfs(struct device *dev)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int rc;

	rc = device_create_file(dev, &dev_attr_command);
	if (rc) {
		tp_log_err( "%s: Error, could not create command\n",
				__func__);
		goto exit;
	}

	rc = device_create_file(dev, &dev_attr_status);
	if (rc) {
		tp_log_err( "%s: Error, could not create status\n",
				__func__);
		goto unregister_command;
	}

	rc = device_create_file(dev, &dev_attr_response);
	if (rc) {
		tp_log_err( "%s: Error, could not create response\n",
				__func__);
		goto unregister_status;
	}

	rc = kobject_init_and_add(&dad->mfg_test, &cyttsp5_ktype, &dev->kobj,
			"mfg_test");
	if (rc) {
		tp_log_err( "Unable to create mfg_test kobject\n");
		goto unregister_response;
	}

	rc = cyttsp5_create_file(dev, &cy_attr_panel_scan);
	if (rc) {
		tp_log_err( "%s: Error, could not create panel_scan\n",
				__func__);
		goto put_kobject;
	}

	rc = cyttsp5_create_file(dev, &cy_attr_auto_shorts);
	if (rc) {
		tp_log_err( "%s: Error, could not create auto_shorts\n",
				__func__);
		goto unregister_panel_scan;
	}

	rc = cyttsp5_create_file(dev, &cy_attr_opens);
	if (rc) {
		tp_log_err( "%s: Error, could not create opens\n",
				__func__);
		goto unregister_auto_shorts;
	}

	rc = cyttsp5_create_file(dev, &cy_attr_get_idac);
	if (rc) {
		tp_log_err( "%s: Error, could not create get_idac\n",
				__func__);
		goto unregister_opens;
	}

	rc = cyttsp5_create_file(dev, &cy_attr_calibrate);
	if (rc) {
		tp_log_err( "%s: Error, could not create calibrate\n",
				__func__);
		goto unregister_get_idac;
	}

	rc = cyttsp5_create_file(dev, &cy_attr_baseline);
	if (rc) {
		tp_log_err( "%s: Error, could not create baseline\n",
				__func__);
		goto unregister_calibrate;
	}

	tp_log_warning("%s: create link and add sysfs interfaces\n", __func__);
	rc = sysfs_create_link(touch_screen_kobject_ts, &dev->kobj, "touch_mmi_test");
	if(rc < 0){
		tp_log_err( "%s: Error, could not create link\n",__func__);
		goto unregister_baseline;
	}

	rc = add_sysfs_interfaces(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error, fail sysfs init\n", __func__);
		goto unregister_link;
	}

#ifdef TTHE_TUNER_SUPPORT
	dad->tthe_get_panel_data_debugfs = debugfs_create_file(
			CYTTSP5_TTHE_TUNER_GET_PANEL_DATA_FILE_NAME,
			0644, NULL, dad, &tthe_get_panel_data_fops);
	if (IS_ERR_OR_NULL(dad->tthe_get_panel_data_debugfs)) {
		tp_log_err( "%s: Error, could not create get_panel_data\n",
				__func__);
		dad->tthe_get_panel_data_debugfs = NULL;
		goto unregister_interfaces;
	}
#endif
	dad->sysfs_nodes_created = true;
	return rc;
#ifdef TTHE_TUNER_SUPPORT
unregister_interfaces:
	remove_sysfs_interfaces(dev);
#endif
unregister_link:
	sysfs_remove_link(touch_screen_kobject_ts, "touch_mmi_test");
unregister_baseline:
	cyttsp5_remove_file(dev, &cy_attr_baseline);
unregister_calibrate:
	cyttsp5_remove_file(dev, &cy_attr_calibrate);
unregister_get_idac:
	cyttsp5_remove_file(dev, &cy_attr_get_idac);
unregister_opens:
	cyttsp5_remove_file(dev, &cy_attr_opens);
unregister_auto_shorts:
	cyttsp5_remove_file(dev, &cy_attr_auto_shorts);
unregister_panel_scan:
	cyttsp5_remove_file(dev, &cy_attr_panel_scan);
put_kobject:
	kobject_put(&dad->mfg_test);
unregister_response:
	device_remove_file(dev, &dev_attr_response);
unregister_status:
	device_remove_file(dev, &dev_attr_status);
unregister_command:
	device_remove_file(dev, &dev_attr_command);
#ifdef CONFIG_HUAWEI_KERNEL
unregister_hw_calibrate:
	sysfs_remove_file(cyttsp5_kobject, &hw_calibrate.attr);
unregister_hw_response:
	sysfs_remove_file(cyttsp5_kobject, &hw_response.attr);
unregister_hw_status:
	sysfs_remove_file(cyttsp5_kobject, &hw_status_node.attr);
unregister_hw_command:
	sysfs_remove_file(cyttsp5_kobject, &hw_command.attr);
unregister_cyttsp5:
	sysfs_remove_file(touch_screen_kobject_ts, &tp_color_info.attr);
unregister_tp_color:
unregister_touch_screen:
#endif /*CONFIG_HUAWEI_KERNEL*/
exit:
	return rc;
}

static int cyttsp5_setup_sysfs_attention(struct device *dev)
{
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);
	int rc = 0;

	dad->si = cmd->request_sysinfo(dev);
	if (!dad->si)
		return -EINVAL;

	rc = cyttsp5_setup_sysfs(dev);

	cmd->unsubscribe_attention(dev, CY_ATTEN_STARTUP,
		CY_MODULE_DEVICE_ACCESS, cyttsp5_setup_sysfs_attention,
		0);

	return rc;
}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICE_ACCESS_API
int cyttsp5_device_access_user_command(const char *core_name, u16 read_len,
		u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	struct cyttsp5_core_data *cd;
	int rc;

	might_sleep();

	/* Check parameters */
	if (!read_buf || !write_buf || !actual_read_len)
		return -EINVAL;

	if (!core_name)
		core_name = CY_DEFAULT_CORE_ID;

	/* Find device */
	cd = cyttsp5_get_core_data((char *)core_name);
	if (!cd) {
		tp_log_err("%s: No device.\n", __func__);
		return -ENODEV;
	}

	pm_runtime_get_sync(cd->dev);
	rc = cmd->nonhid_cmd->user_cmd(cd->dev, 1, read_len, read_buf,
			write_len, write_buf, actual_read_len);
	pm_runtime_put(cd->dev);

	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp5_device_access_user_command);

struct command_work {
	struct work_struct work;
	const char *core_name;
	u16 read_len;
	u8 *read_buf;
	u16 write_len;
	u8 *write_buf;

	void (*cont)(const char *core_name, u16 read_len, u8 *read_buf,
		u16 write_len, u8 *write_buf, u16 actual_read_length,
		int rc);
};

static void cyttsp5_device_access_user_command_work_func(
		struct work_struct *work)
{
	struct command_work *cmd_work =
			container_of(work, struct command_work, work);
	u16 actual_read_length;
	int rc;

	rc = cyttsp5_device_access_user_command(cmd_work->core_name,
			cmd_work->read_len, cmd_work->read_buf,
			cmd_work->write_len, cmd_work->write_buf,
			&actual_read_length);

	if (cmd_work->cont)
		cmd_work->cont(cmd_work->core_name,
			cmd_work->read_len, cmd_work->read_buf,
			cmd_work->write_len, cmd_work->write_buf,
			actual_read_length, rc);

	kfree(cmd_work);
}

int cyttsp5_device_access_user_command_async(const char *core_name,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		void (*cont)(const char *core_name, u16 read_len, u8 *read_buf,
			u16 write_len, u8 *write_buf, u16 actual_read_length,
			int rc))
{
	struct command_work *cmd_work;

	cmd_work = kzalloc(sizeof(*cmd_work), GFP_ATOMIC);
	if (!cmd_work)
		return -ENOMEM;

	cmd_work->core_name = core_name;
	cmd_work->read_len = read_len;
	cmd_work->read_buf = read_buf;
	cmd_work->write_len = write_len;
	cmd_work->write_buf = write_buf;
	cmd_work->cont = cont;

	INIT_WORK(&cmd_work->work,
			cyttsp5_device_access_user_command_work_func);
	schedule_work(&cmd_work->work);

	return 0;
}
EXPORT_SYMBOL_GPL(cyttsp5_device_access_user_command_async);
#endif
static int cyttsp5_device_access_probe(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_device_access_data *dad;
	int rc;

	tp_log_info( "%s:Start probe\n", __func__);
	dad = kzalloc(sizeof(*dad), GFP_KERNEL);
	if (!dad) {
		rc = -ENOMEM;
		goto cyttsp5_device_access_probe_data_failed;
	}

	mutex_init(&dad->sysfs_lock);
	dad->dev = dev;
#ifdef TTHE_TUNER_SUPPORT
	mutex_init(&dad->debugfs_lock);
	dad->heatmap.num_element = 200;
#endif
	cd->cyttsp5_dynamic_data[CY_MODULE_DEVICE_ACCESS] = dad;

	/* get sysinfo */
	dad->si = cmd->request_sysinfo(dev);
	if (dad->si) {
		//cyttsp5_dev_ptr=dev;
		rc = cyttsp5_setup_sysfs(dev);
		if (rc) {
			tp_log_info( "%s:Fail at setup sys, rc:%d.\n", __func__, rc);
			goto cyttsp5_device_access_setup_sysfs_failed;
		}
	} else {
		tp_log_err( "%s: Fail get sysinfo pointer from core p=%p\n",
				__func__, dad->si);
		cmd->subscribe_attention(dev, CY_ATTEN_STARTUP,
			CY_MODULE_DEVICE_ACCESS, cyttsp5_setup_sysfs_attention,
			0);
	}
	cyttsp5_dev_ptr=dev;
	tp_log_info( "%s:Success probe.\n", __func__);
	return 0;

 cyttsp5_device_access_setup_sysfs_failed:
	cd->cyttsp5_dynamic_data[CY_MODULE_DEVICE_ACCESS] = NULL;
	kfree(dad);
 cyttsp5_device_access_probe_data_failed:
	tp_log_err( "%s failed.\n", __func__);
	return rc;
}

static int cyttsp5_device_access_release(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_device_access_data *dad
		= cyttsp5_get_device_access_data(dev);

	if (dad->sysfs_nodes_created) {
		sysfs_remove_file(touch_screen_kobject_ts, &tp_color_info.attr);
		sysfs_remove_file(cyttsp5_kobject, &hw_command.attr);
		sysfs_remove_file(cyttsp5_kobject, &hw_status_node.attr);
		sysfs_remove_file(cyttsp5_kobject, &hw_response.attr);
		sysfs_remove_file(cyttsp5_kobject, &hw_calibrate.attr);
		device_remove_file(dev, &dev_attr_command);
		device_remove_file(dev, &dev_attr_status);
		device_remove_file(dev, &dev_attr_response);
		cyttsp5_remove_file(dev, &cy_attr_panel_scan);
		cyttsp5_remove_file(dev, &cy_attr_auto_shorts);
		cyttsp5_remove_file(dev, &cy_attr_opens);
		cyttsp5_remove_file(dev, &cy_attr_get_idac);
		cyttsp5_remove_file(dev, &cy_attr_calibrate);
		cyttsp5_remove_file(dev, &cy_attr_baseline);
		kobject_del(cyttsp5_kobject);
		sysfs_remove_link(touch_screen_kobject_ts, "touch_mmi_test");
		kobject_put(&dad->mfg_test);
		remove_sysfs_interfaces(dev);
#ifdef TTHE_TUNER_SUPPORT
		debugfs_remove(dad->tthe_get_panel_data_debugfs);
#endif
	} else {
		cmd->unsubscribe_attention(dev, CY_ATTEN_STARTUP,
			CY_MODULE_DEVICE_ACCESS, cyttsp5_setup_sysfs_attention,
			0);
	}

	cd->cyttsp5_dynamic_data[CY_MODULE_DEVICE_ACCESS] = NULL;
	kfree(dad);
	return 0;
}

static char *core_ids[CY_MAX_NUM_CORE_DEVS] = {
	CY_DEFAULT_CORE_ID,
	NULL,
	NULL,
	NULL,
	NULL
};

static int num_core_ids = 1;

module_param_array(core_ids, charp, &num_core_ids, 0);
MODULE_PARM_DESC(core_ids,
	"Core id list of cyttsp5 core devices for device access module");

static int __init cyttsp5_device_access_init(void)
{
	struct cyttsp5_core_data *cd;
	int rc = 0;
	int i, j;

	/* Check for invalid or duplicate core_ids */
	for (i = 0; i < num_core_ids; i++) {
		if (!strlen(core_ids[i])) {
			tp_log_err("%s: core_id %d is empty\n",
				__func__, i+1);
			return -EINVAL;
		}
		for (j = i+1; j < num_core_ids; j++)
			if (!strcmp(core_ids[i], core_ids[j])) {
				tp_log_err("%s: core_ids %d and %d are same\n",
					__func__, i+1, j+1);
				return -EINVAL;
			}
	}

	cmd = cyttsp5_get_commands();
	if (!cmd) {
		tp_log_err("%s:failed to get commands\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < num_core_ids; i++) {
		cd = cyttsp5_get_core_data(core_ids[i]);
		if (!cd)
			continue;

		tp_log_info("%s: Registering device access module for core_id: %s\n",
			__func__, core_ids[i]);
		rc = cyttsp5_device_access_probe(cd->dev);
		if (rc < 0) {
			tp_log_err("%s: Error, failed registering module\n",
				__func__);
			goto fail_unregister_devices;
		}
	}

	tp_log_info("%s: Cypress TTSP Device Access Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return 0;

fail_unregister_devices:
	for (i--; i >= 0; i--) {
		cd = cyttsp5_get_core_data(core_ids[i]);
		if (!cd)
			continue;
		cyttsp5_device_access_release(cd->dev);
		tp_log_info("%s: Unregistering device access module for core_id: %s\n",
			__func__, core_ids[i]);
	}
	return rc;
}
module_init(cyttsp5_device_access_init);

static void __exit cyttsp5_device_access_exit(void)
{
	struct cyttsp5_core_data *cd;
	int i;

	for (i = 0; i < num_core_ids; i++) {
		cd = cyttsp5_get_core_data(core_ids[i]);
		if (!cd)
			continue;
		cyttsp5_device_access_release(cd->dev);
		tp_log_info("%s: Unregistering device access module for core_id: %s\n",
			__func__, core_ids[i]);
	}
}
module_exit(cyttsp5_device_access_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Device Access Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
