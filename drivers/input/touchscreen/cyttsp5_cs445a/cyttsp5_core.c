/*
 * cyttsp5_core.c
 * Cypress TrueTouch(TM) Standard Product V5 Core Module.
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
#include "cyttsp5_core.h"
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#endif/*CONFIG_HUAWEI_DSM*/

int hw_tp_common_debug_mask =1;

#define CONFIG_HWATCH_POWER

#define CY_CORE_STARTUP_RETRY_COUNT		3
u8 tp_color_data = 0;
MODULE_FIRMWARE(CY_FW_FILE_NAME);

static const char *cy_driver_core_name = CYTTSP5_CORE_NAME;
static const char *cy_driver_core_version = CY_DRIVER_VERSION;
static const char *cy_driver_core_date = CY_DRIVER_DATE;

static struct holster_mode cyttsp5_holster_info = {0, 0, 0, 0, 0};
#ifdef CONFIG_HUAWEI_DSM
extern struct tp_dsm_info g_tp_dsm_info;
extern struct dsm_client *tp_cyp_dclient;
extern ssize_t cyttsp5_dsm_record_basic_err_info(struct device *dev);
extern int cyttsp5_tp_report_dsm_err(struct device *dev, int type, int err_numb);
#endif/*CONFIG_HUAWEI_DSM*/
struct device *gdev = NULL;
struct device *cyttsp5_core_dev = NULL;
struct cyttsp5_hid_field {
	int report_count;
	int report_size;
	int size; /* report_count * report_size */
	int offset;
	int data_type;
	int logical_min;
	int logical_max;
	/* Usage Page (Hi 16 bit) + Usage (Lo 16 bit) */
	u32 usage_page;
	u32 collection_usage_pages[CY_HID_MAX_COLLECTIONS];
	struct cyttsp5_hid_report *report;
	bool record_field;
};

struct cyttsp5_hid_report {
	u8 id;
	u8 type;
	int size;
	struct cyttsp5_hid_field *fields[CY_HID_MAX_FIELDS];
	int num_fields;
	int record_field_index;
	int header_size;
	int record_size;
	u32 usage_page;
};

struct atten_node {
	struct list_head node;
	char id;
	struct device *dev;

	int (*func)(struct device *);
	int mode;
};

struct param_node {
	struct list_head node;
	u8 id;
	u32 value;
};

struct cyttsp5_hid_cmd {
	u8 opcode;
	u8 report_type;
	union {
		u8 report_id;
		u8 power_state;
	};
	u8 has_data_register;
	size_t write_length;
	u8 *write_buf;
	u8 *read_buf;
	u8 wait_interrupt;
	u8 reset_cmd;
	u16 timeout_ms;
};

struct cyttsp5_hid_output {
	u8 cmd_type;
	u16 length;
	u8 command_code;
	size_t write_length;
	u8 *write_buf;
	u8 novalidate;
	u8 reset_expected;
	u16 timeout_ms;
};
int gesture_id = 0;
u32 cyttsp_gesture_count[GESTURE_MAX] = {0};

#ifdef CONFIG_HUAWEI_KERNEL
static int cyttsp5_power_off(struct device* dev, struct cyttsp5_core_data *pcore_data);
static int cyttsp5_power_on(struct device* dev, struct cyttsp5_core_data *pcore_data);
#endif /*#ifdef CONFIG_HUAWEI_KERNEL*/

#define SET_CMD_OPCODE(byte, opcode) SET_CMD_LOW(byte, opcode)
#define SET_CMD_REPORT_TYPE(byte, type) SET_CMD_HIGH(byte, ((type) << 4))
#define SET_CMD_REPORT_ID(byte, id) SET_CMD_LOW(byte, id)

#define HID_OUTPUT_APP_COMMAND(command) \
	.cmd_type = HID_OUTPUT_CMD_APP, \
	.command_code = command

#define HID_OUTPUT_BL_COMMAND(command) \
	.cmd_type = HID_OUTPUT_CMD_BL, \
	.command_code = command

#ifdef VERBOSE_DEBUG
void cyttsp5_pr_buf(struct device *dev, u8 *dptr, int size,
		const char *data_name)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	u8 *pr_buf = cd->pr_buf;
	int i, k;
	const char fmt[] = "%02X ";
	int max;

	if (!size)
		return;

	max = (CY_MAX_PRBUF_SIZE - 1) - sizeof(CY_PR_TRUNCATED);

	pr_buf[0] = 0;
	for (i = k = 0; i < size && k < max; i++, k += 3)
		scnprintf(pr_buf + k, CY_MAX_PRBUF_SIZE, fmt, dptr[i]);

	if (size)
		tp_log_vdebug( "%s:  %s[0..%d]=%s%s\n", __func__, data_name,
			size - 1, pr_buf, size <= max ? "" : CY_PR_TRUNCATED);
	else
		tp_log_vdebug( "%s:  %s[]\n", __func__, data_name);
}
EXPORT_SYMBOL_GPL(cyttsp5_pr_buf);
#endif

#ifdef TTHE_TUNER_SUPPORT
static int tthe_print(struct cyttsp5_core_data *cd, u8 *buf, int buf_len,
		const u8 *data_name)
{
	int len = strlen(data_name);
	int i, n;
	u8 *p;
	int remain;

	mutex_lock(&cd->tthe_lock);
	if (!cd->tthe_buf)
		goto exit;

	if (cd->tthe_buf_len + (len + buf_len) > CY_MAX_PRBUF_SIZE)
		goto exit;

	if (len + buf_len == 0)
		goto exit;

	remain = CY_MAX_PRBUF_SIZE - cd->tthe_buf_len;
	if (remain < len)
		len = remain;

	p = cd->tthe_buf + cd->tthe_buf_len;
	memcpy(p, data_name, len);
	cd->tthe_buf_len += len;
	p += len;
	remain -= len;

	*p = 0;
	for (i = 0; i < buf_len; i++) {
		n = scnprintf(p, remain, "%02X ", buf[i]);
		if (!n)
			break;
		p += n;
		remain -= n;
		cd->tthe_buf_len += n;
	}

	n = scnprintf(p, remain, "\n");
	if (!n)
		cd->tthe_buf[cd->tthe_buf_len] = 0;
	cd->tthe_buf_len += n;
	wake_up(&cd->wait_q);
exit:
	mutex_unlock(&cd->tthe_lock);
	return 0;
}

static int _cyttsp5_request_tthe_print(struct device *dev, u8 *buf,
		int buf_len, const u8 *data_name)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return tthe_print(cd, buf, buf_len, data_name);
}
#endif

/*
 * cyttsp5_platform_detect_read()
 *
 * This function is passed to platform detect
 * function to perform a read operation
 */
static int cyttsp5_platform_detect_read(struct device *dev, void *buf, int size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return cyttsp5_adap_read_default(cd, buf, size);
}

/* Must be called with cd->hid_report_lock acquired */
static struct cyttsp5_hid_report *cyttsp5_get_hid_report_(
		struct cyttsp5_core_data *cd, u8 report_type, u8 report_id,
		bool create)
{
	struct cyttsp5_hid_report *report = NULL;
	int i;

	/* Look for created reports */
	for (i = 0; i < cd->num_hid_reports; i++) {
		if (cd->hid_reports[i]->type == report_type
				&& cd->hid_reports[i]->id == report_id) {
			return cd->hid_reports[i];
		}
	}

	/* Create a new report */
	if (create && cd->num_hid_reports < CY_HID_MAX_REPORTS) {
		report = kzalloc(sizeof(struct cyttsp5_hid_report),
				GFP_KERNEL);
		if (!report)
			return NULL;

		report->type = report_type;
		report->id = report_id;
		cd->hid_reports[cd->num_hid_reports++] = report;
	}

	return report;
}

/* Must be called with cd->hid_report_lock acquired */
static void cyttsp5_free_hid_reports_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_report *report;
	int i, j;

	for (i = 0; i < cd->num_hid_reports; i++) {
		report = cd->hid_reports[i];
		for (j = 0; j < report->num_fields; j++)
			kfree(report->fields[j]);
		kfree(report);
		cd->hid_reports[i] = NULL;
	}

	cd->num_hid_reports = 0;
}

static void cyttsp5_free_hid_reports(struct cyttsp5_core_data *cd)
{
	mutex_lock(&cd->hid_report_lock);
	cyttsp5_free_hid_reports_(cd);
	mutex_unlock(&cd->hid_report_lock);
}

/* Must be called with cd->hid_report_lock acquired */
static struct cyttsp5_hid_field *cyttsp5_create_hid_field_(
		struct cyttsp5_hid_report *report)
{
	struct cyttsp5_hid_field *field;

	if (!report)
		return NULL;

	if (report->num_fields == CY_HID_MAX_FIELDS)
		return NULL;

	field = kzalloc(sizeof(struct cyttsp5_hid_field), GFP_KERNEL);
	if (!field)
		return NULL;

	field->report = report;

	report->fields[report->num_fields++] = field;

	return field;
}

static int cyttsp5_add_parameter(struct cyttsp5_core_data *cd,
		u8 param_id, u32 param_value)
{
	struct param_node *param, *param_new;

	/* Check if parameter exists */
	spin_lock(&cd->spinlock);
	list_for_each_entry(param, &cd->param_list, node) {
		if (param->id == param_id) {
			/* Update parameter */
			param->value = param_value;
			tp_log_vdebug( "%s: Update parameter id:%d value:%d\n",
				 __func__, param_id, param_value);
			goto exit_unlock;
		}
	}
	spin_unlock(&cd->spinlock);

	param_new = kzalloc(sizeof(*param_new), GFP_KERNEL);
	if (!param_new)
		return -ENOMEM;

	param_new->id = param_id;
	param_new->value = param_value;

	tp_log_vdebug( "%s: Add parameter id:%d value:%d\n",
		__func__, param_id, param_value);

	spin_lock(&cd->spinlock);
	list_add(&param_new->node, &cd->param_list);
exit_unlock:
	spin_unlock(&cd->spinlock);

	return 0;
}

int request_exclusive(struct cyttsp5_core_data *cd, void *ownptr,
		int timeout_ms)
{
	int t = msecs_to_jiffies(timeout_ms);
	bool with_timeout = (timeout_ms != 0);

	mutex_lock(&cd->system_lock);
	if (!cd->exclusive_dev && cd->exclusive_waits == 0) {
		cd->exclusive_dev = ownptr;
		goto exit;
	}

	cd->exclusive_waits++;
wait:
	mutex_unlock(&cd->system_lock);

	if (with_timeout) {
		t = wait_event_timeout(cd->wait_q, !cd->exclusive_dev, t);
		if (IS_TMO(t)) {
			tp_log_err("%s %d: tmo waiting exclusive access, t = %d\n",
				__func__, __LINE__, t);
			return -ETIME;
		}
	} else {
		wait_event(cd->wait_q, !cd->exclusive_dev);
	}

	mutex_lock(&cd->system_lock);
	if (cd->exclusive_dev)
		goto wait;
	cd->exclusive_dev = ownptr;
	cd->exclusive_waits--;
exit:
	mutex_unlock(&cd->system_lock);
	tp_log_vdebug( "%s: request_exclusive ok=%p\n",
		__func__, ownptr);

	return 0;
}

static int release_exclusive_(struct cyttsp5_core_data *cd, void *ownptr)
{
	if (cd->exclusive_dev != ownptr)
		return -EINVAL;

	tp_log_vdebug( "%s: exclusive_dev %p freed\n",
		__func__, cd->exclusive_dev);
	cd->exclusive_dev = NULL;
	wake_up(&cd->wait_q);
	return 0;
}

/*
 * returns error if was not owned
 */
int release_exclusive(struct cyttsp5_core_data *cd, void *ownptr)
{
	int rc;

	mutex_lock(&cd->system_lock);
	rc = release_exclusive_(cd, ownptr);
	mutex_unlock(&cd->system_lock);

	return rc;
}

static int cyttsp5_hid_exec_cmd_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_cmd *hid_cmd)
{
	int rc;
	u8 *cmd;
	u8 cmd_length;
	u8 cmd_offset = 0;

	cmd_length = 2 /* command register */
		+ 2    /* command */
		+ (hid_cmd->report_id >= 0XF ? 1 : 0)   /* Report ID */
		+ (hid_cmd->has_data_register ? 2 : 0)	/* Data register */
		+ hid_cmd->write_length;                /* Data length */

	cmd = kzalloc(cmd_length, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	/* Set Command register */
	memcpy(&cmd[cmd_offset], &cd->hid_desc.command_register,
			sizeof(cd->hid_desc.command_register));
	cmd_offset += sizeof(cd->hid_desc.command_register);

	/* Set Command */
	SET_CMD_REPORT_TYPE(cmd[cmd_offset], hid_cmd->report_type);

	if (hid_cmd->report_id >= 0XF)
		SET_CMD_REPORT_ID(cmd[cmd_offset], 0xF);
	else
		SET_CMD_REPORT_ID(cmd[cmd_offset], hid_cmd->report_id);
	cmd_offset++;

	SET_CMD_OPCODE(cmd[cmd_offset], hid_cmd->opcode);
	cmd_offset++;

	if (hid_cmd->report_id >= 0XF) {
		cmd[cmd_offset] = hid_cmd->report_id;
		cmd_offset++;
	}

	/* Set Data register */
	if (hid_cmd->has_data_register) {
		memcpy(&cmd[cmd_offset], &cd->hid_desc.data_register,
				sizeof(cd->hid_desc.data_register));
		cmd_offset += sizeof(cd->hid_desc.data_register);
	}

	/* Set Data */
	if (hid_cmd->write_length && hid_cmd->write_buf) {
		memcpy(&cmd[cmd_offset], hid_cmd->write_buf,
				hid_cmd->write_length);
		cmd_offset += hid_cmd->write_length;
	}

	rc = cyttsp5_adap_write_read_specific(cd, cmd_length, cmd,
			hid_cmd->read_buf);
	if (rc)
		tp_log_err( "%s: Fail cyttsp5_adap_transfer\n", __func__);

	kfree(cmd);
	return rc;
}

static int cyttsp5_hid_exec_cmd_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_cmd *hid_cmd)
{
	int rc;
	int t;
	u16 timeout_ms;
	int *cmd_state;

	if (hid_cmd->reset_cmd)
		cmd_state = &cd->hid_reset_cmd_state;
	else
		cmd_state = &cd->hid_cmd_state;

	if (hid_cmd->wait_interrupt) {
		mutex_lock(&cd->system_lock);
		*cmd_state = 1;
		mutex_unlock(&cd->system_lock);
	}

	rc = cyttsp5_hid_exec_cmd_(cd, hid_cmd);
	if (rc) {
		if (hid_cmd->wait_interrupt)
			goto error;

		goto exit;
	}

	if (!hid_cmd->wait_interrupt)
		goto exit;

	if (hid_cmd->timeout_ms)
		timeout_ms = hid_cmd->timeout_ms;
	else
		timeout_ms = CY_HID_RESET_TIMEOUT;

	t = wait_event_timeout(cd->wait_q, (*cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
		tp_log_err( "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	*cmd_state = 0;
	mutex_unlock(&cd->system_lock);

exit:
	return rc;
}

static int cyttsp5_hid_cmd_reset_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_RESET,
		.wait_interrupt = 1,
		.reset_cmd = 1,
		.timeout_ms = CY_HID_RESET_TIMEOUT,
	};

	return cyttsp5_hid_exec_cmd_and_wait_(cd, &hid_cmd);
}

static int cyttsp5_hid_cmd_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_cmd_reset_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_cmd_set_power_(struct cyttsp5_core_data *cd,
		u8 power_state)
{
	int rc;
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_SET_POWER,
		.wait_interrupt = 1,
		.timeout_ms = CY_HID_SET_POWER_TIMEOUT,
	};
	hid_cmd.power_state = power_state;

	rc =  cyttsp5_hid_exec_cmd_and_wait_(cd, &hid_cmd);
	if (rc) {
		tp_log_err( "%s: Failed to set power to state:%d\n",
				__func__, power_state);
			return rc;
	}

	/* validate */
	if ((cd->response_buf[2] != HID_RESPONSE_REPORT_ID)
			|| ((cd->response_buf[3] & 0x3) != power_state)
			|| ((cd->response_buf[4] & 0xF) != HID_CMD_SET_POWER))
		rc = -EINVAL;

	return rc;
}

static int cyttsp5_hid_cmd_set_power(struct cyttsp5_core_data *cd,
		u8 power_state)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_cmd_set_power_(cd, power_state);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static const u16 crc_table[16] = {
	0x0000, 0x1021, 0x2042, 0x3063,
	0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b,
	0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

static u16 _cyttsp5_compute_crc(u8 *buf, u32 size)
{
	u16 remainder = 0xFFFF;
	u16 xor_mask = 0x0000;
	u32 index;
	u32 byte_value;
	u32 table_index;
	u32 crc_bit_width = sizeof(u16) * 8;

	/* Divide the message by polynomial, via the table. */
	for (index = 0; index < size; index++) {
		byte_value = buf[index];
		table_index = ((byte_value >> 4) & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
		table_index = (byte_value & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
	}

	/* Perform the final remainder CRC. */
	return remainder ^ xor_mask;
}

static int cyttsp5_hid_output_validate_bl_response(
		struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	u16 size;
	u16 crc;
	u8 status;

	size = get_unaligned_le16(&cd->response_buf[0]);

	if (hid_output->reset_expected && !size)
		return 0;

	if (cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]
			!= HID_BL_RESPONSE_REPORT_ID) {
		tp_log_err( "%s: HID output response, wrong report_id:%d\n",
			__func__, cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]);
		return -EPROTO;
	}

	if (cd->response_buf[4] != HID_OUTPUT_BL_SOP) {
		tp_log_err( "%s: HID output response, wrong SOP:%d\n",
			__func__, cd->response_buf[4]);
		return -EPROTO;
	}

	if (cd->response_buf[size - 1] != HID_OUTPUT_BL_EOP) {
		tp_log_err( "%s: HID output response, wrong EOP:%d\n",
			__func__, cd->response_buf[size - 1]);
		return -EPROTO;
	}

	crc = _cyttsp5_compute_crc(&cd->response_buf[4], size - 7);
	if (cd->response_buf[size - 3] != LOW_BYTE(crc)
			|| cd->response_buf[size - 2] != HI_BYTE(crc)) {
		tp_log_err( "%s: HID output response, wrong CRC 0x%X\n",
			__func__, crc);
		return -EPROTO;
	}

	status = cd->response_buf[5];
	if (status) {
		tp_log_err( "%s: HID output response, ERROR:%d\n",
			__func__, status);
		return -EPROTO;
	}

	return 0;
}

static int cyttsp5_hid_output_validate_app_response(
		struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int command_code;
	u16 size;

	size = get_unaligned_le16(&cd->response_buf[0]);

	if (hid_output->reset_expected && !size)
		return 0;

	if (cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]
			!= HID_APP_RESPONSE_REPORT_ID) {
		tp_log_err( "%s: HID output response, wrong report_id:%d\n",
			__func__, cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]);
		return -EPROTO;
	}

	command_code = cd->response_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET]
		& HID_OUTPUT_RESPONSE_CMD_MASK;
	if (command_code != hid_output->command_code) {
		tp_log_err(
			"%s: HID output response, wrong command_code:%X\n",
			__func__, command_code);
		return -EPROTO;
	}

	return 0;
}

static void cyttsp5_check_set_parameter(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output, bool raw)
{
	u8 *param_buf;
	u32 param_value = 0;
	u8 param_size;
	u8 param_id;
	int i = 0;

	if (!(cd->cpdata->flags & CY_CORE_FLAG_RESTORE_PARAMETERS))
		return;

	/* Check command input for Set Parameter command */
	if (raw && hid_output->length >= 10 && hid_output->length <= 13
			&& !memcmp(&hid_output->write_buf[0],
					&cd->hid_desc.output_register,
					sizeof(cd->hid_desc.output_register))
			&& hid_output->write_buf[4] ==
					HID_APP_OUTPUT_REPORT_ID
			&& hid_output->write_buf[6] ==
					HID_OUTPUT_SET_PARAM)
		param_buf = &hid_output->write_buf[7];
	else if (!raw && hid_output->cmd_type == HID_OUTPUT_CMD_APP
			&& hid_output->command_code == HID_OUTPUT_SET_PARAM
			&& hid_output->write_length >= 3
			&& hid_output->write_length <= 6)
		param_buf = &hid_output->write_buf[0];
	else
		return;

	/* Get parameter ID, size and value */
	param_id = param_buf[0];
	param_size = param_buf[1];
	if (param_size > 4) {
		tp_log_err( "%s: Invalid parameter size:%d\n", __func__, param_size);
		return;
	}

	param_buf = &param_buf[2];
	while (i < param_size)
		param_value += *(param_buf++) << (8 * i++);

	/* Check command response for Set Parameter command */
	if (cd->response_buf[2] != HID_APP_RESPONSE_REPORT_ID
			|| (cd->response_buf[4] & HID_OUTPUT_CMD_MASK) !=
				HID_OUTPUT_SET_PARAM
			|| cd->response_buf[5] != param_id
			|| cd->response_buf[6] != param_size) {
		tp_log_err( "%s: Set Parameter command not successful\n",
			__func__);
		return;
	}

	cyttsp5_add_parameter(cd, param_id, param_value);
}

static void cyttsp5_check_command(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output, bool raw)
{
	cyttsp5_check_set_parameter(cd, hid_output, raw);
}

static int cyttsp5_hid_output_validate_response(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL)
		return cyttsp5_hid_output_validate_bl_response(cd, hid_output);

	return cyttsp5_hid_output_validate_app_response(cd, hid_output);

}

static int cyttsp5_hid_send_output_user_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;

	if (!hid_output->length || !hid_output->write_buf)
		return -EINVAL;

	rc = cyttsp5_adap_write_read_specific(cd, hid_output->length,
			hid_output->write_buf, NULL);
	if (rc)
		tp_log_err( "%s: Fail cyttsp5_adap_transfer\n", __func__);

	return rc;
}

static int cyttsp5_hid_send_output_user_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;
	int t;

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = HID_OUTPUT_USER_CMD + 1;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_hid_send_output_user_(cd, hid_output);
	if (rc)
		goto error;

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(CY_HID_OUTPUT_USER_TIMEOUT));
	if (IS_TMO(t)) {
		tp_log_err( "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	cyttsp5_check_command(cd, hid_output, true);

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);

exit:
	return rc;
}

static int cyttsp5_hid_send_output_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;
	u8 *cmd;
	u16 length;
	u8 report_id;
	u8 cmd_offset = 0;
	u16 crc;
	u8 cmd_allocated = 0;

	switch (hid_output->cmd_type) {
	case HID_OUTPUT_CMD_APP:
		report_id = HID_APP_OUTPUT_REPORT_ID;
		length = 5;
		break;
	case HID_OUTPUT_CMD_BL:
		report_id = HID_BL_OUTPUT_REPORT_ID;
		length = 11 /* 5 + SOP + LEN(2) + CRC(2) + EOP */;
		break;
	default:
		tp_log_err("%s,cmd_type = %d error.\n", __func__,hid_output->cmd_type);
		return -EINVAL;
	}

	length += hid_output->write_length;

	if (length + 2 > CYTTSP5_PREALLOCATED_CMD_BUFFER) {
		cmd = kzalloc(length + 2, GFP_KERNEL);
		if (!cmd)
			return -ENOMEM;
		cmd_allocated = 1;
	} else {
		cmd = cd->cmd_buf;
	}

	/* Set Output register */
	memcpy(&cmd[cmd_offset], &cd->hid_desc.output_register,
			sizeof(cd->hid_desc.output_register));
	cmd_offset += sizeof(cd->hid_desc.output_register);

	cmd[cmd_offset++] = LOW_BYTE(length);
	cmd[cmd_offset++] = HI_BYTE(length);
	cmd[cmd_offset++] = report_id;
	cmd[cmd_offset++] = 0x0; /* reserved */
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL)
		cmd[cmd_offset++] = HID_OUTPUT_BL_SOP;
	cmd[cmd_offset++] = hid_output->command_code;

	/* Set Data Length for bootloader */
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL) {
		cmd[cmd_offset++] = LOW_BYTE(hid_output->write_length);
		cmd[cmd_offset++] = HI_BYTE(hid_output->write_length);
	}

	/* Set Data */
	if (hid_output->write_length && hid_output->write_buf) {
		memcpy(&cmd[cmd_offset], hid_output->write_buf,
				hid_output->write_length);
		cmd_offset += hid_output->write_length;
	}

	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL) {
		crc = _cyttsp5_compute_crc(&cmd[6],	hid_output->write_length + 4);
		cmd[cmd_offset++] = LOW_BYTE(crc);
		cmd[cmd_offset++] = HI_BYTE(crc);
		cmd[cmd_offset++] = HID_OUTPUT_BL_EOP;
	}

	cyttsp5_pr_buf(cd->dev, cmd, length + 2, "command");
	rc = cyttsp5_adap_write_read_specific(cd, length + 2, cmd, NULL);
	if (rc) {
		tp_log_err("%s %d: Fail cyttsp5_adap_transfer, rc = %d\n",
					__func__, __LINE__, rc);
	}

	if (cmd_allocated)
		kfree(cmd);
	return rc;
}

/*****************************************************************
Parameters    :  cd
                 hid_output
Return        :
Description   :
*****************************************************************/
static int cyttsp5_hid_send_output_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;
	int t;
#ifdef VERBOSE_DEBUG
	u16 size;
#endif
	u16 timeout_ms;

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = hid_output->command_code + 1;
	mutex_unlock(&cd->system_lock);

	if (hid_output->timeout_ms)
		timeout_ms = hid_output->timeout_ms;
	else
		timeout_ms = CY_HID_OUTPUT_TIMEOUT;

	rc = cyttsp5_hid_send_output_(cd, hid_output);
	if (rc)
		goto error;

	/* Workaround for FW defect, CDT165308
	 * bl_launch app creates a glitch in IRQ line */
	if (hid_output->command_code == HID_OUTPUT_BL_LAUNCH_APP) {
		disable_irq(cd->irq);
		msleep(20);
		enable_irq(cd->irq);
	}

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
		tp_log_err( "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	if (!hid_output->novalidate)
		rc = cyttsp5_hid_output_validate_response(cd, hid_output);

	cyttsp5_check_command(cd, hid_output, false);

#ifdef VERBOSE_DEBUG
	size = get_unaligned_le16(&cd->response_buf[0]);
	cyttsp5_pr_buf(cd->dev, cd->response_buf, size, "return_buf");
#endif

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

static int cyttsp5_hid_output_null_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_NULL),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_null(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_null_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_start_bootloader_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_START_BOOTLOADER),
		.timeout_ms = CY_HID_OUTPUT_START_BOOTLOADER_TIMEOUT,
		.reset_expected = 1,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_start_bootloader(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_start_bootloader_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_start_bl(struct device *dev, int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_start_bootloader(cd);

	return cyttsp5_hid_output_start_bootloader_(cd);
}

static void cyttsp5_si_get_cydata(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_cydata *cydata = &cd->sysinfo.cydata;
	struct cyttsp5_cydata_dev *cydata_dev =
		(struct cyttsp5_cydata_dev *)
		&cd->response_buf[HID_SYSINFO_CYDATA_OFFSET];

	cydata->pip_ver_major = cydata_dev->pip_ver_major;
	cydata->pip_ver_minor = cydata_dev->pip_ver_minor;
	cydata->bl_ver_major = cydata_dev->bl_ver_major;
	cydata->bl_ver_minor = cydata_dev->bl_ver_minor;
	cydata->fw_ver_major = cydata_dev->fw_ver_major;
	cydata->fw_ver_minor = cydata_dev->fw_ver_minor;

	cydata->fw_pid = get_unaligned_le16(&cydata_dev->fw_pid);
	cydata->fw_ver_conf = get_unaligned_le16(&cydata_dev->fw_ver_conf);
	cydata->post_code = get_unaligned_le16(&cydata_dev->post_code);
	cydata->revctrl = get_unaligned_le32(&cydata_dev->revctrl);
	cydata->jtag_id_l = get_unaligned_le16(&cydata_dev->jtag_si_id_l);
	cydata->jtag_id_h = get_unaligned_le16(&cydata_dev->jtag_si_id_h);

	memcpy(cydata->mfg_id, cydata_dev->mfg_id, CY_NUM_MFGID);

	cyttsp5_pr_buf(cd->dev, (u8 *)cydata_dev,
			sizeof(struct cyttsp5_cydata_dev), "sysinfo_cydata");
}

static void cyttsp5_si_get_sensing_conf_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sensing_conf_data *scd = &cd->sysinfo.sensing_conf_data;
	struct cyttsp5_sensing_conf_data_dev *scd_dev =
		(struct cyttsp5_sensing_conf_data_dev *)
		&cd->response_buf[HID_SYSINFO_SENSING_OFFSET];

	scd->electrodes_x = scd_dev->electrodes_x;
	scd->electrodes_y = scd_dev->electrodes_y;
	scd->origin_x = scd_dev->origin_x;
	scd->origin_y = scd_dev->origin_y;
	scd->panel_id = scd_dev->panel_id;
	scd->btn = scd_dev->btn;
	scd->scan_mode = scd_dev->scan_mode;
	scd->max_tch = scd_dev->max_num_of_tch_per_refresh_cycle;

	scd->res_x = get_unaligned_le16(&scd_dev->res_x);
	scd->res_y = get_unaligned_le16(&scd_dev->res_y);
	scd->max_z = get_unaligned_le16(&scd_dev->max_z);
	scd->len_x = get_unaligned_le16(&scd_dev->len_x);
	scd->len_y = get_unaligned_le16(&scd_dev->len_y);

	cyttsp5_pr_buf(cd->dev, (u8 *)scd_dev,
			sizeof(struct cyttsp5_sensing_conf_data_dev),
			"sensing_conf_data");
}

static int cyttsp5_si_setup(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int max_tch = si->sensing_conf_data.max_tch;

	if (!si->xy_data)
		si->xy_data = kzalloc(max_tch * si->desc.tch_record_size,
				GFP_KERNEL);
	if (!si->xy_data)
		return -ENOMEM;

	if (!si->xy_mode)
		si->xy_mode = kzalloc(si->desc.tch_header_size, GFP_KERNEL);
	if (!si->xy_mode) {
		kfree(si->xy_data);
		return -ENOMEM;
	}

	return 0;
}

static int cyttsp5_si_get_btn_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int num_btns = 0;
	int num_defined_keys;
	u16 *key_table;
	int btn;
	int i;
	int rc = 0;
	unsigned int btns = cd->response_buf[HID_SYSINFO_BTN_OFFSET]
		& HID_SYSINFO_BTN_MASK;
	size_t btn_keys_size;

	tp_log_vdebug( "%s: get btn data\n", __func__);

	for (i = 0; i < HID_SYSINFO_MAX_BTN; i++) {
		if (btns & (1 << i))
			num_btns++;
	}
	si->num_btns = num_btns;

	if (num_btns) {
		btn_keys_size = num_btns * sizeof(struct cyttsp5_btn);
		if (!si->btn)
			si->btn = kzalloc(btn_keys_size, GFP_KERNEL);
		if (!si->btn)
			return -ENOMEM;

		if (cd->cpdata->sett[CY_IC_GRPNUM_BTN_KEYS] == NULL)
			num_defined_keys = 0;
		else if (cd->cpdata->sett[CY_IC_GRPNUM_BTN_KEYS]->data == NULL)
			num_defined_keys = 0;
		else
			num_defined_keys = cd->cpdata->sett
				[CY_IC_GRPNUM_BTN_KEYS]->size;

		for (btn = 0; btn < num_btns && btn < num_defined_keys; btn++) {
			key_table = (u16 *)cd->cpdata->sett
				[CY_IC_GRPNUM_BTN_KEYS]->data;
			si->btn[btn].key_code = key_table[btn];
			si->btn[btn].enabled = true;
		}
		for (; btn < num_btns; btn++) {
			si->btn[btn].key_code = KEY_RESERVED;
			si->btn[btn].enabled = true;
		}

		return rc;
	}

	kfree(si->btn);
	si->btn = NULL;
	return rc;
}

static void cyttsp5_si_put_log_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	struct cyttsp5_cydata *cydata = &si->cydata;
	struct cyttsp5_sensing_conf_data *scd = &si->sensing_conf_data;
	int i;

	tp_log_debug( "%s: pip_ver_major =0x%02X (%d)\n", __func__,
		cydata->pip_ver_major, cydata->pip_ver_major);
	tp_log_debug( "%s: pip_ver_minor =0x%02X (%d)\n", __func__,
		cydata->pip_ver_minor, cydata->pip_ver_minor);
	tp_log_debug( "%s: fw_pid =0x%04X (%d)\n", __func__,
		cydata->fw_pid, cydata->fw_pid);
	tp_log_debug( "%s: fw_ver_major =0x%02X (%d)\n", __func__,
		cydata->fw_ver_major, cydata->fw_ver_major);
	tp_log_debug( "%s: fw_ver_minor =0x%02X (%d)\n", __func__,
		cydata->fw_ver_minor, cydata->fw_ver_minor);
	tp_log_debug( "%s: revctrl =0x%08X (%d)\n", __func__,
		cydata->revctrl, cydata->revctrl);
	tp_log_debug( "%s: fw_ver_conf =0x%04X (%d)\n", __func__,
		cydata->fw_ver_conf, cydata->fw_ver_conf);
	tp_log_debug( "%s: bl_ver_major =0x%02X (%d)\n", __func__,
		cydata->bl_ver_major, cydata->bl_ver_major);
	tp_log_debug( "%s: bl_ver_minor =0x%02X (%d)\n", __func__,
		cydata->bl_ver_minor, cydata->bl_ver_minor);
	tp_log_debug( "%s: jtag_id_h =0x%04X (%d)\n", __func__,
		cydata->jtag_id_h, cydata->jtag_id_h);
	tp_log_debug( "%s: jtag_id_l =0x%04X (%d)\n", __func__,
		cydata->jtag_id_l, cydata->jtag_id_l);
	for (i = 0; i < CY_NUM_MFGID; i++)
		tp_log_debug( "%s: mfg_id[%d] =0x%02X (%d)\n", __func__, i,
			cydata->mfg_id[i], cydata->mfg_id[i]);
	tp_log_debug( "%s: post_code =0x%04X (%d)\n", __func__,
		cydata->post_code, cydata->post_code);

	tp_log_debug( "%s: electrodes_x =0x%02X (%d)\n", __func__,
		scd->electrodes_x, scd->electrodes_x);
	tp_log_debug( "%s: electrodes_y =0x%02X (%d)\n", __func__,
		scd->electrodes_y, scd->electrodes_y);
	tp_log_debug( "%s: len_x =0x%04X (%d)\n", __func__,
		scd->len_x, scd->len_x);
	tp_log_debug( "%s: len_y =0x%04X (%d)\n", __func__,
		scd->len_y, scd->len_y);
	tp_log_debug( "%s: res_x =0x%04X (%d)\n", __func__,
		scd->res_x, scd->res_x);
	tp_log_debug( "%s: res_y =0x%04X (%d)\n", __func__,
		scd->res_y, scd->res_y);
	tp_log_debug( "%s: max_z =0x%04X (%d)\n", __func__,
		scd->max_z, scd->max_z);
	tp_log_debug( "%s: origin_x =0x%02X (%d)\n", __func__,
		scd->origin_x, scd->origin_x);
	tp_log_debug( "%s: origin_y =0x%02X (%d)\n", __func__,
		scd->origin_y, scd->origin_y);
	tp_log_debug( "%s: panel_id =0x%02X (%d)\n", __func__,
		scd->panel_id, scd->panel_id);
	tp_log_debug( "%s: btn =0x%02X (%d)\n", __func__,
		scd->btn, scd->btn);
	tp_log_debug( "%s: scan_mode =0x%02X (%d)\n", __func__,
		scd->scan_mode, scd->scan_mode);
	tp_log_debug( "%s: max_num_of_tch_per_refresh_cycle =0x%02X (%d)\n",
		__func__, scd->max_tch, scd->max_tch);

	tp_log_debug( "%s: xy_mode =%p\n", __func__,
		si->xy_mode);
	tp_log_debug( "%s: xy_data =%p\n", __func__,
		si->xy_data);
}

static int cyttsp5_get_sysinfo_regs(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;

	rc = cyttsp5_si_get_btn_data(cd);
	if (rc < 0)
		return rc;

	cyttsp5_si_get_cydata(cd);

	cyttsp5_si_get_sensing_conf_data(cd);

	cyttsp5_si_setup(cd);

	cyttsp5_si_put_log_data(cd);

	/* can not read panel id in bootloader model */
	cd->panel_id = cd->sysinfo.sensing_conf_data.panel_id;
	tp_log_warning( "%s: from FW Panel ID =%d\n", __func__,cd->panel_id);
	si->ready = true;
	return rc;
}

static void cyttsp5_free_si_ptrs(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;

	kfree(si->btn);
	kfree(si->xy_mode);
	kfree(si->xy_data);
}

static int cyttsp5_hid_output_get_sysinfo_(struct cyttsp5_core_data *cd)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SYSINFO),
		.timeout_ms = CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	rc = cyttsp5_get_sysinfo_regs(cd);
	if (rc)
		cyttsp5_free_si_ptrs(cd);

	return rc;
}

static int cyttsp5_hid_output_get_sysinfo(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_sysinfo_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_suspend_scanning_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SUSPEND_SCANNING),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_suspend_scanning(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_suspend_scanning_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_suspend_scanning(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_suspend_scanning(cd);

	return cyttsp5_hid_output_suspend_scanning_(cd);
}

static int cyttsp5_hid_output_resume_scanning_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RESUME_SCANNING),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_resume_scanning(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_resume_scanning_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_resume_scanning(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_resume_scanning(cd);

	return cyttsp5_hid_output_resume_scanning_(cd);
}

static int cyttsp5_hid_output_get_param_(struct cyttsp5_core_data *cd,
		u8 param_id, u32 *value)
{
	int write_length = 1;
	u8 param[1] = { param_id };
	u8 read_param_id;
	int param_size;
	u8 *ptr;
	int rc;
	int i;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_PARAM),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	read_param_id = cd->response_buf[5];
	if (read_param_id != param_id)
		return -EPROTO;

	param_size = cd->response_buf[6];
	ptr = &cd->response_buf[7];
	*value = 0;
	for (i = 0; i < param_size; i++)
		*value += ptr[i] << (i * 8);
	return 0;
}

static int cyttsp5_hid_output_get_param(struct cyttsp5_core_data *cd,
		u8 param_id, u32 *value)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_param_(cd, param_id, value);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

int _cyttsp5_request_hid_output_get_param(struct device *dev,
		int protect, u8 param_id, u32 *value)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_get_param(cd, param_id, value);

	return cyttsp5_hid_output_get_param_(cd, param_id, value);
}

static int cyttsp5_hid_output_set_param_(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value)
{
	u8 write_buf[6];
	u8 *ptr = &write_buf[1];
	u8 size = 0;
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SET_PARAM),
		.write_buf = write_buf,
	};

	write_buf[0] = param_id;
	while (value) {
		size++;
		ptr[size] = value & 0xFF;
		value = value >> 8;
	}
	if (size == 0) {
		size = 1;
		ptr[size] = 0;
	}

	write_buf[1] = size;
	hid_output.write_length = 2 + size;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (param_id != cd->response_buf[5] || size != cd->response_buf[6])
		return -EPROTO;

	return 0;
}

static int cyttsp5_hid_output_set_param(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_set_param_(cd, param_id, value);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

int _cyttsp5_request_hid_output_set_param(struct device *dev,
		int protect, u8 param_id, u32 value)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_set_param(cd, param_id, value);

	return cyttsp5_hid_output_set_param_(cd, param_id, value);
}

static int cyttsp5_hid_output_enter_easywake_state_(
		struct cyttsp5_core_data *cd, u8 data, u8 *return_data)
{
	int write_length = 6;
	u8 param[6] = { 0x00 };
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_ENTER_EASYWAKE_STATE),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*return_data = cd->response_buf[6];
	return rc;
}

static int cyttsp5_hid_output_exit_easywake_state_(
		struct cyttsp5_core_data *cd, u8 data, u8 *return_data)
{
	int write_length = 1;
	u8 param[1] = { 0x00 };
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_EXIT_EASYWAKE_STATE),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*return_data = cd->response_buf[6];
	return rc;
}

static int cyttsp5_hid_output_verify_config_block_crc_(
		struct cyttsp5_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int write_length = 1;
	u8 param[1] = { ebid };
	u8 *ptr;
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_VERIFY_CONFIG_BLOCK_CRC),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	ptr = &cd->response_buf[5];
	*status = ptr[0];
	*calculated_crc = get_unaligned_le16(&ptr[1]);
	*stored_crc = get_unaligned_le16(&ptr[3]);
	return 0;
}

static int cyttsp5_hid_output_verify_config_block_crc(
		struct cyttsp5_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_verify_config_block_crc_(cd, ebid, status,
			calculated_crc, stored_crc);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_verify_config_block_crc(
		struct device *dev, int protect, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_verify_config_block_crc(cd, ebid,
				status, calculated_crc, stored_crc);

	return cyttsp5_hid_output_verify_config_block_crc_(cd, ebid,
			status, calculated_crc, stored_crc);
}

static int cyttsp5_hid_output_get_config_row_size_(struct cyttsp5_core_data *cd,
		u16 *row_size)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_CONFIG_ROW_SIZE),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*row_size = get_unaligned_le16(&cd->response_buf[5]);
	return 0;
}

static int cyttsp5_hid_output_get_config_row_size(struct cyttsp5_core_data *cd,
		u16 *row_size)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_config_row_size_(cd, row_size);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_get_config_row_size(struct device *dev,
		int protect, u16 *row_size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_get_config_row_size(cd, row_size);

	return cyttsp5_hid_output_get_config_row_size_(cd, row_size);
}

static int cyttsp5_hid_output_read_conf_block_(struct cyttsp5_core_data *cd,
		u16 row_number, u16 length, u8 ebid, u8 *read_buf, u16 *crc)
{
	int read_ebid;
	int read_length;
	int status;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_READ_CONF_BLOCK),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(row_number);
	write_buf[cmd_offset++] = HI_BYTE(row_number);
	write_buf[cmd_offset++] = LOW_BYTE(length);
	write_buf[cmd_offset++] = HI_BYTE(length);
	write_buf[cmd_offset++] = ebid;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_ebid = cd->response_buf[6];
	if ((read_ebid != ebid) || (cd->response_buf[9] != 0))
		return -EPROTO;

	read_length = get_unaligned_le16(&cd->response_buf[7]);
	if (length < read_length)
		length = read_length;

	memcpy(read_buf, &cd->response_buf[10], length);
	*crc = get_unaligned_le16(&cd->response_buf[read_length + 10]);

	return 0;
}

static int cyttsp5_hid_output_read_conf_ver_(struct cyttsp5_core_data *cd,
		u16 *config_ver)
{
	int rc;
	u8 read_buf[CY_TTCONFIG_VERSION_OFFSET + CY_TTCONFIG_VERSION_SIZE];
	u16 crc;

	rc = cyttsp5_hid_output_read_conf_block_(cd, CY_TTCONFIG_VERSION_ROW,
			CY_TTCONFIG_VERSION_OFFSET + CY_TTCONFIG_VERSION_SIZE,
			CY_TCH_PARM_EBID, read_buf, &crc);
	if (rc)
		return rc;

	*config_ver = get_unaligned_le16(
				&read_buf[CY_TTCONFIG_VERSION_OFFSET]);

	return 0;
}

static int cyttsp5_hid_output_write_conf_block_(struct cyttsp5_core_data *cd,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	/* row_number + write_len + ebid + security_key + crc */
	int full_write_length = 2 + 2 + 1 + write_length + 8 + 2;
	u8 *full_write_buf;
	u8 cmd_offset = 0;
	u16 crc;
	int status;
	int rc;
	int read_ebid;
	u8 *data;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_WRITE_CONF_BLOCK),
		.write_length = full_write_length,
		.timeout_ms = CY_HID_OUTPUT_WRITE_CONF_BLOCK_TIMEOUT,
	};

	full_write_buf = kzalloc(full_write_length, GFP_KERNEL);
	if (!full_write_buf)
		return -ENOMEM;

	hid_output.write_buf = full_write_buf;
	full_write_buf[cmd_offset++] = LOW_BYTE(row_number);
	full_write_buf[cmd_offset++] = HI_BYTE(row_number);
	full_write_buf[cmd_offset++] = LOW_BYTE(write_length);
	full_write_buf[cmd_offset++] = HI_BYTE(write_length);
	full_write_buf[cmd_offset++] = ebid;
	data = &full_write_buf[cmd_offset];
	memcpy(data, write_buf, write_length);
	cmd_offset += write_length;
	memcpy(&full_write_buf[cmd_offset], security_key, 8);
	cmd_offset += 8;
	crc = _cyttsp5_compute_crc(data, write_length);
	full_write_buf[cmd_offset++] = LOW_BYTE(crc);
	full_write_buf[cmd_offset++] = HI_BYTE(crc);

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		goto exit;

	status = cd->response_buf[5];
	if (status) {
		rc = -EINVAL;
		goto exit;
	}

	read_ebid = cd->response_buf[6];
	if (read_ebid != ebid) {
		rc = -EPROTO;
		goto exit;
	}

	*actual_write_len = get_unaligned_le16(&cd->response_buf[7]);

exit:
	kfree(full_write_buf);
	return rc;
}

static int cyttsp5_hid_output_write_conf_block(struct cyttsp5_core_data *cd,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_write_conf_block_(cd, row_number, write_length,
			ebid, write_buf, security_key, actual_write_len);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_write_conf_block(struct device *dev,
		int protect, u16 row_number, u16 write_length, u8 ebid,
		u8 *write_buf, u8 *security_key, u16 *actual_write_len)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_write_conf_block(cd, row_number,
				write_length, ebid, write_buf, security_key,
				actual_write_len);

	return cyttsp5_hid_output_write_conf_block_(cd, row_number,
			write_length, ebid, write_buf, security_key,
			actual_write_len);
}

static int cyttsp5_hid_output_get_data_structure_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 data_id, u8 *status, u8 *data_format, u16 *actual_read_len,
		u8 *data)
{
	int rc;
	u16 total_read_len = 0;
	u16 read_len;
	u16 off_buf = 0;
	u8 write_buf[5];
	u8 read_data_id;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_DATA_STRUCTURE),
		.write_length = 5,
		.write_buf = write_buf,
	};

again:
	write_buf[0] = LOW_BYTE(read_offset);
	write_buf[1] = HI_BYTE(read_offset);
	write_buf[2] = LOW_BYTE(read_length);
	write_buf[3] = HI_BYTE(read_length);
	write_buf[4] = data_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	read_data_id = cd->response_buf[6];
	if (read_data_id != data_id)
		return -EPROTO;

	read_len = get_unaligned_le16(&cd->response_buf[7]);
	if (read_len && data) {
		memcpy(&data[off_buf], &cd->response_buf[10], read_len);

		total_read_len += read_len;

		if (read_len < read_length) {
			read_offset += read_len;
			off_buf += read_len;
			read_length -= read_len;
			goto again;
		}
	}

	if (status)
		*status = cd->response_buf[5];
	if (data_format)
		*data_format = cd->response_buf[9];
	if (actual_read_len)
		*actual_read_len = total_read_len;

	return rc;
}

static int cyttsp5_hid_output_get_data_structure(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 data_id, u8 *status, u8 *data_format, u16 *actual_read_len,
		u8 *data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_data_structure_(cd, read_offset,
			read_length, data_id, status, data_format,
			actual_read_len, data);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_get_data_structure(struct device *dev,
		int protect, u16 read_offset, u16 read_length, u8 data_id,
		u8 *status, u8 *data_format, u16 *actual_read_len, u8 *data)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_get_data_structure(cd,
				read_offset, read_length, data_id, status,
				data_format, actual_read_len, data);

	return cyttsp5_hid_output_get_data_structure_(cd,
			read_offset, read_length, data_id, status,
			data_format, actual_read_len, data);
}

static int cyttsp5_hid_output_run_selftest_(
		struct cyttsp5_core_data *cd, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;
	u8 write_buf[2];
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RUN_SELF_TEST),
		.write_length = 2,
		.write_buf = write_buf,
		.timeout_ms = CY_HID_OUTPUT_RUN_SELF_TEST_TIMEOUT,
	};

	write_buf[0] = test_id;
	write_buf[1] = write_idacs_to_flash;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (status)
		*status = cd->response_buf[5];
	if (summary_result)
		*summary_result = cd->response_buf[6];
	if (results_available)
		*results_available = cd->response_buf[7];

	return rc;
}

static int cyttsp5_hid_output_run_selftest(
		struct cyttsp5_core_data *cd, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_run_selftest_(cd, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_run_selftest(struct device *dev,
		int protect, u8 test_id, u8 write_idacs_to_flash, u8 *status,
		u8 *summary_result, u8 *results_available)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_run_selftest(cd, test_id,
				write_idacs_to_flash, status, summary_result,
				results_available);

	return cyttsp5_hid_output_run_selftest_(cd, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);
}

static int cyttsp5_hid_output_get_selftest_result_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u8 *status, u16 *actual_read_len, u8 *data)
{
	int rc;
	u16 total_read_len = 0;
	u16 read_len;
	u16 off_buf = 0;
	u8 write_buf[5];
	u8 read_test_id;
	bool repeat;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SELF_TEST_RESULT),
		.write_length = 5,
		.write_buf = write_buf,
	};

	/* Repeat reading for Opens test */
	repeat = test_id == CY_ST_ID_OPENS;

again:
	write_buf[0] = LOW_BYTE(read_offset);
	write_buf[1] = HI_BYTE(read_offset);
	write_buf[2] = LOW_BYTE(read_length);
	write_buf[3] = HI_BYTE(read_length);
	write_buf[4] = test_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	read_test_id = cd->response_buf[6];
	if (read_test_id != test_id)
		return -EPROTO;

	read_len = get_unaligned_le16(&cd->response_buf[7]);
	if (read_len && data) {
		memcpy(&data[off_buf], &cd->response_buf[10], read_len);

		total_read_len += read_len;

		if (repeat && read_len < read_length) {
			read_offset += read_len;
			off_buf += read_len;
			read_length -= read_len;
			goto again;
		}
	}
	if (status)
		*status = cd->response_buf[5];
	if (actual_read_len)
		*actual_read_len = total_read_len;

	return rc;
}


static int cyttsp5_hid_output_get_selftest_result(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u8 *status, u16 *actual_read_len, u8 *data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_selftest_result_(cd, read_offset,
			read_length, test_id, status, actual_read_len, data);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}


static int _cyttsp5_request_hid_output_get_selftest_result(struct device *dev,
		int protect, u16 read_offset, u16 read_length, u8 test_id,
		u8 *status, u16 *actual_read_len, u8 *data)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_get_selftest_result(cd, read_offset,
				read_length, test_id, status, actual_read_len,
				data);

	return cyttsp5_hid_output_get_selftest_result_(cd, read_offset,
			read_length, test_id, status, actual_read_len,
			data);
}

static int cyttsp5_hid_output_calibrate_idacs_(struct cyttsp5_core_data *cd,
		u8 mode, u8 *status)
{
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_CALIBRATE_IDACS),
		.write_length = write_length,
		.write_buf = write_buf,
		.timeout_ms = CY_HID_OUTPUT_CALIBRATE_IDAC_TIMEOUT,
	};

	write_buf[cmd_offset++] = mode;
	rc =  cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*status = cd->response_buf[5];
	if (*status)
		return -EINVAL;

	return 0;
}

static int cyttsp5_hid_output_calibrate_idacs(struct cyttsp5_core_data *cd,
		u8 mode, u8 *status)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_calibrate_idacs_(cd, mode, status);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_calibrate_idacs(struct device *dev,
		int protect, u8 mode, u8 *status)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_calibrate_idacs(cd, mode, status);

	return cyttsp5_hid_output_calibrate_idacs_(cd, mode, status);
}

static int cyttsp5_hid_output_initialize_baselines_(
		struct cyttsp5_core_data *cd, u8 test_id, u8 *status)
{
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_INITIALIZE_BASELINES),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = test_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*status = cd->response_buf[5];
	if (*status)
		return -EINVAL;

	return rc;
}

static int cyttsp5_hid_output_initialize_baselines(struct cyttsp5_core_data *cd,
		u8 test_id, u8 *status)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_initialize_baselines_(cd, test_id, status);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_initialize_baselines(struct device *dev,
		int protect, u8 test_id, u8 *status)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_initialize_baselines(cd, test_id,
				status);

	return cyttsp5_hid_output_initialize_baselines_(cd, test_id, status);
}

static int cyttsp5_hid_output_exec_panel_scan_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_EXEC_PANEL_SCAN),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_exec_panel_scan(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_exec_panel_scan_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_exec_panel_scan(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_exec_panel_scan(cd);

	return cyttsp5_hid_output_exec_panel_scan_(cd);
}

/* @response: set none NULL only if all response required including header */
static int cyttsp5_hid_output_retrieve_panel_scan_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_count,
		u8 data_id, u8 *response, u8 *config, u16 *actual_read_len,
		u8 *read_buf)
{
	int status;
	u8 read_data_id;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	u8 data_elem_size;
	int size;
	int data_size;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RETRIEVE_PANEL_SCAN),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(read_offset);
	write_buf[cmd_offset++] = HI_BYTE(read_offset);
	write_buf[cmd_offset++] = LOW_BYTE(read_count);
	write_buf[cmd_offset++] = HI_BYTE(read_count);
	write_buf[cmd_offset++] = data_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_data_id = cd->response_buf[6];
	if (read_data_id != data_id)
		return -EPROTO;

	size = get_unaligned_le16(&cd->response_buf[0]);
	*actual_read_len = get_unaligned_le16(&cd->response_buf[7]);
	*config = cd->response_buf[9];

	data_elem_size = *config & 0x07;
	data_size = *actual_read_len * data_elem_size;

	if (read_buf)
		memcpy(read_buf, &cd->response_buf[10], data_size);
	if (response)
		memcpy(response, cd->response_buf, size);
	return rc;
}

static int cyttsp5_hid_output_retrieve_panel_scan(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_count,
		u8 data_id, u8 *response, u8 *config, u16 *actual_read_len,
		u8 *read_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_retrieve_panel_scan_(cd, read_offset,
			read_count, data_id, response, config,
			actual_read_len, read_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_retrieve_panel_scan(struct device *dev,
		int protect, u16 read_offset, u16 read_count, u8 data_id,
		u8 *response, u8 *config, u16 *actual_read_len, u8 *read_buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_retrieve_panel_scan(cd,
				read_offset, read_count, data_id, response,
				config, actual_read_len, read_buf);

	return cyttsp5_hid_output_retrieve_panel_scan_(cd,
			read_offset, read_count, data_id, response,
			config, actual_read_len, read_buf);
}

static int cyttsp5_hid_output_user_cmd_(struct cyttsp5_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;
	u16 size;
#ifdef TTHE_TUNER_SUPPORT
	int command_code = 0;
	int len;
#endif
	struct cyttsp5_hid_output hid_output = {
		.length = write_len,
		.write_buf = write_buf,
	};

	rc = cyttsp5_hid_send_output_user_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	size = get_unaligned_le16(&cd->response_buf[0]);
	if (size == 0)
		size = 2;

	if (size > read_len) {
		*actual_read_len = 0;
		return -EINVAL;
	}

	memcpy(read_buf, cd->response_buf, size);
	*actual_read_len = size;

#ifdef TTHE_TUNER_SUPPORT
	/* print up to cmd code */
	len = HID_OUTPUT_CMD_OFFSET + 1;
	if (write_len < len)
		len = write_len;
	else
		command_code = write_buf[HID_OUTPUT_CMD_OFFSET]
			& HID_OUTPUT_CMD_MASK;

	/* Do not print for EXEC_PANEL_SCAN & RETRIEVE_PANEL_SCAN commands */
	if (command_code != HID_OUTPUT_EXEC_PANEL_SCAN
			&& command_code != HID_OUTPUT_RETRIEVE_PANEL_SCAN)
		tthe_print(cd, write_buf, len, "CMD=");
#endif

	return 0;
}

static int cyttsp5_get_config_ver_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;
	u16 config_ver = 0;

	rc = cyttsp5_hid_output_suspend_scanning_(cd);
	if (rc)
		goto error;

	rc = cyttsp5_hid_output_read_conf_ver_(cd, &config_ver);
	if (rc)
		goto exit;

	si->cydata.fw_ver_conf = config_ver;

exit:
	cyttsp5_hid_output_resume_scanning_(cd);
error:
	tp_log_debug( "%s: CONFIG_VER:%04X\n", __func__, config_ver);
	return rc;
}

static int cyttsp5_hid_output_user_cmd(struct cyttsp5_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_user_cmd(struct device *dev,
		int protect, u16 read_len, u8 *read_buf, u16 write_len,
		u8 *write_buf, u16 *actual_read_len)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_user_cmd(cd, read_len, read_buf,
				write_len, write_buf, actual_read_len);

	return cyttsp5_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);
}

static int cyttsp5_hid_output_bl_get_information_(struct cyttsp5_core_data *cd,
		u8 *return_data)
{
	int rc;
	int data_len;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_GET_INFO),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	data_len = get_unaligned_le16(&cd->input_buf[6]);
	if (!data_len)
		return -EPROTO;

	memcpy(return_data, &cd->response_buf[8], data_len);

	return 0;
}

static int cyttsp5_hid_output_bl_get_information(struct cyttsp5_core_data *cd,
		u8 *return_data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_get_information_(cd, return_data);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_get_information(struct device *dev,
		int protect, u8 *return_data)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_get_information(cd, return_data);

	return cyttsp5_hid_output_bl_get_information_(cd, return_data);
}

static int cyttsp5_hid_output_bl_initiate_bl_(struct cyttsp5_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	u16 write_length = key_size + row_size;
	u8 *write_buf;
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_INITIATE_BL),
		.write_length = write_length,
		.timeout_ms = CY_HID_OUTPUT_BL_INITIATE_BL_TIMEOUT,
	};

	write_buf = kzalloc(write_length, GFP_KERNEL);
	if (!write_buf)
		return -ENOMEM;

	hid_output.write_buf = write_buf;

	if (key_size)
		memcpy(write_buf, key_buf, key_size);

	if (row_size)
		memcpy(&write_buf[key_size], metadata_row_buf, row_size);

	rc =  cyttsp5_hid_send_output_and_wait_(cd, &hid_output);

	kfree(write_buf);
	return rc;
}

static int cyttsp5_hid_output_bl_initiate_bl(struct cyttsp5_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
			row_size, metadata_row_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_initiate_bl(struct device *dev,
		int protect, u16 key_size, u8 *key_buf, u16 row_size,
		u8 *metadata_row_buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_initiate_bl(cd, key_size, key_buf,
				row_size, metadata_row_buf);

	return cyttsp5_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
			row_size, metadata_row_buf);
}

static int cyttsp5_hid_output_bl_program_and_verify_(
		struct cyttsp5_core_data *cd, u16 data_len, u8 *data_buf)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_PROGRAM_AND_VERIFY),
		.write_length = data_len,
		.write_buf = data_buf,
		.timeout_ms = CY_HID_OUTPUT_BL_PROGRAM_AND_VERIFY_TIMEOUT,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_bl_program_and_verify(
		struct cyttsp5_core_data *cd, u16 data_len, u8 *data_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_program_and_verify_(cd, data_len, data_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_program_and_verify(
		struct device *dev, int protect, u16 data_len, u8 *data_buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_program_and_verify(cd, data_len,
				data_buf);

	return cyttsp5_hid_output_bl_program_and_verify_(cd, data_len,
			data_buf);
}

static int cyttsp5_hid_output_bl_verify_app_integrity_(
		struct cyttsp5_core_data *cd, u8 *result)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_VERIFY_APP_INTEGRITY),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		*result = 0;
		return rc;
	}

	*result = cd->response_buf[8];
	return 0;
}

static int cyttsp5_hid_output_bl_verify_app_integrity(
		struct cyttsp5_core_data *cd, u8 *result)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_verify_app_integrity_(cd, result);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_verify_app_integrity(
		struct device *dev, int protect, u8 *result)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_verify_app_integrity(cd, result);

	return cyttsp5_hid_output_bl_verify_app_integrity_(cd, result);
}

static int cyttsp5_hid_output_bl_launch_app_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_LAUNCH_APP),
		.reset_expected = 1,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_bl_launch_app(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_launch_app_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_launch_app(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_launch_app(cd);

	return cyttsp5_hid_output_bl_launch_app_(cd);
}

static int cyttsp5_hid_output_bl_get_panel_id_(
		struct cyttsp5_core_data *cd, u8 *panel_id)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_GET_PANEL_ID),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc == -EPROTO && cd->response_buf[5] == ERROR_COMMAND) {
		tp_log_debug( "%s: Get Panel ID command not supported\n",
			__func__);
		*panel_id = PANEL_ID_NOT_ENABLED;
		return 0;
	} else if (rc < 0) {
		tp_log_err( "%s: Error on Get Panel ID command\n",
			__func__);
		return rc;
	}

	*panel_id = cd->response_buf[8];
	return 0;
}

/*
static int cyttsp5_hid_output_bl_get_panel_id(
		struct cyttsp5_core_data *cd, u8 *panel_id)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_get_panel_id_(cd, panel_id);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}
*/
static int cyttsp5_get_panel_id(struct cyttsp5_core_data *cd, u8 *panel_id)
{
    *panel_id = cd->response_buf[47];
    return 0;
}


static int cyttsp5_hid_output_get_panel_id_(struct cyttsp5_core_data *cd, u8 *panel_id)
{
    int rc;
    struct cyttsp5_hid_output hid_output = {
        HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SYSINFO),
        .timeout_ms = CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT,
    };

    rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
    if (rc)
        return rc;

    return cyttsp5_get_panel_id(cd, panel_id);
}


static int cyttsp5_hid_output_get_panel_id(struct device *dev, int protect, u8 *panel_id)
{
    int rc;
    struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

    rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
    if (rc < 0) {
        dev_err(cd->dev, "%s: fail get exclusive ex=%p own=%p\n",
            __func__, cd->exclusive_dev, cd->dev);
        return rc;
    }

    rc = cyttsp5_hid_output_get_panel_id_(cd, panel_id);

    if (release_exclusive(cd, cd->dev) < 0)
        dev_err(cd->dev, "%s: fail to release exclusive\n", __func__);

    return rc;
}
/*
static int _cyttsp5_request_hid_output_bl_get_panel_id(
		struct device *dev, int protect, u8 *panel_id)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_get_panel_id(cd, panel_id);

	return cyttsp5_hid_output_bl_get_panel_id_(cd, panel_id);
}
*/

static int cyttsp5_get_hid_descriptor_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	int rc;
	int t;
	u8 cmd[2];

	/* Read HID descriptor length and version */
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	/* Set HID descriptor register */
	memcpy(cmd, &cd->hid_core.hid_desc_register,
		sizeof(cd->hid_core.hid_desc_register));

	rc = cyttsp5_adap_write_read_specific(cd, 2, cmd, NULL);
	if (rc) {
		tp_log_err( "%s: failed to get HID descriptor length and version, rc=%d\n",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(CY_HID_GET_HID_DESCRIPTOR_TIMEOUT));
	if (IS_TMO(t)) {
		tp_log_err( "%s: HID get descriptor timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	memcpy((u8 *)desc, cd->response_buf, sizeof(struct cyttsp5_hid_desc));

	/* Check HID descriptor length and version */
	tp_log_vdebug( "%s: HID len:%X HID ver:%X\n", __func__,
		le16_to_cpu(desc->hid_desc_len),
		le16_to_cpu(desc->bcd_version));

	if (le16_to_cpu(desc->hid_desc_len) != sizeof(*desc) ||
		le16_to_cpu(desc->bcd_version) != CY_HID_VERSION) {
		tp_log_err( "%s: Unsupported HID version\n", __func__);
		return -ENOSYS;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

static int cyttsp5_get_hid_descriptor(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_get_hid_descriptor_(cd, desc);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_get_hid_desc(struct device *dev, int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_get_hid_descriptor(cd, &cd->hid_desc);

	return cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
}

static int cyttsp5_hw_soft_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	if (cd->hid_desc.hid_desc_len == 0) {
		rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0)
			return rc;
	}

	rc = cyttsp5_hid_cmd_reset_(cd);
	if (rc < 0) {
		tp_log_err( "%s: FAILED to execute SOFT reset\n",
				__func__);
		return rc;
	}
	tp_log_debug( "%s: execute SOFT reset\n", __func__);
	return 0;
}

static int cyttsp5_hw_hard_reset(struct cyttsp5_core_data *cd)
{
	if (cd->cpdata->xres) {
		cd->cpdata->xres(cd->cpdata, cd->dev);
		tp_log_debug( "%s: execute HARD reset\n", __func__);
		return 0;
	}
	tp_log_err( "%s: FAILED to execute HARD reset\n", __func__);
	return -ENOSYS;
}

static int cyttsp5_hw_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	rc = cyttsp5_hw_hard_reset(cd);
	mutex_unlock(&cd->system_lock);
	if (rc == -ENOSYS)
		rc = cyttsp5_hw_soft_reset(cd);
	return rc;
}

static inline int get_hid_item_data(u8 *data, int item_size)
{
	if (item_size == 1)
		return (int)*data;
	else if (item_size == 2)
		return (int)get_unaligned_le16(data);
	else if (item_size == 4)
		return (int)get_unaligned_le32(data);
	return 0;
}

static int parse_report_descriptor(struct cyttsp5_core_data *cd,
		u8 *report_desc, size_t len)
{
	struct cyttsp5_hid_report *report;
	struct cyttsp5_hid_field *field;
	u8 *buf = report_desc;
	u8 *end = buf + len;
	int rc = 0;
	int offset = 0;
	int i;
	u8 report_type;
	u32 up_usage;
	/* Global items */
	u8 report_id = 0;
	u16 usage_page = 0;
	int report_count = 0;
	int report_size = 0;
	int logical_min = 0;
	int logical_max = 0;
	/* Local items */
	u16 usage = 0;
	/* Main items - Collection stack */
	u32 collection_usages[CY_HID_MAX_NESTED_COLLECTIONS] = {0};
	u8 collection_types[CY_HID_MAX_NESTED_COLLECTIONS] = {0};
	/* First collection for header, second for report */
	int logical_collection_count = 0;
	int collection_nest = 0;

	tp_log_vdebug( "%s: Report descriptor length: %zu\n",
		__func__, len);

	mutex_lock(&cd->hid_report_lock);
	cyttsp5_free_hid_reports_(cd);

	while (buf < end) {
		int item_type;
		int item_size;
		int item_tag;
		u8 *data;

		/* Get Item */
		item_size = HID_GET_ITEM_SIZE(buf[0]);
		if (item_size == 3)
			item_size = 4;
		item_type = HID_GET_ITEM_TYPE(buf[0]);
		item_tag = HID_GET_ITEM_TAG(buf[0]);

		data = ++buf;
		buf += item_size;

		/* Process current item */
		switch (item_type) {
		case HID_ITEM_TYPE_GLOBAL:
			switch (item_tag) {
			case HID_GLOBAL_ITEM_TAG_REPORT_ID:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}
				report_id = get_hid_item_data(data, item_size);
				offset = 0;
				logical_collection_count = 0;
				break;
			case HID_GLOBAL_ITEM_TAG_USAGE_PAGE:
				if (item_size == 0 || item_size == 4) {
					rc = -EINVAL;
					goto exit;
				}
				usage_page = (u16)get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				logical_min = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				logical_max = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_REPORT_COUNT:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				report_count = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_REPORT_SIZE:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				report_size = get_hid_item_data(data,
						item_size);
				break;
			default:
				tp_log_info("%s: Unrecognized Global tag %d\n",__func__, item_tag);
			}
			break;
		case HID_ITEM_TYPE_LOCAL:
			switch (item_tag) {
			case HID_LOCAL_ITEM_TAG_USAGE:
				if (item_size == 0 || item_size == 4) {
					rc = -EINVAL;
					goto exit;
				}
				usage = (u16)get_hid_item_data(data,
						item_size);
				break;
			case HID_LOCAL_ITEM_TAG_USAGE_MINIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				/* usage_min = */
				get_hid_item_data(data, item_size);
				break;
			case HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				/* usage_max = */
				get_hid_item_data(data, item_size);
				break;
			default:
				tp_log_info("%s: Unrecognized Local tag %d\n",__func__, item_tag);
			}
			break;
		case HID_ITEM_TYPE_MAIN:
			switch (item_tag) {
			case HID_MAIN_ITEM_TAG_BEGIN_COLLECTION:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}
				if (CY_HID_MAX_NESTED_COLLECTIONS ==
						collection_nest) {
					rc = -EINVAL;
					goto exit;
				}

				up_usage = usage_page << 16 | usage;

				/* Update collection stack */
				collection_usages[collection_nest] = up_usage;
				collection_types[collection_nest] =
					get_hid_item_data(data, item_size);

				if (collection_types[collection_nest] ==
						HID_COLLECTION_LOGICAL)
					logical_collection_count++;

				collection_nest++;
				break;
			case HID_MAIN_ITEM_TAG_END_COLLECTION:
				if (item_size != 0) {
					rc = -EINVAL;
					goto exit;
				}
				if (--collection_nest < 0) {
					rc = -EINVAL;
					goto exit;
				}
				break;
			case HID_MAIN_ITEM_TAG_INPUT:
				report_type = HID_INPUT_REPORT;
				goto continue_main_item;
			case HID_MAIN_ITEM_TAG_OUTPUT:
				report_type = HID_OUTPUT_REPORT;
				goto continue_main_item;
			case HID_MAIN_ITEM_TAG_FEATURE:
				report_type = HID_FEATURE_REPORT;
continue_main_item:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}

				up_usage = usage_page << 16 | usage;

				/* Get or create report */
				report = cyttsp5_get_hid_report_(cd,
						report_type, report_id, true);
				if (!report) {
					rc = -ENOMEM;
					goto exit;
				}
				if (!report->usage_page && collection_nest > 0)
					report->usage_page =
						collection_usages
							[collection_nest - 1];

				/* Create field */
				field = cyttsp5_create_hid_field_(report);
				if (!field) {
					rc = -ENOMEM;
					goto exit;
				}

				field->report_count = report_count;
				field->report_size = report_size;
				field->size = report_count * report_size;
				field->offset = offset;
				field->data_type =
					get_hid_item_data(data, item_size);
				field->logical_min = logical_min;
				field->logical_max = logical_max;
				field->usage_page = up_usage;

				for (i = 0; i < collection_nest; i++) {
					field->collection_usage_pages
							[collection_types[i]] =
						collection_usages[i];
				}

				/* Update report's header or record size */
				if (logical_collection_count == 1) {
					report->header_size += field->size;
				} else if (logical_collection_count == 2) {
					field->record_field = true;
					field->offset -= report->header_size;
					/* Set record field index */
					if (report->record_field_index == 0)
						report->record_field_index =
							report->num_fields - 1;
					report->record_size += field->size;
				}

				report->size += field->size;

				offset += field->size;
				break;
			default:
				tp_log_info("%s: Unrecognized Main tag %d\n",__func__, item_tag);
			}

			/* Reset all local items */
			usage = 0;
			break;
		}
	}

	if (buf != end) {
		tp_log_err( "%s: Report descriptor length invalid\n",
			__func__);
		rc = -EINVAL;
		goto exit;
	}

	if (collection_nest) {
		tp_log_err( "%s: Unbalanced collection items (%d)\n",
			__func__, collection_nest);
		rc = -EINVAL;
		goto exit;
	}

exit:
	if (rc)
		cyttsp5_free_hid_reports_(cd);
	mutex_unlock(&cd->hid_report_lock);
	return rc;
}

static struct cyttsp5_hid_field *find_report_desc_field(
		struct cyttsp5_core_data *cd, u32 usage_page,
		u32 collection_usage_page)
{
	struct cyttsp5_hid_report *report = NULL;
	struct cyttsp5_hid_field *field = NULL;
	int i;
	int j;
	u32 field_cup;
	u32 field_up;

	for (i = 0; i < cd->num_hid_reports; i++) {
		report = cd->hid_reports[i];
		for (j = 0; j < report->num_fields; j++) {
			field = report->fields[j];
			field_cup = field->collection_usage_pages
				[HID_COLLECTION_LOGICAL];
			field_up = field->usage_page;
			if (field_cup == collection_usage_page
					&& field_up == usage_page) {
				return field;
			}
		}
	}

	return NULL;
}

static int fill_tch_abs(struct cyttsp5_tch_abs_params *tch_abs,
		struct cyttsp5_hid_field *field)
{
	tch_abs->ofs = field->offset / 8;
	tch_abs->size = field->report_size / 8;
	if (field->report_size % 8)
		tch_abs->size += 1;
	tch_abs->min = 0;
	tch_abs->max = 1 << field->report_size;
	tch_abs->bofs = field->offset - (tch_abs->ofs << 3);

	return 0;
}

static struct cyttsp5_hid_report *find_report_desc(struct cyttsp5_core_data *cd,
		u32 usage_page)
{
	struct cyttsp5_hid_report *report = NULL;
	int i;

	for (i = 0; i < cd->num_hid_reports; i++) {
		if (cd->hid_reports[i]->usage_page == usage_page) {
			report = cd->hid_reports[i];
			break;
		}
	}

	return report;
}

static int setup_report_descriptor(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	struct cyttsp5_hid_report *report;
	struct cyttsp5_hid_field *field;
	int i;
	u32 tch_collection_usage_page = HID_CY_TCH_COL_USAGE_PG;
	u32 btn_collection_usage_page = HID_CY_BTN_COL_USAGE_PG;

	for (i = CY_TCH_X; i < CY_TCH_NUM_ABS; i++) {
		field = find_report_desc_field(cd,
				cyttsp5_tch_abs_field_map[i],
				tch_collection_usage_page);
		if (field) {
			tp_log_vdebug(
				" Field %p: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				field, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);
			fill_tch_abs(&si->tch_abs[i], field);
			si->tch_abs[i].report = 1;
			tp_log_vdebug( "%s: ofs:%zu size:%zu min:%zu max:%zu bofs:%zu report:%d",
				cyttsp5_tch_abs_string[i],
				si->tch_abs[i].ofs, si->tch_abs[i].size,
				si->tch_abs[i].min, si->tch_abs[i].max,
				si->tch_abs[i].bofs, si->tch_abs[i].report);

		} else {
			si->tch_abs[i].report = 0;
		}
	}
	for (i = CY_TCH_TIME; i < CY_TCH_NUM_HDR; i++) {
		field = find_report_desc_field(cd,
				cyttsp5_tch_hdr_field_map[i],
				tch_collection_usage_page);
		if (field) {
			tp_log_vdebug(
				" Field %p: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				field, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);
			fill_tch_abs(&si->tch_hdr[i], field);
			si->tch_hdr[i].report = 1;
			tp_log_vdebug( "%s: ofs:%zu size:%zu min:%zu max:%zu bofs:%zu report:%d",
				cyttsp5_tch_hdr_string[i],
				si->tch_abs[i].ofs, si->tch_abs[i].size,
				si->tch_abs[i].min, si->tch_abs[i].max,
				si->tch_abs[i].bofs, si->tch_abs[i].report);

		} else {
			si->tch_hdr[i].report = 0;
		}
	}

	report = find_report_desc(cd, tch_collection_usage_page);
	if (report) {
		si->desc.tch_report_id = report->id;
		si->desc.tch_record_size = report->record_size / 8;
		si->desc.tch_header_size = (report->header_size / 8) + 3;
	} else {
		si->desc.tch_report_id = HID_TOUCH_REPORT_ID;
		si->desc.tch_record_size = TOUCH_REPORT_SIZE;
		si->desc.tch_header_size = TOUCH_INPUT_HEADER_SIZE;
	}

	report = find_report_desc(cd, btn_collection_usage_page);
	if (report)
		si->desc.btn_report_id = report->id;
	else
		si->desc.tch_report_id = HID_BTN_REPORT_ID;

	for (i = 0; i < cd->num_hid_reports; i++) {
		struct cyttsp5_hid_report *report = cd->hid_reports[i];

		switch (report->id) {
		case HID_WAKEUP_REPORT_ID:
			cd->features.easywake = 1;
			break;
		case HID_NOISE_METRIC_REPORT_ID:
			cd->features.noise_metric = 1;
			break;
		case HID_TRACKING_HEATMAP_REPOR_ID:
			cd->features.tracking_heatmap = 1;
			break;
		case HID_SENSOR_DATA_REPORT_ID:
			cd->features.sensor_data = 1;
			break;
		default:
			break;
		}
	}

	tp_log_debug( "Features: easywake:%d noise_metric:%d tracking_heatmap:%d sensor_data:%d\n",
			cd->features.easywake, cd->features.noise_metric,
			cd->features.tracking_heatmap,
			cd->features.sensor_data);

	return 0;
}

static int cyttsp5_get_report_descriptor_(struct cyttsp5_core_data *cd)
{

	u8 cmd[2];
	int rc;
	int t;

	/* Read report descriptor length and version */
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	/* Set report descriptor register */
	memcpy(cmd, &cd->hid_desc.report_desc_register,
		sizeof(cd->hid_desc.report_desc_register));

	rc = cyttsp5_adap_write_read_specific(cd, 2, cmd, NULL);
	if (rc) {
		tp_log_err( "%s: failed to get HID descriptor length and version, rc=%d\n",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
		msecs_to_jiffies(CY_HID_GET_REPORT_DESCRIPTOR_TIMEOUT));
	if (IS_TMO(t)) {
		tp_log_err( "%s: HID get descriptor timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	cyttsp5_pr_buf(cd->dev, cd->response_buf,
		cd->hid_core.hid_report_desc_len, "Report Desc");

	rc = parse_report_descriptor(cd, cd->response_buf + 3,
		get_unaligned_le16(&cd->response_buf[0]) - 3);
	if (rc) {
		tp_log_err( "%s: Error parsing report descriptor r=%d\n",
			__func__, rc);
	}

	tp_log_debug( "%s: %d reports found in descriptor\n", __func__,
		cd->num_hid_reports);

	for (t = 0; t < cd->num_hid_reports; t++) {
		struct cyttsp5_hid_report *report = cd->hid_reports[t];
		int j;

		tp_log_vdebug(
			"Report %d: type:%d id:%02X size:%d fields:%d rec_fld_index:%d hdr_sz:%d rec_sz:%d usage_page:%08X\n",
			t, report->type, report->id,
			report->size, report->num_fields,
			report->record_field_index, report->header_size,
			report->record_size, report->usage_page);

		for (j = 0; j < report->num_fields; j++) {
			struct cyttsp5_hid_field *field = report->fields[j];

			tp_log_vdebug(
				" Field %d: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				j, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);

			tp_log_vdebug( "  Collections Phys:%08X App:%08X Log:%08X\n",
				field->collection_usage_pages
					[HID_COLLECTION_PHYSICAL],
				field->collection_usage_pages
					[HID_COLLECTION_APPLICATION],
				field->collection_usage_pages
					[HID_COLLECTION_LOGICAL]);
		}
	}

	rc = setup_report_descriptor(cd);

	/* Free it for now */
	cyttsp5_free_hid_reports_(cd);

	tp_log_debug( "%s: %d reports found in descriptor\n", __func__,
		cd->num_hid_reports);

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

static int cyttsp5_get_mode(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	if (CY_HID_APP_REPORT_ID == desc->packet_id)
		return CY_MODE_OPERATIONAL;
	else if (CY_HID_BL_REPORT_ID == desc->packet_id)
		return CY_MODE_BOOTLOADER;

	return CY_MODE_UNKNOWN;
}

static int _cyttsp5_request_get_mode(struct device *dev, int protect, u8 *mode)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	if (protect)
		rc = cyttsp5_get_hid_descriptor(cd, &cd->hid_desc);
	else
		rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);

	if (rc)
		*mode = CY_MODE_UNKNOWN;
	else
		*mode = cyttsp5_get_mode(cd, &cd->hid_desc);

	return rc;
}

static void cyttsp5_queue_startup_(struct cyttsp5_core_data *cd)
{
	if (cd->startup_state == STARTUP_NONE) {
		cd->startup_state = STARTUP_QUEUED;
		schedule_work(&cd->startup_work);
		tp_log_info("%s: cyttsp5_startup queued\n", __func__);
	} else {
		tp_log_debug( "%s: startup_state = %d\n", __func__,
			cd->startup_state);
	}
}

static void cyttsp5_queue_startup(struct cyttsp5_core_data *cd)
{
	mutex_lock(&cd->system_lock);
	cyttsp5_queue_startup_(cd);
	mutex_unlock(&cd->system_lock);
}

static void call_atten_cb(struct cyttsp5_core_data *cd,
		enum cyttsp5_atten_type type, int mode)
{
	struct atten_node *atten, *atten_n;

	tp_log_vdebug( "%s: check list type=%d mode=%d\n",
		__func__, type, mode);
	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(atten, atten_n,
			&cd->atten_list[type], node) {
		if (!mode || atten->mode & mode) {
			spin_unlock(&cd->spinlock);
			tp_log_vdebug( "%s: attention for '%s'", __func__,
				dev_name(atten->dev));
			atten->func(atten->dev);
			spin_lock(&cd->spinlock);
		}
	}
	spin_unlock(&cd->spinlock);
}

void cyttsp5_start_wd_timer(struct cyttsp5_core_data *cd)
{
	if (!CY_WATCHDOG_TIMEOUT) {
		tp_log_info("%s:watch dog timeout:%d\n", __func__, CY_WATCHDOG_TIMEOUT);
		return;
	}

	mod_timer(&cd->watchdog_timer, jiffies +
			msecs_to_jiffies(CY_WATCHDOG_TIMEOUT));
}

void cyttsp5_stop_wd_timer(struct cyttsp5_core_data *cd)
{
	if (!CY_WATCHDOG_TIMEOUT) {
		tp_log_info("%s:watch dog timeout:%d\n", __func__, CY_WATCHDOG_TIMEOUT);
		return;
	}
	/*
	 * Ensure we wait until the watchdog timer
	 * running on a different CPU finishes
	 */
	del_timer_sync(&cd->watchdog_timer);
	cancel_work_sync(&cd->watchdog_work);
	del_timer_sync(&cd->watchdog_timer);
}

static int start_fw_upgrade(void *data)
{
	struct cyttsp5_core_data *cd = (struct cyttsp5_core_data *)data;

	call_atten_cb(cd, CY_ATTEN_LOADER, 0);
	return 0;
}

static void cyttsp5_watchdog_work(struct work_struct *work)
{
	struct cyttsp5_core_data *cd =
			container_of(work, struct cyttsp5_core_data,
					watchdog_work);
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		goto start_wd_timer;
	}

	rc = cyttsp5_hid_output_null_(cd);

	if (release_exclusive(cd, cd->dev) < 0) {
		tp_log_err( "%s: fail to release exclusive\n", __func__);
	}
	if (rc) {
		tp_log_err(
			"%s: failed to access device in watchdog timer r=%d\n",
			__func__, rc);

		/* Already tried FW upgrade because of watchdog but failed */

		/* if dog fail times > CY_WATCHDOG_RETRY_COUNT, still call start_up */
		tp_log_info("%s:current startup_retry_count:%d\n", __func__, cd->startup_retry_count);

		if (cd->startup_retry_count++ == CY_WATCHDOG_RETRY_COUNT)
			kthread_run(start_fw_upgrade, cd, "cyttp5_loader");
		else
			cyttsp5_queue_startup(cd);
		return;
	}

start_wd_timer:
	cyttsp5_start_wd_timer(cd);
}

static void cyttsp5_watchdog_timer(unsigned long handle)
{
	struct cyttsp5_core_data *cd = (struct cyttsp5_core_data *)handle;

	if (!cd)
		return;

	tp_log_vdebug( "%s: Watchdog timer triggered\n", __func__);

	if (!work_pending(&cd->watchdog_work))
		schedule_work(&cd->watchdog_work);
}

static int cyttsp5_put_device_into_easy_wakeup_(struct cyttsp5_core_data *cd)
{
	int rc;
	u8 status = 0;

	mutex_lock(&cd->system_lock);
	cd->wait_until_wake = 0;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_hid_output_enter_easywake_state_(cd,
			cd->easy_wakeup_gesture, &status);
	if (rc || status == 1) {
		tp_log_err("%s:rc = %d, gesture = %d, status = %d", __func__,
			rc, cd->easy_wakeup_gesture, status);
		return -EBUSY;
	}
	/* add for easy wakeup, when system sleep,
	 * we should enable the function of touch panel irq gpio for wakeup system
	 */
	if (device_may_wakeup(cd->dev)) {
		rc = enable_irq_wake(cd->irq);
		if (!rc){
			tp_log_info( "%s Device may wakeup\n", __func__);
		} else {
			tp_log_err( "%s enable irq wake fail, rc = %d\n", __func__, rc);
		}
	} else {
		tp_log_info( "%s Device may not wakeup\n", __func__);
	}

	tp_log_info("%s:rc = %d, gesture = %d, status = %d", __func__,
			rc, cd->easy_wakeup_gesture, status);
	return rc;
}

static int cyttsp5_put_device_into_deep_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = cyttsp5_hid_cmd_set_power_(cd, HID_POWER_SLEEP);
	if (rc)
		rc = -EBUSY;
	return rc;
}

static int cyttsp5_put_device_into_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;

	if (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture)) {
		tp_log_info("%s:put_device into deep sleep\n", __func__);
		rc = cyttsp5_put_device_into_deep_sleep_(cd);
	} else {
		tp_log_info("%s:put_device into easy wakeup\n", __func__);
		rc = cyttsp5_put_device_into_easy_wakeup_(cd);
	}
	return rc;
}

static int cyttsp5_core_poweroff_device_(struct cyttsp5_core_data *cd)
{
	int rc;

	if (cd->irq_enabled) {
		cd->irq_enabled = false;
		disable_irq_nosync(cd->irq);
	}

	rc = cd->cpdata->power(cd->cpdata, 0, cd->dev, 0);
	if (rc < 0)
		tp_log_err( "%s: HW Power down fails r=%d\n",
				__func__, rc);
	return rc;
}

static int cyttsp5_core_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_OFF) {
		cd->sleep_state = SS_SLEEPING;
	} else {
		mutex_unlock(&cd->system_lock);
		tp_log_info("%s:Cyttsp5 is already sleep\n", __func__);
		return 1;
	}
	mutex_unlock(&cd->system_lock);

	/* Ensure watchdog and startup works stopped */
	cyttsp5_stop_wd_timer(cd);
	cancel_work_sync(&cd->startup_work);
	cyttsp5_stop_wd_timer(cd);

	if (cd->cpdata->flags & CY_CORE_FLAG_POWEROFF_ON_SLEEP) {
		tp_log_info("%s: pwoer off device\n", __func__);
		rc = cyttsp5_core_poweroff_device_(cd);
	} else {
		tp_log_info("%s: put device into sleep\n", __func__);
		rc = cyttsp5_put_device_into_sleep_(cd);
	}

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_ON;
	mutex_unlock(&cd->system_lock);

	return rc;
}

static int cyttsp5_core_sleep(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return 0;
	}

	rc = cyttsp5_core_sleep_(cd);

	if (release_exclusive(cd, cd->dev) < 0) {
		tp_log_info( "%s: fail to release exclusive\n", __func__);
	} else {
		tp_log_info( "%s: pass release exclusive\n", __func__);
	}

	return rc;
}

static int cyttsp5_wakeup_host(struct cyttsp5_core_data *cd)
{
	int i = 0;
	int rc = 0;
	int event_id;
	int size = get_unaligned_le16(&cd->input_buf[0]);

	/* Validate report */
	if ((size != CYTTSP5_SIZE_OF_CLICK_DATA && size!= CYTTSP5_SIZE_OF_LETTER_DATA)
			|| cd->input_buf[2] != 4)
		rc = -EINVAL;

	event_id = cd->input_buf[3];

	gesture_id = get_unaligned_le16(&cd->input_buf[4]);
	for(i = 0; i<POINT_NUM; i++){
		cd->gest_pos[i].x = -1;
		cd->gest_pos[i].y = -1;
	}


	for(i=0; i<(size-6)/4; i++) {
		cd->gest_pos[i].x = get_unaligned_le16(&cd->input_buf[6+i*4]);
		cd->gest_pos[i].y = get_unaligned_le16(&cd->input_buf[8+i*4]);
	}

	tp_log_debug( "%s: e=%d, rc=%d\n", __func__, event_id, rc);

	// detect easy wakeup double click area
	if(gesture_id == CYTTSP5_GESTURE_DOUBLE_CLICK && cd->double_tap_enabled) {
		for(i = 0; i < CYTTSP5_CLICK_LOCUS_NUM; i++) {
			if (cd->gest_pos[i].x < cd->dtz_x0 || cd->gest_pos[i].x > cd->dtz_x1||
			    cd->gest_pos[i].y < cd->dtz_y0 || cd->gest_pos[i].y > cd->dtz_y1) {
				tp_log_warning("%s %d:Double click out of area:x0 = %d, y0 = %d ",
								__func__, __LINE__, cd->dtz_x0, cd->dtz_y0);
				tp_log_warning("x1 = %d, y1 = %d\n", cd->dtz_x1, cd->dtz_y1);
				tp_log_warning("%s %d:Click at:x = %d, y = %d\n",
								__func__, __LINE__, cd->gest_pos[i].x, cd->gest_pos[i].y);

				goto exit;
			}
		}
	}

	if (rc) {
		cyttsp5_core_sleep_(cd);
		tp_log_info("%s: rc=%d\n", __func__,rc);
		goto exit;
	}

	/* attention WAKE */
	call_atten_cb(cd, CY_ATTEN_WAKE, 0);
exit:
	return rc;
}

static void cyttsp5_get_touch_axis(struct cyttsp5_core_data *cd,
	int *axis, int size, int max, u8 *data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		*axis = *axis + ((data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;
}

static int move_tracking_hetmap_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "THM=");
#endif
	memcpy(si->xy_mode, cd->input_buf, SENSOR_HEADER_SIZE);
	return 0;
}

static int move_sensor_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "sensor_monitor=");
#endif
	memcpy(si->xy_mode, cd->input_buf, SENSOR_HEADER_SIZE);
	return 0;
}

static int move_button_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "OpModeData=");
#endif
	memcpy(si->xy_mode, cd->input_buf, BTN_INPUT_HEADER_SIZE);
	cyttsp5_pr_buf(cd->dev, (u8 *)si->xy_mode, BTN_INPUT_HEADER_SIZE,
			"xy_mode");

	memcpy(si->xy_data, &cd->input_buf[BTN_INPUT_HEADER_SIZE],
			BTN_REPORT_SIZE);
	cyttsp5_pr_buf(cd->dev, (u8 *)si->xy_data, BTN_REPORT_SIZE, "xy_data");
	return 0;
}

static int move_touch_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
	int max_tch = si->sensing_conf_data.max_tch;
	int num_cur_tch;
	int length;
	struct cyttsp5_tch_abs_params *tch = &si->tch_hdr[CY_TCH_NUM];
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "OpModeData=");
#endif

	memcpy(si->xy_mode, cd->input_buf, si->desc.tch_header_size);
	cyttsp5_pr_buf(cd->dev, (u8 *)si->xy_mode, si->desc.tch_header_size,
			"xy_mode");

	cyttsp5_get_touch_axis(cd, &num_cur_tch, tch->size,
			tch->max, si->xy_mode + 3 + tch->ofs, tch->bofs);
	if (unlikely(num_cur_tch > max_tch))
		num_cur_tch = max_tch;

	length = num_cur_tch * si->desc.tch_record_size;

	memcpy(si->xy_data, &cd->input_buf[si->desc.tch_header_size], length);
	cyttsp5_pr_buf(cd->dev, (u8 *)si->xy_data, length, "xy_data");
	return 0;
}

static int parse_touch_input(struct cyttsp5_core_data *cd, int size)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int report_id = cd->input_buf[2];
	int rc = -EINVAL;

	tp_log_vdebug( "%s: Received touch report\n", __func__);
	if (!si->ready) {
		tp_log_err(
			"%s: Need system information to parse touches\n",
			__func__);
		return 0;
	}

	if (!si->xy_mode || !si->xy_data)
		return rc;

	if (report_id == si->desc.tch_report_id)
		rc = move_touch_data(cd, si);
	else if (report_id == si->desc.btn_report_id)
		rc = move_button_data(cd, si);
	else if (report_id == HID_SENSOR_DATA_REPORT_ID)
		rc = move_sensor_data(cd, si);
	else if (report_id == HID_TRACKING_HEATMAP_REPOR_ID)
		rc = move_tracking_hetmap_data(cd, si);

	if (rc)
		return rc;

	/* attention IRQ */
	call_atten_cb(cd, CY_ATTEN_IRQ, cd->mode);

	return 0;
}

static int parse_command_input(struct cyttsp5_core_data *cd, int size)
{
	tp_log_vdebug( "%s: Received cmd interrupt\n", __func__);

	memcpy(cd->response_buf, cd->input_buf, size);

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
	wake_up(&cd->wait_q);

	return 0;
}

static int cyttsp5_parse_input(struct cyttsp5_core_data *cd)
{
	int report_id;
	int is_command = 0;
	int size;

	size = get_unaligned_le16(&cd->input_buf[0]);

	/* check reset */
	if (size == 0) {
		tp_log_debug( "%s: Reset complete\n", __func__);
		memcpy(cd->response_buf, cd->input_buf, 2);
		mutex_lock(&cd->system_lock);
		if (!cd->hid_reset_cmd_state && !cd->hid_cmd_state) {
			mutex_unlock(&cd->system_lock);
			tp_log_debug( "%s: Device Initiated Reset\n",
					__func__);
			return 0;
		}

		cd->hid_reset_cmd_state = 0;
		if (cd->hid_cmd_state == HID_OUTPUT_START_BOOTLOADER + 1
				|| cd->hid_cmd_state ==
					HID_OUTPUT_BL_LAUNCH_APP + 1
				|| cd->hid_cmd_state ==
					HID_OUTPUT_USER_CMD + 1)
			cd->hid_cmd_state = 0;
		wake_up(&cd->wait_q);
		mutex_unlock(&cd->system_lock);
		return 0;
	} else if (size == 2)
		return 0;

	report_id = cd->input_buf[2];
	tp_log_vdebug( "%s: report_id:%X\n", __func__, report_id);

	/* ESD may lead to (cd->input_buf[5] & 0x80 == 1) */
	if((report_id == HID_TOUCH_REPORT_ID) && (cd->input_buf[5] & 0x80)) {
		tp_log_err("%s %d:ESD error detected\n", __func__, __LINE__);
		cd->transient_state = 1;
		cyttsp5_queue_startup(cd);
		return 0;
	}

	/* Check wake-up report */
	if (report_id == HID_WAKEUP_REPORT_ID) {
		cyttsp5_wakeup_host(cd);
		return 0;
	}

	/* update watchdog expire time */
	mod_timer_pending(&cd->watchdog_timer, jiffies +
			msecs_to_jiffies(CY_WATCHDOG_TIMEOUT));

	if (report_id != cd->sysinfo.desc.tch_report_id
			&& report_id != cd->sysinfo.desc.btn_report_id
			&& report_id != HID_SENSOR_DATA_REPORT_ID
			&& report_id != HID_TRACKING_HEATMAP_REPOR_ID)
		is_command = 1;

	if (unlikely(is_command)) {
		parse_command_input(cd, size);
		return 0;
	}
	parse_touch_input(cd, size);
	return 0;
}

static int cyttsp5_read_input(struct cyttsp5_core_data *cd)
{
	/* delete for fix easy wakeup function do not work */
	int rc = 0;
	unsigned int size = 0;

	rc = cyttsp5_adap_read_default_nosize(cd, cd->input_buf, CY_MAX_INPUT);
	if (rc) {
		tp_log_err( "%s: Error getting report, r=%d\n",
				__func__, rc);
		return rc;
	}

	size = get_unaligned_le16(&cd->input_buf[0]);
	if( size > CY_MAX_INPUT ) {
		tp_log_err("%s line%d: max input size exceeded!size=%d\n",__func__,__LINE__,size);
		size = CY_MAX_INPUT;
		cd->input_buf[0] = (char)size;
		cd->input_buf[1] = (char)(size>>8);
		tp_log_info("%s line%d: manual set size to CY_MAX_INPUT: %d\n",__func__,__LINE__,size);
	}

	tp_log_vdebug( "%s: Read input successfully\n", __func__);
	return rc;
}

static irqreturn_t cyttsp5_irq(int irq, void *handle)
{
	struct cyttsp5_core_data *cd = handle;
	int rc;

	rc = cyttsp5_read_input(cd);
	if (!rc)
		cyttsp5_parse_input(cd);

	return IRQ_HANDLED;
}

int _cyttsp5_subscribe_attention(struct device *dev,
	enum cyttsp5_atten_type type, char id, int (*func)(struct device *),
	int mode)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct atten_node *atten, *atten_new;

	atten_new = kzalloc(sizeof(*atten_new), GFP_KERNEL);
	if (!atten_new)
		return -ENOMEM;

	tp_log_debug( "%s from '%s'\n", __func__, dev_name(cd->dev));

	spin_lock(&cd->spinlock);
	list_for_each_entry(atten, &cd->atten_list[type], node) {
		if (atten->id == id && atten->mode == mode) {
			spin_unlock(&cd->spinlock);
			kfree(atten_new);
			tp_log_vdebug( "%s: %s=%p %s=%d\n",
				 __func__,
				 "already subscribed attention",
				 dev, "mode", mode);

			return 0;
		}
	}

	atten_new->id = id;
	atten_new->dev = dev;
	atten_new->mode = mode;
	atten_new->func = func;

	list_add(&atten_new->node, &cd->atten_list[type]);
	spin_unlock(&cd->spinlock);

	return 0;
}

int _cyttsp5_unsubscribe_attention(struct device *dev,
	enum cyttsp5_atten_type type, char id, int (*func)(struct device *),
	int mode)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct atten_node *atten, *atten_n;

	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(atten, atten_n, &cd->atten_list[type], node) {
		if (atten->id == id && atten->mode == mode) {
			list_del(&atten->node);
			spin_unlock(&cd->spinlock);
			kfree(atten);
			tp_log_vdebug( "%s: %s=%p %s=%d\n",
				__func__,
				"unsub for atten->dev", atten->dev,
				"atten->mode", atten->mode);
			return 0;
		}
	}
	spin_unlock(&cd->spinlock);

	return -ENODEV;
}

static int _cyttsp5_request_exclusive(struct device *dev,
		int timeout_ms)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return request_exclusive(cd, (void *)dev, timeout_ms);
}

static int _cyttsp5_release_exclusive(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return release_exclusive(cd, (void *)dev);
}

static int cyttsp5_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	/* reset hardware */
	tp_log_debug( "%s: reset hw...\n", __func__);
	rc = cyttsp5_hw_reset(cd);
	if (rc < 0)
		tp_log_err( "%s: %s dev='%s' r=%d\n", __func__,
			"Fail hw reset", dev_name(cd->dev), rc);
	return rc;
}

static int cyttsp5_reset_and_wait(struct cyttsp5_core_data *cd)
{
	int rc;
	int t;

	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_reset(cd);
	if (rc < 0)
		goto error;

	t = wait_event_timeout(cd->wait_q, (cd->hid_reset_cmd_state == 0),
			msecs_to_jiffies(CY_HID_RESET_TIMEOUT));
	if (IS_TMO(t)) {
		tp_log_err( "%s: reset timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

/*
 * returns err if refused or timeout(core uses fixed timeout period) occurs;
 * blocks until ISR occurs
 */
static int _cyttsp5_request_reset(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_reset(cd);
	if (rc < 0) {
		tp_log_err( "%s: Error on h/w reset r=%d\n",
			__func__, rc);
		mutex_lock(&cd->system_lock);
		cd->hid_reset_cmd_state = 0;
		mutex_unlock(&cd->system_lock);
	}

	return rc;
}

/*
 * returns err if refused ; if no error then restart has completed
 * and system is in normal operating mode
 */
static int _cyttsp5_request_restart(struct device *dev, bool wait)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	cyttsp5_queue_startup(cd);

	if (wait)
		wait_event(cd->wait_q, cd->startup_state == STARTUP_NONE);

	return 0;
}

/*
 * returns NULL if sysinfo has not been acquired from the device yet
 */
struct cyttsp5_sysinfo *_cyttsp5_request_sysinfo(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (cd->sysinfo.ready)
		return &cd->sysinfo;

	return NULL;
}

static struct cyttsp5_loader_platform_data *_cyttsp5_request_loader_pdata(
		struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return cd->pdata->loader_pdata;
}

static int _cyttsp5_request_stop_wd(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	cyttsp5_stop_wd_timer(cd);
	return 0;
}

static int cyttsp5_core_wake_device_from_deep_sleep_(
		struct cyttsp5_core_data *cd)
{
	int rc = -EAGAIN;
	int retry_times = 0;

	/* wakeup error my happen to CS445A , if wakeup fail,
	watchdog should use 3 second to discover this error, it's a long time,
	so we use retry function to fix this error */
	for (retry_times = 0; retry_times < CY_WAKEUP_RETRY_TIMES; retry_times++) {
		rc = cyttsp5_hid_cmd_set_power_(cd, HID_POWER_ON);
		if (!rc) {
			tp_log_info("%s %d:set power to wake up success\n",
				__func__, __LINE__);
			break;
		}
#ifdef CONFIG_HUAWEI_DSM
		cyttsp5_tp_report_dsm_err(cd->dev, DSM_TP_CYTTSP5_WAKEUP_ERROR_NO, 0);
#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err("%s %d: Fail to set power status, retry = %d\n",
			__func__, __LINE__, retry_times);
		rc =  -EAGAIN;
	}
	/* Prevent failure on sequential wake/sleep requests from OS */
	msleep(20);

	return rc;
}

static int cyttsp5_core_wake_device_from_easy_wakeup_(
		struct cyttsp5_core_data *cd)
{
	int rc;
	u8 status = 0;

	rc = cyttsp5_hid_output_exit_easywake_state_(cd,
			cd->easy_wakeup_gesture, &status);
	if (rc || status == 1){
		tp_log_err("%s: failed, rc=%d, status=%d\n", __func__, rc, status);
		return -EBUSY;
	}

	if (device_may_wakeup(cd->dev)) {
		rc = disable_irq_wake(cd->irq);
		if (!rc){
			tp_log_info( "%s Device may wakeup\n", __func__);
		} else {
			tp_log_err( "%s disable irq wake fail, rc = %d\n", __func__, rc);
		}
	} else {
		tp_log_info( "%s Device may not wakeup\n", __func__);
	}

	return rc;
}

static int cyttsp5_core_wake_device_(struct cyttsp5_core_data *cd)
{
	if (!IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture)) {
		mutex_lock(&cd->system_lock);
		cd->wait_until_wake = 1;
		mutex_unlock(&cd->system_lock);
		wake_up(&cd->wait_q);
		msleep(20);

		if (cd->wake_initiated_by_device) {
			cd->wake_initiated_by_device = 0;
			tp_log_info("%s: wake_initiated_by_device=%d\n", __func__,
						cd->wake_initiated_by_device);
			return 0;
		}
		tp_log_info( "%s:wake device from easy wakeup\n", __func__);
		return cyttsp5_core_wake_device_from_easy_wakeup_(cd);
	}

	tp_log_info( "%s:wake device from deep sleep\n", __func__);
	return cyttsp5_core_wake_device_from_deep_sleep_(cd);
}

static int cyttsp5_restore_parameters_(struct cyttsp5_core_data *cd)
{
	struct param_node *param;
	int rc = 0;

	if (!(cd->cpdata->flags & CY_CORE_FLAG_RESTORE_PARAMETERS)) {
		tp_log_info("%s %d:Exit, flag = %d, para = %d\n", __func__, __LINE__,
			cd->cpdata->flags, CY_CORE_FLAG_RESTORE_PARAMETERS);
		goto exit;
	}

	spin_lock(&cd->spinlock);
	list_for_each_entry(param, &cd->param_list, node) {
		spin_unlock(&cd->spinlock);
		tp_log_vdebug( "%s: Parameter id:%d value:%d\n",
			 __func__, param->id, param->value);
		rc = cyttsp5_hid_output_set_param_(cd, param->id,
				param->value);
		if (rc)
			goto exit;
		spin_lock(&cd->spinlock);
	}
	spin_unlock(&cd->spinlock);
exit:
	return rc;
}

static int _fast_startup(struct cyttsp5_core_data *cd)
{
	int retry = CY_CORE_STARTUP_RETRY_COUNT;
	int rc;

reset:
	if (retry != CY_CORE_STARTUP_RETRY_COUNT)
		tp_log_debug( "%s: Retry %d\n", __func__,
			CY_CORE_STARTUP_RETRY_COUNT - retry);

	rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
	if (rc < 0) {
		tp_log_err( "%s: Error on getting HID descriptor r=%d\n",
			__func__, rc);
		RETRY_OR_EXIT(retry--, reset, exit);
	}
	cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);

	if (cd->mode == CY_MODE_BOOTLOADER) {
		tp_log_info("%s: Bootloader mode\n", __func__);
		rc = cyttsp5_hid_output_bl_launch_app_(cd);
		if (rc < 0) {
			tp_log_err( "%s: Error on launch app r=%d\n",
				__func__, rc);
			RETRY_OR_EXIT(retry--, reset, exit);
		}
		rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0) {
			tp_log_err(
				"%s: Error on getting HID descriptor r=%d\n",
				__func__, rc);
			RETRY_OR_EXIT(retry--, reset, exit);
		}
		cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);
		if (cd->mode == CY_MODE_BOOTLOADER)
			RETRY_OR_EXIT(retry--, reset, exit);
	}

	rc = cyttsp5_restore_parameters_(cd);
	if (rc)
		tp_log_err( "%s: failed to restore parameters rc=%d\n",
			__func__, rc);

exit:
	return rc;
}

static int cyttsp5_core_poweron_device_(struct cyttsp5_core_data *cd)
{
	struct device *dev = cd->dev;
	int rc;

	rc = cd->cpdata->power(cd->cpdata, 1, dev, 0);
	if (rc < 0) {
		tp_log_err( "%s: HW Power up fails r=%d\n", __func__, rc);
		goto exit;
	}

	if (!cd->irq_enabled) {
		cd->irq_enabled = true;
		enable_irq(cd->irq);
	}

	rc = _fast_startup(cd);
exit:
	return rc;
}

static int cyttsp5_core_wake_(struct cyttsp5_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_ON) {
		cd->sleep_state = SS_WAKING;
	} else {
		mutex_unlock(&cd->system_lock);
		tp_log_info("%s %d:Cyttsp5 is already awke\n", __func__, __LINE__);
		return 1;
	}
	mutex_unlock(&cd->system_lock);

	if (cd->cpdata->flags & CY_CORE_FLAG_POWEROFF_ON_SLEEP) {
		tp_log_info("%s %d:Power on deivce\n", __func__, __LINE__);
		rc = cyttsp5_core_poweron_device_(cd);
	} else {
		tp_log_info("%s %d:Wake deivce\n", __func__, __LINE__);
		rc = cyttsp5_core_wake_device_(cd);
	}

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_OFF;
	mutex_unlock(&cd->system_lock);

	cyttsp5_start_wd_timer(cd);
	return rc;
}

static int cyttsp5_core_wake(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return 0;
	}

	rc = cyttsp5_core_wake_(cd);

	if (release_exclusive(cd, cd->dev) < 0) {
		tp_log_err("%s: fail to release exclusive\n", __func__);
	} else {
		tp_log_info("%s: pass release exclusive\n", __func__);
	}
	return rc;
}

static int cyttsp5_get_ic_crc_(struct cyttsp5_core_data *cd, u8 ebid)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;
	u8 status;
	u16 calculated_crc = 0;
	u16 stored_crc = 0;

	rc = cyttsp5_hid_output_suspend_scanning_(cd);
	if (rc)
		goto error;

	rc = cyttsp5_hid_output_verify_config_block_crc_(cd, ebid, &status,
			&calculated_crc, &stored_crc);
	if (rc)
		goto exit;

	if (status) {
		rc = -EINVAL;
		cd->config_crc_fail_flag = true;
		tp_log_err("%s: crc fail ,status=%d\n",__func__,status);
		goto exit;
	} else {
		cd->config_crc_fail_flag = false;
	}

	si->ttconfig.crc = stored_crc;

exit:
	cyttsp5_hid_output_resume_scanning_(cd);
error:
	tp_log_debug( "%s: CRC: ebid:%d, crc:0x%04X\n",
			__func__, ebid, si->ttconfig.crc);
	return rc;
}

static int cyttsp5_check_and_deassert_int(struct cyttsp5_core_data *cd)
{
	u16 size;
	u8 buf[2];
	u8 *p;
	u8 retry = 3;
	int rc;

	do {
		rc = cyttsp5_adap_read_default(cd, buf, 2);
		if (rc < 0)
			return rc;
		size = get_unaligned_le16(&buf[0]);

		if (size == 2 || size == 0)
			return 0;

		p = kzalloc(size, GFP_KERNEL);
		if (!p)
			return -ENOMEM;

		rc = cyttsp5_adap_read_default(cd, p, size);
		kfree(p);
		if (rc < 0)
			return rc;
	} while (retry--);

	return -EINVAL;
}

static int cyttsp5_set_touch_mode(struct cyttsp5_core_data *cd,u32 value)
{
	int rc = -1;
	u8 param_id = 0x02;

	tp_log_debug("%s: value = %d\n",__func__, value);

	rc = cyttsp5_hid_output_set_param(cd, param_id, value);
	if (rc < 0) {
		tp_log_err("%s: cyttsp5_hid_output_set_param failed r=%d\n", __func__, rc);
		return rc;
	}

	return rc;
}

/*
 * Used to set TP IC
 * param_id: register address
 * size: size of the value
 * Notes:The parameters "size" and "param_id" need to be supplied by FAE
 */
static int cyttsp5_hid_output_set_param_by_size_(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value, u8 size)
{
	u8 write_buf[6];
	u8 *ptr = &write_buf[2];
	int rc = -1;
	int i = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SET_PARAM),
		.write_buf = write_buf,
	};

	write_buf[0] = param_id;
	write_buf[1] = size;

	for (i = 0; i < size; i++) {
		ptr[i] = value & 0xFF;
		value = value >> 8;
	}

	hid_output.write_length = 2 + size;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		tp_log_err("%s: cyttsp5_hid_send_output_and_wait_ failed\n", __func__);
		return rc;
	}

	if ((param_id != cd->response_buf[5]) || (size != cd->response_buf[6])) {
		tp_log_err("%s: param_id or size reponse failed\n", __func__);
		return -EPROTO;
	}

	return 0;
}

static int cyttsp5_hid_output_set_param_by_size(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value, u8 size)
{
	int rc = -1;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err("%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_set_param_by_size_(cd, param_id, value, size);

	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err("%s: fail to release exclusive\n", __func__);

	return rc;
}

static int set_effective_window(struct cyttsp5_core_data *cd,
		struct holster_mode effective_window_info)
{
	int ret = 0;
	int i = 0;
	/*
	 * x start	0x26
	 * x end	0x27
	 * y start	0x28
	 * y end	0x29
	 */
	u8 param_id[4] = {0x26, 0x27, 0x28, 0x29};
	u32 window_edge[4];

	window_edge[0] = effective_window_info.top_left_x0;
	window_edge[1] = effective_window_info.bottom_right_x1;
	window_edge[2] = effective_window_info.top_left_y0;
	window_edge[3] = effective_window_info.bottom_right_y1;
	tp_log_info("cyttsp5 %s (%d, %d) (%d, %d)\n", __func__,
			window_edge[0], window_edge[2], window_edge[1], window_edge[3]);

	for (i = 0; i < ARRAY_SIZE(window_edge); i++) {
		ret = cyttsp5_hid_output_set_param_by_size(cd, param_id[i], window_edge[i], sizeof(u16));
		if (ret < 0) {
			tp_log_err("%s: cyttsp5_hid_output_set_param %#x failed ret = %d\n", __func__, param_id[i], ret);
			return ret;
		}
	}

	tp_log_info("cyttsp5 %s success\n", __func__);

	return 0;
}

static int cyttsp5_set_holster_mode(struct cyttsp5_core_data *cd, bool enable)
{
	int ret = -1;
	u8 param_id = 0x25; /*used to set the holster mode*/

	if (enable) {
		ret = set_effective_window(cd, cyttsp5_holster_info); /* set the holster zone before enalbe holster mode */
		if (ret < 0) {
			tp_log_err("%s:holster window zone write error! ret = %d\n",
					__func__,ret);
			return ret;
		}
	}

	/*
	 * 0 disable holster mode
	 * 1 enable holster mode
	 */
	ret = cyttsp5_hid_output_set_param_by_size(cd, param_id, enable, sizeof(u8));
	if (ret < 0) {
		tp_log_err("%s: set holster mode = %d failed ret = %d\n", __func__, enable, ret);
		return ret;
	}
	tp_log_info("%s: set holster mode = %d success\n", __func__, enable);

	return ret;
}

static int cyttsp5_startup_(struct cyttsp5_core_data *cd, bool reset)
{
	int retry = CY_CORE_STARTUP_RETRY_COUNT;
	int rc;
	bool detected = false;

#ifdef TTHE_TUNER_SUPPORT
	tthe_print(cd, NULL, 0, "enter startup");
#endif

	cyttsp5_stop_wd_timer(cd);

	/* ESD may lead (buf[5] & 0x80 == 1), if this issue happened,
	   cd->transient_state will be set to 1 at cyttsp5_parse_input */
#ifdef CONFIG_HUAWEI_KERNEL
	if(cd->transient_state) {
		tp_log_err("%s %d:ESD error detected, going to repower ic\n", __func__, __LINE__);
		cd->transient_state = 0;
		cyttsp5_power_off(cd->dev, cd);
		mdelay(1000);
		cyttsp5_power_on(cd->dev, cd);
		mdelay(100);
	}
#endif /*#ifdef CONFIG_HUAWEI_KERNEL*/


reset:
	if (retry != CY_CORE_STARTUP_RETRY_COUNT)
		tp_log_debug( "%s: Retry %d\n", __func__,
			CY_CORE_STARTUP_RETRY_COUNT - retry);

	tp_log_warning("%s: start to check i2c.\n", __func__);
	rc = cyttsp5_check_and_deassert_int(cd);
	/* when fail to reset 3 times, re-power the ic */
	tp_log_info("%s:retry:%d, rc:%d\n", __func__, retry, rc);
	if (reset || retry != CY_CORE_STARTUP_RETRY_COUNT) {
		/* reset hardware */
		rc = cyttsp5_reset_and_wait(cd);
		if (rc < 0 && retry > 0) {
			tp_log_err( "%s: Error on h/w reset retry = %d,r = %d\n", __func__, retry, rc);
			retry = retry - 1;
			goto reset;
		}
	}

	/* if hardware rest 3 times, the ic is still not work,
	 * we re-power it */
	if ((0 == retry) && (rc < 0)) {
#ifdef CONFIG_HUAWEI_DSM
		g_tp_dsm_info.constraints_ESD_status = rc;
		cyttsp5_tp_report_dsm_err(cd->dev, DSM_TP_ESD_ERROR_NO, g_tp_dsm_info.constraints_ESD_status);
#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_info("%s: start to re-power\n", __func__);
		gpio_set_value(cd->cpdata->rst_gpio, 0);

#ifdef CONFIG_HUAWEI_KERNEL
		cyttsp5_power_off(cd->dev, cd);

		mdelay(CY_HID_REGULATOR_DELAYED_TIME);
		rc = cyttsp5_power_on(cd->dev, cd);
		if (rc < 0)
		{
			tp_log_err( "%s: failed to enable vbus_cypress,rc=%d\n", __func__, rc);
			goto exit;
		}

		mdelay(CY_HID_REGULATOR_DELAYED_TIME);
		gpio_set_value(cd->cpdata->rst_gpio, 1);
#endif /*#ifdef CONFIG_HUAWEI_KERNEL*/

		rc = cyttsp5_reset_and_wait(cd);
		if (rc < 0) {
			tp_log_err( "%s: Error on h/w reset2 r=%d\n", __func__, rc);
			goto exit;
		}
	}
	tp_log_info("%s: start to get hid descriptor.\n", __func__);
	rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
	if (rc < 0) {
		tp_log_err( "%s: Error on getting HID descriptor r=%d\n",
			__func__, rc);
		RETRY_OR_EXIT(retry--, reset, exit);
	}
	tp_log_info("%s: start to get mode.\n", __func__);
	cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);

	detected = true;

	/* Switch to bootloader mode to get Panel ID */
	if (cd->mode == CY_MODE_OPERATIONAL) {
		tp_log_info("%s: start to start bootloader.\n", __func__);
		rc = cyttsp5_hid_output_start_bootloader_(cd);
		if (rc < 0) {
			tp_log_err( "%s: Error on start bootloader r=%d\n",
				__func__, rc);
			RETRY_OR_EXIT(retry--, reset, exit);
		}
		tp_log_info("%s: Bootloader mode\n", __func__);
	}
	tp_log_info("%s: start to get panel id from bootloader mode.\n", __func__);
	cyttsp5_hid_output_bl_get_panel_id_(cd, &cd->panel_id);
	tp_log_info("%s: from bootloader Panel ID: 0x%02X\n", __func__, cd->panel_id);
	tp_log_info("%s: start to launch_app.\n", __func__);
	rc = cyttsp5_hid_output_bl_launch_app_(cd);
	if (rc < 0) {
		tp_log_err( "%s: Error on launch app r=%d\n",
			__func__, rc);
		RETRY_OR_EXIT(retry--, reset, exit);
	}

	tp_log_info("%s: start to get hid descriptor again.\n", __func__);
	rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
	if (rc < 0) {
		tp_log_err(
			"%s: Error on getting HID descriptor r=%d\n",
			__func__, rc);
		RETRY_OR_EXIT(retry--, reset, exit);
	}
	tp_log_info("%s: start to get mode again.\n", __func__);
	cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);
	if (cd->mode == CY_MODE_BOOTLOADER)
		RETRY_OR_EXIT(retry--, reset, exit);

	mutex_lock(&cd->system_lock);
	/* Read descriptor lengths */
	cd->hid_core.hid_report_desc_len =
		le16_to_cpu(cd->hid_desc.report_desc_len);
	cd->hid_core.hid_max_input_len =
		le16_to_cpu(cd->hid_desc.max_input_len);
	cd->hid_core.hid_max_output_len =
		le16_to_cpu(cd->hid_desc.max_output_len);

	cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);
	if (cd->mode == CY_MODE_OPERATIONAL)
		tp_log_info("%s: Operational mode\n", __func__);
	else if (cd->mode == CY_MODE_BOOTLOADER)
		tp_log_info("%s: Bootloader mode\n", __func__);
	else if (cd->mode == CY_MODE_UNKNOWN) {
		tp_log_err( "%s: Unknown mode\n", __func__);
		rc = -ENODEV;
		mutex_unlock(&cd->system_lock);
		RETRY_OR_EXIT(retry--, reset, exit);
	}
	mutex_unlock(&cd->system_lock);

	tp_log_info("%s: Reading report descriptor\n", __func__);
	rc = cyttsp5_get_report_descriptor_(cd);
	if (rc < 0) {
		tp_log_err( "%s: Error on getting report descriptor r=%d\n",
			__func__, rc);
		RETRY_OR_EXIT(retry--, reset, exit);
	}

	if (!cd->features.easywake)
		cd->easy_wakeup_gesture = CY_CORE_EWG_NONE;

	rc = cyttsp5_hid_output_get_sysinfo_(cd);
	if (rc) {
		tp_log_err( "%s: Error on getting sysinfo r=%d\n",
			__func__, rc);
		RETRY_OR_EXIT(retry--, reset, exit);
	}
	//set_tp_type(cd->panel_id);
	tp_log_info("cyttsp5 Protocol Version: %d.%d\n",
			cd->sysinfo.cydata.pip_ver_major,
			cd->sysinfo.cydata.pip_ver_minor);

	/* Read config version directly if PIP version < 1.2 */
	if (!IS_PIP_VER_GE(&cd->sysinfo, 1, 2)) {
		rc = cyttsp5_get_config_ver_(cd);
		if (rc)
			tp_log_err( "%s: failed to read config version rc=%d\n",
				__func__, rc);
	}

	tp_log_info("%s: start to get ic crc.\n", __func__);
	rc = cyttsp5_get_ic_crc_(cd, CY_TCH_PARM_EBID);
	if (rc)
		tp_log_err( "%s: failed to crc data rc=%d\n",
			__func__, rc);

	tp_log_info("%s: start to restore parameters.\n", __func__);
	rc = cyttsp5_restore_parameters_(cd);
	if (rc)
		tp_log_err( "%s: failed to restore parameters rc=%d\n",
			__func__, rc);

	/* attention startup */
	call_atten_cb(cd, CY_ATTEN_STARTUP, 0);
	tp_log_warning("%s: start up successful.\n",__func__);
exit:
	if (!rc)
		cd->startup_retry_count = 0;

	cyttsp5_start_wd_timer(cd);

	if (!detected){
		tp_log_err("%s: detect failed, detected = %d\n",__func__, detected);
		rc = -ENODEV;
	}

#ifdef TTHE_TUNER_SUPPORT
	tthe_print(cd, NULL, 0, "exit startup");
#endif

	return rc;
}

static int cyttsp5_startup(struct cyttsp5_core_data *cd, bool reset)
{
	int rc;
	int res = -1;

	mutex_lock(&cd->system_lock);
	cd->startup_state = STARTUP_RUNNING;
	mutex_unlock(&cd->system_lock);

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		goto exit;
	}

	rc = cyttsp5_startup_(cd, reset);

	if (release_exclusive(cd, cd->dev) < 0)
		/* Don't return fail code, mode is already changed. */
		tp_log_err( "%s: fail to release exclusive\n", __func__);
	else
		tp_log_debug("%s: pass release exclusive\n", __func__);
	if (cd->glove_mode_enabled) {
		res = cyttsp5_set_touch_mode(cd, FINGER_GLOVE_MODE);
		if (res < 0) {
			tp_log_info("%s: set glove mode failed, rc = %d.\n", __func__, res);
		} else {
			tp_log_info("%s: set glove mode success.\n", __func__);
		}
	}

	if (cd->holster_mode_enabled) {
		res = cyttsp5_set_holster_mode(cd, cd->holster_mode_enabled);
		if (res < 0) {
			tp_log_err("%s: set holster mode failed, rc = %d.\n", __func__, res);
		} else {
			tp_log_info("%s: set holster mode success.\n", __func__);
		}
	}

exit:
	mutex_lock(&cd->system_lock);
	cd->startup_state = STARTUP_NONE;
	mutex_unlock(&cd->system_lock);

	/* Wake the waiters for end of startup */
	wake_up(&cd->wait_q);

	return rc;
}

static void cyttsp5_startup_work_function(struct work_struct *work)
{
	struct cyttsp5_core_data *cd =  container_of(work,
		struct cyttsp5_core_data, startup_work);
	int rc;

	rc = cyttsp5_startup(cd, true);
	if (rc < 0)
		tp_log_err( "%s: Fail queued startup r=%d\n",
			__func__, rc);
}
#if defined(CONFIG_PM_RUNTIME)
static int cyttsp5_core_rt_suspend(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	tp_log_warning("%s %d:Enter rt suspend.\n", __func__, __LINE__);
	/* move to the end of core sleep */
	rc = cyttsp5_core_sleep(cd);
	if (rc < 0) {
		tp_log_err("%s: Error on sleep\n", __func__);
		return 0;
	}
	/* attention suspend: release fingers */
	call_atten_cb(cd, CY_ATTEN_SUSPEND, 0);

	tp_log_info("%s %d:Exit runtime suspend:success.\n", __func__, __LINE__);
	return 0;
}

static int cyttsp5_core_rt_resume(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	tp_log_warning("%s %d:Enter rt resume.\n", __func__, __LINE__);
	rc = cyttsp5_core_wake(cd);
	if (rc < 0) {
		tp_log_err("%s: Error on wake\n", __func__);
		/*no need to return negative number, when wake IC failed .Because negative number
		  leads to pm runtime unavailable*/
	}

	tp_log_info("%s %d:Exit runtime resume:success.\n", __func__, __LINE__);
	return 0;
}

#elif defined(CONFIG_PM_SLEEP)
static int cyttsp5_core_suspend(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	tp_log_info("%s %d: beging to suspend\n", __func__, __LINE__);
	cyttsp5_core_sleep(cd);

	if (!(cd->cpdata->flags & CY_CORE_FLAG_WAKE_ON_GESTURE))
		return 0;

	/*
	 * This will not prevent resume
	 * Required to prevent interrupts before i2c awake
	 */
	disable_irq(cd->irq);
	cd->irq_disabled = 1;

	if (device_may_wakeup(dev)) {
		tp_log_info( "%s Device MAY wakeup\n", __func__);
		if (!enable_irq_wake(cd->irq))
			cd->irq_wake = 1;
	} else {
		tp_log_info( "%s Device MAY NOT wakeup\n", __func__);
	}

	return 0;
}

static int cyttsp5_core_resume(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	tp_log_info("%s %d: beging to resume\n", __func__, __LINE__);
	if (!(cd->cpdata->flags & CY_CORE_FLAG_WAKE_ON_GESTURE))
		goto exit;

	/*
	 * I2C bus pm does not call suspend if device runtime suspended
	 * This flag is cover that case
	 */
	if (cd->irq_disabled) {
		enable_irq(cd->irq);
		cd->irq_disabled = 0;
	}

	if (device_may_wakeup(dev)) {
		tp_log_info( "%s Device MAY wakeup\n", __func__);
		if (cd->irq_wake) {
			disable_irq_wake(cd->irq);
			cd->irq_wake = 0;
		}
	} else {
		tp_log_info("%s Device MAY NOT wakeup\n", __func__);
	}

exit:
	cyttsp5_core_wake(cd);

	return 0;
}
#endif

#if NEED_SUSPEND_NOTIFIER
static int cyttsp5_pm_notifier(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct cyttsp5_core_data *cd = container_of(nb,
			struct cyttsp5_core_data, pm_notifier);

	if (action == PM_SUSPEND_PREPARE) {
		tp_log_debug( "%s: Suspend prepare\n", __func__);

		/*
		 * If not runtime PM suspended, either call runtime
		 * PM suspend callback or wait until it finishes
		 */
		if (!pm_runtime_suspended(cd->dev))
			pm_runtime_suspend(cd->dev);

		(void) cyttsp5_core_suspend(cd->dev);
	}

	return NOTIFY_DONE;
}
#endif

const struct dev_pm_ops cyttsp5_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(cyttsp5_core_rt_suspend, cyttsp5_core_rt_resume,
			NULL)
#elif CONFIG_PM_SLEEP
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_core_suspend, cyttsp5_core_resume)
#endif/*CONFIG_PM_RUNTIME*/
};
EXPORT_SYMBOL_GPL(cyttsp5_pm_ops);

/*
 * Show Firmware version via sysfs
 */
static ssize_t cyttsp5_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_cydata *cydata = &cd->sysinfo.cydata;

	return sprintf(buf,
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%08X\n"
		"%s: 0x%04X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n",
		"Firmware Major Version", cydata->fw_ver_major,
		"Firmware Minor Version", cydata->fw_ver_minor,
		"Revision Control Number", cydata->revctrl,
		"Firmware Configuration Version", cydata->fw_ver_conf,
		"Bootloader Major Version", cydata->bl_ver_major,
		"Bootloader Minor Version", cydata->bl_ver_minor,
		"Protocol Major Version", cydata->pip_ver_major,
		"Protocol Minor Version", cydata->pip_ver_minor);
}

/*
 * Show Driver version via sysfs
 */
static ssize_t cyttsp5_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		cy_driver_core_name, cy_driver_core_version,
		cy_driver_core_date);
}

/*
 * HW reset via sysfs
 */
static ssize_t cyttsp5_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = cyttsp5_startup(cd, true);
	if (rc < 0)
		tp_log_err( "%s: HW reset failed r=%d\n",
			__func__, rc);

	return size;
}

/*
 * Show IRQ status via sysfs
 */
static ssize_t cyttsp5_hw_irq_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int retval;

	if (cd->cpdata->irq_stat) {
		retval = cd->cpdata->irq_stat(cd->cpdata, dev);
		switch (retval) {
		case 0:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	}

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Function irq_stat() undefined.\n");
}

/*
 * Show IRQ enable/disable status via sysfs
 */
static ssize_t cyttsp5_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->irq_enabled)
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Driver interrupt is ENABLED\n");
	else
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Driver interrupt is DISABLED\n");
	mutex_unlock(&cd->system_lock);

	return ret;
}

/*
 * Enable/disable IRQ via sysfs
 */
static ssize_t cyttsp5_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		tp_log_err( "%s: Invalid value\n", __func__);
		goto cyttsp5_drv_irq_store_error_exit;
	}

	mutex_lock(&cd->system_lock);
	switch (value) {
	case 0:
		if (cd->irq_enabled) {
			cd->irq_enabled = false;
			/* Disable IRQ */
			disable_irq_nosync(cd->irq);
			tp_log_info( "%s: Driver IRQ now disabled\n",
				__func__);
		} else
			tp_log_info( "%s: Driver IRQ already disabled\n",
				__func__);
		break;

	case 1:
		if (cd->irq_enabled == false) {
			cd->irq_enabled = true;
			/* Enable IRQ */
			enable_irq(cd->irq);
			tp_log_info( "%s: Driver IRQ now enabled\n",
				__func__);
		} else
			tp_log_info( "%s: Driver IRQ already enabled\n",
				__func__);
		break;

	default:
		tp_log_err( "%s: Invalid value\n", __func__);
	}
	mutex_unlock(&(cd->system_lock));

cyttsp5_drv_irq_store_error_exit:

	return size;
}

/*
 * Debugging options via sysfs
 */
static ssize_t cyttsp5_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int rc;
	u8 return_data[8];

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		tp_log_err( "%s: Invalid value\n", __func__);
		goto cyttsp5_drv_debug_store_exit;
	}

	switch (value) {
	case CY_DBG_SUSPEND:
		tp_log_info( "%s: SUSPEND (cd=%p)\n", __func__, cd);
		rc = cyttsp5_core_sleep(cd);
		if (rc)
			tp_log_err( "%s: Suspend failed rc=%d\n",
				__func__, rc);
		else
			tp_log_info( "%s: Suspend succeeded\n", __func__);
		break;

	case CY_DBG_RESUME:
		tp_log_info( "%s: RESUME (cd=%p)\n", __func__, cd);
		rc = cyttsp5_core_wake(cd);
		if (rc)
			tp_log_err( "%s: Resume failed rc=%d\n",
				__func__, rc);
		else
			tp_log_info( "%s: Resume succeeded\n", __func__);
		break;
	case CY_DBG_SOFT_RESET:
		tp_log_info( "%s: SOFT RESET (cd=%p)\n", __func__, cd);
		rc = cyttsp5_hw_soft_reset(cd);
		break;
	case CY_DBG_RESET:
		tp_log_info( "%s: HARD RESET (cd=%p)\n", __func__, cd);
		rc = cyttsp5_hw_hard_reset(cd);
		break;
	case CY_DBG_HID_RESET:
		tp_log_info( "%s: hid_reset (cd=%p)\n", __func__, cd);
		cyttsp5_hid_cmd_reset(cd);
		break;
	case CY_DBG_HID_SET_POWER_ON:
		tp_log_info( "%s: hid_set_power_on (cd=%p)\n", __func__, cd);
		cyttsp5_hid_cmd_set_power(cd, HID_POWER_ON);
		cyttsp5_start_wd_timer(cd);
		break;
	case CY_DBG_HID_SET_POWER_SLEEP:
		tp_log_info( "%s: hid_set_power_off (cd=%p)\n", __func__, cd);
		cyttsp5_stop_wd_timer(cd);
		cyttsp5_hid_cmd_set_power(cd, HID_POWER_SLEEP);
		break;
	case CY_DBG_HID_NULL:
		tp_log_info( "%s: hid_null (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_null(cd);
		break;
	case CY_DBG_HID_ENTER_BL:
		tp_log_info( "%s: start_bootloader (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_start_bootloader(cd);
		break;
	case CY_DBG_HID_SYSINFO:
		tp_log_info( "%s: get_sysinfo (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_get_sysinfo(cd);
		break;
	case CY_DBG_HID_SUSPEND_SCAN:
		tp_log_info( "%s: suspend_scanning (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_suspend_scanning(cd);
		break;
	case CY_DBG_HID_RESUME_SCAN:
		tp_log_info( "%s: resume_scanning (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_resume_scanning(cd);
		break;
	case CY_DBG_HID_STOP_WD:
		tp_log_info( "%s: stop watchdog (cd=%p)\n", __func__, cd);
		cyttsp5_stop_wd_timer(cd);
		break;
	case CY_DBG_HID_START_WD:
		tp_log_info( "%s: start watchdog (cd=%p)\n", __func__, cd);
		cyttsp5_start_wd_timer(cd);
		break;
	case HID_OUTPUT_BL_VERIFY_APP_INTEGRITY:
		tp_log_info( "%s: verify app integ (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_verify_app_integrity(cd, &return_data[0]);
		break;
	case HID_OUTPUT_BL_GET_INFO:
		tp_log_info( "%s: bl get info (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_get_information(cd, return_data);
		break;
	case HID_OUTPUT_BL_PROGRAM_AND_VERIFY:
		tp_log_info( "%s: program and verify (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_program_and_verify(cd, 0, NULL);
		break;
	case HID_OUTPUT_BL_LAUNCH_APP:
		tp_log_info( "%s: launch app (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_launch_app(cd);
		break;
	case HID_OUTPUT_BL_INITIATE_BL:
		tp_log_info( "%s: initiate bl (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_initiate_bl(cd, 0, NULL, 0, NULL);
		break;
	default:
		tp_log_err( "%s: Invalid value\n", __func__);
	}

cyttsp5_drv_debug_store_exit:
	return size;
}

/*
 * Show system status on deep sleep status via sysfs
 */
static ssize_t cyttsp5_sleep_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_ON)
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "off\n");
	else
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "on\n");
	mutex_unlock(&cd->system_lock);

	return ret;
}

static ssize_t cyttsp5_easy_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;
	int i;

	mutex_lock(&cd->system_lock);
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "0x%02X\n", cd->easy_wakeup_gesture);
	for(i = 0;i < GESTURE_MAX ;i++)
	{
		ret += snprintf(&buf[ret], PAGE_SIZE, "%s %d : %d\n","gesture",i,cyttsp_gesture_count[i]);
	}
	mutex_unlock(&cd->system_lock);
	tp_log_info("%s %d:easy_wakeup_gesture:0x%x\n", __func__, __LINE__, cd->easy_wakeup_gesture);
	return ret;
}

static ssize_t cyttsp5_easy_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (!(cd->cpdata->flags & CY_CORE_FLAG_WAKE_ON_GESTURE)) {
		tp_log_err("%s %d:check flag fail, flag = %d\n",
					__func__, __LINE__, cd->cpdata->flags);
		return -EINVAL;
	}

	if (!cd->features.easywake) {
		tp_log_err("%s %d:not suppert easy wakeup, flag = %d\n",
					__func__, __LINE__, cd->features.easywake);
		return -EINVAL;
	}

	/* open/close easy wakeup function when system is sleep will make touch panel do not work.
	   so, when system is sleep, stop to set easy wakeup mode */
	if (cd->sleep_state != SS_SLEEP_OFF) {
		tp_log_err("%s,system is not awake, can not set gesture, state:%d\n",
			__func__, cd->sleep_state);
		return -EINVAL;
	}

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		tp_log_err("%s %d:Parse input value fail, value: %s, ret:%d\n",
					__func__, __LINE__, buf, ret);
		return -EINVAL;
	}

	tp_log_info("%s %d:Input value: %lu\n", __func__, __LINE__, value);
	if (value > 0xFFFF) {
		tp_log_err("%s %d:Error value: %lu\n", __func__, __LINE__, value);
		return -EINVAL;
	}

	pm_runtime_get_sync(dev);

	mutex_lock(&cd->system_lock);
	if (cd->sysinfo.ready && IS_PIP_VER_GE(&cd->sysinfo, 1, 2)) {
		cd->easy_wakeup_gesture = (unsigned int)value;
	} else {
		tp_log_err("%s %d:easy wakeup Error value: %d\n", __func__, __LINE__,
					cd->sysinfo.ready);
		tp_log_err("%s %d:easy wakeup Error value: %d\n", __func__, __LINE__,
					IS_PIP_VER_GE(&cd->sysinfo, 1, 2));
		ret = -ENODEV;
	}
	mutex_unlock(&cd->system_lock);

	pm_runtime_put(dev);

	if (ret) {
		tp_log_err("%s %d:end with error NO.: %d\n", __func__, __LINE__, ret);
		return ret;
	}
	return size;
}

/*Show gesture position via sysfs*/
static ssize_t cyttsp5_easy_wakeup_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int i = 0;
	int print_idx = 0;

	mutex_lock(&cd->system_lock);

	if (gesture_id == CYTTSP5_GESTURE_DOUBLE_CLICK) {
		for (i = 0; i < CYTTSP5_CLICK_LOCUS_NUM; i++){
			print_idx += snprintf(buf + print_idx, CY_MAX_PRBUF_SIZE - print_idx,
					"%04x%04x",
					cd->gest_pos[i].x,
					cd->gest_pos[i].y);
		}

		print_idx += snprintf(buf + print_idx, CY_MAX_PRBUF_SIZE - print_idx, "\n");

	}

	if (gesture_id == CYTTSP5_GESTURE_LETTER_C || gesture_id == CYTTSP5_GESTURE_LETTER_E ||
	    gesture_id == CYTTSP5_GESTURE_LETTER_M || gesture_id == CYTTSP5_GESTURE_LETTER_W) {

		for(i = 0; i < CYTTSP5_LETTER_LOCUS_NUM; i++){
			print_idx += snprintf(buf + print_idx, CY_MAX_PRBUF_SIZE - print_idx,
					"%04x%04x",
					cd->gest_pos[i].x,
					cd->gest_pos[i].y);
		}

		print_idx += snprintf(buf + print_idx, CY_MAX_PRBUF_SIZE - print_idx, "\n");
		tp_log_info("%s:easy_wakeup_position:%s", __func__,buf);
	}

	mutex_unlock(&cd->system_lock);

	return print_idx;
}

/* Show Panel ID via sysfs */
static ssize_t cyttsp5_panel_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "0x%02X\n",
			cd->panel_id);
	return ret;
}

static struct device_attribute attributes[] = {
	__ATTR(ic_ver, S_IRUGO, cyttsp5_ic_ver_show, NULL),
	__ATTR(drv_ver, S_IRUGO, cyttsp5_drv_ver_show, NULL),
	__ATTR(hw_reset, S_IWUSR, NULL, cyttsp5_hw_reset_store),
	__ATTR(hw_irq_stat, S_IRUSR, cyttsp5_hw_irq_stat_show, NULL),
	__ATTR(drv_irq, S_IRUSR | S_IWUSR, cyttsp5_drv_irq_show,
		cyttsp5_drv_irq_store),
	__ATTR(drv_debug, S_IWUSR, NULL, cyttsp5_drv_debug_store),
	__ATTR(sleep_status, S_IRUSR, cyttsp5_sleep_status_show, NULL),
	__ATTR(easy_wakeup_gesture, S_IRUSR | S_IWUSR,
		cyttsp5_easy_wakeup_gesture_show,
		cyttsp5_easy_wakeup_gesture_store),
	__ATTR(easy_wakeup_position, S_IRUGO, cyttsp5_easy_wakeup_position_show, NULL),
	__ATTR(panel_id, S_IRUGO, cyttsp5_panel_id_show, NULL),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		if (device_create_file(dev, attributes + i)) {
			tp_log_err("%s %d:%s create fail\n", __func__,
							__LINE__, attributes[i].attr.name);
			goto undo;
		}
	}

	return 0;

undo:
	for (i--; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	tp_log_err( "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

#ifdef TTHE_TUNER_SUPPORT
static int tthe_debugfs_open(struct inode *inode, struct file *filp)
{
	struct cyttsp5_core_data *cd = inode->i_private;

	filp->private_data = inode->i_private;

	if (cd->tthe_buf)
		return -EBUSY;

	cd->tthe_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (!cd->tthe_buf)
		return -ENOMEM;

	return 0;
}

static int tthe_debugfs_close(struct inode *inode, struct file *filp)
{
	struct cyttsp5_core_data *cd = filp->private_data;

	filp->private_data = NULL;

	kfree(cd->tthe_buf);
	cd->tthe_buf = NULL;

	return 0;
}

static ssize_t tthe_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_core_data *cd = filp->private_data;
	int size;
	int ret;

	wait_event_interruptible(cd->wait_q,
			cd->tthe_buf_len != 0 || cd->tthe_exit);
	mutex_lock(&cd->tthe_lock);
	if (cd->tthe_exit) {
		mutex_unlock(&cd->tthe_lock);
		return 0;
	}
	if (count > cd->tthe_buf_len)
		size = cd->tthe_buf_len;
	else
		size = count;
	if (!size) {
		mutex_unlock(&cd->tthe_lock);
		return 0;
	}

	ret = copy_to_user(buf, cd->tthe_buf, cd->tthe_buf_len);
	if (ret == size)
		return -EFAULT;
	size -= ret;
	cd->tthe_buf_len -= size;
	mutex_unlock(&cd->tthe_lock);
	*ppos += size;
	return size;
}

static const struct file_operations tthe_debugfs_fops = {
	.open = tthe_debugfs_open,
	.release = tthe_debugfs_close,
	.read = tthe_debugfs_read,
};
#endif

static struct cyttsp5_core_nonhid_cmd _cyttsp5_core_nonhid_cmd = {
	.start_bl = _cyttsp5_request_hid_output_start_bl,
	.suspend_scanning = _cyttsp5_request_hid_output_suspend_scanning,
	.resume_scanning = _cyttsp5_request_hid_output_resume_scanning,
	.get_param = _cyttsp5_request_hid_output_get_param,
	.set_param = _cyttsp5_request_hid_output_set_param,
	.verify_config_block_crc =
		_cyttsp5_request_hid_output_verify_config_block_crc,
	.get_config_row_size = _cyttsp5_request_hid_output_get_config_row_size,
	.get_data_structure = _cyttsp5_request_hid_output_get_data_structure,
	.run_selftest = _cyttsp5_request_hid_output_run_selftest,
	.get_selftest_result = _cyttsp5_request_hid_output_get_selftest_result,
	.calibrate_idacs = _cyttsp5_request_hid_output_calibrate_idacs,
	.initialize_baselines =
		_cyttsp5_request_hid_output_initialize_baselines,
	.exec_panel_scan = _cyttsp5_request_hid_output_exec_panel_scan,
	.retrieve_panel_scan = _cyttsp5_request_hid_output_retrieve_panel_scan,
	.write_conf_block = _cyttsp5_request_hid_output_write_conf_block,
	.user_cmd = _cyttsp5_request_hid_output_user_cmd,
	.get_bl_info = _cyttsp5_request_hid_output_bl_get_information,
	.initiate_bl = _cyttsp5_request_hid_output_bl_initiate_bl,
	.launch_app = _cyttsp5_request_hid_output_launch_app,
	.prog_and_verify = _cyttsp5_request_hid_output_bl_program_and_verify,
	.verify_app_integrity =
		_cyttsp5_request_hid_output_bl_verify_app_integrity,
     //.get_panel_id = _cyttsp5_request_hid_output_bl_get_panel_id,
    .get_panel_id = cyttsp5_hid_output_get_panel_id,
};

static struct cyttsp5_core_commands _cyttsp5_core_commands = {
	.subscribe_attention = _cyttsp5_subscribe_attention,
	.unsubscribe_attention = _cyttsp5_unsubscribe_attention,
	.request_exclusive = _cyttsp5_request_exclusive,
	.release_exclusive = _cyttsp5_release_exclusive,
	.request_reset = _cyttsp5_request_reset,
	.request_restart = _cyttsp5_request_restart,
	.request_sysinfo = _cyttsp5_request_sysinfo,
	.request_loader_pdata = _cyttsp5_request_loader_pdata,
	.request_stop_wd = _cyttsp5_request_stop_wd,
	.request_get_hid_desc = _cyttsp5_request_get_hid_desc,
	.request_get_mode = _cyttsp5_request_get_mode,
#ifdef TTHE_TUNER_SUPPORT
	.request_tthe_print = _cyttsp5_request_tthe_print,
#endif
	.nonhid_cmd = &_cyttsp5_core_nonhid_cmd,
};

struct cyttsp5_core_commands *cyttsp5_get_commands(void)
{
	return &_cyttsp5_core_commands;
}
EXPORT_SYMBOL_GPL(cyttsp5_get_commands);

static LIST_HEAD(core_list);
static int core_number;
struct cyttsp5_core_data *cyttsp5_get_core_data(char *id)
{
	struct cyttsp5_core_data *d;

	list_for_each_entry(d, &core_list, node)
		if (!strncmp(d->core_id, id, 20))
			return d;
	return NULL;
}
EXPORT_SYMBOL_GPL(cyttsp5_get_core_data);

static void cyttsp5_add_core(struct device *dev)
{
	struct cyttsp5_core_data *d;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	list_for_each_entry(d, &core_list, node)
		if (d->dev == dev)
			return;

	list_add(&cd->node, &core_list);
}

static void cyttsp5_del_core(struct device *dev)
{
	struct cyttsp5_core_data *d, *d_n;

	list_for_each_entry_safe(d, d_n, &core_list, node) {
		if (d->dev == dev) {
			list_del(&d->node);
			return;
		}
	}
}

static inline ssize_t cyttsp5_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	tp_log_info("%s Attempted to write to read-only attribute %s\n", __func__, attr->attr.name);
	return -EPERM;
}
#ifdef CONFIG_HUAWEI_KERNEL
static ssize_t hw_cyttsp5_easy_wakeup_position_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct device *cdev = cyttsp5_core_dev;
	if (!cdev){
		tp_log_err("%s: device is null \n", __func__);
		return -EINVAL;
	}

	return cyttsp5_easy_wakeup_position_show(cdev, NULL, buf);
}

static ssize_t hw_cyttsp5_easy_wakeup_position_store(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct device *cdev = cyttsp5_core_dev;
	if (!cdev){
		tp_log_err("%s: device is null \n", __func__);
		return -EINVAL;
	}

	return cyttsp5_store_error(cdev, NULL, buf, size);
}

static inline ssize_t hw_cyttsp5_store_error(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct device *cdev = cyttsp5_core_dev;
	if (!cdev){
		tp_log_err("%s: device is null", __func__);
		return -EINVAL;
	}

	return cyttsp5_store_error(cdev, NULL, buf, size);
}

static ssize_t cyttsp5_easy_wakeup_supported_gestures_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	tp_log_info("%s: easy_wakeup_supported_gestures=0x%04x", __func__,
					cd->easy_wakeup_supported_gestures);
	ret = snprintf(buf, PAGE_SIZE, "0x%04x\n", cd->easy_wakeup_supported_gestures);
	mutex_unlock(&cd->system_lock);
	tp_log_info("%s:easy_wakeup_supported_gestures:%s", __func__,buf);
	return ret;
}

static ssize_t hw_cyttsp5_glove_func_store(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	int rc = 0;
	int glove_mode;
	struct device *cdev = cyttsp5_core_dev;
	struct cyttsp5_core_data *cd = dev_get_drvdata(cdev);

	rc = sscanf(buf, "%d", &glove_mode);
	if(rc != 1){
		tp_log_err("%s:input value is error, rc = %d.\n", __func__, rc);
		return -EINVAL;
	}

	tp_log_info("%s:set glove_mode  = %d.\n", __func__, glove_mode);

	if (cd->glove_mode_enabled == glove_mode) {
		tp_log_info("%s:TP is already in glove_mode  = %d.\n", __func__, glove_mode);
		goto out;
	}

	pm_runtime_get_sync(cdev);

	if (glove_mode) {
		rc = cyttsp5_set_touch_mode(cd, FINGER_GLOVE_MODE);
	} else {
		rc = cyttsp5_set_touch_mode(cd, FINGER_ONLY_MODE);
	}

	pm_runtime_put(cdev);

	if (rc < 0) {
		tp_log_info("%s: set mode %d failed, rc = %d.\n", __func__, glove_mode, rc);
		goto out;
	} else {
		cd->glove_mode_enabled = glove_mode;
		rc = 0;
	}

	tp_log_debug("%s:set TP glove mode done!\n", __func__);

out:
	return size;
}

static ssize_t hw_cyttsp5_glove_func_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct device *cdev = cyttsp5_core_dev;
	struct cyttsp5_core_data *cd = dev_get_drvdata(cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", cd->glove_mode_enabled);
}

static ssize_t hw_cyttsp5_holster_func_store(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	int enable = 0;
	int x0 = 0;
	int y0 = 0;
	int x1 = 0;
	int y1 = 0;
	struct device *cdev = cyttsp5_core_dev;
	struct cyttsp5_core_data *cd = dev_get_drvdata(cdev);

	ret = sscanf(buf, "%d %d %d %d %d", &enable, &x0, &y0, &x1, &y1);
	if(!ret){
		tp_log_err("%s:input value is error, rc = %d.\n", __func__, ret);
		ret = -EINVAL;
		goto out;
	}

	tp_log_info("%s:sscanf value is enable=%d, (%d,%d), (%d,%d)\n",__func__, enable, x0, y0, x1, y1);

	if (enable && ((x0 < 0) || (y0 < 0) || (x1 <= x0) || (y1 <= y0))) {
		tp_log_err("%s:invalid value is %d (%d,%d), (%d,%d)\n",__func__, enable, x0, y0, x1, y1);
		ret = -EINVAL;
		goto out;
	}

	/* init holster window info*/
	cyttsp5_holster_info.top_left_x0 = x0;
	cyttsp5_holster_info.top_left_y0 = y0;
	cyttsp5_holster_info.bottom_right_x1 = x1;
	cyttsp5_holster_info.bottom_right_y1 = y1;
	cyttsp5_holster_info.holster_enable = enable;

	pm_runtime_get_sync(cdev);
	ret = cyttsp5_set_holster_mode(cd, enable);
	pm_runtime_put(cdev);
	if (ret < 0) {
		tp_log_err("%s: set mode %d failed, rc = %d.\n", __func__, enable, ret);
		goto out;
	} else {
		cd->holster_mode_enabled = enable;
		ret = 0;
	}
	tp_log_info("%s:set TP holster mode  = %d done!\n", __func__, enable);

out:
	if (ret < 0)
		return ret;

	return size;
}

static ssize_t hw_cyttsp5_holster_func_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	tp_log_info("%s: come\n", __func__);
	return snprintf(buf,PAGE_SIZE,"%ld %d %d %d %d\n",
			cyttsp5_holster_info.holster_enable, cyttsp5_holster_info.top_left_x0,
			cyttsp5_holster_info.top_left_y0, cyttsp5_holster_info.bottom_right_x1,
			cyttsp5_holster_info.bottom_right_y1);
}

static ssize_t hw_cyttsp5_easy_wakeup_gesture_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct device *cdev = cyttsp5_core_dev;
	if (!cdev){
		tp_log_err("%s: device is null", __func__);
		return -EINVAL;
	}

	return cyttsp5_easy_wakeup_gesture_show(cdev, NULL, buf);
}

static ssize_t hw_cyttsp5_easy_wakeup_gesture_store(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct device *cdev = cyttsp5_core_dev;
	if (!cdev){
		tp_log_err("%s: device is null", __func__);
		return -EINVAL;
	}

	return cyttsp5_easy_wakeup_gesture_store(cdev, NULL, buf, size);
}

static ssize_t hw_cyttsp5_easy_wakeup_supported_gestures_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct device *cdev = cyttsp5_core_dev;
	if (!cdev){
		tp_log_err("%s: device is null", __func__);
		return -EINVAL;
	}

	return cyttsp5_easy_wakeup_supported_gestures_show(cdev, NULL, buf);
}

static struct kobj_attribute easy_wakeup_gesture = {
	.attr = {.name = "easy_wakeup_gesture", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
	.show = hw_cyttsp5_easy_wakeup_gesture_show,
	.store = hw_cyttsp5_easy_wakeup_gesture_store,
};

static struct kobj_attribute easy_wakeup_supported_gestures = {
	.attr = {.name = "easy_wakeup_supported_gestures", .mode = S_IRUGO},
	.show = hw_cyttsp5_easy_wakeup_supported_gestures_show,
	.store = hw_cyttsp5_store_error,
};

/* easy wakeup position file node create struct */
static struct kobj_attribute easy_wakeup_position = {
	.attr = {.name = "easy_wakeup_position", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
	.show = hw_cyttsp5_easy_wakeup_position_show,
	.store = hw_cyttsp5_easy_wakeup_position_store,
};

static struct kobj_attribute glove_func = {
	.attr = {.name = "signal_disparity", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
	.show = hw_cyttsp5_glove_func_show,
	.store = hw_cyttsp5_glove_func_store,
};

static struct kobj_attribute holster_func = {
	.attr = {.name = "holster_touch_window", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
	.show = hw_cyttsp5_holster_func_show,
	.store = hw_cyttsp5_holster_func_store,
};

static int add_easy_wakeup_interfaces(struct device *dev)
{
	int error = 0;
	struct kobject *properties_kobj;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	properties_kobj = tp_get_touch_screen_obj();
	if( NULL == properties_kobj )
	{
		tp_log_err("%s: Error, get kobj failed!\n", __func__);
		return -1;
	}

	if(0 != cd->easy_wakeup_supported_gestures)
	{
		/*add the node easy_wakeup_gesture_supported for apk to read*/
		error = sysfs_create_file(properties_kobj, &easy_wakeup_supported_gestures.attr);
		if (error)
		{
			kobject_put(properties_kobj);
			tp_log_err("%s: cyttsp5_easy_wakeup_gesture_supported create file error\n", __func__);
			return -ENODEV;
		}

		/*add the node easy_wakeup_gesture apk to write*/
		error = sysfs_create_file(properties_kobj, &easy_wakeup_gesture.attr);
		if (error)
		{
			kobject_put(properties_kobj);
			tp_log_err("%s: cyttsp5_easy_wakeup_gesture create file error\n", __func__);
			return -ENODEV;
		}

		/*add the node easy_wakeup_position for apk to write*/
		error = sysfs_create_file(properties_kobj, &easy_wakeup_position.attr);
		if (error)
		{
			kobject_put(properties_kobj);
			tp_log_err("%s: easy_wakeup_position create file error\n", __func__);
			return -ENODEV;
		}
	}

	return 0;
}
static int add_glove_interface(struct device *dev)
{
	int rc = 0;
	struct kobject *glove_kobj = NULL;

	glove_kobj = tp_get_glove_func_obj();
	if( NULL == glove_kobj ) {
		tp_log_err("%s: Error, get glove kobj failed!\n", __func__);
		return -EINVAL;
	}

	/* add the node glove_func/signal_disparity for apk to write*/
	rc = sysfs_create_file(glove_kobj, &glove_func.attr);
	if (rc) {
		kobject_put(glove_kobj);
		rc = sysfs_create_file(glove_kobj, &glove_func.attr);
		if (rc) {
			kobject_put(glove_kobj);
			tp_log_err("%s: glove_func create file error\n", __func__);
			return -ENODEV;
		}
	}

	return 0;
}

static int add_holster_interface(struct device *dev)
{
	int rc = 0;
	struct kobject *properties_kobj = NULL;

	properties_kobj = tp_get_touch_screen_obj();
	if( NULL == properties_kobj )
	{
		tp_log_err("%s: Error, get kobj failed!\n", __func__);
		return -EINVAL;
	}

	/* add the node holster_touch_window for apk to write*/
	rc = sysfs_create_file(properties_kobj, &holster_func.attr);
	if (rc) {
		kobject_put(properties_kobj);
		rc = sysfs_create_file(properties_kobj, &holster_func.attr);
		if (rc) {
			kobject_put(properties_kobj);
			tp_log_err("%s: holster_func create file error\n", __func__);
			return -ENODEV;
		}
	}

	return 0;
}
#endif /* CONFIG_HUAWEI_KERNEL*/


#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
static void cyttsp5_pm_runtime_get(struct cyttsp5_core_data *cd)
{
	int i;

	tp_log_debug( "%s:system count:%d,device count:%d,usage count:%d\n", __func__,
				cd->dev->power.usage_count.counter,
				cd->number_of_open_input_device,
				cd->pm_runtime_usage_count);

	for (i = 0; i < cd->number_of_open_input_device; i++) {
		pm_runtime_get_sync(cd->dev);
		cd->pm_runtime_usage_count++;
	}

	tp_log_debug( "%s:system count:%d,device count:%d,usage count:%d\n", __func__,
				cd->dev->power.usage_count.counter,
				cd->number_of_open_input_device,
				cd->pm_runtime_usage_count);
}

static void cyttsp5_pm_runtime_put(struct cyttsp5_core_data *cd)
{
	int i;

	tp_log_debug( "%s:system count:%d,device count:%d,usage count:%d\n", __func__,
			cd->dev->power.usage_count.counter,
			cd->number_of_open_input_device,
			cd->pm_runtime_usage_count);
	for (i = 0; i < cd->number_of_open_input_device; i++) {
		pm_runtime_put(cd->dev);
		cd->pm_runtime_usage_count--;
	}

	tp_log_debug( "%s:system count:%d,device count:%d,usage count:%d\n", __func__,
		cd->dev->power.usage_count.counter,
		cd->number_of_open_input_device,
		cd->pm_runtime_usage_count);
}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp5_early_suspend(struct early_suspend *h)
{
	struct cyttsp5_core_data *cd =
		container_of(h, struct cyttsp5_core_data, es);

	cyttsp5_pm_runtime_put(cd);
}

static void cyttsp5_late_resume(struct early_suspend *h)
{
	struct cyttsp5_core_data *cd =
		container_of(h, struct cyttsp5_core_data, es);

	cyttsp5_pm_runtime_get(cd);
}

static void cyttsp5_setup_early_suspend(struct cyttsp5_core_data *cd)
{
	cd->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	cd->es.suspend = cyttsp5_early_suspend;
	cd->es.resume = cyttsp5_late_resume;

	register_early_suspend(&cd->es);
}
#elif defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct cyttsp5_core_data *cd =
		container_of(self, struct cyttsp5_core_data, fb_notifier);
	struct fb_event *evdata = data;
	int *blank;
	if ((event == FB_EVENT_BLANK) && evdata && evdata->data && cd ){
		blank = evdata->data;
		if ((*blank == FB_BLANK_UNBLANK)
			&& (CY_STATE_RESUME != cd->suspend_resume_flag)) {
			tp_log_info("%s: UNBLANK!\n", __func__);
			cd->suspend_resume_flag = CY_STATE_RESUME;
			cyttsp5_pm_runtime_get(cd);
		} else if (((*blank == FB_BLANK_POWERDOWN)
				|| (*blank == FB_BLANK_VSYNC_SUSPEND))
				&& (CY_STATE_SUSPEND != cd->suspend_resume_flag)) {

			tp_log_info("%s: POWERDOWN! or VSYNC_SUSPEND\n", __func__);
			cd->suspend_resume_flag = CY_STATE_SUSPEND;
			cyttsp5_pm_runtime_put(cd);
		}
	}
	return 0;
}

static void cyttsp5_setup_fb_notifier(struct cyttsp5_core_data *cd)
{
	int rc;

	cd->fb_notifier.notifier_call = fb_notifier_callback;

	rc = fb_register_client(&cd->fb_notifier);
	if (rc)
		tp_log_err( "Unable to register fb_notifier: %d\n", rc);
}
#endif
int cyttsp5_read_mfg_val0_(struct cyttsp5_core_data *cd)
{
	u16 crc;
	int rc;
	u8 mfg_val0[5];

	rc = cyttsp5_hid_output_suspend_scanning_(cd);
	if (rc){
		tp_log_err("%s: failed to suspend scanning\n", __func__);
		goto error;
	}
	rc = cyttsp5_hid_output_read_conf_block_(cd,CY_TPCOLOR_ROW,
		CY_TPCOLOR_READ_SIZE,CY_MDATA_EBID, mfg_val0, &crc);
	if (rc) {
		tp_log_err("%s: failed to read design data size\n", __func__);
		goto exit;
	}

	tp_color_data = mfg_val0[4];
	tp_log_warning("%s: tp_color_data = 0x%x\n", __func__, tp_color_data);
exit:
	cyttsp5_hid_output_resume_scanning_(cd);
error:
	return rc;
}
int cyttsp5_hid_output_write_mfg_val0(struct cyttsp5_core_data *cd, u8 val)
{
	int rc;
	int ret;
	u8 tmp_buf[128]={0};
	u16 crc;
	u16 calculated_crc;
	u16 stored_crc;
	u8 status;
	u8 security_key_tmp[8] = {0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A};
	u16 actual_write_len;
	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		tp_log_err("%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}
	rc = cyttsp5_hid_output_suspend_scanning_(cd);
	if(rc){
		tp_log_err("%s: Error on suspend scanning\n", __func__);
		goto error;
	}

	rc=cyttsp5_hid_output_read_conf_block_(cd,CY_TPCOLOR_ROW,
		CY_TPCOLOR_WRITE_SIZE,CY_MDATA_EBID,&tmp_buf[0],&crc);
	if (rc) {
		tp_log_err("%s: Error on read design data size\n", __func__);
		goto resume_scan;
	}
	/*change mfg_val0 value here*/
	tmp_buf[4] = val;
	rc = cyttsp5_hid_output_write_conf_block_(cd,CY_TPCOLOR_ROW,
		CY_TPCOLOR_WRITE_SIZE,CY_MDATA_EBID,&tmp_buf[0],&security_key_tmp[0],&actual_write_len);
	if(rc){
		tp_log_err("%s: Error on write_conf_block\n", __func__);
		goto resume_scan;
	}

	rc=cyttsp5_hid_output_verify_config_block_crc_(cd,CY_MDATA_EBID,
		&status,&calculated_crc,&stored_crc);
	if (rc) {
		tp_log_err("%s: Error onverify_config_block_crc\n", __func__);
		goto resume_scan;
	}
	/*change the crc of the mfg block here*/
	tmp_buf[126]=(calculated_crc & 0xff);
	tmp_buf[127]=(calculated_crc & 0xff00)>>CY_TPCOLOR_WRITE_OFFSET;

	rc=cyttsp5_hid_output_write_conf_block_(cd,CY_TPCOLOR_ROW,
		CY_TPCOLOR_WRITE_SIZE,CY_MDATA_EBID,&tmp_buf[0],&security_key_tmp[0],&actual_write_len);
	if(rc){
		tp_log_err("%s: Error on write_conf_block after crc changed\n", __func__);
		goto resume_scan;
	}
	tp_color_data = val;
resume_scan:
	ret=cyttsp5_hid_output_resume_scanning_(cd);
	if(ret){
		tp_log_err("%s: Error on resume scanning\n", __func__);
	}
error:
	if (release_exclusive(cd, cd->dev) < 0)
		tp_log_err("%s: fail to release exclusive\n", __func__);
	return rc;
}

static int cyttsp5_setup_irq_gpio(struct cyttsp5_core_data *cd)
{
	struct device *dev = cd->dev;
	unsigned long irq_flags;
	int rc;

	/* Initialize IRQ */
	cd->irq = gpio_to_irq(cd->cpdata->irq_gpio);
	if (cd->irq < 0) {
		tp_log_err("%s %d:error irq:%d\n", __func__, __LINE__, cd->irq);
		return -EINVAL;
	}

	cd->irq_enabled = true;

	tp_log_debug("%s %d: initialize threaded irq=%d\n", __func__, __LINE__, cd->irq);
	if (cd->cpdata->level_irq_udelay > 0) {
		/* use level triggered interrupts */
		irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	} else {
		/* use edge triggered interrupts */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	}

	rc = request_threaded_irq(cd->irq, NULL, cyttsp5_irq, irq_flags,
		dev_name(dev), cd);
	if (rc < 0) {
		tp_log_err("%s %d: Error, could not request irq, rc = %d\n",
					__func__, __LINE__, rc);
	}
	return rc;
}

#ifdef CONFIG_HWATCH_POWER
static int cyttsp5_power_init(struct device* dev, struct cyttsp5_core_data *pcore_data)
{
	int rc = 0;
	struct cyttsp5_power_config *cpower = NULL;

	if (NULL == dev || NULL == pcore_data || NULL == pcore_data->cpdata) {
		tp_log_err("%s %d: input parameter missing!\n", __func__, __LINE__);
		return  -EINVAL;
	}

	cpower = pcore_data->cpdata->power_config;
	if (NULL == cpower) {
		tp_log_err("%s %d: Power config is NULL!\n",__func__,__LINE__);
		return -EINVAL;
	}

	/* vdd init */
	if (CY_POWER_PMU == cpower->vdd_type) {
		tp_log_info("%s %d: get reg of %s\n", __func__,__LINE__,cpower->vdd_name);
		cpower->vdd_reg = regulator_get(dev,cpower->vdd_name);
		if (IS_ERR(cpower->vdd_reg)) {
			tp_log_err("%s %d: %s get fail!\n", __func__,__LINE__,cpower->vdd_name);
			return -EINVAL;
		}

		rc = regulator_set_voltage(cpower->vdd_reg, 2850000,2850000);
		if (rc < 0) {
			tp_log_err( "%s %d: vdd regulator set fail, rc=%d\n", __func__,__LINE__,rc);
			regulator_put(cpower->vdd_reg);
			rc = -EINVAL;
		}
	} else if (CY_POWER_GPIO == cpower->vdd_type) {
		/* -1 means an invalid gpio */
		if (-1 == cpower->vdd_en_gpio) {
			tp_log_err("%s %d: Invalid gpio: %d.",__func__,__LINE__,cpower->vdd_en_gpio);
			return -EINVAL;
		}

		tp_log_info("%s %d: request gpio of vdd_en: %d\n", __func__,__LINE__,cpower->vdd_en_gpio);
		rc = gpio_request(cpower->vdd_en_gpio, "cyttsp5_vdd_enable_gpio");
		if(rc){
			tp_log_err("%s: unable to reques vdd-enable-gpio : %d.\n",__func__, cpower->vdd_en_gpio);
			return -EINVAL;
		}
	} else if (CY_POWER_NONE == cpower->vdd_type){
		tp_log_info("%s %d: vdd config none: %d\n", __func__,__LINE__, cpower->vdd_type);
	} else {
		tp_log_err("%s %d: vdd config invalid: %d\n", __func__,__LINE__, cpower->vdd_type);
		return -EINVAL;
	}

	/* vbus init */
	if (CY_POWER_PMU == cpower->vbus_type) {
		tp_log_info("%s %d: get reg of %s\n", __func__,__LINE__,cpower->vbus_name);
		cpower->vbus_reg = regulator_get(dev,cpower->vbus_name);
		if (IS_ERR(cpower->vbus_reg)) {
			tp_log_err("%s %d: %s get fail!\n", __func__,__LINE__,cpower->vbus_name);
			rc = -EINVAL;
			goto err_vdd_free;
		}
#ifndef CONFIG_HWATCH_POWER
		rc = regulator_set_voltage(cpower->vbus_reg, 1800000,1800000);
		if (rc < 0) {
			tp_log_err( "%s %d: vbus regulator set fail, rc=%d\n", __func__,__LINE__,rc);
			regulator_put(cpower->vbus_reg);
			rc = -EINVAL;
			goto err_vdd_free;
		}
#endif
	} else if (CY_POWER_GPIO == cpower->vbus_type) {
		/* -1 means an invalid gpio */
		if (-1 == cpower->vbus_en_gpio) {
			tp_log_err("%s %d: Invalid gpio: %d.",__func__,__LINE__,cpower->vbus_en_gpio);
			rc = -EINVAL;
			goto err_vdd_free;
		}

		tp_log_info("%s %d: request gpio of vbus_en: %d\n", __func__,__LINE__,cpower->vbus_en_gpio);
		rc = gpio_request(cpower->vbus_en_gpio, "cyttsp5_vdd_enable_gpio");
		if(rc){
			tp_log_err("%s: unable to reques vbus-enable-gpio : %d.\n",__func__, cpower->vbus_en_gpio);
			rc = -EINVAL;
			goto err_vdd_free;
		}
	} else if (CY_POWER_NONE == cpower->vbus_type){
		 tp_log_info("%s %d: vbus config none: %d\n", __func__,__LINE__, cpower->vbus_type);
	} else {
		tp_log_err("%s %d: vbus config invalid: %d\n", __func__,__LINE__, cpower->vbus_type);
		rc = -EINVAL;
		goto err_vdd_free;
	}

	return 0;
err_vdd_free:
	if(CY_POWER_PMU == cpower->vdd_type) {
		regulator_put(cpower->vdd_reg);
	} else if (CY_POWER_GPIO == cpower->vdd_type) {
		gpio_free(cpower->vdd_en_gpio);
	}

	return rc;
}

static void cyttsp5_power_release(struct device* dev, struct cyttsp5_core_data *pcore_data)
{
	struct cyttsp5_power_config *cpower;

	if (NULL == dev || NULL == pcore_data || NULL == pcore_data->cpdata) {
		tp_log_err("%s %d: input parameter missing!\n", __func__, __LINE__);
		return;
	}

	cpower = pcore_data->cpdata->power_config;
	if (NULL == cpower) {
		tp_log_err("%s %d: Power config is NULL!\n",__func__,__LINE__);
		return;
	}
	return;
}

static int cyttsp5_power_on(struct device* dev, struct cyttsp5_core_data *pcore_data)
{
	int rc = 0;
	struct cyttsp5_power_config *cpower;

	if (NULL == dev || NULL == pcore_data || NULL == pcore_data->cpdata) {
		tp_log_err("%s %d: input parameter missing!\n", __func__, __LINE__);
		return -EINVAL;
	}

	cpower = pcore_data->cpdata->power_config;
	if (NULL == cpower) {
		tp_log_err("%s %d: Power config is NULL!\n",__func__,__LINE__);
		return -EINVAL;
	}

	/* VBUS on*/
	if(CY_POWER_PMU == cpower->vbus_type) {
		rc = regulator_enable(cpower->vbus_reg);
		if (rc < 0) {
			tp_log_err( "%s %d: vbus regulator enable fail, rc=%d\n", __func__,__LINE__,rc);
			return -EINVAL;
		}
	} else if (CY_POWER_GPIO == cpower->vbus_type) {
		rc = gpio_direction_output(cpower->vbus_en_gpio, 1);
		if(rc){
			tp_log_err("%s %d: Fail set vbus_en_gpio.\n", __func__,__LINE__);
			return -EINVAL;
		}
	} else {
		tp_log_debug("%s %d: vbus config none: %d\n", __func__,__LINE__, cpower->vbus_type);
	}

	msleep(2);
	/* VDD on*/
	if(CY_POWER_PMU == cpower->vdd_type) {
		rc = regulator_enable(cpower->vdd_reg);
		if (rc < 0) {
			tp_log_err( "%s %d: vdd regulator enable fail, rc=%d\n", __func__,__LINE__,rc);
			return -EINVAL;
		}
	} else if (CY_POWER_GPIO == cpower->vdd_type) {
		rc = gpio_direction_output(cpower->vdd_en_gpio, 1);
		if(rc){
			tp_log_err("%s %d: Fail set vdd_en_gpio.\n", __func__,__LINE__);
			return -EINVAL;
		}
	} else {
		tp_log_debug("%s %d: vdd config none: %d\n", __func__,__LINE__, cpower->vdd_type);
	}

	msleep(2);
	return 0;
}

static int cyttsp5_power_off(struct device* dev, struct cyttsp5_core_data *pcore_data)
{
	int rc = 0;
	struct cyttsp5_power_config *cpower;

	if (NULL == dev || NULL == pcore_data || NULL == pcore_data->cpdata) {
		tp_log_err("%s %d: input parameter missing!\n", __func__, __LINE__);
		return -EINVAL;
	}

	cpower = pcore_data->cpdata->power_config;
	if (NULL == cpower) {
		tp_log_err("%s %d: Power config is NULL!\n",__func__,__LINE__);
		return -EINVAL;
	}

	/* VBUS off */
	if(CY_POWER_PMU == cpower->vbus_type) {
		rc = regulator_disable(cpower->vbus_reg);
		if (rc < 0) {
			tp_log_err( "%s %d: vbus regulator disable fail, rc=%d\n", __func__,__LINE__,rc);
			return -EINVAL;
		}
	} else if (CY_POWER_GPIO == cpower->vbus_type) {
		rc = gpio_direction_output(cpower->vbus_en_gpio, 0);
		if(rc){
			tp_log_err("%s %d: Fail set vbus_en_gpio.\n", __func__,__LINE__);
			return -EINVAL;
		}
	} else {
		tp_log_debug("%s %d: vbus config none: %d\n", __func__,__LINE__, cpower->vbus_type);
	}

	/* VDD off */
	if(CY_POWER_PMU == cpower->vdd_type) {
		rc = regulator_disable(cpower->vdd_reg);
		if (rc < 0) {
			tp_log_err( "%s %d: vdd regulator disable fail, rc=%d\n", __func__,__LINE__,rc);
			return -EINVAL;
		}
	} else if (CY_POWER_GPIO == cpower->vdd_type) {
		rc = gpio_direction_output(cpower->vdd_en_gpio, 0);
		if(rc){
			tp_log_err("%s %d: Fail set vdd_en_gpio.\n", __func__,__LINE__);
			return -EINVAL;
		}
	} else {
		tp_log_debug("%s %d: vdd config none: %d\n", __func__,__LINE__, cpower->vdd_type);
	}

	return 0;
}
#endif /*CONFIG_HWATCH_POWER*/

int cyttsp5_probe(const struct cyttsp5_bus_ops *ops, struct device *dev,
		u16 irq, size_t xfer_buf_size)
{
	struct cyttsp5_core_data *cd;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	enum cyttsp5_atten_type type;
	int rc = 0;

	tp_log_warning("%s %d:cyttsp5_core_probe start.\n", __func__, __LINE__);
	if (!pdata || !pdata->core_pdata || !pdata->mt_pdata) {
		tp_log_err("%s %d: Missing platform data\n", __func__, __LINE__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	/*  */
	if (pdata->core_pdata->flags & CY_CORE_FLAG_POWEROFF_ON_SLEEP) {
		if (!pdata->core_pdata->power) {
			tp_log_err("%s %d: Missing platform data function\n",
					__func__, __LINE__);
			rc = -ENODEV;
			goto error_no_pdata;
		}
	}

	/* get context and debug print buffers */
	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (!cd) {
		tp_log_err("%s %d:cd kzalloc fail.\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto error_alloc_data;
	}

	/* Initialize device info */
	cd->dev = dev;
	cd->pdata = pdata;
	cd->cpdata = pdata->core_pdata;
	cd->bus_ops = ops;
	scnprintf(cd->core_id, 20, "%s%d", CYTTSP5_CORE_NAME, core_number++);

	cd->glove_mode_enabled = 0;//set default glove mode as disable
	cd->holster_mode_enabled = 0;//set default holster mode as disable

#ifdef CONFIG_HWATCH_POWER
	/* power on */
	tp_log_info("%s %d:cyttsp5_core_probe power on.\n", __func__, __LINE__);

	rc = cyttsp5_power_init(dev,cd);
	if (rc) {
		tp_log_err("%s %d:power init fail.\n", __func__, __LINE__);
		goto error_power_init;
	}

	rc = cyttsp5_power_on(dev, cd);
	if (rc) {
		tp_log_err("%s %d:power on fail.\n", __func__, __LINE__);
		goto error_power_on;
	}
#endif /* CONFIG_HWATCH_POWER */

	/* Initialize mutexes and spinlocks */
	mutex_init(&cd->system_lock);
	mutex_init(&cd->adap_lock);
	mutex_init(&cd->hid_report_lock);
	spin_lock_init(&cd->spinlock);

	/* Initialize attention lists */
	for (type = 0; type < CY_ATTEN_NUM_ATTEN; type++)
		INIT_LIST_HEAD(&cd->atten_list[type]);

	/* Initialize parameter list */
	INIT_LIST_HEAD(&cd->param_list);

	/* Initialize wait queue */
	init_waitqueue_head(&cd->wait_q);

	/* Initialize works */
	INIT_WORK(&cd->startup_work, cyttsp5_startup_work_function);
	INIT_WORK(&cd->watchdog_work, cyttsp5_watchdog_work);

	/* Initialize HID specific data */
	cd->hid_core.hid_vendor_id = (cd->cpdata->vendor_id) ?
		cd->cpdata->vendor_id : CY_HID_VENDOR_ID;
	cd->hid_core.hid_product_id = (cd->cpdata->product_id) ?
		cd->cpdata->product_id : CY_HID_APP_PRODUCT_ID;
	cd->hid_core.hid_desc_register =
		cpu_to_le16(cd->cpdata->hid_desc_register);

	/* Set platform easywake value */
	cd->easy_wakeup_gesture = cd->cpdata->easy_wakeup_gesture;

	cd->easy_wakeup_supported_gestures = cd->cpdata->easy_wakeup_supported_gestures;
	cd->wakeup_keys = cd->cpdata->wakeup_keys;
	cd->double_tap_enabled = cd->cpdata->double_tap_enabled;
	cd->dtz_x0 = cd->cpdata->dtz_x0;
	cd->dtz_y0 = cd->cpdata->dtz_y0;
	cd->dtz_x1 = cd->cpdata->dtz_x1;
	cd->dtz_y1 = cd->cpdata->dtz_y1;

	cd->glove_support = cd->cpdata->glove_support;
	cd->holster_support = cd->cpdata->holster_support;
	cd->mmi_test_support = cd->cpdata->mmi_test_support;
	tp_log_info("%s mmi_test_support = %d\n", __func__, cd->mmi_test_support);
	/* Set Panel ID to Not Enabled */
	cd->panel_id = PANEL_ID_NOT_ENABLED;

	dev_set_drvdata(dev, cd);
	cyttsp5_add_core(dev);

	/* Call platform init function */
	if (cd->cpdata->init) {
		tp_log_debug("%s %d: Init HW\n", __func__, __LINE__);
		rc = cd->cpdata->init(cd->cpdata, 1, cd->dev);
	} else {
		tp_log_info("%s %d: No HW INIT function\n", __func__, __LINE__);
		rc = 0;
	}

	if (rc < 0) {
		tp_log_err("%s %d: HW Init fail r=%d\n", __func__, __LINE__, rc);
	}

	/* Call platform detect function */
	if (cd->cpdata->detect) {
		tp_log_info("%s %d: Detect HW\n", __func__, __LINE__);
		rc = cd->cpdata->detect(cd->cpdata, cd->dev,
				cyttsp5_platform_detect_read);
		if (rc) {
			tp_log_info("%s %d: No HW detected, rc = %d\n", __func__, __LINE__, rc);
			rc = -ENODEV;
			goto error_detect;
		}
	}

	/* Setup watchdog timer */
	setup_timer(&cd->watchdog_timer, cyttsp5_watchdog_timer, (unsigned long)cd);

	tp_log_info("%s %d:cyttsp5_core_probe setup irq.\n", __func__, __LINE__);
	rc = cyttsp5_setup_irq_gpio(cd);
	if (rc < 0) {
		tp_log_err("%s: Error, could not setup IRQ\n", __func__);
		goto error_setup_irq;
	}

	tp_log_info("%s: add sysfs interfaces\n", __func__);
	rc = add_sysfs_interfaces(dev);
	if (rc < 0) {
		tp_log_err("%s: Error, fail sysfs init\n", __func__);
		goto error_attr_create;
	}

#ifdef TTHE_TUNER_SUPPORT
	mutex_init(&cd->tthe_lock);
	cd->tthe_debugfs = debugfs_create_file(CYTTSP5_TTHE_TUNER_FILE_NAME,
			0644, NULL, cd, &tthe_debugfs_fops);
#endif
	rc = device_init_wakeup(dev, 1);
	if (rc < 0)
		tp_log_err("%s: Error, device_init_wakeup rc:%d\n",
				__func__, rc);

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	/*
	 * call startup directly to ensure that the device
	 * is tested before leaving the probe
	 */
	tp_log_info("%s: call startup\n", __func__);
	rc = cyttsp5_startup(cd, false);

	/* Do not fail probe if startup fails but the device is detected */
	if (rc == -ENODEV) {
		tp_log_err( "%s: Fail initial startup r=%d\n",
			__func__, rc);
		goto error_startup;
	}
	rc = cyttsp5_read_mfg_val0_(cd);
	if (rc < 0) {
		tp_log_warning("%s: driver could not read tp color data\n", __func__);
	}
	pm_runtime_put_sync(dev);
	tp_log_info("%s %d:cyttsp5_mt_probe start.\n", __func__, __LINE__);
	rc = cyttsp5_mt_probe(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error, fail mt probe\n", __func__);
		goto error_startup;
	}

	tp_log_info("%s %d:cyttsp5_btn_probe start.\n", __func__, __LINE__);
	rc = cyttsp5_btn_probe(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error, fail btn probe\n", __func__);
		goto error_startup_mt;
	}

	tp_log_info("%s %d:cyttsp5_proximity_probe start.\n", __func__, __LINE__);
	rc = cyttsp5_proximity_probe(dev);
	if (rc < 0) {
		tp_log_err( "%s: Error, fail proximity probe\n", __func__);
		goto error_startup_btn;
	}

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	/* detect current device successful, set the flag as present */
	set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	cyttsp5_setup_early_suspend(cd);
#elif defined(CONFIG_FB)
	cd->suspend_resume_flag = CY_STATE_RESUME;
	cyttsp5_setup_fb_notifier(cd);
#endif

#if NEED_SUSPEND_NOTIFIER
	cd->pm_notifier.notifier_call = cyttsp5_pm_notifier;
	register_pm_notifier(&cd->pm_notifier);
#endif

	cyttsp5_core_dev = cd->dev;

	if (cd->mmi_test_support) {
		cyttsp5_procfs_create();
	}

	gdev = dev;
	tp_log_warning("%s %d:cyttsp5_core_probe success.\n", __func__, __LINE__);
	return 0;

error_startup_btn:
	cyttsp5_btn_release(dev);
error_startup_mt:
	cyttsp5_mt_release(dev);
error_startup:
	pm_runtime_disable(dev);
	cancel_work_sync(&cd->startup_work);
	cyttsp5_stop_wd_timer(cd);
	cyttsp5_free_si_ptrs(cd);
	remove_sysfs_interfaces(dev);
error_attr_create:
	free_irq(cd->irq, cd);
	del_timer(&cd->watchdog_timer);
error_setup_irq:
error_detect:
	if (cd->cpdata->init)
		cd->cpdata->init(cd->cpdata, 0, dev);
#ifdef CONFIG_HWATCH_POWER
	cyttsp5_power_off(dev, cd);
#endif /*#ifdef CONFIG_HWATCH_POWER*/

	cyttsp5_del_core(dev);
	dev_set_drvdata(dev, NULL);

#ifdef CONFIG_HWATCH_POWER
error_power_on:
	cyttsp5_power_release(dev,cd);
error_power_init:
	kfree(cd);
#endif/*#ifdef CONFIG_HWATCH_POWER*/
error_alloc_data:
error_no_pdata:
	tp_log_err( "%s failed.\n", __func__);
	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp5_probe);

int cyttsp5_release(struct cyttsp5_core_data *cd)
{
	struct device *dev = cd->dev;

	cyttsp5_proximity_release(dev);
	cyttsp5_btn_release(dev);
	cyttsp5_mt_release(dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cd->es);
#elif defined(CONFIG_FB)
	fb_unregister_client(&cd->fb_notifier);
#endif

#if NEED_SUSPEND_NOTIFIER
	unregister_pm_notifier(&cd->pm_notifier);
#endif

	/*
	 * Suspend the device before freeing the startup_work and stopping
	 * the watchdog since sleep function restarts watchdog on failure
	 */
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);

	cancel_work_sync(&cd->startup_work);

	cyttsp5_stop_wd_timer(cd);

	device_init_wakeup(dev, 0);

#ifdef TTHE_TUNER_SUPPORT
	mutex_lock(&cd->tthe_lock);
	cd->tthe_exit = 1;
	wake_up(&cd->wait_q);
	mutex_unlock(&cd->tthe_lock);
	debugfs_remove(cd->tthe_debugfs);
#endif
	remove_sysfs_interfaces(dev);
	free_irq(cd->irq, cd);
	if (cd->cpdata->init)
		cd->cpdata->init(cd->cpdata, 0, dev);
	dev_set_drvdata(dev, NULL);
	cyttsp5_del_core(dev);
	cyttsp5_free_si_ptrs(cd);
	cyttsp5_free_hid_reports(cd);
	kfree(cd);
	return 0;
}
EXPORT_SYMBOL_GPL(cyttsp5_release);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Core Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
