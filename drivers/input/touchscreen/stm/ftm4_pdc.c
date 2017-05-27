/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name                   : ftm4_pdc.c
* Authors                      : AMS(Analog Mems Sensor) Team
* Description     : FTS Capacitive touch screen controller (FingerTipS)
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*******************************************************************************/

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/list.h>
#include "ftm4_ts.h"

#define TSP_FACTEST_RESULT_PASS		2
#define TSP_FACTEST_RESULT_FAIL		1
#define TSP_FACTEST_RESULT_NONE		0

#define BUFFER_MAX			((256 * 1024) - 16)
#define READ_CHUNK_SIZE			128

#define FTS_F_WIX1_ADDR			0x1FE7
#define FTS_S_WIX1_ADDR			0x1FE8
#define FTS_F_WIX2_ADDR			0x18FD
#define FTS_S_WIX2_ADDR			0x1929
#define FTS_WATER_SELF_RAW_ADDR		0x1E

#define FTS_MAX_TX_LENGTH		44
#define FTS_MAX_RX_LENGTH		64

#define FTS_CX2_READ_LENGTH		4
#define FTS_CX2_ADDR_OFFSET		3
#define FTS_CX2_TX_START		0
#define FTS_CX2_BASE_ADDR		0x1000
#define SEC_CMD_STR_LEN			12

#define DEBUG_MSG 1
enum {
	TYPE_RAW_DATA = 0,
	TYPE_FILTERED_DATA = 2,
	TYPE_STRENGTH_DATA = 4,
	TYPE_BASELINE_DATA = 6
};

enum {
	BUILT_IN = 0,
	UMS,
};

enum CMD_STATUS {
	CMD_STATUS_WAITING = 0,
	CMD_STATUS_RUNNING,
	CMD_STATUS_OK,
	CMD_STATUS_FAIL,
	CMD_STATUS_NOT_APPLICABLE,
};

static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_config_ver(void *device_data);
static void get_threshold(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void run_rawcap_read(void *device_data);
static void get_rawcap(void *device_data);
static void run_ix_data_read(void *device_data);
static void run_ix_data_read_all(void *device_data);
static void run_self_raw_read(void *device_data);
static void run_self_raw_read_all(void *device_data);
static void get_cx_data(void *device_data);
static void run_cx_data_read(void *device_data);
static void get_cx_all_data(void *device_data);
static void get_strength_all_data(void *device_data);
static void set_tsp_test_result(void *device_data);
static void get_tsp_test_result(void *device_data);
static void run_trx_short_test(void *device_data);
static void report_rate(void *device_data);
static void delay(void *device_data);
static void debug(void *device_data);
static void run_autotune_enable(void *device_data);
static void run_autotune(void *device_data);
static void not_support_cmd(void *device_data);

static ssize_t store_cmd(struct device *dev, struct device_attribute *devattr,
	const char *buf, size_t count);
static ssize_t show_cmd_status(struct device *dev,
	struct device_attribute *devattr, char *buf);
static ssize_t show_cmd_result(struct device *dev,
	struct device_attribute *devattr, char *buf);
static ssize_t cmd_list_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t store_upgrade(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count);
static ssize_t store_check_fw(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count);
static ssize_t show_version_info(struct device *dev,
	struct device_attribute *devattr, char *buf);

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
static void tui_mode_cmd(struct fts_ts_info *info);
#endif

struct fts_cmd fts_commands[] = {
	{FTS_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{FTS_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{FTS_CMD("get_config_ver", get_config_ver),},
	{FTS_CMD("get_threshold", get_threshold),},
	{FTS_CMD("get_x_num", get_x_num),},
	{FTS_CMD("get_y_num", get_y_num),},
	{FTS_CMD("run_rawcap_read", run_rawcap_read),},
	{FTS_CMD("get_rawcap", get_rawcap),},
	{FTS_CMD("run_ix_data_read", run_ix_data_read),},
	{FTS_CMD("run_ix_data_read_all", run_ix_data_read_all),},
	{FTS_CMD("run_self_raw_read", run_self_raw_read),},
	{FTS_CMD("run_self_raw_read_all", run_self_raw_read_all),},
	{FTS_CMD("get_cx_data", get_cx_data),},
	{FTS_CMD("run_cx_data_read", run_cx_data_read),},
	{FTS_CMD("get_cx_all_data", get_cx_all_data),},
	{FTS_CMD("get_strength_all_data", get_strength_all_data),},
	{FTS_CMD("set_tsp_test_result", set_tsp_test_result),},
	{FTS_CMD("get_tsp_test_result", get_tsp_test_result),},
	{FTS_CMD("report_rate", report_rate),},
	{FTS_CMD("delay", delay),},
	{FTS_CMD("debug", debug),},
	{FTS_CMD("run_autotune_enable", run_autotune_enable),},
	{FTS_CMD("run_autotune", run_autotune),},
	{FTS_CMD("run_trx_short_test", run_trx_short_test),},
	{FTS_CMD("not_support_cmd", not_support_cmd),},
};

static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);
static DEVICE_ATTR(cmd_list, S_IRUGO, cmd_list_show, NULL);
static DEVICE_ATTR(fw_upgrade, S_IWUSR | S_IWGRP, NULL, store_upgrade);
static DEVICE_ATTR(check_fw, S_IWUSR | S_IWGRP, NULL, store_check_fw);
static DEVICE_ATTR(version, S_IRUGO, show_version_info, NULL);

static struct attribute *touch_pdc_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	&dev_attr_cmd_list.attr,
	&dev_attr_fw_upgrade.attr,
	&dev_attr_check_fw.attr,
	&dev_attr_version.attr,
	NULL,
};

static struct attribute_group touch_pdc_attr_group = {
	.attrs = touch_pdc_attributes,
};

static ssize_t store_check_fw(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	unsigned int input = 0;
	int ret = 0;

	if (sscanf(buf, "%u", &input) != 1) {
		tsp_debug_err(&info->client->dev, "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (input) {
		mutex_lock(&info->device_mutex);

		info->test_fwpath[0] = '\0';
		ret = fts_fw_verify_update(info);

		mutex_unlock(&info->device_mutex);
	}

	if (ret)
		return ret;
	else
		return count;
}

static ssize_t store_upgrade(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	int ret = 0;

	if (strlcpy(&info->test_fwpath[0], buf, count) <= 0) {
		tsp_debug_err(&info->client->dev, "%s: invalid firmware name\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&info->device_mutex);

	ret = fts_fw_verify_update(info);
	info->test_fwpath[0] = '\0';

	mutex_unlock(&info->device_mutex);

	if (ret)
		return ret;
	else
		return count;
}

static ssize_t show_version_info(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	int offset = 0;
	char str[16] = {0};
	int ret = 0;

	mutex_lock(&info->device_mutex);

	if (fts_get_version_info(info) < 0) {
		mutex_unlock(&info->device_mutex);
		return -EINVAL;
	}

	mutex_unlock(&info->device_mutex);

	ret += snprintf(str + ret, sizeof(str) - ret,
		"v%d.%02d", info->ic_fw_ver.major, info->ic_fw_ver.minor);

	if (info->ic_fw_ver.build) {
		ret += snprintf(str + ret, sizeof(str) - ret,
				".%d", info->ic_fw_ver.build);
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s\n", str);

	return offset;
}

static int fts_check_index(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };
	int node;

	if (info->cmd_param[0] < 0 ||
	    info->cmd_param[0] >= info->SenseChannelLength ||
	    info->cmd_param[1] < 0 ||
	    info->cmd_param[1] >= info->ForceChannelLength) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		strncat(info->cmd_result, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_FAIL;
		tsp_debug_info(&info->client->dev,
			"%s: parameter error: %u,%u\n",
			__func__, info->cmd_param[0], info->cmd_param[1]);
		node = -1;
		return node;
	}
	node = info->cmd_param[1] * info->SenseChannelLength +
		info->cmd_param[0];
	tsp_debug_info(&info->client->dev, "%s: node = %d\n", __func__, node);
	return node;
}

static ssize_t store_cmd(struct device *dev, struct device_attribute *devattr,
			   const char *buf, size_t count)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	char *cur, *start, *end;
	char buff[CMD_STR_LEN] = { 0 };
	int len, i;
	struct fts_cmd *ft_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;

	if (!info) {
		tsp_debug_err(&info->client->dev,
				"%s: No platform data found\n", __func__);
		return -EINVAL;
	}

	if (!info->input_dev) {
		tsp_debug_err(&info->client->dev,
				"%s: No input_dev data found\n", __func__);
		return -EINVAL;
	}

	if (count > CMD_STR_LEN) {
		tsp_debug_err(&info->client->dev,
				"%s: overflow command length\n", __func__);
		return -EINVAL;
	}

	if (info->cmd_is_running == true) {
		tsp_debug_err(&info->client->dev,
				"ft_cmd: other cmd is running.\n");
		if (strncmp("clear_cover_mode", buf, 16) == 0) {
			cancel_delayed_work(&info->cover_cmd_work);
			tsp_debug_err(&info->client->dev,
				"[cmd is delayed] %d, param = %d, %d\n",
				__LINE__, buf[17]-'0', buf[19]-'0');
			info->delayed_cmd_param[0] = buf[17]-'0';
			if (info->delayed_cmd_param[0] > 1)
				info->delayed_cmd_param[1] = buf[19]-'0';

			schedule_delayed_work(&info->cover_cmd_work,
					msecs_to_jiffies(10));
		}
		return -EBUSY;
	} else if (info->reinit_done == false) {
		tsp_debug_err(&info->client->dev,
				"ft_cmd: reinit is working\n");
		if (strncmp("clear_cover_mode", buf, 16) == 0) {
			cancel_delayed_work(&info->cover_cmd_work);
			tsp_debug_err(&info->client->dev,
				"[cmd is delayed] %d, param = %d, %d\n",
				__LINE__, buf[17]-'0', buf[19]-'0');
			info->delayed_cmd_param[0] = buf[17]-'0';
			if (info->delayed_cmd_param[0] > 1)
				info->delayed_cmd_param[1] = buf[19]-'0';

			if (info->delayed_cmd_param[0] == 0)
				schedule_delayed_work(&info->cover_cmd_work,
						msecs_to_jiffies(300));
		}
	}

	/* check lock   */
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = true;
	mutex_unlock(&info->cmd_lock);
	info->cmd_state = 1;
	memset(info->cmd_param, 0x00, ARRAY_SIZE(info->cmd_param));

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);
	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);

	else
		memcpy(buff, buf, len);
	tsp_debug_info(&info->client->dev, "COMMAND : %s\n", buff);

	/* find command */
	list_for_each_entry(ft_cmd_ptr, &info->cmd_list_head, list) {
		if (!strncmp(buff, ft_cmd_ptr->cmd_name, CMD_STR_LEN)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(ft_cmd_ptr, &info->cmd_list_head, list) {
			if (!strncmp
			 ("not_support_cmd", ft_cmd_ptr->cmd_name,
			  CMD_STR_LEN))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));

		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strnlen(buff, ARRAY_SIZE(buff))) =
				'\0';
				if (kstrtoint
				 (buff, 10,
				  info->cmd_param + param_cnt) < 0)
					goto err_out;
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}
	tsp_debug_info(&info->client->dev, "cmd = %s\n", ft_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		tsp_debug_info(&info->client->dev, "cmd param %d= %d\n", i,
			  info->cmd_param[i]);

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_INPUT_SECURED & trustedui_get_current_mode())
		tui_mode_cmd(info);
	else
#endif
	ft_cmd_ptr->cmd_func(info);

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	char buff[16] = {0};

	if (!info) {
		tsp_debug_err(&info->client->dev,
				"%s: No platform data found\n", __func__);
		return -EINVAL;
	}

	if (!info->input_dev) {
		tsp_debug_err(&info->client->dev,
				"%s: No input_dev data found\n", __func__);
		return -EINVAL;
	}

	tsp_debug_info(&info->client->dev, "tsp cmd: status:%d\n", info->cmd_state);
	if (info->cmd_state == CMD_STATUS_WAITING)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (info->cmd_state == CMD_STATUS_RUNNING)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (info->cmd_state == CMD_STATUS_OK)
		snprintf(buff, sizeof(buff), "OK");

	else if (info->cmd_state == CMD_STATUS_FAIL)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (info->cmd_state == CMD_STATUS_NOT_APPLICABLE)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

static ssize_t show_cmd_result(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	if (!info) {
		tsp_debug_err(&info->client->dev,
				"%s: No platform data found\n", __func__);
		return -EINVAL;
	}

	if (!info->input_dev) {
		tsp_debug_err(&info->client->dev,
				"%s: No input_dev data found\n", __func__);
		return -EINVAL;
	}

	tsp_debug_info(&info->client->dev, "tsp cmd: result: %s\n",
		   info->cmd_result);
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);
	info->cmd_state = 0;
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->cmd_result);
}

static ssize_t cmd_list_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	int ii = 0;
	char buffer[info->cmd_buf_size+CMD_STR_LEN];
	char buffer_name[CMD_STR_LEN];

	snprintf(buffer, CMD_STR_LEN, "++factory command list++\n");
	while (strncmp(fts_commands[ii].cmd_name, "not_support_cmd", 16) != 0) {
		snprintf(buffer_name, CMD_STR_LEN,
				"%s\n", fts_commands[ii].cmd_name);
		strcat(buffer, buffer_name);
		ii++;
	}

	tsp_debug_info(&info->client->dev,
		"%s: length : %u / %d\n", __func__,
		(unsigned int)strlen(buffer), info->cmd_buf_size+CMD_STR_LEN);

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buffer);
}

static void set_default_result(struct fts_ts_info *info)
{
	char delim = ':';

	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd, strnlen(info->cmd, CMD_STR_LEN));
	strncat(info->cmd_result, &delim, 1);
}

static void set_cmd_result(struct fts_ts_info *info, char *buff, int len)
{
	strncat(info->cmd_result, buff, len);
}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
static void tui_mode_cmd(struct fts_ts_info *info)
{
	char buff[16] = "TUImode:FAIL";

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}
#endif

static void not_support_cmd(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
	tsp_debug_info(&info->client->dev, "%s: \"%s\"\n", __func__, buff);
}

void procedure_cmd_event(struct fts_ts_info *info, unsigned char *data)
{
	char buff[16] = {0};

	if ((data[1] == 0x00) && (data[2] == 0x62)) {
		snprintf(buff, sizeof(buff), "%d",
					*(unsigned short *)&data[3]);
		tsp_debug_info(&info->client->dev, "%s: %s\n", "get_threshold", buff);
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_OK;

	} else if ((data[1] == 0x07) && (data[2] == 0xE7)) {
		if (data[3] <= TSP_FACTEST_RESULT_PASS) {
			sprintf(buff, "%s",
				data[3] == TSP_FACTEST_RESULT_PASS ? "PASS" :
				data[3] == TSP_FACTEST_RESULT_FAIL ? "FAIL" : "NONE");
			tsp_debug_info(&info->client->dev,
				"%s: success [%s][%d]", "get_tsp_test_result",
				data[3] == TSP_FACTEST_RESULT_PASS ? "PASS" :
				data[3] == TSP_FACTEST_RESULT_FAIL ? "FAIL" :
				"NONE", data[3]);
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			info->cmd_state = CMD_STATUS_OK;
		} else {
			snprintf(buff, sizeof(buff), "%s", "NG");
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			info->cmd_state = CMD_STATUS_FAIL;
			tsp_debug_info(&info->client->dev, "%s: %s\n",
						"get_tsp_test_result",
						buff);
		}
	}
}
EXPORT_SYMBOL(procedure_cmd_event);

void fts_print_frame(struct fts_ts_info *info, short *min, short *max)
{
	int i = 0;
	int j = 0;
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = {0};

	pStr = kzalloc(6 * (info->SenseChannelLength + 1), GFP_KERNEL);
	if (pStr == NULL) {
		tsp_debug_info(&info->client->dev, "FTS pStr kzalloc failed\n");
		return;
	}

	memset(pStr, 0x0, 6 * (info->SenseChannelLength + 1));
	snprintf(pTmp, sizeof(pTmp), "    ");
	strncat(pStr, pTmp, 6 * info->SenseChannelLength);

	for (i = 0; i < info->SenseChannelLength; i++) {
		snprintf(pTmp, sizeof(pTmp), "Rx%02d  ", i);
		strncat(pStr, pTmp, 6 * info->SenseChannelLength);
	}

	tsp_debug_info(&info->client->dev, "FTS %s\n", pStr);
	memset(pStr, 0x0, 6 * (info->SenseChannelLength + 1));
	snprintf(pTmp, sizeof(pTmp), " +");
	strncat(pStr, pTmp, 6 * info->SenseChannelLength);

	for (i = 0; i < info->SenseChannelLength; i++) {
		snprintf(pTmp, sizeof(pTmp), "------");
		strncat(pStr, pTmp, 6 * info->SenseChannelLength);
	}

	tsp_debug_info(&info->client->dev, "FTS %s\n", pStr);

	for (i = 0; i < info->ForceChannelLength; i++) {
		memset(pStr, 0x0, 6 * (info->SenseChannelLength + 1));
		snprintf(pTmp, sizeof(pTmp), "Tx%02d | ", i);
		strncat(pStr, pTmp, 6 * info->SenseChannelLength);

		for (j = 0; j < info->SenseChannelLength; j++) {
			snprintf(pTmp, sizeof(pTmp), "%5d ",
				info->pFrame[(i * info->SenseChannelLength) + j]);

			if (i > 0) {
				if (info->pFrame[(i * info->SenseChannelLength) + j] < *min)
					*min = info->pFrame[(i * info->SenseChannelLength) + j];

				if (info->pFrame[(i * info->SenseChannelLength) + j] > *max)
					*max = info->pFrame[(i * info->SenseChannelLength) + j];
			}
			strncat(pStr, pTmp, 6 * info->SenseChannelLength);
		}
		tsp_debug_info(&info->client->dev, "FTS %s\n", pStr);
	}

	kfree(pStr);
}

static int fts_panel_ito_test(struct fts_ts_info *info)
{
	unsigned char cmd = READ_ONE_EVENT;
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char regAdd[4] = {0xB0, 0x03, 0x60, 0xFB};
	unsigned char wregAdd[3] = {0xA7, 0x01, 0x00};
	uint8_t *errortypes[16] = {
		"F open", "S open", "F2G short", "S2G short", "F2V short",
		"S2V short", "F2F short", "S2S short", "F2S short",
		"FPC F open", "FPC S open", "Key F open", "Key S open",
		"Reserved", "Reserved", "Reserved"};
	int retry = 0;
	int result = -1;

	info->fts_systemreset(info);
	info->fts_wait_for_ready(info);
	info->fts_irq_enable(info, false);
	info->fts_interrupt_set(info, INT_DISABLE);
	info->fts_write_reg(info, &regAdd[0], 4);
	info->fts_command(info, FLUSHBUFFER);
	info->fts_write_reg(info, &wregAdd[0], 3);
	fts_delay(200);
	memset(data, 0x0, FTS_EVENT_SIZE);
	while (info->fts_read_reg
			(info, &cmd, 1, (unsigned char *)data, FTS_EVENT_SIZE)) {
		if ((data[0] == 0x0F) && (data[1] == 0x05)) {
			switch (data[2]) {
			case NO_ERROR:
				if (data[3] == 0x00) {
					tsp_debug_info(
						&info->client->dev,
						"ITO open / short test PASS!!\n");
					return 1;
				}
				break;
			case ITO_FORCE_OPEN:
			case ITO_SENSE_OPEN:
			case ITO_FORCE_SHRT_GND:
			case ITO_SENSE_SHRT_GND:
			case ITO_FORCE_SHRT_VCM:
			case ITO_SENSE_SHRT_VCM:
			case ITO_FORCE_SHRT_FORCE:
			case ITO_SENSE_SHRT_SENSE:
			case ITO_F2E_SENSE:
			case ITO_FPC_FORCE_OPEN:
			case ITO_FPC_SENSE_OPEN:
			case ITO_KEY_FORCE_OPEN:
			case ITO_KEY_SENSE_OPEN:
			case ITO_RESERVED0:
			case ITO_RESERVED1:
			case ITO_RESERVED2:
			case ITO_MAX_ERR_REACHED:
				tsp_debug_info(
					&info->client->dev,
					"ITO open / short test FAIL!! Error Type : %s, Channel : %d\n",
					errortypes[data[2]], data[3]);
				break;
			}
			break;
		}
		if (retry++ > 30) {
			tsp_debug_info(&info->client->dev,
				"Time over - wait for result of ITO test\n");
			break;
		}
		fts_delay(10);
	}
	info->fts_systemreset(info);
	info->fts_wait_for_ready(info);
	info->fts_command(info, SENSEON);
	info->touch_count = 0;
	info->fts_command(info, FLUSHBUFFER);
	info->fts_interrupt_set(info, INT_ENABLE);
	info->fts_irq_enable(info, true);
	return result;
}
int fts_read_frame(struct fts_ts_info *info, unsigned char type, short *min,
		 short *max)
{
	unsigned char pFrameAddress[8] = {
		0xD0, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
	unsigned int FrameAddress = 0;
	unsigned int writeAddr = 0;
	unsigned int start_addr = 0;
	unsigned int end_addr = 0;
	unsigned int totalbytes = 0;
	unsigned int remained = 0;
	unsigned int readbytes = 0xFF;
	unsigned int dataposition = 0;
	unsigned char *pRead = NULL;
	int rc = 0;
	int ret = 0;
	int i = 0;

	tsp_debug_info(&info->client->dev,
		"===> fts_read_frame digital rev( %d ) sense ( %d ) force (%d)\n",
		info->digital_rev, info->SenseChannelLength, info->ForceChannelLength);

	pRead = kzalloc(BUFFER_MAX, GFP_KERNEL);
	if (pRead == NULL) {
		tsp_debug_info(&info->client->dev,
				"FTS pRead kzalloc failed\n");
		rc = 1;
		goto ErrorExit;
	}

	pFrameAddress[2] = type;
	totalbytes = info->SenseChannelLength * info->ForceChannelLength * 2;
	ret = info->fts_read_reg(info, &pFrameAddress[0], 3, pRead, pFrameAddress[3]);

	if (ret > 0) {
		if (info->digital_rev == FTS_DIGITAL_REV_1)
			FrameAddress = pRead[0] + (pRead[1] << 8);
		else if (info->digital_rev == FTS_DIGITAL_REV_2)
			FrameAddress = pRead[1] + (pRead[2] << 8);

		start_addr = FrameAddress+info->SenseChannelLength * 2;
		end_addr = start_addr + totalbytes;
	} else {
		tsp_debug_info(&info->client->dev,
				"FTS read failed rc = %d\n", ret);
		rc = 2;
		goto ErrorExit;
	}

#ifdef DEBUG_MSG
	tsp_debug_info(&info->client->dev,
			"FTS FrameAddress = %X\n", FrameAddress);
	tsp_debug_info(&info->client->dev,
			"FTS start_addr = %X, end_addr = %X\n",
			start_addr, end_addr);
#endif

	remained = totalbytes;
	for (writeAddr = start_addr; writeAddr < end_addr;
			writeAddr += READ_CHUNK_SIZE) {
		pFrameAddress[1] = (writeAddr >> 8) & 0xFF;
		pFrameAddress[2] = writeAddr & 0xFF;

		if (remained >= READ_CHUNK_SIZE)
			readbytes = READ_CHUNK_SIZE;
		else
			readbytes = remained;

		memset(pRead, 0x0, readbytes);

#ifdef DEBUG_MSG
		tsp_debug_info(&info->client->dev,
				"FTS %02X%02X%02X readbytes=%d\n",
				pFrameAddress[0], pFrameAddress[1],
				pFrameAddress[2], readbytes);

#endif
		if (info->digital_rev == FTS_DIGITAL_REV_1) {
			info->fts_read_reg(info, &pFrameAddress[0],
					3, pRead, readbytes);
			remained -= readbytes;

			for (i = 0; i < readbytes; i += 2) {
				info->pFrame[dataposition++] =
				pRead[i] + (pRead[i + 1] << 8);
			}
		} else if (info->digital_rev == FTS_DIGITAL_REV_2) {
			info->fts_read_reg(info, &pFrameAddress[0],
					3, pRead, readbytes + 1);
			remained -= readbytes;

			for (i = 1; i < (readbytes+1); i += 2) {
				info->pFrame[dataposition++] =
				pRead[i] + (pRead[i + 1] << 8);
			}
		}
	}
	kfree(pRead);

#ifdef DEBUG_MSG
	tsp_debug_info(&info->client->dev,
		   "FTS writeAddr = %X, start_addr = %X, end_addr = %X\n",
		   writeAddr, start_addr, end_addr);
#endif

	switch (type) {
	case TYPE_RAW_DATA:
		tsp_debug_info(&info->client->dev,
			"FTS [Raw Data : 0x%X%X]\n", pFrameAddress[0],
			FrameAddress);
		break;
	case TYPE_FILTERED_DATA:
		tsp_debug_info(&info->client->dev,
			"FTS [Filtered Data : 0x%X%X]\n",
			pFrameAddress[0], FrameAddress);
		break;
	case TYPE_STRENGTH_DATA:
		tsp_debug_info(&info->client->dev,
			"FTS [Strength Data : 0x%X%X]\n",
			pFrameAddress[0], FrameAddress);
		break;
	case TYPE_BASELINE_DATA:
		tsp_debug_info(&info->client->dev,
			"FTS [Baseline Data : 0x%X%X]\n",
			pFrameAddress[0], FrameAddress);
		break;
	}
	fts_print_frame(info, min, max);

ErrorExit:
	return rc;
}

static void get_fw_ver_bin(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	if (strncmp(info->board->model_name, "G925", 4) == 0) {
		info->tspid_val = gpio_get_value(info->board->tspid);
		info->tspid2_val = gpio_get_value(info->board->tspid2);

		sprintf(buff, "ST%01X%01X%04X",
				info->tspid_val, info->tspid2_val,
				info->fw_main_version_of_bin);
	} else {
		sprintf(buff, "ST%02X%04X",
				info->panel_revision,
				info->fw_main_version_of_bin);
	}

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_OK;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_fw_ver_ic(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);

	if (strncmp(info->board->model_name, "G925", 4) == 0) {
		info->tspid_val = gpio_get_value(info->board->tspid);
		info->tspid2_val = gpio_get_value(info->board->tspid2);

		sprintf(buff, "ST%01X%01X%04X",
				info->tspid_val, info->tspid2_val,
				info->fw_main_version_of_ic);
	} else {
		sprintf(buff, "ST%02X%04X",
				info->panel_revision,
				info->fw_main_version_of_ic);
	}

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_OK;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_config_ver(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[20] = {0};
	const char *name = NULL;

	if (info->board->model_name)
		name = info->board->model_name;
	else if (info->board->project_name)
		name = info->board->project_name;

	snprintf(buff, sizeof(buff), "%s_ST_%04X",
			name? name : STM_DEVICE_NAME,
			info->config_version_of_ic);

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_OK;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_threshold(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	unsigned char cmd[4] = {0xB2, 0x00, 0x62, 0x02};
	char buff[CMD_STR_LEN] = {0};
	int timeout = 0;

	set_default_result(info);

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	info->fts_write_reg(info, &cmd[0], 4);
	info->cmd_state = CMD_STATUS_RUNNING;

	while (info->cmd_state == CMD_STATUS_RUNNING) {
		if (timeout++ > 30) {
			info->cmd_state = CMD_STATUS_FAIL;
			break;
		}
		fts_delay(10);
	}
}

static void get_x_num(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[16] = { 0 };

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%d", info->SenseChannelLength);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_y_num(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[16] = { 0 };

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%d", info->ForceChannelLength);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_OK;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void run_rawcap_read(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };
	short min = 0x7FFF;
	short max = 0x8000;

	set_default_result(info);
	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	fts_read_frame(info, TYPE_FILTERED_DATA, &min, &max);
	snprintf(buff, sizeof(buff), "%d,%d", min, max);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_OK;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);

}

static void get_rawcap(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };
	short val = 0;
	int node = 0;

	set_default_result(info);
	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	node = fts_check_index(info);
	if (node < 0)
		return;

	val = info->pFrame[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_OK;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_strength_all_data(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };
	short min = 0x7FFF;
	short max = 0x8000;
	const int str_size = info->ForceChannelLength *
		info->SenseChannelLength * 5;
	char all_strbuff[str_size];
	int i, j;

	memset(all_strbuff, 0, sizeof(char)*(str_size)); /* size 5 ex(1125,) */

	set_default_result(info);
	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	fts_read_frame(info, TYPE_STRENGTH_DATA, &min, &max);


	for (i = 0; i < info->ForceChannelLength; i++) {
		for (j = 0; j < info->SenseChannelLength; j++) {
			sprintf(buff, "%d,",
				info->pFrame[
				(i * info->SenseChannelLength) + j]);
			strcat(all_strbuff, buff);
		}
	}

	info->cmd_state = CMD_STATUS_OK;

	set_cmd_result(info, all_strbuff,
			strnlen(all_strbuff, sizeof(all_strbuff)));
	tsp_debug_info(&info->client->dev,
			"%ld (%ld)\n", strnlen(all_strbuff,
			sizeof(all_strbuff)), sizeof(all_strbuff));
}

void fts_read_self_frame(struct fts_ts_info *info, unsigned short oAddr)
{
	char buff[66] = {0, };
	short *data = 0;
	char temp[9] = {0, };
	char temp2[512] = {0, };
	int i = 0;
	int rc = 0;
	unsigned char regAdd[6] = {0xD0, 0x00, 0x00, 0xD0, 0x00, 0x00};

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
			__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	regAdd[1] = (oAddr >> 8) & 0xff;
	regAdd[2] = oAddr & 0xff;
	rc = info->fts_read_reg(info, &regAdd[0], 3, (unsigned char *)&buff[0], 5);
	if (rc < 0) {
		info->cmd_state = CMD_STATUS_FAIL;
		return;
	}

	if (info->digital_rev == FTS_DIGITAL_REV_1) {
		tsp_debug_info(&info->client->dev,
				"%s: Force Address : %02x%02x\n",
				__func__, buff[1], buff[0]);
		tsp_debug_info(&info->client->dev,
				"%s: Sense Address : %02x%02x\n",
				__func__, buff[3], buff[2]);
		regAdd[1] = buff[3];
		regAdd[2] = buff[2];
		regAdd[4] = buff[1];
		regAdd[5] = buff[0];
	} else if (info->digital_rev == FTS_DIGITAL_REV_2) {
		tsp_debug_info(&info->client->dev,
				"%s: Force Address : %02x%02x\n",
				__func__, buff[2], buff[1]);
		tsp_debug_info(&info->client->dev,
				"%s: Sense Address : %02x%02x\n",
				__func__, buff[4], buff[3]);
		regAdd[1] = buff[4];
		regAdd[2] = buff[3];
		regAdd[4] = buff[2];
		regAdd[5] = buff[1];
	}

	rc = info->fts_read_reg(info, &regAdd[0], 3,
			(unsigned char *)&buff[0],
			info->SenseChannelLength * 2 + 1);
	if (rc < 0) {
		info->cmd_state = CMD_STATUS_FAIL;
		return;
	}

	if (info->digital_rev == FTS_DIGITAL_REV_1)
		data = (short *)&buff[0];
	else
		data = (short *)&buff[1];

	memset(temp, 0x00, ARRAY_SIZE(temp));
	memset(temp2, 0x00, ARRAY_SIZE(temp2));

	for (i = 0; i < info->SenseChannelLength; i++) {
		tsp_debug_info(&info->client->dev,
				"%s: Rx [%d] = %d\n", __func__,
				i,
				*data);
		sprintf(temp, "%d,", *data);
		strncat(temp2, temp, 9);
		data++;
	}

	rc = info->fts_read_reg(info, &regAdd[3], 3,
			(unsigned char *)&buff[0],
			info->ForceChannelLength * 2 + 1);
	if (rc < 0) {
		info->cmd_state = CMD_STATUS_FAIL;
		return;
	}

	if (info->digital_rev == FTS_DIGITAL_REV_1)
		data = (short *)&buff[0];
	else
		data = (short *)&buff[1];

	for (i = 0; i < info->ForceChannelLength; i++) {
		tsp_debug_info(&info->client->dev,
				"%s: Tx [%d] = %d\n", __func__, i, *data);
		sprintf(temp, "%d,", *data);
		strncat(temp2, temp, 9);
		data++;
	}

	set_cmd_result(info, temp2, strnlen(temp2, sizeof(temp2)));

	info->cmd_state = CMD_STATUS_OK;
}

static void fts_read_ix_data(struct fts_ts_info *info, bool allnode)
{
	char buff[33] = { 0 };

	unsigned short max_tx_ix_sum = 0;
	unsigned short min_tx_ix_sum = 0xFFFF;

	unsigned short max_rx_ix_sum = 0;
	unsigned short min_rx_ix_sum = 0xFFFF;

	unsigned char tx_ix2[info->ForceChannelLength + 4];
	unsigned char rx_ix2[info->SenseChannelLength + 4];

	unsigned char regAdd[FTS_EVENT_SIZE];
	unsigned short tx_ix1 = 0, rx_ix1 = 0;

	unsigned short force_ix_data[info->ForceChannelLength * 2 + 1];
	unsigned short sense_ix_data[info->SenseChannelLength * 2 + 1];
	int buff_size, j;
	char *mbuff = NULL;
	int num, n, a, fzero;
	char cnum;
	int i = 0;
	int comp_header_addr, comp_start_tx_addr, comp_start_rx_addr;
	unsigned int rx_num, tx_num;

	set_default_result(info);

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	info->fts_irq_enable(info, false);
	info->fts_interrupt_set(info, INT_DISABLE);

	info->fts_command(info, SENSEOFF);

	info->fts_command(info, FLUSHBUFFER); /* Clear FIFO */
	fts_delay(50);

	info->fts_release_all_finger(info);

	/* Request compensation data */
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20; /* SELF IX */
	regAdd[2] = 0x00;
	info->fts_write_reg(info, &regAdd[0], 3);
	fts_fw_wait_for_specific_event(info,
			EVENTID_STATUS_REQUEST_COMP, 0x20, 0x00);

	/* Read an address of compensation data */
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = FTS_SI_COMPENSATION_OFFSET_ADDR;
	info->fts_read_reg(info, regAdd, 3, &buff[0], 3);
	comp_header_addr = buff[1] + (buff[2] << 8);

	/* Read header of compensation area */
	regAdd[0] = 0xD0;
	regAdd[1] = (comp_header_addr >> 8) & 0xFF;
	regAdd[2] = comp_header_addr & 0xFF;
	info->fts_read_reg(info, regAdd, 3, &buff[0], 16 + 1);
	tx_num = buff[5];
	rx_num = buff[6];
	tsp_debug_info(&info->client->dev,
			"%s: [FTS] tx : %d, rx : %d",
			__func__, tx_num, rx_num);

	tx_ix1 = (short) buff[10];
	rx_ix1 = (short) buff[11];

	comp_start_tx_addr = comp_header_addr + 0x10;
	comp_start_rx_addr = comp_start_tx_addr + tx_num;

	memset(tx_ix2, 0x0, tx_num);
	memset(rx_ix2, 0x0, rx_num);

	/* Read Self TX Ix2 */
	regAdd[0] = 0xD0;
	regAdd[1] = (comp_start_tx_addr >> 8) & 0xFF;
	regAdd[2] = comp_start_tx_addr & 0xFF;
	info->fts_read_reg(info, regAdd, 3, &tx_ix2[0], tx_num + 1);

	/* Read Self RX Ix2 */
	regAdd[0] = 0xD0;
	regAdd[1] = (comp_start_rx_addr >> 8) & 0xFF;
	regAdd[2] = comp_start_rx_addr & 0xFF;
	info->fts_read_reg(info, regAdd, 3, &rx_ix2[0], rx_num + 1);

	for (i = 0; i < info->ForceChannelLength; i++) {
		force_ix_data[i] = tx_ix1 + tx_ix2[i + 1];
		if (max_tx_ix_sum < force_ix_data[i])
			max_tx_ix_sum = force_ix_data[i];
		if (min_tx_ix_sum > force_ix_data[i])
			min_tx_ix_sum = force_ix_data[i];
	}

	for (i = 0; i < info->SenseChannelLength; i++) {
		sense_ix_data[i] = rx_ix1 + rx_ix2[i + 1];
		if (max_rx_ix_sum < sense_ix_data[i])
			max_rx_ix_sum = sense_ix_data[i];
		if (min_rx_ix_sum > sense_ix_data[i])
			min_rx_ix_sum = sense_ix_data[i];
	}

	tsp_debug_info(&info->client->dev,
			"%s: MIN_TX_IX_SUM : %d MAX_TX_IX_SUM : %d\n",
	__func__, min_tx_ix_sum, max_tx_ix_sum);
	tsp_debug_info(&info->client->dev,
			"%s: MIN_RX_IX_SUM : %d MAX_RX_IX_SUM : %d\n",
	__func__, min_rx_ix_sum, max_rx_ix_sum);

	info->fts_command(info, SENSEON);

	info->fts_irq_enable(info, true);
	info->fts_interrupt_set(info, INT_ENABLE);

	if (allnode == true) {
		buff_size = (info->ForceChannelLength +
				info->SenseChannelLength + 2) * 5;
		mbuff = kzalloc(buff_size, GFP_KERNEL);
	}
	if (mbuff != NULL) {
		char *pBuf = mbuff;

		for (i = 0; i < info->ForceChannelLength; i++) {
			num =  force_ix_data[i];
			n = 100000;
			fzero = 0;
			for (j = 5; j > 0; j--) {
				n = n / 10;
				a = num / n;
				if (a)
				fzero = 1;
				cnum = a + '0';
				num  = num - a*n;
				if (fzero)
				*pBuf++ = cnum;
			}
			if (!fzero)
			*pBuf++ = '0';
			*pBuf++ = ',';
			tsp_debug_info(&info->client->dev,
					"Force[%d] %d\n", i, force_ix_data[i]);
		}
		for (i = 0; i < info->SenseChannelLength; i++) {
			num =  sense_ix_data[i];
			n = 100000;
			fzero = 0;
			for (j = 5; j > 0; j--) {
				n = n / 10;
				a = num / n;
				if (a)
				fzero = 1;
				cnum = a + '0';
				num  = num - a * n;
				if (fzero)
				*pBuf++ = cnum;
			}
			if (!fzero)
			*pBuf++ = '0';
			if (i < (info->SenseChannelLength - 1))
				*pBuf++ = ',';
			tsp_debug_info(&info->client->dev,
					"Sense[%d] %d\n", i, sense_ix_data[i]);
		}

		set_cmd_result(info, mbuff, buff_size);
		info->cmd_state = CMD_STATUS_OK;
		kfree(mbuff);
	} else {
		if (allnode == true) {
			snprintf(buff, sizeof(buff), "%s", "kzalloc failed");
			info->cmd_state = CMD_STATUS_FAIL;
		} else{
			snprintf(buff, sizeof(buff), "%d,%d,%d,%d",
				min_tx_ix_sum, max_tx_ix_sum,
				min_rx_ix_sum, max_rx_ix_sum);
			info->cmd_state = CMD_STATUS_OK;
		}
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		tsp_debug_info(&info->client->dev,
				"%s: %s\n", __func__, buff);
	}
}

static void run_ix_data_read(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;

	set_default_result(info);
	fts_read_ix_data(info, false);
}

static void run_ix_data_read_all(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;

	set_default_result(info);
	fts_read_ix_data(info, true);
}

static void fts_read_self_raw_frame(struct fts_ts_info *info,
		unsigned short oAddr, bool allnode)
{
	char buff[32 * 2 + 1] = { 0 };
	unsigned char D0_offset = 1;
	unsigned char regAdd[3] = {0xD0, 0x00, 0x00};
	unsigned char ReadData[info->SenseChannelLength * 2 + 1];
	unsigned short self_force_raw_data[info->ForceChannelLength * 2 + 1];
	unsigned short self_sense_raw_data[info->SenseChannelLength * 2 + 1];
	unsigned int FrameAddress = 0;
	unsigned char count = 0;
	int buff_size, i, j;
	char *mbuff = NULL;
	int num, n, a, fzero;
	char cnum;
	unsigned short min_tx_self_raw_data = 0xFFFF;
	unsigned short max_tx_self_raw_data = 0;
	unsigned short min_rx_self_raw_data = 0xFFFF;
	unsigned short max_rx_self_raw_data = 0;

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	info->fts_irq_enable(info, false);
	info->fts_interrupt_set(info, INT_DISABLE);
	info->fts_command(info, SENSEOFF);

	fts_delay(50);

	info->fts_command(info, FLUSHBUFFER); /* Clear FIFO */
	fts_delay(50);

	regAdd[1] = 0x00;
	regAdd[2] = oAddr;
	info->fts_read_reg(info, regAdd, 3, &ReadData[0], 4);

	/* D1 : DOFFSET = 0, D2 : DOFFSET : 1 */
	FrameAddress = ReadData[D0_offset] + (ReadData[D0_offset + 1] << 8);

	regAdd[1] = (FrameAddress >> 8) & 0xFF;
	regAdd[2] = FrameAddress & 0xFF;

	info->fts_read_reg(info, regAdd, 3, &ReadData[0], info->ForceChannelLength * 2 + 1);

	for (count = 0; count < info->ForceChannelLength; count++) {
		self_force_raw_data[count] = ReadData[count*2+D0_offset] + (ReadData[count*2+D0_offset+1]<<8);

		if (max_tx_self_raw_data < self_force_raw_data[count])
			max_tx_self_raw_data = self_force_raw_data[count];
		if (min_tx_self_raw_data > self_force_raw_data[count])
			min_tx_self_raw_data = self_force_raw_data[count];
	}

	regAdd[1] = 0x00;
	regAdd[2] = oAddr + 2;
	info->fts_read_reg(info, regAdd, 3, &ReadData[0], 4);

	FrameAddress = ReadData[D0_offset] + (ReadData[D0_offset + 1] << 8);           /* D1 : DOFFSET = 0, D2 : DOFFSET : 1 */

	regAdd[1] = (FrameAddress >> 8) & 0xFF;
	regAdd[2] = FrameAddress & 0xFF;

	info->fts_read_reg(info, regAdd, 3, &ReadData[0],
			info->SenseChannelLength * 2 + 1);

	for (count = 0; count < info->SenseChannelLength; count++) {
		self_sense_raw_data[count] = ReadData[count*2+D0_offset] +
			(ReadData[count*2+D0_offset+1]<<8);

		if (max_rx_self_raw_data < self_sense_raw_data[count])
			max_rx_self_raw_data = self_sense_raw_data[count];
		if (min_rx_self_raw_data > self_sense_raw_data[count])
			min_rx_self_raw_data = self_sense_raw_data[count];
	}

	tsp_debug_info(&info->client->dev,
			"%s MIN_TX_SELF_RAW: %d MAX_TX_SELF_RAW : %d\n",
			__func__, min_tx_self_raw_data, max_tx_self_raw_data);
	tsp_debug_info(&info->client->dev,
			"%s MIN_RX_SELF_RAW : %d MIN_RX_SELF_RAW : %d\n",
			__func__, min_rx_self_raw_data, max_rx_self_raw_data);

	fts_delay(1);
	info->fts_command(info, SENSEON);

	info->fts_irq_enable(info, true);
	info->fts_interrupt_set(info, INT_ENABLE);

	if (allnode == true) {
		buff_size = (info->ForceChannelLength +
				info->SenseChannelLength + 2)*10;
		mbuff = kzalloc(buff_size, GFP_KERNEL);
	}
	if (mbuff != NULL) {
		char *pBuf = mbuff;

		for (i = 0; i < info->ForceChannelLength; i++) {
			num = self_force_raw_data[i];
			n = 100000;
			fzero = 0;
			for (j = 5; j > 0; j--) {
				n = n/10;
				a = num/n;
				if (a)
					fzero = 1;
				cnum = a + '0';
				num  = num - a*n;
				if (fzero)
					*pBuf++ = cnum;
			}
			if (!fzero)
				*pBuf++ = '0';
			*pBuf++ = ',';
			tsp_debug_info(&info->client->dev, "%d ", self_force_raw_data[i]);
		}
		for (i = 0; i < info->SenseChannelLength; i++) {
			num =  self_sense_raw_data[i];
			n = 100000;
			fzero = 0;
			for (j = 5; j > 0; j--) {
				n = n/10;
				a = num/n;
				if (a)
					fzero = 1;
				cnum = a + '0';
				num  = num - a*n;
				if (fzero)
					*pBuf++ = cnum;
			}
			if (!fzero)
				*pBuf++ = '0';
			if (i < (info->SenseChannelLength-1))
				*pBuf++ = ',';
			tsp_debug_info(&info->client->dev, "%d ", self_sense_raw_data[i]);
		}


		set_cmd_result(info, mbuff, buff_size);
		info->cmd_state = CMD_STATUS_OK;
		kfree(mbuff);
	} else {
		if (allnode == true) {
			snprintf(buff, sizeof(buff), "%s", "kzalloc failed");
			info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		} else{
			snprintf(buff, sizeof(buff), "%d,%d,%d,%d",
				min_tx_self_raw_data,
				max_tx_self_raw_data,
				min_rx_self_raw_data,
				max_rx_self_raw_data);
			info->cmd_state = CMD_STATUS_OK;
		}
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		tsp_debug_info(&info->client->dev, "%s: %s\n",
				__func__, buff);
	}
}

static void run_self_raw_read(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;

	set_default_result(info);
	fts_read_self_raw_frame(info, FTS_WATER_SELF_RAW_ADDR, false);
}

static void run_self_raw_read_all(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;

	set_default_result(info);
	fts_read_self_raw_frame(info, FTS_WATER_SELF_RAW_ADDR, true);
}

static void get_cx_data(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };
	short val = 0;
	int node = 0;

	set_default_result(info);
	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	node = fts_check_index(info);
	if (node < 0)
		return;

	val = info->cx_data[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_OK;
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);

}

static void run_cx_data_read(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;

	char buff[SEC_CMD_STR_LEN] = { 0 };
	unsigned char ReadData[info->ForceChannelLength]
		[info->SenseChannelLength + FTS_CX2_READ_LENGTH];
	int cxdiffData_rx[info->ForceChannelLength *
		(info->SenseChannelLength + FTS_CX2_READ_LENGTH)];
	int cxdiffData_tx[info->ForceChannelLength *
		(info->SenseChannelLength + FTS_CX2_READ_LENGTH)];
	int Max_cxdiffData_rx = 0;
	int Low_cxdiffData_rx = 0;
	int Max_cxdiffData_tx = 0;
	int Low_cxdiffData_tx = 0;
	unsigned char regAdd[8];
	unsigned int addr, rx_num, tx_num;
	int i, j;
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = { 0 };

	int comp_header_addr, comp_start_addr;

	set_default_result(info);

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	pStr = kzalloc(4 * (info->SenseChannelLength + 1), GFP_KERNEL);
	if (pStr == NULL) {
		tsp_debug_info(&info->client->dev,
				"%s: pStr kzalloc failed\n", __func__);
		return;
	}

	tsp_debug_info(&info->client->dev, "%s: start\n", __func__);

	info->fts_irq_enable(info, false);
	tsp_debug_info(&info->client->dev, "%s: disable_irq\n", __func__);
	info->fts_interrupt_set(info, INT_DISABLE);
	tsp_debug_info(&info->client->dev,
			"%s: fts_interrupt_set\n", __func__);
	info->fts_command(info, SENSEOFF);
	tsp_debug_info(&info->client->dev, "%s: fts_command\n", __func__);
	fts_delay(50);
	tsp_debug_info(&info->client->dev, "%s: senseoff\n", __func__);

	info->fts_command(info, FLUSHBUFFER);
	fts_delay(50);

	info->fts_release_all_finger(info);

	/* Request compensation data */
	regAdd[0] = 0xB8;
	regAdd[1] = 0x04; /* MUTUAL CX (LPA) */
	regAdd[2] = 0x00;
	info->fts_write_reg(info, &regAdd[0], 3);
	tsp_debug_info(&info->client->dev,
			"%s: Writing Request compensation data\n", __func__);
	fts_cmd_completion_check(info, EVENTID_STATUS_REQUEST_COMP,
			regAdd[1], regAdd[2]);

	/* Read an address of compensation data */
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	info->fts_read_reg(info, regAdd, 3, &buff[0], 4);
	comp_header_addr = buff[1] + (buff[2] << 8);
	tsp_debug_info(&info->client->dev,
			"%s:Read an address of compensation data\n", __func__);

	/* Read header of compensation area */
	regAdd[0] = 0xD0;
	regAdd[1] = (comp_header_addr >> 8) & 0xFF;
	regAdd[2] = comp_header_addr & 0xFF;
	info->fts_read_reg(info, regAdd, 3, &buff[0], 16 + 1);
	tx_num = buff[5];
	rx_num = buff[6];
	comp_start_addr = comp_header_addr + 0x10;

	tsp_debug_info(&info->client->dev,
			"%s:Read header of compensation area data\n", __func__);
	tsp_debug_info(&info->client->dev,
			"%s: Tx num ( %d ) Rx num ( %d )\n", __func__,
			tx_num, rx_num);
	tsp_debug_info(&info->client->dev,
			"%s: comp_stat_addr : 0x%x\n", __func__,
			comp_start_addr);
	/* Read compensation data */
	for (j = 0; j < tx_num; j++) {
		memset(&ReadData[j], 0x0, rx_num);
		memset(pStr, 0x0, 4 * (rx_num + 1));
		snprintf(pTmp, sizeof(pTmp), "Tx%02d | ", j);
		strncat(pStr, pTmp, 4 * rx_num);

		addr = comp_start_addr + (rx_num * j);
		regAdd[0] = 0xD0;
		regAdd[1] = (addr >> 8) & 0xFF;
		regAdd[2] = addr & 0xFF;
		info->fts_read_reg(info, regAdd, 3, &ReadData[j][0], rx_num + 1);
		for (i = 1; i < rx_num + 1; i++) {
			snprintf(pTmp, sizeof(pTmp), "%3d", ReadData[j][i]);
			strncat(pStr, pTmp, 4 * rx_num);
		}
		tsp_debug_info(&info->client->dev, "%s\n", pStr);
	}

	tsp_debug_info(&info->client->dev,
			"%s:Read compensation data\n", __func__);
	if (info->cx_data) {
		for (j = 0; j < tx_num; j++) {
			for (i = 1; i < rx_num + 1; i++)
				info->cx_data[(j * rx_num) + i - 1] =
					ReadData[j][i];
		}
	}
	tsp_debug_err(&info->client->dev,
			"===================> %s : Rx diff\n", __func__);
	for (j = 0; j < tx_num; j++) {
		memset(pStr, 0x0, 4 * (rx_num + 1));
		snprintf(pTmp, sizeof(pTmp), "Tx%02d | ", j);
		strncat(pStr, pTmp, 16);
		for (i = 0; i < (rx_num-1); i++) {
			cxdiffData_rx[(j*rx_num)+i] =
				info->cx_data[(j*rx_num)+i] -
				info->cx_data[(j*rx_num)+i+1];
			if ((j == 0) && (i == 0)) {
				Max_cxdiffData_rx = cxdiffData_rx[(j*rx_num)+i];
				Low_cxdiffData_rx = cxdiffData_rx[(j*rx_num)+i];
			}
			if (cxdiffData_rx[(j*rx_num)+i] > Max_cxdiffData_rx)
				Max_cxdiffData_rx = cxdiffData_rx[(j*rx_num)+i];
			if (cxdiffData_rx[(j*rx_num)+i] < Low_cxdiffData_rx)
				Low_cxdiffData_rx = cxdiffData_rx[(j*rx_num)+i];
			snprintf(pTmp, sizeof(pTmp), "%4d", cxdiffData_rx[(j*rx_num)+i]);
			strcat(pStr, pTmp);
		}
		tsp_debug_info(&info->client->dev, "FTS %s\n", pStr);
	}
	tsp_debug_err(&info->client->dev,
			"======>  Max : %d / Low : %d <=========\n",
			Max_cxdiffData_rx, Low_cxdiffData_rx);
	tsp_debug_err(&info->client->dev,
			" %s : Tx diff\n  <=======================", __func__);
	for (j = 0; j < (tx_num - 1); j++) {
		memset(pStr, 0x0, 4 * (rx_num + 1));
		snprintf(pTmp, sizeof(pTmp), "Tx%02d | ", j);
		strncat(pStr, pTmp, 16);
		for (i = 0; i < rx_num; i++) {
			cxdiffData_tx[(j*rx_num)+i] =
				info->cx_data[(j*rx_num)+i] -
				info->cx_data[((j+1)*rx_num)+i];
			if ((j == 0) && (i == 0)) {
				Max_cxdiffData_tx = cxdiffData_rx[(j*rx_num)+i];
				Low_cxdiffData_tx = cxdiffData_rx[(j*rx_num)+i];
			}
			if (cxdiffData_tx[(j*rx_num)+i] > Max_cxdiffData_tx)
				Max_cxdiffData_tx = cxdiffData_tx[(j*rx_num)+i];
			if (cxdiffData_tx[(j*rx_num)+i] < Low_cxdiffData_tx)
				Low_cxdiffData_tx = cxdiffData_tx[(j*rx_num)+i];
				snprintf(pTmp, sizeof(pTmp), "%4d",
						cxdiffData_tx[(j*rx_num)+i]);
				strcat(pStr, pTmp);
		}
		tsp_debug_info(&info->client->dev, "FTS %s\n", pStr);
	}
	tsp_debug_err(&info->client->dev,
			"======>  Max : %d / Low : %d <=========\n",
			Max_cxdiffData_tx, Low_cxdiffData_tx);

	kfree(pStr);

	snprintf(buff, sizeof(buff), "%s", "OK");
	info->fts_irq_enable(info, true);
	info->fts_interrupt_set(info, INT_ENABLE);
	info->fts_command(info, SENSEON);
	info->cmd_state = CMD_STATUS_OK;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);

}

static void get_cx_all_data(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	unsigned char ReadData[info->ForceChannelLength]
		[info->SenseChannelLength + FTS_CX2_READ_LENGTH];
	unsigned char regAdd[8];
	unsigned int addr, rx_num, tx_num;
	int i, j;
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = { 0 };
	char all_strbuff[(info->ForceChannelLength)*
		(info->SenseChannelLength)*3];

	int comp_header_addr, comp_start_addr;

	set_default_result(info);

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
			"%s: [ERROR] Touch is stopped\n",
			__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	tsp_debug_info(&info->client->dev, "%s: start\n", __func__);

	info->fts_command(info, SENSEOFF);
	info->fts_irq_enable(info, false);
	info->fts_command(info, FLUSHBUFFER);
	fts_delay(50);

	info->fts_release_all_finger(info);

	tx_num = info->ForceChannelLength;
	rx_num = info->SenseChannelLength;

	pStr = kzalloc(4 * (info->SenseChannelLength + 1), GFP_KERNEL);
	if (pStr == NULL) {
		tsp_debug_info(&info->client->dev,
				"%s: pStr kzalloc failed\n", __func__);
		goto out;
	}

	/* size 3  ex(45,) */
	memset(all_strbuff, 0, sizeof(char) * (tx_num*rx_num*3));

	/* Request compensation data */
	regAdd[0] = 0xB8;
	regAdd[1] = 0x04; /* MUTUAL CX (LPA) */
	regAdd[2] = 0x00;
	info->fts_write_reg(info, &regAdd[0], 3);
	fts_fw_wait_for_specific_event(info, EVENTID_STATUS_REQUEST_COMP, 0x04, 0x00);

	/* Read an address of compensation data */
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = FTS_SI_COMPENSATION_OFFSET_ADDR;
	info->fts_read_reg(info, regAdd, 3, &buff[0], 4);
	comp_header_addr = buff[1] + (buff[2] << 8);

	/* Read header of compensation area */
	regAdd[0] = 0xD0;
	regAdd[1] = (comp_header_addr >> 8) & 0xFF;
	regAdd[2] = comp_header_addr & 0xFF;
	info->fts_read_reg(info, regAdd, 3, &buff[0], 16 + 1);
	tx_num = buff[5];
	rx_num = buff[6];
	comp_start_addr = comp_header_addr + 0x10;

	/* Read compensation data */
	for (j = 0; j < tx_num; j++) {
		memset(&ReadData[j], 0x0, rx_num);
		memset(pStr, 0x0, 4 * (rx_num + 1));
		snprintf(pTmp, sizeof(pTmp), "Tx%02d | ", j);
		strncat(pStr, pTmp, 4 * rx_num);

		addr = comp_start_addr + (rx_num * j);
		regAdd[0] = 0xD0;
		regAdd[1] = (addr >> 8) & 0xFF;
		regAdd[2] = addr & 0xFF;
		info->fts_read_reg(info, regAdd, 3, &ReadData[j][0], rx_num + 1);
		for (i = 0; i < rx_num; i++) {
			snprintf(pTmp, sizeof(pTmp), "%3d", ReadData[j][i]);
			strncat(pStr, pTmp, 4 * rx_num);
		}
		tsp_debug_info(&info->client->dev, "%s\n", pStr);
	}

	if (info->cx_data) {
		for (j = 0; j < tx_num; j++) {
			for (i = 0; i < rx_num; i++) {
				info->cx_data[(j * rx_num) + i] =
					ReadData[j][i];
				snprintf(buff, sizeof(buff),
						"%d,", ReadData[j][i]);
				strncat(all_strbuff, buff, sizeof(buff));
			}
		}
	}

	kfree(pStr);

out:
	info->fts_irq_enable(info, true);
	info->fts_command(info, SENSEON);
	info->cmd_state = CMD_STATUS_OK;
	set_cmd_result(info, all_strbuff,
			strnlen(all_strbuff, sizeof(all_strbuff)));
	tsp_debug_info(&info->client->dev,
			"%s: %ld (%ld)\n", __func__,
			strnlen(all_strbuff, sizeof(all_strbuff)),
			sizeof(all_strbuff));
}


static void set_tsp_test_result(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };
	unsigned char regAdd[4] = {0xB0, 0x07, 0xE7, 0x00};

	set_default_result(info);

	if (info->cmd_param[0] < TSP_FACTEST_RESULT_NONE
			|| info->cmd_param[0] > TSP_FACTEST_RESULT_PASS) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
		return;
	}

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	info->fts_irq_enable(info, false);
	info->fts_interrupt_set(info, INT_DISABLE);

	regAdd[3] = info->cmd_param[0];
	info->fts_write_reg(info, &regAdd[0], 4);
	fts_delay(100);
	info->fts_command(info, FTS_CMD_SAVE_FWCONFIG);

	fts_delay(230);
	fts_fw_wait_for_event(info, STATUS_EVENT_FLASH_WRITE_CONFIG, 0x00);

	info->fts_irq_enable(info, true);
	info->fts_interrupt_set(info, INT_ENABLE);

	snprintf(buff, sizeof(buff), "%s", "OK");
	info->cmd_state = CMD_STATUS_OK;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_tsp_test_result(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	unsigned char cmd[4] = {0xB2, 0x07, 0xE7, 0x01};
	char buff[CMD_STR_LEN] = { 0 };
	int timeout = 0;

	set_default_result(info);

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	info->fts_command(info, FLUSHBUFFER);
	info->fts_write_reg(info, &cmd[0], 4);
	info->cmd_state = CMD_STATUS_RUNNING;

	while (info->cmd_state == CMD_STATUS_RUNNING) {
		if (timeout++ > 30) {
			info->cmd_state = CMD_STATUS_FAIL;
			break;
		}
		fts_delay(10);
	}
}

static void run_trx_short_test(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };
	int ret = 0;

	set_default_result(info);
	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}
	info->fts_irq_enable(info, false);
	ret = fts_panel_ito_test(info);
	if (ret == 1)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "FAIL");
	info->fts_irq_enable(info, true);
	info->cmd_state = CMD_STATUS_OK;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void report_rate(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };

	set_default_result(info);

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		goto out;
	}

	if (info->cmd_param[0] < 0 || info->cmd_param[0] > 2) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		info->cmd_state = CMD_STATUS_FAIL;
	} else {
		snprintf(buff, sizeof(buff), "%s", "OK");
		info->cmd_state = CMD_STATUS_OK;
	}
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = CMD_STATUS_WAITING;

out:
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void delay(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };

	set_default_result(info);

	info->delay_time = info->cmd_param[0];

	tsp_debug_info(&info->client->dev,
			"%s: delay time is %d\n", __func__, info->delay_time);
	snprintf(buff, sizeof(buff), "%d", info->delay_time);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);
	info->cmd_state = CMD_STATUS_WAITING;

	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void debug(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };

	set_default_result(info);

	info->debug_string = info->cmd_param[0];

	tsp_debug_info(&info->client->dev,
			"%s: command is %d\n", __func__, info->debug_string);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);
	info->cmd_state = CMD_STATUS_WAITING;

	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void run_autotune_enable(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };

	set_default_result(info);

	info->run_autotune = info->cmd_param[0];

	tsp_debug_info(&info->client->dev, "%s: command is %s\n",
			__func__, info->run_autotune ? "ENABLE" : "DISABLE");

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);
	info->cmd_state = CMD_STATUS_WAITING;

	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void run_autotune(void *device_data)
{
	struct fts_ts_info *info = (struct fts_ts_info *)device_data;
	char buff[CMD_STR_LEN] = { 0 };

	set_default_result(info);

	if (info->touch_stopped) {
		dev_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n", __func__);
	}

	if (info->touch_stopped) {
		tsp_debug_info(&info->client->dev,
				"%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = CMD_STATUS_NOT_APPLICABLE;
		return;
	}

	if (!info->run_autotune) {
		tsp_debug_info(&info->client->dev,
				"%s: autotune is disabled, %d\n",
				__func__, info->run_autotune);
		goto autotune_fail;
	}

	info->fts_irq_enable(info, false);

	if (info->digital_rev == FTS_DIGITAL_REV_2) {
		info->fts_interrupt_set(info, INT_DISABLE);

		info->fts_command(info, SENSEOFF);
		fts_delay(50);

		info->fts_command(info, FTS_CMD_TRIM_LOW_POWER_OSCILLATOR);
		fts_delay(200);

		info->fts_command(info, FLUSHBUFFER);

		info->fts_release_all_finger(info);
		fts_execute_autotune(info);

		info->fts_command(info, SENSEON);

		info->fts_interrupt_set(info, INT_ENABLE);
	} else {
		tsp_debug_info(&info->client->dev,
				"%s: digital_rev not matched, %d\n",
				__func__, info->digital_rev);
		goto autotune_fail;
	}

	info->fts_irq_enable(info, true);
	snprintf(buff, sizeof(buff), "%s", "OK");
	info->cmd_state = CMD_STATUS_OK;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
	return;

autotune_fail:
	snprintf(buff, sizeof(buff), "%s", "NG");
	info->cmd_state = CMD_STATUS_FAIL;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	tsp_debug_info(&info->client->dev, "%s: %s\n", __func__, buff);
	return;
}

void fts_production_init(void *device_info)
{
	char pdc_dir_name[20] = {0, };
	int j = 0;
	struct fts_ts_info *info = (struct fts_ts_info *)device_info;
	int retval = 0;

	INIT_LIST_HEAD(&info->cmd_list_head);

	info->cmd_buf_size = 0;
	for (j = 0; j < ARRAY_SIZE(fts_commands); j++) {
		list_add_tail(&fts_commands[j].list, &info->cmd_list_head);
		if (fts_commands[j].cmd_name)
			info->cmd_buf_size += strlen(fts_commands[j].cmd_name) + 1;
	}

	mutex_init(&info->cmd_lock);
	info->cmd_is_running = false;

	sprintf(pdc_dir_name, "ftm4_touch");

	info->pdc_dev_ts = device_create(info->input_dev->dev.class,
			NULL, 0,  NULL, pdc_dir_name);
	if (IS_ERR(info->pdc_dev_ts)) {
		tsp_debug_err(&info->client->dev,
				"FTS Failed to create device for the sysfs\n");
		retval = -ENOENT;
		goto err_sysfs;
	}

	dev_set_drvdata(info->pdc_dev_ts, info);

	retval = sysfs_create_group(&info->pdc_dev_ts->kobj,
				 &touch_pdc_attr_group);
	if (retval < 0) {
		tsp_debug_err(&info->client->dev,
				"FTS Failed to create sysfs group\n");
		goto err_sysfs;
	}


	if (retval < 0) {
		tsp_debug_err(&info->client->dev,
				"%s: Failed to create link\n", __func__);
		goto err_sysfs;
	}

	return;
err_sysfs:
	mutex_destroy(&info->cmd_lock);
}
EXPORT_SYMBOL(fts_production_init);
