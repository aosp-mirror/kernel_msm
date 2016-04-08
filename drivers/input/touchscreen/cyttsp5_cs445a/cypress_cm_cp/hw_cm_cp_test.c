/******************************************************************************
 * @file hw_cm_cp_test.c
 *
 * version.h
 *
 * @version 0.0.1
 * @authors
 *
 *****************************************************************************//*
 * Copyright (2014), Cypress Semiconductor Corporation. All rights reserved.
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
 *****************************************************************************/
#include "../cyttsp5_regs.h"
#include "../cyttsp5_core.h"
#include "csv.h"
#include "cm_cp_test.h"
#include <linux/proc_fs.h>

 struct cyttsp5_panel_info {
    u8 *product_name;
    u8 *chip_name;
    u8 panel_id;
    u8 *id2str;
 };

 static struct cyttsp5_panel_info hw_panel_info[] = {
    {"x1s","tma568", 0,"ofilm" },
    {"x1s","tma568", 1,"lens"    },
    {"Gemini","tma568",0,"ofilm"},
    {"Gemini","tma568",1,"lens"},
    {"mozart","CS448", 0,"ofilm"},
    {"mozart","CS448", 1,"truly"},
    {"liszt","CS448", 1,"truly"},
    {"liszt","CS448", 0,"mutto"},
    {"T2A","CS448", 0,"ofilm"},
    {"T2A","CS448", 3,"mutto"},
 };

#define CM_CP_TEST_SUCCESS 0
#define CM_CP_TEST_FAILED -1

 enum tp_test_result
{
    TP_FAIL,
    TP_PASS,
    TP_UNKNOWN,
    TP_INV_DEV,
};

#define CONFIG_NAME_LENGTH 100
atomic_t mmi_test_status = ATOMIC_INIT(0);
#define RAW_DATA_SIZE (PAGE_SIZE * 32)
static struct cyttsp5_core_commands *cmd;

#define TP_CONFIG_FILE_PATH "/system/etc/tp_test_parameters/"

extern struct device *gdev;
extern void cyttsp5_start_wd_timer(struct cyttsp5_core_data *cd);
extern void cyttsp5_stop_wd_timer(struct cyttsp5_core_data *cd);

int cyttsp5_command_response(struct device *dev,u8 *buf)
{
    int ret = -1;
    ssize_t num_read;
    struct cyttsp5_device_access_data *dad = cyttsp5_get_device_access_data(dev);

    mutex_lock(&dad->sysfs_lock);
    if (!dad->status) {
        goto error;
    }
    num_read = dad->response_length;
    memcpy(buf,dad->response_buf,num_read);
    if (num_read > 2)/* when length <= 2 we can't get any data*/
        ret = num_read;
error:
    mutex_unlock(&dad->sysfs_lock);
    return ret;
}

int cyttsp5_send_command(struct device *dev,char *buf)
{
    struct cyttsp5_device_access_data *dad
        = cyttsp5_get_device_access_data(dev);
    ssize_t length = 0;
    int rc = -1;

    mutex_lock(&dad->sysfs_lock);
    dad->status = 0;
    dad->response_length = 0;
    length = cyttsp5_ic_parse_input(dev, buf, strlen(buf), dad->ic_buf,
            CY_MAX_PRBUF_SIZE);
    if (length <= 0) {
        tp_log_err("%s: %s Group Data store\n", __func__,"Malformed input for");
        goto exit;
    }
    /* write ic_buf to log */
    cyttsp5_pr_buf(dev, dad->ic_buf, length, "ic_buf");

    rc = cmd->nonhid_cmd->user_cmd(dev, 1, CY_MAX_PRBUF_SIZE,
            dad->response_buf, length, dad->ic_buf,
            &dad->response_length);
    if (rc) {
        dad->response_length = 0;
        tp_log_err("%s: Failed to store command\n", __func__);
    } else {
        dad->status = 1;
    }
    tp_log_debug("%s send cmd  ok.\n",__func__);
exit:
    mutex_unlock(&dad->sysfs_lock);
    tp_log_debug("%s: return rc = %d\n", __func__, rc);
    return rc;
}

static int cyttsp5_get_tp_config_file(struct device *dev, char *parameter_file, char *config_file)
{
    int error = 0;
    u8 panel_id = 0;
    int length = 0;
    int i = 0;
    struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
    const char *product_name = cd->cpdata->product_name;
    const char *chip_name = cd->cpdata->chip_name;

    if(parameter_file == NULL || config_file == NULL) {
        tp_log_err("%s: paramaters is invalid\n",
                __func__);
        return -EINVAL;
    }
    cmd = cyttsp5_get_commands();
    if (!cmd) {
        error = -EINVAL;
        tp_log_err("%s Failed to cyttsp5_get_commands.\n",__func__);
        goto out;
    };
    error = cmd->nonhid_cmd->get_panel_id(dev, 1, &panel_id);
    if (error < 0) {
        error = -EINVAL;
        tp_log_err("%s Failed to get panel id.\n",__func__);
        goto out;
    }
    tp_log_info("%s, panel_id = %d\n", __func__, panel_id);
    strncat(parameter_file,TP_CONFIG_FILE_PATH, strlen(TP_CONFIG_FILE_PATH));
    strncat(parameter_file,product_name,strlen(product_name));
    strncat(parameter_file,"_",strlen("_"));
    strncat(parameter_file,"parameters.csv",strlen("parameters.csv"));
    tp_log_info("%s, parameters = %s\n", __func__, parameter_file);
    strncat(config_file,TP_CONFIG_FILE_PATH, strlen(TP_CONFIG_FILE_PATH));
    strncat(config_file,product_name,strlen(product_name));
    strncat(config_file,"_",strlen("_"));
    length = sizeof(hw_panel_info)/sizeof(struct cyttsp5_panel_info);
    for(i = 0; i < length; i++) {
        if(panel_id == hw_panel_info[i].panel_id) {
            error = strncasecmp(hw_panel_info[i].product_name, product_name, strlen(hw_panel_info[i].product_name));
            if(error == 0) {
                error = strncasecmp(hw_panel_info[i].chip_name, chip_name, strlen(hw_panel_info[i].chip_name));
                if(error == 0) {
                    strncat(config_file,hw_panel_info[i].id2str,strlen(hw_panel_info[i].id2str));
                }
            }
        }
    }
    strncat(config_file,"_",strlen("_"));
    strncat(config_file,"input_format.csv",strlen("input_format.csv"));
    tp_log_info("%s, config_file = %s\n", __func__, config_file);
    error = 0;
out:
    return error;
}

int cyttsp5_cm_cp_test(struct seq_file *m, void *v)
{
    int error = 0;
    struct device *dev = gdev;
    struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
    char parameter_filename[CONFIG_NAME_LENGTH] = {0};
    char config_filename[CONFIG_NAME_LENGTH] = {0};
    struct file *config_file = NULL;
    struct file *parameter_file = NULL;
    bool run_cm_test = true, run_cp_test = true;
    bool cm_test_pass = 1;
    bool cp_test_pass = 1;
    int vdda = 0;
    int cp_ret=0;
    int cm_ret=0;
    mm_segment_t fs;

    if(atomic_read(&mmi_test_status)){
        tp_log_err("%s cm_cp_test already has been called.\n",__func__);
        return -1;
    }
    atomic_set(&mmi_test_status, 1);
    tp_log_info("%s, cm_cp_test proc buffer size:%u\n", __func__, (unsigned int)m->size);
    if(m->size <= RAW_DATA_SIZE/4) {
        m->count = m->size;
        goto out;
    }
    error = cyttsp5_get_tp_config_file(dev,parameter_filename, config_filename);
    if(error < 0){
        tp_log_err("%s, get tp config failed\n",__func__);
        goto out;
    }
    //open config name
    fs =get_fs();
    set_fs(KERNEL_DS);
    config_file = filp_open(config_filename, O_RDONLY, 0);
    if (IS_ERR(config_file)){
        tp_log_err("%s,  filp_open error, file name is %s.\n", __func__, config_filename);
        error = -1;
        goto exit_open_config_file;
    }
    tp_log_debug("%s,  filp_open %s success.\n", __func__, config_filename);
    parameter_file = filp_open(parameter_filename, O_RDONLY, 0);
    if (IS_ERR(parameter_file)){
        tp_log_err("%s,  filp_open error, file name is %s.\n", __func__, parameter_filename);
        error = -1;
        goto exit_open_parameter_file;
    }
    tp_log_debug("%s,  filp_open %s success.\n", __func__, parameter_filename);
    cyttsp5_stop_wd_timer(cd);
    error = cm_cp_test_run(NULL, parameter_file, config_file,
                        m, vdda, run_cm_test, run_cp_test,
                        &cm_test_pass, &cp_test_pass);
    if (run_cm_test){
          if (cm_test_pass){
              tp_log_info("%s:PASS: Cm test\n" ,__FUNCTION__ );
              cm_ret = TP_PASS;
          } else {
              tp_log_err("%s:FAIL: Cm test\n", __FUNCTION__ );
              cm_ret = TP_FAIL;
          }
    }
    if (run_cp_test){
         if (cp_test_pass){
             tp_log_info("%s:PASS: Cp test\n" ,__FUNCTION__ );
             cp_ret = TP_PASS;
         }else {
             tp_log_err("%s:FAIL: Cp test\n", __FUNCTION__ );
             cp_ret = TP_FAIL;
        }
    }
    if( (error == 0) && cm_ret &&cp_ret) {
        tp_log_info("%s, cm_cp_test success\n", __func__);
    }
    else {
        tp_log_err("%s, cm_cp_test failed\n", __func__);
    }
    cyttsp5_start_wd_timer(cd);
    filp_close(parameter_file, NULL);
exit_open_parameter_file:
    filp_close(config_file, NULL);
exit_open_config_file:
    set_fs(fs);
out:
    atomic_set(&mmi_test_status, 0);
    tp_log_info("%s, cm_cp_test proc done\n", __func__);
    return error;
}

static int cyttsp5_cm_cp_open(struct inode *inode, struct file *file)
{
    return single_open(file, cyttsp5_cm_cp_test, NULL);
}

static const struct file_operations cyttsp5_cm_cp_proc_fops = {
    .open = cyttsp5_cm_cp_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

void cyttsp5_procfs_create(void)
{
    if (!proc_mkdir("touchscreen", NULL)){
        tp_log_err("%s: Error, failed to creat procfs.\n",__func__);
        return;
    }
    proc_create("touchscreen/cm_cp_test", 0444, NULL, &cyttsp5_cm_cp_proc_fops);
    return;
}

