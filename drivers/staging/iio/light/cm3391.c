/* drivers/input/misc/cm3391.c - cm3391 sensor driver
 *
 * Copyright (C) 2016 Vishay Capella Microsystems. Inc.
 * Author: Frank Hsieh <frank.hsieh@vishay.com>
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

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <linux/cm3391.h>
#include <asm/setup.h>
#include <linux/jiffies.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define LS_POLLING_DELAY 600

#define REL_CHN_RED		REL_X
#define REL_CHN_GREEN	REL_Y
#define REL_CHN_BLUE	REL_Z
#define REL_CHN_X	    REL_RX
#define REL_CHN_Y     REL_RY
#define REL_CHN_Z	    REL_RZ
#define REL_CHN_LUX	  REL_HWHEEL
#define REL_CHN_LIGHT	REL_MISC

#define CHANNEL_SEL_R CM3391_RD_SEL_R
#define CHANNEL_SEL_G CM3391_RD_SEL_G
#define CHANNEL_SEL_B CM3391_RD_SEL_B

#define CHANNEL_SEL_X CM3391_RD_SEL_X
#define CHANNEL_SEL_Y CM3391_RD_SEL_Y
#define CHANNEL_SEL_Z CM3391_RD_SEL_Z

/* lux_gain factors: nomerator, denomerator, IT_CODE*/
static const int lux_gain[][3] = {
    {1, 8, CM3391_IT_50MS}, {1, 4, CM3391_IT_100MS}, {1, 2, CM3391_IT_200MS},
    {1, 1, CM3391_IT_400MS}, {2, 1, CM3391_IT_800MS}
};

/* hd_gain factors: nomerator, denomerator, HD_CODE*/
static const int hd_gain[][3] = {
    {2, 1, CM3391_HD_DISABLE},
    {1, 1, CM3391_HD_ENABLE}
};

static void report_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_work, report_do_work);

struct cm3391_info {
    struct class *cm3391_class;
    struct device *ls_dev;
    struct input_dev *ls_input_dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    struct i2c_client *i2c_client;
    struct workqueue_struct *lp_wq;

    int als_enable;
    int als_enabled_before_suspend;

    int ls_calibrate;
    int (*power)(int, uint8_t); /* power to the chip */
    uint16_t RGB_slave_address;

    int HDCode;
    int it_setting;

    int lux_factor_num;
    int lux_factor_denum;
    int HD_factor_num;
    int HD_factor_denum;

    int lightsensor_opened;
    int polling_delay;
};

struct cm3391_info *lp_info;
int enable_log = 0;
static uint16_t ALS_CONF = 0;

static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int lightsensor_enable(struct cm3391_info *lpi);
static int lightsensor_disable(struct cm3391_info *lpi);

static uint16_t cm3391_adc_red, cm3391_adc_green, cm3391_adc_blue, cm3391_adc_white;
static uint16_t cm3391_adc_ir, cm3391_adc_uv;

static void setupLuxGain(int it){
    int i;
    struct cm3391_info *lpi = lp_info;

    for(i=0;i<sizeof(lux_gain[0])/sizeof(lux_gain[0][0]);i++){
        if(lux_gain[i][2] == it)
            break;
    }
    lpi->lux_factor_num   = lux_gain[i][0];
    lpi->lux_factor_denum = lux_gain[i][1];

    return;
}

static void setupHDGain(int hdCode){
    int i;
    struct cm3391_info *lpi = lp_info;

    for(i=0;i<sizeof(hd_gain[0])/sizeof(hd_gain[0][0]);i++){
        if(hd_gain[i][2] == hdCode)
            break;
    }

    lpi->HD_factor_num   = hd_gain[i][0];
    lpi->HD_factor_denum = hd_gain[i][1];

    return;
}


static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
    uint8_t loop_i;
    uint8_t subaddr[1];

    struct i2c_msg msg[] = {
        {
            .addr = slaveAddr,
            .flags = 0,
            .len = 1,
            .buf = subaddr,
        },
        {
            .addr = slaveAddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxData,
        },
    };

    subaddr[0] = cmd;

    for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

        if (i2c_transfer(lp_info->i2c_client->adapter, msg, 2) > 0)
            break;

        msleep(10);
    }
    if (loop_i >= I2C_RETRY_COUNT) {
        printk(KERN_ERR "[LS][CM3391 error] %s retry over %d\n",
                __func__, I2C_RETRY_COUNT);
        return -EIO;
    }

    return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
    uint8_t loop_i;

    struct i2c_msg msg[] = {
        {
            .addr = slaveAddr,
            .flags = 0,
            .len = length,
            .buf = txData,
        },
    };

    for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
        if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
            break;

        msleep(10);
    }

    if (loop_i >= I2C_RETRY_COUNT) {
        printk(KERN_ERR "[ERR][CM3391 error] %s retry over %d\n",
                __func__, I2C_RETRY_COUNT);
        return -EIO;
    }

    return 0;
}


static int _cm3391_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
    uint8_t buffer[2];
    int ret = 0;

    if (pdata == NULL)
        return -EFAULT;

    ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
    if (ret < 0) {
        pr_err(
                "[ERR][CM3391 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
                __func__, slaveAddr, cmd);
        return ret;
    }

    *pdata = (buffer[1]<<8)|buffer[0];
#if 0
    /* Debug use */
    printk(KERN_DEBUG "[CM3391] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
            __func__, slaveAddr, cmd, *pdata);
#endif
    return ret;
}

static int _cm3391_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
    char buffer[3];
    int ret = 0;
#if 0
    /* Debug use */
    printk(KERN_DEBUG
            " %s: _cm3391_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
            __func__, SlaveAddress, cmd, data);
#endif

    buffer[0] = cmd;
    buffer[1] = (uint8_t)(data&0xff);
    buffer[2] = (uint8_t)((data&0xff00)>>8);

    ret = I2C_TxData(SlaveAddress, buffer, 3);
    if (ret < 0) {
        pr_err("[ERR][CM3391 error]%s: I2C_TxData fail\n", __func__);
        return -EIO;
    }

    return ret;
}

static void report_lsensor_input_event(struct cm3391_info *lpi, bool resume)
{

    /*when resume need report a data, so the paramerter need to quick reponse*/
    uint16_t slaveAddr;
    long lux;
    int lux_gain_num, lux_gain_denum, hd_gain_num, hd_gain_denum;

    slaveAddr = lpi->RGB_slave_address;

    lux_gain_num    = lpi->lux_factor_num;
    lux_gain_denum  = lpi->lux_factor_denum;
    hd_gain_num     = lpi->HD_factor_num;
    hd_gain_denum   = lpi->HD_factor_denum;

    mutex_lock(&als_get_adc_mutex);

    _cm3391_I2C_Read_Word(slaveAddr, CHANNEL_SEL_R, &cm3391_adc_red);
    _cm3391_I2C_Read_Word(slaveAddr, CHANNEL_SEL_G, &cm3391_adc_green);
    _cm3391_I2C_Read_Word(slaveAddr, CHANNEL_SEL_B, &cm3391_adc_blue);

    _cm3391_I2C_Read_Word(slaveAddr, CHANNEL_SEL_X, &cm3391_adc_white);
    _cm3391_I2C_Read_Word(slaveAddr, CHANNEL_SEL_Y, &cm3391_adc_ir);
    _cm3391_I2C_Read_Word(slaveAddr, CHANNEL_SEL_Z, &cm3391_adc_uv);

    //calculate lux
    //lux = (uint16_t) (0.176767*((float)g_val/(lux_gain*HD_gain )));
    lux =  (cm3391_adc_green * lux_gain_denum) / lux_gain_num  * hd_gain_denum / hd_gain_num ;
    lux =  lux * 17676 / 100000;

    input_report_rel(lpi->ls_input_dev, REL_CHN_RED,   (int)cm3391_adc_red);
    input_report_rel(lpi->ls_input_dev, REL_CHN_GREEN, (int)cm3391_adc_green);
    input_report_rel(lpi->ls_input_dev, REL_CHN_BLUE,  (int)cm3391_adc_blue);
    input_report_rel(lpi->ls_input_dev, REL_CHN_X,     (int)cm3391_adc_white);
    input_report_rel(lpi->ls_input_dev, REL_CHN_Y,     (int)cm3391_adc_ir);
    input_report_rel(lpi->ls_input_dev, REL_CHN_Z,     (int)cm3391_adc_uv);
    input_report_rel(lpi->ls_input_dev, REL_CHN_LUX,   (int)lux);

    input_report_rel(lpi->ls_input_dev, REL_CHN_LIGHT, (int)1);

    input_sync(lpi->ls_input_dev);
    D("[LS][CM3391] %s %x %x %x %x %x %x %x \n", __func__,
            cm3391_adc_red, cm3391_adc_green, cm3391_adc_blue,
            cm3391_adc_white, cm3391_adc_ir, cm3391_adc_uv, (int)lux
     );

    mutex_unlock(&als_get_adc_mutex);

}

static void report_do_work(struct work_struct *work)
{
    struct cm3391_info *lpi = lp_info;

    if (enable_log)
        D("[CM3391] %s\n", __func__);

    report_lsensor_input_event(lpi, 0);

    queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
}

static int als_power(int enable)
{
    struct cm3391_info *lpi = lp_info;

    if (lpi->power)
        lpi->power(LS_PWR_ON, 1);

    return 0;
}

static int lightsensor_enable(struct cm3391_info *lpi)
{
    int ret = 0;
    uint16_t cmd = 0;
    uint16_t slaveAddr = lpi->RGB_slave_address;

    mutex_lock(&als_enable_mutex);
    D("[LS][CM3391] %s\n", __func__);

    cmd = ALS_CONF & CM3391_SD_MASK ;
    ret = _cm3391_I2C_Write_Word(slaveAddr, CM3391_CMD_Addr, cmd);
    if (ret < 0)
        pr_err(
                "[LS][CM3391 error]%s: set auto light sensor fail\n",
                __func__);
    else {
        msleep(160);/*wait for 50 ms for the first report adc*/
        /* report an invalid value first to ensure we
         * trigger an event when adc_level is zero.
         */
        //input_report_rel(lpi->ls_input_dev, REL_MISC, -1);
        //input_sync(lpi->ls_input_dev);

        report_lsensor_input_event(lpi, 1);/*resume, IOCTL and DEVICE_ATTR*/
        lpi->als_enable = 1;
    }

    queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
    lpi->als_enable=1;
    mutex_unlock(&als_enable_mutex);

    return ret;
}

static int lightsensor_disable(struct cm3391_info *lpi)
{
    int ret = 0;
    uint16_t cmd = 0;
    uint16_t slaveAddr = lpi->RGB_slave_address;

    mutex_lock(&als_disable_mutex);

    D("[LS][CM3391] %s\n", __func__);

    cmd = ALS_CONF | CM3391_SD ;
    ret = _cm3391_I2C_Write_Word(slaveAddr, CM3391_CMD_Addr, cmd);
    if (ret < 0)
        pr_err("[LS][CM3391 error]%s: disable auto light sensor fail\n",
                __func__);
    else {
        lpi->als_enable = 0;
    }

    cancel_delayed_work_sync(&report_work);

    lpi->als_enable=0;
    mutex_unlock(&als_disable_mutex);

    return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
    struct cm3391_info *lpi = lp_info;
    int rc = 0;

    D("[LS][CM3391] %s\n", __func__);
    if (lpi->lightsensor_opened) {
        pr_err("[LS][CM3391 error]%s: already opened\n", __func__);
        rc = -EBUSY;
    }
    lpi->lightsensor_opened = 1;
    return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
    struct cm3391_info *lpi = lp_info;

    D("[LS][CM3391] %s\n", __func__);
    lpi->lightsensor_opened = 0;
    return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)
{
    int rc, val;
    struct cm3391_info *lpi = lp_info;

    /*D("[CM3391] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

    switch (cmd) {
        case LIGHTSENSOR_IOCTL_ENABLE:
            if (get_user(val, (unsigned long __user *)arg)) {
                rc = -EFAULT;
                break;
            }
            D("[LS][CM3391] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
                    __func__, val);
            rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
            break;
        case LIGHTSENSOR_IOCTL_GET_ENABLED:
            val = lpi->als_enable;
            D("[LS][CM3391] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
                    __func__, val);
            rc = put_user(val, (unsigned long __user *)arg);
            break;
        default:
            pr_err("[LS][CM3391 error]%s: invalid cmd %d\n",
                    __func__, _IOC_NR(cmd));
            rc = -EINVAL;
    }

    return rc;
}

static const struct file_operations lightsensor_fops = {
    .owner = THIS_MODULE,
    .open = lightsensor_open,
    .release = lightsensor_release,
    .unlocked_ioctl = lightsensor_ioctl
};

static ssize_t ls_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct cm3391_info *lpi = lp_info;

    ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
            lpi->als_enable);

    return ret;
}

static ssize_t ls_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret = 0;
    int ls_auto;
    struct cm3391_info *lpi = lp_info;

    ls_auto = -1;
    sscanf(buf, "%d", &ls_auto);

    if (ls_auto != 0 && ls_auto != 1 )
        return -EINVAL;

    if (ls_auto) {
        ret = lightsensor_enable(lpi);
    } else {
        ret = lightsensor_disable(lpi);
    }

    D("[LS][CM3391] %s: lpi->als_enable = %d, lpi->ls_calibrate = %d, ls_auto=%d\n",
            __func__, lpi->als_enable, lpi->ls_calibrate, ls_auto);

    if (ret < 0)
        pr_err(
                "[LS][CM3391 error]%s: set auto light sensor fail\n",
                __func__);

    return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct cm3391_info *lpi = lp_info;

    ret = sprintf(buf, "Light sensor Poll Delay = %d ms\n",
            jiffies_to_msecs(lpi->polling_delay));

    return ret;
}

static ssize_t ls_poll_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int new_delay;
    struct cm3391_info *lpi = lp_info;

    sscanf(buf, "%d", &new_delay);

    D("new delay = %d ms, old delay = %d ms \n",
            new_delay, jiffies_to_msecs(lpi->polling_delay));

    lpi->polling_delay = msecs_to_jiffies(new_delay);

    if( lpi->als_enable ){
        lightsensor_disable(lpi);
        lightsensor_enable(lpi);
    }

    return count;
}

static ssize_t ls_conf_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "ALS_CONF = %x\n", ALS_CONF);
}
static ssize_t ls_conf_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int value = 0;
    uint16_t slaveAddr;
    struct cm3391_info *lpi = lp_info;

    sscanf(buf, "0x%x", &value);

    slaveAddr = lpi->RGB_slave_address;

    ALS_CONF = value;
    printk(KERN_INFO "[LS]set ALS_CONF = %x\n", ALS_CONF);
    _cm3391_I2C_Write_Word(slaveAddr, CM3391_CMD_Addr, ALS_CONF);
    return count;
}

static ssize_t ls_red_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cm3391_adc_red);
}

static ssize_t ls_green_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cm3391_adc_green);
}

static ssize_t ls_blue_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cm3391_adc_blue);
}

static ssize_t ls_white_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cm3391_adc_white);
}

static ssize_t ls_ir_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cm3391_adc_ir);
}

static ssize_t ls_uv_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%x\n", cm3391_adc_uv);
}

static struct device_attribute dev_attr_light_enable =
__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_light_poll_delay =
__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, ls_poll_delay_show, ls_poll_delay_store);

static struct device_attribute dev_attr_light_conf =
__ATTR(conf, S_IRUGO | S_IWUSR | S_IWGRP, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_light_red =
__ATTR(in_intensity_red, S_IRUGO | S_IRUSR | S_IRGRP, ls_red_show, NULL);

static struct device_attribute dev_attr_light_green =
__ATTR(in_intensity_green, S_IRUGO | S_IRUSR | S_IRGRP, ls_green_show, NULL);

static struct device_attribute dev_attr_light_blue =
__ATTR(in_intensity_blue, S_IRUGO | S_IRUSR | S_IRGRP, ls_blue_show, NULL);

static struct device_attribute dev_attr_light_white =
__ATTR(in_intensity_white, S_IRUGO | S_IRUSR | S_IRGRP, ls_white_show, NULL);

static struct device_attribute dev_attr_light_ir =
__ATTR(in_intensity_ir, S_IRUGO | S_IRUSR | S_IRGRP, ls_ir_show, NULL);

static struct device_attribute dev_attr_light_uv =
__ATTR(in_intensity_uv, S_IRUGO | S_IRUSR | S_IRGRP, ls_uv_show, NULL);

static struct attribute *light_sysfs_attrs[] = {
    &dev_attr_light_enable.attr,
    &dev_attr_light_poll_delay.attr,
    &dev_attr_light_conf.attr,
    &dev_attr_light_red.attr,
    &dev_attr_light_green.attr,
    &dev_attr_light_blue.attr,
    &dev_attr_light_white.attr,
    &dev_attr_light_ir.attr,
    &dev_attr_light_uv.attr,
    NULL
};

static struct attribute_group light_attribute_group = {
    .attrs = light_sysfs_attrs,
};

static int lightsensor_setup(struct cm3391_info *lpi)
{
    int ret;

    lpi->ls_input_dev = input_allocate_device();
    if (!lpi->ls_input_dev) {
        pr_err(
                "[LS][CM3391 error]%s: could not allocate ls input device\n",
                __func__);
        return -ENOMEM;
    }
    lpi->ls_input_dev->name = "cm3391-ls";

    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_RED);
    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_GREEN);
    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_BLUE);
    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_X);
    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_Y);
    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_Z);
    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_LUX);
    input_set_capability(lpi->ls_input_dev, EV_REL, REL_CHN_LIGHT);

    ret = input_register_device(lpi->ls_input_dev);
    if (ret < 0) {
        pr_err("[LS][CM3391 error]%s: can not register ls input device\n",
                __func__);
        goto err_free_ls_input_device;
    }

    return ret;

err_free_ls_input_device:
    input_free_device(lpi->ls_input_dev);
    return ret;
}

static int cm3391_setup(struct cm3391_info *lpi)
{
    int ret = 0;
    uint16_t slaveAddr = lpi->RGB_slave_address;
    uint16_t cmd;

    lpi->it_setting = CM3391_IT_100MS;
    setupLuxGain(lpi->it_setting);

    lpi->HDCode     = CM3391_HD_DISABLE;
    setupHDGain(lpi->HDCode);

    ALS_CONF = CM3391_IT_100MS | CM3391_HD_DISABLE;

    cmd = ALS_CONF | CM3391_SD;

    als_power(1);
    msleep(5);

    ret = _cm3391_I2C_Write_Word(slaveAddr, CM3391_CMD_Addr, cmd );
    if(ret<0)
        return ret;

    msleep(10);

    cmd = ALS_CONF & CM3391_SD_MASK;
    ret = _cm3391_I2C_Write_Word(slaveAddr, CM3391_CMD_Addr, cmd );

    msleep(160);

    return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm3391_early_suspend(struct early_suspend *h)
{
    struct cm3391_info *lpi = lp_info;

    D("[LS][CM3391] %s\n", __func__);

    if (lpi->als_enable)
        lightsensor_disable(lpi);
}

static void cm3391_late_resume(struct early_suspend *h)
{
    struct cm3391_info *lpi = lp_info;

    D("[LS][CM3391] %s\n", __func__);

    if (!lpi->als_enable)
        lightsensor_enable(lpi);
}
#endif

static int cm3391_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret = 0;
    struct cm3391_info *lpi;
#ifndef CONFIG_OF
    struct cm3391_platform_data *pdata;
#endif

    D("[CM3391] %s\n", __func__);

    lpi = kzalloc(sizeof(struct cm3391_info), GFP_KERNEL);
    if (!lpi)
        return -ENOMEM;

    lpi->i2c_client = client;

    i2c_set_clientdata(client, lpi);

#ifndef CONFIG_OF
    pdata = client->dev.platform_data;
    if (!pdata) {
        pr_err("[CM3391 error]%s: Assign platform_data error!!\n",
                __func__);
        ret = -EBUSY;
        goto err_platform_data_null;
    }

    lpi->RGB_slave_address = pdata->RGB_slave_address;
    lpi->power = pdata->power;
#else
    lpi->RGB_slave_address = client->addr;
    lpi->power = NULL;
#endif

    lpi->polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);

    lp_info = lpi;

    mutex_init(&als_enable_mutex);
    mutex_init(&als_disable_mutex);
    mutex_init(&als_get_adc_mutex);

    ret = lightsensor_setup(lpi);
    if (ret < 0) {
        pr_err("[LS][CM3391 error]%s: lightsensor_setup error!!\n",
                __func__);
        goto err_lightsensor_setup;
    }

    lpi->lp_wq = create_singlethread_workqueue("cm3391_wq");
    if (!lpi->lp_wq) {
        pr_err("[CM3391 error]%s: can't create workqueue\n", __func__);
        ret = -ENOMEM;
        goto err_create_singlethread_workqueue;
    }

    ret = cm3391_setup(lpi);
    if (ret < 0) {
        pr_err("[ERR][CM3391 error]%s: cm3391_setup error!\n", __func__);
        goto err_cm3391_setup;
    }

    lpi->cm3391_class = class_create(THIS_MODULE, "optical_sensors");
    if (IS_ERR(lpi->cm3391_class)) {
        ret = PTR_ERR(lpi->cm3391_class);
        lpi->cm3391_class = NULL;
        goto err_create_class;
    }

    lpi->ls_dev = device_create(lpi->cm3391_class,
            NULL, 0, "%s", "lightsensor");
    if (unlikely(IS_ERR(lpi->ls_dev))) {
        ret = PTR_ERR(lpi->ls_dev);
        lpi->ls_dev = NULL;
        goto err_create_ls_device;
    }

    /* register the attributes */
    ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj,
            &light_attribute_group);
    if (ret) {
        pr_err("[LS][CM3391 error]%s: could not create sysfs group\n", __func__);
        goto err_sysfs_create_group_light;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    lpi->early_suspend.level =
        EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    lpi->early_suspend.suspend = cm3391_early_suspend;
    lpi->early_suspend.resume = cm3391_late_resume;
    register_early_suspend(&lpi->early_suspend);
#endif

    lpi->als_enable=0;
    lpi->als_enabled_before_suspend = 0;

    D("[CM3391] %s: Probe success!\n", __func__);

    return ret;

err_sysfs_create_group_light:
    device_unregister(lpi->ls_dev);
err_create_ls_device:
    class_destroy(lpi->cm3391_class);
err_create_class:
err_cm3391_setup:
    destroy_workqueue(lpi->lp_wq);
    mutex_destroy(&als_enable_mutex);
    mutex_destroy(&als_disable_mutex);
    mutex_destroy(&als_get_adc_mutex);
    input_unregister_device(lpi->ls_input_dev);
    input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
err_lightsensor_setup:
err_platform_data_null:
    kfree(lpi);
    return ret;
}

#ifdef CONFIG_PM_SLEEP
static int cm3391_suspend(struct device *dev)
{
    struct cm3391_info *lpi = lp_info;

    D("[LS][CM3391] %s\n", __func__);

    lpi->als_enabled_before_suspend = lpi->als_enable;
    if (lpi->als_enable)
        lightsensor_disable(lpi);

    return 0;
}

static int cm3391_resume(struct device *dev)
{
    struct cm3391_info *lpi = lp_info;

    D("[LS][CM3391] %s\n", __func__);

    if (lpi->als_enabled_before_suspend)
        lightsensor_enable(lpi);

    return 0;
}
#else
#define cm3391_suspend NULL
#define cm3391_resume NULL
#endif /* CONFIG_PM_SLEEP */

SIMPLE_DEV_PM_OPS(cm3391_pm, cm3391_suspend, cm3391_resume);

static const struct i2c_device_id cm3391_i2c_id[] = {
    {CM3391_I2C_NAME, 0},
    {}
};


#ifdef CONFIG_OF
static struct of_device_id cm3391_match_table[] = {
    { .compatible = "capella,cm3391",},
    { },
};
#else
#define cm3391_match_table NULL
#endif


static struct i2c_driver cm3391_driver = {
    .id_table = cm3391_i2c_id,
    .probe = cm3391_probe,
    .driver = {
        .name = CM3391_I2C_NAME,
        .owner = THIS_MODULE,
        .pm = &cm3391_pm,
        .of_match_table = of_match_ptr(cm3391_match_table),

    },
};

static int __init cm3391_init(void)
{
    return i2c_add_driver(&cm3391_driver);
}

static void __exit cm3391_exit(void)
{
    i2c_del_driver(&cm3391_driver);
}

module_init(cm3391_init);
module_exit(cm3391_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3391 Driver");
MODULE_AUTHOR("Frank Hsieh <frank.hsieh@vishay.com>");
