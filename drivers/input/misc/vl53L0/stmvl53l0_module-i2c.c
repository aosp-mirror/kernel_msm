/*
 *  stmvl53l0_module-i2c.c - Linux kernel modules for STM VL53L0 FlightSense TOF
 *							sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
/***************************
 * power specific includes
 **************************/
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
/****************
 * API includes
 ****************/
#include "vl53l0_api.h"
#include "vl53l0_def.h"
#include "vl53l0_platform.h"
#include "stmvl53l0-i2c.h"
#include "stmvl53l0-cci.h"
#include "stmvl53l0.h"
#ifndef CAMERA_CCI

#define HTC_MODIFY
#define HTC
// HTC ADD ++//
#ifdef HTC
#define CALIBRATION_DATA_PATH "/calibration_data"
#define SENSOR_FLASH_DATA "gyro_flash"
#endif
// HTC ADD --//
/***************
 * Global data
 **************/
static int stmvl53l0_parse_vdd(struct device *dev, struct i2c_data *data);
//* HTC ADD *//
struct laser_device_data *g_laser_data;

/***************************
 * QCOM specific functions
 ***************************/
static int stmvl53l0_parse_vdd(struct device *dev, struct i2c_data *data)
{
    int ret = 0;

    vl53l0_dbgmsg("Enter++\n");

    if (dev->of_node) {
        data->vana = regulator_get(dev, "vdd");
        if (IS_ERR(data->vana)) {
            vl53l0_errmsg("vdd supply is not provided\n");
            ret = -1;
        }
    }
    vl53l0_dbgmsg("End--\n");

    return ret;
}

#ifdef HTC
// HTC ADD ++//
static int Laser_parse_dt(struct device *dev, struct stmvl53l0_data *data)
{
    struct device_node *dt = dev->of_node;
    struct device_node *sensor_offset;
    char *sensor_cali_data = NULL;
    int sensor_cali_size = 0;
    int i = 0;

    if (of_property_read_string(dt, "laser,calib-file", &data->calib_file))
        data->calib_file = NULL;
    else
        vl53l0_errmsg("calib_file = %s\n", data->calib_file);

    if (!gpio_is_valid(data->pwdn_gpio))
        vl53l0_errmsg("pwdn_gpio value is not valid\n");
    else
        vl53l0_dbgmsg("pwdn_gpio = %d\n", data->pwdn_gpio);

    data->pwdn_gpio = of_get_named_gpio(dt, "laser,pwdn-gpio", 0);
    if (!gpio_is_valid(data->pwdn_gpio))
        vl53l0_errmsg("pwdn_gpio value is not valid\n");
    else
        vl53l0_dbgmsg("pwdn_gpio = %d\n", data->pwdn_gpio);

    data->power_2v8 = devm_regulator_get(dev, "power_2v8");
    if (IS_ERR(data->power_2v8)) {
        data->power_2v8 = NULL;
        vl53l0_errmsg("Unable to get power_2v8\n");
    }
    data->laser_irq_gpio = of_get_named_gpio(dt, "laser,intr-gpio", 0);
    if (!gpio_is_valid(data->laser_irq_gpio))
        vl53l0_errmsg("laser_irq_gpio value is not valid\n");
    else
        vl53l0_dbgmsg("laser_irq_gpio = %d\n", data->laser_irq_gpio);

    data->camio_1v8 = devm_regulator_get(dev, "CAMIO_1v8");
    if (IS_ERR(data->camio_1v8)) {
        data->camio_1v8 = NULL;
        E("%s: Unable to get CAMIO_1v8\n", __func__);
    }

    sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
    if (sensor_offset) {
        sensor_cali_data = (char *)
            of_get_property(sensor_offset,
                    SENSOR_FLASH_DATA,
                    &sensor_cali_size);
        vl53l0_dbgmsg("%s: sensor cali_size = %d\n", __func__, sensor_cali_size);

        if (sensor_cali_data) {
            for (i = 100; (i < sensor_cali_size) && (i < 108); i++) {
                vl53l0_dbgmsg("gyro sensor cali_data[%d] = %02x\n", i, sensor_cali_data[i]);
            }

            data->offset_kvalue = (s8) sensor_cali_data[103];
            vl53l0_dbgmsg("%s: Update offset_kvalue = %d\n",__func__, data->offset_kvalue);
            if (data->offset_kvalue != 0)
                data->cali_status = 0x10;

            data->xtalk_kvalue = (sensor_cali_data[107]<<8) | sensor_cali_data[106];
            vl53l0_dbgmsg("%s: Update xtalk_kvalue = 0x%X\n", __func__, data->xtalk_kvalue);
            if (data->xtalk_kvalue != 0)
                data->cali_status |= 0x1;

            /* Reference SPADs Calibration Data
             * isApertureSpads: type of reference SPADS
             * refSpadCount: number of reference SPADS */

            data->isApertureSpads = (u8) sensor_cali_data[101];
            data->refSpadCount = (u32) sensor_cali_data[102];
            I("%s: Update SpadCount = %d and IsAperture = %d\n", __func__,
                    data->refSpadCount, data->isApertureSpads);
            if (data->isApertureSpads || data->refSpadCount)
                data->cali_status |= 0x20;
        }
    } else {
        vl53l0_errmsg("%s: Sensor Calibration data offset not found\n", __func__);
    }
    return 0;
}

static int Laser_pinctrl_init(struct stmvl53l0_data *laser_data)
{
    int retval = 0;
    int ret = 0;

    vl53l0_dbgmsg("Enter++\n");

    laser_data->pinctrl = devm_pinctrl_get(laser_data->sensor_dev);
    if (IS_ERR_OR_NULL(laser_data->pinctrl)) {
        vl53l0_errmsg("Target does not use pinctrl\n");
        retval = PTR_ERR(laser_data->pinctrl);
        laser_data->pinctrl = NULL;
        return retval;
    }

    laser_data->gpio_state_init = pinctrl_lookup_state(laser_data->pinctrl, "laser_gpio_init");
    if (IS_ERR_OR_NULL(laser_data->gpio_state_init)) {
        vl53l0_errmsg("Can not get ts default pinstate\n");
        retval = PTR_ERR(laser_data->gpio_state_init);
        laser_data->pinctrl = NULL;
        return retval;
    }

    ret = pinctrl_select_state(laser_data->pinctrl, laser_data->gpio_state_init);
    if (ret) {
        vl53l0_errmsg("can not init gpio\n");
        return ret;
    }

    ret = regulator_enable(laser_data->camio_1v8);
    if (ret) {
        vl53l0_errmsg("Failed to enable camio_1v8\n");
        return ret;
    }

    ret = regulator_enable(laser_data->power_2v8);
    if (ret) {
        vl53l0_errmsg("Failed to enable power_2v8\n");
        return ret;
    }

    ret = gpio_direction_output(laser_data->pwdn_gpio, 1);
    if (ret) {
        vl53l0_errmsg("Failed to pull up pwdn_gpio\n");
        return ret;
    }

    vl53l0_dbgmsg("End--\n");
    return 0;
}
#endif

// HTC ADD //
static int stmvl53l0_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int rc = 0;
    struct stmvl53l0_data *vl53l0_data = NULL;
    struct i2c_data *i2c_object = NULL;

    vl53l0_dbgmsg("Enter++\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
        rc = -EIO;
        return rc;
    }

    vl53l0_data = kzalloc(sizeof(struct stmvl53l0_data), GFP_KERNEL);
    if (!vl53l0_data) {
        rc = -ENOMEM;
        return rc;
    }
    if (vl53l0_data) {
        vl53l0_data->client_object = kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
        i2c_object = (struct i2c_data *)vl53l0_data->client_object;
    }
    i2c_object->client = client;
    vl53l0_data->sensor_dev = &client->dev;

#ifdef HTC
    if (client->dev.of_node) {
        Laser_parse_dt(&client->dev, vl53l0_data);
    }
    if (vl53l0_data->calib_file)
        stmvl53l0_read_calibration(vl53l0_data);
#endif
    /* setup bus type */
    vl53l0_data->bus_type = I2C_BUS;

    /* setup regulator */
    stmvl53l0_parse_vdd(&i2c_object->client->dev, i2c_object);

    /* setup device name */
    vl53l0_data->dev_name = dev_name(&client->dev);

    /* setup device data */
    dev_set_drvdata(&client->dev, vl53l0_data);

    /* setup client data */
    i2c_set_clientdata(client, vl53l0_data);

    /* gpio/power init */
    Laser_pinctrl_init(vl53l0_data);

    /* setup other stuff */
    rc = stmvl53l0_setup(vl53l0_data);

    /* init default value */
    i2c_object->power_up = 0;

    gpio_set_value(vl53l0_data->pwdn_gpio, 0);

    rc = regulator_disable(vl53l0_data->power_2v8);
    if (rc)
        vl53l0_errmsg("Failed to disable power_2v8\n");

    rc = regulator_disable(vl53l0_data->camio_1v8);
    if (rc)
        vl53l0_errmsg("Failed to disable camio_1v8\n");

    vl53l0_dbgmsg("Success--\n");
    return rc;
}

static int stmvl53l0_remove(struct i2c_client *client)
{
    struct stmvl53l0_data *data = i2c_get_clientdata(client);

    vl53l0_dbgmsg("Enter\n");

    /* Power down the device */
    stmvl53l0_power_down_i2c(data->client_object);
	stmvl53l0_cleanup(data);
    kfree(data->client_object);
    kfree(data);
    vl53l0_dbgmsg("End\n");
    return 0;
}

static const struct i2c_device_id stmvl53l0_id[] = {
    { STMVL53L0_DRV_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, stmvl53l0_id);

static const struct of_device_id st_stmvl53l0_dt_match[] = {
    //	{ .compatible = "st,stmvl53l0", },
    { .compatible = "htc_laser"},
    { .compatible = "wahoo_laser"},
    { },
};

static struct i2c_driver stmvl53l0_driver = {
    .driver = {
        .name	= STMVL53L0_DRV_NAME,
        .owner	= THIS_MODULE,
        .of_match_table = st_stmvl53l0_dt_match,
        .probe_type = PROBE_PREFER_ASYNCHRONOUS,
    },
    .probe	= stmvl53l0_probe,
    .remove	= stmvl53l0_remove,
    .id_table = stmvl53l0_id,

};

int stmvl53l0_power_up_i2c(void *i2c_object, unsigned int *preset_flag)
{
    int ret = 0;
    struct i2c_data *data = (struct i2c_data *)i2c_object;

    vl53l0_dbgmsg("Enter\n");
#ifdef HTC_MODIFY
    data->power_up = 1;
    *preset_flag = 1;
#endif
    vl53l0_dbgmsg("End\n");
    return ret;
}

int stmvl53l0_power_down_i2c(void *i2c_object)
{
    int ret = 0;
    struct i2c_data *data = (struct i2c_data *)i2c_object;

    vl53l0_dbgmsg("Enter\n");
#ifdef HTC_MODIFY
    data->power_up = 0;
#endif
    vl53l0_dbgmsg("End\n");
    return ret;
}

int stmvl53l0_init_i2c(void)
{
    int ret = 0;

#ifdef STM_TEST
    struct i2c_client *client = NULL;
    struct i2c_adapter *adapter;
    struct i2c_board_info info = {
        .type = "stmvl53l0",
        .addr = STMVL53L0_SLAVE_ADDR,
    };
#endif

    vl53l0_dbgmsg("Enter\n");

    /* register as a i2c client device */
    ret = i2c_add_driver(&stmvl53l0_driver);
    if (ret)
        vl53l0_errmsg("%d erro ret:%d\n", __LINE__, ret);

#ifdef STM_TEST
    if (!ret) {
        adapter = i2c_get_adapter(4);
        if (!adapter)
            ret = -EINVAL;
        else
            client = i2c_new_device(adapter, &info);
        if (!client)
            ret = -EINVAL;
    }
#endif

    vl53l0_dbgmsg("End with rc:%d\n", ret);

    return ret;
}

void stmvl53l0_exit_i2c(void *i2c_object)
{
    vl53l0_dbgmsg("Enter\n");
    i2c_del_driver(&stmvl53l0_driver);

    vl53l0_dbgmsg("End\n");
}

#endif /* end of NOT CAMERA_CCI */
