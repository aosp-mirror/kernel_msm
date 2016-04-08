/*
 * cyttsp5_i2c.c
 * Cypress TrueTouch(TM) Standard Product V5 I2C Module.
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

#include <linux/i2c.h>
#include <linux/version.h>

#define CY_I2C_DATA_SIZE  (2 * 256)

#ifdef CONFIG_HUAWEI_DSM
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <dsm/dsm_pub.h>
#define DSMINFO_LEN_MAX        64
static struct dsm_dev dsm_cyp_tp = {
	.name = "dsm_i2c_bus",	// dsm client name
	.fops = NULL,
	.buff_size = TP_RADAR_BUF_MAX,
};

struct tp_dsm_info g_tp_dsm_info;
struct dsm_client *tp_cyp_dclient = NULL;

ssize_t cyttsp5_dsm_record_basic_err_info(struct device *dev)
{
	ssize_t size = 0;
	ssize_t total_size = 0;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	/* power status,include mode, is enable, voltage */
	if((cd->cpdata->power_config->vbus_type == CY_POWER_PMU)&&(cd->cpdata->power_config->vbus_reg))
	{
		tp_log_debug("%s: record power!\n", __func__);
		size = dsm_client_record(tp_cyp_dclient,
					"[vbus power] mode:%d, enable:%d, vol:%d\n",
					regulator_get_mode(cd->cpdata->power_config->vbus_reg),
					regulator_is_enabled(cd->cpdata->power_config->vbus_reg),
					regulator_get_voltage(cd->cpdata->power_config->vbus_reg));

		total_size += size;

		tp_log_err("[vbus power] mode:%d, enable:%d, vol:%d\n",
					regulator_get_mode(cd->cpdata->power_config->vbus_reg),
					regulator_is_enabled(cd->cpdata->power_config->vbus_reg),
					regulator_get_voltage(cd->cpdata->power_config->vbus_reg));
	}

	if((cd->cpdata->power_config->vdd_type == CY_POWER_PMU)&&(cd->cpdata->power_config->vdd_reg))
	{
		tp_log_debug("%s: record power!\n", __func__);
		size = dsm_client_record(tp_cyp_dclient,
					"[vdd power]  mode:%d, enable:%d, vol:%d\n",
					regulator_get_mode(cd->cpdata->power_config->vdd_reg),
					regulator_is_enabled(cd->cpdata->power_config->vdd_reg),
					regulator_get_voltage(cd->cpdata->power_config->vdd_reg));

		total_size += size;

		tp_log_err("[vdd power]  mode:%d, enable:%d, vol:%d\n",
					regulator_get_mode(cd->cpdata->power_config->vdd_reg),
					regulator_is_enabled(cd->cpdata->power_config->vdd_reg),
					regulator_get_voltage(cd->cpdata->power_config->vdd_reg));
	}
	/* record irq and reset gpio status */
	tp_log_debug("%s: record irq and reset gpio!\n", __func__);
	size = dsm_client_record(tp_cyp_dclient,
				"[irq gpio]   num:%d, irq gpio status:%d\n"
				"[reset gpio] num:%d, reset gpio status:%d\n",
				g_tp_dsm_info.irq_gpio, gpio_get_value(g_tp_dsm_info.irq_gpio),
				g_tp_dsm_info.rst_gpio, gpio_get_value(g_tp_dsm_info.rst_gpio));
	total_size += size;

	tp_log_err("[irq gpio]   num:%d, irq gpio status:%d\n"
				"[reset gpio] num:%d, reset gpio status:%d\n",
				g_tp_dsm_info.irq_gpio, gpio_get_value(g_tp_dsm_info.irq_gpio),
				g_tp_dsm_info.rst_gpio, gpio_get_value(g_tp_dsm_info.rst_gpio));

	return total_size;
}

/* report error infomation */
ssize_t cyttsp5_dsm_record_common_err_info(char * err_info)
{

	ssize_t size = 0;
	ssize_t total_size = 0;

	tp_log_err("%s: entry!\n", __func__);

	/* err number */
	size = dsm_client_record(tp_cyp_dclient, "%s\n", err_info);
	total_size += size;

	return total_size;
}
/* i2c error infomation: err number, register infomation */
ssize_t cyttsp5_dsm_record_i2c_err_info( int err_numb )
{

	ssize_t size = 0;
	ssize_t total_size = 0;

	tp_log_err("%s: entry!\n", __func__);

	/* err number */
	size = dsm_client_record(tp_cyp_dclient, "i2c err number:%d\n", err_numb );
	total_size += size;

	return total_size;
}
/* fw err infomation: err number */
ssize_t cyttsp5_dsm_record_fw_err_info( int err_numb )
{

	ssize_t size = 0;
	ssize_t total_size = 0;

	tp_log_err("%s: entry!\n", __func__);

	/* err number */
	size = dsm_client_record(tp_cyp_dclient, "fw update result:failed, retval is %d\n", err_numb);
	total_size += size;

	/* fw err status */
	size = dsm_client_record(tp_cyp_dclient, "updata status is %d\n", g_tp_dsm_info.constraints_UPDATE_status);
	total_size += size;

	return total_size;
}

ssize_t cyttsp5_dsm_record_esd_err_info( int err_numb )
{

	ssize_t size = 0;
	ssize_t total_size = 0;

	tp_log_info("%s: entry!\n", __func__);

	/* err number */
	size = dsm_client_record(tp_cyp_dclient, "esd err number:%d\n", err_numb );
	total_size += size;

	return total_size;
}

/* tp report err according to err type */
int cyttsp5_tp_report_dsm_err(struct device *dev, int type, int err_numb)
{
	tp_log_err("%s: entry! type:%d\n", __func__, type);

	if( NULL == tp_cyp_dclient )
	{
		tp_log_err("%s: there is not tp_dclient!\n", __func__);
		return -1;
	}

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(tp_cyp_dclient))
	{
		/* buffer is busy */
		tp_log_err("%s: buffer is busy!\n", __func__);
		return -1;
	}

	/* tp report err according to err type */
	switch(type)
	{
		case DSM_TP_I2C_RW_ERROR_NO:
			/* report tp basic infomation */
			cyttsp5_dsm_record_basic_err_info(dev);
			/* report i2c infomation */
			cyttsp5_dsm_record_i2c_err_info(err_numb);
			break;
		case DSM_TP_FW_ERROR_NO:
			/* report tp basic infomation */
			cyttsp5_dsm_record_basic_err_info(dev);
			/* report fw infomation */
			cyttsp5_dsm_record_fw_err_info(err_numb);
			break;
		case DSM_TP_CYTTSP5_WAKEUP_ERROR_NO:
			/* report tp basic infomation */
			cyttsp5_dsm_record_basic_err_info(dev);
			/* report error infomation */
			cyttsp5_dsm_record_common_err_info("wakeup error");
			break;
		case DSM_TP_ESD_ERROR_NO:
			/* report tp basic infomation */
			cyttsp5_dsm_record_basic_err_info(dev);
			/* report esd infomation */
			cyttsp5_dsm_record_esd_err_info(err_numb);
			break;
		default:
			break;
	}
	dsm_client_notify(tp_cyp_dclient, type);

	return 0;
}
#endif/*CONFIG_HUAWEI_DSM*/
static int cyttsp5_i2c_read_default(struct device *dev, void *buf, int size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

	if (!buf || !size || size > CY_I2C_DATA_SIZE)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, size);
//drop device monitor

	return (rc < 0) ? rc : rc != size ? -EIO : 0;
}

/*****************************************************************
Parameters    :  dev
                 buf
                 max
Return        :  success return 0, fail return error number
Description   :  i2c read function
*****************************************************************/
static int cyttsp5_i2c_read_default_nosize(struct device *dev, u8 *buf, u32 max)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u32 size;

	if (!buf) {
		tp_log_err("%s %d:input parameter error.\n", __func__, __LINE__);
		return -EINVAL;
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msgs[0].len = 2;
	msgs[0].buf = buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count) {
		tp_log_err("%s %d:I2C transfer error, rc = %d, msg_count = %d\n",
					__func__, __LINE__, rc, msg_count);
		return (rc < 0) ? rc : -EIO;
	}

	size = get_unaligned_le16(&buf[0]);
	if (!size || size == 2) {
		tp_log_info("%s %d:get_unaligned_le16, size = %d\n", __func__,
					__LINE__, size);
		return 0;
	}

	if (size > max) {
		tp_log_err("%s %d:ERROR, size = %d, max = %d\n", __func__,
					__LINE__, size, max);
		return -EINVAL;
	}

	rc = i2c_master_recv(client, buf, size);
//drop device monitor

	if (rc < 0) {
		tp_log_err("%s %d:I2C read error, rc = %d\n", __func__,
					__LINE__, rc);
		return rc;
	} else if (rc != size) {
		tp_log_err("%s %d:I2C read error, rc = %d, size = %d\n",
					__func__, __LINE__, rc, size);
		return -EIO;
	}

	return 0;
}

/*****************************************************************
Parameters    :  dev
                 write_len
                 write_buf
                 read_buf
Return        :
Description   :
*****************************************************************/
static int cyttsp5_i2c_write_read_specific(struct device *dev, u8 write_len,
		u8 *write_buf, u8 *read_buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;

	if (!write_buf || !write_len) {
		tp_log_err("%s %d:input parameter error.\n", __func__, __LINE__);
		return -EINVAL;
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
	msgs[0].buf = write_buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc < 0 || rc != msg_count) {
//drop device monitor
		tp_log_err("%s %d:I2C transfer error, rc = %d\n", __func__, __LINE__, rc);
		return (rc < 0) ? rc : -EIO;
	}

	rc = 0;

	if (read_buf) {
		rc = cyttsp5_i2c_read_default_nosize(dev, read_buf,
				CY_I2C_DATA_SIZE);
		if (rc) {
			tp_log_err("%s %d:I2C read error, rc = %d\n", __func__, __LINE__, rc);
		}
	}

	return rc;
}

static struct cyttsp5_bus_ops cyttsp5_i2c_bus_ops = {
	.bustype = BUS_I2C,
	.read_default = cyttsp5_i2c_read_default,
	.read_default_nosize = cyttsp5_i2c_read_default_nosize,
	.write_read_specific = cyttsp5_i2c_write_read_specific,
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
static struct of_device_id cyttsp5_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp5_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp5_i2c_of_match);
#endif


/*****************************************************************
Parameters    :  client
                 i2c_id
Return        :
Description   :  call cyttsp5_probe in cyttsp5_core.c
*****************************************************************/
static int cyttsp5_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	const struct of_device_id *match = NULL;
#endif
	int rc = 0;

	tp_log_warning("%s %d:Probe start\n", __func__, __LINE__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		tp_log_err("%s %d:I2C functionality not Supported.\n", __func__, __LINE__);
		return -EIO;
	}

/* if support device tree, get pdata from device tree */
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
	if (match) {
		rc = cyttsp5_devtree_create_and_get_pdata(dev);
		if (rc < 0) {
			tp_log_err("%s %d:device tree create and get pdata fail, rc = %d.\n",
						__func__, __LINE__, rc);
			return rc;
		}
	} else {
		tp_log_err("%s %d:No device mathced.\n", __func__, __LINE__);
		return -ENODEV;
	}
#endif

#ifdef CONFIG_HUAWEI_DSM
	tp_cyp_dclient = dsm_register_client(&dsm_cyp_tp);
	if (!tp_cyp_dclient)
		tp_log_err("%s: dsm register client failed\n", __func__);
#endif/*CONFIG_HUAWEI_DSM*/

	rc = cyttsp5_probe(&cyttsp5_i2c_bus_ops, &client->dev, client->irq,
			  CY_I2C_DATA_SIZE);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	if (rc && match) {
		cyttsp5_devtree_clean_pdata(dev);
#ifdef CONFIG_HUAWEI_DSM
		if (tp_cyp_dclient) {
			dsm_unregister_client(tp_cyp_dclient, &dsm_cyp_tp);
			tp_cyp_dclient = NULL;
		}
#endif/*CONFIG_HUAWEI_DSM*/
		tp_log_err("%s %d:cyttsp5 probe fail.\n", __func__, __LINE__);
		return rc;
	}
#endif

	tp_log_info("%s %d:cyttsp5 probe success.\n", __func__, __LINE__);

	return rc;
}

static int cyttsp5_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	struct device *dev = &client->dev;
	const struct of_device_id *match;
#endif
	struct cyttsp5_core_data *cd = i2c_get_clientdata(client);

	cyttsp5_release(cd);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
	if (match)
		cyttsp5_devtree_clean_pdata(dev);
#endif

	return 0;
}

static const struct i2c_device_id cyttsp5_i2c_id[] = {
	{ CYTTSP5_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp5_i2c_id);

static struct i2c_driver cyttsp5_i2c_driver = {
	.driver = {
		.name = CYTTSP5_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &cyttsp5_pm_ops,
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
		.of_match_table = cyttsp5_i2c_of_match,
#endif
	},
	.probe = cyttsp5_i2c_probe,
	.remove = cyttsp5_i2c_remove,
	.id_table = cyttsp5_i2c_id,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
module_i2c_driver(cyttsp5_i2c_driver);
#else
static int __init cyttsp5_i2c_init(void)
{
	int rc = i2c_add_driver(&cyttsp5_i2c_driver);

	if (rc) {
		tp_log_err("%s %d: Cypress v5 I2C Driver add fail, rc = %d.\n",
		 			__func__, __LINE__, rc);
	} else {
		tp_log_info("%s %d: Cypress TTSP v5 I2C Driver add success.\n",
		 			__func__, __LINE__, rc);
	}
	return rc;
}
module_init(cyttsp5_i2c_init);

static void __exit cyttsp5_i2c_exit(void)
{
	i2c_del_driver(&cyttsp5_i2c_driver);
}
module_exit(cyttsp5_i2c_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
