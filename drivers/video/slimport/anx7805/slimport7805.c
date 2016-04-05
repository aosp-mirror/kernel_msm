/*
* Copyright(c) 2012-2013, Analogix Semiconductor All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/slimport.h>
#include <linux/async.h>
#include <linux/of_platform.h>

#include "slimport7805_tx_drv.h"
#include <linux/slimport.h>

struct anx7805_data {
	struct i2c_client *client;
	struct anx7805_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock slimport_lock;
	int gpio_p_dwn;
	int gpio_reset;
	int gpio_int;
	int gpio_i2c;
	struct regulator *avdd33_reg;
	struct regulator *avdd18_reg;
	struct regulator *vdd_reg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	bool update_chg_type;
};

struct anx7805_data *the_chip;
struct completion init_aux_ch_completion;

static int anx7805_regulator_configure(struct anx7805_data *chip, int on)
{
	int ret = 0;

	if (on == 0)
		goto  err_set_vdd;

	if (!chip->avdd33_reg) {
		chip->avdd33_reg = regulator_get(&chip->client->dev, "analogix,vdd_33");
		if (IS_ERR(chip->avdd33_reg)) {
			ret = PTR_ERR(chip->avdd33_reg);
			pr_err("regulator_get avdd33 failed. ret = %d\n",ret);
			chip->avdd33_reg = NULL;
			goto out;
		}
	}

	if (regulator_count_voltages(chip->avdd33_reg) > 0) {
		ret = regulator_set_voltage(chip->avdd33_reg, 3300000, 3300000);
		if (ret) {
			pr_err("regulator set avdd33 failed ret =%d\n",ret);
			goto err_set_avdd33;
		}
	}

	if (!chip->avdd18_reg) {
		chip->avdd18_reg = regulator_get(&chip->client->dev, "analogix,vdd_18");
		if (IS_ERR(chip->avdd18_reg)) {
			ret = PTR_ERR(chip->avdd18_reg);
			pr_err("regulator_get avdd18 failed. rc = %d\n",ret);
			goto err_get_avdd18;
		}
	}

	if (regulator_count_voltages(chip->avdd18_reg) > 0) {
		ret = regulator_set_voltage(chip->avdd18_reg, 1800000, 1800000);
		if (ret) {
			pr_err("regulator set avdd18 failed ret =%d\n",ret);
			goto err_set_avdd18;
		}
	}

	if (!chip->vdd_reg) {
		chip->vdd_reg = regulator_get(&chip->client->dev, "analogix,vdd_10");
		if (IS_ERR(chip->vdd_reg)) {
			ret = PTR_ERR(chip->vdd_reg);
			pr_err("regulator_get vdd_10 failed. ret = %d\n",ret);
			goto err_get_vdd;
		}
	}

	if (regulator_count_voltages(chip->vdd_reg) > 0) {
		ret = regulator_set_voltage(chip->vdd_reg, 2950000, 2950000);
		pr_err("%s %s set 1.0V\n",LOG_TAG,__func__);
		if (ret) {
			pr_err("regulator set vdd failed ret =%d\n",ret);
			goto err_set_vdd;
		}
	}

	goto out;

err_set_vdd:
	regulator_put(chip->vdd_reg);
err_get_vdd:
	chip->vdd_reg = NULL;
err_set_avdd18:
	regulator_put(chip->avdd18_reg);
err_get_avdd18:
	chip->avdd18_reg = NULL;
err_set_avdd33:
	regulator_put(chip->avdd33_reg);
out:
	return ret;
}

static int anx7805_avdd_3p3_power(struct anx7805_data *chip, int on)
{
	int ret = 0;

	if (on) {

		ret = regulator_set_optimum_mode(chip->avdd33_reg,100000);
		if (ret < 0) {
			pr_err("Regulator vdd_reg set_opt failed rc=%d\n", ret);
			return ret;
		}
		ret = regulator_enable(chip->avdd33_reg);
		if (ret) {
			pr_err("avdd33_reg enable failed (%d)\n", ret);
			goto err_avdd33;
		}
	} else {
		regulator_set_optimum_mode(chip->avdd33_reg, 0);
		ret = regulator_disable(chip->avdd33_reg);
		if (ret) {
			pr_err("avdd33_reg disable failed (%d)\n", ret);
			goto err_avdd33;
		}
	}

	return ret;

err_avdd33:
	regulator_put(chip->avdd33_reg);
	chip->avdd33_reg = NULL;

	return ret;
}

static int anx7805_avdd_1p8_power(struct anx7805_data *chip, int on)
{
	int ret = 0;

	if (on) {
		ret = regulator_set_optimum_mode(chip->avdd18_reg,100000);
		if (ret < 0) {
			pr_err("Regulator vdd_reg set_opt failed rc=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(chip->avdd18_reg);
		if (ret) {
			pr_err("avdd18_reg enable failed (%d)\n", ret);
			goto err_avdd18;
		}
	} else {
		regulator_set_optimum_mode(chip->avdd18_reg, 0);
		ret = regulator_disable(chip->avdd18_reg);
		if (ret) {
			pr_err("avdd18_reg disable failed (%d)\n", ret);
			goto err_avdd18;
		}
	}

	return ret;

err_avdd18:
	regulator_put(chip->avdd18_reg);
	chip->avdd18_reg = NULL;

	return ret;
}

static int anx7805_vdd_1p0_power(struct anx7805_data *chip, int on)
{
	int ret = 0;

	if (on) {
		ret = regulator_set_optimum_mode(chip->vdd_reg,100000);
		if (ret < 0) {
			pr_err("Regulator vdd_reg set_opt failed rc=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(chip->vdd_reg);
		if (ret) {
			pr_err("vdd_reg enable failed (%d)\n", ret);
			goto err_vdd;
		}
	} else {
		regulator_set_optimum_mode(chip->vdd_reg, 0);
		ret = regulator_disable(chip->vdd_reg);
		if (ret) {
			pr_err("vdd_reg disable failed (%d)\n", ret);
			goto err_vdd;
		}
	}

	return ret;

err_vdd:
	regulator_put(chip->vdd_reg);
	chip->vdd_reg = NULL;

	return ret;
}

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;

	if (!the_chip)
		return -EINVAL;

	the_chip->client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(the_chip->client, offset);
	if (ret < 0) {
		pr_err("failed to read i2c addr=%x, ret=%d\n", slave_addr,ret);
		return ret;
	}
	*buf = (uint8_t) ret;

	return 0;
}

int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value)
{
	int ret = 0;

	if (!the_chip)
		return -EINVAL;

	the_chip->client->addr = (slave_addr >> 1);
	ret = i2c_smbus_write_byte_data(the_chip->client, offset, value);
	if (ret < 0) {
		pr_err("failed to write i2c addr=%x, ret=%d\n", slave_addr,ret);
	}
	return ret;
}

void sp_tx_hardware_poweron(void)
{
	if (!the_chip)
		return;

	gpio_set_value(the_chip->gpio_reset, 0);
	msleep(1);
	gpio_set_value(the_chip->gpio_p_dwn, 0);
	msleep(2);
	anx7805_vdd_1p0_power(the_chip, 1);
	msleep(5);
	gpio_set_value(the_chip->gpio_reset, 1);

	pr_info("anx7805 power on\n");
}

void sp_tx_hardware_powerdown(void)
{
	if (!the_chip)
		return;

	gpio_set_value(the_chip->gpio_reset, 0);
	msleep(1);
	anx7805_vdd_1p0_power(the_chip, 0);
	msleep(2);
	gpio_set_value(the_chip->gpio_p_dwn, 1);
	msleep(1);

	pr_info("anx7805 power down\n");
}

int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	if (block == 0) {
		memcpy(edid_buf, bEDID_firstblock, sizeof(bEDID_firstblock));
	} else if (block == 1) {
		memcpy(edid_buf, bEDID_extblock, sizeof(bEDID_extblock));
	} else {
		pr_err("%s: block number %d is invalid\n", __func__, block);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(slimport_read_edid_block);

bool slimport_is_connected(void)
{
	bool result = true;

	if (!the_chip)
		return false;

	return result;
}
EXPORT_SYMBOL(slimport_is_connected);

static void anx7805_free_gpio(struct anx7805_data *anx7805)
{
	gpio_free(anx7805->gpio_int);
	gpio_free(anx7805->gpio_reset);
	gpio_free(anx7805->gpio_p_dwn);
	gpio_free(anx7805->gpio_i2c);
}

static int anx7805_init_gpio(struct anx7805_data *anx7805)
{
	int ret = 0;

	ret = gpio_request(anx7805->gpio_p_dwn, "anx7805_p_dwn_ctl");
	if (ret) {
		pr_err("%s : failed to request p_dwn gpio %d\n", __func__,
				anx7805->gpio_p_dwn);
		goto out;
	}

	gpio_direction_output(anx7805->gpio_p_dwn, 1);

	ret = gpio_request(anx7805->gpio_reset, "anx7805_reset_n");
	if (ret) {
		pr_err("%s : failed to request reset gpio %d\n", __func__,
				anx7805->gpio_reset);
		goto err0;
	}
	gpio_direction_output(anx7805->gpio_reset, 0);

	ret = gpio_request(anx7805->gpio_int, "anx7805_int_n");
	if (ret) {
		pr_err("%s : failed to request int gpio %d\n", __func__,
				anx7805->gpio_int);
		goto err1;
	}
	gpio_direction_input(anx7805->gpio_int);

	ret = gpio_request(anx7805->gpio_i2c, "anx7805_i2c_pull_up");
	if (ret) {
		pr_err("%s : failed to request i2c gpio %d\n", __func__,
				anx7805->gpio_i2c);
		goto err2;
	}
	gpio_direction_output(anx7805->gpio_i2c, 1);

	gpio_set_value(anx7805->gpio_i2c, 1);
	gpio_set_value(anx7805->gpio_reset, 0);
	gpio_set_value(anx7805->gpio_p_dwn, 1);

	goto out;

err2:
	gpio_free(anx7805->gpio_int);
err1:
	gpio_free(anx7805->gpio_reset);
err0:
	gpio_free(anx7805->gpio_p_dwn);
out:
	return ret;
}

static int anx7805_pinctrl_init(struct anx7805_data *anx7805)
{
	anx7805->pinctrl = devm_pinctrl_get(&anx7805->client->dev);
	if (IS_ERR_OR_NULL(anx7805->pinctrl)) {
		pr_err("%s: failed to get pinctrl\n", __func__);
		return PTR_ERR(anx7805->pinctrl);
	}

	anx7805->gpio_state_active
		= pinctrl_lookup_state(anx7805->pinctrl, "sp_active");
	if (IS_ERR_OR_NULL(anx7805->gpio_state_active))
		pr_warn("%s: can not get default pinstate\n", __func__);

	anx7805->gpio_state_suspend
		= pinctrl_lookup_state(anx7805->pinctrl, "sp_sleep");
	if (IS_ERR_OR_NULL(anx7805->gpio_state_suspend))
		pr_warn("%s: can not get sleep pinstate\n", __func__);

	return 0;
}

static int anx7805_pinctrl_select(struct anx7805_data *anx7805, bool active)
{
	struct pinctrl_state *pin_state;
	int ret = 0;

	pin_state = active ? anx7805->gpio_state_active
		: anx7805->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pin_state)) {
		ret = pinctrl_select_state(anx7805->pinctrl, pin_state);
		if (ret)
			pr_err("%s: can not set %s pins\n", __func__,
			       active ? "anx7805_default"
			       : "anx7805_slee");
	} else {
		pr_err("%s: invalid '%s' pinstate\n", __func__,
		       active ? "anx7805_default"
		       : "anx7805_sleep");

	}
	return ret;
}

static int anx7805_system_init(void)
{
	int ret = 0;

	ret = SP_CTRL_Chip_Detect();
	if (ret == 0) {
		pr_err("failed to detect anx7805\n");
		return -ENODEV;
	}

	SP_CTRL_Chip_Initial();
	return 0;
}

static void anx7805_work_func(struct work_struct *work)
{
	struct anx7805_data *td = container_of(work, struct anx7805_data,
	                                       work.work);

	SP_CTRL_Main_Procss();
	queue_delayed_work(td->workqueue, &td->work,
	                   msecs_to_jiffies(300));
}

static int anx7805_parse_dt(struct device_node *node,
                            struct anx7805_data *anx7805)
{
	int ret = 0;

	anx7805->gpio_p_dwn =
	    of_get_named_gpio(node, "analogix,p-dwn-gpio", 0);
	if (anx7805->gpio_p_dwn < 0) {
		pr_err("failed to get analogix,p-dwn-gpio.\n");
		ret = anx7805->gpio_p_dwn;
		goto out;
	}

	anx7805->gpio_reset =
	    of_get_named_gpio(node, "analogix,reset-gpio", 0);
	if (anx7805->gpio_reset < 0) {
		pr_err("failed to get analogix,reset-gpio.\n");
		ret = anx7805->gpio_reset;
		goto out;
	}

	anx7805->gpio_int =
	    of_get_named_gpio(node, "analogix,irq-gpio", 0);
	if (anx7805->gpio_int < 0) {
		pr_err("failed to get analogix,irq-gpio.\n");
		ret = anx7805->gpio_int;
		goto out;
	}

	anx7805->gpio_i2c =
	    of_get_named_gpio(node, "analogix,i2c-pull-up-gpio", 0);
	if (anx7805->gpio_i2c < 0) {
		pr_err("failed to get analogix,i2c-pull-up-gpio.\n");
		ret = anx7805->gpio_i2c;
		goto out;
	}

out:
	return ret;
}

static int anx7805_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
	struct anx7805_data *anx7805;
	struct anx7805_platform_data *pdata;
	struct device_node *dev_node = client->dev.of_node;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter,
	                             I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("i2c bus does not support anx7805\n");
		ret = -ENODEV;
		goto exit;
	}

	anx7805 = kzalloc(sizeof(struct anx7805_data), GFP_KERNEL);
	if (!anx7805) {
		pr_err("failed to allocate driver data\n");
		ret = -ENOMEM;
		goto exit;
	}

	anx7805->client = client;
	i2c_set_clientdata(client, anx7805);

	if (dev_node) {
		ret = anx7805_parse_dt(dev_node, anx7805);
		if (ret) {
			pr_err("failed to parse dt\n");
			goto err0;
		}
	} else {
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			pr_err("no platform data.\n");
			goto err0;
		}

		anx7805->gpio_p_dwn = pdata->gpio_p_dwn;
		anx7805->gpio_reset = pdata->gpio_reset;
		anx7805->gpio_int = pdata->gpio_int;
		anx7805->gpio_i2c = pdata->gpio_i2c;
	}

	the_chip = anx7805;

	mutex_init(&anx7805->lock);
	init_completion(&init_aux_ch_completion);

	ret = anx7805_regulator_configure(anx7805, 1);
	if (ret) {
		pr_err("failed to regulator config\n");
		goto err0;
	}

	ret = anx7805_pinctrl_init(anx7805);
	if (!ret) {
		ret = anx7805_pinctrl_select(anx7805, true);
		if (ret) {
		pr_err("failed to pinctrl\n");
			goto err1;
		}
	}

	ret = anx7805_init_gpio(anx7805);
	if (ret) {
		pr_err("failed to initialize gpio\n");
		goto err2;
	}

	INIT_DELAYED_WORK(&anx7805->work, anx7805_work_func);

	anx7805->workqueue = create_singlethread_workqueue("anx7805_work");
	if (!anx7805->workqueue) {
		pr_err("failed to create work queue\n");
		ret = -ENOMEM;
		goto err3;
	}

	ret = anx7805_avdd_1p8_power(anx7805, 1);
	if (ret)
		goto err5;

	ret = anx7805_avdd_3p3_power(anx7805, 1);
	if (ret)
		goto err4;

	msleep(2);

	ret = anx7805_vdd_1p0_power(anx7805, 1);

	if (ret)
		goto err6;

	msleep(2);

	ret = anx7805_vdd_1p0_power(anx7805, 0);

	msleep(2);
	if (ret)
		goto err6;

	ret = anx7805_system_init();
	if (ret) {
		pr_err("failed to initialize anx7805\n");
		goto err7;
	}

	wake_lock_init(&anx7805->slimport_lock, WAKE_LOCK_SUSPEND,
	               "slimport_wake_lock");

	goto exit;

err7:
	if (!anx7805->vdd_reg)
		regulator_put(anx7805->vdd_reg);
err6:
	if (!anx7805->avdd18_reg)
		regulator_put(anx7805->avdd18_reg);
err5:
	if (!anx7805->avdd33_reg)
		regulator_put(anx7805->avdd33_reg);
err4:
	destroy_workqueue(anx7805->workqueue);
err3:
	anx7805_free_gpio(anx7805);
err2:
	anx7805_pinctrl_select(anx7805, false);
err1:
	anx7805_regulator_configure(anx7805, 0);
err0:
	the_chip = NULL;
	kfree(anx7805);

	return ret;
exit:
	pr_info("%s probe done\n",LOG_TAG);
	queue_delayed_work(anx7805->workqueue, &anx7805->work, 0);

	return ret;
}

static int anx7805_i2c_remove(struct i2c_client *client)
{
	struct anx7805_data *anx7805 = i2c_get_clientdata(client);

	wake_lock_destroy(&anx7805->slimport_lock);
	anx7805_regulator_configure(anx7805, 0);
	if (!anx7805->vdd_reg)
		regulator_put(anx7805->vdd_reg);
	if (!anx7805->avdd18_reg)
		regulator_put(anx7805->avdd18_reg);
	if (!anx7805->avdd33_reg)
		regulator_put(anx7805->avdd33_reg);
	destroy_workqueue(anx7805->workqueue);
	anx7805_free_gpio(anx7805);
	the_chip = NULL;
	kfree(anx7805);
	return 0;
}

static const struct i2c_device_id anx7805_id[] = {
	{ "anx7805", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, anx7805_id);

static struct of_device_id anx_match_table[] = {
	{ .compatible = "analogix,anx7805",},
	{ },
};

static struct i2c_driver anx7805_driver = {
	.driver = {
		.name = "anx7805",
		.owner = THIS_MODULE,
		.of_match_table = anx_match_table,
	},
	.probe = anx7805_i2c_probe,
	.remove = anx7805_i2c_remove,
	.id_table = anx7805_id,
};

static void __init anx7805_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&anx7805_driver);
	if (ret)
		pr_err("%s: failed to register anx7805 driver\n", __func__);
}

static int __init anx7805_init(void)
{
	async_schedule(anx7805_init_async, NULL);
	return 0;
}

static void __exit anx7805_exit(void)
{
	i2c_del_driver(&anx7805_driver);
}

module_init(anx7805_init);
module_exit(anx7805_exit);

MODULE_DESCRIPTION("Slimport transmitter ANX7805 driver");
MODULE_AUTHOR("swang@analogixsemi.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
