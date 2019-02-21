/*
 * LP5569 LED chip driver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This driver was implemented with lp5521 led linux driver as reference
 * by "Milo Kim <milo.kim@ti.com>"
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/platform_data/leds-lp55xx.h>
#include <linux/slab.h>

#include "leds-lp55xx-common.h"

#define LP5569_PROGRAM_LENGTH   	32
#define LP5569_MAX_LEDS         	3
#define LP5569_NUM_ENGINES		3
#define LP5569_NUM_PAGES		16

/* Registers */
#define LP5569_REG_ENABLE		0x00
#define LP5569_REG_EXEC  		0x01
#define LP5569_REG_OP_MODE		0x02
#define LP5569_REG_ENABLE_LEDS_MSB      0x04
#define LP5569_REG_ENABLE_LEDS_LSB      0x05
#define LP5569_REG_LED_CTRL_BASE	0x07
#define LP5569_REG_LED_PWM_BASE 	0x16
#define LP5569_REG_LED_CURRENT_BASE     0x22
#define LP5569_REG_CONFIG		0x2F
#define LP5569_REG_STATUS		0x3C
#define LP5569_REG_RESET		0x3F
#define LP5569_REG_MASTER_FADER_BASE    0x46
#define LP5569_REG_CH1_PROG_START	0x4B
#define LP5569_REG_CH2_PROG_START	0x4C
#define LP5569_REG_CH3_PROG_START	0x4D
#define LP5569_REG_PROG_PAGE_SEL	0x4F
#define LP5569_REG_PROG_MEM		0x50
#define LP5569_PAGE_SEL_START		0
/* Bit description in registers */
#define LP5569_ENABLE			0x40
#define LP5569_AUTO_INC 		0x40
#define LP5569_PWR_SAVE 		0x20
#define LP5569_PWM_PWR_SAVE		0x04
#define LP5569_CP_AUTO			0x18
#define LP5569_AUTO_CLK 		0x02
#define LP5569_INTERNAL_CLK		0x01

#define LP5569_EN_LEDTEST		0x80
#define LP5569_LEDTEST_DONE		0x80
#define LP5569_RESET			0xFF
#define LP5569_ADC_SHORTCIRC_LIM	80
#define LP5569_EXT_CLK_USED		0x08
#define LP5569_ENG_STATUS_MASK  	0x07

#define LP5569_FADER_MAPPING_MASK 	0xC0
#define LP5569_FADER_MAPPING_SHIFT	6

/* Memory Page Selection */
#define LP5569_PAGE_ENG1		0
#define LP5569_PAGE_ENG2		1
#define LP5569_PAGE_ENG3		2
#define LP5569_PAGE_MUX1		3
#define LP5569_PAGE_MUX2		4
#define LP5569_PAGE_MUX3		5

/* Program Memory Operations */
#define LP5569_MODE_ENG1_M		0xC0 /* Operation Mode Register */
#define LP5569_MODE_ENG2_M		0x30
#define LP5569_MODE_ENG3_M		0x0C

#define LP5569_LOAD_ENG1		0x40
#define LP5569_LOAD_ENG2		0x10
#define LP5569_LOAD_ENG3		0x04

#define LP5569_REG_VARIABLE		0x42
#define LP5569_ENGINE1_PC  		0x30

#define LP5569_ENG1_IS_LOADING(mode)	\
	((mode & LP5569_MODE_ENG1_M) == LP5569_LOAD_ENG1)
#define LP5569_ENG2_IS_LOADING(mode)	\
	((mode & LP5569_MODE_ENG2_M) == LP5569_LOAD_ENG2)
#define LP5569_ENG3_IS_LOADING(mode)	\
	((mode & LP5569_MODE_ENG3_M) == LP5569_LOAD_ENG3)

#define LP5569_EXEC_ENG1_M		0xC0
#define LP5569_EXEC_ENG2_M		0x30
#define LP5569_EXEC_ENG3_M		0x0C
#define LP5569_EXEC_M     		0xFC
#define LP5569_RUN_ENG1   		0x80
#define LP5569_RUN_ENG2   		0x20
#define LP5569_RUN_ENG3   		0x08
#define LP5569_CLK_32K    		32768
#define LP5569_MAX_FW_LEN 		(LP5569_NUM_PAGES*LP5569_PROGRAM_LENGTH)

#define LED_ACTIVE(mux, led)		(!!(mux & (0x0001 << led)))

enum lp5569_chip_id {
	LP5569,
};

static inline void lp5569_wait_opmode_done(void)
{
	usleep_range(1000, 2000);
}

static void lp5569_set_led_current(struct lp55xx_led *led, u8 led_current)
{
	led->led_current = led_current;
	lp55xx_write(led->chip, LP5569_REG_LED_CURRENT_BASE + led->chan_nr,
		led_current);
}

static int lp5569_post_init_device(struct lp55xx_chip *chip)
{
	int ret = 0;
	u8 conf = 0;

	ret = lp55xx_write(chip, LP5569_REG_ENABLE, LP5569_ENABLE);
	if (ret)
		goto error;

	lp5569_wait_opmode_done();

	ret = lp55xx_write(chip, LP5569_REG_CONFIG,
			    LP5569_AUTO_INC | LP5569_PWR_SAVE |
			    LP5569_CP_AUTO | LP5569_INTERNAL_CLK);

	ret = lp55xx_read(chip, LP5569_REG_CONFIG, &conf);
	dev_info(&chip->cl->dev, "initialized lp5569 device with config %x", conf);

error:
	if(ret)
		dev_err(&chip->cl->dev, "lp55xx_write() failed!\n");

	return ret;
}

static void lp5569_load_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5569_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5569_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5569_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5569_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5569_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5569_LOAD_ENG3,
	};

	lp55xx_update_bits(chip, LP5569_REG_OP_MODE, mask[idx], val[idx]);

	lp5569_wait_opmode_done();
}

static void lp5569_stop_engine(struct lp55xx_chip *chip)
{
	lp55xx_write(chip, LP5569_REG_OP_MODE, 0);
	lp5569_wait_opmode_done();
}

static void lp5569_turn_off_channels(struct lp55xx_chip *chip)
{
	int i;
	for (i = 0; i < LP5569_MAX_LEDS; i++)
		lp55xx_write(chip, LP5569_REG_LED_PWM_BASE + i, 0);
}

static void lp5569_run_engine(struct lp55xx_chip *chip, bool start)
{
	u8 mode = 0;
	u8 exec = 0;
	u8 m = 0;
	u8 e = 0;
	int ret = 0;

	ret = lp55xx_read(chip, LP5569_REG_OP_MODE, &mode);
	if (ret < 0)
		return;

	ret = lp55xx_read(chip, LP5569_REG_EXEC, &exec);
	if (ret < 0)
		return;

	/* stop engine */
	if (!start) {
		dev_info(&chip->cl->dev, "stopping lp5569 engine");
		lp5569_stop_engine(chip);
		lp5569_turn_off_channels(chip);
		return;
	}

	dev_info(&chip->cl->dev, "starting lp5569 engine with mode %x exec %x", mode, exec);

	/* change operation mode to RUN only when each engine is loading */
	if (LP5569_ENG1_IS_LOADING(mode)) {
		m |= LP5569_RUN_ENG1;
		e |= LP5569_RUN_ENG1;
	}

	if (LP5569_ENG2_IS_LOADING(mode)) {
		m |= LP5569_RUN_ENG2;
		e |= LP5569_RUN_ENG2;
	}

	if (LP5569_ENG3_IS_LOADING(mode)) {
		m |= LP5569_RUN_ENG3;
		e |= LP5569_RUN_ENG3;
	}
	lp55xx_write(chip, LP5569_REG_OP_MODE, m);
	lp5569_wait_opmode_done();
	lp55xx_write(chip, LP5569_REG_EXEC, e);
}

static ssize_t lp5569_init(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int pos = 0;
	int ret;

	ret = lp55xx_write(chip, LP5569_REG_RESET, LP5569_RESET);
	if (ret) {
		dev_info(dev, "lp55xx_write() failed!\n");
		pos = sprintf(buf, "FAIL\n");
	}

	ret = lp55xx_write(chip, LP5569_REG_CONFIG,
			    LP5569_AUTO_INC | LP5569_PWR_SAVE |
			    LP5569_CP_AUTO | LP5569_INTERNAL_CLK);

	if (ret) {
		dev_info(dev, "lp55xx_write() failed!\n");
		pos = sprintf(buf, "FAIL\n");
	}

	ret = lp55xx_write(chip, LP5569_REG_ENABLE, LP5569_ENABLE);
	if (ret) {
		dev_info(dev, "lp55xx_write() failed!\n");
		pos = sprintf(buf, "FAIL\n");
	}
	lp5569_wait_opmode_done();
	pos = sprintf(buf, "OK\n");
	return pos;
}

/* Loop through an input hex octet string and put the output in a byte array */
static ssize_t lp5569_start_addr3(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;
	int i = 0;
	u8 start_addr[LP5569_NUM_ENGINES];
	int start_offset[] = {
		LP5569_REG_CH1_PROG_START,
		LP5569_REG_CH2_PROG_START,
		LP5569_REG_CH3_PROG_START,};

	if ((len % 2) || (len / 2 != LP5569_NUM_ENGINES)) {
		dev_info(dev, "Input was incorrect or odd length\n");
		return -EINVAL;
	}

	ret = hex2bin(start_addr, buf, LP5569_NUM_ENGINES);
	if (ret < 0) {
		dev_info(dev, "Input was not convertable to hex\n");
		return -EINVAL;
	}

	/* program start addresses to the controller */
	mutex_lock(&chip->lock);
	for (i = 0; i < LP5569_NUM_ENGINES; i++) {
		ret = lp55xx_write(chip, start_offset[i], start_addr[i]);
		if (ret) {
			dev_info(dev, "lp55xx_write() failed!\n");
			ret = -EIO;
			goto leave;
		}

		ret = lp55xx_write(chip, LP5569_ENGINE1_PC + i, start_addr[i]);
		if (ret) {
			dev_info(dev, "lp55xx_write() failed!\n");
			ret = -EIO;
			goto leave;
		}
	}

	ret = len;

leave:
	mutex_unlock(&chip->lock);
	return ret;
}

static ssize_t lp5569_leds_run(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;
	u8 run = 0;

	/* read input */
	ret = kstrtou8(buf, 0, &run);
	if (ret) {
		dev_info(dev, "kstrou8() failed\n");
		return -EINVAL;
	}

	if (run != 0 && run != 1) {
		dev_info(dev, "leds_run, valid values: 0/1\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	if (run == 0)
		lp5569_run_engine(chip, false);
	else
		lp5569_run_engine(chip, true);
	mutex_unlock(&chip->lock);

	return len;
}

static void lp5569_load_all_engines(struct lp55xx_chip *chip)
{
	/* put all three engines in load mode */
	int i;
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		chip->engine_idx = i;
		lp5569_load_engine(chip);
	}
}

static ssize_t lp5569_firmware_load(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int page_num = LP5569_PAGE_SEL_START;
	u8 fw_bytes[LP5569_MAX_FW_LEN] = {0};
	int ret;
	int fw_len = len/2;

	if ((len % 2) || (fw_len > LP5569_MAX_FW_LEN)) {
		dev_info(dev, "Input was too long or of odd length\n %d %d", (int)len, (int)fw_len);
		return -EINVAL;
	}

	ret = hex2bin(fw_bytes, buf, fw_len);
	if (ret < 0) {
		dev_info(dev, "Input was not convertable to hex\n");
		return -EINVAL;
	}

	/* Start writing f/w to LED controller mem */
	mutex_lock(&chip->lock);

	/* put all engines in load mode */
	lp5569_load_all_engines(chip);

	for (page_num = 0; page_num < LP5569_NUM_PAGES; page_num++) {
		ret = lp55xx_write(chip, LP5569_REG_PROG_PAGE_SEL, page_num);
		if (ret) {
			dev_info(dev, "lp55xx_write(PAGE_SEL) failed!\n");
			goto leave;
		}

		ret = i2c_smbus_write_i2c_block_data(to_i2c_client(dev),
			LP5569_REG_PROG_MEM, LP5569_PROGRAM_LENGTH,
			fw_bytes + page_num*LP5569_PROGRAM_LENGTH);
		if (ret) {
			dev_info(dev, "i2c_smbus_write_i2c_block_data failed!\n");
			goto leave;
		}
	}

	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}

static ssize_t show_variable(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	u8 variable;
	int ret;

	ret = lp55xx_read(chip, LP5569_REG_VARIABLE, &variable);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", variable);
}

static ssize_t store_variable(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	u8 variable;
	int ret;

	ret = kstrtou8(buf, 0, &variable);
	if (ret)
		return ret;

	dev_dbg(dev, "Writing down %d\n", variable);

	ret = lp55xx_write(chip, LP5569_REG_VARIABLE, variable);
	if (ret)
		return ret;

	return len;
}

static ssize_t lp5569_selftest(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	int i, ret, pos = 0;
	u8 status;

	mutex_lock(&chip->lock);

	ret = lp55xx_read(chip, LP5569_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	for (i = 0; i < LP5569_MAX_LEDS; i++) {
		/* Skip non-existing channels */
		if (pdata->led_config[i].led_current == 0)
			continue;

		/* Set default current */
		lp55xx_write(chip, LP5569_REG_LED_CURRENT_BASE + i,
			pdata->led_config[i].led_current);

		lp55xx_write(chip, LP5569_REG_LED_PWM_BASE + i, 0xff);

		usleep_range(3000, 6000);

		lp55xx_write(chip, LP5569_REG_LED_PWM_BASE + i, 0x00);

		/* Restore current */
		lp55xx_write(chip, LP5569_REG_LED_CURRENT_BASE + i,
			led->led_current);
		led++;
	}
	pos = sprintf(buf, "OK\n");
	goto release_lock;
fail:
	pos = sprintf(buf, "FAIL\n");

release_lock:
	mutex_unlock(&chip->lock);

	return pos;
}

static int lp5569_led_brightness(struct lp55xx_led *led)
{
	struct lp55xx_chip *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);
	ret = lp55xx_write(chip, LP5569_REG_LED_PWM_BASE + led->chan_nr,
		     led->brightness);
	mutex_unlock(&chip->lock);

	return ret;
}

static LP55XX_DEV_ATTR_WO(firmware_load, lp5569_firmware_load);
static LP55XX_DEV_ATTR_WO(start_addr3, lp5569_start_addr3);
static LP55XX_DEV_ATTR_WO(leds_run, lp5569_leds_run);
static LP55XX_DEV_ATTR_RO(init, lp5569_init);
static LP55XX_DEV_ATTR_RO(selftest, lp5569_selftest);
static LP55XX_DEV_ATTR_RW(variable, show_variable, store_variable);

static struct attribute *lp5569_attributes[] = {
	&dev_attr_firmware_load.attr,
	&dev_attr_start_addr3.attr,
	&dev_attr_leds_run.attr,
	&dev_attr_selftest.attr,
	&dev_attr_variable.attr,
	&dev_attr_init.attr,
	NULL,
};

static const struct attribute_group lp5569_group = {
	.attrs = lp5569_attributes,
};

/* Chip specific configurations */
static struct lp55xx_device_config lp5569_cfg = {
	.reset = {
		.addr = LP5569_REG_RESET,
		.val  = LP5569_RESET,
	},
	.enable = {
		.addr = LP5569_REG_ENABLE,
		.val  = LP5569_ENABLE,
	},
	.max_channel  	    = LP5569_MAX_LEDS,
	.post_init_device   = lp5569_post_init_device,
	.brightness_fn	    = lp5569_led_brightness,
	.set_led_current    = lp5569_set_led_current,
	.dev_attr_group     = &lp5569_group,
};

static int lp5569_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct lp55xx_chip *chip;
	struct lp55xx_led *led;
	struct lp55xx_platform_data *pdata = dev_get_platdata(&client->dev);
	struct device_node *np = client->dev.of_node;
	struct clk *clk;

	if (!pdata) {
		if (np) {
			pdata = lp55xx_of_populate_pdata(&client->dev, np);
			if (IS_ERR(pdata))
				return PTR_ERR(pdata);
		} else {
			dev_err(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	led = devm_kzalloc(&client->dev,
			sizeof(*led) * pdata->num_channels, GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	chip->cl = client;
	chip->pdata = pdata;
	chip->cfg = &lp5569_cfg;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, led);

	ret = lp55xx_init_device(chip);
	if (ret)
		goto err_init;

	dev_info(&client->dev, "%s Programmable led chip found\n", id->name);

	ret = lp55xx_register_leds(led, chip);
	if (ret)
		goto err_register_leds;

	ret = lp55xx_register_sysfs(chip);
	if (ret) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto err_register_sysfs;
	}

	if (pdata->clock_mode == LP55XX_CLOCK_EXT) {
		clk = devm_clk_get(&chip->cl->dev, "32k_clk");
		if (IS_ERR(clk))
			goto use_internal_clk;

		ret = clk_prepare_enable(clk);
		if (ret)
			goto use_internal_clk;

		clk_set_rate(clk, LP5569_CLK_32K);

		if (clk_get_rate(clk) != LP5569_CLK_32K) {
			clk_disable_unprepare(clk);
			goto use_internal_clk;
		}

		dev_info(&chip->cl->dev, "%dHz external clock used\n",  LP5569_CLK_32K);
	}

	return 0;

use_internal_clk:
	dev_info(&chip->cl->dev, "unable to get ext_clk, using internal\n");
	return 0;

err_register_sysfs:
	lp55xx_unregister_leds(led, chip);
err_register_leds:
	lp55xx_deinit_device(chip);
err_init:
	return ret;
}

static int lp5569_remove(struct i2c_client *client)
{
	struct lp55xx_led *led = i2c_get_clientdata(client);
	struct lp55xx_chip *chip = led->chip;

	lp5569_stop_engine(chip);
	lp55xx_unregister_sysfs(chip);
	lp55xx_unregister_leds(led, chip);
	lp55xx_deinit_device(chip);

	return 0;
}

static const struct i2c_device_id lp5569_id[] = {
	{ "lp5569",  LP5569 },
	{}
};

MODULE_DEVICE_TABLE(i2c, lp5569_id);

#ifdef CONFIG_OF
static const struct of_device_id of_lp5569_leds_match[] = {
	{ .compatible = "ti,lp5569", },
	{},
};

MODULE_DEVICE_TABLE(of, of_lp5569_leds_match);
#endif

static struct i2c_driver lp5569_driver = {
	.driver = {
		.name	= "lp5569",
		.of_match_table = of_match_ptr(of_lp5569_leds_match),
	},
	.probe		= lp5569_probe,
	.remove		= lp5569_remove,
	.id_table	= lp5569_id,
};

module_i2c_driver(lp5569_driver);

MODULE_AUTHOR("Kushal Prakash <kushalk@google.com>");
MODULE_DESCRIPTION("LP5569 LED engine");
MODULE_LICENSE("GPL v2");
