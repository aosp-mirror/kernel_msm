/* MonolithicPower MP3377 8-Channel, Synchronous, Boost, WLED Driver
 *
 * Copyright (C) 2017 Google, Inc.
 * Author: fairdzs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#define DRIVER_NAME "mp3377"

/* MP3377 registers. */
#define MP3377_REG_LED_CURRENT  0x00 /* LED Current Full-Scale and Channel Enable Register */
#define MP3377_REG_DIM_MODE     0x01 /* Dimming Mode and Parameter Set Register */
#define MP3377_REG_PROG_ENABLE  0x02 /* One-Time Program Enable and Analog Dimming Register */
#define MP3377_REG_SLOP_FREQ    0x03 /* Slope and PWM Dimming Frequency Register */
#define MP3377_REG_PWM_DIM      0x04 /* PWM Dimming Register */
#define MP3377_REG_ID_FAULT     0x05 /* ID and Fault Register */

/* Constants */
#define MP3377_CURRENT_PER_STEP         196     /* current increases by .196 mA/step */
#define MP3377_CURRENT_UPPER_LIMIT      50000   /* 50mA is the max current */
#define MP3377_CURRENT_DEFAULT          0x19FF  /* 5mA  */
#define MP3377_ONE_TIME_REG_DEFAULT     0x03FF  /* one time register default value */
#define MP3377_ENABLE_WAIT_TIME         2       /* wait for EN to stabilize */
#define MP3377_ENABLE_ALL_CHANNELS      0xFF    /* Enable all 8 channels */

/*
 * struct MP3377 - device data
 * @regmap:	Register map.
 */
struct mp3377 {
	struct regmap *regmap;
};

static bool mp3377_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MP3377_REG_ID_FAULT:
		return false;
	default:
		return true;
	}
}

static bool mp3377_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MP3377_REG_ID_FAULT:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config mp3377_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MP3377_REG_ID_FAULT,
	.writeable_reg = mp3377_writeable_reg,
	.volatile_reg = mp3377_volatile_reg,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

/* Configure current via its register with associated device properties. */
static int mp3377_of_init(struct device *dev)
{
	struct mp3377 *mp3377 = dev_get_drvdata(dev);
	int err;
	u32 val;
	u16 word;

	if (!of_property_read_u32(dev->of_node, "mps,current-microamps",
				  &val)) {
		if (val > MP3377_CURRENT_UPPER_LIMIT)
			val = MP3377_CURRENT_UPPER_LIMIT;

		val = val / MP3377_CURRENT_PER_STEP;
		word = (val << 8) | MP3377_ENABLE_ALL_CHANNELS;
	}
	else {
		word = MP3377_CURRENT_DEFAULT;
	}

	err = regmap_read(mp3377->regmap, MP3377_REG_LED_CURRENT, &val);
	if (err < 0) {
		dev_err(dev, "mp3377 regmap read failed: %d", err);
		return err;
	}

	err = regmap_write(mp3377->regmap, MP3377_REG_LED_CURRENT, word);
	if (err < 0) {
		dev_err(dev, "mp3377 regmap write failed: %d", err);
		return err;
	}

	return 0;
}

static int mp3377_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	unsigned int regval;
	struct mp3377 *mp3377;
	struct gpio_desc *enable_gpio;
	int err;

	mp3377 = devm_kzalloc(dev, sizeof(*mp3377), GFP_KERNEL);
	if (!mp3377)
		return -ENOMEM;

	dev_set_drvdata(dev, mp3377);

	mp3377->regmap = devm_regmap_init_i2c(client, &mp3377_regmap_config);
	if (IS_ERR(mp3377->regmap)) {
		err = PTR_ERR(mp3377->regmap);
		dev_err(dev, "mp3377 regmap init failed: %d", err);
		return err;
	}

	enable_gpio = gpiod_get(dev, "mps,enable", GPIOD_OUT_HIGH);
	err = PTR_ERR(enable_gpio);
	if (err == -ENOENT || err == -ENODEV) {
		dev_dbg(dev, "No mp3377 enable gpio.");
		enable_gpio = NULL;
	} else if (err == -EPROBE_DEFER) {
		dev_dbg(dev, "Defer due to mp3377 enable gpio.");
		return err;
	} else if (IS_ERR(enable_gpio)) {
		dev_err(dev, "Error getting mp3377 enable gpio: %d", err);
		return err;
	}

	if (enable_gpio) {
		/* wait for enable high before continue */
		msleep(MP3377_ENABLE_WAIT_TIME);
	}

	err = regmap_read(mp3377->regmap, MP3377_REG_ID_FAULT, &regval);
	if (err < 0) {
		dev_err(dev, "failed to read mp3377 FAULT register: %d", err);
		goto exit;
	}

	err = mp3377_of_init(dev);
	if (err < 0) {
		dev_err(dev, "failed mp3377 mp3377_of_init %d", err);
		goto exit;
	}

exit:
	if (enable_gpio) {
		/* TODO set the GPIO pin low before exit
		 * gpiod_set_value_cansleep(enable_gpio, 0);
		 */
		gpiod_put(enable_gpio);
	}

	return err;
}

static const struct i2c_device_id mp3377_i2c_ids[] = {
	{ "mp3377", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mp3377_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id mp3377_of_ids[] = {
	{ .compatible = "mps,mp3377", },
	{}
};
MODULE_DEVICE_TABLE(of, mp3377_of_ids);
#endif

static struct i2c_driver mp3377_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(mp3377_of_ids),
	},
	.probe		= mp3377_probe,
	.id_table	= mp3377_i2c_ids,
};

module_i2c_driver(mp3377_driver);

MODULE_AUTHOR("Farid Zare Seisan <faridzs@google.com>");
MODULE_DESCRIPTION("MPS MP3377 8-Channel, Synchronous, Boost, WLED Driver");
MODULE_LICENSE("GPL");