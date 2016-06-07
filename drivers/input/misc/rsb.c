/*
 * Copyright (C) 2016 Google, Inc.
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define NUM_WRITE_RETRIES	5

#define RSB_MAGIC_PID		0x30

/* MOTION Register addresses */
#define RSB_MOTION	0x02
#define RSB_DELTA_X	0x03
#define RSB_DELTA_Y	0x04
#define RSB_CONFIG	0x06
#define MOTION_BITMASK	0x80

#define RSB_DELAY_MS_AFTER_VDD 10

struct rsb_drv_data {
	struct spi_device *device;
	struct dentry *dent;
	struct input_dev *in_dev;
	int cs;
	struct regulator *vld_reg; /* power supply voltage v3.3 */
	struct regulator *vdd_reg; /* power supply voltage for IO v1.8 */
	struct work_struct init_work;
};

static struct spi_device_id rsb_spi_id[] = {
	{"rsb", 0},
	{},
};

/*
 * We iterate over this array, writing all the registers to the addresses
 * NOTE: Please make sure the #define values are kept in lock step with the
 * size of the array. If not, the RSB will *not* be initialized properly!
 */
#define INIT_WRITES_FIRST_BATCH_INDEX	5
#define INIT_WRITES_SECOND_BATCH_INDEX	34

static uint8_t init_writes[][2] = {
	{0x05, 0xA0 }, /* OPERATION_MODE */
	{0x0D, 0x05 }, /* RES_X */
	{0x0E, 0x0A }, /* RES_Y */
	{0x19, 0x04 }, /* ORIENTATION */
	{0x2B, 0x6D },
	{0x5C, 0xD7 }, /* LD_SRC */
	{0x09, 0x22 }, /* WRITE_PROTECT */
	{0x2A, 0x03 },
	{0x30, 0x4C },
	{0x33, 0x90 },
	{0x36, 0xCC },
	{0x37, 0x51 },
	{0x38, 0x01 },
	{0x3A, 0x7A },
	{0x40, 0x38 },
	{0x41, 0x33 },
	{0x42, 0x4F },
	{0x43, 0x83 },
	{0x44, 0x4F },
	{0x45, 0x80 },
	{0x46, 0x23 },
	{0x47, 0x49 },
	{0x48, 0xC3 },
	{0x49, 0x49 },
	{0x4A, 0xC0 },
	{0x52, 0x00 },
	{0x61, 0x80 },
	{0x62, 0x51 },
	{0x67, 0x53 },
	{0x68, 0x13 },
	{0x6C, 0x10 },
	{0x6F, 0xF6 },
	{0x71, 0x28 },
	{0x72, 0x28 },
	{0x79, 0x08 },
};

/*
 * SPI device protocol to read a register.
 * Write a 8 bit address value, and read an 8 bit value back.
 */
static int rsb_read(struct rsb_drv_data *rsb_data, uint8_t *rx, uint8_t addr)
{

	/* TODO(pmalani): Do we need a timeout check?? */
	int ret = 0;
	uint8_t read_buf[2];
	struct spi_transfer xfers[2] = {
		{
			.tx_buf = &read_buf[0],
			.len = 1,
			.delay_usecs = 10,
		},
		{
			.rx_buf = &read_buf[1],
			.len = 1,
		},
	};

	read_buf[0] = addr;

	gpio_set_value(rsb_data->cs, 0);
	ret = spi_sync_transfer(rsb_data->device, xfers, 2);
	if (ret == 0) {
		*rx = read_buf[1];
	} else {
		dev_err(&rsb_data->device->dev,
			"SPI Protocol message read failed\n");
	}

	gpio_set_value(rsb_data->cs, 1);
	return ret;
}

static int get_test_read(void *data, u64 *val)
{
	struct rsb_drv_data *rsb_data = data;
	uint8_t rx_data;

	dev_info(&rsb_data->device->dev, "Writing to debugfs\n");

	/* Read the sensorPID */
	if (rsb_read(data, &rx_data, 0x00) == 0)
		dev_info(&rsb_data->device->dev, "PID is %x\n",
			rx_data);
	else
		dev_err(&rsb_data->device->dev, "read error\n");

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(test_read_fops, get_test_read, NULL, "%llu\n");

/*
 * SPI Protocol to write a byte to an address.
 * Returns 0 on success, -1 otherwise.
 */
static int rsb_write(struct rsb_drv_data *rsb_data, uint8_t tx_val,
	uint8_t addr)
{
	int ret = 0;
	uint8_t write_buf[2];

	struct spi_transfer xfer = {
			.tx_buf = write_buf,
			.len = 2,
	};

	write_buf[0] = (1 << 7) | addr;
	write_buf[1] = tx_val;

	gpio_set_value(rsb_data->cs, 0);
	ret = spi_sync_transfer(rsb_data->device, &xfer, 1);
	if (ret != 0) {
		dev_err(&rsb_data->device->dev,
			"SPI Protocol write message failed\n");
	}
	gpio_set_value(rsb_data->cs, 1);

	return ret;
}

/*
 * SPI Protocol to write a byte to an address.
 * Also reads back the address to make sure it's written correctly.
 * Returns 0 on success, -1 otherwise.
 */
static int rsb_write_read(struct rsb_drv_data *rsb_data, uint8_t tx_val,
	uint8_t addr)
{
	uint8_t num_retries = NUM_WRITE_RETRIES;
	uint8_t read_val;

	do {
		if (rsb_write(rsb_data, tx_val, addr))
			break;

		if (rsb_read(rsb_data, &read_val, addr))
			break;

		if (read_val == tx_val) {
			dev_info(&rsb_data->device->dev,
				"Addr %x: Wrote %x got back %x\n",
				addr, tx_val, read_val);
			return 0;
		}
	} while (num_retries-- > 0);

	dev_warn(&rsb_data->device->dev,
		"Write_read %x to addr %x failed\n", tx_val, addr);
	return -EIO;
}

static int rsb_open(struct rsb_drv_data *rsb_data)
{
	int ret;

	/* TODO(pmalani): How much of this is prepopulated from DT? */
	rsb_data->device->max_speed_hz = 2000000;
	rsb_data->device->mode = SPI_MODE_0;
	rsb_data->device->bits_per_word = 8;
	ret = spi_setup(rsb_data->device);
	if (!ret) {
		dev_info(&rsb_data->device->dev,
			"SPI device set up successfully!\n");
		/* Toggle CS low for 1ms at power up. */
		gpio_set_value(rsb_data->cs, 0);
		udelay(1000);
		gpio_set_value(rsb_data->cs, 1);
	}

	return ret;
}

static int rsb_create_debugfs(struct rsb_drv_data *rsb_data)
{
	struct dentry *file;

	rsb_data->dent = debugfs_create_dir("rsb", NULL);
	if (IS_ERR(rsb_data->dent)) {
		dev_err(&rsb_data->device->dev,
			"rsb driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	file = debugfs_create_file("test_read", 0644, rsb_data->dent,
			(void *)rsb_data, &test_read_fops);
	if (IS_ERR(file)) {
		dev_err(&rsb_data->device->dev,
			"debugfs create file for test_read failed\n");
		return -EFAULT;
	}
	return 0;
}

#ifdef CONFIG_OF
static int rsb_parse_dt(struct spi_device *spi_dev)
{
	int ret;
	struct device_node *dt = spi_dev->dev.of_node;
	struct rsb_drv_data *rsb_data = spi_get_drvdata(spi_dev);

	ret = rsb_data->cs = of_get_named_gpio(dt, "rsb,spi-cs-gpio", 0);
	dev_info(&spi_dev->dev, "cs GPIO read from DT:%u\n",
			rsb_data->cs);

	return 0;
}
#else
static int rsb_parse_dt(struct spi_device *spi_dev)
{
	dev_err(&spi_dev->dev,
		"Kernel not configured with DT support\n");
	return -EINVAL;
}
#endif

/*
 * Sequence of start up writes mandated by the RSB data sheet.
 */
static int rsb_init_sequence(struct rsb_drv_data *rsb_data)
{
	uint8_t read_val;
	int i;

	/* Read the SensorPID to ensure the SPI Link is valid */
	if (rsb_read(rsb_data, &read_val, 0x00) == 0) {
		if (read_val != RSB_MAGIC_PID) {
			dev_err(&rsb_data->device->dev,
				"Couldn't read SPI Magic PID,"
				 "value read: %u\n", read_val);
			return -EIO;
		}
	}

	if (rsb_write(rsb_data, 0x00, 0x7F) != 0)
		return -EIO;

	if (rsb_write_read(rsb_data, 0x5A, 0x09) != 0)
		return -EIO;

	for (i = 0; i <= INIT_WRITES_FIRST_BATCH_INDEX; i++) {
		if (rsb_write_read(rsb_data, init_writes[i][1],
			init_writes[i][0]) != 0) {
			return -EIO;
		}
	}

	if (rsb_write(rsb_data, 0x01, 0x7F) != 0)
		return -EIO;

	for (; i <= INIT_WRITES_SECOND_BATCH_INDEX; i++) {
		if (rsb_write_read(rsb_data, init_writes[i][1],
			init_writes[i][0]) != 0) {
			return -EIO;
		}
	}

	if (rsb_write(rsb_data, 0x00, 0x7F) != 0)
		return -EIO;

	if (rsb_write_read(rsb_data, 0x00, 0x09) != 0)
		return -EIO;

	dev_info(&rsb_data->device->dev, "Rsb init success\n");
	return 0;
}

static irqreturn_t rsb_handler(int irq, void *dev_id)
{
	static int8_t delta_x, delta_y, motion;
	struct rsb_drv_data *rsb_data = dev_id;

	do {
		if (rsb_read(rsb_data, &delta_x, RSB_DELTA_X) != 0)
			break;
		if (rsb_read(rsb_data, &delta_y, RSB_DELTA_Y) != 0)
			break;
		if (delta_x != 0) {
			input_report_rel(rsb_data->in_dev, REL_WHEEL,
					delta_x);
			input_sync(rsb_data->in_dev);
		}
		if (rsb_read(rsb_data, &motion, RSB_MOTION) != 0)
			break;
	} while (motion & MOTION_BITMASK);

	return IRQ_HANDLED;
}

static int rsb_init_regulator(struct spi_device *spi_dev)
{
	struct rsb_drv_data *rsb_data = spi_get_drvdata(spi_dev);
	int ret = 0;

	if (!rsb_data->vld_reg) {
		rsb_data->vld_reg = devm_regulator_get(&spi_dev->dev,
			"rsb,vld");
		if (IS_ERR(rsb_data->vld_reg)) {
			dev_warn(&spi_dev->dev,
				"regulator: VLD request failed\n");
			ret = (int)rsb_data->vld_reg;
			rsb_data->vld_reg = NULL;
			return ret;
		}
	}


	if (!rsb_data->vdd_reg) {
		rsb_data->vdd_reg = devm_regulator_get(&spi_dev->dev,
			"rsb,vdd");
		if (IS_ERR(rsb_data->vdd_reg)) {
			dev_warn(&spi_dev->dev,
				"regulator: VDD request failed\n");
			ret = (int)rsb_data->vld_reg;
			rsb_data->vdd_reg = NULL;
			return ret;
		}
	}
	return 0;
}

static int rsb_set_regulator(struct regulator *reg, int enable)
{
	int ret;

	if (!reg)
		return 0;

	if (enable)
		ret = regulator_enable(reg);
	else
		ret = regulator_disable(reg);

	return ret;
}

static int rsb_set_regulator_vld(struct spi_device *spi, int enable)
{
	struct rsb_drv_data *rsb_data = spi_get_drvdata(spi);
	int ret;

	ret = rsb_set_regulator(rsb_data->vld_reg, enable);
	if (ret)
		dev_err(&spi->dev, "couldn't %s regulator vld\n",
				enable? "enable" : "disable");
	return ret;
}

static int rsb_set_regulator_vdd(struct spi_device *spi, int enable)
{
	struct rsb_drv_data *rsb_data = spi_get_drvdata(spi);
	int ret;

	ret = rsb_set_regulator(rsb_data->vdd_reg, enable);
	if (ret)
		dev_err(&spi->dev, "couldn't %s regulator vdd\n",
				enable? "enable" : "disable");

	return ret;
}

static void rsb_init_work(struct work_struct *work)
{
	struct rsb_drv_data *rsb_data = container_of(work,
			struct rsb_drv_data, init_work);
	struct spi_device *spi = rsb_data->device;

	/* Initialize regulators */
	if (rsb_init_regulator(spi))
		return;

	/* Turn on VDD */
	if (rsb_set_regulator_vdd(spi, 1))
		return;

	msleep(RSB_DELAY_MS_AFTER_VDD);

	/* Should the open of the SPI bus be done only once?? */
	rsb_open(rsb_data);

	if (rsb_init_sequence(rsb_data))
		goto error;

	/* Turn on VLD */
	if (rsb_set_regulator_vld(spi, 1))
		goto error;

	return;

error:
	rsb_set_regulator_vdd(spi, 0);
	dev_err(&spi->dev, "RSB init failed\n");
}

static int rsb_probe(struct spi_device *spi)
{
	struct rsb_drv_data *rsb_data;
	int err = 0;

	rsb_data = devm_kzalloc(&spi->dev, sizeof(struct rsb_drv_data),
		GFP_KERNEL);

	if (!rsb_data)
		return -ENOMEM;

	spi_set_drvdata(spi, rsb_data);
	rsb_data->device = spi;

	err = rsb_parse_dt(spi);
	if (err)
		return err;

	if (gpio_is_valid(rsb_data->cs)) {
		err = devm_gpio_request(&spi->dev, rsb_data->cs,
			"rsb_spi_cs");
		if (err) {
			dev_err(&spi->dev,
				"spi_cs_gpio:%d request failed\n",
				rsb_data->cs);
			return err;
		} else {
			gpio_direction_output(rsb_data->cs, 1);
		}
	} else {
		dev_err(&spi->dev, "spi_cs_gpio:%d is not valid\n",
			rsb_data->cs);
		return -EINVAL;
	}

	INIT_WORK(&rsb_data->init_work, rsb_init_work);
	schedule_work(&rsb_data->init_work);

	/* Allocate and register an input device */
	rsb_data->in_dev = devm_input_allocate_device(&spi->dev);
	if (!rsb_data->in_dev) {
		dev_err(&spi->dev, "Couldn't allocate input device\n");
		return -ENOMEM;
	}

	rsb_data->in_dev->evbit[0] = BIT_MASK(EV_REL);
	rsb_data->in_dev->relbit[0] = BIT_MASK(REL_WHEEL);
	rsb_data->in_dev->name = "rsb";

	err = input_register_device(rsb_data->in_dev);
	if (err) {
		dev_err(&spi->dev, "Failed to register rsb device\n");
		return err;
	}

	err = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
		rsb_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING |
		IRQF_TRIGGER_LOW, "rsb_handler", rsb_data);
	if (err) {
		dev_err(&spi->dev,
			"Failed to register irq handler IRQ:%d\n",
			spi->irq);
		return err;
	}

	rsb_create_debugfs(rsb_data);
	return 0;
}

static int rsb_remove(struct spi_device *spi)
{
	struct rsb_drv_data *rsb_data;

	rsb_data = spi_get_drvdata(spi);

	gpio_set_value(rsb_data->cs, 1);

	debugfs_remove_recursive(rsb_data->dent);

	rsb_set_regulator_vld(spi, 0);
	rsb_set_regulator_vdd(spi, 0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rsb_suspend(struct device *device)
{
	struct spi_device *spidev = container_of(device,
		struct spi_device, dev);
	struct rsb_drv_data *rsb_data = spi_get_drvdata(spidev);
	int ret;

	ret = rsb_write(rsb_data, 0x8, RSB_CONFIG);
	if (ret)
		dev_warn(device, "Failed to put RSB into low power.\n");
	disable_irq(spidev->irq);
	return 0;
}

static int rsb_resume(struct device *device)
{
	struct spi_device *spidev = container_of(device,
		struct spi_device, dev);
	struct rsb_drv_data *rsb_data = spi_get_drvdata(spidev);
	int ret;

	enable_irq(spidev->irq);
	ret = rsb_write(rsb_data, 0x0, RSB_CONFIG);
	if (ret)
		dev_warn(device,
			"Failed to take RSB out of low power.\n");
	return 0;
}
#endif

SIMPLE_DEV_PM_OPS(rsb_pm_ops, rsb_suspend, rsb_resume);

static struct spi_driver rsb_driver = {
	.driver = {
		.name = "rsb",
		.owner = THIS_MODULE,
		.pm  = &rsb_pm_ops,
	},
	.probe = rsb_probe,
	.remove = rsb_remove,
	.id_table = rsb_spi_id,
};

int __init rsb_init(void)
{
	return spi_register_driver(&rsb_driver);
}

static void __exit rsb_exit(void)
{
	spi_unregister_driver(&rsb_driver);
}


module_init(rsb_init);
module_exit(rsb_exit);

MODULE_AUTHOR("Prashant Malani");
MODULE_LICENSE("GPL");
