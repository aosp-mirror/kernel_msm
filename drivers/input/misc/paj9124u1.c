/*
 *
 * PAJ9124U1 Optical Track Sensor driver.
 *
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

#define NUM_WRITE_RETRIES	5
#define SAMPLING_PERIOD_US_MIN	5000
#define SAMPLING_PERIOD_US_MAX	9000

#define PAJ9124U1_MAGIC_PID		0x30

/* MOTION Register addresses */
#define PAJ9124U1_MOTION	0x02
#define PAJ9124U1_DELTA_X	0x03
#define PAJ9124U1_DELTA_Y	0x04
#define PAJ9124U1_CONFIG	0x06
#define MOTION_BITMASK	0x80

#define PAJ9124U1_DELAY_MS_AFTER_VDD 10

struct paj9124u1_drv_data {
	struct spi_device *device;
	struct dentry *dent;
	struct input_dev *in_dev;
	struct regulator *vld_reg; /* power supply voltage v3.3 */
	struct regulator *vdd_reg; /* power supply voltage for IO v1.8 */
	struct work_struct init_work;
};

static struct spi_device_id paj9124u1_spi_id[] = {
	{"paj9124u1", 0},
	{},
};

/*
 * We iterate over this array, writing all the registers to the addresses
 * NOTE: Please make sure the #define values are kept in lock step with the
 * size of the array. If not, the paj9124u1 will *not* be initialized properly!
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
static int paj9124u1_read(struct paj9124u1_drv_data *paj9124u1_data, uint8_t *rx,
	uint8_t addr)
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

	ret = spi_sync_transfer(paj9124u1_data->device, xfers, 2);
	if (ret == 0) {
		*rx = read_buf[1];
	} else {
		dev_err(&paj9124u1_data->device->dev,
			"SPI Protocol message read failed\n");
	}

	return ret;
}

static int get_test_read(void *data, u64 *val)
{
	struct paj9124u1_drv_data *paj9124u1_data = data;
	uint8_t rx_data;

	dev_info(&paj9124u1_data->device->dev, "Writing to debugfs\n");

	/* Read the sensorPID */
	if (paj9124u1_read(data, &rx_data, 0x00) == 0)
		dev_info(&paj9124u1_data->device->dev, "PID is %x\n",
			rx_data);
	else
		dev_err(&paj9124u1_data->device->dev, "read error\n");

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(test_read_fops, get_test_read, NULL, "%llu\n");

/*
 * SPI Protocol to write a byte to an address.
 * Returns 0 on success, -1 otherwise.
 */
static int paj9124u1_write(struct paj9124u1_drv_data *paj9124u1_data, uint8_t tx_val,
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

	ret = spi_sync_transfer(paj9124u1_data->device, &xfer, 1);
	if (ret != 0) {
		dev_err(&paj9124u1_data->device->dev,
			"SPI Protocol write message failed\n");
	}

	return ret;
}

/*
 * SPI Protocol to write a byte to an address.
 * Also reads back the address to make sure it's written correctly.
 * Returns 0 on success, -1 otherwise.
 */
static int paj9124u1_write_read(struct paj9124u1_drv_data *paj9124u1_data,
	uint8_t tx_val, uint8_t addr)
{
	uint8_t num_retries = NUM_WRITE_RETRIES;
	uint8_t read_val;

	do {
		if (paj9124u1_write(paj9124u1_data, tx_val, addr))
			break;

		if (paj9124u1_read(paj9124u1_data, &read_val, addr))
			break;

		if (read_val == tx_val) {
			dev_info(&paj9124u1_data->device->dev,
				"Addr %x: Wrote %x got back %x\n",
				addr, tx_val, read_val);
			return 0;
		}
	} while (num_retries-- > 0);

	dev_warn(&paj9124u1_data->device->dev,
		"Write_read %x to addr %x failed\n", tx_val, addr);
	return -EIO;
}

static int paj9124u1_open(struct paj9124u1_drv_data *paj9124u1_data)
{
	int ret;
	struct spi_transfer xfer = {
		.len = 0,
		.delay_usecs = 1000,
	};

	/* TODO(pmalani): How much of this is prepopulated from DT? */
	paj9124u1_data->device->max_speed_hz = 2000000;
	paj9124u1_data->device->mode = SPI_MODE_3;
	paj9124u1_data->device->bits_per_word = 8;
	ret = spi_setup(paj9124u1_data->device);
	if (!ret) {
		dev_info(&paj9124u1_data->device->dev,
			"SPI device set up successfully!\n");

		/* Assert chip select for 1ms at power up. */
		ret = spi_sync_transfer(paj9124u1_data->device, &xfer, 1);
		if (ret != 0) {
			dev_err(&paj9124u1_data->device->dev,
				"SPI Protocol write message failed\n");
		}
	}

	return ret;
}

static int paj9124u1_create_debugfs(struct paj9124u1_drv_data *paj9124u1_data)
{
	struct dentry *file;

	paj9124u1_data->dent = debugfs_create_dir("paj9124u1", NULL);
	if (IS_ERR(paj9124u1_data->dent)) {
		dev_err(&paj9124u1_data->device->dev,
			"paj9124u1 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	file = debugfs_create_file("test_read", 0644, paj9124u1_data->dent,
			(void *)paj9124u1_data, &test_read_fops);
	if (IS_ERR(file)) {
		dev_err(&paj9124u1_data->device->dev,
			"debugfs create file for test_read failed\n");
		return -EFAULT;
	}
	return 0;
}

/*
 * Sequence of start up writes mandated by the paj9124u1 data sheet.
 */
static int paj9124u1_init_sequence(struct paj9124u1_drv_data *paj9124u1_data)
{
	uint8_t read_val;
	int i;

	/* Read the SensorPID to ensure the SPI Link is valid */
	if (paj9124u1_read(paj9124u1_data, &read_val, 0x00) == 0) {
		if (read_val != PAJ9124U1_MAGIC_PID) {
			dev_err(&paj9124u1_data->device->dev,
				"Couldn't read SPI Magic PID,"
				 "value read: %u\n", read_val);
			return -EIO;
		}
	}

	if (paj9124u1_write(paj9124u1_data, 0x00, 0x7F) != 0)
		return -EIO;

	if (paj9124u1_write_read(paj9124u1_data, 0x5A, 0x09) != 0)
		return -EIO;

	for (i = 0; i <= INIT_WRITES_FIRST_BATCH_INDEX; i++) {
		if (paj9124u1_write_read(paj9124u1_data, init_writes[i][1],
			init_writes[i][0]) != 0) {
			return -EIO;
		}
	}

	if (paj9124u1_write(paj9124u1_data, 0x01, 0x7F) != 0)
		return -EIO;

	for (; i <= INIT_WRITES_SECOND_BATCH_INDEX; i++) {
		if (paj9124u1_write_read(paj9124u1_data, init_writes[i][1],
			init_writes[i][0]) != 0) {
			return -EIO;
		}
	}

	if (paj9124u1_write(paj9124u1_data, 0x00, 0x7F) != 0)
		return -EIO;

	if (paj9124u1_write_read(paj9124u1_data, 0x00, 0x09) != 0)
		return -EIO;

	dev_info(&paj9124u1_data->device->dev, "paj9124u1 init success\n");
	return 0;
}

static irqreturn_t paj9124u1_handler(int irq, void *dev_id)
{
	static int8_t delta_x, delta_y, motion;
	struct paj9124u1_drv_data *paj9124u1_data = dev_id;

	do {
		if (paj9124u1_read(paj9124u1_data, &delta_x, PAJ9124U1_DELTA_X) != 0)
			break;
		if (paj9124u1_read(paj9124u1_data, &delta_y, PAJ9124U1_DELTA_Y) != 0)
			break;
		if (delta_x != 0) {
			input_report_rel(paj9124u1_data->in_dev, REL_WHEEL,
				delta_x);
			input_sync(paj9124u1_data->in_dev);
		}
		usleep_range(SAMPLING_PERIOD_US_MIN, SAMPLING_PERIOD_US_MAX);
		if (paj9124u1_read(paj9124u1_data, &motion, PAJ9124U1_MOTION) != 0)
			break;
	} while (motion & MOTION_BITMASK);

	return IRQ_HANDLED;
}

static int paj9124u1_setup_regulators(struct paj9124u1_drv_data *paj9124u1_data)
{
	struct spi_device *spi = paj9124u1_data->device;
	int ret;

	/* Initialize regulators */
	paj9124u1_data->vld_reg = devm_regulator_get(&spi->dev,
		"pixart,vld");
	if (IS_ERR(paj9124u1_data->vld_reg)) {
		dev_warn(&spi->dev,
			"regulator: VLD request failed\n");
		ret = (int)paj9124u1_data->vld_reg;
		paj9124u1_data->vld_reg = NULL;
		return ret;
	}

	paj9124u1_data->vdd_reg = devm_regulator_get(&spi->dev,
		"pixart,vdd");
	if (IS_ERR(paj9124u1_data->vdd_reg)) {
		dev_warn(&spi->dev,
			"regulator: VDD request failed\n");
		ret = (int)paj9124u1_data->vld_reg;
		paj9124u1_data->vdd_reg = NULL;
		return ret;
	}

	/* Turn on VDD */
	ret = regulator_enable(paj9124u1_data->vdd_reg);
	if (ret) {
		dev_err(&spi->dev, "couldn't enable regulator vdd\n");
		return ret;
	}

	msleep(PAJ9124U1_DELAY_MS_AFTER_VDD);

	/* Should the open of the SPI bus be done only once?? */
	paj9124u1_open(paj9124u1_data);

	ret = paj9124u1_init_sequence(paj9124u1_data);
	if (ret)
		goto error;

	/* Turn on VLD */
	ret = regulator_enable(paj9124u1_data->vld_reg);
	if (ret) {
		dev_err(&spi->dev, "couldn't enable regulator vld\n");
		goto error;
	}

	return 0;

error:
	regulator_disable(paj9124u1_data->vdd_reg);
	dev_err(&spi->dev, "paj9124u1 init failed\n");
	return ret;
}

static int paj9124u1_probe(struct spi_device *spi)
{
	struct paj9124u1_drv_data *paj9124u1_data;
	int err = 0;

	paj9124u1_data = devm_kzalloc(&spi->dev, sizeof(struct paj9124u1_drv_data),
		GFP_KERNEL);

	if (!paj9124u1_data)
		return -ENOMEM;

	spi_set_drvdata(spi, paj9124u1_data);
	paj9124u1_data->device = spi;

	err = paj9124u1_setup_regulators(paj9124u1_data);
	if (err)
		return err;

	/* Allocate and register an input device */
	paj9124u1_data->in_dev = devm_input_allocate_device(&spi->dev);
	if (!paj9124u1_data->in_dev) {
		dev_err(&spi->dev, "Couldn't allocate input device\n");
		return -ENOMEM;
	}

	paj9124u1_data->in_dev->evbit[0] = BIT_MASK(EV_REL);
	paj9124u1_data->in_dev->relbit[0] = BIT_MASK(REL_WHEEL);
	paj9124u1_data->in_dev->name = "paj9124u1";

	err = input_register_device(paj9124u1_data->in_dev);
	if (err) {
		dev_err(&spi->dev, "Failed to register paj9124u1 device\n");
		return err;
	}

	err = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
		paj9124u1_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING |
		IRQF_TRIGGER_LOW, "paj9124u1_handler", paj9124u1_data);
	if (err) {
		dev_err(&spi->dev,
			"Failed to register irq handler IRQ:%d\n",
			spi->irq);
		return err;
	}

	paj9124u1_create_debugfs(paj9124u1_data);
	return 0;
}

static int paj9124u1_remove(struct spi_device *spi)
{
	struct paj9124u1_drv_data *paj9124u1_data;

	paj9124u1_data = spi_get_drvdata(spi);

	debugfs_remove_recursive(paj9124u1_data->dent);

	regulator_disable(paj9124u1_data->vld_reg);
	regulator_disable(paj9124u1_data->vdd_reg);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int paj9124u1_suspend(struct device *device)
{
	struct spi_device *spidev = container_of(device,
		struct spi_device, dev);
	struct paj9124u1_drv_data *paj9124u1_data = spi_get_drvdata(spidev);
	int ret;

	ret = paj9124u1_write(paj9124u1_data, 0x8, PAJ9124U1_CONFIG);
	if (ret)
		dev_warn(device, "Failed to put paj9124u1 into low power.\n");
	disable_irq(spidev->irq);
	return 0;
}

static int paj9124u1_resume(struct device *device)
{
	struct spi_device *spidev = container_of(device,
		struct spi_device, dev);
	struct paj9124u1_drv_data *paj9124u1_data = spi_get_drvdata(spidev);
	int ret;

	enable_irq(spidev->irq);
	ret = paj9124u1_write(paj9124u1_data, 0x0, PAJ9124U1_CONFIG);
	if (ret)
		dev_warn(device,
			"Failed to take paj9124u1 out of low power.\n");
	return 0;
}
#endif

SIMPLE_DEV_PM_OPS(paj9124u1_pm_ops, paj9124u1_suspend, paj9124u1_resume);

static struct spi_driver paj9124u1_driver = {
	.driver = {
		.name = "paj9124u1",
		.owner = THIS_MODULE,
		.pm  = &paj9124u1_pm_ops,
	},
	.probe = paj9124u1_probe,
	.remove = paj9124u1_remove,
	.id_table = paj9124u1_spi_id,
};

int __init paj9124u1_init(void)
{
	return spi_register_driver(&paj9124u1_driver);
}

static void __exit paj9124u1_exit(void)
{
	spi_unregister_driver(&paj9124u1_driver);
}


module_init(paj9124u1_init);
module_exit(paj9124u1_exit);

MODULE_AUTHOR("Prashant Malani");
MODULE_LICENSE("GPL");
