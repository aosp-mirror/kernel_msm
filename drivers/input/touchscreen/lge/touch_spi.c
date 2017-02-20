/*
 * touch_spi.c
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
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
#define TS_MODULE "[spi]"

#include <linux/platform_device.h>
#include <linux/spi/spi.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>
#include <touch_spi.h>

int touch_spi_read(struct spi_device *spi, struct touch_bus_msg *msg)
{
	struct spi_transfer x = { 0, };
	struct spi_message m;

	if (msg->rx_size > MAX_BUF_SIZE || msg->tx_size > MAX_BUF_SIZE) {
		TOUCH_E("buffer overflow\n");
		return -1;
	}

	spi_message_init(&m);
	x.tx_buf = msg->tx_buf;
	x.rx_buf = msg->rx_buf;
	x.len = msg->rx_size;
	x.cs_change = 0;
	x.bits_per_word = spi->bits_per_word;
	spi_message_add_tail(&x, &m);
	return spi_sync(spi, &m);
}

int touch_spi_write(struct spi_device *spi, struct touch_bus_msg *msg)
{
	struct spi_transfer x = { 0, };
	struct spi_message m;

	if (msg->tx_size > MAX_BUF_SIZE) {
		TOUCH_E("buffer overflow\n");
		return -1;
	}

	spi_message_init(&m);
	x.tx_buf = msg->tx_buf;
	x.rx_buf = msg->rx_buf;
	x.len = msg->tx_size;
	x.cs_change = 0;
	x.bits_per_word = spi->bits_per_word;
	spi_message_add_tail(&x, &m);
	return spi_sync(spi, &m);
}

int touch_spi_xfer(struct spi_device *spi, struct touch_xfer_msg *xfer)
{
	struct touch_xfer_data_t *tx = NULL;
	struct touch_xfer_data_t *rx = NULL;
	struct spi_transfer x[MAX_XFER_COUNT];
	struct spi_message m;
	int i = 0;

	if (xfer->msg_count > MAX_XFER_COUNT) {
		TOUCH_E("count exceed\n");
		return -1;
	}

	spi_message_init(&m);
	memset(x, 0, sizeof(x));

	for (i = 0; i < xfer->msg_count; i++) {
		tx = &xfer->data[i].tx;
		rx = &xfer->data[i].rx;

		if (tx->size > MAX_XFER_BUF_SIZE ||
			rx->size > MAX_XFER_BUF_SIZE) {
			TOUCH_E("buffer overflow\n");
			return -1;
		}

		if (i < (xfer->msg_count) - 1)
			x[i].cs_change = 1;

		x[i].bits_per_word = spi->bits_per_word;

		if (rx->size) {
			x[i].tx_buf = tx->data;
			x[i].rx_buf = rx->data;
			x[i].len = rx->size;
		} else {
			x[i].tx_buf = tx->data;
			x[i].rx_buf = NULL;
			x[i].len = tx->size;
		}

		spi_message_add_tail(&x[i], &m);
	}

	return spi_sync(spi, &m);
}

static void touch_spi_release(struct device *dev)
{
	if (dev->platform_data)
		dev->platform_data = NULL;
}

struct touch_bus_info {
	struct spi_driver bus_driver;
	struct touch_hwif *hwif;
	struct touch_driver *touch_driver;
};

static int touch_spi_probe(struct spi_device *spi)
{
	struct touch_bus_info *info =
		container_of(to_spi_driver(spi->dev.driver),
			struct touch_bus_info, bus_driver);
	struct platform_device *pdev;
	struct touch_core_data *ts;
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("%s\n, platform ptr = %p\n",
			__func__, spi->dev.platform_data);

	if (!info) {
		TOUCH_E("Failed to get touch bus info\n");
		return -ENODEV;
	}

	ts = devm_kzalloc(&spi->dev, sizeof(*ts), GFP_KERNEL);

	if (!ts) {
		TOUCH_E("Failed to allocate memory for touch_core_data\n");
		return -ENOMEM;
	}

	ts->bus_type = info->hwif->bus_type;
	ts->dev = &spi->dev;
	ts->irq = spi->irq;
	ts->driver = info->touch_driver;
	dev_set_drvdata(&spi->dev, ts);

	spi->chip_select = 0;
	spi->bits_per_word = info->hwif->bits_per_word;
	spi->mode = info->hwif->spi_mode;
	spi->max_speed_hz = info->hwif->max_freq;
	TOUCH_I("%s : %d Mhz, chip_select = %d\n",
		__func__, spi->max_speed_hz/1000000, spi->chip_select);

	pdev = devm_kzalloc(&spi->dev, sizeof(*pdev), GFP_KERNEL);

	if (!pdev) {
		TOUCH_E("Failed to allocate memory for touch platform_devce\n");
		return -ENOMEM;
	}

	ts->pdev = pdev;

	pdev->name = LGE_TOUCH_DRIVER_NAME;
	pdev->id = 0;
	pdev->num_resources = 0;
	pdev->dev.parent = &spi->dev;
	pdev->dev.platform_data = ts;
	pdev->dev.release = touch_spi_release;

	TOUCH_I("platform device register\n");
	ret = platform_device_register(pdev);

	if (ret) {
		TOUCH_E("Failed to allocate memory for touch platform_devce\n");
		return -ENODEV;
	}
	TOUCH_I("platform device registered ...\n");

	return 0;
}

static int touch_spi_remove(struct spi_device *spi)
{
	TOUCH_TRACE();
	return 0;
}

static int touch_spi_pm_suspend(struct device *dev)
{
	struct touch_core_data *ts;

	TOUCH_TRACE();

	ts = (struct touch_core_data *) dev_get_drvdata(dev);
	atomic_set(&ts->state.pm, DEV_PM_SUSPEND);
	TOUCH_I("%s : DEV_PM_SUSPEND\n", __func__);

	return 0;
}

static int touch_spi_pm_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (atomic_read(&ts->state.pm) == DEV_PM_SUSPEND_IRQ) {
		atomic_set(&ts->state.pm, DEV_PM_RESUME);
		touch_set_irq_pending(ts->irq);
		touch_resend_irq(ts->irq);
		TOUCH_I("%s : DEV_PM_RESUME\n", __func__);
		return 0;
	}

	atomic_set(&ts->state.pm, DEV_PM_RESUME);
	TOUCH_I("%s : DEV_PM_RESUME\n", __func__);
	return 0;
}

static const struct dev_pm_ops touch_pm_ops = {
	.suspend = touch_spi_pm_suspend,
	.resume = touch_spi_pm_resume,
};

static struct spi_device_id touch_id[] = {
	{ LGE_TOUCH_NAME, 0 },
};

int touch_spi_device_init(struct touch_hwif *hwif, void *driver)
{
	struct touch_bus_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);

	if (!info) {
		TOUCH_E("faied to allocate spi_driver\n");
		return -ENOMEM;
	}

	hwif->info = info;

	info->bus_driver.driver.name = hwif->name;
	info->bus_driver.driver.owner = hwif->owner;
	info->bus_driver.driver.of_match_table = hwif->of_match_table;
	info->bus_driver.driver.pm = &touch_pm_ops;

	info->bus_driver.probe = touch_spi_probe;
	info->bus_driver.remove = touch_spi_remove;
	info->bus_driver.id_table = touch_id;

	info->hwif = hwif;
	info->touch_driver = driver;

	return spi_register_driver(&info->bus_driver);
}

void touch_spi_device_exit(struct touch_hwif *hwif)
{
	struct touch_bus_info *info = (struct touch_bus_info *) hwif->info;

	if (info) {
		kfree(info);
		hwif->info = NULL;
	}
}
