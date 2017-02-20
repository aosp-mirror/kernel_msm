/*
 * touch_i2c.c
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
#define TS_MODULE "[i2c]"

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>
#include <touch_i2c.h>

int touch_i2c_read(struct i2c_client *client, struct touch_bus_msg *msg)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = msg->tx_size,
			.buf = msg->tx_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = msg->rx_size,
			.buf = msg->rx_buf,
		},
	};
#ifdef USE_I2C_MTK_EXT
	{
		struct touch_core_data *ts =
			(struct touch_core_data *) i2c_get_clientdata(client);
		if (ts->tx_pa) {
			msgs[0].addr &= I2C_MASK_FLAG;
			msgs[0].addr |= I2C_ENEXT_FLAG;
			msgs[0].timing = 400;

			if (msgs[0].len >= 8) {
				msgs[0].addr |= I2C_DMA_FLAG;
				msgs[0].buf = (u8 *)ts->tx_pa;
			}
		}

		if (ts->rx_pa) {
			msgs[1].addr &= I2C_MASK_FLAG;
			msgs[1].addr |= I2C_ENEXT_FLAG;
			msgs[1].timing = 400;

			if (msgs[1].len >= 8) {
				msgs[1].addr |= I2C_DMA_FLAG;
				msgs[1].buf = (u8 *)ts->rx_pa;
			}
		}
	}
#endif

	ret = i2c_transfer(client->adapter, &msgs[0], 1);
	ret += i2c_transfer(client->adapter, &msgs[1], 1);

	if (ret == ARRAY_SIZE(msgs)) {
		return 0;
	} else if (ret < 0) {
		TOUCH_E("i2c_transfer - errno[%d]\n", ret);
	} else if (ret != ARRAY_SIZE(msgs)) {
        TOUCH_E("i2c_transfer - size[%d] result[%d]\n",
				(int) ARRAY_SIZE(msgs), ret);
	} else {
		TOUCH_E("unknown error [%d]\n", ret);
	}
	return ret;

}

int touch_i2c_write(struct i2c_client *client, struct touch_bus_msg *msg)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = msg->tx_size,
			.buf = msg->tx_buf,
		},
	};

#ifdef USE_I2C_MTK_EXT
	{
		struct touch_core_data *ts =
			(struct touch_core_data *) i2c_get_clientdata(client);
		if (ts->tx_pa) {
			msgs[0].addr &= I2C_MASK_FLAG;
			msgs[0].addr |= I2C_ENEXT_FLAG;
			msgs[0].timing = 400;

			if (msgs[0].len >= 8) {
				msgs[0].addr |= I2C_DMA_FLAG;
				msgs[0].buf = (u8 *)ts->tx_pa;
			}
		}
	}
#endif
	return i2c_transfer(client->adapter, msgs, 1);
}

static void touch_i2c_release(struct device *dev)
{
	if (dev->platform_data)
		dev->platform_data = NULL;
}

struct touch_bus_info {
	struct i2c_driver bus_driver;
	struct touch_hwif *hwif;
	struct touch_driver *touch_driver;
};

static int touch_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct touch_bus_info *info =
		container_of(to_i2c_driver(i2c->dev.driver),
			struct touch_bus_info, bus_driver);

	struct platform_device *pdev;
	struct touch_core_data *ts;
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("i2c slave address : %x\n", i2c->addr);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	TOUCH_I("%s\n, platform ptr = %p\n",
			__func__, i2c->dev.platform_data);

	if (!info) {
		TOUCH_E("info is NULL\n");
		return -ENOMEM;
	}

	ts = devm_kzalloc(&i2c->dev, sizeof(*ts), GFP_KERNEL);

	if (!ts) {
		TOUCH_E("Failed to allocate memory for touch_core_data\n");
		return -ENOMEM;
	}

	ts->bus_type = info->hwif->bus_type;
	ts->dev = &i2c->dev;
	ts->irq = i2c->irq;
	ts->driver = info->touch_driver;

	dev_set_drvdata(&i2c->dev, ts);

	pdev = devm_kzalloc(&i2c->dev, sizeof(*pdev), GFP_KERNEL);

	if (!pdev) {
		TOUCH_E("Failed to allocate memory for touch platform_devce\n");
		return -ENOMEM;
	}

	ts->pdev = pdev;

	pdev->name = LGE_TOUCH_DRIVER_NAME;
	pdev->id = 0;
	pdev->num_resources = 0;
	pdev->dev.parent = &i2c->dev;
	pdev->dev.platform_data = ts;
	pdev->dev.release = touch_i2c_release;

	TOUCH_I("platform device register\n");
	ret = platform_device_register(pdev);

	if (ret) {
		TOUCH_E("Failed to allocate memory for touch platform_devce\n");
		return -ENODEV;
	}
	TOUCH_I("platform device registered ...\n");

	return 0;
}

static int touch_i2c_remove(struct i2c_client *i2c)
{
	TOUCH_TRACE();
	return 0;
}

static int touch_i2c_pm_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	atomic_set(&ts->state.pm, DEV_PM_SUSPEND);
	TOUCH_I("%s : DEV_PM_SUSPEND\n", __func__);

	return 0;
}

static int touch_i2c_pm_resume(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	if (atomic_read(&ts->state.pm) == DEV_PM_SUSPEND_IRQ) {
		atomic_set(&ts->state.pm, DEV_PM_RESUME);
		TOUCH_I("%s : DEV_PM_RESUME0\n", __func__);
		touch_set_irq_pending(ts->irq);
		touch_resend_irq(ts->irq);
            return 0;
	}

	atomic_set(&ts->state.pm, DEV_PM_RESUME);
	TOUCH_I("%s : DEV_PM_RESUME\n", __func__);
	return 0;
}

static const struct dev_pm_ops touch_pm_ops = {
	.suspend = touch_i2c_pm_suspend,
	.resume = touch_i2c_pm_resume,
};

static struct i2c_device_id touch_id[] = {
	{ LGE_TOUCH_NAME, 0 },
};

int touch_i2c_device_init(struct touch_hwif *hwif, void *driver)
{
	struct touch_bus_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);

	if (!info) {
		TOUCH_E("faied to allocate i2c_driver\n");
		return -ENOMEM;
	}

	hwif->info = info;

	info->bus_driver.driver.name = hwif->name;
	info->bus_driver.driver.owner = hwif->owner;
	info->bus_driver.driver.of_match_table = hwif->of_match_table;
	info->bus_driver.driver.pm = &touch_pm_ops;

	info->bus_driver.probe = touch_i2c_probe;
	info->bus_driver.remove = touch_i2c_remove;
	info->bus_driver.id_table = touch_id;

	info->hwif = hwif;
	info->touch_driver = driver;

	return i2c_register_driver(info->hwif->owner, &info->bus_driver);
}

void touch_i2c_device_exit(struct touch_hwif *hwif)
{
	struct touch_bus_info *info = (struct touch_bus_info *) hwif->info;

	if (info) {
		kfree(info);
		hwif->info = NULL;
	}
}
