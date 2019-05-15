/* drivers/nfc/ese/pn81a.c
 *
 * Copyright 2017 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

/* Relies on struct nfcc_data and nqx_ese_pwr().
 * This can easily be made generic is a function is exported
 * that happily takes a void *.
 */
#include "../nq-nci.h"

#define PN81A_MAX_BUF 258U

struct ese_dev {
	struct spi_device	*spi;
	struct device		*nfcc_device;
	struct nqx_dev	*nfcc_data;
	struct mutex		mutex;
	struct	miscdevice	device;
	int			gpio_clear_n;
	const char		*nfcc_name;
	char *kbuf;
};

static long ese_clear_gpio(struct ese_dev *ese_dev, unsigned long arg)
{
	long r = 0;
	int val = arg ? 1 : 0;
	if (ese_dev->gpio_clear_n == -EINVAL) {
		r = -ENODEV;
	} else {
		gpio_set_value(ese_dev->gpio_clear_n, val);
		usleep_range(1000, 1100);
		if (gpio_get_value(ese_dev->gpio_clear_n) != val) {
			dev_err(&ese_dev->spi->dev,
				"gpio_clear_n won't change to %d\n", val);
			r = -EBUSY;
		} else {
			dev_info(&ese_dev->spi->dev,
					"%s: gpio_clear_n changed: %d\n",
					__func__, val);
		}
	}
	return r;
}

#ifdef CONFIG_COMPAT
static long ese_compat_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	long r = 0;
	struct ese_dev *ese_dev = filp->private_data;
	mutex_lock(&ese_dev->mutex);

	arg = (compat_u64)arg;
	switch (cmd) {
	case ESE_SET_PWR:
		nqx_ese_pwr(ese_dev->nfcc_data, arg);
		break;
	case ESE_GET_PWR:
		nqx_ese_pwr(ese_dev->nfcc_data, 3);
		break;
	case ESE_CLEAR_GPIO:
		r = ese_clear_gpio(ese_dev, arg);
		break;
	default:
		r = -ENOTTY;
	}
	mutex_unlock(&ese_dev->mutex);
	return r;
}
#endif

static long ese_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int r = 0;
	struct ese_dev *ese_dev = filp->private_data;
	mutex_lock(&ese_dev->mutex);

	switch (cmd) {
	case ESE_SET_PWR:
		r = nqx_ese_pwr(ese_dev->nfcc_data, arg);
		break;
	case ESE_GET_PWR:
		r = nqx_ese_pwr(ese_dev->nfcc_data, 3);
		break;
	case ESE_CLEAR_GPIO:
		r = ese_clear_gpio(ese_dev, arg);
		break;
	default:
		r = -ENOIOCTLCMD;
	}
	mutex_unlock(&ese_dev->mutex);
	return r;
}

static int ese_open(struct inode *inode, struct file *filp)
{
	int pwr = 0;
	struct ese_dev *ese_dev = container_of(filp->private_data,
				struct ese_dev, device);

	mutex_lock(&ese_dev->mutex);
	/* Find the NFC parent device if it exists. */
	if (ese_dev->nfcc_data == NULL) {
		struct device *nfc_dev = bus_find_device_by_name(
					&i2c_bus_type,
					NULL,
					ese_dev->nfcc_name);
		if (!nfc_dev) {
			dev_err(&ese_dev->spi->dev,
				"%s: cannot find NFC controller '%s'\n",
				__func__, ese_dev->nfcc_name);
			goto err;
		}
		ese_dev->nfcc_data = dev_get_drvdata(nfc_dev);
		if (!ese_dev->nfcc_data) {
			dev_err(&ese_dev->spi->dev,
				"%s: cannot find NFC controller device data\n",
				__func__);
			put_device(nfc_dev);
			goto err;
		}
		dev_info(&ese_dev->spi->dev,
				"%s: NFC controller found\n", __func__);
		ese_dev->nfcc_device = nfc_dev;
	}
	mutex_unlock(&ese_dev->mutex);

	if (nqx_claim_ese(ese_dev->nfcc_data, true) != 0)
		return -EBUSY;
	filp->private_data = ese_dev;
	dev_dbg(&ese_dev->spi->dev,
			"%s: major,minor: %d,%d\n",
			__func__, imajor(inode), iminor(inode));

	/* Note, opening and closing is treated wholly independently
	 * from power management.  This ensures that the eSE can go to
	 * a deep power down state as dictated by the software stack.
	 */
	pwr = ese_ioctl(filp, ESE_GET_PWR, 0);
	dev_dbg(&ese_dev->spi->dev,
			"%s: eSE opened (power: %d)\n", __func__, pwr);

	return 0;

err:
	mutex_unlock(&ese_dev->mutex);
	return -ENODEV;
}

static int ese_release(struct inode *ino, struct file *filp)
{
	struct ese_dev *ese_dev = filp->private_data;
	int pwr = 0;

	nqx_claim_ese(ese_dev->nfcc_data, false);

	pwr = ese_ioctl(filp, ESE_GET_PWR, 0);

	mutex_lock(&ese_dev->mutex);
	dev_dbg(&ese_dev->spi->dev,
			"%s: power: %d\n", __func__, pwr);
	mutex_unlock(&ese_dev->mutex);
	return 0;
}

static ssize_t ese_write(struct file *filp, const char __user *ubuf,
				size_t len, loff_t *offset)
{
	struct ese_dev *ese_dev = filp->private_data;
	ssize_t ret = -EFAULT;
	size_t bytes = len;
	char *tx_buf = NULL;

	if (len > INT_MAX)
		return -EINVAL;

	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < PN81A_MAX_BUF ? bytes : PN81A_MAX_BUF;

		tx_buf = memdup_user(ubuf, block);
		if (IS_ERR(tx_buf)) {
			dev_dbg(&ese_dev->spi->dev, "memdup_user failed\n");
			ret = PTR_ERR(tx_buf);
			goto err;
		}

		ret = spi_write(ese_dev->spi, tx_buf, block);
		if (ret < 0) {
			dev_dbg(&ese_dev->spi->dev, "failed to write to SPI\n");
			goto err;
		}
		kfree(tx_buf);
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	kfree(tx_buf);
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static ssize_t ese_read(struct file *filp, char __user *ubuf,
				size_t len, loff_t *offset)
{
	struct ese_dev *ese_dev = filp->private_data;
	ssize_t ret = -EFAULT;
	size_t bytes = len;
	char *rx_buf = NULL;

	if (len > INT_MAX)
		return -EINVAL;

	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < PN81A_MAX_BUF ? bytes : PN81A_MAX_BUF;

		rx_buf = ese_dev->kbuf;
		if (!rx_buf) {
			dev_dbg(&ese_dev->spi->dev, "rx_buf does not exist anymore\n");
			ret = -ENOMEM;
			goto err;
		}

		memset(rx_buf, 0, PN81A_MAX_BUF);
		ret = spi_read(ese_dev->spi, rx_buf, block);
		if (ret < 0) {
			dev_dbg(&ese_dev->spi->dev, "failed to read from SPI\n");
			goto err;
		}
		if (copy_to_user(ubuf, rx_buf, block)) {
			dev_dbg(&ese_dev->spi->dev, "failed to copy from user\n");
			goto err;
		}
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static const struct file_operations ese_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read  = ese_read,
	.write = ese_write,
	.open = ese_open,
	.release = ese_release,
	.unlocked_ioctl = ese_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ese_compat_ioctl
#endif
};

static int pn81a_probe(struct spi_device *spi)
{
	struct ese_dev *ese_dev;
	struct device_node *np = dev_of_node(&spi->dev);
	int ret;
	int gpio;

	dev_dbg(&spi->dev, "%s: called\n", __func__);

	if (!np) {
		dev_err(&spi->dev, "%s: device tree data missing\n", __func__);
		return -EINVAL;
	}

	ese_dev = kzalloc(sizeof(*ese_dev), GFP_KERNEL);
	if (ese_dev == NULL)
		return -ENOMEM;

	ese_dev->kbuf = kzalloc(PN81A_MAX_BUF, GFP_KERNEL);
	if (ese_dev->kbuf == NULL)
		return -ENOMEM;

	ese_dev->spi = spi;
	ese_dev->device.minor = MISC_DYNAMIC_MINOR;
	ese_dev->device.name = "pn81a";
	ese_dev->device.fops = &ese_dev_fops;

	spi->bits_per_word = 8;

	gpio = of_get_named_gpio(np, "nxp,clear-n", 0);
	if (!gpio_is_valid(gpio)) {
		dev_warn(&spi->dev,
			"%s: nxp,clear-n invalid or missing in device tree\n",
			__func__);
		gpio = -EINVAL;
	}
	ese_dev->gpio_clear_n = gpio;
	if (ese_dev->gpio_clear_n != -EINVAL) {
		ret = gpio_request(ese_dev->gpio_clear_n, "ese_clear_n");
		if (ret) {
			dev_warn(&spi->dev,
				  "%s: unable to request ese clear n gpio [%d]\n",
				  __func__,
				  gpio);
			ese_dev->gpio_clear_n = -EINVAL;
			goto skip_gpio;
		}
		ret = gpio_direction_output(ese_dev->gpio_clear_n, 1);
		if (ret) {
			dev_warn(&spi->dev,
				"%s: failed to set direction for ese clear n[%d]\n",
				__func__,
				ese_dev->gpio_clear_n);
		}
	}

skip_gpio:
	mutex_init(&ese_dev->mutex);
	ret = of_property_read_string(np, "nxp,nfcc", &ese_dev->nfcc_name);
	if (ret < 0) {
		dev_err(&spi->dev,
			"%s: nxp,nfcc invalid or missing in device tree (%d)\n",
			__func__, ret);
		goto err;
	}
	dev_info(&spi->dev, "%s: device tree set '%s' as eSE power controller\n",
		__func__, ese_dev->nfcc_name);

	ret = misc_register(&ese_dev->device);
	if (ret) {
		dev_err(&spi->dev, "%s: misc_register failed\n", __func__);
		goto err;
	}
	dev_info(&spi->dev, "%s: eSE is configured\n", __func__);
	spi_set_drvdata(spi, ese_dev);

	return 0;
err:
	mutex_destroy(&ese_dev->mutex);
	kfree(ese_dev->kbuf);
	kfree(ese_dev);
	return ret;
}

static int pn81a_remove(struct spi_device *spi)
{
	struct ese_dev *ese_dev = spi_get_drvdata(spi);
	int ret = 0;

	if (!ese_dev) {
		dev_err(&spi->dev,
		"%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto err;
	}
	/* If we have a NFC device, release it. */
	if (ese_dev->nfcc_device) {
		put_device(ese_dev->nfcc_device);
		ese_dev->nfcc_data = NULL;
		ese_dev->nfcc_device = NULL;
	}
	misc_deregister(&ese_dev->device);
	mutex_destroy(&ese_dev->mutex);
	if (ese_dev->gpio_clear_n != -EINVAL)
		gpio_free(ese_dev->gpio_clear_n);
	kfree(ese_dev->kbuf);
	kfree(ese_dev);
err:
	return ret;
}

static const struct of_device_id pn81a_match_table[] = {
	{ .compatible = "nxp,pn81a" },
	{ }
};
MODULE_DEVICE_TABLE(of, pn81a_match_table);

static struct spi_driver pn81a_driver = {
	.driver = {
		.name = "pn81a",
		.of_match_table = pn81a_match_table,
	},
	.probe = pn81a_probe,
	.remove = pn81a_remove,
};
module_spi_driver(pn81a_driver);

MODULE_DESCRIPTION("PN81A eSE driver");
MODULE_ALIAS("spi:pn81a");
MODULE_AUTHOR("Google Inc");
MODULE_LICENSE("GPL");
