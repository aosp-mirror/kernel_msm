/*
 * Citadel transport driver
 *
 * Copyright (C) 2017 Fernando Lugo <flugo@google.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/citadel.h>

#define CITADEL_TPM_READ	0x80000000

#define MAX_DATA_SIZE		2044
#define CITADEL_MAX_DEVICES	4
#define CITADEL_TPM_TIMEOUT	5000

struct citadel_data {
	dev_t			devt;
	struct cdev		cdev;
	struct spi_device	*spi;
	atomic_t		users;
	void			*tx_buf;
	void			*rx_buf;
};

static struct class *citadel_class;
static dev_t citadel_devt;

static int citadel_wait_cmd_done(struct spi_device *spi)
{
	struct spi_message m;
	int ret;
	u8 val;
	unsigned long to = jiffies + msecs_to_jiffies(CITADEL_TPM_TIMEOUT);
	struct spi_transfer spi_xfer = {
		.rx_buf = &val,
		.len = 1,
		.cs_change = 1,
	};

	do {
		spi_message_init(&m);
		spi_message_add_tail(&spi_xfer, &m);
		ret = spi_sync_locked(spi, &m);
		if (ret)
			return ret;
		if (time_after(jiffies, to))
			return -EBUSY;
	} while (!(val & 0x01));

	return 0;
}

static int citadel_tpm_datagram(struct citadel_data *citadel,
			       struct citadel_ioc_tpm_datagram *dg)
{
	int is_read;
	int ret;
	int ignore_result = 0;
	struct spi_device *spi = citadel->spi;
	u32 command;
	struct spi_message m;
	struct spi_transfer spi_xfer = {
		.tx_buf = &command,
		.len = 4,
		.cs_change = 1,
	};

	/* Read == from SPI, to userland. */
	is_read = dg->command & CITADEL_TPM_READ;

	/* Lock the SPI bus until we're completely done */
	spi_bus_lock(spi->master);

	/* The command must be big-endian */
	command = cpu_to_be32(dg->command);

	/* Prepare to send the command */
	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);

	/* Send the command out */
	ret = spi_sync_locked(spi, &m);
	if (ret)
		goto exit;

	/* Wait for the reply bit to go high */
	ret = citadel_wait_cmd_done(spi);
	if (ret)
		goto exit;

	/* TODO: If there's no data, how do we disable the CS? */
	if (!dg->len) {
		/* For now, just transfer one more byte and throw it away */
		is_read = 1;
		dg->len = 1;
		ignore_result = 1;
	}

	/* Now we can transfer the data */
	spi_xfer.cs_change = 0;
	spi_xfer.len = dg->len;
	if (is_read) {
		spi_xfer.rx_buf = citadel->rx_buf;
		spi_xfer.tx_buf = NULL;
	} else {
		spi_xfer.rx_buf = NULL;
		spi_xfer.tx_buf = citadel->tx_buf;

		if (copy_from_user(citadel->tx_buf,
				   (void __user *)dg->buf, dg->len)) {
			ret = -EFAULT;
			goto exit;
		}
	}

	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);
	ret = spi_sync_locked(spi, &m);
	if (ret)
		goto exit;

	if (ignore_result)
		goto exit;

	if (is_read && copy_to_user((void __user *)dg->buf,
				    citadel->rx_buf, dg->len))
		ret = -EFAULT;

exit:
	spi_bus_unlock(spi->master);

	return ret;
}

static long
citadel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	u32 tmp;
	struct citadel_data *citadel = filp->private_data;
	struct citadel_ioc_tpm_datagram dg;

	/* check magic */
	if (_IOC_TYPE(cmd) != CITADEL_IOC_MAGIC)
		return -ENOTTY;
	switch (cmd) {
	case CITADEL_IOC_TPM_DATAGRAM:
		tmp = _IOC_SIZE(cmd);
		if (tmp != sizeof(dg))
			return -EINVAL;

		if (copy_from_user(&dg, (void __user *)arg, sizeof(dg)))
			return -EFAULT;

		if (dg.len > MAX_DATA_SIZE)
			return -E2BIG;

		/* translate to spi_message, execute */
		return citadel_tpm_datagram(citadel, &dg);
	}
	return -EINVAL;
}

static ssize_t
citadel_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int ret;
	size_t c = 0;
	size_t len;
	struct citadel_data *citadel = filp->private_data;

	while (count) {
		len = count > MAX_DATA_SIZE ? MAX_DATA_SIZE : count;
		ret = spi_read(citadel->spi, citadel->rx_buf, len);
		if (ret)
			return ret;
		ret = copy_to_user(buf + c, citadel->rx_buf, len);
		if (ret)
			return -EFAULT;
		c += len;
		count -= len;
	}

	return c;
}

static ssize_t
citadel_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	int ret;
	size_t c = 0;
	size_t len;
	struct citadel_data *citadel = filp->private_data;

	while (count) {
		len = count > MAX_DATA_SIZE ? MAX_DATA_SIZE : count;
		ret = copy_from_user(citadel->tx_buf, buf + c, len);
		if (ret)
			return -EFAULT;
		ret = spi_write(citadel->spi, citadel->tx_buf, len);
		if (ret)
			return ret;
		c += len;
		count -= len;
	}

	return c;
}

static int citadel_open(struct inode *inode, struct file *filp)
{
	struct citadel_data *citadel;

	citadel = container_of(inode->i_cdev, struct citadel_data, cdev);

	/* we only support 1 user at the same time */
	if (!atomic_add_unless(&citadel->users, 1, 1))
		return -EBUSY;

	filp->private_data = citadel;
	nonseekable_open(inode, filp);

	return 0;
}

static int citadel_release(struct inode *inode, struct file *filp)
{
	struct citadel_data *citadel = filp->private_data;

	atomic_dec(&citadel->users);

	return 0;
}

static const struct file_operations citadel_fops = {
	.owner =		THIS_MODULE,
	.write =		citadel_write,
	.read =			citadel_read,
	.open =			citadel_open,
	.release =		citadel_release,
	.unlocked_ioctl =	citadel_ioctl,
	.llseek =		no_llseek,
};

#ifdef CONFIG_OF
static const struct of_device_id citadel_dt_ids[] = {
	{ .compatible = "google,citadel" },
	{},
};
MODULE_DEVICE_TABLE(of, citadel_dt_ids);
#endif

static int citadel_probe(struct spi_device *spi)
{
	struct device *dev;
	struct cdev cdev;
	struct citadel_data *citadel;
	int ret;
	dev_t devt;
	u32 minor;

	/* use chip select as minor */
	minor = (u32)spi->chip_select;
	if (minor >= CITADEL_MAX_DEVICES) {
		dev_err(&spi->dev, "minor %u out of boundaries\n", minor);
		return -ENXIO;
	}

	citadel = kmalloc(sizeof(*citadel), GFP_KERNEL);
	if (!citadel)
		return -ENOMEM;

	/* init citadel structure */
	citadel->tx_buf = (void *)__get_free_pages(GFP_KERNEL,
						   get_order(MAX_DATA_SIZE));
	citadel->rx_buf = (void *)__get_free_pages(GFP_KERNEL,
						   get_order(MAX_DATA_SIZE));
	if (!citadel->tx_buf || !citadel->rx_buf) {
		ret = -ENOMEM;
		goto free_citadel;
	}
	atomic_set(&citadel->users, 0);
	citadel->spi = spi;
	devt = MKDEV(MAJOR(citadel_devt), minor);
	citadel->devt = devt;
	spi_set_drvdata(spi, citadel);

	dev = device_create(citadel_class, &spi->dev, devt, NULL,
			    "citadel%u", minor);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto free_citadel;
	}

	cdev_init(&citadel->cdev, &citadel_fops);
	cdev.owner = THIS_MODULE;
	ret = cdev_add(&citadel->cdev, citadel_devt, 1);
	if (ret)
		goto destroy_device;

	return 0;

destroy_device:
	device_destroy(citadel_class, devt);
free_citadel:
	free_pages((unsigned long)citadel->rx_buf, get_order(MAX_DATA_SIZE));
	free_pages((unsigned long)citadel->tx_buf, get_order(MAX_DATA_SIZE));
	kfree(citadel);
	return ret;
}

static int citadel_remove(struct spi_device *spi)
{
	struct citadel_data *citadel = spi_get_drvdata(spi);

	cdev_del(&citadel->cdev);
	device_destroy(citadel_class, citadel->devt);
	free_pages((unsigned long)citadel->rx_buf, get_order(MAX_DATA_SIZE));
	free_pages((unsigned long)citadel->tx_buf, get_order(MAX_DATA_SIZE));
	kfree(citadel);

	return 0;
}

static struct spi_driver citadel_spi_driver = {
	.driver = {
		.name =		"citadel",
		.of_match_table = of_match_ptr(citadel_dt_ids),
	},
	.probe =	citadel_probe,
	.remove =	citadel_remove,
};

static int __init citadel_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&citadel_devt, 0, CITADEL_MAX_DEVICES,
				  "citadel");
	if (ret) {
		pr_err("%s: failed to alloc cdev region %d\n", __func__, ret);
		return ret;
	}

	citadel_class = class_create(THIS_MODULE, "citadel");
	if (IS_ERR(citadel_class)) {
		unregister_chrdev_region(citadel_devt, 1);
		return PTR_ERR(citadel_class);
	}

	ret = spi_register_driver(&citadel_spi_driver);
	if (ret < 0) {
		class_destroy(citadel_class);
		unregister_chrdev_region(citadel_devt, 1);
	}
	return ret;
}

static void __exit citadel_exit(void)
{
	spi_unregister_driver(&citadel_spi_driver);
	class_destroy(citadel_class);
	unregister_chrdev_region(citadel_devt, 1);
}

module_init(citadel_init);
module_exit(citadel_exit);

MODULE_AUTHOR("Fernando Lugo, <flugo@google.com>");
MODULE_DESCRIPTION("CITADEL TPM driver");
MODULE_LICENSE("GPL");
