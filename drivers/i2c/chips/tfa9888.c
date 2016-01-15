/* driver/i2c/chip/tfa9888.c
 *
 * NXP tfa9888 Speaker Amp
 *
 * Copyright (C) 2015 HTC Corporation
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include "tfa9888.h"
#include <linux/debugfs.h>
#include <linux/module.h>
#include <sound/htc_acoustic_alsa.h>

#undef pr_info
#undef pr_err
#define pr_aud_fmt(fmt) "[AUD] " KBUILD_MODNAME ": " fmt
#define pr_info(fmt, ...) printk(KERN_INFO pr_aud_fmt(fmt), ##__VA_ARGS__)
#define pr_err(fmt, ...) printk(KERN_ERR pr_aud_fmt(fmt), ##__VA_ARGS__)

static struct i2c_client *this_client;

static int tfa_i2c_write(char *txdata, int length)
{
	int rc;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	rc = i2c_transfer(this_client->adapter, msg, 1);
	if (rc < 0)
		pr_err("%s: transfer error %d\n", __func__, rc);

	return rc;

}

static ssize_t tfa9888_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
	int rc = 0;
	char wbuf[255];

	if (length > sizeof(wbuf)) {
		pr_err("%s: copy size out of memory: %zu\n", __func__, length);
		return -EINVAL;
	}

	if (copy_from_user(wbuf, buffer, length)) {
		pr_err("%s: copy buffer fail from user space %d\n", __func__, rc);
		return -EFAULT;
	}
	rc = tfa_i2c_write(wbuf, length);
	if (rc < 0)
		pr_err("%s: tfa write op i2c error %d\n", __func__, rc);
	else
		rc = length;

	return rc;
}

static int tfa_i2c_read(char *rxdata, int length)
{
	int rc = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxdata,
		},
	};

	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if (rc < 0)
		pr_err("%s: transfer i2c error %d\n", __func__, rc);
	else
		rc = length;

	return rc;
}

static ssize_t tfa9888_read(struct file *file, char __user *buffer, size_t length, loff_t *offset)
{
	int rc = 0;
	char rbuf[255];

	if (length > sizeof(rbuf)) {
		pr_err("%s: copy size out of memory: %zu\n", __func__, length);
		return -EINVAL;
	}

	rc = tfa_i2c_read(rbuf, length);
	if (rc < 0) {
		pr_err("%s: tfa read op i2c fail %d\n", __func__, rc);
		return rc;
	}
	if (copy_to_user(buffer, rbuf, length)) {
		pr_err("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return rc;
}

static void tfa9888_version(unsigned char* ver) {
	unsigned char get_ver[1] = {0x03};

	tfa_i2c_write(get_ver, 1);
	tfa_i2c_read(ver, 2);
}

static int tfa9888_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int tfa9888_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations tfa9888_fops = {
	.owner = THIS_MODULE,
	.read = tfa9888_read,
	.write = tfa9888_write,
	.open = tfa9888_open,
	.release = tfa9888_release,
};

static struct miscdevice tfa9888_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tfa9888",
	.fops = &tfa9888_fops,
};

int tfa9888_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	this_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	ret = misc_register(&tfa9888_device);
	if (ret) {
		pr_err("%s: tfa9888_device register failed\n", __func__);
		goto err;
	}

	htc_acoustic_register_spk_version(tfa9888_version);

	return 0;

err:
	return ret;
}

static int tfa9888_remove(struct i2c_client *client)
{
	return 0;
}

static int tfa9888_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int tfa9888_resume(struct i2c_client *client)
{
	return 0;
}

static struct of_device_id tfa9888_match_table[] = {
	{ .compatible = "nxp,tfa9888-amp",},
	{ },
};

static const struct i2c_device_id tfa9888_id[] = {
	{ TFA9888_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tfa9888_driver = {
	.probe = tfa9888_probe,
	.remove = tfa9888_remove,
	.suspend = tfa9888_suspend,
	.resume = tfa9888_resume,
	.id_table = tfa9888_id,
	.driver = {
		.name = TFA9888_I2C_NAME,
		.of_match_table = tfa9888_match_table,
	},
};

static int __init tfa9888_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&tfa9888_driver);
}

static void __exit tfa9888_exit(void)
{
	i2c_del_driver(&tfa9888_driver);
}

module_init(tfa9888_init);
module_exit(tfa9888_exit);

MODULE_DESCRIPTION("tfa9888 Speaker Amp driver");
MODULE_LICENSE("GPL");
