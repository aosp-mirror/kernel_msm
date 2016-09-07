/*
 * Copyright (C) 2010 NXP Semiconductors
 * Copyright (C) 2016 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c/pn548.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

struct pn548_dev {
	struct i2c_client       *client;
	struct device           *dev;
	wait_queue_head_t       read_wq;
	struct mutex            read_mutex;
	struct miscdevice       pn548_device;
	unsigned int            ven_gpio;
	unsigned int            firm_gpio;
	unsigned int            irq_gpio;
	struct clk              *clk_cont;
	struct clk              *clk_pin;
	bool                    irq_enabled;
	spinlock_t              irq_enabled_lock;
	bool                    powered;
};

#define MAX_BUFFER_SIZE     512
#define NFC_TIMEOUT_MS      2000

static struct i2c_client *pn548_client;

static void pn548_disable_irq(struct pn548_dev *pn548_dev)
{
	spin_lock(&pn548_dev->irq_enabled_lock);
	if (pn548_dev->irq_enabled) {
		disable_irq(pn548_dev->client->irq);
		disable_irq_wake(pn548_dev->client->irq);
		pn548_dev->irq_enabled = false;
		dev_dbg(pn548_dev->dev, "%s disable IRQ\n", __func__);
	} else {
		dev_dbg(pn548_dev->dev, "%s IRQ is already disabled!\n",
				__func__);
	}
	spin_unlock(&pn548_dev->irq_enabled_lock);
}

static void pn548_enable_irq(struct pn548_dev *pn548_dev)
{
	spin_lock(&pn548_dev->irq_enabled_lock);
	if (!pn548_dev->irq_enabled) {
		pn548_dev->irq_enabled = true;
		enable_irq(pn548_dev->client->irq);
		enable_irq_wake(pn548_dev->client->irq);
		dev_dbg(pn548_dev->dev, "%s enable IRQ\n", __func__);
	} else {
		dev_dbg(pn548_dev->dev, "%s IRQ is already enabled!\n",
				__func__);
	}
	spin_unlock(&pn548_dev->irq_enabled_lock);
}

static irqreturn_t pn548_dev_irq_handler(int irq, void *dev_id)
{
	struct pn548_dev *pn548_dev = dev_id;
	unsigned int val;

	val = gpio_get_value(pn548_dev->irq_gpio);
	if (val == 0 || !pn548_dev->powered) {
		dev_warn(pn548_dev->dev, "%s: False Interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	/* Wake up waiting readers */
	wake_up(&pn548_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn548_dev_read(struct file *filp, char __user *buf,
					size_t count, loff_t *offset)
{
	struct pn548_dev *pn548_dev = filp->private_data;
	static char tmp[MAX_BUFFER_SIZE];
	int ret;
	static bool isFirstPacket = true;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (isFirstPacket == false) {
		ret = wait_event_interruptible_timeout(pn548_dev->read_wq,
				gpio_get_value(pn548_dev->irq_gpio),
				msecs_to_jiffies(NFC_TIMEOUT_MS));
		if (!ret) {
			dev_dbg(pn548_dev->dev,
				"%s: no more interrupt after %dms (%d)!\n",
			       __func__, NFC_TIMEOUT_MS,
			       gpio_get_value(pn548_dev->irq_gpio));
			isFirstPacket = true;
		}
	}

	if (isFirstPacket == true) {
		ret = wait_event_interruptible(pn548_dev->read_wq,
					gpio_get_value(pn548_dev->irq_gpio));
		if (!ret)
			isFirstPacket = false;
	}

	if (ret == -ERESTARTSYS)
		return ret;

	if (!pn548_dev->powered)
		return -EIO;

	/* Read data */
	if (count > MAX_BUFFER_SIZE) {
		dev_err(pn548_dev->dev,
			"%s: requested too big bytes(max:%d, count:%d)\n",
			__func__, MAX_BUFFER_SIZE, count);
		return -EIO;
	}

	if (!count) {
		dev_err(pn548_dev->dev, "%s: requested 0 byte read\n",
				__func__);
		return -EIO;
	}

	mutex_lock(&pn548_dev->read_mutex);
	memset(tmp, 0x00, MAX_BUFFER_SIZE);
	ret = i2c_master_recv(pn548_dev->client, tmp, count);
	mutex_unlock(&pn548_dev->read_mutex);
	if (ret < 0) {
		dev_err(pn548_dev->dev, "%s: i2c_master_recv returned %d\n",
				__func__, ret);
		return ret;
	}
	if (copy_to_user(buf, tmp, ret)) {
		dev_warn(pn548_dev->dev,
			"%s: failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return ret;
}

static ssize_t pn548_dev_write(struct file *filp, const char __user *buf,
						size_t count, loff_t *offset)
{
	struct pn548_dev *pn548_dev = filp->private_data;
	static char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (!pn548_dev->powered)
		return -EIO;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(tmp, 0x00, MAX_BUFFER_SIZE);
	if (copy_from_user(tmp, buf, count)) {
		dev_err(pn548_dev->dev,
				"%s: failed to copy from user space\n",
				__func__);
		return -EFAULT;
	}

	mutex_lock(&pn548_dev->read_mutex);
	ret = i2c_master_send(pn548_dev->client, tmp, count);
	mutex_unlock(&pn548_dev->read_mutex);
	if (ret != count) {
		dev_err(pn548_dev->dev, "%s: i2c_master_send returned %d\n",
				__func__, ret);
		ret = -EIO;
	}

	return ret;
}

static int pn548_dev_open(struct inode *inode, struct file *filp)
{
	struct pn548_dev *pn548_dev = i2c_get_clientdata(pn548_client);

	filp->private_data = pn548_dev;
	dev_dbg(pn548_dev->dev, "%s: %d,%d\n",
			__func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn548_dev_unlocked_ioctl(struct file *filp,
				unsigned int cmd, unsigned long arg)
{
	struct pn548_dev *pn548_dev = filp->private_data;

	switch (cmd) {
	case pn548_SET_PWR:
		if (arg == 2) {
			/*  power on with firmware download */
			dev_dbg(pn548_dev->dev,
				"%s: power on with firmware\n",
				__func__);
			gpio_set_value(pn548_dev->ven_gpio, 1);
			gpio_set_value(pn548_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn548_dev->ven_gpio, 0);
			msleep(10);
			gpio_set_value(pn548_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			dev_dbg(pn548_dev->dev, "%s: power on\n", __func__);
			if (!pn548_dev->powered) {
				gpio_set_value(pn548_dev->firm_gpio, 0);
				gpio_set_value(pn548_dev->ven_gpio, 1);
				msleep(10);

				pn548_dev->powered = true;
				pn548_enable_irq(pn548_dev);
			} else {
				dev_warn(pn548_dev->dev,
					"%s: NFC is already On!\n", __func__);
			}
		} else  if (arg == 0) {
			/* power off */
			dev_dbg(pn548_dev->dev, "%s: power off\n", __func__);
			if (pn548_dev->powered) {
				pn548_dev->powered = false;
				gpio_set_value(pn548_dev->firm_gpio, 0);
				gpio_set_value(pn548_dev->ven_gpio, 0);
				msleep(10);

				pn548_disable_irq(pn548_dev);
			} else {
				dev_warn(pn548_dev->dev,
					"%s: NFC is already Off!\n", __func__);
			}
		} else {
			dev_err(pn548_dev->dev, "%s: bad arg %ld\n",
					__func__, arg);
			return -EINVAL;
		}
		break;

	case pn548_HW_REVISION:
		return 0;

	default:
		dev_err(pn548_dev->dev, "%s bad ioctl %d\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static int pn548_parse_dt(struct device *dev, struct pn548_dev *pn548_dev)
{
	struct device_node *np = dev->of_node;

	pn548_dev->ven_gpio = of_get_named_gpio_flags(np,
					"nxp,gpio_ven", 0, NULL);
	if (pn548_dev->ven_gpio < 0) {
		dev_err(dev, "couldn't read nxp,gpio_ven from dt\n");
		return pn548_dev->ven_gpio;
	}

	pn548_dev->firm_gpio = of_get_named_gpio_flags(np,
					"nxp,gpio_mode", 0, NULL);
	if (pn548_dev->firm_gpio < 0) {
		dev_err(dev, "couldn't read nxp,gpio_mode from dt\n");
		return pn548_dev->firm_gpio;
	}

	pn548_dev->irq_gpio = of_get_named_gpio_flags(np,
					"nxp,gpio_irq", 0, NULL);
	if (pn548_dev->irq_gpio < 0) {
		dev_err(dev, "couldn't read nxp,irq_gpio from dt\n");
		return pn548_dev->irq_gpio;
	}

	return 0;
}

static const struct file_operations pn548_dev_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read   = pn548_dev_read,
	.write  = pn548_dev_write,
	.open   = pn548_dev_open,
	.unlocked_ioctl = pn548_dev_unlocked_ioctl,
	.compat_ioctl   = pn548_dev_unlocked_ioctl,
};

static int pn548_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	struct pn548_dev *pn548_dev = NULL;

	pn548_client = client;
	pn548_dev = devm_kzalloc(&client->dev, sizeof(*pn548_dev), GFP_KERNEL);
	if (!pn548_dev) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	pn548_dev->client = client;
	pn548_dev->dev = &client->dev;
	i2c_set_clientdata(client, pn548_dev);

	ret = pn548_parse_dt(&client->dev, pn548_dev);
	if (ret) {
		dev_err(&client->dev, "failed to parse device tree\n");
		return ret;
	}

	ret = devm_gpio_request_one(&client->dev, pn548_dev->irq_gpio,
			GPIOF_IN,  "nfc_int");
	if (ret) {
		dev_err(&client->dev, "%s: nfc_int request failed!\n",
				__func__);
		return ret;
	}

	ret = devm_gpio_request_one(&client->dev, pn548_dev->ven_gpio,
			GPIOF_OUT_INIT_LOW, "nfc_ven");
	if (ret) {
		dev_err(&client->dev, "%s: nfc_ven request failed!\n",
				__func__);
		return ret;
	}

	ret = devm_gpio_request_one(&client->dev, pn548_dev->firm_gpio,
			GPIOF_OUT_INIT_LOW, "nfc_firm");
	if (ret) {
		dev_err(&client->dev, "%s: nfc_firm request failed!\n",
				__func__);
		return ret;
	}

	/* init mutex and queues */
	init_waitqueue_head(&pn548_dev->read_wq);
	mutex_init(&pn548_dev->read_mutex);
	spin_lock_init(&pn548_dev->irq_enabled_lock);

	pn548_dev->pn548_device.minor = MISC_DYNAMIC_MINOR;
	pn548_dev->pn548_device.name = PN548_DRV_NAME;
	pn548_dev->pn548_device.fops = &pn548_dev_fops;

	ret = misc_register(&pn548_dev->pn548_device);
	if (ret) {
		dev_err(&client->dev, "%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}

	/* request irq. the irq is set whenever the chip has data available
	 * for reading. it is cleared when all data has been read.
	 */
	ret = devm_request_irq(&client->dev, client->irq,
			pn548_dev_irq_handler,
			IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND,
			client->name, pn548_dev);
	if (ret) {
		dev_err(&client->dev, "%s: request_irq failed\n", __func__);
		goto err_request_irq_failed;
	}

	/*
	* Below code does not pack with pn548_enable_irq
	* request irq make irq enable.
	* so if irq enable in pn548_enable_irq called,
	* unbalanced enable log will be appeared.
	*/
	pn548_dev->irq_enabled = true;
	enable_irq_wake(client->irq);
	pn548_disable_irq(pn548_dev);

	dev_info(&client->dev, "PN548 probed\n");
	return 0;

err_request_irq_failed:
	misc_deregister(&pn548_dev->pn548_device);

err_misc_register:
	mutex_destroy(&pn548_dev->read_mutex);

	return ret;
}

static int pn548_remove(struct i2c_client *client)
{
	struct pn548_dev *pn548_dev = i2c_get_clientdata(client);

	misc_deregister(&pn548_dev->pn548_device);
	mutex_destroy(&pn548_dev->read_mutex);
	pn548_client = NULL;

	return 0;
}

static const struct i2c_device_id pn548_id[] = {
	{ PN548_DRV_NAME, 0 },
	{ }
};

static struct of_device_id pn548_match_table[] = {
	{ .compatible = "nxp,pn548",},
	{ },
};

static struct i2c_driver pn548_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = PN548_DRV_NAME,
		.of_match_table = pn548_match_table,
	},
	.probe      = pn548_probe,
	.remove     = pn548_remove,
	.id_table   = pn548_id,
};

static int __init pn548_dev_init(void)
{
	int ret;

	ret = i2c_add_driver(&pn548_driver);
	if (ret < 0)
		pr_err("%s: failed to i2c_add_driver\n", __func__);

	return ret;
}
module_init(pn548_dev_init);

static void __exit pn548_dev_exit(void)
{
	i2c_del_driver(&pn548_driver);
}
module_exit(pn548_dev_exit);

MODULE_DEVICE_TABLE(i2c, pn548_id);
MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN548 driver");
MODULE_LICENSE("GPL");
