/******************************************************************************
 * Copyright (C) 2015 Broadcom Corporation
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/of.h>

#define PORT_NAME "/dev/ttyBCM"


#ifdef CONFIG_SENSORS_SSP_BBD
extern void bbd_parse_asic_data(unsigned char *pucData, unsigned short usLen,
		void (*to_gpsd)(unsigned char *packet, unsigned short len, void *priv), void *priv);
#endif

struct bcm_tty_priv {
	struct file *tty;
	struct miscdevice misc;
	int mcu_req;
	int mcu_resp;
	int host_req;
};

static int bcm_tty_config(struct file *f)
{
	struct termios termios;
	mm_segment_t fs;
	long ret;

	/* Change address limit */
	fs = get_fs();
	set_fs(KERNEL_DS);

	/* Get termios */
	ret = f->f_op->unlocked_ioctl(f, TCGETS, (unsigned long)&termios);
	if (ret) {
		pr_err("%s TCGETS failed, err=%ld\n", __func__, ret);
		return -1;
	}

	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_lflag = 0;
	termios.c_cflag = CRTSCTS | CLOCAL | CS8 | CREAD;
	termios.c_cc[VMIN] = 0;
	termios.c_cc[VTIME] = 1;	/* 100ms timeout */
	termios.c_cflag |= B921600;	/* baud rate */

	/* Set termios */
	ret = f->f_op->unlocked_ioctl(f, TCSETS, (unsigned long)&termios);
	if (ret) {
		pr_err("%s TCSETS failed, err=%ld\n", __func__, ret);
		return -1;
	}

	/* Restore address limit */
	set_fs(fs);

	return 0;
}

/*
 * bcm4773_hello - wakeup chip by toggling mcu_req while monitoring mcu_resp to check if awake
 *
 */
static bool bcm477x_hello(struct bcm_tty_priv *priv)
{
	int count = 0, retries = 0;

	gpio_set_value(priv->mcu_req, 1);
	while (!gpio_get_value(priv->mcu_resp)) {
		if (count++ > 100) {
			gpio_set_value(priv->mcu_req, 0);
			printk("[SSPBBD] MCU_REQ_RESP timeout. MCU_RESP(gpio%d) not responding to MCU_REQ(gpio%d)\n",
					priv->mcu_resp, priv->mcu_req);
			return false;
		}

		mdelay(1);

		/*if awake, done */
		if (gpio_get_value(priv->mcu_resp))
			break;

		if (count%20 == 0 && retries++ < 3) {
			gpio_set_value(priv->mcu_req, 0);
			mdelay(1);
			gpio_set_value(priv->mcu_req, 1);
			mdelay(1);
		}
	}
	return true;
}

/*
 * bcm4773_bye - set mcu_req low to let chip go to sleep
 *
 */
static void bcm477x_bye(struct bcm_tty_priv *priv)
{
	gpio_set_value(priv->mcu_req, 0);
}

static int bcm_tty_open(struct inode *inode, struct file *filp)
{
	/* Initially, file->private_data points device itself and we can get our priv structs from it. */
	struct bcm_tty_priv *priv = container_of(filp->private_data, struct bcm_tty_priv, misc);

	pr_info("%s++\n", __func__);

	/* Open tty */
	priv->tty = filp_open(PORT_NAME, O_RDWR, 0);
	if (IS_ERR(priv->tty)) {
		int ret = (int)PTR_ERR(priv->tty);
		pr_err("%s can not open %s, error=%d\n", __func__, PORT_NAME, ret);
		return ret;
	}

	/* Config tty */
	if (bcm_tty_config(priv->tty)) {
		pr_err("%s can not change %s setting.\n", __func__, PORT_NAME);
		return -EIO;
	}

	filp->private_data = priv;

	pr_info("%s--\n", __func__);
	return 0;
}

static int bcm_tty_release(struct inode *inode, struct file *filp)
{
	struct bcm_tty_priv *priv = (struct bcm_tty_priv *) filp->private_data;
	struct file *tty = priv->tty;

	return filp_close(tty, 0);
}

static ssize_t bcm_tty_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct bcm_tty_priv *priv = (struct bcm_tty_priv *) filp->private_data;
	struct file *tty = priv->tty;
	ssize_t len;

	len = tty->f_op->read(tty, buf, size, ppos);

#ifdef CONFIG_SENSORS_SSP_BBD
	/* Call BBD */
	bbd_parse_asic_data(buf, len, NULL, NULL);
#endif

	return len;
}

static ssize_t bcm_tty_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct bcm_tty_priv *priv = (struct bcm_tty_priv *) filp->private_data;
	struct file *tty = priv->tty;
	ssize_t ret;

	bcm477x_hello(priv)
		ret = tty->f_op->write(tty, buf, size, ppos);
	bcm477x_bye(priv);
	return ret;
}

static unsigned int bcm_tty_poll(struct file *filp, poll_table *wait)
{

	struct bcm_tty_priv *priv = (struct bcm_tty_priv *) filp->private_data;
	struct file *tty = priv->tty;

	return tty->f_op->poll(tty, wait);
}


static const struct file_operations bcm_tty_fops = {
	.owner          =  THIS_MODULE,
	.open           =  bcm_tty_open,
	.release        =  bcm_tty_release,
	.read           =  bcm_tty_read,
	.write          =  bcm_tty_write,
	.poll           =  bcm_tty_poll,
};


static irqreturn_t bcm_irq_handler(int irq, void *pdata)
{
	/* We don't need to do something here because this irq is for waking up host. */
	return IRQ_HANDLED;
}


static int __init bcm_tty_init(void)
{
	struct bcm_tty_priv *priv;
	int ret, irq;

	/* Check GPIO# */
#ifndef CONFIG_OF
	pr_err("[SSPBBD]: Check platform_data for bcm device\n");

#define MCU_REQ_GPIO	0xFF
#define MCU_RESP_GPIO	0xFF
#define HOST_REQ_GPIO	0xFF
	mcu_req = MCU_REQ_GPIO;
	mcu_resp = MCU_RESP_GPIO;
	host_req = HOST_REQ_GPIO;
#else
	/*===================================================
	  We need folowing OF node in dts

	  bcm477x-gpio {
		  ssp-mcu-req = <some_gpio_number>
		  ssp-mcu-resp = <some_gpio_number>
		  ssp-host-req = <some_gpio_number>
	  }
	  ===================================================== */
	struct device_node *np = of_find_node_by_name(NULL, "bcm477x-gpio");
	if (!np) {
		pr_err("[SSPBBD] fail to find OF node bcm477x-gpio\n");
		goto err_exit;
	}
	of_property_read_u32(np, "ssp-mcu-req", &mcu_req);
	of_property_read_u32(np, "ssp-mcu-resp", &mcu_resp);
	of_property_read_u32(np, "ssp-host-req", &host_req);
#endif

	pr_info("[SSPBBD] ssp-host-req=%d, ssp-mcu_req=%d, ssp-mcu-resp=%d\n", host_req, mcu_req, mcu_resp);
	if (host_req < 0 || mcu_req < 0 || mcu_resp < 0) {
		pr_err("[SSPBBD]: GPIO value not correct\n");
		goto err_exit;
	}

	/* Check IRQ# */
	irq = gpio_to_irq(host_req);
	if (irq < 0) {
		pr_err("[SSPBBD]: irq=%d for host_req=%d not correct\n", irq, host_req);
		goto err_exit;
	}

	/* Config GPIO */
	gpio_request(mcu_req, "MCU REQ");
	gpio_direction_output(mcu_req, 0);
	gpio_request(mcu_resp, "MCU RESP");
	gpio_direction_input(mcu_resp);


	/* Alloc */
	priv = (struct bcm_tty_priv *) kmalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		pr_err("%s Failed to allocate \"gps_tty\"\n");
		goto err_exit;
	}

	memset(priv, 0, sizeof(*priv));

	/* Init - gpios */
	priv->host_req = host_req;
	priv->mcu_req  = mcu_req;
	priv->mcu_resp = mcu_resp;

	/* Register misc device */
	priv->misc.minor = MISC_DYNAMIC_MINOR;
	priv->misc.name = "gps_tty";
	priv->misc.fops = &bcm_tty_fops;

	ret = misc_register(&priv->misc);
	if (ret) {
		pr_err("%s Failed to register \"gps_tty\". err=%d\n", __func__, ret);
		goto free_mem;
	}

	/* Request IRQ */
	ret = request_irq(irq, bcm_irq_handler, IRQF_TRIGGER_RISING, "bcm477x_host_wake", priv);
	if (ret) {
		pr_err("[SSPBBD]: Failed to register BCM4773 SPI TTY IRQ %d.\n", irq);
		goto free_wq;
	}

	enable_irq_wake(irq);

	return 0;

free_mem:
	if (priv)
		kfree(priv);
err_exit:
	return -ENODEV;
}

static void __exit bcm_tty_exit(void)
{
}

module_init(bcm_tty_init);
module_exit(bcm_tty_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM TTY Driver");

