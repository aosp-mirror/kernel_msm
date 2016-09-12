/* Copyright (c) 2012-2015, mcu gpio init. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <soc/qcom/socinfo.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>


#define GPIO_LEN	  16

int McuGpioBoot0 = 41;
int McuGpioReset = 99;
int McuGpioSenserPower = 98;

/* min reset delay time (ms)*/
#define MIN_MCU_RESET_TIME 20

/* set gpio output value*/
static int mcu_set_gpio(unsigned gpio, int value)
{
	if ( (value != 0) && (value != 1))
	{
		pr_err("mcu_set_gpio value invalid\n");
		return -EINVAL;
	}
	gpio_direction_output(gpio, value);
	pr_info("mcu_set_gpio (%u) value (%d).\n", gpio, value);

	return 0;

}

struct proc_dir_entry *mcu_dir;

static ssize_t mcu_write_proc_gpio(unsigned gpio, const char __user *buffer, size_t count)
{
	char b;

	if (copy_from_user(&b, buffer, 1))
	{
		pr_err("Set value failed copy\n");
		return -EFAULT;
	}
	if (mcu_set_gpio(gpio, b - '0'))
	{
		pr_err("Set value invalid\n");
		return -EINVAL;
	}

	return count;

}

static ssize_t mcu_read_proc_gpio_boot0
				(struct file *file, char __user *userbuf,
				size_t bytes, loff_t *off)
{
	return 0;
}

static ssize_t mcu_write_proc_gpio_boot0
				(struct file *file, const char __user *buffer,
				size_t count, loff_t *pos)
{
	if (count < 1)
	{
		return -EINVAL;
	}

	return mcu_write_proc_gpio(McuGpioBoot0, buffer, count);
}

static ssize_t mcu_read_proc_gpio_reset
				(struct file *file, char __user *userbuf,
				size_t bytes, loff_t *off)
{
	return 0;
}

static ssize_t mcu_write_proc_gpio_reset
				(struct file *file, const char __user *buffer,
				size_t count, loff_t *pos)
{
	if (count < 1)
	{
		return -EINVAL;
	}

	return mcu_write_proc_gpio(McuGpioReset, buffer, count);
}

static ssize_t mcu_read_proc_gpio_senser_power
				(struct file *file, char __user *userbuf,
				size_t bytes, loff_t *off)
{
	return 0;
}

static ssize_t mcu_write_proc_gpio_senser_power
				(struct file *file, const char __user *buffer,
				size_t count, loff_t *pos)
{
	if (1 > count)
	{
		return -EINVAL;
	}

	return mcu_write_proc_gpio(McuGpioSenserPower, buffer, count);
}

static const struct file_operations proc_fops_gpio_boot0 = {
	.owner = THIS_MODULE,
	.read = mcu_read_proc_gpio_boot0,
	.write = mcu_write_proc_gpio_boot0,
};

static const struct file_operations proc_fops_gpio_reset = {
	.owner = THIS_MODULE,
	.read = mcu_read_proc_gpio_reset,
	.write = mcu_write_proc_gpio_reset,
};

static const struct file_operations proc_fops_gpio_senser_power = {
	.owner = THIS_MODULE,
	.read = mcu_read_proc_gpio_senser_power,
	.write = mcu_write_proc_gpio_senser_power,
};

static int mcu_gpio_request(void)
{
	struct device_node *np = NULL;

	/*mcu boot0*/
	np = of_find_compatible_node(NULL, NULL, "mcu,boot0");
	if (!np) {
		printk(KERN_ERR "%s: %s node not found\n", __FUNCTION__, "mcu,boot0");
		return -ENODEV;
	}

	McuGpioBoot0 = of_get_named_gpio(np, "mcu_boot_0", 0);
	if (McuGpioBoot0 >= 0) {
		printk(KERN_ERR "%s: mcu_boot_0:%d.\n", __FUNCTION__, McuGpioBoot0);
	}
	if (gpio_request(McuGpioBoot0, "mcu_boot_0"))
		printk(KERN_ERR "%s: Failed to request gpio %d for mcu_boot_0\n",
			__FUNCTION__, McuGpioBoot0);
	else
		printk(KERN_INFO "%s: gpio_request mcu_boot_0 done\n", __FUNCTION__);

	/*mcu reset*/
	np = of_find_compatible_node(NULL, NULL, "mcu,reset");
	if (!np) {
		printk(KERN_ERR "%s: %s node not found\n", __FUNCTION__, "mcu,reset");
		return -ENODEV;
	}

	McuGpioReset = of_get_named_gpio(np, "mcu_reset", 0);
	if (McuGpioReset >= 0) {
		printk(KERN_ERR "%s: mcu_reset:%d.\n", __FUNCTION__, McuGpioReset);
	}
	if (gpio_request(McuGpioReset, "mcu_reset"))
		printk(KERN_ERR "%s: Failed to request gpio %d for mcu_reset\n",
			__FUNCTION__, McuGpioReset);
	else
		printk(KERN_INFO "%s: gpio_request mcu_reset done\n", __FUNCTION__);

	/*mcu senser power*/
	np = of_find_compatible_node(NULL, NULL, "mcu,senserpower");
	if (!np) {
		printk(KERN_ERR "%s: %s node not found\n", __FUNCTION__, "mcu,senserpower");
		return -ENODEV;
	}

	McuGpioSenserPower = of_get_named_gpio(np, "mcu_senserpower", 0);
	if (McuGpioSenserPower >= 0) {
		printk(KERN_ERR "%s: SenserPower:%d.\n", __FUNCTION__, McuGpioSenserPower);
	}
	if (gpio_request(McuGpioSenserPower, "Senser_Power")) {
		printk(KERN_ERR "%s: Failed to request gpio %d for mcu_reset\n",
			__FUNCTION__, McuGpioSenserPower);
	}
	else {
		printk(KERN_INFO "%s: gpio_request Senser_Power done\n", __FUNCTION__);
	}

	return 0;
}

static int __init board_mcu_gpio_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

	mcu_dir = proc_mkdir("mcu", NULL);
	if (mcu_dir == NULL)
	{
		pr_err("Unable to create /proc/mcu directory.\n");
		return -ENOMEM;
	}

	/* read/write mcu boot0 proc entries */
	ent = proc_create("gpio_mcu_boot0", 0660, mcu_dir, &proc_fops_gpio_boot0);
	if (ent == NULL)
	{
		pr_err("Unable to create /proc/mcu/gpio_mcu_boot0 entry.\n");
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write mcu reset proc entries */
	ent = proc_create("gpio_mcu_reset", 0660, mcu_dir, &proc_fops_gpio_reset);
	if (ent == NULL)
	{
		pr_err("Unable to create /proc/mcu/gpio_mcu_reset entry.\n");
		retval = -ENOMEM;
		goto fail1;
	}

	/* read/write senser power proc entries */
	ent = proc_create("gpio_senser_power", 0660, mcu_dir, &proc_fops_gpio_senser_power);
	if (NULL == ent)
	{
		pr_err("Unable to create /proc/mcu/gpio_senser_power entry.\n");
		retval = -ENOMEM;
		goto fail2;
	}

	/*mcu gpio request*/
	if (mcu_gpio_request())
	{
		pr_err( "failed to request mcu gpio\n");
		retval = -EIO;
		goto fail3;
	}

	return 0;

fail3:
	remove_proc_entry("gpio_senser_power", mcu_dir);
fail2:
	remove_proc_entry("gpio_mcu_reset", mcu_dir);
fail1:
	remove_proc_entry("gpio_mcu_boot0", mcu_dir);
fail:
	remove_proc_entry("mcu", 0);
	return retval;
}

/**
 * Cleans up the module.
 */
static void __exit board_mcu_gpio_exit(void)
{
	gpio_free(McuGpioBoot0);
	gpio_free(McuGpioReset);
	remove_proc_entry("gpio_mcu_boot0", mcu_dir);
	remove_proc_entry("gpio_mcu_reset", mcu_dir);
	remove_proc_entry("mcu", 0);
}

module_init(board_mcu_gpio_init);
module_exit(board_mcu_gpio_exit);
