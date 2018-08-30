/*
 * Google FaceAuth driver
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/faceauth.h>
#include <linux/miscdevice.h>
#include <linux/uio.h>
#include <linux/uaccess.h>

static long faceauth_dev_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	struct faceauth_start_data start_step_data = { 0 };
	struct faceauth_continue_data continue_step_data = { 0 };

	switch (cmd) {
	case FACEAUTH_DEV_IOC_INIT:
		/*
		 * TODO: A number of things need to be done here:
		 *   init PCI link
		 *   load M0 firmware
		 */
		pr_info("faceauth init IOCTL\n");
		break;
	case FACEAUTH_DEV_IOC_START:
		/*
		 * TODO:
		 *   load models from filesystem
		 *   verify models
		 *   clean Airbrush DRAM
		 *   load models into Airbrush DRAM
		 */
		pr_info("faceauth start IOCTL\n");

		if (copy_from_user(&start_step_data, (const void __user *)arg,
				   sizeof(start_step_data)))
			return -EFAULT;

		if (!start_step_data.image_dot_left_size)
			return -EINVAL;
		if (!start_step_data.image_dot_right_size)
			return -EINVAL;
		if (!start_step_data.image_flood_size)
			return -EINVAL;

		/*
		if (copy_from_user(ab_dram_dot_left,
				start_step_data.image_dot_left,
				start_step_data.image_dot_left_size))
			return -EFAULT;
		if (copy_from_user(ab_dram_dot_right,
				start_step_data.image_dot_right,
				start_step_data.image_dot_right_size))
			return -EFAULT;
		if (copy_from_user(ab_dram_flood,
				start_step_data.image_flood,
				start_step_data.image_flood_size))
			return -EFAULT;
		*/
		break;
	case FACEAUTH_DEV_IOC_CONTINUE:
		pr_info("faceauth continue IOCTL\n");

		continue_step_data.completed = 1;
		/* verify that data matches one stored in Citadel */
		continue_step_data.success = 1;

		if (copy_to_user((void __user *)arg, &continue_step_data,
				 sizeof(continue_step_data)))
			return -EFAULT;
		break;
	case FACEAUTH_DEV_IOC_CLEANUP:
		/* TODO cleanup Airbrush DRAM */
		pr_info("faceauth cleanup IOCTL\n");
		break;
	default:
		return -EFAULT;
	}

	return 0;
}

static const struct file_operations faceauth_dev_operations = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = faceauth_dev_ioctl,
	.compat_ioctl = faceauth_dev_ioctl,
};

static struct miscdevice faceauth_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "faceauth",
	.fops = &faceauth_dev_operations,
};

static int __init faceauth_init(void)
{
	int res;

	pr_info("faceauth init\n");

	res = misc_register(&faceauth_miscdevice);

	return res;
}

static void __exit faceauth_exit(void)
{
	pr_debug("faceauth driver exit\n");

	misc_deregister(&faceauth_miscdevice);
}

module_init(faceauth_init);
module_exit(faceauth_exit);

MODULE_AUTHOR("Anatol Pomazau <anatol@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google FaceAuth driver");