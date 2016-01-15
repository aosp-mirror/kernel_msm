/* drivers/input/touchscreen/touch_fw_update.c
 *
 * Copyright (c)2014 HTC.
 *
 * Driver Version: 1.0.0.0
 * Release Date: Aug 28, 2014
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <asm/byteorder.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/input/touch_fw_update.h>
//#include <mach/board.h>
//#include <mach/board_htc.h>

#define TOUCH_LOG_NAME "[TP][FWU]"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) TOUCH_LOG_NAME ": " fmt

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, ...) printk(KERN_INFO pr_fmt(fmt) "\n", ##__VA_ARGS__)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, ...) printk(KERN_ERR TOUCH_LOG_NAME \
			"TOUCH_ERR:(%s:%d): " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

#define RETRY_TIMES 3

#define TOUCH_FWU_IOCTL_CODE			(0x81)
#define FW_UPDATE_PROCCESS			_IO(TOUCH_FWU_IOCTL_CODE, 1)
#define FW_FILE_SIZE				_IOW(TOUCH_FWU_IOCTL_CODE, 2, uint32_t)
#define FW_FILE_REQUEST				_IO(TOUCH_FWU_IOCTL_CODE, 3)
#define FW_LOAD_DONE				_IO(TOUCH_FWU_IOCTL_CODE, 4)
#define FW_UPDATE_BYPASS			_IO(TOUCH_FWU_IOCTL_CODE, 5)

static u32 debug_mask = 0x00000000;
static int driver_probe_status = 0;
static struct kobject *android_touch_kobj;

struct cdev_data {
	int size_count;
	size_t fw_size;
	unsigned char *buf;
};
static struct cdev_data *fwu_cdev_data = NULL;

struct data {
	int (*fwupdate)(struct firmware *fw);
	u8 flash_status;
	u8 download_start;
	u8 update_bypass;
	char fw_vendor[20];
	char fw_ver[20];
	size_t fw_size;
	u32 flash_timeout;
	u32 flash_progress;
	struct firmware *fw;
};
static struct data *fwu_data;

static int fw_update_process(struct firmware *fw);

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%08X\n", debug_mask);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (sscanf(buf, "%ux", &debug_mask) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	return count;
}

static ssize_t touch_vendor_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s", fwu_data->fw_vendor);
}

static ssize_t touch_fw_ver_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s", fwu_data->fw_ver);
}

static ssize_t fw_update_progress_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", fwu_data->flash_progress);
}

static ssize_t fw_update_timeout_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", fwu_data->flash_timeout);
}

static ssize_t fw_update_status_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", fwu_data->flash_status);
}

static ssize_t fw_update_status_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (buf[0] == 0) {
		pr_info("%s: echo 0", __func__);
		fwu_data->download_start= 1;
		fwu_data->flash_status = 0;
	}

	return count;
}

static struct device_attribute dev_attr[] = {
	__ATTR(debug_level, (S_IWUSR|S_IRUGO), debug_show, debug_store),
	__ATTR(vendor, S_IRUGO, touch_vendor_show, NULL),
	__ATTR(fw_ver, S_IRUGO, touch_fw_ver_show, NULL),
	__ATTR(fw_update_status, (S_IWUSR|S_IRUGO), fw_update_status_show, fw_update_status_store),
	__ATTR(fw_update_timeout, S_IRUGO, fw_update_timeout_show, NULL),
	__ATTR(fw_update_progress, S_IRUGO, fw_update_progress_show, NULL),
};

static int touch_fwu_open(struct inode *inode, struct file *filp)
{
	pr_info("%s", __func__);
	fwu_data->download_start= 1;
	fwu_data->flash_status = 1;
	fwu_data->update_bypass = 0;
	fwu_data->flash_progress = 0;
	fwu_cdev_data->size_count = 0;
	fwu_cdev_data->fw_size = 0;
	filp->private_data = fwu_cdev_data;
	return 0;
}

static int touch_fwu_release(struct inode *inode, struct file *filp)
{
	struct cdev_data *fw_cdev = filp->private_data;
	unsigned char *buf;

	pr_info("%s", __func__);
	if (fw_cdev->buf != NULL) {
		pr_info("%s: free buf", __func__);
		buf = fw_cdev->buf;
		kfree(buf);
		fw_cdev->buf = NULL;
	}
	fw_cdev->size_count = 0;
	fw_cdev->fw_size = 0;
	if (fwu_data->update_bypass)
		fwu_data->flash_status = 3;
	else
		fwu_data->flash_status = 2;
	fwu_data->download_start= 0;
	fwu_data->flash_progress = 100;
	return 0;
}

static ssize_t touch_fwu_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	//struct cdev_data *fw_cdev = filp->private_data;
	pr_info("%s: %zu", __func__, count);
	return 0;
}

static ssize_t touch_fwu_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct cdev_data *fw_cdev = file->private_data;
	u16 *tmp;

	pr_info("%s: %zu", __func__, count);
	tmp = kzalloc(count, GFP_KERNEL);
	if (!tmp) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	if(copy_from_user(tmp, buf, count)) {
		pr_err("%s: copy_from_user failed", __func__);
		kfree(tmp);
		return -ENOMEM;
	}

	memcpy(fw_cdev->buf+fw_cdev->size_count, tmp, count);
	fw_cdev->size_count += count;
	kfree(tmp);
	return 0;
}

static unsigned int touch_fwu_poll(struct file *filp, struct poll_table_struct *wait)
{
	return 0;
}

static long touch_fwu_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	int ret = 0;
	u32 data = 0;
	struct cdev_data *fw_cdev = filp->private_data;
	unsigned char *buf;
	struct firmware *fw;

	pr_info("%s: cmd: %x", __func__, cmd);
	switch (cmd) {
		case FW_UPDATE_PROCCESS:
			pr_info("%s: FW_UPDATE_PROCCESS", __func__);
			fw = fwu_data->fw;
			if (fw == NULL) {
				pr_err("%s, no fw data", __func__);
				return -1;
			}
			ret = fw_update_process(fw);
			if (ret == 1) {
				pr_info("%s: fw bypass", __func__);
				fwu_data->update_bypass = 1;
			}
			pr_info("%s: free fw", __func__);
			kfree(fw);
			fwu_data->fw = NULL;
			break;
		case FW_FILE_SIZE:
			data = args;
			fwu_data->fw_size = data;
			fw_cdev->fw_size = data;
			pr_info("%s: FW_FILE_SIZE:%d", __func__, data);
			break;
		case FW_FILE_REQUEST:
			if (fw_cdev->fw_size) {
				if (fw_cdev->buf == NULL) {
					pr_info("%s: allocate buf", __func__);
					buf = kzalloc(
						fw_cdev->fw_size*sizeof(unsigned char),
						GFP_KERNEL);
					if (!buf) {
						pr_err("%s, allocate failed", __func__);
						return -1;
					}
					fw_cdev->buf = buf;
				}
			}
			pr_info("%s: FW_FILE_REQUEST", __func__);
			break;
		case FW_LOAD_DONE:
			pr_info("%s: FW_LOAD_DONE", __func__);
			if (fwu_data->fw != NULL) {
				pr_info("%s: free fw", __func__);
				fw = fwu_data->fw;
				kfree(fw);
			}
			pr_info("%s: allocate fw", __func__);
			fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);
			if (fw == NULL) {
				pr_err("%s, no fw data", __func__);
				return -1;
			}
			fwu_data->fw = fw;
			fwu_data->fw->size = fw_cdev->fw_size;
			fwu_data->fw->data = fw_cdev->buf;
			break;
		case FW_UPDATE_BYPASS:
			pr_info("%s: FW_UPDATE_BYPASS", __func__);
			fwu_data->update_bypass = 1;
			break;
		default:
			break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long
touch_fwu_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return touch_fwu_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define touch_fwu_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int fw_update_process(struct firmware *fw)
{
	pr_info("%s: enter", __func__);
	if (fwu_data->fwupdate)
		return fwu_data->fwupdate(fw);
	else
		return -1;
}

void touch_fw_update_progress(int percentage)
{
	pr_info("%s: %d", __func__, percentage);
	if (percentage >= 100)
		fwu_data->flash_progress = 100;
	else if (percentage <= 0)
		fwu_data->flash_progress = 10;
	else
		fwu_data->flash_progress = percentage;
}
EXPORT_SYMBOL(touch_fw_update_progress);

int register_fw_update(struct touch_fwu_notifier *notifier)
{
	pr_info("%s: enter", __func__);
	if (fwu_data == NULL) {
		if(!driver_probe_status) {
			pr_info("%s: Not probe yet, init first", __func__);
			fwu_data = kzalloc(sizeof(*fwu_data), GFP_KERNEL);
			if (!fwu_data) {
				pr_err("%s: allocate failed", __func__);
				return -1;
			}
		}
		else {
			pr_err("%s: register failed", __func__);
			return -1;
		}
	}

	fwu_data->fwupdate = notifier->fwupdate;
	fwu_data->flash_timeout = notifier->flash_timeout;
	fwu_data->flash_status = 0;
	fwu_data->update_bypass = 0;
	fwu_data->flash_progress = 0;
	memset(fwu_data->fw_vendor, 0, sizeof(fwu_data->fw_vendor));
	memcpy(fwu_data->fw_vendor, notifier->fw_vendor, sizeof(notifier->fw_vendor));
	memcpy(fwu_data->fw_ver, notifier->fw_ver, sizeof(notifier->fw_ver));
	pr_info("%s: register success", __func__);
	return 0;
}
EXPORT_SYMBOL(register_fw_update);

void unregister_fw_update(void)
{
	pr_info("%s", __func__);
	if (fwu_data) {
		fwu_data->fwupdate = NULL;
		fwu_data->flash_timeout = 0;
		fwu_data->flash_progress = 0;
		memset(fwu_data->fw_vendor, 0, sizeof(fwu_data->fw_vendor));
		snprintf(fwu_data->fw_vendor, sizeof(fwu_data->fw_vendor), "NULL");
		memset(fwu_data->fw_ver, 0, sizeof(fwu_data->fw_vendor));
	}
}
EXPORT_SYMBOL(unregister_fw_update);

static int touch_fwu_probe(struct platform_device *pdev)
{
	int ret = 0, attr_count = 0;

	pr_info("%s: enter", __func__);
	if (!fwu_data) {
		fwu_data = kzalloc(sizeof(*fwu_data), GFP_KERNEL);
		if (!fwu_data) {
			pr_err("Fail to allocate fw update data memory");
			ret = -ENOMEM;
			goto err_device_init;
		}
		fwu_data->flash_status = 0;
		fwu_data->update_bypass = 0;
		fwu_data->flash_progress = 0;
		memset(fwu_data->fw_vendor, 0, sizeof(fwu_data->fw_vendor));
		snprintf(fwu_data->fw_vendor, sizeof(fwu_data->fw_vendor), "NULL");
	}

	android_touch_kobj = kobject_create_and_add("android_touch_fwu", NULL);
	if (android_touch_kobj == NULL) {
		pr_err("failed to create kobj");
		ret = -1;
		goto err_create_kobj;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(dev_attr); attr_count++) {
		if (sysfs_create_file(android_touch_kobj, &dev_attr[attr_count].attr) < 0) {
			pr_err("failed to create sysfs file");
			ret = -1;
			goto err_create_sys_file;
		}
	}
	driver_probe_status = 1;
	pr_info("%s: done", __func__);
	return 0;

err_create_sys_file:
	for (attr_count--; attr_count>=0; attr_count--) {
		sysfs_remove_file(android_touch_kobj, &dev_attr[attr_count].attr);
	}
	kobject_del(android_touch_kobj);
err_create_kobj:
	driver_probe_status = 0;
	kfree(fwu_data);
err_device_init:
	return ret;
}

static int touch_fwu_remove(struct platform_device *pdev)
{
	int attr_count = 0;

	for (attr_count = 0; attr_count < ARRAY_SIZE(dev_attr); attr_count++) {
		sysfs_remove_file(android_touch_kobj, &dev_attr[attr_count].attr);
	}
	kobject_del(android_touch_kobj);
	kfree(fwu_data);
	driver_probe_status = 0;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id touch_fwu_mttable[] = {
	{ .compatible = "htc,touch_fwu"},
	{},
};
#else
#define touch_fwu_mttable NULL
#endif

static struct platform_driver touch_fwu_driver = {
	.probe  = touch_fwu_probe,
	.remove = touch_fwu_remove,
	.driver = {
		.name = "touch_fw_update",
		.owner = THIS_MODULE,
		.of_match_table = touch_fwu_mttable,
	},
};

static const struct file_operations touch_fwu_fops = {
	.owner	= THIS_MODULE,
	.read	= touch_fwu_read,
	.write	= touch_fwu_write,
	.unlocked_ioctl = touch_fwu_ioctl,
	.compat_ioctl = touch_fwu_compat_ioctl,
	.open	= touch_fwu_open,
	.release= touch_fwu_release,
	.poll	= touch_fwu_poll,
};

static struct miscdevice touch_fwu_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch_fwu",
	.fops = &touch_fwu_fops,
};

static int __init touch_fwu_init(void)
{
	int ret = 0;

	ret = misc_register(&touch_fwu_misc_dev);
	if (ret) {
		pr_info("misc device register failed");
		return ret;
	}

	fwu_cdev_data = kzalloc(1*sizeof(struct cdev_data), GFP_KERNEL);
	if (fwu_cdev_data == NULL) {
		pr_info("Allocate cdev_data failed");
		return -ENOMEM;
	}

	return platform_driver_register(&touch_fwu_driver);
}

static void __exit touch_fwu_exit(void)
{
	kfree(fwu_cdev_data);
	misc_deregister(&touch_fwu_misc_dev);
	platform_driver_unregister(&touch_fwu_driver);
}

module_init(touch_fwu_init);
module_exit(touch_fwu_exit);

MODULE_AUTHOR("HTC");
MODULE_DESCRIPTION("Touch Firmware update Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0.0");
