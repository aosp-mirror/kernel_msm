#include <linux/export.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/types.h>

static int asusdebug_open(struct inode * inode, struct file * file)
{
	return 0;
}

static int asusdebug_release(struct inode * inode, struct file * file)
{
	return 0;
}

static ssize_t asusdebug_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t asusdebug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char messages[256];

	if (count > 256)
		count = 256;

	memset(messages, 0, sizeof(messages));

	if (copy_from_user(messages, buf, count)) {
		printk("[adbg] %d copy_from_user failed\n", __LINE__);
		return -EFAULT;
	}

	if (strncmp(messages, "adbreboot", 9) == 0) {
		printk("[adbg] rebooting, reason: adb reboot\n");
	}
	return count;
}

static const struct file_operations proc_asusdebug_operations = {
	.read     = asusdebug_read,
	.write    = asusdebug_write,
	.open     = asusdebug_open,
	.release  = asusdebug_release,
};

static int __init proc_asusdebug_init(void)
{
	proc_create("asusdebug", S_IALLUGO, NULL, &proc_asusdebug_operations);
	printk("[adbg] ASUSDebug init\n");
	return 0;
}
module_init(proc_asusdebug_init);
