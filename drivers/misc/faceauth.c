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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/faceauth.h>
#include <linux/firmware.h>

#include <linux/miscdevice.h>
#include <linux/uio.h>
#include <linux/uaccess.h>

#include <abc-pcie-dma.h>
#include "../mfd/abc-pcie-dma.h"

/* ABC AON config regisetr offsets */
#define SYSREG_AON 0x30000
#define SYSREG_REG_GP_INT0 (SYSREG_AON + 0x37C)
#define SYSREG_AON_IPU_REG29 (SYSREG_AON + 0x438)
#define SYSREG_BASE 0x10B00000
#define SYSREG_REG_GP_INT0_ADDR (SYSREG_BASE + SYSREG_REG_GP_INT0)
#define SYSREG_AON_IPU_REG29_ADDR (SYSREG_BASE + SYSREG_AON_IPU_REG29)

/* ABC FW and workload binary offsets */
#define M0_FIRMWARE_ADDR 0x20000000
#define COMPARE_RESULT_FLAG_ADDR 0x21fffff4
#define OPERATION_FLAG_ADDR 0x21fffff8
#define COMPLETION_FLAG_ADDR 0x21fffffc
#define JQS_DEPTH_ADDR 0x22000000
#define JQS_AFFINE_16_ADDR 0x22100000
#define JQS_AFFINE_RGB_ADDR 0x22200000
#define JQS_AFFINE_8_ADDR 0x22300000
#define DOT_IMAGE_LEFT_ADDR 0x22800000
#define DOT_IMAGE_RIGHT_ADDR 0x22900000
#define FLOOD_IMAGE_ADDR 0x23000000

#define DEBUG_PRINT_ADDR 0x23f00000
#define DEBUG_PRINT_SIZE 0x00100000

/* ABC FW and workload path */
#define M0_FIRMWARE_PATH "m0_workload.fw"
#define JQS_DEPTH_PATH "depth.fw"
#define JQS_AFFINE_8_PATH "affine_8.fw"
#define JQS_AFFINE_16_PATH "affine_16.fw"
#define JQS_AFFINE_RGB_PATH "affine_rgb.fw"

/* Timeout */
#define FACEAUTH_TIMEOUT 3000

static int dma_xfer(void *buf, int size, const int remote_addr,
		    enum dma_data_direction dir);
static int dma_xfer_vmalloc(void *buf, int size, const int remote_addr,
			    enum dma_data_direction dir);
static int dma_send_fw(const char *path, const int remote_addr);
static int dma_write_dw(struct file *file, const int remote_addr,
			const int val);
static int dma_read_dw(struct file *file, const int remote_addr, int *val);
static int dma_send_images(struct faceauth_start_data *data);
static int dma_send_workloads(void);
static int dma_gather_debug(struct faceauth_debug_data *data);

struct faceauth_data {
	int dma_dw_buf;
};
bool hypx_enable;
struct dentry *faceauth_debugfs_root;

static long faceauth_dev_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	int err = 0;
	struct faceauth_start_data start_step_data = { 0 };
	struct faceauth_continue_data continue_step_data = { 0 };
	struct faceauth_debug_data debug_step_data = { 0 };
	unsigned long stop, ioctl_start;
	uint32_t result;

	ioctl_start = jiffies;

	switch (cmd) {
	case FACEAUTH_DEV_IOC_INIT:
		pr_info("faceauth init IOCTL\nSend faceauth workloads\n");

		err = dma_send_workloads();
		if (err) {
			pr_err("Error in sending M0 firmware\n");
			goto exit;
		}

		break;
	case FACEAUTH_DEV_IOC_START:
		pr_info("faceauth start IOCTL\n");

		if (copy_from_user(&start_step_data, (const void __user *)arg,
				   sizeof(start_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		if (start_step_data.operation == FACEAUTH_OP_ENROLL ||
		    start_step_data.operation == FACEAUTH_OP_VALIDATE) {
			if (!start_step_data.image_dot_left_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_dot_right_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_flood_size) {
				err = -EINVAL;
				goto exit;
			}

			pr_info("Send images\n");
			err = dma_send_images(&start_step_data);
			if (err) {
				pr_err("Error in sending workload\n");
				goto exit;
			}
		}

		/* Set M0 firmware address */
		pr_info("Set M0 firmware addr = 0x%08x\n", M0_FIRMWARE_ADDR);
		dma_write_dw(file, SYSREG_AON_IPU_REG29_ADDR, M0_FIRMWARE_ADDR);

		/* Set operation flag */
		pr_info("Set faceauth operation flag at 0x%08x\n",
			OPERATION_FLAG_ADDR);
		dma_write_dw(file, OPERATION_FLAG_ADDR,
			     start_step_data.operation);

		/* Reset completion flag */
		pr_info("Clearing completion flag at 0x%08x\n",
			COMPLETION_FLAG_ADDR);
		dma_write_dw(file, COMPLETION_FLAG_ADDR, 0);

		/* Trigger M0 Interrupt */
		pr_info("Interrupting M0\n");
		dma_write_dw(file, SYSREG_REG_GP_INT0_ADDR, 1);

		/* Check completion flag */
		pr_info("Waiting for completion.\n");
		stop = jiffies + msecs_to_jiffies(FACEAUTH_TIMEOUT);
		for (;;) {
			int done;
			dma_read_dw(file, COMPLETION_FLAG_ADDR, &done);
			if (done) {
				pr_info("Faceauth workflow completes.\n");
				break;
			}
			if (time_before(stop, jiffies)) {
				pr_err("Faceauth workflow timeout!\n");
				err = -ETIME;
				goto exit;
			}
			msleep(1);
		}

		break;
	case FACEAUTH_DEV_IOC_CONTINUE:
		pr_info("faceauth continue IOCTL\n");

		continue_step_data.completed = 1;

		pr_info("Read comparison result\n");
		dma_read_dw(file, COMPARE_RESULT_FLAG_ADDR, &result);
		continue_step_data.result = result;

		if (copy_to_user((void __user *)arg, &continue_step_data,
				 sizeof(continue_step_data)))
			err = -EFAULT;
		goto exit;
		break;
	case FACEAUTH_DEV_IOC_CLEANUP:
		/* TODO cleanup Airbrush DRAM */
		pr_info("faceauth cleanup IOCTL\n");
		break;
	case FACEAUTH_DEV_IOC_DEBUG:
		pr_info("faceauth debug IOCTL\n");
		if (copy_from_user(&debug_step_data, (const void __user *)arg,
				   sizeof(debug_step_data))) {
			err = -EFAULT;
			goto exit;
		}
		err = dma_gather_debug(&debug_step_data);
		break;
	default:
		err = -EFAULT;
		goto exit;
	}

exit:
	pr_info("Faceauth action took %dus\n",
		jiffies_to_usecs(jiffies - ioctl_start));
	return err;
}

static int faceauth_open(struct inode *inode, struct file *file)
{
	struct faceauth_data *data;

	data = vmalloc(sizeof(*data));
	if (!data) {
		pr_err("Failed to vmalloc DW buffer\n");
		return -ENOMEM;
	}
	file->private_data = (void *)data;

	return 0;
}

static int faceauth_free(struct inode *inode, struct file *file)
{
	struct faceauth_data *data = file->private_data;

	vfree(data);
	return 0;
}

static const struct file_operations faceauth_dev_operations = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = faceauth_dev_ioctl,
	.compat_ioctl = faceauth_dev_ioctl,
	.open = faceauth_open,
	.release = faceauth_free,
};

static struct miscdevice faceauth_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "faceauth",
	.fops = &faceauth_dev_operations,
};

/**
 * Local function to transfer data between user space memory and Airbrush via
 * PCIE
 * @param[in] buf Address of user space buffer
 * @param[in] size Size of buffer
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] dir Direction of data transfer
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_xfer(void *buf, int size, const int remote_addr,
		    enum dma_data_direction dir)
{
	struct abc_pcie_dma_desc dma_desc;
	int err = 0;

	/* Transfer workload to target memory in Airbrush */
	memset((void *)&dma_desc, 0, sizeof(dma_desc));
	dma_desc.local_buf = buf;
	dma_desc.local_buf_type = DMA_BUFFER_USER;
	dma_desc.remote_buf = remote_addr;
	dma_desc.remote_buf_type = DMA_BUFFER_USER;
	dma_desc.size = size;
	dma_desc.dir = dir;
	pr_info("MBLK AP src = %pK; AB dest = %pK; size = %d\n",
		(unsigned long)dma_desc.local_buf,
		(unsigned long)dma_desc.remote_buf, dma_desc.size);
	err = abc_pcie_issue_dma_xfer(&dma_desc);
	return err;
}

/**
 * Local function to transfer data between kernel vmalloc memory and Airbrush
 * via PCIE
 * @param[in] buf Address of kernel vmalloc memory buffer
 * @param[in] size Size of buffer
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] dir Direction of data transfer
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_xfer_vmalloc(void *buf, int size, const int remote_addr,
			    enum dma_data_direction dir)
{
	struct abc_pcie_dma_desc dma_desc;
	int err = 0;

	/* Transfer workload to target memory in Airbrush */
	memset((void *)&dma_desc, 0, sizeof(dma_desc));
	dma_desc.local_buf = buf;
	dma_desc.local_buf_type = DMA_BUFFER_USER;
	dma_desc.remote_buf = remote_addr;
	dma_desc.remote_buf_type = DMA_BUFFER_USER;
	dma_desc.size = size;
	dma_desc.dir = dir;
	pr_info("MBLK AP src = %pK; AB dest = %pK; size = %d\n",
		(unsigned long)dma_desc.local_buf,
		(unsigned long)dma_desc.remote_buf, dma_desc.size);
	err = abc_pcie_issue_dma_xfer_vmalloc(&dma_desc);
	return err;
}

/**
 * Local function to send firmware to Airbrush memory via PCIE
 * @param[in] path Firmware
 * @param[in] remote_addr Address of Airbrush memory
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_send_fw(const char *path, const int remote_addr)
{
	int err = 0;
	const struct firmware *fw_entry;
	int fw_status;

	fw_status = request_firmware(&fw_entry, path,
				     faceauth_miscdevice.this_device);
	if (fw_status != 0) {
		pr_err("Firmware Not Found: %d\n", fw_status);
		return -EIO;
	}

	err = dma_xfer_vmalloc((void *)(fw_entry->data), fw_entry->size,
			       remote_addr, DMA_TO_DEVICE);
	if (err)
		pr_err("Error from abc_pcie_issue_dma_xfer: %d\n", err);
	release_firmware(fw_entry);
	return err;
}

/**
 * Local function to write one DW to Airbrush memory via PCIE
 * @param[in] file File struct of this module
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] val DW value to write
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_write_dw(struct file *file, const int remote_addr, const int val)
{
	int err = 0;
	struct faceauth_data *data = file->private_data;

	data->dma_dw_buf = val;
	err = dma_xfer_vmalloc((void *)&(data->dma_dw_buf), sizeof(val),
			       remote_addr, DMA_TO_DEVICE);
	if (err)
		pr_err("Error from abc_pcie_issue_dma_xfer: %d\n", err);
	return err;
}

/**
 * Local function to read one DW to Airbrush memory via PCIE
 * @param[in] file File struct of this module
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] val Variable to store read-back DW
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_read_dw(struct file *file, const int remote_addr, int *val)
{
	int err = 0;
	struct faceauth_data *data = file->private_data;

	err = dma_xfer_vmalloc((void *)&(data->dma_dw_buf), sizeof(*val),
			       remote_addr, DMA_FROM_DEVICE);
	if (err) {
		pr_err("Error from abc_pcie_issue_dma_xfer: %d\n", err);
		return err;
	}
	*val = data->dma_dw_buf;
	return 0;
}

/**
 * Local function to send FaceAuth input data to Airbrush memory via PCIE
 * @param[in] data Data structure copied from user space
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_send_images(struct faceauth_start_data *data)
{
	int err = 0;

	pr_info("Send left dot image\n");
	err = dma_xfer(data->image_dot_left, data->image_dot_left_size,
		       DOT_IMAGE_LEFT_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("Error sending left dot image\n");
		return err;
	}

	pr_info("Send right dot image\n");
	err = dma_xfer(data->image_dot_right, data->image_dot_right_size,
		       DOT_IMAGE_RIGHT_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("Error sending right dot image\n");
		return err;
	}

	/* This is data to feed individual TPU stages */
	pr_info("Send flood image\n");
	err = dma_xfer(data->image_flood, data->image_flood_size,
		       FLOOD_IMAGE_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("Error sending flood image\n");
		return err;
	}

	return err;
}

/**
 * Local function to send all FaceAuth firmwares to Airbrush memory via PCIE
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_send_workloads(void)
{
	int err = 0;

	/* Send IPU workload */
	pr_info("Set JQS Depth addr = 0x%08x\n", JQS_DEPTH_ADDR);
	err = dma_send_fw(JQS_DEPTH_PATH, JQS_DEPTH_ADDR);
	if (err) {
		pr_err("Error during JQS binary transfer: %d\n", err);
		return err;
	}

	pr_info("Set JQS Affine16 addr = 0x%08x\n", JQS_AFFINE_16_ADDR);
	err = dma_send_fw(JQS_AFFINE_16_PATH, JQS_AFFINE_16_ADDR);
	if (err) {
		pr_err("Error during JQS binary transfer: %d\n", err);
		return err;
	}

	pr_info("Set JQS Affine RGB addr = 0x%08x\n", JQS_AFFINE_RGB_ADDR);
	err = dma_send_fw(JQS_AFFINE_RGB_PATH, JQS_AFFINE_RGB_ADDR);
	if (err) {
		pr_err("Error during JQS binary transfer: %d\n", err);
		return err;
	}

	pr_info("Set JQS Affine8 addr = 0x%08x\n", JQS_AFFINE_8_ADDR);
	err = dma_send_fw(JQS_AFFINE_8_PATH, JQS_AFFINE_8_ADDR);
	if (err) {
		pr_err("Error during JQS binary transfer: %d\n", err);
		return err;
	}

	/* Send M0 firmware */
	pr_info("Send M0 firmware to addr 0x%08x\n", M0_FIRMWARE_ADDR);
	err = dma_send_fw(M0_FIRMWARE_PATH, M0_FIRMWARE_ADDR);
	if (err) {
		pr_err("Error during M0 firmware transfer: %d\n", err);
		return err;
	}

	return err;
}

static int dma_gather_debug(struct faceauth_debug_data *data)
{
	int err = 0;

	err = dma_xfer((void *)data->print_buffer,
		min((uint32_t)DEBUG_PRINT_SIZE, data->print_buffer_size),
		DEBUG_PRINT_ADDR, DMA_FROM_DEVICE);

	return err;
}

static int hypx_enable_debugfs_show(struct seq_file *m, void *data)
{
	seq_printf(m, "%s\n", hypx_enable ? "1" : "0");
	return 0;
}

static int hypx_enable_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hypx_enable_debugfs_show, inode->i_private);
}

static ssize_t hypx_enable_debugfs_write(struct file *file,
					 const char __user *ubuf, size_t len,
					 loff_t *offp)
{
	char buf[12];

	if (len > sizeof(buf) - 1)
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;

	while (len > 0 && isspace(buf[len - 1]))
		len--;
	buf[len] = '\0';

	if (!strcmp(buf, "0"))
		hypx_enable = false;
	else if (!strcmp(buf, "1"))
		hypx_enable = true;
	else
		return -EINVAL;

	pr_debug("Faceauth hypx enable flag is set to %d\n", hypx_enable);

	return len;
}

static const struct file_operations hypx_enable_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = hypx_enable_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = hypx_enable_debugfs_write,
};

static int __init faceauth_init(void)
{
	int err;
	struct dentry *hypx;

	pr_info("faceauth init\n");

	err = misc_register(&faceauth_miscdevice);
	if (err)
		goto exit1;

	faceauth_debugfs_root = debugfs_create_dir("faceauth", NULL);
	if (IS_ERR_OR_NULL(faceauth_debugfs_root)) {
		pr_err("Failed to create faceauth debugfs");
		err = -EIO;
		goto exit2;
	}

	hypx = debugfs_create_file("hypx_enable", 0400,
				   faceauth_debugfs_root, NULL,
				   &hypx_enable_debugfs_fops);
	if (!hypx) {
		err = -EIO;
		goto exit3;
	}

	return 0;

exit3:
	debugfs_remove_recursive(faceauth_debugfs_root);

exit2:
	misc_deregister(&faceauth_miscdevice);

exit1:
	return err;
}

static void __exit faceauth_exit(void)
{
	pr_debug("faceauth driver exit\n");

	misc_deregister(&faceauth_miscdevice);
	debugfs_remove_recursive(faceauth_debugfs_root);
}

module_init(faceauth_init);
module_exit(faceauth_exit);

MODULE_AUTHOR("Anatol Pomazau <anatol@google.com>, Lei Liu <leliu@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google FaceAuth driver");
