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
#include <linux/firmware.h>

#include <linux/miscdevice.h>
#include <linux/uio.h>
#include <linux/uaccess.h>

#include <abc-pcie-dma.h>
#include "../mfd/abc-pcie-dma.h"

/* ABC AON config regisetr offsets */
#define SYSREG_AON			0x30000
#define SYSREG_REG_GP_INT0		(SYSREG_AON + 0x37C)
#define SYSREG_AON_IPU_REG29		(SYSREG_AON + 0x438)
#define SYSREG_BASE			0x10B00000
#define SYSREG_REG_GP_INT0_ADDR		(SYSREG_BASE + SYSREG_REG_GP_INT0)
#define SYSREG_AON_IPU_REG29_ADDR	(SYSREG_BASE + SYSREG_AON_IPU_REG29)

/* ABC FW and workload binary offsets */
#define WORKLOAD_ADDR			0x20000000
#define COMPLETION_FLAG_ADDR		0x21fffffc
#define JQS_DEPTH_ADDR			0x22000000
#define JQS_AFFINE_ADDR			0x22100000
#define IMAGE_LEFT_ADDR			0x22800000
#define IMAGE_RIGHT_ADDR 		0x22900000
#define DEPTH_RAW_OUT_ADDR		0x22a00000
#define AFFINE_RAW_OUT_ADDR		0x22b00000
#define FSSD_IN_ADDR			0x23000000
#define FSSD_OUT_ADDR			0x23100000
#define FACENET_IN_ADDR			0x23200000
#define FACENET_OUT_ADDR		0x23300000
#define GAZENET_IN_ADDR			0x23400000
#define GAZENET_OUT_ADDR		0x23500000
#define DEPTHID_IN_ADDR			0x23600000
#define DEPTHID_OUT_ADDR		0x23700000

/* ABC FW and workload path */
#define M0_WORKLOAD_PATH "m0_workload.fw"
#define JQS_DEPTH_PATH "depth.fw"
#define JQS_AFFINE_PATH "affine.fw"

/* Timeout */
#define FACEAUTH_TIMEOUT 1000

static int dma_xfer(void *buf, int size, const int remote_addr,
		    enum dma_data_direction dir);
static int dma_xfer_vmalloc(void *buf, int size, const int remote_addr,
			    enum dma_data_direction dir);
static int dma_send_fw(const char *path, const int remote_addr);
static int dma_write_dw(struct file *file, const int remote_addr,
			const int val);
static int dma_read_dw(struct file *file, const int remote_addr, int *val);
static int dma_send_images(struct faceauth_start_data *data);
static int dma_read_result(struct faceauth_start_data *data);
static int dma_send_workloads(void);

struct faceauth_data {
	int dma_dw_buf;
};

static long faceauth_dev_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	int err = 0;
	struct faceauth_start_data start_step_data = { 0 };
	struct faceauth_continue_data continue_step_data = { 0 };
	unsigned long stop;

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
		if (!start_step_data.facenet_input_size)
			return -EINVAL;
		if (!start_step_data.gazenet_input_size)
			return -EINVAL;
		if (!start_step_data.depthid_input_size)
			return -EINVAL;

		pr_info("%s: Send images\n", __func__);
		err = dma_send_images(&start_step_data);
		if (err) {
			pr_err("%s: Error in sending workload\n", __func__);
			return err;
		}

		pr_info("%s: Send workloads\n", __func__);
		err = dma_send_workloads();
		if (err) {
			pr_err("%s: Error in sending workload\n", __func__);
			return err;
		}

		/* Set M0 workload address */
		pr_info("%s: Set M0 workload addr = 0x%08x\n", __func__,
			WORKLOAD_ADDR);
		dma_write_dw(file, SYSREG_AON_IPU_REG29_ADDR, WORKLOAD_ADDR);

		/* Reset completion flag */
		pr_info("%s: Clearing completion flag at 0x%08x\n", __func__,
			COMPLETION_FLAG_ADDR);
		dma_write_dw(file, COMPLETION_FLAG_ADDR, 0);

		/* Trigger M0 Interrupt */
		pr_info("%s: Interrupting M0\n", __func__);
		dma_write_dw(file, SYSREG_REG_GP_INT0_ADDR, 1);

		/* Check completion flag */
		pr_info("%s: Waiting for completion.\n", __func__);
		stop = jiffies + msecs_to_jiffies(FACEAUTH_TIMEOUT);
		for (;;) {
			int done;
			dma_read_dw(file, COMPLETION_FLAG_ADDR, &done);
			if (done) {
				pr_info("%s: Workload completes.\n", __func__);
				break;
			}
			if (time_before(stop, jiffies)) {
				pr_err("%s: Airbrush workload timeout!\n",
				       __func__);
				return -ETIME;
			}
			msleep(1);
		}

		pr_info("%s: Read back result from AB DRAM\n", __func__);
		err = dma_read_result(&start_step_data);
		if (err) {
			pr_err("%s: Error in read back result\n", __func__);
			return err;
		}

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

static int faceauth_open(struct inode *inode, struct file *file)
{
	struct faceauth_data *data;

	data = vmalloc(sizeof(*data));
	if (!data) {
		pr_err("%s: Failed to vmalloc DW buffer\n", __func__);
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
	dma_desc.local_buf_size = size;
	dma_desc.local_buf_type = DMA_BUFFER_USER;
	dma_desc.remote_buf = remote_addr;
	dma_desc.remote_buf_type = DMA_BUFFER_USER;
	dma_desc.dir = dir;
	dma_desc.chan = 1;
	pr_info("%s: MBLK AP src = %pK; AB dest = %pK; size = %d\n",
		__func__,
		(unsigned long)dma_desc.local_buf,
		(unsigned long)dma_desc.remote_buf,
		dma_desc.local_buf_size);
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
	dma_desc.local_buf_size = size;
	dma_desc.local_buf_type = DMA_BUFFER_USER;
	dma_desc.remote_buf = remote_addr;
	dma_desc.remote_buf_type = DMA_BUFFER_USER;
	dma_desc.dir = dir;
	dma_desc.chan = 1;
	pr_info("%s: MBLK AP src = %pK; AB dest = %pK; size = %d\n",
		__func__,
		(unsigned long)dma_desc.local_buf,
		(unsigned long)dma_desc.remote_buf,
		dma_desc.local_buf_size);
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
		pr_err("Firmware Not Found: %d, %d\n", fw_status,
				__LINE__);
		return -EIO;
	}

	err = dma_xfer_vmalloc((void *)(fw_entry->data), fw_entry->size,
			       remote_addr, DMA_TO_DEVICE);
	if (err)
		pr_err("%s: Error from abc_pcie_issue_dma_xfer: %d\n",
		       __func__, err);
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
		pr_err("%s: Error from abc_pcie_issue_dma_xfer: %d\n",
		       __func__, err);
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
		pr_err("%s: Error from abc_pcie_issue_dma_xfer: %d\n",
		       __func__, err);
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
		       IMAGE_LEFT_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("%s: Error sending left dot image\n", __func__);
		return err;
	}

	pr_info("Send right dot image\n");
	err = dma_xfer(data->image_dot_right, data->image_dot_right_size,
		       IMAGE_RIGHT_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("%s: Error sending right dot image\n", __func__);
		return err;
	}

	/* This is data to feed individual TPU stages */
	pr_info("Send flood image\n");
	err = dma_xfer(data->image_flood, data->image_flood_size,
		       FSSD_IN_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("%s: Error sending flood image\n", __func__);
		return err;
	}

	pr_info("Send facenet input\n");
	err = dma_xfer(data->facenet_input, data->facenet_input_size,
		       FACENET_IN_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("%s: Error sending facenet input\n", __func__);
		return err;
	}

	pr_info("Send gazenet input\n");
	err = dma_xfer(data->gazenet_input, data->gazenet_input_size,
		       GAZENET_IN_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("%s: Error sending gazenet input\n", __func__);
		return err;
	}

	pr_info("Send depthid input\n");
	err = dma_xfer(data->depthid_input, data->depthid_input_size,
		       DEPTHID_IN_ADDR, DMA_TO_DEVICE);
	if (err) {
		pr_err("%s: Error sending flood image\n", __func__);
		return err;
	}
	return err;
}

/**
 * Local function to read FaceAuth data from Airbrush memory via PCIE
 * @param[in] data Data structure copied from user space
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_read_result(struct faceauth_start_data *data)
{
	int err = 0;

	pr_info("Read depth output\n");
	err = dma_xfer(data->depth_output, data->depth_output_size,
		       DEPTH_RAW_OUT_ADDR, DMA_FROM_DEVICE);
	if (err) {
		pr_err("%s: Error reading depth output\n", __func__);
		return err;
	}

	pr_info("Read affine output\n");
	err = dma_xfer(data->affine_output, data->affine_output_size,
		       AFFINE_RAW_OUT_ADDR, DMA_FROM_DEVICE);
	if (err) {
		pr_err("%s: Error reading affine output\n", __func__);
		return err;
	}

	pr_info("Read fssd output\n");
	err = dma_xfer(data->fssd_output, data->fssd_output_size,
		       FSSD_OUT_ADDR, DMA_FROM_DEVICE);
	if (err) {
		pr_err("%s: Error reading fssd output\n", __func__);
		return err;
	}

	pr_info("Read facenet output\n");
	err = dma_xfer(data->facenet_output, data->facenet_output_size,
		       FACENET_OUT_ADDR, DMA_FROM_DEVICE);
	if (err) {
		pr_err("%s: Error reading facenet output\n", __func__);
		return err;
	}

	pr_info("Read gazenet output\n");
	err = dma_xfer(data->gazenet_output, data->gazenet_output_size,
		       GAZENET_OUT_ADDR, DMA_FROM_DEVICE);
	if (err) {
		pr_err("%s: Error reading gazenet output\n", __func__);
		return err;
	}

	pr_info("Read depthid output\n");
	err = dma_xfer(data->depthid_output, data->depthid_output_size,
		       DEPTHID_OUT_ADDR, DMA_FROM_DEVICE);
	if (err) {
		pr_err("%s: Error reading depthid output\n", __func__);
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
	pr_info("%s: Set JQS Depth addr = 0x%08x\n", __func__,
		JQS_DEPTH_ADDR);
	err = dma_send_fw(JQS_DEPTH_PATH, JQS_DEPTH_ADDR);
	if (err) {
		pr_err("%s: Error during JQS binary transfer: %d\n",
		       __func__, err);
		return err;
	}

	pr_info("%s: Set JQS Affine addr = 0x%08x\n", __func__,
		JQS_AFFINE_ADDR);
	err = dma_send_fw(JQS_AFFINE_PATH, JQS_AFFINE_ADDR);
	if (err) {
		pr_err("%s: Error during JQS binary transfer: %d\n",
		       __func__, err);
		return err;
	}

	/* Send M0 workload */
	pr_info("%s: Send M0 workload to addr 0x%08x\n", __func__,
		WORKLOAD_ADDR);
	err = dma_send_fw(M0_WORKLOAD_PATH, WORKLOAD_ADDR);
	if (err) {
		pr_err("%s: Error during M0 workload transfer: %d\n",
		       __func__, err);
		return err;
	}

	return err;
}

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
