/*
 * Android Airbrush coprocessor DMA library
 *
 * Copyright 2019 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <uapi/abc-pcie-dma.h>
#include "abc-pcie-dma.h"

int abc_pcie_dma_open(struct inode *inode, struct file *filp)
{
	/* TODO(alexperez): Retrieve file->private_data, use container_of
	 *                  to get to abc_pcie_dma struct, store session_id,
	 *                  replace file->private_data with
	 *                  abc_pcie_dma_struct.
	 */
	return 0;
}

int abc_pcie_dma_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* IOCTL interface */
long abc_pcie_dma_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct abc_pcie_dma_desc dma_desc;
	struct device *dev;
	int err = 0;
	struct miscdevice *mdev = (struct miscdevice *)file->private_data;

	dev = mdev->parent;

	if (_IOC_TYPE(cmd) != ABC_PCIE_DMA_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_LEGACY: {
		struct abc_pcie_dma_desc_legacy dma_desc_legacy;

		dev_dbg(dev,
			"%s: Received IOCTL for legacy DMA request\n",
			__func__);
		err = copy_from_user(&dma_desc_legacy, (void __user *)arg,
				     sizeof(dma_desc_legacy));
		if (err) {
			dev_err(dev,
				"%s: failed to copy from userspace (%d)\n",
				__func__, err);
			return err;
		}
		dma_desc.local_buf_type = dma_desc_legacy.local_buf_type;
		/* abc_pcie_issue_dma_xfer will catch invalid buffer types */
		if (dma_desc.local_buf_type == DMA_BUFFER_USER)
			dma_desc.local_buf = dma_desc_legacy.local_buf;
		else {
			dma_desc.local_dma_buf_fd =
				dma_desc_legacy.local_dma_buf_fd;
			dma_desc.local_dma_buf_off = 0;
		}
		dma_desc.remote_buf_type = dma_desc_legacy.remote_buf_type;
		if (dma_desc.remote_buf_type == DMA_BUFFER_USER)
			dma_desc.remote_buf = dma_desc_legacy.remote_buf;
		else {
			dma_desc.remote_dma_buf_fd =
				dma_desc_legacy.remote_dma_buf_fd;
			dma_desc.remote_dma_buf_off = 0;
		}
		dma_desc.size = dma_desc_legacy.local_buf_size;
		dma_desc.dir = dma_desc_legacy.dir;
		err = abc_pcie_issue_dma_xfer(&dma_desc);
		if (err) {
			dev_err(dev,
				"%s: failed to perform DMA (%d)\n",
				__func__, err);
			return err;
		}
		break;
	}
	case ABC_PCIE_DMA_IOC_POST_DMA_XFER_SYNC:
		dev_dbg(dev,
			"%s: Received IOCTL for synchronous DMA request\n",
			__func__);
		err = copy_from_user(&dma_desc, (void __user *)arg,
				     sizeof(dma_desc));
		if (err) {
			dev_err(dev,
				"%s: failed to copy from userspace (%d)\n",
				__func__, err);
			return err;
		}
		err = abc_pcie_issue_dma_xfer(&dma_desc);
		if (err) {
			dev_err(dev,
				"%s: failed to perform DMA (%d)\n",
				__func__, err);
			return err;
		}
		break;
	default:
		dev_err(dev,
			"%s: unknown ioctl %c, dir=%d, #%d (0x%08x)\n",
			__func__, _IOC_TYPE(cmd), _IOC_DIR(cmd), _IOC_NR(cmd),
			cmd);
		break;
	}

	return err;
}

static const struct file_operations abc_dma_fops = {
	.owner = THIS_MODULE,
	.open = abc_pcie_dma_open,
	.release = abc_pcie_dma_release,
	.unlocked_ioctl = abc_pcie_dma_ioctl,
	.llseek = no_llseek,
};

int init_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi)
{
	int err = 0;

	uapi->mdev.minor = MISC_DYNAMIC_MINOR;
	uapi->mdev.name = DRV_NAME_ABC_PCIE_DMA;
	uapi->mdev.fops = &abc_dma_fops;
	err = misc_register(&uapi->mdev);
	if (err) {
		dev_err((uapi->mdev).parent, "misc_register failed\n");
		return err;
	}

	return 0;
}

void remove_abc_pcie_dma_uapi(struct abc_pcie_dma_uapi *uapi)
{
	misc_deregister(&uapi->mdev);
}
