/*
 * Client driver for the Paintbox programmable IPU
 *
 * Copyright (C) 2015 Google, Inc.
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

#include <linux/ab-dram.h>
#include <linux/completion.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <uapi/ipu.h>

#include "ipu-aon-debug.h"
#include "ipu-apb-debug.h"
#include "ipu-bif-debug.h"
#include "ipu-buffer.h"
#include "ipu-client.h"
#include "ipu-debug.h"
#include "ipu-dma.h"
#include "ipu-lbp.h"
#include "ipu-power.h"
#include "ipu-queue.h"
#include "ipu-regs.h"
#include "ipu-resource.h"
#include "ipu-stp.h"

#define JQS_STARTUP_LATENCY_US (10 * 1000)
#define JQS_AUTO_SUSPEND_TIMEOUT_MS 250

int ipu_jqs_send_sync_message(struct paintbox_data *pb,
		const struct jqs_message *req)
{
	struct jqs_message_ack rsp;
	int ret;

	ret = ipu_kernel_write_sync(pb->dev, req, (struct jqs_message *)&rsp,
			sizeof(rsp));
	if (ret < 0) {
		dev_err(pb->dev,
				"%s: error sending message type %u, err = %d\n",
				__func__, req->type, ret);
		return ret;
	}

	if (rsp.header.type != JQS_MESSAGE_TYPE_ACK ||
			rsp.header.size != sizeof(rsp) ||
			rsp.msg_type != req->type) {
		dev_err(pb->dev,
				"%s: protocol error rsp type 0x%08x req 0x%08x rsp 0x%08x sz %u expected %zu\n",
				__func__, rsp.header.type, req->type,
				rsp.msg_type, rsp.header.size, sizeof(rsp));
		return -EPROTO;
	}

	if (rsp.error != JQS_ERROR_NONE) {
		dev_err(pb->dev,
				"%s: jqs reported error 0x%08x for msg 0x%08x\n",
				__func__, rsp.error, rsp.msg_type);
		/* TODO(b/115401717):  Need to implement a JQS error to
		 * errno conversion function.
		 */
		return -EIO;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
static int ipu_jqs_send_open_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct jqs_message_open_session req;
	dma_addr_t buffer_id_table_ab_paddr = ab_dram_get_dma_buf_paddr(
				session->buffer_id_table);

	dev_dbg(pb->dev,
			"%s: session_id %u buffer table jqs addr %pad sz %zu\n",
			__func__, session->session_id,
			&buffer_id_table_ab_paddr,
			session->buffer_id_table->size);

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_OPEN_SESSION);

	req.session_id = session->session_id;
	req.session_memory_addr = ab_dram_get_dma_buf_paddr(
			session->buffer_id_table);
	req.session_memory_bytes = session->buffer_id_table->size;

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

/* The caller to this function must hold pb->lock */
static int ipu_jqs_send_close_session(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct jqs_message_close_session req;

	dev_dbg(pb->dev, "%s: session_id %u\n", __func__, session->session_id);

	INIT_JQS_MSG(req, JQS_MESSAGE_TYPE_CLOSE_SESSION);

	req.session_id = session->session_id;

	return ipu_jqs_send_sync_message(pb, (const struct jqs_message *)&req);
}

static int ipu_client_open(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session;
	struct paintbox_data *pb;
	struct miscdevice *m = fp->private_data;
	int ret;

	pb = container_of(m, struct paintbox_data, misc_device);

	session = kzalloc(sizeof(struct paintbox_session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	session->dev = pb;
	INIT_LIST_HEAD(&session->dma_list);
	INIT_LIST_HEAD(&session->stp_list);
	INIT_LIST_HEAD(&session->lbp_list);
	INIT_LIST_HEAD(&session->wait_list);
	INIT_LIST_HEAD(&session->cmd_queue_list);

	init_completion(&session->bulk_alloc_completion);

	fp->private_data = session;

	mutex_lock(&pb->lock);

	ret = pm_runtime_get_sync(pb->dev);
	if (ret < 0)
		goto free_session;

	/* TODO(b/117150299):  Remove once support for ROM firmware fallback is
	 * removed.  When the ROM firmware fallback is removed the
	 * pm_runtime_get_sync call will return an error if the JQS firmware
	 * could not be loaded.
	 */
	if (!ipu_is_jqs_ready(pb->dev)) {
		dev_err(pb->dev, "%s: JQS firmware not ready\n", __func__);
		ret = -ENOTCONN;
		goto put_runtime;
	}

	ret = ipu_buffer_init_session(pb, session);
	if (ret < 0)
		goto put_runtime;

	ret = idr_alloc(&pb->session_idr, session, 0,
			PAINTBOX_SESSION_ID_MAX, GFP_KERNEL);
	if (ret < 0) {
		dev_err(pb->dev, "%s: error allocating session id, err = %d\n",
				__func__, ret);
		goto free_buffer_table;
	}

	session->session_id = ret;

	pb->session_count++;

	ret = ipu_jqs_send_open_session(pb, session);
	if (ret < 0)
		goto free_session_idr;

	mutex_unlock(&pb->lock);

	return 0;

free_session_idr:
	pb->session_count--;
	idr_remove(&pb->session_idr, session->session_id);
free_buffer_table:
	ipu_buffer_release_session(pb, session);
put_runtime:
	pm_runtime_mark_last_busy(pb->dev);
	pm_runtime_put_autosuspend(pb->dev);
free_session:
	mutex_unlock(&pb->lock);
	kfree(session);

	return ret;
}

static int ipu_client_release(struct inode *ip, struct file *fp)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	int ret;

	mutex_lock(&pb->lock);

	ipu_queue_session_release(pb, session);

	ret = ipu_resource_session_release(pb, session);

	/* TODO(b/115408270):  Handle errors will releasing resources */
	WARN_ON(ret < 0);

	ipu_resource_remove_session_from_wait_list(session);

	if (WARN_ON(--pb->session_count < 0))
		pb->session_count = 0;

	ipu_buffer_release_session(pb, session);

	ipu_jqs_send_close_session(pb, session);
	/* TODO: (b/114760293) errors from close session are not propagated,
	 * this should result in a catastrophic error and a reset of JQS
	 */

	idr_remove(&pb->session_idr, session->session_id);

	/* Release the buffer table for the session after the close session
	 * JQS message has been sent and acknowledged.
	 */
	ab_dram_free_dma_buf_kernel(session->buffer_id_table);

	pm_runtime_mark_last_busy(pb->dev);
	ret = pm_runtime_put_autosuspend(pb->dev);

	mutex_unlock(&pb->lock);

	kfree(session);

	return ret;
}

static long ipu_client_get_capabilities_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct paintbox_pdata *pdata = pb->dev->platform_data;
	struct ipu_capabilities_rsp caps;
	uint32_t version;

	memset(&caps, 0, sizeof(caps));

	caps.num_lbps = pb->lbp.num_lbps;
	caps.num_stps = pb->stp.num_stps;
	caps.num_dma_channels = pb->dma.num_channels;
	caps.num_interrupts = pb->dma.num_channels + pb->stp.num_stps +
			NUM_BIF_INTERRUPTS + NUM_MMU_INTERRUPTS;

	version = ipu_readl(pb->dev, IPU_VERSION);
	caps.version_major = (version & IPU_VERSION_MAJOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	caps.version_minor = (version & IPU_VERSION_MINOR_MASK) >>
			IPU_VERSION_MAJOR_SHIFT;
	caps.version_build = version & IPU_VERSION_INCR_MASK;
	caps.is_fpga = !!(version & IPU_VERSION_FPGA_BUILD_MASK);

#if IS_ENABLED(CONFIG_IPU_SIM_ADAPTER)
	caps.is_simulator = true;
#endif

#if IS_ENABLED(CONFIG_PAINTBOX_AIRBRUSH_IOMMU)
	caps.iommu_enabled = iommu_present(&ipu_bus_type);
#endif

	caps.hardware_id = pdata->hardware_id;

	if (copy_to_user((void __user *)arg, &caps, sizeof(caps)))
		return -EFAULT;

	return 0;
}

static long ipu_client_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct paintbox_session *session = fp->private_data;
	struct paintbox_data *pb = session->dev;
	int ret;

	switch (cmd) {
	/* TODO(b/115407896):  Migrate the IPU Runtime to a IPU_* capabilities
	 * ioctl.
	 */
	case IPU_GET_CAPABILITIES:
		ret = ipu_client_get_capabilities_ioctl(pb, session, arg);
		break;
	case IPU_ALLOCATE_CMD_QUEUE:
		ret = ipu_queue_allocate_ioctl(pb, session, arg);
		break;
	case IPU_ALLOCATE_RESOURCES:
		ret = ipu_resource_allocate_ioctl(pb, session, arg);
		break;
	case IPU_RELEASE_RESOURCES:
		ret = ipu_resource_release_ioctl(pb, session, arg);
		break;
	case IPU_REGISTER_DMA_BUF:
		ret = ipu_buffer_dma_buf_register_ioctl(pb, session, arg);
		break;
	case IPU_UNREGISTER_DMA_BUF:
		ret = ipu_buffer_dma_buf_unregister_ioctl(pb, session,
				arg);
		break;
	default:
		dev_err(pb->dev, "%s: unknown ioctl 0x%0x\n", __func__, cmd);
		return -EINVAL;
	}

	return ret;
}

static const struct file_operations ipu_client_fops = {
	.owner = THIS_MODULE,
	.open = ipu_client_open,
	.release = ipu_client_release,
	.unlocked_ioctl = ipu_client_ioctl,
};

static void ipu_client_get_capabilities(struct paintbox_data *pb)
{
	struct paintbox_pdata *pdata = pb->dev->platform_data;
	uint8_t major, minor, build;
	uint32_t val;
	bool is_fpga;

	val = ipu_readl(pb->dev, IPU_VERSION);
	major = (val & IPU_VERSION_MAJOR_MASK) >> IPU_VERSION_MAJOR_SHIFT;
	minor = (val & IPU_VERSION_MINOR_MASK) >> IPU_VERSION_MAJOR_SHIFT;
	build = val & IPU_VERSION_INCR_MASK;
	is_fpga = !!(val & IPU_VERSION_FPGA_BUILD_MASK);

	val = ipu_readl(pb->dev, IPU_CAP);
	pb->stp.num_stps = val & IPU_CAP_NUM_STP_MASK;

	/* STP ids start at 1. */
	pb->stp.available_stp_mask = ((1ULL << pb->stp.num_stps) - 1) << 1;
	pb->lbp.num_lbps = (val & IPU_CAP_NUM_LBP_MASK) >>
		IPU_CAP_NUM_LBP_SHIFT;
	pb->lbp.available_lbp_mask = (1ULL << pb->lbp.num_lbps) - 1;
	pb->dma.num_channels = (val & IPU_CAP_NUM_DMA_CHAN_MASK) >>
			IPU_CAP_NUM_DMA_CHAN_SHIFT;
	pb->dma.available_channel_mask = (1ULL << pb->dma.num_channels) - 1;

#if IS_ENABLED(CONFIG_IPU_SIM_ADAPTER)
	dev_dbg(pb->dev,
			"Paintbox IPU Version %u.%u.%u Simulator Hardware ID %u\n",
			major, minor, build, pdata->hardware_id);
#else
	dev_dbg(pb->dev, "Paintbox IPU Version %u.%u.%u %s Hardware ID %u\n",
			major, minor, build, is_fpga ? "FPGA" : "",
			pdata->hardware_id);
#endif
	dev_dbg(pb->dev,
			"STPs %u LBPs %u DMA Channels %u\n",
			pb->stp.num_stps, pb->lbp.num_lbps,
			pb->dma.num_channels);
}

static void ipu_client_firmware_down(struct device *dev)
{
	struct paintbox_data *pb = dev_get_drvdata(dev);

	dev_dbg(pb->dev, "JQS firmware is going down\n");

	/* TODO(b/114760293):  This needs to handle a reset notification in the
	 * middle of a job.  Right now this will only work when the device is
	 * quiescent.
	 */
}

static void ipu_client_firmware_up(struct device *dev)
{
	struct paintbox_data *pb = dev_get_drvdata(dev);

	dev_dbg(pb->dev, "JQS firmware is ready\n");

	/* TODO(b/114760293):  This needs to handle a reset notification in the
	 * middle of a job.  Right now this will only work when the device is
	 * quiescent.
	 */
}

static const struct paintbox_device_ops ipu_client_dev_ops = {
	.firmware_up = ipu_client_firmware_up,
	.firmware_down = ipu_client_firmware_down,
};

static int ipu_client_probe(struct device *dev)
{
	struct paintbox_data *pb;
	int ret;

	pb = devm_kzalloc(dev, sizeof(*pb), GFP_KERNEL);
	if (pb == NULL)
		return -ENOMEM;

	pb->dev = dev;

	idr_init(&pb->session_idr);

	dev_set_drvdata(dev, pb);

	mutex_init(&pb->lock);

	ipu_set_device_ops(dev, &ipu_client_dev_ops);

	ipu_debug_init(pb);

	pb->dma_dev = ipu_get_dma_device(pb->dev);

	ipu_client_get_capabilities(pb);
	ipu_power_init(pb);
	ipu_aon_debug_init(pb);
	ipu_apb_debug_init(pb);
	ipu_bif_debug_init(pb);

	ret = ipu_dma_init(pb);
	if (ret < 0)
		return ret;

	ret = ipu_lbp_init(pb);
	if (ret < 0)
		return ret;

	ret = ipu_stp_init(pb);
	if (ret < 0)
		return ret;

	/* register the misc device */
	pb->misc_device.minor = MISC_DYNAMIC_MINOR,
	pb->misc_device.name  = "ipu",
	pb->misc_device.fops  = &ipu_client_fops,

	INIT_LIST_HEAD(&pb->bulk_alloc_waiting_list);

	ret = misc_register(&pb->misc_device);
	if (ret < 0) {
		pr_err("Failed to register misc device node (ret = %d)", ret);
		return ret;
	}

	ret = dev_pm_qos_add_request(dev, &pb->pm_qos,
			DEV_PM_QOS_RESUME_LATENCY, JQS_STARTUP_LATENCY_US);
	if (ret < 0) {
		dev_err(pb->dev, "%s: Unable to configure pm qos, ret %d\n",
				__func__, ret);
		return ret;
	}

	pm_runtime_set_suspended(pb->dev);
	pm_runtime_set_autosuspend_delay(pb->dev, JQS_AUTO_SUSPEND_TIMEOUT_MS);
	pm_runtime_use_autosuspend(pb->dev);
	pm_runtime_enable(pb->dev);

	return 0;
}

static int ipu_client_remove(struct device *dev)
{
	struct paintbox_data *pb = dev_get_drvdata(dev);

	pm_runtime_disable(pb->dev);

	if (dev_pm_qos_request_active(&pb->pm_qos))
		dev_pm_qos_remove_request(&pb->pm_qos);

	misc_deregister(&pb->misc_device);
	ipu_aon_debug_remove(pb);
	ipu_apb_debug_remove(pb);
	ipu_bif_debug_remove(pb);
	ipu_lbp_remove(pb);
	ipu_stp_remove(pb);
	ipu_dma_remove(pb);

	ipu_debug_remove(pb);

	idr_destroy(&pb->session_idr);

	mutex_destroy(&pb->lock);

	return 0;
}

static struct device_driver ipu_client_driver = {
	.name	= "paintbox-ipu",
	.bus	= &ipu_bus_type,
	.probe	= ipu_client_probe,
	.remove	= ipu_client_remove,
};

int ipu_client_init(void)
{
	return driver_register(&ipu_client_driver);
}

static void __exit ipu_client_exit(void)
{
	driver_unregister(&ipu_client_driver);
}

subsys_initcall(ipu_client_init);
module_exit(ipu_client_exit);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPU Client Driver");
