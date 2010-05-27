/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/android_pmem.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <mach/clk.h>

#include "vcd_api.h"
#include "video_core_init_internal.h"
#include "video_core_init.h"

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define VID_C_NAME "msm_vidc_reg"

#define ERR(x...) printk(KERN_ERR x)

static struct vid_c_dev *vid_c_device_p;
static dev_t vid_c_dev_num;
static struct class *vid_c_class;

static const struct file_operations vid_c_fops = {
	.owner = THIS_MODULE,
	.open = NULL,
	.release = NULL,
	.ioctl = NULL,
};

struct workqueue_struct *vid_c_wq;
struct workqueue_struct *vidc_timer_wq;
static irqreturn_t vid_c_isr(int irq, void *dev);
static spinlock_t vidc_spin_lock;

#define VIDC_BOOT_FW			"vidc_720p_command_control.fw"
#define VIDC_MPG4_DEC_FW		"vidc_720p_mp4_dec_mc.fw"
#define VIDC_H263_DEC_FW		"vidc_720p_h263_dec_mc.fw"
#define VIDC_H264_DEC_FW		"vidc_720p_h264_dec_mc.fw"
#define VIDC_MPG4_ENC_FW		"vidc_720p_mp4_enc_mc.fw"
#define VIDC_H264_ENC_FW		"vidc_720p_h264_enc_mc.fw"

static void vid_c_timer_fn(unsigned long data)
{
	unsigned long flag;
	struct vid_c_timer *hw_timer = NULL;

	DBG("%s() Timer expired \n", __func__);
	spin_lock_irqsave(&vidc_spin_lock, flag);
	hw_timer = (struct vid_c_timer *)data;
	list_add_tail(&hw_timer->list, &vid_c_device_p->vidc_timer_queue);
	spin_unlock_irqrestore(&vidc_spin_lock, flag);
	DBG("Queue the work for timer \n");
	queue_work(vidc_timer_wq, &vid_c_device_p->vidc_timer_worker);
}

static void vid_c_timer_handler(struct work_struct *work)
{
	unsigned long flag = 0;
	u32 islist_empty = 0;
	struct vid_c_timer *hw_timer = NULL;

	DBG("%s() Timer expired \n", __func__);
	do {
		spin_lock_irqsave(&vidc_spin_lock, flag);
		islist_empty = list_empty(&vid_c_device_p->vidc_timer_queue);
		if (!islist_empty) {
			hw_timer = list_first_entry(
				&vid_c_device_p->vidc_timer_queue,
				struct vid_c_timer, list);
			list_del(&hw_timer->list);
		}
		spin_unlock_irqrestore(&vidc_spin_lock, flag);
		if (!islist_empty && hw_timer && hw_timer->cb_func)
			hw_timer->cb_func(hw_timer->userdata);
	} while (!islist_empty);
}

static void vid_c_work_handler(struct work_struct *work)
{
	DBG("vid_c_work_handler()");
	vcd_read_and_clear_interrupt();
	vcd_response_handler();
	enable_irq(vid_c_device_p->irq);
	DBG("vid_c_work_handler() done");
}

static DECLARE_WORK(vid_c_work, vid_c_work_handler);

static int __init vid_c_720p_probe(struct platform_device *pdev)
{
	struct resource *resource;
	DBG("Enter %s()\n", __func__);

	if (pdev->id) {
		ERR("Invalid plaform device ID = %d\n", pdev->id);
		return -EINVAL;
	}
	vid_c_device_p->irq = platform_get_irq(pdev, 0);
	if (unlikely(vid_c_device_p->irq < 0)) {
		ERR("%s(): Invalid irq = %d\n", __func__,
					 vid_c_device_p->irq);
		return -ENXIO;
	}

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource)) {
		ERR("%s(): Invalid resource \n", __func__);
		return -ENXIO;
	}

	vid_c_device_p->phys_base = resource->start;
	vid_c_device_p->virt_base = ioremap(resource->start,
	resource->end - resource->start + 1);

	if (!vid_c_device_p->virt_base) {
		ERR("%s() : ioremap failed\n", __func__);
		return -ENOMEM;
	}
	vid_c_device_p->device = &pdev->dev;
	mutex_init(&vid_c_device_p->lock);

	vid_c_wq = create_singlethread_workqueue("vid_c_worker_queue");
	if (!vid_c_wq) {
		ERR("%s: create workque failed \n", __func__);
		return -ENOMEM;
	}

	return 0;
}

static int __devexit vid_c_720p_remove(struct platform_device *pdev)
{
	if (pdev->id) {
		ERR("Invalid plaform device ID = %d\n", pdev->id);
		return -EINVAL;
	}
	return 0;
}


static struct platform_driver msm_vid_c_720p_platform_driver = {
	.probe = vid_c_720p_probe,
	.remove = vid_c_720p_remove,
	.driver = {
				.name = "msm_vidc_720p",
	},
};

static void __exit vid_c_exit(void)
{
	platform_driver_unregister(&msm_vid_c_720p_platform_driver);
}

static irqreturn_t vid_c_isr(int irq, void *dev)
{
	DBG("\n vid_c_isr() %d ", irq);
	disable_irq_nosync(irq);
	queue_work(vid_c_wq, &vid_c_work);
	return IRQ_HANDLED;
}

static int __init vid_c_init(void)
{
	int rc = 0;
	struct device *class_devp;

	vid_c_device_p = kzalloc(sizeof(struct vid_c_dev), GFP_KERNEL);
	if (!vid_c_device_p) {
		ERR("%s Unable to allocate memory for vid_c_dev\n",
			__func__);
		return -ENOMEM;
	}

	rc = alloc_chrdev_region(&vid_c_dev_num, 0, 1, VID_C_NAME);
	if (rc < 0) {
		ERR("%s: alloc_chrdev_region Failed rc = %d\n",
			__func__, rc);
		goto error_vid_c_alloc_chrdev_region;
	}

	vid_c_class = class_create(THIS_MODULE, VID_C_NAME);
	if (IS_ERR(vid_c_class)) {
		rc = PTR_ERR(vid_c_class);
		ERR("%s: couldn't create vid_c_class rc = %d\n",
		__func__, rc);

		goto error_vid_c_class_create;
	}

	class_devp = device_create(vid_c_class, NULL, vid_c_dev_num, NULL,
					VID_C_NAME);

	if (IS_ERR(class_devp)) {
		rc = PTR_ERR(class_devp);
		ERR("%s: class device_create failed %d\n",
			__func__, rc);
		goto error_vid_c_class_device_create;
	}

	cdev_init(&vid_c_device_p->cdev, &vid_c_fops);
	vid_c_device_p->cdev.owner = THIS_MODULE;
	rc = cdev_add(&(vid_c_device_p->cdev), vid_c_dev_num, 1);

	if (rc < 0) {
		ERR("%s: cdev_add failed %d\n", __func__, rc);
		goto error_vid_c_cdev_add;
	}

	rc = platform_driver_register(&msm_vid_c_720p_platform_driver);
	if (rc) {
		ERR("%s failed to load\n", __func__);
		goto error_vid_c_platfom_register;
	}

	rc = request_irq(vid_c_device_p->irq, vid_c_isr, IRQF_TRIGGER_HIGH,
			 "vid_c", vid_c_device_p->device);

	if (unlikely(rc)) {
		ERR("%s() :request_irq failed\n", __func__);
		goto error_vid_c_platfom_register;
	}

	vidc_timer_wq = create_singlethread_workqueue("vidc_timer_wq");
	if (!vidc_timer_wq) {
		ERR("%s: create workque failed \n", __func__);
		rc = -ENOMEM;
		goto error_vid_c_platfom_register;
	}

	DBG("Disabling IRQ in %s()\n", __func__);
	disable_irq_nosync(vid_c_device_p->irq);
	INIT_WORK(&vid_c_device_p->vidc_timer_worker,
			  vid_c_timer_handler);
	spin_lock_init(&vidc_spin_lock);
	INIT_LIST_HEAD(&vid_c_device_p->vidc_timer_queue);
	vid_c_device_p->clock_enabled = 0;
	vid_c_device_p->ref_count = 0;
	vid_c_device_p->firmware_refcount = 0;
	vid_c_device_p->get_firmware = 0;

	return 0;

error_vid_c_platfom_register:
	cdev_del(&(vid_c_device_p->cdev));
error_vid_c_cdev_add:
	device_destroy(vid_c_class, vid_c_dev_num);
error_vid_c_class_device_create:
	class_destroy(vid_c_class);
error_vid_c_class_create:
	unregister_chrdev_region(vid_c_dev_num, 1);
error_vid_c_alloc_chrdev_region:
	kfree(vid_c_device_p);

	return rc;
}

void __iomem *vid_c_get_ioaddr(void)
{
	return (u8 *)vid_c_device_p->virt_base;
}
EXPORT_SYMBOL(vid_c_get_ioaddr);
#ifdef USE_RES_TRACKER

u32 vid_c_enable_pwr_rail(void)
{
	int rc = -1;
	mutex_lock(&vid_c_device_p->lock);

	if (!vid_c_device_p->rail_enabled) {
		//TODO: internal_pwr_rail_mode(MFC_CLK_ID, MANUAL)

		vid_c_device_p->pclk = clk_get(vid_c_device_p->device,
			"mfc_pclk");

		if (IS_ERR(vid_c_device_p->pclk)) {
			ERR("%s(): mfc_pclk get failed \n", __func__);

			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		vid_c_device_p->hclk = clk_get(vid_c_device_p->device,
			"mfc_clk");

		if (IS_ERR(vid_c_device_p->hclk)) {
			ERR("%s(): mfc_clk get failed \n", __func__);

			clk_put(vid_c_device_p->pclk);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		vid_c_device_p->hclk_div2 =
			clk_get(vid_c_device_p->device, "mfc_div2_clk");

		if (IS_ERR(vid_c_device_p->pclk)) {
			ERR("%s(): mfc_div2_clk get failed \n", __func__);

			clk_put(vid_c_device_p->pclk);
			clk_put(vid_c_device_p->hclk);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		//TODO: internal_pwr_rail_ctl(MFC_CLK_ID, 1)
		msleep(20);

		rc = clk_reset(vid_c_device_p->pclk, CLK_RESET_DEASSERT);
		if (rc) {
			ERR("\n clk_reset failed %d\n", rc);
			return FALSE;
		}
		msleep(20);
	}
	vid_c_device_p->rail_enabled = 1;
	mutex_unlock(&vid_c_device_p->lock);
	return TRUE;
}
EXPORT_SYMBOL(vid_c_enable_pwr_rail);

u32 vid_c_disable_pwr_rail(void)
{
	int rc = -1;
	mutex_lock(&vid_c_device_p->lock);

	if (vid_c_device_p->clock_enabled) {
		mutex_unlock(&vid_c_device_p->lock);
		DBG("\n Calling CLK disable in Power Down \n");
		vid_c_disable_clk();
		mutex_lock(&vid_c_device_p->lock);
	}

	if (!vid_c_device_p->rail_enabled) {
		mutex_unlock(&vid_c_device_p->lock);
		return FALSE;
	}

	vid_c_device_p->rail_enabled = 0;
	rc = clk_reset(vid_c_device_p->pclk, CLK_RESET_ASSERT);
	if (rc) {
		ERR("\n clk_reset failed %d\n", rc);
		mutex_unlock(&vid_c_device_p->lock);
		return FALSE;
	}
	msleep(20);

	//TODO: internal_pwr_rail_ctl(MFC_CLK_ID, 0)

	clk_put(vid_c_device_p->hclk_div2);
	clk_put(vid_c_device_p->hclk);
	clk_put(vid_c_device_p->pclk);

	mutex_unlock(&vid_c_device_p->lock);

	return TRUE;
}
EXPORT_SYMBOL(vid_c_disable_pwr_rail);

u32 vid_c_enable_clk(void)
{
	mutex_lock(&vid_c_device_p->lock);

	if (!vid_c_device_p->clock_enabled) {
		DBG("Enabling IRQ in %s()\n", __func__);
		enable_irq(vid_c_device_p->irq);

		DBG("%s(): Enabling the clocks ...\n", __func__);

		if (clk_enable(vid_c_device_p->pclk)) {
			ERR("vidc pclk Enable failed \n");

			clk_put(vid_c_device_p->hclk);
			clk_put(vid_c_device_p->hclk_div2);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		if (clk_enable(vid_c_device_p->hclk)) {
			ERR("vidc  hclk Enable failed \n");
			clk_put(vid_c_device_p->pclk);
			clk_put(vid_c_device_p->hclk_div2);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		if (clk_enable(vid_c_device_p->hclk_div2)) {
			ERR("vidc  hclk Enable failed \n");
			clk_put(vid_c_device_p->hclk);
			clk_put(vid_c_device_p->pclk);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}
	}

	vid_c_device_p->clock_enabled = 1;
	mutex_unlock(&vid_c_device_p->lock);
	return TRUE;
}
EXPORT_SYMBOL(vid_c_enable_clk);

u32 vid_c_sel_clk_rate(unsigned long hclk_rate)
{
	mutex_lock(&vid_c_device_p->lock);
	if (clk_set_rate(vid_c_device_p->hclk,
		hclk_rate)) {
		ERR("vidc hclk set rate failed \n");
		mutex_unlock(&vid_c_device_p->lock);
		return FALSE;
	}
	vid_c_device_p->hclk_rate = hclk_rate;
	mutex_unlock(&vid_c_device_p->lock);
	return TRUE;
}
EXPORT_SYMBOL(vid_c_sel_clk_rate);

u32 vid_c_get_clk_rate(unsigned long *phclk_rate)
{
	if (!phclk_rate) {
		ERR("vid_c_get_clk_rate(): phclk_rate is NULL\n");
		return FALSE;
	}
	mutex_lock(&vid_c_device_p->lock);
	*phclk_rate = clk_get_rate(vid_c_device_p->hclk);
	if (!(*phclk_rate)) {
		ERR("vidc hclk get rate failed \n");
		mutex_unlock(&vid_c_device_p->lock);
		return FALSE;
	}
	mutex_unlock(&vid_c_device_p->lock);
	return TRUE;
}
EXPORT_SYMBOL(vid_c_get_clk_rate);

u32 vid_c_disable_clk(void)
{
	mutex_lock(&vid_c_device_p->lock);

	if (!vid_c_device_p->clock_enabled) {
		mutex_unlock(&vid_c_device_p->lock);
		return FALSE;
	}

	DBG("Disabling IRQ in %s()\n", __func__);
	disable_irq_nosync(vid_c_device_p->irq);
	DBG("%s(): Disabling the clocks ...\n", __func__);

	vid_c_device_p->clock_enabled = 0;
	clk_disable(vid_c_device_p->hclk);
	clk_disable(vid_c_device_p->hclk_div2);
	clk_disable(vid_c_device_p->pclk);

	mutex_unlock(&vid_c_device_p->lock);

	return TRUE;
}
EXPORT_SYMBOL(vid_c_disable_clk);

#else

u32 vid_c_enable_clk(unsigned long hclk_rate)
{
	int rc = -1;
	mutex_lock(&vid_c_device_p->lock);
	vid_c_device_p->ref_count++;

	if (!vid_c_device_p->clock_enabled) {
		DBG("Enabling IRQ in %s()\n", __func__);
		enable_irq(vid_c_device_p->irq);

		rc = internal_pwr_rail_mode
			(PWR_RAIL_MFC_CLK, PWR_RAIL_CTL_MANUAL);
		if (rc) {
			ERR("%s(): internal_pwr_rail_mode failed %d \n",
			__func__, rc);
			return FALSE;
		}
		DBG("%s(): internal_pwr_rail_mode Success %d \n",
		__func__, rc);

		vid_c_device_p->pclk =
			clk_get(vid_c_device_p->device, "mfc_pclk");

		if (IS_ERR(vid_c_device_p->pclk)) {
			ERR("%s(): mfc_pclk get failed \n", __func__);

			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		vid_c_device_p->hclk =
			clk_get(vid_c_device_p->device, "mfc_clk");

		if (IS_ERR(vid_c_device_p->hclk)) {
			ERR("%s(): mfc_clk get failed \n", __func__);

			clk_put(vid_c_device_p->pclk);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		vid_c_device_p->hclk_div2 =
			clk_get(vid_c_device_p->device, "mfc_div2_clk");

		if (IS_ERR(vid_c_device_p->pclk)) {
			ERR("%s(): mfc_div2_clk get failed \n", __func__);

			clk_put(vid_c_device_p->pclk);
			clk_put(vid_c_device_p->hclk);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		vid_c_device_p->hclk_rate = hclk_rate;

		if (clk_set_rate(vid_c_device_p->hclk,
			vid_c_device_p->hclk_rate)) {
			ERR("vid_c hclk set rate failed \n");
			clk_put(vid_c_device_p->pclk);
			clk_put(vid_c_device_p->hclk);
			clk_put(vid_c_device_p->hclk_div2);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		if (clk_enable(vid_c_device_p->pclk)) {
			ERR("vid_c pclk Enable failed \n");

			clk_put(vid_c_device_p->hclk);
			clk_put(vid_c_device_p->hclk_div2);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		if (clk_enable(vid_c_device_p->hclk)) {
			ERR("vid_c  hclk Enable failed \n");
			clk_put(vid_c_device_p->pclk);
			clk_put(vid_c_device_p->hclk_div2);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}

		if (clk_enable(vid_c_device_p->hclk_div2)) {
			ERR("vid_c  hclk Enable failed \n");
			clk_put(vid_c_device_p->hclk);
			clk_put(vid_c_device_p->pclk);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}
		msleep(20);
		rc = internal_pwr_rail_ctl(PWR_RAIL_MFC_CLK, 1);
		if (rc) {
			ERR("\n internal_pwr_rail_ctl failed %d\n", rc);
			return FALSE;
		}
		DBG("%s(): internal_pwr_rail_ctl Success %d \n",
			__func__, rc);
		msleep(20);
		rc = clk_reset(vid_c_device_p->pclk, CLK_RESET_DEASSERT);
		if (rc) {
			ERR("\n clk_reset failed %d\n", rc);
			return FALSE;
		}
		msleep(20);
	}
	vid_c_device_p->clock_enabled = 1;
	mutex_unlock(&vid_c_device_p->lock);
	return TRUE;
}
EXPORT_SYMBOL(vid_c_enable_clk);

u32 vid_c_disable_clk(void)
{
	int rc = -1;
	mutex_lock(&vid_c_device_p->lock);

	if (!vid_c_device_p->ref_count ||
		!vid_c_device_p->clock_enabled) {
		return FALSE;
	}

	if (vid_c_device_p->ref_count > 0)
		vid_c_device_p->ref_count--;

	if (!vid_c_device_p->ref_count) {
		DBG("Disabling IRQ in %s()\n", __func__);
		disable_irq_nosync(vid_c_device_p->irq);
		rc = clk_reset(vid_c_device_p->pclk, CLK_RESET_ASSERT);
		if (rc) {
			ERR("\n clk_reset failed %d\n", rc);
			return FALSE;
		}
		msleep(20);

		rc = internal_pwr_rail_ctl(PWR_RAIL_MFC_CLK, 0);
		if (rc) {
			ERR("\n internal_pwr_rail_ctl failed %d\n", rc);
			return FALSE;
		}

		vid_c_device_p->clock_enabled = 0;
		clk_disable(vid_c_device_p->hclk);
		clk_disable(vid_c_device_p->hclk_div2);
		clk_disable(vid_c_device_p->pclk);

		clk_put(vid_c_device_p->hclk_div2);
		clk_put(vid_c_device_p->hclk);
		clk_put(vid_c_device_p->pclk);

	}
	mutex_unlock(&vid_c_device_p->lock);
	return TRUE;
}
EXPORT_SYMBOL(vid_c_disable_clk);

#endif
unsigned char *vid_c_command_control_fw;
u32 vid_c_command_control_fw_size;

unsigned char *vid_c_mpg4_dec_fw;
u32 vid_c_mpg4_dec_fw_size;

unsigned char *vid_c_h263_dec_fw;
u32 vid_c_h263_dec_fw_size;

unsigned char *vid_c_h264_dec_fw;
u32 vid_c_h264_dec_fw_size;

unsigned char *vid_c_mpg4_enc_fw;
u32 vid_c_mpg4_enc_fw_size;


unsigned char *vid_c_h264_enc_fw;
u32 vid_c_h264_enc_fw_size;


int vid_c_load_firmware(void)
{
	int rc = 0;
	const struct firmware *fw_boot = NULL;
	const struct firmware *fw_mpg4_dec = NULL;
	const struct firmware *fw_h263_dec = NULL;
	const struct firmware *fw_h264_dec = NULL;
	const struct firmware *fw_mpg4_enc = NULL;
	const struct firmware *fw_h264_enc = NULL;

	u32 status = TRUE;

	mutex_lock(&vid_c_device_p->lock);

	if (!vid_c_device_p->get_firmware) {
		rc = request_firmware(&fw_boot,
			VIDC_BOOT_FW, vid_c_device_p->device);
		if (rc) {
			ERR("request_firmware for %s failed with error %d\n",
					VIDC_BOOT_FW, rc);
			mutex_unlock(&vid_c_device_p->lock);
			return FALSE;
		}
		vid_c_command_control_fw = (unsigned char *)fw_boot->data;
		vid_c_command_control_fw_size = (u32) fw_boot->size;

		rc = request_firmware(&fw_mpg4_dec, VIDC_MPG4_DEC_FW,
			vid_c_device_p->device);
		if (rc) {
			ERR("request_firmware for %s failed with error %d\n",
					VIDC_BOOT_FW, rc);
			status = FALSE;
			goto boot_fw_free;
		}
		vid_c_mpg4_dec_fw = (unsigned char *)fw_mpg4_dec->data;
		vid_c_mpg4_dec_fw_size = (u32) fw_mpg4_dec->size;


		rc = request_firmware(&fw_h263_dec, VIDC_H263_DEC_FW,
			vid_c_device_p->device);
		if (rc) {
			ERR("request_firmware for %s failed with error %d\n",
					VIDC_BOOT_FW, rc);
			status = FALSE;
			goto mp4dec_fw_free;
		}
		vid_c_h263_dec_fw = (unsigned char *)fw_h263_dec->data;
		vid_c_h263_dec_fw_size = (u32) fw_h263_dec->size;

		rc = request_firmware(&fw_h264_dec, VIDC_H264_DEC_FW,
			vid_c_device_p->device);
		if (rc) {
			ERR("request_firmware for %s failed with error %d\n",
					VIDC_BOOT_FW, rc);
			status = FALSE;
			goto h263dec_fw_free;
		}
		vid_c_h264_dec_fw = (unsigned char *)fw_h264_dec->data;
		vid_c_h264_dec_fw_size = (u32) fw_h264_dec->size;

		rc = request_firmware(&fw_mpg4_enc, VIDC_MPG4_ENC_FW,
			vid_c_device_p->device);
		if (rc) {
			ERR("request_firmware for %s failed with error %d\n",
					VIDC_BOOT_FW, rc);
			status = FALSE;
			goto h264dec_fw_free;
		}
		vid_c_mpg4_enc_fw = (unsigned char *)fw_mpg4_enc->data;
		vid_c_mpg4_enc_fw_size = (u32) fw_mpg4_enc->size;

		rc = request_firmware(&fw_h264_enc, VIDC_H264_ENC_FW,
			vid_c_device_p->device);
		if (rc) {
			ERR("request_firmware for %s failed with error %d\n",
					VIDC_BOOT_FW, rc);
			status = FALSE;
			goto mp4enc_fw_free;
		}
		vid_c_h264_enc_fw = (unsigned char *)fw_h264_enc->data;
		vid_c_h264_enc_fw_size = (u32) fw_h264_enc->size;
		vid_c_device_p->get_firmware = 1;
	}

	vid_c_device_p->firmware_refcount++;

	mutex_unlock(&vid_c_device_p->lock);
	return status;

	release_firmware(fw_h264_enc);
mp4enc_fw_free:
	release_firmware(fw_mpg4_enc);
h264dec_fw_free:
	release_firmware(fw_h264_dec);
h263dec_fw_free:
	release_firmware(fw_h263_dec);
mp4dec_fw_free:
	release_firmware(fw_mpg4_dec);
boot_fw_free:
	release_firmware(fw_boot);
	mutex_unlock(&vid_c_device_p->lock);
	return FALSE;
}
EXPORT_SYMBOL(vid_c_load_firmware);

void vid_c_release_firmware(void)
{
	mutex_lock(&vid_c_device_p->lock);
	if (vid_c_device_p->firmware_refcount > 0)
		vid_c_device_p->firmware_refcount--;
	else
		vid_c_device_p->firmware_refcount = 0;
	mutex_unlock(&vid_c_device_p->lock);
}
EXPORT_SYMBOL(vid_c_release_firmware);

u32 vid_c_lookup_addr_table(struct video_client_ctx *client_ctx,
	enum buffer_dir buffer_type,
	u32 search_with_user_vaddr,
	unsigned long *user_vaddr,
	unsigned long *kernel_vaddr,
	unsigned long *phy_addr, int *pmem_fd,
	struct file **file, s32 *buffer_index)
{
	u32 num_of_buffers;
	u32 i;
	struct buf_addr_table *buf_addr_table;
	u32 found = FALSE;

	if (!client_ctx)
		return FALSE;

	if (buffer_type == BUFFER_TYPE_INPUT) {
		buf_addr_table = client_ctx->input_buf_addr_table;
		num_of_buffers = client_ctx->num_of_input_buffers;
		DBG("%s(): buffer_type = INPUT \n", __func__);

	} else {
		buf_addr_table = client_ctx->output_buf_addr_table;
		num_of_buffers = client_ctx->num_of_output_buffers;
		DBG("%s(): buffer_type = OUTPUT \n", __func__);
	}

	for (i = 0; i < num_of_buffers; ++i) {
		if (search_with_user_vaddr) {
			if (*user_vaddr == buf_addr_table[i].user_vaddr) {
				*kernel_vaddr = buf_addr_table[i].kernel_vaddr;
				found = TRUE;
				DBG("%s() : client_ctx = %p."
				" user_virt_addr = 0x%08lx is found",
				__func__, client_ctx, *user_vaddr);
				break;
			}
		} else {
			if (*kernel_vaddr == buf_addr_table[i].kernel_vaddr) {
				*user_vaddr = buf_addr_table[i].user_vaddr;
				found = TRUE;
				DBG("%s() : client_ctx = %p."
				" kernel_virt_addr = 0x%08lx is found",
				__func__, client_ctx, *kernel_vaddr);
				break;
			}
		}
	}

	if (found) {
		*phy_addr = buf_addr_table[i].phy_addr;
		*pmem_fd = buf_addr_table[i].pmem_fd;
		*file = buf_addr_table[i].file;
		*buffer_index = i;

		if (search_with_user_vaddr)
			DBG("kernel_vaddr = 0x%08lx, phy_addr = 0x%08lx "
			" pmem_fd = %d, struct *file	= %p "
			"buffer_index = %d \n", *kernel_vaddr,
			*phy_addr, *pmem_fd, *file, *buffer_index);
		else
			DBG("user_vaddr = 0x%08lx, phy_addr = 0x%08lx "
			" pmem_fd = %d, struct *file	= %p "
			"buffer_index = %d \n", *user_vaddr, *phy_addr,
			*pmem_fd, *file, *buffer_index);
		return TRUE;
	} else {
		if (search_with_user_vaddr)
			DBG("%s() : client_ctx = %p user_virt_addr = 0x%08lx"
			" Not Found.\n", __func__, client_ctx, *user_vaddr);
		else
			DBG("%s() : client_ctx = %p kernel_virt_addr = 0x%08lx"
			" Not Found.\n", __func__, client_ctx,
			*kernel_vaddr);
		return FALSE;
	}
}
EXPORT_SYMBOL(vid_c_lookup_addr_table);

u32 vid_c_timer_create(void (*pf_timer_handler)(void *),
	void *p_user_data, void **pp_timer_handle)
{
	struct vid_c_timer *hw_timer = NULL;
	if (!pf_timer_handler || !pp_timer_handle) {
		DBG("%s(): timer creation failed \n ", __func__);
		return FALSE;
	}
	hw_timer = kzalloc(sizeof(struct vid_c_timer), GFP_KERNEL);
	if (!hw_timer) {
		DBG("%s(): timer creation failed in allocation \n ", __func__);
		return FALSE;
	}
	init_timer(&hw_timer->hw_timeout);
	hw_timer->hw_timeout.data = (unsigned long)hw_timer;
	hw_timer->hw_timeout.function = vid_c_timer_fn;
	hw_timer->cb_func = pf_timer_handler;
	hw_timer->userdata = p_user_data;
	*pp_timer_handle = hw_timer;
	return TRUE;
}
EXPORT_SYMBOL(vid_c_timer_create);

void  vid_c_timer_release(void *p_timer_handle)
{
	kfree(p_timer_handle);
}
EXPORT_SYMBOL(vid_c_timer_release);

void  vid_c_timer_start(void *p_timer_handle, u32 n_time_out)
{
	struct vid_c_timer *hw_timer = (struct vid_c_timer *)p_timer_handle;
	DBG("%s(): start timer\n ", __func__);
	if (hw_timer) {
		hw_timer->hw_timeout.expires = jiffies + 1*HZ;
		add_timer(&hw_timer->hw_timeout);
	}
}
EXPORT_SYMBOL(vid_c_timer_start);

void  vid_c_timer_stop(void *p_timer_handle)
{
	struct vid_c_timer *hw_timer = (struct vid_c_timer *)p_timer_handle;
	DBG("%s(): stop timer\n ", __func__);
	if (hw_timer)
		del_timer(&hw_timer->hw_timeout);
}
EXPORT_SYMBOL(vid_c_timer_stop);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Video decoder/encoder driver Init Module");
MODULE_VERSION("1.0");
module_init(vid_c_init);
module_exit(vid_c_exit);
