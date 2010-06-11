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

#include "vcd_ddl_firmware.h"
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

static struct vid_c_dev *vidc_dev;
static dev_t vidc_dev_num;
static struct class *vidc_class;

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


static void vid_c_timer_fn(unsigned long data)
{
	unsigned long flag;
	struct vid_c_timer *hw_timer = NULL;

	DBG("%s: Timer expired\n", __func__);
	spin_lock_irqsave(&vidc_spin_lock, flag);
	hw_timer = (struct vid_c_timer *)data;
	list_add_tail(&hw_timer->list, &vidc_dev->vidc_timer_queue);
	spin_unlock_irqrestore(&vidc_spin_lock, flag);
	DBG("Queue the work for timer\n");
	queue_work(vidc_timer_wq, &vidc_dev->vidc_timer_worker);
}

static void vid_c_timer_handler(struct work_struct *work)
{
	unsigned long flag = 0;
	u32 islist_empty = 0;
	struct vid_c_timer *hw_timer = NULL;

	DBG("%s: Timer expired\n", __func__);
	do {
		spin_lock_irqsave(&vidc_spin_lock, flag);
		islist_empty = list_empty(&vidc_dev->vidc_timer_queue);
		if (!islist_empty) {
			hw_timer = list_first_entry(&vidc_dev->vidc_timer_queue,
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
	enable_irq(vidc_dev->irq);
	DBG("vid_c_work_handler() done");
}

static DECLARE_WORK(vid_c_work, vid_c_work_handler);

static int __init vid_c_720p_probe(struct platform_device *pdev)
{
	int rc;
	struct resource *resource;
	DBG("Enter %s\n", __func__);

	if (pdev->id) {
		ERR("Invalid platform device ID = %d\n", pdev->id);
		return -EINVAL;
	}
	vidc_dev->irq = platform_get_irq(pdev, 0);
	if (unlikely(vidc_dev->irq < 0)) {
		ERR("%s: Invalid irq = %d\n", __func__, vidc_dev->irq);
		return -ENXIO;
	}

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource)) {
		ERR("%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	vidc_dev->phys_base = resource->start;
	vidc_dev->virt_base = ioremap(resource->start,
	resource->end - resource->start + 1);

	if (!vidc_dev->virt_base) {
		ERR("%s: ioremap failed\n", __func__);
		return -ENOMEM;
	}
	vidc_dev->device = &pdev->dev;
	mutex_init(&vidc_dev->lock);

	vid_c_wq = create_singlethread_workqueue("vid_c_worker_queue");
	if (!vid_c_wq) {
		ERR("%s: create workqueue failed\n", __func__);
		return -ENOMEM;
	}

	rc = vcd_fw_init(vidc_dev->device);
	if (rc)
		ERR("%s: failed to prepare firmware %d\n", __func__, rc);

	return rc;
}

static int __devexit vid_c_720p_remove(struct platform_device *pdev)
{
	if (pdev->id) {
		ERR("Invalid plaform device ID = %d\n", pdev->id);
		return -EINVAL;
	}
	vcd_fw_exit();
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
	DBG("vid_c_isr() %d\n", irq);
	disable_irq_nosync(irq);
	queue_work(vid_c_wq, &vid_c_work);
	return IRQ_HANDLED;
}

static int __init vid_c_init(void)
{
	int rc = 0;
	struct device *class_devp;

	vidc_dev = kzalloc(sizeof(struct vid_c_dev), GFP_KERNEL);
	if (!vidc_dev) {
		ERR("%s Unable to allocate memory for vid_c_dev\n",
			__func__);
		return -ENOMEM;
	}

	rc = alloc_chrdev_region(&vidc_dev_num, 0, 1, VID_C_NAME);
	if (rc < 0) {
		ERR("%s: alloc_chrdev_region failed %d\n", __func__, rc);
		goto error_vid_c_alloc_chrdev_region;
	}

	vidc_class = class_create(THIS_MODULE, VID_C_NAME);
	if (IS_ERR(vidc_class)) {
		rc = PTR_ERR(vidc_class);
		ERR("%s: couldn't create vid_c_class %d\n", __func__, rc);
		goto error_vid_c_class_create;
	}

	class_devp = device_create(vidc_class, NULL, vidc_dev_num, NULL,
					VID_C_NAME);

	if (IS_ERR(class_devp)) {
		rc = PTR_ERR(class_devp);
		ERR("%s: class device_create failed %d\n", __func__, rc);
		goto error_vid_c_class_device_create;
	}

	cdev_init(&vidc_dev->cdev, &vid_c_fops);
	vidc_dev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&(vidc_dev->cdev), vidc_dev_num, 1);

	if (rc < 0) {
		ERR("%s: cdev_add failed %d\n", __func__, rc);
		goto error_vid_c_cdev_add;
	}

	rc = platform_driver_register(&msm_vid_c_720p_platform_driver);
	if (rc) {
		ERR("%s failed to load\n", __func__);
		goto error_vid_c_platfom_register;
	}

	rc = request_irq(vidc_dev->irq, vid_c_isr, IRQF_TRIGGER_HIGH,
			 "vid_c", vidc_dev->device);
	if (unlikely(rc)) {
		ERR("%s:request_irq failed\n", __func__);
		goto error_vid_c_platfom_register;
	}

	vidc_timer_wq = create_singlethread_workqueue("vidc_timer_wq");
	if (!vidc_timer_wq) {
		ERR("%s: create workqueue failed\n", __func__);
		rc = -ENOMEM;
		goto error_vid_c_platfom_register;
	}

	DBG("Disabling IRQ in %s\n", __func__);
	disable_irq_nosync(vidc_dev->irq);
	INIT_WORK(&vidc_dev->vidc_timer_worker, vid_c_timer_handler);
	spin_lock_init(&vidc_spin_lock);
	INIT_LIST_HEAD(&vidc_dev->vidc_timer_queue);
	vidc_dev->clock_enabled = 0;
	vidc_dev->ref_count = 0;
	vidc_dev->firmware_refcount = 0;
	vidc_dev->get_firmware = 0;

	return 0;

error_vid_c_platfom_register:
	cdev_del(&(vidc_dev->cdev));
error_vid_c_cdev_add:
	device_destroy(vidc_class, vidc_dev_num);
error_vid_c_class_device_create:
	class_destroy(vidc_class);
error_vid_c_class_create:
	unregister_chrdev_region(vidc_dev_num, 1);
error_vid_c_alloc_chrdev_region:
	kfree(vidc_dev);

	return rc;
}

void __iomem *vid_c_get_ioaddr()
{
	return vidc_dev->virt_base;
}
EXPORT_SYMBOL(vid_c_get_ioaddr);
#ifdef USE_RES_TRACKER

u32 vid_c_enable_pwr_rail()
{
	int rc;

	mutex_lock(&vidc_dev->lock);

	if (vidc_dev->rail_enabled) {
		mutex_unlock(&vidc_dev->lock);
		return true;
	}

	//TODO: internal_pwr_rail_mode(MFC_CLK_ID, MANUAL)

	vidc_dev->pclk = clk_get(vidc_dev->device, "mfc_pclk");
	if (IS_ERR(vidc_dev->pclk)) {
		ERR("%s: mfc_pclk get failed\n", __func__);
		goto err;
	}

	vidc_dev->hclk = clk_get(vidc_dev->device, "mfc_clk");
	if (IS_ERR(vidc_dev->hclk)) {
		ERR("%s: mfc_clk get failed\n", __func__);
		goto err;
	}

	vidc_dev->hclk_div2 = clk_get(vidc_dev->device, "mfc_div2_clk");
	if (IS_ERR(vidc_dev->hclk_div2)) {
		ERR("%s: mfc_div2_clk get failed\n", __func__);
		goto err;
	}

	//TODO: internal_pwr_rail_ctl(MFC_CLK_ID, 1)

	//TODO msleep must die
	msleep(20);

	rc = clk_reset(vidc_dev->pclk, CLK_RESET_DEASSERT);
	if (rc) {
		ERR("clk_reset failed %d\n", rc);
		goto err;
	}
	//TODO msleep must die
	msleep(20);

	vidc_dev->rail_enabled = 1;
	mutex_unlock(&vidc_dev->lock);
	return true;

err:
	if (!IS_ERR(vidc_dev->pclk))
		clk_put(vidc_dev->pclk);
	if (!IS_ERR(vidc_dev->hclk))
		clk_put(vidc_dev->hclk);
	if (!IS_ERR(vidc_dev->hclk_div2))
		clk_put(vidc_dev->hclk_div2);
	mutex_unlock(&vidc_dev->lock);
	return false;
}
EXPORT_SYMBOL(vid_c_enable_pwr_rail);

u32 vid_c_disable_pwr_rail()
{
	int rc = -1;
	mutex_lock(&vidc_dev->lock);

	if (vidc_dev->clock_enabled) {
		mutex_unlock(&vidc_dev->lock);
		DBG("Calling CLK disable in power down\n");
		vid_c_disable_clk();
		mutex_lock(&vidc_dev->lock);
	}

	if (!vidc_dev->rail_enabled) {
		mutex_unlock(&vidc_dev->lock);
		return false;
	}

	vidc_dev->rail_enabled = 0;
	rc = clk_reset(vidc_dev->pclk, CLK_RESET_ASSERT);
	if (rc) {
		ERR("clk_reset failed %d\n", rc);
		mutex_unlock(&vidc_dev->lock);
		return false;
	}
	msleep(20);

	//TODO: internal_pwr_rail_ctl(MFC_CLK_ID, 0)

	clk_put(vidc_dev->hclk_div2);
	clk_put(vidc_dev->hclk);
	clk_put(vidc_dev->pclk);

	mutex_unlock(&vidc_dev->lock);

	return true;
}
EXPORT_SYMBOL(vid_c_disable_pwr_rail);

u32 vid_c_enable_clk()
{
	mutex_lock(&vidc_dev->lock);

	if (!vidc_dev->rail_enabled) {
		goto err;
	}
	if (vidc_dev->clock_enabled) {
		mutex_unlock(&vidc_dev->lock);
		return true;
	}

	DBG("Enabling IRQ in %s\n", __func__);
	enable_irq(vidc_dev->irq);
	DBG("%s: Enabling the clocks ...\n", __func__);

	if (clk_enable(vidc_dev->pclk)) {
		ERR("vidc pclk enable failed\n");
		goto err;
	}

	if (clk_enable(vidc_dev->hclk)) {
		ERR("vidc hclk enable failed\n");
		goto err;
	}

	if (clk_enable(vidc_dev->hclk_div2)) {
		ERR("vidc hclk_div2 enable failed\n");
		goto err;
	}

	vidc_dev->clock_enabled = 1;
	mutex_unlock(&vidc_dev->lock);
	return true;
err:
	mutex_unlock(&vidc_dev->lock);
	return false;
}
EXPORT_SYMBOL(vid_c_enable_clk);

u32 vid_c_sel_clk_rate(unsigned long hclk_rate)
{
	mutex_lock(&vidc_dev->lock);
	if (clk_set_rate(vidc_dev->hclk, hclk_rate)) {
		ERR("vidc hclk set rate failed\n");
		mutex_unlock(&vidc_dev->lock);
		return false;
	}
	vidc_dev->hclk_rate = hclk_rate;
	mutex_unlock(&vidc_dev->lock);
	return true;
}
EXPORT_SYMBOL(vid_c_sel_clk_rate);

u32 vid_c_get_clk_rate(unsigned long *phclk_rate)
{
	if (!phclk_rate) {
		ERR("vid_c_get_clk_rate(): phclk_rate is NULL\n");
		return false;
	}
	mutex_lock(&vidc_dev->lock);
	*phclk_rate = clk_get_rate(vidc_dev->hclk);
	if (!(*phclk_rate)) {
		ERR("vidc hclk get rate failed\n");
		mutex_unlock(&vidc_dev->lock);
		return false;
	}
	mutex_unlock(&vidc_dev->lock);
	return true;
}
EXPORT_SYMBOL(vid_c_get_clk_rate);

u32 vid_c_disable_clk(void)
{
	mutex_lock(&vidc_dev->lock);

	if (!vidc_dev->clock_enabled) {
		mutex_unlock(&vidc_dev->lock);
		return false;
	}

	DBG("Disabling IRQ in %s\n", __func__);
	disable_irq_nosync(vidc_dev->irq);
	DBG("%s: Disabling the clocks ...\n", __func__);

	vidc_dev->clock_enabled = 0;
	clk_disable(vidc_dev->hclk);
	clk_disable(vidc_dev->hclk_div2);
	clk_disable(vidc_dev->pclk);

	mutex_unlock(&vidc_dev->lock);

	return true;
}
EXPORT_SYMBOL(vid_c_disable_clk);

//TODO: consider deleting USE_RES_TRACKER
#else

u32 vid_c_enable_clk(unsigned long hclk_rate)
{
	int rc = -1;
	mutex_lock(&vidc_dev->lock);
	vidc_dev->ref_count++;

	if (!vidc_dev->clock_enabled) {
		DBG("Enabling IRQ in %s()\n", __func__);
		enable_irq(vidc_dev->irq);

		rc = internal_pwr_rail_mode
			(PWR_RAIL_MFC_CLK, PWR_RAIL_CTL_MANUAL);
		if (rc) {
			ERR("%s(): internal_pwr_rail_mode failed %d\n",
			__func__, rc);
			return false;
		}
		DBG("%s(): internal_pwr_rail_mode Success %d\n",
		__func__, rc);

		vidc_dev->pclk =
			clk_get(vidc_dev->device, "mfc_pclk");

		if (IS_ERR(vidc_dev->pclk)) {
			ERR("%s(): mfc_pclk get failed\n", __func__);

			mutex_unlock(&vidc_dev->lock);
			return false;
		}

		vidc_dev->hclk =
			clk_get(vidc_dev->device, "mfc_clk");

		if (IS_ERR(vidc_dev->hclk)) {
			ERR("%s(): mfc_clk get failed\n", __func__);

			clk_put(vidc_dev->pclk);
			mutex_unlock(&vidc_dev->lock);
			return false;
		}

		vidc_dev->hclk_div2 =
			clk_get(vidc_dev->device, "mfc_div2_clk");

		if (IS_ERR(vidc_dev->pclk)) {
			ERR("%s(): mfc_div2_clk get failed\n", __func__);

			clk_put(vidc_dev->pclk);
			clk_put(vidc_dev->hclk);
			mutex_unlock(&vidc_dev->lock);
			return false;
		}

		vidc_dev->hclk_rate = hclk_rate;

		if (clk_set_rate(vidc_dev->hclk,
			vidc_dev->hclk_rate)) {
			ERR("vid_c hclk set rate failed\n");
			clk_put(vidc_dev->pclk);
			clk_put(vidc_dev->hclk);
			clk_put(vidc_dev->hclk_div2);
			mutex_unlock(&vidc_dev->lock);
			return false;
		}

		if (clk_enable(vidc_dev->pclk)) {
			ERR("vid_c pclk Enable failed\n");

			clk_put(vidc_dev->hclk);
			clk_put(vidc_dev->hclk_div2);
			mutex_unlock(&vidc_dev->lock);
			return false;
		}

		if (clk_enable(vidc_dev->hclk)) {
			ERR("vid_c  hclk Enable failed\n");
			clk_put(vidc_dev->pclk);
			clk_put(vidc_dev->hclk_div2);
			mutex_unlock(&vidc_dev->lock);
			return false;
		}

		if (clk_enable(vidc_dev->hclk_div2)) {
			ERR("vid_c  hclk Enable failed\n");
			clk_put(vidc_dev->hclk);
			clk_put(vidc_dev->pclk);
			mutex_unlock(&vidc_dev->lock);
			return false;
		}
		msleep(20);
		rc = internal_pwr_rail_ctl(PWR_RAIL_MFC_CLK, 1);
		if (rc) {
			ERR("\n internal_pwr_rail_ctl failed %d\n", rc);
			return false;
		}
		DBG("%s(): internal_pwr_rail_ctl Success %d\n",
			__func__, rc);
		msleep(20);
		rc = clk_reset(vidc_dev->pclk, CLK_RESET_DEASSERT);
		if (rc) {
			ERR("\n clk_reset failed %d\n", rc);
			return false;
		}
		msleep(20);
	}
	vidc_dev->clock_enabled = 1;
	mutex_unlock(&vidc_dev->lock);
	return true;
}
EXPORT_SYMBOL(vid_c_enable_clk);

u32 vid_c_disable_clk(void)
{
	int rc = -1;
	mutex_lock(&vidc_dev->lock);

	if (!vidc_dev->ref_count ||
		!vidc_dev->clock_enabled) {
		return false;
	}

	if (vidc_dev->ref_count > 0)
		vidc_dev->ref_count--;

	if (!vidc_dev->ref_count) {
		DBG("Disabling IRQ in %s()\n", __func__);
		disable_irq_nosync(vidc_dev->irq);
		rc = clk_reset(vidc_dev->pclk, CLK_RESET_ASSERT);
		if (rc) {
			ERR("\n clk_reset failed %d\n", rc);
			return false;
		}
		msleep(20);

		rc = internal_pwr_rail_ctl(PWR_RAIL_MFC_CLK, 0);
		if (rc) {
			ERR("\n internal_pwr_rail_ctl failed %d\n", rc);
			return false;
		}

		vidc_dev->clock_enabled = 0;
		clk_disable(vidc_dev->hclk);
		clk_disable(vidc_dev->hclk_div2);
		clk_disable(vidc_dev->pclk);

		clk_put(vidc_dev->hclk_div2);
		clk_put(vidc_dev->hclk);
		clk_put(vidc_dev->pclk);

	}
	mutex_unlock(&vidc_dev->lock);
	return true;
}
EXPORT_SYMBOL(vid_c_disable_clk);

#endif

u32 vid_c_lookup_addr_table(struct video_client_ctx *client_ctx,
	enum buffer_dir buffer_type, u32 search_with_user_vaddr,
	void __user **user_addr, void **kern_addr, phys_addr_t *phys_addr,
	int *pmem_fd, struct file **file, s32 *buffer_index)
{
	u32 num_of_buffers;
	u32 i;
	struct buf_addr_table *buf_addr_table;
	u32 found = false;

	if (!client_ctx)
		return false;

	if (buffer_type == BUFFER_TYPE_INPUT) {
		buf_addr_table = client_ctx->input_buf_addr_table;
		num_of_buffers = client_ctx->num_of_input_buffers;
		DBG("%s: buffer_type = INPUT\n", __func__);
	} else {
		buf_addr_table = client_ctx->output_buf_addr_table;
		num_of_buffers = client_ctx->num_of_output_buffers;
		DBG("%s: buffer_type = OUTPUT\n", __func__);
	}

	for (i = 0; i < num_of_buffers; ++i) {
		if (search_with_user_vaddr) {
			if (*user_addr == buf_addr_table[i].user_addr) {
				*kern_addr = buf_addr_table[i].kern_addr;
				found = true;
				DBG("%s: client_ctx=%p user_addr=%p is found\n",
					__func__, client_ctx, *user_addr);
				break;
			}
		} else {
			if (*kern_addr == buf_addr_table[i].kern_addr) {
				*user_addr = buf_addr_table[i].user_addr;
				found = true;
				DBG("%s: client_ctx=%p kern_addr=%p is found",
					__func__, client_ctx, *kern_addr);
				break;
			}
		}
	}

	if (!found) {
		if (search_with_user_vaddr)
			DBG("%s: client_ctx=%p user_addr=%p not found\n",
				__func__, client_ctx, *user_addr);
		else
			DBG("%s: client_ctx=%p kern_addr=%p not found\n",
				__func__, client_ctx, *kern_addr);
		return false;
	}

	*phys_addr = buf_addr_table[i].phys_addr;
	*pmem_fd = buf_addr_table[i].pmem_fd;
	*file = buf_addr_table[i].file;
	*buffer_index = i;

	if (search_with_user_vaddr)
		DBG("kern_addr=%p phys_addr=%X pmem_fd=%d "
			"struct *file=%p buffer_index=%d\n", *kern_addr,
			*phys_addr, *pmem_fd, *file, *buffer_index);
	else
		DBG("user_addr=%p phys_addr=%X pmem_fd=%d, "
			"struct *file=%p buffer_index=%d\n", *user_addr,
			*phys_addr, *pmem_fd, *file, *buffer_index);
	return true;
}
EXPORT_SYMBOL(vid_c_lookup_addr_table);

u32 vid_c_timer_create(void (*pf_timer_handler)(void *), void *user_data,
	void **pp_timer_handle)
{
	struct vid_c_timer *hw_timer = NULL;
	if (!pf_timer_handler || !pp_timer_handle) {
		DBG("%s: timer creation failed\n", __func__);
		return false;
	}
	hw_timer = kzalloc(sizeof(struct vid_c_timer), GFP_KERNEL);
	if (!hw_timer) {
		DBG("%s: timer creation failed in allocation\n", __func__);
		return false;
	}
	init_timer(&hw_timer->hw_timeout);
	hw_timer->hw_timeout.data = (unsigned long)hw_timer;
	hw_timer->hw_timeout.function = vid_c_timer_fn;
	hw_timer->cb_func = pf_timer_handler;
	hw_timer->userdata = user_data;
	*pp_timer_handle = hw_timer;
	return true;
}
EXPORT_SYMBOL(vid_c_timer_create);

void  vid_c_timer_release(void *timer_handle)
{
	kfree(timer_handle);
}
EXPORT_SYMBOL(vid_c_timer_release);

void  vid_c_timer_start(void *timer_handle, u32 time_out)
{
	struct vid_c_timer *hw_timer = timer_handle;
	DBG("%s: start timer\n ", __func__);
	if (hw_timer) {
		hw_timer->hw_timeout.expires = jiffies + 1 * HZ;
		add_timer(&hw_timer->hw_timeout);
	}
}
EXPORT_SYMBOL(vid_c_timer_start);

void  vid_c_timer_stop(void *timer_handle)
{
	struct vid_c_timer *hw_timer = timer_handle;
	DBG("%s: stop timer\n ", __func__);
	if (hw_timer)
		del_timer(&hw_timer->hw_timeout);
}
EXPORT_SYMBOL(vid_c_timer_stop);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Video decoder/encoder driver Init Module");
MODULE_VERSION("1.0");
module_init(vid_c_init);
module_exit(vid_c_exit);
