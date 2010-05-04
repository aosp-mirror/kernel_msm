/* drivers/video/msm_fb/mddi_client_simple.c
 *
 * Support for simple mddi client devices which require no special
 * initialization code except for what may be provided in the board file.
 * If the clients do not provide board specific code, this driver's
 * panel operations are no-ops.
 *
 * Copyright (C) 2007-2010, Google Incorporated
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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <mach/msm_fb.h>

struct panel_info {
	struct platform_device		pdev;
	struct msm_mddi_client_data	*client_data;
	struct msm_panel_data		panel_data;
	struct msmfb_callback		*fb_callback;
	wait_queue_head_t		vsync_wait;
	int				got_vsync;
	int				irq;
};

#define to_panel_info(pd)    container_of((pd), struct panel_info, panel_data)

static void mddi_simple_request_vsync(struct msm_panel_data *panel_data,
				      struct msmfb_callback *callback)
{
	struct panel_info *panel = to_panel_info(panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	panel->fb_callback = callback;
	if (panel->got_vsync) {
		panel->got_vsync = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}
}

static void mddi_simple_wait_vsync(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = to_panel_info(panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	int ret;

	if (panel->got_vsync) {
		panel->got_vsync = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}

	ret = wait_event_timeout(panel->vsync_wait, panel->got_vsync, HZ/2);
	if (!ret && !panel->got_vsync)
		pr_err("mddi_client_simple: timeout waiting for vsync\n");

	panel->got_vsync = 0;
	/* interrupt clears when screen dma starts */
}


static int mddi_simple_suspend(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = to_panel_info(panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
			client_data->private_client_data;
	int ret;

	if (!bridge_data->uninit)
		return 0;

	ret = bridge_data->uninit(bridge_data, client_data);
	if (ret) {
		pr_info("%s: non zero return from uninit\n", __func__);
		return ret;
	}
	client_data->suspend(client_data);
	return 0;
}

static int mddi_simple_resume(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = to_panel_info(panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
			client_data->private_client_data;

	if (!bridge_data->init)
		return 0;

	client_data->resume(client_data);
	return bridge_data->init(bridge_data, client_data);
}

static int mddi_simple_blank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = to_panel_info(panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
			client_data->private_client_data;

	if (!bridge_data->blank)
		return 0;
	return bridge_data->blank(bridge_data, client_data);
}

static int mddi_simple_unblank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = to_panel_info(panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
			client_data->private_client_data;

	if (!bridge_data->unblank)
		return 0;
	return bridge_data->unblank(bridge_data, client_data);
}

static irqreturn_t handle_vsync_irq(int irq, void *data)
{
	struct panel_info *panel = data;

	panel->got_vsync = 1;
	if (panel->fb_callback) {
		panel->fb_callback->func(panel->fb_callback);
		panel->fb_callback = NULL;
	}

	wake_up(&panel->vsync_wait);
	return IRQ_HANDLED;
}

static int mddi_simple_probe(struct platform_device *pdev)
{
	struct msm_mddi_client_data *client_data = pdev->dev.platform_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
			client_data->private_client_data;
	struct panel_info *panel;
	int ret;

	pr_debug("%s()\n", __func__);

	panel = kzalloc(sizeof(struct panel_info), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	platform_set_drvdata(pdev, panel);

	init_waitqueue_head(&panel->vsync_wait);

	panel->irq = platform_get_irq_byname(pdev, "vsync");
	if (panel->irq >= 0) {
		ret = request_irq(panel->irq, handle_vsync_irq,
				  IRQF_TRIGGER_RISING, "mddi_c_simple_vsync",
				  panel);
		if (ret) {
			pr_err("%s: request vsync irq %d failed (%d)\n",
			       __func__, panel->irq, ret);
			goto err_req_irq;
		}

		panel->panel_data.wait_vsync = mddi_simple_wait_vsync;
		panel->panel_data.request_vsync = mddi_simple_request_vsync;
	}

	panel->client_data = client_data;
	panel->panel_data.suspend = mddi_simple_suspend;
	panel->panel_data.resume = mddi_simple_resume;
	panel->panel_data.blank = mddi_simple_blank;
	panel->panel_data.unblank = mddi_simple_unblank;
	panel->panel_data.caps = bridge_data->panel_caps;
	panel->panel_data.fb_data = &bridge_data->fb_data;

	panel->pdev.name = "msm_panel";
	panel->pdev.id = pdev->id;
	platform_device_add_resources(&panel->pdev,
				      client_data->fb_resource, 1);
	panel->pdev.dev.platform_data = &panel->panel_data;

	if (bridge_data->init)
		bridge_data->init(bridge_data, client_data);

	ret = platform_device_register(&panel->pdev);
	if (ret) {
		pr_err("%s: Can't register platform device\n", __func__);
		goto err_plat_dev_reg;
	}

	return 0;

err_plat_dev_reg:
	if (panel->irq >= 0)
		free_irq(panel->irq, panel);
err_req_irq:
	platform_set_drvdata(pdev, NULL);
	kfree(panel);
	return ret;
}

static int mddi_simple_remove(struct platform_device *pdev)
{
	struct panel_info *panel = platform_get_drvdata(pdev);
	kfree(panel);
	return 0;
}

static struct platform_driver mddi_client_simple = {
	.probe	= mddi_simple_probe,
	.remove	= mddi_simple_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "mddi_c_simple"
	},
};

static int __init mddi_client_simple_init(void)
{
	platform_driver_register(&mddi_client_simple);
	return 0;
}

module_init(mddi_client_simple_init);
