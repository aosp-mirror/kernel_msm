/*
 * iaxxx-odsp-celldrv.c -- IAxxx OpenDSP cell driver
 *
 * Copyright 2017 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/adnc/iaxxx-odsp.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/module.h>

#include "iaxxx-odsp-celldrv.h"

static struct odsp_cell_params odsp_cell_priv;



static long odsp_dev_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct odsp_device_priv *odsp_dev_priv = file->private_data;
	struct iaxxx_priv *priv = to_iaxxx_priv(odsp_dev_priv->parent);
	struct iaxxx_plugin_info plg_info;
	struct iaxxx_plugin_param param_info;
	struct iaxxx_plugin_param_blk param_blk_info;
	struct iaxxx_plugin_create_cfg cfg_info;
	struct iaxxx_set_event set_event;
	struct iaxxx_get_event get_event;
	struct iaxxx_evt_info event_info;
	struct iaxxx_pkg_mgmt_info pkg_info;
	void __user *blk_buff = NULL;
	uint32_t id;
	uint32_t bitmap;
	int ret = -EINVAL;

	pr_debug("%s() cmd %d\n", __func__, cmd);

	if (atomic_read(&odsp_dev_priv->power_state) != IAXXX_NORMAL)
		return -ENXIO;
	if (!priv)
		return -EINVAL;
	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "Chip state NULL\n");
		return -EINVAL;
	}
	if (priv->iaxxx_state->fw_state != FW_APP_MODE) {
		dev_err(priv->dev, "FW state not valid %d %s()\n",
			priv->iaxxx_state->fw_state, __func__);
		return -EIO;
	}

	switch (cmd) {
	case ODSP_PLG_CREATE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;

		ret = iaxxx_core_create_plg(odsp_dev_priv->parent,
					plg_info.inst_id,
					plg_info.priority, plg_info.pkg_id,
					plg_info.plg_idx, plg_info.block_id);
		if (ret) {
			pr_err("%s() Plugin create fail\n", __func__);
			return ret;
		}
		break;
	case ODSP_PLG_DESTROY:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;
		iaxxx_core_destroy_plg(odsp_dev_priv->parent, plg_info.inst_id,
						plg_info.block_id);
		break;
	case ODSP_PLG_ENABLE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;
		iaxxx_core_change_plg_state(odsp_dev_priv->parent,
				plg_info.inst_id, 1, plg_info.block_id);
		break;
	case ODSP_PLG_DISABLE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;
		iaxxx_core_change_plg_state(odsp_dev_priv->parent,
				plg_info.inst_id, 0, plg_info.block_id);
		break;
	case ODSP_PLG_RESET:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;
		ret = iaxxx_core_reset_plg(odsp_dev_priv->parent,
						plg_info.inst_id,
						plg_info.block_id);
		if (ret) {
			pr_err("%s() Plugin reset fail\n", __func__);
			return ret;
		}
		break;
	case ODSP_PLG_SET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
						sizeof(param_info)))
			return -EFAULT;
		ret = iaxxx_core_plg_set_param_by_inst(odsp_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				param_info.param_val, param_info.block_id);
		if (ret) {
			pr_err("%s() Plugin set param fail\n", __func__);
			pr_err("Param info inst 0x%x p_id 0x%x p_val 0x%x\n",
					param_info.inst_id, param_info.param_id,
					param_info.param_val);
			return ret;
		}
		break;
	case ODSP_PLG_GET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
						sizeof(param_info)))
			return -EFAULT;
		ret = iaxxx_core_plg_get_param_by_inst(odsp_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				&param_info.param_val, param_info.block_id);
		if (ret) {
			pr_err("%s() Plugin get param fail\n", __func__);
			pr_err("Param info inst 0x%x p_id 0x%x\n",
				param_info.inst_id, param_info.param_id);
			return ret;
		}
		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &param_info,
						sizeof(param_info)))
			return -EFAULT;

		break;
	case ODSP_PLG_SET_PARAM_BLK:
		if (copy_from_user(&param_blk_info, (void __user *)arg,
				sizeof(struct iaxxx_plugin_param_blk)))
			return -EFAULT;

		blk_buff = (void __user *) (uintptr_t)param_blk_info.param_blk;
		blk_buff = memdup_user(blk_buff, param_blk_info.param_size);
		param_blk_info.param_blk = (uintptr_t)blk_buff;
		if (IS_ERR(blk_buff)) {
			ret = PTR_ERR(blk_buff);
			pr_err("%s memdup failed %d\n", __func__, ret);
			return ret;
		}
		blk_buff = (void *)(uintptr_t)(param_blk_info.param_blk);
		ret = iaxxx_core_set_param_blk(odsp_dev_priv->parent,
			param_blk_info.inst_id, param_blk_info.param_size,
			blk_buff, param_blk_info.block_id,
			param_blk_info.id);
		kfree(blk_buff);
		if (ret) {
			pr_err("%s() Set param blk fail\n", __func__);
			return ret;
		}
		break;
	case ODSP_PLG_SET_CREATE_CFG:
		if (copy_from_user(&cfg_info, (void __user *)arg,
						sizeof(cfg_info)))
			return -EFAULT;
		cfg_info.file_name[sizeof(cfg_info.file_name) - 1] = '\0';
		ret = iaxxx_core_set_create_cfg(odsp_dev_priv->parent,
				cfg_info.inst_id, cfg_info.cfg_size,
				cfg_info.cfg_val, cfg_info.block_id,
				cfg_info.file_name);
		if (ret) {
			pr_err("%s() Plugin create cfg fail\n", __func__);
			return ret;
		}
		break;
	case ODSP_PLG_SET_EVENT:
		if (copy_from_user(&set_event, (void __user *)arg,
						sizeof(set_event)))
			return -EFAULT;
		ret = iaxxx_core_set_event(odsp_dev_priv->parent,
					set_event.inst_id,
					set_event.event_enable_mask,
					set_event.block_id);
		if (ret) {
			pr_err("%s() Set event fail\n", __func__);
			return ret;
		}
		break;
	case ODSP_EVENT_SUBSCRIBE:
		if (copy_from_user(&event_info, (void __user *)arg,
						sizeof(event_info)))
			return -EFAULT;
		ret = iaxxx_core_evt_subscribe(odsp_dev_priv->parent,
					event_info.src_id,
					event_info.event_id,
					event_info.dst_id,
					event_info.dst_opaque);
		if (ret) {
			pr_err("%s() Event subscribe fail\n", __func__);
			return ret;
		}
		break;
	case ODSP_GET_EVENT:
		if (copy_from_user(&get_event, (void __user *)arg,
						sizeof(get_event)))
			return -EFAULT;
		ret = iaxxx_core_retrieve_event(odsp_dev_priv->parent,
				&get_event.event_id,
				&get_event.data);
		if (ret) {
			pr_err("%s() Get event fail\n", __func__);
			return ret;
		}
		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &get_event,
						sizeof(get_event)))
			return -EFAULT;
		break;
	case ODSP_EVENT_UNSUBSCRIBE:
		if (copy_from_user(&event_info, (void __user *)arg,
						sizeof(event_info)))
			return -EFAULT;
		ret = iaxxx_core_evt_unsubscribe(odsp_dev_priv->parent,
					event_info.src_id,
					event_info.event_id,
					event_info.dst_id,
					event_info.dst_opaque);
		if (ret) {
			pr_err("%s() Event unsubscribe fail\n", __func__);
			return ret;
		}
		break;
	case ODSP_LOAD_PACKAGE:
		if (copy_from_user(&pkg_info, (void __user *)arg,
						sizeof(pkg_info)))
			return -EFAULT;
		ret = iaxxx_package_load(odsp_dev_priv->parent,
			pkg_info.pkg_name, pkg_info.pkg_id,
			&pkg_info.proc_id);
		pr_info("%s()Pkg name %s id %d\n", __func__,
				pkg_info.pkg_name, pkg_info.pkg_id);
		if (ret) {
			pr_err("%s() Load package failed\n", __func__);
			if (ret == -EEXIST) {
				if (copy_to_user((void __user *)arg, &pkg_info,
							sizeof(pkg_info)))
					return -EFAULT;
			}
			return ret;
		}
		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &pkg_info,
						sizeof(pkg_info)))
			return -EFAULT;
		break;

	case ODSP_UNLOAD_PACKAGE:
		if (copy_from_user(&pkg_info, (void __user *)arg,
						sizeof(pkg_info)))
			return -EFAULT;
		ret = iaxxx_package_unload(odsp_dev_priv->parent,
			pkg_info.pkg_name, pkg_info.proc_id);
		pr_info("%s()Pkg name %s id %d\n", __func__,
				pkg_info.pkg_name, pkg_info.pkg_id);
		if (ret) {
			pr_err("%s() Unload package failed\n", __func__);
			return ret;
		}
		break;
	case ODSP_UNLOAD_KW_MODEL:
		if (copy_from_user(&param_blk_info, (void __user *)arg,
				sizeof(struct iaxxx_plugin_param_blk)))
			return -EFAULT;
		ret = iaxxx_unload_kw_model(odsp_dev_priv->parent,
				param_blk_info.inst_id, param_blk_info.block_id,
				param_blk_info.id);
		return ret;
	case ODSP_START_RECOGNITION:
		if (copy_from_user(&id, (void __user *)arg,
				sizeof(uint32_t)))
			return -EFAULT;
		ret = iaxxx_start_recognition(odsp_dev_priv->parent, id);
		return ret;
	case ODSP_STOP_RECOGNITION:
		if (copy_from_user(&id, (void __user *)arg,
				sizeof(uint32_t)))
			return -EFAULT;
		ret = iaxxx_stop_recognition(odsp_dev_priv->parent, id);
		return ret;
	case ODSP_GET_KW_RECOGNIZE_BITMAP:
		ret = iaxxx_get_kw_recognize_bitmap
			(odsp_dev_priv->parent, &bitmap);
		if (copy_to_user
			((void __user *)arg, &bitmap, sizeof(uint32_t))) {
			pr_info("%s() copy_to_user failed\n", __func__);
			return -EFAULT;
		}
		return ret;
	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long odsp_dev_compat_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	return odsp_dev_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int odsp_dev_open(struct inode *inode, struct file *file)
{
	struct odsp_device_priv *odsp_dev_priv = container_of(inode->i_cdev,
						struct odsp_device_priv, cdev);
	file->private_data = odsp_dev_priv;

	return 0;
}

static int odsp_dev_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations odsp_dev_fops = {
	.open = odsp_dev_open,
	.release = odsp_dev_release,
	.unlocked_ioctl	= odsp_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= odsp_dev_compat_ioctl,
#endif
};


static int iaxxx_odsp_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct odsp_device_priv *odsp_dev_priv;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);
	int ret;

	pr_info("Enter :%s\n", __func__);
	if (odsp_cell_priv.cdev_minor == ODSP_CDEV_MAX_DEVICES) {
		pr_err("Minor nos exhausted. Increase CVQ_CDEV_MAX_DEVICES\n");
		return -ENOBUFS;
	}

	odsp_dev_priv = kzalloc(sizeof(*odsp_dev_priv), GFP_KERNEL);
	if (!odsp_dev_priv) {

		ret = -ENOMEM;
		goto out_err;
	}

	odsp_dev_priv->regmap = priv->regmap;
	odsp_dev_priv->parent = dev->parent;
	odsp_dev_priv->cdev.owner = THIS_MODULE;
	odsp_dev_priv->dev_num =
	MKDEV(MAJOR(odsp_cell_priv.dev_num), odsp_cell_priv.cdev_minor);
	cdev_init(&odsp_dev_priv->cdev, &odsp_dev_fops);
	ret = cdev_add(&odsp_dev_priv->cdev, odsp_dev_priv->dev_num, 1);
	if (ret) {
		pr_err("failed to add cdev error: %d", ret);
		goto free_odsp;
	}

	odsp_dev_priv->dev = device_create(odsp_cell_priv.cdev_class, NULL,
			odsp_dev_priv->dev_num, odsp_dev_priv, dev_name(dev));
	if (IS_ERR(odsp_dev_priv->dev)) {
		ret = PTR_ERR(dev);
		pr_err("Failed (%d) to create cdev device\n", ret);
		goto err_device_create;
	}

	dev_set_drvdata(dev, odsp_dev_priv);
	pr_info("ODSP device cdev initialized.\n");

	return 0;

err_device_create:
	cdev_del(&odsp_dev_priv->cdev);
free_odsp:
	kfree(odsp_dev_priv);
out_err:
	pr_err("cdev setup failure: no cdevs available!\n");

	return ret;

}
static int iaxxx_odsp_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct odsp_device_priv *odsp_dev_priv = dev_get_drvdata(dev);

	device_destroy(odsp_cell_priv.cdev_class, odsp_dev_priv->dev_num);
	cdev_del(&odsp_dev_priv->cdev);

	kfree(odsp_dev_priv);

	return 0;
}


static const struct of_device_id iaxxx_odsp_dt_match[] = {
	{.compatible = "knowles,iaxxx-odsp-celldrv"},
	{}
};

static struct platform_driver iaxxx_odsp_driver = {
	.probe  = iaxxx_odsp_dev_probe,
	.remove = iaxxx_odsp_dev_remove,
	.driver = {
		.name = "iaxxx-odsp-celldrv",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_odsp_dt_match,
	},
};

static int __init iaxxx_odsp_init(void)
{
	int ret;

	odsp_cell_priv.cdev_minor  = 0;
	odsp_cell_priv.cdev_class = class_create(THIS_MODULE,
						"iaxxx-odsp-celldrv");
	if (IS_ERR(odsp_cell_priv.cdev_class)) {
		ret = PTR_ERR(odsp_cell_priv.cdev_class);
		pr_err("Err (%d) creating iaxxx-odsp-celldrv class.\n", ret);
		goto out_err;
	}

	ret = alloc_chrdev_region(&odsp_cell_priv.dev_num, 0,
				ODSP_CDEV_MAX_DEVICES, "iaxxx-odsp-celldrv");
	if (ret) {
		pr_err("Failed to alloc chardev region (%d)", ret);
		goto err_alloc_cdev;
	}

	pr_info("Allocated Major device number %d\n",
						MAJOR(odsp_cell_priv.dev_num));

	ret =  platform_driver_register(&iaxxx_odsp_driver);
	if (ret)
		goto err_plat_register;

	return 0;

err_plat_register:
	unregister_chrdev_region(odsp_cell_priv.dev_num, ODSP_CDEV_MAX_DEVICES);
err_alloc_cdev:
	class_destroy(odsp_cell_priv.cdev_class);
out_err:
	pr_err("ODSP Cell init failure (%d). No CVQ cdevs available!\n", ret);
	return ret;
}


static void __exit iaxxx_odsp_exit(void)
{

	platform_driver_unregister(&iaxxx_odsp_driver);

	unregister_chrdev_region(odsp_cell_priv.dev_num,
						ODSP_CDEV_MAX_DEVICES);
	class_destroy(odsp_cell_priv.cdev_class);
}
module_init(iaxxx_odsp_init);
module_exit(iaxxx_odsp_exit);
