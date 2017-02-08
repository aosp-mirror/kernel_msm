/*
 * Copyright (c) 2016 HTC Corporation.
 *
 *     @file   kernel/drivers/htc_radio/htc_radio_smem.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/ctype.h>
#include "htc_radio_smem.h"
#include <soc/qcom/smem.h>

#define DEVICE_TREE_RADIO_PATH "/chosen/radio"

static void smem_init(struct htc_smem_type *smem)
{
	int i = 0;

	smem->version = 0;
	smem->struct_size = 0;
	smem->htc_smem_pid = 0;
	smem->htc_smem_app_run_mode = 0;
	smem->htc_smem_flag = 0;
	smem->htc_smem_factory_reset = 0;

	for (i = 0; i < sizeof(smem->htc_rom_ver); i++)
		smem->htc_rom_ver[i] = 0;

	for (i = 0; i < sizeof(smem->htc_smem_skuid); i++)
		smem->htc_smem_skuid[i] = 0;

	for (i = 0; i < sizeof(smem->reserved); i++)
		smem->reserved[i] = 0;
}

static int htc_radio_smem_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct device_node *dnp;
	struct htc_smem_type *htc_radio_smem;

	pr_info("[smem]%s: start.\n", __func__);

	htc_radio_smem = smem_alloc(SMEM_ID_VENDOR0,
				sizeof(struct htc_smem_type),
				0, SMEM_ANY_HOST_FLAG);

	if (htc_radio_smem) {
		pr_info("[smem]%s: smem_via_smd=0x%p.\n", __func__,
				htc_radio_smem);
	} else {
		ret = -ENOMEM;
		pr_err("[smem]%s: smd alloc fail !!\n", __func__);
		return ret;
	}

	/* set smem init 0 */
	smem_init(htc_radio_smem);

	/* get smem data from radio data note */
	dnp = of_find_node_by_path(DEVICE_TREE_RADIO_PATH);
	if (dnp) {
		htc_radio_smem->version = HTC_RADIO_SMEM_VERSION;
		htc_radio_smem->struct_size = sizeof(struct htc_smem_type);
		of_property_read_u32(dnp, "htc_smem_radio_dbg_flag",
				&htc_radio_smem->htc_smem_flag);
		of_property_read_u32(dnp, "htc_smem_app_run_mode",
				&htc_radio_smem->htc_smem_app_run_mode);
		of_property_read_u32(dnp, "htc_smem_pid",
				&htc_radio_smem->htc_smem_pid);
		of_property_read_u32(dnp, "htc_smem_factory_reset",
				&htc_radio_smem->htc_smem_factory_reset);
		of_property_read_u8_array(dnp, "htc_rom_ver",
				&htc_radio_smem->htc_rom_ver[0],
				sizeof(htc_radio_smem->htc_rom_ver));
		of_property_read_u8_array(dnp, "sku_id",
				&htc_radio_smem->htc_smem_skuid[0],
				sizeof(htc_radio_smem->htc_smem_skuid));
	} else
		pr_err("[smem]%s: cannot find path %s.\n", __func__,
			DEVICE_TREE_RADIO_PATH);

	pr_info("[smem]%s: end.\n", __func__);
	return 0;
}

static const struct of_device_id htc_radio_smem_of_match[] = {
	{.compatible = "htc,htc_radio_smem",}
};
MODULE_DEVICE_TABLE(of, htc_radio_smem_of_match);

static struct platform_driver htc_radio_smem_driver = {
	.probe = htc_radio_smem_probe,
	.driver = {
		.name = "htc_radio_smem",
		.owner = THIS_MODULE,
		.of_match_table = htc_radio_smem_of_match,
	}
};

static int __init htc_radio_smem_init(void)
{
	int ret = -1;

	pr_info("[smem]%s.\n", __func__);

	ret = platform_driver_register(&htc_radio_smem_driver);
	if (ret < 0)
		pr_err("[smem]%s platform_driver register fail. ret:%d\n",
			__func__, ret);

	return ret;
}

static void __exit htc_radio_smem_exit(void)
{
	platform_driver_unregister(&htc_radio_smem_driver);
}

module_init(htc_radio_smem_init);
module_exit(htc_radio_smem_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("htc radio smem driver");
