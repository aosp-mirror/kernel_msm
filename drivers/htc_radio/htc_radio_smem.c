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
#include <asm/io.h>
#include "htc_radio_smem.h"
#include <soc/qcom/smem.h>

#define DEVICE_TREE_RADIO_PATH "/chosen/radio"

static int htc_radio_smem_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct device_node *dnp;
	struct htc_smem_type *htc_radio_smem;
	u32 value;
	u8 buffer[max(sizeof(htc_radio_smem->htc_rom_ver),
		      sizeof(htc_radio_smem->htc_smem_skuid))];

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

	/* set smem to 0 */
	memset_io(htc_radio_smem, 0, sizeof(*htc_radio_smem));

	/* get smem data from radio data note */
	dnp = of_find_node_by_path(DEVICE_TREE_RADIO_PATH);
	if (dnp) {
		htc_smem_set_u32(htc_radio_smem, version,
			HTC_RADIO_SMEM_VERSION);
		htc_smem_set_u32(htc_radio_smem, struct_size,
			sizeof(struct htc_smem_type));

		of_property_read_u32(dnp, "htc_smem_radio_dbg_flag", &value);
		htc_smem_set_u32(htc_radio_smem, htc_smem_flag, value);

		of_property_read_u32(dnp, "htc_smem_app_run_mode", &value);
		htc_smem_set_u32(htc_radio_smem, htc_smem_app_run_mode, value);

		of_property_read_u32(dnp, "htc_smem_pid", &value);
		htc_smem_set_u32(htc_radio_smem, htc_smem_pid, value);

		of_property_read_u32(dnp, "htc_smem_factory_reset", &value);
		htc_smem_set_u32(htc_radio_smem, htc_smem_factory_reset,
			value);

		of_property_read_u8_array(dnp, "htc_rom_ver", buffer,
				sizeof(htc_radio_smem->htc_rom_ver));
		htc_smem_copy(htc_radio_smem, htc_rom_ver, buffer);

		of_property_read_u8_array(dnp, "sku_id", buffer,
				sizeof(htc_radio_smem->htc_smem_skuid));
		htc_smem_copy(htc_radio_smem, htc_smem_skuid, buffer);
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
