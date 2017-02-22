/* Copyright (c) 2017,  HUAWEI TECHNOLOGIES CO., LTD.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/of_gpio.h>

enum {
    POWER_SUPPLY_BATTERY_ID_UNKNOWN = 0,
    POWER_SUPPLY_BATTERY_ID_GUANGYU,
    POWER_SUPPLY_BATTERY_ID_DESAY,
};

#define BATT_ID_VOL_MIN_UV                    0
#define BATT_ID_VOL_AVG_UV                    548038
#define BATT_ID_VOL_MAX_UV                    1600000

static int global_batt_id = -1;

int get_global_batt_id(void)
{
    return global_batt_id;
}

/**************************************************************************
FUNCTION   get_batt_id

DESCRIPTION :Return batt id on success, 0 otherwise

**************************************************************************/
static int get_batt_id(struct qpnp_vadc_chip *vadc)
{
    int rc = 0;
    struct qpnp_vadc_result results = {0};

    if (NULL == vadc)
    {
        pr_err("get batt id error, vadc is not exist\n");
        return POWER_SUPPLY_BATTERY_ID_UNKNOWN;
    }

    rc = qpnp_vadc_read(vadc, LR_MUX2_BAT_ID, &results);
    if (rc)
    {
        pr_err("unable to read channel = %d\n", LR_MUX2_BAT_ID);
        return POWER_SUPPLY_BATTERY_ID_UNKNOWN;
    }

    if ((results.physical > BATT_ID_VOL_MIN_UV)
                && (results.physical <= BATT_ID_VOL_AVG_UV))
    {
        rc = POWER_SUPPLY_BATTERY_ID_GUANGYU;
    }
    else if ((results.physical > BATT_ID_VOL_AVG_UV)
                && (results.physical <= BATT_ID_VOL_MAX_UV))
    {
        rc = POWER_SUPPLY_BATTERY_ID_DESAY;
    }
    else
    {
        rc = POWER_SUPPLY_BATTERY_ID_UNKNOWN;
    }

    return rc;
}

static int battid_probe(struct platform_device *pdev)
{
    int rc = 0;
    struct device_node *np  = NULL;
    struct qpnp_vadc_chip *chip_battid = NULL;

    pr_info("battid probe begin...\n");
    /*get device tree node*/
    np = pdev->dev.of_node;
    if (!np)
    {
        pr_err("get device tree node failed!\n ");
        return -1;
    }

    /* get VADC */
    chip_battid = qpnp_get_vadc(&(pdev->dev), "battid");
    if (IS_ERR(chip_battid))
    {
        rc = PTR_ERR(chip_battid);
        if (rc != -EPROBE_DEFER)
        {
            pr_err("battid vadc property missing\n");
        }
        return rc;
    }

    /* initialize global batt id */
    global_batt_id = get_batt_id(chip_battid);
    pr_info("global batt id is %d\n", global_batt_id);

    return 0;
}

static int battid_remove(struct platform_device *pdev)
{
    return 0;
}

static struct of_device_id battid_match_table[] = {
    {.compatible = "huawei,battid"},
    {},
};

static struct platform_driver battid_driver = {
    .probe  = battid_probe,
    .remove = battid_remove,
    .driver = {
        .name           = "battid",
        .owner          = THIS_MODULE,
        .of_match_table = battid_match_table,
    },
};

static int __init battid_init(void)
{
    return platform_driver_register(&battid_driver);
}

static void __exit battid_exit(void)
{
    platform_driver_unregister(&battid_driver);
}

module_init(battid_init);
module_exit(battid_exit);

MODULE_AUTHOR("hw Inc.");
MODULE_DESCRIPTION("Driver for battery id");

