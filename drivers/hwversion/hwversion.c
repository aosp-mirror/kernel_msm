/*
** =============================================================================
** Copyright (c) 2015 hw Inc.
** File:
**     hwversion.c
**
** Description:
**     hwversion
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License version 2 and
** only version 2 as published by the Free Software Foundation.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
** =============================================================================
*/
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


#define HW_VER_PROC "hw_ver_numb"
#define PROC_MODE (0666)

typedef struct
{
    int low_vol;
    int hig_vol;
}vol_range;

/*ADC voltage table unit:mv*/
static const vol_range g_vol_table[] =
{
    {0,150},
    {151,300},
    {301,500},
    {501,700},
    {701,900},
    {901,1100},
    {1101,1300},
    {1301,1500},
    {1501,1650},
    {1651,1800},
};

static int hwver_num0 = 0;
static int pcb_num1 = 0;
static int pcb_num2 = 0;


/**************************************************************************
FUNCTION   get_hwver_num

DESCRIPTION :Return hardware version on success, -1 otherwise

**************************************************************************/
int get_hwver_num(struct qpnp_vadc_chip *vadc, enum qpnp_vadc_channels channel)
{
    int rc = 0;
    int i = 0;
    struct qpnp_vadc_result results_ver = {0};

    if(NULL == vadc)
    {
        printk(KERN_ERR "get_hwver_num ERROR.channel = %d \n", channel);
        return -1;
    }
    rc = qpnp_vadc_read(vadc, channel, &results_ver);

    if (rc)
    {
        printk(KERN_ERR "Unable to read channel = %d\n", channel);
        return -1;
    }

    rc = (results_ver.physical >> 10);  /*uv to mv*/
    printk(KERN_ERR "rc=%d\n",rc);
    for(i = 0;i < ARRAY_SIZE(g_vol_table);i++)
    {
        if((rc < g_vol_table[i].hig_vol) && (rc > g_vol_table[i].low_vol))
            break;
    }

    if(i >= ARRAY_SIZE(g_vol_table))
    {
        printk(KERN_ERR "read rc = %d, error channel = %d\n", rc, channel);
        return -1;
    }
    return i;
}

/**************************************************************************
FUNCTION   hwver_proc_read

DESCRIPTION: Transfer hardware version to user

**************************************************************************/
ssize_t hwver_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char num[3]= {0};
    int len = 0;
    len = sizeof(num);

    num[0] = '0' + hwver_num0;
    num[1] = '0' + (( ( pcb_num1 & 0x1 ) << 1 ) | ( pcb_num2 & 0x1 ));
    num[2] = '\n';
    if(NULL == ppos || *ppos > len)
    {
        return 0;
    }
    if(count > len - *ppos)
    {
        count = len - *ppos;
    }

    ret = copy_to_user(buf, num, count);
    if(0 != ret)
    {
        printk(KERN_ERR"hwver_proc_read fail.\n");
        return -1;
    }

    *ppos += count;
    return count;
}

static const struct file_operations hwver_proc_fops = {
    .read = hwver_proc_read,
};

static int hwver_probe(struct platform_device *pdev)
{
    int rc = 0;
    int pcbver_gpio1 = -1;     /*pcb_ver0 gpio*/
    int pcbver_gpio2 = -1;     /*pcb_ver1 gpio*/
    int pcb1_err = 0;
    int pcb1_status = -1;
    int pcb2_err = 0;
    int pcb2_status = -1;
    struct device_node *np  = NULL;
    static struct proc_dir_entry *hw_version = NULL;
    struct qpnp_vadc_chip *g_chip_ver0 = NULL;
    struct pinctrl* hwver_pinctrl = NULL;
    struct pinctrl_state *set_state = NULL;

    printk(KERN_ERR "hwver_probe begin...\n");

    /*get device tree node*/
    np = pdev->dev.of_node;
    if (!np)
    {
        printk(KERN_ERR "get device tree node failed!\n ");
        return -1;
    }

    /*  VADC get */
    g_chip_ver0 = qpnp_get_vadc(&(pdev->dev), "hwver0");
    if (IS_ERR(g_chip_ver0))
    {
        rc = PTR_ERR(g_chip_ver0);
        if (rc != -EPROBE_DEFER)
        {
            printk(KERN_ERR "hwver0 vadc property missing\n");
            return -1;
        }
    }

    /*get hardware version*/
    hwver_num0 = get_hwver_num(g_chip_ver0, P_MUX4_1_1);
    if(0 > hwver_num0)
    {
        printk(KERN_ERR "P_MUX4_1_1 get error hwver_num0 = %d.\n", hwver_num0);
        return -1;
    }

    /*get PCB version GPIOs */
    pcbver_gpio1 = of_get_named_gpio(np, "pcbver1-gpios", 0);
    if ( !gpio_is_valid(pcbver_gpio1) )
    {
        printk(KERN_ERR "PCB version GPIO1 get failed!\n ");
        return -1;
    }

    pcbver_gpio2 = of_get_named_gpio(np, "pcbver2-gpios", 0);
    if (!gpio_is_valid(pcbver_gpio2))
    {
        printk(KERN_ERR "PCB version GPIO2 get failed!\n ");
        return -1;
    }

    /*set the pinctrl state*/
    hwver_pinctrl = devm_pinctrl_get(&(pdev->dev));
    if(!hwver_pinctrl)
    {
        printk(KERN_ERR "pinctrl get failed\n");
        return -1;
    }
    set_state = pinctrl_lookup_state(hwver_pinctrl,"default");
    if(!set_state)
    {
        printk(KERN_ERR "can not find pinctrl setstate\n");
        return -1;
    }
    pinctrl_select_state(hwver_pinctrl,set_state);

    /*get gpio status*/
    pcb1_err = gpio_request(pcbver_gpio1,"pcbver1-gpios");
    if (pcb1_err)
    {
        printk(KERN_ERR "request gpio82 failed\n");
        return -1;
    }
    if ( gpio_direction_input(pcbver_gpio1) )
    {
        printk(KERN_ERR "GPIO input_mode set failed\n");
        return -1;
    }
    pcb1_status = gpio_get_value_cansleep(pcbver_gpio1);
    if ( pcb1_status != 0 && pcb1_status != 1 )
    {
        printk(KERN_ERR "gpio82 status error\n");
        return -1;
    }

    pcb2_err = gpio_request(pcbver_gpio2,"pcbver2-gpios");
    if (pcb2_err)
    {
        printk(KERN_ERR "request gpio83 failed\n");
        return -1;
    }
    if ( gpio_direction_input(pcbver_gpio2) )
    {
        printk(KERN_ERR "GPIO input_mode set failed\n");
        return -1;
    }
    pcb2_status = gpio_get_value_cansleep(pcbver_gpio2);
    if ( pcb2_status != 0 && pcb2_status != 1 )
    {
        printk(KERN_ERR "gpio83 status error\n");
        return -1;
    }

    pcb_num1 = pcb1_status;
    pcb_num2 = pcb2_status;

    hw_version = proc_create(HW_VER_PROC, PROC_MODE, NULL, &hwver_proc_fops);
    if(NULL == hw_version)
    {
        printk(KERN_ERR "can't creat /proc/%s .\n", HW_VER_PROC);
        return -1;
    }

    return 0;
}


static int hwver_remove(struct platform_device *pdev)
{
    remove_proc_entry(HW_VER_PROC, NULL);
    return 0;
}

static struct of_device_id hwver_match_table[] = {
    {.compatible = "qcom,hwversion"},
    {},
};

static struct platform_driver hwver_driver = {
    .probe  = hwver_probe,
    .remove = hwver_remove,
    .driver = {
        .name           = "hardware_version",
        .owner          = THIS_MODULE,
        .of_match_table = hwver_match_table,
    },
};

static int __init hwver_init(void)
{
    return platform_driver_register(&hwver_driver);
}

static void __exit hwver_exit(void)
{
    platform_driver_unregister(&hwver_driver);
}

module_init(hwver_init);
module_exit(hwver_exit);

MODULE_AUTHOR("hw Inc.");
MODULE_DESCRIPTION("Driver for hardware version");

