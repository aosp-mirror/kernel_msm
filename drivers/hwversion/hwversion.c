/* Copyright (c) 2016,  HUAWEI TECHNOLOGIES CO., LTD.  All rights reserved.
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
#include <soc/qcom/smem.h>
#include <linux/pinctrl/consumer.h>

#define HW_VER_PROC             "hw_ver_numb"
#define PROC_MODE               (0666)
#define HWVER_SMEM_SIZE         8

#define MAX_ADC_RANGE           9
#define HWERR                  -1

#define BT_MOD_INDEX            0
#define SIM_ESIM_INDEX          1
#define GPS_INDEX               2
#define NFC_INDEX               3
#define HIGH_LOW_INDEX          4
#define FREQ_RANGE_INDEX        5
#define PCB_INDEX               6

#define HWVER1_GPIO_NAME        "hwver-gpio1"
#define HWVER2_GPIO_NAME        "hwver-gpio2"
#define HWVER3_GPIO_NAME        "hwver-gpio3"
#define PCBVER1_GPIO_NAME       "pcbver-gpio1"
#define PCBVER2_GPIO_NAME       "pcbver-gpio2"
#define HW_VADC_NAME            "hwver0"

typedef struct
{
    int pcbver_gpio1;
    int pcbver_gpio2;
    int hwver_gpio1;
    int hwver_gpio2;
    int hwver_gpio3;
    struct qpnp_vadc_chip *vadc;
    enum qpnp_vadc_channels channel;
    int adc_range;
    unsigned int pcb1_status;
    unsigned int pcb2_status;
    unsigned int hwver1_status;
    unsigned int hwver2_status;
    unsigned int hwver3_status;
}hwver_struct;

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

typedef enum
{
    BT_VER = 0,
    MODEM_VER
}modem_bt_type;

typedef enum
{
    SIM_VER = 0,
    ESIM_VER
}sim_esim_type;

typedef enum
{
    UBLOX = 0,
    QCOM
}gps_type;

typedef enum
{
    PN66T = 0,
    PN551
}nfc_type;

typedef enum
{
    LOW = 0,
    HIGH
}config_type;

char version[HWVER_SMEM_SIZE] = {0};
char *smem = NULL;

static int hw_get_high_low_config(int adc_range)
{
    if((adc_range < 0) || (adc_range > MAX_ADC_RANGE))
    {
        printk(KERN_ERR "hwver:%s:adc_range is invalid!\n",__func__);
        return HWERR;
    }
    if(adc_range % 2)
    {
        return HIGH;
    }
    else
    {
        return LOW;
    }
}

static int hw_get_adc_range(hwver_struct *hardware)
{
    int rc = 0;
    int i = 0;
    struct qpnp_vadc_result results_ver = {0};

    if((NULL == hardware) || (NULL == hardware->vadc))
    {
        printk(KERN_ERR "hwver:%s: hardware or vadc is NULL.\n", __func__);
        return HWERR;
    }

    rc = qpnp_vadc_read(hardware->vadc, hardware->channel, &results_ver);
    if (rc)
    {
        printk(KERN_ERR "hwver:%s:unable to read channel = %d\n", __func__, hardware->channel);
        return HWERR;
    }

    rc = (results_ver.physical >> 10);  /*uv to mv*/

    for(i = 0; i < ARRAY_SIZE(g_vol_table); i++)
    {
        if((rc < g_vol_table[i].hig_vol) && (rc > g_vol_table[i].low_vol))
        {
            break;
        }
    }

    if(i >= ARRAY_SIZE(g_vol_table))
    {
        printk(KERN_ERR "hwver:%s:read rc = %d, error channel = %d\n", __func__, rc, hardware->channel);
        return HWERR;
    }
    hardware->adc_range = i;

    return 0;
}

static void hw_set_pcb_version(int pcb_gpio1_status, int pcb_gpio2_status)
{
    int val = 0;
    val |= (pcb_gpio1_status & 0x1) << 1;
    val |= (pcb_gpio2_status & 0x1);
    version[PCB_INDEX] = '0' + val;
}

static int hw_calc_hardware_version(hwver_struct *hardware)
{
    modem_bt_type modem_bt = BT_VER;
    sim_esim_type sim_esim = SIM_VER;
    gps_type gps = UBLOX;
    nfc_type nfc = PN66T;
    config_type high_low = LOW;
    int val = 0;

    if(NULL == hardware)
    {
        printk(KERN_ERR "hwver:%s:hardware pointer is null!\n", __func__);
        return HWERR;
    }
    if((hardware->adc_range > MAX_ADC_RANGE) || (hardware->adc_range < 0))
    {
        printk(KERN_ERR "hwver:%s:adc_range is invalid!\n", __func__);
        return HWERR;
    }

    val |= ((hardware->hwver1_status & 0x01) << 2);
    val |= ((hardware->hwver2_status & 0x01) << 1);
    val |= (hardware->hwver3_status & 0x01);
    switch(val)
    {
        /*GPIO: 0 0 0*/
        /*BTMODEM:BT GPS:UBLOX SIMESIM:NONE*/
        case 0:
        {
            modem_bt = BT_VER;
            gps = UBLOX;
            /*NFC*/
            if((hardware->adc_range > 4) && (hardware->adc_range <= MAX_ADC_RANGE))
            {
                nfc = PN551;
            }
            else
            {
                nfc = PN66T;
            }
            /*high or low configuration*/
            high_low = hw_get_high_low_config(hardware->adc_range);
            break;
        }
        /*GPIO: 0 0 1*/
        /*BTMODEM:MODEM SIMESIM:SIM NFC:PN66T GPS:UBLOX HIGHLOW:default LOW*/
        case 1:
        {
            modem_bt = MODEM_VER;
            sim_esim = SIM_VER;
            gps = UBLOX;
            nfc = PN66T;
            break;
        }
        /*GPIO: 0 1 X*/
        /*BTMODEM:MODEM SIMESIM:SIM NFC:PN551 GPS:UBLOX HIGHLOW default LOW*/
        case 2:
        case 3:
        {
            modem_bt = MODEM_VER;
            sim_esim = SIM_VER;
            gps = UBLOX;
            nfc = PN551;
            break;
        }
        /*GPIO: 1 0 0*/
        /*BTMODEM:MODEM SIMESIM:ESIM NFC:PN551 GPS:QCOM*/
        case 4:
        {
            modem_bt = MODEM_VER;
            sim_esim = ESIM_VER;
            /*get high or low config*/
            high_low = hw_get_high_low_config(hardware->adc_range);
            gps = QCOM;
            nfc = PN551;
            break;
        }
        /*GPIO: 1 0 1*/
        /*BTMODEM:MODEM SIMESIM:ESIM NFC:PN66T GPS:UBLOX*/
        case 5:
        {
            modem_bt = MODEM_VER;
            sim_esim = ESIM_VER;
            /*get high or low config*/
            high_low = hw_get_high_low_config(hardware->adc_range);
            gps = UBLOX;
            nfc = PN66T;
            break;
        }
        /*GPIO: 1 1 X*/
        /*MODEMBT:MODEM SIMESIM:ESIM GPS:UBLOX NFC:PN551*/
        case 6:
        case 7:
        {
            modem_bt = MODEM_VER;
            sim_esim = ESIM_VER;
            /*get high or low config*/
            high_low = hw_get_high_low_config(hardware->adc_range);
            gps = UBLOX;
            nfc = PN551;
            break;
        }
        /*ERROR*/
        default:
        {
            printk(KERN_ERR "hwver:%s:hardware version is invalid!\n", __func__);
            return HWERR;
        }
    }

    version[BT_MOD_INDEX] = '0' + modem_bt;
    version[SIM_ESIM_INDEX] = '0' + sim_esim;
    version[GPS_INDEX] = '0' + gps;
    version[NFC_INDEX] = '0' + nfc;
    version[HIGH_LOW_INDEX] = '0' + high_low;
    version[FREQ_RANGE_INDEX] = '0' + hardware->adc_range;

    return 0;
}

static bool hw_is_gpios_valid(hwver_struct *hardware)
{
    bool val = false;

    if(NULL == hardware)
    {
        printk(KERN_ERR "hwver:%s:hardware pointer is null\n", __func__);
        return HWERR;
    }
    val = gpio_is_valid(hardware->hwver_gpio1) && gpio_is_valid(hardware->hwver_gpio2) && gpio_is_valid(hardware->hwver_gpio3) && gpio_is_valid(hardware->pcbver_gpio1) && gpio_is_valid(hardware->pcbver_gpio2);
    if(val)
    {
        return true;
    }
    return false;
}

static int hw_set_gpios_direction_input(hwver_struct *hardware)
{
    bool val = false;

    val = gpio_direction_input(hardware->pcbver_gpio1) || gpio_direction_input(hardware->pcbver_gpio2) || gpio_direction_input(hardware->hwver_gpio1) || gpio_direction_input(hardware->hwver_gpio2) || gpio_direction_input(hardware->hwver_gpio3);
    if(val)
    {
        return HWERR;
    }
    return 0;
}

static int hw_init(struct platform_device *pdev, hwver_struct *hardware)
{
    struct device_node *np  = NULL;
    struct pinctrl* hwver_pinctrl = NULL;
    struct pinctrl_state *set_state = NULL;
    int rc = 0;

    /*get device tree node*/
    np = pdev->dev.of_node;
    if (!np)
    {
        printk(KERN_ERR "hwver:%s:get device tree node failed!\n", __func__);
        return HWERR;
    }
    /*  VADC get */
    hardware->vadc = qpnp_get_vadc(&(pdev->dev), HW_VADC_NAME);
    if (IS_ERR(hardware->vadc))
    {
        rc = PTR_ERR(hardware->vadc);
        if (rc != -EPROBE_DEFER)
        {
            printk(KERN_ERR "hwver:%s:vadc property missing\n", __func__);
            return HWERR;
        }
    }
    /*Get all gpios*/
    hardware->hwver_gpio1 = of_get_named_gpio(np, HWVER1_GPIO_NAME, 0);
    hardware->hwver_gpio2 = of_get_named_gpio(np, HWVER2_GPIO_NAME, 0);
    hardware->hwver_gpio3 = of_get_named_gpio(np, HWVER3_GPIO_NAME, 0);
    hardware->pcbver_gpio1 = of_get_named_gpio(np, PCBVER1_GPIO_NAME, 0);
    hardware->pcbver_gpio2 = of_get_named_gpio(np, PCBVER2_GPIO_NAME, 0);

    if(!hw_is_gpios_valid(hardware))
    {
        printk(KERN_ERR "hwver:%s:GPIO get failed!\n", __func__);
        return HWERR;
    }
    /*Set GPIO default configs*/
    hwver_pinctrl = devm_pinctrl_get(&(pdev->dev));
    if(!hwver_pinctrl)
    {
        printk(KERN_ERR "hwver:%s:pinctrl get failed\n", __func__);
        return HWERR;
    }

    set_state = pinctrl_lookup_state(hwver_pinctrl, "default");
    if(!set_state)
    {
        printk(KERN_ERR "hwver:%s:can not find pinctrl setstate\n", __func__);
        return HWERR;
    }
    rc = pinctrl_select_state(hwver_pinctrl, set_state);
    if(rc != 0)
    {
        printk(KERN_ERR "hwver:%s:can not select pinctrl setstate\n", __func__);
        return HWERR;
    }

    hardware->channel = P_MUX4_1_1;
    return 0;
}

static int hw_gpio_request_confg_in(hwver_struct *hardware)
{
    int rc = 0;

    rc = gpio_request(hardware->pcbver_gpio1, PCBVER1_GPIO_NAME);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:request GPIO:%d failed\n", __func__, hardware->pcbver_gpio1);
        goto gpio_request_exit5;
    }

    rc = gpio_request(hardware->pcbver_gpio2, PCBVER2_GPIO_NAME);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:request GPIO:%d failed\n", __func__, hardware->pcbver_gpio2);
        goto gpio_request_exit4;
    }

    rc = gpio_request(hardware->hwver_gpio1,HWVER1_GPIO_NAME);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:request GPIO:%d failed\n", __func__, hardware->hwver_gpio1);
        goto gpio_request_exit3;
    }

    rc = gpio_request(hardware->hwver_gpio2,HWVER2_GPIO_NAME);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:request GPIO:%d failed\n", __func__, hardware->hwver_gpio2);
        goto gpio_request_exit2;
    }

    rc = gpio_request(hardware->hwver_gpio3,HWVER3_GPIO_NAME);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:request GPIO:%d failed\n", __func__, hardware->hwver_gpio3);
        goto gpio_request_exit1;
    }

    if(hw_set_gpios_direction_input(hardware))
    {
        printk(KERN_ERR "hwver:%s:GPIO input_mode set failed\n", __func__);
        goto gpio_request_exit;
    }
    return 0;

gpio_request_exit:
    gpio_free(hardware->hwver_gpio3);
gpio_request_exit1:
    gpio_free(hardware->hwver_gpio2);
gpio_request_exit2:
    gpio_free(hardware->hwver_gpio1);
gpio_request_exit3:
    gpio_free(hardware->pcbver_gpio2);
gpio_request_exit4:
    gpio_free(hardware->pcbver_gpio1);
gpio_request_exit5:
    return HWERR;
}

static void hw_gpio_free(hwver_struct *hardware)
{
    gpio_free(hardware->pcbver_gpio1);
    gpio_free(hardware->pcbver_gpio2);
    gpio_free(hardware->hwver_gpio1);
    gpio_free(hardware->hwver_gpio2);
    gpio_free(hardware->hwver_gpio3);
}

static int hw_get_gpio_status(hwver_struct *hardware)
{
    int rc = 0;

    if(NULL == hardware)
    {
        printk(KERN_ERR "hwver:%s:hardware pointer is null\n", __func__);
        return HWERR;
    }
    rc = hw_gpio_request_confg_in(hardware);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:gpio request fail!\n", __func__);
        return HWERR;
    }
    hardware->pcb1_status = (unsigned int)gpio_get_value_cansleep(hardware->pcbver_gpio1);
    hardware->pcb2_status = (unsigned int)gpio_get_value_cansleep(hardware->pcbver_gpio2);
    hardware->hwver1_status = (unsigned int)gpio_get_value_cansleep(hardware->hwver_gpio1);
    hardware->hwver2_status = (unsigned int)gpio_get_value_cansleep(hardware->hwver_gpio2);
    hardware->hwver3_status = (unsigned int)gpio_get_value_cansleep(hardware->hwver_gpio3);
    hw_gpio_free(hardware);

    return 0;
}

ssize_t hwver_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    int len = 0;
    len = sizeof(version);

    if((NULL == ppos) || (*ppos > len))
    {
        return 0;
    }
    if(count > (len - *ppos))
    {
        count = len - *ppos;
    }
    ret = copy_to_user(buf, version, count);
    if(ret)
    {
        printk(KERN_ERR"hwver:%s:hwver_proc_read fail.\n", __func__);
        return HWERR;
    }

    *ppos += count;
    return count;
}

ssize_t hwver_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int ret;
    char c = 0;

    if(count < 1)
    {
        return HWERR;
    }

    ret = copy_from_user(&c, buf, 1);
    if(ret)
    {
        printk(KERN_ERR"hwver:%s:hwver_proc_write fail.\n", __func__);
        return HWERR;
    }

    if('1' == c)
    {
        printk(KERN_INFO "hwver:write smem LEOXXU\n");
        smem[HWVER_SMEM_SIZE-1] = '1';
    }
    else
    {
        printk(KERN_ERR "hwver:write smem LEOXXE\n");
        smem[HWVER_SMEM_SIZE-1] = '0';
    }

    *ppos += count;
    return count;
}

static const struct file_operations hwver_proc_fops = {
    .read = hwver_proc_read,
    .write = hwver_proc_write,
};

static int hwver_probe(struct platform_device *pdev)
{
    int rc = 0;
    hwver_struct hardware;
    static struct proc_dir_entry *hw_version = NULL;

    smem = (char *) smem_alloc(SMEM_ID_VENDOR2, HWVER_SMEM_SIZE, 0,SMEM_ANY_HOST_FLAG);
    if(NULL == smem)
    {
        printk(KERN_ERR "hwver:%s:alloc share memory fail!\n", __func__);
        return HWERR;
    }
    memset(smem, 0, HWVER_SMEM_SIZE);

    rc = hw_init(pdev, &hardware);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:hardware init fail!\n", __func__);
        return HWERR;
    }

    rc = hw_get_adc_range(&hardware);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:get adc_range fail!\n", __func__);
        return HWERR;
    }

    rc = hw_get_gpio_status(&hardware);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:get gpio status fail!\n", __func__);
        return HWERR;
    }

    rc = hw_calc_hardware_version(&hardware);
    if(rc)
    {
        printk(KERN_ERR "hwver:%s:hardware version calculate error!\n", __func__);
        return HWERR;
    }

    hw_set_pcb_version(hardware.pcb1_status, hardware.pcb2_status);

    memcpy(smem, version, HWVER_SMEM_SIZE);

    hw_version = proc_create(HW_VER_PROC, PROC_MODE, NULL, &hwver_proc_fops);
    if(NULL == hw_version)
    {
        printk(KERN_ERR "hwver:%s:can't creat /proc/%s .\n", __func__, HW_VER_PROC);
        return HWERR;
    }

    printk(KERN_INFO "hwver:hardware version is %s\n", version);
    return 0;
}

static struct of_device_id hwver_match_table[] = {
    {.compatible = "huawei,hwversion"},
    {},
};

static int hwver_remove(struct platform_device *pdev)
{
    remove_proc_entry(HW_VER_PROC, NULL);
    return 0;
}

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

