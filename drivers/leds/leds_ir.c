/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/platform_device.h>

int gpio_ir0_pin = -1;
int gpio_ir1_pin = -1;
int gpio_ir0_flag = -1;
int gpio_ir1_flag = -1;
static struct class *gpio_ir_power_class = NULL;
static struct device *gpio_ir_power_dev = NULL;

#define CTL_POWER_ON    "1"
#define CTL_POWER_OFF   "0"

static ssize_t gpio_ir_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
       printk("%s\n", __func__);
       sprintf(buf, "gpio_ir0=%d gpio_ir1=%d\n", gpio_ir0_flag, gpio_ir1_flag);
       return strlen(buf);
}

static ssize_t gpio_ir_store(struct device *dev,
        struct device_attribute *attr, const char *buf,
        size_t count)
{
       if(!strncmp(buf, CTL_POWER_ON, strlen(CTL_POWER_ON))) {
                printk("%s: to enable gpio_ir\n", __func__);
               gpio_set_value_cansleep(gpio_ir0_pin, 1);
               gpio_ir0_flag = 1;
               gpio_set_value_cansleep(gpio_ir1_pin, 1);
               gpio_ir1_flag = 1;

       } else if(!strncmp(buf, CTL_POWER_OFF, strlen(CTL_POWER_OFF))) {
                       printk("%s: to disable gpio_ir\n", __func__);
               gpio_set_value_cansleep(gpio_ir0_pin, 0);
               gpio_ir0_flag = 0;
               gpio_set_value_cansleep(gpio_ir1_pin, 0);
               gpio_ir1_flag = 0;
       }

       return count;
}

static struct device_attribute gpio_ir_dev_attr = {
       .attr = {
                       .name = "gpio_ir",
               .mode = S_IRWXU|S_IRWXG|S_IRWXO,
       },
       .show = gpio_ir_show,
       .store = gpio_ir_store,
};

static int gpio_ir_power_probe(struct platform_device *pdev)
{
       struct device *gpio_ir_power_dev;
       int ret = 0;
       unsigned int default_on = 0;
       printk("Entering gpio_ir_power_probe\n");

       default_on = of_property_read_bool(pdev->dev.of_node, "default-on");    
       gpio_ir0_pin = of_get_named_gpio(pdev->dev.of_node, "qcom,gpio_ir0_pin", 0);
       pr_err("%s gpio_ir0_pin:%d\n", __func__, gpio_ir0_pin);
       if (gpio_ir0_pin < 0)
                       printk("gpio_ir0_pin is not available \n");

       gpio_ir1_pin = of_get_named_gpio(pdev->dev.of_node, "qcom,gpio_ir1_pin", 0);
       if (gpio_ir1_pin < 0)
                       printk("gpio_ir1_pin is not available \n");

       ret = gpio_request(gpio_ir0_pin, "gpio_ir0_pin");
       if(0 != ret) {
               printk("gpio request %d failed.", gpio_ir0_pin);
               goto fail1;
       }
    
       ret = gpio_request(gpio_ir1_pin, "gpio_ir1_pin");
       if(0 != ret) {
               printk("gpio request %d failed.", gpio_ir1_pin);
               goto fail1;
       }
       pr_err("%s gpio_ir0_pin:%d\n", __func__, gpio_ir0_pin);
       pr_err("%s gpio_ir1_pin:%d\n", __func__, gpio_ir1_pin);

       gpio_direction_output(gpio_ir0_pin, 0);
       gpio_direction_output(gpio_ir1_pin, 0);
       if(default_on){
               gpio_set_value_cansleep(gpio_ir0_pin, 1);
               gpio_ir0_flag = 1;

               gpio_set_value_cansleep(gpio_ir1_pin, 1);
               gpio_ir1_flag = 1;
       }else{
               gpio_set_value_cansleep(gpio_ir0_pin, 0);
               gpio_ir0_flag = 0;

               gpio_set_value_cansleep(gpio_ir1_pin, 0);
               gpio_ir1_flag = 0;
       }

       gpio_ir_power_class = class_create(THIS_MODULE, "gpio_ir_power");
       if(IS_ERR(gpio_ir_power_class)){
               ret = PTR_ERR(gpio_ir_power_class);
               printk("Failed to create class.\n");
               return ret;
       }

       gpio_ir_power_dev = device_create(gpio_ir_power_class, NULL, 0, NULL, "gpio_gpio_ir");
       if (IS_ERR(gpio_ir_power_dev)){
               ret = PTR_ERR(gpio_ir_power_class);
               printk("Failed to create device(gpio_ir_power_dev)!\n");
               return ret;
       }

       ret = device_create_file(gpio_ir_power_dev, &gpio_ir_dev_attr);
       if(ret){
                       pr_err("%s: gpio_ir creat sysfs failed\n",__func__);
               return ret;
       }

       printk("End of gpio_ir_power_probe\n");

fail1:
    return ret;
}
static int gpio_ir_power_remove(struct platform_device *pdev)
{
       device_destroy(gpio_ir_power_class, 0);
       class_destroy(gpio_ir_power_class);
       device_remove_file(gpio_ir_power_dev, &gpio_ir_dev_attr);

       return 0;
}

static struct of_device_id gpio_ir_power_dt_match[] = {
       { .compatible = "qcom,gpio_ir_power",},
       { },
};
MODULE_DEVICE_TABLE(of, gpio_ir_power_dt_match);

static struct platform_driver gpio_ir_driver = {
       .driver = {
               .name = "gpio_ir_power",
               .owner = THIS_MODULE,
               .of_match_table = of_match_ptr(gpio_ir_power_dt_match),
       },
       .probe = gpio_ir_power_probe,
       .remove = gpio_ir_power_remove,
       //.suspend = gpio_ir_power_suspend,
       //.resume = gpio_ir_power_resume,
};
module_platform_driver(gpio_ir_driver);
#if 0
static __init int gpio_ir_init(void)
{
    return platform_driver_register(&gpio_ir_driver);
}

static void __exit gpio_ir_exit(void)
{
    platform_driver_unregister(&gpio_ir_driver);
}
module_init(gpio_ir_init);
module_exit(gpio_ir_exit);
#endif
MODULE_AUTHOR("GPIO_IR_POWER, Inc.");
MODULE_DESCRIPTION("QCOM,GPIO_IR_POWER");
MODULE_LICENSE("GPL");

