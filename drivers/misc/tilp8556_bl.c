/*
 * TI LP8556 Backlight Driver 
 * Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
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
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define LP8556_CFG98            0x98
#define LP8556_CFG9E            0x9E
#define LP8556_CFG0             0xA0
#define LP8556_CFG1             0xA1
#define LP8556_CFG2             0xA2
#define LP8556_CFG3             0xA3
#define LP8556_CFG4             0xA4
#define LP8556_CFG5             0xA5
#define LP8556_CFG6             0xA6
#define LP8556_CFG7             0xA7
#define LP8556_CFG8             0xA8
#define LP8556_CFG9             0xA9
#define LP8556_CFGA             0xAA
#define LP8556_CFGB             0xAB
#define LP8556_CFGC             0xAC
#define LP8556_CFGD             0xAD
#define LP8556_CFGE             0xAE
#define LP8556_CFGF             0xAF
#define LP8556_BR_CTRL          0x00
#define LP8556_DEV_CNTRL        0x01
#define LP8556_STATUS           0x02
#define LP8556_ID               0x03
#define LP8556_DIR_CNTRL        0x04
#define LP8556_LED_EN           0x16

static int reg_array[] = {
LP8556_CFG98,0xa3,
LP8556_CFG9E,0x20,
LP8556_CFG0,0xCB,
LP8556_CFG1,0xAB,
LP8556_CFG2,0x28,
LP8556_CFG3,0x02,
LP8556_CFG4,0x72,
LP8556_CFG5,0x84,
LP8556_CFG6,0x80,
LP8556_CFG7,0x36,
//LP8556_CFG8,0x00, RESERVED
LP8556_CFG9,0xE0,
LP8556_CFGA,0x0F,
//LP8556_CFGB,0x00, RO RESERVED
//LP8556_CFGC,0x00, RO RESERVED
//LP8556_CFGD,0x00, RO RESERVED
LP8556_CFGE,0x0F,
//LP8556_CFGF,0x63, RO RESERVED
LP8556_BR_CTRL,0x80,
LP8556_DEV_CNTRL,0x84,
//LP8556_STATUS,0x00 
//LP8556_ID,0x0, 
LP8556_DIR_CNTRL,0x0,
LP8556_LED_EN,0x0F
};

struct lp8556_bl {
       struct i2c_client *client;
       int lcd_en_gpio;
};

static int lp8556_write(struct i2c_client *client, char *writebuf, int writelen)
{
       int ret;

       if (!client || !writebuf) {
                return -EINVAL;
       }
       if((ret=i2c_master_send(client, writebuf, writelen))!=writelen)
               pr_err("%s:i2c write failed, ret:%d\n",__func__, ret);

       return ret;
}

static int lp8556_read(struct i2c_client *client,char *writebuf,int writelen,
                        char *readbuf,int readlen)
{

       int ret = 0;

       if((!client) || (!writebuf) || (!readbuf))
               return -EINVAL;

       if((ret=i2c_master_send(client, writebuf, writelen))!=writelen)
           pr_err("%s:i2c write failed\n",__func__);

       if(readlen != 0)
               ret = i2c_master_recv(client, readbuf, readlen);

       return ret;
}

static int lp8556_gpio_configure(struct lp8556_bl *lp8556, bool on)
{
       struct i2c_client *client = lp8556->client;
       struct device *dev = &client->dev;
       int err = 0;
       if (on) {
               if (gpio_is_valid(lp8556->lcd_en_gpio)
               && (!devm_gpio_request(dev, lp8556->lcd_en_gpio,
                       "lp8556_lcden_gpio")))
               {
                       /* lcd_en lp8556 */
                       gpio_direction_output(lp8556->lcd_en_gpio, 1);
                       gpio_set_value_cansleep(lp8556->lcd_en_gpio, 1);
                       
               } else {
                   pr_err("failed to request lcd_en_gpio gpio for lp8556\n");
                   err= -1;
                   goto err_lcd_en_gpio_dir;
               }
               return 0;
       } else {
          gpio_set_value_cansleep(lp8556->lcd_en_gpio, 0);
       }
err_lcd_en_gpio_dir:
       if (gpio_is_valid(lp8556->lcd_en_gpio))
               gpio_free(lp8556->lcd_en_gpio);
       return err;
}

static int lp8556_parse_dt(struct lp8556_bl *lp8556)
{
       struct i2c_client *client = lp8556->client;
       struct device *dev = &client->dev;
       struct device_node *np = dev->of_node;

       lp8556->lcd_en_gpio = of_get_named_gpio(np, "lp8556,lcden-gpio", 0);
       return lp8556_gpio_configure(lp8556, true);
}

static int lp8556_probe(struct i2c_client *client,
                           const struct i2c_device_id *id)
{
       struct lp8556_bl *lp8556;
       int error = -1;
       unsigned char writeBuf[5],readBuf[5];
       int ret =0;
       int i, size = sizeof(reg_array)/sizeof(reg_array[0]);
       

       if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
               pr_err("i2c_check_functionality error\n");
               return -ENXIO;
       }

       lp8556 = devm_kzalloc(&client->dev, sizeof(*lp8556), GFP_KERNEL);
       if (!lp8556)
               return -ENOMEM;

       lp8556->client = client;
       error = lp8556_parse_dt(lp8556);
       if (error) {
               pr_err("Failed to parse device tree, error: %d\n", error);
               return error;
       }

       i2c_set_clientdata(client, lp8556);
       memset(writeBuf, 0, sizeof(writeBuf));

       for(i=0; i < size; i+=2) {
           writeBuf[0] = reg_array[i];
           writeBuf[1] = reg_array[i+1];
           pr_err("index:%d, reg:%x, value:%x\n", i, reg_array[i], reg_array[i+1]);
           ret = lp8556_write(client, writeBuf, 2);
       }
       pr_err("%s, finished writing to eeprom\n", __func__);

       writeBuf[0] = LP8556_STATUS;
       memset(readBuf, 0, sizeof(readBuf));
       ret=lp8556_read(client,writeBuf,1,readBuf,1);
       if(ret)
           pr_err("%s:LP8556_STATUS=0x%x\n",__func__,readBuf[0]);

       pr_err("%s succeed\n",__func__);
       return 0;
}

static const struct i2c_device_id lp8556_id[] = {
       {"lp8556", 0 },
       { },
};
MODULE_DEVICE_TABLE(i2c, lp8556_id);

static struct of_device_id lp8556_of_match[] = {
       { .compatible = "ti,lp8556", },
       { }
};
MODULE_DEVICE_TABLE(of, lp8556_of_match);

static struct i2c_driver lp8556_driver = {
       .driver = {
          .name   = "lp8556",
          .owner  = THIS_MODULE,
          .of_match_table = lp8556_of_match,
       },
       .probe          = lp8556_probe,
       .id_table       = lp8556_id,
};
module_i2c_driver(lp8556_driver);

MODULE_DESCRIPTION("TI LP8556 Backlight Driver");
MODULE_LICENSE("GPL v2");

