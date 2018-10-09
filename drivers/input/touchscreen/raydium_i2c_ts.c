/* drivers/input/touchscreen/raydium_ts.c
 *
 * Raydium TouchScreen driver.
 *
 * Copyright (c) 2010  Raydium tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/of_gpio.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif //end of CONFIG_FB

#include "raydium_i2c_ts.h"
#include "RADFWImage.h"

// TODO: Using struct+memcpy instead of array+offset
enum raydium_pt_report_idx {
    POS_SEQ = 0,/*1:touch, 0:no touch*/
    POS_PT_AMOUNT,
    POS_PT_ID = 0,
    POS_X_L,
    POS_X_H,
    POS_Y_L,
    POS_Y_H,
    POS_PRESSURE_L,
    POS_PRESSURE_H,
    POS_WX_L,
    POS_WX_H,
    POS_WY_L,
    POS_WY_H,
    LENGTH_PT = 11
};

struct raydium_slot_status {
    unsigned char occupied_pt_id;      //Occupied point ID
    unsigned char need_update;         //Mark as info need to be updated
    unsigned char pt_report_offset;    //point info offset in report
};

struct raydium_slot_status gst_slot_status[MAX_TOUCH_NUM * 2];// The first 3 elements are currently occupied. therest is new coming points
struct raydium_slot_status gst_slot_init_status = {0xFF, 0, 0};

struct raydium_ts_platform_data {
    const char *name;
    u32 irqflags;
    u32 irq_gpio;
    u32 irq_gpio_flags;
    u32 reset_gpio;
    u32 reset_gpio_flags;
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 hard_rst_dly;
    u32 soft_rst_dly;
    u32 num_max_touches;
};

struct raydium_ts_data {
    unsigned int irq;
    unsigned int rst;
    unsigned int x_max;
    unsigned int y_max;
    unsigned int x_pos;
    unsigned int y_pos;
    unsigned int pressure;
    unsigned int is_suspend;
    unsigned int key_flag;
    unsigned int is_sleep;
    unsigned int is_close;
    #ifdef GESTURE_EN
    unsigned int is_palm;
    #endif

    struct i2c_client *client;
    struct input_dev *input_dev;
    struct raydium_ts_platform_data *pdata;
    struct mutex lock;
    struct work_struct  work;
    struct workqueue_struct *workqueue;
    struct irq_desc *irq_desc;
    bool irq_enabled;
    bool irq_wake;

    #if defined(CONFIG_FB)
    struct notifier_block fb_notif;
    volatile int blank;
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
    #endif //end of CONFIG_FB

    //struct regulator *vdd;
    struct regulator *vcc_i2c;
    unsigned int fw_version;

    #ifdef MSM_NEW_VER
    struct pinctrl *ts_pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
    struct pinctrl_state *pinctrl_state_release;
    #endif //end of MSM_NEW_VER
};

#if (defined(CONFIG_RM_SYSFS_DEBUG))
static const struct attribute_group raydium_attr_group;
#endif //end of CONFIG_RM_SYSFS_DEBUG

volatile unsigned char g_uc_raydium_flag = 0;
volatile unsigned char g_uc_gesture_status = 0;
volatile unsigned char g_uc_raydium_int_flag = 0;
volatile unsigned char g_uc_raydium_selftest_flag = 0;
volatile unsigned char g_uc_pre_palm_status = 0;
unsigned char u8_i2c_mode = PDA2_MODE;
unsigned char upgrade_type = 0;
unsigned char g_uc_raw_data_type = RAYDIUM_FT_UPDATE;
unsigned int g_ui_raw_data_length = 36 * 2;    // 72 bytes
unsigned long g_ul_addr = RAYDIUM_CHECK_I2C_CMD;
unsigned int g_ui_length = 1;
volatile unsigned char g_u8_unlockflag = false;

/*******************************************************************************
*  Name: raydium_gpio_configure
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/
static int raydium_gpio_configure(struct raydium_ts_data *data, bool on)
{
    int err = 0;

    if (on)
    {
        if (gpio_is_valid(data->pdata->irq_gpio))
        {
            err = gpio_request(data->pdata->irq_gpio, "raydium_irq_gpio");
            if (err)
            {
                dev_err(&data->client->dev, "[touch]irq gpio request failed");
                goto err_irq_gpio_req;
            }

            err = gpio_direction_input(data->pdata->irq_gpio);
            if (err)
            {
                dev_err(&data->client->dev, "[touch]set_direction for irq gpio failed\n");
                goto err_irq_gpio_dir;
            }
        }
        return 0;
    } else
    {
        if (gpio_is_valid(data->pdata->irq_gpio))
        {
            gpio_free(data->pdata->irq_gpio);
        }
        return 0;
    }

err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->irq_gpio))
    {
        gpio_free(data->pdata->irq_gpio);
    }
err_irq_gpio_req:
    return err;
}

/*******************************************************************************
*  Name: raydium_power_on
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/
static int raydium_power_on(struct raydium_ts_data *data, bool on)
{
    int rc = 0;

    if (!on)
    {
        goto power_off;
    }

    regulator_set_optimum_mode(data->vcc_i2c,100000);
    rc = regulator_enable(data->vcc_i2c);
    if (rc)
    {
        dev_err(&data->client->dev, "[touch]Regulator vcc_i2c enable failed rc=%d\n", rc);
    }
    return rc;

power_off:
    regulator_set_optimum_mode(data->vcc_i2c,100);
    rc = regulator_disable(data->vcc_i2c);
    if (rc)
    {
        dev_err(&data->client->dev, "[touch]Regulator vcc_i2c disable failed rc=%d\n", rc);
    }
    return rc;
}

/*******************************************************************************
*  Name: raydium_power_init
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/
static int raydium_power_init(struct raydium_ts_data *data, bool on)
{
    int rc;

    if (!on)
    {
        dev_err(&data->client->dev, "[touch]power_init false \n");
        goto pwr_deinit;
    }

    data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
    if (IS_ERR(data->vcc_i2c))
    {
        rc = PTR_ERR(data->vcc_i2c);
        dev_err(&data->client->dev, "[touch]regulator get failed vcc_i2c rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(data->vcc_i2c) > 0)
    {
        rc = regulator_set_voltage(data->vcc_i2c, I2C_VTG_MIN_UV, I2C_VTG_MAX_UV);
        if (rc)
        {
            dev_err(&data->client->dev, "[touch]regulator set_vtg failed vcc_i2c rc=%d\n", rc);
            goto reg_vcc_i2c_put;
        }
    }
    return 0;

reg_vcc_i2c_put:
    regulator_put(data->vcc_i2c);
    return rc;

pwr_deinit:
    if (regulator_count_voltages(data->vcc_i2c) > 0)
    {
        regulator_set_voltage(data->vcc_i2c, 0, I2C_VTG_MAX_UV);
    }
    regulator_put(data->vcc_i2c);
    return 0;
}

/*******************************************************************************
*  Name: raydium_ts_pinctrl_init
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/
#ifdef MSM_NEW_VER
static int raydium_ts_pinctrl_init(struct raydium_ts_data *data)
{
    int retval;

    /* Get pinctrl if target uses pinctrl */
    data->ts_pinctrl = devm_pinctrl_get(&(data->client->dev));
    if (IS_ERR_OR_NULL(data->ts_pinctrl))
    {
        retval = PTR_ERR(data->ts_pinctrl);
        dev_err(&data->client->dev, "[touch]target does not use pinctrl %d\n", retval);
        goto err_pinctrl_get;
    }

    data->pinctrl_state_active
        = pinctrl_lookup_state(data->ts_pinctrl, PINCTRL_STATE_ACTIVE);
    if (IS_ERR_OR_NULL(data->pinctrl_state_active))
    {
        retval = PTR_ERR(data->pinctrl_state_active);
        dev_err(&data->client->dev,
            "[touch]Can not lookup %s pinstate %d\n",
            PINCTRL_STATE_ACTIVE, retval);
        goto err_pinctrl_lookup;
    }

    data->pinctrl_state_suspend
        = pinctrl_lookup_state(data->ts_pinctrl,
            PINCTRL_STATE_SUSPEND);
    if (IS_ERR_OR_NULL(data->pinctrl_state_suspend))
    {
        retval = PTR_ERR(data->pinctrl_state_suspend);
        dev_err(&data->client->dev,
            "[touch]Can not lookup %s pinstate %d\n",
            PINCTRL_STATE_SUSPEND, retval);
        goto err_pinctrl_lookup;
    }

    data->pinctrl_state_release
        = pinctrl_lookup_state(data->ts_pinctrl,
            PINCTRL_STATE_RELEASE);
    if (IS_ERR_OR_NULL(data->pinctrl_state_release))
    {
        retval = PTR_ERR(data->pinctrl_state_release);
        dev_err(&data->client->dev,
            "[touch]Can not lookup %s pinstate %d\n",
            PINCTRL_STATE_RELEASE, retval);
    }

    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(data->ts_pinctrl);
err_pinctrl_get:
    data->ts_pinctrl = NULL;
    return retval;
}
#endif//end of MSM_NEW_VER

static int raydium_i2c_pda_set_address(struct raydium_ts_data *data,
        unsigned long address, unsigned char mode)
{
    int retval = 0;
    unsigned char retry;
    unsigned char buf[RAYDIUM_I2C_PDA_ADDRESS_LENGTH];
    struct i2c_client *i2c = data->client;

    u8_i2c_mode = PDA_MODE;

    i2c->addr = RAYDIUM_I2C_EID;
    buf[0] = (address & 0x0000FF00) >> 8;
    buf[1] = (address & 0x00FF0000) >> 16;
    buf[2] = (address & 0xFF000000) >> 24;
    buf[3] = mode;

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++)
    {
        retval = i2c_master_send(i2c, buf, RAYDIUM_I2C_PDA_ADDRESS_LENGTH);
        if (retval != RAYDIUM_I2C_PDA_ADDRESS_LENGTH)
        {
            dev_err(&i2c->dev,"[touch]%s: I2C retry %d\n",__func__, retry + 1);
            msleep(1);
        } else
        {
            break;
        }
    }

    return (retval == RAYDIUM_I2C_PDA_ADDRESS_LENGTH) ? retval : -EIO;
}

//device attribute raydium_i2c_pda2_mode used
static int raydium_i2c_pda_read(struct i2c_client *client,
        unsigned long addr, unsigned char *r_data, unsigned short length)
{
    int retval;
    unsigned char retry;
    unsigned char mode = 0x00;
    unsigned char buf;
    struct raydium_ts_data *data = i2c_get_clientdata(client);

    struct i2c_msg msg[] = {
        {
            .addr = RAYDIUM_I2C_NID,
            .flags = RAYDIUM_I2C_WRITE,
            .len = 1,
            .buf = &buf,
        },
        {
            .addr = RAYDIUM_I2C_NID,
            .flags = RAYDIUM_I2C_READ,
            .len = length,
            .buf = r_data,
        },
    };

    if (length == 4)
    {
        mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE | RAYDIUM_I2C_PDA_MODE_WORD_MODE;
    } else
    {
        mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE;
    }
    buf = addr & MASK_8BIT;

    retval = raydium_i2c_pda_set_address(data, addr, mode);
    if (retval != RAYDIUM_I2C_PDA_ADDRESS_LENGTH)
    {
        goto exit;
    }
    usleep_range(50, 80);

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++)
    {
        if (i2c_transfer(data->client->adapter, msg, 2) == 2)
        {
            retval = length;
            break;
        }
        dev_err(&data->client->dev,"%s: I2C retry %d\n",__func__, retry + 1);
        msleep(1);
    }

    if (retry == SYN_I2C_RETRY_TIMES)
    {
        dev_err(&data->client->dev,"%s: I2C read over retry limit\n",__func__);
        retval = -EIO;
    }
exit:
    return retval;
}

static int raydium_i2c_pda_write(struct i2c_client *client,
        unsigned long addr, unsigned char *w_data, unsigned short length)
{
    int retval;
    unsigned char retry;
    unsigned char mode = 0x00;
    unsigned char buf[MAX_WRITE_PACKET_SIZE + 1];
    struct raydium_ts_data *data = i2c_get_clientdata(client);

    struct i2c_msg msg[] = {
        {
            .addr = RAYDIUM_I2C_NID,
            .flags = RAYDIUM_I2C_WRITE,
            .len = length + 1,
            .buf = buf,
        },
    };


    if (length > MAX_WRITE_PACKET_SIZE)
    {
        return -EINVAL;
    }

    if (length == 4)
    {
        mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE | RAYDIUM_I2C_PDA_MODE_WORD_MODE;
    }
    else
    {
        mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE;
    }

    buf[0] = addr & MASK_8BIT;
    memcpy(&buf[1], w_data, length);

    retval = raydium_i2c_pda_set_address(data, addr, mode);
    if (retval != RAYDIUM_I2C_PDA_ADDRESS_LENGTH)
    {
        goto exit;
    }
    usleep_range(50, 80);

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
        {
            retval = length;
            break;
        }
        dev_err(&data->client->dev,"[touch]%s: I2C retry %d\n",__func__, retry + 1);
        msleep(1);
    }

    if (retry == SYN_I2C_RETRY_TIMES)
    {
        dev_err(&data->client->dev,"[touch]%s: I2C write over retry limit\n",__func__);
        retval = -EIO;
    }
exit:
    return retval;
}

static int raydium_i2c_pda2_set_page(struct i2c_client *client, unsigned char page)
{
    int retval = -1;
    unsigned char retry;
    unsigned char buf[RAYDIUM_I2C_PDA2_PAGE_LENGTH];
    struct raydium_ts_data *data = i2c_get_clientdata(client);

    struct i2c_msg msg[] = {
        {
            .addr = RAYDIUM_I2C_NID,
            .flags = RAYDIUM_I2C_WRITE,
            .len = RAYDIUM_I2C_PDA2_PAGE_LENGTH,
            .buf = buf,
        },
    };

    buf[0] = RAYDIUM_PDA2_PAGE_ADDR;
    buf[1] = page;

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
        {
            retval = RAYDIUM_I2C_PDA2_PAGE_LENGTH;
            break;
        }
        msleep(1);
    }

    if (retry == SYN_I2C_RETRY_TIMES)
    {
        dev_err(&data->client->dev,"[touch]%s: I2C write over retry limit\n",__func__);
        retval = -EIO;
    }

    return retval;
}

static int raydium_i2c_pda2_read(struct i2c_client *client,
        unsigned char addr, unsigned char *r_data, unsigned short length)
{
    int retval = -1;
    unsigned char retry;
    struct raydium_ts_data *data = i2c_get_clientdata(client);

    struct i2c_msg msg[] = {
        {
            .addr = RAYDIUM_I2C_NID,
            .flags = RAYDIUM_I2C_WRITE,
            .len = 1,
            .buf = &addr,
        },
        {
            .addr = RAYDIUM_I2C_NID,
            .flags = RAYDIUM_I2C_READ,
            .len = length,
            .buf = r_data,
        },
    };
    u8_i2c_mode = PDA2_MODE;
    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++)
    {
        if (i2c_transfer(data->client->adapter, msg, 2) == 2)
        {
            retval = length;
            break;
        }
        msleep(1);
    }

    if (retry == SYN_I2C_RETRY_TIMES)
    {
        dev_err(&data->client->dev,"[touch]%s: I2C read over retry limit\n",__func__);
        retval = -EIO;
    }

    return retval;
}

static int raydium_i2c_pda2_write(struct i2c_client *client,
        unsigned char addr, unsigned char *w_data, unsigned short length)
{
    int retval = -1;
    unsigned char retry;
    unsigned char buf[MAX_WRITE_PACKET_SIZE + 1];
    struct raydium_ts_data *data = i2c_get_clientdata(client);

    struct i2c_msg msg[] = {
        {
            .addr = RAYDIUM_I2C_NID,
            .flags = RAYDIUM_I2C_WRITE,
            .len = length + 1,
            .buf = buf,
        },
    };


    if (length > MAX_WRITE_PACKET_SIZE)
    {
        return -EINVAL;
    }
    u8_i2c_mode = PDA2_MODE;
    buf[0] = addr;
    memcpy(&buf[1], w_data, length);

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
        {
            retval = length;
            break;
        }
        msleep(1);
    }

    if (retry == SYN_I2C_RETRY_TIMES)
    {
        dev_err(&data->client->dev,"[touch]%s: I2C write over retry limit\n",__func__);
        retval = -EIO;
    }

    return retval;
}

static void raydium_irq_control(struct raydium_ts_data *ts, bool enable)
{
    if (enable)
    {
        if (ts->irq_enabled)
        {
            return;
        }

        /* Clear interrupts first */
        if (ts->blank != FB_BLANK_POWERDOWN)
        {
            mutex_lock(&ts->lock);
            if (raydium_i2c_pda2_set_page(ts->client, RAYDIUM_PDA2_PAGE_0) < 0)
            {
                dev_err(&ts->client->dev, "[touch]failed to set page %s\n", __func__);
            }
            mutex_unlock(&ts->lock);
            mdelay(1);
        }
        while (ts->irq_desc->depth > 0)
        {
            ts->irq_enabled = true;
            enable_irq(ts->irq);
            printk(KERN_INFO "[touch]irq enable\n");
        }
    } else
    {
        if (ts->irq_enabled)
        {
            if (ts->irq_desc->depth == 0)
            {
                disable_irq(ts->irq);
                ts->irq_enabled = false;
                printk(KERN_INFO "[touch]irq disable\n");
            }
        }
    }
}

#ifdef CONFIG_RM_SYSFS_DEBUG
static int raydium_get_palm_state(struct raydium_ts_data *raydium_ts, unsigned char pre_palm_status)
{
    unsigned char rbuffer[1];
    unsigned int ui_palm_status = 0;
    unsigned char tp_status[MAX_TCH_STATUS_PAKAGE_SIZE];
    int ret = -1;

    mutex_lock(&raydium_ts->lock);
    // Read palm state
    ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
    {
        goto exit_error;
    }
    ret = raydium_i2c_pda2_read(raydium_ts->client, RAYDIUM_PDA2_PALM_STATUS_ADDR, rbuffer, 1);
    if (ret < 0)
    {
        goto exit_error;
    }
    ui_palm_status = rbuffer[0];
    if (ui_palm_status == RAYDIUM_PALM_MODE_ENABLE || (ui_palm_status == RAYDIUM_PALM_MODE_DISABLE && pre_palm_status == RAYDIUM_PALM_MODE_ENABLE))
    {
        //clear seq num
        tp_status[POS_SEQ] = 0;
        ret = raydium_i2c_pda2_write(raydium_ts->client, RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, tp_status, 1);
        if (ret < 0)
        {
            dev_err(&raydium_ts->client->dev, "[touch]%s: failed to write data: %d\n",__func__, ret);
            goto exit_error;
        }
    }
    mutex_unlock(&raydium_ts->lock);
    return ui_palm_status;

exit_error:
    mutex_unlock(&raydium_ts->lock);
    return ret;
}

static int raydium_set_palm_state(struct raydium_ts_data *raydium_ts, unsigned char set_palm)
{
    unsigned char wbuffer[1];
    unsigned int ui_palm_status = 0;
    int ret = -1;

    mutex_lock(&raydium_ts->lock);
    // Read palm state
    ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
    {
        goto exit_error;
    }
    wbuffer[0] = set_palm;
    ret = raydium_i2c_pda2_write(raydium_ts->client, RAYDIUM_PDA2_PALM_STATUS_ADDR, wbuffer, 1);
    if (ret < 0)
    {
        goto exit_error;
    }
    mutex_unlock(&raydium_ts->lock);
    return ui_palm_status;

exit_error:
    mutex_unlock(&raydium_ts->lock);
    return ret;
}

static ssize_t raydium_touch_calibration_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    unsigned char rbuffer[1];
    int buf_len = 0;
    int ret = -1;
    unsigned char retry = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts  = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (ts->is_suspend)
    {
        printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
    }
    raydium_irq_control(ts, DISABLE);
    mutex_lock(&ts->lock);

    ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
    {
        goto exit_i2c_error;
    }
    rbuffer[0] = RAYDIUM_HOST_CMD_CALIBRATION;
    ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR, rbuffer, 1);
    if (ret < 0)
    {
        goto exit_i2c_error;
    }

    do {
        if (rbuffer[0] == RAYDIUM_HOST_CMD_NO_OP)
        {
            break;
        } else
        {
            msleep(1000);
        }
        ret = raydium_i2c_pda2_read(client, RAYDIUM_PDA2_HOST_CMD_ADDR, rbuffer, 1);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
        printk(KERN_INFO "[touch]Raydium %s return 0x%02x!!\n", __func__, rbuffer[0]);
    } while (retry++ < (SYN_I2C_RETRY_TIMES * 2));

    memcpy(buf, rbuffer, 1);

    buf_len = strlen(buf);
    ret = buf_len + 1;

exit_i2c_error:
    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);

    return ret;
}

static ssize_t raydium_check_i2c_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    unsigned char rbuffer[4];
    int buf_len = 0;
    int ret = -1;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts  = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (ts->is_suspend)
    {
        printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
    }
    raydium_irq_control(ts, DISABLE);
    mutex_lock(&ts->lock);

    if (u8_i2c_mode == PDA2_MODE)
    {
        ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
        *(unsigned long *)rbuffer = (RAYDIUM_I2C_PDA_MODE_ENABLE << 24) | ((RAYDIUM_CHECK_I2C_CMD & (~MASK_8BIT)) >> 8);    //using byte mode to read 4 bytes

        ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_PDA_CFG_ADDR, rbuffer, 4);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
        ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_ENABLE_PDA);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
        ret = raydium_i2c_pda2_read(client, (unsigned char)(RAYDIUM_CHECK_I2C_CMD& MASK_8BIT), rbuffer, 4);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
    } else
    {
        ret = raydium_i2c_pda_read(client, RAYDIUM_CHECK_I2C_CMD, rbuffer, 4);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
    }

    sprintf(buf, "[touch]Raydium Touch check i2c : %02X%02X\n", rbuffer[3], rbuffer[2]);
    buf_len = strlen(buf);
    ret = buf_len + 1;

exit_i2c_error:
    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);
    return ret;
}

static int raydium_i2c_mode_control(struct i2c_client *client, unsigned char mode)
{
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    switch (mode)
    {
        case 0:    /* Disable INT flag */
            printk(KERN_INFO "[touch]Raydium INT flag : %d\n", ts->irq_enabled);
            disable_irq(ts->irq);
            ts->irq_enabled = false;
            printk(KERN_INFO "[touch]Raydium irq disable\n");
            break;
        case 1:    /* Enable INT flag */
            printk(KERN_INFO "[touch]Raydium INT flag : %d\n", ts->irq_enabled);
            enable_irq(ts->irq);
            ts->irq_enabled = true;
            printk(KERN_INFO "[touch]Raydium irq enable\n");
            break;
        case 2: /* Disable INT */
            raydium_irq_control(ts, DISABLE);
            printk(KERN_INFO "[touch]Raydium disable INT\n");
            break;
        case 3: /* Enable INT */
            raydium_irq_control(ts, ENABLE);
            printk(KERN_INFO "[touch]Raydium enable INT\n");
            break;
        case 4:
            printk(KERN_INFO "[touch]Raydium INT depth : %d\n", ts->irq_desc->depth);
    }
    return 0;
}

/*****************************************************************************
** Function name:        bits_reverse
** Descriptions:        MSB/LSB exchange.
** parameters:            num:input data, bit_num:data bit number
** Returned value:        reverse value
*****************************************************************************/
static unsigned int bits_reverse(unsigned int num, unsigned int bit_num)
{
    unsigned int reverse = 0, i;

    for (i = 0; i < bit_num; i++)
    {
        if (num & (1 << i))
        {
            reverse |= 1<< ((bit_num - 1) -i);
        }
    }
    return reverse;
}
/*****************************************************************************
** Function name:        rc_crc32
** Descriptions:        32 bits checksum calculation.
** parameters:            buf:start address, len:length
** Returned value:        checksum
*****************************************************************************/
static unsigned int rc_crc32(const char *buf, unsigned int len, unsigned int crc)
{
    unsigned int i;
    unsigned char flash_byte, uc_current, j;

    for (i = 0; i < len; i++)
    {
        flash_byte = buf[i];
        uc_current = (unsigned char)bits_reverse(flash_byte, 8);
        for (j = 0; j <8; j++)
        {
            if ((crc ^ uc_current ) & 0x01)
            {
                crc = (crc >> 1) ^ 0xedb88320;
            } else
            {
                crc >>= 1;
            }
            uc_current >>= 1;
        }
    }
    return crc;
}

/* upgrade firmware with image file */
static int raydium_fw_upgrade_with_image(struct i2c_client *client, unsigned long ul_fw_addr, unsigned char type)
{
    int ret = -1;
    unsigned long ul_fw_size = 0;
    unsigned char *firmware_data;
    unsigned long ul_write_offset = 0;
    unsigned short us_write_length = 0;
    unsigned int ui_checksum = 0xFFFFFFFF;

    firmware_data = kzalloc(RAYDIUM_FW_MAX_SIZE, GFP_KERNEL);
    if (firmware_data  == NULL )
    {
        pr_err("kzalloc firmware_data failed\n");
        return -ENOMEM;
    }

    memset(firmware_data, 0, ul_fw_size);

    switch (type)
    {
        case RAYDIUM_INIT:
            memcpy(firmware_data, RadInitImage, 0x1FC);
            ul_fw_size = 0x1FC;
            break;
        case RAYDIUM_BASELINE:
            memcpy(firmware_data, (RadFWImage + 0xA04), 0x2CC);
            ul_fw_size = 0x2CC;
            break;
        case RAYDIUM_COMP:
            memcpy(firmware_data, (RadFWImage + 0xCD4), 0x244);
            ul_fw_size = 0x244;
            break;
        case RAYDIUM_PARA:
            memcpy(firmware_data, (RadFWImage + 0xF1C), 0xE0);
            ul_fw_size = 0xE0;
            break;
        case RAYDIUM_FIRMWARE:
            memcpy(firmware_data, (RadFWImage + 0x1000), 0x4FFC);
            ul_fw_size = 0x4FFC;
            break;
        case RAYDIUM_BOOTLOADER:
            memcpy(firmware_data, RadBootImage, 0x7FC);
            memcpy((firmware_data + 0x800), RadInitImage, 0x1FC);
            ul_fw_size = 0x9FC;
            break;
        case RAYDIUM_TEST_FW:
            memcpy((firmware_data + 4), (RadTestFWImage + 0xA04), 0x55FC);
            ul_fw_size = 0x55FC;
            break;
    }

    if (type == RAYDIUM_TEST_FW)
    {
        ui_checksum = rc_crc32((firmware_data + 4) , ul_fw_size, ui_checksum);
        printk(KERN_INFO "[touch]Raydium ui_checksum : 0x%08X\n", ui_checksum);
        ui_checksum = bits_reverse(ui_checksum, 32);
        memcpy(firmware_data, &ui_checksum, 4);
    } else
    {
        ui_checksum = rc_crc32(firmware_data, ul_fw_size, ui_checksum);
        printk(KERN_INFO "[touch]Raydium ui_checksum : 0x%08X\n", ui_checksum);
        ui_checksum = bits_reverse(ui_checksum, 32);
        memcpy((firmware_data + ul_fw_size), &ui_checksum, 4);
    }
    printk(KERN_INFO "[touch]Raydium ui_checksum after bits reverse: 0x%08X\n", ui_checksum);
    printk(KERN_INFO "[touch]Raydium (firmware_data + ul_fw_size): 0x%08X\n", *(unsigned int*)(firmware_data + ul_fw_size));
    ul_fw_size += 4;

    ul_write_offset = 0;
    while (ul_write_offset < ul_fw_size)
    {
        if ((ul_write_offset + MAX_WRITE_PACKET_SIZE) < ul_fw_size)
        {
            us_write_length = MAX_WRITE_PACKET_SIZE;
        } else
        {
            us_write_length = (unsigned short)(ul_fw_size - ul_write_offset);
        }

        ret = raydium_i2c_pda_write(client, (ul_fw_addr + ul_write_offset), (firmware_data + ul_write_offset), us_write_length);
        if (ret < 0)
        {
            goto exit_upgrate;
        }
        ul_write_offset += (unsigned long)us_write_length;
    }
    ul_fw_addr += ul_write_offset;

exit_upgrate:
    kfree(firmware_data);
    if (ret < 0)
    {
        pr_err("[touch]upgrade firmware Failed\n");
    return ret;
    } else
    {
        pr_err("[touch]upgrade firmware Successfully\n");
        return 0;
    }
}

static int raydium_do_software_reset(struct i2c_client * client)
{
    int ret = SUCCESS;
    unsigned int retry, retry_time = 1000;
    unsigned int irq_value;
    unsigned char buf[4];
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    //#enable INT
    g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;
    printk(KERN_INFO "[touch]ready to software reset => enable INT\n");
    raydium_irq_control(ts, ENABLE);

    //SW reset
    memset(buf, 0, sizeof(buf));
    buf[0]=0x01;
    printk(KERN_INFO "[touch]Raydium SW reset\n");
    ret = raydium_i2c_pda_write(client, 0x40000004, buf, 4);
    if (ret < 0)
    {
        goto exit;
    }
    retry = retry_time;
    while (retry != 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        msleep(2);
        retry--;
        printk(KERN_INFO "[touch]wait software reset finish => confirm interrupt_flag result %d\n", retry);
    }

    //#disable INT
    if (retry  == 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        printk(KERN_INFO "[touch]no interrupt => disable INT\n");
        raydium_irq_control(ts, DISABLE);
        g_uc_raydium_flag = 0;
    }

    if (g_uc_raydium_flag == RAYDIUM_INTERRUPT_FLAG)
    {
        retry = retry_time;
        irq_value = 0;
        while (retry != 0 && irq_value != 1)
        {
            irq_value = gpio_get_value(ts->pdata->irq_gpio);
            printk(KERN_INFO "[touch]irq_value is %d\n", irq_value);
            msleep(2);
            retry--;
        }
    }

    if (retry == 0)
    {
        printk(KERN_ERR "[touch]%s, FW not ready, retry error!\n", __func__);
        ret = ERROR;
    }

exit:
    return ret;
}

static int raydium_wait_ft_int(struct i2c_client * client)
{
    int ret = SUCCESS;
    unsigned int retry, retry_time = 1000;
    unsigned int irq_value;

    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    //#enable INT
    g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;

    retry = retry_time;
    while (retry != 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        msleep(2);
        retry--;
    }

    //#disable INT
    if (retry  == 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        g_uc_raydium_flag = 0;
    }

    if (g_uc_raydium_flag == RAYDIUM_INTERRUPT_FLAG)
    {
        retry = retry_time;
        irq_value = 0;
        while (retry != 0 && irq_value != 1)
        {
            irq_value = gpio_get_value(ts->pdata->irq_gpio);
            printk(KERN_INFO "[touch]irq_value is %d\n", irq_value);
            msleep(2);
            retry--;
        }
    }

    if (retry == 0)
    {
        printk(KERN_INFO "[touch]%s, FT Test not ready, retry error!\n", __func__);
        ret = ERROR;
    } else
    {
        raydium_irq_control(ts, ENABLE);
    }
    return ret;
}

static int raydium_check_fw_ready(struct i2c_client * client)
{
    int ret = SUCCESS;
    unsigned int retry, retry_time = 400;
    unsigned char buf[4];

    buf[1] = 0;
    retry = retry_time;
    while (buf[1] != 0x40 && retry != 0)
    {
        printk(KERN_INFO "[touch]confirm boot is finish 0x%x\n", buf[1]);
        ret = raydium_i2c_pda_read(client, 0x50000918, buf, 4);
        if (ret < 0)
        {
            goto exit;
        }
        retry --;
        msleep(5);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, confirm boot is finish 0x%x\n", buf[1]);
        ret = -1;
        goto exit;
    }

    if (retry == 0)
    {
        printk(KERN_ERR "[touch]%s, FW not ready, retry error!\n", __func__);
        ret = ERROR;
    } else
    {
        printk(KERN_INFO "[touch]%s, FW is ready!!\n", __func__);
        msleep(5);
    }

exit:
    return ret;
}

static int set_skip_load(struct i2c_client * client)
{
    int ret = SUCCESS;
    unsigned char buf[4];

    /*Return to Bootloader in PRAM*/
    //Skip load
    memset(buf, 0, sizeof(buf));
    buf[0]=0x10;
    buf[1]=0x08;
    printk(KERN_INFO "[touch]Raydium skip load\n");
    ret = raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_do_software_reset(client);

exit_upgrade:
    return ret;
}

static int raydium_load_test_fw(struct i2c_client * client)
{
    int ret = SUCCESS;
    unsigned char buf[4];
    unsigned int retry, retry_time = 200;
    unsigned char is_MCU_HOLD;
    unsigned int irq_value;
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    ret = raydium_i2c_pda_read(client, 0x50000918, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    is_MCU_HOLD = buf[1] & 0x20;
    printk(KERN_INFO "[touch]is_MCU_HOLD = %d\n", is_MCU_HOLD);

    //#enable INT
    g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;
    printk(KERN_INFO "[touch]ready to software reset => enable INT\n");
    raydium_irq_control(ts, ENABLE);

    //Skip load
    memset(buf, 0, sizeof(buf));
    buf[0]=0x10;
    buf[1]=0x08;
    printk(KERN_INFO "[touch]Raydium skip load\n");
    ret = raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    buf[0]=0x01;
    ret = raydium_i2c_pda_write(client, 0x40000004, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    if (is_MCU_HOLD == 0)
    {
        retry = retry_time;
        while (retry != 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
        {
            msleep(10);
            retry--;
            printk(KERN_INFO "[touch]wait software reset finish => confirm interrupt_flag result %d\n", retry);
        }

        //#disable INT
        if (retry  == 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
        {
            printk(KERN_INFO "[touch]no interrupt => disable INT\n");
            raydium_irq_control(ts, DISABLE);
            g_uc_raydium_flag = 0;
        }

        if (g_uc_raydium_flag == RAYDIUM_INTERRUPT_FLAG)
        {
            retry = retry_time;
            irq_value = 0;
            while (retry!=0 && irq_value!=1)
            {
                irq_value = gpio_get_value(ts->pdata->irq_gpio);
                printk(KERN_INFO "[touch]irq_value is %d\n", irq_value);
                msleep(10);
                retry--;
            }
        }

        if (retry == 0)
        {
            printk(KERN_ERR "[touch]%s, FW not ready, retry error!\n", __func__);
            ret = ERROR;
            goto exit_upgrade;
        }
    } else
    {
        msleep(100);
        raydium_irq_control(ts, DISABLE);
    }

    //set mcu hold
    raydium_i2c_pda_read(client, 0x50000918, buf, 4);
    buf[0] |= 0x20;
    raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    raydium_i2c_pda_read(client, 0x40000004, buf, 4);
    buf[0] |= 0x01;
    raydium_i2c_pda_write(client, 0x40000004, buf, 4);

    /*WRT boot-loader to PRAM first*/
    memset(buf, 0, sizeof(buf));
    printk("[touch]Raydium WRT boot-loader to PRAM first\n");
    ret = raydium_i2c_pda_write(client, 0x50000900, buf, 4);

    /*Sending bootloader*/
    ret = raydium_fw_upgrade_with_image(client, 0x0000, RAYDIUM_BOOTLOADER);
    if (ret < 0)
    {
        goto exit_upgrade;
    }

    //release mcu hold
    raydium_i2c_pda_read(client, 0x50000918, buf, 4);
    buf[0] &= ~0x20;
    buf[0] |= 0x10;
    raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    raydium_i2c_pda_read(client, 0x40000004, buf, 4);
    buf[0] |= 0x01;
    raydium_i2c_pda_write(client, 0x40000004, buf, 4);
    msleep(50);

    ret = raydium_i2c_pda_read(client, 0x40000000, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    buf[3] |= 0x40;
    ret = raydium_i2c_pda_write(client, 0x40000000, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_i2c_pda_read(client, 0x40000014, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    buf[0] |= 0x04;
    buf[1] |= 0x04;
    ret = raydium_i2c_pda_write(client, 0x40000014, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }

    memset(buf, 0, sizeof(buf));
    printk(KERN_INFO "[touch]Raydium WRT test_fw to PRAM\n");

    ret = raydium_i2c_pda_write(client, 0x50000900, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    /*Sending test fw*/
    ret = raydium_fw_upgrade_with_image(client, 0xA00, RAYDIUM_TEST_FW);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //Skip load
    memset(buf, 0, sizeof(buf));
    buf[0]=0x10;
    printk(KERN_INFO "[touch]Raydium skip load\n");
    ret = raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_do_software_reset(client);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_check_fw_ready(client);

exit_upgrade:
    return ret;
}

static int raydium_boot_upgrade(struct i2c_client * client)
{
    int ret = SUCCESS;
    unsigned char buf[4];
    unsigned int retry, retry_time = 200;
    unsigned char is_MCU_HOLD;
    unsigned int irq_value;
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    ret = raydium_i2c_pda_read(client, 0x50000918, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    is_MCU_HOLD = buf[1] & 0x20;
    printk(KERN_INFO "[touch]is_MCU_HOLD = %d\n", is_MCU_HOLD);

    //#enable INT
    g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;
    printk(KERN_INFO "[touch]ready to software reset => enable INT\n");
    raydium_irq_control(ts, ENABLE);

    /* Return to Bootloader */
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x00;
    buf[1] = 0x08;
    raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    memset(buf, 0, sizeof(buf));
    buf[0] |= 0x01;
    raydium_i2c_pda_write(client, 0x40000004, buf, 4);

    if (is_MCU_HOLD == 0)
    {
        retry = retry_time;
        while (retry != 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
        {
            msleep(10);
            retry--;
            printk(KERN_INFO "[touch]wait software reset finish => confirm interrupt_flag result %d\n", retry);
        }

        //#disable INT
        if (retry  == 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
        {
            printk(KERN_INFO "[touch]no interrupt => disable INT\n");
            raydium_irq_control(ts, DISABLE);
            g_uc_raydium_flag = 0;
        }

        if (g_uc_raydium_flag == RAYDIUM_INTERRUPT_FLAG)
        {
            retry = retry_time;
            irq_value = 0;
            while (retry!=0 && irq_value!=1)
            {
                irq_value = gpio_get_value(ts->pdata->irq_gpio);
                printk(KERN_INFO "[touch]irq_value is %d\n", irq_value);
                msleep(10);
                retry--;
            }
        }

        if (retry == 0)
        {
            printk(KERN_ERR "[touch]%s, FW not ready, retry error!\n", __func__);
            ret = ERROR;
            goto exit_upgrade;
        }
    } else
    {
        raydium_irq_control(ts, DISABLE);
        msleep(100);
    }

    //set mcu hold
    raydium_i2c_pda_read(client, 0x50000918, buf, 4);
    buf[0] |= 0x20;
    raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    raydium_i2c_pda_read(client, 0x40000004, buf, 4);
    buf[0] |= 0x01;
    raydium_i2c_pda_write(client, 0x40000004, buf, 4);

    /*WRT boot-loader to PRAM first*/
    memset(buf, 0, sizeof(buf));
    printk(KERN_INFO "[touch]WRT boot-loader to PRAM first\n");
    ret = raydium_i2c_pda_write(client, 0x50000900, buf, 4);

    /*Sending bootloader*/
    ret = raydium_fw_upgrade_with_image(client, 0x0000, RAYDIUM_BOOTLOADER);
    if (ret < 0)
    {
        goto exit_upgrade;
    }

    //release mcu hold
    raydium_i2c_pda_read(client, 0x50000918, buf, 4);
    buf[0] &= ~0x20;
    buf[0] |= 0x10;
    raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    raydium_i2c_pda_read(client, 0x40000004, buf, 4);
    buf[0] |= 0x01;
    raydium_i2c_pda_write(client, 0x40000004, buf, 4);
    msleep(50);

    ret = raydium_i2c_pda_read(client, 0x40000000, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    buf[3] |= 0x40;
    ret = raydium_i2c_pda_write(client, 0x40000000, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_i2c_pda_read(client, 0x40000014, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    buf[0] |= 0x04;
    buf[1] |= 0x04;
    ret = raydium_i2c_pda_write(client, 0x40000014, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //Skip load
    memset(buf, 0, sizeof(buf));
    buf[0]=0x10;
    buf[1]=0x08;
    printk("Raydium skip load\n");
    ret = raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_do_software_reset(client);

exit_upgrade:
    return ret;
}

/* Raydium fireware upgrade flow */
static int raydium_fw_upgrade(struct i2c_client * client, unsigned char type, unsigned char isSkipLoad, unsigned char isCheckCRCError)
{
    int ret = 0;
    unsigned char buf[4];
    unsigned int retry, retry_time = 200;
    unsigned int irq_value;
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    //##### wait for boot-loader start #####
    printk(KERN_INFO "[touch]type is %d\n", type);
    //#set main state as burning mode, normal init state
    //#sync_data:200h, main_state:204h, normal_state:208h, burning_state:20Ch
    //#sync_data:210h, cmd_type:210h; ret_data:214h; test_mode:218h
    memset(buf, 0, sizeof(buf));
    buf[0]=0x01;
    ret = raydium_i2c_pda_write(client, 0x20000204, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x20000208, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x2000020C, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    buf[0]=0x01;
    ret = raydium_i2c_pda_write(client, 0x20000218, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#confirm in burn mode
    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 63 && retry != 0)
    {
        printk(KERN_INFO "[touch]confirm in burn mode 0x%x\n", buf[0]);
        ret = raydium_i2c_pda_read(client, 0x50000900, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        msleep(10);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, confirm in burn mode 0x%x\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    printk(KERN_INFO "[touch]VVVVVVVVVVVVVVV Type : %d ==> Start VVVVVVVVVVVVVVV\n", type);

    //Clear BL_CRC
    memset(buf, 0, sizeof(buf));
    buf[0]=0x10;
    ret = raydium_i2c_pda_write(client, 0x50000918, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }

    //#write PRAM relative data
    //#set PRAM type (at 0x50000904), wrt param code
    //#init_code:0x01, baseline:0x02, COMP:0x04, param:0x08, FW:0x10, bootloader:0x20
    memset(buf, 0, sizeof(buf));
    buf[0]=type;
    ret = raydium_i2c_pda_write(client, 0x50000904, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#set PRAM addr (at 'h5000_0908)
    //#init_code:0x800, Baseline:0xA00, COMP:0xCD4, para:0xF1C, FW:0x1000, BOOT:0x5000
    //#set PRAM length (at 'h5000_090C)
    if (type == RAYDIUM_INIT)
    {
        printk(KERN_INFO "[touch]Set PRAM addr & length type=0x01\n");
        memset(buf, 0, sizeof(buf));
        buf[0]=0x00;
        buf[1]=0x08;
        ret = raydium_i2c_pda_write(client, 0x50000908, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        memset(buf, 0, sizeof(buf));
        buf[0]=0x00;
        buf[1]=0x02;
        ret = raydium_i2c_pda_write(client, 0x5000090C, buf, 4);
        if (ret < 0)
        {
                goto exit_upgrade;
        }
    } else if (type == RAYDIUM_BASELINE)
    {
        printk(KERN_INFO "[touch]Set PRAM addr & length type=0x02\n");
        memset(buf, 0, sizeof(buf));
        buf[0]=0x04;
        buf[1]=0x0a;
        ret = raydium_i2c_pda_write(client, 0x50000908, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        memset(buf, 0, sizeof(buf));
        buf[0]=0xd0;
        buf[1]=0x02;
        ret = raydium_i2c_pda_write(client, 0x5000090C, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
    } else if (type == RAYDIUM_COMP)
    {
        printk(KERN_INFO "[touch]Set PRAM addr & length type=0x04\n");
        memset(buf, 0, sizeof(buf));
        buf[0]=0xd4;
        buf[1]=0x0c;
        ret = raydium_i2c_pda_write(client, 0x50000908, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        memset(buf, 0, sizeof(buf));
        buf[0]=0x48;
        buf[1]=0x02;
        ret = raydium_i2c_pda_write(client, 0x5000090C, buf, 4);
        if (ret < 0)
        {
                goto exit_upgrade;
        }
    } else if (type == RAYDIUM_PARA)
    {
        printk(KERN_INFO "[touch]Set PRAM addr & length type=0x08\n");
        memset(buf, 0, sizeof(buf));
        buf[0]=0x1c;
        buf[1]=0x0f;
        ret = raydium_i2c_pda_write(client, 0x50000908, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        memset(buf, 0, sizeof(buf));
        buf[0]=0xe4;
        ret = raydium_i2c_pda_write(client, 0x5000090C, buf, 4);
        if (ret < 0)
        {
                goto exit_upgrade;
        }
    } else if (type == RAYDIUM_FIRMWARE)
    {
        printk(KERN_INFO "[touch]Set PRAM addr & length type=0x10\n");
        memset(buf, 0, sizeof(buf));
        buf[0]=0x1C;
        buf[1]=0x0F;
        ret = raydium_i2c_pda_write(client, 0x50000908, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        memset(buf, 0, sizeof(buf));
        buf[0]=0xE4;
        buf[1]=0x50;
        ret = raydium_i2c_pda_write(client, 0x5000090C, buf, 4);
        if (ret < 0)
        {
                goto exit_upgrade;
        }
    } else if (type == RAYDIUM_BOOTLOADER)
    {
        printk(KERN_INFO "[touch]Set PRAM addr & length type=0x20\n");
        memset(buf, 0, sizeof(buf));
        buf[0]=0x00;
    buf[1]=0x40;
        ret = raydium_i2c_pda_write(client, 0x50000908, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        memset(buf, 0, sizeof(buf));
        buf[0]=0x00;
        buf[1]=0x0A;
        ret = raydium_i2c_pda_write(client, 0x5000090C, buf, 4);
        if (ret < 0)
        {
                goto exit_upgrade;
        }
    }

    //#set sync_data(0x20000200) = 0 as WRT data finish
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x20000200, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#Wait bootloader check addr and PRAM unlock
    //#Confirm g_u8_sync_data.ret_data at 0x20000214 is SET_ADDR_READY
    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 161 && retry != 0)
    {
        msleep(10);
        ret = raydium_i2c_pda_read(client, 0x20000214, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        printk(KERN_INFO "[touch]Confirm g_u8_sync_data.ret_data is SET_ADDR_READY %d\n", buf[0]);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, Confirm g_u8_sync_data.ret_data is SET_ADDR_READY %d\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    //#Confirm g_u8_sync_data.cmd_type at 0x20000210 is WRT_PRAM_DATA
    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 163 && retry != 0)
    {
        msleep(10);
        ret = raydium_i2c_pda_read(client, 0x20000210, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        printk(KERN_INFO "[touch]Confirm g_u8_sync_data.cmd_type is WRT_PRAM_DATA %d\n", buf[0]);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, Confirm g_u8_sync_data.cmd_type is WRT_PRAM_DATA %d\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    //#start write data to PRAM
    if (type == RAYDIUM_INIT)
    {
        ret = raydium_fw_upgrade_with_image(client, 0x800, RAYDIUM_INIT);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
    }  else if (type == RAYDIUM_PARA)
    {
        ret = raydium_fw_upgrade_with_image(client, 0xF1C, RAYDIUM_PARA);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
    } else if (type == RAYDIUM_FIRMWARE)
    {
        ret = raydium_fw_upgrade_with_image(client, 0xF1C, RAYDIUM_PARA);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        ret = raydium_fw_upgrade_with_image(client, 0x1000, RAYDIUM_FIRMWARE);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
    } else if (type == RAYDIUM_BOOTLOADER)
    {
        ret = raydium_fw_upgrade_with_image(client, 0x4000, RAYDIUM_BOOTLOADER);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
    }

    //#set sync_data(0x20000200) = 0 as WRT data finish, bootloader check checksum
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x20000200, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#wait(checksum okay) ACK cmd(gu8I2CSyncData.cmd_type=0xa5 at 0x20000210)
    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 165 && retry != 0)
    {
        msleep(10);
        ret = raydium_i2c_pda_read(client, 0x20000210, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        printk(KERN_INFO "[touch]wait(checksum okay) ACK cmd 0x%x\n", buf[0]);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, wait(checksum okay) ACK cmd 0x%x\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    //#confirm ACK cmd result(gu8I2CSyncData.ret_data=0xa5 at 0x20000214)
    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 165 && retry != 0)
    {
        msleep(10);
        ret = raydium_i2c_pda_read(client, 0x20000214, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        printk(KERN_INFO "[touch]confirm ACK cmd result 0x%x\n", buf[0]);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, confirm ACK cmd result 0x%x\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    //#set ACK return data = 0x5A
    //#adb shell "echo 20000210 1 A5 > /sys/bus/i2c/drivers/raydium_ts/1-0039/raydium_i2c_pda_access"
    //#above command can be ignored, due to previous while loop has check its value.
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x5a;
    ret = raydium_i2c_pda_write(client, 0x20000214, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#clr sync_data(0x20000200) = 0 as finish
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x20000200, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#wait for input unlock key
    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 168 && retry != 0)
    {
        msleep(10);
        ret = raydium_i2c_pda_read(client, 0x20000210, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        printk(KERN_INFO "[touch]wait for input unlock key 0x%x\n", buf[0]);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, wait for input unlock key 0x%x\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    //#unlock key
    memset(buf, 0, sizeof(buf));
    buf[0] = 0xd7;
    ret = raydium_i2c_pda_write(client, 0x50000938, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    buf[0] = 0xa5;
    ret = raydium_i2c_pda_write(client, 0x50000934, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x50000934, buf, 4);
    if (ret < 0)
    {
            goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    buf[0] = 0xa5;
    ret = raydium_i2c_pda_write(client, 0x50000934, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x50000938, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#wrt return data as unlock value
    memset(buf, 0, sizeof(buf));
    buf[0] = 0xa8;
    ret = raydium_i2c_pda_write(client, 0x20000214, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    //#clr sync_data(0x20000200) = 0 as finish
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x20000200, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    msleep(500);

    //#wait erase/wrt finish
    //#confirm burning_state result(gu8I2CSyncData.burning_state=BURNING_WRT_FLASH_FINISH at 0x2000020C)
    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 6 && retry != 0)
    {
        msleep(10);
        ret = raydium_i2c_pda_read(client, 0x2000020c, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        printk(KERN_INFO "[touch]confirm burning_state result %d\n", buf[0]);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, confirm burning_state result %d\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    //############## Use Interrupt to wait sofware reset ##############
    //#clr sync_data(0x20000200) = 0 as finish
    //#Set bootloader flag
    //#RAYDIUM_INTERRUPT_FLAG = 0x01
    //#RAYDIUM_BOOTLOADER_FLAG = 0x02
    g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;
    //#enable INT
    printk(KERN_INFO "[touch]ready to software reset => enable INT\n");
    raydium_irq_control(ts, ENABLE);

    if (type == RAYDIUM_BOOTLOADER)
    {
        ret = raydium_i2c_pda_read(client, 0x50000918, buf, 4);
    if (ret < 0)
        {
            goto exit_upgrade;
        }
        buf[0] |= 0x10;
        ret = raydium_i2c_pda_write(client, 0x50000918, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
    }

    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(client, 0x20000200, buf, 4);
    if (ret < 0)
    {
        goto exit_upgrade;
    }

    //#wait software reset finish
    //#confirm RAYDIUM_INTERRUPT_FLAG result (if raydium_flag = 1 => pass)
    retry = retry_time;
    while (retry != 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        msleep(10);
        retry--;
        printk(KERN_INFO "[touch]wait software reset finish => confirm interrupt_flag result %d\n", g_uc_raydium_flag);
    }

    //#disable INT
    if (retry == 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
            printk(KERN_INFO "[touch]no interrupt => disable INT\n");
            raydium_irq_control(ts, DISABLE);
            g_uc_raydium_flag = 0;
    }

    if (g_uc_raydium_flag == RAYDIUM_INTERRUPT_FLAG)
    {
        retry = retry_time;
        irq_value = 0;
        while (retry != 0 && irq_value != 1)
        {
            irq_value = gpio_get_value(ts->pdata->irq_gpio);
            printk(KERN_INFO "[touch]irq_value is %d\n", irq_value);
            msleep(10);
            retry--;
        }
    }

    if (retry == 0)
    {
        printk(KERN_ERR "[touch]%s, FW not ready, retry error!\n", __func__);
        ret = ERROR;
        goto exit_upgrade;
    }
    //#################################################################

    buf[0] = 0;
    retry = retry_time;
    while (buf[0] != 130 && retry != 0)
    {
        msleep(10);
        ret = raydium_i2c_pda_read(client, 0x20000214, buf, 1);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        retry --;
        printk(KERN_INFO "[touch]wait software reset finished %d\n", buf[0]);
    }
    if (retry == 0)
    {
        printk(KERN_ERR "[touch]Error, wait software reset finished %d\n", buf[0]);
        ret = -1;
        goto exit_upgrade;
    }

    if (type == RAYDIUM_BASELINE || type == RAYDIUM_INIT || isCheckCRCError == 1)
    {
        //#set test_mode = 1
        memset(buf, 0, sizeof(buf));
        buf[0] = 0x01;
        ret = raydium_i2c_pda_write(client, 0x20000218, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        //#wait crc check finish
        //#confirm gu8I2CSyncData.normal_state=NORMAL_CRC_NOTIFY at 0x20000208
        buf[0] = 0;
        retry = retry_time;
        while (buf[0] != 2 && retry != 0)
        {
            msleep(10);
            ret = raydium_i2c_pda_read(client, 0x20000208, buf, 1);
            if (ret < 0)
            {
                goto exit_upgrade;
            }
            retry --;
            printk(KERN_INFO "[touch]confirm gu8I2CSyncData.normal_state=NORMAL_CRC_NOTIFY %d\n", buf[0]);
        }
        if (retry == 0)
        {
            printk(KERN_ERR "[touch]Error, confirm gu8I2CSyncData.normal_state=NORMAL_CRC_NOTIFY %d\n", buf[0]);
            ret = -1;
            goto exit_upgrade;
        }

        //#crc check pass 0x20000214 = 0x81
        buf[0] = 0;
        retry = retry_time;
        while (buf[0] != 0x81 && retry != 0)
        {
            msleep(10);
            ret = raydium_i2c_pda_read(client, 0x20000214, buf, 1);
            if (ret < 0)
            {
                goto exit_upgrade;
            }
            retry --;
            printk(KERN_INFO "[touch]confirm crc result 0x20000214 = %d\n", buf[0]);
        }
        if (retry == 0)
        {
            printk(KERN_ERR "[touch]Error, confirm crc result 0x20000214 = %d\n", buf[0]);
            ret = -1;
            goto exit_upgrade;
        }
    }

    //#run to next step
    printk(KERN_INFO "[touch]^^^^^^^^^^^^^^^ Type: 0x%x ==> Pass ^^^^^^^^^^^^^^^\n", type);

    if (isCheckCRCError)
    {
        buf[0] = 0;
        retry = retry_time;
        while (buf[0] != 2 && retry != 0)
        {
            ret = raydium_i2c_pda_read(client, 0x50000918, buf, 1);
            if (ret < 0)
            {
                goto exit_upgrade;
            }
            msleep(10);
            buf[0] = buf[0] & 47;
            retry --;
            printk(KERN_INFO "[touch]BL1_CRC_ERR %d\n", buf[0]);
        }
        if (retry == 0)
        {
            printk(KERN_ERR "[touch]Error, BL1_CRC_ERR %d\n", buf[0]);
            ret = -1;
            goto exit_upgrade;
        }

        //#clr sync para
        memset(buf, 0, sizeof(buf));
        ret = raydium_i2c_pda_write(client, 0x20000210, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        ret = raydium_i2c_pda_write(client, 0x20000214, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }
        ret = raydium_i2c_pda_write(client, 0x20000218, buf, 4);
        if (ret < 0)
        {
            goto exit_upgrade;
        }

        printk(KERN_INFO "[touch]Raydium HW reset\n");
        gpio_direction_output(ts->rst, 1);
        gpio_direction_output(ts->rst, 0);
        msleep(RAYDIUM_RESET_INTERVAL_MSEC);
        gpio_direction_output(ts->rst, 1);
        msleep(500);

        u8_i2c_mode = PDA2_MODE;

        printk(KERN_INFO "[touch]Burn FW finish!\n");
    }

exit_upgrade:
    return ret;
}


static int raydium_burn_fw(struct i2c_client *client)
{
    int ret = 0;

    ret = raydium_boot_upgrade(client);

    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_fw_upgrade(client, RAYDIUM_BOOTLOADER, 0, 0);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
    ret = raydium_fw_upgrade(client, RAYDIUM_FIRMWARE, 0, 1);
    if (ret < 0)
    {
        goto exit_upgrade;
    }
exit_upgrade:
    return ret;
}

static ssize_t raydium_fw_upgrade_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }
    ret = kstrtou8(buf, 16, &upgrade_type);
    if (ret < 0)
    {
        return ret;
    }
    return count;
}

static ssize_t raydium_fw_upgrade_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int ret = 0, result = 0;
    int buf_len = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    raydium_irq_control(ts, DISABLE);
    printk(KERN_INFO "[touch]Raydium %s burn type is %d!!\n", __func__, upgrade_type);

    if (upgrade_type == 1)
    {
        ret = raydium_burn_fw(client);
        if (ret < 0)
        {
            result = FAIL;
            goto exit_upgrade;
        }
        result = SUCCESS;
    } else if (upgrade_type == 2)
    {
        ret = set_skip_load(client);
        if (ret < 0)
        {
            result = FAIL;
            goto exit_upgrade;
        }
        ret = raydium_fw_upgrade(client, RAYDIUM_COMP, 0, 1);
        if (ret < 0)
        {
            result = FAIL;
            goto exit_upgrade;
        }
        result = SUCCESS;
    } else if (upgrade_type == 3)
    {
        ret = set_skip_load(client);
        if (ret < 0)
        {
            result = FAIL;
            goto exit_upgrade;
        }
        ret = raydium_fw_upgrade(client, RAYDIUM_BASELINE, 0, 1);
        if (ret < 0)
        {
            result = FAIL;
            goto exit_upgrade;
        }
        result = SUCCESS;
    } else if (upgrade_type == 4)
    {
        ret = raydium_load_test_fw(client);
        if (ret < 0)
        {
            result = FAIL;
            goto exit_upgrade;
        }
        result = SUCCESS;
    }

exit_upgrade:
    raydium_irq_control(ts, ENABLE);
    upgrade_type = 0;
    sprintf(buf, "FW Upgrade result : %d\n", result);
    buf_len = strlen(buf);
    return buf_len+1;
}


/* upgrade firmware with *.bin file */
static int raydium_fw_upgrade_with_bin_file(struct i2c_client *client, char *arguments, size_t count)
{
    int ret = 0;
    char *temp_buf, *token, *free_temp_buf, *free_token;
    const char *delim = " ,";
    unsigned long ul_fw_offset = 0;
    unsigned char uc_fw_amount = 0;
    mm_segment_t old_fs;
    struct file *fw_fd = NULL;
    unsigned long ul_fw_size = 0, ul_fw_read_size = 0;
    char *firmware_data;
    unsigned long ul_write_offset = 0;
    unsigned short us_write_length = 0;
    unsigned int ui_checksum = 0xFFFFFFFF;

    /* receive command line arguments string */
    if (count < 2)
    {
        return -EINVAL;
    }
    firmware_data = kzalloc(RAYDIUM_FW_MAX_SIZE, GFP_KERNEL);
    if (firmware_data == NULL)
    {
        pr_err("[touch]kzalloc firmware_data failed\n");
        return -ENOMEM;
    }

    temp_buf = kzalloc(RAYDIUM_FW_BIN_PATH_LENGTH+1, GFP_KERNEL);
    if (temp_buf  == NULL)
    {
        pr_err("[touch]kzalloc temp_buf failed\n");
        kfree(firmware_data);
        return -ENOMEM;
    }

    token = kzalloc(RAYDIUM_FW_BIN_PATH_LENGTH+1, GFP_KERNEL);
    if (token == NULL )
    {
        pr_err("[touch]kzalloc token failed\n");
        kfree(firmware_data);
        kfree(temp_buf);
        return -ENOMEM;
    }

    free_temp_buf = temp_buf;
    free_token = token;
    strncpy(temp_buf, arguments, count);
    token = strsep(&temp_buf, delim);

    ret = kstrtoul(token, 16, &ul_fw_offset);
    if (ret < 0)
    {
        goto exit_upgrate;
    }

    token = strsep(&temp_buf, delim);
    ret = kstrtou8(token, 16, &uc_fw_amount);
    if (ret < 0)
    {
        goto exit_upgrate;
    }

    while (uc_fw_amount--)
    {
        if (!temp_buf)
        {
            break;
        }

        token = strsep(&temp_buf, delim);
        ret = kstrtoul(token, 16, &ul_fw_size);
        if (ret < 0)
        {
            goto exit_upgrate;
        }

        token = strsep(&temp_buf, delim);

        old_fs = get_fs();
        set_fs(get_ds());

        fw_fd = filp_open(token, O_RDONLY, 0);
        if (IS_ERR(fw_fd))
        {
            pr_err("[touch]error occured while opening file %s.\n", token);
            set_fs(old_fs);
            kfree(firmware_data);
            kfree(free_token);
            kfree(free_temp_buf);
            return -ENOENT;
        }

        memset(firmware_data, 0, ul_fw_size);
        ul_fw_read_size = fw_fd->f_op->read(fw_fd, firmware_data, ul_fw_size, &fw_fd->f_pos);
        ui_checksum = rc_crc32(firmware_data, ul_fw_size, ui_checksum);

        if (uc_fw_amount == 0)
        {
            ui_checksum = bits_reverse(ui_checksum, 32);
            memcpy((firmware_data + ul_fw_size), &ui_checksum, 4);
            ul_fw_size += 4;
        }

        set_fs(old_fs);
        filp_close(fw_fd,NULL);

        ul_write_offset = 0;
        while (ul_write_offset < ul_fw_size)
        {
            if ((ul_write_offset + MAX_WRITE_PACKET_SIZE) < ul_fw_size)
            {
                us_write_length = MAX_WRITE_PACKET_SIZE;
            }
            else
            {
                us_write_length = (unsigned short)(ul_fw_size - ul_write_offset);
            }
            ret = raydium_i2c_pda_write(client, (ul_fw_offset + ul_write_offset), (firmware_data + ul_write_offset), us_write_length);
            if (ret < 0)
            {
                goto exit_upgrate;
            }
            ul_write_offset += (unsigned long)us_write_length;
        }
        ul_fw_offset += ul_write_offset;
    }

exit_upgrate:
    kfree(firmware_data);
    kfree(free_token);
    kfree(free_temp_buf);
    if (ret < 0)
    {
        pr_err("[touch]Upgrage firmware Failed\n");
        return ret;
    } else {
        pr_err("[touch]Upgrage firmware Successfully\n");
        return 0;
    }
}

static ssize_t raydium_upgrade_firmware_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;
    char fwname[RAYDIUM_FW_BIN_PATH_LENGTH];
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (count > RAYDIUM_FW_BIN_PATH_LENGTH)
    {
        return -EINVAL;
    }
    memset(fwname, 0, sizeof(fwname));
    sprintf(fwname, "%s", buf);
    fwname[count - 1] = '\0';

    if (ts->is_suspend)
    {
        printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
    }

    /* start to upgrade binary file*/
    mutex_lock(&ts->lock);
    ret = raydium_fw_upgrade_with_bin_file(client, fwname, count);
    mutex_unlock(&ts->lock);
    if (ret < 0)
    {
        return ret;
    }
    return count;
}

static ssize_t raydium_i2c_pda2_mode_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;
    unsigned char mode;

    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (ts->is_suspend)
    {
        printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
    }

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }

    ret = kstrtou8(buf, 16, &mode);
    if (ret < 0)
    {
        return ret;
    }
    ret = raydium_i2c_mode_control(client, mode);
    if (ret < 0)
    {
        return ret;
    }

    return count;
}

static ssize_t raydium_i2c_pda_access_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    unsigned char rbuffer[4];
    int buf_len = 0;
    int ret = -1;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (g_ui_length > 4)
    {
        return -EINVAL;
    }
    memset(rbuffer, 0x00, 4);
    mutex_lock(&ts->lock);
    ret = raydium_i2c_pda_read(client, g_ul_addr, rbuffer, g_ui_length);
    mutex_unlock(&ts->lock);
    if (ret < 0)
    {
        return ret;
    }

    sprintf(buf, "0x%08lX : 0x%02X%02X%02X%02X\n", g_ul_addr,
                rbuffer[3], rbuffer[2], rbuffer[1], rbuffer[0]);
    buf_len = strlen(buf);

    return buf_len+1;
}

static ssize_t raydium_i2c_pda_access_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0, result = 0;
    char *temp_buf, *token, *free_temp_buf, *free_token;
    const char *delim = " ,";
    unsigned char w_data[MAX_WRITE_PACKET_SIZE];
    unsigned int data_count = 0;
    unsigned int data_index = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    /* receive command line arguments string */
    if (count < 2)
    {
        return -EINVAL;
    }

    temp_buf = kzalloc(count+1, GFP_KERNEL);
    if (temp_buf == NULL)
    {
        printk(KERN_INFO "[touch]kzalloc temp_buf failed\n");
        return -ENOMEM;
    }

    token = kzalloc(count+1, GFP_KERNEL);
    if (token == NULL)
    {
        printk(KERN_INFO "[touch]kzalloc token failed\n");
        kfree(temp_buf);
        return -ENOMEM;
    }

    free_temp_buf = temp_buf;
    free_token = token;

    ret = (ssize_t) count;

    strncpy(temp_buf, buf, count);

    token = strsep(&temp_buf, delim);

    result = kstrtoul(token, 16, &g_ul_addr);

    token = strsep(&temp_buf, delim);
    if (token)
    {
        result = kstrtouint(token, 16, &data_count);
    } else
    {
        goto exit_error;
    }
    if (g_ui_length > MAX_WRITE_PACKET_SIZE)
    {
        return -EINVAL;
    }
    g_ui_length = data_count;

    memset(w_data, 0x00, MAX_WRITE_PACKET_SIZE);

    if (temp_buf && data_count)
    {
        data_index = 0;
        while (data_count)
        {
            token = strsep(&temp_buf, delim);
            result = kstrtou8(token, 16, &w_data[data_index++]);
            if (result < 0)
            {
                ret = result;
                goto exit_error;
            }
            data_count--;
        }
        mutex_lock(&ts->lock);
        result = raydium_i2c_pda_write(client, g_ul_addr, w_data, g_ui_length);
        mutex_unlock(&ts->lock);
        if (result < 0)
        {
            ret = result;
        }
    }

exit_error:
    kfree(free_token);
    kfree(free_temp_buf);
    return ret;
}

unsigned char g_uc_addr = RAYDIUM_PDA2_PDA_CFG_ADDR;

static ssize_t raydium_i2c_pda2_access_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    unsigned char rbuffer[4];
    int buf_len = 0;
    int ret = -1;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts  = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (g_ui_length > 4)
    {
        return -EINVAL;
    }
    memset(rbuffer, 0x00, 4);

    mutex_lock(&ts->lock);
    ret = raydium_i2c_pda2_read(client, g_uc_addr, rbuffer, g_ui_length);
    mutex_unlock(&ts->lock);
    if (ret < 0)
    {
        return ret;
    }

    sprintf(buf, "0x%04X : 0x%02X%02X%02X%02X\n", g_uc_addr,
                rbuffer[3], rbuffer[2], rbuffer[1], rbuffer[0]);
    buf_len = strlen(buf);

    return buf_len+1;
}

static ssize_t raydium_i2c_pda2_access_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0, result = 0;
    char *temp_buf, *token, *free_temp_buf, *free_token;
    const char *delim = " ,";
    unsigned char w_data[MAX_WRITE_PACKET_SIZE];
    unsigned int data_count = 0;
    unsigned int data_index = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    /* receive command line arguments string */
    if (count < 2)
    {
        return -EINVAL;
    }
    temp_buf = kzalloc(count+1, GFP_KERNEL);
    if (temp_buf  == NULL )
    {
        printk(KERN_ERR "[touch]kzalloc temp_buf failed\n");
        return -ENOMEM;
    }

    token = kzalloc(count+1, GFP_KERNEL);
    if (token  == NULL )
    {
        printk(KERN_ERR "[touch]kzalloc token failed\n");
        kfree(temp_buf);
        return -ENOMEM;
    }

    free_temp_buf = temp_buf;
    free_token = token;

    ret = (ssize_t) count;

    strncpy(temp_buf, buf, count);

    token = strsep(&temp_buf, delim);

    result = kstrtou8(token, 16, &g_uc_addr);

    token = strsep(&temp_buf, delim);
    if (token)
    {
        result = kstrtouint(token, 16, &data_count);
    } else
    {
        ret = -EINVAL;
        goto exit_error;
    }

    if (data_count > MAX_WRITE_PACKET_SIZE)
    {
        ret = -EINVAL;
        goto exit_error;
    }

    memset(w_data, 0x00, MAX_WRITE_PACKET_SIZE);

    g_ui_length = data_count;

    if (temp_buf && data_count)
    {
        data_index = 0;
        while (data_count)
        {
            token = strsep(&temp_buf, delim);
            result = kstrtou8(token, 16, &w_data[data_index++]);
            if (result < 0)
            {
                ret = result;
                goto exit_error;
            }
            data_count--;
        }

        mutex_lock(&ts->lock);
        result = raydium_i2c_pda2_write(client, g_uc_addr, w_data, g_ui_length);
        mutex_unlock(&ts->lock);
        if (result < 0)
        {
            ret = result;
            goto exit_error;
        }
    }

exit_error:
    kfree(free_token);
    kfree(free_temp_buf);
    return ret;
}

static ssize_t raydium_i2c_pda2_page_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0, result = 0;
    unsigned char page = 0;
    char *temp_buf, *token, *free_temp_buf, *free_token;
    const char *delim = " ,";
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    /* receive command line arguments string */
    if (count < 2)
    {
        return -EINVAL;
    }
    temp_buf = kzalloc(count+1, GFP_KERNEL);
    if (temp_buf == NULL)
    {
        printk(KERN_ERR "[touch]kzalloc temp_buf failed\n");
        return -ENOMEM;
    }

    token = kzalloc(count+1, GFP_KERNEL);
    if (token == NULL)
    {
        printk(KERN_ERR "[touch]kzalloc token failed\n");
        kfree(temp_buf);
        return -ENOMEM;
    }

    free_temp_buf = temp_buf;
    free_token = token;

    ret = (ssize_t) count;

    strncpy(temp_buf, buf, count);

    token = strsep(&temp_buf, delim);

    if (temp_buf)
    {
        printk(KERN_ERR "[touch]Raydium input error, extra auguments!n");
        ret = -EINVAL;
        goto exit_error;
    }
    result = kstrtou8(token, 16, &page);

    if (result < 0)
    {
        ret = result;
        goto exit_error;
    }

    raydium_irq_control(ts, DISABLE);
    mutex_lock(&ts->lock);

    result = raydium_i2c_pda2_set_page(client, page);
    if (result < 0)
    {
        ret = result;
        goto exit_set_error;
    }
// TODO: Page check, Due to ISR will change page back to Page_0. Or disable IRQ during PDA2 access period

exit_set_error:
    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);

exit_error:
    kfree(free_token);
    kfree(free_temp_buf);
    return ret;
}

static ssize_t raydium_i2c_raw_data_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    unsigned char rbuffer[MAX_READ_PACKET_SIZE];
    unsigned int ui_read_target_addr;
    unsigned int ui_read_offset;
    unsigned short us_read_length;
    int ret = -1;
    int retry = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    memset(rbuffer, 0x00, MAX_READ_PACKET_SIZE);

    // make sure update flag was set
    for (retry = 0 ; retry < SYN_I2C_RETRY_TIMES ; retry++)
    {
        mutex_lock(&ts->lock);
        ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
        ret = raydium_i2c_pda2_read(client, RAYDIUM_PDA2_HOST_CMD_ADDR, rbuffer, RAYDIUM_FT_CMD_LENGTH);
        mutex_unlock(&ts->lock);
        if (ret < 0)
        {
            goto exit_flag_error;
        }
        if ((rbuffer[RAYDIUM_FT_CMD_POS] & RAYDIUM_FT_UPDATE) == RAYDIUM_FT_UPDATE)
        {
            break;
        }
        msleep(5);
    }

    if (retry == SYN_I2C_RETRY_TIMES)
    {
        ret = -EAGAIN;
        goto exit_flag_error;
    }

    ui_read_offset = 0;
    us_read_length = 0;
    while (ui_read_offset < g_ui_raw_data_length)
    {
        if ((ui_read_offset + MAX_READ_PACKET_SIZE) < g_ui_raw_data_length)
        {
            us_read_length = MAX_READ_PACKET_SIZE;
        }
        else
        {
            us_read_length = (unsigned short)(g_ui_raw_data_length - ui_read_offset);
        }
        ui_read_target_addr = RAYDIUM_READ_FT_DATA_CMD + ui_read_offset;

        mutex_lock(&(ts->lock));
        ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
        if (ret < 0)
            goto exit_i2c_error;

        *(unsigned long *)rbuffer = (RAYDIUM_I2C_PDA_MODE_ENABLE << 24) | ((ui_read_target_addr & (~MASK_8BIT)) >> 8);    //using byte mode to read 4 bytes

        ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_PDA_CFG_ADDR, rbuffer, 4);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
        ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_ENABLE_PDA);
        if (ret < 0)
        {
            goto exit_i2c_error;
        }
        ret = raydium_i2c_pda2_read(client, (unsigned char)(ui_read_target_addr& MASK_8BIT), rbuffer, us_read_length);
        mutex_unlock(&(ts->lock));
        if (ret < 0)
        {
            goto exit_flag_error;
        }
        memcpy((buf + ui_read_offset), rbuffer, us_read_length);

        ui_read_offset += us_read_length;
    }

    // clear update flag to get next one
    rbuffer[RAYDIUM_HOST_CMD_POS] = RAYDIUM_HOST_CMD_NO_OP;
    rbuffer[RAYDIUM_FT_CMD_POS] = g_uc_raw_data_type;
    mutex_lock(&ts->lock);
    ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR, rbuffer, RAYDIUM_FT_CMD_LENGTH);
    mutex_unlock(&ts->lock);
    if (ret < 0)
    {
        goto exit_flag_error;
    }

    return g_ui_raw_data_length;
exit_i2c_error:
    mutex_unlock(&(ts->lock));
exit_flag_error:
    return ret;
}

static ssize_t raydium_i2c_raw_data_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;
    int result = 0;
    char *temp_buf, *token, *free_temp_buf, *free_token;
    const char *delim = " ,";
    unsigned char w_data[RAYDIUM_FT_CMD_LENGTH];

    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    /* receive command line arguments string */
    if (count < 2)
    {
        return -EINVAL;
    }
    temp_buf = kzalloc(count+1, GFP_KERNEL);
    if (temp_buf  == NULL )
    {
        printk(KERN_ERR "[touch]kzalloc temp_buf failed\n");
        return -ENOMEM;
    }

    token = kzalloc(count+1, GFP_KERNEL);
    if (token  == NULL )
    {
        printk(KERN_ERR "[touch]kzalloc token failed\n");
        kfree(temp_buf);
        return -ENOMEM;
    }

    free_temp_buf = temp_buf;
    free_token = token;

    ret = (ssize_t) count;

    strncpy(temp_buf, buf, count);

    token = strsep(&temp_buf, delim);

    result = kstrtou8(token, 16, &g_uc_raw_data_type);

    token = strsep(&temp_buf, delim);
    if (token)
    {
        result = kstrtouint(token, 16, &g_ui_raw_data_length);
        if (result < 0)
        {
            ret = result;
            goto exit_error;
        }

    } else // without length info
    {
        ret = -EINVAL;
        goto exit_error;
    }

    if (temp_buf) // too much arguments
    {
        ret = -E2BIG;
        goto exit_error;
    }

    memset(w_data, 0x00, RAYDIUM_FT_CMD_LENGTH);

    mutex_lock(&ts->lock);
    result = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
    if (result < 0)
    {
        mutex_unlock(&ts->lock);
        ret = result;
        goto exit_error;
    }

    if (g_uc_raw_data_type > 1)
    {
        w_data[RAYDIUM_HOST_CMD_POS]=RAYDIUM_HOST_CMD_FT_MODE;
    } else
    {
        w_data[RAYDIUM_HOST_CMD_POS]=RAYDIUM_HOST_CMD_TP_MODE;
    }
    result = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR, w_data, 1);
    if (result < 0)
    {
        mutex_unlock(&ts->lock);
        ret = result;
        goto exit_error;
    }

    w_data[RAYDIUM_HOST_CMD_POS] = RAYDIUM_HOST_CMD_NO_OP;
    w_data[RAYDIUM_FT_CMD_POS] = g_uc_raw_data_type;

    result = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR, w_data, RAYDIUM_FT_CMD_LENGTH);
    mutex_unlock(&ts->lock);
    if (result < 0)
    {
        ret = result;
        goto exit_error;
    }

exit_error:
    kfree(free_token);
    kfree(free_temp_buf);

    return ret;
}
static ssize_t raydium_flag_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int buf_len = 0;


    sprintf(buf, "[touch]Raydium flag : %d\n", g_uc_raydium_flag);
    g_uc_raydium_flag &= ~RAYDIUM_INTERRUPT_FLAG;
    buf_len = strlen(buf);

    return buf_len+1;
}

static ssize_t raydium_flag_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;
    unsigned char flag = 0;

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }
    ret = kstrtou8(buf, 16, &flag);
    if (ret < 0)
    {
        return ret;
    }
    g_uc_raydium_flag = flag;
    return count;
}

static ssize_t raydium_wait_ft_int_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    unsigned int ret;
    ret = raydium_wait_ft_int(client);
    if (ret < 0)
    {
        return ret;
    }
    sprintf(buf, "Raydium raydium ftint : %d\n", ret);
    return strlen(buf) + 1;
}

static ssize_t raydium_wait_ft_int_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    return count;
}

static int raydium_hw_reset_fun(struct i2c_client *client)
{
    int ret = SUCCESS;
    unsigned int retry, retry_time = 1000;
    unsigned int irq_value;
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    //#enable INT
    g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;

    //HW reset
    printk(KERN_INFO "[touch]Raydium HW reset\n");
    gpio_direction_output(ts->rst, 1);
    gpio_direction_output(ts->rst, 0);
    msleep(RAYDIUM_RESET_INTERVAL_MSEC);
    gpio_direction_output(ts->rst, 1);

    u8_i2c_mode = PDA2_MODE;

    retry = retry_time;
    while (retry != 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        msleep(2);
        retry--;
        printk(KERN_INFO "[touch]wait software reset finish => confirm interrupt_flag result %d\n", retry);
    }

    //#disable INT
    if (retry == 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        g_uc_raydium_flag = 0;
    }

    if (g_uc_raydium_flag == RAYDIUM_INTERRUPT_FLAG)
    {
        retry = retry_time;
        irq_value = 0;
        while (retry != 0 && irq_value != 1)
        {
            irq_value = gpio_get_value(ts->pdata->irq_gpio);
            printk(KERN_INFO "[touch]irq_value is %d\n", irq_value);
            msleep(2);
            retry--;
        }
    }

    if (retry == 0)
    {
        printk(KERN_ERR "[touch]%s, FW not ready, retry error!\n", __func__);
        ret = ERROR;
    } else
    {
        msleep(500);
        raydium_irq_control(ts, ENABLE);
    }

    printk(KERN_INFO "[touch]Raydium HW reset : %d\n", ret);
    return ret;
}

static ssize_t raydium_hw_reset_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    int ret = SUCCESS;
    unsigned int retry, retry_time = 1000;
    unsigned int irq_value;
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    //#enable INT
    g_uc_raydium_flag = RAYDIUM_BOOTLOADER_FLAG;

    //HW reset
    printk(KERN_INFO "[touch]Raydium HW reset\n");
    gpio_direction_output(ts->rst, 1);
    gpio_direction_output(ts->rst, 0);
    msleep(RAYDIUM_RESET_INTERVAL_MSEC);
    gpio_direction_output(ts->rst, 1);

    u8_i2c_mode = PDA2_MODE;

    retry = retry_time;
    while (retry != 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        msleep(2);
        retry--;
        printk(KERN_INFO "[touch]wait software reset finish => confirm interrupt_flag result %d\n", retry);
    }

    //#disable INT
    if (retry == 0 && g_uc_raydium_flag != RAYDIUM_INTERRUPT_FLAG)
    {
        g_uc_raydium_flag = 0;
    }

    if (g_uc_raydium_flag == RAYDIUM_INTERRUPT_FLAG)
    {
        retry = retry_time;
        irq_value = 0;
        while (retry != 0 && irq_value != 1)
        {
            irq_value = gpio_get_value(ts->pdata->irq_gpio);
            printk(KERN_INFO "[touch]irq_value is %d\n", irq_value);
            msleep(2);
            retry--;
        }
    }

    if (retry == 0)
    {
        printk(KERN_ERR "[touch]%s, FW not ready, retry error!\n", __func__);
        ret = ERROR;
    } else
    {
        if (g_uc_raydium_selftest_flag == 0)
        {
            msleep(500);
        }
        raydium_irq_control(ts, ENABLE);
    }

    sprintf(buf, "Raydium HW Reset : %d\n", ret);
    printk("%s\n", buf);
    return strlen(buf) + 1;
}

static ssize_t raydium_palm_status_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int buf_len = 0;
    unsigned int palm_status;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    palm_status = raydium_get_palm_state(ts,0);
    sprintf(buf, "[Raydium] palm_status : %d\n", palm_status);
    buf_len = strlen(buf);
    return buf_len+1;
}

static ssize_t raydium_palm_status_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret = 0;
    unsigned char palm_value = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }

    ret = kstrtou8(buf, 16, &palm_value);
    raydium_set_palm_state(ts, palm_value);
    return count;
}

static ssize_t raydium_palm_area_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int ret = 0;
    unsigned char w_data[1];
    unsigned char palm_area = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }

    ret = kstrtou8(buf, 16, &palm_area);

    mutex_lock(&ts->lock);
    ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
    {
        mutex_unlock(&ts->lock);
        goto exit_error;
    }
    w_data[0] = palm_area;
    ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_PALM_AREA_ADDR, w_data, 1);
    if (ret < 0)
    {
        mutex_unlock(&ts->lock);
        goto exit_error;
    }
    mutex_unlock(&ts->lock);

exit_error:
    return count;
}

static ssize_t raydium_int_flag_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int buf_len = 0;

    sprintf(buf, "%d", g_uc_raydium_int_flag);
    printk(KERN_INFO "[touch]Raydium int flag = %s!!\n", buf);

    buf_len = strlen(buf);

    return buf_len+1;
}

static ssize_t raydium_int_flag_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;
    unsigned char flag = 0;

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }

    ret = kstrtou8(buf, 16, &flag);
    if (ret < 0)
    {
        return ret;
    }
    g_uc_raydium_int_flag = flag;
    printk(KERN_INFO "[touch]Raydium int flag store = %d!!\n", g_uc_raydium_int_flag);
    return count;
}

static ssize_t raydium_selftest_flag_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int buf_len = 0;

    sprintf(buf, "%d", g_uc_raydium_selftest_flag);
    printk(KERN_INFO "[touch]Raydium selftest flag = %s!!\n", buf);
    buf_len = strlen(buf);
    return buf_len+1;
}

static ssize_t raydium_selftest_flag_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;
    unsigned char flag = 0;
    printk(KERN_INFO "[touch]Raydium %s!!\n", __func__);

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }

    ret = kstrtou8(buf, 16, &flag);
    if (ret < 0)
    {
        return ret;
    }
    g_uc_raydium_selftest_flag = flag;

    printk(KERN_INFO "[touch]Raydium selftest flag store = %d!!\n", g_uc_raydium_selftest_flag);

    return count;
}

static ssize_t raydium_touch_lock_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ret = 0;
    unsigned char mode;
    unsigned char wbuffer[1];
    int i = 0;
    unsigned char buf2[4];

    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (ts->is_suspend)
    {
        printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
    }

    /* receive command line arguments string */
    if (count > 2)
    {
        return -EINVAL;
    }

    ret = kstrtou8(buf, 16, &mode);
    if (ret < 0)
    {
        return ret;
    }

    switch (mode)
    {
        case 0: /* Disable Touch lock */
            printk(KERN_INFO "[touch]ts->blank:%x\n",ts->blank);
            ts->is_close = 0;
            if (ts->is_sleep == 1)
            {
                //turn on i2c pull-up power
                ret = raydium_power_on(ts, true);
                if (ret)
                {
                    dev_err(&ts->client->dev, "[touch]%s:power on failed",__func__);
                }
                udelay(RAYDIUM_POWERON_DELAY_USEC);//500us
                mutex_lock(&ts->lock);
                if (gpio_is_valid(ts->rst))
                {
                    ret = gpio_request(ts->rst, "raydium_reset_gpio");
                    if (ret < 0)
                    {
                        dev_err(&ts->client->dev, "[touch]reset gpio request failed");
                    }
                    gpio_direction_output(ts->rst, 1);
                    gpio_direction_output(ts->rst, 0);
                    msleep(RAYDIUM_RESET_INTERVAL_MSEC);//5ms
                    gpio_direction_output(ts->rst, 1);
                    msleep(RAYDIUM_RESET_DELAY_MSEC);//100ms
                    u8_i2c_mode = PDA2_MODE;
                    gpio_free(ts->rst);
                    g_u8_unlockflag = true;
                }
                mutex_unlock(&ts->lock);
                raydium_irq_control(ts, ENABLE);
                ts->is_sleep = 0;
                printk(KERN_INFO "[touch]disable touch lock.\n");
            }
            break;

        case 1: /* Enable Touch lock */
            if (ts->is_sleep == 1)
            {
                printk(KERN_INFO "[touch]touch lock already enabled.\n");
            }

            ts->is_close = 1;
            raydium_irq_control(ts, DISABLE);

            /* release all touches */
            for (i = 0; i < ts->pdata->num_max_touches; i++)
            {
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
            }
            input_mt_report_pointer_emulation(ts->input_dev, false);
            input_sync(ts->input_dev);

            mutex_lock(&ts->lock);
            ret = raydium_i2c_pda2_set_page(ts->client, RAYDIUM_PDA2_PAGE_0);
            if (ret < 0)
            {
                printk(KERN_ERR "[touch]ret:%d\n",ret);
                goto exit_i2c_error;
            }

            wbuffer[0] = RAYDIUM_HOST_CMD_PWR_SLEEP;//fw enter sleep mode
            ret = raydium_i2c_pda2_write(ts->client, RAYDIUM_PDA2_HOST_CMD_ADDR, wbuffer, 1);
            if (ret < 0)
            {
                printk(KERN_ERR "[touch]ret:%d\n",ret);
                goto exit_i2c_error;
            }

            mdelay(750);
            memset(buf2, 0, sizeof(buf2));
            ret = raydium_i2c_pda_write(ts->client, 0x5000000C, buf2, 4);
            if (ret < 0)
            {
                printk(KERN_ERR "[touch]Disable M0 NG ret:%d\n",ret);
                goto exit_i2c_error;
            }

            //turn off i2c pull-up power
            ret = raydium_power_on(ts, false);
            if (ret)
            {
                dev_err(&ts->client->dev, "[touch]%s:power off failed",__func__);
            }
            mutex_unlock(&ts->lock);

            ts->is_sleep = 1;
            printk(KERN_INFO "[touch]enable touch lock.\n");
            break;
    }

    ret = (ssize_t) count;
    goto exit;

exit_i2c_error:
    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);

exit:
    return ret;
}

static ssize_t raydium_check_driver_version_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int buf_len = 0;
    int ret = -1;

    sprintf(buf, "Raydium Driver Version : 0x%X\n", RAYDIUM_VER);
    buf_len = strlen(buf);
    ret = buf_len + 1;
    return ret;
}

static ssize_t raydium_check_fw_version_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    unsigned char rbuffer[4];
    int buf_len = 0;
    int ret = -1;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts  = (struct raydium_ts_data *)i2c_get_clientdata(client);
    unsigned int fw_version, image_version;

    if (ts->is_suspend)
    {
        printk("[touch]Raydium IC is_suspend at %s\n", __func__);
    }

    raydium_irq_control(ts, DISABLE);
    mutex_lock(&ts->lock);

    ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
    {
        goto exit_i2c_error;
    }
    ret = raydium_i2c_pda2_read(client, RAYDIUM_PDA2_FW_VERSION_ADDR, rbuffer, 4);
    if (ret < 0)
    {
        goto exit_i2c_error;
    }
    sprintf(buf, "Raydium Touch FW Version : %02X%02X%02X%02X\n",rbuffer[0], rbuffer[1], rbuffer[2], rbuffer[3]);

    fw_version = (rbuffer[0] << 24) | (rbuffer[1] << 16) | (rbuffer[2] << 8) | rbuffer[3];
    printk(KERN_ERR "[touch]Raydium IC FW version is 0x%x\n", fw_version);

    image_version = (RadFWImage[0xF21] << 24) | (RadFWImage[0xF22] << 16) | (RadFWImage[0xF23] << 8) | RadFWImage[0xF24];
    printk(KERN_ERR "[touch]Raydium Image FW version is 0x%x\n", image_version);

    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);

    if (fw_version != image_version)
    {
        printk(KERN_INFO "[touch]%s, FW need upgrade.\n", __func__);
    }

    buf_len = strlen(buf);
    ret = buf_len + 1;
    goto exit_upgrade;

exit_i2c_error:
    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);

exit_upgrade:
    return ret;
}

static ssize_t raydium_check_panel_version_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    unsigned char rbuffer[8];
    int buf_len = 0;
    int ret = -1;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts  = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (ts->is_suspend)
        {
        printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
    }
    raydium_irq_control(ts, DISABLE);
    mutex_lock(&ts->lock);

    ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
        {
        goto exit_i2c_error;
        }
    ret = raydium_i2c_pda2_read(client, RAYDIUM_PDA2_PANEL_VERSION_ADDR, rbuffer, 8);
    if (ret < 0)
        {
        goto exit_i2c_error;
        }
    sprintf(buf, "Raydium Touch Panel Version : %02X%02X%02X%02X%02X%02X\n",
            rbuffer[0], rbuffer[1], rbuffer[2], rbuffer[3], rbuffer[4], rbuffer[5]);
    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);

    buf_len = strlen(buf);
    ret = buf_len + 1;
    goto exit_upgrade;

exit_i2c_error:
    mutex_unlock(&ts->lock);
    raydium_irq_control(ts, ENABLE);
exit_upgrade:
    return ret;
}

static ssize_t fw_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct raydium_ts_data *ts  = (struct raydium_ts_data *)i2c_get_clientdata(client);

    if (ts->fw_version == 0)
    {
        count += sprintf(buf + count, "FAIL\n");
    } else
    {
        count += sprintf(buf + count, "Pass, (fw_ver : 0x%x)\n", ts->fw_version);
    }
    return count;
}

static unsigned char i2c_burst_read(struct i2c_client *client, int int_addr, unsigned short u16ReadLen, unsigned char* p_u8_read_data)
{
    unsigned char result = ERROR;
    unsigned short u16_already_read_len = 0;
    char u8_read = 0;
    unsigned char rbuffer[4];
    int ret = 0;

    if (u16ReadLen > 4)
    {
        u8_read = 4;
    } else
    {
        u8_read = u16ReadLen;
    }
    while (u16_already_read_len < u16ReadLen)
    {
        ret = raydium_i2c_pda_read(client, int_addr, rbuffer, u8_read);
        if (ret < 0)
        {
            return result;
        }
        memcpy(p_u8_read_data + u16_already_read_len, rbuffer, u8_read);
        u16_already_read_len += u8_read;
    }
    result = SUCCESS;
    return result;
}

static unsigned char i2c_burst_write(struct i2c_client *client, int int_addr, unsigned short bWriteLen, unsigned char* bValue)
{
    unsigned char result = ERROR;
    int ret = 0;
    unsigned short u16_len = 0;
    unsigned char u8_w_len = 4;

    while (u16_len < bWriteLen)
    {
        ret = raydium_i2c_pda_write(client, int_addr, bValue + u16_len, u8_w_len);
        if (ret < 0)
        {
            return result;
        }
        u16_len += u8_w_len;
        if (u16_len + u8_w_len > bWriteLen)
        {
            u8_w_len = bWriteLen - u16_len;
        }
    }
    result = SUCCESS;
    return result;
}

static unsigned int i2c_read_4byte(struct i2c_client *client, unsigned int addr)
{
    unsigned int u32Data = 0, u16ii;
    unsigned char u8_buf[4];

    if (i2c_burst_read(client, addr, 4, u8_buf) == SUCCESS)
    {
        for (u16ii = 0; u16ii < 4 && u16ii < sizeof(u8_buf); u16ii++)
        {
            u32Data += (u8_buf[u16ii] << (8 * u16ii));
        }
    }
    return u32Data;
}

void i2c_write(struct i2c_client *client, uint32_t addr, uint32_t data)
{
    unsigned char u8_data[4];
    u8_data[0] = (uint8_t)(data);
    u8_data[1] = (uint8_t)(data >> 8);
    u8_data[2] = (uint8_t)(data >> 16);
    u8_data[3] = (uint8_t)(data >> 24);
    i2c_burst_write(client, addr, 4, u8_data);
}

static unsigned char raydium_selftest_stop_mcu(struct i2c_client *client)
{
    unsigned char wData[4];
    unsigned int u32_read;
    unsigned short u16_time_out = 10;
    unsigned int MCU_HOLD_STATUS = (0x00000001<<13);

    printk(KERN_INFO "[touch]Stop MCU\r\n");
    i2c_burst_read(client, 0x50000918, 4, wData);
    wData[0] |= 0x20 | 0x10;
    i2c_burst_write(client, 0x50000918, 4, wData);
    memset(wData, 0, sizeof(wData));
    wData[0] = 0x01;
    i2c_burst_write(client, 0x40000004, 4, wData);

    u32_read = i2c_read_4byte(client, 0x50000918);
    while ((u32_read & MCU_HOLD_STATUS) == 0 && u16_time_out-- > 0)
    {
        msleep(10);
        u32_read = i2c_read_4byte(client, 0x50000918);
    }
    if ((u32_read & MCU_HOLD_STATUS) == 0)
    {
        printk(KERN_ERR "[touch]Not Enter MCU HOLD\r\n");
        return ERROR;
    } else
    {
        printk(KERN_INFO "[touch]Enter MCU HOLD\r\n");
    }

    u32_read = i2c_read_4byte(client, 0x50000610);
    u32_read &= ~(0x7f << 8);
    u32_read |= (1 << 7);
    u32_read &= ~((1 << 7) | (1 << 6));
    u32_read |= (0xF << 8);
    i2c_write(client, 0x50000610, u32_read);

    return SUCCESS;
}

static ssize_t raydium_i2c_test_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    int ret = SUCCESS;

    unsigned char wData[64], readData[64];
    unsigned int writeIndex = 0, i2cWriteLen = 0, i2cReadLength = 0, readLength = 0;
    int addr = 0x20000000;
    unsigned int u32_test_length = 256;
    short i16_time_out = 3;
    unsigned char u8_is_pass = 0;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    printk(KERN_INFO "[touch]##RM6D010 I2C Test##\r\n");
    g_ui_length = 4;
    g_ul_addr = 0x50000918;
    raydium_i2c_pda_access_show(dev, attr, buf);
    //Stop MCU
    if (raydium_selftest_stop_mcu(client) == ERROR)
    {
        printk(KERN_ERR "[touch]Stop MCU NG\r\n");
        goto NG_CASE;
    }
    printk(KERN_INFO "[touch]After STOP MCU!\n");
    for (writeIndex = 0; writeIndex < 64; writeIndex++)
    {
        if (writeIndex % 4 == 0)
        {
            wData[writeIndex] = 0x55;
        } else if (writeIndex % 4 == 1)
        {
            wData[writeIndex] = 0xaa;
        } else if (writeIndex % 4 == 2)
        {
            wData[writeIndex] = 0xa5;
        } else if (writeIndex % 4 == 3)
        {
            wData[writeIndex] = 0x5a;
        }
    }

RETRY_I2C:
    while (i16_time_out > 0 && u8_is_pass != 1)
    {
        i2cWriteLen = 64;
        writeIndex = 0;
        printk(KERN_INFO "[touch]I2C Test Write!!!\r\n");

        while (writeIndex < u32_test_length)
        {
            if (i2c_burst_write(client, addr + writeIndex, i2cWriteLen, wData) == 0)
            {
                printk(KERN_ERR "[touch]i2c_test Write NG(ADDR=0x%x)!!\r\n", addr + writeIndex);
                i16_time_out--;
                if (raydium_hw_reset_fun(client) == ERROR)
                {
                    goto NG_CASE;
                }
                if (raydium_selftest_stop_mcu(client) < 0)
                {
                    goto NG_CASE;
                }
                goto RETRY_I2C;
            }

            writeIndex += i2cWriteLen;

            if ((u32_test_length - writeIndex) < i2cWriteLen)
            {
                i2cWriteLen = (u32_test_length - writeIndex);
            }
        }

        printk(KERN_INFO "[touch]I2C Test Read!!!\r\n");
        i2cReadLength = 64;

        while (readLength < u32_test_length)
        {
            if (i2c_burst_read(client, addr + readLength, i2cReadLength, readData) == 0)
            {
                i16_time_out--;
                printk(KERN_ERR "[touch]i2c_test Read NG(ADDR=0x%x)!!\r\n", addr + writeIndex);
                if (raydium_hw_reset_fun(client) == ERROR)
                {
                    goto NG_CASE;
                }
                if (raydium_selftest_stop_mcu(client) < 0)
                {
                    goto NG_CASE;
                }
                goto RETRY_I2C;
            }

            for (writeIndex = 0; writeIndex < i2cReadLength; writeIndex++)
            {
                if (wData[writeIndex] != readData[writeIndex])
                {
                    printk(KERN_ERR "[touch]i2c_test Read NG [%d], W=0x%x,R=0x%x\r\n", writeIndex, wData[writeIndex], readData[writeIndex]);
                    i16_time_out--;
                    if (raydium_hw_reset_fun(client) == ERROR)
                    {
                        goto NG_CASE;
                    }
                    if (raydium_selftest_stop_mcu(client) < 0)
                    {
                        goto NG_CASE;
                    }
                    goto RETRY_I2C;
                }
            }

            readLength += i2cReadLength;

            if (u32_test_length - readLength < i2cReadLength)
            {
                i2cReadLength = u32_test_length - readLength;
            }
        }

        u8_is_pass = 1;
    }

    if (!u8_is_pass)
    {
        goto NG_CASE;
    }
    ret = u8_is_pass;
    printk(KERN_INFO "[touch]I2C Test Pass!!!\r\n");
    sprintf(buf, "Raydium I2C Test : %d\n", ret);
    printk("%s\n", buf);
    return strlen(buf) + 1;

NG_CASE:
    ret = 0;
    sprintf(buf, "Raydium I2C Test : %d\n", ret);
    printk("%s\n", buf);
    return strlen(buf) + 1;
}

/* panel calibration cmd (R)
 *  example:cat raydium_ic_verion
 */
static DEVICE_ATTR(raydium_touch_calibration, S_IRUGO|S_IWUSR,
        raydium_touch_calibration_show,
        NULL);

/* check the i2c (R)
 *  example:cat raydium_check_i2c
 */
static DEVICE_ATTR(raydium_check_i2c, S_IRUGO|S_IWUSR,
        raydium_check_i2c_show,
        NULL);

/* upgrade configurate and algo firmware from app.bin (W)
 *  example:echo "offset num_of_bin length *_app.bin [length *_app.bin]" > raydium_fw_upgrade_mode
 */
static DEVICE_ATTR(raydium_fw_upgrade, S_IRUGO|S_IWUSR,
        raydium_fw_upgrade_show,
        raydium_fw_upgrade_store);

/* upgrade configurate and algo firmware from app.bin (W)
 *  example:echo "offset num_of_bin length *_app.bin [length *_app.bin]" > raydium_fw_upgrade_mode
 */
static DEVICE_ATTR(raydium_upgrade_firmware, S_IRUGO|S_IWUSR,
        NULL,
        raydium_upgrade_firmware_store);

/* change I2C comunication mode (W)
 *  example:    echo 1 > raydium_i2c_pda2_mode ==> enable pda2 mode
 *            echo 0 > raydium_i2c_pda2_mode ==> disable pda2 mode
 */
static DEVICE_ATTR(raydium_i2c_pda2_mode, S_IRUGO|S_IWUSR,
        NULL,
        raydium_i2c_pda2_mode_store);

/* I2C pda mode (R/W)
 *  example:    cat raydium_i2c_pda_access ==> read pda address provided by the following cmd
 *            echo ADDRinHEX [DATAinHEX] > raydium_i2c_pda_access ==> write pda address [data]
 */
static DEVICE_ATTR(raydium_i2c_pda_access, S_IRUGO|S_IWUSR,
        raydium_i2c_pda_access_show,
        raydium_i2c_pda_access_store);

/* I2C pda2 mode (R/W)
 *  example:    cat raydium_i2c_pda2_access ==> read pda2 address provided by the following cmd
 *            echo ADDRinHEX [DATAinHEX] > raydium_i2c_pda2_access ==> write pda2 address [data]
 */
static DEVICE_ATTR(raydium_i2c_pda2_access, S_IRUGO|S_IWUSR,
        raydium_i2c_pda2_access_show,
        raydium_i2c_pda2_access_store);

/* I2C pda2 mode page (W)
 *  example:    echo PAGEinHEX > raydium_i2c_pda2_page ==> write pda2 page
 */
static DEVICE_ATTR(raydium_i2c_pda2_page, S_IRUGO|S_IWUSR,
        NULL,
        raydium_i2c_pda2_page_store);

/* I2C read/set FT raw data (R/W)
 *  example:    cat raydium_i2c_raw_data ==> read raw data with specific length of corresponding type provided by the following cmd
 *            echo DataTypeinHEX RawDataLengthinHEX > raydium_i2c_raw_data ==> set raw data type and its length
 */
static DEVICE_ATTR(raydium_i2c_raw_data, S_IRUGO|S_IWUSR,
        raydium_i2c_raw_data_show,
        raydium_i2c_raw_data_store);

/* Read interrupt flag cmd (R)
 *  example:cat raydium_flag
 */
static DEVICE_ATTR(raydium_flag, S_IRUGO|S_IWUSR,
        raydium_flag_show,
        raydium_flag_store);

/* Read interrupt flag cmd (R)
 *  example:cat raydium_int_flag
 */
static DEVICE_ATTR(raydium_int_flag, S_IRUGO|S_IWUSR,
        raydium_int_flag_show,
        raydium_int_flag_store);

/* Read selftest flag cmd (R)
 *  example:cat raydium_int_flag
 */
static DEVICE_ATTR(raydium_selftest_flag, S_IRUGO|S_IWUSR,
        raydium_selftest_flag_show,
        raydium_selftest_flag_store);

/* Touch lock (W)
 *  example:    echo 1 > raydium_i2c_touch_lock ==> enable touch lock
 *            echo 0 > raydium_i2c_touch_lock ==> disable touch lock
 */
static DEVICE_ATTR(raydium_i2c_touch_lock, S_IRUGO|S_IWUSR,
        NULL,
        raydium_touch_lock_store);

/* show the fw version (R)
 *  example:cat raydium_fw_version
 */
static DEVICE_ATTR(raydium_check_fw_version, S_IRUGO|S_IWUSR,
        raydium_check_fw_version_show,
        NULL);

static DEVICE_ATTR(raydium_check_driver_version, S_IRUGO | S_IWUSR,
        raydium_check_driver_version_show,
        NULL);

static DEVICE_ATTR(fw_check, S_IRUGO, fw_check_show, NULL);
/* show the panel version (R)
 *  example:cat raydium_panel_version
 */
static DEVICE_ATTR(raydium_check_panel_version, S_IRUGO | S_IWUSR,
        raydium_check_panel_version_show,
        NULL);

static DEVICE_ATTR(raydium_hw_reset, S_IRUGO|S_IWUSR,
        raydium_hw_reset_show,
        NULL);
static DEVICE_ATTR(raydium_wait_ft_int, S_IRUGO|S_IWUSR,
        raydium_wait_ft_int_show,
        raydium_wait_ft_int_store);
static DEVICE_ATTR(raydium_palm_status, S_IRUGO|S_IWUSR,
        raydium_palm_status_show,
        raydium_palm_status_store);

static DEVICE_ATTR(raydium_palm_area, S_IRUGO|S_IWUSR,
        NULL,
        raydium_palm_area_store);
static DEVICE_ATTR(raydium_i2c_test, S_IRUGO | S_IWUSR,
        raydium_i2c_test_show,
        NULL);

/*add your attr in here*/
static struct attribute *raydium_attributes[] = {
    &dev_attr_raydium_touch_calibration.attr,
    &dev_attr_raydium_check_i2c.attr,
    &dev_attr_raydium_upgrade_firmware.attr,
    &dev_attr_raydium_i2c_pda2_mode.attr,
    &dev_attr_raydium_i2c_pda_access.attr,
    &dev_attr_raydium_i2c_pda2_access.attr,
    &dev_attr_raydium_i2c_pda2_page.attr,
    &dev_attr_raydium_i2c_raw_data.attr,
    &dev_attr_raydium_flag.attr,
    &dev_attr_raydium_i2c_touch_lock.attr,
    &dev_attr_raydium_fw_upgrade.attr,
    &dev_attr_raydium_check_fw_version.attr,
    &dev_attr_raydium_check_panel_version.attr,
    &dev_attr_raydium_hw_reset.attr,
    &dev_attr_raydium_palm_status.attr,
    &dev_attr_raydium_int_flag.attr,
    &dev_attr_raydium_selftest_flag.attr,
    &dev_attr_raydium_wait_ft_int.attr,
    &dev_attr_raydium_palm_area.attr,
    &dev_attr_fw_check.attr,
    &dev_attr_raydium_i2c_test.attr,
    &dev_attr_raydium_check_driver_version.attr,
    NULL
};

static const struct attribute_group raydium_attr_group = {
    .attrs = raydium_attributes
};

/*create sysfs for debug update firmware*/
static int raydium_create_sysfs(struct i2c_client *client)
{
    int ret = -1;

    ret = sysfs_create_group(&(client->dev.kobj), &raydium_attr_group);
    if (ret)
    {
        dev_err(&client->dev, "[touch]failed to register sysfs\n");
        sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
        return -EIO;
    } else
    {
        printk(KERN_INFO "[touch]create raydium sysfs attr_group succesful\n");
    }

    return ret;
}

static void raydium_release_sysfs(struct i2c_client *client)
{
    sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
}
#endif //end of CONFIG_RM_SYSFS_DEBUG

static int raydium_read_touchdata(struct raydium_ts_data *data)
{
    unsigned char buf[MAX_REPORT_PAKAGE_SIZE];
    unsigned char tp_status[MAX_TCH_STATUS_PAKAGE_SIZE];
    int ret = 0;
    unsigned char i, j, offset;
    unsigned char u8_seq_no = 0, u8_points_amount, u8_point_id;
    signed short s16_x, s16_y, s16_pressure, s16_wx, s16_wy;

    #ifdef GESTURE_EN
    //display idle mode, touch idle mode

    if (data->blank == FB_BLANK_VSYNC_SUSPEND || data->blank == FB_BLANK_POWERDOWN)
    {
	if (g_u8_unlockflag == true)
	{
		mutex_lock(&data->lock);
		ret = raydium_i2c_pda2_set_page(data->client, RAYDIUM_PDA2_PAGE_0);
		if (ret < 0)
                {
			dev_err(&data->client->dev, "%s: failed to set page for reading gesture report: %d\n", __func__, ret);
			mutex_unlock(&data->lock);
			return ret;
		}

		ret = raydium_i2c_pda2_read(data->client, RAYDIUM_PDA2_GESTURE_RPT_ADDR, buf, 4);
		if (ret < 0)
                {
			dev_err(&data->client->dev, "%s: failed to read gesture report: %d\n", __func__, ret);
			mutex_unlock(&data->lock);
			return ret;
		}

		//clear seq_num
		tp_status[POS_SEQ] = 0;
		ret = raydium_i2c_pda2_write(data->client, RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, tp_status, 1);
		if (ret < 0)
                {
			dev_err(&data->client->dev, "%s: failed to write data: %d\n", __func__, ret);
			mutex_unlock(&data->lock);
			return ret;
		}

		mutex_unlock(&data->lock);
		if (buf[0] == 1)
                {
			input_mt_slot(data->input_dev, 0);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, 100);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 100);
			input_sync(data->input_dev);
			mdelay(1);
			input_mt_slot(data->input_dev, 0);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
			input_mt_report_pointer_emulation(data->input_dev, false);
			input_sync(data->input_dev);
			g_uc_gesture_status = RAYDIUM_GESTURE_DISABLE;
			printk(KERN_INFO "[touch]display wake up with unlockflag\n");
		}
		else
                {
			printk(KERN_INFO "[touch]filter wrong interrupt\n");
		}
	}
	else
	{
		input_mt_slot(data->input_dev, 0);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, 100);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 100);
		input_sync(data->input_dev);
		mdelay(1);
		input_mt_slot(data->input_dev, 0);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
		input_mt_report_pointer_emulation(data->input_dev, false);
		input_sync(data->input_dev);
		g_uc_gesture_status = RAYDIUM_GESTURE_DISABLE;
		printk(KERN_INFO "[touch]display wake up\n");
	}
	return 0;
    }
    else
    {
    #endif
        mutex_lock(&data->lock);

        memset(buf, 0, MAX_REPORT_PAKAGE_SIZE);
        memset(tp_status, 0, MAX_TCH_STATUS_PAKAGE_SIZE);
        g_u8_unlockflag = false;
        //read touch point information
        ret = raydium_i2c_pda2_read(data->client, RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, tp_status, MAX_TCH_STATUS_PAKAGE_SIZE);
        if (ret < 0)
        {
            dev_err(&data->client->dev, "[touch]%s: failed to read data: %d\n",__func__, ret);
            mutex_unlock(&data->lock);
            return ret;
        }

        // inform IC to prepare next report
        if (u8_seq_no == tp_status[POS_SEQ])
        {    //report not updated
            mutex_unlock(&data->lock);
            //printk("%s -> report not updated.\n", __func__);
            return 0;
        }
        u8_points_amount = tp_status[POS_PT_AMOUNT];
        if (u8_points_amount > MAX_TOUCH_NUM)
        {
            mutex_unlock(&data->lock);
            return 0;
        }

        //read touch point report
        //PDA2 only support word mode
        if (u8_points_amount != 0)
        {
            ret = raydium_i2c_pda2_read(data->client, RAYDIUM_PDA2_TCH_RPT_ADDR, buf, u8_points_amount * LENGTH_PT);
        }

        if (ret < 0)
        {
            dev_err(&data->client->dev, "[touch]%s: failed to read data: %d\n", __func__, ret);
            mutex_unlock(&data->lock);
            return ret;
        }
        u8_seq_no = tp_status[POS_SEQ];
        tp_status[POS_SEQ] = 0;
        ret = raydium_i2c_pda2_write(data->client, RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, tp_status, 1);
        if (ret < 0)
        {
            dev_err(&data->client->dev, "[touch]%s: failed to write data: %d\n",__func__, ret);
            mutex_unlock(&data->lock);
            return ret;
        }
        mutex_unlock(&data->lock);
    //Report
    // TODO: Using struct+memcpy instead of array+offset
        //initialize slot update info
        for (i = 0; i < (MAX_TOUCH_NUM * 2); i++)
        {
            gst_slot_status[i].need_update = 0;
            gst_slot_status[i].pt_report_offset = 0;
        }

        //Check incoming point info
        for (i = 0; i < u8_points_amount; i++)
        {
            u8_point_id = buf[POS_PT_ID + i * LENGTH_PT];
            // Current
            for (j = 0; j < MAX_TOUCH_NUM; j++)
            {
                if (u8_point_id == gst_slot_status[j].occupied_pt_id)
                {
                    gst_slot_status[j].need_update = 1;
                    gst_slot_status[j].pt_report_offset = i;
                    break;
                }
            }
            // New coming
            if (j == MAX_TOUCH_NUM)
            {
                for (j = MAX_TOUCH_NUM; j < (MAX_TOUCH_NUM * 2); j++)
                {
                    if (!gst_slot_status[j].need_update)
                    {
                        gst_slot_status[j].occupied_pt_id = u8_point_id;
                        gst_slot_status[j].need_update = 1;
                        gst_slot_status[j].pt_report_offset = i;
                        printk(KERN_INFO "[touch]x:%d,y:%d\n",
                            buf[POS_X_L + offset] | buf[POS_X_H + offset] << SHORT_HIGH_BYTE_SHIFT,
                            buf[POS_Y_L + offset] | buf[POS_Y_H + offset] << SHORT_HIGH_BYTE_SHIFT);
                        break;
                    }
                }
            }
        }

        //Release slot with non-occupied point
        for (i = 0; i < MAX_TOUCH_NUM; i++)
        {
            if (!gst_slot_status[i].need_update)
            {
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
                gst_slot_status[i].occupied_pt_id = 0xFF;
                gst_slot_status[i].pt_report_offset = 0;
                gst_slot_status[i].need_update = 0;
            }
        }
        //Assign new one to non-occupied slot
        for (i = MAX_TOUCH_NUM; i < (MAX_TOUCH_NUM * 2); i++)
        {
            if (gst_slot_status[i].need_update)
            {
                for (j = 0; j < MAX_TOUCH_NUM; j++)
                {
                    if (!gst_slot_status[j].need_update)
                    {
                        gst_slot_status[j] = gst_slot_status[i];
                        gst_slot_status[i] = gst_slot_init_status;
                        break;
                    }
                }
            }
            else
            {
                break;
            }
        }

        for (i = 0; i < MAX_TOUCH_NUM; i++)
        {
            if (gst_slot_status[i].need_update)
            {
                offset = gst_slot_status[i].pt_report_offset * LENGTH_PT;
                s16_x = buf[POS_X_L + offset] | buf[POS_X_H + offset] << SHORT_HIGH_BYTE_SHIFT;
                s16_y = buf[POS_Y_L + offset] | buf[POS_Y_H + offset] << SHORT_HIGH_BYTE_SHIFT;
                s16_pressure = buf[POS_PRESSURE_L + offset] | buf[POS_PRESSURE_H + offset] << SHORT_HIGH_BYTE_SHIFT;
                s16_wx = buf[POS_WX_L + offset] | buf[POS_WX_H + offset] << SHORT_HIGH_BYTE_SHIFT;
                s16_wy = buf[POS_WY_L + offset] | buf[POS_WY_H + offset] << SHORT_HIGH_BYTE_SHIFT;

                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

                input_report_abs(data->input_dev, ABS_MT_POSITION_X, s16_x);
                input_report_abs(data->input_dev, ABS_MT_POSITION_Y, s16_y);
                input_report_abs(data->input_dev, ABS_MT_PRESSURE, s16_pressure);
                input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, max(s16_wx, s16_wy));
                input_report_abs(data->input_dev, ABS_MT_TOUCH_MINOR, min(s16_wx, s16_wy));
                //printk(KERN_INFO "[touch]x:%d, y:%d\n", s16_x,s16_y);
            }
        }

        input_mt_sync_frame(data->input_dev);
        input_sync(data->input_dev);

        /*
        printk("Raydium Slot status #: %d %d ; %d %d ; %d %d ; %d %d\n",
            gst_slot_status[0].need_update, gst_slot_status[0].occupied_pt_id,
            gst_slot_status[1].need_update, gst_slot_status[1].occupied_pt_id,
            gst_slot_status[2].need_update, gst_slot_status[2].occupied_pt_id,
            gst_slot_status[3].need_update, gst_slot_status[3].occupied_pt_id);
        */
    #ifdef GESTURE_EN
    }
    #endif

    return 0;
}

static void raydium_work_handler(struct work_struct *work)
{
    struct raydium_ts_data *raydium_ts =  \
            container_of(work, struct raydium_ts_data, work);
    int ret = 0;

    #ifdef GESTURE_EN
    unsigned int palm_status = 0;
    int i = 0;

    if (raydium_ts->blank == FB_BLANK_UNBLANK) //when display on can use palm to suspend
    {
        if (u8_i2c_mode == PDA2_MODE)
        {
            palm_status = raydium_get_palm_state(raydium_ts, g_uc_pre_palm_status);
        }
        if (palm_status == RAYDIUM_PALM_MODE_ENABLE)
        {
            if (raydium_ts->is_palm == 0)
            {
                // release all touches
                for (i = 0; i < raydium_ts->pdata->num_max_touches; i++)
                {
                    input_mt_slot(raydium_ts->input_dev, i);
                    input_mt_report_slot_state(raydium_ts->input_dev, MT_TOOL_FINGER, false);
                }

                input_mt_report_pointer_emulation(raydium_ts->input_dev, false);
                input_report_key(raydium_ts->input_dev, KEY_SLEEP, true); //press sleep key
                input_sync(raydium_ts->input_dev);

                printk(KERN_INFO "[touch]palm_status = %d.\n", palm_status);
                g_uc_pre_palm_status = RAYDIUM_PALM_MODE_ENABLE;
                raydium_ts->is_palm = 1;
                goto exit;
            }
        } else if ((palm_status == RAYDIUM_PALM_MODE_DISABLE) && (g_uc_pre_palm_status == RAYDIUM_PALM_MODE_ENABLE))
        {
            printk(KERN_INFO "[touch]leave palm mode.\n");
            input_report_key(raydium_ts->input_dev, KEY_SLEEP, false); //release sleep key
            input_sync(raydium_ts->input_dev);
            g_uc_pre_palm_status = RAYDIUM_PALM_MODE_DISABLE;
            raydium_ts->is_palm = 0;
            goto exit;
        }
    }
    #endif

    if (u8_i2c_mode == PDA2_MODE)
    {
        ret = raydium_read_touchdata(raydium_ts);
        if (ret < 0)
        {
            printk(KERN_ERR "[touch]raydium_read_touchdata error, ret:%d\n", ret);
        }
    }

#ifdef GESTURE_EN
exit:
#endif
    return;
}

/*The raydium device will signal the host about TRIGGER_FALLING.
 *Processed when the interrupt is asserted.
 */
static irqreturn_t raydium_ts_interrupt(int irq, void *dev_id)
{
    struct raydium_ts_data *raydium_ts = dev_id;
    bool result = false;

    if (g_uc_raydium_selftest_flag == 1)
    {
        disable_irq_nosync(raydium_ts->irq);
        raydium_ts->irq_enabled = false;
        printk(KERN_INFO "[touch]g_uc_raydium_selftest_flag = 1\n");
        g_uc_raydium_int_flag = 1;
    }
    //For bootloader wrt/erase flash and software reset interrupt
    else if ((g_uc_raydium_flag & RAYDIUM_BOOTLOADER_FLAG) != 0)
    {
        disable_irq_nosync(raydium_ts->irq);
        raydium_ts->irq_enabled = false;
        printk(KERN_INFO "[touch]g_uc_raydium_flag = %d\n", g_uc_raydium_flag);
        g_uc_raydium_flag = RAYDIUM_INTERRUPT_FLAG;
    } else
    {
        if (!work_pending(&raydium_ts->work))
        {
            result = queue_work(raydium_ts->workqueue, &raydium_ts->work);
            if (result == false) //queue_work fail
            {
                printk(KERN_ERR "[touch]queue_work fail.\n");
            } else
            {
                if (raydium_ts->blank == FB_BLANK_POWERDOWN)
                {
                    if (g_u8_unlockflag != true)
                    {
                        disable_irq_nosync(raydium_ts->irq);
                        raydium_ts->irq_enabled = false;
                    }
                } else
                {
                    /* Clear interrupts*/
                    mutex_lock(&raydium_ts->lock);
                    if (raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0) < 0)
                    {
                        disable_irq_nosync(raydium_ts->irq);
                        raydium_ts->irq_enabled = false;
                        dev_err(&raydium_ts->client->dev, "[touch]%s: failed to set page\n",__func__);
                    }
                    mutex_unlock(&raydium_ts->lock);
                }
            }
        } else //work pending
        {
            /* Clear interrupts*/
            mutex_lock(&raydium_ts->lock);
            if (raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0) < 0)
            {
                disable_irq_nosync(raydium_ts->irq);
                raydium_ts->irq_enabled = false;
                dev_err(&raydium_ts->client->dev, "[touch]%s: failed to set page in work_pending\n", __func__);
            }
            mutex_unlock(&raydium_ts->lock);
            printk(KERN_INFO "[touch]work_pending\n");
        }
    }

    return IRQ_HANDLED;
}

static int raydium_check_i2c_ready(struct raydium_ts_data *raydium_ts)
{
    unsigned char temp_buf[4];
    int ret = -1;

    mutex_lock(&raydium_ts->lock);

    if (u8_i2c_mode == PDA2_MODE)
    {
        ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0);
        if (ret < 0)
        {
            goto exit_error;
        }

        *(unsigned long *)temp_buf = (RAYDIUM_I2C_PDA_MODE_ENABLE << 24) | ((RAYDIUM_CHECK_I2C_CMD & (~MASK_8BIT)) >> 8);    //using byte mode to read 4 bytes
        ret = raydium_i2c_pda2_write(raydium_ts->client, RAYDIUM_PDA2_PDA_CFG_ADDR, temp_buf, 4);
        if (ret < 0)
        {
            goto exit_error;
        }

        ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_ENABLE_PDA);
        if (ret < 0)
        {
            goto exit_error;
        }

        ret = raydium_i2c_pda2_read(raydium_ts->client, (unsigned char)(RAYDIUM_CHECK_I2C_CMD& MASK_8BIT), temp_buf, 4);
        if (ret < 0)
        {
            goto exit_error;
        }
    } else
    {
        ret = raydium_i2c_pda_read(raydium_ts->client, RAYDIUM_CHECK_I2C_CMD, temp_buf, 4);
        if (ret < 0)
        {
            goto exit_error;
        }
    }

    printk(KERN_ERR "[touch]Raydium Touch check I2C:0x%02X%02X\n", temp_buf[3], temp_buf[2]);

exit_error:
    mutex_unlock(&raydium_ts->lock);
    return ret;
}

static int raydium_fw_update_check(struct raydium_ts_data *raydium_ts)
{

    unsigned char rbuffer[4];
    unsigned int fw_version, image_version;
    int ret = -1;

    mutex_lock(&raydium_ts->lock);
    ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
    {
        goto exit_error;
    }
    ret = raydium_i2c_pda2_read(raydium_ts->client, RAYDIUM_PDA2_FW_VERSION_ADDR, rbuffer, 4);
    if (ret < 0)
    {
        goto exit_error;
    }
    mutex_unlock(&raydium_ts->lock);

    fw_version = (rbuffer[0] << 24) | (rbuffer[1] << 16) | (rbuffer[2] << 8) | rbuffer[3];
    printk(KERN_ERR "[touch]Raydium IC FW version is 0x%x\n", fw_version);

    raydium_ts->fw_version = fw_version;

    image_version = (RadFWImage[0xF21] << 24) | (RadFWImage[0xF22] << 16) | (RadFWImage[0xF23] << 8) | RadFWImage[0xF24];
    printk(KERN_ERR "[touch]Raydium Image FW version is 0x%x\n", image_version);

    if (fw_version != image_version)
    {
        printk(KERN_ERR "[touch]FW need update.\n");
        raydium_irq_control(raydium_ts, DISABLE);
        ret = raydium_burn_fw(raydium_ts->client);
        if (ret < 0)
        {
            printk(KERN_ERR "[touch]FW update fail:%d\n", ret);
        }
        raydium_irq_control(raydium_ts, ENABLE);
        mutex_lock(&raydium_ts->lock);
        ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0);
        if (ret < 0)
        {
            goto exit_error;
        }
        ret = raydium_i2c_pda2_read(raydium_ts->client, RAYDIUM_PDA2_FW_VERSION_ADDR, rbuffer, 4);
        if (ret < 0)
        {
            goto exit_error;
        }
        mutex_unlock(&raydium_ts->lock);
        fw_version = (rbuffer[0] << 24) | (rbuffer[1] << 16) | (rbuffer[2] << 8) | rbuffer[3];
        printk(KERN_ERR "[touch]Raydium IC FW version is 0x%x\n", fw_version);
        raydium_ts->fw_version = fw_version;
    } else
    {
        printk(KERN_ERR "[touch]FW is the latest version.\n");
    }

    return ret;

exit_error:
    mutex_unlock(&raydium_ts->lock);
    return ret;
}

#if defined(CONFIG_PM)
static void raydium_ts_do_suspend(struct raydium_ts_data *ts)
{
    int i = 0;

    if (ts->is_suspend == 1)
    {
        printk(KERN_INFO "[touch]Already in suspend state\n");
        return;
    }

    raydium_irq_control(ts, DISABLE);

    //clear workqueue
    if (cancel_work_sync(&ts->work))
    {
        printk(KERN_INFO "[touch]workqueue is not empty!\n");
    }

    /* release all touches */
    for (i = 0; i < ts->pdata->num_max_touches; i++)
    {
        input_mt_slot(ts->input_dev, i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
    }
    input_mt_report_pointer_emulation(ts->input_dev, false);
    input_sync(ts->input_dev);

    #ifdef GESTURE_EN
    if (device_may_wakeup(&ts->client->dev))
    {
        printk(KERN_INFO "[touch]Device may wakeup\n");
        if (!enable_irq_wake(ts->irq))
        {
            ts->irq_wake = 1;
        }
    } else
    {
        printk(KERN_INFO "[touch]Device not wakeup\n");
    }
    raydium_irq_control(ts, ENABLE);
    #endif

    ts->is_suspend = 1;
}
static void raydium_ts_do_resume(struct raydium_ts_data *ts)
{
    if (ts->is_suspend == 0)
    {
        printk(KERN_INFO "[touch]Already in resume state\n");
        return;
    }

    //clear workqueue
    if (cancel_work_sync(&ts->work))
    {
        printk(KERN_INFO "[touch]workqueue is not empty!\n");
    }
    if (ts->is_close != 1)
    {
        printk(KERN_INFO "[touch]Already in open state\n");
        raydium_irq_control(ts, ENABLE);
    }

    #ifdef GESTURE_EN
    if (device_may_wakeup(&ts->client->dev))
    {
        printk(KERN_INFO "[touch]Device may wakeup\n");
        if (ts->irq_wake)
        {
            disable_irq_wake(ts->irq);
            ts->irq_wake = 0;
        }
    } else
    {
        printk(KERN_INFO "[touch]Device not wakeup\n");
    }
    #endif

    ts->is_suspend = 0;
    g_u8_unlockflag = false;
}

static int raydium_ts_suspend(struct device *dev)
{
    struct raydium_ts_data *ts = dev_get_drvdata(dev);
    raydium_ts_do_suspend(ts);
    return 0;
}

static int raydium_ts_resume(struct device *dev)
{
    struct raydium_ts_data *ts = dev_get_drvdata(dev);
    raydium_ts_do_resume(ts);
    return 0;
}

static const struct dev_pm_ops raydium_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
    .suspend    = raydium_ts_suspend,
    .resume        = raydium_ts_resume,
#endif //end of CONFIG_PM
};

//used for touch lock feature
static int raydium_ts_open(struct input_dev *input_dev)
{
    struct raydium_ts_data *ts;
    int ret = 0;

    printk(KERN_INFO"[touch]%s()+\n", __func__);
    ts = input_get_drvdata(input_dev);

    printk(KERN_INFO "[touch]ts->blank:%x\n",ts->blank);

    ts->is_close = 0;
    if (ts->is_sleep == 1)
    {
        //turn on i2c pull-up power
        ret = raydium_power_on(ts, true);
        if (ret)
        {
            dev_err(&ts->client->dev, "[touch]%s:power on failed",__func__);
        }
        udelay(RAYDIUM_POWERON_DELAY_USEC);//500us

        mutex_lock(&ts->lock);
        if (gpio_is_valid(ts->rst))
        {
            ret = gpio_request(ts->rst, "raydium_reset_gpio");
            if (ret < 0)
            {
                dev_err(&ts->client->dev, "[touch]reset gpio request failed");
            }
            gpio_direction_output(ts->rst, 1);
            gpio_direction_output(ts->rst, 0);
            msleep(RAYDIUM_RESET_INTERVAL_MSEC);//5ms
            gpio_direction_output(ts->rst, 1);
            msleep(RAYDIUM_RESET_DELAY_MSEC);//100ms
            u8_i2c_mode = PDA2_MODE;
            gpio_free(ts->rst);
            g_u8_unlockflag = true;
        }
        mutex_unlock(&ts->lock);
        raydium_irq_control(ts, ENABLE);
        ts->is_sleep = 0;
        printk(KERN_INFO "[touch]disable touch lock.\n");
    }

    return ret;
}

static void raydium_ts_close(struct input_dev *input_dev)
{
    struct raydium_ts_data *ts;
    int ret = 0;
    int i = 0;
    unsigned char wbuffer[1];
    unsigned char buf[4];

    printk(KERN_INFO "[touch]%s()+\n", __func__);
    ts = input_get_drvdata(input_dev);

    if (ts->is_sleep == 1)
    {
        printk(KERN_INFO "[touch]touch lock already enabled.\n");
        return;
    }

    ts->is_close = 1;
    raydium_irq_control(ts, DISABLE);

    /* release all touches */
    for (i = 0; i < ts->pdata->num_max_touches; i++)
    {
        input_mt_slot(ts->input_dev, i);
        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
    }
    input_mt_report_pointer_emulation(ts->input_dev, false);
    input_sync(ts->input_dev);

    mutex_lock(&ts->lock);
    ret = raydium_i2c_pda2_set_page(ts->client, RAYDIUM_PDA2_PAGE_0);
    if (ret < 0)
    {
        printk(KERN_ERR "[touch]ret:%d\n",ret);
        goto exit_i2c_error;
    }

    wbuffer[0] = RAYDIUM_HOST_CMD_PWR_SLEEP;//fw enter sleep mode
    ret = raydium_i2c_pda2_write(ts->client, RAYDIUM_PDA2_HOST_CMD_ADDR, wbuffer, 1);
    if (ret < 0)
    {
        printk(KERN_ERR "[touch]ret:%d\n",ret);
        goto exit_i2c_error;
    }

    mdelay(750);
    memset(buf, 0, sizeof(buf));
    ret = raydium_i2c_pda_write(ts->client, 0x5000000C, buf, 4);
    if (ret < 0)
    {
        printk(KERN_ERR "[touch]Disable M0 NG ret:%d\n",ret);
        goto exit_i2c_error;
    }

    //turn off i2c pull-up power
    ret = raydium_power_on(ts, false);
    if (ret)
    {
        dev_err(&ts->client->dev, "[touch]%s:power off failed",__func__);
    }
    mutex_unlock(&ts->lock);

    ts->is_sleep = 1;
    printk(KERN_INFO "[touch]enable touch lock.\n");
    return;

exit_i2c_error:
    mutex_unlock(&ts->lock);
    ts->is_close = 0;
    raydium_irq_control(ts, ENABLE);
    return;
}

#else
static int raydium_ts_suspend(struct device *dev)
{
    return 0;
}

static int raydium_ts_resume(struct device *dev)
{
    return 0;
}
#endif //end of CONFIG_FB

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    struct raydium_ts_data *raydium_ts =
        container_of(self, struct raydium_ts_data, fb_notif);

    if (evdata && evdata->data && event == FB_EVENT_BLANK &&
            raydium_ts && raydium_ts->client)
    {
        blank = evdata->data;
        raydium_ts->blank = (*blank);

        switch (*blank)
        {
            case FB_BLANK_UNBLANK: //screen on
                printk(KERN_INFO "[touch]FB_BLANK_UNBLANK\n");
                #ifdef GESTURE_EN
                /* clear palm status */
                g_uc_pre_palm_status = 0;
                raydium_ts->is_palm = 0;
                #endif
                raydium_ts_resume(&raydium_ts->client->dev);
                break;

            case FB_BLANK_POWERDOWN://screen off
                printk(KERN_INFO "[touch]FB_BLANK_POWERDOWN\n");
                #ifdef GESTURE_EN
                /* clear palm status */
                g_uc_pre_palm_status = 0;
                raydium_ts->is_palm = 0;
                #endif
                raydium_ts_suspend(&raydium_ts->client->dev);
                break;

            case FB_BLANK_VSYNC_SUSPEND://ambient mode
                printk(KERN_INFO "[touch]FB_BLANK_VSYNC_SUSPEND\n");
                #ifdef GESTURE_EN
                /* clear palm status */
                g_uc_pre_palm_status = 0;
                raydium_ts->is_palm = 0;
                #endif
                raydium_ts_suspend(&raydium_ts->client->dev);
                break;

            default:
                break;
        }
    }

    return 0;
}

static void raydium_register_notifier(struct raydium_ts_data *raydium_ts)
{
    memset(&raydium_ts->fb_notif,0,sizeof(raydium_ts->fb_notif));
    raydium_ts->fb_notif.notifier_call = fb_notifier_callback;

    /* register on the fb notifier and work with fb*/
    fb_register_client(&raydium_ts->fb_notif);
}

static void raydium_unregister_notifier(struct raydium_ts_data *raydium_ts)
{
    fb_unregister_client(&raydium_ts->fb_notif);
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void raydium_ts_early_suspend(struct early_suspend *handler)
{
    struct raydium_ts_data *raydium_ts = container_of(handler,
                        struct raydium_ts_data,
                        early_suspend);

    raydium_ts_do_suspend(raydium_ts);
}

static void raydium_ts_late_resume(struct early_suspend *handler)
{
    struct raydium_ts_data *raydium_ts = container_of(handler,
                        struct raydium_ts_data,
                        early_suspend);

    raydium_ts_do_resume(raydium_ts);
}
#endif //end of CONFIG_FB

#ifdef CONFIG_OF
static int raydium_get_dt_coords(struct device *dev, char *name,
                struct raydium_ts_platform_data *pdata)
{
    u32 coords[COORDS_ARR_SIZE];
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size, rc;

    prop = of_find_property(np, name, NULL);
    if (!prop)
    {
        return -EINVAL;
    }
    if (!prop->value)
    {
        return -ENODATA;
    }

    coords_size = prop->length / sizeof(u32);
    if (coords_size != COORDS_ARR_SIZE)
    {
        dev_err(dev, "[touch]invalid %s\n", name);
        return -EINVAL;
    }

    rc = of_property_read_u32_array(np, name, coords, coords_size);
    if (rc && (rc != -EINVAL))
    {
        dev_err(dev, "[touch]unable to read %s\n", name);
        return rc;
    }

    if (!strcmp(name, "raydium,display-coords"))
    {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    } else {
        dev_err(dev, "[touch]unsupported property %s\n", name);
        return -EINVAL;
    }

    return 0;
}

static int raydium_parse_dt(struct device *dev, struct raydium_ts_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    int rc = 0;
    u32 temp_val = 0;

    pdata->name = RAYDIUM_NAME;

    rc = raydium_get_dt_coords(dev, "raydium,display-coords", pdata);
    if (rc)
    {
        return rc;
    }

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "raydium,reset-gpio", 0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
    {
        return pdata->reset_gpio;
    }

    pdata->irq_gpio = of_get_named_gpio_flags(np, "raydium,irq-gpio", 0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
    {
        return pdata->irq_gpio;
    }

    rc = of_property_read_u32(np, "raydium,hard-reset-delay-ms", &temp_val);
    if (!rc)
    {
        pdata->hard_rst_dly = temp_val;
    } else
    {
        return rc;
    }

    rc = of_property_read_u32(np, "raydium,soft-reset-delay-ms", &temp_val);
    if (!rc)
    {
        pdata->soft_rst_dly = temp_val;
    } else
    {
        return rc;
    }

    rc = of_property_read_u32(np, "raydium,num-max-touches", &temp_val);
    if (!rc)
    {
        pdata->num_max_touches = temp_val;
    } else
    {
        return rc;
    }

    return 0;
}
#else
static int raydium_parse_dt(struct device *dev, struct raydium_ts_platform_data *pdata)
{
    return -ENODEV;
}
#endif //end of CONFIG_OF

static void raydium_input_set(struct input_dev *input_dev, struct raydium_ts_data *ts)
{
    int ret = 0;
    unsigned char i;
    input_dev->name = "raydium_ts";//name need same with .idc
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &ts->client->dev;
    input_dev->open = raydium_ts_open;//touch lock
    input_dev->close = raydium_ts_close;
    input_set_drvdata(input_dev, ts);

    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    /* Multitouch input params setup */
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, WIDTH_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, WIDTH_MAX, 0, 0);

    ret = input_mt_init_slots(input_dev, MAX_TOUCH_NUM,
                                INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
    if (ret)
    {
        dev_err(&ts->client->dev, "[touch]failed to initialize MT slots: %d\n", ret);
    }

    for (i = 0; i < (MAX_TOUCH_NUM * 2); i++)
    {
        gst_slot_status[i]=gst_slot_init_status;
    }

}

static int raydium_ts_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct raydium_ts_platform_data *pdata;
    struct raydium_ts_data *raydium_ts;
    struct input_dev *input_dev;
    int ret = 0;

    if (client->dev.of_node)
    {
        pdata = devm_kzalloc(&client->dev, sizeof(struct raydium_ts_platform_data), GFP_KERNEL);
        if (!pdata)
        {
            dev_err(&client->dev, "[touch]failed to allocate memory\n");
            return -ENOMEM;
        }

        ret = raydium_parse_dt(&client->dev, pdata);
        if (ret)
        {
            dev_err(&client->dev, "[touch]device tree parsing failed\n");
            goto parse_dt_failed;
        }
    } else
    {
        pdata = client->dev.platform_data;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        ret = -ENODEV;
        goto exit_check_functionality_failed;
    }

    raydium_ts = devm_kzalloc(&client->dev, sizeof(struct raydium_ts_data), GFP_KERNEL);
    if (!raydium_ts)
    {
        dev_err(&client->dev, "[touch]failed to allocate input driver data\n");
        return -ENOMEM;
    }

    mutex_init(&raydium_ts->lock);

    i2c_set_clientdata(client, raydium_ts);
    raydium_ts->irq = client->irq;
    raydium_ts->irq_enabled = true;
    raydium_ts->irq_wake = false;

    //raydium_ts->irq = pdata->irq_gpio;
    raydium_ts->rst = pdata->reset_gpio;
    raydium_ts->client = client;
    raydium_ts->pdata = pdata;
    raydium_ts->x_max = pdata->x_max - 1;
    raydium_ts->y_max = pdata->y_max - 1;
    raydium_ts->is_suspend = 0;
    raydium_ts->is_sleep = 0;
    raydium_ts->is_close = 0;
    #ifdef GESTURE_EN
    raydium_ts->is_palm = 0;
    #endif
    raydium_ts->fw_version = 0;

    device_init_wakeup(&client->dev,1);

    ret = raydium_power_init(raydium_ts, true);
    if (ret)
    {
        dev_err(&client->dev, "[touch]raydium_power_init failed\n");
        goto exit_regulator_failed;
    }

    ret = raydium_power_on(raydium_ts, true);
    if (ret)
    {
        dev_err(&client->dev, "[touch]raydium_power_on failed\n");
        goto pwr_deinit;
    }

    #ifdef MSM_NEW_VER
    ret = raydium_ts_pinctrl_init(raydium_ts);
    if (!ret && raydium_ts->ts_pinctrl)
    {
        /*
         * Pinctrl handle is optional. If pinctrl handle is found
         * let pins to be configured in active state. If not
         * found continue further without error.
         */
        ret = pinctrl_select_state(raydium_ts->ts_pinctrl,
                    raydium_ts->pinctrl_state_active);
        if (ret < 0)
        {
            dev_err(&client->dev, "[touch]failed to select pin to active state\n");
        }
    }
    #endif //end of MSM_NEW_VER

    ret = raydium_gpio_configure(raydium_ts, true);
    if (ret < 0)
    {
        dev_err(&client->dev,"[touch]failed to configure the gpios\n");
        goto err_gpio_req;
    }

    msleep(raydium_ts->pdata->soft_rst_dly);//modify dtsi to 360

    //print touch i2c ready
    ret = raydium_check_i2c_ready(raydium_ts);
    if (ret < 0)
    {
        dev_err(&client->dev, "[touch]Check I2C failed\n");
        ret = -ENODEV;
        goto exit_check_i2c;
    }

    //input device initialization
    input_dev = input_allocate_device();
    if (!input_dev)
    {
        ret = -ENOMEM;
        dev_err(&client->dev, "[touch]failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }

    raydium_ts->input_dev = input_dev;
    raydium_input_set(input_dev, raydium_ts);

    ret = input_register_device(input_dev);
    if (ret)
    {
        dev_err(&client->dev,
                    "[touch]raydium_ts_probe: failed to register input device: %s\n",
                    dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

    #ifdef GESTURE_EN
    input_set_capability(input_dev, EV_KEY, KEY_SLEEP);
    #endif

    //suspend/resume routine
    #if defined(CONFIG_FB)
    raydium_register_notifier(raydium_ts);
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
    raydium_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1; //1: Early-suspend level
    raydium_ts->early_suspend.suspend = raydium_ts_early_suspend;
    raydium_ts->early_suspend.resume = raydium_ts_late_resume;
    register_early_suspend(&raydium_ts->early_suspend);
    #endif//end of CONFIG_FB

    #ifdef CONFIG_RM_SYSFS_DEBUG
    raydium_create_sysfs(client);
    #endif//end of CONFIG_RM_SYSFS_DEBUG

    INIT_WORK(&raydium_ts->work, raydium_work_handler);
    raydium_ts->workqueue = create_singlethread_workqueue("raydium_ts");

    dev_err(&client->dev, "[touch]pdata irq : %d \n", raydium_ts->pdata->irq_gpio);//13
    dev_err(&client->dev, "[touch]client irq : %d, pdata flags : %d \n", client->irq, pdata->irqflags);//108

    ret = request_threaded_irq(gpio_to_irq(raydium_ts->pdata->irq_gpio), NULL, raydium_ts_interrupt,
                    pdata->irqflags | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->dev.driver->name,
                    raydium_ts);
    if (ret < 0)
    {
        dev_err(&client->dev, "[touch]raydium_probe: request irq failed\n");
        goto exit_irq_request_failed;
    }


    raydium_ts->irq_desc = irq_to_desc(client->irq);

    //disable_irq then enable_irq for avoid Unbalanced enable for IRQ warning
    raydium_irq_control(raydium_ts, DISABLE);
    raydium_irq_control(raydium_ts, ENABLE);

    printk(KERN_ERR "[touch]Raydium Touch driver version :0x%04X\n", RAYDIUM_VER);

    //fw update check
    ret = raydium_fw_update_check(raydium_ts);
    if (ret < 0)
    {
        dev_err(&client->dev, "[touch]FW update check failed\n");
        ret = -ENODEV;
        goto exit_irq_request_failed;
    }
    return 0;

exit_irq_request_failed:
    #if defined(CONFIG_FB)
    raydium_unregister_notifier(raydium_ts);
    #endif//end of CONFIG_FB

    cancel_work_sync(&raydium_ts->work);
    input_unregister_device(input_dev);

exit_input_register_device_failed:
    input_free_device(input_dev);

exit_input_dev_alloc_failed:
exit_check_i2c:
    if (gpio_is_valid(pdata->reset_gpio))
    {
        gpio_free(pdata->reset_gpio);
    }
    if (gpio_is_valid(pdata->irq_gpio))
    {
        gpio_free(pdata->irq_gpio);
    }
err_gpio_req:
    #ifdef MSM_NEW_VER
    if (raydium_ts->ts_pinctrl)
    {
        if (IS_ERR_OR_NULL(raydium_ts->pinctrl_state_release))
        {
            devm_pinctrl_put(raydium_ts->ts_pinctrl);
            raydium_ts->ts_pinctrl = NULL;
        } else
        {
            ret = pinctrl_select_state(raydium_ts->ts_pinctrl,
                    raydium_ts->pinctrl_state_release);
            if (ret)
            {
                pr_err("[touch]failed to select relase pinctrl state\n");
            }
        }
    }
    #endif//end of MSM_NEW_VER
    raydium_power_on(raydium_ts, false);

pwr_deinit:
    raydium_power_init(raydium_ts, false);

exit_regulator_failed:
    i2c_set_clientdata(client, NULL);

parse_dt_failed:
exit_check_functionality_failed:
    return ret;

}

static int raydium_ts_remove(struct i2c_client *client)
{
    struct raydium_ts_data *raydium_ts;
    raydium_ts = i2c_get_clientdata(client);

    #if defined(CONFIG_FB)
    raydium_unregister_notifier(raydium_ts);
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&raydium_ts->early_suspend);
    #endif//end of CONFIG_FB
    input_unregister_device(raydium_ts->input_dev);
    input_free_device(raydium_ts->input_dev);
    gpio_free(raydium_ts->rst);

#ifdef CONFIG_RM_SYSFS_DEBUG
    raydium_release_sysfs(client);
#endif //end of CONFIG_RM_SYSFS_DEBUG

    free_irq(client->irq, raydium_ts);

    if (gpio_is_valid(raydium_ts->pdata->reset_gpio))
    {
        gpio_free(raydium_ts->pdata->reset_gpio);
    }
    if (gpio_is_valid(raydium_ts->pdata->irq_gpio))
    {
        gpio_free(raydium_ts->pdata->irq_gpio);
    }
    raydium_power_on(raydium_ts, false);
    raydium_power_init(raydium_ts, false);

    kfree(raydium_ts);

    i2c_set_clientdata(client, NULL);
    return 0;
}

static const struct i2c_device_id raydium_ts_id[] = {
    {RAYDIUM_NAME, 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, raydium_ts_id);

#ifdef CONFIG_OF
static struct of_device_id raydium_match_table[] = {
    { .compatible = "raydium,raydium-ts",},
    { },
};
#else
#define raydium_match_table NULL
#endif//end of CONFIG_OF

static struct i2c_driver raydium_ts_driver = {
    .probe = raydium_ts_probe,
    .remove = raydium_ts_remove,
    .id_table = raydium_ts_id,
    .driver = {
        .name = RAYDIUM_NAME,
        .owner = THIS_MODULE,
        .of_match_table = raydium_match_table,
#if defined(CONFIG_PM)
        .pm    = &raydium_ts_pm_ops,
#endif//end of CONFIG_PM
    },
};

static int __init raydium_ts_init(void)
{
    int ret;
    ret = i2c_add_driver(&raydium_ts_driver);
    return ret;
}

static void __exit raydium_ts_exit(void)
{
    i2c_del_driver(&raydium_ts_driver);
}

module_init(raydium_ts_init);
module_exit(raydium_ts_exit);

MODULE_AUTHOR("<Rejion>");
MODULE_DESCRIPTION("Raydium TouchScreen driver");
MODULE_LICENSE("GPL");
