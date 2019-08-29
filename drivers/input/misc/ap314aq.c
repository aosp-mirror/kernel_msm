/*
 * This file is part of the AP3426, AP314AQ ,AP3212C and AP3216C sensor driver.
 * AP314AQ is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap314aq.c
 *
 * Summary:
 *	AP314AQ device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine.
 *					 2. Fix the index of reg array error in em write.
 * 02/22/12 YC       3. Merge AP3426 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John	  Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3426 Ver 1.0, ported for Nexus 7
 * 09/24/14 kevin    Modify for Qualcomm8x10 to support device tree
 */

#include <linux/kernel.h>
#include <linux/i2c-dev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "ap314aq.h"
#include <linux/regulator/consumer.h>
#include <linux/ioctl.h>

#define AP314AQ_DRV_NAME	"ap314aq"
#define DRIVER_VERSION		"1.0"
//#define AP314AQ_DEBUG_MORE

#define PL_TIMER_DELAY 200

/* misc define */
#define MIN_ALS_POLL_DELAY_MS	110


#define AP314AQ_VDD_MIN_UV	2000000
#define AP314AQ_VDD_MAX_UV	3300000
#define AP314AQ_VIO_MIN_UV	1750000
#define AP314AQ_VIO_MAX_UV	1950000

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("LDBG: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif

static void pl_timer_callback(unsigned long pl_data);
static int ap314aq_power_ctl(struct ap314aq_data *data, bool on);
static int ap314aq_power_init(struct ap314aq_data*data, bool on);
static int ap314aq_read_cal_data(char *file_path);

static struct ap314aq_data *private_pl_data = NULL;
// AP314AQ register
static u8 ap314aq_reg_to_idx_array[AP314AQ_MAX_REG_NUM] = {
	0,	1,	2,	0xff,	0xff,	0xff,	3,	0xff,
	0xff,	0xff,	4,	5,	6,	7,	8,	9,
	10,	0xff,	0xff,	0xff,	0xff,	0xff,	0xff,	0xff,
	0xff,	0xff,	11,	12,	13,	14,	0xff,	0xff,
	15,	16,	17,	18,	19,	20,	21,	0xff,
	22,	23,	24,	25,	26,	27         //20-2f
};
static u8 ap314aq_reg[AP314AQ_NUM_CACHABLE_REGS] = {
	0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
	0x10,0x1A,0x1B,0x1C,0x1D,0x20,0x21,0x22,0x23,0x24,
	0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D
};
// AP314AQ range
static int ap314aq_range[4] = {32768,8192,2048,512};
//static u16 ap314aq_threshole[8] = {28,444,625,888,1778,3555,7222,0xffff};

static u8 *reg_array = ap314aq_reg;
static int *range = ap314aq_range;

static int misc_ps_opened = 0;

struct regulator *vdd;
struct regulator *vio;
bool power_enabled;
/*
 * register access helpers
 */

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "ap314aq-llob",
	.vendor = "DYNA IMAGE",
	.version = 1,
	.handle = SENSORS_OFFBODY_DETEC_HANDLE,
	.type = SENSOR_TYPE_LOW_LATENCY_OFFBODY_DETECT,
	.max_range = "2.0",
	.resolution = "2.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int __ap314aq_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap314aq_data *data = i2c_get_clientdata(client);

    return (data->reg_cache[ap314aq_reg_to_idx_array[reg]] & mask) >> shift;
}

static int __ap314aq_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap314aq_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;

    tmp = data->reg_cache[ap314aq_reg_to_idx_array[reg]];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    if (!ret)
	data->reg_cache[ap314aq_reg_to_idx_array[reg]] = tmp;

    return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap314aq_get_range(struct i2c_client *client)
{
    u8 idx = __ap314aq_read_reg(client, AP314AQ_REG_ALS_CONF,
	    AP314AQ_ALS_RANGE_MASK, AP314AQ_ALS_RANGE_SHIFT);
    return range[idx];
}

static int ap314aq_set_range(struct i2c_client *client, int range)
{
    return __ap314aq_write_reg(client, AP314AQ_REG_ALS_CONF,
	    AP314AQ_ALS_RANGE_MASK, AP314AQ_ALS_RANGE_SHIFT, range);
}

/* mode */
static int ap314aq_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap314aq_read_reg(client, AP314AQ_REG_SYS_CONF,
	    AP314AQ_REG_SYS_CONF_MASK, AP314AQ_REG_SYS_CONF_SHIFT);
    return ret;
}

static int ap314aq_set_mode(struct i2c_client *client, int mode)
{
    int ret;

#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s, mode = %d\n", __func__, mode);
#endif
    misc_ps_opened = mode & AP314AQ_SYS_ALS_ENABLE;

    ret = __ap314aq_write_reg(client, AP314AQ_REG_SYS_CONF,
	    AP314AQ_REG_SYS_CONF_MASK, AP314AQ_REG_SYS_CONF_SHIFT, mode);
    return ret;
}

/* PX low threshold */
static int ap314aq_get_plthres(struct i2c_client *client)
{
    int lsb, msb;

    lsb = i2c_smbus_read_byte_data(client, AP314AQ_REG_PS_THDL_L);
    msb = i2c_smbus_read_byte_data(client, AP314AQ_REG_PS_THDL_H);
    return ((msb << 8) | lsb);
}

static int ap314aq_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP314AQ_REG_PS_THDL_L_MASK;

    err = __ap314aq_write_reg(client, AP314AQ_REG_PS_THDL_L,
	    AP314AQ_REG_PS_THDL_L_MASK, AP314AQ_REG_PS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap314aq_write_reg(client, AP314AQ_REG_PS_THDL_H,
	    AP314AQ_REG_PS_THDL_H_MASK, AP314AQ_REG_PS_THDL_H_SHIFT, msb);

    return err;
}

/* PX high threshold */
static int ap314aq_get_phthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = i2c_smbus_read_byte_data(client, AP314AQ_REG_PS_THDH_L);
    msb = i2c_smbus_read_byte_data(client, AP314AQ_REG_PS_THDH_H);
    return ((msb << 8) | lsb);
}

static int ap314aq_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP314AQ_REG_PS_THDH_L_MASK;

    err = __ap314aq_write_reg(client, AP314AQ_REG_PS_THDH_L,
	    AP314AQ_REG_PS_THDH_L_MASK, AP314AQ_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap314aq_write_reg(client, AP314AQ_REG_PS_THDH_H,
	    AP314AQ_REG_PS_THDH_H_MASK, AP314AQ_REG_PS_THDH_H_SHIFT, msb);

    return err;
}
/* reset p sensor pthreshold include calibration data */
static int ap314aq_set_ps_thres(struct i2c_client *client)
{
    int value_l, value_h;

#if 0
    if (n2f > 0 && n2f < AP314AQ_PS_THRESHOLD_LOW){
	value_l = n2f/AP314AQ_PS_CAL_GAIN;
    }
    else{
	value_l = AP314AQ_PS_THRESHOLD_LOW/AP314AQ_PS_CAL_GAIN;
    }
#endif
    value_l = AP314AQ_PS_THRESHOLD_LOW; //Using golden value to setting the low threshold
    value_h = AP314AQ_PS_THRESHOLD_HIGH;

    ap314aq_set_plthres(client, value_l);
    ap314aq_set_phthres(client, value_h);
    LDBG("Complete to set plthres = %d, phthres = %d\n", value_l, value_h);

    return 0;
}

static int ap314aq_get_object(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP314AQ_OBJ_COMMAND);
    LDBG("val=%x\n", val);
    val &= AP314AQ_OBJ_MASK;

//    return val >> AP314AQ_OBJ_SHIFT;
	return (val >> AP314AQ_OBJ_SHIFT);
}

static int ap314aq_get_intstat(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP314AQ_REG_SYS_INTSTATUS);
    val &= AP314AQ_REG_SYS_INT_MASK;

    return val >> AP314AQ_REG_SYS_INT_SHIFT;
}

static int ap314aq_get_px_value(struct i2c_client *client)
{
    int lsb, msb;

    lsb = i2c_smbus_read_byte_data(client, AP314AQ_REG_PS_DATA_LOW);

    if (lsb < 0)
	return lsb;

    msb = i2c_smbus_read_byte_data(client, AP314AQ_REG_PS_DATA_HIGH);

    if (msb < 0)
	return msb;

    LDBG("%s, IR(lsb) = %d, IR(msb) = %d\n", __func__, (u32)(lsb), (u32)(msb));
    return (u32)(((msb & AL314AQ_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AL314AQ_REG_PS_DATA_LOW_MASK));
}

static int ap314aq_ps_enable(struct ap314aq_data *ps_data,int enable)
{
    int32_t ret;
#ifdef AP314AQ_DEBUG_MORE
     LDBG("%s, misc_ps_opened = %d, enable= %d \n", __func__, misc_ps_opened, enable);
#endif
    if(misc_ps_opened == enable){
	if(enable)
		queue_work(ps_data->psensor_wq, &ps_data->psensor_work);
	return 0;
    }

    if(enable && !(ps_data->load_cal))
    {
	ap314aq_set_ps_thres(ps_data->client);
	ps_data->load_cal = true;
    }
    misc_ps_opened = enable;
    ret = __ap314aq_write_reg(ps_data->client,
        AP314AQ_REG_SYS_CONF, AP314AQ_REG_SYS_INT_PMASK, 1, enable);
    if(ret < 0){
	printk("ps enable error!!!!!!\n");
    }

    if(enable) {
	ret = mod_timer(&ps_data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
	enable_irq(ps_data->client->irq);
	enable_irq_wake(ps_data->client->irq);
    }

    return ret;
}

/*********************************************************************
proximity sensor register & unregister
********************************************************************/
static ssize_t ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start..\n", __func__);
#endif

    ret = misc_ps_opened;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        struct ap314aq_data *ps_data =  dev_get_drvdata(dev);
        uint8_t en;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start(en=%s)..\n", __func__,buf);
#endif
        if (sysfs_streq(buf, "1"))
                en = 1;
        else if (sysfs_streq(buf, "0"))
                en = 0;
        else
        {
                printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
                return -EINVAL;
        }
		LDBG("%s, en = %d\n", __func__, (u32)(en));
    ap314aq_ps_enable(ps_data, en);
    return size;
}

static struct device_attribute ps_enable_attribute = __ATTR(enable, 0664 , ps_enable_show, ps_enable_store);

static struct attribute *ap314aq_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    NULL
};

static struct attribute_group ap314aq_ps_attribute_group = {
        .attrs = ap314aq_ps_attrs,
};

static int ap314aq_register_psensor_device(struct i2c_client *client, struct ap314aq_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device psensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->psensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "llob";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for psensor\n", __FUNCTION__);
	goto done;
    }

    rc = sysfs_create_group(&input_dev->dev.kobj, &ap314aq_ps_attribute_group);// every devices register his own devices

done:
    return rc;
}

static void ap314aq_unregister_psensor_device(struct i2c_client *client, struct ap314aq_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}

/* range */
static ssize_t ap314aq_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start..\n", __func__);
#endif
    return sprintf(buf, "%i\n", ap314aq_get_range(data->client));
}

static ssize_t ap314aq_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start..\n", __func__);
#endif

    if ((kstrtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap314aq_set_range(data->client, val);

    return (ret < 0)? ret:count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
	ap314aq_show_range, ap314aq_store_range);

//kevindang for msm8916 20141010
static ssize_t ap314aq_ps_enable_set(struct sensors_classdev *sensors_cdev,
					   unsigned int enabled)
{
   struct ap314aq_data *ps_data = container_of(sensors_cdev,
					   struct ap314aq_data, ps_cdev);
   int err;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start (enabled=%d )..\n", __func__,enabled);
#endif

   err = ap314aq_ps_enable(ps_data,enabled);


   if (err < 0)
	   return err;
   return 0;
}


//end
static int ap314aq_power_ctl(struct ap314aq_data *data, bool on)
{
	int ret = 0;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start (on=%d :power_enabled=%d )..\n", __func__,on,data->power_enabled);
#endif
	if (!on && data->power_enabled)
	{
		ret = regulator_disable(data->vdd);
		if (ret)
		{
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret)
		{
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			ret = regulator_enable(data->vdd);
			if (ret)
			{
				dev_err(&data->client->dev,
					"Regulator vdd enable failed ret=%d\n",
					ret);
			}
			return ret;
		}

		data->power_enabled = on;
		printk(KERN_INFO "%s: disable ap314aq power", __func__);
		dev_dbg(&data->client->dev, "ap314aq_power_ctl on=%d\n",
				on);
	}
	else if (on && !data->power_enabled)
	{
		ret = regulator_enable(data->vdd);
		if (ret)
		{
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
		msleep(15);
		ret = regulator_enable(data->vio);
		if (ret)
		{
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}

		data->power_enabled = on;
		printk(KERN_INFO "%s: enable ap314aq power", __func__);
	}
	else
	{
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int ap314aq_power_init(struct ap314aq_data*data, bool on)
{
	int ret;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start (on=%d)..\n", __func__,on);
#endif

	if (!on)
	{
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, AP314AQ_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, AP314AQ_VIO_MAX_UV);

		regulator_put(data->vio);
	}
	else
	{
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd))
		{
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0)
		{
			ret = regulator_set_voltage(data->vdd,
					AP314AQ_VDD_MIN_UV,
					AP314AQ_VDD_MAX_UV);
			if (ret)
			{
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio))
		{
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0)
		{
			ret = regulator_set_voltage(data->vio,
					AP314AQ_VIO_MIN_UV,
					AP314AQ_VIO_MAX_UV);
			if (ret)
			{
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, AP314AQ_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}


/* mode */
static ssize_t ap314aq_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap314aq_get_mode(data->client));
}

static ssize_t ap314aq_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if ((kstrtoul(buf, 10, &val) < 0) || (val > 7))
	return -EINVAL;

    ret = ap314aq_set_mode(data->client, val);

    if (ret < 0)
	return ret;
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));

    if(ret)
	LDBG("Timer Error\n");
    return count;
}

static DEVICE_ATTR(mode, 0664 ,
	ap314aq_show_mode, ap314aq_store_mode);

/* Px data */
static ssize_t ap314aq_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);

    /* No Px data if power down */
    if (ap314aq_get_mode(data->client) == AP314AQ_SYS_DEV_DOWN)
	return -EBUSY;

    return sprintf(buf, "%d\n", ap314aq_get_px_value(data->client));
}

static DEVICE_ATTR(pxvalue, S_IRUGO, ap314aq_show_pxvalue, NULL);


/* proximity object detect */
static ssize_t ap314aq_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap314aq_get_object(data->client));
}

static DEVICE_ATTR(object, S_IRUGO, ap314aq_show_object, NULL);

/* proximity ping detect */
static ssize_t ap314aq_show_ping(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    int val,pass;

    val = ap314aq_get_phthres(data->client);
    if(val == AP314AQ_PS_THRESHOLD_HIGH)
	pass=1;
    else
	pass=0;
    LDBG("val=%d, PS_THRESHOLD_HIGH=%d : pass=%d\n",val,AP314AQ_PS_THRESHOLD_HIGH,pass);

    return sprintf(buf, "Ping : %d\n",pass );
}

static DEVICE_ATTR(ping, S_IRUGO, ap314aq_show_ping, NULL);

/* Px low threshold */
static ssize_t ap314aq_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap314aq_get_plthres(data->client));
}

static ssize_t ap314aq_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (kstrtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap314aq_set_plthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(plthres, S_IWUSR | S_IRUGO,
	ap314aq_show_plthres, ap314aq_store_plthres);

/* Px high threshold */
static ssize_t ap314aq_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap314aq_get_phthres(data->client));
}

static ssize_t ap314aq_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (kstrtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap314aq_set_phthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(phthres, S_IWUSR | S_IRUGO,
	ap314aq_show_phthres, ap314aq_store_phthres);

/* read p sensor calibration data */
static int ap314aq_read_cal_data(char *file_path)
{
    struct file *cal_filp = NULL;
    mm_segment_t old_fs;
    char buf[128];
    int val = 0;

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    cal_filp = filp_open(file_path, O_RDONLY, 0666);

    if (IS_ERR(cal_filp))
    {
	LDBG(" : Can't open calibration file (%s)\n", file_path);
	set_fs(old_fs);
	return AP314AQ_PS_THRESHOLD_BIAS;
    }

    memset(buf,'\0',sizeof(buf));
    cal_filp->f_op->read(cal_filp, buf, sizeof(buf), &cal_filp->f_pos);

    if (kstrtouint(buf, 10, &val) < 0){
	LDBG(" : read calibration file error (%s)!\n", file_path);
	return AP314AQ_PS_THRESHOLD_BIAS;
    }

    filp_close(cal_filp, current->files);
    set_fs(old_fs);

    return val;
}
/* Write p sensor calibration data */
static int ap314aq_write_cal_data(char *file_path, uint32_t adc_value)
{
    struct file *cal_filp = NULL;
    mm_segment_t old_fs;
    char result_buf[128];
    int w_len=0;
    int ret = 0;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    cal_filp = filp_open(file_path, O_CREAT | O_TRUNC | O_WRONLY, 0666);

    if (IS_ERR(cal_filp))
    {
	LDBG(" : Can't create calibration file (%s)\n", file_path);
	set_fs(old_fs);
	return -1;
    }

    memset(result_buf,'\0',sizeof(result_buf));
    w_len = sprintf(result_buf, "%d", adc_value);
    ret = cal_filp->f_op->write(cal_filp, result_buf, w_len, &cal_filp->f_pos);

    if (ret != w_len)
    {
	LDBG(" : write calibration file error (%s)!\n", file_path);
	return -2;
    }

    filp_close(cal_filp, current->files);
    set_fs(old_fs);
    return 0;
}
/* Get P sensor average adc */
static int ap314aq_read_ps_adc_value(struct i2c_client *client, uint32_t *adc_value)
{
    static const int count = 5;
    int total_adc = 0 ;
    int i = 0 ;
    //Reset the wait time to 0 & disable INT
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_WAITTIME,0);
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_INTCTRL,AP314AQ_SYS_DEV_INT_DISABLE);
    //enable p
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_CONF,AP314AQ_SYS_PS_ENABLE);

    for(i = 0 ; i < count ; i++)
    {
	int tmp_adc = 0;
	msleep(10);
	tmp_adc = ap314aq_get_px_value(client);
        if(tmp_adc < 0){
		LDBG(" : get px value Fail = %d \n", tmp_adc);
		return -1;
	}
	total_adc += tmp_adc;
	LDBG(" : count = %d  , adc = %d, total_adc = %d\n", i, tmp_adc,total_adc);
    }
    //Recover the wait time to default & enable P INT
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_WAITTIME,AP314AQ_WAITING_TIME);
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_INTCTRL,AP314AQ_SYS_PS_INT_ENABLE);
    //disable p
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_CONF,AP314AQ_SYS_DEV_DOWN);
    misc_ps_opened = 0;
    *adc_value = (int)total_adc/count;
    return 0;
}

/* P sensor calibration bias*/
static ssize_t ap314aq_show_pxcal(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    uint32_t adc_value =0;
    int ret = 0;

    ret = ap314aq_read_ps_adc_value(data->client,&adc_value);
    if (ret < 0)
    {
	LDBG(" : read ps adc value Fail = %d \n", adc_value);
	ret = sprintf(buf, "Fail : %d\n", adc_value);
	return ret;
    }
    ret = ap314aq_write_cal_data(AP314AQ_CLAIBRATION_BIAS_PATH,adc_value);
    LDBG(" K1 criteria rule : %d need < %d\n", adc_value, AP314AQ_K1_CRITERIA);
    if (ret < 0 || adc_value >= AP314AQ_K1_CRITERIA){
	ret = sprintf(buf, "Fail : %d\n", adc_value);
    }
    else{
	ret = sprintf(buf, "Pass : %d\n", adc_value);
    }
    return ret;
}

static DEVICE_ATTR(pxcal, S_IWUSR | S_IRUGO,
	ap314aq_show_pxcal, NULL);

/* P sensor calibration near to far */
static ssize_t ap314aq_show_pxcal2(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    uint32_t adc_value =0;
    int ret = 0;
    int k1 = ap314aq_read_cal_data(AP314AQ_CLAIBRATION_BIAS_PATH);

    ret = ap314aq_read_ps_adc_value(data->client,&adc_value);
    if (ret < 0)
    {
	LDBG(" : read ps adc value Fail = %d \n", adc_value);
	ret = sprintf(buf, "Fail : %d\n", adc_value);
	return ret;
    }

    ret = ap314aq_write_cal_data(AP314AQ_CLAIBRATION_N2F_PATH,adc_value);
    LDBG(" K2 criteria rule : ( %d - %d ) = %d need > %d\n", adc_value, k1, (adc_value-k1), AP314AQ_K2_CRITERIA);
    if (ret < 0 || ((int)(adc_value - k1) <= AP314AQ_K2_CRITERIA)){
	ret = sprintf(buf, "Fail : %d\n", adc_value);
    }
    else{
	ret = sprintf(buf, "Pass : %d\n", adc_value);
    }
    msleep(10);
    ap314aq_set_ps_thres(data->client);
    return ret;
}

static DEVICE_ATTR(pxcal2, S_IWUSR | S_IRUGO,
	ap314aq_show_pxcal2, NULL);

/* P sensor get avg raw */
static ssize_t ap314aq_show_pxavgerage(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap314aq_data *data = input_get_drvdata(input);
    uint32_t adc_value =0;
    int ret = 0;

    ret = ap314aq_read_ps_adc_value(data->client,&adc_value);
    if (ret < 0)
	LDBG(" : read ps adc value Fail = %d \n", adc_value);
    ret = sprintf(buf, "%d\n",adc_value);
    return ret;
}

static DEVICE_ATTR(pxavg, S_IWUSR | S_IRUGO,
	ap314aq_show_pxavgerage, NULL);

#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap314aq_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap314aq_data *data = i2c_get_clientdata(client);
    int i;
    u8 tmp;

    LDBG("%s..start!\n", __func__);

    for (i = 0; i < AP314AQ_NUM_CACHABLE_REGS; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);
	LDBG("%s : Reg[0x%x] Val[0x%x]\n", __func__, reg_array[i], tmp);
    }

    return 0;
}

static ssize_t ap314aq_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap314aq_data *data = i2c_get_clientdata(client);
    u32 addr,val;
    int ret = 0;

    LDBG("DEBUG ap314aq_em_write..\n");

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    if (!ret)
	    data->reg_cache[ap314aq_reg_to_idx_array[addr]] = val;

    return count;
}
static DEVICE_ATTR(em, S_IWUSR |S_IRUGO,
	ap314aq_em_read, ap314aq_em_write);
#endif

static struct attribute *ap314aq_attributes[] = {
    &dev_attr_range.attr,
    &dev_attr_mode.attr,
    &dev_attr_object.attr,
    &dev_attr_ping.attr,
    &dev_attr_pxvalue.attr,
    &dev_attr_plthres.attr,
    &dev_attr_phthres.attr,
    &dev_attr_pxcal.attr,
    &dev_attr_pxcal2.attr,
    &dev_attr_pxavg.attr,
#ifdef LSC_DBG
    &dev_attr_em.attr,
#endif
    NULL
};

static const struct attribute_group ap314aq_attr_group = {
    .attrs = ap314aq_attributes,
};

static int ap314aq_init_client(struct i2c_client *client)
{
    struct ap314aq_data *data = i2c_get_clientdata(client);
    int i,lsb, msb;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s: start..\n", __func__);
#endif
		/*Init control : Only PS INT enable*/
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_INTCTRL,AP314AQ_SYS_PS_INT_ENABLE);
		/*Set wait time*/
    i2c_smbus_write_byte_data(client,AP314AQ_REG_SYS_WAITTIME,AP314AQ_WAITING_TIME);
		/*Set p-sensor gain*/
//    i2c_smbus_write_byte_data(client,AP314AQ_REG_PS_CONF,AP314AQ_PS_GAIN_2);
		/*lsensor high low thread*/
    i2c_smbus_write_byte_data(client, AP314AQ_REG_ALS_THDL_L, 0);
    i2c_smbus_write_byte_data(client, AP314AQ_REG_ALS_THDL_H, 0);
    i2c_smbus_write_byte_data(client, AP314AQ_REG_ALS_THDH_L, 0);
    i2c_smbus_write_byte_data(client, AP314AQ_REG_ALS_THDH_H, 0);
		/*psensor high low thread*/
    msb = AP314AQ_PS_THRESHOLD_LOW >> 8;
    lsb = AP314AQ_PS_THRESHOLD_LOW & AP314AQ_REG_PS_THDL_L_MASK;
    i2c_smbus_write_byte_data(client, AP314AQ_REG_PS_THDL_L, lsb);
    i2c_smbus_write_byte_data(client, AP314AQ_REG_PS_THDL_H, msb);

    msb = AP314AQ_PS_THRESHOLD_HIGH >> 8;
    lsb = AP314AQ_PS_THRESHOLD_HIGH & AP314AQ_REG_PS_THDH_L_MASK;
    i2c_smbus_write_byte_data(client, AP314AQ_REG_PS_THDH_L, lsb);
    i2c_smbus_write_byte_data(client, AP314AQ_REG_PS_THDH_H, msb);

    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < AP314AQ_NUM_CACHABLE_REGS; i++) {
	int v = i2c_smbus_read_byte_data(client, reg_array[i]);
	if (v < 0)
	    return -ENODEV;
	data->reg_cache[i] = v;
    }
    /* set defaults */
    ap314aq_set_range(client, AP314AQ_ALS_RANGE_0);
    ap314aq_set_mode(client, AP314AQ_SYS_DEV_DOWN);
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s: finish..\n", __func__);
#endif
    return 0;
}

static int ap314aq_check_id(struct ap314aq_data *data)
{
    return 0;
}

void pl_timer_callback(unsigned long pl_data)
{
    struct ap314aq_data *data;

    data = private_pl_data;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : ps_open=%d \n",__func__,misc_ps_opened);
#endif
    if(1 == misc_ps_opened)
    {
	queue_work(data->psensor_wq, &data->psensor_work);
    }

}

static void psensor_work_handler(struct work_struct *w)
{

    struct ap314aq_data *data =
	container_of(w, struct ap314aq_data, psensor_work);
    struct timespec hw_time = ktime_to_timespec(ktime_get_boottime());
    int distance,pxvalue;

    distance = ap314aq_get_object(data->client);
    pxvalue = ap314aq_get_px_value(data->client); //test
    pm_wakeup_event(&data->psensor_input_dev->dev, 200);

    printk("distance=%d pxvalue=%d\n",distance,pxvalue);
    input_event(data->psensor_input_dev, EV_SYN, SYN_TIME_SEC, hw_time.tv_sec);
    input_event(data->psensor_input_dev, EV_SYN, SYN_TIME_NSEC, hw_time.tv_nsec);
    input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
    input_sync(data->psensor_input_dev);
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : distance=%d pxvalue=%d\n", __func__,distance,pxvalue);
#endif
}

static void ap314aq_work_handler(struct work_struct *w)
{

    struct ap314aq_data *data =
	container_of(w, struct ap314aq_data, ap314aq_work);
    u8 int_stat;
    int pxvalue;
    int distance;
    struct timespec hw_time = ktime_to_timespec(ktime_get_boottime());
    int_stat = ap314aq_get_intstat(data->client);
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s start ..(int_stat=0x%x)\n", __func__,int_stat);
#endif

    if((1 == misc_ps_opened) && (int_stat & AP314AQ_REG_SYS_INT_PMASK))
    {
	distance = ap314aq_get_object(data->client);
	pxvalue = ap314aq_get_px_value(data->client); //test
	pm_wakeup_event(&data->psensor_input_dev->dev, 200);

	printk("distance=%d pxvalue=%d\n",distance,pxvalue);
	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
	input_event(data->psensor_input_dev, EV_SYN, SYN_TIME_SEC, hw_time.tv_sec);
	input_event(data->psensor_input_dev, EV_SYN, SYN_TIME_NSEC, hw_time.tv_nsec);
	input_sync(data->psensor_input_dev);
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s ps_opened : distance=%d pxvalue=%d\n", __func__,distance,pxvalue);
#endif
    }
    enable_irq(data->client->irq);
    enable_irq_wake(data->client->irq);
}

static irqreturn_t ap314aq_irq(int irq, void *data_)
{
    struct ap314aq_data *data = data_;

    printk("!!!!!!!!!!!!!!!!!!!!!ap314aq_irq !!!!!!!!!!!!!!!!  %d ,  %d\n",data->client->irq,gpio_to_irq(data->int_pin));
    disable_irq_nosync(data->client->irq);
    queue_work(data->ap314aq_wq, &data->ap314aq_work);

    return IRQ_HANDLED;
}


#ifdef CONFIG_OF
static int ap314aq_parse_dt(struct device *dev, struct ap314aq_data *pdata)
{
    struct device_node *dt = dev->of_node;
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start !!\n", __func__);
#endif
    if (pdata == NULL)
    {
	LDBG("%s: pdata is NULL\n", __func__);
	return -EINVAL;
    }
    pdata->int_pin = of_get_named_gpio_flags(dt, "ap314aq,irq-gpio",
                                0, &pdata->irq_flags);
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : int_pin = %d \n", __func__,pdata->int_pin);
#endif
    if (pdata->int_pin < 0)
    {
	dev_err(dev, "Unable to read irq-gpio\n");
	return pdata->int_pin;
    }
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : end !!\n", __func__);
#endif
    return 0;
}
#endif

static int ap314aq_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ap314aq_data *data;
    int err = 0;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
	err = -EIO;
	goto exit_free_gpio;
    }

    data = kzalloc(sizeof(struct ap314aq_data), GFP_KERNEL);
    if (!data)
    {
	err = -ENOMEM;
	goto exit_free_gpio;
    }

#ifdef CONFIG_OF
    if (client->dev.of_node)
    {
	LDBG("Device Tree parsing.");

	err = ap314aq_parse_dt(&client->dev, data);
#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : Device Tree parsing. err = %d\n", __func__,err);
#endif
	if (err)
	{
	    dev_err(&client->dev, "%s: ap314aq_parse_dt "
		    "for pdata failed. err = %d",
		    __func__, err);
	    goto exit_parse_dt_fail;
	}
    }
#else
    data->irq = client->irq;
#endif

    data->load_cal = false;
    data->client = client;
    i2c_set_clientdata(client, data);

    err = ap314aq_power_init(data, true);
    if (err)
	goto err_power_on;

    err = ap314aq_power_ctl(data, true);
    if (err)
	goto err_power_ctl;

    /* initialize the AP314AQ chip */
    msleep(100);
    err = ap314aq_init_client(client);
    if (err)
	goto exit_kfree;

    if(ap314aq_check_id(data) !=0 )
    {
	dev_err(&client->dev, "failed to check ap314aq id\n");
        goto exit_kfree;
    }

    err = ap314aq_register_psensor_device(client, data);
    if (err)
    {
	dev_err(&client->dev, "failed to register_psensor_device\n");
	goto exit_free_ps_device;
    }

    err = gpio_request(data->int_pin,"ap314aq-int");
    if(err < 0)
    {
	printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
        return err;
    }
    err = gpio_direction_input(data->int_pin);
    if(err < 0)
    {
        printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
        return err;
    }

    /* device wakeup initialization */
    device_init_wakeup(&client->dev, 1);

    err = request_threaded_irq(gpio_to_irq(data->int_pin), NULL, ap314aq_irq,
	    IRQF_TRIGGER_LOW  | IRQF_ONESHOT,
	    "ap314aq", data);
    if (err)
    {
	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,gpio_to_irq(data->int_pin));
	goto exit_free_ps_device;
    }

    data->psensor_wq = create_singlethread_workqueue("psensor_wq");
    if (!data->psensor_wq)
    {
	LDBG("%s: create psensor_wq workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }
    INIT_WORK(&data->psensor_work, psensor_work_handler);

    setup_timer(&data->pl_timer, pl_timer_callback, 0);

    err = sysfs_create_group(&data->client->dev.kobj, &ap314aq_attr_group);

    data->ap314aq_wq = create_singlethread_workqueue("ap314aq_wq");
    if (!data->ap314aq_wq)
    {
	LDBG("%s: create ap314aq_wq workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }
    INIT_WORK(&data->ap314aq_work, ap314aq_work_handler);


   data->ps_cdev = sensors_proximity_cdev;
   data->ps_cdev.sensors_enable = ap314aq_ps_enable_set;
   err = sensors_classdev_register(&client->dev, &data->ps_cdev);
   if(err)
	goto exit_pwoer_ctl;

    private_pl_data = data;

    err = ap314aq_power_ctl(data, true); //kevindnag for msm8916 20141010
    if (err)
	goto err_power_on;                     //end

    dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
    return 0;

err_create_wq_failed:
    if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
    if (data->psensor_wq)
	destroy_workqueue(data->psensor_wq);
    if (data->ap314aq_wq)
	destroy_workqueue(data->ap314aq_wq);

exit_free_ps_device:
    ap314aq_unregister_psensor_device(client,data);

exit_pwoer_ctl:
    ap314aq_power_ctl(data, false);
    sensors_classdev_unregister(&data->ps_cdev);

err_power_on:
    ap314aq_power_init(data, false);

err_power_ctl:

exit_kfree:
    kfree(data);
    device_init_wakeup(&client->dev, 0);

#ifdef CONFIG_OF
exit_parse_dt_fail:

#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : exit_parse_dt_fail: err = %d\n", __func__,err);
#endif
return err;
/*
    if (client->dev.of_node && data->client->dev.platform_data)
	kfree(data->client->dev.platform_data);
*/
#endif
exit_free_gpio:
    return err;
}

static int ap314aq_remove(struct i2c_client *client)
{
    struct ap314aq_data *data = i2c_get_clientdata(client);
    free_irq(gpio_to_irq(data->int_pin), data);

    ap314aq_power_ctl(data, false);

    sysfs_remove_group(&data->client->dev.kobj, &ap314aq_attr_group);

    sysfs_remove_group(&data->psensor_input_dev->dev.kobj, &ap314aq_ps_attribute_group);// every devices register his own devices

    ap314aq_unregister_psensor_device(client,data);

    ap314aq_power_init(data, false);

    device_init_wakeup(&client->dev, 0);

    ap314aq_set_mode(client, 0);
    kfree(i2c_get_clientdata(client));

    if (data->psensor_wq)
	destroy_workqueue(data->psensor_wq);
    if (data->ap314aq_wq)
	destroy_workqueue(data->ap314aq_wq);
    if(&data->pl_timer)
	del_timer(&data->pl_timer);
    return 0;
}

static void ap314aq_shutdown(struct i2c_client *client)
{
    struct ap314aq_data *data = i2c_get_clientdata(client);

    free_irq(gpio_to_irq(data->int_pin), data);

    ap314aq_power_ctl(data, false);

    sysfs_remove_group(&data->client->dev.kobj, &ap314aq_attr_group);

    sysfs_remove_group(&data->psensor_input_dev->dev.kobj, &ap314aq_ps_attribute_group);// every devices register his own devices

    ap314aq_unregister_psensor_device(client,data);

    ap314aq_power_init(data, false);

    device_init_wakeup(&client->dev, 0);

    ap314aq_set_mode(client, 0);
    kfree(i2c_get_clientdata(client));

    if (data->psensor_wq)
	destroy_workqueue(data->psensor_wq);
    if (data->ap314aq_wq)
	destroy_workqueue(data->ap314aq_wq);
    if(&data->pl_timer)
	del_timer(&data->pl_timer);

}

static const struct i2c_device_id ap314aq_id[] =
{
    { AP314AQ_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ap314aq_id);

#ifdef CONFIG_OF
static struct of_device_id ap314aq_match_table[] =
{
        {.compatible = "di_ap314aq" },
        {},
};
#else
#define ap314aq_match_table NULL
#endif

static int ap314aq_suspend(struct device *dev)
{

#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start !!\n",__func__);
#endif
        return 0;
}

static int ap314aq_resume(struct device *dev)
{

#ifdef AP314AQ_DEBUG_MORE
    LDBG("%s : start !!\n",__func__);
#endif
        return 0;
}

static SIMPLE_DEV_PM_OPS(ap314aq_pm_ops, ap314aq_suspend, ap314aq_resume);

static struct i2c_driver ap314aq_driver = {
    .driver = {
	.name	= AP314AQ_DRV_NAME,
	.owner	= THIS_MODULE,
	.of_match_table = ap314aq_match_table,
	.pm     = &ap314aq_pm_ops,
    },
    .probe	= ap314aq_probe,
    .remove	= ap314aq_remove,
    .shutdown	= ap314aq_shutdown,
    .id_table = ap314aq_id,
};

static int __init ap314aq_init(void)
{
    int ret;

    ret = i2c_add_driver(&ap314aq_driver);
    return ret;

}

static void __exit ap314aq_exit(void)
{
    i2c_del_driver(&ap314aq_driver);
}

module_init(ap314aq_init);
module_exit(ap314aq_exit);
MODULE_AUTHOR("Kevin.dang, <kevin.dang@dyna-image.com>");
MODULE_DESCRIPTION("AP314AQ driver.");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
