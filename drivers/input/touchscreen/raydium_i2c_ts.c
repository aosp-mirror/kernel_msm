/* drivers/input/touchscreen/raydium_ts.c
 *
 * Raydium TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
	unsigned char occupied_pt_id;	//Occupied point ID
	unsigned char need_update;	//Mark as info need to be updated
	unsigned char pt_report_offset;	//point info offset in report
};

struct raydium_slot_status gst_slot_status[MAX_TOUCH_NUM * 2];// The first 3 elements are currently occupied. therest is new coming points
struct raydium_slot_status gst_slot_init_status = {0xFF, 0, 0};


struct raydium_ts_platform_data {
	const char *name;
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
	unsigned int tp_firmware_algo_ver;
	unsigned int tp_firmware_cfg_ver;
	unsigned int is_sleep;
	unsigned int is_palm;
		
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct raydium_ts_platform_data *pdata;
	struct mutex lock;
	struct work_struct  work;
	struct workqueue_struct *workqueue;
	
	#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	int blank;
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
	#endif //end of CONFIG_FB
	
	//struct regulator *vdd;
	struct regulator *vcc_i2c;
	
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

unsigned char g_uc_raydium_flag = 0;
unsigned char g_uc_gesture_status = 0;
volatile unsigned char g_uc_raydium_int_flag = 0;

//unsigned char g_uc_is_sleep = 0;

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
	} else {
		if (gpio_is_valid(data->pdata->irq_gpio))
		{
			gpio_free(data->pdata->irq_gpio);
		}
		return 0;
	}
	
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
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
	if (IS_ERR_OR_NULL(data->ts_pinctrl)) {
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
			msleep(20);
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
		mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_MODE_WORD_MODE;
	} else
	{
		mode |= RAYDIUM_I2C_PDA_MODE_ENABLE;
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
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) 
	{
		dev_err(&data->client->dev,"%s: I2C read over retry limit\n",__func__);
		retval = -EIO;
	}
exit:
	return retval;
}

int raydium_i2c_pda_write(struct i2c_client *client,
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
		mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_MODE_WORD_MODE;
	}
	else
	{
		mode |= RAYDIUM_I2C_PDA_MODE_ENABLE;
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
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) 
	{
		dev_err(&data->client->dev,"[touch]%s: I2C write over retry limit\n",__func__);
		retval = -EIO;
	}
exit:
	return retval;
}

int raydium_i2c_pda2_set_page(struct i2c_client *client, unsigned char page)
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

		msleep(20);
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
	
	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) 
	{
		if (i2c_transfer(data->client->adapter, msg, 2) == 2) 
		{
			retval = length;
			break;
		}
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) 
	{
		dev_err(&data->client->dev,"[touch]%s: I2C read over retry limit\n",__func__);
		retval = -EIO;
	}

	return retval;
}

int raydium_i2c_pda2_write(struct i2c_client *client,
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
	
	buf[0] = addr;
	memcpy(&buf[1], w_data, length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) 
	{
		if (i2c_transfer(client->adapter, msg, 1) == 1) 
		{
			retval = length;
			break;
		}
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) 
	{
		dev_err(&data->client->dev,"[touch]%s: I2C write over retry limit\n",__func__);
		retval = -EIO;
	}

	return retval;
}

#ifdef CONFIG_RM_SYSFS_DEBUG
static int raydium_get_palm_state(struct raydium_ts_data *raydium_ts)
{
	unsigned char rbuffer[1];
	unsigned int ui_palm_status = 0;
	int ret = -1;

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
	return ui_palm_status;
	
exit_error:
	return ret;
}

static int raydium_get_gesture_state(struct raydium_ts_data *raydium_ts)
{
	unsigned char rbuffer[1];
	int ret = -1;

	// Read gesture state
	ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0);
	if (ret < 0)
	{
		goto exit_error;
	}
	ret = raydium_i2c_pda2_read(raydium_ts->client, RAYDIUM_PDA2_GESTURE_STATUS_ADDR, rbuffer, 1);
	if (ret < 0)
	{
		goto exit_error;
	}
	g_uc_gesture_status = rbuffer[0];
	printk(KERN_INFO "[touch]%s, g_uc_gesture_status = %d.\n", __func__, g_uc_gesture_status);		
exit_error:
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
	disable_irq(ts->irq);
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
		}
		else
		{
			msleep(1000);
		}
		ret = raydium_i2c_pda2_read(client, RAYDIUM_PDA2_HOST_CMD_ADDR, rbuffer, 1);
		if (ret < 0) {
			goto exit_i2c_error;
		}
		printk(KERN_INFO "[touch]Raydium %s return 0x%02x!!\n", __func__, rbuffer[0]);
	} while (retry++ < (SYN_I2C_RETRY_TIMES * 2));

	memcpy(buf, rbuffer, 1);

	buf_len = strlen(buf);
	ret = buf_len + 1;

exit_i2c_error:
	mutex_unlock(&ts->lock);
	enable_irq(ts->irq);
	return ret;
}

static ssize_t raydium_ic_version_show(struct device *dev,
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
	disable_irq(ts->irq);
	mutex_lock(&ts->lock);

	ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
	if (ret < 0)
	{
		goto exit_i2c_error;
	}
	*(unsigned long *)rbuffer = (RAYDIUM_I2C_PDA_MODE_ENABLE << 24) | ((RAYDIUM_IC_VERSION_CMD & (~MASK_8BIT)) >> 8);	//using byte mode to read 4 bytes

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
	ret = raydium_i2c_pda2_read(client, (unsigned char)(RAYDIUM_IC_VERSION_CMD& MASK_8BIT), rbuffer, 4);
	if (ret < 0)
	{
		goto exit_i2c_error;
	}

	sprintf(buf, "[touch]Raydium Touch IC Version : %02X%02X%02X%02X\n",
				rbuffer[3], rbuffer[2], rbuffer[1], rbuffer[0]);
	buf_len = strlen(buf);
	ret = buf_len + 1;

exit_i2c_error:
	mutex_unlock(&ts->lock);
	enable_irq(ts->irq);

	return ret;
}

/*****************************************************************************
** Function name:		bits_reverse
** Descriptions:		MSB/LSB exchange.
** parameters:			num:input data, bit_num:data bit number
** Returned value:		reverse value
*****************************************************************************/
unsigned int bits_reverse(unsigned int num, unsigned int bit_num)
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
** Function name:		rc_crc32
** Descriptions:		32 bits checksum calculation.
** parameters:			buf:start address, len:length
** Returned value:		checksum
*****************************************************************************/
unsigned int rc_crc32(const char *buf, unsigned int len, unsigned int crc)
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

/* upgrade firmware with *.bin file
 * */
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
	if(firmware_data  == NULL ) 
	{
		pr_err("[touch]kzalloc firmware_data failed\n");
		return -ENOMEM;
	}

	temp_buf = kzalloc(RAYDIUM_FW_BIN_PATH_LENGTH+1, GFP_KERNEL);
	if(temp_buf  == NULL ) 
	{
		pr_err("[touch]kzalloc temp_buf failed\n");
		kfree(firmware_data);
		return -ENOMEM;
	}

	token = kzalloc(RAYDIUM_FW_BIN_PATH_LENGTH+1, GFP_KERNEL);
	if(token  == NULL ) 
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

	/* power on */
	if (ts->is_suspend) 
	{
		printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
// TODO:Power management
	}

	/* start to upgrade binary file*/
	mutex_lock(&ts->lock);
	ret = raydium_fw_upgrade_with_bin_file(client, fwname, count);
	mutex_unlock(&ts->lock);
	if (ret < 0)
	{
		count = ret;
	}
	return count;
}

static ssize_t raydium_i2c_pda2_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char mode;
	static unsigned char mode_change;
	unsigned char temp_buf[4];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

	if (ts->is_suspend) 
	{
		printk(KERN_INFO "[touch]Raydium IC is_suspend at %s\n", __func__);
// TODO:Power management
//		goto exit_suspend;
	}

	/* receive command line arguments string */

	if (count > 2) 
	{
		return -EINVAL;
	}

	ret = kstrtou8(buf, 16, &mode);

	switch (mode) 
	{
	case 0: /* Disable PDA2 function */
		if (!mode_change) 
		{
			disable_irq(ts->irq);
		}
		mutex_lock(&(ts->lock));
		ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_2_PDA);
		mutex_unlock(&(ts->lock));
		if (ret < 0) 
		{
			enable_irq(ts->irq);
			goto exit_i2c_error;
		} else 
		{
			printk(KERN_INFO "[touch]Raydium disable i2c PDA2\n");
			mode_change = 1;
		}
		break;
	case 1: /* Enable PDA2 function */
		mutex_lock(&(ts->lock));
		ret = raydium_i2c_pda_read(client, RAYDIUM_PDA2_CTRL_CMD, temp_buf, 1);
		if (ret < 0) 
		{
			mutex_unlock(&(ts->lock));
			goto exit_i2c_error;
		}
		temp_buf[0] |= RAYDIM_ENABLE_PDA2;
		ret = raydium_i2c_pda_write(client, RAYDIUM_PDA2_CTRL_CMD, temp_buf, 1);
		mutex_unlock(&(ts->lock));
		if (ret < 0)
		{
			goto exit_i2c_error;
		}
		else 
		{
			if (mode_change) 
			{
				printk(KERN_INFO "[touch]Raydium enable PDA2 function\n");
				enable_irq(ts->irq);
				mode_change = 0;
			}
		}
		break;
	case 2: /* Disable INT */
		disable_irq(ts->irq);
		printk(KERN_INFO "[touch]Raydium disable INT\n");
		break;
	case 3: /* Enable INT */
		enable_irq(ts->irq);
		printk(KERN_INFO "[touch]Raydium enable INT\n");
		break;
	case 4: /* Disable PDA2 function , no disable INT*/
		
		mutex_lock(&(ts->lock));
		ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_2_PDA);
		mutex_unlock(&(ts->lock));
		if (ret < 0) {		
			goto exit_i2c_error;
		} else {
			printk(KERN_INFO "[touch]Raydium disable i2c PDA2(Enable INT IRQ)\n");
			mode_change = 1;
		}
		break;
	case 5: /* Enable PDA2 function , no disable INT*/
		mutex_lock(&(ts->lock));
		ret = raydium_i2c_pda_read(client, RAYDIUM_PDA2_CTRL_CMD, temp_buf, 1);
		if (ret < 0) {
			mutex_unlock(&(ts->lock));
			goto exit_i2c_error;
		}
		temp_buf[0] |= RAYDIM_ENABLE_PDA2;
		ret = raydium_i2c_pda_write(client, RAYDIUM_PDA2_CTRL_CMD, temp_buf, 1);
		mutex_unlock(&(ts->lock));
		if (ret < 0)
			goto exit_i2c_error;
		else {
			if (mode_change) {
				printk(KERN_INFO "[touch]Raydium enable PDA2 function(Enable INT IRQ)\n");				
				mode_change = 0;
			}
		}
		break;
	}

	ret = (ssize_t) count;

exit_i2c_error:
	return ret;
}

unsigned long g_ul_addr = RAYDIUM_IC_VERSION_CMD;
unsigned int g_ui_length = 1;

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
		return -EINVAL;

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
static ssize_t raydium_hw_reset_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return strlen(buf) + 1;
}
static ssize_t raydium_hw_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);
	
	printk(KERN_INFO "%s.\n", __func__);
	if (gpio_is_valid(ts->rst)) 
	{
		disable_irq(ts->irq);
		mutex_lock(&ts->lock);
		udelay(RAYDIUM_POWERON_DELAY_USEC);//500us
	
		gpio_request(ts->rst, "raydium_reset_gpio");
		
		gpio_direction_output(ts->rst, 1);
		gpio_direction_output(ts->rst, 0);
		msleep(RAYDIUM_RESET_INTERVAL_MSEC);//5ms
		gpio_direction_output(ts->rst, 1);
		msleep(RAYDIUM_RESET_DELAY_MSEC);//100ms
		gpio_free(ts->rst);
		mutex_unlock(&ts->lock);
	enable_irq(ts->irq);
	}

	printk(KERN_INFO "Raydium %s end!!\n", __func__);
	return count;
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
	if(temp_buf  == NULL ) 
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

	disable_irq(ts->irq);
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
	enable_irq(ts->irq);

exit_error:
	kfree(free_token);
	kfree(free_temp_buf);
	return ret;
}

unsigned char g_uc_raw_data_type = RAYDIUM_FT_UPDATE;
unsigned int g_ui_raw_data_length = 18 * 30 * 2;	// 1080 bytes

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
		msleep(3);
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

		*(unsigned long *)rbuffer = (RAYDIUM_I2C_PDA_MODE_ENABLE << 24) | ((ui_read_target_addr & (~MASK_8BIT)) >> 8);	//using byte mode to read 4 bytes

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
	if(temp_buf  == NULL ) 
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
		if (result < 0) {
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

	w_data[RAYDIUM_HOST_CMD_POS] = RAYDIUM_HOST_CMD_NO_OP;
	w_data[RAYDIUM_FT_CMD_POS] = g_uc_raw_data_type;

	mutex_lock(&ts->lock);
	result = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
	if (result < 0) 
	{
		mutex_unlock(&ts->lock);
		ret = result;
		goto exit_error;
	}
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
	g_uc_raydium_flag = flag;
	return count;

	
	

}
static ssize_t raydium_int_flag_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int buf_len = 0;

	printk(KERN_INFO "Raydium %s!!\n", __func__);

	sprintf(buf, "%d", g_uc_raydium_int_flag);
	printk(KERN_INFO "Raydium INT flag = %s!!\n", buf);
//	g_uc_raydium_int_flag = 0;
	buf_len = strlen(buf);

	return buf_len+1;
}

static ssize_t raydium_int_flag_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char flag = 0;
	printk(KERN_INFO "Raydium %s!!\n", __func__);

	/* receive command line arguments string */

	if (count > 2) {
		return -EINVAL;
	}
	
	ret = kstrtou8(buf, 16, &flag);
	g_uc_raydium_int_flag = flag;
	
	printk(KERN_INFO "Raydium flag = %d!!\n", g_uc_raydium_int_flag);

	return count;
}

static ssize_t raydium_touch_lock_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned char mode;
	unsigned char wbuffer[1];

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct raydium_ts_data *ts = (struct raydium_ts_data *)i2c_get_clientdata(client);

//	dev_err(dev, "%s.\n", __func__);
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
//	printk("Raydium tocuh lock mode: %d\n", mode);

	disable_irq(ts->irq);
	mutex_lock(&ts->lock);

	switch (mode) 
	{
		case 0: /* Disable Touch lock */	
		
			if(ts->is_sleep != 1)
			{
				break;
			}
			
			if (gpio_is_valid(ts->rst)) 
			{
				ret = gpio_request(ts->rst, "raydium_reset_gpio");
				if (ret) 
				{
					dev_err(&ts->client->dev, "[touch]reset gpio request failed\n");
					break;
				}
				gpio_direction_output(ts->rst, 1);
				gpio_direction_output(ts->rst, 0);
				msleep(RAYDIUM_RESET_INTERVAL_MSEC);//5ms
				gpio_direction_output(ts->rst, 1);
				msleep(RAYDIUM_RESET_DELAY_MSEC);//100ms
				gpio_free(ts->rst);
			}
			printk(KERN_INFO "[touch]Raydium %s disable touch lock!!\n", __func__);
			ts->is_sleep = 0;
			break;
		
		case 1: /* Enable Touch lock */
			if(ts->is_sleep == 1)
			{
				break;
			}
			ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
			if (ret < 0)
			{
				goto exit_i2c_error;
			}
			wbuffer[0] = RAYDIUM_HOST_CMD_PWR_SLEEP;		//fw enter sleep mode
			ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR, wbuffer, 1);
			if (ret < 0)
			{
				goto exit_i2c_error;
			}
			printk(KERN_INFO "[touch]Raydium %s enable touch lock!!\n", __func__);
			ts->is_sleep = 1;
			break;
	}

	ret = (ssize_t) count;

exit_i2c_error:
	mutex_unlock(&ts->lock);
	enable_irq(ts->irq);
	return ret;
}

static ssize_t raydium_test_gesture_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char rbuffer[1];
	int buf_len = 0;
	int ret = -1;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct raydium_ts_data *ts  = (struct raydium_ts_data *)i2c_get_clientdata(client);

	if (ts->is_suspend) 
	{
		printk(KERN_INFO "Raydium IC is_suspend at %s\n", __func__);
	}

	//fake gesture mode
	ts->is_suspend = 1;
	printk(KERN_INFO "%s fake gesture mode enable, ts->is_suspend = %d.\n", __func__, ts->is_suspend);	
	
	disable_irq(ts->irq);
	mutex_lock(&ts->lock);

	ret = raydium_i2c_pda2_set_page(client, RAYDIUM_PDA2_PAGE_0);
	if (ret < 0)
	{
		goto exit_i2c_error;
	}
	
	rbuffer[0] = RAYDIUM_GESTURE_ENABLE;		//fw enter gesture mode
	ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_GESTURE_STATUS_ADDR, rbuffer, 1);
	if (ret < 0)
	{
		goto exit_i2c_error;
	}
	msleep(100);

	/* Read fw gesture status */
	raydium_get_gesture_state(ts);

	sprintf(buf, "%d", rbuffer[0]);	//debug open
	memcpy(buf, rbuffer, 1);

	buf_len = strlen(buf);
	ret = buf_len + 1;

exit_i2c_error:
	mutex_unlock(&ts->lock);
	enable_irq(ts->irq);
	return ret;
}


/* fake gesture mode control cmd (R)
 *  example:cat raydium_gesture
 */
static DEVICE_ATTR(raydium_gesture, S_IRUGO|S_IWUSR,
		raydium_test_gesture_show,
		NULL);


/* panel calibration cmd (R)
 *  example:cat raydium_ic_verion
 */
static DEVICE_ATTR(raydium_touch_calibration, S_IRUGO|S_IWUSR,
		raydium_touch_calibration_show,
		NULL);

/* show the ic version (R)
 *  example:cat raydium_ic_verion
 */
static DEVICE_ATTR(raydium_ic_version, S_IRUGO|S_IWUSR,
		raydium_ic_version_show,
		NULL);

/* upgrade configurate and algo firmware from app.bin (W)
 *  example:echo "offset num_of_bin length *_app.bin [length *_app.bin]" > raydium_fw_upgrade_mode
 */
static DEVICE_ATTR(raydium_upgrade_firmware, S_IRUGO|S_IWUSR,
		NULL,
		raydium_upgrade_firmware_store);

/* change I2C comunication mode (W)
 *  example:	echo 1 > raydium_i2c_pda2_mode ==> enable pda2 mode
 *			echo 0 > raydium_i2c_pda2_mode ==> disable pda2 mode
 */
static DEVICE_ATTR(raydium_i2c_pda2_mode, S_IRUGO|S_IWUSR,
		NULL,
		raydium_i2c_pda2_mode_store);

/* I2C pda mode (R/W)
 *  example:	cat raydium_i2c_pda_access ==> read pda address provided by the following cmd
 *			echo ADDRinHEX [DATAinHEX] > raydium_i2c_pda_access ==> write pda address [data]
 */
static DEVICE_ATTR(raydium_i2c_pda_access, S_IRUGO|S_IWUSR,
		raydium_i2c_pda_access_show,
		raydium_i2c_pda_access_store);

/* I2C pda2 mode (R/W)
 *  example:	cat raydium_i2c_pda2_access ==> read pda2 address provided by the following cmd
 *			echo ADDRinHEX [DATAinHEX] > raydium_i2c_pda2_access ==> write pda2 address [data]
 */
static DEVICE_ATTR(raydium_i2c_pda2_access, S_IRUGO|S_IWUSR,
		raydium_i2c_pda2_access_show,
		raydium_i2c_pda2_access_store);

/* I2C pda2 mode page (W)
 *  example:	echo PAGEinHEX > raydium_i2c_pda2_page ==> write pda2 page
 */
static DEVICE_ATTR(raydium_i2c_pda2_page, S_IRUGO|S_IWUSR,
		NULL,
		raydium_i2c_pda2_page_store);

/* I2C read/set FT raw data (R/W)
 *  example:	cat raydium_i2c_raw_data ==> read raw data with specific length of corresponding type provided by the following cmd
 *			echo DataTypeinHEX RawDataLengthinHEX > raydium_i2c_raw_data ==> set raw data type and its length
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
/* Control Hardware reset cmd (R)
 *  example:echo raydium_hw_reset
 */
static DEVICE_ATTR(raydium_hw_reset, S_IRUGO | S_IWUSR,
		raydium_hw_reset_show,
		raydium_hw_reset_store);
/* Read interrupt flag cmd (R)
 *  example:cat raydium_int_flag
 */
static DEVICE_ATTR(raydium_int_flag, S_IRUGO | S_IWUSR,
		raydium_int_flag_show,
		raydium_int_flag_store);

/* Touch lock (W)
 *  example:	echo 1 > raydium_i2c_touch_lock ==> enable touch lock
 *			echo 0 > raydium_i2c_touch_lock ==> disable touch lock
 */
static DEVICE_ATTR(raydium_i2c_touch_lock, S_IRUGO | S_IWUSR,
		NULL,
		raydium_touch_lock_store);

/*add your attr in here*/
static struct attribute *raydium_attributes[] = {
	&dev_attr_raydium_touch_calibration.attr,
	&dev_attr_raydium_ic_version.attr,
	&dev_attr_raydium_upgrade_firmware.attr,
	&dev_attr_raydium_i2c_pda2_mode.attr,
	&dev_attr_raydium_i2c_pda_access.attr,
	&dev_attr_raydium_i2c_pda2_access.attr,
	&dev_attr_raydium_i2c_pda2_page.attr,
	&dev_attr_raydium_i2c_raw_data.attr,
	&dev_attr_raydium_flag.attr,
	&dev_attr_raydium_i2c_touch_lock.attr,
    &dev_attr_raydium_hw_reset.attr,
    &dev_attr_raydium_int_flag.attr,
	&dev_attr_raydium_gesture.attr,
	NULL
};

static const struct attribute_group raydium_attr_group = {
	.attrs = raydium_attributes
};

/*create sysfs for debug update firmware*/
int raydium_create_sysfs(struct i2c_client *client)
{
	int err;

	err = sysfs_create_group(&(client->dev.kobj), &raydium_attr_group);
	if (err)
	{
		dev_err(&client->dev, "[touch]failed to register sysfs\n");
		sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
		return -EIO;
	} else
	{
		printk(KERN_INFO "[touch]create raydium sysfs attr_group succesful\n");
	}

	return err;
}

void raydium_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
}
#endif //end of CONFIG_RM_SYSFS_DEBUG

static int raydium_read_touchdata(struct raydium_ts_data *data)
{
	unsigned char buf[MAX_REPORT_PAKAGE_SIZE];
    unsigned char tp_status[MAX_TCH_STATUS_PAKAGE_SIZE];
	int error = 0;
	unsigned char i, j, offset;
	unsigned char u8_seq_no = 0, u8_points_amount, u8_point_id;
	signed short s16_x, s16_y, s16_pressure, s16_wx, s16_wy;

	//display idle mode, touch idle mode
	if (data->blank == FB_BLANK_VSYNC_SUSPEND)
	{
	    //need check small area
		printk(KERN_INFO "[touch]display idle touch idle\n");
		input_mt_slot(data->input_dev, 0);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, 100);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 100);
		input_sync(data->input_dev);
		mdelay(10);
		input_mt_slot(data->input_dev, 0);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
		input_mt_report_pointer_emulation(data->input_dev, false);
		input_sync(data->input_dev);
        g_uc_gesture_status = RAYDIUM_GESTURE_DISABLE;
		return 0;
	} else if (data->blank == FB_BLANK_POWERDOWN) 
	{
		printk(KERN_INFO "[touch]display off\n");
		mutex_lock(&data->lock);
        error = raydium_i2c_pda2_set_page(data->client, RAYDIUM_PDA2_PAGE_0);
		if (error < 0) 
		{
			dev_err(&data->client->dev, "[touch]%s: failed to set page for reading gesture report: %d\n",
				__func__, error);
			mutex_unlock(&data->lock);
			return error;
		}
		
		error = raydium_i2c_pda2_read(data->client, RAYDIUM_PDA2_GESTURE_RPT_ADDR, buf, 4);
		if (error < 0) 
		{
			dev_err(&data->client->dev, "%s: failed to read gesture report: %d\n", __func__, error);
			mutex_unlock(&data->lock);
			return error;
		}
		mutex_unlock(&data->lock);				
		
		//display off mode, touch gesture mode
		if (buf[0] == 1) 
		{
			printk(KERN_INFO "[touch]%s : gesture result %d %d %d.\n", __func__, buf[0], buf[1], buf[2]);
			switch (buf[2])
			{
				case 0:
				case 2:
					if (buf[2] == 0)
					{
						printk(KERN_INFO "[touch]%s:click\n", __func__);
					} else
					{
						printk(KERN_INFO "[touch]%s:swipe\n", __func__);
					}
					input_mt_slot(data->input_dev, 0);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
					input_report_abs(data->input_dev, ABS_MT_POSITION_X, 100);
					input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 100);
					input_sync(data->input_dev);
					mdelay(10);
					input_mt_slot(data->input_dev, 0);
					input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
					input_mt_report_pointer_emulation(data->input_dev, false);
					input_sync(data->input_dev);
			        g_uc_gesture_status = RAYDIUM_GESTURE_DISABLE;			   
					break;
				default:
				    printk(KERN_INFO "[touch]%s:default\n", __func__);	

			        g_uc_gesture_status = RAYDIUM_GESTURE_DISABLE;			              
					break;
			}
			// clear gesture result
			mutex_lock(&data->lock);
			error = raydium_i2c_pda2_set_page(data->client, RAYDIUM_PDA2_PAGE_0);
			if (error < 0) 
			{
				dev_err(&data->client->dev, "[touch]%s: failed to set page for clearing gesture report: %d\n",__func__, error);
				mutex_unlock(&data->lock);
				return error;
			}

			memset(buf, 0, MAX_GESTURERESULT_SIZE);
			error = raydium_i2c_pda2_write(data->client, RAYDIUM_PDA2_GESTURE_RPT_ADDR, buf, MAX_GESTURERESULT_SIZE);
			if (error < 0) 
			{
				mutex_unlock(&data->lock);
				return error;
			}
			mutex_unlock(&data->lock);
		}

		   
	} else 
	{
		mutex_lock(&data->lock);
		error = raydium_i2c_pda2_set_page(data->client, RAYDIUM_PDA2_PAGE_0);
		if (error < 0) 
		{
			dev_err(&data->client->dev, "[touch]%s: failed to set page for reading point report: %d\n",__func__, error);
			mutex_unlock(&data->lock);
			return error;
		}

		memset(buf, 0, MAX_REPORT_PAKAGE_SIZE);
        memset(tp_status, 0, MAX_TCH_STATUS_PAKAGE_SIZE);
        //read touch point information
		error = raydium_i2c_pda2_read(data->client, RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, tp_status, MAX_TCH_STATUS_PAKAGE_SIZE);
		if (error < 0) 
		{
			dev_err(&data->client->dev, "[touch]%s: failed to read data: %d\n",__func__, error);
			mutex_unlock(&data->lock);
			return error;
		}

		// inform IC to prepare next report
		if (u8_seq_no == tp_status[POS_SEQ]) 
		{	//report not updated
			mutex_unlock(&data->lock);
			//printk("%s -> report not updated.\n", __func__);
			return 0;
		}
        u8_points_amount = tp_status[POS_PT_AMOUNT];
		if (u8_points_amount > MAX_TOUCH_NUM)
		{
			return 0;
		}

		//read touch point report
		//PDA2 only support word mode
		if (u8_points_amount != 0)
		{
			error = raydium_i2c_pda2_read(data->client, RAYDIUM_PDA2_TCH_RPT_ADDR, buf, u8_points_amount * LENGTH_PT);
		}
		
		if (error < 0) 
		{
			dev_err(&data->client->dev, "[touch]%s: failed to read data: %d\n", __func__, error);
			mutex_unlock(&data->lock);
			return error;
		}
		u8_seq_no = tp_status[POS_SEQ];
		tp_status[POS_SEQ] = 0;
		error = raydium_i2c_pda2_write(data->client, RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, tp_status, 1);
		if (error < 0) 
		{
			dev_err(&data->client->dev, "[touch]%s: failed to write data: %d\n",__func__, error);
			mutex_unlock(&data->lock);
			return error;
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
	}

	return 0;
}

static void raydium_work_handler(struct work_struct *work)
{
	struct raydium_ts_data *raydium_ts =  \
			container_of(work, struct raydium_ts_data, work);
	int ret = 0;
    unsigned int palm_status = 0;
	int i = 0;
	static unsigned char pre_palm_status = 0;

	if (raydium_ts->blank == FB_BLANK_UNBLANK) //when display on can use palm to suspend
	{
		palm_status = raydium_get_palm_state(raydium_ts);		
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
				pre_palm_status = RAYDIUM_PALM_MODE_ENABLE;
				raydium_ts->is_palm = 1;
				goto exit;
			}
		} else if ((palm_status == RAYDIUM_PALM_MODE_DISABLE) && (pre_palm_status == RAYDIUM_PALM_MODE_ENABLE)) 
		{
			printk(KERN_INFO "[touch]leave palm mode.\n");
			input_report_key(raydium_ts->input_dev, KEY_SLEEP, false); //release sleep key
			input_sync(raydium_ts->input_dev);
			
			pre_palm_status = RAYDIUM_PALM_MODE_DISABLE;
			raydium_ts->is_palm = 0;
			goto exit;
		}
	} else 
	{
		palm_status = raydium_get_palm_state(raydium_ts);
		if (palm_status == RAYDIUM_PALM_MODE_ENABLE) 
		{
			printk(KERN_INFO "[touch]palm detected in display idle or off\n");
			goto exit;
		}
	}
	
	ret = raydium_read_touchdata(raydium_ts);
	
exit:		
	enable_irq(raydium_ts->irq);
}

/*The raydium device will signal the host about TRIGGER_FALLING.
 *Processed when the interrupt is asserted.
 */
static irqreturn_t raydium_ts_interrupt(int irq, void *dev_id)
{
	struct raydium_ts_data *raydium_ts = dev_id;
	bool result = false;
	
	disable_irq_nosync(raydium_ts->irq);

    /*
	if (raydium_ts->is_suspend && g_uc_gesture_status != RAYDIUM_GESTURE_ENABLE) 
	{
		dev_err(&raydium_ts->client->dev, "[touch]%s: is_suspend = %d\n",
				__func__, raydium_ts->is_suspend);
		return IRQ_HANDLED;
	}
      */
	  g_uc_raydium_int_flag = 1;
	//For bootloader wrt/erase flash and software reset interrupt
	if((g_uc_raydium_flag & RAYDIUM_BOOTLOADER_FLAG) != 0) 
	{
		g_uc_raydium_flag = RAYDIUM_INTERRUPT_FLAG;
	} else 
	{
		if (!work_pending(&raydium_ts->work)) 
		{
			result = queue_work(raydium_ts->workqueue, &raydium_ts->work);
			if (result == false) //queue_work fail, enable_irq in fts_touch_irq_work() not work
			{ 
				enable_irq(raydium_ts->client->irq);
			}
		} else //pending re-enable irq
		{
			enable_irq(raydium_ts->irq);
		}
	}

	return IRQ_HANDLED;
}

// Read IC version
static int raydium_get_version(struct raydium_ts_data *raydium_ts)
{
	unsigned char temp_buf[4];
	int ret = -1;//, retry = 0;

	mutex_lock(&raydium_ts->lock);
	
	ret = raydium_i2c_pda2_set_page(raydium_ts->client, RAYDIUM_PDA2_PAGE_0);
	if (ret < 0)
	{
		goto exit_error;
	}
	
	*(unsigned long *)temp_buf = (RAYDIUM_I2C_PDA_MODE_ENABLE << 24) | ((RAYDIUM_IC_VERSION_CMD & (~MASK_8BIT)) >> 8);	//using byte mode to read 4 bytes
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
	
	ret = raydium_i2c_pda2_read(raydium_ts->client, (unsigned char)(RAYDIUM_IC_VERSION_CMD& MASK_8BIT), temp_buf, 4);
	if (ret < 0)
	{
		goto exit_error;
	}
	printk(KERN_ERR "[touch]Raydium Touch IC version:0x%02X%02X%02X%02X\n", temp_buf[3], temp_buf[2], temp_buf[1], temp_buf[0]);

exit_error:
	mutex_unlock(&raydium_ts->lock);
	return ret;
}

#if defined(CONFIG_PM)
static void raydium_ts_do_suspend(struct raydium_ts_data *ts)
{
	int i = 0;
	printk(KERN_INFO "[touch]%s, %d.\n", __func__, ts->is_suspend);
	if (ts->is_suspend == 0) 
	{
		ts->is_suspend = 1;

		mutex_lock(&ts->lock);
		raydium_get_gesture_state(ts);
		mutex_unlock(&ts->lock);
		
		/* release all touches */
		for (i = 0; i < ts->pdata->num_max_touches; i++) 
		{
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
		input_mt_report_pointer_emulation(ts->input_dev, false);
		input_sync(ts->input_dev);

		#ifndef GESTURE_EN
		disable_irq(ts->irq);
		#endif
	}

}
static int raydium_ts_do_resume(struct raydium_ts_data *ts)
{
	printk(KERN_INFO "[touch]%s, %d.\n", __func__, ts->is_suspend);
	if (ts->is_suspend) 
	{
		ts->is_suspend = 0;
        ts->is_sleep = 0;

		#ifndef GESTURE_EN
		enable_irq(ts->irq);
		#endif
	}
	return 0;
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
	.resume		= raydium_ts_resume,
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
	if (ts->is_sleep == 1)
	{	
		if((ts->blank == FB_BLANK_VSYNC_SUSPEND) || (ts->blank == FB_BLANK_UNBLANK))//only handle display is on case, avoid conflict with display driver
		{													
			disable_irq(ts->irq);	
			mutex_lock(&ts->lock);					
			if (gpio_is_valid(ts->rst)) 
			{
				udelay(RAYDIUM_POWERON_DELAY_USEC);//500us
			
				ret = gpio_request(ts->rst, "raydium_reset_gpio");
				if (ret) 
				{
					dev_err(&ts->client->dev, "[touch]reset gpio request failed");
				}
				gpio_direction_output(ts->rst, 1);
				gpio_direction_output(ts->rst, 0);
				msleep(RAYDIUM_RESET_INTERVAL_MSEC);//5ms
				gpio_direction_output(ts->rst, 1);
				msleep(RAYDIUM_RESET_DELAY_MSEC);//100ms
				gpio_free(ts->rst);
			}
			//turn on i2c pull-up power
			ret = raydium_power_on(ts, true);
			if (ret) 
			{
				dev_err(&ts->client->dev, "[touch]%s:power on failed",__func__);
			}
			mutex_unlock(&ts->lock);
			enable_irq(ts->irq);
			ts->is_sleep = 0;
			printk(KERN_INFO "[touch]disable touch lock.\n");
		}
	}
	
	return ret;															  
}

static void raydium_ts_close(struct input_dev *input_dev)
{
	struct raydium_ts_data *ts;												  
	int ret = 0;
	unsigned char wbuffer[1];
	
	printk(KERN_INFO "[touch]%s()+\n", __func__);
	ts = input_get_drvdata(input_dev);
	
	if (ts->is_sleep == 1)
	{
		printk(KERN_INFO "[touch]touch lock already enabled.\n");
		return;
	}
	
	disable_irq(ts->irq);
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
	enable_irq(ts->irq);
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
				raydium_ts_resume(&raydium_ts->client->dev);
				break;

			case FB_BLANK_POWERDOWN://screen off
				printk(KERN_INFO "[touch]FB_BLANK_POWERDOWN\n");
				raydium_ts_suspend(&raydium_ts->client->dev);
				break;
			case FB_BLANK_VSYNC_SUSPEND://ambient mode
				printk(KERN_INFO "[touch]FB_BLANK_VSYNC_SUSPEND\n");
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
/*******************************************************************************
*  Name: raydium_get_dt_coords
*  Brief:
*  Input:
*  Output:
*  Return: 
*******************************************************************************/
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

/*******************************************************************************
*  Name: raydium_parse_dt
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
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
	int err = 0;
	unsigned char i;
	input_dev->name = "raydium_ts";//name need same with .idc
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &ts->client->dev;
	input_dev->open = raydium_ts_open;//touch lock
	input_dev->close = raydium_ts_close;
	input_set_drvdata(input_dev, ts);
	
	//__set_bit(KEY_POWER, input_dev->keybit);
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
	
	err = input_mt_init_slots(input_dev, MAX_TOUCH_NUM,
		                        INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (err)
	{
		dev_err(&ts->client->dev, "[touch]failed to initialize MT slots: %d\n", err);
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
	int err = 0;

	if (client->dev.of_node) 
	{
		pdata = devm_kzalloc(&client->dev, sizeof(struct raydium_ts_platform_data), GFP_KERNEL);
		if (!pdata) 
		{
			dev_err(&client->dev, "[touch]failed to allocate memory\n");
			return -ENOMEM;
		}

		err = raydium_parse_dt(&client->dev, pdata);
		if (err) 
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
		err = -ENODEV;
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
	raydium_ts->rst = pdata->reset_gpio;
	raydium_ts->client = client;
	raydium_ts->pdata = pdata;
	raydium_ts->x_max = pdata->x_max - 1;	
	raydium_ts->y_max = pdata->y_max - 1;	
    raydium_ts->is_suspend = 0;
    raydium_ts->is_sleep = 0;
	raydium_ts->is_palm = 0;
	
	err = raydium_power_init(raydium_ts, true);
	if (err) 
	{
		dev_err(&client->dev, "[touch]raydium_power_init failed\n");
		goto exit_regulator_failed;
	}

	err = raydium_power_on(raydium_ts, true);
	if (err) 
	{
		dev_err(&client->dev, "[touch]raydium_power_on failed\n");
		goto pwr_deinit;
	}
     
    #ifdef MSM_NEW_VER
	err = raydium_ts_pinctrl_init(raydium_ts);
	if (!err && raydium_ts->ts_pinctrl) 
	{
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(raydium_ts->ts_pinctrl,
					raydium_ts->pinctrl_state_active);
		if (err < 0) 
		{
			dev_err(&client->dev, "[touch]failed to select pin to active state\n");
		}
	}
	#endif //end of MSM_NEW_VER
	
	err = raydium_gpio_configure(raydium_ts, true);
	if (err < 0) 
	{
		dev_err(&client->dev,"[touch]failed to configure the gpios\n");
		goto err_gpio_req;
	}
	
	msleep(raydium_ts->pdata->soft_rst_dly);//modify dtsi to 360

    //print touch ic version
	err = raydium_get_version(raydium_ts);
	if (err < 0) 
	{
		dev_err(&client->dev, "[touch]Read IC version failed\n");
		err = -ENODEV;
		goto exit_get_version;
	}

    //input device initialization
	input_dev = input_allocate_device();
	if (!input_dev) 
	{
		err = -ENOMEM;
		dev_err(&client->dev, "[touch]failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	raydium_ts->input_dev = input_dev;
	raydium_input_set(input_dev, raydium_ts);

	err = input_register_device(input_dev);
	if (err) 
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

	err = request_irq(raydium_ts->irq, raydium_ts_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->dev.driver->name,
					raydium_ts);

	if (err < 0) 
	{
		dev_err(&client->dev, "[touch]raydium_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	#ifdef GESTURE_EN
	enable_irq(raydium_ts->irq);
	#endif
	
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
exit_get_version:	
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
			err = pinctrl_select_state(raydium_ts->ts_pinctrl,
					raydium_ts->pinctrl_state_release);
			if (err) 
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
	return err;

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
	gpio_free(raydium_ts->irq);

#ifdef CONFIG_RM_SYSFS_DEBUG
	raydium_release_sysfs(client);
#endif //end of CONFIG_RM_SYSFS_DEBUG

	free_irq(client->irq, NULL);
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
		.pm	= &raydium_ts_pm_ops,
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
