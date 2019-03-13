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
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/kprobes.h>
#include <asm/traps.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>
#include "raydium_driver.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif /*end of CONFIG_FB*/



struct raydium_slot_status {
	unsigned char pt_id;      /*Occupied point ID*/
	unsigned char need_update;         /*Mark as info need to be updated*/
	unsigned char pt_report_offset;    /*point info offset in report*/
};
/*The first 3 elements are currently occupied. therest is new coming points*/
struct raydium_slot_status gst_slot[MAX_TOUCH_NUM * 2];
struct raydium_slot_status gst_slot_init = {0xFF, 0, 0};

#if (defined(CONFIG_RM_SYSFS_DEBUG))
const struct attribute_group raydium_attr_group;
#endif /*end of CONFIG_RM_SYSFS_DEBUG*/

unsigned char g_u8_addr;
unsigned char g_u8_raydium_flag;
unsigned char g_u8_i2c_mode;
unsigned char g_u8_upgrade_type;
unsigned char g_u8_raw_data_type;
unsigned int g_u32_raw_data_len;    /* 72 bytes*/
unsigned long g_u32_addr;
unsigned int g_u32_length;
unsigned char *g_rad_fw_image, *g_rad_init_image;
unsigned char *g_rad_boot_image, *g_rad_para_image;
unsigned char *g_rad_testfw_image, *g_rad_testpara_image;
unsigned char g_u8_table_setting;

/*******************************************************************************
*  Name: raydium_variable_init
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/
static void raydium_variable_init(void)
{
	g_u8_addr = RAYDIUM_PDA2_PDA_CFG_ADDR;
	g_u8_raydium_flag = 0;
	g_u8_i2c_mode = PDA2_MODE;
	g_u8_upgrade_type = 0;
	g_u8_raw_data_type = RAYDIUM_FT_UPDATE;
	g_u32_raw_data_len = 36 * 2;    /* 72 bytes*/
	g_u32_addr = RAD_CHK_I2C_CMD;
	g_u32_length = 1;
	g_u8_table_setting = 0;
	g_rad_fw_image = NULL;
	g_rad_init_image = NULL;
	g_rad_boot_image = NULL;
	g_rad_para_image = NULL;
	g_rad_testfw_image = NULL;
	g_rad_testpara_image = NULL;
}


/*******************************************************************************
*  Name: raydium_gpio_configure
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/

static int raydium_gpio_configure(struct raydium_ts_data *ts, bool on)
{
	int i32_err = 0;

	if (on) {
		if (gpio_is_valid(ts->irq_gpio)) {
			i32_err = gpio_request(ts->irq_gpio,
					   "raydium_irq_gpio");
			if (i32_err) {
				dev_err(&ts->client->dev,
					"[touch]irq gpio request failed");
				goto err_irq_gpio_req;
			}

			i32_err = gpio_direction_input(ts->irq_gpio);
			if (i32_err) {
				dev_err(&ts->client->dev,
					"[touch]set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}
		return 0;
	} else {
		if (gpio_is_valid(ts->irq_gpio))
			gpio_free(ts->irq_gpio);
		return 0;
	}

err_irq_gpio_dir:
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
err_irq_gpio_req:
	return i32_err;
}

/*******************************************************************************
*  Name: raydium_ts_pinctrl_init
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/
#ifdef MSM_NEW_VER
static int raydium_ts_pinctrl_init(struct raydium_ts_data *ts)
{
	int i32_ret;

	/* Get pinctrl if target uses pinctrl */
	ts->ts_pinctrl = devm_pinctrl_get(&(ts->client->dev));
	if (IS_ERR_OR_NULL(ts->ts_pinctrl)) {
		i32_ret = PTR_ERR(ts->ts_pinctrl);
		pr_err("[touch]target does not use pinctrl %d\n", i32_ret);
		goto err_pinctrl_get;
	}

	ts->pinctrl_state_active
		= pinctrl_lookup_state(ts->ts_pinctrl, PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
		i32_ret = PTR_ERR(ts->pinctrl_state_active);
		pr_err("[touch]Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, i32_ret);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_state_suspend
		= pinctrl_lookup_state(ts->ts_pinctrl,
				       PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
		i32_ret = PTR_ERR(ts->pinctrl_state_suspend);
		pr_err("[touch]Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, i32_ret);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_state_release
		= pinctrl_lookup_state(ts->ts_pinctrl,
				       PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(ts->pinctrl_state_release)) {
		i32_ret = PTR_ERR(ts->pinctrl_state_release);
		pr_err("[touch]Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, i32_ret);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ts->ts_pinctrl);
err_pinctrl_get:
	ts->ts_pinctrl = NULL;
	return i32_ret;
}
#endif/*end of MSM_NEW_VER*/

int raydium_i2c_pda_set_address(struct raydium_ts_data *ts,
				       unsigned int u32_address,
				       unsigned char u8_mode)
{
	int i32_ret = 0;
	unsigned char u8_retry;
	unsigned char u8_buf[RAD_I2C_PDA_ADDRESS_LENGTH];
	struct i2c_client *i2c = ts->client;

	/*g_u8_i2c_mode = PDA_MODE;*/

	i2c->addr = RAYDIUM_I2C_EID;
	u8_buf[0] = (u32_address & 0x0000FF00) >> 8;
	u8_buf[1] = (u32_address & 0x00FF0000) >> 16;
	u8_buf[2] = (u32_address & 0xFF000000) >> 24;
	u8_buf[3] = u8_mode;

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		i32_ret = i2c_master_send(i2c, u8_buf,
					 RAD_I2C_PDA_ADDRESS_LENGTH);
		if (i32_ret != RAD_I2C_PDA_ADDRESS_LENGTH) {
			pr_err("[touch]%s: I2C retry %d\n",
				__func__, u8_retry + 1);
			usleep_range(500, 1500);
		} else {
			break;
		}
	}

	return (i32_ret == RAD_I2C_PDA_ADDRESS_LENGTH) ? i32_ret : -EIO;
}

/*device attribute raydium_i2c_pda2_mode used*/
int raydium_i2c_pda_read(struct i2c_client *client,
				unsigned int u32_addr, unsigned char *u8_r_data,
				unsigned short u16_length)
{
	int i32_ret;
	unsigned char u8_retry;
	unsigned char u8_mode = 0x00;
	unsigned char u8_buf;
	struct raydium_ts_data *ts = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = 1,
			.buf = &u8_buf,
		},
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_READ,
			.len = u16_length,
			.buf = u8_r_data,
		},
	};

	if (u16_length == 4)
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			RAD_I2C_PDA_2_MODE_DISABLE |
			RAD_I2C_PDA_MODE_WORD_MODE;
	else
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			RAD_I2C_PDA_2_MODE_DISABLE;

	u8_mode |= 0x03;

	u8_buf = u32_addr & MASK_8BIT;

	i32_ret = raydium_i2c_pda_set_address(ts, u32_addr, u8_mode);
	if (i32_ret != RAD_I2C_PDA_ADDRESS_LENGTH)
		goto exit;
	usleep_range(50, 80);

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(ts->client->adapter, msg, 2) == 2) {
			i32_ret = u16_length;
			break;
		}
		pr_err("%s: I2C retry %d\n", __func__, u8_retry + 1);
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		pr_err("%s: I2C read over retry limit\n", __func__);
		i32_ret = -EIO;
	}
exit:
	return i32_ret;
}

int raydium_i2c_pda_write(struct i2c_client *client,
			unsigned int u32_addr, unsigned char *u8_w_data,
			unsigned short u16_length)
{
	int i32_ret;
	unsigned char u8_retry;
	unsigned char u8_mode = 0x00;
	unsigned char u8_buf[MAX_WRITE_PACKET_SIZE + 1];
	struct raydium_ts_data *ts = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = u16_length + 1,
			.buf = u8_buf,
		},
	};

	if (u16_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;

	if (u16_length == 4)
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			RAD_I2C_PDA_2_MODE_DISABLE |
			RAD_I2C_PDA_MODE_WORD_MODE;
	else
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			RAD_I2C_PDA_2_MODE_DISABLE;

	u8_buf[0] = u32_addr & MASK_8BIT;
	memcpy(&u8_buf[1], u8_w_data, u16_length);

	i32_ret = raydium_i2c_pda_set_address(ts, u32_addr, u8_mode);
	if (i32_ret != RAD_I2C_PDA_ADDRESS_LENGTH)
		goto exit;
	usleep_range(50, 80);

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			i32_ret = u16_length;
			break;
		}
		pr_err("[touch]%s: I2C retry %d\n", __func__, u8_retry + 1);
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		pr_err("[touch]%s: I2C write over retry limit\n", __func__);
		i32_ret = -EIO;
	}
exit:
	return i32_ret;
}

int raydium_i2c_pda2_set_page(struct i2c_client *client,
					unsigned int is_suspend,
					unsigned char u8_page)
{
	int i32_ret = -1;
	unsigned char u8_retry;
	unsigned int u8_ret = (is_suspend) ? 10 : 2;
	unsigned char u8_buf[RAYDIUM_I2C_PDA2_PAGE_LENGTH];

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = RAYDIUM_I2C_PDA2_PAGE_LENGTH,
			.buf = u8_buf,
		},
	};

	u8_buf[0] = RAYDIUM_PDA2_PAGE_ADDR;
	u8_buf[1] = u8_page;
	for (; u8_ret > 0; u8_ret--) {
		for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
			if (i2c_transfer(client->adapter, msg, 1) == 1) {
				i32_ret = RAYDIUM_I2C_PDA2_PAGE_LENGTH;
				break;
			}
			usleep_range(500, 1500);
	}
		if (i32_ret == RAYDIUM_I2C_PDA2_PAGE_LENGTH)
			break;
		usleep_range(2000, 5000);
	}

	if (0 == u8_ret) {
		pr_err("[touch]%s: I2C write over retry limit\n", __func__);
		i32_ret = -EIO;
	}

	return i32_ret;
}

int raydium_i2c_pda2_read(struct i2c_client *client,
				 unsigned char u8_addr,
				 unsigned char *u8_r_data,
				 unsigned short u16_length)
{
	int i32_ret = -1;
	unsigned char u8_retry;
	struct raydium_ts_data *ts = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = 1,
			.buf = &u8_addr,
		},
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_READ,
			.len = u16_length,
			.buf = u8_r_data,
		},
	};
	g_u8_i2c_mode = PDA2_MODE;
	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(ts->client->adapter, msg, 2) == 2) {
			i32_ret = u16_length;
			break;
		}
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		pr_err("[touch]%s: I2C read over retry limit\n", __func__);
		i32_ret = -EIO;
	}

	return i32_ret;
}

int raydium_i2c_pda2_write(struct i2c_client *client,
				  unsigned char u8_addr,
				  unsigned char *u8_w_data,
				  unsigned short u16_length)
{
	int i32_ret = -1;
	unsigned char u8_retry;
	unsigned char u8_buf[MAX_WRITE_PACKET_SIZE + 1];

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = u16_length + 1,
			.buf = u8_buf,
		},
	};

	if (u16_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;
	g_u8_i2c_mode = PDA2_MODE;
	u8_buf[0] = u8_addr;
	memcpy(&u8_buf[1], u8_w_data, u16_length);

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			i32_ret = u16_length;
			break;
		}
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		pr_err("[touch]%s: I2C write over retry limit\n", __func__);
		i32_ret = -EIO;
	}

	return i32_ret;
}

void raydium_irq_control(struct raydium_ts_data *ts, bool enable)
{
	if (enable) {
		if (ts->irq_enabled) {
			/*mutex_unlock(&ts->lock);*/
			dev_info(&ts->client->dev,
				"[touch]Already enable irq\n");
			return;
		}

		/* Clear interrupts first */
		if (ts->blank != FB_BLANK_POWERDOWN) {
			mutex_lock(&ts->lock);
			if (raydium_i2c_pda2_set_page(ts->client,
							  ts->is_suspend,
						      RAYDIUM_PDA2_PAGE_0) < 0)
				pr_err("[touch]failed to set page %s\n",
					__func__);
			mutex_unlock(&ts->lock);
			usleep_range(500, 1500);
		}
		while (ts->irq_desc->depth > 0) {
			pr_info("[touch]irq enable\n");
			ts->irq_enabled = true;
			enable_irq(ts->irq);
		}
	} else {
		if (ts->irq_enabled) {
			if (ts->irq_desc->depth == 0) {
				disable_irq(ts->irq);
				ts->irq_enabled = false;
				pr_info("[touch]irq disable\n");
			}
		}
	}
}

#ifdef CONFIG_RM_SYSFS_DEBUG

int raydium_i2c_mode_control(struct i2c_client *client,
				    unsigned char u8_mode)
{
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);
	unsigned char u8_buf[4];

	switch (u8_mode) {
	case 0:	/* Disable INT flag */
		pr_info("[touch]RAD INT flag : %d\n", ts->irq_enabled);
		disable_irq(ts->irq);
		ts->irq_enabled = false;
		pr_info("[touch]RAD irq disable\n");
		break;
	case 1:	/* Enable INT flag */
		pr_info("[touch]RAD INT flag : %d\n", ts->irq_enabled);
		enable_irq(ts->irq);
		ts->irq_enabled = true;
		pr_info("[touch]RAD irq enable\n");
		break;
	case 2: /* Disable INT by raydium_irq_control */
		raydium_irq_control(ts, DISABLE);
		break;
	case 3: /* Enable INT by raydium_irq_control */
		raydium_irq_control(ts, ENABLE);
		break;
	case 4: /* Show RAD INT depth */
		pr_info("[touch]RAD INT depth : %d\n", ts->irq_desc->depth);
		break;
	case 7:
		raydium_i2c_pda2_set_page(client,
			ts->is_suspend , RAYDIUM_PDA2_2_PDA);
		g_u8_i2c_mode = PDA_MODE;
		pr_info("[touch]Disable PDA2_MODE;\n");
		break;
	case 8:
		raydium_i2c_pda_read(client, RAD_PDA2_CTRL_CMD, u8_buf, 4);
		u8_buf[0] |= RAD_ENABLE_PDA2 | RAD_ENABLE_SI2;
		raydium_i2c_pda_write(client, RAD_PDA2_CTRL_CMD, u8_buf, 4);
		raydium_i2c_pda_set_address(ts, 0x50000628, DISABLE);

		g_u8_i2c_mode = PDA2_MODE;
		pr_info("[touch]Enable PDA2_MODE;\n");
		break;
	}
	return 0;
}


unsigned char i2c_burst_read(struct i2c_client *client,
				    int i32_addr,
				    unsigned short u16_read_len,
				    unsigned char *p_u8_read_data)
{
	unsigned char u8_result = ERROR;
	unsigned short u16_already_read_len = 0;
	unsigned char u8_read = 0;
	unsigned char u8_rbuffer[4];
	int i32_ret = 0;

	if (u16_read_len > 4)
		u8_read = 4;
	else
		u8_read = u16_read_len;

	while (u16_already_read_len < u16_read_len) {

		i32_ret = raydium_i2c_pda_read(client,
					      i32_addr,
					      u8_rbuffer,
					      u8_read);
		if (i32_ret < 0)
			return u8_result;

		memcpy(p_u8_read_data + u16_already_read_len,
			u8_rbuffer,
			u8_read);

		u16_already_read_len += u8_read;
	}
	u8_result = SUCCESS;
	return u8_result;
}

unsigned char i2c_burst_write(struct i2c_client *client,
				     int i32_addr,
				     unsigned short u16_write_len,
				     unsigned char *u8_value)
{
	unsigned char u8_result = ERROR;
	int i32_ret = 0;
	unsigned short u16_len = 0;
	unsigned char u8_w_len = 4;

	while (u16_len < u16_write_len) {
		i32_ret = raydium_i2c_pda_write(client, i32_addr,
					    u8_value + u16_len,
					    u8_w_len);
		if (i32_ret < 0)
			return u8_result;
		u16_len += u8_w_len;
		if (u16_len + u8_w_len > u16_write_len)
			u8_w_len = u16_write_len - u16_len;
	}
	u8_result = SUCCESS;
	return u8_result;
}

void i2c_write(struct i2c_client *client,
		unsigned int u32_addr,
		unsigned int u32_data)
{
	unsigned char u8_data[4];
	u8_data[0] = (unsigned char)(u32_data);
	u8_data[1] = (unsigned char)(u32_data >> 8);
	u8_data[2] = (unsigned char)(u32_data >> 16);
	u8_data[3] = (unsigned char)(u32_data >> 24);
	i2c_burst_write(client, u32_addr, 4, u8_data);
}

const struct attribute_group raydium_attr_group = {
	.attrs = raydium_attributes
};

/*create sysfs for debug update firmware*/
static int raydium_create_sysfs(struct i2c_client *client)
{
	int ret = -1;

	ret = sysfs_create_group(&(client->dev.kobj), &raydium_attr_group);
	if (ret) {
		pr_err("[touch]failed to register sysfs\n");
		sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
		return -EIO;
	} else	{
		pr_info("[touch]create raydium sysfs attr_group successful\n");
	}
	return ret;
}

static void raydium_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
}
#endif /*end of CONFIG_RM_SYSFS_DEBUG*/

#ifdef FILTER_POINTS
int raydium_pointer_filter(struct raydium_ts_data *ts,
				int i32_index,
				unsigned short u16_diff_x,
				unsigned short u16_diff_y)
{
	if (abs(ts->x_pos[i32_index] - ts->last_x_pos[i32_index]) >  u16_diff_x)
		return 0;
	if (abs(ts->y_pos[i32_index] - ts->last_y_pos[i32_index]) >  u16_diff_y)
		return 0;

	return 1;
}
#endif
static int raydium_touch_report(struct raydium_ts_data *ts,
				unsigned char *p_u8_buf,
				unsigned char u8_points_amount)
{
	unsigned char u8_i, u8_j, u8_offset = 0, u8_pt_id;
	signed short i16_wx, i16_wy;
	/* number of touch points */
	unsigned char u8_touch_count = 0;
	DECLARE_BITMAP(ids, ts->u8_max_touchs);
	bitmap_zero(ids, ts->u8_max_touchs);

	for (u8_i = 0; u8_i < (ts->u8_max_touchs * 2); u8_i++) {
		gst_slot[u8_i].need_update = 0;
		gst_slot[u8_i].pt_report_offset = 0;
	}

	/*Check incoming point info*/
	for (u8_i = 0; u8_i < u8_points_amount; u8_i++) {
		u8_pt_id = p_u8_buf[POS_PT_ID + u8_i * LEN_PT];
		/* Current*/
		for (u8_j = 0; u8_j < ts->u8_max_touchs; u8_j++) {
			if (u8_pt_id == gst_slot[u8_j].pt_id) {
				gst_slot[u8_j].need_update = 1;
				gst_slot[u8_j].pt_report_offset = u8_i;
				break;
			}
		}
		/* New coming*/
		if (u8_j == ts->u8_max_touchs) {
			for (u8_j = ts->u8_max_touchs;
				u8_j < (ts->u8_max_touchs * 2); u8_j++) {
				if (!gst_slot[u8_j].need_update) {
					gst_slot[u8_j].pt_id = u8_pt_id;
					gst_slot[u8_j].need_update = 1;
					gst_slot[u8_j].pt_report_offset = u8_i;
					pr_info("[touch]x:%d,y:%d\n",
					p_u8_buf[POS_X_L + u8_offset] |
					p_u8_buf[POS_X_H + u8_offset] << 8,
					p_u8_buf[POS_Y_L + u8_offset] |
					p_u8_buf[POS_Y_H + u8_offset] << 8);
					break;
				}
			}
		}
	}

	/*Release slot with non-occupied point*/
	for (u8_i = 0; u8_i < ts->u8_max_touchs; u8_i++) {
		if (!gst_slot[u8_i].need_update) {
			input_mt_slot(ts->input_dev, u8_i);
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, false);
			gst_slot[u8_i].pt_id = 0xFF;
			gst_slot[u8_i].pt_report_offset = 0;
			gst_slot[u8_i].need_update = 0;
		}
	}
	/*Assign new one to non-occupied slot*/
	for (u8_i = ts->u8_max_touchs; u8_i < (ts->u8_max_touchs * 2); u8_i++) {
		if (gst_slot[u8_i].need_update) {
			for (u8_j = 0; u8_j < ts->u8_max_touchs; u8_j++) {
				if (!gst_slot[u8_j].need_update) {
					gst_slot[u8_j] = gst_slot[u8_i];
					gst_slot[u8_i] = gst_slot_init;
					break;
				}
			}
		} else {
			break;
		}
	}
#ifdef FILTER_POINTS
	for (u8_i = 0; u8_i < ts->u8_max_touchs; u8_i++) {
		if (gst_slot[u8_i].need_update) {
			u8_offset = gst_slot[u8_i].pt_report_offset * LEN_PT;
			ts->x_pos[i] =  p_u8_buf[POS_X_L + u8_offset] |
				p_u8_buf[POS_X_H + u8_offset] << BYTE_SHIFT;
			ts->y_pos[i] = p_u8_buf[POS_Y_L + u8_offset] |
				p_u8_buf[POS_Y_H + u8_offset] << BYTE_SHIFT;
			ts->pressure = p_u8_buf[POS_PRESSURE_L + u8_offset] |
			p_u8_buf[POS_PRESSURE_H + u8_offset] << BYTE_SHIFT;
			i16_wx = p_u8_buf[POS_WX_L + u8_offset] |
				p_u8_buf[POS_WX_H + u8_offset] << BYTE_SHIFT;
			i16_wy = p_u8_buf[POS_WY_L + u8_offset] |
				p_u8_buf[POS_WY_H + u8_offset] << BYTE_SHIFT;

		if (!raydium_pointer_filter(ts, u8_i, DELTA_X, DELTA_Y)) {
			input_mt_slot(ts->input_dev, u8_i);
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, true);
			__set_bit(i, ids);

			input_report_abs(ts->input_dev,
				ABS_MT_POSITION_X, ts->x_pos[u8_i]);
			input_report_abs(ts->input_dev,
				ABS_MT_POSITION_Y, ts->y_pos[u8_i]);
			input_report_abs(ts->input_dev,
				ABS_MT_PRESSURE, ts->pressure);
			input_report_abs(ts->input_dev,
				ABS_MT_TOUCH_MAJOR, max(i16_wx, i16_wy));
			input_report_abs(ts->input_dev,
				ABS_MT_TOUCH_MINOR, min(i16_wx, i16_wy));
			input_report_key(ts->input_dev,
				BTN_TOUCH, 1);
			input_report_key(ts->input_dev,
				BTN_TOOL_FINGER, 1);
			input_sync(ts->input_dev);
			ts->last_x_pos[u8_i] = ts->x_pos[u8_i];
			ts->last_y_pos[u8_i] = ts->y_pos[u8_i];
		} else {
			ts->x_pos[u8_i] = ts->last_x_pos[u8_i];
			ts->y_pos[u8_i] = ts->last_y_pos[u8_i];
		}
		}
	}

	if ((gst_slot[0].need_update == 0) &&
		(gst_slot[1].need_update == 0)) {
		for (u8_i = 0; u8_i < ts->u8_max_touchs; u8_i++) {
			if (test_bit(u8_i, ids))
				continue;
			input_mt_slot(ts->input_dev, u8_i);
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, false);
		}
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
		input_sync(ts->input_dev);
	}
#else
	for (u8_i = 0; u8_i < ts->u8_max_touchs; u8_i++) {
		if (gst_slot[u8_i].need_update) {
			u8_offset = gst_slot[u8_i].pt_report_offset * LEN_PT;
			ts->x_pos[u8_i] = p_u8_buf[POS_X_L + u8_offset] |
				p_u8_buf[POS_X_H + u8_offset] << BYTE_SHIFT;
			ts->y_pos[u8_i] = p_u8_buf[POS_Y_L + u8_offset] |
				p_u8_buf[POS_Y_H + u8_offset] << BYTE_SHIFT;
			ts->pressure = p_u8_buf[POS_PRESSURE_L + u8_offset] |
			p_u8_buf[POS_PRESSURE_H + u8_offset] << BYTE_SHIFT;
			i16_wx = p_u8_buf[POS_WX_L + u8_offset] |
				p_u8_buf[POS_WX_H + u8_offset] << BYTE_SHIFT;
			i16_wy = p_u8_buf[POS_WY_L + u8_offset] |
				p_u8_buf[POS_WY_H + u8_offset] << BYTE_SHIFT;

			input_mt_slot(ts->input_dev, u8_i);
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, true);
			__set_bit(u8_i, ids);

			input_report_abs(ts->input_dev,
				ABS_MT_POSITION_X, ts->x_pos[u8_i]);
			input_report_abs(ts->input_dev,
				ABS_MT_POSITION_Y, ts->y_pos[u8_i]);
			input_report_abs(ts->input_dev,
				ABS_MT_PRESSURE, ts->pressure);
			input_report_abs(ts->input_dev,
				ABS_MT_TOUCH_MAJOR, max(i16_wx, i16_wy));
			input_report_abs(ts->input_dev,
				ABS_MT_TOUCH_MINOR, min(i16_wx, i16_wy));

			input_report_key(ts->input_dev,
				BTN_TOUCH, 1);
			input_report_key(ts->input_dev,
				BTN_TOOL_FINGER, 1);
			input_sync(ts->input_dev);

			u8_touch_count++;
		}
	}

	for (u8_i = 0; u8_i < ts->u8_max_touchs; u8_i++) {
		if (test_bit(u8_i, ids))
			continue;
		input_mt_slot(ts->input_dev, u8_i);
		input_mt_report_slot_state(ts->input_dev,
			MT_TOOL_FINGER, false);
		input_sync(ts->input_dev);
	}
#endif
	return 0;
}
int raydium_read_touchdata(struct raydium_ts_data *ts,
	unsigned char *p_u8_tp_status,  unsigned char *p_u8_buf)
{
	int i32_ret = 0;
	unsigned char u8_points_amount;
	static unsigned char u8_seq_no;

	mutex_lock(&ts->lock);
	i32_ret = raydium_i2c_pda2_set_page(ts->client,
		ts->is_suspend, RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0) {
		pr_err("[touch]%s: set page failed, %d\n", __func__, i32_ret);
		mutex_unlock(&ts->lock);
		return i32_ret;
	}

	memset(p_u8_buf, 0, MAX_REPORT_PACKET_SIZE);
	memset(p_u8_tp_status, 0, MAX_TCH_STATUS_PACKET_SIZE);

	/*read touch point information*/
	i32_ret = raydium_i2c_pda2_read(ts->client,
			RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR,
			p_u8_tp_status, MAX_TCH_STATUS_PACKET_SIZE);
	if (i32_ret < 0) {
		pr_err("[touch]%s: failed to read data: %d\n",
			__func__, __LINE__);
		goto exit_error;
	}

	/* inform IC to prepare next report*/
	if (u8_seq_no == p_u8_tp_status[POS_SEQ]) {
		pr_err("%s -> report not updated.\n", __func__);
		goto exit_error;
	}
	u8_points_amount = p_u8_tp_status[POS_PT_AMOUNT];

	if (u8_points_amount > MAX_TOUCH_NUM)
		goto exit_error;

	/*read touch point report*/
	/*PDA2 only support word mode*/
	if (u8_points_amount != 0)
		i32_ret = raydium_i2c_pda2_read(ts->client,
				RAYDIUM_PDA2_TCH_RPT_ADDR, p_u8_buf,
				u8_points_amount * LEN_PT);

	if (i32_ret < 0) {
		pr_err("[touch]%s: failed to read data: %d\n",
			__func__, __LINE__);
		goto exit_error;
	}

	u8_seq_no = p_u8_tp_status[POS_SEQ];
	p_u8_tp_status[POS_SEQ] = 0;
	i32_ret = raydium_i2c_pda2_write(ts->client,
			RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, p_u8_tp_status, 1);
	if (i32_ret < 0) {
		pr_err("[touch]%s: write data failed: %d\n", __func__, i32_ret);
		goto exit_error;
	}

	raydium_touch_report(ts, p_u8_buf, u8_points_amount);

exit_error:
	mutex_unlock(&ts->lock);

	return i32_ret;
}

static void raydium_work_handler(struct work_struct *work)
{

	struct raydium_ts_data *ts =
		container_of(work, struct raydium_ts_data, work);
	int i32_ret = 0;
	unsigned char u8_tp_status[MAX_TCH_STATUS_PACKET_SIZE];
	unsigned char u8_buf[MAX_REPORT_PACKET_SIZE];
	unsigned char u8_i;

	if (g_u8_i2c_mode == PDA2_MODE) {
		i32_ret = raydium_read_touchdata(ts, u8_tp_status, u8_buf);
		if (i32_ret < 0) {
			pr_info("[touch]%s, read_touchdata error, ret:%d\n",
				__func__, i32_ret);
		}
	}

#ifdef GESTURE_EN
	/*when display on can use palm to suspend*/
	if (ts->blank == FB_BLANK_UNBLANK) {
		if (u8_tp_status[POS_GES_STATUS] == RAD_PALM_ENABLE) {
			if (ts->is_palm == 0) {
				/* release all touches*/
				for (u8_i = 0; u8_i < ts->u8_max_touchs;
						u8_i++) {
					input_mt_slot(ts->input_dev, u8_i);
				input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, false);
				}
				input_mt_report_pointer_emulation(ts->input_dev,
					false);
				/*press sleep key*/
				input_report_key(ts->input_dev,
					KEY_SLEEP, true);
				input_sync(ts->input_dev);

				pr_info("[touch]palm_status = %d.\n",
					 u8_tp_status[POS_GES_STATUS]);

				ts->is_palm = 1;
				/*goto exit;*/
			}
		} else if ((u8_tp_status[POS_GES_STATUS] == RAD_PALM_DISABLE)
			&& (ts->is_palm == 1)) {
			pr_info("[touch]leave palm mode.\n");
			input_report_key(ts->input_dev, KEY_SLEEP, false);
			input_sync(ts->input_dev);

			/*raydium_irq_control(raydium_ts, DISABLE);*/
			ts->is_palm = 0;
			/*goto exit;*/
		}
	} else if (ts->blank == FB_BLANK_VSYNC_SUSPEND ||
			ts->blank == FB_BLANK_POWERDOWN) {
		/*need check small area*/
		if (u8_tp_status[POS_GES_STATUS] == RAD_WAKE_UP) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 100);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 100);
			input_sync(ts->input_dev);
			usleep_range(9500, 10500);
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, 0);
			input_mt_report_pointer_emulation(ts->input_dev, false);
			input_sync(ts->input_dev);
			pr_info("[touch]display wake up\n");
			/*goto exit;*/
		}
	}
#endif

	return;
}

/*The raydium device will signal the host about TRIGGER_FALLING.
 *Processed when the interrupt is asserted.
 */
static irqreturn_t raydium_ts_interrupt(int irq, void *dev_id)
{
	struct raydium_ts_data *ts = dev_id;
	bool result = false;

	/*For bootloader wrt/erase flash and software reset interrupt*/
	if ((g_u8_raydium_flag & ENG_MODE) != 0) {
		disable_irq_nosync(ts->irq);
		ts->irq_enabled = false;
		pr_info("[touch]RAD_ENG_MODE\n");
		g_u8_raydium_flag |= INT_FLAG;
	} else {
		if (!work_pending(&ts->work)) {
			/* Clear interrupts*/
			mutex_lock(&ts->lock);
			if (raydium_i2c_pda2_set_page(ts->client,
				ts->is_suspend, RAYDIUM_PDA2_PAGE_0) < 0) {
				pr_err("[touch]%s: failed to set page ontime\n",
					__func__);
				mutex_unlock(&ts->lock);
				return IRQ_HANDLED;
			}
			mutex_unlock(&ts->lock);
			result = queue_work(ts->workqueue,
					&ts->work);

			if (result == false) {
				/*queue_work fail*/
				pr_err("[touch]queue_work fail.\n");
			} else {
				if (ts->blank == FB_BLANK_POWERDOWN) {
					pr_info("[touch]%s, disable depth : %u\n",
					__func__,
					 ts->irq_desc->depth);

					disable_irq_nosync(ts->irq);
					ts->irq_enabled = false;
					pr_info("[touch]FB_BLANK_POWERDOWN disable irq\n");
				}
			}


		} else {
			/*work pending*/
				mutex_lock(&ts->lock);
				if (raydium_i2c_pda2_set_page(ts->client,
					ts->is_suspend,
					RAYDIUM_PDA2_PAGE_0) < 0) {

					pr_err("[touch]%s: failed to set page in work_pending\n",
					__func__);
				}
				mutex_unlock(&ts->lock);

			pr_info("[touch]work_pending\n");
		}
	}
	return IRQ_HANDLED;
}

static int raydium_check_i2c_ready(struct raydium_ts_data *ts,
			unsigned short *u16_i2c_data)
{
	unsigned char u8_buf[4];
	int i32_ret = -1;

	mutex_lock(&ts->lock);

	if (g_u8_i2c_mode == PDA2_MODE) {
		i32_ret = raydium_i2c_pda2_set_page(ts->client,
				ts->is_suspend,
				RAYDIUM_PDA2_PAGE_0);
		if (i32_ret < 0)
			goto exit_error;

		*(unsigned int *)u8_buf =
			(RAD_I2C_PDA_MODE_ENABLE << 24) |
			((RAD_CHK_I2C_CMD & (~MASK_8BIT)) >> 8);
		/*using byte mode to read 4 bytes*/
		i32_ret = raydium_i2c_pda2_write(ts->client,
					     RAYDIUM_PDA2_PDA_CFG_ADDR,
					     u8_buf,
					     4);
		if (i32_ret < 0)
			goto exit_error;

		i32_ret = raydium_i2c_pda2_set_page(ts->client,
						ts->is_suspend,
						RAYDIUM_PDA2_ENABLE_PDA);
		if (i32_ret < 0)
			goto exit_error;

		i32_ret = raydium_i2c_pda2_read(
				ts->client,
				(unsigned char)(RAD_CHK_I2C_CMD & MASK_8BIT),
				u8_buf, 4);
		if (i32_ret < 0)
			goto exit_error;

	} else {
		i32_ret = raydium_i2c_pda_read(ts->client,
					   RAD_CHK_I2C_CMD, u8_buf,
					   4);
		if (i32_ret < 0)
			goto exit_error;

	}

	*u16_i2c_data = u8_buf[3] << 8 | u8_buf[2];

	pr_info("[touch]RAD check I2C : 0x%02X%02X\n", u8_buf[3], u8_buf[2]);

exit_error:
	mutex_unlock(&ts->lock);
	return i32_ret;
}

#if defined(CONFIG_PM)
static void raydium_ts_do_suspend(struct raydium_ts_data *ts)
{
	unsigned char u8_i = 0;

	if (ts->is_suspend == 1) {
		pr_info("[touch]Already in suspend state\n");
		return;
	}

	/*#ifndef GESTURE_EN*/
	raydium_irq_control(ts, DISABLE);
	/*#endif*/

	/*clear workqueue*/
	if (!cancel_work_sync(&ts->work))
		pr_info("[touch]workqueue is empty!\n");

	pr_info("[touch]%s.\n", __func__);

	/* release all touches */
	for (u8_i = 0; u8_i < ts->u8_max_touchs; u8_i++) {
		input_mt_slot(ts->input_dev, u8_i);
		input_mt_report_slot_state(ts->input_dev,
					   MT_TOOL_FINGER,
					   false);
	}
	input_mt_report_pointer_emulation(ts->input_dev, false);
	input_sync(ts->input_dev);

#ifdef GESTURE_EN
	if (device_may_wakeup(&ts->client->dev)) {
		pr_info("[touch]Device may wakeup\n");
		if (!enable_irq_wake(ts->irq))
			ts->irq_wake = 1;

	} else
		pr_info("[touch]Device not wakeup\n");
	raydium_irq_control(ts, ENABLE);
#endif

	ts->is_suspend = 1;
}

static void raydium_ts_do_resume(struct raydium_ts_data *ts)
{
	pr_info("[touch]%s, %d.\n", __func__, ts->is_suspend);
	if (ts->is_suspend == 0) {
		pr_info("[touch]Already in resume state\n");
		return;
	}

	/* clear interrupts*/
	mutex_lock(&ts->lock);
	if (raydium_i2c_pda2_set_page(ts->client,
		ts->is_suspend, RAYDIUM_PDA2_PAGE_0) < 0) {
		pr_err("[ raydium ]%s: failed to set page\n", __func__);
		mutex_unlock(&ts->lock);
		return;
	}
	mutex_unlock(&ts->lock);

	/* clear workqueue*/
	if (!cancel_work_sync(&ts->work))
		pr_info("[ raydium ]workqueue is empty!\n");

	raydium_irq_control(ts, ENABLE);

#ifdef GESTURE_EN
	if (device_may_wakeup(&ts->client->dev)) {
		pr_info("[touch]Device may wakeup\n");
		if (ts->irq_wake) {
			disable_irq_wake(ts->irq);
			ts->irq_wake = 0;
		}
	} else
		pr_info("[touch]Device not wakeup\n");
#endif

	ts->is_suspend = 0;
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
#endif /*end of CONFIG_PM*/
};

/*used for touch lock feature*/
static int raydium_ts_open(struct input_dev *input_dev)
{
	int i32_ret = 0;
	struct raydium_ts_data *ts;

	ts = input_get_drvdata(input_dev);

	pr_info("[touch]%s()+\n", __func__);

	pr_info("[touch]ts->blank:%x\n", ts->blank);

	if (ts->is_sleep == 1) {
		mutex_lock(&ts->lock);
		if (gpio_is_valid(ts->rst_gpio)) {

			i32_ret = gpio_request(ts->rst_gpio,
				"raydium_reset_gpio");
			if (i32_ret < 0)
				pr_err("[touch]reset gpio request failed");

			gpio_direction_output(ts->rst_gpio, 1);
			gpio_direction_output(ts->rst_gpio, 0);
			msleep(RAYDIUM_RESET_INTERVAL_MSEC);
			gpio_direction_output(ts->rst_gpio, 1);
			msleep(RAYDIUM_RESET_DELAY_MSEC);
			g_u8_i2c_mode = PDA2_MODE;
			gpio_free(ts->rst_gpio);
		}
		mutex_unlock(&ts->lock);
		raydium_irq_control(ts, ENABLE);
		ts->is_sleep = 0;
		pr_info("[touch]disable touch lock.\n");
	}

	return i32_ret;
}

static void raydium_ts_close(struct input_dev *input_dev)
{
	struct raydium_ts_data *ts;
	int i32_ret = 0;
	unsigned char u8_i = 0;
	unsigned char u8_wbuffer[1];
	unsigned char buf[4];

	ts = input_get_drvdata(input_dev);
	pr_info("[touch]%s()+\n", __func__);

	if (ts->is_sleep == 1) {
		pr_info("[touch]touch lock already enabled.\n");
		return;
	}

	raydium_irq_control(ts, DISABLE);

	for (u8_i = 0; u8_i < ts->u8_max_touchs; u8_i++) {
		input_mt_slot(ts->input_dev, u8_i);
		input_mt_report_slot_state(ts->input_dev,
			MT_TOOL_FINGER,
			false);
	}
	input_mt_report_pointer_emulation(ts->input_dev, false);
	input_sync(ts->input_dev);
	mutex_lock(&ts->lock);
	i32_ret = raydium_i2c_pda2_set_page(ts->client,
		ts->is_suspend, RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0) {
		pr_err("[touch]ret:%d\n", i32_ret);
		goto exit_i2c_error;
	}
	u8_wbuffer[0] = RAYDIUM_HOST_CMD_PWR_SLEEP;
	i32_ret = raydium_i2c_pda2_write(ts->client,
		RAYDIUM_PDA2_HOST_CMD_ADDR,
		u8_wbuffer,
		1);
	if (i32_ret < 0) {
		pr_err("[touch]ret:%d\n", i32_ret);
		goto exit_i2c_error;
	}

	msleep(750);
	memset(buf, 0, sizeof(buf));
	i32_ret = raydium_i2c_pda_write(ts->client, 0x5000000C, buf, 4);
	if (i32_ret < 0) {
		pr_err("[touch]Disable M0 NG ret:%d\n", i32_ret);
		goto exit_i2c_error;
	}

	mutex_unlock(&ts->lock);
	ts->is_sleep = 1;
	pr_info("[touch]enable touch lock.\n");
	return;

exit_i2c_error:
	mutex_unlock(&ts->lock);
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
#endif /*end of CONFIG_FB*/

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event,
				void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct raydium_ts_data *ts =
		container_of(self, struct raydium_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    ts && ts->client) {
		blank = evdata->data;
		ts->blank = (*blank);

		switch (*blank) {

		/*screen on*/
		case FB_BLANK_UNBLANK:
			pr_info("[touch]FB_BLANK_UNBLANK\n");
#ifdef GESTURE_EN

			/* clear palm status */

			ts->is_palm = 0;
#endif
			raydium_ts_resume(&ts->client->dev);
			break;

		/*screen off*/
		case FB_BLANK_POWERDOWN:
			pr_info("[touch]FB_BLANK_POWERDOWN\n");
#ifdef GESTURE_EN

			/* clear palm status */

			ts->is_palm = 0;
#endif
			raydium_ts_suspend(&ts->client->dev);
			break;

		/*ambient mode*/
		case FB_BLANK_VSYNC_SUSPEND:
			pr_info("[touch]FB_BLANK_VSYNC_SUSPEND\n");
#ifdef GESTURE_EN

			/* clear palm status */

			ts->is_palm = 0;
#endif
			raydium_ts_suspend(&ts->client->dev);
			break;

		default:
			break;
		}
	}

	return 0;
}

static void raydium_register_notifier(struct raydium_ts_data *ts)
{
	memset(&ts->fb_notif, 0, sizeof(ts->fb_notif));
	ts->fb_notif.notifier_call = fb_notifier_callback;

	/* register on the fb notifier and work with fb*/
	if (fb_register_client(&ts->fb_notif))
		pr_err("[touch]register notifier failed\n");
}

static void raydium_unregister_notifier(struct raydium_ts_data *ts)
{
	fb_unregister_client(&ts->fb_notif);
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void raydium_ts_early_suspend(struct early_suspend *handler)
{
	struct raydium_ts_data *ts = container_of(handler,
					     struct raydium_ts_data,
					     early_suspend);

	raydium_ts_do_suspend(ts);
}

static void raydium_ts_late_resume(struct early_suspend *handler)
{
	struct raydium_ts_data *ts = container_of(handler,
					     struct raydium_ts_data,
					     early_suspend);

	raydium_ts_do_resume(ts);
}
#endif /*end of CONFIG_FB*/

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
		return -EINVAL;

	if (!prop->value)
		return -ENODATA;


	coords_size = prop->length / sizeof(u32);
	if (coords_size != COORDS_ARR_SIZE) {
		pr_err("[touch]invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		pr_err("[touch]unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "raydium,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		pr_err("[touch]unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int raydium_parse_dt(struct device *dev,
			    struct raydium_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int rc = 0;
	u32 temp_val = 0;

	pdata->name = RAYDIUM_NAME;

	rc = raydium_get_dt_coords(dev, "raydium,display-coords", pdata);
	if (rc)
		return rc;


	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np,
			    "raydium,reset-gpio",
			    0,
			    &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;


	pdata->irq_gpio = of_get_named_gpio_flags(np,
			  "raydium,irq-gpio",
			  0,
			  &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;


	rc = of_property_read_u32(np,
			"raydium,hard-reset-delay-ms", &temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;


	rc = of_property_read_u32(np,
			"raydium,soft-reset-delay-ms", &temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;


	rc = of_property_read_u32(np, "raydium,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;


	return 0;
}
#else
static int raydium_parse_dt(struct device *dev,
			    struct raydium_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif /*end of CONFIG_OF*/

static void raydium_input_set(struct input_dev *input_dev,
			      struct raydium_ts_data *ts)
{
	int ret = 0;
	unsigned char i;
	input_dev->name = "raydium_ts";/*name need same with .idc*/
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &ts->client->dev;
	input_dev->open = raydium_ts_open;/*touch lock*/
	input_dev->close = raydium_ts_close;
	input_set_drvdata(input_dev, ts);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	pr_info("[touch]set abs prarams x[%d], y[%d]\n", ts->x_max, ts->y_max);

	/* Multitouch input params setup */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TOUCH_MAJOR, 0, WIDTH_MAX, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TOUCH_MINOR, 0, WIDTH_MAX, 0, 0);

	ret = input_mt_init_slots(input_dev, MAX_TOUCH_NUM,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		pr_err("[touch]failed to initialize MT slots: %d\n", ret);

	for (i = 0; i < (MAX_TOUCH_NUM * 2); i++)
		gst_slot[i] = gst_slot_init;

}
static int raydium_set_resolution(struct raydium_ts_data *ts)
{
	unsigned char u8_buf[4];
	int i32_ret = -1;
	unsigned int u32_x, u32_y;

	mutex_lock(&ts->lock);

	i32_ret = raydium_i2c_pda2_set_page(ts->client,
			ts->is_suspend,
			RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0)
		goto exit_error;

	i32_ret = raydium_i2c_pda2_read(ts->client,
				RAYDIUM_PDA2_DISPLAY_INFO_ADDR,
				u8_buf, 4);
	if (i32_ret < 0)
		goto exit_error;

	u32_x = u8_buf[3] << 8 | u8_buf[2];
	u32_y = u8_buf[1] << 8 | u8_buf[0];

	pr_info("[touch]RAD display info x:%d, y:%d\n", u32_x, u32_y);

	if (u32_x > 300 && u32_y > 300 &&
			u32_x < 500 && u32_y < 500) {
		ts->x_max = u32_x - 1;
		ts->y_max = u32_y - 1;
	}

exit_error:
	mutex_unlock(&ts->lock);
	return i32_ret;
}

static int raydium_ts_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct raydium_ts_platform_data *pdata =
		(struct raydium_ts_platform_data *)client->dev.platform_data;

	struct raydium_ts_data *ts;
	struct input_dev *input_dev;
	unsigned short u16_i2c_data;
	int ret = 0;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct raydium_ts_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev,
				"[touch]failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = raydium_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev,
				"[touch]device tree parsing failed\n");
			goto parse_dt_failed;
		}
	} else
		pdata = client->dev.platform_data;


	pr_info("[touch] probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ts = devm_kzalloc(&client->dev,
				  sizeof(struct raydium_ts_data),
				  GFP_KERNEL);
	if (!ts) {
		pr_err("[touch]failed to allocate input driver data\n");
		return -ENOMEM;
	}
	raydium_variable_init();

	mutex_init(&ts->lock);

	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->irq_enabled = false;
	ts->irq_wake = false;

	ts->irq_gpio = pdata->irq_gpio;
	ts->rst_gpio = pdata->reset_gpio;
	client->irq = ts->irq_gpio;
	ts->u8_max_touchs = pdata->num_max_touches;
	ts->client = client;
	ts->x_max = pdata->x_max - 1;
	ts->y_max = pdata->y_max - 1;
	ts->is_suspend = 0;
	ts->is_sleep = 0;
#ifdef GESTURE_EN
	ts->is_palm = 0;
#endif
	ts->fw_version = 0;

	device_init_wakeup(&client->dev, 1);

#ifdef MSM_NEW_VER
	ret = raydium_ts_pinctrl_init(ts);
	if (!ret && ts->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret = pinctrl_select_state(ts->ts_pinctrl,
					   ts->pinctrl_state_active);
		if (ret < 0)
			pr_err("[touch]failed to set pin to active state\n");
	}
#endif /*end of MSM_NEW_VER*/

	ret = raydium_gpio_configure(ts, true);
	if (ret < 0) {
		pr_err("[touch]failed to configure the gpios\n");
		goto err_gpio_req;
	}
	/*modify dtsi to 360*/
	msleep(pdata->soft_rst_dly);

	/*SW reset*/
	/*
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x01;
	dev_info(&client->dev, "[touch]Raydium SW reset\n");

	ret = raydium_i2c_pda_write(client, 0x40000004, buf, 4);

	msleep(pdata->soft_rst_dly);

	g_u8_i2c_mode = PDA2_MODE;
	*/

	/*print touch i2c ready*/
	ret = raydium_check_i2c_ready(ts, &u16_i2c_data);
	if (ret < 0) {
		pr_err("[touch]Check I2C failed\n");
		ret = -ENODEV;
		goto exit_check_i2c;
	}

	/*input device initialization*/
	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		pr_err("[touch]failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	raydium_set_resolution(ts);

	ts->input_dev = input_dev;
	raydium_input_set(input_dev, ts);

	ret = input_register_device(input_dev);
	if (ret) {
		pr_err("[touch]failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef GESTURE_EN
	input_set_capability(input_dev, EV_KEY, KEY_SLEEP);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
#endif

	/*suspend/resume routine*/
#if defined(CONFIG_FB)
	raydium_register_notifier(ts);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	/*Early-suspend level*/
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = raydium_ts_early_suspend;
	ts->early_suspend.resume = raydium_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif/*end of CONFIG_FB*/

#ifdef CONFIG_RM_SYSFS_DEBUG
	raydium_create_sysfs(client);
#endif/*end of CONFIG_RM_SYSFS_DEBUG*/

	INIT_WORK(&ts->work, raydium_work_handler);

	ts->workqueue = create_singlethread_workqueue("raydium_ts");
	/*irq_gpio = 13 irqflags = 108*/
	pr_info("[touch]pdata irq : %d\n", ts->irq_gpio);
	pr_info("[touch]client irq : %d, pdata flags : %d\n",
		client->irq, pdata->irqflags);

	ts->irq = gpio_to_irq(pdata->irq_gpio);
	ret = request_threaded_irq(ts->irq, NULL, raydium_ts_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
			client->dev.driver->name, ts);

	if (ret < 0) {
		pr_err("[touch]raydium_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	ts->irq_desc = irq_to_desc(ts->irq);
	ts->irq_enabled = true;

	/*disable_irq then enable_irq for avoid Unbalanced enable for IRQ */

	/*raydium_irq_control(ts, ENABLE);*/

	pr_info("[touch]RAD Touch driver ver :0x%04X\n", RAYDIUM_VER);

	/*fw update check*/
	ret = raydium_fw_update_check(ts, u16_i2c_data);
	if (ret < 0) {
		pr_err("[touch]FW update check failed\n");
		ret = -ENODEV;
		goto exit_irq_request_failed;
	}

	return 0;

exit_irq_request_failed:
#if defined(CONFIG_FB)
	raydium_unregister_notifier(ts);
#endif/*end of CONFIG_FB*/

	cancel_work_sync(&ts->work);
	input_unregister_device(input_dev);

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
exit_check_i2c:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);

	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);

err_gpio_req:
#ifdef MSM_NEW_VER
	if (ts->ts_pinctrl) {
		if (IS_ERR_OR_NULL(ts->pinctrl_state_release)) {
			devm_pinctrl_put(ts->ts_pinctrl);
			ts->ts_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts->ts_pinctrl,
					ts->pinctrl_state_release);
			if (ret)
				pr_err("[touch]pinctrl_select_state failed\n");
		}
	}
#endif/*end of MSM_NEW_VER*/

parse_dt_failed:
exit_check_functionality_failed:
	return ret;

}

static int raydium_ts_remove(struct i2c_client *client)
{
	struct raydium_ts_data *ts;
	ts = i2c_get_clientdata(client);

#if defined(CONFIG_FB)
	raydium_unregister_notifier(ts);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif/*end of CONFIG_FB*/
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	gpio_free(ts->rst_gpio);

#ifdef CONFIG_RM_SYSFS_DEBUG
	raydium_release_sysfs(client);
#endif /*end of CONFIG_RM_SYSFS_DEBUG*/

	free_irq(client->irq, ts);

	if (gpio_is_valid(ts->rst_gpio))
		gpio_free(ts->rst_gpio);

	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);

	cancel_work_sync(&ts->work);
	destroy_workqueue(ts->workqueue);


	kfree(ts);

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
#endif/*end of CONFIG_OF*/

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
#endif/*end of CONFIG_PM*/
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
