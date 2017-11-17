/* drivers/input/misc/ots_pat9126/pat9126_linux_driver.c
 *
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/input.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include "pixart_ots.h"
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <asm/uaccess.h>

#include <linux/notifier.h>
#include <linux/reboot.h>
#if defined(CONFIG_FB)
#include <linux/fb.h>
#endif
#include<linux/time.h>
#include <linux/rtc.h>
#include <linux/string.h>

struct pixart_pat9126_data {
	struct i2c_client *client;
	struct input_dev *input;
	int irq_gpio;
	int irq;
	u32	irq_flags;
	struct device *pat9126_device;
	u32 press_keycode;
	bool press_en;
	bool inverse_x;
	bool inverse_y;
	struct regulator *vdd;
	struct regulator *vld;
	struct regulator *pvddio_reg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
	#if defined(CONFIG_FB)
		struct notifier_block fb_notif;
	#endif
};

/*IRQ Flags*/
struct irq_desc *pat9126_irq_desc = NULL;

/*Persist Flags*/
static int already_calibrated = 0;
extern char *saved_command_line;

/*Calibration Data Flags*/
int16_t Calib_res_x = 0;

struct mutex irq_mutex;
int en_irq_cnt = 0; /*Calculate times of enable irq*/
int dis_irq_cnt = 0;/*Calculate times of disable irq*/

struct rtc_time pat9126_tm;

struct rw_reg_info {
	char flag; /*R/W char*/
	long w_addr;
	long r_addr;
	long r_data;
};

struct rw_reg_info pat9126_reg_info;

/*Pixel Grab*/
const int pix_grab_len = 324;

/* Declaration of suspend and resume functions */
static int pat9126_suspend(struct device *dev);
static int pat9126_resume(struct device *dev);
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
static int pat9126_i2c_write(struct i2c_client *client, u8 reg, u8 *data,
		int len)
{
	u8 buf[MAX_BUF_SIZE];
	int ret = 0, i;
	struct device *dev = &client->dev;

	buf[0] = reg;
	if (len >= MAX_BUF_SIZE) {
		dev_err(dev, "%s Failed: buffer size is %d [Max Limit is %d]\n",
			__func__, len, MAX_BUF_SIZE);
		return -ENODEV;
	}
	for (i = 0 ; i < len; i++)
		buf[i+1] = data[i];
	/* Returns negative errno, or else the number of bytes written. */
	ret = i2c_master_send(client, buf, len+1);
	if (ret != len+1)
		dev_err(dev, "%s Failed: writing to reg 0x%x\n", __func__, reg);

	return ret;
}

static int pat9126_i2c_read(struct i2c_client *client, u8 reg, u8 *data)
{
	u8 buf[MAX_BUF_SIZE];
	int ret;
	struct device *dev = &client->dev;

	buf[0] = reg;
	/*
	 * If everything went ok (1 msg transmitted), return #bytes transmitted,
	 * else error code. thus if transmit is ok return value 1
	 */
	ret = i2c_master_send(client, buf, 1);
	if (ret != 1) {
		dev_err(dev, "%s Failed: writing to reg 0x%x\n", __func__, reg);
		return ret;
	}
	/* returns negative errno, or else the number of bytes read */
	ret = i2c_master_recv(client, buf, 1);
	if (ret != 1) {
		dev_err(dev, "%s Failed: reading reg 0x%x\n", __func__, reg);
		return ret;
	}
	*data = buf[0];

	return ret;
}

u8 read_data(struct i2c_client *client, u8 addr)
{
	u8 data = 0xff;

	pat9126_i2c_read(client, addr, &data);
	return data;
}

void write_data(struct i2c_client *client, u8 addr, u8 data)
{
	pat9126_i2c_write(client, addr, &data, 1);
}

extern void delay(int ms)
{
	msleep(ms);
}

void pat9126_get_time(char *output)
{
	struct timeval time;
	unsigned long local_time;

	do_gettimeofday(&time);
	local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
	rtc_time_to_tm(local_time, &pat9126_tm);

	printk(" @ (%04d-%02d-%02d %02d:%02d:%02d)\n", pat9126_tm.tm_year + 1900,
		pat9126_tm.tm_mon + 1, pat9126_tm.tm_mday, pat9126_tm.tm_hour,
		pat9126_tm.tm_min, pat9126_tm.tm_sec);

	sprintf(output, "%d%d%d%d%d%d", pat9126_tm.tm_year + 1900,
		pat9126_tm.tm_mon + 1, pat9126_tm.tm_mday,
		pat9126_tm.tm_hour, pat9126_tm.tm_min, pat9126_tm.tm_sec);
}

void pat9126_get_exec_time(char *output)
{
	struct timeval time;
	unsigned long local_time;

	do_gettimeofday(&time);
	local_time = (u32)(time.tv_sec - (sys_tz.tz_minuteswest * 60));
	rtc_time_to_tm(local_time, &pat9126_tm);

	printk(" @ (%04d-%02d-%02d %02d:%02d:%02d)\n", pat9126_tm.tm_year + 1900,
		pat9126_tm.tm_mon + 1, pat9126_tm.tm_mday, pat9126_tm.tm_hour,
		pat9126_tm.tm_min, pat9126_tm.tm_sec);

	sprintf(output, "(%04d-%02d-%02d %02d:%02d:%02d)",
		pat9126_tm.tm_year + 1900,
		pat9126_tm.tm_mon + 1, pat9126_tm.tm_mday,
		pat9126_tm.tm_hour, pat9126_tm.tm_min, pat9126_tm.tm_sec);
}

void pat9126_enter_pix_grab_mode(struct i2c_client *client)
{
	write_data(client, PIXART_PAT9126_WRITE_PROTECT_REG,
		PIXART_PAT9126_DISABLE_WRITE_PROTECT);
	write_data(client, PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP0_REG,
		PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP0_VAL);
	write_data(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG,
		PIXART_PAT9126_SLEEP1_MODE_EN);
	/*delay 2 ms*/
	delay(2);

	write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL2);
	write_data(client, PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP1_REG,
		PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP1_VAL);
	write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL1);
	write_data(client, PIXART_PAT9126_PIXEL_GRAB_CLOCK_REG,
		PIXART_PAT9126_PIXEL_GRAB_CLOCK_VAL);
}

void pat9126_exit_pix_grab_mode(struct i2c_client *client)
{
	write_data(client, PIXART_PAT9126_WRITE_PROTECT_REG,
		PIXART_PAT9126_DISABLE_WRITE_PROTECT);
	write_data(client, PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP0_REG,
		PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP0_VAL1);
	write_data(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG,
		PIXART_PAT9126_WAKEUP_MODE);
	/*delay 2 ms*/
	delay(2);

	write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL2);
	write_data(client, PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP1_REG,
		PIXART_PAT9126_PIXEL_GRAB_FRAME_SKIP1_VAL1);
	write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
		PIXART_PAT9126_SELECT_BANK_VAL1);
	write_data(client, PIXART_PAT9126_PIXEL_GRAB_CLOCK_REG,
		PIXART_PAT9126_PIXEL_GRAB_CLOCK_VAL1);
}

static irqreturn_t pat9126_irq(int irq, void *dev_data)
{
	int16_t delta_x = 0, delta_y = 0;
	struct pixart_pat9126_data *data = dev_data;
	struct input_dev *ipdev = data->input;
	struct device *dev = &data->client->dev;

	/* check if MOTION bit is set or not */
	ots_read_motion(data->client, &delta_x, &delta_y);
	pr_debug("[PAT9126]delta_x: %d, delta_y: %d \n", delta_x, delta_y);

	/* Inverse x depending upon the device orientation */
	delta_x = (data->inverse_x) ? -delta_x : delta_x;
	/* Inverse y depending upon the device orientation */
	delta_y = (data->inverse_y) ? -delta_y : delta_y;

	dev_dbg(dev, "delta_x = 0x%2x, delta_y = 0x%2x\n",
				delta_x, delta_y);

	if (delta_x != 0) {
		/* Send delta_x as REL_WHEEL for rotation */
		input_report_rel(ipdev, REL_WHEEL, (s8) delta_x);
		input_sync(ipdev);
	}

	return IRQ_HANDLED;
}

/*********beginning of calibration function***************/
static mm_segment_t oldfs;

static struct file *pat9126_calib_file_open(char *file_name)
{

	struct file* filp = NULL;
	char filepath[128];
	int err = 0;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", file_name);

	oldfs = get_fs();
	set_fs(get_ds());


	filp = filp_open(filepath, O_WRONLY|O_CREAT|O_APPEND , 0666);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}

	return filp;
}

int pat9126_calib_file_write(struct file* file, unsigned char *data, int len)
{

	int ret;
	ret = file->f_op->write(file, data, len, &file->f_pos);

	return ret;
}

void pat9126_calib_file_close(struct file* file)
{

	set_fs(oldfs);
	filp_close(file, NULL);
}
/*********end of calibration function**************/

/*********read persist file*********/
int str2num(char* ptr, int a) {
	int num;
	if (ptr[a] < 0x3a && ptr[a] > 0x2f) num = ptr[a] -0x30;
	else if (ptr[a] > 0x60 && ptr[a] < 0x7b) num = (ptr[a] -0x61+10);
	else num = 0 ;

	return num;
}

uint8_t pat9126_get_persist_data(char *ptr, int idx1, int idx2)
{
	int a, b;
	uint8_t num;

	a = str2num(ptr, idx1);
	b = str2num(ptr, idx2);

	a = (a << 4) & 0xf0;
	num = a + b;
	return num;
}

void pat9126_get_calib_data(void) {
	char* ptr;
	int idx = 0;
	ptr = strstr(saved_command_line, "androidboot.ots=");
	if (ptr != NULL) {
		ptr += strlen("androidboot.ots=");
		pr_err("[PAT9126]: %s\n", ptr);
		idx = 0;
		if (ptr[0] - 0x30 != 0 && ptr[0] - 0x30 != 1) {
			already_calibrated = 0;
			return;
		}
		already_calibrated = str2num(ptr, idx);
		if (already_calibrated == 1) {
			Calib_res_x = pat9126_get_persist_data(ptr, 2, 3);
		} else {
			Calib_res_x = PIXART_PAT9126_CPI_RESOLUTION_X;
		}
	}
	else
		already_calibrated = 0;

}
/*********read persist file*********/

static ssize_t pat9126_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int mode;

	if (kstrtoint(buf, 10, &mode)) {
		dev_err(dev, "failed to read input for sysfs\n");
		return -EINVAL;
	}

	if (mode == 1)
		pat9126_suspend(&client->dev);
	else if (mode == 0)
		pat9126_resume(&client->dev);

	return count;
}

static ssize_t pat9126_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char s[256], *p = s;
	int reg_data = 0, i;
	long rd_addr, wr_addr, wr_data;
	struct pixart_pat9126_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	for (i = 0; i < sizeof(s); i++)
		s[i] = buf[i];
	*(s+1) = '\0';
	*(s+4) = '\0';
	*(s+7) = '\0';
	/* example(in console): echo w 12 34 > rw_reg */
	if (*p == 'w') {
		p += 2;
		if (!kstrtol(p, 16, &wr_addr)) {
			p += 3;
			if (!kstrtol(p, 16, &wr_data)) {
				dev_dbg(dev, "w 0x%x 0x%x\n",
					(u8)wr_addr, (u8)wr_data);
				write_data(client, (u8)wr_addr, (u8)wr_data);
			}
		}
	}
	/* example(in console): echo r 12 > rw_reg */
	else if (*p == 'r') {
		p += 2;

		if (!kstrtol(p, 16, &rd_addr)) {
			reg_data = read_data(client, (u8)rd_addr);
			dev_dbg(dev, "r 0x%x 0x%x\n",
				(unsigned int)rd_addr, reg_data);
			printk(KERN_DEBUG "r 0x%x 0x%x\n",
				(unsigned int)rd_addr, reg_data);
		}
	}
	return count;
}

static ssize_t pat9126_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* cat */
	int count = 0;
	uint8_t sensor_pid = 0;
	uint8_t tmp = 0 ;
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pr_debug("%s (%d) :\n", __func__, __LINE__);
	/* Read sensor_pid in address 0x00 to check if
	the serial link is valid, PID should be 0x31 */
	sensor_pid = read_data(client, PIXART_PAT9126_PRODUCT_ID1_REG);
	count += sprintf(buf + count, "Sensor PID: 0x%2x\n", sensor_pid);

	tmp = read_data(client, PIXART_PAT9126_MOTION_STATUS_REG);
	count += sprintf(buf + count, "Motion Status: 0x%2x\n", tmp);

	tmp = read_data(client, PIXART_PAT9126_DELTA_X_LO_REG);
	count += sprintf(buf + count, "X_Lo: 0x%2x\n", tmp);

	tmp = read_data(client, PIXART_PAT9126_DELTA_Y_LO_REG);
	count += sprintf(buf + count, "Y_Lo: 0x%2x\n", tmp);

	tmp = read_data(client, PIXART_PAT9126_DELTA_XY_HI_REG);
	count += sprintf(buf + count, "XY_Hi: 0x%2x\n", tmp);

	tmp = read_data(client, PIXART_PAT9126_FRAME_AVG_REG);
	count += sprintf(buf + count, "Frame Avg: 0x%2x\n", tmp);

	tmp = read_data(client, 0x70);
	count += sprintf(buf + count, "Laser: 0x%2x\n", tmp);

	tmp = read_data(client, PIXART_PAT9126_SET_CPI_RES_X_REG);
	count += sprintf(buf + count, "RES_X: %d, 0x%2x\n", tmp, tmp);

	return count;
}

static ssize_t pat9126_rw_reg_store
	(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char s[256];
	char *p = s;
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	pr_debug("%s: read/write register. \n", __func__);

	memcpy(s, buf, sizeof(s));

	*(s+1) = '\0';
	*(s+4) = '\0';
	*(s+7) = '\0';
	/* example(in console): echo w 12 34 > rw_reg */
	if (*p == 'w') {
		long write_addr, wdata;
		pat9126_reg_info.flag = *p;
		printk(KERN_DEBUG "[PAT9126]%s, r/w flag: %c. \n", __func__,
			pat9126_reg_info.flag);
		p += 2;
		if (!kstrtol(p, 16, &write_addr)) {
			pat9126_reg_info.w_addr = write_addr;
			printk(KERN_DEBUG "[PAT9126]%s, w_addr: 0x%2x. \n", __func__,
				(u8)pat9126_reg_info.w_addr);
			p += 3;
			if (!kstrtol(p, 16, &wdata)) {
				printk(KERN_DEBUG "[PAT9126]%s, write register 0x%x: 0x%x\n",
					__func__, (u8)write_addr, (u8)wdata);
				write_data(client, (u8)write_addr, (u8)wdata);
			}
		}
	}
	/* example(in console): echo r 12 > rw_reg */
	else if (*p == 'r') {
		long read_addr;
		pat9126_reg_info.flag = *p;
		printk(KERN_DEBUG "[PAT9126]%s, r/w flag: %c. \n", __func__,
			(u8)pat9126_reg_info.flag);
		p += 2;
		if (!kstrtol(p, 16, &read_addr)) {
			int data = 0;
			data = read_data(client, (u8)read_addr);
			pat9126_reg_info.r_addr = read_addr;
			pat9126_reg_info.r_data = data;
			printk(KERN_DEBUG "[PAT9126]%s, r_addr: 0x%2x, r_data: 0x%2x. \n",
				__func__, (u8)pat9126_reg_info.r_addr, (u8)pat9126_reg_info.r_data);
			printk(KERN_DEBUG "[PAT9126]%s, read register 0x%x: 0x%x\n",
				__func__, (unsigned int)read_addr, data);
		}
	}
	else {
		printk(KERN_DEBUG "[PAT9126]%s, Error Command, %c\n", __func__, *p);
		pat9126_reg_info.w_addr = 0;
		pat9126_reg_info.r_addr = 0;
		pat9126_reg_info.r_data = 0;
		pat9126_reg_info.flag = 'e';
	}

	return count;
}

static ssize_t pat9126_rw_reg_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	if (pat9126_reg_info.flag == 'w') {
		int data = 0;
		data = read_data(client, (u8)pat9126_reg_info.w_addr);
		count += sprintf(buf + count, "[PAT9126]%s, Write Register 0x%2x: 0x%2x. \n",
			__func__, (u8)pat9126_reg_info.w_addr, data);
	}
	else if (pat9126_reg_info.flag == 'r') {
		count += sprintf(buf + count, "[PAT9126]%s, Read Register  0x%2x: 0x%2x. \n",
			__func__, (u8)pat9126_reg_info.r_addr, (u8)pat9126_reg_info.r_data);
	}
	else
		count += sprintf(buf + count, "[PAT9126]%s, No such command!\n", __func__);

	return count;
}

/*Doing Pixel Grab at active state is recommanded*/
static ssize_t pat9126_pixel_grab_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct file *w_file;
	int w_len=0;
	char result_buf[128];
	bool open_log_success = false;
	uint8_t rval = 0;
	uint8_t frame_valid_state = 0;
	uint8_t pix_valid_state = 0;
	uint8_t pixels[pix_grab_len];
	int i;
	uint8_t sensor_pid = 0;
	char w_filename[20] ={'\0'};
	char pat9126_pixel_filepath[128] = "/sdcard/pixel_grab_";
	char *txt_name = ".txt";
	char funexectime[32] ={'\0'};
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	for( i = 0; i < pix_grab_len; ++i) pixels[i] = 0;

	/*Get function execution time*/
	pat9126_get_exec_time(funexectime);

	sensor_pid = read_data(client, PIXART_PAT9126_PRODUCT_ID1_REG);
	if (sensor_pid != PIXART_PAT9126_SENSOR_ID) {
		count += sprintf(buf + count, "[PAT9126]%s, Didn't find sensor. \n", __func__);
		return count;
	}

	pat9126_get_time(w_filename);
	if (w_filename[0] != '\0') {
		strcat(pat9126_pixel_filepath, w_filename);
	}
	strcat(pat9126_pixel_filepath, txt_name);
	count += sprintf(buf + count, "[PAT9126]%s, Pixel file name: %s. \n",
		__func__, pat9126_pixel_filepath);

	/*Open file to record calibration steps*/
	w_file = pat9126_calib_file_open(pat9126_pixel_filepath);
	if (!w_file) {
		printk("[PAT9126][OTS] %s : Open log file fail !\n", __func__);
		open_log_success = false;
	}
	else {
		open_log_success = true;
		count += sprintf(buf + count, "[PAT9126]Open TXT file success. \n");
		w_len = sprintf(result_buf, "------%s start %s------\n", __func__, funexectime);
		pat9126_calib_file_write(w_file, result_buf, w_len);
	}

	/*Step1: Register initialization for Pixel Grabbing*/
	pat9126_enter_pix_grab_mode(client);

	/*Step2: Enter Pixel Grabbing Mode*/
	write_data(client, PIXART_PAT9126_PIXEL_GRAB_VAL_REG,
		PIXART_PAT9126_PIXEL_GRAB_RESET_VAL);
	delay(10);
	for( i = 0; i < pix_grab_len; ++i ) {
		rval = read_data(client, PIXART_PAT9126_PIXEL_GRAB_STATUS_REG);
		frame_valid_state = (rval & OTS_FRAME_VALID) >> 7;
		pix_valid_state = (rval & OTS_PIXEL_VALID) >> 6;
		delay(10);
		if (frame_valid_state == 1) {
			if(pix_valid_state == 1) {
				pixels[i] = read_data(client, PIXART_PAT9126_PIXEL_GRAB_VAL_REG);
				delay(10);
			}
			else {
				break;
			}
		}
		else {
			if(pix_valid_state == 1) {
				pixels[i] = read_data(client, PIXART_PAT9126_PIXEL_GRAB_VAL_REG);
				delay(10);
			}
			else {
				break;
			}
		}
	}
	/*Step: Print out pixel values*/
	count += sprintf(buf + count, "[PAT9126]%s, Pixel value: \n", __func__);
	if (open_log_success) {
		w_len = sprintf(result_buf, "Pixel value: \n");
		pat9126_calib_file_write(w_file, result_buf, w_len);
	}

	for( i = 0; i < pix_grab_len; ++i) {
		count += sprintf(buf + count, "%d, ", pixels[i]);
		if (open_log_success) {
			w_len = sprintf(result_buf, "%d, ", pixels[i]);
			pat9126_calib_file_write(w_file, result_buf, w_len);
		}
	}
	count += sprintf(buf + count, "\n");
	if (open_log_success) {
		w_len = sprintf(result_buf, "\n");
		pat9126_calib_file_write(w_file, result_buf, w_len);
	}

	/*Step3: Exit Pixel Grabbing and Return Normal Mode*/
	pat9126_exit_pix_grab_mode(client);

	if (open_log_success) {
		w_len = sprintf(result_buf, "Pixel Grab Finished. \n");
		pat9126_calib_file_write(w_file, result_buf, w_len);

		w_len = sprintf(result_buf, "------%s end------\n\n", __func__);
		pat9126_calib_file_write(w_file, result_buf, w_len);

		pat9126_calib_file_close(w_file);
		open_log_success = false;
	}
	return count;
}

static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR | S_IWGRP,
		NULL, pat9126_suspend_store);
static DEVICE_ATTR(test, S_IRUGO | S_IWUSR | S_IWGRP,
		pat9126_test_show, pat9126_test_store);
static DEVICE_ATTR
	(rw_reg, S_IRUGO | S_IWUSR | S_IWGRP, pat9126_rw_reg_show, pat9126_rw_reg_store);
static DEVICE_ATTR
	(pixel_grab, S_IRUGO, pat9126_pixel_grab_show, NULL);

static struct attribute *pat9126_attr_list[] = {
	&dev_attr_test.attr,
	&dev_attr_rw_reg.attr,
	&dev_attr_pixel_grab.attr,
	&dev_attr_suspend.attr,
	NULL,
};

static struct attribute_group pat9126_attr_grp = {
	.attrs = pat9126_attr_list,
};

static int pixart_pinctrl_init(struct pixart_pat9126_data *data)
{
	int err;
	struct device *dev = &data->client->dev;

	data->pinctrl = devm_pinctrl_get(&(data->client->dev));
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		err = PTR_ERR(data->pinctrl);
		dev_err(dev, "Target does not use pinctrl %d\n", err);
		return err;
	}

	data->pinctrl_state_active = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(data->pinctrl_state_active)) {
		err = PTR_ERR(data->pinctrl_state_active);
		dev_err(dev, "Can not lookup active pinctrl state %d\n", err);
		return err;
	}

	data->pinctrl_state_suspend = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(data->pinctrl_state_suspend)) {
		err = PTR_ERR(data->pinctrl_state_suspend);
		dev_err(dev, "Can not lookup suspend pinctrl state %d\n", err);
		return err;
	}

	data->pinctrl_state_release = pinctrl_lookup_state(data->pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
		err = PTR_ERR(data->pinctrl_state_release);
		dev_err(dev, "Can not lookup release pinctrl state %d\n", err);
		return err;
	}
	return 0;
}

static int pat9126_regulator_init(struct pixart_pat9126_data *data)
{
	int err = 0;
	struct device *dev = &data->client->dev;

	data->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(data->vdd)) {
		dev_err(dev, "Failed to get regulator vdd %ld\n",
					PTR_ERR(data->vdd));
		return PTR_ERR(data->vdd);
	}

	data->vld = devm_regulator_get(dev, "vld");
	if (IS_ERR(data->vld)) {
		dev_err(dev, "Failed to get regulator vld %ld\n",
					PTR_ERR(data->vld));
		return PTR_ERR(data->vld);
	}

	data->pvddio_reg = devm_regulator_get(dev, "vddio");
	if (IS_ERR(data->pvddio_reg)) {
		dev_err(dev, "Failed to get regulator vddio %ld\n",
					PTR_ERR(data->pvddio_reg));
		return PTR_ERR(data->pvddio_reg);
	}

	err = regulator_set_voltage(data->vdd, VDD_VTG_MIN_UV, VDD_VTG_MAX_UV);
	if (err) {
		dev_err(dev, "Failed to set voltage for vdd reg %d\n", err);
		return err;
	}

	err = regulator_set_optimum_mode(data->vdd, VDD_ACTIVE_LOAD_UA);
	if (err < 0) {
		dev_err(dev, "Failed to set opt mode for vdd reg %d\n", err);
		return err;
	}

	err = regulator_set_voltage(data->vld, VLD_VTG_MIN_UV, VLD_VTG_MAX_UV);
	if (err) {
		dev_err(dev, "Failed to set voltage for vld reg %d\n", err);
		return err;
	}

	err = regulator_set_optimum_mode(data->vld, VLD_ACTIVE_LOAD_UA);
	if (err < 0) {
		dev_err(dev, "Failed to set opt mode for vld reg %d\n", err);
		return err;
	}

	err = regulator_set_voltage(data->pvddio_reg, VDD_VTG_MIN_UV, VDD_VTG_MAX_UV);
	if (err) {
		dev_err(dev, "Failed to set voltage for vddio reg %d\n", err);
		return err;
	}

	err = regulator_set_optimum_mode(data->pvddio_reg, VDD_ACTIVE_LOAD_UA);
	if (err < 0) {
		dev_err(dev, "Failed to set opt mode for vddio reg %d\n", err);
		return err;
	}

	return 0;
}

static int pat9126_power_on(struct pixart_pat9126_data *data, bool on)
{
	int err = 0;
	struct device *dev = &data->client->dev;

	if (on) {
		err = regulator_enable(data->vdd);
		if (err) {
			dev_err(dev, "Failed to enable vdd reg %d\n", err);
			return err;
		}

		err = regulator_enable(data->pvddio_reg);
		if (err) {
			dev_err(dev, "Failed to enable vddio reg %d\n", err);
			return err;
		}

		usleep_range(DELAY_BETWEEN_REG_US, DELAY_BETWEEN_REG_US + 1);

		err = regulator_enable(data->vld);
		if (err) {
			dev_err(dev, "Failed to enable vld reg %d\n", err);
			return err;
		}
	} else {
		err = regulator_disable(data->vld);
		if (err) {
			dev_err(dev, "Failed to disable vld reg %d\n", err);
			return err;
		}

		err = regulator_disable(data->pvddio_reg);
		if (err) {
			dev_err(dev, "Failed to disable vddio reg %d\n", err);
			return err;
		}

		err = regulator_disable(data->vdd);
		if (err) {
			dev_err(dev, "Failed to disable vdd reg %d\n", err);
			return err;
		}
	}

	return 0;
}

static int pat9126_parse_dt(struct device *dev,
		struct pixart_pat9126_data *data)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int ret;

	data->inverse_x = of_property_read_bool(np, "pixart,inverse-x");
	data->inverse_y = of_property_read_bool(np, "pixart,inverse-y");
	data->press_en = of_property_read_bool(np, "pixart,press-enabled");
	if (data->press_en) {
		ret = of_property_read_u32(np, "pixart,press-keycode",
						&temp_val);
		if (!ret) {
			data->press_keycode = temp_val;
		} else {
			dev_err(dev, "Unable to parse press-keycode\n");
			return ret;
		}
	}

	data->irq_gpio = of_get_named_gpio_flags(np, "pixart,irq-gpio",
						0, NULL);

	return 0;
}

static int pat9126_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct pixart_pat9126_data *data;
	struct input_dev *input;
	struct device *dev = &client->dev;

	err = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	if (err < 0) {
		dev_err(dev, "I2C not supported\n");
		return -ENXIO;
	}

	if (client->dev.of_node) {
		data = devm_kzalloc(dev, sizeof(struct pixart_pat9126_data),
				GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		err = pat9126_parse_dt(dev, data);
		if (err) {
			dev_err(dev, "DT parsing failed, errno:%d\n", err);
			return err;
		}
	} else {
		data = client->dev.platform_data;
		if (!data) {
			dev_err(dev, "Invalid pat9126 data\n");
			return -EINVAL;
		}
	}
	data->client = client;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "Failed to alloc input device\n");
		return -ENOMEM;
	}

	input_set_capability(input, EV_REL, REL_WHEEL);
	if (data->press_en)
		input_set_capability(input, EV_KEY, data->press_keycode);

	i2c_set_clientdata(client, data);
	input_set_drvdata(input, data);
	input->name = PAT9126_DEV_NAME;

	data->input = input;
	err = input_register_device(data->input);
	if (err < 0) {
		dev_err(dev, "Failed to register input device\n");
		return err;
	}

	err = pixart_pinctrl_init(data);
	if (!err && data->pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Could not set pin to active state %d\n",
									err);
	} else {
		if (gpio_is_valid(data->irq_gpio)) {
			err = devm_gpio_request(dev, data->irq_gpio,
						"pixart_pat9126_irq_gpio");
			if (err) {
				dev_err(dev, "Couldn't request gpio %d\n", err);
				return err;
			}
			err = gpio_direction_input(data->irq_gpio);
			if (err) {
				dev_err(dev, "Couldn't set dir for gpio %d\n",
									err);
				return err;
			}
		} else {
			dev_err(dev, "Invalid gpio %d\n", data->irq_gpio);
			return -EINVAL;
		}
	}

	/*Read Persist File for filling calibraiton datas*/
	pat9126_get_calib_data();

	err = pat9126_regulator_init(data);
	if (err) {
		dev_err(dev, "Failed to init regulator, %d\n", err);
		return err;
	}

	err = pat9126_power_on(data, true);
	if (err) {
		dev_err(dev, "Failed to power-on the sensor %d\n", err);
		goto err_power_on;
	}

	usleep_range(DELAY_BETWEEN_REG_US, DELAY_BETWEEN_REG_US + 1);

	/*
		* Initialize pixart sensor after some delay, when vdd
		* regulator is enabled
	*/
	if (!ots_sensor_init(data->client, Calib_res_x)) {
		err = -ENODEV;
		dev_err(dev, "Failed to initialize sensor %d\n", err);
		return err;
	}

	pat9126_irq_desc = irq_to_desc(data->irq);

	if (already_calibrated == 1 && en_irq_cnt == 0) {
		pr_err("[PAT9126]: Probe Enable Irq. \n");
		err = devm_request_threaded_irq(dev, client->irq, NULL, pat9126_irq,
				 IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				"pixart_pat9126_irq", data);

		if (err) {
			dev_err(dev, "Req irq %d failed, errno:%d\n", client->irq, err);
			goto err_request_threaded_irq;
		}
		en_irq_cnt++;
	} else
		en_irq_cnt = 0;

	err = sysfs_create_group(&(input->dev.kobj), &pat9126_attr_grp);
	if (err) {
		dev_err(dev, "Failed to create sysfs group, errno:%d\n", err);
		goto err_sysfs_create;
	}

	#if defined(CONFIG_FB)
		data->fb_notif.notifier_call = fb_notifier_callback;

		err = fb_register_client(&data->fb_notif);

		if (err)
			dev_err(&client->dev, "Unable to register fb_notifier: %d\n", err);
	#endif

	return 0;

err_sysfs_create:
err_request_threaded_irq:
err_power_on:
	regulator_set_optimum_mode(data->vdd, 0);
	regulator_set_optimum_mode(data->pvddio_reg, 0);
	regulator_set_optimum_mode(data->vld, 0);
	if (pat9126_power_on(data, false) < 0)
		dev_err(dev, "Failed to disable regulators\n");
	if (data->pinctrl)
		if (pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_release) < 0)
			dev_err(dev, "Couldn't set pin to release state\n");

	return err;
}

static int pat9126_i2c_remove(struct i2c_client *client)
{
	struct pixart_pat9126_data *data = i2c_get_clientdata(client);
	struct device *dev = &data->client->dev;

	#if defined(CONFIG_FB)
		if (fb_unregister_client(&data->fb_notif))
			dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
	#endif

	sysfs_remove_group(&(data->input->dev.kobj), &pat9126_attr_grp);
	if (data->pinctrl)
		if (pinctrl_select_state(data->pinctrl,
				data->pinctrl_state_release) < 0)
			dev_err(dev, "Couldn't set pin to release state\n");
	regulator_set_optimum_mode(data->vdd, 0);
	regulator_set_optimum_mode(data->vld, 0);
	pat9126_power_on(data, false);
	return 0;
}

static int pat9126_suspend(struct device *dev)
{
	int rc;
	int ret = 0;
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);

	printk(KERN_ERR "[PAT9126]%s, start\n", __func__);
	if(pat9126_irq_desc->action == NULL) {
		printk(KERN_DEBUG "[PAT9126]%s: Not initialize yet \n.", __func__);
		return 0;
	}
	else
		printk(KERN_DEBUG "[PAT9126]%s: Already initialized \n.", __func__);

	if (dis_irq_cnt == 0){
		disable_irq(data->client->irq);
		if (data->pinctrl) {
			rc = pinctrl_select_state(data->pinctrl,
					data->pinctrl_state_suspend);
			if (rc < 0)
				dev_err(dev, "Could not set pin to suspend state %d\n",
										rc);
		}
		dis_irq_cnt++;
	}
	else {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}
	en_irq_cnt = 0;

	/*Write Register for Suspend Mode*/
	ret = ots_disable_mot(data->client,
		PIXART_PAT9126_SLEEP_MODE_DETECT_FREQ_DEFAULT);
	if (ret != 0){
		pr_err("[PAT9126]: Disable Motion FAIL.");
	}

	return 0;
}

static int pat9126_resume(struct device *dev)
{
	int rc;
	int ret = 0;
	struct pixart_pat9126_data *data =
		(struct pixart_pat9126_data *) dev_get_drvdata(dev);

	printk(KERN_ERR "[PAT9126]%s, start\n", __func__);

	if( pat9126_irq_desc->action == NULL) {
		printk(KERN_DEBUG "[PAT9126]%s: Not initialize yet. \n", __func__);
		return 0;
	}
	else
		printk(KERN_DEBUG "[PAT9126]%s: Already initialized. \n", __func__);

	ret = ots_enable_mot(data->client);
	if (ret != 0){
		pr_err("[PAT9126]: Enable Motion FAIL. \n");
		return 0;
	}

	if (en_irq_cnt == 0){
		if (data->pinctrl) {
			rc = pinctrl_select_state(data->pinctrl,
					data->pinctrl_state_active);
			if (rc < 0)
				dev_err(dev, "Could not set pin to active state %d\n",
										rc);
		}
		enable_irq(data->client->irq);
		en_irq_cnt++;
	}
	else {
		dev_info(dev, "Already in wake state \n");
		return 0;
	}
	dis_irq_cnt = 0;

	return 0;
}

#if defined(CONFIG_FB)
/*******************************************************************************
*  Name: fb_notifier_callback
*  Brief:
*  Input:
*  Output:
*  Return:
*******************************************************************************/
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct pixart_pat9126_data *ots_data =
		container_of(self, struct pixart_pat9126_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ots_data && ots_data->client) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			pat9126_resume(&ots_data->client->dev);
		break;

		case FB_BLANK_POWERDOWN:
		case FB_BLANK_VSYNC_SUSPEND:
			pat9126_suspend(&ots_data->client->dev);
		break;

		default:
		break;
		}
	}

	return 0;
}
#endif

static const struct i2c_device_id pat9126_device_id[] = {
	{PAT9126_DEV_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pat9126_device_id);

static const struct dev_pm_ops pat9126_pm_ops = {
#if (!defined(CONFIG_FB))
	.suspend = pat9126_suspend,
	.resume = pat9126_resume
#endif
};

static const struct of_device_id pixart_pat9126_match_table[] = {
	{ .compatible = "pixart,pat9126",},
	{ },
};

static struct i2c_driver pat9126_i2c_driver = {
	.driver = {
		   .name = PAT9126_DEV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &pat9126_pm_ops,
		   .of_match_table = pixart_pat9126_match_table,
		   },
	.probe = pat9126_i2c_probe,
	.remove = pat9126_i2c_remove,
	.id_table = pat9126_device_id,
};
module_i2c_driver(pat9126_i2c_driver);

MODULE_AUTHOR("pixart");
MODULE_DESCRIPTION("pixart pat9126 driver");
MODULE_LICENSE("GPL");
