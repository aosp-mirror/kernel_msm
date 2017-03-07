/*
* Copyright c 2016 PixArt Imaging Inc.. All Rights Reserved.

* This program is free software; you may redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>

#include "pixart_ots.h"
#include "pixart_platform.h"

#include <linux/regulator/consumer.h>

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <asm/uaccess.h>

#include <linux/notifier.h>
#include <linux/reboot.h>

static int pat9127_init_input_data(void);

#define PAT9127_DEV_NAME	"pixart_pat9127"
#define PINCTRL_STATE_IDSEL	"pmx_rot_switch_idsel"
#define PINCTRL_STATE_ACTIVE	"pmx_rot_switch_active"
#define PINCTRL_STATE_SUSPEND	"pmx_rot_switch_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_rot_switch_release"

#define FOR_CALIBRATION_DEBUG

#define PAT9127_RESULT_FILEPATH "/sdcard/"
#define PAT9127_FOR_PERSIST_FILEPATH "/persist/"

struct pat9127_linux_data_t {
	struct i2c_client *client;
	struct input_dev *pat9127_input_dev;
	struct device *i2c_dev;
	int irq_gpio;
	int irq;
	int pwr_reset_gpio;
	u32 irq_flags;
	u32 pwr_reset_flags;
	struct device *pat9127_device;
	u64 last_jiffies;
	struct regulator *pvdd_reg;
	struct regulator *pvddio_reg;
	struct regulator *pvld_reg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_idsel;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
};

static struct pat9127_linux_data_t pat9127data;

static uint8_t BtnRLSframeavg = 0;
static uint8_t BtnPRSframeavg = 0;

static int already_calibrated = 0;
extern char *saved_command_line;

uint8_t btn_fa_rls = 0;
/*For record to csv/persist file*/
uint8_t btn_hithd = 0, btn_lothd = 0;
uint8_t shutter_c = 0, shutter_f = 0;
int16_t Calib_res_x = 0;

struct mutex irq_mutex;

static void pat9127_stop(void);

/*notify_sys: for solving current leaking problem during devices power off*/
static int pat9127_notify_sys(struct notifier_block *this,
	unsigned long code, void *unused)
{
	uint8_t sensor_pid = 0;

	sensor_pid = OTS_Read_Reg(PIXART_PAT9127_PRODUCT_ID1_REG);
	if (sensor_pid == PIXART_PAT9127_SENSOR_ID) {
		/* Disable irq */
		pat9127_stop();
	}

	return NOTIFY_DONE;
}

static struct notifier_block pat9127_notifier = {
	.notifier_call  = pat9127_notify_sys,
};

static int pat9127_i2c_write(u8 reg, u8 *data, int len);
static int pat9127_i2c_read(u8 reg, u8 *data);

extern unsigned char OTS_Read_Reg(unsigned char addr)
{
	u8 data = 0xff;
	pat9127_i2c_read(addr, &data);
	return data;
}

extern void OTS_Write_Reg(unsigned char addr, unsigned char data)
{
	pat9127_i2c_write(addr, &data, 1);
}

extern void delay(int ms)
{
	msleep(ms);
}

static int pat9127_i2c_write(u8 reg, u8 *data, int len)
{
	u8  buf[20];
	int rc;
	int ret = 0;
	int i;

	buf[0] = reg;
	if (len >= 20) {
		pr_debug("%s (%d) : FAILED: buffer size is limitted(20) %d\n",
			__func__, __LINE__, len);
		return -ENODEV;
	}

	for (i = 0 ; i < len; i++)
		buf[i+1] = data[i];

	/* Returns negative errno, or else the number of bytes written. */
	rc = i2c_master_send(pat9127data.client, buf, len+1);

	if (rc != len+1) {
		pr_debug("%s (%d) : FAILED: writing to reg 0x%x\n",
			__func__, __LINE__, reg);

		ret = -ENODEV;
	}

	return ret;
}

static int pat9127_i2c_read(u8 reg, u8 *data)
{

	u8  buf[20];
	int rc;

	buf[0] = reg;

	/* If everything went ok (i.e. 1 msg transmitted),
	return #bytes  transmitted, else error code.
	thus if transmit is ok  return value 1 */
	rc = i2c_master_send(pat9127data.client, buf, 1);
	if (rc != 1) {
		pr_debug("%s (%d) : FAILED: writing to address 0x%x\n",
			__func__, __LINE__, reg);
		return -ENODEV;
	}

	/* returns negative errno, or else the number of bytes read */
	rc = i2c_master_recv(pat9127data.client, buf, 1);
	if (rc != 1) {
		pr_debug("%s (%d) : FAILED: reading data\n",
			__func__, __LINE__);
		return -ENODEV;
	}

	*data = buf[0];
	return 0;
}

static void time_interval_check(struct pat9127_linux_data_t *ots)
{
	#define TIME_INTERVAL_RESET_TH 2000 /* 2 sec */

	int interval_ms;
	interval_ms = jiffies_to_msecs(jiffies_64 - ots->last_jiffies);

	ots->last_jiffies = jiffies;
	if (interval_ms > TIME_INTERVAL_RESET_TH)	{
		pr_debug("Reset Variables %d ms\n", interval_ms);
		OTS_Reset_Variables();
	}
}

void pixart_pat9127_ist(void)
{
	/* "cat /proc/kmsg" to see kernel message */
	int16_t deltaX, deltaY;
	uint8_t OutBtnState;
	uint8_t OutOtsState;
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);

	time_interval_check(&pat9127data);
	OutBtnState = OTS_Check_MotionAndButton(&deltaX, &deltaY);
	OutOtsState = OTS_Detect_Rotation(deltaX, deltaY);
	pr_debug("delta(%d, %d)\n", deltaX, deltaY);
	pr_debug("status(%d, %d)\n", OutOtsState, OutBtnState);

	if (OutOtsState == OTS_ROT_UP) {
		/* Right Key */
		input_report_rel(pat9127data.pat9127_input_dev,
			REL_WHEEL, (int)(deltaX*(-1)) );
		input_sync(pat9127data.pat9127_input_dev);
		pr_debug("[PAT9127]: Roll Up, delta(%d, %d)\n", -deltaX, deltaY);
	}
	else if (OutOtsState == OTS_ROT_DOWN) {
		/* Left Key */
		input_report_rel(pat9127data.pat9127_input_dev,
			REL_WHEEL, (int)(deltaX*(-1)) );
		input_sync(pat9127data.pat9127_input_dev);
		pr_debug("[PAT9127]: Roll Down, delta(%d, %d)\n", -deltaX, deltaY);
	}

	if (OutBtnState == OTS_BTN_RELEASE)	{
		BtnRLSframeavg = OTS_Read_Reg(PIXART_PAT9127_FRAME_AVG_REG);
		pr_debug("[PAT9127]: Btn release state: %d\n", BtnRLSframeavg);
	}

	if (OutBtnState == OTS_BTN_PRESS) {
		BtnPRSframeavg = OTS_Read_Reg(PIXART_PAT9127_FRAME_AVG_REG);
		pr_debug("[PAT9127]: Btn press state: %d\n", BtnPRSframeavg);
	}
}

static irqreturn_t pixart_pat9127_irq(int irq, void *handle)
{
	/* "cat /proc/kmsg" to see kernel message */
	pr_debug("[PAT9127]: Start irq function. \n");
	mutex_lock(&irq_mutex);
	pixart_pat9127_ist();
	mutex_unlock(&irq_mutex);
	return IRQ_HANDLED;
}

static void pat9127_stop(void)
{
	int err = 0;
	uint8_t tmp_1 = 0;
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
	disable_irq(pat9127data.client->irq);

	OTS_WriteRead_Reg(PIXART_PAT9127_SENSOR_MODE_SELECT_REG,
		PIXART_PAT9127_SENSOR_DEFAULT_MODE); // Set btn, motion to non-open drain
	tmp_1 = OTS_Read_Reg(PIXART_PAT9127_SENSOR_MODE_SELECT_REG);
	pr_debug("[PAT9127]: pat9127 open drain mode motion: 0x%2x. \n", tmp_1);

	/*Setting Motion Interrupt to pull down*/

	err = pinctrl_select_state(pat9127data.pinctrl,
		pat9127data.pinctrl_state_idsel);
	if (err < 0)
		pr_err("[PAT9127]: Could not set pin to idsel state %d\n", err);

	free_irq(pat9127data.irq, &pat9127data);
}

static ssize_t pat9127_fops_read(struct file *filp, char *buf,
	size_t count, loff_t *l)
{
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
	return 0;
}

static ssize_t pat9127_fops_write(struct file *filp, const char *buf,
	size_t count, loff_t *f_ops)
{
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
	return 0;
}

static long pat9127_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
	return 0;
}

static int pat9127_fops_open(struct inode *inode, struct file *filp)
{
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
	return 0;
}

static int pat9127_fops_release(struct inode *inode, struct file *filp)
{
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
	return 0;
}

static const struct file_operations pat9127_fops = {
	owner: THIS_MODULE,
	read: pat9127_fops_read,
	write: pat9127_fops_write,
	unlocked_ioctl: pat9127_fops_ioctl,
	open: pat9127_fops_open,
	release: pat9127_fops_release,
};

struct miscdevice pat9127_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = PAT9127_DEV_NAME,
	.fops = &pat9127_fops,
};

/*********read persist file*********/
int str2num(char* ptr, int a) {
	int num;
	if (ptr[a] < 0x3a && ptr[a] > 0x2f) num = ptr[a] -0x30;
	else if (ptr[a] > 0x60 && ptr[a] < 0x7b) num = (ptr[a] -0x61+10);
	else num = 0 ;

	return num;
}

uint8_t pat9127_get_persist_data(char *ptr, int idx1, int idx2)
{
	int a, b;
	uint8_t num;

	a = str2num(ptr, idx1);
	b = str2num(ptr, idx2);
	a = (a << 4) & 0xf0;
	num = a + b;
	return num;
}

void pat9127_get_calib_data(void) {
	char* ptr;
	int idx = 0;
	ptr = strstr(saved_command_line, "androidboot.ots=");
	ptr += strlen("androidboot.ots=");
	if (ptr != NULL) {
		pr_debug("[PAT9127]: %s \n", ptr);

		idx = 0;
		already_calibrated = str2num(ptr, idx);
		shutter_c = pat9127_get_persist_data(ptr, 2, 3);
		shutter_f = pat9127_get_persist_data(ptr, 5, 6);
		btn_hithd = pat9127_get_persist_data(ptr, 8, 9);
		btn_lothd = pat9127_get_persist_data(ptr, 11, 12);
		Calib_res_x = pat9127_get_persist_data(ptr, 14, 15);
	}
	else
		already_calibrated = 0;
}
/*********read persist file*********/

/* :/sys/class/misc/pixart_pat9127/ */
static ssize_t pat9127_test_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	char s[256];
	char *p = s;

	pr_debug("%s (%d) : write_reg_store\n", __func__, __LINE__);
	memcpy(s, buf, sizeof(s));

	*(s+1) = '\0';
	*(s+4) = '\0';
	*(s+7) = '\0';
	/* example(in console): echo w 12 34 > rw_reg */
	if (*p == 'w') {
		long write_addr, write_data;
		p += 2;
		if (!kstrtol(p, 16, &write_addr)) {
			p += 3;
			if (!kstrtol(p, 16, &write_data)) {
				pr_debug("w 0x%x 0x%x\n",
					(u8)write_addr, (u8)write_data);
				OTS_Write_Reg((u8)write_addr, (u8)write_data);
			}
		}
	}
	/* example(in console): echo r 12 > rw_reg */
	else if (*p == 'r') {
		long read_addr;
		p += 2;

		if (!kstrtol(p, 16, &read_addr)) {
			int data = 0;
			data = OTS_Read_Reg((u8)read_addr);
			pr_debug("r 0x%x 0x%x\n", (unsigned int)read_addr, data);
		}
	}
	return count;
}

static ssize_t pat9127_test_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* cat */
	int count = 0;
	uint8_t sensor_pid = 0;
	uint8_t tmp = 0 ;

	pr_debug("%s (%d) :\n", __func__, __LINE__);
	/* Read sensor_pid in address 0x00 to check if
	the serial link is valid, PID should be 0x31 */
	sensor_pid = OTS_Read_Reg(PIXART_PAT9127_PRODUCT_ID1_REG);
	count += sprintf(buf + count, "Sensor PID: 0x%2x\n", sensor_pid);

	tmp = OTS_Read_Reg(PIXART_PAT9127_MOTION_STATUS_REG);
	count += sprintf(buf + count, "Motion Status: 0x%2x\n", tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_DELTA_X_LO_REG);
	count += sprintf(buf + count, "X_Lo: 0x%2x\n", tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_DELTA_Y_LO_REG);
	count += sprintf(buf + count, "Y_Lo: 0x%2x\n", tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_DELTA_XY_HI_REG);
	count += sprintf(buf + count, "XY_Hi: 0x%2x\n", tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_FRAME_AVG_REG);
	count += sprintf(buf + count, "Frame Avg: 0x%2x\n", tmp);

	tmp = OTS_Read_Reg(0x70);
	count += sprintf(buf + count, "Laser: 0x%2x\n", tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_SHUTTER_C_REG);
	count += sprintf(buf + count, "SHUTTER_C: %d, 0x%2x\n", tmp, tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_SHUTTER_F_REG);
	count += sprintf(buf + count, "SHUTTER_F: %d, 0x%2x\n", tmp, tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_BTN_HITHD_REG);
	count += sprintf(buf + count, "HI_TH: %d, 0x%2x\n", tmp, tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_BTN_LOTHD_REG);
	count += sprintf(buf + count, "LO_TH: %d, 0x%2x\n", tmp, tmp);

	tmp = OTS_Read_Reg(PIXART_PAT9127_SET_CPI_RES_X_REG);
	count += sprintf(buf + count, "RES_X: %d, 0x%2x\n", tmp, tmp);

	return count;
}

static DEVICE_ATTR
	(test, 0644 , pat9127_test_show, pat9127_test_store);

static struct device_attribute *pat9127_attr_list[] = {
	&dev_attr_test,
};

static int pixart_pinctrl_init(struct pat9127_linux_data_t *data)
{
	int err;

	data->pinctrl = devm_pinctrl_get(&(data->client->dev));
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		err = PTR_ERR(data->pinctrl);
		pr_err("Target does not use pinctrl %d\n", err);
		return err;
	}

	data->pinctrl_state_idsel = pinctrl_lookup_state(data->pinctrl,
		PINCTRL_STATE_IDSEL);
	if (IS_ERR_OR_NULL(data->pinctrl_state_idsel)) {
		err = PTR_ERR(data->pinctrl_state_idsel);
		pr_err("Can not lookup idsel pinctrl state %d\n", err);
		return err;
	}

	data->pinctrl_state_active = pinctrl_lookup_state(data->pinctrl,
		PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(data->pinctrl_state_active)) {
		err = PTR_ERR(data->pinctrl_state_active);;
		pr_err("Can not lookup active pinctrl state %d\n", err);
		return err;
	}

	data->pinctrl_state_suspend = pinctrl_lookup_state(data->pinctrl,
		PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(data->pinctrl_state_suspend)) {
		err = PTR_ERR(data->pinctrl_state_suspend);
		pr_err("Can not lookup suspend pinctrl state %d\n", err);
		return err;
	}

	data->pinctrl_state_release = pinctrl_lookup_state(data->pinctrl,
		PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
		err = PTR_ERR(data->pinctrl_state_release);
		pr_err("Can not lookup release pinctrl state %d\n", err);
		return err;
	}
	return 0;
}

static int pat9127_create_attr(struct device *dev)
{
	int idx, err = 0;
	int num = (int)(sizeof(pat9127_attr_list)/sizeof(pat9127_attr_list[0]));
	if (!dev)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = device_create_file(dev, pat9127_attr_list[idx]);
		if (err) {
			pr_debug("device_create_file (%s) = %d\n",
				pat9127_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

static int pat9127_delete_attr(struct device *dev)
{
	int idx , err = 0;
	int num = (int)(sizeof(pat9127_attr_list)/sizeof(pat9127_attr_list[0]));
	if (!dev)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		device_remove_file(dev, pat9127_attr_list[idx]);

	return err;
}

static int pat9127_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;
	int err_id = 0;
	struct device_node *np;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	pat9127data.i2c_dev = &client->dev;
	np = pat9127data.i2c_dev->of_node;
	mutex_init(&irq_mutex);

	pr_debug("%s (%d):[PAT9127]: probe module....\n", __func__, __LINE__);

	memset(&pat9127data, 0, sizeof(pat9127data));
	pat9127data.client = client;

	err_id = pixart_pinctrl_init(&pat9127data);

	if (err_id || !pat9127data.pinctrl) {
		pr_err("[PAT9127]: Could not configure idsel pin state \n");
		goto error_return;
	}
	/*
	 * Pinctrl handle is optional. If pinctrl handle is found
	 * let pins to be configured in active state. If not
	 * found continue further without error.
	 */
	err = pinctrl_select_state(pat9127data.pinctrl,
		pat9127data.pinctrl_state_idsel);
	if (err < 0)
		pr_debug("[PAT9127]: Set pin state error %d\n", err);

	/*power reset*/
	pat9127data.pwr_reset_gpio = of_get_named_gpio_flags(np,
		"pixart,pwr-reset-gpio", 0, &pat9127data.pwr_reset_flags);

	if (!gpio_is_valid(pat9127data.pwr_reset_gpio)) {
		err = (-1);
		pr_debug("invalid pwr_reset_gpio: %d\n",
			pat9127data.pwr_reset_gpio);
		goto error_return;
	}
	err = gpio_request(pat9127data.pwr_reset_gpio,
		"pixart_pat9127_pwr_reset_gpio");
	if (err) {
		pr_err("unable to request gpio [%d], [%d]\n",
			pat9127data.pwr_reset_gpio, err);
		goto error_return;
	}

	err = gpio_direction_output(pat9127data.pwr_reset_gpio, 1);
	if (err) {
		pr_err("unable to set dir for gpio[%d], [%d]\n",
			pat9127data.pwr_reset_gpio, err);
		goto error_return;
	}
	msleep(20);

	err = gpio_direction_output(pat9127data.pwr_reset_gpio, 0);
	if (err) {
		pr_err("unable to set dir for gpio[%d], [%d]\n",
			pat9127data.pwr_reset_gpio, err);
		goto error_return;
	}

	pat9127data.pvddio_reg = regulator_get(&client->dev, "vddio");
	if (IS_ERR(pat9127data.pvddio_reg)) {
		pr_err("%s: could not get pat9127 pvddio\n", __func__);
		pat9127data.pvddio_reg = NULL;
	}

	if (pat9127data.pvddio_reg != NULL)
	{
		err = regulator_enable(pat9127data.pvddio_reg);
		if (err < 0){
			pr_err("%s: not able to enable pvddio\n", __func__);
		}
	}

	err = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE);
	if (err < 0)
		goto error_return;

	err = register_reboot_notifier(&pat9127_notifier);
	if (err != 0)
		pr_err("cannot register reboot notifier (err=%d)\n", err);
	err = misc_register(&pat9127_device);
	if (err) {
		pr_err("[pat9127]: device register failed\n");
		goto error_return;
	}

	pat9127data.pat9127_device = pat9127_device.this_device;
	err = pat9127_create_attr(pat9127data.pat9127_device);
	if (err) {
		pr_err("[pat9127]: create attribute err = %d\n", err);
		goto error_return;
	}

	if (pat9127_init_input_data() < 0)
		goto error_return;

	/* interrupt initialization */
	pat9127data.irq_gpio = of_get_named_gpio_flags(np,
		"pixart,irq-gpio", 0, &pat9127data.irq_flags);

	pr_debug("irq_gpio: %d, irq_flags: 0x%x\n",
		pat9127data.irq_gpio, pat9127data.irq_flags);

	if (!gpio_is_valid(pat9127data.irq_gpio)) {
		err = (-1);
		pr_err("invalid irq_gpio: %d\n", pat9127data.irq_gpio);
		goto error_return;
	}
	err = gpio_request(pat9127data.irq_gpio, "pixart_pat9127_irq_gpio");
	if (err) {
		pr_err("unable to request gpio [%d], [%d]\n",
			pat9127data.irq_gpio, err);
		goto error_return;
	}

	err = gpio_direction_input(pat9127data.irq_gpio);
	if (err) {
		pr_err("unable to set dir for gpio[%d], [%d]\n",
			pat9127data.irq_gpio, err);
		goto error_return;
	}

	pat9127data.irq = gpio_to_irq(pat9127data.irq_gpio);

	/*Read Persist File for filling calibraiton datas*/
	pat9127_get_calib_data();

	if (!OTS_Sensor_Init(already_calibrated, shutter_c, shutter_f,
			btn_hithd, btn_lothd, Calib_res_x))
		goto error_return;

	/*Change Pin Select to NO-PULL*/
	err = pinctrl_select_state(pat9127data.pinctrl,
			pat9127data.pinctrl_state_active);
	if (err < 0)
		pr_err("[pat9127]: Set pin state error %d\n", err);

	if (already_calibrated) {
		err = devm_request_threaded_irq(&client->dev, pat9127data.irq,
				NULL, pixart_pat9127_irq,
				pat9127data.irq_flags | IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				"pixart_pat9127_irq", &pat9127data);
		if (err){
			pr_err("[pat9127]: Req irq %d failed, errno:%d\n",
				client->irq, err);
			goto error_return;
		}
	}

	pat9127data.last_jiffies = jiffies;
	return 0;

error_return:
	return err;
}

static int pat9127_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int pat9127_suspend(struct device *dev)
{
	pr_debug("%s (%d) : pat9127 suspend\n", __func__, __LINE__);
	return 0;
}

static int pat9127_resume(struct device *dev)
{
	pr_debug("%s (%d) : pat9127 resume\n", __func__, __LINE__);
	return 0;
}

static const struct i2c_device_id pat9127_device_id[] = {
	{PAT9127_DEV_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, pat9127_device_id);

static const struct dev_pm_ops pat9127_pm_ops = {
	.suspend = pat9127_suspend,
	.resume = pat9127_resume
};

static struct of_device_id pixart_pat9127_match_table[] = {
	{ .compatible = "pixart,pat9127",},
	{ },
};

static struct i2c_driver pat9127_i2c_driver = {
	.driver = {
		   .name = PAT9127_DEV_NAME,
		   .owner = THIS_MODULE,
		   .pm = &pat9127_pm_ops,
		   .of_match_table = pixart_pat9127_match_table,
	},
	.probe = pat9127_i2c_probe,
	.remove = pat9127_i2c_remove,
	.id_table = pat9127_device_id,
};
static int pat9127_open(struct input_dev *dev)
{
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
	return 0;
}

static void pat9127_close(struct input_dev *dev)
{
	pr_debug(">>> %s (%d)\n", __func__, __LINE__);
}

static int pat9127_init_input_data(void)
{
	int ret = 0;

	pr_debug("%s (%d) : initialize data\n", __func__, __LINE__);

	pat9127data.pat9127_input_dev = input_allocate_device();
	if (!pat9127data.pat9127_input_dev) {
		pr_debug("%s (%d) : could not allocate mouse input device\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	__set_bit(EV_KEY, pat9127data.pat9127_input_dev->evbit);
	__set_bit(KEY_LEFT, pat9127data.pat9127_input_dev->keybit);
	__set_bit(KEY_RIGHT, pat9127data.pat9127_input_dev->keybit);
	__set_bit(KEY_ENTER, pat9127data.pat9127_input_dev->keybit);
	input_set_capability(pat9127data.pat9127_input_dev, EV_REL, REL_WHEEL);

	input_set_drvdata(pat9127data.pat9127_input_dev, &pat9127data);
	pat9127data.pat9127_input_dev->name = PAT9127_DEV_NAME;

	pat9127data.pat9127_input_dev->open = pat9127_open;
	pat9127data.pat9127_input_dev->close = pat9127_close;

	ret = input_register_device(pat9127data.pat9127_input_dev);
	if (ret < 0) {
		input_free_device(pat9127data.pat9127_input_dev);
		pr_debug("%s (%d) : could not register input device\n",
			__func__, __LINE__);
		return ret;
	}

	return 0;
}

static int __init pat9127_linux_init(void)
{
	pr_debug("%s (%d) :init module\n", __func__, __LINE__);
	return i2c_add_driver(&pat9127_i2c_driver);
}

static void __exit pat9127_linux_exit(void)
{
	pr_debug("%s (%d) : exit module\n", __func__, __LINE__);
	pat9127_stop();
	misc_register(&pat9127_device);
	pat9127_delete_attr(pat9127data.pat9127_device);
	i2c_del_driver(&pat9127_i2c_driver);
}

module_init(pat9127_linux_init);
module_exit(pat9127_linux_exit);
MODULE_AUTHOR("pixart");
MODULE_DESCRIPTION("pixart pat9127 driver");
MODULE_LICENSE("GPL");
