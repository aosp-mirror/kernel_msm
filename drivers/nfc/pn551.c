/* Copyright (c) 2016, The Linux Foundation. All rightsreserved.
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
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include "pn551.h"
#include <linux/types.h>
#include <linux/regulator/consumer.h>

struct regulator *nfc_vreg_l6;
static unsigned int NFC_I2C_SCL;
static unsigned int NFC_I2C_SDA;

int is_debug = 0;
int is_debug_en = 0;
int s_wdcmd_cnt = 0;

#define DBUF(buff,count) \
	if (is_debug) \
		for (i = 0; i < count; i++) \
			printk(KERN_DEBUG "[NFC] %s : [%d] = 0x%x\n", \
				__func__, i, buff[i]);

#define D(x...)	\
	if (is_debug)	\
		printk(KERN_DEBUG "[NFC] " x)
#define I(x...) printk(KERN_INFO "[NFC] " x)
#define E(x...) printk(KERN_ERR "[NFC] [Err] " x)

#define MAX_BUFFER_SIZE	512

#define I2C_RETRY_COUNT 10

struct pn551_dev	{
	struct class		*pn551_class;
	struct device		*pn_dev;
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct wake_lock io_wake_lock;
	struct i2c_client	*client;
	struct miscdevice	pn551_device;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	unsigned int 		ven_gpio;
	unsigned int		ven_value;
	unsigned int 		firm_gpio;
	unsigned int		pvdd_en;
	void (*gpio_init) (void);
	unsigned int 		ven_enable;
	bool                     isReadBlock;
};

struct pn551_dev *pn_info;

static int pn551_RxData(uint8_t *rxData, int length)
{
	int ret;
	uint8_t loop_i;
	struct pn551_dev *pni = pn_info;

	struct i2c_msg msg[] = {
		{
		 .addr = pni->client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	D("%s: [addr=%x flag=%x len=%x]\n", __func__,
		msg[0].addr, msg[0].flags, msg[0].len);

	rxData[0] = 0;

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		D("%s: retry %d ........\n", __func__, loop_i);
		if (i2c_transfer(pni->client->adapter, msg, 1) > 0)
			break;
		if (loop_i >= 3) {
			E("%s : nfc_vreg_l6 regulator_is_enabled = %d\n", __func__, regulator_is_enabled(nfc_vreg_l6));
			E("%s : nfc_vreg_l6 regulator_get_voltage = %d\n", __func__, regulator_get_voltage(nfc_vreg_l6));
			E("%s : irq_gpio = %d, ven_gpio = %d, firm_gpio = %d\n", __func__, \
			gpio_get_value(pni->irq_gpio), gpio_get_value(pni->ven_gpio), gpio_get_value(pni->firm_gpio));
			if(!regulator_is_enabled(nfc_vreg_l6)) {
				ret = regulator_enable(nfc_vreg_l6);
				I("%s : vreg_l16 regulator_enable\n", __func__);
				I("%s : vreg_l16 regulator_is_enabled = %d\n", __func__, regulator_is_enabled(nfc_vreg_l6));
				if (ret < 0) {
					E("%s : vreg_l16 regulator_enable fail\n", __func__);
				}
			}

		}

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		E("%s Error: retry over %d\n", __func__,
			I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int pn551_TxData(uint8_t *txData, int length)
{
	int ret;
	uint8_t loop_i;
	struct pn551_dev *pni = pn_info;
	struct i2c_msg msg[] = {
		{
		 .addr = pni->client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	D("%s: [addr=%x flag=%x len=%x]\n", __func__,
		msg[0].addr, msg[0].flags, msg[0].len);

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		D("%s: retry %d ........\n", __func__, loop_i);
		if (i2c_transfer(pni->client->adapter, msg, 1) > 0)
			break;
		if (loop_i >= 3) {
			E("%s : nfc_vreg_l6 regulator_is_enabled = %d\n", __func__, regulator_is_enabled(nfc_vreg_l6));
			E("%s : nfc_vreg_l6 regulator_get_voltage = %d\n", __func__, regulator_get_voltage(nfc_vreg_l6));
			E("%s : irq_gpio = %d, ven_gpio = %d, firm_gpio = %d\n", __func__, \
			gpio_get_value(pni->irq_gpio), gpio_get_value(pni->ven_gpio), gpio_get_value(pni->firm_gpio));
			if(!regulator_is_enabled(nfc_vreg_l6)) {
				ret = regulator_enable(nfc_vreg_l6);
				I("%s : vreg_l16 regulator_enable\n", __func__);
				I("%s : vreg_l16 regulator_is_enabled = %d\n", __func__, regulator_is_enabled(nfc_vreg_l6));
				if (ret < 0) {
					E("%s : vreg_l16 regulator_enable fail\n", __func__);
				}
			}
		}
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		E("%s:  Error: retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static void pn551_disable_irq(struct pn551_dev *pn551_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn551_dev->irq_enabled_lock, flags);
	if (pn551_dev->irq_enabled) {
		disable_irq_nosync(pn551_dev->client->irq);
		pn551_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn551_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn551_dev_irq_handler(int irq, void *dev_id)
{
	struct pn551_dev *pn551_dev = dev_id;
	static unsigned long orig_jiffies = 0;

	if (gpio_get_value(pn551_dev->irq_gpio) == 0) {
		I("%s: irq_workaround PN551\n", __func__);
		return IRQ_HANDLED;
	}

	pn551_disable_irq(pn551_dev);

	/* Wake up waiting readers */
	wake_up(&pn551_dev->read_wq);

	if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(1000)))
		I("%s: irq=%d\n", __func__, irq);
	orig_jiffies = jiffies;

	return IRQ_HANDLED;
}

static void pn551_Enable(void)
{
	struct pn551_dev *pni = pn_info;
	unsigned int set_value = pni->ven_enable;
	I("%s: gpio=%d set_value=%d\n", __func__, pni->ven_gpio, set_value);

	gpio_set_value(pni->ven_gpio, set_value);
	I("%s: [NFC] gpio=%d check ven_gpio value=%d\n", __func__, pni->ven_gpio, gpio_get_value(pni->ven_gpio));
	pni->ven_value = 1;
}

static void pn551_Disable(void)
{
	struct pn551_dev *pni = pn_info;
	unsigned int set_value = !pni->ven_enable;
	I("%s: gpio=%d set_value=%d\n", __func__, pni->ven_gpio, set_value);

	gpio_set_value(pni->ven_gpio, set_value);
	I("%s: [NFC] gpio=%d check ven_gpio value=%d\n", __func__, pni->ven_gpio, gpio_get_value(pni->ven_gpio));

	pni->ven_value = 0;
}


static int pn551_isEn(void)
{
	struct pn551_dev *pni = pn_info;
	return pni->ven_value;
}

uint8_t read_buffer[MAX_BUFFER_SIZE];

static ssize_t pn551_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn551_dev *pni = pn_info;
	int ret;
	int val;
	int i;
	i = 0;

	D("%s: start count = %zu\n", __func__, count);

	if (count > MAX_BUFFER_SIZE) {
		E("%s : count =%zu> MAX_BUFFER_SIZE\n", __func__, count);
		count = MAX_BUFFER_SIZE;
	}

	val = gpio_get_value(pni->irq_gpio);

	D("%s: reading %zu bytes, irq_gpio = %d\n",
		__func__, count, val);

	mutex_lock(&pni->read_mutex);

	if (!gpio_get_value(pni->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			I("%s : f_flags & O_NONBLOCK read again\n", __func__);
			ret = -EAGAIN;
			goto fail;
		}

		pni->irq_enabled = true;
		enable_irq(pni->client->irq);
		D("%s: waiting read-event INT, because "
			"irq_gpio = 0\n", __func__);
		pni->isReadBlock = true;
		ret = wait_event_interruptible(pni->read_wq,
				gpio_get_value(pni->irq_gpio));

		pn551_disable_irq(pni);

		D("%s : wait_event_interruptible done\n", __func__);

		if (ret) {
			I("pn551_dev_read wait_event_interruptible breaked ret=%d\n", ret);
			goto fail;
		}

	}

	pni->isReadBlock = false;
    wake_lock_timeout(&pni ->io_wake_lock, IO_WAKE_LOCK_TIMEOUT);
	/* Read data */
	memset(read_buffer, 0, MAX_BUFFER_SIZE);
	ret = pn551_RxData(read_buffer, count);
	mutex_unlock(&pni->read_mutex);

	if (ret < 0) {
		E("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		E("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}

	DBUF(read_buffer, count);

	if (copy_to_user(buf, read_buffer, count)) {
		E("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	D("%s done count = %zu\n", __func__, count);
	return count;

fail:
	mutex_unlock(&pni->read_mutex);
	return ret;
}

static ssize_t pn551_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn551_dev *pni = pn_info;
	char buffer[MAX_BUFFER_SIZE];
	int ret;
	int i;
	i = 0;

	D("%s: start count = %zu\n", __func__, count);
	wake_lock_timeout(&pni ->io_wake_lock, IO_WAKE_LOCK_TIMEOUT);

	if (count > MAX_BUFFER_SIZE) {
		E("%s : count =%zu> MAX_BUFFER_SIZE\n", __func__, count);
		count = MAX_BUFFER_SIZE;
	}

	if ( is_debug && (s_wdcmd_cnt++ < 3))
		I("%s: writing %zu bytes\n",__func__, count);
	else {
		is_debug = is_debug_en;
		s_wdcmd_cnt = 4;
	}

	if (copy_from_user(buffer, buf, count)) {
		E("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	DBUF(buffer, count);

	/* Write data */
	ret = pn551_TxData(buffer, count);
	if (ret < 0) {
		E("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	} else {
		D("%s done count = %zu\n", __func__, count);
		return count;
	}

	return ret;
}

static int pn551_dev_open(struct inode *inode, struct file *filp)
{
	struct pn551_dev *pn551_dev = container_of(filp->private_data,
						struct pn551_dev,
						pn551_device);

	filp->private_data = pn551_dev;
	I("%s : major=%d, minor=%d\n", \
		__func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn551_dev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct pn551_dev *pni = pn_info;

	switch (cmd) {
	case PN551_SET_PWR:
		if (arg == 3) {
			/* Software reset */
			E("%s Useless !!!!! arg=3 for PN551\n", __func__);
		} else if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			I("%s power on with firmware\n", __func__);
			pn551_Enable();
			gpio_set_value(pni->firm_gpio, 1);
			msleep(50);
			pn551_Disable();
			msleep(50);
			pn551_Enable();
			msleep(50);
		} else if (arg == 1) {
			/* power on */
			I("%s power on (delay50)\n", __func__);
			gpio_set_value(pni->firm_gpio, 0);
			pn551_Enable();
			msleep(50);
			is_debug = 1;
			s_wdcmd_cnt = 0;
			I("%s pn551_Enable, set is_debug = %d, s_wdcmd_cnt : %d\n", __func__, is_debug, s_wdcmd_cnt);
		} else  if (arg == 0) {
			/* power off */
			I("%s power off (delay50)\n", __func__);
			gpio_set_value(pni->firm_gpio, 0);
			pn551_Disable();
			msleep(50);
			is_debug = is_debug_en;
			I("%s pn551_Disable, set is_debug = %d, s_wdcmd_cnt = %d\n", __func__, is_debug, s_wdcmd_cnt);
		} else {
			E("%s bad arg %lu\n", __func__, arg);
			goto fail;
		}
		break;
	default:
		E("%s bad ioctl %u for PN551\n", __func__, cmd);
		goto fail;
	}

	return 0;
fail:
	return -EINVAL;
}

static const struct file_operations pn551_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn551_dev_read,
	.write	= pn551_dev_write,
	.open	= pn551_dev_open,
	.unlocked_ioctl = pn551_dev_ioctl,
	.compat_ioctl  = pn551_dev_ioctl,
};


static ssize_t debug_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	I("debug_enable_show\n");

	ret = snprintf(buf, MAX_BUFFER_SIZE*2 , "is_debug=%d\n", is_debug);
	return ret;
}

static ssize_t debug_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	sscanf(buf, "%d", &is_debug_en);
	if(is_debug_en == 0) {
	  is_debug = 0;
	  I("is_debug = 0 log disabled!\n");
	} else {
	  is_debug = 1;
	  is_debug_en = 1;
          I("is_debug = 1 log enabled!\n");
	}
	return count;
}

static DEVICE_ATTR(debug_enable, 0664, debug_enable_show, debug_enable_store);

static int pn551_parse_dt(struct device *dev, struct pn551_i2c_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	I("%s: Start\n", __func__);
	prop = of_find_property(dt, "nxp,ven_isinvert", NULL);
	if (prop) {
		of_property_read_u32(dt, "nxp,ven_isinvert", &pdata->ven_isinvert);
		printk(KERN_INFO "[NFC] %s:ven_isinvert = %d", __func__, pdata->ven_isinvert);
	} else {
		goto parse_error;
	}

	/* irq, ven, firm gpio info */
	I("%s: chk irq-gpio\n", __func__);
	pdata->irq_gpio = of_get_named_gpio_flags(dt, "nxp,irq-gpio",
                                0, &pdata->irq_gpio_flags);

	if(!gpio_is_valid(pdata->irq_gpio)) {
		I("%s: chk irq-gpio fail\n", __func__);
		goto parse_error;
	}

	I("%s: chk ven-gpio\n", __func__);
	pdata->ven_gpio = of_get_named_gpio_flags(dt, "nxp,ven-gpio",
				0, &pdata->ven_gpio_flags);
	if(!gpio_is_valid(pdata->ven_gpio)) {
		I("%s: chk nxp,ven-gpio fail\n", __func__);
		goto parse_error;
	}

	I("%s: chk fwdl-gpio\n", __func__);
	pdata->firm_gpio = of_get_named_gpio_flags(dt, "nxp,fwdl-gpio",
				0, &pdata->firm_gpio_flags);
	if(!gpio_is_valid(pdata->firm_gpio)) {
		I("%s: chk nxp,fwdl-gpio fail\n", __func__);
		goto parse_error;
	}
	I("%s: chk nfc_i2c pins\n", __func__);
	NFC_I2C_SCL = of_get_named_gpio_flags(dt, "nfc_i2c_scl", 0, NULL);
	if(!gpio_is_valid(NFC_I2C_SCL)) {
		I("%s: chk nxp, nfc_i2c_scl fail\n", __func__);
		goto parse_error;
	}
	NFC_I2C_SDA = of_get_named_gpio_flags(dt, "nfc_i2c_sda", 0, NULL);
	if(!gpio_is_valid(NFC_I2C_SDA)) {
		I("%s: chk nxp, nfc_i2c_sda fail\n", __func__);
		goto parse_error;
	}

	I("%s: End, irq_gpio:%d, ven_gpio:%d, firm_gpio:%d\n", __func__, pdata->irq_gpio, pdata->ven_gpio,pdata->firm_gpio);

	return 0;
parse_error:
	return 1;
}

void pn551_power_off_sequence(void)
{
	int ret;
	printk(KERN_INFO "[NFC] %s ++\n", __func__);
	ret = gpio_request(NFC_I2C_SCL , "nfc_i2c_scl");
	if(ret) {
		E("%s request scl error\n",__func__);
	}
	ret = gpio_request(NFC_I2C_SDA , "nfc_i2c_sda");
	if(ret){
		E("%s request sda error\n",__func__);
	}

	ret = gpio_direction_output(NFC_I2C_SCL, 0);
	I("%s : NFC_I2C_SCL set 0 %d \n", __func__,ret);
	mdelay(1);
	ret = gpio_direction_output(NFC_I2C_SDA, 0);
	I("%s : NFC_I2C_SDA set 0 %d \n", __func__,ret);
	mdelay(50);
	printk(KERN_INFO "[NFC] %s --\n", __func__);
}

static int pn551_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct pn551_i2c_platform_data *platform_data;
	struct pn551_dev *pni;

	I("%s:\n", __func__);
	nfc_vreg_l6 = regulator_get(&client->dev, "pm8994_l6");
	I("%s : vreg_l6 regulator_get\n", __func__);
	if (nfc_vreg_l6< 0) {
		E("%s : vreg_l6 regulator_get fail\n", __func__);
		return -ENODEV;
	}
	ret = regulator_set_voltage(nfc_vreg_l6, 1800000, 1800000);
	I("%s : vreg_l6 regulator_set_voltage\n", __func__);
	if (ret < 0) {
		E("%s : vreg_l6 regulator_set_voltage fail\n", __func__);
		return -ENODEV;
	}
	ret = regulator_enable(nfc_vreg_l6);
	I("%s : vreg_l6 regulator_enable\n", __func__);
	I("%s : vreg_l6 regulator_is_enabled = %d\n", __func__, regulator_is_enabled(nfc_vreg_l6));
	if (ret < 0) {
		E("%s : vreg_l6 regulator_enable fail\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	if (client->dev.of_node) {
		 platform_data = kzalloc(sizeof(*platform_data), GFP_KERNEL);
		 if (platform_data == NULL) {
			 E("%s : nfc probe fail because platform_data \
				 is NULL\n", __func__);
			 return  -ENODEV;
		 }
		 ret = pn551_parse_dt(&client->dev, platform_data);
		 if (ret) {
			E("%s : pn551_parse_dt fail\n", __func__);
			ret = -ENODEV;
			goto err_exit;
		 }
	} else {
		 platform_data = client->dev.platform_data;
		 if (platform_data == NULL) {
			 E("%s : nfc probe fail because platform_data \
				 is NULL\n", __func__);
			 return  -ENODEV;
		 }
	}

	/* IRQ_GPIO */
	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret) {
		E("%s : request gpio%d fail\n",
			__func__, platform_data->irq_gpio);
		ret = -ENODEV;
		goto err_exit;
	}

	/* NFC_EN GPIO */
	ret = gpio_request(platform_data->ven_gpio, "nfc_en");
	if (ret) {
		E("%s : request gpio %d fail\n",
			__func__, platform_data->ven_gpio);
		ret = -ENODEV;
		goto err_request_gpio_ven;
	}
	/* NFC_FIRM GPIO */

	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
	if (ret) {
		E("%s : request gpio %d fail\n",
			__func__, platform_data->firm_gpio);
		ret = -ENODEV;
		goto err_request_gpio_firm;
	}
	pni = kzalloc(sizeof(struct pn551_dev), GFP_KERNEL);
	if (pni == NULL) {
		dev_err(&client->dev, \
				"pn551_probe : failed to allocate \
				memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	pn_info = pni;

	if (platform_data->gpio_init != NULL) {
		I("%s: gpio_init\n", __func__);
		platform_data->gpio_init();
	}

	pni->irq_gpio = platform_data->irq_gpio;
	pni->ven_gpio  = platform_data->ven_gpio;
	pni->firm_gpio  = platform_data->firm_gpio;
	pni->client   = client;
	pni->gpio_init = platform_data->gpio_init;
	pni->ven_enable = !platform_data->ven_isinvert;
	pni->isReadBlock = false;
	I("%s : irq_gpio:%d, ven_gpio:%d, firm_gpio:%d, ven_enable:%d\n", __func__, pni->irq_gpio, pni->ven_gpio, pni->firm_gpio, pni->ven_enable);

	ret = gpio_direction_input(pni->irq_gpio);
	I("%s : irq_gpio set input %d \n", __func__,ret);
	ret = gpio_direction_output(pni->ven_gpio, 0);
	I("%s : ven_gpio set 0 %d \n", __func__,ret);
	ret = gpio_direction_output(pni->firm_gpio, 0);
	I("%s : firm_gpio set 0 %d \n", __func__,ret);

	/* init mutex and queues */
	init_waitqueue_head(&pni->read_wq);
	mutex_init(&pni->read_mutex);
	spin_lock_init(&pni->irq_enabled_lock);

	I("%s: init io_wake_lock\n", __func__);
	wake_lock_init(&pni->io_wake_lock, WAKE_LOCK_SUSPEND, PN551_I2C_NAME);

	pni->pn551_device.minor = MISC_DYNAMIC_MINOR;
	pni->pn551_device.name = "pn551";
	pni->pn551_device.fops = &pn551_dev_fops;
	ret = misc_register(&pni->pn551_device);
	if (ret) {
		E("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}


	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	client->irq = gpio_to_irq(platform_data->irq_gpio);
	I("%s : requesting IRQ %d\n", __func__, client->irq);

	pni->irq_enabled = true;
	ret = request_irq(client->irq, pn551_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pni);
	if (ret) {
		dev_err(&client->dev, "pn551_probe : request_irq failed\n");
		goto err_request_irq_failed;
	}
	pn551_disable_irq(pni);
	i2c_set_clientdata(client, pni);

	pni->pn551_class = class_create(THIS_MODULE, "NFC_sensor");
	if (IS_ERR(pni->pn551_class)) {
		ret = PTR_ERR(pni->pn551_class);
		pni->pn551_class = NULL;
		E("%s : class_create failed\n", __func__);
		goto err_create_class;
	}

	pni->pn_dev = device_create(pni->pn551_class, NULL, 0, "%s", "pn551");
	if (unlikely(IS_ERR(pni->pn_dev))) {
		ret = PTR_ERR(pni->pn_dev);
		pni->pn_dev = NULL;
		E("%s : device_create failed\n", __func__);
		goto err_create_pn_device;
	}

	/* register the attributes */

	ret = device_create_file(pni->pn_dev, &dev_attr_debug_enable);
	if (ret) {
		E("pn551_probe device_create_file dev_attr_debug_enable failed\n");
		goto err_create_pn_file;
	}

    I("%s: Turn off NFC VEN by default\n", __func__);
        pn551_Disable();

	I("%s: Probe success!\n", __func__);
	return 0;

err_create_pn_file:
	device_unregister(pni->pn_dev);
err_create_pn_device:
	class_destroy(pni->pn551_class);
err_create_class:
err_request_irq_failed:
	misc_deregister(&pni->pn551_device);
err_misc_register:
	mutex_destroy(&pni->read_mutex);
	wake_lock_destroy(&pni->io_wake_lock);
	kfree(pni);
	pn_info = NULL;
err_kzalloc:
	gpio_free(platform_data->firm_gpio);
err_request_gpio_firm:
	gpio_free(platform_data->ven_gpio);
err_request_gpio_ven:
	gpio_free(platform_data->irq_gpio);
err_exit:
	kfree(platform_data);
	E("%s: prob fail\n", __func__);
	return ret;
}

static void pn551_shutdown(struct i2c_client *client) {
	pn551_power_off_sequence();
}

static int pn551_remove(struct i2c_client *client)
{
	struct pn551_dev *pn551_dev;
	I("%s:\n", __func__);

	pn551_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn551_dev);
	misc_deregister(&pn551_dev->pn551_device);
	mutex_destroy(&pn551_dev->read_mutex);
	wake_lock_destroy(&pn551_dev->io_wake_lock);
	gpio_free(pn551_dev->irq_gpio);
	gpio_free(pn551_dev->ven_gpio);
	gpio_free(pn551_dev->firm_gpio);
	kfree(pn551_dev);
	pn_info = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int pn551_suspend(struct i2c_client *client, pm_message_t state)
{
	struct pn551_dev *pni = pn_info;

        I("%s: irq = %d, ven_gpio = %d, isEn = %d, isReadBlock =%d\n", __func__, \
                gpio_get_value(pni->irq_gpio), gpio_get_value(pni->ven_gpio), pn551_isEn(), pni->isReadBlock);

	if (pni->ven_value && pni->isReadBlock) {
		pni->irq_enabled = true;
		enable_irq(pni->client->irq);
		irq_set_irq_wake(pni->client->irq, 1);
	}

	return 0;
}

static int pn551_resume(struct i2c_client *client)
{
	struct pn551_dev *pni = pn_info;

        I("%s: irq = %d, ven_gpio = %d, isEn = %d, isReadBlock =%d\n", __func__, \
                gpio_get_value(pni->irq_gpio), gpio_get_value(pni->ven_gpio), pn551_isEn(), pni->isReadBlock);

	if (pni->ven_value && pni->isReadBlock) {
		pn551_disable_irq(pni);
		irq_set_irq_wake(pni->client->irq, 0);
	}

	return 0;
}
#endif

static const struct i2c_device_id pn551_id[] = {
	{ "pn551", 0 },
	{ }
};

static struct of_device_id pn551_match_table[] = {
	{ .compatible = "nxp,pn551-nfc",},
	{ },
};

static struct i2c_driver pn551_driver = {
	.id_table	= pn551_id,
	.probe		= pn551_probe,
	.remove		= pn551_remove,
	.shutdown = pn551_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn551",
		.of_match_table = pn551_match_table,
	},
#if CONFIG_PM
	.suspend	= pn551_suspend,
	.resume		= pn551_resume,
#endif
};

/*
 * module load/unload record keeping
 */

static int __init pn551_dev_init(void)
{
	I("%s: Loading pn551 driver\n", __func__);
	return i2c_add_driver(&pn551_driver);
}
module_init(pn551_dev_init);

static void __exit pn551_dev_exit(void)
{
	I("%s: Unloading pn551 driver\n", __func__);
	i2c_del_driver(&pn551_driver);
}
module_exit(pn551_dev_exit);

MODULE_AUTHOR("HTC SSD NFC");
MODULE_DESCRIPTION("NFC PN551 driver");
MODULE_LICENSE("GPL");
