/*
 * Copyright (C) 2016 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
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
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
#include <linux/clk.h>
#endif
#include "st21nfc.h"

#define MAX_BUFFER_SIZE 260

#define DRIVER_VERSION "2.0.2"

/* prototypes */
static irqreturn_t st21nfc_dev_irq_handler(int irq, void *dev_id);
/*
 * The platform data member 'polarity_mode' defines
 * how the wakeup pin is configured and handled.
 * it can take the following values :
 *   IRQF_TRIGGER_RISING
 *   IRQF_TRIGGER_HIGH
 */

struct st21nfc_platform {
	struct mutex read_mutex;
	struct i2c_client *client;
	unsigned int irq_gpio;
	unsigned int reset_gpio;
	unsigned int ena_gpio;
#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
	unsigned int clkreq_gpio;
	uint8_t pinctrl_en;
#endif
	unsigned int polarity_mode;
};

static bool irqIsAttached;

static bool device_open; /* Is device open? */

struct st21nfc_dev {
	wait_queue_head_t read_wq;
	struct miscdevice st21nfc_device;
	bool irq_enabled;
	struct st21nfc_platform platform_data;
	spinlock_t irq_enabled_lock;
	uint8_t rx_buffer[MAX_BUFFER_SIZE];
	uint8_t tx_buffer[MAX_BUFFER_SIZE];
#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
	/* CLK control */
	bool                    clk_run;
	struct  clk             *s_clk;
#endif
};

#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
/*
 * Routine to enable clock.
 * this routine can be extended to select from multiple
 * sources based on clk_src_name.
 */
static int st_clock_select(struct st21nfc_dev *st21nfc_dev)
{
	int r = 0;

	st21nfc_dev->s_clk = clk_get(&st21nfc_dev->platform_data.client->dev, "nfc_ref_clk");

	/* if NULL we assume external crystal and dont fail */
	if ((st21nfc_dev->s_clk == NULL) || IS_ERR(st21nfc_dev->s_clk))
		return 0;

	if (st21nfc_dev->clk_run == false)
		r = clk_prepare_enable(st21nfc_dev->s_clk);

	if (r)
		goto err_clk;

	st21nfc_dev->clk_run = true;
	return r;

err_clk:
	return -1;
}

/*
 * Routine to disable clocks
 */
static int st_clock_deselect(struct st21nfc_dev *st21nfc_dev)
{
	int r = -1;

	/* if NULL we assume external crystal and dont fail */
	if ((st21nfc_dev->s_clk == NULL) || IS_ERR(st21nfc_dev->s_clk))
		return 0;

	if ((st21nfc_dev->s_clk != NULL) && !IS_ERR(st21nfc_dev->s_clk)) {
		if (st21nfc_dev->clk_run == true) {
			clk_disable_unprepare(st21nfc_dev->s_clk);
			st21nfc_dev->clk_run = false;
		}
		return 0;
	}
	return r;
}
#endif

static int st21nfc_loc_set_polaritymode(struct st21nfc_dev *st21nfc_dev,
					int mode)
{
	struct i2c_client *client = st21nfc_dev->platform_data.client;
	unsigned int irq_type;
	int ret;

	st21nfc_dev->platform_data.polarity_mode = mode;
	/* setup irq_flags */
	switch (mode) {
	case IRQF_TRIGGER_RISING:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	case IRQF_TRIGGER_HIGH:
		irq_type = IRQ_TYPE_LEVEL_HIGH;
		break;
	default:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	}
	if (irqIsAttached) {
		free_irq(client->irq, st21nfc_dev);
		irqIsAttached = false;
	}
	ret = irq_set_irq_type(client->irq, irq_type);
	if (ret) {
		pr_err("%s : set_irq_type failed\n", __FILE__);
		return -ENODEV;
	}
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_debug("%s : requesting IRQ %d\n", __func__, client->irq);
	st21nfc_dev->irq_enabled = true;

	ret = request_irq(client->irq, st21nfc_dev_irq_handler,
			  st21nfc_dev->platform_data.polarity_mode,
			  client->name, st21nfc_dev);
	if (!ret)
		irqIsAttached = true;

	return ret;
}


static void st21nfc_disable_irq(struct st21nfc_dev *st21nfc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&st21nfc_dev->irq_enabled_lock, flags);
	if (st21nfc_dev->irq_enabled) {
		pr_debug("%s : IRQ %d\n", __func__,
				st21nfc_dev->platform_data.client->irq);
		disable_irq_nosync(st21nfc_dev->platform_data.client->irq);
		st21nfc_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&st21nfc_dev->irq_enabled_lock, flags);
}

static irqreturn_t st21nfc_dev_irq_handler(int irq, void *dev_id)
{
	struct st21nfc_dev *st21nfc_dev = dev_id;

	pr_debug("%s : enter\n", __func__);
	st21nfc_disable_irq(st21nfc_dev);

	/* Wake up waiting readers */
	wake_up(&st21nfc_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t st21nfc_dev_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct st21nfc_dev *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_dev,
						       st21nfc_device);
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_debug("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&st21nfc_dev->platform_data.read_mutex);

	/* Read data */
	ret = i2c_master_recv(st21nfc_dev->platform_data.client,
				st21nfc_dev->rx_buffer, count);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		mutex_unlock(&st21nfc_dev->platform_data.read_mutex);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
		       __func__, ret);
		mutex_unlock(&st21nfc_dev->platform_data.read_mutex);
		return -EIO;
	}
	if (copy_to_user(buf, st21nfc_dev->rx_buffer, ret)) {
		pr_warn("%s : failed to copy to user space\n", __func__);
		mutex_unlock(&st21nfc_dev->platform_data.read_mutex);
		return -EFAULT;
	}
	mutex_unlock(&st21nfc_dev->platform_data.read_mutex);
	return ret;
}

static ssize_t st21nfc_dev_write(struct file *filp, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct st21nfc_dev *st21nfc_dev;
	int ret = count;

	st21nfc_dev = container_of(filp->private_data,
				   struct st21nfc_dev, st21nfc_device);
	pr_debug("%s: st21nfc_dev ptr %p\n", __func__, st21nfc_dev);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	// Writes to i2c are serialized by the HAL layer and therefore doesn't
	// need to be thread safe.
	if (copy_from_user(st21nfc_dev->tx_buffer, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(st21nfc_dev->platform_data.client,
				st21nfc_dev->tx_buffer, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	return ret;
}

static int st21nfc_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct st21nfc_dev *st21nfc_dev = NULL;

	if (device_open) {
		ret = -EBUSY;
		pr_err("%s : device already opened ret= %d\n", __func__, ret);
	} else {
		device_open = true;
		st21nfc_dev = container_of(filp->private_data,
					struct st21nfc_dev,
					st21nfc_device);
		pr_debug("%s : device_open = %d", __func__, device_open);
		pr_debug("%s : %d,%d ", __func__, imajor(inode), iminor(inode));
		pr_debug("%s: st21nfc_dev ptr %p\n", __func__, st21nfc_dev);
	}
	return ret;
}


static int st21nfc_release(struct inode *inode, struct file *file)
{
	device_open = false;
	pr_debug("%s : device_open  = %d\n", __func__, device_open);

	return 0;
}

static long st21nfc_dev_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct st21nfc_dev *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_dev,
						       st21nfc_device);

	int ret = 0;

	pr_info("%s cmd=%d", __func__, cmd);

	switch (cmd) {

	case ST21NFC_SET_POLARITY_RISING:
		pr_info(" ### ST21NFC_SET_POLARITY_RISING ###");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_RISING);
		break;

	case ST21NFC_SET_POLARITY_HIGH:
		pr_info(" ### ST21NFC_SET_POLARITY_HIGH ###");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_HIGH);
		break;

	case ST21NFC_PULSE_RESET:
#if 0
		pr_info("%s Pulse Request\n", __func__);
		if (st21nfc_dev->platform_data.reset_gpio != 0) {
			/* pulse low for 20 millisecs */
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					0);
			msleep(20);
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					1);
			pr_info("%s done Pulse Request\n", __func__);
		}
		break;
#else
		/* Double pulse is done to exit Quick boot mode.*/
		pr_info("%s Double Pulse Request\n", __func__);
		if (st21nfc_dev->platform_data.reset_gpio != 0) {
			/* pulse low for 20 millisecs */
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					0);
			msleep(20);
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					1);
			msleep(10);
			/* pulse low for 20 millisecs */
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					0);
			msleep(20);
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					1);
			pr_info("%s done Double Pulse Request\n", __func__);
		}
		break;
#endif

	case ST21NFC_GET_WAKEUP:
		/* deliver state of Wake_up_pin as return value of ioctl */
		ret = gpio_get_value(st21nfc_dev->platform_data.irq_gpio);
		/*
		 * Warning: depending on gpio_get_value implementation,
		 * it can returns a value different than 1 in case of high level
		 */
		if (ret > 0)
			ret = 1;
		else
			ret = 0;
		pr_debug("%s get gpio result %d\n", __func__, ret);
		break;
	case ST21NFC_GET_POLARITY:
		ret = st21nfc_dev->platform_data.polarity_mode;
		pr_debug("%s get polarity %d\n", __func__, ret);
		break;
	case ST21NFC_RECOVERY:
		/* For ST21NFCD usage only */
		pr_info("%s Recovery Request\n", __func__);
		if (st21nfc_dev->platform_data.reset_gpio != 0) {
			/* pulse low for 20 millisecs */
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					0);
			msleep(10);
			/*
			 * During the reset, force IRQ OUT as
			 * DH output instead of input in normal usage
			 */
			ret = gpio_direction_output(st21nfc_dev->platform_data.irq_gpio, 1);
			if (ret) {
				pr_err("%s : gpio_direction_output failed\n",
						__FILE__);
				ret = -ENODEV;
				break;
			}
			gpio_set_value(st21nfc_dev->platform_data.irq_gpio,
					1);
			msleep(10);
			gpio_set_value(st21nfc_dev->platform_data.reset_gpio,
					1);
			pr_info("%s done Pulse Request\n", __func__);
		}
		msleep(20);
		gpio_set_value(st21nfc_dev->platform_data.irq_gpio, 0);
		msleep(20);
		gpio_set_value(st21nfc_dev->platform_data.irq_gpio, 1);
		msleep(20);
		gpio_set_value(st21nfc_dev->platform_data.irq_gpio, 0);
		msleep(20);
		pr_info("%s Recovery procedure finished\n", __func__);
		ret = gpio_direction_input(st21nfc_dev->platform_data.irq_gpio);
		if (ret) {
			pr_err("%s : gpio_direction_input failed\n", __FILE__);
			ret = -ENODEV;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static unsigned int st21nfc_poll(struct file *file, poll_table *wait)
{
	struct st21nfc_dev *st21nfc_dev = container_of(file->private_data,
						       struct st21nfc_dev,
						       st21nfc_device);
	unsigned int mask = 0;
	int pinlev = 0;

	/* wait for Wake_up_pin == high  */
	poll_wait(file, &st21nfc_dev->read_wq, wait);

	pinlev = gpio_get_value(st21nfc_dev->platform_data.irq_gpio);

	if (pinlev > 0) {
		pr_debug("%s return ready\n", __func__);
		mask = POLLIN | POLLRDNORM;	/* signal data avail */
		st21nfc_disable_irq(st21nfc_dev);
	} else {
		/* Wake_up_pin is low. Activate ISR  */
		if (!st21nfc_dev->irq_enabled) {
			pr_debug("%s enable irq\n", __func__);
			st21nfc_dev->irq_enabled = true;
			enable_irq(st21nfc_dev->platform_data.client->irq);
		} else {
			pr_debug("%s irq already enabled\n", __func__);
		}
	}
	return mask;
}

static const struct file_operations st21nfc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = st21nfc_dev_read,
	.write = st21nfc_dev_write,
	.open = st21nfc_dev_open,
	.poll = st21nfc_poll,
	.release = st21nfc_release,

	.unlocked_ioctl = st21nfc_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = st21nfc_dev_ioctl
#endif
};

/* st21nfc_show_i2c_addr() */
static ssize_t st21nfc_show_i2c_addr(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client != NULL)
		return sprintf(buf, "0x%.2x\n", client->addr);
	return 0;
}

/* st21nfc_change_i2c_addr() */
static ssize_t st21nfc_change_i2c_addr(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct st21nfc_dev *data = dev_get_drvdata(dev);
	long new_addr = 0;

	if (data != NULL && data->platform_data.client != NULL) {
		if (!kstrtol(buf, 10, &new_addr)) {
			mutex_lock(&data->platform_data.read_mutex);
			data->platform_data.client->addr = new_addr;
			mutex_unlock(&data->platform_data.read_mutex);
			return count;
		}
		return -EINVAL;
	}
	return 0;
}

/* st21nfc_version */
static ssize_t st21nfc_version(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_VERSION);
}

static DEVICE_ATTR(i2c_addr, S_IRUGO | S_IWUSR, st21nfc_show_i2c_addr,
		   st21nfc_change_i2c_addr);

static DEVICE_ATTR(version, S_IRUGO, st21nfc_version, NULL);

static struct attribute *st21nfc_attrs[] = {
	&dev_attr_i2c_addr.attr,
	&dev_attr_version.attr,
	NULL,
};

static struct attribute_group st21nfc_attr_grp = {
	.attrs = st21nfc_attrs,
};

#ifdef CONFIG_OF
static int nfc_parse_dt(struct device *dev, struct st21nfc_platform_data *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;

	np = of_find_compatible_node(NULL, NULL, "st,st21nfc");
	if (IS_ERR_OR_NULL(np)) {
		pr_err("[dsc]%s: cannot find compatible node \"%s\"", __func__,
				"st,st21nfc");
		return -ENODEV;
	}

	pdata->reset_gpio = of_get_named_gpio(np, "st,reset_gpio", 0);
	if ((!gpio_is_valid(pdata->reset_gpio))) {
		pr_err("[dsc]%s: fail to get reset_gpio\n", __func__);
		return -EINVAL;
	}
	pdata->irq_gpio = of_get_named_gpio(np, "st,irq_gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio))) {
		pr_err("[dsc]%s: fail to get irq_gpio\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
	pdata->clkreq_gpio = of_get_named_gpio(np, "st,clkreq_gpio", 0);
	if ((!gpio_is_valid(pdata->clkreq_gpio))) {
		pr_err("[dsc]%s: [OPTIONAL] fail to get clkreq_gpio\n", __func__);
	}
	pdata->pinctrl_en = 1;
	if (!of_property_read_bool(np,"st,clk_pinctrl")) {
		pr_info("[dsc]%d:[OPTIONAL] clk_pinctrl not set\n",__func__);
		pdata->pinctrl_en = 0;
	}
#endif

	pdata->polarity_mode = IRQF_TRIGGER_RISING;
	pr_err("[dsc]%s : get reset_gpio[%d], irq_gpio[%d], \
			polarity_mode[%d]\n", __func__, pdata->reset_gpio,
			pdata->irq_gpio, pdata->polarity_mode);
	return r;
}
#else
static int nfc_parse_dt(struct device *dev, struct st21nfc_platform_data *pdata)
{
	return 0;
}
#endif


static int st21nfc_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct st21nfc_platform_data *platform_data;
	struct st21nfc_dev *st21nfc_dev;

	pr_info("__func__\n");

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(struct st21nfc_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev,
					"nfc-nci prob: Failed to allocate memory\n");
			return -ENOMEM;
		}
		pr_info("%s : Parse st21nfc DTS\n", __func__);
		ret = nfc_parse_dt(&client->dev, platform_data);
		if (ret)
			return ret;
	} else {
		platform_data = client->dev.platform_data;
		pr_info("%s : No st21nfc DTS\n", __func__);
	}
	if (!platform_data)
		return -EINVAL;
	dev_dbg(&client->dev, "nfc-nci probe: %s, inside nfc-nci flags = %x\n",
			__func__, client->flags);

	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc-nci probe: failed\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	client->adapter->timeout = msecs_to_jiffies(3 * 10);  /* 30ms */
	client->adapter->retries = 0;

	st21nfc_dev = kzalloc(sizeof(*st21nfc_dev), GFP_KERNEL);
	if (st21nfc_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pr_debug("%s : dev_cb_addr %p\n", __func__, st21nfc_dev);

	/* store for later use */
	st21nfc_dev->platform_data.irq_gpio = platform_data->irq_gpio;
#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
	st21nfc_dev->platform_data.clkreq_gpio = platform_data->clkreq_gpio;
	st21nfc_dev->platform_data.pinctrl_en = platform_data->pinctrl_en;
#endif
	st21nfc_dev->platform_data.ena_gpio = platform_data->ena_gpio;
	st21nfc_dev->platform_data.reset_gpio = platform_data->reset_gpio;
	st21nfc_dev->platform_data.polarity_mode = platform_data->polarity_mode;
	st21nfc_dev->platform_data.client = client;

	ret = gpio_request(platform_data->irq_gpio, "irq_gpio");
	if (ret) {
		pr_err("%s : gpio_request failed\n", __FILE__);
		ret = -ENODEV;
		goto err_free_buffer;
	}

	ret = gpio_direction_input(platform_data->irq_gpio);
	if (ret) {
		pr_err("%s : gpio_direction_input failed\n", __FILE__);
		ret = -ENODEV;
		goto err_free_buffer;
	}

#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
	ret = gpio_request(platform_data->clkreq_gpio, "clkreq_gpio");
	if (ret) {
		pr_err("%s : [OPTIONAL] gpio_request failed\n", __FILE__);
		ret = 0;
	} else {
		ret = gpio_direction_input(platform_data->clkreq_gpio);
		if (ret) {
			pr_err("%s : [OPTIONAL] gpio_direction_input failed\n", __FILE__);
			ret = 0;
		}
	}
#endif

	/* initialize irqIsAttached variable */
	irqIsAttached = false;

	/* initialize device_open variable */
	device_open = 0;

	/* handle optional RESET */
	if (platform_data->reset_gpio != 0) {
		ret = gpio_request(platform_data->reset_gpio, "reset_gpio");
		if (ret) {
			pr_err("%s : reset gpio_request failed\n", __FILE__);
			ret = -ENODEV;
			goto err_free_buffer;
		}
		ret = gpio_direction_output(platform_data->reset_gpio, 1);
		if (ret) {
			pr_err("%s : reset gpio_direction_output failed\n",
					__FILE__);
			ret = -ENODEV;
			goto err_free_buffer;
		}
		/* low active */
		gpio_set_value(st21nfc_dev->platform_data.reset_gpio, 1);
	}

	client->irq = gpio_to_irq(platform_data->irq_gpio);

	enable_irq_wake(client->irq);
	/* init mutex and queues */
	init_waitqueue_head(&st21nfc_dev->read_wq);
	mutex_init(&st21nfc_dev->platform_data.read_mutex);
	spin_lock_init(&st21nfc_dev->irq_enabled_lock);
	pr_debug("%s : debug irq_gpio = %d, client-irq =  %d\n", __func__,
			platform_data->irq_gpio, client->irq);
	st21nfc_dev->st21nfc_device.minor = MISC_DYNAMIC_MINOR;
	st21nfc_dev->st21nfc_device.name = "st21nfc";
	st21nfc_dev->st21nfc_device.fops = &st21nfc_dev_fops;
	st21nfc_dev->st21nfc_device.parent = &client->dev;

	i2c_set_clientdata(client, st21nfc_dev);
	ret = misc_register(&st21nfc_dev->st21nfc_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	if (sysfs_create_group(&client->dev.kobj, &st21nfc_attr_grp)) {
		pr_err("%s : sysfs_create_group failed\n", __FILE__);
		goto err_request_irq_failed;
	}
	st21nfc_disable_irq(st21nfc_dev);

#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
	/* check if clock pinctrl already enabled */
	if (platform_data->pinctrl_en != 0)
		st21nfc_dev->clk_run = true;
	ret = st_clock_select(st21nfc_dev);
	if (ret < 0) {
		pr_err("%s : st_clock_select failed\n", __FILE__);
		goto err_request_irq_failed;
	}
	pr_info("%s:successfully\n", __func__);
#endif
	return 0;

err_request_irq_failed:
	misc_deregister(&st21nfc_dev->st21nfc_device);
err_misc_register:
	mutex_destroy(&st21nfc_dev->platform_data.read_mutex);
err_free_buffer:
	kfree(st21nfc_dev);
err_exit:
	gpio_free(platform_data->irq_gpio);
	if (platform_data->ena_gpio != 0)
		gpio_free(platform_data->ena_gpio);
	return ret;
}

static int st21nfc_remove(struct i2c_client *client)
{
	struct st21nfc_dev *st21nfc_dev;

	st21nfc_dev = i2c_get_clientdata(client);
#ifdef CONFIG_NFC_ST21NFC_NO_CRYSTAL
	st_clock_deselect(st21nfc_dev);
#endif
	free_irq(client->irq, st21nfc_dev);
	misc_deregister(&st21nfc_dev->st21nfc_device);
	mutex_destroy(&st21nfc_dev->platform_data.read_mutex);
	gpio_free(st21nfc_dev->platform_data.irq_gpio);
	if (st21nfc_dev->platform_data.ena_gpio != 0)
		gpio_free(st21nfc_dev->platform_data.ena_gpio);
	kfree(st21nfc_dev);

	return 0;
}

static struct i2c_device_id st21nfc_id[] = {
	{"st21nfc", 0},
	{}
};

static struct of_device_id st21nfc_of_match[] = {
	{ .compatible = "st,st21nfc", },
	{}
};

// MODULE_DEVICE_TABLE(of, st21nfc_of_match);
static struct i2c_driver st21nfc_driver = {
	.id_table = st21nfc_id,
	.driver = {
		.name	= "st21nfc",
		.owner	= THIS_MODULE,
		.of_match_table	= st21nfc_of_match,
	},
	.probe		= st21nfc_probe,
	.remove		= st21nfc_remove,
};

#if 0
static struct of_device_id st21nfc_board_of_match[] = {
	{ .compatible = "st,st21nfc", },
	{}
};

static int st21nfc_platform_probe(struct platform_device *pdev)
{
	pr_info("__func__ pr_info\n");
	pr_err("__func__ pr_err\n");
	return 0;
}

static int st21nfc_platform_remove(struct platform_device *pdev)
{
	pr_err("__func__\n");
	return 0;
}


static struct platform_driver st21nfc_platform_driver = {
	.probe = st21nfc_platform_probe,
	.remove = st21nfc_platform_remove,
	.driver = {
		.name	= "st21nfc",
		.owner	= THIS_MODULE,
		.of_match_table	= st21nfc_board_of_match,
	},
};
#endif
/*
 * module load/unload record keeping
 */

static int __init st21nfc_dev_init(void)
{
	pr_info("%s: Loading st21nfc driver\n", __func__);
#if 0
	//add by wuling to fix compilation error
	platform_driver_register(&st21nfc_platform_driver);
#endif
	return i2c_add_driver(&st21nfc_driver);
}

module_init(st21nfc_dev_init);

static void __exit st21nfc_dev_exit(void)
{
	pr_debug("Unloading st21nfc driver\n");
	i2c_del_driver(&st21nfc_driver);
}

module_exit(st21nfc_dev_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("NFC ST21NFC driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
