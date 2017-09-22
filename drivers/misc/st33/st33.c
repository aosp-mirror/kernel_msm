/*
 * Copyright (C) 2017 ST Microelectronics S.A.
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
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "st33.h"
#include <linux/of_gpio.h>

#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

//static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define NCI_MAX_SIZE 255
#define NDLC_SIZE 1
#define NDLC_CRC_SIZE 2
#define MAX_BUFFER_SIZE	(NCI_MAX_SIZE + NDLC_SIZE + NDLC_CRC_SIZE)

#define DRIVER_VERSION "1.0.0"

#define DEBUG
#define INFO
#define ERROR

#ifdef DEBUG
#define DBG_MSG(msg...) {pr_info(msg);}
#else
#define DBG_MSG(msg...) {}
#endif
#ifdef INFO
#define INFO_MSG(msg...) {pr_info(msg);}
#else
#define INFO_MSG(msg...) {}
#endif
#ifdef ERROR
#define ERR_MSG(msg...) {pr_err(msg);}
#else
#define ERR_MSG(msg...) {}
#endif

/* prototypes */
static irqreturn_t st33_dev_irq_handler(int irq, void *dev_id);

struct st33_data {
	wait_queue_head_t 	read_wq;
	struct spi_device	*spi;
	struct miscdevice	st33_device;
	bool 				irq_enabled;
	struct mutex 		read_mutex;
	unsigned int 		irq_gpio;
	unsigned int 		pow_gpio;
	unsigned int 		polarity_mode;
	spinlock_t 			irq_lock;
	unsigned char 		*rx_buffer;
    unsigned int        trans_speed;
};

/* Global Variables */
static bool irqIsAttached;
static bool device_open; /* Is device open? */

static int st33_set_polarity_mode(struct st33_data *st33_dev)
{
	struct spi_device *spi = st33_dev->spi;
	unsigned int irq_type;
	unsigned int polarity_mode;
	int ret;

	DBG_MSG("Entry : %s\n",__func__);
	
	irq_type = IRQ_TYPE_EDGE_FALLING;
	polarity_mode = IRQF_TRIGGER_FALLING;

	if (irqIsAttached) {
		free_irq(spi->irq, st33_dev);
		irqIsAttached = false;
	}
	ret = irq_set_irq_type(spi->irq, irq_type);
	if (ret) {
		ERR_MSG("%s : set_irq_type failed (ret=%d)\n", __func__, ret);
		return -ENODEV;
	}
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	DBG_MSG("%s : requesting IRQ %d\n", __func__, spi->irq);
	st33_dev->irq_enabled = true;

	ret = request_irq(spi->irq, st33_dev_irq_handler,
					  polarity_mode, 
					  st33_dev->st33_device.name,
					  st33_dev);

	if (ret) {
		ERR_MSG("%s : request_irq failed (ret=%d)\n", __func__, ret);
		return -ENODEV;
	}
	
	irqIsAttached = true;

	DBG_MSG("Exit : %s\n",__func__);
	
	return ret;
} 

static void st33_disable_irq(struct st33_data *st33_dev)
{
	unsigned long flags;
	DBG_MSG("Entry : %s\n",__func__);
	
	spin_lock_irqsave(&st33_dev->irq_lock, flags);
	if (st33_dev->irq_enabled) {
		disable_irq_nosync(st33_dev->spi->irq);
		st33_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&st33_dev->irq_lock, flags);
	DBG_MSG("Exit : %s\n",__func__);
}

static irqreturn_t st33_dev_irq_handler(int irq, void *dev_id)
{
	struct st33_data *st33_dev = dev_id;

	DBG_MSG("Entry : %s\n",__func__);

	st33_disable_irq(st33_dev);

	/* Wake up waiting readers */
	wake_up(&st33_dev->read_wq);
	
	DBG_MSG("Exit : %s\n",__func__);

	return IRQ_HANDLED;
}

static ssize_t st33_dev_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct st33_data *st33_dev = container_of(filp->private_data,
											struct st33_data,
											st33_device);
	struct spi_message msg;
	struct spi_transfer xfer = {
		.rx_buf = st33_dev->rx_buffer,
		.len = count,
	};
	int ret;

	DBG_MSG("Entry : %s (reading %zu bytes)\n",__func__,count);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	/* Read data */
	mutex_lock(&st33_dev->read_mutex);
    xfer.speed_hz  = st33_dev->trans_speed;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(st33_dev->spi, &msg);
	mutex_unlock(&st33_dev->read_mutex);

	if (ret < 0) {
		ERR_MSG("Exit %s with ERROR: spi_sync returned %d\n", __func__, ret);
		return ret;
	}
	if (copy_to_user(buf, st33_dev->rx_buffer, count)) {
		ERR_MSG("Exit %s with ERROR: failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	
	DBG_MSG("Exit : %s (ret=%d)\n",__func__,ret);
    ret = count;
	return ret;
}

static ssize_t st33_dev_write(struct file *filp, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct st33_data *st33_dev = container_of(filp->private_data,
											struct st33_data,
											st33_device);
	unsigned char buffer[MAX_BUFFER_SIZE];
	int ret = 0;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = buffer,
		.rx_buf = st33_dev->rx_buffer,
		.len = count,
	};

	DBG_MSG("Entry : %s\n",__func__);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(buffer, buf, count)) {
		ERR_MSG("Exit %s with ERROR: failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes (speed=%d).\n", __func__, count,st33_dev->trans_speed);
	/* Write data */
    xfer.speed_hz  = st33_dev->trans_speed;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(st33_dev->spi, &msg);

	if (ret < 0) {
		ERR_MSG("%s ERROR: spi_sync returned %d\n", __func__, ret);
	}
	
	ret = count;
	
	DBG_MSG("Exit : %s (ret=%d)\n",__func__,ret);
	return ret;
}

static int st33_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct st33_data *st33_dev = NULL;
	
	DBG_MSG("Entry : %s\n",__func__);
	
	if (device_open) {
		ret = -EBUSY;
		ERR_MSG("%s : device already opened ret= %d\n", __func__, ret);
	} 
	else {
		device_open = true;
		st33_dev = container_of(filp->private_data,
								struct st33_data,
								st33_device);
		DBG_MSG("%s : device_open = %d", __func__, device_open);
		DBG_MSG("%s : Major No: %d, Minor No: %d ", __func__, imajor(inode), iminor(inode));
		DBG_MSG("%s : st33_dev ptr %p\n", __func__, st33_dev);
	}
	
	DBG_MSG("Exit : %s\n",__func__);
	
	return ret;
}

static int st33_release(struct inode *inode, struct file *file)
{
	device_open = false;
	DBG_MSG("Entry : %s\n",__func__);

	return 0;
}

static long st33_dev_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct st33_data *st33_dev = container_of(filp->private_data,
						       struct st33_data,
						       st33_device);

	int ret = 0;
    unsigned int tmp;

	DBG_MSG("Entry : %s (cmd=%d)\n",__func__,cmd);

	switch (cmd) {

	case ST33_POW_ON:
		INFO_MSG("%s Power On Request\n", __func__);
		gpio_set_value(st33_dev->pow_gpio,1);
		break;

	case ST33_POW_OFF:
		INFO_MSG("%s Power Off Request\n", __func__);
		gpio_set_value(st33_dev->pow_gpio,0);
		break;

	case ST33_GET_SPI_SPEED_HZ:
        INFO_MSG("%s Get SPI Speed request\n", __func__);
		ret = st33_dev->trans_speed;
		break;
    case ST33_SET_SPI_SPEED_HZ:
		ret = __get_user(tmp, (__u32 __user *)arg);
        INFO_MSG("%s Set SPI Speed request (%d Hz)\n", __func__, tmp);
        if (ret == 0) {
			if ( (tmp >= ST33_SPI_FREQ_MIN) && (tmp <= ST33_SPI_FREQ_MAX) ) {
                st33_dev->trans_speed = tmp;
                INFO_MSG("%s Set SPI Speed to %d Hz (max)\n",__func__, tmp);
            }
		}
		break;

	case ST33_GET_IRQ_LEVEL:
		/* deliver state of Wake_up_pin as return value of ioctl */
		ret = gpio_get_value(st33_dev->irq_gpio);
		/* change the result as the polarity is active low */
		if (ret > 0)
			ret = 0;
		else if (ret == 0)
			ret = 1;
		INFO_MSG("%s : gpio level is %d\n", __func__,ret);
		break;

	default:
		ERR_MSG("%s : bad ioctl %u\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned int st33_poll(struct file *filp, poll_table *wait)
{
	struct st33_data *st33_dev = container_of(filp->private_data,
											  struct st33_data,
											  st33_device);
	unsigned int mask = 0;
	int pinlev = 0;

	DBG_MSG("Entry : %s\n",__func__);
	
	/* wait for Wake_up_pin == high  */
	poll_wait(filp, &st33_dev->read_wq, wait);

	pinlev = gpio_get_value(st33_dev->irq_gpio);

	if (pinlev == 0) {
		DBG_MSG("%s detected IRQ pin to 0, disabling IRQ\n", __func__);
		mask = POLLIN | POLLRDNORM;	/* signal data available */
		st33_disable_irq(st33_dev);
	} else {
		/* Wake_up_pin is high. Activate ISR  */
		if (!st33_dev->irq_enabled) {
			DBG_MSG("%s enable irq\n", __func__);
			st33_dev->irq_enabled = true;
			enable_irq(st33_dev->spi->irq);
		} else {
			DBG_MSG("%s irq already enabled\n", __func__);
		}
	}
	
	DBG_MSG("Exit : %s\n",__func__);
	
	return mask;
}

static const struct file_operations st33_dev_fops = {
	.owner 			= THIS_MODULE,
	.write 			= st33_dev_write,
	.read  			= st33_dev_read,
	.unlocked_ioctl	= st33_dev_ioctl,
	.open  			= st33_dev_open,
	.poll 			= st33_poll,
	.release 		= st33_release,
	.llseek 		= no_llseek,
#ifdef CONFIG_COMPAT
	.compat_ioctl 	= st33_dev_ioctl
#endif
};

static int st33_parse_dt(struct device *dev, struct st33_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	
	DBG_MSG("Entry : %s\n",__func__);
	
	np = of_find_compatible_node(NULL, NULL, "st,st33");
	if (IS_ERR_OR_NULL(np)) {
		ERR_MSG("%s: cannot find compatible node \"%s\"", __func__, "st,st33");
		return -ENODEV;
	}
	
	pdata->pow_gpio = of_get_named_gpio(np, "st,pow_gpio", 0);
	if ((!gpio_is_valid(pdata->pow_gpio))) {
		pr_err("[dsc]%s: fail to get pow_gpio\n", __func__);
		return -EINVAL;
	}
	
	pdata->irq_gpio = of_get_named_gpio(np, "st,irq_gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio))) {
		ERR_MSG("%s: fail to get irq_gpio\n", __func__);
		return -EINVAL;
	}

	DBG_MSG("Exit : %s (get irq_gpio[%d])\n", __func__, pdata->irq_gpio);
	       
	return 0;
}

/**
 * \ingroup spi_driver
 * \brief Probe ST33 SPI device and create device entry for user space.
 *
 * \param[in]       spi_device*
 *
 * \retval          0 if ok.
 *
*/
static int st33_probe(struct spi_device *spi)
{
	int ret = -1;
	struct st33_data	*st33_dev = NULL;
	struct st33_platform_data platform_data;
	
	DBG_MSG("Entry : %s, chip select=%d, bus number=%d\n",
	        __func__, spi->chip_select, spi->master->bus_num);
	
	/* check that dts file is correct */
	ret = st33_parse_dt(&spi->dev, &platform_data);
	if (ret) {
		ERR_MSG("%s - Failed to parse DT\n", __func__);
		return ret;
	}
	
	/* Allocate driver memory */
	st33_dev = kzalloc(sizeof(*st33_dev), GFP_KERNEL);
	if (!st33_dev) {
		ERR_MSG("%s : Failed to allocate memory for module data\n", __func__);
		return -ENOMEM;
	}
	
	st33_dev->irq_gpio = platform_data.irq_gpio;
	st33_dev->pow_gpio = platform_data.pow_gpio;

	/* GPIO IRQ initialization (Input) */
	ret = gpio_request(st33_dev->irq_gpio, "irq_gpio");
	if (ret) {
		ERR_MSG("%s : gpio_request failed (gpio %d)\n", __func__, st33_dev->irq_gpio);
		return -ENODEV;
	}
	
	ret = gpio_direction_input(st33_dev->irq_gpio);
	if (ret) {
		ERR_MSG("%s : gpio_direction_input failed (gpio %d)\n", __func__, st33_dev->irq_gpio);
		return -ENODEV;
	}

	/* GPIO POW initialization (Output, 0)*/
	ret = gpio_request(st33_dev->pow_gpio, "pow_gpio");
	if (ret) {
		ERR_MSG("%s : pow_gpio gpio_request failed\n", __func__);
		return -ENOMEM;
	}
	ret = gpio_direction_output(st33_dev->pow_gpio, 1);
	if (ret) {
		pr_err("%s : pow_gpio gpio_direction_output failed\n",__func__);
		return -ENOMEM;
	}
	gpio_set_value(st33_dev->pow_gpio, 0);
	
	/* Initialize the driver data */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = ST33_SPI_FREQ_MAX;
    st33_dev->trans_speed = ST33_SPI_FREQ_MAX;
	ret = spi_setup(spi);
	if (ret) {
		ERR_MSG("%s : spi_setup failed\n", __func__);
		return ret;
	}

	st33_dev->spi = spi;
    st33_dev -> st33_device.minor = MISC_DYNAMIC_MINOR;
    st33_dev -> st33_device.name = ST33_NAME;
    st33_dev -> st33_device.fops = &st33_dev_fops;
    st33_dev -> st33_device.parent = &spi->dev;
	st33_dev->rx_buffer = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);

	if (!st33_dev->rx_buffer) {
		ERR_MSG("%s : Failed to allocate rx_buffer\n", __func__);
		return -ENOMEM;
	}
	spi_set_drvdata(spi, st33_dev);
	
	irqIsAttached = false;
	device_open = 0;

	spi->irq = gpio_to_irq(st33_dev->irq_gpio);
	enable_irq_wake(st33_dev->spi->irq);

	/* init mutex and queues */
	init_waitqueue_head(&st33_dev->read_wq);
	mutex_init(&st33_dev->read_mutex);

	spin_lock_init(&st33_dev->irq_lock);

	ret = misc_register(&st33_dev->st33_device);
	if (ret) {
		ERR_MSG("%s : misc_register failed\n", __func__);
		return ret;
	}

	st33_set_polarity_mode(st33_dev);
	
	st33_disable_irq(st33_dev);

	DBG_MSG("Exit : %s\n",__func__);
	
	return 0;
}

/**
 * \ingroup spi_driver
 * \brief Called to release the resources when the device is removed.
 *
 * \param[in]       spi_device*
 *
 * \retval          0
 *
*/

static int st33_remove(struct spi_device *spi)
{
	struct st33_data	*st33_dev = spi_get_drvdata(spi);
	
	DBG_MSG("Entry : %s\n",__func__);
	
	free_irq(st33_dev->spi->irq, st33_dev);
	gpio_free(st33_dev->irq_gpio);
	
	mutex_destroy(&st33_dev->read_mutex);
	misc_deregister(&st33_dev->st33_device);
	
	kfree(st33_dev->rx_buffer);
	st33_dev->rx_buffer = NULL;
	
	if (st33_dev) {
		kfree(st33_dev);
	}
	
	DBG_MSG("Exit : %s\n",__func__);

	return 0;
}

static struct of_device_id st33_of_match[] = {
	{ .compatible = "st,st33", },  //it's same as the compatible of ese in dts.
	{}
};

static struct spi_driver st33_driver = {
	.driver = {
		.name =		ST33_NAME,
		.owner =	THIS_MODULE,
		.bus = 		&spi_bus_type,
		.of_match_table = st33_of_match,
	},
	.probe		= st33_probe,
	.remove		= st33_remove,
};

/**
 * \ingroup spi_driver
 * \brief Module init interface
 *
 * \param[in]       void
 *
 * \retval          spi_register_driver(&st33_driver)
 *
*/
static int __init st33_dev_init(void)
{
	int ret = 0;
	INFO_MSG("\nEntry : %s, loading st33 driver\n", __func__);
	
	ret = spi_register_driver(&st33_driver);
	
	INFO_MSG("\nExit : %s, spi_register_driver ret = %d\n", __func__,ret);
	return ret;
}

module_init(st33_dev_init);

/**
 * \ingroup spi_driver
 * \brief Module exit interface
 *
 * \param[in]       void
 *
 * \retval          void
 *
*/

static void __exit st33_dev_exit(void)
{
	DBG_MSG("Entry : %s\n",__func__);
	spi_unregister_driver(&st33_driver);
	DBG_MSG("Exit : %s\n",__func__);
}

module_exit(st33_dev_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("ST33 SPI driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

/** @} */
