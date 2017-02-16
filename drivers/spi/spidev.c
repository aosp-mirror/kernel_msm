/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/wakelock.h>


/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

#define SPIDEV_BUF_MAX_NODE_N 64
#define SPIDEV_NON_BLOCK_READ_TIMEOUT (2*HZ)
#define SPIDEV_READ_DELAY_TIME_US (100)
#define SPIDEV_READ_MAX_DELAY_TIMES (50)
#define SPIDEV_WRITE_DELAY_TIME_US (100)
#define SPIDEV_WRITE_MAX_DELAY_TIMES (5000)

/* MCU wake ap timeout */
#define DATA_TRANSFER_INTERVAL (1*HZ)
#define WAKEUP_DISPLAY_INTERVAL (1*HZ)

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

typedef struct _spidev_buf_list
{
	struct list_head list;
	unsigned int read_cnt;
	unsigned char *buffer;
}spidev_buf_list;

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	struct mutex		tx_buf_lock;
	struct mutex		buf_list_lock;
	struct mutex		spi_op_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;

	unsigned char *tx_buf;
	unsigned int wake_irq;
	unsigned int wake_display_irq;
	int wake_irq_gpio;
	int wakeup_mcu_gpio;
	int read_sync_gpio;
	int write_sync_gpio;
	int wake_display_gpio;
	spidev_work_mode_type work_mode;
	spidev_wakeup_disp_type wakeup_disp_enable;
	bool is_suspended;
	bool pending_irq;
	spidev_buf_list *idle_buf_head;
	spidev_buf_list *read_buf_head;
	struct completion read_compl;
	struct work_struct wakeup_read_work;
	struct work_struct wakeup_display_work;
	struct wake_lock wake_lock;
	struct wake_lock wake_display_lock;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*
 * This can be used for testing the controller, given the busnum and the
 * cs required to use. If those parameters are used, spidev is
 * dynamically added as device on the busnum, and messages can be sent
 * via this interface.
 */
static int busnum = -1;
module_param(busnum, int, S_IRUGO);
MODULE_PARM_DESC(busnum, "bus num of the controller");

static int chipselect = -1;
module_param(chipselect, int, S_IRUGO);
MODULE_PARM_DESC(chipselect, "chip select of the desired device");

static int maxspeed = 10000000;
module_param(maxspeed, int, S_IRUGO);
MODULE_PARM_DESC(maxspeed, "max_speed of the desired device");

static int spimode = SPI_MODE_3;
module_param(spimode, int, S_IRUGO);
MODULE_PARM_DESC(spimode, "mode of the desired device");

static struct spi_device *spi;

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spidev->tx_buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= spidev->rx_buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_write_ext(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
		.tx_buf	= spidev->tx_buf,
		.len	= len,
	};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read_ext(struct spidev_data *spidev, spidev_buf_list *node, size_t len)
{
	struct spi_transfer	t = {
		.rx_buf	= node->buffer,
		.len	= len,
	};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static int spidev_request_gpio(struct spidev_data	*spidev)
{
	struct device_node *np = NULL;

	/*Get MCU Wakeup AP GPIO*/
	np = of_find_compatible_node(NULL, NULL, "mcu,wakeupap");
	if (!np)
	{
		pr_err("%s: %s node not found\n", __FUNCTION__, "mcu,wakeupap");
		return -ENODEV;
	}

	spidev->wake_irq_gpio= of_get_named_gpio(np, "mcu_wakeup_ap", 0);
	if (spidev->wake_irq_gpio < 0)
	{
		pr_err("mcu_wakeup_ap:%d.\n", spidev->wake_irq_gpio);
		return -ENODEV;
	}

	if (gpio_request(spidev->wake_irq_gpio, "mcu_wakeup_ap") < 0)
	{
		pr_err("Failed to request gpio %d for mcu_wakeup_ap\n",spidev->wake_irq_gpio);
		return -ENODEV;
	}

	if (gpio_direction_input(spidev->wake_irq_gpio) < 0)
	{
		pr_err("Failed to set dir %d for mcu_wakeup_ap\n",spidev->wake_irq_gpio);
		return -ENODEV;
	}

	spidev->wake_irq = gpio_to_irq(spidev->wake_irq_gpio);

	pr_info("spidev_request_irq_gpio spidev->wake_irq = %u",spidev->wake_irq);

	/*Get AP Wakeup MCU Gpio*/
	np = of_find_compatible_node(NULL, NULL, "mcu,wakeupmcu");
	if (!np)
	{
		pr_err("%s: %s node not found\n", __FUNCTION__, "mcu,wakeupmcu");
		return -ENODEV;
	}

	spidev->wakeup_mcu_gpio= of_get_named_gpio(np, "ap_wakeup_mcu", 0);
	if (spidev->wakeup_mcu_gpio < 0)
	{
		pr_err("%s: %s gpio not found:\n", __FUNCTION__, "ap_wakeup_mcu");
		return -ENODEV;
	}

	if (gpio_request(spidev->wakeup_mcu_gpio, "ap_wakeup_mcu"))
	{
		pr_err("Failed to request gpio %d for ap_wakeup_mcu\n", spidev->wakeup_mcu_gpio);
		return -ENODEV;
	}

	pr_info("gpio_request ap_wakeup_mcu gpio:%d done\n", spidev->wakeup_mcu_gpio);

	/*Get AP Read Sync Gpio*/
	np = of_find_compatible_node(NULL, NULL, "mcu,readsync");
	if (!np)
	{
		pr_err("%s: %s node not found\n", __FUNCTION__, "mcu,readsync");
		return -ENODEV;
	}

	spidev->read_sync_gpio= of_get_named_gpio(np, "ap_read_sync", 0);
	if (spidev->read_sync_gpio < 0)
	{
		pr_err("%s: %s gpio not found:\n", __FUNCTION__, "read_sync_gpio");
		return -ENODEV;
	}

	if (gpio_request(spidev->read_sync_gpio, "ap_read_sync"))
	{
		pr_err("Failed to request gpio %d for ap_wakeup_mcu\n", spidev->read_sync_gpio);
		return -ENODEV;
	}

	pr_info("gpio_request read_sync_gpio gpio:%d done\n", spidev->read_sync_gpio);

	/*Get AP Write Sync Gpio*/
	np = of_find_compatible_node(NULL, NULL, "mcu,writesync");
	if (!np)
	{
		pr_err("%s: %s node not found\n", __FUNCTION__, "mcu,writesync");
		return -ENODEV;
	}

	spidev->write_sync_gpio = of_get_named_gpio(np, "ap_write_sync", 0);
	if (spidev->write_sync_gpio < 0)
	{
		pr_err("%s: %s gpio not found:\n", __FUNCTION__, "write_sync_gpio");
		return -ENODEV;
	}

	if (gpio_request(spidev->write_sync_gpio, "ap_write_sync"))
	{
		pr_err("Failed to request gpio %d for ap_write_sync\n", spidev->write_sync_gpio);
		return -ENODEV;
	}

	if (gpio_direction_input(spidev->write_sync_gpio) < 0)
	{
		pr_err("Failed to set dir %d for write_sync_gpio\n",spidev->write_sync_gpio);
		return -ENODEV;
	}

	pr_info("gpio_request write_sync_gpio gpio:%d done\n", spidev->write_sync_gpio);

	/*Get MCU Wakeup Display GPIO*/
	np = of_find_compatible_node(NULL, NULL, "mcu,wakeupdisplay");
	if (!np)
	{
		pr_err("%s: %s node not found\n", __FUNCTION__, "mcu,wakeupdisplay");
		return -ENODEV;
	}

	spidev->wake_display_gpio= of_get_named_gpio(np, "mcu_wakeup_display", 0);
	if (spidev->wake_display_gpio < 0)
	{
		pr_err("mcu_wakeup_display:%d.\n", spidev->wake_display_gpio);
		return -ENODEV;
	}

	if (gpio_request(spidev->wake_display_gpio, "mcu_wakeup_display") < 0)
	{
		pr_err("Failed to request gpio %d for mcu_wakeup_display\n",spidev->wake_display_gpio);
		return -ENODEV;
	}

	if (gpio_direction_input(spidev->wake_display_gpio) < 0)
	{
		pr_err("Failed to set dir %d for mcu_wakeup_display\n",spidev->wake_display_gpio);
		return -ENODEV;
	}

	spidev->wake_display_irq = gpio_to_irq(spidev->wake_display_gpio);

	pr_info("spidev_request_gpio spidev->wake_display_gpio = %u",spidev->wake_display_gpio);

	return 0;
}

static void spidev_release_gpio(struct spidev_data	*spidev)
{
	if (spidev)
	{
		gpio_free(spidev->wake_irq_gpio);
		gpio_free(spidev->wakeup_mcu_gpio);
		gpio_free(spidev->read_sync_gpio);
		gpio_free(spidev->write_sync_gpio);
		gpio_free(spidev->wake_display_gpio);
	}
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;
	spidev_buf_list *spidev_buf_node = NULL;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	if (spidev->work_mode == SPIDEV_WORK_MODE_USER)
	{
		mutex_lock(&spidev->buf_lock);
		status = spidev_sync_read(spidev, count);
		if (status > 0) {
			unsigned long	missing;

			missing = copy_to_user(buf, spidev->rx_buffer, status);
			if (missing == status)
				status = -EFAULT;
			else
				status = status - missing;
		}
		mutex_unlock(&spidev->buf_lock);
	}
	else
	{
		pr_debug("spidev read in kernel mode\n");

		/*wait for spi read complete*/
		if (filp->f_flags & O_NONBLOCK)
		{
			if(wait_for_completion_timeout(&spidev->read_compl, SPIDEV_NON_BLOCK_READ_TIMEOUT) == 0)
			{
				pr_info("O_NONBLOCK, spidev_read timeout\n");
				return -ETIMEDOUT;
			}
		}
		else
		{
			if (wait_for_completion_interruptible(&spidev->read_compl) < 0)
			{
				pr_info("spidev_read interuptible\n");
				return -ERESTARTSYS;
			}
		}

		mutex_lock(&spidev->buf_list_lock);
		if (!list_empty(&spidev->read_buf_head->list))
		{
			spidev_buf_node = list_first_entry(&spidev->read_buf_head->list, spidev_buf_list, list);
			/*spi read completion, and copy to userspace*/
			status = spidev_buf_node->read_cnt;

			if (status > 0)
			{
				unsigned long missing = 0;

				missing = copy_to_user(buf, spidev_buf_node->buffer, status);

				if (missing == status)
				{
					status = -EFAULT;
				}
				else
				{
					status = status - missing;
				}
			}

			list_move_tail(&spidev_buf_node->list, &spidev->idle_buf_head->list);

			if (list_empty(&spidev->read_buf_head->list))
			{
				wake_unlock(&spidev->wake_lock);
			}
		}
		else
		{
			status = -EFAULT;
		}

		mutex_unlock(&spidev->buf_list_lock);
		pr_debug("SPIDEV: user read done: %d\n", status);

	}

	return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;
	unsigned long		missing;
	unsigned int delay_times = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	if (spidev->work_mode == SPIDEV_WORK_MODE_USER)
	{
		mutex_lock(&spidev->buf_lock);
		missing = copy_from_user(spidev->tx_buffer, buf, count);
		if (missing == 0) {
			status = spidev_sync_write(spidev, count);
		} else
			status = -EFAULT;
		mutex_unlock(&spidev->buf_lock);
	}
	else
	{
		pr_debug("spidev write in kernel mode\n");
		/*spidev kernel mode write*/
		if (count > SPIDEV_KERNEL_MODE_LENGTH)
		{
			pr_err("SPIDEV Kernel mode tx length = %d out of range \n",count);
			status = -EINVAL;
		}
		else
		{
			mutex_lock(&spidev->spi_op_lock);
			memset(spidev->tx_buf, 0x00, SPIDEV_KERNEL_MODE_LENGTH);
			missing = copy_from_user(spidev->tx_buf, buf, count);
			if (missing == 0)
			{
				gpio_direction_output(spidev->wakeup_mcu_gpio, 1);
				do
				{
					udelay(SPIDEV_WRITE_DELAY_TIME_US);
					delay_times++;
					if (SPIDEV_WRITE_MAX_DELAY_TIMES < delay_times)
					{
						status = -ETIME;
						gpio_direction_output(spidev->wakeup_mcu_gpio, 0);
						mutex_unlock(&spidev->spi_op_lock);
						pr_err("spidev_write delay times is out of range\n");
						return status;
					}
				}while(gpio_get_value(spidev->write_sync_gpio) != 1);

				status = spidev_sync_write_ext(spidev, SPIDEV_KERNEL_MODE_LENGTH);
				gpio_direction_output(spidev->wakeup_mcu_gpio, 0);
			}
			else
			{
				pr_err("SPIDEV data missing while copy from user\n");
				status = -EFAULT;
			}
			mutex_unlock(&spidev->spi_op_lock);
		}
	}

	return status;
}

static int spidev_message(struct spidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*tx_buf, *rx_buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = spidev->tx_buffer;
	rx_buf = spidev->rx_buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Check total length of transfers.  Also check each
		 * transfer length to avoid arithmetic overflow.
		 */
		if (total > bufsiz || k_tmp->len > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = rx_buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(tx_buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		tx_buf += k_tmp->len;
		rx_buf += k_tmp->len;

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	rx_buf = spidev->rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, rx_buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		rx_buf += u_tmp->len;
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

void build_wakeup_display_data_struct(spidev_buf_list *buf_node)
{
	static char wakeup_gesture_data[] =
		{0x12,0x5A,0x00,0x0C,0x00,0x15,0x81,0x01,0x01,0x44,0x02,0x01,0x00,0x03,0x01,0x00,0xAC,0x00};

	if (NULL == buf_node)
	{
		return;
	}

	/*build wakeup gesture data in node*/
	memcpy(buf_node->buffer, wakeup_gesture_data, sizeof(wakeup_gesture_data));
	buf_node->read_cnt = sizeof(wakeup_gesture_data);
}

static void spidev_wakeup_read_work(struct work_struct *work)
{
	struct spidev_data *spidev = container_of(work, struct spidev_data, wakeup_read_work);
	spidev_buf_list *spidev_buf_node = NULL;
	unsigned int delay_times = 0;

	mutex_lock(&spidev->spi_op_lock);
	if (spidev)
	{
	retry:
		mutex_lock(&spidev->buf_list_lock);

		if (!list_empty(&spidev->idle_buf_head->list))
		{
			/*Get buf note from idle list*/
			spidev_buf_node = list_entry(spidev->idle_buf_head->list.prev, spidev_buf_list, list);

			gpio_direction_output(spidev->read_sync_gpio, 1);

			do
			{
				udelay(SPIDEV_READ_DELAY_TIME_US);
				if (delay_times++ > SPIDEV_READ_MAX_DELAY_TIMES)
				{
					pr_err("spidev_wakeup_read_work delay times is out of range\n");
					break;
				}
			}while(gpio_get_value(spidev->wake_irq_gpio) == 1);
			/*Execute Sync Read Operation*/
			spidev_buf_node->read_cnt = spidev_sync_read_ext(spidev, spidev_buf_node, SPIDEV_KERNEL_MODE_LENGTH);

			gpio_direction_output(spidev->read_sync_gpio, 0);

			/*move the buf node to read buf list*/
			list_move_tail(&spidev_buf_node->list, &spidev->read_buf_head->list);
			mutex_unlock(&spidev->buf_list_lock);
			spidev_complete(&spidev->read_compl);
			wake_lock_timeout(&spidev->wake_lock, DATA_TRANSFER_INTERVAL);
		}
		else
		{
			/*No Buf in idle list, sleep 2ms and retry*/
			mutex_unlock(&spidev->buf_list_lock);
			pr_info("spidev_wakeup_read_work buf list is full wait 2ms\n");
			msleep(2);
			goto retry;
		}
	}
	else
	{
		pr_err("spidev_wakeup_read_task spidev NULL\n");
	}

	mutex_unlock(&spidev->spi_op_lock);
}

static void spidev_wakeup_display_work(struct work_struct *work)
{
	struct spidev_data *spidev = container_of(work, struct spidev_data, wakeup_display_work);
	spidev_buf_list *spidev_buf_node = NULL;

	if (spidev)
	{
	    wake_lock_timeout(&spidev->wake_display_lock, WAKEUP_DISPLAY_INTERVAL);
	retry:
		mutex_lock(&spidev->buf_list_lock);
		if (!list_empty(&spidev->idle_buf_head->list))
		{
			/*Get buf note from idle list*/
			spidev_buf_node = list_entry(spidev->idle_buf_head->list.prev, spidev_buf_list, list);

			build_wakeup_display_data_struct(spidev_buf_node);
			/*move the buf node to read buf list*/
			list_move(&spidev_buf_node->list, &spidev->read_buf_head->list);
			mutex_unlock(&spidev->buf_list_lock);
			spidev_complete(&spidev->read_compl);
			wake_lock_timeout(&spidev->wake_lock, DATA_TRANSFER_INTERVAL);
		}
		else
		{
			/*No Buf in idle list, sleep 2ms and retry*/
			mutex_unlock(&spidev->buf_list_lock);
			pr_info("spidev_wakeup_display_work buf list is full wait 2ms\n");
			msleep(2);
			goto retry;
		}
	}
	else
	{
		pr_err("spidev_wakeup_display_work spidev NULL\n");
	}
}

static irqreturn_t spidev_wake_irq(int irq, void *arg)
{
	struct spidev_data *spidev = (struct spidev_data *)arg;

	if (spidev && (spidev->work_mode == SPIDEV_WORK_MODE_KERNEL))
	{
		if (spidev->is_suspended)
		{
			spidev->pending_irq = true;
		}
		else
		{
			schedule_work(&spidev->wakeup_read_work);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t spidev_wakeup_display_irq(int irq, void *arg)
{
	struct spidev_data *spidev = (struct spidev_data *)arg;

	if (spidev && (SPIDEV_WORK_MODE_KERNEL == spidev->work_mode)
        && (SPIDEV_WAKEUP_DISPLAY_ENALBE == spidev->wakeup_disp_enable))
	{
		schedule_work(&spidev->wakeup_display_work);
	}
	return IRQ_HANDLED;
}


static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct spidev_data	*spidev;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;
	spidev_work_mode_type old_work_mode;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *	data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u32 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE)
			retval = __get_user(tmp, (u8 __user *)arg);
		else
			retval = __get_user(tmp, (u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u16)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u32	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;

	case SPI_IOC_WR_WORK_MODE:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval != 0)
		{
			dev_err(&spi->dev, "get user data error\n");
			break;
		}

		if (((spidev_work_mode_type)tmp == SPIDEV_WORK_MODE_KERNEL)
		&& (bufsiz < SPIDEV_KERNEL_MODE_LENGTH))
		{
			dev_err(&spi->dev, "spi buffer size is not enough to run in kernel mode\n");
			retval = -EPERM;
			break;
		}

		/*Set Spi mode*/
		old_work_mode = spidev->work_mode;
		spidev->work_mode = (spidev_work_mode_type)(tmp == 0 ? 0 : 1);

		pr_info("spi work mode = %d spidev->wake_irq:%d\n",spidev->work_mode, spidev->wake_irq);

		if (old_work_mode == spidev->work_mode)
		{
			dev_info(&spi->dev, "spi work mode is not changed\n");
			break;
		}

		if (spidev->work_mode == SPIDEV_WORK_MODE_KERNEL)
		{
			retval = request_irq(spidev->wake_irq, spidev_wake_irq,
					IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"spidev wake irq", spidev);
			if (retval < 0)
			{
				dev_err(&spi->dev,"Couldn't acquire MCU HOST WAKE UP IRQ reval = %d\n",retval);
				break;
			}
		}
		else if (SPIDEV_WORK_MODE_USER == spidev->work_mode)
		{
			dev_err(&spi->dev,"set work mode is user cancel work,free irq \n");
			cancel_work_sync(&spidev->wakeup_read_work);
			free_irq(spidev->wake_irq, spidev);

			if (SPIDEV_WAKEUP_DISPLAY_ENALBE == spidev->wakeup_disp_enable)
			{
				cancel_work_sync(&spidev->wakeup_display_work);
				free_irq(spidev->wake_display_irq, spidev);
				spidev->wakeup_disp_enable = SPIDEV_WAKEUP_DISPLAY_DISABLE;
			}
		}
		else
		{
			;
		}

		break;

	case SPI_IOC_WAKEUP_DISPLAY_CTRL:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval != 0)
		{
			dev_err(&spi->dev, "wakeup display get user data error\n");
			break;
		}

		pr_info("spi work mode = %d, spidev->wakeup_display_irq:%d, enable:%d\n",
			spidev->work_mode, spidev->wake_display_irq, tmp);
		if (SPIDEV_WORK_MODE_KERNEL == spidev->work_mode)
		{
			if (spidev->wakeup_disp_enable == (spidev_wakeup_disp_type)(tmp == 0? 0: 1))
			{
				pr_info("wakeup display control is not charged:%d\n",spidev->wakeup_disp_enable);
			}
			else
			{
				if (SPIDEV_WAKEUP_DISPLAY_ENALBE == (spidev_wakeup_disp_type)(!!tmp))
				{
					retval = request_irq(spidev->wake_display_irq, spidev_wakeup_display_irq,
										IRQF_DISABLED | IRQF_TRIGGER_RISING,
									"spidev wakeup display irq", spidev);
					if (retval < 0)
					{
						dev_err(&spi->dev,"Couldn't acquire mcu wakeup display IRQ reval = %d\n",retval);
					}
				}
				else
				{
					free_irq(spidev->wake_display_irq, spidev);
				}
				spidev->wakeup_disp_enable = (spidev_wakeup_disp_type)(!!tmp);
			}
		}

		break;

	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		retval = spidev_message(spidev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = -ENXIO;
	int  idx = 0;
	spidev_buf_list *spidev_buf_node = NULL;
	spidev_buf_list *spidev_tmp_node = NULL;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	if (!spidev->tx_buffer) {
		spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->tx_buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			goto err_find_dev;
			}
		}

	if (!spidev->rx_buffer) {
		spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->rx_buffer) {
			dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	if (!spidev->tx_buf)
	{
		spidev->tx_buf = kmalloc(SPIDEV_KERNEL_MODE_LENGTH, GFP_KERNEL);
		if (!spidev->tx_buf)
		{
			dev_dbg(&spidev->spi->dev, "open tx buf/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}
	if (!spidev->read_buf_head)
	{
		spidev->read_buf_head = kmalloc(sizeof(spidev_buf_list), GFP_KERNEL);
		if (!spidev->read_buf_head)
		{
			dev_err(&spidev->spi->dev, "open read_buf_head alloc /ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
		else
		{
			INIT_LIST_HEAD(&spidev->read_buf_head->list);
		}
	}

	if (!spidev->idle_buf_head)
	{
		spidev->idle_buf_head = kmalloc(sizeof(spidev_buf_list), GFP_KERNEL);
		if (!spidev->idle_buf_head)
		{
			dev_err(&spidev->spi->dev, "open idle_buf_head alloc /ENOMEM\n");
			status =  -ENOMEM;
			goto err_alloc_rx_buf;
		}
		else
		{
			INIT_LIST_HEAD(&spidev->idle_buf_head->list);

			/*alloc spidev node buffer*/
			for (idx = 0; idx < SPIDEV_BUF_MAX_NODE_N; idx++)
			{
				spidev_buf_node = kmalloc(sizeof(spidev_buf_list), GFP_KERNEL);

				if (spidev_buf_node == NULL)
				{
					dev_err(&spidev->spi->dev, "open spidev buf node/ENOMEM\n");
					status = -ENOMEM;
					goto err_alloc_rx_buf;
				}
				else
				{
					list_add_tail(&spidev_buf_node->list, &spidev->idle_buf_head->list);
					spidev_buf_node->read_cnt = 0;
					spidev_buf_node->buffer = kmalloc(SPIDEV_KERNEL_MODE_LENGTH, GFP_KERNEL);
					if (spidev_buf_node->buffer == NULL)
					{
						dev_err(&spidev->spi->dev, "open spidev buf /ENOMEM\n");
						status = -ENOMEM;
						goto err_alloc_rx_buf;
					}
				}
			}

		}
	}
	spidev->users++;
	filp->private_data = spidev;
	nonseekable_open(inode, filp);

	mutex_unlock(&device_list_lock);
	return 0;

err_alloc_rx_buf:
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;
	if (spidev->rx_buffer)
	{
		kfree(spidev->rx_buffer);
		spidev->rx_buffer = NULL;
	}

	if (spidev->tx_buf)
	{
		kfree(spidev->tx_buf);
		spidev->tx_buf = NULL;
	}
	if (spidev->read_buf_head)
	{
		kfree(spidev->read_buf_head);
		spidev->read_buf_head = NULL;
	}

	if (spidev->idle_buf_head)
	{
		/*free buffer node*/
		list_for_each_entry_safe(spidev_buf_node, spidev_tmp_node, &(spidev->idle_buf_head->list), list)
		{
			if (spidev_buf_node)
			{
				if (spidev_buf_node->buffer)
				{
					kfree(spidev_buf_node->buffer);
				}
				list_del(&spidev_buf_node->list);
				kfree(spidev_buf_node);
			}
		}

		/*free head list*/
		kfree(spidev->idle_buf_head);
		spidev->idle_buf_head = NULL;
	}
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = 0;
	spidev_buf_list *spidev_buf_node = NULL;
	spidev_buf_list *spidev_tmp_node = NULL;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int		dofree;

		if (SPIDEV_WORK_MODE_KERNEL == spidev->work_mode)
		{
			cancel_work_sync(&spidev->wakeup_read_work);
			free_irq(spidev->wake_irq, spidev);
			if (SPIDEV_WAKEUP_DISPLAY_ENALBE == spidev->wakeup_disp_enable)
			{
				cancel_work_sync(&spidev->wakeup_display_work);
				free_irq(spidev->wake_display_irq, spidev);
				spidev->wakeup_disp_enable = SPIDEV_WAKEUP_DISPLAY_DISABLE;
			}
			spidev->work_mode = SPIDEV_WORK_MODE_USER;
		}

		kfree(spidev->tx_buffer);
		spidev->tx_buffer = NULL;

		kfree(spidev->rx_buffer);
		spidev->rx_buffer = NULL;
		kfree(spidev->tx_buf);
		spidev->tx_buf = NULL;
		list_for_each_entry_safe(spidev_buf_node, spidev_tmp_node,&(spidev->idle_buf_head->list), list)
		{
			kfree(spidev_buf_node->buffer);
			list_del(&spidev_buf_node->list);
			kfree(spidev_buf_node);
		}

		list_for_each_entry_safe(spidev_buf_node, spidev_tmp_node,&(spidev->read_buf_head->list), list)
		{
			kfree(spidev_buf_node->buffer);
			list_del(&spidev_buf_node->list);
			kfree(spidev_buf_node);
		}
		kfree(spidev->idle_buf_head);
		spidev->idle_buf_head = NULL;
		kfree(spidev->read_buf_head);
		spidev->read_buf_head = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{
	struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);
	mutex_init(&spidev->tx_buf_lock);
	mutex_init(&spidev->buf_list_lock);
	mutex_init(&spidev->spi_op_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
					spidev, "spidev%d.%d",
					spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}

	init_completion(&spidev->read_compl);
	status = spidev_request_gpio(spidev);

	INIT_WORK(&spidev->wakeup_read_work,spidev_wakeup_read_work);

	INIT_WORK(&spidev->wakeup_display_work,spidev_wakeup_display_work);

	mutex_unlock(&device_list_lock);

	wake_lock_init(&spidev->wake_lock, WAKE_LOCK_SUSPEND, "mcu_commu");

	wake_lock_init(&spidev->wake_display_lock, WAKE_LOCK_SUSPEND, "mcu_display");

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
		kfree(spidev);

	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	wake_lock_destroy(&spidev->wake_lock);
	wake_lock_destroy(&spidev->wake_display_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);

	if (spidev->work_mode == SPIDEV_WORK_MODE_KERNEL)
	{
		cancel_work_sync(&spidev->wakeup_read_work);
		free_irq(spidev->wake_irq, spidev);
		if (SPIDEV_WAKEUP_DISPLAY_ENALBE == spidev->wakeup_disp_enable)
		{
			cancel_work_sync(&spidev->wakeup_display_work);
			free_irq(spidev->wake_display_irq, spidev);
			spidev->wakeup_disp_enable = SPIDEV_WAKEUP_DISPLAY_DISABLE;
		}
		spidev->work_mode = SPIDEV_WORK_MODE_USER;
	}
	spidev_release_gpio(spidev);

	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static int spidev_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);
	int retval = 0;

	pr_info("%s:enter",__func__);
	if (spidev->work_mode == SPIDEV_WORK_MODE_KERNEL)
	{
		retval = enable_irq_wake(spidev->wake_irq);
		if (retval < 0)
		{
			dev_err(&spi->dev,"Couldn't enable mcu_host_wake as wakeup interrupt\n");
		}

		if (SPIDEV_WAKEUP_DISPLAY_ENALBE == spidev->wakeup_disp_enable)
		{
			retval = enable_irq_wake(spidev->wake_display_irq);
			if (retval < 0)
			{
				dev_err(&spi->dev,"Couldn't enable mcu wakeup display irq as wakeup interrupt\n");
			}
		}

		/*Reset flag to capture pending irq before resume */
		spidev->pending_irq = false;

		spidev->is_suspended = true;
	}

	return 0;
}

static int spidev_resume(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);
	int retval = 0;

	pr_info("%s:enter",__func__);
	if (spidev->work_mode == SPIDEV_WORK_MODE_KERNEL)
	{
		retval = disable_irq_wake(spidev->wake_irq);
		if (retval < 0)
		{
			dev_err(&spi->dev,"Couldn't disable mcu_host_wake as wakeup interrupt\n");
		}

		if (SPIDEV_WAKEUP_DISPLAY_ENALBE == spidev->wakeup_disp_enable)
		{
			retval = disable_irq_wake(spidev->wake_display_irq);
			if (retval < 0)
			{
				dev_err(&spi->dev,"Couldn't disable mcu wakeup display irq as wakeup interrupt\n");
			}
		}

		spidev->is_suspended = false;

		if (spidev->pending_irq)
		{
			pr_info("%s: pending spidev irq\n", __func__);
			spidev->pending_irq = false;
			schedule_work(&spidev->wakeup_read_work);
		}
	}

	return 0;
}


static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "rohm,dh2228fv", },
	{ .compatible = "qcom,spi-msm-codec-slave", },
	{},
};

MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidev",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,
	.suspend =	spidev_suspend,
	.resume =	spidev_resume,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;

	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		status = PTR_ERR(spidev_class);
		goto error_class;
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0)
		goto error_register;

	if (busnum != -1 && chipselect != -1) {
		struct spi_board_info chip = {
					.modalias	= "spidev",
					.mode		= spimode,
					.bus_num	= busnum,
					.chip_select	= chipselect,
					.max_speed_hz	= maxspeed,
		};

		struct spi_master *master;

		master = spi_busnum_to_master(busnum);
		if (!master) {
			status = -ENODEV;
			goto error_busnum;
		}

		/* We create a virtual device that will sit on the bus */
		spi = spi_new_device(master, &chip);
		if (!spi) {
			status = -EBUSY;
			goto error_mem;
		}
		dev_dbg(&spi->dev, "busnum=%d cs=%d bufsiz=%d maxspeed=%d",
			busnum, chipselect, bufsiz, maxspeed);
	}
	return 0;
error_mem:
error_busnum:
	spi_unregister_driver(&spidev_spi_driver);
error_register:
	class_destroy(spidev_class);
error_class:
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	if (spi) {
		spi_unregister_device(spi);
		spi = NULL;
	}
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
