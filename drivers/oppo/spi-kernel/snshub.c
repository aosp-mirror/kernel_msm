#include <linux/init.h>
#include <linux/module.h>
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kfifo.h>
#include <linux/ioctl.h>
#include <linux/sched.h>


/* IOCTL magic number */
#define SNSHUB_MAGIC 'x'

#define    SNSHUB_GPIO_DN_INT_GET          _IOR(SNSHUB_MAGIC, 1, int)
#define    SNSHUB_GPIO_DN_INT_SET          _IOW(SNSHUB_MAGIC, 2, int)
#define    SNSHUB_GPIO_DN_ACK_GET          _IOR(SNSHUB_MAGIC, 3, int)
#define    SNSHUB_GPIO_WAIT_DN_ACK_PU      _IOWR(SNSHUB_MAGIC, 4, int)
#define    SNSHUB_GPIO_WAIT_DN_ACK_PD      _IOWR(SNSHUB_MAGIC, 5, int)
#define    SNSHUB_GPIO_UP_INT_GET          _IOR(SNSHUB_MAGIC, 6, int)
#define    SNSHUB_GPIO_WAIT_UP_INT_PU      _IO(SNSHUB_MAGIC, 7)
#define    SNSHUB_GPIO_UP_ACK_GET          _IOR(SNSHUB_MAGIC, 8, int)
#define    SNSHUB_GPIO_UP_ACK_SET          _IOW(SNSHUB_MAGIC, 9, int)
#define    SNSHUB_GPIO_UP_ACK_SEND_PULSE   _IOW(SNSHUB_MAGIC, 10, int)
#define    SNSHUB_CLEAN_DN_ACK_PU_SEM      _IOR(SNSHUB_MAGIC, 11, int)
#define    SNSHUB_CLEAN_UP_INT_PU_SEM      _IOR(SNSHUB_MAGIC, 12, int)
#define    SNSHUB_UP_PM_RELAX              _IO(SNSHUB_MAGIC, 13)
#define    SNSHUB_GET_WRITE_BUF(size)      _IOC(_IOC_READ, SNSHUB_MAGIC, 14, (size))
#define    SNSHUB_NOTIFY_WRITE_STATUS      _IOW(SNSHUB_MAGIC, 15, int)
#define    SNSHUB_PUT_TO_READ_BUF(size)    _IOC(_IOC_WRITE, SNSHUB_MAGIC, 16, (size))
#define    SNSHUB_SPI_READ(size)           _IOC(_IOC_READ, SNSHUB_MAGIC, 17, (size))
#define    SNSHUB_SPI_WRITE(size)          _IOC(_IOC_WRITE, SNSHUB_MAGIC, 18, (size))
#define    SNSHUB_SET_THREAD_TO_FIFO       _IOR(SNSHUB_MAGIC, 19, int)

#define log_e(fmt, ...)\
	printk(KERN_ERR "snshub:%s():%d: " fmt "\n", __func__ , __LINE__, ##__VA_ARGS__)

static struct spidev_data *spidev = NULL;

struct spidev_data {
	spinlock_t          spi_lock;
	dev_t               devt;
	struct device       *dev;
	struct spi_device   *spi;

	uint8_t            *spi_rx_buffer;
	uint8_t            *spi_tx_buffer;

	int                 up_int_gpio;
	int                 dn_ack_gpio;
	int                 dn_int_gpio;
	int                 up_ack_gpio;

	int                 up_int_irq;
	int                 dn_ack_irq;

	struct semaphore    up_int_pu_sem;
	struct semaphore    dn_ack_pu_sem;

	int                 is_suspended;

	struct wakeup_source upload_wakesrc;

	uint8_t             *write_buf;
	struct semaphore    write_buf_ready_sem;
	int                 write_status;
	struct semaphore    status_ready_sem;
	struct mutex        write_lock;

	uint8_t             *read_packet_area;
};

struct data_cdev {
	dev_t               devt;
	struct device       *dev;
	uint16_t            channel_id;
	const char          *dev_name;

	struct mutex        read_lock;

	struct semaphore    rx_fifo_packet_cnt_sem;
	struct kfifo        rx_fifo;
	struct mutex        rx_fifo_lock;

	struct wakeup_source wakesrc;
	char ws_name[32];
};

struct oppo_comm_packet {
	uint16_t channel_id;
	uint16_t data_size;
};

struct cdev_info_list_unit {
	const char          *dev_name;
	uint16_t            channel_id;
	struct data_cdev    *cdev_info;
};

struct cdev_info_list_unit g_cdev_info_list[] = {
	{"snshub",          0},
	{"snshub_data",     1},
	{"mcu_factory",     2},
	{"gps_data",        3},
	{"std_sns",         4},
	{"sns_batch",       5},
};

static struct data_cdev *get_cdev_info_by_channel_id(int channel_id)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
		if (channel_id == g_cdev_info_list[i].channel_id)
			return g_cdev_info_list[i].cdev_info;
	}
	return g_cdev_info_list[0].cdev_info;  //avoid null pointer
}

#define PACKET_HEAD_SIZE 4

static int distribute_packet_to_data_cdev(struct oppo_comm_packet *packet)
{
	int ret = 0;
	struct data_cdev *cdev = get_cdev_info_by_channel_id(packet->channel_id);

	mutex_lock(&cdev->rx_fifo_lock);
	if (kfifo_avail(&cdev->rx_fifo) >= PACKET_HEAD_SIZE + packet->data_size) {
		kfifo_in(&cdev->rx_fifo, packet, PACKET_HEAD_SIZE + packet->data_size);
		up(&cdev->rx_fifo_packet_cnt_sem);
		__pm_wakeup_event(&cdev->wakesrc, 1000);
	} else {
		log_e("rx_fifo of %s is full, abandon a frame", cdev->dev_name);
		ret = -1;
	}
	mutex_unlock(&cdev->rx_fifo_lock);
	return ret;
}


static irqreturn_t up_int_irq_fn(int irq, void *handle)
{
	struct device *dev = (struct device *)handle;

	if (!dev ||!spidev) {
		log_e("NULL pointer dev %p or spidev %p", dev, spidev);
		return IRQ_HANDLED;
	}

	if (irq == spidev->up_int_irq) {
		if (gpio_get_value(spidev->up_int_gpio) == 1) {
			__pm_wakeup_event(&spidev->upload_wakesrc, 1000);
			up(&spidev->up_int_pu_sem);
		} else {
			log_e("after rising edge, up_int is 0");
		}
	} else {
		log_e("(%d, %p) irq not int pin", irq, handle);
	}
	return IRQ_HANDLED;
}

static irqreturn_t dn_ack_irq_fn(int irq, void *handle)
{
	struct device *dev = (struct device *)handle;

	if (!dev ||!spidev ) {
		log_e("NULL pointer dev %p or spidev %p", dev, spidev);
		return IRQ_HANDLED;
	}

	if (irq == spidev->dn_ack_irq) {
		if (gpio_get_value(spidev->dn_ack_gpio) == 1) {
			up(&spidev->dn_ack_pu_sem);
		} else {
			log_e("after rising edge, dn_ack is 0");
		}
	} else {
		log_e("(%d, %p) irq not int pin", irq, handle);
	}
	return IRQ_HANDLED;
}

static void spidev_complete(void *arg)
{
	complete(arg);
}

static int spidev_spi_sync(void *tx, void *rx, size_t size)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status = 0;
	struct spi_message message;
	struct spi_transfer	t = {
		.tx_buf		= tx,
		.rx_buf		= rx,
		.len		= size,
	};

	while (spidev->is_suspended)
		msleep(10);

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	message.complete = spidev_complete;
	message.context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi != NULL)
		status = spi_async(spidev->spi, &message);
	else
		status = -ESHUTDOWN;
	spin_unlock_irq(&spidev->spi_lock);

	if (status != 0)
		return -EFAULT;

	wait_for_completion(&done);
	status = message.status;
	return status == 0 ? 0 : -EFAULT;
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct sched_param param = {.sched_priority = MAX_RT_PRIO / 2};
	int input = 0;
	int output = 0;
	int ret = 0;
	int i = 0;

	if (_IOC_TYPE(cmd) != SNSHUB_MAGIC)
		return -ENOTTY;

	//some big-data-about-copy
	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SNSHUB_SPI_READ(0)):
		if (_IOC_SIZE(cmd) > PAGE_SIZE)
			return -EINVAL;
		spidev->spi_rx_buffer[0] = 0x00;
		if (spidev_spi_sync(NULL, spidev->spi_rx_buffer, _IOC_SIZE(cmd) + 1) < 0)
			return -EFAULT;
		if (copy_to_user((void *)arg, spidev->spi_rx_buffer + 1, _IOC_SIZE(cmd)))
			return -EFAULT;
		return 0;

	case _IOC_NR(SNSHUB_SPI_WRITE(0)):
		if (_IOC_SIZE(cmd) > PAGE_SIZE)
			return -EINVAL;
		spidev->spi_tx_buffer[0] = 0x80;
		if (copy_from_user(spidev->spi_tx_buffer + 1, (void *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
		if (spidev_spi_sync(spidev->spi_tx_buffer, NULL, _IOC_SIZE(cmd) + 1) < 0)
			return -EFAULT;
		return 0;

	case _IOC_NR(SNSHUB_GET_WRITE_BUF(0)):
		if (_IOC_SIZE(cmd) > PAGE_SIZE)
			return -EINVAL;
		if (down_interruptible(&spidev->write_buf_ready_sem)) {
			return -EINTR;
		}
		if (copy_to_user((void *)arg, spidev->write_buf, _IOC_SIZE(cmd)))
			return -EFAULT;
		return 0;

	case _IOC_NR(SNSHUB_PUT_TO_READ_BUF(0)):
		if (copy_from_user(spidev->read_packet_area, (void *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
		return distribute_packet_to_data_cdev((struct oppo_comm_packet *)spidev->read_packet_area);
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(&input, (void *)arg, sizeof(input)))
			return -EFAULT;
	}

	switch (cmd) {
	case SNSHUB_GPIO_DN_INT_GET:
		output = gpio_get_value(spidev->dn_int_gpio);
		break;

	case SNSHUB_GPIO_DN_INT_SET:
		gpio_direction_output(spidev->dn_int_gpio, input);
		break;

	case SNSHUB_GPIO_DN_ACK_GET:
		output = gpio_get_value(spidev->dn_ack_gpio);
		break;

	case SNSHUB_GPIO_WAIT_DN_ACK_PU:
		output = down_timeout(&spidev->dn_ack_pu_sem, msecs_to_jiffies(input));
		break;

	case SNSHUB_GPIO_WAIT_DN_ACK_PD:
		output = -ETIMEDOUT;
		for (i = 0; i < 1000; ++i) {
			if (gpio_get_value(spidev->dn_ack_gpio) == 0) {
				output = 0;
				break;
			}
			udelay(1);
		}
		for (i = 0; i < input - 1; ++i) {
			if (gpio_get_value(spidev->dn_ack_gpio) == 0) {
				output = 0;
				break;
			}
			msleep(1);
		}
		break;

	case SNSHUB_GPIO_UP_INT_GET:
		output = gpio_get_value(spidev->up_int_gpio);
		break;

	case SNSHUB_GPIO_WAIT_UP_INT_PU:
		if (down_interruptible(&spidev->up_int_pu_sem)) {
			return -EINTR;
		}
		break;

	case SNSHUB_GPIO_UP_ACK_GET:
		output = gpio_get_value(spidev->up_ack_gpio);
		break;

	case SNSHUB_GPIO_UP_ACK_SET:
		gpio_direction_output(spidev->up_ack_gpio, input);
		break;

	case SNSHUB_GPIO_UP_ACK_SEND_PULSE:
		gpio_direction_output(spidev->up_ack_gpio, 1);
		(input < 1000) ? udelay(input) : msleep(input / 1000);
		gpio_direction_output(spidev->up_ack_gpio, 0);
		break;

	case SNSHUB_CLEAN_DN_ACK_PU_SEM:
		while(!down_trylock(&spidev->dn_ack_pu_sem))
			++output;
		break;

	case SNSHUB_CLEAN_UP_INT_PU_SEM:
		while(!down_trylock(&spidev->up_int_pu_sem))
			++output;
		break;

	case SNSHUB_UP_PM_RELAX:
		__pm_relax(&spidev->upload_wakesrc);
		break;

	case SNSHUB_NOTIFY_WRITE_STATUS:
		spidev->write_status = input;
		up(&spidev->status_ready_sem);
		break;

	case SNSHUB_SET_THREAD_TO_FIFO:
		output = sched_setscheduler(current, SCHED_FIFO, &param);
		break;

	default:
		log_e("no such command!");
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		if (copy_to_user((void *)arg, (const void *)&output, sizeof(output)))
			log_e("copy to user failed!");
	}

	return ret;
}


static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct data_cdev *cdev = filp->private_data;
	unsigned long missing = 0;
	uint8_t packet_area[520] = {0};
	struct oppo_comm_packet *packet = (struct oppo_comm_packet *)packet_area;
	int    status = 0;
	int    data_size = 0;

	mutex_lock(&cdev->read_lock);

	if (down_trylock(&cdev->rx_fifo_packet_cnt_sem)) {
		__pm_relax(&cdev->wakesrc);
		if (filp->f_flags & O_NONBLOCK) {
			status = -EAGAIN;
			goto out_spidev_read;
		} else {
			if (down_interruptible(&cdev->rx_fifo_packet_cnt_sem)) {
				status = -EINTR;
				goto out_spidev_read;
			}
		}
	}

	mutex_lock(&cdev->rx_fifo_lock);

	if (kfifo_out(&cdev->rx_fifo, packet, PACKET_HEAD_SIZE) != PACKET_HEAD_SIZE) {
		status = -EFAULT;
		log_e("impossible situation occured in LINE %d", __LINE__);
	}

	data_size = packet->data_size;
	if (kfifo_out(&cdev->rx_fifo, packet + 1, data_size) != data_size) {
		status = -EFAULT;
		log_e("impossible situation occured in LINE %d", __LINE__);
	}

	if (status < 0) {
		log_e("clean rx_fifo and sem to avoid fatal err");
		kfifo_reset(&cdev->rx_fifo);
		while (!down_trylock(&cdev->rx_fifo_packet_cnt_sem));
	}

	mutex_unlock(&cdev->rx_fifo_lock);

	if (status == 0) {
		if (count < data_size) {
			data_size = count;
			log_e("missing some data (%d bytes) to user because count < data_size", data_size - count);
		}
		missing = copy_to_user(buf, packet + 1, data_size);
		if (!missing) {
			status = data_size;
		} else {
			log_e("copy_to_user missing some data");
			status = -EFAULT;
		}
	}

out_spidev_read:
	mutex_unlock(&cdev->read_lock);
	return status;

}

static ssize_t spidev_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	int status = 0;
	struct data_cdev *cdev = filp->private_data;
	struct oppo_comm_packet *packet = (struct oppo_comm_packet *)spidev->write_buf;

	if (count > 516)
		return -EINVAL;

	mutex_lock(&spidev->write_lock);

	while(!down_trylock(&spidev->write_buf_ready_sem));
	while(!down_trylock(&spidev->status_ready_sem));

	if (copy_from_user(packet + 1, buf, count)) {
		log_e("copy from user missing some data");
		status = -EFAULT;
		goto out_spidev_write;

	}
	packet->channel_id = cdev->channel_id;
	packet->data_size = (uint16_t)count;

	up(&spidev->write_buf_ready_sem);

	if (down_timeout(&spidev->status_ready_sem, msecs_to_jiffies(5000)) < 0) {
		log_e("user snshub driver has no response!");
		while(!down_trylock(&spidev->write_buf_ready_sem));
		status = -EFAULT;
		goto out_spidev_write;
	}

	if (spidev->write_status < 0) {
		log_e("write failed, refer to user snshub driver (%d)", -(spidev->write_status));
		status = -EFAULT;
		goto out_spidev_write;
	}

	status = count;

out_spidev_write:
	mutex_unlock(&spidev->write_lock);
	return status;

}

static int spidev_open(struct inode *inode, struct file *filp)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i) {
		if (g_cdev_info_list[i].cdev_info->devt == inode->i_rdev) {
			filp->private_data = g_cdev_info_list[i].cdev_info;
			nonseekable_open(inode, filp);
			return 0;
		}
	}

	log_e("cannot find data_cdev!");
	return -ENXIO;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	.write =	spidev_write,
	.read =		spidev_read,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
	.unlocked_ioctl = spidev_ioctl,
};


#define SPIDEV_MAJOR			153
#define N_SPI_MINORS			32

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static struct class *spidev_class;
struct spi_device   *spi;

static const struct class_attribute attributes[] = {
};

static int get_gpio_from_devtree(struct spidev_data *spidev, const char *gpio_name)
{
	struct device *dev = &(spidev->spi->dev);
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;
	int gpio;

	if (!node)
		return -ENODEV;

	gpio  = of_get_named_gpio_flags(node, gpio_name, 0, &flags);
	if (gpio < 0 && gpio != -EPROBE_DEFER) {
		log_e("Failed to get gpio flags, error");
		return -ENODEV;
	}
	if (gpio_is_valid(gpio)) {
		if (gpio_request(gpio, "up_int_gpio") < 0) {
			log_e("devm_gpio_request, error");
			return -ENODEV;
		}
	}
	return gpio;
}

static int spidev_gpio_request_irq(struct spidev_data *spidev, int gpio,
		unsigned long irq_flags, irqreturn_t (*irqfunc)(int irq, void *handle))
{
	int irq;
	struct device *dev = &(spidev->spi->dev);

	/* Initialize IRQ */
	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		log_e("gpio(%d) gpio_to_irq failed: %d\n", gpio, irq);
		return -EINVAL;
	}

	if (request_irq(irq, irqfunc, irq_flags, dev_name(dev), dev) < 0) {
		log_e("Error, could not request irq\n");
		return -EINVAL;
	}
	return irq;
}


static int spidev_gpio_init(struct spidev_data *spidev)
{
	spidev->up_int_gpio = get_gpio_from_devtree(spidev, "up_int_gpio");
	if (spidev->up_int_gpio < 0)
		return -ENODEV;
	gpio_direction_input(spidev->up_int_gpio);
	spidev->up_int_irq = spidev_gpio_request_irq(spidev, spidev->up_int_gpio,
			IRQF_TRIGGER_RISING, up_int_irq_fn);
	if (spidev->up_int_irq < 0)
		return -ENODEV;
	if (enable_irq_wake(spidev->up_int_irq) != 0)
		return -EINVAL;

	spidev->up_ack_gpio = get_gpio_from_devtree(spidev, "up_ack_gpio");
	if (spidev->up_ack_gpio < 0)
		return -ENODEV;
	gpio_direction_output(spidev->up_ack_gpio, 0);

	spidev->dn_int_gpio = get_gpio_from_devtree(spidev, "dn_int_gpio");
	if (spidev->dn_int_gpio < 0)
		return -ENODEV;
	gpio_direction_output(spidev->dn_int_gpio, 0);

	spidev->dn_ack_gpio = get_gpio_from_devtree(spidev, "dn_ack_gpio");
	if (spidev->dn_ack_gpio < 0)
		return -ENODEV;
	gpio_direction_input(spidev->dn_ack_gpio);
	spidev->dn_ack_irq = spidev_gpio_request_irq(spidev, spidev->dn_ack_gpio,
			IRQF_TRIGGER_RISING, dn_ack_irq_fn);
	if (spidev->dn_ack_irq < 0)
		return -ENODEV;

	return 0;
}

static int data_cdev_init(struct spidev_data *spidev, struct cdev_info_list_unit *list_unit)
{
	int                 status;
	unsigned long		minor;
	struct spi_device 	*spi = spidev->spi;
	struct data_cdev    *data_cdev = NULL;
	int ret;

	data_cdev = kzalloc(sizeof(*data_cdev), GFP_KERNEL);

	if (!data_cdev)
		return -ENOMEM;

	data_cdev->channel_id = list_unit->channel_id;
	data_cdev->dev_name = list_unit->dev_name;

	sema_init(&data_cdev->rx_fifo_packet_cnt_sem, 0);
	mutex_init(&data_cdev->read_lock);

	ret = kfifo_alloc(&data_cdev->rx_fifo, 32 * 1024, GFP_KERNEL);
	if (ret)
		return -ENOMEM;
	mutex_init(&data_cdev->rx_fifo_lock);

	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		data_cdev->devt = MKDEV(SPIDEV_MAJOR, minor);
		data_cdev->dev = device_create(spidev_class, &spi->dev, data_cdev->devt,
					data_cdev, list_unit->dev_name);
		status = PTR_ERR_OR_ZERO(data_cdev->dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		data_cdev->dev = NULL;
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
	}

	snprintf(data_cdev->ws_name, sizeof(data_cdev->ws_name), "ws_%s", list_unit->dev_name);
	wakeup_source_init(&(data_cdev->wakesrc), data_cdev->ws_name);

	list_unit->cdev_info = data_cdev;

	return status;
}

static int spidev_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int status = 0;
	int i = 0;

	if (dev == NULL) {
		log_e("dev is null!!!!!");
	}

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);

	sema_init(&spidev->up_int_pu_sem, 0);
	sema_init(&spidev->dn_ack_pu_sem, 0);

	spidev->spi_rx_buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!spidev->spi_rx_buffer)
		return -ENOMEM;

	spidev->spi_tx_buffer = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!spidev->spi_tx_buffer)
		return -ENOMEM;

	if (spidev_gpio_init(spidev) < 0)
		log_e("spidev_gpio_init failed!");

	spi_set_drvdata(spi, spidev);

	wakeup_source_init(&spidev->upload_wakesrc, "upload_wakesrc");

	spidev->write_buf = kzalloc(1024, GFP_KERNEL);
	if (!spidev->write_buf)
		return -ENOMEM;
	sema_init(&spidev->write_buf_ready_sem, 0);
	spidev->write_status = 0;
	sema_init(&spidev->status_ready_sem, 0);
	mutex_init(&spidev->write_lock);
	spidev->read_packet_area = kzalloc(1024, GFP_KERNEL);
	if (!spidev->read_packet_area)
		return -ENOMEM;


	for (i = 0; i < ARRAY_SIZE(g_cdev_info_list); ++i)
		data_cdev_init(spidev, &g_cdev_info_list[i]);

	printk(KERN_INFO "snshub: >>>>>snshub device driver init successfully<<<<<\n");
	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	return 0;
}

static int snshub_suspend(struct device *dev)
{
	spidev->is_suspended = true;
	return 0;
}

static int snshub_resume(struct device *dev)
{
	spidev->is_suspended = false;
	return 0;
}

static const struct dev_pm_ops snshub_pm_ops = {
	.resume         = snshub_resume,
	.suspend        = snshub_suspend,
};

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidev-snshub",
		.owner =	THIS_MODULE,
		.pm             = &snshub_pm_ops,
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,
};

static int spidev_plat_probe(struct platform_device *pdev)
{
	int status;
	unsigned int busnum;
	struct spi_master *master;
	int i;

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0)
		return status;

	spidev_class = class_create(THIS_MODULE, "snshub");
	if (IS_ERR(spidev_class)) {
		status = PTR_ERR(spidev_class);
		goto error_class;
	}

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (class_create_file(spidev_class, attributes + i))
			goto undo_create_file;

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0)
		goto error_register;

	status = of_property_read_u32(pdev->dev.of_node, "spi-bus-num", &busnum);
	if (status)
		goto error_busnum;

	master = spi_busnum_to_master(busnum);
	if (!master) {
		status = -ENODEV;
		goto error_busnum;
	}

	spi = oppo_of_register_spi_device(master, pdev->dev.of_node);
	if (!spi) {
		status = -EBUSY;
		goto error_mem;
	}
	return 0;

error_mem:
error_busnum:
	spi_unregister_driver(&spidev_spi_driver);
error_register:

undo_create_file:
	for (i--; i >= 0; i--)
		class_remove_file(spidev_class, attributes + i);

	class_destroy(spidev_class);
error_class:
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	return status;
}

static int spidev_plat_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "oppo,spidev-snshub", },
	{},
};

static struct platform_driver spidev_plat_driver = {
	.driver = {
		.name = "spidev-snshub",
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe = spidev_plat_probe,
	.remove = spidev_plat_remove,
};

static int __init spidev_init(void)
{
	return platform_driver_register(&spidev_plat_driver);
}

static void __exit spidev_exit(void)
{
	platform_driver_unregister(&spidev_plat_driver);
}

late_initcall(spidev_init);
module_exit(spidev_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
