/******************************************************************************
* Copyright (C) 2013 Broadcom Corporation
*
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation version 2.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
******************************************************************************/

#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/unistd.h>
#include <linux/bug.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include "bcm_gps_i2c.h"
#include <linux/of_gpio.h>

#define GPS_VERSION	"2.23"
#define PFX			"bcmgps:"

#ifdef CONFIG_SENSORS_SSP_BBD
void bbd_parse_asic_data(unsigned char *pucData, unsigned short usLen, void (*to_gpsd)(unsigned char *packet, unsigned short len, void* priv), void* priv);
#endif

/* ring buffer functions */
typedef struct ring_buffer {
	struct mutex *lock;
	int start;
	int end;
	unsigned int size_of_ring_buffer;
	unsigned char *buffer;
	int  (*push_back_byte)(struct ring_buffer *self, unsigned char byte);
	int  (*pop_front_byte)(struct ring_buffer *self, unsigned char *byte);
	int  (*push_back)(struct ring_buffer *self, unsigned char *buffer, unsigned int sizeOfBuffer);
	int  (*pop_front)(struct ring_buffer *self, unsigned char *buffer, unsigned int sizeOfBuffer);
	int  (*is_empty_internal)(struct ring_buffer *self);
	int  (*is_full_internal)(struct ring_buffer *self);
	int  (*is_empty)(struct ring_buffer *self);
	int  (*is_full)(struct ring_buffer *self);
	void (*reset)(struct ring_buffer *self);
} ring_buffer;

static ring_buffer *ring_buffer_init(int size_of_ring_buffer, struct mutex *lock);
static void ring_buffer_free(ring_buffer *self);
static int  ring_buffer_pushBackByte(struct ring_buffer *self, unsigned char byte);
static int  ring_buffer_popFrontByte(ring_buffer *self, unsigned char *byte);
static int  ring_buffer_pushBack(ring_buffer *self, unsigned char *buffer, unsigned int sizeOfBuffer);
static int  ring_buffer_popFront(ring_buffer *self, unsigned char *buffer, unsigned int sizeOfBuffer);
static int  ring_buffer_isEmptyInternal(ring_buffer *self);
static int  ring_buffer_isFullInternal(ring_buffer *self);
static int  ring_buffer_isEmpty(ring_buffer *self);
static int  ring_buffer_isFull(ring_buffer *self);
static void ring_buffer_reset(ring_buffer *self);

#define MAX_RX_PCKT_LEN  256
#define MAX_TX_PCKT_LEN  256

#define MAX_TEMP_READ_BUFFER (256 * 2)

#define SIZE_OF_READ_RING_BUFFER 256*128
#define SIZE_OF_WRITE_RING_BUFFER 256*64

static DEFINE_MUTEX(gps_lock);
static DEFINE_MUTEX(gps_read_lock);
static DEFINE_MUTEX(gps_write_lock);

struct gps_irq {
	wait_queue_head_t wait;
	int irq;
	int host_req_pin;
	struct miscdevice misc;
	struct i2c_client *client;
	ring_buffer *read_rbuf;
	ring_buffer *write_rbuf;
	struct work_struct read_task;
	struct work_struct write_task;
	int is_opened;
};


bool ssi_dbg;

static unsigned long init_time = 0;
static unsigned long clock_get_ms(void)
{
	struct timeval t;
	unsigned long now;

	do_gettimeofday(&t);
	now = t.tv_usec / 1000 + t.tv_sec * 1000;
	if ( init_time == 0 )
		init_time = now;

	return now - init_time;
}

static void pk_log(char* dir, unsigned char* data, int len)
{
	char acB[960];
	char *p = acB;
	int  the_trans = len;
	int i,j,n;

	char ic = 'D';
	char xc = dir[0] == 'r' || dir[0] == 'w' ? 'x':'X';

	if (likely(!ssi_dbg))
		return;

	/*FIXME: There is print issue. Printing 7 digits instead of 6 when clock is over 1000000. "% 1000000" added
	 * E.g.
	 *#999829D w 0x68,	 1: A2
	 *#999829D r 0x68,	34: 8D 00 01 52 5F B0 01 B0 00 8E 00 01 53 8B B0 01 B0 00 8F 00 01 54 61 B0 01 B0 00 90 00 01 55 B5
	 *		 r			  B0 01
	 *#1000001D w 0x68,	 1: A1
	 *#1000001D r 0x68,	 1: 00
	 */
	n = len;
	p += snprintf(acB,sizeof(acB),"#%06ld%c %2s,	  %5d: ",
			clock_get_ms() % 1000000,ic, dir, n);

	for (i=0, n=32; i<len; i=j, n=32)
	{
		for(j = i; j < (i + n) && j < len && the_trans != 0; j++,the_trans--) {
			p += snprintf(p,sizeof(acB) - (p - acB), "%02X ", data[j]);
		}
		pr_info("%s\n",acB);
		if(j < len) {
			p = acB;
			if ( the_trans == 0 )  { dir[0] = xc; the_trans--; }
			p += snprintf(acB,sizeof(acB),"		 %2s			  ",dir);
		}
	}

	if (i==0)
		pr_info("%s\n",acB);
}



void read_work(struct work_struct *work)
{
	struct gps_irq *ac_data =
		container_of(work, struct gps_irq, read_task);

	int number_of_read = 0;
	unsigned char size_of_packet = 0;
	unsigned char temp_read_buffer[MAX_RX_PCKT_LEN] = {0};
	char host_req_gpio_value = 0;

	while(1)
	{
		/* 1. check conditions. */
		if (ac_data->is_opened == 0) break;

		host_req_gpio_value = gpio_get_value(ac_data->host_req_pin);
		if (host_req_gpio_value == 0) break;

		/* 2. Initialize values. */
		size_of_packet = 0;
		memset(temp_read_buffer, 0x00, MAX_RX_PCKT_LEN);

		/* 3. Read the length of message. */
		number_of_read = i2c_master_recv(ac_data->client, &size_of_packet, 1);
		pk_log("r", &size_of_packet, 1);

		/* 4. check the length, if it is zero, sender doesn't have data anymore. */
		if (size_of_packet == 0) break;

		/* 5. Read message. */
		number_of_read = i2c_master_recv(ac_data->client, temp_read_buffer, (size_of_packet + 1));
		pk_log("r", temp_read_buffer, size_of_packet+1);

		/* 6. push message into ring buffer. */
		/*	-- push message without first byte that is the length of packet. */
		ac_data->read_rbuf->push_back(ac_data->read_rbuf, &(temp_read_buffer[1]), size_of_packet);

	/* 6-1. Call BBD */
#ifdef CONFIG_SENSORS_SSP_BBD
	bbd_parse_asic_data(&temp_read_buffer[1], size_of_packet, NULL, NULL);
#endif

		/* 7. wake up the task for poll. */
		wake_up_interruptible(&ac_data->wait);
	}
}

void write_work(struct work_struct *work)
{
	struct gps_irq *ac_data =
		container_of(work, struct gps_irq, write_task);

	int number_of_sent = 0;
	int size_of_packet = 0;
	unsigned char temp_write_buffer[MAX_TX_PCKT_LEN] = {0};
	int number_of_i2c_sent = 0;

	while(1)
	{
		/* 1. check condition */
		if (ac_data->write_rbuf->is_empty(ac_data->write_rbuf)) break;
		if (ac_data->is_opened == 0) break;

		/* 2. initialize values */
		memset(temp_write_buffer, 0x00, MAX_TX_PCKT_LEN);

		/* 3. read data from ring buffer */
		size_of_packet = ac_data->write_rbuf->pop_front(ac_data->write_rbuf, temp_write_buffer, MAX_TX_PCKT_LEN);

		/* 4. send them through i2c */
		number_of_i2c_sent = i2c_master_send(ac_data->client, temp_write_buffer, size_of_packet);
		pk_log("w", temp_write_buffer, size_of_packet);

		number_of_sent += size_of_packet;

	}

	pr_debug(PFX PFX "write_work() : addr = 0x%02X, sent %d bytes.\n", ac_data->client->addr, number_of_sent);
}

irqreturn_t gps_irq_handler(int irq, void *dev_id)
{
	struct gps_irq *ac_data = dev_id;
	char gpio_value = 0x00;

	gpio_value = gpio_get_value(ac_data->host_req_pin);

	/* If HOST_REQ is set, start to read data. */
	if (gpio_value)
		schedule_work(&ac_data->read_task);

	return IRQ_HANDLED;
}

static int gps_irq_open(struct inode *inode, struct file *filp)
{
	struct gps_irq *ac_data = container_of(filp->private_data,
			struct gps_irq,
			misc);
	int ret = 0;

	mutex_lock(&gps_lock);
	if (ac_data->is_opened == 1)
	{
		ret = -EBUSY;
	}
	else
	{
		ac_data->is_opened = 1;
	}
	mutex_unlock(&gps_lock);

	if (ret < 0)
	{
		pr_err(PFX "open error(%d)\n", ret);
		return ret;
	}

	pr_notice(PFX "gps_irq_open() : 0x%p\n", ac_data);

	filp->private_data = ac_data;

	ac_data->read_rbuf->reset(ac_data->read_rbuf);
	ac_data->write_rbuf->reset(ac_data->write_rbuf);

	return ret;
}

static int gps_irq_release(struct inode *inode, struct file *filp)
{
	struct gps_irq *ac_data = filp->private_data;

	mutex_lock(&gps_lock);
	ac_data->is_opened  = 0;
	mutex_unlock(&gps_lock);

	filp->private_data = ac_data;

	pr_notice(PFX "gps_irq_release() ac_data = 0x%p\n", ac_data);

	return 0;
}

static unsigned int gps_irq_poll(struct file *file, poll_table *wait)
{
	struct gps_irq *ac_data = file->private_data;

	BUG_ON(!ac_data);

	poll_wait(file, &ac_data->wait, wait);

	if (!(ac_data->read_rbuf->is_empty(ac_data->read_rbuf)))
		return POLLIN | POLLRDNORM;

	return 0;
}

static ssize_t gps_irq_read(struct file *file, char __user *buf,
		size_t count, loff_t *offset)
{
	struct gps_irq *ac_data = file->private_data;
	int number_of_read = 0;
	int size_of_pop = 0;
	int number_of_popped = 0;
	unsigned char temp_read_buffer[MAX_TEMP_READ_BUFFER] = {0};

	while (1)
	{
		/* 1. check condition */
		if (ac_data->read_rbuf->is_empty(ac_data->read_rbuf)) break; /* no more data in ring buffer */
		if (number_of_read >= count) break; /* user buf is full. */

		/* 2. initialize values */
		memset(temp_read_buffer, 0x00, MAX_RX_PCKT_LEN);

		/* 3. read data from ring buffer */
		size_of_pop = (count - number_of_read) < MAX_TEMP_READ_BUFFER ? (count - number_of_read) : MAX_TEMP_READ_BUFFER;
		number_of_popped = ac_data->read_rbuf->pop_front(ac_data->read_rbuf, temp_read_buffer, size_of_pop);

		/* 4. copy data to user */
		if (copy_to_user((buf + number_of_read), temp_read_buffer, number_of_popped))
		{
			/* reset ring buffer */
			ac_data->read_rbuf->reset(ac_data->read_rbuf);
			return -EFAULT;
		}
		number_of_read += number_of_popped;
	}

	pr_debug(PFX "gps_irq_read() : # of read = %d\n", number_of_read);

	return number_of_read;
}

static ssize_t gps_irq_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct gps_irq *ac_data = file->private_data;

	unsigned char temp_write_buffer[MAX_TX_PCKT_LEN] = {0};
	int number_of_sent = 0;
	int number_of_push = 0;
	int number_of_pushed = 0;

	while (1) {
		/* 1. initialize values */
		memset(temp_write_buffer, 0x00, MAX_TX_PCKT_LEN);

		/* 2. copy buf to kernel */
		number_of_push = (count - number_of_sent) < MAX_TX_PCKT_LEN ? (count - number_of_sent) : MAX_TX_PCKT_LEN;
		if (copy_from_user(temp_write_buffer, (buf + number_of_sent), number_of_push))
		{
			/* reset write ring buffer. */
			ac_data->write_rbuf->reset(ac_data->write_rbuf);
			return -EFAULT;
		}
		/* 3. push data to ring buffer */
		number_of_pushed = ac_data->write_rbuf->push_back(ac_data->write_rbuf, temp_write_buffer, number_of_push);
		number_of_sent += number_of_pushed;

		/* 4. check escape condition */
		if (number_of_sent >= count) break; /* done */
		if (number_of_push != number_of_pushed) break; /* write_rbuf is full. */
	}

	/* 4. schedule write work */
	schedule_work(&ac_data->write_task);

	pr_debug(PFX "gps_irq_write() : writing %d bytes returns %d\n", count, number_of_sent);

	return number_of_sent;
}


static long gps_irq_ioctl( struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct gps_irq *ac_data = filp->private_data;
	struct i2c_client *client = ac_data->client;

	BUG_ON(!client);

	/* dev_dbg(&client->dev, "ioctl: cmd = 0x%02x, arg=0x%02lx\n", cmd, arg); */
	pr_info(PFX "cmd = 0x%02x, arg=0x%02lx\n", cmd, arg);

	switch (cmd) {
		case I2C_SLAVE:
		case I2C_SLAVE_FORCE:
			if (arg > 0x7f) {
				pr_err(PFX "out of range: 0x%x should be less than 0x7f.", (unsigned int)arg);
				return -EINVAL;
			}
			client->addr = arg;
			pr_info(PFX "ioctl: client->addr = 0x%x\n", client->addr);
			break;

		case I2C_RETRIES:
			client->adapter->retries = arg;
			pr_info("ioctl, client->adapter->retries = %d\n",client->adapter->retries);
			break;

		case I2C_TIMEOUT:
			/* For historical reasons, user-space sets the timeout
			 * value in units of 10 ms.
			 */
			client->adapter->timeout = msecs_to_jiffies(arg * 10);
			pr_info(PFX "ioctl, client->adapter->timeout = %d\n",client->adapter->timeout);
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}

static const struct file_operations gps_irq_fops = {
	.owner = THIS_MODULE,
	.open = gps_irq_open,
	.release = gps_irq_release,
	.poll = gps_irq_poll,
	.read = gps_irq_read,
	.write = gps_irq_write,
	.unlocked_ioctl = gps_irq_ioctl
};

static int gps_hostwake_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct bcm_gps_platform_data *pdata = (struct bcm_gps_platform_data*)client->dev.platform_data;
	struct gps_irq *ac_data = kzalloc(sizeof(struct gps_irq), GFP_KERNEL);

	int irq = -1;
	int ret = -1;
	int err = -1;
	unsigned int gps_gpio_i2c = 0;

#ifdef CONFIG_OF
	enum of_gpio_flags flags;
	if (!client->dev.of_node) {
		pr_err(PFX "of_node of 475x i2c device is NULL\n");
		return -ENODEV;
	}
	gps_gpio_i2c = of_get_named_gpio_flags(client->dev.of_node, "brcm,irq_gpio", 0, &flags);
	if (gps_gpio_i2c < 0) {
		pr_err(PFX "475x host wake gpio %d err\n", gps_gpio_i2c);
		return -1;
	}
	pr_notice(PFX "%s pdata->gpio_i2c = %d, ac_data = 0x%p\n",__func__, gps_gpio_i2c, ac_data);
#else
	gps_gpio_i2c = pdata->gpio_i2c;
	pr_notice(PFX "%s pdata->gpio_i2c = %d, ac_data = 0x%p\n",__func__, pdata->gpio_i2c, ac_data);
#endif

	ac_data->read_rbuf  = ring_buffer_init(SIZE_OF_READ_RING_BUFFER, &gps_read_lock);
	ac_data->write_rbuf = ring_buffer_init(SIZE_OF_WRITE_RING_BUFFER, &gps_write_lock);
	ac_data->is_opened  = 0;

	pr_notice(PFX "%s\n",__func__);

	init_waitqueue_head(&ac_data->wait);

	if ((err = gpio_request(gps_gpio_i2c, "gps_irq"))) {
		pr_warning(PFX "Can't request HOST_REQ GPIO %d.It may be already registered in init.xyz.3rdparty.rc/init.xyz.rc\n",gps_gpio_i2c);
		/*	   gpio_free(gps_gpio_i2c);  */
		/*	   return -EIO; */
	}
	gpio_export(gps_gpio_i2c, 1);
	gpio_direction_input(gps_gpio_i2c);

	irq = gpio_to_irq(gps_gpio_i2c);
	if (irq < 0) {
		pr_err(PFX KERN_ERR "Could not get GPS IRQ = %d!\n",gps_gpio_i2c);
		gpio_free(gps_gpio_i2c);
		return -EIO;
	}

	ac_data->irq = irq;
	ac_data->host_req_pin = gps_gpio_i2c;
	ret = request_irq(irq, gps_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			/* KOM -- IRQF_TRIGGER_RISING, */
			"gps_interrupt",
			ac_data);

	ac_data->client = client;

	ac_data->misc.minor = MISC_DYNAMIC_MINOR;
	ac_data->misc.name = "gps_irq";
	ac_data->misc.fops = &gps_irq_fops;

	pr_notice(PFX "%s misc register, name = %s, irq = %d, GPS gpio =  %d\n",__func__, ac_data->misc.name, irq, gps_gpio_i2c);
	if (0 != (ret = misc_register(&ac_data->misc))) {
		pr_err(PFX "cannot register gps miscdev on minor=%d (%d)\n",MISC_DYNAMIC_MINOR, ret);
		free_irq(ac_data->irq, ac_data);
		gpio_free(gps_gpio_i2c);
		return ret;
	}

	INIT_WORK(&ac_data->read_task, read_work);
	INIT_WORK(&ac_data->write_task, write_work);

	i2c_set_clientdata(client, ac_data);

	pr_notice(PFX "Initialized.\n");

	return 0;
}

static int gps_hostwake_remove(struct i2c_client *client)
{
	struct bcm_gps_platform_data *pdata = 0;
	struct gps_irq *ac_data = 0;

	pr_notice(PFX " %s : called\n", __func__);

	pdata = client->dev.platform_data;
	gpio_free(pdata->gpio_i2c);

	ac_data = i2c_get_clientdata(client);
	free_irq(ac_data->irq, ac_data);
	misc_deregister(&ac_data->misc);

	cancel_work_sync(&ac_data->read_task);
	cancel_work_sync(&ac_data->write_task);

	ring_buffer_free(ac_data->read_rbuf);
	ring_buffer_free(ac_data->write_rbuf);
	kfree(ac_data);
	return 0;
}

static const struct i2c_device_id gpsi2c_id[] = {
	{"gpsi2c", 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id gps_match_table[] = {
	{ .compatible = "brcm,gps",},
	{},
};
#endif

static struct i2c_driver gps_driver = {
	.id_table = gpsi2c_id,
	.probe = gps_hostwake_probe,
	.remove = gps_hostwake_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "gpsi2c",
#ifdef CONFIG_OF
		.of_match_table = gps_match_table,
#endif
	},
};

static int gps_irq_init(void)
{
	pr_notice(PFX "Broadcom GPS i2c Driver v%s\n", GPS_VERSION);
	return i2c_add_driver(&gps_driver);
}

static void gps_irq_exit(void)
{
	i2c_del_driver(&gps_driver);
}

/* ring_buffer implementation */
static ring_buffer *ring_buffer_init(int size_of_ring_buffer, struct mutex *lock)
{
	ring_buffer *new_rb = 0;

	new_rb = kzalloc(sizeof(ring_buffer), GFP_KERNEL);

	new_rb->lock = lock;
	new_rb->push_back_byte = ring_buffer_pushBackByte;
	new_rb->pop_front_byte = ring_buffer_popFrontByte;
	new_rb->push_back = ring_buffer_pushBack;
	new_rb->pop_front = ring_buffer_popFront;
	new_rb->is_empty_internal = ring_buffer_isEmptyInternal;
	new_rb->is_full_internal = ring_buffer_isFullInternal;
	new_rb->is_empty = ring_buffer_isEmpty;
	new_rb->is_full = ring_buffer_isFull;
	new_rb->reset = ring_buffer_reset;

	new_rb->start = 0;
	new_rb->end   = 0;
	new_rb->size_of_ring_buffer = size_of_ring_buffer;

	new_rb->buffer = kzalloc(new_rb->size_of_ring_buffer, GFP_KERNEL);

	return new_rb;
}

static void ring_buffer_free(ring_buffer *self)
{
	self->start = 0;
	self->end   = 0;
	self->size_of_ring_buffer = 0;

	kfree(self->buffer);
	kfree(self);

	self = 0;
}

static int  ring_buffer_pushBackByte(struct ring_buffer *self, unsigned char byte)
{
	if (self->is_full_internal(self)) return 0;

	self->buffer[self->end] = byte;
	self->end = (self->end + 1) % self->size_of_ring_buffer;

	return 1;
}

static int  ring_buffer_popFrontByte(ring_buffer *self, unsigned char *byte)
{
	if (self->is_empty_internal(self)) return 0;

	*byte = self->buffer[self->start];
	self->start = (self->start + 1) % self->size_of_ring_buffer;

	return 1;
}

static int  ring_buffer_pushBack(ring_buffer *self, unsigned char *buffer, unsigned int sizeOfBuffer)
{
	int i = 0;
	int nPush = 0;

	if (self->lock) mutex_lock(self->lock);
	for (i = 0; i < sizeOfBuffer; i++)
	{
		if (self->push_back_byte(self, buffer[i]) == 0) break;
		++nPush;
	}
	if (self->lock) mutex_unlock(self->lock);

	return nPush;
}

static int  ring_buffer_popFront(ring_buffer *self, unsigned char *buffer, unsigned int sizeOfBuffer)
{
	int i = 0;
	int nPop = 0;
	unsigned char byte;

	if (self->lock) mutex_lock(self->lock);
	for (i = 0; i < sizeOfBuffer; i++)
	{
		if (self->pop_front_byte(self, &byte) == 0) break;
		buffer[i] = byte;
		++nPop;
	}
	if (self->lock) mutex_unlock(self->lock);

	return nPop;
}

static int  ring_buffer_isEmptyInternal(ring_buffer *self)
{
	int ret = 0;

	ret = (self->start == self->end);

	return ret;
}

static int  ring_buffer_isFullInternal(ring_buffer *self)
{
	int ret = 0;

	if ((self->start - 1) > 0)
	{
		ret = ((self->start - 1) == self->end);
	}
	else if (self->end == (self->size_of_ring_buffer - 1))
	{
		ret = 1;
	}

	return ret;
}

static int  ring_buffer_isEmpty(ring_buffer *self)
{
	int ret = 0;

	if (self->lock) mutex_lock(self->lock);
	ret = (self->start == self->end);
	if (self->lock) mutex_unlock(self->lock);

	return ret;
}

static int  ring_buffer_isFull(ring_buffer *self)
{
	int ret = 0;

	if (self->lock) mutex_lock(self->lock);
	if ((self->start - 1) > 0)
	{
		ret = ((self->start - 1) == self->end);
	}
	else if (self->end == (self->size_of_ring_buffer - 1))
	{
		ret = 1;
	}
	if (self->lock) mutex_unlock(self->lock);

	return ret;
}

static void ring_buffer_reset(ring_buffer *self)
{
	if (self->lock) mutex_lock(self->lock);
	self->start = 0;
	self->end   = 0;
	if (self->lock) mutex_unlock(self->lock);
}

module_init(gps_irq_init);
module_exit(gps_irq_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom GPS Driver with host wake interrupt");
