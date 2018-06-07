/******************************************************************************
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kthread.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/kernel_stat.h>

#include "bbd.h"

#define WORD_BURST_SIZE			4
#define CONFIG_SPI_DMA_BYTES_PER_WORD	4
#define CONFIG_SPI_DMA_BITS_PER_WORD	32

#define SSI_MODE_STREAM		0x00
#define SSI_MODE_DEBUG		0x80

#define SSI_MODE_HALF_DUPLEX	0x00
#define SSI_MODE_FULL_DUPLEX	0x40

#define SSI_WRITE_TRANS		0x00
#define SSI_READ_TRANS		0x20

#define SSI_WRITE_HD (SSI_WRITE_TRANS | SSI_MODE_HALF_DUPLEX)
#define SSI_READ_HD  (SSI_READ_TRANS  | SSI_MODE_HALF_DUPLEX)

#define DEBUG_TIME_STAT

#ifdef CONFIG_SENSORS_SSP_BBD
extern void bbd_parse_asic_data(unsigned char *pucData, unsigned short usLen, void (*to_gpsd)(unsigned char *packet, unsigned short len, void* priv), void* priv);
#endif

bool ssi_dbg;

/*
 *
 *			   Structs
 *
 */
#define BCM_SPI_READ_BUF_SIZE	(8*PAGE_SIZE)
#define BCM_SPI_WRITE_BUF_SIZE	(8*PAGE_SIZE)

struct bcm_ssi_tx_frame
{
	unsigned char cmd;
	unsigned char data[BCM_SPI_WRITE_BUF_SIZE-1];
} __attribute__((__packed__));

struct bcm_ssi_rx_frame
{
	unsigned char status;
	unsigned char len;
	unsigned char data[BCM_SPI_READ_BUF_SIZE-2];
} __attribute__((__packed__));

struct bcm_spi_priv
{
	struct spi_device *spi;

	/* Char device stuff */
	struct miscdevice misc;
	bool busy;
	struct circ_buf read_buf;
	struct circ_buf write_buf;
	struct mutex rlock;			/* Lock for read_buf */
	struct mutex wlock;			/* Lock for write_buf */
	char _read_buf[BCM_SPI_READ_BUF_SIZE];
	char _write_buf[BCM_SPI_WRITE_BUF_SIZE];
	wait_queue_head_t poll_wait;		/* for poll */

		/* GPIO pins */
	int host_req;
	int mcu_req;
	int mcu_resp;

		/* IRQ and its control */
	atomic_t irq_enabled;
	spinlock_t irq_lock;

	/* Work */
	struct work_struct rxtx_work;
	struct workqueue_struct *serial_wq;
	atomic_t suspending;

	/* SPI tx/rx buf */
	struct bcm_ssi_tx_frame *tx_buf;
	struct bcm_ssi_rx_frame *rx_buf;

	struct wake_lock bcm_wake_lock;
};

static struct bcm_spi_priv *g_bcm_gps;

/*
 *
 *			   File Operations
 *
 */
static int bcm_spi_open(struct inode *inode, struct file *filp)
{
	/* Initially, file->private_data points device itself and we can get our priv structs from it. */
	struct bcm_spi_priv *priv = container_of(filp->private_data, struct bcm_spi_priv, misc);
	unsigned long int flags;

	pr_info("%s++\n", __func__);

	if (priv->busy)
		return -EBUSY;

	priv->busy = true;

	/* Reset circ buffer */
	priv->read_buf.head = priv->read_buf.tail = 0;
	priv->write_buf.head = priv->write_buf.tail = 0;


	/* Enable irq */
	spin_lock_irqsave( &priv->irq_lock, flags);
	if (!atomic_xchg(&priv->irq_enabled, 1))
		enable_irq(priv->spi->irq);

	spin_unlock_irqrestore( &priv->irq_lock, flags);

	enable_irq_wake(priv->spi->irq);

	filp->private_data = priv;
#ifdef DEBUG_1HZ_STAT
	bbd_enable_stat();
#endif
	pr_info("%s--\n", __func__);
	return 0;
}

static int bcm_spi_release(struct inode *inode, struct file *filp)
{
	struct bcm_spi_priv *priv = filp->private_data;
	unsigned long int flags;

	pr_info("%s++\n", __func__);
	priv->busy = false;

#ifdef DEBUG_1HZ_STAT
	bbd_disable_stat();
#endif
	/* Disable irq */
	spin_lock_irqsave( &priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(priv->spi->irq);

	spin_unlock_irqrestore( &priv->irq_lock, flags);

	disable_irq_wake(priv->spi->irq);

	pr_info("%s--\n", __func__);
	return 0;
}

static ssize_t bcm_spi_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *circ = &priv->read_buf;
	size_t rd_size=0;

	/* pr_err("[SSPBBD] %s++\n", __func__); */
	mutex_lock(&priv->rlock);

	/* Copy from circ buffer to user
	 * We may require 2 copies from [tail..end] and [end..head]
	 */
	do {
		size_t cnt_to_end = CIRC_CNT_TO_END(circ->head, circ->tail, BCM_SPI_READ_BUF_SIZE);
		size_t copied = min(cnt_to_end, size);

		WARN_ON(copy_to_user(buf + rd_size, (void*) circ->buf + circ->tail, copied));
		size -= copied;
		rd_size += copied;
		circ->tail = (circ->tail + copied) & (BCM_SPI_READ_BUF_SIZE-1);

	} while (size>0 && CIRC_CNT(circ->head, circ->tail, BCM_SPI_READ_BUF_SIZE));

	mutex_unlock(&priv->rlock);

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(STAT_RX_LHD, rd_size);
#endif

	/* pr_err("[SSPBBD] %s--(%zd bytes)\n", __func__, rd_size); */
	return rd_size;
}

static ssize_t bcm_spi_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *circ = &priv->write_buf;
	size_t wr_size=0;

	/* pr_err("[SSPBBD] %s++(%zd bytes)\n", __func__, size); */

	mutex_lock(&priv->wlock);
	/* Copy from user into circ buffer
	 * We may require 2 copies from [tail..end] and [end..head]
	 */
	do {
		size_t space_to_end = CIRC_SPACE_TO_END(circ->head, circ->tail, BCM_SPI_WRITE_BUF_SIZE);
		size_t copied = min(space_to_end, size);


		WARN_ON(copy_from_user((void*) circ->buf + circ->head, buf + wr_size, copied));
		size -= copied;
		wr_size += copied;
		circ->head = (circ->head + copied) & (BCM_SPI_WRITE_BUF_SIZE-1);

	} while (size>0 && CIRC_SPACE(circ->head, circ->tail, BCM_SPI_WRITE_BUF_SIZE));

	mutex_unlock(&priv->wlock);

	/* pr_err("[SSPBBD] %s--(%zd bytes)\n", __func__, wr_size); */

	/* kick start rxtx thread */
	/* we don't want to queue work in suspending and shutdown */
	if (!atomic_read(&priv->suspending))
		queue_work(priv->serial_wq, &(priv->rxtx_work) );

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(STAT_TX_LHD, wr_size);
#endif
	return wr_size;
}

static unsigned int bcm_spi_poll(struct file *filp, poll_table *wait)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *rd_circ = &priv->read_buf;
	struct circ_buf *wr_circ = &priv->write_buf;
	unsigned int mask = 0;

	poll_wait(filp, &priv->poll_wait, wait);

	if (CIRC_CNT(rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE))
		mask |= POLLIN;

	if (CIRC_SPACE(wr_circ->head, wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE))
		mask |= POLLOUT;

	return mask;
}


static const struct file_operations bcm_spi_fops = {
	.owner		  =  THIS_MODULE,
	.open		   =  bcm_spi_open,
	.release		=  bcm_spi_release,
	.read		   =  bcm_spi_read,
	.write		  =  bcm_spi_write,
	.poll		   =  bcm_spi_poll,
};



/*
 * bcm4773_hello - wakeup chip by toggling mcu_req while monitoring mcu_resp to check if awake
 *
 */
static bool bcm4773_hello(struct bcm_spi_priv *priv)
{
	int count=0, retries=0;

	gpio_set_value(priv->mcu_req, 1);
	/* pr_err("[SSPBBD]: gpio_get_value =%d\n", gpio_get_value(priv->mcu_req)); */
	while (!gpio_get_value(priv->mcu_resp)) {
		if (count++ > 10000) {
			gpio_set_value(priv->mcu_req, 0);
			pr_err("[SSPBBD] MCU_REQ_RESP timeout. MCU_RESP(gpio%d) not responding to MCU_REQ(gpio%d)\n", 
				priv->mcu_resp, priv->mcu_req);
			return false;
		}

		mdelay(1);

		/*if awake, done */
		if (gpio_get_value(priv->mcu_resp)) break;

		if (count%20==0 && retries++ < 3) {
			gpio_set_value(priv->mcu_req, 0);
			mdelay(1);
			gpio_set_value(priv->mcu_req, 1);
			mdelay(1);
		}
	}
	return true;
}

/*
 * bcm4773_bye - set mcu_req low to let chip go to sleep
 *
 */
static void bcm4773_bye(struct bcm_spi_priv *priv)
{
	gpio_set_value(priv->mcu_req, 0);
}


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
		pr_err("%s\n",acB);
		if(j < len) {
			p = acB;
			if ( the_trans == 0 )  { dir[0] = xc; the_trans--; }
			p += snprintf(acB,sizeof(acB),"		 %2s			  ",dir);
		}
	}

	if (i==0)
		pr_err("%s\n",acB);
}



/*
 *
 *			   SSI tx/rx functions
 *
 */
static int bcm_spi_sync(struct bcm_spi_priv *priv, void *tx_buf, void *rx_buf, int len, int bits_per_word)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret;

	/* Init */
	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(xfer));
	spi_message_add_tail(&xfer, &msg);

	/* Setup */
	msg.spi = priv->spi;
	msg.spi->mode |= (SPI_CPHA | SPI_CPOL);
	xfer.len = len;
	xfer.tx_buf = tx_buf;
	xfer.rx_buf = rx_buf;

	/* Sync */
	pk_log("w", (unsigned char *)xfer.tx_buf, len);
	ret = spi_sync(msg.spi, &msg);
	/* pr_err("[SSPBBD} msg.spi->mode =%d\n", msg.spi->mode); */
	pk_log("r", (unsigned char *)xfer.rx_buf, len);

	if (ret)
		pr_err("[SSPBBD} spi_sync error, return=%d\n", ret);

	return ret;
}


static int bcm_ssi_tx(struct bcm_spi_priv *priv, int length)
{
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;
	int bits_per_word = (length >= 255) ? CONFIG_SPI_DMA_BITS_PER_WORD : 8;
	int ret;

	/* pr_err("[SSPBBD]: %s ++ (%d bytes)\n", __func__, length); */

	tx->cmd = SSI_WRITE_HD;

	ret = bcm_spi_sync(priv, tx, rx, length+1, bits_per_word); //1 for tx cmd

	/* pr_err("[SSPBBD]: %s --\n", __func__); */
	return ret;
}

static int bcm_ssi_rx( struct bcm_spi_priv *priv, size_t *length)
{
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;
	const unsigned char MAX_RX_LEN = 254;

	/* pr_err("[SSPBBD]: %s ++\n", __func__); */

	memset(tx, 0, MAX_RX_LEN);
	tx->cmd = SSI_READ_HD;
	rx->status = 0;
	if (bcm_spi_sync(priv, tx, rx, 2, 8))  /* 2 for rx status + len */
		return -1;

	/* Check Sanity */
	if (rx->status) {
		pr_err("[SSPBBD] spi_sync error, status = 0x%02X\n", rx->status);
	}

	if (rx->len == 0) {
		rx->len = MAX_RX_LEN;
		pr_err("[SSPBBD] rx->len is still read to 0. set MAX_RX_LEN\n");
	}

	/* Max SSI payload length 255 and max SPI transfer size is 255+2 = 257 which will require DMA
	 * But, we don't want DMA because sometimes it's buggy. So, limit max payload to 254
	 */
	*length = min(MAX_RX_LEN, rx->len);
	if (bcm_spi_sync(priv, tx, rx, *length+2, 8)) /* 2 for rx status + len */
		return -1;

	/* Check Sanity */
	if (rx->status) {
		pr_err("[SSPBBD] spi_sync error, status = 0x%02X\n", rx->status);
	}

	if (rx->len < *length) {
		/* pr_err("[SSPBBD]: %s read error. Expected %zd but read %d\n", __func__, *length, rx->len); */
		*length = rx->len; /* workaround  */
	}

#ifdef DEBUG_1HZ_STAT
	bbd_update_stat(STAT_RX_SSI, *length);
#endif
	/* pr_err("[SSPBBD]: %s -- (%d bytes)\n", __func__, *length);  */

	return 0;
}

void bcm_on_packet_recieved(void *_priv, unsigned char *data, unsigned int size)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv *)_priv;
	struct circ_buf *rd_circ = &priv->read_buf;
	size_t written=0, avail = size;

	/* pr_info("[SSPBBD]: %s ++\n", __func__); */
	/* Copy into circ buffer */
	mutex_lock(&priv->rlock);
	do {
		size_t space_to_end = CIRC_SPACE_TO_END(rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE);
		size_t copied = min(space_to_end, avail);

		memcpy((void*) rd_circ->buf + rd_circ->head, data + written, copied);
		avail -= copied;
		written += copied;
		rd_circ->head = (rd_circ->head + copied) & (BCM_SPI_READ_BUF_SIZE-1);

	} while (avail>0 && CIRC_SPACE(rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE));
	mutex_unlock(&priv->rlock);
	wake_up(&priv->poll_wait);

	if (avail>0)
		pr_err("[SSPBBD]: input overrun error by %zd bytes!\n", avail);

	/* pr_info("[SSPBBD]: %s received -- (%d bytes)\n", __func__, size); */
}

static void bcm_rxtx_work( struct work_struct *work )
{
#ifdef DEBUG_1HZ_STAT
	u64 ts_rx_start = 0;
	u64 ts_rx_end = 0;
#endif
	struct bcm_spi_priv *priv = container_of(work, struct bcm_spi_priv, rxtx_work);
	struct circ_buf *wr_circ = &priv->write_buf;

	if (!bcm4773_hello(priv)) {
		pr_err("[SSPBBD]: %s timeout!!\n", __func__);
		return;
	}

	do {
		/* Read first */
		/* pr_err("[SSPBBD]before read ssp-host-req=%d\n", gpio_get_value(priv->host_req)); */
		if (gpio_get_value(priv->host_req)) {
			size_t avail;
#ifdef DEBUG_1HZ_STAT
			struct timespec ts;
			if (stat1hz.ts_irq) {
				ts = ktime_to_timespec(ktime_get_boottime());
				ts_rx_start = ts.tv_sec * 1000000000ULL
					   			+ ts.tv_nsec;
			}
#endif
			/* Receive SSI frame */
			if (bcm_ssi_rx(priv, &avail))
				break;

#ifdef DEBUG_1HZ_STAT
			bbd_update_stat(STAT_RX_SSI, avail);
			if (ts_rx_start && !gpio_get_value(priv->host_req)) {
				   	ts = ktime_to_timespec(ktime_get_boottime());
				ts_rx_end = ts.tv_sec * 1000000000ULL
					  			+ ts.tv_nsec;
			}
#endif
			/* Call BBD */
			bbd_parse_asic_data(priv->rx_buf->data, avail, NULL, priv);
		}

		/* Next, write */
		if (CIRC_CNT(wr_circ->head, wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE)) {
			size_t written=0, avail=0;

			mutex_lock(&priv->wlock);

			avail = CIRC_CNT(wr_circ->head, wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE);
			/* For big packet, we should align xfer size to DMA word size and burst size.
			 * That is, SSI payload + one byte command should be multiple of (DMA word size * burst size)
			 */
			if (avail + 1 > 256)
				avail -= (avail % (CONFIG_SPI_DMA_BYTES_PER_WORD * WORD_BURST_SIZE)) + 1; //1 for "1 byte cmd"

			/* Copy from wr_circ the data */
			do {
				size_t cnt_to_end = CIRC_CNT_TO_END(wr_circ->head, wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE);
				size_t copied = min(cnt_to_end, avail);

				/* pr_info("[SSPBBD]: %s writing ++ (%d bytes)\n", __func__, copied); */

				memcpy(priv->tx_buf->data + written, wr_circ->buf + wr_circ->tail, copied);
				avail -= copied;
				written += copied;
				wr_circ->tail = (wr_circ->tail + copied) & (BCM_SPI_WRITE_BUF_SIZE-1);

				/* pr_info("[SSPBBD]: %s writing --\n", __func__); */

			} while (avail>0);

			mutex_unlock(&priv->wlock);

			/* Transmit SSI frame */
			if (bcm_ssi_tx(priv, written))
				break;
#ifdef DEBUG_1HZ_STAT
			bbd_update_stat(STAT_TX_SSI, written);
#endif
			wake_up(&priv->poll_wait);
		}

	} while (gpio_get_value(priv->host_req) || CIRC_CNT(wr_circ->head, wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE));

	bcm4773_bye(priv);

	/* Enable irq */
	{
		unsigned long int flags;
		spin_lock_irqsave( &priv->irq_lock, flags);

 		/* we dont' want to enable irq when going to suspending */
		if (!atomic_read(&priv->suspending))
			if (!atomic_xchg(&priv->irq_enabled, 1))
				enable_irq(priv->spi->irq);

		spin_unlock_irqrestore( &priv->irq_lock, flags);
	}

#ifdef DEBUG_1HZ_STAT
	if (stat1hz.ts_irq && ts_rx_start && ts_rx_end) {
		u64 lat = ts_rx_start - stat1hz.ts_irq;
		u64 dur = ts_rx_end - ts_rx_start;
		stat1hz.min_rx_lat = (lat < stat1hz.min_rx_lat) ?
			   			lat : stat1hz.min_rx_lat;
		stat1hz.max_rx_lat = (lat > stat1hz.max_rx_lat) ?
			   			lat : stat1hz.max_rx_lat;
		stat1hz.min_rx_dur = (dur < stat1hz.min_rx_dur) ?
			   			dur : stat1hz.min_rx_dur; 
		stat1hz.max_rx_dur = (dur > stat1hz.max_rx_dur) ?
			   			dur : stat1hz.max_rx_dur;
		stat1hz.ts_irq = 0;
	}
#endif
}


/*
 *
 *			   IRQ Handler
 *
 */
static irqreturn_t bcm_irq_handler(int irq, void *pdata)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv *) pdata;

	if (!gpio_get_value(priv->host_req))
		return IRQ_HANDLED;
#ifdef DEBUG_1HZ_STAT
	{
	struct timespec ts;
	ts = ktime_to_timespec(ktime_get_boottime());
	stat1hz.ts_irq = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
	}
#endif
	/* Disable irq */
	spin_lock(&priv->irq_lock);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(priv->spi->irq);

	spin_unlock(&priv->irq_lock);

	/* we don't want to queue work in suspending and shutdown */
	if (!atomic_read(&priv->suspending))
		queue_work(priv->serial_wq, &priv->rxtx_work);

	/* pr_err("[SSPBBD]: %s ---debug--- \n", __func__); */

	return IRQ_HANDLED;
}


/*
 *
 *			   SPI driver operations
 *
 */
static int bcm_spi_suspend(struct spi_device *spi, pm_message_t state)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv*) spi_get_drvdata(spi);
	unsigned long int flags;

	printk("[SSPBBD]: %s ++ \n", __func__);

	atomic_set(&priv->suspending, 1);

	/* Disable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(spi->irq);

	spin_unlock_irqrestore(&priv->irq_lock, flags);

	flush_workqueue(priv->serial_wq);

	printk("[SSPBBD]: %s -- \n", __func__);
	return 0;
}

static int bcm_spi_resume(struct spi_device *spi)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv*) spi_get_drvdata(spi);
	unsigned long int flags;

	atomic_set(&priv->suspending, 0);

	/* Enable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (!atomic_xchg(&priv->irq_enabled, 1))
		enable_irq(spi->irq);

	spin_unlock_irqrestore( &priv->irq_lock, flags);

	/* wake_lock_timeout(&spi->bcm_wake_lock, HZ/2); */

	return 0;
}

static void bcm_spi_shutdown(struct spi_device *spi)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv*) spi_get_drvdata(spi);
	unsigned long int flags;

	printk("[SSPBBD]: %s ++ \n", __func__);

	atomic_set(&priv->suspending, 1);

	/* Disable irq */
	spin_lock_irqsave( &priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(spi->irq);

	spin_unlock_irqrestore( &priv->irq_lock, flags);

	printk("[SSPBBD]: %s ** \n", __func__);

	flush_workqueue(priv->serial_wq );
	destroy_workqueue( priv->serial_wq );

	printk("[SSPBBD]: %s -- \n", __func__);
}

static int bcm_spi_probe(struct spi_device *spi)
{
	int host_req, mcu_req, mcu_resp, vdd_pmu_in, nstandby;
	struct bcm_spi_priv *priv;
	int pin_ttyBCM, pin_MCU_REQ, pin_MCU_RESP;
	int ret;
	struct gpio_desc *desc;

	/* Check GPIO# */
#ifndef CONFIG_OF
	pr_err("[SSPBBD]: Check platform_data for bcm device\n");
#endif
	if (!spi->dev.of_node) {
		pr_err("[SSPBBD]: Failed to find of_node\n");
		goto err_exit;
	}

	vdd_pmu_in = of_get_named_gpio(spi->dev.of_node, "ssp-vdd-pmu-in", 0);
	nstandby = of_get_named_gpio(spi->dev.of_node, "ssp-nstandby", 0);
	host_req = of_get_named_gpio(spi->dev.of_node, "ssp-host-req", 0);
	mcu_req  = of_get_named_gpio(spi->dev.of_node, "ssp-mcu-req", 0);
	mcu_resp = of_get_named_gpio(spi->dev.of_node, "ssp-mcu-resp", 0);
	if (vdd_pmu_in < 0 || nstandby < 0 || host_req < 0 ||
	     mcu_req < 0 || mcu_resp < 0) {
		pr_err("[SSPBBD]: GPIO value not correct\n");
		goto err_exit;
	}

	/*loadswitch config start*/
	desc = gpio_to_desc(vdd_pmu_in);
	if (!desc) {
		pr_err("[SSPBBD]: vdd_pmu_in invalid gpio_todesc!\n");
		goto err_exit;
	}

	ret = gpio_request(vdd_pmu_in, "gpio30");
	if (ret) {
		pr_err("[SSPBBD]: vdd_pmu_in gpio_request failed!\n");
		goto err_exit;
	}
	ret = gpiod_export(desc, true);
	if (ret) {
		pr_err("[SSPBBD]: vdd_pum_in gpoid_export failed!\n");
		goto err_exit;
	}
	ret = gpio_direction_output(vdd_pmu_in, 1);
	if (ret) {
		pr_err("[SSPBBD]: vdd_pmu_in set direction failed!\n");
		goto err_exit;
	}
	pr_info("GPS loadswitch enable!\n");
	gpio_set_value(vdd_pmu_in, 1);
	msleep(100);
	/*loadswitch config end*/

	/*nstandby config start*/
	desc = gpio_to_desc(nstandby);
	if (!desc) {
		pr_warn("[SSPBBD]: nstandby invalid gpio_todesc!");
		goto err_exit;
	}
	ret = gpio_request(nstandby, "gpio74");
	if (ret) {
		pr_warn("[SSPBBD]: nstandby gpio_request failed!\n");
		goto err_exit;
	}
	ret = gpiod_export(desc, true);
	if (ret) {
		pr_warn("[SSPBBD]: nstandby gpoid_export failed!");
		goto err_exit;
	}

	ret = gpio_direction_output(nstandby, 1);
	if (ret) {
		pr_warn("[SSPBBD]: nstandby set direction failed!\n");
		goto err_exit;
	}
	/*nstandby config end*/

	/* Check IRQ# */
	spi->irq = gpio_to_irq(host_req);
	if (spi->irq < 0) {
		pr_err("[SSPBBD]: irq=%d for host_req=%d not correct\n",
		       spi->irq, host_req);
		goto err_exit;
	}

	/* Config GPIO */
	ret = gpio_request(mcu_req, "MCU REQ");
	pr_err("[SSPBBD]: request MCU REQ, ret:%d", ret);
	if (ret){
		pr_err("[SSPBBD]: failed to request MCU REQ, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(mcu_req, 0);
	if (ret) {
		pr_err("[SSPBBD]: failed set MCU REQ as input mode, ret:%d", ret);
		goto err_exit;
	}

	ret = gpio_request(mcu_resp, "MCU RESP");
	if (ret){
		pr_err("[SSPBBD]: failed to request MCU RESP, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_input(mcu_resp);
	pr_err("[SSPBBD]: set MCU RESP as input mode, ret:%d", ret);
	if (ret) {
		pr_err("[SSPBBD]: failed set MCU RESP as input mode, ret:%d", ret);
		goto err_exit;
	}

	/* Alloc everything */
	priv = (struct bcm_spi_priv*) kmalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		pr_err("[SSPBBD]: Failed to allocate memory for the BCM SPI driver\n");
		goto err_exit;
	}

	memset(priv, 0, sizeof(*priv));

	priv->spi = spi;

	priv->tx_buf = kmalloc(sizeof(struct bcm_ssi_tx_frame), GFP_KERNEL);
	priv->rx_buf = kmalloc(sizeof(struct bcm_ssi_rx_frame), GFP_KERNEL);
	if (!priv->tx_buf || !priv->rx_buf) {
		pr_err("[SSPBBD]: Failed to allocate xfer buffer. tx_buf=%p, rx_buf=%p\n",
			priv->tx_buf, priv->rx_buf);
		goto free_mem;
	}

	priv->serial_wq = alloc_workqueue("bcm4773_wq", WQ_HIGHPRI|WQ_UNBOUND|WQ_MEM_RECLAIM, 1);
	if (!priv->serial_wq) {
		pr_err("[SSPBBD]: Failed to allocate workqueue\n");
		goto free_mem;
	}

	/* Request IRQ */
	ret = request_irq(spi->irq, bcm_irq_handler, IRQF_TRIGGER_HIGH, "ttyBCM", priv);
	if (ret) {
		pr_err("[SSPBBD]: Failed to register BCM4773 SPI TTY IRQ %d.\n",spi->irq);
		goto free_wq;
	}

	disable_irq(spi->irq);

	pr_notice("[SSPBBD]: Probe OK. ssp-host-req=%d, irq=%d, priv=0x%p\n", host_req, spi->irq, priv);

	/* Register misc device */
	priv->misc.minor = MISC_DYNAMIC_MINOR;
	priv->misc.name = "ttyBCM";
	priv->misc.fops = &bcm_spi_fops;

	ret = misc_register(&priv->misc);
	if (ret) {
		pr_err("[SSPBBD]: Failed to register bcm_gps_spi's misc dev. err=%d\n", ret);
		goto free_irq;
	}

	/* Set driver data */
	spi_set_drvdata(spi, priv);

	/* Init - miscdev stuff */
	init_waitqueue_head(&priv->poll_wait);
	priv->read_buf.buf = priv->_read_buf;
	priv->write_buf.buf = priv->_write_buf;
	mutex_init(&priv->rlock);
	mutex_init(&priv->wlock);
	priv->busy = false;

	/* Init - work */
	INIT_WORK(&priv->rxtx_work, bcm_rxtx_work);

	/* Init - irq stuff */
	spin_lock_init(&priv->irq_lock);
	atomic_set(&priv->irq_enabled, 0);
	atomic_set(&priv->suspending, 0);

	/* Init - gpios */
	priv->host_req = host_req;
	priv->mcu_req  = mcu_req;
	priv->mcu_resp = mcu_resp;
	pin_ttyBCM = gpio_get_value(host_req);
	pin_MCU_REQ = gpio_get_value(mcu_req);
	pin_MCU_RESP = gpio_get_value(mcu_resp);
	pr_info("[SSPBBD]: %s pin_ttyBCM:%d, pin_MCU_REQ:%d, pin_MCU_RESP:%d\n",
		__func__, pin_ttyBCM, pin_MCU_REQ, pin_MCU_RESP);

	/* Init - etc */
	wake_lock_init(&priv->bcm_wake_lock, WAKE_LOCK_SUSPEND, "bcm_spi_wake_lock");

	g_bcm_gps = priv;
	/* Init BBD & SSP */
	bbd_init(&spi->dev);

	return 0;

free_irq:
	if (spi->irq)
		free_irq( spi->irq, priv );
free_wq:
	if (priv->serial_wq)
		destroy_workqueue(priv->serial_wq);
free_mem:
	if (priv->tx_buf)
		kfree(priv->tx_buf);
	if (priv->rx_buf)
		kfree(priv->rx_buf);
	if (priv)
		kfree(priv);
err_exit:
	return -ENODEV;
}


static int bcm_spi_remove(struct spi_device *spi)
{
	struct bcm_spi_priv *priv = (struct bcm_spi_priv*) spi_get_drvdata(spi);
	unsigned long int flags;
	int vdd_pmu_in;

	pr_info("[SSPBBD]:  %s : called\n", __func__);

	vdd_pmu_in = of_get_named_gpio(spi->dev.of_node, "ssp-vdd-pmu-in", 0);

	atomic_set(&priv->suspending, 1);

	/* Disable irq */
	spin_lock_irqsave(&priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(spi->irq);

	spin_unlock_irqrestore( &priv->irq_lock, flags);

	/* Flush work */
	flush_workqueue(priv->serial_wq);
	destroy_workqueue(priv->serial_wq);

	/* Free everything */
	free_irq( spi->irq, priv );
	kfree(priv->tx_buf);
	kfree(priv->rx_buf);
	kfree( priv );

	/*Disable GPS loadswitch*/
	if (vdd_pmu_in > 0) {
		pr_info("GPS loadswitch disable\n");
		gpio_set_value(vdd_pmu_in, 0);
	}

	g_bcm_gps = NULL;

	return 0;
}

void bcm4773_debug_info(void)
{
	int pin_ttyBCM, pin_MCU_REQ, pin_MCU_RESP;
	int irq_enabled, irq_count;

	pin_ttyBCM = gpio_get_value(g_bcm_gps->host_req);
	pin_MCU_REQ = gpio_get_value(g_bcm_gps->mcu_req);
	pin_MCU_RESP = gpio_get_value(g_bcm_gps->mcu_resp);

	irq_enabled = atomic_read(&g_bcm_gps->irq_enabled);
	irq_count = kstat_irqs_cpu(g_bcm_gps->spi->irq, 0);

	printk("[SSPBBD]: %s pin_ttyBCM:%d, pin_MCU_REQ:%d, pin_MCU_RESP:%d\n",
			__func__, pin_ttyBCM, pin_MCU_REQ, pin_MCU_RESP);
	printk("[SSPBBD]: %s irq_enabled:%d, irq_count:%d\n",
			__func__, irq_enabled, irq_count);
}


static const struct spi_device_id bcm_spi_id[] = {
	{"ssp-spi", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, bcm_spi_id);

#ifdef CONFIG_OF
static struct of_device_id match_table[] = {
	{ .compatible = "ssp,BCM4773",},
	{},
};
#endif

static struct spi_driver bcm_spi_driver =
{
	.id_table = bcm_spi_id,
	.probe = bcm_spi_probe,
	.remove = bcm_spi_remove,
	.suspend = bcm_spi_suspend,
	.resume = bcm_spi_resume,
	.shutdown = bcm_spi_shutdown,
	.driver = {
		.name = "brcm gps spi",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = match_table
#endif
	},
};


/*
 *
 *			   Module init/exit
 *
 */
static int __init bcm_spi_init(void)
{
	return spi_register_driver(&bcm_spi_driver);
}

static void __exit bcm_spi_exit(void)
{
	spi_unregister_driver(&bcm_spi_driver);
}

module_init(bcm_spi_init);
module_exit(bcm_spi_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM SPI/SSI Driver");

