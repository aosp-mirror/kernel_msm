/* drivers/serial/msm_serial_hs.c
 *
 * MSM High speed uart driver
 *
 * Copyright (c) 2007-2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/module.h>

#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/dma.h>

#include "msm_serial_hs_hwreg.h"
#include "msm_serial_hs.h"

struct msm_uartdm_dmov_cb {
	struct msm_dmov_cmd xfer;
	struct msm_uartdm_port *msm_uport;
};

struct msm_uartdm_tx {
	unsigned int tx_ready_int_en;
	struct msm_uartdm_dmov_cb xfer;
	/* Simple command buffer.  Allocated and freed by driver. */
	dmov_box *command_ptr;
	u32 *command_ptr_ptr;
	dma_addr_t mapped_cmd_ptr;
	struct workqueue_struct *tx_wq;
	int tx_count;
};

struct msm_uartdm_rx {
	unsigned int rx_ready_int_en;
	struct msm_uartdm_dmov_cb xfer;
	/* Simple command buffer.  Allocated and freed by driver. */
	dmov_box *command_ptr;
	u32 *command_ptr_ptr;
	dma_addr_t mapped_cmd_ptr;
	struct workqueue_struct *rx_wq;
	struct completion complete;
	dma_addr_t rbuffer;
	unsigned char *buffer;
	struct dma_pool *rx_pool;
};

struct msm_uartdm_port {
	struct uart_port base;
	unsigned long imr_reg;	/* shadow value of UARTDM_IMR */
	struct clk *clk_id;
	atomic_t open_cnt;
	struct msm_uartdm_tx tx_desc;
	struct msm_uartdm_rx rx_desc;
	struct work_struct rx_data_worker;
	struct work_struct tx_data_worker;
	int dma_tx_channel;
	int dma_rx_channel;
	int dma_tx_crci;
	int dma_rx_crci;
};

/* DM burst size (in bytes) (Constant) */
#define MSM_UARTDM_BURST_SIZE 16
#define UARTDM_TX_BUF_SIZE UART_XMIT_SIZE
#define UARTDM_RX_BUF_SIZE 512

/* cable status polling time value 250 msec */
#define RX_GPIO_TIMER ((1000 * HZ)/1000)
/* RX Cable connect denounce */
#define DEBOUNCE_RX_GPIO 3

#define UARTDM_MAX_COUNT 2

static struct msm_uartdm_port q_uart_port[UARTDM_MAX_COUNT];
static struct platform_driver msm_uartdm_pd;
static struct uart_driver msm_uartdm_driver;
static struct uart_ops msm_uartdm_ops;
static char *uartdm_clks[] = { "uart1dm_clk", "uart2dm_clk" };

#if 1
#define TRACE() do {} while (0)

#define TRACE_DEBUG(fmt,args...) do {} while (0)

#define TRACE_ERR(fmt,args...)  do {} while (0)
#else
#define TRACE()  printk(KERN_ALERT "%s() :%d\n", __FUNCTION__ , __LINE__)

#define TRACE_DEBUG(fmt,args...)  \
	printk(KERN_ALERT "%s() :%d " fmt "\n", \
		__FUNCTION__ , __LINE__ , ##args)

#define TRACE_ERR(fmt,args...)  \
	printk(KERN_ERR "%s() :%d " fmt "\n", __FUNCTION__ , __LINE__ , ##args)
#endif

static inline unsigned int uport_read32(struct uart_port *uport,
					unsigned int offset)
{
	return (ioread32((void *)((unsigned int)uport->mapbase + offset)));
}

static inline void uport_write32(struct uart_port *uport, unsigned int offset,
				 unsigned int value)
{
	iowrite32(value, (void *)((unsigned int)uport->mapbase + offset));
}

/* Stub routine, current not needed (Used for power management) */
static int msm_uartdm_wake(struct uart_port *uport, unsigned int state)
{
	return 0;
}

/* Stubbed for future uses. (Called when system issues shutdown command) */
static void msm_uartdm_pd_shutdown(struct platform_device *pdev)
{
	TRACE();
}

/*  Called by msm_uart_remove to do port specific shutdown */
static void msm_uartdm_remove_port(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport;

	msm_uport = (struct msm_uartdm_port *)uport;

	TRACE();

	uart_remove_one_port(&msm_uartdm_driver, uport);
	clk_put(msm_uport->clk_id);

	/* Free the cable detect IRQ */
	free_irq(uport->irq, uport);

	/* Free the tx resources */
	kfree(msm_uport->tx_desc.command_ptr);
	kfree(msm_uport->tx_desc.command_ptr_ptr);

	/* Free the rx resources */
	kfree(msm_uport->rx_desc.command_ptr);
	kfree(msm_uport->rx_desc.command_ptr_ptr);

	dma_pool_free(msm_uport->rx_desc.rx_pool, msm_uport->rx_desc.buffer,
		      msm_uport->rx_desc.rbuffer);
	dma_pool_destroy(msm_uport->rx_desc.rx_pool);
}

/*
 * When platform_driver_unregister() is called, this remove function of the
 * driver will be called.
 */
static int msm_uartdm_remove(struct platform_device *pdev)
{
	int i;
	struct uart_port *uport = NULL;

	for (i = 0; i < UARTDM_MAX_COUNT; i++) {
		if (0 != q_uart_port[i].base.type) {
			uport = (struct uart_port *)&(q_uart_port[i].base);
			msm_uartdm_remove_port(uport);
		};
	}

	return 0;
}

static void msm_uartdm_release_port(struct uart_port *port)
{
	iounmap((void *)(port->mapbase));
}

/* Called during system suspend to do port specific suspend */
static void msm_uartdm_suspend_port(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	unsigned long data;

	/* ready to sleep */
	if (atomic_read(&(msm_uport->open_cnt))) {
		data = uport_read32(uport, UARTDM_ISR_ADDR);
		if (data & UARTDM_ISR_CURRENT_CTS_BMSK) {
			data = uport_read32(uport, UARTDM_MR1_ADDR);
			data &= ~(UARTDM_MR1_CTS_CTL_BMSK);
			uport_write32(uport, UARTDM_MR1_ADDR, data);
		}
	}

	/* ready to sleep */
	uart_suspend_port(&msm_uartdm_driver, uport);
}

/*
 * This suspend function is called when interrupt is enabled.
 * You can sleep or do something, but be short.
 */
static int msm_uartdm_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	struct uart_port *uport = NULL;
	TRACE();

	for (i = 0; i < UARTDM_MAX_COUNT; i++) {
		if (q_uart_port[i].base.type) {
			uport = (struct uart_port *)&(q_uart_port[i].base);
			msm_uartdm_suspend_port(uport);
		};
	}

	return 0;
}

/*
 * This suspend_late function is called when interrupt is disabled. You can't
 * sleep here. This is mainly used to finish the unfinished job from suspend
 * function.
 *
 *  Note:  The time between suspend and suspend_late is undeterministic.
 */
static int msm_uartdm_suspend_late(struct platform_device *pdev,
				   pm_message_t state)
{
	return 0;
}

/*
 * This resume_early function is called when interrupt is
 * disabled. You can't sleep here. This is mainly used to turn
 * on the things at a early time without being interrupted.
 *
 * Note: The time between resume_early and resume is undeterministic.
 *
 */
static int msm_uartdm_resume_early(struct platform_device *pdev)
{
	return 0;
}

static int msm_uartdm_resume_port(struct uart_port *uport)
{
	uart_resume_port(&msm_uartdm_driver, uport);
	return 0;
}

/* This resume function is called when interrupt is enabled. */
static int msm_uartdm_resume(struct platform_device *pdev)
{
	int i;
	struct uart_port *uport = NULL;
	TRACE();

	for (i = 0; i < UARTDM_MAX_COUNT; i++) {
		if (q_uart_port[i].base.type) {
			uport = (struct uart_port *)&(q_uart_port[i].base);
			msm_uartdm_resume_port(uport);
		};
	}
	return 0;
}

static int msm_uartdm_ioctl(struct uart_port *uport, unsigned int cmd,
			    unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static int msm_uartdm_enable_clk(struct uart_port *uport)
{
	int ret;

	/* Set up the MREG/NREG/DREG/MNDREG */
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	ret = clk_enable(msm_uport->clk_id);
	if (ret) {
		printk(KERN_ERR "Error could not turn on UART clk\n");
		return ret;
	}

	ret = clk_set_rate(msm_uport->clk_id, uport->uartclk);
	if (ret) {
		printk(KERN_WARNING "Error setting clock rate on UART\n");
		return ret;
	}

	return 0;
}

static inline void msm_uartdm_disable_clk(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	clk_disable(msm_uport->clk_id);
}

/* Enable and Disable clocks  (Used for power management) */
static void msm_uartdm_pm(struct uart_port *uport, unsigned int state,
			  unsigned int oldstate)
{
	if (state == 3) {
		msm_uartdm_disable_clk(uport);
	} else if (state == 0) {
		if (msm_uartdm_enable_clk(uport)) {
			/* Clock not enabled */
			BUG_ON(1);
		}
	}
}

/*
 * programs the UARTDM_CSR register with correct bit rates
 *
 * Interrupts should be disabled before we are called, as
 * we modify Set Baud rate
 * Set receive stale interrupt level, dependant on Bit Rate
 * Goal is to have around 8 ms before indicate stale.
 * roundup (((Bit Rate * .008) / 10) + 1
 */
static void msm_uartdm_set_bps(struct uart_port *uport, unsigned int bps)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	unsigned long rxstale;
	unsigned long data;

	switch (bps) {
	case 300:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x00);
		rxstale = 1;
		break;
	case 600:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x11);
		rxstale = 1;
		break;
	case 1200:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x22);
		rxstale = 1;
		break;
	case 2400:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x33);
		rxstale = 1;
		break;
	case 4800:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x44);
		rxstale = 1;
		break;
	case 9600:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x55);
		rxstale = 2;
		break;
	case 14400:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x66);
		rxstale = 3;
		break;
	case 19200:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x77);
		rxstale = 4;
		break;
	case 28800:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x88);
		rxstale = 6;
		break;
	case 38400:
		uport_write32(uport, UARTDM_CSR_ADDR, 0x99);
		rxstale = 8;
		break;
	case 57600:
		uport_write32(uport, UARTDM_CSR_ADDR, 0xaa);
		rxstale = 16;
		break;
	case 76800:
		uport_write32(uport, UARTDM_CSR_ADDR, 0xbb);
		rxstale = 16;
		break;
	case 115200:
		uport_write32(uport, UARTDM_CSR_ADDR, 0xcc);
		rxstale = 31;
		break;
	case 230400:
		uport_write32(uport, UARTDM_CSR_ADDR, 0xee);
		rxstale = 31;
		break;
	case 460800:
		uport_write32(uport, UARTDM_CSR_ADDR, 0xff);
		rxstale = 31;
		break;
	case 4000000:
	case 3686400:
	case 3200000:
	case 3500000:
	case 3000000:
	case 2500000:
	case 1500000:
	case 1152000:
	case 1000000:
	case 921600:
		uport_write32(uport, UARTDM_CSR_ADDR, 0xff);
		rxstale = 31;
		break;
	default:
		uport_write32(uport, UARTDM_CSR_ADDR, 0xff);
		/* default to 9600 */
		bps = 9600;
		rxstale = 2;
		break;
	}
	if (bps > 460800) {
		uport->uartclk = bps * 16;
	} else {
		uport->uartclk = 7372800;
	}
	if (clk_set_rate(msm_uport->clk_id, uport->uartclk)) {
		printk(KERN_WARNING "Error setting clock rate on UART\n");
		return;
	}

	data = rxstale & UARTDM_IPR_STALE_LSB_BMSK;
	data |= UARTDM_IPR_STALE_TIMEOUT_MSB_BMSK & (rxstale << 2);

	uport_write32(uport, UARTDM_IPR_ADDR, data);
}

/*
 * termios :  new ktermios
 * oldtermios:  old ktermios previous setting
 *
 * Configure the serial port
 */
static void msm_uartdm_set_termios(struct uart_port *uport,
				   struct ktermios *termios,
				   struct ktermios *oldtermios)
{
	unsigned int bps;
	unsigned long data;
	unsigned int c_cflag = termios->c_cflag;
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;

	TRACE();

	/* 300 is the minimum baud support by the driver  */
	bps = uart_get_baud_rate(uport, termios, oldtermios, 200, 4000000);

	/* Temporary remapping  200 BAUD to 3.2 mbps */
	if (bps == 200)
		bps = 3200000;

	msm_uartdm_set_bps(uport, bps);

	data = uport_read32(uport, UARTDM_MR2_ADDR);
	data &= ~UARTDM_MR2_PARITY_MODE_BMSK;
	/* set parity */
	if (PARENB == (c_cflag & PARENB)) {
		if (PARODD == (c_cflag & PARODD)) {
			data |= ODD_PARITY;
		} else if (CMSPAR == (c_cflag & CMSPAR)) {
			data |= SPACE_PARITY;
		} else {
			data |= EVEN_PARITY;
		}
	}

	/* Set bits per char */
	data &= ~UARTDM_MR2_BITS_PER_CHAR_BMSK;

	switch (c_cflag & CSIZE) {
	case CS5:
		data |= FIVE_BPC;
		break;
	case CS6:
		data |= SIX_BPC;
		break;
	case CS7:
		data |= SEVEN_BPC;
		break;
	default:
		data |= EIGHT_BPC;
		break;

	}
	/* stop bits */
	if (c_cflag & CSTOPB) {
		data |= STOP_BIT_TWO;
	} else {
		/* otherwise 1 stop bit */
		data |= STOP_BIT_ONE;
	}
	data |= UARTDM_MR2_ERROR_MODE_BMSK;
	/* write parity/bits per char/stop bit configuration */
	uport_write32(uport, UARTDM_MR2_ADDR, data);

	/* Configure HW flow control */
	data = uport_read32(uport, UARTDM_MR1_ADDR);

	data &= ~(UARTDM_MR1_CTS_CTL_BMSK | UARTDM_MR1_RX_RDY_CTL_BMSK);

	if (c_cflag & CRTSCTS) {
		data |= UARTDM_MR1_CTS_CTL_BMSK;
		data |= UARTDM_MR1_RX_RDY_CTL_BMSK;
	}

	uport_write32(uport, UARTDM_MR1_ADDR, data);

	uport->ignore_status_mask = termios->c_iflag & INPCK;
	uport->ignore_status_mask |= termios->c_iflag & IGNPAR;
	uport->read_status_mask = (termios->c_cflag & CREAD);

	uport_write32(uport, UARTDM_IMR_ADDR, 0);

	/* Set Transmit software time out */
	uart_update_timeout(uport, c_cflag, bps);

	flush_workqueue(msm_uport->rx_desc.rx_wq);
	uport_write32(uport, UARTDM_CR_ADDR, RESET_RX);
	uport_write32(uport, UARTDM_CR_ADDR, RESET_TX);

	init_completion(&(msm_uport->rx_desc.complete));
	msm_dmov_flush(msm_uport->dma_rx_channel);

	uport_write32(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
}

/*
 *  Standard API, Transmitter
 *  Any character in the transmit shift register is sent
 */
static unsigned int msm_uartdm_tx_empty(struct uart_port *uport)
{
	unsigned int data;

	data = uport_read32(uport, UARTDM_SR_ADDR);
	if (UARTDM_SR_TXEMT_BMSK == (data & UARTDM_SR_TXEMT_BMSK)) {
		return TIOCSER_TEMT;
	} else {
		return 0;
	}
}

/*
 *  Standard API, Stop transmitter.
 *  Any character in the transmit shift register is sent as
 *  well as the current data mover transfer .
 */
static void msm_uartdm_stop_tx(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;

	TRACE();
	msm_uport->tx_desc.tx_ready_int_en = 0;
}

/*
 *  Standard API, Stop receiver as soon as possible.
 *
 *  Function immediately terminates the operation of the
 *  channel receiver and any incoming characters are lost. None
 *  of the receiver status bits are affected by this command and
 *  characters that are already in the receive FIFO there.
 */
static void msm_uartdm_stop_rx(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	struct msm_uartdm_rx *rx_desc_ptr;

	TRACE();

	rx_desc_ptr = &(msm_uport->rx_desc);

	/* Disable the receiver */
	uport_write32(uport, UARTDM_CR_ADDR, UARTDM_CR_RX_DISABLE_BMSK);
	if (rx_desc_ptr->rx_ready_int_en == 1) {
		init_completion(&(rx_desc_ptr->complete));
		rx_desc_ptr->rx_ready_int_en = 0;
		msm_dmov_flush(msm_uport->dma_rx_channel);
	}
}

/*  Transmit the next chunk of data */
static void msm_uartdm_submit_tx(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	struct circ_buf *tx_buf = &uport->info->xmit;
	int tx_count;
	int left;
	struct msm_uartdm_tx *tx_desc_ptr;
	struct msm_dmov_cmd *tx_xfer_ptr = NULL;

	tx_desc_ptr = &(msm_uport->tx_desc);

	TRACE();
	if (uart_circ_empty(tx_buf) || uport->info->port.tty->stopped) {
		msm_uartdm_stop_tx(uport);
		return;
	}

	tx_count = uart_circ_chars_pending(tx_buf);

	if (UARTDM_TX_BUF_SIZE < tx_count)
		tx_count = UARTDM_TX_BUF_SIZE;

	left = UART_XMIT_SIZE - tx_buf->tail;

	if (tx_count > left)
		tx_count = left;


	tx_desc_ptr->command_ptr->num_rows = (((tx_count + 15) >> 4) << 16) |
					      ((tx_count + 15) >> 4);
	tx_desc_ptr->command_ptr->src_row_addr =
		(u32) dma_map_single(NULL, &(tx_buf->buf [tx_buf->tail]),
				     tx_count, DMA_TO_DEVICE);

	tx_desc_ptr->mapped_cmd_ptr = dma_map_single(NULL,
						     tx_desc_ptr->command_ptr,
						     (sizeof(dmov_box)),
						     DMA_TO_DEVICE);

	*(tx_desc_ptr->command_ptr_ptr) = CMD_PTR_LP |
	    DMOV_CMD_ADDR(tx_desc_ptr->mapped_cmd_ptr);

	tx_xfer_ptr = (struct msm_dmov_cmd *)&tx_desc_ptr->xfer;

	tx_xfer_ptr->cmdptr = DMOV_CMD_ADDR(dma_map_single(NULL,
							   tx_desc_ptr->
							   command_ptr_ptr,
							   sizeof(u32 *),
							   DMA_TO_DEVICE));

	/* Save tx_count to use in Callback */
	tx_desc_ptr->tx_count = tx_count;
	uport_write32(uport, UARTDM_NCF_TX_ADDR, tx_count);

	/* Disable the tx_ready interrupt */
	msm_uport->imr_reg &= ~UARTDM_ISR_TX_READY_BMSK;
	uport_write32(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
	msm_dmov_enqueue_cmd(msm_uport->dma_tx_channel, tx_xfer_ptr);
}

/* Start to receive the next chunk of data */
static void msm_uart_start_rx(struct uart_port *uport)
{

	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	struct msm_uartdm_rx *rx_desc_ptr;
	struct msm_dmov_cmd *rx_xfer_ptr;

	TRACE();
	rx_desc_ptr = &(msm_uport->rx_desc);
	rx_desc_ptr->command_ptr->num_rows = ((UARTDM_RX_BUF_SIZE >> 4) << 16) |
					      (UARTDM_RX_BUF_SIZE >> 4);


	rx_desc_ptr->command_ptr->dst_row_addr = rx_desc_ptr->rbuffer;

	rx_desc_ptr->mapped_cmd_ptr = dma_map_single(NULL,
						     rx_desc_ptr->command_ptr,
						     (sizeof(dmov_box)),
						     DMA_TO_DEVICE);

	*(rx_desc_ptr->command_ptr_ptr) = CMD_PTR_LP |
	    DMOV_CMD_ADDR(rx_desc_ptr->mapped_cmd_ptr);

	rx_xfer_ptr = (struct msm_dmov_cmd *)&rx_desc_ptr->xfer;
	rx_xfer_ptr->cmdptr = DMOV_CMD_ADDR(dma_map_single(NULL,
							   rx_desc_ptr->
							   command_ptr_ptr,
							   sizeof(u32 *),
							   DMA_TO_DEVICE));

	init_completion(&(rx_desc_ptr->complete));
	msm_dmov_enqueue_cmd(msm_uport->dma_rx_channel, rx_xfer_ptr);

	uport_write32(uport, UARTDM_CR_ADDR, RESET_STALE_INT);

	uport_write32(uport, UARTDM_DMRX_ADDR, UARTDM_RX_BUF_SIZE);
	uport_write32(uport, UARTDM_CR_ADDR, STALE_EVENT_ENABLE);
}

/*
 * This is a workqueue handler to complete  DMA tx
 * transactions and submit new transactions
 */
void msm_uart_tx_workqueue_handler(struct work_struct *work)
{
	struct msm_uartdm_port *msm_uport = NULL;
	struct uart_port *uport = NULL;
	struct msm_uartdm_tx *tx_desc_ptr = NULL;
	struct circ_buf *tx_buf = NULL;
	struct msm_dmov_cmd *tx_xfer_ptr = NULL;

	TRACE();
	msm_uport = container_of(work, struct msm_uartdm_port, tx_data_worker);
	if (msm_uport) {
		uport = (struct uart_port *)msm_uport;
		tx_buf = &uport->info->xmit;
		tx_desc_ptr = &(msm_uport->tx_desc);

		/* DMA is done un-map memmory addresses */
		tx_xfer_ptr = (struct msm_dmov_cmd *)&tx_desc_ptr->xfer;
		dma_unmap_single(NULL, (dma_addr_t) tx_xfer_ptr->cmdptr,
				 sizeof(u32 *), DMA_TO_DEVICE);

		dma_unmap_single(NULL, tx_desc_ptr->mapped_cmd_ptr,
				 (sizeof(dmov_box)), DMA_TO_DEVICE);

		dma_unmap_single(NULL,
				 (dma_addr_t) tx_desc_ptr->command_ptr->
				 src_row_addr, tx_desc_ptr->tx_count,
				 DMA_TO_DEVICE);

		tx_buf->tail = (tx_buf->tail + tx_desc_ptr->tx_count) &
		    (~UART_XMIT_SIZE);

		uport->icount.tx += tx_desc_ptr->tx_count;
		if (msm_uport->tx_desc.tx_ready_int_en)
			msm_uartdm_submit_tx(uport);

		if (uart_circ_chars_pending(tx_buf) < WAKEUP_CHARS)
			uart_write_wakeup(uport);
	}
}

/*
 *  This is a workqueue handler to complete DMA rx
 *  transactions and start another rx transaction
 */
void msm_uart_rx_workqueue_handler(struct work_struct *work)
{
	struct msm_uartdm_port *msm_uport = NULL;
	struct msm_uartdm_rx *rx_desc_ptr = NULL;
	struct uart_port *uport = NULL;
	struct tty_struct *tty = NULL;
	struct msm_dmov_cmd *rx_xfer_ptr = NULL;

	int retval;
	int rx_count;

	TRACE();

	msm_uport = container_of(work, struct msm_uartdm_port, rx_data_worker);
	if (msm_uport) {
		uport = (struct uart_port *)msm_uport;
		tty = uport->info->port.tty;
		rx_desc_ptr = &(msm_uport->rx_desc);
		rx_xfer_ptr = (struct msm_dmov_cmd *)&rx_desc_ptr->xfer;

		/* DMA is done un-map memmory addresses */
		dma_unmap_single(NULL, (dma_addr_t) rx_xfer_ptr->cmdptr,
				 sizeof(u32 *), DMA_TO_DEVICE);

		dma_unmap_single(NULL, rx_desc_ptr->mapped_cmd_ptr,
				 (sizeof(dmov_box)), DMA_TO_DEVICE);

		rx_count = uport_read32(uport, UARTDM_RX_TOTAL_SNAP_ADDR);

		/* Ignore Parity Errors */

		if (0 != (uport->read_status_mask & CREAD)) {
			retval = tty_insert_flip_string(tty,
							rx_desc_ptr->buffer,
							rx_count);
			BUG_ON(retval != rx_count);
		}

		tty_flip_buffer_push(tty);

		if (1 == rx_desc_ptr->rx_ready_int_en)
			msm_uart_start_rx(uport);
		complete(&(rx_desc_ptr->complete));
	}
}

/* Enable the transmitter Interrupt */
static void msm_uartdm_start_tx(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	TRACE();
	if (0 == msm_uport->tx_desc.tx_ready_int_en) {
		msm_uport->tx_desc.tx_ready_int_en = 1;
		msm_uartdm_submit_tx(uport);
	}
}

/*
 *  This routine is called when we are done with a DMA transfer
 *
 *  This routine is registered with Data mover when we set
 *  up a Data Mover transfer. It is called from Data mover ISR
 *  when the DMA transfer is done.
 */
static void msm_uartdm_dmov_tx_callback(struct msm_dmov_cmd *cmd_ptr,
					unsigned int result,
					struct msm_dmov_errdata *err)
{
	struct msm_uartdm_dmov_cb *xfer;
	struct msm_uartdm_port *msm_uport;
	struct uart_port *uport;
	struct msm_uartdm_tx *tx_desc_ptr;

	xfer = (struct msm_uartdm_dmov_cb *)cmd_ptr;
	msm_uport = xfer->msm_uport;
	uport = (struct uart_port *)msm_uport;

	TRACE();
	tx_desc_ptr = &(msm_uport->tx_desc);

	/* DMA did not finish properly error out */
	BUG_ON(result != 0x80000002);

	msm_uport->imr_reg |= UARTDM_ISR_TX_READY_BMSK;
	uport_write32(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
}

/*
 * This routine is called when we are done with a DMA transfer or the
 * a flush has been sent to the data mover driver.
 *
 * This routine is registered with Data mover when we set up a Data Mover
 *  transfer. It is called from Data mover ISR when the DMA transfer is done.
 */
static void msm_uartdm_dmov_rx_callback(struct msm_dmov_cmd *cmd_ptr,
					unsigned int result,
					struct msm_dmov_errdata *err)
{
	struct msm_uartdm_dmov_cb *xfer;
	struct msm_uartdm_port *msm_uport;
	struct uart_port *uport;
	struct msm_uartdm_rx *rx_desc_ptr;
	struct tty_struct *tty;
	unsigned long status;
	int retval;
	unsigned int error_f = 0;

	TRACE();

	xfer = (struct msm_uartdm_dmov_cb *)cmd_ptr;
	msm_uport = xfer->msm_uport;
	uport = (struct uart_port *)msm_uport;
	tty = uport->info->port.tty;

	rx_desc_ptr = &(msm_uport->rx_desc);

	uport_write32(uport, UARTDM_CR_ADDR, STALE_EVENT_DISABLE);

	status = uport_read32(uport, UARTDM_SR_ADDR);

	/* overflow is not connect to data in a FIFO */
	if (unlikely((status & UARTDM_SR_OVERRUN_BMSK) &&
		     (uport->read_status_mask & CREAD))) {
		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		uport->icount.buf_overrun++;
		error_f = 1;
	}

	if (!(uport->ignore_status_mask & INPCK))
		status = status & (~UARTDM_SR_PAR_FRAME_BMSK);

	if (unlikely(status & UARTDM_SR_PAR_FRAME_BMSK)) {
		/*Can not tell difference between parity & frame error */
		uport->icount.parity++;
		error_f = 1;
		if (uport->ignore_status_mask & IGNPAR)
			tty_insert_flip_char(tty, 0, TTY_PARITY);
	}

	if (error_f)
		uport_write32(uport, UARTDM_CR_ADDR, RESET_ERROR_STATUS);

	retval = queue_work(rx_desc_ptr->rx_wq, &(msm_uport->rx_data_worker));

	/* Failed to queue DMA transfer */
	BUG_ON(0 == retval);
}

/*
 *  Standard API, Current states of modem control inputs
 *
 * Since CTS can be handled entirely by HARDWARE we always
 * indicate clear to send and count on the TX FIFO to block when
 * it fills up.
 *
 * - TIOCM_DCD
 * - TIOCM_CTS
 * - TIOCM_DSR
 * - TIOCM_RI
 *  (Unsupported) DCD and DSR will return them high. RI will return low.
 */
static unsigned int msm_uartdm_get_mctrl(struct uart_port *uport)
{
	unsigned int ret;

	ret = TIOCM_DSR | TIOCM_CAR | TIOCM_CTS;
	return ret;
}

/*
 *  Standard API, Set or clear RFR_signal
 *
 * Set RFR high, (Indicate we are not ready for data), we disable auto
 * ready for receiving and then set RFR_N high. To set RFR to low we just turn
 * back auto ready for receiving and it should lower RFR signal
 * when hardware is ready
 *
 */
static void msm_uartdm_set_mctrl(struct uart_port *uport, unsigned int mctrl)
{
	unsigned int set_rts;
	unsigned int data = 0;

	TRACE();

	/* RTS is active low */

	if (TIOCM_RTS & mctrl) {
		set_rts = 0;
	} else {
		set_rts = 1;
	}

	data = uport_read32(uport, UARTDM_MR1_ADDR);
	if (set_rts) {
		/*disable auto ready-for-receiving */
		data &= ~UARTDM_MR1_RX_RDY_CTL_BMSK;
		uport_write32(uport, UARTDM_MR1_ADDR, data);
		/* set RFR_N to high */
		uport_write32(uport, UARTDM_CR_ADDR, RFR_HIGH);
	} else {
		/* Enable auto ready-for-receiving */
		data |= UARTDM_MR1_RX_RDY_CTL_BMSK;
		uport_write32(uport, UARTDM_MR1_ADDR, data);
	}
}

/* Standard API, Enable modem status (CTS) interrupt  */
static void msm_uartdm_enable_ms(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;

	/* Enable DELTA_CTS Interrupt */
	msm_uport->imr_reg |= UARTDM_ISR_DELTA_CTS_BMSK;
	uport_write32(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
}

/*
 *  Standard API, Break Signal
 *
 * Control the transmission of a break signal. ctl eq 0 => break
 * signal terminate ctl ne 0 => start break signal
 */
static void msm_uartdm_break_ctl(struct uart_port *uport, int ctl)
{
	if (ctl) {
		uport_write32(uport, UARTDM_CR_ADDR, START_BREAK);
	} else {
		uport_write32(uport, UARTDM_CR_ADDR, STOP_BREAK);
	}
}

/* Need to be non-zero value or port will not init.  */
static void msm_uartdm_config_port(struct uart_port *uport, int flags)
{
	/* Set port type to any non-zero value */
	if (flags & UART_CONFIG_TYPE)
		uport->type = 1;
}

/*  Handle CTS changes (Called from interrupt handler) */
static void msm_uartdm_handle_delta_cts(struct uart_port *uport)
{
	/* clear interrupt */
	uport_write32(uport, UARTDM_CR_ADDR, RESET_CTS);
	uport->icount.cts++;
	/* clear the IOCTL TIOCMIWAIT if called */
	wake_up_interruptible(&uport->info->delta_msr_wait);
}

static irqreturn_t msm_uartdm_isr(int irq, void *dev)
{
	struct uart_port *uport = (struct uart_port *)dev;
	unsigned long isr_status;
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	u32 retval;
	struct msm_uartdm_tx *tx_desc_ptr;
	tx_desc_ptr = &(msm_uport->tx_desc);

	isr_status = uport_read32(uport, UARTDM_MISR_ADDR);

	/* Mask UART interrupts */
	disable_irq(uport->irq);
	TRACE();

	/* Stale rx interrupt */
	if (isr_status & UARTDM_ISR_RXSTALE_BMSK) {
		uport_write32(uport, UARTDM_CR_ADDR, STALE_EVENT_DISABLE);
		uport_write32(uport, UARTDM_CR_ADDR, RESET_STALE_INT);

		msm_dmov_flush(msm_uport->dma_rx_channel);
	}
	/* tx ready interrupt */
	if (isr_status & UARTDM_ISR_TX_READY_BMSK) {
		/*Clear  TX Ready */
		uport_write32(uport, UARTDM_CR_ADDR, CLEAR_TX_READY);
		retval = queue_work(tx_desc_ptr->tx_wq,
				    &(msm_uport->tx_data_worker));
		BUG_ON(1 != retval);
	}

	/* Change in CTS interrupt */
	if (isr_status & UARTDM_ISR_DELTA_CTS_BMSK)
		msm_uartdm_handle_delta_cts(uport);

	enable_irq(uport->irq);
	return IRQ_HANDLED;
}

static const char *msm_uartdm_type(struct uart_port *port)
{
	return ("MSM UART");
}

/*
 *  Called by the upper layer when port is open.
 *
 *      - Initializes the port
 *      - Hook the ISR
 *	- Enable Receive Interrupt
 */
static int msm_uartdm_startup(struct uart_port *uport)
{
	int rfr_level;
	unsigned int data;
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	struct msm_uartdm_tx *tx_desc_ptr;
	struct msm_uartdm_rx *rx_desc_ptr;
	struct msm_dmov_cmd *tx_xfer_ptr;
	struct msm_dmov_cmd *rx_xfer_ptr;

	TRACE();
	atomic_inc(&(msm_uport->open_cnt));

	tx_desc_ptr = &(msm_uport->tx_desc);
	rx_desc_ptr = &(msm_uport->rx_desc);

	if (uport->fifosize > 16) {
		rfr_level = uport->fifosize - 16;
	} else {
		rfr_level = uport->fifosize;
	}

	/* Set auto RFR Level */
	data = uport_read32(uport, UARTDM_MR1_ADDR);
	data &= (~UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK);
	data &= (~UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK);
	data |= (UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK & (rfr_level << 2));
	data |= (UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK & rfr_level);
	uport_write32(uport, UARTDM_MR1_ADDR, data);

	/* Make sure RXSTALE count is non-zero */
	data = uport_read32(uport, UARTDM_IPR_ADDR);
	if (!data) {
		data |= 0x1f & UARTDM_IPR_STALE_LSB_BMSK;
		uport_write32(uport, UARTDM_IPR_ADDR, data);
	}

	/* Enable Data Mover Mode */
	data = UARTDM_TX_DM_EN_BMSK | UARTDM_RX_DM_EN_BMSK;
	uport_write32(uport, UARTDM_DMEN_ADDR, data);

	/* Reset TX */
	uport_write32(uport, UARTDM_CR_ADDR, RESET_TX);

	uport_write32(uport, UARTDM_CR_ADDR, RESET_RX);
	uport_write32(uport, UARTDM_CR_ADDR, RESET_ERROR_STATUS);
	uport_write32(uport, UARTDM_CR_ADDR, RESET_BREAK_INT);
	uport_write32(uport, UARTDM_CR_ADDR, RESET_STALE_INT);
	uport_write32(uport, UARTDM_CR_ADDR, RESET_CTS);
	uport_write32(uport, UARTDM_CR_ADDR, RFR_LOW);
	/* Turn on Uart Receiver */
	uport_write32(uport, UARTDM_CR_ADDR, UARTDM_CR_RX_EN_BMSK);

	/* Turn on Uart Transmitter */
	uport_write32(uport, UARTDM_CR_ADDR, UARTDM_CR_TX_EN_BMSK);

	/* Initialize the tx */
	tx_desc_ptr->tx_ready_int_en = 0;

	tx_desc_ptr->xfer.msm_uport = msm_uport;
	tx_xfer_ptr = (struct msm_dmov_cmd *)&tx_desc_ptr->xfer;
	tx_xfer_ptr->complete_func = msm_uartdm_dmov_tx_callback;

	tx_desc_ptr->command_ptr->cmd = CMD_LC |
	    CMD_DST_CRCI(msm_uport->dma_tx_crci) | CMD_MODE_BOX;

	tx_desc_ptr->command_ptr->src_dst_len = (MSM_UARTDM_BURST_SIZE << 16)
						| (MSM_UARTDM_BURST_SIZE);

	tx_desc_ptr->command_ptr->row_offset = (MSM_UARTDM_BURST_SIZE << 16);

	tx_desc_ptr->command_ptr->dst_row_addr = MSM_UART1DM_PHYS + UARTDM_TF_ADDR;


	/* Turn on Uart Receive */

	rx_desc_ptr->xfer.msm_uport = msm_uport;
	rx_xfer_ptr = (struct msm_dmov_cmd *)&rx_desc_ptr->xfer;
	rx_xfer_ptr->complete_func = msm_uartdm_dmov_rx_callback;

	rx_desc_ptr->command_ptr->cmd = CMD_LC |
	    CMD_SRC_CRCI(msm_uport->dma_rx_crci) | CMD_MODE_BOX;

	rx_desc_ptr->command_ptr->src_dst_len = (MSM_UARTDM_BURST_SIZE << 16)
						| (MSM_UARTDM_BURST_SIZE);
	rx_desc_ptr->command_ptr->src_row_addr = MSM_UART1DM_PHYS + UARTDM_RF_ADDR;

	rx_desc_ptr->command_ptr->row_offset =  MSM_UARTDM_BURST_SIZE;

	rx_desc_ptr->command_ptr->src_row_addr = (unsigned int)uport->membase +
	    UARTDM_RF_ADDR;


	rx_desc_ptr->rx_wq = create_singlethread_workqueue("rx_wq");
	INIT_WORK(&(msm_uport->rx_data_worker), msm_uart_rx_workqueue_handler);

	tx_desc_ptr->tx_wq = create_singlethread_workqueue("tx_wq");
	INIT_WORK(&(msm_uport->tx_data_worker), msm_uart_tx_workqueue_handler);

	rx_desc_ptr->rx_ready_int_en = 1;

	msm_uport->imr_reg |= UARTDM_ISR_RXSTALE_BMSK;
	/* Enable reading the current CTS, no harm even if CTS is ignored */
	msm_uport->imr_reg |= UARTDM_ISR_CURRENT_CTS_BMSK;

	enable_irq(uport->irq);
	uport_write32(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);

	msm_uart_start_rx(uport);

	return 0;
}

/* Initializes uart_port data structure and adds the port to the driver */
int uartdm_init_port(struct uart_driver *udriver, struct uart_port *uport,
		     unsigned long base_addr, unsigned long phy_base_addr,
		     unsigned long irq_num, unsigned long fifo_size, char *desc)
{
	int ret;
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	struct msm_uartdm_tx *tx_desc_ptr;
	struct msm_uartdm_rx *rx_desc_ptr;

	TRACE();

	atomic_set(&(msm_uport->open_cnt), 0);
	rx_desc_ptr = &(msm_uport->rx_desc);
	tx_desc_ptr = &(msm_uport->tx_desc);
	uport->iotype = UPIO_MEM;
	uport->irq = irq_num;
	uport->fifosize = fifo_size;

	uport->membase = (void *)phy_base_addr;
	/* base_addr is a virtual address */
	uport->mapbase = base_addr;

	uport->ops = &msm_uartdm_ops;
	uport->flags = UPF_BOOT_AUTOCONF;
	((struct msm_uartdm_port *)uport)->imr_reg = 0x0;

	/* Perform the hardware initialization */

	msm_uport->clk_id = clk_get(NULL, desc);

	ret = msm_uartdm_enable_clk(uport);
	if (ret) {
		printk(KERN_ERR "Error could not enable clk");
		return ret;
	}

	ret = request_irq(uport->irq, msm_uartdm_isr,
			  (IRQF_TRIGGER_HIGH), "msm_uartdm", uport);

	/* irq will be enabled when port is opened */
	disable_irq(uport->irq);

	/* Failed to Hook ISR error Out */
	if (ret) {
		printk(KERN_ERR "Failed to hook up main ISR \n");
		return ret;
	}

	/* Allocate the command pointer. Needs to be 64 bit aligned */
	tx_desc_ptr->command_ptr = kmalloc((sizeof(dmov_box)),
					   GFP_KERNEL | __GFP_DMA);

	tx_desc_ptr->command_ptr_ptr = kmalloc((sizeof(void *)),
					       GFP_KERNEL | __GFP_DMA);

	if ((tx_desc_ptr->command_ptr == NULL) ||
	    (tx_desc_ptr->command_ptr_ptr == NULL))
		return -ENOMEM;

	rx_desc_ptr->rx_pool = dma_pool_create("rx_buffer_pool", NULL,
					       UARTDM_RX_BUF_SIZE, 16, 0);

	rx_desc_ptr->buffer = dma_pool_alloc(rx_desc_ptr->rx_pool, GFP_KERNEL,
					     &(rx_desc_ptr->rbuffer));

	/* Allocate the command pointer. Needs to be 64 bit aligned */
	rx_desc_ptr->command_ptr = kmalloc((sizeof(dmov_box)),
					   GFP_KERNEL | __GFP_DMA);

	rx_desc_ptr->command_ptr_ptr = kmalloc((sizeof(void *)),
					       GFP_KERNEL | __GFP_DMA);

	if ((rx_desc_ptr->command_ptr == NULL) ||
	    (rx_desc_ptr->command_ptr_ptr == NULL) ||
	    (rx_desc_ptr->rx_pool == NULL) || (rx_desc_ptr->buffer == NULL))
		return -ENOMEM;

	/* configure the CR Protection to Enable */
	uport_write32(uport, UARTDM_CR_ADDR, CR_PROTECTION_EN);

	msm_uartdm_disable_clk(uport);

	ret = uart_add_one_port(udriver, uport);
	if (ret) {
		printk(KERN_ERR "uartdm_init_port Failed to add port \n");
		return ret;
	}
	return 0;
}

static int msm_uartdm_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct uart_port *uport;
	struct msm_uartdm_port *msm_uport;
	unsigned int base_addr = 0;
	unsigned long phy_base_addr = 0;

	/* Interrupt from UARTDM controller */
	unsigned int uartdm_irq = 0;

	TRACE_DEBUG("plaform device ID = %d", pdev->id);

	if (pdev->id != 1) {
		TRACE_ERR(" Invalid plaform device ID = %d\n", pdev->id);

		return -EINVAL;
	}

	if ((pdev->resource == NULL) || (pdev->num_resources != 4)) {
		TRACE_ERR(" Invalid num of resources  pdev->resource = %u"
			  "pdev->num_resources = %d \n",
			  (unsigned int)(pdev->resource), pdev->num_resources);

		return -ENXIO;
	}

	/* Init all UARTS as non-configured */
	for (i = 0; i < UARTDM_MAX_COUNT; i++) {
		q_uart_port[i].base.type = 0;

		q_uart_port[i].dma_tx_channel = 0;
		q_uart_port[i].dma_rx_channel = 0;

		q_uart_port[i].dma_tx_crci = 0;
		q_uart_port[i].dma_rx_crci = 0;

		/* Default Clock */
		q_uart_port[i].base.uartclk = 7372800;
	}

	uport = (struct uart_port *)&(q_uart_port[pdev->id - 1]);
	msm_uport = &(q_uart_port[pdev->id - 1]);

	for (i = 0; i < pdev->num_resources; i++) {

		if (pdev->resource[i].flags & IORESOURCE_MEM) {

			phy_base_addr = pdev->resource[i].start;
			base_addr =
			    (unsigned int)ioremap(phy_base_addr, PAGE_SIZE);
			if (!base_addr) {
				TRACE_ERR("No Memory\n");
				return -ENOMEM;
			}
			TRACE_DEBUG("phy_base_addr = %08x,"
				    " virt base_addr = %08x ",
				    phy_base_addr, base_addr);

		}
		if (pdev->resource[i].flags & IORESOURCE_IRQ) {
			uartdm_irq = pdev->resource[i].start;

			TRACE_DEBUG("uartdm_irq = %d ", uartdm_irq);
		}
		if (pdev->resource[i].flags & IORESOURCE_DMA) {
			if (!strcmp(pdev->resource[i].name, "uartdm_chanels")) {
				msm_uport->dma_tx_channel =
				    pdev->resource[i].start;

				msm_uport->dma_rx_channel =
				    pdev->resource[i].end;

				TRACE_DEBUG("dma_tx_channel = %d,"
					    " dma_rx_channel = %d ",
					    msm_uport->dma_tx_channel,
					    msm_uport->dma_rx_channel);
			}
			if (!strcmp(pdev->resource[i].name, "uartdm_crci")) {
				msm_uport->dma_tx_crci =
				    pdev->resource[i].start;
				msm_uport->dma_rx_crci = pdev->resource[i].end;

				TRACE_DEBUG("dma_tx_crci = %d, "
					    "dma_rx_crci= %d ",
					    msm_uport->dma_tx_crci,
					    msm_uport->dma_rx_crci);
			}
		}
	}

	if (!uartdm_irq || !msm_uport->dma_tx_channel ||
	    !msm_uport->dma_rx_channel || !msm_uport->dma_tx_crci ||
	    !msm_uport->dma_rx_crci) {

		TRACE_ERR("Invalid irq, or DM channels or DM CRCIs");
		return -ENXIO;
	}

	/* FIFO size for UARTDM is 64 */

	uport->dev = &(pdev->dev);

	ret = uartdm_init_port(&msm_uartdm_driver, uport, base_addr,
			       phy_base_addr, uartdm_irq, 64,
			       uartdm_clks[pdev->id - 1]);

	if (ret != 0) {
		printk(KERN_ERR "msm_uartdm_probe failed on port %d \n",
		       pdev->id);
		uart_unregister_driver(&msm_uartdm_driver);
		return ret;
	}

	return ret;
}

static int __init msm_uartdm_init(void)
{
	int ret = 0;
	TRACE();
	ret = uart_register_driver(&msm_uartdm_driver);
	if (ret) {
		printk(KERN_ERR "msm_uart_init failed to load module \n");
		return ret;
	}
	ret = platform_driver_register(&msm_uartdm_pd);
	if (ret) {
		printk(KERN_ERR "msm_uart_init failed to load module \n");
		uart_unregister_driver(&msm_uartdm_driver);
		return ret;
	}
	printk(KERN_INFO "msm_uart_hs module loaded!\n");
	return ret;
}

/*
 *  Called by the upper layer when port is closed.
 *     - Disables the port
 *     - Unhook the ISR
 */
static void msm_uartdm_shutdown(struct uart_port *uport)
{
	struct msm_uartdm_port *msm_uport = (struct msm_uartdm_port *)uport;
	struct msm_uartdm_rx *rx_desc_ptr;
	struct msm_uartdm_tx *tx_desc_ptr;

	rx_desc_ptr = &(msm_uport->rx_desc);

	tx_desc_ptr = &(msm_uport->tx_desc);
	TRACE();

	atomic_dec(&(msm_uport->open_cnt));
	/* Disable the transmitter */
	uport_write32(uport, UARTDM_CR_ADDR, UARTDM_CR_TX_DISABLE_BMSK);
	/* Disable the receiver */
	uport_write32(uport, UARTDM_CR_ADDR, UARTDM_CR_RX_DISABLE_BMSK);

	/* Free the interrupt */
	disable_irq(uport->irq);
	msm_uport->imr_reg = 0;
	uport_write32(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
	wait_for_completion(&rx_desc_ptr->complete);
	flush_workqueue(msm_uport->rx_desc.rx_wq);
	destroy_workqueue(rx_desc_ptr->rx_wq);

	flush_workqueue(msm_uport->tx_desc.tx_wq);
	destroy_workqueue(tx_desc_ptr->tx_wq);
}

static void __exit msm_uartdm_exit(void)
{
	printk(KERN_INFO "msm_uart_hs module removed!\n");
	platform_driver_unregister(&msm_uartdm_pd);

	uart_unregister_driver(&msm_uartdm_driver);
}

/* for ttyHS0  pass 0 as line parameter */
unsigned int msm_uart_hs_tx_empty(int line)
{
	struct uart_port *uport;
	unsigned int data;
	uport = (struct uart_port *)&(q_uart_port[line]);

	data = uport_read32(uport, UARTDM_SR_ADDR);
	if (UARTDM_SR_TXEMT_BMSK == (data & UARTDM_SR_TXEMT_BMSK)) {
		return 1;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(msm_uart_hs_tx_empty);

/*
 * To be used by a  kernel module  so that it can turn UART on and off to save
 * power.It is up to  the upper layer to manage if it is  correct to do.
 */
void msm_uart_hs_safe_pm(int line, enum msm_uart_hs_pm_e mode)
{
	struct uart_port *uport;
	uport = (struct uart_port *)&(q_uart_port[line]);

	switch (mode) {
	case MSM_UART_HS_POWER_DOWN:
		disable_irq(uport->irq);
		msm_uartdm_disable_clk(uport);
		break;
	case MSM_UART_HS_POWER_UP:
		if (msm_uartdm_enable_clk(uport))
			BUG_ON(1);
		enable_irq(uport->irq);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(msm_uart_hs_safe_pm);

static struct platform_driver msm_uartdm_pd = {
	.probe = msm_uartdm_probe,
	.remove = msm_uartdm_remove,
	.suspend = msm_uartdm_suspend,
	.suspend_late = msm_uartdm_suspend_late,
	.resume_early = msm_uartdm_resume_early,
	.resume = msm_uartdm_resume,
	.shutdown = msm_uartdm_pd_shutdown,
	.driver = {
		   /* Driver name must match the device name  used
		      in platform_devices for uarts_hs devices .
		    */
		   .name = "msm_serial_hs",
		   },
};

static struct uart_driver msm_uartdm_driver = {
	.owner = THIS_MODULE,
	.driver_name = "msm_uartdm",
	.dev_name = "ttyHS",
	.major = 0,
	.minor = 0,
	.nr = UARTDM_MAX_COUNT,
	.cons = 0,
};

static struct uart_ops msm_uartdm_ops = {
	.tx_empty = msm_uartdm_tx_empty,
	.set_mctrl = msm_uartdm_set_mctrl,
	.get_mctrl = msm_uartdm_get_mctrl,
	.stop_tx = msm_uartdm_stop_tx,
	.start_tx = msm_uartdm_start_tx,
	.stop_rx = msm_uartdm_stop_rx,
	.enable_ms = msm_uartdm_enable_ms,
	.break_ctl = msm_uartdm_break_ctl,
	.startup = msm_uartdm_startup,
	.shutdown = msm_uartdm_shutdown,
	.set_termios = msm_uartdm_set_termios,
	.pm = msm_uartdm_pm,
	.set_wake = msm_uartdm_wake,
	.type = msm_uartdm_type,
	.config_port = msm_uartdm_config_port,
	.ioctl = msm_uartdm_ioctl,
	.release_port = msm_uartdm_release_port,
};

module_init(msm_uartdm_init);
module_exit(msm_uartdm_exit);
MODULE_DESCRIPTION("High Speed UART Driver for the MSM chipset");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
