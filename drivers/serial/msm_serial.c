/*
 * drivers/serial/msm_serial.c - driver for msm7k serial device and console
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Robert Love <rlove@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#if defined(CONFIG_SERIAL_MSM_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
# define SUPPORT_SYSRQ
#endif

#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include "msm_serial.h"

#ifdef CONFIG_SERIAL_MSM_CLOCK_CONTROL
enum msm_clk_states_e {
	MSM_CLK_PORT_OFF,     /* uart port not in use */
	MSM_CLK_OFF,          /* clock enabled */
	MSM_CLK_REQUEST_OFF,  /* disable after TX flushed */
	MSM_CLK_ON,           /* clock disabled */
};
#endif

struct msm_port {
	struct uart_port	uart;
	char			name[16];
	struct clk		*clk;
	unsigned int		imr;
#ifdef CONFIG_SERIAL_MSM_CLOCK_CONTROL
	enum msm_clk_states_e	clk_state;
	struct hrtimer		clk_off_timer;
	ktime_t			clk_off_delay;
#endif
};

static void msm_stop_tx(struct uart_port *port)
{
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);

	msm_port->imr &= ~UART_IMR_TXLEV;
	msm_write(port, msm_port->imr, UART_IMR);

	clk_disable(msm_port->clk);
}

static void msm_start_tx(struct uart_port *port)
{
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);

	msm_port->imr |= UART_IMR_TXLEV;
	msm_write(port, msm_port->imr, UART_IMR);

	clk_disable(msm_port->clk);
}

static void msm_stop_rx(struct uart_port *port)
{
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);

	msm_port->imr &= ~(UART_IMR_RXLEV | UART_IMR_RXSTALE);
	msm_write(port, msm_port->imr, UART_IMR);

	clk_disable(msm_port->clk);
}

static void msm_enable_ms(struct uart_port *port)
{
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);

	msm_port->imr |= UART_IMR_DELTA_CTS;
	msm_write(port, msm_port->imr, UART_IMR);

	clk_disable(msm_port->clk);
}

#ifdef CONFIG_SERIAL_MSM_CLOCK_CONTROL
/* turn clock off if TX buffer is empty, otherwise reschedule */
static enum hrtimer_restart msm_serial_clock_off(struct hrtimer *timer) {
	struct msm_port *msm_port = container_of(timer, struct msm_port,
						 clk_off_timer);
	struct uart_port *port = &msm_port->uart;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long flags;
	int ret = HRTIMER_NORESTART;

	spin_lock_irqsave(&port->lock, flags);

	if (msm_port->clk_state == MSM_CLK_REQUEST_OFF) {
		if (uart_circ_empty(xmit)) {
			struct msm_port *msm_port = UART_TO_MSM(port);
			clk_disable(msm_port->clk);
			msm_port->clk_state = MSM_CLK_OFF;
		} else {
			hrtimer_forward_now(timer, msm_port->clk_off_delay);
			ret = HRTIMER_RESTART;
		}
	}

	spin_unlock_irqrestore(&port->lock, flags);

	return HRTIMER_NORESTART;
}

/* request to turn off uart clock once pending TX is flushed */
void msm_serial_clock_request_off(struct uart_port *port) {
	unsigned long flags;
	struct msm_port *msm_port = UART_TO_MSM(port);

	spin_lock_irqsave(&port->lock, flags);
	if (msm_port->clk_state == MSM_CLK_ON) {
		msm_port->clk_state = MSM_CLK_REQUEST_OFF;
		/* turn off TX later. unfortunately not all msm uart's have a
		 * TXDONE available, and TXLEV does not wait until completely
		 * flushed, so a timer is our only option
		 */
		hrtimer_start(&msm_port->clk_off_timer,
			      msm_port->clk_off_delay, HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&port->lock, flags);
}

/* request to immediately turn on uart clock.
 * ignored if there is a pending off request, unless force = 1.
 */
void msm_serial_clock_on(struct uart_port *port, int force) {
	unsigned long flags;
	struct msm_port *msm_port = UART_TO_MSM(port);

	spin_lock_irqsave(&port->lock, flags);

	switch (msm_port->clk_state) {
	case MSM_CLK_OFF:
		clk_enable(msm_port->clk);
		force = 1;
	case MSM_CLK_REQUEST_OFF:
		if (force) {
			hrtimer_try_to_cancel(&msm_port->clk_off_timer);
			msm_port->clk_state = MSM_CLK_ON;
		}
		break;
	case MSM_CLK_ON: break;
	case MSM_CLK_PORT_OFF: break;
	}

	spin_unlock_irqrestore(&port->lock, flags);
}
#endif

#ifdef CONFIG_SERIAL_MSM_RX_WAKEUP
#define WAKE_UP_IND	0x32
static irqreturn_t msm_rx_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct msm_port *msm_port = UART_TO_MSM(port);
	int inject_wakeup = 0;

	spin_lock(&port->lock);

	if (msm_port->clk_state == MSM_CLK_OFF)
		inject_wakeup = 1;

	msm_serial_clock_on(port, 0);

	/* we missed an rx while asleep - it must be a wakeup indicator
	 */
	if (inject_wakeup) {
		struct tty_struct *tty = port->state->port.tty;
		tty_insert_flip_char(tty, WAKE_UP_IND, TTY_NORMAL);
		tty_flip_buffer_push(tty);
	}

	spin_unlock(&port->lock);
	return IRQ_HANDLED;
}
#endif

static void handle_rx(struct uart_port *port)
{
	struct tty_struct *tty = port->state->port.tty;
	unsigned int sr;

	/*
	 * Handle overrun. My understanding of the hardware is that overrun
	 * is not tied to the RX buffer, so we handle the case out of band.
	 */
	if ((msm_read(port, UART_SR) & UART_SR_OVERRUN)) {
		port->icount.overrun++;
		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		msm_write(port, UART_CR_CMD_RESET_ERR, UART_CR);
	}

	/* and now the main RX loop */
	while ((sr = msm_read(port, UART_SR)) & UART_SR_RX_READY) {
		unsigned int c;
		char flag = TTY_NORMAL;

		c = msm_read(port, UART_RF);

		if (sr & UART_SR_RX_BREAK) {
			port->icount.brk++;
			if (uart_handle_break(port))
				continue;
		} else if (sr & UART_SR_PAR_FRAME_ERR) {
			port->icount.frame++;
		} else {
			port->icount.rx++;
		}

		/* Mask conditions we're ignorning. */
		sr &= port->read_status_mask;

		if (sr & UART_SR_RX_BREAK) {
			flag = TTY_BREAK;
		} else if (sr & UART_SR_PAR_FRAME_ERR) {
			flag = TTY_FRAME;
		}

		if (!uart_handle_sysrq_char(port, c))
			tty_insert_flip_char(tty, c, flag);
	}

	tty_flip_buffer_push(tty);
}

static void handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct msm_port *msm_port = UART_TO_MSM(port);
	int sent_tx;

	if (port->x_char) {
		msm_write(port, port->x_char, UART_TF);
		port->icount.tx++;
		port->x_char = 0;
	}

	while (msm_read(port, UART_SR) & UART_SR_TX_READY) {
		if (uart_circ_empty(xmit)) {
			/* disable tx interrupts */
			msm_port->imr &= ~UART_IMR_TXLEV;
			msm_write(port, msm_port->imr, UART_IMR);
			break;
		}

		msm_write(port, xmit->buf[xmit->tail], UART_TF);

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		sent_tx = 1;
	}

#ifdef CONFIG_SERIAL_MSM_CLOCK_CONTROL
	if (sent_tx && msm_port->clk_state == MSM_CLK_REQUEST_OFF)
		/* new TX - restart the timer */
		if (hrtimer_try_to_cancel(&msm_port->clk_off_timer) == 1)
			hrtimer_start(&msm_port->clk_off_timer,
				msm_port->clk_off_delay, HRTIMER_MODE_REL);
#endif

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void handle_delta_cts(struct uart_port *port)
{
	msm_write(port, UART_CR_CMD_RESET_CTS, UART_CR);
	port->icount.cts++;
	wake_up_interruptible(&port->state->port.delta_msr_wait);
}

static irqreturn_t msm_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct msm_port *msm_port = UART_TO_MSM(port);
	unsigned int misr;

	spin_lock(&port->lock);
	clk_enable(msm_port->clk);
	misr = msm_read(port, UART_MISR);
	msm_write(port, 0, UART_IMR); /* disable interrupt */

	if (misr & (UART_IMR_RXLEV | UART_IMR_RXSTALE))
		handle_rx(port);
	if (misr & UART_IMR_TXLEV)
		handle_tx(port);
	if (misr & UART_IMR_DELTA_CTS)
		handle_delta_cts(port);

	msm_write(port, msm_port->imr, UART_IMR); /* restore interrupt */
	clk_disable(msm_port->clk);
	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

static unsigned int msm_tx_empty(struct uart_port *port)
{
	unsigned int ret;
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);
	ret = (msm_read(port, UART_SR) & UART_SR_TX_EMPTY) ? TIOCSER_TEMT : 0;
	clk_disable(msm_port->clk);

	return ret;
}

static unsigned int msm_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR | TIOCM_RTS;
}

static void msm_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned int mr;
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);

	mr = msm_read(port, UART_MR1);

	if (!(mctrl & TIOCM_RTS)) {
		mr &= ~UART_MR1_RX_RDY_CTL;
		msm_write(port, mr, UART_MR1);
		msm_write(port, UART_CR_CMD_RESET_RFR, UART_CR);
	} else {
		mr |= UART_MR1_RX_RDY_CTL;
		msm_write(port, mr, UART_MR1);
	}

	clk_disable(msm_port->clk);
}

static void msm_break_ctl(struct uart_port *port, int break_ctl)
{
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);

	if (break_ctl)
		msm_write(port, UART_CR_CMD_START_BREAK, UART_CR);
	else
		msm_write(port, UART_CR_CMD_STOP_BREAK, UART_CR);

	clk_disable(msm_port->clk);
}

static int msm_set_baud_rate(struct uart_port *port, unsigned int baud)
{
	unsigned int baud_code, rxstale, watermark;

	switch (baud) {
	case 300:
		baud_code = UART_CSR_300;
		rxstale = 1;
		break;
	case 600:
		baud_code = UART_CSR_600;
		rxstale = 1;
		break;
	case 1200:
		baud_code = UART_CSR_1200;
		rxstale = 1;
		break;
	case 2400:
		baud_code = UART_CSR_2400;
		rxstale = 1;
		break;
	case 4800:
		baud_code = UART_CSR_4800;
		rxstale = 1;
		break;
	case 9600:
		baud_code = UART_CSR_9600;
		rxstale = 2;
		break;
	case 14400:
		baud_code = UART_CSR_14400;
		rxstale = 3;
		break;
	case 19200:
		baud_code = UART_CSR_19200;
		rxstale = 4;
		break;
	case 28800:
		baud_code = UART_CSR_28800;
		rxstale = 6;
		break;
	case 38400:
		baud_code = UART_CSR_38400;
		rxstale = 8;
		break;
	case 57600:
		baud_code = UART_CSR_57600;
		rxstale = 16;
		break;
	case 115200:
	default:
		baud_code = UART_CSR_115200;
		baud = 115200;
		rxstale = 31;
		break;
	}

	msm_write(port, baud_code, UART_CSR);

	/* RX stale watermark */
	watermark = UART_IPR_STALE_LSB & rxstale;
	watermark |= UART_IPR_RXSTALE_LAST;
	watermark |= UART_IPR_STALE_TIMEOUT_MSB & (rxstale << 2);
	msm_write(port, watermark, UART_IPR);

	/* set RX watermark */
	watermark = (port->fifosize * 3) / 4;
	msm_write(port, watermark, UART_RFWR);

	/* set TX watermark */
	msm_write(port, 10, UART_TFWR);

	return baud;
}

static void msm_reset(struct uart_port *port)
{
	/* reset everything */
	msm_write(port, UART_CR_CMD_RESET_RX, UART_CR);
	msm_write(port, UART_CR_CMD_RESET_TX, UART_CR);
	msm_write(port, UART_CR_CMD_RESET_ERR, UART_CR);
	msm_write(port, UART_CR_CMD_RESET_BREAK_INT, UART_CR);
	msm_write(port, UART_CR_CMD_RESET_CTS, UART_CR);
	msm_write(port, UART_CR_CMD_SET_RFR, UART_CR);
}

static void msm_init_clock(struct uart_port *port)
{
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);
#ifdef CONFIG_SERIAL_MSM_CLOCK_CONTROL
	msm_port->clk_state = MSM_CLK_ON;
#endif

	msm_serial_set_mnd_regs(port);
}

static int msm_startup(struct uart_port *port)
{
	struct msm_port *msm_port = UART_TO_MSM(port);
	unsigned int data, rfr_level;
	int ret;

	snprintf(msm_port->name, sizeof(msm_port->name),
		 "msm_serial%d", port->line);

	ret = request_irq(port->irq, msm_irq, IRQF_TRIGGER_HIGH,
			  msm_port->name, port);
	if (unlikely(ret))
		return ret;

	msm_init_clock(port);

	if (likely(port->fifosize > 12))
		rfr_level = port->fifosize - 12;
	else
		rfr_level = port->fifosize;

	/* set automatic RFR level */
	data = msm_read(port, UART_MR1);
	data &= ~UART_MR1_AUTO_RFR_LEVEL1;
	data &= ~UART_MR1_AUTO_RFR_LEVEL0;
	data |= UART_MR1_AUTO_RFR_LEVEL1 & (rfr_level << 2);
	data |= UART_MR1_AUTO_RFR_LEVEL0 & rfr_level;
	msm_write(port, data, UART_MR1);

	/* make sure that RXSTALE count is non-zero */
	data = msm_read(port, UART_IPR);
	if (unlikely(!data)) {
		data |= UART_IPR_RXSTALE_LAST;
		data |= UART_IPR_STALE_LSB;
		msm_write(port, data, UART_IPR);
	}

	msm_reset(port);

	msm_write(port, 0x05, UART_CR);	/* enable TX & RX */

	/* turn on RX and CTS interrupts */
	msm_port->imr = UART_IMR_RXLEV | UART_IMR_RXSTALE |
			UART_IMR_CURRENT_CTS;
	msm_write(port, msm_port->imr, UART_IMR);

#ifdef CONFIG_SERIAL_MSM_RX_WAKEUP
	/* Apply the RX GPIO wake irq workaround to the bluetooth uart */
	if (port->line == 0) {  /* BT is serial device 0 */
		ret = request_irq(MSM_GPIO_TO_INT(45), msm_rx_irq,
				  IRQF_TRIGGER_FALLING, "msm_serial0_rx",
				  port);
		if (unlikely(ret))
			return ret;
	}
#endif

	return 0;
}

static void msm_shutdown(struct uart_port *port)
{
	struct msm_port *msm_port = UART_TO_MSM(port);

	clk_enable(msm_port->clk);

	msm_port->imr = 0;
	msm_write(port, 0, UART_IMR); /* disable interrupts */

	clk_disable(msm_port->clk);

	free_irq(port->irq, port);

#ifdef CONFIG_SERIAL_MSM_RX_WAKEUP
	if (port->line == 0)
		free_irq(MSM_GPIO_TO_INT(45), port);
#endif

#ifdef CONFIG_SERIAL_MSM_CLOCK_CONTROL
	if (msm_port->clk_state != MSM_CLK_OFF)
		clk_disable(msm_port->clk);
	msm_port->clk_state = MSM_CLK_PORT_OFF;
#else
	clk_disable(msm_port->clk);
#endif
}

static void msm_set_termios(struct uart_port *port, struct ktermios *termios,
			    struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, mr;
	struct msm_port *msm_port = UART_TO_MSM(port);

	spin_lock_irqsave(&port->lock, flags);
	clk_enable(msm_port->clk);

	/* calculate and set baud rate */
	baud = uart_get_baud_rate(port, termios, old, 300, 115200);
	baud = msm_set_baud_rate(port, baud);
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
	
	/* calculate parity */
	mr = msm_read(port, UART_MR2);
	mr &= ~UART_MR2_PARITY_MODE;
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			mr |= UART_MR2_PARITY_MODE_ODD;
		else if (termios->c_cflag & CMSPAR)
			mr |= UART_MR2_PARITY_MODE_SPACE;
		else
			mr |= UART_MR2_PARITY_MODE_EVEN;
	}

	/* calculate bits per char */
	mr &= ~UART_MR2_BITS_PER_CHAR;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		mr |= UART_MR2_BITS_PER_CHAR_5;
		break;
	case CS6:
		mr |= UART_MR2_BITS_PER_CHAR_6;
		break;
	case CS7:
		mr |= UART_MR2_BITS_PER_CHAR_7;
		break;
	case CS8:
	default:
		mr |= UART_MR2_BITS_PER_CHAR_8;
		break;
	}

	/* calculate stop bits */
	mr &= ~(UART_MR2_STOP_BIT_LEN_ONE | UART_MR2_STOP_BIT_LEN_TWO);
	if (termios->c_cflag & CSTOPB)
		mr |= UART_MR2_STOP_BIT_LEN_TWO;
	else
		mr |= UART_MR2_STOP_BIT_LEN_ONE;

	/* set parity, bits per char, and stop bit */
	msm_write(port, mr, UART_MR2);

	/* calculate and set hardware flow control */
	mr = msm_read(port, UART_MR1);
	mr &= ~(UART_MR1_CTS_CTL | UART_MR1_RX_RDY_CTL);
	if (termios->c_cflag & CRTSCTS) {
		mr |= UART_MR1_CTS_CTL;
		mr |= UART_MR1_RX_RDY_CTL;
	}
	msm_write(port, mr, UART_MR1);

	/* Configure status bits to ignore based on termio flags. */
	port->read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_SR_PAR_FRAME_ERR;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_SR_RX_BREAK;

	uart_update_timeout(port, termios->c_cflag, baud);

	clk_disable(msm_port->clk);
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *msm_type(struct uart_port *port)
{
	return "MSM";
}

static void msm_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *resource;
	resource_size_t size;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return;
	size = resource->end - resource->start + 1;

	release_mem_region(port->mapbase, size);
	iounmap(port->membase);
	port->membase = NULL;
}

static int msm_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *resource;
	resource_size_t size;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return -ENXIO;
	size = resource->end - resource->start + 1;

	if (unlikely(!request_mem_region(port->mapbase, size, "msm_serial")))
		return -EBUSY;

	port->membase = ioremap(port->mapbase, size);
	if (!port->membase) {
		release_mem_region(port->mapbase, size);
		return -EBUSY;
	}

	return 0;
}

static void msm_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_MSM;
		msm_request_port(port);
	}
}

static int msm_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_MSM))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}

static void msm_power(struct uart_port *port, unsigned int state,
		      unsigned int oldstate)
{
#ifndef CONFIG_SERIAL_MSM_CLOCK_CONTROL
	struct msm_port *msm_port = UART_TO_MSM(port);

	switch (state) {
	case 0:
		clk_enable(msm_port->clk);
		break;
	case 3:
		clk_disable(msm_port->clk);
		break;
	default:
		printk(KERN_ERR "msm_serial: Unknown PM state %d\n", state);
	}
#endif
}

static struct uart_ops msm_uart_pops = {
	.tx_empty = msm_tx_empty,
	.set_mctrl = msm_set_mctrl,
	.get_mctrl = msm_get_mctrl,
	.stop_tx = msm_stop_tx,
	.start_tx = msm_start_tx,
	.stop_rx = msm_stop_rx,
	.enable_ms = msm_enable_ms,
	.break_ctl = msm_break_ctl,
	.startup = msm_startup,
	.shutdown = msm_shutdown,
	.set_termios = msm_set_termios,
	.type = msm_type,
	.release_port = msm_release_port,
	.request_port = msm_request_port,
	.config_port = msm_config_port,
	.verify_port = msm_verify_port,
	.pm = msm_power,
};

static struct msm_port msm_uart_ports[] = {
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 512,
			.line = 0,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 512,
			.line = 1,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 2,
		},
	},
};

#define UART_NR	ARRAY_SIZE(msm_uart_ports)

static inline struct uart_port *get_port_from_line(unsigned int line)
{
	return &msm_uart_ports[line].uart;
}

#ifdef CONFIG_SERIAL_MSM_CONSOLE

static void msm_console_putchar(struct uart_port *port, int c)
{
	while (!(msm_read(port, UART_SR) & UART_SR_TX_READY))
		;
	msm_write(port, c, UART_TF);
}

static void msm_console_write(struct console *co, const char *s,
			      unsigned int count)
{
	struct uart_port *port;
	struct msm_port *msm_port;

	BUG_ON(co->index < 0 || co->index >= UART_NR);

	port = get_port_from_line(co->index);
	msm_port = UART_TO_MSM(port);

	spin_lock(&port->lock);
	clk_enable(msm_port->clk);
	uart_console_write(port, s, count, msm_console_putchar);
	clk_disable(msm_port->clk);
	spin_unlock(&port->lock);
}

static int __init msm_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud, flow, bits, parity;

	if (unlikely(co->index >= UART_NR || co->index < 0))
		return -ENXIO;

	port = get_port_from_line(co->index);

	if (unlikely(!port->membase))
		return -ENXIO;

	port->cons = co;

	msm_init_clock(port);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	bits = 8;
	parity = 'n';
	flow = 'n';
	msm_write(port, UART_MR2_BITS_PER_CHAR_8 | UART_MR2_STOP_BIT_LEN_ONE,
		  UART_MR2);	/* 8N1 */

	if (baud < 300 || baud > 115200)
		baud = 115200;
	msm_set_baud_rate(port, baud);

	msm_reset(port);

	printk(KERN_INFO "msm_serial: console setup on port #%d\n", port->line);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver msm_uart_driver;

static struct console msm_console = {
	.name = "ttyMSM",
	.write = msm_console_write,
	.device = uart_console_device,
	.setup = msm_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &msm_uart_driver,
};

#define MSM_CONSOLE	(&msm_console)

#else
#define MSM_CONSOLE	NULL
#endif

static struct uart_driver msm_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "msm_serial",
	.dev_name = "ttyMSM",
	.nr = UART_NR,
	.cons = MSM_CONSOLE,
};

static int __init msm_serial_probe(struct platform_device *pdev)
{
	struct msm_port *msm_port;
	struct resource *resource;
	struct uart_port *port;
	int irq;

	if (unlikely(pdev->id < 0 || pdev->id >= UART_NR))
		return -ENXIO;

	printk(KERN_INFO "msm_serial: detected port #%d\n", pdev->id);

	port = get_port_from_line(pdev->id);
	port->dev = &pdev->dev;
	msm_port = UART_TO_MSM(port);

	msm_port->clk = clk_get(&pdev->dev, "uart_clk");
	if (unlikely(IS_ERR(msm_port->clk)))
		return PTR_ERR(msm_port->clk);
	port->uartclk = clk_get_rate(msm_port->clk);
	printk(KERN_INFO "uartclk = %d\n", port->uartclk);


	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return -ENXIO;
	port->mapbase = resource->start;

	irq = platform_get_irq(pdev, 0);
	if (unlikely(irq < 0))
		return -ENXIO;
	port->irq = irq;

	platform_set_drvdata(pdev, port);

	if (unlikely(set_irq_wake(port->irq, 1)))
		return -ENXIO;

#ifdef CONFIG_SERIAL_MSM_RX_WAKEUP
	if (port->line == 0)  /* BT is serial device 0 */
		if (unlikely(set_irq_wake(MSM_GPIO_TO_INT(45), 1)))
			return -ENXIO;
#endif

#ifdef CONFIG_SERIAL_MSM_CLOCK_CONTROL
	msm_port->clk_state = MSM_CLK_PORT_OFF;
	hrtimer_init(&msm_port->clk_off_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	msm_port->clk_off_timer.function = msm_serial_clock_off;
	msm_port->clk_off_delay = ktime_set(0, 1000000);  /* 1 ms */
#endif

	return uart_add_one_port(&msm_uart_driver, port);
}

static int __devexit msm_serial_remove(struct platform_device *pdev)
{
	struct msm_port *msm_port = platform_get_drvdata(pdev);

	clk_put(msm_port->clk);

	return 0;
}

static struct platform_driver msm_platform_driver = {
	.remove = msm_serial_remove,
	.driver = {
		.name = "msm_serial",
		.owner = THIS_MODULE,
	},
};

static int __init msm_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&msm_uart_driver);
	if (unlikely(ret))
		return ret;

	ret = platform_driver_probe(&msm_platform_driver, msm_serial_probe);
	if (unlikely(ret))
		uart_unregister_driver(&msm_uart_driver);

	printk(KERN_INFO "msm_serial: driver initialized\n");

	return ret;
}

static void __exit msm_serial_exit(void)
{
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	unregister_console(&msm_console);
#endif
	platform_driver_unregister(&msm_platform_driver);
	uart_unregister_driver(&msm_uart_driver);
}

module_init(msm_serial_init);
module_exit(msm_serial_exit);

MODULE_AUTHOR("Robert Love <rlove@google.com>");
MODULE_DESCRIPTION("Driver for msm7x serial device");
MODULE_LICENSE("GPL");
