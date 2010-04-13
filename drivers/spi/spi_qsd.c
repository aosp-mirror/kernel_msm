/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * SPI driver for Qualcomm QSD platforms
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <mach/msm_spi.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <mach/dma.h>
#include <asm/atomic.h>

#define SPI_CONFIG                    0x0000
#define SPI_IO_CONTROL                0x0004
#define SPI_IO_MODES                  0x0008
#define SPI_SW_RESET                  0x000C
#define SPI_TIME_OUT                  0x0010
#define SPI_TIME_OUT_CURRENT          0x0014
#define SPI_MX_OUTPUT_COUNT           0x0018
#define SPI_MX_OUTPUT_CNT_CURRENT     0x001C
#define SPI_MX_INPUT_COUNT            0x0020
#define SPI_MX_INPUT_CNT_CURRENT      0x0024
#define SPI_MX_READ_COUNT             0x0028
#define SPI_MX_READ_CNT_CURRENT       0x002C
#define SPI_OPERATIONAL               0x0030
#define SPI_ERROR_FLAGS               0x0034
#define SPI_ERROR_FLAGS_EN            0x0038
#define SPI_DEASSERT_WAIT             0x003C
#define SPI_OUTPUT_DEBUG              0x0040
#define SPI_INPUT_DEBUG               0x0044
#define SPI_FIFO_WORD_CNT             0x0048
#define SPI_TEST_CTRL                 0x004C
#define SPI_OUTPUT_FIFO               0x0100
#define SPI_INPUT_FIFO                0x0200

/* SPI_CONFIG fields */
#define SPI_CFG_INPUT_FIRST           0x00000200
#define SPI_NO_INPUT                  0x00000080
#define SPI_NO_OUTPUT                 0x00000040
#define SPI_CFG_LOOPBACK              0x00000100
#define SPI_CFG_N                     0x0000001F

/* SPI_IO_CONTROL fields */
#define SPI_IO_C_CLK_IDLE_HIGH        0x00000400
#define SPI_IO_C_MX_CS_MODE           0x00000100
#define SPI_IO_C_CS_N_POLARITY        0x000000F0
#define SPI_IO_C_CS_N_POLARITY_0      0x00000010
#define SPI_IO_C_CS_SELECT            0x0000000C
#define SPI_IO_C_TRISTATE_CS          0x00000002
#define SPI_IO_C_NO_TRI_STATE         0x00000001

/* SPI_IO_MODES fields */
#define SPI_IO_M_OUTPUT_BIT_SHIFT_EN  0x00004000
#define SPI_IO_M_PACK_EN              0x00002000
#define SPI_IO_M_UNPACK_EN            0x00001000
#define SPI_IO_M_INPUT_MODE           0x00000C00
#define SPI_IO_M_OUTPUT_MODE          0x00000300
#define SPI_IO_M_INPUT_FIFO_SIZE      0x000000C0
#define SPI_IO_M_INPUT_BLOCK_SIZE     0x00000030
#define SPI_IO_M_OUTPUT_FIFO_SIZE     0x0000000C
#define SPI_IO_M_OUTPUT_BLOCK_SIZE    0x00000003

/* SPI_OPERATIONAL fields */
#define SPI_OP_MAX_INPUT_DONE_FLAG    0x00000800
#define SPI_OP_MAX_OUTPUT_DONE_FLAG   0x00000400
#define SPI_OP_INPUT_SERVICE_FLAG     0x00000200
#define SPI_OP_OUTPUT_SERVICE_FLAG    0x00000100
#define SPI_OP_INPUT_FIFO_FULL        0x00000080
#define SPI_OP_OUTPUT_FIFO_FULL       0x00000040
#define SPI_OP_IP_FIFO_NOT_EMPTY      0x00000020
#define SPI_OP_OP_FIFO_NOT_EMPTY      0x00000010
#define SPI_OP_STATE_VALID            0x00000004
#define SPI_OP_STATE                  0x00000003
#define SPI_OP_STATE_RESET            0x00000000
#define SPI_OP_STATE_RUN              0x00000001
#define SPI_OP_STATE_PAUSE            0x00000003

/* SPI_ERROR_FLAGS fields */
#define SPI_ERR_TIME_OUT_ERR          0x00000040
#define SPI_ERR_OUTPUT_OVER_RUN_ERR   0x00000020
#define SPI_ERR_INPUT_UNDER_RUN_ERR   0x00000010
#define SPI_ERR_OUTPUT_UNDER_RUN_ERR  0x00000008
#define SPI_ERR_INPUT_OVER_RUN_ERR    0x00000004
#define SPI_ERR_CLK_OVER_RUN_ERR      0x00000002
#define SPI_ERR_CLK_UNDER_RUN_ERR     0x00000001
#define SPI_ERR_MASK                  0x0000007F

/* We don't allow transactions larger than 4K-64 */
#define SPI_MAX_TRANSFERS             0x000FC0
#define SPI_MAX_LEN                   (SPI_MAX_TRANSFERS * dd->bytes_per_word)
#define SPI_MAX_TIMEOUT               0x00010000
#define SPI_MIN_TRANS_TIME            50

#define SPI_NUM_CHIPSELECTS           4
#define SPI_QSD_NAME                  "spi_qsd"

/* Data Mover burst size */
#define DM_BURST_SIZE                 16
/* Data Mover commands should be aligned to 64 bit(8 bytes) */
#define DM_BYTE_ALIGN                 8

enum msm_spi_mode {
	SPI_FIFO_MODE  = 0x0,  /* 00 */
	SPI_BLOCK_MODE = 0x1,  /* 01 */
	SPI_DMOV_MODE  = 0x2,  /* 10 */
	SPI_MODE_NONE  = 0xFF, /* invalid value */
};

/* Structures for Data Mover */
struct spi_dmov_cmd {
	dmov_box box;      /* data aligned to max(dm_burst_size, block_size)
							   (<= fifo_size) */
	dmov_s single_pad; /* data unaligned to max(dm_burst_size, block_size)
			      padded to fit */
	dma_addr_t cmd_ptr;
};

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
MODULE_ALIAS("platform:spi_qsd");

#ifdef CONFIG_DEBUG_FS
/* Used to create debugfs entries */
static const struct {
	const char *name;
	mode_t mode;
	int offset;
} debugfs_spi_regs[] = {
	{"config",                S_IRUGO | S_IWUSR, SPI_CONFIG},
	{"io_control",            S_IRUGO | S_IWUSR, SPI_IO_CONTROL},
	{"io_modes",              S_IRUGO | S_IWUSR, SPI_IO_MODES},
	{"sw_reset",                        S_IWUSR, SPI_SW_RESET},
	{"time_out",              S_IRUGO | S_IWUSR, SPI_TIME_OUT},
	{"time_out_current",      S_IRUGO,           SPI_TIME_OUT_CURRENT},
	{"mx_output_count",       S_IRUGO | S_IWUSR, SPI_MX_OUTPUT_COUNT},
	{"mx_output_cnt_current", S_IRUGO,           SPI_MX_OUTPUT_CNT_CURRENT},
	{"mx_input_count",        S_IRUGO | S_IWUSR, SPI_MX_INPUT_COUNT},
	{"mx_input_cnt_current",  S_IRUGO,           SPI_MX_INPUT_CNT_CURRENT},
	{"mx_read_count",         S_IRUGO | S_IWUSR, SPI_MX_READ_COUNT},
	{"mx_read_cnt_current",   S_IRUGO,           SPI_MX_READ_CNT_CURRENT},
	{"operational",           S_IRUGO | S_IWUSR, SPI_OPERATIONAL},
	{"error_flags",           S_IRUGO | S_IWUSR, SPI_ERROR_FLAGS},
	{"error_flags_en",        S_IRUGO | S_IWUSR, SPI_ERROR_FLAGS_EN},
	{"deassert_wait",         S_IRUGO | S_IWUSR, SPI_DEASSERT_WAIT},
	{"output_debug",          S_IRUGO,           SPI_OUTPUT_DEBUG},
	{"input_debug",           S_IRUGO,           SPI_INPUT_DEBUG},
	{"fifo_word_cnt",         S_IRUGO,           SPI_FIFO_WORD_CNT},
	{"test_ctrl",             S_IRUGO | S_IWUSR, SPI_TEST_CTRL},
	{"output_fifo",                     S_IWUSR, SPI_OUTPUT_FIFO},
	{"input_fifo" ,           S_IRUSR,           SPI_INPUT_FIFO},
};
#endif

struct msm_spi {
	u8                      *read_buf;
	const u8                *write_buf;
	void __iomem		*base;
	struct device           *dev;
	spinlock_t               queue_lock;
	struct list_head         queue;
	struct workqueue_struct	*workqueue;
	struct work_struct       work_data;
	struct spi_message      *cur_msg;
	struct spi_transfer     *cur_transfer;
	struct completion        transfer_complete;
	struct clk              *clk;
	struct clk              *pclk;
	int                      max_clock_speed;
	unsigned long            mem_phys_addr;
	size_t                   mem_size;
	u32                      rx_bytes_remaining;
	u32                      tx_bytes_remaining;
	u32                      clock_speed;
	u32                      irq_in;
	u32                      irq_out;
	u32                      irq_err;
	int                      bytes_per_word;
	bool                     suspended;
	bool                     transfer_in_progress;
	/* DMA data */
	enum msm_spi_mode        mode;
	bool                     use_dma;
	int                      tx_dma_chan;
	int                      tx_dma_crci;
	int                      rx_dma_chan;
	int                      rx_dma_crci;
	/* Data Mover Commands */
	struct spi_dmov_cmd      *tx_dmov_cmd;
	struct spi_dmov_cmd      *rx_dmov_cmd;
	/* Physical address of the tx dmov box command */
	dma_addr_t               tx_dmov_cmd_dma;
	dma_addr_t               rx_dmov_cmd_dma;
	struct msm_dmov_cmd      tx_hdr;
	struct msm_dmov_cmd      rx_hdr;
	int                      block_size;
	int                      burst_size;
	atomic_t                 rx_irq_called;
	/* Used to pad messages unaligned to block size */
	u8                       *tx_padding;
	dma_addr_t               tx_padding_dma;
	u8                       *rx_padding;
	dma_addr_t               rx_padding_dma;
	u32                      unaligned_len;
	/* DMA statistics */
	u32                      stat_dmov_err;
	u32                      stat_rx;
	u32                      stat_tx;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent_spi;
	struct dentry *debugfs_spi_regs[ARRAY_SIZE(debugfs_spi_regs)];
#endif
};

static int input_fifo_size;

static void msm_spi_clock_set(struct msm_spi *dd, int speed)
{
	int rc;

	rc = clk_set_rate(dd->clk, speed);
	if (!rc)
		dd->clock_speed = speed;
}

/* Assumption: input_fifo=output_fifo */
static void __init msm_spi_calculate_fifo_size(struct msm_spi *dd)
{
	u32 spi_iom;
	int block;
	int mult;
	int words;

	spi_iom = readl(dd->base + SPI_IO_MODES);
	block = (spi_iom & SPI_IO_M_INPUT_BLOCK_SIZE) >> 4;
	mult = (spi_iom & SPI_IO_M_INPUT_FIFO_SIZE) >> 6;
	switch (block) {
	case 0:
		words = 1;
		break;
	case 1:
		words = 4;
		break;
	case 2:
		words = 8;
		break;
	default:
		goto fifo_size_err;
	}
	switch (mult) {
	case 0:
		input_fifo_size = words * 2;
		break;
	case 1:
		input_fifo_size = words * 4;
		break;
	case 2:
		input_fifo_size = words * 8;
		break;
	default:
		goto fifo_size_err;
	}

	/* we can't use dma with 8 bytes of fifo since dm burst size is 16 */
	if (input_fifo_size*sizeof(u32) < DM_BURST_SIZE)
		dd->use_dma = 0;
	if (dd->use_dma) {
		dd->block_size = words*sizeof(u32); /* in bytes */
		dd->burst_size = max(dd->block_size, DM_BURST_SIZE);
	}

	return;

fifo_size_err:
	printk(KERN_WARNING "%s: invalid FIFO size, SPI_IO_MODES=0x%x\n",
	       __func__, spi_iom);
	return;
}

static void msm_spi_read_word_from_fifo(struct msm_spi *dd)
{
	u32   data_in;
	int   i;
	int   shift;

	data_in = readl(dd->base + SPI_INPUT_FIFO);
	if (dd->read_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->rx_bytes_remaining; i++) {
			/* The data format depends on bytes_per_word:
			   4 bytes: 0x12345678
			   3 bytes: 0x00123456
			   2 bytes: 0x00001234
			   1 byte : 0x00000012
			*/
			shift = 8 * (dd->bytes_per_word - i - 1);
			*dd->read_buf++ = (data_in & (0xFF << shift)) >> shift;
			dd->rx_bytes_remaining--;
		}
	} else {
		if (dd->rx_bytes_remaining >= dd->bytes_per_word)
			dd->rx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->rx_bytes_remaining = 0;
	}
}

static void msm_spi_setup_dm_transfer(struct msm_spi *dd)
{
	dmov_box *box;
	int bytes_to_send, num_rows, bytes_sent;
	u32 num_transfers, timeout;

	atomic_set(&dd->rx_irq_called, 0);
	bytes_sent = dd->cur_transfer->len - dd->tx_bytes_remaining;
	/* We'll send in chunks of SPI_MAX_LEN if larger */
	bytes_to_send = dd->tx_bytes_remaining / SPI_MAX_LEN ?
			  SPI_MAX_LEN : dd->tx_bytes_remaining;
	num_transfers = DIV_ROUND_UP(bytes_to_send, dd->bytes_per_word);
	dd->unaligned_len = bytes_to_send % dd->burst_size;
	/* We multiply by 8bitsinbyte and multiply by 1.5 for safety */
	timeout = bytes_to_send * 12 > SPI_MAX_TIMEOUT ?
			0 : roundup(bytes_to_send * 12, SPI_MIN_TRANS_TIME);
	num_rows = bytes_to_send / dd->burst_size;

	dd->mode = SPI_DMOV_MODE;

	if (num_rows) {
		/* src in 16 MSB, dst in 16 LSB */
		box = &dd->tx_dmov_cmd->box;
		box->src_row_addr = dd->cur_transfer->tx_dma + bytes_sent;
		box->src_dst_len = (dd->burst_size << 16) | dd->burst_size;
		box->num_rows = (num_rows << 16) | num_rows;
		box->row_offset = (dd->burst_size << 16) | 0;

		box = &dd->rx_dmov_cmd->box;
		box->dst_row_addr = dd->cur_transfer->rx_dma + bytes_sent;
		box->src_dst_len = (dd->burst_size << 16) | dd->burst_size;
		box->num_rows = (num_rows << 16) | num_rows;
		box->row_offset = (0 << 16) | dd->burst_size;

		dd->tx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, box));
		dd->rx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, box));
	} else {
		dd->tx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, single_pad));
		dd->rx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, single_pad));
	}

	if (!dd->unaligned_len) {
		dd->tx_dmov_cmd->box.cmd |= CMD_LC;
		dd->rx_dmov_cmd->box.cmd |= CMD_LC;
	} else {
		dmov_s *tx_cmd = &(dd->tx_dmov_cmd->single_pad);
		dmov_s *rx_cmd = &(dd->rx_dmov_cmd->single_pad);
		u32 offset = dd->cur_transfer->len - dd->unaligned_len;

		dd->tx_dmov_cmd->box.cmd &= ~CMD_LC;
		dd->rx_dmov_cmd->box.cmd &= ~CMD_LC;

		memset(dd->tx_padding, 0, dd->burst_size);
		memset(dd->rx_padding, 0, dd->burst_size);
		if (dd->write_buf)
			memcpy(dd->tx_padding, dd->write_buf + offset,
			       dd->unaligned_len);

		tx_cmd->src = dd->tx_padding_dma;
		rx_cmd->dst = dd->rx_padding_dma;
		tx_cmd->len = rx_cmd->len = dd->burst_size;
	}
	/* This also takes care of the padding dummy buf
	   Since this is set to the correct length, the
	   dummy bytes won't be actually sent */
	if (dd->write_buf)
		writel(num_transfers, dd->base + SPI_MX_OUTPUT_COUNT);
	if (dd->read_buf)
		writel(num_transfers, dd->base + SPI_MX_INPUT_COUNT);
	/* Write timeout */
	writel(timeout, dd->base + SPI_TIME_OUT);
}

static void msm_spi_enqueue_dm_commands(struct msm_spi *dd)
{
	if (dd->write_buf)
		msm_dmov_enqueue_cmd(dd->tx_dma_chan, &dd->tx_hdr);
	if (dd->read_buf)
		msm_dmov_enqueue_cmd(dd->rx_dma_chan, &dd->rx_hdr);
}

/* SPI core can send maximum of 4K transfers, because there is HW problem
   with infinite mode.
   Therefore, we are sending several chunks of 3K or less (depending on how
   much is left).
   Upon completion we send the next chunk, or complete the transfer if
   everything is finished.
*/
static int msm_spi_dm_send_next(struct msm_spi *dd)
{
	/* By now we should have sent all the bytes in FIFO mode,
	 * However to make things right, we'll check anyway.
	 */
	if (dd->mode != SPI_DMOV_MODE)
		return 0;

	/* We need to send more chunks, if we sent max last time */
	if (dd->tx_bytes_remaining > SPI_MAX_LEN) {
		dd->tx_bytes_remaining -= SPI_MAX_LEN;
		writel((readl(dd->base + SPI_OPERATIONAL)
			& ~SPI_OP_STATE) | SPI_OP_STATE_PAUSE,
			dd->base + SPI_OPERATIONAL);
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);

		writel((readl(dd->base + SPI_OPERATIONAL)
			& ~SPI_OP_STATE) | SPI_OP_STATE_RUN,
			dd->base + SPI_OPERATIONAL);
		return 1;
	}

	return 0;
}

static inline void msm_spi_ack_transfer(struct msm_spi *dd)
{
	writel(SPI_OP_MAX_INPUT_DONE_FLAG | SPI_OP_MAX_OUTPUT_DONE_FLAG,
	       dd->base + SPI_OPERATIONAL);
	writel(0, dd->base + SPI_TIME_OUT);
}

static irqreturn_t msm_spi_input_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;

	dd->stat_rx++;

	if (dd->mode == SPI_MODE_NONE)
		return IRQ_HANDLED;

	if (dd->mode == SPI_DMOV_MODE) {
		u32 op = readl(dd->base + SPI_OPERATIONAL);
		if ((!dd->read_buf || op & SPI_OP_MAX_INPUT_DONE_FLAG) &&
		    (!dd->write_buf || op & SPI_OP_MAX_OUTPUT_DONE_FLAG)) {
			msm_spi_ack_transfer(dd);
			if (dd->unaligned_len == 0) {
				if (atomic_inc_return(&dd->rx_irq_called) == 1)
					return IRQ_HANDLED;
			}
			complete(&dd->transfer_complete);
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}

	/* fifo mode */
	while ((readl(dd->base + SPI_OPERATIONAL) & SPI_OP_IP_FIFO_NOT_EMPTY) &&
	       (dd->rx_bytes_remaining > 0)) {
		msm_spi_read_word_from_fifo(dd);
	}
	if (dd->rx_bytes_remaining == 0)
		complete(&dd->transfer_complete);

	return IRQ_HANDLED;
}

static void msm_spi_write_word_to_fifo(struct msm_spi *dd)
{
	u32    word;
	u8     byte;
	int    i;

	word = 0;
	if (dd->write_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->tx_bytes_remaining; i++) {
			dd->tx_bytes_remaining--;
			byte = *dd->write_buf++;
			word |= (byte << (BITS_PER_BYTE * (3 - i)));
		}
	} else
		if (dd->tx_bytes_remaining > dd->bytes_per_word)
			dd->tx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->tx_bytes_remaining = 0;
	writel(word, dd->base + SPI_OUTPUT_FIFO);
}

static irqreturn_t msm_spi_output_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;
	int                     count = 0;

	dd->stat_tx++;

	if (dd->mode == SPI_MODE_NONE)
		return IRQ_HANDLED;

	if (dd->mode == SPI_DMOV_MODE) {
		/* TX_ONLY transaction is handled here
		   This is the only place we send complete at tx and not rx */
		if (dd->read_buf == NULL && readl(dd->base + SPI_OPERATIONAL) &
					    SPI_OP_MAX_OUTPUT_DONE_FLAG) {
			msm_spi_ack_transfer(dd);
			complete(&dd->transfer_complete);
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}

	/* Output FIFO is empty. Transmit any outstanding write data. */
	/* There could be one word in input FIFO, so don't send more  */
	/* than input_fifo_size - 1 more words.                       */
	while ((dd->tx_bytes_remaining > 0) &&
	       (count < input_fifo_size - 1) &&
	       !(readl(dd->base + SPI_OPERATIONAL) & SPI_OP_OUTPUT_FIFO_FULL)) {
		msm_spi_write_word_to_fifo(dd);
		count++;
	}

	return IRQ_HANDLED;
}

static irqreturn_t msm_spi_error_irq(int irq, void *dev_id)
{
	struct spi_master	*master = dev_id;
	struct msm_spi          *dd = spi_master_get_devdata(master);
	u32                      spi_err;

	spi_err = readl(dd->base + SPI_ERROR_FLAGS);
	if (spi_err & SPI_ERR_TIME_OUT_ERR) {
		dev_warn(master->dev.parent, "SPI timeout error\n");
		msm_dmov_flush(dd->tx_dma_chan);
		msm_dmov_flush(dd->rx_dma_chan);
	}
	if (spi_err & SPI_ERR_OUTPUT_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output overrun error\n");
	if (spi_err & SPI_ERR_INPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI input underrun error\n");
	if (spi_err & SPI_ERR_OUTPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output underrun error\n");
	if (spi_err & SPI_ERR_INPUT_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI input overrun error\n");
	if (spi_err & SPI_ERR_CLK_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock overrun error\n");
	if (spi_err & SPI_ERR_CLK_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock underrun error\n");
	writel(SPI_ERR_MASK, dd->base + SPI_ERROR_FLAGS);
	return IRQ_HANDLED;
}

static void msm_spi_unmap_dma_buffers(struct msm_spi *dd)
{
	struct device *dev;

	dev = &dd->cur_msg->spi->dev;
	if (!dd->cur_msg->is_dma_mapped) {
		if (dd->cur_transfer->rx_buf)
			dma_unmap_single(dev, dd->cur_transfer->rx_dma,
					 dd->cur_transfer->len,
					 DMA_FROM_DEVICE);
		if (dd->cur_transfer->tx_buf)
			dma_unmap_single(dev, dd->cur_transfer->tx_dma,
					 dd->cur_transfer->len,
					 DMA_TO_DEVICE);
	}
	/* If we padded the transfer, we copy it from the padding buf */
	if (dd->unaligned_len && dd->read_buf) {
		u32 offset = dd->cur_transfer->len - dd->unaligned_len;
		memcpy(dd->read_buf + offset, dd->rx_padding,
		       dd->unaligned_len);
	}
}

/**
 * msm_use_dm - decides whether to use data mover for this
 * 		transfer
 * @dd:       device
 * @tr:       transfer
 *
 * Start using DM if:
 * 1. Transfer is longer than 3*block size.
 * 2. Buffers should be aligned to cache line.
 * 3. If Transfer is bigger than max_output_count, we accept only aligned to
 *    block size transfers.
 */
static inline int msm_use_dm(struct msm_spi *dd, struct spi_transfer *tr)
{
	u32 cache_line = dma_get_cache_alignment();

	if (!dd->use_dma)
		return 0;

	if (tr->len < 3*dd->block_size)
		return 0;

	if (tr->tx_buf) {
		if (!IS_ALIGNED((size_t)tr->tx_buf, cache_line))
			return 0;
	}
	if (tr->rx_buf) {
		if (!IS_ALIGNED((size_t)tr->rx_buf, cache_line))
			return 0;
	}
	return 1;
}

static void msm_spi_process_transfer(struct msm_spi *dd)
{
	u8  bpw;
	u32 spi_config;
	u32 spi_ioc;
	u32 spi_iom;
	u32 spi_ioc_orig;
	u32 max_speed;
	u32 chip_select;
	u32 read_count;
	u32 timeout;

	if (!dd->cur_transfer->len)
		return;
	dd->tx_bytes_remaining = dd->cur_transfer->len;
	dd->rx_bytes_remaining = dd->cur_transfer->len;
	dd->read_buf           = dd->cur_transfer->rx_buf;
	dd->write_buf          = dd->cur_transfer->tx_buf;
	init_completion(&dd->transfer_complete);
	if (dd->cur_transfer->bits_per_word)
		bpw = dd->cur_transfer->bits_per_word;
	else
		if (dd->cur_msg->spi->bits_per_word)
			bpw = dd->cur_msg->spi->bits_per_word;
		else
			bpw = 8;
	dd->bytes_per_word = (bpw + 7) / 8;

	if (dd->cur_transfer->speed_hz)
		max_speed = dd->cur_transfer->speed_hz;
	else
		max_speed = dd->cur_msg->spi->max_speed_hz;
	if (!dd->clock_speed || max_speed < dd->clock_speed)
		msm_spi_clock_set(dd, max_speed);

	read_count = DIV_ROUND_UP(dd->cur_transfer->len, dd->bytes_per_word);
	if (!msm_use_dm(dd, dd->cur_transfer)) {
		dd->mode = SPI_FIFO_MODE;
		/* read_count cannot exceed fifo_size, and only one READ COUNT
		   interrupt is generated per transaction, so for transactions
		   larger than fifo size READ COUNT must be disabled.
		   For those transactions we usually move to Data Mover mode.
		*/
		if (read_count <= input_fifo_size)
			writel(read_count, dd->base + SPI_MX_READ_COUNT);
		else
			writel(0, dd->base + SPI_MX_READ_COUNT);
	} else
		dd->mode = SPI_DMOV_MODE;

	/* Write mode - fifo or data mover*/
	spi_iom = readl(dd->base + SPI_IO_MODES);
	spi_iom &= ~(SPI_IO_M_INPUT_MODE | SPI_IO_M_OUTPUT_MODE);
	spi_iom = (spi_iom | (dd->mode << 10));
	spi_iom = (spi_iom | (dd->mode << 8));
	/* Turn on packing for data mover */
	if (dd->mode == SPI_DMOV_MODE)
		spi_iom |= SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN;
	else
		spi_iom &= ~(SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN);
	writel(spi_iom, dd->base + SPI_IO_MODES);

	spi_config = readl(dd->base + SPI_CONFIG);
	if ((bpw - 1) != (spi_config & SPI_CFG_N))
		spi_config = (spi_config & ~SPI_CFG_N) | (bpw - 1);
	if (dd->cur_msg->spi->mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;
	if (dd->cur_msg->spi->mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;
	spi_config &= ~(SPI_NO_INPUT|SPI_NO_OUTPUT);
	if (dd->mode == SPI_DMOV_MODE) {
		if (dd->read_buf == NULL)
			spi_config |= SPI_NO_INPUT;
		if (dd->write_buf == NULL)
			spi_config |= SPI_NO_OUTPUT;
	}
	writel(spi_config, dd->base + SPI_CONFIG);

	spi_ioc = readl(dd->base + SPI_IO_CONTROL);
	spi_ioc_orig = spi_ioc;
	if (dd->cur_msg->spi->mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;
	chip_select = dd->cur_msg->spi->chip_select << 2;
	if ((spi_ioc & SPI_IO_C_CS_SELECT) != chip_select)
		spi_ioc = (spi_ioc & ~SPI_IO_C_CS_SELECT) | chip_select;
	if (!dd->cur_transfer->cs_change)
		spi_ioc |= SPI_IO_C_MX_CS_MODE;
	if (spi_ioc != spi_ioc_orig)
		writel(spi_ioc, dd->base + SPI_IO_CONTROL);

	if (dd->mode == SPI_DMOV_MODE) {
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);
	}
	/* The output fifo interrupt handler will handle all writes after
	   the first. Restricting this to one write avoids contention
	   issues and race conditions between this thread and the int handler
	*/
	else if (dd->mode == SPI_FIFO_MODE)
		msm_spi_write_word_to_fifo(dd);

	/* Only enter the RUN state after the first word is written into
	   the output FIFO.  Otherwise, the output FIFO EMPTY interrupt
	   might fire before the first word is written resulting in a
	   possible race condition.
	 */
	writel((readl(dd->base + SPI_OPERATIONAL)
		& ~SPI_OP_STATE) | SPI_OP_STATE_RUN,
	       dd->base + SPI_OPERATIONAL);

	timeout = 100 * msecs_to_jiffies(
	      DIV_ROUND_UP(dd->cur_transfer->len * 8,
		 max_speed / MSEC_PER_SEC));
	do {
		if (!wait_for_completion_timeout(&dd->transfer_complete,
						 timeout)) {
				dev_err(dd->dev, "%s: SPI transaction "
						 "timeout\n", __func__);
				dd->cur_msg->status = -EIO;
				if (dd->mode == SPI_DMOV_MODE) {
					writel(0, dd->base + SPI_TIME_OUT);
					msm_dmov_flush(dd->tx_dma_chan);
					msm_dmov_flush(dd->rx_dma_chan);
				}
				break;
		}
	} while (msm_spi_dm_send_next(dd));

	if (dd->mode == SPI_DMOV_MODE)
		msm_spi_unmap_dma_buffers(dd);
	dd->mode = SPI_MODE_NONE;

	writel(spi_ioc & ~SPI_IO_C_MX_CS_MODE, dd->base + SPI_IO_CONTROL);
	writel((readl(dd->base + SPI_OPERATIONAL)
		& ~SPI_OP_STATE) | SPI_OP_STATE_RESET,
	       dd->base + SPI_OPERATIONAL);
}

/* workqueue - pull messages from queue & process */
static void msm_spi_workq(struct work_struct *work)
{
	struct msm_spi      *dd =
		container_of(work, struct msm_spi, work_data);
	unsigned long        flags;
	u32                  spi_op;
	bool                 status_error = 0;

	spi_op = readl(dd->base + SPI_OPERATIONAL);
	if (spi_op & SPI_OP_STATE_VALID) {
		spi_op &= ~SPI_OP_STATE;
		spi_op |= SPI_OP_STATE_RUN;
	} else {
		dev_err(dd->dev, "%s: SPI operational state not valid\n",
			__func__);
		status_error = 1;
	}

	dd->transfer_in_progress = 1;
	spin_lock_irqsave(&dd->queue_lock, flags);
	while (!list_empty(&dd->queue)) {
		dd->cur_msg = list_entry(dd->queue.next,
					 struct spi_message, queue);
		list_del_init(&dd->cur_msg->queue);
		spin_unlock_irqrestore(&dd->queue_lock, flags);
		if (status_error)
			dd->cur_msg->status = -EIO;
		else {
			list_for_each_entry(dd->cur_transfer,
					    &dd->cur_msg->transfers,
					    transfer_list) {
				msm_spi_process_transfer(dd);
				if (dd->cur_msg->status == -EINPROGRESS)
					dd->cur_msg->status = 0;
			}
		}
		if (dd->cur_msg->complete)
			dd->cur_msg->complete(dd->cur_msg->context);
		spin_lock_irqsave(&dd->queue_lock, flags);
	}
	spin_unlock_irqrestore(&dd->queue_lock, flags);
	dd->transfer_in_progress = 0;
}

static int msm_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct msm_spi	*dd;
	unsigned long    flags;
	struct spi_transfer *tr;

	dd = spi_master_get_devdata(spi->master);
	if (dd->suspended)
		return -EBUSY;

	if (list_empty(&msg->transfers) || !msg->complete)
		return -EINVAL;

	list_for_each_entry(tr, &msg->transfers, transfer_list) {
		void *tx_buf = (void *)tr->tx_buf;
		void       *rx_buf = tr->rx_buf;
		unsigned len = tr->len;

		/* Check message parameters */
		if (tr->speed_hz > dd->max_clock_speed || (tr->bits_per_word &&
			 (tr->bits_per_word < 4 || tr->bits_per_word > 32)) ||
			 (tx_buf == NULL && rx_buf == NULL)) {
			dev_err(&spi->dev, "Invalid transfer: %d Hz, %d bpw"
					   "tx=%p, rx=%p\n",
					    tr->speed_hz, tr->bits_per_word,
					    tx_buf, rx_buf);
			goto error;
		}

		if (!msm_use_dm(dd, tr) || msg->is_dma_mapped)
			continue;

		/* Do DMA mapping "early" for better error reporting */
		if (tx_buf != NULL) {
			tr->tx_dma = dma_map_single(&spi->dev, tx_buf, len,
						     DMA_TO_DEVICE);
			if (dma_mapping_error(NULL, tr->tx_dma)) {
				dev_err(&spi->dev, "dma %cX %d bytes error\n",
						   'T', len);
				goto error;
			}
		}

		if (rx_buf != NULL) {
			tr->rx_dma = dma_map_single(&spi->dev, rx_buf, len,
					DMA_FROM_DEVICE);
			if (dma_mapping_error(NULL, tr->rx_dma)) {
				dev_err(&spi->dev, "dma %cX %d bytes error\n",
						   'R', len);
				if (tx_buf != NULL)
					dma_unmap_single(NULL, tr->tx_dma,
							len, DMA_TO_DEVICE);
				goto error;
			}
		}
	}

	spin_lock_irqsave(&dd->queue_lock, flags);
	list_add_tail(&msg->queue, &dd->queue);
	spin_unlock_irqrestore(&dd->queue_lock, flags);
	queue_work(dd->workqueue, &dd->work_data);
	return 0;

error:
	list_for_each_entry_continue_reverse(tr, &msg->transfers, transfer_list)
	{
		if (msm_use_dm(dd, tr) && !msg->is_dma_mapped) {
			if (tr->rx_buf != NULL)
				dma_unmap_single(&spi->dev, tr->rx_dma, tr->len,
						 DMA_TO_DEVICE);
			if (tr->tx_buf != NULL)
				dma_unmap_single(&spi->dev, tr->tx_dma, tr->len,
						  DMA_FROM_DEVICE);
		}
	}
	return -EINVAL;
}

static int msm_spi_setup(struct spi_device *spi)
{
	struct msm_spi	*dd;
	int              rc = 0;
	u32              spi_ioc;
	u32              spi_config;
	u32              mask;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;
	if (spi->bits_per_word < 4 || spi->bits_per_word > 32) {
		dev_err(&spi->dev, "%s: invalid bits_per_word %d\n",
			__func__, spi->bits_per_word);
		rc = -EINVAL;
	}
	if (spi->mode & ~(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LOOP)) {
		dev_err(&spi->dev, "%s, unsupported mode bits %x\n",
			__func__,
			spi->mode & ~(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH
							  | SPI_LOOP));
		rc = -EINVAL;
	}
	if (spi->chip_select > SPI_NUM_CHIPSELECTS-1) {
		dev_err(&spi->dev, "%s, chip select %d exceeds max value %d\n",
			__func__, spi->chip_select, SPI_NUM_CHIPSELECTS - 1);
		rc = -EINVAL;
	}

	if (rc)
		goto err_setup_exit;

	dd = spi_master_get_devdata(spi->master);
	spi_ioc = readl(dd->base + SPI_IO_CONTROL);
	mask = SPI_IO_C_CS_N_POLARITY_0 << spi->chip_select;
	if (spi->mode & SPI_CS_HIGH)
		spi_ioc |= mask;
	else
		spi_ioc &= ~mask;
	if (spi->mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;
	writel(spi_ioc, dd->base + SPI_IO_CONTROL);

	spi_config = readl(dd->base + SPI_CONFIG);
	if (spi->mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;
	if (spi->mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;
	writel(spi_config, dd->base + SPI_CONFIG);

err_setup_exit:
	return rc;
}

#ifdef CONFIG_DEBUG_FS
static int debugfs_iomem_x32_set(void *data, u64 val)
{
	iowrite32(val, data);
	wmb();
	return 0;
}

static int debugfs_iomem_x32_get(void *data, u64 *val)
{
	*val = ioread32(data);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_iomem_x32, debugfs_iomem_x32_get,
			debugfs_iomem_x32_set, "0x%08llx\n");

static void spi_debugfs_init(struct msm_spi *dd)
{
	dd->dent_spi = debugfs_create_dir(dev_name(dd->dev), NULL);
	if (dd->dent_spi) {
		int i;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++) {
			dd->debugfs_spi_regs[i] =
			   debugfs_create_file(
			       debugfs_spi_regs[i].name,
			       debugfs_spi_regs[i].mode,
			       dd->dent_spi,
			       dd->base + debugfs_spi_regs[i].offset,
			       &fops_iomem_x32);
		}
	}
}

static void spi_debugfs_exit(struct msm_spi *dd)
{
	if (dd->dent_spi) {
		int i;
		debugfs_remove_recursive(dd->dent_spi);
		dd->dent_spi = NULL;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++)
			dd->debugfs_spi_regs[i] = NULL;
	}
}
#else
static void spi_debugfs_init(struct msm_spi *dd) {}
static void spi_debugfs_exit(struct msm_spi *dd) {}
#endif

/* ===Device attributes begin=== */
static ssize_t show_stats(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct msm_spi *dd =  spi_master_get_devdata(master);

	return snprintf(buf, PAGE_SIZE,
			"Device       %s\n"
			"use_dma ?    %s\n"
			"DMA configuration:\n"
			"tx_ch=%d, rx_ch=%d, tx_crci= %d, rx_crci=%d\n"
			"--statistics--\n"
			"Rx isrs  = %d\n"
			"Tx isrs  = %d\n"
			"DMA error  = %d\n"
			"--debug--\n"
			"NA yet\n",
			dev_name(dev),
			dd->use_dma ? "yes" : "no",
			dd->tx_dma_chan,
			dd->rx_dma_chan,
			dd->tx_dma_crci,
			dd->rx_dma_crci,
			dd->stat_rx,
			dd->stat_tx,
			dd->stat_dmov_err
			);
}

/* Reset statistics on write */
static ssize_t set_stats(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct msm_spi *dd = dev_get_drvdata(dev);
	dd->stat_rx = 0;
	dd->stat_tx = 0;
	dd->stat_dmov_err = 0;
	return count;
}

static DEVICE_ATTR(stats, S_IRUGO | S_IWUSR, show_stats, set_stats);

static struct attribute *dev_attrs[] = {
	&dev_attr_stats.attr,
	NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};
/* ===Device attributes end=== */

/**
 * spi_dmov_tx_complete_func - DataMover tx completion callback
 *
 * Executed in IRQ context (Data Mover's IRQ) DataMover's
 * spinlock @msm_dmov_lock held.
 */
static void spi_dmov_tx_complete_func(struct msm_dmov_cmd *cmd,
				      unsigned int result,
				      struct msm_dmov_errdata *err)
{
	struct msm_spi *dd;

	if (!(result & DMOV_RSLT_VALID)) {
		pr_err("Invalid DMOV result: rc=0x%08x, cmd = %p", result, cmd);
		return;
	}
	/* restore original context */
	dd = container_of(cmd, struct msm_spi, tx_hdr);
	if (result & DMOV_RSLT_DONE)
		dd->stat_tx++;
	else {
		/* Error or flush */
		if (result & DMOV_RSLT_ERROR) {
			dev_err(dd->dev, "DMA error (0x%08x)\n", result);
			dd->stat_dmov_err++;
		}
		if (result & DMOV_RSLT_FLUSH) {
			/*
			 * Flushing normally happens in process of
			 * removing, when we are waiting for outstanding
			 * DMA commands to be flushed.
			 */
			dev_info(dd->dev,
				 "DMA channel flushed (0x%08x)\n", result);
		}
		if (err)
			dev_err(dd->dev,
				"Flush data(%08x %08x %08x %08x %08x %08x)\n",
				err->flush[0], err->flush[1], err->flush[2],
				err->flush[3], err->flush[4], err->flush[5]);
		dd->cur_msg->status = -EIO;
		writel(0, dd->base + SPI_TIME_OUT);
		complete(&dd->transfer_complete);
	}
}

/**
 * spi_dmov_rx_complete_func - DataMover rx completion callback
 *
 * Executed in IRQ context (Data Mover's IRQ)
 * DataMover's spinlock @msm_dmov_lock held.
 */
static void spi_dmov_rx_complete_func(struct msm_dmov_cmd *cmd,
				      unsigned int result,
				      struct msm_dmov_errdata *err)
{
	struct msm_spi *dd;

	if (!(result & DMOV_RSLT_VALID)) {
		pr_err("Invalid DMOV result(rc = 0x%08x, cmd = %p)",
		       result, cmd);
		return;
	}
	/* restore original context */
	dd = container_of(cmd, struct msm_spi, rx_hdr);
	if (result & DMOV_RSLT_DONE) {
		dd->stat_rx++;
		if (atomic_inc_return(&dd->rx_irq_called) == 1)
			return;
		complete(&dd->transfer_complete);
	} else {
		/** Error or flush  */
		if (result & DMOV_RSLT_ERROR) {
			dev_err(dd->dev, "DMA error(0x%08x)\n", result);
			dd->stat_dmov_err++;
		}
		if (result & DMOV_RSLT_FLUSH) {
			dev_info(dd->dev,
				"DMA channel flushed(0x%08x)\n", result);
		}
		if (err)
			dev_err(dd->dev,
				"Flush data(%08x %08x %08x %08x %08x %08x)\n",
				err->flush[0], err->flush[1], err->flush[2],
				err->flush[3], err->flush[4], err->flush[5]);
		dd->cur_msg->status = -EIO;
		writel(0, dd->base + SPI_TIME_OUT);
		complete(&dd->transfer_complete);
	}
}

static inline u32 get_chunk_size(struct msm_spi *dd)
{
	u32 cache_line = dma_get_cache_alignment();

	return (roundup(sizeof(struct spi_dmov_cmd), DM_BYTE_ALIGN) +
			  roundup(dd->burst_size, cache_line))*2;
}

static void msm_spi_teardown_dma(struct msm_spi *dd)
{
	int limit = 0;

	if (!dd->use_dma)
		return;

	while (dd->mode == SPI_DMOV_MODE && limit++ < 50) {
		msm_dmov_flush(dd->tx_dma_chan);
		msm_dmov_flush(dd->rx_dma_chan);
		msleep(10);
	}

	dma_free_coherent(NULL, get_chunk_size(dd), dd->tx_dmov_cmd,
			  dd->tx_dmov_cmd_dma);
	dd->tx_dmov_cmd = dd->rx_dmov_cmd = NULL;
	dd->tx_padding = dd->rx_padding = NULL;
}

static __init int msm_spi_init_dma(struct msm_spi *dd)
{
	dmov_box *box;
	u32 cache_line = dma_get_cache_alignment();

	/* Allocate all as one chunk, since all is smaller than page size */

	/* We send NULL device, since it requires coherent_dma_mask id
	   device definition, we're okay with using system pool */
	dd->tx_dmov_cmd = dma_alloc_coherent(NULL, get_chunk_size(dd),
					     &dd->tx_dmov_cmd_dma, GFP_KERNEL);
	if (dd->tx_dmov_cmd == NULL)
		return -ENOMEM;

	/* DMA addresses should be 64 bit aligned aligned */
	dd->rx_dmov_cmd = (struct spi_dmov_cmd *)
			  ALIGN((size_t)&dd->tx_dmov_cmd[1], DM_BYTE_ALIGN);
	dd->rx_dmov_cmd_dma = ALIGN(dd->tx_dmov_cmd_dma +
			      sizeof(struct spi_dmov_cmd), DM_BYTE_ALIGN);

	/* Buffers should be aligned to cache line */
	dd->tx_padding = (u8 *)ALIGN((size_t)&dd->rx_dmov_cmd[1], cache_line);
	dd->tx_padding_dma = ALIGN(dd->rx_dmov_cmd_dma +
			      sizeof(struct spi_dmov_cmd), cache_line);
	dd->rx_padding = (u8 *)ALIGN((size_t)(dd->tx_padding + dd->burst_size),
				     cache_line);
	dd->rx_padding_dma = ALIGN(dd->tx_padding_dma + dd->burst_size,
				      cache_line);

	/* Setup DM commands */
	box = &(dd->rx_dmov_cmd->box);
	box->cmd = CMD_MODE_BOX | CMD_SRC_CRCI(dd->rx_dma_crci);
	box->src_row_addr = (uint32_t)dd->mem_phys_addr + SPI_INPUT_FIFO;
	dd->rx_hdr.cmdptr = DMOV_CMD_PTR_LIST |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, cmd_ptr));
	dd->rx_hdr.complete_func = spi_dmov_rx_complete_func;

	box = &(dd->tx_dmov_cmd->box);
	box->cmd = CMD_MODE_BOX | CMD_DST_CRCI(dd->tx_dma_crci);
	box->dst_row_addr = (uint32_t)dd->mem_phys_addr + SPI_OUTPUT_FIFO;
	dd->tx_hdr.cmdptr = DMOV_CMD_PTR_LIST |
			    DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
			    offsetof(struct spi_dmov_cmd, cmd_ptr));
	dd->tx_hdr.complete_func = spi_dmov_tx_complete_func;

	dd->tx_dmov_cmd->single_pad.cmd = CMD_MODE_SINGLE | CMD_LC |
					  CMD_DST_CRCI(dd->tx_dma_crci);
	dd->tx_dmov_cmd->single_pad.dst = (uint32_t)dd->mem_phys_addr +
					   SPI_OUTPUT_FIFO;
	dd->rx_dmov_cmd->single_pad.cmd = CMD_MODE_SINGLE | CMD_LC |
					  CMD_SRC_CRCI(dd->rx_dma_crci);
	dd->rx_dmov_cmd->single_pad.src = (uint32_t)dd->mem_phys_addr +
					  SPI_INPUT_FIFO;

	/* Clear remaining activities on channel */
	msm_dmov_flush(dd->tx_dma_chan);
	msm_dmov_flush(dd->rx_dma_chan);

	return 0;
}

static int __init msm_spi_probe(struct platform_device *pdev)
{
	struct spi_master      *master;
	struct msm_spi	       *dd;
	struct resource	       *resource;
	int			rc = 0;
	struct clk	       *pclk;
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	master = spi_alloc_master(&pdev->dev, sizeof(struct msm_spi));
	if (!master) {
		rc = -ENOMEM;
		dev_err(&pdev->dev, "master allocation failed\n");
		goto err_probe_exit;
	}

	master->bus_num        = pdev->id;
	master->num_chipselect = SPI_NUM_CHIPSELECTS;
	master->setup          = msm_spi_setup;
	master->transfer       = msm_spi_transfer;
	platform_set_drvdata(pdev, master);
	dd = spi_master_get_devdata(master);

	dd->irq_in  = platform_get_irq_byname(pdev, "irq_in");
	dd->irq_out = platform_get_irq_byname(pdev, "irq_out");
	dd->irq_err = platform_get_irq_byname(pdev, "irq_err");
	if ((dd->irq_in < 0) || (dd->irq_out < 0) || (dd->irq_err < 0))
		goto err_probe_res;

	resource  = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		rc = -ENXIO;
		goto err_probe_res;
	}
	dd->mem_phys_addr = resource->start;
	dd->mem_size = (resource->end - resource->start) + 1;

	if (pdata && pdata->dma_config) {
		rc = pdata->dma_config();
		if (!rc) {
			resource  = platform_get_resource_byname(pdev,
					IORESOURCE_DMA, "spidm_channels");
			if (resource) {
				dd->rx_dma_chan = resource->start;
				dd->tx_dma_chan = resource->end;

				resource  = platform_get_resource_byname(pdev,
						IORESOURCE_DMA, "spidm_crci");
				if (!resource) {
					rc = -ENXIO;
					goto err_probe_res;
				}
				dd->rx_dma_crci = resource->start;
				dd->tx_dma_crci = resource->end;
				dd->use_dma = 1;
			}
		}
	}

	if (pdata && pdata->gpio_config) {
		rc = pdata->gpio_config();
		if (rc) {
			dev_err(&pdev->dev, "%s: error configuring GPIOs\n",
			       __func__);
			goto err_probe_gpio;
		}
	}

	spin_lock_init(&dd->queue_lock);
	INIT_LIST_HEAD(&dd->queue);
	INIT_WORK(&dd->work_data, msm_spi_workq);
	dd->workqueue = create_singlethread_workqueue(
		dev_name(master->dev.parent));
	if (!dd->workqueue)
		goto err_probe_workq;

	if (!request_mem_region(dd->mem_phys_addr, dd->mem_size,
				SPI_QSD_NAME)) {
		rc = -ENXIO;
		goto err_probe_reqmem;
	}

	dd->base = ioremap(dd->mem_phys_addr, dd->mem_size);
	if (!dd->base)
		goto err_probe_ioremap;

	dd->dev = &pdev->dev;
	dd->clk = clk_get(&pdev->dev, "spi_clk");
	if (IS_ERR(dd->clk)) {
		dev_err(&pdev->dev, "%s: unable to get spi_clk\n", __func__);
		rc = PTR_ERR(dd->clk);
		goto err_probe_clk_get;
	}
	rc = clk_enable(dd->clk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable spi_clk\n",
			__func__);
		goto err_probe_clk_enable;
	}
	pclk = clk_get(&pdev->dev, "spi_pclk");
	if (!IS_ERR(pclk)) {
		dd->pclk = pclk;
		rc = clk_enable(dd->pclk);
		if (rc) {
			dev_err(&pdev->dev, "%s: unable to enable spi_pclk\n",
				__func__);
			goto err_probe_pclk_enable;
		}
	}
	if (pdata && pdata->max_clock_speed) {
		msm_spi_clock_set(dd, pdata->max_clock_speed);
		dd->max_clock_speed = pdata->max_clock_speed;
	}
	msm_spi_calculate_fifo_size(dd);
	writel(0x1, dd->base + SPI_SW_RESET);
	if (dd->use_dma) {
		rc = msm_spi_init_dma(dd);
		if (rc)
			goto err_probe_dma;
	}
	writel(0x00000000, dd->base + SPI_OPERATIONAL);
	writel(0x00000000, dd->base + SPI_CONFIG);
	writel(0x00000000, dd->base + SPI_IO_MODES);
	writel(SPI_IO_C_NO_TRI_STATE, dd->base + SPI_IO_CONTROL);
	if (!(readl(dd->base + SPI_OPERATIONAL) & SPI_OP_STATE_VALID)) {
		dev_err(&pdev->dev, "%s: SPI operational state not valid\n",
			__func__);
		rc = -1;
		goto err_probe_state;
	}
	writel(SPI_OP_STATE_RUN, dd->base + SPI_OPERATIONAL);

	dd->suspended = 0;
	dd->transfer_in_progress = 0;
	dd->mode = SPI_MODE_NONE;

	rc = request_irq(dd->irq_in, msm_spi_input_irq, IRQF_TRIGGER_RISING,
			  pdev->name, dd);
	if (rc)
		goto err_probe_irq1;
	rc = request_irq(dd->irq_out, msm_spi_output_irq, IRQF_TRIGGER_RISING,
			  pdev->name, dd);
	if (rc)
		goto err_probe_irq2;
	rc = request_irq(dd->irq_err, msm_spi_error_irq, IRQF_TRIGGER_RISING,
			  pdev->name, master);
	if (rc)
		goto err_probe_irq3;

	rc = spi_register_master(master);
	if (rc)
		goto err_probe_reg_master;

	rc = sysfs_create_group(&(dd->dev->kobj), &dev_attr_grp);
	if (rc) {
		dev_err(&pdev->dev, "failed to create dev. attrs : %d\n", rc);
		goto err_attrs;
	}

	spi_debugfs_init(dd);

	return 0;

err_attrs:
err_probe_reg_master:
	free_irq(dd->irq_err, master);
err_probe_irq3:
	free_irq(dd->irq_out, dd);
err_probe_irq2:
	free_irq(dd->irq_in, dd);
err_probe_irq1:
err_probe_state:
	msm_spi_teardown_dma(dd);
err_probe_dma:
	if (dd->pclk)
		clk_disable(dd->pclk);
err_probe_pclk_enable:
	if (dd->pclk)
		clk_put(dd->pclk);
	clk_disable(dd->clk);
err_probe_clk_enable:
	clk_put(dd->clk);
err_probe_clk_get:
	iounmap(dd->base);
err_probe_ioremap:
	release_mem_region(dd->mem_phys_addr, dd->mem_size);
err_probe_reqmem:
	destroy_workqueue(dd->workqueue);
err_probe_workq:
err_probe_gpio:
	if (pdata && pdata->gpio_release)
		pdata->gpio_release();
err_probe_res:
	spi_master_put(master);
err_probe_exit:
	return rc;
}

#ifdef CONFIG_PM
static int msm_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd;
	int                limit = 0;

	if (!master)
		goto suspend_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto suspend_exit;
	dd->suspended = 1;
	while ((!list_empty(&dd->queue) || dd->transfer_in_progress) &&
	       limit < 50) {
		if (dd->mode == SPI_DMOV_MODE) {
			msm_dmov_flush(dd->tx_dma_chan);
			msm_dmov_flush(dd->rx_dma_chan);
		}
		limit++;
		msleep(1);
	}

	disable_irq(dd->irq_in);
	disable_irq(dd->irq_out);
	disable_irq(dd->irq_err);
	clk_disable(dd->clk);

suspend_exit:
	return 0;
}

static int msm_spi_resume(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd;
	int rc;

	if (!master)
		goto resume_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto resume_exit;

	rc = clk_enable(dd->clk);
	if (rc) {
		dev_err(dd->dev, "%s: unable to enable spi_clk\n",
			__func__);
		goto resume_exit;
	}

	enable_irq(dd->irq_in);
	enable_irq(dd->irq_out);
	enable_irq(dd->irq_err);
	dd->suspended = 0;
resume_exit:
	return 0;
}
#else
#define msm_spi_suspend NULL
#define msm_spi_resume NULL
#endif /* CONFIG_PM */

static int __devexit msm_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd = spi_master_get_devdata(master);
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	spi_debugfs_exit(dd);
	sysfs_remove_group(&pdev->dev.kobj, &dev_attr_grp);

	free_irq(dd->irq_in, dd);
	free_irq(dd->irq_out, dd);
	free_irq(dd->irq_err, master);

	msm_spi_teardown_dma(dd);

	if (pdata && pdata->gpio_release)
		pdata->gpio_release();

	iounmap(dd->base);
	release_mem_region(dd->mem_phys_addr, dd->mem_size);
	clk_disable(dd->clk);
	clk_put(dd->clk);
	if (dd->pclk) {
		clk_disable(dd->pclk);
		clk_put(dd->pclk);
	}
	destroy_workqueue(dd->workqueue);
	platform_set_drvdata(pdev, 0);
	spi_unregister_master(master);
	spi_master_put(master);

	return 0;
}

static struct platform_driver msm_spi_driver = {
	.probe          = msm_spi_probe,
	.driver		= {
		.name	= "msm_spi",
		.owner	= THIS_MODULE,
	},
	.suspend        = msm_spi_suspend,
	.resume         = msm_spi_resume,
	.remove		= __exit_p(msm_spi_remove),
};

static int __init msm_spi_init(void)
{
	return platform_driver_register(&msm_spi_driver);
}
module_init(msm_spi_init);

static void __exit msm_spi_exit(void)
{
	platform_driver_unregister(&msm_spi_driver);
}
module_exit(msm_spi_exit);
