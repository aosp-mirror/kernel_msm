/*
 * dap_swd.c - Linux kernel modules for DAP-SWD
 *
 * Copyright (C), 2008-2019, OPPO Mobile Comm Corp., Ltd.
 * Author: Zeng Zhaoxiu <zengzhaoxiu@oppo.com>
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
 * along with this program.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "dap_swd.h"
#include <../oppo_sensorhub/oppo_sensorhub.h>

#define SWD_DEBUG

/*
 * SWD Transfer Request
 */
#define SWD_REQ_APnDP_BIT	0
#define SWD_REQ_RnW_BIT		1
#define SWD_REQ_REG_BIT		2
#define SWD_REQ_PARITY_BIT	4
#define SWD_REQ_STOP_BIT	5
#define SWD_REQ_PARK_BIT	6

#define SWD_REQ_REG_MASK	(0x3 << SWD_REQ_REG_BIT)
#define SWD_REQ_REG(n)		((n) << SWD_REQ_REG_BIT)

#define SWD_PKT_REQ(RnW, APnDP, reg, stop, park) \
({ \
	u8 req; \
	req  = !!(APnDP) << SWD_REQ_APnDP_BIT; \
	req |= !!(RnW) << SWD_REQ_RnW_BIT; \
	req |= (reg) & SWD_REQ_REG_MASK; \
	req |= parity4(req) << SWD_REQ_PARITY_BIT; \
	req |= !!(stop) << SWD_REQ_STOP_BIT; \
	req |= !!(park) << SWD_REQ_PARK_BIT; \
	req; \
})

#define SWD_REQ_READ_DP(reg)	SWD_PKT_REQ(1, 0, (reg), 0, 1)
#define SWD_REQ_WRITE_DP(reg)	SWD_PKT_REQ(0, 0, (reg), 0, 1)
#define SWD_REQ_READ_AP(reg)	SWD_PKT_REQ(1, 1, (reg), 0, 1)
#define SWD_REQ_WRITE_AP(reg)	SWD_PKT_REQ(0, 1, (reg), 0, 1)

/*
 * SWD Transfer Response
 */
#define SWD_ACK_OK				0x1
#define SWD_ACK_WAIT			0x2
#define SWD_ACK_FAULT			0x4
#define SWD_ACK_PARITY			0x8

/*
 * Debug Port Register
 */
#define DP_IDCODE				0x00	// IDCODE Register (SW Read only)
#define DP_ABORT				0x00	// Abort Register (SW Write only)
#define DP_CTRL_STAT			0x04	// Control & Status
#define DP_WCR					0x04	// Wire Control Register (SW Only)
#define DP_SELECT				0x08	// Select Register (JTAG R/W & SW W)
#define DP_RESEND				0x08	// Resend (SW Read Only)
#define DP_RDBUFF				0x0C	// Read Buffer (Read Only)

// Debug Abort Register definitions
#define DAPABORT				0x00000001	// DAP Abort
#define STKCMPCLR				0x00000002	// Clear STICKYCMP Flag (SW Only)
#define STKERRCLR				0x00000004	// Clear STICKYERR Flag (SW Only)
#define WDERRCLR				0x00000008	// Clear WDATAERR Flag (SW Only)
#define ORUNERRCLR				0x00000010	// Clear STICKYORUN Flag (SW Only)

// Debug Control and Status definitions
#define ORUNDETECT				0x00000001	// Overrun Detect
#define STICKYORUN				0x00000002	// Sticky Overrun
#define TRNMODE					0x0000000C	// Transfer Mode Mask
#define TRNNORMAL				0x00000000	// Transfer Mode: Normal
#define TRNVERIFY				0x00000004	// Transfer Mode: Pushed Verify
#define TRNCOMPARE				0x00000008	// Transfer Mode: Pushed Compare
#define STICKYCMP				0x00000010	// Sticky Compare
#define STICKYERR				0x00000020	// Sticky Error
#define READOK					0x00000040	// Read OK (SW Only)
#define WDATAERR				0x00000080	// Write Data Error (SW Only)
#define MASKLANE				0x00000F00	// Mask Lane Mask
#define MASKLANE0				0x00000100	// Mask Lane 0
#define MASKLANE1				0x00000200	// Mask Lane 1
#define MASKLANE2				0x00000400	// Mask Lane 2
#define MASKLANE3				0x00000800	// Mask Lane 3
#define TRNCNT					0x001FF000	// Transaction Counter Mask
#define CDBGRSTREQ				0x04000000	// Debug Reset Request
#define CDBGRSTACK				0x08000000	// Debug Reset Acknowledge
#define CDBGPWRUPREQ			0x10000000	// Debug Power-up Request
#define CDBGPWRUPACK			0x20000000	// Debug Power-up Acknowledge
#define CSYSPWRUPREQ			0x40000000	// System Power-up Request
#define CSYSPWRUPACK			0x80000000	// System Power-up Acknowledge

// Debug Select Register definitions
#define CTRLSEL					0x00000001	// CTRLSEL (SW Only)
#define APBANKSEL				0x000000F0	// APBANKSEL Mask
#define APSEL					0xFF000000	// APSEL Mask

/*
 * Access Port Register Addresses
 */
#define AP_CSW					0x00	// Control and Status Word
#define AP_TAR					0x04	// Transfer Address
#define AP_DRW					0x0C	// Data Read/Write
#define AP_BD0					0x10	// Banked Data 0
#define AP_BD1					0x14	// Banked Data 1
#define AP_BD2					0x18	// Banked Data 2
#define AP_BD3					0x1C	// Banked Data 3
#define AP_ROM					0xF8	// Debug ROM Address
#define AP_IDR					0xFC	// Identification Register

// AP Control and Status Word definitions
#define CSW_SIZE				0x00000007	// Access Size: Selection Mask
#define CSW_SIZE8				0x00000000	// Access Size: 8-bit
#define CSW_SIZE16				0x00000001	// Access Size: 16-bit
#define CSW_SIZE32				0x00000002	// Access Size: 32-bit
#define CSW_ADDRINC				0x00000030	// Auto Address Increment Mask
#define CSW_NADDRINC			0x00000000	// No Address Increment
#define CSW_SADDRINC			0x00000010	// Single Address Increment
#define CSW_PADDRINC			0x00000020	// Packed Address Increment
#define CSW_DBGSTAT				0x00000040	// Debug Status
#define CSW_TINPROG				0x00000080	// Transfer in progress
#define CSW_HPROT				0x02000000	// User/Privilege Control
#define CSW_MSTRTYPE			0x20000000	// Master Type Mask
#define CSW_MSTRCORE			0x00000000	// Master Type: Core
#define CSW_MSTRDBG				0x20000000	// Master Type: Debug
#define CSW_RESERVED			0x01000000	// Reserved Value

// AP CSW register, base value
#define CSW_VALUE	(CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)

/*
 * Default Core debug base addresses
 */
#define DBG_Addr     0xe000edf0

// Debug Halting Control and Status Register definitions
#define DBG_HCSR       (DBG_Addr + 0x00)
#define C_DEBUGEN      0x00000001  // Debug Enable
#define C_HALT         0x00000002  // Halt
#define C_STEP         0x00000004  // Step
#define C_MASKINTS     0x00000008  // Mask Interrupts
#define C_SNAPSTALL    0x00000020  // Snap Stall
#define S_REGRDY       0x00010000  // Register R/W Ready Flag
#define S_HALT         0x00020000  // Halt Flag
#define S_SLEEP        0x00040000  // Sleep Flag
#define S_LOCKUP       0x00080000  // Lockup Flag
#define S_RETIRE_ST    0x01000000  // Sticky Retire Flag
#define S_RESET_ST     0x02000000  // Sticky Reset Flag
#define DBGKEY         0xA05F0000  // Debug Key

// Debug Core Register Selector Register
#define DBG_CRSR       (DBG_Addr + 0x04)
#define REGWnR         (1 << 16)

// Debug Core Register Data Register
#define DBG_CRDR       (DBG_Addr + 0x08)

// Debug Exception and Monitor Control Register definitions
#define DBG_EMCR       (DBG_Addr + 0x0C)
#define VC_CORERESET   0x00000001  // Reset Vector Catch
#define VC_MMERR       0x00000010  // Debug Trap on MMU Fault
#define VC_NOCPERR     0x00000020  // Debug Trap on No Coprocessor Fault
#define VC_CHKERR      0x00000040  // Debug Trap on Checking Error Fault
#define VC_STATERR     0x00000080  // Debug Trap on State Error Fault
#define VC_BUSERR      0x00000100  // Debug Trap on Bus Error Fault
#define VC_INTERR      0x00000200  // Debug Trap on Interrupt Error Fault
#define VC_HARDERR     0x00000400  // Debug Trap on Hard Fault
#define MON_EN         0x00010000  // Monitor Enable
#define MON_PEND       0x00020000  // Monitor Pend
#define MON_STEP       0x00040000  // Monitor Step
#define MON_REQ        0x00080000  // Monitor Request
#define TRCENA         0x01000000  // Trace Enable (DWT, ITM, ETM, TPIU)

// This can vary from target to target and should be in the structure or flash blob
#define TARGET_AUTO_INCREMENT_PAGE_SIZE	(1024)

#define SCB_AIRCR_PRIGROUP_Pos	8								/*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk	(7UL << SCB_AIRCR_PRIGROUP_Pos)	/*!< SCB AIRCR: PRIGROUP Mask */

#define MAX_SWD_RETRY	100

#ifdef CONFIG_PINCTRL_MSM
extern int msm_gpio_get_reg(struct gpio_desc *desc, struct msm_gpio_reg *reg);
#endif

struct swd_ctx {
	struct device *dev;
	struct pinctrl *pinctrl;
#ifdef CONFIG_PINCTRL_MSM
	struct msm_gpio_reg gpio_clk_reg;
	struct msm_gpio_reg gpio_data_reg;
#else
	struct gpio_desc *gpio_clk;
	struct gpio_desc *gpio_data;
#endif

	struct miscdevice misc;
	struct notifier_block rst_nb;

	struct mutex timing_lock;
	unsigned int turnaround;	// Turnaround period
	unsigned int idle_cycles;	// Idle cycles after transfer
	bool data_phase;			// Always generate Data Phase

	uint32_t apsel;				// APSEL for the family
	uint32_t soft_reset;
	bool connect_under_reset;

	atomic_t opened;
	struct mutex lock;
	bool connected;
	bool halted;
	uint8_t *io_buf;
	uint32_t idcode;
	uint32_t dp_select;
	uint32_t ap_csw;
#ifdef SWD_DEBUG
	unsigned int sysfs_mem_addr;
	unsigned int sysfs_reg_nr;
#endif
};

#define AMBIQ_APOLLO3_ID		0x2ba01477
#define REG_APOLLO3_BOOTLDR		0x400401A0
#define REG_APOLLO3_SCRATCH0	0x400401B0

static int swd_xfer_retry(struct swd_ctx *swd, uint8_t req, uint32_t *data);

/*
 * The probe outputs data to SWDIO on the falling edge of SWDCLK.
 * The probe captures data from SWDIO on the rising edge of SWDCLK.
 * The target outputs data to SWDIO on the rising edge of SWDCLK.
 * The target captures data from SWDIO on the rising edge of SWDCLK.
 */
#ifdef CONFIG_PINCTRL_MSM

#define MSM_GPIO_CTL_OE		BIT(9)
#define MSM_GPIO_IO_IN		BIT(0)
#define MSM_GPIO_IO_OUT		BIT(1)

#define SWD_CLK_OUT_HI		writel(MSM_GPIO_IO_OUT, swd->gpio_clk_reg.io_reg)
#define SWD_CLK_OUT_LO		writel(0, swd->gpio_clk_reg.io_reg)

#define SWD_DATA_DIR_OUT	writel(data_ctl | MSM_GPIO_CTL_OE, swd->gpio_data_reg.ctl_reg)
#define SWD_DATA_DIR_IN		writel(data_ctl & ~MSM_GPIO_CTL_OE, swd->gpio_data_reg.ctl_reg)
#define SWD_DATA_OUT(x)		writel((x) ? MSM_GPIO_IO_OUT : 0, swd->gpio_data_reg.io_reg)
#define SWD_DATA_IN			(readl(swd->gpio_data_reg.io_reg) & MSM_GPIO_IO_IN)

static __always_inline void swd_strobe_clk(struct swd_ctx *swd)
{
	SWD_CLK_OUT_HI;
	SWD_CLK_OUT_LO;
}

static __always_inline void swd_strobe_nclks(struct swd_ctx *swd, unsigned int nclks)
{
	unsigned int i;

	for (i = 0; i < nclks; i++) {
		swd_strobe_clk(swd);
	}
}

static __always_inline void swd_tx_bit(struct swd_ctx *swd, unsigned int val)
{
	SWD_DATA_OUT(val);
	SWD_CLK_OUT_HI;
	SWD_CLK_OUT_LO;
}

static __always_inline unsigned int swd_rx_bit(struct swd_ctx *swd)
{
	unsigned int res;

	res = SWD_DATA_IN;
	SWD_CLK_OUT_HI;
	SWD_CLK_OUT_LO;

	return res;
}

/* LSB-first */
static __always_inline void swd_tx_nbits(struct swd_ctx *swd, uint32_t val, unsigned int nbits)
{
	unsigned int i;

	for (i = 0; i < nbits; i++) {
		swd_tx_bit(swd, val & 1);
		val >>= 1;
	}
}

static __always_inline uint32_t swd_rx_nbits(struct swd_ctx *swd, unsigned int nbits)
{
	uint32_t res = 0;
	unsigned int i;

	for (i = 0; i < nbits; i++) {
		res |= swd_rx_bit(swd) << i;
	}

	return res;
}

/*
 * Completes one serial wire packet transfer.
 * Expects SWDIO to be an output on entry.
 */
static inline int swd_xfer_pkt(struct swd_ctx *swd, uint8_t req, uint32_t *pdata)
{
	u32 data_ctl = readl(swd->gpio_data_reg.ctl_reg);
	int ack;

	/* Packet Request */
	swd_tx_nbits(swd, req << 1 | 1, 8);		/* add start bit */

	/* Turnaround */
	SWD_DATA_DIR_IN;
	swd_strobe_nclks(swd, swd->turnaround);

	/* Acknowledge response */
	ack = swd_rx_nbits(swd, 3);

	switch (ack) {
	case SWD_ACK_OK: /* Data transfer */
		{
			uint32_t data;
			unsigned int parity;

			if (req & BIT(SWD_REQ_RnW_BIT)) {
				/* Read data */
				data = swd_rx_nbits(swd, 32);
				parity = swd_rx_bit(swd);

				if (parity32(data) != parity)
					ack = SWD_ACK_PARITY;
				if (pdata)
					*pdata = le32_to_cpu(data);

				/* Turnaround */
				swd_strobe_nclks(swd, swd->turnaround);
				SWD_DATA_OUT(0);
				SWD_DATA_DIR_OUT;
			} else {
				/* Turnaround */
				swd_strobe_nclks(swd, swd->turnaround);
				SWD_DATA_OUT(0);
				SWD_DATA_DIR_OUT;

				data = cpu_to_le32p(pdata);
				parity = parity32(data);

				/* Write data */
				swd_tx_nbits(swd, data, 32);
				swd_tx_bit(swd, parity);

				SWD_DATA_OUT(0);
			}

			/* Idle cycles */
			if (swd->idle_cycles) {
				swd_strobe_nclks(swd, swd->idle_cycles);
			}
		}
		break;

	case SWD_ACK_WAIT:
	case SWD_ACK_FAULT: /* WAIT or FAULT response */
		if (swd->data_phase && (req & BIT(SWD_REQ_RnW_BIT))) {
			/* Dummy Read RDATA[0:31] + Parity */
			swd_strobe_nclks(swd, 32 + 1);
		}

		/* Turnaround */
		swd_strobe_nclks(swd, swd->turnaround);
		SWD_DATA_OUT(0);
		SWD_DATA_DIR_OUT;

		if (swd->data_phase && !(req & BIT(SWD_REQ_RnW_BIT))) {
			/* Dummy Write WDATA[0:31] + Parity */
			swd_strobe_nclks(swd, 32 + 1);
		}
		break;

	default: /* Protocol error */
		/* Back off data phase */
		swd_strobe_nclks(swd, swd->turnaround + 32 + 1);
		SWD_DATA_OUT(0);
		SWD_DATA_DIR_OUT;
		break;
	}

	return ack;
}

static __always_inline int swd_read_dp(struct swd_ctx *swd, uint8_t adr, uint32_t *val)
{
	uint8_t req = SWD_REQ_READ_DP(adr);
	return swd_xfer_retry(swd, req, val);
}

static inline int jtag_to_swd(struct swd_ctx *swd)
{
	u32 data_ctl = readl(swd->gpio_data_reg.ctl_reg);
	int ack;

	/*
	 * Puts the SWD into the reset state by holding the data signal HIGH for at
	 * least 50 clock cycles.
	 */
	SWD_DATA_OUT(1);
	SWD_DATA_DIR_OUT;
	swd_strobe_nclks(swd, 56);

	/*
	 * Select the Serial Wire Debug Port
	 * Skip this switch sequence if the device does not have the swj_dp port
	 * Serial Wire + JTAG
	 */
	swd_tx_nbits(swd, 0xE79E, 16);

	/*
	 * A line reset is achieved by holding the data signal HIGH for at
	 * least 50 clock cycles, followed by at least two idle cycles.
	 */
	SWD_DATA_OUT(1);
	swd_strobe_nclks(swd, 56);
	SWD_DATA_OUT(0);
	swd_strobe_nclks(swd, 8);

	ack = swd_read_dp(swd, DP_IDCODE, &swd->idcode);
	if (ack != SWD_ACK_OK)
		return ack;

	dev_info(swd->dev, "ID = %08x", swd->idcode);
	return SWD_ACK_OK;
}

static inline void swd_to_jtag(struct swd_ctx *swd)
{
	u32 data_ctl = readl(swd->gpio_data_reg.ctl_reg);

	/*
	 * Puts the SWD into the reset state by holding the data signal HIGH for at
	 * least 50 clock cycles.
	 */
	SWD_DATA_OUT(1);
	SWD_DATA_DIR_OUT;
	swd_strobe_nclks(swd, 56);

	/*
	 * Select the JTAG Debug Port
     * Skip this switch sequence if the device does not have the swj_dp port
     * Serial Wire + JTAG
	 */
	swd_tx_nbits(swd, 0xE73C, 16);
}

#else

static __always_inline void swd_strobe_clk(struct swd_ctx *swd)
{
	gpiod_set_raw_value(swd->gpio_clk, 1);
	gpiod_set_raw_value(swd->gpio_clk, 0);
}

static __always_inline void swd_strobe_nclks(struct swd_ctx *swd, unsigned int nclks)
{
	unsigned int i;

	for (i = 0; i < nclks; i++) {
		swd_strobe_clk(swd);
	}
}

static __always_inline void swd_tx_bit(struct swd_ctx *swd, unsigned int val)
{
	gpiod_set_raw_value(swd->gpio_data, val ? 1 : 0);
	gpiod_set_raw_value(swd->gpio_clk, 1);
	gpiod_set_raw_value(swd->gpio_clk, 0);
}

static __always_inline unsigned int swd_rx_bit(struct swd_ctx *swd)
{
	unsigned int res;

	res = gpiod_get_raw_value(swd->gpio_data);
	gpiod_set_raw_value(swd->gpio_clk, 1);
	gpiod_set_raw_value(swd->gpio_clk, 0);
	return res;
}

/* LSB-first */
static __always_inline void swd_tx_nbits(struct swd_ctx *swd, uint32_t val, unsigned int nbits)
{
	unsigned int i;

	for (i = 0; i < nbits; i++) {
		swd_tx_bit(swd, val & 1);
		val >>= 1;
	}
}

static __always_inline uint32_t swd_rx_nbits(struct swd_ctx *swd, unsigned int nbits)
{
	uint32_t res = 0;
	unsigned int i;

	for (i = 0; i < nbits; i++) {
		res |= swd_rx_bit(swd) << i;
	}

	return res;
}

/*
 * Completes one serial wire packet transfer.
 * Expects SWDIO to be an output on entry.
 */
static inline int swd_xfer_pkt(struct swd_ctx *swd, uint8_t req, uint32_t *pdata)
{
	int ack;

	/* Packet Request */
	swd_tx_nbits(swd, req << 1 | 1, 8);		/* add start bit */

	/* Turnaround */
	gpiod_direction_input(swd->gpio_data);
	swd_strobe_nclks(swd, swd->turnaround);

	/* Acknowledge response */
	ack = swd_rx_nbits(swd, 3);

	switch (ack) {
	case SWD_ACK_OK: /* Data transfer */
		{
			uint32_t data;
			unsigned int parity;

			if (req & BIT(SWD_REQ_RnW_BIT)) {
				/* Read data */
				data = swd_rx_nbits(swd, 32);
				parity = swd_rx_bit(swd);

				if (parity32(data) != parity)
					ack = SWD_ACK_PARITY;
				if (pdata)
					*pdata = le32_to_cpu(data);

				/* Turnaround */
				swd_strobe_nclks(swd, swd->turnaround);
				gpiod_direction_output_raw(swd->gpio_data, 0);
			} else {
				/* Turnaround */
				swd_strobe_nclks(swd, swd->turnaround);
				gpiod_direction_output_raw(swd->gpio_data, 0);

				data = cpu_to_le32p(pdata);
				parity =  parity32(data);

				/* Write data */
				swd_tx_nbits(swd, data, 32);
				swd_tx_bit(swd, parity);

				gpiod_set_raw_value(swd->gpio_data, 0);
			}

			/* Idle cycles */
			if (swd->idle_cycles) {
				swd_strobe_nclks(swd, swd->idle_cycles);
			}
		}
		break;

	case SWD_ACK_WAIT:
	case SWD_ACK_FAULT: /* WAIT or FAULT response */
		if (swd->data_phase && (req & BIT(SWD_REQ_RnW_BIT))) {
			/* Dummy Read RDATA[0:31] + Parity */
			swd_strobe_nclks(swd, 32 + 1);
		}

		/* Turnaround */
		swd_strobe_nclks(swd, swd->turnaround);
		gpiod_direction_output_raw(swd->gpio_data, 0);

		if (swd->data_phase && !(req & BIT(SWD_REQ_RnW_BIT))) {
			/* Dummy Write WDATA[0:31] + Parity */
			swd_strobe_nclks(swd, 32 + 1);
		}
		break;

	default: /* Protocol error */
		/* Back off data phase */
		swd_strobe_nclks(swd, swd->turnaround + 32 + 1);
		gpiod_direction_output_raw(swd->gpio_data, 0);
		break;
	}

	return ack;
}

static __always_inline int swd_read_dp(struct swd_ctx *swd, uint8_t adr, uint32_t *val)
{
	uint8_t req = SWD_REQ_READ_DP(adr);
	return swd_xfer_retry(swd, req, val);
}

static inline int jtag_to_swd(struct swd_ctx *swd)
{
	int ack;

	/*
	 * Puts the SWD into the reset state by holding the data signal HIGH for at
	 * least 50 clock cycles.
	 */
	gpiod_direction_output_raw(swd->gpio_data, 1);
	swd_strobe_nclks(swd, 56);

	/*
	 * Select the Serial Wire Debug Port
	 * Skip this switch sequence if the device does not have the swj_dp port
	 * Serial Wire + JTAG
	 */
	swd_tx_nbits(swd, 0xE79E, 16);

	/*
	 * A line reset is achieved by holding the data signal HIGH for at
	 * least 50 clock cycles, followed by at least two idle cycles.
	 */
	gpiod_set_raw_value(swd->gpio_data, 1);
	swd_strobe_nclks(swd, 56);
	gpiod_set_raw_value(swd->gpio_data, 0);
	swd_strobe_nclks(swd, 8);

	ack = swd_read_dp(swd, DP_IDCODE, &swd->idcode);
	if (ack != SWD_ACK_OK)
		return ack;

	dev_info(swd->dev, "ID = %08x", swd->idcode);
	return SWD_ACK_OK;
}

static inline void swd_to_jtag(struct swd_ctx *swd)
{
	/*
	 * Puts the SWD into the reset state by holding the data signal HIGH for at
	 * least 50 clock cycles.
	 */
	gpiod_direction_output_raw(swd->gpio_data, 1);
	swd_strobe_nclks(swd, 56);

	/*
	 * Select the JTAG Debug Port
     * Skip this switch sequence if the device does not have the swj_dp port
     * Serial Wire + JTAG
	 */
	swd_tx_nbits(swd, 0xE73C, 16);
}

#endif

static int swd_xfer_retry(struct swd_ctx *swd, uint8_t req, uint32_t *data)
{
	uint8_t i, ack;

	for (i = 0; i < MAX_SWD_RETRY; i++) {
		mutex_lock(&swd->timing_lock);
		ack = swd_xfer_pkt(swd, req, data);
		mutex_unlock(&swd->timing_lock);

		if (ack != SWD_ACK_WAIT)
			break;

		mdelay(1);
	}

	return ack;
}

static inline int swd_write_dp(struct swd_ctx *swd, uint8_t adr, uint32_t val)
{
	uint8_t req;
	int ack;

	/* check if the right bank is already selected */
	if ((adr == DP_SELECT) && (swd->dp_select == val)) {
		return SWD_ACK_OK;
	}

	req = SWD_REQ_WRITE_DP(adr);

	ack = swd_xfer_retry(swd, req, &val);
	if ((ack == SWD_ACK_OK) && (adr == DP_SELECT)) {
		swd->dp_select = val;
	}

	return ack;
}

static __always_inline uint32_t swd_get_apsel(struct swd_ctx *swd, uint32_t adr)
{
	if (!swd->apsel)
		return adr & APSEL;
	else
		return swd->apsel;
}

#define SWD_REG_ADR(a)	((a) & 0x0c)

static inline int swd_read_ap(struct swd_ctx *swd, uint32_t adr, uint32_t *val)
{
	uint32_t apsel = swd_get_apsel(swd, adr);
	uint32_t bank_sel = adr & APBANKSEL;
	int ack;
	uint8_t req;

	ack = swd_write_dp(swd, DP_SELECT, apsel | bank_sel);
	if (ack != SWD_ACK_OK)
		return ack;

	req = SWD_REQ_READ_AP(SWD_REG_ADR(adr));

	/* first dummy read */
	ack = swd_xfer_retry(swd, req, NULL);
	if (ack != SWD_ACK_OK)
		return ack;

	return swd_xfer_retry(swd, req, val);
}

static inline int swd_write_ap(struct swd_ctx *swd, uint32_t adr, uint32_t val)
{
	uint32_t apsel = swd_get_apsel(swd, adr);
	uint32_t bank_sel = adr & APBANKSEL;
	int ack;
	uint8_t req;

	ack = swd_write_dp(swd, DP_SELECT, apsel | bank_sel);
	if (ack != SWD_ACK_OK)
		return ack;

	if ((adr == AP_CSW) && (swd->ap_csw == val))
		return SWD_ACK_OK;

	req = SWD_REQ_WRITE_AP(SWD_REG_ADR(adr));

	ack = swd_xfer_retry(swd, req, &val);
	if (ack != SWD_ACK_OK)
		return ack;

	ack = swd_read_dp(swd, DP_RDBUFF, NULL);
	if ((ack == SWD_ACK_OK) && (adr == AP_CSW))
		swd->ap_csw = val;

	return ack;
}

/*
 * Read 32-bit word aligned values from target memory using address auto-increment.
 */
static int swd_read_block(struct swd_ctx *swd, uint32_t addr, uint32_t *data, uint32_t words)
{
	int ack;
	uint8_t req;

	if (words == 0)
		return SWD_ACK_OK;

	/* CSW register */
	ack = swd_write_ap(swd, AP_CSW, CSW_VALUE | CSW_SIZE32);
	if (ack != SWD_ACK_OK)
		return ack;

	/* TAR write */
	req = SWD_REQ_WRITE_AP(SWD_REG_ADR(AP_TAR));

	ack = swd_xfer_retry(swd, req, &addr);
	if (ack != SWD_ACK_OK)
		return ack;

	/* read data */
	req = SWD_REQ_READ_AP(SWD_REG_ADR(AP_DRW));

	/* initiate first read, data comes back in next read */
	ack = swd_xfer_retry(swd, req, NULL);
	if (ack != SWD_ACK_OK)
		return ack;

	while (words-- > 1) {
		ack = swd_xfer_retry(swd, req, data);
		if (ack != SWD_ACK_OK)
			return ack;
		data++;
	}

	/* read last word */
	return swd_read_dp(swd, DP_RDBUFF, data);
}

/*
 * Write 32-bit word aligned values to target memory using address auto-increment.
 */
static int swd_write_block(struct swd_ctx *swd, uint32_t addr, uint32_t *data, uint32_t words)
{
	uint8_t req;
	int ack;

	if (words == 0)
		return SWD_ACK_OK;

	/* CSW register */
	ack = swd_write_ap(swd, AP_CSW, CSW_VALUE | CSW_SIZE32);
	if (ack != SWD_ACK_OK)
		return ack;

	/* TAR write */
	req = SWD_REQ_WRITE_AP(SWD_REG_ADR(AP_TAR));

	ack = swd_xfer_retry(swd, req, &addr);
	if (ack != SWD_ACK_OK)
		return ack;

	/* write data */
	req = SWD_REQ_WRITE_AP(SWD_REG_ADR(AP_DRW));

	while (words-- > 0) {
		ack = swd_xfer_retry(swd, req, data);
		if (ack != SWD_ACK_OK)
			return ack;
		data++;
	}

	/* dummy read */
	return swd_read_dp(swd, DP_RDBUFF, NULL);
}

static inline int swd_read_data(struct swd_ctx *swd, uint32_t addr, uint32_t *val)
{
	uint8_t req;
	int ack;

	/* put address in TAR register */
	req = SWD_REQ_WRITE_AP(SWD_REG_ADR(AP_TAR));

	ack = swd_xfer_retry(swd, req, &addr);
	if (ack != SWD_ACK_OK)
		return ack;

	/* read data */
	req = SWD_REQ_READ_AP(SWD_REG_ADR(AP_DRW));

	/* initiate first read, data comes back in next read */
	ack = swd_xfer_retry(swd, req, NULL);
	if (ack != SWD_ACK_OK)
		return ack;

	return swd_read_dp(swd, DP_RDBUFF, val);
}

static int swd_read_word(struct swd_ctx *swd, uint32_t addr, uint32_t *val)
{
	int ack;

	ack = swd_write_ap(swd, AP_CSW, CSW_VALUE | CSW_SIZE32);
	if (ack != SWD_ACK_OK)
		return ack;

	return swd_read_data(swd, addr, val);
}

static int swd_read_byte(struct swd_ctx *swd, uint32_t addr, uint8_t *val)
{
	uint32_t tmp;
	int ack;

	ack = swd_write_ap(swd, AP_CSW, CSW_VALUE | CSW_SIZE8);
	if (ack != SWD_ACK_OK)
		return ack;

	ack = swd_read_data(swd, addr, &tmp);
	if (ack != SWD_ACK_OK)
		return ack;

	*val = (uint8_t)(tmp >> ((addr & 0x03) * 8));
	return SWD_ACK_OK;
}

static inline int swd_write_data(struct swd_ctx *swd, uint32_t addr, uint32_t data)
{
	uint8_t req;
	int ack;

	/* put address in TAR register */
	req = SWD_REQ_WRITE_AP(SWD_REG_ADR(AP_TAR));

	ack = swd_xfer_retry(swd, req, &addr);
	if (ack != SWD_ACK_OK)
		return ack;

	/* write data */
	req = SWD_REQ_WRITE_AP(SWD_REG_ADR(AP_DRW));

	ack = swd_xfer_retry(swd, req, &data);
	if (ack != SWD_ACK_OK)
		return ack;

	/* dummy read */
	return swd_read_dp(swd, DP_RDBUFF, NULL);
}

static int swd_write_word(struct swd_ctx *swd, uint32_t addr, uint32_t val)
{
	int ack;

	ack = swd_write_ap(swd, AP_CSW, CSW_VALUE | CSW_SIZE32);
	if (ack != SWD_ACK_OK)
		return ack;

	return swd_write_data(swd, addr, val);
}

uint8_t swd_write_byte(struct swd_ctx *swd, uint32_t addr, uint8_t val)
{
	uint32_t tmp;
	int ack;

	ack = swd_write_ap(swd, AP_CSW, CSW_VALUE | CSW_SIZE8);
	if (ack != SWD_ACK_OK)
		return ack;

	tmp = val << ((addr & 0x03) * 8);
	return swd_write_data(swd, addr, tmp);
}

static int swd_read_mem(struct swd_ctx *swd, uint32_t addr, uint8_t *data, uint32_t size)
{
	int ack;

	/* Read bytes until word aligned */
	while ((size > 0) && ((ptrdiff_t)addr & 0x3)) {
		ack = swd_read_byte(swd, addr, data);
		if (ack != SWD_ACK_OK)
			return ack;

		addr++;
		data++;
		size--;
	}

	/* Read word aligned blocks */
	while (size >= 4) {
		// Limit to auto increment page size
		uint32_t n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (addr & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
		if (size < n) {
			n = size & ~3U; // Only count complete words remaining
		}

		ack = swd_read_block(swd, addr, (uint32_t *)data, n / 4);
		if (ack != SWD_ACK_OK)
			return ack;

		addr += n;
		data += n;
		size -= n;
	}

	/* Read remaining bytes */
	while (size > 0) {
		ack = swd_read_byte(swd, addr, data);
		if (ack != SWD_ACK_OK)
			return ack;

		addr++;
		data++;
		size--;
	}

	return SWD_ACK_OK;
}

static int swd_write_mem(struct swd_ctx *swd, uint32_t addr, uint8_t *data, uint32_t size)
{
	int ack;

	/* Write bytes until word aligned */
	while ((size > 0) && ((ptrdiff_t)addr & 0x3)) {
		ack = swd_write_byte(swd, addr, *data);
		if (ack != SWD_ACK_OK)
			return ack;

		addr++;
		data++;
		size--;
	}

	/* Write word aligned blocks */
	while (size >= 4) {
		// Limit to auto increment page size
		uint32_t n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (addr & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
		if (size < n) {
			n = size & ~3U; // Only count complete words remaining
		}

		ack = swd_write_block(swd, addr, (uint32_t *)data, n / 4);
		if (ack != SWD_ACK_OK)
			return ack;

		addr += n;
		data += n;
		size -= n;
	}

	/* Write remaining bytes */
	while (size > 0) {
		ack = swd_write_byte(swd, addr, *data);
		if (ack != SWD_ACK_OK)
			return ack;

		addr++;
		data++;
		size--;
	}

	return SWD_ACK_OK;
}

static int swd_read_core_register(struct swd_ctx *swd, uint32_t n, uint32_t *val)
{
	int timeout = 100;
	int ack;

	ack = swd_write_word(swd, DBG_CRSR, n);
	if (ack != SWD_ACK_OK)
		return ack;

	/* wait for S_REGRDY */
	do {
		uint32_t tmp;

		ack = swd_read_word(swd, DBG_HCSR, &tmp);
		if (ack != SWD_ACK_OK)
			return ack;

		if (tmp & S_REGRDY)
			return swd_read_word(swd, DBG_CRDR, val);

		mdelay(1);
	} while (--timeout);

	return SWD_ACK_WAIT;
}

static int swd_write_core_register(struct swd_ctx *swd, uint32_t n, uint32_t val)
{
	int timeout = 100;
	int ack;

	ack = swd_write_word(swd, DBG_CRDR, val);
	if (ack != SWD_ACK_OK)
		return ack;

	ack = swd_write_word(swd, DBG_CRSR, n | REGWnR);
	if (ack != SWD_ACK_OK)
		return ack;

	/* wait for S_REGRDY */
	do {
		uint32_t tmp;

		ack = swd_read_word(swd, DBG_HCSR, &tmp);
		if (ack != SWD_ACK_OK)
			return ack;

		if (tmp & S_REGRDY)
			return SWD_ACK_OK;

		mdelay(1);
	} while (--timeout);

	return SWD_ACK_WAIT;
}

static int swd_pinctrl_configure(struct swd_ctx *swd, bool active)
{
	struct pinctrl_state *state;
	int ret;

	state = pinctrl_lookup_state(swd->pinctrl, active ? "active" : "default");
	if (IS_ERR(state)) {
		dev_err(swd->dev, "cannot get pinctrl %s state\n", active ? "active" : "default");
		return PTR_ERR(state);
	}

	ret = pinctrl_select_state(swd->pinctrl, state);
	if (ret) {
		dev_err(swd->dev, "cannot set pinctrl state\n");
		return ret;
	}

	return 0;
}

static inline int swd_clear_errors(struct swd_ctx *swd)
{
	return swd_write_dp(swd, DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);
}

static int swd_init_debug(struct swd_ctx *swd)
{
	int ack;
	int timeout = 100;

	// init dap state with fake values
	swd->dp_select = 0xffffffff;
	swd->ap_csw = 0xffffffff;

#if 0
	/*
	 * this function can do several stuff before really initing the debug
	 */
	if (swd->pdata->target_before_init_debug) {
		swd->pdata->target_before_init_debug();
	}
#endif

	ack = jtag_to_swd(swd);
	if (ack != SWD_ACK_OK)
		return ack;

	ack = swd_clear_errors(swd);
	if (ack != SWD_ACK_OK)
		return ack;

	ack = swd_write_dp(swd, DP_SELECT, 0);
	if (ack != SWD_ACK_OK)
		return ack;

	// Power up
	ack = swd_write_dp(swd, DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ);
	if (ack != SWD_ACK_OK)
		return ack;

	do {
		uint32_t tmp;

		ack = swd_read_dp(swd, DP_CTRL_STAT, &tmp);
		if (ack != SWD_ACK_OK)
			return ack;

		if ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK)) {
			// Break from loop if powerup is complete
			break;
		}

		mdelay(1);
	} while (--timeout);

	if (0 == timeout)
		return SWD_ACK_WAIT;

	ack = swd_write_dp(swd, DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE);
	if (ack != SWD_ACK_OK)
		return ack;

#if 0
	/*
	 * some target can enter in a lock state, this function can unlock these targets
	 */
	if (swd->pdata->target_unlock_sequence) {
		swd->pdata->target_unlock_sequence();
	}
#endif

	return swd_write_dp(swd, DP_SELECT, 0);
}

static inline int swd_ack_to_errno(int ack)
{
	switch (ack) {
	case SWD_ACK_OK:
		return 0;
	case SWD_ACK_FAULT:
		return -EFAULT;
	case SWD_ACK_WAIT:
		return -EBUSY;
	case SWD_ACK_PARITY:
	default:
		return -EIO;
	}
}

static int swd_connect(struct swd_ctx *swd)
{
	int ret;
	int retries = 5;

	if (swd->connected)
		return 0;

	swd_pinctrl_configure(swd, true);

	for (;;) {
		ret = swd_init_debug(swd);
		if (ret == SWD_ACK_OK) {
			swd->connected = true;
			return 0;
		}

		if (retries-- == 0) {
			dev_dbg(swd->dev, "%s: timedout\n", __func__);
			break;
		}

		// do an abort on stale target, then reset the device
		swd_write_dp(swd, DP_ABORT, DAPABORT);

		oppo_sensorhub_reset(swd->dev);
	}

	swd_to_jtag(swd);
	swd_pinctrl_configure(swd, false);

	return -ETIMEDOUT;
}

static void swd_disconnect(struct swd_ctx *swd)
{
	if (!swd->connected)
		return;

	swd_write_word(swd, DBG_HCSR, DBGKEY);
	swd_write_dp(swd, DP_CTRL_STAT, 0);

	swd_to_jtag(swd);
	swd_pinctrl_configure(swd, false);

	swd->halted = false;
	swd->connected = false;
}

static int swd_reset_halt(struct swd_ctx *swd)
{
	int8_t ap_retries = 5;
	uint32_t tmp;
	int ret;

	if (swd->soft_reset) {
		// Enable debug and halt the core (DHCSR <- 0xA05F0003)
		for (;;) {
			ret = swd_write_word(swd, DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT);
			if (ret == SWD_ACK_OK)
				break;

			if (ap_retries-- == 0)
				return -ETIMEDOUT;

			// Target is in invalid state?
			oppo_sensorhub_reset(swd->dev);
		}

		// Wait until core is halted
		do {
			ret = swd_read_word(swd, DBG_HCSR, &tmp);
			if (ret != SWD_ACK_OK)
				return swd_ack_to_errno(ret);
			mdelay(1);
		} while ((tmp & S_HALT) == 0);

		tmp = 0;
		if (swd->idcode == AMBIQ_APOLLO3_ID) {
			ret = swd_read_word(swd, REG_APOLLO3_BOOTLDR, &tmp);
			if (ret != SWD_ACK_OK)
				return swd_ack_to_errno(ret);
		}

		if ((tmp & 0x0C000000) == 0x04000000) {
			ret = swd_read_word(swd, REG_APOLLO3_SCRATCH0, &tmp);
			if (ret != SWD_ACK_OK)
				return swd_ack_to_errno(ret);

			ret = swd_write_word(swd, REG_APOLLO3_SCRATCH0, tmp | 1);
			if (ret != SWD_ACK_OK)
				return swd_ack_to_errno(ret);
		} else {
			// Enable halt on reset
			ret = swd_write_word(swd, DBG_EMCR, VC_CORERESET);
			if (ret != SWD_ACK_OK)
				return swd_ack_to_errno(ret);
		}

		// Perform a soft reset
		ret = swd_read_word(swd, NVIC_AIRCR, &tmp);
		if (ret != SWD_ACK_OK)
			return swd_ack_to_errno(ret);

		ret = swd_write_word(swd, NVIC_AIRCR, VECTKEY | (tmp & SCB_AIRCR_PRIGROUP_Msk) | swd->soft_reset);
		if (ret != SWD_ACK_OK)
			return swd_ack_to_errno(ret);
	} else {
		if (swd->connect_under_reset)
			oppo_sensorhub_assert_reset(swd->dev);

		// Enable debug
		for (;;) {
			ret = swd_write_word(swd, DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT);
			if (ret == SWD_ACK_OK)
				break;

			if (ap_retries-- == 0) {
				if (swd->connect_under_reset)
					oppo_sensorhub_deassert_reset(swd->dev);
				return -ETIMEDOUT;
			}

			// Target is in invalid state?
			if (swd->connect_under_reset) {
				oppo_sensorhub_deassert_reset(swd->dev);
				oppo_sensorhub_assert_reset(swd->dev);
			} else {
				oppo_sensorhub_assert_reset(swd->dev);
				oppo_sensorhub_deassert_reset(swd->dev);
			}
		}

		// Enable halt on reset
		ret = swd_write_word(swd, DBG_EMCR, VC_CORERESET);
		if (ret != SWD_ACK_OK)
			return swd_ack_to_errno(ret);

		if (!swd->connect_under_reset)
			oppo_sensorhub_assert_reset(swd->dev);

		oppo_sensorhub_deassert_reset(swd->dev);
	}

	msleep(100);

	// Wait until core is halted
	do {
		ret = swd_read_word(swd, DBG_HCSR, &tmp);
		if (ret != SWD_ACK_OK)
			return swd_ack_to_errno(ret);
		mdelay(1);
	} while ((tmp & S_HALT) == 0);

	// Disable halt on reset
	ret = swd_write_word(swd, DBG_EMCR, 0);
	if (ret != SWD_ACK_OK)
		return swd_ack_to_errno(ret);

	swd->halted = true;
	return 0;
}

static int swd_halt(struct swd_ctx *swd)
{
	uint32_t tmp;
	int ret;

	if (swd->halted)
		return 0;

	// Enable debug and halt the core (DHCSR <- 0xA05F0003)
	ret = swd_write_word(swd, DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT);
	if (ret != SWD_ACK_OK)
		return swd_ack_to_errno(ret);

	// Wait until core is halted
	do {
		ret = swd_read_word(swd, DBG_HCSR, &tmp);
		if (ret != SWD_ACK_OK)
			return swd_ack_to_errno(ret);
		mdelay(1);
	} while ((tmp & S_HALT) == 0);

	swd->halted = true;
	return 0;
}

static int swd_run(struct swd_ctx *swd, bool debug)
{
	int ret;

	if (!swd->halted)
		return 0;

	ret = swd_write_word(swd, DBG_HCSR, DBGKEY | (debug ? C_DEBUGEN : 0));
	if (ret != SWD_ACK_OK)
		return swd_ack_to_errno(ret);

	swd->halted = false;
	return 0;
}

static void swd_reset_run(struct swd_ctx *swd)
{
	if (swd->connected)
		swd_disconnect(swd);

	oppo_sensorhub_reset(swd->dev);
}

static int swd_open(struct inode *inode, struct file *file)
{
	struct swd_ctx *swd = container_of(file->private_data, struct swd_ctx, misc);

	if (atomic_inc_return(&swd->opened) != 1) {
		atomic_dec(&swd->opened);
		return -EBUSY;
	}

	swd->io_buf = (void *) __get_free_page(GFP_KERNEL);
	if (swd->io_buf == NULL) {
		atomic_dec(&swd->opened);
		return -ENOMEM;
	}

	return 0;
}

static int swd_release(struct inode *inode, struct file *file)
{
	struct swd_ctx *swd = container_of(file->private_data, struct swd_ctx, misc);

	mutex_lock(&swd->lock);
	if (swd->connected) {
		if (swd->halted)
			swd_run(swd, false);
		swd_disconnect(swd);
	}
	mutex_unlock(&swd->lock);

	free_page((unsigned long) swd->io_buf);
	swd->io_buf = NULL;

	atomic_dec(&swd->opened);

	return 0;
}

static ssize_t swd_read(struct file *file, char __user *buf,
			 size_t n_bytes, loff_t *pos)
{
	struct swd_ctx *swd = container_of(file->private_data, struct swd_ctx, misc);
	size_t rd_bytes = 0;
	size_t step = PAGE_SIZE;
	int ret;

	if (!swd->connected)
		return -EINVAL;

	while (n_bytes > 0) {
		if (step > n_bytes)
			step = n_bytes;

		mutex_lock(&swd->lock);
		ret = swd_read_mem(swd, *pos, swd->io_buf, step);
		if (ret == SWD_ACK_OK)
			ret = copy_to_user(buf, swd->io_buf, step);
		else
			ret = swd_ack_to_errno(ret);
		mutex_unlock(&swd->lock);

		if (ret)
			break;

		buf += step;
		*pos += step;
		rd_bytes += step;
		n_bytes -= step;

		if (signal_pending(current))
			break;
	}

	return rd_bytes ? rd_bytes : ret;
}

static ssize_t swd_write(struct file *file, const char __user *buf,
			 size_t n_bytes, loff_t *pos)
{
	struct swd_ctx *swd = container_of(file->private_data, struct swd_ctx, misc);
	size_t wr_bytes = 0;
	size_t step = PAGE_SIZE;
	int ret;

	if (!swd->connected)
		return -EINVAL;

	while (n_bytes > 0) {
		if (step > n_bytes)
			step = n_bytes;

		mutex_lock(&swd->lock);
		ret = copy_from_user(swd->io_buf, buf, step);
		if (!ret) {
			ret = swd_write_mem(swd, *pos, swd->io_buf, step);
			ret = swd_ack_to_errno(ret);
		}
		mutex_unlock(&swd->lock);

		if (ret)
			break;

		buf += step;
		*pos += step;
		wr_bytes += step;
		n_bytes -= step;

		if (signal_pending(current))
			break;
	}

	return wr_bytes ? wr_bytes : ret;
}

static long swd_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct swd_ctx *swd = container_of(file->private_data, struct swd_ctx, misc);
	unsigned int u_args[2];
	int ret = 0;

	switch (cmd) {
	case SWD_IOCTL_CONNECT:
		mutex_lock(&swd->lock);
		ret = swd_connect(swd);
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_DISCONNECT:
		mutex_lock(&swd->lock);
		swd_disconnect(swd);
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_RESET_HALT:
		mutex_lock(&swd->lock);
		if (swd->connected)
			ret = swd_reset_halt(swd);
		else
			ret = -EINVAL;
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_RUN:
		mutex_lock(&swd->lock);
		if (swd->connected)
			ret = swd_run(swd, arg);
		else
			ret = -EINVAL;
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_HALT:
		mutex_lock(&swd->lock);
		if (swd->connected)
			ret = swd_halt(swd);
		else
			ret = -EINVAL;
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_RESET_RUN:
		mutex_lock(&swd->lock);
		swd_reset_run(swd);
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_WR_REG:
		ret = copy_from_user(u_args, (void __user *)arg, sizeof(u_args));
		if (ret)
			return ret;

		mutex_lock(&swd->lock);
		if (swd->connected && swd->halted) {
			ret = swd_write_core_register(swd, u_args[0], u_args[1]);
			ret = swd_ack_to_errno(ret);
		} else
			ret = -EINVAL;
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_RD_REG:
		ret = __get_user(u_args[0], (unsigned int *) arg);
		if (ret)
			return ret;

		mutex_lock(&swd->lock);
		if (swd->connected && swd->halted) {
			ret = swd_read_core_register(swd, u_args[0], &u_args[1]);
			ret = swd_ack_to_errno(ret);
		} else
			ret = -EINVAL;
		mutex_unlock(&swd->lock);

		if (ret)
			return ret;

		ret = __put_user(u_args[1], (unsigned int *)(arg + sizeof(unsigned int)));
		break;

	case SWD_IOCTL_WR_MEM:
		ret = copy_from_user(u_args, (void __user *)arg, sizeof(u_args));
		if (ret)
			return ret;

		mutex_lock(&swd->lock);
		if (swd->connected) {
			ret = swd_write_word(swd, u_args[0], u_args[1]);
			ret = swd_ack_to_errno(ret);
		} else
			ret = -EINVAL;
		mutex_unlock(&swd->lock);
		break;

	case SWD_IOCTL_RD_MEM:
		ret = __get_user(u_args[0], (unsigned int *) arg);
		if (ret)
			return ret;

		mutex_lock(&swd->lock);
		if (swd->connected) {
			ret = swd_read_word(swd, u_args[0], &u_args[1]);
			ret = swd_ack_to_errno(ret);
		} else
			ret = -EINVAL;
		mutex_unlock(&swd->lock);

		if (ret)
			return ret;

		ret = __put_user(u_args[1], (unsigned int *)(arg + sizeof(unsigned int)));
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static const struct file_operations swd_fops = {
	.owner = THIS_MODULE,
	.open = swd_open,
	.release = swd_release,
	.read = swd_read,
	.write = swd_write,
	.llseek = default_llseek,
	.unlocked_ioctl = swd_ioctl,
};

static ssize_t swd_show_turnaround_cycles(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", swd->turnaround);
}

static ssize_t swd_set_turnaround_cycles(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;
	if (val <= 0)
		return -EINVAL;

	mutex_lock(&swd->timing_lock);
	swd->turnaround = val;
	mutex_unlock(&swd->timing_lock);

	return count;
}

static ssize_t swd_show_idle_cycles(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", swd->idle_cycles);
}

static ssize_t swd_set_idle_cycles(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->timing_lock);
	swd->idle_cycles = val;
	mutex_unlock(&swd->timing_lock);

	return count;
}

static ssize_t swd_show_data_phase(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", swd->data_phase ? 1 : 0);
}

static ssize_t swd_set_data_phase(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->timing_lock);
	swd->data_phase = val ? true : false;
	mutex_unlock(&swd->timing_lock);

	return count;
}

static ssize_t swd_show_ap_sel(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%08x\n", swd->apsel);
}

static ssize_t swd_set_ap_sel(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 16, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	swd->apsel = val;
	mutex_unlock(&swd->lock);

	return count;
}

static ssize_t swd_show_soft_reset(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%08x\n", swd->soft_reset);
}

static ssize_t swd_set_soft_reset(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	swd->soft_reset = val;
	mutex_unlock(&swd->lock);

	return count;
}

static ssize_t swd_show_connect_under_reset(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", swd->connect_under_reset ? 1 : 0);
}

static ssize_t swd_set_connect_under_reset(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	swd->connect_under_reset = val ? true : false;
	mutex_unlock(&swd->lock);

	return count;
}

#ifdef SWD_DEBUG
static ssize_t swd_show_connect_state(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", swd->connected ? "connected" : "disconnected");
}

static ssize_t swd_store_connect_state(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;
	int ret = 0;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	if (val)
		ret = swd_connect(swd);
	else
		swd_disconnect(swd);
	mutex_unlock(&swd->lock);

	return ret ? ret : count;
}

static ssize_t swd_store_reset_halt(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;
	int ret;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	if (val) {
		if (swd->connected)
			ret = swd_reset_halt(swd);
		else
			ret = -EINVAL;
	}
	mutex_unlock(&swd->lock);

	return ret ? ret : count;
}

static ssize_t swd_show_halt(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", swd->halted ? "halted" : "running");
}

static ssize_t swd_store_halt(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;
	int ret = -EINVAL;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	if (swd->connected) {
		if (val)
			ret = swd_halt(swd);
		else
			ret = swd_run(swd, false);
	}
	mutex_unlock(&swd->lock);

	return ret ? ret : count;
}

static ssize_t swd_show_memory_addr(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%08x\n", swd->sysfs_mem_addr);
}

static ssize_t swd_store_memory_addr(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	swd->sysfs_mem_addr = val;
	mutex_unlock(&swd->lock);

	return count;
}

static ssize_t swd_show_memory(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	int ack = SWD_ACK_FAULT;
	uint32_t addr;
	uint32_t val;

	mutex_lock(&swd->lock);
	if (swd->connected) {
		addr = swd->sysfs_mem_addr;
		ack = swd_read_word(swd, addr, &val);
	}
	mutex_unlock(&swd->lock);

	if (ack != SWD_ACK_OK)
		return swd_ack_to_errno(ack);

	return sprintf(buf, "[0x%08x] = 0x%08x\n", addr, val);
}

static ssize_t swd_store_memory(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	int ack = SWD_ACK_FAULT;
	unsigned long val;

	if (kstrtoul(buf, 16, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	if (swd->connected)
		ack = swd_write_word(swd, swd->sysfs_mem_addr, val);
	mutex_unlock(&swd->lock);

	if (ack != SWD_ACK_OK)
		return swd_ack_to_errno(ack);

	return count;
}

static ssize_t swd_show_register_nr(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", swd->sysfs_reg_nr);
}

static ssize_t swd_store_register_nr(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	swd->sysfs_reg_nr = val;
	mutex_unlock(&swd->lock);

	return count;
}

static ssize_t swd_show_register(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	unsigned int reg_nr;
	int ack = SWD_ACK_FAULT;
	uint32_t val;

	mutex_lock(&swd->lock);
	if (swd->connected && swd->halted) {
		reg_nr = swd->sysfs_reg_nr;
		ack = swd_read_core_register(swd, reg_nr, &val);
	}
	mutex_unlock(&swd->lock);

	if (ack != SWD_ACK_OK)
		return swd_ack_to_errno(ack);

	return sprintf(buf, "R%d = 0x%08x\n", reg_nr, val);
}

static ssize_t swd_store_register(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct swd_ctx *swd = dev_get_drvdata(dev);
	int ack = SWD_ACK_FAULT;
	unsigned long val;

	if (kstrtoul(buf, 16, &val) != 0)
		return -EINVAL;

	mutex_lock(&swd->lock);
	if (swd->connected && swd->halted)
		ack = swd_write_core_register(swd, swd->sysfs_reg_nr, val);
	mutex_unlock(&swd->lock);

	if (ack != SWD_ACK_OK)
		return swd_ack_to_errno(ack);

	return count;
}
#endif

static const DEVICE_ATTR(turnaround_cycles, 0664, swd_show_turnaround_cycles, swd_set_turnaround_cycles);
static const DEVICE_ATTR(idle_cycles, 0664, swd_show_idle_cycles, swd_set_idle_cycles);
static const DEVICE_ATTR(force_data_phase, 0664, swd_show_data_phase, swd_set_data_phase);
static const DEVICE_ATTR(ap_sel, 0664, swd_show_ap_sel, swd_set_ap_sel);
static const DEVICE_ATTR(soft_reset, 0664, swd_show_soft_reset, swd_set_soft_reset);
static const DEVICE_ATTR(connect_under_reset, 0664, swd_show_connect_under_reset, swd_set_connect_under_reset);
#ifdef SWD_DEBUG
static const DEVICE_ATTR(connect, 0664, swd_show_connect_state, swd_store_connect_state);
static const DEVICE_ATTR(reset_halt, 0664, NULL, swd_store_reset_halt);
static const DEVICE_ATTR(halt, 0664, swd_show_halt, swd_store_halt);
static const DEVICE_ATTR(memory_addr, 0664, swd_show_memory_addr, swd_store_memory_addr);
static const DEVICE_ATTR(memory, 0664, swd_show_memory, swd_store_memory);
static const DEVICE_ATTR(register_nr, 0664, swd_show_register_nr, swd_store_register_nr);
static const DEVICE_ATTR(register, 0664, swd_show_register, swd_store_register);
#endif

static const struct attribute *swd_attributes[] = {
	&dev_attr_turnaround_cycles.attr,
	&dev_attr_idle_cycles.attr,
	&dev_attr_force_data_phase.attr,
	&dev_attr_ap_sel.attr,
	&dev_attr_soft_reset.attr,
	&dev_attr_connect_under_reset.attr,
#ifdef SWD_DEBUG
	&dev_attr_connect.attr,
	&dev_attr_reset_halt.attr,
	&dev_attr_halt.attr,
	&dev_attr_memory_addr.attr,
	&dev_attr_memory.attr,
	&dev_attr_register_nr.attr,
	&dev_attr_register.attr,
#endif
	NULL
};

static const struct attribute_group swd_attr_group = {
	.attrs	= (struct attribute **)swd_attributes,
};

static int swd_reset_notify_call(struct notifier_block *nb, unsigned long event, void *data)
{
	struct swd_ctx *swd = container_of(nb, struct swd_ctx, rst_nb);

	if (data == (void *) swd->dev) {
		/* reset by myself */
		return 0;
	}

	if (event == SNSHUB_PRE_RESET) {
		mutex_lock(&swd->lock);
		if (swd->connected)
			swd_disconnect(swd);
		mutex_unlock(&swd->lock);
	} else if (event != SNSHUB_POST_RESET)
		return -EINVAL;

	return kobject_uevent(&swd->misc.this_device->kobj,
			event == SNSHUB_PRE_RESET ? KOBJ_OFFLINE : KOBJ_ONLINE);
}

static struct dap_swd_platform_data *swd_parse_dt(struct device *dev)
{
	struct dap_swd_platform_data *pdata;
	int rc = 0;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	/* swd-clk gpio */
	rc = of_get_named_gpio_flags(dev->of_node,
			"swd_clk-gpio", 0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read swd_clk gpio\n");
		return NULL;
	}
	pdata->gpio_clk = rc;

	/* swd-data gpio */
	rc = of_get_named_gpio_flags(dev->of_node,
			"swd_data-gpio", 0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read swd_data gpio\n");
		return NULL;
	}
	pdata->gpio_data = rc;

	return pdata;
}

static int swd_probe(struct platform_device *pdev)
{
	struct dap_swd_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct swd_ctx *swd;
	int ret = 0;

	if (!pdata) {
		pdata = swd_parse_dt(&pdev->dev);
		if (!pdata) {
			dev_err(&pdev->dev, "Platform data not found!\n");
			return -ENODEV;
		}
	}

	swd = devm_kzalloc(&pdev->dev, sizeof(struct swd_ctx), GFP_KERNEL);
	if (!swd)
		return -ENOMEM;

	swd->dev = &pdev->dev;
#ifdef CONFIG_PINCTRL_MSM
	msm_gpio_get_reg(gpio_to_desc(pdata->gpio_clk), &swd->gpio_clk_reg);
	msm_gpio_get_reg(gpio_to_desc(pdata->gpio_data), &swd->gpio_data_reg);
#else
	swd->gpio_clk = gpio_to_desc(pdata->gpio_clk);
	swd->gpio_data = gpio_to_desc(pdata->gpio_data);
#endif
	atomic_set(&swd->opened, 0);
	mutex_init(&swd->lock);
	mutex_init(&swd->timing_lock);

	if (!dev_get_platdata(&pdev->dev)) {
		kfree(pdata);
		pdata = NULL;
	}

	swd->turnaround = 1;
	swd->idle_cycles = 0;
	swd->data_phase = false;

	swd->apsel = 0;
	swd->soft_reset = SYSRESETREQ;
	swd->connect_under_reset = false;

	/* Get pinctrl if target uses pinctrl */
	swd->pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(swd->pinctrl)) {
		if (PTR_ERR(swd->pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(&pdev->dev, "cannot get pinctrl\n");
		return PTR_ERR(swd->pinctrl);
	}

	swd->rst_nb.notifier_call = swd_reset_notify_call;
	ret = oppo_sensorhub_reset_notifier_register(&pdev->dev, &swd->rst_nb);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot register notifier\n");
		return ret;
	}

	swd->misc.minor = MISC_DYNAMIC_MINOR;
	swd->misc.name = "swd";
	swd->misc.fops = &swd_fops;

	/* Register for sensor ioctl */
	ret = misc_register(&swd->misc);
	if (ret) {
		oppo_sensorhub_reset_notifier_unregister(&pdev->dev, &swd->rst_nb);

		dev_err(&pdev->dev, "Unable to register swd\n");
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &swd_attr_group);
	if (ret) {
		oppo_sensorhub_reset_notifier_unregister(&pdev->dev, &swd->rst_nb);
		misc_deregister(&swd->misc);

		dev_err(&pdev->dev,
			"Failed to create attribute group: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, swd);
	dev_info(&pdev->dev, "SWD register successful\n");


	return 0;
}

static int swd_remove(struct platform_device *pdev)
{
	struct swd_ctx *swd = platform_get_drvdata(pdev);

	oppo_sensorhub_reset_notifier_unregister(&pdev->dev, &swd->rst_nb);

	swd_pinctrl_configure(swd, false);

	sysfs_remove_group(&pdev->dev.kobj, &swd_attr_group);

	misc_deregister(&swd->misc);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id swd_dt_ids[] = {
	{ .compatible = "oppo,swd", },
	{ },
};
#endif

static struct platform_driver swd_driver = {
	.driver = {
		.name = "swd",
		.of_match_table = of_match_ptr(swd_dt_ids),
	},
	.probe = swd_probe,
	.remove = swd_remove,
};

static int __init swd_init(void)
{
	return platform_driver_register(&swd_driver);
}

static void __exit swd_exit(void)
{
	platform_driver_unregister(&swd_driver);
}

late_initcall(swd_init);
module_exit(swd_exit);

MODULE_AUTHOR("Zeng Zhaoxiu <zengzhaoxiu@oppo.com>");
MODULE_DESCRIPTION("OPPO-SWD driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: OPPO-SWD");
