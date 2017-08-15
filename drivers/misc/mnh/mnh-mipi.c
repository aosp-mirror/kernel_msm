/*
*
* MNH MIPI APIs
* Copyright (c) 2016, Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
*/
#define DEBUG
#include <linux/delay.h>

#include "mnh-hwio.h"
#include "mnh-hwio-mipi-rx.h"
#include "mnh-hwio-mipi-tx.h"
#include "mnh-hwio-mipi-top.h"
#include "mnh-hwio-bases.h"
#include "mnh-mipi.h"

#define REF_FREQ_KHZ    25000

#define TOP_IN(reg)             HW_IN(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, reg)
#define TOP_MASK(reg, fld)      HWIO_MIPI_TOP_##reg##_##fld##_FLDMASK
#define TOP_OUTf(reg, fld, val) \
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, reg, fld, val)
#define TOP_OUTf_LOCAL(reg, fld, val, curr) \
			((curr & ~HWIO_MIPI_TOP_##reg##_##fld##_FLDMASK) | \
			((val << HWIO_MIPI_TOP_##reg##_##fld##_FLDSHFT) & \
			HWIO_MIPI_TOP_##reg##_##fld##_FLDMASK))

#define TX_IN(reg)		HW_IN(baddr,   MIPI_TX, reg)
#define TX_MASK(reg, fld)	HWIO_MIPI_TX_##reg##_##fld##_FLDMASK

#define RX_IN(reg)		HW_IN(baddr,   MIPI_RX, reg)
#define RX_MASK(reg, fld)	HWIO_MIPI_RX_##reg##_##fld##_FLDMASK

struct mipi_rate_config {
	uint32_t rate;
	uint32_t freq_range_code;
	uint32_t osc_freq_code;
};

struct mnh_mipi_vco_range_cntrl {
	uint32_t freq_kHz;
	uint8_t cntrl;
};

/* MIPI Timing override registers */
struct mipi_dev_ovr_cfg {
	uint32_t rate;
	/* Timing override registers */
	uint32_t reg_0x5A;
	uint32_t reg_0x5B;
	uint32_t reg_0x5C;
	uint32_t reg_0x5D;
	uint32_t reg_0x5E;
	uint32_t reg_0x5F;
	uint32_t reg_0x61;
	uint32_t reg_0x62;
	uint32_t reg_0x63;
	uint32_t reg_0x64;
	uint32_t reg_0x65;
	/* PHY_STOP_WAIT_TIME */
	uint32_t wait_time; /* Table A-4 High-speed exit */
	bool use_ovrd;
};

/* must be in ascending order */
static struct mipi_rate_config mipi_rate_configs[] = {
	/*  rate   freq_range_code   osc_freq_code  */
	{     80,        0x00,           0x1b6},
	{     90,        0x10,           0x1b6},
	{    100,        0x20,           0x1b6},
	{    110,        0x30,           0x1b6},
	{    120,        0x01,           0x1b6},
	{    130,        0x11,           0x1b6},
	{    140,        0x21,           0x1b6},
	{    150,        0x31,           0x1b6},
	{    160,        0x02,           0x1b6},
	{    170,        0x12,           0x1b6},
	{    180,        0x22,           0x1b6},
	{    190,        0x32,           0x1b6},
	{    205,        0x03,           0x1b6},
	{    220,        0x13,           0x1b6},
	{    235,        0x23,           0x1b6},
	{    250,        0x33,           0x1b6},
	{    275,        0x04,           0x1b6},
	{    300,        0x14,           0x1b6},
	{    325,        0x25,           0x1b6},
	{    350,        0x35,           0x1b6},
	{    400,        0x05,           0x1b6},
	{    450,        0x16,           0x1b6},
	{    500,        0x26,           0x1b6},
	{    550,        0x37,           0x1b6},
	{    600,        0x07,           0x1b6},
	{    650,        0x18,           0x1b6},
	{    700,        0x28,           0x1b6},
	{    750,        0x39,           0x1b6},
	{    800,        0x09,           0x1b6},
	{    850,        0x19,           0x1b6},
	{    900,        0x29,           0x1b6},
	{    950,        0x3a,           0x1b6},
	{   1000,        0x0a,           0x1b6},
	{   1050,        0x1a,           0x1b6},
	{   1100,        0x2a,           0x1b6},
	{   1150,        0x3b,           0x1b6},
	{   1200,        0x0b,           0x1b6},
	{   1250,        0x1b,           0x1b6},
	{   1300,        0x2b,           0x1b6},
	{   1350,        0x3c,           0x1b6},
	{   1400,        0x0c,           0x1b6},
	{   1450,        0x1c,           0x1b6},
	{   1500,        0x2c,           0x1b6},
	{   1550,        0x3d,           0x10f},
	{   1600,        0x0d,           0x118},
	{   1650,        0x1d,           0x121},
	{   1700,        0x2e,           0x12a},
	{   1750,        0x3e,           0x132},
	{   1800,        0x0e,           0x13b},
	{   1850,        0x1e,           0x144},
	{   1900,        0x2f,           0x14d},
	{   1950,        0x3f,           0x155},
	{   2000,        0x0f,           0x15e},
	{   2050,        0x40,           0x167},
	{   2100,        0x41,           0x170},
	{   2150,        0x42,           0x178},
	{   2200,        0x43,           0x181},
	{   2250,        0x44,           0x18a},
	{   2300,        0x45,           0x193},
	{   2350,        0x46,           0x19b},
	{   2400,        0x47,           0x1a4},
	{   2450,        0x48,           0x1ad},
	{   2500,        0x49,           0x1b6},
};

/* must be in ascending order */
static struct mnh_mipi_vco_range_cntrl vco_range_cntrls[] = {
	/* freq_kHz   cntrl   */
	{    40000,    0x3f   },
	{    52500,    0x39   },
	{    80000,    0x2f   },
	{   105000,    0x29   },
	{   160000,    0x1f   },
	{   210000,    0x19   },
	{   320000,    0x0f   },
	{   420000,    0x09   },
	{   630000,    0x03   },
	{  1100000,    0x01   },
	{  1150000,    0x01   },
};

static struct mipi_dev_ovr_cfg mipi_dev_ovr_cfgs[] = {
	/* rate 0x05A 0x05B 0x05C 0x05D 0x05E 0x05F 0x061 0x062 0x063 0x064 0x065 wait_time */
	{  340, 0x57, 0x43, 0xC2, 0x44, 0x42, 0x8A, 0x43, 0xC3, 0x44, 0x43, 0x83, 0x14, true},
	{  360, 0x57, 0x43, 0xC2, 0x44, 0x42, 0x8A, 0x43, 0xC3, 0x44, 0x43, 0x83, 0x14, false},
	{  446, 0x57, 0x45, 0xC3, 0x9D, 0x43, 0x8E, 0x45, 0xC3, 0x9D, 0x44, 0x84, 0x17, true},
	{  488, 0x58, 0x45, 0xC3, 0x03, 0x43, 0x90, 0x45, 0xC3, 0x03, 0x44, 0x85, 0x18, true},
	{  550, 0x4C, 0x48, 0xC5, 0x03, 0x44, 0x91, 0x47, 0xC6, 0x03, 0x43, 0x85, 0x1A, true},
	{  648, 0x4E, 0x48, 0xC5, 0x05, 0x47, 0x94, 0x48, 0xC5, 0x05, 0x47, 0x85, 0x1C, true},
	{  685, 0x4C, 0x48, 0xC5, 0x03, 0x44, 0x91, 0x47, 0xC6, 0x03, 0x43, 0x85, 0x1D, true},
	{  720, 0x4E, 0x49, 0xC5, 0x05, 0x48, 0x96, 0x49, 0xC6, 0x05, 0x48, 0x86, 0x1D, true},
	{  984, 0x50, 0x4D, 0xC7, 0x05, 0x48, 0xA0, 0x4D, 0xC8, 0x05, 0x4A, 0x8A, 0x26, true},
	{ 1104, 0x5D, 0x4F, 0xC6, 0x09, 0x49, 0xA7, 0x4F, 0xC9, 0x07, 0x4B, 0x8B, 0x27, true},
	{ 1200, 0x53, 0x51, 0xCA, 0x09, 0x4E, 0xA7, 0x50, 0xCC, 0x07, 0x4B, 0x8D, 0x2B, true},
	{ 1296, 0x53, 0x51, 0xCA, 0x09, 0x4E, 0xA7, 0x50, 0xCC, 0x07, 0x4C, 0x8D, 0x2E, true},
	{ 1300, 0x53, 0x51, 0xCA, 0x09, 0x4E, 0xA7, 0x50, 0xCC, 0x07, 0x4C, 0x8D, 0x2E, true},
	{ 1368, 0x53, 0x51, 0xCA, 0x09, 0x4E, 0xA7, 0x50, 0xCC, 0x08, 0x4C, 0x8D, 0x2F, true},
	{ 1776, 0x58, 0x59, 0xCD, 0x0B, 0x55, 0xC0, 0x50, 0xCE, 0x0B, 0x52, 0x90, 0x36, true},
	{ 1850, 0x58, 0x59, 0xCD, 0x0B, 0x55, 0xC0, 0x59, 0xCE, 0x0B, 0x52, 0x98, 0x36, true},
	{ 1900, 0x58, 0x59, 0xCD, 0x0B, 0x55, 0xC0, 0x59, 0xCE, 0x0B, 0x52, 0x98, 0x1B, true},
	{ 2000, 0x58, 0x59, 0xCD, 0x0B, 0x55, 0xC0, 0x59, 0xCE, 0x0B, 0x52, 0x98, 0x1B, true},
	{ 2100, 0x9A, 0x9B, 0xD0, 0x0E, 0x50, 0xC0, 0x39, 0xD3, 0x0C, 0x4A, 0xA7, 0x1E, true},
};

static int mipi_debug;

static void mnh_sm_mipi_rx_dphy_write_gen3(int command, int data,
					   uint32_t device)
{
	/* Write 4-bit testcode MSB */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTEN, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTEN, 1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTDIN, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTEN, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTDIN, ((command & 0xF00) >> 8));
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 1);

	/* Write 8-bit testcode LSB */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTEN, 1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTDIN, (command & 0xFF));
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTEN, 0);

	/* Write the data */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTDIN, data);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 1);
	udelay(1);
}

static void mnh_sm_mipi_tx_dphy_write_gen3(int command, int data,
					   uint32_t device)
{
	/* Write 4-bit testcode MSB */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTEN, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTEN, 1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTDIN, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTEN, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTDIN, ((command & 0xF00) >> 8));
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 1);

	/* Write 8-bit testcode LSB */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTEN, 1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTDIN, (command & 0xFF));
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTEN, 0);

	/* Write the data */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTDIN, data);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 1);

	udelay(1);
}

/* Bugfix for b/63578602 */
static void mnh_mipi_gen3_lprxpon_wa(struct device *dev, u32 device, int enable)
{
	dev_dbg(dev, "%s: dev %d\n", __func__, device);
	if (enable == 1) {
		mnh_sm_mipi_rx_dphy_write_gen3(0x1AE, 0x07, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x1AD, 0xe0, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x305, 0x06, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x505, 0x06, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x705, 0x06, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x905, 0x06, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0xB05, 0x06, device);
	} else {
		mnh_sm_mipi_rx_dphy_write_gen3(0x1AE, 0x00, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x1AD, 0x00, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x305, 0x00, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x505, 0x00, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x705, 0x00, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0x905, 0x00, device);
		mnh_sm_mipi_rx_dphy_write_gen3(0xB05, 0x00, device);
	}
}

static int mnh_mipi_gen3_lookup_freq_code(uint32_t rate)
{
	int i;

	if ((rate < mipi_rate_configs[0].rate) ||
	    (rate > mipi_rate_configs[ARRAY_SIZE(mipi_rate_configs)-1].rate))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(mipi_rate_configs); i++) {
		if (rate < mipi_rate_configs[i].rate)
			return i - 1;
	}

	return i - 1;
}

static int mnh_mipi_gen3_lookup_device_cfg_idx(uint32_t rate)
{
	int i;

	if ((rate < mipi_dev_ovr_cfgs[0].rate) ||
	    (rate > mipi_dev_ovr_cfgs[ARRAY_SIZE(mipi_dev_ovr_cfgs)-1].rate))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(mipi_dev_ovr_cfgs); i++) {
		if (rate <= mipi_dev_ovr_cfgs[i].rate)
			return i;
	}

	return i - 1;
}

static void mnh_mipi_gen3_host(struct device *dev, uint32_t device,
			       uint32_t rate)
{
	uint32_t code_index, freq_range_code, osc_freq_code;

	/* only support devices 0-2 */
	if (device > 2)
		return;

	dev_dbg(dev, "%s: dev %d, rate %d\n", __func__, device, rate);

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, N_LANES, N_LANES, 0x0);

	/* clear interrupts */
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, INT_MSK_PHY_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, INT_MSK_PKT_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, INT_MSK_FRAME_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, INT_MSK_PHY,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, INT_MSK_PKT,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, INT_MSK_LINE,
	       0xFFFFFFFF);

	/* enable clock to controller */
	if (device == 0)
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX0_CG, 0x0);
	else if (device == 1)
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX1_CG, 0x0);
	else
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX2_CG, 0x0);

	/* disable the PHY before changing settings */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, DPHY_RSTZ, DPHY_RSTZ,
		0x0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x0);

	/* get the PHY settings */
	code_index = mnh_mipi_gen3_lookup_freq_code(rate);
	if (code_index < 0)
		return;
	freq_range_code = mipi_rate_configs[code_index].freq_range_code;
	osc_freq_code = mipi_rate_configs[code_index].osc_freq_code;

	/* update PHY configuration */
	if (device == 0)
		HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_DPHY_CONFIG,
		       0x8400 | freq_range_code);
	else if (device == 1)
		HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX1_DPHY_CONFIG,
		       0x8400 | freq_range_code);
	else
		HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX2_DPHY_CONFIG,
		       0x8400 | freq_range_code);

	/* update PHY PLL settings */
	mnh_sm_mipi_rx_dphy_write_gen3(0xe2, osc_freq_code & 0xFF, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0xe3, osc_freq_code >> 8, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0xe4, 0x01, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0x08, 0x20, device);
	udelay(1);

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, N_LANES, N_LANES, 0x3);
	udelay(1);

	/* b/63578602 */
	mnh_mipi_gen3_lprxpon_wa(dev, device, 1);

	/* release reset */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x1);
	udelay(20);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, DPHY_RSTZ, DPHY_RSTZ,
		0x1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, CSI2_RESETN,
		CSI2_RESETN, 0x1);

	udelay(30);

	mnh_mipi_gen3_lprxpon_wa(dev, device, 0);
}

static uint8_t mnh_mipi_get_vco_cntrl(uint32_t rate)
{
	int i;
	/* vco frequency is half the desired rate */
	uint32_t freq_kHz = rate * 1000 / 2;

	for (i = 0; i < ARRAY_SIZE(vco_range_cntrls); i++) {
		if (freq_kHz < vco_range_cntrls[i].freq_kHz)
			return vco_range_cntrls[i - 1].cntrl;
	}

	return vco_range_cntrls[i].cntrl;
}

/* M is required to be between 64 and 625 */
/* N is required to be between 6 and 16 */
static void mnh_mipi_get_pll_coeffs(uint32_t rate, uint16_t *m, uint8_t *n)
{
	uint32_t freq_kHz = rate * 1000 / 2; /* to not lose precision */
	int vco_div_factor;
	int est_ratio_scaled; /* M/N ratio scaled by 1000 to losing precision */
	int iter_n, iter_m;
	int best_error = INT_MAX;
	int best_n = 0, best_m = 0;
	int calc_ratio_scaled;

	/* determine the vco division factor */
	if (freq_kHz < 80000)
		vco_div_factor = 8;
	else if (freq_kHz < 160000)
		vco_div_factor = 4;
	else if (freq_kHz < 320000)
		vco_div_factor = 2;
	else
		vco_div_factor = 1;

	/* determine desired ratio, scaled by 1000 to avoid losing precision */
	est_ratio_scaled = freq_kHz * 1000 * vco_div_factor / REF_FREQ_KHZ;

	/* search through values of N and find ratio with least error */
	for (iter_n = 4; iter_n <= 12; iter_n++) {
		/* always round up so we don't get lower tx rate than we want */
		iter_m = (est_ratio_scaled * iter_n + 999) / 1000;
		calc_ratio_scaled = iter_m * 1000 / iter_n;

		if (iter_m < 64)
			continue;
		else if (iter_m > 625)
			break;

		/* calculate the error given this M and N combo */
		if ((calc_ratio_scaled - est_ratio_scaled) < best_error) {
			best_n = iter_n;
			best_m = iter_m;
			best_error = calc_ratio_scaled - est_ratio_scaled;
		}
	}

	/* assign the outputs */
	*m = best_m;
	*n = best_n;
}

static void mnh_mipi_gen3_device(struct device *dev, uint32_t device,
				 uint32_t rate)
{
	uint32_t code_index, freq_range_code, osc_freq_code;
	struct mipi_dev_ovr_cfg *dev_ovr_cfg;
	uint32_t mipi_dev_cfg_index;
	unsigned long data;
	uint8_t vco_cntrl;
	uint16_t pll_m;
	uint8_t pll_n;
	int i = 0;

	dev_dbg(dev, "%s: dev %d, rate %d\n", __func__, device, rate);

	if (!mipi_debug) {
		if (rate <= 360) {
			dev_dbg(dev, "%s: %d config. default 360\n",
				 __func__, rate);
			rate = 360;
		} else if (rate <= 550) { /* debug mode */
			dev_dbg(dev, "%s: %d config. default 550\n",
				 __func__, rate);
			rate = 550;
		} else if (rate <= 685) { /* front cam photo */
			dev_dbg(dev, "%s: %d config. default 685\n",
				 __func__, rate);
			rate = 685;
		} else if (rate <= 720) { /* main cam 60fps */
			dev_dbg(dev, "%s: %d config. default 720\n",
				 __func__, rate);
			rate = 720;
		} else if (rate <= 984) { /* main cam 30fps */
			dev_dbg(dev, "%s: %d config. default 984\n",
				 __func__, rate);
			rate = 984;
		} else if (rate <= 1104) { /* main cam slo.mo. 120fps */
			dev_dbg(dev, "%s: %d config. default 1104\n",
				 __func__, rate);
			rate = 1104;
		} else if (rate <= 1200) {
			dev_dbg(dev, "%s: %d config. default 1200\n",
				 __func__, rate);
			rate = 1200;
		} else if (rate <= 1368) {
			dev_dbg(dev, "%s: %d config. default 1368\n",
				 __func__, rate);
			rate = 1368;
		} else if (rate <= 1850) {
			dev_dbg(dev, "%s: %d config. default 1850\n",
				 __func__, rate);
			rate = 1850;
		}
	}

	/* only support devices 0-1 */
	if (device > 1)
		return;

	if (device == 0)
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_TX0_CG, 0x0);
	else
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_TX1_CG, 0x0);

	/* mipicsi_device_hw_init */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 0x1);

	/* mipicsi_device_dphy_reset */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ,
		0x1);
	udelay(1000);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ,
		0x0);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, CSI2_RESETN, 0x0);
	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, CSI2_RESETN, 0x1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, INT_MASK_N_VPG,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, INT_MASK_N_IDI,
	       0xFFFFFFFF);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ,
		0x0);

	/* disable clock gating */
	if (device == 0) {
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_PLL_CNTRL,
			PLL_SHADOW_CONTROL, 0x1);
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_PLL_CNTRL,
			CLK_SEL, 0x1);
	} else {
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_PLL_CNTRL,
			PLL_SHADOW_CONTROL, 0x1);
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_PLL_CNTRL,
			CLK_SEL, 0x1);
	}

	/* disable the PHY before changing settings */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, CSI2_RESETN,
		CSI2_RESETN_RW, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, 0x0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x0);

	/* get the PHY settings */
	code_index = mnh_mipi_gen3_lookup_freq_code(rate);
	if (code_index < 0)
		return;
	freq_range_code = mipi_rate_configs[code_index].freq_range_code;
	osc_freq_code = mipi_rate_configs[code_index].osc_freq_code;

	/* update PHY configuration */
	if (device == 0)
		HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_CONFIG,
		       0x8400 | freq_range_code);
	else
		HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_CONFIG,
		       0x8400 | freq_range_code);

	/* configure slew rate calibration */
	if (rate > 1500) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x26B, 0x44, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x272, 0x00, device);
	} else if (rate > 1000) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x270, 0xD0, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x271, 0x07, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x272, 0x10, device);
	} else if (rate > 500) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x270, 0xE2, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x271, 0x04, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x272, 0x11, device);
	} else {
		mnh_sm_mipi_tx_dphy_write_gen3(0x270, 0x84, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x271, 0x03, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x272, 0x11, device);
	}

	/* get PLL coefficients and codes */
	vco_cntrl = mnh_mipi_get_vco_cntrl(rate);
	mnh_mipi_get_pll_coeffs(rate, &pll_m, &pll_n);

	/* mipi device configuration settings from lookup table */
	mipi_dev_cfg_index = mnh_mipi_gen3_lookup_device_cfg_idx(rate);
	if (mipi_dev_cfg_index < 0)
		return;
	dev_ovr_cfg = &mipi_dev_ovr_cfgs[mipi_dev_cfg_index];
	dev_dbg(dev, "%s: Device configuration index: %d\n", __func__,
		mipi_dev_cfg_index);

	/* adjust the values to meet the register definition */
	pll_m -= 2;
	pll_n -= 1;

	/* configure PLL frequency multiplication and division factors */
	mnh_sm_mipi_tx_dphy_write_gen3(0x17B, ((vco_cntrl & 0x3F) << 1) | 0x81,
				       device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x178, 0x80 | ((pll_n & 0xF) << 3),
				       device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x179, pll_m & 0xFF, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x17A, (pll_m >> 8) & 0x3, device);

	/* configure rate-independent PLL settings */
	mnh_sm_mipi_tx_dphy_write_gen3(0x15E, 0x10, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x162, 0x04, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x16E, 0x0C, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x173, 0x02, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x174, 0x00, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x175, 0x60, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x176, 0x03, device);

	/* TODO: Maybe do a read-modify-write */
	if (rate <= 450)
		mnh_sm_mipi_tx_dphy_write_gen3(0x1AC, 1 << 4, device);

	/* Wait for 15ns */
	udelay(1);

	/* Timing register overrides */
	if (!mipi_debug && dev_ovr_cfg->use_ovrd) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x5A, dev_ovr_cfg->reg_0x5A,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5B, dev_ovr_cfg->reg_0x5B,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5C, dev_ovr_cfg->reg_0x5C,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5D, dev_ovr_cfg->reg_0x5D,
					       device);
		/*
		 * mnh_sm_mipi_tx_dphy_write_gen3(0x5E, dev_ovr_cfg->reg_0x5E,
		 * device);
		 */
		mnh_sm_mipi_tx_dphy_write_gen3(0x5F, dev_ovr_cfg->reg_0x5F,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x61, dev_ovr_cfg->reg_0x61,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x62, dev_ovr_cfg->reg_0x62,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x63, dev_ovr_cfg->reg_0x63,
					       device);
		/*
		 * mnh_sm_mipi_tx_dphy_write_gen3(0x64, dev_ovr_cfg->reg_0x64,
		 * device);
		 */
		mnh_sm_mipi_tx_dphy_write_gen3(0x65, dev_ovr_cfg->reg_0x65,
					       device);
	}

	/* Enable high speed drivers (required for Tlpx < 500ns) */
	/* Clock lane driver */
	mnh_sm_mipi_tx_dphy_write_gen3(0x304, (1 << 2) | (1 << 3), device);
	/* Data lane 0 driver */
	mnh_sm_mipi_tx_dphy_write_gen3(0x504, (1 << 2) | (1 << 3), device);
	/* Data lane 1 driver */
	mnh_sm_mipi_tx_dphy_write_gen3(0x704, (1 << 2) | (1 << 3), device);
	/* Data lane 2 driver */
	mnh_sm_mipi_tx_dphy_write_gen3(0x904, (1 << 2) | (1 << 3), device);
	/* Data lane 3 driver */
	mnh_sm_mipi_tx_dphy_write_gen3(0xB04, (1 << 2) | (1 << 3), device);

	/* Enable lanes */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		LANE_EN_NUM, 3);

	/* Table A-4 High-Speed Transition Times: Data HS-LP */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		PHY_STOP_WAIT_TIME, dev_ovr_cfg->wait_time);

	/* enable the PHY */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ, 1);

	/* wait for data & clk lanes to reach the low-power state */
	i = 0;
	do {
		data = HW_IN(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX,
			     PHY_STATUS);
		udelay(10);
		i++;
	} while ((i < 40) && ((data & 0x1550) != 0x1550));

	if ((i >= 100) && ((data & 0x1550) != 0x1550))
		dev_err(dev,
			"device %d could not drive lp state, status 0x%lx\n",
		       device, data);

	/* INIT period in continuous clock mode is software controlled */
	udelay(1000); /* relevant timing parameter, do not remove */

	/* Enable ct clock, this takes immediate effect */
	dev_dbg(dev, "%s: enabling ct. clock mode\n", __func__);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX,
		LPCLK_CTRL, PHY_TXREQCLKHS_CON, 1);
}

static int mnh_mipi_config_mux_sel(struct device *dev,
				    struct mnh_mipi_config *config)
{

	uint32_t rxdev, txdev, mode;
	uint32_t vc_en_mask, byp_tx_en_mask, ipu_en_mask;
	uint32_t tx_func_mask, tx_byp_sel_mask;
	uint32_t rx_mode, tx_mode;

	rxdev = config->rxdev;
	txdev = config->txdev;
	vc_en_mask = config->vc_en_mask;
	mode = config->mode;

	dev_dbg(dev, "%s: MUX rxdev %d, txdev %d, vc_en_mask 0x%1x, mode %d\n",
		 __func__, rxdev, txdev, vc_en_mask, mode);

	/*
	 * 0. Upper layers should issue a MUX reset prior to configuring
	 *    the MUX
	 * 1. Configure RX section of the MUX unless rxdev is MIPI_RX_IPU
	 *    in that case we are in functional mode IPU -> Tx[n] and only
	 *    the tx_mode register must be changed
	 * 2. Configure TX section of the MUX unless txdev is MIPI_TX_IPU
	 *    in that case we are in functional mode Rx[n] -> IPU and only
	 *    the rx_mode register must be changed
	 */

	/*
	 * construct the rx mode register. This register is synchronized with
	 * the IDI domain when the RX clock is present. However, if the clock is
	 * not present, only the first write to the register will succeed.
	 * Therefore, use read-modify-write to construct register so we only
	 * need to write once.
	 */
	if (rxdev == MIPI_RX0)
		rx_mode = HW_IN(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_MODE);
	else if (rxdev == MIPI_RX1)
		rx_mode = HW_IN(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX1_MODE);
	else if (rxdev == MIPI_RX2)
		rx_mode = HW_IN(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX2_MODE);
	else
		rx_mode = 0; /* rx_mode register not required for IPU source */
	/*
	 * it doesn't actually matter which RX# is used since this is just to
	 * create the mask. The actual register write occurs below.
	 */
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_VC_EN_FLDMASK;
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_BYP_TX0_EN_FLDMASK;
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_BYP_TX1_EN_FLDMASK;
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_IPU_EN_FLDMASK;

	vc_en_mask <<= HWIO_MIPI_TOP_RX0_MODE_RX0_VC_EN_FLDSHFT;
	vc_en_mask &= HWIO_MIPI_TOP_RX0_MODE_RX0_VC_EN_FLDMASK;

	/* Bypass mode to one of the two Tx channels */
	if (mode == MIPI_MODE_BYPASS) { /* no IPU, TXn_EN */
		byp_tx_en_mask =
			HWIO_MIPI_TOP_RX0_MODE_RX0_BYP_TX0_EN_FLDMASK << txdev;
		ipu_en_mask = 0;
		tx_func_mask = 0;
		tx_byp_sel_mask = (1 + rxdev)
			<< HWIO_MIPI_TOP_TX0_MODE_TX0_BYP_SEL_FLDSHFT;
	/* Bypass mode with IPU sink enabled */
	} else if (mode == MIPI_MODE_BYPASS_W_IPU) {
		byp_tx_en_mask =
			HWIO_MIPI_TOP_RX0_MODE_RX0_BYP_TX0_EN_FLDMASK << txdev;
		ipu_en_mask = 1 << HWIO_MIPI_TOP_RX0_MODE_RX0_IPU_EN_FLDSHFT;
		tx_func_mask = 0;
		tx_byp_sel_mask = (1 + rxdev)
			<< HWIO_MIPI_TOP_TX0_MODE_TX0_BYP_SEL_FLDSHFT;
	/* Functional mode, direct bypass between Rx & TX disabled */
	} else if (mode == MIPI_MODE_FUNCTIONAL) {
		byp_tx_en_mask = 0;
		ipu_en_mask = 1 << HWIO_MIPI_TOP_RX0_MODE_RX0_IPU_EN_FLDSHFT;
		tx_func_mask = 1 << HWIO_MIPI_TOP_TX0_MODE_TX0_FUNC_FLDSHFT;
		tx_byp_sel_mask = 0; /* do not care */
	} else {
		return -EINVAL;
	}

	rx_mode = (rx_mode | vc_en_mask | byp_tx_en_mask | ipu_en_mask);
	tx_mode = (tx_func_mask | tx_byp_sel_mask);
	dev_dbg(dev, "%s: rx_mode=%d | tx_mode=%d\n", __func__,
		rx_mode, tx_mode);

	switch (txdev) { /* Sink */
	case MIPI_TX0:
		switch (rxdev) { /* Source */
		case MIPI_RX0: /* RX0 -> TX0 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX0_MODE, rx_mode);
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX0_MODE, tx_mode);
			break;
		case MIPI_RX1: /* RX1 -> TX0 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX1_MODE, rx_mode);
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX0_MODE, tx_mode);
			break;
		case MIPI_RX2: /* RX2 -> TX0 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX2_MODE, rx_mode);
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX0_MODE, tx_mode);
			break;
		case MIPI_RX_IPU: /* IPU -> TX0 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX0_MODE, tx_mode);
			break;
		default:
			dev_err(dev, "%s: invalid rx device %d!\n", __func__,
				rxdev);
			return -EINVAL;
		}
		break;
	case MIPI_TX1:
		switch (rxdev) {
		case MIPI_RX0: /* RX0 -> TX1 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX0_MODE, rx_mode);
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX1_MODE, tx_mode);
			break;
		case MIPI_RX1: /* RX1 -> TX1 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX1_MODE, rx_mode);
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX1_MODE, tx_mode);
			break;
		case MIPI_RX2: /* RX2 -> TX1 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX2_MODE, rx_mode);
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX1_MODE, tx_mode);
			break;
		case MIPI_RX_IPU:
			/* Configure IPU -> Tx1 */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, TX1_MODE, tx_mode);
			break;
		default:
			dev_err(dev, "%s: invalid rx device %d!\n", __func__,
				rxdev);
			return -EINVAL;
		}
		break;
	case MIPI_TX_IPU:
		switch (rxdev) {
		case MIPI_RX0: /* Rx0 -> IPU */
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX0_MODE, rx_mode);
			break;
		case MIPI_RX1:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX1_MODE, rx_mode);
			break;
		case MIPI_RX2:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX2_MODE, rx_mode);
			break;
		default:
			dev_err(dev, "%s: invalid rx device %d!\n", __func__,
				rxdev);
			return -EINVAL;
		}
		break;
	default:
		dev_err(dev, "%s: invalid tx device %d!\n", __func__, txdev);
		return -EINVAL;
	}
	return 0;
}

int mnh_mipi_config(struct device *dev, struct mnh_mipi_config config)
{
	uint32_t txdev = config.txdev;
	uint32_t rxdev = config.rxdev;
	uint32_t rx_rate = config.rx_rate;
	uint32_t tx_rate = config.tx_rate;
	uint32_t vc_en_mask = config.vc_en_mask;

	dev_dbg(dev, "%s: init rxdev %d, txdev %d, rx_rate %d, tx_rate %d, vc_en_mask 0x%1x\n",
		__func__, rxdev, txdev, rx_rate, tx_rate, vc_en_mask);

	/* configure rx / MIPI-source */
	switch (rxdev) {
	case MIPI_RX0:
	case MIPI_RX1:
	case MIPI_RX2:
		dev_dbg(dev, "%s: configuring host controller Rx%d\n",
			 __func__, rxdev);
		mnh_mipi_gen3_host(dev, rxdev, rx_rate);
		break;
	case MIPI_RX_IPU:
		dev_dbg(dev, "%s: configuring IPU IDI Tx%d as MIPI source\n",
			 __func__, txdev);
		break;
	default:
		dev_dbg(dev, "%s: Invalid MIPI input device\n", __func__);
		break;
	}

	/* configure tx / MIPI-sink */
	switch (txdev) {
	case MIPI_TX0:
	case MIPI_TX1:
		dev_dbg(dev, "%s: configuring device controller Tx%d\n",
			 __func__, txdev);
		mnh_mipi_gen3_device(dev, txdev, tx_rate);
		break;
	case MIPI_TX_IPU:
		dev_dbg(dev, "%s: configuring IPU IDI Rx%d as MIPI sink\n",
			 __func__, rxdev);
		break;
	default:
		dev_dbg(dev, "%s: Invalid MIPI output device\n", __func__);
		break;
	}

	/* configure mux select */
	mnh_mipi_config_mux_sel(dev, &config);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_mipi_config);

int mnh_mipi_stop(struct device *dev, struct mnh_mipi_config config)
{
	uint32_t txdev = config.txdev;
	uint32_t rxdev = config.rxdev;

	dev_dbg(dev, "%s: stopping rxdev %d, txdev %d\n", __func__, rxdev,
		 txdev);

	/* Shutdown host */
	mnh_mipi_stop_host(dev, rxdev);

	/* Shutdown device */
	mnh_mipi_stop_device(dev, txdev);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_mipi_stop);

void mnh_mipi_stop_device(struct device *dev, int txdev)
{
	/* shut down the PHY */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, PHY_RSTZ,
		PHY_RSTZ, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 0);
	/* shut down the controller logic */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, CSI2_RESETN,
		CSI2_RESETN_RW, 0);
	/* enable clock gating (disable clock) */
	switch (txdev) {
	case MIPI_TX0:
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_TX0_CG, 0x1);
		break;
	case MIPI_TX1:
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_TX1_CG, 0x1);
		break;
	default:
		dev_err(dev, "%s invalid mipi device!\n", __func__);
		break;
	}
}
EXPORT_SYMBOL_GPL(mnh_mipi_stop_device);

void mnh_mipi_stop_host(struct device *dev, int rxdev)
{
	/* shut down the PHY */
	/* see 7.3.1: Rx-DPHY databook */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, DPHY_RSTZ,
		DPHY_RSTZ, 0x0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLR, 0x1);
	/* shut down the controller logic */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, CSI2_RESETN,
		CSI2_RESETN, 0x0);
	/* enable clock gating (disable clock) */
	switch (rxdev) {
	case MIPI_RX0:
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX0_CG, 0x1);
		break;
	case MIPI_RX1:
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX1_CG, 0x1);
		break;
	case MIPI_RX2:
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX2_CG, 0x1);
		break;
	default:
		dev_err(dev, "%s invalid mipi host!\n", __func__);
		break;
	}
}
EXPORT_SYMBOL_GPL(mnh_mipi_stop_host);

void mnh_mipi_set_debug(int val)
{
	mipi_debug = val;
}
EXPORT_SYMBOL(mnh_mipi_set_debug);

int mnh_mipi_get_top_interrupts(struct device *dev,
				struct mipi_device_irq_st *int_status)
{
	u32 status;

	switch (int_status->dev) {
	case 0:
		status = TOP_IN(TX0_BYPINT);
		if (status & TOP_MASK(TX0_BYPINT, TX0_BYP_OF)) {
			TOP_OUTf(TX0_BYPINT, TX0_BYP_OF, 1);
			dev_info(dev, "TX0_BYPINT BYP_OF occurred\n");
			int_status->fifo_overflow = 1;
		}
		break;
	case 1:
		status = TOP_IN(TX1_BYPINT);
		if (status & TOP_MASK(TX1_BYPINT, TX1_BYP_OF)) {
			TOP_OUTf(TX1_BYPINT, TX1_BYP_OF, 1);
			dev_info(dev, "TX1_BYPINT BYP_OF occurred\n");
			int_status->fifo_overflow = 1;
		}
		break;
	default:
		break;
	}
	return status;
}

int mnh_mipi_get_device_interrupts(struct device *dev,
			struct mipi_device_irq_st *int_status)
{
	uint32_t baddr = HWIO_MIPI_TX_BASE_ADDR(int_status->dev);

	int_status->main = TX_IN(INT_ST_MAIN);
	dev_info(dev, "MIPI device controller %d interrupt main: %x\n",
		 int_status->dev, int_status->main);

	if (int_status->main & TX_MASK(INT_ST_MAIN, INT_ST_VPG)) {
		int_status->vpg = TX_IN(INT_ST_VPG);
		dev_info(dev, "CSI INT_ST_VPG: %x\n", int_status->vpg);
	}
	if (int_status->main & TX_MASK(INT_ST_MAIN, INT_ST_IDI)) {
		int_status->idi = TX_IN(INT_ST_IDI);
		dev_info(dev, "CSI INT_ST_IDI: %x\n", int_status->idi);
	}
	if (int_status->main & TX_MASK(INT_ST_MAIN, INT_ST_PHY)) {
		int_status->phy = TX_IN(INT_ST_PHY);
		dev_info(dev, "CSI INT_ST_PHY: %x\n", int_status->phy);
	}
	mnh_mipi_get_top_interrupts(dev, int_status);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_mipi_get_device_interrupts);

int mnh_mipi_get_host_interrupts(struct device *dev,
				 struct mipi_host_irq_st *int_status)
{

	uint32_t baddr = HWIO_MIPI_RX_BASE_ADDR(int_status->dev);

	int_status->main = RX_IN(INT_ST_MAIN);
	dev_info(dev, "MIPI host controller %d interrupt main: %x\n",
		 int_status->dev, int_status->main);

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PHY_FATAL)) {
		int_status->phy_fatal = RX_IN(INT_ST_PHY_FATAL);
		dev_info(dev, "CSI INT PHY FATAL: %x\n",
				int_status->phy_fatal);
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PKT_FATAL)) {
		int_status->pkt_fatal = RX_IN(INT_ST_PKT_FATAL);
		dev_info(dev, "CSI INT PKT FATAL: %x\n",
				int_status->pkt_fatal);
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_FRAME_FATAL)) {
		int_status->frame_fatal = RX_IN(INT_ST_FRAME_FATAL);
		dev_info(dev, "CSI INT FRAME FATAL: %x\n",
				int_status->frame_fatal);
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PHY)) {
		int_status->phy = RX_IN(INT_ST_PHY);
		dev_info(dev, "CSI INT PHY: %x\n", int_status->phy);
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_PKT)) {
		int_status->pkt = RX_IN(INT_ST_PKT);
		dev_info(dev, "CSI INT PKT: %x\n", int_status->pkt);
	}

	if (int_status->main & RX_MASK(INT_ST_MAIN, STATUS_INT_LINE)) {
		int_status->line = RX_IN(INT_ST_LINE);
		dev_info(dev, "CSI INT LINE: %x\n", int_status->line);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_mipi_get_host_interrupts);
