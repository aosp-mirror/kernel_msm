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

#include <linux/delay.h>

#include "mnh-hwio.h"
#include "mnh-hwio-mipi-rx.h"
#include "mnh-hwio-mipi-tx.h"
#include "mnh-hwio-mipi-top.h"
#include "mnh-hwio-bases.h"
#include "mnh-mipi.h"

#define REF_FREQ_KHZ    25000

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
	{  340, 0x57, 0x43, 0xC2, 0x44, 0x42, 0x8A, 0x43, 0xC3, 0x44, 0x43, 0x83, 0x13, true},
	{  446, 0x57, 0x45, 0xC3, 0x9D, 0x43, 0x8E, 0x45, 0xC3, 0x9D, 0x44, 0x84, 0x17, true},
	{  488, 0x58, 0x45, 0xC3, 0xBC, 0x43, 0x90, 0x45, 0xC3, 0xBC, 0x44, 0x85, 0x17, true},
	{  648, 0x4C, 0x48, 0xC5, 0x03, 0x44, 0x91, 0x47, 0xC6, 0x03, 0x43, 0x85, 0xE,  true},
	{  675, 0x4C, 0x48, 0xC5, 0x03, 0x44, 0x91, 0x47, 0xC6, 0x03, 0x43, 0x85, 0xE,  true},
	{  720, 0x4E, 0x48, 0xC5, 0x03, 0x46, 0x95, 0x48, 0xC6, 0x03, 0x47, 0x86, 0x1D, true},
	{  984, 0x50, 0x4D, 0xC7, 0x05, 0x4A, 0xA0, 0x4D, 0xC8, 0x05, 0x4A, 0x8A, 0x13, true},
	{ 1104, 0x5D, 0x4F, 0xC6, 0xB5, 0x49, 0xA7, 0x4F, 0xC7, 0xB5, 0x49, 0x8E, 0x27, true},
	{ 1296, 0x53, 0x51, 0xCA, 0x09, 0x4A, 0xA7, 0x50, 0xCC, 0x07, 0x46, 0x8D, 0x2E, true},
	{ 1350, 0x53, 0x51, 0xCA, 0x09, 0x4A, 0xA7, 0x50, 0xCC, 0x07, 0x46, 0x8D, 0x2E, true},
	{ 1776, 0x58, 0x59, 0xCC, 0x0B, 0x54, 0xC0, 0x59, 0xCE, 0x0B, 0x51, 0x97, 0x17, true},
	{ 2100, 0x9A, 0x9B, 0xD0, 0x0E, 0x50, 0xC0, 0x39, 0xD3, 0x0C, 0x4A, 0xA7, 0x1E, true},
};

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

	return i;
}

static int mnh_mipi_gen3_lookup_device_cfg_idx(uint32_t rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mipi_dev_ovr_cfgs); i++)
		if (rate == mipi_dev_ovr_cfgs[i].rate)
			return i;

	return -EINVAL;
}

static void mnh_mipi_gen3_host(struct device *dev, uint32_t device,
			       uint32_t rate)
{
	uint32_t code_index, freq_range_code, osc_freq_code;

	/* only support devices 0-2 */
	if (device > 2)
		return;

	dev_info(dev, "%s: dev %d, rate %d\n", __func__, device, rate);

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
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);

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

	/* release reset */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, DPHY_RSTZ, DPHY_RSTZ,
		0x1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, CSI2_RESETN,
		CSI2_RESETN, 0x1);
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

	dev_info(dev, "%s: dev %d, rate %d\n", __func__, device, rate);

	/* Functional configurations are currently 1350, 720, 675 */
	if (rate <= 675) {
		dev_info(dev, "%s: %d config. default 675\n", __func__, rate);
		rate = 675;
	} else if (rate <= 720) {
		dev_info(dev, "%s: %d config. default 720\n", __func__, rate);
		rate = 720;
	} else if (rate <= 1350) {
		dev_info(dev, "%s: %d config. default 1350\n", __func__, rate);
		rate = 1350;
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
	dev_ovr_cfg = &mipi_dev_ovr_cfgs[mipi_dev_cfg_index];
	dev_info(dev, "%s: Device configuration index: %d\n", __func__,
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

	/* Enable lanes */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		LANE_EN_NUM, 3);

	/* Timing register overrides */
	if (dev_ovr_cfg->use_ovrd) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x5A, dev_ovr_cfg->reg_0x5A,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5B, dev_ovr_cfg->reg_0x5B,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5C, dev_ovr_cfg->reg_0x5C,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5D, dev_ovr_cfg->reg_0x5D,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5E, dev_ovr_cfg->reg_0x5E,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5F, dev_ovr_cfg->reg_0x5F,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x61, dev_ovr_cfg->reg_0x61,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x62, dev_ovr_cfg->reg_0x62,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x63, dev_ovr_cfg->reg_0x63,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x64, dev_ovr_cfg->reg_0x64,
					       device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x65, dev_ovr_cfg->reg_0x65,
					       device);
	}

	/* TODO: Double-check these wait times */
#if 0
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		PHY_STOP_WAIT_TIME, dev_ovr_cfg->wait_time);
#else
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		    PHY_STOP_WAIT_TIME, ((rate / 100) - 1));
#endif

	/* enable the PHY */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ, 1);

	/* wait for the lanes to reach the low-power state */
	i = 0;
	do {
		data = HW_IN(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX,
			     PHY_STATUS);
		udelay(10);
		i++;
	} while ((i < 40) && ((data & 0x1550) != 0x1550));

	if ((i >= 40) && ((data & 0x1550) != 0x1550))
		dev_err(dev, "device %d could not drive low-power state, status 0x%lx\n",
		       device, data);
}

static void mnh_mipi_config_mux_sel(struct device *dev, uint32_t rxdev,
				    uint32_t txdev, uint32_t vc_en_mask)
{
	uint32_t rx_mode, tx_en_mode;

	/*
	 * construct the rx mode register. This register is synchronized with
	 * the IDI domain when the RX clock is present. However, if the clock is
	 * not present, only the first write to the register will succeed.
	 * Therefore, use read-modify-write to construct register so we only
	 * need to write once.
	 */
	if (rxdev == 0)
		rx_mode = HW_IN(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_MODE);
	else if (rxdev == 1)
		rx_mode = HW_IN(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX1_MODE);
	else
		rx_mode = HW_IN(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX2_MODE);

	/*
	 * it doesn't actually matter which RX# I use since this is just to
	 * create the mask. The actual register write occurs below.
	 */
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_VC_EN_FLDMASK;
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_BYP_TX0_EN_FLDMASK;
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_BYP_TX1_EN_FLDMASK;
	rx_mode &= ~HWIO_MIPI_TOP_RX0_MODE_RX0_IPU_EN_FLDMASK;

	vc_en_mask <<= HWIO_MIPI_TOP_RX0_MODE_RX0_VC_EN_FLDSHFT;
	vc_en_mask &= HWIO_MIPI_TOP_RX0_MODE_RX0_VC_EN_FLDMASK;

	tx_en_mode = HWIO_MIPI_TOP_RX0_MODE_RX0_BYP_TX0_EN_FLDMASK << txdev;

	rx_mode = (rx_mode | vc_en_mask | tx_en_mode);

	switch (txdev) {
	case 0:
		switch (rxdev) {
		case 0:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX0_MODE, rx_mode);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX0_MODE, TX0_BYP_SEL, 0x1);
			break;
		case 1:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX1_MODE, rx_mode);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX0_MODE, TX0_BYP_SEL, 0x2);
			break;
		case 2:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX2_MODE, rx_mode);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX0_MODE, TX0_BYP_SEL, 0x3);
			break;
		default:
			dev_err(dev, "%s: invalid rx device %d!\n", __func__,
				rxdev);
		}
		break;
	case 1:
		switch (rxdev) {
		case 0:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX0_MODE, rx_mode);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX1_MODE, TX1_BYP_SEL, 0x1);
			break;
		case 1:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX1_MODE, rx_mode);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX1_MODE, TX1_BYP_SEL, 0x2);
			break;
		case 2:
			HW_OUT(HWIO_MIPI_TOP_BASE_ADDR,
			       MIPI_TOP, RX2_MODE, rx_mode);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX1_MODE, TX1_BYP_SEL, 0x3);
			break;
		default:
			dev_err(dev, "%s: invalid rx device %d!\n", __func__,
				rxdev);
		}
		break;
	default:
		dev_err(dev, "%s: invalid tx device %d!\n", __func__, txdev);
	}
}

int mnh_mipi_config(struct device *dev, struct mnh_mipi_config config)
{
	uint32_t txdev = config.txdev;
	uint32_t rxdev = config.rxdev;
	uint32_t rx_rate = config.rx_rate;
	uint32_t tx_rate = config.tx_rate;
	uint32_t vc_en_mask = config.vc_en_mask;

	dev_info(dev, "%s: init rxdev %d, txdev %d, rx_rate %d, tx_rate %d, vc_en_mask 0x%1x\n",
		__func__, rxdev, txdev, rx_rate, tx_rate, vc_en_mask);

	/* configure rx */
	mnh_mipi_gen3_host(dev, rxdev, rx_rate);

	/* configure tx */
	mnh_mipi_gen3_device(dev, txdev, tx_rate);

	/* configure mux select */
	mnh_mipi_config_mux_sel(dev, rxdev, txdev, vc_en_mask);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_mipi_config);
