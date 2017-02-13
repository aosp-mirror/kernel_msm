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

#define HWIO_MIPI_TX_BASE_ADDR_TXO HWIO_MIPI_TX_BASE_ADDR(0)
#define HWIO_MIPI_RX_BASE_ADDR_RXO HWIO_MIPI_RX_BASE_ADDR(0)

static void mnh_sm_mipi_rx_dphy_write_haps(int command, int data)
{
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, DPHY_RSTZ,
		DPHY_RSTZ, 0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL0, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 1);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL1, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTDIN, command);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTEN, 1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTEN, 0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL1,
		PHY_TESTDIN, data);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL0,
		PHY_TESTCLK, 1);
	udelay(1);
}

static void mnh_sm_mipi_tx_dphy_write_haps(int command, int data)
{
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLR, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTDIN, command);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTEN, 1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTEN, 0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL1,
		PHY0_TESTDIN, data);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL0,
		PHY0_TESTCLK, 1);
	udelay(1);
}

int mnh_sm_mipi_bypass_init_haps(void)
{
	unsigned int long data;
	int i = 0;

	/*
	 * ##########################################################
	 * #  mipicsi_host_hw_init
	 * ##########################################################
	 */

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, N_LANES, N_LANES, 0x0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, INT_MSK_PHY_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, INT_MSK_PKT_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, INT_MSK_FRAME_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, INT_MSK_PHY, 0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, INT_MSK_PKT, 0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, INT_MSK_LINE, 0xFFFFFFFF);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);

	/*
	 * ##########################################################
	 * #  Setup RX (Host)
	 * ##########################################################
	 */
	/* mipicsi_host_start */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, DPHY_RSTZ,
		DPHY_RSTZ, 0x0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_TEST_CTRL0, 0x1);
	mnh_sm_mipi_rx_dphy_write_haps(0x3B, 0x08);
	mnh_sm_mipi_rx_dphy_write_haps(0xB0, 0x1E);
	mnh_sm_mipi_rx_dphy_write_haps(0xAC, 0x03);
	udelay(1);

	HW_OUT(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX,
	       PHY_TEST_CTRL0, 0x00000000);
	udelay(1);

	/* setup freq */
	mnh_sm_mipi_rx_dphy_write_haps(0x44, 0x2E);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, N_LANES,
		N_LANES, 0x03);

	mnh_sm_mipi_rx_dphy_write_haps(0x75, 0xAB);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, DPHY_RSTZ,
		DPHY_RSTZ, 0x1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX, CSI2_RESETN,
		CSI2_RESETN, 0x1);

	do {
		data = HW_IN(HWIO_MIPI_RX_BASE_ADDR_RXO, MIPI_RX,
			     PHY_STOPSTATE);
		udelay(10);
		i++;
	} while ((i < 20) && ((data & 0x1000F) != 0x1000F));

	/*
	 * ##########################################################
	 * #  Setup TX (Device)
	 * ##########################################################
	 */
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, CSI2_TX0_CG,
		0x1);

	/* mipicsi_device_hw_init */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, PHY_SHUTDOWNZ,
		0x1);
	/* mipicsi_device_dphy_reset */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, PHY_RSTZ, 0x1);
	udelay(1000);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, PHY_RSTZ, 0x0);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, CSI2_RESETN, 0x0);
	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, CSI2_RESETN, 0x1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, INT_MASK_N_VPG, 0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, INT_MASK_N_IDI, 0xFFFFFFFF);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, PHY_SHUTDOWNZ,
		0x0);

	/* mipicsi_device_start */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, CSI2_RESETN,
		CSI2_RESETN_RW, 0x1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, 0x0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, PHY_ENABLECLK,
		0x1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL0, 0x1);

	mnh_sm_mipi_tx_dphy_write_haps(0x3B, 0x0E);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_IF_CFG, LANE_EN_NUM,
		0x3);

	mnh_sm_mipi_tx_dphy_write_haps(0x4B, 0x02);
	mnh_sm_mipi_tx_dphy_write_haps(0x5B, 0x02);
	mnh_sm_mipi_tx_dphy_write_haps(0x8B, 0x02);
	mnh_sm_mipi_tx_dphy_write_haps(0x9B, 0x02);

	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY0_TST_CTRL0, 0x0);
	udelay(1);

	/* mipicsi_device_set_pll */
	/* set freq */
	mnh_sm_mipi_tx_dphy_write_haps(0x44, 0x2E);
	mnh_sm_mipi_tx_dphy_write_haps(0x10, 0x81);
	mnh_sm_mipi_tx_dphy_write_haps(0x12, 0xD0);
	mnh_sm_mipi_tx_dphy_write_haps(0x11, 0x07);
	mnh_sm_mipi_tx_dphy_write_haps(0x19, 0x34);
	mnh_sm_mipi_tx_dphy_write_haps(0x17, 0x09);
	mnh_sm_mipi_tx_dphy_write_haps(0x18, 0x1E);
	mnh_sm_mipi_tx_dphy_write_haps(0x18, 0x87);

	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, LPCLK_CTRL,
		PHY_TXREQCLKHS_CON, 0x0);

	/* config_clk_data_timing */
	mnh_sm_mipi_tx_dphy_write_haps(0x60, 0x83);
	mnh_sm_mipi_tx_dphy_write_haps(0x61, 0x85);
	mnh_sm_mipi_tx_dphy_write_haps(0x62, 0x93);
	mnh_sm_mipi_tx_dphy_write_haps(0x63, 0x86);
	mnh_sm_mipi_tx_dphy_write_haps(0x64, 0x2B);
	mnh_sm_mipi_tx_dphy_write_haps(0x65, 0x2C);
	mnh_sm_mipi_tx_dphy_write_haps(0x70, 0x83);
	mnh_sm_mipi_tx_dphy_write_haps(0x71, 0x84);
	mnh_sm_mipi_tx_dphy_write_haps(0x72, 0x89);
	mnh_sm_mipi_tx_dphy_write_haps(0x73, 0x85);
	mnh_sm_mipi_tx_dphy_write_haps(0x74, 0x3F);

	HW_OUT(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_RSTZ, 0x07);
	udelay(1);

	i = 0;

	do {
		data = HW_IN(HWIO_MIPI_TX_BASE_ADDR_TXO, MIPI_TX, PHY_STATUS);
		udelay(10);
		i++;

	} while ((i < 20) && ((data & 0x1550) != 0x1550));


	/*
	 * ##########################################################
	 * #  Setup TX (Device)
	 * ##########################################################
	 */
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_MODE, 0x02);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_MODE, 0x02);

	return 0;
}


/***************************************************************
* GEN3 configurations
***************************************************************/
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

struct mipi_rate_config {
	uint32_t rate;
	uint32_t freq_range_code;
	uint32_t osc_freq_code;
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

static int mnh_mipi_gen3_lookup_freq_code(uint32_t rate)
{
	int i;

	if ((rate < mipi_rate_configs[0].rate) ||
	    (rate > mipi_rate_configs[ARRAY_SIZE(mipi_rate_configs)-1].rate))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(mipi_rate_configs); i++) {
		if (rate < mipi_rate_configs[i].rate)
			return i-1;
	}

	return i;
}

static void mnh_mipi_gen3_host(uint32_t device, uint32_t rate)
{
	uint32_t code_index, freq_range_code, osc_freq_code;

	pr_info("%s: dev %d, rate %d", __func__, device, rate);

	/* enable clock to controller */
	if (device == 0)
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX0_CG, 0x0);
	else
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_RX1_CG, 0x0);

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

	/*  update PHY configuration */
	if (device == 0)
		HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_DPHY_CONFIG,
		       0x8400 | freq_range_code);
	else
		HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX1_DPHY_CONFIG,
		       0x8400 | freq_range_code);


	mnh_sm_mipi_rx_dphy_write_gen3(0xe2, osc_freq_code & 0xFF, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0xe3, osc_freq_code >> 8, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0xe4, 0x01, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0x08, 0x20, device);
	udelay(1);

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, N_LANES, N_LANES, 0x3);
	udelay(1);

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, DPHY_RSTZ, DPHY_RSTZ,
		0x1);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, CSI2_RESETN,
		CSI2_RESETN, 0x1);
}

static void mnh_mipi_gen3_device(uint32_t device, uint32_t rate)
{
	uint32_t code_index, freq_range_code, osc_freq_code;

	pr_info("%s: dev %d, rate %d", __func__, device, rate);

	/* disable clock gating */
	if (device == 0) {
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_PLL_CNTRL,
			PLL_SHADOW_CONTROL, 0x1);
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_PLL_CNTRL,
			CLK_SEL, 0x1);
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_TX0_CG, 0x0);
	} else {
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_PLL_CNTRL,
			PLL_SHADOW_CONTROL, 0x1);
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_PLL_CNTRL,
			CLK_SEL, 0x1);
		HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL,
			CSI2_TX1_CG, 0x0);
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

	/* mipicsi_device_set_pll */
	/* set frequency */
	/* TODO: These values should come from a table or calculated */
	if (rate == 640) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x270, 0xE2, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x271, 0x04, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x272, 0x11, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x179, 0x7E, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x17A, 0x00, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x17B, 0x9F, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x178, 0xC8, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x15E, 0x10, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x162, 0x04, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x16E, 0x0C, device);
	} else if (rate == 1296) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x270, 0xD0, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x271, 0x07, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x272, 0x10, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x179, 0x0C, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x17A, 0x01, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x17B, 0x87, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x178, 0xC8, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x15E, 0x10, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x162, 0x04, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x16E, 0x0C, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x173, 0x02, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x174, 0x00, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x175, 0x60, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x176, 0x03, device);
	} else if (rate == 2100) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x270, 0xD0, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x271, 0x07, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x272, 0x10, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x179, 0xa2, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x17A, 0x01, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x17B, 0x87, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x178, 0xC8, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x15E, 0x10, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x162, 0x04, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x16E, 0x0C, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x173, 0x02, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x174, 0x00, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x175, 0x60, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x176, 0x03, device);
	}

	udelay(1);

	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		LANE_EN_NUM, 3);

	if (rate == 640) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x5A, 0x4C, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5B, 0x48, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5C, 0xC5, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5D, 0x03, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5E, 0x44, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5F, 0x91, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x61, 0x47, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x62, 0xC6, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x63, 0x03, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x64, 0x43, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x65, 0x85, device);
	} else if (rate == 1296) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x5A, 0x53, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5B, 0x51, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5C, 0xCA, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5D, 0x09, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5E, 0x4E, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5F, 0xA7, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x61, 0x50, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x62, 0xCC, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x63, 0x07, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x64, 0x4D, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x65, 0x8D, device);
	} else if (rate == 2100) {
		mnh_sm_mipi_tx_dphy_write_gen3(0x5A, 0x9a, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5B, 0x9b, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5C, 0xd0, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5D, 0x0e, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5E, 0x50, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x5F, 0xc0, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x61, 0x39, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x62, 0xd3, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x63, 0x0c, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x64, 0x4a, device);
		mnh_sm_mipi_tx_dphy_write_gen3(0x65, 0xa7, device);
	}

	/* This may change */
	/* TODO: these wait times should come from a table or be calculated */
	if (rate == 640)
		HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
			PHY_STOP_WAIT_TIME, 0x5);
	else if (rate == 1296)
		HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
			PHY_STOP_WAIT_TIME, 0xC03);
	else if (rate == 2100)
		HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
			PHY_STOP_WAIT_TIME, 0x1303);

	/* enable the PHY */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ, 1);
}

static int mnh_sm_mipi_bypass_gen3_init(struct mnh_mipi_config config)
{
	unsigned int long data;
	int i = 0;
	uint32_t txdev = config.txdev;
	uint32_t rxdev = config.rxdev;
	uint32_t rx_rate = config.rx_rate;
	uint32_t tx_rate = config.tx_rate;
	uint32_t vc_en_mask = config.vc_en_mask;
	uint32_t rx_mode, tx_en_mode;

	pr_info("%s: init rxdev %d, txdev %d, rx_rate %d, tx_rate %d, vc_en_mask 0x%01x",
		__func__, rxdev, txdev, rx_rate, tx_rate, vc_en_mask);

	/*
	 * ##########################################################
	 * #  mipicsi_host_hw_init
	 * ##########################################################
	 */
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, N_LANES, N_LANES, 0x0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, INT_MSK_PHY_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, INT_MSK_PKT_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, INT_MSK_FRAME_FATAL,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, INT_MSK_PHY, 0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, INT_MSK_PKT, 0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, INT_MSK_LINE,
	       0xFFFFFFFF);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);

	/*
	 * ##########################################################
	 * #  Setup RX (Host)
	 * ##########################################################
	 */
	/* mipicsi_host_start */
	mnh_mipi_gen3_host(rxdev, rx_rate);

	/*
	 * ##########################################################
	 * #  Setup TX (Device)
	 * ##########################################################
	 */
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, CSI2_TX0_CG,
		0x0);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, CSI2_TX1_CG,
		0x0);

	/* mipicsi_device_hw_init */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, PHY_RSTZ, PHY_SHUTDOWNZ,
		0x1);
	/* mipicsi_device_dphy_reset */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, PHY_RSTZ, PHY_RSTZ,
		0x1);
	udelay(1000);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, PHY_RSTZ, PHY_RSTZ,
		0x0);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, CSI2_RESETN, 0x0);
	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, CSI2_RESETN, 0x1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, INT_MASK_N_VPG,
	       0xFFFFFFFF);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, INT_MASK_N_IDI,
	       0xFFFFFFFF);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX, PHY_RSTZ, PHY_SHUTDOWNZ,
		0x0);

	/* mipicsi_device_start */
	mnh_mipi_gen3_device(txdev, tx_rate);

	i = 0;
	do {
		data = HW_IN(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX,
			     PHY_STATUS);
		udelay(10);
		i++;
	} while ((i < 20) && ((data & 0x1550) != 0x1550));

	if ((i >= 20) && ((data & 0x1550) != 0x1550))
		pr_err("Status register checked failed for gen3 device data: %ld\n",
		       data);

	/*
	 * ##########################################################
	 * #  Setup TX (Device)
	 * ##########################################################
	 */

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
			pr_err("mnh_sm: invalid rx device %d!\n", rxdev);
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
			pr_err("mnh_sm: invalid rx device %d!\n", rxdev);
		}
		break;
	default:
		pr_err("mnh_sm: invalid tx device %d!\n", txdev);
	}

	return 0;

}

int mnh_sm_mipi_bypass_init(struct mnh_mipi_config config)
{
	pr_info("%s: txdev %d, rxdev %d, rx rate %d, tx rate %d\n",
		__func__, config.txdev, config.rxdev, config.rx_rate,
		config.tx_rate);

	if (config.is_gen3 == 0)
		mnh_sm_mipi_bypass_init_haps();
	else
		mnh_sm_mipi_bypass_gen3_init(config);

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_sm_mipi_bypass_init);
