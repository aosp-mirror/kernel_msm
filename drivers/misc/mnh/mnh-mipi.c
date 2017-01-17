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
#include "mnh-sm-config.h"


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

static void mnh_mipi_gen3_device_640(uint32_t device)
{
	pr_info("%s: dev %d", __func__, device);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP,
		TX0_DPHY_PLL_CNTRL, PLL_SHADOW_CONTROL, 0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_PLL_CNTRL, CLK_SEL,
		0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP,
		TX1_DPHY_PLL_CNTRL, PLL_SHADOW_CONTROL, 0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_PLL_CNTRL, CLK_SEL,
		0x1);

	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, CSI2_RESETN,
		CSI2_RESETN_RW, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, 0x0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x0);


	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_CONFIG, 0x8418);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_CONFIG, 0x8418);

	/* mipicsi_device_set_pll */
	/* set frequency */
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

	udelay(1);

	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		LANE_EN_NUM, 3);

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

	/* This may change */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		PHY_STOP_WAIT_TIME, 5);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ, 1);
}

static void mnh_mipi_gen3_host_640(uint32_t device)
{
	pr_info("%s: dev %d", __func__, device);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, CSI2_RX0_CG,
		0x0);

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, DPHY_RSTZ, DPHY_RSTZ,
		0x0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x0);

	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_DPHY_CONFIG, 0x8418);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX1_DPHY_CONFIG, 0x8418);

	mnh_sm_mipi_rx_dphy_write_gen3(0xe2, 0xb6, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0xe3, 0x01, device);
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


static void mnh_mipi_gen3_device_1296(uint32_t device)
{
	pr_info("%s: dev %d", __func__, device);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP,
		TX0_DPHY_PLL_CNTRL, PLL_SHADOW_CONTROL, 0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_PLL_CNTRL, CLK_SEL,
		0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP,
		TX1_DPHY_PLL_CNTRL, PLL_SHADOW_CONTROL, 0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_PLL_CNTRL, CLK_SEL,
		0x1);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, 0x0);

	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, CSI2_RESETN,
		CSI2_RESETN_RW, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, 0x0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x0);

	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_CONFIG, 0x842B);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_CONFIG, 0x842B);

	/* mipicsi_device_set_pll */
	/* set frequency */
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

	udelay(1);

	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		LANE_EN_NUM, 3);

	mnh_sm_mipi_tx_dphy_write_gen3(0x5A, 0x53, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x5B, 0x51, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x5C, 0xCA, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x5D, 0x09, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x5E, 0x4A, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x5F, 0xA7, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x61, 0x50, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x62, 0xCC, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x63, 0x07, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x64, 0x46, device);
	mnh_sm_mipi_tx_dphy_write_gen3(0x65, 0x8D, device);

	/* This may change */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		PHY_STOP_WAIT_TIME, 0xC03);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ, 1);
}

static void mnh_mipi_gen3_host_1296(uint32_t device)
{
	pr_info("%s: dev %d", __func__, device);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, CSI2_RX0_CG,
		0x0);

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, DPHY_RSTZ, DPHY_RSTZ,
		0x0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x0);

	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_DPHY_CONFIG, 0x842B);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX1_DPHY_CONFIG, 0x842B);

	mnh_sm_mipi_rx_dphy_write_gen3(0xe2, 0xB6, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0xe3, 0x01, device);
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

/* updated for 2.1Gbps [16th Jan] */
static void mnh_mipi_gen3_device_2100(uint32_t device)
{
	pr_info("%s: dev %d", __func__, device);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP,
		TX0_DPHY_PLL_CNTRL, PLL_SHADOW_CONTROL, 0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_PLL_CNTRL, CLK_SEL,
		0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP,
		TX1_DPHY_PLL_CNTRL, PLL_SHADOW_CONTROL, 0x1);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_PLL_CNTRL, CLK_SEL,
		0x1);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, 0x0);

	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, CSI2_RESETN,
		CSI2_RESETN_RW, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, 0x0);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY0_TST_CTRL0, 0x0);

	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX0_DPHY_CONFIG, 0x8441);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, TX1_DPHY_CONFIG, 0x8441);

	/* mipicsi_device_set_pll */

	/* set frequency */
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

	udelay(1);

	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		LANE_EN_NUM, 3);

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

	/* This may change */
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_IF_CFG,
		PHY_STOP_WAIT_TIME, 0x1303);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_ENABLECLK, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ,
		PHY_SHUTDOWNZ, 1);
	udelay(1);
	HW_OUTf(HWIO_MIPI_TX_BASE_ADDR(device), MIPI_TX, PHY_RSTZ, PHY_RSTZ, 1);
}

/* updated for 2.1Gbps [16th Jan] */
static void mnh_mipi_gen3_host_2100(uint32_t device)
{
	pr_info("%s: txdev %d", __func__, device);
	HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, CSI_CLK_CTRL, CSI2_RX0_CG,
		0x0);

	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_SHUTDOWNZ,
		PHY_SHUTDOWNZ, 0x0);
	HW_OUTf(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, DPHY_RSTZ, DPHY_RSTZ,
		0x0);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x1);
	udelay(1);
	HW_OUT(HWIO_MIPI_RX_BASE_ADDR(device), MIPI_RX, PHY_TEST_CTRL0, 0x0);

	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX0_DPHY_CONFIG, 0x8441);
	HW_OUT(HWIO_MIPI_TOP_BASE_ADDR, MIPI_TOP, RX1_DPHY_CONFIG, 0x8441);

	mnh_sm_mipi_rx_dphy_write_gen3(0xe2, 0x70, device);
	mnh_sm_mipi_rx_dphy_write_gen3(0xe3, 0x01, device);
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

int mnh_sm_mipi_bypass_gen3_init(uint32_t freq, uint32_t rxdev, uint32_t txdev)
{
	unsigned int long data;
	int i = 0;

	pr_info("%s: init freq %d, rxdev %d, txdev %d", __func__, freq, rxdev,
		txdev);

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
	switch (freq) {
	case 640:
		mnh_mipi_gen3_host_640(rxdev);
		break;
	case 1296:
		mnh_mipi_gen3_host_1296(rxdev);
		break;
	case 2100:
		mnh_mipi_gen3_host_2100(rxdev);
		break;
	default:
		pr_err("mnh_sm: invalid host frequency %d!\n", freq);
		return -EINVAL;
	}

	do {
		data = HW_IN(HWIO_MIPI_RX_BASE_ADDR(rxdev), MIPI_RX,
			     PHY_STOPSTATE);
		udelay(10);
		i++;
	} while ((i < 20) && ((data & 0x1000F) != 0x1000F));

	if ((i < 20) && ((data & 0x1000F) != 0x1000F))
		pr_err("Status register checked failed for gen3 host data: %ld\n",
		       data);

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
	switch (freq) {
	case 640:
		mnh_mipi_gen3_device_640(txdev);
		break;
	case 1296:
		mnh_mipi_gen3_device_1296(txdev);
		break;
	case 2100:
		mnh_mipi_gen3_device_2100(txdev);
		break;
	default:
		pr_err("mnh_sm: invalid device frequency %d!\n", freq);
		return -EINVAL;
	}

	i = 0;

	do {
		data = HW_IN(HWIO_MIPI_TX_BASE_ADDR(txdev), MIPI_TX,
			     PHY_STATUS);
		udelay(10);
		i++;
	} while ((i < 20) && ((data & 0x1550) != 0x1550));

	if ((i < 20) && ((data & 0x1550) != 0x1550))
		pr_err("Status register checked failed for gen3 device data: %ld\n",
		       data);

	/*
	 * ##########################################################
	 * #  Setup TX (Device)
	 * ##########################################################
	 */
	switch (txdev) {
	case 0:
		switch (rxdev) {
		case 0:
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, RX0_MODE, RX0_BYP_TX0_EN, 1);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX0_MODE, TX0_BYP_SEL, 0x1);
			break;
		case 1:
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
			MIPI_TOP, RX1_MODE, RX1_BYP_TX0_EN, 1);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX0_MODE, TX0_BYP_SEL, 0x2);
			break;
		default:
			pr_err("mnh_sm: invalid rx device %d!\n", rxdev);
		}
		break;
	case 1:
		switch (rxdev) {
		case 0:
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, RX0_MODE, RX0_BYP_TX1_EN, 1);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX1_MODE, TX1_BYP_SEL, 0x1);
			break;
		case 1:
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, RX1_MODE, RX1_BYP_TX1_EN, 1);
			HW_OUTf(HWIO_MIPI_TOP_BASE_ADDR,
				MIPI_TOP, TX1_MODE, TX1_BYP_SEL, 0x2);
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
EXPORT_SYMBOL_GPL(mnh_sm_mipi_bypass_gen3_init);


int mnh_sm_mipi_bypass_init(struct mnh_sm_configuration *mnh_sm_boot_args)
{
	int i;
	struct mnh_mipi_conf mipi_config;
	struct mnh_tx_conf txconf;

	pr_info("%s: start\n", __func__);
	for (i = 0; i < MNH_MUX_DEVICE_TX_MAX; i++) {
		txconf = mnh_sm_boot_args->tx_configs[i];
		pr_info("%s: txconf %d, rxdev %d\n", __func__, i, txconf.rxdev);
		if (txconf.conf_sel < 0)
			continue;
		mipi_config =
			mnh_sm_boot_args->mipi_configs[txconf.conf_sel];
		pr_info("%s: mipi_config %d, freq %d\n", __func__,
			txconf.conf_sel, mipi_config.freq);
		if (mipi_config.is_gen3 == 0)
			mnh_sm_mipi_bypass_init_haps();
		else {
			mnh_sm_mipi_bypass_gen3_init(mipi_config.freq,
						     txconf.rxdev,
						     i);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_sm_mipi_bypass_init);
