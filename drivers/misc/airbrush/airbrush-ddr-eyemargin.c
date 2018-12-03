/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Eye Margin tool.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "airbrush-ddr.h"
#include "airbrush-ddr-internal.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"

#define MAX_RW_OFFSETS		(512)
#define MR_READ_DELAY_USEC	(100)

static void ddrphy_set_read_offset(int offset_phy)
{
	uint32_t ctrl_offsetr;

	if (offset_phy < 0)
		ctrl_offsetr = 0x100 | ((offset_phy * -1) & 0xff);
	else
		ctrl_offsetr = offset_phy;

	ddr_reg_clr_set(DPHY_OFFSETR_CON0,
			CTRL_OFFSETR0_MSK | CTRL_OFFSETR1_MSK,
			CTRL_OFFSETR0(ctrl_offsetr) |
			CTRL_OFFSETR1(ctrl_offsetr));

	ddr_reg_clr_set(DPHY2_OFFSETR_CON0,
			CTRL_OFFSETR0_MSK | CTRL_OFFSETR1_MSK,
			CTRL_OFFSETR0(ctrl_offsetr) |
			CTRL_OFFSETR1(ctrl_offsetr));

	/* When CTRL_RESYNC bit transits from LOW to HIGH, pointers of FIFO
	 * within PHY and all of the DLL information (Read/Write/CA/CS DLL)
	 * is updated.
	 */
	ddr_reg_clr(DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(DPHY_OFFSETD_CON0, CTRL_RESYNC);

	ddr_reg_clr(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
}

static void ddrphy_set_write_offset(int offset_phy)
{
	uint32_t ctrl_offsetw;

	if (offset_phy < 0)
		ctrl_offsetw = 0x100 | ((offset_phy * -1) & 0xff);
	else
		ctrl_offsetw = offset_phy;

	ddr_reg_clr_set(DPHY_OFFSETW_CON0,
			CTRL_OFFSETW0_MSK | CTRL_OFFSETW1_MSK,
			CTRL_OFFSETW0(ctrl_offsetw) |
			CTRL_OFFSETW1(ctrl_offsetw));

	ddr_reg_clr_set(DPHY2_OFFSETW_CON0,
			CTRL_OFFSETW0_MSK | CTRL_OFFSETW1_MSK,
			CTRL_OFFSETW0(ctrl_offsetw) |
			CTRL_OFFSETW1(ctrl_offsetw));

	/* When CTRL_RESYNC bit transits from LOW to HIGH, pointers of FIFO
	 * within PHY and all of the DLL information (Read/Write/CA/CS DLL)
	 * is updated.
	 */
	ddr_reg_clr(DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(DPHY_OFFSETD_CON0, CTRL_RESYNC);

	ddr_reg_clr(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_set(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(DPHY2_OFFSETD_CON0, CTRL_RESYNC);
}

static void ddrphy_measure_eye_read(int samples, uint32_t eye_data)
{
	int vrefIdx;
	int maxOffset;
	int offsetIdx, result_idx;
	static char read_eye[PHY_VREF_LEVELS][MAX_RW_OFFSETS];

	maxOffset = samples;

	for (vrefIdx = VREF_FROM; vrefIdx < PHY_VREF_LEVELS;
						vrefIdx += VREF_STEP) {
		ddrphy_set_read_vref(ddr_get_phy_vref(vrefIdx),
				     ddr_get_phy_vref(vrefIdx), VREF_BYTE_ALL);
		pr_info("SET READ_VREF: 0x%02x\n", ddr_get_phy_vref(vrefIdx));

		result_idx = 0;
		for (offsetIdx = (maxOffset * -1); offsetIdx <= maxOffset;
							offsetIdx++) {
			ddrphy_set_read_offset(offsetIdx);
			if (!ab_ddr_read_write_test(eye_data))
				read_eye[vrefIdx][result_idx] = 'o';
			else
				read_eye[vrefIdx][result_idx] = '.';

			if (offsetIdx == 0)
				read_eye[vrefIdx][result_idx] = '|';

			result_idx++;
		}

		read_eye[vrefIdx][result_idx] = '\0';
	}

	for (vrefIdx = VREF_FROM; vrefIdx < PHY_VREF_LEVELS;
				  vrefIdx += VREF_STEP)
		pr_info("VREF_0x%02x : %s\n", ddr_get_phy_vref(vrefIdx),
					      read_eye[vrefIdx]);
}

static void ddrphy_measure_eye_write(int samples, uint32_t eye_data)
{
	int vrefIdx;
	int maxOffset;
	int offsetIdx, result_idx;
	static char write_eye[DRAM_VREF_LEVELS][MAX_RW_OFFSETS];

	maxOffset = samples;

	for (vrefIdx = VREF_FROM; vrefIdx < DRAM_VREF_LEVELS;
				  vrefIdx += VREF_STEP) {
		ddrphy_set_write_vref(ddr_get_dram_vref(vrefIdx),
				      VREF_BYTE_ALL);
		pr_info("SET WRITE_VREF: 0x%02x\n", ddr_get_dram_vref(vrefIdx));

		result_idx = 0;
		for (offsetIdx = (maxOffset * -1); offsetIdx <= maxOffset;
							offsetIdx++) {
			ddrphy_set_write_offset(offsetIdx);
			if (!ab_ddr_read_write_test(eye_data))
				write_eye[vrefIdx][result_idx] = 'o';
			else
				write_eye[vrefIdx][result_idx] = '.';

			if (offsetIdx == 0)
				write_eye[vrefIdx][result_idx] = '|';

			result_idx++;
		}

		write_eye[vrefIdx][result_idx] = '\0';
	}

	for (vrefIdx = VREF_FROM; vrefIdx < DRAM_VREF_LEVELS;
				  vrefIdx += VREF_STEP)
		pr_info("VREF_0x%02x : %s\n", ddr_get_dram_vref(vrefIdx),
							write_eye[vrefIdx]);
}

int ab_ddr_measure_eye(struct ab_state_context *sc, unsigned int data)
{
	uint32_t read_vref_phy0, read_vref_phy1;
	uint32_t write_vref;
	uint32_t num_samples;
	uint32_t eye_data;

	if (!sc || !sc->ddr_data) {
		pr_err("%s, error!! Invalid AB state/ddr context\n", __func__);
		return DDR_FAIL;
	}

	/* Disable the PHY control logic clock gating for s/w margin
	 * offset settings to work
	 */
	ddr_reg_clr(DPHY_LP_CON0, PCL_PD);
	ddr_reg_clr(DPHY2_LP_CON0, PCL_PD);

	read_vref_phy0 = ddr_reg_rd(DPHY_ZQ_CON9) & 0x3f;
	read_vref_phy1 = ddr_reg_rd(DPHY2_ZQ_CON9) & 0x3f;

	/* Read the MR14 register to get the VREF(DQ) information.
	 * Read the information while DDR is in self-refresh mode.
	 */
	ab_ddr_selfrefresh_enter(sc);
	ddr_reg_wr(DREX_DIRECTCMD, 0x09011800);
	ddr_usleep(MR_READ_DELAY_USEC);
	write_vref = ddr_reg_rd(DREX_MRSTATUS) & 0x3f;
	ab_ddr_selfrefresh_exit(sc);

	/* "data" provides the information about the type of test to be run
	 * (memtester/pcie_dma) for checking ddr data integrity
	 */
	eye_data = data;
	num_samples = DDR_TEST_EYE_MARGIN_SAMPLES(eye_data);
	if (num_samples < 50)
		num_samples = 50;

	pr_info("current read vref: 0x%02x, write vref: 0x%02x\n",
		(read_vref_phy0 + read_vref_phy1) / 2, write_vref);
	pr_info("RD_EYE: sweeping vref [0x%02x -> 0x%02x]\n",
		ddr_get_phy_vref(VREF_FROM),
		ddr_get_phy_vref(PHY_VREF_LEVELS - 1));

	/* sweep read vref to get the eye margin on READ */
	ddrphy_measure_eye_read(num_samples, eye_data);

	/* Restore the working Read Vref & Offset */
	ddrphy_set_read_vref(read_vref_phy0, read_vref_phy1, VREF_BYTE_ALL);
	ddrphy_set_read_offset(0);

	pr_info("WR_EYE: sweeping vref 0x%02x -> 0x%02x\n",
		ddr_get_dram_vref(VREF_FROM),
		ddr_get_dram_vref(DRAM_VREF_LEVELS - 1));

	/* sweep write vref to get the eye margin on WRITE */
	ddrphy_measure_eye_write(num_samples, eye_data);

	/* Restore the working write Vref & Offset */
	ddrphy_set_write_vref(write_vref, VREF_BYTE_ALL);
	ddrphy_set_write_offset(0);

	/* Enable the PHY control logic clock gating after eyemargin test */
	ddr_reg_set(DPHY_LP_CON0, PCL_PD);
	ddr_reg_set(DPHY2_LP_CON0, PCL_PD);

	return DDR_SUCCESS;
}
EXPORT_SYMBOL(ab_ddr_measure_eye);
