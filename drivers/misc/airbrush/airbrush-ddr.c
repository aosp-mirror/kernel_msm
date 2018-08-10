/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Initialization and Training sequence.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/airbrush-sm-ctrl.h>

#include "airbrush-ddr.h"
#include "airbrush-ddr-internal.h"

unsigned long g_timer_end;
unsigned long g_timer_running;

static void ddr_timer_start(uint32_t usec)
{
	g_timer_end = jiffies + usecs_to_jiffies(usec);
	g_timer_running = 1;
}

static void ddr_timer_stop(void)
{
	g_timer_end = 0;
	g_timer_running = 0;
}

static uint32_t ddr_timer_expired(void)
{
	if (!g_timer_running)
		return 1;

	if (time_before(jiffies, g_timer_end))
		return 0;

	g_timer_end = 0;
	g_timer_running = 0;

	return 1;
}

static void ddr_sleep(uint32_t usec)
{
	usleep_range(usec, usec);
}

//for vref voltage of PHY (13.5% ~ 46.1%)
uint32_t gPhy_vref[PHY_MAX_VREF + 1] =
{
	0x0,	0x1,	0x2,	0x3,	0x4,	0x5,
	0x6,	0x7,	0x8,	0x9,	0xa,	0xb,
	0xc,	0xd,	0xe,	0xf,	0x10,	0x11,
	0x12,	0x13,	0x14,	0x15,	0x16,	0x17,
	0x18,	0x19,	0x1a,	0x1b,	0x1c,	0x1d,
	0x1e,	0x1f,	0x20,	0x21,	0x22,	0x23,
	0x24,	0x25,	0x26,	0x27,	0x28,	0x29,
	0x2a,	0x2b,	0x2c,	0x2d,	0x2e,	0x2f,
	0x30,	0x31,	0x32,	0x33,	0x34,	0x35,
	0x36,	0x37,	0x38,	0x39,	0x3a,	0x3b,
	0x3c,	0x3d,	0x3e,	0x3f
};

//for vref voltage of DRAM (10.0% ~ 30.0%)
uint32_t gDram_vref[DRAM_MAX_VREF + 1] =
{
	0x0,	0x1,	0x2,	0x3,	0x4,	0x5,
	0x6,	0x7,	0x8,	0x9,	0xa,	0xb,
	0xc,	0xd,	0xe,	0xf,	0x10,	0x11,
	0x12,	0x13,	0x14,	0x15,	0x16,	0x17,
	0x18,	0x19,	0x1a,	0x1b,	0x1c,	0x1d,
	0x1e,	0x1f,	0x20,	0x21,	0x22,	0x23,
	0x24,	0x25,	0x26,	0x27,	0x28,	0x29,
	0x2a,	0x2b,	0x2c,	0x2d,	0x2e,	0x2f,
	0x30,	0x31,	0x32
};

void ddrphy_set_read_vref(uint32_t vref_phy0, uint32_t vref_phy1, eVref_byte byte)
{
	sZQ_CON9 zq_con9;

	zq_con9.n = DDRPHY_VREF_RD32(DDRPHY0_BASE_ADDR, rZQ_CON9_offset);

	if (byte == VREF_BYTE0) {
		zq_con9.bits.zq_ds0_vref = vref_phy0;
	} else if (byte == VREF_BYTE1) {
		zq_con9.bits.zq_ds1_vref = vref_phy0;
	} else {    //VREF_BYTE_ALL or default
		zq_con9.bits.zq_ds0_vref = vref_phy0;
		zq_con9.bits.zq_ds1_vref = vref_phy0;
	}

	DDRPHY_VREF_WR32(DDRPHY0_BASE_ADDR, rZQ_CON9_offset, zq_con9.n);

	zq_con9.n = DDRPHY_VREF_RD32(DDRPHY1_BASE_ADDR, rZQ_CON9_offset);

	if (byte == VREF_BYTE0) {
		zq_con9.bits.zq_ds0_vref = vref_phy1;
	} else if (byte == VREF_BYTE1) {
		zq_con9.bits.zq_ds1_vref = vref_phy1;
	} else {    //VREF_BYTE_ALL or default
		zq_con9.bits.zq_ds0_vref = vref_phy1;
		zq_con9.bits.zq_ds1_vref = vref_phy1;
	}

	DDRPHY_VREF_WR32(DDRPHY1_BASE_ADDR, rZQ_CON9_offset, zq_con9.n);
}

void ddrphy_set_write_vref(uint32_t vref, eVref_byte byte)
{
	//issue MRW command for DRAM vref
	DDRPHY_VREF_WR32(DREX_BASE_ADDR, rDIRECTCMD_offset, (0x8c << 9) | (vref <<2));
}

uint32_t ddrphy_get_byte_svref(eVref_byte byte, ePHY phy)
{
	sZQ_CON9 zq_con9;
	int DDRPHY_BASE_ADDR = phy ? DDRPHY1_BASE_ADDR : DDRPHY0_BASE_ADDR;

	zq_con9.n = DDRPHY_VREF_RD32(DDRPHY_BASE_ADDR,rZQ_CON9_offset);

	if (byte == VREF_BYTE1)  {
		return zq_con9.bits.zq_ds1_vref;
	} else {   //VREF_BYTE0 or default
		return zq_con9.bits.zq_ds0_vref;
	}
}

void ddrphy_set_prbs_training_init(void)
{
	WR_REG(DDRPHY0_BASE_ADDR + rPRBS_CON0_offset, 0x50000);
	WR_REG(DDRPHY0_BASE_ADDR + rPRBS_CON1_offset, RD_DDR_OTP(o_PCIe_reg_address_79));

	WR_REG(DDRPHY1_BASE_ADDR + rPRBS_CON0_offset, 0x50000);
	WR_REG(DDRPHY1_BASE_ADDR + rPRBS_CON1_offset, RD_DDR_OTP(o_PCIe_reg_address_79));
}

int32_t ddrphy_run_prbs_training(eVref_op rw)
{
	sPRBS_CON0 prbs_con0;
	sCAL_CON0 cal_con0;
	uint32_t poll_multiplier;

	poll_multiplier = RD_DDR_OTP(o_SECURE_JTAG2) + 1;

	prbs_con0.n = DDRPHY_VREF_RD32(DDRPHY0_BASE_ADDR, rPRBS_CON0_offset);
	if (rw == VREF_WRITE){
		prbs_con0.bits.prbs_write_start = 0x1;
	} else { //VREF_READ or default
		prbs_con0.bits.prbs_read_start = 0x1;
	}

	DDRPHY_VREF_WR32(DDRPHY0_BASE_ADDR, rPRBS_CON0_offset,prbs_con0.n);

	prbs_con0.n = DDRPHY_VREF_RD32(DDRPHY1_BASE_ADDR, rPRBS_CON0_offset);
	if (rw == VREF_WRITE){
		prbs_con0.bits.prbs_write_start = 0x1;
	} else { //VREF_READ or default
		prbs_con0.bits.prbs_read_start = 0x1;
	}

	DDRPHY_VREF_WR32(DDRPHY1_BASE_ADDR, rPRBS_CON0_offset,prbs_con0.n);

	ddr_timer_start(VREF_PRBS_TIMEOUT_USEC * poll_multiplier);
	while ((!(RD_REG(DDRPHY0_BASE_ADDR + rPRBS_CON0_offset) & 0x1)) && !ddr_timer_expired());
	if (ddr_timer_expired()) {
		return VREF_PRBS_TIMEOUT;
	}
	ddr_timer_stop();

	ddr_timer_start(VREF_PRBS_TIMEOUT_USEC * poll_multiplier);
	while ((!(RD_REG(DDRPHY1_BASE_ADDR + rPRBS_CON0_offset) & 0x1)) && !ddr_timer_expired());
	if (ddr_timer_expired()) {
		return VREF_PRBS_TIMEOUT;
	}
	ddr_timer_stop();

	prbs_con0.n = DDRPHY_VREF_RD32(DDRPHY0_BASE_ADDR, rPRBS_CON0_offset);
	if (rw == VREF_WRITE) {
		prbs_con0.bits.prbs_write_start = 0x0;
	} else { //VREF_READ or default
		prbs_con0.bits.prbs_read_start = 0x0;
	}

	DDRPHY_VREF_WR32(DDRPHY0_BASE_ADDR, rPRBS_CON0_offset,prbs_con0.n);

	prbs_con0.n = DDRPHY_VREF_RD32(DDRPHY1_BASE_ADDR, rPRBS_CON0_offset);
	if (rw == VREF_WRITE) {
		prbs_con0.bits.prbs_write_start = 0x0;
	} else { //VREF_READ or default
		prbs_con0.bits.prbs_read_start = 0x0;
	}

	DDRPHY_VREF_WR32(DDRPHY1_BASE_ADDR, rPRBS_CON0_offset,prbs_con0.n);

	ddr_timer_start(VREF_PRBS_TIMEOUT_USEC * poll_multiplier);
	while (((RD_REG(DDRPHY0_BASE_ADDR + rPRBS_CON0_offset) & 0x1)) && !ddr_timer_expired());
	if (ddr_timer_expired()) {
		return VREF_PRBS_TIMEOUT;
	}
	ddr_timer_stop();

	ddr_timer_start(VREF_PRBS_TIMEOUT_USEC * poll_multiplier);
	while (((RD_REG(DDRPHY1_BASE_ADDR + rPRBS_CON0_offset) & 0x1)) && !ddr_timer_expired());
	if (ddr_timer_expired()) {
		return VREF_PRBS_TIMEOUT;
	}
	ddr_timer_stop();

	cal_con0.n = DDRPHY_VREF_RD32(DDRPHY0_BASE_ADDR, rCAL_CON0_offset);
	cal_con0.bits.wrlvl_mode = 0x1;
	cal_con0.bits.ca_cal_mode = 0x1;
	DDRPHY_VREF_WR32(DDRPHY0_BASE_ADDR, rCAL_CON0_offset,cal_con0.n );

	cal_con0.n = DDRPHY_VREF_RD32(DDRPHY0_BASE_ADDR, rCAL_CON0_offset);
	cal_con0.bits.wrlvl_mode = 0x0;
	cal_con0.bits.ca_cal_mode = 0x0;
	DDRPHY_VREF_WR32(DDRPHY0_BASE_ADDR, rCAL_CON0_offset,cal_con0.n );

	cal_con0.n = DDRPHY_VREF_RD32(DDRPHY1_BASE_ADDR, rCAL_CON0_offset);
	cal_con0.bits.wrlvl_mode = 0x1;
	cal_con0.bits.ca_cal_mode = 0x1;
	DDRPHY_VREF_WR32(DDRPHY1_BASE_ADDR, rCAL_CON0_offset,cal_con0.n );

	cal_con0.n = DDRPHY_VREF_RD32(DDRPHY1_BASE_ADDR, rCAL_CON0_offset);
	cal_con0.bits.wrlvl_mode = 0x0;
	cal_con0.bits.ca_cal_mode = 0x0;
	DDRPHY_VREF_WR32(DDRPHY1_BASE_ADDR, rCAL_CON0_offset,cal_con0.n );

	return VREF_PRBS_SUCCESS;
}

uint32_t ddrphy_get_prbs_training_result(eVref_byte byte, ePHY phy)
{
	uint32_t byte0 = 0 , byte1 = 0;
	sPRBS_CON6 prbs_con6;
	sPRBS_CON7 prbs_con7;
	int DDRPHY_BASE_ADDR = phy ? DDRPHY1_BASE_ADDR : DDRPHY0_BASE_ADDR;

	if (byte == VREF_BYTE0) {
		prbs_con6.n = DDRPHY_VREF_RD32(DDRPHY_BASE_ADDR, rPRBS_CON6_offset);
		prbs_con7.n = DDRPHY_VREF_RD32(DDRPHY_BASE_ADDR, rPRBS_CON7_offset);

		if (prbs_con6.bits.prbs_offset_left0 == 0x1ff)
			byte0 = 0;
		else
			byte0 = prbs_con6.bits.prbs_offset_left0;

		if (prbs_con7.bits.prbs_offset_right0 == 0x1ff)
			byte0 += 0;
		else
			byte0 += prbs_con7.bits.prbs_offset_right0;

		return byte0;

	} else if (byte == VREF_BYTE1) {
		prbs_con6.n = DDRPHY_VREF_RD32(DDRPHY_BASE_ADDR, rPRBS_CON6_offset);
		prbs_con7.n = DDRPHY_VREF_RD32(DDRPHY_BASE_ADDR, rPRBS_CON7_offset);

		if(prbs_con6.bits.prbs_offset_left1 == 0x1ff)
			byte1 = 0;
		else
			byte1 = prbs_con6.bits.prbs_offset_left1;

		if (prbs_con7.bits.prbs_offset_right1 == 0x1ff)
			byte1 += 0;
		else
			byte1 += prbs_con7.bits.prbs_offset_right1;

		return byte1;

	} else { //VREF_BYTE_ALL or default
		prbs_con6.n = DDRPHY_VREF_RD32(DDRPHY_BASE_ADDR, rPRBS_CON6_offset);
		prbs_con7.n = DDRPHY_VREF_RD32(DDRPHY_BASE_ADDR, rPRBS_CON7_offset);

		if (prbs_con6.bits.prbs_offset_left0 == 0x1ff)
			byte0 = 0;
		else
			byte0 = prbs_con6.bits.prbs_offset_left0;

		if (prbs_con7.bits.prbs_offset_right0 == 0x1ff)
			byte0 += 0;
		else
			byte0 += prbs_con7.bits.prbs_offset_right0;

		if (prbs_con6.bits.prbs_offset_left1 == 0x1ff)
			byte1 = 0;
		else
			byte1 = prbs_con6.bits.prbs_offset_left1;

		if (prbs_con7.bits.prbs_offset_right1 == 0x1ff)
			byte1 += 0;
		else
			byte1 += prbs_con7.bits.prbs_offset_right1;

		if (byte0 < byte1)
			return byte0;
		else
			return byte1;
	}
}

/* Set and Clear PRBS offset value */
void ddrphy_reset_prbs_training_result(void)
{
	WR_REG(DDRPHY0_BASE_ADDR + rCAL_CON3_offset, 0xfc7f9840);
	WR_REG(DDRPHY0_BASE_ADDR + rPRBS_CON4_offset, 0x0);
	WR_REG(DDRPHY0_BASE_ADDR + rPRBS_CON5_offset, 0x0);
	WR_REG(DDRPHY0_BASE_ADDR + rCAL_CON3_offset, 0xfc7f9800);

	WR_REG(DDRPHY1_BASE_ADDR + rCAL_CON3_offset, 0xfc7f9840);
	WR_REG(DDRPHY1_BASE_ADDR + rPRBS_CON4_offset, 0x0);
	WR_REG(DDRPHY1_BASE_ADDR + rPRBS_CON5_offset, 0x0);
	WR_REG(DDRPHY1_BASE_ADDR + rCAL_CON3_offset, 0xfc7f9800);
}

uint32_t ddrphy_sum_vref_training_prbs_result(uint32_t* vwm)
{
	uint32_t i;
	int32_t output = 0;

	for (i = 0; i < VREF_REF_NUM; i++) {
		output += vwm[i];
	}
	return output;
}

void ddrphy_shift_prbs_result(uint32_t* vwm)
{
	uint32_t vwm_idx;

	for (vwm_idx = 1 ; vwm_idx < VREF_REF_NUM ; vwm_idx++) {
		vwm[vwm_idx - 1] = vwm[vwm_idx];
	}
}

int32_t ddrphy_get_optimal_vref(eVref_op rw,eVref_byte byte)
{
	uint32_t vwm_phy0_vref[VREF_REF_NUM];
	uint32_t vwm_phy1_vref[VREF_REF_NUM];
	uint32_t vwm_sum_vref = 0;
	uint32_t vrefIdx;
	uint32_t vwm_vref_idx = 0;
	int32_t  vref_at_max_sum_phy0 = -1;
	int32_t  vref_at_max_sum_phy1 = -1;
	uint32_t max_vwm_sum_phy0_vref = 0;
	uint32_t max_vwm_sum_phy1_vref = 0;
	uint32_t maxVref;
	uint32_t vref;
	int32_t optimalVref;

	// memset(vwm_phyx_vref, 0x00, sizeof(vwm_phyx_vref));
	for (vrefIdx = 0; vrefIdx < VREF_REF_NUM ; vrefIdx++) {
		vwm_phy0_vref[vrefIdx] = 0;
		vwm_phy1_vref[vrefIdx] = 0;
	}
	//////////////////////////////////
	if (rw == VREF_WRITE)
		maxVref = DRAM_MAX_VREF;
	else
		maxVref = PHY_MAX_VREF;

	ddrphy_set_prbs_training_init();

	for (vrefIdx = VREF_FROM; vrefIdx <= maxVref ; vrefIdx += VREF_STEP) {
		vref  = (rw == VREF_WRITE) ? gDram_vref[vrefIdx] : gPhy_vref[vrefIdx];
		if(rw == VREF_READ)
			ddrphy_set_read_vref(vref, vref, byte);
		else
			ddrphy_set_write_vref(vref, byte);

		/* Set and Clear PRBS offset value */
		ddrphy_reset_prbs_training_result();

		if (ddrphy_run_prbs_training(rw) == VREF_PRBS_SUCCESS) {
			if(vrefIdx < (VREF_FROM + VREF_REF_NUM - 1)) {
				vwm_phy0_vref[vwm_vref_idx] = ddrphy_get_prbs_training_result(byte, PHY0);
				vwm_phy1_vref[vwm_vref_idx] = ddrphy_get_prbs_training_result(byte, PHY1);
				vwm_vref_idx +=1;
			} else {
				vwm_phy0_vref[vwm_vref_idx] = ddrphy_get_prbs_training_result(byte, PHY0);
				vwm_phy1_vref[vwm_vref_idx] = ddrphy_get_prbs_training_result(byte, PHY1);
				vwm_sum_vref = ddrphy_sum_vref_training_prbs_result(&vwm_phy0_vref[0]);

				if (max_vwm_sum_phy0_vref < vwm_sum_vref) {
					max_vwm_sum_phy0_vref = vwm_sum_vref;
					vref_at_max_sum_phy0 = vrefIdx;
				}
				ddrphy_shift_prbs_result(&vwm_phy0_vref[0]);

				vwm_sum_vref = ddrphy_sum_vref_training_prbs_result(&vwm_phy1_vref[0]);

				if (max_vwm_sum_phy1_vref < vwm_sum_vref) {
					max_vwm_sum_phy1_vref = vwm_sum_vref;
					vref_at_max_sum_phy1 = vrefIdx;
				}

				ddrphy_shift_prbs_result(&vwm_phy1_vref[0]);
			}
		} else {
			return VREF_ERROR;
		}
	}

	if ((vref_at_max_sum_phy0 - (VREF_REF_NUM/2)) < 0) {
		return VREF_ERROR;
	} else if ((vref_at_max_sum_phy1 - (VREF_REF_NUM/2)) < 0) {
		return VREF_ERROR;
	} else {
		optimalVref = (rw == VREF_WRITE) ?
			gDram_vref[vref_at_max_sum_phy0 - (VREF_REF_NUM/2)] :
			gPhy_vref[vref_at_max_sum_phy0 - (VREF_REF_NUM/2)];

		optimalVref += ((rw == VREF_WRITE) ?
			gDram_vref[vref_at_max_sum_phy1 - (VREF_REF_NUM/2)] << 8 :
			gPhy_vref[vref_at_max_sum_phy1 - (VREF_REF_NUM/2)]) << 8;

		return optimalVref;
	}
}

int32_t ddrphy_run_vref_training(void)
{
	int32_t optimalVref;
	uint32_t sVref_byte0_phy0_init, sVref_byte1_phy0_init;
	uint32_t sVref_byte0_phy1_init, sVref_byte1_phy1_init;
	uint32_t dVref_init;

	//The below section of code is for Read Training of both Phy's
	sVref_byte0_phy0_init = ddrphy_get_byte_svref(VREF_BYTE0, PHY0);
	sVref_byte1_phy0_init = ddrphy_get_byte_svref(VREF_BYTE1, PHY0);
	sVref_byte0_phy1_init = ddrphy_get_byte_svref(VREF_BYTE0, PHY1);
	sVref_byte1_phy1_init = ddrphy_get_byte_svref(VREF_BYTE1, PHY1);
	dVref_init = 0x5;  //MR14_VREF(DQ) value. Should save this value during DRAM initialization.

	//read vref training byte all
	optimalVref = ddrphy_get_optimal_vref(VREF_READ, VREF_BYTE_ALL);
	if (optimalVref < 0 || ((optimalVref >> 8) < 0)) {
		ddrphy_set_read_vref(sVref_byte0_phy0_init, sVref_byte0_phy1_init, VREF_BYTE0);
		ddrphy_set_read_vref(sVref_byte1_phy0_init, sVref_byte1_phy1_init, VREF_BYTE1);
		return VREF_ERROR;
	} else  {
		ddrphy_set_read_vref((optimalVref & 0xff), ((optimalVref >> 8) & 0xff), VREF_BYTE_ALL);
	}

	optimalVref = ddrphy_get_optimal_vref(VREF_WRITE, VREF_BYTE_ALL);
	if (optimalVref < 0) {
		ddrphy_set_write_vref(dVref_init, VREF_BYTE_ALL);
		return VREF_ERROR;
	} else  {
		ddrphy_set_write_vref((optimalVref & 0xff), VREF_BYTE_ALL);
	}

	ddrphy_reset_prbs_training_result();

	return VREF_SUCCESS;
}


static int ddr_register_control(__CONST ddr_reg_control_t *ddr_cfg)
{
	int index = 0;
	__CONST ddr_reg_poll_t *poll = NULL;
	__CONST ddr_reg_control_t *cfg;
	uint32_t poll_multiplier;

	poll_multiplier = RD_DDR_OTP(o_SECURE_JTAG2) + 1;

	while (ddr_cfg[index].reg) {

		cfg = &ddr_cfg[index];

		if (!(cfg->flags & FLG_NON_DIRECT)) {
			WR_REG(cfg->reg, cfg->val);
		} else if (cfg->flags & FLG_OTP) {
			WR_REG(cfg->reg, RD_DDR_OTP(cfg->val));
		} else if (cfg->flags & FLG_POLL) {
			poll = &ddr_reg_poll[cfg->val];
			if (poll->usec_timeout) {
				ddr_timer_start(poll->usec_timeout * poll_multiplier);
				while (((RD_REG(cfg->reg) & poll->mask) != poll->val) && !ddr_timer_expired());
				if (ddr_timer_expired())
					return DDR_FAIL;
				ddr_timer_stop();
			} else {
				while ((RD_REG(cfg->reg) & poll->mask) != poll->val);
			}
		} else if (cfg->flags & FLG_WAIT) {
			ddr_sleep(cfg->val * poll_multiplier);
		} else if (cfg->flags & FLG_SET) {
			WR_REG(cfg->reg, RD_REG(cfg->reg) | cfg->val);
		} else if (cfg->flags & FLG_RESET) {
			WR_REG(cfg->reg, RD_REG(cfg->reg) & (~cfg->val));
		}

		index ++;
	}

	return DDR_SUCCESS;
}

static int ddr_register_control_onstate(__CONST ddr_reg_control_t *ddr_cfg, uint32_t state_flag)
{
	int index = 0;
	__CONST ddr_reg_poll_t *poll = NULL;
	__CONST ddr_reg_control_t *cfg;
	uint32_t poll_multiplier;

	poll_multiplier = RD_DDR_OTP(o_SECURE_JTAG2) + 1;

	while (ddr_cfg[index].reg) {

		cfg = &ddr_cfg[index];

		if (cfg->flags & state_flag) {

			if (!(cfg->flags & FLG_NON_DIRECT)) {
				WR_REG(cfg->reg, cfg->val);
			} else if (cfg->flags & FLG_OTP) {
				WR_REG(cfg->reg, RD_DDR_OTP(cfg->val));
			} else if (cfg->flags & FLG_POLL) {
				poll = &ddr_reg_poll[cfg->val];
				if (poll->usec_timeout) {
					ddr_timer_start(poll->usec_timeout * poll_multiplier);
					while (((RD_REG(cfg->reg) & poll->mask) != poll->val) && !ddr_timer_expired());
					if (ddr_timer_expired())
						return DDR_FAIL;
					ddr_timer_stop();
				} else {
					while ((RD_REG(cfg->reg) & poll->mask) != poll->val);
				}
			} else if (cfg->flags & FLG_WAIT) {
				ddr_sleep(cfg->val * poll_multiplier);
			} else if (cfg->flags & FLG_SET) {
				WR_REG(cfg->reg, RD_REG(cfg->reg) | cfg->val);
			} else if (cfg->flags & FLG_RESET) {
				WR_REG(cfg->reg, RD_REG(cfg->reg) & (~cfg->val));
			}
		}

		index ++;
	}

	return DDR_SUCCESS;
}

static int do_ddr_blk_config(uint32_t inx_ddr_blk, uint32_t state_flag)
{
	if (state_flag) {
		return ddr_register_control_onstate(
			ddr_blk_reg_control[inx_ddr_blk],
			state_flag);

	} else {
		return ddr_register_control(
			ddr_blk_reg_control[inx_ddr_blk]);
	}
}

static void ddr_train_save_configuration(void)
{
	int i;

	for (i = 0; i < s_train_max_index; i++)
		g_ddr_train_save_value[i] = RD_REG(g_ddr_train_save_address[i]);
}

static void ddr_train_restore_configuration(uint32_t *ddr_train_save_value)
{
	int save_idx, restore_idx;

	WR_REG(0x105b0008, 0x112001);	/* reg_DPHY_CAL_CON1 */
	WR_REG(0x105b0014, 0x83004f);	/* reg_DPHY_CAL_CON4 */
	WR_REG(0x105b0008, 0x112000);	/* reg_DPHY_CAL_CON1 */

	WR_REG(0x105c0008, 0x112001);	/* reg_DPHY2_CAL_CON1 */
	WR_REG(0x105c0014, 0x83004f);	/* reg_DPHY2_CAL_CON4 */
	WR_REG(0x105c0008, 0x112000);	/* reg_DPHY2_CAL_CON1 */

	/* Auto DQS clean / Gate training */
	do_ddr_blk_config(b_AutoDQS_clean_Gate_training, 0);

	WR_REG(0x105b00ac, 0x55AA55AA);	/* reg_DPHY_CAL_RD_PATTERN_CON0 */
	WR_REG(0x105c00ac, 0x55AA55AA);	/* reg_DPHY2_CAL_RD_PATTERN_CON0 */

	WR_REG(0x105b0098, 0x55aa55aa);	/* reg_DPHY_CAL_WR_PATTERN_CON0 */
	WR_REG(0x105b009c, 0x55aa55aa);	/* reg_DPHY_CAL_WR_PATTERN_CON1 */
	WR_REG(0x105b00a0, 0x55aa55aa);	/* reg_DPHY_CAL_WR_PATTERN_CON2 */
	WR_REG(0x105b00a4, 0x55aa55aa);	/* reg_DPHY_CAL_WR_PATTERN_CON3 */
	WR_REG(0x105b00a8, 0x5555);	/* reg_DPHY_CAL_WR_PATTERN_CON4 */
	WR_REG(0x10580460, 0xaa55aa55);	/* reg_WRTRA_PATTERN0 */
	WR_REG(0x10580464, 0xaa55aa55);	/* reg_WRTRA_PATTERN1 */
	WR_REG(0x10580468, 0x5555);	/* reg_WRTRA_PATTERN2 */
	WR_REG(0x105c0098, 0x55aa55aa);	/* reg_DPHY2_CAL_WR_PATTERN_CON0 */
	WR_REG(0x105c009c, 0x55aa55aa);	/* reg_DPHY2_CAL_WR_PATTERN_CON1 */
	WR_REG(0x105c00a0, 0x55aa55aa);	/* reg_DPHY2_CAL_WR_PATTERN_CON2 */
	WR_REG(0x105c00a4, 0x55aa55aa);	/* reg_DPHY2_CAL_WR_PATTERN_CON3 */
	WR_REG(0x105c00a8, 0x5555);	/* reg_DPHY2_CAL_WR_PATTERN_CON4 */
	WR_REG(0x10580460, 0xaa55aa55);	/* reg_WRTRA_PATTERN0 */
	WR_REG(0x10580464, 0xaa55aa55);	/* reg_WRTRA_PATTERN1 */
	WR_REG(0x10580468, 0x5555);	/* reg_WRTRA_PATTERN2 */

	/* Restore Read Training Results */
	WR_REG(0x105b0004, 0x780806c8);	/* reg_DPHY_CAL_CON0 */
	WR_REG(0x105b0010, 0xfc7f9900);	/* reg_DPHY_CAL_CON3 */
	WR_REG(0x105c0004, 0x780806c8);	/* reg_DPHY2_CAL_CON0 */
	WR_REG(0x105c0010, 0xfc7f9900);	/* reg_DPHY2_CAL_CON3 */

	restore_idx = 0;
	for (save_idx = s_reg_DPHY_RD_DESKEW_CENTER_CS0_CON_DM;
		save_idx <= s_reg_DPHY2_RD_DQS_VWML_CS1_CON0; save_idx++) {
		WR_REG(g_ddr_train_restore_read_address[restore_idx],
			ddr_train_save_value[save_idx]);
		restore_idx++;
	}

	WR_REG(0x105b0010, 0xfc7f9800);	/* reg_DPHY_CAL_CON3 */
	WR_REG(0x105c0010, 0xfc7f9800);	/* reg_DPHY2_CAL_CON3 */

	/* Restore Write Training Results */
	WR_REG(0x105b0004, 0x780806e8);	/* reg_DPHY_CAL_CON0 */
	WR_REG(0x105b0010, 0xfc7f9a00);	/* reg_DPHY_CAL_CON3 */
	WR_REG(0x105c0004, 0x780806e8);	/* reg_DPHY2_CAL_CON0 */
	WR_REG(0x105c0010, 0xfc7f9a00);	/* reg_DPHY2_CAL_CON3 */


	restore_idx = 0;
	for (save_idx = s_reg_DPHY_WR_DESKEWC_CS0_CON0;
		save_idx <= s_reg_DPHY2_DM_DESKEWL_CS1_CON0; save_idx++) {
		WR_REG(g_ddr_train_restore_write_address[restore_idx],
			ddr_train_save_value[save_idx]);
		restore_idx++;
	}

	WR_REG(0x105b0010, 0xfc7f9800);	/* reg_DPHY_CAL_CON3 */
	WR_REG(0x105c0010, 0xfc7f9800);	/* reg_DPHY2_CAL_CON3 */

	/* Restore PRBS training result */
	WR_REG(0x105b0010, 0xfc7f9840);
	WR_REG(0x105b0694, ddr_train_save_value[s_reg_DPHY_PRBS_CON2]);
	WR_REG(0x105b0698, ddr_train_save_value[s_reg_DPHY_PRBS_CON3]);
	WR_REG(0x105b0010, 0xfc7f9800);

	WR_REG(0x105c0010, 0xfc7f9840);
	WR_REG(0x105c0694, ddr_train_save_value[s_reg_DPHY2_PRBS_CON2]);
	WR_REG(0x105c0698, ddr_train_save_value[s_reg_DPHY2_PRBS_CON3]);
	WR_REG(0x105c0010, 0xfc7f9800);

	/* Restore ctrl_lock_value_init after restoring training result */
	WR_REG(0x105b00b4, ddr_train_save_value[s_reg_DPHY_MDLL_CON1]);
	WR_REG(0x105c00b4, ddr_train_save_value[s_reg_DPHY2_MDLL_CON1]);

	WR_REG(0x105b00b4, 0x7c6310);	/* reg_DPHY_MDLL_CON1 */
	WR_REG(0x105c00b4, 0x7c6310);	/* reg_DPHY2_MDLL_CON1 */
	WR_REG(0x105b00b4, 0x5c6310);	/* reg_DPHY_MDLL_CON1 */
	WR_REG(0x105c00b4, 0x5c6310);	/* reg_DPHY2_MDLL_CON1 */

#ifndef CONFIG_DDR_VREF_DISABLE
	WR_REG(0x105b03ec, ddr_train_save_value[s_reg_DPHY_ZQ_CON9]);
	WR_REG(0x105c03ec, ddr_train_save_value[s_reg_DPHY2_ZQ_CON9]);
#endif
}

#ifdef CONFIG_DDR_BOOT_TEST
void ab_ddr_read_write_test(int bExtraRead)
{
	uint32_t ddr_addr[] = {
		0x20000000, 0x20008004, 0x20800008, 0x28000000,
		0x28008004, 0x28800008, 0x30000000, 0x30008004,
		0x30800008, 0x38000000, 0x38008004, 0x38800008,
		0x3ffffffc };

	uint32_t ddr_data[] = {
		0x12345678, 0xabcdef89, 0xa5a5a5a5, 0x12345678,
		0xabcdef89, 0xa5a5a5a5, 0x12345678, 0xabcdef89,
		0xa5a5a5a5, 0x12345678, 0xabcdef89, 0xa5a5a5a5,
		0xa5a5a5a5 };

	int i;

	for (i = 0; i < 13; i++)
		WR_DDR_REG(ddr_addr[i], ddr_data[i]);

	for (i = 0; i < 13; i++)
		printk("0x%x: 0x%x\n", ddr_addr[i], RD_DDR_REG(ddr_addr[i]));
}

#endif /* CONFIG_DDR_BOOT_TEST */

static int ab_ddr_train(uint32_t DDR_SR)
{
	/* Check RD_REG(TRN_ADDR) value and if non zero, the previous training
	 * results are stored in here. Restore the Training parameters.
	 */
	if (GET_REG_TRN_ADDR()) {

		/* Self-refresh entry sequence */
		if (do_ddr_blk_config(b_Enter_Self_Refresh_mode, 0))
			goto ddr_train_fail;

		/* Set upd_mode = 0. Taken care in other sequence */

		/* Copy training results from TRN_ADDR to PHY/DRAM */
		ddr_train_restore_configuration((uint32_t *)(uint64_t)GET_REG_TRN_ADDR());

		/* Self-refresh exit sequence */
		if (do_ddr_blk_config(b_Exit_Self_Refresh_mode, 0))
			goto ddr_train_fail;

		if (do_ddr_blk_config(b_AXI_Enable_After_All_training, 0))
			goto ddr_train_fail;

	} else {

		if (!DDR_SR) {
			/* Self-refresh entry sequence */
			if (do_ddr_blk_config(b_Enter_Self_Refresh_mode, 0))
				goto ddr_train_fail;
		}

		/* configure PLL_MIF via CMU_MIF_for_Highfreq and wait for pll lock */
		if (do_ddr_blk_config(b_config_PLL_MIF_via_CMU_MIF_for_HighFreq, 0))
			goto ddr_train_fail;

		/* for CKE High to CS delay */
		ddr_sleep(100);

		/* Enable DLL */
		if (do_ddr_blk_config(b_Enable_DLL, 0))
			goto ddr_train_fail;

		/* Power-down exit sequence */
		if (do_ddr_blk_config(b_Power_down_exit_sequence, 0))
			goto ddr_train_fail;

		if (do_ddr_blk_config(b_Set_DREX_timing_parameters, 0))
			goto ddr_train_fail;

		if (do_ddr_blk_config(b_Set_DREX_address_parameters, 0))
			goto ddr_train_fail;

		/* CA Training */
		if (do_ddr_blk_config(b_prepare_training, 0))
			goto ddr_train_fail;

		if (do_ddr_blk_config(b_CA_training, 0))
			goto ddr_train_fail;

		/* ODT Training */
		if (do_ddr_blk_config(b_ODT_training, 0))
			goto ddr_train_fail;

		/* Auto DQS clean / Gate training */
		if (do_ddr_blk_config(b_AutoDQS_clean_Gate_training, 0))
			goto ddr_train_fail;

		/* Calibrate / Level DQ for Read */
		if (do_ddr_blk_config(b_Read_DQ_Calibration, 0))
			goto ddr_train_fail;

		/* Calibrate / Level DQ for Write */
		if (do_ddr_blk_config(b_Write_DQ_Calibration, 0))
			goto ddr_train_fail;

		/* Reset the prbs_dram_act_enable */
		WR_REG(reg_DPHY_PRBS_CON8, RD_REG(reg_DPHY_PRBS_CON8) & ~(1 << 31));
		WR_REG(reg_DPHY2_PRBS_CON8, RD_REG(reg_DPHY2_PRBS_CON8) & ~(1 << 31));

		/* PRBS training init */
		if (do_ddr_blk_config(b_PRBS_training_init, 0))
			goto ddr_train_fail;

		/* PRBS training - Read */
		if (do_ddr_blk_config(b_PRBS_training_read, 0))
			goto ddr_train_fail;

		/* PRBS training - Write */
		if (do_ddr_blk_config(b_PRBS_training_write, 0))
			goto ddr_train_fail;

#ifndef CONFIG_DDR_VREF_DISABLE
		if (!DDR_SR) {
#ifdef CONFIG_ZEBU_EMULATION
			/* PRBS vref training for both DPHY0/1 */
			ddrphy_run_vref_training();
#else
			/* PRBS vref training for both DPHY0/1 */
			if (ddrphy_run_vref_training())
				goto ddr_train_fail;
#endif
			/* CA Training */
			if (do_ddr_blk_config(b_prepare_training, 0))
				goto ddr_train_fail;
			if (do_ddr_blk_config(b_CA_training, 0))
				goto ddr_train_fail;

			/* ODT Training */
			if (do_ddr_blk_config(b_ODT_training, 0))
				goto ddr_train_fail;

			/* Auto DQS clean / Gate training */
			if (do_ddr_blk_config(b_AutoDQS_clean_Gate_training, 0))
				goto ddr_train_fail;

			/* Calibrate / Level DQ for Read */
			if (do_ddr_blk_config(b_Read_DQ_Calibration, 0))
				goto ddr_train_fail;

			/* Calibrate / Level DQ for Write */
			if (do_ddr_blk_config(b_Write_DQ_Calibration, 0))
				goto ddr_train_fail;

			/* PRBS training init */
			if (do_ddr_blk_config(b_PRBS_training_init, 0))
				goto ddr_train_fail;

			/* PRBS training - Read */
			if (do_ddr_blk_config(b_PRBS_training_read, 0))
				goto ddr_train_fail;

			/* PRBS training - Write */
			if (do_ddr_blk_config(b_PRBS_training_write, 0))
				goto ddr_train_fail;
		}
#endif /* CONFIG_DDR_VREF_DISABLE */

		/* Self-refresh exit sequence */
		if (do_ddr_blk_config(b_Exit_Self_Refresh_mode, 0))
			goto ddr_train_fail;

		/* Set the prbs_dram_act_enable */
		WR_REG(reg_DPHY_PRBS_CON8, RD_REG(reg_DPHY_PRBS_CON8) | (1 << 31));
		WR_REG(reg_DPHY2_PRBS_CON8, RD_REG(reg_DPHY2_PRBS_CON8) | (1 << 31));

		if (do_ddr_blk_config(b_AXI_Enable_After_All_training, 0))
			goto ddr_train_fail;

		/* Save training results to TRN_ADDR */
		ddr_train_save_configuration();

		/* Update TRN_ADDR value */
		SET_REG_TRN_ADDR((uint32_t)(uint64_t)g_ddr_train_save_value);
	}

	/* Signal INTC_MSI_IRQ_7 */

	return DDR_SUCCESS;

ddr_train_fail:
	return DDR_FAIL;
}

static int32_t ab_ddr_init_internal_isolation(void)
{
	uint32_t DDR_SR;

	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	/* deassert PHY reset while PHY clock is gated */
	if (do_ddr_blk_config(b_deassert_PHY_reset_while_PHY_is_gated, 0))
		goto ddr_init_fail;

	/* configure PLL_MIF via CMU_MIF_for_Lowfreq and wait for pll lock */
	if (do_ddr_blk_config(b_config_PLL_MIF_via_CMU_MIF_for_LowFreq, 0))
		goto ddr_init_fail;

	/* Ungate PHY Clock */
	if (do_ddr_blk_config(b_ungate_PHY_clock, 0))
		goto ddr_init_fail;

	/* Deassert Internal Isolation
	 * SYSREG_Central_PMU ::
	 * PMU_CONTROL[0x4] :: PHY_RET_ON[7:7] = 0
	 */
	PMU_CONTROL_PHY_RET_OFF();

	/* Initialize PHY */
	if (do_ddr_blk_config(b_initialize_PHY_pre, 0))
		goto ddr_init_fail;

	if (do_ddr_blk_config(b_initialize_PHY, 0))
		goto ddr_init_fail;

	/* Initialize DFI Interface */
	if (do_ddr_blk_config(b_initialize_DFI, 0))
		goto ddr_init_fail;

	if (DDR_SR) { /* SUSPEND to ACTIVE sequence */

		/* IO Initialization for suspend */
		if (do_ddr_blk_config(b_IO_Initialization, FLG_STATE_SUSPEND))
			goto ddr_init_fail;

	} else { /* OFF to ACTIVE sequence */

		/* DRAM reset sequence */
		if (do_ddr_blk_config(b_DRAM_reset_sequence, 0))
			goto ddr_init_fail;

		/* Power-down exit sequence */
		if (do_ddr_blk_config(b_Power_down_exit_sequence, 0))
			goto ddr_init_fail;

		/* IO Initialization */
		if (do_ddr_blk_config(b_IO_Initialization, 0))
			goto ddr_init_fail;

		/* MRWs (Set VREF, ODT, etc) */
		if (do_ddr_blk_config(b_MRWs_Set_VREF_ODT_etc, 0))
			goto ddr_init_fail;

		/* DRAM ZQ Calibration */
		if (do_ddr_blk_config(b_ZQ_Calibration, 0))
			goto ddr_init_fail;
	}

	/* DDR training */
	if (ab_ddr_train(DDR_SR)) {
		goto ddr_init_fail;
	}

	/* Now M0 can enter WFI for GPIO_DDR_TRAIN or REG_DDR_TRAIN */
	return DDR_SUCCESS;

ddr_init_fail:
	/* [TODO] Anything need to be done in case of failure ?? */
	return DDR_FAIL;
}

/* Perform ddr re-training or train result restore */
void ab_ddr_train_gpio(void)
{
	uint32_t DDR_SR;

	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	while(GPIO_CKE_IN_SENSE());

	/* self-refresh exit sequence */
	do_ddr_blk_config(b_Exit_Self_Refresh_mode, 0);

	ab_ddr_train(DDR_SR);
}

/* Perform ddr re-training */
void ab_ddr_train_sysreg(void)
{
	uint32_t DDR_SR;

	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	SET_REG_TRN_ADDR(0);

	ab_ddr_train(DDR_SR);
}

/* Raise DDR_TRAIN GPIO interrupt to M0 for re-training/restore */
void ab_ddr_train_gpio_interrupt(void)
{
	/* TBD: Host specific GPIO. Get GPIO information from DT */
}

/* Raise DDR_TRAIN Sysreg interrupt to M0 for re-training */
void ab_ddr_train_sysreg_interrupt(void)
{
	/* Raise the DDR train gpio interrupt */
	WR_REG(SYSREG_REG_DDR_INIT, 0x1);

	/* TBD: Any delay required ?? */

	WR_REG(SYSREG_REG_DDR_INIT, 0x0);
}

void ab_ddr_save_training_results(void)
{
	/* TBD: Can use the PCIe DMA api's for the same */
}

void ab_ddr_restore_training_results(uint32_t sram_address)
{
	/* TBD: Can use the PCIe DMA api's for the same */
}

int32_t ab_ddr_resume(struct ab_state_context *sc)
{
	/* [TODO] Control PMIC to configure GPIOs and disable regulators
	 * ./pmic_ctrl buck2 1
	 * ./pmic_ctrl ldo4 1
	 * ./pmic_ctrl ldo5 1
	 * ./pmic_ctrl buck1 1
	 * ./pmic_ctrl ldo3 1
	 * ./pmic_ctrl ldo2 1
	 *
	 * ./pmic_pgood 1
	 * ./pmic_ddr_iso 0
	 */

	/* PCIe RESCAN/Enumeration Logic */

	ab_ddr_init(sc);
#ifdef CONFIG_DDR_BOOT_TEST
	ab_ddr_read_write_test(0);
#endif
	return DDR_SUCCESS;
}
EXPORT_SYMBOL(ab_ddr_resume);

int32_t ab_ddr_suspend(struct ab_state_context *sc)
{
	/* Self-refresh entry sequence */
	if (do_ddr_blk_config(b_Enter_Self_Refresh_mode, 0))
		goto ddr_suspend_fail;

	/* Enable the PMU Retention */
	PMU_CONTROL_PHY_RET_ON();

	/* [TODO] Control PMIC to configure GPIOs and disable regulators
	 * ./pmic_ddr_sr 1
	 * ./pmic_ddr_iso 1
	 * ./pmic_pgood 0
	 *
	 * ./pmic_ctrl ldo2 0
	 * ./pmic_ctrl ldo3 0
	 * ./pmic_ctrl buck1 0
	 * ./pmic_ctrl ldo5 0
	 * ./pmic_ctrl ldo4 0
	 * ./pmic_ctrl buck2 0
	 */

	return DDR_SUCCESS;

ddr_suspend_fail:
	return DDR_FAIL;
}
EXPORT_SYMBOL(ab_ddr_suspend);

int32_t ab_ddr_init(struct ab_state_context *sc)
{
	int32_t ret;
	int data;
	struct platform_device *pdev;
	uint32_t DDR_SR;

	/* Read the DDR_SR */
	DDR_SR = GPIO_DDR_SR();

	if (!(sc && sc->pdev))
		return DDR_FAIL;

	pdev = sc->pdev;

	/* If DDR OTP is not flashed, then use the OTP array instead */
	if (of_property_read_u32(pdev->dev.of_node, "ddr-otp-flashed", &data))
		RD_DDR_OTP = &RD_OTP_Array;
	else
		RD_DDR_OTP = &RD_OTP_Wrapper;

	ret = ab_ddr_init_internal_isolation();

#ifdef CONFIG_DDR_BOOT_TEST
	if (!DDR_SR && !ret)
		ab_ddr_read_write_test(0);
#endif
	return ret;
}
EXPORT_SYMBOL(ab_ddr_init);
