/*
* Copyright c 2016 PixArt Imaging Inc.. All Rights Reserved.

* This program is free software; you may redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include "pixart_ots.h"

/* Bit Define */
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)

/* Global Variables */

int16_t deltaX16;
int16_t deltaY16;
uint8_t OTS_ROT_Status;
uint8_t OTS_BTN_Status;

int32_t x_sum = 0;
int32_t ds_x_sum = 0;
int32_t pre_dsCountX = 0;
uint16_t OTS_BTN_Press_Cnt = 0;

/* Register write & read back check function */
void OTS_WriteRead_Reg(uint8_t address, uint8_t wdata)
{
	uint8_t rdata;

	do {
		/* Write data to specified address */
		OTS_Write_Reg(address, wdata);
		/* Read back previous written data */
		rdata = OTS_Read_Reg(address);
		/* Check if the data is correctly written */
	} while (rdata != wdata);
}

bool OTS_Sensor_Init(int check_calib, uint8_t s_c, uint8_t s_f, uint8_t btn_hi,
		uint8_t btn_lo, int16_t res_x)
{
	uint8_t sensor_pid = 0;
	bool read_id_ok = false;
	uint8_t frame_avg = 0;
	int ret = 0;

	/* Read sensor_pid in address 0x00 to check if
	the serial link is valid, PID should be 0x31 */
	sensor_pid = OTS_Read_Reg(PIXART_PAT9127_PRODUCT_ID1_REG);
	pr_debug("[PAT9127]: Sensor_pid = 0x%02x. \n", sensor_pid);

	if (sensor_pid == PIXART_PAT9127_SENSOR_ID) {
		read_id_ok = true;
		/* switch to bank0, not allowed to perform OTS_WriteRead_Reg */
		OTS_Write_Reg(PIXART_PAT9127_SELECT_BANK_REG, PIXART_PAT9127_BANK0);
		/* Software Reset (i.e. set bit7 to 1),
		then it will reset to 0 automatically */
		OTS_Write_Reg(PIXART_PAT9127_CONFIG_REG, PIXART_PAT9127_RESET);
		delay(1);	/* delay 1ms */
		/* sleep1 mode frequency and enter time */
		/* OTS_WriteRead_Reg(0x0A, 0x77); */

		ret = pat9127_enable_mot();
		if (ret != 0){
			pr_err("[PAT9127]: Enable Motion FAIL.");
		}

		OTS_WriteRead_Reg(PIXART_PAT9127_SET_CPI_RES_Y_REG,
			PIXART_PAT9127_CPI_RESOLUTION_Y);/* y-axis resolution */

		/* ONLY for VDD=VDDA=1.7~1.9V: for power saving */
		OTS_WriteRead_Reg(PIXART_PAT9127_VOLTAGE_SEGMENT_SEL_REG,
			PIXART_PAT9127_LOW_VOLTAGE_SEGMENT);

		/* The optimized values for following registers
		have to be determined from calibration procedure
		and save them to EEPROM. */
		/* These settings need be read from EEPROM and
		written to PAT9127 whenever sensor be reset or powered on. */

		frame_avg = OTS_Read_Reg(PIXART_PAT9127_FRAME_AVG_REG);
		pr_err("[PAT9127]: FA: %d, HiTh: %d, LoTh: %d, s_c: %d.\n",
			frame_avg, btn_hi, btn_lo, s_c);
		if (check_calib == 1) {
			OTS_WriteRead_Reg(PIXART_PAT9127_SHUTTER_C_REG, s_c);
			OTS_WriteRead_Reg(PIXART_PAT9127_SHUTTER_F_REG, s_f);
			OTS_WriteRead_Reg(PIXART_PAT9127_BTN_HITHD_REG, btn_hi);
			OTS_WriteRead_Reg(PIXART_PAT9127_BTN_LOTHD_REG, btn_lo);
			OTS_WriteRead_Reg(PIXART_PAT9127_SET_CPI_RES_X_REG, res_x);
		}
		else {
			OTS_WriteRead_Reg(PIXART_PAT9127_BTN_HITHD_REG, 0x00);
			OTS_WriteRead_Reg(PIXART_PAT9127_BTN_LOTHD_REG, 0x00);
		}
	}
	return read_id_ok;
}

int pat9127_disable_mot(int16_t detect_freq)
{
	uint8_t tmp_1 = 0;
	uint8_t sensor_pid = 0;

	sensor_pid = OTS_Read_Reg(PIXART_PAT9127_PRODUCT_ID1_REG);
	if (sensor_pid == PIXART_PAT9127_SENSOR_ID) {
		OTS_WriteRead_Reg(PIXART_PAT9127_SENSOR_MODE_SELECT_REG,
			PIXART_PAT9127_SENSOR_DEFAULT_MODE); // Set motion to open drain
		tmp_1 = OTS_Read_Reg(PIXART_PAT9127_SENSOR_MODE_SELECT_REG);
		pr_debug("[PAT9127]: Open drain mode motion: 0x%2x. \n", tmp_1);

		OTS_Write_Reg(PIXART_PAT9127_SELECT_BANK_REG,
			PIXART_PAT9127_SELECT_BANK_VAL2);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_FTWK,
			PIXART_PAT9127_BANK_FTWK_VAL1);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_FTWK_D2,
			PIXART_PAT9127_BANK_FTWK_D2_VAL1);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_CTB,
			PIXART_PAT9127_BANK_CTB_VAL1);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_HI_SAD_K,
			PIXART_PAT9127_BANK_HI_SAD_K_VAL1);

		OTS_Write_Reg(PIXART_PAT9127_SELECT_BANK_REG,
			PIXART_PAT9127_SELECT_BANK_VAL1);

		delay(1);				  /* delay 1ms */

		OTS_WriteRead_Reg(PIXART_PAT9127_SLEEP2_MODE_FREQ_REG,
			detect_freq);

		OTS_Write_Reg(PIXART_PAT9127_SLEEP_MODE_SELECT_REG,
			PIXART_PAT9127_FORCE_ENTER_SLEEP2_MODE);
		return 0;
	}
	else
		return (-1);
}

int pat9127_enable_mot(void)
{
	uint8_t tmp_1 = 0;
	uint8_t sensor_pid = 0;

	sensor_pid = OTS_Read_Reg(PIXART_PAT9127_PRODUCT_ID1_REG);
	if (sensor_pid == PIXART_PAT9127_SENSOR_ID) {

		OTS_WriteRead_Reg(PIXART_PAT9127_SENSOR_MODE_SELECT_REG,
				PIXART_PAT9127_SENSOR_SET_MODE); // Set motion to drive mode
		tmp_1 = OTS_Read_Reg(PIXART_PAT9127_SENSOR_MODE_SELECT_REG);
		pr_debug("[PAT9127]: Drive mode motion: 0x%2x. \n", tmp_1);

		/*Write Register for Active Mode*/
		OTS_Write_Reg(PIXART_PAT9127_SELECT_BANK_REG,
			PIXART_PAT9127_SELECT_BANK_VAL2);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_FTWK,
			PIXART_PAT9127_BANK_FTWK_DEFAULT_VAL);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_FTWK_D2,
			PIXART_PAT9127_BANK_FTWK_D2_DEFAULT_VAL);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_CTB,
			PIXART_PAT9127_BANK_CTB_DEFAULT_VAL);

		OTS_WriteRead_Reg(PIXART_PAT9127_BANK_HI_SAD_K,
			PIXART_PAT9127_BANK_HI_SAD_K_DEFAULT_VAL);

		OTS_Write_Reg(PIXART_PAT9127_SELECT_BANK_REG,
			PIXART_PAT9127_SELECT_BANK_VAL1);

		delay(1);				  /* delay 1ms */

		OTS_WriteRead_Reg(PIXART_PAT9127_SLEEP_MODE_SELECT_REG,
			PIXART_PAT9127_WAKEUP_MODE);
		tmp_1 = OTS_Read_Reg(PIXART_PAT9127_SLEEP_MODE_SELECT_REG);
		pr_debug("[PAT9127]: Enable sleep1 and disable sleep2 mode: 0x%2x. \n", tmp_1);
		return 0;
	}
	else
		return (-1);
}

/* Read motion */
uint8_t OTS_Check_MotionAndButton(int16_t *dx16, int16_t *dy16)
{
	uint8_t Reg2_Value = 0;
	uint8_t BtnState = 0;
	uint8_t OutBtnState = OTS_BTN_NO_CHANGE;

	int16_t deltaX_l = 0, deltaY_l = 0, deltaXY_h = 0;
	int16_t deltaX_h = 0, deltaY_h = 0;

	Reg2_Value = OTS_Read_Reg(PIXART_PAT9127_MOTION_STATUS_REG);
	BtnState = Reg2_Value & PIXART_PAT9127_VALID_PRE_AND_REL_DATA;

	if (BtnState == PIXART_PAT9127_VALID_PRESS_BIT_DATA) {
		OutBtnState = OTS_BTN_PRESS;
	} else if (BtnState == PIXART_PAT9127_VALID_RELEASE_DATA) {
		OutBtnState = OTS_BTN_RELEASE;
	} else if (BtnState == PIXART_PAT9127_VALID_PRE_AND_REL_DATA) {
		OutBtnState = OTS_BTN_PRE_AND_REL;
	}

	if (Reg2_Value & PIXART_PAT9127_VALID_MOTION_DATA) {
		deltaX_l = OTS_Read_Reg(PIXART_PAT9127_DELTA_X_LO_REG);
		deltaY_l = OTS_Read_Reg(PIXART_PAT9127_DELTA_Y_LO_REG);
		deltaXY_h = OTS_Read_Reg(PIXART_PAT9127_DELTA_XY_HI_REG);

		deltaX_h = (deltaXY_h<<4) & 0xF00;
		/* 12-bit data convert to 16-bit */
		if (deltaX_h & 0x800)
			deltaX_h |= 0xf000;

		deltaY_h = (deltaXY_h<<8) & 0xF00;
		/* 12-bit data convert to 16-bit */
		if (deltaY_h & 0x800)
			deltaY_h |= 0xf000;
	}

	/* inverse the data (depends on sensor's orientation and application) */
	*dx16 = -(deltaX_h | deltaX_l);
	*dy16 = -(deltaY_h | deltaY_l);

	return OutBtnState;
}

void OTS_Reset_Variables(void)
{
	/* reset variables */
	x_sum = 0;
	ds_x_sum = 0;
	pre_dsCountX = 0;
	OTS_BTN_Press_Cnt = 0;
}

static uint8_t Detect_Rotation(int32_t dsCountX)
{
	#define EVENT_NUM_PER_ROUND	60
	#define EVENT_COUNT_TH	(EXPECTED_COUNT_PER_ROUND / EVENT_NUM_PER_ROUND)

	int32_t diff_count = 0;
	uint8_t OutRotState = OTS_ROT_NO_CHANGE;

	diff_count = dsCountX - pre_dsCountX;
	if (diff_count >= EVENT_COUNT_TH) {
		pre_dsCountX = dsCountX;
		OutRotState = OTS_ROT_UP;
	} else if (diff_count <= (-EVENT_COUNT_TH)) {
		pre_dsCountX = dsCountX;
		OutRotState = OTS_ROT_DOWN;
	}
	return OutRotState;
}

static int32_t OTS_Resolution_Downscale(int16_t delta_count)
{
	int32_t ret;
	x_sum += delta_count;
	ret = (x_sum * EXPECTED_COUNT_PER_ROUND / REAL_AVG_COUNT_PER_ROUND);
	return ret;
}

uint8_t OTS_Detect_Rotation(int16_t dx16, int16_t dy16)
{
	ds_x_sum = OTS_Resolution_Downscale(dx16);
	return Detect_Rotation(ds_x_sum);
}

