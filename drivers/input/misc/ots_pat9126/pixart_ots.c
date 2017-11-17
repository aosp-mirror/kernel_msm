/* drivers/input/misc/ots_pat9126/pixart_ots.c
 *
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 */

#include "pixart_platform.h"
#include "pixart_ots.h"

static void ots_write_read(struct i2c_client *client, u8 address, u8 wdata)
{
	u8 read_value;

	do {
		write_data(client, address, wdata);
		read_value = read_data(client, address);
	} while (read_value != wdata);
}

bool ots_sensor_init(struct i2c_client *client, int16_t res_x)
{
	u8 sensor_pid = 0;
	bool read_id_ok = false;
	int ret = 0;

	/*
	 * Read sensor_pid in address 0x00 to check if the
	 * serial link is valid, read value should be 0x31.
	 */
	sensor_pid = read_data(client, PIXART_PAT9126_PRODUCT_ID1_REG);

	if (sensor_pid == PIXART_PAT9126_SENSOR_ID) {
		read_id_ok = true;

		/*
		 * PAT9126 sensor recommended settings:
		 * switch to bank0, not allowed to perform ots_write_read
		 */
		write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
				PIXART_PAT9126_BANK0);
		/*
		 * software reset (i.e. set bit7 to 1).
		 * It will reset to 0 automatically
		 * so perform OTS_RegWriteRead is not allowed.
		 */
		write_data(client, PIXART_PAT9126_CONFIG_REG,
				PIXART_PAT9126_RESET);

		/* delay 1ms */
		usleep_range(RESET_DELAY_US, RESET_DELAY_US + 1);

		/* disable write protect */
		ots_write_read(client, PIXART_PAT9126_WRITE_PROTECT_REG,
				PIXART_PAT9126_DISABLE_WRITE_PROTECT);
		/* set X-axis resolution (depends on application) */
		ots_write_read(client, PIXART_PAT9126_SET_CPI_RES_X_REG, res_x);
		/* set Y-axis resolution (depends on application) */
		ots_write_read(client, PIXART_PAT9126_SET_CPI_RES_Y_REG,
				PIXART_PAT9126_CPI_RESOLUTION_Y);
		/* set 12-bit X/Y data format (depends on application) */
		ots_write_read(client, PIXART_PAT9126_ORIENTATION_REG,
				PIXART_PAT9126_MOTION_DATA_LENGTH);
		/* ONLY for VDD=VDDA=1.7~1.9V: for power saving */
		ots_write_read(client, PIXART_PAT9126_VOLTAGE_SEGMENT_SEL_REG,
				PIXART_PAT9126_LOW_VOLTAGE_SEGMENT);

		ots_write_read(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG,
				PIXART_PAT9126_SENSOR_SET_MODE2);

		ots_write_read(client, PIXART_PAT9126_AE_ENABLE, PIXART_PAT9126_AE_ENABLE_VAL);
		ots_write_read(client, PIXART_PAT9126_NY_MIN, PIXART_PAT9126_NY_MIN_VAL);

		ret = ots_enable_mot(client);
		if (ret != 0){
			pr_err("[PAT9126]: Enable Motion FAIL.");
		}

		/* enable write protect */
		ots_write_read(client, PIXART_PAT9126_WRITE_PROTECT_REG,
				PIXART_PAT9126_ENABLE_WRITE_PROTECT);
	}
	return read_id_ok;
}

int ots_disable_mot(struct i2c_client *client, int16_t detect_freq)
{
	uint8_t tmp_1 = 0;
	uint8_t sensor_pid = 0;

	sensor_pid = read_data(client, PIXART_PAT9126_PRODUCT_ID1_REG);
	if (sensor_pid == PIXART_PAT9126_SENSOR_ID) {
		ots_write_read(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG,
			PIXART_PAT9126_SENSOR_DEFAULT_MODE2); // Set motion to open drain
		tmp_1 = read_data(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG);
		pr_debug("[PAT9126]: Open drain mode motion: 0x%2x. \n", tmp_1);

		/*Switch to bank1*/
		write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
			PIXART_PAT9126_SELECT_BANK_VAL2);

		ots_write_read(client, PIXART_PAT9126_BANK_FTWK,
			PIXART_PAT9126_BANK_FTWK_VAL1);

		ots_write_read(client, PIXART_PAT9126_BANK_FTWK_D2,
			PIXART_PAT9126_BANK_FTWK_D2_VAL1);

		ots_write_read(client, PIXART_PAT9126_BANK_CTB,
			PIXART_PAT9126_BANK_CTB_VAL1);

		ots_write_read(client, PIXART_PAT9126_BANK_HI_SAD_K,
			PIXART_PAT9126_BANK_HI_SAD_K_VAL1);

		write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
			PIXART_PAT9126_SELECT_BANK_VAL1);

		delay(1);				  /* delay 1ms */

		ots_write_read(client, PIXART_PAT9126_SLEEP2_MODE_FREQ_REG,
			detect_freq);

		write_data(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG,
			PIXART_PAT9126_FORCE_ENTER_SLEEP2_MODE);
		return 0;
	}
	else
		return (-1);
}

int ots_enable_mot(struct i2c_client *client)
{
	uint8_t tmp_1 = 0;
	uint8_t sensor_pid = 0;

	sensor_pid = read_data(client, PIXART_PAT9126_PRODUCT_ID1_REG);
	if (sensor_pid == PIXART_PAT9126_SENSOR_ID) {
		ots_write_read(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG,
				PIXART_PAT9126_SENSOR_SET_MODE2); // Set motion to drive mode
		tmp_1 = read_data(client, PIXART_PAT9126_SENSOR_MODE_SELECT_REG);
		pr_debug("[PAT9126]: Drive mode motion: 0x%2x. \n", tmp_1);

		delay(1);				  /* delay 1ms */

		/*Read Register for Pulling Up Motion IRQ*/
		tmp_1 = read_data(client, PIXART_PAT9126_MOTION_STATUS_REG);
		tmp_1 = read_data(client, PIXART_PAT9126_DELTA_X_LO_REG);
		tmp_1 = read_data(client, PIXART_PAT9126_DELTA_Y_LO_REG);
		tmp_1 = read_data(client, PIXART_PAT9126_DELTA_XY_HI_REG);

		/*Write Register for Active Mode*/
		/*Switch to bank1*/
		write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
			PIXART_PAT9126_SELECT_BANK_VAL2);

		ots_write_read(client, PIXART_PAT9126_BANK_FTWK,
			PIXART_PAT9126_BANK_FTWK_DEFAULT_VAL);

		ots_write_read(client, PIXART_PAT9126_BANK_FTWK_D2,
			PIXART_PAT9126_BANK_FTWK_D2_DEFAULT_VAL);

		ots_write_read(client, PIXART_PAT9126_BANK_CTB,
			PIXART_PAT9126_BANK_CTB_DEFAULT_VAL);

		ots_write_read(client, PIXART_PAT9126_BANK_HI_SAD_K,
			PIXART_PAT9126_BANK_HI_SAD_K_DEFAULT_VAL);

		write_data(client, PIXART_PAT9126_SELECT_BANK_REG,
			PIXART_PAT9126_SELECT_BANK_VAL1);

		delay(1);				  /* delay 1ms */

		ots_write_read(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG,
			PIXART_PAT9126_WAKEUP_MODE);
		tmp_1 = read_data(client, PIXART_PAT9126_SLEEP_MODE_SELECT_REG);
		pr_debug("[PAT9126]: Enable sleep1 and disable sleep2 mode: 0x%2x. \n", tmp_1);
		return 0;
	}
	else
		return (-1);
}

/* Read motion */
void ots_read_motion(struct i2c_client *client, int16_t *dx16, int16_t *dy16)
{
	int16_t deltaX_l = 0, deltaY_l = 0, deltaXY_h = 0;
	int16_t deltaX_h = 0, deltaY_h = 0;
	uint8_t motion = 0;

	motion = read_data(client, PIXART_PAT9126_MOTION_STATUS_REG);
	pr_debug("[pat9126]: Motion BIT: 0x%2x\n", motion);

	if (motion & PIXART_PAT9126_VALID_MOTION_DATA) {
		deltaX_l = read_data(client, PIXART_PAT9126_DELTA_X_LO_REG);
		deltaY_l = read_data(client, PIXART_PAT9126_DELTA_Y_LO_REG);
		deltaXY_h = read_data(client, PIXART_PAT9126_DELTA_XY_HI_REG);

		deltaX_h = (deltaXY_h<<4) & 0xF00;
		/* 12-bit data convert to 16-bit */
		if (deltaX_h & 0x800)
			deltaX_h |= 0xf000;

		deltaY_h = (deltaXY_h<<8) & 0xF00;
		/* 12-bit data convert to 16-bit */
		if (deltaY_h & 0x800)
			deltaY_h |= 0xf000;
	}

	*dx16 = deltaX_h | deltaX_l;
	*dy16 = deltaY_h | deltaY_l;
}
