/*
*
* MNH Thermal Driver
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

/* #define DEBUG */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/timer.h>

#include "mnh-efuse.h"
#include "mnh-hwio.h"
#include "mnh-hwio-bases.h"
#include "mnh-hwio-scu.h"
#include "mnh-sm.h"
#include "mnh-thermal.h"

#define DEVICE_NAME "mnh_thermal"

/* PVT operation mode configuration */
static struct mnh_thermal_op_param mnh_op_mode_table[] = {
	{0, 0}, /* temperature: vsample b0, psample b00 */
	{1, 0}, /* voltage: vsample b1, psample b00 */
	{0, 1}, /* LVT: vsample b0, psample b01 */
	{0, 2}, /* HVT: vsample b0, psample b10 */
	{0, 3}  /* SVT: vsample b0, psample b11 */
};

/* PVT dts trim efuse addresses */
static const struct mnh_efuse_addr mnh_thermal_trim_addr[] = {
	/* id0, ipu1 */
	{
	.row1 = 2,
	.row2 = 3,
	.bit_high = 9,
	.bit_low = 0,
	},
	/* id1, ipu2 */
	{
	.row1 = 2,
	.row2 = 3,
	.bit_high = 19,
	.bit_low = 10,
	},
	/* id2, cpu */
	{
	.row1 = 2,
	.row2 = 3,
	.bit_high = 29,
	.bit_low = 20,
	},
	/* id3, lpddr */
	{
	.row1 = 0,
	.row2 = 1,
	.bit_high = 9,
	.bit_low = 0,
	}
};

/**
 * This checks mnh sm state to see mnh is out of reset
 * Return: 0 if mnh is out of reset, an error code otherwise.
 */
static int check_mnh_hw_init(void)
{
	int state = mnh_sm_get_state();

	if ((state == MNH_STATE_OFF) || (state == MNH_STATE_SUSPEND))
		return -EIO;

	return 0;
}

static void config_dts_trim(struct mnh_thermal_device *mnh_dev)
{
	uint32_t i, value;
	uint32_t slope, offset;

	for (i = 0; i < MNH_NUM_PVT_SENSORS; i++) {
		value = mnh_efuse_read(&mnh_thermal_trim_addr[i]);

		/* Make sure code is 10bit */
		value = value & 0x3ff;

		/* Calculate slope and offset */
		slope =  ((unsigned)value >> API_BITS_OFFSET) &
			API_BITS_SLOPE_MASK;
		if (slope >= 16)
			slope = slope - 32;

		offset = value & API_BITS_OFFSET_MASK;
		if (offset >= 16)
			offset = offset - 32;

		dev_dbg(mnh_dev->dev, "%s: pvt%d efuse:0x%x, s:%d, o:%d\n",
			__func__, i, value, slope, offset);

		mnh_dev->sensors[i]->slope = slope;
		mnh_dev->sensors[i]->offset = offset;
	}
}

/**
 * Configure 1.2MHz clock to PVT sensor and wait for 2usecs
 * until the clock is ticking if enabled
 */
static void config_pvt_clk_en(struct mnh_thermal_device *mnh_dev, bool clk_en)
{
	dev_dbg(mnh_dev->dev, "%s: enable %d\n", __func__, clk_en);

	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PVT_CLKEN, clk_en);
	udelay(2);
}

/**
 * Check pvt clock is enabled
 * Return: 0 if pvt clk is off, 1 if pvt clk is on
 */
static uint32_t read_pvt_clk_en(struct mnh_thermal_device *mnh_dev)
{
	uint32_t val = HW_INf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PVT_CLKEN);
	return val;
}

/*
 * Caculate PVT_DATA output to millicelsius.
 *
 * Step #1 : Calculate Code to DegC using ideal equation.
 * DegC= API_Poly_N4 * Code^4 + API_Poly_N3 * Code^3 + API_Poly_N2 * Code^2
 *       + API_Poly_N1 * Code^1 + API_Poly_N0
 * Step #2 : Calculate Error using slope and offset.
 * Error= Slope * DegC + Offset
 * Step #3 : Calculate Final_temp by removing the above error from DegC
 * Final_Temp =  DegC - Error
 *
 * Note: kernel doesn't support floaing point calculation, so convert all the
 * predefined floating number to 64bit number. All variables and predefined
 * constants with an underscore "_" carry an implicit E-15 factor.
 */
static int calculate_temp(struct mnh_thermal_device *mnh_dev, int code,
			  int slope, int offset)
{
	long n1_, n2_, n3_, n4_, n0_, n01234_, final_temp_, error_;
	int temp_md;

	dev_dbg(mnh_dev->dev, "%s: pvt data:%d, slope:%d, offset:%d\n",
		__func__, code, slope, offset);

	/* Make sure code is 10bit */
	code = code & 0x3FF;

	/* Calculate degree Celcius value
	 * n4: 55bits
	 * n3: 57bits
	 * n2: 55bits
	 * n1: 56bits
	 * n0: 59bits
	 */
	n4_ = (long)code*code*code*code * API_POLY_N4;
	n3_ = (long)code*code*code * API_POLY_N3 * N3_E15_MULTIPLIER;
	n2_ = (long)code*code * API_POLY_N2 * N2_E15_MULTIPLIER;
	n1_ = (long)code * API_POLY_N1 * N1_E15_MULTIPLIER;
	n0_ = API_POLY_N0 * N0_E15_MULTIPLIER;
	n01234_ = n4_ + n3_ + n2_ + n1_ + n0_;

	/* slope = trim_slope * API_RES_SLOPE
	 * offset = trim_offset * API_RES_OFFSET
	 * error = slope * degC + offset
	 * n01234_ is 59bit, max slope is 5bit, which may overflow
	 * when multiplied without first down-scaling.
	 */
	error_ = n01234_ / RES_SLOPE_DIVIDER * slope * API_RES_SLOPE
		+ (long)offset * RES_OFFSET_E15_MULTIPLIER * API_RES_OFFSET;

	final_temp_ = n01234_ - error_;
	temp_md = (final_temp_ + N12_ROUNDING_NUM) / N12_DIVIDER;

	dev_dbg(mnh_dev->dev, "%s: deg:%ld, err:%ld, temp:%ld\n",
		__func__, n01234_, error_, final_temp_);

	return temp_md;
}

/* Read raw temperature data from sensors
 * Enable 1.2MHz Clock to PVT Sensor
 * Wait for 2 msecs for the clock to PVT sensor to start ticking
 * Program VSAMPLE/PSAMPLE
 * Enable PVT sensor access
 * Wait for conversion cycle time or PVT_DATA.DATAVALID or
 * wait for SCU interrupt
 * Read PVT_DATA[x].DATA_OUT[9:0]
 * Disable PVT sensor access
 * Reset PVT_DATA.DATAVALID
 * Disable 1.2MHz clock to PVT sensor
*/
static int mnh_thermal_get_data(void *data, int *data_out)
{
	struct mnh_thermal_sensor *sensor = (struct mnh_thermal_sensor *)data;
	struct mnh_thermal_device *mnh_dev = sensor->mnh_dev;
	u32 val = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	u32 op_mode, psample, vsample;
	uint32_t mnh_scu_base = mnh_dev->regs;
	uint32_t id = sensor->id;

	op_mode = THERMAL_OPMODE_TEMP;
	psample = mnh_op_mode_table[op_mode].psample;
	vsample = mnh_op_mode_table[op_mode].vsample;
	dev_dbg(mnh_dev->dev, "%s: pvt op:%d, vs:%d, ps:%d\n",
		__func__, op_mode, vsample, psample);

	/* Enable PVT clock */
	if (!read_pvt_clk_en(mnh_dev))
		config_pvt_clk_en(mnh_dev, 1);

	/* Program VSAMPLE/PSAMPLE for temperature evaulation */
	HW_OUTxf(mnh_scu_base, SCU, PVT_CONTROL, id, PSAMPLE, psample);
	HW_OUTxf(mnh_scu_base, SCU, PVT_CONTROL, id, VSAMPLE, vsample);

	/* Enable PVT sensor access */
	HW_OUTxf(mnh_scu_base, SCU, PVT_CONTROL, id, ENA, 1);

	/* Wait until PVT data conversion is finished in
	 * temperature and voltage mode.
	 * Conversion time: 376 PVT_SENSOR_CLK cycles
	 */
	do {
		val = HW_INxf(mnh_scu_base, SCU, PVT_DATA, id, DATAVALID);
		if (val == 1)
			break;
		udelay(mnh_dev->wait_time_us);
	} while (time_before(jiffies, timeout));

	/* Read DATA_OUT from sensor */
	val = HW_INxf(mnh_scu_base, SCU, PVT_DATA, id, DATA);

	/* Disable PVT sensor access */
	HW_OUTxf(mnh_scu_base, SCU, PVT_CONTROL, id, ENA, 0);

	/* Reset DATAVALID bit */
	HW_OUTxf(mnh_scu_base, SCU, PVT_DATA, id, DATAVALID, 1);

	*data_out = val;

	return 0;
}

/*
 * This will read raw_temp_code and convert to DecC temperature.
 */
static int mnh_thermal_get_temp(void *data, int *temp_out)
{
	struct mnh_thermal_sensor *sensor = (struct mnh_thermal_sensor *)data;
	struct mnh_thermal_device *mnh_dev = sensor->mnh_dev;
	int temp_raw = 0;
	uint32_t id = sensor->id;

	if (check_mnh_hw_init())
		return -EIO;

	/* read dts trim data if it is not configured yet */
	if (!mnh_dev->mnh_trim_init_done) {
		config_dts_trim(mnh_dev);
		mnh_dev->mnh_trim_init_done = 1;
	}

	/* Get raw temp code */
	mnh_thermal_get_data(data, &temp_raw);

	/* Convert raw code to milli DecC */
	*temp_out = calculate_temp(mnh_dev, temp_raw,
		mnh_dev->sensors[id]->slope, mnh_dev->sensors[id]->offset);

	return 0;
}

static const struct thermal_zone_of_device_ops mnh_of_thermal_ops = {
	.get_temp = mnh_thermal_get_temp,
};

static int mnh_thermal_probe(struct platform_device *pdev)
{
	struct mnh_thermal_device *mnh_dev;
	unsigned int i;
	int err;

	dev_dbg(&pdev->dev, "%s: enter\n", __func__);

	mnh_dev = devm_kzalloc(&pdev->dev, sizeof(*mnh_dev),
			GFP_KERNEL);
	if (!mnh_dev)
		return -ENOMEM;

	mnh_dev->pdev = pdev;
	mnh_dev->dev = &pdev->dev;
	mnh_dev->regs = HWIO_SCU_BASE_ADDR;
	mnh_dev->wait_time_us = PVT_WAIT_US(MNH_PRECISION);
	platform_set_drvdata(pdev, mnh_dev);

	/* Initialize thermctl sensors */
	for (i = 0; i < ARRAY_SIZE(mnh_dev->sensors); ++i) {
		struct mnh_thermal_sensor *sensor =
			devm_kzalloc(&pdev->dev, sizeof(*sensor),
			GFP_KERNEL);

		sensor->mnh_dev = mnh_dev;
		sensor->id = i;
		sensor->tzd = thermal_zone_of_sensor_register(&pdev->dev, i,
				sensor, &mnh_of_thermal_ops);

		if (IS_ERR(sensor->tzd)) {
			err = PTR_ERR(sensor->tzd);
			dev_err(mnh_dev->dev, "%s: failed to register sensor: %d\n",
				__func__, err);
			goto unregister_sensors;
		}

		mnh_dev->sensors[i] = sensor;
	}

	dev_info(&pdev->dev, "%s: initialized\n", __func__);

	return 0;

unregister_sensors:
	while (i--)
		thermal_zone_of_sensor_unregister(&pdev->dev,
			mnh_dev->sensors[i]->tzd);

	return err;
}

static int mnh_thermal_remove(struct platform_device *pdev)
{
	struct mnh_thermal_device *mnh_dev = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mnh_dev->sensors); ++i) {
		thermal_zone_of_sensor_unregister(&pdev->dev,
			mnh_dev->sensors[i]->tzd);
	}

	return 0;
}

/*
 * of_device_id structure
 */
static const struct of_device_id mnh_thermal_of_match[] = {
	{ .compatible = "intel,mnh_thermal" },
	{ }
};

MODULE_DEVICE_TABLE(of, mnh_thermal_of_match);

/*
 * Platform driver structure
 */
static struct platform_driver mnh_thermal_driver = {
	.probe = mnh_thermal_probe,
	.remove = mnh_thermal_remove,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mnh_thermal_of_match,
	},
};
module_platform_driver(mnh_thermal_driver);

MODULE_DESCRIPTION("MonetteHill Thermal Driver");
MODULE_LICENSE("GPL");
