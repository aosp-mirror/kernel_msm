/*
 *  stmvl53l0.h - Linux kernel modules for STM VL53L0 FlightSense Time-of-Flight sensor
 *
 *  Copyright (C) 2014 STMicroelectronics Imaging Division
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
/*
 * Defines
 */
#define STMVL53L0_DRV_NAME	"stmvl53l0"

enum laser_scan {
	LASER_SCAN_ID = 0,
	LASER_SCAN_X,
	LASER_SCAN_Y,
	LASER_SCAN_Z,
	LASER_SCAN_XX,
	LASER_SCAN_YY,
	LASER_SCAN_ZZ,
	LASER_SCAN_TIMESTAMP,
};

enum laser_type {
	LASER_RANGE_DATA	= 0,
	LASER_META_DATA		= 1
};

#if 0
#define DRIVER_VERSION		"1.0"
#define I2C_M_WR			0x00
#define INT_POLLING_DELAY	5


//if don't want to have output from vl6180_dbgmsg, comment out #DEBUG macro
#define DEBUG
//#define vl6180_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)
#define vl6180_dbgmsg(str, args...) printk("%s: " str, __func__, ##args)



/**
 * VL6180 register addresses
 */
//Device Registers
#define VL6180_MODEL_ID_REG			    	0x0000
#define VL6180_MODEL_REV_MAJOR_REG		    0x0001
#define VL6180_MODEL_REV_MINOR_REG		    0x0002
#define VL6180_MODULE_REV_MAJOR_REG		    0x0003
#define VL6180_MODULE_REV_MINOR_REG		    0x0004

#define VL6180_REVISION_ID_REG			    0x0005
#define VL6180_REVISION_ID_REG_BYTES		1
#define VL6180_DATE_HI_REG			    	0x0006
#define VL6180_DATE_HI_REG_BYTES		    1
#define VL6180_DATE_LO_REG			    	0x0007
#define VL6180_DATE_LO_REG_BYTES	   	    1
#define VL6180_TIME_REG			    	    0x0008
#define VL6180_TIME_REG_BYTES			    2
#define VL6180_CODE_REG			    	    0x000a
#define VL6180_CODE_REG_BYTES			    1
#define VL6180_FIRMWARE_REVISION_ID_REG	    	    0x000b
#define VL6180_FIRMWARE_REVISION_ID_REG_BYTES	    1

// Result Registers
#define VL6180_RESULT__RANGE_RAW_REG                                0x0064
#define VL6180_RESULT__RANGE_RAW_REG_BYTES                          1
#define VL6180_RESULT__RANGE_RETURN_RATE_REG                        0x0066
#define VL6180_RESULT__RANGE_RETURN_RATE_REG_BYTES                  2
#define VL6180_RESULT__RANGE_REFERENCE_RATE_REG                     0x0068
#define VL6180_RESULT__RANGE_REFERENCE_RATE_REG_BYTES               2
#define VL6180_RESULT__RANGE_RETURN_VCSEL_COUNT_REG                 0x006c
#define VL6180_RESULT__RANGE_RETURN_VCSEL_COUNT_REG_BYTES           4           
#define VL6180_RESULT__RANGE_REFERENCE_VCSEL_COUNT_REG              0x0070
#define VL6180_RESULT__RANGE_REFERENCE_VCSEL_COUNT_REG_BYTES        4
#define VL6180_RESULT__RANGE_RETURN_AMB_COUNT_REG                   0x0074
#define VL6180_RESULT__RANGE_RETURN_AMB_COUNT_REG_BYTES             4
#define VL6180_RESULT__RANGE_REFERENCE_AMB_COUNT_REG                0x0078
#define VL6180_RESULT__RANGE_REFERENCE_AMB_COUNT_REG_BYTES          4
#define VL6180_RESULT__RANGE_RETURN_CONV_TIME_REG                   0x007c
#define VL6180_RESULT__RANGE_RETURN_CONV_TIME_REG_BYTES             4
#define VL6180_RESULT__RANGE_REFERENCE_CONV_TIME_REG                0x0080
#define VL6180_RESULT__RANGE_REFERENCE_CONV_TIME_REG_BYTES          4




/*
 *  driver data structs
 */
struct stmvl6180_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct delayed_work	dwork;		/* for PS  work handler */
	struct input_dev *input_dev_ps;

	int 	irq;
	unsigned int enable;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;

	/* PS parameters */
	unsigned int ps_is_singleshot;
	unsigned int ps_data;			/* to store PS data */
	unsigned int enable_distance_filter;

	/* Range Data */
	//RangeData rangeData;
	//sensor_RowRangeData rangeData;
	VL6180x_RangeData_t rangeData;

	struct mutex work_mutex;
	unsigned int ps_count;
	/* Debug */
	unsigned int enableDebug;
};

static char* vl6180_error_stings[30] = {
        "NoError",                /*!< 0  0b0000 NoError  */
        "VCSEL_Continuity_Test",     /*!< 1  0b0001 VCSEL_Continuity_Test */
        "VCSEL_Watchdog_Test",       /*!< 2  0b0010 VCSEL_Watchdog_Test */
        "VCSEL_Watchdog",            /*!< 3  0b0011 VCSEL_Watchdog */
        "PLL1_Lock",                 /*!< 4  0b0100 PLL1_Lock */
        "PLL2_Lock",                 /*!< 5  0b0101 PLL2_Lock */
        "Early_Convergence_Estimate",/*!< 6  0b0110 Early_Convergence_Estimate */
        "Max_Convergence",           /*!< 7  0b0111 Max_Convergence */
        "No_Target_Ignore",          /*!< 8  0b1000 No_Target_Ignore */
        "Not_used_9",                /*!< 9  0b1001 Not_used */
        "Not_used_10",               /*!< 10 0b1010 Not_used_ */
        "Max_Signal_To_Noise_Ratio", /*!< 11 0b1011 Max_Signal_To_Noise_Ratio*/
        "Raw_Ranging_Algo_Underflow",/*!< 12 0b1100 Raw_Ranging_Algo_Underflow*/
        "Raw_Ranging_Algo_Overflow", /*!< 13 0b1101 Raw_Ranging_Algo_Overflow */
        "Ranging_Algo_Underflow",    /*!< 14 0b1110 Ranging_Algo_Underflow */
        "Ranging_Algo_Overflow",     /*!< 15 0b1111 Ranging_Algo_Overflow */

        /* code below are addition for API/software side they are not hardware*/
        "RangingFiltered"     /*!< 16 0b10000 filtered by post processing*/
};
#endif
