/*
 * fts.c
 *
 * FTS Capacitive touch screen controller (FingerTipS)
 *
 * Copyright (C) 2017, STMicroelectronics
 * Authors: AMG(Analog Mems Group)
 *
 *		marco.cali@st.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 */

/*!
* \file fts.h
* \brief Contains all the definitions and structs used generally by the driver
*/

#ifndef _LINUX_FTS_I2C_H_
#define _LINUX_FTS_I2C_H_

#include <linux/device.h>
#include "fts_lib/ftsSoftware.h"
#include "fts_lib/ftsHardware.h"


/****************** CONFIGURATION SECTION ******************/
/** @defgroup conf_section	 Driver Configuration Section
* Settings of the driver code in order to suit the HW set up and the application behavior
* @{
*/
//**** CODE CONFIGURATION ****
#define FTS_TS_DRV_NAME                     "fts"								///< driver name
#define FTS_TS_DRV_VERSION                  "5.1.2"								///< driver version string format
#define FTS_TS_DRV_VER						0x05010200							///< driver version u32 format

#define DEBUG																	///< define to print more logs in the kernel log and better follow the code flow

#define DRIVER_TEST																///< if defined allow to use and test special functions of the driver and fts_lib from comand shell (usefull for enginering/debug operations)

#define ENGINEERING_CODE														///< if defined allow to have some procedures at the boot or from file node to assure that touch works under any condition that usually are disabled in the MP stage of the project
#define COMPUTE_CX_ON_PHONE														///< initialization of CX memory allowed on the phone

/*#define FW_H_FILE*/			/* include the FW data as header file */
#ifdef FW_H_FILE
#define FW_SIZE_NAME myArray_size	/* FW data array size */
#define FW_ARRAY_NAME myArray		/* FW data array name */
/*#define FW_UPDATE_ON_PROBE*/		/* No delay updating FW */
#endif

#ifndef FW_UPDATE_ON_PROBE
#define LIMITS_H_FILE															///< include the Production Limit File as header file, can be commented to use a .csv file instead
#ifdef LIMITS_H_FILE
	#define LIMITS_SIZE_NAME myArray2_size										///< name of the variable in the limits header file which specified the dimension of the limits data array
	#define LIMITS_ARRAY_NAME myArray2											///< name of the variable in the limits header file which specified the limits data array
#endif
#else
//if execute fw update in the probe the limit file must be a .h
#define LIMITS_H_FILE															///< include the Production Limit File as header file, DO NOT COMMENT!
#define LIMITS_SIZE_NAME myArray2_size											///< name of the variable in the limits header file which specified the dimension of the limits data array
#define LIMITS_ARRAY_NAME myArray2												///< name of the variable in the limits header file which specified the limits data array
#endif

//#define USE_ONE_FILE_NODE														///< allow to enable/disable all the features just using one file node

#ifndef FW_UPDATE_ON_PROBE
#define EXP_FN_WORK_DELAY_MS				1000								///< time in ms elapsed after the probe to start the work which execute FW update and the Initialization of the IC
#endif

//**** END ****


//**** FEATURES USED IN THE IC ****

#define PHONE_KEY																///< enable the support of keys

#define GESTURE_MODE															///< enable the support of the gestures
#ifdef GESTURE_MODE
	#define USE_GESTURE_MASK													///< the gestures to select are referred using a gesture bitmask instead of their gesture IDs
#endif


#define CHARGER_MODE															///< enable the support to charger mode feature (comment to disable)

#define GLOVE_MODE																///< enable the support to glove mode feature (comment to disable)

#define COVER_MODE																///< enable the support to cover mode feature (comment to disable)

#define STYLUS_MODE																///< enable the support to stylus mode feature (comment to disable)

#define GRIP_MODE																///< enable the support to grip mode feature (comment to disable)


//**** END ****


//**** PANEL SPECIFICATION ****
#define X_AXIS_MIN                          0									///< min X coordinate of the display
#define Y_AXIS_MIN                          0									///< min Y coordinate of the display

#define PRESSURE_MIN                        0									///< min value of pressure reported
#define PRESSURE_MAX                        127									///< Max value of pressure reported

#define DISTANCE_MIN						0									///< min distance between the tool and the display
#define DISTANCE_MAX						127									///< Max distance between the tool and the display

#define TOUCH_ID_MAX                        10									///< Max number of simoultaneous touches reported

#define AREA_MIN                            PRESSURE_MIN						///< min value of Major/minor axis reported
#define AREA_MAX                            PRESSURE_MAX						///< Man value of Major/minor axis reported
//**** END ****

#define SKIP_PRESSURE

/**@}*/
/*********************************************************/


/*
 * Configuration mode
 *
 * bitmask which can assume the value defined as features in ftsSoftware.h or the following values
 */

/** @defgroup mode_section	 IC Status Mode
* Bitmask which keeps track of the features and working mode enabled in the IC.
* The meaning of the the LSB of the bitmask must be interpreted considering that the value defined in @link feat_opt Feature Selection Option @endlink correspond to the position of the corresponding bit in the mask
* @{
*/
#define MODE_NOTHING						0x00000000											///< nothing enabled (sense off)
#define MODE_ACTIVE(_mask, _sett)           _mask |= (SCAN_MODE_ACTIVE<<24)|(_sett<<16)			///< store the status of scan mode active and its setting
#define MODE_LOW_POWER(_mask, _sett)        _mask |= (SCAN_MODE_LOW_POWER<<24)|(_sett<<16)		///< store the status of scan mode low power and its setting

/** @}*/

#define CMD_STR_LEN							32									///< max number of parameters that can accept the MP file node (stm_fts_cmd)

#define TSP_BUF_SIZE						PAGE_SIZE							///< max number of bytes printable on the shell in the normal file nodes


/**
 * Struct which contains information about the HW platform and set up
 */
struct fts_hw_platform_data {
	int (*power) (bool on);
	int switch_gpio; /* (optional) I2C switch */
	int irq_gpio;																///< number of the gpio associated to the interrupt pin
	int reset_gpio;																///< number of the gpio associated to the reset pin
	const char *vdd_reg_name;													///< name of the VDD regulator
	const char *avdd_reg_name;													///< name of the AVDD regulator
	int x_axis_max;
	int y_axis_max;
};

/*
 * Forward declaration
 */
struct fts_ts_info;
extern char tag[8];																///< forward the definition of the label used to print the log in the kernel log

/*
 * Dispatch event handler
 */
typedef void (*event_dispatch_handler_t)
                                (struct fts_ts_info *info, unsigned char *data);

/**
 * FTS capacitive touch screen device information
 * - dev             Pointer to the structure device \n
 * - client          client structure \n
 * - input_dev       Input device structure \n
 * - work            Work thread \n
 * - event_wq        Event queue for work thread \n
 * - event_dispatch_table  Event dispatch table handlers \n
 * - attrs           SysFS attributes \n
 * - mode            Device operating mode (bitmask) \n
 * - touch_id        Bitmask for touch id (mapped to input slots) \n
 * - stylus_id       Bitmask for tracking the stylus touches (mapped using the touchId) \n
 * - timer           Timer when operating in polling mode \n
 * - power           Power on/off routine \n
 * - board           HW info retrieved from device tree \n
 * - vdd_reg         DVDD power regulator \n
 * - avdd_reg        AVDD power regulator \n
 * - resume_bit      Indicate if screen off/on \n
 * - fwupdate_stat   Store the result of a fw update triggered by the host \n
 * - notifier        Used for be notified from a suspend/resume event \n
 * - sensor_sleep    true suspend was called, false resume was called \n
 * - wakesrc         Wakeup Source struct \n
 * - input_report_mutex  mutex for handling the pressure of keys \n
 * - series_of_switches  to store the enabling status of a particular feature from the host \n
 */
struct fts_ts_info{
	struct device		*dev;		/* Pointer to the device */
#ifdef I2C_INTERFACE
	struct i2c_client	*client;	/* I2C client structure */
#else
	struct spi_device	*client;	/* SPI client structure */
#endif
	struct input_dev	*input_dev;	/* Input device structure */

	struct work_struct	work;		/* Event work thread */
	struct work_struct	suspend_work;	/* Suspend work thread */
	struct work_struct	resume_work;	/* Resume work thread */
	struct workqueue_struct	*event_wq;	/* Used for event handler, */
						/* suspend, resume threads */

#ifndef FW_UPDATE_ON_PROBE
	struct delayed_work	fwu_work;	/* Work for fw update */
	struct workqueue_struct	*fwu_workqueue;	/* Fw update work queue */
#endif
	event_dispatch_handler_t *event_dispatch_table; /* Dispatch table */

	struct attribute_group	attrs;		/* SysFS attributes */

	unsigned int		mode;		/* Device operating mode */
						/* MSB - active or lpm */
	unsigned long		touch_id;	/* Bitmask for touch id */
#ifdef STYLUS_MODE
	unsigned long		stylus_id;	/* Bitmask for the stylus */
#endif

	struct fts_hw_platform_data	*board;	/* HW info from device tree */
	struct regulator	*vdd_reg;	/* DVDD power regulator */
	struct regulator	*avdd_reg;	/* AVDD power regulator */

	int			resume_bit;	/* Indicate if screen off/on */
	int			fwupdate_stat;	/* Result of a fw update */

	struct notifier_block	notifier;	/* Notify on suspend/resume */
	bool			sensor_sleep;	/* True if suspend called */
	struct wakeup_source	wakesrc;	/* Wake Lock struct */

	/* input lock */
	struct mutex	input_report_mutex;	/* Mutex for pressure report */

	/* switches for features */
	int		gesture_enabled;	/* Gesture during suspend */
	int		glove_enabled;		/* Glove mode */
	int		charger_enabled;	/* Charger mode */
	int		stylus_enabled;		/* Stylus mode */
	int		cover_enabled;		/* Cover mode */
	int		grip_enabled;		/* Grip mode */
};

int fts_chip_powercycle(struct fts_ts_info *info);
extern int input_register_notifier_client(struct notifier_block * nb);
extern int input_unregister_notifier_client(struct notifier_block * nb);

//export declaration of functions in fts_proc.c
extern int fts_proc_init(void);
extern int fts_proc_remove(void);

#endif
