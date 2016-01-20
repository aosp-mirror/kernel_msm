/* CwMcuSensor.c - driver file for HTC SensorHUB
 *
 * Copyright (C) 2014 HTC Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __CWMCUSENSOR_H__
#define __CWMCUSENSOR_H__
#include <linux/ioctl.h>

#define CWMCU_I2C_NAME "CwMcuSensor"

enum ABS_status {
	CW_SCAN_ID = 0,
	CW_SCAN_X,
	CW_SCAN_Y,
	CW_SCAN_Z,
	CW_SCAN_XX,
	CW_SCAN_YY,
	CW_SCAN_ZZ,
	CW_SCAN_TIMESTAMP,
};


typedef enum {
	CW_ACCELERATION                = 0,
	CW_MAGNETIC                    = 1,
	CW_GYRO                        = 2,
	CW_LIGHT                       = 3,
	CW_PROXIMITY                   = 4,
	CW_PRESSURE                    = 5,
	CW_ORIENTATION                 = 6,
	CW_ROTATIONVECTOR              = 7,
	CW_LINEARACCELERATION          = 8,
	CW_GRAVITY                     = 9,
	CW_MAGNETIC_UNCALIBRATED       = 16,
	CW_GYROSCOPE_UNCALIBRATED      = 17,
	CW_GAME_ROTATION_VECTOR        = 18,
	CW_GEOMAGNETIC_ROTATION_VECTOR = 19,
	CW_SIGNIFICANT_MOTION          = 20,
	CW_STEP_DETECTOR               = 21,
	CW_STEP_COUNTER                = 22,
	HTC_GESTURE_MOTION             = 24,
	HTC_ANY_MOTION                 = 28,
	CW_SENSORS_ID_FW /* Be careful, do not exceed 31, Firmware ID limit */,
	CW_ACCELERATION_W                = 32,
	CW_MAGNETIC_W                    = 33,
	CW_GYRO_W                        = 34,
	CW_PRESSURE_W                    = 37,
	CW_ORIENTATION_W                 = 38,
	CW_ROTATIONVECTOR_W              = 39,
	CW_LINEARACCELERATION_W          = 40,
	CW_GRAVITY_W                     = 41,
	CW_MAGNETIC_UNCALIBRATED_W       = 48,
	CW_GYROSCOPE_UNCALIBRATED_W      = 49,
	CW_GAME_ROTATION_VECTOR_W        = 50,
	CW_GEOMAGNETIC_ROTATION_VECTOR_W = 51,
	CW_STEP_DETECTOR_W               = 53,
	CW_STEP_COUNTER_W                = 54,
	CW_SENSORS_ID_TOTAL              = 55, /* Includes Wake up version */
	TIME_DIFF_EXHAUSTED            = 97,
	CW_TIME_BASE                   = 98,
	CW_META_DATA                   = 99,
	CW_MAGNETIC_UNCALIBRATED_BIAS  = 100,
	CW_GYROSCOPE_UNCALIBRATED_BIAS = 101,
} CW_SENSORS_ID;


typedef enum {
	CWSTM32_ENABLE_REG                               = 0x01,
	CWSTM32_INT_ST1                                  = 0x08,
	CWSTM32_INT_ST2                                  = 0x09,
	CWSTM32_INT_ST3                                  = 0x0A,
	CWSTM32_INT_ST4                                  = 0x0B,

	FIRMWARE_VERSION                                 = 0x10,
	CW_I2C_REG_MCU_TIME                              = 0x11,
	CW_I2C_REG_FLASH_CHECKSUM                        = 0x12,
	CW_I2C_REG_REBOOT_MODE                           = 0x13,
	TOUCH_SOLUTION_REGISTER                          = 0x14,
	FIRMWARE_INFO                                    = 0x15,
	CW_I2C_REG_DISPLAY_STATE                         = 0x16,
	CW_I2C_REG_DOTVIEW_STATUS                        = 0x17,
	HTC_SYSTEM_STATUS_REG                            = 0x1E,
	CWSTM32_ERR_ST                                   = 0x1F,

	CWSTM32_READ_Acceleration                        = 0x20,
	CWSTM32_READ_Magnetic                            = 0x21,
	CWSTM32_READ_Gyro                                = 0x22,
	CWSTM32_READ_Light                               = 0x23,
	CWSTM32_READ_Proximity                           = 0x24,
	CWSTM32_READ_Pressure                            = 0x25,
	CWSTM32_READ_Orientation                         = 0x26,
	CWSTM32_READ_RotationVector                      = 0x27,
	CWSTM32_READ_LinearAcceleration                  = 0x28,
	CWSTM32_READ_Gravity                             = 0x29,

	CWSTM32_READ_MAGNETIC_UNCALIBRATED               = 0x30,
	CWSTM32_READ_GYROSCOPE_UNCALIBRATED              = 0x31,
	CWSTM32_READ_GAME_ROTATION_VECTOR                = 0x32,
	CWSTM32_READ_GEOMAGNETIC_ROTATION_VECTOR         = 0x33,
	CWSTM32_READ_SIGNIFICANT_MOTION                  = 0x34,
	CWSTM32_READ_STEP_DETECTOR                       = 0x35,
	CWSTM32_READ_STEP_COUNTER                        = 0x36,
	CWSTM32_READ_Gesture_Motion                      = 0x38,
	CWSTM32_READ_Any_Motion                          = 0x3F,

	CWSTM32_BATCH_MODE_COMMAND                       = 0x40,
	/* Bit 2: Timeout flag (R/WC), Bit 4: Buffer full flag (R/WC) */
	CW_BATCH_ENABLE_REG                              = 0x41,
	CWSTM32_BATCH_MODE_DATA_QUEUE                    = 0x45,
	CWSTM32_BATCH_MODE_TIMEOUT                       = 0x46,
	CWSTM32_BATCH_MODE_DATA_COUNTER                  = 0x47,
	CWSTM32_BATCH_FLUSH                              = 0x48,

	CW_WAKE_UP_BATCH_ENABLE_REG                      = 0x51,
	CWSTM32_WAKE_UP_BATCH_MODE_DATA_QUEUE            = 0x55,
	CWSTM32_WAKE_UP_BATCH_MODE_TIMEOUT               = 0x56,
	CWSTM32_WAKE_UP_BATCH_MODE_DATA_COUNTER          = 0x57,

	CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_ACC         = 0x60,
	G_SENSORS_STATUS                                 = 0x60,
	CW_I2C_REG_GSENSOR_HW_ID                         = 0x63,
	GENSOR_POSITION                                  = 0x65,
	ACCE_UPDATE_RATE                                 = 0x66,
	ACCE_WAKE_UPDATE_RATE                            = 0x67,
	CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_ACC       = 0x68,
	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC       = 0x68,
	//TODO: remove below two if we don't need g-sensor L/R calibration
	CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC         = 0x69,
	CW_I2C_REG_SENSORS_CALIBRATOR_RESULT_RL_ACC      = 0x6A,

	CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_MAG         = 0x70,
	ECOMPASS_SENSORS_STATUS                          = 0x70,
	CW_I2C_REG_COMPASS_HW_ID                         = 0x73,
	COMPASS_POSITION                                 = 0x75,
	MAGN_UPDATE_RATE                                 = 0x76,
	MAGN_WAKE_UPDATE_RATE                            = 0x77,
	CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_MAG       = 0x78,
	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_MAG       = 0x78,
	CW_I2C_REG_SENSORS_ACCURACY_MAG                  = 0x79,

	CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_GYRO        = 0x80,
	GYRO_SENSORS_STATUS                              = 0x80,
	CW_I2C_REG_GYRO_HW_ID                            = 0x83,
	GYRO_POSITION                                    = 0x85,
	GYRO_UPDATE_RATE                                 = 0x86,
	GYRO_WAKE_UPDATE_RATE                            = 0x87,
	CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_GYRO      = 0x88,
	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO      = 0x88,

	CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_LIGHT       = 0x90,
	LIGHT_SENSORS_STATUS                             = 0x90,
	LIGHT_UPDATE_PERIOD                              = 0x96,
	CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_LIGHT     = 0x98,
	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT     = 0x98,
	LIGHT_SENSORS_CALIBRATION_DATA                   = 0x98,
	CW_I2C_REG_SENSORS_SET_LEVEL_LIGHT               = 0x99,

	PROXIMITY_SENSORS_STATUS                         = 0xA0,
	PROXIMITY_SENSORS_CALIBRATION_DATA               = 0xA8,
	CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY = 0xA8,
	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY = 0xA8,

	CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_PRESSURE    = 0xB0,
	PRESSURE_UPDATE_RATE                             = 0xB6,
	PRESSURE_WAKE_UPDATE_RATE                        = 0xB7,
	CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PRESSURE  = 0xB8,
	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE  = 0xB8,

	ORIE_UPDATE_RATE                                 = 0xC0,
	ROTA_UPDATE_RATE                                 = 0xC1,
	LINE_UPDATE_RATE                                 = 0xC2,
	GRAV_UPDATE_RATE                                 = 0xC3,
	MAGN_UNCA_UPDATE_RATE                            = 0xC4,
	GYRO_UNCA_UPDATE_RATE                            = 0xC5,
	GAME_ROTA_UPDATE_RATE                            = 0xC6,
	GEOM_ROTA_UPDATE_RATE                            = 0xC7,
	SIGN_UPDATE_RATE                                 = 0xC8,
	ORIE_WAKE_UPDATE_RATE                            = 0xC9,
	ROTA_WAKE_UPDATE_RATE                            = 0xCA,
	LINE_WAKE_UPDATE_RATE                            = 0xCB,
	GRAV_WAKE_UPDATE_RATE                            = 0xCC,
	MAGN_UNCA_WAKE_UPDATE_RATE                       = 0xCD,
	GYRO_UNCA_WAKE_UPDATE_RATE                       = 0xCE,
	GAME_ROTA_WAKE_UPDATE_RATE                       = 0xCF,

	CW_CPU_STATUS_REG                                = 0xD1,
	GEOM_ROTA_WAKE_UPDATE_RATE                       = 0xD2,
	STEP_COUNTER_UPDATE_PERIOD                       = 0xD3,
	GESTURE_MOTION_UPDATE_ATTRIBUTE                  = 0xDF,

	CW_I2C_REG_EVENT_SIZE                            = 0xE4,
	CW_I2C_REG_EVENT_DATA                            = 0xE5,
	CW_I2C_REG_WATCHDOG_STATUS                       = 0xE6,
	CW_I2C_REG_DUMP_BACKUP_REG                       = 0xE7,
	CW_I2C_REG_CAPTURE_RAMDUMP                       = 0xE8,
	CW_I2C_REG_TRIGGER_CRASH                         = 0xE9,
	CW_I2C_REG_LOG_LEVEL                             = 0xEB,
	CW_I2C_REG_LOG_MASK                              = 0xEC,
	CW_I2C_REG_LOG_SIZE                              = 0xED,
	CW_I2C_REG_LOG_DATA                              = 0xEE,

	TOUCH_STATUS_REGISTER                            = 0xF2,
	CW_I2C_REG_WATCH_DOG_ENABLE                      = 0xF9,
	CW_I2C_REG_WARN_MSG_ENABLE                       = 0xFA,
	CW_I2C_REG_WARN_MSG_BUFFER_LEN                   = 0xFB,
	CW_I2C_REG_WARN_MSG_BUFFER                       = 0xFC,
	CW_I2C_REG_EXCEPTION_BUFFER_LEN                  = 0xFD,
	CW_I2C_REG_EXCEPTION_BUFFER                      = 0xFE,
} CWMCU_I2C_REGISTER;


#define NS_PER_US 1000000LL

int touch_solution(u8 solution);

#if defined(CONFIG_SYNC_TOUCH_STATUS)
int touch_status(u8 status);
#endif


#define TOUCH_STATUS_TURN_HUB_OFF                  0
#define TOUCH_STATUS_TURN_HUB_ON                   1

#define TOUCH_SOLUTION_MAXIM1187X                  0
#define TOUCH_SOLUTION_SYNAPTICS3351               1


typedef struct {
    uint32_t calculate_done;
    uint32_t check_sum;
} mcu_fw_checksum_t;

#define MCU_SYS_STATUS_DLOAD                    (0x776655AA)
#define MCU_SYS_STATUS_SHUB                     (0x77665500)

#define CWMCU_MAX_DELAY	                1000
#define CWMCU_NO_POLLING_DELAY         	10000
#define CWMCU_LIGHT_POLLING_DELAY	1000

#define GESTURE_MOTION_UPDATE_ATTRIBUTE_LEN                     (4)

#define STEP_COUNTER_MASK ((1ULL << CW_STEP_COUNTER) | \
			   (1ULL << CW_STEP_COUNTER_W))

#define IIO_CONTINUOUS_MASK ((1ULL << CW_ACCELERATION) | \
			     (1ULL << CW_MAGNETIC) | \
			     (1ULL << CW_GYRO) | \
			     (1ULL << CW_PRESSURE) | \
			     (1ULL << CW_ORIENTATION) | \
			     (1ULL << CW_ROTATIONVECTOR) | \
			     (1ULL << CW_LINEARACCELERATION) | \
			     (1ULL << CW_GRAVITY) | \
			     (1ULL << CW_MAGNETIC_UNCALIBRATED) | \
			     (1ULL << CW_GYROSCOPE_UNCALIBRATED) | \
			     (1ULL << CW_GAME_ROTATION_VECTOR) | \
			     (1ULL << CW_GEOMAGNETIC_ROTATION_VECTOR) | \
			     (1ULL << CW_STEP_DETECTOR) | \
			     (1ULL << CW_STEP_COUNTER) | \
			     (1ULL << CW_ACCELERATION_W) | \
			     (1ULL << CW_MAGNETIC_W) | \
			     (1ULL << CW_GYRO_W) | \
			     (1ULL << CW_PRESSURE_W) | \
			     (1ULL << CW_ORIENTATION_W) | \
			     (1ULL << CW_ROTATIONVECTOR_W) | \
			     (1ULL << CW_LINEARACCELERATION_W) | \
			     (1ULL << CW_GRAVITY_W) | \
			     (1ULL << CW_MAGNETIC_UNCALIBRATED_W) | \
			     (1ULL << CW_GYROSCOPE_UNCALIBRATED_W) | \
			     (1ULL << CW_GAME_ROTATION_VECTOR_W) | \
			     (1ULL << CW_GEOMAGNETIC_ROTATION_VECTOR_W) | \
			     (1ULL << CW_STEP_DETECTOR_W) | \
			     (1ULL << CW_STEP_COUNTER_W))


struct cwmcu_ramdump_param {
    u32 start_addr;
    u32 size;
};


#define WATCHDOG_STATUS_LEN                                     12
#define I2C_EVENT_READ_LEN                                      24
#define I2C_RAMDUMP_READ_LEN 128
#define I2C_LOG_READ_LEN 32

#define EXCEPTION_BUFFER_LEN_SIZE                         4
#define EXCEPTION_BLOCK_LEN                               16
#define EXCEPTION_LEN_MAX                                 1024

#define WARN_MSG_BUFFER_LEN_SIZE                          8
#define WARN_MSG_BLOCK_LEN                                16
#define WARN_MSG_PER_ITEM_LEN                             120


#define DEFAULT_DELAY_US           300000

#define UPDATE_RATE_NONE                0
#define UPDATE_RATE_NORMAL              1
#define UPDATE_RATE_UI                  2
#define UPDATE_RATE_GAME                3
#define UPDATE_RATE_FASTEST             4
#define UPDATE_RATE_RATE_10Hz           5
#define UPDATE_RATE_RATE_25Hz           6


#define standardbase			0
#define	acceleration			CW_ACCELERATION
#define	magnetic			CW_MAGNETIC
#define	gyro				CW_GYRO
#define	light				CW_LIGHT
#define	proximity			CW_PROXIMITY
#define	pressure			CW_PRESSURE
#define	orientation			CW_ORIENTATION
#define	rotation_vector			CW_ROTATIONVECTOR
#define	linear_acceleration		CW_LINEARACCELERATION
#define	gravity				CW_GRAVITY

#define magnetic_uncalibrated		CW_MAGNETIC_UNCALIBRATED
#define gyroscope_uncalibrated		CW_GYROSCOPE_UNCALIBRATED
#define game_rotation_vector		CW_GAME_ROTATION_VECTOR
#define geomagnetic_rotation_vector	CW_GEOMAGNETIC_ROTATION_VECTOR

#define significant_motion		CW_SIGNIFICANT_MOTION
#define step_detector			CW_STEP_DETECTOR
#define step_counter			CW_STEP_COUNTER

#define num_sensors			CW_SENSORS_ID_TOTAL


typedef struct {
    uint32_t boot_sec;
    uint32_t boot_nsec;
    uint16_t year;              /* range: 1900~2xxx */
    uint8_t  month;             /* range: 1~12 */
    uint8_t  day;               /* range: 1~31 */
    uint8_t  hour;              /* range: 0~23 */
    uint8_t  minute;            /* range: 0~59 */
    uint8_t  second;            /* range: 0~59 */
    uint8_t  hour_format_24;    /* 0/1 */
} MCU_TIME_SYNC_T;

#define MAX_EVENT_COUNT                         5000

/* If queue is empty */
#define CWMCU_NODATA	0xFF


/* INT_ST1 */
#define CW_MCU_INT_BIT_LIGHT                    (1 << 3)
#define CW_MCU_INT_BIT_PROXIMITY                (1 << 4)

/* INT_ST2 */
#define CW_MCU_INT_BIT_SHUB_BOOTUP              (1 << 5)
#define CW_MCU_INT_BIT_LOG_AVAILABLE            (1 << 6)

/* INT_ST3 */
#define CW_MCU_INT_BIT_SIGNIFICANT_MOTION       (1 << 4)
#define CW_MCU_INT_BIT_STEP_DETECTOR            (1 << 5)
#define CW_MCU_INT_BIT_STEP_COUNTER             (1 << 6)

/* INT_ST4 */
#define CW_MCU_INT_BIT_HTC_GESTURE_MOTION       (1 << 0)
#define CW_MCU_INT_BIT_ANY_MOTION               (1 << 4)

/* ERR_ST */
#define CW_MCU_INT_BIT_ERROR_WARN_MSG           (1 << 5)
#define CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION      (1 << 6)
#define CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET     (1 << 7)

/* batch_st */
#define CW_MCU_INT_BIT_BATCH_TIMEOUT      (1 << 2)
#define CW_MCU_INT_BIT_BATCH_BUFFER_FULL  (1 << 4)
#define CW_MCU_INT_BIT_BATCH_TRIGGER_READ (CW_MCU_INT_BIT_BATCH_TIMEOUT |\
					   CW_MCU_INT_BIT_BATCH_BUFFER_FULL)
#define CW_MCU_INT_BIT_BATCH_INT_MASK     CW_MCU_INT_BIT_BATCH_TRIGGER_READ

/*#define IIO_SENSORS_MASK (((u64)(~0ULL)) & ~(1ULL << HTC_MAGIC_COVER) & \
					   ~(1ULL << (HTC_MAGIC_COVER+32)))*/
#define IIO_SENSORS_MASK ((u64)(~0ULL))

#define CW_MCU_BIT_LIGHT_POLLING		(1 << 5)
#define CW_MCU_BIT_PROXIMITY_POLLING		(1 << 5)


#define FW_DOES_NOT_EXIST                       (1 << 0)
#define FW_UPDATE_QUEUED                        (1 << 1)
#define FW_ERASE_FAILED                         (1 << 2)
#define FW_FLASH_FAILED                         (1 << 3)

#define HTC_GESTURE_MOTION_TYPE_SWIPE_UP             2
#define HTC_GESTURE_MOTION_TYPE_SWIPE_DOWN           3
#define HTC_GESTURE_MOTION_TYPE_SWIPE_LEFT           4
#define HTC_GESTURE_MOTION_TYPE_SWIPE_RIGHT          5
#define HTC_GESTURE_MOTION_TYPE_LAUNCH_CAMERA        6
#define HTC_GESTURE_MOTION_TYPE_DOUBLE_TAP           15

#define DOTVIEW_NO_COVER 0
#define DOTVIEW_COVER    1

/* Backlight 2.0 Conversion */
#define ADC_TO_LUX(x) 		(x*(2))

#ifdef __KERNEL__
struct cwmcu_platform_data {
	unsigned char acceleration_axes;
	unsigned char magnetic_axes;
	unsigned char gyro_axes;
	uint32_t gpio_wake_mcu;
	uint32_t gpio_reset;
	uint32_t gpio_chip_mode;
	uint32_t gpio_mcu_irq;
	int gs_chip_layout;

};
#endif /* __KERNEL */

#endif /* __CWMCUSENSOR_H__ */
