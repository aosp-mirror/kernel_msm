/* drivers/input/touchscreen/ektf.c - ELAN EKTF verions of driver
*
* Copyright (C) 2011 Elan Microelectronics Corporation.
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
* 2014/0/28: The first release, version 0x0006
*             Integrated 2 ,5 ,and 10 fingers driver code together and
*             auto-mapping resolution.
*             Please change following parameters
*                 1. For 5 fingers protocol, please enable ELAN_PROTOCOL.
*                    The packet size is 18 or 24 bytes.
*                 2. For 10 fingers, please enable both ELAN_PROTOCOL and ELAN_TEN_FINGERS.
*                    The packet size is 40 or 4+40+40+40 (Buffer mode) bytes.
*                 3. Please enable the ELAN_BUTTON configuraton to support button.
*		  4. For ektf3k serial, Add Re-Calibration Machanism 
*                    So, please enable the define of RE_CALIBRATION.
*		  5. Please enable the define of ESD_CHECK, if your firmware support 
*		     "I am live" packet(0x78 0x78 0x78 0x78).
*                   
*								 
*/

/* The ELAN_PROTOCOL support normanl packet format */	
/* Quanta BU10SW, Stanley Tsao, 2015.12.09, Change finger number to 2 for XU1 { */
//#define FINGER_NUM 10
#define FINGER_NUM 2
/* Quanta BU10SW, Stanley Tsao, 2015.12.09, Change finger number to 2 for XU1 { */

#define ELAN_PROTOCOL	
//#define ELAN_BUFFER_MODE
//#define ELAN_BUTTON
/* #define RE_CALIBRATION */   /* Re-Calibration after system resume. */
//#define ELAN_2WIREICE
#define ELAN_POWER_SOURCE
#define ELAN_RESUME_RST
#define DEVICE_NAME "elan_ktf" 
#define EKTF3K_FLASH
#if 1 /* Only one of the protocols can be enabled */
#define PROTOCOL_A    /* multi-touch protocol  */
#else
#define PROTOCOL_B    /* Default: PROTOCOL B */
#endif
//#define ELAN_HID_I2C	/* for hid over i2c protocol */

/* Quanta BU10SW, Stanley Tsao, 2015.11.23, Add Quanta customized gesture events {*/
//#define FEATURE_QUANTA_GESTURE_CUSTOMIZATION
/* Quanta BU10SW, Stanley Tsao, 2015.11.23, Add Quanta customized gesture events }*/

/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */
#define QUANTA_DEFER_ENABLE_IRQ
/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */


/* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
#define MSM_NEW_VER	//cotrol new platform  // Stanley Tsao, enable this for msm8909
/* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 } */

#define FEATURE_PALM_DETECTION

#define FEATURE_QUANTA_GESTURE_TO_BOOTLOADER
#define FEATURE_QUANTA_GESTURE_TO_RECOVERY

#if defined(FEATURE_QUANTA_GESTURE_TO_BOOTLOADER) || defined(FEATURE_QUANTA_GESTURE_TO_RECOVERY)
#include <linux/reboot.h>
#endif


#include <linux/module.h>
#include <linux/input.h>
#ifdef PROTOCOL_B
#include <linux/input/mt.h>
#endif
#ifdef PROTOCOL_A
#include <linux/input.h>
#endif
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Fix compiling error { */
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Fix compiling error } */
#endif
// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Place elan_ktf.h in the folder in which elan_ktf.c is placed { */
/* #include <linux/elan_ktf.h> */
#include <elan_ktf.h> 
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Place elan_ktf.h in the folder in which elan_ktf.c is placed } */
#include <linux/kthread.h>
//#include "elan_ktf.h"
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/regulator/machine.h>
//#include <linux/regulator/krait-regulator.h>
#endif

/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check { */
#include <linux/version.h>
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check } */

/* Quanta BU10SW, Stanley Tsao, 2015.12.01, Add DT parsing function { */
#include <linux/of_gpio.h>
/* Quanta BU10SW, Stanley Tsao, 2015.12.01, Add DT parsing function } */

#include <linux/qpnp/qpnp-vibrator.h>



#define PACKET_SIZE		8		/* support 2 fingers packet for watch */
#define MAX_FINGER_SIZE		255
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_IDLE          2
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54
#define RESET_PKT		0x77
#define CALIB_PKT		0x66
#define IamAlive_PKT	0x78
#define IDLE_MODE_PKT   0xED  /* ED ED ED ED */
#define PEN_PKT			0x71

#define HELLO_PKT		0x55
#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define ELAN_HID_PKT			0x3F
#define BUFFER_PKT		0x63
#define BUFFER55_PKT		0x66

#define FEATURE_CHECK_TRUE_INTERRUPT

#ifdef FEATURE_CHECK_TRUE_INTERRUPT
static int touchINT_cnt = 0;
static int IamAlive_PKT_INT_cnt = 0;
#endif


#ifdef FEATURE_PALM_DETECTION
#define PALM_DETECTION_PKT 0xBA
#endif

static const int NEWEST_FW_VER = 0xb013;

/* Common for quanta gestures */
#if defined(FEATURE_QUANTA_GESTURE_TO_BOOTLOADER) || defined(FEATURE_QUANTA_GESTURE_TO_RECOVERY)
#define QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_X   150
#define QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_Y   150

struct quanta_gesture_point {
    uint16_t x;
    uint16_t y;
};

struct quanta_gesture_point_3 {
    struct quanta_gesture_point p1;
    struct quanta_gesture_point p2;
    struct quanta_gesture_point p3;
};

static bool quanta_gesture_checkpoint
(
	uint16_t reported_x,
	uint16_t reported_y,
	struct quanta_gesture_point target_point
)
{
    if (reported_x < target_point.x + QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_X &&
        reported_x > target_point.x - QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_X &&
        reported_y < target_point.y + QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_Y &&
        reported_y > target_point.y - QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_Y )
	{
        return true;
	}
    else
        return false;
}
#endif

#ifdef FEATURE_QUANTA_GESTURE_TO_RECOVERY
#define QUANTA_GESTURE_TO_RECOVERY_1_X 874
#define QUANTA_GESTURE_TO_RECOVERY_1_Y 150
#define QUANTA_GESTURE_TO_RECOVERY_2_X 512
#define QUANTA_GESTURE_TO_RECOVERY_2_Y 512
#define QUANTA_GESTURE_TO_RECOVERY_3_X 150
#define QUANTA_GESTURE_TO_RECOVERY_3_Y 874

#define QUANTA_GESTURE_TO_RECOVERY_1_POINT_FLAG 0x01
#define QUANTA_GESTURE_TO_RECOVERY_2_POINT_FLAG 0x02
#define QUANTA_GESTURE_TO_RECOVERY_3_POINT_FLAG 0x04
#define QUANTA_GESTURE_TO_RECOVERY_FLAG \
        (QUANTA_GESTURE_TO_RECOVERY_1_POINT_FLAG | \
		 QUANTA_GESTURE_TO_RECOVERY_2_POINT_FLAG | \
		 QUANTA_GESTURE_TO_RECOVERY_3_POINT_FLAG )

static int  quanta_gesture_to_recovery = 0x00;
static bool is_quanta_gesture_to_rcvr_enabled = true;
static bool is_enalbe_vib = true;

/* Pre-defined gesture points */
static struct quanta_gesture_point_3 toRecovery = {
    {QUANTA_GESTURE_TO_RECOVERY_1_X, QUANTA_GESTURE_TO_RECOVERY_1_Y},
    {QUANTA_GESTURE_TO_RECOVERY_2_X, QUANTA_GESTURE_TO_RECOVERY_2_Y},
    {QUANTA_GESTURE_TO_RECOVERY_3_X, QUANTA_GESTURE_TO_RECOVERY_3_Y}
};

static int quanta_gesture_clear_recovery_flag(void)
{
    quanta_gesture_to_recovery = 0x00;
    printk("[quanta]%s \n", __func__);

	return 0;
}

/*
               /
			  /
			 /
			/
		   /
		  <
   Fomular: y = -x + 1024
*/
static bool is_quanta_gesture_to_recovery_point_on_the_line(uint16_t reported_x, uint16_t reported_y)
{
	if (reported_y < (-reported_x + 1024) - QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_Y ||
	    reported_y > (-reported_x + 1024) + QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_Y)
	{
		return false;
	}
	return true;
}

static int quanta_gesture_to_recovery_check(uint16_t reported_x, uint16_t reported_y)
{
    // Check the 1st point
    if(!(quanta_gesture_to_recovery & QUANTA_GESTURE_TO_RECOVERY_1_POINT_FLAG))
    {
        if (quanta_gesture_checkpoint(reported_x, reported_y, toRecovery.p1))
        {
            quanta_gesture_to_recovery |= QUANTA_GESTURE_TO_RECOVERY_1_POINT_FLAG;
            printk("[quanta]%s: bootloader point 1 check PASS, quanta_gesture_to_bootloader=%x \n", __func__, quanta_gesture_to_recovery);
        }
		return 0;
    }

	/* If any of the points after the 1st matched point is not on the defined line --> reset gesture flag
	   The folumar for the defined line of gesture to bootloader is y=x
	*/
	if (!is_quanta_gesture_to_recovery_point_on_the_line(reported_x, reported_y))
	{
		quanta_gesture_clear_recovery_flag();
		return 0;
	}

    // Check the 2nd point
    if(!(quanta_gesture_to_recovery & QUANTA_GESTURE_TO_RECOVERY_2_POINT_FLAG))
    {
        if (quanta_gesture_checkpoint(reported_x, reported_y, toRecovery.p2))
        {
            quanta_gesture_to_recovery |= QUANTA_GESTURE_TO_RECOVERY_2_POINT_FLAG;
            printk("[quanta]%s: bootloader point 2 check PASS, quanta_gesture_to_bootloader=%x \n", __func__, quanta_gesture_to_recovery);
        }
		return 0;
    }

	/* If any of the points after the 1st matched point is not on the defined line --> reset gesture flag
	   The folumar for the defined line of gesture to bootloader is y=x
	*/
	if (!is_quanta_gesture_to_recovery_point_on_the_line(reported_x, reported_y))
	{
		quanta_gesture_clear_recovery_flag();
		return 0;
	}

    // Check the 3rd point
    if(!(quanta_gesture_to_recovery & QUANTA_GESTURE_TO_RECOVERY_3_POINT_FLAG))
    {
        if (quanta_gesture_checkpoint(reported_x, reported_y, toRecovery.p3))
        {
            quanta_gesture_to_recovery |= QUANTA_GESTURE_TO_RECOVERY_3_POINT_FLAG;
            printk("[quanta]%s: bootloader point 3 check PASS, quanta_gesture_to_bootloader=%x \n", __func__, quanta_gesture_to_recovery);
        }
		return 0;
    }

    if( quanta_gesture_to_recovery & QUANTA_GESTURE_TO_RECOVERY_FLAG)
    {
        printk("[quanta]%s: quanta_gesture_to_bootloader %x, restart to bootloader \n", __func__, quanta_gesture_to_recovery);
        kernel_restart("recovery");
    }

	return 0;
}
#endif //FEATURE_QUANTA_GESTURE_TO_BOOTLOADER

#ifdef FEATURE_QUANTA_GESTURE_TO_BOOTLOADER
/* Assume touch resolution is 1024 x 1024 */
#define QUANTA_GESTURE_TO_BOOTLOADER_1_X 150
#define QUANTA_GESTURE_TO_BOOTLOADER_1_Y 150
#define QUANTA_GESTURE_TO_BOOTLOADER_2_X 512
#define QUANTA_GESTURE_TO_BOOTLOADER_2_Y 512
#define QUANTA_GESTURE_TO_BOOTLOADER_3_X 874
#define QUANTA_GESTURE_TO_BOOTLOADER_3_Y 874

#define QUANTA_GESTURE_TO_BOOTLOADER_1_POINT_FLAG 0x01
#define QUANTA_GESTURE_TO_BOOTLOADER_2_POINT_FLAG 0x02
#define QUANTA_GESTURE_TO_BOOTLOADER_3_POINT_FLAG 0x04
#define QUANTA_GESTURE_TO_BOOTLOER_FLAG \
        (QUANTA_GESTURE_TO_BOOTLOADER_1_POINT_FLAG | \
		 QUANTA_GESTURE_TO_BOOTLOADER_2_POINT_FLAG | \
		 QUANTA_GESTURE_TO_BOOTLOADER_3_POINT_FLAG )

static int  quanta_gesture_to_bootloader = 0x00;
static bool is_quanta_gesture_to_bld_enabled = true;

/* Pre-defined gesture points */
static struct quanta_gesture_point_3 toBootloader = {
    {QUANTA_GESTURE_TO_BOOTLOADER_1_X, QUANTA_GESTURE_TO_BOOTLOADER_1_Y},
    {QUANTA_GESTURE_TO_BOOTLOADER_2_X, QUANTA_GESTURE_TO_BOOTLOADER_2_Y},
    {QUANTA_GESTURE_TO_BOOTLOADER_3_X, QUANTA_GESTURE_TO_BOOTLOADER_3_Y}
};

static int quanta_gesture_clear_bootloader_flag(void)
{
    quanta_gesture_to_bootloader = 0x00;
    printk("[quanta]%s \n", __func__);

	return 0;
}

static bool is_quanta_gesture_to_bootlader_point_on_the_line(uint16_t reported_x, uint16_t reported_y)
{
	if (reported_y < reported_x - QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_Y ||
	    reported_y > reported_x + QUANTA_GESTURE_POINT_OFFSET_TOLERANCE_Y)
	{
		return false;
	}
	return true;
}

static int quanta_gesture_to_bootloader_check(uint16_t reported_x, uint16_t reported_y)
{

    // Check the 1st point
    if(!(quanta_gesture_to_bootloader & QUANTA_GESTURE_TO_BOOTLOADER_1_POINT_FLAG))
    {
        if (quanta_gesture_checkpoint(reported_x, reported_y, toBootloader.p1))
        {
            quanta_gesture_to_bootloader |= QUANTA_GESTURE_TO_BOOTLOADER_1_POINT_FLAG;
            printk("[quanta]%s: bootloader point 1 check PASS, quanta_gesture_to_bootloader=%x \n", __func__, quanta_gesture_to_bootloader);
        }
		return 0;
    }

	/* If any of the points after the 1st matched point is not on the defined line --> reset gesture flag
	   The folumar for the defined line of gesture to bootloader is y=x
	*/
	if (!is_quanta_gesture_to_bootlader_point_on_the_line(reported_x, reported_y))
	{
		quanta_gesture_clear_bootloader_flag();
		return 0;
	}

    // Check the 2nd point
    if(!(quanta_gesture_to_bootloader & QUANTA_GESTURE_TO_BOOTLOADER_2_POINT_FLAG))
    {
        if (quanta_gesture_checkpoint(reported_x, reported_y, toBootloader.p2))
        {
            quanta_gesture_to_bootloader |= QUANTA_GESTURE_TO_BOOTLOADER_2_POINT_FLAG;
            printk("[quanta]%s: bootloader point 2 check PASS, quanta_gesture_to_bootloader=%x \n", __func__, quanta_gesture_to_bootloader);
        }
		return 0;
    }

	/* If any of the points after the 1st matched point is not on the defined line --> reset gesture flag
	   The folumar for the defined line of gesture to bootloader is y=x
	*/
	if (!is_quanta_gesture_to_bootlader_point_on_the_line(reported_x, reported_y))
	{
		quanta_gesture_clear_bootloader_flag();
		return 0;
	}

    // Check the 3rd point
    if(!(quanta_gesture_to_bootloader & QUANTA_GESTURE_TO_BOOTLOADER_3_POINT_FLAG))
    {
        if (quanta_gesture_checkpoint(reported_x, reported_y, toBootloader.p3))
        {
            quanta_gesture_to_bootloader |= QUANTA_GESTURE_TO_BOOTLOADER_3_POINT_FLAG;
            printk("[quanta]%s: bootloader point 3 check PASS, quanta_gesture_to_bootloader=%x \n", __func__, quanta_gesture_to_bootloader);
        }
		return 0;
    }

    if( quanta_gesture_to_bootloader & QUANTA_GESTURE_TO_BOOTLOER_FLAG)
    {
        printk("[quanta]%s: quanta_gesture_to_bootloader %x, restart to bootloader \n", __func__, quanta_gesture_to_bootloader);
        kernel_restart("bootloader");
    }

	return 0;
}
#endif //FEATURE_QUANTA_GESTURE_TO_BOOTLOADER


/* Quanta BU10SW, Stanley Tsao, 2015.11.23, Add Quanta customized gesture events {*/
#ifdef FEATURE_QUANTA_GESTURE_CUSTOMIZATION
#define QUANTA_GESTURE_PKT 0x9E
#define QUANTA_GESTURE__NUM_GESTURES 8

#define LINUX_KEY_F1 0x3b
#define LINUX_KEY_F2 0x3c
#define LINUX_KEY_F3 0x3d
#define LINUX_KEY_F4 0x3e
#define LINUX_KEY_F5 0x3f
#define LINUX_KEY_F6 0x40
#define LINUX_KEY_F7 0x41
#define LINUX_KEY_F8 0x42

static uint8_t gesture_map_table[QUANTA_GESTURE__NUM_GESTURES] =
                {LINUX_KEY_F1, /* toucH ic output 0x12, 12-> 3  */
                 LINUX_KEY_F2, /* toucH ic output 0x13, 3 -> 12 */
                 LINUX_KEY_F3, /* toucH ic output 0x14, 3 -> 6  */
                 LINUX_KEY_F4, /* toucH ic output 0x15, 6 -> 3  */
                 LINUX_KEY_F5, /* toucH ic output 0x16, 6 -> 9  */
                 LINUX_KEY_F6, /* toucH ic output 0x17, 9 -> 6  */
                 LINUX_KEY_F7, /* toucH ic output 0x18, 9 -> 12 */
                 LINUX_KEY_F8, /* toucH ic output 0x19, 12-> 9  */
                };
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.11.23, Add Quanta customized gesture events }*/

/* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
#ifdef MSM_NEW_VER
#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"
#endif
/* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 } */

#ifdef PROTOCOL_A
#define CURRENT_MT_PROTOCOL "A"
#endif

#ifdef PROTOCOL_B
#define CURRENT_MT_PROTOCOL "B"
#endif

// Reset pin need to be modified by customer
/* Quanta BU10SW, Stanley Tsao, 2015.12.01, Change for XU1 { */
#if 0
#define SYSTEM_RESET_PIN_SR 31	// nexus7_grouper TEGRA_GPIO_PH6: 62, nexus7_flo 31
#else
#define SYSTEM_RESET_PIN_SR 12
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.12.01, Change for XU1 } */

//Add these Define
#define IAP_PORTION            	
#define PAGERETRY  30
#define IAPRESTART 5

#define ESD_CHECK
#if defined( ESD_CHECK )
  static int    have_interrupts = 0;
  static struct workqueue_struct *esd_wq = NULL;
  static struct delayed_work      esd_work;
  static unsigned long  delay = 3*HZ;
  static int    suspend_ESD = 0;
//declare function
  static void elan_touch_esd_func(struct work_struct *work);
#endif

// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_FW_INFO  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)

#define CUSTOMER_IOCTLID	0xA0
#define IOCTL_CIRCUIT_CHECK  _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

#define ELAN_FW_FILENAME  "ElanFW.fw"

/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_ERROR  1
#define DEBUG_INFO     2
#define DEBUG_MESSAGES 5
#define DEBUG_TRACE   10


#define SYSFS_MAX_LEN 100
static unsigned int gPrint_point = 0;
static int debug = DEBUG_TRACE;

#define touch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
		printk("[ektf]:" __VA_ARGS__); \
	} while (0)


uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
/* Quanta BU10SW, Stanley Tsao, 2015.12.01, For XU1 touch resolution { */
#if 0
int X_RESOLUTION=1344;	// nexus7 1280 1344
int Y_RESOLUTION=2240;	// nexus7 2112 2240
#else /* Quanta XU1 project */
int X_RESOLUTION=1024;	
int Y_RESOLUTION=1024; 
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.12.01, For XU1 touch resolution } */

/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */
#ifdef QUANTA_DEFER_ENABLE_IRQ
#define DEFER_ENABLE_IRQ_TIMER_MS 0
 #endif /* QUANTA_DEFER_ENABLE_IRQ */
/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */

int BC_VERSION= 0x00;
int FW_ID=0x00;
int work_lock=0x00;
int power_lock=0x00;
int circuit_ver=0x01;
/*++++i2c transfer start+++++++*/
int file_fops_addr=0x10;
/*++++i2c transfer end+++++++*/
struct mutex ktf_mutex;
int button_state = 0;

#ifdef IAP_PORTION
uint8_t ic_status=0x00;	//0:OK 1:master fail 2:slave fail
int update_progree=0;
uint8_t I2C_DATA[3] = {0x10, 0x20, 0x21};/*I2C devices address*/  
int is_OldBootCode = 0; // 0:new 1:old
static unsigned char firmware[52800];


/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data[] = {
	#include "fw_data_eWD1000_XU1_B013.i"
};


enum
{
	PageSize		= 132,
	ACK_Fail		= 0x00,
	ACK_OK			= 0xAA,
	ACK_REWRITE		= 0x55,
};

//int	PageNum		  = sizeof(file_fw_data)/132; /*for ektf2xxx/3xxx serial, the page number is 249/351*/
int	PageNum = 0;

enum power_mode_type
{
    POWER_MODE_NORMAL = 0,
    POWER_MODE_IDLE   = 1,
    POWER_MODE_SLEEP  = 2,
};


enum
{
	E_FD			= -1,
};
#endif
//#define _ENABLE_DBG_LEVEL

#ifdef _ENABLE_DBG_LEVEL
#define PROC_FS_NAME    "ektf_dbg"
#define PROC_FS_MAX_LEN 8
static struct proc_dir_entry *dbgProcFile;
#endif

struct elan_ktf_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */
    #ifdef QUANTA_DEFER_ENABLE_IRQ
	struct delayed_work workd; 	
    #endif /* QUANTA_DEFER_ENABLE_IRQ */
    /* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq } */   
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	int intr_gpio;
	int rst_gpio;
	// Firmware Information
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
	// For Firmare Update 
	struct miscdevice firmware;
	struct wake_lock wakelock;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function { */
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function } */
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */    
    #ifdef MSM_NEW_VER
    struct pinctrl *ts_pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
    struct pinctrl_state *pinctrl_state_release;
    #endif
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
    uint power_mode;
	int irq;

};

static struct elan_ktf_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf_ts_calibrate(struct i2c_client *client);
static int elan_ktf_ts_resume(struct i2c_client *client);
static int elan_ktf_ts_suspend(struct i2c_client *client, pm_message_t mesg);
void elan_ktf_ts_hw_reset(void);
static int __hello_packet_handler(struct i2c_client *client);

#ifdef IAP_PORTION
//static int Update_FW_in_Driver(void *x);
static int Update_FW_One(int source);
#endif

#ifdef ELAN_2WIREICE
int elan_TWO_WIRE_ICE( struct i2c_client *client);
#endif


/* Quanta BU10SW, Stanley Tsao, 2015.12.01, Add DT parsing function { */
#ifdef CONFIG_OF
static int elan_ktf_parse_dt(struct device *dev, struct elan_ktf_i2c_platform_data *pdata)
{
    //int rc;
    struct device_node *np = dev->of_node;
    //struct property *prop;

    printk("[Stanley]%s called \n", __func__);
    
    // get interrupt gpio
    // get reset gpio
	pdata->rst_gpio = of_get_named_gpio_flags(np, "elan_ktf,reset-gpio", 0, &pdata->rst_gpio_flags);
    if (pdata->rst_gpio < 0)
        return pdata->rst_gpio;

    printk("[stanley]%s: reset gpio=%d, flags=%d \n", __func__, pdata->rst_gpio, pdata->rst_gpio_flags);

    pdata->intr_gpio = of_get_named_gpio_flags(np, "elan_ktf,irq-gpio", 0, &pdata->intr_gpio_flags);
    if (pdata->intr_gpio < 0)
        return pdata->intr_gpio;

    printk("[stanley]%s: interrupt gpio=%d, flags=%d \n", __func__, pdata->intr_gpio, pdata->intr_gpio_flags);
    

    return 0;
}
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.12.01, Add DT parsing function } */


/* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
#ifdef MSM_NEW_VER
static int elan_ktf_ts_pinctrl_init(struct elan_ktf_ts_data *elan_ktf_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	elan_ktf_data->ts_pinctrl = devm_pinctrl_get(&(elan_ktf_data->client->dev));
	if (IS_ERR_OR_NULL(elan_ktf_data->ts_pinctrl)) {
		retval = PTR_ERR(elan_ktf_data->ts_pinctrl);
		dev_dbg(&elan_ktf_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	elan_ktf_data->pinctrl_state_active
		= pinctrl_lookup_state(elan_ktf_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(elan_ktf_data->pinctrl_state_active)) {
		retval = PTR_ERR(elan_ktf_data->pinctrl_state_active);
		dev_err(&elan_ktf_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	elan_ktf_data->pinctrl_state_suspend
		= pinctrl_lookup_state(elan_ktf_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(elan_ktf_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(elan_ktf_data->pinctrl_state_suspend);
		dev_err(&elan_ktf_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	elan_ktf_data->pinctrl_state_release
		= pinctrl_lookup_state(elan_ktf_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(elan_ktf_data->pinctrl_state_release)) {
		retval = PTR_ERR(elan_ktf_data->pinctrl_state_release);
		dev_dbg(&elan_ktf_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(elan_ktf_data->ts_pinctrl);
err_pinctrl_get:
	elan_ktf_data->ts_pinctrl = NULL;
	return retval;
}
#endif
/* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 | */

/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */
#ifdef QUANTA_DEFER_ENABLE_IRQ
static void elan_work(struct work_struct *work)
{
	//struct elan_ktf_ts_data *ts = container_of(work, struct elan_ktf_ts_data, work.work);
	
	printk("[Stanley]%s: elan_work\n", __func__);
	printk("[Stanley]%s: case elan_work() \n", __func__);
	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);
}
#endif /* #ifdef QUANTA_DEFER_ENABLE_IRQ */
/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq } */


static int __elan_ktf_ts_poll(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 10;

	do {
		status = gpio_get_value(ts->intr_gpio);
		if(status==0) break;
		
		touch_debug(DEBUG_MESSAGES, "%s: status = %d\n", __func__, status);
		retry--;
		mdelay(50);
	} while (status == 1 && retry > 0);

	touch_debug(DEBUG_INFO, "[elan]%s: poll interrupt status %s\n",
	__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf_ts_poll(struct i2c_client *client)
{
	return __elan_ktf_ts_poll(client);
}

/************************************
* Restet TP 
*************************************/
void elan_ktf_ts_hw_reset()
{
	//reset
	//gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
	//msleep(20);
	//gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
	//msleep(10);
    /* Quanta BU10SW, Stalney Tsao, 2015.12.25, get reset pin number from device tree { */
    #if 0
    gpio_direction_output(SYSTEM_RESET_PIN_SR, 0);
    msleep(20);
    gpio_direction_output(SYSTEM_RESET_PIN_SR, 1);
    msleep(10);
    #else
    gpio_direction_output(private_ts->rst_gpio, 0);
    msleep(20);
    gpio_direction_output(private_ts->rst_gpio, 1);
    msleep(10);
    #endif
    /* Quanta BU10SW, Stalney Tsao, 2015.12.25, get reset pin number from device tree } */
}

/*
void read_test(void)
{
	int pos=0;
	struct file *firmware_fp;
	mm_segment_t oldfs;
	oldfs=get_fs();
	set_fs(KERNEL_DS);
	firmware_fp = filp_open("/data/local/tmp/ME571_IP_10.ekt", O_RDONLY, S_IRUSR |S_IRGRP);
	if(PTR_ERR(firmware_fp) == -ENOENT){
		touch_debug(DEBUG_ERROR, "open file error\n");
		return;
	}
	PageNum=0;
	firmware_fp->f_pos = 0;
	for(pos = 0; pos < 500*132; pos += 132,PageNum++){
		if(firmware_fp->f_op->read(firmware_fp, firmware + pos,
					132, &firmware_fp->f_pos) != 132){
			break;
		}
	}
	touch_debug(DEBUG_INFO, "%s: PageNUM %d, Ver %x %x, ID %x %x\n",__func__,PageNum,firmware[34058],firmware[34059],firmware[34586],firmware[34587]);
	set_fs(oldfs);
	return;
}
*/

// For Firmware Update 
int elan_iap_open(struct inode *inode, struct file *filp){ 
	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_open\n");
	if (private_ts == NULL)  touch_debug(DEBUG_ERROR,"private_ts is NULL\n");
	
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
	int ret;
	char *tmp;
	touch_debug(DEBUG_MESSAGES, "[ELAN]into elan_iap_write\n");
#if 0
	/*++++i2c transfer start+++++++*/    	
	struct i2c_adapter *adap = private_ts->client->adapter;    	
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++*/	
#endif
	if (count > 8192)
	count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	
	if (tmp == NULL)
	return -ENOMEM;

	if (copy_from_user(tmp, buff, count)) {
		return -EFAULT;
	}

	/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	msg.flags = 0x00;// 0x00
	msg.len = count;
	msg.buf = (char *)tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
	
	ret = i2c_master_send(private_ts->client, tmp, count);
#endif	
	/*++++i2c transfer end+++++++*/

	//if (ret != count) printk("ELAN i2c_master_send fail, ret=%d \n", ret);
	kfree(tmp);
	//return ret;
	return (ret == 1) ? count : ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
	char *tmp;
	int ret;  
	long rc;
	touch_debug(DEBUG_MESSAGES,"[ELAN]into elan_iap_read\n");
#if 0
	/*++++i2c transfer start+++++++*/
	struct i2c_adapter *adap = private_ts->client->adapter;
	struct i2c_msg msg;
	/*++++i2c transfer end+++++++*/
#endif

	if (count > 8192)
	count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
	return -ENOMEM;
	/*++++i2c transfer start+++++++*/
#if 0
	//down(&worklock);
	msg.addr = file_fops_addr;
	//msg.flags |= I2C_M_RD;
	msg.flags = 0x00;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
	ret = i2c_master_recv(private_ts->client, tmp, count);
#endif
	/*++++i2c transfer end+++++++*/
	if (ret >= 0)
	rc = copy_to_user(buff, tmp, count);
	
	kfree(tmp);

	//return ret;
	return (ret == 1) ? count : ret;
	
}

static long elan_iap_ioctl( struct file *filp,    unsigned int cmd, unsigned long arg){

	int __user *ip = (int __user *)arg;

	touch_debug(DEBUG_MESSAGES,"[ELAN]into elan_iap_ioctl\n");
	touch_debug(DEBUG_MESSAGES,"cmd value %x\n",cmd);
	
	switch (cmd) {        
	case IOCTL_I2C_SLAVE: 
		private_ts->client->addr = (int __user)arg;
		//file_fops_addr = 0x10;
		break;   
    case IOCTL_FW_INFO:
        if(__fw_packet_handler(private_ts->client) < 0)
            printk(KERN_ERR "[elan] IOCTL_FW_INFO, res < 0, __fw_packet_handler fail.\n");
		break;        
	case IOCTL_MINOR_FW_VER:            
		break;        
	case IOCTL_RESET:
		// modify
		elan_ktf_ts_hw_reset();
		break;
	case IOCTL_IAP_MODE_LOCK:
		if(work_lock==0)
		{
			work_lock=1;
			disable_irq(private_ts->client->irq);
			cancel_work_sync(&private_ts->work);
#if defined( ESD_CHECK )
        		cancel_delayed_work_sync( &esd_work );
#endif
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if(work_lock==1)
		{			
			work_lock=0;
			enable_irq(private_ts->client->irq);
#if defined( ESD_CHECK )  //0604
			queue_delayed_work( esd_wq, &esd_work, delay );
#endif
		}
		break;
	case IOCTL_CHECK_RECOVERY_MODE:
		return RECOVERY;
		break;
	case IOCTL_FW_VER:
		//__fw_packet_handler(private_ts->client);
		return FW_VERSION;
		break;
	case IOCTL_X_RESOLUTION:
		//__fw_packet_handler(private_ts->client);
		return X_RESOLUTION;
		break;
	case IOCTL_Y_RESOLUTION:
		//__fw_packet_handler(private_ts->client);
		return Y_RESOLUTION;
		break;
	case IOCTL_FW_ID:
		//__fw_packet_handler(private_ts->client);
		return FW_ID;
		break;
	case IOCTL_BC_VER:
		return BC_VERSION;
		break;
	case IOCTL_ROUGH_CALIBRATE:
		return elan_ktf_ts_calibrate(private_ts->client);
	case IOCTL_I2C_INT:
		put_user(gpio_get_value(private_ts->intr_gpio), ip);
		break;	
	case IOCTL_RESUME:
		//elan_ktf_ts_resume(private_ts->client);
		break;	
	case IOCTL_POWER_LOCK:
		power_lock=1;
		break;
	case IOCTL_POWER_UNLOCK:
		power_lock=0;
		break;
#ifdef IAP_PORTION		
	case IOCTL_GET_UPDATE_PROGREE:
		update_progree=(int __user)arg;
		break; 
	case IOCTL_FW_UPDATE:
        //read_test();
        //Update_FW_in_Driver(0);
        Update_FW_One(0);
		break;
#endif
#ifdef ELAN_2WIREICE
	case IOCTL_2WIREICE:
		elan_TWO_WIRE_ICE(private_ts->client);
		break;		
#endif
	case IOCTL_CIRCUIT_CHECK:
		return circuit_ver;
		break;
	default:      
		touch_debug(DEBUG_ERROR,"[elan] Un-known IOCTL Command %d\n", cmd);   
		break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {
    .open =         elan_iap_open,    
	.write =        elan_iap_write,    
	.read = 	elan_iap_read,    
	.release =	elan_iap_release,    
	.unlocked_ioctl=elan_iap_ioctl, 
};

#ifdef IAP_PORTION
int HID_EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a}; 
	uint8_t isp_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34}; 
	uint8_t check_addr[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10}; 
	uint8_t buff[67] = {0};


	len = i2c_master_send(private_ts->client, flash_key,  37);
	if (len != 37) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

	mdelay(20);

        len = i2c_master_send(private_ts->client, isp_cmd,  37);
	if (len != 37) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[7], isp_cmd[8], isp_cmd[9], isp_cmd[10]);


	mdelay(20);
 	len = i2c_master_send(private_ts->client, check_addr,  sizeof(check_addr));
	if (len != sizeof(check_addr)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] Check Address write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", check_addr[7], check_addr[8], check_addr[9], check_addr[10]);
	
	mdelay(20);	

	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address Read Data error. len=%d \r\n", len);
		return -1;
	}
	else {
printk("[Check Addr]: ");
for (j=0; j<37; j++)
	printk("%x ", buff[j]);
printk("\n");
	
	}
	
	return 0;
}
int EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50}; //{0x45, 0x49, 0x41, 0x50};
	len = i2c_master_send(private_ts->client, isp_cmd,  4);
	if (len != 4) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
	if (len != byte) 
	{
        touch_debug(DEBUG_ERROR,"[elan] %s: read page error, read error. len=%d\r\n", __func__, len);
		return -1;
	}

	return 0;
}

int WritePage(const u8 * szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage,  byte);
	if (len != byte) 
	{
        touch_debug(DEBUG_ERROR,"[elan] %s: write page error, write error. len=%d\r\n", __func__, len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int rc = 0;

	uint8_t buff[67] = {0};
#ifdef ELAN_HID_I2C	
	rc = elan_ktf_ts_poll(client);
#endif
	rc = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (rc != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] %s: Read ACK Data error. rc=%d\r\n", __func__, rc);
		return -1;
	}

    touch_debug(DEBUG_MESSAGES, "[elan] %s: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",__func__,buff[0],buff[1],buff[2],buff[3],buff[4],buff[5],buff[6],buff[7],buff[8],buff[9],buff[10],buff[11]);
#ifndef ELAN_HID_I2C
	if (buff[0] == 0xaa && buff[1] == 0xaa) 
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;
#endif	
	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent,page_tatol=351,percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i=0; i<((page)/10); i++) {
		str[i] = '#';
		str[i+1] = '\0';
	}
	/*
	page_tatol=page+PageNum*(ic_num-j);
	percent = ((100*page)/(PageNum));
	percent_tatol = ((100*page_tatol)/(PageNum*ic_num));
*/
	percent = ((100*page)/(PageNum));
	if ((page) == (PageNum))
	percent = 100;

	if ((page_tatol) == (PageNum*ic_num))
	percent_tatol = 100;		

    touch_debug(DEBUG_INFO, "\r[elan]progress %s| %d %d", str, percent,page);
	
	if (page == (PageNum))
		touch_debug(DEBUG_INFO, "\n");

}

/*	fw_source = 0: update fw_data.i compile with driver code
	fw_source = 1: update fw at /system/etc/firmware/ElanFW.fw
	fw_source = 2: update ekt file at /data/local/tmp/ElanFW.ekt       */
static int Update_FW_One(int fw_source)
{
    int res = 0,ic_num = 1;
    int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
    int i = 0;
    uint8_t data;
	//struct timeval tv1, tv2;
	// for open user space file
	int pos=0;
	struct file *firmware_fp;
	mm_segment_t oldfs;


    uint8_t boot_buffer[4] = {0};
    int byte_count = 0;

    const uint8_t *szBuff = NULL;
    int curIndex;
    const struct firmware *p_fw_entry;
    const u8 *fw_data;
    int rc, fw_size;

    mutex_lock(&ktf_mutex);
    touch_debug(DEBUG_INFO, "[elan] %s: Update FW\n", __func__);

IAP_RESTART:

    curIndex=0;
    data=I2C_DATA[0];//Master
    touch_debug(DEBUG_INFO, "[elan] %s: address data=0x%x \r\n", __func__, data);

    if(RECOVERY != 0x80)
    {
        touch_debug(DEBUG_MESSAGES, "[elan] Firmware upgrade normal mode !\n");
    } else
        touch_debug(DEBUG_MESSAGES, "[elan] Firmware upgrade recovery mode !\n");

    /*Send enter bootcode cmd*/
    elan_ktf_ts_hw_reset();
    mdelay(13);
    res = EnterISPMode(private_ts->client);	 //enter ISP mode
    //elan_ktf_ts_poll(private_ts->client);
    mdelay(100);
    /*check enter bootcode cmd*/
    res = i2c_master_recv(private_ts->client, boot_buffer, 4);   //55 aa 33 cc
    touch_debug(DEBUG_MESSAGES, "[elan] %s :%x,%x,%x,%x\n",__func__,boot_buffer[0],boot_buffer[1],boot_buffer[2],boot_buffer[3]);

    /* Send Dummy Byte	*/
    res = i2c_master_send(private_ts->client, &data,  sizeof(data));
    if(res!=sizeof(data))
    {
        touch_debug(DEBUG_ERROR, "[elan] dummy error code = %d\n",res);
    }
    else
        touch_debug(DEBUG_ERROR, "[elan] Send Dummy Byte Success!!\n");

    /*check fw_source: 0 = fw_data.i; 1 = request_fw at /system; 2 = /data/local/tmp*/
    if(fw_source == 1) { 	//use request firmware at /system/etc/firmware/
        touch_debug(DEBUG_INFO, "[elan] request_firmware name = %s\n",ELAN_FW_FILENAME);
        rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &private_ts->client->dev);
        if (rc != 0) {
            touch_debug(DEBUG_ERROR,"[elan] rc=%d, request_firmware fail\n", rc);
            return -1;
        }
        else
            printk(KERN_DEBUG "[elan] Request Firmware Size=%zu\n", p_fw_entry->size);

        fw_data = p_fw_entry->data;
        fw_size = p_fw_entry->size;
        PageNum = (fw_size/sizeof(uint8_t)/PageSize);
    }
    else if(fw_source == 2) {  //use request firmware at /data/local/tmp
        printk(KERN_DEBUG "[elan] Update Firmware from /data/local/tmp/ElanFW.ekt\n");
        oldfs=get_fs();
        set_fs(KERNEL_DS);
        firmware_fp = filp_open("/data/local/tmp/ElanFW.ekt", O_RDONLY, S_IRUSR |S_IRGRP);
        if(PTR_ERR(firmware_fp) == -ENOENT) {
            touch_debug(DEBUG_ERROR, "[elan] open file error.\n");
            return -1;
        } else
            touch_debug(DEBUG_ERROR, "[elan] open file success.\n");
        PageNum=0;
        firmware_fp->f_pos = 0;

        for(pos = 0; pos < 1000*132; pos += 132,PageNum++) {
            if(firmware_fp->f_op->read(firmware_fp, firmware + pos, 132, &firmware_fp->f_pos) != 132) {
                break;
            }
            //touch_debug(DEBUG_ERROR, "[elan]pos = %d, PageNum = %d\n", pos, PageNum);
        }
        fw_data = firmware;
        //touch_debug(DEBUG_INFO, "[elan]%s: PageNUM %d, FW_Ver %x %x, FW_ID %x %x\n",__func__,PageNum,firmware[34058],firmware[34059],firmware[34586],firmware[34587]);
        touch_debug(DEBUG_INFO, "[elan]%s: ekt's PageNUM = %d, \n",__func__, PageNum);
        set_fs(oldfs);
        filp_close(firmware_fp, NULL);

        /*test*/
        //touch_debug(DEBUG_INFO, "[elan]%s: test3, return 1\n",__func__);
        //mutex_unlock(&ktf_mutex);
        //return 1;
    }
    else {		//use fw_data.i
        printk(KERN_DEBUG "[elan] Update Firmware by fw_data.i\n");
        PageNum = (sizeof(file_fw_data)/sizeof(uint8_t)/PageSize);
        fw_data = file_fw_data;
    }
    printk(KERN_DEBUG "[elan] PageNum = %d.\n", PageNum);
    /*end check firmware*/

    /* Start IAP*/
    for( iPage = 1; iPage <= PageNum; iPage++ )
    {
#if 1 // 8byte mode
        // 8 bytes
        //szBuff = fw_data + ((iPage-1) * PageSize);
        for(byte_count=1; byte_count<=17; byte_count++)
        {
            //touch_debug(DEBUG_INFO, "[elan] byte %d, curIndex = %d\n", byte_count, curIndex );
            szBuff = fw_data + curIndex;
            if(byte_count!=17)
            {
                //printk("curIndex =%d\n",curIndex);

                curIndex =  curIndex + 8;

                //ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
                res = WritePage(szBuff, 8);
            }
            else
			{
                //printk("curIndex =%d\n",curIndex);
                curIndex =  curIndex + 4;
				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 4);
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif
#if 0 	// 132byte mode
		//szBuff = fw_data + curIndex;
		//szBuff = firmware + curIndex;
		szBuff = fw_data + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif
#if 1 //if use old bootcode
		if(iPage==PageNum || iPage==1)
		{
			mdelay(600);
		}
		else
		{
			mdelay(50);
		}
#endif

		res = GetAckData(private_ts->client);

		if (ACK_OK != res)
		{
			mdelay(50);
			touch_debug(DEBUG_ERROR, "[elan] ERROR: GetAckData fail! res=%d\r\n", res);
			rewriteCnt = rewriteCnt + 1;
			if (rewriteCnt == PAGERETRY)
			{
				touch_debug(DEBUG_ERROR, "[elan] %dth page ReWrite %d times fails!\n", iPage, PAGERETRY);
				mutex_unlock(&ktf_mutex);
				return E_FD;
			}
			else
			{
				touch_debug(DEBUG_ERROR, "[elan] %d page ReWrite %d times!\n",  iPage, rewriteCnt);
				goto IAP_RESTART;
			}
		}
		else
		{
			rewriteCnt = 0;
			print_progress(iPage,ic_num,i);
			if (iPage == PageNum)
				touch_debug(DEBUG_ERROR, "[elan] %s Firmware Update Successfully!\n", __func__);
		}
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	elan_ktf_ts_hw_reset();   //20160114 added, to reset ic
	mdelay(100);
	RECOVERY=0;
	res = __hello_packet_handler(private_ts->client);

	if(res == 0x80)
		touch_debug(DEBUG_ERROR, "[elan] Recovery mode! res = %02x.\n", res);
	else
		touch_debug(DEBUG_ERROR, "[elan] hello_packet_handler() res = %d.\n", res);

	res = __fw_packet_handler(private_ts->client);
	if (res < 0)
		touch_debug(DEBUG_ERROR, "[elan] res = %d, Get Firmware information error, maybe received by system interrupt!\n", res);

	touch_debug(DEBUG_ERROR, "[elan] fw_update end.\n");
	mutex_unlock(&ktf_mutex);
	return res;   /* 0:sucessfully, 0x80: Recovery, -1: No response */
}

#if 0
static int Update_FW_in_Driver(void *x)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	//struct timeval tv1, tv2;

	uint8_t boot_buffer[4] = {0};
	//int byte_count;

	uint8_t *szBuff = NULL;
	int curIndex = 0;
	mutex_lock(&ktf_mutex);
    touch_debug(DEBUG_INFO, "[elan] %s: Update FW\n", __func__);
	IAP_RESTART:	
	
	curIndex=0;
	data=I2C_DATA[0];//Master
    touch_debug(DEBUG_INFO, "[elan] %s: address data=0x%x \r\n", __func__, data);

	if(RECOVERY != 0x80)
	{
        touch_debug(DEBUG_MESSAGES, "[elan] Firmware upgrade normal mode !\n");
	} else
        touch_debug(DEBUG_MESSAGES, "[elan] Firmware upgrade recovery mode !\n");
	
	/*Send enter bootcode cmd*/
	elan_ktf_ts_hw_reset();
	mdelay(13); 
	res = EnterISPMode(private_ts->client);	 //enter ISP mode
	//elan_ktf_ts_poll(private_ts->client);
	mdelay(100); 
	/*check enter bootcode cmd*/
	res = i2c_master_recv(private_ts->client, boot_buffer, 4);   //55 aa 33 cc 
    touch_debug(DEBUG_MESSAGES, "[elan] %s :%x,%x,%x,%x\n",__func__,boot_buffer[0],boot_buffer[1],boot_buffer[2],boot_buffer[3]);
	
	/* Send Dummy Byte	*/
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
        touch_debug(DEBUG_ERROR, "[elan] dummy error code = %d\n",res);
	}	
    else
        touch_debug(DEBUG_ERROR, "[elan] Send Dummy Byte Success!!\n");
	/* Start IAP*/
	for( iPage = 1; iPage <= PageNum; iPage++ ) 
	{
#if 0 // 8byte mode
		// 8 bytes
		//szBuff = fw_data + ((iPage-1) * PageSize); 
		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{		
				//			printk("[ELAN] byte %d\n",byte_count);	
				//			printk("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 8;

				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 8);
			}
			else
			{
				//			printk("byte %d\n",byte_count);
				//			printk("curIndex =%d\n",curIndex);
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 4;
				//ioctl(fd, IOCTL_IAP_MODE_LOCK, data);
				res = WritePage(szBuff, 4); 
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif 
#if 1 // 132byte mode		
		//szBuff = file_fw_data + curIndex;
		szBuff = firmware + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif
		#if 1 //if use old bootcode
		if(iPage==PageNum || iPage==1)
		{
			mdelay(600); 			 
		}
		else
		{
			mdelay(50); 			 
		}	
		#endif
		
		res = GetAckData(private_ts->client);

		if (ACK_OK != res) 
		{
			mdelay(50); 
			touch_debug(DEBUG_ERROR, "[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			rewriteCnt = rewriteCnt + 1;
			if (rewriteCnt == PAGERETRY)
			{
				touch_debug(DEBUG_ERROR, "[ELAN] %dth page ReWrite %d times fails!\n", iPage, PAGERETRY);
				mutex_unlock(&ktf_mutex);
				return E_FD;
			}
			else
			{
				touch_debug(DEBUG_ERROR, "[ELAN] %d page ReWrite %d times!\n",  iPage, rewriteCnt);
				goto IAP_RESTART;
			}

		}
		else
		{       
			print_progress(iPage,ic_num,i);
		}
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	RECOVERY=0;
	res= __hello_packet_handler(private_ts->client);
	if (res > 0)
		touch_debug(DEBUG_ERROR, "[ELAN] Update ALL Firmware successfully!\n");

	mutex_unlock(&ktf_mutex);
	return res;   /* 0:sucessfully, 0x80: Recovery, -1: No response */
}
#endif

#endif
// End Firmware Update

// Star 2wireIAP which used I2C to simulate JTAG function
#ifdef ELAN_2WIREICE
static uint8_t file_bin_data[] = {
	//#include "2wireice.i"
};

int write_ice_status=0;
int shift_out_16(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbf,0xff};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}
int tms_reset(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0xff,0xff,0xff,0xbf};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int mode_gen(struct i2c_client *client){
	int res;
	int retry = 5;
	uint8_t buff[] = {0xff,0xff,0xff,0x31,0xb7,0xb7,0x7b,0xb7,0x7b,0x7b,0xb7,0x7b,0xf3,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xf1};
	uint8_t buff_1[] = {0x2a,0x6a,0xa6,0xa6,0x6e};
	char mode_buff[2]={0};
	do {
		res = i2c_master_send(client, buff,  sizeof(buff));
		if (res != sizeof(buff)) {
			touch_debug(DEBUG_ERROR,"[ELAN] ERROR: mode_gen write buff error, write  error. res=%d\r\n", res);
		}
		else{
			touch_debug(DEBUG_MESSAGES,"[ELAN] mode_gen write buff successfully.\r\n");
			break;
		}
		mdelay(20);
		retry -=1;
	} while(retry);
	res = i2c_master_recv(client, mode_buff, sizeof(mode_buff));
	if (res != sizeof(mode_buff)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: mode_gen read data error, write  error. res=%d\r\n", res);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[ELAN] mode gen read successfully(a6 59)! buff[0]=0x%x  buff[1]=0x%x \r\n", mode_buff[0], mode_buff[1]);

	res = i2c_master_send(client, buff_1,  sizeof(buff_1));
	if (res != sizeof(buff_1)) {
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: mode_gen write buff_1 error. res=%d\r\n", res);
		return -1;
	}
	return res;
}

int word_scan_out(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26,0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}

int long_word_scan_out(struct i2c_client *client){
	int res;
	uint8_t buff[] = {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26,0x66};
	res = i2c_master_send(client, buff,  sizeof(buff));
	return res;
}


int bit_manipulation(int TDI, int TMS, int TCK, int TDO,int TDI_1, int TMS_1, int TCK_1, int TDO_1){
	int res; 
	res= ((TDI<<3 |TMS<<2 |TCK |TDO)<<4) |(TDI_1<<3 |TMS_1<<2 |TCK_1 |TDO_1);
	return res;
}

int ins_write(struct i2c_client *client, uint8_t buf){
	int res=0;
	int length=13;
	uint8_t write_buf[7]={0};
	int TDI_bit[13]={0};
	int TMS_bit[13]={0};
	int i=0;
	uint8_t buf_rev=0;
	int TDI=0, TMS=0, TCK=0,TDO=0;
	int bit_tdi, bit_tms;
	int len;
	
	for(i=0;i<8;i++) 
	{
		buf_rev = buf_rev | (((buf >> i) & 0x01) << (7-i));
	}
	
	
	TDI = (0x7<<10) | buf_rev <<2 |0x00;
	TMS = 0x1007;
	TCK=0x2;
	TDO=1;
	
	for ( len=0; len<=length-1; len++){
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		TDI_bit[length-1-len] =bit_tdi;
		TMS_bit[length-1-len] = bit_tms;
		TDI = TDI >>1;
		TMS = TMS >>1;
	}

	for (len=0;len<=length-1;len=len+2){
		if (len == length-1 && len%2 ==0)
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0, 0, 0, 0); 	
		else
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO); 	
		write_buf[len/2] = res;
	}

	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}


int word_scan_in(struct i2c_client *client, uint16_t buf){
	int res=0;
	uint8_t write_buf[10]={0};
	int TDI_bit[20]={0};
	int TMS_bit[20]={0};
	
	
	int TDI =  buf <<2 |0x00;
	int  TMS = 0x7;
	int  TCK=0x2;
	int TDO=1;
	
	int bit_tdi, bit_tms;
	int len;
	
	for ( len=0; len<=19; len++){    //length =20
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		
		TDI_bit[19-len] =bit_tdi;
		TMS_bit[19-len] = bit_tms;
		TDI = TDI >>1;
		TMS = TMS >>1;
	}

	for (len=0;len<=19;len=len+2){
		if (len == 19 && len%2 ==0)
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0,0,0,0); 
		else
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO); 
		write_buf[len/2] = res;
	}

	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

int long_word_scan_in(struct i2c_client *client, int buf_1, int buf_2){
	uint8_t write_buf[18]={0};
	uint8_t TDI_bit[36]={0};
	uint8_t TMS_bit[36]={0};

	int TDI_1 = buf_1;
	int TDI_2 = (buf_2<<2) |0x00;
	int TMS = 0x7;
	int TCK=0x2;
	int TDO=1;
	
	int bit_tdi, bit_tms;
	int len=0;
	int res=0;


	for ( len=0; len<=35; len++){    //length =36

		if(len<18)
		{
			bit_tdi = TDI_2 & 0x1;
		}
		else
		{
			bit_tdi = TDI_1 & 0x1;
		}
		bit_tms = TMS & 0x1;
		
		TDI_bit[35-len] =bit_tdi;
		TMS_bit[35-len] = bit_tms;
		if(len<18)
		{
			TDI_2 = TDI_2 >>1;
		}
		else
		{
			TDI_1 = TDI_1 >>1;
		}
		TMS = TMS >>1;
		bit_tdi=0;
		bit_tms=0;
	}


	for (len=0;len<=35;len=len+2){
		if (len == 35 && len%2 ==0)
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, 0,0,0,1);
		else
		res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK, TDO, TDI_bit[len+1], TMS_bit[len+1], TCK, TDO);
		write_buf[len/2] = res;
	}
	
	res = i2c_master_send(client, write_buf,  sizeof(write_buf));
	return res;
}

uint16_t trimtable[8]={0};

int Read_SFR(struct i2c_client *client, int open){
	uint8_t voltage_recv[2]={0};
	
	int count, ret;
	//uint16_t address_1[8]={0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007};
	
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);  
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	//  0
	ins_write(client, 0x6f);  //IO Write
	long_word_scan_in(client, 0x007f, 0x9002);  //TM=2h
	ins_write(client, 0x68);  //Program Memory Sequential Read
	word_scan_in(client, 0x0000);  //set Address 0x0000
	shift_out_16(client);   //move data to I2C buf
	
	mdelay(10);
	count = 0;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  1
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0001);
	shift_out_16(client); 

	mdelay(1);
	count=1;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  2
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0002);
	shift_out_16(client); 

	mdelay(1);
	count=2;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  3
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0003);
	shift_out_16(client); 

	mdelay(1);
	count=3;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  4
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0004);
	shift_out_16(client); 

	mdelay(1);
	count=4;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}


	//  5
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0005);
	shift_out_16(client); 

	mdelay(1);
	count=5;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}

	
	//  6
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0006);
	shift_out_16(client); 

	mdelay(1);
	count=6;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	else
	{
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
		touch_debug(DEBUG_MESSAGES,"[ELAN] read data successfully! voltage_recv buff[0]=0x%x  buff[1]=0x%x  trimtable[%d]=0x%x\r\n", voltage_recv[0], voltage_recv[1], count, trimtable[count]);
	}
	//  7
	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0007);
	shift_out_16(client); 

	mdelay(1);
	count=7;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
	if (ret != sizeof(voltage_recv)) {
		touch_debug(DEBUG_ERROR,"[ELAN] %s: read data error. ret=%d\r\n", __func__, ret);
		return -1;
	}
	if (open == 1)
	trimtable[count]=voltage_recv[0]<<8 |  (voltage_recv[1] & 0xbf);
	else
	trimtable[count]=voltage_recv[0]<<8 | (voltage_recv[1] | 0x40);
	touch_debug(DEBUG_ERROR,"[ELAN] Open_High_Voltage recv  voltage_recv buff[0]=%x buff[1]=%x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]);


	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);


	
	/*	
	for (count =0; count <8; count++){

	ins_write(client, 0x6f); // IO write
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, address_1[count]);
		shift_out_16(client); 

	mdelay(10);
	//count=6;
		ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv)); 
		trimtable[count]=voltage_recv[0]<<8 | voltage_recv[1];
	printk("[elan] Open_High_Voltage recv -1 1word =%x %x, trimtable[%d]=%x \n", voltage_recv[0],voltage_recv[1], count, trimtable[count]); 

	}
	
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

*/	
	return 0;
}

int Write_SFR_2k(struct i2c_client *client, int open){

	//set page 1
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x0001, 0x0100);
	if(open==1)
	{
		//set HV enable
		touch_debug(DEBUG_MESSAGES,"%s set HV enable\n",__func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc041);
	}
	else
	{
		//set HV disable
		touch_debug(DEBUG_MESSAGES,"%s set HV disable\n",__func__);
		ins_write(client, 0x6f);
		long_word_scan_in(client, 0x0050, 0xc040);
	}
	return 0;
}

int Write_SFR(struct i2c_client *client){

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9001);


	ins_write(client, 0x66);  // Program Memory Write
	long_word_scan_in(client, 0x0000, trimtable[0]);
	ins_write(client, 0xfd);  //Set up the initial addr for sequential access
	word_scan_in(client,0x7f);
	
	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0001, trimtable[1]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0002, trimtable[2]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0003, trimtable[3]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0004, trimtable[4]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0005, trimtable[5]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);
	
	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0006, trimtable[6]);	
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0007, trimtable[7]);
	ins_write(client, 0xfd);
	word_scan_in(client,0x7f);


	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x7f, 0x8000);	   
	/*
	for (count=0;count<8;count++){
			ins_write(client, 0x66);
		long_word_scan_in(client, 0x0000+count, trimtable[count]);
		
	}
	*/

	return 0;
}

int Enter_Mode(struct i2c_client *client){
	mode_gen(client);
	tms_reset(client);
	ins_write(client,0xfc); //system reset
	tms_reset(client);
	return 0;
}
int Open_High_Voltage(struct i2c_client *client, int open){
#ifdef EKTF3K_FLASH
	Read_SFR(client, open);
	Write_SFR(client);
	Read_SFR(client, open);

#endif
	Write_SFR_2k(client, open);
	return 0;

}

int Mass_Erase(struct i2c_client *client){
	char mass_buff[4]={0};
	char mass_buff_1[2]={0};
	int ret, finish=0, i=0;
	touch_debug(DEBUG_MESSAGES,"[Elan] Mass_Erase!!!!\n");
	ins_write(client,0x01); //id code read
	mdelay(2);
	long_word_scan_out(client);

	ret = i2c_master_recv(client, mass_buff, sizeof(mass_buff));
	touch_debug(DEBUG_MESSAGES,"[elan] Mass_Erase mass_buff=%x %x %x %x(c0 08 01 00)\n", mass_buff[0],mass_buff[1],mass_buff[2],mass_buff[3]);  //id: c0 08 01 00
	/* / add for test
	ins_write(client, 0xf3);
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	printk("[elan] Mass_Erase mass_buff_1=%x %x(a0 00)\n", mass_buff_1[0],mass_buff_1[1]);  // a0 00 : stop
//add for test

	//read low->high 5th bit
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);

// add for test
	ins_write(client, 0xf3);
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
	printk("[elan] Mass_Erase (II) mass_buff_1=%x %x(40 00)\n", mass_buff_1[0],mass_buff_1[1]);  // 40 00
//add for test
	mdelay(10); //for malata
	*/
	
	ins_write(client,0x6f);  //IO Write
	/*add by herman*/
	long_word_scan_in(client,0x007e,0x0020);

	long_word_scan_in(client,0x007f,0x4000);//orig 4000
	long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x8000);
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x9040);
	ins_write(client,0x66); //Program data Write
	long_word_scan_in(client, 0x0000,0x8765);//change by herman
	ins_write(client,0x6f);  //IO Write
	long_word_scan_in(client, 0x007f,0x8000);	//clear flash control PROG

	ins_write(client,0xf3);
	
	while (finish==0){
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));			
		if (ret != sizeof(mass_buff_1)) {
			touch_debug(DEBUG_ERROR,"[ELAN] ERROR: read data error. res=%d\r\n", ret);
			return -1;
		}
		else
		{
			finish = (mass_buff_1[1] >> 4 ) & 0x01;
			touch_debug(DEBUG_MESSAGES,"[ELAN] mass_buff_1[0]=%x, mass_buff_1[1]=%x (80 10)!!!!!!!!!! finish=%d \n", mass_buff_1[0], mass_buff_1[1], finish);  //80 10: OK, 80 00: fail
		}
		if (mass_buff_1[1]!= I2C_DATA[0] && finish!=1 && i<100) {  
			mdelay(100);
			//printk("[elan] mass_buff_1[1] >>4  !=1\n");
			i++;
			if (i == 50) {
				touch_debug(DEBUG_ERROR,"[elan] Mass_Erase fail ! \n");
				//return -1;  //for test
			}
		}
		
	}

	return 0;
}

int Reset_ICE(struct i2c_client *client){
	//struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int res;
	touch_debug(DEBUG_INFO,"[Elan] Reset ICE!!!!\n");
	ins_write(client, 0x94);
	ins_write(client, 0xd4);
	ins_write(client, 0x20);
	client->addr = I2C_DATA[0];////Modify address before 2-wire
	elan_ktf_ts_hw_reset(); 
	mdelay(250);
	res = __hello_packet_handler(client);
	
	return 0;
}

int normal_write_func(struct i2c_client *client, int j, uint8_t *szBuff){
	//char buff_check=0;
	uint16_t szbuff=0, szbuff_1=0;
	uint16_t sendbuff=0;
	int write_byte, iw;
	
	ins_write(client,0xfd);
	word_scan_in(client, j*64); 
	
	ins_write(client,0x65);  //Program data sequential write

	write_byte =64;

	for(iw=0;iw<write_byte;iw++){ 
		szbuff = *szBuff;
		szbuff_1 = *(szBuff+1);
		sendbuff = szbuff_1 <<8 |szbuff;
		touch_debug(DEBUG_MESSAGES,"[elan]  Write Page sendbuff=0x%04x @@@\n", sendbuff);
		//mdelay(1);
		word_scan_in(client, sendbuff); //data????   buff_read_data
		szBuff+=2;
		
	}
	return 0;
}

int fastmode_write_func(struct i2c_client *client, int j, uint8_t *szBuff){
	uint8_t szfwbuff=0, szfwbuff_1=0;
	uint8_t sendfwbuff[130]={0};
	uint8_t tmpbuff;
	int i=0, len=0;
	private_ts->client->addr = 0x76;

	sendfwbuff[0] = (j*64)>>8;
	tmpbuff = ((j*64)<< 8) >> 8;
	sendfwbuff[1] = tmpbuff;
	//printk("fastmode_write_func, sendfwbuff[0]=0x%x, sendfwbuff[1]=0x%x\n", sendfwbuff[0], sendfwbuff[1]);

	for (i=2;i < 129; i=i+2) {      //  1 Page = 64 word, 1 word=2Byte
		
		szfwbuff = *szBuff;
		szfwbuff_1 = *(szBuff+1);
		sendfwbuff[i] = szfwbuff_1;
		sendfwbuff[i+1] = szfwbuff;
		szBuff+=2;
		//printk("[elan] sendfwbuff[%d]=0x%x, sendfwbuff[%d]=0x%x\n", i, sendfwbuff[i], i+1, sendfwbuff[i+1]);
	}

	
	len = i2c_master_send(private_ts->client, sendfwbuff,  130);
	if (len != 130) {  //address+data(128)
		touch_debug(DEBUG_ERROR,"[ELAN] ERROR: fastmode write page error, write error. len=%d, Page %d\r\n", len, j);
		return -1;
	}
	//printk("fastmode_write_func, send len=%d (130), Page %d --\n", len, j);

	private_ts->client->addr = 0x77;
	
	return 0;
}


int ektSize;
int lastpage_byte;
int lastpage_flag=0;
int Write_Page(struct i2c_client *client, int j, uint8_t *szBuff){
	int len, finish=0;
	char buff_read_data[2];
	int i=0;
	
	ins_write(client,0x6f);   //IO Write
	//long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x8000);
	long_word_scan_in(client,0x007f,0x9400);

	ins_write(client,0x66);    //Program Data Write
	//long_word_scan_in(client,0x0000,0x5a5a);
	long_word_scan_in(client, j*64,0x0000);
	//printk("[elan] j*64=0x%x @@ \n", j*64);
	//long_word_scan_in(client, j*64,0x5a5a);  //set ALE
	
	//normal_write_func(client, j, szBuff); ////////////choose one : normal / fast mode
	fastmode_write_func(client, j, szBuff); //////////
	
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x9000);

	//ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x8000);

	ins_write(client, 0xf3);  //Debug Reg Read
	
	while (finish==0){
		word_scan_out(client);
		len=i2c_master_recv(client, buff_read_data, sizeof(buff_read_data));
		if (len != sizeof(buff_read_data)) {
			touch_debug(DEBUG_ERROR,"[ELAN] ERROR: Write_Page read buff_read_data error, len=%d\r\n", len);
			return E_FD;
		}
		else
		{
			finish = (buff_read_data[1] >> 4 ) & 0x01;
			//printk("[ELAN] read data successfully! buff[0]=0x%x  buff[1]=0x%x  finish=%d\r\n", buff_read_data[0], buff_read_data[1], finish);  //80 10: ok
		}
		if (finish!=1) {  
			mdelay(10);
			//printk("[elan] Write_Page finish !=1\n");
			i++;
			if (i==50){ 
				touch_debug(DEBUG_ERROR,"[elan] Write_Page finish !=1, Page=%d\n", j);
				write_ice_status=1;
				return -1;
			}
		}

	}
	return 0;
}

int fastmode_read_func(struct i2c_client *client, int j, uint8_t *szBuff){
	uint8_t szfrbuff=0, szfrbuff_1=0;
	uint8_t sendfrbuff[2]={0};
	uint8_t recvfrbuff[130]={0};
	uint16_t tmpbuff;
	int i=0, len=0, retry=0;

	
	ins_write(client,0x67);
	
	private_ts->client->addr = 0x76;

	sendfrbuff[0] = (j*64)>>8;
	tmpbuff = ((j*64)<< 8) >> 8;
	sendfrbuff[1] = tmpbuff;
	//printk("fastmode_write_func, sendfrbuff[0]=0x%x, sendfrbuff[1]=0x%x\n", sendfrbuff[0], sendfrbuff[1]);
	len = i2c_master_send(private_ts->client, sendfrbuff,  sizeof(sendfrbuff));

	len = i2c_master_recv(private_ts->client, recvfrbuff,  sizeof(recvfrbuff));
	//printk("fastmode_read_func, recv len=%d (128)\n", len);
	
	for (i=2;i < 129;i=i+2){ 
		szfrbuff=*szBuff;
		szfrbuff_1=*(szBuff+1);
		szBuff+=2;
		if (recvfrbuff[i] != szfrbuff_1 || recvfrbuff[i+1] != szfrbuff)  
		{
			touch_debug(DEBUG_ERROR,"[elan] @@@@Read Page Compare Fail. recvfrbuff[%d]=%x, recvfrbuff[i+1]=%x, szfrbuff_1=%x, szfrbuff=%x, ,j =%d@@@@@@@@@@@@@@@@\n\n", i,recvfrbuff[i], recvfrbuff[i+1], szfrbuff_1, szfrbuff, j);
			write_ice_status=1;
			retry=1;
		}
		break;//for test
	}

	private_ts->client->addr = 0x77;
	if(retry==1)
	{
		return -1;
	}
	else
	return 0;
}


int normal_read_func(struct i2c_client *client, int j,  uint8_t *szBuff){
	char read_buff[2];
	int m, len, read_byte;
	uint16_t szbuff=0, szbuff_1=0;
	
	ins_write(client,0xfd);
	
	//printk("[elan] Read_Page, j*64=0x%x\n", j*64);
	word_scan_in(client, j*64);
	ins_write(client,0x67);

	word_scan_out(client);

	read_byte=64;
	//for(m=0;m<64;m++){
	for(m=0;m<read_byte;m++){
		// compare......
		word_scan_out(client);
		len=i2c_master_recv(client, read_buff, sizeof(read_buff));

		szbuff=*szBuff;
		szbuff_1=*(szBuff+1);
		szBuff+=2;
		touch_debug(DEBUG_MESSAGES,"[elan] Read Page: byte=%x%x, szbuff=%x%x \n", read_buff[0], read_buff[1],szbuff, szbuff_1);

		if (read_buff[0] != szbuff_1 || read_buff[1] != szbuff) 
		{
			touch_debug(DEBUG_ERROR,"[elan] @@@@@@@@@@Read Page Compare Fail. j =%d. m=%d.@@@@@@@@@@@@@@@@\n\n", j, m);
			write_ice_status=1;
		}
	}
	return 0;
}


int Read_Page(struct i2c_client *client, int j,  uint8_t *szBuff){
	int res=0;
	ins_write(client,0x6f);
	//long_word_scan_in(client,0x007e,0x0023);
	long_word_scan_in(client,0x007f,0x9000);
	ins_write(client,0x68);

	//mdelay(10); //for malata
	//normal_read_func(client, j,  szBuff); ////////////////choose one: normal / fastmode
	fastmode_read_func(client, j,  szBuff);

	//Clear Flashce
	ins_write(client,0x6f);
	long_word_scan_in(client,0x007f,0x0000);
	if(res==-1)
	{
		return -1;
	}
	return 0;
	
}



int TWO_WIRE_ICE(struct i2c_client *client){
	int i;

	uint8_t *szBuff = NULL;
	//char szBuff[128]={0};
	int curIndex = 0;
	int PageSize=128;
	int res;
	//int ektSize;
	//test
	write_ice_status=0;
	ektSize = sizeof(file_bin_data) /PageSize;
	client->addr = 0x77;////Modify address before 2-wire

	touch_debug(DEBUG_INFO,"[Elan] ektSize=%d ,modify address = %x\n ", ektSize, client->addr);

	i = Enter_Mode(client);
	i = Open_High_Voltage(client, 1);     
	if (i == -1)
	{
		touch_debug(DEBUG_ERROR,"[Elan] Open High Voltage fail\n");
		return -1;
	}
	//return 0;

	i = Mass_Erase(client);  //mark temp
	if (i == -1)  {
		touch_debug(DEBUG_ERROR,"[Elan] Mass Erase fail\n");
		return -1;
	}


	//for fastmode
	ins_write(client,0x6f);
	long_word_scan_in(client, 0x007e, 0x0036);
	long_word_scan_in(client, 0x007f, 0x8000);
	long_word_scan_in(client, 0x007e, 0x0023);	//add by herman


	// client->addr = 0x76;////Modify address before 2-wire 
	touch_debug(DEBUG_INFO,"[Elan-test] client->addr =%2x\n", client->addr);    
	//for fastmode
	for (i =0 ; i<ektSize; i++)
	{
		szBuff = file_bin_data + curIndex; 
		curIndex =  curIndex + PageSize; 	
		//	printk("[Elan] Write_Page %d........................wait\n ", i);	

		res=Write_Page(client, i, szBuff);
		if (res == -1) 
		{
			touch_debug(DEBUG_ERROR,"[Elan] Write_Page %d fail\n ", i);
			break;
		}
		//printk("[Elan] Read_Page %d........................wait\n ", i);
		mdelay(1);
		Read_Page(client,i, szBuff);
		//printk("[Elan] Finish  %d  Page!!!!!!!.........wait\n ", i);	
	}


	if(write_ice_status==0)
	{
		touch_debug(DEBUG_INFO,"[elan] Update_FW_Boot Finish!!! \n");
	}
	else
	{
		touch_debug(DEBUG_INFO,"[elan] Update_FW_Boot fail!!! \n");
	}

	i = Open_High_Voltage(client, 0);     
	if (i == -1) return -1; //test

	Reset_ICE(client);

	return 0;	
}

int elan_TWO_WIRE_ICE( struct i2c_client *client) // for driver internal 2-wire ice
{
	work_lock=1;
	disable_irq(private_ts->client->irq);
	//wake_lock(&private_ts->wakelock);
	TWO_WIRE_ICE(client);
	work_lock=0;
	enable_irq(private_ts->client->irq);
	//wake_unlock(&private_ts->wakelock);
	return 0;
}
// End 2WireICE
#endif


int CheckISPstatus(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t checkstatus[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x18}; 
	uint8_t buff[67] = {0};

	len = i2c_master_send(private_ts->client, checkstatus, sizeof(checkstatus));
	if (len != sizeof(checkstatus)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n", checkstatus[0], checkstatus[1], checkstatus[2], checkstatus[3], checkstatus[4], checkstatus[5]);
	
	mdelay(10);	
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address Read Data error. len=%d \r\n", len);
		return -1;
	}
	else {
        printk("[elan][Check status]: ");
for (j=0; j<37; j++)
	printk("%x ", buff[j]);
printk("\n");
		if (buff[6] == 0x80)	return 0x80; /* return recovery mode 0x88 */
	}
	
	return 0;
}

int HID_RecoveryISP(struct i2c_client *client)
{
	int len = 0;
	int j;
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0xc0, 0xe1, 0x5a}; 
//	uint8_t isp_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04, 0x54, 0x00, 0x12, 0x34}; 
	uint8_t check_addr[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x01, 0x10}; 
	uint8_t buff[67] = {0};


	len = i2c_master_send(private_ts->client, flash_key,  37);
	if (len != 37) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Flash key fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] FLASH key write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", flash_key[7], flash_key[8], flash_key[9], flash_key[10]);

	mdelay(20);
/*
        len = i2c_master_send(private_ts->client, isp_cmd,  37);
	if (len != 37) {
		touch_debug(DEBUG_ERROR,"[elan] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		touch_debug(DEBUG_MESSAGES,"[elan] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[7], isp_cmd[8], isp_cmd[9], isp_cmd[10]);
*/

	mdelay(20);
 	len = i2c_master_send(private_ts->client, check_addr,  sizeof(check_addr));
	if (len != sizeof(check_addr)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] Check Address write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", check_addr[7], check_addr[8], check_addr[9], check_addr[10]);
	
	mdelay(20);	
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Check Address Read Data error. len=%d \r\n", len);
		return -1;
	}
	else {
        printk("[elan][Check Addr]: ");
for (j=0; j<37; j++)
	printk("%x ", buff[j]);
printk("\n");
	
	}

	return 0;
}

int SendEndCmd(struct i2c_client *client)
{
	int len = 0;
	uint8_t send_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x1A}; 

	len = i2c_master_send(private_ts->client, send_cmd, sizeof(send_cmd));
	if (len != sizeof(send_cmd)) {
        touch_debug(DEBUG_ERROR,"[elan] ERROR: Send Cmd fail! len=%d\r\n", len);
		return -1;
	}
	else
        touch_debug(DEBUG_MESSAGES,"[elan] check status write data successfully! cmd = [%x, %x, %x, %x, %x, %x]\n", send_cmd[0], send_cmd[1], send_cmd[2], send_cmd[3], send_cmd[4], send_cmd[5]);
	
	
	return 0;
}

#if 0
static int HID_FW_Update(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0; /* rewriteCnt = 0; rewriteCnt for PAGE_REWRITE */
	int j = 0; // i=0;
	int write_times = 142;
//	int write_bytes = 12;
	uint8_t data;
//	int restartCnt = 0; // For IAP_RESTART
	int byte_count;
	const u8 *szBuff = NULL;
	u8 write_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x21, 0x00, 0x00, 0x28};
	u8 cmd_iap_write[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x22};
	int curIndex = 0;
	int offset = 0;
	// 0x54, 0x00, 0x12, 0x34
	int rc, fw_size;
	
	/* Star Request Firmware */
	const u8 *fw_data;
	const struct firmware *p_fw_entry;
	
	touch_debug(DEBUG_INFO, "Request_firmware name = %s\n",ELAN_FW_FILENAME);
	rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &client->dev);
	if (rc != 0) {
		touch_debug(DEBUG_ERROR,"rc=%d, Request_firmware fail\n", rc);
		return -1;
	} else
	PageNum=473;
	touch_debug(DEBUG_INFO,"Firmware Size=%zu, PageNum=%d\n", p_fw_entry->size,PageNum);

	fw_data = p_fw_entry->data;
	fw_size = p_fw_entry->size;                                                
	/* End Request Firmware */                             
	
    touch_debug(DEBUG_INFO, "[elan] %s:  ic_num=%d\n", __func__, ic_num);

	data=I2C_DATA[0];//Master
    touch_debug(DEBUG_INFO, "[elan] %s: address data=0x%x \r\n", __func__, data);

	elan_ktf_ts_hw_reset();
	mdelay(200); 
	
	res = CheckISPstatus(private_ts->client);
	mdelay(20);
	if (res == 0x80) { /* 0x88 recovery mode  */
        	elan_ktf_ts_hw_reset();
		mdelay(200); 
		printk("[elan hid iap] Recovery mode\n");
		res = HID_RecoveryISP(private_ts->client);
	}
	else {
		printk("[elan hid iap] Normal mode\n");
		res = HID_EnterISPMode(private_ts->client);   //enter HID ISP mode
	}

	mdelay(50);

	// Start HID IAP
	for( iPage = 1; iPage <= 473; iPage +=30 )
	{
		offset=0;
		if (iPage == 451 ) {
			write_times = 109;
			//write_bytes = 18;
		}
		else {	
			write_times = 142;
			//write_bytes = 12;
		}
		mdelay(5);
		for(byte_count=1; byte_count <= write_times; byte_count++)
		{
			mdelay(5);
			if(byte_count != write_times)
			{
				szBuff = fw_data + curIndex;
				write_buf[8] = 28;
				write_buf[7] = offset & 0x00ff;
				write_buf[6] = offset >> 8;
				offset +=28;
				curIndex =  curIndex + 28;
				for (j=0; j<28; j++)
					write_buf[j+9]=szBuff[j];
/*
if (iPage == 451) {
printk("[hid_iap] P=%d i=%d: ", iPage,byte_count);
for (j=0; j<37; j++)
	printk("%x ", write_buf[j]);
printk("\n");
} */
				res = WritePage(write_buf, 37);
			}
			else
			{

				szBuff = fw_data + curIndex;
				write_buf[8] = 12;
				write_buf[7] = offset & 0x00ff;
				write_buf[6] = offset >> 8;
				curIndex =  curIndex + 12;
				for (j=0; j<12; j++)
					write_buf[j+9]=szBuff[j];

printk("[elan hid iap packet] iPage=%d i=%d times=%d data ", iPage,byte_count, write_times);
for (j=0; j<37; j++)
	printk("%x ", write_buf[j]);
printk("\n");

				res = WritePage(write_buf, 37);
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)


	   mdelay(10);
/*
printk("[elan iap write] cmd ");
for (j=0; j<37; j++)
	printk("%x ", cmd_iap_write[j]);
printk("\n");
*/
	   res = WritePage(cmd_iap_write, 37);
	   mdelay(200);
	   res = GetAckData(private_ts->client);
	   mdelay(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)



	res = SendEndCmd(private_ts->client);

	mdelay(200);
	elan_ktf_ts_hw_reset();
	mdelay(200);
	res = elan_ktf_ts_calibrate(private_ts->client); 
	mdelay(100);
	
    touch_debug(DEBUG_INFO,"[elan] Update Firmware successfully!\n");

	return res;
}

#endif

#if 0
static int FW_Update(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	int restartCnt = 0; // For IAP_RESTART
	int byte_count;
	const u8 *szBuff = NULL;
	int curIndex = 0;
	// 0x54, 0x00, 0x12, 0x34
	int rc, fw_size;
	
	/* Star Request Firmware */
	const u8 *fw_data;
	const struct firmware *p_fw_entry;
	
	touch_debug(DEBUG_INFO, "request_firmware name = %s\n",ELAN_FW_FILENAME);
	rc = request_firmware(&p_fw_entry, ELAN_FW_FILENAME, &client->dev);
	if (rc != 0) {
		touch_debug(DEBUG_ERROR,"rc=%d, request_firmware fail\n", rc);
		return -1;
	} else
	touch_debug(DEBUG_INFO,"Firmware Size=%zu\n", p_fw_entry->size);

	fw_data = p_fw_entry->data;
	fw_size = p_fw_entry->size;                                                
	/* End Request Firmware */                             
	
    touch_debug(DEBUG_INFO, "[elan] %s:  ic_num=%d\n", __func__, ic_num);
	IAP_RESTART:

	data=I2C_DATA[0];//Master
    touch_debug(DEBUG_INFO, "[elan] %s: address data=0x%x \r\n", __func__, data);

	if(recovery != 0x80)
	{
        touch_debug(DEBUG_INFO,"[elan] Firmware upgrade normal mode !\n");
		elan_ktf_ts_hw_reset();
		res = EnterISPMode(private_ts->client);   //enter ISP mode
	} else
        touch_debug(DEBUG_INFO,"[elan] Firmware upgrade recovery mode !\n");


    touch_debug(DEBUG_INFO,"[elan] send one byte data:%x,%x",private_ts->client->addr,data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
        printk("[elan] dummy error code = %d\n",res);
        touch_debug(DEBUG_ERROR,"[elan] dummy error code = %d\n",res);
	}
	mdelay(10);
	// Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ )
	{
		PAGE_REWRITE:

		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{
				szBuff = fw_data + curIndex;
				curIndex =  curIndex + 8;
				res = WritePage(szBuff, 8);
			}
			else
			{

				szBuff = fw_data + curIndex;
				curIndex =  curIndex + 4;
				res = WritePage(szBuff, 4);
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
		if(iPage==377 || iPage==1)
		{
			mdelay(600);
		}
		else
		{
			mdelay(50);
		}
		res = GetAckData(private_ts->client);

		if (ACK_OK != res)
		{
			mdelay(50);
            touch_debug(DEBUG_ERROR,"[elan] ERROR: GetAckData fail! res=%d\r\n", res);
			if ( res == ACK_REWRITE )
			{
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY)
				{
                    touch_debug(DEBUG_ERROR,"[elan] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					return E_FD;
				}
				else
				{
                    touch_debug(DEBUG_ERROR,"[elan] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			}
			else
			{
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5)
				{
                    touch_debug(DEBUG_ERROR,"[elan] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					return E_FD;
				}
				else
				{
                    printk("[elan] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
                    touch_debug(DEBUG_ERROR,"[elan] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		}
		else
		{
			rewriteCnt=0;
			print_progress(iPage,ic_num,i);
		}

		mdelay(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	res= __hello_packet_handler(private_ts->client);
    if(res > 0)	touch_debug(DEBUG_INFO,"[elan] Update Firmware successfully!\n");

	return res;
}
#endif

/*
static ssize_t set_debug_mesg(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int level[SYSFS_MAX_LEN];
	sscanf(buf,"%x",&level[0]);
	debug=level[0];
	touch_debug(DEBUG_INFO, "debug level %d, size %d\n", debug,count);
	return count;
}

static ssize_t show_debug_mesg(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Debug level %d \n", debug);
}*/

static ssize_t show_gpio_int(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(private_ts->intr_gpio));
}

static ssize_t show_reset(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	elan_ktf_ts_hw_reset();

	return sprintf(buf, "Reset Touch Screen Controller \n");
}

static ssize_t store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	elan_ktf_ts_hw_reset();
	printk("[elan] %s: Reset Touch Screen Controller.\n",__func__);
	return count;
}

static ssize_t show_enable_irq(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//	struct i2c_client *client = to_i2c_client(dev);
	//	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);

	return sprintf(buf, "Enable IRQ \n");
}

static ssize_t store_enable_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	work_lock=0;
    enable_irq(private_ts->client->irq);
    wake_unlock(&private_ts->wakelock);

	printk("[elan] %s: Enable IRQ.\n",__func__);
	return count;
}

static ssize_t show_disable_irq(struct device *dev,
struct device_attribute *attr, char *buf)
{
	//	struct i2c_client *client = to_i2c_client(dev);
	//	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);

	return sprintf(buf, "Disable IRQ \n");
}

static ssize_t store_disable_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	work_lock=1;
    disable_irq(private_ts->client->irq);
    wake_lock(&private_ts->wakelock);

	printk("[elan] %s: Disable IRQ.\n",__func__);
	return count;
}

static ssize_t show_calibrate(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	ret = elan_ktf_ts_calibrate(client);
	return sprintf(buf, "%s\n", (ret == 0) ? " [elan] Calibrate Finish" : "[elan] Calibrate Fail");
}

static ssize_t store_calibrate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
    int ret = 0;

    ret = elan_ktf_ts_calibrate(client);

	if (ret == 0)
		touch_debug(DEBUG_INFO, " [elan] Calibrate Finish!");
    else
        touch_debug(DEBUG_INFO, "[elan] Calibrate Fail!");

	return count;
}

static ssize_t show_fw_update_in_driver(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
	//struct i2c_client *client = to_i2c_client(dev);

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);
	
    //ret = Update_FW_in_Driver(0);
    ret = Update_FW_One(0);
	
	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);
	
    return sprintf(buf, "[elan] Update Firmware in driver used fw_data.i\n");
}

static ssize_t store_fw_update_in_driver(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	/* Compare current FW version with newest FW version */
	if (private_ts->fw_ver < NEWEST_FW_VER) {
		work_lock=1;
		disable_irq(private_ts->client->irq);
		wake_lock(&private_ts->wakelock);

		ret = Update_FW_One(0);

		work_lock=0;
		enable_irq(private_ts->client->irq);
		wake_unlock(&private_ts->wakelock);

		printk("[elan] %s: Update Firmware in driver used fw_data.i\n", __func__);
		return count;
	} else{
		printk("[elan]%s: Current FW is the newest version %0x \n", __func__, private_ts->fw_ver);
		return count;
	}
}

#if 0
static ssize_t show_hid_fw_update(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);
	
	ret = HID_FW_Update(client, 0);
	
	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);
	
	return sprintf(buf, "HID Update Firmware\n");
}
#endif

static ssize_t show_fw_update(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
    //struct i2c_client *client = to_i2c_client(dev);

    work_lock=1;
    disable_irq(private_ts->client->irq);
    wake_lock(&private_ts->wakelock);

    //ret = FW_Update(client, 0);
    ret = Update_FW_One(1);  //use request firmware

    work_lock=0;
    enable_irq(private_ts->client->irq);
    wake_unlock(&private_ts->wakelock);

    return sprintf(buf, "Update Firmware\n");
}

static ssize_t store_fw_update(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

    work_lock=1;
    disable_irq(private_ts->client->irq);
    wake_lock(&private_ts->wakelock);

    ret = Update_FW_One(1);  //use request firmware

    work_lock=0;
    enable_irq(private_ts->client->irq);
    wake_unlock(&private_ts->wakelock);

	printk("[elan] %s: Update Firmware at /system/etc/firmware/\n", __func__);
	return count;
}

#if 0
static ssize_t show_fw_update_elan(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    int ret;

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);
	
    ret = Update_FW_One(2);  //firmware is located at /data/local/tmp
	
	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);
	
	return sprintf(buf, "Update Firmware\n");
}
#endif

#if 0
static ssize_t store_fw_update_elan(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

    work_lock=1;
    disable_irq(private_ts->client->irq);
    wake_lock(&private_ts->wakelock);

    ret = Update_FW_One(2);  //firmware is located at /data/local/tmp

    work_lock=0;
    enable_irq(private_ts->client->irq);
    wake_unlock(&private_ts->wakelock);

	printk("[elan] %s: Update Firmware at /data/local/tmp\n", __func__);
	return count;
}
#endif

#ifdef ELAN_2WIREICE
static ssize_t show_2wire(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	work_lock=1;
	disable_irq(private_ts->client->irq);
	wake_lock(&private_ts->wakelock);
	
	ret = TWO_WIRE_ICE(client);
	
	work_lock=0;
	enable_irq(private_ts->client->irq);
	wake_unlock(&private_ts->wakelock);
	
	return sprintf(buf, "Update Firmware by 2wire JTAG\n");
}
#endif

static ssize_t show_fw_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "FW VER = %0*x\n", 4, ts->fw_ver);
}

static ssize_t show_fw_id_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

    return sprintf(buf, "FW ID = %0*x\n", 4, ts->fw_id);
}

static ssize_t show_bc_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

    return sprintf(buf, "BC VER = %0*x\n", 4, ts->bc_ver);
}

static ssize_t show_drv_version_value(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "Elan driver version 0x0006");
}

static ssize_t show_iap_mode(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n", 
	(ts->fw_ver == 0) ? "Recovery" : "Normal");
}

static ssize_t show_mt_protocol(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Current MT protocol %s\n", CURRENT_MT_PROTOCOL);
}

static ssize_t show_power_mode(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n",
	(ts->power_mode == POWER_MODE_NORMAL) ? "Normal" : "Idle");
}

#ifdef FEATURE_CHECK_TRUE_INTERRUPT
static ssize_t show_int_count(struct device *dev,
struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "touchCNT =%d, 78intCNT = %d\n", touchINT_cnt, IamAlive_PKT_INT_cnt);
}
#endif



static ssize_t show_goto_idle_mode(struct device *dev,
struct device_attribute *attr, char *buf)
{
    char idle_cmd[] = {0x54, 0x52, 0x00, 0x01};
	struct i2c_client *client = to_i2c_client(dev);


    int len = i2c_master_send(client, idle_cmd, sizeof(idle_cmd)/sizeof(idle_cmd[0]));
    if(len != sizeof(idle_cmd)/sizeof(idle_cmd[0]))
        {
            return sprintf(buf, "%s\n", "Failed to send i2c cmd to enter idle mode\n");
        }
    return sprintf(buf, "%s\n", "Enter idle mode, check mode by cat power_mode!! \n");
}

#ifdef FEATURE_QUANTA_GESTURE_TO_BOOTLOADER
static ssize_t store_gestr_bld_ctl(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count
)
{
	if (!strncmp("1", buf, 1))
	{
		is_quanta_gesture_to_bld_enabled = true;
	}else if (!strncmp("0", buf, 1)){
		is_quanta_gesture_to_bld_enabled = false;
	}else {
		printk("[quanta]%s: unknown parameter %s \n", __func__, buf);
	}

	printk("[quanta]%s: is_quanta_gesture_to_bld_enabled %s \n",__func__,
			(is_quanta_gesture_to_bld_enabled == true) ? "enable":"disable");
	return count;
}

static ssize_t show_gestr_bld_ctl(struct device *dev,
struct device_attribute *attr, char *buf)
{
   return sprintf(buf, "%s\n", is_quanta_gesture_to_bld_enabled == true? "enable":"disable");
}
#endif

#ifdef FEATURE_QUANTA_GESTURE_TO_RECOVERY
static ssize_t store_gestr_rcvr_ctl(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count
)
{
	if (!strncmp("1", buf, 1))
	{
		is_quanta_gesture_to_rcvr_enabled = true;
	}else if (!strncmp("0", buf, 1)){
		is_quanta_gesture_to_rcvr_enabled = false;
	}else {
		printk("[quanta]%s: unknown parameter %s \n", __func__, buf);
	}

	printk("[quanta]%s: is_quanta_gesture_to_rcvr_enabled %s \n",__func__,
			(is_quanta_gesture_to_rcvr_enabled == true) ? "enable":"disable");
	return count;
}

static ssize_t show_gestr_rcvr_ctl(struct device *dev,
struct device_attribute *attr, char *buf)
{
   return sprintf(buf, "%s\n", is_quanta_gesture_to_rcvr_enabled == true? "enable":"disable");
}
#endif


//static DEVICE_ATTR(debug_mesg, S_IRUGO | S_IWUGO, show_debug_mesg, set_debug_mesg);
static DEVICE_ATTR(gpio_int, S_IRUGO, show_gpio_int, NULL);
static DEVICE_ATTR(reset, /*S_IRUGO | S_IWUGO*/0660, show_reset, store_reset);
static DEVICE_ATTR(enable_irq, /*S_IRUGO | S_IWUGO*/0660, show_enable_irq, store_enable_irq);
static DEVICE_ATTR(disable_irq, /*S_IRUGO | S_IWUGO*/0660, show_disable_irq, store_disable_irq);
static DEVICE_ATTR(calibrate, /*S_IRUGO | S_IWUGO*/0660, show_calibrate, store_calibrate);
static DEVICE_ATTR(fw_version, S_IRUGO, show_fw_version_value, NULL);
static DEVICE_ATTR(fw_id, S_IRUGO, show_fw_id_value, NULL);
static DEVICE_ATTR(bc_version, S_IRUGO, show_bc_version_value, NULL);
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);
static DEVICE_ATTR(fw_update_in_driver, /*S_IRUGO | S_IWUGO*/0660, show_fw_update_in_driver, store_fw_update_in_driver);
static DEVICE_ATTR(fw_update, /*S_IRUGO | S_IWUGO*/0660, show_fw_update, store_fw_update);
//static DEVICE_ATTR(fw_update_elan, S_IRUGO | S_IWUGO, show_fw_update_elan, store_fw_update_elan);
//static DEVICE_ATTR(hid_fw_update, S_IRUGO, show_hid_fw_update, NULL);
#ifdef ELAN_2WIREICE
static DEVICE_ATTR(2wire, S_IRUGO, show_2wire, NULL);
#endif
static DEVICE_ATTR(iap_mode, S_IRUGO, show_iap_mode, NULL);

static DEVICE_ATTR(mt_protocol, S_IRUGO, show_mt_protocol, NULL);
static DEVICE_ATTR(power_mode, S_IRUGO, show_power_mode, NULL);
static DEVICE_ATTR(goto_idle_mode, S_IRUGO, show_goto_idle_mode, NULL);

#ifdef FEATURE_QUANTA_GESTURE_TO_BOOTLOADER
static DEVICE_ATTR(gestr_bld_ctl, 0660, show_gestr_bld_ctl, store_gestr_bld_ctl);
#endif

#ifdef FEATURE_QUANTA_GESTURE_TO_RECOVERY
static DEVICE_ATTR(gestr_rcvr_ctl, 0660, show_gestr_rcvr_ctl, store_gestr_rcvr_ctl);
#endif

#ifdef FEATURE_CHECK_TRUE_INTERRUPT
static DEVICE_ATTR(intCnt, S_IRUGO, show_int_count, NULL);
#endif


static struct attribute *elan_attributes[] = {
	//&dev_attr_debug_mesg.attr,
	&dev_attr_gpio_int.attr,
	&dev_attr_reset.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_id.attr,
	&dev_attr_bc_version.attr,
	&dev_attr_drv_version.attr,
	&dev_attr_fw_update_in_driver.attr,
	&dev_attr_fw_update.attr,
    //&dev_attr_fw_update_elan.attr,
    //&dev_attr_hid_fw_update.attr,
	#ifdef ELAN_2WIREICE
	&dev_attr_2wire.attr,
	#endif
	&dev_attr_iap_mode.attr,
	&dev_attr_mt_protocol.attr,
	&dev_attr_power_mode.attr,
	&dev_attr_goto_idle_mode.attr,
	#ifdef FEATURE_QUANTA_GESTURE_TO_BOOTLOADER
	&dev_attr_gestr_bld_ctl.attr,
	#endif
    #ifdef FEATURE_QUANTA_GESTURE_TO_RECOVERY
    &dev_attr_gestr_rcvr_ctl.attr,
    #endif
    #ifdef FEATURE_CHECK_TRUE_INTERRUPT
    &dev_attr_intCnt.attr,
    #endif
	NULL
};

static struct attribute_group elan_attribute_group = {
	.name = DEVICE_NAME,
	.attrs = elan_attributes,
};


// Start sysfs
static ssize_t elan_ktf_gpio_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	touch_debug(DEBUG_MESSAGES, "GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf_gpio_show, NULL);

static ssize_t elan_ktf_vendor_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "elan_ktf", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static int elan_ktf_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		touch_debug(DEBUG_ERROR,"[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}	

// end sysfs



static int elan_ktf_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t w_size,  size_t r_size)
{
	int rc;

	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
	return -EINVAL;

	if ((i2c_master_send(client, cmd, w_size)) != w_size) {
        dev_err(&client->dev, "[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf_ts_poll(client);
	if (rc < 0)
        printk("%s: poll is high\n",__func__);

	if(r_size <= 0) r_size=w_size;
	
	if (i2c_master_recv(client, buf, r_size) != r_size)	return -EINVAL;

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ktf_ts_poll(client);
	if (rc < 0) {
		printk( "[elan] %s: Int poll failed!\n", __func__);
	}

	rc = i2c_master_recv(client, buf_recv, 8);
	printk("[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

    /* Quanta BU10SW, Stanley Tsao, 2015.12.14, if hello packet is 0s ==> No response from device, return immediately { */    
	if(buf_recv[0]==0x00 && buf_recv[1]==0x00 && buf_recv[2]==0x00 && buf_recv[3]==0x00)
	{
        printk("[Stanley]%s: hello packet is 0 \n", __func__);
		return -ENODEV;
	}
    /* Quanta BU10SW, Stanley Tsao, 2015.12.14, if hello packet is 0s ==> No response from device, return immediately } */    


	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
		RECOVERY=0x80;
		return RECOVERY; 
	}
	

    /*this IC won't report 0x66 0x66 0x66 0x66, no need to receive*/
	/*
	mdelay(300);
	rc = elan_ktf_ts_poll(client);
	if (rc < 0) {
		printk( "[elan] %s: Int poll failed!\n", __func__);
	}
	rc = i2c_master_recv(client, buf_recv, 8);
	printk("[elan] %s: Try Re-Calibration packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	*/
	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
#if 0  /* for one-layer */
	uint8_t cmd_x[]   = { 0x53, 0x60, 0x00, 0x00 };     /*Get x resolution*/
	uint8_t cmd_y[]   = { 0x53, 0x63, 0x00, 0x00 };     /*Get y resolution*/
#endif 
	//uint8_t info_buff[] = { 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00 }; /*Get IC info*/
	//uint8_t info_buff_resp[17] = { 0 };
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};
	// Firmware version
	rc = elan_ktf_ts_get_data(client, cmd, buf_recv, 4,4);
	if (rc < 0)
	{
		printk("Get Firmware version error\n");
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 { */
        return rc;
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 } */
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;

	// Firmware ID
	rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv, 4,4);
	if (rc < 0)
	{
		printk("Get Firmware ID error\n");        
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 { */
        return rc;
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 } */
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	// Bootcode version
	rc = elan_ktf_ts_get_data(client, cmd_bc, buf_recv, 4,4);
	if (rc < 0)
	{
		printk("Get Bootcode version error\n");
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 { */
        return rc;
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 } */
        
	}
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;
    BC_VERSION = ts->bc_ver;

	/*Get XY info*/
#if 0  /* for one-layer */
/* X Resolution */
  rc = elan_ktf_ts_get_data( client, cmd_x, buf_recv, 4, 4);
  if( rc < 0 )
    return rc;
  minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
  ts->x_resolution =minor;
  X_RESOLUTION = minor;

/* Y Resolution */
  rc = elan_ktf_ts_get_data( client, cmd_y, buf_recv, 4, 4);
  if( rc < 0 )
    return rc;
  minor = (( buf_recv[2] )) | (( buf_recv[3] & 0xF0 ) << 4 );
  ts->y_resolution =minor;
  Y_RESOLUTION = minor;
#endif
#if 0
	rc = elan_ktf_ts_get_data(client, info_buff, info_buff_resp, sizeof(info_buff), sizeof(info_buff_resp));
	if (rc < 0)
	{
        printk("[elan] Get XY info error\n");
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 { */
        return rc;
        /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Return rc if rc < 0 } */        
	}
	ts->x_resolution = (info_buff_resp[2] + info_buff_resp[6]
	+ info_buff_resp[10] + info_buff_resp[14] - 1)*64;
//	X_RESOLUTION = ts->x_resolution;

	ts->y_resolution = (info_buff_resp[3] + info_buff_resp[7]
	+ info_buff_resp[11] + info_buff_resp[15] - 1)*64;
//	Y_RESOLUTION = ts->y_resolution;
#endif

    printk(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n", __func__, ts->fw_ver);
    printk(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n", __func__, ts->fw_id);
    printk(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n", __func__, ts->bc_ver);
    printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n", __func__, X_RESOLUTION, Y_RESOLUTION);
	
	return 0;
}

static inline int elan_ktf_pen_parse_xy(uint8_t *data,
uint16_t *x, uint16_t *y, uint16_t *p)
{
	*x = *y = *p = 0;

	*x = data[3];
	*x <<= 8;
	*x |= data[2];

	*y = data[5];
	*y <<= 8;
	*y |= data[4];

	*p = data[7];
	*p <<= 8;
	*p |= data[6];

	return 0;
}

static inline int elan_ktf_ts_parse_xy(uint8_t *data,
uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

    //printk("[Stanley]%s: x=%d y=%d \n", __func__, *x, *y); // Stanley Tsao debug
    
	return 0;
}

static int elan_ktf_ts_setup(struct i2c_client *client)
{
	int rc;
	
	rc = __hello_packet_handler(client);

    /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Hello packet is 0, return ENODEV { */
    if (rc == -ENODEV)
    {
        return rc;
    }
    /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Hello packet is 0, return ENODEV } */

	if (rc != 0x80){
		rc = __fw_packet_handler(client);
		if (rc < 0)
		printk("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
		dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n", __func__);
		//Check for FW_VERSION, if 0x0000 means FW update fail!
		if ( FW_VERSION == 0x00)
		{
			rc = 0x80;
			printk("[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION);
		}
	}
	return rc;
}

static int elan_ktf_ts_calibrate(struct i2c_client *client){

#ifdef ELAN_HID_I2C
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0xc0, 0xe1, 0x5a};
	uint8_t cal_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,CMD_W_PKT, 0x29, 0x00, 0x01};

	dev_info(&client->dev, "[elan] %s: Flash Key cmd\n", __func__);
	if ((i2c_master_send(client, flash_key, sizeof(flash_key))) != sizeof(flash_key)) {
        dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
    dev_info(&client->dev, "[elan] %s: Calibration cmd: %02x, %02x, %02x, %02x\n", __func__,
	cal_cmd[7], cal_cmd[8], cal_cmd[9], cal_cmd[10]);
	if ((i2c_master_send(client, cal_cmd, sizeof(cal_cmd))) != sizeof(cal_cmd)) {
        dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

#else	
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	printk("[elan] %s: enter\n", __func__);
    dev_info(&client->dev, "[elan] dump cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
        dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
#endif
	return 0;
}

#ifdef ELAN_POWER_SOURCE
static unsigned now_usb_cable_status=0;

#if 0
static int elan_ktf_ts_hw_reset(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	touch_debug(DEBUG_INFO, "[ELAN] Start HW reset!\n");
	gpio_direction_output(ts->rst_gpio, 0);
	usleep_range(1000,1500);
	gpio_direction_output(ts->rst_gpio, 1);
	msleep(5);
	return 0;
}

static int elan_ktf_ts_set_power_source(struct i2c_client *client, u8 state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};
	int length = 0;

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
	/*0x52 0x40 0x00 0x01  =>    Battery Mode
	0x52 0x41 0x00 0x01  =>   AC Adapter Mode
	0x52 0x42 0x00 0x01 =>    USB Mode */
	cmd[1] |= state & 0x0F;

	dev_dbg(&client->dev,
	"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	cmd[0], cmd[1], cmd[2], cmd[3]);
	
	down(&pSem);
	length = i2c_master_send(client, cmd, sizeof(cmd));
	up(&pSem);
	if (length != sizeof(cmd)) {
		dev_err(&client->dev,
		"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}



static void update_power_source(){
	unsigned power_source = now_usb_cable_status;
	if(private_ts == NULL || work_lock) return;

	if(private_ts->abs_x_max == ELAN_X_MAX) //TF 700T device
	return; // do nothing for TF700T;
	
	touch_debug(DEBUG_INFO, "Update power source to %d\n", power_source);
	switch(power_source){
	case USB_NO_Cable:
		elan_ktf_ts_set_power_source(private_ts->client, 0);
		break;
	case USB_Cable:
		elan_ktf_ts_set_power_source(private_ts->client, 1);
		break;
	case USB_AC_Adapter:
		elan_ktf_ts_set_power_source(private_ts->client, 2);
	}
}
#endif

void touch_callback(unsigned cable_status){ 
	now_usb_cable_status = cable_status;
	//update_power_source();
}
#endif

static int elan_ktf_ts_recv_data(struct i2c_client *client, uint8_t *buf, int bytes_to_recv)
{

	int rc;
	if (buf == NULL)
	return -EINVAL;

	memset(buf, 0, bytes_to_recv);

	/* The ELAN_PROTOCOL support normanl packet format */	
#ifdef ELAN_PROTOCOL		
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	//printk("[elan] Elan protocol rc = %d \n", rc);
	if (rc != bytes_to_recv) {
		dev_err(&client->dev, "[elan] %s: i2c_master_recv error?! \n", __func__);
		return -1;
	}

#else 
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	if (rc != 8)
	{
		printk("[elan] Read the first package error.\n");
		mdelay(30);
		return -1;
	}
	printk("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	mdelay(1);
	
	if (buf[0] == 0x6D){    //for five finger
		rc = i2c_master_recv(client, buf+ 8, 8);	
		if (rc != 8)
		{
			printk("[elan] Read the second package error.\n");
			mdelay(30);
			return -1;
		}		      
		printk("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
		rc = i2c_master_recv(client, buf+ 16, 2);
		if (rc != 2)
		{
			printk("[elan] Read the third package error.\n");
			mdelay(30);
			return -1;
		}		      
		mdelay(1);
		printk("[elan_debug] %x %x \n", buf[16], buf[17]);
	}
#endif
	//printk("[elan_debug] end ts_work\n");
	return rc;
}

#ifdef PROTOCOL_B
/* Protocol B  */
static int mTouchStatus[FINGER_NUM] = {0};  /* finger_num=10 */
void force_release_pos(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int i;
	for (i=0; i < FINGER_NUM; i++) {
		if (mTouchStatus[i] == 0) continue;
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		mTouchStatus[i] = 0;
	}

	input_sync(ts->input_dev);
}

static inline int elan_ktf_hid_parse_xy(uint8_t *data,
uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[6]);
	*x <<= 8;
	*x |= data[5];

	*y = (data[10]);
	*y <<= 8;
	*y |= data[9];

	return 0;
}

static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x =0, y =0,touch_size, pressure_size;
	uint16_t fbits=0;
	uint8_t i, num;
	uint16_t active = 0; 
	uint8_t idx, btn_idx;
	int finger_num;
	int finger_id;
	static uint8_t size_index[10] = {35, 35, 36, 36, 37, 37, 38, 38, 39, 39};
	int pen_hover = 0;
	int pen_down = 0;
	uint16_t p = 0;

    /* Quanta BU10SW, Stanley Tsao, 2015.12.09, For customized gesture events { */
    #ifdef FEATURE_QUANTA_GESTURE_CUSTOMIZATION
    //uint8_t LKC_F1 = 0x3b;   // Linux Key Code F1
    uint8_t TEV_MIN = 0x12;  // Touch Event Value Minimum
    //uint8_t code_offset = LKC_F1 - TEV_MIN; // (Linux F1 key code 0x3b) - (minimum touch event value)
    uint8_t LKC = 0;
    #endif
    /* Quanta BU10SW, Stanley Tsao, 2015.12.09, For customized gesture events } */

	/* for 10 fingers */
	if (buf[0] == TEN_FINGERS_PKT){
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx=3;
		btn_idx=33;
	}
	/* for 5 fingers  */
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >>3;
		idx=2;
		btn_idx=17;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.09, For customized gesture events { */
    #ifdef FEATURE_QUANTA_GESTURE_CUSTOMIZATION
    }else if (buf[0] == QUANTA_GESTURE_PKT){
        //finger_num = 1;
    #endif
    /* Quanta BU10SW, Stanley Tsao, 2015.12.09, For customized gesture events } */
	}else{
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;    // for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		//        fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06
		idx=1;
		btn_idx=7;
	}

	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:

		for(i = 0; i < finger_num; i++){
			active = fbits & 0x1;
			if(active || mTouchStatus[i]){
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
				if(active){
					elan_ktf_ts_parse_xy(&buf[idx], &x, &y);
					//y=Y_RESOLUTION -y;
					touch_size = ((i & 0x01) ? buf[size_index[i]] : (buf[size_index[i]] >> 4)) & 0x0F;
					pressure_size = touch_size << 4; // shift left touch size value to 4 bits for max pressure value 255   	      	
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
					input_report_abs(idev, ABS_MT_PRESSURE, 100);
                    /* Quanta BU10SW, Stanley Tsao, 2015.12.16, Rotate clockwise 90 degree { */
                    #if 1 /* Touch coordinates match with display coordinates */
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
                    #else /* Rotate clockwise 90 degree */
					input_report_abs(idev, ABS_MT_POSITION_X, y);
					input_report_abs(idev, ABS_MT_POSITION_Y, X_RESOLUTION-x);
                    #endif
                    /* Quanta BU10SW, Stanley Tsao, 2015.12.16, Rotate clockwise 90 degree } */
					input_report_key(idev, BTN_TOUCH, 1);
					if(unlikely(gPrint_point)) 
					touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d size=%d press=%d \n", i, x, y, touch_size, pressure_size);
				}
			}
			mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		if (num == 0){
			//printk("[ELAN] ALL Finger Up\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case PEN_PKT:
		
		pen_hover = buf[1] & 0x1;
		pen_down = buf[1] & 0x03;
		input_mt_slot(ts->input_dev, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, pen_hover);
		if (pen_hover){
			elan_ktf_pen_parse_xy(&buf[0], &x, &y, &p);
			//y=Y_RESOLUTION -y;
			if (pen_down == 0x01) {  /* report hover function  */
				input_report_abs(idev, ABS_MT_PRESSURE, 0);
				input_report_abs(idev, ABS_MT_DISTANCE, 15);
				touch_debug(DEBUG_INFO, "[elan pen] Hover DISTANCE=15 \n");
			}
			else {
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 20);
				input_report_abs(idev, ABS_MT_PRESSURE, p);
				touch_debug(DEBUG_INFO, "[elan pen] PEN PRESSURE=%d \n", p);
			}
			input_report_abs(idev, ABS_MT_POSITION_X, x);
			input_report_abs(idev, ABS_MT_POSITION_Y, y);
		}
		if(unlikely(gPrint_point)) {
			touch_debug(DEBUG_INFO, "[elan pen] %x %x %x %x %x %x %x %x \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
			touch_debug(DEBUG_INFO, "[elan] x=%d y=%d p=%d \n", x, y, p);
		}
		if (pen_down == 0){
			//printk("[ELAN] ALL Finger Up\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			force_release_pos(client);
		}
		input_sync(idev);
		break;
	case ELAN_HID_PKT:
		finger_num = buf[62];
		if (finger_num > 5)	finger_num = 5;   /* support 5 fingers    */ 
		idx=3;
		num = 5;
		for(i = 0; i < finger_num; i++){						
			if ((buf[idx]&0x03) == 0x00)	active = 0;   /* 0x03: finger down, 0x00 finger up  */
			else	active = 1;

			if ((buf[idx] & 0x03) == 0) num --;
			finger_id = (buf[idx] & 0xfc) >> 2;
			input_mt_slot(ts->input_dev, finger_id);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
			if(active){
				elan_ktf_hid_parse_xy(&buf[idx], &x, &y);
				//y = Y_RESOLUTION - y;
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(idev, ABS_MT_PRESSURE, 100);
				input_report_abs(idev, ABS_MT_POSITION_X, x);
				input_report_abs(idev, ABS_MT_POSITION_Y, y);
				touch_debug(DEBUG_INFO, "[elan hid] i=%d finger_id=%d x=%d y=%d Finger NO.=%d \n", i, finger_id, x, y, finger_num);
			}
			mTouchStatus[i] = active;
			idx += 11;
		}
		if (num == 0){  
            printk("[elan] Release ALL Finger\n");
			input_report_key(idev, BTN_TOUCH, 0); //for all finger up
			force_release_pos(client);
		}
		input_sync(idev);
		break ;
    case IamAlive_PKT:
	    ts->power_mode = POWER_MODE_NORMAL;
		/* touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] ); */
		break;
    case IDLE_MODE_PKT:
        ts->power_mode = POWER_MODE_IDLE;
		#ifdef FEATURE_CHECK_TRUE_INTERRUPT
		touchINT_cnt = 0;
		#endif
		break;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.09, For customized gesture events { */
    #ifdef FEATURE_QUANTA_GESTURE_CUSTOMIZATION
    case QUANTA_GESTURE_PKT:
        /* Map touch-reported event value to Linux F1-F8 key codes */
        LKC = gesture_map_table[buf[1] - TEV_MIN];

        /* Report Linux F1-F8 key codes */
        //input_event(idev, EV_MSC, MSC_SCAN, LKC);
        input_report_key(idev, LKC, 1);
        input_sync(idev);
        input_report_key(idev, LKC, 0);
        input_sync(idev);
        
        printk("[Stanley]%s: QUANTA_GESTURE_PKT, Receive packet type: %x, event value=%x, LKC=%x \n",
                __func__, buf[0], buf[1], LKC);
        break;
    #endif
    /* Quanta BU10SW, Stanley Tsao, 2015.12.09, For customized gesture events } */
    #ifdef FEATURE_PALM_DETECTION
    case PALM_DETECTION_PKT:
        input_report_key(idev, KEY_SLEEP, 1);
        input_sync(idev);
        input_report_key(idev, KEY_SLEEP, 0);
        input_sync(idev);
        break;
    #endif
	default:
		dev_err(&client->dev,
		"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
		break;
	} // end switch

	return;
}

#endif

#ifdef PROTOCOL_A
/* Protocol A  */
static void elan_ktf_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0;
	uint8_t i, num, reported = 0;
	uint8_t idx, btn_idx;
	int finger_num;
	
	/* for 10 fingers	*/
	if (buf[0] == TEN_FINGERS_PKT){
		finger_num = 10;
		num = buf[2] & 0x0f; 
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
		idx=3;
		btn_idx=33;
	}
	/* for 5 fingers	*/
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
		finger_num = 5;
		num = buf[1] & 0x07; 
		fbits = buf[1] >>3;
		idx=2;
		btn_idx=17;
	}else{
		/* for 2 fingers */    
		finger_num = 2;
		num = buf[7] & 0x03;		// for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		//        fbits = (buf[7] & 0x03) >> 1;	// for elan old 5A protocol the finger ID is 0x06
		idx=1;
		btn_idx=7;
	}
	
	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:	
	case TEN_FINGERS_PKT:
		//input_report_key(idev, BTN_TOUCH, 1);
		if (num == 0) {
			input_report_key(idev, BTN_TOUCH, 0);
            #ifdef FEATURE_QUANTA_GESTURE_TO_BOOTLOADER
			if(is_quanta_gesture_to_bld_enabled == true)
			{
				quanta_gesture_clear_bootloader_flag();
			}
            #endif /* FEATURE_QUANTA_GESTURE_TO_BOOTLOADER */
            #ifdef FEATURE_QUANTA_GESTURE_TO_RECOVERY
            if(is_quanta_gesture_to_rcvr_enabled == true)
            {
                quanta_gesture_clear_recovery_flag();
            }
            #endif
#ifdef ELAN_BUTTON
			if (buf[btn_idx] == 0x21) 
			{
				button_state = 0x21;
				input_report_key(idev, KEY_BACK, 1);
				input_report_key(idev, KEY_BACK, 0);
				printk("[elan_debug] button %x \n", buf[btn_idx]);
			} 
			else if (buf[btn_idx] == 0x41)
			{
				button_state = 0x41;
				input_report_key(idev, KEY_HOME, 1);
			} 
			else if (buf[btn_idx] == 0x81)
			{
				button_state = 0x81;
				input_report_key(idev, KEY_MENU, 1);
			} 
			else if (button_state == 0x21) 
			{
				button_state=0;
				input_report_key(idev, KEY_BACK, 0);
			} 
			else if (button_state == 0x41) 
			{
				button_state=0;
				input_report_key(idev, KEY_HOME, 0);
			} 
			else if (button_state == 0x81) 
			{
				button_state=0;
				input_report_key(idev, KEY_MENU, 0);
			}
			else
			{
				dev_dbg(&client->dev, "no press\n");
				input_mt_sync(idev);

			}
			
#endif	
		} else {			
			dev_dbg(&client->dev, "[elan] %d fingers\n", num);      
			input_report_key(idev, BTN_TOUCH, 1);			
			for (i = 0; i < finger_num; i++) {	
				if ((fbits & 0x01)) {
					elan_ktf_ts_parse_xy(&buf[idx], &x, &y);  		 
					#ifdef FEATURE_QUANTA_GESTURE_TO_BOOTLOADER
					if(is_quanta_gesture_to_bld_enabled == true)
					{
						quanta_gesture_to_bootloader_check(x, y);
					}
					#endif
					#ifdef FEATURE_QUANTA_GESTURE_TO_RECOVERY
					if(is_quanta_gesture_to_rcvr_enabled == true)
					{
						quanta_gesture_to_recovery_check(x, y);
					}
					#endif
					//x = X_RESOLUTION-x;	 
					//y = Y_RESOLUTION-y;			     
					//printk("[elan_debug] %s, x=%x, y=%x\n",__func__, x , y);
					input_report_abs(idev, ABS_MT_TRACKING_ID, i);
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
					input_report_abs(idev, ABS_MT_PRESSURE, 80);
					input_report_abs(idev, ABS_MT_POSITION_X, x);
					input_report_abs(idev, ABS_MT_POSITION_Y, y);
					input_mt_sync(idev);
					reported++;
					if(unlikely(gPrint_point)) 
					touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d \n", i, x, y);
				} // end if finger status
				fbits = fbits >> 1;
				idx += 3;

			} // end for
		}
		if (reported)
		input_sync(idev);
		else {
			input_mt_sync(idev);
			input_sync(idev);
		}

		break;
    case IamAlive_PKT://0512
        ts->power_mode = POWER_MODE_NORMAL;
        /* touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] ); */
        break;
	case HELLO_PKT:
        //touch_debug(DEBUG_ERROR,"[elan] Report Hello packet: %x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
        printk("[elan] Report Hello packet: %x %x %x %x\n",buf[0],buf[1],buf[2],buf[3]);
        break;
    case CALIB_PKT:
        //touch_debug(DEBUG_ERROR,"[elan] Report Calibration packet: %x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
        printk("[elan] Report Calibration packet: %x %x %x %x\n",buf[0],buf[1],buf[2],buf[3]);
        break;
    case IDLE_MODE_PKT:
        ts->power_mode = POWER_MODE_IDLE;
		#ifdef FEATURE_CHECK_TRUE_INTERRUPT
		touchINT_cnt = 0;
		#endif
        break;
    #ifdef FEATURE_PALM_DETECTION
    case PALM_DETECTION_PKT:
        input_report_key(idev, KEY_SLEEP, 1);
        input_sync(idev);
        if(is_enalbe_vib) {
                printk(KERN_INFO "vibrator_enable\n");
                is_enalbe_vib = false;
                vibrator_enable(50);
        }
        input_report_key(idev, KEY_SLEEP, 0);
        input_sync(idev);
        break;
    #endif
	default:
        dev_err(&client->dev, "[elan] %s: unknown packet type: %0x %0x %0x %0x\n", __func__, buf[0], buf[1], buf[2], buf[3]);
		break;
	} // end switch

	return;
}
#endif

static irqreturn_t elan_ktf_ts_irq_handler(int irq, void *dev_id)
{
	int rc;
	struct elan_ktf_ts_data *ts = dev_id;
#ifdef ELAN_BUFFER_MODE
	uint8_t buf[4+PACKET_SIZE] = { 0 };
	uint8_t buf1[PACKET_SIZE] = { 0 };
#else
	uint8_t buf[PACKET_SIZE] = { 0 };
#endif

#if defined( ESD_CHECK )
	have_interrupts = 1;
#endif
	if (gpio_get_value(ts->intr_gpio))
	{
		printk("[elan] Detected the jitter on INT pin");
		return IRQ_HANDLED;
	}
	
#ifdef ELAN_BUFFER_MODE
	rc = elan_ktf_ts_recv_data(ts->client, buf,4+PACKET_SIZE);
	if (rc < 0)
	{
		printk("[elan] Received the packet Error.\n");
		return IRQ_HANDLED;
	}
#else
	rc = elan_ktf_ts_recv_data(ts->client, buf, PACKET_SIZE);
	if (rc < 0)
	{
		printk("[elan] Received the packet Error.\n");
		return IRQ_HANDLED;
	}
    #ifdef FEATURE_CHECK_TRUE_INTERRUPT
    if(buf[0] == IamAlive_PKT)
    {
        IamAlive_PKT_INT_cnt +=1;
    }
	else
	{
		touchINT_cnt +=1;
	}
    #endif
#endif
	//printk("[elan_debug] %2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x ....., %2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[17]);

#ifndef ELAN_BUFFER_MODE
	elan_ktf_ts_report_data(ts->client, buf);
#else
    if((buf[0] == HELLO_PKT) || ((buf[0] == CALIB_PKT) && (buf[1] == CALIB_PKT)))
	{
		elan_ktf_ts_report_data(ts->client, buf);
	}
	else
	elan_ktf_ts_report_data(ts->client, buf+4);

	// Second package
	if (((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3))) {
		rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
		if (rc < 0){
			return IRQ_HANDLED;
		}
		elan_ktf_ts_report_data(ts->client, buf1);
		// Final package
		if (buf[1] == 3) {
			rc = elan_ktf_ts_recv_data(ts->client, buf1, PACKET_SIZE);
			if (rc < 0){
				return IRQ_HANDLED;
			}
			elan_ktf_ts_report_data(ts->client, buf1);
		}
	}
#endif

	return IRQ_HANDLED;
}

static int elan_ktf_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_threaded_irq(ts->irq, NULL, elan_ktf_ts_irq_handler,
	IRQF_TRIGGER_LOW /*| IRQF_TRIGGER_FALLING*/ | IRQF_ONESHOT,
	client->name, ts);
	if (err) {
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
		__func__, client->irq);
	}

	/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */
    #ifdef QUANTA_DEFER_ENABLE_IRQ
	printk("[ELAN] disable IRQ %d", ts->client->irq);
	if (private_ts != NULL)
		printk("[ELAN] disable IRQ %d", private_ts->client->irq);
	work_lock=1;
	disable_irq(ts->client->irq);
    #endif /* QUANTA_DEFER_ENABLE_IRQ */
	/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */
	
	return err;
}

#ifdef _ENABLE_DBG_LEVEL
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check { */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static int ektf_proc_read(struct file *file, char __user *userbuf, size_t bytes, loff_t *off)
{
    int ret;
    ret=0;
    return ret;
}

static int ektf_proc_write(struct file *file, const char __user *userbuf, size_t bytes, loff_t *off)
{

}

/* Quanta BU10SW, Stanley Tsao, 2015.11.18, Add for kernel versio above 3.10 { */
struct file_operations elan_touch_proc_fops = {
    .owner = THIS_MODULE,
    .read = ektf_proc_read,
    .write = ektf_proc_write
};
/* Quanta BU10SW, Stanley Tsao, 2015.11.18, Add for kernel versio above 3.10 } */

#else
static int ektf_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data )
{
	int ret;

	touch_debug(DEBUG_MESSAGES, "call proc_read\n");

	if(offset > 0)  /* we have finished to read, return 0 */
	ret  = 0;
	else
	ret = sprintf(buffer, "Debug Level: Release Date: %s\n","2011/10/05");

	return ret;
}

static int ektf_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0;
	int i, ret = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};
	unsigned int command;

	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN )
	procfs_buffer_size = PROC_FS_MAX_LEN+1;

	if( copy_from_user(procfs_buf, buffer, procfs_buffer_size) )
	{
		touch_debug(DEBUG_ERROR, " proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	command = 0;
	for(i=0; i<procfs_buffer_size-1; i++)
	{
		if( procfs_buf[i]>='0' && procfs_buf[i]<='9' )
		command |= (procfs_buf[i]-'0');
		else if( procfs_buf[i]>='A' && procfs_buf[i]<='F' )
		command |= (procfs_buf[i]-'A'+10);
		else if( procfs_buf[i]>='a' && procfs_buf[i]<='f' )
		command |= (procfs_buf[i]-'a'+10);

		if(i!=procfs_buffer_size-2)
		command <<= 4;
	}

	command = command&0xFFFFFFFF;
	switch(command){
	case 0xF1:
		gPrint_point = 1;
		break;
	case 0xF2:
		gPrint_point = 0;
		break;
	case 0xFF:
		ret = elan_ktf_ts_calibrate(private_ts->client);
		break;
	}
	touch_debug(DEBUG_INFO, "Run command: 0x%08X  result:%d\n", command, ret);

	return count; // procfs_buffer_size;
}

#endif
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check } */


#endif // #ifdef _ENABLE_DBG_LEV

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	pm_message_t pmMsg;
	struct elan_ktf_ts_data *elan_dev_data =
	container_of(self, struct elan_ktf_ts_data, fb_notif);
	is_enalbe_vib = false;

	printk("%s fb notifier callback event = %ld\n", __func__, event);
	if (evdata && evdata->data && elan_dev_data && private_ts->client) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;

			if (*blank == FB_BLANK_UNBLANK)
			{
				printk("resume\n");
				printk(KERN_INFO "set is_enable_vib to true\n");
				is_enalbe_vib = true;
				elan_ktf_ts_resume(private_ts->client);
			}
			else if (*blank == FB_BLANK_POWERDOWN)
			{
				printk("suspend\n");
				pmMsg.event = PM_EVENT_SUSPEND;
				elan_ktf_ts_suspend(private_ts->client, pmMsg);
			}
		}
	}

	return 0;
}
#endif

#if defined( ESD_CHECK ) 
static void elan_touch_esd_func(struct work_struct *work)
{

  //touch_debug(DEBUG_INFO, "[elan esd] %s: enter.......\n", __FUNCTION__);  /* elan_dlx */

  if( (have_interrupts == 1) || (work_lock == 1) || (suspend_ESD == 1))
  {
    /* touch_debug(DEBUG_INFO, "[elan esd] : had interrupt not need check\n"); */
  }
  else
  {
	/* Reset TP, if touch controller no any response  */
	touch_debug(DEBUG_INFO, "[elan esd] : touch controller no any response, reset TP\n");
	elan_ktf_ts_hw_reset();
  }

  have_interrupts = 0;
  queue_delayed_work( esd_wq, &esd_work, delay );
 /* touch_debug(DEBUG_INFO, "[elan esd] %s: exit.......\n", __FUNCTION__ );*/  /* elan_dlx */
}
#endif


/* Quanta BU10SW, Stanley Tsaos, 2015.12.03, Add VDD, VDDIO control function { */
static int elan_ktf_power_init(struct elan_ktf_ts_data *data, bool on)
{
	int rc;

    printk("[Stanley]%s \n", __func__);


	if (!on)
	{
		dev_err(&data->client->dev, "elan_ktf_power_init false \n");
		goto pwr_deinit;
	}

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev, "Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, ELAN_KTF_VTG_MIN_UV, ELAN_KTF_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev, "Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, ELAN_KTF_I2C_VTG_MIN_UV, ELAN_KTF_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;
    
reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, ELAN_KTF_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, ELAN_KTF_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, ELAN_KTF_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int elan_ktf_power_on(struct elan_ktf_ts_data *data, bool on)
{
	int rc = 0;

    printk("[Stanley]%s: on=%d \n", __func__, on);

	if (!on)
	{
		printk("[Stanley] Enter %s false ! \n", __func__);
		goto power_off;
	}

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;


power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}
/* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function } */



static int elan_ktf_ts_probe(struct i2c_client *client,
const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	struct elan_ktf_i2c_platform_data *pdata;
	struct elan_ktf_ts_data *ts;
	int New_FW_ID;	
	int New_FW_VER;	
	//struct task_struct *fw_update_thread;

    printk("[Stanley]%s called \n", __func__);

    /* Quanta BU10SW, Stanley Tsao, 2015.12.01, parse device tree { */
    if (client->dev.of_node) {
        printk("[Stanley]%s: client->dev.of_node exists \n", __func__);
        pdata = devm_kzalloc(&client->dev, sizeof(struct elan_ktf_i2c_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }
        err = elan_ktf_parse_dt(&client->dev, pdata);
        if (err) {
            dev_err(&client->dev, "DT parsing failed\n");
            //return err;
        }
        printk("[Stanley]%s: parse elan touch dt done \n", __func__);
    }
    else
        pdata = client->dev.platform_data;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.01, parse device tree } */    

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "[elan] %s: allocate elan_ktf_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	/* Quanta BU10SW, Stanley Tsao, 2015.12.25, Assign pointer right after memory allocation for ts { */
	private_ts = ts;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.25, Assign pointer right after memory allocation for ts } */

    printk("[Stanley]%s current MT protocol=%s \n", __func__, CURRENT_MT_PROTOCOL); // Stanley dbg

	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ktf_mutex);
	// james: maybe remove	
    /*Quanta BU10SW, Stanley Tsao, 2015.12.03, Remove old code { */
    #if 0
    pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		ts->intr_gpio = pdata->intr_gpio;
        printk("[Stanley]%s: set intr_gpio elan_ktf_ts_data \n", __func__);
	}
    #endif
    ts->intr_gpio = pdata->intr_gpio;
	ts->irq = gpio_to_irq(pdata->intr_gpio);
    /*Quanta BU10SW, Stanley Tsao, 2015.12.03, Remove old code { */

    /* Quanta BU10SW, Stalney Tsao, 2015.12.25, set reset pin from device tree { */
    ts->rst_gpio = pdata->rst_gpio;
    /* Quanta BU10SW, Stalney Tsao, 2015.12.25, set reset pin from device tree } */

    
    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function { */
    err = elan_ktf_power_init(ts, true);
    if(err) {
        dev_err(&client->dev, "elan_ktf_power_init power init failed");
        goto unreg_inputdev;
    }

    err = elan_ktf_power_on(ts, true);
    if (err) {
        dev_err(&client->dev, "power on failed");
        goto pwr_deinit;
    }
    
    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function } */

    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
    #ifdef MSM_NEW_VER
	err = elan_ktf_ts_pinctrl_init(ts);
	if (!err && ts->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(ts->ts_pinctrl,
					ts->pinctrl_state_active);
		if (err < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
		}
	}
    #endif
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 } */

    
	elan_ktf_ts_hw_reset();
	err = gpio_request(private_ts->intr_gpio, "elan_ktf_irq_gpio");
	if (err)
		dev_err(&client->dev,
				"Request elan irq gpio failed\n");
	gpio_direction_input(private_ts->intr_gpio);

	fw_err = elan_ktf_ts_setup(client);
    /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Hello packet is 0, return ENODEV { */
    if(fw_err == -ENODEV)
    {
        printk("[Stanley]%s: elan_ktf_ts_setup() failed, error=%d \n", __func__, fw_err);
        /* Power off VDD, VDDIO */
        elan_ktf_power_on(ts, false);
        printk("[Stanley]%s: power off vdd, vddio \n", __func__);
        elan_ktf_power_init(ts,false);
        printk("[Stanley]%s: Deinit power \n", __func__);
        kfree(ts);
        
        return fw_err;
    }
    
    /* Quanta BU10SW, Stanley Tsao, 2015.12.14, Hello packet is 0, return ENODEV } */

    if (fw_err < 0) {
		printk(KERN_INFO "No Elan chip inside\n");
		//		fw_err = -ENODEV;  
	}

	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "elan-touchscreen");
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";     

#ifdef PROTOCOL_A
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
#endif
#ifdef ELAN_BUTTON
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif

#ifdef PROTOCOL_B
    /* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check { */
    #if (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(ts->input_dev, FINGER_NUM);
    #else
    input_mt_init_slots(ts->input_dev, FINGER_NUM, 0);
    #endif
    /* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check } */
#endif

#ifdef PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_X, 0,  X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0,  Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);	
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_DISTANCE, 0, MAX_FINGER_SIZE, 0, 0);

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);

    /* Quanta BU10SW, Stanley Tsao, 2015.11.23, Add Quanta customized gesture events {*/
    #ifdef FEATURE_QUANTA_GESTURE_CUSTOMIZATION
    __set_bit(EV_MSC, ts->input_dev->evbit);
    __set_bit(KEY_F1,ts->input_dev->keybit);
    __set_bit(KEY_F2,ts->input_dev->keybit);
    __set_bit(KEY_F3,ts->input_dev->keybit);
    __set_bit(KEY_F4,ts->input_dev->keybit);
    __set_bit(KEY_F5,ts->input_dev->keybit);
    __set_bit(KEY_F6,ts->input_dev->keybit);
    __set_bit(KEY_F7,ts->input_dev->keybit);
    __set_bit(KEY_F8,ts->input_dev->keybit);
    printk("[Stanley]%s set KEY_F1~F8 keybit \n", __func__);
	input_set_capability(ts->input_dev, EV_MSC, MSC_SCAN);
    #endif
    /* Quanta BU10SW, Stanley Tsao, 2015.11.23, Add Quanta customized gesture events }*/
    
    #ifdef FEATURE_PALM_DETECTION
    __set_bit(KEY_SLEEP, ts->input_dev->keybit);
    #endif
    
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
		"[elan]%s: unable to register %s input device\n",
		__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function { */
    #if 0
    err = elan_ktf_power_init(ts, true);
    if(err) {
        dev_err(&client->dev, "elan_ktf_power_init power init failed");
        goto unreg_inputdev;
    }

    err = elan_ktf_power_on(ts, true);
    if (err) {
        dev_err(&client->dev, "power on failed");
        goto pwr_deinit;
    }
    #endif
    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function } */

	elan_ktf_ts_register_interrupt(ts->client);

    /* Quanta BU10SW, Stanley Tsao, 2015.12.25, Assign pointer right after memory allocation for ts { */
	//private_ts = ts;
    /* Quanta BU10SW, Stanley Tsao, 2015.12.25, Assign pointer right after memory allocation for ts } */

	elan_ktf_touch_sysfs_init();

	dev_info(&client->dev, "[elan] Start touchscreen %s in interrupt mode\n",
	ts->input_dev->name);

	// Firmware Update
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO; 

	if (misc_register(&ts->firmware) < 0)
        printk("[elan]misc_register failed!!\n");
	else
        printk("[elan]misc_register finished!!\n");
	// End Firmware Update	

	/* register sysfs */
	if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
	dev_err(&client->dev, "sysfs create group error\n");

#ifdef IAP_PORTION
	if(0)
	{
        printk("[elan]misc_register finished!!\n");
		work_lock=1;
		disable_irq(ts->client->irq);
		//cancel_work_sync(&ts->work);
		
		power_lock = 1;
		/* FW ID & FW VER*/
		/* for ektf31xx iap ekt file   */	
        printk("[elan] [7bd8]=0x%02x, [7bd9]=0x%02x, [7bda]=0x%02x, [7bdb]=0x%02x\n", file_fw_data[31704],file_fw_data[31705],file_fw_data[31706],file_fw_data[31707]);
		New_FW_ID = file_fw_data[31707]<<8  | file_fw_data[31708] ;	       
		New_FW_VER = file_fw_data[31705]<<8  | file_fw_data[31704] ;
        printk("[elan] FW_ID=0x%x, New_FW_ID=0x%x\n", FW_ID, New_FW_ID);
        printk("[elan] FW_VERSION=0x%x, New_FW_VER=0x%x\n", FW_VERSION  , New_FW_VER);

		/* for firmware auto-upgrade            
	if (New_FW_ID   ==  FW_ID){		      
			if (New_FW_VER > (FW_VERSION)) 
	{
		Update_FW_One(0);
		//Update_FW_in_Driver(client, RECOVERY);
	}
	else
		printk("No need to update fw.\n");
		} else {                        
						printk("FW_ID is different!");		
		}
*/         	
        /*
        if (FW_ID == 0)  RECOVERY=0x80;
        Update_FW_One(0);
        //Update_FW_in_Driver(client, RECOVERY);
        */
		power_lock = 0;

		work_lock=0;
		enable_irq(ts->client->irq);

	}
#endif	

	#if 0
	read_test();
    fw_update_thread = kthread_run(Update_FW_One, NULL, "elan_update");
	if(IS_ERR(fw_update_thread))
	{
		printk("[elan]  failed to create kernel thread\n");
	}
	#endif


#ifdef _ENABLE_DBG_LEVEL
    /* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check { */
    #if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
    dbgProcFile = proc_create(PROC_FS_NAME, 0600, NULL, &elan_touch_proc_fops);
    if (dbgProcFile == NULL)
    {
        touch_debug(DEBUG_INFO, " Could not initialize /proc/%s\n", PROC_FS_NAME);
    }
    #else
    dbgProcFile = create_proc_entry(PROC_FS_NAME, 0600, NULL);
    
	if (dbgProcFile == NULL)
	{
		remove_proc_entry(PROC_FS_NAME, NULL);
		touch_debug(DEBUG_INFO, " Could not initialize /proc/%s\n", PROC_FS_NAME);
	}
	else
	{
		dbgProcFile->read_proc = ektf_proc_read;
		dbgProcFile->write_proc = ektf_proc_write;
		touch_debug(DEBUG_INFO, " /proc/%s created\n", PROC_FS_NAME);
	}
    #endif
    /* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check } */


    
#endif // #ifdef _ENABLE_DBG_LEVEL
#if defined(CONFIG_FB)
	private_ts->fb_notif.notifier_call = fb_notifier_callback;

	fb_register_client(&private_ts->fb_notif);

#endif

#if defined( ESD_CHECK ) //0604
  INIT_DELAYED_WORK( &esd_work, elan_touch_esd_func );
  esd_wq = create_singlethread_workqueue( "esd_wq" );
  if( !esd_wq )
  {
	printk("[elan]  failed to create esd thread\n");
  }

  queue_delayed_work( esd_wq, &esd_work, delay );
#endif

    /* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */
	printk("[Stanley]%s: INIT_DEFERRABLE_WORK \n", __func__);
	printk("[Stanley]%s: INIT_DEFERRABLE_WORK\n", __func__);
	INIT_DEFERRABLE_WORK(&ts->workd, elan_work);
	schedule_delayed_work(&ts->workd, msecs_to_jiffies(DEFER_ENABLE_IRQ_TIMER_MS));
    /* Quanta BU10SW, Stanley Tsao, 2015.12.25, Defer enabling touch irq { */


	return 0;

	err_input_register_device_failed:
	if (ts->input_dev)
	input_free_device(ts->input_dev);

	err_input_dev_alloc_failed: 
	kfree(ts);
#ifdef _ENABLE_DBG_LEVEL
	remove_proc_entry(PROC_FS_NAME, NULL);
#endif

	err_alloc_data_failed:
	err_check_functionality_failed:

    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function { */ 
    unreg_inputdev:
        input_unregister_device(ts->input_dev);
        ts->input_dev = NULL;
        
    pwr_deinit:
        elan_ktf_power_init(ts, false);
    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function } */
    
	return err;
}

static int elan_ktf_ts_remove(struct i2c_client *client)
{
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	//unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	gpio_free(private_ts->intr_gpio);

    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function { */
    elan_ktf_power_on(ts, false);
    elan_ktf_power_init(ts, false);
    /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add VDD, VDDIO control function } */
    
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	if (state == PWR_STATE_DEEP_SLEEP || state == PWR_STATE_NORMAL)
	{
		cmd[1] |= (state << 3);
	}else if (state == PWR_STATE_IDLE) {
		cmd[1] = 0x52;
	}else {
		printk("[elan]%s: unknown state:%d \n", __func__, state);
		return -EINVAL;;
	}

	dev_dbg(&client->dev,
	"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
	cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
		"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	disable_irq(private_ts->client->irq);   //prevent from receving by report function 20160420
	rc = elan_ktf_ts_get_data(client, cmd, buf, 4, 4);
	enable_irq(private_ts->client->irq);
	if (rc)
	return rc;


    power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
    dev_err(&client->dev, "[elan] power state = %s\n", power_state == PWR_STATE_DEEP_SLEEP ? "Deep Sleep" : "Normal/Idle");

	return power_state;
}

static int elan_ktf_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
    #ifdef MSM_NEW_VER
    struct elan_ktf_ts_data *data = i2c_get_clientdata(client);
    int err;
    #endif
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 } */

    
	int rc = 0;

    #ifdef FEATURE_CHECK_TRUE_INTERRUPT
    touchINT_cnt = 0;
    IamAlive_PKT_INT_cnt = 0;
    #endif

	if(power_lock==0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		printk(KERN_INFO "[elan] %s: enter\n", __func__);
		rc = elan_ktf_ts_set_power_state(client, PWR_STATE_IDLE);
	}

#if defined( ESD_CHECK )
	suspend_ESD = 1;
#endif
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
    #ifdef MSM_NEW_VER
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(&client->dev, "Cannot get suspend pinctrl state\n");
	}
    #endif
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 } */

	gpio_direction_input(private_ts->intr_gpio);
	enable_irq_wake(private_ts->irq);
	return 0;
}

static int elan_ktf_ts_resume(struct i2c_client *client)
{
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */
    #ifdef MSM_NEW_VER
    struct elan_ktf_ts_data *data = i2c_get_clientdata(client);
    int err;
    #endif
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 } */
    
	int rc = 0, retry = 3;
#ifdef RE_CALIBRATION
	uint8_t buf_recv[4] = { 0 };
#endif
	if(power_lock==0)   /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
		printk(KERN_INFO "[elan] %s: enter\n", __func__);
#ifdef ELAN_RESUME_RST
		printk("[elan] %s: Used Rest instead of command to resume touch panel\n", __func__);
		elan_ktf_ts_hw_reset();
		/* return 0; */
#endif
		do {
			/* eWD1000 resumes by HW, driver does not have to set power state */
			#if 0
			rc = elan_ktf_ts_set_power_state(client, PWR_STATE_NORMAL);
			mdelay(200);
			#endif
#ifdef RE_CALIBRATION
			rc = i2c_master_recv(client, buf_recv, 4);
			printk("[elan] %s: Re-Calibration Packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
			if (buf_recv[0] != 0x66) {
				mdelay(200);
				rc = i2c_master_recv(client, buf_recv, 4);
				printk("[elan] %s: Re-Calibration Packet, re-try again %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
			}
#endif
			rc = elan_ktf_ts_get_power_state(client);
			if (rc != PWR_STATE_NORMAL)
                printk(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n", __func__, rc);
			else
			break;
		} while (--retry);

	}

#if defined( ESD_CHECK )
	suspend_ESD = 0;
#endif

    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 { */    
    #ifdef MSM_NEW_VER
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
				data->pinctrl_state_active);
		if (err < 0)
			dev_err(&client->dev, "Cannot get active pinctrl state\n");
	}
    #endif
    /* Quanta, BU10SW, Stanley Tsao, 2015.12.22, enable pinctrl usage for msm8909 } */
    
	return 0;
}

static const struct i2c_device_id elan_ktf_ts_id[] = {
	{ ELAN_KTF_NAME, 0 },
	{ }
};

/* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add for support of device tree { */
#ifdef CONFIG_OF
static struct of_device_id elan_ktf_match_table[] = {
	{ .compatible = ELAN_KTF_NAME,},
	{ },
};
#else
#define elan_ktf_match_table NULL
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add for support of device tree } */



static struct i2c_driver ektf_ts_driver = {
	.probe		= elan_ktf_ts_probe,
	.remove		= elan_ktf_ts_remove,
	.suspend	= elan_ktf_ts_suspend,
	.resume		= elan_ktf_ts_resume,
	.id_table	= elan_ktf_ts_id,
	.driver		= {
		.name = ELAN_KTF_NAME,
        /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add for support of device tree { */
        #ifdef CONFIG_OF
        .of_match_table = elan_ktf_match_table,
        #endif
        /* Quanta BU10SW, Stanley Tsao, 2015.12.03, Add for support of device tree } */
	},
};

/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check { */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static int __init elan_ktf_ts_init(void)
#else
static int __devinit elan_ktf_ts_init(void)
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check } */
{
	printk(KERN_INFO "[elan] %s driver version 0x0005: Integrated 2, 5, and 10 fingers together and auto-mapping resolution\n", __func__);
	return i2c_add_driver(&ektf_ts_driver);
}

/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check { */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static void __exit elan_ktf_ts_exit(void)
#else
static void __exit elan_ktf_ts_exit(void)
#endif
/* Quanta BU10SW, Stanley Tsao, 2015.11.17, Add kernel version check } */

{
	i2c_del_driver(&ektf_ts_driver);
	return;
}

module_init(elan_ktf_ts_init);
module_exit(elan_ktf_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");



