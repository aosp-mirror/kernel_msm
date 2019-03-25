/*
* Copyright © 2016 FocalTech Systems Co., Ltd.  All Rights Reserved.
*
* This program is free software; you may redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#ifndef __LINUX_FTS_H__
#define __LINUX_FTS_H__

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
/* #include <linux/sensors.h> */
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include "ft_gesture_lib.h"
#include <linux/wakelock.h>
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_DRIVER_INFO  "Qualcomm_Ver 1.3.1 2015-07-06"
#define FT5X06_ID	0x55
#define FT5X16_ID	0x0A
#define FT5X36_ID	0x14
#define FT6X06_ID	0x06
#define FT6X36_ID	0x36

#define FT5316_ID	0x0A
#define FT5306I_ID	0x55

#define LEN_FLASH_ECC_MAX	0xFFFE

#define FTS_MAX_POINTS		2

#define FTS_WORKQUEUE_NAME	"fts_wq"

#define FTS_DEBUG_DIR_NAME	"fts_debug"

#define FTS_INFO_MAX_LEN	512
#define FTS_FW_NAME_MAX_LEN	50

#define FTS_REG_ID		0xA3
#define FTS_REG_FW_VER		0xA6
#define FTS_REG_FW_VENDOR_ID	0xA8
#define FTS_REG_POINT_RATE	0x88

#define FTS_FACTORYMODE_VALUE	0x40
#define FTS_WORKMODE_VALUE	0x00

/* #define CONFIG_TOUCHSCREEN_FTS_PSENSOR */
#define MSM_NEW_VER		/*cotrol new platform*/

#define FTS_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FTS_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= %s\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FTS_DRIVER_INFO, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FTS_DBG_EN 1
#if FTS_DBG_EN
	#define FTS_DBG(fmt, args...)	printk("[fts]" fmt, ## args)
#else
	#define FTS_DBG(fmt, args...)	do { } while (0)
#endif

#define FTS_INFO(fmt, args...) printk(KERN_INFO "[FTS][Info]"fmt"\n", ##args)
#define FTS_ERROR(fmt, args...) printk(KERN_ERR "[FTS][Error]"fmt"\n", ##args)

#define NORMAL_MODE		FTS_WORKMODE_VALUE
#define FACTORY_MODE		FTS_FACTORYMODE_VALUE

/* Working Mode Register Map */
#define DEV_MODE	0
#define GEST_ID		0x01
#define TD_STATUS	0x02
#define P1_XH		0x03
#define P1_XL		0x04
#define P1_YH		0x05
#define P1_YL		0x06
#define P1_WEIGHT	0x07
#define P1_MISC		0x08
#define P2_XH		0x09
#define P2_XL		0x0A
#define P2_YH		0x0B
#define P2_YL		0x0C
#define P2_WEIGHT	0x0D
#define P2_MISC		0x0E

/* Factory */
#define MAX_CHANNEL_NUMBER	22
#define MAX_KEY_NUMBER	0
#define TEST_PASS		1
#define TEST_FAIL		0
#define ERR_ENTER_FACTORY	-1
#define ERR_ENTER_NORMAL	-2
#define ERR_READ_CHL_KEY	-3
#define ERR_READ_RAW_DATA	-4
#define ERR_READ_CB_DATA	-5
#define ERR_READ_CB_DELTA_DATA	-6

/*TODO: This testing value needs to discuss with Focaltech FAE and TP Maker*/
#define MAX_RAWDATA		17000
#define MIN_RAWDATA		13000
#define MAX_CBDATA		1000
#define MIN_CBDATA		3
#define MAX_CBDELTADATA		200
#define MIN_CBDELTADATA		50
#define TEST_TOLERANCE_PERCENT	25

#define TP_CHNL_NUM		0x0A
#define TP_KEY_NUM		0x0B
#define TP_A_CMD_REG		0x08
#define CB_DATA_ADDR_R		0x33
#define RAW_DATA_ADDR_R		0x34
#define RAW_DATA_BUF		0x35
#define CB_DATA_BUF		0x39
#define FACTORY_TEST_MODE_ADDR	0xAE
/* for Factory mode register 0xAE*/
#define F_NORMAL		0x00
#define F_TESTMODE_1		0x01
#define F_TESTMODE_2		0x02
/* Factory END*/
#define TP_ID_T	0x98
#define TP_ID_S	0x99
#define TP_ID_S_TRULY	0x9A
#define TP_ID_S2	0x9D

#define TP_INI_FILE_T "FT3X67_CEI_FZW6_TULIP_0x98_V03_OnlyTest_20180521.ini"
#define TP_INI_FILE_S "FT3X67_CEI_FZW7_SAKURA_0x99_V04_OnlyTest_20180530.ini"
#define TP_INI_FILE_S_TRULY \
			"FT3267_20180425_FZW7_HOST_Truly_0x9A_V01_OnlyTest.ini"
#define FTS_GESTURE_SETTING_ADRESS	0xD0
#define FTS_GESTURE_OUTPUT_ADRESS	0xD3
#define GESTURE_UNKNOWN	0x00
#define GESTURE_SMALL_AREA	0x25	/* TP Coverage < 50% */
#define GESTURE_LARGE_AREA	0x26	/* TP Coverage > 50% */

#define STATE_NORMAL	0x00
#define STATE_GESTRUE	0x01
/* #define FTS_RECORD_DATA */
#define FTS_GESTRUE_EN

#ifndef FTS_GESTRUE_EN

#define FTS_POWER_CONTROL
#define FTS_GPIO_CONTROL

#endif
#define SELFTEST_ITEM_NUM 4
/* unit : ms */
#define TP_RECOVERY_TIME 400

#define FTS_UPGRADE_AA                              0xAA
#define FTS_UPGRADE_55                              0x55
#define FTS_DELAY_FC_AA                             10
#define FTS_DELAY_UPGRADE_RESET                     80
#define FTS_DELAY_READ_ID                           20
#define FTS_CMD_START1                      0x55
#define FTS_CMD_START2                      0xAA
#define FTS_CMD_START_DELAY                 10
#define FTS_CMD_READ_ID_LEN                 4
#define FTS_CMD_READ_ID_LEN_INCELL          1
#define FTS_UPGRADE_LOOP                            30
#define FTS_CMD_ERASE_APP                           0x61
#define FTS_FLASH_STATUS_OK_FT6336GU                0xB002
#define FTS_CMD_FLASH_STATUS                        0x6A
#define FTS_CMD_FLASH_STATUS_LEN                    2
#define FTS_CMD_FLASH_STATUS_NOP                    0x0000
#define FTS_CMD_FLASH_STATUS_ECC_OK                 0xF055
#define FTS_CMD_FLASH_STATUS_ERASE_OK               0xF0AA
#define FTS_CMD_FLASH_STATUS_WRITE_OK               0x1000
#define FTS_FLASH_PACKET_LENGTH                     32	/* max=128 */
#define FTS_CMD_WRITE_LEN                           6
#define FTS_CMD_WRITE                               0xBF
#define FTS_RETRIES_DELAY_WRITE                     1
#define FTS_DELAY_UPGRADE_RESET                     80
#define FTX_MAX_COMPATIBLE_TYPE                     4
#define FTS_REG_CHIP_ID                     0xA3
#define INTERVAL_READ_REG                   100	/* unit:ms */
#define TIMEOUT_READ_REG                    1000	/* unit:ms */
#define FTS_CMD_READ_ID                     0x90
#define FTS_REG_UPGRADE2                        0xBC
#define FTS_RETRIES_WRITE                           100
#define FTS_CMD_RESET                               0x07
#define BYTE_OFF_0(x)           (u8)((x) & 0xFF)
#define BYTE_OFF_8(x)           (u8)((x >> 8) & 0xFF)
#define BYTE_OFF_16(x)          (u8)((x >> 16) & 0xFF)

enum FW_STATUS {
	FTS_RUN_IN_ERROR,
	FTS_RUN_IN_APP,
	FTS_RUN_IN_ROM,
	FTS_RUN_IN_PRAM,
	FTS_RUN_IN_BOOTLOADER,
};

/* IC info */
struct upgrade_func {
	u64 ctype[FTX_MAX_COMPATIBLE_TYPE];
	int newmode;
	u32 fwveroff;
	u32 fwcfgoff;
	u32 appoff;
	u32 licoff;
	u32 paramcfgoff;
	u32 paramcfgveroff;
	u32 paramcfg2off;
	bool hid_supported;
	bool pramboot_supported;
	u8 *pramboot;
	u32 pb_length;
	int (*init)(void);
	int (*upgrade)(struct i2c_client *, u8 *, u32);
	int (*get_hlic_ver)(u8 *);
	int (*lic_upgrade)(struct i2c_client *, u8 *, u32);
	int (*param_upgrade)(struct i2c_client *, u8 *, u32);
	int (*force_upgrade)(struct i2c_client *, u8 *, u32);
};

struct ft_chip_t {
	u64 type;
	u8 chip_idh;
	u8 chip_idl;
	u8 rom_idh;
	u8 rom_idl;
	u8 pb_idh;
	u8 pb_idl;
	u8 bl_idh;
	u8 bl_idl;
};

struct ts_ic_info {
	bool is_incell;
	bool hid_supported;
	struct ft_chip_t ids;
};

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
enum {
	ENTER_FACTORY_MODE_TEST = 0,
	RAWDATA_TEST,
	CB_TEST,
	DIFF_CB_TEST
};

enum {
	FW_CHECK_PASS = 0,
	FW_CHECK_FAIL,
	TP_ID_IS_WRONG
};

enum {
	DISABLE_TAG = 48,	/* string "0" */
	ENABLE_POINT_TAG,
#ifdef FTS_GESTRUE_EN
	ENABLE_GESTURE_TAG
#endif
};

struct fts_Upgrade_Info {
	u8 CHIP_ID;
	u8 TPD_MAX_POINTS;
	u8 AUTO_CLB;
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_erase_flash;	/*delay of earse flash */
};

struct fts_ts_platform_data {
	struct fts_Upgrade_Info info;
	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	bool psensor_support;
	int (*power_init)(bool);
	int (*power_on)(bool);
};

struct ts_event {
	u16 au16_x[FTS_MAX_POINTS];	/*x coordinate */
	u16 au16_y[FTS_MAX_POINTS];	/*y coordinate */
	u16 pressure[FTS_MAX_POINTS];
	u8 au8_touch_event[FTS_MAX_POINTS];	/*touch event:
						   0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[FTS_MAX_POINTS];	/*touch ID */
	u8 area[FTS_MAX_POINTS];
	u8 touch_point;
	u8 point_num;
#ifdef FTS_GESTRUE_EN
	u8 gesture_id;
#endif
};

struct fts_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct fts_ts_platform_data *pdata;
	struct ts_ic_info ic_info;
	struct fts_psensor_platform_data *psensor_pdata;
	struct work_struct touch_event_work;
	struct delayed_work touch_event_recovery_work;
	struct workqueue_struct *ts_workqueue;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FTS_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	bool suspending;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
#ifdef FTS_GESTRUE_EN
	int tp_gesture_id;
	struct wake_lock	wlock;
#endif
	int touchs;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#ifdef MSM_NEW_VER
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
#endif
/*	int shipping_fw_version; */	/* add for FTM */
};

#ifdef CONFIG_TOUCHSCREEN_FTS_PSENSOR
struct fts_psensor_platform_data {
	struct input_dev *input_psensor_dev;
	struct sensors_classdev ps_cdev;
	int tp_psensor_opened;
	char tp_psensor_data;	/* 0 near, 1 far */
	struct fts_ts_data *data;
};
#endif

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
/* Function Switchs: define to open,  comment to close */
/* #define FTS_GESTRUE_EN 0 */
#define GTP_ESD_PROTECT 0
#define FTS_APK_DEBUG
#define FTS_SYSFS_DEBUG
#define FTS_CTL_IIC
#define FTS_AUTO_UPGRADE

extern struct fts_Upgrade_Info fts_updateinfo_curr;
extern struct i2c_client *fts_i2c_client;
extern struct fts_ts_data *fts_wq_data;
extern struct input_dev *fts_input_dev;
extern bool print_point;

#ifdef FTS_GESTRUE_EN
extern bool print_gesture;
extern bool en_big_area_func;
#endif

extern bool fts_wq_queue_result;
extern unsigned int irq_handler_recovery_count;
extern unsigned int suspend_resume_recovery_count;
extern unsigned int plam_recovery_count;

extern bool ts_pwr_disabled;
/* record tp data */
#define RECORD_SIZE (1024*100)

extern bool en_record_tp_data_func;
extern char TPdata_buf[RECORD_SIZE];
extern int TPdata_buf_len;

static DEFINE_MUTEX(i2c_rw_access);

/* Getstre functions */
extern int fts_Gesture_init(struct input_dev *input_dev);
extern int fts_read_Gestruedata(void);
extern int fetch_object_sample(unsigned char *buf, short pointnum);
extern void init_para(int x_pixel, int y_pixel, int time_slot, int cut_x_pixel,
		      int cut_y_pixel);

/* upgrade functions */
extern void fts_update_fw_vendor_id(struct fts_ts_data *data);
extern void fts_update_fw_ver(struct fts_ts_data *data);
extern void fts_get_upgrade_array(void);
extern int fts_ctpm_auto_upgrade(struct i2c_client *client);
extern int fts_ctpm_auto_upgrade_for_cci(struct i2c_client *client,
					 const u8 tp_id, bool force_upgrade);
extern int fts_fw_upgrade(struct device *dev, bool force);
extern int fts_ctpm_auto_clb(struct i2c_client *client);
extern int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
					     char *firmware_name);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
extern int fts_ctpm_get_i_file_ver(void);

/* Apk and functions */
extern int fts_create_apk_debug_channel(struct i2c_client *client);
extern void fts_release_apk_debug_channel(void);

/* ADB functions */
extern int fts_create_sysfs(struct i2c_client *client);
extern int fts_remove_sysfs(struct i2c_client *client);

/* char device for old apk */
extern int fts_rw_iic_drv_init(struct i2c_client *client);
extern void fts_rw_iic_drv_exit(void);

/* Base functions */
extern int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen,
			char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf,
			 int writelen);
extern int fts_read_reg(struct i2c_client *client, u8 addr, u8 *val);
extern int fts_write_reg(struct i2c_client *client, u8 addr, const u8 val);

extern void fts_reset_chip(void);
extern int fts_ts_stop(struct device *dev);

extern int fts_ts_disable(struct device *dev);
extern int fts_ts_start(struct device *dev);
/*******************************************************************************
* Static function prototypes
*******************************************************************************/
int fts_ts_resume(struct device *dev);

#endif
