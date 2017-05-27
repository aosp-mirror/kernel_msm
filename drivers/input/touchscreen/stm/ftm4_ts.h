#ifndef _LINUX_FTM4_TS_H_
#define _LINUX_FTM4_TS_H_

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c/fts.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_INPUT_BOOSTER
#include <linux/input/input_booster.h>
#endif
#include <linux/atomic.h>

#include <linux/printk.h>
#define tsp_debug_dbg(dev, fmt, ...)	dev_dbg(dev, fmt, ## __VA_ARGS__)
#define tsp_debug_info(dev, fmt, ...)	dev_info(dev, fmt, ## __VA_ARGS__)
#define tsp_debug_err(dev, fmt, ...)	dev_err(dev, fmt, ## __VA_ARGS__)
#ifdef CONFIG_TOUCHSCREEN_FTM4_SHOW_EVENTS
#define tsp_debug_event(dev, fmt, ...)	dev_dbg(dev, fmt, ## __VA_ARGS__)
#else
#define tsp_debug_event(dev, fmt, ...)
#endif

#define USE_OPEN_CLOSE
#define FEATURE_FTS_PRODUCTION_CODE	1

#ifdef USE_OPEN_DWORK
#define TOUCH_OPEN_DWORK_TIME		10
#endif

#define FIRMWARE_IC			"fts_ic"

#define FTS_MAX_FW_PATH			64

#define FTS_TS_DRV_NAME			"fts_touch"
#define FTS_TS_DRV_VERSION		"0132"

#define STM_DEVICE_NAME			"STM"

#define FTS_ID0				0x36
#define FTS_ID1				0x70

#define FTS_SEC_IX1_TX_MULTIPLIER	(4)
#define FTS_SEC_IX1_RX_MULTIPLIER	(2)

#define FTS_DIGITAL_REV_1		0x01
#define FTS_DIGITAL_REV_2		0x02
#define FTS_FIFO_MAX			32
#define FTS_EVENT_SIZE			8
#define FTS_FW_UPDATE_RETRY		3
#define FTS_LOCKDOWNCODE_SIZE		13

#define PRESSURE_MIN			0
#define PRESSURE_MAX			127
#define P70_PATCH_ADDR_START		0x00420000
#define FINGER_MAX			10
#define AREA_MIN			PRESSURE_MIN
#define AREA_MAX			PRESSURE_MAX

#define EVENTID_NO_EVENT		0x00
#define EVENTID_ENTER_POINTER		0x03
#define EVENTID_LEAVE_POINTER		0x04
#define EVENTID_MOTION_POINTER		0x05
#define EVENTID_HOVER_ENTER_POINTER	0x07
#define EVENTID_HOVER_LEAVE_POINTER	0x08
#define EVENTID_HOVER_MOTION_POINTER	0x09
#define EVENTID_PROXIMITY_IN		0x0B
#define EVENTID_PROXIMITY_OUT		0x0C
#define EVENTID_MSKEY			0x0E
#define EVENTID_ERROR			0x0F
#define EVENTID_CONTROLLER_READY	0x10
#define EVENTID_SLEEPOUT_CONTROLLER_READY	0x11
#define EVENTID_RESULT_READ_REGISTER		0x12
#define EVENTID_STATUS_REQUEST_COMP		0x13

#define EVENTID_STATUS_EVENT			0x16
#define EVENTID_INTERNAL_RELEASE_INFO		0x14
#define EVENTID_EXTERNAL_RELEASE_INFO		0x15

#define EVENTID_FROM_STRING			0x80
#define EVENTID_GESTURE				0x20

#define EVENTID_SIDE_SCROLL			0x40
/* side touch event-id for debug, remove after f/w fixed */
#define EVENTID_SIDE_TOUCH_DEBUG		0xDB
#define EVENTID_SIDE_TOUCH			0x0B

#define EVENTID_ERROR_FLASH_CORRUPTION		0x03

/* define flash corruption type */
#define EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_1	0x01
#define EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_2	0x02
#define EVENTID_ERROR_CX_FLASH_CORRUPTION	0x03

#define EVENTID_LOCKDOWN_CODE			0x1E
#define EVENTID_ERROR_LOCKDOWN			0x0B

#define STATUS_EVENT_MUTUAL_AUTOTUNE_DONE	0x01
#define STATUS_EVENT_SELF_AUTOTUNE_DONE		0x02
#define STATUS_EVENT_WATER_SELF_AUTOTUNE_DONE	0x4E
#ifdef FTS_SUPPORT_WATER_MODE
#define STATUS_EVENT_WATER_SELF_DONE 		0x17
#endif
#define STATUS_EVENT_FLASH_WRITE_CONFIG		0x03
#define STATUS_EVENT_FLASH_WRITE_CXTUNE_VALUE	0x04
#define STATUS_EVENT_FORCE_CAL_MUTUAL_SELF	0x05
#define STATUS_EVENT_FORCE_CAL_DONE		0x06

#define STATUS_EVENT_FORCE_CAL_MUTUAL		0x15
#define STATUS_EVENT_FORCE_CAL_SELF		0x06
#define	STATUS_EVENT_PARAM1_FCAL_MS_SS_DONE	0x23
#define STATUS_EVENT_WATERMODE_ON		0x07
#define STATUS_EVENT_WATERMODE_OFF		0x08
#define STATUS_EVENT_RTUNE_MUTUAL		0x09
#define STATUS_EVENT_RTUNE_SELF			0x0A
#define STATUS_EVENT_PANEL_TEST_RESULT		0x0B
#define STATUS_EVENT_GLOVE_MODE			0x0C
#define STATUS_EVENT_RAW_DATA_READY		0x0D
#define STATUS_EVENT_MUTUAL_CAL_FRAME_CHECK	0xC1
#define STATUS_EVENT_SELF_CAL_FRAME_CHECK	0xC2
#define STATUS_EVENT_CHARGER_CONNECTED		0xCC
#define STATUS_EVENT_CHARGER_DISCONNECTED	0xCD
#define STATUS_EVENT_PURE_AUTOTUNE_FLAG_WRITE_FINISH 0x10
#define STATUS_EVENT_PURE_AUTOTUNE_FLAG_CLEAR_FINISH 0x11

#define INT_ENABLE			0x48
#define INT_DISABLE			0x08

#define READ_STATUS			0x84
#define READ_ONE_EVENT			0x85
#define READ_ALL_EVENT			0x86

#define SENSEOFF			0x92
#define SENSEON				0x93
#define FTS_CMD_HOVER_OFF		0x94
#define FTS_CMD_HOVER_ON		0x95

#define FTS_CMD_MSKEY_AUTOTUNE		0x96
#define FTS_CMD_TRIM_LOW_POWER_OSCILLATOR	0x97

#define FTS_CMD_KEY_SENSE_OFF		0x9A
#define FTS_CMD_KEY_SENSE_ON		0x9B
#define FTS_CMD_SET_FAST_GLOVE_MODE	0x9D

#define FTS_CMD_MSHOVER_OFF		0x9E
#define FTS_CMD_MSHOVER_ON		0x9F
#define FTS_CMD_SET_NOR_GLOVE_MODE	0x9F

#define FLUSHBUFFER			0xA1
#define FORCECALIBRATION		0xA2
#define CX_TUNNING			0xA3
#define SELF_AUTO_TUNE			0xA4

#define FTS_CMD_CHARGER_PLUGGED		0xA8
#define FTS_CMD_CHARGER_UNPLUGGED	0xAB

#define FTS_CMD_RELEASEINFO		0xAA
#define FTS_CMD_STYLUS_OFF		0xAB
#define FTS_CMD_STYLUS_ON		0xAC
#define FTS_CMD_LOWPOWER_MODE		0xAD

#define FTS_CMS_ENABLE_FEATURE		0xC1
#define FTS_CMS_DISABLE_FEATURE		0xC2

#define LOCKDOWN_READ			0xC4

#define FTS_CMD_WRITE_PRAM		0xF0
#define FTS_CMD_BURN_PROG_FLASH		0xF2
#define FTS_CMD_ERASE_PROG_FLASH	0xF3
#define FTS_CMD_READ_FLASH_STAT		0xF4
#define FTS_CMD_UNLOCK_FLASH		0xF7
#define FTS_CMD_SAVE_FWCONFIG		0xFB
#define FTS_CMD_SAVE_CX_TUNING		0xFC

#define FTS_CMD_FAST_SCAN		0x01
#define FTS_CMD_SLOW_SCAN		0x02
#define FTS_CMD_USLOW_SCAN		0x03

#define REPORT_RATE_90HZ		0
#define REPORT_RATE_60HZ		1
#define REPORT_RATE_30HZ		2

#define FTS_CMD_STRING_ACCESS		0xEC00
#define FTS_CMD_NOTIFY			0xC0

#define FTS_RETRY_COUNT			10

/* QUICK SHOT : Quick Camera Launching */
#define FTS_STRING_EVENT_REAR_CAM		(1 << 0)
#define FTS_STRING_EVENT_FRONT_CAM		(1 << 1)

/* SCRUB : Display Watch, Event Status / Fast Access Event */
#define FTS_STRING_EVENT_WATCH_STATUS		(1 << 2)
#define FTS_STRING_EVENT_FAST_ACCESS		(1 << 3)
#define FTS_STRING_EVENT_DIRECT_INDICATOR	((1 << 3) | (1 << 2))
#define FTS_STRING_EVENT_SPAY			(1 << 4)
#define FTS_STRING_EVENT_SPAY1			(1 << 5)
#define FTS_STRING_EVENT_SPAY2			((1 << 4) | (1 << 5))

#define FTS_SIDEGESTURE_EVENT_SINGLE_STROKE	0xE0
#define FTS_SIDEGESTURE_EVENT_DOUBLE_STROKE	0xE1
#define FTS_SIDEGESTURE_EVENT_INNER_STROKE	0xE3

#define FTS_SIDETOUCH_EVENT_LONG_PRESS		0xBB
#define FTS_SIDETOUCH_EVENT_REBOOT_BY_ESD	0xED

#define FTS_ENABLE		1
#define FTS_DISABLE		0

#define FTS_MODE_QUICK_SHOT		(1 << 0)
#define FTS_MODE_SCRUB			(1 << 1)
#define FTS_MODE_SPAY			(1 << 1)
#define FTS_MODE_QUICK_APP_ACCESS	(1 << 2)
#define FTS_MODE_DIRECT_INDICATOR	(1 << 3)

#define TSP_BUF_SIZE 2048
#define CMD_STR_LEN 32
#define CMD_RESULT_STR_LEN 2048
#define CMD_PARAM_NUM 8

#define FTS_LOWP_FLAG_QUICK_CAM		(1 << 0)
#define FTS_LOWP_FLAG_2ND_SCREEN	(1 << 1)
#define FTS_LOWP_FLAG_BLACK_UI		(1 << 2)
#define FTS_LOWP_FLAG_QUICK_APP_ACCESS	(1 << 3)
#define FTS_LOWP_FLAG_DIRECT_INDICATOR	(1 << 4)
#define FTS_LOWP_FLAG_SPAY		(1 << 5)
#define FTS_LOWP_FLAG_TEMP_CMD		(1 << 6)

enum fts_error_return {
	FTS_NOT_ERROR = 0,
	FTS_ERROR_INVALID_CHIP_ID,
	FTS_ERROR_INVALID_CHIP_VERSION_ID,
	FTS_ERROR_INVALID_SW_VERSION,
	FTS_ERROR_EVENT_ID,
	FTS_ERROR_TIMEOUT,
	FTS_ERROR_FW_UPDATE_FAIL,
};
#define RAW_MAX	3750
/**
 * struct fts_finger - Represents fingers.
 * @ state: finger status (Event ID).
 * @ mcount: moving counter for debug.
 */
struct fts_finger {
	unsigned char state;
	unsigned short mcount;
	int lx;
	int ly;
};

enum tsp_power_mode {
	FTS_POWER_STATE_ACTIVE = 0,
	FTS_POWER_STATE_LOWPOWER,
	FTS_POWER_STATE_POWERDOWN,
	FTS_POWER_STATE_DEEPSLEEP,
};

enum fts_cover_id {
	FTS_FLIP_WALLET = 0,
	FTS_VIEW_COVER,
	FTS_COVER_NOTHING1,
	FTS_VIEW_WIRELESS,
	FTS_COVER_NOTHING2,
	FTS_CHARGER_COVER,
	FTS_VIEW_WALLET,
	FTS_LED_COVER,
	FTS_CLEAR_FLIP_COVER,
	FTS_QWERTY_KEYBOARD_EUR,
	FTS_QWERTY_KEYBOARD_KOR,
	FTS_MONTBLANC_COVER = 100,
};

enum fts_customer_feature {
	FTS_FEATURE_ORIENTATION_GESTURE = 1,
	FTS_FEATURE_STYLUS,
	FTS_FEATURE_QUICK_SHORT_CAMERA_ACCESS,
	FTS_FEATURE_SIDE_GUSTURE,
	FTS_FEATURE_COVER_GLASS,
	FTS_FEATURE_COVER_WALLET,
	FTS_FEATURE_COVER_LED,
	FTS_FEATURE_COVER_CLEAR_FLIP,
	FTS_FEATURE_DUAL_SIDE_GUSTURE,
	FTS_FEATURE_CUSTOM_COVER_GLASS_ON,
};

enum ftsito_error_type {
	NO_ERROR = 0,
	ITO_FORCE_OPEN,
	ITO_SENSE_OPEN,
	ITO_FORCE_SHRT_GND,
	ITO_SENSE_SHRT_GND,
	ITO_FORCE_SHRT_VCM,
	ITO_SENSE_SHRT_VCM,
	ITO_FORCE_SHRT_FORCE,
	ITO_SENSE_SHRT_SENSE,
	ITO_F2E_SENSE,
	ITO_FPC_FORCE_OPEN,
	ITO_FPC_SENSE_OPEN,
	ITO_KEY_FORCE_OPEN,
	ITO_KEY_SENSE_OPEN,
	ITO_RESERVED0,
	ITO_RESERVED1,
	ITO_RESERVED2,
	ITO_MAX_ERR_REACHED = 0xFF
};

struct fts_version {
	u8 build: 4;
	u8 major: 4;
	u8 minor;
};

struct fts_prd_info {
	u8 product_id[3];
	u8 chip_rev:4;
	u8 fpc_rev:4;
	u8 t_sensor_rev;
	u8 site;
	u8 inspector_no;
	u8 date[6];
};

struct fts_flash_corruption_info {
	bool fw_broken;
	bool cfg_broken;
	bool cx_broken;
};

struct fts_ts_info {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct timer_list timer_charger;
	struct timer_list timer_firmware;
	struct work_struct work;

	int irq;
	int irq_type;
	atomic_t irq_enabled;
	struct fts_i2c_platform_data *board;
	void (*register_cb)(void *);
	struct fts_callbacks callbacks;
	bool enabled;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
#ifdef FEATURE_FTS_PRODUCTION_CODE
	struct device *pdc_dev_ts;
	struct list_head cmd_list_head;
	u8 cmd_state;
	char cmd[CMD_STR_LEN];
	int cmd_param[CMD_PARAM_NUM];
	char cmd_result[CMD_RESULT_STR_LEN];
	int cmd_buf_size;
	struct mutex cmd_lock;
	bool cmd_is_running;
	int SenseChannelLength;
	int ForceChannelLength;
	short *pFrame;
	unsigned char *cx_data;
	struct delayed_work cover_cmd_work;
	int delayed_cmd_param[2];
#endif /* FEATURE_FTS_PRODUCTION_CODE */
	bool flip_enable;
	bool run_autotune;
	bool mainscr_disable;
	unsigned int cover_type;

	unsigned char lowpower_flag;
	bool lowpower_mode;
	bool deepsleep_mode;
	bool wirelesscharger_mode;
	int fts_power_state;
#ifdef FTS_SUPPORT_TA_MODE
	bool TA_Pluged;
#endif
	int digital_rev;
	int touch_count;
	struct fts_finger finger[FINGER_MAX];
	bool palm_pressed;

	int touch_mode;
	int retry_hover_enable_after_wakeup;

	struct fts_prd_info prd_info;
	int fw_version_of_ic;		/* firmware version of IC */
	int fw_version_of_bin;		/* firmware version of binary */
	int config_version_of_ic;	/* Config release data from IC */
	int config_version_of_bin;	/* Config release data from IC */
	unsigned short fw_main_version_of_ic;	/* firmware main version of IC */
	unsigned short fw_main_version_of_bin;	/* firmware main version of binary */
	int panel_revision;			/* Octa panel revision */
	int tspid_val;
	int tspid2_val;

#ifdef USE_OPEN_DWORK
	struct delayed_work open_work;
#endif

#ifdef FTS_SUPPORT_NOISE_PARAM
	struct fts_noise_param noise_param;
	int (*fts_get_noise_param_address)(struct fts_ts_info *info);
#endif
	unsigned int delay_time;
	unsigned int debug_string;
	struct delayed_work reset_work;

	unsigned int scrub_id;
	unsigned int scrub_x;
	unsigned int scrub_y;

	struct mutex i2c_mutex;
	struct mutex device_mutex;
	spinlock_t lock;
	bool touch_stopped;
	bool reinit_done;

	unsigned char data[FTS_EVENT_SIZE * FTS_FIFO_MAX];
	unsigned char ddi_type;

	char test_fwpath[256];
	struct fts_version ic_fw_ver;

	unsigned char o_afe_ver;
	unsigned char afe_ver;

	struct fts_flash_corruption_info flash_corruption_info;

	unsigned int checksum_error;

	int (*stop_device)(struct fts_ts_info *info);
	int (*start_device)(struct fts_ts_info *info);

	int (*fts_write_reg)(struct fts_ts_info *info, unsigned char *reg,
			unsigned short num_com);
	int (*fts_read_reg)(struct fts_ts_info *info, unsigned char *reg,
			int cnum, unsigned char *buf, int num);
	int (*fts_systemreset)(struct fts_ts_info *info);
	int (*fts_wait_for_ready)(struct fts_ts_info *info);
	void (*fts_command)(struct fts_ts_info *info, unsigned char cmd);
	void (*fts_enable_feature)(struct fts_ts_info *info, unsigned char cmd,
			int enable);
#ifdef FEATURE_FTS_PRODUCTION_CODE
	int (*fts_get_channel_info)(struct fts_ts_info *info);
	int (*fts_get_version_info)(struct fts_ts_info *info);
	void (*fts_interrupt_set)(struct fts_ts_info *info, int enable);
	void (*fts_irq_enable)(struct fts_ts_info *info, bool enable);
	void (*fts_release_all_finger)(struct fts_ts_info *info);
#endif /* FEATURE_FTS_PRODUCTION_CODE */
};

#ifdef FEATURE_FTS_PRODUCTION_CODE
#define FTS_CMD(name, func)	.cmd_name = name, .cmd_func = func
struct fts_cmd {
	struct list_head list;
	const char *cmd_name;
	void (*cmd_func)(void *device_data);
};

extern struct fts_cmd fts_commands[];
#endif /* FEATURE_FTS_PRODUCTION_CODE */

#define WRITE_CHUNK_SIZE			32
#define FLASH_CHUNK				(64 * 1024)
#define DMA_CHUNK				32

#define FW_HEADER_SIZE				64
#define FW_HEADER_FTB_SIGNATURE			0xAA55AA55
#define FW_FTB_VER				0x00000001
#define FW_BYTES_ALLIGN				4
#define FW_BIN_VER_OFFSET			16
#define FW_BIN_CONFIG_VER_OFFSET		20

/* Command for flash */
#define FLASH_CMD_UNLOCK			0xF7
#define FLASH_CMD_WRITE_64K			0xF8
#define FLASH_CMD_READ_REGISTER			0xF9
#define FLASH_CMD_WRITE_REGISTER		0xFA

/* Parameters for commands */
#define ADDR_WARM_BOOT				0x001E
#define WARM_BOOT_VALUE				0x38
#define FLASH_ADDR_CODE				0x00000000
#define FLASH_ADDR_CONFIG			0x0000FC00

#define FLASH_UNLOCK_CODE0			0x74
#define FLASH_UNLOCK_CODE1			0x45

#define FLASH_ERASE_UNLOCK_CODE0		0x72
#define FLASH_ERASE_UNLOCK_CODE1		0x03
#define FLASH_ERASE_UNLOCK_CODE2		0x02
#define FLASH_ERASE_CODE0			0x02
#define FLASH_ERASE_CODE1			0xC0
#define FLASH_DMA_CODE0				0x05
#define FLASH_DMA_CODE1				0xC0
#define FLASH_DMA_CONFIG			0x06

enum binfile_type {
	BIN_FTS128 = 1,
	BIN_FTS256 = 2,
	BIN_FTB = 3
};

struct FW_FTB_HEADER {
	uint32_t	signature;
	uint32_t	ftb_ver;
	uint32_t	target;
	uint32_t	fw_id;
	uint32_t	fw_ver;
	uint32_t	cfg_id;
	uint32_t	cfg_ver;
	uint32_t	reserved[2];
	uint32_t	bl_fw_ver;
	uint32_t	ext_ver;
	uint32_t	sec0_size;
	uint32_t	sec1_size;
	uint32_t	sec2_size;
	uint32_t	sec3_size;
	uint32_t	hdr_crc;
};

#ifdef FEATURE_FTS_PRODUCTION_CODE
enum fts_system_information_address {
	FTS_SI_FILTERED_RAW_ADDR		= 0x02,
	FTS_SI_STRENGTH_ADDR			= 0x04,
	FTS_SI_SELF_FILTERED_FORCE_RAW_ADDR	= 0x1E,
	FTS_SI_SELF_FILTERED_SENSE_RAW_ADDR	= 0x20,
	FTS_SI_NOISE_PARAM_ADDR			= 0x40,
	FTS_SI_PURE_AUTOTUNE_FLAG		= 0x4E,
	FTS_SI_COMPENSATION_OFFSET_ADDR		= 0x50,
	FTS_SI_PURE_AUTOTUNE_CONFIG		= 0x52,
	FTS_SI_FACTORY_RESULT_FLAG		= 0x56,
	FTS_SI_AUTOTUNE_CNT			= 0x58,
	FTS_SI_SENSE_CH_LENGTH			= 0x5A, /* 2 bytes */
	FTS_SI_FORCE_CH_LENGTH			= 0x5C, /* 2 bytes */
	FTS_SI_FINGER_THRESHOLD			= 0x60, /* 2 bytes */
	FTS_SI_AUTOTUNE_PROTECTION_CONFIG	= 0x62, /* 2 bytes */
	FTS_SI_REPORT_PRESSURE_RAW_DATA		= 0x64, /* 2 bytes */
	FTS_SI_SS_KEY_THRESHOLD			= 0x66, /* 2 bytes */
	FTS_SI_MS_TUNE_VERSION			= 0x68, /* 2 bytes */
	FTS_SI_CONFIG_CHECKSUM			= 0x6A, /* 4 bytes */
	FTS_SI_PRESSURE_FILTERED_RAW_ADDR	= 0x70,
	FTS_SI_PRESSURE_STRENGTH_ADDR		= 0x72,
	FTS_SI_PRESSURE_THRESHOLD		= 0x76,
};
#endif

void fts_delay(unsigned int ms);
int fts_cmd_completion_check(struct fts_ts_info *info, uint8_t event1, uint8_t event2, uint8_t event3);
int fts_fw_update(struct fts_ts_info *info);
int fts_fw_verify_update(struct fts_ts_info *info);
int fts_get_version_info(struct fts_ts_info *info);
void fts_get_afe_info(struct fts_ts_info *info);
void fts_execute_autotune(struct fts_ts_info *info);
int fts_fw_wait_for_event(struct fts_ts_info *info, unsigned char eid1, unsigned char eid2);
int fts_systemreset(struct fts_ts_info *info);
int fts_wait_for_ready(struct fts_ts_info *info);
int fts_read_chip_id(struct fts_ts_info *info);

#ifdef FEATURE_FTS_PRODUCTION_CODE
int fts_fw_wait_for_specific_event(struct fts_ts_info *info,
		unsigned char eid0, unsigned char eid1, unsigned char eid2);
void procedure_cmd_event(struct fts_ts_info *info, unsigned char *data);
void fts_production_init(void *device_info);
#endif
#endif /* _LINUX_FTM4_TS_H_ */
