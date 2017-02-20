/* include/linux/lge_touch_core.h
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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

#ifndef LGE_TOUCH_CORE_H
#define LGE_TOUCH_CORE_H
#include <linux/wakelock.h>
#include <linux/interrupt.h>

#define MAX_FINGER	10
#define MAX_BUTTON	4
#define FW_VER_INFO_NUM 4
#define MAX_POINT_SIZE_FOR_LPWG	256
#define I2C_MAX_TRY 3
#define MAX_RETRY_COUNT 3
#define TRX_MAX 32


struct bouncing_filter_role {
	u32	enable;
};

struct grip_filter_role {
	u32	enable;
	u32	edge_region;
	u32	max_delta;
	u32	width_ratio;
};

struct accuracy_filter_role {
	u32	enable;
	u32	min_delta;
	u32	curr_ratio;
	u32	min_pressure;
};

struct jitter_filter_role {
	u32	enable;
	u32	curr_ratio;
};

struct quickcover_filter_role {
	u32 enable;
	u32 X1;
	u32 X2;
	u32 Y1;
	u32 Y2;
};

struct ghost_role {
	u32 ghost_chk_cnt;
	u32 jitter_value;
	u32 first_finger_time;
	u32 ta_debouncing_cnt;
	u32 ta_debouncing_finger_num;
	bool pressure_zero;
	bool pressure_high;
	u32 press_interval;
	u32 diff_finger_num;
	u32 subtraction_time;
	u32 subtraction_finger_cnt;
	bool force_continuous_mode;
	u32 long_press_chk_time;
	u32 long_press_cnt;
	u32 button_int_num;
	u32 button_duration;
	u32 rebase_since_init;
	u32 rebase_since_rebase;
	u8 enable;
	u8 ta_noise_chk;
	u8 incoming_call_chk;
	u8 first_finger_chk;
	u8 pressure_zero_chk;
	u8 pressure_high_chk;
	u8 ta_debouncing_chk;
	u8 press_interval_chk;
	u8 diff_fingers_chk;
	u8 subtraction_finger_chk;
	u8 long_press_chk;
	u8 button_chk;
	u8 rebase_repetition_chk;
};

struct crack_role {
	u32 use_crack_mode; /* Yes = 1, No = 0 */
	u32 min_cap_value;
};

struct swp_caps {
	u32 min_distance;
	u32 ratio_thres;
	u32 ratio_chk_period;
	u32 ratio_chk_min_distance;
	u32 min_time_thres;
	u32 max_time_thres;
	u32 active_area_x0;
	u32 active_area_y0;
	u32 active_area_x1;
	u32 active_area_y1;
	u32 min_distance_reg_offset;
	u32 ratio_thres_reg_offset;
	u32 ratio_chk_period_reg_offset;
	u32 ratio_chk_min_distance_reg_offset;
	u32 min_time_thres_reg_offset;
	u32 max_time_thres_reg_offset;
	u32 active_area_x0_reg_offset;
	u32 active_area_y0_reg_offset;
	u32 active_area_x1_reg_offset;
	u32 active_area_y1_reg_offset;
};

struct touch_device_caps {
	u32	button_support;
	u32	number_of_button;
	u32	button_name[MAX_BUTTON];
	u32	max_x;
	u32	max_y;
	u32	max_pressure;
	u32	max_width;
	u32	max_orientation;
	u32	max_id;
};

struct touch_operation_role {
	u32	protocol_type;
	u32	report_mode;
	u32	delta_pos_threshold;
	u32	booting_delay;
	u32	reset_delay;
	u32	softreset_delay;
	u32	wake_up_by_touch;
	u32	use_sleep_mode; /* Yes = 1, No = 0 */
	u32     use_lpwg_all;
	u32     touch_solution;
	u32     use_security_mode;
	u32	use_lcd_notifier_callback;
	u32     thermal_check;
	u32	use_hover_finger;
	u32	use_rmi_dev;
	u32	use_lpwg_test;
	u32	mfts_lpwg;
	u32	palm_ctrl_mode;
	u32	fw_index;
	unsigned long	irqflags;
	struct bouncing_filter_role	*bouncing_filter;
	struct grip_filter_role		*grip_filter;
	struct accuracy_filter_role	*accuracy_filter;
	struct jitter_filter_role	*jitter_filter;
	struct quickcover_filter_role	*quickcover_filter;
	struct ghost_role		*ghost;
	struct crack_role		*crack;
};

struct touch_power_module {
	u32	use_regulator;
	char	vdd[30];
	u32	vdd_voltage;
	char	vio[30];
	u32	vio_voltage;
	u32	vio_control;
	u32	reset_control;
	int	(*power)(int on);
};

struct touch_firmware_module {
	char	fw_image[256];
	u8	need_upgrade;
};

struct touch_platform_data {
	u32	int_pin;
	u32	reset_pin;
	int	maker_id_pin;
	int	panel_id;
	u32     vio_pin;
	u32 fw_version[FW_VER_INFO_NUM];
	struct mutex			thread_lock;
	struct touch_device_caps	*caps;
	struct touch_operation_role	*role;
	struct touch_power_module	*pwr;
	struct touch_firmware_module	*fw;
	struct swp_caps			*swp_down_caps;
	struct swp_caps			*swp_up_caps;
	const char *inbuilt_fw_name;
	const char *inbuilt_fw_name_list[4];
	const char *panel_spec;
	const char *panel_spec_mfts_folder;
	const char *panel_spec_mfts_flat;
	const char *panel_spec_mfts_curved;
	int tx_cap[TRX_MAX];
	int rx_cap[TRX_MAX];
	int tx_ch_count;
	int rx_ch_count;
	u8 touch_count_num;
	u32 ref_chk_option[4];
	u32 fw_ver_addr;
	u32 fw_ver_addr_list[4];
	u32 fw_pid_addr;
	u32 fw_pid_addr_list[4];
	u8 swipe_stat[2];
	u8 swipe_pwr_ctr;
	int lockscreen_stat;
};

struct t_data {
	u16	id;
	u16	x;
	u16	y;
	u16	raw_x;		/* Do not change it. (in filter function) */
	u16	raw_y;		/* Do not change it. (in filter function) */
	u16	width_major;
	u16	width_minor;
	u16	orientation;
	u16	pressure;
	/* finger, palm, pen, glove, hover */
	u16	type;
};

struct b_data {
	u16	key_code;
	u16	state;
};

struct touch_data {
	u8		total_num;
	u8		state;
	u32		id_mask;
	u32		report_id_mask;
	struct t_data	touch[MAX_FINGER];
	struct b_data	button;
};

struct touch_fw_info {
	u8      ic_identifier[31];	/* String */
	u8      ic_version[11];		/* String */
	char	fw_path[256];		/* used for dynamic firmware upgrade */
	u8      force_upgrade;		/* used for dynamic firmware upgrade */
	u8      force_upgrade_cat;	/* used for dynamic firmware upgrade */
};

struct bouncing_filter_data {
	u16	count;
};

struct grip_filter_data {
	u8	grip_mask;
};

struct accuracy_filter_data {
	u8	do_filter;
	u16	down_x;
	u16	down_y;
	u16	down_z;
	u32	delta_pos;
	u32	delta_z;
	int	mod_x;
	int	mod_y;
};

struct jitter_filter_data {
	int	delta_x;
	int	delta_y;
};

struct quickcover_filter_data {
	u8 quickcover_mask;
};

struct bouncing_filter_info {
	struct bouncing_filter_role	*role;
	struct bouncing_filter_data	data;
};

struct grip_filter_info {
	struct grip_filter_role		*role;
	struct grip_filter_data		data;
};

struct accuracy_filter_info {
	struct accuracy_filter_role	*role;
	struct accuracy_filter_data	data[MAX_FINGER];
};

struct jitter_filter_info {
	struct jitter_filter_role	*role;
	struct jitter_filter_data	data[MAX_FINGER];
};

struct quickcover_filter_info {
	struct quickcover_filter_role	*role;
	struct quickcover_filter_data	data;
};

struct state_info {
	atomic_t power;
	atomic_t device_init;
	atomic_t interrupt;
	atomic_t upgrade;
	atomic_t ta;
	atomic_t temperature;
	atomic_t proximity;
	atomic_t hallic;
	atomic_t uevent;
	atomic_t pm;
	atomic_t rebase;
	atomic_t incoming_call;
	atomic_t crack_test;
};

struct point {
	int x;
	int y;
};

/* filter_func
 *
 * report_id_mask
 * - If an event is not included 'report_id_mask', it will not be reported.
 *
 * return value (frt : filter_return_type)
 * - return FRT_IGNORE if the event should be ignored. (FRT_REPORT if not)
 *
 * Each fliter can change only the 'curr_data' and 'filter_info'.
 * It should NOT change any other values such as 'pdata', 'prev_data', etc.
 */
enum filter_return_type {
	FRT_IGNORE = 0,
	FRT_REPORT,
};

struct filter_func {
	char *name;
	enum filter_return_type (*filter)(struct i2c_client *client,
					  u32 *report_id_mask);
	struct list_head list;
};

struct filter_data {
	u16 id_checklist[MAX_FINGER];
	u16 index;
};


/* touch_device_driver
 *
 * return values
 * : All functions in 'touch_device_driver'
 *   should use 'error_type' for return value.
 *
 * - NO_ERROR : NO Problem.
 * - ERROR : Error occurs, so the device will be reset.
 * - IGNORE_EVENT : Event will be ignored.
 * - IGNORE_EVENT_BUT_SAVE_IT : Event will not be reported,
 *                              but saved in 'prev' data.
 *
 */

enum error_type {
	NO_ERROR = 0,
	ERROR,
	NO_FILTER,
	NO_UPGRADE,
	IGNORE_EVENT,
	IGNORE_EVENT_BUT_SAVE_IT,
	ERROR_IN_LPWG,
};

enum window_status {
	NO_CRACK = 0,
	CRACK,
};

struct touch_device_driver {
	enum error_type (*probe)(struct i2c_client *client,
			struct touch_platform_data *lge_ts_data,
			struct state_info *state);
	enum error_type (*remove)(struct i2c_client *client);
	enum error_type (*shutdown)(struct i2c_client *client);
	enum error_type (*suspend)(struct i2c_client *client);
	enum error_type (*resume)(struct i2c_client *client);
	enum error_type (*init)(struct i2c_client *client);
	enum error_type (*data)(struct i2c_client *client,
			struct touch_data *curr_data,
			const struct touch_data *prev_data);
	enum error_type (*filter)(struct i2c_client *client,
			struct touch_data *curr_data,
			const struct touch_data *prev_data);
	enum error_type (*power)(struct i2c_client *client, int power_ctrl);
	enum error_type (*ic_ctrl)(struct i2c_client *client,
			u8 code, u32 value, u32 *ret);
	enum error_type (*fw_upgrade)(struct i2c_client *client,
			struct touch_fw_info *info,
			struct touch_firmware_module *fw);
	enum error_type (*notify)(struct i2c_client *client,
			u8 code, u32 value);
	enum error_type (*lpwg)(struct i2c_client *client,
			u32 code, int64_t value, struct point *data);
	void (*ime_drumming)(struct i2c_client *client, int ime_status);
	void (*toggle_swipe)(struct i2c_client *client);
	enum window_status (*inspection_crack)(struct i2c_client *client);
	void (*sleepmode_change)(struct i2c_client *client);
	void (*incoming_call)(struct i2c_client *client, int value);
	int (*register_sysfs)(struct kobject *k);
	int (*get_type_bl)(struct i2c_client *client);
};

struct lge_touch_data {
	void				*h_touch;
	struct state_info		state;
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct touch_platform_data	*pdata;
	struct touch_data		curr_data;
	struct touch_data		prev_data;
	struct touch_data		report_data;
	struct touch_fw_info		fw_info;
	struct kobject			lge_touch_kobj;
	struct delayed_work		work_init;
	struct delayed_work		work_upgrade;
	struct delayed_work		work_ime_drumming;
	struct delayed_work		work_swipe;
	struct delayed_work		work_trigger_handle;
	struct delayed_work             work_thermal;
	struct delayed_work             work_crack;
	struct bouncing_filter_info	bouncing_filter;
	struct grip_filter_info		grip_filter;
	struct accuracy_filter_info	accuracy_filter;
	struct jitter_filter_info	jitter_filter;
	struct quickcover_filter_info	quickcover_filter;
	struct filter_func		filter_head;
	struct filter_data		f_data;
	struct wake_lock		lpwg_wake_lock;
	struct notifier_block		notif;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_suspend;
#elif defined(CONFIG_FB)
	struct notifier_block		fb_notif;
#endif
	struct pinctrl			*ts_pinctrl;
	struct pinctrl_state		*ts_pinset_state_active;
	struct pinctrl_state		*ts_pinset_state_suspend;
};

enum {
	MT_PROTOCOL_A = 0,
	MT_PROTOCOL_B,
};

enum {
	CHARGERLOGO_MODE = 0,
	NORMAL_BOOT_MODE,
};

enum {
	POWER_OFF = 0,
	POWER_ON,
	POWER_SLEEP,
	POWER_WAKE,
	POWER_SLEEP_STATUS,
};

enum {
	INTERRUPT_DISABLE = 0,
	INTERRUPT_ENABLE,
};

enum {
	TA_DISCONNECTED = 0,
	TA_CONNECTED,
};

enum {
	PROXIMITY_FAR = 0,
	PROXIMITY_NEAR,
};

enum {
	HALL_NONE = 0,
	HALL_COVERED,
};

enum {
	UEVENT_IDLE = 0,
	UEVENT_BUSY,
};

enum {
	UPGRADE_FINISH = 0,
	UPGRADE_START,
};

enum {
	CRACK_TEST_FINISH = 0,
	CRACK_TEST_START,
};

enum {
	NON_FILTER = 0,
	BOUNCING_FILTER,
	GRIP_FILTER,
	ACCURACY_FILTER,
	JITTER_FILTER,
	QUICKCOVER_FILTER,
};

enum {
	FINGER = 0,
	PALM,
	PEN,
	GLOVE,
	HOVER,
};

enum {
	KEY_NONE = 0,
	TOUCH_HARD_KEY,
	TOUCH_SOFT_KEY,
	VIRTUAL_KEY,
};

enum {
	CONTINUOUS_MODE = 0,
	REDUCED_MODE,
	DEFAULT_REPORT_MODE,
};

enum {
	RESET_NONE = 0,
	SOFT_RESET,
	PIN_RESET,
	VDD_RESET,
};

enum {
	PM_RESUME = 0,
	PM_SUSPEND,
	PM_SUSPEND_IRQ,
};

enum {
	REBASE_DONE = 0,
	REBASE_DOING,
};

enum {
	INIT_NONE = 0,
	INIT_DONE,
};

enum {
	BUTTON_RELEASED	= 0,
	BUTTON_PRESSED	= 1,
	BUTTON_CANCLED	= 0xff,
};

enum {
	IC_CTRL_READ = 1,
	IC_CTRL_WRITE,
	IC_CTRL_BASELINE_REBASE,
	IC_CTRL_REPORT_MODE,
	IC_CTRL_THERMAL,
	IC_CTRL_RESET,
};

enum {
	NOTIFY_TA_CONNECTION = 1,
	NOTIFY_HDMI_CONNECTION,
	NOTIFY_TEMPERATURE_CHANGE,
	NOTIFY_PROXIMITY,
	NOTIFY_HALL_IC,
};

enum {
	LPWG_NONE = 0,
	LPWG_DOUBLE_TAP,
	LPWG_PASSWORD,
	LPWG_SIGNATURE,
	LPWG_SWIPE_DOWN,
	LPWG_SWIPE_UP,
};

enum {
	LPWG_READ = 1,
	LPWG_ENABLE,
	LPWG_LCD_X,
	LPWG_LCD_Y,
	LPWG_ACTIVE_AREA_X1,
	LPWG_ACTIVE_AREA_X2,
	LPWG_ACTIVE_AREA_Y1,
	LPWG_ACTIVE_AREA_Y2,
	LPWG_TAP_COUNT,
	LPWG_LENGTH_BETWEEN_TAP,
	LPWG_EARLY_SUSPEND,
	LPWG_SENSOR_STATUS,
	LPWG_DOUBLE_TAP_CHECK,
	LPWG_REPLY,
	LPWG_UPDATE_ALL,
	LPWG_INCELL_LPWG_ON,
	LPWG_INCELL_LPWG_OFF,
	LPWG_INCELL_NO_SLEEP,
};

enum {
	DEBUG_NONE			= 0,
	DEBUG_BASE_INFO			= (1U << 0),	/* 1 */
	DEBUG_TRACE			= (1U << 1),	/* 2 */
	DEBUG_GET_DATA			= (1U << 2),	/* 4 */
	DEBUG_ABS			= (1U << 3),	/* 8 */
	DEBUG_BUTTON			= (1U << 4),	/* 16*/
	DEBUG_FW_UPGRADE		= (1U << 5),	/* 32 */
	DEBUG_GHOST			= (1U << 6),	/* 64 */
	DEBUG_IRQ_HANDLE		= (1U << 7),	/* 128 */
	DEBUG_POWER			= (1U << 8),	/* 256 */
	DEBUG_JITTER			= (1U << 9),	/* 512 */
	DEBUG_ACCURACY			= (1U << 10),	/* 1024 */
	DEBUG_BOUNCING			= (1U << 11),	/* 2048 */
	DEBUG_GRIP			= (1U << 12),	/* 4096 */
	DEBUG_FILTER_RESULT		= (1U << 13),	/* 8192 */
	DEBUG_QUICKCOVER		= (1U << 12),	/* 4096 */
	DEBUG_LPWG			= (1U << 14),	/* 16384 */
	DEBUG_NOISE			= (1U << 15),	/* 32768 */
	DEBUG_LPWG_COORDINATES          = (1U << 16),   /* 65536 */
};

#ifdef LGE_TOUCH_TIME_DEBUG
enum {
	TIME_INTERRUPT = 0,
	TIME_WORKQUEUE_START,
	TIME_WORKQUEUE_END,
	TIME_FW_UPGRADE_START,
	TIME_FW_UPGRADE_END,
	TIME_PROFILE_MAX,
};

enum {
	DEBUG_TIME_PROFILE_NONE			= 0,
	DEBUG_TIME_INTERRUPT			= (1U << 0),	/* 1 */
	DEBUG_TIME_WORKQUEUE			= (1U << 1),	/* 2 */
	DEBUG_TIME_FW_UPGRADE			= (1U << 2),	/* 4 */
	DEBUG_TIME_PROFILE_ALL			= (1U << 3),	/* 8 */
};
#endif

enum {
	KEYGUARD_RESERVED = 0,
	KEYGUARD_ENABLE,
};

enum {
	IME_OFF = 0,
	IME_ON,
	IME_SWYPE,
};

enum {
	QUICKCOVER_OPEN = 0,
	QUICKCOVER_CLOSE,
};

enum {
	NO_CAL = (1U << 0),
	FORCE_CAL = (1U << 1),
};

enum {
	INCOMING_CALL_IDLE,
	INCOMING_CALL_RINGING,
	INCOMING_CALL_OFFHOOK,
};

enum {
	EX_INIT,
	EX_1ST_INT,
	EX_PREV_PRESS,
	EX_CURR_PRESS,
	EX_BTN_START,
	EX_BTN_END,
	EX_1ST_GHOST,
	EX_2ND_GHOST,
	EX_CURR_INT,
	EX_PROFILE_MAX
};

enum ghost_error_type {
	NO_ACTION,
	ERROR_CASE,
	NEED_TO_OUT,
	DONE_REBASE,
	NEED_TO_INIT,
	NEED_TO_REBASE,
	NEED_SOFT_RESET,
};

enum{
	PALM_REJECT_FW,
	PALM_REJECT_DRIVER,
	PALM_REPORT,
};

enum{
	BL_VER_LOWER = 0,
	BL_VER_HIGHER = 1 << 1,
	NO_NEED_BL =	1 << 2,
};

enum {
	SWIPE_DONE = 0,
	DO_SWIPE,
};

enum {
	WAIT_SWIPE_WAKEUP,
	SKIP_PWR_CON,
	WAIT_TOUCH_PRESS,
	WAIT_TOUCH_RELEASE,
};
enum {
	SET_SLEEP_STATUS = 0,
	SET_NO_SLEEP,
	SET_EARLY_RESET,
	SET_POWER_OFF,
	SET_DEEP_ACTIVE,
};

#define LGE_TOUCH_NAME		"lge_touch"

/* Basic Logging Macro
 *
 */
#define TOUCH_I(fmt, args...)			\
	pr_err("[Touch__] " fmt, ##args)

#define TOUCH_E(fmt, args...)		\
	pr_err("[Touch E__] [%s %d] " fmt,	\
		__func__, __LINE__, ##args)

/* For Error Handling
  *
  * DO_IF : execute 'do_work', and if the result is true,
  *         print 'error_log' and goto 'goto_error'.
  * DO_SAFE : execute 'do_work', and if the result is '< 0',
  *           print 'error_log' and goto 'goto_error'
  * ASSIGN : excute 'do_assign', and if the result is 'NULL',
  *          print 'error_log' and goto 'goto_error'
  * ERROR_IF : if the condition is true(ERROR),
  *            print 'string' and goto 'goto_error'.
  */
#define DO_IF(do_work, goto_error)				\
do {								\
	if (do_work) {						\
		pr_info("[Touch E__] Action Failed [%s %d]\n",	\
			__func__, __LINE__); \
		goto goto_error;				\
	}							\
} while (0)

#define DO_SAFE(do_work, goto_error)				\
	DO_IF(unlikely((do_work) < 0), goto_error)

#define ASSIGN(do_assign, goto_error)				\
do {								\
	if ((do_assign) == NULL) {				\
		pr_info("[Touch E__] Assign Failed [%s %d]\n",	\
			__func__, __LINE__);		\
		goto goto_error;				\
	}							\
} while (0)

#define ERROR_IF(cond, string, goto_error)			\
do {								\
	if (cond) {						\
		TOUCH_E(string);				\
		goto goto_error;				\
	}							\
} while (0)


/* For using debug_mask more easily
  *
  */
extern u32 touch_ts_debug_mask;
#define TOUCH_D(condition, fmt, args...)			\
do {								\
	if (unlikely(touch_ts_debug_mask & (condition)))		\
		pr_info("[Touch__] " fmt, ##args);	\
} while (0)

#ifdef LGE_TOUCH_TIME_DEBUG
extern u32 touch_time_debug_mask;
#define TOUCH_TIME_DEBUG(condition, fmt, args...)		\
do {								\
	if (unlikely(touch_time_debug_mask & (condition)))	\
		pr_info("[Touch__] " fmt, ##args);	\
} while (0)
#endif

#define TOUCH_TRACE()						\
	TOUCH_D(DEBUG_TRACE, " * %s %d\n", __func__, __LINE__)


extern int mfts_mode;
extern int boot_mode;
extern struct pseudo_batt_info_type pseudo_batt_info;

extern int is_sensing;
extern bool i2c_suspended;
extern bool ghost_do_not_reset;
extern void release_all_touch_event(struct lge_touch_data *ts);
extern int touch_ic_init(struct lge_touch_data *ts, int is_error);
extern irqreturn_t touch_irq_handler(int irq, void *dev_id);
extern irqreturn_t touch_thread_irq_handler(int irq, void *dev_id);

extern void touch_ts_enable_irq(unsigned int irq);
extern void touch_ts_disable_irq(unsigned int irq);
/* sysfs
 *
 */
struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct i2c_client *client, char *buf);
	ssize_t (*store)(struct i2c_client *client,
		const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name	\
	= __ATTR(_name, _mode, _show, _store)


int  touch_driver_register(struct touch_device_driver *driver,
	struct of_device_id *match_table);
void touch_driver_unregister(void);

void touch_sleep_status(struct i2c_client *client, int data);
void touch_swipe_status(struct i2c_client *client, int mode);
void set_touch_handle(struct i2c_client *client, void *h_touch);
void *get_touch_handle(struct i2c_client *client);
void send_uevent(struct device *dev, char *string[2]);
void send_uevent_lpwg(struct i2c_client *client, int type);
int touch_ts_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf);
int touch_ts_i2c_write(struct i2c_client *client, u8 reg, int len, u8 *buf);
int touch_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data);
#endif
