/* touch_sw49408.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
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

#ifndef LGE_TOUCH_SW49408_H
#define LGE_TOUCH_SW49408_H

#include "touch_sw49408_asc.h"	/* ASC */
#include <linux/pm_qos.h>

/* debug info */
#define DEBUG_BUF_SIZE 1024

struct sw49408_touch_debug {
	u8 protocol_ver;
	u8 reserved_1;
	u32 frame_cnt;
	u8 rn_max_bfl;
	u8 rn_max_afl;
	u8 rn_min_bfl;
	u8 rn_min_afl;
	u8 rn_max_afl_x;
	u8 rn_max_afl_y;
	u8 forcehold_cnt;
	u8 baseline_sts;
	u8 rnstable_bfl;
	u8 rn_stable;
	u8 seg1_cnt:4;
	u8 seg2_cnt:4;
	u8 seg1_thr;
	u8 rn_pos_cnt;
	u8 rn_neg_cnt;
	u8 rn_pos_sum;
	u8 rn_neg_sum;
	u8 rebase[8];
	u8 rn_max_tobj[12];
	u8 track_bit[10];
	u8 palm[8];
	u8 nm;
	u16 nm_trans_cnt;
	u8 noise_detect[5];
	s8 lf_oft[18];
	u8 reserved_2[15];
	u16 runtime_dbg_case;
	u8 runtime_dbg_inttype;
	u32 ic_debug[3];
	u32 ic_debug_info;
} __packed;

/* report packet */
struct sw49408_touch_data {
	u32 track_id:5;
	u32 tool_type:3;
	u32 angle:8;
	u32 event:2;
	u32 x:14;

	u32 y:14;
	u32 pressure:8;
	u32 reserve1:10;

	u32 reserve2:4;
	u32 width_major:14;
	u32 width_minor:14;
} __packed;

struct sw49408_touch_info {
	u32 ic_status;
	u32 device_status;
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 palm_bit:16;
	struct sw49408_touch_data data[10];
	u32 reserved[3];
	/* debug info */
	struct sw49408_touch_debug debug;
} __packed;

#define WATER_ID			14
#define PALM_ID				15

#define info_ptr_addr			(0x43)

/* device info offset*/
#define tc_version			(0)
#define tc_product_code			(0x1)
#define tc_product_id1			(0x2)
#define tc_product_id2			(0x3)
#define tc_conf_version			(0x4)

#define chip_rev_addr			(0x001)
#define tc_ic_status			(0x200)
/* sw49408_touch_info base addr*/
#define tc_status			(0x201)
#define spr_subdisp_st			(0x022)
#define spr_boot_st			(0x011)

/* device control offset */
#define tc_device_ctl			(0x0)
#define tc_interrupt_ctl		(0x1)
#define tc_interrupt_status		(0x2)
#define tc_driving_ctl			(0x3)
#define tc_flash_dn_ctl			(0x5)

#define command_start_addr		(0xC00)
#define command_end_addr		(0xEFF)

#define tc_rtc_te_interval_cnt  (56)
#define pt_info_lcm_type        (6)
#define pt_info_lot_num         (7)
#define pt_info_fpc_type        (8)
#define pt_info_date            (9)
#define pt_info_time            (10)

#define config_s_info1			(0x480)
#define config_s_info2			(0x481)

/* ABT CMD SPI offset : r_abt_cmd_spi_addr */
#define TCI_ENABLE_W				(0x0)	/* @AP: 0xC30 */
#define TAP_COUNT_W				(0x1)	/* @AP: 0xC31 */
#define MIN_INTERTAP_W				(0x2)	/* @AP: 0xC32 */
#define MAX_INTERTAP_W				(0x3)	/* @AP: 0xC33 */
#define TOUCH_SLOP_W				(0x4)	/* @AP: 0xC34 */
#define TAP_DISTANCE_W				(0x5)	/* @AP: 0xC35 */
#define INT_DELAY_W				(0x6)	/* @AP: 0xC36 */
#define ACT_AREA_X1_W				(0x7)	/* @AP: 0xC37 */
#define ACT_AREA_Y1_W				(0x8)	/* @AP: 0xC38 */
#define ACT_AREA_X2_W				(0x9)	/* @AP: 0xC39 */
#define ACT_AREA_Y2_W				(0xa)	/* @AP: 0xC3A */

#define TCI_FAIL_DEBUG_W			(0xc)	/* @AP: 0xC3C */
#define TCI_FAIL_BIT_W				(0xd)	/* @AP: 0xC3D */

#define SWIPE_ENABLE_W				(0x10)	/* @AP: 0xC40 */
#define SWIPE_DIST_W				(0x11)	/* @AP: 0xC41 */
#define SWIPE_RATIO_THR_W			(0x12)	/* @AP: 0xC42 */
#define SWIPE_RATIO_DIST_W			(0x13)	/* @AP: 0xC43 */
#define SWIPE_RATIO_PERIOD_W			(0x14)	/* @AP: 0xC44 */
#define SWIPE_HORIZONTAL_TIME_MIN_W		(0x15)	/* @AP: 0xC45 */
#define SWIPE_HORIZONTAL_TIME_MAX_W		(0x16)	/* @AP: 0xC46 */
#define SWIPE_HORIZONTAL_ACT_AREA_X1_W		(0x17)	/* @AP: 0xC47 */
#define SWIPE_HORIZONTAL_ACT_AREA_Y1_W		(0x18)	/* @AP: 0xC48 */
#define SWIPE_HORIZONTAL_ACT_AREA_X2_W		(0x19)	/* @AP: 0xC49 */
#define SWIPE_HORIZONTAL_ACT_AREA_Y2_W		(0x1a)	/* @AP: 0xC4A */
#define SWIPE_WRONG_DIRECTION_W			(0x1b)	/* @AP: 0xC4B */
#define SWIPE_HORIZONTAL_START_AREA_X1_W	(0x40)	/* @AP: 0xC66 */
#define SWIPE_HORIZONTAL_START_AREA_Y1_W	(0x41)	/* @AP: 0xC67 */
#define SWIPE_HORIZONTAL_START_AREA_X2_W	(0x42)	/* @AP: 0xC68 */
#define SWIPE_HORIZONTAL_START_AREA_Y2_W	(0x43)	/* @AP: 0xC69 */

#define SWIPE_FAIL_DEBUG_W			(0x1d)	/* @AP: 0xC4D */
#define SWIPE_FAIL_BIT_W			(0x1f)

#define SWIPE_VERTICAL_TIME_MIN_W		(0x20)	/* @AP: 0xC50 */
#define SWIPE_VERTICAL_TIME_MAX_W		(0x21)	/* @AP: 0xC51 */
#define SWIPE_VERTICAL_ACT_AREA_X1_W		(0x22)	/* @AP: 0xC52 */
#define SWIPE_VERTICAL_ACT_AREA_Y1_W		(0x23)	/* @AP: 0xC53 */
#define SWIPE_VERTICAL_ACT_AREA_X2_W		(0x24)	/* @AP: 0xC54 */
#define SWIPE_VERTICAL_ACT_AREA_Y2_W		(0x25)	/* @AP: 0xC55 */
#define SWIPE_VERTICAL_START_AREA_X1_W		(0x44)	/* @AP: 0xC6A */
#define SWIPE_VERTICAL_START_AREA_Y1_W		(0x45)	/* @AP: 0xC6B */
#define SWIPE_VERTICAL_START_AREA_X2_W		(0x46)	/* @AP: 0xC6C */
#define SWIPE_VERTICAL_START_AREA_Y2_W		(0x47)	/* @AP: 0xC6D */

#define RUNTIME_DEBUG				(0x2A)   /* @AP: 0xC5A */
#define QCOVER_SENSITIVITY			(0x2f)	 /* @AP: 0xC5F */

#define SPR_CHARGER_STS				(0x30)	 /* @AP: 0xC60 */
#define REG_IME_STATE				(0x31)	 /* @AP: 0xC61 */

#define REG_CALL_STATE				(0x34)	 /* @AP: 0xC64 */

/* ABT STS SPI offset : r_abt_sts_spi_addr */
#define TCI_FAIL_DEBUG_R			(0xb)	/* @AP : 0x2ce */
#define TCI_FAIL_BIT_R				(0xc)	/* @AP : 0x2cf */
#define SWIPE_FAIL_DEBUG_R			(0x1b)	/* @AP : 0x2de */
#define TCI_DEBUG_R				(0x2c)	/* @AP : 0x2ef */
#define SWIPE_DEBUG_R				(0x35)	/* @AP : 0x2f8 */

#define R_HEADER_SIZE			(6)
#define W_HEADER_SIZE			(2)

#define SPI_RST_CTL			0xFE0
#define SPI_CLK_CTL			0xFE1
#define SPI_OSC_CTL			0xFE2

#define CONNECT_NONE			(0x00)
#define CONNECT_USB			(0x01)
#define CONNECT_TA			(0x02)
#define CONNECT_OTG			(0x03)
#define CONNECT_WIRELESS		(0x10)

#define CFG_MAGIC_CODE		0xCACACACA
#define CFG_CHIP_ID		49408
#define CFG_C_MAX_SIZE		2048
#define CFG_S_MAX_SIZE		4048

/* SPI Setting for IC, default: both i2c, spi on */
#define SPI_ERROR_ST			0x021
#define SERIAL_I2C_EN			0xFE5
#define SPI_TATTN_OPT			0xFF3
#define SPI_FAULT_TYPE			0xFF4

#define ABNORMAL_IC_DETECTION		(ATTN_ESD_EN | ATTN_WDOG_EN)

enum {
	SW_RESET = 0,
	HW_RESET,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	ABS_MODE = 0,
	KNOCK_1,
	KNOCK_2,
	SWIPE_RIGHT,
	SWIPE_LEFT,
	SWIPE_DOWN,
	SWIPE_UP,
	CUSTOM_DEBUG = 200,
	KNOCK_OVERTAP = 201,
};

enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

enum {
	SWIPE_R = 0,
	SWIPE_D,
	SWIPE_L,
	SWIPE_U
};

enum {
	SWIPE_RIGHT_BIT	= 1,
	SWIPE_DOWN_BIT	= 1 << 8,
	SWIPE_LEFT_BIT	= 1 << 16,
	SWIPE_UP_BIT	= 1 << 24,
};

enum {
	SWIPE_RIGHT_RELEASE_ENABLE = 1 << 1,
	SWIPE_DOWN_RELEASE_ENABLE = 1 << 9,
	SWIPE_LEFT_RELEASE_ENABLE = 1 << 17,
	SWIPE_UP_RELEASE_ENABLE = 1 << 25,
};

/* swipe */
enum {
	SWIPE_ENABLE_CTRL = 0,
	SWIPE_DISABLE_CTRL,
	SWIPE_DIST_CTRL,
	SWIPE_RATIO_THR_CTRL,
	SWIPE_RATIO_DIST_CTRL,
	SWIPE_RATIO_PERIOD_CTRL,
	SWIPE_TIME_MIN_CTRL,
	SWIPE_TIME_MAX_CTRL,
	SWIPE_AREA_CTRL,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	LOG_WRITE_DONE = 0,
	DO_WRITE_LOG,
};

enum {
	PROTOCOL_DISABLE = 0,
	PROTOCOL_ENABLE,
};

enum {
	ATTN_ESD_EN                 = (1U << 0),    /* 1 */
	ATTN_WDOG_EN                = (1U << 1),    /* 2 */
	ATTN_ABNORMAL_SPI_EN        = (1U << 2),    /* 4 */
};

enum {
	RUNTIME_DEBUG_DISABLE = 0,
	RUNTIME_DEBUG_ENABLE,
};

/* SPR control */

/* Firmware control */
#define spr_rst_ctl			(0x006)
#define spr_boot_ctl			(0x00F)
#define fw_boot_code_addr		(0x044)
#define spr_sram_ctl			(0x010)
#define spr_code_offset			(0x086)//(0x07D)
#define spr_data_offset			(0x08B)//(0x082)
#define rst_cnt_addr			(0xFF5)
//#define tc_flash_dn_sts			(0) // not used

#define code_access_addr		(0xFD0)
#define data_access_addr		(0xFD1)

#define MAX_RW_SIZE			(60 * 1024)
#define FLASH_FW_SIZE			(84 * 1024)
#define FLASH_SIZE			(128 * 1024)
#define FLASH_CONF_SIZE			(1 * 1024)

#define FLASH_KEY_CODE_CMD		0xDFC1
#define FLASH_KEY_CONF_CMD		0xE87B
#define FLASH_BOOTCHK_VALUE		0x0A0A0000
#define FW_BOOT_LOADER_INIT		0x74696E69
#define FW_BOOT_LOADER_CODE		0x544F4F42
#define FLASH_CODE_DNCHK_VALUE		0x42
#define FLASH_CONF_DNCHK_VALUE		0x8C

enum {
	E_FW_CODE_SIZE_ERR = 1,
	E_FW_CODE_ONLY_VALID = 2,
	E_FW_CODE_AND_CFG_VALID = 3,
	E_FW_CODE_CFG_ERR = 4
};

typedef union {
	struct {
		unsigned common_cfg_size : 16;
		unsigned specific_cfg_size : 16;
	} b;
	u32 w;
} t_cfg_size;

typedef struct {
	u32 cfg_magic_code;
	u32 cfg_info_reserved0;
	u32 cfg_chip_id;
	u32 cfg_struct_version;
	u32 cfg_specific_cnt;
	t_cfg_size cfg_size;
	u32 cfg_global_date;
	u32 cfg_global_time;
} t_cfg_info_def;

typedef union {
	struct {
		unsigned chip_rev : 8;
		unsigned model_id : 8;
		unsigned lcm_id : 8;
		unsigned fpcb_id : 8;
	} b;
	u32 w;
} t_cfg_specific_info1;

typedef union {
	struct {
		unsigned lot_id : 8;
		unsigned reserved : 24;
	} b;
	u32 w;
} t_cfg_specific_info2;

typedef struct {
	t_cfg_specific_info1 cfg_specific_info1;
	t_cfg_specific_info2 cfg_specific_info2;
	u32 cfg_specific_version;
	u32 cfg_model_name;
} t_cfg_s_header_def;

typedef struct {
	u32 cfg_common_ver;
} t_cfg_c_header_def;

struct sw49408_version {
	u8 build : 4;
	u8 major : 4;
	u8 minor;
};

struct sw49408_ic_info {
	struct sw49408_version version;
	u8 product_id[8];
	u8 image_version[2];
	u8 image_product_id[8];
	u8 revision;
	u32 lot;
	u32 lcm;
	u32 fpc;
	u32 chip_revision;
};

struct swipe_start_area {
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
};

struct swipe_info {
	u8	distance;
	u8	ratio_thres;
	u8	ratio_distance;
	u8	ratio_period;
	u16	min_time;
	u16	max_time;
	u32	fail_reason_on;
	struct active_area area;
	struct swipe_start_area start_area;
	u8	wrong_direction_thres;
};

struct swipe_ctrl {
	u32 mode;
	struct swipe_info info[4]; /* right, down, left, up */
};

struct sw49408_chip_info {
	u32 r_version;
	u32 r_product_code;
	u32 r_product_id1;
	u32 r_product_id2;
	u32 r_conf_dn_index;
	u32 r_conf_version;
};

struct sw49408_reg_info {
	u16	r_info_ptr_spi_addr;
	u16 r_tc_cmd_spi_addr;
	u16 r_watch_cmd_spi_addr;
	u16 r_test_cmd_spi_addr;
	u16 r_abt_cmd_spi_addr;
	u16 r_cfg_c_sram_oft;
	u16 r_cfg_s_sram_oft;
	u16 r_sys_buf_sram_oft;
	u16 r_abt_buf_sram_oft;
	u16 r_dbg_buf_sram_oft;
	u16 r_ic_status_spi_addr;
	u16 r_tc_status_spi_addr;
	u16 r_abt_report_spi_addr;
	u16 r_reserv_spi_addr;
	u16 r_chip_info_spi_addr;
	u16 r_reg_info_spi_addr;
	u16 r_pt_info_spi_addr;
	u16 r_tc_sts_spi_addr;
	u16 r_abt_sts_spi_addr;
	u16 r_tune_code_spi_addr;
	u16 r_aod_spi_addr;
};

typedef union {
	struct {
	unsigned r_goft_tune_m1:	4;
	unsigned r_goft_tune_m1_sign:	1;
	unsigned r_goft_tune_m2:	4;
	unsigned r_goft_tune_m2_sign:	1;
	unsigned r_goft_tune_nd:	5;
	unsigned reserved:		17;
	} b;
	u32 w;
} t_goft_tune;

#define MAX_CHANNEL				(32)
struct sw49408_tune_data {
	u32		r_tune_code_magic;

	t_goft_tune	r_goft_tune_u3_m1m2_left;
	u16		r_loft_tune_u3_m1_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u3_m2_g1_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u3_m2_g2_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u3_m2_g3_left[MAX_CHANNEL/2];

	t_goft_tune	r_goft_tune_u3_m1m2_right;
	u16		r_loft_tune_u3_m1_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u3_m2_g1_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u3_m2_g2_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u3_m2_g3_right[MAX_CHANNEL/2];

	t_goft_tune	r_goft_tune_u2_m1m2_left;
	u16		r_loft_tune_u2_m1_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u2_m2_g1_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u2_m2_g2_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u2_m2_g3_left[MAX_CHANNEL/2];

	t_goft_tune	r_goft_tune_u2_m1m2_right;
	u16		r_loft_tune_u2_m1_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u2_m2_g1_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u2_m2_g2_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u2_m2_g3_right[MAX_CHANNEL/2];

	t_goft_tune	r_goft_tune_u0_m1m2_left;
	u16		r_loft_tune_u0_m1_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u0_m2_g1_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u0_m2_g2_left[MAX_CHANNEL/2];
	u16		r_loft_tune_u0_m2_g3_left[MAX_CHANNEL/2];

	t_goft_tune	r_goft_tune_u0_m1m2_right;
	u16		r_loft_tune_u0_m1_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u0_m2_g1_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u0_m2_g2_right[MAX_CHANNEL/2];
	u16		r_loft_tune_u0_m2_g3_right[MAX_CHANNEL/2];
};

struct sw49408_data {
	struct device *dev;
	struct kobject kobj;

	struct sw49408_touch_info info;
	struct sw49408_ic_info ic_info;
	struct sw49408_chip_info chip_info;
	struct sw49408_reg_info reg_info;
	struct sw49408_asc_info asc;	/* ASC */
	struct workqueue_struct *wq_log;
	u8 lcd_mode;
	u8 prev_lcd_mode;
	u8 driving_mode;
	u8 u3fake;
//[Bringup]	struct watch_data watch;
	struct swipe_ctrl swipe;
	struct mutex spi_lock;
	struct delayed_work font_download_work;
	struct delayed_work fb_notify_work;
	struct delayed_work te_test_work;
	u32 charger;
	u32 earjack;
	u8 tci_debug_type;
	u32 swipe_debug_type;
//[Bringup]	atomic_t block_watch_cfg;
	atomic_t init;
	struct pm_qos_request pm_qos_req;
	u32 q_sensitivity;
	char te_test_log[64];
	int te_ret;
	u8 te_write_log;
	u8 tc_status_rst_cnt;
	u8 tc_status_fwup_cnt;
};

#define TCI_MAX_NUM				2
#define SWIPE_MAX_NUM				4
#define TCI_DEBUG_MAX_NUM			16
#define SWIPE_DEBUG_MAX_NUM			8
#define DISTANCE_INTER_TAP			(0x1 << 1) /* 2 */
#define DISTANCE_TOUCHSLOP			(0x1 << 2) /* 4 */
#define TIMEOUT_INTER_TAP_LONG			(0x1 << 3) /* 8 */
#define MULTI_FINGER				(0x1 << 4) /* 16 */
#define DELAY_TIME				(0x1 << 5) /* 32 */
#define TIMEOUT_INTER_TAP_SHORT			(0x1 << 6) /* 64 */
#define PALM_STATE				(0x1 << 7) /* 128 */
#define EDGE_SCRUBBED				(0x1 << 9) /* 512 */
#define LONG_TIME_TAP				(0x1 << 10) /* 1024 */
#define ABNORMAL_TOUCH				(0x1 << 11) /* 2048 */
#define TCI_DEBUG_ALL (DISTANCE_INTER_TAP | DISTANCE_TOUCHSLOP |\
	TIMEOUT_INTER_TAP_LONG | MULTI_FINGER | DELAY_TIME |\
	TIMEOUT_INTER_TAP_SHORT | PALM_STATE | EDGE_SCRUBBED |\
	LONG_TIME_TAP | ABNORMAL_TOUCH)

static inline struct sw49408_data *to_sw49408_data(struct device *dev)
{
	return (struct sw49408_data *)touch_get_device(to_touch_core(dev));
}

static inline struct sw49408_data *to_sw49408_data_from_kobj(
						struct kobject *kobj)
{
	return (struct sw49408_data *)container_of(kobj,
			struct sw49408_data, kobj);
}

int sw49408_reg_read(struct device *dev, u16 addr, void *data, int size);
int sw49408_reg_write(struct device *dev, u16 addr, void *data, int size);
int sw49408_ic_info(struct device *dev);
int sw49408_te_info(struct device *dev, char *buf);
int sw49408_tc_driving(struct device *dev, int mode);
int sw49408_irq_abs(struct device *dev);
int sw49408_irq_abs_data(struct device *dev);
int sw49408_irq_lpwg(struct device *dev);
void sw49408_irq_runtime_engine_debug(struct device *dev);
int sw49408_irq_handler(struct device *dev);
int sw49408_check_status(struct device *dev);
int sw49408_chip_info_load(struct device* dev);

static inline int sw49408_read_value(struct device *dev,
					u16 addr, u32 *value)
{
	return sw49408_reg_read(dev, addr, value, sizeof(*value));
}

static inline int sw49408_write_value(struct device *dev,
					 u16 addr, u32 value)
{
	return sw49408_reg_write(dev, addr, &value, sizeof(value));
}

//extern bool lge_panel_recovery_mode(void);
#endif /* LGE_TOUCH_SW49408_H */
