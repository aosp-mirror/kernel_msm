/*move hw_tp_common.h to here*/
/*Update from android kk to L version*/
/*Add for huawei TP*/
/*
 * Copyright (c) 2014 Huawei Device Company
 *
 * This file provide common requeirment for different touch IC.
 *
 * 2014-01-04:Add "tp_get_touch_screen_obj" by sunlibin
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __HW_TP_COMMON__
#define __HW_TP_COMMON__

//#define CONFIG_HUAWEI_KERNEL

/*IC type*/
#define IC_TYPE_3207 3207

#define FW_OFILM_STR   "000"
#define FW_EELY_STR    "001"
#define FW_TRULY_STR   "002"
#define FW_MUTTO_STR   "003"
#define FW_JUNDA_STR   "005"
#define FW_LENSONE_STR "006"
#define FW_YASSY_STR   "007"
#define FW_JDI_STR     "008"
#define FW_SAMSUNG_STR "009"
#define FW_LG_STR      "010"
#define FW_TIANMA_STR  "011"
#define FW_CMI_STR     "012"
#define FW_BOE_STR     "013"
#define FW_CTC_STR     "014"
#define FW_EDO_STR     "015"
#define FW_SHARP_STR   "016"

#define MODULE_STR_LEN 3
#define IC_TYPE_STR_LEN 2

/* buffer size for dsm tp client */
#define TP_RADAR_BUF_MAX	4096

enum f54_product_module_name {
	FW_OFILM		= 0,
	FW_EELY			= 1,
	FW_TRULY		= 2,
	FW_MUTTO		= 3,
	FW_GIS			= 4,
	FW_JUNDA		= 5,
	FW_LENSONE		= 6,
	FW_YASSY		= 7,
	FW_JDI			= 8,
	FW_SAMSUNG		= 9,
	FW_LG			= 10,
	FW_TIANMA		= 11,
	FW_CMI			= 12,
	FW_BOE			= 13,
	FW_CTC			= 14,
	FW_EDO			= 15,
	FW_SHARP		= 16,
	UNKNOW_PRODUCT_MODULE = 0xff,
};

struct holster_mode{
	unsigned long holster_enable;
	int top_left_x0;
	int top_left_y0;
	int bottom_right_x1;
	int bottom_right_y1;
};

struct kobject* tp_get_touch_screen_obj(void);
struct kobject* tp_get_virtual_key_obj(char *name);
struct kobject* tp_get_glove_func_obj(void);

/* add phone name so that a tp-driver can behave differentlly
accroding to different products*/
#define PHONE_NAME_Y550     "Y550"
#define PHONE_NAME_ULC02    "ULC02"
#define PHONE_NAME_RIO	    "Rio"
#define PHONE_NAME_Y538     "Y538"
#define PHONE_NAME_SCALE    "SCALE"
#define PHONE_NAME_KIWI     "Kiwi"
#define PHONE_NAME_ALICE    "Alice"

#define MMITEST

#define TP_ERR  1
#define TP_WARNING 2
#define TP_INFO 3
#define TP_DBG  4
#define TP_VDBG  5

/* define tp capacitance test type */
#define NORMALIZE_TP_CAPACITANCE_TEST "Normalize_type"
#define LEGACY_TP_CAPACITANCE_TEST    "legacy_type"

#define MMI_TEST_FAIL_REASON_SOFTWARE "software_reason"
#define MMI_TEST_FAIL_REASON_PANEL    "panel_reason"

#if defined(CONFIG_DEBUG_HUAWEI_FLOW_LOGLEVEL) && CONFIG_DEBUG_HUAWEI_FLOW_LOGLEVEL
extern int KERNEL_HWFLOW;
#else
#define KERNEL_HWFLOW 	1
#endif
extern int hw_tp_common_debug_mask;
#ifndef TP_LOG_NAME
#define TP_LOG_NAME "[COMMON]"
#endif
#ifndef tp_log_err
#define tp_log_err(x...)                \
do{                                     \
    if( hw_tp_common_debug_mask >= TP_ERR )   \
    {                                   \
	printk(KERN_ERR TP_LOG_NAME "[ERR]" x); 	\
    }                                   \
                                        \
}while(0)
#endif

#ifndef tp_log_warning
#define tp_log_warning(x...)               \
do{                                     \
    if( hw_tp_common_debug_mask >= TP_WARNING )  \
    {                                   \
	printk(KERN_ERR TP_LOG_NAME "[WARNING]" x); 	\
    }                                   \
                                        \
}while(0)
#endif

#ifndef tp_log_info
#define tp_log_info(x...)               \
do{                                     \
    if( (KERNEL_HWFLOW)  &&  (hw_tp_common_debug_mask >= TP_INFO))  \
    {                                   \
	printk(KERN_ERR TP_LOG_NAME "[INFO]" x); 	\
    }                                   \
                                        \
}while(0)
#endif

#ifndef tp_log_debug
#define tp_log_debug(x...)              \
do{                                     \
    if( (KERNEL_HWFLOW)  &&  (hw_tp_common_debug_mask >= TP_DBG) )   \
    {                                   \
	printk(KERN_ERR TP_LOG_NAME "[DBG]" x); 	\
    }                                   \
                                        \
}while(0)
#endif

#ifndef tp_log_vdebug
#define tp_log_vdebug(x...)              \
do{                                     \
    if( (KERNEL_HWFLOW)  && (hw_tp_common_debug_mask >= TP_VDBG) )   \
    {                                   \
	printk(KERN_ERR TP_LOG_NAME "[VDBG]" x); 	\
    }                                   \
                                        \
}while(0)
extern bool pt_test_enable_tp;
#endif

extern struct touch_hw_platform_data touch_hw_data;

struct touch_hw_platform_data
{
	int (*touch_power)(int on); /* Only valid in first array entry */
	int (*set_touch_interrupt_gpio)(void);/*it will config the gpio*/
	void (*set_touch_probe_flag)(int detected);/*we use this to detect the probe is detected*/
	int (*read_touch_probe_flag)(void);/*when the touch is find we return a value*/
	int (*touch_reset)(void);
	int (*get_touch_reset_gpio)(void);
	// int (*get_touch_resolution)(struct tp_resolution_conversion *tp_resolution_type);/*add this function for judge the tp type*/
	int (*read_button_flag)(void);
	// int (*get_touch_button_map)(struct tp_button_map *tp_button_map);
};

const char * get_module_name_by_id(unsigned int product_id);

enum touch_panel_data_type {
	RAW_DATA,
	DELTA_DATA,
};

#define REG_OPERATE_READ 0
#define REG_OPERATE_WRITE 1

struct touch_panel_reg_operate {
	int operate_type;		// read or write
	unsigned int addr;		// reg addr
	char* data;				// data to write or data have read
	int length;				// length of data
	int bit;				// reg bit
};

#define ENABLE_BIT(value, bit) ((value) | (1 << (bit)))
#define DISABLE_BIT(value, bit) ((value) & ~(1 << (bit)))
#define GET_BIT(value, bit)   (((value) >> (bit)) & 0x01)

//unsigned char already_has_tp_driver_running(void);

//void set_tp_driver_running(void);

// int get_tp_type(void);
// void set_tp_type(int type);

#endif
/* do not add code after this line !!!
 * add befor #endif !!!!
*/
