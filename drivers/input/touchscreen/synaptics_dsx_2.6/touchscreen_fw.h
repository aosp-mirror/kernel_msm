#ifndef 	__TOUCHSCREEN_FW__
#define 	__TOUCHSCREEN_FW__

#if defined(CONFIG_BOARD_PLATY)
#define SYN_ECW_FW_NAME		""
#else

#define SYN_TPK_FW_NAME		""
#define SYN_TURLY_FW_NAME	""
#define SYN_SUCCESS_FW_NAME	""
#define SYN_OFILM_FW_NAME	""
#define SYN_LEAD_FW_NAME	""
#define SYN_WINTEK_FW_NAME	""
#define SYN_LAIBAO_FW_NAME	""
#define SYN_CMI_FW_NAME		""
#define SYN_ECW_FW_NAME		""
#define SYN_GOWORLD_FW_NAME	""
#define SYN_BAOMING_FW_NAME	""
#define SYN_JUNDA_FW_NAME	""
#define SYN_JIAGUAN_FW_NAME	""
#define SYN_MUDONG_FW_NAME	""
#define SYN_EACHOPTO_FW_NAME	""
#define SYN_SAMSUNG_FW_NAME  ""

#define FTC_TPK_FW_NAME		""
#define FTC_TURLY_FW_NAME	""
#define FTC_SUCCESS_FW_NAME	""
#define FTC_OFILM_FW_NAME	""
#define FTC_LEAD_FW_NAME	""
#define FTC_WINTEK_FW_NAME	""
#define FTC_LAIBAO_FW_NAME	""
#define FTC_CMI_FW_NAME		""
#define FTC_ECW_FW_NAME		""
#define FTC_GOWORLD_FW_NAME	""
#define FTC_BAOMING_FW_NAME	""
#define FTC_JUNDA_FW_NAME	""
#define FTC_JIAGUAN_FW_NAME	""
#define FTC_MUDONG_FW_NAME	""
#define FTC_EACHOPTO_FW_NAME	""
#define FTC_AVC_FW_NAME	""

#define GTP_SENSOR_ID_1_FW_NAME		""
#define GTP_SENSOR_ID_2_FW_NAME		""
#define GTP_SENSOR_ID_3_FW_NAME		""
#define GTP_SENSOR_ID_4_FW_NAME		""
#define GTP_SENSOR_ID_5_FW_NAME		""
#define GTP_SENSOR_ID_6_FW_NAME		""

#define CY_TPK_FW_NAME		""
#define CY_TURLY_FW_NAME	""
#define CY_SUCCESS_FW_NAME	""
#define CY_OFILM_FW_NAME	""
#define CY_LEAD_FW_NAME	""
#define CY_WINTEK_FW_NAME	""
#define CY_LAIBAO_FW_NAME	""
#define CY_CMI_FW_NAME		""
#define CY_ECW_FW_NAME		""
#define CY_GOWORLD_FW_NAME	""
#define CY_BAOMING_FW_NAME	""
#define CY_JUNDA_FW_NAME	""
#define CY_JIAGUAN_FW_NAME	""
#define CY_MUDONG_FW_NAME	""
#define CY_EACHOPTO_FW_NAME	""

#endif

#ifndef SYN_TPK_FW_NAME
#define SYN_TPK_FW_NAME		""
#endif
#ifndef SYN_TURLY_FW_NAME
#define SYN_TURLY_FW_NAME	""
#endif
#ifndef SYN_SUCCESS_FW_NAME
#define SYN_SUCCESS_FW_NAME	""
#endif
#ifndef SYN_OFILM_FW_NAME
#define SYN_OFILM_FW_NAME	""
#endif
#ifndef SYN_LEAD_FW_NAME
#define SYN_LEAD_FW_NAME	""
#endif
#ifndef SYN_WINTEK_FW_NAME
#define SYN_WINTEK_FW_NAME	""
#endif
#ifndef SYN_LAIBAO_FW_NAME
#define SYN_LAIBAO_FW_NAME	""
#endif
#ifndef SYN_CMI_FW_NAME
#define SYN_CMI_FW_NAME		""
#endif
#ifndef SYN_ECW_FW_NAME
#define SYN_ECW_FW_NAME		""
#endif
#ifndef SYN_GOWORLD_FW_NAME
#define SYN_GOWORLD_FW_NAME	""
#endif
#ifndef SYN_BAOMING_FW_NAME
#define SYN_BAOMING_FW_NAME	""
#endif
#ifndef SYN_JUNDA_FW_NAME
#define SYN_JUNDA_FW_NAME	""
#endif
#ifndef SYN_JIAGUAN_FW_NAME
#define SYN_JIAGUAN_FW_NAME	""
#endif
#ifndef SYN_MUDONG_FW_NAME
#define SYN_MUDONG_FW_NAME	""
#endif
#ifndef SYN_EACHOPTO_FW_NAME
#define SYN_EACHOPTO_FW_NAME	""
#endif

#ifndef FTC_TPK_FW_NAME
#define FTC_TPK_FW_NAME		""
#endif
#ifndef FTC_TURLY_FW_NAME
#define FTC_TURLY_FW_NAME	""
#endif
#ifndef FTC_SUCCESS_FW_NAME
#define FTC_SUCCESS_FW_NAME	""
#endif
#ifndef FTC_OFILM_FW_NAME
#define FTC_OFILM_FW_NAME	""
#endif
#ifndef FTC_LEAD_FW_NAME
#define FTC_LEAD_FW_NAME	""
#endif
#ifndef FTC_WINTEK_FW_NAME
#define FTC_WINTEK_FW_NAME	""
#endif
#ifndef FTC_LAIBAO_FW_NAME
#define FTC_LAIBAO_FW_NAME	""
#endif
#ifndef FTC_CMI_FW_NAME
#define FTC_CMI_FW_NAME		""
#endif
#ifndef FTC_ECW_FW_NAME
#define FTC_ECW_FW_NAME		""
#endif
#ifndef FTC_GOWORLD_FW_NAME
#define FTC_GOWORLD_FW_NAME	""
#endif
#ifndef FTC_BAOMING_FW_NAME
#define FTC_BAOMING_FW_NAME	""
#endif
#ifndef FTC_JUNDA_FW_NAME
#define FTC_JUNDA_FW_NAME	""
#endif
#ifndef FTC_JIAGUAN_FW_NAME
#define FTC_JIAGUAN_FW_NAME	""
#endif
#ifndef FTC_MUDONG_FW_NAME
#define FTC_MUDONG_FW_NAME	""
#endif
#ifndef FTC_EACHOPTO_FW_NAME
#define FTC_EACHOPTO_FW_NAME	""
#endif

#ifndef FTC_AVC_FW_NAME
#define FTC_AVC_FW_NAME	""
#endif

enum TOUCH_MOUDLE
{
	TPK=0,
	TRULY,
	SUCCESS,
	OFILM,
	LEAD,
	WINTEK,
	LAIBAO,
	CMI,
	ECW,
	GOWORLD,
	BAOMING,
	JUNDA,
	JIAGUAN,
	MUDONG,
	EACHOPTO,
	AVC,
	SAMSUNG,
	UNKNOW=0xff
};

#define SYN_MOUDLE_NUM_MAX 15
#define FTC_MOUDLE_NUM_MAX 16
#define GTP_MOUDLE_NUM_MAX 6
#define CY_MOUDLE_NUM_MAX 15
#endif
