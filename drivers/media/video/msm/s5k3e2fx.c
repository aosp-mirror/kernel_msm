/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

static uint16_t g_usModuleVersion;	/*0: rev.4, 1: rev.5 */

/* prepare for modify PCLK*/
#define REG_PLL_MULTIPLIER_LSB_VALUE	  0x90
/* 0xA0 for PCLK=80MHz */
/* 0x90 for PCLK=72MHz */

/* prepare for modify initial gain*/
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB_VALUE	0x80

#define S5K3E2FX_REG_MODEL_ID   0x0000
#define S5K3E2FX_MODEL_ID       0x3E2F

#define S5K3E2FX_REG_MODULE_VER 0x0002

#define S5K3E2FX_DEF_MCLK		24000000

#define S5K3E2FX_QTR_SIZE_WIDTH		1296
#define S5K3E2FX_QTR_SIZE_HEIGHT	972

#define S5K3E2FX_FULL_SIZE_WIDTH	2608
#define S5K3E2FX_FULL_SIZE_HEIGHT	1960

/* AEC_FLASHING */
#define REG_GROUPED_PARAMETER_HOLD    0x0104
#define GROUPED_PARAMETER_HOLD        0x01
#define GROUPED_PARAMETER_UPDATE      0x00

/* Greenish in low light */
#define REG_MASK_CORRUPTED_FRAMES     0x0105
#define MASK                          0x01
#define NO_MASK                       0x00

/* PLL Registers */
#define REG_PRE_PLL_CLK_DIV           0x0305
#define REG_PLL_MULTIPLIER_MSB        0x0306
#define REG_PLL_MULTIPLIER_LSB        0x0307
#define REG_VT_PIX_CLK_DIV            0x0301
#define REG_VT_SYS_CLK_DIV            0x0303
#define REG_OP_PIX_CLK_DIV            0x0309
#define REG_OP_SYS_CLK_DIV            0x030B

/* Data Format Registers */
#define REG_CCP_DATA_FORMAT_MSB       0x0112
#define REG_CCP_DATA_FORMAT_LSB       0x0113

/* Output Size */
#define REG_X_OUTPUT_SIZE_MSB         0x034C
#define REG_X_OUTPUT_SIZE_LSB         0x034D
#define REG_Y_OUTPUT_SIZE_MSB         0x034E
#define REG_Y_OUTPUT_SIZE_LSB         0x034F

/* Binning */
#define REG_X_EVEN_INC                0x0381
#define REG_X_ODD_INC                 0x0383
#define REG_Y_EVEN_INC                0x0385
#define REG_Y_ODD_INC                 0x0387
/*Reserved register */
#define REG_BINNING_ENABLE            0x3014

/* Frame Fotmat */
#define REG_FRAME_LENGTH_LINES_MSB    0x0340
#define REG_FRAME_LENGTH_LINES_LSB    0x0341
#define REG_LINE_LENGTH_PCK_MSB       0x0342
#define REG_LINE_LENGTH_PCK_LSB       0x0343

/* MSR setting */
/* Reserved registers */
#define REG_SHADE_CLK_ENABLE          0x30AC
#define REG_SEL_CCP                   0x30C4
#define REG_VPIX                      0x3024
#define REG_CLAMP_ON                  0x3015
#define REG_OFFSET                    0x307E

/* CDS timing settings */
/* Reserved registers */
#define REG_LD_START                  0x3000
#define REG_LD_END                    0x3001
#define REG_SL_START                  0x3002
#define REG_SL_END                    0x3003
#define REG_RX_START                  0x3004
#define REG_S1_START                  0x3005
#define REG_S1_END                    0x3006
#define REG_S1S_START                 0x3007
#define REG_S1S_END                   0x3008
#define REG_S3_START                  0x3009
#define REG_S3_END                    0x300A
#define REG_CMP_EN_START              0x300B
#define REG_CLP_SL_START              0x300C
#define REG_CLP_SL_END                0x300D
#define REG_OFF_START                 0x300E
#define REG_RMP_EN_START              0x300F
#define REG_TX_START                  0x3010
#define REG_TX_END                    0x3011
#define REG_STX_WIDTH                 0x3012
#define REG_TYPE1_AF_ENABLE           0x3130
#define DRIVER_ENABLED                0x0001
#define AUTO_START_ENABLED            0x0010
#define REG_NEW_POSITION              0x3131
#define REG_3152_RESERVED             0x3152
#define REG_315A_RESERVED             0x315A
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB 0x0204
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB 0x0205
#define REG_FINE_INTEGRATION_TIME         0x0200
#define REG_COARSE_INTEGRATION_TIME       0x0202
#define REG_COARSE_INTEGRATION_TIME_LSB   0x0203

/* Mode select register */
#define S5K3E2FX_REG_MODE_SELECT        0x0100
#define S5K3E2FX_MODE_SELECT_STREAM     0x01	/* start streaming */
#define S5K3E2FX_MODE_SELECT_SW_STANDBY 0x00	/* software standby */
#define S5K3E2FX_REG_SOFTWARE_RESET     0x0103
#define S5K3E2FX_SOFTWARE_RESET         0x01
#define REG_TEST_PATTERN_MODE           0x0601

/* Samsung other MSR setting*/
#define REG_301D_RESERVED             0x301D
#define REG_3028_RESERVED             0x3028
#define REG_3070_RESERVED             0x3070
#define REG_3072_RESERVED             0x3072
#define REG_301B_RESERVED             0x301B
#define REG_30BD_RESERVED             0x30BD
#define REG_30C2_RESERVED             0x30C2
#define REG_3151_RESERVED             0x3151
#define REG_3029_RESERVED             0x3029
#define REG_30BF_RESERVED             0x30BF
#define REG_3022_RESERVED             0x3022
#define REG_3019_RESERVED             0x3019
#define REG_3150_RESERVED             0x3150
#define REG_3157_RESERVED             0x3157
#define REG_3159_RESERVED             0x3159
/* LC Preview/Snapshot difference register */
#define REG_SH4CH_BLK_WIDTH_R         0x309E
#define REG_SH4CH_BLK_HEIGHT_R        0x309F
#define REG_SH4CH_STEP_X_R_MSB        0x30A0
#define REG_SH4CH_STEP_X_R_LSB        0x30A1
#define REG_SH4CH_STEP_Y_R_MSB        0x30A2
#define REG_SH4CH_STEP_Y_R_LSB        0x30A3
#define REG_SH4CH_START_BLK_CNT_X_R   0x30A4
#define REG_SH4CH_START_BLK_INT_X_R   0x30A5
#define REG_SH4CH_START_FRAC_X_R_MSB  0x30A6
#define REG_SH4CH_START_FRAC_X_R_LSB  0x30A7
#define REG_SH4CH_START_BLK_CNT_Y_R   0x30A8
#define REG_SH4CH_START_BLK_INT_Y_R   0x30A9
#define REG_SH4CH_START_FRAC_Y_R_MSB  0x30AA
#define REG_SH4CH_START_FRAC_Y_R_LSB  0x30AB
#define REG_X_ADDR_START_MSB          0x0344
#define REG_X_ADDR_START_LSB          0x0345
#define REG_Y_ADDR_START_MSB          0x0346
#define REG_Y_ADDR_START_LSB          0x0347
#define REG_X_ADDR_END_MSB            0x0348
#define REG_X_ADDR_END_LSB            0x0349
#define REG_Y_ADDR_END_MSB            0x034A
#define REG_Y_ADDR_END_LSB            0x034B

#define NUM_INIT_REG                  94
#define NUM_LC_REG                    434

struct s5k3e2fx_i2c_reg_conf {
	unsigned short waddr;
	unsigned char bdata;
};

/* Separate the EVT4/EVT5 sensor init and LC setting start */
struct s5k3e2fx_i2c_reg_conf Init_setting[2][NUM_INIT_REG] = {
/*EVT4 */
	{
	 {REG_PRE_PLL_CLK_DIV, 0x06},	/* PLL setting */
	 {REG_PLL_MULTIPLIER_MSB, 0x00},
	 {REG_PLL_MULTIPLIER_LSB, REG_PLL_MULTIPLIER_LSB_VALUE},
	 {REG_VT_PIX_CLK_DIV, 0x08},
	 {REG_VT_SYS_CLK_DIV, 0x01},
	 {REG_OP_PIX_CLK_DIV, 0x08},
	 {REG_OP_SYS_CLK_DIV, 0x01},
/* Data Format */
	 {REG_CCP_DATA_FORMAT_MSB, 0x0a},
	 {REG_CCP_DATA_FORMAT_LSB, 0x0a},
/* Preview Output Size */
	 {REG_X_OUTPUT_SIZE_MSB, 0x05},
	 {REG_X_OUTPUT_SIZE_LSB, 0x10},
	 {REG_Y_OUTPUT_SIZE_MSB, 0x03},
	 {REG_Y_OUTPUT_SIZE_LSB, 0xcc},
	 {REG_X_ADDR_START_MSB, 0x00},
	 {REG_X_ADDR_START_LSB, 0x08},
	 {REG_Y_ADDR_START_MSB, 0x00},
	 {REG_Y_ADDR_START_LSB, 0x08},
	 {REG_X_ADDR_END_MSB, 0x0a},
	 {REG_X_ADDR_END_LSB, 0x27},
	 {REG_Y_ADDR_END_MSB, 0x07},
	 {REG_Y_ADDR_END_LSB, 0x9f},
/* Frame format */
	 {REG_FRAME_LENGTH_LINES_MSB, 0x03},
	 {REG_FRAME_LENGTH_LINES_LSB, 0xe2},
	 {REG_LINE_LENGTH_PCK_MSB, 0x0a},
	 {REG_LINE_LENGTH_PCK_LSB, 0xac},
/* Preview Binning */
	 {REG_X_EVEN_INC, 0x01},
	 {REG_X_ODD_INC, 0x01},
	 {REG_Y_EVEN_INC, 0x01},
	 {REG_Y_ODD_INC, 0x03},
	 {REG_BINNING_ENABLE, 0x06},
/* Samsung MSR Setting */
	 {REG_SEL_CCP, 0x01},
	 {REG_LD_START, 0x03},
/* Add EVT5 sensor Samsung MSR setting, Start */
	 {REG_LD_END, 0x94},
	 {REG_SL_START, 0x02},
	 {REG_SL_END, 0x95},
	 {REG_RX_START, 0x0f},
	 {REG_S1_START, 0x05},
	 {REG_S1_END, 0x3c},
	 {REG_S1S_START, 0x8c},
	 {REG_S1S_END, 0x93},
	 {REG_S3_START, 0x05},
	 {REG_S3_END, 0x3a},
	 {REG_CMP_EN_START, 0x10},
	 {REG_CLP_SL_START, 0x02},
	 {REG_CLP_SL_END, 0x3e},
	 {REG_OFF_START, 0x02},
	 {REG_RMP_EN_START, 0x0e},
	 {REG_TX_START, 0x46},
	 {REG_TX_END, 0x64},
	 {REG_STX_WIDTH, 0x1e},
	 {REG_CLAMP_ON, 0x00},
	 {REG_301D_RESERVED, 0x3f},
	 {REG_VPIX, 0x04},
	 {REG_3028_RESERVED, 0x40},
	 {REG_3070_RESERVED, 0xdf},
	 {REG_3072_RESERVED, 0x20},
	 {REG_301B_RESERVED, 0x73},
	 {REG_OFFSET, 0x02},
	 {REG_30BD_RESERVED, 0x06},
	 {REG_30C2_RESERVED, 0x0b},
	 {REG_SHADE_CLK_ENABLE, 0x81},
	 {REG_3151_RESERVED, 0xe6},
	 {REG_3029_RESERVED, 0x02},
	 {REG_30BF_RESERVED, 0x00},
	 {REG_3022_RESERVED, 0x87},
	 {REG_3019_RESERVED, 0x60},
	 {REG_3019_RESERVED, 0x60},
	 {REG_3019_RESERVED, 0x60},
	 {REG_3019_RESERVED, 0x60},
	 {REG_3019_RESERVED, 0x60},
	 {REG_3019_RESERVED, 0x60},
	 {REG_3152_RESERVED, 0x08},
	 {REG_3150_RESERVED, 0x50}, /* from 0x40 to 0x50 for PCLK=80MHz */
/* Inverse PCLK = 0x50 */
	 {REG_3157_RESERVED, 0x04}, /* from 0x00 to 0x04 for PCLK=80MHz */
/* PCLK Delay offset; 0x0a will delay around 4ns at 80MHz */
	 {REG_3159_RESERVED, 0x0f}, /* from 0x00 to 0x0f for PCLK=80MHz */
/* HS, VS driving strength [3:2]=>VS, [1:0]=>HS 00:2mA, 01:4mA, 10:6mA,
 * 11:8mA
 */
	 {REG_315A_RESERVED, 0xf0}, /* from 0x10 to 0xf0 for PCLK=80MHz */
/* PCLK, DATA driving strength [7:6]=>data, [5:4]=>PCLK 00:2mA, 01:4mA
 * 10:6mA, 11:8mA
 */
/* AEC Setting */
	 {REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, 0x00},
	 {REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB_VALUE},
	 {REG_FINE_INTEGRATION_TIME, 0x02},
	 {REG_COARSE_INTEGRATION_TIME, 0x03},
/* Preview LC config Setting */
	 {REG_SH4CH_BLK_WIDTH_R, 0x52},
	 {REG_SH4CH_BLK_HEIGHT_R, 0x3e},
	 {REG_SH4CH_STEP_X_R_MSB, 0x03},
	 {REG_SH4CH_STEP_X_R_LSB, 0x1f},
	 {REG_SH4CH_STEP_Y_R_MSB, 0x04},
	 {REG_SH4CH_STEP_Y_R_LSB, 0x21},
	 {REG_SH4CH_START_BLK_CNT_X_R, 0x04},
	 {REG_SH4CH_START_BLK_INT_X_R, 0x00},
	 {REG_SH4CH_START_FRAC_X_R_MSB, 0x0c},
	 {REG_SH4CH_START_FRAC_X_R_LSB, 0x7c},
	 {REG_SH4CH_START_BLK_CNT_Y_R, 0x04},
	 {REG_SH4CH_START_BLK_INT_Y_R, 0x00},
	 {REG_SH4CH_START_FRAC_Y_R_MSB, 0x10},
	 {REG_SH4CH_START_FRAC_Y_R_LSB, 0x84},
	 },

/* EVT5 */
	{
	 {REG_PRE_PLL_CLK_DIV, 0x06}, /* PLL setting */
	 {REG_PLL_MULTIPLIER_MSB, 0x00},
	 {REG_PLL_MULTIPLIER_LSB, REG_PLL_MULTIPLIER_LSB_VALUE},
	 {REG_VT_PIX_CLK_DIV, 0x08},
	 {REG_VT_SYS_CLK_DIV, 0x01},
	 {REG_OP_PIX_CLK_DIV, 0x08},
	 {REG_OP_SYS_CLK_DIV, 0x01},
/* Data Format */
	 {REG_CCP_DATA_FORMAT_MSB, 0x0a},
	 {REG_CCP_DATA_FORMAT_LSB, 0x0a},
/* Preview Output Size */
	 {REG_X_OUTPUT_SIZE_MSB, 0x05},
	 {REG_X_OUTPUT_SIZE_LSB, 0x10},
	 {REG_Y_OUTPUT_SIZE_MSB, 0x03},
	 {REG_Y_OUTPUT_SIZE_LSB, 0xcc},
	 {REG_X_ADDR_START_MSB, 0x00},
	 {REG_X_ADDR_START_LSB, 0x08},
	 {REG_Y_ADDR_START_MSB, 0x00},
	 {REG_Y_ADDR_START_LSB, 0x08},
	 {REG_X_ADDR_END_MSB, 0x0a},
	 {REG_X_ADDR_END_LSB, 0x27},
	 {REG_Y_ADDR_END_MSB, 0x07},
	 {REG_Y_ADDR_END_LSB, 0x9f},
/* Frame format */
	 {REG_FRAME_LENGTH_LINES_MSB, 0x03},
	 {REG_FRAME_LENGTH_LINES_LSB, 0xe2},
	 {REG_LINE_LENGTH_PCK_MSB, 0x0a},
	 {REG_LINE_LENGTH_PCK_LSB, 0xac},
/* Preview Binning */
	 {REG_X_EVEN_INC, 0x01},
	 {REG_X_ODD_INC, 0x01},
	 {REG_Y_EVEN_INC, 0x01},
	 {REG_Y_ODD_INC, 0x03},
	 {REG_BINNING_ENABLE, 0x06},
/* Samsung MSR Setting */
	 {REG_SEL_CCP, 0x01},
	 {REG_LD_START, 0x03},
/* EVT5 sensor Samsung MSR setting */
	 {REG_LD_END, 0x99},
	 {REG_SL_START, 0x02},
	 {REG_SL_END, 0x9A},
	 {REG_RX_START, 0x0f},
	 {REG_S1_START, 0x05},
	 {REG_S1_END, 0x3c},
	 {REG_S1S_START, 0x8c},
	 {REG_S1S_END, 0x26},
	 {REG_S3_START, 0x05},
	 {REG_S3_END, 0x3a},
	 {REG_CMP_EN_START, 0x10},
	 {REG_CLP_SL_START, 0x02},
	 {REG_CLP_SL_END, 0x3e},
	 {REG_OFF_START, 0x02},
	 {REG_RMP_EN_START, 0x0e},
	 {REG_TX_START, 0x46},
	 {REG_TX_END, 0x64},
	 {REG_STX_WIDTH, 0x1e},
	 {REG_CLAMP_ON, 0x00},
	 {REG_301D_RESERVED, 0x3f},
	 {REG_VPIX, 0x04},
	 {REG_3028_RESERVED, 0x40},
	 {REG_3070_RESERVED, 0xdf},
	 {REG_3072_RESERVED, 0x20},
	 {REG_301B_RESERVED, 0x73},
	 {REG_OFFSET, 0x02},
	 {REG_30BD_RESERVED, 0x06},
	 {REG_30C2_RESERVED, 0x0b},
	 {REG_SHADE_CLK_ENABLE, 0x81},
	 {REG_3151_RESERVED, 0xe6},
	 {REG_3029_RESERVED, 0x02},
	 {REG_30BF_RESERVED, 0x00},
	 {REG_3022_RESERVED, 0x87},
	 {REG_3019_RESERVED, 0x60},
	 {0x3060, 0x03},
	 {0x3061, 0x6C},
	 {0x3062, 0x00},
	 {0x3063, 0xD6},
	 {0x3023, 0x0C},
	 {REG_3152_RESERVED, 0x08},
	 {REG_3150_RESERVED, 0x50}, /* from 0x40 to 0x50 for PCLK=80MHz */
/* Inverse PCLK = 0x50 */
	 {REG_3157_RESERVED, 0x04}, /* from 0x00 to 0x04 for PCLK=80MHz */
/* PCLK Delay offset; 0x0a will delay around 4ns at 80MHz */
	 {REG_3159_RESERVED, 0x0f}, /* from 0x00 to 0x0f for PCLK=80MHz */
/* HS, VS driving strength [3:2]=>VS, [1:0]=>HS 00:2mA, 01:4mA, 10:6mA,
 * 11:8mA
 */
	 {REG_315A_RESERVED, 0xf0}, /* from 0x10 to 0xf0 for PCLK=80MHz */
/* PCLK, DATA driving strength [7:6]=>data, [5:4]=>PCLK 00:2mA, 01:4mA,
 * 10:6mA, 11:8mA
 */
/* AEC Setting */
	 {REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB, 0x00},
	 {REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB, REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB_VALUE},
	 {REG_FINE_INTEGRATION_TIME, 0x02},
	 {REG_COARSE_INTEGRATION_TIME, 0x03},
/* Preview LC config Setting */
	 {REG_SH4CH_BLK_WIDTH_R, 0x52},
	 {REG_SH4CH_BLK_HEIGHT_R, 0x3e},
	 {REG_SH4CH_STEP_X_R_MSB, 0x03},
	 {REG_SH4CH_STEP_X_R_LSB, 0x1f},
	 {REG_SH4CH_STEP_Y_R_MSB, 0x04},
	 {REG_SH4CH_STEP_Y_R_LSB, 0x21},
	 {REG_SH4CH_START_BLK_CNT_X_R, 0x04},
	 {REG_SH4CH_START_BLK_INT_X_R, 0x00},
	 {REG_SH4CH_START_FRAC_X_R_MSB, 0x0c},
	 {REG_SH4CH_START_FRAC_X_R_LSB, 0x7c},
	 {REG_SH4CH_START_BLK_CNT_Y_R, 0x04},
	 {REG_SH4CH_START_BLK_INT_Y_R, 0x00},
	 {REG_SH4CH_START_FRAC_Y_R_MSB, 0x10},
	 {REG_SH4CH_START_FRAC_Y_R_LSB, 0x84},
	 }
};

struct s5k3e2fx_i2c_reg_conf lc_setting[2][NUM_LC_REG] = {
/*EVT4 */
	{
	/*EVT4 */  /* 100108 Modify LC setting DNP light source t75-r73*/
	{0x3200, 0x00},
	{0x3201, 0x99},
	{0x3202, 0xc1},
	{0x3203, 0x0f},
	{0x3204, 0xd0},
	{0x3205, 0x1b},
	{0x3206, 0x00},
	{0x3207, 0x24},
	{0x3208, 0x8d},
	{0x3209, 0x0f},
	{0x320a, 0xee},
	{0x320b, 0x0f},
	{0x320c, 0x00},
	{0x320d, 0x04},
	{0x320e, 0x5c},
	{0x320f, 0x00},
	{0x3210, 0x07},
	{0x3211, 0x68},
	{0x3212, 0x0f},
	{0x3213, 0xc2},
	{0x3214, 0x82},
	{0x3215, 0x00},
	{0x3216, 0x29},
	{0x3217, 0x3e},
	{0x3218, 0x0f},
	{0x3219, 0xd3},
	{0x321a, 0x63},
	{0x321b, 0x00},
	{0x321c, 0x22},
	{0x321d, 0x6c},
	{0x321e, 0x0f},
	{0x321f, 0xf8},
	{0x3220, 0xce},
	{0x3221, 0x0f},
	{0x3222, 0xed},
	{0x3223, 0x30},
	{0x3224, 0x00},
	{0x3225, 0x37},
	{0x3226, 0x87},
	{0x3227, 0x0f},
	{0x3228, 0xc2},
	{0x3229, 0x87},
	{0x322a, 0x00},
	{0x322b, 0x2a},
	{0x322c, 0xc6},
	{0x322d, 0x0f},
	{0x322e, 0xf3},
	{0x322f, 0xd9},
	{0x3230, 0x0f},
	{0x3231, 0xea},
	{0x3232, 0x1a},
	{0x3233, 0x00},
	{0x3234, 0x2d},
	{0x3235, 0x9f},
	{0x3236, 0x0f},
	{0x3237, 0xde},
	{0x3238, 0x7d},
	{0x3239, 0x00},
	{0x323a, 0x37},
	{0x323b, 0x1e},
	{0x323c, 0x0f},
	{0x323d, 0xed},
	{0x323e, 0x9c},
	{0x323f, 0x0f},
	{0x3240, 0xf6},
	{0x3241, 0xfd},
	{0x3242, 0x00},
	{0x3243, 0x15},
	{0x3244, 0xeb},
	{0x3245, 0x0f},
	{0x3246, 0xd3},
	{0x3247, 0xca},
	{0x3248, 0x00},
	{0x3249, 0x08},
	{0x324a, 0xe6},
	{0x324b, 0x0f},
	{0x324c, 0xf4},
	{0x324d, 0x7a},
	{0x324e, 0x0f},
	{0x324f, 0xed},
	{0x3250, 0x1e},
	{0x3251, 0x00},
	{0x3252, 0x0d},
	{0x3253, 0x46},
	{0x3254, 0x00},
	{0x3255, 0x0c},
	{0x3256, 0x3e},
	{0x3257, 0x00},
	{0x3258, 0x09},
	{0x3259, 0xcf},
	{0x325a, 0x00},
	{0x325b, 0x09},
	{0x325c, 0xb5},
	{0x325d, 0x0f},
	{0x325e, 0xec},
	{0x325f, 0x47},
	{0x3260, 0x00},
	{0x3261, 0x1d},
	{0x3262, 0xd8},
	{0x3263, 0x0f},
	{0x3264, 0xf7},
	{0x3265, 0x11},
	{0x3266, 0x0f},
	{0x3267, 0xea},
	{0x3268, 0x3d},
	{0x3269, 0x00},
	{0x326a, 0x09},
	{0x326b, 0xcc},
	{0x326c, 0x00},
	{0x326d, 0x9b},
	{0x326e, 0x73},
	{0x326f, 0x0f},
	{0x3270, 0xd4},
	{0x3271, 0x9e},
	{0x3272, 0x00},
	{0x3273, 0x1a},
	{0x3274, 0x87},
	{0x3275, 0x0f},
	{0x3276, 0xfd},
	{0x3277, 0xeb},
	{0x3278, 0x0f},
	{0x3279, 0xf5},
	{0x327a, 0xb4},
	{0x327b, 0x00},
	{0x327c, 0x0d},
	{0x327d, 0x8c},
	{0x327e, 0x0f},
	{0x327f, 0xc9},
	{0x3280, 0x4d},
	{0x3281, 0x00},
	{0x3282, 0x1d},
	{0x3283, 0x2d},
	{0x3284, 0x0f},
	{0x3285, 0xea},
	{0x3286, 0x5b},
	{0x3287, 0x00},
	{0x3288, 0x04},
	{0x3289, 0x76},
	{0x328a, 0x00},
	{0x328b, 0x10},
	{0x328c, 0x2d},
	{0x328d, 0x0f},
	{0x328e, 0xe6},
	{0x328f, 0xde},
	{0x3290, 0x00},
	{0x3291, 0x26},
	{0x3292, 0x85},
	{0x3293, 0x0f},
	{0x3294, 0xcf},
	{0x3295, 0x12},
	{0x3296, 0x00},
	{0x3297, 0x14},
	{0x3298, 0x0f},
	{0x3299, 0x00},
	{0x329a, 0x0b},
	{0x329b, 0x36},
	{0x329c, 0x0f},
	{0x329d, 0xe4},
	{0x329e, 0xa4},
	{0x329f, 0x00},
	{0x32a0, 0x21},
	{0x32a1, 0x1f},
	{0x32a2, 0x0f},
	{0x32a3, 0xf3},
	{0x32a4, 0x99},
	{0x32a5, 0x00},
	{0x32a6, 0x30},
	{0x32a7, 0x8f},
	{0x32a8, 0x0f},
	{0x32a9, 0xf9},
	{0x32aa, 0x35},
	{0x32ab, 0x0f},
	{0x32ac, 0xee},
	{0x32ad, 0x6e},
	{0x32ae, 0x00},
	{0x32af, 0x09},
	{0x32b0, 0x19},
	{0x32b1, 0x0f},
	{0x32b2, 0xf0},
	{0x32b3, 0x57},
	{0x32b4, 0x00},
	{0x32b5, 0x01},
	{0x32b6, 0xcc},
	{0x32b7, 0x0f},
	{0x32b8, 0xf1},
	{0x32b9, 0x0b},
	{0x32ba, 0x0f},
	{0x32bb, 0xee},
	{0x32bc, 0x99},
	{0x32bd, 0x00},
	{0x32be, 0x11},
	{0x32bf, 0x3d},
	{0x32c0, 0x00},
	{0x32c1, 0x10},
	{0x32c2, 0x64},
	{0x32c3, 0x0f},
	{0x32c4, 0xf6},
	{0x32c5, 0xab},
	{0x32c6, 0x00},
	{0x32c7, 0x03},
	{0x32c8, 0x19},
	{0x32c9, 0x0f},
	{0x32ca, 0xf3},
	{0x32cb, 0xc9},
	{0x32cc, 0x00},
	{0x32cd, 0x17},
	{0x32ce, 0xb3},
	{0x32cf, 0x0f},
	{0x32d0, 0xf2},
	{0x32d1, 0x3d},
	{0x32d2, 0x0f},
	{0x32d3, 0xf4},
	{0x32d4, 0x7e},
	{0x32d5, 0x00},
	{0x32d6, 0x09},
	{0x32d7, 0x46},
	{0x32d8, 0x00},
	{0x32d9, 0x7c},
	{0x32da, 0x79},
	{0x32db, 0x0f},
	{0x32dc, 0xde},
	{0x32dd, 0x19},
	{0x32de, 0x00},
	{0x32df, 0x19},
	{0x32e0, 0xe8},
	{0x32e1, 0x0f},
	{0x32e2, 0xf3},
	{0x32e3, 0x41},
	{0x32e4, 0x00},
	{0x32e5, 0x03},
	{0x32e6, 0x4c},
	{0x32e7, 0x00},
	{0x32e8, 0x05},
	{0x32e9, 0x73},
	{0x32ea, 0x0f},
	{0x32eb, 0xd6},
	{0x32ec, 0xa5},
	{0x32ed, 0x00},
	{0x32ee, 0x1f},
	{0x32ef, 0x81},
	{0x32f0, 0x0f},
	{0x32f1, 0xdc},
	{0x32f2, 0xe6},
	{0x32f3, 0x00},
	{0x32f4, 0x18},
	{0x32f5, 0x65},
	{0x32f6, 0x00},
	{0x32f7, 0x00},
	{0x32f8, 0x11},
	{0x32f9, 0x0f},
	{0x32fa, 0xed},
	{0x32fb, 0x65},
	{0x32fc, 0x00},
	{0x32fd, 0x23},
	{0x32fe, 0x12},
	{0x32ff, 0x0f},
	{0x3300, 0xcf},
	{0x3301, 0x28},
	{0x3302, 0x00},
	{0x3303, 0x2b},
	{0x3304, 0xda},
	{0x3305, 0x0f},
	{0x3306, 0xef},
	{0x3307, 0xae},
	{0x3308, 0x0f},
	{0x3309, 0xeb},
	{0x330a, 0x13},
	{0x330b, 0x00},
	{0x330c, 0x27},
	{0x330d, 0xb8},
	{0x330e, 0x0f},
	{0x330f, 0xec},
	{0x3310, 0x69},
	{0x3311, 0x00},
	{0x3312, 0x2f},
	{0x3313, 0x5f},
	{0x3314, 0x0f},
	{0x3315, 0xdf},
	{0x3316, 0x4f},
	{0x3317, 0x00},
	{0x3318, 0x05},
	{0x3319, 0x70},
	{0x331a, 0x00},
	{0x331b, 0x0f},
	{0x331c, 0xd2},
	{0x331d, 0x0f},
	{0x331e, 0xe1},
	{0x331f, 0xd8},
	{0x3320, 0x00},
	{0x3321, 0x09},
	{0x3322, 0xcf},
	{0x3323, 0x0f},
	{0x3324, 0xf2},
	{0x3325, 0x6e},
	{0x3326, 0x0f},
	{0x3327, 0xf6},
	{0x3328, 0xb4},
	{0x3329, 0x00},
	{0x332a, 0x0d},
	{0x332b, 0x87},
	{0x332c, 0x00},
	{0x332d, 0x08},
	{0x332e, 0x1e},
	{0x332f, 0x0f},
	{0x3330, 0xfa},
	{0x3331, 0x6e},
	{0x3332, 0x0f},
	{0x3333, 0xff},
	{0x3334, 0xaa},
	{0x3335, 0x0f},
	{0x3336, 0xf2},
	{0x3337, 0xc0},
	{0x3338, 0x00},
	{0x3339, 0x1d},
	{0x333a, 0x18},
	{0x333b, 0x0f},
	{0x333c, 0xef},
	{0x333d, 0xed},
	{0x333e, 0x0f},
	{0x333f, 0xec},
	{0x3340, 0xf6},
	{0x3341, 0x00},
	{0x3342, 0x16},
	{0x3343, 0x8e},
	{0x3344, 0x00},
	{0x3345, 0x9c},
	{0x3346, 0x52},
	{0x3347, 0x0f},
	{0x3348, 0xcf},
	{0x3349, 0xb9},
	{0x334a, 0x00},
	{0x334b, 0x29},
	{0x334c, 0xe9},
	{0x334d, 0x0f},
	{0x334e, 0xe2},
	{0x334f, 0x83},
	{0x3350, 0x00},
	{0x3351, 0x11},
	{0x3352, 0xcc},
	{0x3353, 0x0f},
	{0x3354, 0xff},
	{0x3355, 0xf4},
	{0x3356, 0x0f},
	{0x3357, 0xc1},
	{0x3358, 0xa4},
	{0x3359, 0x00},
	{0x335a, 0x2f},
	{0x335b, 0xce},
	{0x335c, 0x0f},
	{0x335d, 0xc5},
	{0x335e, 0xbb},
	{0x335f, 0x00},
	{0x3360, 0x35},
	{0x3361, 0x2a},
	{0x3362, 0x0f},
	{0x3363, 0xe6},
	{0x3364, 0x2a},
	{0x3365, 0x0f},
	{0x3366, 0xf7},
	{0x3367, 0x44},
	{0x3368, 0x00},
	{0x3369, 0x31},
	{0x336a, 0xfe},
	{0x336b, 0x0f},
	{0x336c, 0xb6},
	{0x336d, 0x84},
	{0x336e, 0x00},
	{0x336f, 0x3c},
	{0x3370, 0x71},
	{0x3371, 0x0f},
	{0x3372, 0xe5},
	{0x3373, 0xfe},
	{0x3374, 0x0f},
	{0x3375, 0xf2},
	{0x3376, 0x87},
	{0x3377, 0x00},
	{0x3378, 0x29},
	{0x3379, 0x2b},
	{0x337a, 0x0f},
	{0x337b, 0xe5},
	{0x337c, 0x3f},
	{0x337d, 0x00},
	{0x337e, 0x45},
	{0x337f, 0xc6},
	{0x3380, 0x0f},
	{0x3381, 0xdf},
	{0x3382, 0xe6},
	{0x3383, 0x0f},
	{0x3384, 0xfb},
	{0x3385, 0x0f},
	{0x3386, 0x00},
	{0x3387, 0x0f},
	{0x3388, 0xf4},
	{0x3389, 0x0f},
	{0x338a, 0xdf},
	{0x338b, 0x72},
	{0x338c, 0x00},
	{0x338d, 0x0e},
	{0x338e, 0xaf},
	{0x338f, 0x0f},
	{0x3390, 0xed},
	{0x3391, 0x7a},
	{0x3392, 0x0f},
	{0x3393, 0xe5},
	{0x3394, 0xab},
	{0x3395, 0x00},
	{0x3396, 0x18},
	{0x3397, 0x43},
	{0x3398, 0x00},
	{0x3399, 0x1b},
	{0x339a, 0x41},
	{0x339b, 0x0f},
	{0x339c, 0xea},
	{0x339d, 0x84},
	{0x339e, 0x0f},
	{0x339f, 0xfd},
	{0x33a0, 0xdb},
	{0x33a1, 0x0f},
	{0x33a2, 0xe9},
	{0x33a3, 0xbd},
	{0x33a4, 0x00},
	{0x33a5, 0x30},
	{0x33a6, 0x77},
	{0x33a7, 0x0f},
	{0x33a8, 0xe9},
	{0x33a9, 0x93},
	{0x33aa, 0x0f},
	{0x33ab, 0xd7},
	{0x33ac, 0xde},
	{0x33ad, 0x00},
	{0x33ae, 0x2a},
	{0x33af, 0x14},
	{0x309D, 0x62},
	{0x309d, 0x22},

/* LC setting End */
	 },
/*EVT5 */
	{
/* LC setting Start */
	 {0x3200, 0x00}, /* 100108 Modify LC setting DNP light source t75-r73*/
	 {0x3201, 0x99},
	 {0x3202, 0xc1},
	 {0x3203, 0x0f},
	 {0x3204, 0xd0},
	 {0x3205, 0x1b},
	 {0x3206, 0x00},
	 {0x3207, 0x24},
	 {0x3208, 0x8d},
	 {0x3209, 0x0f},
	 {0x320a, 0xee},
	 {0x320b, 0x0f},
	 {0x320c, 0x00},
	 {0x320d, 0x04},
	 {0x320e, 0x5c},
	 {0x320f, 0x00},
	 {0x3210, 0x07},
	 {0x3211, 0x68},
	 {0x3212, 0x0f},
	 {0x3213, 0xc2},
	 {0x3214, 0x82},
	 {0x3215, 0x00},
	 {0x3216, 0x29},
	 {0x3217, 0x3e},
	 {0x3218, 0x0f},
	 {0x3219, 0xd3},
	 {0x321a, 0x63},
	 {0x321b, 0x00},
	 {0x321c, 0x22},
	 {0x321d, 0x6c},
	 {0x321e, 0x0f},
	 {0x321f, 0xf8},
	 {0x3220, 0xce},
	 {0x3221, 0x0f},
	 {0x3222, 0xed},
	 {0x3223, 0x30},
	 {0x3224, 0x00},
	 {0x3225, 0x37},
	 {0x3226, 0x87},
	 {0x3227, 0x0f},
	 {0x3228, 0xc2},
	 {0x3229, 0x87},
	 {0x322a, 0x00},
	 {0x322b, 0x2a},
	 {0x322c, 0xc6},
	 {0x322d, 0x0f},
	 {0x322e, 0xf3},
	 {0x322f, 0xd9},
	 {0x3230, 0x0f},
	 {0x3231, 0xea},
	 {0x3232, 0x1a},
	 {0x3233, 0x00},
	 {0x3234, 0x2d},
	 {0x3235, 0x9f},
	 {0x3236, 0x0f},
	 {0x3237, 0xde},
	 {0x3238, 0x7d},
	 {0x3239, 0x00},
	 {0x323a, 0x37},
	 {0x323b, 0x1e},
	 {0x323c, 0x0f},
	 {0x323d, 0xed},
	 {0x323e, 0x9c},
	 {0x323f, 0x0f},
	 {0x3240, 0xf6},
	 {0x3241, 0xfd},
	 {0x3242, 0x00},
	 {0x3243, 0x15},
	 {0x3244, 0xeb},
	 {0x3245, 0x0f},
	 {0x3246, 0xd3},
	 {0x3247, 0xca},
	 {0x3248, 0x00},
	 {0x3249, 0x08},
	 {0x324a, 0xe6},
	 {0x324b, 0x0f},
	 {0x324c, 0xf4},
	 {0x324d, 0x7a},
	 {0x324e, 0x0f},
	 {0x324f, 0xed},
	 {0x3250, 0x1e},
	 {0x3251, 0x00},
	 {0x3252, 0x0d},
	 {0x3253, 0x46},
	 {0x3254, 0x00},
	 {0x3255, 0x0c},
	 {0x3256, 0x3e},
	 {0x3257, 0x00},
	 {0x3258, 0x09},
	 {0x3259, 0xcf},
	 {0x325a, 0x00},
	 {0x325b, 0x09},
	 {0x325c, 0xb5},
	 {0x325d, 0x0f},
	 {0x325e, 0xec},
	 {0x325f, 0x47},
	 {0x3260, 0x00},
	 {0x3261, 0x1d},
	 {0x3262, 0xd8},
	 {0x3263, 0x0f},
	 {0x3264, 0xf7},
	 {0x3265, 0x11},
	 {0x3266, 0x0f},
	 {0x3267, 0xea},
	 {0x3268, 0x3d},
	 {0x3269, 0x00},
	 {0x326a, 0x09},
	 {0x326b, 0xcc},
	 {0x326c, 0x00},
	 {0x326d, 0x99},
	 {0x326e, 0x45},
	 {0x326f, 0x0f},
	 {0x3270, 0xd3},
	 {0x3271, 0x80},
	 {0x3272, 0x00},
	 {0x3273, 0x20},
	 {0x3274, 0xf7},
	 {0x3275, 0x0f},
	 {0x3276, 0xef},
	 {0x3277, 0x0d},
	 {0x3278, 0x00},
	 {0x3279, 0x09},
	 {0x327a, 0x3c},
	 {0x327b, 0x00},
	 {0x327c, 0x01},
	 {0x327d, 0x16},
	 {0x327e, 0x0f},
	 {0x327f, 0xc9},
	 {0x3280, 0x36},
	 {0x3281, 0x00},
	 {0x3282, 0x21},
	 {0x3283, 0xff},
	 {0x3284, 0x0f},
	 {0x3285, 0xdc},
	 {0x3286, 0xc2},
	 {0x3287, 0x00},
	 {0x3288, 0x1e},
	 {0x3289, 0xc0},
	 {0x328a, 0x0f},
	 {0x328b, 0xf0},
	 {0x328c, 0xa7},
	 {0x328d, 0x0f},
	 {0x328e, 0xf9},
	 {0x328f, 0x2a},
	 {0x3290, 0x00},
	 {0x3291, 0x29},
	 {0x3292, 0x5c},
	 {0x3293, 0x0f},
	 {0x3294, 0xc9},
	 {0x3295, 0x2a},
	 {0x3296, 0x00},
	 {0x3297, 0x1f},
	 {0x3298, 0x5c},
	 {0x3299, 0x0f},
	 {0x329a, 0xfa},
	 {0x329b, 0x0c},
	 {0x329c, 0x0f},
	 {0x329d, 0xf3},
	 {0x329e, 0x94},
	 {0x329f, 0x00},
	 {0x32a0, 0x1c},
	 {0x32a1, 0xce},
	 {0x32a2, 0x0f},
	 {0x32a3, 0xed},
	 {0x32a4, 0xb7},
	 {0x32a5, 0x00},
	 {0x32a6, 0x34},
	 {0x32a7, 0x51},
	 {0x32a8, 0x0f},
	 {0x32a9, 0xfa},
	 {0x32aa, 0x7d},
	 {0x32ab, 0x0f},
	 {0x32ac, 0xe6},
	 {0x32ad, 0xbf},
	 {0x32ae, 0x00},
	 {0x32af, 0x18},
	 {0x32b0, 0xc6},
	 {0x32b1, 0x0f},
	 {0x32b2, 0xe0},
	 {0x32b3, 0x72},
	 {0x32b4, 0x00},
	 {0x32b5, 0x08},
	 {0x32b6, 0x23},
	 {0x32b7, 0x0f},
	 {0x32b8, 0xf1},
	 {0x32b9, 0x54},
	 {0x32ba, 0x0f},
	 {0x32bb, 0xe1},
	 {0x32bc, 0x84},
	 {0x32bd, 0x00},
	 {0x32be, 0x26},
	 {0x32bf, 0xb1},
	 {0x32c0, 0x0f},
	 {0x32c1, 0xfa},
	 {0x32c2, 0xc2},
	 {0x32c3, 0x00},
	 {0x32c4, 0x05},
	 {0x32c5, 0x3d},
	 {0x32c6, 0x0f},
	 {0x32c7, 0xff},
	 {0x32c8, 0xaf},
	 {0x32c9, 0x0f},
	 {0x32ca, 0xf1},
	 {0x32cb, 0xe5},
	 {0x32cc, 0x00},
	 {0x32cd, 0x21},
	 {0x32ce, 0xdd},
	 {0x32cf, 0x0f},
	 {0x32d0, 0xe8},
	 {0x32d1, 0x6a},
	 {0x32d2, 0x0f},
	 {0x32d3, 0xf4},
	 {0x32d4, 0xfb},
	 {0x32d5, 0x00},
	 {0x32d6, 0x0c},
	 {0x32d7, 0x89},
	 {0x32d8, 0x00},
	 {0x32d9, 0x7c},
	 {0x32da, 0x79},
	 {0x32db, 0x0f},
	 {0x32dc, 0xde},
	 {0x32dd, 0x19},
	 {0x32de, 0x00},
	 {0x32df, 0x19},
	 {0x32e0, 0xe8},
	 {0x32e1, 0x0f},
	 {0x32e2, 0xf3},
	 {0x32e3, 0x41},
	 {0x32e4, 0x00},
	 {0x32e5, 0x03},
	 {0x32e6, 0x4c},
	 {0x32e7, 0x00},
	 {0x32e8, 0x05},
	 {0x32e9, 0x73},
	 {0x32ea, 0x0f},
	 {0x32eb, 0xd6},
	 {0x32ec, 0xa5},
	 {0x32ed, 0x00},
	 {0x32ee, 0x1f},
	 {0x32ef, 0x81},
	 {0x32f0, 0x0f},
	 {0x32f1, 0xdc},
	 {0x32f2, 0xe6},
	 {0x32f3, 0x00},
	 {0x32f4, 0x18},
	 {0x32f5, 0x65},
	 {0x32f6, 0x00},
	 {0x32f7, 0x00},
	 {0x32f8, 0x11},
	 {0x32f9, 0x0f},
	 {0x32fa, 0xed},
	 {0x32fb, 0x65},
	 {0x32fc, 0x00},
	 {0x32fd, 0x23},
	 {0x32fe, 0x12},
	 {0x32ff, 0x0f},
	 {0x3300, 0xcf},
	 {0x3301, 0x28},
	 {0x3302, 0x00},
	 {0x3303, 0x2b},
	 {0x3304, 0xda},
	 {0x3305, 0x0f},
	 {0x3306, 0xef},
	 {0x3307, 0xae},
	 {0x3308, 0x0f},
	 {0x3309, 0xeb},
	 {0x330a, 0x13},
	 {0x330b, 0x00},
	 {0x330c, 0x27},
	 {0x330d, 0xb8},
	 {0x330e, 0x0f},
	 {0x330f, 0xec},
	 {0x3310, 0x69},
	 {0x3311, 0x00},
	 {0x3312, 0x2f},
	 {0x3313, 0x5f},
	 {0x3314, 0x0f},
	 {0x3315, 0xdf},
	 {0x3316, 0x4f},
	 {0x3317, 0x00},
	 {0x3318, 0x05},
	 {0x3319, 0x70},
	 {0x331a, 0x00},
	 {0x331b, 0x0f},
	 {0x331c, 0xd2},
	 {0x331d, 0x0f},
	 {0x331e, 0xe1},
	 {0x331f, 0xd8},
	 {0x3320, 0x00},
	 {0x3321, 0x09},
	 {0x3322, 0xcf},
	 {0x3323, 0x0f},
	 {0x3324, 0xf2},
	 {0x3325, 0x6e},
	 {0x3326, 0x0f},
	 {0x3327, 0xf6},
	 {0x3328, 0xb4},
	 {0x3329, 0x00},
	 {0x332a, 0x0d},
	 {0x332b, 0x87},
	 {0x332c, 0x00},
	 {0x332d, 0x08},
	 {0x332e, 0x1e},
	 {0x332f, 0x0f},
	 {0x3330, 0xfa},
	 {0x3331, 0x6e},
	 {0x3332, 0x0f},
	 {0x3333, 0xff},
	 {0x3334, 0xaa},
	 {0x3335, 0x0f},
	 {0x3336, 0xf2},
	 {0x3337, 0xc0},
	 {0x3338, 0x00},
	 {0x3339, 0x1d},
	 {0x333a, 0x18},
	 {0x333b, 0x0f},
	 {0x333c, 0xef},
	 {0x333d, 0xed},
	 {0x333e, 0x0f},
	 {0x333f, 0xec},
	 {0x3340, 0xf6},
	 {0x3341, 0x00},
	 {0x3342, 0x16},
	 {0x3343, 0x8e},
	 {0x3344, 0x00},
	 {0x3345, 0x9c},
	 {0x3346, 0x52},
	 {0x3347, 0x0f},
	 {0x3348, 0xcf},
	 {0x3349, 0xb9},
	 {0x334a, 0x00},
	 {0x334b, 0x29},
	 {0x334c, 0xe9},
	 {0x334d, 0x0f},
	 {0x334e, 0xe2},
	 {0x334f, 0x83},
	 {0x3350, 0x00},
	 {0x3351, 0x11},
	 {0x3352, 0xcc},
	 {0x3353, 0x0f},
	 {0x3354, 0xff},
	 {0x3355, 0xf4},
	 {0x3356, 0x0f},
	 {0x3357, 0xc1},
	 {0x3358, 0xa4},
	 {0x3359, 0x00},
	 {0x335a, 0x2f},
	 {0x335b, 0xce},
	 {0x335c, 0x0f},
	 {0x335d, 0xc5},
	 {0x335e, 0xbb},
	 {0x335f, 0x00},
	 {0x3360, 0x35},
	 {0x3361, 0x2a},
	 {0x3362, 0x0f},
	 {0x3363, 0xe6},
	 {0x3364, 0x2a},
	 {0x3365, 0x0f},
	 {0x3366, 0xf7},
	 {0x3367, 0x44},
	 {0x3368, 0x00},
	 {0x3369, 0x31},
	 {0x336a, 0xfe},
	 {0x336b, 0x0f},
	 {0x336c, 0xb6},
	 {0x336d, 0x84},
	 {0x336e, 0x00},
	 {0x336f, 0x3c},
	 {0x3370, 0x71},
	 {0x3371, 0x0f},
	 {0x3372, 0xe5},
	 {0x3373, 0xfe},
	 {0x3374, 0x0f},
	 {0x3375, 0xf2},
	 {0x3376, 0x87},
	 {0x3377, 0x00},
	 {0x3378, 0x29},
	 {0x3379, 0x2b},
	 {0x337a, 0x0f},
	 {0x337b, 0xe5},
	 {0x337c, 0x3f},
	 {0x337d, 0x00},
	 {0x337e, 0x45},
	 {0x337f, 0xc6},
	 {0x3380, 0x0f},
	 {0x3381, 0xdf},
	 {0x3382, 0xe6},
	 {0x3383, 0x0f},
	 {0x3384, 0xfb},
	 {0x3385, 0x0f},
	 {0x3386, 0x00},
	 {0x3387, 0x0f},
	 {0x3388, 0xf4},
	 {0x3389, 0x0f},
	 {0x338a, 0xdf},
	 {0x338b, 0x72},
	 {0x338c, 0x00},
	 {0x338d, 0x0e},
	 {0x338e, 0xaf},
	 {0x338f, 0x0f},
	 {0x3390, 0xed},
	 {0x3391, 0x7a},
	 {0x3392, 0x0f},
	 {0x3393, 0xe5},
	 {0x3394, 0xab},
	 {0x3395, 0x00},
	 {0x3396, 0x18},
	 {0x3397, 0x43},
	 {0x3398, 0x00},
	 {0x3399, 0x1b},
	 {0x339a, 0x41},
	 {0x339b, 0x0f},
	 {0x339c, 0xea},
	 {0x339d, 0x84},
	 {0x339e, 0x0f},
	 {0x339f, 0xfd},
	 {0x33a0, 0xdb},
	 {0x33a1, 0x0f},
	 {0x33a2, 0xe9},
	 {0x33a3, 0xbd},
	 {0x33a4, 0x00},
	 {0x33a5, 0x30},
	 {0x33a6, 0x77},
	 {0x33a7, 0x0f},
	 {0x33a8, 0xe9},
	 {0x33a9, 0x93},
	 {0x33aa, 0x0f},
	 {0x33ab, 0xd7},
	 {0x33ac, 0xde},
	 {0x33ad, 0x00},
	 {0x33ae, 0x2a},
	 {0x33af, 0x14},
	 {0x309D, 0x62},
	 {0x309d, 0x22}, /* shading enable */
	 /*LC setting End */
	 }
}; /* lc_setting} */

static struct wake_lock s5k3e2fx_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&s5k3e2fx_wake_lock, WAKE_LOCK_IDLE, "s5k3e2fx");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&s5k3e2fx_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&s5k3e2fx_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&s5k3e2fx_wake_lock);
}

struct reg_struct {
/* PLL setting */
	uint8_t pre_pll_clk_div;	/* 0x0305 */
	uint8_t pll_multiplier_msb;	/* 0x0306 */
	uint8_t pll_multiplier_lsb;	/* 0x0307 */
	uint8_t vt_pix_clk_div;	/* 0x0301 */
	uint8_t vt_sys_clk_div;	/* 0x0303 */
	uint8_t op_pix_clk_div;	/* 0x0309 */
	uint8_t op_sys_clk_div;	/* 0x030B */
/* Data Format */
	uint8_t ccp_data_format_msb;	/* 0x0112 */
	uint8_t ccp_data_format_lsb;	/* 0x0113 */
/* Preview Output Size */
	uint8_t x_output_size_msb;	/* 0x034C */
	uint8_t x_output_size_lsb;	/* 0x034D */
	uint8_t y_output_size_msb;	/* 0x034E */
	uint8_t y_output_size_lsb;	/* 0x034F */
/* add the X-Y addr setting position */
	uint8_t x_addr_start_MSB;	/* 0x0344 */
	uint8_t x_addr_start_LSB;	/* 0x0345 */
	uint8_t y_addr_start_MSB;	/* 0x0346 */
	uint8_t y_addr_start_LSB;	/* 0x0347 */
	uint8_t x_addr_end_MSB;	/* 0x0348 */
	uint8_t x_addr_end_LSB;	/* 0x0349 */
	uint8_t y_addr_end_MSB;	/* 0x034A */
	uint8_t y_addr_end_LSB;	/* 0x034B */
/* change the setting position */
/* Frame format */
	uint8_t frame_length_lines_msb;	/* 0x0340 */
	uint8_t frame_length_lines_lsb;	/* 0x0341 */
	uint8_t line_length_pck_msb;	/* 0x0342 */
	uint8_t line_length_pck_lsb;	/* 0x0343 */
/* binning */
	uint8_t x_even_inc;	/* 0x0381 */
	uint8_t x_odd_inc;	/* 0x0383 */
	uint8_t y_even_inc;	/* 0x0385 */
	uint8_t y_odd_inc;	/* 0x0387 */
	uint8_t binning_enable;	/* 0x3014 */
/* Samsung MSR Setting */
	uint8_t sel_ccp;	/* 0x30C4 */
	uint8_t ld_start;	/* 0x3000 */
	uint8_t ld_end;		/* 0x3001 */
	uint8_t sl_start;	/* 0x3002 */
	uint8_t sl_end;		/* 0x3003 */
	uint8_t rx_start;	/* 0x3004 */
	uint8_t s1_start;	/* 0x3005 */
	uint8_t s1_end;		/* 0x3006 */
	uint8_t s1s_start;	/* 0x3007 */
	uint8_t s1s_end;	/* 0x3008 */
	uint8_t s3_start;	/* 0x3009 */
	uint8_t s3_end;		/* 0x300A */
	uint8_t cmp_en_start;	/* 0x300B */
	uint8_t clp_sl_start;	/* 0x300C */
	uint8_t clp_sl_end;	/* 0x300D */
	uint8_t off_start;	/* 0x300E */
	uint8_t rmp_en_start;	/* 0x300F */
	uint8_t tx_start;	/* 0x3010 */
	uint8_t tx_end;		/* 0x3011 */
	uint8_t stx_width;	/* 0x3012 */
/* Samsung other MSR setting */
	uint8_t clamp_on;	/* 0x3015 */
	uint8_t reg_301d_reserved;	/* 0x301D */
	uint8_t vpix;		/* 0x3024 */
	uint8_t reg_3028_reserved;	/* 0x3028 */
	uint8_t reg_3070_reserved;	/* 0x3070 */
	uint8_t reg_3072_reserved;	/* 0x3072 */
	uint8_t reg_301b_reserved;	/* 0x301B */
	uint8_t offset;		/* 0x307E */
	uint8_t reg_30bd_reserved;	/* 0x30BD */
	uint8_t reg_30c2_reserved;	/* 0x30C2 */
	uint8_t shade_clk_enable;	/* 0x30AC */
	uint8_t reg_3051_reserved;	/* 0x3051 */
	uint8_t reg_3029_reserved;	/* 0x3029 */
	uint8_t reg_30bf_reserved;	/* 0x30BF */
	uint8_t reg_3022_reserved;	/* 0x3022 */
	uint8_t reg_3019_reserved;	/* 0x3019 */
/* end: Samsung other MSR setting */
	uint8_t reg_3152_reserved;	/* 0x3152 */
/* Samsung signal output setting */
	uint8_t reg_3150_reserved;	/* 0x3150 */
	uint8_t reg_3157_reserved;	/* 0x3157 */
	uint8_t reg_3159_reserved;	/* 0x3159 */
/* end: Samsung signal output setting */
	uint8_t reg_315A_reserved;	/* 0x315A */
/* AEC Setting */
	uint8_t analogue_gain_code_global_msb;	/* 0x0204 */
	uint8_t analogue_gain_code_global_lsb;	/* 0x0205 */
	uint8_t fine_integration_time;	/* 0x0200 */
	uint8_t coarse_integration_time;	/* 0x0202 */
/* LC Preview/Snapshot difference register */
/* Preview LC Setting */
	uint8_t sh4ch_blk_width_r;	/* 0x309E */
	uint8_t sh4ch_blk_height_r;	/* 0x309F */
	uint8_t sh4ch_step_x_r_MSB;	/* 0x30A0 */
	uint8_t sh4ch_step_x_r_LSB;	/* 0x30A1 */
	uint8_t sh4ch_step_y_r_MSB;	/* 0x30A2 */
	uint8_t sh4ch_step_y_r_LSB;	/* 0x30A3 */
	uint8_t sh4ch_start_blk_cnt_x_r;	/* 0x30A4 */
	uint8_t sh4ch_start_blk_int_x_r;	/* 0x30A5 */
	uint8_t sh4ch_start_frac_x_r_MSB;	/* 0x30A6 */
	uint8_t sh4ch_start_frac_x_r_LSB;	/* 0x30A7 */
	uint8_t sh4ch_start_blk_cnt_y_r;	/* 0x30A8 */
	uint8_t sh4ch_start_blk_int_y_r;	/* 0x30A9 */
	uint8_t sh4ch_start_frac_y_r_MSB;	/* 0x30AA */
	uint8_t sh4ch_start_frac_y_r_LSB;	/* 0x30AB */
/* end: LC Preview/Snapshot difference register */
	uint32_t size_h;
	uint32_t blk_l;
	uint32_t size_w;
	uint32_t blk_p;
};

struct reg_struct s5k3e2fx_reg_pat[2] = {
	{			/* Preview */
/* PLL setting */
	 0x06,			/* pre_pll_clk_div               REG=0x0305 */
	 0x00,			/* pll_multiplier_msb            REG=0x0306 */
	 REG_PLL_MULTIPLIER_LSB_VALUE,
				/* pll_multiplier_lsb            REG=0x0307 */
	 0x08,			/* vt_pix_clk_div                REG=0x0301 */
	 0x01,			/* vt_sys_clk_div                REG=0x0303 */
	 0x08,			/* op_pix_clk_div                REG=0x0309 */
	 0x01,			/* op_sys_clk_div                REG=0x030B */
/* Data Format */
	 0x0a,			/* ccp_data_format_msb   REG=0x0112 */
	 0x0a,			/* ccp_data_format_lsb   REG=0x0113 */
/* Preview Output Size */
	 0x05,			/* x_output_size_msb     REG=0x034C */
	 0x10,			/* x_output_size_lsb     REG=0x034D */
	 0x03,			/* y_output_size_msb     REG=0x034E */
	 0xcc,			/* y_output_size_lsb     REG=0x034F */
/* X-Y addr setting position. Start */
	 0x00,			/* x_addr_start_MSB              REG=0x0344 */
	 0x08,			/* x_addr_start_LSB              REG=0x0345 */
	 0x00,			/* y_addr_start_MSB              REG=0x0346 */
	 0x08,			/* y_addr_start_LSB              REG=0x0347 */
	 0x0a,			/* x_addr_end_MSB                REG=0x0348 */
	 0x27,			/* x_addr_end_LSB                REG=0x0349 */
	 0x07,			/* y_addr_end_MSB                REG=0x034A */
	 0x9f,			/* y_addr_end_LSB                REG=0x034B */
/* change the setting position */
/* Frame format */
	 0x03,			/* frame_length_lines_msb        REG=0x0340 */
	 0xe2,			/* frame_length_lines_lsb        REG=0x0341 */
	 0x0a,			/* line_length_pck_msb           REG=0x0342 */
	 0xac,			/* line_length_pck_lsb           REG=0x0343 */
/* enable binning for preview */
	 0x01,			/* x_even_inc             REG=0x0381 */
	 0x01,			/* x_odd_inc              REG=0x0383 */
	 0x01,			/* y_even_inc             REG=0x0385 */
	 0x03,			/* y_odd_inc              REG=0x0387 */
	 0x06,			/* binning_enable         REG=0x3014 */
/* Samsung MSR Setting */
	 0x01,			/* sel_ccp                       REG=0x30C4 */
	 0x03,			/* ld_start                      REG=0x3000 */
	 0x94,			/* ld_end                        REG=0x3001 */
	 0x02,			/* sl_start                      REG=0x3002 */
	 0x95,			/* sl_end                        REG=0x3003 */
	 0x0f,			/* rx_start                      REG=0x3004 */
	 0x05,			/* s1_start                      REG=0x3005 */
	 0x3c,			/* s1_end                        REG=0x3006 */
	 0x8c,			/* s1s_start                     REG=0x3007 */
	 0x93,			/* s1s_end                       REG=0x3008 */
	 0x05,			/* s3_start                      REG=0x3009 */
	 0x3a,			/* s3_end                        REG=0x300A */
	 0x10,			/* cmp_en_start                  REG=0x300B */
	 0x02,			/* clp_sl_start                  REG=0x300C */
	 0x3e,			/* clp_sl_end                    REG=0x300D */
	 0x02,			/* off_start                     REG=0x300E */
	 0x0e,			/* rmp_en_start                  REG=0x300F */
	 0x46,			/* tx_start                      REG=0x3010 */
	 0x64,			/* tx_end                        REG=0x3011 */
	 0x1e,			/* stx_width                     REG=0x3012 */
/* Samsung other MSR setting. */
	 0x00,			/* clamp_on                      REG=0x3015 */
	 0x3f,			/* reg_301d_reserved             REG=0x301D */
	 0x04,			/* vpix                          REG=0x3024 */
	 0x40,			/* reg_3028_reserved             REG=0x3028 */
	 0xdf,			/* reg_3070_reserved             REG=0x3070 */
	 0x20,			/* reg_3072_reserved             REG=0x3072 */
	 0x73,			/* reg_3073_reserved             REG=0x301B */
	 0x02,			/* offset                        REG=0x307E */
	 0x06,			/* reg_30bd_reserved             REG=0x30BD */
	 0x0b,			/* reg_30c2_reserved             REG=0x30C2 */
	 0x81,			/* shade_clk_enable              REG=0x30AC */
	 0xe6,			/* reg_3051_reserved             REG=0x3051 */
	 0x02,			/* reg_3029_reserved             REG=0x3029 */
	 0x00,			/* reg_30bf_reserved             REG=0x30BF */
	 0x87,			/* reg_3022_reserved             REG=0x3022 */
	 0x60,			/* reg_3019_reserved             REG=0x3019 */
/* end: Samsung other MSR setting. */
	 0x08,			/* reg_3152_reserved             REG=0x3152 */
	 0x50,			/* reg_3150_reserved             REG=0x3150 */
/* Inverse PCLK */
	 0x04,			/* reg_3157_reserved             REG=0x3157 */
/* PCLK Delay offset; 0x0a will delay around 4ns at 80MHz */
	 0x0f,			/* reg_3159_reserved             REG=0x3159 */
/* HS, VS driving strength [3:2]=>VS, [1:0]=>HS 00:2mA, 01:4mA, 10:6mA,
 * 11:8mA
 */
	 0xf0,			/* reg_315A_reserved             REG=0x315A */
/* PCLK, DATA driving strength [7:6]=>data, [5:4]=>PCLK 00:2mA, 01:4mA, 10:6mA,
 * 11:8mA
 */
/* AEC Setting */
	 0x00,			/* analogue_gain_code_global_msb REG=0x0204 */
	 REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB_VALUE,
				/* analogue_gain_code_global_lsb REG=0x0205 */
	 0x02,			/* fine_integration_time         REG=0x0200 */
	 0x03,			/* coarse_integration_time       REG=0x0202 */
/* LC Preview/Snapshot difference register. */
/* Preview LC config Setting */
	 0x52,			/* sh4ch_blk_width_r             REG=0x309E */
	 0x3e,			/* sh4ch_blk_height_r            REG=0x309F */
	 0x03,			/* sh4ch_step_x_r_MSB            REG=0x30A0 */
	 0x1f,			/* sh4ch_step_x_r_LSB            REG=0x30A1 */
	 0x04,			/* sh4ch_step_y_r_MSB            REG=0x30A2 */
	 0x21,			/* sh4ch_step_y_r_LSB            REG=0x30A3 */
	 0x04,			/* sh4ch_start_blk_cnt_x_r       REG=0x30A4 */
	 0x00,			/* sh4ch_start_blk_int_x_r       REG=0x30A5 */
	 0x0c,			/* sh4ch_start_frac_x_r_MSB      REG=0x30A6 */
	 0x7c,			/* sh4ch_start_frac_x_r_LSB      REG=0x30A7 */
	 0x04,			/* sh4ch_start_blk_cnt_y_r       REG=0x30A8 */
	 0x00,			/* sh4ch_start_blk_int_y_r       REG=0x30A9 */
	 0x10,			/* sh4ch_start_frac_y_r_MSB      REG=0x30AA */
	 0x84,			/* sh4ch_start_frac_y_r_LSB      REG=0x30AB */
/* end: LC Preview/Snapshot difference register. */
	 S5K3E2FX_QTR_SIZE_HEIGHT,
	 18,
	 S5K3E2FX_QTR_SIZE_WIDTH,
	 1436},
	{			/* Snapshot */
/* PLL setting */
	 0x06,			/* pre_pll_clk_div               REG=0x0305 */
	 0x00,			/* pll_multiplier_msb            REG=0x0306 */
	 REG_PLL_MULTIPLIER_LSB_VALUE,
				/* pll_multiplier_lsb            REG=0x0307 */
	 0x08,			/* vt_pix_clk_div                REG=0x0301 */
	 0x01,			/* vt_sys_clk_div                REG=0x0303 */
	 0x08,			/* op_pix_clk_div                REG=0x0309 */
	 0x01,			/* op_sys_clk_div                REG=0x030B */
/* Data Format */
	 0x0a,			/* ccp_data_format_msb           REG=0x0112 */
	 0x0a,			/* ccp_data_format_lsb           REG=0x0113 */
/* Snapshot Output Size */
	 0x0a,			/* x_output_size_msb             REG=0x034C */
	 0x30,			/* x_output_size_lsb             REG=0x034D */
	 0x07,			/* y_output_size_msb             REG=0x034E */
	 0xa8,			/* y_output_size_lsb             REG=0x034F */
/* add the X-Y addr setting position. */
	 0x00,			/* x_addr_start_MSB              REG=0x0344 */
	 0x00,			/* x_addr_start_LSB              REG=0x0345 */
	 0x00,			/* y_addr_start_MSB              REG=0x0346 */
	 0x00,			/* y_addr_start_LSB              REG=0x0347 */
	 0x0a,			/* x_addr_end_MSB                REG=0x0348 */
	 0x2F,			/* x_addr_end_LSB                REG=0x0349 */
	 0x07,			/* y_addr_end_MSB                REG=0x034A */
	 0xA7,			/* y_addr_end_LSB                REG=0x034B */
/* Change the setting position. */
/* Frame format */
	 0x07,			/* frame_length_lines_msb        REG=0x0340 */
	 0xb6,			/* frame_length_lines_lsb        REG=0x0341 */
	 0x0a,			/* line_length_pck_msb           REG=0x0342 */
	 0xac,			/* line_length_pck_lsb           REG=0x0343 */
/* disable binning for snapshot */
	 0x01,			/* x_even_inc                    REG=0x0381 */
	 0x01,			/* x_odd_inc                     REG=0x0383 */
	 0x01,			/* y_even_inc                    REG=0x0385 */
	 0x01,			/* y_odd_inc                     REG=0x0387 */
	 0x00,			/* binning_enable                REG=0x3014 */
/* Samsung MSR Setting */
	 0x01,			/* sel_ccp                       REG=0x30C4 */
	 0x03,			/* ld_start                      REG=0x3000 */
	 0x94,			/* ld_end                        REG=0x3001 */
	 0x02,			/* sl_start                      REG=0x3002 */
	 0x95,			/* sl_end                        REG=0x3003 */
	 0x0f,			/* rx_start                      REG=0x3004 */
	 0x05,			/* s1_start                      REG=0x3005 */
	 0x3c,			/* s1_end                        REG=0x3006 */
	 0x8c,			/* s1s_start                     REG=0x3007 */
	 0x93,			/* s1s_end                       REG=0x3008 */
	 0x05,			/* s3_start                      REG=0x3009 */
	 0x3a,			/* s3_end                        REG=0x300A */
	 0x10,			/* cmp_en_start                  REG=0x300B */
	 0x02,			/* clp_sl_start                  REG=0x300C */
	 0x3e,			/* clp_sl_end                    REG=0x300D */
	 0x02,			/* off_start                     REG=0x300E */
	 0x0e,			/* rmp_en_start                  REG=0x300F */
	 0x46,			/* tx_start                      REG=0x3010 */
	 0x64,			/* tx_end                        REG=0x3011 */
	 0x1e,			/* stx_width                     REG=0x3012 */
/* Add Samsung other MSR setting. */
	 0x00,			/* clamp_on                      REG=0x3015 */
	 0x3f,			/* reg_301d_reserved             REG=0x301D */
	 0x04,			/* vpix                          REG=0x3024 */
	 0x40,			/* reg_3028_reserved             REG=0x3028 */
	 0xdf,			/* reg_3070_reserved             REG=0x3070 */
	 0x20,			/* reg_3072_reserved             REG=0x3072 */
	 0x73,			/* reg_3073_reserved             REG=0x301B */
	 0x02,			/* offset                        REG=0x307E */
	 0x06,			/* reg_30bd_reserved             REG=0x30BD */
	 0x0b,			/* reg_30c2_reserved             REG=0x30C2 */
	 0x81,			/* shade_clk_enable              REG=0x30AC */
	 0xe6,			/* reg_3051_reserved             REG=0x3051 */
	 0x02,			/* reg_3029_reserved             REG=0x3029 */
	 0x00,			/* reg_30bf_reserved             REG=0x30BF */
	 0x87,			/* reg_3022_reserved             REG=0x3022 */
	 0x60,			/* reg_3019_reserved             REG=0x3019 */
/* end: Add Samsung other MSR setting. */
	 0x08,			/* reg_3152_reserved             REG=0x3152 */
/* Add Samsung signal output setting. */
	 0x50,			/* reg_3150_reserved             REG=0x3150 */
/* Inverse PCLK = 0x50 */
	 0x04,			/* reg_3157_reserved             REG=0x3157 */
/* PCLK Delay offset; 0x0a will delay around 4ns at 80MHz */
	 0x0f,			/* reg_3159_reserved             REG=0x3159 */
/* HS, VS driving strength [3:2]=>VS, [1:0]=>HS 00:2mA, 01:4mA, 10:6mA,
 * 11:8mA
 */
	 0xf0,			/* reg_315A_reserved             REG=0x315A */
/* PCLK, DATA driving strength [7:6]=>data, [5:4]=>PCLK 00:2mA, 01:4mA, 10:6mA,
 * 11:8mA
 */
/* AEC Setting */
	 0x00,			/* analogue_gain_code_global_msb REG=0x0204 */
	 REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB_VALUE,
				/* analogue_gain_code_global_lsb REG=0x0205 */
	 0x02,			/* fine_integration_time         REG=0x0200 */
	 0x03,			/* coarse_integration_time       REG=0x0202 */
/* Add LC Preview/Snapshot diff register. */
/* Snapshot LC config Setting */
	 0x52,			/* sh4ch_blk_width_r             REG=0x309E */
	 0x7b,			/* sh4ch_blk_height_r            REG=0x309F */
	 0x03,			/* sh4ch_step_x_r_MSB            REG=0x30A0 */
	 0x1f,			/* sh4ch_step_x_r_LSB            REG=0x30A1 */
	 0x02,			/* sh4ch_step_y_r_MSB            REG=0x30A2 */
	 0x15,			/* sh4ch_step_y_r_LSB            REG=0x30A3 */
	 0x00,			/* sh4ch_start_blk_cnt_x_r       REG=0x30A4 */
	 0x00,			/* sh4ch_start_blk_int_x_r       REG=0x30A5 */
	 0x00,			/* sh4ch_start_frac_x_r_MSB      REG=0x30A6 */
	 0x00,			/* sh4ch_start_frac_x_r_LSB      REG=0x30A7 */
	 0x00,			/* sh4ch_start_blk_cnt_y_r       REG=0x30A8 */
	 0x00,			/* sh4ch_start_blk_int_y_r       REG=0x30A9 */
	 0x00,			/* sh4ch_start_frac_y_r_MSB      REG=0x30AA */
	 0x00,			/* sh4ch_start_frac_y_r_LSB      REG=0x30AB */
/* diff: Add LC Preview/Snapshot diff register. */
	 S5K3E2FX_FULL_SIZE_HEIGHT,
	 14,
	 S5K3E2FX_FULL_SIZE_WIDTH,
	 124}
};

struct s5k3e2fx_work {
	struct work_struct work;
};
static struct s5k3e2fx_work *s5k3e2fx_sensorw;
static struct i2c_client *s5k3e2fx_client;

struct s5k3e2fx_ctrl {
	const struct msm_camera_sensor_info *sensordata;

	int sensormode;
	uint32_t fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode set_test;
};

static struct s5k3e2fx_ctrl *s5k3e2fx_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(s5k3e2fx_wait_queue);

#define MAX_I2C_RETRIES 20
static int i2c_transfer_retry(struct i2c_adapter *adap,
			struct i2c_msg *msgs,
			int len)
{
	int i2c_retry = 0;
	int ns; /* number sent */

	while (i2c_retry++ < MAX_I2C_RETRIES) {
		ns = i2c_transfer(adap, msgs, len);
		if (ns == len)
			break;
		pr_err("%s: try %d/%d: i2c_transfer sent: %d, len %d\n",
			__func__,
			i2c_retry, MAX_I2C_RETRIES, ns, len);
		msleep(10);
	}

	return ns == len ? 0 : -EIO;
}

static inline int s5k3e2fx_i2c_rxdata(unsigned short saddr, unsigned char *rxdata,
			       int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = 2,
			.buf = rxdata,
		},
		{
			.addr = saddr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};

	return i2c_transfer_retry(s5k3e2fx_client->adapter, msgs, 2);
}

static inline int s5k3e2fx_i2c_txdata(unsigned short saddr,
				   unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	return i2c_transfer_retry(s5k3e2fx_client->adapter, msg, 1);
}

static int s5k3e2fx_i2c_write_b(unsigned short saddr, unsigned short waddr,
				    unsigned char bdata)
{
	int rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	rc = s5k3e2fx_i2c_txdata(saddr, buf, 3);

	if (rc < 0)
		pr_err("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
		       waddr, bdata);

	return rc;
}

static int s5k3e2fx_i2c_write_table(struct s5k3e2fx_i2c_reg_conf
					*reg_cfg_tbl, int num)
{
	int i;
	int rc = -EFAULT;
	CDBG("s5k3e2fx_i2c_write_table starts\n");
	for (i = 0; i < num; i++) {
		CDBG("%d: waddr = 0x%x, bdata = 0x%x\n", i,
		     (int)reg_cfg_tbl->waddr, (int)reg_cfg_tbl->bdata);
		rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
					  reg_cfg_tbl->waddr,
					  reg_cfg_tbl->bdata);
		if (rc < 0)
			break;
		reg_cfg_tbl++;
	}

	CDBG("s5k3e2fx_i2c_write_table ends\n");
	return rc;
}

static int s5k3e2fx_i2c_read_w(unsigned short saddr, unsigned short raddr,
				   unsigned short *rdata)
{
	int rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k3e2fx_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("s5k3e2fx_i2c_read failed!\n");

	return rc;
}

static int s5k3e2fx_i2c_read_b(unsigned short saddr, unsigned short raddr,
				   unsigned short *rdata)
{
	int rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k3e2fx_i2c_rxdata(saddr, buf, 1);
	if (rc < 0)
		return rc;

	*rdata = buf[0];

	if (rc < 0)
		pr_err("s5k3e2fx_i2c_read failed!\n");

	return rc;
}

static int s5k3e2fx_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int rc;
	uint16_t chipid = 0;
	uint16_t modulever = 0;

	CDBG("s5k3e2fx: gpio_request: %d\n", data->sensor_reset);
	rc = gpio_request(data->sensor_reset, "s5k3e2fx");
	if (!rc)
		gpio_direction_output(data->sensor_reset, 1);
	else {
		pr_err("s5k3e2fx: request GPIO(sensor_reset): %d failed\n",
			data->sensor_reset);
		goto init_probe_fail;
	}
	CDBG("s5k3e2fx: gpio_free: %d\n", data->sensor_reset);

	gpio_free(data->sensor_reset);

	msleep(20);

	CDBG("s5k3e2fx_sensor_init(): reseting sensor.\n");

	rc = s5k3e2fx_i2c_read_w(s5k3e2fx_client->addr, S5K3E2FX_REG_MODEL_ID,
				 &chipid);
	if (rc < 0) {
		pr_err("s5k3e2fx: read model_id failed: %d\n", rc);
		goto init_probe_fail;
	}
	CDBG("s5k3e2fx_sensor_init(): model_id=0x%X\n", chipid);

	if (chipid != S5K3E2FX_MODEL_ID) {
		pr_err("S5K3E2FX wrong model_id = 0x%x\n", chipid);
		rc = -ENODEV;
		goto init_probe_fail;
	}

	rc = s5k3e2fx_i2c_read_b(s5k3e2fx_client->addr,
				 S5K3E2FX_REG_MODULE_VER, &modulever);
	if (rc < 0) {
		pr_err("S5K3E2FX read module version failed, line=%d\n",
		       __LINE__);
		goto init_probe_fail;
	}
	/* modulever = (0xF000 & modulever) >> 8; */
	modulever = 0x00F0 & modulever;
	CDBG("s5k3e2fx_sensor_init(): module version=0x%X\n", modulever);

	if (modulever == 0x40)
		g_usModuleVersion = 0;
	else if (modulever == 0x50)
		g_usModuleVersion = 1;
	goto init_probe_done;

init_probe_fail:
	pr_err("s5k3e2fx: prob init sensor failed\n");
init_probe_done:
	return rc;
}

static int s5k3e2fx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k3e2fx_wait_queue);
	return 0;
}

static const struct i2c_device_id s5k3e2fx_i2c_id[] = {
	{"s5k3e2fx", 0},
	{}
};

static int s5k3e2fx_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("s5k3e2fx_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	s5k3e2fx_sensorw = kzalloc(sizeof(struct s5k3e2fx_work), GFP_KERNEL);
	if (!s5k3e2fx_sensorw) {
		pr_err("kzalloc failed\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k3e2fx_sensorw);
	s5k3e2fx_init_client(client);
	s5k3e2fx_client = client;

	msleep(50);

	CDBG("s5k3e2fx_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("s5k3e2fx_probe failed! rc = %d\n", rc);
	return rc;
}

static struct i2c_driver s5k3e2fx_i2c_driver = {
	.id_table = s5k3e2fx_i2c_id,
	.probe = s5k3e2fx_i2c_probe,
	.driver = {
		.name = "s5k3e2fx",
	},
};

#if 0
static int s5k3e2fx_test(enum msm_s_test_mode mo)
{
	int rc = 0;

	if (mo == S_TEST_OFF)
		rc = 0;
	else
		rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
					  REG_TEST_PATTERN_MODE, (uint16_t) mo);

	return rc;
}
#endif
static int s5k3e2fx_setting(enum msm_s_reg_update rupdate,
				enum msm_s_setting rt)
{
	int rc = 0;
	uint16_t num_lperf;

	switch (rupdate) {
	case S_UPDATE_PERIODIC:{
			if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {
				struct s5k3e2fx_i2c_reg_conf tbl_1[] = {
					{REG_X_OUTPUT_SIZE_MSB,
					 s5k3e2fx_reg_pat[rt].
					 x_output_size_msb},
					{REG_X_OUTPUT_SIZE_LSB,
					 s5k3e2fx_reg_pat[rt].
					 x_output_size_lsb},
					{REG_Y_OUTPUT_SIZE_MSB,
					 s5k3e2fx_reg_pat[rt].
					 y_output_size_msb},
					{REG_Y_OUTPUT_SIZE_LSB,
					 s5k3e2fx_reg_pat[rt].
					 y_output_size_lsb},
					/* Start-End address */
					{REG_X_ADDR_START_MSB,
					 s5k3e2fx_reg_pat[rt].x_addr_start_MSB},
					{REG_X_ADDR_START_LSB,
					 s5k3e2fx_reg_pat[rt].x_addr_start_LSB},
					{REG_Y_ADDR_START_MSB,
					 s5k3e2fx_reg_pat[rt].y_addr_start_MSB},
					{REG_Y_ADDR_START_LSB,
					 s5k3e2fx_reg_pat[rt].y_addr_start_LSB},
					{REG_X_ADDR_END_MSB,
					 s5k3e2fx_reg_pat[rt].x_addr_end_MSB},
					{REG_X_ADDR_END_LSB,
					 s5k3e2fx_reg_pat[rt].x_addr_end_LSB},
					{REG_Y_ADDR_END_MSB,
					 s5k3e2fx_reg_pat[rt].y_addr_end_MSB},
					{REG_Y_ADDR_END_LSB,
					 s5k3e2fx_reg_pat[rt].y_addr_end_LSB},
					/* Binning */
					{REG_X_EVEN_INC,
					 s5k3e2fx_reg_pat[rt].x_even_inc},
					{REG_X_ODD_INC,
					 s5k3e2fx_reg_pat[rt].x_odd_inc},
					{REG_Y_EVEN_INC,
					 s5k3e2fx_reg_pat[rt].y_even_inc},
					{REG_Y_ODD_INC,
					 s5k3e2fx_reg_pat[rt].y_odd_inc},
					{REG_BINNING_ENABLE,
					 s5k3e2fx_reg_pat[rt].binning_enable},
				};
				struct s5k3e2fx_i2c_reg_conf tbl_2[] = {
					{REG_FRAME_LENGTH_LINES_MSB, 0},
					{REG_FRAME_LENGTH_LINES_LSB, 0},
					{REG_LINE_LENGTH_PCK_MSB,
					 s5k3e2fx_reg_pat[rt].
					 line_length_pck_msb},
					{REG_LINE_LENGTH_PCK_LSB,
					 s5k3e2fx_reg_pat[rt].
					 line_length_pck_lsb},
					{REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB,
					 s5k3e2fx_reg_pat[rt].
					 analogue_gain_code_global_msb},
					{REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB,
					 s5k3e2fx_reg_pat[rt].
					 analogue_gain_code_global_lsb},
					{REG_FINE_INTEGRATION_TIME,
					 s5k3e2fx_reg_pat[rt].
					 fine_integration_time},
					{REG_COARSE_INTEGRATION_TIME,
					 s5k3e2fx_reg_pat[rt].
					 coarse_integration_time},
					/* LC Preview/Snapshot difference
					 * register
					 */
					{REG_SH4CH_BLK_WIDTH_R,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_blk_width_r},
					{REG_SH4CH_BLK_HEIGHT_R,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_blk_height_r},
					{REG_SH4CH_STEP_X_R_MSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_step_x_r_MSB},
					{REG_SH4CH_STEP_X_R_LSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_step_x_r_LSB},
					{REG_SH4CH_STEP_Y_R_MSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_step_y_r_MSB},
					{REG_SH4CH_STEP_Y_R_LSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_step_y_r_LSB},
					{REG_SH4CH_START_BLK_CNT_X_R,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_blk_cnt_x_r},
					{REG_SH4CH_START_BLK_INT_X_R,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_blk_int_x_r},
					{REG_SH4CH_START_FRAC_X_R_MSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_frac_x_r_MSB},
					{REG_SH4CH_START_FRAC_X_R_LSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_frac_x_r_LSB},
					{REG_SH4CH_START_BLK_CNT_Y_R,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_blk_cnt_y_r},
					{REG_SH4CH_START_BLK_INT_Y_R,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_blk_int_y_r},
					{REG_SH4CH_START_FRAC_Y_R_MSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_frac_y_r_MSB},
					{REG_SH4CH_START_FRAC_Y_R_LSB,
					 s5k3e2fx_reg_pat[rt].
					 sh4ch_start_frac_y_r_LSB},
				};

/* add EVT5 sensor Samsung difference MSR setting between Preview and Capture */

				struct s5k3e2fx_i2c_reg_conf
				    tbl_only_for_EVT5[2][2] = {
					{	/* S_RES_PREVIEW */
					 {0x3062, 0x00},
					 {0x3063, 0xD6},
					 },
					{	/* S_RES_CAPTURE */
					 {0x3062, 0x01},
					 {0x3063, 0x16},
					 }
				};

				/* Most registers are directly applied at next frame after
				   writing except shutter and analog gain. Shutter and gain are
				   applied at 2nd or 1st frame later depending on register
				   writing time. When the camera is switched from preview to
				   snapshot, the first frame may have wrong shutter/gain and
				   should be discarded. The register REG_MASK_CORRUPTED_FRAMES
				   can discard the frame that has wrong shutter/gain. But in
				   preview mode, the frames should not be dropped. Otherwise
				   the preview will not be smooth. */
				if (rt == S_RES_PREVIEW) {
					/* Frames will be not discarded after exposure and gain are
					   written. */
					s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
						REG_MASK_CORRUPTED_FRAMES, NO_MASK);
				} else {
					/* Solve greenish in lowlight. Prevent corrupted frame */
					s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
						REG_MASK_CORRUPTED_FRAMES, MASK);
				}

/* solve greenish: hold for both */
				rc = s5k3e2fx_i2c_write_b(
					s5k3e2fx_client->addr,
					REG_GROUPED_PARAMETER_HOLD,
					GROUPED_PARAMETER_HOLD);
				if (rc < 0)
					return rc;

				CDBG("Binning_enable = 0x %2x"
				     "[s5k3e2fx.c s5k3e2fx_setting]\r\n",
				     s5k3e2fx_reg_pat[rt].binning_enable);

				rc = s5k3e2fx_i2c_write_table(&tbl_1[0],
							ARRAY_SIZE
							(tbl_1));
				if (rc < 0) {
					pr_err("UPDATE_PERIODIC, tb1_1 failed");
					return rc;
				}

				num_lperf =
				    (uint16_t) ((s5k3e2fx_reg_pat[rt].
						 frame_length_lines_msb
						 << 8) & 0xFF00) +
				    s5k3e2fx_reg_pat[rt].
				    frame_length_lines_lsb;

				num_lperf =
				    num_lperf *
				    s5k3e2fx_ctrl->fps_divider / 0x0400;

				tbl_2[0] =
				    (struct s5k3e2fx_i2c_reg_conf) {
					REG_FRAME_LENGTH_LINES_MSB,
					    (num_lperf & 0xFF00) >> 8};
				tbl_2[1] =
				    (struct s5k3e2fx_i2c_reg_conf) {
					REG_FRAME_LENGTH_LINES_LSB,
					    (num_lperf & 0x00FF)};

				rc = s5k3e2fx_i2c_write_table(&tbl_2[0],
							ARRAY_SIZE
							(tbl_2));
				if (rc < 0) {
					pr_err("UPDATE_PERIODIC, tb1_2 failed");
					return rc;
				}

				/* only for evt5 */
				if (g_usModuleVersion == 1) {
					rc = s5k3e2fx_i2c_write_table
					    (&tbl_only_for_EVT5[rt][0],
					     2);
					if (rc < 0)
						return rc;
				}

				/* solve greenish: only release for preview */
				if (s5k3e2fx_ctrl->sensormode == SENSOR_PREVIEW_MODE)
				{
					rc = s5k3e2fx_i2c_write_b(
						s5k3e2fx_client->addr,
						REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_UPDATE);
					if (rc < 0)
						return rc;
				}

				rc = s5k3e2fx_i2c_write_b
					(s5k3e2fx_client->addr,
					S5K3E2FX_REG_MODE_SELECT,
					S5K3E2FX_MODE_SELECT_STREAM);
				if (rc < 0)
					return rc;
			}
		break; /* UPDATE_PERIODIC */
		}
	case S_REG_INIT:{
			if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {
				struct s5k3e2fx_i2c_reg_conf tbl_3[] = {
/* {S5K3E2FX_REG_SOFTWARE_RESET, S5K3E2FX_SOFTWARE_RESET},*/
					{S5K3E2FX_REG_MODE_SELECT,
					 S5K3E2FX_MODE_SELECT_SW_STANDBY},
					/*Output Size */
					{REG_X_OUTPUT_SIZE_MSB,
					 s5k3e2fx_reg_pat[rt].
					 x_output_size_msb},
					{REG_X_OUTPUT_SIZE_LSB,
					 s5k3e2fx_reg_pat[rt].
					 x_output_size_lsb},
					{REG_Y_OUTPUT_SIZE_MSB,
					 s5k3e2fx_reg_pat[rt].
					 y_output_size_msb},
					{REG_Y_OUTPUT_SIZE_LSB,
					 s5k3e2fx_reg_pat[rt].
					 y_output_size_lsb},
					/* Start-End address */
					{REG_X_ADDR_START_MSB,
					 s5k3e2fx_reg_pat[rt].x_addr_start_MSB},
					{REG_X_ADDR_START_LSB,
					 s5k3e2fx_reg_pat[rt].x_addr_start_LSB},
					{REG_Y_ADDR_START_MSB,
					 s5k3e2fx_reg_pat[rt].y_addr_start_MSB},
					{REG_Y_ADDR_START_LSB,
					 s5k3e2fx_reg_pat[rt].y_addr_start_LSB},
					{REG_X_ADDR_END_MSB,
					 s5k3e2fx_reg_pat[rt].x_addr_end_MSB},
					{REG_X_ADDR_END_LSB,
					 s5k3e2fx_reg_pat[rt].x_addr_end_LSB},
					{REG_Y_ADDR_END_MSB,
					 s5k3e2fx_reg_pat[rt].y_addr_end_MSB},
					{REG_Y_ADDR_END_LSB,
					 s5k3e2fx_reg_pat[rt].y_addr_end_LSB},
					/* Binning */
					{REG_X_EVEN_INC,
					 s5k3e2fx_reg_pat[rt].x_even_inc},
					{REG_X_ODD_INC,
					 s5k3e2fx_reg_pat[rt].x_odd_inc},
					{REG_Y_EVEN_INC,
					 s5k3e2fx_reg_pat[rt].y_even_inc},
					{REG_Y_ODD_INC,
					 s5k3e2fx_reg_pat[rt].y_odd_inc},
					{REG_BINNING_ENABLE,
					 s5k3e2fx_reg_pat[rt].binning_enable},
					/* Frame format */
					{REG_FRAME_LENGTH_LINES_MSB,
					 s5k3e2fx_reg_pat[rt].
					 frame_length_lines_msb},
					{REG_FRAME_LENGTH_LINES_LSB,
					 s5k3e2fx_reg_pat[rt].
					 frame_length_lines_lsb},
					{REG_LINE_LENGTH_PCK_MSB,
					 s5k3e2fx_reg_pat[rt].
					 line_length_pck_msb},
					{REG_LINE_LENGTH_PCK_LSB,
					 s5k3e2fx_reg_pat[rt].
					 line_length_pck_lsb},
					/* MSR setting */
					{REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB,
					 s5k3e2fx_reg_pat[rt].
					 analogue_gain_code_global_msb},
					{REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB,
					 s5k3e2fx_reg_pat[rt].
					 analogue_gain_code_global_lsb},
					{REG_FINE_INTEGRATION_TIME,
					 s5k3e2fx_reg_pat[rt].
					 fine_integration_time},
					{REG_COARSE_INTEGRATION_TIME,
					 s5k3e2fx_reg_pat[rt].
					 coarse_integration_time},
					{S5K3E2FX_REG_MODE_SELECT,
					 S5K3E2FX_MODE_SELECT_STREAM},
				};
				unsigned short rData = 0;
				mdelay(1);
				s5k3e2fx_i2c_read_b(s5k3e2fx_client->
						    addr,
						    REG_3150_RESERVED,
						    &rData);
				s5k3e2fx_i2c_write_b(s5k3e2fx_client->
						     addr,
						     REG_3150_RESERVED,
						     (rData & 0xFFFE));
				mdelay(1);
				s5k3e2fx_i2c_read_b(s5k3e2fx_client->
						    addr,
						    REG_TYPE1_AF_ENABLE,
						    &rData);
				s5k3e2fx_i2c_write_b(s5k3e2fx_client->
						addr,
						REG_TYPE1_AF_ENABLE,
						(rData | 0x0001));
				mdelay(1);

				/* reset fps_divider */
				s5k3e2fx_ctrl->fps_divider = 1 * 0x0400;
				/* write REG_INIT registers */
				rc = s5k3e2fx_i2c_write_table(&tbl_3[0],
							ARRAY_SIZE
							(tbl_3));
				if (rc < 0) {
					pr_err("REG_INIT failed, rc=%d\n", rc);
					return rc;
				}
			}
		}
		break; /* REG_INIT */

	default:
		rc = -EFAULT;
		break;
	} /* switch (rupdate) */

	return rc;
}

static int s5k3e2fx_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc;

	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	s5k3e2fx_ctrl = kzalloc(sizeof(struct s5k3e2fx_ctrl), GFP_KERNEL);
	if (!s5k3e2fx_ctrl) {
		pr_err("s5k3e2fx_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	s5k3e2fx_ctrl->fps_divider = 1 * 0x00000400;
	s5k3e2fx_ctrl->pict_fps_divider = 1 * 0x00000400;
	s5k3e2fx_ctrl->set_test = S_TEST_OFF;
	s5k3e2fx_ctrl->prev_res = S_QTR_SIZE;
	s5k3e2fx_ctrl->pict_res = S_FULL_SIZE;

	if (data)
		s5k3e2fx_ctrl->sensordata = data;

	/* enable mclk first */
	msm_camio_clk_rate_set(S5K3E2FX_DEF_MCLK);

	msleep(20);

	msm_camio_camif_pad_reg_reset();
	msleep(20);

	rc = s5k3e2fx_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail1;

	if (s5k3e2fx_ctrl->prev_res == S_QTR_SIZE)
		rc = s5k3e2fx_setting(S_REG_INIT, S_RES_PREVIEW);
	else
		rc = s5k3e2fx_setting(S_REG_INIT, S_RES_CAPTURE);

	if (rc < 0) {
		pr_err("s5k3e2fx_setting failed. rc = %d\n", rc);
		goto init_fail1;
	}

	rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
				0x3130, 0x03);
	if (rc < 0)
		goto init_fail1;

	goto init_done;

init_fail1:
	kfree(s5k3e2fx_ctrl);
init_done:
	return rc;
}

static void s5k3e2fx_suspend_sensor(void)
{
	unsigned short rData = 0;
	 /*AF*/
	s5k3e2fx_i2c_read_b(s5k3e2fx_client->addr,
			REG_TYPE1_AF_ENABLE, &rData);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			REG_TYPE1_AF_ENABLE, (rData & 0xFFFE));
	mdelay(1);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			S5K3E2FX_REG_MODE_SELECT,
			S5K3E2FX_MODE_SELECT_SW_STANDBY);
	msleep(210);		/*for 5FPS */
	/* hi z */
	s5k3e2fx_i2c_read_b(s5k3e2fx_client->addr, REG_3150_RESERVED, &rData);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			REG_3150_RESERVED, (rData | 0x0001));
	mdelay(1);

}

static int s5k3e2fx_power_down(void)
{
	int rc = -EBADF;
	s5k3e2fx_suspend_sensor();
	return rc;
}

static int s5k3e2fx_sensor_release(void)
{
	int rc = -EBADF;

	s5k3e2fx_suspend_sensor();

	kfree(s5k3e2fx_ctrl);
	s5k3e2fx_ctrl = NULL;

	allow_suspend();

	CDBG("s5k3e2fx_release completed\n");

	return rc;
}

static int s5k3e2fx_probe_init_lens_correction(
		const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	/* LC setting */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			     S5K3E2FX_REG_SOFTWARE_RESET,
			     S5K3E2FX_SOFTWARE_RESET);
	mdelay(2);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			     S5K3E2FX_REG_MODE_SELECT,
			     S5K3E2FX_MODE_SELECT_SW_STANDBY);
	/*20090811  separates the EVT4/EVT5 sensor init and LC setting start */
	s5k3e2fx_i2c_write_table(&Init_setting[g_usModuleVersion][0],
				 NUM_INIT_REG);

	/* 090911  Add for Samsung VCM calibration current Start */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3112, 0x0A);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3112, 0x09);
	mdelay(5);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3145, 0x04);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3146, 0x80);
	/* 090911 Add for Samsung VCM calibration current End */

	s5k3e2fx_i2c_write_table(&lc_setting[g_usModuleVersion][0], NUM_LC_REG);

	/*20090811  separates the EVT4/EVT5 sensor init and LC setting end */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			     S5K3E2FX_REG_MODE_SELECT,
			     S5K3E2FX_MODE_SELECT_STREAM);
	msleep(10);
	s5k3e2fx_suspend_sensor();

	return rc;
}

static void s5k3e2fx_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;	/* Q10 */

	divider = (uint32_t)
	    ((s5k3e2fx_reg_pat[S_RES_PREVIEW].size_h +
	      s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_l) *
	     (s5k3e2fx_reg_pat[S_RES_PREVIEW].size_w +
	      s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_p)) * 0x00000400 /
	    ((s5k3e2fx_reg_pat[S_RES_CAPTURE].size_h +
	      s5k3e2fx_reg_pat[S_RES_CAPTURE].blk_l) *
	     (s5k3e2fx_reg_pat[S_RES_CAPTURE].size_w +
	      s5k3e2fx_reg_pat[S_RES_CAPTURE].blk_p));

	/* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t) (fps * divider / 0x00000400);
}

static uint16_t s5k3e2fx_get_prev_lines_pf(void)
{
	return s5k3e2fx_reg_pat[S_RES_PREVIEW].size_h +
		s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_l;
}

static uint16_t s5k3e2fx_get_prev_pixels_pl(void)
{
	return s5k3e2fx_reg_pat[S_RES_PREVIEW].size_w +
		s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_p;
}

static uint16_t s5k3e2fx_get_pict_lines_pf(void)
{
	return s5k3e2fx_reg_pat[S_RES_CAPTURE].size_h +
		s5k3e2fx_reg_pat[S_RES_CAPTURE].blk_l;
}

static uint16_t s5k3e2fx_get_pict_pixels_pl(void)
{
	return s5k3e2fx_reg_pat[S_RES_CAPTURE].size_w +
		s5k3e2fx_reg_pat[S_RES_CAPTURE].blk_p;
}

static uint32_t s5k3e2fx_get_pict_max_exp_lc(void)
{
	uint32_t snapshot_lines_per_frame;

	if (s5k3e2fx_ctrl->pict_res == S_QTR_SIZE)
		snapshot_lines_per_frame =
		    s5k3e2fx_reg_pat[S_RES_PREVIEW].size_h +
		    s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_l;
	else
		snapshot_lines_per_frame = 3961 * 3;

	return snapshot_lines_per_frame;
}

static int s5k3e2fx_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int rc = 0;

	s5k3e2fx_ctrl->fps_divider = fps->fps_div;

	CDBG("s5k3e2fx_ctrl->fps_divider = %d\n",
		s5k3e2fx_ctrl->fps_divider);

	rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
				  REG_FRAME_LENGTH_LINES_MSB,
				  (((s5k3e2fx_reg_pat[S_RES_PREVIEW].size_h +
				     s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_l) *
				    s5k3e2fx_ctrl->fps_divider /
				    0x400) & 0xFF00) >> 8);
	if (rc < 0)
		goto set_fps_done;

	rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
				  REG_FRAME_LENGTH_LINES_LSB,
				  (((s5k3e2fx_reg_pat[S_RES_PREVIEW].size_h +
				     s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_l) *
				    s5k3e2fx_ctrl->fps_divider /
				    0x400) & 0xFF00));

set_fps_done:
	return rc;
}

static int s5k3e2fx_write_exp_gain(uint16_t gain, uint32_t line)
{
	int rc = 0;

	uint16_t max_legal_gain = 0x0200;
	uint32_t ll_ratio;	/* Q10 */
	uint32_t ll_pck, fl_lines;
	uint16_t offset = 4;
	uint32_t gain_msb, gain_lsb;
	uint32_t intg_t_msb, intg_t_lsb;
	uint32_t ll_pck_msb, ll_pck_lsb;

	struct s5k3e2fx_i2c_reg_conf tbl[2];

	CDBG("Line:%d s5k3e2fx_write_exp_gain gain %d line %d\n",
		__LINE__, gain, line);

	if (s5k3e2fx_ctrl->sensormode == SENSOR_PREVIEW_MODE) {

		s5k3e2fx_ctrl->my_reg_gain = gain;
		s5k3e2fx_ctrl->my_reg_line_count = (uint16_t) line;

		fl_lines = s5k3e2fx_reg_pat[S_RES_PREVIEW].size_h +
		    s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_l;

		ll_pck = s5k3e2fx_reg_pat[S_RES_PREVIEW].size_w +
		    s5k3e2fx_reg_pat[S_RES_PREVIEW].blk_p;

	} else {

		fl_lines = s5k3e2fx_reg_pat[S_RES_CAPTURE].size_h +
		    s5k3e2fx_reg_pat[S_RES_CAPTURE].blk_l;

		ll_pck = s5k3e2fx_reg_pat[S_RES_CAPTURE].size_w +
		    s5k3e2fx_reg_pat[S_RES_CAPTURE].blk_p;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	/* in Q10 */
	line = (line * s5k3e2fx_ctrl->fps_divider);

	if (fl_lines < (line / 0x400))
		ll_ratio = (line / (fl_lines - offset));
	else
		ll_ratio = 0x400;

/* solve greenish: only release for preview */
	if (s5k3e2fx_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
					  REG_GROUPED_PARAMETER_HOLD,
					  GROUPED_PARAMETER_HOLD);
		if (rc < 0) {
			pr_err("s5k3e2fx_i2c_write_b failed on line %d\n",
				__LINE__);
			return rc;
		}
	}

	/* update gain registers */
	gain_msb = (gain & 0xFF00) >> 8;
	gain_lsb = gain & 0x00FF;
	tbl[0].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB;
	tbl[0].bdata = gain_msb;
	tbl[1].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB;
	tbl[1].bdata = gain_lsb;
	rc = s5k3e2fx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));
	if (rc < 0)
		goto write_gain_done;
#if 1 /* Solve EVT5 greenish in lowlight*/
	ll_pck = ll_pck * ll_ratio;
	ll_pck_msb = ((ll_pck / 0x400) & 0xFF00) >> 8;
	ll_pck_lsb = (ll_pck / 0x400) & 0x00FF;
	tbl[0].waddr = REG_LINE_LENGTH_PCK_MSB;
	tbl[0].bdata = ll_pck_msb;
	tbl[1].waddr = REG_LINE_LENGTH_PCK_LSB;
	tbl[1].bdata = ll_pck_lsb;

	rc = s5k3e2fx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));
	if (rc < 0)
		goto write_gain_done;
#else
	if (line / 0x400 + offset > fl_lines)
		ll_pck = line / 0x400 + offset;
	else
		ll_pck = fl_lines;

	ll_pck_msb = ((ll_pck) & 0xFF00) >> 8;
	ll_pck_lsb = (ll_pck) & 0x00FF;
	tbl[0].waddr = REG_FRAME_LENGTH_LINES_MSB;
	tbl[0].bdata = ll_pck_msb;
	tbl[1].waddr = REG_FRAME_LENGTH_LINES_LSB;
	tbl[1].bdata = ll_pck_lsb;

	rc = s5k3e2fx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));
	if (rc < 0)
		goto write_gain_done;
#endif

	line = line / 0x400;
	intg_t_msb = (line & 0xFF00) >> 8;
	intg_t_lsb = (line & 0x00FF);
	tbl[0].waddr = REG_COARSE_INTEGRATION_TIME;
	tbl[0].bdata = intg_t_msb;
	tbl[1].waddr = REG_COARSE_INTEGRATION_TIME_LSB;
	tbl[1].bdata = intg_t_lsb;
	rc = s5k3e2fx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));

/* solve greenish: release for both */
		rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
					  REG_GROUPED_PARAMETER_HOLD,
					  GROUPED_PARAMETER_UPDATE);
		if (rc < 0) {
			pr_err("s5k3e2fx_i2c_write_b failed on line %d\n",
				__LINE__);
			return rc;
		}

write_gain_done:
	return rc;
}

static int s5k3e2fx_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	pr_info("s5k3e2fx_set_pict_exp_gain gain %d line %d\n",
		gain, line);

	return s5k3e2fx_write_exp_gain(gain, line);
}

static int s5k3e2fx_video_config(int mode, int res)
{
	int rc;

	switch (res) {
	case S_QTR_SIZE:
		pr_info("start sensor S_RES_PREVIEW config: %d\n", __LINE__);
		rc = s5k3e2fx_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
		if (rc < 0)
			return rc;
		/* only apply my_reg for returning preview*/
		rc = s5k3e2fx_write_exp_gain(s5k3e2fx_ctrl->my_reg_gain,
				     s5k3e2fx_ctrl->my_reg_line_count);
		break;

	case S_FULL_SIZE:
		rc = s5k3e2fx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
		if (rc < 0)
			return rc;
		break;

	default:
		return 0;
	}

	s5k3e2fx_ctrl->prev_res = res;
	s5k3e2fx_ctrl->curr_res = res;
	s5k3e2fx_ctrl->sensormode = mode;

	return rc;
}

static int s5k3e2fx_set_default_focus(void)
{
	int rc = 0;

	rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3131, 0);
	if (rc < 0)
		return rc;

	rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3132, 0);
	if (rc < 0)
		return rc;

	s5k3e2fx_ctrl->curr_lens_pos = 0;

	return rc;
}

static int s5k3e2fx_move_focus(int direction, int num_steps)
{
	int rc = 0;
	int i;
	int16_t step_direction;
	int16_t actual_step;
	int16_t next_pos, pos_offset;
	int16_t init_code = 0;
	uint8_t next_pos_msb, next_pos_lsb;
	int16_t s_move[5];
	uint32_t gain;		/* Q10 format */

	if (direction == MOVE_NEAR)
		step_direction = 20;
	else if (direction == MOVE_FAR)
		step_direction = -20;
	else {
		pr_err("s5k3e2fx_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	actual_step = step_direction * (int16_t) num_steps;
	pos_offset = init_code + s5k3e2fx_ctrl->curr_lens_pos;
	gain = actual_step * 0x400 / 5;

	for (i = 0; i <= 4; i++) {
		if (actual_step >= 0)
			s_move[i] =
			    ((((i + 1) * gain + 0x200) -
			      (i * gain + 0x200)) / 0x400);
		else
			s_move[i] =
			    ((((i + 1) * gain - 0x200) -
			      (i * gain - 0x200)) / 0x400);
	}

	/* Ring Damping Code */
	for (i = 0; i <= 4; i++) {
		next_pos = (int16_t) (pos_offset + s_move[i]);

		if (next_pos > (738 + init_code))
			next_pos = 738 + init_code;
		else if (next_pos < 0)
			next_pos = 0;

		CDBG("next_position in damping mode = %d\n", next_pos);
		/* Writing the Values to the actuator */
		if (next_pos == init_code)
			next_pos = 0x00;

		next_pos_msb = next_pos >> 8;
		next_pos_lsb = next_pos & 0x00FF;

		rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3131,
					  next_pos_msb);
		if (rc < 0)
			break;

		rc = s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3132,
					  next_pos_lsb);
		if (rc < 0)
			break;

		pos_offset = next_pos;
		s5k3e2fx_ctrl->curr_lens_pos = pos_offset - init_code;
		if (num_steps > 1)
			mdelay(6);
		else
			mdelay(4);
	}

	return rc;
}

static int s5k3e2fx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	CDBG("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		s5k3e2fx_get_pict_fps(cdata.cfg.gfps.prevfps,
				      &(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp, &cdata,
				 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = s5k3e2fx_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = s5k3e2fx_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = s5k3e2fx_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl = s5k3e2fx_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc = s5k3e2fx_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = s5k3e2fx_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc = s5k3e2fx_write_exp_gain(cdata.cfg.exp_gain.gain,
					     cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		rc = s5k3e2fx_set_pict_exp_gain(cdata.cfg.exp_gain.gain,
						cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc = s5k3e2fx_video_config(cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = s5k3e2fx_power_down();
		break;

	case CFG_MOVE_FOCUS:
		rc = s5k3e2fx_move_focus(cdata.cfg.focus.dir,
					 cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = s5k3e2fx_set_default_focus();
		break;

/*	case CFG_GET_AF_MAX_STEPS: */
	case CFG_SET_EFFECT:
		rc = s5k3e2fx_set_default_focus();
		break;

	case CFG_SET_LENS_SHADING:
	default:
		rc = -EFAULT;
		break;
	}

	prevent_suspend();
	return rc;
}

static int s5k3e2fx_sensor_probe(const struct msm_camera_sensor_info *info,
				 struct msm_sensor_ctrl *s)
{
	int rc = 0;
	pr_info("%s\n", __func__);

	rc = i2c_add_driver(&s5k3e2fx_i2c_driver);
	if (rc < 0 || s5k3e2fx_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}

	msm_camio_clk_rate_set(S5K3E2FX_DEF_MCLK);
	msleep(20);

	rc = s5k3e2fx_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	/* lens correction */
	s5k3e2fx_probe_init_lens_correction(info);
	init_suspend();

	s->s_init = s5k3e2fx_sensor_open_init;
	s->s_release = s5k3e2fx_sensor_release;
	s->s_config = s5k3e2fx_sensor_config;

	return rc;

probe_fail:
	pr_err("SENSOR PROBE FAILS!\n");
	return rc;
}

static int s5k3e2fx_suspend(struct platform_device *pdev, pm_message_t state)
{
	int rc;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;

	if (!sinfo->need_suspend)
		return 0;

	CDBG("s5k3e2fx: camera suspend\n");
	rc = gpio_request(sinfo->sensor_reset, "s5k3e2fx");
	if (!rc)
		gpio_direction_output(sinfo->sensor_reset, 0);
	else {
		pr_err("s5k3e2fx: request GPIO(sensor_reset) :%d faile\n",
			sinfo->sensor_reset);
		goto suspend_fail;
	}
	CDBG("s5k3e2fx: gpio_free:%d line:%d\n", sinfo->sensor_reset,
		__LINE__);
	gpio_free(sinfo->sensor_reset);

suspend_fail:
	return rc;
}
static void s5k3e2fx_sensor_resume_setting(void)
{
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			     S5K3E2FX_REG_SOFTWARE_RESET,
			     S5K3E2FX_SOFTWARE_RESET);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0100, 0x00);
	/*--------------PLL setting for 80Mhz*/
	/* PLL setting */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0305, 0x06);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0306, 0x00);
	/*88 54.4Mhz */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0307, 0x83);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0301, 0x08);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0303, 0x01);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0309, 0x08);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x030b, 0x01);
	/*--------------output size*/
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x034c, 0x05);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x034d, 0x10);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x034e, 0x03);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x034f, 0xcc);
	/*--------------frame format (min blanking)*/
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0340, 0x03);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0341, 0xe2);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0342, 0x0a);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0343, 0xac);
	/*--------------Binning */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0381, 0x01);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0383, 0x01);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0385, 0x01);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0387, 0x03);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3014, 0x06);
	/*--------------MSR setting*/
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x30c4, 0x01);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3000, 0x03);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3001, 0x94);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3002, 0x02);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3003, 0x95);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3004, 0x0f);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3005, 0x05);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3006, 0x3c);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3007, 0x8c);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3008, 0x93);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3009, 0x05);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x300a, 0x3a);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x300c, 0x02);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x300d, 0x3e);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x300f, 0x0e);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3010, 0x46);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3011, 0x64);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3012, 0x1e);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x301d, 0x3f);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3024, 0x04);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3028, 0x40);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3070, 0xdf);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x301b, 0x73);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x307e, 0x02);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x30bd, 0x06);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x30c2, 0x0b);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x30ac, 0x81);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3151, 0xe6);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3029, 0x02);
	/*--------------EVT4 setting*/
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x30bf, 0x00);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3022, 0x87);
	/*tune ADC to got batter yield rate in EDS */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3019, 0x60);
	/*AF driving strength */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3146, 0x3c);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3152, 0x08);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x315a, 0xaa);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3159, 0x0a);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0205, 0x80);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0202, 0x03);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x0200, 0x02);
}
static int s5k3e2fx_resume(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;

	if (!sinfo->need_suspend)
		return 0;

	CDBG("s5k3e2fx_resume\n");
	/*init msm,clk ,GPIO,enable */
	msm_camio_probe_on(pdev);
	msm_camio_clk_enable(CAMIO_MDC_CLK);

	CDBG("msm_camio_probe_on\n");
	/*read sensor ID and pull down reset */
	msm_camio_clk_rate_set(S5K3E2FX_DEF_MCLK);
	CDBG("msm_camio_clk_rate_set\n");
	msleep(20);
	s5k3e2fx_probe_init_sensor(sinfo);
	CDBG("s5k3e2fx_probe_init_sensor\n");
	/*init sensor,streaming on, SW init streaming off */
	s5k3e2fx_sensor_resume_setting();
	/*lens sharding */
	s5k3e2fx_probe_init_lens_correction(sinfo);
	/*stream on */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			     S5K3E2FX_REG_MODE_SELECT,
			     S5K3E2FX_MODE_SELECT_STREAM);
	/*software standby */
	msleep(25);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3130, 0x00);
	mdelay(1);
	/*stream off */
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr,
			     S5K3E2FX_REG_MODE_SELECT,
			     S5K3E2FX_MODE_SELECT_SW_STANDBY);
	mdelay(1);
	s5k3e2fx_i2c_write_b(s5k3e2fx_client->addr, 0x3150, 0x51);
	msleep(240);
	/*set RST to low */
	msm_camio_probe_off(pdev);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	CDBG("s5k3e2fx:resume done\n");
	return rc;
}

static int __s5k3e2fx_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, s5k3e2fx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k3e2fx_probe,
	.driver = {
		.name = "msm_camera_s5k3e2fx",
		.owner = THIS_MODULE,
	},
	.suspend = s5k3e2fx_suspend,
	.resume = s5k3e2fx_resume,
};

static int __init s5k3e2fx_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k3e2fx_init);
