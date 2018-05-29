/*
 * iaxxx-codec.c -- IAxxx CODEC driver
 *
 * Copyright 2017 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm1.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm2.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm3.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm4.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm5.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-stream-registers.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-channel-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pad-ctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-gpio.h>

#define IAXXX_MAX_RETRY 5
#define IAXXX_MAX_PROC 3 /* Number of procs on D1400s */
#define IAXXX_MAX_VAL 0xFFFFFFFF

#define PCM_PORT_I2S 1
#define IAXXX_MAX_PORTS		6
#define IAXXX_MAX_PDM_PORTS	7

static int iaxxx_calc_i2s_div(u32 bits_per_frame, u32 sampling_rate,
			u32 *period, u32 *div_val, u32 *nr_val);

static int iaxxx_set_i2s_cfg(struct snd_soc_dai *dai, u32 sampling_rate,
						bool is_pseudo, int id);

static int iaxxx_set_i2s_controller(struct snd_soc_codec *codec,
			u32 sampling_rate, bool is_pseudo, int id);

/* Plugin struct to store param id and val
 * Param ID and Param VAL registers are set as pair
 */
struct plg_param {
	u32 param_id;
	u32 param_val;
	u32 param_id_reg;
	u32 param_val_reg;
};

struct iaxxx_codec_priv {
	int is_codec_master[IAXXX_MAX_PORTS];
	int pcm_dai_fmt[IAXXX_MAX_PORTS];
	struct regmap *regmap;
	struct snd_soc_codec *codec;
	struct device *dev_parent;
	/* Add entry for plg_param struct for each proc
	 * param id and param val registers are set as pair
	 */
	struct plg_param plugin_param[IAXXX_MAX_PROC];
	u32 is_ip_port_master[IAXXX_MAX_PDM_PORTS];
	u32 pcm_port_fmt[IAXXX_MAX_PORTS];
	u32 ip_pdm_clk_src[IAXXX_MAX_PDM_PORTS];

	u32 port_start_en[IAXXX_MAX_PDM_PORTS];
	bool port_filter[IAXXX_MAX_PDM_PORTS];
	bool port_pcm_start[IAXXX_MAX_PORTS];
	bool port_pcm_setup[IAXXX_MAX_PORTS];
	bool plugin_blk_en[32];
	bool stream_en[32];
	u32 op_channels_active;
	/* pdm mic enable flags*/
	u32 portb_mic0_en;
	u32 portb_mic1_en;
	u32 portb_mic2_en;
	u32 portb_mic3_en;
	u32 portc_mic0_en;
	u32 portc_mic1_en;
	u32 portc_mic2_en;
	u32 portc_mic3_en;
	u32 cdc0_mic0_en;
	u32 cdc0_mic1_en;
	u32 cdc0_mic2_en;
	u32 cdc1_mic0_en;
	bool portb_micbias_en;
	bool portc_micbias_en;
	bool cdc0_micbias_en;
	bool cdc1_micbias_en;
	/* pdm bclk and aclk flags for port configuration*/
	u32 pdm_bclk;
	u32 pdm_aclk;

	u32 head_of_strm_all;
	u32 cdc_dmic_enable;
	int is_stream_in_use[IAXXX_MAX_PORTS][2];
};

static const u32 cic_rx_addr[] = {
	IAXXX_CNR0_CIC_RX_0_1_ADDR,
	IAXXX_CNR0_CIC_RX_0_1_ADDR,
	IAXXX_CNR0_CIC_RX_2_3_ADDR,
	IAXXX_CNR0_CIC_RX_2_3_ADDR,
	IAXXX_CNR0_CIC_RX_4_5_ADDR,
	IAXXX_CNR0_CIC_RX_4_5_ADDR,
	IAXXX_CNR0_CIC_RX_6_7_ADDR,
	IAXXX_CNR0_CIC_RX_6_7_ADDR,
};

static const u32 cic_rx_clr_mask[] = {
	IAXXX_CNR0_CIC_RX_0_1_CLR_0_MASK,
	IAXXX_CNR0_CIC_RX_0_1_CLR_1_MASK,
	IAXXX_CNR0_CIC_RX_2_3_CLR_2_MASK,
	IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
	IAXXX_CNR0_CIC_RX_4_5_CLR_4_MASK,
	IAXXX_CNR0_CIC_RX_4_5_CLR_5_MASK,
	IAXXX_CNR0_CIC_RX_6_7_CLR_6_MASK,
	IAXXX_CNR0_CIC_RX_6_7_CLR_7_MASK,
};

static const u32 cic_rx_m_mask[] = {
	IAXXX_CNR0_CIC_RX_0_1_M_0_MASK,
	IAXXX_CNR0_CIC_RX_0_1_M_1_MASK,
	IAXXX_CNR0_CIC_RX_2_3_M_2_MASK,
	IAXXX_CNR0_CIC_RX_2_3_M_3_MASK,
	IAXXX_CNR0_CIC_RX_4_5_M_4_MASK,
	IAXXX_CNR0_CIC_RX_4_5_M_5_MASK,
	IAXXX_CNR0_CIC_RX_6_7_M_6_MASK,
	IAXXX_CNR0_CIC_RX_6_7_M_7_MASK,
};

static const u32 cic_rx_clr_pos[] = {
	IAXXX_CNR0_CIC_RX_0_1_CLR_0_POS,
	IAXXX_CNR0_CIC_RX_0_1_CLR_1_POS,
	IAXXX_CNR0_CIC_RX_2_3_CLR_2_POS,
	IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS,
	IAXXX_CNR0_CIC_RX_4_5_CLR_4_POS,
	IAXXX_CNR0_CIC_RX_4_5_CLR_5_POS,
	IAXXX_CNR0_CIC_RX_6_7_CLR_6_POS,
	IAXXX_CNR0_CIC_RX_6_7_CLR_7_POS,
};

static const u32 cic_rx_m_pos[] = {
	IAXXX_CNR0_CIC_RX_0_1_M_0_POS,
	IAXXX_CNR0_CIC_RX_0_1_M_1_POS,
	IAXXX_CNR0_CIC_RX_2_3_M_2_POS,
	IAXXX_CNR0_CIC_RX_2_3_M_3_POS,
	IAXXX_CNR0_CIC_RX_4_5_M_4_POS,
	IAXXX_CNR0_CIC_RX_4_5_M_5_POS,
	IAXXX_CNR0_CIC_RX_6_7_M_6_POS,
	IAXXX_CNR0_CIC_RX_6_7_M_7_POS,
};

static const u32 dmic_enable_addr[] = {
	0,
	IAXXX_CNR0_DMIC1_ENABLE_ADDR,
	IAXXX_CNR0_DMIC0_ENABLE_ADDR,
	IAXXX_CNR0_CDC1_ENABLE_ADDR,
	0,
	0,
	IAXXX_CNR0_CDC0_ENABLE_ADDR,
};

static const u32 dmic_busy_addr[] = {
	0,
	IAXXX_CNR0_DMIC1_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_DMIC0_ENABLE_BUSY_ADDR,
	IAXXX_CNR0_CDC1_ENABLE_BUSY_ADDR,
	0,
	0,
	IAXXX_CNR0_CDC0_ENABLE_BUSY_ADDR,
};

enum {
	RX_0 = 0,
	RX_1,
	RX_2,
	RX_3,
	RX_4,
	RX_5,
	RX_6,
	RX_7,
	RX_8,
	RX_9,
	RX_10,
	RX_11,
	RX_12,
	RX_13,
	RX_14,
	RX_15,
	TX_0 = 16,
	TX_1,
	TX_2,
	TX_3,
	TX_4,
	TX_5,
	TX_6,
	TX_7,
	TX_8,
	TX_9,
	TX_10,
	TX_11,
	TX_12,
	TX_13,
	TX_14,
	TX_15,
};

enum {
	PCM_PORTA = 0,
	PCM_PORTB = 1,
	PCM_PORTC = 2,
	PCM_PORTD = 3,
	PCM_PORTE = 4,
	PCM_PORTF = 5,
};

enum {
	PCM_SLAVE = 0,
	PCM_MASTER = 1,

};

enum {
    /* Digital PDM Mic In*/
	PDM_DMIC_IN0 = 0,  /*!< PDM DMIC Input0 */
	PDM_DMIC_IN1,      /*!< PDM DMIC Input1 */
	PDM_DMIC_IN2,      /*!< PDM DMIC Input2 */
	PDM_DMIC_IN3,      /*!< PDM DMIC Input3 */
	PDM_DMIC_IN4,      /*!< PDM DMIC Input4 */
	PDM_DMIC_IN5,      /*!< PDM DMIC Input5 */
	PDM_DMIC_IN6,      /*!< PDM DMIC Input6 */
	PDM_DMIC_IN7,      /*!< PDM DMIC Input7 */

	/* Digital PDM Spk out */
	PDM_DMIC_OUT0,     /*!< PDM DMIC Out0 */
	PDM_DMIC_OUT1,     /*!< PDM DMIC Out1 */

	/* A400-ADCs PDM In */
	PDM_CDC0_IN0,      /*!< PDM ADC In0 */
	PDM_CDC1_IN1,      /*!< PDM ADC In1 */
	PDM_CDC2_IN2,      /*!< PDM ADC In2 */
	PDM_CDC3_IN3,      /*!< PDM ADC In3 */

	PDM_CDC0_IN4,      /*!< PDM ADC In0 */
	PDM_CDC1_IN5,      /*!< PDM ADC In1 */
	PDM_CDC2_IN6,      /*!< PDM ADC In2 */
	PDM_CDC3_IN7,      /*!< PDM ADC In3 */

	/* A400-DACs PDM Out */
	PDM_CDC_DAC_OUT0,  /*!< PDM DAC Out0 */
	PDM_CDC_DAC_OUT1,  /*!< PDM DAC Out1 */

	PDM_NUM_IO_MICS
};

enum {
	PLUGIN0 = 0,
	PLUGIN1,
	PLUGIN2,
	PLUGIN3,
	PLUGIN4,
	PLUGIN5,
	PLUGIN6,
	PLUGIN7,
	PLUGIN8,
	PLUGIN9,
	PLUGIN10,
	PLUGIN11,
	PLUGIN12,
	PLUGIN13,
	PLUGIN14,
	PLUGIN15,
};

enum {
	CIC0,
	CIC1,
	CIC2,
	CIC3,
	CIC4,
	CIC5,
	CIC6,
	CIC7,
	CIC_NONE
};

enum {
	DMIC0,
	DMIC1,
	DMIC2,
	DMIC3,
	DMIC4,
	DMIC5,
	DMIC6,
	DMIC7,
};

enum {
	IAXXX_PDM_CLK_0P_512MHZ,
	IAXXX_PDM_CLK_1P_024MHZ,
	IAXXX_PDM_CLK_1P_536MHZ,
	IAXXX_PDM_CLK_2P_832MHZ,
	IAXXX_PDM_CLK_3P_072MHZ,
	IAXXX_PDM_CLK_5P_644MHZ,
	IAXXX_PDM_CLK_6P_144MHZ,
	IAXXX_PDM_CLK_NONE
};

enum {
	IAXXX_AUD_PORT_8K,
	IAXXX_AUD_PORT_12K,
	IAXXX_AUD_PORT_16K,
	IAXXX_AUD_PORT_22_05K,
	IAXXX_AUD_PORT_24K,
	IAXXX_AUD_PORT_32K,
	IAXXX_AUD_PORT_44_1K,
	IAXXX_AUD_PORT_48K,
	IAXXX_AUD_PORT_96K,
	IAXXX_AUD_PORT_192K,
	IAXXX_AUD_PORT_NONE
};

enum {
	PDM_PORT_DMIC,
	PDM_PORT_PDMO,
	PDM_PORT_ADC,
	PDM_PORT_DAC,
	PDM_PORT_MONO,
};

enum {
	/* Primary Clocks */
	IAXXX_PDM_DMIC_PORT_CLK_SRC_PORTC = 0,/*DMIC0_CLK */
	IAXXX_PDM_CDC_ADC_CLK_SRC_CDC_MCLK,/*CDC0_CLK*/
	/* Secondary Clocks */
	IAXXX_PDM_DMIC_PORT_CLK_SRC_PORTB,/*DMIC1_CLK*/
	IAXXX_PDM_CDC_ADC_CLK_SRC_PORTD,/*CDC1_CLK*/
};

struct iaxxx_cic_deci_table {
	u32 cic_dec;
	u32 hb_dec;
};

struct iaxxx_pdm_bit_cfg {
	u32 sample_rate;
	u32 bits_per_frame;
};

/* PDM port number to IO_CTRL register mapping */
static const uint32_t iaxxx_io_ctrl_data[18][2] = {
	{IAXXX_IO_CTRL_PORTC_FS_ADDR,
	IAXXX_IO_CTRL_PORTC_FS_PDM0_DI0_AND_SEL_MASK},/* PDM_DMIC_IN0 */
	{IAXXX_IO_CTRL_PORTC_FS_ADDR,
		IAXXX_IO_CTRL_PORTC_FS_PDM0_DI0_AND_SEL_MASK},/* PDM_DMIC_IN1 */
	{IAXXX_IO_CTRL_PORTC_DI_ADDR,
		IAXXX_IO_CTRL_PORTC_DI_PDM0_DI1_AND_SEL_MASK},/* PDM_DMIC_IN2 */
	{IAXXX_IO_CTRL_PORTC_DI_ADDR,
		IAXXX_IO_CTRL_PORTC_DI_PDM0_DI1_AND_SEL_MASK},/* PDM_DMIC_IN3 */
	{IAXXX_IO_CTRL_PORTB_FS_ADDR,
		IAXXX_IO_CTRL_PORTB_FS_PDM1_DI0_AND_SEL_MASK},/* PDM_DMIC_IN4 */
	{IAXXX_IO_CTRL_PORTB_FS_ADDR,
		IAXXX_IO_CTRL_PORTB_FS_PDM1_DI0_AND_SEL_MASK},/* PDM_DMIC_IN5 */
	{IAXXX_IO_CTRL_PORTB_DI_ADDR,
		IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK},/* PDM_DMIC_IN6 */
	{IAXXX_IO_CTRL_PORTB_DI_ADDR,
		IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK},/* PDM_DMIC_IN7 */
	{IAXXX_IO_CTRL_PORTC_DO_ADDR,
		IAXXX_IO_CTRL_PORTC_DO_FI_19_AND_SEL_MASK},/* PDM_DMIC_OUT0 */
	{IAXXX_IO_CTRL_PORTB_DO_ADDR,
		IAXXX_IO_CTRL_PORTB_DO_FI_15_AND_SEL_MASK},/* PDM_DMIC_OUT1 */
	{IAXXX_IO_CTRL_CDC_PDM0_ADDR,
		IAXXX_IO_CTRL_CDC_PDM0_CDC_ADC_0_AND_SEL_MASK},/*PDM_CDC0_IN0 */
	{IAXXX_IO_CTRL_CDC_PDM1_ADDR,
		IAXXX_IO_CTRL_CDC_PDM1_CDC_ADC_1_AND_SEL_MASK},/*PDM_CDC1_IN1 */
	{IAXXX_IO_CTRL_CDC_PDM2_ADDR,
		IAXXX_IO_CTRL_CDC_PDM2_CDC_ADC_2_AND_SEL_MASK},/*PDM_CDC2_IN2 */
	{IAXXX_IO_CTRL_PORTD_DI_ADDR,
		IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_MASK},/*PDM_CDC3_IN3 */
	{IAXXX_IO_CTRL_CDC_PDM0_ADDR,
		IAXXX_IO_CTRL_CDC_PDM0_CDC_ADC_0_AND_SEL_MASK},/*PDM_CDC0_IN4 */
	{IAXXX_IO_CTRL_CDC_PDM1_ADDR,
		IAXXX_IO_CTRL_CDC_PDM1_CDC_ADC_1_AND_SEL_MASK},/*PDM_CDC1_IN5 */
	{IAXXX_IO_CTRL_CDC_PDM2_ADDR,
		IAXXX_IO_CTRL_CDC_PDM2_CDC_ADC_2_AND_SEL_MASK},/*PDM_CDC2_IN6 */
	{IAXXX_IO_CTRL_PORTD_DI_ADDR,
		IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_MASK},/*PDM_CDC3_IN7 */
};

/* PDM clock to IO_CTRL register mapping */
static const uint32_t iaxxx_io_ctrl_clk[4][3] = {
		{IAXXX_IO_CTRL_PORTC_CLK_ADDR, PDM0_CLK_SLAVE, PDM0_CLK_MASTER},
		{IAXXX_IO_CTRL_CDC_MCLK_ADDR, CDC0_CLK_SLAVE, CDC0_CLK_MASTER},
		{IAXXX_IO_CTRL_PORTB_CLK_ADDR, PDM1_CLK_SLAVE, PDM1_CLK_MASTER},
		{IAXXX_IO_CTRL_PORTD_CLK_ADDR, CDC1_CLK_SLAVE, CDC1_CLK_MASTER},
};

static const uint32_t iaxxx_io_port_clk_fwd[4][2] = {
		{IAXXX_IO_CTRL_PORTC_CLK_ADDR, DMIC_CLK_FWD},
		{IAXXX_IO_CTRL_CDC_MCLK_ADDR, 0},
		{IAXXX_IO_CTRL_PORTB_CLK_ADDR, DMIC_CLK_FWD},
		{IAXXX_IO_CTRL_PORTD_CLK_ADDR, PORTD_CLK_FWD},
};

/* Table taken from FW PDM Driver */
static struct iaxxx_pdm_bit_cfg pdm_cfg[] = {
	{8000, 64},
	{16000, 64},
	{32000, 48},
	{44000, 64},
	{48000, 64},
	{88000, 64},
	{96000, 64},
};

/* This table is two dimension array of CIC decimation and Green box(Half band)
 *  values with PDM_BCLK(rows) * AUD_PORT_CLK(columns)
 */
static struct iaxxx_cic_deci_table deci_rb_enable[][IAXXX_AUD_PORT_NONE] = {
	{/* PDM_PORT_BIT_CLK_FREQ_0_512M */
	{16, 4}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{16, 2}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{8, 2}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},
	{/* PDM_PORT_BIT_CLK_FREQ_1_024M */
	{32, 4}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{16, 4}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{8, 4}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/* PDM_PORT_BIT_CLK_FREQ_1_536M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{32, 4}, /* PDM_PORT_FREQ_12K  */
	{24, 4}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{16, 4}, /* PDM_PORT_FREQ_24K  */
	{12, 4}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{16, 2}, /* PDM_PORT_FREQ_48K  */
	{8, 2}, /* PDM_PORT_FREQ_96K  */
	{8, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/* PDM_PORT_BIT_CLK_FREQ_2_8224M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{32, 4}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{0, 0}, /* PDM_PORT_FREQ_32K  */
	{16, 4}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/*PDM_PORT_BIT_CLK_FREQ_3_072M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{32, 4}, /* PDM_PORT_FREQ_24K  */
	{24, 4}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{16, 4}, /* PDM_PORT_FREQ_48K  */
	{8, 4}, /* PDM_PORT_FREQ_96K  */
	{16, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/*PDM_PORT_BIT_CLK_FREQ_5_6448M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{0, 0}, /* PDM_PORT_FREQ_32K  */
	{32, 4}, /* PDM_PORT_FREQ_44_1K  */
	{0, 0}, /* PDM_PORT_FREQ_48K  */
	{0, 0}, /* PDM_PORT_FREQ_96K  */
	{0, 0}, /* PDM_PORT_FREQ_192K */
	},

	{/* PDM_PORT_BIT_CLK_FREQ_6_144M */
	{0, 0}, /* PDM_PORT_FREQ_8K   */
	{0, 0}, /* PDM_PORT_FREQ_12K  */
	{0, 0}, /* PDM_PORT_FREQ_16K  */
	{0, 0}, /* PDM_PORT_FREQ_22_050K  */
	{0, 0}, /* PDM_PORT_FREQ_24K  */
	{0, 0}, /* PDM_PORT_FREQ_32K  */
	{0, 0}, /* PDM_PORT_FREQ_44_1K  */
	{32, 4}, /* PDM_PORT_FREQ_48K  */
	{16, 4}, /* PDM_PORT_FREQ_96K  */
	{8, 4}, /* PDM_PORT_FREQ_192K */
	},
};

struct iaxxx_i2s_div_config {
	u32 bclk;
	u32 N;
	u32 R;
	u32 HL;
	u32 period;
};

#ifdef CONFIG_IAXXX_ACLK_338640
/* I2S_ACLK_FREQ_338640*/
static struct iaxxx_i2s_div_config i2s_div_config[] = {
	/* BCLK  N  R     HL  P */
	{ 512000, 1, 4082, 12, 4 },          /* I2S_BIT_CLK_FREQ_0_512M  */
	{ 768000, 1, 4082, 8, 4 },           /* I2S_BIT_CLK_FREQ_0_768M  */
	{ 1024000, 1, 4082, 6, 4 },          /* I2S_BIT_CLK_FREQ_1_024M  */
	{ 1536000, 1, 4092, 12, 4 },          /* I2S_BIT_CLK_FREQ_1_536M  */
	{ 2048000, 1, 4094, 15, 4 },          /* I2S_BIT_CLK_FREQ_2_048M  */
	{ 2560000, 1, 4085, 3, 4},	     /* I2S_BIT_CLK_FREQ_2_560M */
	{ 3072000, 1, 4092, 6, 4 },          /* I2S_BIT_CLK_FREQ_3_072M  */
	{ 6144000, 1, 4096, 15, 4 },          /* I2S_BIT_CLK_FREQ_6_144M  */
	{ 7680000, 1, 4096, 12, 4 },          /* I2S_BIT_CLK_FREQ_7_680M  */
	{ 11520000, 1, 4096, 8, 4 },          /* I2S_BIT_CLK_FREQ_11_52M  */
};
#else
/* I2S_ACLK_FREQ_24576*/
static struct iaxxx_i2s_div_config i2s_div_config[] = {
	/* BCLK  N  R     HL  P */
	{ 512000, 1, 4096, 12, 4 },          /* I2S_BIT_CLK_FREQ_0_512M  */
	{ 768000, 1, 4096, 8, 4 },           /* I2S_BIT_CLK_FREQ_0_768M  */
	{ 1024000, 1, 4096, 6, 4 },          /* I2S_BIT_CLK_FREQ_1_024M  */
	{ 1536000, 1, 4096, 4, 4 },          /* I2S_BIT_CLK_FREQ_1_536M  */
	{ 2048000, 1, 4096, 3, 4 },          /* I2S_BIT_CLK_FREQ_2_048M  */
	{ 3072000, 1, 4096, 2, 4 },          /* I2S_BIT_CLK_FREQ_3_072M  */
	{ 4096000, 1, 4096, 1, 6 },          /* I2S_BIT_CLK_FREQ_4_096M  */
	{ 6144000, 1, 4096, 1, 4 },          /* I2S_BIT_CLK_FREQ_6_144M  */
	{ 8192000, 1, 4096, 3, 1 },          /* I2S_BIT_CLK_FREQ_8_192M  */
};
#endif

int get_decimator_val(u32 port_bclk, u32 aud_port_clk,
			u32 *cic_dec, u32 *hb_deci)
{
	int ret = -EINVAL;

	if (port_bclk >= IAXXX_PDM_CLK_NONE)
		return ret;

	if (aud_port_clk >= IAXXX_AUD_PORT_NONE)
		return ret;

	if ((cic_dec == NULL) || (hb_deci == NULL))
		return ret;

	*cic_dec = deci_rb_enable[port_bclk][aud_port_clk].cic_dec;
	if (*cic_dec == 0) {
		/* Not supported value */
		return ret;
	}
	*cic_dec = *cic_dec - 1;

	if (deci_rb_enable[port_bclk][aud_port_clk].hb_dec == 2)
		*hb_deci = 1;
	else if (deci_rb_enable[port_bclk][aud_port_clk].hb_dec == 4)
		*hb_deci = 2;
	else
		*hb_deci = 0;
	return 0;
}

static const u32 port_clk_addr[] = {
	IAXXX_IO_CTRL_PORTA_CLK_ADDR,
	IAXXX_IO_CTRL_PORTB_CLK_ADDR,
	IAXXX_IO_CTRL_PORTC_CLK_ADDR,
	IAXXX_IO_CTRL_PORTD_CLK_ADDR,
	IAXXX_IO_CTRL_PORTE_CLK_ADDR,
	IAXXX_IO_CTRL_COMMB_0_ADDR,
};

static const u32 port_fs_addr[] = {
	IAXXX_IO_CTRL_PORTA_FS_ADDR,
	IAXXX_IO_CTRL_PORTB_FS_ADDR,
	IAXXX_IO_CTRL_PORTC_FS_ADDR,
	IAXXX_IO_CTRL_PORTD_FS_ADDR,
	IAXXX_IO_CTRL_PORTE_FS_ADDR,
	IAXXX_IO_CTRL_COMMB_1_ADDR,
};

static const u32 port_di_addr[] = {
	IAXXX_IO_CTRL_PORTA_DI_ADDR,
	IAXXX_IO_CTRL_PORTB_DI_ADDR,
	IAXXX_IO_CTRL_PORTC_DI_ADDR,
	IAXXX_IO_CTRL_PORTD_DI_ADDR,
	IAXXX_IO_CTRL_PORTE_DI_ADDR,
	IAXXX_IO_CTRL_COMMB_2_ADDR,
};

static const u32 port_do_addr[] = {
	IAXXX_IO_CTRL_PORTA_DO_ADDR,
	IAXXX_IO_CTRL_PORTB_DO_ADDR,
	IAXXX_IO_CTRL_PORTC_DO_ADDR,
	IAXXX_IO_CTRL_PORTD_DO_ADDR,
	IAXXX_IO_CTRL_PORTE_DO_ADDR,
	IAXXX_IO_CTRL_COMMB_3_ADDR,
};

static const u32 port_cdc_addr[] = {
	IAXXX_IO_CTRL_CDC_PDM0_ADDR,
	IAXXX_IO_CTRL_CDC_PDM1_ADDR,
	IAXXX_IO_CTRL_CDC_PDM2_ADDR,
	IAXXX_IO_CTRL_PORTD_DI_ADDR,
};

enum {
	IAXXX_AIF0 = 0,
	IAXXX_AIF1,
	IAXXX_AIF2,
	IAXXX_AIF3,
	IAXXX_AIF4,
	IAXXX_AIF5,
	IAXXX_NUM_CODEC_DAIS,
};

enum {
	STREAM0 = 0,
	STREAM1,
	STREAM2,
	STREAM3,
	STREAM4,
	STREAM5,
	STREAM6,
	STREAM7,
	STREAM8,
	STREAM9,
	STREAM10,
	STREAM11,
	STREAM12,
	STREAM13,
	STREAM14,
	STREAM15,
	STREAM_NONE,
};

enum Encoding_s {
	ENCODING_OPAQUE		= 0x00,
	ENCODING_AFLOAT		= 0x01,
	ENCODING_G711A		= 0x02,
	ENCODING_G711U		= 0x03,
	ENCODING_FLOAT		= 0x04,
	ENCODING_Q15		= 0x0F,
	ENCODING_Q16		= 0x10,
	ENCODING_Q17		= 0x11,
	ENCODING_Q18		= 0x12,
	ENCODING_Q19		= 0x13,
	ENCODING_Q20		= 0x14,
	ENCODING_Q21		= 0x15,
	ENCODING_Q22		= 0x16,
	ENCODING_Q23		= 0x17,
	ENCODING_Q24		= 0x18,
	ENCODING_Q25		= 0x19,
	ENCODING_Q26		= 0x1A,
	ENCODING_Q27		= 0x1B,
	ENCODING_Q28		= 0x1C,
	ENCODING_Q29		= 0x1D,
	ENCODING_Q30		= 0x1E,
	ENCODING_Q31		= 0x1F,
	ENCODING_ERROR		= 0xFFFF,
};

enum Rate_s {
	RATE_8K = 0x0,
	RATE_11P025K = 0x1,
	RATE_12K = 0x2,
	RATE_16K = 0x3,
	RATE_22P050K = 0x4,
	RATE_24K = 0x5,
	RATE_32K = 0x6,
	RATE_44P1K = 0x7,
	RATE_48K = 0x8,
	RATE_96K = 0x9,
	RATE_192K = 0xA,
	RATE_INVALID = 0xF,
};

enum Gain_Ramp_Step {
	STEP_0 = 0x0,
	STEP_300 = 0x12C,
	STEP_600 = 0x258,
	STEP_900 = 0x384,
	STEP_1200 = 0x4B0,
	STEP_1600 = 0x640,
	STEP_2000 = 0x7D0,
	STEP_INST = 0xFFFF,
};

static const char * const pdm_bclk_texts[] = {
	"IAXXX_PDM_CLK_0P_512MHZ",
	"IAXXX_PDM_CLK_1P_024MHZ",
	"IAXXX_PDM_CLK_1P_536MHZ",
	"IAXXX_PDM_CLK_2P_832MHZ",
	"IAXXX_PDM_CLK_3P_072MHZ",
	"IAXXX_PDM_CLK_5P_644MHZ",
	"IAXXX_PDM_CLK_6P_144MHZ",
	"IAXXX_PDM_CLK_NONE",
};

static const char * const pdm_aclk_texts[] = {
	"IAXXX_AUD_PORT_8K",
	"IAXXX_AUD_PORT_12K",
	"IAXXX_AUD_PORT_16K",
	"IAXXX_AUD_PORT_22_05K",
	"IAXXX_AUD_PORT_24K",
	"IAXXX_AUD_PORT_32K",
	"IAXXX_AUD_PORT_44_1K",
	"IAXXX_AUD_PORT_48K",
	"IAXXX_AUD_PORT_96K",
	"IAXXX_AUD_PORT_192K",
	"IAXXX_AUD_PORT_NONE"
};

static const struct soc_enum iaxxx_pdm_bclk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_bclk_texts),
					pdm_bclk_texts);

static const struct soc_enum iaxxx_pdm_aclk_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_aclk_texts),
					pdm_aclk_texts);

static const char * const pdm_clr_texts[] = {
	"NONE",
	"PDM_DMIC_IN0",
	"PDM_DMIC_IN1",
	"PDM_DMIC_IN2",
	"PDM_DMIC_IN3",
	"PDM_DMIC_IN4",
	"PDM_DMIC_IN5",
	"PDM_DMIC_IN6",
	"PDM_DMIC_IN7",
	"PDM_DMIC_OUT0",
	"PDM_DMIC_OUT1",
	"PDM_CDC0_IN0",
	"PDM_CDC1_IN1",
	"PDM_CDC2_IN2",
	"PDM_CDC3_IN3",
	"PDM_CDC0_IN4",
	"PDM_CDC1_IN5",
	"PDM_CDC2_IN6",
	"PDM_CDC3_IN7",
	"PDM_CDC_DAC_OUT0",
	"PDM_CDC_DAC_OUT1",
};
static const struct soc_enum iaxxx_pdm_clr_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(pdm_clr_texts),
					pdm_clr_texts);

static const char * const port_clk_stop_texts[] = {
	"PORTA", "PORTB", "PORTC", "PORTD", "PORTE", "PORTF", "PORTCDC",
};
static const struct soc_enum iaxxx_port_clk_stop_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(port_clk_stop_texts),
						port_clk_stop_texts);

#define ENUM_NAME(NAME) (#NAME)

/* Port values are 16 bit systemID's of Ports
 * 15-12 bits are Pheripheral type which is 1
 * 11-7 bits are which pheripheral type like PCM, PDM
 * 6-0 bits are instance index of port.
 */
static const unsigned int io_port_value[] = {
	 0x0,
	IAXXX_SYSID_PCM0, IAXXX_SYSID_PCM1, IAXXX_SYSID_PCM2,
	IAXXX_SYSID_PCM3, IAXXX_SYSID_PCM4, IAXXX_SYSID_PCM5,
	IAXXX_SYSID_INVALID,
	IAXXX_SYSID_PDMI0, IAXXX_SYSID_PDMI1, IAXXX_SYSID_PDMI2,
	IAXXX_SYSID_PDMI3, IAXXX_SYSID_PDMI4, IAXXX_SYSID_PDMI5,
	IAXXX_SYSID_PDMI6, IAXXX_SYSID_PDMI7, IAXXX_SYSID_PDMO0,
	IAXXX_SYSID_ADC0_IN0, IAXXX_SYSID_ADC1_IN1,
	IAXXX_SYSID_ADC2_IN2, IAXXX_SYSID_ADC3_IN3,
	IAXXX_SYSID_ADC0_IN4, IAXXX_SYSID_ADC1_IN5,
	IAXXX_SYSID_ADC2_IN6, IAXXX_SYSID_ADC3_IN7,
};

static const char * const io_port_texts[] = {
	"NONE",
	"PCM0", "PCM1", "PCM2", "PCM3", "PCM4", "PCM5",
	"SWR",
	"PDMI0", "PDMI1", "PDMI2", "PDMI3",
	"PDMI4", "PDMI5", "PDMI6", "PDMI7",
	"PDMO0",
	"ADC_IN0", "ADC_IN1", "ADC_IN2", "ADC_IN3",
	"ADC_IN4", "ADC_IN5", "ADC_IN6", "ADC_IN7",
};

static const unsigned int str_id_rx_values[] = {0x0, 0x1,
	0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0xFFFF };

static const unsigned int str_id_tx_values[] = { 0x0, 0x8, 0x9,
	0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0xFFFF };

static const char * const str_id_rx_texts[] = {
	ENUM_NAME(STREAMID_0), ENUM_NAME(STREAMID_1), ENUM_NAME(STREAMID_2),
	ENUM_NAME(STREAMID_3), ENUM_NAME(STREAMID_4), ENUM_NAME(STREAMID_5),
	ENUM_NAME(STREAMID_6), ENUM_NAME(STREAMID_7), ENUM_NAME(STREAMID_NONE),
	};

static const char * const str_id_tx_texts[] = {
	ENUM_NAME(STREAMID_NONE),
	ENUM_NAME(STREAMID_8), ENUM_NAME(STREAMID_9), ENUM_NAME(STREAMID_10),
	ENUM_NAME(STREAMID_11), ENUM_NAME(STREAMID_12), ENUM_NAME(STREAMID_13),
	ENUM_NAME(STREAMID_14), ENUM_NAME(STREAMID_15),
	ENUM_NAME(STREAMID_NONE), };

static const unsigned int strm_ch_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xF, };

static const char * const strm_ch_idx_texts[] = {
	ENUM_NAME(STRM_CH0), ENUM_NAME(STRM_CH1), ENUM_NAME(STRM_CH2),
	ENUM_NAME(STRM_CH3), ENUM_NAME(STRM_CH4), ENUM_NAME(STRM_CH5),
	ENUM_NAME(STRM_CH6), ENUM_NAME(STRM_CH7), ENUM_NAME(STRM_CH8),
	ENUM_NAME(STRM_CH9), ENUM_NAME(STRM_CH10), ENUM_NAME(STRM_CH11),
	ENUM_NAME(STRM_CH_NONE), };

static const unsigned int port_ch_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC,
	0xD, 0xE, 0xF, };

static const char * const port_ch_idx_texts[] = {
	ENUM_NAME(PORT_CH0), ENUM_NAME(PORT_CH1), ENUM_NAME(PORT_CH2),
	ENUM_NAME(PORT_CH3), ENUM_NAME(PORT_CH4), ENUM_NAME(PORT_CH5),
	ENUM_NAME(PORT_CH6), ENUM_NAME(PORT_CH7), ENUM_NAME(PORT_CH8),
	ENUM_NAME(PORT_CH9), ENUM_NAME(PORT_CH10), ENUM_NAME(PORT_CH11),
	ENUM_NAME(PORT_CH12), ENUM_NAME(PORT_CH13), ENUM_NAME(PORT_CH14),
	ENUM_NAME(PORT_CH15),};

static const unsigned int gain_ramp_value[] = {
	STEP_0,
	STEP_300, STEP_600, STEP_900, STEP_1200,
	STEP_1600, STEP_2000, STEP_INST, };

static const char * const gain_ramp_texts[] = {
	ENUM_NAME(STEP_0),
	ENUM_NAME(STEP_300), ENUM_NAME(STEP_600), ENUM_NAME(STEP_900),
	ENUM_NAME(STEP_1200), ENUM_NAME(STEP_1600), ENUM_NAME(STEP_2000),
	ENUM_NAME(STEP_INST), };

static const unsigned int gain_rech_evnt_value[] = {
	0x0, 0x1,
};

static const char * const gain_rech_evnt_texts[] = {
	"off", "on",
};

static const unsigned int str_frm_len_values[] = {
	0x0,
	0x20, 0x40, 0xA0,
	0x140, 0x200, 0x280,
	0x640, 0xC80, 0x1900,

	0x40, 0x80, 0x140,
	0x280, 0x400, 0x500,
	0xC80, 0x1900, 0x3200,

	0x60, 0xC0, 0x1E0,
	0x3C0, 0x600, 0x780,
	0x12C0, 0x2580, 0x4B00,

	0x80, 0x100, 0x280,
	0x500, 0x800, 0xA00,
	0x1900, 0x3200, 0x6400,

	0xB0, 0x160, 0x372,
	0x6E0, 0xB00, 0xDC0,
	0x2260, 0x44C0, 0x8980,

	0xC0, 0x180, 0x3C0,
	0x780, 0xC00, 0xF00,
	0x2580, 0x4B00, 0x9600,

	0x180, 0x300, 0x780,
	0xF00, 0x1800, 0x1E00,
	0x4B00, 0x9600,

	0x300, 0x600, 0xF00,
	0x1E00, 0x3000, 0x3C00,
	0x9600,
};

static const char * const str_frm_len_text[] = {
	ENUM_NAME(NONE),
	ENUM_NAME(8K_1MS), ENUM_NAME(8K_2MS), ENUM_NAME(8K_5MS),
	ENUM_NAME(8K_10MS), ENUM_NAME(8K_16MS), ENUM_NAME(8K_20MS),
	ENUM_NAME(8K_50MS), ENUM_NAME(8K_100MS), ENUM_NAME(8K_200MS),

	ENUM_NAME(16K_1MS), ENUM_NAME(16K_2MS), ENUM_NAME(16K_5MS),
	ENUM_NAME(16K_10MS), ENUM_NAME(16K_16MS), ENUM_NAME(16K_20MS),
	ENUM_NAME(16K_50MS), ENUM_NAME(16K_100MS), ENUM_NAME(16K_200MS),

	ENUM_NAME(24K_1MS), ENUM_NAME(24K_2MS), ENUM_NAME(24K_5MS),
	ENUM_NAME(24K_10MS), ENUM_NAME(24K_16MS), ENUM_NAME(24K_20MS),
	ENUM_NAME(24K_50MS), ENUM_NAME(24K_100MS), ENUM_NAME(24K_200MS),

	ENUM_NAME(32K_1MS), ENUM_NAME(32K_2MS), ENUM_NAME(32K_5MS),
	ENUM_NAME(32K_10MS), ENUM_NAME(32K_16MS), ENUM_NAME(32K_20MS),
	ENUM_NAME(32K_50MS), ENUM_NAME(32K_100MS), ENUM_NAME(32K_200MS),

	ENUM_NAME(44.1K_1MS), ENUM_NAME(44.1K_2MS), ENUM_NAME(44.1K_5MS),
	ENUM_NAME(44.1K_10MS), ENUM_NAME(44.1K_16MS), ENUM_NAME(44.1K_20MS),
	ENUM_NAME(44.1K_50MS), ENUM_NAME(44.1K_100MS), ENUM_NAME(44.1K_200MS),

	ENUM_NAME(48K_1MS), ENUM_NAME(48K_2MS), ENUM_NAME(48K_5MS),
	ENUM_NAME(48K_10MS), ENUM_NAME(48K_16MS), ENUM_NAME(48K_20MS),
	ENUM_NAME(48K_50MS), ENUM_NAME(48K_100MS), ENUM_NAME(48K_200MS),

	ENUM_NAME(96K_1MS), ENUM_NAME(96K_2MS), ENUM_NAME(96K_5MS),
	ENUM_NAME(96K_10MS), ENUM_NAME(96K_16MS), ENUM_NAME(96K_20MS),
	ENUM_NAME(96K_50MS), ENUM_NAME(96K_100MS),

	ENUM_NAME(192K_1MS), ENUM_NAME(192K_2MS), ENUM_NAME(192K_5MS),
	ENUM_NAME(192K_10MS), ENUM_NAME(192K_16MS), ENUM_NAME(192K_20MS),
	ENUM_NAME(192K_50MS),
};

static const char * const str_rate_text[] = {
	ENUM_NAME(RATE_8K),
	ENUM_NAME(RATE_11P025K),
	ENUM_NAME(RATE_12K),
	ENUM_NAME(RATE_16K),
	ENUM_NAME(RATE_22P050K),
	ENUM_NAME(RATE_24K),
	ENUM_NAME(RATE_32K),
	ENUM_NAME(RATE_44P1K),
	ENUM_NAME(RATE_48K),
	ENUM_NAME(RATE_96K),
	ENUM_NAME(RATE_192K),
	ENUM_NAME(RATE_INVALID),
};

static const unsigned int str_rate_values[] = {
	RATE_8K,
	RATE_11P025K,
	RATE_12K,
	RATE_16K,
	RATE_22P050K,
	RATE_24K,
	RATE_32K,
	RATE_44P1K,
	RATE_48K,
	RATE_96K,
	RATE_192K,
	RATE_INVALID,
};

/* supported stream encodings */
static const unsigned int str_enc_values[] = {
	ENCODING_OPAQUE,
	ENCODING_AFLOAT,
	ENCODING_G711A,
	ENCODING_G711U,
	ENCODING_FLOAT,
	ENCODING_Q15,
	ENCODING_Q16,
	ENCODING_Q17,
	ENCODING_Q18,
	ENCODING_Q19,
	ENCODING_Q20,
	ENCODING_Q21,
	ENCODING_Q22,
	ENCODING_Q23,
	ENCODING_Q24,
	ENCODING_Q25,
	ENCODING_Q26,
	ENCODING_Q27,
	ENCODING_Q28,
	ENCODING_Q29,
	ENCODING_Q30,
	ENCODING_Q31,
	ENCODING_ERROR,
};

static const char * const str_enc_text[] = {
	ENUM_NAME(ENCODING_OPAQUE),
	ENUM_NAME(ENCODING_AFLOAT),
	ENUM_NAME(ENCODING_G711A),
	ENUM_NAME(ENCODING_G711U),
	ENUM_NAME(ENCODING_FLOAT),
	ENUM_NAME(ENCODING_Q15),
	ENUM_NAME(ENCODING_Q16),
	ENUM_NAME(ENCODING_Q17),
	ENUM_NAME(ENCODING_Q18),
	ENUM_NAME(ENCODING_Q19),
	ENUM_NAME(ENCODING_Q20),
	ENUM_NAME(ENCODING_Q21),
	ENUM_NAME(ENCODING_Q22),
	ENUM_NAME(ENCODING_Q23),
	ENUM_NAME(ENCODING_Q24),
	ENUM_NAME(ENCODING_Q25),
	ENUM_NAME(ENCODING_Q26),
	ENUM_NAME(ENCODING_Q27),
	ENUM_NAME(ENCODING_Q28),
	ENUM_NAME(ENCODING_Q29),
	ENUM_NAME(ENCODING_Q30),
	ENUM_NAME(ENCODING_Q31),
	ENUM_NAME(ENCODING_ERROR),
};

#define IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(channel, channel_name) \
static const char * const channel##_rx_off_on_texts[] = { \
	"Off", \
	"Rx0"channel_name"On", "Rx1"channel_name"On", \
	"Rx2"channel_name"On", "Rx3"channel_name"On", \
	"Rx4"channel_name"On", "Rx5"channel_name"On", \
	"Rx6"channel_name"On", "Rx7"channel_name"On", \
	"Rx8"channel_name"On", "Rx9"channel_name"On", \
	"Rx10"channel_name"On", "Rx11"channel_name"On", \
	"Rx12"channel_name"On", "Rx13"channel_name"On", \
	"Rx14"channel_name"On", "Rx15"channel_name"On", \
	"Plgin0"channel_name"On", "Plgin1"channel_name"On", \
	"Plgin2"channel_name"On", "Plgin3"channel_name"On", \
	"Plgin4"channel_name"On", "Plgin5"channel_name"On", \
	"Plgin6"channel_name"On", "Plgin7"channel_name"On", \
	"Plgin8"channel_name"On", "Plgin9"channel_name"On", \
	"Plgin10"channel_name"On", "Plgin11"channel_name"On", \
	"Plgin12"channel_name"On", "Plgin13"channel_name"On", \
	"Plgin14"channel_name"On", "Plgin15"channel_name"On", \
}

IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_0, "Tx0");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_1, "Tx1");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_2, "Tx2");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_3, "Tx3");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_4, "Tx4");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_5, "Tx5");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_6, "Tx6");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_7, "Tx7");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_8, "Tx8");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_9, "Tx9");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_10, "Tx10");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_11, "Tx11");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_12, "Tx12");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_13, "Tx13");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_14, "Tx14");
IAXXX_INPUT_TO_TX_CH_ON_OFF_TEXTS(TX_15, "Tx15");

static const char * const iaxxx_route_status_texts[] = {
	"InActive", "Active"
};

static const struct soc_enum iaxxx_route_status_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(iaxxx_route_status_texts),
			iaxxx_route_status_texts);

static int iaxxx_get_pdm_bclk(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->pdm_bclk;
	return 0;
}

static int iaxxx_put_pdm_bclk(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	iaxxx->pdm_bclk = ucontrol->value.enumerated.item[0];
	return 0;
}

static int iaxxx_get_pdm_aclk(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.enumerated.item[0] = iaxxx->pdm_aclk;
	return 0;
}

static int iaxxx_put_pdm_aclk(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	iaxxx->pdm_aclk = ucontrol->value.enumerated.item[0];
	return 0;
}

#define IAXXX_UPDATE_BLOCK_SET_GET(blk_name, block) \
static int iaxxx_put_update_##blk_name(struct snd_kcontrol *kcontrol, \
				  struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	u32 status = 0; \
	int ret = 0; \
	pr_debug("enter %s connection\n", __func__); \
	if (ucontrol->value.enumerated.item[0]) { \
		ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
						&status, block); \
	} \
	return ret; \
} \
static int iaxxx_get_update_##blk_name(struct snd_kcontrol *kcontrol, \
				 struct snd_ctl_elem_value *ucontrol) \
{ \
	return 0; \
}
IAXXX_UPDATE_BLOCK_SET_GET(block0, IAXXX_BLOCK_0)
IAXXX_UPDATE_BLOCK_SET_GET(block1, IAXXX_BLOCK_1)
IAXXX_UPDATE_BLOCK_SET_GET(block2, IAXXX_BLOCK_2)

static int iaxxx_put_route_status(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct device *dev = iaxxx->dev_parent;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int ret = 0;

	pr_debug("enter %s connection\n", __func__);
	if (ucontrol->value.enumerated.item[0] && priv->crash_count) {
		dev_info(dev, "Route active request after crash\n");
		ret = iaxxx_tunnel_recovery(priv);
		if (ret)
			dev_err(dev, "not able to restart tunneling\n");
	}
	priv->route_status = ucontrol->value.enumerated.item[0];

	return ret;
}

static int iaxxx_get_route_status(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent);

	ucontrol->value.enumerated.item[0] = priv->route_status;
	return 0;

}
static const DECLARE_TLV_DB_SCALE(gn_ch_ep_tlv, -1200, 100, 0);

/* Dummy CH Port for DAPM */
#define IAXXX_CH_MGR_DAPM_CTLS(channel, channel_name) \
static const struct soc_enum channel##_port_enum = \
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,\
			ARRAY_SIZE(io_port_texts), io_port_texts); \
static const struct snd_kcontrol_new channel##_port_mux =	\
	SOC_DAPM_ENUM(channel_name "Port", channel##_port_enum)

#define IAXXX_CH_MGR_DAPM_MUX(channel, channel_name) \
	SND_SOC_DAPM_MUX(channel_name " Port", SND_SOC_NOPM, 0, 0, \
						&(channel##_port_mux))

#define IAXXX_CH_RX_TO_TX_DAPM_CTLS(channel, channel_name) \
static const SOC_ENUM_SINGLE_DECL(channel##_rx_en_enum, \
			SND_SOC_NOPM, 0, channel##_rx_off_on_texts); \
static const struct snd_kcontrol_new channel##_rx_mux =	\
	SOC_DAPM_ENUM(channel_name "PortTxEn", channel##_rx_en_enum)

#define IAXXX_CH_RX_TO_TX_DAPM_MUX(channel, channel_name) \
	SND_SOC_DAPM_MUX(channel_name " PortMux En", SND_SOC_NOPM, 0, 0, \
						&(channel##_rx_mux))

#define IAXXXCORE_RX_CHMGR_ENUM(channel, num) \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_rmp_enum, \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_REG(num), \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS, \
			(IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK >> \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS), \
			gain_ramp_texts, gain_ramp_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_evnt_enum, \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_REG(num), \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS, \
			(IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK >> \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS), \
			gain_rech_evnt_texts, gain_rech_evnt_value)

IAXXXCORE_RX_CHMGR_ENUM(RX_0, 0);
IAXXXCORE_RX_CHMGR_ENUM(RX_1, 1);
IAXXXCORE_RX_CHMGR_ENUM(RX_2, 2);
IAXXXCORE_RX_CHMGR_ENUM(RX_3, 3);
IAXXXCORE_RX_CHMGR_ENUM(RX_4, 4);
IAXXXCORE_RX_CHMGR_ENUM(RX_5, 5);
IAXXXCORE_RX_CHMGR_ENUM(RX_6, 6);
IAXXXCORE_RX_CHMGR_ENUM(RX_7, 7);
IAXXXCORE_RX_CHMGR_ENUM(RX_8, 8);
IAXXXCORE_RX_CHMGR_ENUM(RX_9, 9);
IAXXXCORE_RX_CHMGR_ENUM(RX_10, 10);
IAXXXCORE_RX_CHMGR_ENUM(RX_11, 11);
IAXXXCORE_RX_CHMGR_ENUM(RX_12, 12);
IAXXXCORE_RX_CHMGR_ENUM(RX_13, 13);
IAXXXCORE_RX_CHMGR_ENUM(RX_14, 14);
IAXXXCORE_RX_CHMGR_ENUM(RX_15, 15);


#define IAXXXCORE_RX_CHMGR_KCTRL(channel, channel_name) \
	SOC_ENUM(channel_name "Chan GnRmp", channel##_gn_rmp_enum), \
	SOC_SINGLE_TLV(channel_name "Ch EpGain", \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_REG(channel), \
			IAXXX_IN_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS, \
			0xFF, 0, gn_ch_ep_tlv), \
	SOC_ENUM(channel_name "Chan GnReEvt", channel##_gn_evnt_enum), \
	SOC_SINGLE(channel_name "Chan Gain En", \
			IAXXX_CH_HDR_CH_GAIN_ADDR, channel, 1, 0)

IAXXX_CH_MGR_DAPM_CTLS(RX_0, "Rx0 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_1, "Rx1 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_2, "Rx2 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_3, "Rx3 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_4, "Rx4 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_5, "Rx5 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_6, "Rx6 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_7, "Rx7 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_8, "Rx8 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_9, "Rx9 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_10, "Rx10 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_11, "Rx11 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_12, "Rx12 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_13, "Rx13 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_14, "Rx14 Mux");
IAXXX_CH_MGR_DAPM_CTLS(RX_15, "Rx15 Mux");


static const unsigned int str_mstr_id_values[] = {0x0, 0x1,
	0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9,
	0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0xFFFF, };

static const char * const str_mstr_id_texts[] = {
	ENUM_NAME(STREAMID_0), ENUM_NAME(STREAMID_1), ENUM_NAME(STREAMID_2),
	ENUM_NAME(STREAMID_3), ENUM_NAME(STREAMID_4), ENUM_NAME(STREAMID_5),
	ENUM_NAME(STREAMID_6), ENUM_NAME(STREAMID_7), ENUM_NAME(STREAMID_8),
	ENUM_NAME(STREAMID_9), ENUM_NAME(STREAMID_10), ENUM_NAME(STREAMID_11),
	ENUM_NAME(STREAMID_12), ENUM_NAME(STREAMID_13), ENUM_NAME(STREAMID_14),
	ENUM_NAME(STREAMID_15), ENUM_NAME(STREAMID_NONE),
};

static const unsigned int strm_pwr_mode_value[] = {
	0x0, 0x1, 0x2, };

static const char * const strm_pwr_mode_texts[] = {
	ENUM_NAME(STANDARD), ENUM_NAME(LOW_POWER), ENUM_NAME(LOW_POWER_VQ), };

static const unsigned int strm_asrc_mode_value[] = {
	0x0, 0x1, 0x2, 0x3, };

static const char * const strm_asrc_mode_texts[] = {
	ENUM_NAME(ASRC_ENABLE), ENUM_NAME(ASRC_DISABLE), ENUM_NAME(REDBOX_2-1),
	ENUM_NAME(REDBOX_1-2), };

  /* stream direction */
#define IAXXXCORE_STREAM_ENUM(stream) \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_asrc_mode_enum, \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS, \
			(IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_MASK >> \
			IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS), \
			strm_asrc_mode_texts, strm_asrc_mode_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_port_enc_enum, \
			IAXXX_STR_GRP_STR_PORT_REG(stream), \
			IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_POS, \
			(IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_MASK >> \
			IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_port_enum, \
			IAXXX_STR_GRP_STR_PORT_REG(stream), \
			IAXXX_STR_GRP_STR_PORT_PORT_POS, \
			(IAXXX_STR_GRP_STR_PORT_PORT_MASK >> \
			IAXXX_STR_GRP_STR_PORT_PORT_POS), \
			io_port_texts, io_port_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_mstr_str_id_enum, \
			IAXXX_STR_GRP_STR_SYNC_REG(stream), \
			IAXXX_STR_GRP_STR_SYNC_MASTER_STR_POS, \
			(IAXXX_STR_GRP_STR_SYNC_MASTER_STR_MASK >> \
			IAXXX_STR_GRP_STR_SYNC_MASTER_STR_POS), \
			str_mstr_id_texts, str_mstr_id_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_enc_enum, \
			IAXXX_STR_GRP_STR_FORMAT_REG(stream), \
			IAXXX_STR_GRP_STR_FORMAT_ENCODING_POS, \
			(IAXXX_STR_GRP_STR_FORMAT_ENCODING_MASK >> \
			IAXXX_STR_GRP_STR_FORMAT_ENCODING_POS), \
			str_enc_text, str_enc_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_sr_enum, \
			IAXXX_STR_GRP_STR_FORMAT_REG(stream), \
			IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS, \
			(IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_MASK >> \
			IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS), \
			str_rate_text, str_rate_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(stream##_frm_len_enum, \
			IAXXX_STR_GRP_STR_FORMAT_REG(stream), \
			IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS, \
			(IAXXX_STR_GRP_STR_FORMAT_LENGTH_MASK >> \
			IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS), \
			str_frm_len_text, str_frm_len_values)

IAXXXCORE_STREAM_ENUM(STREAM0);
IAXXXCORE_STREAM_ENUM(STREAM1);
IAXXXCORE_STREAM_ENUM(STREAM2);
IAXXXCORE_STREAM_ENUM(STREAM3);
IAXXXCORE_STREAM_ENUM(STREAM4);
IAXXXCORE_STREAM_ENUM(STREAM5);
IAXXXCORE_STREAM_ENUM(STREAM6);
IAXXXCORE_STREAM_ENUM(STREAM7);
IAXXXCORE_STREAM_ENUM(STREAM8);
IAXXXCORE_STREAM_ENUM(STREAM9);
IAXXXCORE_STREAM_ENUM(STREAM10);
IAXXXCORE_STREAM_ENUM(STREAM11);
IAXXXCORE_STREAM_ENUM(STREAM12);
IAXXXCORE_STREAM_ENUM(STREAM13);
IAXXXCORE_STREAM_ENUM(STREAM14);
IAXXXCORE_STREAM_ENUM(STREAM15);

#define IAXXX_STREAM_EN_SET_GET(stream) \
static int iaxxxcore_set_strm##stream##_en( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 status = 0; \
	int ret = 0; \
	pr_debug("enter %s connection\n", __func__); \
	if (ucontrol->value.integer.value[0]) { \
		snd_soc_update_bits(codec, IAXXX_STR_HDR_STR_EN_ADDR, \
					1 << stream, 1 << stream); \
		iaxxx->stream_en[stream] = 1; \
	} else { \
		snd_soc_update_bits(codec, IAXXX_STR_HDR_STR_EN_ADDR, \
					1 << stream, 0 << stream); \
		iaxxx->stream_en[stream] = 0; \
	} \
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
				 &status, IAXXX_BLOCK_0); \
	if (ret) \
		dev_err(priv->dev, "Update blk failed %s():%u\n", \
					__func__, status); \
	return ret; \
} \
static int iaxxxcore_get_strm##stream##_en( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.enumerated.item[0] = iaxxx->stream_en[stream]; \
	return 0; \
}

IAXXX_STREAM_EN_SET_GET(STREAM0)
IAXXX_STREAM_EN_SET_GET(STREAM1)
IAXXX_STREAM_EN_SET_GET(STREAM2)
IAXXX_STREAM_EN_SET_GET(STREAM3)
IAXXX_STREAM_EN_SET_GET(STREAM4)
IAXXX_STREAM_EN_SET_GET(STREAM5)
IAXXX_STREAM_EN_SET_GET(STREAM6)
IAXXX_STREAM_EN_SET_GET(STREAM7)
IAXXX_STREAM_EN_SET_GET(STREAM8)
IAXXX_STREAM_EN_SET_GET(STREAM9)
IAXXX_STREAM_EN_SET_GET(STREAM10)
IAXXX_STREAM_EN_SET_GET(STREAM11)
IAXXX_STREAM_EN_SET_GET(STREAM12)
IAXXX_STREAM_EN_SET_GET(STREAM13)
IAXXX_STREAM_EN_SET_GET(STREAM14)
IAXXX_STREAM_EN_SET_GET(STREAM15)

#define IAXXXCORE_STREAM_KCTRL(stream, strm_name, shift) \
	SOC_SINGLE_BOOL_EXT(strm_name" En", 0, \
		       iaxxxcore_get_strm##stream##_en, \
		       iaxxxcore_set_strm##stream##_en), \
	SOC_ENUM(strm_name" Format Enc", stream##_enc_enum), \
	SOC_ENUM(strm_name" Format Sr", stream##_sr_enum), \
	SOC_ENUM(strm_name" Format FrLn", stream##_frm_len_enum), \
	SOC_ENUM(strm_name" ASRC Mode", stream##_asrc_mode_enum), \
	SOC_SINGLE(strm_name" droop comp en", \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_POS, 1, 0), \
	SOC_SINGLE(strm_name" DC block en", \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_DROOP_COMP_ENABLE_POS, 1, 0), \
	SOC_SINGLE(strm_name" tone gen en", \
			IAXXX_STR_GRP_STR_CTRL_REG(stream), \
			IAXXX_STR_GRP_STR_CTRL_TONE_GEN_ENABLE_POS, 1, 0), \
	SOC_ENUM(strm_name" Port", stream##_port_enum), \
	SOC_ENUM(strm_name" Port Enc", stream##_port_enc_enum), \
	SOC_SINGLE(strm_name" Dir", \
			IAXXX_STR_GRP_STR_PORT_REG(stream), \
			IAXXX_STR_GRP_STR_PORT_DIR_POS, 1, 0), \
	SOC_SINGLE(strm_name" CH Mask En", \
			IAXXX_STR_GRP_STR_CHN_REG(stream), \
			shift, 0xFFFF, 0), \
	SOC_ENUM(strm_name" Master Strm Id", stream##_mstr_str_id_enum), \
	SOC_SINGLE(strm_name" inter strm delay", \
				IAXXX_STR_GRP_STR_SYNC_REG(stream), \
				IAXXX_STR_GRP_STR_SYNC_INTER_STR_DELAY_POS, \
				(1<<16) - 1, 0)

static const unsigned int ip_ep_values[] = {
	IAXXX_SYSID_INVALID,
	/* Output Channels 0-15 to EndPoint-0 */
	IAXXX_SYSID_CHANNEL_RX_0_EP_0,
	IAXXX_SYSID_CHANNEL_RX_1_EP_0,
	IAXXX_SYSID_CHANNEL_RX_2_EP_0,
	IAXXX_SYSID_CHANNEL_RX_3_EP_0,
	IAXXX_SYSID_CHANNEL_RX_4_EP_0,
	IAXXX_SYSID_CHANNEL_RX_5_EP_0,
	IAXXX_SYSID_CHANNEL_RX_6_EP_0,
	IAXXX_SYSID_CHANNEL_RX_7_EP_0,
	IAXXX_SYSID_CHANNEL_RX_8_EP_0,
	IAXXX_SYSID_CHANNEL_RX_9_EP_0,
	IAXXX_SYSID_CHANNEL_RX_10_EP_0,
	IAXXX_SYSID_CHANNEL_RX_11_EP_0,
	IAXXX_SYSID_CHANNEL_RX_12_EP_0,
	IAXXX_SYSID_CHANNEL_RX_13_EP_0,
	IAXXX_SYSID_CHANNEL_RX_14_EP_0,
	IAXXX_SYSID_CHANNEL_RX_15_EP_0,

	/* Plugin 0 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_0_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_0_OUT_EP_15,

	/* Plugin 1 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_1_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_1_OUT_EP_15,

	/* Plugin 2 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_2_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_2_OUT_EP_15,

	/* Plugin 3 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_3_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_3_OUT_EP_15,

	/* Plugin 4 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_4_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_4_OUT_EP_15,

	/* Plugin 5 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_5_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_5_OUT_EP_15,

	/* Plugin 6 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_6_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_6_OUT_EP_15,

	/* Plugin 7 EndPoint 0 to 15 */
	IAXXX_SYSID_PLUGIN_7_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_7_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_8_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_8_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_9_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_9_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_10_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_10_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_11_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_11_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_12_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_12_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_13_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_13_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_14_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_14_OUT_EP_15,

	IAXXX_SYSID_PLUGIN_15_OUT_EP_0,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_1,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_2,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_3,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_4,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_5,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_6,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_7,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_8,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_9,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_10,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_11,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_12,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_13,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_14,
	IAXXX_SYSID_PLUGIN_15_OUT_EP_15,
};

static const char * const ip_ep_texts[] = {
	ENUM_NAME(UNKNOWN),
	ENUM_NAME(RX0_ChanMgr), ENUM_NAME(RX1_ChanMgr),
	ENUM_NAME(RX2_ChanMgr), ENUM_NAME(RX3_ChanMgr),
	ENUM_NAME(RX4_ChanMgr), ENUM_NAME(RX5_ChanMgr),
	ENUM_NAME(RX6_ChanMgr), ENUM_NAME(RX7_ChanMgr),
	ENUM_NAME(RX8_ChanMgr), ENUM_NAME(RX9_ChanMgr),
	ENUM_NAME(RX10_ChanMgr), ENUM_NAME(RX11_ChanMgr),
	ENUM_NAME(RX12_ChanMgr), ENUM_NAME(RX13_ChanMgr),
	ENUM_NAME(RX14_ChanMgr), ENUM_NAME(RX15_ChanMgr),

	ENUM_NAME(plugin0Out0), ENUM_NAME(plugin0Out1),
	ENUM_NAME(plugin0Out2), ENUM_NAME(plugin0Out3),
	ENUM_NAME(plugin0Out4), ENUM_NAME(plugin0Out5),
	ENUM_NAME(plugin0Out6), ENUM_NAME(plugin0Out7),
	ENUM_NAME(plugin0Out8), ENUM_NAME(plugin0Out9),
	ENUM_NAME(plugin0Out10), ENUM_NAME(plugin0Out11),
	ENUM_NAME(plugin0Out12), ENUM_NAME(plugin0Out13),
	ENUM_NAME(plugin0Out14), ENUM_NAME(plugin0Out15),
	ENUM_NAME(plugin1Out0), ENUM_NAME(plugin1Out1),
	ENUM_NAME(plugin1Out2), ENUM_NAME(plugin1Out3),
	ENUM_NAME(plugin1Out4), ENUM_NAME(plugin1Out5),
	ENUM_NAME(plugin1Out6), ENUM_NAME(plugin1Out7),
	ENUM_NAME(plugin1Out8), ENUM_NAME(plugin1Out9),
	ENUM_NAME(plugin1Out10), ENUM_NAME(plugin1Out11),
	ENUM_NAME(plugin1Out12), ENUM_NAME(plugin1Out13),
	ENUM_NAME(plugin1Out14), ENUM_NAME(plugin1Out15),
	ENUM_NAME(plugin2Out0), ENUM_NAME(plugin2Out1),
	ENUM_NAME(plugin2Out2), ENUM_NAME(plugin2Out3),
	ENUM_NAME(plugin2Out4), ENUM_NAME(plugin2Out5),
	ENUM_NAME(plugin2Out6), ENUM_NAME(plugin2Out7),
	ENUM_NAME(plugin2Out8), ENUM_NAME(plugin2Out9),
	ENUM_NAME(plugin2Out10), ENUM_NAME(plugin2Out11),
	ENUM_NAME(plugin2Out12), ENUM_NAME(plugin2Out13),
	ENUM_NAME(plugin2Out14), ENUM_NAME(plugin2Out15),
	ENUM_NAME(plugin3Out0), ENUM_NAME(plugin3Out1),
	ENUM_NAME(plugin3Out2), ENUM_NAME(plugin3Out3),
	ENUM_NAME(plugin3Out4), ENUM_NAME(plugin3Out5),
	ENUM_NAME(plugin3Out6), ENUM_NAME(plugin3Out7),
	ENUM_NAME(plugin3Out8), ENUM_NAME(plugin3Out9),
	ENUM_NAME(plugin3Out10), ENUM_NAME(plugin3Out11),
	ENUM_NAME(plugin3Out12), ENUM_NAME(plugin3Out13),
	ENUM_NAME(plugin3Out14), ENUM_NAME(plugin3Out15),
	ENUM_NAME(plugin4Out0), ENUM_NAME(plugin4Out1),
	ENUM_NAME(plugin4Out2), ENUM_NAME(plugin4Out3),
	ENUM_NAME(plugin4Out4), ENUM_NAME(plugin4Out5),
	ENUM_NAME(plugin4Out6), ENUM_NAME(plugin4Out7),
	ENUM_NAME(plugin4Out8), ENUM_NAME(plugin4Out9),
	ENUM_NAME(plugin4Out10), ENUM_NAME(plugin4Out11),
	ENUM_NAME(plugin4Out12), ENUM_NAME(plugin4Out13),
	ENUM_NAME(plugin4Out14), ENUM_NAME(plugin4Out15),
	ENUM_NAME(plugin5Out0), ENUM_NAME(plugin5Out1),
	ENUM_NAME(plugin5Out2), ENUM_NAME(plugin5Out3),
	ENUM_NAME(plugin5Out4), ENUM_NAME(plugin5Out5),
	ENUM_NAME(plugin5Out6), ENUM_NAME(plugin5Out7),
	ENUM_NAME(plugin5Out8), ENUM_NAME(plugin5Out9),
	ENUM_NAME(plugin5Out10), ENUM_NAME(plugin5Out11),
	ENUM_NAME(plugin5Out12), ENUM_NAME(plugin5Out13),
	ENUM_NAME(plugin5Out14), ENUM_NAME(plugin5Out15),
	ENUM_NAME(plugin6Out0), ENUM_NAME(plugin6Out1),
	ENUM_NAME(plugin6Out2), ENUM_NAME(plugin6Out3),
	ENUM_NAME(plugin6Out4), ENUM_NAME(plugin6Out5),
	ENUM_NAME(plugin6Out6), ENUM_NAME(plugin6Out7),
	ENUM_NAME(plugin6Out8), ENUM_NAME(plugin6Out9),
	ENUM_NAME(plugin6Out10), ENUM_NAME(plugin6Out11),
	ENUM_NAME(plugin6Out12), ENUM_NAME(plugin6Out13),
	ENUM_NAME(plugin6Out14), ENUM_NAME(plugin6Out15),
	ENUM_NAME(plugin7Out0), ENUM_NAME(plugin7Out1),
	ENUM_NAME(plugin7Out2), ENUM_NAME(plugin7Out3),
	ENUM_NAME(plugin7Out4), ENUM_NAME(plugin7Out5),
	ENUM_NAME(plugin7Out6), ENUM_NAME(plugin7Out7),
	ENUM_NAME(plugin7Out8), ENUM_NAME(plugin7Out9),
	ENUM_NAME(plugin7Out10), ENUM_NAME(plugin7Out11),
	ENUM_NAME(plugin7Out12), ENUM_NAME(plugin7Out13),
	ENUM_NAME(plugin7Out14), ENUM_NAME(plugin7Out15),
	ENUM_NAME(plugin8Out0), ENUM_NAME(plugin8Out1),
	ENUM_NAME(plugin8Out2), ENUM_NAME(plugin8Out3),
	ENUM_NAME(plugin8Out4), ENUM_NAME(plugin8Out5),
	ENUM_NAME(plugin8Out6), ENUM_NAME(plugin8Out7),
	ENUM_NAME(plugin8Out8), ENUM_NAME(plugin8Out9),
	ENUM_NAME(plugin8Out10), ENUM_NAME(plugin8Out11),
	ENUM_NAME(plugin8Out12), ENUM_NAME(plugin8Out13),
	ENUM_NAME(plugin8Out14), ENUM_NAME(plugin8Out15),
	ENUM_NAME(plugin9Out0), ENUM_NAME(plugin9Out1),
	ENUM_NAME(plugin9Out2), ENUM_NAME(plugin9Out3),
	ENUM_NAME(plugin9Out4), ENUM_NAME(plugin9Out5),
	ENUM_NAME(plugin9Out6), ENUM_NAME(plugin9Out7),
	ENUM_NAME(plugin9Out8), ENUM_NAME(plugin9Out9),
	ENUM_NAME(plugin9Out10), ENUM_NAME(plugin9Out11),
	ENUM_NAME(plugin9Out12), ENUM_NAME(plugin9Out13),
	ENUM_NAME(plugin9Out14), ENUM_NAME(plugin9Out15),
	ENUM_NAME(plugin10Out0), ENUM_NAME(plugin10Out1),
	ENUM_NAME(plugin10Out2), ENUM_NAME(plugin10Out3),
	ENUM_NAME(plugin10Out4), ENUM_NAME(plugin10Out5),
	ENUM_NAME(plugin10Out6), ENUM_NAME(plugin10Out7),
	ENUM_NAME(plugin10Out8), ENUM_NAME(plugin10Out9),
	ENUM_NAME(plugin10Out10), ENUM_NAME(plugin10Out11),
	ENUM_NAME(plugin10Out12), ENUM_NAME(plugin10Out13),
	ENUM_NAME(plugin10Out14), ENUM_NAME(plugin10Out15),
	ENUM_NAME(plugin11Out0), ENUM_NAME(plugin9Out1),
	ENUM_NAME(plugin11Out2), ENUM_NAME(plugin11Out3),
	ENUM_NAME(plugin11Out4), ENUM_NAME(plugin11Out5),
	ENUM_NAME(plugin11Out6), ENUM_NAME(plugin11Out7),
	ENUM_NAME(plugin11Out8), ENUM_NAME(plugin11Out9),
	ENUM_NAME(plugin11Out10), ENUM_NAME(plugin11Out11),
	ENUM_NAME(plugin11Out12), ENUM_NAME(plugin11Out13),
	ENUM_NAME(plugin11Out14), ENUM_NAME(plugin11Out15),
	ENUM_NAME(plugin12Out0), ENUM_NAME(plugin12Out1),
	ENUM_NAME(plugin12Out2), ENUM_NAME(plugin12Out3),
	ENUM_NAME(plugin12Out4), ENUM_NAME(plugin12Out5),
	ENUM_NAME(plugin12Out6), ENUM_NAME(plugin12Out7),
	ENUM_NAME(plugin12Out8), ENUM_NAME(plugin12Out9),
	ENUM_NAME(plugin12Out10), ENUM_NAME(plugin12Out11),
	ENUM_NAME(plugin12Out12), ENUM_NAME(plugin12Out13),
	ENUM_NAME(plugin12Out14), ENUM_NAME(plugin12Out15),
	ENUM_NAME(plugin13Out0), ENUM_NAME(plugin13Out1),
	ENUM_NAME(plugin13Out2), ENUM_NAME(plugin13Out3),
	ENUM_NAME(plugin13Out4), ENUM_NAME(plugin13Out5),
	ENUM_NAME(plugin13Out6), ENUM_NAME(plugin13Out7),
	ENUM_NAME(plugin13Out8), ENUM_NAME(plugin13Out9),
	ENUM_NAME(plugin13Out10), ENUM_NAME(plugin13Out11),
	ENUM_NAME(plugin13Out12), ENUM_NAME(plugin13Out13),
	ENUM_NAME(plugin13Out14), ENUM_NAME(plugin13Out15),
	ENUM_NAME(plugin14Out0), ENUM_NAME(plugin14Out1),
	ENUM_NAME(plugin14Out2), ENUM_NAME(plugin14Out3),
	ENUM_NAME(plugin14Out4), ENUM_NAME(plugin14Out5),
	ENUM_NAME(plugin14Out6), ENUM_NAME(plugin14Out7),
	ENUM_NAME(plugin14Out8), ENUM_NAME(plugin14Out9),
	ENUM_NAME(plugin14Out10), ENUM_NAME(plugin14Out11),
	ENUM_NAME(plugin14Out12), ENUM_NAME(plugin14Out13),
	ENUM_NAME(plugin14Out14), ENUM_NAME(plugin14Out15),
	ENUM_NAME(plugin15Out0), ENUM_NAME(plugin15Out1),
	ENUM_NAME(plugin15Out2), ENUM_NAME(plugin15Out3),
	ENUM_NAME(plugin15Out4), ENUM_NAME(plugin15Out5),
	ENUM_NAME(plugin15Out6), ENUM_NAME(plugin15Out7),
	ENUM_NAME(plugin15Out8), ENUM_NAME(plugin15Out9),
	ENUM_NAME(plugin15Out10), ENUM_NAME(plugin15Out11),
	ENUM_NAME(plugin15Out12), ENUM_NAME(plugin15Out13),
	ENUM_NAME(plugin15Out14), ENUM_NAME(plugin15Out15),
};

#define IAXXXCORE_TX_CHMGR_ENUM(channel) \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_rmp_enum, \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(channel), \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS, \
		(IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK >> \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_POS), \
		gain_ramp_texts, gain_ramp_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_gn_evnt_enum, \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(channel), \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS, \
		(IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_MASK >> \
		IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_REACHED_EVT_POS), \
		gain_rech_evnt_texts, gain_rech_evnt_value); \
static SOC_VALUE_ENUM_SINGLE_DECL(channel##_ip_src_id_enum, \
		IAXXX_OUT_CH_GRP_IN_CONNECT_REG(channel), \
		IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_POS, \
		(IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_MASK >> \
		IAXXX_OUT_CH_GRP_IN_CONNECT_SRC_ID_POS), \
		ip_ep_texts, ip_ep_values)

IAXXXCORE_TX_CHMGR_ENUM(TX_0);
IAXXXCORE_TX_CHMGR_ENUM(TX_1);
IAXXXCORE_TX_CHMGR_ENUM(TX_2);
IAXXXCORE_TX_CHMGR_ENUM(TX_3);
IAXXXCORE_TX_CHMGR_ENUM(TX_4);
IAXXXCORE_TX_CHMGR_ENUM(TX_5);
IAXXXCORE_TX_CHMGR_ENUM(TX_6);
IAXXXCORE_TX_CHMGR_ENUM(TX_7);
IAXXXCORE_TX_CHMGR_ENUM(TX_8);
IAXXXCORE_TX_CHMGR_ENUM(TX_9);
IAXXXCORE_TX_CHMGR_ENUM(TX_10);
IAXXXCORE_TX_CHMGR_ENUM(TX_11);
IAXXXCORE_TX_CHMGR_ENUM(TX_12);
IAXXXCORE_TX_CHMGR_ENUM(TX_13);
IAXXXCORE_TX_CHMGR_ENUM(TX_14);
IAXXXCORE_TX_CHMGR_ENUM(TX_15);

#define IAXXXCORE_TX_CHMGR_KCTRL(channel, channel_name) \
	SOC_ENUM(channel_name "Chan IpSrcId", channel##_ip_src_id_enum), \
	SOC_SINGLE_TLV(channel_name "Chan EpGain", \
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(channel), \
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_POS, \
				0xFF, 0, gn_ch_ep_tlv), \
	SOC_ENUM(channel_name "Chan GnReEvt", channel##_gn_evnt_enum), \
	SOC_ENUM(channel_name "Chan GnRmp", channel##_gn_rmp_enum), \
	SOC_SINGLE(channel_name "Chan Gain En", \
			IAXXX_CH_HDR_CH_GAIN_ADDR, channel, 1, 0)


IAXXX_CH_MGR_DAPM_CTLS(TX_0, "Tx0 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_1, "Tx1 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_2, "Tx2 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_3, "Tx3 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_4, "Tx4 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_5, "Tx5 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_6, "Tx6 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_7, "Tx7 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_8, "Tx8 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_9, "Tx9 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_10, "Tx10 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_11, "Tx11 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_12, "Tx12 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_13, "Tx13 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_14, "Tx14 Mux");
IAXXX_CH_MGR_DAPM_CTLS(TX_15, "Tx15 Mux");

IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_0, "Tx0");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_1, "Tx1");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_2, "Tx2");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_3, "Tx3");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_4, "Tx4");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_5, "Tx5");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_6, "Tx6");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_7, "Tx7");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_8, "Tx8");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_9, "Tx9");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_10, "Tx10");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_11, "Tx11");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_12, "Tx12");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_13, "Tx13");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_14, "Tx14");
IAXXX_CH_RX_TO_TX_DAPM_CTLS(TX_15, "Tx15");

/* FIXME , this should be just integer value */
static const unsigned int plugin_ctrl_opt_value[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, };

static const char * const plugin_ctrl_opt_text[] = {
	"none", "PkgPlgin1", "PkgPlgin2", "PkgPlgin3", "PkgPlgin4",
	"PkgPlgin5", "PkgPlgin6", "PkgPlgin7", "PkgPlgin8", };

static const unsigned int plugin_origin_idx_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, };

static const char * const plugin_origin_idx_texts[] = {
	"none", "PkgPlgin1", "PkgPlgin2", "PkgPlgin3", "PkgPlgin4",
	"PkgPlgin5", "PkgPlgin6", "PkgPlgin7", "PkgPlgin8", };

static const char * const pkg_id_texts[] = {
	"none", "pkg1", "pkg2", "pkg3", "pkg4", "pkg5", "pkg6",
	"pkg7", "pkg8", "pkg9", "pkg10", "pkg11", "pkg12",
	"pkg13", "pkg14", "pkg15", "pkg16",
};

static const unsigned int pkg_id_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA,
	0xB, 0xC, 0xD, 0xE, 0xF,
};

static const char * const priority_texts[] = {
	"pri0", "pri1", "pri2", "pri3", "pri4", "pri5", "pri6", "pri7",
	"pri8", "pri9", "pri10", "pri11", "pri12", "pri13", "pri14", "pri15",
};

static const unsigned int priority_values[] = {
	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xF,
};


static int iaxxxcore_blk0_get_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[0].param_id;
	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk0_put_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;

	iaxxx->plugin_param[0].param_id = ucontrol->value.integer.value[0];
	iaxxx->plugin_param[0].param_id_reg = reg;

	return 0;
}

static int iaxxxcore_blk1_get_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[1].param_id;
	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk1_put_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;

	iaxxx->plugin_param[1].param_id = ucontrol->value.integer.value[0];
	iaxxx->plugin_param[1].param_id_reg = reg;

	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk2_get_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->plugin_param[2].param_id;
	pr_debug("%s\n", __func__);
	return 0;
}

static int iaxxxcore_blk2_put_param_id(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	unsigned int reg = mc->reg;

	iaxxx->plugin_param[2].param_id = ucontrol->value.integer.value[0];
	iaxxx->plugin_param[2].param_id_reg = reg;

	pr_debug("%s\n", __func__);
	return 0;
}

#define IAXXXCORE_PLUGIN_ENUM(plugin, plugin_name) \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ctrl_pri_enum, \
			IAXXX_PLUGIN_INS_GRP_CTRL_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS, \
			(IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_MASK >> \
			IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS), \
			priority_texts, priority_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_inst_pkgid_enum, \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_POS, \
			(IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_POS), \
			pkg_id_texts, pkg_id_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_origin_idx_enum, \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS, \
			(IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_MASK >> \
			IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS), \
			plugin_origin_idx_texts, plugin_origin_idx_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep0_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_0_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep1_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_1_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep2_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_2_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep3_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_3_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep4_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_4_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep5_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_5_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep6_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_6_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep7_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_7_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep8_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_8_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_8_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_8_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_8_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep9_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_9_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_9_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_9_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_9_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep10_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_10_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_10_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_10_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_10_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep11_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_11_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_11_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_11_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_11_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep12_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_12_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_12_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_12_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_12_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep13_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_13_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_13_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_13_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_13_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep14_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_14_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_14_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_14_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_14_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ip_ep15_enum, \
			IAXXX_PLUGIN_INS_GRP_IN_15_CONNECT_REG(plugin), \
			IAXXX_PLUGIN_INS_GRP_IN_15_CONNECT_SOURCEID_POS, \
			(IAXXX_PLUGIN_INS_GRP_IN_15_CONNECT_SOURCEID_MASK >> \
			IAXXX_PLUGIN_INS_GRP_IN_15_CONNECT_SOURCEID_POS), \
			ip_ep_texts, ip_ep_values); \
static SOC_VALUE_ENUM_SINGLE_DECL(plugin##_ctrl_option_enum, \
			IAXXX_PLUGIN_INS_GRP_CREATION_CFG_REG(plugin), 0, \
			IAXXX_PLUGIN_INS_GRP_CREATION_CFG_MASK_VAL, \
			plugin_ctrl_opt_text, plugin_ctrl_opt_value)

#define IAXXXCORE_PLUGIN_KCTRL(plugin, plugin_name) \
	SOC_SINGLE(plugin_name "Create0", \
			IAXXX_PLUGIN_HDR_CREATE_BLOCK_0_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Create1", \
			IAXXX_PLUGIN_HDR_CREATE_BLOCK_1_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Create2", \
			IAXXX_PLUGIN_HDR_CREATE_BLOCK_2_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Reset0", \
			IAXXX_PLUGIN_HDR_RESET_BLOCK_0_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Reset1", \
			IAXXX_PLUGIN_HDR_RESET_BLOCK_1_ADDR, plugin, 1, 0), \
	SOC_SINGLE(plugin_name "Reset2", \
			IAXXX_PLUGIN_HDR_RESET_BLOCK_2_ADDR, plugin, 1, 0), \
	SOC_ENUM(plugin_name "Origin Plugin Idx", plugin##_origin_idx_enum), \
	SOC_ENUM(plugin_name "Pkg ID", plugin##_inst_pkgid_enum), \
	SOC_ENUM(plugin_name "inst priority", plugin##_ctrl_pri_enum), \
	SOC_SINGLE_EXT(plugin_name "Blk0 Param Id", \
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk0_get_param_id, \
			iaxxxcore_blk0_put_param_id), \
	SOC_SINGLE_EXT(plugin_name "Blk1 Param Id", \
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk1_get_param_id, \
			iaxxxcore_blk1_put_param_id), \
	SOC_SINGLE_EXT(plugin_name "Blk2 Param Id", \
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(plugin), 0, \
			IAXXX_MAX_VAL, 0, iaxxxcore_blk2_get_param_id, \
			iaxxxcore_blk2_put_param_id), \
	SOC_ENUM(plugin_name "Ip Ep0 Conf", plugin##_ip_ep0_enum), \
	SOC_ENUM(plugin_name "Ip Ep1 Conf", plugin##_ip_ep1_enum), \
	SOC_ENUM(plugin_name "Ip Ep2 Conf", plugin##_ip_ep2_enum), \
	SOC_ENUM(plugin_name "Ip Ep3 Conf", plugin##_ip_ep3_enum),\
	SOC_ENUM(plugin_name "Ip Ep4 Conf", plugin##_ip_ep4_enum), \
	SOC_ENUM(plugin_name "Ip Ep5 Conf", plugin##_ip_ep5_enum), \
	SOC_ENUM(plugin_name "Ip Ep6 Conf", plugin##_ip_ep6_enum), \
	SOC_ENUM(plugin_name "Ip Ep7 Conf", plugin##_ip_ep7_enum), \
	SOC_ENUM(plugin_name "Ip Ep8 Conf", plugin##_ip_ep8_enum), \
	SOC_ENUM(plugin_name "Ip Ep9 Conf", plugin##_ip_ep9_enum), \
	SOC_ENUM(plugin_name "Ip Ep10 Conf", plugin##_ip_ep10_enum), \
	SOC_ENUM(plugin_name "Ip Ep11 Conf", plugin##_ip_ep11_enum),\
	SOC_ENUM(plugin_name "Ip Ep12 Conf", plugin##_ip_ep12_enum), \
	SOC_ENUM(plugin_name "Ip Ep13 Conf", plugin##_ip_ep13_enum), \
	SOC_ENUM(plugin_name "Ip Ep14 Conf", plugin##_ip_ep14_enum), \
	SOC_ENUM(plugin_name "Ip Ep15 Conf", plugin##_ip_ep15_enum), \
	SOC_ENUM(plugin_name "Cr Conf", plugin##_ctrl_option_enum)


IAXXXCORE_PLUGIN_ENUM(PLUGIN0, "Plgin0");
IAXXXCORE_PLUGIN_ENUM(PLUGIN1, "Plgin1");
IAXXXCORE_PLUGIN_ENUM(PLUGIN2, "Plgin2");
IAXXXCORE_PLUGIN_ENUM(PLUGIN3, "Plgin3");
IAXXXCORE_PLUGIN_ENUM(PLUGIN4, "Plgin4");
IAXXXCORE_PLUGIN_ENUM(PLUGIN5, "Plgin5");
IAXXXCORE_PLUGIN_ENUM(PLUGIN6, "Plgin6");
IAXXXCORE_PLUGIN_ENUM(PLUGIN7, "Plgin7");
IAXXXCORE_PLUGIN_ENUM(PLUGIN8, "Plgin8");
IAXXXCORE_PLUGIN_ENUM(PLUGIN9, "Plgin9");
IAXXXCORE_PLUGIN_ENUM(PLUGIN10, "Plgin10");
IAXXXCORE_PLUGIN_ENUM(PLUGIN11, "Plgin11");
IAXXXCORE_PLUGIN_ENUM(PLUGIN12, "Plgin12");
IAXXXCORE_PLUGIN_ENUM(PLUGIN13, "Plgin13");
IAXXXCORE_PLUGIN_ENUM(PLUGIN14, "Plgin14");
IAXXXCORE_PLUGIN_ENUM(PLUGIN15, "Plgin15");


#define IAXXXCORE_PLUGIN_ON_OFF_TEXTS(plugin, plugin_name) \
static const char * const plugin##_off_on_texts[] = { \
	"Off", \
	"Rx0"plugin_name"On", "Rx1"plugin_name"On", "Rx2"plugin_name"On", \
	"Rx3"plugin_name"On", "Rx4"plugin_name"On", "Rx5"plugin_name"On", \
	"Rx6"plugin_name"On", "Rx7"plugin_name"On", "Rx8"plugin_name"On", \
	"Rx9"plugin_name"On", "Rx10"plugin_name"On", "Rx11"plugin_name"On", \
	"Rx12"plugin_name"On", "Rx13"plugin_name"On", "Rx14"plugin_name"On", \
	"Rx15"plugin_name"On", \
}

IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN0, "Plgin0");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN1, "Plgin1");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN2, "Plgin2");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN3, "Plgin3");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN4, "Plgin4");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN5, "Plgin5");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN6, "Plgin6");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN7, "Plgin7");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN8, "Plgin8");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN9, "Plgin9");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN10, "Plgin10");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN11, "Plgin11");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN12, "Plgin12");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN13, "Plgin13");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN14, "Plgin14");
IAXXXCORE_PLUGIN_ON_OFF_TEXTS(PLUGIN15, "Plgin15");

#define IAXXX_PLUGIN_DAPM_MUX(plugin, plugin_name) \
	SND_SOC_DAPM_MUX(plugin_name "En", SND_SOC_NOPM, 0, 0, &plugin##_mux)


#define IAXXX_PLUGIN_BLK_SET_GET(plugin) \
static int iaxxxcore_set_plgin##plugin##_Blk0En( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 status = 0; \
	int ret = 0; \
	pr_debug("enter %s connection\n", __func__); \
	if (ucontrol->value.enumerated.item[0]) { \
		snd_soc_update_bits(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_0_ADDR, \
					1 << plugin, 1 << plugin); \
		iaxxx->plugin_blk_en[plugin] = 1; \
	} else { \
		snd_soc_update_bits(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_0_ADDR, \
					1 << plugin, 0 << plugin); \
		iaxxx->plugin_blk_en[plugin] = 0; \
	} \
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
			 &status, IAXXX_BLOCK_0); \
	if (ret) \
		dev_err(priv->dev, "Update blk failed %s():%u\n", \
					__func__, status); \
	return ret;\
} \
static int iaxxxcore_get_plgin##plugin##_Blk0En( \
				struct snd_kcontrol *kcontrol, \
				struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.enumerated.item[0] = iaxxx->plugin_blk_en[plugin]; \
	return 0; \
} \
static int iaxxxcore_set_plgin##plugin##_Blk1En( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 status = 0; \
	int ret = 0; \
	pr_debug("enter %s connection\n", __func__); \
	if (ucontrol->value.enumerated.item[0]) { \
		snd_soc_update_bits(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_1_ADDR, \
					1 << plugin, 1 << plugin); \
		iaxxx->plugin_blk_en[plugin] = 1; \
	} else { \
		snd_soc_update_bits(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_1_ADDR, \
					1 << plugin, 0 << plugin); \
		iaxxx->plugin_blk_en[plugin] = 0; \
	} \
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
			 &status, IAXXX_BLOCK_1); \
	if (ret) \
		dev_err(priv->dev, "Update blk failed %s():%u\n", \
					__func__, status); \
	return ret;\
} \
static int iaxxxcore_get_plgin##plugin##_Blk1En( \
				struct snd_kcontrol *kcontrol, \
				struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.enumerated.item[0] = iaxxx->plugin_blk_en[plugin]; \
	return 0; \
} \
static int iaxxxcore_set_plgin##plugin##_Blk2En( \
				struct snd_kcontrol *kcontrol, \
				struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	struct iaxxx_priv *priv = to_iaxxx_priv(iaxxx->dev_parent); \
	u32 status = 0; \
	int ret = 0; \
	pr_debug("enter %s connection\n", __func__); \
	if (ucontrol->value.enumerated.item[0]) { \
		snd_soc_update_bits(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_2_ADDR, \
					1 << plugin, 1 << plugin); \
		iaxxx->plugin_blk_en[plugin] = 1; \
	} else { \
		snd_soc_update_bits(codec, \
					IAXXX_PLUGIN_HDR_ENABLE_BLOCK_2_ADDR, \
					1 << plugin, 0 << plugin); \
		iaxxx->plugin_blk_en[plugin] = 0; \
	} \
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, \
			 &status, IAXXX_BLOCK_2); \
	if (ret) \
		dev_err(priv->dev, "Update blk failed %s():%u\n", \
					__func__, status); \
	return ret; \
} \
static int iaxxxcore_get_plgin##plugin##_Blk2En( \
				struct snd_kcontrol *kcontrol, \
				struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.enumerated.item[0] = iaxxx->plugin_blk_en[plugin]; \
	return 0; \
}

IAXXX_PLUGIN_BLK_SET_GET(PLUGIN0)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN1)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN2)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN3)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN4)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN5)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN6)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN7)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN8)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN9)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN10)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN11)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN12)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN13)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN14)
IAXXX_PLUGIN_BLK_SET_GET(PLUGIN15)

#define IAXXX_PLUGIN_EN_CTLS(plugin, plugin_name) \
	SOC_SINGLE_BOOL_EXT(plugin_name "Blk0En", 0, \
		       iaxxxcore_get_plgin##plugin##_Blk0En, \
		       iaxxxcore_set_plgin##plugin##_Blk0En), \
	SOC_SINGLE_BOOL_EXT(plugin_name "Blk1En", 0, \
		       iaxxxcore_get_plgin##plugin##_Blk1En, \
		       iaxxxcore_set_plgin##plugin##_Blk1En), \
	SOC_SINGLE_BOOL_EXT(plugin_name "Blk2En", 0, \
		       iaxxxcore_get_plgin##plugin##_Blk2En, \
		       iaxxxcore_set_plgin##plugin##_Blk2En)

#define IAXXX_PLUGIN_DAPM_CTLS(plugin, plugin_name) \
static const SOC_ENUM_SINGLE_DECL(plugin##_en_enum, \
						SND_SOC_NOPM, plugin, \
						plugin##_off_on_texts); \
static const struct snd_kcontrol_new plugin##_mux =	\
	SOC_DAPM_ENUM(plugin_name "Enable", plugin##_en_enum)

IAXXX_PLUGIN_DAPM_CTLS(PLUGIN0, "Plgin0");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN1, "Plgin1");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN2, "Plgin2");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN3, "Plgin3");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN4, "Plgin4");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN5, "Plgin5");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN6, "Plgin6");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN7, "Plgin7");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN8, "Plgin8");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN9, "Plgin9");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN10, "Plgin10");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN11, "Plgin11");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN12, "Plgin12");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN13, "Plgin13");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN14, "Plgin14");
IAXXX_PLUGIN_DAPM_CTLS(PLUGIN15, "Plgin15");
/*
 * Each MIC can have a clock source from the port it is
 * conencted or its clock can be derived from other port.
 *
 * This API will determine weather the MIC have its
 * clock source from same port or a different port.
 *
 * DT entry "adnc,ip-pdm-clk-src" will be input for
 * this API to determine the input clock.
 */
/* TODO */
/* If same port MIC clock source is configured from
 * different port then it is not handled here.
 */
static int iaxxx_pdm_port_clk_src(
			struct iaxxx_codec_priv *iaxxx, int port)
{
	int port_clk = 0;

	if (iaxxx->ip_pdm_clk_src[port] == 1) {
		port_clk = port;
	} else {
		for (port_clk = 0; port_clk < IAXXX_MAX_PDM_PORTS; port_clk++)
			if (iaxxx->ip_pdm_clk_src[port_clk])
				break;
	}

	return port_clk;
}

static int iaxxx_put_start_pdm(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	int try = IAXXX_MAX_RETRY, ret = -EINVAL;
	uint32_t port_mics_en = ucontrol->value.integer.value[0];

	pr_debug("enter %s(): mics to en:%u on port:%d\n",
				__func__, port_mics_en, port);

	if (iaxxx->port_start_en[port] == port_mics_en)
		return 0;

	if (port_mics_en > 255)
		return ret;

	/* Read DMIC enable busy reg, check for DMIC is not busy
	 * set DMIC enable only if DMIC enable busy reg is not set
	 * Check again DMIC enable busy reg is cleared by HW.
	 * DMIC enable busy will get cleared once port clock is available.
	 */
	while (snd_soc_read(codec, dmic_busy_addr[port]) && (try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: port: %d DMIC busy enable bit busy before :%d\n",
			__func__, port, __LINE__);
		return ret;
	}

	/* DMIC enable */
	snd_soc_update_bits(codec, dmic_enable_addr[port], 0xFF, port_mics_en);

	try = IAXXX_MAX_RETRY;
	/*  Read DMIC ENABLE busy  */
	while (snd_soc_read(codec, dmic_busy_addr[port]) && (try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: port: %d DMIC busy enable bit busy after :%d\n",
			__func__, port, __LINE__);
		return ret;
	}

	iaxxx->port_start_en[port] = ucontrol->value.integer.value[0];
	return 0;
}

static int iaxxx_put_port_clk_stop(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 status = 0;
	int ret = 0;

	pr_debug("enter %s()\n", __func__);
	if (iaxxx->port_start_en[port] == ucontrol->value.integer.value[0])
		return 0;

	if (port > PDM_CDC) {
		pr_err("PDM port stop failed Invalid port number\n");
		return -EINVAL;
	}

	/* Disable Clock output */
	if (port == PDM_CDC) {
		snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
				IAXXX_AO_CLK_CFG_CDC_MCLK_OE_POS, 0);
	} else {
		snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
			(IAXXX_AO_CLK_CFG_PORTA_CLK_OE_MASK << port), 0);
	}

	/* DISABLE I2S PORT */
	/* CNR0_I2S_Enable  - Disable I2S  */
	snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(port),
			IAXXX_CNR0_I2S_ENABLE_LOW << port);
	/* I2S Trigger - Enable */
	snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);

	snd_soc_update_bits(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
			IAXXX_SRB_I2S_PORT_PWR_EN_MASK_VAL, (0x0 << port));

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
							IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}
	iaxxx->port_start_en[port] = 0;
	return ret;
}

#define IAXXX_PDM_PORT_START_SET_GET(port_name, port) \
static int iaxxx_put_start_##port_name(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_put_start_pdm(kcontrol, ucontrol, port); \
	else \
		return iaxxx_put_port_clk_stop(kcontrol, ucontrol, port); \
} \
static int iaxxx_get_start_##port_name(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_start_en[port]; \
	return 0; \
}

IAXXX_PDM_PORT_START_SET_GET(portc, PDM_PORTC)
IAXXX_PDM_PORT_START_SET_GET(portb, PDM_PORTB)
IAXXX_PDM_PORT_START_SET_GET(cdc0, PDM_CDC)
IAXXX_PDM_PORT_START_SET_GET(cdc1, PDM_PORTD)

static int iaxxx_calc_i2s_div(u32 bits_per_frame, u32 sampling_rate,
			u32 *period, u32 *div_val, u32 *nr_val)
{
	u32 bit_clk;
	u32 divider;
	u32 r_val;
	u32 n_val;
	u32 div_cnfg_len;
	int i = 0;

	*div_val = 0;
	*nr_val = 0;
	/* get bit_clk freq */
	bit_clk = (sampling_rate * (bits_per_frame + 1));
	/* get size of NR divider values struct */
	div_cnfg_len = ARRAY_SIZE(i2s_div_config);

	for (i = 0; i < div_cnfg_len; i++) {
		if (bit_clk == i2s_div_config[i].bclk) {
			n_val = i2s_div_config[i].N;
			r_val = i2s_div_config[i].R;
			*period = i2s_div_config[i].period;
			divider = i2s_div_config[i].HL;

			*div_val = *div_val |
				((divider << IAXXX_I2S_I2S0_HL_P_POS) &
				IAXXX_I2S_I2S0_HL_P_MASK);
			*nr_val = *nr_val |
				(((n_val << IAXXX_I2S_I2S0_NR_N_POS) &
				IAXXX_I2S_I2S0_NR_N_MASK) |
				((r_val << IAXXX_I2S_I2S0_NR_R_POS) &
				IAXXX_I2S_I2S0_NR_R_MASK));
			return 0;
		}
	}

	return -EINVAL;
}

static int iaxxx_set_i2s_controller(struct snd_soc_codec *codec,
			u32 sampling_rate, bool is_pseudo, int id)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	u32 bits_per_frame = 0, pcm_words_per_frame = 0;
	u32 is_i2s_mode = 0;
	u32 period = 0, div_val = 0, nr_val = 0, status = 0;
	u32 clk_ctrl_val = 0, pcm_word_len = 0;

	if (is_pseudo) {
		pr_err("Pseudo mode not supported\n");
		return -EINVAL;
	}

	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << id), (0x1 << id));

	iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			    IAXXX_BLOCK_0);

	/* CNR0_I2S_Enable  - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(id),
		IAXXX_CNR0_I2S_ENABLE_LOW);

	/* I2S Trigger - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);

	/*Bit 0 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
		IAXXX_I2S_I2S0_GEN_CFG_PCM_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	/* Bit 1 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_MASK,
		IAXXX_I2S_GEN_CFG_CLK_POL_HIGH);

	is_i2s_mode = snd_soc_read(codec, IAXXX_PCM_MC_ADDR(id));
	is_i2s_mode &= IAXXX_PCM0_MC_FSP_MASK;

	pcm_word_len = snd_soc_read(codec, IAXXX_PCM_SWLR_ADDR(id));
	pcm_word_len &= IAXXX_PCM0_SWLR_RMASK_VAL;

	/* Bit 2*/
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_I2S_MODE);

	if (id == 4) {
		/* Bit 19:12 */
		snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
			IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK,
			(1 << IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS));
	} else {
		/* Bit 19:12 */
		snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
			IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK,
			((pcm_word_len + 1) <<
			IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS));
	}
	/* Bit 3 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
		IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_MASK,
		IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE);

	/* Bit 11:4 */
	pcm_words_per_frame = snd_soc_read(codec, IAXXX_PCM_SFLR_ADDR(id));
	pcm_words_per_frame &= IAXXX_PCM0_SFLR_RMASK_VAL;

	bits_per_frame = (((pcm_word_len + 1) *
				(pcm_words_per_frame + 1)) - 1);


	pr_debug("bits_per_frame :%d  pcm_words_per_frame: %d\n",
				bits_per_frame, pcm_words_per_frame);

	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
			IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_MASK,
			(bits_per_frame <<
			IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS));

	/* Bit 20 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(id),
		IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_MASK,
		IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE);

	/* FS Align */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_FS_ALIGN_ADDR(id),
		IAXXX_I2S_I2S0_FS_ALIGN_WMASK_VAL,
		IAXXX_I2S_FS_ALIGN_MASTER_MODE);

	iaxxx_calc_i2s_div(bits_per_frame, sampling_rate,
			&period, &div_val, &nr_val);
	/* disable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(id),
		IAXXX_I2S_I2S0_HL_EN_MASK,
		IAXXX_I2S_I2S0_HL_DISABLE);
	/* Set HL value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(id),
		IAXXX_I2S_I2S0_HL_P_MASK,
		div_val);
	/* enable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(id),
		IAXXX_I2S_I2S0_HL_EN_MASK,
		IAXXX_I2S_I2S0_HL_ENABLE);

	/* disable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(id),
		IAXXX_I2S_I2S0_NR_EN_MASK,
		IAXXX_I2S_I2S0_NR_DISABLE);
	/* Set NR value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(id),
		IAXXX_I2S_I2S0_NR_MASK_VAL,
		nr_val);
	/* enable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(id),
		IAXXX_I2S_I2S0_NR_EN_MASK,
		IAXXX_I2S_I2S0_NR_ENABLE);
	/* Clk control */
	clk_ctrl_val = (clk_ctrl_val | ((period/2 - 1) <<
			IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS) |
		((period - 1)<<IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS));

	pr_debug("clk_ctrl_val :0x%x period:%d\n", clk_ctrl_val, period);

	snd_soc_update_bits(codec, IAXXX_I2S_I2S_CLK_CTRL_ADDR(id),
		IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL,
		clk_ctrl_val);

	return 0;
}

static int iaxxx_pdm_mic_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol,
			int port, int cic, int dmic)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 cic_rx_id = 0;
	u32 pdm_bclk = 0;
	u8 aud_port_clk = 0;
	u32 cic_hb = 0, hb_dec = 0;
	u32 io_ctrl_reg, io_ctrl_mask, io_ctrl_val;
	u32 io_ctrl_clk_reg, io_ctrl_clk_val;
	u32 codec_dmic = iaxxx->cdc_dmic_enable;
	u32 cic_ctrl = 0;
	u32 cic_rx_rt_ctrl = 0, cic_rx_reset = 0, cic_rx_dec = 0;
	int ret = -EINVAL;
	int io_port_mic = dmic;
	u32 status = 0;
	int clk_src = 0;
	int pdm_mstr = 0;
	int port_clk = 0;

	cic_rx_id = cic;
	pdm_bclk = iaxxx->pdm_bclk;
	aud_port_clk = iaxxx->pdm_aclk;

	port_clk = iaxxx_pdm_port_clk_src(iaxxx, port);
	switch (port_clk) {
	case PDM_PORTC:
		clk_src = 0;
		pdm_mstr = 1;
		break;
	case PDM_PORTB:
		clk_src = 2;
		pdm_mstr = 1;
		break;
	case PDM_PORTD:
		clk_src = 3;
		pdm_mstr = 1;
		break;
	case PDM_CDC:
		clk_src = 1;
		pdm_mstr = 1;
		break;
	default:
		if (port_clk < IAXXX_MAX_PDM_PORTS)
			pr_info("may be slave pdm %d", port_clk);
		else {
			pr_err("wrong port is requested %d", port_clk);
			return -EINVAL;
		}
	}

	pr_debug("port_clk:%d clk_src:%d pdm_mstr=%d\n", port_clk,
			clk_src, pdm_mstr);
	io_ctrl_clk_reg = iaxxx_io_ctrl_clk[clk_src][0];
	io_ctrl_clk_val = iaxxx_io_ctrl_clk[clk_src][pdm_mstr + 1];

	snd_soc_write(codec, io_ctrl_clk_reg, io_ctrl_clk_val);

#ifndef CONFIG_IAXXX_PDM_DMIC_MODE
	if (port_clk != port) {
		switch (port) {
		case PDM_PORTC:
			clk_src = 0;
			break;
		case PDM_PORTB:
			clk_src = 2;
			break;
		case PDM_PORTD:
			clk_src = 3;
			break;
		case PDM_CDC:
			clk_src = 1;
			break;
		default:
			if (port_clk < IAXXX_MAX_PDM_PORTS)
				pr_info("may be slave pdm %d", port_clk);
			else {
				pr_err("wrong port is requested %d", port_clk);
				return -EINVAL;
			}
		}

		io_ctrl_clk_reg = iaxxx_io_port_clk_fwd[clk_src][0];
		io_ctrl_clk_val = iaxxx_io_port_clk_fwd[clk_src][1];
		snd_soc_write(codec, io_ctrl_clk_reg, io_ctrl_clk_val);
	}
#endif

	snd_soc_update_bits(codec, IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
				(1 << cic_rx_id), (1 << cic_rx_id));

	iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
						    IAXXX_BLOCK_0);

	if ((dmic >= PDM_CDC0_IN0) && (dmic <= PDM_CDC3_IN7)) {
		dmic  = dmic - PDM_CDC0_IN0;
		codec_dmic = 1;
	}

	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_ADTL_CTRL_ADDR,
				IAXXX_CIC_ADTL_RX_MASK(cic_rx_id), 0);

	cic_rx_reset =	IAXXX_CIC_RX_RESET <<
			cic_rx_clr_pos[cic_rx_id];
	snd_soc_update_bits(codec, cic_rx_addr[cic_rx_id],
			cic_rx_clr_mask[cic_rx_id], cic_rx_reset);

	/* Get CIC decimation value and half band decimation value */
	ret = get_decimator_val(pdm_bclk, aud_port_clk,
				&cic_rx_dec, &hb_dec);
	if (ret)
		return ret;
	cic_rx_dec = cic_rx_dec << cic_rx_m_pos[cic_rx_id];

	/* Green box /Half band decimation filter */
	cic_hb = hb_dec << IAXXX_CNR0_CIC_HB_CIC_RX_POS(cic_rx_id);

	/* setup input clk source (base/alternative) & Bit Polarity*/
	if (port_clk == PDM_PORTC || port_clk == PDM_CDC)
		cic_ctrl = IAXXX_DMIC0_CLK << cic_rx_id;
	else
		cic_ctrl = IAXXX_DMIC1_CLK << cic_rx_id;

	cic_ctrl = cic_ctrl | (IAXXX_PDM_POLARITY <<
		IAXXX_CNR0_CIC_CTRL_RX_POL_POS(cic_rx_id));
	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_CTRL_ADDR,
		IAXXX_CNR0_CIC_CTRL_RX_MASK(cic_rx_id), cic_ctrl);


	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_HB_ADDR,
		IAXXX_CNR0_CIC_HB_CIC_RX_MASK(cic_rx_id), cic_hb);

	if (((port_clk != PDM_CDC) && (port_clk != PCM_PORTD))
						&& !codec_dmic) {
		cic_rx_rt_ctrl = IAXXX_CIC_MIC_ENABLE <<
			IAXXX_CNR0_CIC_RX_RT_CTRL_MIC_POS(cic_rx_id);
		cic_rx_rt_ctrl = cic_rx_rt_ctrl | (IAXXX_CIC_S_DMIC_ENABLE<<
			IAXXX_CNR0_CIC_RX_RT_CTRL_S_POS(cic_rx_id));
	}
	cic_rx_rt_ctrl = cic_rx_rt_ctrl | (IAXXX_CLK_ENABLE <<
		IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_POS(cic_rx_id));

	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
		IAXXX_CNR0_CIC_RX_RT_MASK(cic_rx_id), cic_rx_rt_ctrl);

	io_ctrl_reg = iaxxx_io_ctrl_data[io_port_mic][0];
	io_ctrl_mask = iaxxx_io_ctrl_data[io_port_mic][1];
	io_ctrl_val = iaxxx_io_ctrl_data[io_port_mic][1];

	snd_soc_write(codec, io_ctrl_reg, io_ctrl_val);

	snd_soc_update_bits(codec, cic_rx_addr[cic_rx_id],
		cic_rx_m_mask[cic_rx_id], cic_rx_dec);

	snd_soc_update_bits(codec, cic_rx_addr[cic_rx_id],
		cic_rx_clr_mask[cic_rx_id], ~cic_rx_reset);

	return 0;
}

static int iaxxx_pdm_port_clr(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol,
			int port, int port_mic)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 io_ctrl_reg, io_ctrl_mask;
	u32 port_type = 0;
	u32 rx_rt_clr = 0, tx_rt_clr = 0;
	u32 op_port_mic = 0;
	u32 status = 0;
	int try = IAXXX_MAX_RETRY, ret = 0;

	io_ctrl_reg = iaxxx_io_ctrl_data[port_mic][0];
	io_ctrl_mask = iaxxx_io_ctrl_data[port_mic][1];

	if (port_mic > PDM_CDC_DAC_OUT1) {
		pr_err("PDM port stop failed Invalid port number\n");
		return -EINVAL;
	}

	if ((port_mic >= PDM_DMIC_IN0) && (port_mic <= PDM_DMIC_IN7)) {
		/* PDM port is of DMIC type */
		port_type = PDM_PORT_DMIC;
	} else if ((port_mic >= PDM_CDC0_IN0) && (port_mic <= PDM_CDC3_IN7)) {
		/* PDM port is of ADC type */
		port_type = PDM_PORT_ADC;
		port_mic  = port_mic - PDM_CDC0_IN0;
	} else if ((port_mic >= PDM_DMIC_OUT0) && (port_mic <= PDM_DMIC_OUT1)) {
		/* PDM port is of Output type */
		port_type = PDM_PORT_PDMO;
	}

	if ((port_type == PDM_PORT_DMIC) || (port_type == PDM_PORT_ADC)) {
		/* Set state bit, Disable the Filter */
		if ((port_mic & 0x1) != 0) {
			/* we are in the 'odd' channel : 1/3/5/7 */
			snd_soc_update_bits(codec, cic_rx_addr[port_mic],
				IAXXX_CNR0_CIC_RX_0_1_CLR_1_MASK,
				(1 << IAXXX_CNR0_CIC_RX_0_1_CLR_1_POS));
		} else {
			/* we are in even channels : 0/2/4/6 */
			snd_soc_update_bits(codec, cic_rx_addr[port_mic],
				IAXXX_CNR0_CIC_RX_0_1_CLR_0_MASK,
				(1 << IAXXX_CNR0_CIC_RX_0_1_CLR_0_POS));
		}

		/* Set up the IO_CTRL appropriately */
		snd_soc_update_bits(codec, io_ctrl_reg, io_ctrl_mask, 0);

		/* Reset and update routing and disable the Clock to the CIC*/
		rx_rt_clr = (IAXXX_CIC_RX_RT_CTRL_OPT_MASK <<
			(IAXXX_CIC_RX_RT_CTRL_OPT_SIZE * port_mic));
		snd_soc_update_bits(codec, IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
							rx_rt_clr, 0);
	} else if (port_type == PDM_PORT_PDMO) {
		/* get the output port mic number */
		op_port_mic = port_mic - PDM_DMIC_OUT0;
		snd_soc_update_bits(codec, IAXXX_CNR0_CIC_TX_0_1_ADDR,
			(IAXXX_CNR0_CIC_TX_0_1_CLR_0_MASK << op_port_mic), 0);

		/* Set up the IO_CTRL appropriately */
		snd_soc_update_bits(codec, io_ctrl_reg, io_ctrl_mask, 0);

		/* Reset  and Disable Clock to the CIC */
		tx_rt_clr = ((IAXXX_CNR0_CIC_TX_RT_CTRL_CLK_EN_0_MASK) |
				(IAXXX_CNR0_CIC_TX_RT_CTRL_CLK_EN_1_MASK));
		snd_soc_update_bits(codec, IAXXX_CNR0_CIC_TX_RT_CTRL_ADDR,
						tx_rt_clr, 0);
	}

	/* DISABLE DMIC */
	while (snd_soc_read(codec, dmic_busy_addr[port]) &&
					(try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: DMIC busy enable bit busy :%d\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	/* DMIC enable */
	snd_soc_update_bits(codec, dmic_enable_addr[port],
			(IAXXX_DMIC_ENABLE_MASK << port_mic), 0);

	try = IAXXX_MAX_RETRY;
	/* DISABLE DMIC */
	while (snd_soc_read(codec, dmic_busy_addr[port]) &&
					(try-- != 0))
		usleep_range(2000, 2005);

	if (!try) {
		pr_err("%s: DMIC busy enable bit busy :%d\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	if (port_type == PDM_PORT_DMIC) {
		snd_soc_update_bits(codec, IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
						(1 << port_mic), 0);
	} else if (port_type == PDM_PORT_PDMO) {
		snd_soc_update_bits(codec, IAXXX_SRB_PDMO_PORT_PWR_EN_ADDR,
						(1 << op_port_mic),  0);
	}

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
					IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	return 0;
}

static int iaxxx_portb_mic0_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_PORTB;

	if (iaxxx->portb_mic0_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portb_mic0_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portb_mic0_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTB,
						CIC4, PDM_DMIC_IN4);
#else
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTB,
						CIC6, PDM_DMIC_IN4);
#endif
	} else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTB);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTB;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN4);
	}
}

static int iaxxx_portb_mic0_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic0_en;
	return 0;
}

static int iaxxx_portb_mic1_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
	int mstr_port = PDM_PORTB;
#endif

	if (iaxxx->portb_mic1_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portb_mic1_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portb_mic1_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTB,
						CIC5, PDM_DMIC_IN5);
#else
		return 0;
#endif
	} else {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTB);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTB;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN5);
#else
		return 0;
#endif
	}
}

static int iaxxx_portb_mic1_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic1_en;
	return 0;
}

static int iaxxx_portb_mic2_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_PORTB;

	if (iaxxx->portb_mic2_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portb_mic2_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portb_mic2_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTB,
						CIC6, PDM_DMIC_IN6);
#else
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTB,
						CIC7, PDM_DMIC_IN6);
#endif
	} else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTB);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTB;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN6);
	}
}

static int iaxxx_portb_mic2_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic2_en;
	return 0;
}

static int iaxxx_portb_mic3_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
	int mstr_port = PDM_PORTB;
#endif

	if (iaxxx->portb_mic3_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portb_mic3_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portb_mic3_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTB,
						CIC7, PDM_DMIC_IN7);
#else
		return 0;
#endif
	} else {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTB);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTB;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN7);
#else
		return 0;
#endif
	}
}

static int iaxxx_portb_mic3_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_mic3_en;
	return 0;
}

static int iaxxx_portc_mic0_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_PORTC;

	if (iaxxx->portc_mic0_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portc_mic0_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portc_mic0_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTC,
						CIC0, PDM_DMIC_IN0);
#else
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTC,
						CIC4, PDM_DMIC_IN0);
#endif
	} else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTC);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTC;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN0);
	}
}

static int iaxxx_portc_mic0_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic0_en;
	return 0;
}

static int iaxxx_portc_mic1_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
	int mstr_port = PDM_PORTC;
#endif

	if (iaxxx->portc_mic1_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portc_mic1_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portc_mic1_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTC,
						CIC1, PDM_DMIC_IN1);
#else
		return 0;
#endif
	} else {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTC);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTC;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN1);
#else
		return 0;
#endif
	}
}

static int iaxxx_portc_mic1_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic1_en;
	return 0;
}

static int iaxxx_portc_mic2_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_PORTC;

	if (iaxxx->portc_mic2_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portc_mic2_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portc_mic2_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTC,
						CIC2, PDM_DMIC_IN2);
#else
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTC,
						CIC5, PDM_DMIC_IN2);
#endif
	} else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTC);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTC;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN2);
	}
}

static int iaxxx_portc_mic2_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic2_en;
	return 0;
}

static int iaxxx_portc_mic3_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
	int mstr_port = PDM_PORTC;
#endif

	if (iaxxx->portc_mic3_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portc_mic3_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portc_mic3_en) {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTC,
						CIC3, PDM_DMIC_IN3);
#else
		return 0;
#endif
	} else {
#ifdef CONFIG_IAXXX_PDM_DMIC_MODE
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTC);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTC;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_DMIC_IN3);
#else
		return 0;
#endif
	}
}

static int iaxxx_portc_mic3_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_mic3_en;
	return 0;
}


/* CDC start*/
static int iaxxx_cdc0_mic0_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_CDC;

	if (iaxxx->cdc0_mic0_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->cdc0_mic0_en =  ucontrol->value.integer.value[0];
	if (iaxxx->cdc0_mic0_en)
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_CDC,
						CIC0, PDM_CDC0_IN0);
	else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_CDC);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_CDC;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_CDC0_IN0);
	}
}

static int iaxxx_cdc0_mic0_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->cdc0_mic0_en;
	return 0;
}

static int iaxxx_cdc0_mic1_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_CDC;

	if (iaxxx->cdc0_mic1_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->cdc0_mic1_en =  ucontrol->value.integer.value[0];

	if (iaxxx->cdc0_mic1_en)
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_CDC,
						CIC1, PDM_CDC1_IN1);
	else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_CDC);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_CDC;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_CDC1_IN1);
	}
}

static int iaxxx_cdc0_mic1_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->cdc0_mic1_en;
	return 0;
}

static int iaxxx_cdc0_mic2_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_CDC;

	if (iaxxx->cdc0_mic2_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->cdc0_mic2_en =  ucontrol->value.integer.value[0];

	if (iaxxx->cdc0_mic2_en)
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_CDC,
						CIC2, PDM_CDC2_IN2);
	else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_CDC);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_CDC;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_CDC2_IN2);
	}
}

static int iaxxx_cdc0_mic2_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->cdc0_mic2_en;
	return 0;
}

static int iaxxx_cdc1_mic0_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	int mstr_port = PDM_PORTD;

	if (iaxxx->cdc1_mic0_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->cdc1_mic0_en =  ucontrol->value.integer.value[0];

	if (iaxxx->cdc1_mic0_en)
		return iaxxx_pdm_mic_setup(kcontrol, ucontrol, PDM_PORTD,
						CIC3, PDM_CDC3_IN3);
	else {
		mstr_port = iaxxx_pdm_port_clk_src(iaxxx, PDM_PORTD);
		if (mstr_port > IAXXX_MAX_PDM_PORTS)
			mstr_port = PDM_PORTD;
		return iaxxx_pdm_port_clr(kcontrol, ucontrol, mstr_port,
						PDM_CDC3_IN3);
	}
}

static int iaxxx_cdc1_mic0_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->cdc1_mic0_en;
	return 0;
}
/* CDC end */

static int iaxxx_pdm_port_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u8 aud_port_clk = 0;
	u32 period = 0, div_val = 0, nr_val = 0, fs_sync_active = 0;
	u32 pdm_bclk = 0, port_sample_rate = 0, port_bits_per_frame = 0;
	u32 clk_ctrl_val = 0;
	u32 ao_clk_cfg_val = 0;
	u32 ao_clk_cfg_mask = 0, status = 0;
	int ret;

	if (iaxxx->port_filter[port] == ucontrol->value.integer.value[0])
		return 0;
	/*
	 * 1. Configure I2S master if chip is master
	 * 2. Enable AO CLK CFG
	 * 3. Configure ports registers
	 * 4. Configure CIC filter register
	 * 5. Enable Dmic clk
	 */
	if (port == PDM_PORTB) {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_PORTB_CLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_PORTB_CLK_OE_MASK;
	} else if (port == PDM_CDC) {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_CDC_MCLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_CDC_MCLK_OE_MASK;
	} else if (port == PDM_PORTC) {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_PORTC_CLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_PORTC_CLK_OE_MASK;
	} else if (port == PDM_PORTD) {
		ao_clk_cfg_val = IAXXX_AO_BCLK_ENABLE <<
					IAXXX_AO_CLK_CFG_PORTD_CLK_OE_POS;
		ao_clk_cfg_mask = IAXXX_AO_CLK_CFG_PORTD_CLK_OE_MASK;
	} else {
		pr_err("%s: Wrong PDM Port Configuration :%d port req:%d\n",
			__func__, __LINE__, port);
		return -EINVAL;
	}

	snd_soc_update_bits(codec, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << port), (0x1 << port));

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}
	/* Configure I2S clock */
	pdm_bclk = iaxxx->pdm_bclk;
	port_sample_rate = pdm_cfg[pdm_bclk].sample_rate;
	port_bits_per_frame = pdm_cfg[pdm_bclk].bits_per_frame - 1;
	iaxxx_calc_i2s_div(port_bits_per_frame, port_sample_rate,
				&period, &div_val, &nr_val);
	aud_port_clk = iaxxx->pdm_aclk;
	/* Store value to use while system resume */
	clk.port_bits_per_frame = port_bits_per_frame;
	clk.div_val = div_val;
	clk.nr_val = nr_val;
	clk.period = period;

	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(port),
		IAXXX_CNR0_I2S_ENABLE_LOW << port);

	/* I2S Trigger - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);

	/* disable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_DISABLE);
	/* Set HL value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_P_MASK,
		div_val);
	/* enable hl divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_HL_ADDR(port),
		IAXXX_I2S_I2S0_HL_EN_MASK, IAXXX_I2S_I2S0_HL_ENABLE);
	/* disable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_DISABLE);
	/* Set NR value */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_WMASK_VAL, nr_val);
	/* enable NR divider */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_NR_ADDR(port),
		IAXXX_I2S_I2S0_NR_EN_MASK, IAXXX_I2S_I2S0_NR_ENABLE);


	/*Bit 0 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S2_GEN_CFG_PCM_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	/* Bit 1 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLK_POL_MASK,
		IAXXX_I2S_GEN_CFG_CLK_POL_LOW);
	/* Bit 2*/
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_FS_POL_MASK,
		IAXXX_I2S_GEN_CFG_FS_POL_LOW);
	/* Bit 3 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_ABORT_ON_SYNC_MASK,
		IAXXX_I2S_GEN_CFG_ABORT_ON_SYNC_DISABLE);
	/* Bit 11:4 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_MASK,
		((port_bits_per_frame) <<
		IAXXX_I2S_I2S0_GEN_CFG_I2S_CLKS_PER_FS_POS));
	/* For PDM FS is assumed 0 */
	fs_sync_active = (0 << IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_POS);
	/* Bit 19:12 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_FS_VALID_MASK, fs_sync_active);
	/* Bit 20 */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_GEN_CFG_ADDR(port),
		IAXXX_I2S_I2S0_GEN_CFG_GEN_MASTER_MASK,
		IAXXX_I2S_GEN_CFG_GEN_MASTER_MODE);

	/* Clk control */
	clk_ctrl_val = (clk_ctrl_val | ((period/2 - 1) <<
			IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_LOW_POS)|((period - 1)
			<< IAXXX_I2S_I2S0_CLK_CTRL_I2S_CLK_PERIOD_POS));

	snd_soc_update_bits(codec, IAXXX_I2S_I2S_CLK_CTRL_ADDR(port),
		IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL, clk_ctrl_val);

	/* FS Align */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_FS_ALIGN_ADDR(port),
		IAXXX_I2S_I2S0_FS_ALIGN_WMASK_VAL,
		IAXXX_I2S_FS_ALIGN_MASTER_MODE);

	/* AO CLK Config */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		ao_clk_cfg_mask, ao_clk_cfg_val);

	/* CNR0_I2S_Enable  - Disable I2S1 Bit 1 */
	snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
		IAXXX_CNR0_I2S_ENABLE_MASK(port),
		IAXXX_CNR0_I2S_ENABLE_HIGH << port);

	/* I2S Trigger - Disable I2S */
	snd_soc_update_bits(codec, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
		IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
		IAXXX_I2S_TRIGGER_HIGH);

	iaxxx->port_filter[port] = 1;
	return 0;
}

#define IAXXX_PDM_PORT_SETUP_SET_GET(port_name, port) \
static int iaxxx_pdm_##port_name##_put(struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec); \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_pdm_port_setup(kcontrol, ucontrol, port); \
	else { \
		iaxxx->port_filter[port] = 0; \
		return 0; \
	} \
} \
static int iaxxx_pdm_##port_name##_get(struct snd_kcontrol *kcontrol, \
	struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_filter[port]; \
	return 0; \
}

IAXXX_PDM_PORT_SETUP_SET_GET(portb, PDM_PORTB);
IAXXX_PDM_PORT_SETUP_SET_GET(portc, PDM_PORTC);
IAXXX_PDM_PORT_SETUP_SET_GET(cdc, PDM_CDC);
IAXXX_PDM_PORT_SETUP_SET_GET(cdc1, PDM_PORTD);

static int iaxxx_portb_micbias_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	if (iaxxx->portb_micbias_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portb_micbias_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portb_micbias_en) {
		/* set IOCTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_IO_CTRL_COMMF_1_ADDR,
				0x12);
		/* set PAD CTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_PAD_CTRL_COMMF_1_ADDR,
				0x70);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_1_MASK,
				IAXXX_GPIO_SWPORTB_DR_COMMF_1_MASK);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_1_MASK,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_1_MASK);
	} else {
		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_1_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DR_COMMF_1_POS);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_1_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DDR_COMMF_1_POS);
	}
	return 0;
}

static int iaxxx_portb_micbias_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portb_micbias_en;
	return 0;
}

static int iaxxx_portc_micbias_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	if (iaxxx->portc_micbias_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->portc_micbias_en =  ucontrol->value.integer.value[0];

	if (iaxxx->portc_micbias_en) {
		/* set IOCTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_IO_CTRL_COMMF_0_ADDR,
				0x12);

		/* set PAD CTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_PAD_CTRL_COMMF_0_ADDR,
				0x70);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_0_MASK,
				IAXXX_GPIO_SWPORTB_DR_COMMF_0_MASK);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_0_MASK,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_0_MASK);

	} else {
		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_0_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DR_COMMF_0_POS);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_0_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DDR_COMMF_0_POS);
	}
	return 0;
}

static int iaxxx_portc_micbias_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->portc_micbias_en;
	return 0;
}

static int iaxxx_cdc0_micbias_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	if (iaxxx->cdc0_micbias_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->cdc0_micbias_en =  ucontrol->value.integer.value[0];

	if (iaxxx->cdc0_micbias_en) {
		/* set IOCTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_IO_CTRL_COMMF_3_ADDR,
				0x12);

		/* set PAD CTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_PAD_CTRL_COMMF_3_ADDR,
				0x70);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_3_MASK,
				IAXXX_GPIO_SWPORTB_DR_COMMF_3_MASK);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_3_MASK,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_3_MASK);
	} else {
		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_3_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DR_COMMF_3_POS);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_3_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DDR_COMMF_3_POS);
	}
	return 0;
}

static int iaxxx_cdc0_micbias_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->cdc0_micbias_en;
	return 0;
}

static int iaxxx_cdc1_micbias_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	if (iaxxx->cdc1_micbias_en == ucontrol->value.integer.value[0])
		return 0;

	iaxxx->cdc1_micbias_en =  ucontrol->value.integer.value[0];

	if (iaxxx->cdc1_micbias_en) {
		/* set IOCTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_IO_CTRL_COMMF_2_ADDR,
				0x12);

		/* set PAD CTRL to GPIO mode */
		snd_soc_write(codec, IAXXX_PAD_CTRL_COMMF_2_ADDR,
				0x70);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_2_MASK,
				IAXXX_GPIO_SWPORTB_DR_COMMF_2_MASK);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_2_MASK,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_2_MASK);
	} else {
		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_2_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DR_COMMF_2_POS);

		snd_soc_update_bits(codec, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_2_MASK,
				0x0 < IAXXX_GPIO_SWPORTB_DDR_COMMF_2_POS);
	}
	return 0;
}

static int iaxxx_cdc1_micbias_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->cdc1_micbias_en;
	return 0;
}

static int iaxxx_pdm_head_strm_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);
	u32 value = 0;

	iaxxx->head_of_strm_all =  ucontrol->value.integer.value[0];
	if (iaxxx->head_of_strm_all)
		value = iaxxx->head_of_strm_all;

	/* CIC filter config */
	snd_soc_update_bits(codec, IAXXX_CNR0_CIC_RX_HOS_ADDR,
		IAXXX_CNR0_CIC_RX_HOS_MASK_VAL, value);

	return 0;
}

static int iaxxx_pdm_head_strm_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->head_of_strm_all;
	return 0;
}

static int iaxxx_pcm_port_stop(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	pr_debug("%s() port:%d mstrclk:%d\n",
			__func__, port, iaxxx->is_ip_port_master[port]);
	if (iaxxx->port_pcm_start[port] ==  ucontrol->value.integer.value[0])
		return 0;

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(port),
		IAXXX_AO_BCLK_DISABLE <<
			IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(port));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(port),
		IAXXX_AO_FS_DISABLE << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(port));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(port),
		IAXXX_AO_DO_DISABLE << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(port));

	if (iaxxx->is_ip_port_master[port]) {
		/* CNR0_I2S_Enable  - Disable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(port),
			IAXXX_CNR0_I2S_ENABLE_LOW << port);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(port),
		IAXXX_CNR0_PCM_DISABLE <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(port));

	iaxxx->port_pcm_start[port] = 0;
	return 0;
}

static int iaxxx_pcm_port_start(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 word_len = 0;
	u32 frame_len = 0;
	u32 channel_val = 0;
	u32 sampling_rate = 0;
	u32 ao_bclk_val = 0;
	u32 ao_fs_val = 0;
	int count;

	pr_debug("%s() port:%d mstrclk:%d\n",
			__func__, port, iaxxx->is_ip_port_master[port]);

	if (iaxxx->port_pcm_start[port] == ucontrol->value.integer.value[0])
		return 0;

	/* Parse input values */
	word_len    = (ucontrol->value.integer.value[0] & 0x1F);
	channel_val = (ucontrol->value.integer.value[0] & 0x1FE0) >> 5;
	frame_len = channel_val;

	for (count = 0; frame_len != 0; count++)
		frame_len &= frame_len-1;

	frame_len =  count - 1;
	pr_debug("%s() word_len:%u channel_val:%u frame_len: %u\n",
			__func__, word_len, channel_val, frame_len);

	snd_soc_update_bits(codec, IAXXX_PCM_SWLR_ADDR(port),
		IAXXX_PCM0_SWLR_WMASK_VAL, word_len);

	snd_soc_update_bits(codec, IAXXX_PCM_SRSA_ADDR(port),
		IAXXX_PCM0_SRSA_WMASK_VAL, channel_val);

	snd_soc_update_bits(codec, IAXXX_PCM_STSA_ADDR(port),
		IAXXX_PCM0_STSA_WMASK_VAL, channel_val);

	snd_soc_update_bits(codec, IAXXX_PCM_SFLR_ADDR(port),
		IAXXX_PCM0_SFLR_WMASK_VAL, frame_len);

	if (iaxxx->is_ip_port_master[port]) {
		sampling_rate = 16000;
		iaxxx_set_i2s_controller(codec, sampling_rate, false, port);
		ao_bclk_val = IAXXX_AO_BCLK_ENABLE;
		ao_fs_val = IAXXX_AO_FS_ENABLE;
	} else {
		ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
		ao_fs_val = IAXXX_AO_FS_DISABLE;
	}
	/* Set Port  clk, FS, DO reg */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(port),
		ao_bclk_val << IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(port));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(port),
		ao_fs_val << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(port));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(port),
		IAXXX_AO_DO_ENABLE <<
			IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(port));
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(port),
		IAXXX_CNR0_PCM_ENABLE <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(port));

	if (iaxxx->is_ip_port_master[port]) {
		/* CNR0_I2S_Enable  - Enable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(port),
			IAXXX_CNR0_I2S_ENABLE_HIGH << port);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}

	iaxxx->port_pcm_start[port] = ucontrol->value.integer.value[0];
	return 0;
}

static int iaxxx_pcm_port_setup(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int port)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);
	u32 mode = 0;
	u32 reg_srdd_val = 0;
	u32 port_clk_val = 0;
	u32 port_di_val = 0;
	u32 port_do_val = 0;
	u32 port_fs_val = 0;
	u32 status = 0;
	int ret;

	pr_debug("%s() port:%d mstrclk:%d\n",
			__func__, port, iaxxx->is_ip_port_master[port]);
	if (iaxxx->port_pcm_setup[port] ==  ucontrol->value.integer.value[0])
		return 0;

	/* Format for port master slave configuration */
	if (iaxxx->is_ip_port_master[port]) {
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_MASTER;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_MASTER;
	} else {
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_SLAVE;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_SLAVE;
	}

	/* port mode I2S, DSP or TDM mode */
	if (iaxxx->pcm_port_fmt[port] == 0) {
		reg_srdd_val = 1;
		mode = IAXXX_PCM_CTRL_DEFAULT_I2SFMT;
	} else if (iaxxx->pcm_port_fmt[port] == 1) {
		reg_srdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_TDMFMT;
	} else if (iaxxx->pcm_port_fmt[port] == 2) {
		reg_srdd_val = 0;
		mode = IAXXX_PCM_CTRL_DEFAULT_DSPFMT;
	} else {
		dev_err(codec->dev, "unsupported format\n");
		return -EINVAL;
	}

	port_di_val = port_di_val | IAXXX_IO_CTRL_DI;
	port_do_val = port_do_val | IAXXX_IO_CTRL_DO;
	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
				(0x1 << port), (0x1 << port));

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	snd_soc_update_bits(codec, IAXXX_PCM_SRDD_ADDR(port),
		IAXXX_PCM0_SRDD_WMASK_VAL, reg_srdd_val);

	snd_soc_update_bits(codec, IAXXX_PCM_MC_ADDR(port),
		IAXXX_PCM0_MC_WMASK_VAL, mode);

	snd_soc_update_bits(codec, port_clk_addr[port],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_fs_addr[port],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_di_addr[port],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_do_addr[port],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, port_do_val);

	iaxxx->port_pcm_setup[port] =  ucontrol->value.integer.value[0];
	return 0;
}

#define IAXXX_PCM_PORT_SET_GET(port_name, port) \
static int iaxxx_pcm_##port_name##_start_put( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_pcm_port_start(kcontrol, ucontrol, port); \
	else \
		return iaxxx_pcm_port_stop(kcontrol, ucontrol, port); \
} \
static int iaxxx_pcm_##port_name##_start_get( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_pcm_start[port]; \
	return 0; \
} \
static int iaxxx_pcm_##port_name##_setup_put( \
			struct snd_kcontrol *kcontrol, \
			struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec); \
	if (ucontrol->value.integer.value[0]) \
		return iaxxx_pcm_port_setup(kcontrol, ucontrol, port); \
	else { \
		iaxxx->port_pcm_setup[port] =  \
			ucontrol->value.integer.value[0]; \
		return 0; \
	} \
} \
static int iaxxx_pcm_##port_name##_setup_get( \
		struct snd_kcontrol *kcontrol, \
		struct snd_ctl_elem_value *ucontrol) \
{ \
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol); \
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev); \
	ucontrol->value.integer.value[0] = iaxxx->port_pcm_setup[port]; \
	return 0; \
}

IAXXX_PCM_PORT_SET_GET(porta, PCM_PORTA)
IAXXX_PCM_PORT_SET_GET(portb, PCM_PORTB)
IAXXX_PCM_PORT_SET_GET(portc, PCM_PORTC)
IAXXX_PCM_PORT_SET_GET(portd, PCM_PORTD)
IAXXX_PCM_PORT_SET_GET(porte, PCM_PORTE)
IAXXX_PCM_PORT_SET_GET(portf, PCM_PORTF)

static int iaxxx_cdc_dmic_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	return iaxxx->cdc_dmic_enable =  ucontrol->value.integer.value[0];

}

static int iaxxx_cdc_dmic_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(codec->dev);

	ucontrol->value.integer.value[0] = iaxxx->cdc_dmic_enable;
	return 0;
}

static const struct snd_kcontrol_new iaxxx_snd_controls[] = {

	IAXXXCORE_RX_CHMGR_KCTRL(RX_0, "Rx0"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_1, "Rx1"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_2, "Rx2"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_3, "Rx3"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_4, "Rx4"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_5, "Rx5"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_6, "Rx6"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_7, "Rx7"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_8, "Rx8"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_9, "Rx9"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_10, "Rx10"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_11, "Rx11"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_12, "Rx12"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_13, "Rx13"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_14, "Rx14"),
	IAXXXCORE_RX_CHMGR_KCTRL(RX_15, "Rx15"),

	IAXXXCORE_TX_CHMGR_KCTRL(TX_0, "Tx0"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_1, "Tx1"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_2, "Tx2"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_3, "Tx3"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_4, "Tx4"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_5, "Tx5"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_6, "Tx6"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_7, "Tx7"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_8, "Tx8"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_9, "Tx9"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_10, "Tx10"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_11, "Tx11"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_12, "Tx12"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_13, "Tx13"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_14, "Tx14"),
	IAXXXCORE_TX_CHMGR_KCTRL(TX_15, "Tx15"),

	IAXXXCORE_STREAM_KCTRL(STREAM0, "strm0", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM1, "strm1", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM2, "strm2", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM3, "strm3", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM4, "strm4", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM5, "strm5", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM6, "strm6", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM7, "strm7", 0),
	IAXXXCORE_STREAM_KCTRL(STREAM8, "strm8", 16),
	IAXXXCORE_STREAM_KCTRL(STREAM9, "strm9", 16),
	IAXXXCORE_STREAM_KCTRL(STREAM10, "strm10", 16),
	IAXXXCORE_STREAM_KCTRL(STREAM11, "strm11", 16),
	IAXXXCORE_STREAM_KCTRL(STREAM12, "strm12", 16),
	IAXXXCORE_STREAM_KCTRL(STREAM13, "strm13", 16),
	IAXXXCORE_STREAM_KCTRL(STREAM14, "strm14", 16),
	IAXXXCORE_STREAM_KCTRL(STREAM15, "strm15", 16),

	IAXXXCORE_PLUGIN_KCTRL(PLUGIN0, "Plgin0"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN1, "Plgin1"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN2, "Plgin2"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN3, "Plgin3"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN4, "Plgin4"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN5, "Plgin5"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN6, "Plgin6"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN7, "Plgin7"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN8, "Plgin8"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN9, "Plgin9"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN10, "Plgin10"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN11, "Plgin11"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN12, "Plgin12"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN13, "Plgin13"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN14, "Plgin14"),
	IAXXXCORE_PLUGIN_KCTRL(PLUGIN15, "Plgin15"),


	IAXXX_PLUGIN_EN_CTLS(PLUGIN0, "Plgin0"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN1, "Plgin1"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN2, "Plgin2"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN3, "Plgin3"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN4, "Plgin4"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN5, "Plgin5"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN6, "Plgin6"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN7, "Plgin7"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN8, "Plgin8"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN9, "Plgin9"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN10, "Plgin10"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN11, "Plgin11"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN12, "Plgin12"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN13, "Plgin13"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN14, "Plgin14"),
	IAXXX_PLUGIN_EN_CTLS(PLUGIN15, "Plgin15"),


	SOC_SINGLE_BOOL_EXT("Update Block0 Req", 0,
		iaxxx_get_update_block0, iaxxx_put_update_block0),
	SOC_SINGLE_BOOL_EXT("Update Block1 Req", 0,
		iaxxx_get_update_block1, iaxxx_put_update_block1),
	SOC_SINGLE_BOOL_EXT("Update Block2 Req", 0,
		iaxxx_get_update_block2, iaxxx_put_update_block2),
	SOC_ENUM_EXT("PDM BCLK", iaxxx_pdm_bclk_enum,
		       iaxxx_get_pdm_bclk,
		       iaxxx_put_pdm_bclk),
	SOC_ENUM_EXT("PDM Port ACLK", iaxxx_pdm_aclk_enum,
		       iaxxx_get_pdm_aclk,
		       iaxxx_put_pdm_aclk),

	SOC_SINGLE_BOOL_EXT("CDC DMIC Enable", 0,
		iaxxx_cdc_dmic_get, iaxxx_cdc_dmic_put),

	SOC_SINGLE_BOOL_EXT("PCM PortA Setup", 0,
		iaxxx_pcm_porta_setup_get, iaxxx_pcm_porta_setup_put),
	SOC_SINGLE_BOOL_EXT("PCM PortB Setup", 0,
		iaxxx_pcm_portb_setup_get, iaxxx_pcm_portb_setup_put),
	SOC_SINGLE_BOOL_EXT("PCM PortC Setup", 0,
		iaxxx_pcm_portc_setup_get, iaxxx_pcm_portc_setup_put),
	SOC_SINGLE_BOOL_EXT("PCM PortD Setup", 0,
		iaxxx_pcm_portd_setup_get, iaxxx_pcm_portd_setup_put),
	SOC_SINGLE_BOOL_EXT("PCM PortE Setup", 0,
		iaxxx_pcm_porte_setup_get, iaxxx_pcm_porte_setup_put),
	SOC_SINGLE_BOOL_EXT("PCM PortF Setup", 0,
		iaxxx_pcm_portf_setup_get, iaxxx_pcm_portf_setup_put),

	/* Upper mask 0x60 represent the channels input and output
	 * Lower mask 0x1F represent the bits used per channel
	 */
	SOC_SINGLE_EXT("PCM PortA Start", SND_SOC_NOPM, 0, 0x1FFF, 0,
		iaxxx_pcm_porta_start_get, iaxxx_pcm_porta_start_put),
	SOC_SINGLE_EXT("PCM PortB Start", SND_SOC_NOPM, 0, 0x1FFF, 0,
		iaxxx_pcm_portb_start_get, iaxxx_pcm_portb_start_put),
	SOC_SINGLE_EXT("PCM PortC Start", SND_SOC_NOPM, 0, 0x1FFF, 0,
		iaxxx_pcm_portc_start_get, iaxxx_pcm_portc_start_put),
	SOC_SINGLE_EXT("PCM PortD Start", SND_SOC_NOPM, 0, 0x1FFF, 0,
		iaxxx_pcm_portd_start_get, iaxxx_pcm_portd_start_put),
	SOC_SINGLE_EXT("PCM PortE Start", SND_SOC_NOPM, 0, 0x1FFF, 0,
		iaxxx_pcm_porte_start_get, iaxxx_pcm_porte_start_put),
	SOC_SINGLE_EXT("PCM PortF Start", SND_SOC_NOPM, 0, 0x1FFF, 0,
		iaxxx_pcm_portf_start_get, iaxxx_pcm_portf_start_put),

	SOC_SINGLE_BOOL_EXT("Pdm PortB DMic0 En", 0,
		iaxxx_portb_mic0_get, iaxxx_portb_mic0_put),
	SOC_SINGLE_BOOL_EXT("Pdm PortB DMic1 En", 0,
		iaxxx_portb_mic1_get, iaxxx_portb_mic1_put),
	SOC_SINGLE_BOOL_EXT("Pdm PortB DMic2 En", 0,
			iaxxx_portb_mic2_get, iaxxx_portb_mic2_put),
	SOC_SINGLE_BOOL_EXT("Pdm PortB DMic3 En", 0,
			iaxxx_portb_mic3_get, iaxxx_portb_mic3_put),

	SOC_SINGLE_BOOL_EXT("Pdm PortC DMic0 En", 0,
			iaxxx_portc_mic0_get, iaxxx_portc_mic0_put),
	SOC_SINGLE_BOOL_EXT("Pdm PortC DMic1 En", 0,
			iaxxx_portc_mic1_get, iaxxx_portc_mic1_put),
	SOC_SINGLE_BOOL_EXT("Pdm PortC DMic2 En", 0,
			iaxxx_portc_mic2_get, iaxxx_portc_mic2_put),
	SOC_SINGLE_BOOL_EXT("Pdm PortC DMic3 En", 0,
			iaxxx_portc_mic3_get, iaxxx_portc_mic3_put),

	SOC_SINGLE_BOOL_EXT("Pdm CDC0 DMic0 En", 0,
			iaxxx_cdc0_mic0_get, iaxxx_cdc0_mic0_put),
	SOC_SINGLE_BOOL_EXT("Pdm CDC0 DMic1 En", 0,
			iaxxx_cdc0_mic1_get, iaxxx_cdc0_mic1_put),
	SOC_SINGLE_BOOL_EXT("Pdm CDC0 DMic2 En", 0,
			iaxxx_cdc0_mic2_get, iaxxx_cdc0_mic2_put),

	SOC_SINGLE_BOOL_EXT("Pdm CDC1 DMic0 En", 0,
			iaxxx_cdc1_mic0_get, iaxxx_cdc1_mic0_put),

	SOC_SINGLE_BOOL_EXT("Pdm PortB Setup", 0,
		iaxxx_pdm_portb_get, iaxxx_pdm_portb_put),
	SOC_SINGLE_BOOL_EXT("Pdm PortC Setup", 0,
		iaxxx_pdm_portc_get, iaxxx_pdm_portc_put),
	SOC_SINGLE_BOOL_EXT("Pdm CDC Setup", 0,
			iaxxx_pdm_cdc_get, iaxxx_pdm_cdc_put),
	SOC_SINGLE_BOOL_EXT("Pdm CDC1 Setup", 0,
			iaxxx_pdm_cdc1_get, iaxxx_pdm_cdc1_put),

	/* Enable DMIC0_CLK paths. For TX ports [9:8] are 0R/0L,
	 * for RX ports [7/6/5/4/3/2/1/0] are 3R/3L/2R/2L/1R/1L/0R/0L
	 * -> L is left channel, R is right channel.
	 */
	SOC_SINGLE_EXT("PDM PortC Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_portc, iaxxx_put_start_portc),
	SOC_SINGLE_EXT("PDM PortB Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_portb, iaxxx_put_start_portb),
	SOC_SINGLE_EXT("PDM CDC0 Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_cdc0, iaxxx_put_start_cdc0),
	SOC_SINGLE_EXT("PDM CDC1 Start", SND_SOC_NOPM, 0, 0x3FF, 0,
			iaxxx_get_start_cdc1, iaxxx_put_start_cdc1),

	SOC_SINGLE_BOOL_EXT("PortB MicBias", 0,
		iaxxx_portb_micbias_get, iaxxx_portb_micbias_put),
	SOC_SINGLE_BOOL_EXT("PortC MicBias", 0,
		iaxxx_portc_micbias_get, iaxxx_portc_micbias_put),
	SOC_SINGLE_BOOL_EXT("CDC0 MicBias", 0,
		iaxxx_cdc0_micbias_get, iaxxx_cdc0_micbias_put),
	SOC_SINGLE_BOOL_EXT("CDC1 MicBias", 0,
		iaxxx_cdc1_micbias_get, iaxxx_cdc1_micbias_put),

	SOC_SINGLE_EXT("PDM Hos", SND_SOC_NOPM, 0, 0xFF, 0,
			iaxxx_pdm_head_strm_get, iaxxx_pdm_head_strm_put),
	SOC_ENUM_EXT("Route Status", iaxxx_route_status_enum,
			iaxxx_get_route_status,
			iaxxx_put_route_status),
};

static int iaxxxcore_enable_i2srx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	pr_debug("event 0x%x, port id: %x", event, w->id);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	{
		pr_debug("SND_SOC_DAPM_POST_PMU event");
		break;
	}
	case SND_SOC_DAPM_POST_PMD:
	{
		pr_debug("SND_SOC_DAPM_POST_PMD event");
		break;
	}
	default:
		pr_err("Unknown event 0x%x", event);
		break;
	}

	return ret;
}

static int iaxxxcore_enable_i2stx(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	{
		pr_debug("SND_SOC_DAPM_POST_PMU event");
		break;
	}
	case SND_SOC_DAPM_POST_PMD:
	{
		/* Disable PCM ports post streaming */
		pr_debug("SND_SOC_DAPM_POST_PMD event");

		break;
	}
	default:
		pr_debug("Unknown event 0x%x", event);
		break;
	}

	return ret;
}

static const struct snd_soc_dapm_widget iaxxx_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN_E("PCM0.0 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.1 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.2 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM0.3 RX", "I2S PCM0 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM1.0 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.1 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.2 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM1.3 RX", "I2S PCM1 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM2.0 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.1 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.2 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM2.3 RX", "I2S PCM2 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM3.0 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM3.1 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM3.2 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM3.3 RX", "I2S PCM3 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM4.0 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM4.1 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM4.2 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM4.3 RX", "I2S PCM4 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("PCM5.0 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM5.1 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM5.2 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("PCM5.3 RX", "I2S PCM5 Rx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2srx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM0.0 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.1 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.2 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM0.3 TX", "I2S PCM0 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF0, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM1.0 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.1 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.2 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM1.3 TX", "I2S PCM1 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF1, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM2.0 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.1 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.2 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM2.3 TX", "I2S PCM2 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF2, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM3.0 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM3.1 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM3.2 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM3.3 TX", "I2S PCM3 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF3, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM4.0 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM4.1 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM4.2 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM4.3 TX", "I2S PCM4 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF4, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("PCM5.0 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM5.1 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM5.2 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("PCM5.3 TX", "I2S PCM5 Tx", 0,
			SND_SOC_NOPM, IAXXX_AIF5, 0, iaxxxcore_enable_i2stx,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	IAXXX_CH_MGR_DAPM_MUX(RX_0, "Rx0 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_1, "Rx1 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_2, "Rx2 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_3, "Rx3 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_4, "Rx4 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_5, "Rx5 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_6, "Rx6 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_7, "Rx7 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_8, "Rx8 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_9, "Rx9 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_10, "Rx10 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_11, "Rx11 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_12, "Rx12 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_13, "Rx13 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_14, "Rx14 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(RX_15, "Rx15 Mux"),

	IAXXX_CH_MGR_DAPM_MUX(TX_0, "Tx0 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_1, "Tx1 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_2, "Tx2 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_3, "Tx3 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_4, "Tx4 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_5, "Tx5 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_6, "Tx6 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_7, "Tx7 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_8, "Tx8 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_9, "Tx9 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_10, "Tx10 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_11, "Tx11 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_12, "Tx12 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_13, "Tx13 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_14, "Tx14 Mux"),
	IAXXX_CH_MGR_DAPM_MUX(TX_15, "Tx15 Mux"),

	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_0, "Tx0"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_1, "Tx1"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_2, "Tx2"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_3, "Tx3"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_4, "Tx4"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_5, "Tx5"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_6, "Tx6"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_7, "Tx7"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_8, "Tx8"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_9, "Tx9"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_10, "Tx10"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_11, "Tx11"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_12, "Tx12"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_13, "Tx13"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_14, "Tx14"),
	IAXXX_CH_RX_TO_TX_DAPM_MUX(TX_15, "Tx15"),

	IAXXX_PLUGIN_DAPM_MUX(PLUGIN0, "Plgin0"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN1, "Plgin1"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN2, "Plgin2"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN3, "Plgin3"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN4, "Plgin4"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN5, "Plgin5"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN6, "Plgin6"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN7, "Plgin7"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN8, "Plgin8"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN9, "Plgin9"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN10, "Plgin10"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN11, "Plgin11"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN12, "Plgin12"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN13, "Plgin13"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN14, "Plgin14"),
	IAXXX_PLUGIN_DAPM_MUX(PLUGIN15, "Plgin15"),

	SND_SOC_DAPM_OUTPUT("PCMOUTPUT1"),
	SND_SOC_DAPM_OUTPUT("PCMOUTPUT2"),

	SND_SOC_DAPM_INPUT("PCMINPUT0"),
	SND_SOC_DAPM_INPUT("PCMINPUT1"),
	SND_SOC_DAPM_INPUT("PCMINPUT2"),
	SND_SOC_DAPM_INPUT("PDMINPUT0"),
	SND_SOC_DAPM_INPUT("PDMINPUT1"),
};

#define PLUGIN_TO_TX_MUX_ROUTE(Plgin) \
	{"Tx0 PortMux En", Plgin"Tx0On", Plgin"En"}, \
	{"Tx1 PortMux En", Plgin"Tx1On", Plgin"En"}, \
	{"Tx2 PortMux En", Plgin"Tx2On", Plgin"En"}, \
	{"Tx3 PortMux En", Plgin"Tx3On", Plgin"En"}, \
	{"Tx4 PortMux En", Plgin"Tx4On", Plgin"En"}, \
	{"Tx5 PortMux En", Plgin"Tx5On", Plgin"En"}, \
	{"Tx6 PortMux En", Plgin"Tx6On", Plgin"En"}, \
	{"Tx7 PortMux En", Plgin"Tx7On", Plgin"En"}, \
	{"Tx8 PortMux En", Plgin"Tx8On", Plgin"En"}, \
	{"Tx9 PortMux En", Plgin"Tx9On", Plgin"En"}, \
	{"Tx10 PortMux En", Plgin"Tx10On", Plgin"En"}, \
	{"Tx11 PortMux En", Plgin"Tx11On", Plgin"En"}, \
	{"Tx12 PortMux En", Plgin"Tx12On", Plgin"En"}, \
	{"Tx13 PortMux En", Plgin"Tx13On", Plgin"En"}, \
	{"Tx14 PortMux En", Plgin"Tx14On", Plgin"En"}, \
	{"Tx15 PortMux En", Plgin"Tx15On", Plgin"En"}

#define RX_MUX_TO_TX_MUX_ROUTE(Tx) \
	{Tx" PortMux En", "Rx0"Tx"On", "Rx0 Mux Port"}, \
	{Tx" PortMux En", "Rx1"Tx"On", "Rx1 Mux Port"}, \
	{Tx" PortMux En", "Rx2"Tx"On", "Rx2 Mux Port"}, \
	{Tx" PortMux En", "Rx3"Tx"On", "Rx3 Mux Port"}, \
	{Tx" PortMux En", "Rx4"Tx"On", "Rx4 Mux Port"}, \
	{Tx" PortMux En", "Rx5"Tx"On", "Rx5 Mux Port"}, \
	{Tx" PortMux En", "Rx6"Tx"On", "Rx6 Mux Port"}, \
	{Tx" PortMux En", "Rx7"Tx"On", "Rx7 Mux Port"}, \
	{Tx" PortMux En", "Rx8"Tx"On", "Rx8 Mux Port"}, \
	{Tx" PortMux En", "Rx9"Tx"On", "Rx9 Mux Port"}, \
	{Tx" PortMux En", "Rx10"Tx"On", "Rx10 Mux Port"}, \
	{Tx" PortMux En", "Rx11"Tx"On", "Rx11 Mux Port"}, \
	{Tx" PortMux En", "Rx12"Tx"On", "Rx12 Mux Port"}, \
	{Tx" PortMux En", "Rx13"Tx"On", "Rx13 Mux Port"}, \
	{Tx" PortMux En", "Rx14"Tx"On", "Rx14 Mux Port"}, \
	{Tx" PortMux En", "Rx15"Tx"On", "Rx15 Mux Port"}

#define RX_MUX_TO_PLUGIN_ROUTE(name) \
	{"Plgin0En", name"Plgin0On", name" Mux Port"}, \
	{"Plgin1En", name"Plgin1On", name" Mux Port"}, \
	{"Plgin2En", name"Plgin2On", name" Mux Port"}, \
	{"Plgin3En", name"Plgin3On", name" Mux Port"}, \
	{"Plgin4En", name"Plgin4On", name" Mux Port"}, \
	{"Plgin5En", name"Plgin5On", name" Mux Port"}, \
	{"Plgin6En", name"Plgin6On", name" Mux Port"}, \
	{"Plgin7En", name"Plgin7On", name" Mux Port"}, \
	{"Plgin8En", name"Plgin8On", name" Mux Port"}, \
	{"Plgin9En", name"Plgin9On", name" Mux Port"}, \
	{"Plgin10En", name"Plgin10On", name" Mux Port"}, \
	{"Plgin11En", name"Plgin11On", name" Mux Port"}, \
	{"Plgin12En", name"Plgin12On", name" Mux Port"}, \
	{"Plgin13En", name"Plgin13On", name" Mux Port"}, \
	{"Plgin14En", name"Plgin14On", name" Mux Port"}, \
	{"Plgin15En", name"Plgin15On", name" Mux Port"}

#define PORT_TO_RX_MUX_ROUTE(name) \
	{name, "PCM0", "PCM0.0 RX"}, \
	{name, "PCM0", "PCM0.1 RX"}, \
	{name, "PCM0", "PCM0.2 RX"}, \
	{name, "PCM0", "PCM0.3 RX"}, \
	{name, "PCM1", "PCM1.0 RX"}, \
	{name, "PCM1", "PCM1.1 RX"}, \
	{name, "PCM1", "PCM1.2 RX"}, \
	{name, "PCM1", "PCM1.3 RX"}, \
	{name, "PCM2", "PCM2.0 RX"}, \
	{name, "PCM2", "PCM2.1 RX"}, \
	{name, "PCM2", "PCM2.2 RX"}, \
	{name, "PCM2", "PCM2.3 RX"}, \
	{name, "PCM3", "PCM3.0 RX"}, \
	{name, "PCM3", "PCM3.1 RX"}, \
	{name, "PCM3", "PCM3.2 RX"}, \
	{name, "PCM3", "PCM3.3 RX"}, \
	{name, "PCM4", "PCM4.0 RX"}, \
	{name, "PCM4", "PCM4.1 RX"}, \
	{name, "PCM4", "PCM4.2 RX"}, \
	{name, "PCM4", "PCM4.3 RX"}, \
	{name, "PCM5", "PCM5.0 RX"}, \
	{name, "PCM5", "PCM5.1 RX"}, \
	{name, "PCM5", "PCM5.2 RX"}, \
	{name, "PCM5", "PCM5.3 RX"}, \
	{name, "PDMI0", "PDMINPUT0"}, \
	{name, "PDMI1", "PDMINPUT1"}

#define TX_MUX_TO_PORT_ROUTE(name) \
	{"PCM0.0 TX", NULL, name}, \
	{"PCM0.1 TX", NULL, name}, \
	{"PCM0.2 TX", NULL, name}, \
	{"PCM0.3 TX", NULL, name}, \
	{"PCM1.0 TX", NULL, name}, \
	{"PCM1.1 TX", NULL, name}, \
	{"PCM1.2 TX", NULL, name}, \
	{"PCM1.3 TX", NULL, name}, \
	{"PCM2.0 TX", NULL, name}, \
	{"PCM2.1 TX", NULL, name}, \
	{"PCM2.2 TX", NULL, name}, \
	{"PCM2.3 TX", NULL, name}, \
	{"PCM3.0 TX", NULL, name}, \
	{"PCM3.1 TX", NULL, name}, \
	{"PCM3.2 TX", NULL, name}, \
	{"PCM3.3 TX", NULL, name}, \
	{"PCM4.0 TX", NULL, name}, \
	{"PCM4.1 TX", NULL, name}, \
	{"PCM4.2 TX", NULL, name}, \
	{"PCM4.3 TX", NULL, name}, \
	{"PCM5.0 TX", NULL, name}, \
	{"PCM5.1 TX", NULL, name}, \
	{"PCM5.2 TX", NULL, name}, \
	{"PCM5.3 TX", NULL, name}


#define PCM_PORT_TO_OUTPUT_ROUTE(name) \
	{name, NULL, "PCM0.0 TX"}, \
	{name, NULL, "PCM0.1 TX"}, \
	{name, NULL, "PCM0.2 TX"}, \
	{name, NULL, "PCM0.3 TX"}, \
	{name, NULL, "PCM1.0 TX"}, \
	{name, NULL, "PCM1.1 TX"}, \
	{name, NULL, "PCM1.2 TX"}, \
	{name, NULL, "PCM1.3 TX"}, \
	{name, NULL, "PCM2.0 TX"}, \
	{name, NULL, "PCM2.1 TX"}, \
	{name, NULL, "PCM2.2 TX"}, \
	{name, NULL, "PCM2.3 TX"}, \
	{name, NULL, "PCM3.0 TX"}, \
	{name, NULL, "PCM3.1 TX"}, \
	{name, NULL, "PCM3.2 TX"}, \
	{name, NULL, "PCM3.3 TX"}, \
	{name, NULL, "PCM4.0 TX"}, \
	{name, NULL, "PCM4.1 TX"}, \
	{name, NULL, "PCM4.2 TX"}, \
	{name, NULL, "PCM4.3 TX"}, \
	{name, NULL, "PCM5.0 TX"}, \
	{name, NULL, "PCM5.1 TX"}, \
	{name, NULL, "PCM5.2 TX"}, \
	{name, NULL, "PCM5.3 TX"}

#define INPUT_TO_PCM_PORT_ROUTE(name) \
	{"PCM0.0 RX", NULL, name}, \
	{"PCM0.1 RX", NULL, name}, \
	{"PCM0.2 RX", NULL, name}, \
	{"PCM0.3 RX", NULL, name}, \
	{"PCM1.0 RX", NULL, name}, \
	{"PCM1.1 RX", NULL, name}, \
	{"PCM1.2 RX", NULL, name}, \
	{"PCM1.3 RX", NULL, name}, \
	{"PCM2.0 RX", NULL, name}, \
	{"PCM2.1 RX", NULL, name}, \
	{"PCM2.2 RX", NULL, name}, \
	{"PCM2.3 RX", NULL, name}, \
	{"PCM3.0 RX", NULL, name}, \
	{"PCM3.1 RX", NULL, name}, \
	{"PCM3.2 RX", NULL, name}, \
	{"PCM3.3 RX", NULL, name}, \
	{"PCM4.0 RX", NULL, name}, \
	{"PCM4.1 RX", NULL, name}, \
	{"PCM4.2 RX", NULL, name}, \
	{"PCM4.3 RX", NULL, name}, \
	{"PCM5.0 RX", NULL, name}, \
	{"PCM5.1 RX", NULL, name}, \
	{"PCM5.2 RX", NULL, name}, \
	{"PCM5.3 RX", NULL, name}

#define TX_PORT_MUX_TO_TX_PORT(name) \
	{"Tx0 Mux Port", name, "Tx0 PortMux En"}, \
	{"Tx1 Mux Port", name, "Tx1 PortMux En"}, \
	{"Tx2 Mux Port", name, "Tx2 PortMux En"}, \
	{"Tx3 Mux Port", name, "Tx3 PortMux En"}, \
	{"Tx4 Mux Port", name, "Tx4 PortMux En"}, \
	{"Tx5 Mux Port", name, "Tx5 PortMux En"}, \
	{"Tx6 Mux Port", name, "Tx6 PortMux En"}, \
	{"Tx7 Mux Port", name, "Tx7 PortMux En"}, \
	{"Tx8 Mux Port", name, "Tx8 PortMux En"}, \
	{"Tx9 Mux Port", name, "Tx9 PortMux En"}, \
	{"Tx10 Mux Port", name, "Tx10 PortMux En"}, \
	{"Tx11 Mux Port", name, "Tx11 PortMux En"}, \
	{"Tx12 Mux Port", name, "Tx12 PortMux En"}, \
	{"Tx13 Mux Port", name, "Tx13 PortMux En"}, \
	{"Tx14 Mux Port", name, "Tx14 PortMux En"}, \
	{"Tx15 Mux Port", name, "Tx15 PortMux En"}

static const struct snd_soc_dapm_route iaxxx_dapm_routes[] = {

	INPUT_TO_PCM_PORT_ROUTE("PCMINPUT0"),
	INPUT_TO_PCM_PORT_ROUTE("PCMINPUT1"),
	INPUT_TO_PCM_PORT_ROUTE("PCMINPUT2"),

	/* RX Port to RX Manager */
	PORT_TO_RX_MUX_ROUTE("Rx0 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx1 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx2 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx3 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx4 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx5 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx6 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx7 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx8 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx9 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx10 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx11 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx12 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx13 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx14 Mux Port"),
	PORT_TO_RX_MUX_ROUTE("Rx15 Mux Port"),

	RX_MUX_TO_PLUGIN_ROUTE("Rx0"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx1"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx2"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx3"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx4"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx5"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx6"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx7"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx8"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx9"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx10"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx11"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx12"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx13"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx14"),
	RX_MUX_TO_PLUGIN_ROUTE("Rx15"),

	PLUGIN_TO_TX_MUX_ROUTE("Plgin0"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin1"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin2"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin3"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin4"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin5"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin6"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin7"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin8"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin9"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin10"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin11"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin12"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin13"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin14"),
	PLUGIN_TO_TX_MUX_ROUTE("Plgin15"),

	RX_MUX_TO_TX_MUX_ROUTE("Tx0"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx1"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx2"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx3"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx4"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx5"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx6"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx7"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx8"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx9"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx10"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx11"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx12"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx13"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx14"),
	RX_MUX_TO_TX_MUX_ROUTE("Tx15"),

	TX_PORT_MUX_TO_TX_PORT("PCM0"),
	TX_PORT_MUX_TO_TX_PORT("PCM1"),
	TX_PORT_MUX_TO_TX_PORT("PCM2"),
	TX_PORT_MUX_TO_TX_PORT("PCM3"),
	TX_PORT_MUX_TO_TX_PORT("PCM4"),
	TX_PORT_MUX_TO_TX_PORT("PCM5"),

	/* RX Port to RX Manager */
	TX_MUX_TO_PORT_ROUTE("Tx0 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx1 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx2 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx3 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx4 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx5 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx6 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx7 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx8 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx9 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx10 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx11 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx12 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx13 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx14 Mux Port"),
	TX_MUX_TO_PORT_ROUTE("Tx15 Mux Port"),

	PCM_PORT_TO_OUTPUT_ROUTE("PCMOUTPUT1"),
	PCM_PORT_TO_OUTPUT_ROUTE("PCMOUTPUT2"),
};

static int iaxxx_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	int gain;
	u32 status = 0;
	int active = 0, ch_mask = 0xFFFF;
	u32 pcm_op_gn_mask = iaxxx->op_channels_active << 16;
	u32 pcm_op_gn_en_val = iaxxx->op_channels_active << 16;
	int ret;

	pr_debug("%s: mute: %d op_gn_en:%u\n", __func__, mute,
						pcm_op_gn_en_val);

	/* set gain to -60db when mute is called
	 * set gain to 0db when unmute is called
	 */
	if (mute)
		gain = -60; /*0xBC*/
	else
		gain = 0;

	/* Update TX CHANNEL GAIN EN HDR REG */
	snd_soc_update_bits(codec, IAXXX_CH_HDR_CH_GAIN_ADDR, pcm_op_gn_mask,
				pcm_op_gn_en_val);

	while (ch_mask & 1 << active) {
		if (iaxxx->op_channels_active & 1 << active) {
			/* Update the Gain ramp rate for TX Channel REG */
			snd_soc_update_bits(codec,
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(
				(active + TX_0)),
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_RAMP_MASK,
				STEP_INST);

			/* Update the gain based on mute unmute */
			snd_soc_update_bits(codec,
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_REG(
				(active + TX_0)),
				IAXXX_OUT_CH_GRP_CH_GAIN_CTRL_GAIN_TARGET_MASK,
				gain);
		}
		active++;
	}

	/* Update Block to set gain settings */
	ret = iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
			IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	return 0;
}

static int iaxxx_tdm3_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	u32 porte_clk_val = 0;
	u32 porte_fs_val = 0;
	u32 portd_clk_val = 0;
	u32 portd_fs_val = 0;
	u32 port_do_val = 0;
	u32 port_di_val = 0;
	u32 status = 0;
	int id = PCM_PORTD;

	pr_debug("%s\n", __func__);

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d\n", dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is Master , chip is slave */
		iaxxx->is_codec_master[id] = 0;
		porte_clk_val = IAXXX_IO_CTRL_CLK_SLAVE;
		porte_fs_val = IAXXX_IO_CTRL_FS_SLAVE;
		portd_clk_val = IAXXX_IO_CTRL_CLK_SLAVE;
		portd_fs_val = IAXXX_IO_CTRL_FS_SLAVE;

		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CPU is slave , chip is Master */
		iaxxx->is_codec_master[id] = 1;
		porte_clk_val = IAXXX_IO_CTRL_CLK_MASTER;
		porte_fs_val = IAXXX_IO_CTRL_FS_MASTER;
		portd_clk_val = IAXXX_IO_CTRL_CLK_SLAVE;
		portd_fs_val = IAXXX_IO_CTRL_FS_SLAVE;
		break;
	default:
		return -EINVAL;
	}

	port_di_val = IAXXX_IO_CTRL_DI;
	port_do_val = IAXXX_IO_CTRL_DO;
	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
		0x18, 0x18); /* enable PortD and E */

	iaxxx_send_update_block_request(iaxxx->dev_parent, &status,
						    IAXXX_BLOCK_0);

	/* BCLK IOCTRL Configuration */
	snd_soc_update_bits(codec, port_clk_addr[id],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK, porte_clk_val);
	snd_soc_update_bits(codec, port_clk_addr[id + 1],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK, portd_clk_val);

	/* FS IOCTRL Configuration */
	snd_soc_update_bits(codec, port_fs_addr[id],
		IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK, porte_fs_val);
	snd_soc_update_bits(codec, port_fs_addr[id + 1],
		IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK, portd_fs_val);

	/* Data IN IOCTRL Configuration */
	snd_soc_update_bits(codec, port_di_addr[id],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK, port_di_val);
	snd_soc_update_bits(codec, port_di_addr[id + 1],
		IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK, port_di_val);

	/* Data out IOCTRL Configuration */
	snd_soc_update_bits(codec, port_do_addr[id],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, port_do_val);
	snd_soc_update_bits(codec, port_do_addr[id + 1],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK, port_do_val);


	return 0;

}

static int iaxxx_tdm3_startup(struct snd_pcm_substream *substream,
						struct snd_soc_dai *dai)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	unsigned int fmt = SND_SOC_DAIFMT_CBS_CFS;
	int ret = 0;
	int id = PCM_PORTD;

	if (iaxxx->is_codec_master[id])
		fmt = SND_SOC_DAIFMT_CBM_CFM;

	ret = iaxxx_tdm3_set_fmt(dai, fmt);
	if (ret < 0) {
		pr_err("%s: set fmt codec dai failed for TDM , err:%d\n",
				__func__, ret);
		return ret;
	}

	ret = iaxxx_digital_mute(dai, false);
	if (ret < 0)
		pr_err("%s() tx tdm channel unmute fail %d\n", __func__, ret);

	return ret;
}

static int iaxxx_pcm_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	u32 reg_srdd_val = 0;
	u32 port_clk_val = 0;
	u32 port_di_val = 0;
	u32 port_do_val = 0;
	u32 port_fs_val = 0;
	u32 status = 0;
	int ret;
	int id =  dai->id;

	pr_debug("%s\n", __func__);

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d\n", dai->id);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is Master , chip is slave */
		iaxxx->is_codec_master[id] = 0;
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_SLAVE;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_SLAVE;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* CPU is slave , chip is Master */
		iaxxx->is_codec_master[id] = 1;
		port_clk_val = port_clk_val | IAXXX_IO_CTRL_CLK_MASTER;
		port_fs_val = port_fs_val | IAXXX_IO_CTRL_FS_MASTER;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		reg_srdd_val = 1;
		iaxxx->pcm_dai_fmt[id] = SND_SOC_DAIFMT_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		reg_srdd_val = 0;
		iaxxx->pcm_dai_fmt[id] = SND_SOC_DAIFMT_DSP_A;
		break;
	default:
		reg_srdd_val = 0;
		iaxxx->pcm_dai_fmt[id] = 0;
		dev_dbg(codec->dev, "default settings\n");
	}

	port_di_val = port_di_val | IAXXX_IO_CTRL_DI;
	port_do_val = port_do_val | IAXXX_IO_CTRL_DO;
	/* TODO need to move to pm ops functions in future */
	snd_soc_update_bits(codec, IAXXX_SRB_PCM_PORT_PWR_EN_ADDR,
			IAXXX_SRB_PCM_PORT_PWR_EN_MASK_VAL, (1 << id));

	ret = iaxxx_send_update_block_request(iaxxx->dev_parent,
					&status, IAXXX_BLOCK_0);
	if (ret) {
		pr_err("Update block fail %s()\n", __func__);
		return ret;
	}

	snd_soc_update_bits(codec, IAXXX_PCM_SRDD_ADDR(id),
			IAXXX_PCM0_SRDD_WMASK_VAL, reg_srdd_val);

	snd_soc_update_bits(codec, port_clk_addr[id],
		IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK |
		IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK, port_clk_val);

	snd_soc_update_bits(codec, port_fs_addr[id],
		 IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK, port_fs_val);

	snd_soc_update_bits(codec, port_di_addr[id],
		 IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK, port_di_val);

	snd_soc_update_bits(codec, port_do_addr[id],
		 IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK |
		 IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK, port_do_val);

	return 0;
}


static int iaxxx_set_i2s_cfg(struct snd_soc_dai *dai, u32 sampling_rate,
						bool is_pseudo, int id)
{
	struct snd_soc_codec *codec = dai->codec;

	if (is_pseudo) {
		pr_err("Pseudo mode not supported\n");
		return -EINVAL;
	}

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d\n", id);
		return -EINVAL;
	}

	iaxxx_set_i2s_controller(codec, sampling_rate, is_pseudo, id);
	return 0;
}

static int iaxxx_tdm3_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	u32 sampling_rate = 0;
	u32 ao_bclk_val = 0;
	u32 ao_fs_val = 0;
	u32 ao_do_val = IAXXX_AO_DO_ENABLE;
	u32 cnr0_pcm_val = 0;
	u32 cnr0_i2s_val = 0;
	u32 channel_portd_rx;
	u32 channel_portd_tx;
	u32 channel_porte_rx;
	u32 channel_porte_tx;
	u32 misc_ctrl_value;
	u32 word_len;
	u32 frame_length;
	int id = PCM_PORTD;

	pr_debug("%s\n", __func__);
	switch (params_width(params)) {
	/* word length = width-1 */
	case 16:
		word_len = 15;
		break;
	case 20:
		word_len = 19;
		break;
	case 24:
		word_len = 23;
		break;
	case 32:
		word_len = 31;
		break;
	default:
		pr_err("Unsupported word length\n");
		return -EINVAL;
	}

	switch (params_channels(params)) {
	case 12:
		channel_portd_rx = 0;
		channel_portd_tx = 0xFF; /* 8 channels */
		channel_porte_rx = 0;
		channel_porte_tx = 0xF00;  /* 4 channels */
		frame_length = 11; /*words per frame - 1 is 10-1 */
		break;
	case 10:
		channel_portd_rx = 0;
		channel_portd_tx = 0xFF; /* 8 channels */
		channel_porte_rx = 0;
		channel_porte_tx = 0x300;  /* 2 channels */
		frame_length = 9; /*words per frame - 1 is 10-1 */
		break;
	case 8:
		channel_portd_rx = 0;
		channel_portd_tx = 0xFF; /* 8 channels */
		channel_porte_rx = 0;
		channel_porte_tx = 0x00;  /* 2 channels */
		frame_length = 7; /*words per frame - 1 is 10-1 */
		break;
	case 2:
		channel_portd_rx = 0;
		channel_portd_tx = 0x03;
		channel_porte_rx = 0;
		channel_porte_tx = 0x00;
		frame_length = 1; /*words per frame - 1 is 10-1 */
		break;
	case 1:
		channel_portd_rx = 0;
		channel_portd_tx = 0x01;
		channel_porte_rx = 0;
		channel_porte_tx = 0x00;
		frame_length = 0; /*words per frame - 1 is 10-1 */
		break;
	default:
		pr_err("Unsupported channels :%d\n",
				params_channels(params));
		return -EINVAL;
	}


	snd_soc_update_bits(codec, IAXXX_PCM_SWLR_ADDR(id),
		IAXXX_PCM0_SWLR_WMASK_VAL, word_len);
	snd_soc_update_bits(codec, IAXXX_PCM_SWLR_ADDR(id + 1),
		IAXXX_PCM0_SWLR_WMASK_VAL, word_len);
	snd_soc_update_bits(codec, IAXXX_PCM_SFLR_ADDR(id),
		IAXXX_PCM3_SFLR_WMASK_VAL, frame_length);
	snd_soc_update_bits(codec, IAXXX_PCM_SFLR_ADDR(id + 1),
		IAXXX_PCM3_SFLR_WMASK_VAL, frame_length);

	snd_soc_update_bits(codec, IAXXX_PCM_SRSA_ADDR(id),
		IAXXX_PCM0_SRSA_WMASK_VAL, channel_portd_rx);
	snd_soc_update_bits(codec, IAXXX_PCM_STSA_ADDR(id),
		IAXXX_PCM0_STSA_WMASK_VAL, channel_portd_tx);
	snd_soc_update_bits(codec, IAXXX_PCM_SRSA_ADDR(id + 1),
		IAXXX_PCM0_SRSA_WMASK_VAL, channel_porte_rx);
	snd_soc_update_bits(codec, IAXXX_PCM_STSA_ADDR(id + 1),
		IAXXX_PCM0_STSA_WMASK_VAL, channel_porte_tx);


	misc_ctrl_value = IAXXX_PCM3_MC_FSE_MASK | IAXXX_PCM3_MC_RCP_MASK |
			IAXXX_PCM3_MC_END_MASK | IAXXX_PCM3_MC_TRI_MASK |
			IAXXX_PCM3_MC_LEFTJUST_MASK;

	snd_soc_update_bits(codec, IAXXX_PCM_MC_ADDR(id),
			IAXXX_PCM3_MC_MASK_VAL, misc_ctrl_value);
	snd_soc_update_bits(codec, IAXXX_PCM_MC_ADDR(id + 1),
			IAXXX_PCM3_MC_MASK_VAL, misc_ctrl_value);

	ao_do_val = IAXXX_AO_DO_ENABLE;
	if (iaxxx->is_codec_master[id]) {
		sampling_rate = params_rate(params);
		iaxxx_set_i2s_cfg(dai, sampling_rate, false, id + 1);
		ao_bclk_val = IAXXX_AO_BCLK_ENABLE;
		ao_fs_val = IAXXX_AO_FS_ENABLE;
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_HIGH;
	} else {
		ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
		ao_fs_val = IAXXX_AO_FS_DISABLE;
	}
	cnr0_pcm_val = IAXXX_CNR0_PCM_ENABLE;
	/* Port D configuration */
	/* Set Port  clk, FS, DO reg */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(id),
		IAXXX_AO_BCLK_DISABLE <<
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(id),
		IAXXX_AO_FS_DISABLE <<
		IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(id),
		ao_do_val << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(id));


	/* Port E configuration */
	/* Set Port  clk, FS, DO reg */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(id + 1),
		ao_bclk_val << IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(id + 1));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(id + 1),
		ao_fs_val << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(id + 1));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(id + 1),
		ao_do_val << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(id + 1));


	if (iaxxx->is_codec_master[id]) {
		/* CNR0_I2S_Enable  - Enable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK((id + 1)),
			cnr0_i2s_val << (id + 1));

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}

	/* Set cn0 pcm active reg PortD*/
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(id),
		cnr0_pcm_val << IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(id));
	/* Set cn0 pcm active reg portE*/
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(id + 1),
		cnr0_pcm_val << IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(id + 1));

	return 0;

}

static int iaxxx_pcm_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	u32 word_len = 0;
	int mode = 0;
	u32 channel_port_rx = 0;
	u32 channel_port_tx = 0;
	u32 sampling_rate = 0;
	u32 ao_bclk_val = 0;
	u32 ao_fs_val = 0;
	u32 cnr0_i2s_val = 0;
	u32 ch_val;
	u32 frame_length;
	int id = dai->id;

	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d\n", dai->id);
		return -EINVAL;
	}

	pr_debug("%s\n", __func__);
	switch (params_width(params)) {
	case 16:
		word_len = 15;
		break;
	case 20:
		word_len = 19;
		break;
	case 24:
		word_len = 23;
		break;
	case 32:
		word_len = 31;
		break;
	default:
		pr_debug("Unsupported word length\n");
		return -EINVAL;
	}

	switch (params_channels(params)) {
	case 8:
		channel_port_rx = 0xFF;
		channel_port_tx = 0xFF; /* 8 channels */
		frame_length = 7; /*words per frame - 1 */
		break;
	case 6:
		channel_port_rx = 0x3F;
		channel_port_tx = 0x3F;
		frame_length = 5; /*words per frame - 1 */
		break;
	case 4:
		channel_port_rx = 0x0F;
		channel_port_tx = 0x0F;
		frame_length = 3; /*words per frame - 1 */
		break;
	case 3:
		channel_port_rx = 0x07;
		channel_port_tx = 0x07;
		frame_length = 2; /*words per frame - 1 */
		break;
	case 2:
		channel_port_rx = 0x03;
		channel_port_tx = 0x03;
		frame_length = 1; /*words per frame - 1 */
		break;
	case 1:
		channel_port_rx = 0x01;
		channel_port_tx = 0x01;
		frame_length = 0; /*words per frame - 1 */
		break;
	default:
		pr_debug("Unsupported channels :%d\n",
				params_channels(params));
		return -EINVAL;
	}
	pr_debug("supported channels :%d rx: %d tx: %d\n",
			params_channels(params), channel_port_rx,
			channel_port_tx);

	snd_soc_update_bits(codec, IAXXX_PCM_SWLR_ADDR(id),
		IAXXX_PCM0_SWLR_WMASK_VAL, word_len);

	snd_soc_update_bits(codec, IAXXX_PCM_SRSA_ADDR(id),
		IAXXX_PCM0_SRSA_WMASK_VAL, channel_port_rx);

	/* Allow extra cycles for HW to set the value
	 * by doing a read of the same register.
	 */
	ch_val = snd_soc_read(codec, IAXXX_PCM_SRSA_ADDR(id));

	snd_soc_update_bits(codec, IAXXX_PCM_STSA_ADDR(id),
		IAXXX_PCM0_STSA_WMASK_VAL, channel_port_tx);

	/* Allow extra cycles for HW to set the value
	 * by doing a read of the same register.
	 */
	ch_val = snd_soc_read(codec, IAXXX_PCM_STSA_ADDR(id));

	snd_soc_update_bits(codec, IAXXX_PCM_SFLR_ADDR(id),
		IAXXX_PCM0_SFLR_WMASK_VAL, frame_length);

	if (iaxxx->pcm_dai_fmt[id] == SND_SOC_DAIFMT_I2S) {
		mode = IAXXX_PCM_CTRL_DEFAULT_I2SFMT;
	} else if (iaxxx->pcm_dai_fmt[id] == SND_SOC_DAIFMT_DSP_A) {
		mode = IAXXX_PCM_CTRL_DEFAULT_DSPFMT;
	} else if (iaxxx->pcm_port_fmt[id]) {
		mode = IAXXX_PCM_CTRL_DEFAULT_TDMFMT;
	} else {
		pr_err("%s(): DAI FMT configuration is missing\n", __func__);
		return -EINVAL;
	}

	snd_soc_update_bits(codec, IAXXX_PCM_MC_ADDR(id),
			IAXXX_PCM0_MC_WMASK_VAL, mode);

	if (iaxxx->is_codec_master[id]) {
		sampling_rate = params_rate(params);
		iaxxx_set_i2s_cfg(dai, sampling_rate, false, id);
		ao_bclk_val = IAXXX_AO_BCLK_ENABLE;
		ao_fs_val = IAXXX_AO_FS_ENABLE;
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_HIGH;
	} else {
		ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
		ao_fs_val = IAXXX_AO_FS_DISABLE;
	}

	/* Set Port  clk, FS, DO reg */
	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(id),
		ao_bclk_val << IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(id),
		ao_fs_val << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(id),
		IAXXX_AO_DO_ENABLE <<
			IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(id));
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(id),
		IAXXX_CNR0_PCM_ENABLE <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(id));

	if (iaxxx->is_codec_master[id]) {
		/* CNR0_I2S_Enable  - Enable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(id),
			cnr0_i2s_val << id);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		iaxxx->is_stream_in_use[id][0] = true;
	else
		iaxxx->is_stream_in_use[id][1] = true;

	return 0;
}

static int iaxxx_pcm_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	unsigned int dai_fmt = 0;
	unsigned int mstr_fmt = iaxxx->is_codec_master[dai->id];
	int ret = 0;
	int id = dai->id;

	pr_debug("%s\n", __func__);
	if (dai->id >= IAXXX_NUM_CODEC_DAIS) {
		pr_err("Unsupported dai id:%d\n", dai->id);
		return -EINVAL;
	}

	mstr_fmt = mstr_fmt ? SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS;
	if (iaxxx->pcm_dai_fmt[id] == SND_SOC_DAIFMT_I2S) {
		dai_fmt = SND_SOC_DAIFMT_I2S | mstr_fmt;
	} else if (iaxxx->pcm_dai_fmt[id] == SND_SOC_DAIFMT_DSP_A) {
		dai_fmt = SND_SOC_DAIFMT_DSP_A | mstr_fmt;
	} else if (iaxxx->pcm_port_fmt[id]) {
		dai_fmt = mstr_fmt;
	} else {
		pr_err("%s(): DAI FMT configuration is missing\n", __func__);
		return -EINVAL;
	}

	ret = iaxxx_pcm_set_fmt(dai, dai_fmt);
	if (ret < 0) {
		pr_err("%s() PCM dai set fmt fail %d\n", __func__, ret);
		return ret;
	}

	ret = iaxxx_digital_mute(dai, false);
	if (ret < 0)
		pr_err("%s() tx channel unmute fail %d\n", __func__, ret);

	return ret;
}

static void iaxxx_port_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int ret = 0;

	ret = iaxxx_digital_mute(dai, true);
	if (ret < 0)
		pr_err("%s() tx channel mute fail %d\n", __func__, ret);
}

static int iaxxx_tdm3_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	u32 ao_bclk_val = IAXXX_AO_BCLK_DISABLE;
	u32 ao_fs_val = IAXXX_AO_FS_DISABLE;
	u32 ao_do_val = IAXXX_AO_DO_DISABLE;
	u32 cnr0_pcm_val = IAXXX_CNR0_PCM_DISABLE;
	int id  = PCM_PORTD;

	pr_debug("%s\n", __func__);

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(id),
		ao_bclk_val <<
			IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(id),
		ao_fs_val <<
			IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(id),
		ao_do_val << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(id + 1),
		ao_bclk_val <<
			IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(id + 1));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(id + 1),
		ao_fs_val <<
			IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(id + 1));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(id + 1),
		ao_do_val << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(id + 1));

	if (iaxxx->is_codec_master[id]) {
		/* CNR0_I2S_Enable  - Disable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(id + 1),
			IAXXX_CNR0_I2S_ENABLE_LOW << (id + 1));

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(id),
		cnr0_pcm_val <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(id));
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(id + 1),
		cnr0_pcm_val <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(id + 1));

	return 0;
}

static int iaxxx_pcm_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(dai->codec);
	u32 cnr0_i2s_val = 0;
	int id = dai->id;

	pr_debug("%s\n", __func__);

	if (iaxxx->is_codec_master[id])
		cnr0_i2s_val = IAXXX_CNR0_I2S_ENABLE_LOW;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		iaxxx->is_stream_in_use[id][0] = false;
	else
		iaxxx->is_stream_in_use[id][1] = false;

	if (iaxxx->is_stream_in_use[id][0] || iaxxx->is_stream_in_use[id][1]) {
		pr_debug("%s(): one of the stream is still active:%d\n",
				__func__, substream->stream);
		return 0;
	}

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_CLK_OE_MASK(id),
		IAXXX_AO_BCLK_DISABLE <<
			IAXXX_AO_CLK_CFG_PORT_CLK_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_FS_OE_MASK(id),
		IAXXX_AO_FS_DISABLE << IAXXX_AO_CLK_CFG_PORT_FS_OE_POS(id));

	snd_soc_update_bits(codec, IAXXX_AO_CLK_CFG_ADDR,
		IAXXX_AO_CLK_CFG_PORT_DO_OE_MASK(id),
		IAXXX_AO_DO_DISABLE << IAXXX_AO_CLK_CFG_PORT_DO_OE_POS(id));

	if (iaxxx->is_codec_master[id]) {
		/* CNR0_I2S_Enable  - Disable I2S  */
		snd_soc_update_bits(codec, IAXXX_CNR0_I2S_ENABLE_ADDR,
			IAXXX_CNR0_I2S_ENABLE_MASK(id),
			cnr0_i2s_val << id);

		/* I2S Trigger - Enable */
		snd_soc_update_bits(codec,
			IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
			IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
			IAXXX_I2S_TRIGGER_HIGH);
	}
	/* Set cn0 pcm active reg */
	snd_soc_update_bits(codec, IAXXX_CNR0_PCM_ACTIVE_ADDR,
		IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_MASK(id),
		IAXXX_CNR0_PCM_DISABLE <<
			IAXXX_CNR0_PCM_ACTIVE_PCM_ACT_POS(id));

	return 0;
}

#define IAXXX_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S24_3LE|\
			SNDRV_PCM_FMTBIT_S32_LE)

#define IAXXX_PCM_RATES SNDRV_PCM_RATE_8000_48000

static const struct snd_soc_dai_ops iaxxx_pcm_ops = {
	.set_fmt = iaxxx_pcm_set_fmt,
	.hw_params = iaxxx_pcm_hw_params,
	.startup = iaxxx_pcm_startup,
	.shutdown = iaxxx_port_shutdown,
	.hw_free = iaxxx_pcm_hw_free,
};

static const struct snd_soc_dai_ops iaxxx_tdm3_ops = {
	.set_fmt = iaxxx_tdm3_set_fmt,
	.hw_params = iaxxx_tdm3_hw_params,
	.startup = iaxxx_tdm3_startup,
	.shutdown = iaxxx_port_shutdown,
	.hw_free = iaxxx_tdm3_hw_free,
};

static struct snd_soc_dai_driver iaxxx_dai[] = {
	{
		.name = "iaxxx-pcm0",
		.id = IAXXX_AIF0,
		.playback = {
			.stream_name = "I2S PCM0 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM0 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm1",
		.id = IAXXX_AIF1,
		.playback = {
			.stream_name = "I2S PCM1 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM1 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm2",
		.id = IAXXX_AIF2,
		.playback = {
			.stream_name = "I2S PCM2 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM2 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm3",
		.id = IAXXX_AIF3,
		.playback = {
			.stream_name = "I2S PCM3 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM3 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm4",
		.id = IAXXX_AIF4,
		.playback = {
			.stream_name = "I2S PCM4 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM4 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-pcm5",
		.id = IAXXX_AIF5,
		.playback = {
			.stream_name = "I2S PCM5 Rx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S PCM5 Tx",
			.channels_min = 1,
			.channels_max = 8,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_pcm_ops,
	},
	{
		.name = "iaxxx-tdm3",
		.id = IAXXX_AIF3,
		.playback = {
			.stream_name = "I2S TDM3 Rx",
			.channels_min = 1,
			.channels_max = 16,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "I2S TDM3 Tx",
			.channels_min = 1,
			.channels_max = 16,
			.rates = IAXXX_PCM_RATES,
			.formats = IAXXX_PCM_FORMATS,
		},
		.ops = &iaxxx_tdm3_ops,
	},
};

void iaxxx_reset_codec_params(struct iaxxx_priv *priv)
{
	struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(priv->codec_dev);
	int i = 0;

	for (i = 0; i < IAXXX_MAX_PDM_PORTS; i++) {
		iaxxx->port_start_en[i] = 0;
		iaxxx->port_filter[i] = 0;
		if (i < IAXXX_MAX_PORTS) {
			iaxxx->port_pcm_start[i] = 0;
			iaxxx->port_pcm_setup[i] = 0;
		}
	}
	for (i = 0; i < 32; i++) {
		iaxxx->plugin_blk_en[i] = 0;
		iaxxx->stream_en[i] = 0;
	}
	iaxxx->portb_mic0_en = 0;
	iaxxx->portb_mic1_en = 0;
	iaxxx->portb_mic2_en = 0;
	iaxxx->portb_mic3_en = 0;
	iaxxx->portc_mic0_en = 0;
	iaxxx->portc_mic1_en = 0;
	iaxxx->portc_mic2_en = 0;
	iaxxx->portc_mic3_en = 0;
	iaxxx->cdc0_mic0_en = 0;
	iaxxx->cdc0_mic1_en = 0;
	iaxxx->cdc0_mic2_en = 0;
	iaxxx->cdc1_mic0_en = 0;
	iaxxx->portb_micbias_en = 0;
	iaxxx->portc_micbias_en = 0;
	iaxxx->cdc0_micbias_en = 0;
	iaxxx->cdc1_micbias_en = 0;
	iaxxx->pdm_bclk = 0;
	iaxxx->pdm_aclk = 0;
	iaxxx->head_of_strm_all = 0;
	iaxxx->cdc_dmic_enable = 0;

	return;

}

static int iaxxx_add_widgets(struct snd_soc_codec *codec)
{

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0))
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
#else
	struct snd_soc_dapm_context *dapm = &codec->dapm;
#endif
	snd_soc_add_codec_controls(codec, iaxxx_snd_controls,
		ARRAY_SIZE(iaxxx_snd_controls));

	snd_soc_dapm_new_controls(dapm, iaxxx_dapm_widgets,
		ARRAY_SIZE(iaxxx_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, iaxxx_dapm_routes,
		ARRAY_SIZE(iaxxx_dapm_routes));

	return 0;
}

static int iaxxx_codec_probe(struct snd_soc_codec *codec)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 15, 0))
	int ret;
	struct iaxxx_codec_priv *iaxxx = snd_soc_codec_get_drvdata(codec);

	codec->control_data = iaxxx->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 32, 32, SND_SOC_REGMAP);
	if (ret) {
		dev_err(codec->dev, "unable to set the cache io");
		return ret;
	}
#endif

	dev_info(codec->dev, "%s\n", __func__);

	pm_runtime_get_sync(codec->dev);

	iaxxx_add_widgets(codec);

	pm_runtime_put(codec->dev);

	return 0;
}

static int iaxxx_codec_remove(struct snd_soc_codec *codec)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static struct regmap *iaxxx_get_regmap(struct device *dev)
{
	return dev_get_regmap(dev->parent, NULL);
}

static struct snd_soc_codec_driver soc_codec_iaxxx = {
	.probe = iaxxx_codec_probe,
	.remove = iaxxx_codec_remove,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	.get_regmap = iaxxx_get_regmap,
#endif
/*	.suspend = iaxxx_codec_suspend,	*/
/*	.resume = iaxxx_codec_resume,	*/
/*	.set_bias_level = iaxxx_codec_set_bias_level,	*/
};

static const struct of_device_id iaxxx_platform_dt_match[] = {
	{.compatible = "adnc,iaxxx-codec"},
	{}
};

static int iaxxx_codec_driver_probe(struct platform_device *pdev)
{
	struct iaxxx_codec_priv *iaxxx;
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv;
	int count = 0, ret = 0;
	u32 op_ch_active = 0;
	u32 pcm_port_fmt[IAXXX_MAX_PORTS];
	u32 ip_port_master_slave[IAXXX_MAX_PDM_PORTS];
	u32 ip_pdm_clk_src[IAXXX_MAX_PDM_PORTS];
	struct device_node *np = NULL;

	dev_dbg(dev, "%s() Enter..\n", __func__);

	if (pdev->dev.of_node) {
		dev_err(dev, "%s() DT Node exists for iaxxx-codec %s\n",
				__func__, pdev->dev.of_node->name);
	} else {
		/* In MFD probe, the devices do not have DT node, so scan the
		 * DT to see if our device node is present and if so use it.
		 */
		for_each_child_of_node(dev->parent->of_node, np) {
			dev_dbg(dev, "Found child-node %s\n", np->name);
			if (of_device_is_compatible(np,
				iaxxx_platform_dt_match[0].compatible)) {
				dev_dbg(dev, "Child-node is compatible %s\n",
					np->name);
				pdev->dev.of_node = np;
			}
		}
	}

	/* MFD core will provide the regmap instance */
	priv = to_iaxxx_priv(dev->parent);
	if (priv == NULL) {
		pr_err("MFD parent device data not found yet. Deferred.\n");
		return -EPROBE_DEFER;
	}

	iaxxx = devm_kzalloc(&pdev->dev, sizeof(*iaxxx), GFP_KERNEL);
	if (iaxxx == NULL)
		return -ENOMEM;

	iaxxx->regmap = priv->regmap;
	iaxxx->dev_parent = dev->parent;
	iaxxx->plugin_param[0].param_id = IAXXX_MAX_VAL;
	iaxxx->plugin_param[1].param_id = IAXXX_MAX_VAL;
	iaxxx->plugin_param[2].param_id = IAXXX_MAX_VAL;
	iaxxx->cdc_dmic_enable = 0;
	platform_set_drvdata(pdev, iaxxx);
	priv->codec_dev = dev;
	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", "iaxxx-codec");

	if (priv->dev->of_node) {
		ret = of_property_read_u32_array(priv->dev->of_node,
				"adnc,ip-port-master",
				ip_port_master_slave, IAXXX_MAX_PDM_PORTS);
		if (ret) {
			dev_err(dev,
				"%s(): no adnc,ip-port-master in DT node: %d\n",
					__func__, ret);
		} else {
			for (count = 0; count < IAXXX_MAX_PDM_PORTS; count++) {
				iaxxx->is_ip_port_master[count] =
						ip_port_master_slave[count];
			}
		}

		ret = of_property_read_u32_array(priv->dev->of_node,
				"adnc,pcm-port-fmt",
				pcm_port_fmt, IAXXX_MAX_PORTS);
		if (ret) {
			dev_err(dev,
				"%s(): no adnc,pcm-port-fmt in DT node: %d\n",
					__func__, ret);
		} else {
			for (count = 0; count < IAXXX_MAX_PORTS; count++) {
				iaxxx->pcm_port_fmt[count] =
						pcm_port_fmt[count];
			}
		}

		ret = of_property_read_u32(priv->dev->of_node,
				"adnc,op-ch-active", &op_ch_active);
		if (ret < 0)
			dev_err(dev,
				"%s():Failed to read op-ch-active, ret = %d\n",
				__func__, ret);
		else
			iaxxx->op_channels_active = op_ch_active;

		ret = of_property_read_u32_array(priv->dev->of_node,
				"adnc,ip-pdm-clk-src",
				ip_pdm_clk_src, IAXXX_MAX_PDM_PORTS);
		if (ret < 0)
			dev_err(dev,
				"%s(): Failed to read pdm clk src, ret = %d\n",
				__func__, ret);
		else {
			for (count = 0; count < IAXXX_MAX_PDM_PORTS; count++)
				iaxxx->ip_pdm_clk_src[count] =
						ip_pdm_clk_src[count];
		}

	}

	pm_runtime_enable(&pdev->dev);
	pm_runtime_idle(&pdev->dev);

	/* Check FW download done */
	if (priv->iaxxx_state->fw_state != FW_APP_MODE) {
		dev_err(dev, "FW not downloaded\n");
		return -EIO;
	}
	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_iaxxx,
					iaxxx_dai, ARRAY_SIZE(iaxxx_dai));
	if (ret)
		dev_err(dev, "codec registration failed\n");

	return ret;
}

static int iaxxx_codec_driver_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int iaxxx_suspend(struct device *dev)
{
	/* struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(dev); */

	return 0;
}

static int iaxxx_resume(struct device *dev)
{
	/* struct iaxxx_codec_priv *iaxxx = dev_get_drvdata(dev); */

	return 0;
}
#endif

static const struct dev_pm_ops iaxxx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(iaxxx_suspend, iaxxx_resume)
};


static struct platform_driver iaxxx_codec_driver = {
	.probe  = iaxxx_codec_driver_probe,
	.remove = iaxxx_codec_driver_remove,
	.driver = {
		.name = "iaxxx-codec",
		.of_match_table = iaxxx_platform_dt_match,
	},
};

module_platform_driver(iaxxx_codec_driver);

/* Module information */
MODULE_DESCRIPTION("Knowles IAXXX ALSA SoC CODEC Driver");
MODULE_LICENSE("GPL v2");

