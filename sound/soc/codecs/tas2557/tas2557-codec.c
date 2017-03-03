/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** File:
**     tas2557-codec.c
**
** Description:
**     ALSA SoC driver for Texas Instruments TAS2557 High Performance 4W Smart Amplifier
**
** =============================================================================
*/

#ifdef CONFIG_TAS2557_CODEC_STEREO

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas2557-core.h"
#include "tas2557-codec.h"

#define KCONTROL_CODEC

static unsigned int tas2557_codec_read(struct snd_soc_codec *pCodec,
	unsigned int nRegister)
{
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(pCodec);
	int ret = 0;
	unsigned int Value = 0;

	ret = pTAS2557->read(pTAS2557,
		pTAS2557->mnCurrentChannel, nRegister, &Value);
	if (ret < 0) {
		dev_err(pTAS2557->dev, "%s, %d, ERROR happen=%d\n", __func__,
			__LINE__, ret);
		return 0;
	} else
		return Value;
}

static int tas2557_codec_write(struct snd_soc_codec *pCodec, unsigned int nRegister,
	unsigned int nValue)
{
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(pCodec);
	int ret = 0;

	ret = pTAS2557->write(pTAS2557,
		pTAS2557->mnCurrentChannel, nRegister, nValue);
	return ret;
}

static const struct snd_soc_dapm_widget tas2557_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("Stereo ASI1", "Stereo ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("Stereo ASI2", "Stereo ASI2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("Stereo ASIM", "Stereo ASIM Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("Stereo DAC", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUT_DRV("Stereo ClassD", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("Stereo PLL", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Stereo NDivider", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("Stereo OUT")
};

static const struct snd_soc_dapm_route tas2557_audio_map[] = {
	{"Stereo DAC", NULL, "Stereo ASI1"},
	{"Stereo DAC", NULL, "Stereo ASI2"},
	{"Stereo DAC", NULL, "Stereo ASIM"},
	{"Stereo ClassD", NULL, "Stereo DAC"},
	{"Stereo OUT", NULL, "Stereo ClassD"},
	{"Stereo DAC", NULL, "Stereo PLL"},
	{"Stereo DAC", NULL, "Stereo NDivider"},
};

static int tas2557_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	return 0;
}

static void tas2557_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	tas2557_enable(pTAS2557, 0);
}

static int tas2557_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	return 0;
}

static int tas2557_set_dai_sysclk(struct snd_soc_dai *pDAI,
	int nClkID, unsigned int nFreqency, int nDir)
{
	struct snd_soc_codec *pCodec = pDAI->codec;
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(pCodec);

	dev_dbg(pTAS2557->dev, "tas2557_set_dai_sysclk: freq = %u\n", nFreqency);

	return 0;
}

static int tas2557_hw_params(struct snd_pcm_substream *pSubstream,
	struct snd_pcm_hw_params *pParams, struct snd_soc_dai *pDAI)
{
	struct snd_soc_codec *pCodec = pDAI->codec;
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(pCodec);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
/* do bit rate setting during platform data */
/* tas2557_set_bit_rate(pTAS2557, channel_both, snd_pcm_format_width(params_format(pParams))); */
	tas2557_set_sampling_rate(pTAS2557, params_rate(pParams));
	return 0;
}

static int tas2557_set_dai_fmt(struct snd_soc_dai *pDAI, unsigned int nFormat)
{
	struct snd_soc_codec *codec = pDAI->codec;
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	return 0;
}

static int tas2557_prepare(struct snd_pcm_substream *pSubstream,
	struct snd_soc_dai *pDAI)
{
	struct snd_soc_codec *codec = pDAI->codec;
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	tas2557_enable(pTAS2557, 1);

	return 0;
}

static int tas2557_set_bias_level(struct snd_soc_codec *pCodec,
	enum snd_soc_bias_level eLevel)
{
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(pCodec);

	dev_dbg(pTAS2557->dev, "%s: %d\n", __func__, eLevel);
	return 0;
}

static int tas2557_codec_probe(struct snd_soc_codec *pCodec)
{
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(pCodec);

	dev_dbg(pTAS2557->dev, "%s\n", __func__);
	return 0;
}

static int tas2557_codec_remove(struct snd_soc_codec *pCodec)
{
	return 0;
}

static int tas2557_power_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2557->mbPowerUp;
	dev_dbg(pTAS2557->dev, "tas2557_power_ctrl_get = %d\n",
		pTAS2557->mbPowerUp);
	return 0;
}

static int tas2557_power_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	int nPowerOn = pValue->value.integer.value[0];

	dev_dbg(pTAS2557->dev, "tas2557_power_ctrl_put = %d\n", nPowerOn);

	tas2557_enable(pTAS2557, (nPowerOn != 0));
	return 0;
}

static int tas2557_fs_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	int nFS = 48000;

	if (pTAS2557->mpFirmware->mnConfigurations)
		nFS = pTAS2557->mpFirmware->mpConfigurations[pTAS2557->mnCurrentConfiguration].mnSamplingRate;
	pValue->value.integer.value[0] = nFS;
	dev_dbg(pTAS2557->dev, "tas2557_fs_get = %d\n", nFS);
	return 0;
}

static int tas2557_fs_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int nFS = pValue->value.integer.value[0];

	dev_info(pTAS2557->dev, "tas2557_fs_put = %d\n", nFS);
	ret = tas2557_set_sampling_rate(pTAS2557, nFS);
	return ret;
}

static int tas2557_program_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2557->mnCurrentProgram;
	dev_dbg(pTAS2557->dev, "tas2557_program_get = %d\n",
		pTAS2557->mnCurrentProgram);
	return 0;
}

static int tas2557_program_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	unsigned int nProgram = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2557_set_program(pTAS2557, nProgram, -1);
	return ret;
}

static int tas2557_configuration_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2557->mnCurrentConfiguration;
	dev_dbg(pTAS2557->dev, "tas2557_configuration_get = %d\n",
		pTAS2557->mnCurrentConfiguration);
	return 0;
}

static int tas2557_configuration_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	unsigned int nConfiguration = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2557_set_config(pTAS2557, nConfiguration);
	return ret;
}

static int tas2557_calibration_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2557->mnCurrentCalibration;
	dev_info(pTAS2557->dev,
		"tas2557_calibration_get = %d\n",
		pTAS2557->mnCurrentCalibration);
	return 0;
}

static int tas2557_calibration_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	unsigned int nCalibration = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2557_set_calibration(pTAS2557, nCalibration);
	return ret;
}

static int tas2557_ldac_gain_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	unsigned char nGain = 0;
	int ret = -1;

	ret = tas2557_get_DAC_gain(pTAS2557, channel_left, &nGain);
	if (ret >= 0)
		pValue->value.integer.value[0] = nGain;

	dev_dbg(pTAS2557->dev, "%s, ret = %d, %d\n", __func__, ret, nGain);
	return ret;
}

static int tas2557_ldac_gain_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	unsigned int nGain = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2557_set_DAC_gain(pTAS2557, channel_left, nGain);
	return ret;
}

static int tas2557_rdac_gain_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	unsigned char nGain = 0;
	int ret = -1;

	ret = tas2557_get_DAC_gain(pTAS2557, channel_right, &nGain);
	if (ret >= 0)
		pValue->value.integer.value[0] = nGain;

	dev_dbg(pTAS2557->dev, "%s, ret = %d, %d\n", __func__, ret, nGain);
	return ret;
}

static int tas2557_rdac_gain_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	unsigned int nGain = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2557_set_DAC_gain(pTAS2557, channel_right, nGain);
	return ret;
}

static const char * const chl_swap_text[] = {"default", "swap"};
static const struct soc_enum chl_swap_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(chl_swap_text), chl_swap_text),
};

static int tas2557_dsp_chl_swap_get(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2557->mnChannelSwap;

	return 0;
}

static int tas2557_dsp_chl_swap_put(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	int swap = pValue->value.integer.value[0];

	tas2557_SA_SwapChannel(pTAS2557, (swap != 0));
	return 0;
}

static const char * const echoref_ctl_text[] = {"left channel", "right channel", "both channel"};
static const struct soc_enum echoref_ctl_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(echoref_ctl_text), echoref_ctl_text),
};

static int tas2557_echoref_ctl_get(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2557->mnEchoRef;

	return 0;
}

static int tas2557_echoref_ctl_put(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	int echoref = pValue->value.integer.value[0]&0x01;	/* only take care of left/right channel switch */

	if (echoref != pTAS2557->mnEchoRef) {
		pTAS2557->mnEchoRef = echoref;
		tas2557_SA_ctl_echoRef(pTAS2557);
	}
	return 0;
}

static const char * const rom_chl_dev_text[] = {"left channel", "right channel", "both channel"};
static const struct soc_enum rom_chl_dev_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rom_chl_dev_text), rom_chl_dev_text),
};

static int tas2557_rom_chl_dev_get(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	int nChlDev = 0;

	switch (pTAS2557->mnROMChlDev) {
	case channel_left:
		nChlDev = 0;
	break;

	case channel_right:
		nChlDev = 1;
	break;

	case channel_both:
		nChlDev = 2;
	break;

	default:
	break;
	}

	pValue->value.integer.value[0] = nChlDev;
	return 0;
}

static int tas2557_rom_chl_dev_put(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);
	int nChlDev = pValue->value.integer.value[0];
	enum channel chl = channel_left;

	switch (nChlDev) {
	case 0:
		chl = channel_left;
	break;

	case 1:
		chl = channel_right;
	break;

	case 2:
		chl = channel_both;
	break;
	}

	pTAS2557->mnROMChlDev = chl;
	return 0;
}

static const char * const rom_chl_ctl_text[] = {"left channel", "right channel", "mono mix", "stereo"};
static const struct soc_enum rom_chl_ctl_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rom_chl_ctl_text), rom_chl_ctl_text),
};

static int tas2557_rom_chl_ctl_get(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2557->mnROMChlCtrl;
	return 0;
}

static int tas2557_rom_chl_ctl_put(struct snd_kcontrol *pKcontrol,
			struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2557_priv *pTAS2557 = snd_soc_codec_get_drvdata(codec);

	pTAS2557->mnROMChlCtrl = pValue->value.integer.value[0];
	tas2557_ROMMode_Chl_Ctrl(pTAS2557, pTAS2557->mnROMChlDev, pTAS2557->mnROMChlCtrl);
	return 0;
}

static const struct snd_kcontrol_new tas2557_snd_controls[] = {
	SOC_SINGLE_EXT("Stereo LDAC Playback Volume", SND_SOC_NOPM, 0, 0x0f, 0,
		tas2557_ldac_gain_get, tas2557_ldac_gain_put),
	SOC_SINGLE_EXT("Stereo RDAC Playback Volume", SND_SOC_NOPM, 0, 0x0f, 0,
		tas2557_rdac_gain_get, tas2557_rdac_gain_put),
	SOC_SINGLE_EXT("Stereo PowerCtrl", SND_SOC_NOPM, 0, 0x0001, 0,
		tas2557_power_ctrl_get, tas2557_power_ctrl_put),
	SOC_SINGLE_EXT("Stereo Program", SND_SOC_NOPM, 0, 0x00FF, 0,
		tas2557_program_get, tas2557_program_put),
	SOC_SINGLE_EXT("Stereo Configuration", SND_SOC_NOPM, 0, 0x00FF, 0,
		tas2557_configuration_get, tas2557_configuration_put),
	SOC_SINGLE_EXT("Stereo FS", SND_SOC_NOPM, 8000, 48000, 0,
		tas2557_fs_get, tas2557_fs_put),
	SOC_SINGLE_EXT("Stereo Calibration", SND_SOC_NOPM, 0, 0x00FF, 0,
		tas2557_calibration_get, tas2557_calibration_put),
	SOC_ENUM_EXT("Stereo DSPChl Swap", chl_swap_enum[0],
		tas2557_dsp_chl_swap_get, tas2557_dsp_chl_swap_put),
	SOC_ENUM_EXT("Stereo ROMChl Device", rom_chl_dev_enum[0],
		tas2557_rom_chl_dev_get, tas2557_rom_chl_dev_put),
	SOC_ENUM_EXT("Stereo ROMChl Ctrl", rom_chl_ctl_enum[0],
		tas2557_rom_chl_ctl_get, tas2557_rom_chl_ctl_put),
	SOC_ENUM_EXT("Stereo EchoRef Ctrl", echoref_ctl_enum[0],
		tas2557_echoref_ctl_get, tas2557_echoref_ctl_put),
};

static struct snd_soc_codec_driver soc_codec_driver_tas2557 = {
	.probe = tas2557_codec_probe,
	.remove = tas2557_codec_remove,
	.read = tas2557_codec_read,
	.write = tas2557_codec_write,
	.set_bias_level = tas2557_set_bias_level,
	.idle_bias_off = true,
	.controls = tas2557_snd_controls,
	.num_controls = ARRAY_SIZE(tas2557_snd_controls),
	.dapm_widgets = tas2557_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2557_dapm_widgets),
	.dapm_routes = tas2557_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tas2557_audio_map),
};

static struct snd_soc_dai_ops tas2557_dai_ops = {
	.startup = tas2557_startup,
	.shutdown = tas2557_shutdown,
	.digital_mute = tas2557_mute,
	.hw_params = tas2557_hw_params,
	.prepare = tas2557_prepare,
	.set_sysclk = tas2557_set_dai_sysclk,
	.set_fmt = tas2557_set_dai_fmt,
};

#define TAS2557_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)
static struct snd_soc_dai_driver tas2557_dai_driver[] = {
	{
		.name = "tas2557 Stereo ASI1",
		.id = 0,
		.playback = {
				.stream_name = "Stereo ASI1 Playback",
				.channels_min = 2,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_8000_192000,
				.formats = TAS2557_FORMATS,
			},
		.ops = &tas2557_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tas2557 Stereo ASI2",
		.id = 1,
		.playback = {
				.stream_name = "Stereo ASI2 Playback",
				.channels_min = 2,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_8000_192000,
				.formats = TAS2557_FORMATS,
			},
		.ops = &tas2557_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tas2557 Stereo ASIM",
		.id = 2,
		.playback = {
				.stream_name = "Stereo ASIM Playback",
				.channels_min = 2,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_8000_192000,
				.formats = TAS2557_FORMATS,
			},
		.ops = &tas2557_dai_ops,
		.symmetric_rates = 1,
	},
};

int tas2557_register_codec(struct tas2557_priv *pTAS2557)
{
	int nResult = 0;

	dev_info(pTAS2557->dev, "%s, enter\n", __func__);
	nResult = snd_soc_register_codec(pTAS2557->dev,
		&soc_codec_driver_tas2557,
		tas2557_dai_driver, ARRAY_SIZE(tas2557_dai_driver));
	return nResult;
}

int tas2557_deregister_codec(struct tas2557_priv *pTAS2557)
{
	snd_soc_unregister_codec(pTAS2557->dev);
	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2557 ALSA SOC Smart Amplifier Stereo driver");
MODULE_LICENSE("GPL v2");
#endif
