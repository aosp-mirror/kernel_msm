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
**     tas2557-core.c
**
** Description:
**     TAS2557 common functions for Android Linux
**
** =============================================================================
*/

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

#include "tas2557.h"
#include "tas2557-core.h"

#define TAS2557_CAL_NAME    "/data/tas2557_cal.bin"
#define CALIBRATION_DATA_PATH "/calibration_data"
#define AUDIO_DATA "aud_cali_data"

/* set default PLL CLKIN to GPI2 (MCLK) = 0x00 */
#define TAS2557_DEFAULT_PLL_CLKIN 0x00

static void tas2557_load_calibration(struct tas2557_priv *pTAS2557,
	char *pFileName);
static void tas2557_load_data(struct tas2557_priv *pTAS2557, struct TData *pData,
	unsigned int nType);
static void tas2557_load_block(struct tas2557_priv *pTAS2557, struct TBlock *pBlock);
static void tas2557_load_configuration(struct tas2557_priv *pTAS2557,
	unsigned int nConfiguration, bool bLoadSame);

#define TAS2557_UDELAY 0xFFFFFFFE
#define TAS2557_MDELAY 0xFFFFFFFD

#define FW_ERR_HEADER -1
#define FW_ERR_SIZE -2

#define TAS2557_BLOCK_PLL				0x00
#define TAS2557_BLOCK_PGM_ALL			0x0d
#define TAS2557_BLOCK_PGM_DEV_A			0x01
#define TAS2557_BLOCK_PGM_DEV_B			0x08
#define TAS2557_BLOCK_CFG_COEFF_DEV_A	0x03
#define TAS2557_BLOCK_CFG_COEFF_DEV_B	0x0a
#define TAS2557_BLOCK_CFG_PRE_DEV_A		0x04
#define TAS2557_BLOCK_CFG_PRE_DEV_B		0x0b
#define TAS2557_BLOCK_CFG_POST			0x05
#define TAS2557_BLOCK_CFG_POST_POWER	0x06

static unsigned int p_tas2557_irq_config[] = {
	/* channel_both, TAS2557_GPIO4_PIN_REG, 0x07,	set GPIO4 as int1, default */
	channel_both, TAS2557_INT_GEN1_REG, 0x11,	/* enable spk OC and OV */
	channel_both, TAS2557_INT_GEN2_REG, 0x11,	/* enable clk err1 and die OT */
	channel_both, TAS2557_INT_GEN3_REG, 0x11,	/* enable clk err2 and brownout */
	channel_both, TAS2557_INT_GEN4_REG, 0x01,	/* disable SAR, enable clk halt */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_startup_data[] = {
	channel_both, TAS2557_CLK_ERR_CTRL, 0x03,	 /* enable clock error detection */
	channel_both, TAS2557_POWER_CTRL2_REG, 0xA0,	 /* Class-D, Boost power up */
	channel_both, TAS2557_POWER_CTRL2_REG, 0xA3,	 /* Class-D, Boost, IV sense power up */
	channel_both, TAS2557_POWER_CTRL1_REG, 0xF8,	 /* PLL, DSP, clock dividers power up */
	channel_both, TAS2557_UDELAY, 2000,		 /* delay */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_unmute_data[] = {
	channel_both, TAS2557_MUTE_REG, 0x00,		 /* unmute */
	channel_both, TAS2557_SOFT_MUTE_REG, 0x00,	 /* soft unmute */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_enter_broadcast_data[] = {
	channel_both, TAS2557_TEST_MODE_REG, 0x0d,		 /* enter test mode */
	channel_both, TAS2557_BROADCAST_REG, 0x81,	 /* enable broadcast */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_exit_broadcast_data[] = {
	channel_broadcast, TAS2557_TEST_MODE_REG, 0x0d,		 /* enter test mode */
	channel_broadcast, TAS2557_BROADCAST_REG, 0x01,	 /* disable broadcast */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_shutdown_data[] = {
	channel_both, TAS2557_SOFT_MUTE_REG, 0x01,	 /* soft mute */
	channel_both, TAS2557_UDELAY, 10000,		 /* delay 10ms */
	channel_both, TAS2557_MUTE_REG, 0x03,		 /* mute */
	channel_both, TAS2557_POWER_CTRL1_REG, 0x60,	 /* DSP power down */
	channel_both, TAS2557_UDELAY, 2000,		 /* delay 2ms */
	channel_both, TAS2557_POWER_CTRL2_REG, 0x00,	 /* Class-D, Boost power down */
	channel_both, TAS2557_POWER_CTRL1_REG, 0x00,	 /* all power down */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_mute_DSP_down_data[] = {
	channel_both, TAS2557_MUTE_REG, 0x03,		 /* mute */
	channel_both, TAS2557_POWER_CTRL1_REG, 0x60,	 /* DSP power down */
	channel_both, TAS2557_UDELAY, 0xFF,		 /* delay */
	0xFFFFFFFF, 0xFFFFFFFF
};

static int tas2557_dev_load_data(struct tas2557_priv *pTAS2557,
	unsigned int *pData)
{
	int ret = 0;
	unsigned int n = 0;
	enum channel chl;
	unsigned int nRegister;
	unsigned int nData;

	do {
		chl = pData[n * 3];
		nRegister = pData[n * 3 + 1];
		nData = pData[n * 3 + 2];
		if (nRegister == TAS2557_UDELAY)
			udelay(nData);
		else if (nRegister != 0xFFFFFFFF) {
			ret = pTAS2557->write(pTAS2557, chl, nRegister, nData);
			if (ret < 0) {
				dev_err(pTAS2557->dev, "Reg Write err %d\n", ret);
				break;
			}
		}
		n++;
	} while (nRegister != 0xFFFFFFFF);
	return ret;
}

void tas2557_configIRQ(struct tas2557_priv *pTAS2557)
{
	tas2557_dev_load_data(pTAS2557, p_tas2557_irq_config);
}

int tas2557_load_platdata(struct tas2557_priv *pTAS2557)
{
	int ret = 0;
	return ret;
}
int tas2557_load_default(struct tas2557_priv *pTAS2557)
{
	int ret = 0;

	ret = tas2557_load_platdata(pTAS2557);
	return ret;
}

void tas2557_enable(struct tas2557_priv *pTAS2557, bool bEnable)
{
	dev_dbg(pTAS2557->dev, "Enable: %d\n", bEnable);
	if (bEnable) {
		if (!pTAS2557->mbPowerUp) {
			dev_dbg(pTAS2557->dev, "Enable: load startup sequence\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
			dev_dbg(pTAS2557->dev, "Enable: load unmute sequence\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
			/* turn on IRQ */
			pTAS2557->enableIRQ(pTAS2557, true, false);
			pTAS2557->mbPowerUp = true;
		}
	} else {
		if (pTAS2557->mbPowerUp) {
			dev_dbg(pTAS2557->dev, "Enable: load shutdown sequence\n");
			/* turn off IRQ */
			pTAS2557->enableIRQ(pTAS2557, false, true);
			tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_data);
			 /* tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_clk_err); */
			pTAS2557->mbPowerUp = false;
		}
	}
}

int tas2557_set_bit_rate(struct tas2557_priv *pTAS2557,
	enum channel chn, unsigned int nBitRate)
{
	int ret = 0, n = -1;

	dev_dbg(pTAS2557->dev, "tas2557_set_bit_rate: nBitRate = %d\n", nBitRate);

	switch (nBitRate) {
	case 16:
		n = 0;
	break;
	case 20:
		n = 1;
	break;
	case 24:
		n = 2;
	break;
	case 32:
		n = 3;
	break;
	}
	if (n >= 0)
		ret = pTAS2557->update_bits(pTAS2557, chn,
			TAS2557_ASI1_DAC_FORMAT_REG, 0x18, n<<3);
	return ret;
}

int tas2557_get_bit_rate(struct tas2557_priv *pTAS2557,
	enum channel chn, unsigned char *pBitRate)
{
	int ret = 0;
	unsigned int nValue = 0;
	unsigned char bitRate;

	ret = pTAS2557->read(pTAS2557, chn,
		TAS2557_ASI1_DAC_FORMAT_REG, &nValue);
	if (ret >= 0) {
		bitRate = (nValue&0x18)>>3;
		if (bitRate == 0)
			bitRate = 16;
		else if (bitRate == 1)
			bitRate = 20;
		else if (bitRate == 2)
			bitRate = 24;
		else if (bitRate == 3)
			bitRate = 32;
		*pBitRate = bitRate;
	} else {
		dev_err(pTAS2557->dev, "read left channel error %d\n", ret);
	}
	return ret;
}

int tas2557_get_DAC_gain(struct tas2557_priv *pTAS2557, enum channel chl, unsigned char *pnGain)
{
	int ret = 0;
	unsigned int nValue = 0;

	ret = pTAS2557->read(pTAS2557, chl,
			TAS2557_SPK_CTRL_REG, &nValue);
	if (ret >= 0)
		*pnGain = ((nValue&TAS2557_DAC_GAIN_MASK)>>TAS2557_DAC_GAIN_SHIFT);

	return ret;
}

int tas2557_set_DAC_gain(struct tas2557_priv *pTAS2557, enum channel chl, unsigned int nGain)
{
	int ret = 0;

	ret = pTAS2557->update_bits(pTAS2557, chl, TAS2557_SPK_CTRL_REG, TAS2557_DAC_GAIN_MASK,
		(nGain<<TAS2557_DAC_GAIN_SHIFT));
	return ret;
}

int tas2557_set_sampling_rate(struct tas2557_priv *pTAS2557, unsigned int nSamplingRate)
{
	struct TConfiguration *pConfiguration;
	unsigned int nConfiguration;

	dev_dbg(pTAS2557->dev, "tas2557_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	pConfiguration = &(pTAS2557->mpFirmware->mpConfigurations[pTAS2557->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_info(pTAS2557->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2557->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
		if ((pConfiguration->mnSamplingRate == nSamplingRate)
			&& (pConfiguration->mnProgram == pTAS2557->mnCurrentProgram)) {
			dev_info(pTAS2557->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				pConfiguration->mpName, nSamplingRate);
			tas2557_load_configuration(pTAS2557, nConfiguration, false);
			return 0;
		}
	}

	dev_err(pTAS2557->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);
	return -EINVAL;
}

static void fw_print_header(struct tas2557_priv *pTAS2557, struct TFirmware *pFirmware)
{
	dev_info(pTAS2557->dev, "FW Size       = %d", pFirmware->mnFWSize);
	dev_info(pTAS2557->dev, "Checksum      = 0x%04X", pFirmware->mnChecksum);
	dev_info(pTAS2557->dev, "PPC Version   = 0x%04X", pFirmware->mnPPCVersion);
	dev_info(pTAS2557->dev, "FW  Version    = 0x%04X", pFirmware->mnFWVersion);
	dev_info(pTAS2557->dev, "Driver Version= 0x%04X", pFirmware->mnDriverVersion);
	dev_info(pTAS2557->dev, "Timestamp     = %d", pFirmware->mnTimeStamp);
	dev_info(pTAS2557->dev, "DDC Name      = %s", pFirmware->mpDDCName);
	dev_info(pTAS2557->dev, "Description   = %s", pFirmware->mpDescription);
}

inline unsigned int fw_convert_number(unsigned char *pData)
{
	return pData[3] + (pData[2] << 8) + (pData[1] << 16) + (pData[0] << 24);
}

static int fw_parse_header(struct tas2557_priv *pTAS2557,
	struct TFirmware *pFirmware, unsigned char *pData, unsigned int nSize)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };

	if (nSize < 104) {
		dev_err(pTAS2557->dev, "Firmware: Header too short");
		return -EINVAL;
	}

	if (memcmp(pData, pMagicNumber, 4)) {
		dev_err(pTAS2557->dev, "Firmware: Magic number doesn't match");
		return -EINVAL;
	}
	pData += 4;

	pFirmware->mnFWSize = fw_convert_number(pData);
	pData += 4;
	pFirmware->mnChecksum = fw_convert_number(pData);
	pData += 4;
	pFirmware->mnPPCVersion = fw_convert_number(pData);
	pData += 4;
	pFirmware->mnFWVersion = fw_convert_number(pData);
	pData += 4;
	pFirmware->mnDriverVersion = fw_convert_number(pData);
	pData += 4;
	pFirmware->mnTimeStamp = fw_convert_number(pData);
	pData += 4;
	memcpy(pFirmware->mpDDCName, pData, 64);
	pData += 64;
	n = strlen(pData);
	pFirmware->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);

	if (!pFirmware->mpDescription) {
		dev_err(pTAS2557->dev, "%s:mpDescription fail to allocate mmeory", __func__);
		return -ENOMEM;
	}

	pData += n + 1;
	if ((pData - pDataStart) >= nSize) {
		dev_err(pTAS2557->dev, "Firmware: Header too short after DDC description");
		return -EINVAL;
	}

	pFirmware->mnDeviceFamily = fw_convert_number(pData);
	pData += 4;

	if (pFirmware->mnDeviceFamily != 0) {
		dev_err(pTAS2557->dev,
			"deviceFamily %d, not TAS device", pFirmware->mnDeviceFamily);
		return -EINVAL;
	}
	pFirmware->mnDevice = fw_convert_number(pData);
	pData += 4;

	if (pFirmware->mnDevice != 3) {
		dev_err(pTAS2557->dev,
			"device %d, not TAS2557 Dual Mono", pFirmware->mnDevice);
		return -EINVAL;
	}
	fw_print_header(pTAS2557, pFirmware);
	return pData - pDataStart;
}

static int fw_parse_block_data(struct tas2557_priv *pTAS2557,
	struct TBlock *pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;

	dev_dbg(pTAS2557->dev, "TBlock type[%d]\n", pBlock->mnType);
	pBlock->mnCommands = fw_convert_number(pData);
	pData += 4;

	n = pBlock->mnCommands * 4;
	pBlock->mpData = kmemdup(pData, n, GFP_KERNEL);

	if (!pBlock->mpData) {
		dev_err(pTAS2557->dev, "%s:mdata fail to allocate memory\n", __func__);
		return -ENOMEM;
	}

	pData += n;
	return pData - pDataStart;
}

static int fw_parse_data(struct tas2557_priv *pTAS2557,
	struct TData *pImageData, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int nBlock;
	int n;

	memcpy(pImageData->mpName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pImageData->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);

	if (!pImageData->mpDescription) {
		dev_err(pTAS2557->dev, "%s:mpBlocks failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	pData += n + 1;
	pImageData->mnBlocks = (pData[0] << 8) + pData[1];
	pData += 2;
	pImageData->mpBlocks =
		kzalloc(sizeof(struct TBlock) * pImageData->mnBlocks, GFP_KERNEL);

	if (!pImageData->mpBlocks) {
		dev_err(pTAS2557->dev, "%s:mpBlocks failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		n = fw_parse_block_data(pTAS2557,
			&(pImageData->mpBlocks[nBlock]), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s:block parse fail %d\n", __func__, n);
			return n;
		}

		pData += n;
	}
	return pData - pDataStart;
}

static int fw_parse_pll_data(struct tas2557_priv *pTAS2557,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	int n;
	unsigned int nPLL;
	struct TPLL *pPLL;

	pFirmware->mnPLLs = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPLLs = kcalloc(pFirmware->mnPLLs, sizeof(struct TPLL), GFP_KERNEL);

	if (!pFirmware->mpPLLs) {
		dev_err(pTAS2557->dev, "%s:mpPLLS failed to allocate memroy\n", __func__);
		return -ENOMEM;
	}

	for (nPLL = 0; nPLL < pFirmware->mnPLLs; nPLL++) {
		pPLL = &(pFirmware->mpPLLs[nPLL]);

		memcpy(pPLL->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pPLL->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);

		if (!pPLL->mpDescription) {
			dev_err(pTAS2557->dev, "%s:mpDescription out of memory\n", __func__);
			return -ENOMEM;
		}

		pData += n + 1;

		dev_dbg(pTAS2557->dev, "PLL[%d] Name=%s\n", nPLL, pPLL->mpName);
		dev_dbg(pTAS2557->dev, "PLL[%d] Desc=%s\n", nPLL, pPLL->mpDescription);
		n = fw_parse_block_data(pTAS2557, &(pPLL->mBlock), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s:parse block error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}
	return pData - pDataStart;
}

static int fw_parse_program_data(struct tas2557_priv *pTAS2557,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	int n;
	unsigned int nProgram;
	struct TProgram *pProgram;

	pFirmware->mnPrograms = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPrograms =
		kzalloc(sizeof(struct TProgram) * pFirmware->mnPrograms, GFP_KERNEL);

	if (!pFirmware->mpPrograms) {
		dev_err(pTAS2557->dev, "%s:mpPrograms failed to allocate memroy\n", __func__);
		return -ENOMEM;
	}

	for (nProgram = 0; nProgram < pFirmware->mnPrograms; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		memcpy(pProgram->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pProgram->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);

		if (!pProgram->mpDescription) {
			dev_err(pTAS2557->dev, "%s: mpDescription out of memory\n", __func__);
			return -ENOMEM;
		}

		pData += n + 1;
		pProgram->mnAppMode = pData[0];
		pData++;
		pProgram->mnBoost = (pData[0] << 8) + pData[1];
		pData += 2;
		n = fw_parse_data(pTAS2557, &(pProgram->mData), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s: parse data error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}
	return pData - pDataStart;
}

static int fw_parse_configuration_data(struct tas2557_priv *pTAS2557,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	int n;
	unsigned int nConfiguration;
	struct TConfiguration *pConfiguration;

	pFirmware->mnConfigurations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpConfigurations =
		kzalloc(sizeof(struct TConfiguration) * pFirmware->mnConfigurations,
		GFP_KERNEL);

	if (!pFirmware->mpConfigurations) {
		dev_err(pTAS2557->dev, "%s:mpConfigurations failed to allocate memroy\n", __func__);
		return -ENOMEM;
	}

	for (nConfiguration = 0; nConfiguration < pFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		memcpy(pConfiguration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pConfiguration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);

		if (!pConfiguration->mpDescription) {
			dev_err(pTAS2557->dev, "%s:mpDescription out of memory\n", __func__);
			return -ENOMEM;
		}

		pData += n + 1;

		pConfiguration->mnProgram = pData[0];
		pData++;
		pConfiguration->mnPLL = pData[0];
		pData++;

		pConfiguration->mnSamplingRate = fw_convert_number(pData);
		pData += 4;

		n = fw_parse_data(pTAS2557, &(pConfiguration->mData), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s:parse data error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}
	return pData - pDataStart;
}

int fw_parse_calibration_data(struct tas2557_priv *pTAS2557,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	int n;
	unsigned int nCalibration;
	struct TCalibration *pCalibration;

	pFirmware->mnCalibrations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpCalibrations =
		kzalloc(sizeof(struct TCalibration) * pFirmware->mnCalibrations, GFP_KERNEL);

	if (!pFirmware->mpCalibrations) {
		dev_err(pTAS2557->dev, "%s:mpCalibrations failed to allocate memroy\n", __func__);
		return -ENOMEM;
	}

	for (nCalibration = 0;
		nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		pCalibration = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(pCalibration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pCalibration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);

		if (!pCalibration->mpDescription) {
			dev_err(pTAS2557->dev, "%s:mpDescription out of memory\n", __func__);
			return -ENOMEM;
		}

		pData += n + 1;

		pCalibration->mnProgram = pData[0];
		pData++;

		pCalibration->mnConfiguration = pData[0];
		pData++;

		n = fw_parse_data(pTAS2557, &(pCalibration->mData), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s: parse data error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}
	return pData - pDataStart;
}

static int fw_parse(struct tas2557_priv *pTAS2557,
	struct TFirmware *pFirmware, unsigned char *pData, unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(pTAS2557, pFirmware, pData, nSize);
	if (nPosition < 0) {
		dev_err(pTAS2557->dev, "Firmware: Wrong Header");
		return FW_ERR_HEADER;
	}

	if (nPosition >= nSize) {
		dev_err(pTAS2557->dev, "Firmware: Too short");
		return FW_ERR_SIZE;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(pTAS2557, pFirmware, pData);

	if (nPosition < 0) {
		dev_err(pTAS2557->dev, "%s:parse pll fail %d\n", __func__, nPosition);
		return nPosition;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(pTAS2557, pFirmware, pData);

	if (nPosition < 0) {
		dev_err(pTAS2557->dev, "%s:parse program fail %d\n", __func__, nPosition);
		return nPosition;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(pTAS2557, pFirmware, pData);

	if (nPosition < 0) {
		dev_err(pTAS2557->dev, "%s:parse configuration fail %d\n", __func__, nPosition);
		return nPosition;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64) {
		nPosition = fw_parse_calibration_data(pTAS2557, pFirmware, pData);

		if (nPosition < 0) {
			dev_err(pTAS2557->dev, "%s:parse calibration fail %d\n", __func__, nPosition);
			return nPosition;
		}
	}

	return 0;
}

static void tas2557_load_block(struct tas2557_priv *pTAS2557, struct TBlock *pBlock)
{
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nLength;
	enum channel chl;
	unsigned char *pData = pBlock->mpData;

	dev_dbg(pTAS2557->dev, "TAS2557 load block: Type = %d, commands = %d\n",
		pBlock->mnType, pBlock->mnCommands);
	if ((pBlock->mnType == TAS2557_BLOCK_PLL)
		|| (pBlock->mnType == TAS2557_BLOCK_CFG_POST)
		|| (pBlock->mnType == TAS2557_BLOCK_CFG_POST_POWER)) {
		chl = channel_both;
	} else if ((pBlock->mnType == TAS2557_BLOCK_PGM_DEV_A)
		|| (pBlock->mnType == TAS2557_BLOCK_CFG_COEFF_DEV_A)
		|| (pBlock->mnType == TAS2557_BLOCK_CFG_PRE_DEV_A)) {
		chl = channel_left;
	} else if ((pBlock->mnType == TAS2557_BLOCK_PGM_DEV_B)
		|| (pBlock->mnType == TAS2557_BLOCK_CFG_COEFF_DEV_B)
		|| (pBlock->mnType == TAS2557_BLOCK_CFG_PRE_DEV_B)) {
		chl = channel_right;
	} else if (pBlock->mnType == TAS2557_BLOCK_PGM_ALL) {
		chl = channel_broadcast;
	} else {
		dev_err(pTAS2557->dev, "block type error %d\n", pBlock->mnType);
		return;
	}
	if (chl == channel_broadcast)
		tas2557_dev_load_data(pTAS2557, p_tas2557_enter_broadcast_data);

	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;

		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F) {
			pTAS2557->write(pTAS2557, chl, TAS2557_REG(nBook, nPage, nOffset), nData);
		} else if (nOffset == 0x81) {
			unsigned int nSleep = (nBook << 8) + nPage;

			msleep(nSleep);
		} else if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1)
				pTAS2557->bulk_write(pTAS2557, chl, TAS2557_REG(nBook, nPage, nOffset), pData + 3, nLength);
			else
				pTAS2557->write(pTAS2557, chl, TAS2557_REG(nBook, nPage, nOffset), pData[3]);

			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}

	if (chl == channel_broadcast)
		tas2557_dev_load_data(pTAS2557, p_tas2557_exit_broadcast_data);
}

static void tas2557_load_data(struct tas2557_priv *pTAS2557, struct TData *pData, unsigned int nType)
{
	unsigned int nBlock;
	struct TBlock *pBlock;

	dev_dbg(pTAS2557->dev,
		"TAS2557 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName, pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType)
			tas2557_load_block(pTAS2557, pBlock);
	}
}

static void tas2557_load_configuration(struct tas2557_priv *pTAS2557,
	unsigned int nConfiguration, bool bLoadSame)
{
	struct TConfiguration *pCurrentConfiguration = NULL;
	struct TConfiguration *pNewConfiguration = NULL;
	struct TCalibration *pCalibration = NULL;
	struct TPLL *pNewPLL = NULL;

	dev_dbg(pTAS2557->dev, "tas2557_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return;
	}

	if (nConfiguration >= pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return;
	}

	if ((nConfiguration == pTAS2557->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_info(pTAS2557->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		return;
	}

	pCurrentConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[pTAS2557->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	if (pNewConfiguration->mnProgram != pCurrentConfiguration->mnProgram) {
		dev_err(pTAS2557->dev, "Configuration %d, %s doesn't share the same program as current %d\n",
			nConfiguration, pNewConfiguration->mpName, pCurrentConfiguration->mnProgram);
		return;
	}

	if (pNewConfiguration->mnPLL >= pTAS2557->mpFirmware->mnPLLs) {
		dev_err(pTAS2557->dev, "Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		return;
	}
	pNewPLL = &(pTAS2557->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);
	if (pTAS2557->mpCalFirmware->mnCalibrations)
		pCalibration = &(pTAS2557->mpCalFirmware->mpCalibrations[pTAS2557->mnCurrentCalibration]);

	if (pTAS2557->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			pTAS2557->enableIRQ(pTAS2557, false, true);
			dev_dbg(pTAS2557->dev,
				"TAS2557 is powered up, power down DSP before loading new configuration\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_data);
			dev_dbg(pTAS2557->dev, "TAS2557: load new PLL: %s, block data\n", pNewPLL->mpName);
			tas2557_load_block(pTAS2557, &(pNewPLL->mBlock));
			pTAS2557->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2557->dev, "load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_A);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_B);
			dev_dbg(pTAS2557->dev, "TAS2557: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_B);
			if (pTAS2557->mpCalFirmware->mnCalibrations) {
				dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
				tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_A);
				tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_B);
			}
			dev_dbg(pTAS2557->dev, "TAS2557: power up TAS2557\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
			dev_dbg(pTAS2557->dev, "TAS2557: unmute TAS2557\n");
			tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
			pTAS2557->enableIRQ(pTAS2557, true, false);
		} else {
			dev_dbg(pTAS2557->dev,
				"TAS2557 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_B);
			if (pTAS2557->mpCalFirmware->mnCalibrations) {
				dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
				tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_A);
				tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_B);
			}
		}
		pTAS2557->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2557->dev,
			"TAS2557 was powered down\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2557->dev, "TAS2557: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2557_load_block(pTAS2557, &(pNewPLL->mBlock));
			pTAS2557->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2557->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_A);
			tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_B);
		}
		dev_dbg(pTAS2557->dev, "TAS2557: load new configuration: %s, coeff block data\n",
			pNewConfiguration->mpName);
		tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
			TAS2557_BLOCK_CFG_COEFF_DEV_A);
		tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
			TAS2557_BLOCK_CFG_COEFF_DEV_B);
		if (pTAS2557->mpCalFirmware->mnCalibrations) {
			dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
			tas2557_load_data(pTAS2557, &(pCalibration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);
			tas2557_load_data(pTAS2557, &(pCalibration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_B);
		}
		pTAS2557->mbLoadConfigurationPostPowerUp = true;
	}
	pTAS2557->mnCurrentConfiguration = nConfiguration;
}

int tas2557_set_config(struct tas2557_priv *pTAS2557, int config)
{
	struct TConfiguration *pConfiguration;
	struct TProgram *pProgram;
	unsigned int nProgram = pTAS2557->mnCurrentProgram;
	unsigned int nConfiguration = config;

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	if (nConfiguration >= pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return -EINVAL;
	}

	pConfiguration = &(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2557->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2557->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		return -EINVAL;
	}

	tas2557_load_configuration(pTAS2557, nConfiguration, false);
	return 0;
}

void tas2557_clear_firmware(struct TFirmware *pFirmware)
{
	unsigned int n, nn;

	if (!pFirmware)
		return;

	kfree(pFirmware->mpDescription);
	pFirmware->mpDescription = NULL;

	if (pFirmware->mpPLLs != NULL) {
		for (n = 0; n < pFirmware->mnPLLs; n++) {
			kfree(pFirmware->mpPLLs[n].mpDescription);
			pFirmware->mpPLLs[n].mpDescription = NULL;

			kfree(pFirmware->mpPLLs[n].mBlock.mpData);
			pFirmware->mpPLLs[n].mBlock.mpData = NULL;
		}
		kfree(pFirmware->mpPLLs);
		pFirmware->mpPLLs = NULL;
	}

	if (pFirmware->mpPrograms != NULL) {
		for (n = 0; n < pFirmware->mnPrograms; n++) {
			kfree(pFirmware->mpPrograms[n].mpDescription);
			pFirmware->mpPrograms[n].mpDescription = NULL;

			kfree(pFirmware->mpPrograms[n].mData.mpDescription);
			pFirmware->mpPrograms[n].mData.mpDescription = NULL;

			for (nn = 0; nn < pFirmware->mpPrograms[n].mData.mnBlocks; nn++) {
				kfree(pFirmware->mpPrograms[n].mData.mpBlocks[nn].mpData);
				pFirmware->mpPrograms[n].mData.mpBlocks[nn].mpData = NULL;
			}

			kfree(pFirmware->mpPrograms[n].mData.mpBlocks);
			pFirmware->mpPrograms[n].mData.mpBlocks = NULL;
		}
		kfree(pFirmware->mpPrograms);
		pFirmware->mpPrograms = NULL;
	}

	if (pFirmware->mpConfigurations != NULL) {
		for (n = 0; n < pFirmware->mnConfigurations; n++) {

			kfree(pFirmware->mpConfigurations[n].mpDescription);
			pFirmware->mpConfigurations[n].mpDescription = NULL;

			kfree(pFirmware->mpConfigurations[n].mData.mpDescription);
			pFirmware->mpConfigurations[n].mData.mpDescription = NULL;

			for (nn = 0; nn < pFirmware->mpConfigurations[n].mData.mnBlocks; nn++) {
				kfree(pFirmware->mpConfigurations[n].mData.mpBlocks[nn].mpData);
				pFirmware->mpConfigurations[n].mData.mpBlocks[nn].mpData = NULL;
			}

			kfree(pFirmware->mpConfigurations[n].mData.mpBlocks);
			pFirmware->mpConfigurations[n].mData.mpBlocks = NULL;
		}

		kfree(pFirmware->mpConfigurations);
		pFirmware->mpConfigurations = NULL;
	}

	if (pFirmware->mpCalibrations != NULL) {
		for (n = 0; n < pFirmware->mnCalibrations; n++) {

			kfree(pFirmware->mpCalibrations[n].mpDescription);
			pFirmware->mpCalibrations[n].mpDescription = NULL;

			kfree(pFirmware->mpCalibrations[n].mData.mpDescription);
			pFirmware->mpCalibrations[n].mData.mpDescription = NULL;

			for (nn = 0; nn < pFirmware->mpCalibrations[n].mData.mnBlocks; nn++) {
				kfree(pFirmware->mpCalibrations[n].mData.mpBlocks[nn].mpData);
				pFirmware->mpCalibrations[n].mData.mpBlocks[nn].mpData = NULL;
			}

			kfree(pFirmware->mpCalibrations[n].mData.mpBlocks);
			pFirmware->mpCalibrations[n].mData.mpBlocks = NULL;
		}
		kfree(pFirmware->mpCalibrations);
		pFirmware->mpCalibrations = NULL;
	}

	memset(pFirmware, 0x00, sizeof(struct TFirmware));
}

static void tas2557_load_calibration(struct tas2557_priv *pTAS2557,	char *pFileName)
{
	int nResult;
	unsigned char pBuffer[512];
	int nSize = 0;

	struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	int p_size = 0;
	unsigned char *p_data = NULL;
	if (offset != NULL) {
		dev_info(pTAS2557->dev, "TAS2557 cali_data load success from dts\n");
		p_data = (unsigned char *) of_get_property(offset, AUDIO_DATA, &p_size);
	} else {
		dev_err(pTAS2557->dev, "TAS2557 cali_data load fail on dts\n");
	}

	dev_dbg(pTAS2557->dev, "%s:\n", __func__);
	if (p_size != 0) {
		dev_info(pTAS2557->dev, "TAS2557 use cali_data from dts\n");
		memcpy(pBuffer, p_data, 512);
		nSize = p_size;
	}

	if (!nSize)
		return;

	tas2557_clear_firmware(pTAS2557->mpCalFirmware);
	dev_info(pTAS2557->dev, "TAS2557 calibration file size = %d\n", nSize);
	nResult = fw_parse(pTAS2557, pTAS2557->mpCalFirmware, pBuffer, nSize);

	if (nResult) {
		dev_err(pTAS2557->dev, "TAS2557 calibration file is corrupt\n");
		tas2557_clear_firmware(pTAS2557->mpCalFirmware);
		return;
	}

	dev_info(pTAS2557->dev, "TAS2557 calibration: %d calibrations\n",
		pTAS2557->mpCalFirmware->mnCalibrations);
}

void tas2557_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas2557_priv *pTAS2557 = (struct tas2557_priv *) pContext;
	int nResult;
	unsigned int nProgram = 0;
	unsigned int nSampleRate = 0;

	dev_info(pTAS2557->dev, "%s:\n", __func__);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_err(pTAS2557->dev, "%s firmware is not loaded.\n",
			TAS2557_FW_NAME);
		nResult = tas2557_load_default(pTAS2557);
		return;
	}

	if (pTAS2557->mpFirmware->mpConfigurations) {
		nProgram = pTAS2557->mnCurrentProgram;
		nSampleRate = pTAS2557->mnCurrentSampleRate;
		dev_dbg(pTAS2557->dev, "clear current firmware\n");
		tas2557_clear_firmware(pTAS2557->mpFirmware);
	}
	nResult = fw_parse(pTAS2557, pTAS2557->mpFirmware,
		(unsigned char *) (pFW->data),	pFW->size);
	release_firmware(pFW);
	if (nResult) {
		dev_err(pTAS2557->dev, "firmware is corrupt\n");
		tas2557_clear_firmware(pTAS2557->mpFirmware);
		return;
	}

	if (!pTAS2557->mpFirmware->mnPrograms) {
		dev_err(pTAS2557->dev, "firmware contains no programs\n");
		return;
	}
	if (!pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "firmware contains no configurations\n");
		return;
	}
	if (nProgram >= pTAS2557->mpFirmware->mnPrograms) {
		dev_info(pTAS2557->dev,
			"no previous program, set to default\n");
		nProgram = 0;
	}
	pTAS2557->mnCurrentSampleRate = nSampleRate;

	tas2557_set_program(pTAS2557, nProgram, -1);
}

int tas2557_set_program(struct tas2557_priv *pTAS2557, unsigned int nProgram, int nConfig)
{
	struct TPLL *pPLL;
	struct TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	unsigned int Value = 0;
	bool bFound = false;
	int nResult = -1;

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		return -EINVAL;
	}
	if (nProgram >= pTAS2557->mpFirmware->mnPrograms) {
		dev_err(pTAS2557->dev, "TAS2557: Program %d doesn't exist\n",
			nProgram);
		return -EINVAL;
	}

	if (nConfig < 0) {
		nConfiguration = 0;
		nSampleRate = pTAS2557->mnCurrentSampleRate;
		while (!bFound && (nConfiguration < pTAS2557->mpFirmware->mnConfigurations)) {
			if (pTAS2557->mpFirmware->mpConfigurations[nConfiguration].mnProgram == nProgram) {
				if (nSampleRate == 0) {
					bFound = true;
					dev_info(pTAS2557->dev, "find default configuration %d\n", nConfiguration);
				} else if (nSampleRate == pTAS2557->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate) {
					bFound = true;
					dev_info(pTAS2557->dev, "find matching configuration %d\n", nConfiguration);
				} else {
					nConfiguration++;
				}
			} else {
				nConfiguration++;
			}
		}
		if (!bFound) {
			dev_err(pTAS2557->dev,
				"Program %d, no valid configuration found for sample rate %d, ignore\n",
				nProgram, nSampleRate);
			return -EINVAL;
		}
	} else
		nConfiguration = nConfig;

	pTAS2557->mnCurrentProgram = nProgram;
	if (pTAS2557->mbPowerUp) {
		pTAS2557->enableIRQ(pTAS2557, false, true);
		nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_mute_DSP_down_data);
	}

	pTAS2557->write(pTAS2557, channel_both, TAS2557_SW_RESET_REG, 0x01);
	msleep(1);
	nResult = tas2557_load_default(pTAS2557);
	dev_info(pTAS2557->dev, "load program %d\n", nProgram);
	tas2557_load_data(pTAS2557,
		&(pTAS2557->mpFirmware->mpPrograms[nProgram].mData),
		TAS2557_BLOCK_PGM_ALL);
	tas2557_load_data(pTAS2557,
		&(pTAS2557->mpFirmware->mpPrograms[nProgram].mData),
		TAS2557_BLOCK_PGM_DEV_A);
	tas2557_load_data(pTAS2557,
		&(pTAS2557->mpFirmware->mpPrograms[nProgram].mData),
		TAS2557_BLOCK_PGM_DEV_B);

	nResult = pTAS2557->read(pTAS2557, channel_left, TAS2557_CRC_CHECKSUM_REG, &Value);
	dev_info(pTAS2557->dev, "Left uCDSP Checksum: 0x%02x\n", Value);
	nResult = pTAS2557->read(pTAS2557, channel_right, TAS2557_CRC_CHECKSUM_REG, &Value);
	dev_info(pTAS2557->dev, "Right uCDSP Checksum: 0x%02x\n", Value);
	pTAS2557->mnCurrentConfiguration = nConfiguration;

	pConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	pPLL = &(pTAS2557->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
	dev_dbg(pTAS2557->dev,
		"TAS2557 load PLL: %s block for Configuration %s\n",
		pPLL->mpName, pConfiguration->mpName);
	tas2557_load_block(pTAS2557, &(pPLL->mBlock));
	pTAS2557->mnCurrentSampleRate = pConfiguration->mnSamplingRate;
	dev_dbg(pTAS2557->dev,
		"load configuration %s conefficient pre block\n",
		pConfiguration->mpName);
	tas2557_load_data(pTAS2557, &(pConfiguration->mData), TAS2557_BLOCK_CFG_PRE_DEV_A);
	tas2557_load_data(pTAS2557, &(pConfiguration->mData), TAS2557_BLOCK_CFG_PRE_DEV_B);

	tas2557_load_configuration(pTAS2557, nConfiguration, true);
	if (pTAS2557->mbPowerUp) {
		dev_dbg(pTAS2557->dev, "device powered up, load startup\n");
		tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
		dev_dbg(pTAS2557->dev,
			"device powered up, load unmute\n");
		tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
		pTAS2557->enableIRQ(pTAS2557, true, false);
	}
	return 0;
}

int tas2557_set_calibration(struct tas2557_priv *pTAS2557, int nCalibration)
{
	struct TCalibration *pCalibration = NULL;

	if ((!pTAS2557->mpFirmware->mpPrograms)
		|| (!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n\r");
		return -EINVAL;
	}

	if (nCalibration == 0x00FF) {
		dev_info(pTAS2557->dev, "load new calibration file %s\n", TAS2557_CAL_NAME);
		tas2557_load_calibration(pTAS2557, TAS2557_CAL_NAME);
		nCalibration = 0;
	}

	if (nCalibration >= pTAS2557->mpCalFirmware->mnCalibrations) {
		dev_err(pTAS2557->dev,
			"Calibration %d doesn't exist\n", nCalibration);
		return -EINVAL;
	}
	pTAS2557->mnCurrentCalibration = nCalibration;
	pCalibration = &(pTAS2557->mpCalFirmware->mpCalibrations[nCalibration]);
	dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
	tas2557_load_data(pTAS2557, &(pCalibration->mData), TAS2557_BLOCK_CFG_COEFF_DEV_A);
	tas2557_load_data(pTAS2557, &(pCalibration->mData), TAS2557_BLOCK_CFG_COEFF_DEV_B);
	return 0;
}

int tas2557_parse_dt(struct device *dev, struct tas2557_priv *pTAS2557)
{
	struct device_node *np = dev->of_node;
	int rc = 0, ret = 0;
	unsigned int value;

	rc = of_property_read_u32(np, "ti,load", &pTAS2557->mnLoad);
	if (rc) {
		dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
			"ti,load", np->full_name, rc);
		ret = -1;
	} else {
		dev_dbg(pTAS2557->dev, "ti,load=%d", pTAS2557->mnLoad);
	}

	if (ret >= 0) {
		pTAS2557->mnResetGPIO = of_get_named_gpio(np, "ti,cdc-reset-gpio", 0);
		if (pTAS2557->mnResetGPIO < 0) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,cdc-reset-gpio", np->full_name,
				pTAS2557->mnResetGPIO);
			ret = -1;
		} else {
			dev_dbg(pTAS2557->dev, "ti,cdc-reset-gpio=%d\n", pTAS2557->mnResetGPIO);
		}
	}

	if (ret >= 0) {
		pTAS2557->mnLeftChlGpioINT = of_get_named_gpio(np, "ti,irq-gpio-left", 0);
		if (pTAS2557->mnLeftChlGpioINT < 0) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,irq-gpio-left", np->full_name,
				pTAS2557->mnLeftChlGpioINT);
			ret = -1;
		} else {
			dev_dbg(pTAS2557->dev, "ti,irq-gpio-left=%d\n", pTAS2557->mnLeftChlGpioINT);
		}
	}

	if (ret >= 0) {
		pTAS2557->mnRightChlGpioINT = of_get_named_gpio(np, "ti,irq-gpio-right", 0);
		if (pTAS2557->mnRightChlGpioINT < 0) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,irq-gpio-right", np->full_name,
				pTAS2557->mnRightChlGpioINT);
			ret = -1;
		} else {
			dev_dbg(pTAS2557->dev, "ti,irq-gpio-right=%d\n", pTAS2557->mnRightChlGpioINT);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,left-channel", &value);
		if (rc) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,left-channel", np->full_name, rc);
			ret = -2;
		} else {
			pTAS2557->mnLAddr = value;
			dev_dbg(pTAS2557->dev, "ti,left-channel=0x%x\n", pTAS2557->mnLAddr);
		}
	}
	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,right-channel", &value);
		if (rc) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,right-channel", np->full_name, rc);
			ret = -3;
		} else {
			pTAS2557->mnRAddr = value;
			dev_dbg(pTAS2557->dev, "ti,right-channel=0x%x", pTAS2557->mnRAddr);
		}
	}
	return ret;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2557 common functions for Android Linux");
MODULE_LICENSE("GPL v2");
