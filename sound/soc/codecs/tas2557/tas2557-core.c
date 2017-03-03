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
#include <linux/crc8.h>

#include "tas2557.h"
#include "tas2557-core.h"

#define PPC_DRIVER_VERSION			0x00000200
#define TAS2557_CAL_NAME    "/data/tas2557_cal.bin"
#define CALIBRATION_DATA_PATH "/calibration_data"
#define AUDIO_DATA "aud_cali_data"

/* set default PLL CLKIN to GPI2 (MCLK) = 0x00 */
#define TAS2557_DEFAULT_PLL_CLKIN 0x00

static int tas2557_load_calibration(struct tas2557_priv *pTAS2557,
	char *pFileName);
static int tas2557_load_data(struct tas2557_priv *pTAS2557, struct TData *pData,
	unsigned int nType);
static void tas2557_clear_firmware(struct TFirmware *pFirmware);
static int tas2557_load_block(struct tas2557_priv *pTAS2557, struct TBlock *pBlock);
static int tas2557_load_configuration(struct tas2557_priv *pTAS2557,
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

static unsigned int p_tas2557_default_data[] = {
	channel_both, TAS2557_SAR_ADC2_REG, 0x05,	/* enable SAR ADC */
/*TODO channel_both, TAS2555_CLK_ERR_CTRL2, 0x39,	enable clock error detection on PLL */
/*TODO channel_both, TAS2555_CLK_ERR_CTRL3, 0x11,	enable clock error detection on PLL */
	channel_both, TAS2557_SAFE_GUARD_REG, TAS2557_SAFE_GUARD_PATTERN,	/* safe guard */
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_irq_config[] = {
/*	channel_both, TAS2555_CLK_HALT_REG, 0x71,	 TODO */
	channel_both, TAS2557_INT_GEN1_REG, 0x11,	/* enable spk OC and OV */
	channel_both, TAS2557_INT_GEN2_REG, 0x11,	/* enable clk err1 and die OT */
	channel_both, TAS2557_INT_GEN3_REG, 0x11,	/* enable clk err2 and brownout */
	channel_both, TAS2557_INT_GEN4_REG, 0x01,	/* disable SAR, enable clk halt */
	channel_both, TAS2557_INT_MODE_REG, 0x80,	/* active high until INT_STICKY_1 and INT_STICKY_2 are read to be cleared. */
	channel_both, TAS2557_GPIO4_PIN_REG, 0x07,	/* set GPIO4 as int1, default */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_startup_data[] = {
	channel_both, TAS2557_GPI_PIN_REG, 0x15,	/* enable DIN, MCLK, CCI */
	channel_both, TAS2557_GPIO1_PIN_REG, 0x01,	/* enable BCLK */
	channel_both, TAS2557_GPIO2_PIN_REG, 0x01,	/* enable WCLK */
	channel_both, TAS2557_CLK_ERR_CTRL, 0x00,	 /* enable clock error detection */
	channel_both, TAS2557_POWER_CTRL2_REG, 0xA0,	 /* Class-D, Boost power up */
	channel_both, TAS2557_POWER_CTRL2_REG, 0xA3,	 /* Class-D, Boost, IV sense power up */
	channel_both, TAS2557_POWER_CTRL1_REG, 0xF8,	 /* PLL, DSP, clock dividers power up */
	channel_both, TAS2557_UDELAY, 2000,		 /* delay */
/*	channel_both, TAS2557_CLK_ERR_CTRL, 0x03,	 TODO  enable clock error detection */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_romMode_startup_data[] = {
	TAS2557_GPI_PIN_REG, 0x05,	/* enable DIN, MCLK */
	TAS2557_GPIO1_PIN_REG, 0x01,	/* enable BCLK */
	TAS2557_GPIO2_PIN_REG, 0x01,	/* enable WCLK */
	TAS2557_CLK_ERR_CTRL, 0x00,	/* disable clock error detection */
	TAS2557_POWER_CTRL2_REG, 0xA0,	 /* Class-D, Boost power up */
	TAS2557_POWER_CTRL2_REG, 0xA3,	 /* Class-D, Boost, IV sense power up */
	TAS2557_POWER_CTRL1_REG, 0xF8,	 /* PLL, DSP, clock dividers power up */
	TAS2557_UDELAY, 2000,		 /* delay */
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_unmute_data[] = {
	channel_both, TAS2557_MUTE_REG, 0x00,		 /* unmute */
	channel_both, TAS2557_SOFT_MUTE_REG, 0x00,	 /* soft unmute */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_unmute_chl_data[] = {
	TAS2557_MUTE_REG, 0x00,		 /* unmute */
	TAS2557_SOFT_MUTE_REG, 0x00,	 /* soft unmute */
	0xFFFFFFFF, 0xFFFFFFFF
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
	channel_both, TAS2557_CLK_ERR_CTRL, 0x00,	 /* disable clock error detection */
	channel_both, TAS2557_SOFT_MUTE_REG, 0x01,	 /* soft mute */
	channel_both, TAS2557_UDELAY, 10000,		 /* delay 10ms */
	channel_both, TAS2557_MUTE_REG, 0x03,		 /* mute */
	channel_both, TAS2557_POWER_CTRL1_REG, 0x60,	 /* DSP power down */
	channel_both, TAS2557_UDELAY, 2000,		 /* delay 2ms */
	channel_both, TAS2557_POWER_CTRL2_REG, 0x00,	 /* Class-D, Boost power down */
	channel_both, TAS2557_POWER_CTRL1_REG, 0x00,	 /* all power down */
	channel_both, TAS2557_GPIO1_PIN_REG, 0x00,	/* disable BCLK */
	channel_both, TAS2557_GPIO2_PIN_REG, 0x00,	/* disable WCLK */
	channel_both, TAS2557_GPI_PIN_REG, 0x00,	/* disable DIN, MCLK, CCI */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_romMode_shutdown_data[] = {
	TAS2557_CLK_ERR_CTRL, 0x00,	 /* disable clock error detection */
	TAS2557_SOFT_MUTE_REG, 0x01,	 /* soft mute */
	TAS2557_UDELAY, 10000,		 /* delay 10ms */
	TAS2557_MUTE_REG, 0x03,		 /* mute */
	TAS2557_POWER_CTRL1_REG, 0x60,	 /* DSP power down */
	TAS2557_UDELAY, 2000,		 /* delay 2ms */
	TAS2557_POWER_CTRL2_REG, 0x00,	 /* Class-D, Boost power down */
	TAS2557_POWER_CTRL1_REG, 0x00,	 /* all power down */
	TAS2557_GPIO1_PIN_REG, 0x00,	/* disable BCLK */
	TAS2557_GPIO2_PIN_REG, 0x00,	/* disable WCLK */
	TAS2557_GPI_PIN_REG, 0x00,	/* disable DIN, MCLK, CCI */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2557_mute_DSP_down_data[] = {
	channel_both, TAS2557_MUTE_REG, 0x03,		 /* mute */
	channel_both, TAS2557_POWER_CTRL1_REG, 0x60,	 /* DSP power down */
	channel_both, TAS2557_UDELAY, 0xFF,		 /* delay */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
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
			if (ret < 0)
				break;
		}
		n++;
	} while (nRegister != 0xFFFFFFFF);
	return ret;
}

static int tas2557_dev_load_chl_data(struct tas2557_priv *pTAS2557, enum channel chl,
	unsigned int *pData)
{
	int ret = 0;
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = pData[n * 2];
		nData = pData[n * 2 + 1];
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

int tas2557_configIRQ(struct tas2557_priv *pTAS2557)
{
	return tas2557_dev_load_data(pTAS2557, p_tas2557_irq_config);
}

int tas2557_SA_SwapChannel(struct tas2557_priv *pTAS2557, bool swap)
{
	int nResult = 0;
	struct TProgram *pProgram;
	unsigned char buf_a[16], buf_b[16];

	if ((pTAS2557->mpFirmware->mnPrograms == 0)
		|| (pTAS2557->mpFirmware->mnConfigurations == 0)) {
		dev_err(pTAS2557->dev, "%s, firmware not loaded\n", __func__);
		goto end;
	}

	pProgram = &(pTAS2557->mpFirmware->mpPrograms[pTAS2557->mnCurrentProgram]);
	if (pProgram->mnAppMode != TAS2557_APP_TUNINGMODE) {
		dev_err(pTAS2557->dev, "%s, not tuning mode\n", __func__);
		goto end;
	}

/*
* for PG2.1
* for left channel, data[16] =
* {0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
*  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
*
* for right channel, data[16] =
* {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
*  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
*
* for (left+right)/2 , data[16] =
* {0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
*  0x20, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00};
*/

	if (pTAS2557->mnLPGID != TAS2557_PG_VERSION_2P1) {
		dev_err(pTAS2557->dev, "%s, currently we only support PG2.1\n", __func__);
		goto end;
	}

	if (swap != pTAS2557->mnChannelSwap) {
		/* get current configuration */
		nResult = pTAS2557->bulk_read(pTAS2557, channel_left, TAS2557_SA_PG2P1_CHL_CTRL_REG, buf_a, 16);
		if (nResult < 0) {
			dev_err(pTAS2557->dev, "%s, left I2C error\n", __func__);
			goto end;
		}

		nResult = pTAS2557->bulk_read(pTAS2557, channel_right, TAS2557_SA_PG2P1_CHL_CTRL_REG, buf_b, 16);
		if (nResult < 0) {
			dev_err(pTAS2557->dev, "%s, right I2C error\n", __func__);
			goto end;
		}

		/* do channel swap */
		pTAS2557->bulk_write(pTAS2557, channel_left, TAS2557_SA_PG2P1_CHL_CTRL_REG, buf_b, 16);
		pTAS2557->bulk_write(pTAS2557, channel_right, TAS2557_SA_PG2P1_CHL_CTRL_REG, buf_a, 16);
		pTAS2557->mnChannelSwap = swap;
	}

end:

	return nResult;
}

int tas2557_SA_ctl_echoRef(struct tas2557_priv *pTAS2557)
{
	int ret = 0;

	if (pTAS2557->mnEchoRef == echoref_left) {
		/* both TAS2557 can be configured as I2S slave mode (I2S standard) */
		/* disable right channel DOUT */
		ret = pTAS2557->write(pTAS2557, channel_right, TAS2557_GPIO3_PIN_REG, 0x00);
		if (ret < 0)
			goto end;
		ret = pTAS2557->write(pTAS2557, channel_left, TAS2557_GPIO3_PIN_REG, 0x10);
	} else if (pTAS2557->mnEchoRef == echoref_right) {
		/* both TAS2557 can be configured as I2S slave mode (I2S standard) */
		/* disable left channel DOUT */
		ret = pTAS2557->write(pTAS2557, channel_left, TAS2557_GPIO3_PIN_REG, 0x00);
		if (ret < 0)
			goto end;
		ret = pTAS2557->write(pTAS2557, channel_right, TAS2557_GPIO3_PIN_REG, 0x10);
	} else if (pTAS2557->mnEchoRef == echoref_both) {
		/* both TAS2557 can be configured as I2S slave mode (DSP mode) */
		/* BCLK = WCLK * Bits * 2 (echo reference + excursion) * 2 (left channel + right channel) */
		/* set TAS2557 to DSP mode and DOUT tri-state */
		ret = pTAS2557->update_bits(pTAS2557, channel_both, TAS2557_ASI1_DAC_FORMAT_REG, 0xe1, 0x21);
		if (ret < 0)
			goto end;
		/* set TAS2557 BCLK and WCLK inverted */
		ret = pTAS2557->write(pTAS2557, channel_both, TAS2557_ASI1_DAC_BCLK_REG, 0x02);
		ret = pTAS2557->write(pTAS2557, channel_both, TAS2557_ASI1_DAC_WCLK_REG, 0x0a);
		/* left channel offset = 1 */
		ret = pTAS2557->write(pTAS2557, channel_left, TAS2557_ASI1_OFFSET1_REG, 0x01);
		/* right channel offset = 1 + Bits*2 */
		ret = pTAS2557->write(pTAS2557, channel_right, TAS2557_ASI1_OFFSET1_REG, 0x01 + pTAS2557->mnI2SBits * 2);
	}

end:

	return ret;
}

int tas2557_set_bit_rate(struct tas2557_priv *pTAS2557, enum channel chn, unsigned int nBitRate)
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

/*
* ctrl : 0 - left channel; 1 - right channel; 2 - mono mix; 3 - stereo
*/
int tas2557_ROMMode_Chl_Ctrl(struct tas2557_priv *pTAS2557, enum channel chl, unsigned char ctrl)
{
	int nResult = 0;
	unsigned char nValue;
	struct TProgram *pProgram;

	if ((pTAS2557->mpFirmware->mnPrograms == 0)
		|| (pTAS2557->mpFirmware->mnConfigurations == 0)) {
		dev_err(pTAS2557->dev, "%s, firmware not loaded\n", __func__);
		goto end;
	}

	pProgram = &(pTAS2557->mpFirmware->mpPrograms[pTAS2557->mnCurrentProgram]);
	if ((pProgram->mnAppMode != TAS2557_APP_ROM1MODE)
		|| (pProgram->mnAppMode != TAS2557_APP_ROM2MODE)) {
		dev_err(pTAS2557->dev, "%s, not ROM mode\n", __func__);
		goto end;
	}

	switch (ctrl) {
	case 2:	/* mono mix case */
		nValue = 0x04;
		nResult = pTAS2557->update_bits(pTAS2557, chl, TAS2557_ASI_CTL1_REG, 0x06, nValue);
	break;
	case 0:	/* left channel */
		nValue = 0x00;
		nResult = pTAS2557->update_bits(pTAS2557, chl, TAS2557_ASI_CTL1_REG, 0x06, nValue);
	break;
	case 1:	/* right channel */
		nValue = 0x02;
		nResult = pTAS2557->update_bits(pTAS2557, chl, TAS2557_ASI_CTL1_REG, 0x06, nValue);
	break;
	case 3:
		nResult = pTAS2557->update_bits(pTAS2557, channel_left, TAS2557_ASI_CTL1_REG, 0x06, 0x00);
		nResult = pTAS2557->update_bits(pTAS2557, channel_right, TAS2557_ASI_CTL1_REG, 0x06, 0x02);
	break;
	}

end:

	return nResult;
}

/*
* die temperature calculation:
* DieTemp = readout / 2^23
*/
int tas2557_get_die_temperature(struct tas2557_priv *pTAS2557, int *pTemperature)
{
	unsigned char nBuf[4];
	int temp;
	int nResult = 0;

	if (pTAS2557->mbPowerUp) {
		nResult = pTAS2557->bulk_read(pTAS2557, channel_left, TAS2557_DIE_TEMP_REG, nBuf, 4);
		if (nResult >= 0) {
			temp = ((int)nBuf[0] << 24) | ((int)nBuf[1] << 16) | ((int)nBuf[2] << 8) | nBuf[3];
			*pTemperature = temp;
			pr_info("%s: temp %d\n", __func__, temp >> 23);
		}
	} else
		dev_err(pTAS2557->dev, "Get Die Temperature when music is playing\n");

	return nResult;
}

int tas2557_load_platdata(struct tas2557_priv *pTAS2557)
{
	int nResult = 0;
	unsigned char nGain;

	if (gpio_is_valid(pTAS2557->mnLeftChlGpioINT)
		|| gpio_is_valid(pTAS2557->mnRightChlGpioINT)) {
		nResult = tas2557_configIRQ(pTAS2557);
		if (nResult < 0)
			goto end;
		pTAS2557->enableIRQ(pTAS2557, false, true);
	}

	nResult = tas2557_set_bit_rate(pTAS2557, channel_both, pTAS2557->mnI2SBits);
	if (nResult < 0)
		goto end;

	nResult = tas2557_SA_ctl_echoRef(pTAS2557);
	if (nResult < 0)
		goto end;

	nResult = tas2557_get_DAC_gain(pTAS2557, channel_left, &nGain);
	if (nResult >= 0)
		pTAS2557->mnDevGain = nGain;

end:

	return nResult;
}

int tas2557_load_default(struct tas2557_priv *pTAS2557)
{
	int nResult = 0;

	nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_default_data);
	if (nResult < 0)
		goto end;

	nResult = tas2557_load_platdata(pTAS2557);
	if (nResult < 0)
		goto end;

	/* enable DOUT tri-state for extra BCLKs */
	nResult = pTAS2557->update_bits(pTAS2557, channel_both, TAS2557_ASI1_DAC_FORMAT_REG, 0x01, 0x01);
end:

	return nResult;
}

static void failsafe(struct tas2557_priv *pTAS2557)
{
	dev_err(pTAS2557->dev, "%s\n", __func__);
	tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_data);
	pTAS2557->mbPowerUp = false;
	pTAS2557->hw_reset(pTAS2557);
	pTAS2557->write(pTAS2557, channel_both, TAS2557_SW_RESET_REG, 0x01);
	udelay(1000);
	pTAS2557->write(pTAS2557, channel_both, TAS2557_SPK_CTRL_REG, 0x04);
	if (pTAS2557->mpFirmware != NULL)
		tas2557_clear_firmware(pTAS2557->mpFirmware);
}

int tas2557_enable(struct tas2557_priv *pTAS2557, bool bEnable)
{
	int nResult = 0, nRetry = 10;
	unsigned char nBuf[4];
	unsigned int nValue;
	struct TProgram *pProgram;

	dev_dbg(pTAS2557->dev, "Enable: %d\n", bEnable);

	if ((pTAS2557->mpFirmware->mnPrograms == 0)
		|| (pTAS2557->mpFirmware->mnConfigurations == 0)) {
		dev_err(pTAS2557->dev, "%s, firmware not loaded\n", __func__);
		goto end;
	}
	/* check safe guard*/
	nResult = pTAS2557->read(pTAS2557, channel_left, TAS2557_SAFE_GUARD_REG, &nValue);
	if (nResult < 0)
		goto end;
	if ((nValue&0xff) != TAS2557_SAFE_GUARD_PATTERN) {
		dev_err(pTAS2557->dev, "ERROR Left channel safe guard failure!\n");
		nResult = -EPIPE;
		goto end;
	}
	nResult = pTAS2557->read(pTAS2557, channel_right, TAS2557_SAFE_GUARD_REG, &nValue);
	if (nResult < 0)
		goto end;
	if ((nValue&0xff) != TAS2557_SAFE_GUARD_PATTERN) {
		dev_err(pTAS2557->dev, "ERROR right channel safe guard failure!\n");
		nResult = -EPIPE;
		goto end;
	}

	pProgram = &(pTAS2557->mpFirmware->mpPrograms[pTAS2557->mnCurrentProgram]);
	if (bEnable) {
		if (!pTAS2557->mbPowerUp) {
			if ((pProgram->mnAppMode == TAS2557_APP_ROM1MODE)
				|| (pProgram->mnAppMode == TAS2557_APP_ROM2MODE)) {
				/* ROM mode power up*/
				nResult = pTAS2557->enableIRQ(pTAS2557, true, true);
				if (nResult < 0)
					goto end;
				nResult = tas2557_dev_load_chl_data(pTAS2557, pTAS2557->mnROMChlDev, p_tas2557_romMode_startup_data);
				if (nResult < 0)
					goto end;
				nResult = tas2557_dev_load_chl_data(pTAS2557, pTAS2557->mnROMChlDev, p_tas2557_unmute_chl_data);
			} else {
				/* check PLL */
pllcheck:
				nResult = pTAS2557->write(pTAS2557, channel_both, TAS2557_POWER_CTRL1_REG, 0xf8);
				if (nResult < 0)
					goto end;
				usleep_range(2000, 2000);
				/* check TAS2557 device 1 */
				memset(nBuf, 0, 4);
				nResult = pTAS2557->bulk_read(pTAS2557, channel_left, TAS2557_XMEM_44_REG, nBuf, 4);
				if (nResult < 0)
					goto end;
				nValue = ((unsigned int)nBuf[0] << 24) | ((unsigned int)nBuf[1] << 16) | ((unsigned int)nBuf[2] << 8) | nBuf[3];
				if (nValue == 0) {
					nResult = pTAS2557->write(pTAS2557, channel_both, TAS2557_POWER_CTRL1_REG, 0x60);
					if (nResult < 0)
						goto end;
					usleep_range(2000, 2000);
					nResult = pTAS2557->write(pTAS2557, channel_both, TAS2557_POWER_CTRL1_REG, 0x00);
					if (nResult < 0)
						goto end;
					usleep_range(2000, 2000);
					nRetry--;
					nResult = -EAGAIN;
					if (nRetry == 0)
						goto end;

					dev_info(pTAS2557->dev, "PLL is absent, check again %d\n", nRetry);
					goto pllcheck;
				}
				/* check TAS2557 device 2 */
				memset(nBuf, 0, 4);
				nResult = pTAS2557->bulk_read(pTAS2557, channel_right, TAS2557_XMEM_44_REG, nBuf, 4);
				nValue = ((unsigned int)nBuf[0] << 24) | ((unsigned int)nBuf[1] << 16) | ((unsigned int)nBuf[2] << 8) | nBuf[3];
				if (nValue == 0) {
					nResult = pTAS2557->write(pTAS2557, channel_both, TAS2557_POWER_CTRL1_REG, 0x60);
					usleep_range(2000, 2000);
					nResult = pTAS2557->write(pTAS2557, channel_both, TAS2557_POWER_CTRL1_REG, 0x00);
					usleep_range(2000, 2000);
					nRetry--;
					nResult = -EAGAIN;
					if (nRetry == 0)
						goto end;

					dev_info(pTAS2557->dev, "PLL is absent, check again %d\n", nRetry);
					goto pllcheck;
				}

				/* power on device */
				dev_dbg(pTAS2557->dev, "Enable: load startup sequence\n");
				nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
				if (nResult < 0)
					goto end;
				dev_dbg(pTAS2557->dev, "Enable: load unmute sequence\n");
				nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
				if (nResult < 0)
					goto end;
				/* turn on IRQ */
				nResult = pTAS2557->enableIRQ(pTAS2557, true, true);
				if (nResult < 0)
					goto end;

				hrtimer_start(&pTAS2557->mtimer,
					ns_to_ktime((u64)LOW_TEMPERATURE_CHECK_PERIOD * NSEC_PER_MSEC), HRTIMER_MODE_REL);
			}

			pTAS2557->mbPowerUp = true;
		}
	} else {
		if (pTAS2557->mbPowerUp) {
			if (hrtimer_active(&pTAS2557->mtimer))
				hrtimer_cancel(&pTAS2557->mtimer);

			dev_dbg(pTAS2557->dev, "Enable: load shutdown sequence\n");
			/* turn off IRQ */
			nResult = pTAS2557->enableIRQ(pTAS2557, false, true);
			if (nResult < 0)
				goto end;

			if ((pProgram->mnAppMode == TAS2557_APP_ROM1MODE)
				|| (pProgram->mnAppMode == TAS2557_APP_ROM2MODE)) {
				nResult = tas2557_dev_load_chl_data(pTAS2557, pTAS2557->mnROMChlDev, p_tas2557_romMode_shutdown_data);
				if (nResult < 0)
					goto end;
			} else {
				nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_data);
				if (nResult < 0)
					goto end;
			}
			pTAS2557->mbPowerUp = false;
		}
	}

	nResult = 0;

end:
	if (nResult < 0) {
		if (nRetry == 0)
			dev_err(pTAS2557->dev, "PLL is absent and enable timeout\n");
		else
			dev_err(pTAS2557->dev, "enable failure %d\n", nResult);
		failsafe(pTAS2557);
	}

	return nResult;
}

int tas2557_set_sampling_rate(struct tas2557_priv *pTAS2557, unsigned int nSamplingRate)
{
	struct TConfiguration *pConfiguration;
	unsigned int nConfiguration;
	int nResult = 0;

	dev_dbg(pTAS2557->dev, "tas2557_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		nResult = -EINVAL;
		goto end;
	}

	pConfiguration = &(pTAS2557->mpFirmware->mpConfigurations[pTAS2557->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_info(pTAS2557->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		nResult = 0;
		goto end;
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
			nResult = tas2557_load_configuration(pTAS2557, nConfiguration, false);
			goto end;
		}
	}

	dev_err(pTAS2557->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

end:

	return nResult;
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

static int fw_parse_block_data(struct tas2557_priv *pTAS2557, struct TFirmware *pFirmware,
	struct TBlock *pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;

	if (pFirmware->mnDriverVersion >= PPC_DRIVER_VERSION) {
		pBlock->mbPChkSumPresent = pData[0];
		pData++;

		pBlock->mnPChkSum = pData[0];
		pData++;

		pBlock->mbYChkSumPresent = pData[0];
		pData++;

		pBlock->mnYChkSum = pData[0];
		pData++;
	} else {
		pBlock->mbPChkSumPresent = 0;
		pBlock->mbYChkSumPresent = 0;
	}

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

static int fw_parse_data(struct tas2557_priv *pTAS2557, struct TFirmware *pFirmware,
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
		n = fw_parse_block_data(pTAS2557, pFirmware,
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

	if (pFirmware->mnPLLs == 0)
		goto end;

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
		n = fw_parse_block_data(pTAS2557, pFirmware, &(pPLL->mBlock), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s:parse block error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}

end:
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

	if (pFirmware->mnPrograms == 0)
		goto end;

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
		n = fw_parse_data(pTAS2557, pFirmware, &(pProgram->mData), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s: parse data error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}

end:

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

	if (pFirmware->mnConfigurations == 0)
		goto end;

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

		n = fw_parse_data(pTAS2557, pFirmware, &(pConfiguration->mData), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s:parse data error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}

end:

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

	if (pFirmware->mnCalibrations == 0)
		goto end;

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

		n = fw_parse_data(pTAS2557, pFirmware, &(pCalibration->mData), pData);

		if (n < 0) {
			dev_err(pTAS2557->dev, "%s: parse data error %d\n", __func__, n);
			return n;
		}

		pData += n;
	}

end:

	return pData - pDataStart;
}

static int fw_parse(struct tas2557_priv *pTAS2557,
	struct TFirmware *pFirmware, unsigned char *pData, unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(pTAS2557, pFirmware, pData, nSize);
	if (nPosition < 0) {
		dev_err(pTAS2557->dev, "Firmware: Wrong Header");
		return -EINVAL;
	}

	if (nPosition >= nSize) {
		dev_err(pTAS2557->dev, "Firmware: Too short");
		return -EINVAL;
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
		dev_info(pTAS2557->dev, "%s:parse calibration size %d\n", __func__, nPosition);

		if (nPosition < 0) {
			return nPosition;
		}
	}

	return 0;
}


static const unsigned char crc8_lookup_table[CRC8_TABLE_SIZE] = {
0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8
};

static int isInPageYRAM(struct tas2557_priv *pTAS2557, struct TYCRC *pCRCData,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char len)
{
	int nResult = 0;

	if (nBook == TAS2557_YRAM_BOOK1) {
		if (nPage == TAS2557_YRAM1_PAGE) {
			if (nReg >= TAS2557_YRAM1_START_REG) {
				pCRCData->mnOffset = nReg;
				pCRCData->mnLen = len;
				nResult = 1;
			} else if ((nReg + len) > TAS2557_YRAM1_START_REG) {
				pCRCData->mnOffset = TAS2557_YRAM1_START_REG;
				pCRCData->mnLen = len - (TAS2557_YRAM1_START_REG - nReg);
				nResult = 1;
			} else
				nResult = 0;
		} else if (nPage == TAS2557_YRAM3_PAGE) {
			if (nReg > TAS2557_YRAM3_END_REG) {
				nResult = 0;
			} else if (nReg >= TAS2557_YRAM3_START_REG) {
				if ((nReg + len) > TAS2557_YRAM3_END_REG) {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen = TAS2557_YRAM3_END_REG - nReg + 1;
					nResult = 1;
				} else {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen = len;
					nResult = 1;
				}
			} else {
				if ((nReg + len) < TAS2557_YRAM3_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset = TAS2557_YRAM3_START_REG;
					pCRCData->mnLen = len - (TAS2557_YRAM3_START_REG - nReg);
					nResult = 1;
				}
			}
		}
	} else if (nBook == TAS2557_YRAM_BOOK2) {
		if (nPage == TAS2557_YRAM5_PAGE) {
			if (nReg > TAS2557_YRAM5_END_REG) {
				nResult = 0;
			} else if (nReg >= TAS2557_YRAM5_START_REG) {
				if ((nReg + len) > TAS2557_YRAM5_END_REG) {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen = TAS2557_YRAM5_END_REG - nReg + 1;
					nResult = 1;
				} else {
					pCRCData->mnOffset = nReg;
					pCRCData->mnLen = len;
					nResult = 1;
				}
			} else {
				if ((nReg + len) < TAS2557_YRAM5_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset = TAS2557_YRAM5_START_REG;
					pCRCData->mnLen = len - (TAS2557_YRAM5_START_REG - nReg);
					nResult = 1;
				}
			}
		}
	} else
		nResult = 0;

	return nResult;
}

static int isInBlockYRAM(struct tas2557_priv *pTAS2557, struct TYCRC *pCRCData,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char len)
{
	int nResult;

	if (nBook == TAS2557_YRAM_BOOK1) {
		if (nPage < TAS2557_YRAM2_START_PAGE)
			nResult = 0;
		else if (nPage <= TAS2557_YRAM2_END_PAGE) {
			if (nReg > TAS2557_YRAM2_END_REG)
				nResult = 0;
			else if (nReg >= TAS2557_YRAM2_START_REG) {
				pCRCData->mnOffset = nReg;
				pCRCData->mnLen = len;
				nResult = 1;
			} else {
				if ((nReg + len) < TAS2557_YRAM2_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset = TAS2557_YRAM2_START_REG;
					pCRCData->mnLen = nReg + len - TAS2557_YRAM2_START_REG;
					nResult = 1;
				}
			}
		} else
			nResult = 0;
	} else if (nBook == TAS2557_YRAM_BOOK2) {
		if (nPage < TAS2557_YRAM4_START_PAGE)
			nResult = 0;
		else if (nPage <= TAS2557_YRAM4_END_PAGE) {
			if (nReg > TAS2557_YRAM2_END_REG)
				nResult = 0;
			else if (nReg >= TAS2557_YRAM2_START_REG) {
				pCRCData->mnOffset = nReg;
				pCRCData->mnLen = len;
				nResult = 1;
			} else {
				if ((nReg + len) < TAS2557_YRAM2_START_REG)
					nResult = 0;
				else {
					pCRCData->mnOffset = TAS2557_YRAM2_START_REG;
					pCRCData->mnLen = nReg + len - TAS2557_YRAM2_START_REG;
					nResult = 1;
				}
			}
		} else
			nResult = 0;
	} else
		nResult = 0;

	return nResult;
}


static int isYRAM(struct tas2557_priv *pTAS2557, struct TYCRC *pCRCData,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char len)
{
	int nResult;

	nResult = isInPageYRAM(pTAS2557, pCRCData, nBook, nPage, nReg, len);

	if (nResult == 0)
		nResult = isInBlockYRAM(pTAS2557, pCRCData, nBook, nPage, nReg, len);

	return nResult;
}

/*
 * crc8 - calculate a crc8 over the given input data.
 *
 * table: crc table used for calculation.
 * pdata: pointer to data buffer.
 * nbytes: number of bytes in data buffer.
 * crc:	previous returned crc8 value.
 */
static u8 ti_crc8(const u8 table[CRC8_TABLE_SIZE], u8 *pdata, size_t nbytes, u8 crc)
{
	/* loop over the buffer data */
	while (nbytes-- > 0)
		crc = table[(crc ^ *pdata++) & 0xff];

	return crc;
}

static int doSingleRegCheckSum(struct tas2557_priv *pTAS2557, enum channel chl,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char nValue)
{
	int nResult = 0;
	struct TYCRC sCRCData;
	unsigned int nData1 = 0, nData2 = 0;
	unsigned char nRegVal;

	if ((nBook == TAS2557_BOOK_ID(TAS2557_SA_COEFF_SWAP_REG))
		&& (nPage == TAS2557_PAGE_ID(TAS2557_SA_COEFF_SWAP_REG))
		&& (nReg >= TAS2557_PAGE_REG(TAS2557_SA_COEFF_SWAP_REG))
		&& (nReg <= (TAS2557_PAGE_REG(TAS2557_SA_COEFF_SWAP_REG) + 4))) {
		/* DSP swap command, pass */
		nResult = 0;
		goto end;
	}

	nResult = isYRAM(pTAS2557, &sCRCData, nBook, nPage, nReg, 1);
	if (nResult == 1) {
		if (chl == channel_broadcast) {
			nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_exit_broadcast_data);
			if (nResult < 0)
				goto end;
		}

		if ((chl & channel_left) || (chl == channel_broadcast)) {
			nResult = pTAS2557->read(pTAS2557, channel_left, TAS2557_REG(nBook, nPage, nReg), &nData1);
			if (nResult < 0)
				goto end;
		}
		if ((chl & channel_right) || (chl == channel_broadcast)) {
			nResult = pTAS2557->read(pTAS2557, channel_right, TAS2557_REG(nBook, nPage, nReg), &nData2);
			if (nResult < 0)
				goto end;
		}

		if (chl == channel_broadcast)
			nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_enter_broadcast_data);

		if ((chl == channel_both) || (chl == channel_broadcast)) {
			if ((nData1 != nData2) || (nData1 != nValue)) {
				dev_err(pTAS2557->dev,
					"error (line %d),B[0x%x]P[0x%x]R[0x%x] W[0x%x], R1[0x%x], R2[0x%x]\n",
					__LINE__, nBook, nPage, nReg, nValue, nData1, nData2);
				nResult = -EAGAIN;
				goto end;
			}
			nRegVal = nData1;
		} else if (chl == channel_left) {
			if (nData1 != nValue) {
				dev_err(pTAS2557->dev,
					"error2 (line %d),B[0x%x]P[0x%x]R[0x%x] W[0x%x], R[0x%x]\n",
					__LINE__, nBook, nPage, nReg, nValue, nData1);
				nResult = -EAGAIN;
				goto end;
			}
			nRegVal = nData1;
		} else if (chl == channel_right) {
			if (nData2 != nValue) {
				dev_err(pTAS2557->dev,
					"error (line %d),B[0x%x]P[0x%x]R[0x%x] W[0x%x], R[0x%x]\n",
					__LINE__, nBook, nPage, nReg, nValue, nData2);
				nResult = -EAGAIN;
				goto end;
			}
			nRegVal = nData2;
		} else {
			nResult = -EINVAL;
				goto end;
		}

		nResult = ti_crc8(crc8_lookup_table, &nRegVal, 1, 0);
	}

end:

	return nResult;
}

static int doMultiRegCheckSum(struct tas2557_priv *pTAS2557, enum channel chl,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned int len)
{
	int nResult = 0, i;
	unsigned char nCRCChkSum = 0;
	unsigned char nBuf1[128];
	unsigned char nBuf2[128];
	struct TYCRC TCRCData;
	unsigned char *pRegVal;

	if ((nReg + len-1) > 127) {
		nResult = -EINVAL;
		dev_err(pTAS2557->dev, "firmware error\n");
		goto end;
	}

	if ((nBook == TAS2557_BOOK_ID(TAS2557_SA_COEFF_SWAP_REG))
		&& (nPage == TAS2557_PAGE_ID(TAS2557_SA_COEFF_SWAP_REG))
		&& (nReg == TAS2557_PAGE_REG(TAS2557_SA_COEFF_SWAP_REG))
		&& (len == 4)) {
		/* DSP swap command, pass */
		nResult = 0;
		goto end;
	}

	nResult = isYRAM(pTAS2557, &TCRCData, nBook, nPage, nReg, len);
	if (nResult == 1) {
		if (len == 1) {
			dev_err(pTAS2557->dev, "firmware error\n");
			nResult = -EINVAL;
			goto end;
		} else {
			if (chl == channel_broadcast) {
				nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_exit_broadcast_data);
				if (nResult < 0)
					goto end;
			}

			if ((chl & channel_left) || (chl == channel_broadcast)) {
				nResult = pTAS2557->bulk_read(pTAS2557, channel_left,
					TAS2557_REG(nBook, nPage, TCRCData.mnOffset), nBuf1, TCRCData.mnLen);
				if (nResult < 0)
					goto end;
			}
			if ((chl & channel_right) || (chl == channel_broadcast)) {
				nResult = pTAS2557->bulk_read(pTAS2557, channel_right,
					TAS2557_REG(nBook, nPage, TCRCData.mnOffset), nBuf2, TCRCData.mnLen);
				if (nResult < 0)
					goto end;
			}

			if (chl == channel_broadcast)
				nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_enter_broadcast_data);

			if ((chl == channel_both) || (chl == channel_broadcast)) {
				if (memcmp(nBuf1, nBuf2, TCRCData.mnLen) != 0) {
					dev_err(pTAS2557->dev,
						"error (line %d), B[0x%x]P[0x%x]R[0x%x] doesn't match\n",
						__LINE__, nBook, nPage, nReg);
					nResult = -EAGAIN;
					goto end;
				}
				pRegVal = nBuf1;
			} else if (chl == channel_left)
				pRegVal = nBuf1;
			else if (chl == channel_right)
				pRegVal = nBuf2;
			else {
				dev_err(pTAS2557->dev, "channel error %d\n", chl);
				nResult = -EINVAL;
				goto end;
			}

			for (i = 0; i < TCRCData.mnLen; i++) {
				if ((nBook == TAS2557_BOOK_ID(TAS2557_SA_COEFF_SWAP_REG))
					&& (nPage == TAS2557_PAGE_ID(TAS2557_SA_COEFF_SWAP_REG))
					&& ((i + TCRCData.mnOffset)
						>= TAS2557_PAGE_REG(TAS2557_SA_COEFF_SWAP_REG))
					&& ((i + TCRCData.mnOffset)
						<= (TAS2557_PAGE_REG(TAS2557_SA_COEFF_SWAP_REG) + 4))) {
					/* DSP swap command, bypass */
					continue;
				} else
					nCRCChkSum += ti_crc8(crc8_lookup_table, &pRegVal[i], 1, 0);
			}

			nResult = nCRCChkSum;
		}
	}

end:

	return nResult;
}

static int tas2557_load_block(struct tas2557_priv *pTAS2557, struct TBlock *pBlock)
{
	int nResult = 0;
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nValue1, nValue2;
	unsigned int nLength;
	unsigned int nSleep;
	enum channel chl;
	unsigned char nCRCChkSum = 0;
	int nRetry = 6;
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
		nResult = -EINVAL;
		goto end;
	}

start:
	if (pBlock->mbPChkSumPresent) {
		if (chl == channel_broadcast)
			nResult = pTAS2557->write(pTAS2557, channel_both, TAS2557_CRC_RESET_REG, 1);
		else
			nResult = pTAS2557->write(pTAS2557, chl, TAS2557_CRC_RESET_REG, 1);
		if (nResult < 0) {
			dev_err(pTAS2557->dev, "I2C err\n");
			goto end;
		}
	}

	if (pBlock->mbYChkSumPresent)
		nCRCChkSum = 0;

	nCommand = 0;

	if (chl == channel_broadcast) {
		nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_enter_broadcast_data);
		if (nResult < 0)
			goto end;
	}

	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;

		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F) {
			nResult = pTAS2557->write(pTAS2557, chl, TAS2557_REG(nBook, nPage, nOffset), nData);

			if (pBlock->mbYChkSumPresent) {
				nResult = doSingleRegCheckSum(pTAS2557, chl, nBook, nPage, nOffset, nData);
				if (nResult < 0)
					goto check;
				nCRCChkSum += (unsigned char)nResult;
			}
		} else if (nOffset == 0x81) {
			nSleep = (nBook << 8) + nPage;
#if 0
			msleep(nSleep);
#else
			nSleep *= 1000;
			usleep_range(nSleep, nSleep);
#endif
		} else if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1) {
				pTAS2557->bulk_write(pTAS2557, chl, TAS2557_REG(nBook, nPage, nOffset), pData + 3, nLength);

				if (pBlock->mbYChkSumPresent) {
					nResult = doMultiRegCheckSum(pTAS2557, chl, nBook, nPage, nOffset, nLength);
					if (nResult < 0)
						goto check;
					nCRCChkSum += (unsigned char)nResult;
				}
			} else {
				pTAS2557->write(pTAS2557, chl, TAS2557_REG(nBook, nPage, nOffset), pData[3]);

				if (pBlock->mbYChkSumPresent) {
					nResult = doSingleRegCheckSum(pTAS2557, chl, nBook, nPage, nOffset, pData[3]);
					if (nResult < 0)
						goto check;
					nCRCChkSum += (unsigned char)nResult;
				}
			}

			nCommand++;

			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}

	if (chl == channel_broadcast)
		nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_exit_broadcast_data);

	if (pBlock->mbPChkSumPresent) {
		if ((chl & channel_left) || (chl == channel_broadcast))
			nResult = pTAS2557->read(pTAS2557, channel_left, TAS2557_CRC_CHECKSUM_REG, &nValue1);
		if ((chl & channel_right) || (chl == channel_broadcast))
			nResult = pTAS2557->read(pTAS2557, channel_right, TAS2557_CRC_CHECKSUM_REG, &nValue2);

		if ((chl == channel_both) || (chl == channel_broadcast)) {
			if ((nValue1 != nValue2) || (nValue1 != pBlock->mnPChkSum)) {
				dev_err(pTAS2557->dev, "Block PChkSum Error: FW = 0x%x, Reg = 0x%x, 0x%x\n",
					pBlock->mnPChkSum, (nValue1&0xff), (nValue2&0xff));
				nResult = -EAGAIN;
				goto check;
			}
		} else if (chl == channel_left) {
			if (nValue1 != pBlock->mnPChkSum) {
				dev_err(pTAS2557->dev, "Block PChkSum Error: FW = 0x%x, Reg = 0x%x\n",
					pBlock->mnPChkSum, (nValue1&0xff));
				nResult = -EAGAIN;
				goto check;
			}
		} else if (chl == channel_right) {
			if (nValue2 != pBlock->mnPChkSum) {
				dev_err(pTAS2557->dev, "Block PChkSum Error: FW = 0x%x, Reg = 0x%x\n",
					pBlock->mnPChkSum, (nValue2&0xff));
				nResult = -EAGAIN;
				goto check;
			}
		}
		nResult = 0;
		dev_dbg(pTAS2557->dev, "Block[0x%x] PChkSum match\n", pBlock->mnType);
	}

	if (pBlock->mbYChkSumPresent) {
		if (nCRCChkSum != pBlock->mnYChkSum) {
			dev_err(pTAS2557->dev, "Block YChkSum Error: FW = 0x%x, YCRC = 0x%x\n",
				pBlock->mnYChkSum, nCRCChkSum);
			nResult = -EAGAIN;
			goto check;
		}
		nResult = 0;
		dev_dbg(pTAS2557->dev, "Block[0x%x] YChkSum match\n", pBlock->mnType);
	}

check:
	if (nResult == -EAGAIN) {
		nRetry--;
		if (nRetry > 0)
			goto start;
	}

end:
	if (nResult < 0) {
		dev_err(pTAS2557->dev, "Block (%d) load error\n",
				pBlock->mnType);
	}
	return nResult;
}

static int tas2557_load_data(struct tas2557_priv *pTAS2557, struct TData *pData, unsigned int nType)
{
	int nResult = 0;
	unsigned int nBlock;
	struct TBlock *pBlock;

	dev_dbg(pTAS2557->dev,
		"TAS2557 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName, pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType) {
			nResult = tas2557_load_block(pTAS2557, pBlock);
			if (nResult < 0)
				break;
		}
	}

	return nResult;
}

static int tas2557_load_configuration(struct tas2557_priv *pTAS2557,
	unsigned int nConfiguration, bool bLoadSame)
{
	int nResult = 0;
	struct TConfiguration *pCurrentConfiguration = NULL;
	struct TConfiguration *pNewConfiguration = NULL;
	struct TCalibration *pCalibration = NULL;
	struct TPLL *pNewPLL = NULL;

	dev_dbg(pTAS2557->dev, "tas2557_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		nResult = 0;
		goto end;
	}

	if (nConfiguration >= pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		nResult = 0;
		goto end;
	}

	if ((nConfiguration == pTAS2557->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_info(pTAS2557->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		nResult = 0;
		goto end;
	}

	pCurrentConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[pTAS2557->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	if (pNewConfiguration->mnProgram != pCurrentConfiguration->mnProgram) {
		dev_err(pTAS2557->dev, "Configuration %d, %s doesn't share the same program as current %d\n",
			nConfiguration, pNewConfiguration->mpName, pCurrentConfiguration->mnProgram);
		nResult = 0;
		goto end;
	}

	if (pNewConfiguration->mnPLL >= pTAS2557->mpFirmware->mnPLLs) {
		dev_err(pTAS2557->dev, "Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		nResult = 0;
		goto end;
	}
	pNewPLL = &(pTAS2557->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);
	if (pTAS2557->mpCalFirmware->mnCalibrations)
		pCalibration = &(pTAS2557->mpCalFirmware->mpCalibrations[pTAS2557->mnCurrentCalibration]);

	if (pTAS2557->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			nResult = pTAS2557->enableIRQ(pTAS2557, false, true);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2557->dev,
				"TAS2557 is powered up, power down DSP before loading new configuration\n");
			nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_shutdown_data);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2557->dev, "TAS2557: load new PLL: %s, block data\n", pNewPLL->mpName);
			nResult = tas2557_load_block(pTAS2557, &(pNewPLL->mBlock));
			if (nResult < 0)
				goto end;
			pTAS2557->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2557->dev, "load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_A);
			if (nResult < 0)
				goto end;
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_B);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2557->dev, "TAS2557: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);
			if (nResult < 0)
				goto end;
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_B);
			if (nResult < 0)
				goto end;
			if (pTAS2557->mpCalFirmware->mnCalibrations) {
				dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
				nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_A);
				if (nResult < 0)
					goto end;
				nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_B);
				if (nResult < 0)
					goto end;
			}
			dev_dbg(pTAS2557->dev, "TAS2557: power up TAS2557\n");
			nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2557->dev, "TAS2557: unmute TAS2557\n");
			nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
			if (nResult < 0)
				goto end;
			nResult = pTAS2557->enableIRQ(pTAS2557, true, false);
		} else {
			dev_dbg(pTAS2557->dev,
				"TAS2557 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);
			if (nResult < 0)
				goto end;
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_B);
			if (nResult < 0)
				goto end;
			if (pTAS2557->mpCalFirmware->mnCalibrations) {
				dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
				nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_A);
				if (nResult < 0)
					goto end;
				nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData),
					TAS2557_BLOCK_CFG_COEFF_DEV_B);
				if (nResult < 0)
					goto end;
			}
		}
		pTAS2557->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2557->dev,
			"TAS2557 was powered down\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2557->dev, "TAS2557: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			nResult = tas2557_load_block(pTAS2557, &(pNewPLL->mBlock));
			if (nResult < 0)
				goto end;
			pTAS2557->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2557->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_A);
			if (nResult < 0)
				goto end;
			nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
				TAS2557_BLOCK_CFG_PRE_DEV_B);
			if (nResult < 0)
				goto end;
		}
		dev_dbg(pTAS2557->dev, "TAS2557: load new configuration: %s, coeff block data\n",
			pNewConfiguration->mpName);
		nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
			TAS2557_BLOCK_CFG_COEFF_DEV_A);
		if (nResult < 0)
			goto end;
		nResult = tas2557_load_data(pTAS2557, &(pNewConfiguration->mData),
			TAS2557_BLOCK_CFG_COEFF_DEV_B);
		if (nResult < 0)
			goto end;
		if (pTAS2557->mpCalFirmware->mnCalibrations) {
			dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
			nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_A);
			if (nResult < 0)
				goto end;
			nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData),
				TAS2557_BLOCK_CFG_COEFF_DEV_B);
		}
		pTAS2557->mbLoadConfigurationPostPowerUp = true;
	}
	pTAS2557->mnCurrentConfiguration = nConfiguration;

end:

	if (nResult < 0)
		failsafe(pTAS2557);

	return nResult;
}

int tas2557_set_config(struct tas2557_priv *pTAS2557, int config)
{
	struct TConfiguration *pConfiguration;
	struct TProgram *pProgram;
	unsigned int nProgram = pTAS2557->mnCurrentProgram;
	unsigned int nConfiguration = config;
	int nResult = 0;

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		nResult = -EINVAL;
		goto end;
	}

	if (nConfiguration >= pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		nResult = -EINVAL;
		goto end;
	}

	pConfiguration = &(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2557->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2557->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		nResult = -EINVAL;
		goto end;
	}

	nResult = tas2557_load_configuration(pTAS2557, nConfiguration, false);

end:

	return nResult;
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

static int tas2557_load_calibration(struct tas2557_priv *pTAS2557,	char *pFileName)
{
	int nResult = 0;
	int nSize = 0;

	struct device_node *offset = NULL;
	unsigned char *p_data = NULL;

	dev_dbg(pTAS2557->dev, "%s:\n", __func__);

	offset = of_find_node_by_path(CALIBRATION_DATA_PATH);

	if (offset != NULL) {
		dev_info(pTAS2557->dev, "TAS2557 cali_data load success from dts\n");
		p_data = (unsigned char *) of_get_property(offset, AUDIO_DATA, &nSize);
	} else {
		dev_err(pTAS2557->dev, "TAS2557 cali_data load fail on dts\n");
		return -EINVAL;
	}

	if (nSize == 0)
		return -EINVAL;

	tas2557_clear_firmware(pTAS2557->mpCalFirmware);
	dev_info(pTAS2557->dev, "TAS2557 calibration file size = %d\n", nSize);
	nResult = fw_parse(pTAS2557, pTAS2557->mpCalFirmware, p_data, nSize);

	if (nResult) {
		dev_err(pTAS2557->dev, "TAS2557 calibration file is corrupt\n");
		tas2557_clear_firmware(pTAS2557->mpCalFirmware);
		return nResult;
	}

	dev_info(pTAS2557->dev, "TAS2557 calibration: %d calibrations\n",
		pTAS2557->mpCalFirmware->mnCalibrations);

	return nResult;
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
		goto end;
	}

	tas2557_load_calibration(pTAS2557, NULL);

	if (pTAS2557->mpFirmware->mpConfigurations) {
		nProgram = pTAS2557->mnCurrentProgram;
		nSampleRate = pTAS2557->mnCurrentSampleRate;
		dev_dbg(pTAS2557->dev, "clear current firmware\n");
		tas2557_clear_firmware(pTAS2557->mpFirmware);
	}

	nResult = fw_parse(pTAS2557, pTAS2557->mpFirmware,
		(unsigned char *) (pFW->data),	pFW->size);
	release_firmware(pFW);
	if (nResult < 0) {
		dev_err(pTAS2557->dev, "firmware is corrupt\n");
		tas2557_clear_firmware(pTAS2557->mpFirmware);
		return;
	}

	if (!pTAS2557->mpFirmware->mnPrograms) {
		dev_err(pTAS2557->dev, "firmware contains no programs\n");
		nResult = -EINVAL;
		goto end;
	}

	if (!pTAS2557->mpFirmware->mnConfigurations) {
		dev_err(pTAS2557->dev, "firmware contains no configurations\n");
		nResult = -EINVAL;
		goto end;
	}

	if (nProgram >= pTAS2557->mpFirmware->mnPrograms) {
		dev_info(pTAS2557->dev,
			"no previous program, set to default\n");
		nProgram = 0;
	}

	pTAS2557->mnCurrentSampleRate = nSampleRate;

	nResult = tas2557_set_program(pTAS2557, nProgram, -1);

end:
	return;
}

int tas2557_set_program(struct tas2557_priv *pTAS2557, unsigned int nProgram, int nConfig)
{
	struct TPLL *pPLL;
	struct TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	bool bFound = false;
	int nResult = 0;

	if ((!pTAS2557->mpFirmware->mpPrograms) ||
		(!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n");
		nResult = 0;
		goto end;
	}

	if (nProgram >= pTAS2557->mpFirmware->mnPrograms) {
		dev_err(pTAS2557->dev, "TAS2557: Program %d doesn't exist\n",
			nProgram);
		nResult = 0;
		goto end;
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
			nResult = 0;
			goto end;
		}
	} else
		nConfiguration = nConfig;

	pTAS2557->mnCurrentProgram = nProgram;
	if (pTAS2557->mbPowerUp) {
		nResult = pTAS2557->enableIRQ(pTAS2557, false, true);
		if (nResult < 0)
			goto end;
		nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_mute_DSP_down_data);
		if (nResult < 0)
			goto end;
	}

	nResult = pTAS2557->write(pTAS2557, channel_both, TAS2557_SW_RESET_REG, 0x01);
	if (nResult < 0)
		goto end;
	usleep_range(1000, 1000);
	nResult = tas2557_load_default(pTAS2557);
	if (nResult < 0)
		goto end;
	dev_info(pTAS2557->dev, "load program %d\n", nProgram);
	nResult = tas2557_load_data(pTAS2557, &(pTAS2557->mpFirmware->mpPrograms[nProgram].mData), TAS2557_BLOCK_PGM_ALL);
	if (nResult < 0)
		goto end;
	nResult = tas2557_load_data(pTAS2557, &(pTAS2557->mpFirmware->mpPrograms[nProgram].mData), TAS2557_BLOCK_PGM_DEV_A);
	if (nResult < 0)
		goto end;
	nResult = tas2557_load_data(pTAS2557, &(pTAS2557->mpFirmware->mpPrograms[nProgram].mData), TAS2557_BLOCK_PGM_DEV_B);
	if (nResult < 0)
		goto end;

	pTAS2557->mnCurrentConfiguration = nConfiguration;
	pConfiguration = &(pTAS2557->mpFirmware->mpConfigurations[nConfiguration]);
	pPLL = &(pTAS2557->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
	dev_dbg(pTAS2557->dev,
		"TAS2557 load PLL: %s block for Configuration %s\n",
		pPLL->mpName, pConfiguration->mpName);
	nResult = tas2557_load_block(pTAS2557, &(pPLL->mBlock));
	if (nResult < 0) {
		dev_err(pTAS2557->dev, "load pll block fail\n");
		goto end;
	}
	pTAS2557->mnCurrentSampleRate = pConfiguration->mnSamplingRate;
	dev_dbg(pTAS2557->dev,
		"load configuration %s conefficient pre block\n",
		pConfiguration->mpName);
	nResult = tas2557_load_data(pTAS2557, &(pConfiguration->mData), TAS2557_BLOCK_CFG_PRE_DEV_A);
	if (nResult < 0) {
		dev_err(pTAS2557->dev, "load TAS2557_BLOCK_CFG_PRE_DEV_A fail\n");
		goto end;
	}
	nResult = tas2557_load_data(pTAS2557, &(pConfiguration->mData), TAS2557_BLOCK_CFG_PRE_DEV_B);
	if (nResult < 0) {
		dev_err(pTAS2557->dev, "load TAS2557_BLOCK_CFG_PRE_DEV_B fail\n");
		goto end;
	}

	nResult = tas2557_load_configuration(pTAS2557, nConfiguration, true);
	if (nResult < 0) {
		dev_err(pTAS2557->dev, "load configuration fail\n");
		goto end;
	}

	if (pTAS2557->mbPowerUp) {
		dev_dbg(pTAS2557->dev, "device powered up, load startup\n");
		nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_startup_data);
		if (nResult < 0)
			goto end;
		dev_dbg(pTAS2557->dev,
			"device powered up, load unmute\n");
		nResult = tas2557_dev_load_data(pTAS2557, p_tas2557_unmute_data);
		if (nResult < 0)
			goto end;
		nResult = pTAS2557->enableIRQ(pTAS2557, true, false);
	}

end:

	if (nResult < 0)
		failsafe(pTAS2557);

	return nResult;
}

int tas2557_set_calibration(struct tas2557_priv *pTAS2557, int nCalibration)
{
	struct TCalibration *pCalibration = NULL;
	int nResult = 0;

	if ((!pTAS2557->mpFirmware->mpPrograms)
		|| (!pTAS2557->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2557->dev, "Firmware not loaded\n\r");
		nResult = 0;
		goto end;
	}

	if (nCalibration == 0x00FF) {
		dev_info(pTAS2557->dev, "load new calibration file %s\n", TAS2557_CAL_NAME);
		nResult = tas2557_load_calibration(pTAS2557, TAS2557_CAL_NAME);
		if (nResult < 0)
			goto end;
		nCalibration = 0;
	}

	if (!pTAS2557->mpCalFirmware ||
		nCalibration >= pTAS2557->mpCalFirmware->mnCalibrations) {
		dev_err(pTAS2557->dev,
			"Calibration %d doesn't exist\n", nCalibration);
		nResult = 0;
		goto end;
	}

	pTAS2557->mnCurrentCalibration = nCalibration;
	pCalibration = &(pTAS2557->mpCalFirmware->mpCalibrations[nCalibration]);
	dev_dbg(pTAS2557->dev, "Enable: load calibration\n");
	nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData), TAS2557_BLOCK_CFG_COEFF_DEV_A);
	if (nResult < 0)
		goto end;
	nResult = tas2557_load_data(pTAS2557, &(pCalibration->mData), TAS2557_BLOCK_CFG_COEFF_DEV_B);

end:
	if (nResult < 0) {
		tas2557_clear_firmware(pTAS2557->mpCalFirmware);
		nResult = tas2557_set_program(pTAS2557, pTAS2557->mnCurrentProgram, pTAS2557->mnCurrentConfiguration);
	}

	return nResult;
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
		ret = -EINVAL;
	} else {
		dev_dbg(pTAS2557->dev, "ti,load=%d\n", pTAS2557->mnLoad);
	}

	if (ret >= 0) {
		pTAS2557->mnResetGPIO = of_get_named_gpio(np, "ti,cdc-reset-gpio", 0);
		if (pTAS2557->mnResetGPIO < 0) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,cdc-reset-gpio", np->full_name,
				pTAS2557->mnResetGPIO);
			ret = -EINVAL;
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
			ret = -EINVAL;
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
			ret = -EINVAL;
		} else {
			dev_dbg(pTAS2557->dev, "ti,irq-gpio-right=%d\n", pTAS2557->mnRightChlGpioINT);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,left-channel", &value);
		if (rc) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,left-channel", np->full_name, rc);
			ret = -EINVAL;
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
			ret = -EINVAL;
		} else {
			pTAS2557->mnRAddr = value;
			dev_dbg(pTAS2557->dev, "ti,right-channel=0x%x\n", pTAS2557->mnRAddr);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,echo-ref", &value);
		if (rc) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,echo-ref", np->full_name, rc);
			ret = -EINVAL;
		} else {
			pTAS2557->mnEchoRef = value;
			dev_dbg(pTAS2557->dev, "ti,echo-ref=%d\n", pTAS2557->mnEchoRef);
		}
	}

	if (ret >= 0) {
		rc = of_property_read_u32(np, "ti,i2s-bits", &value);
		if (rc) {
			dev_err(pTAS2557->dev, "Looking up %s property in node %s failed %d\n",
				"ti,i2s-bits", np->full_name, rc);
			ret = -EINVAL;
		} else {
			pTAS2557->mnI2SBits = value;
			dev_dbg(pTAS2557->dev, "ti,i2s-bits=%d\n", pTAS2557->mnI2SBits);
		}
	}

	return ret;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2557 common functions for Android Linux");
MODULE_LICENSE("GPL v2");
