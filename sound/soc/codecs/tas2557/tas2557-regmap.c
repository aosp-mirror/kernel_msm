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
**     tas2557-regmap.c
**
** Description:
**     I2C driver with regmap for Texas Instruments TAS2557 High Performance 4W Smart Amplifier
**
** =============================================================================
*/

#ifdef CONFIG_TAS2557_REGMAP_STEREO

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
#include <linux/interrupt.h>
#include "tas2557.h"
#include "tas2557-core.h"

#ifdef CONFIG_TAS2557_CODEC_STEREO
#include "tas2557-codec.h"
#endif

#ifdef CONFIG_TAS2557_MISC_STEREO
#include "tas2557-misc.h"
#endif

#define ENABLE_TILOAD
#ifdef ENABLE_TILOAD
#include "tiload.h"
#endif

#define ENABLE_GPIO_RESET
#define LOW_TEMPERATURE_GAIN 6
/*
* tas2557_i2c_write_device : write single byte to device
* platform dependent, need platform specific support
*/
static int tas2557_i2c_write_device(
	struct tas2557_priv *pTAS2557,
	unsigned char addr,
	unsigned char reg,
	unsigned char value)
{
	int ret = 0;

	pTAS2557->client->addr = addr;
	ret = regmap_write(pTAS2557->mpRegmap, reg, value);
	if (ret < 0)
		dev_err(pTAS2557->dev, "%s[0x%x] Error %d\n",
			__func__, addr, ret);
	else
		ret = 1;

	return ret;
}

/*
* tas2557_i2c_bulkwrite_device : write multiple bytes to device
* platform dependent, need platform specific support
*/
static int tas2557_i2c_bulkwrite_device(
	struct tas2557_priv *pTAS2557,
	unsigned char addr,
	unsigned char reg,
	unsigned char *pBuf,
	unsigned int len)
{
	int ret = 0;

	pTAS2557->client->addr = addr;
	ret = regmap_bulk_write(pTAS2557->mpRegmap, reg, pBuf, len);
	if (ret < 0)
		dev_err(pTAS2557->dev, "%s[0x%x] Error %d\n",
			__func__, addr, ret);
	else
		ret = len;

	return ret;
}

/*
* tas2557_i2c_read_device : read single byte from device
* platform dependent, need platform specific support
*/
static int tas2557_i2c_read_device(
	struct tas2557_priv *pTAS2557,
	unsigned char addr,
	unsigned char reg,
	unsigned char *p_value)
{
	int ret = 0;
	unsigned int val = 0;

	pTAS2557->client->addr = addr;
	ret = regmap_read(pTAS2557->mpRegmap, reg, &val);
	if (ret < 0)
		dev_err(pTAS2557->dev, "%s[0x%x] Error %d\n",
			__func__, addr, ret);
	else {
		*p_value = (unsigned char)val;
		ret = 1;
	}

	return ret;
}

/*
* tas2557_i2c_bulkread_device : read multiple bytes from device
* platform dependent, need platform specific support
*/
static int tas2557_i2c_bulkread_device(
	struct tas2557_priv *pTAS2557,
	unsigned char addr,
	unsigned char reg,
	unsigned char *p_value,
	unsigned int len)
{
	int ret = 0;

	pTAS2557->client->addr = addr;
	ret = regmap_bulk_read(pTAS2557->mpRegmap, reg, p_value, len);

	if (ret < 0)
		dev_err(pTAS2557->dev, "%s[0x%x] Error %d\n",
			__func__, addr, ret);
	else
		ret = len;

	return ret;
}

static int tas2557_i2c_update_bits(
	struct tas2557_priv *pTAS2557,
	unsigned char addr,
	unsigned char reg,
	unsigned char mask,
	unsigned char value)
{
	int ret = 0;

	pTAS2557->client->addr = addr;
	ret = regmap_update_bits(pTAS2557->mpRegmap, reg, mask, value);

	if (ret < 0)
		dev_err(pTAS2557->dev, "%s[0x%x] Error %d\n",
			__func__, addr, ret);
	else
		ret = 1;

	return ret;
}

/* tas2557_change_book_page : switch to certain book and page
* platform independent, don't change unless necessary
*/
static int tas2557_change_book_page(
	struct tas2557_priv *pTAS2557,
	enum channel chn,
	unsigned char nBook,
	unsigned char nPage)
{
	int nResult = 0;

	if (chn&channel_left) {
		if (pTAS2557->mnLCurrentBook == nBook) {
			if (pTAS2557->mnLCurrentPage != nPage) {
				nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnLAddr, TAS2557_BOOKCTL_PAGE, nPage);
				if (nResult >= 0)
					pTAS2557->mnLCurrentPage = nPage;
			}
		} else {
			nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnLAddr, TAS2557_BOOKCTL_PAGE, 0);
			if (nResult >= 0) {
				pTAS2557->mnLCurrentPage = 0;
				nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnLAddr, TAS2557_BOOKCTL_REG, nBook);
				pTAS2557->mnLCurrentBook = nBook;
				if (nPage != 0) {
					tas2557_i2c_write_device(pTAS2557, pTAS2557->mnLAddr, TAS2557_BOOKCTL_PAGE, nPage);
					pTAS2557->mnLCurrentPage = nPage;
				}
			}
		}
	}

	if (chn&channel_right) {
		if (pTAS2557->mnRCurrentBook == nBook) {
			if (pTAS2557->mnRCurrentPage != nPage) {
				nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnRAddr, TAS2557_BOOKCTL_PAGE, nPage);
				if (nResult >= 0)
					pTAS2557->mnRCurrentPage = nPage;
			}
		} else {
			nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnRAddr, TAS2557_BOOKCTL_PAGE, 0);
			if (nResult >= 0) {
				pTAS2557->mnRCurrentPage = 0;
				nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnRAddr, TAS2557_BOOKCTL_REG, nBook);
				pTAS2557->mnRCurrentBook = nBook;
				if (nPage != 0) {
					tas2557_i2c_write_device(pTAS2557, pTAS2557->mnRAddr, TAS2557_BOOKCTL_PAGE, nPage);
					pTAS2557->mnRCurrentPage = nPage;
				}
			}
		}
	}

	if (chn == channel_broadcast) {
		nResult = tas2557_i2c_write_device(pTAS2557, TAS2557_BROADCAST_ADDR, TAS2557_BOOKCTL_PAGE, 0);
		if (nResult >= 0) {
			tas2557_i2c_write_device(pTAS2557, TAS2557_BROADCAST_ADDR, TAS2557_BOOKCTL_REG, nBook);
			tas2557_i2c_write_device(pTAS2557, TAS2557_BROADCAST_ADDR, TAS2557_BOOKCTL_PAGE, nPage);
			pTAS2557->mnLCurrentPage = nPage;
			pTAS2557->mnRCurrentPage = nPage;
			pTAS2557->mnLCurrentBook = nBook;
			pTAS2557->mnRCurrentBook = nBook;
		}
	}

	return nResult;
}

/* tas2557_dev_read :
* platform independent, don't change unless necessary
*/
static int tas2557_dev_read(
	struct tas2557_priv *pTAS2557,
	enum channel chn,
	unsigned int nRegister,
	unsigned int *pValue)
{
	int nResult = 0;
	unsigned char Value = 0;

	mutex_lock(&pTAS2557->dev_lock);

	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)) {
			mutex_unlock(&pTAS2557->dev_lock);
			return 0; /* let only reads from TILoad pass. */
		}
		nRegister &= ~0x80000000;

		dev_dbg(pTAS2557->dev, "TiLoad R CH[%d] REG B[%d]P[%d]R[%d]\n",
				chn,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister));
	}

	nResult = tas2557_change_book_page(pTAS2557,
							chn,
							TAS2557_BOOK_ID(nRegister),
							TAS2557_PAGE_ID(nRegister));
	if (nResult >= 0) {
		if (chn == channel_left)
			nResult = tas2557_i2c_read_device(pTAS2557, pTAS2557->mnLAddr, TAS2557_PAGE_REG(nRegister), &Value);
		else if (chn == channel_right)
			nResult = tas2557_i2c_read_device(pTAS2557, pTAS2557->mnRAddr, TAS2557_PAGE_REG(nRegister), &Value);
		else {
			dev_err(pTAS2557->dev, "read chn ERROR %d\n", chn);
			nResult = -1;
		}

		if (nResult >= 0)
			*pValue = Value;
	}

	mutex_unlock(&pTAS2557->dev_lock);
	return nResult;
}

/* tas2557_dev_write :
* platform independent, don't change unless necessary
*/
static int tas2557_dev_write(
	struct tas2557_priv *pTAS2557,
	enum channel chn,
	unsigned int nRegister,
	unsigned int nValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2557->dev_lock);
	if ((nRegister == 0xAFFEAFFE) && (nValue == 0xBABEBABE)) {
		pTAS2557->mbTILoadActive = true;
		mutex_unlock(&pTAS2557->dev_lock);

		dev_dbg(pTAS2557->dev, "TiLoad Active\n");
		return 0;
	}

	if ((nRegister == 0xBABEBABE) && (nValue == 0xAFFEAFFE)) {
		pTAS2557->mbTILoadActive = false;
		mutex_unlock(&pTAS2557->dev_lock);

		dev_dbg(pTAS2557->dev, "TiLoad DeActive\n");
		return 0;
	}

	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)) {
			mutex_unlock(&pTAS2557->dev_lock);
			return 0;/* let only writes from TILoad pass. */
		}
		nRegister &= ~0x80000000;

		dev_dbg(pTAS2557->dev, "TiLoad W CH[%d] REG B[%d]P[%d]R[%d] =0x%x\n",
						chn,
						TAS2557_BOOK_ID(nRegister),
						TAS2557_PAGE_ID(nRegister),
						TAS2557_PAGE_REG(nRegister),
						nValue);
	}

	nResult = tas2557_change_book_page(pTAS2557,
							chn,
							TAS2557_BOOK_ID(nRegister),
							TAS2557_PAGE_ID(nRegister));

	if (nResult >= 0) {
		if (chn & channel_left)
			nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnLAddr, TAS2557_PAGE_REG(nRegister), nValue);

		if (chn & channel_right)
			nResult = tas2557_i2c_write_device(pTAS2557, pTAS2557->mnRAddr, TAS2557_PAGE_REG(nRegister), nValue);

		if (chn == channel_broadcast)
			nResult = tas2557_i2c_write_device(pTAS2557, TAS2557_BROADCAST_ADDR, TAS2557_PAGE_REG(nRegister), nValue);
	}

	mutex_unlock(&pTAS2557->dev_lock);

	return nResult;
}

/* tas2557_dev_bulk_read :
* platform independent, don't change unless necessary
*/
static int tas2557_dev_bulk_read(
	struct tas2557_priv *pTAS2557,
	enum channel chn,
	unsigned int nRegister,
	u8 *pData,
	unsigned int nLength)
{
	int nResult = 0;
	unsigned char reg = 0;
	unsigned char Addr = 0;

	mutex_lock(&pTAS2557->dev_lock);
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)) {
			mutex_unlock(&pTAS2557->dev_lock);
			return 0; /* let only writes from TILoad pass. */
		}

		nRegister &= ~0x80000000;
		dev_dbg(pTAS2557->dev, "TiLoad BR CH[%d] REG B[%d]P[%d]R[%d], count=%d\n",
				chn,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister),
				nLength);
	}

	nResult = tas2557_change_book_page(pTAS2557,
									chn,
									TAS2557_BOOK_ID(nRegister),
									TAS2557_PAGE_ID(nRegister));
	if (nResult >= 0) {
		reg = TAS2557_PAGE_REG(nRegister);
		if (chn == channel_left)
			Addr = pTAS2557->mnLAddr;
		else if (chn == channel_right)
			Addr = pTAS2557->mnRAddr;
		else {
			dev_err(pTAS2557->dev, "bulk read chn ERROR %d\n", chn);
			nResult = -1;
		}

		if (nResult >= 0)
			nResult = tas2557_i2c_bulkread_device(
				pTAS2557, Addr, reg, pData, nLength);
	}

	mutex_unlock(&pTAS2557->dev_lock);

	return nResult;
}

/* tas2557_dev_bulk_write :
* platform independent, don't change unless necessary
*/
static int tas2557_dev_bulk_write(
	struct tas2557_priv *pTAS2557,
	enum channel chn,
	unsigned int nRegister,
	u8 *pData,
	unsigned int nLength)
{
	int nResult = 0;
	unsigned char reg = 0;

	mutex_lock(&pTAS2557->dev_lock);
	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)) {
			mutex_unlock(&pTAS2557->dev_lock);
			return 0; /* let only writes from TILoad pass. */
		}

		nRegister &= ~0x80000000;

		dev_dbg(pTAS2557->dev, "TiLoad BW CH[%d] REG B[%d]P[%d]R[%d], count=%d\n",
				chn,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister),
				nLength);
	}

	nResult = tas2557_change_book_page(
		pTAS2557,
		chn,
		TAS2557_BOOK_ID(nRegister),
		TAS2557_PAGE_ID(nRegister));

	if (nResult >= 0) {
		reg = TAS2557_PAGE_REG(nRegister);
		if (chn & channel_left) {
			nResult = tas2557_i2c_bulkwrite_device(
				pTAS2557,
				pTAS2557->mnLAddr,
				reg, pData, nLength);
			if (nResult < 0)
				dev_err(pTAS2557->dev, "bulk write error %d\n", nResult);
		}

		if (chn & channel_right) {
			nResult = tas2557_i2c_bulkwrite_device(
				pTAS2557,
				pTAS2557->mnRAddr,
				reg, pData, nLength);
			if (nResult < 0)
				dev_err(pTAS2557->dev, "bulk write error %d\n", nResult);
		}

		if (chn == channel_broadcast)
			nResult = tas2557_i2c_bulkwrite_device(pTAS2557, TAS2557_BROADCAST_ADDR, reg, pData, nLength);
	}
	mutex_unlock(&pTAS2557->dev_lock);

	return nResult;
}

/* tas2557_dev_update_bits :
* platform independent, don't change unless necessary
*/
static int tas2557_dev_update_bits(
	struct tas2557_priv *pTAS2557,
	enum channel chn,
	unsigned int nRegister,
	unsigned int nMask,
	unsigned int nValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2557->dev_lock);

	if (pTAS2557->mbTILoadActive) {
		if (!(nRegister & 0x80000000)) {
			mutex_unlock(&pTAS2557->dev_lock);
			return 0; /* let only writes from TILoad pass. */
		}

		nRegister &= ~0x80000000;
		dev_dbg(pTAS2557->dev, "TiLoad SB CH[%d] REG B[%d]P[%d]R[%d], mask=0x%x, value=0x%x\n",
				chn,
				TAS2557_BOOK_ID(nRegister),
				TAS2557_PAGE_ID(nRegister),
				TAS2557_PAGE_REG(nRegister),
				nMask, nValue);
	}

	nResult = tas2557_change_book_page(
		pTAS2557,
		chn,
		TAS2557_BOOK_ID(nRegister),
		TAS2557_PAGE_ID(nRegister));

	if (nResult >= 0) {
		if (chn&channel_left)
			tas2557_i2c_update_bits(pTAS2557, pTAS2557->mnLAddr, TAS2557_PAGE_REG(nRegister), nMask, nValue);

		if (chn&channel_right)
			tas2557_i2c_update_bits(pTAS2557, pTAS2557->mnRAddr, TAS2557_PAGE_REG(nRegister), nMask, nValue);
	}

	mutex_unlock(&pTAS2557->dev_lock);
	return nResult;
}

int tas2557_enableIRQ(struct tas2557_priv *pTAS2557, bool enable, bool clear)
{
	unsigned int nValue;
	int nResult = 0;

	if (enable) {
		if (clear) {
			nResult = pTAS2557->read(pTAS2557, channel_left, TAS2557_FLAGS_1, &nValue);
			if (nResult < 0)
				goto end;
			pTAS2557->read(pTAS2557, channel_left, TAS2557_FLAGS_2, &nValue);
			nResult = pTAS2557->read(pTAS2557, channel_right, TAS2557_FLAGS_1, &nValue);
			if (nResult < 0)
				goto end;
			pTAS2557->read(pTAS2557, channel_right, TAS2557_FLAGS_2, &nValue);
		}

		if (!pTAS2557->mbIRQEnable) {
			if (pTAS2557->mnLeftChlIRQ != 0)
				enable_irq(pTAS2557->mnLeftChlIRQ);
			if ((pTAS2557->mnRightChlIRQ != 0)
				&& (pTAS2557->mnRightChlIRQ != pTAS2557->mnLeftChlIRQ))
				enable_irq(pTAS2557->mnRightChlIRQ);
			pTAS2557->mbIRQEnable = true;
		}
	} else {
		if (pTAS2557->mbIRQEnable) {
			if (pTAS2557->mnLeftChlIRQ != 0)
				disable_irq_nosync(pTAS2557->mnLeftChlIRQ);
			if ((pTAS2557->mnRightChlIRQ != 0)
				&& (pTAS2557->mnRightChlIRQ != pTAS2557->mnLeftChlIRQ))
				disable_irq_nosync(pTAS2557->mnRightChlIRQ);
			pTAS2557->mbIRQEnable = false;
		}

		if (clear) {
			nResult = pTAS2557->read(pTAS2557, channel_left, TAS2557_FLAGS_1, &nValue);
			if (nResult < 0)
				goto end;
			pTAS2557->read(pTAS2557, channel_left, TAS2557_FLAGS_2, &nValue);
			nResult = pTAS2557->read(pTAS2557, channel_right, TAS2557_FLAGS_1, &nValue);
			if (nResult < 0)
				goto end;
			pTAS2557->read(pTAS2557, channel_right, TAS2557_FLAGS_2, &nValue);
		}
	}

end:

	return nResult;
}

static void tas2557_hw_reset(struct tas2557_priv *pTAS2557)
{
#ifdef ENABLE_GPIO_RESET
	if (gpio_is_valid(pTAS2557->mnResetGPIO)) {
		devm_gpio_request_one(pTAS2557->dev, pTAS2557->mnResetGPIO,
			GPIOF_OUT_INIT_LOW, "TAS2557_RST");
		msleep(10);
		gpio_set_value_cansleep(pTAS2557->mnResetGPIO, 1);
		udelay(1000);
	}
#endif
	pTAS2557->mnLCurrentBook = -1;
	pTAS2557->mnLCurrentPage = -1;
	pTAS2557->mnRCurrentBook = -1;
	pTAS2557->mnRCurrentPage = -1;
}

static void irq_work_routine(struct work_struct *work)
{
	int nResult = 0;
	unsigned int nDevLInt1Status = 0, nDevLInt2Status = 0;
	unsigned int nDevRInt1Status = 0, nDevRInt2Status = 0;
	struct tas2557_priv *pTAS2557 =
		container_of(work, struct tas2557_priv, irq_work.work);

	if (!pTAS2557->mbPowerUp)
		return;

	nResult = tas2557_dev_read(pTAS2557, channel_left, TAS2557_FLAGS_1, &nDevLInt1Status);
	if (nResult < 0)
		dev_err(pTAS2557->dev, "left channel I2C doesn't work\n");
	else
		nResult = tas2557_dev_read(pTAS2557, channel_left, TAS2557_FLAGS_2, &nDevLInt2Status);

	if (nResult < 0)
		goto program;

	nResult = tas2557_dev_read(pTAS2557, channel_right, TAS2557_FLAGS_1, &nDevRInt1Status);
	if (nResult < 0)
		dev_err(pTAS2557->dev, "right channel I2C doesn't work\n");
	else
		nResult = tas2557_dev_read(pTAS2557, channel_right, TAS2557_FLAGS_2, &nDevRInt2Status);

	if (nResult < 0)
		goto program;

	if (((nDevLInt1Status & 0xdc) != 0) || ((nDevLInt2Status & 0x0c) != 0)
		|| ((nDevRInt1Status & 0xdc) != 0) || ((nDevRInt2Status & 0x0c) != 0)) {
		/* in case of INT_OC, INT_UV, INT_OT, INT_BO, INT_CL, INT_CLK1, INT_CLK2 */
		dev_err(pTAS2557->dev, "critical error L: 0x%x, 0x%x; R: 0x%x, 0x%x\n",
			nDevLInt1Status, nDevLInt2Status, nDevRInt1Status, nDevRInt2Status);
		goto program;
	} else {
		tas2557_dev_read(pTAS2557, channel_left, TAS2557_POWER_UP_FLAG_REG, &nDevLInt1Status);
		if ((nDevLInt1Status & 0x40) == 0) {
			/* Class-D doesn't power on */
			tas2557_dev_read(pTAS2557, channel_left, TAS2557_POWER_CTRL2_REG, &nDevLInt2Status);
			if (nDevLInt2Status & 0x80) {
				/* failed to power on the Class-D */
				goto program;
			}
		}
		tas2557_dev_read(pTAS2557, channel_right, TAS2557_POWER_UP_FLAG_REG, &nDevRInt1Status);
		if ((nDevRInt1Status & 0x40) == 0) {
			/* Class-D doesn't power on */
			tas2557_dev_read(pTAS2557, channel_right, TAS2557_POWER_CTRL2_REG, &nDevRInt2Status);
			if (nDevRInt2Status & 0x80) {
				/* failed to power on the Class-D */
				goto program;
			}
		}
		dev_dbg(pTAS2557->dev, "%s, L: 0x%x, 0x%x; R: 0x%x, 0x%x\n",
			__func__, nDevLInt1Status, nDevLInt2Status, nDevRInt1Status, nDevRInt2Status);
	}
	return;

program:
	/* FIXME workaround for IRQ error of bit-clk not ready */
	return;
	/* hardware reset and reload */
	tas2557_hw_reset(pTAS2557);
	tas2557_set_program(pTAS2557, pTAS2557->mnCurrentProgram, pTAS2557->mnCurrentConfiguration);
}

static irqreturn_t tas2557_irq_handler(int irq, void *dev_id)
{
	struct tas2557_priv *pTAS2557 = (struct tas2557_priv *)dev_id;

	tas2557_enableIRQ(pTAS2557, false, false);
	/* get IRQ status after 100 ms */
	schedule_delayed_work(&pTAS2557->irq_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static enum hrtimer_restart temperature_timer_func(struct hrtimer *timer)
{
	struct tas2557_priv *pTAS2557 = container_of(timer, struct tas2557_priv, mtimer);

	if (pTAS2557->mbPowerUp)
		schedule_work(&pTAS2557->mtimerwork);
	return HRTIMER_NORESTART;
}

static void timer_work_routine(struct work_struct *work)
{
	struct tas2557_priv *pTAS2557 = container_of(work, struct tas2557_priv, mtimerwork);
	int nResult, nTemp;

	if (!pTAS2557->mbPowerUp)
		return;

	nResult = tas2557_get_die_temperature(pTAS2557, &nTemp);
	if (nResult >= 0) {
		dev_dbg(pTAS2557->dev, "Die=0x%x\n", nTemp);

		if ((nTemp & 0x80000000) != 0) {
			/* if Die temperature is below ZERO */
			if (pTAS2557->mnDevCurrentGain != LOW_TEMPERATURE_GAIN) {
				tas2557_set_DAC_gain(pTAS2557, channel_both, LOW_TEMPERATURE_GAIN);
				pTAS2557->mnDevCurrentGain = LOW_TEMPERATURE_GAIN;
				dev_info(pTAS2557->dev, "LOW Temp: set gain to %d\n", LOW_TEMPERATURE_GAIN);
			}
		} else {
			/* if Die temperature is above ZERO */
			if (pTAS2557->mnDevCurrentGain != pTAS2557->mnDevGain) {
				tas2557_set_DAC_gain(pTAS2557, channel_both, pTAS2557->mnDevGain);
				pTAS2557->mnDevCurrentGain = pTAS2557->mnDevGain;
				dev_info(pTAS2557->dev, "LOW Temp: set gain to original\n");
			}
		}

		if (pTAS2557->mbPowerUp)
			hrtimer_start(&pTAS2557->mtimer,
				ns_to_ktime((u64)LOW_TEMPERATURE_CHECK_PERIOD * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}
}

static bool tas2557_volatile(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static bool tas2557_writeable(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static const struct regmap_config tas2557_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2557_writeable,
	.volatile_reg = tas2557_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 128,
};

/* tas2557_i2c_probe :
* platform dependent
* should implement hardware reset functionality
*/
static int tas2557_i2c_probe(struct i2c_client *pClient,
	const struct i2c_device_id *pID)
{
	struct tas2557_priv *pTAS2557;
	int nResult = 0;
	unsigned int nValue = 0;
	char *fw_name = TAS2557_FW_NAME;

	dev_info(&pClient->dev, "%s enter\n", __func__);

	pTAS2557 = devm_kzalloc(&pClient->dev, sizeof(struct tas2557_priv), GFP_KERNEL);
	if (!pTAS2557) {
		nResult = -ENOMEM;
		goto err;
	}

	pTAS2557->client = pClient;
	pTAS2557->dev = &pClient->dev;
	i2c_set_clientdata(pClient, pTAS2557);
	dev_set_drvdata(&pClient->dev, pTAS2557);

	pTAS2557->mpRegmap = devm_regmap_init_i2c(pClient, &tas2557_i2c_regmap);
	if (IS_ERR(pTAS2557->mpRegmap)) {
		nResult = PTR_ERR(pTAS2557->mpRegmap);
		dev_err(&pClient->dev, "Failed to allocate register map: %d\n",
			nResult);
		goto err;
	}

	if (pClient->dev.of_node)
		tas2557_parse_dt(&pClient->dev, pTAS2557);

	tas2557_hw_reset(pTAS2557);

	pTAS2557->read = tas2557_dev_read;
	pTAS2557->write = tas2557_dev_write;
	pTAS2557->bulk_read = tas2557_dev_bulk_read;
	pTAS2557->bulk_write = tas2557_dev_bulk_write;
	pTAS2557->update_bits = tas2557_dev_update_bits;
	pTAS2557->enableIRQ = tas2557_enableIRQ;
	pTAS2557->set_config = tas2557_set_config;
	pTAS2557->set_calibration = tas2557_set_calibration;
	pTAS2557->hw_reset = tas2557_hw_reset;

	mutex_init(&pTAS2557->dev_lock);

	/* Reset the chip */
	nResult = tas2557_dev_write(pTAS2557, channel_both, TAS2557_SW_RESET_REG, 1);
	if (nResult < 0) {
		dev_err(&pClient->dev, "I2c fail, %d\n", nResult);
		goto err;
	}

	msleep(1);
	tas2557_dev_read(pTAS2557, channel_left, TAS2557_REV_PGID_REG, &nValue);
	pTAS2557->mnLPGID = nValue;
	tas2557_dev_read(pTAS2557, channel_right, TAS2557_REV_PGID_REG, &nValue);
	pTAS2557->mnRPGID = nValue;
	if (pTAS2557->mnLPGID != pTAS2557->mnRPGID) {
		dev_err(pTAS2557->dev, "HardWare Critical: L-PGID=0x%x, R-PGID=0x%x, please use same version\n",
			pTAS2557->mnLPGID, pTAS2557->mnRPGID);
		nResult = -ENOTSUPP;
		goto err;
	} else
		dev_info(pTAS2557->dev, "PGID = 0x%x\n", pTAS2557->mnLPGID);

	if (gpio_is_valid(pTAS2557->mnLeftChlGpioINT)) {
		nResult = gpio_request(pTAS2557->mnLeftChlGpioINT, "TAS2557-LeftCHL-IRQ");
		if (nResult < 0) {
			dev_err(pTAS2557->dev,
				"%s: GPIO %d request INT error\n",
				__func__, pTAS2557->mnLeftChlGpioINT);
			goto err;
		}

		gpio_direction_input(pTAS2557->mnLeftChlGpioINT);
		pTAS2557->mnLeftChlIRQ = gpio_to_irq(pTAS2557->mnLeftChlGpioINT);
		dev_dbg(pTAS2557->dev, "irq = %d\n", pTAS2557->mnLeftChlIRQ);
		nResult = request_threaded_irq(pTAS2557->mnLeftChlIRQ, tas2557_irq_handler,
				NULL, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				pClient->name, pTAS2557);
		if (nResult < 0) {
			dev_err(pTAS2557->dev,
				"request_irq failed, %d\n", nResult);
			goto err;
		}
		disable_irq_nosync(pTAS2557->mnLeftChlIRQ);
	}

	if (gpio_is_valid(pTAS2557->mnRightChlGpioINT)) {
		if (pTAS2557->mnLeftChlGpioINT != pTAS2557->mnRightChlGpioINT) {
			nResult = gpio_request(pTAS2557->mnRightChlGpioINT, "TAS2557-RightCHL-IRQ");
			if (nResult < 0) {
				dev_err(pTAS2557->dev,
					"%s: GPIO %d request INT error\n",
					__func__, pTAS2557->mnRightChlGpioINT);
				goto err;
			}

			gpio_direction_input(pTAS2557->mnRightChlGpioINT);
			pTAS2557->mnRightChlIRQ = gpio_to_irq(pTAS2557->mnRightChlGpioINT);
			dev_dbg(pTAS2557->dev, "irq = %d\n", pTAS2557->mnRightChlIRQ);
			nResult = request_threaded_irq(pTAS2557->mnRightChlIRQ, tas2557_irq_handler,
					NULL, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					pClient->name, pTAS2557);
			if (nResult < 0) {
				dev_err(pTAS2557->dev,
					"request_irq failed, %d\n", nResult);
				goto err;
			}
			disable_irq_nosync(pTAS2557->mnRightChlIRQ);
		}
	}

	if (gpio_is_valid(pTAS2557->mnLeftChlGpioINT)
		|| gpio_is_valid(pTAS2557->mnRightChlGpioINT)) {
		INIT_DELAYED_WORK(&pTAS2557->irq_work, irq_work_routine);
	}

	pTAS2557->mpFirmware = devm_kzalloc(&pClient->dev, sizeof(struct TFirmware), GFP_KERNEL);
	if (!pTAS2557->mpFirmware) {
		nResult = -ENOMEM;
		goto err;
	}

	pTAS2557->mpCalFirmware = devm_kzalloc(&pClient->dev, sizeof(struct TFirmware), GFP_KERNEL);
	if (!pTAS2557->mpCalFirmware) {
		nResult = -ENOMEM;
		goto err;
	}

#ifdef CONFIG_TAS2557_CODEC_STEREO
	tas2557_register_codec(pTAS2557);
#endif

#ifdef CONFIG_TAS2557_MISC_STEREO
	mutex_init(&pTAS2557->file_lock);
	tas2557_register_misc(pTAS2557);
#endif

#ifdef ENABLE_TILOAD
	tiload_driver_init(pTAS2557);
#endif

	hrtimer_init(&pTAS2557->mtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pTAS2557->mtimer.function = temperature_timer_func;
	INIT_WORK(&pTAS2557->mtimerwork, timer_work_routine);

	switch (pTAS2557->mnLPGID) {
	case TAS2557_PG_VERSION_2P1:
		fw_name = TAS2557_FW_PG21_NAME;
		break;
	}

	nResult = request_firmware_nowait(THIS_MODULE, 1, fw_name,
		pTAS2557->dev, GFP_KERNEL, pTAS2557, tas2557_fw_ready);

err:

	return nResult;
}

static int tas2557_i2c_remove(struct i2c_client *pClient)
{
	struct tas2557_priv *pTAS2557 = i2c_get_clientdata(pClient);

	dev_info(pTAS2557->dev, "%s\n", __func__);

#ifdef CONFIG_TAS2557_CODEC_STEREO
	tas2557_deregister_codec(pTAS2557);
#endif

#ifdef CONFIG_TAS2557_MISC_STEREO
	tas2557_deregister_misc(pTAS2557);
	mutex_destroy(&pTAS2557->file_lock);
#endif

	mutex_destroy(&pTAS2557->dev_lock);
	return 0;
}

static const struct i2c_device_id tas2557_i2c_id[] = {
	{"tas2557s", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tas2557_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2557_of_match[] = {
	{.compatible = "ti,tas2557s"},
	{},
};

MODULE_DEVICE_TABLE(of, tas2557_of_match);
#endif

static struct i2c_driver tas2557_i2c_driver = {
	.driver = {
			.name = "tas2557s",
			.owner = THIS_MODULE,
#if defined(CONFIG_OF)
			.of_match_table = of_match_ptr(tas2557_of_match),
#endif
		},
	.probe = tas2557_i2c_probe,
	.remove = tas2557_i2c_remove,
	.id_table = tas2557_i2c_id,
};

module_i2c_driver(tas2557_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2557 Stereo I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");

#endif
