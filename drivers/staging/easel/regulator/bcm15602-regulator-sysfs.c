/*
 * Copyright (C) 2016 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
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

#define DEBUG

#include <linux/device.h>
#include <linux/module.h>

#include "bcm15602-regulator.h"

/* used for reg_read attribute */
static u8 _reg_read_addr;

/* asr_curr is in uA */
static int bcm15602_get_asr_curr(struct bcm15602_chip *ddata, int *asr_curr)
{
	u16 mstr_curr_chan_data, slv_curr_chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_ASR_MSTR_CURR,
			       &mstr_curr_chan_data);
	bcm15602_read_adc_chan(ddata, BCM15602_ADC_ASR_SLV_CURR,
			       &slv_curr_chan_data);

	*asr_curr = (mstr_curr_chan_data * BCM15602_ADC_SCALE_ASR_CURR) +
		(slv_curr_chan_data * BCM15602_ADC_SCALE_ASR_CURR);

	return 0;
}

/* sdsr_curr is in uA */
static int bcm15602_get_sdsr_curr(struct bcm15602_chip *ddata, int *sdsr_curr)
{
	u16 chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_SDSR_CURR, &chan_data);

	*sdsr_curr = chan_data * BCM15602_ADC_SCALE_SDSR_CURR;

	return 0;
}

/* sdldo_curr is in uA */
static int bcm15602_get_sdldo_curr(struct bcm15602_chip *ddata, int *sdldo_curr)
{
	u16 chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_SDLDO_CURR, &chan_data);

	*sdldo_curr = chan_data * BCM15602_ADC_SCALE_SDLDO_CURR;

	return 0;
}

/* ioldo_curr is in uA */
static int bcm15602_get_ioldo_curr(struct bcm15602_chip *ddata, int *ioldo_curr)
{
	u16 chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_IOLDO_CURR, &chan_data);

	*ioldo_curr = chan_data * BCM15602_ADC_SCALE_IOLDO_CURR;

	return 0;
}

static ssize_t asr_curr_show(struct device *dev,
			     struct device_attribute *mattr,
			     char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int asr_curr;

	bcm15602_get_asr_curr(ddata, &asr_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", asr_curr);
}
static DEVICE_ATTR_RO(asr_curr);

static ssize_t sdsr_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int sdsr_curr;

	bcm15602_get_sdsr_curr(ddata, &sdsr_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", sdsr_curr);
}
static DEVICE_ATTR_RO(sdsr_curr);

static ssize_t sdldo_curr_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int sdldo_curr;

	bcm15602_get_sdldo_curr(ddata, &sdldo_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", sdldo_curr);
}
static DEVICE_ATTR_RO(sdldo_curr);

static ssize_t ioldo_curr_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int ioldo_curr;

	bcm15602_get_ioldo_curr(ddata, &ioldo_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", ioldo_curr);
}
static DEVICE_ATTR_RO(ioldo_curr);

static ssize_t vbat_show(struct device *dev,
			 struct device_attribute *mattr,
			 char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u16 chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_VBAT, &chan_data);

	return snprintf(data, PAGE_SIZE, "%d\n",
			chan_data * BCM15602_ADC_SCALE_VBAT);
}
static DEVICE_ATTR_RO(vbat);

static ssize_t temperature_show(struct device *dev,
				struct device_attribute *mattr,
				char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u16 chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_PTAT, &chan_data);

	return snprintf(data, PAGE_SIZE, "%d\n", PTAT_CODE_TO_TEMP(chan_data));
}
static DEVICE_ATTR_RO(temperature);

/* shows power in mW */
static ssize_t total_power_show(struct device *dev,
				struct device_attribute *mattr,
				char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int asr_curr, sdsr_curr, sdldo_curr, ioldo_curr;
	long int total_power;

	bcm15602_get_asr_curr(ddata, &asr_curr);
	bcm15602_get_sdsr_curr(ddata, &sdsr_curr);
	bcm15602_get_sdldo_curr(ddata, &sdldo_curr);
	bcm15602_get_ioldo_curr(ddata, &ioldo_curr);

	total_power = (((long int)asr_curr) * 900 / 1000) +
		(((long int)sdsr_curr) * 1100 / 1000) +
		(((long int)sdldo_curr) * 1800 / 1000) +
		(((long int)ioldo_curr) * 1800 / 1000);

	return snprintf(data, PAGE_SIZE, "%ld\n", total_power);
}
static DEVICE_ATTR_RO(total_power);

static ssize_t hk_status_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u16 status;

	status = ddata->hk_status;
	ddata->hk_status &= ~status;

	return snprintf(data, PAGE_SIZE, "0x%04x\n", status);
}
static DEVICE_ATTR_RO(hk_status);

static ssize_t asr_vsel_show(struct device *dev,
			     struct device_attribute *mattr,
			     char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;
	int voctrl;

	bcm15602_read_byte(ddata, BCM15602_REG_BUCK_ASR_VOCTRL, &reg_data);
	voctrl = reg_data & 0x7F;

	return snprintf(data, PAGE_SIZE, "%d\n", 565 + voctrl * 5);
}

static ssize_t asr_vsel_store(struct device *dev,
			      struct device_attribute *mattr,
			      const char *data, size_t count)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int vsel, voctrl;

	if (kstrtoint(data, 0, &vsel)) {
		dev_err(dev, "%s: Not a valid int\n", __func__);
		return -EINVAL;
	}

	if (vsel == 0) {
		bcm15602_update_bits(ddata, BCM15602_REG_BUCK_ASR_CTRL0, 0x1,
				     0x0);
		return count;
	} else if ((vsel < 565) || (vsel > 1200)) {
		dev_err(dev, "%s: Not a valid voltage level, must be 0 or 565-1200\n",
			__func__);
		return -EINVAL;
	}

	voctrl = ((vsel - 565) + 4) / 5;
	bcm15602_write_byte(ddata, BCM15602_REG_BUCK_ASR_VOCTRL, voctrl & 0x7F);
	bcm15602_update_bits(ddata, BCM15602_REG_BUCK_ASR_CTRL0, 0x1, 0x1);

	return count;
}
static DEVICE_ATTR_RW(asr_vsel);

static ssize_t ioldo_vsel_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;
	int voctrl, vstep, vbase; /* in mV */

	bcm15602_read_byte(ddata, BCM15602_REG_LDO_IOLDO_VOCTRL,
			   &reg_data);
	vbase = (reg_data & 0x80) ? 1725 : 1380;
	vstep = (reg_data & 0x80) ? 25 : 10;
	voctrl = reg_data & 0x3F;
	dev_info(dev, "%s: vbase=%d, vstep=%d, voctrl=%d",
		 __func__, vbase, vstep, voctrl);

	return snprintf(data, PAGE_SIZE, "%d\n", vbase + voctrl * vstep);
}

static ssize_t ioldo_vsel_store(struct device *dev,
				struct device_attribute *mattr,
				const char *data, size_t count)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;
	int vsel, voctrl, vstep, vbase; /* in mV */

	if (kstrtoint(data, 0, &vsel)) {
		dev_err(dev, "%s: Not a valid int\n", __func__);
		return -EINVAL;
	}

	if (vsel == 0) {
		bcm15602_update_bits(ddata, BCM15602_REG_LDO_IOLDO_ENCTRL, 0x1,
				     0x0);
		return count;
	} else if ((vsel < 1380) || (vsel > 1800)) {
		dev_err(dev, "%s: Not a valid voltage level, must be 0 or 1380-1800\n",
			__func__);
		return -EINVAL;
	}

	bcm15602_read_byte(ddata, BCM15602_REG_LDO_IOLDO_VOCTRL,
			   &reg_data);
	vbase = (reg_data & 0x80) ? 1725 : 1380;
	vstep = (reg_data & 0x80) ? 25 : 10;
	voctrl = reg_data & 0x3F;

	voctrl = ((vsel - vbase) + vstep - 1) / vstep;
	bcm15602_update_bits(ddata, BCM15602_REG_LDO_IOLDO_VOCTRL, 0x3F,
			     voctrl);
	bcm15602_update_bits(ddata, BCM15602_REG_LDO_IOLDO_ENCTRL, 0x1, 0x1);

	return count;
}
static DEVICE_ATTR_RW(ioldo_vsel);

static ssize_t sdldo_vsel_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;
	int voctrl, vstep, vbase; /* in mV */

	bcm15602_read_byte(ddata, BCM15602_REG_LDO_SDLDO_VOCTRL,
			   &reg_data);
	vbase = (reg_data & 0x80) ? 1725 : 1380;
	vstep = (reg_data & 0x80) ? 25 : 10;
	voctrl = reg_data & 0x3F;
	dev_info(dev, "%s: vbase=%d, vstep=%d, voctrl=%d", __func__, vbase,
		 vstep, voctrl);

	return snprintf(data, PAGE_SIZE, "%d\n", vbase + voctrl * vstep);
}

static ssize_t sdldo_vsel_store(struct device *dev,
				struct device_attribute *mattr,
				const char *data, size_t count)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;
	int vsel, voctrl, vstep, vbase; /* in mV */

	if (kstrtoint(data, 0, &vsel)) {
		dev_err(dev, "%s: Not a valid int\n", __func__);
		return -EINVAL;
	}

	if (vsel == 0) {
		bcm15602_update_bits(ddata, BCM15602_REG_LDO_SDLDO_ENCTRL, 0x1,
				     0x0);
		return count;
	} else if ((vsel < 1380) || (vsel > 1800)) {
		dev_err(dev, "%s: Not a valid voltage level, must be 0 or 1380-1800\n",
			__func__);
		return -EINVAL;
	}

	bcm15602_read_byte(ddata, BCM15602_REG_LDO_SDLDO_VOCTRL,
			   &reg_data);
	vbase = (reg_data & 0x80) ? 1725 : 1380;
	vstep = (reg_data & 0x80) ? 25 : 10;
	voctrl = reg_data & 0x3F;

	voctrl = ((vsel - vbase) + vstep - 1) / vstep;
	bcm15602_update_bits(ddata, BCM15602_REG_LDO_SDLDO_VOCTRL, 0x3F,
			     voctrl);
	bcm15602_update_bits(ddata, BCM15602_REG_LDO_SDLDO_ENCTRL, 0x1, 0x1);

	return count;
}
static DEVICE_ATTR_RW(sdldo_vsel);

static ssize_t sdsr_vsel_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;
	int voctrl, vstep, vbase; /* in mV */

	bcm15602_read_byte(ddata, BCM15602_REG_BUCK_SDSR_CTRL1, &reg_data);
	switch ((reg_data & 0xC0) >> 6) {
	case 0:
		vbase = 565;
		vstep = 5;
		break;
	case 1:
		vbase = 678;
		vstep = 6;
		break;
	case 2:
		vbase = 1130;
		vstep = 10;
		break;
	case 3:
		vbase = 1695;
		vstep = 15;
		break;
	default:
		return -EINVAL;
	}
	bcm15602_read_byte(ddata, BCM15602_REG_BUCK_SDSR_VOCTRL, &reg_data);
	voctrl = reg_data & 0x7F;

	dev_info(dev, "%s: vbase=%d, vstep=%d, voctrl=%d", __func__, vbase,
		 vstep, voctrl);

	return snprintf(data, PAGE_SIZE, "%d\n", vbase + voctrl * vstep);
}

static ssize_t sdsr_vsel_store(struct device *dev,
			       struct device_attribute *mattr,
			       const char *data, size_t count)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;
	int vsel, voctrl, vstep, vbase;

	if (kstrtoint(data, 0, &vsel)) {
		dev_err(dev, "%s: Not a valid int\n", __func__);
		return -EINVAL;
	}

	if (vsel == 0) {
		bcm15602_update_bits(ddata, BCM15602_REG_BUCK_SDSR_CTRL0, 0x1,
				     0x0);
		return count;
	} else if ((vsel < 565) || (vsel > 1200)) {
		dev_err(dev, "%s: Not a valid voltage level, must be 0 or 565-1200\n",
			__func__);
		return -EINVAL;
	}

	bcm15602_read_byte(ddata, BCM15602_REG_BUCK_SDSR_CTRL1, &reg_data);
	switch ((reg_data & 0xC0) >> 6) {
	case 0:
		vbase = 565;
		vstep = 5;
		break;
	case 1:
		vbase = 678;
		vstep = 6;
		break;
	case 2:
		vbase = 1130;
		vstep = 10;
		break;
	case 3:
		vbase = 1695;
		vstep = 15;
		break;
	default:
		return -EINVAL;
	}

	voctrl = ((vsel - vbase) + vstep - 1) / vstep;
	bcm15602_write_byte(ddata, BCM15602_REG_BUCK_SDSR_VOCTRL, voctrl &
			    0x7F);
	bcm15602_update_bits(ddata, BCM15602_REG_BUCK_SDSR_CTRL0, 0x1, 0x1);

	return count;
}
static DEVICE_ATTR_RW(sdsr_vsel);

static ssize_t dump_regs_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);

	bcm15602_dump_regs(ddata);

	return snprintf(data, PAGE_SIZE, "ok\n");
}
static DEVICE_ATTR_RO(dump_regs);

static ssize_t toggle_pon_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);

	bcm15602_toggle_pon(ddata);

	return snprintf(data, PAGE_SIZE, "ok\n");
}
static DEVICE_ATTR_RO(toggle_pon);

static ssize_t reg_read_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u8 reg_data;

	bcm15602_read_byte(ddata, _reg_read_addr, &reg_data);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_data);
}

static ssize_t reg_read_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf,
			      size_t count)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int val = 0;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	dev_dbg(ddata->dev, "%s: %d\n", __func__, val);

	if ((val < 0) || (val > 0xff))
		return -EINVAL;

	_reg_read_addr = val;

	return count;
}
static DEVICE_ATTR_RW(reg_read);

static ssize_t reg_write_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int val1 = 0, val2 = 0;
	int ret;
	char *str;

	str = strsep((char **)&buf, ";");
	if (!str)
		return -EINVAL;

	ret = kstrtoint(str, 0, &val1);
	if (ret < 0)
		return ret;

	if ((val1 < 0) || (val1 > 0xff))
		return -EINVAL;

	ret = kstrtoint(buf, 0, &val2);
	if (ret < 0)
		return ret;

	if ((val2 < 0) || (val2 > 0xff))
		return -EINVAL;

	dev_dbg(ddata->dev, "%s: reg 0x%02x, data 0x%02x\n", __func__,
		val1, val2);

	bcm15602_write_byte(ddata, val1, val2);

	return count;
}
static DEVICE_ATTR_WO(reg_write);

static struct attribute *bcm15602_attrs[] = {
	&dev_attr_asr_curr.attr,
	&dev_attr_sdsr_curr.attr,
	&dev_attr_sdldo_curr.attr,
	&dev_attr_ioldo_curr.attr,
	&dev_attr_vbat.attr,
	&dev_attr_temperature.attr,
	&dev_attr_total_power.attr,
	&dev_attr_hk_status.attr,
	&dev_attr_asr_vsel.attr,
	&dev_attr_ioldo_vsel.attr,
	&dev_attr_sdldo_vsel.attr,
	&dev_attr_sdsr_vsel.attr,
	&dev_attr_dump_regs.attr,
	&dev_attr_toggle_pon.attr,
	&dev_attr_reg_read.attr,
	&dev_attr_reg_write.attr,
	NULL
};

static const struct attribute_group bcm15602_attr_group = {
	.attrs = bcm15602_attrs,
};

void bcm15602_config_sysfs(struct device *dev)
{
	int ret;

	ret = sysfs_create_group(&dev->kobj, &bcm15602_attr_group);
	if (ret < 0) {
		dev_err(dev, "%s: could not create sysfs attributes (%d)\n",
			__func__, ret);
	}
}
EXPORT_SYMBOL_GPL(bcm15602_config_sysfs);

