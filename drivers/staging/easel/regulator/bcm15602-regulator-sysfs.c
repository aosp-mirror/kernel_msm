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

static ssize_t bcm15602_attr_show_asr_curr
	(struct device *dev, struct device_attribute *mattr, char *data);
static ssize_t bcm15602_attr_show_sdsr_curr
	(struct device *dev, struct device_attribute *mattr, char *data);
static ssize_t bcm15602_attr_show_sdldo_curr
	(struct device *dev, struct device_attribute *mattr, char *data);
static ssize_t bcm15602_attr_show_ioldo_curr
	(struct device *dev, struct device_attribute *mattr, char *data);
static ssize_t bcm15602_attr_show_vbat
	(struct device *dev, struct device_attribute *mattr, char *data);
static ssize_t bcm15602_attr_show_temperature
	(struct device *dev, struct device_attribute *mattr, char *data);
static ssize_t bcm15602_attr_show_total_power
	(struct device *dev, struct device_attribute *mattr, char *data);
static ssize_t bcm15602_attr_show_hk_status
	(struct device *dev, struct device_attribute *mattr, char *data);

DEVICE_ATTR(asr_curr, 0440, bcm15602_attr_show_asr_curr, NULL);
DEVICE_ATTR(sdsr_curr, 0440, bcm15602_attr_show_sdsr_curr, NULL);
DEVICE_ATTR(sdldo_curr, 0440, bcm15602_attr_show_sdldo_curr, NULL);
DEVICE_ATTR(ioldo_curr, 0440, bcm15602_attr_show_ioldo_curr, NULL);
DEVICE_ATTR(vbat, 0440, bcm15602_attr_show_vbat, NULL);
DEVICE_ATTR(temperature, 0440, bcm15602_attr_show_temperature, NULL);
DEVICE_ATTR(total_power, 0440, bcm15602_attr_show_total_power, NULL);
DEVICE_ATTR(hk_status, 0440, bcm15602_attr_show_hk_status, NULL);

static struct attribute *bcm15602_attrs[] = {
	&dev_attr_asr_curr.attr,
	&dev_attr_sdsr_curr.attr,
	&dev_attr_sdldo_curr.attr,
	&dev_attr_ioldo_curr.attr,
	&dev_attr_vbat.attr,
	&dev_attr_temperature.attr,
	&dev_attr_total_power.attr,
	&dev_attr_hk_status.attr,
	NULL
};

static const struct attribute_group bcm15602_attr_group = {
	.attrs = bcm15602_attrs,
};

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

static ssize_t bcm15602_attr_show_asr_curr(struct device *dev,
					   struct device_attribute *mattr,
					   char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int asr_curr;

	bcm15602_get_asr_curr(ddata, &asr_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", asr_curr);
}

static ssize_t bcm15602_attr_show_sdsr_curr(struct device *dev,
					    struct device_attribute *mattr,
					    char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int sdsr_curr;

	bcm15602_get_sdsr_curr(ddata, &sdsr_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", sdsr_curr);
}

static ssize_t bcm15602_attr_show_sdldo_curr(struct device *dev,
					     struct device_attribute *mattr,
					     char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int sdldo_curr;

	bcm15602_get_sdldo_curr(ddata, &sdldo_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", sdldo_curr);
}

static ssize_t bcm15602_attr_show_ioldo_curr(struct device *dev,
					     struct device_attribute *mattr,
					     char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	int ioldo_curr;

	bcm15602_get_ioldo_curr(ddata, &ioldo_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", ioldo_curr);
}

static ssize_t bcm15602_attr_show_vbat(struct device *dev,
				       struct device_attribute *mattr,
				       char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u16 chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_VBAT, &chan_data);

	return snprintf(data, PAGE_SIZE, "%d\n",
			chan_data * BCM15602_ADC_SCALE_VBAT);
}

static ssize_t bcm15602_attr_show_temperature(struct device *dev,
					      struct device_attribute *mattr,
					      char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u16 chan_data;

	bcm15602_read_adc_chan(ddata, BCM15602_ADC_PTAT, &chan_data);

	return snprintf(data, PAGE_SIZE, "%d\n", PTAT_CODE_TO_TEMP(chan_data));
}

/* shows power in mW */
static ssize_t bcm15602_attr_show_total_power(struct device *dev,
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

	total_power = (((long int)asr_curr) * 950 / 1000) +
		(((long int)sdsr_curr) * 1100 / 1000) +
		(((long int)sdldo_curr) * 1800 / 1000) +
		(((long int)ioldo_curr) * 1800 / 1000);

	return snprintf(data, PAGE_SIZE, "%ld\n", total_power);
}

static ssize_t bcm15602_attr_show_hk_status(struct device *dev,
					    struct device_attribute *mattr,
					    char *data)
{
	struct bcm15602_chip *ddata = dev_get_drvdata(dev);
	u16 status;

	status = ddata->hk_status;
	ddata->hk_status &= ~status;

	return snprintf(data, PAGE_SIZE, "0x%04x\n", status);
}

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

