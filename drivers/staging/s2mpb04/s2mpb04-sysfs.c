/*
 * Copyright (C) 2017 Google, Inc.
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

#include <linux/device.h>
#include <linux/module.h>

#include "s2mpb04-core.h"

/* smps1_volt is in uV */
static int s2mpb04_get_smps1_volt(struct s2mpb04_core *ddata, int *smps1_volt)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_V_SMPS1, &chan_data);

	*smps1_volt = chan_data * S2MPB04_ADC_SCALE_V_SMPS1;

	return 0;
}

/* smps1_curr is in uA */
static int s2mpb04_get_smps1_curr(struct s2mpb04_core *ddata, int *smps1_curr)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_I_SMPS1_SUM, &chan_data);

	*smps1_curr = chan_data * S2MPB04_ADC_SCALE_I_SMPS1;

	return 0;
}

/* smps2_volt is in uV */
static int s2mpb04_get_smps2_volt(struct s2mpb04_core *ddata, int *smps2_volt)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_V_SMPS2, &chan_data);

	*smps2_volt = chan_data * S2MPB04_ADC_SCALE_V_SMPS2;

	return 0;
}

/* smps2_curr is in uA */
static int s2mpb04_get_smps2_curr(struct s2mpb04_core *ddata, int *smps2_curr)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_I_SMPS2, &chan_data);

	*smps2_curr = chan_data * S2MPB04_ADC_SCALE_I_SMPS2;

	return 0;
}

/* ldo1_volt is in uV */
static int s2mpb04_get_ldo1_volt(struct s2mpb04_core *ddata, int *ldo1_volt)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_V_LDO1, &chan_data);

	*ldo1_volt = chan_data * S2MPB04_ADC_SCALE_V_LDO;

	return 0;
}

/* ldo1_curr is in uA */
static int s2mpb04_get_ldo1_curr(struct s2mpb04_core *ddata, int *ldo1_curr)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_I_LDO1, &chan_data);

	*ldo1_curr = chan_data * S2MPB04_ADC_SCALE_I_LDO;

	return 0;
}

/* ldo2_volt is in uV */
static int s2mpb04_get_ldo2_volt(struct s2mpb04_core *ddata, int *ldo2_volt)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_V_LDO2, &chan_data);

	*ldo2_volt = chan_data * S2MPB04_ADC_SCALE_V_LDO;

	return 0;
}

/* ldo2_curr is in uA */
static int s2mpb04_get_ldo2_curr(struct s2mpb04_core *ddata, int *ldo2_curr)
{
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_I_LDO2, &chan_data);

	*ldo2_curr = chan_data * S2MPB04_ADC_SCALE_I_LDO;

	return 0;
}

static ssize_t smps1_volt_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int smps1_volt;

	s2mpb04_get_smps1_volt(ddata, &smps1_volt);

	return snprintf(data, PAGE_SIZE, "%d\n", smps1_volt);
}
DEVICE_ATTR_RO(smps1_volt);

static ssize_t smps1_curr_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int smps1_curr;

	s2mpb04_get_smps1_curr(ddata, &smps1_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", smps1_curr);
}
DEVICE_ATTR_RO(smps1_curr);

static ssize_t smps2_volt_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int smps2_volt;

	s2mpb04_get_smps2_volt(ddata, &smps2_volt);

	return snprintf(data, PAGE_SIZE, "%d\n", smps2_volt);
}
DEVICE_ATTR_RO(smps2_volt);

static ssize_t smps2_curr_show(struct device *dev,
			       struct device_attribute *mattr,
			       char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int smps2_curr;

	s2mpb04_get_smps2_curr(ddata, &smps2_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", smps2_curr);
}
DEVICE_ATTR_RO(smps2_curr);

static ssize_t ldo1_volt_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int ldo1_volt;

	s2mpb04_get_ldo1_volt(ddata, &ldo1_volt);

	return snprintf(data, PAGE_SIZE, "%d\n", ldo1_volt);
}
DEVICE_ATTR_RO(ldo1_volt);

static ssize_t ldo1_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int ldo1_curr;

	s2mpb04_get_ldo1_curr(ddata, &ldo1_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", ldo1_curr);
}
DEVICE_ATTR_RO(ldo1_curr);

static ssize_t ldo2_volt_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int ldo2_volt;

	s2mpb04_get_ldo2_volt(ddata, &ldo2_volt);

	return snprintf(data, PAGE_SIZE, "%d\n", ldo2_volt);
}
DEVICE_ATTR_RO(ldo2_volt);

static ssize_t ldo2_curr_show(struct device *dev,
			      struct device_attribute *mattr,
			      char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int ldo2_curr;

	s2mpb04_get_ldo2_curr(ddata, &ldo2_curr);

	return snprintf(data, PAGE_SIZE, "%d\n", ldo2_curr);
}
DEVICE_ATTR_RO(ldo2_curr);

static ssize_t vbat_show(struct device *dev,
			 struct device_attribute *mattr,
			 char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_VBAT, &chan_data);

	return snprintf(data, PAGE_SIZE, "%d\n",
			chan_data * S2MPB04_ADC_SCALE_VBAT);
}
DEVICE_ATTR_RO(vbat);

static ssize_t temperature_show(struct device *dev,
				struct device_attribute *mattr,
				char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	u8 chan_data;

	s2mpb04_read_adc_chan(ddata, S2MPB04_ADC_PTAT, &chan_data);

	return snprintf(data, PAGE_SIZE, "%d\n", PTAT_CODE_TO_TEMP(chan_data));
}
DEVICE_ATTR_RO(temperature);

/* shows power in mW */
static ssize_t total_power_show(struct device *dev,
				struct device_attribute *mattr,
				char *data)
{
	struct s2mpb04_core *ddata = dev_get_drvdata(dev);
	int smps1_curr, smps2_curr, ldo2_curr, ldo1_curr;
	long int total_power;

	s2mpb04_get_smps1_curr(ddata, &smps1_curr);
	s2mpb04_get_smps2_curr(ddata, &smps2_curr);
	s2mpb04_get_ldo2_curr(ddata, &ldo2_curr);
	s2mpb04_get_ldo1_curr(ddata, &ldo1_curr);

	total_power = (((long int)smps1_curr) * 900 / 1000) +
		(((long int)smps2_curr) * 1100 / 1000) +
		(((long int)ldo2_curr) * 1800 / 1000) +
		(((long int)ldo1_curr) * 1800 / 1000);

	return snprintf(data, PAGE_SIZE, "%ld\n", total_power);
}
DEVICE_ATTR_RO(total_power);

static struct attribute *s2mpb04_attrs[] = {
	&dev_attr_smps1_volt.attr,
	&dev_attr_smps1_curr.attr,
	&dev_attr_smps2_volt.attr,
	&dev_attr_smps2_curr.attr,
	&dev_attr_ldo1_volt.attr,
	&dev_attr_ldo1_curr.attr,
	&dev_attr_ldo2_volt.attr,
	&dev_attr_ldo2_curr.attr,
	&dev_attr_vbat.attr,
	&dev_attr_temperature.attr,
	&dev_attr_total_power.attr,
	NULL
};

static const struct attribute_group s2mpb04_attr_group = {
	.attrs = s2mpb04_attrs,
};

void s2mpb04_config_sysfs(struct device *dev)
{
	int ret;

	ret = sysfs_create_group(&dev->kobj, &s2mpb04_attr_group);
	if (ret < 0) {
		dev_err(dev, "%s: could not create sysfs attributes (%d)\n",
			__func__, ret);
	}
}
EXPORT_SYMBOL_GPL(s2mpb04_config_sysfs);

