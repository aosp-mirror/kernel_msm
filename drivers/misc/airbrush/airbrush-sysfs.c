/*
 * Authors:
 * Shaun Kelsey <shaunkelsey@google.com>
 *
 * Airbrush State Manager Sysfs Entries
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/stat.h>
#include <linux/sysfs.h>

#include "airbrush-regs.h"

static ssize_t chip_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", sc->chip_substate_id);
}

static ssize_t chip_state_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret;
	int val;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	if (kstrtoint(buf, 10, &val) < 0) {
		dev_err(sc->dev, "Bad input format\n");
		return -EIO;
	}

	ret = ab_sm_set_state(sc, val);
	if (ret < 0) {
		dev_err(sc->dev, "%s: State change failed, ret %d\n",
				__func__, ret);
		return ret;
	}

	return count;
}

static ssize_t version_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	uint32_t val;
	int ret = 0;

	ret = ABC_READ(OTP_CHIP_ID_ADDR, &val);
	if (ret < 0)
		return ret;

	val = (val & OTP_CHIP_ID_MASK) >> OTP_CHIP_ID_SHIFT;

	switch (val) {
	case CHIP_ID_A0:
		return scnprintf(buf, PAGE_SIZE, "A0\n");
	case CHIP_ID_B0:
		return scnprintf(buf, PAGE_SIZE, "B0\n");
	default:
		return scnprintf(buf, PAGE_SIZE, "Unknown\n");
	}
	return 0;
}

static struct device_attribute ab_sm_attrs[] = {
	__ATTR_RW(chip_state),
	__ATTR_RO(version),
};

void ab_sm_create_sysfs(struct ab_state_context *sc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ab_sm_attrs); i++)
		device_create_file(sc->dev, &ab_sm_attrs[i]);
}

void ab_sm_remove_sysfs(struct ab_state_context *sc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ab_sm_attrs); i++)
		device_remove_file(sc->dev, &ab_sm_attrs[i]);
}
