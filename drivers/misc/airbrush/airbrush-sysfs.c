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
	u32 val;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	val = ab_sm_get_state((struct ab_state_context *)sc, false);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
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

	ret = ab_sm_set_state(sc, val, false);
	if (ret < 0) {
		dev_err(sc->dev, "%s: State change failed, ret %d\n",
				__func__, ret);
		return ret;
	}

	return count;
}

static ssize_t mapped_chip_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	u32 val;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	val = ab_sm_get_state((struct ab_state_context *)sc, true);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t mapped_chip_state_store(struct device *dev,
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

	ret = ab_sm_set_state(sc, val, true);
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
	enum ab_chip_id chip_id;

	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	chip_id = ab_get_chip_id(sc);

	switch (chip_id) {
	case CHIP_ID_A0:
		return scnprintf(buf, PAGE_SIZE, "A0\n");
	case CHIP_ID_B0:
		return scnprintf(buf, PAGE_SIZE, "B0\n");
	default:
		return scnprintf(buf, PAGE_SIZE, "Unknown\n");
	}

	return 0;
}

static ssize_t state_stats_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	/* keep contents of states array in sync with stat_state enum */
	const char * const states[STAT_STATE_SIZE] = {"ACTIVE", "SLEEP",
		"DEEP SLEEP", "SUSPEND", "OFF", "UNKNOWN"};
	u32 pos = 0;
	u32 i;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	pos += scnprintf(buf + pos, PAGE_SIZE - pos,
			"Pixel Visual Core Subsystem Power Stats\n");

	mutex_lock(&sc->state_transitioning_lock);

	for (i = 0; i < STAT_STATE_SIZE; i++) {
		ktime_t adjusted_duration = sc->state_stats[i].duration;
		ktime_t last_entry = sc->state_stats[i].last_entry;
		ktime_t last_exit = sc->state_stats[i].last_exit;

		/* adjust duration for current state */
		if (ab_chip_state_to_stat_state(sc->curr_chip_substate_id) == i
				&& ktime_after(last_entry, last_exit)) {
			ktime_t partial_duration = ktime_sub(
					ktime_get_boottime(), last_entry);
			adjusted_duration = ktime_add(adjusted_duration,
					partial_duration);
		}

		pos += scnprintf(buf + pos, PAGE_SIZE - pos, "%s\n"
				"\tCumulative count: %llu\n"
				"\tCumulative duration msec:  %llu\n"
				"\tLast entry timestamp msec: %llu\n"
				"\tLast exit timestamp msec:  %llu\n",
				states[i], sc->state_stats[i].counter,
				ktime_to_ms(adjusted_duration),
				ktime_to_ms(last_entry),
				ktime_to_ms(last_exit));
	}

	mutex_unlock(&sc->state_transitioning_lock);

	return pos;
}

/* Trigger an error event for testing purpose; input is ignored */
static ssize_t gen_error_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	dev_warn(dev, "Triggering an error event for testing.\n");
	sysfs_notify(&dev->kobj, NULL, "error_event");
	return count;
}

/* Supposed to be polled by userspace daemon. */
static ssize_t error_event_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Return fixed string "1" */
	return scnprintf(buf, PAGE_SIZE, "%d\n", 1);
}

static struct device_attribute ab_sm_attrs[] = {
	__ATTR_RW(chip_state),
	__ATTR_RW(mapped_chip_state),
	__ATTR_RO(version),
	__ATTR_RO(state_stats),
	__ATTR_RO(error_event),
	__ATTR_WO(gen_error),
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
