// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "airbrush-thermal.h"
#include "airbrush-cooling.h"

#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/thermal.h>

struct ab_thermal;

struct ab_thermal_cooling {
	struct ab_thermal *thermal;
	struct ab_cooling *cooling;
	unsigned long state;
};

static int ab_thermal_cooling_init(struct ab_thermal_cooling *thermal_cooling,
		struct ab_thermal *thermal, struct device_node *np, char *type,
		const struct ab_cooling_ops *ops)
{
	thermal_cooling->thermal = thermal;
	thermal_cooling->cooling = ab_cooling_register(np, type, ops,
			thermal_cooling);
	if (IS_ERR(thermal_cooling->cooling))
		return PTR_ERR(thermal_cooling->cooling);
	thermal_cooling->state = THROTTLE_NONE;
	return 0;
}

static void ab_thermal_cooling_exit(struct ab_thermal_cooling *thermal_cooling)
{
	if (!IS_ERR_OR_NULL(thermal_cooling->cooling))
		ab_cooling_unregister(thermal_cooling->cooling);
}

struct ab_thermal {
	struct device *dev;

	struct mutex ops_lock;
	const struct ab_thermal_ops *ops;
	void *op_data;

	/* Combined from cooling, cooling_internal, and pcie_link_ready */
	enum throttle_state throttle_state;

	struct ab_thermal_cooling cooling;
	struct ab_thermal_cooling cooling_internal;

	struct mutex pcie_link_lock;
	bool pcie_link_ready;
	struct notifier_block pcie_link_blocking_nb;
};

static void ab_thermal_op_stub_throttle_state_updated(
	enum throttle_state throttle_state_id, void *op_data)
{}

static const struct ab_thermal_ops ab_thermal_ops_stub = {
	.throttle_state_updated = ab_thermal_op_stub_throttle_state_updated,
};

static void ab_thermal_exit(struct ab_thermal *thermal)
{
	abc_unregister_pcie_link_blocking_event(
		&thermal->pcie_link_blocking_nb);
	ab_thermal_cooling_exit(&thermal->cooling_internal);
	ab_thermal_cooling_exit(&thermal->cooling);
}

static enum throttle_state ab_thermal_throttle_state(unsigned long state)
{
	switch (state) {
	case 0:
		return THROTTLE_NONE;
	case 1:
		return THROTTLE_TO_MID;
	case 2:
		return THROTTLE_TO_LOW;
	default:
		return THROTTLE_TO_MIN;
	}
}

static void ab_thermal_state_update_no_pcie_link_lock(
		struct ab_thermal *thermal)
{
	unsigned long state = 0;
	enum throttle_state throttle_state;

	if (thermal->pcie_link_ready) {
		state = max(thermal->cooling.state,
				thermal->cooling_internal.state);
	}
	throttle_state = ab_thermal_throttle_state(state);
	if (throttle_state != thermal->throttle_state) {
		thermal->throttle_state = throttle_state;

		mutex_lock(&thermal->ops_lock);
		thermal->ops->throttle_state_updated(throttle_state,
				thermal->op_data);
		mutex_unlock(&thermal->ops_lock);
	}
}

static void ab_thermal_cooling_op_state_updated(
		const struct ab_cooling *cooling, unsigned long old_state,
		unsigned long new_state, void *cooling_op_data)
{
	struct ab_thermal_cooling *thermal_cooling = cooling_op_data;
	struct ab_thermal *thermal = thermal_cooling->thermal;

	thermal_cooling->state = new_state;

	mutex_lock(&thermal->pcie_link_lock);
	ab_thermal_state_update_no_pcie_link_lock(thermal);
	mutex_unlock(&thermal->pcie_link_lock);
}

const static struct ab_cooling_ops ab_thermal_cooling_ops = {
	.state_updated = ab_thermal_cooling_op_state_updated,
};

static void ab_thermal_pcie_link_post_enable(struct ab_thermal *thermal)
{
	mutex_lock(&thermal->pcie_link_lock);
	thermal->pcie_link_ready = true;
	ab_thermal_state_update_no_pcie_link_lock(thermal);
	mutex_unlock(&thermal->pcie_link_lock);
}

static void ab_thermal_pcie_link_pre_disable(struct ab_thermal *thermal)
{
	mutex_lock(&thermal->pcie_link_lock);
	thermal->pcie_link_ready = false;
	ab_thermal_state_update_no_pcie_link_lock(thermal);
	mutex_unlock(&thermal->pcie_link_lock);
}

static int ab_thermal_pcie_link_listener(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct ab_thermal *thermal = container_of(nb,
			struct ab_thermal, pcie_link_blocking_nb);

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		ab_thermal_pcie_link_post_enable(thermal);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_PRE_DISABLE) {
		ab_thermal_pcie_link_pre_disable(thermal);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int ab_thermal_init(struct ab_thermal *thermal, struct device *dev)
{
	int err;
	struct device_node *cooling_node;

	thermal->dev = dev;

	mutex_init(&thermal->ops_lock);
	thermal->ops = &ab_thermal_ops_stub;

	thermal->throttle_state = THROTTLE_NONE;

	mutex_init(&thermal->pcie_link_lock);
	mutex_lock(&thermal->pcie_link_lock);
	thermal->pcie_link_ready = true;
	thermal->pcie_link_blocking_nb.notifier_call =
		ab_thermal_pcie_link_listener;
	err = abc_register_pcie_link_blocking_event(
		&thermal->pcie_link_blocking_nb);
	mutex_unlock(&thermal->pcie_link_lock);
	if (err) {
		dev_err(dev, "failed to subscribe to PCIe blocking link event, ret %d\n",
			err);
		return err;
	}

	err = ab_thermal_cooling_init(&thermal->cooling, thermal, NULL,
			AB_CDEV_NAME, &ab_thermal_cooling_ops);
	if (err) {
		dev_err(dev, "failed to initialize external cooling\n");
		return err;
	}

	cooling_node = of_find_node_by_name(NULL, "abc-cooling");
	if (!cooling_node)
		dev_warn(dev, "Cannot find OF node for self cooling device. Cooling cannot trigger by airbrush itself.");
	err = ab_thermal_cooling_init(&thermal->cooling_internal, thermal,
			cooling_node, AB_CDEV_INTERNAL_NAME,
			&ab_thermal_cooling_ops);
	if (err) {
		dev_err(dev, "failed to initialize internal cooling\n");
		return err;
	}

	return 0;
}

void ab_thermal_set_ops(struct ab_thermal *thermal,
		const struct ab_thermal_ops *ops, void *op_data)
{
	mutex_lock(&thermal->ops_lock);
	thermal->ops = ops ? ops : &ab_thermal_ops_stub;
	thermal->op_data = op_data;
	mutex_unlock(&thermal->ops_lock);
}

static int ab_thermal_probe(struct platform_device *pdev)
{
	struct ab_thermal *thermal;
	int err;

	thermal = devm_kzalloc(&pdev->dev, sizeof(struct ab_thermal),
			GFP_KERNEL);
	if (!thermal)
		return -ENOMEM;

	err = ab_thermal_init(thermal, &pdev->dev);
	if (err < 0)
		return err;

	platform_set_drvdata(pdev, thermal);
	ab_sm_register_thermal(thermal);

	return 0;
}

static int ab_thermal_remove(struct platform_device *pdev)
{
	struct ab_thermal *thermal = platform_get_drvdata(pdev);

	ab_sm_unregister_thermal(thermal);
	ab_thermal_exit(thermal);
	return 0;
}

static const struct of_device_id ab_thermal_of_match[] = {
		{ .compatible = "abc,airbrush-thermal", },
		{ },
};

static struct platform_driver ab_thermal_driver = {
	.probe = ab_thermal_probe,
	.remove = ab_thermal_remove,
	.driver = {
		.name = "ab-thermal",
		.of_match_table = ab_thermal_of_match,
	}
};

module_platform_driver(ab_thermal_driver);
