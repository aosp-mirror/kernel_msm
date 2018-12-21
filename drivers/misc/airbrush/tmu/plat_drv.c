/*
 * airbrush_tmu.c - AIRBRUSH TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2018 Samsung Electronics
 *  Alim Akhtar <alim.akhtar@samsung.com>
 *
 * Based on exynos_tmu.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/abc-pcie.h>

#include "hw.h"
#include "sensor.h"

#define AB_TMU_BASE	0xB90000

/**
 * struct airbrush_tmu_data : A structure to hold the private data of the TMU
	driver
 * @irq_work: pointer to the irq work structure.
 * @sensor: handles to sensor operation, the id of sensor is the same as
 * its index at this array.
 */
struct airbrush_tmu_data {
	struct ab_tmu_hw *hw;
	int irq;
	struct notifier_block tmu_nb;
	struct work_struct irq_work;
	struct ab_tmu_sensor *sensor[AB_TMU_NUM_ALL_PROBE];
};

static void airbrush_tmu_work(struct work_struct *work)
{
	struct airbrush_tmu_data *data = container_of(work,
			struct airbrush_tmu_data, irq_work);
	int i;

	/* TODO trigger only the correct sensor */
	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++)
		ab_tmu_sensor_update(data->sensor[i]);

	ab_tmu_hw_clear_irqs(data->hw);

	enable_irq(ABC_MSI_AON_INTNC);
}

int airbrush_tmu_irq_handler(unsigned int irq, struct airbrush_tmu_data *data)
{
	disable_irq_nosync(ABC_MSI_AON_INTNC);
	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static int tmu_irq_notify(struct notifier_block *nb,
					unsigned long irq, void *data)
{
	struct airbrush_tmu_data *tmu_data =
		container_of(nb, struct airbrush_tmu_data, tmu_nb);
	u32 intnc_val = (u32)data;

	if (irq == ABC_MSI_AON_INTNC &&
			(intnc_val & (1 << (tmu_data->irq - ABC_MSI_COUNT))))
		return airbrush_tmu_irq_handler(irq, tmu_data);

	return 0;
}

static int airbrush_tmu_initialize(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	struct ab_tmu_hw *hw = data->hw;
	unsigned int status;
	int i;

	status = ab_tmu_hw_read(hw, AB_TMU_STATUS);
	if (!status)
		return -EBUSY;

	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++) {
		ab_tmu_sensor_load_trim_info(data->sensor[i]);
		ab_tmu_sensor_save_threshold(data->sensor[i]);
	}

	ab_tmu_hw_clear_irqs(hw);
	return 0;
}

static const struct of_device_id airbrush_tmu_match[] = {
	{ .compatible = "abc,airbrush-tmu", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, airbrush_tmu_match);

static ssize_t temp_probe_show(struct device *dev, int id, char *buf)
{
	struct airbrush_tmu_data *data = dev_get_drvdata(dev);
	struct ab_tmu_hw *hw = data->hw;
	int code, temp;

	code = ab_tmu_hw_read_current_temp(hw, id);
	temp = ab_tmu_sensor_raw_to_cel(data->sensor[id], code);
	return scnprintf(buf, PAGE_SIZE, "%d\n", temp);
}

#define AB_TMU_TEMP_PROBE_ATTR_RO(name, id) \
	static ssize_t temp_probe_##name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		return temp_probe_show(dev, (id), buf); \
	} \
	static DEVICE_ATTR_RO(temp_probe_##name)

AB_TMU_TEMP_PROBE_ATTR_RO(main, AB_TMU_TEMP_PROBE_MAIN);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu0, AB_TMU_TEMP_PROBE_IPU0);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu1, AB_TMU_TEMP_PROBE_IPU1);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu2, AB_TMU_TEMP_PROBE_IPU2);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu_tpu, AB_TMU_TEMP_PROBE_IPU_TPU);
AB_TMU_TEMP_PROBE_ATTR_RO(tpu0, AB_TMU_TEMP_PROBE_TPU0);
AB_TMU_TEMP_PROBE_ATTR_RO(tpu1, AB_TMU_TEMP_PROBE_TPU1);

static struct attribute *airbrush_tmu_attrs[] = {
	&dev_attr_temp_probe_main.attr,
	&dev_attr_temp_probe_ipu0.attr,
	&dev_attr_temp_probe_ipu1.attr,
	&dev_attr_temp_probe_ipu2.attr,
	&dev_attr_temp_probe_ipu_tpu.attr,
	&dev_attr_temp_probe_tpu0.attr,
	&dev_attr_temp_probe_tpu1.attr,
	NULL,
};

ATTRIBUTE_GROUPS(airbrush_tmu);

static int airbrush_tmu_probe(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data;
	int i;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct airbrush_tmu_data),
					GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = devm_device_add_groups(&pdev->dev, airbrush_tmu_groups);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, data);

	/* TODO Probe from device tree instead of a hard coded base. */
	data->hw = devm_ab_tmu_hw_create(&pdev->dev, AB_TMU_BASE);
	if (IS_ERR(data->hw))
		return PTR_ERR(data->hw);

	data->irq = 36;
	INIT_WORK(&data->irq_work, airbrush_tmu_work);

	/*
	 * data->sensor must be created before calling
	 * airbrush_tmu_initialize(), requesting irq and calling
	 * ab_tmu_hw_control().
	 */
	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++) {
		data->sensor[i] = devm_ab_tmu_sensor_create(&pdev->dev,
				data->hw, i);
		if (IS_ERR(data->sensor[i])) {
			dev_err(&pdev->dev, "Failed to register sensor %d", i);
			return PTR_ERR(data->sensor[i]);
		}
	}

	/* register interrupt handler with PCIe subsystem */
	data->tmu_nb.notifier_call = tmu_irq_notify;
	ret = abc_reg_notifier_callback(&data->tmu_nb);
	if (ret) {
		dev_err(&pdev->dev, "TMU probe is deffered\n");
		return -EPROBE_DEFER;
	}

	/* tmu initialization */
	ret = airbrush_tmu_initialize(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize TMU\n");
		return ret;
	}

	ab_tmu_hw_control(data->hw, true);

	dev_dbg(&pdev->dev, "%s: done.\n", __func__);
	return 0;
}

static int airbrush_tmu_remove(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);

	ab_tmu_hw_control(data->hw, false);
	return 0;
}

static struct platform_driver airbrush_tmu_driver = {
	.driver = {
		.name   = "airbrush-tmu",
		.of_match_table = airbrush_tmu_match,
	},
	.probe = airbrush_tmu_probe,
	.remove	= airbrush_tmu_remove,
};

module_platform_driver(airbrush_tmu_driver);

MODULE_DESCRIPTION("AIRBRUSH TMU Driver");
MODULE_AUTHOR("Alim Akhtar <alim.akhtar@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:airbrush-tmu");
