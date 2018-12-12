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

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/abc-pcie.h>
#include "../../../thermal/thermal_core.h"

#include "hw.h"

#define AIRBRUSH_TMU_BASE	0xB90000

#define MCELSIUS	1000

static const u16 no_trimming_error1[] = {287, 286, 287, 287, 286, 286, 286};
static const u16 no_trimming_error2[] = {346, 346, 347, 347, 347, 346, 346};

/**
 * struct airbrush_tmu_data : A structure to hold the private data of the TMU
	driver
 * @pdata: pointer to the tmu platform/configuration data
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @irq_work: pointer to the irq work structure.
 * @lock: lock to implement synchronization.
 * @clk: pointer to the clock structure.
 * @clk_sec: pointer to the clock structure for accessing the base_second.
 * @sclk: pointer to the clock structure for accessing the tmu special clk.
 * @cal_type: calibration type for temperature calculation
 * @temp_error1: fused value of the first point trim.
 * @temp_error2: fused value of the second point trim.
 * @regulator: pointer to the TMU regulator structure.
 * @reg_conf: pointer to structure to register with core thermal.
 * @tmu_initialize: SoC specific TMU initialization method
 * @tmu_control: SoC specific TMU control method
 * @tmu_read: SoC specific TMU temperature read method
 * @tmu_set_emulation: SoC specific TMU emulation setting method
 */
struct airbrush_tmu_data {
	struct airbrush_tmu_platform_data *pdata;
	struct ab_tmu_hw *hw;
	int irq;
	struct notifier_block  tmu_nb;
	struct work_struct irq_work;
	struct clk *clk, *clk_sec, *sclk;
	u8 cal_type[AIRBRUSH_NUM_ALL_PROBE];
	u16 temp_error1[AIRBRUSH_NUM_ALL_PROBE];
	u16 temp_error2[AIRBRUSH_NUM_ALL_PROBE];
	struct regulator *regulator;
	struct thermal_zone_device *tzd;
};

/**
 * struct airbrush_tmu_platform_data
 * @gain: gain of amplifier in the positive-TC generator block
 *	0 < gain <= 15
 * @reference_voltage: reference voltage of amplifier
 *	in the positive-TC generator block
 *	0 < reference_voltage <= 31
 * @noise_cancel_mode: noise cancellation mode
 *	000, 100, 101, 110 and 111 can be different modes
 * @type: determines the type of SOC
 * @efuse_value: platform defined fuse value
 * @min_efuse_value: minimum valid trimming data
 * @max_efuse_value: maximum valid trimming data
 * @default_temp_offset: default temperature offset in case of no trimming
 *
 * This structure is required for configuration of airbrush_tmu driver.
 */
struct airbrush_tmu_platform_data {
	u8 gain;
	u8 reference_voltage;
	u8 noise_cancel_mode;

	u32 efuse_value;
	u32 min_efuse_value;
	u32 max_efuse_value;
	u16 first_point_trim;
	u16 second_point_trim;
	u8 default_temp_offset;
};

static void airbrush_report_trigger(struct airbrush_tmu_data *p)
{
	char data[11], *envp[] = { data, NULL };
	struct thermal_zone_device *tz = p->tzd;
	int temp;
	unsigned int i;

	if (!tz) {
		pr_err("No thermal zone device defined\n");
		return;
	}

	thermal_zone_device_update(tz, THERMAL_EVENT_UNSPECIFIED);

	mutex_lock(&tz->lock);
	/* Find the level for which trip happened */
	for (i = 0; i < of_thermal_get_ntrips(tz); i++) {
		tz->ops->get_trip_temp(tz, i, &temp);
		if (tz->last_temperature < temp)
			break;
	}

	snprintf(data, sizeof(data), "%u", i);
	kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, envp);
	mutex_unlock(&tz->lock);
}

static void airbrush_tmu_clear_irqs(struct airbrush_tmu_data *data)
{
	unsigned int val_irq, i;
	struct ab_tmu_hw *hw = data->hw;

	for (i = 0; i < AIRBRUSH_NUM_ALL_PROBE; i++) {
		val_irq = ab_tmu_hw_read(hw, AIRBRUSH_TMU_REG_INTPEND_P(i));
		ab_tmu_hw_write(hw, AIRBRUSH_TMU_REG_INTPEND_P(i), val_irq);
	}
}

static void airbrush_tmu_work(struct work_struct *work)
{
	struct airbrush_tmu_data *data = container_of(work,
			struct airbrush_tmu_data, irq_work);

	airbrush_report_trigger(data);

	airbrush_tmu_clear_irqs(data);

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

/*
 * TMU treats temperature as a mapped temperature code.
 * The temperature is converted differently depending on the calibration type.
 */
static int temp_to_code(struct airbrush_tmu_data *data, u16 temp, u8 probe)
{
	struct airbrush_tmu_platform_data *pdata = data->pdata;
	int temp_code;

	switch (data->cal_type[probe]) {
	case AIRBRUSH_NO_TRIMMING:
	case AIRBRUSH_TWO_POINT_TRIMMING:
		temp_code = (temp - pdata->first_point_trim) *
			(data->temp_error2[probe] - data->temp_error1[probe]) /
			(pdata->second_point_trim - pdata->first_point_trim) +
			data->temp_error1[probe];
		break;
	case AIRBRUSH_ONE_POINT_TRIMMING:
		temp_code = temp +
			data->temp_error1[probe] -
			pdata->first_point_trim;
		break;
	default:
		temp_code = temp + pdata->default_temp_offset;
		break;
	}

	return temp_code;
}

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree Celsius.
 */
static int code_to_temp(struct airbrush_tmu_data *data, u16 temp_code, u8 probe)
{
	struct airbrush_tmu_platform_data *pdata = data->pdata;
	int temp;

	switch (data->cal_type[probe]) {
	case AIRBRUSH_NO_TRIMMING:
	case AIRBRUSH_TWO_POINT_TRIMMING:
		temp = (temp_code - data->temp_error1[probe]) *
			(pdata->second_point_trim - pdata->first_point_trim) /
			(data->temp_error2[probe] - data->temp_error1[probe]) +
			pdata->first_point_trim;
		break;
	case AIRBRUSH_ONE_POINT_TRIMMING:
		temp = temp_code -
			data->temp_error1[probe] +
			pdata->first_point_trim;
		break;
	default:
		temp = temp_code - pdata->default_temp_offset;
		break;
	}
	pr_debug("ab_tmu_calc: probe=%u val=%u err1=%u err2=%u\n",
		probe, temp_code,
		data->temp_error1[probe], data->temp_error2[probe]);

	return temp;
}

static void airbrush_tmu_core_enable(struct platform_device *pdev, bool enable)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	struct ab_tmu_hw *hw = data->hw;
	unsigned int val;

	val = ab_tmu_hw_read(hw, AIRBRUSH_TMU_REG_CONTROL);

	if (enable) {
		val |= (1 << AIRBRUSH_TMU_EN_TRIP_SHIFT);
		val |= (1 << AIRBRUSH_TMU_CORE_EN_SHIFT);
	} else {
		val &= ~(1 << AIRBRUSH_TMU_EN_TRIP_SHIFT);
		val &= ~(1 << AIRBRUSH_TMU_CORE_EN_SHIFT);
	}

	ab_tmu_hw_write(hw, AIRBRUSH_TMU_REG_CONTROL, val);
}

static void airbrush_tmu_sensor_initialize(struct airbrush_tmu_data *data,
		int i)
{
	struct ab_tmu_hw *hw = data->hw;
	/* TODO We should have one tzd for each sensor instead. */
	unsigned int trim_info;
	struct thermal_zone_device *tz = data->tzd;
	unsigned int rising_threshold = 0, falling_threshold = 0;
	int j;
	int threshold_code;
	int temp, temp_hist;
	unsigned int reg_off, bit_off;

	trim_info = ab_tmu_hw_read(hw, AIRBRUSH_TMU_REG_TRIMINFO_P(i));

	data->cal_type[i] = (trim_info >> AIRBRUSH_TMU_CAL_SHIFT) &
			AIRBRUSH_TMU_CAL_MASK;

	data->temp_error1[i] = trim_info & AIRBRUSH_TMU_TEMP_MASK;
	data->temp_error2[i] = (trim_info >> AIRBRUSH_TMU_TEMP_SHIFT) &
			AIRBRUSH_TMU_TEMP_MASK;

	if (data->cal_type[i] == AIRBRUSH_NO_TRIMMING) {
		data->temp_error1[i] = no_trimming_error1[i];
		data->temp_error2[i] = no_trimming_error2[i];
	}

	// TODO different tz for each sensor
	for (j = (of_thermal_get_ntrips(tz) - 1); j >= 0; j--) {
		reg_off = ((7 - j) / 2) * 4;
		bit_off = ((8 - j) % 2);

		tz->ops->get_trip_temp(tz, j, &temp);
		temp /= MCELSIUS;

		tz->ops->get_trip_hyst(tz, j, &temp_hist);
		temp_hist = temp - (temp_hist / MCELSIUS);

		/* Set 9-bit temp code for rising threshold levels */
		threshold_code = temp_to_code(data, temp, i);
		rising_threshold = ab_tmu_hw_read(hw,
				AIRBRUSH_THD_TEMP_RISE7_6_P(i) + reg_off);
		rising_threshold &= ~(AIRBRUSH_TMU_TEMP_MASK << (16 * bit_off));
		rising_threshold |= threshold_code << (16 * bit_off);

		/* Set 9-bit temp code for falling threshold levels */
		threshold_code = temp_to_code(data, temp_hist, i);
		falling_threshold = ab_tmu_hw_read(hw,
				AIRBRUSH_THD_TEMP_FALL7_6_P(i) + reg_off);
		falling_threshold &= ~(AIRBRUSH_TMU_TEMP_MASK <<
				(16 * bit_off));
		falling_threshold |= threshold_code << (16 * bit_off);

		ab_tmu_hw_write(hw, AIRBRUSH_THD_TEMP_RISE7_6_P(i) + reg_off,
				rising_threshold);
		ab_tmu_hw_write(hw, AIRBRUSH_THD_TEMP_FALL7_6_P(i) + reg_off,
				falling_threshold);
	}
}

static int airbrush_tmu_initialize(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	struct ab_tmu_hw *hw = data->hw;
	unsigned int status;
	int i;

	status = ab_tmu_hw_read(hw, AIRBRUSH_TMU_REG_STATUS);
	if (!status)
		return -EBUSY;

	for (i = 0; i < AIRBRUSH_NUM_ALL_PROBE; i++)
		airbrush_tmu_sensor_initialize(data, i);

	airbrush_tmu_clear_irqs(data);
	return 0;
}

static void airbrush_tmu_control(struct platform_device *pdev, bool on)
{
	unsigned int i, con1, interrupt_en;
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	struct ab_tmu_hw *hw = data->hw;

	con1 = ab_tmu_hw_read(hw, AIRBRUSH_TMU_REG_CONTROL1);
	con1 |= AIRBRUSH_NUM_REMOTE_PROBE << AIRBRUSH_REMOTE_PROBE_SHIFT;

	interrupt_en = on ? AIRBRUSH_TMU_INT_EN : 0;

	for (i = 0; i < AIRBRUSH_NUM_ALL_PROBE; i++)
		ab_tmu_hw_write(hw, AIRBRUSH_TMU_REG_INTEN_P(i),
				interrupt_en);

	ab_tmu_hw_write(hw, AIRBRUSH_TMU_REG_CONTROL1, con1);
	airbrush_tmu_core_enable(pdev, on);
}

static int airbrush_get_temp(void *p, int *temp)
{
	struct airbrush_tmu_data *data = p;
	struct ab_tmu_hw *hw = data->hw;
	bool pcie_link_ready;
	int code;

	if (!data)
		return -EINVAL;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		code = ab_tmu_hw_read_current_temp(hw, 0);
		*temp = code_to_temp(data, code, 0) * MCELSIUS;
	} else {
		/*
		 * Return 0 degree Celsius while TMU is not ready to
		 * suppress the error log while reading the temperature.
		 */
		*temp = 0;
	}
	ab_tmu_hw_pcie_link_unlock(hw);
	return 0;
}

#ifdef CONFIG_THERMAL_EMULATION
void tmu_enable_emulate(struct ab_tmu_hw *hw)
{
	unsigned int emul_con;

	emul_con = ab_tmu_hw_read(hw, AIRBRUSH_EMUL_CON);
	emul_con |= (1 << AIRBRUSH_EMUL_ENABLE_SHIFT);

	ab_tmu_hw_write(hw, AIRBRUSH_EMUL_CON, emul_con);
}

void tmu_disable_emulate(struct ab_tmu_hw *hw)
{
	unsigned int emul_con;

	emul_con = ab_tmu_hw_read(hw, AIRBRUSH_EMUL_CON);
	emul_con &= ~(1 << AIRBRUSH_EMUL_ENABLE_SHIFT);

	ab_tmu_hw_write(hw, AIRBRUSH_EMUL_CON, emul_con);
}

void tmu_set_emulate_data(struct airbrush_tmu_data *data,  u16 next_time,
								u16 next_data)
{
	struct ab_tmu_hw *hw = data->hw;
	u32 uReg;

	next_time &= 0xFFFF;
	next_data &= 0x1FF;
	if (next_time == 0x0)
		next_time = 0x1;
	uReg = (ab_tmu_hw_read(hw, AIRBRUSH_EMUL_CON) >> 0) & 0x1;
	uReg |= (next_time << AIRBRUSH_EMUL_NEXTTIME_SHIFT) |
		(temp_to_code(data, next_data, 0) << AIRBRUSH_EMUL_DATA_SHIFT);

	ab_tmu_hw_write(hw, AIRBRUSH_EMUL_CON, uReg);
}

static int airbrush_tmu_set_emulation(void *drv_data, int temp)
{
	struct airbrush_tmu_data *data = drv_data;
	struct ab_tmu_hw *hw = data->hw;
	bool pcie_link_ready;
	int ret;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		if (temp != 0) {
			tmu_enable_emulate(hw);
			tmu_set_emulate_data(data, 1, temp / MCELSIUS);
		} else {
			tmu_disable_emulate(hw);
		}
		ret = 0;
	} else {
		ret = -ENODEV;
	}
	ab_tmu_hw_pcie_link_unlock(hw);

	return ret;
}
#else
static int airbrush_tmu_set_emulation(void *drv_data, int temp)
{
	return -EINVAL;
}
#endif /* CONFIG_THERMAL_EMULATION */

static const struct of_device_id airbrush_tmu_match[] = {
	{ .compatible = "abc,airbrush-tmu", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, airbrush_tmu_match);

#define	AIRBRUSH_TMU_GAIN			9
#define AIRBRUSH_TMU_REF_VOLT			17
#define AIRBRUSH_TMU_NOSIE_CANCEL_MODE		4
#define AIRBRUSH_TMU_EFUSE_VAL			292
#define AIRBRUSH_TMU_FIRST_POINT_TRIM		25
#define AIRBRUSH_TMU_SECOND_POINT_TRIM		85
#define AIRBRUSH_TMU_DEFAULT_TEMP_OFFSET	50
#define AIRBRUSH_TMU_MIN_EFUSE_VAL		15
#define AIRBRUSH_TMU_MAX_EFUSE_VAL		135

static int airbrush_map_dt_data(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	struct airbrush_tmu_platform_data *pdata;

	if (!data || !pdev->dev.of_node)
		return -ENODEV;

	data->irq = 36;

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct airbrush_tmu_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->gain = AIRBRUSH_TMU_GAIN;
	pdata->reference_voltage = AIRBRUSH_TMU_REF_VOLT;
	pdata->noise_cancel_mode = AIRBRUSH_TMU_NOSIE_CANCEL_MODE;

	pdata->efuse_value = AIRBRUSH_TMU_EFUSE_VAL;

	pdata->first_point_trim = AIRBRUSH_TMU_FIRST_POINT_TRIM;
	pdata->second_point_trim = AIRBRUSH_TMU_SECOND_POINT_TRIM;
	pdata->default_temp_offset = AIRBRUSH_TMU_DEFAULT_TEMP_OFFSET;

	pdata->min_efuse_value = AIRBRUSH_TMU_MIN_EFUSE_VAL;
	pdata->max_efuse_value = AIRBRUSH_TMU_MAX_EFUSE_VAL;

	data->pdata = pdata;

	return 0;
}

static struct thermal_zone_of_device_ops airbrush_sensor_ops = {
	.get_temp = airbrush_get_temp,
	.set_emul_temp = airbrush_tmu_set_emulation,
};

static ssize_t temp_probe_show(struct device *dev, int id, char *buf)
{
	struct airbrush_tmu_data *data = dev_get_drvdata(dev);
	struct ab_tmu_hw *hw = data->hw;
	int code, temp;

	code = ab_tmu_hw_read_current_temp(hw, id);
	temp = code_to_temp(data, code, id);
	return scnprintf(buf, PAGE_SIZE, "%d\n", temp);
}

#define AB_TMU_TEMP_PROBE_ATTR_RO(name, id) \
	static ssize_t temp_probe_##name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		return temp_probe_show(dev, (id), buf); \
	} \
	static DEVICE_ATTR_RO(temp_probe_##name)

AB_TMU_TEMP_PROBE_ATTR_RO(main, AIRBRUSH_TEMP_PROBE_MAIN);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu0, AIRBRUSH_TEMP_PROBE_IPU0);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu1, AIRBRUSH_TEMP_PROBE_IPU1);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu2, AIRBRUSH_TEMP_PROBE_IPU2);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu_tpu, AIRBRUSH_TEMP_PROBE_IPU_TPU);
AB_TMU_TEMP_PROBE_ATTR_RO(tpu0, AIRBRUSH_TEMP_PROBE_TPU0);
AB_TMU_TEMP_PROBE_ATTR_RO(tpu1, AIRBRUSH_TEMP_PROBE_TPU1);

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
	data->hw = devm_ab_tmu_hw_create(&pdev->dev, AIRBRUSH_TMU_BASE);
	if (IS_ERR(data->hw))
		return PTR_ERR(data->hw);

	/* TODO: See if this function can be remove */
	ret = airbrush_map_dt_data(pdev);
	if (ret)
		return ret;

	INIT_WORK(&data->irq_work, airbrush_tmu_work);

	/* TODO: get clock from dt */

	/*
	 * data->tzd must be registered before calling
	 * airbrush_tmu_initialize(), requesting irq and calling
	 * airbrush_tmu_control().
	 */
	data->tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, 0, data,
						    &airbrush_sensor_ops);
	if (IS_ERR(data->tzd)) {
		dev_err(&pdev->dev, "Failed to register sensor: %d\n", ret);
		return PTR_ERR(data->tzd);
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

	airbrush_tmu_control(pdev, true);

	dev_dbg(&pdev->dev, "%s: done.\n", __func__);
	return 0;
}

static int airbrush_tmu_remove(struct platform_device *pdev)
{
	airbrush_tmu_control(pdev, false);
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
