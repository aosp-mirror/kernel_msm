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
#include "../../thermal/thermal_core.h"

/* Exynos generic registers */
#define AIRBRUSH_TMU_REG_TRIMINFO	0x0
#define AIRBRUSH_TMU_REG_CONTROL	0x20
#define AIRBRUSH_TMU_REG_CONTROL1	0x24
#define AIRBRUSH_TMU_REG_STATUS		0x28
#define AIRBRUSH_TMU_SAMPLING_INTERVAL	0x02c
#define AIRBRUSH_TMU_COUNTER_VALUE0	0x030
#define AIRBRUSH_TMU_COUNTER_VALUE1	0x034
#define AIRBRUSH_TMU_AVG_CONTROL	0x034
#define AIRBRUSH_TMU_REG_CURRENT_TEMP0_1	0x040
#define AIRBRUSH_TMU_REG_CURRENT_TEMP2_4	0x044
#define AIRBRUSH_TMU_REG_CURRENT_TEMP5_7	0x048
#define AIRBRUSH_THD_TEMP_RISE7_6		0x050
#define AIRBRUSH_THD_TEMP_RISE5_4		0x054
#define AIRBRUSH_THD_TEMP_RISE3_2		0x058
#define AIRBRUSH_THD_TEMP_RISE1_0		0x05C
#define AIRBRUSH_TMU_REG_INTEN			0x110
#define AIRBRUSH_TMU_REG_INTSTAT		0x114
#define AIRBRUSH_TMU_REG_INTPEND		0x118

#define AIRBRUSH_TMU_INTEN_FALL0_SHIFT		16

#define AIRBRUSH_TMU_TEMP_MASK		0x1ff
#define AIRBRUSH_TMU_BUF_SLOPE_SEL_MASK	0xf
#define AIRBRUSH_TMU_BUF_SLOPE_SEL_SHIFT	8
#define AIRBRUSH_TMU_CORE_EN_SHIFT	0
#define AIRBRUSH_TMU_EN_TRIP_SHIFT	12
#define AIRBRUSH_TMU_TRIP_MODE_SHIFT	13


#define AIRBRUSH_EMUL_CON		0x160
#define AIRBRUSH_DEBUG_CURRENT_TEMP	0x164
#define AIRBRUSH_EMUL_DATA_SHIFT	7
#define AIRBRUSH_EMUL_DATA_MASK		0x1ff
#define AIRBRUSH_EMUL_NEXTTIME_SHIFT	16
#define AIRBRUSH_EMUL_NEXTTIME_VAL	0x1
#define AIRBRUSH_EMUL_ENABLE_SHIFT	0
#define AIRBRUSH_EMUL_ENABLE	0x1

#define AIRBRUSH_TRIMINFO_25_SHIFT	0
#define AIRBRUSH_TRIMINFO_85_SHIFT	8
#define AIRBRUSH_TMU_TRIP_MODE_SHIFT	13
#define AIRBRUSH_TMU_TRIP_MODE_MASK	0x7
#define AIRBRUSH_TMU_THERM_TRIP_EN_SHIFT	12

#define AIRBRUSH_TMU_INTEN_RISE0_SHIFT		0
#define AIRBRUSH_TMU_INTEN_RISE1_SHIFT		1
#define AIRBRUSH_TMU_INTEN_RISE2_SHIFT		2
#define AIRBRUSH_TMU_INTEN_RISE3_SHIFT		3
#define AIRBRUSH_TMU_INTEN_RISE4_SHIFT		4
#define AIRBRUSH_TMU_INTEN_RISE5_SHIFT		5
#define AIRBRUSH_TMU_INTEN_RISE6_SHIFT		6
#define AIRBRUSH_TMU_INTEN_RISE7_SHIFT		7

#define AIRBRUSH_THD_TEMP_FALL7_6	0x60

#define AIRBRUSH_ONE_POINT_TRIMMING	0
#define AIRBRUSH_TWO_POINT_TRIMMING	1

#define AIRBRUSH_TMU_BASE	0xB90000

#define MCELSIUS	1000

static int tmu_read_enable;

void tmu_write(u32 value, u32 offset)
{
	abc_pcie_config_write(offset, 4, value);
}

u32 tmu_read(u32 offset)
{
	u32 data;

	abc_pcie_config_read(offset, 4, &data);
	return data;
}

/**
 * struct airbrush_tmu_data : A structure to hold the private data of the TMU
	driver
 * @id: identifier of the one instance of the TMU controller.
 * @pdata: pointer to the tmu platform/configuration data
 * @base: base address of the single instance of the TMU controller.
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @irq_work: pointer to the irq work structure.
 * @lock: lock to implement synchronization.
 * @clk: pointer to the clock structure.
 * @clk_sec: pointer to the clock structure for accessing the base_second.
 * @sclk: pointer to the clock structure for accessing the tmu special clk.
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
	int id;
	struct airbrush_tmu_platform_data *pdata;
	u32 base;
	int irq;
	struct notifier_block  tmu_nb;
	struct work_struct irq_work;
	struct mutex lock;
	struct clk *clk, *clk_sec, *sclk;
	u16 temp_error1, temp_error2;
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
 * @cal_type: calibration type for temperature
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
	u8 first_point_trim;
	u8 second_point_trim;
	u8 default_temp_offset;

	u32 cal_type;
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
	unsigned int val_irq;

	val_irq = tmu_read(data->base + AIRBRUSH_TMU_REG_INTSTAT);
	tmu_write(val_irq, data->base + AIRBRUSH_TMU_REG_INTPEND);
}

static int airbrush_tmu_read(struct airbrush_tmu_data *data)
{
	return tmu_read(data->base + AIRBRUSH_TMU_REG_CURRENT_TEMP0_1) &
		AIRBRUSH_TMU_TEMP_MASK;
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

static void airbrush_tmu_core_enable(struct platform_device *pdev, bool enable)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int val;

	val = tmu_read(data->base + AIRBRUSH_TMU_REG_CONTROL);

	if (enable) {
		val |= (1 << AIRBRUSH_TMU_EN_TRIP_SHIFT);
		val |= (1 << AIRBRUSH_TMU_CORE_EN_SHIFT);
		tmu_write(val, data->base + AIRBRUSH_TMU_REG_CONTROL);
	} else {
		val &= ~(1 << AIRBRUSH_TMU_EN_TRIP_SHIFT);
		val &= ~(1 << AIRBRUSH_TMU_CORE_EN_SHIFT);
		tmu_write(val, data->base + AIRBRUSH_TMU_REG_CONTROL);
	}
}

static int airbrush_tmu_initialize(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	struct airbrush_tmu_platform_data *pdata = data->pdata;
	unsigned int status, trim_info;
	int ret = 0;

	status = tmu_read(data->base + AIRBRUSH_TMU_REG_STATUS);
	if (!status) {
		ret = -EBUSY;
		goto out;
	}

	trim_info = tmu_read(data->base + AIRBRUSH_TMU_REG_TRIMINFO);

	data->temp_error1 = trim_info & AIRBRUSH_TMU_TEMP_MASK;
	if (!data->temp_error1 ||
	    (pdata->min_efuse_value > data->temp_error1) ||
	    (data->temp_error1 > pdata->max_efuse_value))
		data->temp_error1 = pdata->efuse_value & AIRBRUSH_TMU_TEMP_MASK;

	airbrush_tmu_clear_irqs(data);
out:
	return ret;
}

static void airbrush_tmu_control(struct platform_device *pdev, bool on)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);

	airbrush_tmu_core_enable(pdev, 0);

	tmu_write(0xff007f, data->base + AIRBRUSH_TMU_REG_INTEN);
	tmu_write(0x1003, data->base + AIRBRUSH_TMU_REG_CONTROL);
	airbrush_tmu_core_enable(pdev, 1);
}

static int airbrush_get_temp(void *p, int *temp)
{
	struct airbrush_tmu_data *data = p;

	if (!data)
		return -EINVAL;
	if (tmu_read_enable)
		*temp = airbrush_tmu_read(data) * MCELSIUS;
	else
		*temp = 0;
	/* *temp = code_to_temp(data, airbrush_tmu_read(data)) * MCELSIUS; */
	return 0;
}

#ifdef CONFIG_THERMAL_EMULATION
void tmu_enable_emulate(void)
{
	unsigned int emul_con;

	emul_con = tmu_read(AIRBRUSH_TMU_BASE + AIRBRUSH_EMUL_CON);
	emul_con |= (1 << AIRBRUSH_EMUL_ENABLE_SHIFT);

	tmu_write(emul_con, AIRBRUSH_TMU_BASE + AIRBRUSH_EMUL_CON);
}

void tmu_disable_emulate(void)
{
	unsigned int emul_con;

	emul_con = tmu_read(AIRBRUSH_TMU_BASE + AIRBRUSH_EMUL_CON);
	emul_con &= ~(1 << AIRBRUSH_EMUL_ENABLE_SHIFT);

	tmu_write(emul_con, AIRBRUSH_TMU_BASE + AIRBRUSH_EMUL_CON);
}

void tmu_set_emulate_data(struct airbrush_tmu_data *data,  u16 next_time,
								u16 next_data)
{
	u32 uReg;

	next_time &= 0xFFFF;
	next_data &= 0x1FF;
	if (next_time == 0x0)
		next_time = 0x1;
	uReg = (tmu_read(AIRBRUSH_TMU_BASE + AIRBRUSH_EMUL_CON) >> 0) & 0x1;
	/* use temp_to_code() below once the triminfo is available */
	/* uReg |= (next_time << AIRBRUSH_EMUL_NEXTTIME_SHIFT) |
		(temp_to_code(data, next_data) << AIRBRUSH_EMUL_DATA_SHIFT); */
	uReg |= (next_time << AIRBRUSH_EMUL_NEXTTIME_SHIFT) |
				(next_data << AIRBRUSH_EMUL_DATA_SHIFT);

	tmu_write(uReg, AIRBRUSH_TMU_BASE + AIRBRUSH_EMUL_CON);
}

static int airbrush_tmu_set_emulation(void *drv_data, int temp)
{
	struct airbrush_tmu_data *data = drv_data;
	int ret = -EINVAL;

	if (temp && temp < MCELSIUS)
		goto out;

	if (temp)
		temp /= MCELSIUS;

	tmu_enable_emulate();
	tmu_set_emulate_data(data, 1, temp);
	/* tmu_disable_emulate();*/ /* Enable this once we have triminfo */

	return 0;
out:
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

	data->id = of_alias_get_id(pdev->dev.of_node, "tmuctrl");
	if (data->id < 0)
		data->id = 0;

	data->irq = 36;

	data->base = AIRBRUSH_TMU_BASE;

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
	pdata->cal_type = AIRBRUSH_ONE_POINT_TRIMMING;
	data->pdata = pdata;

	return 0;
}

static struct thermal_zone_of_device_ops airbrush_sensor_ops = {
	.get_temp = airbrush_get_temp,
	.set_emul_temp = airbrush_tmu_set_emulation,
};

static int airbrush_tmu_probe(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct airbrush_tmu_data),
					GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	/* TODO: See if this function can be remove */
	ret = airbrush_map_dt_data(pdev);
	if (ret)
		goto err;

	INIT_WORK(&data->irq_work, airbrush_tmu_work);

	/* TODO: get clock from dt */

	/*
	 * data->tzd must be registered before calling airbrush_tmu_initialize(),
	 * requesting irq and calling airbrush_tmu_control().
	 */
	data->tzd = thermal_zone_of_sensor_register(&pdev->dev, 0, data,
						    &airbrush_sensor_ops);
	if (IS_ERR(data->tzd)) {
		ret = PTR_ERR(data->tzd);
		dev_err(&pdev->dev, "Failed to register sensor: %d\n", ret);
		goto err_thermal;
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
		goto err_thermal;
	}

	/* set tmu contorl register */
	airbrush_tmu_control(pdev, true);
	tmu_read_enable = 1;

	dev_info(&pdev->dev, "%s: done.\n", __func__);

	return 0;

err_thermal:
	thermal_zone_of_sensor_unregister(&pdev->dev, data->tzd);
err:
	/* do something here */
	return ret;
}

static int airbrush_tmu_remove(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);
	struct thermal_zone_device *tzd = data->tzd;

	thermal_zone_of_sensor_unregister(&pdev->dev, tzd);
	airbrush_tmu_control(pdev, false);
	tmu_read_enable = 0;

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
