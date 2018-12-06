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
#include <linux/mfd/abc-pcie-notifier.h>
#include "../../thermal/thermal_core.h"

/* Exynos generic registers */
#define AIRBRUSH_TMU_REG_TRIMINFO	0x0
#define AIRBRUSH_TMU_REG_TRIMINFO_P(n)	((0x0) + ((n)*4))
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
#define AIRBRUSH_THD_TEMP_RISE7_6_P(n) \
		((n == 0) ? (0x050) : (0x170+(n-1)*0x20))
#define AIRBRUSH_THD_TEMP_RISE5_4		0x054
#define AIRBRUSH_THD_TEMP_RISE3_2		0x058
#define AIRBRUSH_THD_TEMP_RISE1_0		0x05C
#define AIRBRUSH_THD_TEMP_FALL7_6_P(n) \
		((n == 0) ? (0x060) : (0x180+(n-1)*0x20))
#define AIRBRUSH_TMU_REG_INTEN_P(n) \
		((n <= 4) ? (0x110+n*0x10) : (0x310+((n-5)*0x10)))
#define AIRBRUSH_TMU_REG_INTSTAT_P(n) \
		((n <= 4) ? (0x114+n*0x10) : (0x314+((n-5)*0x10)))
#define AIRBRUSH_TMU_REG_INTPEND_P(n) \
		((n <= 4) ? (0x118+n*0x10) : (0x318+((n-5)*0x10)))

#define AIRBRUSH_TMU_INT_EN			0xff01ff

#define AIRBRUSH_TMU_INTEN_FALL0_SHIFT		16

#define AIRBRUSH_TMU_CAL_MASK		0x3
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

#define AIRBRUSH_NO_TRIMMING	0
#define AIRBRUSH_ONE_POINT_TRIMMING	1
#define AIRBRUSH_TWO_POINT_TRIMMING	2

#define AIRBRUSH_TMU_BASE	0xB90000
#define AIRBRUSH_TEMP_PROBE_MAIN		0
#define AIRBRUSH_TEMP_PROBE_IPU0		1
#define AIRBRUSH_TEMP_PROBE_IPU1		2
#define AIRBRUSH_TEMP_PROBE_IPU2		3
#define AIRBRUSH_TEMP_PROBE_IPU_TPU		4
#define AIRBRUSH_TEMP_PROBE_TPU0		5
#define AIRBRUSH_TEMP_PROBE_TPU1		6
#define AIRBRUSH_NUM_REMOTE_PROBE		0x6
#define AIRBRUSH_REMOTE_PROBE_SHIFT		16
#define AIRBRUSH_NUM_ALL_PROBE			7
#define AIRBRUSH_TMU_CAL_SHIFT			18
#define AIRBRUSH_TMU_TEMP_SHIFT			9

#define MCELSIUS	1000

static const u16 no_trimming_error1[] = {287, 286, 287, 287, 286, 286, 286};
static const u16 no_trimming_error2[] = {346, 346, 347, 347, 347, 346, 346};

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
	int id;
	struct airbrush_tmu_platform_data *pdata;
	u32 base;
	int irq;
	struct notifier_block  tmu_nb;
	struct work_struct irq_work;
	struct clk *clk, *clk_sec, *sclk;
	u8 cal_type[AIRBRUSH_NUM_ALL_PROBE];
	u16 temp_error1[AIRBRUSH_NUM_ALL_PROBE];
	u16 temp_error2[AIRBRUSH_NUM_ALL_PROBE];
	struct regulator *regulator;
	struct thermal_zone_device *tzd;

	struct mutex pcie_link_lock;
	bool pcie_link_ready;
	struct notifier_block pcie_link_blocking_nb;
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

	for (i = 0; i < AIRBRUSH_NUM_ALL_PROBE; i++) {
		val_irq = tmu_read(data->base + AIRBRUSH_TMU_REG_INTPEND_P(i));
		tmu_write(val_irq, data->base + AIRBRUSH_TMU_REG_INTPEND_P(i));
	}
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
	unsigned int status, trim_info;
	int ret = 0;
	struct thermal_zone_device *tz = data->tzd;
	unsigned int rising_threshold = 0, falling_threshold = 0;
	int threshold_code, i, j;
	int temp, temp_hist;
	unsigned int reg_off, bit_off;

	status = tmu_read(data->base + AIRBRUSH_TMU_REG_STATUS);
	if (!status) {
		ret = -EBUSY;
		goto out;
	}

	for (i = 0; i < AIRBRUSH_NUM_ALL_PROBE; i++) {
		trim_info = tmu_read(data->base +
				     AIRBRUSH_TMU_REG_TRIMINFO_P(i));

		data->cal_type[i] = (trim_info >> AIRBRUSH_TMU_CAL_SHIFT) &
				    (AIRBRUSH_TMU_CAL_MASK);

		data->temp_error1[i] = trim_info & AIRBRUSH_TMU_TEMP_MASK;
		data->temp_error2[i] = (trim_info >> AIRBRUSH_TMU_TEMP_SHIFT) &
				       (AIRBRUSH_TMU_TEMP_MASK);

		if (data->cal_type[i] == AIRBRUSH_NO_TRIMMING) {
			data->temp_error1[i] = no_trimming_error1[i];
			data->temp_error2[i] = no_trimming_error2[i];
		}
	}

	for (i = (of_thermal_get_ntrips(tz) - 1); i >= 0; i--) {

		reg_off = ((7 - i) / 2) * 4;
		bit_off = ((8 - i) % 2);

		tz->ops->get_trip_temp(tz, i, &temp);
		temp /= MCELSIUS;

		tz->ops->get_trip_hyst(tz, i, &temp_hist);
		temp_hist = temp - (temp_hist / MCELSIUS);

		for (j = 0; j < AIRBRUSH_NUM_ALL_PROBE; j++) {
			/* Set 9-bit temp code for rising threshold levels */
			threshold_code = temp_to_code(data, temp, j);
			rising_threshold = tmu_read(data->base +
				AIRBRUSH_THD_TEMP_RISE7_6_P(j) + reg_off);
			rising_threshold &= ~(AIRBRUSH_TMU_TEMP_MASK <<
				(16 * bit_off));
			rising_threshold |= threshold_code << (16 * bit_off);

			/* Set 9-bit temp code for falling threshold levels */
			threshold_code = temp_to_code(data, temp_hist, j);
			falling_threshold = tmu_read(data->base +
				AIRBRUSH_THD_TEMP_FALL7_6_P(j) + reg_off);
			falling_threshold &= ~(AIRBRUSH_TMU_TEMP_MASK <<
				(16 * bit_off));
			falling_threshold |= threshold_code << (16 * bit_off);

			tmu_write(rising_threshold,
				data->base + AIRBRUSH_THD_TEMP_RISE7_6_P(j) +
				reg_off);
			tmu_write(falling_threshold,
				data->base + AIRBRUSH_THD_TEMP_FALL7_6_P(j) +
				reg_off);
		}
	}

	airbrush_tmu_clear_irqs(data);
out:
	return ret;
}

static void airbrush_tmu_control(struct platform_device *pdev, bool on)
{
	unsigned int i, con1, interrupt_en;
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);

	con1 = tmu_read(data->base + AIRBRUSH_TMU_REG_CONTROL1);
	con1 |= AIRBRUSH_NUM_REMOTE_PROBE << AIRBRUSH_REMOTE_PROBE_SHIFT;

	interrupt_en = on ? AIRBRUSH_TMU_INT_EN : 0;

	for (i = 0; i < AIRBRUSH_NUM_ALL_PROBE; i++)
		tmu_write(interrupt_en,
			data->base + AIRBRUSH_TMU_REG_INTEN_P(i));

	tmu_write(con1, data->base + AIRBRUSH_TMU_REG_CONTROL1);
	airbrush_tmu_core_enable(pdev, on);
}

static int airbrush_get_temp(void *p, int *temp)
{
	struct airbrush_tmu_data *data = p;

	if (!data)
		return -EINVAL;

	mutex_lock(&data->pcie_link_lock);
	/*
	 * Return 0 degree Celsius while TMU is not ready to suppress the
	 * error log while reading the temperature.
	 */
	*temp = (tmu_read_enable && data->pcie_link_ready) ?
		code_to_temp(data, airbrush_tmu_read(data), 0) * MCELSIUS:
		0;
	mutex_unlock(&data->pcie_link_lock);
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
	uReg |= (next_time << AIRBRUSH_EMUL_NEXTTIME_SHIFT) |
		(temp_to_code(data, next_data, 0) << AIRBRUSH_EMUL_DATA_SHIFT);

	tmu_write(uReg, AIRBRUSH_TMU_BASE + AIRBRUSH_EMUL_CON);
}

static int airbrush_tmu_set_emulation(void *drv_data, int temp)
{
	struct airbrush_tmu_data *data = drv_data;
	int ret;

	mutex_lock(&data->pcie_link_lock);
	if (data->pcie_link_ready) {
		if (temp != 0) {
			tmu_enable_emulate();
			tmu_set_emulate_data(data, 1, temp / MCELSIUS);
		} else {
			tmu_disable_emulate();
		}
		ret = 0;
	} else {
		ret = -ENODEV;
	}
	mutex_unlock(&data->pcie_link_lock);

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

	data->pdata = pdata;

	return 0;
}

static struct thermal_zone_of_device_ops airbrush_sensor_ops = {
	.get_temp = airbrush_get_temp,
	.set_emul_temp = airbrush_tmu_set_emulation,
};

static inline int airbrush_tmu_get_current_temp(struct airbrush_tmu_data *data,
		int id)
{
	u32 reg_offset, reg_id_min, reg_shift;
	u32 code;

	switch (id) {
	case 0:
	case 1:
		reg_offset = AIRBRUSH_TMU_REG_CURRENT_TEMP0_1;
		reg_id_min = 0;
		break;
	case 2:
	case 3:
	case 4:
		reg_offset = AIRBRUSH_TMU_REG_CURRENT_TEMP2_4;
		reg_id_min = 2;
		break;
	case 5:
	case 6:
	case 7:
		reg_offset = AIRBRUSH_TMU_REG_CURRENT_TEMP5_7;
		reg_id_min = 5;
		break;
	default:
		pr_warn("Bug: bad sensor probe id %d", id);
		return 0;
	}
	reg_shift = AIRBRUSH_TMU_TEMP_SHIFT * (id - reg_id_min);

	code = (tmu_read(data->base + reg_offset) >> reg_shift)
			& AIRBRUSH_TMU_TEMP_MASK;
	return code_to_temp(data, code, id);
}

static ssize_t temp_probe_show(struct device *dev, int id, char *buf)
{
	struct airbrush_tmu_data *data = dev_get_drvdata(dev);
	int temp = airbrush_tmu_get_current_temp(data, id);

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

static int airbrush_tmu_pcie_link_listener(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct airbrush_tmu_data *dev_data = container_of(nb,
			struct airbrush_tmu_data, pcie_link_blocking_nb);

	switch (action) {
	case ABC_PCIE_LINK_POST_ENABLE:
		mutex_lock(&dev_data->pcie_link_lock);
		dev_data->pcie_link_ready = true;
		mutex_unlock(&dev_data->pcie_link_lock);
		break;
	case ABC_PCIE_LINK_PRE_DISABLE:
		mutex_lock(&dev_data->pcie_link_lock);
		dev_data->pcie_link_ready = false;
		mutex_unlock(&dev_data->pcie_link_lock);
		break;
	default:
		return NOTIFY_DONE;  /* Don't care */
	}

	return NOTIFY_OK;
}

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

	mutex_init(&data->pcie_link_lock);
	mutex_lock(&data->pcie_link_lock);
	data->pcie_link_ready = false;
	data->pcie_link_blocking_nb.notifier_call =
			airbrush_tmu_pcie_link_listener;
	ret = abc_register_pcie_link_blocking_event(
			&data->pcie_link_blocking_nb);
	mutex_unlock(&data->pcie_link_lock);
	if (ret) {
		dev_err(&pdev->dev,
				"failed to subscribe to PCIe blocking link event, ret %d\n",
				ret);
		return ret;
	}

	/* tmu initialization */
	ret = airbrush_tmu_initialize(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize TMU\n");
		goto err;
	}

	airbrush_tmu_control(pdev, true);
	tmu_read_enable = 1;

	dev_dbg(&pdev->dev, "%s: done.\n", __func__);
	return 0;

err:
	abc_unregister_pcie_link_blocking_event(&data->pcie_link_blocking_nb);
	return ret;
}

static int airbrush_tmu_remove(struct platform_device *pdev)
{
	struct airbrush_tmu_data *data = platform_get_drvdata(pdev);

	abc_unregister_pcie_link_blocking_event(&data->pcie_link_blocking_nb);
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
