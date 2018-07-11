/*
 * Fuel gauge driver for Maxim 17201/17205
 *
 * Copyright (C) 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>

#define MAX1720X_TRECALL_MS 5
#define MAX1720X_TPOR_MS 150
#define MAX1720X_I2C_DRIVER_NAME "max1720x_fg_irq"
#define MAX1720X_N_OF_HISTORY_PAGES 203
#define MAX1720X_DELAY_INIT_MS 1000
#define FULLCAPNOM_STABILIZE_CYCLES 5

enum max1720x_register {
	/* ModelGauge m5 Register */
	MAX1720X_STATUS = 0x00,
	MAX1720X_VALRTTH = 0x01,
	MAX1720X_TALRTTH = 0x02,
	MAX1720X_SALRTTH = 0x03,
	MAX1720X_ATRATE = 0x04,
	MAX1720X_REPCAP = 0x05,
	MAX1720X_REPSOC = 0x06,
	MAX1720X_AGE = 0x07,
	MAX1720X_TEMP = 0x08,
	MAX1720X_VCELL = 0x09,
	MAX1720X_CURRENT = 0x0A,
	MAX1720X_AVGCURRENT = 0x0B,
	MAX1720X_QRESIDUAL = 0x0C,
	MAX1720X_MIXSOC = 0x0D,
	MAX1720X_AVSOC = 0x0E,
	MAX1720X_MIXCAP = 0x0F,

	MAX1720X_FULLCAP = 0x10,
	MAX1720X_TTE = 0x11,
	MAX1720X_QRTABLE00 = 0x12,
	MAX1720X_FULLSOCTHR = 0x13,
	MAX1720X_RCELL = 0x14,
	MAX1720X_RFAST = 0x15,
	MAX1720X_AVGTA = 0x16,
	MAX1720X_CYCLES = 0x17,
	MAX1720X_DESIGNCAP = 0x18,
	MAX1720X_AVGVCELL = 0x19,
	MAX1720X_MAXMINTEMP = 0x1A,
	MAX1720X_MAXMINVOLT = 0x1B,
	MAX1720X_MAXMINCURR = 0x1C,
	MAX1720X_CONFIG = 0x1D,
	MAX1720X_ICHGTERM = 0x1E,
	MAX1720X_AVCAP = 0x1F,

	MAX1720X_TTF = 0x20,
	MAX1720X_DEVNAME = 0x21,
	MAX1720X_QRTABLE10 = 0x22,
	MAX1720X_FULLCAPNOM = 0x23,
	MAX1720X_AIN0 = 0x27,
	MAX1720X_LEARNCFG = 0x28,
	MAX1720X_FILTERCFG = 0x29,
	MAX1720X_RELAXCFG = 0x2A,
	MAX1720X_MISCCFG = 0x2B,
	MAX1720X_TGAIN = 0x2C,
	Max1720x_TOff = 0x2D,
	MAX1720X_CGAIN = 0x2E,
	MAX1720X_COFF = 0x2F,

	MAX1720X_QRTABLE20 = 0x32,
	MAX1720X_FULLCAPREP = 0x35,
	MAX1720X_IAVGEMPTY = 0x36,
	MAX1720X_RCOMP0 = 0x38,
	MAX1720X_TEMPCO = 0x39,
	MAX1720X_VEMPTY = 0x3A,
	MAX1720X_FSTAT = 0x3D,
	MAX1720X_TIMER = 0x3E,
	MAX1720X_SHDNTIMER = 0x3F,

	MAX1720X_QRTABLE30 = 0x42,
	MAX1720X_DQACC = 0x45,
	MAX1720X_DPACC = 0x46,
	MAX1720X_VFREMCAP = 0x4A,
	MAX1720X_QH = 0x4D,

	MAX1720X_STATUS2 = 0xB0,
	MAX1720X_IALRTTH = 0xB4,
	MAX1720X_VSHDNCFG = 0xB8,
	MAX1720X_AGEFORECAST = 0xB9,
	MAX1720X_HIBCFG = 0xBA,
	MAX1720X_CONFIG2 = 0xBB,
	MAX1720X_VRIPPLE = 0xBC,
	MAX1720X_PACKCFG = 0xBD,
	MAX1720X_TIMERH = 0xBE,

	MAX1720X_AVGCELL4 = 0xD1,
	MAX1720X_AVGCELL3 = 0xD2,
	MAX1720X_AVGCELL2 = 0xD3,
	MAX1720X_AVGCELL1 = 0xD4,
	MAX1720X_CELL4 = 0xD5,
	MAX1720X_CELL3 = 0xD6,
	MAX1720X_CELL2 = 0xD7,
	MAX1720X_CELL1 = 0xD8,
	MAX1720X_CELLX = 0xD9,
	MAX1720X_BATT = 0xDA,
	MAX1720X_ATQRESIDUAL = 0xDC,
	MAX1720X_ATTTE = 0xDD,
	MAX1720X_ATAVSOC = 0xDE,
	MAX1720X_ATAVCAP = 0xDF,

	/* Individual Registers */
	MAX1720X_COMMAND = 0x60,
	MAX1720X_COMMSTAT = 0x61,
	MAX1720X_LOCK = 0x7F,
	MAX1720X_ODSCTH = 0xF2,
	MAX1720X_ODSCCFG = 0xF3,
	MAX1720X_VFOCV = 0xFB,
	MAX1720X_VFSOC = 0xFF,
};

enum max1720x_status_bits {
	MAX1720X_STATUS_POR = BIT(1),
	MAX1720X_STATUS_IMN = BIT(2),
	MAX1720X_STATUS_BST = BIT(3),
	MAX1720X_STATUS_IMX = BIT(6),
	MAX1720X_STATUS_DSOCI = BIT(7),
	MAX1720X_STATUS_VMN = BIT(8),
	MAX1720X_STATUS_TMN = BIT(9),
	MAX1720X_STATUS_SMN = BIT(10),
	MAX1720X_STATUS_BI = BIT(11),
	MAX1720X_STATUS_VMX = BIT(12),
	MAX1720X_STATUS_TMX = BIT(13),
	MAX1720X_STATUS_SMX = BIT(14),
	MAX1720X_STATUS_BR = BIT(15),
};

enum max1720x_commstat_bits {
	MAX1720X_COMMSTAT_NVBUSY = BIT(1),
	MAX1720X_COMMSTAT_NVERROR = BIT(2),
};

enum max1720x_config_bits {
	MAX1720X_CONFIG_BER = BIT(0),
	MAX1720X_CONFIG_BEI = BIT(1),
	MAX1720X_CONFIG_AEN = BIT(2),
	MAX1720X_CONFIG_FTHRM = BIT(3),
	MAX1720X_CONFIG_ETHRM = BIT(4),
	MAX1720X_CONFIG_COMMSH = BIT(6),
	MAX1720X_CONFIG_SHDN = BIT(7),
	MAX1720X_CONFIG_TEX = BIT(8),
	MAX1720X_CONFIG_TEN = BIT(9),
	MAX1720X_CONFIG_AINSH = BIT(10),
	MAX1720X_CONFIG_ALRTP = BIT(11),
	MAX1720X_CONFIG_VS = BIT(12),
	MAX1720X_CONFIG_TS = BIT(13),
	MAX1720X_CONFIG_SS = BIT(14),
};

enum max1720x_nnvcfg0_bits {
	MAX1720X_NNVCFG0_ENSBS = BIT(0),
	MAX1720X_NNVCFG0_ENHCFG = BIT(1),
	MAX1720X_NNVCFG0_ENAF = BIT(2),
	MAX1720X_NNVCFG0_ENMC = BIT(3),
	MAX1720X_NNVCFG0_ENDC = BIT(4),
	MAX1720X_NNVCFG0_ENVE = BIT(5),
	MAX1720X_NNVCFG0_ENCG = BIT(6),
	MAX1720X_NNVCFG0_ENICT = BIT(7),
	MAX1720X_NNVCFG0_ENLCFG = BIT(8),
	MAX1720X_NNVCFG0_ENRCFG = BIT(9),
	MAX1720X_NNVCFG0_ENFCFG = BIT(10),
	MAX1720X_NNVCFG0_ENCFG = BIT(11),
	MAX1720X_NNVCFG0_ENX = BIT(14),
	MAX1720X_NNVCFG0_ENOCV = BIT(15),
};

enum max1720x_command_bits {
	MAX1720X_COMMAND_FUEL_GAUGE_RESET = 0x0001,
	MAX1720X_COMMAND_HARDWARE_RESET = 0x000F,
	MAX1720X_COMMAND_QUERY_REMAINING_UPDATES = 0xE2FA,
	MAX1720X_COMMAND_COPY_NV_BLOCK = 0xE904,
	MAX1720X_COMMAND_HISTORY_RECALL_WRITE_0 = 0xE2FB,
	MAX1720X_COMMAND_HISTORY_RECALL_WRITE_1 = 0xE2FC,
	MAX1720X_COMMAND_HISTORY_RECALL_VALID_0 = 0xE2FC,
	MAX1720X_COMMAND_HISTORY_RECALL_VALID_1 = 0xE2FD,
	MAX1720X_COMMAND_HISTORY_RECALL_VALID_2 = 0xE2FE,
	MAX1720X_READ_HISTORY_CMD_BASE = 0xE226,
};

/** Nonvolatile Register Memory Map */
enum max1720x_nvram {
	MAX1720X_NVRAM_START = 0x80,
	MAX1720X_NUSER18C = 0x8C,
	MAX1720X_NUSER18D = 0x8D,
	MAX1720X_NODSCTH = 0x8E,
	MAX1720X_NODSCCFG = 0x8F,
	MAX1720X_NLEARNCFG = 0x9F,
	MAX1720X_NMISCCFG = 0xB2,
	MAX1720X_NHIBCFG = 0xB4,
	MAX1720X_NCONVGCFG = 0xB7,
	MAX1720X_NNVCFG0 = 0xB8,
	MAX1720X_NUSER1C4 = 0xC4,
	MAX1720X_NUSER1C5 = 0xC5,
	MAX1720X_NCGAIN = 0xC8,
	MAX1720X_NMANFCTRNAME0 = 0xCC,
	MAX1720X_NMANFCTRNAME1 = 0xCD,
	MAX1720X_NMANFCTRNAME2 = 0xCE,
	MAX1720X_NRSENSE = 0xCF,
	MAX1720X_NUSER1D0 = 0xD0,
	MAX1720X_NUSER1D1 = 0xD1,
	MAX1720X_NUSER1D4 = 0xD4,
	MAX1720X_NMANFCTRDATE = 0xD6,
	MAX1720X_NFIRSTUSED = 0xD7,
	MAX1720X_NSERIALNUMBER0 = 0xD8,
	MAX1720X_NSERIALNUMBER1 = 0xD9,
	MAX1720X_NSERIALNUMBER2 = 0xDA,
	MAX1720X_NDEVICENAME0 = 0xDB,
	MAX1720X_NDEVICENAME1 = 0xDC,
	MAX1720X_NDEVICENAME2 = 0xDD,
	MAX1720X_NDEVICENAME3 = 0xDE,
	MAX1720X_NDEVICENAME4 = 0xDF,
	MAX1720X_NVRAM_END = 0xE0,
	MAX1720X_NVRAM_HISTORY_WRITE_STATUS_START = 0xE1,
	MAX1720X_NVRAM_HISTORY_WRITE_STATUS_END = 0xEA,
	MAX1720X_NVRAM_HISTORY_VALID_STATUS_START = 0xEB,
	MAX1720X_NVRAM_HISTORY_VALID_STATUS_END = 0xE4,
	MAX1720X_NVRAM_REMAINING_UPDATES = 0xED,
	MAX1720X_NVRAM_HISTORY_END = 0xEF,
};

#define BUCKET_COUNT 10

const unsigned int max1720x_cycle_counter_addr[BUCKET_COUNT] = {
	MAX1720X_NODSCTH,
	MAX1720X_NODSCCFG,
	MAX1720X_NMISCCFG,
	MAX1720X_NHIBCFG,
	MAX1720X_NMANFCTRNAME1,
	MAX1720X_NMANFCTRNAME2,
	MAX1720X_NFIRSTUSED,
	MAX1720X_NDEVICENAME4,
	MAX1720X_NUSER1C4,
	MAX1720X_NUSER1C5,
};

#define NVRAM_U16_INDEX(reg) (reg - MAX1720X_NVRAM_START)
#define NVRAM_BYTE_INDEX(reg) (NVRAM_U16_INDEX(reg) * sizeof(u16))
#define NVRAM_BYTE_INDEX_FROM(reg, base) ((reg - base) * sizeof(u16))
#define MAX1720X_NVRAM_U16_SIZE NVRAM_U16_INDEX(MAX1720X_NVRAM_END)
#define MAX1720X_NVRAM_SIZE NVRAM_BYTE_INDEX(MAX1720X_NVRAM_END)

#define MAX1720X_HISTORY_PAGE_SIZE \
		(MAX1720X_NVRAM_HISTORY_END - MAX1720X_NVRAM_END + 1)

#define MAX1720X_N_OF_HISTORY_FLAGS_REG				\
	(MAX1720X_NVRAM_HISTORY_END -				\
		MAX1720X_NVRAM_HISTORY_WRITE_STATUS_START + 1 + \
		MAX1720X_NVRAM_HISTORY_WRITE_STATUS_END -	\
		MAX1720X_NVRAM_END + 1)
#define MAX1720X_N_OF_QRTABLES 4

struct max1720x_cyc_ctr_data {
	u16 count[BUCKET_COUNT];
	struct mutex lock;
	int prev_soc;
};

struct max1720x_chip {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work init_work;
	struct work_struct cycle_count_work;
	struct i2c_client *primary;
	struct i2c_client *secondary;
	struct regmap *regmap_nvram;
	struct device_node *batt_node;
	struct iio_channel *iio_ch;
	struct max1720x_cyc_ctr_data cyc_ctr;
	u16 RSense;
	u16 RConfig;
	bool init_complete;
	bool resume_complete;
	u16 *history;
	int history_count;
	int history_index;
	u16 status;
	int fake_capacity;
	int fake_temperature;
	int previous_qh;
	int current_capacity;
	int prev_charge_state;
	char serial_number[25];
	bool offmode_charger;
};

static inline int max1720x_regmap_read(struct regmap *map,
				       unsigned int reg,
				       u16 *val,
				       const char *name)
{
	unsigned int tmp;
	int rtn = regmap_read(map, reg, &tmp);

	if (rtn)
		pr_err("Failed to read %s\n", name);
	else
		*val = tmp;

	return rtn;

}
#define REGMAP_READ(regmap, what, dst) \
	max1720x_regmap_read(regmap, what, dst, #what)

#define REGMAP_WRITE(regmap, what, data)			\
	do {							\
		int rtn;					\
		rtn = regmap_write(regmap, what, data);		\
		if (rtn) {					\
			pr_err("Failed to write %s\n", #what);	\
		}						\
	} while (0)

static char *psy_status_str[] = {
	"Unknown", "Charging", "Discharging", "NotCharging", "Full"
};

bool max1720x_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX1720X_COMMAND:
	case MAX1720X_COMMSTAT:
	case MAX1720X_LOCK:
	case MAX1720X_ODSCTH:
	case MAX1720X_ODSCCFG:
	case MAX1720X_VFOCV:
	case MAX1720X_VFSOC:
	case 0x00 ... 0x4F:
	case 0xB0 ... 0xDF:
		return true;
	}
	return false;
}

static const struct regmap_config max1720x_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1720X_VFSOC,
	.readable_reg = max1720x_is_reg,
	.volatile_reg = max1720x_is_reg,

};

bool max1720x_is_nvram_reg(struct device *dev, unsigned int reg)
{
	return (reg >= MAX1720X_NVRAM_START
		&& reg <= MAX1720X_NVRAM_HISTORY_END);
}

static const struct regmap_config max1720x_regmap_nvram_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1720X_NVRAM_HISTORY_END,
	.readable_reg = max1720x_is_nvram_reg,
	.volatile_reg = max1720x_is_nvram_reg,
};

static inline int reg_to_percentage(u16 val)
{
	/* LSB: 1/256% */
	return val >> 8;
}

static inline int reg_to_twos_comp_int(u16 val)
{
	/* Convert u16 to twos complement  */
	return -(val & 0x8000) + (val & 0x7FFF);
}

static inline int reg_to_micro_amp_h(u16 val, u16 rsense)
{
	/* LSB: 5.0μVh/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((u64) val * 500000, rsense);
}

static inline int reg_to_micro_volt(u16 val)
{
	/* LSB: 0.078125mV */
	return div_u64((u64) val * 78125, 1000);
}

static inline int reg_to_micro_amp(s16 val, u16 rsense)
{
	/* LSB: 1.5625μV/RSENSE ; Rsense LSB is 10μΩ */
	return div_s64((s64) val * 156250, rsense);
}

static inline int reg_to_deci_deg_cel(s16 val)
{
	/* LSB: 1/256°C */
	return div_s64((s64) val * 10, 256);
}

static inline int reg_to_resistance_micro_ohms(s16 val, u16 rsense)
{
	/* LSB: 1/4096 Ohm */
	return div_s64((s64) val * 1000 * rsense, 4096);
}

static inline int reg_to_cycles(s16 val)
{
	/* LSB: 16% of one cycle */
	return DIV_ROUND_CLOSEST((int) val * 16, 100);
}

static inline int reg_to_seconds(s16 val)
{
	/* LSB: 5.625 seconds */
	return DIV_ROUND_CLOSEST((int) val * 5625, 1000);
}

static void max1720x_read_log_write_status(struct max1720x_chip *chip,
					   u16 *buffer)
{
	int i;
	u16 data = 0;

	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_WRITE_0);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_NVRAM_HISTORY_WRITE_STATUS_START;
	     i <= MAX1720X_NVRAM_HISTORY_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_WRITE_1);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_NVRAM_END;
	     i <= MAX1720X_NVRAM_HISTORY_WRITE_STATUS_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
}

static void max1720x_read_log_valid_status(struct max1720x_chip *chip,
					   u16 *buffer)
{
	int i;
	u16 data = 0;

	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_VALID_0);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_NVRAM_HISTORY_VALID_STATUS_START;
	     i <= MAX1720X_NVRAM_HISTORY_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_VALID_1);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_NVRAM_END; i <= MAX1720X_NVRAM_HISTORY_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_HISTORY_RECALL_VALID_2);
	msleep(MAX1720X_TRECALL_MS);
	for (i = MAX1720X_NVRAM_END;
	     i <= MAX1720X_NVRAM_HISTORY_VALID_STATUS_END; i++) {
		(void) REGMAP_READ(chip->regmap_nvram, i, &data);
		*buffer++ = data;
	}
}

static int get_battery_history_status(struct max1720x_chip *chip,
				      bool *page_status)
{
	u16 *write_status, *valid_status;
	int i, addr_offset, bit_offset;
	int valid_history_entry_count = 0;

	write_status = kmalloc_array(MAX1720X_N_OF_HISTORY_FLAGS_REG,
				     sizeof(u16), GFP_KERNEL);
	if (!write_status)
		return -ENOMEM;

	valid_status = kmalloc_array(MAX1720X_N_OF_HISTORY_FLAGS_REG,
				     sizeof(u16), GFP_KERNEL);
	if (!valid_status) {
		kfree(write_status);
		return -ENOMEM;
	}

	max1720x_read_log_write_status(chip, write_status);
	max1720x_read_log_valid_status(chip, valid_status);

	/* Figure out the pages with valid history entry */
	for (i = 0; i < MAX1720X_N_OF_HISTORY_PAGES; i++) {
		addr_offset = i / 8;
		bit_offset = i % 8;
		page_status[i] =
		    ((write_status[addr_offset] & BIT(bit_offset)) ||
		     (write_status[addr_offset] & BIT(bit_offset + 8))) &&
		    ((valid_status[addr_offset] & BIT(bit_offset)) ||
		     (valid_status[addr_offset] & BIT(bit_offset + 8)));
		if (page_status[i])
			valid_history_entry_count++;
	}
	chip->history_count = valid_history_entry_count;
	kfree(write_status);
	kfree(valid_status);
	return 0;
}

static void get_battery_history(struct max1720x_chip *chip,
				bool *page_status, u16 *history)
{
	int i, j, index = 0;
	u16 data = 0;

	for (i = 0; i < MAX1720X_N_OF_HISTORY_PAGES; i++) {
		if (!page_status[i])
			continue;
		REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
			     MAX1720X_READ_HISTORY_CMD_BASE + i);
		msleep(MAX1720X_TRECALL_MS);
		for (j = 0; j < MAX1720X_HISTORY_PAGE_SIZE; j++) {
			(void) REGMAP_READ(chip->regmap_nvram,
					   MAX1720X_NVRAM_END + j,
					   &data);
			history[index * MAX1720X_HISTORY_PAGE_SIZE + j] = data;
		}
		index++;
	}
}

static ssize_t max1720x_history_count_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{

	struct power_supply *psy;
	struct max1720x_chip *chip;
	int length, ret;
	bool *page_status;

	psy = container_of(dev, struct power_supply, dev);
	chip = power_supply_get_drvdata(psy);

	page_status = kcalloc(MAX1720X_N_OF_HISTORY_PAGES,
			      sizeof(bool), GFP_KERNEL);
	if (!page_status)
		return -ENOMEM;

	ret = get_battery_history_status(chip, page_status);
	if (ret) {
		kfree(page_status);
		return ret;
	}

	length = scnprintf(buf, PAGE_SIZE, "%i\n", chip->history_count);
	kfree(page_status);
	return length;
}

static const DEVICE_ATTR(history_count, 0444,
			 max1720x_history_count_show, NULL);

static ssize_t max1720x_history_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy;
	struct max1720x_chip *chip;
	int i, ret;
	int length = 0;
	bool *page_status = NULL;

	psy = container_of(dev, struct power_supply, dev);
	chip = power_supply_get_drvdata(psy);

	/*
	 * Total history size can be up to 6.7KB, which is greater than
	 * PAGE_SIZE, which is 4K. Complete history may need to be queried
	 * in two calls.
	 * If chip->histry is NULL, the history needs to be recalled from
	 * fuel gauge. Otherwise, no recall is needed. Simply return the
	 * rest of the history.
	 */
	if (chip->history == NULL) {
		chip->history_index = 0;
		page_status = kcalloc(MAX1720X_N_OF_HISTORY_PAGES,
				      sizeof(bool), GFP_KERNEL);
		if (!page_status)
			return -ENOMEM;

		ret = get_battery_history_status(chip, page_status);
		if (ret) {
			kfree(page_status);
			return ret;
		}

		if (chip->history_count == 0) {
			dev_info(chip->dev,
				 "No battery history has been recorded\n");
			kfree(page_status);
			return 0;
		}
		chip->history = kmalloc_array(chip->history_count *
					      MAX1720X_HISTORY_PAGE_SIZE,
					      sizeof(u16), GFP_KERNEL);
		if (!chip->history) {
			kfree(page_status);
			return -ENOMEM;
		}

		get_battery_history(chip, page_status, chip->history);
	}

	/*
	 * The 81 in the next line is arrived with 16*5: one history log
	 * is 16 of u16. We print each u16 to 4 chars and we leave one space
	 * between the hex data in the buffer. And lastly we need a new line
	 * for every history log. (16x4 + 16x1 + 1) = 81.
	 */
	for (; (chip->history_index < chip->history_count)
	     && (length < (PAGE_SIZE - 81)); chip->history_index++) {
		for (i = 0; i < MAX1720X_HISTORY_PAGE_SIZE; i++) {
			length += scnprintf(buf + length,
				PAGE_SIZE - length, "%04x ",
				chip->history[chip->history_index *
					      MAX1720X_HISTORY_PAGE_SIZE + i]);
		}
		length += scnprintf(buf + length, PAGE_SIZE - length, "\n");
	}
	if (chip->history_index >= chip->history_count) {
		kfree(chip->history);
		kfree(page_status);
		chip->history = NULL;
		chip->history_index = 0;
		chip->history_count = 0;
	}

	return length;
}

static const DEVICE_ATTR(history, 0444, max1720x_history_show, NULL);

static enum power_supply_property max1720x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static void max1720x_cycle_count_work(struct work_struct *work)
{
	int bucket, cnt, batt_soc;
	u16 data;
	struct max1720x_chip *chip = container_of(work, struct max1720x_chip,
						  cycle_count_work);

	if (REGMAP_READ(chip->regmap, MAX1720X_REPSOC, &data))
		return;
	batt_soc = reg_to_percentage(data);

	mutex_lock(&chip->cyc_ctr.lock);

	if (chip->cyc_ctr.prev_soc != -1 &&
	    batt_soc >= 0 && batt_soc <= 100 &&
	    batt_soc > chip->cyc_ctr.prev_soc) {
		for (cnt = batt_soc ; cnt > chip->cyc_ctr.prev_soc ; cnt--) {
			bucket = cnt * BUCKET_COUNT / 100;
			if (bucket >= BUCKET_COUNT)
				bucket = BUCKET_COUNT - 1;
			chip->cyc_ctr.count[bucket]++;
			REGMAP_WRITE(chip->regmap_nvram,
				     max1720x_cycle_counter_addr[bucket],
				     chip->cyc_ctr.count[bucket]);
			pr_debug("Stored count: prev_soc=%d, soc=%d bucket=%d count=%d\n",
				 chip->cyc_ctr.prev_soc, cnt, bucket,
				 chip->cyc_ctr.count[bucket]);
		}
	}
	chip->cyc_ctr.prev_soc = batt_soc;

	mutex_unlock(&chip->cyc_ctr.lock);
}

static void max1720x_restore_cycle_counter(struct max1720x_chip *chip)
{
	int i;
	u16 data = 0;

	mutex_lock(&chip->cyc_ctr.lock);

	for (i = 0; i < BUCKET_COUNT; i++) {
		if (!REGMAP_READ(chip->regmap_nvram,
				 max1720x_cycle_counter_addr[i],
				 &data)) {
			chip->cyc_ctr.count[i] = data;
			pr_debug("max1720x_cycle_counter[%d], addr=0x%02X, count=%d\n",
				 i, max1720x_cycle_counter_addr[i],
				 chip->cyc_ctr.count[i]);
		}
	}
	chip->cyc_ctr.prev_soc = -1;

	mutex_unlock(&chip->cyc_ctr.lock);
}

static ssize_t max1720x_get_cycle_counts_bins(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct power_supply *psy;
	struct max1720x_chip *chip;
	int len = 0, i;

	psy = container_of(dev, struct power_supply, dev);
	chip = power_supply_get_drvdata(psy);

	mutex_lock(&chip->cyc_ctr.lock);

	for (i = 0; i < BUCKET_COUNT; i++) {

		len += scnprintf(buf + len, PAGE_SIZE - len, "%d",
				 chip->cyc_ctr.count[i]);

		if (i == BUCKET_COUNT-1)
			len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		else
			len += scnprintf(buf + len, PAGE_SIZE - len, " ");
	}

	mutex_unlock(&chip->cyc_ctr.lock);

	return len;
}

static ssize_t max1720x_set_cycle_counts_bins(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct power_supply *psy;
	struct max1720x_chip *chip;
	int val[BUCKET_COUNT], i;

	psy = container_of(dev, struct power_supply, dev);
	chip = power_supply_get_drvdata(psy);

	if (sscanf(buf, "%d %d %d %d %d %d %d %d %d %d",
		   &val[0], &val[1], &val[2], &val[3], &val[4],
		   &val[5], &val[6], &val[7], &val[8], &val[9]) != BUCKET_COUNT)
		return -EINVAL;

	mutex_lock(&chip->cyc_ctr.lock);
	for (i = 0; i < BUCKET_COUNT; i++) {
		if (val[i] >= 0 && val[i] < U16_MAX) {
			chip->cyc_ctr.count[i] = val[i];
			REGMAP_WRITE(chip->regmap_nvram,
				     max1720x_cycle_counter_addr[i],
				     chip->cyc_ctr.count[i]);
		}
	}
	mutex_unlock(&chip->cyc_ctr.lock);

	return count;
}

static DEVICE_ATTR(cycle_counts_bins, 0660,
		   max1720x_get_cycle_counts_bins,
		   max1720x_set_cycle_counts_bins);

static ssize_t max1720x_get_offmode_charger(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max1720x_chip *chip = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%hhd\n", chip->offmode_charger);
}

static ssize_t max1720x_set_offmode_charger(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max1720x_chip *chip = power_supply_get_drvdata(psy);

	if (kstrtobool(buf, &chip->offmode_charger))
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(offmode_charger, 0660,
		   max1720x_get_offmode_charger,
		   max1720x_set_offmode_charger);

static int max1720x_get_battery_soc(struct max1720x_chip *chip)
{
	u16 data;
	int capacity, err;

	if (chip->fake_capacity >= 0 && chip->fake_capacity <= 100)
		return chip->fake_capacity;

	err = REGMAP_READ(chip->regmap, MAX1720X_REPSOC, &data);
	if (err)
		return err;
	capacity = reg_to_percentage(data);

	if (capacity == 100 && chip->offmode_charger)
		chip->fake_capacity = 100;

	return capacity;
}

static void max1720x_prime_battery_qh_capacity(struct max1720x_chip *chip,
					       int status)
{
	u16 data = 0;

	(void) REGMAP_READ(chip->regmap, MAX1720X_MIXCAP, &data);
	chip->current_capacity = data;

	REGMAP_WRITE(chip->regmap_nvram, MAX1720X_NUSER18C, ~data);
	dev_info(chip->dev, "Capacity primed to %d on %s\n",
		 data, psy_status_str[status]);

	(void) REGMAP_READ(chip->regmap, MAX1720X_QH, &data);
	chip->previous_qh = reg_to_twos_comp_int(data);

	REGMAP_WRITE(chip->regmap_nvram, MAX1720X_NUSER18D, data);
	dev_info(chip->dev, "QH primed to %d on %s\n",
		 data, psy_status_str[status]);
}

static int max1720x_get_battery_status(struct max1720x_chip *chip)
{
	u16 data = 0;
	int current_avg, ichgterm, fullsocthr;
	int status = POWER_SUPPLY_STATUS_UNKNOWN, err;

	err = REGMAP_READ(chip->regmap, MAX1720X_AVGCURRENT, &data);
	if (err)
		return err;
	current_avg = -reg_to_micro_amp(data, chip->RSense);

	err = REGMAP_READ(chip->regmap, MAX1720X_ICHGTERM, &data);
	if (err)
		return err;
	ichgterm = reg_to_micro_amp(data, chip->RSense);

	err = REGMAP_READ(chip->regmap, MAX1720X_FULLSOCTHR, &data);
	if (err)
		return err;
	fullsocthr = reg_to_percentage(data);

	if (current_avg < -ichgterm) {
		status = POWER_SUPPLY_STATUS_CHARGING;
		if (chip->prev_charge_state == POWER_SUPPLY_STATUS_DISCHARGING)
			max1720x_prime_battery_qh_capacity(chip, status);
		chip->prev_charge_state = POWER_SUPPLY_STATUS_CHARGING;
	} else if (current_avg <= 0 &&
		 max1720x_get_battery_soc(chip) >= fullsocthr) {
		status = POWER_SUPPLY_STATUS_FULL;
		if (chip->prev_charge_state != POWER_SUPPLY_STATUS_FULL)
			max1720x_prime_battery_qh_capacity(chip, status);
		chip->prev_charge_state = POWER_SUPPLY_STATUS_FULL;
	} else {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		chip->prev_charge_state = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	return status;
}

static int max1720x_get_battery_health(struct max1720x_chip *chip)
{
	/* For health report what ever was recently alerted and clear it */

	if (chip->status & MAX1720X_STATUS_VMX) {
		chip->status &= ~MAX1720X_STATUS_VMX;
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}

	if (chip->status & MAX1720X_STATUS_TMN) {
		chip->status &= ~MAX1720X_STATUS_TMN;
		return POWER_SUPPLY_HEALTH_COLD;
	}

	if (chip->status & MAX1720X_STATUS_TMX) {
		chip->status &= ~MAX1720X_STATUS_TMX;
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	}

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int max1720x_set_battery_soc(struct max1720x_chip *chip,
				    const union power_supply_propval *val)
{
	chip->fake_capacity = val->intval;

	if (chip->psy)
		power_supply_changed(chip->psy);

	return 0;
}

static int max1720x_set_battery_temp(struct max1720x_chip *chip,
				     const union power_supply_propval *val)
{
	chip->fake_temperature = val->intval;

	if (chip->psy)
		power_supply_changed(chip->psy);

	return 0;
}

static int max1720x_update_battery_qh_based_capacity(struct max1720x_chip *chip)
{
	u16 data;
	int current_qh, err = 0;

	err = REGMAP_READ(chip->regmap, MAX1720X_QH, &data);
	if (err)
		return err;
	current_qh = reg_to_twos_comp_int(data);

	/* QH value accumulates as battery charges */
	chip->current_capacity -= (chip->previous_qh - current_qh);
	chip->previous_qh = current_qh;

	return 0;
}

static void max1720x_restore_battery_qh_capacity(struct max1720x_chip *chip)
{
	u16 data = 0, nvram_capacity;
	int current_qh, nvram_qh;

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NUSER18C, &data);
	nvram_capacity = ~data;

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NUSER18D, &data);
	nvram_qh = reg_to_twos_comp_int(data);

	(void) REGMAP_READ(chip->regmap, MAX1720X_QH, &data);
	current_qh = reg_to_twos_comp_int(data);

	/* QH value accumulates as battery discharges */
	chip->current_capacity = (int) nvram_capacity - (nvram_qh - current_qh);
	dev_info(chip->dev, "Capacity restored to %d\n",
		 chip->current_capacity);
	chip->previous_qh = current_qh;
	dev_info(chip->dev, "QH value restored to %d\n",
		 chip->previous_qh);
}

static int max1720x_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct max1720x_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	u16 data = 0;
	int err = 0;

	pm_runtime_get_sync(chip->dev);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max1720x_get_battery_status(chip);
		if (val->intval < 0)
			return val->intval;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max1720x_get_battery_health(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max1720x_get_battery_soc(chip);
		if (val->intval < 0)
			return val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		err = max1720x_update_battery_qh_based_capacity(chip);
		if (err < 0)
			return err;
		val->intval = reg_to_micro_amp_h(chip->current_capacity,
						 chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		/*
		 * Snap charge_full to DESIGNCAP during early charge cycles to
		 * prevent large fluctuations in FULLCAPNOM. MAX1720X_CYCLES LSB
		 * is 16%
		 */
		err = REGMAP_READ(map, MAX1720X_CYCLES, &data);
		if (!err) {
			if (reg_to_cycles(data) <= FULLCAPNOM_STABILIZE_CYCLES)
				err = REGMAP_READ(map, MAX1720X_DESIGNCAP,
						  &data);
			else
				err = REGMAP_READ(map, MAX1720X_FULLCAPNOM,
						  &data);
			val->intval = reg_to_micro_amp_h(data, chip->RSense);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		err = REGMAP_READ(map, MAX1720X_DESIGNCAP, &data);
		val->intval = reg_to_micro_amp_h(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		err = REGMAP_READ(map, MAX1720X_AVGCURRENT, &data);
		/* current is positive value when flowing to device */
		val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		err = REGMAP_READ(map, MAX1720X_CURRENT, &data);
		/* current is positive value when flowing to device */
		val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		err = REGMAP_READ(map, MAX1720X_CYCLES, &data);
		val->intval = reg_to_cycles(data);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		err = REGMAP_READ(map, MAX1720X_STATUS, &data);
		val->intval = (((u16) data) & MAX1720X_STATUS_BST) ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		err = REGMAP_READ(map, MAX1720X_RCELL, &data);
		val->intval = reg_to_resistance_micro_ohms(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (chip->fake_temperature != -EINVAL)
			val->intval = chip->fake_temperature;
		else {
			err = REGMAP_READ(map, MAX1720X_TEMP, &data);
			val->intval = reg_to_deci_deg_cel(data);
		}
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		err = REGMAP_READ(map, MAX1720X_TTE, &data);
		val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		err = REGMAP_READ(map, MAX1720X_TTF, &data);
		val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		err = REGMAP_READ(map, MAX1720X_AVGVCELL, &data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		err = REGMAP_READ(map, MAX1720X_MAXMINVOLT, &data);
		/* LSB: 20mV */
		val->intval = ((data >> 8) & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		err = REGMAP_READ(map, MAX1720X_MAXMINVOLT, &data);
		/* LSB: 20mV */
		val->intval = (data & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		err = REGMAP_READ(map, MAX1720X_VCELL, &data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		err = REGMAP_READ(map, MAX1720X_VFOCV, &data);
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = chip->serial_number;
		break;
	default:
		return -EINVAL;
	}
	if (err < 0)
		return err;
	return 0;
}

static int max1720x_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct max1720x_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	pm_runtime_get_sync(chip->dev);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = max1720x_set_battery_soc(chip, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		rc = max1720x_set_battery_temp(chip, val);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max1720x_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_TEMP:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * A fuel gauge reset resets only the fuel gauge operation without resetting IC
 * hardware. This is useful for testing different configurations without writing
 * nonvolatile memory.
 */
static void max1720x_fg_reset(struct max1720x_chip *chip)
{
	REGMAP_WRITE(chip->regmap, MAX1720X_CONFIG2,
		     MAX1720X_COMMAND_FUEL_GAUGE_RESET);
	msleep(MAX1720X_TPOR_MS);
}

static irqreturn_t max1720x_fg_irq_thread_fn(int irq, void *obj)
{
	struct max1720x_chip *chip = obj;
	u16 fg_status;
	int err = 0;

	if (!chip || irq != chip->primary->irq) {
		WARN_ON_ONCE(1);
		return IRQ_NONE;
	}

	pm_runtime_get_sync(chip->dev);
	if (!chip->init_complete || !chip->resume_complete) {
		pm_runtime_put_sync(chip->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(chip->dev);
	err = REGMAP_READ(chip->regmap, MAX1720X_STATUS, &fg_status);
	if (err)
		return IRQ_NONE;
	if (fg_status == 0)
		return IRQ_HANDLED;

	chip->status |= fg_status;
	if (fg_status & MAX1720X_STATUS_POR) {
		fg_status &= ~MAX1720X_STATUS_POR;
		pr_debug("POR is set\n");
	}
	if (fg_status & MAX1720X_STATUS_IMN)
		pr_debug("IMN is set\n");

	if (fg_status & MAX1720X_STATUS_BST)
		pr_debug("BST is set\n");

	if (fg_status & MAX1720X_STATUS_IMX)
		pr_debug("IMX is set\n");

	if (fg_status & MAX1720X_STATUS_DSOCI) {
		fg_status &= ~MAX1720X_STATUS_DSOCI;
		pr_debug("DSOCI is set\n");
		schedule_work(&chip->cycle_count_work);
	}
	if (fg_status & MAX1720X_STATUS_VMN) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status &= ~MAX1720X_STATUS_VMN;
		pr_debug("VMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_TMN) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status &= ~MAX1720X_STATUS_TMN;
		pr_debug("TMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_SMN) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status &= ~MAX1720X_STATUS_SMN;
		pr_debug("SMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_BI)
		pr_debug("BI is set\n");

	if (fg_status & MAX1720X_STATUS_VMX) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status &= ~MAX1720X_STATUS_VMX;
		pr_debug("VMX is set\n");
	}
	if (fg_status & MAX1720X_STATUS_TMX) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status &= ~MAX1720X_STATUS_TMX;
		pr_debug("TMX is set\n");
	}
	if (fg_status & MAX1720X_STATUS_SMX) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status &= ~MAX1720X_STATUS_SMX;
		pr_debug("SMX is set\n");
	}

	if (fg_status & MAX1720X_STATUS_BR)
		pr_debug("BR is set\n");

	REGMAP_WRITE(chip->regmap, MAX1720X_STATUS, fg_status);
	if (chip->psy)
		power_supply_changed(chip->psy);

	return IRQ_HANDLED;
}


static int max1720x_handle_dt_batt_id(struct max1720x_chip *chip)
{
	int ret, batt_id;
	u32 batt_id_range = 20, batt_id_kohm;
	const char *dt_iio_ch_name, *iio_ch_name;
	struct device_node *node = chip->dev->of_node;
	struct device_node *config_node, *child_node;

	ret = of_property_read_string(node, "io-channel-names",
				      &dt_iio_ch_name);
	if (ret == -EINVAL)
		return 0;

	if (ret) {
		dev_warn(chip->dev, "failed to read io-channel-names\n");
		/* Don't fail probe on that, just ignore error */
		return 0;
	}

	iio_ch_name = devm_kstrdup(chip->dev, dt_iio_ch_name, GFP_KERNEL);
	if (!iio_ch_name)
		/* Don't fail probe on that, just ignore error */
		return 0;

	chip->iio_ch = iio_channel_get(chip->dev, iio_ch_name);
	if (PTR_ERR(chip->iio_ch) == -EPROBE_DEFER) {
		dev_warn(chip->dev, "iio_channel_get %s not ready\n",
			 iio_ch_name);
		return -EPROBE_DEFER;
	} else if (IS_ERR(chip->iio_ch)) {
		dev_warn(chip->dev, "iio_channel_get %s error: %ld\n",
			 iio_ch_name, PTR_ERR(chip->iio_ch));
		/* Don't fail probe on that, just ignore error */
		return 0;
	}

	ret = iio_read_channel_processed(chip->iio_ch, &batt_id);
	if (ret < 0) {
		dev_warn(chip->dev, "Failed to read battery id: %d\n", ret);
		return 0;
	}

	batt_id /= 1000;
	dev_info(chip->dev, "device battery RID: %d\n", batt_id);

	ret = of_property_read_u32(node, "maxim,batt-id-range-pct",
				   &batt_id_range);
	if (ret && ret == -EINVAL)
		dev_warn(chip->dev, "failed to read maxim,batt-id-range-pct\n");

	config_node = of_find_node_by_name(node, "maxim,config");
	if (!config_node) {
		dev_warn(chip->dev, "Failed to find maxim,config setting\n");
		return 0;
	}

	for_each_child_of_node(config_node, child_node) {
		ret = of_property_read_u32(child_node, "maxim,batt-id-kohm",
					   &batt_id_kohm);
		if (!ret &&
		    (batt_id < (batt_id_kohm * (100 + batt_id_range) / 100)) &&
		    (batt_id > (batt_id_kohm * (100 - batt_id_range) / 100))) {
			chip->batt_node = child_node;
			break;
		}
	}
	if (!chip->batt_node)
		dev_warn(chip->dev, "No child node found matching ID\n");

	return 0;
}

static int max1720x_apply_regval_shadow(struct max1720x_chip *chip,
					struct device_node *node,
					u16 *nRAM, int nb)
{
	int ret, idx;
	u16 *regs;

	if (!node || nb <= 0)
		return 0;

	if (nb & 1) {
		dev_warn(chip->dev, "%s maxim,n_regval u16 elems count is not even: %d\n",
			 node->name, nb);
		return -EINVAL;
	}

	regs = kmalloc_array(nb, sizeof(u16), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	ret = of_property_read_u16_array(node, "maxim,n_regval", regs, nb);
	if (ret) {
		dev_warn(chip->dev, "failed to read maxim,n_regval: %d\n", ret);
		goto shadow_out;
	}

	for (idx = 0; idx < nb; idx += 2) {
		if ((regs[idx] >= MAX1720X_NVRAM_START) &&
		    (regs[idx] < MAX1720X_NVRAM_END)) {
			nRAM[regs[idx] - MAX1720X_NVRAM_START] = regs[idx + 1];
		}
	}
shadow_out:
	kfree(regs);
	return ret;
}

static int max1720x_handle_dt_shadow_config(struct max1720x_chip *chip)
{
	int ret = 0;
	u16 *nRAM_current, *nRAM_updated;
	int batt_cnt = 0, glob_cnt;

	ret = max1720x_handle_dt_batt_id(chip);
	if (ret)
		return ret;

	if (chip->batt_node)
		batt_cnt = of_property_count_elems_of_size(chip->batt_node,
							   "maxim,n_regval",
							   sizeof(u16));
	glob_cnt = of_property_count_elems_of_size(chip->dev->of_node,
						   "maxim,n_regval",
						   sizeof(u16));

	nRAM_current = kmalloc_array(MAX1720X_NVRAM_U16_SIZE,
				     sizeof(u16), GFP_KERNEL);
	if (!nRAM_current)
		return -ENOMEM;

	nRAM_updated = kmalloc_array(MAX1720X_NVRAM_U16_SIZE,
				     sizeof(u16), GFP_KERNEL);
	if (!nRAM_updated) {
		ret = -ENOMEM;
		goto error_out;
	}

	ret = regmap_raw_read(chip->regmap_nvram, MAX1720X_NVRAM_START,
			      nRAM_current, MAX1720X_NVRAM_SIZE);
	if (ret) {
		dev_err(chip->dev,
			"Failed to read config from shadow RAM\n");
		goto error_out;
	}
	memcpy(nRAM_updated, nRAM_current, MAX1720X_NVRAM_SIZE);
	if (chip->batt_node)
		max1720x_apply_regval_shadow(chip, chip->batt_node,
					     nRAM_updated, batt_cnt);
	max1720x_apply_regval_shadow(chip, chip->dev->of_node,
				     nRAM_updated, glob_cnt);

	/* Ensure nCGain is not 0 if nNVCfg0.enCG is set */
	if ((nRAM_updated[NVRAM_U16_INDEX(MAX1720X_NNVCFG0)] &
	     MAX1720X_NNVCFG0_ENCG) &&
	    ((nRAM_updated[NVRAM_U16_INDEX(MAX1720X_NCGAIN)] == 0) ||
	     (nRAM_updated[NVRAM_U16_INDEX(MAX1720X_NCGAIN)] == 0x0400)))
		nRAM_updated[NVRAM_U16_INDEX(MAX1720X_NCGAIN)] = 0x4000;

	if (memcmp(nRAM_updated, nRAM_current, MAX1720X_NVRAM_SIZE)) {
		ret = regmap_raw_write(chip->regmap_nvram, MAX1720X_NVRAM_START,
				       nRAM_updated, MAX1720X_NVRAM_SIZE);
		if (ret) {
			dev_err(chip->dev,
				"Failed to write config from shadow RAM\n");
			goto error_out;
		}
		/* nConvgCfg change take effect without resetting the gauge */
		nRAM_current[NVRAM_U16_INDEX(MAX1720X_NCONVGCFG)] =
		    nRAM_updated[NVRAM_U16_INDEX(MAX1720X_NCONVGCFG)];
		if (memcmp(nRAM_updated, nRAM_current, MAX1720X_NVRAM_SIZE)) {
			dev_info(chip->dev,
				 "DT config differs from shadow, resetting\n");
			max1720x_fg_reset(chip);
		}
	}

error_out:
	kfree(nRAM_current);
	kfree(nRAM_updated);

	return ret;
}

static int max1720x_apply_regval_register(struct max1720x_chip *chip,
					struct device_node *node)
{
	int cnt, ret = 0, idx, err;
	u16 *regs, data;

	cnt =  of_property_count_elems_of_size(node, "maxim,r_regval",
					       sizeof(u16));
	if (!node || cnt <= 0)
		return 0;

	if (cnt & 1) {
		dev_warn(chip->dev, "%s maxim,r_regval u16 elems count is not even: %d\n",
			 node->name, cnt);
		return -EINVAL;
	}

	regs = kmalloc_array(cnt, sizeof(u16), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	ret = of_property_read_u16_array(node, "maxim,r_regval", regs, cnt);
	if (ret) {
		dev_warn(chip->dev, "failed to read %s maxim,r_regval: %d\n",
			 node->name, ret);
		goto register_out;
	}

	for (idx = 0; idx < cnt; idx += 2) {
		if (max1720x_is_reg(chip->dev, regs[idx])) {
			err = REGMAP_READ(chip->regmap, regs[idx], &data);
			if (!err && data != regs[idx + 1])
				REGMAP_WRITE(chip->regmap, regs[idx],
					     regs[idx + 1]);
		}
	}
register_out:
	kfree(regs);
	return ret;
}

static int max1720x_handle_dt_register_config(struct max1720x_chip *chip)
{
	int ret = 0;

	if (chip->batt_node)
		ret = max1720x_apply_regval_register(chip, chip->batt_node);

	if (ret)
		return ret;

	ret = max1720x_apply_regval_register(chip, chip->dev->of_node);

	return ret;
}

static int max1720x_init_chip(struct max1720x_chip *chip)
{
	u16 data = 0;
	int ret;

	ret = max1720x_handle_dt_shadow_config(chip);
	if (ret)
		return ret;

	ret = max1720x_handle_dt_register_config(chip);
	if (ret)
		return ret;

	chip->fake_capacity = -EINVAL;
	chip->fake_temperature = -EINVAL;
	chip->init_complete = true;
	chip->resume_complete = true;

	ret = REGMAP_READ(chip->regmap, MAX1720X_STATUS, &data);
	if (!ret && data & MAX1720X_STATUS_BR) {
		dev_info(chip->dev, "Clearing Battery Removal bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_STATUS,
				   MAX1720X_STATUS_BR, 0x0);
	}
	if (!ret && data & MAX1720X_STATUS_BI) {
		dev_info(chip->dev, "Clearing Battery Insertion bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_STATUS,
				   MAX1720X_STATUS_BI, 0x0);
	}
	if (!ret && data & MAX1720X_STATUS_POR) {
		dev_info(chip->dev, "Clearing Power-On Reset bit\n");
		regmap_update_bits(chip->regmap, MAX1720X_STATUS,
				   MAX1720X_STATUS_POR, 0x0);
	}

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NRSENSE, &chip->RSense);
	dev_info(chip->dev, "RSense value %d micro Ohm\n", chip->RSense * 10);
	(void) REGMAP_READ(chip->regmap, MAX1720X_CONFIG, &chip->RConfig);
	dev_info(chip->dev, "Config: 0x%04x\n", chip->RConfig);
	(void) REGMAP_READ(chip->regmap, MAX1720X_ICHGTERM, &data);
	dev_info(chip->dev, "IChgTerm: %d\n",
		 reg_to_micro_amp(data, chip->RSense));
	(void) REGMAP_READ(chip->regmap, MAX1720X_VEMPTY, &data);
	dev_info(chip->dev, "VEmpty: VE=%dmV VR=%dmV\n",
		 ((data >> 7) & 0x1ff) * 10, (data & 0x7f) * 40);

	/*
	 * Capacity data is stored as complement so it will not be zero. Using
	 * zero case to detect new un-primed pack
	 */
	ret = REGMAP_READ(chip->regmap_nvram, MAX1720X_NUSER18C, &data);
	if (!ret && data == 0)
		max1720x_prime_battery_qh_capacity(chip,
						   POWER_SUPPLY_STATUS_UNKNOWN);
	else
		max1720x_restore_battery_qh_capacity(chip);

	chip->prev_charge_state = POWER_SUPPLY_STATUS_UNKNOWN;

	/* Handle any IRQ that might have been set before init */
	max1720x_fg_irq_thread_fn(chip->primary->irq, chip);

	return 0;
}

static void max1720x_set_serial_number(struct max1720x_chip *chip)
{
	u16 data0 = 0, data1 = 0, data2 = 0;
	int date, count = 0, shift, err = 0;

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NMANFCTRNAME0, &data0);
	if (data0 == 0x5357) /* "SW": SWD */
		shift = 0;
	else if (data0 == 0x4257) /* "BW": DSY */
		shift = 8;
	else
		return;

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NSERIALNUMBER0, &data0);
	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NSERIALNUMBER1, &data1);
	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NSERIALNUMBER2, &data2);
	count += scnprintf(chip->serial_number + count,
			   sizeof(chip->serial_number) - count,
			   "%02X%02X%02X",
			   data0 >> shift, data1 >> shift, data2 >> shift);

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NMANFCTRDATE, &data0);
	date = (((((data0 >> 9) & 0x3f) + 1980) * 10000) +
		((data0 >> 5) & 0xf) * 100 + (data0 & 0x1F));
	count += scnprintf(chip->serial_number + count,
		 sizeof(chip->serial_number) - count,
		 "%d", date);

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NMANFCTRNAME0, &data0);
	count += scnprintf(chip->serial_number + count,
		 sizeof(chip->serial_number) - count,
		 "%c%c", data0 >> 8, data0 & 0xFF);

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NDEVICENAME0, &data0);
	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NDEVICENAME1, &data1);
	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NDEVICENAME2, &data2);
	count += scnprintf(chip->serial_number + count,
			   sizeof(chip->serial_number) - count,
			   "%c%c%c",
			   data0 >> shift, data1 >> shift, data2 >> shift);

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NDEVICENAME3, &data0);
	if (data0 >> 8 == 0)
		data0 = ('?' << 8) | (data0 & 0xFF);
	if ((data0 & 0xFF) == 0)
		data0 = (data0 & 0xFF00) | '?';
	count += scnprintf(chip->serial_number + count,
		 sizeof(chip->serial_number) - count,
		 "%c%c", data0 >> 8, data0 & 0xFF);

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NUSER1D1, &data0);
	count += scnprintf(chip->serial_number + count,
		 sizeof(chip->serial_number) - count,
		 "%c", data0 >> 8);

	(void) REGMAP_READ(chip->regmap_nvram, MAX1720X_NUSER1D0, &data0);
	if (shift == 8) {
		if (data0 >> 8 == 0xb1) {
			count += scnprintf(chip->serial_number + count,
					   sizeof(chip->serial_number) - count,
					   "B1");
		} else if (data0 >> 8 == 0xc1) {
			count += scnprintf(chip->serial_number + count,
					   sizeof(chip->serial_number) - count,
					   "C1");
		} else {
			count += scnprintf(chip->serial_number + count,
					   sizeof(chip->serial_number) - count,
					   "??");
		}
	} else {
		count += scnprintf(chip->serial_number + count,
				   sizeof(chip->serial_number) - count,
				   "%c%c", data0 >> 8, data0 & 0xFF);
	}
	if (err)
		chip->serial_number[0] = '\0';
}

static struct power_supply_desc max1720x_psy_desc = {
	.name = "maxfg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = max1720x_get_property,
	.set_property = max1720x_set_property,
	.property_is_writeable = max1720x_property_is_writeable,
	.properties = max1720x_battery_props,
	.num_properties = ARRAY_SIZE(max1720x_battery_props),
};

static void max1720x_init_work(struct work_struct *work)
{
	struct max1720x_chip *chip = container_of(work, struct max1720x_chip,
						  init_work.work);
	int ret = max1720x_init_chip(chip);

	if (ret == -EPROBE_DEFER) {
		schedule_delayed_work(&chip->init_work,
				      msecs_to_jiffies(MAX1720X_DELAY_INIT_MS));
		return;
	}
}

static int max1720x_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max1720x_chip *chip;
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = { };

	int ret = 0;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	i2c_set_clientdata(client, chip);
	chip->primary = client;
	chip->secondary = i2c_new_secondary_device(client, "nvram", 0xb);
	if (chip->secondary == NULL) {
		dev_err(dev, "Failed to initialize secondary i2c device\n");
		return -EINVAL;
	}
	i2c_set_clientdata(chip->secondary, chip);
	chip->regmap = devm_regmap_init_i2c(client, &max1720x_regmap_cfg);
	if (IS_ERR(chip->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		ret = -EINVAL;
		goto i2c_unregister;
	}

	chip->regmap_nvram =
	    devm_regmap_init_i2c(chip->secondary, &max1720x_regmap_nvram_cfg);
	if (IS_ERR(chip->regmap_nvram)) {
		dev_err(dev, "Failed to initialize nvram regmap\n");
		ret = -EINVAL;
		goto i2c_unregister;
	}

	mutex_init(&chip->cyc_ctr.lock);
	max1720x_restore_cycle_counter(chip);

	if (chip->primary->irq) {
		ret = request_threaded_irq(chip->primary->irq, NULL,
					   max1720x_fg_irq_thread_fn,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   MAX1720X_I2C_DRIVER_NAME, chip);
		if (ret != 0) {
			dev_err(chip->dev, "Unable to register IRQ handler\n");
			goto i2c_unregister;
		}
		enable_irq_wake(chip->primary->irq);
	}

	if (of_property_read_bool(dev->of_node, "maxim,psy-type-unknown"))
		max1720x_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	psy_cfg.drv_data = chip;
	chip->psy = devm_power_supply_register(dev,
					       &max1720x_psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy)) {
		dev_err(dev, "Couldn't register as power supply\n");
		ret = PTR_ERR(chip->psy);
		goto irq_unregister;
	}

	chip->history_index = 0;
	ret = device_create_file(&chip->psy->dev, &dev_attr_history_count);
	if (ret) {
		dev_err(dev, "Failed to create history_count attribute\n");
		goto psy_unregister;
	}

	ret = device_create_file(&chip->psy->dev, &dev_attr_history);
	if (ret) {
		dev_err(dev, "Failed to create history attribute\n");
		goto psy_unregister;
	}

	ret = device_create_file(&chip->psy->dev, &dev_attr_cycle_counts_bins);
	if (ret) {
		dev_err(dev, "Failed to create cycle_counts_bins attribute\n");
		goto psy_unregister;
	}

	ret = device_create_file(&chip->psy->dev, &dev_attr_offmode_charger);
	if (ret) {
		dev_err(dev, "Failed to create offmode_charger attribute\n");
		goto psy_unregister;
	}

	max1720x_set_serial_number(chip);

	INIT_WORK(&chip->cycle_count_work, max1720x_cycle_count_work);
	INIT_DELAYED_WORK(&chip->init_work, max1720x_init_work);
	schedule_delayed_work(&chip->init_work,
			      msecs_to_jiffies(MAX1720X_DELAY_INIT_MS));

	return 0;

psy_unregister:
	power_supply_unregister(chip->psy);
irq_unregister:
	free_irq(chip->primary->irq, chip);
i2c_unregister:
	i2c_unregister_device(chip->secondary);

	return ret;
}

static int max1720x_remove(struct i2c_client *client)
{
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->init_work);
	iio_channel_release(chip->iio_ch);
	if (chip->primary->irq)
		free_irq(chip->primary->irq, chip);
	power_supply_unregister(chip->psy);
	i2c_unregister_device(chip->secondary);

	return 0;
}

static const struct of_device_id max1720x_of_match[] = {
	{ .compatible = "maxim,max1720x"},
	{},
};
MODULE_DEVICE_TABLE(of, max1720x_of_match);

static const struct i2c_device_id max1720x_id[] = {
	{"max1720x", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max1720x_id);

#ifdef CONFIG_PM_SLEEP
static int max1720x_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	max1720x_fg_irq_thread_fn(chip->primary->irq, chip);

	pm_runtime_get_sync(chip->dev);
	chip->resume_complete = false;
	pm_runtime_put_sync(chip->dev);

	return 0;
}

static int max1720x_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1720x_chip *chip = i2c_get_clientdata(client);

	pm_runtime_get_sync(chip->dev);
	chip->resume_complete = true;
	pm_runtime_put_sync(chip->dev);

	max1720x_fg_irq_thread_fn(chip->primary->irq, chip);

	return 0;
}
#endif
static const struct dev_pm_ops max1720x_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(max1720x_pm_suspend, max1720x_pm_resume)
};

static struct i2c_driver max1720x_i2c_driver = {
	.driver = {
		   .name = "max1720x",
		   .of_match_table = max1720x_of_match,
		   .pm = &max1720x_pm_ops,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   },
	.id_table = max1720x_id,
	.probe = max1720x_probe,
	.remove = max1720x_remove,
};

module_i2c_driver(max1720x_i2c_driver);
MODULE_AUTHOR("Thierry Strudel <tstrudel@google.com>");
MODULE_DESCRIPTION("MAX17201/MAX17205 Fuel Gauge");
MODULE_LICENSE("GPL");
