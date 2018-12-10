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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h> /* register_chrdev, unregister_chrdev */
#include <linux/module.h>
#include <linux/seq_file.h> /* seq_read, seq_lseek, single_release */

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define MAX1720X_TRECALL_MS 5
#define MAX1720X_TPOR_MS 150
#define MAX1720X_TICLR_MS 500
#define MAX1720X_I2C_DRIVER_NAME "max1720x_fg_irq"
#define MAX1720X_N_OF_HISTORY_PAGES 203
#define MAX1720X_DELAY_INIT_MS 1000
#define FULLCAPNOM_STABILIZE_CYCLES 5

#define HISTORY_DEVICENAME "maxfg_history"

#define MAX1720X_GAUGE_TYPE	0
#define MAX1730X_GAUGE_TYPE	1

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
	MAX1720X_NUSER18C = 0x8C,	/* QH Capacity */
	MAX1720X_NUSER18D = 0x8D,	/* QH Capacity */
	MAX1720X_NODSCTH = 0x8E,	/* CCLC */
	MAX1720X_NODSCCFG = 0x8F,	/* CCLC */
	MAX1720X_NLEARNCFG = 0x9F,	/* not referred here */
	MAX1720X_NMISCCFG = 0xB2,	/* CCLC */
	MAX1720X_NHIBCFG = 0xB4,	/* CCLC */
	MAX1720X_NCONVGCFG = 0xB7,	/* convergence configuration */
	MAX1720X_NNVCFG0 = 0xB8,	/* 'NCG0' with NCG1 */
	MAX1720X_NUSER1C4 = 0xC4,	/* CCLC */
	MAX1720X_NUSER1C5 = 0xC5,	/* CCLC */
	MAX1720X_NCGAIN = 0xC8,		/* ....  */
	MAX1720X_NMANFCTRNAME0 = 0xCC,	/* SNUM */
	MAX1720X_NMANFCTRNAME1 = 0xCD,	/* CCLC */
	MAX1720X_NMANFCTRNAME2 = 0xCE,	/* CCLC */
	MAX1720X_NRSENSE = 0xCF,	/* value of sense resistor */
	MAX1720X_NUSER1D0 = 0xD0,	/* SNUM */
	MAX1720X_NUSER1D1 = 0xD1,	/* SNUM */
	MAX1720X_NUSER1D4 = 0xD4,	/* URST */
	MAX1720X_NMANFCTRDATE = 0xD6,	/* SNUM */
	MAX1720X_NFIRSTUSED = 0xD7,	/* CCLC */
	MAX1720X_NSERIALNUMBER0 = 0xD8,	/* SNUM */
	MAX1720X_NSERIALNUMBER1 = 0xD9,	/* SNUM */
	MAX1720X_NSERIALNUMBER2 = 0xDA,	/* SNUM */
	MAX1720X_NDEVICENAME0 = 0xDB,	/* SNUM */
	MAX1720X_NDEVICENAME1 = 0xDC,	/* SNUM */
	MAX1720X_NDEVICENAME2 = 0xDD,	/* SNUM */
	MAX1720X_NDEVICENAME3 = 0xDE,	/* SNUM */
	MAX1720X_NDEVICENAME4 = 0xDF,	/* CCLC */
	MAX1720X_NVRAM_END = 0xE0,
	MAX1720X_NVRAM_HISTORY_WRITE_STATUS_START = 0xE1,
	MAX1720X_NVRAM_HISTORY_VALID_STATUS_END = 0xE4,
	MAX1720X_NVRAM_HISTORY_WRITE_STATUS_END = 0xEA,
	MAX1720X_NVRAM_HISTORY_VALID_STATUS_START = 0xEB,
	MAX1720X_NVRAM_REMAINING_UPDATES = 0xED,
	MAX1720X_NVRAM_HISTORY_END = 0xEF,
};

enum max1730x_nvram {
	MAX1730X_NVRAM_START = 0x80,
	MAX1730X_NVRAM_END = 0xF0,
};

enum max1730x_register {
	MAX1730X_MAXMINVOLT = 0x08,
	MAX1730X_MAXMINTEMP = 0x09,
	MAX1730X_MAXMINCURR = 0x0A,
	MAX1730X_FULLCAPREP = 0x10,
	MAX1730X_VCELL = 0x1A,
	MAX1730X_TEMP = 0x1B,
	MAX1730X_CURRENT = 0x1C,
	MAX1730X_AVGCURRENT = 0x1D,
	MAX1730X_MIXCAP = 0x2B,
	MAX1730X_FULLCAP = 0x35,
	MAX1730X_LEARNCFG = 0xA1,
	MAX1730X_MAXPEAKPWR = 0xA4,
	MAX1730X_SUSPEAKPWR = 0xA5,
	MAX1730X_PACKRESISTANCE = 0xA6,
	MAX1730X_SYSRESISTANCE = 0xA7,
	MAX1730X_MINSYSVOLTAGE = 0xA8,
	MAX1730X_MPPCURRENT = 0xA9,
	MAX1730X_SPPCURRENT = 0xAA,
	MAX1730X_CONFIG2 = 0xAB,
	MAX1730X_IALRTTH = 0xAC,
	MAX1730X_MINVOLT = 0xAD,
	MAX1730X_MINCURR = 0xAE,
};

enum max1730x_command_bits {
	MAX1730X_COMMAND_FUEL_GAUGE_RESET = 0x8000,
};

enum max17xxx_register {
	MAX17XXX_MAXMINVOLT	= MAX1720X_MAXMINVOLT,
	MAX17XXX_VCELL		= MAX1720X_VCELL,
	MAX17XXX_TEMP		= MAX1720X_TEMP,
	MAX17XXX_CURRENT	= MAX1720X_CURRENT,
	MAX17XXX_AVGCURRENT	= MAX1720X_AVGCURRENT,
	MAX17XXX_MIXCAP		= MAX1720X_MIXCAP,
};

#define BUCKET_COUNT 10

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


struct max1720x_history {
	loff_t history_index;
	int history_count;
	bool *page_status;
	u16 *history;
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

	/* history */
	struct mutex history_lock;
	int hcmajor;
	struct cdev hcdev;
	struct class *hcclass;
	bool history_available;
	bool history_added;

	u16 RSense;
	u16 RConfig;
	int batt_id;
	bool init_complete;
	bool resume_complete;
	u16 health_status;
	int fake_capacity;
	int previous_qh;
	int current_capacity;
	int prev_charge_state;
	char serial_number[25];
	bool offmode_charger;
	u32 convgcfg_hysteresis;
	int nb_convgcfg;
	int curr_convgcfg_idx;
	s16 *temp_convgcfg;
	u16 *convgcfg_values;
	struct mutex convgcfg_lock;
	unsigned int debug_irq_none_cnt;
	bool shadow_override;
};

static int max1730x_regmap_map(int reg)
{
	int out;

	switch (reg) {
	case MAX17XXX_MAXMINVOLT:
		out = MAX1730X_MAXMINVOLT;
		break;
	case MAX17XXX_VCELL:
		out = MAX1730X_VCELL;
		break;
	case MAX17XXX_TEMP:
		out = MAX1730X_TEMP;
		break;
	case MAX17XXX_CURRENT:
		out = MAX1730X_CURRENT;
		break;
	case MAX17XXX_AVGCURRENT:
		out = MAX1730X_AVGCURRENT;
		break;
	case MAX17XXX_MIXCAP:
		out = MAX1730X_MIXCAP;
		break;
	default:
		out = reg;
		break;
	}

	return out;
}

/* when 1 use max17301 features */
static int max17xxx_gauge_type = -1;

static inline int max1720x_regmap_read(struct regmap *map,
				       unsigned int reg,
				       u16 *val,
				       const char *name)
{
	int rtn;
	unsigned int tmp;

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE)
		reg = max1730x_regmap_map(reg);
	else if (reg != MAX1720X_DEVNAME && max17xxx_gauge_type == -1)
		pr_warn("using default MAX1720X regmap\n");

	rtn = regmap_read(map, reg, &tmp);
	if (rtn)
		pr_err("Failed to read %s\n", name);
	else
		*val = tmp;

	return rtn;
}

#define REGMAP_READ(regmap, what, dst) \
	max1720x_regmap_read(regmap, what, dst, #what)

static inline int max1720x_regmap_write(struct regmap *map,
				       unsigned int reg,
				       u16 data,
				       const char *name)
{
	int rtn;

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE)
		reg = max1730x_regmap_map(reg);
	else if (reg != MAX1720X_DEVNAME && max17xxx_gauge_type == -1)
		pr_warn("using default MAX1720X regmap\n");

	rtn = regmap_write(map, reg, data);
	if (rtn)
		pr_err("Failed to write %s\n", name);

	return rtn;
}

#define REGMAP_WRITE(regmap, what, value) \
	max1720x_regmap_write(regmap, what, value, #what)

/* ------------------------------------------------------------------------- */

enum max17x0x_reg_types {
	GBMS_ATOM_TYPE_MAP = 0,
	GBMS_ATOM_TYPE_REG = 1,
	GBMS_ATOM_TYPE_ZONE = 2,
};

struct max17x0x_reg {
	u32 tag;
	int type;
	int size;
	union {
		unsigned int base;
		unsigned int reg;
		const u8 *map;
	};
};

/* this is a map for u16 registers */
#define ATOM_INIT_MAP(...)			\
	.type = GBMS_ATOM_TYPE_MAP,		\
	.size = 2 * sizeof((u8[]){__VA_ARGS__}),\
	.map = (u8[]){__VA_ARGS__}

#define ATOM_INIT_REG16(r)		\
	.type = GBMS_ATOM_TYPE_REG,	\
	.size = 2,			\
	.reg = r

#define ATOM_INIT_ZONE(start, sz)	\
	.type = GBMS_ATOM_TYPE_ZONE,	\
	.size = sz,			\
	.base = start

/* the point of the '' constants is to avoid defines for tag names...
 * so please don't add them ;-)
 */
static const struct max17x0x_reg max1720x[] = {
	{ 'BCNT', ATOM_INIT_MAP(0x8e, 0x8f, 0xb2, 0xb4, 0xcd,
				0xce, 0xd7, 0xdf, 0xc4, 0xc5) },
	{ 'QHCA', ATOM_INIT_REG16(0x8c) },
	{ 'QHQH', ATOM_INIT_REG16(0x8d) },
	{ 'SNUM', ATOM_INIT_MAP(0xcc, 0xd8, 0xd9, 0xda, 0xd6,
				0xdb, 0xdc, 0xdd, 0xde, 0xd1,
				0xd0) },
};

/* see b/119416045 for layout */
static const struct max17x0x_reg max1730x[] = {
	{ 'BCNT', ATOM_INIT_MAP(0x8c, 0x8d, 0x8f, 0x8e, 0xb4,
				0xc7, 0x9f, 0x9e, 0xb6, 0xb2)},
	{ 'QHCA', ATOM_INIT_REG16(0xe2) },
	{ 'QHQH', ATOM_INIT_REG16(0x9d) },
	{ 'SNUM', ATOM_INIT_MAP(0xce, 0xe6, 0xe7, 0xe8, 0xe9,
				0xea, 0xeb, 0xec, 0xed, 0xee,
				0xef) },
};

struct max17x0x_device_info {
	const struct max17x0x_reg *map;
	unsigned int max;
};

static const struct max17x0x_device_info max17x0x[] = {
	{ .map = max1720x, .max = ARRAY_SIZE(max1720x) },
	{ .map = max1730x, .max = ARRAY_SIZE(max1730x) },
};

const struct max17x0x_reg *max17x0x_find_by_index(int index)
{
	if (max17xxx_gauge_type == -1)
		return NULL;

	if (index < 0 || index >= max17x0x[max17xxx_gauge_type].max)
		return NULL;

	return &max17x0x[max17xxx_gauge_type].map[index];
}

/* NOTE: could use switch() cases with fallthrough */
const struct max17x0x_reg *max17x0x_find_by_id(u32 id)
{
	int i;

	if (max17xxx_gauge_type == -1)
		return NULL;

	for (i = 0; i < max17x0x[max17xxx_gauge_type].max ; i++) {
		if (max17x0x[max17xxx_gauge_type].map[i].tag == id)
			return &max17x0x[max17xxx_gauge_type].map[i];
	}

	return NULL;
}

/* ------------------------------------------------------------------------- */

/* offset of the register in this atom.
 * NOTE: this is the byte offset regardless of the size of the register
 */
static int max17x0x_reg_offset_of(const struct max17x0x_reg *a,
				  unsigned int reg)
{
	int i;

	switch (a->type) {
	case GBMS_ATOM_TYPE_REG:
		return (reg == a->reg) ? 0 : -EINVAL;
	case GBMS_ATOM_TYPE_ZONE:
		if (reg >= a->base && reg < a->base + a->size)
			return (reg - a->base) * 2;
		break;
	case GBMS_ATOM_TYPE_MAP:
		for (i = 0 ; i < a->size ; i++)
			if (a->map[i] == reg)
				return i * 2;
		break;
	}

	return -ERANGE;
}

static int max17x0x_reg_store_sz(struct regmap *regmap,
				 const struct max17x0x_reg *a,
				 const void *data,
				 int size)
{
	int ret;

	if (size > a->size)
		size = a->size;

	if (a->type == GBMS_ATOM_TYPE_MAP) {
		int i;
		const u16 *b = (u16 *)data;

		if (size % 2)
			return -ERANGE;

		for (i = 0; i < size / 2 ; i++) {
			ret = regmap_write(regmap, a->map[i], b[i]);
			if (ret < 0)
				break;
		}
	} else {
		ret = regmap_raw_write(regmap, a->base, data, size);
	}

	return ret;
}

static int max17x0x_reg_load_sz(struct regmap *regmap,
				const struct max17x0x_reg *a,
				void *data,
				int size)
{
	int ret;

	if (size > a->size)
		size = a->size;

	if (a->type == GBMS_ATOM_TYPE_MAP) {
		int i;
		unsigned int tmp;
		u16 *b = (u16 *)data;

		if (size % 2)
			return -ERANGE;

		for (i = 0; i < size / 2 ; i++) {
			ret = regmap_read(regmap,
					  (unsigned int)a->map[i],
					  &tmp);
			if (ret < 0)
				break;
			b[i] = tmp;
		}
	} else {
		ret = regmap_raw_read(regmap, a->base, data, size);
	}

	return ret;
}

#define max17x0x_reg_store(map, a, data) \
	max17x0x_reg_store_sz(map, a, data, (a)->size)

#define max17x0x_reg_load(map, a, data) \
	max17x0x_reg_load_sz(map, a, data, (a)->size)

/* CACHE ----------------------------------------------------------------- */

struct max17x0x_cache_data {
	struct max17x0x_reg atom;
	u16 *cache_data;
};

static int max17x0x_cache_index_of(const struct max17x0x_cache_data *cache,
				   unsigned int reg)
{
	const int offset = max17x0x_reg_offset_of(&cache->atom, reg);

	return (offset < 0) ? offset : offset / 2;
}

#define max17x0x_cache_store(cache, regmap) \
	max17x0x_reg_store(regmap, &(cache)->atom, (cache)->cache_data)

#define max17x0x_cache_load(cache, regmap) \
	max17x0x_reg_load(regmap, &(cache)->atom, (cache)->cache_data)

#define max17x0x_cache_memcmp(src, dst) \
	memcmp((src)->cache_data, (dst)->cache_data, (src)->atom.size)

static void max17x0x_cache_free(struct max17x0x_cache_data *cache)
{
	kfree(cache->cache_data);
	cache->cache_data = NULL;
}

static int max17x0x_cache_dup(struct max17x0x_cache_data *dst,
			      const struct max17x0x_cache_data *src)
{
	memcpy(dst, src, sizeof(*dst));

	dst->cache_data = (u16 *)kmalloc(src->atom.size, GFP_KERNEL);
	if (!dst->cache_data)
		return -ENOMEM;

	memcpy(dst->cache_data, src->cache_data, src->atom.size);
	return 0;
}

static int max17x0x_nvram_cache_init(struct max17x0x_cache_data *cache,
				     u16 start, int end)
{
	const int count = end - start + 1; /* includes end */

	memset(cache, 0, sizeof(*cache));

	cache->cache_data = (u16 *)kmalloc_array(count, sizeof(u16),
						 GFP_KERNEL);
	if (!cache->cache_data)
		return -ENOMEM;

	cache->atom.type = GBMS_ATOM_TYPE_ZONE;
	cache->atom.size = count * sizeof(u16);
	cache->atom.base = start;
	return 0;
}

#define max1720x_nvram_cache_init(cache) \
	max17x0x_nvram_cache_init(cache, MAX1720X_NVRAM_START, \
					 MAX1720X_NVRAM_END)
#define max1730x_nvram_cache_init(cache) \
	max17x0x_nvram_cache_init(cache, MAX1730X_NVRAM_START, \
					 MAX1730X_NVRAM_END)


/** ------------------------------------------------------------------------ */

/* TODO: factor with the one in google_bms.c */
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

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		switch (reg) {
		case MAX1730X_LEARNCFG:
			return true;
		}
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
	return (reg >= MAX1720X_NVRAM_START &&
		reg <= MAX1720X_NVRAM_HISTORY_END);
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

/* @return the number of pages or negative for error */
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

	kfree(write_status);
	kfree(valid_status);

	return valid_history_entry_count;
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

static int format_battery_history_entry(char *temp, int size, u16 *line)
{
	int length = 0, i;

	for (i = 0; i < MAX1720X_HISTORY_PAGE_SIZE; i++) {
		length += scnprintf(temp + length,
			size - length, "%04x ",
			line[i]);
	}

	if (length > 0)
		temp[--length] = 0;
	return length;
}

/* @return number of valid entries */
static int max1720x_history_read(struct max1720x_chip *chip,
				 struct max1720x_history *hi)
{
	memset(hi, 0, sizeof(*hi));

	hi->page_status = kcalloc(MAX1720X_N_OF_HISTORY_PAGES,
				sizeof(bool), GFP_KERNEL);
	if (!hi->page_status)
		return -ENOMEM;

	mutex_lock(&chip->history_lock);

	hi->history_count = get_battery_history_status(chip, hi->page_status);
	if (hi->history_count < 0) {
		goto error_exit;
	} else if (hi->history_count != 0) {
		const int size = hi->history_count * MAX1720X_HISTORY_PAGE_SIZE;

		hi->history = kmalloc_array(size, sizeof(u16), GFP_KERNEL);
		if (!hi->history) {
			hi->history_count = -ENOMEM;
			goto error_exit;
		}

		get_battery_history(chip, hi->page_status, hi->history);
	}

	mutex_unlock(&chip->history_lock);
	return hi->history_count;

error_exit:
	mutex_unlock(&chip->history_lock);
	kfree(hi->page_status);
	hi->page_status = NULL;
	return hi->history_count;

}

static void max1720x_history_free(struct max1720x_history *hi)
{
	kfree(hi->page_status);
	kfree(hi->history);

	hi->history = NULL;
	hi->page_status = NULL;
}

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
	POWER_SUPPLY_PROP_RESISTANCE_ID,
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

static void max17x0x_cycle_count_work(struct work_struct *work)
{
	u16 data;
	int bucket, cnt, batt_soc;
	const struct max17x0x_reg *bcnt;
	struct max1720x_chip *chip = container_of(work, struct max1720x_chip,
						  cycle_count_work);

	if (REGMAP_READ(chip->regmap, MAX1720X_REPSOC, &data))
		return;
	batt_soc = reg_to_percentage(data);

	bcnt = max17x0x_find_by_id('BCNT');
	if (!bcnt)
		return;

	mutex_lock(&chip->cyc_ctr.lock);

	if (chip->cyc_ctr.prev_soc != -1 &&
	    batt_soc >= 0 && batt_soc <= 100 &&
	    batt_soc > chip->cyc_ctr.prev_soc) {
		for (cnt = batt_soc ; cnt > chip->cyc_ctr.prev_soc ; cnt--) {
			bucket = cnt * BUCKET_COUNT / 100;
			if (bucket >= BUCKET_COUNT)
				bucket = BUCKET_COUNT - 1;
			chip->cyc_ctr.count[bucket]++;
#if 0
			/* Disable saving of bin cycle count to maxim NV storage
			   since the NV layout is not finalized yet */
			REGMAP_WRITE(chip->regmap_nvram,
				     bcnt->map[bucket],
				     chip->cyc_ctr.count[bucket]);
#endif
			pr_debug("Stored count: prev_soc=%d, soc=%d bucket=%d count=%d\n",
				 chip->cyc_ctr.prev_soc, cnt, bucket,
				 chip->cyc_ctr.count[bucket]);
		}
	}

	chip->cyc_ctr.prev_soc = batt_soc;

	mutex_unlock(&chip->cyc_ctr.lock);
}

static void max17x0x_restore_cycle_counter(struct max1720x_chip *chip)
{
	int ret, i;
	const struct max17x0x_reg *bcnt;

	bcnt = max17x0x_find_by_id('BCNT');
	if (!bcnt)
		return;

	mutex_lock(&chip->cyc_ctr.lock);
	ret = max17x0x_reg_load(chip->regmap_nvram, bcnt, chip->cyc_ctr.count);
	for (i = 0; ret == 0 && i < BUCKET_COUNT ; i++) {
		pr_debug("max1720x_cycle_counter[%d], addr=0x%02X, count=%d\n",
				i, bcnt->map[i], chip->cyc_ctr.count[i]);
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

static ssize_t max17x0x_set_cycle_counts_bins(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct power_supply *psy;
	struct max1720x_chip *chip;
	const struct max17x0x_reg *bcnt;

	psy = container_of(dev, struct power_supply, dev);
	chip = power_supply_get_drvdata(psy);

	bcnt = max17x0x_find_by_id('BCNT');
	if (bcnt) {
		u16 bincnt[BUCKET_COUNT];
		int ret, i, val[BUCKET_COUNT];
		const int bucket_count = bcnt->size / sizeof(u16);

		if (bucket_count != BUCKET_COUNT)
			return -ENODATA;

		if (sscanf(buf, "%d %d %d %d %d %d %d %d %d %d",
				&val[0], &val[1], &val[2], &val[3], &val[4],
				&val[5], &val[6], &val[7], &val[8], &val[9])
				!= bucket_count)
			return -EINVAL;

		mutex_lock(&chip->cyc_ctr.lock);

		for (i = 0; i < bucket_count ; i++)
			if (val[i] >= 0 && val[i] < U16_MAX)
				bincnt[i] = val[i];
			else
				bincnt[i] = chip->cyc_ctr.count[i];

		ret = max17x0x_reg_store(chip->regmap_nvram, bcnt, bincnt);
		if (ret < 0)
			count = -EINVAL;
		else
			memcpy(chip->cyc_ctr.count, bincnt,
				sizeof(chip->cyc_ctr.count));

		mutex_unlock(&chip->cyc_ctr.lock);
	}

	return count;
}


static DEVICE_ATTR(cycle_counts_bins, 0660,
		   max1720x_get_cycle_counts_bins,
		   max17x0x_set_cycle_counts_bins);

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

	(void) REGMAP_READ(chip->regmap, MAX17XXX_MIXCAP, &data);
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

	err = REGMAP_READ(chip->regmap, MAX17XXX_AVGCURRENT, &data);
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

	if (chip->health_status & MAX1720X_STATUS_VMX) {
		chip->health_status &= ~MAX1720X_STATUS_VMX;
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}

	if (chip->health_status & MAX1720X_STATUS_TMN) {
		chip->health_status &= ~MAX1720X_STATUS_TMN;
		return POWER_SUPPLY_HEALTH_COLD;
	}

	if (chip->health_status & MAX1720X_STATUS_TMX) {
		chip->health_status &= ~MAX1720X_STATUS_TMX;
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

static void max1720x_handle_update_nconvgcfg(struct max1720x_chip *chip,
					     int temp)
{
	int idx = -1;

	if (chip->temp_convgcfg == NULL)
		return;

	if (temp <= chip->temp_convgcfg[0]) {
		idx = 0;
	} else if (temp > chip->temp_convgcfg[chip->nb_convgcfg - 1]) {
		idx = chip->nb_convgcfg - 1;
	} else {
		for (idx = 1 ; idx < chip->nb_convgcfg; idx++) {
			if (temp > chip->temp_convgcfg[idx - 1] &&
			    temp <= chip->temp_convgcfg[idx])
				break;
		}
	}
	mutex_lock(&chip->convgcfg_lock);
	/* We want to switch to higher slot only if above temp + hysteresis
	 * but when temperature drops, we want to change at the level
	 */
	if ((idx != chip->curr_convgcfg_idx) &&
	    (chip->curr_convgcfg_idx == -1 || idx < chip->curr_convgcfg_idx ||
	     temp >= chip->temp_convgcfg[chip->curr_convgcfg_idx] +
	     chip->convgcfg_hysteresis)) {
		REGMAP_WRITE(chip->regmap_nvram, MAX1720X_NCONVGCFG,
			     chip->convgcfg_values[idx]);
		chip->curr_convgcfg_idx = idx;
		dev_info(chip->dev, "updating nConvgcfg to 0x%04x as temp is %d (idx:%d)\n",
			 chip->convgcfg_values[idx], temp, idx);
	}
	mutex_unlock(&chip->convgcfg_lock);
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
		err = REGMAP_READ(map, MAX17XXX_AVGCURRENT, &data);
		/* current is positive value when flowing to device */
		val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		err = REGMAP_READ(map, MAX17XXX_CURRENT, &data);
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
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = chip->batt_id;
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		err = REGMAP_READ(map, MAX1720X_RCELL, &data);
		val->intval = reg_to_resistance_micro_ohms(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		REGMAP_READ(map, MAX17XXX_TEMP, &data);
		val->intval = reg_to_deci_deg_cel(data);
		max1720x_handle_update_nconvgcfg(chip, val->intval);
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
		err = REGMAP_READ(map, MAX17XXX_MAXMINVOLT, &data);
		/* LSB: 20mV */
		val->intval = ((data >> 8) & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		err = REGMAP_READ(map, MAX17XXX_MAXMINVOLT, &data);
		/* LSB: 20mV */
		val->intval = (data & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		err = REGMAP_READ(map, MAX17XXX_VCELL, &data);
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
static void max17x0x_fg_reset(struct max1720x_chip *chip)
{
	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		REGMAP_WRITE(chip->regmap, MAX1730X_CONFIG2,
			     MAX1730X_COMMAND_FUEL_GAUGE_RESET);
	} else {
		REGMAP_WRITE(chip->regmap, MAX1720X_CONFIG2,
			     MAX1720X_COMMAND_FUEL_GAUGE_RESET);
	}

	msleep(MAX1720X_TPOR_MS);
}

/*
 * A full reset restores the ICs to their power-up state the same as if power
 * had been cycled.
 */
static int max1720x_full_reset(struct max1720x_chip *chip)
{
	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		/* a full (hw) reset on max1730x cause charge and discharge FET
		 * to toggle and the device will lose power. Will need to
		 * connect the device to a charger to get max1730x firmware to
		 * start and max1730x to close the FETs. Never send a HW reset
		 * to a 1730x while in system...
		 */
		dev_warn(chip->dev, "ignore full reset of fuel gauge\n");
		return 0;
	}

	REGMAP_WRITE(chip->regmap, MAX1720X_COMMAND,
		     MAX1720X_COMMAND_HARDWARE_RESET);

	msleep(MAX1720X_TPOR_MS);

	return 0;
}

static irqreturn_t max1720x_fg_irq_thread_fn(int irq, void *obj)
{
	struct max1720x_chip *chip = obj;
	u16 fg_status, fg_status_clr;
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
	if (fg_status == 0) {
		chip->debug_irq_none_cnt++;
		pr_debug("spurius: fg_status=0 cnt=%d\n",
			chip->debug_irq_none_cnt);
		/* rate limit spurius interrupts */
		msleep(MAX1720X_TICLR_MS);
		return IRQ_HANDLED;
	}

	/* only used to report health */
	chip->health_status |= fg_status;
	/* write 0 to clear will loose interrupts when we don't write 1 to the
	 * bits that are not set. Just inverting fg_status cause an interrupt
	 * storm, only setting the bits marked as "host must clear" in the DS
	 * seems to work eg:
	 *
	 * fg_status_clr = fg_status
	 * fg_status_clr |= MAX1720X_STATUS_POR | MAX1720X_STATUS_DSOCI
	 *                | MAX1720X_STATUS_BI;
	 *
	 * If the above logic is sound, we probably need to set also the bits
	 * that config mark as "host must clear". Maxim to confirm.
	 */
	fg_status_clr = fg_status;

	if (fg_status & MAX1720X_STATUS_POR) {
		fg_status_clr &= ~MAX1720X_STATUS_POR;
		pr_debug("POR is set\n");
	}
	if (fg_status & MAX1720X_STATUS_IMN)
		pr_debug("IMN is set\n");

	if (fg_status & MAX1720X_STATUS_BST)
		pr_debug("BST is set\n");

	if (fg_status & MAX1720X_STATUS_IMX)
		pr_debug("IMX is set\n");

	if (fg_status & MAX1720X_STATUS_DSOCI) {
		fg_status_clr &= ~MAX1720X_STATUS_DSOCI;
		pr_debug("DSOCI is set\n");
		schedule_work(&chip->cycle_count_work);
	}
	if (fg_status & MAX1720X_STATUS_VMN) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status_clr &= ~MAX1720X_STATUS_VMN;
		pr_debug("VMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_TMN) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status_clr &= ~MAX1720X_STATUS_TMN;
		pr_debug("TMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_SMN) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status_clr &= ~MAX1720X_STATUS_SMN;
		pr_debug("SMN is set\n");
	}
	if (fg_status & MAX1720X_STATUS_BI)
		pr_debug("BI is set\n");

	if (fg_status & MAX1720X_STATUS_VMX) {
		if (chip->RConfig & MAX1720X_CONFIG_VS)
			fg_status_clr &= ~MAX1720X_STATUS_VMX;
		pr_debug("VMX is set\n");
	}
	if (fg_status & MAX1720X_STATUS_TMX) {
		if (chip->RConfig & MAX1720X_CONFIG_TS)
			fg_status_clr &= ~MAX1720X_STATUS_TMX;
		pr_debug("TMX is set\n");
	}
	if (fg_status & MAX1720X_STATUS_SMX) {
		if (chip->RConfig & MAX1720X_CONFIG_SS)
			fg_status_clr &= ~MAX1720X_STATUS_SMX;
		pr_debug("SMX is set\n");
	}

	if (fg_status & MAX1720X_STATUS_BR)
		pr_debug("BR is set\n");

	REGMAP_WRITE(chip->regmap, MAX1720X_STATUS, fg_status_clr);

	if (chip->psy)
		power_supply_changed(chip->psy);

	/* oneshot w/o filter will unmask on return but gauge will take up
	 * to 351 ms to clear ALRM1.
	 */
	msleep(MAX1720X_TICLR_MS);

	return IRQ_HANDLED;
}

static int max1720x_read_batt_id(const struct max1720x_chip *chip, int *batt_id)
{
	struct device_node *node = chip->dev->of_node;
	const char *batt_psy_name = NULL;
	struct power_supply *batt_psy;
	union power_supply_propval val;
	int rc = 0, temp_id;

	*batt_id = 0;

	rc = of_property_read_u32(node, "maxim,force-batt-id", &temp_id);
	if (rc == 0) {
		dev_warn(chip->dev, "forcing battery RID %d\n", temp_id);
		*batt_id = temp_id;
		return 0;
	}

	rc = of_property_read_string(node, "maxim,bat-power-supply",
				     &batt_psy_name);
	if (rc) {
		/* only method now */
		dev_warn(chip->dev,
			"cannot read bat-power-supply, rc=%d\n", rc);
		return -EINVAL;
	}

	batt_psy = power_supply_get_by_name(batt_psy_name);
	if (!batt_psy) {
		dev_warn(chip->dev, "failed to get battery power supply\n");
		return -EPROBE_DEFER;
	}

	/* POWER_SUPPLY_PROP_RESISTANCE_ID is in ohms */
	rc = power_supply_get_property(batt_psy,
				       POWER_SUPPLY_PROP_RESISTANCE_ID, &val);
	if (rc == -EINVAL || ((rc == 0) && (val.intval == -EINVAL))) {
		return -EPROBE_DEFER;
	} else if (rc < 0) {
		dev_err(chip->dev, "failed to get batt-id rc=%d\n", rc);
		return -EINVAL;
	}

	*batt_id = val.intval;

	return 0;
}

/* TODO: fix detection of 17301 for non samples looking at FW version too */
static int max1720x_read_gauge_type(struct max1720x_chip *chip)
{
	u16 devname;
	int ret, gauge_type = MAX1720X_GAUGE_TYPE;

	ret = REGMAP_READ(chip->regmap, MAX1720X_DEVNAME, &devname);
	if (ret != 0) {
		dev_err(chip->dev, "cannot read device name %d\n", ret);
	} else {
		switch (devname >> 4) {
		case 0x406:  /* max1730x pass2 silicon */
		case 0x405:  /* max1730x pass2 silicon initial samples */
			gauge_type = MAX1730X_GAUGE_TYPE;
			break;
		case 0x404:  /* max1730x sample */
			chip->shadow_override = false;
			gauge_type = MAX1730X_GAUGE_TYPE;
			break;
		default: /* default to max1720x */
			break;
		}
	}

	return gauge_type;
}


static int max1720x_handle_dt_batt_id(struct max1720x_chip *chip)
{
	int ret, batt_id;
	u32 batt_id_range = 20, batt_id_kohm;
	struct device_node *node = chip->dev->of_node;
	struct device_node *config_node, *child_node;

	ret = max1720x_read_batt_id(chip, &chip->batt_id);
	if (ret == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (ret < 0 || chip->batt_id < 0)
		/* Don't fail probe on that, just ignore error */
		return 0;

	dev_info(chip->dev, "device battery RID: %d kohm\n", chip->batt_id);

	ret = of_property_read_u32(node, "maxim,batt-id-range-pct",
				   &batt_id_range);
	if (ret && ret == -EINVAL)
		dev_warn(chip->dev, "failed to read maxim,batt-id-range-pct\n");

	config_node = of_find_node_by_name(node, "maxim,config");
	if (!config_node) {
		dev_warn(chip->dev, "Failed to find maxim,config setting\n");
		return 0;
	}

	batt_id = chip->batt_id / 1000;
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

static int max17x0x_apply_regval_shadow(struct max1720x_chip *chip,
					struct device_node *node,
					struct max17x0x_cache_data *nRAM,
					int nb)
{
	u16 *regs;
	int ret, i;
	const char *propname;

	propname = (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) ?
		 "maxim,n_regval_1730x" : "maxim,n_regval_1720x";

	if (!node || nb <= 0)
		return 0;

	if (nb & 1) {
		dev_warn(chip->dev, "%s %s u16 elems count is not even: %d\n",
			 node->name, propname, nb);
		return -EINVAL;
	}

	regs = (u16 *)kmalloc_array(nb, sizeof(u16), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	ret = of_property_read_u16_array(node, propname, regs, nb);
	if (ret) {
		dev_warn(chip->dev, "failed to read %s: %d\n", propname, ret);
		goto shadow_out;
	}

	for (i = 0; i < nb; i += 2) {
		const int idx = max17x0x_cache_index_of(nRAM, regs[i]);

		if (idx >= 0)
			nRAM->cache_data[idx] = regs[i + 1];
	}
shadow_out:
	kfree(regs);
	return ret;
}

/* support for initial batch of ill configured max1720x packs */
static void max1720x_consistency_check(struct max17x0x_cache_data *cache)
{
	int nvcfg_idx = max17x0x_cache_index_of(cache, MAX1720X_NNVCFG0);
	int ncgain_idx = max17x0x_cache_index_of(cache, MAX1720X_NCGAIN);
	u16 *nRAM_updated = cache->cache_data;

	if ((nRAM_updated[nvcfg_idx] & MAX1720X_NNVCFG0_ENCG) &&
		((nRAM_updated[ncgain_idx] == 0) ||
		(nRAM_updated[ncgain_idx] == 0x0400)))
		nRAM_updated[ncgain_idx] = 0x4000;
}

static bool max17x0x_should_reset(struct max1720x_chip *chip,
				  struct max17x0x_cache_data *nRAM_c,
				  struct max17x0x_cache_data *nRAM_u)
{
	const char *propname;
	int ret, idx = -1;
	u32 nver_reg;

	propname = (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) ?
		"maxim,n_regval_1730x_ver" : "maxim,n_regval_1720x_ver";

	/* if n_regval_1730x_ver is present, reset fg only when ver changes */
	ret = of_property_read_u32(chip->batt_node, propname, &nver_reg);
	if (ret == 0)
		idx = max17x0x_cache_index_of(nRAM_u, nver_reg);

	if (idx < 0) {
		/* nConvgCfg change take effect without resetting the gauge */
		idx = max17x0x_cache_index_of(nRAM_u, MAX1720X_NCONVGCFG);
		nRAM_c->cache_data[idx] = nRAM_u->cache_data[idx];

		return max17x0x_cache_memcmp(nRAM_u, nRAM_c) != 0;
	}

	return nRAM_c->cache_data[idx] < nRAM_u->cache_data[idx];
}

static int max17x0x_handle_dt_shadow_config(struct max1720x_chip *chip)
{
	int ret, glob_cnt;
	const char *propname = NULL;
	struct max17x0x_cache_data nRAM_c;
	struct max17x0x_cache_data nRAM_u;

	ret = max1720x_handle_dt_batt_id(chip);
	if (ret)
		return ret;

	/* for devices that don't support max1720x_fg_reset() */
	if (!chip->shadow_override) {
		dev_info(chip->dev, "ignore shadow override\n");
		return 0;
	}

	if (max17xxx_gauge_type == -1)
		return -EINVAL;

	if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
		propname = "maxim,n_regval_1730x";
		ret = max1730x_nvram_cache_init(&nRAM_c);
	} else {
		propname = "maxim,n_regval_1720x";
		ret = max1720x_nvram_cache_init(&nRAM_c);
	}

	if (ret < 0)
		return ret;

	ret = max17x0x_cache_load(&nRAM_c, chip->regmap_nvram);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read config from shadow RAM\n");
		goto error_out;
	}

	ret = max17x0x_cache_dup(&nRAM_u, &nRAM_c);
	if (ret < 0)
		goto error_out;


	/* apply overrides */
	if (chip->batt_node) {
		int batt_cnt;

		batt_cnt = of_property_count_elems_of_size(chip->batt_node,
							   propname,
							   sizeof(u16));
		max17x0x_apply_regval_shadow(chip, chip->batt_node,
					     &nRAM_u,
					     batt_cnt);
	}

	glob_cnt = of_property_count_elems_of_size(chip->dev->of_node,
						   propname,
						   sizeof(u16));
	max17x0x_apply_regval_shadow(chip, chip->dev->of_node,
				     &nRAM_u,
				     glob_cnt);

	if (max17xxx_gauge_type == MAX1720X_GAUGE_TYPE)
		max1720x_consistency_check(&nRAM_u);

	if (max17x0x_cache_memcmp(&nRAM_c, &nRAM_u)) {
		bool fg_reset;

		ret = max17x0x_cache_store(&nRAM_u, chip->regmap_nvram);
		if (ret < 0) {
			dev_err(chip->dev,
				"Failed to write config from shadow RAM\n");
			goto error_out;
		}

		fg_reset = max17x0x_should_reset(chip, &nRAM_c, &nRAM_u);
		if (fg_reset) {
			dev_info(chip->dev,
				"DT config differs from shadow, resetting\n");
			max17x0x_fg_reset(chip);
		}
	}

error_out:
	max17x0x_cache_free(&nRAM_c);
	max17x0x_cache_free(&nRAM_u);

	return ret;
}

static int max17x0x_apply_regval_register(struct max1720x_chip *chip,
					struct device_node *node)
{
	int cnt, ret = 0, idx, err;
	u16 *regs, data;
	const char *propname;

	propname = (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) ?
		 "maxim,r_regval_1730x" : "maxim,r_regval_1720x";

	cnt =  of_property_count_elems_of_size(node, propname, sizeof(u16));
	if (!node || cnt <= 0)
		return 0;

	if (cnt & 1) {
		dev_warn(chip->dev, "%s %s u16 elems count is not even: %d\n",
			 node->name, propname, cnt);
		return -EINVAL;
	}

	regs = kmalloc_array(cnt, sizeof(u16), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	ret = of_property_read_u16_array(node, propname, regs, cnt);
	if (ret) {
		dev_warn(chip->dev, "failed to read %s %s: %d\n",
			 node->name, propname, ret);
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

static int max17x0x_handle_dt_register_config(struct max1720x_chip *chip)
{
	int ret = 0;

	if (chip->batt_node)
		ret = max17x0x_apply_regval_register(chip, chip->batt_node);

	if (ret)
		return ret;

	ret = max17x0x_apply_regval_register(chip, chip->dev->of_node);

	return ret;
}

static int max1720x_handle_dt_nconvgcfg(struct max1720x_chip *chip)
{
	int ret = 0, i;
	struct device_node *node = chip->dev->of_node;

	chip->curr_convgcfg_idx = -1;
	mutex_init(&chip->convgcfg_lock);
	chip->nb_convgcfg =
	    of_property_count_elems_of_size(node, "maxim,nconvgcfg-temp-limits",
					    sizeof(s16));
	if (!chip->nb_convgcfg)
		return 0;

	ret = of_property_read_u32(node, "maxim,nconvgcfg-temp-hysteresis",
				   &chip->convgcfg_hysteresis);
	if (ret < 0)
		chip->convgcfg_hysteresis = 10;

	if (chip->nb_convgcfg != of_property_count_elems_of_size(node,
						  "maxim,nconvgcfg-values",
						  sizeof(u16))) {
		dev_warn(chip->dev, "%s maxim,nconvgcfg-values and maxim,nconvgcfg-temp-limits are missmatching number of elements\n",
			 node->name);
		return -EINVAL;
	}
	chip->temp_convgcfg = devm_kmalloc_array(chip->dev, chip->nb_convgcfg,
						 sizeof(s16), GFP_KERNEL);
	if (!chip->temp_convgcfg)
		return -ENOMEM;

	chip->convgcfg_values = devm_kmalloc_array(chip->dev, chip->nb_convgcfg,
						   sizeof(u16), GFP_KERNEL);
	if (!chip->convgcfg_values) {
		devm_kfree(chip->dev, chip->temp_convgcfg);
		chip->temp_convgcfg = NULL;
		return -ENOMEM;
	}

	ret = of_property_read_u16_array(node, "maxim,nconvgcfg-temp-limits",
					 (u16 *) chip->temp_convgcfg,
					 chip->nb_convgcfg);
	if (ret) {
		dev_warn(chip->dev, "failed to read maxim,nconvgcfg-temp-limits: %d\n",
			 ret);
		goto error;
	}

	ret = of_property_read_u16_array(node, "maxim,nconvgcfg-values",
					 chip->convgcfg_values,
					 chip->nb_convgcfg);
	if (ret) {
		dev_warn(chip->dev, "failed to read maxim,nconvgcfg-values: %d\n",
			 ret);
		goto error;
	}
	for (i = 1; i < chip->nb_convgcfg; i++) {
		if (chip->temp_convgcfg[i] < chip->temp_convgcfg[i-1]) {
			dev_warn(chip->dev, "nconvgcfg-temp-limits idx:%d < idx:%d\n",
				 i, i-1);
			goto error;
		}
		if ((chip->temp_convgcfg[i] - chip->temp_convgcfg[i-1])
		    <= chip->convgcfg_hysteresis) {
			dev_warn(chip->dev, "nconvgcfg-temp-hysteresis smaller than idx:%d, idx:%d\n",
				 i, i-1);
			goto error;
		}
	}

error:
	if (ret) {
		devm_kfree(chip->dev, chip->temp_convgcfg);
		devm_kfree(chip->dev, chip->convgcfg_values);
		chip->temp_convgcfg = NULL;
	}

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static int get_irq_none_cnt(void *data, u64 *val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)data;

	*val = chip->debug_irq_none_cnt;
	return 0;
}

static int set_irq_none_cnt(void *data, u64 val)
{
	struct max1720x_chip *chip = (struct max1720x_chip *)data;

	if (val == 0)
		chip->debug_irq_none_cnt = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(irq_none_cnt_fops, get_irq_none_cnt,
	set_irq_none_cnt, "%llu\n");
#endif

static int init_debugfs(struct max1720x_chip *chip)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *de;

	de = debugfs_create_dir("max1720x", 0);
	if (de)
		debugfs_create_file("irq_none_cnt", 0644, de,
				   chip, &irq_none_cnt_fops);
#endif
	return 0;
}

static u16 max1720x_read_rsense(const struct max1720x_chip *chip)
{
	u16 rsense = 0;
	u32 rsense_default = 0;
	int ret;

	(void)of_property_read_u32(chip->dev->of_node, "maxim,rsense-default",
		&rsense_default);

	ret = REGMAP_READ(chip->regmap_nvram, MAX1720X_NRSENSE, &rsense);
	if (rsense_default && (ret || rsense != rsense_default)) {
		rsense = rsense_default;
		dev_warn(chip->dev, "RSense, forcing %d micro Ohm (%d)\n",
			rsense * 10, ret);
	}

	return rsense;
}

static int max1720x_init_chip(struct max1720x_chip *chip)
{
	u16 data = 0;
	int ret;

	if (of_property_read_bool(chip->dev->of_node, "maxim,force-hard-reset"))
		max1720x_full_reset(chip);

	ret = max17x0x_handle_dt_shadow_config(chip);
	if (ret == -EPROBE_DEFER)
		return ret;

	ret = max17x0x_handle_dt_register_config(chip);
	if (ret == -EPROBE_DEFER)
		return ret;

	(void) max1720x_handle_dt_nconvgcfg(chip);

	chip->fake_capacity = -EINVAL;
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

	chip->RSense = max1720x_read_rsense(chip);
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

	init_debugfs(chip);

	/* Handle any IRQ that might have been set before init */
	max1720x_fg_irq_thread_fn(chip->primary->irq, chip);

	return 0;
}

static int max1730x_decode_sn(char *serial_number,
			      unsigned int max,
			      const u16 *data)
{
	/* TODO: b/113300630 */
	return 0;
}

static int max1720x_decode_sn(char *serial_number,
			      unsigned int max,
			      const u16 *data)
{
	int tmp, count = 0, shift;

	if (data[0] == 0x5357) /* "SW": SWD */
		shift = 0;
	else if (data[0] == 0x4257) /* "BW": DSY */
		shift = 8;
	else
		return -EINVAL;

	count += scnprintf(serial_number + count, max - count, "%02X%02X%02X",
			   data[1] >> shift,
			   data[2] >> shift,
			   data[3] >> shift);

	tmp = (((((data[4] >> 9) & 0x3f) + 1980) * 10000) +
		((data[4] >> 5) & 0xf) * 100 + (data[4] & 0x1F));
	count += scnprintf(serial_number + count, max - count, "%d",
			   tmp);

	count += scnprintf(serial_number + count, max - count, "%c%c",
			   data[0] >> 8,
			   data[0] & 0xFF);

	count += scnprintf(serial_number + count, max - count, "%c%c%c",
			   data[5] >> shift,
			   data[6] >> shift,
			   data[7] >> shift);

	tmp = data[8];
	if (tmp >> 8 == 0)
		tmp = ('?' << 8) | (tmp & 0xFF);
	if ((tmp & 0xFF) == 0)
		tmp = (tmp & 0xFF00) | '?';
	count += scnprintf(serial_number + count, max - count, "%c%c",
			   tmp >> 8,
			   tmp & 0xFF);

	count += scnprintf(serial_number + count, max - count, "%c",
			   data[9] >> 8);

	if (shift == 8) {
		if (data[10] >> 8 == 0xb1) {
			count += scnprintf(serial_number + count, max - count,
					   "B1");
		} else if (data[10] >> 8 == 0xc1) {
			count += scnprintf(serial_number + count, max - count,
					   "C1");
		} else {
			count += scnprintf(serial_number + count, max - count,
					   "??");
		}
	} else {
		count += scnprintf(serial_number + count, max - count, "%c%c",
				   data[10] >> 8, data[10] & 0xFF);
	}

	return 0;
}

static void max17x0x_set_serial_number(struct max1720x_chip *chip)
{
	int err = -EINVAL;
	const struct max17x0x_reg *snum;

	snum = max17x0x_find_by_id('SNUM');
	if (snum) {
		char buff[snum->size];

		err = max17x0x_reg_load(chip->regmap_nvram, snum, buff);
		if (err < 0) {
			err = -EIO;
		} else if (max17xxx_gauge_type == MAX1730X_GAUGE_TYPE) {
			err = max1730x_decode_sn(chip->serial_number,
						 sizeof(chip->serial_number),
						 (u16 *)buff);
		} else {
			err = max1720x_decode_sn(chip->serial_number,
						 sizeof(chip->serial_number),
						 (u16 *)buff);
		}
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


static void *ct_seq_start(struct seq_file *s, loff_t *pos)
{
	struct max1720x_history *hi =
		(struct max1720x_history *)s->private;

	if (*pos >= hi->history_count)
		return NULL;
	hi->history_index = *pos;

	return &hi->history_index;
}

static void *ct_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	loff_t *spos = (loff_t *)v;
	struct max1720x_history *hi =
		(struct max1720x_history *)s->private;

	*pos = ++*spos;
	if (*pos >= hi->history_count)
		return NULL;

	return spos;
}

static void ct_seq_stop(struct seq_file *s, void *v)
{
	/* iterator in hi, no need to free */
}

static int ct_seq_show(struct seq_file *s, void *v)
{
	char temp[96];
	loff_t *spos = (loff_t *)v;
	struct max1720x_history *hi =
		(struct max1720x_history *)s->private;
	const size_t offset = *spos * MAX1720X_HISTORY_PAGE_SIZE;

	format_battery_history_entry(temp, sizeof(temp), &hi->history[offset]);
	seq_printf(s, "%s\n", temp);

	return 0;
}

static const struct seq_operations ct_seq_ops = {
	.start = ct_seq_start,
	.next  = ct_seq_next,
	.stop  = ct_seq_stop,
	.show  = ct_seq_show
};

static int history_dev_open(struct inode *inode, struct file *file)
{
	struct max1720x_chip *chip =
		container_of(inode->i_cdev, struct max1720x_chip, hcdev);
	struct max1720x_history *hi;
	int history_count;

	hi = __seq_open_private(file, &ct_seq_ops, sizeof(*hi));
	if (!hi)
		return -ENOMEM;

	history_count = max1720x_history_read(chip, hi);
	if (history_count < 0) {
		return history_count;
	} else if (history_count == 0) {
		dev_info(chip->dev, "No battery history has been recorded\n");
	}

	return 0;
}

static int history_dev_release(struct inode *inode, struct file *file)
{
	struct max1720x_history *hi =
		((struct seq_file *)file->private_data)->private;

	if (hi) {
		max1720x_history_free(hi);
		seq_release_private(inode, file);
	}

	return 0;
}

static const struct file_operations hdev_fops = {
	.open = history_dev_open,
	.owner = THIS_MODULE,
	.read = seq_read,
	.release = history_dev_release,
};

static void max1720x_cleanup_history(struct max1720x_chip *chip)
{
	if (chip->history_added)
		cdev_del(&chip->hcdev);
	if (chip->history_available)
		device_destroy(chip->hcclass, chip->hcmajor);
	if (chip->hcclass)
		class_destroy(chip->hcclass);
	if (chip->hcmajor != -1)
		unregister_chrdev_region(chip->hcmajor, 1);
}

static int max1720x_init_history(struct max1720x_chip *chip)
{
	struct device *hcdev;

	mutex_init(&chip->history_lock);

	chip->hcmajor = -1;

	/* cat /proc/devices */
	if (alloc_chrdev_region(&chip->hcmajor, 0, 1, HISTORY_DEVICENAME) < 0)
		goto no_history;
	/* ls /sys/class */
	chip->hcclass = class_create(THIS_MODULE, HISTORY_DEVICENAME);
	if (chip->hcclass == NULL)
		goto no_history;
	/* ls /dev/ */
	hcdev = device_create(chip->hcclass, NULL, chip->hcmajor, NULL,
		HISTORY_DEVICENAME);
	if (hcdev == NULL)
		goto no_history;

	chip->history_available = true;
	cdev_init(&chip->hcdev, &hdev_fops);
	if (cdev_add(&chip->hcdev, chip->hcmajor, 1) == -1)
		goto no_history;

	chip->history_added = true;
	return 0;

no_history:
	max1720x_cleanup_history(chip);
	return -ENODEV;
}

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

	(void)max1720x_init_history(chip);
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

	chip->shadow_override = true;
	if (of_property_read_bool(dev->of_node, "maxim,max1730x-compat")) {
		/* NOTE: NEED TO COME BEFORE REGISTER ACCESS */
		max17xxx_gauge_type = max1720x_read_gauge_type(chip);
		dev_warn(chip->dev, "device gauge_type: %d override=%d\n",
			max17xxx_gauge_type, chip->shadow_override);
	} else {
		max17xxx_gauge_type = MAX1720X_GAUGE_TYPE;
	}

	max17x0x_restore_cycle_counter(chip);

	if (chip->primary->irq) {
		ret = request_threaded_irq(chip->primary->irq, NULL,
					   max1720x_fg_irq_thread_fn,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
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

	max17x0x_set_serial_number(chip);

	INIT_WORK(&chip->cycle_count_work, max17x0x_cycle_count_work);
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

	max1720x_cleanup_history(chip);
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
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_DESCRIPTION("MAX17x01/MAX17x05 Fuel Gauge");
MODULE_LICENSE("GPL");
