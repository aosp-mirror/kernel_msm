/*
 * UFS impaired storage emulation.
 *
 * Copyright (C) 2018, Google, Inc.
 *
 * Authors:
 *	Hyojun Kim <hyojun@google.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * See the COPYING file in the top-level directory or visit
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This program is provided "AS IS" and "WITH ALL FAULTS" and
 * without warranty of any kind. You are solely responsible for
 * determining the appropriateness of using and distributing
 * the program and assume all risks associated with your exercise
 * of rights with respect to the program, including but not limited
 * to infringement of third party rights, the risks and costs of
 * program errors, damage to or loss of data, programs or equipment,
 * and unavailability or interruption of operations. Under no
 * circumstances will the contributor of this Program be liable for
 * any damages of any kind arising from your use or distribution of
 * this program.
 *
 * The Linux Foundation chooses to take subject only to the GPLv2
 * license terms, and distributes only under these terms.
 */

#include <linux/random.h>
#include "ufshcd.h"

#define UFS_IMPAIRED_SLEEP_US			(500)
#define UFS_IMPAIRED_ERROR_US			(10)

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL

/**
 *  Latency distribution model for small (<128KiB) read requests.
 *    - captured from Micron 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_small_read[] = {
	86, 87, 89, 90, 91, 92, 93, 96,
	98, 100, 101, 103, 105, 106, 108, 108,
	109, 111, 113, 115, 117, 120, 122, 124,
	127, 130, 134, 136, 138, 139, 140, 141,
	143, 144, 145, 147, 149, 151, 153, 155,
	157, 159, 162, 164, 168, 172, 176, 180,
	183, 187, 191, 194, 196, 199, 203, 207,
	211, 215, 218, 223, 227, 232, 237, 242,
	246, 251, 256, 259, 264, 269, 275, 278,
	283, 288, 294, 299, 304, 310, 315, 321,
	326, 333, 338, 344, 348, 355, 361, 367,
	373, 379, 385, 391, 398, 406, 412, 420,
	425, 433, 441, 448, 457, 464, 471, 479,
	489, 500, 508, 517, 527, 535, 544, 555,
	565, 576, 589, 600, 605, 610, 616, 621,
	626, 632, 637, 642, 648, 651, 654, 657,
	661, 664, 667, 672, 678, 684, 689, 694,
	699, 704, 708, 713, 719, 724, 729, 735,
	742, 748, 756, 764, 770, 777, 787, 795,
	804, 812, 821, 831, 840, 849, 858, 866,
	876, 885, 894, 903, 911, 920, 930, 939,
	946, 957, 966, 979, 987, 997, 1006, 1017,
	1027, 1036, 1049, 1062, 1073, 1087, 1100, 1110,
	1121, 1134, 1145, 1159, 1172, 1188, 1203, 1221,
	1239, 1258, 1276, 1288, 1307, 1330, 1353, 1376,
	1403, 1427, 1453, 1484, 1509, 1537, 1567, 1596,
	1629, 1656, 1704, 1744, 1782, 1847, 1893, 1957,
	2011, 2068, 2131, 2204, 2277, 2359, 2443, 2545,
	2645, 2730, 2832, 2965, 3064, 3193, 3297, 3436,
	3558, 3670, 3829, 4025, 4241, 4482, 4786, 5266,
	5709, 6366, 8044, 10499, 11754, 11806, 11850, 11895,
	11943, 11978, 12009, 12039, 12092, 12181, 12663, 50713,
};

/**
 *  Latency distribution model for large (>=128KiB) read requests.
 *    - captured from Micron 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_large_read[] = {
	324, 364, 380, 382, 384, 385, 387, 387,
	388, 389, 390, 390, 390, 391, 391, 391,
	391, 391, 392, 393, 393, 393, 393, 393,
	393, 394, 394, 395, 395, 395, 395, 395,
	396, 396, 396, 396, 396, 396, 396, 397,
	398, 398, 398, 398, 398, 398, 398, 398,
	399, 399, 399, 399, 399, 399, 400, 401,
	401, 401, 402, 403, 403, 404, 404, 405,
	406, 409, 410, 411, 412, 414, 415, 417,
	418, 420, 422, 423, 426, 428, 431, 437,
	441, 444, 449, 453, 460, 463, 468, 471,
	474, 476, 480, 482, 484, 487, 490, 492,
	495, 499, 503, 508, 512, 516, 520, 524,
	529, 536, 540, 546, 552, 557, 565, 572,
	580, 595, 601, 614, 622, 631, 640, 645,
	654, 668, 676, 684, 697, 707, 718, 731,
	748, 760, 775, 794, 802, 815, 821, 836,
	845, 866, 877, 889, 900, 907, 911, 918,
	927, 943, 958, 979, 987, 997, 1005, 1010,
	1021, 1035, 1048, 1059, 1078, 1095, 1105, 1121,
	1164, 1180, 1210, 1236, 1259, 1286, 1314, 1364,
	1403, 1449, 1481, 1548, 1602, 1615, 1630, 1659,
	1686, 1712, 1776, 1817, 1855, 1905, 1994, 2029,
	2097, 2164, 2246, 2318, 2382, 2448, 2516, 2547,
	2615, 2663, 2733, 2834, 2921, 2987, 3052, 3113,
	3224, 3292, 3373, 3494, 3615, 3717, 3805, 3958,
	4100, 4225, 4380, 4452, 4557, 4637, 4794, 5033,
	5341, 5748, 6100, 6638, 7473, 8374, 11764, 11932,
	11999, 12024, 12033, 12037, 12040, 12044, 12047, 12049,
	12053, 12056, 12060, 12062, 12064, 12069, 12071, 12079,
	12087, 12101, 12128, 12142, 12150, 12158, 12169, 12184,
	12200, 12211, 12224, 12275, 12347, 12464, 13961, 50629,
};

/**
 *  Latency distribution model for small (<128KiB) write requests.
 *    - captured from Samsung 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_small_write[] = {
	41, 42, 43, 43, 43, 43, 43, 43,
	43, 43, 43, 43, 44, 44, 44, 44,
	44, 44, 45, 45, 45, 45, 46, 46,
	46, 46, 46, 46, 46, 46, 47, 47,
	47, 48, 49, 49, 49, 49, 50, 50,
	50, 51, 52, 54, 54, 55, 57, 58,
	59, 60, 62, 62, 63, 65, 66, 67,
	70, 73, 74, 75, 78, 81, 82, 85,
	86, 89, 90, 92, 94, 95, 100, 103,
	106, 109, 111, 117, 121, 125, 131, 134,
	143, 151, 158, 164, 169, 176, 183, 190,
	197, 205, 214, 221, 228, 230, 232, 235,
	238, 248, 260, 273, 283, 294, 314, 328,
	347, 372, 418, 469, 501, 528, 559, 585,
	598, 607, 621, 633, 658, 671, 725, 774,
	810, 833, 842, 847, 853, 861, 873, 884,
	893, 904, 916, 935, 954, 960, 962, 963,
	965, 966, 968, 971, 974, 978, 981, 982,
	983, 986, 988, 991, 996, 999, 1003, 1008,
	1013, 1018, 1025, 1030, 1036, 1043, 1048, 1052,
	1057, 1061, 1066, 1072, 1080, 1085, 1095, 1103,
	1121, 1154, 1183, 1210, 1245, 1270, 1307, 1340,
	1385, 1427, 1447, 1473, 1502, 1530, 1588, 1620,
	1671, 1727, 1788, 1822, 1871, 2019, 2107, 2161,
	2202, 2267, 2324, 2339, 2347, 2351, 2358, 2363,
	2367, 2371, 2377, 2383, 2386, 2390, 2395, 2401,
	2411, 2416, 2422, 2455, 2496, 2555, 2626, 2793,
	2883, 2956, 3075, 3104, 3136, 3257, 3720, 3990,
	4569, 5193, 5462, 5599, 5774, 5965, 6304, 6350,
	6367, 6460, 6589, 6648, 7090, 7666, 7921, 8528,
	8620, 8758, 8933, 9089, 9310, 10996, 13298, 15989,
	27039, 111816, 228143, 242631, 249738, 277515, 344243, 644551,
};

/**
 *  Latency distribution model for large (>=128KiB) write requests.
 *    - captured from Samsung 64GB / DoU App / fragftl90 precondition.
 */
static int impaired_latency_large_write[] = {
	931, 1368, 1597, 1717, 1770, 2102, 2190, 2203,
	2227, 2321, 2651, 2964, 3044, 3107, 3125, 3187,
	3619, 3740, 4298, 4404, 4500, 4518, 4536, 4612,
	4680, 5248, 5408, 5803, 6486, 7171, 7603, 8139,
	8590, 8934, 9661, 10135, 10815, 11387, 11653, 12698,
	13028, 13461, 13918, 14262, 14653, 15023, 16056, 16959,
	18488, 19161, 20670, 21717, 22937, 23748, 25017, 26392,
	28037, 30083, 31255, 32115, 33473, 33867, 35541, 36389,
	36881, 37604, 38095, 38809, 40167, 42621, 45367, 49762,
	51501, 55550, 58046, 59042, 59974, 60293, 61093, 63117,
	65404, 66943, 67522, 69874, 74325, 76652, 79318, 80906,
	82579, 83591, 85002, 86212, 87530, 88704, 90705, 93660,
	96113, 100477, 103027, 104373, 106349, 106865, 107940, 109363,
	110861, 112585, 114433, 117842, 121719, 124761, 127053, 127735,
	128408, 130274, 131996, 133333, 136319, 138339, 140988, 143100,
	145622, 148063, 149843, 151187, 153021, 155009, 157893, 160171,
	162224, 165607, 168586, 172387, 173645, 174777, 176540, 178189,
	179568, 182432, 186736, 190361, 192542, 195341, 196778, 197484,
	198562, 200706, 203469, 208503, 210125, 213157, 216006, 218331,
	220201, 221135, 222878, 224437, 226826, 231460, 234924, 236136,
	238898, 240970, 243045, 245437, 246771, 249320, 255538, 257795,
	261502, 266142, 270622, 273351, 274984, 282415, 289584, 295506,
	299918, 307177, 310620, 317469, 330186, 339543, 353783, 365386,
	384748, 418406, 452769, 487990, 522070, 531204, 539830, 548083,
	551891, 554911, 560052, 563793, 564732, 565626, 567059, 571978,
	574526, 576305, 578064, 579098, 581912, 582565, 582703, 583912,
	585966, 588347, 590459, 592906, 595216, 597190, 599410, 603470,
	605402, 606438, 609488, 612438, 615461, 618508, 620230, 620825,
	624151, 624814, 625431, 626220, 627428, 628625, 629637, 630157,
	631669, 633001, 633431, 634438, 635181, 635682, 637344, 637964,
	638930, 639790, 640542, 641104, 641752, 642765, 643495, 644437,
	645105, 646311, 648009, 652824, 658469, 680256, 709137, 735646,
};

#define MODEL_SIZE	(sizeof(impaired_latency_large_write) / sizeof(int))
#define REQSZ_THRESHOLD	(128 * 1024)

#endif

/**
 *  Impaired storage sysfs entries
 */
struct impaired_attr {
	struct attribute attr;

	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);

	ssize_t (*store)(struct kobject *kobj, struct attribute *attr,
			 char *buf, size_t count);
};

struct impaired_delay_attr {
	struct attribute attr;

	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);

	ssize_t (*store)(struct kobject *kobj, struct attribute *attr,
			 char *buf, size_t count);
	enum ufs_impaired_optype optype;
	enum ufs_impaired_delaytype delaytype;
	u32 maximum;
};

struct impaired_stat_attr {
	struct attribute attr;

	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);

	ssize_t (*store)(struct kobject *kobj, struct attribute *attr,
			 char *buf, size_t count);
	enum ufs_impaired_optype optype;
	enum ufs_impaired_stattype stattype;
};

static inline struct ufs_hba *__get_hba_from_kobj(struct kobject *obj)
{
	struct device *dev = container_of(obj->parent, struct device, kobj);
	return dev_get_drvdata(dev);
}

static ssize_t
impaired_show_enabled(struct kobject *obj, struct attribute *attr, char *buf)
{
	struct ufs_hba *hba = __get_hba_from_kobj(obj);
	return snprintf(buf, PAGE_SIZE, "%d\n", hba->impaired.enabled);
}

static int __impaired_thread_fn(void *param);

static ssize_t
impaired_store_enabled(struct kobject *obj, struct attribute *attr, char *buf,
		size_t count)
{
	struct ufs_hba *hba = __get_hba_from_kobj(obj);
	unsigned long flags, value;

	if (kstrtol(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&hba->impaired_thread_mutex);
	if (value && !hba->impaired_thread)
		hba->impaired_thread =
			kthread_run(__impaired_thread_fn, hba, "impaired");
	mutex_unlock(&hba->impaired_thread_mutex);

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->impaired.enabled = (value) ? 1 : 0;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

static ssize_t
impaired_show_delay(struct kobject *obj, struct attribute *_attr, char *buf)
{
	struct ufs_hba *hba = __get_hba_from_kobj(obj);
	struct impaired_delay_attr *attr = (struct impaired_delay_attr *)_attr;
	u32 value = hba->impaired.io[attr->optype].delay[attr->delaytype];

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	if (attr->delaytype == UFS_IMPAIRED_DELAYTYPE_MODEL) {
		return snprintf(buf, PAGE_SIZE,
				(value) ? "fragftl\n" : "none\n");
	}
#endif
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t
impaired_store_delay(struct kobject *obj, struct attribute *_attr, char *buf,
		size_t count)
{
	struct ufs_hba *hba = __get_hba_from_kobj(obj);
	struct impaired_delay_attr *attr = (struct impaired_delay_attr *)_attr;
	unsigned long flags, value;

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	if (attr->delaytype == UFS_IMPAIRED_DELAYTYPE_MODEL) {
		if (strncmp(buf, "none", 4) == 0)
			value = 0;
		else if (strncmp(buf, "fragftl", 7) == 0)
			value = 1;
		else
			return -EINVAL;
	} else if (kstrtol(buf, 0, &value)) {
		return -EINVAL;
	}
#else
	if (kstrtol(buf, 0, &value))
		return -EINVAL;
#endif

	if (value > attr->maximum)
		return -EINVAL;

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->impaired.io[attr->optype].delay[attr->delaytype] = (u32)value;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

static ssize_t
impaired_show_stat(struct kobject *obj, struct attribute *_attr, char *buf)
{
	struct ufs_hba *hba = __get_hba_from_kobj(obj);
	struct impaired_stat_attr *attr = (struct impaired_stat_attr *)_attr;

	return snprintf(buf, PAGE_SIZE, "%lld\n",
			hba->impaired.io[attr->optype].stat[attr->stattype]);
}

static ssize_t
impaired_store_stat(struct kobject *obj, struct attribute *_attr, char *buf,
		size_t count)
{
	struct ufs_hba *hba = __get_hba_from_kobj(obj);
	struct impaired_stat_attr *attr = (struct impaired_stat_attr *)_attr;
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	memset(&hba->impaired.io[attr->optype].stat, 0,
			sizeof(hba->impaired.io[attr->optype].stat));
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return count;
}

#define IMPAIRED_ATTR_RW(_name) 					\
	static struct impaired_attr impaired_##_name = {		\
		.attr = {.name = __stringify(_name),			\
			 .mode = 0644 },				\
		.show = impaired_show_##_name, 				\
		.store = impaired_store_##_name,			\
	}

#define IMPAIRED_DELAY_ATTR(_name, _op, _delay, _maximum)		\
	static struct impaired_delay_attr 				\
	impaired_##_name = { 						\
		.attr = {.name = __stringify(_name),			\
			 .mode = 0644 },				\
		.show = impaired_show_delay, 				\
		.store = impaired_store_delay,				\
		.optype = _op,						\
		.delaytype = _delay,					\
		.maximum = _maximum,					\
	}

#define IMPAIRED_STAT_ATTR(_name, _op, _stat)				\
	static struct impaired_stat_attr 				\
	impaired_##_name = { 						\
		.attr = {.name = __stringify(_name),			\
			 .mode = 0644 },				\
		.show = impaired_show_stat, 				\
		.store = impaired_store_stat,				\
		.optype = _op,						\
		.stattype = _stat,					\
	}

/* sysfs impaired: root */
IMPAIRED_ATTR_RW(enabled);

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
IMPAIRED_DELAY_ATTR(read_model, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_DELAYTYPE_MODEL, 1);
#endif
IMPAIRED_DELAY_ATTR(read_delay_us, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_DELAYTYPE_US, 1000000);
IMPAIRED_DELAY_ATTR(read_delay_percent, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_DELAYTYPE_PERCENT, 100000);

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
IMPAIRED_DELAY_ATTR(write_model, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_DELAYTYPE_MODEL, 1);
#endif
IMPAIRED_DELAY_ATTR(write_delay_us, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_DELAYTYPE_US, 1000000);
IMPAIRED_DELAY_ATTR(write_delay_percent, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_DELAYTYPE_PERCENT, 100000);

/* sysfs impaired/advanced_control */
IMPAIRED_DELAY_ATTR(read_max_delayed_us, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_DELAYTYPE_MAX, 2000000);
IMPAIRED_DELAY_ATTR(read_skip_delay_cnt, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_DELAYTYPE_SKIP, 2000000);

IMPAIRED_DELAY_ATTR(write_max_delayed_us, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_DELAYTYPE_MAX, 2000000);
IMPAIRED_DELAY_ATTR(write_skip_delay_cnt, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_DELAYTYPE_SKIP, 2000000);

/* sysfs impaired/stat */
IMPAIRED_STAT_ATTR(total_read_cnt, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_STATTYPE_TOTAL_CNT);
IMPAIRED_STAT_ATTR(total_read_original_us, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_STATTYPE_TOTAL_ORIGINAL_US);
IMPAIRED_STAT_ATTR(total_read_delayed_us, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US);

IMPAIRED_STAT_ATTR(total_write_cnt, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_STATTYPE_TOTAL_CNT);
IMPAIRED_STAT_ATTR(total_write_original_us, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_STATTYPE_TOTAL_ORIGINAL_US);
IMPAIRED_STAT_ATTR(total_write_delayed_us, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US);

/* sysfs impaired/delay_stat */
IMPAIRED_STAT_ATTR(max_read_delay_us, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_STATTYPE_MAX_DELAY);
IMPAIRED_STAT_ATTR(max_read_delay_error_us, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_STATTYPE_MAX_ERROR);
IMPAIRED_STAT_ATTR(total_read_delay_error_us, UFS_IMPAIRED_OPTYPE_READ, \
		UFS_IMPAIRED_STATTYPE_TOTAL_DELAY_ERROR);

IMPAIRED_STAT_ATTR(max_write_delay_us, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_STATTYPE_MAX_DELAY);
IMPAIRED_STAT_ATTR(max_write_delay_error_us, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_STATTYPE_MAX_ERROR);
IMPAIRED_STAT_ATTR(total_write_delay_error_us, UFS_IMPAIRED_OPTYPE_WRITE, \
		UFS_IMPAIRED_STATTYPE_TOTAL_DELAY_ERROR);

static struct attribute *impaired_attrs[] = {
	/* Basic delay control */
	&impaired_enabled.attr,

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	&impaired_read_model.attr,
#endif
	&impaired_read_delay_us.attr,
	&impaired_read_delay_percent.attr,

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	&impaired_write_model.attr,
#endif
	&impaired_write_delay_us.attr,
	&impaired_write_delay_percent.attr,
	NULL
};

const struct attribute_group impaired_group = {
	.attrs = impaired_attrs,
};

static struct attribute *impaired_advanced_attrs[] = {
	/* Advanced delay control */
	&impaired_read_max_delayed_us.attr,
	&impaired_read_skip_delay_cnt.attr,

	&impaired_write_max_delayed_us.attr,
	&impaired_write_skip_delay_cnt.attr,
	NULL
};

const struct attribute_group impaired_advanced_group = {
	.name = "advanced_control",
	.attrs = impaired_advanced_attrs,
};

static struct attribute *impaired_stat_attrs[] = {
	/* Statistics */
	&impaired_total_read_cnt.attr,
	&impaired_total_read_original_us.attr,
	&impaired_total_read_delayed_us.attr,

	&impaired_total_write_cnt.attr,
	&impaired_total_write_original_us.attr,
	&impaired_total_write_delayed_us.attr,
	NULL
};

const struct attribute_group impaired_stat_group = {
	.name = "stat",
	.attrs = impaired_stat_attrs,
};

static struct attribute *impaired_delay_stat_attrs[] = {
	/* Delay related statistics */
	&impaired_max_read_delay_us.attr,
	&impaired_max_read_delay_error_us.attr,
	&impaired_total_read_delay_error_us.attr,

	&impaired_max_write_delay_us.attr,
	&impaired_max_write_delay_error_us.attr,
	&impaired_total_write_delay_error_us.attr,
	NULL
};

const struct attribute_group impaired_delay_stat_group = {
	.name = "delay_stat",
	.attrs = impaired_delay_stat_attrs,
};

const struct attribute_group *impaired_sysfs_groups[] = {
	&impaired_group,
	&impaired_advanced_group,
	&impaired_stat_group,
	&impaired_delay_stat_group,
	NULL
};

void ufs_impaired_init_sysfs(struct ufs_hba *hba)
{
	hba->impaired_kobj = kobject_create_and_add("impaired",
			&hba->dev->kobj);
	if (!hba->impaired_kobj)
		dev_err(hba->dev, "Failed to create impaired storage sysfs folder\n");

	if (sysfs_create_groups(hba->impaired_kobj, impaired_sysfs_groups))
		dev_err(hba->dev, "Failed to create impaired storage sysfs group\n");
}

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
/**
 * __pick_target_latency_with_model
 *   - pick a latency for a UFS request by using latency distribution models.
 * @op: operation type
 * @req_size: request size in bytes
 */
static int __pick_target_latency_with_model(
		enum ufs_impaired_optype op, u32 req_size)
{
	unsigned int idx;
	int *model;

	/* Pick a model based on request type and size */
	if (op == UFS_IMPAIRED_OPTYPE_READ)
		model = (req_size < REQSZ_THRESHOLD) ?
			impaired_latency_small_read :
			impaired_latency_large_read;
	else
		model = (req_size < REQSZ_THRESHOLD) ?
			impaired_latency_small_write :
			impaired_latency_large_write;

	/* Generate random index and pick a target latency */
	prandom_bytes(&idx, sizeof(idx));
	return model[idx % MODEL_SIZE];
}
#endif

/**
 * __set_delay_in_lrb - pick a target delay and set lrbp->complete_time and
 *                      lrbp->target_compete_time fields.
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to add delay
 */
void __set_delay_in_lrb(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct scsi_cmnd *cmd = lrbp->cmd;
	struct request *rq = (cmd) ? cmd->request : NULL;
	s32 actual_us = ktime_us_delta(lrbp->complete_time_stamp,
				lrbp->issue_time_stamp);
	struct ufs_impaired_io *io;
	enum ufs_impaired_optype op;
	s32 target_us;

	/* Set default value: no delay */
	lrbp->complete_delay = 0;
	lrbp->target_complete_time = lrbp->complete_time_stamp;

	/* Return if delay is not needed */
	if (!hba->impaired.enabled ||
			!rq ||
			!(rq->cmd_type & REQ_TYPE_FS) ||
			(rq->cmd_flags & REQ_PREFLUSH) ||
			req_op(rq) == REQ_OP_FLUSH ||
			req_op(rq) == REQ_OP_DISCARD ||
			!rq->bio)
		return;

	op = (rq_data_dir(rq) == READ) ?
		UFS_IMPAIRED_OPTYPE_READ : UFS_IMPAIRED_OPTYPE_WRITE;
	io = &hba->impaired.io[op];

	/* Apply skip logic: delay one request after N skipping */
	if (io->delay[UFS_IMPAIRED_DELAYTYPE_SKIP] > io->skip_count++)
		return;
	io->skip_count = 0;

	/* Set default target latency */
	target_us = actual_us;

#ifdef CONFIG_SCSI_UFS_IMPAIRED_FRAGFTL
	/* If fragmented FTL emulation is on,
	   call __pick_target_latency_with_model() to pick a target latency */
	if (io->delay[UFS_IMPAIRED_DELAYTYPE_MODEL])
		target_us = __pick_target_latency_with_model(op,
				scsi_get_bytes(lrbp->cmd));
#endif

	/* Apply additional knobs */
	target_us = target_us * io->delay[UFS_IMPAIRED_DELAYTYPE_PERCENT] / 100;
	target_us += io->delay[UFS_IMPAIRED_DELAYTYPE_US];

	/* Apply maximum latency */
	if (io->delay[UFS_IMPAIRED_DELAYTYPE_MAX] &&
			target_us > io->delay[UFS_IMPAIRED_DELAYTYPE_MAX])
		target_us = io->delay[UFS_IMPAIRED_DELAYTYPE_MAX];

	/* Target latency cannot be smaller than the actual latency */
	if (target_us < actual_us)
		target_us = actual_us;

	/* Update delay related statistics */
	io->stat[UFS_IMPAIRED_STATTYPE_TOTAL_CNT] += 1;
	io->stat[UFS_IMPAIRED_STATTYPE_TOTAL_ORIGINAL_US] += actual_us;

	/* Delayed time will be added later when the request is completed */
	io->stat[UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US] += actual_us;

	/* Set delay fields */
	lrbp->complete_delay = (int)(target_us - actual_us);
	lrbp->target_complete_time = ktime_add_us(lrbp->complete_time_stamp,
			lrbp->complete_delay);

	/* Keep track of the maximum delay */
	if (io->stat[UFS_IMPAIRED_STATTYPE_MAX_DELAY] < lrbp->complete_delay)
		io->stat[UFS_IMPAIRED_STATTYPE_MAX_DELAY] =
			lrbp->complete_delay;
}


/**
 * __queue_lrb - put a lrb in to the delay list
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to be queued
 */
void __queue_lrb(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	struct list_head *parent;
	struct ufshcd_lrb *l;

	/* Find insertion point in the sorted list by target_complete_time */
	parent = &hba->impaired_list_head;
	list_for_each_entry(l, &hba->impaired_list_head, list) {
		if (ktime_compare(l->target_complete_time,
					lrbp->target_complete_time) > 0)
			break;
		parent = &l->list;
	}

	/* Insert the LRB at the right position */
	list_add(&lrbp->list, parent);

	/* Mark the LRB is being delayed */
	__set_bit(lrbp - hba->lrb, &hba->delayed_reqs);
}

/**
 * __update_statistics - update delay related statistics
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to update
 */
static void __update_statistics(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	enum ufs_impaired_optype op = (rq_data_dir(lrbp->cmd->request) == READ)
		? UFS_IMPAIRED_OPTYPE_READ : UFS_IMPAIRED_OPTYPE_WRITE;
	s64 delay_error;

	/* Final (actual) completion time with delay */
	ktime_t actual_completion_time = ktime_get();

	/* Update statistics */
	hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_TOTAL_DELAYED_US] +=
		ktime_us_delta(actual_completion_time,
				lrbp->complete_time_stamp);

	/* Monitor how much more delayed than intended */
	delay_error = ktime_us_delta(actual_completion_time,
			lrbp->target_complete_time);
	hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_TOTAL_DELAY_ERROR] +=
		delay_error;

	/* Keep track the maximum delay error */
	if (hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_MAX_ERROR]
			< delay_error)
		hba->impaired.io[op].stat[UFS_IMPAIRED_STATTYPE_MAX_ERROR] =
			delay_error;


	/* Update completion time for UFS driver stats */
	lrbp->complete_time_stamp = actual_completion_time;
}

/**
 * __impaired_thread_fn - kernel thread to complete requests
 * @param: kthread parameter (hba)
 */
static int __impaired_thread_fn(void *param)
{
	struct ufs_hba *hba = param;
	struct ufshcd_lrb *lrbp;
	unsigned long flags;
	s64 delay_us;

	spin_lock_irqsave(hba->host->host_lock, flags);
	while (!kthread_should_stop()) {
		if (list_empty(&hba->impaired_list_head)) {
			/* Go to sleep if the list is empty */
			spin_unlock_irqrestore(hba->host->host_lock, flags);

			usleep_range(UFS_IMPAIRED_SLEEP_US,
					UFS_IMPAIRED_SLEEP_US * 2);

			spin_lock_irqsave(hba->host->host_lock, flags);
			continue;
		}

		/* Get the first entry */
		lrbp = list_first_entry(&hba->impaired_list_head,
				struct ufshcd_lrb, list);

		/* Calculate needed delay time at this point */
		delay_us = ktime_us_delta(lrbp->target_complete_time,
				ktime_get());

		if (delay_us < UFS_IMPAIRED_ERROR_US) {
			/* Complete an LRB */
			list_del(&lrbp->list);
			__update_statistics(hba, lrbp);
			ufshcd_complete_lrb(hba, lrbp);
			wake_up(&hba->dev_cmd.tag_wq);
		} else {
			/* Do sleep */
			if (delay_us > UFS_IMPAIRED_SLEEP_US)
				delay_us = UFS_IMPAIRED_SLEEP_US;

			spin_unlock_irqrestore(hba->host->host_lock, flags);
			usleep_range(delay_us / 2, delay_us);
			spin_lock_irqsave(hba->host->host_lock, flags);
		}
	}

	/* Complete pending I/Os, if exist */
	list_for_each_entry(lrbp, &hba->impaired_list_head, list) {
		__update_statistics(hba, lrbp);
		ufshcd_complete_lrb(hba, lrbp);
	}

	/* Empty the queue */
	INIT_LIST_HEAD(&hba->impaired_list_head);
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return 0;
}

/**
 * ufs_impaired_delay_completion - delay an LRB.
 * @hba: per adapter instance
 * @lrbp: pointer to local reference block to be queued
 */
void ufs_impaired_delay_completion(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	/* Pick delay and set in LRB */
	__set_delay_in_lrb(hba, lrbp);

	/* If delay is not needed, complete and return.
	   Otherwise, put the LRB into the delay queue */
	if (!lrbp->complete_delay)
		ufshcd_complete_lrb(hba, lrbp);
	else {
		__queue_lrb(hba, lrbp);
		wake_up_process(hba->impaired_thread);
	}
}

/**
 * ufs_impaired_init - Start impaired storage emulation service.
 * @hba: per adapter instance
 */
void ufs_impaired_init(struct ufs_hba *hba)
{
	/* Set default value to the sysfs entries */
	hba->impaired.io[UFS_IMPAIRED_OPTYPE_READ].delay[
		UFS_IMPAIRED_DELAYTYPE_PERCENT] = 100;
	hba->impaired.io[UFS_IMPAIRED_OPTYPE_WRITE].delay[
		UFS_IMPAIRED_DELAYTYPE_PERCENT] = 100;

	/* Initialize impaired list */
	INIT_LIST_HEAD(&hba->impaired_list_head);

	/* Initialize a mutext to protect thread creation */
	mutex_init(&hba->impaired_thread_mutex);
}

/**
 * ufs_impaired_exit - Stop impaired storage emulation service.
 * @hba: per adapter instance
 */
void ufs_impaired_exit(struct ufs_hba *hba)
{
	mutex_lock(&hba->impaired_thread_mutex);
	if (!hba->impaired_thread) {
		mutex_unlock(&hba->impaired_thread_mutex);
		return;
	}
	mutex_unlock(&hba->impaired_thread_mutex);

	/* Wake up impaired_thread to let suicide */
	wake_up_process(hba->impaired_thread);
	kthread_stop(hba->impaired_thread);
}

