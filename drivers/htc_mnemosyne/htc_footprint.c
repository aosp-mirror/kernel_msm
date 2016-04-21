/* arch/arm/mach-msm/htc_footprint.c
 * Copyright (C) 2013 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <asm/smp_plat.h>
#include "../../../drivers/power/qcom/idle.h"
#include <htc_mnemosyne/htc_mnemosyne.h>
#include <htc_mnemosyne/htc_footprint.h>

#define APPS_WDOG_FOOT_PRINT_MAGIC	0xACBDFE00
#define HOTPLUG_ON_MAGIC		0xACBDBB00
#define MNEMOSYNE_MAX_CPUS_PER_CLUSTER	2
#define MNEMOSYNE_NR_CPUS		4

#define S2H(c)		cpu_sw_idx_to_hw_idx(c)
/* Translate kernel cpu index to hw cpu index according to MPIDR.
 * Ex: for big-Little clusters, big core 0 is kernel core 0,
 *     but its MPIDR might be 0x101.
 *     TZ or asm codes will simply translate its index to Aff1*MNEMOSYNE_MAX_CPUS_PER_CLUSTER + CPUID.
 * To be consistent, we use hw cpu index for all scenarios.
 */
static inline int cpu_sw_idx_to_hw_idx(int cpu)
{
	uint32_t cluster_id = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
	uint32_t cpu_id = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0);

	if (cluster_id >= MAX_NUM_CLUSTER || cpu_id >= MNEMOSYNE_MAX_CPUS_PER_CLUSTER) {
		WARN(1, "cluster_id = %d, cpu_id = %d are not valid.\n", cluster_id, cpu_id);
		return 0;
	}

	return (cluster_id * MNEMOSYNE_MAX_CPUS_PER_CLUSTER + cpu_id);
}

int clk_get_cpu_idx(struct clk *c);

int __weak clk_get_cpu_idx(struct clk *c)
{
	WARN_ONCE(1, "WARNING: %s is not implemented. Related footprints won't be written.\n", __func__);
	return -1;
}

int clk_get_l2_idx(struct clk *c);

int __weak clk_get_l2_idx(struct clk *c)
{
	WARN_ONCE(1, "WARNING: %s is not implemented. Related footprints won't be written.\n", __func__);
	return -1;
}

int read_backup_cc_uah(void)
{
	int cur_batt_magic, cur_cc_backup_uah;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_cc_backup_uah = MNEMOSYNE_GET(cc_backup_uah);

	pr_info("%s: cc_backup_uah=%d, magic=%x\n", __func__,
			cur_cc_backup_uah, cur_batt_magic);
	if((cur_batt_magic & 0xFFFFFF00) == MAGIC_NUM_FOR_BATT_SAVE) {
		if ((cur_batt_magic & 0xFF) <= BATT_SAVE_MASK) {
			if ((cur_batt_magic & HTC_BATT_SAVE_CC)
					== HTC_BATT_SAVE_CC)
				return cur_cc_backup_uah;
		}
	}
	return 0;
}

void write_backup_cc_uah(int cc_reading)
{
	int cur_batt_magic, cur_cc_backup_uah;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_cc_backup_uah = MNEMOSYNE_GET(cc_backup_uah);

	pr_info("%s: ori cc_backup_uah=%d, cc_reading=%d, magic_num=%x\n",
		__func__, cur_cc_backup_uah, cc_reading, cur_batt_magic);
	if ((cur_batt_magic & ~BATT_SAVE_MASK) != MAGIC_NUM_FOR_BATT_SAVE)
		cur_batt_magic = MAGIC_NUM_FOR_BATT_SAVE;

	cur_batt_magic |= HTC_BATT_SAVE_CC;

	MNEMOSYNE_SET(batt_magic, cur_batt_magic);
	MNEMOSYNE_SET(cc_backup_uah, cc_reading);
	mb();
}

int read_backup_ocv_uv(void)
{
	int cur_batt_magic, cur_ocv_backup_uv;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_ocv_backup_uv = MNEMOSYNE_GET(ocv_backup_uv);

	pr_info("%s: ocv_backup_uv=%d, magic=%x\n", __func__,
			cur_ocv_backup_uv, cur_batt_magic);
	if((cur_batt_magic & 0xFFFFFF00) == MAGIC_NUM_FOR_BATT_SAVE) {
		if ((cur_batt_magic & 0xFF) <= BATT_SAVE_MASK) {
			if ((cur_batt_magic & HTC_BATT_SAVE_OCV_UV)
					== HTC_BATT_SAVE_OCV_UV)
				return cur_ocv_backup_uv;
		}
	}
	return 0;
}

void write_backup_ocv_uv(int ocv_backup)
{
	int cur_batt_magic, cur_ocv_backup_uv;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_ocv_backup_uv = MNEMOSYNE_GET(ocv_backup_uv);
	pr_info("%s: ori ocv_backup_uv=%d, ocv_backup=%d, magic_num=%x\n",
		__func__, cur_ocv_backup_uv, ocv_backup, cur_batt_magic);
	if((cur_batt_magic & ~BATT_SAVE_MASK) != MAGIC_NUM_FOR_BATT_SAVE)
		cur_batt_magic = MAGIC_NUM_FOR_BATT_SAVE;

	cur_batt_magic |= HTC_BATT_SAVE_OCV_UV;

	MNEMOSYNE_SET(batt_magic, cur_batt_magic);
	MNEMOSYNE_SET(ocv_backup_uv, ocv_backup);
	mb();
}

void set_msm_watchdog_en_footprint(int enable)
{
	MNEMOSYNE_SET(apps_watchdog_en, (APPS_WDOG_FOOT_PRINT_MAGIC | enable));
	mb();
}

/* UTC time depends on linux timekeeping.
 * timekeeping will be suspended during suspend flow.
 * We need to handle this footprint separatedly.
 */
void set_msm_watchdog_pet_time_utc(void)
{
	struct timespec ts;

	getnstimeofday(&ts);

	MNEMOSYNE_SET(apps_watchdog_pet_utc, ts.tv_sec);

	mb();
}

void set_msm_watchdog_pet_footprint(void __iomem *sleep_clk_base)
{
	uint32_t sleep_clk = 0;
	unsigned long long timestamp_ms;

	if (sleep_clk_base)
		sleep_clk = __raw_readl(sleep_clk_base);
	else
		pr_warn("%s: sleep clk base is not valid.\n", __func__);
	timestamp_ms = sched_clock();

	do_div(timestamp_ms, NSEC_PER_MSEC);

	/* If watchdog disable is deferred to actual suspend call,
	 * there will WARN message while calling getnstimeofday
	 * because timekeeping is suspended during syscore suspend flow.
	 */
	if (likely(!timekeeping_suspended)) {
		set_msm_watchdog_pet_time_utc();
	}

	MNEMOSYNE_SET(apps_watchdog_pet, sleep_clk);
	MNEMOSYNE_SET(apps_watchdog_pet_schedclk, (unsigned long)timestamp_ms);
	mb();
}

void set_acpuclk_footprint(unsigned cpu, unsigned state)
{
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(acpuclk_set_rate_footprint_cpu, S2H(cpu), (CPU_FOOT_PRINT_MAGIC | state));
	mb();
}

/* for those cpu clks integrated to clk subsystem,
 * cpu index is not passed directly and several cpus share one clk.
 * use this function to set footprint.
 *
 * `clk_get_cpu_idx' must be implemented in clock-cpu-<chipset>.c
 * to map clk to effective cpu index.
 *
 * clk_get_cpu_idx returns positive number as cpu index,
 * otherwise negative number for a non-cpu clk.
 */
void set_acpuclk_footprint_by_clk(struct clk* c, unsigned state)
{
	int cpu = clk_get_cpu_idx(c);

	/* not a valid cpu clk */
	if (cpu < 0)
		return;

	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	/* now we can set desired footprint. */
	set_acpuclk_footprint(cpu, state);
}

void set_acpuclk_cpu_freq_footprint(enum FREQ_TYPE type, unsigned cpu, unsigned khz)
{
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	switch (type) {
		case FT_PREV_RATE:
			MNEMOSYNE_SET_I(cpu_prev_frequency, S2H(cpu), khz);
			break;
		case FT_CUR_RATE:
			MNEMOSYNE_SET_I(cpu_frequency, S2H(cpu), khz);
			break;
		case FT_NEW_RATE:
			MNEMOSYNE_SET_I(cpu_new_frequency, S2H(cpu), khz);
			break;
	}
	mb();
}

void set_acpuclk_cpu_freq_footprint_by_clk(enum FREQ_TYPE type, struct clk *c, unsigned khz)
{
	int cpu = clk_get_cpu_idx(c);

	/* not a valid cpu clk */
	if (cpu < 0)
		return;

	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	set_acpuclk_cpu_freq_footprint(type, cpu, khz);
}

void set_acpuclk_l2_freq_footprint(enum FREQ_TYPE type, unsigned khz)
{
	switch (type) {
		case FT_PREV_RATE:
			MNEMOSYNE_SET(l2_prev_frequency, khz);
			break;
		case FT_CUR_RATE:
			MNEMOSYNE_SET(l2_frequency, khz);
			break;
		case FT_NEW_RATE:
			MNEMOSYNE_SET(l2_new_frequency, khz);
			break;
	}
	mb();
}

void set_acpuclk_l2_freq_footprint_by_clk(enum FREQ_TYPE type, struct clk* c, unsigned khz)
{
	int l2 = clk_get_l2_idx(c);

	/* not a valid l2 clk */
	if (l2 < 0)
		return;

	set_acpuclk_l2_freq_footprint(type, khz);
}

void inc_kernel_exit_counter_from_pc(unsigned cpu)
{
	unsigned int counter;
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to increase kernel exit counter from PC for core %d\n",
			MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	counter = MNEMOSYNE_GET_I(kernel_exit_counter_from_cpu, S2H(cpu)) + 1;
	MNEMOSYNE_SET_I(kernel_exit_counter_from_cpu, S2H(cpu), counter);
	mb();
}

/* Status Check:
 *          from_idle, notify_rpm, core
 *       PC:    false,       true,    0
 *  IDLE PC:     true,       true,  0-3
 *      FPC:    false,      false,  0-3
 * IDLE FPC:     true,      false,  0-3
 *  HOTPLUG:    false,       true,  1-3
 */
void init_cpu_foot_print(unsigned cpu, bool from_idle, bool notify_rpm)
{
	unsigned state = CPU_FOOT_PRINT_MAGIC_HOTPLUG;
	bool is_FPC = !notify_rpm;
	bool not_hotplug = !cpu || from_idle;

	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	if (not_hotplug) {
		if (is_FPC)
			state = (from_idle) ? CPU_FOOT_PRINT_MAGIC_FPC_FROM_IDLE : CPU_FOOT_PRINT_MAGIC_FPC;
		else
			state = (from_idle) ? CPU_FOOT_PRINT_MAGIC_FROM_IDLE : CPU_FOOT_PRINT_MAGIC;
	}

	MNEMOSYNE_SET_I(kernel_footprint_cpu, S2H(cpu), state);
	mb();
}

void set_cpu_foot_print(unsigned cpu, unsigned state)
{
	unsigned mask = 0xFF;
	unsigned new_state;

	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	new_state = (MNEMOSYNE_GET_I(kernel_footprint_cpu, S2H(cpu)) & ~mask) | (state & mask);

	MNEMOSYNE_SET_I(kernel_footprint_cpu, S2H(cpu), new_state)
	mb();
}

void clean_reset_vector_debug_info(unsigned cpu)
{
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(reset_vector_for_cpu, S2H(cpu), RESET_VECTOR_CLEAN_MAGIC);
	mb();
}

void set_reset_vector(unsigned cpu)
{
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(reset_vector_for_cpu, S2H(cpu), msm_pm_boot_vector[S2H(cpu)]);
	mb();
}

void set_reset_vector_address_after_pc(unsigned cpu)
{
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(cpu_reset_vector_address, S2H(cpu), virt_to_phys(&msm_pm_boot_vector[S2H(cpu)]));
	mb();
}

void set_reset_vector_value_after_pc(unsigned cpu)
{
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(cpu_reset_vector_address_value, S2H(cpu), msm_pm_boot_vector[S2H(cpu)]);
	mb();
}

void store_pm_boot_entry_addr(void)
{
	MNEMOSYNE_SET(msm_pm_boot_entry, virt_to_phys(msm_pm_boot_entry));
	mb();
}

void store_pm_boot_vector_addr(u64 value)
{
	MNEMOSYNE_SET(msm_pm_boot_vector, value);
	mb();
}

void set_hotplug_on_footprint(unsigned cpu, unsigned value)
{
	if (unlikely(cpu >= MNEMOSYNE_NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", MNEMOSYNE_NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(cpu_hotplug_on, S2H(cpu), HOTPLUG_ON_MAGIC | (value & 0xFF));
	mb();
}

/*
 * This function will initialize cpu footprint on booting a device.
 * Core0 is on already to execute these codes.
 * We consider it exiting from PC, because core0 is never hot-plugged.
 * All cores except core0 are taken from reset state, so it goes through hotplug path.
 */
static int __init boot_init_footprint(void)
{
	int i;

	/* we cannot guarantee order of the same level initcall. */
	if (!mnemosyne_is_ready()) {
		int ret = mnemosyne_early_init();
		if (ret != 0) {
			pr_warn("%s: mnemosyne init failed (%d).\n", __func__, ret);
			return ret;
		}
	}

	for (i = 0; i < MNEMOSYNE_NR_CPUS; i++) {
		MNEMOSYNE_SET_I(kernel_footprint_cpu, S2H(i), CPU_FOOT_PRINT_MAGIC_HOTPLUG | 0x1);
		MNEMOSYNE_SET_I(kernel_exit_counter_from_cpu, S2H(i), 0x0);
	}
	/* 0xb is the last footprint of power collapse path. */
	/* others are still off now, they are considered as hotplugged cpus. */
	MNEMOSYNE_SET_I(kernel_footprint_cpu, S2H(0), CPU_FOOT_PRINT_MAGIC | 0xb);
	mb();
	pr_info("%s: htc footprint init done.\n", __func__);

	return 0;
}
early_initcall(boot_init_footprint);
