/*
 * Copyright (c) 2009 Google, Inc.
 * Copyright (c) 2008 QUALCOMM Incorporated.
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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/cpufreq.h>
#include <linux/regulator/consumer.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>

#include "acpuclock.h"
#include "proc_comm.h"

#if 0
#define DEBUG(x...) pr_info(x)
#else
#define DEBUG(x...) do {} while (0)
#endif

#define SHOT_SWITCH	4
#define HOP_SWITCH	5
#define SIMPLE_SLEW	6
#define COMPLEX_SLEW	7

#define SPSS_CLK_CNTL_ADDR	(MSM_CSR_BASE + 0x100)
#define SPSS_CLK_SEL_ADDR	(MSM_CSR_BASE + 0x104)

/* Scorpion PLL registers */
#define SCPLL_CTL_ADDR		(MSM_SCPLL_BASE + 0x4)
#define SCPLL_STATUS_ADDR	(MSM_SCPLL_BASE + 0x18)
#define SCPLL_FSM_CTL_EXT_ADDR	(MSM_SCPLL_BASE + 0x10)

struct clkctl_acpu_speed {
	unsigned acpu_khz;
	unsigned clk_cfg;
	unsigned clk_sel;
	unsigned sc_l_value;
	unsigned lpj;
	int      vdd;
};

/* clock sources */
#define CLK_TCXO	0 /* 19.2 MHz */
#define CLK_GLOBAL_PLL	1 /* 768 MHz */
#define CLK_MODEM_PLL	4 /* 245 MHz (UMTS) or 235.93 MHz (CDMA) */

#define CCTL(src, div) (((src) << 4) | (div - 1))

/* core sources */
#define SRC_RAW		0 /* clock from SPSS_CLK_CNTL */
#define SRC_SCPLL	1 /* output of scpll 128-998 MHZ */
#define SRC_AXI		2 /* 128 MHz */
#define SRC_PLL1	3 /* 768 MHz */

struct clkctl_acpu_speed acpu_freq_tbl[] = {
	{  19200, CCTL(CLK_TCXO, 1),		SRC_RAW, 0, 0, 1050 },
	{ 128000, CCTL(CLK_TCXO, 1),		SRC_AXI, 0, 0, 1050 },
	{ 245000, CCTL(CLK_MODEM_PLL, 1),	SRC_RAW, 0, 0, 1050 },
	{ 256000, CCTL(CLK_GLOBAL_PLL, 3),	SRC_RAW, 0, 0, 1050 },
	{ 384000, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x0A, 0, 1050 },
	{ 422400, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x0B, 0, 1050 },
	{ 460800, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x0C, 0, 1050 },
	{ 499200, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x0D, 0, 1075 },
	{ 537600, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x0E, 0, 1100 },
	{ 576000, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x0F, 0, 1100 },
	{ 614400, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x10, 0, 1125 },
	{ 652800, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x11, 0, 1150 },
	{ 691200, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x12, 0, 1175 },
	{ 729600, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x13, 0, 1200 },
	{ 768000, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x14, 0, 1200 },
	{ 806400, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x15, 0, 1225 },
	{ 844800, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x16, 0, 1250 },
	{ 883200, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x17, 0, 1275 },
	{ 921600, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x18, 0, 1275 },
	{ 960000, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x19, 0, 1275 },
	{ 998400, CCTL(CLK_TCXO, 1),		SRC_SCPLL, 0x1A, 0, 1275 },
	{ 0 },
};

/* select the standby clock that is used when switching scpll
 * frequencies
 *
 * Currently: MPLL
 */
struct clkctl_acpu_speed *acpu_stby = &acpu_freq_tbl[2];
#define IS_ACPU_STANDBY(x)	(((x)->clk_cfg == acpu_stby->clk_cfg) && \
				 ((x)->clk_sel == acpu_stby->clk_sel))

struct clkctl_acpu_speed *acpu_mpll = &acpu_freq_tbl[2];

#ifdef CONFIG_CPU_FREQ_TABLE
static struct cpufreq_frequency_table freq_table[ARRAY_SIZE(acpu_freq_tbl)];

static void __init acpuclk_init_cpufreq_table(void)
{
	int i;
	int vdd;
	for (i = 0; acpu_freq_tbl[i].acpu_khz; i++) {
		freq_table[i].index = i;
		freq_table[i].frequency = CPUFREQ_ENTRY_INVALID;

		/* Skip speeds using the global pll */
		if (acpu_freq_tbl[i].acpu_khz == 256000 ||
				acpu_freq_tbl[i].acpu_khz == 19200)
			continue;

		vdd = acpu_freq_tbl[i].vdd;
		/* Allow mpll and the first scpll speeds */
		if (acpu_freq_tbl[i].acpu_khz == acpu_mpll->acpu_khz ||
				acpu_freq_tbl[i].acpu_khz == 384000) {
			freq_table[i].frequency = acpu_freq_tbl[i].acpu_khz;
			continue;
		}

		/* Take the fastest speed available at the specified VDD level */
		if (vdd != acpu_freq_tbl[i + 1].vdd)
			freq_table[i].frequency = acpu_freq_tbl[i].acpu_khz;
	}

	freq_table[i].index = i;
	freq_table[i].frequency = CPUFREQ_TABLE_END;

	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());
}
#else
#define acpuclk_init_cpufreq_table() do {} while (0);
#endif

struct clock_state {
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
	unsigned long			power_collapse_khz;
	unsigned long			wait_for_irq_khz;
	struct regulator                *regulator;
};

static struct clock_state drv_state = { 0 };

static DEFINE_SPINLOCK(acpu_lock);

#define PLLMODE_POWERDOWN	0
#define PLLMODE_BYPASS		1
#define PLLMODE_STANDBY		2
#define PLLMODE_FULL_CAL	4
#define PLLMODE_HALF_CAL	5
#define PLLMODE_STEP_CAL	6
#define PLLMODE_NORMAL		7
#define PLLMODE_MASK		7

static void scpll_power_down(void)
{
	uint32_t val;

	/* Wait for any frequency switches to finish. */
	while (readl(SCPLL_STATUS_ADDR) & 0x1)
		;

	/* put the pll in standby mode */
	val = readl(SCPLL_CTL_ADDR);
	val = (val & (~PLLMODE_MASK)) | PLLMODE_STANDBY;
	writel(val, SCPLL_CTL_ADDR);
	dmb();

	/* wait to stabilize in standby mode */
	udelay(10);

	val = (val & (~PLLMODE_MASK)) | PLLMODE_POWERDOWN;
	writel(val, SCPLL_CTL_ADDR);
	dmb();
}

static void scpll_set_freq(uint32_t lval)
{
	uint32_t val, ctl;

	if (lval > 33)
		lval = 33;
	if (lval < 10)
		lval = 10;

	/* wait for any calibrations or frequency switches to finish */
	while (readl(SCPLL_STATUS_ADDR) & 0x3)
		;

	ctl = readl(SCPLL_CTL_ADDR);

	if ((ctl & PLLMODE_MASK) != PLLMODE_NORMAL) {
		/* put the pll in standby mode */
		writel((ctl & (~PLLMODE_MASK)) | PLLMODE_STANDBY, SCPLL_CTL_ADDR);
		dmb();

		/* wait to stabilize in standby mode */
		udelay(10);

		/* switch to 384 MHz */
		val = readl(SCPLL_FSM_CTL_EXT_ADDR);
		val = (val & (~0x1FF)) | (0x0A << 3) | SHOT_SWITCH;
		writel(val, SCPLL_FSM_CTL_EXT_ADDR);
		dmb();

		ctl = readl(SCPLL_CTL_ADDR);
		writel(ctl | PLLMODE_NORMAL, SCPLL_CTL_ADDR);
		dmb();

		/* wait for frequency switch to finish */
		while (readl(SCPLL_STATUS_ADDR) & 0x1)
			;

		/* completion bit is not reliable for SHOT switch */
		udelay(25);
	}

	/* write the new L val and switch mode */
	val = readl(SCPLL_FSM_CTL_EXT_ADDR);
	val = (val & (~0x1FF)) | (lval << 3) | HOP_SWITCH;
	writel(val, SCPLL_FSM_CTL_EXT_ADDR);
	dmb();

	ctl = readl(SCPLL_CTL_ADDR);
	writel(ctl | PLLMODE_NORMAL, SCPLL_CTL_ADDR);
	dmb();

	/* wait for frequency switch to finish */
	while (readl(SCPLL_STATUS_ADDR) & 0x1)
		;
}

/* this is still a bit weird... */
static void select_clock(unsigned src, unsigned config)
{
	uint32_t val;

	if (src == SRC_RAW) {
		uint32_t sel = readl(SPSS_CLK_SEL_ADDR);
		unsigned shift = (sel & 1) ? 8 : 0;

		/* set other clock source to the new configuration */
		val = readl(SPSS_CLK_CNTL_ADDR);
		val = (val & (~(0x7F << shift))) | (config << shift);
		writel(val, SPSS_CLK_CNTL_ADDR);

		/* switch to other clock source */
		writel(sel ^ 1, SPSS_CLK_SEL_ADDR);

		dmb(); /* necessary? */
	}

	/* switch to new source */
	val = readl(SPSS_CLK_SEL_ADDR) & (~6);
	writel(val | ((src & 3) << 1), SPSS_CLK_SEL_ADDR);
}

static int acpuclk_set_vdd_level(int vdd)
{
	if (!drv_state.regulator || IS_ERR(drv_state.regulator)) {
		drv_state.regulator = regulator_get(NULL, "acpu_vcore");
		if (IS_ERR(drv_state.regulator)) {
			pr_info("acpuclk_set_vdd_level %d no regulator\n", vdd);
			/* Assume that the PMIC supports scaling the processor
			 * to its maximum frequency at its default voltage.
			 */
			return 0;
		}
		pr_info("acpuclk_set_vdd_level got regulator\n");
	}
	vdd *= 1000; /* mV -> uV */
	return regulator_set_voltage(drv_state.regulator, vdd, vdd);
}

int acpuclk_set_rate(unsigned long rate, int for_power_collapse)
{
	struct clkctl_acpu_speed *cur, *next;
	unsigned long flags;

	cur = drv_state.current_speed;

	/* convert to KHz */
	rate /= 1000;

	DEBUG("acpuclk_set_rate(%d,%d)\n", (int) rate, for_power_collapse);

	if (rate == 0 || rate == cur->acpu_khz)
		return 0;

	next = acpu_freq_tbl;
	for (;;) {
		if (next->acpu_khz == rate)
			break;
		if (next->acpu_khz == 0)
			return -EINVAL;
		next++;
	}

	if (!for_power_collapse) {
		mutex_lock(&drv_state.lock);
		/* Increase VDD if needed. */
		if (next->vdd > cur->vdd) {
			if (acpuclk_set_vdd_level(next->vdd)) {
				pr_err("acpuclock: Unable to increase ACPU VDD.\n");
				mutex_unlock(&drv_state.lock);
				return -EINVAL;
			}
		}
	}

	spin_lock_irqsave(&acpu_lock, flags);

	DEBUG("sel=%d cfg=%02x lv=%02x -> sel=%d, cfg=%02x lv=%02x\n",
	      cur->clk_sel, cur->clk_cfg, cur->sc_l_value,
	      next->clk_sel, next->clk_cfg, next->sc_l_value);

	if (next->clk_sel == SRC_SCPLL) {
		if (!IS_ACPU_STANDBY(cur))
			select_clock(acpu_stby->clk_sel, acpu_stby->clk_cfg);
		loops_per_jiffy = next->lpj;
		scpll_set_freq(next->sc_l_value);
		select_clock(SRC_SCPLL, 0);
	} else {
		loops_per_jiffy = next->lpj;
		if (cur->clk_sel == SRC_SCPLL) {
			select_clock(acpu_stby->clk_sel, acpu_stby->clk_cfg);
			select_clock(next->clk_sel, next->clk_cfg);
			scpll_power_down();
		} else {
			select_clock(next->clk_sel, next->clk_cfg);
		}
	}

	drv_state.current_speed = next;

	spin_unlock_irqrestore(&acpu_lock, flags);
	if (!for_power_collapse) {
		/* Drop VDD level if we can. */
		if (next->vdd < cur->vdd) {
			if (acpuclk_set_vdd_level(next->vdd))
				pr_err("acpuclock: Unable to drop ACPU VDD.\n");
		}
		mutex_unlock(&drv_state.lock);
	}

	return 0;
}

static unsigned __init acpuclk_find_speed(void)
{
	uint32_t sel, val;

	sel = readl(SPSS_CLK_SEL_ADDR);
	switch ((sel & 6) >> 1) {
	case 1:
		val = readl(SCPLL_FSM_CTL_EXT_ADDR);
		val = (val >> 3) & 0x3f;
		return val * 38400;
	case 2:
		return 128000;
	default:
		pr_err("acpu_find_speed: failed\n");
		BUG();
		return 0;
	}
}

#define PCOM_MODEM_PLL	0
static int pll_request(unsigned id, unsigned on)
{
	on = !!on;
	return msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
}

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *speed;
	unsigned init_khz;

	init_khz = acpuclk_find_speed();

	/* request the modem pll, and then drop it. We don't want to keep a
	 * ref to it, but we do want to make sure that it is initialized at
	 * this point. The ARM9 will ensure that the MPLL is always on
	 * once it is fully booted, but it may not be up by the time we get
	 * to here. So, our pll_request for it will block until the mpll is
	 * actually up. We want it up because we will want to use it as a
	 * temporary step during frequency scaling. */
	pll_request(PCOM_MODEM_PLL, 1);
	pll_request(PCOM_MODEM_PLL, 0);

	if (!(readl(MSM_CLK_CTL_BASE + 0x300) & 1)) {
		pr_err("%s: MPLL IS NOT ON!!! RUN AWAY!!\n", __func__);
		BUG();
	}

	/* Move to 768MHz for boot, which is a safe frequency
	 * for all versions of Scorpion at the moment.
	 */
	speed = acpu_freq_tbl;
	for (;;) {
		if (speed->acpu_khz == 768000)
			break;
		if (speed->acpu_khz == 0) {
			pr_err("acpuclk_init: cannot find 768MHz\n");
			BUG();
		}
		speed++;
	}

	if (init_khz != speed->acpu_khz) {
		/* Bootloader needs to have SCPLL operating, but we're
		 * going to step over to the standby clock and make sure
		 * we select the right frequency on SCPLL and then
		 * step back to it, to make sure we're sane here.
		 */
		select_clock(acpu_stby->clk_sel, acpu_stby->clk_cfg);
		scpll_power_down();
		scpll_set_freq(speed->sc_l_value);
		select_clock(SRC_SCPLL, 0);
	}
	drv_state.current_speed = speed;

	for (speed = acpu_freq_tbl; speed->acpu_khz; speed++)
		speed->lpj = cpufreq_scale(loops_per_jiffy,
					   init_khz, speed->acpu_khz);

	loops_per_jiffy = drv_state.current_speed->lpj;
}

unsigned long acpuclk_get_rate(void)
{
	return drv_state.current_speed->acpu_khz;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

unsigned long acpuclk_power_collapse(void)
{
	int ret = acpuclk_get_rate();
	if (ret > drv_state.power_collapse_khz)
		acpuclk_set_rate(drv_state.power_collapse_khz * 1000, 1);
	return ret * 1000;
}

unsigned long acpuclk_get_wfi_rate(void)
{
	return drv_state.wait_for_irq_khz * 1000;
}

unsigned long acpuclk_wait_for_irq(void)
{
	int ret = acpuclk_get_rate();
	if (ret > drv_state.wait_for_irq_khz)
		acpuclk_set_rate(drv_state.wait_for_irq_khz * 1000, 1);
	return ret * 1000;
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	spin_lock_init(&acpu_lock);
	mutex_init(&drv_state.lock);

	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	drv_state.max_speed_delta_khz = clkdata->max_speed_delta_khz;
	drv_state.vdd_switch_time_us = clkdata->vdd_switch_time_us;
	drv_state.power_collapse_khz = clkdata->power_collapse_khz;
	drv_state.wait_for_irq_khz = clkdata->wait_for_irq_khz;

	if (clkdata->mpll_khz)
		acpu_mpll->acpu_khz = clkdata->mpll_khz;

	acpuclk_init();
	acpuclk_init_cpufreq_table();
}
