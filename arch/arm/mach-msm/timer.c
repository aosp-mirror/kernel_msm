/* linux/arch/arm/mach-msm/timer.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <asm/mach/time.h>
#include <mach/msm_iomap.h>

#include "smd_private.h"

enum {
	MSM_TIMER_DEBUG_SYNC_STATE = 1U << 0,
	MSM_TIMER_DEBUG_SYNC_UPDATE = 1U << 1,
	MSM_TIMER_DEBUG_SYNC = 1U << 2,
};
static int msm_timer_debug_mask;
module_param_named(debug_mask, msm_timer_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define TIMER_MATCH_VAL         0x0000
#define TIMER_COUNT_VAL         0x0004
#define TIMER_ENABLE            0x0008
#define TIMER_ENABLE_CLR_ON_MATCH_EN    2
#define TIMER_ENABLE_EN                 1
#define TIMER_CLEAR             0x000C

#define CSR_PROTECTION          0x0020
#define CSR_PROTECTION_EN               1

#define GPT_HZ 32768

#if defined(CONFIG_ARCH_QSD8X50)
#define DGT_HZ (19200000 / 4) /* 19.2 MHz / 4 by default */
#define MSM_DGT_SHIFT (0)
#elif defined(CONFIG_ARCH_MSM7X30)
#define DGT_HZ (24576000 / 4) /* 24.576 MHz (LPXO) / 4 by default */
#define MSM_DGT_SHIFT (0)
#else
#define DGT_HZ 19200000 /* 19.2 MHz or 600 KHz after shift */
#define MSM_DGT_SHIFT (5)
#endif

enum {
	MSM_CLOCK_FLAGS_UNSTABLE_COUNT = 1U << 0,
	MSM_CLOCK_FLAGS_ODD_MATCH_WRITE = 1U << 1,
	MSM_CLOCK_FLAGS_DELAYED_WRITE_POST = 1U << 2,
};

struct msm_clock {
	struct clock_event_device   clockevent;
	struct clocksource          clocksource;
	struct irqaction            irq;
	void __iomem                *regbase;
	uint32_t                    freq;
	uint32_t                    shift;
	uint32_t                    flags;
	uint32_t                    write_delay;
	uint32_t                    last_set;
	uint32_t                    offset;
	uint32_t                    alarm_vtime;
	uint32_t                    smem_offset;
	uint32_t                    smem_in_sync;
	cycle_t                     stopped_tick;
	int                         stopped;
};
enum {
	MSM_CLOCK_GPT,
	MSM_CLOCK_DGT,
};
static struct msm_clock msm_clocks[];
static struct msm_clock *msm_active_clock;
static DEFINE_SPINLOCK(msm_fast_timer_lock);
static int msm_fast_timer_enabled;

static irqreturn_t msm_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	if (evt->event_handler)
		evt->event_handler(evt);
	return IRQ_HANDLED;
}

static uint32_t msm_read_timer_count(struct msm_clock *clock)
{
	uint32_t t1, t2;
	int loop_count = 0;

	t1 = readl(clock->regbase + TIMER_COUNT_VAL);
	if (!(clock->flags & MSM_CLOCK_FLAGS_UNSTABLE_COUNT))
		return t1;
	while (1) {
		t2 = readl(clock->regbase + TIMER_COUNT_VAL);
		if (t1 == t2)
			return t1;
		if (loop_count++ > 10) {
			printk(KERN_ERR "msm_read_timer_count timer %s did not"
			       "stabilize %u != %u\n", clock->clockevent.name,
			       t2, t1);
			return t2;
		}
		t1 = t2;
	}
}

static cycle_t msm_gpt_read(struct clocksource *cs)
{
	struct msm_clock *clock = &msm_clocks[MSM_CLOCK_GPT];
	if (clock->stopped)
		return clock->stopped_tick;
	else
		return msm_read_timer_count(clock) + clock->offset;
}

static cycle_t msm_dgt_read(struct clocksource *cs)
{
	struct msm_clock *clock = &msm_clocks[MSM_CLOCK_DGT];
	if (clock->stopped)
		return clock->stopped_tick;
	return (msm_read_timer_count(clock) + clock->offset) >> MSM_DGT_SHIFT;
}

static int msm_timer_set_next_event(unsigned long cycles,
				    struct clock_event_device *evt)
{
	int i;
	struct msm_clock *clock;
	uint32_t now;
	uint32_t alarm;
	int late;

	clock = container_of(evt, struct msm_clock, clockevent);
	now = msm_read_timer_count(clock);
	alarm = now + (cycles << clock->shift);
	if (clock->flags & MSM_CLOCK_FLAGS_ODD_MATCH_WRITE)
		while (now == clock->last_set)
			now = msm_read_timer_count(clock);
	writel(alarm, clock->regbase + TIMER_MATCH_VAL);
	if (clock->flags & MSM_CLOCK_FLAGS_DELAYED_WRITE_POST) {
		/* read the counter four extra times to make sure write posts
		   before reading the time */
		for (i = 0; i < 4; i++)
			readl(clock->regbase + TIMER_COUNT_VAL);
	}
	now = msm_read_timer_count(clock);
	clock->last_set = now;
	clock->alarm_vtime = alarm + clock->offset;
	late = now - alarm;
	if (late >= (int)(-clock->write_delay << clock->shift) && late < DGT_HZ*5) {
		static int print_limit = 10;
		if (print_limit > 0) {
			print_limit--;
			printk(KERN_NOTICE "msm_timer_set_next_event(%lu) "
			       "clock %s, alarm already expired, now %x, "
			       "alarm %x, late %d%s\n",
			       cycles, clock->clockevent.name, now, alarm, late,
			       print_limit ? "" : " stop printing");
		}
		return -ETIME;
	}
	return 0;
}

static void msm_timer_set_mode(enum clock_event_mode mode,
			      struct clock_event_device *evt)
{
	struct msm_clock *clock;
	unsigned long irq_flags;

	clock = container_of(evt, struct msm_clock, clockevent);
	local_irq_save(irq_flags);

	switch (mode) {
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		clock->stopped = 0;
		clock->offset = -msm_read_timer_count(clock) + clock->stopped_tick;
		msm_active_clock = clock;
		writel(TIMER_ENABLE_EN, clock->regbase + TIMER_ENABLE);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		msm_active_clock = NULL;
		clock->smem_in_sync = 0;
		clock->stopped = 1;
		clock->stopped_tick = (msm_read_timer_count(clock) +
					clock->offset) >> clock->shift;
		writel(0, clock->regbase + TIMER_ENABLE);
		break;
	}
	local_irq_restore(irq_flags);
}

static inline int check_timeout(struct msm_clock *clock, uint32_t timeout)
{
	return (int32_t)(msm_read_timer_count(clock) - timeout) <= 0;
}

#ifndef CONFIG_ARCH_MSM_SCORPION

static uint32_t msm_timer_sync_smem_clock(int exit_sleep)
{
	struct msm_clock *clock = &msm_clocks[MSM_CLOCK_GPT];
	uint32_t *smem_clock;
	uint32_t smem_clock_val;
	uint32_t timeout;
	uint32_t entry_time;
	uint32_t timeout_delta;
	uint32_t last_state;
	uint32_t state;
	uint32_t new_offset;

	smem_clock = smem_alloc(SMEM_SMEM_SLOW_CLOCK_VALUE,
				sizeof(uint32_t));

	if (smem_clock == NULL) {
		printk(KERN_ERR "no smem clock\n");
		return 0;
	}

	if (!exit_sleep && clock->smem_in_sync)
		return 0;

	timeout_delta = (clock->freq >> (7 - clock->shift)); /* 7.8ms */

	last_state = state = smsm_get_state(SMSM_STATE_MODEM);
	if (*smem_clock) {
		printk(KERN_INFO "get_smem_clock: invalid start state %x "
		       "clock %u\n", state, *smem_clock);
		smsm_change_state(SMSM_STATE_APPS, SMSM_TIMEWAIT, SMSM_TIMEINIT);
		entry_time = msm_read_timer_count(clock);
		timeout = entry_time + timeout_delta;
		while (*smem_clock != 0 && check_timeout(clock, timeout))
			;
		if (*smem_clock) {
			printk(KERN_INFO "get_smem_clock: timeout still "
			       "invalid state %x clock %u in %d ticks\n",
			       state, *smem_clock,
			       msm_read_timer_count(clock) - entry_time);
			return 0;
		}
	}
	entry_time = msm_read_timer_count(clock);
	timeout = entry_time + timeout_delta;
	smsm_change_state(SMSM_STATE_APPS, SMSM_TIMEINIT, SMSM_TIMEWAIT);
	do {
		smem_clock_val = *smem_clock;
		state = smsm_get_state(SMSM_STATE_MODEM);
		if (state != last_state) {
			last_state = state;
			if (msm_timer_debug_mask & MSM_TIMER_DEBUG_SYNC_STATE)
				pr_info("get_smem_clock: state %x clock %u\n",
					state, smem_clock_val);
		}
	} while (smem_clock_val == 0 && check_timeout(clock, timeout));
	if (smem_clock_val) {
		new_offset = smem_clock_val - msm_read_timer_count(clock);
		if (clock->offset + clock->smem_offset != new_offset) {
			if (exit_sleep)
				clock->offset = new_offset - clock->smem_offset;
			else
				clock->smem_offset = new_offset - clock->offset;
			clock->smem_in_sync = 1;
			if (msm_timer_debug_mask & MSM_TIMER_DEBUG_SYNC_UPDATE)
				printk(KERN_INFO "get_smem_clock: state %x "
				       "clock %u new offset %u+%u\n",
				       state, smem_clock_val,
				       clock->offset, clock->smem_offset);
		} else if (msm_timer_debug_mask & MSM_TIMER_DEBUG_SYNC) {
			printk(KERN_INFO "get_smem_clock: state %x "
			       "clock %u offset %u+%u\n",
			       state, smem_clock_val,
			       clock->offset, clock->smem_offset);
		}
	} else {
		printk(KERN_INFO "get_smem_clock: timeout state %x clock %u "
		       "in %d ticks\n", state, *smem_clock,
		       msm_read_timer_count(clock) - entry_time);
	}
	smsm_change_state(SMSM_STATE_APPS, SMSM_TIMEWAIT, SMSM_TIMEINIT);
	entry_time = msm_read_timer_count(clock);
	timeout = entry_time + timeout_delta;
	while (*smem_clock != 0 && check_timeout(clock, timeout)) {
		uint32_t astate = smsm_get_state(SMSM_STATE_APPS);
		if ((astate & SMSM_TIMEWAIT) || !(astate & SMSM_TIMEINIT)) {
			if (msm_timer_debug_mask & MSM_TIMER_DEBUG_SYNC_STATE)
				pr_info("get_smem_clock: modem overwrote "
					"apps state %x\n", astate);
			smsm_change_state(SMSM_STATE_APPS,
					  SMSM_TIMEWAIT, SMSM_TIMEINIT);
		}
	}
	if (*smem_clock)
		printk(KERN_INFO "get_smem_clock: exit timeout state %x "
		       "clock %u in %d ticks\n", state, *smem_clock,
		       msm_read_timer_count(clock) - entry_time);
	return smem_clock_val;
}

#else

/* Time Master State Bits */
#define DEM_TIME_MASTER_TIME_PENDING_APPS	BIT(0)

/* Time Slave State Bits */
#define DEM_TIME_SLAVE_TIME_REQUEST         0x0400
#define DEM_TIME_SLAVE_TIME_POLL            0x0800
#define DEM_TIME_SLAVE_TIME_INIT            0x1000

static uint32_t msm_timer_sync_smem_clock(int exit_sleep)
{
	struct msm_clock *clock = &msm_clocks[MSM_CLOCK_GPT];
	uint32_t *smem_clock;
	uint32_t smem_clock_val;
	uint32_t bad_clock = 0;
	uint32_t timeout;
	uint32_t entry_time;
	uint32_t timeout_delta;
	uint32_t last_state;
	uint32_t state;
	uint32_t new_offset;

	smem_clock = smem_alloc(SMEM_SMEM_SLOW_CLOCK_VALUE,
				sizeof(uint32_t));

	if (smem_clock == NULL) {
		printk(KERN_ERR "no smem clock\n");
		return 0;
	}

	if (!exit_sleep && clock->smem_in_sync)
		return 0;

	timeout_delta = (clock->freq >> (7 - clock->shift)); /* 7.8ms */

	entry_time = msm_read_timer_count(clock);
	last_state = state = smsm_get_state(SMSM_STATE_TIME_MASTER_DEM);
	timeout = entry_time + timeout_delta;
	while ((smsm_get_state(SMSM_STATE_TIME_MASTER_DEM)
		& DEM_TIME_MASTER_TIME_PENDING_APPS)
		&& check_timeout(clock, timeout))
		;
	if ((smsm_get_state(SMSM_STATE_TIME_MASTER_DEM) &
				DEM_TIME_MASTER_TIME_PENDING_APPS)) {
		printk(KERN_INFO "get_smem_clock: invalid start state %x "
		       "clock %u in %d ticks\n",
		       state, *smem_clock,
		       msm_read_timer_count(clock) - entry_time);
		bad_clock = *smem_clock;
	}
	entry_time = msm_read_timer_count(clock);
	timeout = entry_time + timeout_delta;
	smsm_change_state(SMSM_STATE_APPS_DEM,
			DEM_TIME_SLAVE_TIME_INIT, DEM_TIME_SLAVE_TIME_REQUEST);
	while (!(smsm_get_state(SMSM_STATE_TIME_MASTER_DEM)
		& DEM_TIME_MASTER_TIME_PENDING_APPS)
		&& check_timeout(clock, timeout))
		;
	if (!(smsm_get_state(SMSM_STATE_TIME_MASTER_DEM) &
					DEM_TIME_MASTER_TIME_PENDING_APPS)) {
		printk(KERN_INFO "get_smem_clock: invalid start state %x "
		       "clock %u in %d ticks\n",
		       state, *smem_clock,
		       msm_read_timer_count(clock) - entry_time);
		bad_clock = *smem_clock;
	}
	smsm_change_state(SMSM_STATE_APPS_DEM,
		DEM_TIME_SLAVE_TIME_REQUEST, DEM_TIME_SLAVE_TIME_POLL);
	do {
		smem_clock_val = *smem_clock;
		state = smsm_get_state(SMSM_STATE_TIME_MASTER_DEM);
		if (state != last_state) {
			last_state = state;
			if (msm_timer_debug_mask & MSM_TIMER_DEBUG_SYNC_STATE)
				pr_info("get_smem_clock: state %x clock %u\n",
					state, smem_clock_val);
		}
	} while ((!smem_clock_val || smem_clock_val == bad_clock)
		 && check_timeout(clock, timeout));
	if (smem_clock_val && smem_clock_val != bad_clock) {
		new_offset = smem_clock_val - msm_read_timer_count(clock);
		if (clock->offset + clock->smem_offset != new_offset) {
			if (exit_sleep)
				clock->offset = new_offset - clock->smem_offset;
			else
				clock->smem_offset = new_offset - clock->offset;
			clock->smem_in_sync = 1;
			if (msm_timer_debug_mask & MSM_TIMER_DEBUG_SYNC_UPDATE)
				printk(KERN_INFO "get_smem_clock: state %x "
				       "clock %u new offset %u+%u\n",
				       state, smem_clock_val,
				       clock->offset, clock->smem_offset);
		} else if (msm_timer_debug_mask & MSM_TIMER_DEBUG_SYNC) {
			printk(KERN_INFO "get_smem_clock: state %x "
			       "clock %u offset %u+%u\n",
			       state, smem_clock_val,
			       clock->offset, clock->smem_offset);
		}
	} else {
		printk(KERN_INFO "get_smem_clock: timeout state %x clock %u "
		       "in %d ticks\n", state, *smem_clock,
		       msm_read_timer_count(clock) - entry_time);
	}
	smsm_change_state(SMSM_STATE_APPS_DEM,
		DEM_TIME_SLAVE_TIME_POLL, DEM_TIME_SLAVE_TIME_INIT);
#if 1 /* debug */
	entry_time = msm_read_timer_count(clock);
	timeout = entry_time + timeout_delta;
	while ((smsm_get_state(SMSM_STATE_TIME_MASTER_DEM)
		& DEM_TIME_MASTER_TIME_PENDING_APPS)
		&& check_timeout(clock, timeout))
		;
	if (smsm_get_state(SMSM_STATE_TIME_MASTER_DEM) &
					DEM_TIME_MASTER_TIME_PENDING_APPS)
		printk(KERN_INFO "get_smem_clock: exit timeout state %x "
		       "clock %u in %d ticks\n", state, *smem_clock,
		       msm_read_timer_count(clock) - entry_time);
#endif
	return smem_clock_val;
}

#endif

static void msm_timer_reactivate_alarm(struct msm_clock *clock)
{
	long alarm_delta = clock->alarm_vtime - clock->offset -
		msm_read_timer_count(clock);
	if (alarm_delta < (long)clock->write_delay + 4)
		alarm_delta = clock->write_delay + 4;
	while (msm_timer_set_next_event(alarm_delta, &clock->clockevent))
		;
}

int64_t msm_timer_enter_idle(void)
{
	struct msm_clock *clock = msm_active_clock;
	uint32_t alarm;
	uint32_t count;
	int32_t delta;

	if (clock != &msm_clocks[MSM_CLOCK_GPT] || msm_fast_timer_enabled)
		return 0;

	msm_timer_sync_smem_clock(0);

	count = msm_read_timer_count(clock);
	if (clock->stopped++ == 0)
		clock->stopped_tick = (count + clock->offset) >> clock->shift;
	alarm = clock->alarm_vtime - clock->offset;
	delta = alarm - count;
	if (delta <= -(int32_t)((clock->freq << clock->shift) >> 10)) {
		/* timer should have triggered 1ms ago */
		printk(KERN_ERR "msm_timer_enter_idle: timer late %d, "
			"reprogram it\n", delta);
		msm_timer_reactivate_alarm(clock);
	}
	if (delta <= 0)
		return 0;
	return clocksource_cyc2ns((alarm - count) >> clock->shift,
			clock->clocksource.mult, clock->clocksource.shift);
}

void msm_timer_exit_idle(int low_power)
{
	struct msm_clock *clock = msm_active_clock;
	uint32_t smem_clock;

	if (clock != &msm_clocks[MSM_CLOCK_GPT])
		return;

	if (low_power) {
		if (!(readl(clock->regbase + TIMER_ENABLE) & TIMER_ENABLE_EN)) {
			writel(TIMER_ENABLE_EN, clock->regbase + TIMER_ENABLE);
			smem_clock = msm_timer_sync_smem_clock(1);
		}
		msm_timer_reactivate_alarm(clock);
	}
	clock->stopped--;
}

unsigned long long sched_clock(void)
{
	static cycle_t saved_ticks;
	static int saved_ticks_valid;
	static unsigned long long base;
	static unsigned long long last_result;

	unsigned long irq_flags;
	static cycle_t last_ticks;
	cycle_t ticks;
	static unsigned long long result;
	struct clocksource *cs;
	struct msm_clock *clock = msm_active_clock;

	local_irq_save(irq_flags);
	if (clock) {
		cs = &clock->clocksource;

		last_ticks = saved_ticks;
		saved_ticks = ticks = cs->read(cs);
		if (!saved_ticks_valid) {
			saved_ticks_valid = 1;
			last_ticks = ticks;
			base -= clocksource_cyc2ns(ticks, cs->mult, cs->shift);
		}
		if (ticks < last_ticks) {
			base += clocksource_cyc2ns(cs->mask,
						   cs->mult, cs->shift);
			base += clocksource_cyc2ns(1, cs->mult, cs->shift);
		}
		last_result = result =
			clocksource_cyc2ns(ticks, cs->mult, cs->shift) + base;
	} else {
		base = result = last_result;
		saved_ticks_valid = 0;
	}
	local_irq_restore(irq_flags);
	return result; 
}

#ifdef CONFIG_MSM7X00A_USE_GP_TIMER
	#define DG_TIMER_RATING 100
#else
	#define DG_TIMER_RATING 300
#endif

static struct msm_clock msm_clocks[] = {
	[MSM_CLOCK_GPT] = {
		.clockevent = {
			.name           = "gp_timer",
			.features       = CLOCK_EVT_FEAT_ONESHOT,
			.shift          = 32,
			.rating         = 200,
			.set_next_event = msm_timer_set_next_event,
			.set_mode       = msm_timer_set_mode,
		},
		.clocksource = {
			.name           = "gp_timer",
			.rating         = 200,
			.read           = msm_gpt_read,
			.mask           = CLOCKSOURCE_MASK(32),
			.shift          = 17,
			.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
		},
		.irq = {
			.name    = "gp_timer",
			.flags   = IRQF_DISABLED | IRQF_TIMER |
				   IRQF_TRIGGER_RISING,
			.handler = msm_timer_interrupt,
			.dev_id  = &msm_clocks[0].clockevent,
			.irq     = INT_GP_TIMER_EXP
		},
		.regbase = MSM_GPT_BASE,
		.freq = GPT_HZ,
		.flags   =
			MSM_CLOCK_FLAGS_UNSTABLE_COUNT |
			MSM_CLOCK_FLAGS_ODD_MATCH_WRITE |
			MSM_CLOCK_FLAGS_DELAYED_WRITE_POST,
		.write_delay = 9,
	},
	[MSM_CLOCK_DGT] = {
		.clockevent = {
			.name           = "dg_timer",
			.features       = CLOCK_EVT_FEAT_ONESHOT,
			.shift          = 32 + MSM_DGT_SHIFT,
			.rating         = DG_TIMER_RATING,
			.set_next_event = msm_timer_set_next_event,
			.set_mode       = msm_timer_set_mode,
		},
		.clocksource = {
			.name           = "dg_timer",
			.rating         = DG_TIMER_RATING,
			.read           = msm_dgt_read,
			.mask           = CLOCKSOURCE_MASK((32-MSM_DGT_SHIFT)),
			.shift          = 24 - MSM_DGT_SHIFT,
			.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
		},
		.irq = {
			.name    = "dg_timer",
			.flags   = IRQF_DISABLED | IRQF_TIMER |
				   IRQF_TRIGGER_RISING,
			.handler = msm_timer_interrupt,
			.dev_id  = &msm_clocks[1].clockevent,
			.irq     = INT_DEBUG_TIMER_EXP
		},
		.regbase = MSM_DGT_BASE,
		.freq = DGT_HZ >> MSM_DGT_SHIFT,
		.shift = MSM_DGT_SHIFT,
		.write_delay = 2,
	}
};

/**
 * msm_enable_fast_timer - Enable fast timer
 *
 * Prevents low power idle, but the caller must call msm_disable_fast_timer
 * before suspend completes.
 * Reference counted.
 */
void msm_enable_fast_timer(void)
{
	u32 max;
	unsigned long irq_flags;
	struct msm_clock *clock = &msm_clocks[MSM_CLOCK_DGT];

	spin_lock_irqsave(&msm_fast_timer_lock, irq_flags);
	if (msm_fast_timer_enabled++)
		goto done;
	if (msm_active_clock == &msm_clocks[MSM_CLOCK_DGT]) {
		pr_warning("msm_enable_fast_timer: timer already in use, "
			"returned time will jump when hardware timer wraps\n");
		goto done;
	}
	max = (clock->clockevent.mult >> (clock->clockevent.shift - 32)) - 1;
	writel(max, clock->regbase + TIMER_MATCH_VAL);
	writel(TIMER_ENABLE_EN | TIMER_ENABLE_CLR_ON_MATCH_EN,
		clock->regbase + TIMER_ENABLE);
done:
	spin_unlock_irqrestore(&msm_fast_timer_lock, irq_flags);
}

/**
 * msm_enable_fast_timer - Disable fast timer
 */
void msm_disable_fast_timer(void)
{
	unsigned long irq_flags;
	struct msm_clock *clock = &msm_clocks[MSM_CLOCK_DGT];

	spin_lock_irqsave(&msm_fast_timer_lock, irq_flags);
	if (!WARN(!msm_fast_timer_enabled, "msm_disable_fast_timer undeflow")
	    && !--msm_fast_timer_enabled
	    && msm_active_clock != &msm_clocks[MSM_CLOCK_DGT])
		writel(0, clock->regbase + TIMER_ENABLE);
	spin_unlock_irqrestore(&msm_fast_timer_lock, irq_flags);
}

/**
 * msm_enable_fast_timer - Read fast timer
 *
 * Returns 32bit nanosecond time value.
 */
u32 msm_read_fast_timer(void)
{
	cycle_t ticks;
	struct msm_clock *clock = &msm_clocks[MSM_CLOCK_DGT];
	ticks = msm_read_timer_count(clock) >> MSM_DGT_SHIFT;
	return clocksource_cyc2ns(ticks, clock->clocksource.mult,
					clock->clocksource.shift);
}

static void __init msm_timer_init(void)
{
	int i;
	int res;

	for (i = 0; i < ARRAY_SIZE(msm_clocks); i++) {
		struct msm_clock *clock = &msm_clocks[i];
		struct clock_event_device *ce = &clock->clockevent;
		struct clocksource *cs = &clock->clocksource;
		writel(0, clock->regbase + TIMER_ENABLE);
		writel(0, clock->regbase + TIMER_CLEAR);
		writel(~0, clock->regbase + TIMER_MATCH_VAL);
		while (msm_read_timer_count(clock)) ; /* wait for clock to clear */

		ce->mult = div_sc(clock->freq, NSEC_PER_SEC, ce->shift);
		/* allow at least 10 seconds to notice that the timer wrapped */
		ce->max_delta_ns =
			clockevent_delta2ns(0xf0000000 >> clock->shift, ce);
		/* ticks gets rounded down by one */
		ce->min_delta_ns =
			clockevent_delta2ns(clock->write_delay + 4, ce);
		ce->cpumask = cpumask_of(0);

		cs->mult = clocksource_hz2mult(clock->freq, cs->shift);
		res = clocksource_register(cs);
		if (res)
			printk(KERN_ERR "msm_timer_init: clocksource_register "
			       "failed for %s\n", cs->name);

		res = setup_irq(clock->irq.irq, &clock->irq);
		if (res)
			printk(KERN_ERR "msm_timer_init: setup_irq "
			       "failed for %s\n", cs->name);

		clockevents_register_device(ce);
	}
}

struct sys_timer msm_timer = {
	.init = msm_timer_init
};
