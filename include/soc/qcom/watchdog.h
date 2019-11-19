/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 */

#ifndef _ASM_ARCH_MSM_WATCHDOG_H_
#define _ASM_ARCH_MSM_WATCHDOG_H_

#ifdef CONFIG_QCOM_FORCE_WDOG_BITE_ON_PANIC
#define WDOG_BITE_ON_PANIC 1
#else
#define WDOG_BITE_ON_PANIC 0
#endif

#if IS_ENABLED(CONFIG_QCOM_WATCHDOG_V2)
#include <linux/module.h>

extern void _msm_trigger_wdog_bite(void) __cold;

static inline void msm_trigger_wdog_bite(void) __cold;
static inline void msm_trigger_wdog_bite(void)
{
	void (*thunk)(void);

	thunk = symbol_get(_msm_trigger_wdog_bite);
	if (!thunk) {
		pr_err("Failed to find _msm_trigger_wdog_bite\n");
		return;
	}
	(*thunk)();
	symbol_put(thunk);
}
#else
static inline void msm_trigger_wdog_bite(void) { }
#endif

#endif
