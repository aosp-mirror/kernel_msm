/*
 * Shadow Call Stack support.
 *
 * Copyright (C) 2018 Google LLC
 */

#include <linux/cpuhotplug.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/slab.h>
#include <linux/scs.h>
#include <linux/vmstat.h>

#include <asm/scs.h>

#define SCS_END_MAGIC	0xaf0194819b1635f6UL

static inline void *__scs_base(struct task_struct *tsk)
{
	return (void *)((uintptr_t)task_scs(tsk) & ~(SCS_SIZE - 1));
}

static inline void *scs_alloc(int node)
{
	return kmalloc(SCS_SIZE, SCS_GFP);
}

static inline void scs_free(void *s)
{
	kfree(s);
}

static struct page *__scs_page(struct task_struct *tsk)
{
	return virt_to_page(__scs_base(tsk));
}

static inline unsigned long *scs_magic(struct task_struct *tsk)
{
	return (unsigned long *)(__scs_base(tsk) + SCS_SIZE - sizeof(long));
}

static inline void scs_set_magic(struct task_struct *tsk)
{
	*scs_magic(tsk) = SCS_END_MAGIC;
}

void scs_task_init(struct task_struct *tsk)
{
	task_set_scs(tsk, NULL);
}

void scs_set_init_magic(struct task_struct *tsk)
{
	scs_save(tsk);
	scs_set_magic(tsk);
	scs_load(tsk);
}

static void scs_account(struct task_struct *tsk, int account)
{
	mod_zone_page_state(page_zone(__scs_page(tsk)), NR_KERNEL_SCS_BYTES,
		account * SCS_SIZE);
}

int scs_prepare(struct task_struct *tsk, int node)
{
	void *s;

	s = scs_alloc(node);
	if (!s)
		return -ENOMEM;

	task_set_scs(tsk, s);
	scs_set_magic(tsk);
	scs_account(tsk, 1);

	return 0;
}

#ifdef CONFIG_DEBUG_STACK_USAGE
static inline unsigned long scs_used(struct task_struct *tsk)
{
	unsigned long *p = __scs_base(tsk);
	unsigned long *end = scs_magic(tsk);
	uintptr_t s = (uintptr_t)p;

	while (p < end && *p)
		p++;

	return (uintptr_t)p - s;
}

static void scs_check_usage(struct task_struct *tsk)
{
	static DEFINE_SPINLOCK(lock);
	static unsigned long highest;
	unsigned long used = scs_used(tsk);

	if (used <= highest)
		return;

	spin_lock(&lock);

	if (used > highest) {
		pr_info("%s: highest shadow stack usage %lu bytes\n",
			__func__, used);
		highest = used;
	}

	spin_unlock(&lock);
}
#else
static inline void scs_check_usage(struct task_struct *tsk)
{
}
#endif

bool scs_corrupted(struct task_struct *tsk)
{
	return *scs_magic(tsk) != SCS_END_MAGIC;
}

void scs_release(struct task_struct *tsk)
{
	void *s;

	s = __scs_base(tsk);
	if (!s)
		return;

	BUG_ON(scs_corrupted(tsk));
	scs_check_usage(tsk);

	scs_account(tsk, -1);
	scs_task_init(tsk);
	scs_free(s);
}
