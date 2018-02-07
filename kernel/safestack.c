/*
 * SafeStack unsafe stack management
 *
 * Copyright (C) 2017 Google, Inc.
 */

#include <linux/sched.h>
#include <linux/safestack.h>
#include <linux/mm.h>
#include <linux/memcontrol.h>
#include <linux/slab.h>

static struct kmem_cache *unsafe_stack_cache;

void init_unsafe_stack_cache()
{
	unsafe_stack_cache = kmem_cache_create("unsafe_stack",
					UNSAFE_STACK_SIZE, UNSAFE_STACK_ALIGN,
					0, NULL);
	BUG_ON(unsafe_stack_cache == NULL);
}

int alloc_unsafe_stack(struct task_struct *tsk, int node)
{
	struct page *first;
	void *stack;

	stack = kmem_cache_alloc_node(unsafe_stack_cache,
				      THREADINFO_GFP | __GFP_ZERO, node);
	if (unlikely(!stack))
		return -ENOMEM;

	first = virt_to_page(stack);

	/* account as kernel stack */
	mod_zone_page_state(page_zone(first), NR_KERNEL_STACK_KB,
		UNSAFE_STACK_SIZE / 1024);
	memcg_kmem_update_page_stat(first, MEMCG_KERNEL_STACK_KB,
		UNSAFE_STACK_SIZE / 1024);

	tsk->unsafe_stack = stack;
	tsk->unsafe_stack_ptr = stack + UNSAFE_STACK_SIZE;
	tsk->unsafe_saved_ptr = NULL;

	return 0;
}

void free_unsafe_stack(struct task_struct *tsk)
{
	struct page *first;

	if (unlikely(!tsk->unsafe_stack))
		return;

	first = virt_to_page(tsk->unsafe_stack);

	mod_zone_page_state(page_zone(first), NR_KERNEL_STACK_KB,
		-(long)UNSAFE_STACK_SIZE / 1024);
	memcg_kmem_update_page_stat(first, MEMCG_KERNEL_STACK_KB,
		-(long)UNSAFE_STACK_SIZE / 1024);

	kmem_cache_free(unsafe_stack_cache, tsk->unsafe_stack);

	tsk->unsafe_stack = NULL;
	tsk->unsafe_stack_ptr = NULL;
	tsk->unsafe_saved_ptr = NULL;
}
