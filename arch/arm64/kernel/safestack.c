/*
 * SafeStack unsafe stack management
 *
 * Copyright (C) 2017 Google, Inc.
 */

#include <linux/sched.h>
#include <asm/current.h>

/* Per-CPU unsafe stack and unsafe stack pointer */
DEFINE_PER_CPU(unsigned long [UNSAFE_STACK_SIZE / sizeof(unsigned long)],
	irq_unsafe_stack) __aligned(UNSAFE_STACK_ALIGN);

__nosafestack void** __safestack_pointer_address(void)
{
	return &current->unsafe_stack_ptr;
}
EXPORT_SYMBOL(__safestack_pointer_address);
