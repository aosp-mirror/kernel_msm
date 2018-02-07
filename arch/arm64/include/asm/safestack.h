#ifndef _ASM_SAFESTACK_H
#define _ASM_SAFESTACK_H

#include <asm/memory.h>

/* Clang assumes the unsafe stack is 16-byte aligned */
#define UNSAFE_STACK_ALIGN	16

#ifdef CONFIG_SAFESTACK_COLORING
#define UNSAFE_STACK_SIZE	THREAD_SIZE
#else
/*
 * Without stack coloring unsafe stack slots won't be reused, so we'll need
 * to allocate a larger stack to compensate.
 */
#define UNSAFE_STACK_SIZE	(2 * THREAD_SIZE)
#endif

#endif /*_ASM_SAFESTACK_H */
