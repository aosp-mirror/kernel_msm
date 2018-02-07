#ifndef _LINUX_SAFESTACK_H
#define _LINUX_SAFESTACK_H

#ifdef CONFIG_SAFESTACK
#include <asm/safestack.h>

#define UNSAFE_STACK_END_MAGIC	0x5A78A200

extern void init_unsafe_stack_cache(void);
extern int alloc_unsafe_stack(struct task_struct *tsk, int node);
extern void free_unsafe_stack(struct task_struct *tsk);

static inline void set_unsafe_stack_end_magic(struct task_struct *tsk)
{
	*(unsigned long *)tsk->unsafe_stack = UNSAFE_STACK_END_MAGIC;
}

static inline bool unsafe_stack_corrupted(struct task_struct *tsk)
{
	return *(unsigned long *)tsk->unsafe_stack != UNSAFE_STACK_END_MAGIC;
}

static inline bool object_is_on_unsafe_stack(void *obj)
{
	void *stack = current->unsafe_stack;

	return obj >= stack && obj < (stack + UNSAFE_STACK_SIZE);
}

#ifdef CONFIG_DEBUG_STACK_USAGE
static inline unsigned long unsafe_stack_not_used(struct task_struct *tsk)
{
	unsigned long *n = tsk->unsafe_stack;
	unsigned long *end = tsk->unsafe_stack + UNSAFE_STACK_SIZE;

	do { 	/* Skip over canary */
		n++;
	} while (n < end && !*n);

	return (unsigned long)n - (unsigned long)tsk->unsafe_stack;
}
#endif

#else /* CONFIG_SAFESTACK */

#define UNSAFE_STACK_SIZE THREAD_SIZE

static inline void init_unsafe_stack_cache(void)
{
}

static inline int alloc_unsafe_stack(struct task_struct *tsk, int node)
{
	return 0;
}

static inline void free_unsafe_stack(struct task_struct *tsk)
{
}

static inline void set_unsafe_stack_end_magic(struct task_struct *tsk)
{
}

static inline bool unsafe_stack_corrupted(struct task_struct *tsk)
{
	return false;
}

static inline bool object_is_on_unsafe_stack(void *obj)
{
	return false;
}

#ifdef CONFIG_DEBUG_STACK_USAGE
static inline unsigned long unsafe_stack_not_used(struct task_struct *tsk)
{
	return UNSAFE_STACK_SIZE;
}
#endif

#endif /* CONFIG_SAFESTACK */
#endif /* _LINUX_SAFESTACK_H */
