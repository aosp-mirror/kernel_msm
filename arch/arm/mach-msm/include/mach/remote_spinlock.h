/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ASM__ARCH_MSM_REMOTE_SPINLOCK_H
#define __ASM__ARCH_MSM_REMOTE_SPINLOCK_H

#include <linux/spinlock.h>
#include <linux/types.h>

/* Remote spinlock type definitions. */
struct raw_remote_spinlock;
typedef struct {
	spinlock_t			local;
#if defined(CONFIG_MSM_REMOTE_SPINLOCK)
	struct raw_remote_spinlock	*remote;
#endif
} remote_spinlock_t;

#if defined(CONFIG_MSM_REMOTE_SPINLOCK)
int _remote_spin_lock_init(remote_spinlock_t *lock, const char *name);
void _remote_spin_lock(remote_spinlock_t *lock);
void _remote_spin_unlock(remote_spinlock_t *lock);
#else
static inline int _remote_spin_lock_init(remote_spinlock_t *lock,
					 const char *name) { return 0; }
static inline void _remote_spin_lock(remote_spinlock_t *lock) { }
static inline void _remote_spin_unlock(remote_spinlock_t *lock) { }
#endif

/* Note: only the below functions constitute the supported interface */
static inline int remote_spin_lock_init(remote_spinlock_t *lock,
					const char *name)
{
	spin_lock_init(&lock->local);
	return _remote_spin_lock_init(lock, name);
}

#define remote_spin_lock(lock)				\
	do {						\
		typecheck(remote_spinlock_t *, lock);	\
		spin_lock(&((lock)->local));		\
		_remote_spin_lock(lock);		\
	} while (0)

#define remote_spin_unlock(lock)			\
	do {						\
		typecheck(remote_spinlock_t *, lock);	\
		_remote_spin_unlock(lock);		\
		spin_unlock(&((lock)->local));		\
	} while (0)


#define remote_spin_lock_irqsave(lock,flags)			\
	do {							\
		typecheck(remote_spinlock_t *, lock);		\
		spin_lock_irqsave(&((lock)->local), (flags));	\
		_remote_spin_lock(lock);			\
	} while (0)

#define remote_spin_unlock_irqrestore(lock,flags)			\
	do {								\
		typecheck(remote_spinlock_t *, lock);			\
		_remote_spin_unlock(lock);				\
		spin_unlock_irqrestore(&((lock)->local), (flags));	\
	} while (0)

#endif /* __ASM__ARCH_MSM_REMOTE_SPINLOCK_H */
