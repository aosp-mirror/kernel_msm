/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/remote_spinlock.h>
#include "smd_private.h"

#define SMEM_SPINLOCK_COUNT 8
#define SMEM_SPINLOCK_ARRAY_SIZE (SMEM_SPINLOCK_COUNT * sizeof(uint32_t))

struct raw_remote_spinlock {
	union {
		volatile u32 lock;
		struct {
			volatile u8 self_lock;
			volatile u8 other_lock;
			volatile u8 next_yield;
			u8 pad;
		} dek;
	};
};

static inline void __raw_remote_ex_spin_lock(struct raw_remote_spinlock *lock)
{
	unsigned long tmp;

	asm volatile (
		"1:	ldrex	%0, [%1]\n"
		"	teq	%0, #0\n"
		"	strexeq	%0, %2, [%1]\n"
		"	teqeq	%0, #0\n"
		"	bne	1b"
		: "=&r" (tmp)
		: "r" (&lock->lock), "r" (1)
		: "cc");

	smp_mb();
}

static inline void __raw_remote_ex_spin_unlock(struct raw_remote_spinlock *lock)
{
	smp_mb();

	asm volatile (
		"	str	%1, [%0]\n"
		:
		: "r" (&lock->lock), "r" (0)
		: "cc");
}

static inline void __raw_remote_swp_spin_lock(struct raw_remote_spinlock *lock)
{
	unsigned long tmp;

	asm volatile (
		"1:	swp	%0, %2, [%1]\n"
		"	teq	%0, #0\n"
		"	bne	1b"
		: "=&r" (tmp)
		: "r" (&lock->lock), "r" (1)
		: "cc");

	smp_mb();
}

static inline void __raw_remote_swp_spin_unlock(struct raw_remote_spinlock *lock)
{
	smp_mb();

	asm volatile (
		"	str	%1, [%0]"
		:
		: "r" (&lock->lock), "r" (0)
		: "cc");
}

#define DEK_LOCK_REQUEST		1
#define DEK_LOCK_YIELD			(!DEK_LOCK_REQUEST)
#define DEK_YIELD_TURN_SELF		0
static void __raw_remote_dek_spin_lock(struct raw_remote_spinlock *lock)
{
	lock->dek.self_lock = DEK_LOCK_REQUEST;

	while (lock->dek.other_lock) {

		if (lock->dek.next_yield == DEK_YIELD_TURN_SELF)
			lock->dek.self_lock = DEK_LOCK_YIELD;

		while (lock->dek.other_lock)
			;

		lock->dek.self_lock = DEK_LOCK_REQUEST;
	}
	lock->dek.next_yield = DEK_YIELD_TURN_SELF;

	smp_mb();
}

static void __raw_remote_dek_spin_unlock(struct raw_remote_spinlock *lock)
{
	smp_mb();

	lock->dek.self_lock = DEK_LOCK_YIELD;
}

#if defined(CONFIG_MSM_REMOTE_SPINLOCK_DEKKERS)
/* Use Dekker's algorithm when LDREX/STREX and SWP are unavailable for
 * shared memory */
#define _raw_remote_spin_lock(lock)	__raw_remote_dek_spin_lock(lock)
#define _raw_remote_spin_unlock(lock)	__raw_remote_dek_spin_unlock(lock)
#elif defined(CONFIG_MSM_REMOTE_SPINLOCK_SWP)
/* Use SWP-based locks when LDREX/STREX are unavailable for shared memory. */
#define _raw_remote_spin_lock(lock)	__raw_remote_swp_spin_lock(lock)
#define _raw_remote_spin_unlock(lock)	__raw_remote_swp_spin_unlock(lock)
#else
/* Use LDREX/STREX for shared memory locking, when available */
#define _raw_remote_spin_lock(lock)	__raw_remote_ex_spin_lock(lock)
#define _raw_remote_spin_unlock(lock)	__raw_remote_ex_spin_unlock(lock)
#endif

void _remote_spin_lock(remote_spinlock_t *lock)
{
	_raw_remote_spin_lock(lock->remote);
}
EXPORT_SYMBOL(_remote_spin_lock);

void _remote_spin_unlock(remote_spinlock_t *lock)
{
	_raw_remote_spin_unlock(lock->remote);
}
EXPORT_SYMBOL(_remote_spin_unlock);

static int remote_spin_lock_smem_init(remote_spinlock_t *lock, int id)
{
	void *start;

	if (id >= SMEM_SPINLOCK_COUNT)
		return -EINVAL;

	start = smem_alloc(SMEM_SPINLOCK_ARRAY, SMEM_SPINLOCK_ARRAY_SIZE);
	if (start == NULL)
		return -ENXIO;

	lock->remote =
		(struct raw_remote_spinlock *)(start + id * sizeof(uint32_t));
	return 0;
}

#define DAL_CHUNK_NAME_LENGTH 12
struct dal_chunk_header {
	uint32_t size;
	char name[DAL_CHUNK_NAME_LENGTH];
	uint32_t lock;
	uint32_t reserved;
	uint32_t type;
	uint32_t version;
};

static int remote_spin_lock_dal_init(remote_spinlock_t *lock, const char *name)
{
	unsigned long start;
	unsigned long end;
	unsigned size;
	struct dal_chunk_header *cur_hdr;

	if (!name)
		return -EINVAL;

	start = (unsigned long)smem_item(SMEM_DAL_AREA, &size);
	if (!start)
		return -ENXIO;

	end = start + size;

	/* Find first chunk header */
	cur_hdr = (struct dal_chunk_header *)ALIGN(start, 4096);
	lock->remote = NULL;
	while (((unsigned long)(cur_hdr + 1) <= end) && (cur_hdr->size != 0)) {
		if (!strncmp(cur_hdr->name, name, DAL_CHUNK_NAME_LENGTH)) {
			lock->remote =
				(struct raw_remote_spinlock *)&cur_hdr->lock;
			return 0;
		}
		cur_hdr = (void *)cur_hdr + cur_hdr->size;
	}

	pr_err("%s: DAL remote spin lock '%s' not found.\n", __func__, name);
	return -EINVAL;
}

int _remote_spin_lock_init(remote_spinlock_t *lock, const char *name)
{
	BUG_ON(name == NULL);

	/* remote spinlocks can be one of two formats:
	 * D:<dal chunk name>
	 * S:<single digit smem lock id>
	 */
	if (!strncmp(name, "D:", 2)) {
		return remote_spin_lock_dal_init(lock, &name[2]);
	} else if (!strncmp(name, "S:", 2)) {
		BUG_ON(name[3] != '\0');
		return remote_spin_lock_smem_init(lock, (uint8_t)(name[2]-'0'));
	}

	return -EINVAL;
}
EXPORT_SYMBOL(_remote_spin_lock_init);
