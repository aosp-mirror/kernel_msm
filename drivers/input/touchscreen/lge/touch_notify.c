/*
 * touch_core.c
 *
 * Copyright (c) 2015 LGE.
 *
 * author : hoyeon.jang@lge.com
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

#include <linux/notifier.h>
#include <linux/input/lge_touch_notify.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(touch_blocking_notifier);
static ATOMIC_NOTIFIER_HEAD(touch_atomic_notifier);

int touch_blocking_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&touch_blocking_notifier, nb);
}
EXPORT_SYMBOL(touch_blocking_notifier_register);

int touch_blocking_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&touch_blocking_notifier, nb);
}
EXPORT_SYMBOL(touch_blocking_notifier_unregister);

int touch_blocking_notifier_call(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&touch_blocking_notifier, val, v);
}
EXPORT_SYMBOL_GPL(touch_blocking_notifier_call);

int touch_atomic_notifier_register(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&touch_atomic_notifier, nb);
}
EXPORT_SYMBOL(touch_atomic_notifier_register);

int touch_atomic_notifier_unregister(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&touch_atomic_notifier, nb);
}
EXPORT_SYMBOL(touch_atomic_notifier_unregister);

int touch_atomic_notifier_call(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&touch_atomic_notifier, val, v);
}
EXPORT_SYMBOL_GPL(touch_atomic_notifier_call);

int touch_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&touch_blocking_notifier, nb);
}
EXPORT_SYMBOL(touch_register_client);

int touch_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&touch_blocking_notifier, nb);
}
EXPORT_SYMBOL(touch_unregister_client);

int touch_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&touch_blocking_notifier, val, v);
}
EXPORT_SYMBOL_GPL(touch_notifier_call_chain);

