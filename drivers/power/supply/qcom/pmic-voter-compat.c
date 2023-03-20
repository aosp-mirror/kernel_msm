/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <misc/gvotable.h>
#include <linux/pmic-voter.h>

#define V2EL(x) ((struct gvotable_election *)(v))
#define MAX_NAME_LEN 16

struct votable_data {
	const char *name;
	void *callback_data;   /* data passed to create_votable */
	int (*callback)(struct votable *votable,
			void *data,
			int effective_result,
			const char *effective_client);
};

bool is_client_vote_enabled_locked(struct votable *v, const char *client_str)
{
	int ret;
	bool enabled = false;

	ret = gvotable_is_enabled(V2EL(v), client_str, &enabled);
	if (ret)
		pr_err("Error gvotable_is_enabled returned %d\n", ret);

	return enabled;
}

bool is_client_vote_enabled(struct votable *votable, const char *client_str)
{
	return is_client_vote_enabled_locked(votable, client_str);
}

int get_client_vote_locked(struct votable *v, const char *client_str)
{
	void *ptr;
	int ret;

	ret = gvotable_get_vote(V2EL(votable), client_str, &ptr);
	return ret ? : (uintptr_t)ptr;
}

int get_client_vote(struct votable *votable, const char *client_str)
{
	return get_client_vote_locked(votable, client_str);
}
EXPORT_SYMBOL_GPL(get_client_vote);

int get_effective_result_locked(struct votable *v)
{
	const void *ptr;
	int ret;

	ret = gvotable_get_current_vote(V2EL(v), &ptr);
	return ret ? : (uintptr_t)ptr;
}
EXPORT_SYMBOL_GPL(get_effective_result_locked);

int get_effective_result(struct votable *votable)
{
	return get_effective_result_locked(votable);
}
EXPORT_SYMBOL_GPL(get_effective_result);

int vote(struct votable *v, const char *client_str, bool state, int val)
{
	return gvotable_cast_vote(V2EL(v), client_str, (void *)(long)val,
				  state);
}
EXPORT_SYMBOL_GPL(vote);

int vote_override(struct votable *votable, const char *override_client,
		  bool enabled, int val)
{
	return 0;
}
EXPORT_SYMBOL_GPL(vote_override);

bool is_override_vote_enabled_locked(struct votable *votable)
{
	return false;
}
EXPORT_SYMBOL_GPL(is_override_vote_enabled_locked);

bool is_override_vote_enabled(struct votable *votable)
{
	return false;
}
EXPORT_SYMBOL_GPL(is_override_vote_enabled);

int rerun_election(struct votable *votable)
{
	return 0;
}
EXPORT_SYMBOL_GPL(rerun_election);

const char * get_effective_client(struct votable *v) {
	static char client_str[GVOTABLE_MAX_REASON_LEN] = { 0 };
	gvotable_get_current_reason(V2EL(v), client_str, sizeof(client_str));
	return client_str;

}
EXPORT_SYMBOL_GPL(get_effective_client);

/*
 * It needs to work for new and instances so we can mix the code
 */
struct votable *find_votable(const char *name)
{
	char truncated_name[MAX_NAME_LEN];

	if (!name)
		return ERR_PTR(-EINVAL);
	strlcpy(truncated_name, name, MAX_NAME_LEN);
	return (struct votable *)gvotable_election_get_handle(truncated_name);
}
EXPORT_SYMBOL_GPL(find_votable);

static int pmic_voter_compat_cb(struct gvotable_election *el,
				 const char *cb_reason, void *cb_result)
{
	struct votable_data *vd = (struct votable_data *)gvotable_get_data(el);
	char reason[GVOTABLE_MAX_REASON_LEN] = { 0 };
	char *effective_reason = NULL;
	int effective_result = -EINVAL;
	const void *ptr;
	int ret;

	if (!vd->callback)
		return -EINVAL;

	ret = gvotable_get_current_vote(el, &ptr);
	if (ret == 0)
		effective_result = (uintptr_t)ptr;

	ret = gvotable_get_current_reason(el, reason, sizeof(reason));
	if (ret > 0) {
		effective_reason = reason;
		ret = 0;
	}

	/* for SET_ANY voter, the value is always same as enabled. */
	pr_debug("%s: name=%s result=%d reason=%s\n", __func__, vd->name,
		 effective_result, effective_reason ? effective_reason : "<>");

	/* call with NULL reason and -EINVAL if votes no enabled */
	vd->callback((struct votable *)el, vd->callback_data,
			effective_result, effective_reason);
	return ret;
}

/* Allow redefining the allocator: required for testing */
#ifndef kzalloc_votable
#define kzalloc_votable(p, f) (typeof(p))kzalloc(sizeof(*(p)), f)
#endif

struct votable *create_votable(const char *name,
		int votable_type,
		int (*callback)(struct votable *votable,
				void *data,
				int effective_result,
				const char *effective_client),
		void *callback_data)
{
	int (*comp_fn)(void * , void *) = NULL;
	struct gvotable_election *el;
	struct votable_data *vd;
	int ret;
	char truncated_name[MAX_NAME_LEN];

	if (!name)
		return ERR_PTR(-EINVAL);

	strlcpy(truncated_name, name, MAX_NAME_LEN);
	/* create extra votable data */
	vd = kzalloc_votable(vd, GFP_KERNEL);
	if (!vd)
		return ERR_PTR(-ENOMEM);
	vd->callback_data = callback_data;
	vd->callback = callback;
	vd->name = truncated_name;

	switch (votable_type) {
	case VOTE_MIN:
		comp_fn = gvotable_comparator_int_min;
		break;
	case VOTE_MAX:
		comp_fn = gvotable_comparator_int_max;
		break;
	case VOTE_SET_ANY:
		break;
	default:
		kfree(vd);
		return ERR_PTR(-EINVAL);
		break;
	}

	if (votable_type == VOTE_SET_ANY) {
		el = gvotable_create_bool_election(NULL, pmic_voter_compat_cb,
						   vd);
	} else {
		el = gvotable_create_int_election(NULL, comp_fn,
						  pmic_voter_compat_cb,
						  vd);
	}

	if (IS_ERR_OR_NULL(el)) {
		kfree(vd);
		return ERR_PTR(-ENOMEM);
	}

	gvotable_set_vote2str(el, gvotable_v2s_int);
	/* votables of type ANY have a default but they don't use it */
	if (votable_type == VOTE_SET_ANY) {
		gvotable_set_default(el, 0);
		gvotable_use_default(el, false);
	}

	ret = gvotable_election_set_name(el, vd->name);
	if (ret < 0) {
		gvotable_destroy_election(el);
		kfree(vd);
		return ERR_PTR(-EEXIST);
	}


	return (struct votable *)el;
}
EXPORT_SYMBOL_GPL(create_votable);

void destroy_votable(struct votable *v)
{
	if (!v)
		return;

	kfree(gvotable_get_data(V2EL(v)));
	gvotable_destroy_election(V2EL(v));
}
EXPORT_SYMBOL_GPL(destroy_votable);

MODULE_AUTHOR("Jim Wylder <jwylder@google.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_DESCRIPTION("QC PMIC Votable compatibility");
MODULE_LICENSE("GPL");
