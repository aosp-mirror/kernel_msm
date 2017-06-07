// SPDX-License-Identifier: GPL-2.0
/*
 * Base unit test (KUnit) API.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <os.h>
#include <test/test.h>

static int test_vprintk_emit(const struct test *test,
			     int level,
			     const char *fmt,
			     va_list args)
{
	return vprintk_emit(0, level, NULL, 0, fmt, args);
}

static int test_printk_emit(const struct test *test,
			    int level,
			    const char *fmt, ...)
{
	va_list args;
	int ret;

	va_start(args, fmt);
	ret = test_vprintk_emit(test, level, fmt, args);
	va_end(args);

	return ret;
}

static void test_vprintk(const struct test *test,
			 const char *level,
			 struct va_format *vaf)
{
	test_printk_emit(test,
			 level[1] - '0',
			 "kunit %s: %pV", test->name, vaf);
}

static void test_fail(struct test *test, struct test_stream *stream)
{
	test->success = false;
	stream->set_level(stream, KERN_ERR);
	stream->commit(stream);
}

int test_init_test(struct test *test, const char *name)
{
	INIT_LIST_HEAD(&test->resources);
	test->name = name;
	test->vprintk = test_vprintk;
	test->fail = test_fail;

	return 0;
}

/*
 * Initializes and runs test case. Does not clean up or do post validations.
 */
static void test_run_case_internal(struct test *test,
				   struct test_module *module,
				   struct test_case *test_case)
{
	int ret;

	if (module->init) {
		ret = module->init(test);
		if (ret) {
			test_err(test, "failed to initialize: %d", ret);
			test->success = false;
			return;
		}
	}

	test_case->run_case(test);
}

static void test_case_internal_cleanup(struct test *test)
{
	test_cleanup(test);
}

/*
 * Performs post validations and cleanup after a test case was run.
 * XXX: Should ONLY BE CALLED AFTER test_run_case_internal!
 */
static void test_run_case_cleanup(struct test *test,
				  struct test_module *module,
				  struct test_case *test_case)
{
	if (module->exit)
		module->exit(test);

	test_case_internal_cleanup(test);
}

/*
 * Performs all logic to run a test case.
 */
static bool test_run_case(struct test *test,
			  struct test_module *module,
			  struct test_case *test_case)
{
	test->success = true;

	test_run_case_internal(test, module, test_case);
	test_run_case_cleanup(test, module, test_case);

	return test->success;
}

int test_run_tests(struct test_module *module)
{
	bool all_passed = true, success;
	struct test_case *test_case;
	struct test test;
	int ret;

	ret = test_init_test(&test, module->name);
	if (ret)
		return ret;

	for (test_case = module->test_cases; test_case->run_case; test_case++) {
		success = test_run_case(&test, module, test_case);
		if (!success)
			all_passed = false;

		test_info(&test,
			  "%s %s",
			  test_case->name,
			  success ? "passed" : "failed");
	}

	if (all_passed)
		test_info(&test, "all tests passed");
	else
		test_info(&test, "one or more tests failed");

	return 0;
}

struct test_resource *test_alloc_resource(struct test *test,
					  int (*init)(struct test_resource *,
						      void *),
					  void (*free)(struct test_resource *),
					  void *context)
{
	struct test_resource *res;
	int ret;

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res)
		return NULL;

	ret = init(res, context);
	if (ret)
		return NULL;

	res->free = free;
	list_add_tail(&res->node, &test->resources);

	return res;
}

void test_free_resource(struct test *test, struct test_resource *res)
{
	res->free(res);
	list_del(&res->node);
	kfree(res);
}

struct test_kmalloc_params {
	size_t size;
	gfp_t gfp;
};

static int test_kmalloc_init(struct test_resource *res, void *context)
{
	struct test_kmalloc_params *params = context;

	res->allocation = kmalloc(params->size, params->gfp);
	if (!res->allocation)
		return -ENOMEM;

	return 0;
}

static void test_kmalloc_free(struct test_resource *res)
{
	kfree(res->allocation);
}

void *test_kmalloc(struct test *test, size_t size, gfp_t gfp)
{
	struct test_kmalloc_params params;
	struct test_resource *res;

	params.size = size;
	params.gfp = gfp;

	res = test_alloc_resource(test,
				  test_kmalloc_init,
				  test_kmalloc_free,
				  &params);

	if (res)
		return res->allocation;
	else
		return NULL;
}

void test_cleanup(struct test *test)
{
	struct test_resource *resource, *resource_safe;

	list_for_each_entry_safe(resource,
				 resource_safe,
				 &test->resources,
				 node) {
		test_free_resource(test, resource);
	}
}

void test_printk(const char *level,
		 const struct test *test,
		 const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	test->vprintk(test, level, &vaf);

	va_end(args);
}
