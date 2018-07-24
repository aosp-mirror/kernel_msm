// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit test for mock.h.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <test/test.h>
#include <test/mock.h>

#include "test-mock.h"

struct mock_test_context {
	struct MOCK(test)	*mock_test;
	struct mock		*mock;
};

static void mock_test_do_expect_basic(struct test *test)
{
	struct mock_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = ctx->mock_test;
	struct test *trgt = mock_get_trgt(mock_test);
	struct mock *mock = ctx->mock;
	int param0 = 5, param1 = -4;
	static const char * const two_param_types[] = {"int", "int"};
	const void *two_params[] = {&param0, &param1};
	struct mock_param_matcher *matchers_any_two[] = {any(trgt), any(trgt)};
	struct mock_expectation *expectation;
	const void *ret;

	expectation = mock_add_matcher(mock,
				       "",
				       NULL,
				       matchers_any_two,
				       ARRAY_SIZE(matchers_any_two));
	expectation->action = int_return(trgt, 5);
	EXPECT_EQ(test, 0, expectation->times_called);

	ret = mock->do_expect(mock,
			      "",
			      NULL,
			      two_param_types,
			      two_params,
			      ARRAY_SIZE(two_params));
	ASSERT_NOT_ERR_OR_NULL(test, ret);
	EXPECT_EQ(test, 5, *((int *) ret));
	EXPECT_EQ(test, 1, expectation->times_called);
}

static void mock_test_ptr_eq(struct test *test)
{
	struct mock_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = ctx->mock_test;
	struct test *trgt = mock_get_trgt(mock_test);
	struct mock *mock = ctx->mock;
	void *param0 = ctx, *param1 = trgt;
	static const char * const two_param_types[] = {"void *", "void *"};
	const void *two_params[] = {&param0, &param1};
	struct mock_param_matcher *matchers_two_ptrs[] = {
		ptr_eq(trgt, param0), ptr_eq(trgt, param1)
	};
	struct mock_expectation *expectation;
	const void *ret;

	expectation = mock_add_matcher(mock,
				       "",
				       NULL,
				       matchers_two_ptrs,
				       ARRAY_SIZE(matchers_two_ptrs));
	expectation->action = int_return(trgt, 0);
	EXPECT_EQ(test, 0, expectation->times_called);

	ret = mock->do_expect(mock,
			      "",
			      NULL,
			      two_param_types,
			      two_params,
			      ARRAY_SIZE(two_params));
	ASSERT_NOT_ERR_OR_NULL(test, ret);
	EXPECT_EQ(test, 1, expectation->times_called);
}

static void mock_test_ptr_eq_not_equal(struct test *test)
{
	struct mock_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = ctx->mock_test;
	struct test *trgt = mock_get_trgt(mock_test);
	struct mock *mock = ctx->mock;
	void *param0 = ctx, *param1 = trgt;
	static const char * const two_param_types[] = {"void *", "void *"};
	const void *two_params[] = {&param0, &param1};
	struct mock_param_matcher *matchers_two_ptrs[] = {
		ptr_eq(trgt, param0), ptr_eq(trgt, param1 - 1)
	};
	struct mock_expectation *expectation;
	const void *ret;

	expectation = mock_add_matcher(mock,
				       "",
				       NULL,
				       matchers_two_ptrs,
				       ARRAY_SIZE(matchers_two_ptrs));
	expectation->action = int_return(trgt, 0);
	EXPECT_EQ(test, 0, expectation->times_called);

	ret = mock->do_expect(mock,
			      "",
			      NULL,
			      two_param_types,
			      two_params,
			      ARRAY_SIZE(two_params));
	EXPECT_FALSE(test, ret);
	EXPECT_EQ(test, 0, expectation->times_called);
}

/*
 * In order for us to be able to rely on EXPECT_CALL to validate other behavior,
 * we need to test that unsatisfied EXPECT_CALL causes a test failure.
 */
static void mock_test_failed_expect_call_fails_test(struct test *test)
{
	struct mock_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = ctx->mock_test;
	struct mock *mock = ctx->mock;

	/* mock is a pretend mock belonging to our mocked_test */

	/* Put an expectation on mocked mock */
	EXPECT_CALL(fail(mock, any(mock_get_trgt(mock_test))));

	/*
	 * Expect that mock_test will fail because the above won't be satisfied
	 */
	EXPECT_CALL(fail(mock_get_ctrl(mock_test), any(test)));

	/*
	 * Validate expectations of mocked mock, which should fail mocked test
	 */
	mock_validate_expectations(mock);

	/* Validate mock_test's expectations, that is, it should have failed */
	mock_validate_expectations(mock_get_ctrl(mock_test));
	EXPECT_FALSE(test, mock_get_trgt(mock_test)->success);
}

static void mock_test_do_expect_default_return(struct test *test)
{
	struct mock_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = ctx->mock_test;
	struct test *trgt = mock_get_trgt(mock_test);
	struct mock *mock = ctx->mock;
	int param0 = 5, param1 = -5;
	static const char * const two_param_types[] = {"int", "int"};
	const void *two_params[] = {&param0, &param1};
	struct mock_param_matcher *matchers[] = {
		int_eq(trgt, 5),
		int_eq(trgt, -4)
	};
	struct mock_expectation *expectation;
	const void *ret;

	expectation = mock_add_matcher(mock,
				       "test_printk",
				       test_printk,
				       matchers,
				       ARRAY_SIZE(matchers));
	expectation->action = int_return(trgt, 5);
	EXPECT_EQ(test, 0, expectation->times_called);

	EXPECT_FALSE(test, mock_set_default_action(mock,
						   "test_printk",
						   test_printk,
						   int_return(trgt, -4)));

	ret = mock->do_expect(mock,
			      "test_printk",
			      test_printk,
			      two_param_types,
			      two_params,
			      ARRAY_SIZE(two_params));
	ASSERT_NOT_ERR_OR_NULL(test, ret);
	EXPECT_EQ(test, -4, *((int *) ret));
	EXPECT_EQ(test, 0, expectation->times_called);
}

/*
 * Method called on naggy mock with no expectations will not fail, but will show
 * a warning message
 */
static void mock_test_naggy_no_expectations_no_fail(struct test *test)
{
	struct mock_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = ctx->mock_test;
	struct test *trgt = mock_get_trgt(mock_test);
	struct mock *mock = ctx->mock;
	int param0 = 5, param1 = -5;
	static const char * const two_param_types[] = {"int", "int"};
	const void *two_params[] = {&param0, &param1};
	struct mock_expectation *expectation;

	mock_set_default_action(mock, "test_printk", test_printk,
		int_return(trgt, -4));

	expectation = EXPECT_CALL(fail(mock_get_ctrl(mock_test), any(test)));
	expectation->min_calls_expected = 0;
	expectation->max_calls_expected = 0;

	EXPECT_CALL(mock_vprintk(mock_get_ctrl(mock_test), any(test),
		va_format_cmp(test, str_contains(test,
			"Method was called with no expectations declared"),
		any(test))));

	mock->do_expect(mock,
			"test_printk",
			test_printk,
			two_param_types,
			two_params,
			ARRAY_SIZE(two_params));
	mock_validate_expectations(mock);
}

static void mock_test_mock_validate_expectations(struct test *test)
{
	struct mock_test_context *ctx = test->priv;
	struct MOCK(test) *mock_test = ctx->mock_test;
	struct test *trgt = mock_get_trgt(mock_test);
	struct mock *mock = ctx->mock;
	struct mock_param_matcher *matchers[] = {
		int_eq(trgt, 5),
		int_eq(trgt, -4)
	};
	struct mock_expectation *expectation;

	EXPECT_EQ(test, mock_get_trgt(mock_test), mock->test);

	expectation = mock_add_matcher(mock,
				       "test_printk",
				       test_printk,
				       matchers,
				       ARRAY_SIZE(matchers));
	expectation->times_called = 0;
	expectation->min_calls_expected = 1;
	expectation->max_calls_expected = 1;

	EXPECT_CALL(fail(mock_get_ctrl(mock_test), any(test)));

	mock_validate_expectations(mock);
}

void *do_mocked_fail(struct mock_action *this, const void **params, int len)
{
	static const int ret;
	struct test_stream * const *stream_ptr = params[0];
	struct test_stream *stream = *stream_ptr;

	stream->set_level(stream, KERN_ERR);
	stream->commit(stream);
	return (void *) &ret;
}

static struct mock_action mocked_fail = {
	.do_action = do_mocked_fail
};

static int mock_test_init(struct test *test)
{
	struct mock_test_context *ctx;

	ctx = test_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	test->priv = ctx;

	ctx->mock_test = CONSTRUCT_MOCK(test, test);
	if (!ctx->mock_test)
		return -EINVAL;

	ctx->mock = test_kzalloc(test, sizeof(*ctx->mock), GFP_KERNEL);
	if (!ctx->mock)
		return -ENOMEM;
	mock_init_ctrl(mock_get_trgt(ctx->mock_test), ctx->mock);

	/* This test suite tests the behaviour of the error messages printed
	 * when mocks fail, which requires the mocked fail to commit the
	 * stream.
	 */
	mock_set_default_action(mock_get_ctrl(ctx->mock_test),
		"fail", fail, &mocked_fail);
	return 0;
}

static struct test_case mock_test_cases[] = {
	TEST_CASE(mock_test_do_expect_basic),
	TEST_CASE(mock_test_ptr_eq),
	TEST_CASE(mock_test_ptr_eq_not_equal),
	TEST_CASE(mock_test_failed_expect_call_fails_test),
	TEST_CASE(mock_test_do_expect_default_return),
	TEST_CASE(mock_test_mock_validate_expectations),
	TEST_CASE(mock_test_naggy_no_expectations_no_fail),
	{},
};

static struct test_module mock_test_module = {
	.name = "mock-test",
	.init = mock_test_init,
	.test_cases = mock_test_cases,
};

module_test(mock_test_module);
