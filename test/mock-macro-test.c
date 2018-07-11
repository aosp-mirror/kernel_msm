// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit test for parameter list parsing macros.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <test/test.h>
#include <test/params.h>

#define TO_STR_INTERNAL(...) #__VA_ARGS__
#define TO_STR(...) TO_STR_INTERNAL(__VA_ARGS__)

static void mock_macro_is_equal(struct test *test)
{
	EXPECT_STREQ(test, "dropped, 1", TO_STR(EQUAL(1, 1)));
	EXPECT_EQ(test, 1, DROP_FIRST_ARG(dropped, 1, 0));
	EXPECT_EQ(test, 0, DROP_FIRST_ARG(1, 0));
	EXPECT_EQ(test, 0, DROP_FIRST_ARG(EQUAL(1, 0), 0));
	EXPECT_EQ(test, 1, IS_EQUAL(1, 1));
	EXPECT_EQ(test, 0, IS_EQUAL(1, 0));
}

static void mock_macro_if(struct test *test)
{
	EXPECT_STREQ(test, "body", ""IF(1)("body"));
	EXPECT_STREQ(test, "", ""IF(0)("body"));
	EXPECT_STREQ(test, "body", ""IF(IS_EQUAL(1, 1))("body"));
	EXPECT_STREQ(test, "", ""IF(IS_EQUAL(0, 1))("body"));
}

static void mock_macro_apply_tokens(struct test *test)
{
	EXPECT_STREQ(test, "type", TO_STR(APPLY_TOKENS(type, 1, 0)));
	EXPECT_STREQ(test, ", type", TO_STR(APPLY_TOKENS(type, 1, 1)));
	EXPECT_STREQ(test, "", TO_STR(APPLY_TOKENS(type, 0, 1)));
}

#define IDENTITY(context, type, index) type

static void mock_macro_param_list_recurse(struct test *test)
{
	EXPECT_STREQ(test, "", TO_STR(PARAM_LIST_RECURSE(0,
							 0,
							 IDENTITY,
							 FILTER_NONE,
							 not_used)));
	EXPECT_STREQ(test, "type", TO_STR(EXPAND(PARAM_LIST_RECURSE(0,
								    1,
								    IDENTITY,
								    FILTER_NONE,
								    not_used,
								    type))));
	EXPECT_STREQ(test,
		     "type0 , type1 , type2 , type3 , type4 , type5 , "
		     "type6 , type7 , type8 , type9 , type10 , type11 , "
		     "type12 , type13 , type14 , type15",
		     TO_STR(EXPAND(PARAM_LIST_RECURSE(0, 16,
						      IDENTITY, FILTER_NONE,
						      not_used,
						      type0, type1, type2,
						      type3, type4, type5,
						      type6, type7, type8,
						      type9, type10, type11,
						      type12, type13, type14,
						      type15))));
}

static void mock_macro_for_each_param(struct test *test)
{
	EXPECT_STREQ(test, "type0 , type1", TO_STR(FOR_EACH_PARAM(IDENTITY,
								  FILTER_NONE,
								  not_used,
								  type0,
								  type1)));
	EXPECT_STREQ(test, "type1", TO_STR(FOR_EACH_PARAM(IDENTITY,
							  FILTER_INDEX,
							  0,
							  type0,
							  type1)));
}

static void mock_macro_param_list_from_types_basic(struct test *test)
{
	EXPECT_STREQ(test, "", TO_STR(PARAM_LIST_FROM_TYPES()));
	EXPECT_STREQ(test, "int arg0", TO_STR(PARAM_LIST_FROM_TYPES(int)));
	EXPECT_STREQ(test, "struct test_struct * arg0 , int arg1",
		     TO_STR(PARAM_LIST_FROM_TYPES(struct test_struct *, int)));
	EXPECT_STREQ(test,
		     "type0 arg0 , type1 arg1 , type2 arg2 , type3 arg3 , "
		     "type4 arg4 , type5 arg5 , type6 arg6 , type7 arg7 , "
		     "type8 arg8 , type9 arg9 , type10 arg10 , type11 arg11 , "
		     "type12 arg12 , type13 arg13 , type14 arg14 , "
		     "type15 arg15",
		     TO_STR(PARAM_LIST_FROM_TYPES(type0, type1, type2,
						  type3, type4, type5,
						  type6, type7, type8,
						  type9, type10, type11,
						  type12, type13, type14,
						  type15)));
}

static void mock_macro_arg_names_from_types(struct test *test)
{
	EXPECT_STREQ(test, "", TO_STR(ARG_NAMES_FROM_TYPES(0)));
	EXPECT_STREQ(test, "", TO_STR(ARG_NAMES_FROM_TYPES(0, int)));
	EXPECT_STREQ(test,
		     "arg1",
		     TO_STR(ARG_NAMES_FROM_TYPES(0,
						 struct test_struct *,
						 int)));
	EXPECT_STREQ(test,
		     "arg0 , arg1 , arg3 , arg4 , arg5 , arg6 , arg7 , arg8 , "
		     "arg9 , arg10 , arg11 , arg12 , arg13 , arg14 , arg15",
		     TO_STR(ARG_NAMES_FROM_TYPES(2, type0, type1, type2,
						  type3, type4, type5,
						  type6, type7, type8,
						  type9, type10, type11,
						  type12, type13, type14,
						  type15)));
}

static struct test_case mock_macro_test_cases[] = {
	TEST_CASE(mock_macro_is_equal),
	TEST_CASE(mock_macro_if),
	TEST_CASE(mock_macro_apply_tokens),
	TEST_CASE(mock_macro_param_list_recurse),
	TEST_CASE(mock_macro_for_each_param),
	TEST_CASE(mock_macro_param_list_from_types_basic),
	TEST_CASE(mock_macro_arg_names_from_types),
	{},
};

static struct test_module mock_macro_test_module = {
	.name = "mock-macro-test",
	.test_cases = mock_macro_test_cases,
};
module_test(mock_macro_test_module);
