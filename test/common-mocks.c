// SPDX-License-Identifier: GPL-2.0
/*
 * Common KUnit mock call arg matchers and formatters.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <linux/kernel.h>
#include <test/mock.h>

static bool match_any(struct mock_param_matcher *pmatcher,
		      struct test_stream *stream,
		      const void *actual)
{
	stream->add(stream, "don't care");
	return true;
}

static struct mock_param_matcher any_matcher = {
	.match = match_any,
};

struct mock_param_matcher *any(struct test *test)
{
	return &any_matcher;
}

#define DEFINE_MATCHER_STRUCT(type_name, type)				       \
		struct mock_##type_name##_matcher {			       \
			struct mock_param_matcher matcher;		       \
			type expected;					       \
		};

#define DEFINE_TO_MATCHER_STRUCT(type_name)				       \
		struct mock_##type_name##_matcher *			       \
		to_mock_##type_name##_matcher(				       \
				struct mock_param_matcher *matcher)	       \
		{							       \
			return container_of(matcher,			       \
					    struct mock_##type_name##_matcher, \
					    matcher);			       \
		}

#define DEFINE_MATCH_FUNC(type_name, type, op_name, op)			       \
		bool match_##type_name##_##op_name(			       \
				struct mock_param_matcher *pmatcher,	       \
				struct test_stream *stream,		       \
				const void *pactual)			       \
		{							       \
			struct mock_##type_name##_matcher *matcher =	       \
				to_mock_##type_name##_matcher(pmatcher);       \
			type actual = *((type *) pactual);		       \
			bool matches = actual op matcher->expected;	       \
									       \
			if (matches)					       \
				stream->add(stream,			       \
					    "%d "#op" %d",		       \
					    actual,			       \
					    matcher->expected);		       \
			else						       \
				stream->add(stream,			       \
					    "%d not "#op" %d",		       \
					    actual,			       \
					    matcher->expected);		       \
									       \
			return matches;					       \
		}

#define DEFINE_MATCH_FACTORY(type_name, type, op_name)			       \
		struct mock_param_matcher *type_name##_##op_name(	       \
				struct test *test, type expected)	       \
		{							       \
			struct mock_##type_name##_matcher *matcher;	       \
									       \
			matcher = test_kmalloc(test,			       \
					       sizeof(*matcher),	       \
					       GFP_KERNEL);		       \
			if (!matcher)					       \
				return NULL;				       \
									       \
			matcher->matcher.match = match_##type_name##_##op_name;\
			matcher->expected = expected;			       \
			return &matcher->matcher;			       \
		}

#define DEFINE_MATCHER_WITH_TYPENAME(type_name, type)			       \
		DEFINE_MATCHER_STRUCT(type_name, type)			       \
		DEFINE_TO_MATCHER_STRUCT(type_name)			       \
		DEFINE_MATCH_FUNC(type_name, type, eq, ==)		       \
		DEFINE_MATCH_FACTORY(type_name, type, eq)		       \
		DEFINE_MATCH_FUNC(type_name, type, ne, !=)		       \
		DEFINE_MATCH_FACTORY(type_name, type, ne)		       \
		DEFINE_MATCH_FUNC(type_name, type, le, <=)		       \
		DEFINE_MATCH_FACTORY(type_name, type, le)		       \
		DEFINE_MATCH_FUNC(type_name, type, lt, <)		       \
		DEFINE_MATCH_FACTORY(type_name, type, lt)		       \
		DEFINE_MATCH_FUNC(type_name, type, ge, >=)		       \
		DEFINE_MATCH_FACTORY(type_name, type, ge)		       \
		DEFINE_MATCH_FUNC(type_name, type, gt, >)		       \
		DEFINE_MATCH_FACTORY(type_name, type, gt)

#define DEFINE_MATCHER(type) DEFINE_MATCHER_WITH_TYPENAME(type, type)

DEFINE_MATCHER(u8);
DEFINE_MATCHER(u16);
DEFINE_MATCHER(u32);
DEFINE_MATCHER(u64);
DEFINE_MATCHER(char);
DEFINE_MATCHER_WITH_TYPENAME(uchar, unsigned char);
DEFINE_MATCHER_WITH_TYPENAME(schar, signed char);
DEFINE_MATCHER(short);
DEFINE_MATCHER_WITH_TYPENAME(ushort, unsigned short);
DEFINE_MATCHER(int);
DEFINE_MATCHER_WITH_TYPENAME(uint, unsigned int);
DEFINE_MATCHER(long);
DEFINE_MATCHER_WITH_TYPENAME(ulong, unsigned long);
DEFINE_MATCHER_WITH_TYPENAME(longlong, long long);
DEFINE_MATCHER_WITH_TYPENAME(ulonglong, unsigned long long);

DEFINE_MATCHER_WITH_TYPENAME(ptr, void *);

struct mock_memeq_matcher {
	struct mock_param_matcher matcher;
	const void *expected;
	size_t size;
};

static bool match_memeq(struct mock_param_matcher *pmatcher,
			struct test_stream *stream,
			const void *pactual)
{
	struct mock_memeq_matcher *matcher =
			container_of(pmatcher,
				     struct mock_memeq_matcher,
				     matcher);
	const void *actual = CONVERT_TO_ACTUAL_TYPE(const void *, pactual);
	bool matches = !memcmp(actual, matcher->expected, matcher->size);
	int i;

	for (i = 0; i < matcher->size; i++)
		stream->add(stream, "%02x, ", ((const char *) actual)[i]);
	if (matches)
		stream->add(stream, "== ");
	else
		stream->add(stream, "!= ");
	for (i = 0; i < matcher->size; i++)
		stream->add(stream,
			    "%02x, ",
			    ((const char *) matcher->expected)[i]);

	return matches;
}

struct mock_param_matcher *memeq(struct test *test,
				 const void *buf,
				 size_t size)
{
	struct mock_memeq_matcher *matcher;

	matcher = test_kzalloc(test, sizeof(*matcher), GFP_KERNEL);
	if (!matcher)
		return NULL;

	matcher->matcher.match = match_memeq;
	matcher->expected = buf;
	matcher->size = size;

	return &matcher->matcher;
}

struct mock_streq_matcher {
	struct mock_param_matcher matcher;
	const char *expected;
};

static bool match_streq(struct mock_param_matcher *pmatcher,
			struct test_stream *stream,
			const void *pactual)
{
	struct mock_streq_matcher *matcher =
			container_of(pmatcher,
				     struct mock_streq_matcher,
				     matcher);
	const char *actual = CONVERT_TO_ACTUAL_TYPE(const char *, pactual);
	bool matches = !strcmp(actual, matcher->expected);

	if (matches)
		stream->add(stream, "%s == %s", actual, matcher->expected);
	else
		stream->add(stream, "%s != %s", actual, matcher->expected);

	return matches;
}

struct mock_param_matcher *streq(struct test *test, const char *str)
{
	struct mock_streq_matcher *matcher;

	matcher = test_kzalloc(test, sizeof(*matcher), GFP_KERNEL);
	if (!matcher)
		return NULL;

	matcher->matcher.match = match_streq;
	matcher->expected = str;

	return &matcher->matcher;
}

struct mock_str_contains_matcher {
	struct mock_param_matcher matcher;
	const char *needle;
};

static bool match_str_contains(struct mock_param_matcher *pmatcher,
			       struct test_stream *stream,
			       const void *phaystack)
{
	struct mock_str_contains_matcher *matcher =
		container_of(pmatcher,
			     struct mock_str_contains_matcher,
			     matcher);
	const char *haystack = CONVERT_TO_ACTUAL_TYPE(const char *, phaystack);
	bool matches = strstr(haystack, matcher->needle);

	if (matches)
		stream->add(stream,
			    "'%s' found in '%s'",
			    matcher->needle,
			    haystack);
	else
		stream->add(stream,
			    "'%s' not found in '%s'",
			    matcher->needle,
			    haystack);
	return matches;
}

struct mock_param_matcher *str_contains(struct test *test, const char *str)
{
	struct mock_str_contains_matcher *matcher;

	matcher = test_kzalloc(test, sizeof(*matcher), GFP_KERNEL);
	if (!matcher)
		return NULL;

	matcher->matcher.match = match_str_contains;
	matcher->needle = str;

	return &matcher->matcher;
}

struct mock_param_matcher *va_format_cmp(struct test *test,
					 struct mock_param_matcher *fmt_matcher,
					 struct mock_param_matcher *va_matcher)
{
	struct mock_struct_matcher_entry *entries;

	entries = test_kzalloc(test, sizeof(*entries) * 3, GFP_KERNEL);
	if (!entries)
		return NULL;

	INIT_MOCK_STRUCT_MATCHER_ENTRY(&entries[0],
				       struct va_format,
				       fmt,
				       fmt_matcher);
	INIT_MOCK_STRUCT_MATCHER_ENTRY(&entries[1],
				       struct va_format,
				       va,
				       va_matcher);
	INIT_MOCK_STRUCT_MATCHER_ENTRY_LAST(&entries[2]);

	return struct_cmp(test, "va_format", entries);
}

struct mock_struct_matcher {
	struct mock_param_matcher matcher;
	const char *struct_name;
	struct mock_struct_matcher_entry *entries;
};

static bool match_struct(struct mock_param_matcher *pmatcher,
			 struct test_stream *stream,
			 const void *pactual)
{
	struct mock_struct_matcher *matcher =
			container_of(pmatcher,
				     struct mock_struct_matcher,
				     matcher);
	struct mock_struct_matcher_entry *entry;
	const char *actual = CONVERT_TO_ACTUAL_TYPE(const char *, pactual);
	const char *member_ptr;
	bool matches = true, tmp;

	stream->add(stream, "struct %s {", matcher->struct_name);
	for (entry = matcher->entries; entry->matcher; entry++) {
		member_ptr = actual + entry->member_offset;
		tmp = entry->matcher->match(entry->matcher, stream, member_ptr);
		matches = matches && tmp;
		stream->add(stream, ", ");
	}
	stream->add(stream, "}");

	return matches;
}

struct mock_param_matcher *struct_cmp(struct test *test,
				      const char *struct_name,
				      struct mock_struct_matcher_entry *entries)
{
	struct mock_struct_matcher *matcher;

	matcher = test_kzalloc(test, sizeof(*matcher), GFP_KERNEL);
	if (!matcher)
		return NULL;

	matcher->matcher.match = match_struct;
	matcher->struct_name = struct_name;
	matcher->entries = entries;

	return &matcher->matcher;
}

#define DEFINE_RETURN_ACTION_STRUCT(type_name, type)			       \
		struct mock_##type_name##_action {			       \
			struct mock_action action;			       \
			type ret;					       \
		};

#define DEFINE_RETURN_ACTION_FUNC(type_name, type)			       \
		void *do_##type_name##_return(struct mock_action *paction,     \
					      const void **params,	       \
					      int len)			       \
		{							       \
			struct mock_##type_name##_action *action =	       \
					container_of(paction,		       \
						     struct mock_##type_name##_action,\
						     action);		       \
									       \
			return (void *) &action->ret;			       \
		}

#define DEFINE_RETURN_ACTION_FACTORY(type_name, type)			       \
		struct mock_action *type_name##_return(struct test *test,      \
						       type ret)	       \
		{							       \
			struct mock_##type_name##_action *action;	       \
									       \
			action = test_kmalloc(test,			       \
					      sizeof(*action),		       \
					      GFP_KERNEL);		       \
			if (!action)					       \
				return NULL;				       \
									       \
			action->action.do_action = do_##type_name##_return;    \
			action->ret = ret;				       \
									       \
			return &action->action;				       \
		}

#define DEFINE_RETURN_ACTION_WITH_TYPENAME(type_name, type)		       \
		DEFINE_RETURN_ACTION_STRUCT(type_name, type);		       \
		DEFINE_RETURN_ACTION_FUNC(type_name, type);		       \
		DEFINE_RETURN_ACTION_FACTORY(type_name, type);

#define DEFINE_RETURN_ACTION(type) \
		DEFINE_RETURN_ACTION_WITH_TYPENAME(type, type)

DEFINE_RETURN_ACTION(u8);
DEFINE_RETURN_ACTION(u16);
DEFINE_RETURN_ACTION(u32);
DEFINE_RETURN_ACTION(u64);
DEFINE_RETURN_ACTION(char);
DEFINE_RETURN_ACTION_WITH_TYPENAME(uchar, unsigned char);
DEFINE_RETURN_ACTION_WITH_TYPENAME(schar, signed char);
DEFINE_RETURN_ACTION(short);
DEFINE_RETURN_ACTION_WITH_TYPENAME(ushort, unsigned short);
DEFINE_RETURN_ACTION(int);
DEFINE_RETURN_ACTION_WITH_TYPENAME(uint, unsigned int);
DEFINE_RETURN_ACTION(long);
DEFINE_RETURN_ACTION_WITH_TYPENAME(ulong, unsigned long);
DEFINE_RETURN_ACTION_WITH_TYPENAME(longlong, long long);
DEFINE_RETURN_ACTION_WITH_TYPENAME(ulonglong, unsigned long long);
DEFINE_RETURN_ACTION_WITH_TYPENAME(ptr, void *);

