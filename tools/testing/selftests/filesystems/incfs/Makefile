# SPDX-License-Identifier: GPL-2.0
<<<<<<< HEAD
CFLAGS += -D_FILE_OFFSET_BITS=64 -Wall -Werror -I../.. -I../../../../..
LDLIBS := -llz4 -lzstd -lcrypto -lpthread
TEST_GEN_PROGS := incfs_test incfs_stress incfs_perf

include ../../lib.mk

# Put after include ../../lib.mk since that changes $(TEST_GEN_PROGS)
# Otherwise you get multiple targets, this becomes the default, and it's a mess
EXTRA_SOURCES := utils.c
$(TEST_GEN_PROGS) : $(EXTRA_SOURCES)
=======
CFLAGS += -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -Wall
CFLAGS += -I../.. -I../../../../..

LDLIBS := -llz4 -lcrypto
EXTRA_SOURCES := utils.c
TEST_GEN_PROGS := incfs_test

$(TEST_GEN_PROGS): $(EXTRA_SOURCES)

include ../../lib.mk
>>>>>>> partner/qcom-msm-4.14
