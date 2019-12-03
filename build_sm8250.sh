# SPDX-License-Identifier: GPL-2.0
BUILD_BOOT_IMG=true BUILD_CONFIG=private/msm-google/build.config.gki.sm8250 \
  build/build.sh CONFIG_BUILD_ARM64_DT_OVERLAY=y "$@"
