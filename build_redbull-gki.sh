# SPDX-License-Identifier: GPL-2.0
if [[ "${BUILD_ABI:-0}" -eq 0 ]]; then
BUILD_BOOT_IMG=true BUILD_CONFIG=private/msm-google/build.config.gki.redbull \
  build/build.sh "$@"
else
BUILD_BOOT_IMG=true BUILD_CONFIG=private/msm-google/build.config.gki.redbull \
  build/build_abi.sh --update "$@"
fi
