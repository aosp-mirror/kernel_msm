# SPDX-License-Identifier: GPL-2.0
BUILD_SCRIPT="build/build.sh"
CONFIG="private/msm-google/build.config.gki.redbull"

if [[ "${BUILD_ABI:-0}" -eq 1 ]]; then
	BUILD_SCRIPT="build/build_abi.sh --update"
	CONFIG="private/msm-google/build.config.redbull.vintf"
fi

BUILD_BOOT_IMG=true \
	ENABLE_THINLTO=1 \
	BUILD_CONFIG=${CONFIG} \
	${BUILD_SCRIPT} "$@"
