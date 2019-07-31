ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_BOARD_SUNFISH) += \
		sm6150-sunfish-overlay.dtbo

sm6150-sunfish-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
