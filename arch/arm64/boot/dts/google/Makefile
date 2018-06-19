ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws-overlay.dtbo

sm8150-floral-jaws-overlay.dtbo-base	:= qcom-base/sm8150.dtb qcom-base/sm8150-v2.dtb
else
dtb-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
