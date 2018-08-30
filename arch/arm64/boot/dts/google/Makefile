ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws-overlay.dtbo \
	sm8150-floral-coral-overlay.dtbo \
	sm8150-floral-flame-overlay.dtbo

sm8150-floral-jaws-overlay.dtbo-base	:= qcom-base/sm8150.dtb
sm8150-floral-coral-overlay.dtbo-base	 := qcom-base/sm8150-v2.dtb
sm8150-floral-flame-overlay.dtbo-base	 := qcom-base/sm8150-v2.dtb
else
dtb-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws.dtb \
	sm8150-floral-coral.dtb \
	sm8150-floral-flame.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
