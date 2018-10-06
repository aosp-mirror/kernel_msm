ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws-overlay.dtbo \
	sm8150-coral-overlay.dtbo \
	sm8150-flame-overlay.dtbo \
	sm8150-flame-notouch-overlay.dtbo

sm8150-floral-jaws-overlay.dtbo-base	:= qcom-base/sm8150.dtb
sm8150-coral-overlay.dtbo-base		:= qcom-base/sm8150-v2.dtb
sm8150-flame-overlay.dtbo-base		:= qcom-base/sm8150-v2.dtb
sm8150-flame-notouch-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
else
dtb-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws.dtb \
	sm8150-coral.dtb \
	sm8150-flame.dtb \
	sm8150-flame-notouch.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
