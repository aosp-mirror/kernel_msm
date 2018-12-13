ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws-overlay.dtbo \
	sm8150-coral-proto-overlay.dtbo \
	sm8150-coral-evt-overlay.dtbo \
	sm8150-coral-overlay.dtbo \
	sm8150-flame-proto-overlay.dtbo \
	sm8150-flame-notouch-overlay.dtbo \
	sm8150-flame-evt-overlay.dtbo \
	sm8150-flame-overlay.dtbo

sm8150-floral-jaws-overlay.dtbo-base	:= qcom-base/sm8150.dtb
sm8150-coral-proto-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-coral-evt-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-coral-overlay.dtbo-base		:= qcom-base/sm8150-v2.dtb
sm8150-flame-proto-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-notouch-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-evt-overlay.dtbo-base	:= qcom-base/sm8150-v2.dtb
sm8150-flame-overlay.dtbo-base		:= qcom-base/sm8150-v2.dtb
else
dtb-$(CONFIG_BOARD_FLORAL) += \
	sm8150-floral-jaws.dtb \
	sm8150-coral-proto.dtb \
	sm8150-coral-evt.dtb \
	sm8150-coral.dtb \
	sm8150-flame-proto.dtb \
	sm8150-flame-notouch.dtb \
	sm8150-flame-evt.dtb \
	sm8150-flame.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
