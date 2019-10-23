ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_BOARD_SUNFISH) += \
		sm7150-sunfish-dev-overlay.dtbo \
		sm7150-sunfish-evt1.0-overlay.dtbo \
		sm7150-sunfish-evt1.1-overlay.dtbo \
		sm7150-sunfish-dvt1.0-overlay.dtbo \
		sm7150-sunfish-pvt1.0-overlay.dtbo

sm7150-sunfish-dev-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-evt1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-evt1.1-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-dvt1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
sm7150-sunfish-pvt1.0-overlay.dtbo-base := qcom-base/sdmmagpie.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
