ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_BOARD_BRAMBLE) += \
	lito-bramble-dev1.0-overlay.dtbo

lito-bramble-dev1.0-overlay.dtbo-base := qcom-base/lito.dtb
endif

always          := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb
