ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_BOARD_BRAMBLE) += \
	lito-bramble-overlay.dtbo

lito-bramble-overlay.dtbo-base := qcom-base/lito.dtb
endif

always          := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb
