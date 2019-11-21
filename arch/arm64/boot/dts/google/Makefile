ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_BOARD_BRAMBLE) += \
	lito-bramble-dev1.0-overlay.dtbo \
	lito-bramble-dev1.1-overlay.dtbo \
	lito-bramble-proto-overlay.dtbo \
	lito-bramble-proto1_1-overlay.dtbo \
	lito-bramble-proto1_6-overlay.dtbo \
	lito-bramble-evt-overlay.dtbo


lito-bramble-dev1.0-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-dev1.1-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-proto-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-proto1_1-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-proto1_6-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-evt-overlay.dtbo-base := qcom-base/lito.dtb


dtbo-$(CONFIG_BOARD_BRAMBLE) += \
	lito-bramble_m-proto-overlay.dtbo


lito-bramble_m-proto-overlay.dtbo-base := qcom-base/lito.dtb


dtbo-$(CONFIG_BOARD_REDFIN) += \
	lito-redfin-dev1.0-overlay.dtbo \
	lito-redfin-proto-sop-overlay.dtbo \
	lito-redfin-proto-overlay.dtbo \
	lito-redfin-proto1_1-overlay.dtbo

lito-redfin-dev1.0-overlay.dtbo-base := qcom-base/lito.dtb
lito-redfin-proto-sop-overlay.dtbo-base := qcom-base/lito.dtb
lito-redfin-proto-overlay.dtbo-base := qcom-base/lito.dtb
lito-redfin-proto1_1-overlay.dtbo-base := qcom-base/lito.dtb

endif

always          := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb
