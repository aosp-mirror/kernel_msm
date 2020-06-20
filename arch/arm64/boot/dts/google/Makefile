ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-bramble += \
	lito-bramble-dev1.0-overlay.dtbo \
	lito-bramble-dev1.1-overlay.dtbo \
	lito-bramble-proto-overlay.dtbo \
	lito-bramble-proto1_1-overlay.dtbo \
	lito-v2-bramble-proto1_2-overlay.dtbo \
	lito-bramble-proto1_6-overlay.dtbo \
	lito-v2-bramble-evt-overlay.dtbo \
	lito-v2-bramble-evt1_8-overlay.dtbo \
	lito-v2-bramble-evt1_9-overlay.dtbo \
	lito-v2-bramble-dvt-overlay.dtbo \
	lito-v2-bramble-pvt-overlay.dtbo


lito-bramble-dev1.0-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-dev1.1-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-proto-overlay.dtbo-base := qcom-base/lito.dtb
lito-bramble-proto1_1-overlay.dtbo-base := qcom-base/lito.dtb
lito-v2-bramble-proto1_2-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-bramble-proto1_6-overlay.dtbo-base := qcom-base/lito.dtb
lito-v2-bramble-evt-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble-evt1_8-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble-evt1_9-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble-dvt-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble-pvt-overlay.dtbo-base := qcom-base/lito-v2.dtb


dtbo-$(CONFIG_BOARD_BRAMBLE) += \
	lito-v2-bramble_m-dev1_1-overlay.dtbo \
	lito-v2-bramble_m-dev1_2-overlay.dtbo \
	lito-v2-bramble_m-evt-overlay.dtbo \
	lito-v2-bramble_m-evt1_1-overlay.dtbo \
	lito-v2-bramble_m-dvt-overlay.dtbo \
	lito-v2-bramble_m-pvt-overlay.dtbo


lito-v2-bramble_m-dev1_1-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble_m-dev1_2-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble_m-evt-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble_m-evt1_1-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble_m-dvt-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-bramble_m-pvt-overlay.dtbo-base := qcom-base/lito-v2.dtb


dtbo-redfin += \
	lito-redfin-dev1.0-overlay.dtbo \
	lito-redfin-proto-sop-overlay.dtbo \
	lito-redfin-proto-overlay.dtbo \
	lito-redfin-proto1_1-overlay.dtbo \
	lito-v2-redfin-proto1_1-overlay.dtbo \
	lito-v2-redfin-evt-overlay.dtbo \
	lito-v2-redfin-dvt-overlay.dtbo \
	lito-v2-redfin-pvt-overlay.dtbo

lito-redfin-dev1.0-overlay.dtbo-base := qcom-base/lito.dtb
lito-redfin-proto-sop-overlay.dtbo-base := qcom-base/lito.dtb
lito-redfin-proto-overlay.dtbo-base := qcom-base/lito.dtb
lito-redfin-proto1_1-overlay.dtbo-base := qcom-base/lito.dtb
lito-v2-redfin-proto1_1-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-redfin-evt-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-redfin-dvt-overlay.dtbo-base := qcom-base/lito-v2.dtb
lito-v2-redfin-pvt-overlay.dtbo-base := qcom-base/lito-v2.dtb

endif

always          := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb
