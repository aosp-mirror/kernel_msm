
dtb-$(CONFIG_ARCH_SDXPOORWILLS) += sdxpoorwills-rumi.dtb \
	sdxpoorwills-cdp.dtb \
	sdxpoorwills-mtp.dtb \
	sdxpoorwills-atp.dtb \
	sdxpoorwills-ttp.dtb \
	sdxpoorwills-cdp-256.dtb \
	sdxpoorwills-mtp-256.dtb \
	sdxpoorwills-dualwifi-cdp.dtb \
	sdxpoorwills-dualwifi-mtp.dtb \
	sdxpoorwills-pcie-ep-cdp-256.dtb \
	sdxpoorwills-pcie-ep-mtp-256.dtb \
	sdxpoorwills-pcie-ep-cdp.dtb \
	sdxpoorwills-pcie-ep-mtp.dtb \
	sdxpoorwills-v2-mtp.dtb \
	sdxpoorwills-v2-cdp.dtb

dtb-$(CONFIG_ARCH_MDM9650) += mdm9650-nand-mtp.dtb \
	mdm9650-ttp.dtb \
	mdm9650-v1.1-nand-mtp.dtb \
	mdm9650-v1.1-nand-cv2x.dtb

dtb-$(CONFIG_ARCH_MDM9607) += mdm9607-mtp.dtb \
	mdm9607-ttp.dtb

targets += dtbs

8x09_dtb = msm8909w-darter
ifeq ($(CONFIG_BUILD_TRIGGERFISH),y)
	8x09_dtb = msm8909w-triggerfish
endif
include $(srctree)/arch/arm/boot/dts//8x09/$(8x09_dtb)/Makefile
$(obj)/%.dtb: $(src)/../../../../arm/boot/dts/8x09/$(8x09_dtb)/%.dts FORCE
	$(call if_changed_dep,dtc)
	@echo "$< $@"
dtbs: $(addprefix $(obj)/,$(dtb-y))
clean-files := *.dtb
