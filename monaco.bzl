load(":target_variants.bzl", "la_variants")
load(":msm_kernel_la.bzl", "define_msm_la")
load(":image_opts.bzl", "boot_image_opts")

target_name = "monaco"

def define_monaco():
    _monaco_in_tree_modules = [
        # keep sorted
        "drivers/clk/qcom/clk-qcom.ko",
        "drivers/cpufreq/cpufreq_ondemand.ko",
        "drivers/cpufreq/cpufreq_userspace.ko",
        "drivers/cpufreq/qcom-cpufreq-hw.ko",
        "drivers/dma-buf/heaps/qcom_dma_heaps.ko",
        "drivers/dma/qcom/bam_dma.ko",
        "drivers/firmware/qcom-scm.ko",
        "drivers/hwspinlock/qcom_hwspinlock.ko",
        "drivers/i2c/busses/i2c-qcom-geni.ko",
        "drivers/iio/adc/qcom-spmi-adc5.ko",
        "drivers/iio/adc/qcom-vadc-common.ko",
        "drivers/iommu/arm/arm-smmu/arm_smmu.ko",
        "drivers/iommu/msm_dma_iommu_mapping.ko",
        "drivers/irqchip/qcom-mpm.ko",
        "drivers/mailbox/qcom-apcs-ipc-mailbox.ko",
        "drivers/mfd/qcom-spmi-pmic.ko",
        "drivers/mmc/host/cqhci.ko",
        "drivers/mmc/host/sdhci-msm.ko",
        "drivers/nvmem/nvmem_qcom-spmi-sdam.ko",
        "drivers/nvmem/nvmem_qfprom.ko",
        "drivers/phy/phy-xgene.ko",
        "drivers/pinctrl/qcom/pinctrl-monaco.ko",
        "drivers/pinctrl/qcom/pinctrl-msm.ko",
        "drivers/remoteproc/qcom_common.ko",
        "drivers/remoteproc/qcom_pil_info.ko",
        "drivers/remoteproc/qcom_q6v5.ko",
        "drivers/remoteproc/qcom_q6v5_pas.ko",
        "drivers/remoteproc/qcom_sysmon.ko",
        "drivers/rpmsg/qcom_glink.ko",
        "drivers/rpmsg/qcom_glink_rpm.ko",
        "drivers/rpmsg/qcom_glink_smem.ko",
        "drivers/rpmsg/qcom_smd.ko",
        "drivers/rpmsg/rpm-smd.ko",
        "drivers/rtc/rtc-pm8xxx.ko",
        "drivers/slimbus/slim-qcom-ngd-ctrl.ko",
        "drivers/slimbus/slimbus.ko",
        "drivers/soc/qcom/mdt_loader.ko",
        "drivers/soc/qcom/mem-hooks.ko",
        "drivers/soc/qcom/mem_buf/mem_buf.ko",
        "drivers/soc/qcom/mem_buf/mem_buf_dev.ko",
        "drivers/soc/qcom/panel_event_notifier.ko",
        "drivers/soc/qcom/pdr_interface.ko",
        "drivers/soc/qcom/qcom_aoss.ko",
        "drivers/soc/qcom/qcom_soc_wdt.ko",
        "drivers/soc/qcom/qcom_wdt_core.ko",
        "drivers/soc/qcom/qmi_helpers.ko",
        "drivers/soc/qcom/rpm-smd-debug.ko",
        "drivers/soc/qcom/secure_buffer.ko",
        "drivers/soc/qcom/smem.ko",
        "drivers/soc/qcom/smp2p.ko",
        "drivers/soc/qcom/socinfo.ko",
        "drivers/spi/spi-geni-qcom.ko",
        "drivers/thermal/qcom/qcom-spmi-temp-alarm.ko",
        "drivers/thermal/qcom/qcom_tsens.ko",
        "drivers/usb/misc/ehset.ko",
        "drivers/usb/phy/phy-generic.ko",
        "kernel/msm_sysstats.ko",
        "net/qrtr/qrtr.ko",
        "net/qrtr/qrtr-smd.ko",
        "net/wireless/cfg80211.ko",
    ]

    _monaco_consolidate_in_tree_modules = _monaco_in_tree_modules + [
        # keep sorted
        "drivers/misc/lkdtm/lkdtm.ko",
        "kernel/locking/locktorture.ko",
        "kernel/rcu/rcutorture.ko",
        "kernel/torture.ko",
        "lib/atomic64_test.ko",
        "lib/test_user_copy.ko",
    ]

    kernel_vendor_cmdline_extras = [
        # do not sort
        "console=ttyMSM0,115200n8",
        "qcom_geni_serial.con_enabled=1",
        "bootconfig",
    ]

    for variant in la_variants:
        board_kernel_cmdline_extras = []
        board_bootconfig_extras = []

        if variant == "consolidate":
            mod_list = _monaco_consolidate_in_tree_modules
        else:
            mod_list = _monaco_in_tree_modules
            board_kernel_cmdline_extras += ["nosoftlockup"]
            kernel_vendor_cmdline_extras += ["nosoftlockup"]
            board_bootconfig_extras += ["androidboot.console=0"]

        define_msm_la(
            msm_target = target_name,
            variant = variant,
            in_tree_module_list = mod_list,
            boot_image_opts = boot_image_opts(
                earlycon_addr = "qcom_geni,0x04a98000",
                kernel_vendor_cmdline_extras = kernel_vendor_cmdline_extras,
                board_kernel_cmdline_extras = board_kernel_cmdline_extras,
                board_bootconfig_extras = board_bootconfig_extras,
            ),
        )
