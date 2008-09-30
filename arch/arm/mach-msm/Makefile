obj-y += proc_comm.o
obj-y += io.o idle.o timer.o dma.o
obj-y += vreg.o
obj-y += acpuclock-arm11.o
obj-y += clock.o clock-pcom.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o

ifdef CONFIG_MSM_VIC
obj-y += irq-vic.o
else
obj-y += irq.o
endif

obj-$(CONFIG_ARCH_QSD8X50) += sirc.o
obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_MSM_CPU_FREQ_SCREEN) += cpufreq.o

obj-$(CONFIG_MACH_TROUT) += board-trout.o devices-msm7x00.o
obj-$(CONFIG_MACH_TROUT) += board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o
obj-$(CONFIG_MACH_TROUT) += htc_akm_cal.o htc_wifi_nvs.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o devices-msm7x00.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut-keypad.o
obj-$(CONFIG_ARCH_MSM7X30) += board-msm7x30.o devices-msm7x30.o
obj-$(CONFIG_ARCH_QSD8X50) += board-qsd8x50.o devices-qsd8x50.o

