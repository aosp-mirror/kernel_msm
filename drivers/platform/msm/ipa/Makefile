# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_IPA3) += ipa_v3/ ipa_clients/
obj-$(CONFIG_IPA_UT) += test/

obj-$(CONFIG_IPA3) += ipa3.o
ipa3-objs += ipa_api.o ipa_rm.o ipa_rm_dependency_graph.o ipa_rm_peers_list.o \
	ipa_rm_resource.o ipa_rm_inactivity_timer.o

ipa3-objs += $(patsubst %,ipa_v3/%,\
	ipa.o ipa_debugfs.o ipa_hdr.o ipa_flt.o ipa_rt.o ipa_dp.o ipa_client.o \
	ipa_utils.o ipa_nat.o ipa_intf.o teth_bridge.o ipa_interrupts.o ipa_uc.o \
	ipa_uc_wdi.o ipa_dma.o ipa_uc_mhi.o ipa_mhi.o ipa_uc_ntn.o ipa_hw_stats.o \
	ipa_pm.o ipa_wdi3_i.o ipa_odl.o ipa_wigig_i.o)

ipa3-objs += $(patsubst %,ipa_v3/ipahal/%,\
	ipahal.o ipahal_reg.o ipahal_fltrt.o ipahal_hw_stats.o ipahal_nat.o)

ipa3-objs += $(patsubst %,ipa_clients/%,\
	ipa_usb.o odu_bridge.o ipa_mhi_client.o ipa_uc_offload.o ipa_wdi3.o ipa_gsb.o \
	ipa_wigig.o)

ipa3-$(CONFIG_RNDIS_IPA) += ipa_clients/rndis_ipa.o

ipa3-$(CONFIG_IPA_EMULATION) += ipa_v3/ipa_dt_replacement.o

ipa3-$(CONFIG_IPA3_MHI_PRIME_MANAGER) += ipa_v3/ipa_mpm.o

ipa3-$(CONFIG_RMNET_IPA3) += $(patsubst %,ipa_v3/%,\
	rmnet_ipa.o ipa_qmi_service_v01.o ipa_qmi_service.o rmnet_ipa_fd_ioctl.o)
ccflags-$(CONFIG_RMNET_IPA3) += -Idrivers/platform/msm/ipa/ipa_v3

ipa3-$(CONFIG_IPA3_REGDUMP) += ipa_v3/dump/ipa_reg_dump.o
ccflags-$(CONFIG_IPA3_REGDUMP) += -Idrivers/platform/msm/ipa/ipa_v3/dump
ccflags-$(CONFIG_IPA3_REGDUMP_IPA_4_5) += -Idrivers/platform/msm/ipa/ipa_v3/dump/ipa4.5
