# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for msm-bus driver specific files
#
obj-$(CONFIG_QCOM_BUS_SCALING) +=  msm_bus.o
msm_bus-objs +=  msm_bus_core.o
msm_bus-objs += msm_bus_client_api.o
msm_bus-$(CONFIG_OF) += msm_bus_of.o
msm_bus-$(CONFIG_MSM_RPM_SMD) += msm_bus_rpm_smd.o

obj-$(CONFIG_QCOM_BUS_CONFIG_RPMH) += msm_bus_rpmh.o
msm_bus_rpmh-objs += msm_bus_fabric_rpmh.o msm_bus_arb_rpmh.o msm_bus_rules.o \
	msm_bus_bimc_rpmh.o msm_bus_noc_rpmh.o
msm_bus_rpmh-$(CONFIG_OF) += msm_bus_of_rpmh.o

ifdef CONFIG_DEBUG_FS
obj-$(CONFIG_QCOM_BUS_CONFIG_RPMH) += msm_bus_dbg_rpmh.o
endif
