# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_QPNP_PMIC_VOTER) += pmic-voter.o
obj-$(CONFIG_QPNP_BATTERY) += qpnp-battery.o
qpnp-battery-objs += battery.o
obj-$(CONFIG_QPNP_SMB5)			+= qpnp-smb5-charger.o
qpnp-smb5-charger-objs += step-chg-jeita.o qpnp-smb5.o smb5-lib.o storm-watch.o schgm-flash.o
obj-$(CONFIG_SMB1390_CHARGE_PUMP_PSY)	+= smb1390-charger-psy.o
obj-$(CONFIG_SMB1355_SLAVE_CHARGER)	+= smb1355-charger.o
obj-$(CONFIG_QPNP_QNOVO5)		+= qpnp-qnovo5.o
obj-$(CONFIG_QPNP_FG_GEN4)		+= qpnp-fg4.o
qpnp-fg4-objs += qpnp-fg-gen4.o fg-memif.o fg-util.o fg-alg.o
obj-$(CONFIG_QPNP_QG)			+= qpnp-qgauge.o
qpnp-qgauge-objs += qpnp-qg.o  qg-util.o qg-soc.o qg-sdam.o qg-battery-profile.o qg-profile-lib.o fg-alg.o
obj-$(CONFIG_HL6111R)			+= hl6111r.o
obj-$(CONFIG_SMB1398_CHARGER)		+= smb1398-charger.o pmic-voter.o
obj-$(CONFIG_QPNP_SMBLITE)		+= step-chg-jeita.o battery.o qpnp-smblite.o smblite-lib.o pmic-voter.o storm-watch.o schgm-flashlite.o
