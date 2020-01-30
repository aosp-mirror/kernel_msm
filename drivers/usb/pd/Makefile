# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for USB Power Delivery drivers
#

obj-$(CONFIG_USB_PD_POLICY)	+= qpnp_pdphy.o
obj-$(CONFIG_USB_PD_ENGINE)	+= qpnp_pdphy.o
obj-$(CONFIG_QPNP_USB_PDPHY)	+= qpnp_pdphy.o
qpnp_pdphy-$(CONFIG_USB_PD_POLICY)	+= policy_engine.o
qpnp_pdphy-$(CONFIG_USB_PD_ENGINE)	+= pd_engine.o
qpnp_pdphy-$(CONFIG_QPNP_USB_PDPHY)	+= qpnp-pdphy.o
