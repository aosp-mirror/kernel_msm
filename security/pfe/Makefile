#
# Makefile for the MSM specific security device drivers.
#

ccflags-y += -Isecurity/selinux -Isecurity/selinux/include
ccflags-y += -Ifs/crypto

obj-$(CONFIG_PFK) += pfk.o pfk_kc.o pfk_ice.o pfk_fscrypt.o

# TODO: remove me b/62058353
subdir-ccflags-y += \
	-Wno-ignored-attributes
