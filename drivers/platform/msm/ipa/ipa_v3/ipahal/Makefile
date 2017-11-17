obj-$(CONFIG_IPA3) += ipa_hal.o

ipa_hal-y := ipahal.o ipahal_reg.o ipahal_fltrt.o ipahal_hw_stats.o ipahal_nat.o

# TODO: remove me b/62058353
subdir-ccflags-y += \
	-Wno-shift-overflow \
