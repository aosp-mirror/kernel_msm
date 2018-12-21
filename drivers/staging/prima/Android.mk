# Android makefile for the WLAN Module

# Assume no targets will be supported
WLAN_CHIPSET :=

# Build/Package options for 8960 target
ifeq ($(TARGET_BOARD_PLATFORM),msm8960)
WLAN_CHIPSET := prima
WLAN_SELECT := CONFIG_PRIMA_WLAN=m
endif

# Build/Package options for 8916, 8974, 8226, 8610, 8909, 8952, 8937, 8953 targets
ifneq (,$(filter msm8916 msm8974 msm8226 msm8610 msm8909 msm8952 msm8937 msm8953 titanium,$(TARGET_BOARD_PLATFORM)))
WLAN_CHIPSET := pronto
WLAN_SELECT := CONFIG_PRONTO_WLAN=m
endif

# Build/Package only in case of supported target
ifneq ($(WLAN_CHIPSET),)

LOCAL_PATH := $(call my-dir)

ifeq ($(TARGET_SUPPORTS_WEARABLES),true)
ifneq ($(findstring device,$(LOCAL_PATH)),)
    WLAN_DLKM := 1
else
ifneq ($(findstring vendor,$(LOCAL_PATH)),)
    WLAN_DLKM := 1
else
    WLAN_DLKM := 0
endif # findstring device
endif
else
ifneq ($(findstring vendor,$(LOCAL_PATH)),)
    WLAN_DLKM := 1
else
    WLAN_DLKM := 0
endif # findstring vendor
endif # TARGET_SUPPORTS_WEARABLES

# This makefile is only for DLKM
ifeq ($(WLAN_DLKM),1)

# Determine if we are Proprietary or Open Source
ifneq ($(findstring opensource,$(LOCAL_PATH)),)
    WLAN_PROPRIETARY := 0
else
    WLAN_PROPRIETARY := 1
endif

ifeq ($(WLAN_PROPRIETARY),1)
    WLAN_BLD_DIR := vendor/qcom/proprietary/wlan
else
ifneq ($(TARGET_SUPPORTS_WEARABLES),true)
ifneq ($(ANDROID_BUILD_TOP),)
    WLAN_BLD_DIR := $(ANDROID_BUILD_TOP)/vendor/qcom/opensource/wlan
else
    WLAN_BLD_DIR := vendor/qcom/opensource/wlan
endif
else
ifneq ($(ANDROID_BUILD_TOP),)
    WLAN_BLD_DIR := $(ANDROID_BUILD_TOP)/device/qcom/msm8909w/opensource/wlan
else
    WLAN_BLD_DIR := device/qcom/msm8909w/opensource/wlan
endif
endif
endif

# DLKM_DIR was moved for JELLY_BEAN (PLATFORM_SDK 16)
ifeq (1,$(filter 1,$(shell echo "$$(( $(PLATFORM_SDK_VERSION) >= 16 ))" )))
ifneq ($(TARGET_SUPPORTS_WEARABLES),true)
       DLKM_DIR := $(TOP)/device/qcom/common/dlkm
else
       DLKM_DIR := $(BOARD_DLKM_DIR)
endif
else
       DLKM_DIR := build/dlkm
endif

# Copy WCNSS_cfg.dat file from firmware_bin/ folder to target out directory.
ifeq ($(WLAN_PROPRIETARY),0)

$(shell mkdir -p $(TARGET_OUT_ETC)/firmware/wlan/prima)
$(shell rm -f $(TARGET_OUT_ETC)/firmware/wlan/prima/WCNSS_cfg.dat)
$(shell cp $(LOCAL_PATH)/firmware_bin/WCNSS_cfg.dat $(TARGET_OUT_ETC)/firmware/wlan/prima)

else

include $(CLEAR_VARS)
LOCAL_MODULE       := WCNSS_qcom_wlan_nv.bin
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH  := $(PRODUCT_OUT)/persist
LOCAL_SRC_FILES    := firmware_bin/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := WCNSS_cfg.dat
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH  := $(TARGET_OUT_ETC)/firmware/wlan/prima
LOCAL_SRC_FILES    := firmware_bin/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := WCNSS_qcom_cfg.ini
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH  := $(PRODUCT_OUT)/persist
LOCAL_SRC_FILES    := firmware_bin/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

endif

ifeq ($(TARGET_KERNEL_VERSION),)
$(info "WLAN: TARGET_KERNEL_VERSION is not defined, assuming default")
TARGET_KERNEL_SOURCE := kernel
KERNEL_TO_BUILD_ROOT_OFFSET := ../
endif

ifeq ($(KERNEL_TO_BUILD_ROOT_OFFSET),)
$(info "WLAN: KERNEL_TO_BUILD_ROOT_OFFSET is not defined, assuming default")
KERNEL_TO_BUILD_ROOT_OFFSET := ../
endif

# Build wlan.ko as either prima_wlan.ko or pronto_wlan.ko
###########################################################

# This is set once per LOCAL_PATH, not per (kernel) module

ifneq ($(ANDROID_BUILD_TOP),)
KBUILD_OPTIONS := WLAN_ROOT=$(WLAN_BLD_DIR)/prima
endif

ifeq ($(KBUILD_OPTIONS),)
KBUILD_OPTIONS := WLAN_ROOT=$(KERNEL_TO_BUILD_ROOT_OFFSET)$(WLAN_BLD_DIR)/prima
endif

# We are actually building wlan.ko here, as per the
# requirement we are specifying <chipset>_wlan.ko as LOCAL_MODULE.
# This means we need to rename the module to <chipset>_wlan.ko
# after wlan.ko is built.
KBUILD_OPTIONS += MODNAME=wlan
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
KBUILD_OPTIONS += $(WLAN_SELECT)


ifeq ($(KERNEL_TO_BUILD_ROOT_OFFSET),../../)
VERSION=$(shell grep -w "VERSION =" $(TOP)/kernel/msm-$(TARGET_KERNEL_VERSION)/Makefile | sed 's/^VERSION = //' )
PATCHLEVEL=$(shell grep -w "PATCHLEVEL =" $(TOP)/kernel/msm-$(TARGET_KERNEL_VERSION)/Makefile | sed 's/^PATCHLEVEL = //' )
else
VERSION=$(shell grep -w "VERSION =" $(TOP)/kernel/Makefile | sed 's/^VERSION = //' )
PATCHLEVEL=$(shell grep -w "PATCHLEVEL =" $(TOP)/kernel/Makefile | sed 's/^PATCHLEVEL = //' )
endif

include $(CLEAR_VARS)
LOCAL_MODULE              := $(WLAN_CHIPSET)_wlan.ko
LOCAL_MODULE_KBUILD_NAME  := wlan.ko
LOCAL_MODULE_TAGS         := debug
LOCAL_MODULE_DEBUG_ENABLE := true
ifeq ($(PRODUCT_VENDOR_MOVE_ENABLED), true)
    ifeq ($(WIFI_DRIVER_INSTALL_TO_KERNEL_OUT),true)
        LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
    else
        LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/$(WLAN_CHIPSET)
    endif
else
    LOCAL_MODULE_PATH         := $(TARGET_OUT)/lib/modules/$(WLAN_CHIPSET)
endif # PRODUCT_VENDOR_MOVE_ENABLED
include $(DLKM_DIR)/AndroidKernelModule.mk
###########################################################

#Create symbolic link
ifeq ($(PRODUCT_VENDOR_MOVE_ENABLED), true)
$(shell mkdir -p $(TARGET_OUT_VENDOR)/lib/modules; \
        ln -sf /$(TARGET_COPY_OUT_VENDOR)/lib/modules/$(WLAN_CHIPSET)/$(WLAN_CHIPSET)_wlan.ko \
               $(TARGET_OUT_VENDOR)/lib/modules/wlan.ko)
else
$(shell mkdir -p $(TARGET_OUT)/lib/modules; \
        ln -sf /system/lib/modules/$(WLAN_CHIPSET)/$(WLAN_CHIPSET)_wlan.ko \
               $(TARGET_OUT)/lib/modules/wlan.ko)
endif # PRODUCT_VENDOR_MOVE_ENABLED
endif # DLKM check

endif # supported target check
