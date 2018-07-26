#
#  Makefile for DS device drivers, currently Apex or Knoll.
#  Note that (for now) building the modules out of tree, requires a tiny bit
#  of hand crafting, since things like kernel path, HASH and sign keys aren't
#  set automatically.

obj-$(CONFIG_DS_FRAMEWORK) += ds.o
obj-$(CONFIG_DWN_DRIVER) += dwn.o

PWD := $(shell pwd)

# Set this based on your kernel config
MODULE_HASH := sha1
DWN_VARIANT := oscar_platform
ccflags-y += -DOSCAR_AIRBRUSH

ds-objs	:= ds_generic.o ds_interrupt.o ds_page_table.o ds_sysfs.o ds_ioctl.o
ifeq ($(DWN_VARIANT),)
dwn-objs	:= apex_driver.o
else
# Allows for in kernel compiling in x86 kernel
dwn-objs	:= $(DWN_VARIANT)_driver.o
endif

#default:
#	$(MAKE) CFLAGS='-s' ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules
#ifeq ($(SIGN_MODULE),yes)
#	$(KERNEL_SCRIPTS_DIR)/sign-file $(MODULE_HASH) $(KERNEL_DIR)/signing_key.priv $(KERNEL_DIR)/signing_key.x509 ds.ko
#	$(KERNEL_SCRIPTS_DIR)/sign-file $(MODULE_HASH) $(KERNEL_DIR)/signing_key.priv $(KERNEL_DIR)/signing_key.x509 dwn.ko
#endif

clean:
	@rm -rf *.o .*.cmd .tmp_versions/ Module.symvers *.ko *.mod.c *.mod.o modules.order
