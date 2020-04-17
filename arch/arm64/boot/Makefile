#
# arch/arm64/boot/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 2012, ARM Ltd.
# Author: Will Deacon <will.deacon@arm.com>
#
# Based on the ia64 boot/Makefile.
#

include $(srctree)/arch/arm64/boot/dts/Makefile

OBJCOPYFLAGS_Image :=-O binary -R .note -R .note.gnu.build-id -R .comment -S

targets := Image Image.bz2 Image.gz Image.lz4 Image.lzma Image.lzo

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
DTB_OBJS := $(addprefix $(obj)/dts/,$(DTB_LIST))
else
DTB_OBJS := $(shell find $(obj)/dts/ -name \*.dtb)
endif
DTBO_OBJS := $(shell find -L $(obj)/dts/ -name \*.dtbo)

# Add RTIC DTB to the DTB list if RTIC MPGen is enabled
ifdef RTIC_MPGEN
DTB_OBJS += rtic_mp.dtb
endif

rtic_mp.dtb: vmlinux FORCE
	$(RTIC_MPGEN) --objcopy="${OBJCOPY}" --objdump="${OBJDUMP}" \
	--binpath="" --vmlinux="vmlinux" --config=${KCONFIG_CONFIG} \
	--cc="${CC}" --dts=rtic_mp.dts && \
	$(DTC) -O dtb -o rtic_mp.dtb -b 0 $(DTC_FLAGS) rtic_mp.dts

$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/Image.bz2: $(obj)/Image FORCE
	$(call if_changed,bzip2)

$(obj)/Image-dtb: $(obj)/Image $(DTB_OBJS) FORCE
	$(call if_changed,cat)

$(obj)/Image.gz: $(obj)/Image FORCE
	$(call if_changed,gzip)

$(obj)/Image.lz4: $(obj)/Image FORCE
	$(call if_changed,lz4)

$(obj)/Image.lzma: $(obj)/Image FORCE
	$(call if_changed,lzma)

$(obj)/Image.lzo: $(obj)/Image FORCE
	$(call if_changed,lzo)

$(obj)/Image.gz-dtb: $(obj)/Image.gz $(DTB_OBJS) FORCE
	$(call if_changed,cat)

$(obj)/Image.lz4-dtb: $(obj)/Image.lz4 $(DTB_OBJS) FORCE
	$(call if_changed,cat)

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
ifeq ($(CONFIG_BOARD_B1C1),y)
$(obj)/dtbo.img: $(obj)/dts/dtboimg.cfg $(DTBO_OBJS) FORCE
	$(call if_changed,mkdtimg,4096) # align dtbo.img to 4kB
else
$(obj)/dtbo.img: $(obj)/dts/dtboimg-sdm670.cfg $(DTBO_OBJS) FORCE
	$(call if_changed,mkdtimg,4096) # align dtbo.img to 4kB
endif
endif

install:
	$(CONFIG_SHELL) $(srctree)/$(src)/install.sh $(KERNELRELEASE) \
	$(obj)/Image System.map "$(INSTALL_PATH)"

zinstall:
	$(CONFIG_SHELL) $(srctree)/$(src)/install.sh $(KERNELRELEASE) \
	$(obj)/Image.gz System.map "$(INSTALL_PATH)"
