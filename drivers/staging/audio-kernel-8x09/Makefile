# auto-detect subdirs

# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(srctree)/drivers/staging/audio-kernel-8x09/include/uapi 

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/drivers/staging/audio-kernel-8x09/include/uapi \
                -I$(srctree)/drivers/staging/audio-kernel-8x09/include


obj-y += asoc/
obj-y += dsp/
obj-y += ipc/
obj-y += soc/
obj-m += asoc/codecs/msm_bg
