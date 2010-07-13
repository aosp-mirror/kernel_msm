  zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000

# override for Sapphire
  zreladdr-$(CONFIG_MACH_SAPPHIRE)		:= 0x02008000
params_phys-$(CONFIG_MACH_SAPPHIRE)		:= 0x02000100
initrd_phys-$(CONFIG_MACH_SAPPHIRE)		:= 0x02800000

# for now, override for QSD8x50
  zreladdr-$(CONFIG_ARCH_QSD8X50)		:= 0x20008000
params_phys-$(CONFIG_ARCH_QSD8X50)		:= 0x20000100
initrd_phys-$(CONFIG_ARCH_QSD8X50)		:= 0x21000000

  zreladdr-$(CONFIG_ARCH_MSM7X30)		:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X30)		:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X30)		:= 0x01200000
