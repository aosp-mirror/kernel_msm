#ifndef __LINUX_PLATFORM_DATA_NANOHUB_H
#define __LINUX_PLATFORM_DATA_NANOHUB_H

#include <linux/types.h>

struct nanohub_flash_bank {
	int bank;
	u32 address;
	size_t length;
};

struct nanohub_platform_data {
	u32 wakeup_gpio;
	u32 nreset_gpio;
	bool nreset_polarity;
	u32 boot0_gpio;
	u32 int_gpio;
	u32 mode1_gpio;
	u32 mode2_gpio;
	u32 mode3_gpio;
	u32 mode4_gpio;
	u32 irq1_gpio;
	u32 irq2_gpio;
	u32 irq3_gpio;
	u32 spi_cs_gpio;
	u32 bl_addr;
	u32 num_flash_banks;
	struct nanohub_flash_bank *flash_banks;
	u32 num_shared_flash_banks;
	struct nanohub_flash_bank *shared_flash_banks;
	u32 custom_flash_addr;
	u32 custom_flash_len;
};

#endif /* __LINUX_PLATFORM_DATA_NANOHUB_H */
