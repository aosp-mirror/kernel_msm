#ifndef __MNH_DDR_H__
#define __MNH_DDR_H__

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>

#define MNH_DDR_NUM_CTL_REG	(558 + 1)
#define MNH_DDR_NUM_PHY_REG	(1100 + 1)
#define MNH_DDR_NUM_PI_REG	(191 + 1)

#define MNH_DDR_NUM_FSPS (4)
#define MNH_DDR_PHY_NUM_FSPS (MNH_DDR_NUM_FSPS - 1)

enum mnh_ddr_error {
	MNH_DDR_OK,
	MNH_DDR_PARAMETERS_INIT_FAILED,
	MNH_DDR_PLL_NOT_LOCKED,
	MNH_DDR_PLL_OP_FREQ_INVALID,
};

struct mnh_ddr_reg_bases {
	u32 ctl_base;
	u32 phy_base;
	u32 pi_base;
};

struct mnh_ddr_reg_config {
	u32 ctl[MNH_DDR_NUM_CTL_REG];
	u32 pi[MNH_DDR_NUM_PI_REG];
	u32 phy[MNH_DDR_NUM_PHY_REG];
};

struct mnh_ddr_state {
	struct mnh_ddr_reg_bases	bases;
	u32				fsps[MNH_DDR_NUM_FSPS];
	struct mnh_ddr_reg_config	*config;
};

struct mnh_ddr_internal_state {
	u32 ctl_base;
	u32 phy_base;
	u32 pi_base;
	u32 ctl[MNH_DDR_NUM_CTL_REG];
	u32 pi[MNH_DDR_NUM_PI_REG];
	u32 phy[MNH_DDR_PHY_NUM_FSPS][MNH_DDR_NUM_PHY_REG];
	u32 fsps[MNH_DDR_NUM_FSPS];
};

int mnh_ddr_po_init(struct device *dev);
int mnh_ddr_resume(struct device *dev, struct gpio_desc *iso_n);
int mnh_ddr_suspend(struct device *dev, struct gpio_desc *iso_n);

#endif /* __MNH_DDR_H__ */
