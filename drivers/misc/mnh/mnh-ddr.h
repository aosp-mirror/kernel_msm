#ifndef __MNH_DDR_H__
#define __MNH_DDR_H__

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>

#include "mnh-pll.h"

#define MNH_DDR_NUM_CTL_REG	(558 + 1)
#define MNH_DDR_NUM_PHY_REG	(1100 + 1)
#define MNH_DDR_NUM_PI_REG	(191 + 1)

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

#define MNH_DDR_REG_CONFIG_ELEMS 2
#define MNH_DDR_REG_CONFIG_ELEM_VAL 0
#define MNH_DDR_REG_CONFIG_ELEM_FLAGS 1
/* [i][0]:
 * i-th register value
 * not keeping explicit address for registers
 * as it is derived from the mnh_ddr_reg_bases + element index.
 * [i][1]:
 * i-th register bit flags for to mark skipping, for instance.
 * flags to be determined
 */
#define MNH_DDR_REG_CONFIG_FLAG_SKIP_PUSH (0x00000001)
#define MNH_DDR_REG_CONFIG_FLAG_SKIP_PULL (0x00000002)
#define MNH_DDR_REG_CONFIG_FLAG_SAVE_REST (0x00000004)
struct mnh_ddr_reg_config {
	u32 ctl[MNH_DDR_NUM_CTL_REG][MNH_DDR_REG_CONFIG_ELEMS];
	u32 phy[MNH_DDR_NUM_PHY_REG][MNH_DDR_REG_CONFIG_ELEMS];
	u32 pi[MNH_DDR_NUM_PI_REG][MNH_DDR_REG_CONFIG_ELEMS];
};

struct mnh_ddr_reg {
	u32 addr;
	u32 value;
};

struct mnh_ddr_state {
	struct mnh_ddr_reg_bases	bases;
	u32				fsps[MNH_NUM_FSPS];
	struct mnh_ddr_reg_config	*configs[MNH_NUM_FSPS];
};

int mnh_ddr_po_init(struct device *dev, struct mnh_ddr_state *state);


#endif /* __MNH_DDR_H__ */
