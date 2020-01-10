// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm ICE (Inline Cryptographic Engine) support.
 *
 * Copyright (c) 2014-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019 Google LLC
 */

#include <linux/platform_device.h>
#include <linux/qcom_scm.h>

#include "ufshcd-crypto.h"
#include "ufs-qcom.h"

#define AES_256_XTS_KEY_SIZE			64

/* QCOM ICE registers */

#define QCOM_ICE_REGS_CONTROL			0x0000
#define QCOM_ICE_REGS_RESET			0x0004
#define QCOM_ICE_REGS_VERSION			0x0008
#define QCOM_ICE_REGS_FUSE_SETTING		0x0010
#define QCOM_ICE_REGS_PARAMETERS_1		0x0014
#define QCOM_ICE_REGS_PARAMETERS_2		0x0018
#define QCOM_ICE_REGS_PARAMETERS_3		0x001C
#define QCOM_ICE_REGS_PARAMETERS_4		0x0020
#define QCOM_ICE_REGS_PARAMETERS_5		0x0024

/* QCOM ICE v3.X only */
#define QCOM_ICE_GENERAL_ERR_STTS		0x0040
#define QCOM_ICE_INVALID_CCFG_ERR_STTS		0x0030
#define QCOM_ICE_GENERAL_ERR_MASK		0x0044

/* QCOM ICE v2.X only */
#define QCOM_ICE_REGS_NON_SEC_IRQ_STTS		0x0040
#define QCOM_ICE_REGS_NON_SEC_IRQ_MASK		0x0044

#define QCOM_ICE_REGS_NON_SEC_IRQ_CLR		0x0048
#define QCOM_ICE_REGS_STREAM1_ERROR_SYNDROME1	0x0050
#define QCOM_ICE_REGS_STREAM1_ERROR_SYNDROME2	0x0054
#define QCOM_ICE_REGS_STREAM2_ERROR_SYNDROME1	0x0058
#define QCOM_ICE_REGS_STREAM2_ERROR_SYNDROME2	0x005C
#define QCOM_ICE_REGS_STREAM1_BIST_ERROR_VEC	0x0060
#define QCOM_ICE_REGS_STREAM2_BIST_ERROR_VEC	0x0064
#define QCOM_ICE_REGS_STREAM1_BIST_FINISH_VEC	0x0068
#define QCOM_ICE_REGS_STREAM2_BIST_FINISH_VEC	0x006C
#define QCOM_ICE_REGS_BIST_STATUS		0x0070
#define QCOM_ICE_REGS_BYPASS_STATUS		0x0074
#define QCOM_ICE_REGS_ADVANCED_CONTROL		0x1000
#define QCOM_ICE_REGS_ENDIAN_SWAP		0x1004
#define QCOM_ICE_REGS_TEST_BUS_CONTROL		0x1010
#define QCOM_ICE_REGS_TEST_BUS_REG		0x1014

/* BIST ("built-in self-test"?) status flags */
#define ICE_BIST_STATUS_MASK			0xF0000000

#define ICE_FUSE_SETTING_MASK			0x1
#define ICE_FORCE_HW_KEY0_SETTING_MASK		0x2
#define ICE_FORCE_HW_KEY1_SETTING_MASK		0x4

#define qcom_ice_writel(host, val, reg)	\
	writel((val), (host)->ice_mmio + (reg))
#define qcom_ice_readl(host, reg)	\
	readl((host)->ice_mmio + (reg))

static int qcom_ice_check_version(struct ufs_qcom_host *host)
{
	u32 regval = qcom_ice_readl(host, QCOM_ICE_REGS_VERSION);
	int major = regval >> 24;
	int minor = (regval >> 16) & 0xFF;
	int step = regval & 0xFFFF;

	/* For now this driver only supports ICE version 3. */
	if (major != 3) {
		dev_warn(host->hba->dev,
			 "Unsupported ICE device (%d.%d.%d) @0x%pK\n",
			 major, minor, step, host->ice_mmio);
		return -ENODEV;
	}

	dev_info(host->hba->dev, "QC ICE %d.%d.%d device found @0x%pK\n",
		 major, minor, step, host->ice_mmio);

	host->ice_ver.major = major;
	host->ice_ver.minor = minor;
	host->ice_ver.step = step;
	return 0;
}

static int qcom_ice_check_fuses(struct ufs_qcom_host *host)
{
	u32 regval = qcom_ice_readl(host, QCOM_ICE_REGS_FUSE_SETTING);

	if (regval & (ICE_FUSE_SETTING_MASK |
		      ICE_FORCE_HW_KEY0_SETTING_MASK |
		      ICE_FORCE_HW_KEY1_SETTING_MASK)) {
		dev_err(host->hba->dev, "Fuses are blown; ICE is unusable!\n");
		return -ENODEV;
	}
	return 0;
}

int ufs_qcom_ice_init(struct ufs_qcom_host *host)
{
	struct device *dev = host->hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;
	int err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res)
		return -ENODEV;

	if (!qcom_scm_ice_available()) {
		dev_warn(host->hba->dev,
			 "ICE device is available, but SCM interface isn't.\n");
		return -ENODEV;
	}

	host->ice_mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->ice_mmio)) {
		err = PTR_ERR(host->ice_mmio);
		dev_err(dev, "failed to map ICE mmio registers, err %d\n", err);
		return err;
	}

	err = qcom_ice_check_version(host);
	if (err)
		return err;

	return qcom_ice_check_fuses(host);
}

static void qcom_ice_low_power_mode_enable(struct ufs_qcom_host *host)
{
	u32 regval;

	regval = qcom_ice_readl(host, QCOM_ICE_REGS_ADVANCED_CONTROL);
	/*
	 * Enable low power mode sequence
	 * [0]-0, [1]-0, [2]-0, [3]-E, [4]-0, [5]-0, [6]-0, [7]-0
	 */
	regval |= 0x7000;
	qcom_ice_writel(host, regval, QCOM_ICE_REGS_ADVANCED_CONTROL);
}

static void qcom_ice_optimization_enable(struct ufs_qcom_host *host)
{
	u32 regval;

	/* ICE Optimizations Enable Sequence */
	regval = qcom_ice_readl(host, QCOM_ICE_REGS_ADVANCED_CONTROL);
	regval |= 0xD807100;
	/* ICE HPG requires delay before writing */
	udelay(5);
	qcom_ice_writel(host, regval, QCOM_ICE_REGS_ADVANCED_CONTROL);
	udelay(5);
}

int ufs_qcom_ice_enable(struct ufs_qcom_host *host)
{
	if (!ufshcd_is_crypto_enabled(host->hba))
		return 0;
	qcom_ice_low_power_mode_enable(host);
	qcom_ice_optimization_enable(host);
	return ufs_qcom_ice_resume(host);
}

/* Poll until all BIST bits are reset */
static int qcom_ice_wait_bist_status(struct ufs_qcom_host *host)
{
	int count;
	u32 reg;

	for (count = 0; count < 100; count++) {
		reg = qcom_ice_readl(host, QCOM_ICE_REGS_BIST_STATUS);
		if (!(reg & ICE_BIST_STATUS_MASK))
			break;
		udelay(50);
	}
	if (reg)
		return -ETIMEDOUT;
	return 0;
}

int ufs_qcom_ice_resume(struct ufs_qcom_host *host)
{
	int err;

	if (!ufshcd_is_crypto_enabled(host->hba))
		return 0;

	err = qcom_ice_wait_bist_status(host);
	if (err) {
		dev_err(host->hba->dev, "BIST status error (%d)\n", err);
		return err;
	}
	return 0;
}

/*
 * Program a key into a QC ICE keyslot, or evict a keyslot.  QC ICE requires
 * vendor-specific SCM calls for this; it doesn't support the standard way.
 */
int ufs_qcom_ice_program_key(struct ufs_hba *hba,
			     const union ufs_crypto_cfg_entry *cfg, int slot)
{
	union ufs_crypto_cap_entry cap;
	enum qcom_scm_ice_cipher_mode mode;
	union {
		u8 bytes[UFS_CRYPTO_KEY_MAX_SIZE];
		u32 words[UFS_CRYPTO_KEY_MAX_SIZE / sizeof(u32)];
	} key;
	int key_size;
	int i;
	int err;

	if (!(cfg->config_enable & UFS_CRYPTO_CONFIGURATION_ENABLE))
		return qcom_scm_ice_invalidate_key(slot);

	/* Only AES-256-XTS has been tested so far. */
	cap = hba->crypto_cap_array[cfg->crypto_cap_idx];
	if (cap.algorithm_id == UFS_CRYPTO_ALG_AES_XTS &&
	    cap.key_size == UFS_CRYPTO_KEY_SIZE_256) {
		mode = QCOM_SCM_ICE_CIPHER_MODE_XTS_256;
		key_size = AES_256_XTS_KEY_SIZE;
		memcpy(key.bytes, cfg->crypto_key, key_size);
	} else {
		dev_err_ratelimited(hba->dev,
				    "Unhandled crypto capability; algorithm_id=%d, key_size=%d\n",
				    cap.algorithm_id, cap.key_size);
		err = -ENOTSUPP;
		goto out;
	}

	/*
	 * ICE (or maybe the SCM call?) byte-swaps the 32-bit words of the key.
	 * So we have to do the same, in order for the final key be correct.
	 */
	for (i = 0; i < key_size / sizeof(u32); i++)
		__cpu_to_be32s(&key.words[i]);

	err = qcom_scm_ice_set_key(slot, key.bytes, key_size, mode,
				   cfg->data_unit_size);
out:
	memzero_explicit(&key, sizeof(key));
	return err;
}
