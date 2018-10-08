/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *
 * PMIC rails controller for airbrush state manager.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#include <linux/airbrush-sm-ctrl.h>

int ab_blk_pw_rails_enable(struct ab_state_context *sc,
				   block_name_t blk_name)
{
	switch (blk_name) {

	case BLK_IPU:
	case BLK_TPU:
		if (!regulator_is_enabled(sc->smps2))
			if (regulator_enable(sc->smps2))
				goto fail_regulator_enable;
		if (!regulator_is_enabled(sc->ldo4))
			if (regulator_enable(sc->ldo4))
				goto fail_regulator_enable;
		if (!regulator_is_enabled(sc->ldo5))
			if (regulator_enable(sc->ldo5))
				goto fail_regulator_enable;
		if (!regulator_is_enabled(sc->smps1))
			if (regulator_enable(sc->smps1))
				goto fail_regulator_enable;
		if (!regulator_is_enabled(sc->ldo3))
			if (regulator_enable(sc->ldo3))
				goto fail_regulator_enable;
		if (!regulator_is_enabled(sc->ldo2))
			if (regulator_enable(sc->ldo2))
				goto fail_regulator_enable;
		break;
	case BLK_AON:
		if (!regulator_is_enabled(sc->smps2))
			if (regulator_enable(sc->smps2))
				goto fail_regulator_enable;
		if (!regulator_is_enabled(sc->ldo4))
			if (regulator_enable(sc->ldo4))
				goto fail_regulator_enable;
		if (!regulator_is_enabled(sc->ldo5))
			if (regulator_enable(sc->ldo5))
				goto fail_regulator_enable;
		break;
	case DRAM:
		if (!regulator_is_enabled(sc->ldo2))
			if (regulator_enable(sc->ldo2))
				goto fail_regulator_enable;
		break;
	case BLK_MIF:
		break;
	case BLK_FSYS:
		break;
	default:
		return -EINVAL;
	}
	return 0;

fail_regulator_enable:
	if (regulator_is_enabled(sc->ldo3))
		regulator_disable(sc->ldo3);
	if (regulator_is_enabled(sc->smps1))
		regulator_disable(sc->smps1);
	if (regulator_is_enabled(sc->ldo5))
		regulator_disable(sc->ldo5);
	if (regulator_is_enabled(sc->ldo4))
		regulator_disable(sc->ldo4);
	if (regulator_is_enabled(sc->smps3))
		regulator_disable(sc->smps3);
	if (regulator_is_enabled(sc->ldo1))
		regulator_disable(sc->ldo1);
	if (regulator_is_enabled(sc->smps2))
		regulator_disable(sc->smps2);
	dev_err(sc->dev, "%s: PMIC power up failure\n", __func__);

	return -ENODEV;

}

int ab_blk_pw_rails_disable(struct ab_state_context *sc,
				   block_name_t blk_name)
{
	switch (blk_name) {

	case BLK_IPU:
	case BLK_TPU:
		if (regulator_is_enabled(sc->ldo3)) {
			if (regulator_disable(sc->ldo3))
				goto fail_regulator_disable;
		}
		if (regulator_is_enabled(sc->smps1)) {
			if (regulator_disable(sc->smps1))
				goto fail_regulator_disable;
		}
		break;

	case BLK_AON:
		if (regulator_is_enabled(sc->ldo5))
			if (regulator_disable(sc->ldo5))
				goto fail_regulator_disable;
		if (regulator_is_enabled(sc->ldo4))
			if (regulator_disable(sc->ldo4))
				goto fail_regulator_disable;
		if (regulator_is_enabled(sc->smps2))
			if (regulator_disable(sc->smps2))
				goto fail_regulator_disable;
		break;
	case DRAM:
		break;

	case BLK_MIF:
	case BLK_FSYS:
	default:
		return -EINVAL;
	}
	return 0;

fail_regulator_disable:
	dev_err(sc->dev, "%s: PMIC power down failure\n", __func__);

	return -ENODEV;
}

int ab_pmic_on(struct ab_state_context *sc)
{
	if (regulator_enable(sc->smps2))
		goto fail_regulator_smps2;
	if (regulator_enable(sc->ldo1))
		goto fail_regulator_ldo1;
	if (regulator_enable(sc->smps3))
		goto fail_regulator_smps3;
	if (regulator_enable(sc->ldo4))
		goto fail_regulator_ldo4;
	if (regulator_enable(sc->ldo5))
		goto fail_regulator_ldo5;
	if (regulator_enable(sc->smps1))
		goto fail_regulator_smps1;
	if (regulator_enable(sc->ldo3))
		goto fail_regulator_ldo3;
	if (regulator_enable(sc->ldo2))
		goto fail_regulator_ldo2;
	return 0;

fail_regulator_ldo2:
		regulator_disable(sc->ldo3);
fail_regulator_ldo3:
		regulator_disable(sc->smps1);
fail_regulator_smps1:
		regulator_disable(sc->ldo5);
fail_regulator_ldo5:
		regulator_disable(sc->ldo4);
fail_regulator_ldo4:
		regulator_disable(sc->smps3);
fail_regulator_smps3:
		regulator_disable(sc->ldo1);
fail_regulator_ldo1:
		regulator_disable(sc->smps2);
fail_regulator_smps2:
	dev_err(sc->dev, "%s: PMIC power up failure\n", __func__);
	return -ENODEV;

}

int ab_get_pmic_resources(struct ab_state_context *sc)
{
	struct device *dev = sc->dev;

	mutex_lock(&sc->lock);

	if (!sc->soc_pwrgood) {
		sc->soc_pwrgood =
			devm_gpiod_get(dev, "soc-pwrgood", GPIOD_OUT_LOW);
		if (IS_ERR(sc->soc_pwrgood)) {
			dev_err(dev, "%s: Could not get pmic_soc_pwrgood gpio (%ld)\n",
					__func__, PTR_ERR(sc->soc_pwrgood));
			goto fail;
		}
	}

	if (!sc->smps1) {
		sc->smps1 = devm_regulator_get(dev, "s2mpb04_smps1");
		if (IS_ERR(sc->smps1)) {
			dev_err(dev, "%s: failed to get s2mpb04_smps1 supply (%ld)\n",
					__func__, PTR_ERR(sc->smps1));
			goto fail;
		}
	}

	if (!sc->smps2) {
		sc->smps2 = devm_regulator_get(dev, "s2mpb04_smps2");
		if (IS_ERR(sc->smps2)) {
			dev_err(dev, "%s: failed to get s2mpb04_smps2 supply (%ld)\n",
					__func__, PTR_ERR(sc->smps2));
			goto fail;
		}
	}

	if (!sc->smps3) {
		sc->smps3 = devm_regulator_get(dev, "s2mpb04_smps3");
		if (IS_ERR(sc->smps3)) {
			dev_err(dev, "%s: failed to get s2mpb04_smps3 supply (%ld)\n",
					__func__, PTR_ERR(sc->smps3));
			goto fail;
		}
	}

	if (!sc->ldo1) {
		sc->ldo1 = devm_regulator_get(dev, "s2mpb04_ldo1");
		if (IS_ERR(sc->ldo1)) {
			dev_err(dev, "%s: failed to get s2mpb04_ldo1 supply (%ld)\n",
					__func__, PTR_ERR(sc->ldo1));
			goto fail;
		}
	}

	if (!sc->ldo2) {
		sc->ldo2 = devm_regulator_get(dev, "s2mpb04_ldo2");
		if (IS_ERR(sc->ldo2)) {
			dev_err(dev, "%s: failed to get s2mpb04_ldo2 supply (%ld)\n",
					__func__, PTR_ERR(sc->ldo2));
			goto fail;
		}
	}

	if (!sc->ldo3) {
		sc->ldo3 = devm_regulator_get(dev, "s2mpb04_ldo3");
		if (IS_ERR(sc->ldo3)) {
			dev_err(dev, "%s: failed to get s2mpb04_ldo3 supply (%ld)\n",
					__func__, PTR_ERR(sc->ldo3));
			goto fail;
		}
	}

	if (!sc->ldo4) {
		sc->ldo4 = devm_regulator_get(dev, "s2mpb04_ldo4");
		if (IS_ERR(sc->ldo4)) {
			dev_err(dev, "%s: failed to get s2mpb04_ldo4 supply (%ld)\n",
					__func__, PTR_ERR(sc->ldo4));
			goto fail;
		}
	}

	if (!sc->ldo5) {
		sc->ldo5 = devm_regulator_get(dev, "s2mpb04_ldo5");
		if (IS_ERR(sc->ldo5)) {
			dev_err(dev, "%s: failed to get s2mpb04_ldo5 supply (%ld)\n",
					__func__, PTR_ERR(sc->ldo5));
			goto fail;
		}
	}

	mutex_unlock(&sc->lock);
	return 0;

fail:
	mutex_unlock(&sc->lock);
	return -ENODEV;
}


