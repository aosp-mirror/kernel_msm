/*
 *
 * MNH PWR APIs
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

/* #define DEBUG */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/interrupt.h>
#include <linux/msm_pcie.h>
#include <linux/regulator/consumer.h>

#include "mnh-pcie.h"
#include "mnh-pwr.h"
#include "mnh-sm.h"

#define MNH_PCIE_RC_INDEX   1
#define MNH_PCIE_VENDOR_ID  0x8086
#define MNH_PCIE_DEVICE_ID  0x3140

#define PM_OPT_SUSPEND MSM_PCIE_CONFIG_LINKDOWN
#define PM_OPT_RESUME MSM_PCIE_CONFIG_NO_CFG_FREE

struct mnh_pwr_data {
	struct platform_device *pdev;
	struct device *dev;

	/* regulators */
	struct regulator *smps1_supply;
	struct regulator *smps2_supply;
	struct regulator *ldo2_supply;
	struct regulator *ldo1_supply;
	struct notifier_block smps1_nb;
	struct notifier_block smps2_nb;
	struct notifier_block ldo1_nb;
	struct notifier_block ldo2_nb;

	/* clocks */
	struct clk *ref_clk;
	bool ref_clk_enabled;

	/* pins */
	struct gpio_desc *soc_pwr_good_pin;

	/* pcie device */
	struct pci_dev *pcidev;
	struct pci_saved_state *pristine_state;
	struct msm_pcie_register_event pci_link_event;

	struct work_struct shutdown_work;

	bool pcie_failure;

	enum mnh_pwr_state state;

	struct mutex lock;
};

static struct mnh_pwr_data *mnh_pwr;

static void mnh_pwr_shutdown_work(struct work_struct *data)
{
	dev_err(mnh_pwr->dev, "%s: begin emergency power down\n", __func__);

	mnh_pwr->pcie_failure = true;
	mnh_pwr_set_state(MNH_PWR_S4);
	mnh_sm_pwr_error_cb();
}

static int mnh_pwr_smps1_notifier_cb(struct notifier_block *nb,
				   unsigned long event, void *cookie)
{
	dev_dbg(mnh_pwr->dev, "%s: received event %ld\n", __func__, event);

	/* force emergency shutdown if regulator output has failed */
	if (event == REGULATOR_EVENT_FAIL) {
		dev_err(mnh_pwr->dev,
			"%s: smps1 supply has failed, forcing shutdown\n",
			__func__);

		if (mnh_pwr->state != MNH_PWR_S4)
			schedule_work(&mnh_pwr->shutdown_work);
	}

	return 0;
}

static int mnh_pwr_smps2_notifier_cb(struct notifier_block *nb,
				   unsigned long event, void *cookie)
{
	dev_dbg(mnh_pwr->dev, "%s: received event %ld\n", __func__, event);

	/* force emergency shutdown if regulator output has failed */
	if (event == REGULATOR_EVENT_FAIL) {
		dev_err(mnh_pwr->dev,
			"%s: smps2 supply has failed, forcing shutdown\n",
			__func__);

		if (mnh_pwr->state != MNH_PWR_S4)
			schedule_work(&mnh_pwr->shutdown_work);
	}

	return 0;
}

static int mnh_pwr_ldo1_notifier_cb(struct notifier_block *nb,
				   unsigned long event, void *cookie)
{
	dev_dbg(mnh_pwr->dev, "%s: received event %ld\n", __func__, event);

	/* force emergency shutdown if regulator output has failed */
	if (event == REGULATOR_EVENT_FAIL) {
		dev_err(mnh_pwr->dev,
			"%s: ldo1 supply has failed, forcing shutdown\n",
			__func__);

		if (mnh_pwr->state != MNH_PWR_S4)
			schedule_work(&mnh_pwr->shutdown_work);
	}

	return 0;
}

static int mnh_pwr_ldo2_notifier_cb(struct notifier_block *nb,
				   unsigned long event, void *cookie)
{
	dev_dbg(mnh_pwr->dev, "%s: received event %ld\n", __func__, event);

	/* force emergency shutdown if regulator output has failed */
	if (event == REGULATOR_EVENT_FAIL) {
		dev_err(mnh_pwr->dev,
			"%s: ldo2 supply has failed, forcing shutdown\n",
			__func__);

		if (mnh_pwr->state != MNH_PWR_S4)
			schedule_work(&mnh_pwr->shutdown_work);
	}

	return 0;
}

void mnh_pwr_pcie_link_state_cb(struct msm_pcie_notify *notify)
{
	struct mnh_pwr_data *mnh_pwr = notify->data;

	switch (notify->event) {
	case MSM_PCIE_EVENT_LINKDOWN:
		dev_err(mnh_pwr->dev,
			"%s: PCIe link is down, forcing power down\n",
			__func__);

		/* force emergency shutdown */
		schedule_work(&mnh_pwr->shutdown_work);
		break;
	default:
		dev_err(mnh_pwr->dev,
			"%s: received invalid pcie link state event (%d)\n",
			__func__, notify->event);
		break;
	}
}

static int mnh_pwr_pcie_enumerate(void)
{
	struct pci_dev *pcidev = NULL;
	int ret;

	/* enumerate PCIE */
	ret = msm_pcie_enumerate(MNH_PCIE_RC_INDEX);
	if (ret < 0) {
		dev_err(mnh_pwr->dev, "%s: pcie enumeration failed (%d)\n",
			__func__, ret);
		return ret;
	}

	/* search for PCIE device in our domain */
	do {
		pcidev = pci_get_device(MNH_PCIE_VENDOR_ID, MNH_PCIE_DEVICE_ID, pcidev);
		if (!pcidev)
			break;

		if (pci_domain_nr(pcidev->bus) == MNH_PCIE_RC_INDEX)
			break;
	} while (true);
	if (!pcidev) {
		dev_err(mnh_pwr->dev, "%s: could not find mnh device\n",
			__func__);
		return -ENODEV;
	}

	/* save current state in pcidev */
	ret = pci_save_state(pcidev);
	if (ret) {
		dev_err(mnh_pwr->dev, "%s: pci_save_state failed (%d)\n",
			__func__, ret);
		pci_dev_put(pcidev);
		return ret;
	}

	/* store saved state so we can recall it after resume */
	mnh_pwr->pristine_state = pci_store_saved_state(pcidev);
	if (!mnh_pwr->pristine_state) {
		dev_err(mnh_pwr->dev,
			"%s: pci_store_saved_state failed\n",
			__func__);
		pci_dev_put(pcidev);
		return ret;
	}

	/* save device to driver struct */
	mnh_pwr->pcidev = pcidev;

	return 0;
}

static int mnh_pwr_pcie_suspend(bool pcie_failure)
{
	struct pci_dev *pcidev = mnh_pwr->pcidev;
	int ret;

	if (!pcidev)
		return -ENODEV;

	/* suspend the driver state */
	ret = mnh_pci_suspend();
	if (ret) {
		dev_err(mnh_pwr->dev, "%s: mnh_pci_suspend failed (%d)\n",
			__func__, ret);
	}

	if (pcie_failure) {
		/* call the platform driver to update link status */
		ret = msm_pcie_pm_control(MSM_PCIE_SUSPEND, pcidev->bus->number,
			pcidev, NULL,
			PM_OPT_SUSPEND | MSM_PCIE_CONFIG_NO_CFG_RESTORE);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: msm_pcie_pm_control(suspend) failed (%d)\n",
				__func__, ret);
			return ret;
		}

		mnh_pwr->pcie_failure = false;
	} else {
		/* prepare the root complex and endpoint for going to suspend */
		ret = pci_prepare_to_sleep(pcidev);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: pci_prepare_to_sleep failed (%d)\n",
				__func__, ret);
		}

		/* call the platform driver to suspend PCIe link */
		ret = msm_pcie_pm_control(MSM_PCIE_SUSPEND, pcidev->bus->number,
					  pcidev, NULL, PM_OPT_SUSPEND);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: msm_pcie_pm_control(suspend) failed (%d)\n",
				__func__, ret);
			return ret;
		}
	}

	return 0;
}

static int mnh_pwr_pcie_resume(void)
{
	struct pci_dev *pcidev = mnh_pwr->pcidev;
	int ret;

	/* check for valid pcidev */
	if (!pcidev) {
		/* enumerate pci device */
		ret = mnh_pwr_pcie_enumerate();
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: mnh_pwr_pcie_enumerate failed (%d)\n",
				__func__, ret);
			return ret;
		}

		/* save the newly enumerated device to the local copy */
		pcidev = mnh_pwr->pcidev;

		/* register for link down events so we can handle them */
		mnh_pwr->pci_link_event.events = MSM_PCIE_EVENT_LINKDOWN;
		mnh_pwr->pci_link_event.user = pcidev;
		mnh_pwr->pci_link_event.callback = mnh_pwr_pcie_link_state_cb;
		mnh_pwr->pci_link_event.notify.data = mnh_pwr;
		ret = msm_pcie_register_event(&mnh_pwr->pci_link_event);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: msm_pcie_register_event failed (%d)\n",
				__func__, ret);
		}
	} else {
		ret = msm_pcie_pm_control(MSM_PCIE_RESUME, pcidev->bus->number,
					  pcidev, NULL, PM_OPT_RESUME);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: msm_pcie_pm_control(resume) failed (%d)\n",
				__func__, ret);
			return ret;
		}

		/* prepare the root complex and endpoint */
		ret = pci_back_from_sleep(pcidev);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: pci_back_from_sleep failed (%d)\n",
				__func__, ret);
			goto fail_pcie_resume_awake;
		}

		/* load the saved state in the device buffer */
		ret = pci_load_saved_state(pcidev, mnh_pwr->pristine_state);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: pci_load_saved_state failed (%d)\n",
				__func__, ret);
			goto fail_pcie_resume_load_state;
		}

		/* apply the saved state to the device */
		pci_restore_state(pcidev);

		/* resume the driver state */
		ret = mnh_pci_resume();
		if (ret) {
			dev_err(mnh_pwr->dev, "%s: mnh_pci_resume failed (%d)\n",
				__func__, ret);
			goto fail_pcie_resume_mnh_init;
		}
	}

	mnh_pwr->pcie_failure = false;

	return 0;

fail_pcie_resume_mnh_init:
fail_pcie_resume_load_state:
	pci_prepare_to_sleep(pcidev);
fail_pcie_resume_awake:
	msm_pcie_pm_control(MSM_PCIE_SUSPEND, pcidev->bus->number, pcidev,
			     NULL, PM_OPT_SUSPEND);

	return ret;
}

static int __mnh_pwr_down(bool pcie_failure)
{
	int ret;

	if (mnh_sm_get_boot_mode() == MNH_BOOT_MODE_PCIE) {
		/* suspend pcie link */
		ret = mnh_pwr_pcie_suspend(pcie_failure);
		if (ret) {
			dev_err(mnh_pwr->dev, "%s: failed to suspend pcie link (%d)\n",
				__func__, ret);
			goto fail_pwr_down_pcie;
		}
	} else {
		/* assert reset */
		msm_pcie_set_reset(MNH_PCIE_RC_INDEX, true);
	}

	/* deassert soc_pwr_good */
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 0);

	/* disable clocks */
	if (mnh_pwr->ref_clk_enabled) {
		clk_disable_unprepare(mnh_pwr->ref_clk);
		mnh_pwr->ref_clk_enabled = false;
	}

	/* disable supplies: smps2 -> smps1 -> ldo1 -> ldo2 */
	ret = regulator_disable(mnh_pwr->smps2_supply);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: failed to disable smps2 (%d)\n", __func__, ret);
		goto fail_pwr_down_smps2;
	}

	if (mnh_pwr->state == MNH_PWR_S0) {
		ret = regulator_disable(mnh_pwr->smps1_supply);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: failed to disable smps1 (%d)\n",
				__func__, ret);
			goto fail_pwr_down_smps1;
		}

		regulator_disable(mnh_pwr->ldo1_supply);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: failed to disable ldo1 (%d)\n",
				__func__, ret);
			goto fail_pwr_down_ldo1;
		}
	}
	regulator_disable(mnh_pwr->ldo2_supply);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: failed to disable ldo2 (%d)\n", __func__, ret);
		goto fail_pwr_down_ldo2;
	}

	mnh_pwr->state = MNH_PWR_S4;

	return 0;

fail_pwr_down_pcie:
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 0);
	if (mnh_pwr->ref_clk_enabled) {
		clk_disable_unprepare(mnh_pwr->ref_clk);
		mnh_pwr->ref_clk_enabled = false;
	}
	if (regulator_is_enabled(mnh_pwr->smps2_supply))
		regulator_disable(mnh_pwr->smps2_supply);
fail_pwr_down_smps2:
	if (regulator_is_enabled(mnh_pwr->smps1_supply))
		regulator_disable(mnh_pwr->smps1_supply);
fail_pwr_down_smps1:
	if (regulator_is_enabled(mnh_pwr->ldo1_supply))
		regulator_disable(mnh_pwr->ldo1_supply);
fail_pwr_down_ldo1:
	if (regulator_is_enabled(mnh_pwr->ldo2_supply))
		regulator_disable(mnh_pwr->ldo2_supply);
fail_pwr_down_ldo2:

	dev_err(mnh_pwr->dev,
		"%s: force shutdown because of powerdown failure (%d)\n",
		__func__, ret);

	mnh_pwr->state = MNH_PWR_S4;

	return ret;
}

static inline int mnh_pwr_down(void)
{
	return __mnh_pwr_down(false);
}

static int mnh_pwr_suspend(void)
{
	int ret;

	if (mnh_sm_get_boot_mode() == MNH_BOOT_MODE_PCIE) {
		/* suspend pcie link */
		ret = mnh_pwr_pcie_suspend(false);
		if (ret) {
			dev_err(mnh_pwr->dev, "%s: failed to suspend pcie link (%d)\n",
				__func__, ret);
			goto fail_pwr_suspend_pcie;
		}
	} else {
		/* assert reset */
		msm_pcie_set_reset(MNH_PCIE_RC_INDEX, true);
	}

	/* deassert soc_pwr_good */
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 0);

	/* disable clocks */
	if (mnh_pwr->ref_clk_enabled) {
		clk_disable_unprepare(mnh_pwr->ref_clk);
		mnh_pwr->ref_clk_enabled = false;
	}

	/* disable core supplies */
	ret = regulator_disable(mnh_pwr->smps1_supply);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: failed to disable smps1 (%d)\n", __func__, ret);
		goto fail_pwr_suspend_regulators;
	}

	regulator_disable(mnh_pwr->ldo1_supply);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: failed to disable ldo1 (%d)\n", __func__, ret);
		goto fail_pwr_suspend_regulators;
	}

	mnh_pwr->state = MNH_PWR_S3;

	return 0;

fail_pwr_suspend_pcie:
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 0);
	if (mnh_pwr->ref_clk_enabled) {
		clk_disable_unprepare(mnh_pwr->ref_clk);
		mnh_pwr->ref_clk_enabled = false;
	}
fail_pwr_suspend_regulators:
	if (regulator_is_enabled(mnh_pwr->smps2_supply))
		regulator_disable(mnh_pwr->smps2_supply);
	if (regulator_is_enabled(mnh_pwr->smps1_supply))
		regulator_disable(mnh_pwr->smps1_supply);
	if (regulator_is_enabled(mnh_pwr->ldo1_supply))
		regulator_disable(mnh_pwr->ldo1_supply);
	if (regulator_is_enabled(mnh_pwr->ldo2_supply))
		regulator_disable(mnh_pwr->ldo2_supply);

	dev_err(mnh_pwr->dev,
		"%s: force shutdown because of suspend failure (%d)\n",
		__func__, ret);

	mnh_pwr->state = MNH_PWR_S4;

	return ret;
}

static int mnh_pwr_up(enum mnh_pwr_state next_state)
{
	int ret;

	/* enable supplies */
	/* ldo2 -> ldo1 -> smps1 -> smps2 */
	if (mnh_pwr->state == MNH_PWR_S4) {
		ret = regulator_enable(mnh_pwr->ldo2_supply);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: failed to enable ldo2 (%d)\n",
				__func__, ret);
			goto fail_pwr_up_regulators;
		}
	}

	ret = regulator_enable(mnh_pwr->ldo1_supply);
	if (ret) {
		dev_err(mnh_pwr->dev, "%s: failed to enable ldo1 (%d)\n",
			__func__, ret);
		goto fail_pwr_up_regulators;
	}

	ret = regulator_enable(mnh_pwr->smps1_supply);
	if (ret) {
		dev_err(mnh_pwr->dev, "%s: failed to enable ldo1 (%d)\n",
			__func__, ret);
		goto fail_pwr_up_regulators;
	}

	if (mnh_pwr->state == MNH_PWR_S4) {
		ret = regulator_enable(mnh_pwr->smps2_supply);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: failed to enable smps2 (%d)\n",
				__func__, ret);
			goto fail_pwr_up_regulators;
		}
	}

	/* turn on clocks */
	if (!mnh_pwr->ref_clk_enabled) {
		ret = clk_prepare_enable(mnh_pwr->ref_clk);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: failed to enable ref clk (%d)\n",
				__func__, ret);
			goto fail_pwr_up_ref_clk;
		}
		mnh_pwr->ref_clk_enabled = true;
	}

	/* assert soc_pwr_good */
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 1);

	/* give the PLLs some time to initialize */
	udelay(60);

	if (mnh_sm_get_boot_mode() == MNH_BOOT_MODE_PCIE) {
		/* resume pcie link */
		ret = mnh_pwr_pcie_resume();
		if (ret) {
			dev_err(mnh_pwr->dev, "%s: failed to resume pcie link (%d)\n",
				__func__, ret);
			goto fail_pwr_up_pcie;
		}
	} else {
		/* deassert reset */
		msm_pcie_set_reset(MNH_PCIE_RC_INDEX, false);
	}

	mnh_pwr->state = next_state;

	return 0;

fail_pwr_up_pcie:
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 0);
	if (mnh_pwr->ref_clk_enabled) {
		clk_disable_unprepare(mnh_pwr->ref_clk);
		mnh_pwr->ref_clk_enabled = false;
	}
fail_pwr_up_ref_clk:
fail_pwr_up_regulators:
	if (regulator_is_enabled(mnh_pwr->smps2_supply))
		regulator_disable(mnh_pwr->smps2_supply);
	if (regulator_is_enabled(mnh_pwr->smps1_supply))
		regulator_disable(mnh_pwr->smps1_supply);
	if (regulator_is_enabled(mnh_pwr->ldo1_supply))
		regulator_disable(mnh_pwr->ldo1_supply);
	if (regulator_is_enabled(mnh_pwr->ldo2_supply))
		regulator_disable(mnh_pwr->ldo2_supply);

	dev_err(mnh_pwr->dev,
		"%s: force shutdown because of power up failure (%d)\n",
		__func__, ret);

	mnh_pwr->state = MNH_PWR_S4;

	return ret;
}

static int mnh_pwr_get_resources(void)
{
	struct platform_device *pdev = mnh_pwr->pdev;
	struct device *dev = mnh_pwr->dev;
	int ret;

	/* request supplies */
	mnh_pwr->smps1_supply = devm_regulator_get(&pdev->dev, "s2mpb04_smps1");
	if (IS_ERR(mnh_pwr->smps1_supply)) {
		dev_err(dev, "%s: failed to get smps1 supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->smps1_supply));
		return PTR_ERR(mnh_pwr->smps1_supply);
	}

	mnh_pwr->smps2_supply = devm_regulator_get(&pdev->dev, "s2mpb04_smps2");
	if (IS_ERR(mnh_pwr->smps2_supply)) {
		dev_err(dev, "%s: failed to get smps2 supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->smps2_supply));
		return PTR_ERR(mnh_pwr->smps2_supply);
	}

	mnh_pwr->ldo1_supply = devm_regulator_get(&pdev->dev,
						   "s2mpb04_ldo1");
	if (IS_ERR(mnh_pwr->ldo1_supply)) {
		dev_err(dev, "%s: failed to get ldo1 supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->ldo1_supply));
		return PTR_ERR(mnh_pwr->ldo1_supply);
	}

	mnh_pwr->ldo2_supply = devm_regulator_get(&pdev->dev,
						   "s2mpb04_ldo2");
	if (IS_ERR(mnh_pwr->ldo2_supply)) {
		dev_err(dev, "%s: failed to get ldo2 supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->ldo2_supply));
		return PTR_ERR(mnh_pwr->ldo2_supply);
	}

	/* register the notifier for each of the supplies */
	mnh_pwr->smps1_nb.notifier_call = mnh_pwr_smps1_notifier_cb;
	ret = devm_regulator_register_notifier(mnh_pwr->smps1_supply,
					       &mnh_pwr->smps1_nb);
	if (ret) {
		dev_err(dev,
			"%s: failed to register notifier block for smps1 supply (%d)\n",
			__func__, ret);
		return ret;
	}

	mnh_pwr->smps2_nb.notifier_call = mnh_pwr_smps2_notifier_cb;
	ret = devm_regulator_register_notifier(mnh_pwr->smps2_supply,
					       &mnh_pwr->smps2_nb);
	if (ret) {
		dev_err(dev,
			"%s: failed to register notifier block for smps2 supply (%d)\n",
			__func__, ret);
		return ret;
	}

	mnh_pwr->ldo1_nb.notifier_call = mnh_pwr_ldo1_notifier_cb;
	ret = devm_regulator_register_notifier(mnh_pwr->ldo1_supply,
					       &mnh_pwr->ldo1_nb);
	if (ret) {
		dev_err(dev,
			"%s: failed to register notifier block for ldo1 supply (%d)\n",
			__func__, ret);
		return ret;
	}

	mnh_pwr->ldo2_nb.notifier_call = mnh_pwr_ldo2_notifier_cb;
	ret = devm_regulator_register_notifier(mnh_pwr->ldo2_supply,
					       &mnh_pwr->ldo2_nb);
	if (ret) {
		dev_err(dev,
			"%s: failed to register notifier block for ldo2 supply (%d)\n",
			__func__, ret);
		return ret;
	}

	/* request gpio descriptors */
	mnh_pwr->soc_pwr_good_pin = devm_gpiod_get(&pdev->dev, "soc-pwr-good",
						   GPIOD_OUT_LOW);
	if (IS_ERR(mnh_pwr->soc_pwr_good_pin)) {
		dev_err(dev, "%s: could not get soc_pwr_good gpio (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->soc_pwr_good_pin));
		return PTR_ERR(mnh_pwr->soc_pwr_good_pin);
	}

	/* request clocks */
	mnh_pwr->ref_clk = devm_clk_get(&pdev->dev, "ref_clk");
	if (IS_ERR(mnh_pwr->ref_clk)) {
		dev_err(dev, "%s: could not get ref clk (%ld)\n", __func__,
			PTR_ERR(mnh_pwr->ref_clk));
		return PTR_ERR(mnh_pwr->ref_clk);
	}

	return 0;
}

int mnh_pwr_set_state(enum mnh_pwr_state system_state)
{
	int ret = 0;
	enum mnh_pwr_state curr_state =  mnh_pwr_get_state();

	dev_dbg(mnh_pwr->dev, "%s req: %d, current: %d\n", __func__,
		system_state, curr_state);

	mutex_lock(&mnh_pwr->lock);

	if (system_state != mnh_pwr->state) {
		switch (system_state) {
		case MNH_PWR_S0:
			ret = mnh_pwr_up(system_state);
			break;
		case MNH_PWR_S3:
			ret = mnh_pwr_suspend();
			break;
		case MNH_PWR_S4:
			ret = mnh_pwr_down();
			break;
		default:
			dev_err(mnh_pwr->dev, "%s: invalid state %d\n",
				__func__, system_state);
			ret = -EINVAL;
			break;
		}

		if (ret)
			dev_err(mnh_pwr->dev,
				"%s: state transition failed (%d)\n",
				__func__, ret);
		else
			dev_dbg(mnh_pwr->dev, "%s done with state: %d\n",
				__func__, system_state);
	} else {
		dev_dbg(mnh_pwr->dev, "%s: no state change needed\n", __func__);
	}

	mutex_unlock(&mnh_pwr->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(mnh_pwr_set_state);

enum mnh_pwr_state mnh_pwr_get_state(void)
{
	enum mnh_pwr_state curr_state;

	mutex_lock(&mnh_pwr->lock);
	curr_state = mnh_pwr->state;
	mutex_unlock(&mnh_pwr->lock);

	return curr_state;
}
EXPORT_SYMBOL_GPL(mnh_pwr_get_state);

int mnh_pwr_init(struct platform_device *pdev, struct device *dev)
{
	int ret;

	/* allocate memory for mnh_pwr_data struct */
	mnh_pwr = devm_kzalloc(dev, sizeof(struct mnh_pwr_data), GFP_KERNEL);
	if (!mnh_pwr)
		return -ENOMEM;

	/* save a local copy of the device struct */
	mnh_pwr->pdev = pdev;
	mnh_pwr->dev = dev;

	/* initialize power state to powered down */
	mnh_pwr->state = MNH_PWR_S4;

	/* get platform resources */
	ret = mnh_pwr_get_resources();
	if (ret) {
		dev_err(dev, "%s: failed to get platform resources (%d)\n",
			__func__, ret);
		return ret;
	}

	/* initialize some structures */
	INIT_WORK(&mnh_pwr->shutdown_work, mnh_pwr_shutdown_work);
	mutex_init(&mnh_pwr->lock);

	/* power on the device to enumerate PCIe */
	ret = mnh_pwr_up(MNH_PWR_S0);
	if (ret) {
		dev_err(dev, "%s: failed initial power up (%d)", __func__, ret);
		return ret;
	}

	/* power down the device */
	ret = mnh_pwr_down();
	if (ret) {
		dev_err(dev, "%s: failed initial power down (%d)", __func__,
			ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mnh_pwr_init);
