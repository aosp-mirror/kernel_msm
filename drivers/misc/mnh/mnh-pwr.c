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

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/interrupt.h>
#include <linux/msm_pcie.h>
#include <linux/regulator/consumer.h>

#include "mnh-pwr.h"

/* defines the timeout in jiffies for ready signal assertion */
#define MNH_READY_TIMEOUT msecs_to_jiffies(250)

#define MNH_PCIE_RC_INDEX 0
#define MNH_PCIE_VENDOR_ID  0x8086
#define MNH_PCIE_DEVICE_ID  0x3140
#define DISABLE_PCIE_L1_MASK  0xFFFFFFFD
#define PCIE20_CAP_LINKCTRLSTATUS  0x80

#define PM_OPT_SUSPEND (MSM_PCIE_CONFIG_NO_CFG_RESTORE | \
			MSM_PCIE_CONFIG_LINKDOWN)
#define PM_OPT_RESUME MSM_PCIE_CONFIG_NO_CFG_RESTORE

struct mnh_pwr_data {
	struct device *dev;

	/* regulators */
	struct regulator *asr_supply;
	struct regulator *sdsr_supply;
	struct regulator *sdldo_supply;
	struct regulator *ioldo_supply;

	/* clocks */
	struct clk *ref_clk;
	struct clk *sleep_clk;

	/* pins */
	struct gpio_desc *boot_mode_pin;
	struct gpio_desc *soc_pwr_good_pin;
	struct gpio_desc *ddr_pad_iso_n_pin;
	struct gpio_desc *ready_pin;

	/* irqs */
	unsigned int ready_irq;

	/* pcie device */
	struct pci_dev *pcidev;

	/* completion used for easel ready signal */
	struct completion ready_complete;

	enum mnh_pwr_state state;
};

static struct mnh_pwr_data *mnh_pwr;

static int mnh_pwr_pcie_enumerate(void)
{
	struct pci_dev *pcidev = NULL;
	int ret;

	/* enumerate PCIE */
	ret = msm_pcie_enumerate(MNH_PCIE_RC_INDEX);
	if (ret < 0) {
		dev_err(mnh_pwr->dev,
			"%s: enumeration failed\n",
			__func__);
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
		dev_err(mnh_pwr->dev,
			"%s: could not find mnh device\n",
			__func__);
		return -ENODEV;
	}

	mnh_pwr->pcidev = pcidev;

	return 0;
}

static int mnh_pwr_pcie_suspend(void)
{
	struct pci_dev *pcidev = mnh_pwr->pcidev;
	int ret;

	if (!pcidev)
		return -ENODEV;

	ret = pci_save_state(pcidev);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: pci_save_state failed (%d)\n",
			__func__, ret);
		return ret;
	}

	ret = msm_pcie_pm_control(MSM_PCIE_SUSPEND, pcidev->bus->number,
				  pcidev, NULL, PM_OPT_SUSPEND);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: msm_pcie_pm_control(SUSPEND) failed (%d)\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int mnh_pwr_pcie_resume(void)
{
	struct pci_dev *pcidev = mnh_pwr->pcidev;
	u32 val;
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

		pcidev = mnh_pwr->pcidev;
	} else {
		ret = msm_pcie_pm_control(MSM_PCIE_RESUME, pcidev->bus->number,
					  pcidev, NULL, PM_OPT_RESUME);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: msm_pcie_pm_control(RESUME) failed (%d)\n",
				__func__, ret);
		}

		ret = msm_pcie_recover_config(pcidev);
		if (ret) {
			dev_err(mnh_pwr->dev,
				"%s: msm_pcie_recover_config failed (%d)\n",
				__func__, ret);
		}
	}

	/* Disable L1 */
	ret = pci_read_config_dword(mnh_pwr->pcidev,
				   PCIE20_CAP_LINKCTRLSTATUS, &val);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: reading PCIE20_CAP_LINKCTRLSTATUS failed:%d\n",
			__func__, ret);
		return ret;
	}
	val &= DISABLE_PCIE_L1_MASK; /* disable bit 1 */
	dev_dbg(mnh_pwr->dev, "writing PCIE20_CAP_LINKCTRLSTATUS (val 0x%x)\n",
		 val);
	ret = pci_write_config_dword(mnh_pwr->pcidev,
				    PCIE20_CAP_LINKCTRLSTATUS, val);
	if (ret) {
		dev_err(mnh_pwr->dev,
			"%s: writing PCIE20_CAP_LINKCTRLSTATUS (val 0x%x) failed:%d\n",
			__func__, val, ret);
		return ret;
	}

	return 0;
}

static void mnh_pwr_down(void)
{
	/* assert ddr_pad_iso_n */
	gpiod_set_value_cansleep(mnh_pwr->ddr_pad_iso_n_pin, 1);

	/* suspend pcie link */
	mnh_pwr_pcie_suspend();

	/* deassert soc_pwr_good */
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 0);

	/* disable clocks */
	clk_disable_unprepare(mnh_pwr->ref_clk);
	clk_disable_unprepare(mnh_pwr->sleep_clk);

	/* disable supplies */
	regulator_disable(mnh_pwr->sdldo_supply);
	regulator_disable(mnh_pwr->sdsr_supply);
	if (mnh_pwr->state == MNH_PWR_S0) {
		regulator_disable(mnh_pwr->asr_supply);
		regulator_disable(mnh_pwr->ioldo_supply);
	}

	mnh_pwr->state = MNH_PWR_S4;
}

static void mnh_pwr_suspend(void)
{
	/* assert ddr_pad_iso_n */
	gpiod_set_value_cansleep(mnh_pwr->ddr_pad_iso_n_pin, 0);

	/* suspend pcie link */
	mnh_pwr_pcie_suspend();

	/* deassert soc_pwr_good */
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 0);

	/* disable clocks */
	clk_disable_unprepare(mnh_pwr->ref_clk);
	clk_disable_unprepare(mnh_pwr->sleep_clk);

	/* disable core supplies */
	regulator_disable(mnh_pwr->asr_supply);
	regulator_disable(mnh_pwr->ioldo_supply);

	mnh_pwr->state = MNH_PWR_S3;
}

static void mnh_pwr_up(void)
{
	int ret, timeout;

	/* enable supplies */
	ret = regulator_enable(mnh_pwr->ioldo_supply);
	ret = regulator_enable(mnh_pwr->asr_supply);
	if (mnh_pwr->state == MNH_PWR_S4) {
		ret = regulator_enable(mnh_pwr->sdldo_supply);
		ret = regulator_enable(mnh_pwr->sdsr_supply);
	}

	/* turn on clocks */
	ret = clk_prepare_enable(mnh_pwr->ref_clk);
	ret = clk_prepare_enable(mnh_pwr->sleep_clk);

	/* deassert ddr_pad_iso_n */
	gpiod_set_value_cansleep(mnh_pwr->ddr_pad_iso_n_pin, 1);

	/* assert soc_pwr_good */
	gpiod_set_value_cansleep(mnh_pwr->soc_pwr_good_pin, 1);

	/* wait for easel to assert ready */
	reinit_completion(&mnh_pwr->ready_complete);
	timeout = wait_for_completion_timeout(&mnh_pwr->ready_complete,
					      MNH_READY_TIMEOUT);
	if (timeout <= 0) {
		ret = (timeout == 0) ? -ETIMEDOUT : timeout;
		dev_err(mnh_pwr->dev,
			"error waiting for device to assert ready (%d)\n",
			ret);
		/* TODO: return and handle error */
	}

	/* resume pcie link */
	mnh_pwr_pcie_resume();

	mnh_pwr->state = MNH_PWR_S0;
}

static int mnh_pwr_get_resources(void)
{
	struct device *dev = mnh_pwr->dev;

	/* request supplies */
	mnh_pwr->asr_supply = devm_regulator_get(dev, "bcm15602_asr");
	if (IS_ERR(mnh_pwr->asr_supply)) {
		dev_err(dev, "%s: failed to get asr supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->asr_supply));
		return PTR_ERR(mnh_pwr->asr_supply);
	}

	mnh_pwr->sdsr_supply = devm_regulator_get(dev, "bcm15602_sdsr");
	if (IS_ERR(mnh_pwr->sdsr_supply)) {
		dev_err(dev, "%s: failed to get sdsr supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->sdsr_supply));
		return PTR_ERR(mnh_pwr->sdsr_supply);
	}

	mnh_pwr->ioldo_supply = devm_regulator_get(dev, "bcm15602_ioldo");
	if (IS_ERR(mnh_pwr->ioldo_supply)) {
		dev_err(dev, "%s: failed to get ioldo supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->ioldo_supply));
		return PTR_ERR(mnh_pwr->ioldo_supply);
	}

	mnh_pwr->sdldo_supply = devm_regulator_get(dev, "bcm15602_sdldo");
	if (IS_ERR(mnh_pwr->sdldo_supply)) {
		dev_err(dev, "%s: failed to get sdldo supply (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->sdldo_supply));
		return PTR_ERR(mnh_pwr->sdldo_supply);
	}

	/* request gpio descriptors */
	mnh_pwr->boot_mode_pin = devm_gpiod_get(dev, "boot-mode",
						   GPIOD_OUT_LOW);
	if (IS_ERR(mnh_pwr->boot_mode_pin)) {
		dev_err(dev, "%s: could not get boot_mode gpio (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->boot_mode_pin));
		return PTR_ERR(mnh_pwr->boot_mode_pin);
	}

	mnh_pwr->soc_pwr_good_pin = devm_gpiod_get(dev, "soc-pwr-good",
						   GPIOD_OUT_LOW);
	if (IS_ERR(mnh_pwr->soc_pwr_good_pin)) {
		dev_err(dev, "%s: could not get soc_pwr_good gpio (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->soc_pwr_good_pin));
		return PTR_ERR(mnh_pwr->soc_pwr_good_pin);
	}

	mnh_pwr->ddr_pad_iso_n_pin = devm_gpiod_get(dev, "ddr-pad-iso-n",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(mnh_pwr->ddr_pad_iso_n_pin)) {
		dev_err(dev, "%s: could not get ddr_pad_iso_n gpio (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->ddr_pad_iso_n_pin));
		return PTR_ERR(mnh_pwr->ddr_pad_iso_n_pin);
	}

	mnh_pwr->ready_pin = devm_gpiod_get(dev, "ready", GPIOD_IN);
	if (IS_ERR(mnh_pwr->ready_pin)) {
		dev_err(dev, "%s: could not get ready gpio (%ld)\n",
			__func__, PTR_ERR(mnh_pwr->ready_pin));
		return PTR_ERR(mnh_pwr->ready_pin);
	}

	/* request irqs */
	mnh_pwr->ready_irq = gpiod_to_irq(mnh_pwr->ready_pin);

	/* request clocks */
	mnh_pwr->ref_clk = devm_clk_get(dev, "ref_clk");
	if (IS_ERR(mnh_pwr->ref_clk)) {
		dev_err(dev, "%s: could not get ref clk (%ld)\n", __func__,
			PTR_ERR(mnh_pwr->ref_clk));
		return PTR_ERR(mnh_pwr->ref_clk);
	}

	mnh_pwr->sleep_clk = devm_clk_get(dev, "sleep_clk");
	if (IS_ERR(mnh_pwr->sleep_clk)) {
		dev_err(dev, "%s: could not get sleep clk (%ld)\n", __func__,
			PTR_ERR(mnh_pwr->sleep_clk));
		return PTR_ERR(mnh_pwr->sleep_clk);
	}

	return 0;
}

static irqreturn_t mnh_pwr_ready_irq_handler(int irq, void *cookie)
{
	struct mnh_pwr_data *_mnh_pwr = (struct mnh_pwr_data *)cookie;

	complete(&_mnh_pwr->ready_complete);

	return IRQ_HANDLED;
}

int mnh_pwr_set_state(enum mnh_pwr_state system_state)
{
	dev_info(mnh_pwr->dev, "%s req: %d, current: %d\n",
		 __func__, system_state, mnh_pwr_get_state());

	if (system_state != mnh_pwr->state) {
		switch (system_state) {
		case MNH_PWR_S0:
			mnh_pwr_up();
			break;
		case MNH_PWR_S3:
			mnh_pwr_suspend();
			break;
		case MNH_PWR_S4:
			mnh_pwr_down();
			break;
		default:
			dev_err(mnh_pwr->dev, "%s: invalid state %d\n",
				__func__, system_state);
			return -EINVAL;
			break;
		}

		dev_info(mnh_pwr->dev, "%s done with state: %d\n",
			 __func__, mnh_pwr_get_state());
	} else {
		dev_info(mnh_pwr->dev, "%s: no state change needed\n",
			 __func__);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mnh_pwr_set_state);

enum mnh_pwr_state mnh_pwr_get_state(void)
{
	return mnh_pwr->state;
}
EXPORT_SYMBOL_GPL(mnh_pwr_get_state);

int mnh_pwr_init(struct device *dev)
{
	int ret;

	/* allocate memory for mnh_pwr_data struct */
	mnh_pwr = devm_kzalloc(dev, sizeof(struct mnh_pwr_data), GFP_KERNEL);
	if (!mnh_pwr)
		return -ENOMEM;

	/* save a local copy of the device struct */
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

	/* request irq for ready pin */
	ret = devm_request_threaded_irq(dev, mnh_pwr->ready_irq, NULL,
					mnh_pwr_ready_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"easel-ready", mnh_pwr);

	/* initialize easel ready completion */
	init_completion(&mnh_pwr->ready_complete);

	/* TODO: temporarily disable until pcie resume sequence is finished */
#if 0
	/* power on the device, but do not resume pcie link */
	mnh_pwr_up();

	/* power down the device */
	mnh_pwr_down();
#endif

	return 0;
}
EXPORT_SYMBOL(mnh_pwr_init);
