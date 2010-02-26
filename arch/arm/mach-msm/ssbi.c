/* arch/arm/mach-msm/ssbi.c
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, Google Inc.
 *
 * Original authors: Code Aurura Forum
 *
 * Author: Dima Zavin <dima@android.com>
 *  - Largely rewritten from original to not be an i2c driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/msm_ssbi.h>
#include <mach/remote_spinlock.h>

/* SSBI 2.0 controller registers */
#define SSBI2_CTL			0x0000
#define SSBI2_RESET			0x0004
#define SSBI2_CMD			0x0008
#define SSBI2_RD			0x0010
#define SSBI2_STATUS			0x0014
#define SSBI2_PRIORITIES		0x0018
#define SSBI2_MODE2			0x001C

/* SSBI_CMD fields */
#define SSBI_CMD_SEND_TERM_SYM		(1 << 27)
#define SSBI_CMD_WAKEUP_SLAVE		(1 << 26)
#define SSBI_CMD_USE_ENABLE		(1 << 25)
#define SSBI_CMD_RDWRN			(1 << 24)

/* SSBI_STATUS fields */
#define SSBI_STATUS_DATA_IN		(1 << 4)
#define SSBI_STATUS_RD_CLOBBERED	(1 << 3)
#define SSBI_STATUS_RD_READY		(1 << 2)
#define SSBI_STATUS_READY		(1 << 1)
#define SSBI_STATUS_MCHN_BUSY		(1 << 0)

/* SSBI_RD fields */
#define SSBI_RD_USE_ENABLE		(1 << 25)
#define SSBI_RD_RDWRN			(1 << 24)

/* SSBI_MODE2 fields */
#define SSBI_MODE2_SSBI2_MODE		(1 << 0)

#define SSBI_TIMEOUT_US			100

struct msm_ssbi {
	struct device		*dev;
	struct device		*slave;
	void __iomem		*base;
	remote_spinlock_t	rspin_lock;
};

#define to_msm_ssbi(dev)	platform_get_drvdata(to_platform_device(dev))

static inline u32 ssbi_readl(struct msm_ssbi *ssbi, unsigned long reg)
{
	return readl(ssbi->base + reg);
}

static inline void ssbi_writel(struct msm_ssbi *ssbi, u32 val,
			       unsigned long reg)
{
	writel(val, ssbi->base + reg);
}

//poll_for_device_ready === SSBI_STATUS_READY
//poll_for_transfer_completed === SSBI_STATUS_MCHN_BUSY
//poll_for_read_completed === SSBI_STATUS_RD_READY
static int ssbi_wait_mask(struct msm_ssbi *ssbi, u32 set_mask, u32 clr_mask)
{
	u32 timeout = SSBI_TIMEOUT_US;
	u32 val;

	while (timeout--) {
		val = ssbi_readl(ssbi, SSBI2_STATUS);
		if (((val & set_mask) == set_mask) && ((val & clr_mask) == 0))
			return 0;
		udelay(1);
	}

	dev_err(ssbi->dev, "%s: timeout (status %x set_mask %x clr_mask %x)\n",
		__func__, ssbi_readl(ssbi, SSBI2_STATUS), set_mask, clr_mask);
	return -ETIMEDOUT;
}

int msm_ssbi_read(struct device *dev, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	unsigned long flags;
	u32 read_cmd = SSBI_CMD_RDWRN | ((addr & 0xff) << 16);
	u32 mode2;
	int ret = 0;

	BUG_ON(ssbi->dev != dev);

	remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);

	mode2 = ssbi_readl(ssbi, SSBI2_MODE2);
	if (mode2 & SSBI_MODE2_SSBI2_MODE) {
		mode2 = (mode2 & 0xf) | (((addr >> 8) & 0x7f) << 4);
		ssbi_writel(ssbi, mode2, SSBI2_MODE2);
	}

	while (len) {
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_READY, 0);
		if (ret)
			goto err;

		ssbi_writel(ssbi, read_cmd, SSBI2_CMD);
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_RD_READY, 0);
		if (ret)
			goto err;
		*buf++ = ssbi_readl(ssbi, SSBI2_RD) & 0xff;
		len--;
	}

err:
	remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	return ret;
}
EXPORT_SYMBOL(msm_ssbi_read);

int msm_ssbi_write(struct device *dev, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	unsigned long flags;
	u32 mode2;
	int ret = 0;

	BUG_ON(ssbi->dev != dev);

	remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);

	mode2 = readl(ssbi->base + SSBI2_MODE2);
	if (mode2 & SSBI_MODE2_SSBI2_MODE) {
		mode2 = (mode2 & 0xf) | (((addr >> 8) & 0x7f) << 4);
		ssbi_writel(ssbi, mode2, SSBI2_MODE2);
	}

	while (len) {
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_READY, 0);
		if (ret)
			goto err;

		ssbi_writel(ssbi, ((addr & 0xff) << 16) | *buf, SSBI2_CMD);
		ret = ssbi_wait_mask(ssbi, 0, SSBI_STATUS_MCHN_BUSY);
		if (ret)
			goto err;
		buf++;
		len--;
	}

err:
	remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	return ret;
}
EXPORT_SYMBOL(msm_ssbi_write);

static int __init msm_ssbi_add_slave(struct msm_ssbi *ssbi,
				     struct msm_ssbi_slave_info *slave)
{
	struct platform_device *slave_pdev;
	struct resource slave_irq_res;
	int ret;

	if (ssbi->slave) {
		pr_err("%s: slave already attached??\n", __func__);
		return -EBUSY;
	}

	slave_pdev = platform_device_alloc(slave->name, -1);
	if (!slave_pdev) {
		pr_err("%s: cannot allocate pdev for slave '%s'", __func__,
		       slave->name);
		ret = -ENOMEM;
		goto err;
	}

	slave_pdev->dev.parent = ssbi->dev;
	slave_pdev->dev.platform_data = slave->platform_data;

	memset(&slave_irq_res, 0, sizeof(struct resource));
	slave_irq_res.start = slave->irq;
	slave_irq_res.end = slave->irq;
	slave_irq_res.flags = IORESOURCE_IRQ;
	ret = platform_device_add_resources(slave_pdev, &slave_irq_res, 1);
	if (ret) {
		pr_err("%s: can't add irq resource for '%s'\n", __func__,
		       slave->name);
		goto err;
	}

	ret = platform_device_add(slave_pdev);
	if (ret) {
		pr_err("%s: cannot add slave platform device for '%s'\n",
		       __func__, slave->name);
		goto err;
	}

	ssbi->slave = &slave_pdev->dev;
	return 0;

err:
	if (slave_pdev)
		platform_device_put(slave_pdev);
	return ret;
}

static int __init msm_ssbi_probe(struct platform_device *pdev)
{
	struct msm_ssbi_platform_data *pdata = pdev->dev.platform_data;
	struct resource *mem_res;
	struct msm_ssbi *ssbi;
	int ret = 0;

	if (!pdata) {
		pr_err("%s: missing platform data\n", __func__);
		return -EINVAL;
	}

	ssbi = kzalloc(sizeof(struct msm_ssbi), GFP_KERNEL);
	if (!ssbi) {
		pr_err("%s: cannot allocate ssbi_data\n", __func__);
		return -ENOMEM;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		pr_err("%s: missing mem resource\n", __func__);
		ret = -EINVAL;
		goto err_get_mem_res;
	}

	ssbi->base = ioremap(mem_res->start, resource_size(mem_res));
	if (!ssbi->base) {
		pr_err("%s: ioremap of 0x%p failed\n", __func__,
		       (void *)mem_res->start);
		ret = -EINVAL;
		goto err_ioremap;
	}
	ssbi->dev = &pdev->dev;
	platform_set_drvdata(pdev, ssbi);

	ret = remote_spin_lock_init(&ssbi->rspin_lock, pdata->rspinlock_name);
	if (ret) {
		pr_err("%s: cannot init remote spinlock '%s'\n", __func__,
		       pdata->rspinlock_name);
		goto err_remote_spinlock_init;
	}

	ret = msm_ssbi_add_slave(ssbi, &pdata->slave);
	if (ret)
		goto err_ssbi_add_slave;

	pr_info("msm_ssbi: io=%08x rsl='%s'\n", mem_res->start,
		pdata->rspinlock_name);

	return 0;

err_remote_spinlock_init:
	platform_set_drvdata(pdev, NULL);
err_ssbi_add_slave:
	iounmap(ssbi->base);
err_ioremap:
err_get_mem_res:
	kfree(ssbi);
	return ret;
}

static struct platform_driver msm_ssbi_driver = {
	.probe		= msm_ssbi_probe,
	.driver		= {
		.name	= "msm_ssbi",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_ssbi_init(void)
{
	return platform_driver_register(&msm_ssbi_driver);
}

postcore_initcall(msm_ssbi_init);
