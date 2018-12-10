/*
 * iaxxx-sensor-tunnel.c -- functions related to sensor control though tunnel
 *
 * Copyright 2018 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-stream-registers.h>
#include <linux/mfd/adnc/iaxxx-channel-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-internal.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pad-ctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-gpio.h>
#include <linux/mfd/adnc/iaxxx-tunnel-intf.h>
#include <linux/uaccess.h>
#include "iaxxx.h"
#include "iaxxx-cdev.h"
#include "iaxxx-tunnel-priv.h"

#define ENCODING_Q23	0x17
#define ENCODING_AFLOAT	0x1
#define K8_10MS	0x140
#define RATE_8K	0x0
#define REDBOX_2_1	0x2
#define RETRY_COUNT	5
#define I2S3_HL_DEVIDER_CLK_PERIOD	0x16
#define I2S3_NR_DIVIDER_VALUE	0x3000001
#define I2S3_CLK_CFG_VALUE		0x1017F6
#define I2S3_CLK_CTRL_VALUE		0x103
#define I2S3_FS_ALIGN_VALUE		0x0
#define PORTD_IOCTRL_CFG_VALUE	0x2005
#define CNR0_CIC_RX_2_3_VALUE	0x3c0000
#define CNR0_HB_CIC_RX_3_POS	7
#define BUSY_SLEEP_RANGE	2000
#define BUSY_SLEEP_RANGE_MAX	2005

/*
 *[TODO]This is a first drop for testing flicker sensor.
 * Because the scenario is under discussing, It will refine later
 * b/113305927 will trace the progress.
 */
static int sensor_tunnel_route_setup(struct iaxxx_priv *priv,
				bool enable)
{
	int ret = 0;
	u32 status;
	int try = RETRY_COUNT;
	unsigned int port_id = PDM_PORTD, strm_id = 0x7, chn_id = 0xF;

	if (priv == NULL)
		return -EINVAL;
	if (enable) { /* Set up the route */
		regmap_update_bits(priv->regmap, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_2_MASK,
				(0x1 << IAXXX_GPIO_SWPORTB_DDR_COMMF_2_POS));
		regmap_update_bits(priv->regmap, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_2_MASK,
				(0x1 << IAXXX_GPIO_SWPORTB_DR_COMMF_2_POS));
		regmap_update_bits(priv->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << port_id), (0x1 << port_id));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			return -EIO;
		}
		regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
				(0x1 << PDM_PORTD), IAXXX_CNR0_I2S_ENABLE_LOW);
		regmap_update_bits(priv->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
				IAXXX_I2S_TRIGGER_HIGH);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_HL_ADDR(port_id), 0x0);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_HL_ADDR(port_id),
				I2S3_HL_DEVIDER_CLK_PERIOD);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_NR_ADDR(port_id), 0x0);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_NR_ADDR(port_id),
				I2S3_NR_DIVIDER_VALUE);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_GEN_CFG_ADDR(port_id),
				I2S3_CLK_CFG_VALUE);
		regmap_update_bits(priv->regmap,
				IAXXX_I2S_I2S_CLK_CTRL_ADDR(port_id),
				IAXXX_I2S_I2S0_CLK_CTRL_MASK_VAL,
				I2S3_CLK_CTRL_VALUE);
		regmap_write(priv->regmap, IAXXX_I2S_I2S_FS_ALIGN_ADDR(port_id),
				I2S3_FS_ALIGN_VALUE);
		regmap_update_bits(priv->regmap, IAXXX_AO_CLK_CFG_ADDR,
				IAXXX_AO_CLK_CFG_PORTD_CLK_OE_MASK,
				(1 << IAXXX_AO_CLK_CFG_PORTD_CLK_OE_POS));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
				(0x1 << PDM_PORTD), (0x1 << PDM_PORTD));
		regmap_update_bits(priv->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
				IAXXX_I2S_TRIGGER_HIGH);
		regmap_write(priv->regmap, IAXXX_IO_CTRL_PORTD_CLK_ADDR,
				PORTD_IOCTRL_CFG_VALUE);
		regmap_update_bits(priv->regmap,
				IAXXX_SRB_PDMI_PORT_PWR_EN_ADDR,
				(0x1 << PDM_PORTD), (0x1 << PDM_PORTD));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			return -EIO;
		}
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_HOS_ADDR,
				(0x1 << PDM_PORTD), (0x1 << PDM_PORTD));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_ADTL_CTRL_ADDR,
				IAXXX_CIC_ADTL_RX_MASK(port_id), 0);
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
				(1 << IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_M_3_MASK,
				CNR0_CIC_RX_2_3_VALUE);
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_CTRL_ADDR,
				(0x1 << port_id), (0x1 << port_id));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_HB_ADDR,
				IAXXX_CNR0_CIC_HB_CIC_RX_MASK(port_id),
				0x1 << CNR0_HB_CIC_RX_3_POS);
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_MASK,
				(0x1 <<
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_POS));
		regmap_update_bits(priv->regmap, IAXXX_IO_CTRL_PORTD_DI_ADDR,
				IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_MASK,
				(0x1 <<
				IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_POS));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
				(0 << IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS));

		do {
			usleep_range(BUSY_SLEEP_RANGE, BUSY_SLEEP_RANGE_MAX);
			regmap_read(priv->regmap,
				IAXXX_CNR0_CDC1_ENABLE_BUSY_ADDR, &status);
		} while (status && (try-- != 0));

		regmap_update_bits(priv->regmap, IAXXX_CNR0_CDC1_ENABLE_ADDR,
				(0x1 << PDM_PORTD), (0x1 << PDM_PORTD));

		try = RETRY_COUNT;
		do {
			usleep_range(BUSY_SLEEP_RANGE, BUSY_SLEEP_RANGE_MAX);
			regmap_read(priv->regmap,
				IAXXX_CNR0_CDC1_ENABLE_BUSY_ADDR, &status);
		} while (status && (try-- != 0));

		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			return -EIO;
		}

		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_CTRL_REG(strm_id),
				IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_MASK,
				REDBOX_2_1 <<
				IAXXX_STR_GRP_STR_CTRL_ASRC_MODE_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_SYNC_REG(strm_id),
				IAXXX_STR_GRP_STR_SYNC_MASTER_STR_MASK,
				strm_id);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_FORMAT_REG(strm_id),
				IAXXX_STR_GRP_STR_FORMAT_LENGTH_MASK,
				K8_10MS <<
				IAXXX_STR_GRP_STR_FORMAT_LENGTH_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_FORMAT_REG(strm_id),
				IAXXX_STR_GRP_STR_FORMAT_ENCODING_MASK,
				ENCODING_AFLOAT <<
				IAXXX_STR_GRP_STR_FORMAT_ENCODING_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_FORMAT_REG(strm_id),
				IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_MASK,
				RATE_8K  <<
				IAXXX_STR_GRP_STR_FORMAT_SAMPLE_RATE_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_PORT_REG(strm_id),
				IAXXX_STR_GRP_STR_PORT_PORT_MASK,
				IAXXX_SYSID_PDMI3 <<
				IAXXX_STR_GRP_STR_PORT_PORT_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_PORT_REG(strm_id),
				IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_MASK,
				ENCODING_Q23  <<
				IAXXX_STR_GRP_STR_PORT_PORT_ENCODING_POS);
		regmap_update_bits(priv->regmap,
				IAXXX_STR_GRP_STR_CHN_REG(strm_id),
				(0x1 << chn_id), (0x1 << chn_id));
		regmap_update_bits(priv->regmap, IAXXX_STR_HDR_STR_EN_ADDR,
				(0x1 << strm_id), (0x1 << strm_id));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			return -EIO;
		}
	} else {
		regmap_update_bits(priv->regmap, IAXXX_GPIO_SWPORTB_DDR_ADDR,
				IAXXX_GPIO_SWPORTB_DDR_COMMF_2_MASK,
				(0x0 << IAXXX_GPIO_SWPORTB_DDR_COMMF_2_POS));
		regmap_update_bits(priv->regmap, IAXXX_GPIO_SWPORTB_DR_ADDR,
				IAXXX_GPIO_SWPORTB_DR_COMMF_2_MASK,
				(0x0 << IAXXX_GPIO_SWPORTB_DR_COMMF_2_POS));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM route\n");
			return -EIO;
		}
		regmap_update_bits(priv->regmap, IAXXX_STR_HDR_STR_EN_ADDR,
				(0x1 << strm_id), 0x0);
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error in setting up PDM  route\n");
			return -EIO;
		}
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_2_3_ADDR,
				IAXXX_CNR0_CIC_RX_2_3_CLR_3_MASK,
				0x1 << IAXXX_CNR0_CIC_RX_2_3_CLR_3_POS);
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CIC_RX_RT_CTRL_ADDR,
				IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_MASK,
				(0x0 <<
				 IAXXX_CNR0_CIC_RX_RT_CTRL_CLK_EN_3_POS));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_CDC1_ENABLE_ADDR,
				(0x1 << PDM_PORTD), 0x0);
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error tearing up PDM  route\n");
			return -EIO;
		}
		regmap_update_bits(priv->regmap, IAXXX_AO_CLK_CFG_ADDR,
				IAXXX_AO_CLK_CFG_PORTD_CLK_OE_MASK,
				(0x0 << IAXXX_AO_CLK_CFG_PORTD_CLK_OE_POS));
		regmap_update_bits(priv->regmap, IAXXX_CNR0_I2S_ENABLE_ADDR,
				(0x1 << PDM_PORTD), IAXXX_CNR0_I2S_ENABLE_LOW);
		regmap_update_bits(priv->regmap, IAXXX_I2S_I2S_TRIGGER_GEN_ADDR,
				IAXXX_I2S_I2S_TRIGGER_GEN_WMASK_VAL,
				IAXXX_I2S_TRIGGER_HIGH);
		regmap_update_bits(priv->regmap, IAXXX_SRB_I2S_PORT_PWR_EN_ADDR,
				(0x1 << port_id), (0x0 << port_id));
		ret = iaxxx_send_update_block_request(priv->dev, &status,
				IAXXX_BLOCK_0);
		if (ret) {
			dev_err(priv->dev, "Error tearing up PDM  route\n");
			return -EIO;
		}
	}

	return 0;
}

static int sensor_tunnel_open(struct inode *inode, struct file *filp)
{
	int rc;

	rc = iaxxx_tunnel_open_common(inode, filp, TUNNEL_1);
	return rc;
}


static int sensor_tunnel_release(struct inode *inode, struct file *filp)
{
	int rc;

	rc = iaxxx_tunnel_release_common(inode, filp, TUNNEL_1);
	return rc;
}

static ssize_t sensor_tunnel_read(struct file *filp, char __user *buf,
				size_t count, loff_t *f_pos)
{
	return iaxxx_tunnel_read(filp, buf, count);
}

static long sensor_tunnel_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct iaxxx_tunnel_client * const client =
			(struct iaxxx_tunnel_client *)filp->private_data;
	struct iaxxx_tunnel_data *t_intf_priv =
			(struct iaxxx_tunnel_data *)client->tunnel_data;
	struct iaxxx_priv * const priv =
			(struct iaxxx_priv *)t_intf_priv->priv;
	struct tunlMsg msg;
	int ret = 0;

	if (!priv) {
		pr_err("Unable to fetch tunnel private data\n");
		return -EINVAL;
	}

	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "Chip state NULL\n");
		return -EINVAL;
	}

	if (arg != 0 && _IOC_DIR(cmd) == (_IOC_READ | _IOC_WRITE) &&
		_IOC_SIZE(cmd) == sizeof(struct tunlMsg)) {
		if (copy_from_user(&msg, (void __user *)arg,
					sizeof(struct tunlMsg))) {
			pr_err("parameter copy from user failed\n");
			return -EFAULT;
		}

		pr_debug("cmd: %x, TunlSrc: %x, tunlMode: %x, tunlEncode: %x",
			cmd, (uint32_t)msg.tunlSrc, (uint32_t)msg.tunlMode,
			(uint32_t)msg.tunlEncode);
	}

	switch (cmd) {
	case TUNNEL_SETUP:
		ret = sensor_tunnel_route_setup(priv, true);
		if (ret) {
			pr_err("Unable to setup sensor route\n");
				return -EIO;
		}
		ret = iaxxx_tunnel_setup(client, msg.tunlSrc, msg.tunlMode,
					msg.tunlEncode);
		if (ret) {
			pr_err("Unable to setup tunnel");
			return	-EINVAL;
		}
		break;
	case TUNNEL_TERMINATE:
		ret = iaxxx_tunnel_term(client, msg.tunlSrc, msg.tunlMode,
					msg.tunlEncode);
		if (ret) {
			pr_err("Unable to terminate tunnel");
			ret = -EINVAL;
		}
		ret = sensor_tunnel_route_setup(priv, false);
		if (ret) {
			pr_err("Unable to setup sensor route\n");
				return -EIO;
		}
		break;
	default:
		pr_err("Invalid ioctl command received %x", cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sensor_tunnel_compat_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	return sensor_tunnel_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations sensor_tunnel_fops = {
	.owner = THIS_MODULE,
	.open = sensor_tunnel_open,
	.read = sensor_tunnel_read,
	.unlocked_ioctl	= sensor_tunnel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= sensor_tunnel_compat_ioctl,
#endif
	.release = sensor_tunnel_release,
};

int iaxxx_sensor_tunnel_init(struct iaxxx_priv *priv)
{
	int err;
	struct iaxxx_tunnel_data *t_intf_priv;

	if (priv == NULL) {
		pr_err("priv is NULL in %s\n", __func__);
		return -EINVAL;
	}

	t_intf_priv = priv->tunnel_data;
	err = iaxxx_cdev_create(&t_intf_priv->tunnel_sensor_cdev,
				t_intf_priv->dev,
				&sensor_tunnel_fops, t_intf_priv,
				IAXXX_CDEV_SENSOR_TUNNEL_DEV);
	if (err) {
		pr_err("Error in creating the char device %s\n", __func__);
		return -EIO;
	}
	return 0;
}

int iaxxx_sensor_tunnel_exit(struct iaxxx_priv *priv)
{
	struct iaxxx_tunnel_data *t_intf_priv;

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer");
		return -EINVAL;
	}

	t_intf_priv = priv->tunnel_data;
	iaxxx_cdev_destroy(&t_intf_priv->tunnel_sensor_cdev);
	return 0;
}
