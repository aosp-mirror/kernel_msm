/* arch/arm/mach-msm/qdsp5v2/marimba.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

struct codec_reg {
	unsigned char addr, mask, val;
};

static struct codec_reg init_rx[] = {
	{ 0x23, 0xF8, 0x00 },
	{ 0x24, 0x6F, 0x00 },
	{ 0x25, 0x7F, 0x00 },
	{ 0x26, 0xFC, 0x00 },
	{ 0x28, 0xFE, 0x00 },
	{ 0x29, 0xFE, 0x00 },
	{ 0x33, 0xFF, 0x00 },
	{ 0x34, 0xFF, 0x00 },
	{ 0x35, 0xFC, 0x00 },
	{ 0x36, 0xFE, 0x00 },
	{ 0x37, 0xFE, 0x00 },
	{ 0x38, 0xFE, 0x00 },
	{ 0x39, 0xF0, 0x00 },
	{ 0x3A, 0xFF, 0x0A },
	{ 0x3B, 0xFC, 0xAC },
	{ 0x3C, 0xFC, 0xAC },
	{ 0x3D, 0xFF, 0x55 },
	{ 0x3E, 0xFF, 0x55 },
	{ 0x3F, 0xCF, 0x00 },
	{ 0x40, 0x3F, 0x00 },
	{ 0x41, 0x3F, 0x00 },
	{ 0x42, 0xFF, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x44, 0xF7, 0x00 },
	{ 0x45, 0xFF, 0x00 },
	{ 0x46, 0xFF, 0x00 },
	{ 0x47, 0xF7, 0x00 },
	{ 0x48, 0xF7, 0x00 },
	{ 0x49, 0xFF, 0x00 },
	{ 0x4A, 0xFF, 0x00 },
	{ 0x80, 0x02, 0x00 },
	{ 0x81, 0xFF, 0x4C },
	{ 0x83, 0x23, 0x00 },
	{ 0x84, 0xFF, 0xAC },
	{ 0x85, 0xFF, 0xAC },
	{ 0x88, 0xFF, 0xFF },
	{ 0x8A, 0x0F, 0x03 },
	{ 0x8B, 0xFF, 0xAC },
	{ 0x8C, 0x03, 0x01 },
	{ 0x8D, 0xFF, 0x00 },
	{ 0x8E, 0xFF, 0x00 },

/* lb regs */
	{ 0x2B, 0x8F, 0x02 },
	{ 0x2C, 0x8F, 0x02 },

	{ 0xFF, 0x00, 0x00 },
};

static struct codec_reg init_handset_48k_256_mono[] = {
	{ 0x80, 0x02, 0x02 },
	{ 0x80, 0x02, 0x00 },

	{ 0x24, 0x6F, 0x44 },
	{ 0x04, 0xFF, 0x8C },
	{ 0x81, 0xFF, 0x4e },
	{ 0x25, 0x0F, 0x0b },
	{ 0x26, 0xfc, 0xfc },
	{ 0x36, 0xc0, 0x80 },
	{ 0x3A, 0xFF, 0x2B },
	{ 0x23, 0xff, 0x20 },
	{ 0x3d, 0xFF, 0x55 },
	{ 0x83, 0x21, 0x21 },
	{ 0x33, 0x80, 0x80 },

	{ 0xFF, 0x00, 10 },

	{ 0x33, 0x40, 0x40 },
	{ 0x84, 0xff, 0x00 },
	{ 0x8A, 0x05, 0x04 },

	{ 0xFF, 0x00, 0x00 },
};
	
static struct codec_reg init_speaker_48k_256_stereo[] = {
	{ 0x80, 0x02, 0x02 },
	{ 0x80, 0x02, 0x00 },

	{ 0x24, 0x6F, 0x64 },
	{ 0x25, 0x0F, 0x0B },
	{ 0x26, 0xfc, 0xfc },
	{ 0x37, 0xe6, 0x80 },
	{ 0x3A, 0xFF, 0x2B },
	{ 0x3d, 0xFF, 0x55 },
	{ 0x83, 0x23, 0x23 },
	{ 0x23, 0xff, 0x20 },
	{ 0x33, 0x8a, 0x8a },
	{ 0x33, 0x05, 0x05 },

	{ 0xFF, 0x00, 30 },

	{ 0x84, 0xff, 0x03 },
	{ 0x85, 0xff, 0x03 },
	{ 0x8A, 0x0f, 0x0c },

	{ 0xFF, 0x00, 0x00 },
};


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/vreg.h>

static struct vreg *vreg_marimba1;
static struct vreg *vreg_marimba2;
static struct vreg *vreg_marimba3;

static int marimba_vreg_init(void)
{
	vreg_marimba1 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_marimba1))
		return PTR_ERR(vreg_marimba1);
	vreg_marimba2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba2))
		return PTR_ERR(vreg_marimba2);
	/* codec vreg */
	vreg_marimba3 = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_marimba3))
		return PTR_ERR(vreg_marimba3);
	return 0;
}

static void marimba_vreg_enable(void)
{
	vreg_enable(vreg_marimba1);
	vreg_enable(vreg_marimba2);
	vreg_enable(vreg_marimba3);
}

#define MARIMBA_ADDR_MARIMBA	0x0C
#define MARIMBA_ADDR_FM		0x2A
#define MARIMBA_ADDR_CDC	0x77
#define MARIMBA_ADDR_QMEMBIST	0X66

#define MARIMBA_REG_ID_FM	0x01
#define MARIMBA_REG_ID_CDC	0x02
#define MARIMBA_REG_ID_QMEMBIST	0x03
#define MARIMBA_REG_ID_TSADC	0x04

static int marimba_raw_write(struct i2c_client *client,
			     u8 addr, u8 reg, u8 value)
{
	struct i2c_msg msg;
	u8 data[2];
	int ret;

	msg.addr = addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;
	data[0] = reg;
	data[1] = value;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret != 1)
		pr_err("marimba_write: fail %d\n", ret);

	return ret;
}

static int marimba_raw_read(struct i2c_client *client, u8 addr, u8 reg)
{
	struct i2c_msg msg[2];
	u8 value;
	int ret;

	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &value;

	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret != 2)
		pr_err("marimba_read: fail %d\n", ret);

	if (ret == 2)
		return value;
	return ret;
}

static u8 marimba_shadow[256];

static int marimba_write(struct i2c_client *client, u8 reg, u8 value)
{
	marimba_shadow[reg] = value;
	return marimba_raw_write(client, client->addr, reg, value);
}

static int marimba_write_mask(struct i2c_client *client, u8 reg, u8 mask, u8 value)
{
	value = (marimba_shadow[reg] & (~mask)) | (value & mask);
	marimba_shadow[reg] = value;
	return marimba_raw_write(client, client->addr, reg, value);
}

static int marimba_read(struct i2c_client *client, u8 reg)
{
	return marimba_raw_read(client, client->addr, reg);
}


static struct i2c_client *marimba_client;

void adie_load(struct i2c_client *client, struct codec_reg *regs)
{
	int n;
	for (n = 0;; n++) {
		if (regs[n].addr == 0xff) {
			if (regs[n].val == 0)
				return;
			msleep(regs[n].val);
			continue;
		}
		marimba_write_mask(client, regs[n].addr,
				   regs[n].mask, regs[n].val);
	}
}

void adie_enable(void)
{
	struct i2c_client *client = marimba_client;

	marimba_vreg_enable();

	marimba_write(client, 0xff, 0x08); /* bring up codec */
	marimba_write(client, 0xff, 0x0a); /* GDFS_EN_FEW=1 */
	marimba_write(client, 0xff, 0x0e); /* GDFS_EN_REST=1 */
	marimba_write(client, 0xff, 0x07); /* RESET_N=1 */
	marimba_write(client, 0xff, 0x17); /* clock enable */
	marimba_write(client, 0x03, 0x04); /* enable band gap */
	marimba_write(client, 0x8F, 0x44); /* dither delay select, dmic gain bypass */

	msleep(100);

	adie_load(client, init_rx);
	adie_load(client, init_speaker_48k_256_stereo);
}

static int marimba_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;

	marimba_client = client;

	printk("*** marimba probe %p '%s' @ 0x%x ** *\n",
	       client, client->name, client->addr);

	/* 0x10 -> MARIMBA_MODE ?! */
	marimba_raw_write(client, MARIMBA_ADDR_MARIMBA, 0x00, 0x10);

	/* program address into marimba master device */
	ret = marimba_raw_write(client, MARIMBA_ADDR_MARIMBA,
				MARIMBA_REG_ID_CDC, client->addr);

	if (ret != 1) {
		pr_err("marimba_probe() cannot set address\n");
		return ret;
	}

	return 0;
}


static const struct i2c_device_id marimba_id[] = {
	{ "marimba-codec", 0 },
	{ }
};

static struct i2c_driver marimba_driver = {
	.probe		= marimba_probe,
	.id_table	= marimba_id,
	.driver		= {
		.name = "marimba",
	},
};


static int marimba_init(void)
{
	int ret;	
	ret = marimba_vreg_init();
	if (ret)
		return ret;
	return i2c_add_driver(&marimba_driver);
}

module_init(marimba_init);
