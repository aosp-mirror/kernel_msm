/*
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/board.h>

#include "devices.h"
#include "proc_comm.h"
#include "smd_private.h"

#include <asm/mach/flash.h>

#include "clock-pcom.h"

#include <mach/mmc.h>

static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart3[] = {
	{
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART3_PHYS,
		.end	= MSM_UART3_PHYS + MSM_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};

struct platform_device msm_device_uart3 = {
	.name	= "msm_serial",
	.id	= 2,
	.num_resources	= ARRAY_SIZE(resources_uart3),
	.resource	= resources_uart3,
};

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART1DM_RX,
		.end   = INT_UART1DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart2_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART2DM_RX,
		.end   = INT_UART2DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART2_TX_CHAN,
		.end   = DMOV_HSUART2_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART2_TX_CRCI,
		.end   = DMOV_HSUART2_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm2_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm2 = {
	.name = "msm_serial_hs",
	.id = 1,
	.num_resources = ARRAY_SIZE(msm_uart2_dm_resources),
	.resource = msm_uart2_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource resources_i2c[] = {
	{
		.start	= MSM_I2C_PHYS,
		.end	= MSM_I2C_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_i2c),
	.resource	= resources_i2c,
};

#define GPIO_I2C_CLK 70
#define GPIO_I2C_DAT 71
void msm_set_i2c_mux(bool gpio, int *gpio_clk, int *gpio_dat)
{
	unsigned id;
	if (gpio) {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		*gpio_clk = GPIO_I2C_CLK;
		*gpio_dat = GPIO_I2C_DAT;
	} else {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT , 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static struct resource resources_i2c2[] = {
	{
		.start	= MSM_I2C_2_PHYS,
		.end	= MSM_I2C_2_PHYS + MSM_I2C_2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C_2,
		.end	= INT_PWB_I2C_2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c2 = {
	.name		= "msm_i2c",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_i2c2),
	.resource	= resources_i2c2,
};

static struct resource resources_qup[] = {
	{
		.name   = "qup_phys_addr",
		.start	= MSM_QUP_PHYS,
		.end	= MSM_QUP_PHYS + MSM_QUP_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI_QUP_I2C_PHYS,
		.end	= MSM_GSBI_QUP_I2C_PHYS + MSM_GSBI_QUP_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "qup_in_intr",
		.start	= INT_PWB_QUP_IN,
		.end	= INT_PWB_QUP_IN,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_out_intr",
		.start	= INT_PWB_QUP_OUT,
		.end	= INT_PWB_QUP_OUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_err_intr",
		.start	= INT_PWB_QUP_ERR,
		.end	= INT_PWB_QUP_ERR,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_qup_i2c = {
	.name		= "qup_i2c",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_qup),
	.resource	= resources_qup,
};

static struct resource resources_hsusb[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct flash_platform_data msm_nand_data = {
	.parts		= NULL,
	.nr_parts	= 0,
};

static struct resource resources_nand[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.id	= -1,
};

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_PHYS,
		.end	= MSM_SDC1_PHYS + MSM_SDC1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
	{
		.start	= INT_SDC1_1,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_PHYS,
		.end	= MSM_SDC2_PHYS + MSM_SDC2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC2_1,
		.end	= INT_SDC2_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_PHYS,
		.end	= MSM_SDC3_PHYS + MSM_SDC3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC3_1,
		.end	= INT_SDC3_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_PHYS,
		.end	= MSM_SDC4_PHYS + MSM_SDC4_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC4_1,
		.end	= INT_SDC4_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags)
{
	struct platform_device	*pdev;
	struct resource *res;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "status_irq");
	if (!res)
		return -EINVAL;
	else if (stat_irq) {
		res->start = res->end = stat_irq;
		res->flags &= ~IORESOURCE_DISABLED;
		res->flags |= stat_irq_flags;
	}

	return platform_device_register(pdev);
}

static struct resource resources_mddi0[] = {
	{
		.start	= MSM_PMDH_PHYS,
		.end	= MSM_PMDH_PHYS + MSM_PMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_PRI,
		.end	= INT_MDDI_PRI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource resources_mddi1[] = {
	{
		.start	= MSM_EMDH_PHYS,
		.end	= MSM_EMDH_PHYS + MSM_EMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_EXT,
		.end	= INT_MDDI_EXT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mddi0 = {
	.name = "msm_mddi",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mddi0),
	.resource = resources_mddi0,
	.dev = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

struct platform_device msm_device_mddi1 = {
	.name = "msm_mddi",
	.id = 1,
	.num_resources = ARRAY_SIZE(resources_mddi1),
	.resource = resources_mddi1,
	.dev = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

static struct resource resources_mdp[] = {
	{
		.start	= MSM_MDP_PHYS,
		.end	= MSM_MDP_PHYS + MSM_MDP_SIZE - 1,
		.name	= "mdp",
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= INT_MDP,
		.end	= INT_MDP,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mdp = {
	.name = "msm_mdp",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mdp),
	.resource = resources_mdp,
};


static struct resource resources_ssbi_pmic[] = {
	{
		.start	= MSM_PMIC_SSBI_PHYS,
		.end	= MSM_PMIC_SSBI_PHYS + MSM_PMIC_SSBI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi_pmic = {
	.name		= "msm_ssbi",
	.id		= -1,
	.resource	= resources_ssbi_pmic,
	.num_resources	= ARRAY_SIZE(resources_ssbi_pmic),
};

static struct resource resources_spi[] = {
	{
		.start	= MSM_SPI_PHYS,
		.end	= MSM_SPI_PHYS + MSM_SPI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.name	= "irq_in",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.name	= "irq_out",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.name	= "irq_err",
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_spi = {
	.name		= "msm_spi",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_spi),
	.resource	= resources_spi,
};

static struct resource msm_vidc_720p_resources[] = {
	{
		.start	= 0xA3B00000,
		.end	= 0xA3B00000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MFC720,
		.end	= INT_MFC720,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_vidc_720p = {
	.name = "msm_vidc_720p",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_vidc_720p_resources),
	.resource = msm_vidc_720p_resources,
};

struct clk msm_clocks_7x30[] = {
	CLK_PCOM("adm_clk",	ADM_CLK,	NULL, 0),
	CLK_PCOM("adsp_clk",	ADSP_CLK,	NULL, 0),
	CLK_PCOM("cam_m_clk",	CAM_M_CLK,	NULL, 0),
	CLK_PCOM("ebi1_clk",	EBI1_CLK,	NULL, CLK_MIN | CLKFLAG_SHARED),
	CLK_PCOM("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLK_PCOM("emdh_clk",	EMDH_CLK,	&msm_device_mddi1.dev, OFF | CLK_MINMAX),
	CLK_PCOM("emdh_pclk",	EMDH_P_CLK,	&msm_device_mddi1.dev, OFF),
	CLK_PCOM("gp_clk",	GP_CLK,		NULL, 0),
	CLK_PCOM("grp_2d_clk",	GRP_2D_CLK,	NULL, 0),
	CLK_PCOM("grp_2d_pclk",	GRP_2D_P_CLK,	NULL, 0),
	CLK_PCOM("grp_clk",	GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("grp_pclk",	GRP_3D_P_CLK,	NULL, 0),
	CLK_7X30S("grp_src_clk", GRP_3D_SRC_CLK, GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("hdmi_clk",	HDMI_CLK,	NULL, 0),
	CLK_PCOM("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_PCOM("i2c_clk",	I2C_CLK,	&msm_device_i2c.dev, OFF),
	CLK_PCOM("i2c_clk",	I2C_2_CLK,	&msm_device_i2c2.dev, OFF),
	CLK_PCOM("jpeg_clk",	JPEG_CLK,	NULL, OFF),
	CLK_PCOM("jpeg_pclk",	JPEG_P_CLK,	NULL, OFF),
	CLK_PCOM("lpa_codec_clk",	LPA_CODEC_CLK,		NULL, 0),
	CLK_PCOM("lpa_core_clk",	LPA_CORE_CLK,		NULL, 0),
	CLK_PCOM("lpa_pclk",		LPA_P_CLK,		NULL, 0),
	CLK_PCOM("mdc_clk",	MDC_CLK,	NULL, 0),
	CLK_PCOM("mddi_clk",	PMDH_CLK,	&msm_device_mddi0.dev, OFF | CLK_MINMAX),
	CLK_PCOM("mddi_pclk",	PMDH_P_CLK,	&msm_device_mddi0.dev, OFF | CLK_MINMAX),
	CLK_PCOM("mdp_clk",	MDP_CLK,	&msm_device_mdp.dev, OFF),
	CLK_PCOM("mdp_pclk",	MDP_P_CLK,	&msm_device_mdp.dev, OFF),
	CLK_PCOM("lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, &msm_device_mdp.dev, 0),
	CLK_PCOM("lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, &msm_device_mdp.dev, 0),
	CLK_PCOM("mdp_vsync_clk",	MDP_VSYNC_CLK,  &msm_device_mdp.dev, 0),
	CLK_PCOM("mfc_clk",		MFC_CLK,		NULL, 0),
	CLK_PCOM("mfc_div2_clk",	MFC_DIV2_CLK,		NULL, 0),
	CLK_PCOM("mfc_pclk",		MFC_P_CLK,		NULL, 0),
	CLK_PCOM("mi2s_m_clk",		MI2S_M_CLK,  		NULL, 0),
	CLK_PCOM("mi2s_s_clk",		MI2S_S_CLK,  		NULL, 0),
	CLK_PCOM("mi2s_codec_rx_mclk",	MI2S_CODEC_RX_M_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_rx_sclk",	MI2S_CODEC_RX_S_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_tx_mclk",	MI2S_CODEC_TX_M_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_tx_sclk",	MI2S_CODEC_TX_S_CLK,  NULL, 0),
	CLK_PCOM("pbus_clk",	PBUS_CLK,	NULL, CLK_MIN),
	CLK_PCOM("pcm_clk",	PCM_CLK,	NULL, 0),
	CLK_PCOM("rotator_clk",	AXI_ROTATOR_CLK,		NULL, 0),
	CLK_PCOM("rotator_imem_clk",	ROTATOR_IMEM_CLK,	NULL, OFF),
	CLK_PCOM("rotator_pclk",	ROTATOR_P_CLK,		NULL, OFF),
	CLK_PCOM("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLK_PCOM("sdc_clk",	SDC1_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC1_P_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC2_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC2_P_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC3_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC3_P_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC4_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC4_P_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_PCOM("spi_clk",	SPI_CLK,	&msm_device_spi.dev, 0),
	CLK_PCOM("spi_pclk",	SPI_P_CLK,	&msm_device_spi.dev, 0),
	CLK_7X30S("tv_src_clk",	TV_CLK, 	TV_ENC_CLK,	NULL, 0),
	CLK_PCOM("tv_dac_clk",	TV_DAC_CLK,	NULL, 0),
	CLK_PCOM("tv_enc_clk",	TV_ENC_CLK,	NULL, 0),
	CLK_PCOM("uart_clk",	UART1_CLK,	&msm_device_uart1.dev, OFF),
	CLK_PCOM("uart_clk",	UART2_CLK,	&msm_device_uart2.dev, OFF),
	CLK_PCOM("uart_clk",	UART3_CLK,	&msm_device_uart3.dev, OFF),
	CLK_PCOM("uartdm_clk",	UART1DM_CLK,	&msm_device_uart_dm1.dev, OFF),
	CLK_PCOM("uartdm_clk",	UART2DM_CLK,	&msm_device_uart_dm2.dev, OFF),
	CLK_PCOM("usb_hs_clk",		USB_HS_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs_pclk",		USB_HS_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs_core_clk",	USB_HS_CORE_CLK,	&msm_device_hsusb.dev, OFF),
	CLK_PCOM("usb_hs2_clk",		USB_HS2_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs2_pclk",	USB_HS2_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs2_core_clk",	USB_HS2_CORE_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs3_clk",		USB_HS3_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs3_pclk",	USB_HS3_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs3_core_clk",	USB_HS3_CORE_CLK,	NULL, OFF),
	CLK_PCOM("vdc_clk",	VDC_CLK,	NULL, OFF | CLK_MIN),
	CLK_PCOM("vfe_camif_clk",	VFE_CAMIF_CLK, 	NULL, 0),
	CLK_PCOM("vfe_clk",	VFE_CLK,	NULL, 0),
	CLK_PCOM("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, 0),
	CLK_PCOM("vfe_pclk",	VFE_P_CLK,	NULL, OFF),
	CLK_PCOM("vpe_clk",	VPE_CLK,	NULL, 0),
	CLK_PCOM("qup_clk",	QUP_I2C_CLK,	&msm_device_qup_i2c.dev, OFF),
	CLK_PCOM("qup_pclk",	QUP_I2C_P_CLK,	&msm_device_qup_i2c.dev, OFF),

	/* 7x30 v2 hardware only. */
	CLK_PCOM("csi_clk",	CSI0_CLK,	NULL, 0),
	CLK_PCOM("csi_pclk",	CSI0_P_CLK,	NULL, 0),
	CLK_PCOM("csi_vfe_clk",	CSI0_VFE_CLK,	NULL, 0),
};

unsigned msm_num_clocks_7x30 = ARRAY_SIZE(msm_clocks_7x30);

