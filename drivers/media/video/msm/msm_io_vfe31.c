/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 *
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/clk.h>

#define CAMIF_CFG_RMSK             0x1fffff
#define CAM_SEL_BMSK               0x2
#define CAM_PCLK_SRC_SEL_BMSK      0x60000
#define CAM_PCLK_INVERT_BMSK       0x80000
#define CAM_PAD_REG_SW_RESET_BMSK  0x100000

#define EXT_CAM_HSYNC_POL_SEL_BMSK 0x10000
#define EXT_CAM_VSYNC_POL_SEL_BMSK 0x8000
#define MDDI_CLK_CHICKEN_BIT_BMSK  0x80

#define CAM_SEL_SHFT               0x1
#define CAM_PCLK_SRC_SEL_SHFT      0x11
#define CAM_PCLK_INVERT_SHFT       0x13
#define CAM_PAD_REG_SW_RESET_SHFT  0x14

#define EXT_CAM_HSYNC_POL_SEL_SHFT 0x10
#define EXT_CAM_VSYNC_POL_SEL_SHFT 0xF
#define MDDI_CLK_CHICKEN_BIT_SHFT  0x7

static struct clk *camio_vfe_mdc_clk;
static struct clk *camio_mdc_clk;
static struct clk *camio_vfe_clk;
static struct clk *camio_vfe_camif_clk;
static struct clk *camio_vfe_pbdg_clk;
static struct clk *camio_cam_m_clk;
static struct clk *camio_camif_pad_pbdg_clk;
static struct msm_camera_io_ext camio_ext;
static struct resource *camifpadio;
void __iomem *camifpadbase;

static void camif_io_w(u32 data)
{
	writel(data, camifpadbase);
	wmb();
}

static u32 camif_io_r(void)
{
	uint32_t data = readl(camifpadbase);
	rmb();
	return data;
}

int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		camio_vfe_mdc_clk = clk = clk_get(NULL, "vfe_mdc_clk");
		break;

	case CAMIO_MDC_CLK:
		camio_mdc_clk = clk = clk_get(NULL, "mdc_clk");
		break;

	case CAMIO_VFE_CLK:
		camio_vfe_clk = clk = clk_get(NULL, "vfe_clk");
		clk_set_rate(clk, 122880000);
		break;

	case CAMIO_VFE_CAMIF_CLK:
		camio_vfe_camif_clk = clk = clk_get(NULL, "vfe_camif_clk");
		break;

	case CAMIO_VFE_PBDG_CLK:
		camio_vfe_pbdg_clk = clk = clk_get(NULL, "vfe_pclk");
		break;

	case CAMIO_CAM_MCLK_CLK:
		camio_cam_m_clk = clk = clk_get(NULL, "cam_m_clk");
		clk_set_rate(clk, 24000000);
		break;

	case CAMIO_CAMIF_PAD_PBDG_CLK:
		camio_camif_pad_pbdg_clk = clk = clk_get(NULL, "camif_pad_pclk");
		break;

	default:
		break;
	}

	if (!IS_ERR(clk))
		clk_enable(clk);
	else
		rc = -1;
	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk;
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		break;

	case CAMIO_VFE_CAMIF_CLK:
		clk = camio_vfe_camif_clk;
		break;

	case CAMIO_VFE_PBDG_CLK:
		clk = camio_vfe_pbdg_clk;
		break;

	case CAMIO_CAM_MCLK_CLK:
		clk = camio_cam_m_clk;
		break;

	case CAMIO_CAMIF_PAD_PBDG_CLK:
		clk = camio_camif_pad_pbdg_clk;
		break;

	default:
		break;
	}

	if (!IS_ERR(clk)) {
		clk_disable(clk);
		clk_put(clk);
	} else
		rc = -1;

	return rc;
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_cam_m_clk;
	clk_set_rate(clk, rate);
}

int msm_camio_enable(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;

	camio_ext = camdev->ioext;

	camdev->camera_gpio_on();
	msm_camio_clk_enable(CAMIO_VFE_PBDG_CLK);
	msm_camio_clk_enable(CAMIO_CAMIF_PAD_PBDG_CLK);
	msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
	msm_camio_clk_enable(CAMIO_VFE_CLK);
	camifpadio = request_mem_region(camio_ext.camifpadphy,
		camio_ext.camifpadsz, pdev->name);
	if (!camifpadio) {
		rc = -EBUSY;
		goto common_fail;
	}
	camifpadbase = ioremap(camio_ext.camifpadphy, camio_ext.camifpadsz);
	if (!camifpadbase) {
		rc = -ENOMEM;
		goto parallel_busy;
	}
	msm_camio_clk_enable(CAMIO_VFE_CAMIF_CLK);
	return 0;

parallel_busy:
	release_mem_region(camio_ext.camifpadphy, camio_ext.camifpadsz);
common_fail:
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	msm_camio_clk_disable(CAMIO_CAMIF_PAD_PBDG_CLK);
	msm_camio_clk_disable(CAMIO_VFE_PBDG_CLK);
	camdev->camera_gpio_off();
	return rc;
}

void msm_camio_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;

	msm_camio_clk_disable(CAMIO_VFE_CAMIF_CLK);
	iounmap(camifpadbase);
	release_mem_region(camio_ext.camifpadphy, camio_ext.camifpadsz);
	CDBG("disable clocks\n");

	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	msm_camio_clk_disable(CAMIO_CAMIF_PAD_PBDG_CLK);
	msm_camio_clk_disable(CAMIO_VFE_PBDG_CLK);
	camdev->camera_gpio_off();
}

void msm_camio_camif_pad_reg_reset(void)
{
	uint32_t reg;

	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_INTERNAL);
	msleep(10);

	reg = camif_io_r() & CAMIF_CFG_RMSK;
	reg |= 0x3;
	camif_io_w(reg);
	msleep(10);

	reg = camif_io_r() & CAMIF_CFG_RMSK;
	reg |= 0x10;
	camif_io_w(reg);
	msleep(10);

	reg = camif_io_r() & CAMIF_CFG_RMSK;
	/* Need to be uninverted*/
	reg &= 0x03;
	camif_io_w(reg);
	msleep(10);
}

void msm_camio_vfe_blk_reset(void)
{
	return;
}

void msm_camio_clk_sel(enum msm_camio_clk_src_type srctype)
{
	if (camio_vfe_clk != NULL) {
		switch (srctype) {
		case MSM_CAMIO_CLK_SRC_INTERNAL:
			clk_set_flags(camio_vfe_clk, 0x00000100 << 1);
			break;

		case MSM_CAMIO_CLK_SRC_EXTERNAL:
			clk_set_flags(camio_vfe_clk, 0x00000100);
			break;

		default:
			break;
		}
	}
}
int msm_camio_probe_on(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_on();
	return msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
}

int msm_camio_probe_off(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_off();
	return msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
}
