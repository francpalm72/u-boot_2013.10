/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
//#include <micrel.h>
DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_22K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_120ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define OTG_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

//iomux_v3_cfg_t const otg_pads[] = {
//	MX6_PAD_KEY_COL4__USBOH3_USBOTG_OC | MUX_PAD_CTRL(OTG_PAD_CTRL),
//	MX6_PAD_EIM_D22__USBOH3_USBOTG_PWR | MUX_PAD_CTRL(OTG_PAD_CTRL),
//};
static void setup_iomux_otg(void)
{
	//gpio_direction_output(IMX_GPIO_NR(3, 22), 0);	//MX6_PAD_EIM_D22__USBOH3_USBOTG_PWR
	//gpio_direction_output(IMX_GPIO_NR(4, 14), 1);	//MX6_PAD_KEY_COL4__USBOH3_USBOTG_OC

	//gpio_set_value(IMX_GPIO_NR(3, 22), 1);		//MX6_PAD_EIM_D22__USBOH3_USBOTG_PWR

	//imx_iomux_v3_setup_multiple_pads(otg_pads, ARRAY_SIZE(otg_pads));
}


iomux_v3_cfg_t const uart3_pads[] = {
	MX6_PAD_EIM_D24__UART3_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D25__UART3_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads1[] = {
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),	//???????
	MX6_PAD_ENET_CRS_DV__GPIO_1_25		| MUX_PAD_CTRL(NO_PAD_CTRL),	//reset
	MX6_PAD_ENET_RXD1__GPIO_1_26		| MUX_PAD_CTRL(NO_PAD_CTRL),	//interrupt
	
	MX6_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	
	MX6_PAD_RGMII_RXC__ENET_RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__ENET_RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__ENET_RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__ENET_RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__ENET_RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));
	
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	mdelay(10);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
	udelay(100);
}

iomux_v3_cfg_t const usdhc2_pads[] = {
	
};

iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__USDHC3_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__USDHC3_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__USDHC3_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__USDHC3_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_RST__USDHC3_RST   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__USDHC4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__USDHC4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__USDHC4_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__USDHC4_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__USDHC4_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__USDHC4_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__USDHC4_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__USDHC4_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__USDHC4_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__USDHC4_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_ALE__USDHC4_RST | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	//{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

/*#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)*/
/*#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)*/

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	//case USDHC2_BASE_ADDR:
	//	ret = 0; /* USD2 is never present*/
	//	break;
	case USDHC3_BASE_ADDR:
		ret = 1;//!gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		//case 0:
			//imx_iomux_v3_setup_multiple_pads(
			//	usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			//gpio_direction_input(USDHC2_CD_GPIO);
			//usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			//break;
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			//gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}
#endif

//int mx6_rgmii_rework(struct phy_device *phydev)
//{
//	unsigned short val;
//
//	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
//	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);
//
//	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
//	val &= 0xffe3;
//	val |= 0x18;
//	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);
//
//	/* introduce tx clock delay */
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
//	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
//	val |= 0x0100;
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);
//
//	return 0;
//}
//
//int board_phy_config(struct phy_device *phydev)
//{
//	//mx6_rgmii_rework(phydev);
//
//	/* min rx data delay */
//	//ksz9031_phy_extended_write(phydev, 0x0, MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0, 0x0);
//	/* min tx data delay */
//	//ksz9031_phy_extended_write(phydev, 0x0, MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0, 0x0);
//	/* max rx/tx clock delay, min rx/tx control */
//	//ksz9031_phy_extended_write(phydev, 0x0, MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0x0, 0xf0f0);
//	
//	/* adjust KSZ9031 ethernet phy */
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x4);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000);
//
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x5);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000);
//
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x6);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x0000); //0xffff);
//
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x2);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x8);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0xc002);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0e, 0x3fff);
//	phy_write(phydev, MDIO_DEVAD_NONE, 0x0d, 0x0);
//
//	if (phydev->drv->config)
//		phydev->drv->config(phydev);
//	
//	return 0;
//}


#if defined(CONFIG_VIDEO_IPUV3)
struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= (IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT | IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT);
	writel(reg, &iomux->gpr[2]);
}

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK | IOMUXC_GPR2_LVDS_CH1_MODE_MASK);
	writel(reg, &iomux->gpr[2]);
}

static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static struct display_info_t const displays[] = 
{
	{
         .bus    = -1,
         .addr   = 0,
         .pixfmt = IPU_PIX_FMT_LVDS666,
         .detect = NULL,
         .enable = enable_lvds,
         .mode   = {
                .name           = "wvga-lvds0",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
		} 
	},
	{
         .bus    = -1,
         .addr   = 0,
         .pixfmt = IPU_PIX_FMT_LVDS666,
         .detect = NULL,
         .enable = enable_lvds,
         .mode   = {
                .name           = "wvga-lvds1",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
		} 
	},
	{
	.bus	= 3,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1920,
		.yres           = 1080,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
		} 
	}
};

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			printf("No panel detected: default to %s\n", panel);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}

static void setup_lvds_pin(void)
{
	//gpio_direction_output(IMX_GPIO_NR(2, 2), 0);	//MX6Q_PAD_NANDF_D2__GPIO_2_2,	/* F16_DPS_0 */
	//gpio_set_value(IMX_GPIO_NR(2, 3), 0);		//MX6Q_PAD_NANDF_D3__GPIO_2_3,	/* D17_FRC_0 */

	//gpio_direction_output(IMX_GPIO_NR(2, 5), 0);	//MX6Q_PAD_NANDF_D5__GPIO_2_5,	/* B18_DPS_1 */
	//gpio_set_value(IMX_GPIO_NR(2, 6), 0);		//MX6Q_PAD_NANDF_D6__GPIO_2_6,	/* E17_FRC_1 */

	//gpio_direction_output(IMX_GPIO_NR(6, 15), 0);	//Backlight Enable
	//gpio_set_value(IMX_GPIO_NR(6, 15), 0);		//Enable backlight
	
	return;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();

	imx_setup_hdmi();


	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);


	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);


	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);


	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);


	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0	//IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;	
	writel(reg, &iomux->gpr[2]);


	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;

	/* scan phy */
	phydev = phy_find_by_mask(bus, (0xFF), PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		return 0;
	}

	printf("using phy at: %d\n", phydev->addr);
	
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}

	printf("phy dev id: %X\n", phydev->phy_id);
	printf("phy drv name: %s\n", phydev->drv->name);
	printf("phy dev speed: %d\n", phydev->speed);


#endif

return 0;
}

int board_early_init_f(void)
{
	
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	printf("setup_display\n");
	setup_display();
	setup_lvds_pin();
#endif
	setup_iomux_otg();

	return 0;
}

int board_init(void)
{
	/* LCD Backlight Enable OFF*/
	//gpio_direction_output(IMX_GPIO_NR(5, 5), 0);
    //gpio_set_value(IMX_GPIO_NR(5, 5), 0); 

	/* Buzzer */
	//gpio_direction_output(IMX_GPIO_NR(5, 9), 0);
        //gpio_set_value(IMX_GPIO_NR(5, 9), 0); 

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	
	/* 5v enable AUX_5V_EN */
	//gpio_direction_output(IMX_GPIO_NR(6, 10), 1);
	//gpio_set_value(IMX_GPIO_NR(6, 10), 1);   
	
	/* 5v enable IO_PWR_EN */
	//gpio_direction_output(IMX_GPIO_NR(1, 30), 1);
	//gpio_set_value(IMX_GPIO_NR(1, 30), 1);   

	/* usb otg power enable */
	//gpio_direction_output(IMX_GPIO_NR(3, 22), 1);
    //gpio_set_value(IMX_GPIO_NR(3, 22), 1);   

	/* keyb0 supply output */
	//gpio_direction_output(IMX_GPIO_NR(4, 18), 1);
    //gpio_set_value(IMX_GPIO_NR(4, 18), 1);

	/* keyb1 supply output */
	//gpio_direction_output(IMX_GPIO_NR(4, 19), 0);
    //gpio_set_value(IMX_GPIO_NR(4, 19), 0); 

	/* Key0 */
	//gpio_direction_input(IMX_GPIO_NR(7, 1));
	/* Key1 */
	//gpio_direction_input(IMX_GPIO_NR(7, 0));
	/* Key2 */
	//gpio_direction_input(IMX_GPIO_NR(1, 10));
	/* Key3 */
	//gpio_direction_input(IMX_GPIO_NR(1, 6));
	/* Key4 */
	//gpio_direction_input(IMX_GPIO_NR(3, 20));
//GPIO OUTPUT
	gpio_direction_output(IMX_GPIO_NR(2, 27), 0); //SPARE_1_OUT_CPU - 59
	gpio_direction_output(IMX_GPIO_NR(2, 26), 0); //SPARE_2_OUT_CPU - 58
	gpio_direction_output(IMX_GPIO_NR(6, 6), 0); //SPARE_3_OUT_CPU - 166
	gpio_direction_output(IMX_GPIO_NR(2, 22), 0); //EASAU_CPU - 54
	gpio_direction_output(IMX_GPIO_NR(2, 21), 0); //EEO_CPU - 53
	gpio_direction_output(IMX_GPIO_NR(2, 20), 0); //ESS_CPU - 52
	gpio_direction_output(IMX_GPIO_NR(2, 19), 0); //EPA_CPU - 51
	gpio_direction_output(IMX_GPIO_NR(2, 18), 0); //EDB_CPU - 50
	gpio_direction_output(IMX_GPIO_NR(2, 17), 0); //EFB_CPU - 49
	gpio_direction_output(IMX_GPIO_NR(2, 16), 0); //EAB_CPU - 48
	gpio_direction_output(IMX_GPIO_NR(5, 4), 0); //INH_SORVOLO_CPU - 132
	gpio_direction_output(IMX_GPIO_NR(2, 30), 0); //EBT_CPU - 62
	gpio_direction_output(IMX_GPIO_NR(2, 31), 0); //INH_G_SWITCH_CPU - 63
	gpio_direction_output(IMX_GPIO_NR(5, 0), 0); //BIT_L_CPU - 128
	gpio_direction_output(IMX_GPIO_NR(2, 28), 0); //EBT_SK_CPU - 60
	gpio_direction_output(IMX_GPIO_NR(4, 16), 0); //SKR_PWR_CTR_CPU - 112
	gpio_direction_output(IMX_GPIO_NR(4, 29), 0); //CMD_CONSENSOFUOCO_CPU - 125
	gpio_direction_output(IMX_GPIO_NR(4, 30), 0); //CMD_FMP_INT_CPU - 126
	gpio_direction_output(IMX_GPIO_NR(5, 5), 0); //SENS_D00_CPU - 133
	gpio_direction_output(IMX_GPIO_NR(5, 6), 0); //SENS_D01_CPU - 134
	gpio_direction_output(IMX_GPIO_NR(5, 10), 0); //ID_00_CPU - 138
	gpio_direction_output(IMX_GPIO_NR(5, 11), 0); //ID_01_CPU - 139
	gpio_direction_output(IMX_GPIO_NR(4, 11), 0); //TFUEL_RANGE_SLC - 107
	gpio_direction_output(IMX_GPIO_NR(1, 0), 0); //MAINT_SK_CPU - 0
	gpio_direction_output(IMX_GPIO_NR(1, 9), 0); //SEL_ANT_CPU - 9
	gpio_direction_output(IMX_GPIO_NR(7, 11), 0); //RESET_L_CPU - 203
	gpio_direction_output(IMX_GPIO_NR(7, 12), 0); //CMD_SPIRA_CPU - 204
	gpio_direction_output(IMX_GPIO_NR(6, 9), 0); //IMX6_SPARE_LED - 169
	gpio_direction_output(IMX_GPIO_NR(2, 4), 0); //SAFE_SPOLETTA_CPU - 36
	gpio_direction_output(IMX_GPIO_NR(2, 5), 0); //ENABLE_DL_CPU - 37
	gpio_direction_output(IMX_GPIO_NR(2, 6), 0); //ESA_CPU - 38
	gpio_direction_output(IMX_GPIO_NR(2, 7), 0); //ESF_CPU - 39
	gpio_direction_output(IMX_GPIO_NR(1, 14), 0); //OK_CPU - 14
	gpio_direction_output(IMX_GPIO_NR(1, 13), 0); //GO_SW_CPU - 13

	gpio_direction_output(IMX_GPIO_NR(6, 5), 0); //IT_SL_1 - 165
	gpio_direction_output(IMX_GPIO_NR(5, 15), 0); //IT_SL_2 - 143
	gpio_direction_output(IMX_GPIO_NR(5, 16), 0); //IT_SL_3 - 144
	gpio_direction_output(IMX_GPIO_NR(5, 17), 0); //IT_SL_4 - 145
	gpio_direction_output(IMX_GPIO_NR(1, 24), 0); //IT_SL_5 - 24
	gpio_direction_output(IMX_GPIO_NR(1, 28), 0); //IT_SL_6 - 28
	gpio_direction_output(IMX_GPIO_NR(1, 27), 0); //IT_SL_7 - 27
	gpio_direction_output(IMX_GPIO_NR(1, 30), 0); //IT_SL_8 - 30
	gpio_direction_output(IMX_GPIO_NR(1, 29), 0); //IT_SL_10 - 29
	gpio_direction_output(IMX_GPIO_NR(1, 5), 0); //ARM_EVENTI - 5
	gpio_direction_output(IMX_GPIO_NR(5, 18), 0); //ARM_EVENTO - 146
	gpio_direction_output(IMX_GPIO_NR(5, 19), 0); //ARM_TRACE_CTL - 147
	gpio_direction_output(IMX_GPIO_NR(5, 20), 0); //ARM_TRACE_CLK - 148
	gpio_direction_output(IMX_GPIO_NR(5, 21), 0); //ARM_TRACE_00 - 149
	gpio_direction_output(IMX_GPIO_NR(5, 22), 0); //ARM_TRACE_01 - 150
	gpio_direction_output(IMX_GPIO_NR(5, 23), 0); //ARM_TRACE_02 - 151
	gpio_direction_output(IMX_GPIO_NR(5, 24), 0); //ARM_TRACE_03 - 152
	gpio_direction_output(IMX_GPIO_NR(5, 25), 0); //ARM_TRACE_04 - 153
	gpio_direction_output(IMX_GPIO_NR(5, 26), 0); //ARM_TRACE_05 - 154
	gpio_direction_output(IMX_GPIO_NR(5, 27), 0); //ARM_TRACE_06 - 155
	gpio_direction_output(IMX_GPIO_NR(5, 28), 0); //ARM_TRACE_07 - 156
	gpio_direction_output(IMX_GPIO_NR(5, 29), 0); //ARM_TRACE_08 - 157
	gpio_direction_output(IMX_GPIO_NR(5, 30), 0); //ARM_TRACE_09 - 158
	gpio_direction_output(IMX_GPIO_NR(5, 31), 0); //ARM_TRACE_10 - 159
	gpio_direction_output(IMX_GPIO_NR(6, 0), 0); //ARM_TRACE_11 - 160
	gpio_direction_output(IMX_GPIO_NR(6, 1), 0); //ARM_TRACE_12 - 161
	gpio_direction_output(IMX_GPIO_NR(6, 2), 0); //ARM_TRACE_13 - 162
	gpio_direction_output(IMX_GPIO_NR(6, 3), 0); //ARM_TRACE_14 - 163
	gpio_direction_output(IMX_GPIO_NR(6, 4), 0); //ARM_TRACE_15 - 164
	gpio_direction_output(IMX_GPIO_NR(4, 13), 0); //SPARE1_ARTIX7 - 109
	gpio_direction_output(IMX_GPIO_NR(6, 11), 0); //SPARE2_ARTIX7 - 171


//GPIO INPUT
	gpio_direction_input(IMX_GPIO_NR(2, 23)); //SPARE_1_IN_CPU - 55
	gpio_direction_input(IMX_GPIO_NR(2, 25)); //SPARE_2_IN_CPU - 57
	gpio_direction_input(IMX_GPIO_NR(2, 24)); //SPARE_3_IN_CPU - 56
	gpio_direction_input(IMX_GPIO_NR(5, 2)); //OK_TH_CPU - 130
	gpio_direction_input(IMX_GPIO_NR(3, 29)); //CK_PLUG_0_CPU - 93
	gpio_direction_input(IMX_GPIO_NR(3, 30)); //CK_PLUG_1_CPU - 94
	gpio_direction_input(IMX_GPIO_NR(6, 31)); //LOOPBACK_SIGNAL_CPU - 191
				
	gpio_direction_input(IMX_GPIO_NR(4, 18)); //OK_SPOLETTA_CPU - 114
	gpio_direction_input(IMX_GPIO_NR(4, 19)); //OK_DL_CPU - 115
	gpio_direction_input(IMX_GPIO_NR(4, 20)); //OK_PS_TH_CPU - 116
	gpio_direction_input(IMX_GPIO_NR(4, 17)); //OK_GPS_CPU - 113
	gpio_direction_input(IMX_GPIO_NR(4, 28)); //STATUS_ANT_CPU - 124
	gpio_direction_input(IMX_GPIO_NR(4, 31)); //OK_PS_SWRF_CPU - 127
	gpio_direction_input(IMX_GPIO_NR(5, 7)); //FUS_BT1_CPU - 135
	gpio_direction_input(IMX_GPIO_NR(5, 8)); //FUS_BT2_CPU - 136
	gpio_direction_input(IMX_GPIO_NR(5, 9)); //FAILSAFE_ENABLE_CPU - 137
	gpio_direction_input(IMX_GPIO_NR(5, 12)); //MAINT_SEL_CPU - 140
	gpio_direction_input(IMX_GPIO_NR(5, 13)); //OK_BT_SK_PSB_CPU - 141
	gpio_direction_input(IMX_GPIO_NR(5, 14)); //OK_PS_WCU_CPU - 142
				
	gpio_direction_input(IMX_GPIO_NR(4, 8)); //FUS_SK_CPU - 104
	gpio_direction_input(IMX_GPIO_NR(4, 10)); //PB_SX_CPU - 106
				
	gpio_direction_input(IMX_GPIO_NR(1, 2)); //TELLBACKBT1_CPU - 2
	gpio_direction_input(IMX_GPIO_NR(1, 4)); //CTR_TELLBACKBT2_CPU - 4
	gpio_direction_input(IMX_GPIO_NR(1, 7)); //CK_BT2_CPU - 7
	gpio_direction_input(IMX_GPIO_NR(1, 8)); //TB2_CPU - 8
	gpio_direction_input(IMX_GPIO_NR(7, 13)); //TELLBACKBT2_CPU - 205
	gpio_direction_input(IMX_GPIO_NR(4, 5)); //T0_OMB_CPU - 101
				
	gpio_direction_input(IMX_GPIO_NR(6, 14)); //TERMINATE_CMD_1_CPU - 174
	gpio_direction_input(IMX_GPIO_NR(6, 15)); //TERMINATE_CMD_2_CPU - 175
	gpio_direction_input(IMX_GPIO_NR(6, 16)); //SENSOR_CPU - 176
				
	gpio_direction_input(IMX_GPIO_NR(6, 10)); //CK_PRES_BT2_CPU - 170
	gpio_direction_input(IMX_GPIO_NR(2, 0)); //TIMER_SWITCH_CPU - 32
	gpio_direction_input(IMX_GPIO_NR(2, 1)); //STATO_D1_CPU - 33
	gpio_direction_input(IMX_GPIO_NR(2, 2)); //STATO_D2_CPU - 34
	gpio_direction_input(IMX_GPIO_NR(2, 3)); //PB_DX_CPU - 35
				
	gpio_direction_input(IMX_GPIO_NR(1, 11)); //GO_HW_CPU - 11
	gpio_direction_input(IMX_GPIO_NR(1, 10)); //GO_NOGO_MSL_CPU - 10
	gpio_direction_input(IMX_GPIO_NR(1, 15)); //GO_NOGO_UA_CPU - 15

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	//{"sd2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	/* 8 bit bus width */
	{"mmc0", MAKE_CFGVAL(0x60, 0x50, 0x00, 0x00)},
	{"mmc1", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6-Marta SDHC4\n");
	return 0;
}
