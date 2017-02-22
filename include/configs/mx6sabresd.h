/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QSABRESD_CONFIG_H
#define __MX6QSABRESD_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>


#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART3_BASE
#define CONFIG_CONSOLE_DEV	"ttymxc2"
#define CONFIG_MMCROOT		"/dev/mmcblk0p1"
#if defined(CONFIG_MX6Q)
#define CONFIG_DEFAULT_FDT_FILE	"imx6q-marta.dtb"
#elif defined(CONFIG_MX6DL)
#define CONFIG_DEFAULT_FDT_FILE	"imx6dl-marta.dtb"
#endif
#define PHYS_SDRAM_SIZE		(2u * 1024 * 1024 * 1024)

#include "mx6sabre_common.h"

//#define CONFIG_CMD_BMP
//#define CONFIG_VIDEO_LOGO
//#define CONFIG_VIDEO_BMP_LOGO

#define CONFIG_SYS_FSL_USDHC_NUM	3
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		1	/* 1=SDHC3 ; 2=SDHC4 */
#endif

/* Framebuffer */
//#define CONFIG_VIDEO
//#define CONFIG_VIDEO_IPUV3
//#define CONFIG_CFB_CONSOLE
//#define CONFIG_VGA_AS_SINGLE_DEVICE
//#define CONFIG_SYS_CONSOLE_IS_IN_ENV
//#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
//#define CONFIG_VIDEO_BMP_RLE8
//#define CONFIG_SPLASH_SCREEN
//#define CONFIG_SPLASH_SCREEN_ALIGN
//#define CONFIG_BMP_16BPP
//#define CONFIG_VIDEO_LOGO
//#define CONFIG_VIDEO_BMP_LOGO
//#define CONFIG_IPUV3_CLK 260000000
//#define CONFIG_IMX_HDMI

#ifdef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_EXTRA_ENV_SETTINGS
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
  CONFIG_MFG_ENV_SETTINGS \
  "fdt_addr=0x18000000\0" \
  "fdt_high=0xffffffff\0"   \
  "display=lvds0\0"\
  "panel=wvga-lvds0\0"\
  "video0=mxcfb0\0"\
  "video1=mxcfb1\0"\
  "lvds0=${video0}:dev=ldb,LDB-XGA,if=RGB666\0"\
  "lvds1=${video1}:dev=ldb,LDB-XGA,if=RGB666\0"\
  "mmc_recovery_partition=1\0"\
  "mmc_normal_partition=1\0"\
  "uimage=/boot/uImage\0"\
  "fdt=/boot/" CONFIG_DEFAULT_FDT_FILE "\0"\
  "console=" CONFIG_CONSOLE_DEV "\0"\
  "mmcdev=1\0"\
  "bootargs_recovery=console=ttymxc2,115200 root=/dev/mmcblk0p1 ro rootwait lpj=7905280 quiet\0"\
  "bootargs_normal=console=ttymxc2,115200 root=/dev/mmcblk0p1 ro rootwait lpj=7905280 quiet\0"\
  "bootcmd_reset=mw.b 0x20000000 0 0x10000; mmc write 0x20000000 0x0 0x10000; reset\0"\
  "bootcmd_normal=setenv bootargs ${bootargs_normal}; ext2load mmc ${mmcdev}:${mmc_normal_partition} ${loadaddr} ${uimage}; ext2load mmc ${mmcdev}:${mmc_normal_partition} ${fdt_addr} ${fdt}; clrlogo; bootm ${loadaddr} - ${fdt_addr}\0"\
  "bootcmd_recovery= setenv bootargs ${bootargs_recovery}; ext2load mmc ${mmcdev}:${mmc_recovery_partition} ${loadaddr} ${uimage}; ext2load mmc ${mmcdev}:${mmc_recovery_partition} ${fdt_addr} ${fdt}; clrlogo; bootm ${loadaddr} - ${fdt_addr}\0"\
  "bootcmd=run bootcmd_normal\0"

/* Ethernet */
#define CONFIG_PHY_MARVELL
#define CONFIG_IPADDR	192.168.168.200
#define CONFIG_ETHADDR 00:11:22:00:11:22
#undef CONFIG_CMD_DHCP

#endif                         /* __MX6QSABRESD_CONFIG_H */
