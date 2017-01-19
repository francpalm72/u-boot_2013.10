/*
 * (C) Copyright 2007
 * Michael Schwingen, michael@schwingen.org
 *
 * hardware register definitions for the AcTux-2 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _ACTUX2_HW_H
#define _ACTUX2_HW_H

/* 0 = LED off,1 = green, 2 = red, 3 = orange */
#define ACTUX2_LED1(a)	writeb((a ? 2 : 0), IXP425_EXP_BUS_CS7_BASE_PHYS + 0)
#define ACTUX2_LED2(a)	writeb((a ? 2 : 0), IXP425_EXP_BUS_CS7_BASE_PHYS + 1)
#define ACTUX2_LED3(a)	writeb((a ? 0 : 2), IXP425_EXP_BUS_CS7_BASE_PHYS + 2)
#define ACTUX2_LED4(a)	writeb((a ? 0 : 2), IXP425_EXP_BUS_CS7_BASE_PHYS + 3)

#define ACTUX2_DBG_PORT	IXP425_EXP_BUS_CS5_BASE_PHYS
#define ACTUX2_BOARDREL	(readb(IXP425_EXP_BUS_CS6_BASE_PHYS) & 0x0F)
#define ACTUX2_OPTION	(readb(IXP425_EXP_BUS_CS6_BASE_PHYS) & 0xF0)

/*
 * GPIO settings
 */
#define CONFIG_SYS_GPIO_DBGINT			0
#define CONFIG_SYS_GPIO_ETHINT			1
#define CONFIG_SYS_GPIO_ETHRST			2	/* Out */
#define CONFIG_SYS_GPIO_LED5_GN		3	/* Out */
#define CONFIG_SYS_GPIO_UNUSED4		4
#define CONFIG_SYS_GPIO_UNUSED5		5
#define CONFIG_SYS_GPIO_DSR			6	/* Out */
#define CONFIG_SYS_GPIO_DCD			7	/* Out */
#define CONFIG_SYS_GPIO_IPAC_INT		8
#define CONFIG_SYS_GPIO_DBGJUMPER		9
#define CONFIG_SYS_GPIO_BUTTON1		10
#define CONFIG_SYS_GPIO_DBGSENSE		11
#define CONFIG_SYS_GPIO_DTR			12
#define CONFIG_SYS_GPIO_IORST			13	/* Out */
#define CONFIG_SYS_GPIO_PCI_CLK		14	/* Out */
#define CONFIG_SYS_GPIO_EXTBUS_CLK		15	/* Out */

#endif
