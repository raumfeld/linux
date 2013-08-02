/*
 *  linux/arch/arm/mach-pxa/pxa-dt.c
 *
 *  Copyright (C) 2012 Daniel Mack
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/irqs.h>
#include <mach/pxa3xx.h>
#include <plat/mfp.h>
#include <mach/mfp-pxa300.h>

#include "generic.h"


static mfp_cfg_t raumfeld_connector_pin_config[] __initdata = {
	/* UART1 */
	GPIO77_UART1_RXD,
	GPIO78_UART1_TXD,
	GPIO79_UART1_CTS,
	GPIO81_UART1_DSR,
	GPIO83_UART1_DTR,
	GPIO84_UART1_RTS,

	/* UART3 */
	GPIO110_UART3_RXD,

	/* USB Host */
	GPIO0_2_USBH_PEN,
	GPIO1_2_USBH_PWR,

	/* I2C */
	GPIO21_I2C_SCL | MFP_LPM_FLOAT | MFP_PULL_FLOAT,
	GPIO22_I2C_SDA | MFP_LPM_FLOAT | MFP_PULL_FLOAT,

	/* SPI */
	GPIO34_GPIO,	/* SPDIF_CS */
	GPIO96_GPIO,	/* MCLK_CS */
	GPIO125_GPIO,	/* ACCEL_CS */

	/* MMC */
	GPIO3_MMC1_DAT0,
	GPIO4_MMC1_DAT1,
	GPIO5_MMC1_DAT2,
	GPIO6_MMC1_DAT3,
	GPIO7_MMC1_CLK,
	GPIO8_MMC1_CMD,

	/* Ethernet */
	GPIO1_nCS2,			/* CS */
	GPIO40_GPIO | MFP_PULL_HIGH,	/* IRQ */

	/* SSP for I2S */
	GPIO85_SSP1_SCLK,
	GPIO89_SSP1_EXTCLK,
	GPIO86_SSP1_FRM,
	GPIO87_SSP1_TXD,
	GPIO88_SSP1_RXD,
	GPIO90_SSP1_SYSCLK,

	/* SSP2 for S/PDIF */
	GPIO25_SSP2_SCLK,
	GPIO26_SSP2_FRM,
	GPIO27_SSP2_TXD,
	GPIO29_SSP2_EXTCLK,
};

#ifdef CONFIG_PXA3xx
extern void __init pxa3xx_dt_init_irq(void);

static const struct of_dev_auxdata pxa3xx_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("mrvl,pxa-uart",		0x40100000, "pxa2xx-uart.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-uart",		0x40200000, "pxa2xx-uart.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-uart",		0x40700000, "pxa2xx-uart.2", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-uart",		0x41600000, "pxa2xx-uart.3", NULL),
	OF_DEV_AUXDATA("marvell,pxa-mmc",	0x41100000, "pxa2xx-mci.0", NULL),
	OF_DEV_AUXDATA("intel,pxa3xx-gpio",	0x40e00000, "pxa3xx-gpio", NULL),
	OF_DEV_AUXDATA("marvell,pxa-ohci",	0x4c000000, "pxa27x-ohci", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-i2c",		0x40301680, "pxa2xx-i2c.0", NULL),
	OF_DEV_AUXDATA("mrvl,pwri2c",		0x40f500c0, "pxa3xx-pwri2c.1", NULL),
	OF_DEV_AUXDATA("marvell,pxa3xx-nand",	0x43100000, "pxa3xx-nand", NULL),
	OF_DEV_AUXDATA("mrvl,pxa3xx-ssp",	0x41000000, "pxa3xx-ssp.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxa3xx-ssp",	0x41700000, "pxa3xx-ssp.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxa3xx-ssp",	0x41900000, "pxa3xx-ssp.2", NULL),
	OF_DEV_AUXDATA("mrvl,pxa3xx-ssp",	0x41a00000, "pxa3xx-ssp.3", NULL),
	{}
};

static void __init pxa3xx_dt_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table,
			     pxa3xx_auxdata_lookup, NULL);
	pxa3xx_mfp_config(ARRAY_AND_SIZE(raumfeld_connector_pin_config));
}

static const char *pxa3xx_dt_board_compat[] __initdata = {
	"marvell,pxa300",
	"marvell,pxa310",
	"marvell,pxa320",
	NULL,
};
#endif

#ifdef CONFIG_PXA3xx
DT_MACHINE_START(PXA_DT, "Marvell PXA3xx (Device Tree Support)")
	.map_io		= pxa3xx_map_io,
	.init_irq	= pxa3xx_dt_init_irq,
	.handle_irq	= pxa3xx_handle_irq,
	.init_time	= pxa_timer_init,
	.restart	= pxa_restart,
	.init_machine	= pxa3xx_dt_init,
	.dt_compat	= pxa3xx_dt_board_compat,
MACHINE_END
#endif
