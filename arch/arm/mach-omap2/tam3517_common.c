/*
 * Copyright (C) 2012 Stefano Babic <sbabic@denx.de>
 *
 * Modified from mach-omap2/board-am3517evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/spi.h>
#include <linux/can/platform/ti_hecc.h>

#include <linux/smsc911x.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/usb.h>

#include "am35xx.h"

#include "am35xx-emac.h"
#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "common-board-devices.h"
#include "common.h"

/*
 * Default NAND partitions if not passed in command line
 * with mtdparts parameter
 */
static struct mtd_partition tam3517_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "SPL",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 8 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env1",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x180000 */
		.size		= 2 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env2",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1C0000 */
		.size		= 2 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x200000 */
		.size		= 48 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x800000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

/*
 * use fake regulator for vdds_dsi as we can't find this pin inside
 * AM3517 datasheet.
 */
static struct regulator_consumer_supply tam3517_vdds_dsi_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
};

static struct regulator_init_data tam3517_vdds_dsi = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tam3517_vdds_dsi_supply),
	.consumer_supplies	= tam3517_vdds_dsi_supply,
};

static struct fixed_voltage_config tam3517_display = {
	.supply_name		= "display",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &tam3517_vdds_dsi,
};

static struct platform_device tam3517_display_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data	= &tam3517_display,
	},
};

/* TPS65023 specific initialization */
/* VDCDC1 -> VDD_CORE */
static struct regulator_consumer_supply am3517_vdcdc1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

/* VDCDC2 -> VDDSHV */
static struct regulator_consumer_supply am3517_vdcdc2_supplies[] = {
	{
		.supply = "vddshv",
	},
};

/*
 * VDCDC2 |-> VDDS
 *	  |-> VDDS_SRAM_CORE_BG
 *	  |-> VDDS_SRAM_MPU
 */
static struct regulator_consumer_supply am3517_vdcdc3_supplies[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
};

/*
 * LDO1 |-> VDDA1P8V_USBPHY
 *	|-> VDDA_DAC
 */
static struct regulator_consumer_supply am3517_ldo1_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
	{
		.supply = "vdda_dac",
	},
};

/* LDO2 -> VDDA3P3V_USBPHY */
static struct regulator_consumer_supply am3517_ldo2_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};

static struct regulator_init_data tam3517_regulator_data[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_vdcdc1_supplies),
		.consumer_supplies = am3517_vdcdc1_supplies,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_vdcdc2_supplies),
		.consumer_supplies = am3517_vdcdc2_supplies,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_vdcdc3_supplies),
		.consumer_supplies = am3517_vdcdc3_supplies,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_ldo1_supplies),
		.consumer_supplies = am3517_ldo1_supplies,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_ldo2_supplies),
		.consumer_supplies = am3517_ldo2_supplies,
	},
};

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
/*
 * use fake regulator for tsc2046 vcc
 */
static struct regulator_consumer_supply tam3517_tsc2046_vcc_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
};

static struct regulator_init_data tam3517_tsc2046_vcc = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tam3517_tsc2046_vcc_supply),
	.consumer_supplies	= tam3517_tsc2046_vcc_supply,
};

static struct fixed_voltage_config tam3517_ts = {
	.supply_name		= "touchscreen",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &tam3517_tsc2046_vcc,
};

static struct platform_device tam3517_ts_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &tam3517_ts,
	},
};

static struct ads7846_platform_data tsc2046_config __initdata = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 30,
	.debounce_tol		= 10,
	.debounce_rep		= 1,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 100,
};
#endif

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || \
	defined(CONFIG_USB_EHCI_HCD_OMAP_MODULE)
#define USB_PHY1_RESET		25

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = USB_PHY1_RESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};
#endif

#if defined(CONFIG_MMC_OMAP_HS) || \
	defined(CONFIG_MMC_OMAP_HS)
#define SD_CARD_CD		126
static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd        = SD_CARD_CD,
		.gpio_wp        = -EINVAL,
	},
	{}      /* Terminator */
};
#endif

#if defined(CONFIG_CAN_TI_HECC) || \
	defined(CONFIG_CAN_TI_HECC_MODULE)
static struct resource am3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 24 + OMAP_INTC_START,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct ti_hecc_platform_data am3517_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
};

static struct platform_device am3517_hecc_device = {
	.name		= "ti_hecc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_hecc_resources),
	.resource	= am3517_hecc_resources,
	.dev		= {
		.platform_data = &am3517_hecc_pdata,
	},
};
#endif
