/*
 * Copyright (C) 2011 Ilya Yanok, Emcraft Systems
 *
 * Modified from mach-omap2/board-mcx.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mmc/host.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/spi.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/uio_driver.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/mcspi.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/usb.h>

#include <mach/am35xx.h>

#include "am35xx-emac.h"
#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "common-board-devices.h"
#include "common.h"

/* FPGA */
#define FPGA_BASE_ADDR		0x20000000

static struct resource mt_ventoux_fpga_resource[] = {
	[0] = {
		.name = "FPGAONCS1",
		.start = FPGA_BASE_ADDR,
		.end   = FPGA_BASE_ADDR + SZ_128K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct uio_info mt_ventoux_fpga_data = {
	.name = "fpga",
	.version = "0.1",
	.irq = UIO_IRQ_NONE,
};

static struct platform_device mt_ventoux_fpga_device = {
	.name = "uio_pdrv",
	.id = 0,
	.dev = {
		.platform_data = &mt_ventoux_fpga_data,
	},
	.resource = mt_ventoux_fpga_resource,
	.num_resources = ARRAY_SIZE(mt_ventoux_fpga_resource),
};

#define MCX_MDIO_FREQUENCY	(1000000)

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_HOST,
	.power                  = 500,
	.set_phy_power		= am35x_musb_phy_power,
	.clear_irq		= am35x_musb_clear_irq,
	.set_mode		= am35x_set_mode,
	.reset			= am35x_musb_reset,
};

#define USB2_PWR_EN		127
static __init void mt_ventoux_musb_init(void)
{
	u32 devconf2;
	int err;

	omap_mux_init_gpio(USB2_PWR_EN, OMAP_PIN_OUTPUT);
	err = gpio_request_one(USB2_PWR_EN, GPIOF_OUT_INIT_HIGH, "musb-pwr");
	if (err < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d\n",
			USB2_PWR_EN);
		return;
	}

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}

static struct mtd_partition mt_ventoux_nand_partitions[] = {
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

#define PANEL_PWR_PIN		138
#define LCD_PANEL_PON_PIN	139

static int lcd_enabled;

static struct gpio mt_ventoux_dss_gpios[] __initdata = {
	{ PANEL_PWR_PIN, GPIOF_OUT_INIT_LOW, "panel_pwr"		},
	{ LCD_PANEL_PON_PIN, GPIOF_OUT_INIT_HIGH, "lcd_power_pon"	},
};

static int mt_ventoux_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_PON_PIN, 1);
	gpio_set_value(PANEL_PWR_PIN, 0);
	lcd_enabled = 1;

	return 0;
}

static void mt_ventoux_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(PANEL_PWR_PIN, 1);
	gpio_set_value(LCD_PANEL_PON_PIN, 0);
	lcd_enabled = 0;
}

static struct panel_generic_dpi_data chimei_lcd_panel = {
	.name			= "chimei_G070Y2-L01",
	.platform_enable	= mt_ventoux_panel_enable_lcd,
	.platform_disable	= mt_ventoux_panel_disable_lcd,
};

static struct panel_generic_dpi_data ortustech_lcd_panel = {
	.name			= "ortustech_com43h4m10xtc",
	.platform_enable	= mt_ventoux_panel_enable_lcd,
	.platform_disable	= mt_ventoux_panel_disable_lcd,
};

static struct panel_generic_dpi_data *mt_ventoux_panels[] = {
	&ortustech_lcd_panel,
	&chimei_lcd_panel
};

static struct omap_dss_device mt_ventoux_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "generic_dpi_panel",
	.data			= &chimei_lcd_panel,
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *mt_ventoux_dss_devices[] = {
	&mt_ventoux_lcd_device,
};

static struct omap_dss_board_info mt_ventoux_dss_data = {
	.num_devices	= ARRAY_SIZE(mt_ventoux_dss_devices),
	.devices	= mt_ventoux_dss_devices,
	.default_device	= &mt_ventoux_lcd_device,
};

static int __init mt_ventoux_panel_setup(char *str)
{
	int i;

	if (!str)
		return 1;

	for (i = 0; i < ARRAY_SIZE(mt_ventoux_panels); i++) {
		if (!strcmp(str, mt_ventoux_panels[i]->name))
			break;
	}

	if (i == ARRAY_SIZE(mt_ventoux_panels))
		i = 0;

	mt_ventoux_lcd_device.data = mt_ventoux_panels[i];

	printk("Panel=%s\n", mt_ventoux_panels[i]->name);

	return 1;
}

__setup("panel=", mt_ventoux_panel_setup);

/*
 * use fake regulator for vdds_dsi as we can't find this pin inside
 * AM3517 datasheet.
 */
static struct regulator_consumer_supply mt_ventoux_vdds_dsi_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
};

static struct regulator_init_data mt_ventoux_vdds_dsi = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mt_ventoux_vdds_dsi_supply),
	.consumer_supplies	= mt_ventoux_vdds_dsi_supply,
};

static struct fixed_voltage_config mt_ventoux_display = {
	.supply_name		= "display",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &mt_ventoux_vdds_dsi,
};

static struct platform_device mt_ventoux_display_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data	= &mt_ventoux_display,
	},
};

static void __init mt_ventoux_display_init(void)
{
	int r;

	omap_mux_init_gpio(PANEL_PWR_PIN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(LCD_PANEL_PON_PIN, OMAP_PIN_OUTPUT);

	r = gpio_request_array(mt_ventoux_dss_gpios, ARRAY_SIZE(mt_ventoux_dss_gpios));
	if (r) {
		pr_err("failed to get DSS control GPIOs\n");
		return;
	}

	r = omap_display_init(&mt_ventoux_dss_data);
	if (r) {
		pr_err("Failed to register DSS device\n");
		gpio_free_array(mt_ventoux_dss_gpios, ARRAY_SIZE(mt_ventoux_dss_gpios));
	}
}

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

static struct regulator_init_data mt_ventoux_regulator_data[] = {
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

static struct i2c_board_info __initdata mt_ventoux_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &mt_ventoux_regulator_data[0],
	},
        {
                I2C_BOARD_INFO("24c02", 0x50),
        },
};

static struct i2c_board_info __initdata mt_ventoux_i2c2_devices[] = {
        {
                I2C_BOARD_INFO("24c02", 0x50),
        },
};

static struct i2c_board_info __initdata mt_ventoux_i2c3_devices[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
};

static void __init mt_ventoux_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, mt_ventoux_i2c1_devices,
			ARRAY_SIZE(mt_ventoux_i2c1_devices));
	omap_register_i2c_bus(2, 400, mt_ventoux_i2c2_devices,
			ARRAY_SIZE(mt_ventoux_i2c2_devices));
	omap_register_i2c_bus(3, 400, mt_ventoux_i2c3_devices,
			ARRAY_SIZE(mt_ventoux_i2c3_devices));
}

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
/*
 * use fake regulator for tsc2046 vcc
 */
static struct regulator_consumer_supply mt_ventoux_tsc2046_vcc_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
};

static struct regulator_init_data mt_ventoux_tsc2046_vcc = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mt_ventoux_tsc2046_vcc_supply),
	.consumer_supplies	= mt_ventoux_tsc2046_vcc_supply,
};

static struct fixed_voltage_config mt_ventoux_ts = {
	.supply_name		= "touchscreen",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &mt_ventoux_tsc2046_vcc,
};

static struct platform_device mt_ventoux_ts_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &mt_ventoux_ts,
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

#ifdef CONFIG_SENSORS_ADCXX
static struct omap2_mcspi_device_config adc128s_mcspi_config = {
	.turbo_mode	= 0,
};

static struct spi_board_info adc128s_spi_board_info __initdata = {
	.modalias		= "adcxx8s",
	.bus_num		= 2,
	.chip_select		= 1,
	.max_speed_hz		= 1600000,
	.controller_data	= &adc128s_mcspi_config,
	.irq			= -EINVAL,
};

static inline void adc128s_init(void)
{
	spi_register_board_info(&adc128s_spi_board_info, 1);
}
#else
static inline void adc128s_init(void)
{
}
#endif

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE4 ),
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PULL_UP),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct resource am3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
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

static struct platform_device *mt_ventoux_devices[] __initdata = {
	&mt_ventoux_display_device,
	&mt_ventoux_ts_device,
	&mt_ventoux_fpga_device,
	&am3517_hecc_device,
};

#define TS_IRQ_PIN	163

static void __init mt_ventoux_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	mt_ventoux_i2c_init();
	platform_add_devices(mt_ventoux_devices, ARRAY_SIZE(mt_ventoux_devices));
	omap_serial_init();
	omap_sdrc_init(NULL, NULL);

	mt_ventoux_display_init();

	/* Configure EHCI ports */
	omap_mux_init_gpio(USB_PHY1_RESET, OMAP_PIN_OUTPUT);
	usbhs_init(&usbhs_bdata);

	/* NAND */
	omap_nand_flash_init(NAND_BUSWIDTH_16, mt_ventoux_nand_partitions,
			     ARRAY_SIZE(mt_ventoux_nand_partitions));

	/* touchscreen */
	omap_mux_init_gpio(TS_IRQ_PIN, OMAP_PIN_INPUT);
	omap_ads7846_init(1, TS_IRQ_PIN, 310, &tsc2046_config);

	/* Ethernet */
	am35xx_emac_init(MCX_MDIO_FREQUENCY, 1);

	/* MMC init */
	omap_hsmmc_init(mmc);

	/* ADC128S022 init */
	adc128s_init();

	/* MUSB : USB-2 */
	mt_ventoux_musb_init();
}

static const char *mt_ventoux_dt_match[] __initdata = {
	"teejet,mt_ventoux",
	NULL
};

MACHINE_START(AM3517_MT_VENTOUX, "TeeJet MT_VENTOUX")
	/* Maintainer: Ilya Yanok */
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= mt_ventoux_init,
	.init_late	= am35xx_init_late,
	.timer		= &omap3_timer,
	.dt_compat	= mt_ventoux_dt_match,
	.restart	= omap_prcm_restart,
MACHINE_END
