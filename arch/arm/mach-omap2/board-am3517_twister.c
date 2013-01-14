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
#include "tam3517_common.c"

#include <linux/serial_8250.h>
#include <linux/pwm_backlight.h>
#include <plat/pwm.h>
#include <plat/gpmc.h>

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
#define TAM3517_SMSC911X_IRQ_PIN  153
#define TAM3517_SMSC911X_CS    5
#define	TAM3517_SMSC911X_RESET_PIN	142

#include <linux/smsc911x.h>

#include "gpmc-smsc911x.h"

static struct omap_smsc911x_platform_data smsc911x_cfg = {
	.id		= 0,
	.cs             = TAM3517_SMSC911X_CS,
	.gpio_irq       = TAM3517_SMSC911X_IRQ_PIN,
	.gpio_reset     = TAM3517_SMSC911X_RESET_PIN,
        .flags          = SMSC911X_USE_16BIT,
};

static void __init tam3517_init_smsc911x(void)
{
        omap_mux_init_gpio(TAM3517_SMSC911X_IRQ_PIN,
		OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);
	gpmc_smsc911x_init(&smsc911x_cfg);
}

#else
static inline void __init tam3517_init_smsc911x(void) { return; }
#endif

#define PANEL_PWR_PIN		138
#define LCD_PANEL_PON_PIN	139
#define LCD_PANEL_BKLIGHT_PIN	53
#define DVI_PON_PIN		24

static int lcd_enabled;
static int dvi_enabled;

static int tam3517_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		pr_err("cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	gpio_set_value(LCD_PANEL_PON_PIN, 1);
	gpio_set_value(PANEL_PWR_PIN, 0);
	gpio_set_value(DVI_PON_PIN, 0);
	lcd_enabled = 1;

	return 0;
}

static void tam3517_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(PANEL_PWR_PIN, 1);
	gpio_set_value(LCD_PANEL_PON_PIN, 0);
	lcd_enabled = 0;
}

static struct panel_generic_dpi_data lcd_panel = {
	.name			= "thb-835",
	.platform_enable	= tam3517_panel_enable_lcd,
	.platform_disable	= tam3517_panel_disable_lcd,
};

static struct omap_dss_device tam3517_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "generic_dpi_panel",
	.data			= &lcd_panel,
	.phy.dpi.data_lines	= 24,
};

/*
 * TV Output
 */

static int tam3517_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void tam3517_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device tam3517_tv_device = {
	.type			= OMAP_DISPLAY_TYPE_VENC,
	.name			= "tv",
	.driver_name		= "venc",
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= tam3517_panel_enable_tv,
	.platform_disable	= tam3517_panel_disable_tv,
};

/*
 * DVI/HDMI Output
 */

static int tam3517_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		pr_err("cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}
	dvi_enabled = 1;
	gpio_set_value(DVI_PON_PIN, 1);
	return 0;
}

static void tam3517_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	dvi_enabled = 0;
	gpio_set_value(DVI_PON_PIN, 0);
}

static struct panel_generic_dpi_data dvi_panel = {
	.platform_enable	= tam3517_panel_enable_dvi,
	.platform_disable	= tam3517_panel_disable_dvi,
};
static struct omap_dss_device tam3517_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "dvi",
	.data			= &dvi_panel,
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *tam3517_dss_devices[] = {
	&tam3517_lcd_device,
	&tam3517_tv_device,
	&tam3517_dvi_device,
};

static struct omap_dss_board_info tam3517_dss_data = {
	.num_devices	= ARRAY_SIZE(tam3517_dss_devices),
	.devices	= tam3517_dss_devices,
	.default_device	= &tam3517_lcd_device,
};

static struct gpio tam3517_dss_gpios[] __initdata = {
	{ PANEL_PWR_PIN, GPIOF_OUT_INIT_HIGH, "panel_pwr"		},
	{ LCD_PANEL_PON_PIN, GPIOF_OUT_INIT_LOW, "lcd_power_pon"	},
	{ DVI_PON_PIN, GPIOF_OUT_INIT_LOW, "dvi_pon"			},
};

static void __init tam3517_display_init(void)
{
	int r;

	omap_mux_init_gpio(PANEL_PWR_PIN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(LCD_PANEL_PON_PIN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(DVI_PON_PIN, OMAP_PIN_OUTPUT);

	r = gpio_request_array(tam3517_dss_gpios, ARRAY_SIZE(tam3517_dss_gpios));
	if (r) {
		pr_err("failed to get DSS control GPIOs\n");
		return;
	}

	r = omap_display_init(&tam3517_dss_data);
	if (r) {
		pr_err("Failed to register DSS device\n");
		gpio_free_array(tam3517_dss_gpios, ARRAY_SIZE(tam3517_dss_gpios));
	}
}

static struct omap2_pwm_platform_config pwm_config = {
        .timer_id       = 9,    /* GPT9_PWM_EVT */
        .polarity       = 1     /* Active-high  */
};

static struct platform_device pwm_device = {
        .name           = "omap-pwm",
        .id             = 0,
        .dev = {
                .platform_data  = &pwm_config
        }
};

static struct platform_pwm_backlight_data tam3517_bl_data = {
        .pwm_id = 0,
        .pwm_period_ns  = 10000,
        .max_brightness = 100,
        .dft_brightness = 50,
};

static struct platform_device tam3517_backlight_device = {
        .name                   = "pwm-backlight",
        .id                     = -1,
        .dev                    = {
                .platform_data  = &tam3517_bl_data ,
        },
};

static struct i2c_board_info __initdata tam3517_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &tam3517_regulator_data[0],
	},
        {
                I2C_BOARD_INFO("24c02", 0x50),
        },
        {
                I2C_BOARD_INFO("tlv320aic23", 0x1a),
        },
};

static struct i2c_board_info __initdata tam3517_i2c2_devices[] = {
        {
                I2C_BOARD_INFO("24c02", 0x50),
        },
};

static struct i2c_board_info __initdata tam3517_i2c3_devices[] = {
        {
                I2C_BOARD_INFO("s35390a", 0x30),
                .type           = "s35390a",
        },
        {
                I2C_BOARD_INFO("ds1307", 0x68),
        },
};

static void __init tam3517_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, tam3517_i2c1_devices,
			ARRAY_SIZE(tam3517_i2c1_devices));
	omap_register_i2c_bus(2, 400, tam3517_i2c2_devices,
			ARRAY_SIZE(tam3517_i2c2_devices));
	omap_register_i2c_bus(3, 400, tam3517_i2c3_devices,
			ARRAY_SIZE(tam3517_i2c3_devices));
}

#define XR16L2571_PORTA_CS		1
#define XR16L2571_PORTB_CS		3
#define GPIO_XR16L2571_PORTA_IRQ_PIN	155
#define GPIO_XR16L2571_PORTB_IRQ_PIN	137

static struct plat_serial8250_port tam3517_xr16l2571_ports[] = {
	{
		.mapbase	= 0x21000000,
		.flags		= UPF_BOOT_AUTOCONF|UPF_IOREMAP|UPF_SHARE_IRQ,
		.irqflags	= IRQF_SHARED | IRQF_TRIGGER_HIGH,			\
		.iotype		= UPIO_MEM,
		.regshift	= 1,							\
		.uartclk	= 14745600,						\
	},
	{
		.mapbase	= 0x23000000,
		.flags		= UPF_BOOT_AUTOCONF|UPF_IOREMAP|UPF_SHARE_IRQ,
		.irqflags	= IRQF_SHARED | IRQF_TRIGGER_HIGH,			\
		.iotype		= UPIO_MEM,
		.regshift	= 1,							\
		.uartclk	= 14745600,						\
	}, {
		.flags		= 0
	}
};

static struct platform_device tam3517_xr16l2571_device =
{
        .name                   = "serial8250",
        .id                     = 3,
        .dev                    = {
                .platform_data  = &tam3517_xr16l2571_ports,
	},
};

static inline void __init tam3517_init_xr16l2571(void)
{
	unsigned long cs_mem_base;

	tam3517_xr16l2571_ports[0].irq = gpio_to_irq(GPIO_XR16L2571_PORTA_IRQ_PIN);
	tam3517_xr16l2571_ports[1].irq = gpio_to_irq(GPIO_XR16L2571_PORTB_IRQ_PIN);

	omap_mux_init_gpio(GPIO_XR16L2571_PORTA_IRQ_PIN, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_XR16L2571_PORTB_IRQ_PIN, OMAP_PIN_INPUT);
	if (gpmc_cs_request(XR16L2571_PORTA_CS, SZ_16M, &cs_mem_base) < 0)
	{
		printk(KERN_ERR "Failed request for GPMC mem for xr16l2571 porta\n");
		return;
	}
	tam3517_xr16l2571_ports[0].mapbase = cs_mem_base;

	if(gpio_request(GPIO_XR16L2571_PORTA_IRQ_PIN, "xr16l2571 porta irq"))
	{
		printk(KERN_ERR "Failed request gpio for xr16l2571 porta irq\n");
		return;
	}

	gpio_direction_input(GPIO_XR16L2571_PORTA_IRQ_PIN);

	if (gpmc_cs_request(XR16L2571_PORTB_CS, SZ_16M, &cs_mem_base) < 0)
	{
		printk(KERN_ERR "Failed request for GPMC mem for xr16l2571 portb\n");
		return;
	}
	tam3517_xr16l2571_ports[1].mapbase = cs_mem_base;

	if(gpio_request(GPIO_XR16L2571_PORTB_IRQ_PIN, "xr16l2571 portb irq"))
	{
		printk(KERN_ERR "Failed request gpio for xr16l2571 portb irq\n");
		return;
	}
	gpio_direction_input(GPIO_XR16L2571_PORTB_IRQ_PIN);

	platform_device_register(&tam3517_xr16l2571_device);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE4 ),
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PULL_UP),

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	/* Mux for GPIO Keys */
	OMAP3_MUX(MCBSP3_DX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(MCBSP3_DR, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(UART1_CTS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_WAIT1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_WAIT2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_WAIT3, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
#endif
#if defined(CONFIG_CAN_TI_HECC) || \
	defined(CONFIG_CAN_TI_HECC_MODULE)
	OMAP3_MUX(GPMC_A9, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
#endif
        /* PWM output for backlight */
        OMAP3_MUX(GPMC_NCS2, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct platform_device *tam3517_devices[] __initdata = {
	&tam3517_display_device,
	&tam3517_ts_device,
	&am3517_hecc_device,
	&pwm_device,
	&tam3517_backlight_device,
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button tam3517_gpio_buttons[] = {
        {
                .code                   = KEY_HOME,
                .gpio                   = 65,
                .desc                   = "home",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_ENTER,
                .gpio                   = 64,
                .desc                   = "enter",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_BACK,
                .gpio                   = 63,
                .desc                   = "back",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_MENU,
                .gpio                   = 150,
                .desc                   = "menu",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_BRIGHTNESSUP,
                .gpio                   = 143,
                .desc                   = "brightness up",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_BRIGHTNESSDOWN,
                .gpio                   = 142,
                .desc                   = "brightness down",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_VOLUMEUP,
                .gpio                   = 141,
                .desc                   = "volume up",
                .wakeup                 = 1,
                .active_low             = 1,
        }, {
                .code                   = KEY_VOLUMEDOWN,
                .gpio                   = 140,
                .desc                   = "volume down",
                .wakeup                 = 1,
                .active_low             = 1,
        },
};

static struct gpio_keys_platform_data tam3517_gpio_key_info = {
        .buttons        = tam3517_gpio_buttons,
        .nbuttons       = ARRAY_SIZE(tam3517_gpio_buttons),
};

static struct platform_device tam3517_keys_gpio = {
        .name   = "gpio-keys",
        .id     = -1,
        .dev    = {
                .platform_data  = &tam3517_gpio_key_info,
        },
};

static void __init tam3517_gpio_keys_init(void)
{
	platform_device_register(&tam3517_keys_gpio);
}
#else
static inline void __init tam3517_gpio_keys_init(void) { return; }
#endif


#define TS_IRQ_PIN	136

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	.mode		= MUSB_PERIPHERAL,
	.set_phy_power	= am35x_musb_phy_power,
	.clear_irq	= am35x_musb_clear_irq,
	.set_mode	= am35x_set_mode,
	.reset	= am35x_musb_reset,
};

static __init void am3517_musb_init(void)
{
	u32 devconf2;

	/* Set up USB clock/mode in the DEVCONF2 register. */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |= CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
		| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}

static void __init tam3517_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	tam3517_i2c_init();
	platform_add_devices(tam3517_devices, ARRAY_SIZE(tam3517_devices));
	omap_serial_init();
	omap_sdrc_init(NULL, NULL);

	tam3517_display_init();

	/* Configure EHCI ports */
	omap_mux_init_gpio(USB_PHY1_RESET, OMAP_PIN_OUTPUT);
	usbhs_init(&usbhs_bdata);

	/* NAND */
	omap_nand_flash_init(NAND_BUSWIDTH_16, tam3517_nand_partitions,
			     ARRAY_SIZE(tam3517_nand_partitions));

	/* touchscreen */
	omap_mux_init_gpio(TS_IRQ_PIN, OMAP_PIN_INPUT);
	omap_ads7846_init(1, TS_IRQ_PIN, 310, &tsc2046_config);

	/* Ethernet */
	am35xx_emac_init(AM35XX_DEFAULT_MDIO_FREQUENCY, 1);
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	tam3517_init_smsc911x();
#endif

	/*init gpio buttom*/
	tam3517_gpio_keys_init();

	/* init exar xr16l2571 dual port uart */
	tam3517_init_xr16l2571();

	/* MMC init */
	omap_hsmmc_init(mmc);

	/* MUSB init */
	am3517_musb_init();
}

static const char *tam3517_dt_match[] __initdata = {
	"technexion,tam3517",
	NULL
};

MACHINE_START(AM3517_TAM3517, "Technexion TAM3517 Evaluation Board")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= tam3517_init,
	.timer		= &omap3_timer,
	.dt_compat	= tam3517_dt_match,
	.restart	= omap_prcm_restart,
MACHINE_END
