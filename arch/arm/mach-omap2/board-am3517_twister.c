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
#include <linux/input/edt-ft5x06.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c-mux.h>
#include <linux/i2c/at24.h>

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
#define TAM3517_SMSC911X_IRQ_GPIO_PATH "sdmmc1_clk.gpio_120"
#define TAM3517_SMSC911X_CS    5
#define	TAM3517_SMSC911X_RESET_PIN	142

#include <linux/smsc911x.h>

#include "gpmc-smsc911x.h"

static struct omap_smsc911x_platform_data smsc911x_cfg = {
	.id		= 0,
	.cs             = TAM3517_SMSC911X_CS,
	.gpio_irq       = -1,   /* will be initialized in `tam3517_init_smsc911x()' */
	.gpio_reset     = TAM3517_SMSC911X_RESET_PIN,
        .flags          = SMSC911X_USE_16BIT,
};

static void __init tam3517_init_smsc911x(void)
{
        struct omap_mux *mux = NULL;
        struct omap_mux_partition *partition = NULL;

        int mux_mode = omap_mux_get_by_name(TAM3517_SMSC911X_IRQ_GPIO_PATH, &partition, &mux);

        if (mux_mode < 0) {
          pr_err("%s: failed to find GPIO path \"%s\"\n", __func__, TAM3517_SMSC911X_IRQ_GPIO_PATH);
          return;
        }

        smsc911x_cfg.gpio_irq = mux->gpio;
        omap_mux_init_signal(TAM3517_SMSC911X_IRQ_GPIO_PATH, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);

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

	lcd_enabled = 1;

	return 0;
}

static void tam3517_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	lcd_enabled = 0;
}

static struct panel_generic_dpi_data lcd_panel = {
	.name			= "edt_etm0700g0dh6",
	.platform_enable	= tam3517_panel_enable_lcd,
	.platform_disable	= tam3517_panel_disable_lcd,
};

static struct omap_dss_device tam3517_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "generic_dpi_panel",
	.data			= &lcd_panel,
	.phy.dpi.data_lines	= 18,
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
};

static struct omap_dss_board_info tam3517_dss_data = {
	.num_devices	= ARRAY_SIZE(tam3517_dss_devices),
	.devices	= tam3517_dss_devices,
	.default_device	= &tam3517_lcd_device,
};

static struct gpio tam3517_dss_gpios[] __initdata = {
};

static void __init tam3517_display_init(void)
{
	int r;

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

/*
 * Flatpack LEDs
 */
static struct led_info flatpack_front_leds_info[8] = {
		{.name ="red_1", .default_trigger = NULL, .flags = 0,},
		{.name ="grn_1", .default_trigger = NULL, .flags = 0,},
		{.name ="red_3", .default_trigger = NULL, .flags = 0,},
		{.name ="grn_3", .default_trigger = NULL, .flags = 0,},
		{.name ="red_2", .default_trigger = NULL, .flags = 0,},
		{.name ="grn_2", .default_trigger = NULL, .flags = 0,},
		{.name ="red_4", .default_trigger = NULL, .flags = 0,},
		{.name ="grn_4", .default_trigger = NULL, .flags = 0,},
};

static struct led_platform_data flatpack_front_leds = {
		.num_leds = 8,
		.leds = flatpack_front_leds_info,
};

/*
 * DLCpro Touchscreen and OPC stuff
 */

#define TOPTICA_TS_IRQ_PIN  138
#define TOPTICA_TS_RST_PIN  116
#define TOPTICA_TS_WAKE_PIN  117

#define TOPTICA_OPC_IRQ_PIN 127

static struct edt_ft5x06_platform_data edt_ft5x06_pdata = {
    .reset_pin = TOPTICA_TS_RST_PIN,
    .irq_pin = TOPTICA_TS_IRQ_PIN,
};

static void inline edt_ft5x06_dev_init(void)
{
        omap_mux_init_gpio(TOPTICA_TS_IRQ_PIN, OMAP_PIN_INPUT);// OMAP_PIN_INPUT_PULLUP|OMAP_MUX_MODE4);	
	
        omap_mux_init_gpio(TOPTICA_OPC_IRQ_PIN, OMAP_PIN_INPUT);//OMAP_PIN_INPUT_PULLUP|OMAP_MUX_MODE4);	
		
        printk(KERN_INFO "setting up pins for capacitive touch ...");
  
        if (gpio_request(TOPTICA_TS_WAKE_PIN, "edt_ft5x06 wake") < 0) {
                printk(KERN_ERR "can't get edt_ft5x06 pen down GPIO\n");
                return;
        }
        
        gpio_direction_output(TOPTICA_TS_WAKE_PIN, 1);

        printk(KERN_INFO "edt_ft5x06_dev_init done\n");
}

#define I_OPC	1

struct at24_platform_data dlcpro_eeprom_data = {
	.byte_len = 8192,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16 | AT24_FLAG_IRUGO,
};

static struct i2c_board_info __initdata tam3517_i2c2_devices[] = {
     [0] = {
				/* GPIO on power supply */
				I2C_BOARD_INFO("pca9557-readbyte", 0x1A),
           },
     [I_OPC] = {
				I2C_BOARD_INFO("disira-opc", 0x28),
//		.irq = gpio_to_irq(TOPTICA_OPC_IRQ_PIN),
           },
	 [2]= {
				 /* humidity and temperature sensor on MC*/
                I2C_BOARD_INFO("sht21", 0x40),
           },
     [3] = {
        		/* temperature sensor on power supply*/
                I2C_BOARD_INFO("lm73", 0x49),
           },
     [4] = {
				/* ADC on power supply */
               .type           = "ads7828",
				.addr			= 0x4b,
	        },
     [5] = {
				/* EEPROM on power supply */
                I2C_BOARD_INFO("24c64", 0x55),
                .platform_data = &dlcpro_eeprom_data,
           },
     [6] = {
				/* EEPROM on backplane Flatpack*/
                I2C_BOARD_INFO("24c64", 0x52),
				.platform_data = &dlcpro_eeprom_data,
           },
     [7] = {
				/* EEPROM on backplane DLCpro*/
                I2C_BOARD_INFO("24c64", 0x53),
                .platform_data = &dlcpro_eeprom_data,
           },
	 [8] = { 
				/* LEDs Flatpack*/
				I2C_BOARD_INFO("pca9551", 0x63),
				.platform_data = &flatpack_front_leds,
			},           
     [9]= {
				/* pressure sensor on MC*/
        		I2C_BOARD_INFO("bmp180", 0x77),
        	},

};

static struct i2c_board_info __initdata tam3517_i2c3_devices[] = {
		{
			/* GPIO on MC, for variant detection
			 * with unconfigurable, soldered info about pcb */
			I2C_BOARD_INFO("pca9557-readbyte", 0x18),
	   },
        {
                I2C_BOARD_INFO("s35390a", 0x30),
                .type           = "s35390a",
        },
        {
                I2C_BOARD_INFO("ds1307", 0x68),
        },
		{
        		/* realtime clock on DiSiRa boad */
				I2C_BOARD_INFO("mcp7941x", 0x6F),
		},{
                I2C_BOARD_INFO("24c64", 0x50),
                .platform_data = &dlcpro_eeprom_data,
        },
};

static struct i2c_board_info __initdata dlcpro_board_info[] = {
        {       /* Temperature sensor on slot */
                I2C_BOARD_INFO("lm73", 0x48),
        },
        {       /* EEPROM on slot */
                I2C_BOARD_INFO("24c64", 0x50),
                .platform_data = &dlcpro_eeprom_data,
        },
};

static struct i2c_board_info __initdata dlcpro_laserhead_info[] = {
		{
			/* GPIO on laser heads */
			I2C_BOARD_INFO("pca9557-readbyte", 0x19),
		},
        {   /* EEPROM on laserheads */
            I2C_BOARD_INFO("24c64", 0x50),
            .platform_data = &dlcpro_eeprom_data,
        },
};

static struct i2c_board_info __initdata dlcpro_touchdriver_info[] = {
	 {
		// projective touch controller
		.type           = "edt-ft5x06",
		.addr           = 0x38,
//            .irq            = gpio_to_irq(TOPTICA_TS_IRQ_PIN),
		.platform_data  = &edt_ft5x06_pdata,
	   },
};

/* all M-Slots, S-Slots and F-Slot */
static uint32_t __initdata dlcpro_i2c_muxes_boards[] = 
									{1, 2, 3, 4, 5, 8, 11, 14, 17};

/* all Laserheads connected to TC-boards */
static uint32_t __initdata dlcpro_i2c_muxes_laserheads[] = 
									{6, 7, 9, 10, 12, 13, 15, 16};

static void __init tam3517_i2c_init(void)
{
	int i, j;

	edt_ft5x06_dev_init();


	/* ATTENTION:
	 * make sure these modify the correct array for edt... and disira-opc members!!!!!
	 */
	dlcpro_touchdriver_info[0].irq = gpio_to_irq(TOPTICA_TS_IRQ_PIN);
	tam3517_i2c2_devices[I_OPC].irq = gpio_to_irq(TOPTICA_OPC_IRQ_PIN);


	omap_register_i2c_bus(1, 400, tam3517_i2c1_devices,
			ARRAY_SIZE(tam3517_i2c1_devices));
	omap_register_i2c_bus(2, 200, NULL, 0);
	omap_register_i2c_bus(3, 400, tam3517_i2c3_devices,
			ARRAY_SIZE(tam3517_i2c3_devices));
	

	/* register I2C devices for permanent members of I2C bus */
	printk(KERN_DEBUG "DLCPRO-I2C: register unmuxed devices ...\n");
	i2c_register_board_info(200, tam3517_i2c2_devices,
                       ARRAY_SIZE(tam3517_i2c2_devices));
                
    /* register I2C devices for boards assigned to boards */
	for(i=0; i <  ARRAY_SIZE(dlcpro_i2c_muxes_boards); i++) {
		uint32_t mux = dlcpro_i2c_muxes_boards[i];
		printk(KERN_DEBUG "DLCPRO-I2C: register devices for %d muxed board...\n", mux);
		i2c_register_board_info(200+mux, dlcpro_board_info,
                       ARRAY_SIZE(dlcpro_board_info));    
    }
    
    /* register I2C devices for boards assigned to laser heads */
    
    for(i=0, j = 0; i <  ARRAY_SIZE(dlcpro_i2c_muxes_laserheads); i++, j++) {
		uint32_t mux = dlcpro_i2c_muxes_laserheads[i];
		printk(KERN_DEBUG "DLCPRO-I2C: register devices for %d muxed laser head ...\n", mux);
		i2c_register_board_info(200+mux, dlcpro_laserhead_info,
                       ARRAY_SIZE(dlcpro_laserhead_info));    
    }
    
    i2c_register_board_info(200+18, dlcpro_touchdriver_info,
                       ARRAY_SIZE(dlcpro_touchdriver_info));    
				
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS3, OMAP_MUX_MODE0 ),  /* set pins for FPGA memory map chip select */
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE0 ),  /* set pins for FPGA memory map chip select */
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
	/* USB OTG DRVVBUS offset = 0x212 (fom cm3517 patches)*/
	OMAP3_MUX(SAD2D_MCAD23, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),

	/* GPIO91 for MC boards */
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	
	/* moving UART2 to different pins, for Flatpack compatibility */
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE1), /* TX for UART2*/
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT), /* RX for UART2*/
	
	OMAP3_MUX(UART2_TX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* make original TX pin passive */
	OMAP3_MUX(UART2_RX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* make original RX pin passive */
	
	OMAP3_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO 163 for THz */
	OMAP3_MUX(UART3_RTS_SD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO 164 for THz*/

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct platform_device *tam3517_devices[] __initdata = {
	&tam3517_display_device,
	&pwm_device,
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


static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	.mode		= MUSB_OTG,
	.power		= 100,
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

#define CS_DCLK		1	/* chipselect 1 as data clock for configuration */
#define CS_MMAP1		3   /* chipselect 3 as data clock for memory mapping 1 */
#define CS_MMAP2		4   /* chipselect 3 as data clock for memory mapping 2 */
#define GMPC_CONFIG2_CSONTIME(x)	((x & 0xf) << 0)
#define GMPC_CONFIG2_CSRDOFFTIME(x)	((x & 0x1f) << 8)
#define GMPC_CONFIG2_CSWROFFTIME(x)	((x & 0x1f) << 16)

#define GMPC_CONFIG4_OEONTIME(x)	((x & 0xf) << 0)
#define GMPC_CONFIG4_OEOFFTIME(x)	((x & 0x1f) << 8)
#define GMPC_CONFIG4_WEONTIME(x)	((x & 0xf) << 16)
#define GMPC_CONFIG4_WEOFFTIME(x)	((x & 0x1f) << 24)

#define GMPC_CONFIG5_RDCYCLETIME(x)	((x & 0x1f) << 0)
#define GMPC_CONFIG5_WRCYCLETIME(x)	((x & 0x1f) << 8)
#define GMPC_CONFIG5_RDACCESSTIME(x)	((x & 0x1f) << 16)


static __init void disira_fpga_mem_cs_init(int cs)
{
	unsigned long config_base;
	u32 config1, config2, config4, config5;
	int ret;
	
   ret = gpmc_cs_request(cs, PAGE_SIZE, &config_base);
	if (ret) {
		printk(KERN_ERR "Cannot get chip select %d for configuration\n",cs);
		return;
	}

	printk(KERN_INFO "CS%d for FPGA memory mapping ...\n", cs);
	config1 = GPMC_CONFIG1_PAGE_LEN(2)
			| GPMC_CONFIG1_WAIT_PIN_SEL(0)
			| GPMC_CONFIG1_DEVICESIZE_16
			| GPMC_CONFIG1_DEVICETYPE_NOR
			| GPMC_CONFIG1_TIME_PARA_GRAN;

	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1, config1);
	config1 = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);
	printk(KERN_INFO "CS Config1 0x%08x\n", config1);

	config2 = GMPC_CONFIG2_CSONTIME(2)
			| GMPC_CONFIG2_CSRDOFFTIME(8)
			| GMPC_CONFIG2_CSWROFFTIME(9);

	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, config2);
	config2 = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG2);
	printk(KERN_INFO "CS Config2 0x%08x\n", config2);

	config4 = GMPC_CONFIG4_OEONTIME(4)
			| GMPC_CONFIG4_OEOFFTIME(7)
			| GMPC_CONFIG4_WEONTIME(4)
			| GMPC_CONFIG4_WEOFFTIME(8);

	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, config4);
	config4 = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG4);
	printk(KERN_INFO "CS Config4 0x%08x\n", config4);

	config5 = GMPC_CONFIG5_RDCYCLETIME(8)
			| GMPC_CONFIG5_WRCYCLETIME(10)
			| GMPC_CONFIG5_RDACCESSTIME(7);

	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG5, config5);
	config5 = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG5);
	printk(KERN_INFO "CS Config5 0x%08x\n", config5);

	gpmc_cs_free(cs);	
}

static __init void disira_fgpa_config_cs_init(void)
{
       /* configure chipselect for FPGA configuration */
		unsigned long config_base;
        u32 config1, config2, config4, config5;
		int ret;

#define nSTATUS   103
#define CONF_DONE 100
#define nCONFIG   102
#define OE_FPGA   104
#define INIT_DONE 101

		omap_mux_init_gpio(nSTATUS, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);  /* FIXME: do we need pullup? */
		ret = gpio_request(nSTATUS, "nSTATUS");
		if (ret < 0)
				printk(KERN_ERR "couldn't get GPIO %d for nSTATUS", nSTATUS);
		ret = gpio_direction_input(nSTATUS);
		//ret = gpio_direction_output(nSTATUS, 1);
		gpio_export(nSTATUS, 1);



		//omap_mux_init_gpio(CONF_DONE, OMAP_PIN_INPUT_PULLUP|OMAP_MUX_MODE4); /* FIXME: do we need pullup? */
		omap_mux_init_gpio(CONF_DONE, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4); /* FIXME: do we need pullup? */
		ret = gpio_request(CONF_DONE, "CONF_DONE");
		if (ret < 0)
				printk(KERN_ERR "couldn't get GPIO %d for CONF_DONE", CONF_DONE);
		ret = gpio_direction_input(CONF_DONE);
		//ret = gpio_direction_output(CONF_DONE, 1);
		gpio_export(CONF_DONE, 1);

		omap_mux_init_gpio(INIT_DONE, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);  /* FIXME: do we need pullup? */
		ret = gpio_request(INIT_DONE, "INIT_DONE");
		if (ret < 0)
				printk(KERN_ERR "couldn't get GPIO %d for INIT_DONE", INIT_DONE);
		ret = gpio_direction_input(INIT_DONE);
		//ret = gpio_direction_output(INIT_DONE, 1);
		gpio_export(INIT_DONE, 1);

		omap_mux_init_gpio(OE_FPGA, OMAP_PIN_OUTPUT|OMAP_MUX_MODE4);
		ret = gpio_request(OE_FPGA, "OE_FPGA");
		if (ret < 0)
				printk(KERN_ERR "couldn't get GPIO %d for OE_FPGA", OE_FPGA);
		ret = gpio_direction_output(OE_FPGA, 0);
		//ret = gpio_direction_input(OE_FPGA);
		gpio_export(OE_FPGA, 0);

		omap_mux_init_signal("ccdc_d3.gpio_102", OMAP_PIN_OUTPUT);
		ret = gpio_request(nCONFIG, "nCONFIG");
		if (ret < 0)
				printk(KERN_ERR "couldn't get GPIO %d for nCONFIG", nCONFIG);

		//pin_mux(nCONFIG, OMAP_PIN_OUTPUT|OMAP_MUX_MODE4);
		//pin_mux(nCONFIG, OMAP_PIN_INPUT|OMAP_MUX_MODE4);
		ret = gpio_direction_output(nCONFIG, 1);
		//ret = gpio_direction_input(nCONFIG);
		gpio_export(nCONFIG, 1 );



        ret = gpmc_cs_request(CS_DCLK, PAGE_SIZE, &config_base);
		if (ret) {
			printk(KERN_ERR "Cannot get chip select %d for configuration\n",CS_DCLK);
			return;
		}

		printk(KERN_INFO "CS for FPGA configuration ...\n");
		config1 = GPMC_CONFIG1_PAGE_LEN(2)
				| GPMC_CONFIG1_WAIT_PIN_SEL(0)
				| GPMC_CONFIG1_DEVICESIZE_16
				| GPMC_CONFIG1_DEVICETYPE_NOR
				| GPMC_CONFIG1_TIME_PARA_GRAN;

		gpmc_cs_write_reg(CS_DCLK, GPMC_CS_CONFIG1, config1);
		config1 = gpmc_cs_read_reg(CS_DCLK, GPMC_CS_CONFIG1);
		printk(KERN_INFO "CS Config1 0x%08x\n", config1);

		config2 = GMPC_CONFIG2_CSONTIME(2)
				| GMPC_CONFIG2_CSRDOFFTIME(8)
				| GMPC_CONFIG2_CSWROFFTIME(9);

		gpmc_cs_write_reg(CS_DCLK, GPMC_CS_CONFIG2, config2);
		config2 = gpmc_cs_read_reg(CS_DCLK, GPMC_CS_CONFIG2);
		printk(KERN_INFO "CS Config2 0x%08x\n", config2);

		config4 = GMPC_CONFIG4_OEONTIME(4)
				| GMPC_CONFIG4_OEOFFTIME(7)
				| GMPC_CONFIG4_WEONTIME(4)
				| GMPC_CONFIG4_WEOFFTIME(8);

		gpmc_cs_write_reg(CS_DCLK, GPMC_CS_CONFIG4, config4);
		config4 = gpmc_cs_read_reg(CS_DCLK, GPMC_CS_CONFIG4);
		printk(KERN_INFO "CS Config4 0x%08x\n", config4);

		config5 = GMPC_CONFIG5_RDCYCLETIME(8)
				| GMPC_CONFIG5_WRCYCLETIME(10)
				| GMPC_CONFIG5_RDACCESSTIME(7);

		gpmc_cs_write_reg(CS_DCLK, GPMC_CS_CONFIG5, config5);
		config5 = gpmc_cs_read_reg(CS_DCLK, GPMC_CS_CONFIG5);
		printk(KERN_INFO "CS Config5 0x%08x\n", config5);

		gpmc_cs_free(CS_DCLK);

     
}


# define pin_mux_output(pin, name, start) \
	omap_mux_init_gpio(pin, OMAP_PIN_OUTPUT); \
	gpio_request(pin, name); \
	gpio_direction_output(pin, start); \
    gpio_export(pin, 1)

# define pin_mux_input(pin, name) \
	omap_mux_init_gpio(pin, OMAP_PIN_INPUT); \
	gpio_request(pin, name); \
	gpio_direction_input(pin); \
    gpio_export(pin, 1)

# define pin_mux(pin, flags) \
	omap_mux_init_gpio(pin, flags); \
    gpio_export(pin, 1)

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

	/* Ethernet */
	am35xx_emac_init(AM35XX_DEFAULT_MDIO_FREQUENCY, 1);
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	tam3517_init_smsc911x();
#endif

	/* MUSB init */
	am3517_musb_init();

	disira_fgpa_config_cs_init();
	disira_fpga_mem_cs_init(CS_MMAP1);
	disira_fpga_mem_cs_init(CS_MMAP2);

	/* MC board pin for USB Host power detection (?) */
	omap_mux_init_gpio(91, OMAP_PIN_INPUT);
	gpio_export(91, 1);

	/* more GPIOs for DiSiRa */
	omap_mux_init_signal("sdmmc1_dat2.gpio_124", OMAP_PIN_INPUT);
	gpio_request(124, "disableButton");
	gpio_export(124,0 );
	gpio_direction_input(124);

	omap_mux_init_signal("sdmmc1_cmd.gpio_121", OMAP_PIN_INPUT);
	gpio_request(121, "enableButton");
	gpio_export(121, 0);
	gpio_direction_input(121);

	//pin_mux_output(24, "laser_enable" );
	pin_mux(24, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);
	gpio_request(24, "interlock");
	gpio_direction_input(24);
	gpio_export(24, 1);

	pin_mux(26, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);
	gpio_request(26, "lasern_status");
	gpio_direction_input(26);
	gpio_export(26, 1);

	/* laser enable on new MC-boads A03 */
	pin_mux_input(94, "laser_enable");

	/* front USB power enable */
	pin_mux_output(90, "USB-power", 1);


	/* temporary test of other gpios */
	pin_mux_input(95, "gpio95" );
	pin_mux_input(99, "gpio99" );
	
	/* interrupt lines for MC FPGA */
	pin_mux_input(117, "FPGA-gpio117" );
	pin_mux_input(118, "FPGA-gpio118" );
	pin_mux_input(119, "FPGA-gpio119" );
	pin_mux_input(141, "FPGA-gpio141" );
	pin_mux_input(140, "FPGA-gpio140" );
	pin_mux_input(152, "FPGA-gpio152" );
	pin_mux_input(153, "FPGA-gpio153" );
	pin_mux_input(154, "FPGA-gpio154" );
	pin_mux_input(155, "FPGA-gpio155" );
	pin_mux_input(150, "FPGA-gpio150" );
	pin_mux_input(149, "FPGA-gpio149" );
	pin_mux_input(151, "FPGA-gpio151" );
	pin_mux_input(148, "FPGA-gpio148" );
	
	/* interrupt lines for modules */
	pin_mux_input(128, "Slot-S1-gpio128" );
	pin_mux_input(124, "Slot-S2-gpio124" );
	pin_mux_input(125, "Slot-S3-gpio125" );
	pin_mux_input(96,  "Slot-S4-gpio96" );
	
	pin_mux_input(98,  "Slot-M1-gpio98" );
	pin_mux_input(122, "Slot-M2-gpio122" );
	pin_mux_input(123, "Slot-M3-gpio123" );
	pin_mux_input(97,  "Slot-M4-gpio97" );
	
	pin_mux_input(29,  "Slot-E-gpio29" );
	
	pin_mux_input(121, "Slot-F-gpio121" );
	
	/* JTAG for FPGA configuration */
	pin_mux_output(171, "jtag_tck", 1);
	pin_mux_output(172, "jtag_tdi", 1);
	pin_mux_input(173, "jtag_tdo" );
	pin_mux_output(174, "jtag_tms", 1);
	
	/* THz GPIOs */
	pin_mux_output(163, "gpio163", 0); 
	pin_mux_output(164, "gpio164", 0); 

}

static const char *tam3517_dt_match[] __initdata = {
	"technexion,tam3517",
	NULL
};

MACHINE_START(AM3517_TAM3517, "TOPTICA TAM3517 MC-Board 200kBit-muxed")
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
