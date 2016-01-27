/*
 * arch/arm/mach-mx25/stk5-custom.c
 *
 * Copyright (C) 2009  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * Custom initialisation for the Ka-Ro Starterkit-5 base board.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
 *
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <linux/gpio_keys.h>
#include <linux/dma-mapping.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/input/matrix_keypad.h>
#include <video/ili9322.h>

#include <asm/setup.h>
#include <asm/irq.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/irqs.h>
#include <mach/imxfb.h>
#include <mach/i2c.h>
#include <mach/board-tx25.h>
#include <mach/spi.h>
#include <mach/imx_spi.h>
#include <mach/mxc_audio.h>

#include "devices.h"
#include "karo.h"

#ifdef CONFIG_I2C
static struct at24_platform_data stk5_custom_eeprom = {
	.byte_len = 2048,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16 | AT24_FLAG_TAKE8ADDR,
};

static struct pad_desc mxc_i2c2_pins[] = {
	MX25_PAD_GPIO_C__I2C2_CLK,
	MX25_PAD_GPIO_D__I2C2_DAT,
};

static int stk5_custom_i2c2_init(struct device *dev)
{
	DBG(-1, "%s: \n", __FUNCTION__);
	return mxc_iomux_v3_setup_multiple_pads(mxc_i2c2_pins,
						ARRAY_SIZE(mxc_i2c2_pins));
}

static void stk5_custom_i2c2_exit(struct device *dev)
{
	DBG(-1, "%s: \n", __FUNCTION__);
	mxc_iomux_v3_release_multiple_pads(mxc_i2c2_pins,
					   ARRAY_SIZE(mxc_i2c2_pins));
}

static struct imxi2c_platform_data stk5_custom_i2c2_data = {
	.bitrate = 100000,
	.init = stk5_custom_i2c2_init,
	.exit = stk5_custom_i2c2_exit,
};

static struct i2c_board_info stk5_custom_i2c2_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("24c16", 0x50),
		.platform_data = &stk5_custom_eeprom,
		.type = "24c16",
	},
	{
		I2C_BOARD_INFO("rtc-ds1307", 0x68),
		.type = "ds1339",
	},
};

int __init stk5_custom_i2c2_register(void)
{
	int ret;

	DBG(0, "%s: Registering I2C2 bus\n", __FUNCTION__);
	ret = mxc_register_device(&mx25_i2c2_device, &stk5_custom_i2c2_data);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C2 device: %d\n", ret);
		return ret;
	}

	ret = i2c_register_board_info(1, stk5_custom_i2c2_boardinfo,
				      ARRAY_SIZE(stk5_custom_i2c2_boardinfo));
	if (ret != 0)
		printk(KERN_ERR "Failed to register I2C2 board info: %d\n", ret);

	return ret;
}
device_initcall(stk5_custom_i2c2_register);
#endif /* CONFIG_I2C */

#if defined(CONFIG_KEYBOARD_IMX) || defined(CONFIG_KEYBOARD_IMX_MODULE)
/*
 * This array is used for mapping keypad scancodes to keyboard keycodes.
 */
static const uint32_t tx25_kpd_keycodes[] = {
	/* specify your keymap with KEY(row, col, keycode), */
	KEY(0, 0, KEY_POWER),
};

static struct matrix_keymap_data tx25_keypad_data = {
	.keymap = tx25_kpd_keycodes,
	.keymap_size = ARRAY_SIZE(tx25_kpd_keycodes),
};

static struct pad_desc tx25_keypad_pads[] __initdata = {
	MX25_PAD_KPP_ROW0__KPP_ROW0,
	MX25_PAD_KPP_ROW1__KPP_ROW1,
	MX25_PAD_KPP_ROW2__KPP_ROW2,
	MX25_PAD_KPP_ROW3__KPP_ROW3,

	MX25_PAD_KPP_COL0__KPP_COL0,
	MX25_PAD_KPP_COL1__KPP_COL1,
	MX25_PAD_KPP_COL2__KPP_COL2,
	MX25_PAD_KPP_COL3__KPP_COL3,
};

static int __init tx25_keypad_init(void)
{
	return mxc_iomux_v3_setup_multiple_pads(tx25_keypad_pads,
					       ARRAY_SIZE(tx25_keypad_pads));
}
device_initcall(tx25_keypad_init);
#endif

#if (defined(CONFIG_LCD_GPM1076A0) || defined(CONFIG_LCD_GPM1076A0_MODULE)) && \
	(defined(CONFIG_FB_IMX) || defined(CONFIG_FB_IMX_MODULE))
#define STK5_LCD_BACKLIGHT_GPIO		(GPIO_PORTA | 26)
#define STK5_LCD_RESET_GPIO		(GPIO_PORTB | 4)
#define STK5_LCD_SPI_CS_GPIO		(GPIO_PORTB | 5)

/*
 * Setup GPIO for LCDC device to be active
 *
 */
static struct pad_desc stk5_custom_lcdc_gpios[] = {
	MX25_PAD_A18__GPIO_2_4,		/* LCD Reset (active LOW) */
	MX25_PAD_LSCLK__LSCLK,
	MX25_PAD_LD0__LD0,
	MX25_PAD_LD1__LD1,
	MX25_PAD_LD2__LD2,
	MX25_PAD_LD3__LD3,
	MX25_PAD_LD4__LD4,
	MX25_PAD_LD5__LD5,
	MX25_PAD_LD6__LD6,
	MX25_PAD_LD7__LD7,
	MX25_PAD_LD8__LD8,
	MX25_PAD_LD9__LD9,
	MX25_PAD_LD10__LD10,
	MX25_PAD_LD11__LD11,
	MX25_PAD_LD12__LD12,
	MX25_PAD_LD13__LD13,
	MX25_PAD_LD14__LD14,
	MX25_PAD_LD15__LD15,
	MX25_PAD_D15__LD16,
	MX25_PAD_D14__LD17,
	MX25_PAD_HSYNC__HSYNC,
	MX25_PAD_VSYNC__VSYNC,
	MX25_PAD_OE_ACD__OE_ACD,
};

static int gpm1076a0_lcd_active(struct platform_device *dev)
{
	int ret;

	DBG(0, "%s: Setting up GPIO pins for LCD\n", __FUNCTION__);
	ret = mxc_iomux_v3_setup_multiple_pads(stk5_custom_lcdc_gpios,
					ARRAY_SIZE(stk5_custom_lcdc_gpios));
	if (ret) {
		dev_err(&dev->dev, "Failed to setup GPIO pins for LCD: %d\n",
			ret);
		return ret;
	}
	ret = gpio_request(STK5_LCD_RESET_GPIO, "LCD RESET");
	if (ret) {
		dev_err(&dev->dev, "Failed to request GPIO for LCD RESET: %d\n",
			ret);
		goto free_gpio;
	}

	gpio_direction_output(STK5_LCD_RESET_GPIO, 0);
	msleep(30);
	return 0;

free_gpio:
	mxc_iomux_v3_release_multiple_pads(stk5_custom_lcdc_gpios,
					ARRAY_SIZE(stk5_custom_lcdc_gpios));
	return ret;
}

/*
 * Setup GPIO for LCDC device to be inactive
 *
 */
static void gpm1076a0_lcd_inactive(struct platform_device *dev)
{
	mxc_iomux_v3_release_multiple_pads(stk5_custom_lcdc_gpios,
					ARRAY_SIZE(stk5_custom_lcdc_gpios));
}

static void gpm1076a0_lcd_reset(int on)
{
	DBG(0, "%s: Switching LCD reset %s\n", __FUNCTION__, on ? "off" : "on");
	if (on) {
		gpio_set_value(STK5_LCD_RESET_GPIO, 0);
	} else {
		gpio_set_value(STK5_LCD_RESET_GPIO, 1);
	}
}

static struct imx_fb_videomode gpm1076a0_fb_modes[] = {
	{
		.bpp	= 16,
		.mode = {
			.name = "GPM1076A0-16",
			.pixclock	= 154178,

			.xres		= 320,
			.yres		= 240,

			.hsync_len	= 3,
			.left_margin	= 40,
			.right_margin	= 37,

			.vsync_len	= 3,
			.upper_margin	= 16,
			.lower_margin	= 18,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp	= 32,
		.mode = {
			.name = "GPM1076A0-18",
			.pixclock	= 154178,

			.xres		= 320,
			.yres		= 240,

			.hsync_len	= 3,
			.left_margin	= 40,
			.right_margin	= 37,

			.vsync_len	= 3,
			.upper_margin	= 16,
			.lower_margin	= 18,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp	= 8,
		.mode = {
			.name = "GPM1076A0-8",
			.pixclock	= 154178,

			.xres		= 320,
			.yres		= 240,

			.hsync_len	= 3,
			.left_margin	= 40,
			.right_margin	= 37,

			.vsync_len	= 3,
			.upper_margin	= 16,
			.lower_margin	= 18,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_END_BYTE_SWAP |
		PCR_BPIX_8 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
};

static struct imx_fb_platform_data gpm1076a0_fb_data = {
	.init		= gpm1076a0_lcd_active,
	.exit		= gpm1076a0_lcd_inactive,

	.mode		= gpm1076a0_fb_modes,
	.num_modes	= ARRAY_SIZE(gpm1076a0_fb_modes),

	.dmacr		= 0x80040060,

	.cmap_greyscale	= 0,
	.cmap_inverse	= 0,
	.cmap_static	= 0,

	.fixed_screen_cpu = NULL,
};
#endif /* CONFIG_LCD_GPM1076A0 || CONFIG_LCD_GPM1076A0_MODULE */

#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)
#if (defined(CONFIG_LCD_GPM1076A0) || defined(CONFIG_LCD_GPM1076A0_MODULE))
static struct pad_desc stk5_custom_cspi1_pads[] = {
	MX25_PAD_CSPI1_MOSI__CSPI1_MOSI,
	MX25_PAD_CSPI1_MISO__CSPI1_MISO,
	MX25_PAD_CSPI1_SS0__CSPI1_SS0,
	MX25_PAD_CSPI1_SS1__CSPI1_SS1,
	MX25_PAD_CSPI1_SCLK__CSPI1_SCLK,
	MX25_PAD_CSPI1_RDY__CSPI1_RDY,
	MX25_PAD_A19__GPIO_2_5,		/* LCD SPI CS */
};

static struct ili9322_reg gpm1076a0_reg_data[] = {
#if 0
	/* default data set up by the driver
	 * uncomment and change if necessary
	 */
	{ ILI9322_REG_GAMMA1, 0xa7, },
	{ ILI9322_REG_GAMMA2, 0x57, },
	{ ILI9322_REG_GAMMA3, 0x73, },
	{ ILI9322_REG_GAMMA4, 0x72, },
	{ ILI9322_REG_GAMMA5, 0x73, },
	{ ILI9322_REG_GAMMA6, 0x55, },
	{ ILI9322_REG_GAMMA7, 0x17, },
	{ ILI9322_REG_GAMMA8, 0x62, },
#endif
};

static struct ili9322_platdata gpm1076a0_platform_data = {
	.reset = gpm1076a0_lcd_reset,
	.intf_mode = ILI9322_HS_VS_DE_MODE,
	.data_fmt = ILI9322_FMT_RGB24,
	.reg_data = gpm1076a0_reg_data,
	.reg_cnt = ARRAY_SIZE(gpm1076a0_reg_data),
};

static int stk5_custom_spi_chipselect[] = {
	STK5_LCD_SPI_CS_GPIO,
};

static struct spi_board_info stk5_custom_spi_info[] = {
	{
		.modalias = "GPM1076A0",
		.max_speed_hz = 2000000,
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &gpm1076a0_platform_data,
	},
};
#elif defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
static struct pad_desc stk5_custom_cspi1_pads[] = {
	MX25_PAD_CSPI1_MOSI__CSPI1_MOSI,
	MX25_PAD_CSPI1_MISO__CSPI1_MISO,
	MX25_PAD_CSPI1_SS0__CSPI1_SS0,
	MX25_PAD_CSPI1_SS1__CSPI1_SS1,
	MX25_PAD_CSPI1_SCLK__CSPI1_SCLK,
	MX25_PAD_CSPI1_RDY__CSPI1_RDY,
};

static int stk5_custom_spi_chipselect[] = {
	MXC_SPI_CS(0),
};

static struct spi_board_info stk5_custom_spi_info[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 2000000,
		.bus_num = 0,
		.chip_select = 0,
	},
};
#else
#error No SPI chip driver configured
#endif

static struct spi_imx_master stk5_custom_spi1_data = {
	.chipselect = stk5_custom_spi_chipselect,
	.num_chipselect = ARRAY_SIZE(stk5_custom_spi_chipselect),
};

static int __init stk5_custom_spi_register(void)
{
	int ret;

	ret = mxc_iomux_v3_setup_multiple_pads(stk5_custom_cspi1_pads,
					ARRAY_SIZE(stk5_custom_cspi1_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure CSPI1 pads\n");
		return ret;
	}
	ret = spi_register_board_info(stk5_custom_spi_info,
				ARRAY_SIZE(stk5_custom_spi_info));
	if (ret) {
		printk(KERN_ERR "%s: Failed to register SPI board_info: %d\n",
			__FUNCTION__, ret);
		goto err;
	}
#ifdef STK5_LCD_SPI_CS_GPIO
	ret = gpio_request(STK5_LCD_SPI_CS_GPIO, "SPI");
	if (ret)
		goto err;

	gpio_direction_output(STK5_LCD_SPI_CS_GPIO, 1);
	/* Free the CS GPIO, to allow SPI driver to register it again */
	gpio_free(STK5_LCD_SPI_CS_GPIO);
#endif
	ret = mxc_register_device(&mxc_spi_device0, &stk5_custom_spi1_data);
	DBG(0, "%s: mxc_register_device() returned: %d\n", __FUNCTION__, ret);
	return 0;

err:
	mxc_iomux_v3_release_multiple_pads(stk5_custom_cspi1_pads,
					ARRAY_SIZE(stk5_custom_cspi1_pads));
	return ret;
}
device_initcall(stk5_custom_spi_register);
#endif // defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)

static struct platform_dev_list {
	struct platform_device *pdev;
	void *pdata;
} stk5_custom_devices[] __initdata = {
#if defined(CONFIG_LCD_GPM1076A0) || defined(CONFIG_LCD_GPM1076A0_MODULE)
	{ .pdev = &mx25_fb_device, .pdata = &gpm1076a0_fb_data, },
#endif
#if defined(CONFIG_KEYBOARD_IMX) || defined(CONFIG_KEYBOARD_IMX_MODULE)
	{ .pdev = &imx_keypad_device, .pdata = &tx25_keypad_data, },
#endif
};
#define STK5_CUSTOM_NUM_DEVICES		ARRAY_SIZE(stk5_custom_devices)

__init int stk5_custom_board_init(void)
{
	int ret;
	int i;

	DBG(0, "%s: \n", __FUNCTION__);
	for (i = 0; i < STK5_CUSTOM_NUM_DEVICES; i++) {
		if (stk5_custom_devices[i].pdev == NULL) continue;
		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, stk5_custom_devices[i].pdev, &stk5_custom_devices[i].pdev->dev,
			stk5_custom_devices[i].pdev->name);
		if (stk5_custom_devices[i].pdata) {
			ret = mxc_register_device(stk5_custom_devices[i].pdev,
						stk5_custom_devices[i].pdata);
		} else {
			ret = platform_device_register(stk5_custom_devices[i].pdev);
		}
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, stk5_custom_devices[i].pdev->name, ret);
		}
	}
	DBG(0, "%s: Done\n", __FUNCTION__);
	return 0;
}
