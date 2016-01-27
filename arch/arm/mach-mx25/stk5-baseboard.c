/*
 * arch/arm/mach-mx25/stk5-baseboard.c
 *
 * Copyright (C) 2009  Lothar Wassmann <LW@KARO-electronics.de>
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
 * This file adds support for devices found on Ka-Ro electronics
 * Starterkit-5 (STK5) baseboard
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/i2c-gpio.h>
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/pwm_backlight.h>
#include <linux/i2c/tsc2007.h>
#include <linux/dma-mapping.h>
#include <linux/input/matrix_keypad.h>

#include <asm/clkdev.h>

#include <mach/clock.h>
#include <mach/common.h>
#include <mach/gpio.h>
#include <mach/sdhci.h>
#include <mach/mxc_ehci.h>
#include <mach/iomux.h>
#include <mach/mxc-ac97.h>
#include <mach/mxc_audmux.h>
#include <mach/mxc_audio.h>
#include <mach/imx-uart.h>
#include <mach/imxfb.h>
#include <mach/i2c.h>
#include <mach/board-tx25.h>

#include "devices.h"
#include "karo.h"

static int stk5_board_rev;

#if defined(CONFIG_SERIAL_IMX) || defined(CONFIG_SERIAL_IMX_MODULE)
static struct pad_desc stk5_uart_pads[][4] = {
	{
		MX25_PAD_UART1_TXD__UART1_TXD,
		MX25_PAD_UART1_RXD__UART1_RXD,
		MX25_PAD_UART1_CTS__UART1_CTS,
		MX25_PAD_UART1_RTS__UART1_RTS,
	},
	{
		MX25_PAD_UART2_TXD__UART2_TXD,
		MX25_PAD_UART2_RXD__UART2_RXD,
		MX25_PAD_UART2_CTS__UART2_CTS,
		MX25_PAD_UART2_RTS__UART2_RTS,
	},
	{
		/* UART 3 not useable on TX25 */
	},
	{
		/* UART 4 not useable on TX25 */
	},
	{
		MX25_PAD_ECB__UART5_TXD_MUX,
		MX25_PAD_LBA__UART5_RXD_MUX,
		MX25_PAD_CS4__UART5_CTS,
		MX25_PAD_CS5__UART5_RTS,
	},
};

static int stk5_uart_init(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	if (pdev->id >= ARRAY_SIZE(stk5_uart_pads)) {
		return -ENODEV;
	}
	return mxc_iomux_v3_setup_multiple_pads(stk5_uart_pads[pdev->id],
						ARRAY_SIZE(stk5_uart_pads[pdev->id]));
}

static void stk5_uart_exit(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	BUG_ON(pdev->id >= ARRAY_SIZE(stk5_uart_pads));
	mxc_iomux_v3_release_multiple_pads(stk5_uart_pads[pdev->id],
					ARRAY_SIZE(stk5_uart_pads[pdev->id]));
}

static struct imxuart_platform_data stk5_uart_port_data = {
	.init = stk5_uart_init,
	.exit = stk5_uart_exit,
	.flags = IMXUART_HAVE_RTSCTS,
};
#endif

#if defined(CONFIG_ARCH_MXC_EHCI_USBH2) || defined(CONFIG_ARCH_MXC_EHCI_USBOTG)

/* USB register offsets */
#define REG_USBCTRL		0x600
#define REG_PHY_CTRL		0x608

#define PHY_CTRL_CLKVLD		(1 << 27)
#define PHY_CTRL_SUSP		(1 << 26)
#define PHY_CTRL_RESET		(1 << 25)
#define PHY_CTRL_USBEN		(1 << 24)
#define PHY_CTRL_EVDO		(1 << 23)
#define PHY_CTRL_LSFE		(1 << 22)

/* USB Host/OTG register offsets */
#define REG_USBCMD		0x140
#define REG_USBSTS		0x144
#define REG_PORTSC1		0x184
#define REG_USBMODE		0x1a8

#define USBCMD_RST		(1 << 1)
#define USBCMD_RUN		(1 << 0)

#define USBSTS_HCH		(1 << 12)

/* USB_CTRL register bits */
#define USBCTRL_OCPOL_HST	(1 << 2)
#define USBCTRL_OCPOL_OTG	(1 << 3)
#define USBCTRL_USBTE		(1 << 4)
#define USBCTRL_HSDT		(1 << 5)
#define USBCTRL_PUE_DWN		(1 << 6)
#define USBCTRL_XCSH		(1 << 9)
#define USBCTRL_XCSO		(1 << 10)
#define USBCTRL_PP_OTG		(1 << 11)
#define USBCTRL_HLKEN		(1 << 12)
#define USBCTRL_OLKEN		(1 << 13)
#define USBCTRL_HPM		(1 << 16)
#define USBCTRL_PP_HST		(1 << 18)
#define USBCTRL_HWIE		(1 << 19)
#define USBCTRL_HUIE		(1 << 20)
#define USBCTRL_HSIC_MASK	(0x3 << 21)
#define USBCTRL_HSIC(n)		(((n) << 21) & USBCTRL_HSIC_MASK)
#define USBCTRL_OPM		(1 << 24)
#define USBCTRL_OEXTEN		(1 << 25)
#define USBCTRL_HEXTEN		(1 << 26)
#define USBCTRL_OWIE		(1 << 27)
#define USBCTRL_OUIE		(1 << 28)
#define USBCTRL_OSIC_MASK	(0x3 << 29)
#define USBCTRL_OSIC(n)		(((n) << 29) & USBCTRL_OSIC_MASK)

#ifdef DEBUG
#define usb_reg_write(v,b,r)	_usb_reg_write(v,b,r,#r)
static inline void _usb_reg_write(unsigned long val, void __iomem *base, int reg,
				const char *name)
{
	DBG(0, "%s: Writing %08lx to %s[%03x]\n", __FUNCTION__, val, name, reg);
	__raw_writel(val, base + reg);
}

#define usb_reg_read(b,r)	_usb_reg_read(b,r,#r)
static inline unsigned long _usb_reg_read(void __iomem *base, int reg, const char *name)
{
	unsigned long val;

	val = __raw_readl(base + reg);
	DBG(0, "%s: Read %08lx from %s[%03x]\n", __FUNCTION__, val, name, reg);
	return val;
}
#else
static inline void usb_reg_write(unsigned long val, void __iomem *base, int reg)
{
	__raw_writel(val, base + reg);
}

static inline unsigned long usb_reg_read(void __iomem *base, int reg)
{
	return __raw_readl(base + reg);
}
#endif

//#define USE_USB_PWR

static int tx25_usb_init(struct platform_device *pdev, void __iomem *base, int host_mode)
{
	unsigned long val;
	unsigned long flags;
	const char __maybe_unused *name = pdev->id ? "USBH2" : "USBOTG";
	unsigned int loops = 0;
	void __iomem *otg_base = MX25_IO_ADDRESS(OTG_BASE_ADDR);

	if (!(usb_reg_read(base, REG_USBSTS) & USBSTS_HCH)) {
		unsigned int loops = 0;

		if (usb_reg_read(base, REG_USBCMD) & USBCMD_RUN) {
			DBG(0, "%s: %s[%p] is busy: %08lx\n", __FUNCTION__, name,
				base + REG_USBSTS, usb_reg_read(base, REG_USBSTS));
			usb_reg_write(usb_reg_read(base, REG_USBCMD) & ~USBCMD_RUN,
				base, REG_USBCMD);
			while (!(usb_reg_read(base, REG_USBSTS) & USBSTS_HCH)) {
				udelay(100);
				loops++;
				if (loops > 100)
					break;
			}
			if (usb_reg_read(base, REG_USBSTS) & USBSTS_HCH) {
				DBG(0, "USB controller idle after %u loops\n", loops);
			} else {
				DBG(0, "USB controller NOT idle after %u loops\n", loops);
			}
		} else {
			DBG(0, "%s: Ignoring bogus %s_HCH\n", __FUNCTION__,
				name);
		}
	}

	DBG(0, "%s: PHY_CTRL[%p]=%08lx\n", __FUNCTION__, otg_base + REG_PHY_CTRL,
		usb_reg_read(otg_base, REG_PHY_CTRL));
	DBG(0, "%s: USBCMD[%p]=%08lx\n", __FUNCTION__, base + REG_USBCMD,
		usb_reg_read(base, REG_USBCMD));
	DBG(0, "%s: USBSTS[%p]=%08lx\n", __FUNCTION__, base + REG_USBSTS,
		usb_reg_read(base, REG_USBSTS));

	/* reset USB Host controller */
	usb_reg_write(usb_reg_read(base, REG_USBCMD) | USBCMD_RST,
		base, REG_USBCMD);
	while (usb_reg_read(base, REG_USBCMD) & USBCMD_RST) {
		cpu_relax();
		loops++;
	}
	DBG(0, "USB controller reset finished after %u loops\n", loops);

	local_irq_save(flags);
	val = usb_reg_read(otg_base, REG_USBCTRL);
	if (pdev->id == 1) {
#ifdef USE_USB_PWR
		val &= ~(USBCTRL_OCPOL_HST |
			USBCTRL_HEXTEN |
			USBCTRL_HSIC_MASK |
			USBCTRL_HWIE |
			USBCTRL_HPM);
		val |=	USBCTRL_HLKEN |
			USBCTRL_PP_HST |
			USBCTRL_XCSH |
			USBCTRL_HSDT | /* disable TLL */
			USBCTRL_USBTE | /* enable serial XCVR */
			USBCTRL_PUE_DWN |
			USBCTRL_HSIC(0);
#else
		val &= ~(USBCTRL_OCPOL_HST |
			USBCTRL_HEXTEN |
			USBCTRL_HSIC_MASK |
			USBCTRL_HWIE);
		val |=	USBCTRL_HLKEN |
			USBCTRL_PP_HST |
			USBCTRL_XCSH |
			USBCTRL_HSDT | /* disable TLL */
			USBCTRL_USBTE | /* enable serial XCVR */
			USBCTRL_PUE_DWN |
			USBCTRL_HSIC(0) |
			USBCTRL_HPM;

#endif
	} else {
#ifdef USE_USB_PWR
		val &= ~(USBCTRL_OCPOL_OTG |
			USBCTRL_OEXTEN |
			USBCTRL_XCSO |
			USBCTRL_OLKEN |
			USBCTRL_OWIE |
			USBCTRL_OPM);
		val |= USBCTRL_PP_OTG;
#else
		val &= ~(USBCTRL_OCPOL_OTG |
			USBCTRL_OEXTEN |
			USBCTRL_XCSO |
			USBCTRL_OLKEN |
			USBCTRL_OWIE);
		val |= USBCTRL_PP_OTG |
			USBCTRL_OPM;
#endif
	}
	DBG(0, "%s: Changing USBCTRL from %08lx to %08lx\n", __FUNCTION__,
		usb_reg_read(otg_base, REG_USBCTRL), val);
	usb_reg_write(val, otg_base, REG_USBCTRL);
	local_irq_restore(flags);

	val = usb_reg_read(base, REG_PORTSC1);
	if (pdev->id == 1) {
		/* select serial transceiver for USBH2 port */
		val = (val & ~(3 << 30)) | (3 << 30);
	} else {
		/* select UTMI transceiver for USBOTG port */
		val = (val & ~(3 << 30)) | (0 << 30) | (1 << 28);
	}
	DBG(0, "%s: Changing %s_PORTSC1 from %08lx to %08lx\n", __FUNCTION__, name,
		usb_reg_read(base, REG_PORTSC1), val);
	usb_reg_write(val, base, REG_PORTSC1);
	if (pdev->id == 0) {
		/* assert USB reset to correctly detect state of ID pin */
		usb_reg_write(usb_reg_read(base, REG_USBCMD) & ~USBCMD_RUN,
			base, REG_USBCMD);
		while (usb_reg_read(base, REG_USBCMD) & USBCMD_RUN) {
			cpu_relax();
			loops++;
		}
		usb_reg_write(usb_reg_read(base, REG_USBCMD) | USBCMD_RST,
			base, REG_USBCMD);
		while (usb_reg_read(base, REG_USBCMD) & USBCMD_RST) {
			cpu_relax();
			loops++;
		}
		DBG(0, "USB controller reset for OTG port finished after %u loops\n", loops);
		msleep(100);
	}
	return 0;
}

#ifdef CONFIG_ARCH_MXC_EHCI_USBH2
/*
 * The USB power switch (MAX893L) used on the STK5 base board
 * produces a pulse (~100us) on the OC output whenever
 * the ON input is activated. This disturbs the USB controller.
 * As a workaround don't use USB power switching.
 * If you have a hardware that works cleanly you may
 * #define USE_USB_PWR to enable port power control for
 * the EHCI controller.
 */
static struct pad_desc karo_tx25_usbh2_pads[] = {
#ifdef USE_USB_PWR
	MX25_PAD_D9__USBH2_PWR,
	MX25_PAD_D8__USBH2_OC,
#else
	MX25_PAD_D9__GPIO_4_11,
	MX25_PAD_D8__GPIO_4_12,
#endif
};

static int tx25_usbh2_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = MX25_IO_ADDRESS(OTG_BASE_ADDR + 0x400);
#ifndef USE_USB_PWR
	const int pwr_gpio = 3 * 32 + 11;
#endif
	ret = mxc_iomux_v3_setup_multiple_pads(karo_tx25_usbh2_pads,
					ARRAY_SIZE(karo_tx25_usbh2_pads));
#ifdef USE_USB_PWR
	if (ret) {
		return ret;
	}
#else
	ret = gpio_request(pwr_gpio, "USBH2_PWR");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO %d\n", __FUNCTION__,
			pwr_gpio);
		mxc_iomux_v3_release_multiple_pads(karo_tx25_usbh2_pads,
						ARRAY_SIZE(karo_tx25_usbh2_pads));
		return ret;
	}

	gpio_direction_output(pwr_gpio, 1);
#endif
	if (ret != 0) {
		mxc_iomux_v3_release_multiple_pads(karo_tx25_usbh2_pads,
						ARRAY_SIZE(karo_tx25_usbh2_pads));
		goto exit;
	}
	ret = tx25_usb_init(pdev, base, 1);

exit:
#ifndef USE_USB_PWR
	gpio_free(pwr_gpio);
#endif
	return ret;
}

static int tx25_usbh2_exit(struct platform_device *pdev)
{
	mxc_iomux_v3_release_multiple_pads(karo_tx25_usbh2_pads,
					ARRAY_SIZE(karo_tx25_usbh2_pads));
	return 0;
}

static struct mxc_usbh_platform_data tx25_usbh2_data = {
	.init = tx25_usbh2_init,
	.exit = tx25_usbh2_exit,
};

int tx25_usbh2_register(void)
{
	int ret;

	ret = mxc_register_device(&mxc_usbh2_device, &tx25_usbh2_data);
	return ret;
}
device_initcall(tx25_usbh2_register);
#endif // CONFIG_ARCH_MXC_EHCI_USBH2

#ifdef CONFIG_ARCH_MXC_EHCI_USBOTG
static struct pad_desc karo_tx25_usbotg_pads[] = {
#ifdef USE_USB_PWR
	MX25_PAD_GPIO_A__USBOTG_PWR,
	MX25_PAD_GPIO_B__USBOTG_OC,
#else
	MX25_PAD_GPIO_A__GPIO_A,
	MX25_PAD_GPIO_B__GPIO_B,
#endif
};

static int tx25_usbotg_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = MX25_IO_ADDRESS(OTG_BASE_ADDR + 0x000);
#ifndef USE_USB_PWR
	const int pwr_gpio = 0 * 32 + 0;
#endif
	DBG(0, "%s: \n", __FUNCTION__);

	ret = mxc_iomux_v3_setup_multiple_pads(karo_tx25_usbotg_pads,
					ARRAY_SIZE(karo_tx25_usbotg_pads));
#ifdef USE_USB_PWR
	if (ret) {
		return ret;
	}
#else
	ret = gpio_request(pwr_gpio, "USBOTG_PWR");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO %d\n", __FUNCTION__,
			pwr_gpio);
		mxc_iomux_v3_release_multiple_pads(karo_tx25_usbh2_pads,
						ARRAY_SIZE(karo_tx25_usbh2_pads));
		return ret;
	}

	gpio_direction_output(pwr_gpio, 1);
#endif
	if (ret != 0) {
		mxc_iomux_v3_release_multiple_pads(karo_tx25_usbotg_pads,
						ARRAY_SIZE(karo_tx25_usbotg_pads));
		goto exit;
	}
	ret = tx25_usb_init(pdev, base, 1);

exit:
#ifndef USE_USB_PWR
	gpio_free(pwr_gpio);
#endif
	return ret;
}

static int tx25_usbotg_exit(struct platform_device *pdev)
{
	mxc_iomux_v3_release_multiple_pads(karo_tx25_usbotg_pads,
					ARRAY_SIZE(karo_tx25_usbotg_pads));
	return 0;
}

static struct mxc_usbh_platform_data tx25_usbotg_data = {
	.init = tx25_usbotg_init,
	.exit = tx25_usbotg_exit,
};

int tx25_usbotg_register(void)
{
	int ret;

	ret = mxc_register_device(&mxc_otg_device, &tx25_usbotg_data);
	return ret;
}
device_initcall(tx25_usbotg_register);
#endif // CONFIG_ARCH_MXC_EHCI_USBOTG
#endif // CONFIG_ARCH_MXC_EHCI_USBH2 || CONFIG_ARCH_MXC_EHCI_USBOTG

#if defined(CONFIG_USB_FSL_USB2) || defined(CONFIG_USB_FSL_USB2_MODULE)
static struct fsl_usb2_platform_data tx25_usb_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI,
};

static int __init tx25_usb_gadget_register(void)
{
	return mxc_register_device(&mxc_otg_udc_device, &tx25_usb_pdata);
}
device_initcall(tx25_usb_gadget_register);
#endif

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
#define STK5_GPIO_LED		(GPIO_PORTB | 7)

static struct pad_desc stk5_led_pads[] = {
	MX25_PAD_A21__GPIO_2_7,
};

static struct gpio_led stk5_leds[] = {
	{
		.name = "GPIO-LED",
		.default_trigger = "heartbeat",
		.gpio = STK5_GPIO_LED,
	},
};

static struct gpio_led_platform_data stk5_led_data = {
	.leds = stk5_leds,
	.num_leds = ARRAY_SIZE(stk5_leds),
};

static struct platform_device stk5_led_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &stk5_led_data,
	},
};

static int __init stk5_led_init(void)
{
	int ret;

	ret = mxc_iomux_v3_setup_multiple_pads(stk5_led_pads,
					ARRAY_SIZE(stk5_led_pads));
	if (ret)
		DBG(0, "%s: Failed to setup PADS for LED: %d\n",
			__FUNCTION__, ret);

	ret = gpio_request(STK5_GPIO_LED, "LED");
	if (ret)
		DBG(0, "%s: Failed to request GPIO%d_%d for LED: %d\n",
			__FUNCTION__, STK5_GPIO_LED / 32,
			STK5_GPIO_LED % 32, ret);

	gpio_direction_output(STK5_GPIO_LED, 0);
	/* free the GPIO, so that the LED driver can grab it */
	gpio_free(STK5_GPIO_LED);

	return ret;
}
arch_initcall(stk5_led_init);
#endif

#if defined(CONFIG_KEYBOARD_IMX) || defined(CONFIG_KEYBOARD_IMX_MODULE)
/*
 * This array is used for mapping keypad scancodes to keyboard keycodes.
 */
static const uint32_t tx25_kpd_keycodes[] = {
	/* specify your keymap with KEY(row, col, keycode), */
	KEY(0, 0, KEY_POWER),
};

static struct matrix_keymap_data tx25_keymap_data = {
	.keymap = tx25_kpd_keycodes,
	.keymap_size = ARRAY_SIZE(tx25_kpd_keycodes),
};

static struct matrix_keypad_platform_data tx25_keypad_data = {
	.keymap_data = &tx25_keymap_data,
	.wakeup = 1,
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

#if (!defined(CONFIG_LCD_GPM1076A0) && !defined(CONFIG_LCD_GPM1076A0_MODULE)) && \
	(defined(CONFIG_FB_IMX) || defined(CONFIG_FB_IMX_MODULE))
#define STK5_LCD_BACKLIGHT_GPIO		(GPIO_PORTA | 26)
#define STK5_LCD_RESET_GPIO		(GPIO_PORTB | 4)
#define STK5_LCD_POWER_GPIO		(GPIO_PORTB | 5)

/*
 * Setup GPIO for LCDC device to be active
 *
 */
static struct pad_desc mx25_lcdc_gpios[] = {
#ifdef STK5_LCD_BACKLIGHT_GPIO
	MX25_PAD_A18__GPIO_2_4,		/* LCD Reset (active LOW) */
#if !defined(CONFIG_MXC_PWM) && !defined(CONFIG_MXC_PWM_MODULE)
	MX25_PAD_PWM__GPIO_1_26,	/* LCD Backlight brightness 0: full 1: off */
#endif
	MX25_PAD_A19__GPIO_2_5,		/* LCD Power Enable 0: off 1: on */
#endif
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

static int stk5_gpio_lcdc_active(struct platform_device *dev)
{
	int ret;

	DBG(0, "%s: Setting up GPIO pins for LCD\n", __FUNCTION__);
	ret = mxc_iomux_v3_setup_multiple_pads(mx25_lcdc_gpios,
					ARRAY_SIZE(mx25_lcdc_gpios));
	if (ret) {
		DBG(0, "%s: Failed to setup GPIO pins for LCD: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
#ifdef STK5_LCD_BACKLIGHT_GPIO
	ret = gpio_request(STK5_LCD_POWER_GPIO, "LCD POWER");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO for LCD POWER: %d\n",
			__FUNCTION__, ret);
		goto release_pins;
	}
#if !defined(CONFIG_MXC_PWM) && !defined(CONFIG_MXC_PWM_MODULE)
	ret = gpio_request(STK5_LCD_BACKLIGHT_GPIO, "LCD Backlight");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO for backlight control: %d\n",
			__FUNCTION__, ret);
		goto free_gpio1;
	}
#endif
	ret = gpio_request(STK5_LCD_RESET_GPIO, "LCD RESET");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO for LCD RESET: %d\n",
			__FUNCTION__, ret);
		goto free_gpio2;
	}

	gpio_direction_output(STK5_LCD_POWER_GPIO, 1);
#if !defined(CONFIG_MXC_PWM) && !defined(CONFIG_MXC_PWM_MODULE)
	gpio_direction_output(STK5_LCD_BACKLIGHT_GPIO, 1);
#endif
	gpio_direction_output(STK5_LCD_RESET_GPIO, 0);
#endif
	return 0;

free_gpio2:
#if !defined(CONFIG_MXC_PWM) && !defined(CONFIG_MXC_PWM_MODULE)
	gpio_free(STK5_LCD_BACKLIGHT_GPIO);
free_gpio1:
#endif
	gpio_free(STK5_LCD_POWER_GPIO);
release_pins:
	mxc_iomux_v3_release_multiple_pads(mx25_lcdc_gpios,
					ARRAY_SIZE(mx25_lcdc_gpios));
	return ret;
}

/*
 * Setup GPIO for LCDC device to be inactive
 *
 */
static void stk5_gpio_lcdc_inactive(struct platform_device *dev)
{
	mxc_iomux_v3_release_multiple_pads(mx25_lcdc_gpios,
					ARRAY_SIZE(mx25_lcdc_gpios));
}

#ifdef STK5_LCD_BACKLIGHT_GPIO
#if !defined(CONFIG_MXC_PWM) && !defined(CONFIG_MXC_PWM_MODULE)
static void stk5_lcdc_backlight(int on)
{
	DBG(0, "%s: Switching LCD backlight %s\n", __FUNCTION__, on ? "on" : "off");
	if (on) {
		gpio_set_value(STK5_LCD_BACKLIGHT_GPIO, 0);
	} else {
		gpio_set_value(STK5_LCD_BACKLIGHT_GPIO, 1);
	}
}
#else
#define stk5_lcdc_backlight	NULL
#endif

static void stk5_lcdc_power(int on)
{
	DBG(0, "%s: Switching LCD reset %s\n", __FUNCTION__, on ? "off" : "on");
	if (on) {
		gpio_set_value(STK5_LCD_POWER_GPIO, 1);
		gpio_set_value(STK5_LCD_RESET_GPIO, 1);
	} else {
		gpio_set_value(STK5_LCD_RESET_GPIO, 0);
		gpio_set_value(STK5_LCD_POWER_GPIO, 0);
	}
}
#else
#define stk5_lcdc_backlight	NULL
#define stk5_lcdc_power		NULL
#endif

static struct imx_fb_videomode stk5_fb_modes[] = {
	{
		.bpp	= 8,
		.mode = {
			.name = "G-ETV570G0DMU-8",
			.pixclock	= 33333,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.left_margin	= 96,
			.right_margin	= 80,

			.vsync_len	= 3,
			.upper_margin	= 46,
			.lower_margin	= 39,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_BPIX_8 |
		PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL | PCR_END_BYTE_SWAP,
	},
	{
		.bpp	= 16,
		.mode = {
			.name = "G-ETV570G0DMU-16",
			.pixclock	= 33333,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.left_margin	= 96,
			.right_margin	= 80,

			.vsync_len	= 3,
			.upper_margin	= 46,
			.lower_margin	= 39,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 |
		PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp	= 32,
		.mode = {
			.name = "G-ETV570G0DMU-18",
			.pixclock	= 33333,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.left_margin	= 96,
			.right_margin	= 80,

			.vsync_len	= 3,
			.upper_margin	= 46,
			.lower_margin	= 39,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_18 |
		PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL | PCR_END_SEL,
	},
	{
		.bpp	= 8,
		.mode = {
			.name = "Xenarc_700_Y_VGA-8",
			.pixclock	= 33333,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.left_margin	= 91,
			.right_margin	= 91,

			.vsync_len	= 7,
			.upper_margin	= 30,
			.lower_margin	= 44,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_BPIX_8 |
		PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL | PCR_END_BYTE_SWAP,
	},
	{
		.bpp	= 16,
		.mode = {
			.name = "Xenarc_700_Y_VGA-16",
			.pixclock	= 33333,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.right_margin	= 91,
			.left_margin	= 91,

			.vsync_len	= 7,
			.lower_margin	= 30,
			.upper_margin	= 44,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 |
		PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp	= 32,
		.mode = {
			.name = "Xenarc_700_Y_VGA-18",
			.pixclock	= 33333,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.right_margin	= 91,
			.left_margin	= 91,

			.vsync_len	= 7,
			.lower_margin	= 30,
			.upper_margin	= 44,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_18 |
		PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL | PCR_END_SEL,
	},
	{
		.bpp	= 16,
		.mode = {
			.name = "SHARP_LQ10D42-16",

			.pixclock	= 34576,
			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.right_margin	= 138 + 1,
			.left_margin	= 118 + 3,

			.vsync_len	= 7,
			.lower_margin	= 60,
			.upper_margin	= 28,

		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 |
		PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp	= 16,
		.mode = {
			.name = "SHARP_LQ104V1DG61-16",

			.pixclock	= 40000,
			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 32,
			.right_margin	= 32 + 1,
			.left_margin	= 0 + 3,

			.vsync_len	= 35,
			.lower_margin	= 0,
			.upper_margin	= 0,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 | PCR_BPIX_16 |
		PCR_FLMPOL | PCR_LPPOL | PCR_CLKPOL | PCR_SCLK_SEL,
	},
};

static struct imx_fb_platform_data stk5_fb_data = {
	.init		= stk5_gpio_lcdc_active,
	.exit		= stk5_gpio_lcdc_inactive,
	.lcd_power	= stk5_lcdc_power,
	.backlight_power = stk5_lcdc_backlight,

	.mode		= stk5_fb_modes,
	.num_modes	= ARRAY_SIZE(stk5_fb_modes),

	.dmacr		= 0x80040060,

	.cmap_greyscale	= 0,
	.cmap_inverse	= 0,
	.cmap_static	= 0,

	.fixed_screen_cpu = NULL,
};
#endif

#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
static struct pad_desc stk5_pwm_pads[] = {
	MX25_PAD_PWM__PWM,
};

static int stk5_backlight_init(struct device *dev)
{
	int ret;
	ret = mxc_iomux_v3_setup_pad(&stk5_pwm_pads[0]);
	return ret;
}

static int stk5_backlight_notify(int brightness)
{
	DBG(0, "%s: brightness=%d->%d\n", __FUNCTION__, brightness,
		100 - brightness);
	return 100 - brightness;
}

static void stk5_backlight_exit(struct device *dev)
{
	mxc_iomux_v3_release_pad(&stk5_pwm_pads[0]);
}

static struct platform_pwm_backlight_data stk5_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = KHZ2PICOS(200), /* kHz -> ps is the same as Hz -> ns */
	.init = stk5_backlight_init,
	.notify = stk5_backlight_notify,
	.exit = stk5_backlight_exit,
};

static struct platform_device stk5_backlight_pwm_device = {
	.name = "pwm-backlight",
	.dev = {
		.platform_data = &stk5_backlight_data,
	},
};
#endif /* CONFIG_BACKLIGHT_PWM || CONFIG_BACKLIGHT_PWM_MODULE */

#if defined(CONFIG_MMC_SDHCI_MXC) || defined(CONFIG_MMC_SDHCI_MXC_MODULE)
#define SDHC1_CD_GPIO	(GPIO_PORTD | 4)

/*
 * Resource definition for the SDHC1
 */
static struct resource stk5_sdhc1_resources[] = {
	{
		.start = MMC_SDHC1_BASE_ADDR,
		.end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_SDHC1,
		.end = MXC_INT_SDHC1,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = gpio_to_irq(SDHC1_CD_GPIO),
		.end = gpio_to_irq(SDHC1_CD_GPIO),
		.flags = IORESOURCE_IRQ,
	},
};

static inline int stk5_esdhci_get_irq(int id)
{
	int irq;

	switch (id) {
	case 0:
		irq = stk5_sdhc1_resources[2].start;
		break;
	default:
		BUG();
	}
	return irq;
}

static const char *stk5_esdhci_irqdesc[] = {
	"ESDHCI card 0 detect",
};

static struct pad_desc stk5_sdhc_pads[] = {
	MX25_PAD_SD1_CMD__SD1_CMD,
	MX25_PAD_SD1_CLK__SD1_CLK,
	MX25_PAD_SD1_DATA0__SD1_DATA0,
	MX25_PAD_SD1_DATA1__SD1_DATA1,
	MX25_PAD_SD1_DATA2__SD1_DATA2,
	MX25_PAD_SD1_DATA3__SD1_DATA3,
	/* card detect GPIO */
	MX25_PAD_BCLK__GPIO_4_4,
};

static int tx51_esdhci_status(struct device *dev)
{
	return !!gpio_get_value(SDHC1_CD_GPIO);
}

static int stk5_esdhci_init(struct device *dev, irqreturn_t (*esdhci_detect_irq)(int, void *),
			void *data)
{
	int err;
	struct mmc_host *host = data;
	int id = to_platform_device(dev)->id;
	int irq = stk5_esdhci_get_irq(id);

	err = mxc_iomux_v3_setup_multiple_pads(stk5_sdhc_pads,
					ARRAY_SIZE(stk5_sdhc_pads));
	if (err) {
		return err;
	}

	host->caps |= MMC_CAP_4_BIT_DATA;

	DBG(0, "%s: Requesting IRQ %d\n", __FUNCTION__, irq);
	err = request_irq(irq, esdhci_detect_irq, IRQF_DISABLED |
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			stk5_esdhci_irqdesc[id], data);
	if (err) {
		dev_err(dev, "Error %d requesting ESDHCI card detect IRQ %d\n",
			err, irq);
		return err;
	}
	device_set_wakeup_capable(dev, 1);

	return 0;
}

static void stk5_esdhci_exit(struct device *dev, void *data)
{
	int id = to_platform_device(dev)->id;
	int irq = stk5_esdhci_get_irq(id);

	DBG(0, "%s: Freeing IRQ %d\n", __FUNCTION__, irq);
	free_irq(irq, data);
	mxc_iomux_v3_release_multiple_pads(stk5_sdhc_pads,
					ARRAY_SIZE(stk5_sdhc_pads));
}

static int stk5_esdhci_suspend(struct device *dev)
{
	int id = to_platform_device(dev)->id;
	int irq = stk5_esdhci_get_irq(id);

	if (device_may_wakeup(dev)) {
		DBG(0, "%s: Enabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return enable_irq_wake(irq);
	}
	return 0;
}

static int stk5_esdhci_resume(struct device *dev)
{
	int id = to_platform_device(dev)->id;
	int irq = stk5_esdhci_get_irq(id);

	if (device_may_wakeup(dev)) {
		DBG(0, "%s: Disabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return disable_irq_wake(irq);
	}
	return 0;
}

static struct mxc_sdhci_platform_data stk5_sdhc1_data = {
	.ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34,
	.init = stk5_esdhci_init,
	.exit = stk5_esdhci_exit,
	.suspend = stk5_esdhci_suspend,
	.resume = stk5_esdhci_resume,
	.status = tx51_esdhci_status,
	.min_clk = 150000,
	.max_clk = 25000000,
	.detect_delay = 100,
};

static struct platform_device stk5_sdhc1_device = {
	.name = "sdhci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &stk5_sdhc1_data,
	},
	.num_resources = ARRAY_SIZE(stk5_sdhc1_resources),
	.resource = stk5_sdhc1_resources,
};
#endif

#if defined(CONFIG_AC97_BUS) || defined(CONFIG_AC97_BUS_MODULE)
static u64 stk5_dma_mask = DMA_BIT_MASK(32);

static struct pad_desc stk5_ac97_pads_on[] = {
	MX25_PAD_VSTBY_ACK__GPIO_3_18,	/* AC97 Reset */
	MX25_PAD_EXT_ARMCLK__GPIO_3_15,	/* Codec IRQ */

	MX25_PAD_RW__AUD4_TXFS,
	MX25_PAD_EB0__AUD4_TXD,
	MX25_PAD_EB1__AUD4_RXD,
	MX25_PAD_OE__AUD4_TXC,
};

static struct pad_desc stk5_ac97_pads_off[] = {
	MX25_PAD_VSTBY_ACK__GPIO_3_18, /* UCB1400 Reset */
	MX25_PAD_EXT_ARMCLK__GPIO_3_15,	/* Codec IRQ */

	MX25_PAD_RW__GPIO_3_25,
	MX25_PAD_EB0__GPIO_2_12,
	MX25_PAD_EB1__AUD4_RXD,
	MX25_PAD_OE__AUD4_TXC,
};

static int stk5_ac97_gpios[] = {
	/* configure the UCB1400 strap pins to the correct values */
	GPIO_PORTC | 18,
	GPIO_PORTC | 25,
	GPIO_PORTB | 12,
};

static int stk5_ac97_init(struct platform_device *dev, int ssi)
{
	int ret;
	const int irq = gpio_to_irq(GPIO_PORTC | 15);
	int i;

	DBG(0, "%s: \n", __FUNCTION__);
	/* AC97 is hardwired to AUDMUX port 4 */
	ret = mxc_audmux_configure_sync_slave(ssi + 1, 4, MXC_AUDMUX_MODE_AC97);
	if (ret) {
		DBG(0, "%s: Failed to configure AUDMUX: %d\n",
			__FUNCTION__, ret);
		return ret;
	}

	/*
	 * IRQs are disabled for probing whenever enable_irq() is called.
	 * Thus reenable the UCB1400 IRQ for probing in case the ucb1400-ts
	 * module has been reloaded.
	 */
	DBG(0, "%s: Enabling IRQ%d for probing\n", __FUNCTION__, irq);
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	/* configure RESET_OUT, AUD4_TXFS, AUD4_TXD as GPIO
	 * to prevent AC97 codecs from entering test mode
	 * due to TXFS or TXD sampled HIGH when RESET is deasserted
	 */
	ret = mxc_iomux_v3_setup_multiple_pads(stk5_ac97_pads_off,
					ARRAY_SIZE(stk5_ac97_pads_off));
	if (ret) {
		DBG(0, "%s: Failed to configure AC97 pads: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
	for (i = 0; i < ARRAY_SIZE(stk5_ac97_gpios); i++) {
		int gpio = stk5_ac97_gpios[i];

		ret = gpio_request(gpio, "AC97");
		if (ret < 0) {
			DBG(0, "%s: Failed to request GPIO%d_%d: %d\n", __FUNCTION__,
				gpio / 32 + 1, gpio % 32, ret);
			goto rel_mux;
		}
		DBG(1, "%s: Switching GPIO%d_%d to 0\n",
			__FUNCTION__, gpio / 32 + 1, gpio % 32);
		gpio_direction_output(gpio, 0);
	}
	udelay(1);
	/* Deassert UCB1400 RESET */
	DBG(1, "%s: Switching GPIO%d_%d to 1\n",
		__FUNCTION__, stk5_ac97_gpios[0] / 32 + 1,
		stk5_ac97_gpios[0] % 32);
	gpio_set_value(stk5_ac97_gpios[0], 1);

	mxc_iomux_v3_release_multiple_pads(stk5_ac97_pads_off,
					ARRAY_SIZE(stk5_ac97_pads_off));

	ret = mxc_iomux_v3_setup_multiple_pads(stk5_ac97_pads_on,
					ARRAY_SIZE(stk5_ac97_pads_on));
	if (ret != 0) {
		DBG(0, "%s: Failed to reconfigure AC97 pads: %d\n",
			__FUNCTION__, ret);
		goto rel_gpio;
	}
	return ret;

rel_mux:
	mxc_iomux_v3_release_multiple_pads(stk5_ac97_pads_off,
					ARRAY_SIZE(stk5_ac97_pads_off));
rel_gpio:
	while (--i >= 0) {
		int gpio = stk5_ac97_gpios[i];
#ifdef DEBUG
		int grp = gpio / 32 + 1;
		int ofs = gpio % 32;

		DBG(0, "%s: Freeing GPIO%d_%d\n", __FUNCTION__,
			grp, ofs);
#endif
		gpio_free(gpio);
	}
	return ret;
}

static void stk5_ac97_exit(struct platform_device *dev)
{
	int i;

	DBG(0, "%s: Releasing AC97 GPIO pins\n", __FUNCTION__);
	mxc_iomux_v3_release_multiple_pads(stk5_ac97_pads_on,
					ARRAY_SIZE(stk5_ac97_pads_on));
	for (i = 0; i < ARRAY_SIZE(stk5_ac97_gpios); i++) {
		int gpio = stk5_ac97_gpios[i];
#ifdef DEBUG
		int grp = gpio / 32 + 1;
		int ofs = gpio % 32;

		DBG(0, "%s: Freeing GPIO%d_%d\n", __FUNCTION__,
			grp, ofs);
#endif
		gpio_free(gpio);
	}
}

static struct mxc_ac97_audio_ops stk5_ac97_ops = {
	.init = stk5_ac97_init,
	.exit = stk5_ac97_exit,
	.startup = NULL,
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
	.priv = NULL,
};

static struct platform_device stk5_ac97_device = {
	.name		= "mxc-ac97",
	.id		= -1,
	.dev = {
		.dma_mask = &stk5_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &stk5_ac97_ops,
	},
};

static int __init stk5_ac97_register(void)
{
	if (stk5_board_rev < 3) {
		DBG(0, "%s: Registering device %s\n", __FUNCTION__,
			stk5_ac97_device.name);
		return platform_device_register(&stk5_ac97_device);
	}
	return 0;
}
arch_initcall(stk5_ac97_register);
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
static struct pad_desc stk5_sgtl5000_pads[] = {
	MX25_PAD_RW__AUD4_TXFS,
	MX25_PAD_EB0__AUD4_TXD,
	MX25_PAD_EB1__AUD4_RXD,
	MX25_PAD_OE__AUD4_TXC,
};

static int stk5_sgtl5000_plat_init(void)
{
	int ret;

	DBG(0, "%s: \n", __FUNCTION__);
	ret = mxc_iomux_v3_setup_multiple_pads(stk5_sgtl5000_pads,
					ARRAY_SIZE(stk5_sgtl5000_pads));
	return ret;
}

static void stk5_sgtl5000_plat_finit(void)
{
	DBG(0, "%s: \n", __FUNCTION__);

	mxc_iomux_v3_release_multiple_pads(stk5_sgtl5000_pads,
					ARRAY_SIZE(stk5_sgtl5000_pads));
}

static struct mxc_audio_platform_data stk5_sgtl5000_data = {
	.ssi_num = 0,
	.src_port = 1,
	.ext_port = 4,
	.sysclk = 26000000,
	.init = stk5_sgtl5000_plat_init,
	.finit = stk5_sgtl5000_plat_finit,
};

static struct platform_device stk5_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
	.dev = {
		.platform_data = &stk5_sgtl5000_data,
	},
};

static int __init stk5_sgtl5000_init(void)
{
	if (stk5_board_rev >= 3) {
		DBG(0, "%s: Registering device %s\n", __FUNCTION__,
			stk5_sgtl5000_device.name);
		return platform_device_register(&stk5_sgtl5000_device);
	}
	return 0;
}
arch_initcall(stk5_sgtl5000_init);
#endif /* CONFIG_SND_SOC_IMX_3STACK_SGTL5000 */

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
#define TSC2007_PEN_GPIO		(GPIO_PORTC | 15)

static struct pad_desc stk5_tsc2007_pads[] = {
	MX25_PAD_EXT_ARMCLK__GPIO_3_15,
};

static int stk5_tsc2007_init(void)
{
	int ret;

	ret = mxc_iomux_v3_setup_multiple_pads(stk5_tsc2007_pads,
					ARRAY_SIZE(stk5_tsc2007_pads));
	if (ret)
		return ret;
	ret = gpio_request(TSC2007_PEN_GPIO, "TSC2007");
	if (ret) {
		mxc_iomux_v3_release_multiple_pads(stk5_tsc2007_pads,
						ARRAY_SIZE(stk5_tsc2007_pads));
		return ret;
	}
	ret = gpio_direction_input(TSC2007_PEN_GPIO);
	return ret;
}

static void stk5_tsc2007_exit(void)
{
	mxc_iomux_v3_release_multiple_pads(stk5_tsc2007_pads,
					ARRAY_SIZE(stk5_tsc2007_pads));
	gpio_free(TSC2007_PEN_GPIO);
}

static int stk5_get_pendown(void)
{
#ifndef DEBUG
	return !gpio_get_value(TSC2007_PEN_GPIO);
#else
	int val = gpio_get_value(TSC2007_PEN_GPIO);

	DBG(0, "%s: TS pen is %s\n", __FUNCTION__, val ? "up" : "down");
	return !val;
#endif
}

static struct tsc2007_platform_data stk5_tsc2007_pdata = {
	.model = 2007,
	.x_plate_ohms = 660,
	.get_pendown_state = stk5_get_pendown,
	.clear_penirq = NULL,
	.init_platform_hw = stk5_tsc2007_init,
	.exit_platform_hw = stk5_tsc2007_exit,
};
#endif /* CONFIG_TOUCHSCREEN_TSC2007 || CONFIG_TOUCHSCREEN_TSC2007_MODULE */

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
#if defined(CONFIG_I2C_IMX) || defined(CONFIG_I2C_IMX_MODULE)
static struct pad_desc stk5_i2c1_pins[] = {
	MX25_PAD_I2C1_CLK__I2C1_CLK,
	MX25_PAD_I2C1_DAT__I2C1_DAT,
};
#elif defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
static struct pad_desc stk5_i2c1_pins[] = {
	MX25_PAD_I2C1_CLK__GPIO_1_12,
	MX25_PAD_I2C1_DAT__GPIO_1_13,
};
#else
#error No suitable I2C bus driver configured
#endif

static int stk5_i2c1_init(struct device *dev)
{
	return mxc_iomux_v3_setup_multiple_pads(stk5_i2c1_pins,
						ARRAY_SIZE(stk5_i2c1_pins));
}

static void stk5_i2c1_exit(struct device *dev)
{
	mxc_iomux_v3_release_multiple_pads(stk5_i2c1_pins,
					   ARRAY_SIZE(stk5_i2c1_pins));
}

static struct imxi2c_platform_data stk5_i2c1_data = {
	.bitrate = 100000,
	.init = stk5_i2c1_init,
	.exit = stk5_i2c1_exit,
};

static struct i2c_board_info stk5_i2c1_boardinfo[] __initdata = {
#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
	{
		I2C_BOARD_INFO("sgtl5000-i2c", 0x0a),
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_TSC2007) || defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
#define TSC2007_PEN_GPIO		(GPIO_PORTC | 15)
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq = gpio_to_irq(TSC2007_PEN_GPIO),
		.platform_data = &stk5_tsc2007_pdata,
	},
#endif
};

int __init stk5_i2c1_register(void)
{
	int ret;

	if (stk5_board_rev >= 3) {
		DBG(0, "%s: Registering I2C1 bus\n", __FUNCTION__);
		ret = mxc_register_device(&mx25_i2c1_device, &stk5_i2c1_data);
		if (ret != 0) {
			printk(KERN_ERR "Failed to register I2C1 device: %d\n", ret);
			return ret;
		}

		ret = i2c_register_board_info(0, stk5_i2c1_boardinfo,
					ARRAY_SIZE(stk5_i2c1_boardinfo));
		if (ret != 0)
			printk(KERN_ERR "Failed to register I2C1 board info: %d\n", ret);
		return ret;
	}
	return 0;
}
device_initcall(stk5_i2c1_register);
#endif /* CONFIG_I2C || CONFIG_I2C_MODULE */

static struct platform_dev_list {
	struct platform_device *pdev;
	void *pdata;
} stk5_devices[] __initdata = {
#if defined(CONFIG_SERIAL_IMX) || defined(CONFIG_SERIAL_IMX_MODULE)
#if UART1_ENABLED
	{ &mxc_uart_device0, &stk5_uart_port_data, },
#endif
#if UART2_ENABLED
	{ &mxc_uart_device1, &stk5_uart_port_data, },
#endif
#if UART3_ENABLED
	{ &mxc_uart_device2, &stk5_uart_port_data, },
#endif
#if UART4_ENABLED
	{ &mxc_uart_device3, &stk5_uart_port_data, },
#endif
#if UART5_ENABLED
	{ &mxc_uart_device4, &stk5_uart_port_data, },
#endif
#endif
#if !defined(CONFIG_LCD_GPM1076A0) && !defined(CONFIG_LCD_GPM1076A0_MODULE)
#if defined(CONFIG_FB_IMX) || defined(CONFIG_FB_IMX_MODULE)
	{ .pdev = &mx25_fb_device, .pdata = &stk5_fb_data, },
#endif
#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
	{ .pdev = &stk5_backlight_pwm_device, .pdata = &stk5_backlight_data, },
#endif
#endif /* CONFIG_LCD_GPM1076A0 && !CONFIG_LCD_GPM1076A0_MODULE */
#if defined(CONFIG_MXC_PWM) || defined(CONFIG_MXC_PWM_MODULE)
	{ .pdev = &mxc_pwm_device0, },
#endif
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	{ .pdev = &stk5_led_device, },
#endif
#if defined(CONFIG_MMC_SDHCI_MXC) || defined(CONFIG_MMC_SDHCI_MXC_MODULE)
	{ .pdev = &stk5_sdhc1_device, },
#endif
#if defined(CONFIG_MXC_AUDMUX_V3) || defined(CONFIG_MXC_AUDMUX_V3_MODULE)
	{ .pdev = &mxc_audmux_v3_device, },
#endif
#if defined(CONFIG_KEYBOARD_IMX) || defined(CONFIG_KEYBOARD_IMX_MODULE)
	{ .pdev = &imx_keypad_device, .pdata = &tx25_keypad_data, },
#endif
};
#define STK5_NUM_DEVICES		ARRAY_SIZE(stk5_devices)

#ifdef CONFIG_MACH_TX25_CUSTOM
extern __init int stk5_custom_board_init(void);
#else
static inline int stk5_custom_board_init(void)
{
	return 0;
}
#endif

static __init int karo_stk5_board_init(void)
{
	int ret;
	int i;

	DBG(0, "%s: \n", __FUNCTION__);

	for (i = 0; i < STK5_NUM_DEVICES; i++) {
		if (stk5_devices[i].pdev == NULL) continue;
		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, stk5_devices[i].pdev, &stk5_devices[i].pdev->dev,
			stk5_devices[i].pdev->name);
		if (stk5_devices[i].pdata) {
			ret = mxc_register_device(stk5_devices[i].pdev,
						stk5_devices[i].pdata);
		} else {
			ret = platform_device_register(stk5_devices[i].pdev);
		}
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, stk5_devices[i].pdev->name, ret);
		}
	}
	DBG(0, "%s: Done\n", __FUNCTION__);
	return stk5_custom_board_init();
}
subsys_initcall(karo_stk5_board_init);

static int __init stk5_board_rev_setup(char *cmdline)
{
	get_option(&cmdline, &stk5_board_rev);
	DBG(0, "%s: Board rev set to 0x%02x\n",
		__FUNCTION__, stk5_board_rev);
	return 1;
}
__setup("stk5_board_rev=", stk5_board_rev_setup);
