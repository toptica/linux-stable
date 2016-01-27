/*
 * arch/arm/mach-mx2/stk5-baseboard.c
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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/spi/spi.h>
#include <linux/serial.h>
#include <linux/fsl_devices.h>
#include <linux/irq.h>
#include <linux/mmc/host.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/elliptec.h>
#include <linux/pwm.h>
#include <linux/dma-mapping.h>

#include <media/soc_camera.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/imxfb.h>
#include <mach/i2c.h>
#include <mach/mmc.h>
#include <mach/imx-uart.h>
#include <mach/mxc_ehci.h>
#include <mach/board-stk5.h>
#include <mach/spi.h>
#include <mach/sdhci.h>
#include <mach/mx25_camera.h>

#include "devices.h"
#include "karo.h"

#if defined(CONFIG_SERIAL_IMX) || defined(CONFIG_SERIAL_IMX_MODULE)
static struct pad_desc stk5_uart_pads[][4] = {
	{
		MX25_PAD_UART1_TXD__UART1_TXD,
		MX25_PAD_UART1_RXD__UART1_RXD,
		MX25_PAD_UART1_CTS__UART1_CTS,
		MX25_PAD_UART1_RTS__UART1_RTS,
	}, {
		MX25_PAD_UART2_TXD__UART2_TXD,
		MX25_PAD_UART2_RXD__UART2_RXD,
		MX25_PAD_UART2_CTS__UART2_CTS,
		MX25_PAD_UART2_RTS__UART2_RTS,
	}, {
		MX25_PAD_ECB__UART5_TXD_MUX,
		MX25_PAD_LBA__UART5_RXD_MUX,
		MX25_PAD_CS4__UART5_CTS,
		MX25_PAD_CS5__UART5_RTS,
#if 0
	}, {
		MX25_PAD_UART4_TXD__UART4_TXD,
		MX25_PAD_UART4_RXD__UART4_RXD,
		MX25_PAD_UART4_CTS__UART4_CTS,
		MX25_PAD_UART4_RTS__UART4_RTS,
	}, {
		MX25_PAD_UART5_TXD__UART5_TXD,
		MX25_PAD_UART5_RXD__UART5_RXD,
		MX25_PAD_UART5_CTS__UART5_CTS,
		MX25_PAD_UART5_RTS__UART5_RTS,
#endif
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

static struct imxuart_platform_data stk5_uart_ports[] = {
	{
		.init = stk5_uart_init,
		.exit = stk5_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = stk5_uart_init,
		.exit = stk5_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = stk5_uart_init,
		.exit = stk5_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = stk5_uart_init,
		.exit = stk5_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = stk5_uart_init,
		.exit = stk5_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = stk5_uart_init,
		.exit = stk5_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},
};

static struct platform_device *stk5_uart_devices[] = {
#if UART1_ENABLED
	&mxc_uart_device0,
#endif
#if UART2_ENABLED
	&mxc_uart_device1,
#endif
#if UART3_ENABLED
	&mxc_uart_device2,
#endif
#if UART4_ENABLED
	&mxc_uart_device3,
#endif
#if UART5_ENABLED
	&mxc_uart_device4,
#endif
};

static void __init karo_stk5_serial_init(void)
{
	int i;

	printk(KERN_INFO "%s: Registering serial ports\n",__FUNCTION__);
	for (i = 0; i < ARRAY_SIZE(stk5_uart_devices); i++) {
		int ret;
		int port = stk5_uart_devices[i]->id;

		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, stk5_uart_devices[i],
			&stk5_uart_devices[i]->dev, stk5_uart_devices[i]->name);
		ret = mxc_register_device(stk5_uart_devices[i],
					&stk5_uart_ports[port]);
		if (ret != 0) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, stk5_uart_devices[i]->name, ret);
		}
	}
}
#else
static void __init karo_stk5_serial_init(void)
{
	printk(KERN_INFO "%s: NO INITIALIZATION !!!!!!!!!!!!!!!!!!!!!!\n",__FUNCTION__);
}
#endif

#if defined(CONFIG_ARCH_MXC_EHCI_USBH2) || defined(CONFIG_ARCH_MXC_EHCI_USBOTG)

/* USB register offsets */
#define REG_USBCTRL		0x600
#define REG_PHY_CTRL		0x608

#define PHY_CTRL_USBEN		(1 << 24)

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
#define USBCTRL_OPM		(1 << 24)
#define USBCTRL_OEXTEN		(1 << 25)
#define USBCTRL_HEXTEN		(1 << 26)
#define USBCTRL_OWIE		(1 << 27)
#define USBCTRL_OUIE		(1 << 28)

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

static int tx25_usb_init(struct platform_device *pdev, void __iomem *base, int host_mode)
{
	unsigned long val;
	unsigned long flags;
	const char __maybe_unused *name = pdev->id ? "USBH2" : "USBOTG";
	unsigned int loops = 0;
	void __iomem *otg_base = MX25_IO_ADDRESS(OTG_BASE_ADDR);
	int ll = console_loglevel;

	console_loglevel = 8;
#if 0
	if (!(usb_reg_read(base, REG_USBSTS) & USBSTS_HCH)) {
		unsigned int loops = 0;

		DBG(0, "%s: %s[%p] is busy: %08lx\n", __FUNCTION__, name,
			base + REG_USBSTS, usb_reg_read(base, REG_USBSTS));
		usb_reg_write(usb_reg_read(base, REG_USBCTRL) & ~USBCMD_RUN,
			base, REG_USBCTRL);
		while (usb_reg_read(base, REG_USBCTRL) & USBCMD_RUN) {
			usb_reg_write(usb_reg_read(base, REG_USBCTRL) & ~USBCMD_RUN,
				base, REG_USBCTRL);
			cpu_relax();
			loops++;
			if (loops > 100)
				break;
		}
		if (usb_reg_read(base, REG_USBSTS) & USBSTS_HCH) {
			DBG(0, "USB controller idle after %u loops\n", loops);
		} else {
			DBG(0, "USB controller NOT idle after %u loops\n", loops);
		}
	}
#endif
	DBG(0, "%s: PHY_CTRL[%p]=%08lx\n", __FUNCTION__, otg_base + REG_PHY_CTRL,
		usb_reg_read(otg_base, REG_PHY_CTRL));
	DBG(0, "%s: USBCMD[%p]=%08lx\n", __FUNCTION__, base + REG_USBCMD,
		usb_reg_read(base, REG_USBCMD));
	DBG(0, "%s: USBSTS[%p]=%08lx\n", __FUNCTION__, base + REG_USBSTS,
		usb_reg_read(base, REG_USBSTS));

	/* reset USB Host controller */
	usb_reg_write(USBCMD_RST, base, REG_USBCMD);
	while (usb_reg_read(base, REG_USBCMD) & USBCMD_RST) {
		cpu_relax();
		loops++;
	}
	DBG(0, "USB controller reset finished after %u loops\n", loops);

	/* Switch to Host mode */
	val = usb_reg_read(base, REG_USBMODE);
	DBG(0, "%s: Changing %s_USBMODE from %08lx to %08lx\n", __FUNCTION__, name,
		val, val | (host_mode ? 0x3 : 0x02));
	usb_reg_write(val | (host_mode ? 0x3 : 0x02), base, REG_USBMODE);

	local_irq_save(flags);
	val = usb_reg_read(otg_base, REG_USBCTRL);
	if (pdev->id == 1) {
		val &= ~(USBCTRL_OCPOL_HST | USBCTRL_HPM |
			USBCTRL_HEXTEN | USBCTRL_HWIE);
		val |= USBCTRL_PP_HST | USBCTRL_HSDT | USBCTRL_USBTE |
			USBCTRL_XCSH | USBCTRL_PUE_DWN;
	} else {
		val &= ~(USBCTRL_OCPOL_OTG | USBCTRL_OPM |
			USBCTRL_OEXTEN | USBCTRL_OWIE);
		val |= USBCTRL_PP_OTG | USBCTRL_XCSO;
	}
	DBG(0, "%s: Changing %s_USBCTRL from %08lx to %08lx\n", __FUNCTION__, name,
		usb_reg_read(otg_base, REG_USBCTRL), val);
	usb_reg_write(val, otg_base, REG_USBCTRL);
	local_irq_restore(flags);

	val = usb_reg_read(base, REG_PORTSC1);
	if (pdev->id == 1) {
		/* select serial transceiver for USBH2 port */
		val = (val & ~(3 << 30)) | (3 << 30);
	} else {
		/* select UTMI transceiver for USBOTG port */
		val = (val & ~((3 << 30) | (0 << 28))) | (0 << 30);
	}
	DBG(0, "%s: Changing %s_PORTSC1 from %08lx to %08lx\n", __FUNCTION__, name,
		usb_reg_read(base, REG_PORTSC1), val);
	usb_reg_write(val, base, REG_PORTSC1);

	console_loglevel = ll;
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
#endif
	MX25_PAD_D8__USBH2_OC,
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

	ret = mxc_register_device(&mxc_usbh2, &tx25_usbh2_data);
	return ret;
}
device_initcall(tx25_usbh2_register);
#endif // CONFIG_ARCH_MXC_EHCI_USBH2

#ifdef CONFIG_ARCH_MXC_EHCI_USBOTG
static struct pad_desc karo_tx25_usbotg_pads[] = {
#ifdef USE_USB_PWR
	MX25_PAD_GPIO_A__USBOTG_PWR,
#endif
	MX25_PAD_GPIO_B__USBOTG_OC,
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

	ret = mxc_register_device(&mxc_otg, &tx25_usbotg_data);
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


#if defined(CONFIG_TOUCHSCREEN_MXC_ADC) || defined(CONFIG_TOUCHSCREEN_MXC_ADC_MODULE)

#define ADC_INTERNAL_REFERENCE 1
#define ADC_EXTERNAL_REFERENCE 0

static struct pad_desc mx25_adc_extmux_gpios[] = {
	MX25_PAD_A25__GPIO_2_11,
	MX25_PAD_A15__GPIO_2_1,
};

static int __init karo_tx25_adc_register(void)
{
	int ret;
	printk(KERN_INFO "registering ADC device ...\n");

	DBG(0, "%s: Setting up GPIO pins for external ADC muxing\n", __FUNCTION__);
	ret = mxc_iomux_v3_setup_multiple_pads(mx25_adc_extmux_gpios,
					ARRAY_SIZE(mx25_adc_extmux_gpios));
	if (ret) {
		DBG(0, "%s: Failed to setup GPIO pins for external ADC muxing: %d\n",
			__FUNCTION__, ret);
		return ret;
	}

	ret = mxc_register_device(&mx25_tsc_device, (void *) ADC_INTERNAL_REFERENCE);
	if (ret) {
		printk(KERN_WARNING "Failed to register TSC device: %d\n", ret);
	}
	return ret;
}
device_initcall(karo_tx25_adc_register);
#endif


#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led stk5_leds[] = {
	{
		.name = "GPIO-LED",
		.default_trigger = "heartbeat",
		.gpio = GPIO_PORTB | 7,
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
#endif

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
/*!
 * This array is used for mapping mx25 ADS keypad scancodes to input keyboard
 * keycodes.
 */
static u16 stk5_kpd_keycodes[] = {
	KEY_POWER,
};

static struct keypad_data stk5_keypad = {
	.rowmax = 1,
	.colmax = 1,
	.irq = MXC_INT_KPP,
	.learning = 0,
	//.delay = 2, /* unused in the driver! */
	.matrix = stk5_kpd_keycodes,
};

static struct resource stk5_kpp_resources[] = {
	{
		.start = MXC_INT_KPP,
		.end = MXC_INT_KPP,
		.flags = IORESOURCE_IRQ,
	},
};

/* stk5 keypad driver */
static struct platform_device stk5_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(stk5_kpp_resources),
	.resource = stk5_kpp_resources,
	.dev = {
		.platform_data = &stk5_keypad,
	},
};
#endif

#if defined(CONFIG_FB_IMX) || defined(CONFIG_FB_IMX_MODULE)
#define STK5_LCD_BACKLIGHT_GPIO		(GPIO_PORTA | 26)
#define STK5_LCD_RESET_GPIO		(GPIO_PORTB | 4)

/*
 * Setup GPIO for LCDC device to be active
 *
 */
static struct pad_desc mx25_lcdc_gpios[] = {
#ifdef STK5_LCD_BACKLIGHT_GPIO
	MX25_PAD_A18__GPIO_2_4, /* LCD Reset (active LOW) */
	MX25_PAD_PWM__GPIO_1_26, /* LCD Backlight brightness 0: full 1: off */
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
	ret = gpio_request(STK5_LCD_BACKLIGHT_GPIO, "LCD Backlight");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO for backlight control: %d\n",
			__FUNCTION__, ret);
		goto release_pins;
	}

	ret = gpio_request(STK5_LCD_RESET_GPIO, "LCD RESET");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO for LCD RESET: %d\n",
			__FUNCTION__, ret);
		goto free_gpio;
	}
	gpio_direction_output(STK5_LCD_BACKLIGHT_GPIO, 1);
	gpio_direction_output(STK5_LCD_RESET_GPIO, 0);
#endif
	return 0;

free_gpio:
	gpio_free(STK5_LCD_BACKLIGHT_GPIO);
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
static void stk5_lcdc_backlight(int on)
{
	DBG(0, "%s: Switching LCD backlight %s\n", __FUNCTION__, on ? "on" : "off");
	if (on) {
		gpio_set_value(STK5_LCD_BACKLIGHT_GPIO, 0);
	} else {
		gpio_set_value(STK5_LCD_BACKLIGHT_GPIO, 1);
	}
}

static void stk5_lcdc_power(int on)
{
	DBG(0, "%s: Switching LCD reset %s\n", __FUNCTION__, on ? "off" : "on");
	if (on) {
		gpio_set_value(STK5_LCD_RESET_GPIO, 1);
	} else {
		gpio_set_value(STK5_LCD_RESET_GPIO, 0);
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
			.name = "G-ETV570G0DMU",
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
		.pcr	= PCR_TFT | PCR_COLOR | PCR_END_BYTE_SWAP |
		PCR_BPIX_8 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	}, {
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
		.pcr	= PCR_TFT | PCR_COLOR | PCR_END_BYTE_SWAP |
		PCR_BPIX_8 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	}, {
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
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	}, {
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
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	}, {
		.bpp	= 16,
		.mode = {
			.name = "SHARP LQ10D42-16",

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
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	}, {
		.bpp	= 16,
		.mode = {
			.name = "SHARP LQ104V1DG61-16",

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
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_CLKPOL | PCR_SCLK_SEL,
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

static int __init karo_stk5_fb_register(void)
{
	int ret;

	ret = mxc_register_device(&mx25_fb_device, &stk5_fb_data);
	if (ret != 0) {
		DBG(0, "%s: Failed to register FB device: %d\n", __FUNCTION__, ret);
	}
	return ret;
}
#else
static inline int karo_stk5_fb_register(void)
{
	return 0;
}
#endif

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
	err = request_irq(irq, esdhci_detect_irq,
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

#if 0
static int stk5_esdhci_suspend(struct device *dev, pm_message_t state)
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
#endif

static struct mxc_sdhci_platform_data stk5_sdhc1_data = {
	.ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34,
	.init = stk5_esdhci_init,
	.exit = stk5_esdhci_exit,
	.status = tx51_esdhci_status,
	.min_clk = 150000,
	.max_clk = 25000000,
	.detect_delay = 100,
};
#if 0
static struct imxmmc_platform_data stk5_sdhc1_data = {
	.ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34,
	//.min_clk = 150000,
	//.max_clk = 25000000,
	//.detect_delay = 20,
	.init = stk5_mmc_init,
	.exit = stk5_mmc_exit,
//	.suspend = stk5_mmc_suspend,
//	.resume = stk5_mmc_resume,
};
#endif

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

#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)

static int mxcspi1_available_cs[5] = {-32,-31,-30,-29,20};

static struct spi_imx_master mxcspi1_data = {
	.num_chipselect = 5,
	.chipselect = mxcspi1_available_cs,
};


static struct pad_desc stk5_cspi1_pads[] = {
	MX25_PAD_CSPI1_MOSI__CSPI1_MOSI,
	MX25_PAD_CSPI1_MISO__CSPI1_MISO,
	MX25_PAD_CSPI1_SS0__CSPI1_SS0,
	MX25_PAD_CSPI1_SS1__CSPI1_SS1,
	MX25_PAD_CSPI1_SCLK__CSPI1_SCLK,
	MX25_PAD_CSPI1_RDY__CSPI1_RDY,
};

static struct spi_board_info spi_board_info[] __initdata = {
{
	.modalias	= "spidev",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 1040000,
	.bus_num	= 0,
	.chip_select	= 0,
},
{
	.modalias	= "max1111",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 1040000,
	.bus_num	= 0,
	.chip_select	= 1,
},
{
	.modalias	= "spidev",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 17000000,
	.bus_num	= 0,
	.chip_select	= 2,
},
{
	.modalias	= "max1111",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 200000,
	.bus_num	= 0,
	.chip_select	= 3,
},
};

static int __init stk5_spi_register(void)
{
	int ret;
	printk(KERN_INFO "%s: -----------------------------------------\n", __FUNCTION__);


	printk(KERN_INFO "Configuring CSPI1 pads\n");
	ret = mxc_iomux_v3_setup_multiple_pads(stk5_cspi1_pads,
					ARRAY_SIZE(stk5_cspi1_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure CSPI1 pads %d\n",ret);
		return ret;
	}
	ret = mxc_register_device(&mxc_spi_device0, &mxcspi1_data);
	printk(KERN_INFO "%s: mxc_register_device() returned: %d\n", __FUNCTION__, ret);
	if (ret) {
				printk(KERN_ERR "Failed to register devices %d\n",ret);
				return ret;
			}
	printk(KERN_INFO "%s: registering %d SPI devices\n",__FUNCTION__,ARRAY_SIZE(spi_board_info));
	ret = spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	if (ret) {
		printk(KERN_WARNING "%s: error registering SPI devices: %d\n",
			   __FUNCTION__, ret);
	}
	return ret;
}




device_initcall(stk5_spi_register);
#endif // defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)

static struct pad_desc stk5_pwm_pads[] = {
	MX25_PAD_PWM__PWM,
	MX25_PAD_GPIO_C__PWM4,
	MX25_PAD_GPIO_A__PWM2,
};


static struct elliptec_pwm_platform_data elliptec_info0 = {
	.name="herbert",
};

static struct elliptec_pwm_platform_data elliptec_info1 = {
	.name="lisa",
};

static struct elliptec_pwm_platform_data elliptec_info2 = {
	.name="alfons",
};

static struct elliptec_pwm_platform_data elliptec_info3 = {
	.name="paul",
};



static int __init stk5_pwm_register(void)
{
	int ret;
	printk(KERN_INFO "%s: -----------------------------------------\n", __FUNCTION__);


	printk(KERN_INFO "Configuring PWM pads\n");
	ret = mxc_iomux_v3_setup_multiple_pads(stk5_pwm_pads,
					ARRAY_SIZE(stk5_pwm_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure PWM pads %d\n",ret);
		return ret;
	}
	ret = mxc_register_device(&mxc_pwm_device0, &elliptec_info0);
	printk(KERN_INFO "%s: mxc_register_device() returned: %d\n", __FUNCTION__, ret);
	if (ret) {
			printk(KERN_ERR "Failed to register devices %d\n",ret);
			return ret;
		}
	ret = mxc_register_device(&mxc_pwm_device3, &elliptec_info3);
	printk(KERN_INFO "%s: mxc_register_device() returned: %d\n", __FUNCTION__, ret);
	if (ret) {
			printk(KERN_ERR "Failed to register devices %d\n",ret);
			return ret;
		}

	return ret;

}
device_initcall(stk5_pwm_register);

#if defined(CONFIG_AC97_BUS) || defined(CONFIG_AC97_BUS_MODULE)
static u64 stk5_dma_mask = ~0UL;

static struct pad_desc stk5_ac97_pads_on[] = {
	MX25_PAD_VSTBY_ACK__GPIO_3_18, /* UCB1400 Reset */
	MX25_PAD_RW__AUD4_TXFS,
	MX25_PAD_EB0__AUD4_TXD,
	MX25_PAD_EB1__AUD4_RXD,
	MX25_PAD_OE__AUD4_TXC,
};

static struct pad_desc stk5_ac97_pads_off[] = {
	MX25_PAD_VSTBY_ACK__GPIO_3_18, /* UCB1400 Reset */
	MX25_PAD_RW__GPIO_3_25,
	MX25_PAD_EB0__GPIO_2_12,
	MX25_PAD_EB1__AUD4_RXD,
	MX25_PAD_OE__AUD4_TXC,
};

static struct gpio_desc {
	unsigned int gpio:7;
	unsigned int dir:1;
	unsigned int level:1;
} stk5_ac97_gpios[] = {
	/* configure the PHY strap pins to the correct values */
	{ GPIO_PORTC | 18, 1, 0, },
	{ GPIO_PORTC | 25, 1, 0, },
	{ GPIO_PORTB | 12, 1, 0, },
};

static int stk5_ac97_init(struct platform_device *dev)
{
	int ret;
	const int irq = gpio_to_irq(GPIO_PORTC | 15);
	int i;

	DBG(0, "%s: \n", __FUNCTION__);

	/*
	 * IRQs are disabled for probing whenever enable_irq() is called.
	 * Thus reenable the UCB1400 IRQ for probing in case the ucb1400-ts
	 * module has been reloaded.
	 */
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	ret = mxc_iomux_v3_setup_multiple_pads(stk5_ac97_pads_off,
					ARRAY_SIZE(stk5_ac97_pads_off));
	if (ret == 0) {
		for (i = 0; i < ARRAY_SIZE(stk5_ac97_gpios); i++) {
			struct gpio_desc *pd = &stk5_ac97_gpios[i];

			ret = gpio_request(pd->gpio, "AC97");
			if (ret < 0) {
				DBG(0, "%s: Failed to request GPIO%d_%d: %d\n",
					__FUNCTION__, pd->gpio / 32 + 1, pd->gpio % 32, ret);
				goto rel_mux;
			}
			if (pd->dir) {
				gpio_direction_output(pd->gpio,
						pd->level);
			} else {
				gpio_direction_input(pd->gpio);
			}
		}

		ret = mxc_iomux_v3_setup_multiple_pads(stk5_ac97_pads_on,
						ARRAY_SIZE(stk5_ac97_pads_on));
		if (ret != 0) {
			goto rel_gpio;
		}
		udelay(1);
		gpio_set_value(stk5_ac97_gpios[0].gpio, !stk5_ac97_gpios[0].level);
	}
	return ret;

rel_mux:
	mxc_iomux_v3_release_multiple_pads(stk5_ac97_pads_off,
					ARRAY_SIZE(stk5_ac97_pads_off));
rel_gpio:
	while (--i >= 0) {
		struct gpio_desc *pd = &stk5_ac97_gpios[i];
		int grp = pd->gpio / 32 + 1;
		int ofs = pd->gpio % 32;

		DBG(0, "%s: Freeing GPIO%d_%d\n", __FUNCTION__,
			grp, ofs);
		gpio_free(pd->gpio);
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
		struct gpio_desc *pd = &stk5_ac97_gpios[i];
		int grp = pd->gpio / 32 + 1;
		int ofs = pd->gpio % 32;

		DBG(0, "%s: Freeing GPIO%d_%d\n", __FUNCTION__,
			grp, ofs);
		gpio_free(pd->gpio);
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

static struct platform_device ac97_device = {
	.name		= "mx25-ac97",
	.id		= -1,
	.dev = {
		.dma_mask = &stk5_dma_mask,
		.coherent_dma_mask = ~0UL,
		.platform_data = &stk5_ac97_ops,
	},
};
#endif


static struct i2c_board_info pcm037_i2c_0_devices[] = {
	{
		I2C_BOARD_INFO("mt9m001", 0x5d),
	},
};

static unsigned long stk5_camera_query_bus_param(struct soc_camera_link *icl)
{
	return SOCAM_DATAWIDTH_8 | SOCAM_DATAWIDTH_10;
			/*SOCAM_PCLK_SAMPLE_FALLING | SOCAM_PCLK_SAMPLE_RISING |
			SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
			SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
			SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATA_ACTIVE_LOW |
			SOCAM_MASTER;*/
}

static int stk5_set_bus_param(struct soc_camera_link *icl,
				 unsigned long flags)
{
	return  0;
}
static struct soc_camera_link iclink = {
	.bus_id		= 0,		/* Must match with the camera ID */
	.power		= NULL,
	.board_info	= &pcm037_i2c_0_devices[0],
	.query_bus_param = stk5_camera_query_bus_param,
	.set_bus_param = stk5_set_bus_param,
	.i2c_adapter_id	= 0,
	.module_name	= "mt9m001",
};


static struct platform_device pcm037_camera = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.platform_data = &iclink,
	},
};

struct mx25_camera_pdata camera_pdata = {
	.flags		= MX25_CAMERA_DATA_HIGH | MX25_CAMERA_PCLK_RISING,
	MX25_CAMERA_DATAWIDTH_8 | MX25_CAMERA_DATAWIDTH_10,
	.mclk_10khz	= 1000,
};

static struct imxi2c_platform_data pcm037_i2c_0_data = {
	.bitrate = 20000,
};

static struct pad_desc stk5_camera_pads[] = {
		MX25_PAD_LD0__CSI_D0,
		MX25_PAD_LD1__CSI_D1,
		MX25_PAD_CSI_D2__CSI_D2,
		MX25_PAD_CSI_D3__CSI_D3,
		MX25_PAD_CSI_D4__CSI_D4,
		MX25_PAD_CSI_D5__CSI_D5,
		MX25_PAD_CSI_D6__CSI_D6,
		MX25_PAD_CSI_D7__CSI_D7,
		MX25_PAD_CSI_D8__CSI_D8,
		MX25_PAD_CSI_D9__CSI_D9,
		MX25_PAD_CSI_MCLK__CSI_MCLK,
		MX25_PAD_CSI_VSYNC__CSI_VSYNC,
		MX25_PAD_CSI_HSYNC__CSI_HSYNC,
		MX25_PAD_CSI_PIXCLK__CSI_PIXCLK,
};

static int __init stk5_camera_register(void)
{
	int ret;
	printk(KERN_INFO "%s: -----------------------------------------\n", __FUNCTION__);


	printk(KERN_INFO "Configuring CSI pads\n");
	ret = mxc_iomux_v3_setup_multiple_pads(stk5_camera_pads,
					ARRAY_SIZE(stk5_camera_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure CSI pads %d\n",ret);
		return ret;
	}
	ret = mxc_register_device(&mx25_csi_device, &camera_pdata);
	printk(KERN_INFO "%s: mxc_register_device() returned: %d\n", __FUNCTION__, ret);
	if (ret) {
			printk(KERN_ERR "Failed to register devices %d\n",ret);
			return ret;
		}
	printk(KERN_INFO "registerint i2c\n");

	return ret;

}
device_initcall(stk5_camera_register);

static struct platform_dev_list {
	struct platform_device *pdev;
	int flag;
} stk5_devices[] __initdata = {
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	{ .pdev = &stk5_led_device, .flag = -1, },
#endif
#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
	{ .pdev = &stk5_keypad_device, .flag = 1, },
#endif
#if defined(CONFIG_AC97_BUS) || defined(CONFIG_AC97_BUS_MODULE)
	{ .pdev = &ac97_device, .flag = 1, },
#endif
#if defined(CONFIG_MMC_SDHCI_MXC) || defined(CONFIG_MMC_SDHCI_MXC_MODULE)
	{ .pdev = &stk5_sdhc1_device, .flag = 1, },
#endif
//	{ .pdev = &stk5_ledpwm_device, .flag = 1, },
	{ .pdev = &pcm037_camera, .flag = 1,},
};
#define STK5_NUM_DEVICES		ARRAY_SIZE(stk5_devices)

static __init int karo_stk5_board_init(void)
{
	int ret;
	int i;

	if (karo_get_board_type() != BOARD_KARO_STK5) {
		return -ENODEV;
	}
	DBG(0, "%s: \n", __FUNCTION__);

	karo_stk5_serial_init();

	/* enable SSI1_INT (GPIO_3_15) for IRQ probing */
	set_irq_flags(gpio_to_irq(GPIO_PORTC | 15), IRQF_VALID | IRQF_PROBE);

	ret = karo_stk5_fb_register();
	if (ret) {
		printk(KERN_WARNING "%s: karo_stk5_fb_register() failed: %d\n",
			__FUNCTION__, ret);
	}
	//mxc_register_device(&mxc_i2c_device0, &pcm037_i2c_0_data);

	for (i = 0; i < STK5_NUM_DEVICES; i++) {
		if (stk5_devices[i].pdev == NULL) continue;
		if (!stk5_devices[i].flag) {
			DBG(0, "%s: Skipping platform device[%d] @ %p dev %p: %s\n",
				__FUNCTION__, i, stk5_devices[i].pdev, &stk5_devices[i].pdev->dev,
				stk5_devices[i].pdev->name);
			continue;
		}
		printk(KERN_INFO "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, stk5_devices[i].pdev, &stk5_devices[i].pdev->dev,
			stk5_devices[i].pdev->name);
		ret = platform_device_register(stk5_devices[i].pdev);
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, stk5_devices[i].pdev->name, ret);
		}
	}

	DBG(0, "%s: Done\n", __FUNCTION__);
	return 0;
}
subsys_initcall(karo_stk5_board_init);
