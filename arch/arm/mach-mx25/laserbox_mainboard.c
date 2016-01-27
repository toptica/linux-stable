/*
 * arch/arm/mach-mx2/laserbox_mainboard.c
 *
 * Copyright (C) 2010 Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 * based on arch/arm/mach-mx25/stk5-baseboard.c
 *  by Lothar Wassmann <LW@KARO-electronics.de>
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

/*
 * UART configuration for laserbox
 *
 * - only RxD and TxD lines are used
 * - UART 3 is used on KPP_ROW pads,
 * 		not available in original iomux-mx25.h
 */

#define MX25_PAD_KPP_ROW0__UART3_RXD	IOMUX_PAD(0x3a0, 0x1a8, 0x11, 0x0568, 1, NO_PAD_CTRL)
#define MX25_PAD_KPP_ROW1__UART3_TXD	IOMUX_PAD(0x3a4, 0x1ac, 0x11, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_CSPI1_MOSI__UART3_RXD	IOMUX_PAD(0x350, 0x158, 0x12, 0x0568, 0, NO_PAD_CTRL)
#define MX25_PAD_CSPI1_MISO__UART3_TXD	IOMUX_PAD(0x354, 0x15c, 0x12, 0, 0, NO_PAD_CTRL)

static struct pad_desc laserbox_uart_pads[][2] = {
	{
		MX25_PAD_UART1_TXD__UART1_TXD,
		MX25_PAD_UART1_RXD__UART1_RXD,
	}, {
		MX25_PAD_UART2_TXD__UART2_TXD,
		MX25_PAD_UART2_RXD__UART2_RXD,
	}, {
		MX25_PAD_CSPI1_MOSI__UART3_RXD,
		MX25_PAD_CSPI1_MISO__UART3_TXD,
	},
};

static int laserbox_uart_init(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	if (pdev->id >= ARRAY_SIZE(laserbox_uart_pads)) {
		return -ENODEV;
	}
	return mxc_iomux_v3_setup_multiple_pads(laserbox_uart_pads[pdev->id],
						ARRAY_SIZE(laserbox_uart_pads[pdev->id]));
}

static void laserbox_uart_exit(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	BUG_ON(pdev->id >= ARRAY_SIZE(laserbox_uart_pads));
	mxc_iomux_v3_release_multiple_pads(laserbox_uart_pads[pdev->id],
					ARRAY_SIZE(laserbox_uart_pads[pdev->id]));
}

static struct imxuart_platform_data laserbox_uart_ports[] = {
	{
		.init = laserbox_uart_init,
		.exit = laserbox_uart_exit,
		.flags = 0,
	}, {
		.init = laserbox_uart_init,
		.exit = laserbox_uart_exit,
		.flags = 0,
	}, {
		.init = laserbox_uart_init,
		.exit = laserbox_uart_exit,
		.flags = 0,
	}, {
		.init = laserbox_uart_init,
		.exit = laserbox_uart_exit,
		.flags = 0,
	}, {
		.init = laserbox_uart_init,
		.exit = laserbox_uart_exit,
		.flags = 0,
	}, {
		.init = laserbox_uart_init,
		.exit = laserbox_uart_exit,
		.flags = 0,
	},
};


static struct platform_device *laserbox_uart_devices[] = {
	&mxc_uart_device0,
	&mxc_uart_device1,
	&mxc_uart_device2,
};

static void __init karo_laserbox_serial_init(void)
{
	int i;
	printk(KERN_INFO "%s: Registering serial ports\n",__FUNCTION__);
	for (i = 0; i < ARRAY_SIZE(laserbox_uart_devices); i++) {
		int ret;
		int port = laserbox_uart_devices[i]->id;

		printk(KERN_INFO "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, laserbox_uart_devices[i],
			&laserbox_uart_devices[i]->dev, laserbox_uart_devices[i]->name);
		ret = mxc_register_device(laserbox_uart_devices[i],
					&laserbox_uart_ports[port]);
		if (ret != 0) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, laserbox_uart_devices[i]->name, ret);
		}
	}
}


/*
 *  USB has to be configured sooner or laster ...
 */
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


/*
 * ADC configuration for laserbox
 *
 * + all 8 touchscreen pins are used as single ended ADC inputs
 * + an GPIO is used for additional multiplexing to get 16 ADC inputs
 * + all use external reference voltage
 *
 */


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

	ret = mxc_register_device(&mx25_tsc_device, (void *) ADC_EXTERNAL_REFERENCE);
	if (ret) {
		printk(KERN_WARNING "Failed to register TSC device: %d\n", ret);
	}
	return ret;
}
device_initcall(karo_tx25_adc_register);


#if 0
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led laserbox_leds[] = {
	{
		.name = "GPIO-LED",
		.default_trigger = "heartbeat",
		.gpio = GPIO_PORTB | 7,
	},
};

static struct gpio_led_platform_data laserbox_led_data = {
	.leds = laserbox_leds,
	.num_leds = ARRAY_SIZE(laserbox_leds),
};

static struct platform_device laserbox_led_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &laserbox_led_data,
	},
};
#endif
#endif


/*
 * SPI configuration for laserbox
 *
 * + CSPI2 is used!
 * + the 4 native chip selects SS0 .. SS3 are used for the
 * 		4 laser driver ICs
 *
 *	spi1.0 -> MX25_PAD_SD1_DATA2__CSPI2_SS0
 *	spi1.1 -> MX25_PAD_SD1_DATA3__CSPI2_SS1
 *	spi1.2 -> MX25_PAD_GPIO_C__CSPI2_SS2
 *	spi1.3 -> MX25_PAD_UART2_RTS__CSPI2_SS3
 */


/* negative numbers are for "native" slave select lines of CSPI
 * ports, -4 means SS0, -3 means SS1 etc.
 */
static int mxcspi2_available_cs[4] = {-1, -2, -3, -4};

static struct spi_imx_master mxcspi2_data = {
	.num_chipselect = 4,
	.chipselect = mxcspi2_available_cs,
};

#define MX25_PAD_LD12__CSPI2_MOSI		IOMUX_PAD(0x2f0, 0x0f8, 0x12, 0x04a0, 0, NO_PAD_CTRL)
#define MX25_PAD_LD13__CSPI2_MISO		IOMUX_PAD(0x2f4, 0x0fc, 0x12, 0x049c, 0, PAD_CTL_PUS_100K_DOWN)
#define MX25_PAD_SD1_DATA0__CSPI2_SCLK	IOMUX_PAD(0x390, 0x198, 0x11, 0x0494, 1, NO_PAD_CTRL)
#define MX25_PAD_SD1_DATA2__CSPI2_SS0	IOMUX_PAD(0x398, 0x1a0, 0x11, 0x04a4, 1, NO_PAD_CTRL)
#define MX25_PAD_SD1_DATA3__CSPI2_SS1	IOMUX_PAD(0x39c, 0x1a4, 0x11, 0x04a8, 1, NO_PAD_CTRL)
#define MX25_PAD_GPIO_C__CSPI2_SS2		IOMUX_PAD(0x3f8, 0x1fc, 0x17, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_UART2_RTS__CSPI2_SS3	IOMUX_PAD(0x380, 0x188, 0x16, 0, 0, NO_PAD_CTRL)

static struct pad_desc laserbox_cspi2_pads[] = {
	MX25_PAD_LD12__CSPI2_MOSI,
	MX25_PAD_LD13__CSPI2_MISO,
	MX25_PAD_SD1_DATA0__CSPI2_SCLK,
	MX25_PAD_SD1_DATA2__CSPI2_SS0,
	MX25_PAD_SD1_DATA3__CSPI2_SS1,
	MX25_PAD_GPIO_C__CSPI2_SS2,
	MX25_PAD_UART2_RTS__CSPI2_SS3,
};

static struct spi_board_info spi_board_info[] __initdata = {
{
	.modalias	= "laserdriver",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 10000000,
	.bus_num	= 1,
	.chip_select	= 0,
},
{
	.modalias	= "laserdriver",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 10000000,
	.bus_num	= 1,
	.chip_select	= 1,
},
{
	.modalias	= "laserdriver",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 10000000,
	.bus_num	= 1,
	.chip_select	= 2,
},
{
	.modalias	= "laserdriver",
	.platform_data	= NULL,
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 10000000,
	.bus_num	= 1,
	.chip_select	= 3,
},
};

static int __init laserbox_spi_register(void)
{
	int ret;
	printk(KERN_INFO "%s: -----------------------------------------\n", __FUNCTION__);


	printk(KERN_INFO "Configuring CSPI2 pads\n");
	ret = mxc_iomux_v3_setup_multiple_pads(laserbox_cspi2_pads,
					ARRAY_SIZE(laserbox_cspi2_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure CSPI2 pads %d\n",ret);
		return ret;
	}
	ret = mxc_register_device(&mxc_spi_device1, &mxcspi2_data);
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

device_initcall(laserbox_spi_register);


/*
 * Elliptec motor support for laserbox
 *
 * + 2 PWM	outputs of PWM0 and PWM4 are used for controlling the motors
 * + 5x multiplexing with GPIOs
 * + opt. 2 end switches
 */

#define MX25_PAD_CONTRAST__PWM4_PWMO	IOMUX_PAD(0x310, 0x118, 0x14, 0, 0, NO_PAD_CTRL)

static struct pad_desc laserbox_pwm_pads[] = {
	/* PWMs */
	MX25_PAD_PWM__PWM,
	MX25_PAD_CONTRAST__PWM4_PWMO,
	/* multiplex GPIOs */
	MX25_PAD_LSCLK__GPIO_1_24,
	MX25_PAD_A19__GPIO_2_5,
	MX25_PAD_D14__GPIO_4_6,
	MX25_PAD_D15__GPIO_4_5,
	MX25_PAD_A18__GPIO_2_4,
	MX25_PAD_A20__GPIO_2_6,
	/* opt. end switches */
	MX25_PAD_A21__GPIO_2_7,
	MX25_PAD_A22__GPIO_2_8,
};

static struct elliptec_pwm_platform_data elliptec_info0 = {
	.name="herbert",
};

static struct elliptec_pwm_platform_data elliptec_info3 = {
	.name="paul",
};



static int __init laserbox_pwm_register(void)
{
	int ret;
	printk(KERN_INFO "Configuring PWM pads\n");
	ret = mxc_iomux_v3_setup_multiple_pads(laserbox_pwm_pads,
					ARRAY_SIZE(laserbox_pwm_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure PWM pads %d\n",ret);
		return ret;
	}
	ret = mxc_register_device(&mxc_pwm_device0, &elliptec_info0);
	if (ret) {
			printk(KERN_ERR "Failed to register devices %d\n",ret);
			return ret;
		}
	ret = mxc_register_device(&mxc_pwm_device3, &elliptec_info3);
	if (ret) {
			printk(KERN_ERR "Failed to register devices %d\n",ret);
			return ret;
		}

	return ret;

}
device_initcall(laserbox_pwm_register);

/*
 * camera support for laserbox
 *
 * + up to 12 data bits
 * + soc_camera_link evtl. necessary to allow desired datawidth
 * + in real hardware GPIO for camera reset line
 */

static struct i2c_board_info laserbox_i2c_0_devices[] = {
	{
		I2C_BOARD_INFO("mt9m001", 0x5d),
	},
};

static unsigned long laserbox_camera_query_bus_param(struct soc_camera_link *icl)
{
	return SOCAM_DATAWIDTH_8 | SOCAM_DATAWIDTH_10;
			/*SOCAM_PCLK_SAMPLE_FALLING | SOCAM_PCLK_SAMPLE_RISING |
			SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
			SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
			SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATA_ACTIVE_LOW |
			SOCAM_MASTER;*/
}

static int laserbox_set_bus_param(struct soc_camera_link *icl,
				 unsigned long flags)
{
	return  0;
}
#define CSI_RESET_GPIO (GPIO_PORTA | 25)

int laserbox_camera_power(struct device *dev, int onoff)
{
	gpio_set_value(CSI_RESET_GPIO, onoff?1:0);
	return 0;
}


static struct soc_camera_link iclink = {
	.bus_id		= 0,		/* Must match with the camera ID */
	.power		= laserbox_camera_power,
	.board_info	= &laserbox_i2c_0_devices[0],
	.query_bus_param = laserbox_camera_query_bus_param,
	.set_bus_param = laserbox_set_bus_param,
	.i2c_adapter_id	= 0,
	.module_name	= "mt9m001",
};


static struct platform_device laserbox_camera = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.platform_data = &iclink,
	},
};

struct mx25_camera_pdata camera_pdata = {
	.flags		= MX25_CAMERA_DATA_HIGH | MX25_CAMERA_PCLK_RISING |
	MX25_CAMERA_DATAWIDTH_8 | MX25_CAMERA_DATAWIDTH_10,
	.mclk_10khz	= 1000,
};

static struct imxi2c_platform_data laserbox_i2c_0_data = {
	.bitrate = 20000,
};

#define MX25_PAD_LD7__CSI_D10		IOMUX_PAD(0x2dc, 0x0e4, 0x12, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_LD6__CSI_D11		IOMUX_PAD(0x2d8, 0x0e0, 0x12, 0, 0, NO_PAD_CTRL)

static struct pad_desc laserbox_camera_pads[] = {
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
		MX25_PAD_LD7__CSI_D10,
		MX25_PAD_LD6__CSI_D11,
		MX25_PAD_CSI_MCLK__CSI_MCLK,
		MX25_PAD_CSI_VSYNC__CSI_VSYNC,
		MX25_PAD_CSI_HSYNC__CSI_HSYNC,
		MX25_PAD_CSI_PIXCLK__CSI_PIXCLK,
		MX25_PAD_OE_ACD__GPIO_1_25, /* reset line */
};

static int __init laserbox_camera_register(void)
{
	/*
	 * FIXME: reset line has to be handled somehow for the real hardware
	 *
	 */
	int ret;
	printk(KERN_INFO "Configuring CSI pads\n");
	ret = mxc_iomux_v3_setup_multiple_pads(laserbox_camera_pads,
					ARRAY_SIZE(laserbox_camera_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure CSI pads %d\n",ret);
		return ret;
	}

	printk(KERN_INFO "Preparing CSI reset pin\n");
	ret = gpio_request(CSI_RESET_GPIO, "mx25_camera");
	if (ret) {
		printk(KERN_INFO "%s: Failed to request GPIO for camera %d\n",
			__FUNCTION__, ret);
	} else {
		ret = gpio_direction_output(CSI_RESET_GPIO, 0);
		if (ret) {
			printk(KERN_INFO "%s: Failed to set GPIOdirection for camera %d\n",
				__FUNCTION__, ret);
		}
		gpio_export(CSI_RESET_GPIO, 0);
	}


	ret = mxc_register_device(&mx25_csi_device, &camera_pdata);
	if (ret) {
			printk(KERN_ERR "Failed to register devices %d\n",ret);
			return ret;

	}
	printk(KERN_INFO "Registering camera platformdevice\n");
	ret = platform_device_register(&laserbox_camera);
	if (ret) {
		printk(KERN_WARNING "%s: Failed to register platform_device: %s: %d\n",
			__FUNCTION__, laserbox_camera.name, ret);
	}
	return ret;

}
device_initcall(laserbox_camera_register);

/*
 * buzzer
 */
#define MX25_PAD_GPIO_D__GPT2_CMPOUT1		IOMUX_PAD(0x3fc, 0x200, 0x14, 0, 0, NO_PAD_CTRL)

static struct pad_desc laserbox_buzzer_pads[] = {
		MX25_PAD_GPIO_D__GPT2_CMPOUT1,
};


static int __init laserbox_buzzer_register(void)
{
	int ret;

	printk(KERN_INFO "Configuring GPT pads\n");
	ret = mxc_iomux_v3_setup_multiple_pads(laserbox_buzzer_pads,
					ARRAY_SIZE(laserbox_buzzer_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure PWM pads %d\n",ret);
		return ret;
	}
	ret = mxc_register_device(&mxc_gpt_device1, NULL);
	if (ret) {
			printk(KERN_ERR "Failed to register devices %d\n",ret);
			return ret;
		}

	return ret;

}
device_initcall(laserbox_buzzer_register);

/*
 * FPAG configuration for laserbox
 *
 * + 5 lines for configuration interface
 * + 8 data lines (default pads)
 * + 8 address lines (default pads)
 * + misc (default pads)
 */

static struct pad_desc laserbox_fpga_pads[] = {
	/* fpga configuration */
	MX25_PAD_KPP_COL3__GPIO_3_4,
	MX25_PAD_UART2_CTS__GPIO_4_29,
	MX25_PAD_SD1_DATA1__GPIO_2_26,
	MX25_PAD_CS5__CS5,
	MX25_PAD_LD5__GPIO_1_19,
	MX25_PAD_RW__RW,
	MX25_PAD_D0__D0,
	MX25_PAD_D1__D1,
	MX25_PAD_D2__D2,
	MX25_PAD_D3__D3,
	MX25_PAD_D4__D4,
	MX25_PAD_D5__D5,
	MX25_PAD_D6__D6,
	MX25_PAD_D7__D7,
};

/*
 * TEC IOs for laserbox
 *
 */

#define MX25_PAD_A17__GPIO_2_3_PULLUP		IOMUX_PAD(0x238, 0x01c, 0x15, 0, 0, PAD_CTL_PKE)
#define MX25_PAD_A24__GPIO_2_10_PULLUP		IOMUX_PAD(0x250, 0x038, 0x15, 0, 0, PAD_CTL_PKE)
#define MX25_PAD_A23__GPIO_2_9_PULLUP		IOMUX_PAD(0x24c, 0x034, 0x15, 0, 0, PAD_CTL_PKE)
#define MX25_PAD_LD4__GPIO_2_19_PULLUP		IOMUX_PAD(0x2d0, 0x0d8, 0x15, 0, 0, PAD_CTL_PKE | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE)

static struct pad_desc laserbox_tec_pads[] = {
		MX25_PAD_LD3__GPIO_2_18,
		MX25_PAD_A17__GPIO_2_3,
		MX25_PAD_A24__GPIO_2_10,
		MX25_PAD_LD2__GPIO_2_17,
		MX25_PAD_LD4__GPIO_2_19,
		MX25_PAD_A23__GPIO_2_9,
};

/*
 * external laser control for laserbox
 *
 */
#define MX25_PAD_CSPI1_SS1__GPIO_1_17_PULLUP	IOMUX_PAD(0x35c, 0x164, 0x15, 0, 0, PAD_CTL_PKE | PAD_CTL_PUS_100K_UP)

static struct pad_desc laserbox_extlaser_pads[] = {
		MX25_PAD_KPP_COL0__GPIO_3_1,
		MX25_PAD_KPP_COL1__GPIO_3_2,
		MX25_PAD_KPP_COL2__GPIO_3_3,
		MX25_PAD_CSPI1_SCLK__GPIO_1_18,
		MX25_PAD_CSPI1_SS0__GPIO_1_16,
		MX25_PAD_CSPI1_SS1__GPIO_1_17_PULLUP,
};

/*
 * reserved PADs, not directly used by laserbox
 */

#define MX25_PAD_VSYNC__I2C3_SDA		IOMUX_PAD(0x304, 0x10c, 0x12, 0x0528, 0, NO_PAD_CTRL)
#define MX25_PAD_HSYNC__I2C3_SCL		IOMUX_PAD(0x300, 0x108, 0x12, 0x0524, 0, NO_PAD_CTRL)

#define MX25_PAD_VSTBY_ACK__EPIT1_EPITO	IOMUX_PAD(0x40c, 0x218, 0x16, 0, 0, NO_PAD_CTRL)

#define MX25_PAD_UART1_RTS__GPT3_CAPIN1	IOMUX_PAD(0x370, 0x178, 0x12, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_UART1_CTS__GPT3_CMPOUT1	IOMUX_PAD(0x374, 0x17c, 0x12, 0, 0, NO_PAD_CTRL)

#define MX25_PAD_LD8__AUD3_TXD		IOMUX_PAD(0x2e0, 0x0e8, 0x14, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_LD9__AUD3_RXD		IOMUX_PAD(0x2e4, 0x0ec, 0x14, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_LD10__AUD3_TXC		IOMUX_PAD(0x2e8, 0x0f0, 0x14, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_LD11__AUD3_TXFS		IOMUX_PAD(0x2ec, 0x0f4, 0x14, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_LD14__AUD3_RXC		IOMUX_PAD(0x2f8, 0x100, 0x16, 0, 0, NO_PAD_CTRL)
#define MX25_PAD_LD15__AUD3_RXFS		IOMUX_PAD(0x2fc, 0x104, 0x16, 0, 0, NO_PAD_CTRL)

static struct pad_desc laserbox_reserved_pads[] = {
		/* i2c reserved */
		MX25_PAD_VSYNC__I2C3_SDA,
		MX25_PAD_HSYNC__I2C3_SCL,
		/* 1Wire reserved */
		MX25_PAD_RTCK__OWIRE,
		/* CAN reserved */
		MX25_PAD_GPIO_A__CAN1_TX,
		MX25_PAD_GPIO_B__CAN1_RX,
		/* timer0 reserved */
		MX25_PAD_VSTBY_ACK__EPIT1_EPITO,
		/* timer2 reserved */
		MX25_PAD_UART1_RTS__GPT3_CAPIN1,
		MX25_PAD_UART1_CTS__GPT3_CMPOUT1,
		/* SSI1 reserved */
		MX25_PAD_LD8__AUD3_TXD,
		MX25_PAD_LD9__AUD3_RXD,
		MX25_PAD_LD10__AUD3_TXC,
		MX25_PAD_LD11__AUD3_TXFS,
		MX25_PAD_LD14__AUD3_RXC,
		MX25_PAD_LD15__AUD3_RXFS,
		/* just reserved */
		MX25_PAD_CSPI1_RDY__GPIO_2_22,
};


/*
 * miscellaneous signals for laserbox
 * + inputs and outputs
 *
 */
static struct pad_desc laserbox_misc_pads[] = {
		/* clock */
		MX25_PAD_CLKO__CLKO,
		/* error */
		MX25_PAD_KPP_ROW0__GPIO_2_29,
		/*wait */
		MX25_PAD_KPP_ROW1__GPIO_2_30,
		/* shutter */
		MX25_PAD_KPP_ROW2__GPIO_2_31,
		/* interlock */
		MX25_PAD_KPP_ROW3__GPIO_3_0,
		/* power ok */
		MX25_PAD_POWER_FAIL__POWER_FAIL,
		/* test */
		MX25_PAD_SD1_CLK__GPIO_2_24,
		MX25_PAD_SD1_CMD__GPIO_2_23,
};

#define PAD_CONFIG(pads,label)	\
		printk(KERN_INFO "Configuring " label " pads\n");\
		ret = mxc_iomux_v3_setup_multiple_pads(pads,\
						ARRAY_SIZE(pads)); \
		if (ret) { \
			printk(KERN_ERR "Failed to configure " label "pads %d\n",ret); \
		}

static struct platform_dev_list {
	struct platform_device *pdev;
	int flag;
} laserbox_devices[] __initdata = {
//#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
//	{ .pdev = &laserbox_led_device, .flag = -1, },
//#endif
//	{ .pdev = &stk5_ledpwm_device, .flag = 1, },
//	{ .pdev = &mxc_spi_device1, .flag = 1, },
//	{ .pdev = &laserbox_camera, .flag = 1,},
};
#define STK5_NUM_DEVICES		ARRAY_SIZE(laserbox_devices)

static __init int karo_laserbox_board_init(void)
{
	int ret;
	int i;

	printk(KERN_INFO "%s: \n", __FUNCTION__);

	karo_laserbox_serial_init();

	PAD_CONFIG(laserbox_fpga_pads,		"FPGA")
	PAD_CONFIG(laserbox_tec_pads,		"TEC")
	PAD_CONFIG(laserbox_extlaser_pads,	"external laser (pullup)")
	PAD_CONFIG(laserbox_reserved_pads,	"reserved")
	PAD_CONFIG(laserbox_misc_pads,		"miscellaneous")

	for (i = 0; i < STK5_NUM_DEVICES; i++) {
		if (laserbox_devices[i].pdev == NULL) continue;
		if (!laserbox_devices[i].flag) {
			DBG(0, "%s: Skipping platform device[%d] @ %p dev %p: %s\n",
				__FUNCTION__, i, laserbox_devices[i].pdev, &laserbox_devices[i].pdev->dev,
				laserbox_devices[i].pdev->name);
			continue;
		}
		printk(KERN_INFO "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, laserbox_devices[i].pdev, &laserbox_devices[i].pdev->dev,
			laserbox_devices[i].pdev->name);
		ret = platform_device_register(laserbox_devices[i].pdev);
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, laserbox_devices[i].pdev->name, ret);
		}
	}

	DBG(0, "%s: Done\n", __FUNCTION__);
	return 0;
}
subsys_initcall(karo_laserbox_board_init);
