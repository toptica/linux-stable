/*
 * arch/arm/mach-mx27/karo-tx27.c
 *
 * Copyright (C) 2008  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * based on: arch/arm/mach-mx27ads.c (C) Freescale Semiconductor, Inc.
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
#include <linux/serial_8250.h>
#include <linux/fec_enet.h>
#include <mtd/mtd-abi.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>

#include <linux/serial.h>
#include <linux/fsl_devices.h>
#include <linux/rtc/ds13xx.h>
#include <linux/irq.h>
#include <linux/mmc/host.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>

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
#include <mach/imx_spi.h>
#include <mach/imx_i2c.h>
#include <mach/mmc.h>
#include <mach/imx-uart.h>
#include <mach/mxc_nand.h>
#include <mach/ulpi.h>
#include <mach/mxc_ehci.h>
#include <mach/mxc-ac97.h>
#include <mach/board-tx27.h>

#include "crm_regs.h"
#include "devices.h"
#include "karo.h"

#ifdef DEBUG
int tx27_debug = 1;
#endif

#if defined(CONFIG_SERIAL_IMX) || defined(CONFIG_SERIAL_IMX_MODULE)
static int tx27_uart_pins[][4] = {
	{
		PE12_PF_UART1_TXD,
		PE13_PF_UART1_RXD,
		PE14_PF_UART1_CTS,
		PE15_PF_UART1_RTS,
	},{
		PE6_PF_UART2_TXD,
		PE7_PF_UART2_RXD,
		PE3_PF_UART2_CTS,
		PE4_PF_UART2_RTS,
	},{
		PE8_PF_UART3_TXD,
		PE9_PF_UART3_RXD,
		PE10_PF_UART3_CTS,
		PE11_PF_UART3_RTS,
	},{
		PB28_AF_UART4_TXD,
		PB31_AF_UART4_RXD,
		PB29_AF_UART4_CTS,
		PB26_AF_UART4_RTS,
	},{
		PB18_AF_UART5_TXD,
		PB19_AF_UART5_RXD,
		PB20_AF_UART5_CTS,
		PB21_AF_UART5_RTS,
	},{
		PB10_AF_UART6_TXD,
		PB11_AF_UART6_RXD,
		PB12_AF_UART6_CTS,
		PB13_AF_UART6_RTS,
	},
};

static int tx27_uart_init(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	return mxc_gpio_setup_multiple_pins(tx27_uart_pins[pdev->id],
					    ARRAY_SIZE(tx27_uart_pins[pdev->id]), "UART");
}

static int tx27_uart_exit(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	mxc_gpio_release_multiple_pins(tx27_uart_pins[pdev->id],
			ARRAY_SIZE(tx27_uart_pins[pdev->id]));
	return 0;
}

static struct imxuart_platform_data tx27_uart_ports[] = {
	{
		.init = tx27_uart_init,
		.exit = tx27_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},{
		.init = tx27_uart_init,
		.exit = tx27_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},{
		.init = tx27_uart_init,
		.exit = tx27_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},{
		.init = tx27_uart_init,
		.exit = tx27_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},{
		.init = tx27_uart_init,
		.exit = tx27_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},{
		.init = tx27_uart_init,
		.exit = tx27_uart_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},
};

static struct platform_device *tx27_uart_devices[] = {
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
#if UART6_ENABLED
	&mxc_uart_device5,
#endif
};
#endif

#ifdef CONFIG_USB_EHCI_MXC

#define SMSC_VENDOR_ID		0x0424
#define USB3317_PROD_ID		0x0006
#define ULPI_FCTL_RESET		(1 << 5)

/* USB register offsets */
#define REG_USBCTRL		0x600
#define REG_OTGMIRROR		0x604

/* USB Host/OTG register offsets */
#define REG_USBCMD		0x140
#define REG_USBSTS		0x144
#define REG_ULPIVIEW		0x170
#define REG_PORTSC1		0x184
#define REG_OTGSC		0x1a4
#define REG_USBMODE		0x1a8

/* USBCMD register bits */
#define USBCMD_RST		(1 << 1)
#define USBCMD_RUN		(1 << 0)

/* USBSTS register bits */
#define USBSTS_SRI		(1 << 7)
#define USBSTS_HCH		(1 << 12)

/* USBMODE register bits */
#define USBMODE_CM_SHIFT	0
#define USBMODE_CM_MASK		(3 << USBMODE_CM_SHIFT)
#define USBMODE_CM_IDLE		(0 << USBMODE_CM_SHIFT)
#define USBMODE_CM_DEVICE	(2 << USBMODE_CM_SHIFT)
#define USBMODE_CM_HOST		(3 << USBMODE_CM_SHIFT)

/* USBCTRL register bits */
#define USBCTRL_BPE		(1 << 0)
#define USBCTRL_H1DT		(1 << 4)
#define USBCTRL_H2DT		(1 << 5)
#define USBCTRL_PUE_DWN		(1 << 6)
#define USBCTRL_H1PM		(1 << 8)
#define USBCTRL_H1BPVAL_MSK	(1 << 9)
#define USBCTRL_H1WIE		(1 << 11)
#define USBCTRL_H1SIC_MSK	(3 << 13)
#define USBCTRL_H1WIR		(1 << 15)
#define USBCTRL_H2PM		(1 << 16)
#define USBCTRL_H2WIE		(1 << 19)
#define USBCTRL_H2UIE		(1 << 20)
#define USBCTRL_H2SIC_MSK	(3 << 21)
#define USBCTRL_H2WIR		(1 << 23)
#define USBCTRL_OPM		(1 << 24)
#define USBCTRL_OBPVAL_MSK	(3 << 25)
#define USBCTRL_HEXTEN		(1 << 26)
#define USBCTRL_OWIE		(1 << 27)
#define USBCTRL_OUIE		(1 << 28)
#define USBCTRL_OSIC_MSK	(3 << 29)
#define USBCTRL_OWIR		(1 << 31)

/* OTGSC register bits */
#define OTGSC_ID		(1 << 8)
#define OTGSC_IDPU		(1 << 5)

static inline const char *ulpi_name(void __iomem *view)
{
	if ((unsigned long)view & 0x400) {
		return "USBH2";
	} else {
		return "USBOTG";
	}
}

static int usb3317_init(void __iomem *view)
{
	int vid, pid, ret;

	ret = ulpi_set(ULPI_FCTL_RESET, ISP1504_FCTCTL, view);
	if (ret != 0) {
		goto err;
	}

	ret = ulpi_read(ISP1504_VID_HIGH, view);
	if (ret < 0) {
		goto err;
	}
	vid = ret << 8;

	ret = ulpi_read(ISP1504_VID_LOW, view);
	if (ret < 0) {
		goto err;
	}
	vid |= ret;

	ret = ulpi_read(ISP1504_PID_HIGH, view);
	if (ret < 0) {
		goto err;
	}
	pid = ret << 8;

	ret = ulpi_read(ISP1504_PID_LOW, view);
	if (ret < 0) {
		goto err;
	}
	pid |= ret;

	pr_info("ULPI on %s port Vendor ID 0x%x Product ID 0x%x\n",
		ulpi_name(view), vid, pid);
	if (vid != SMSC_VENDOR_ID || pid != USB3317_PROD_ID) {
		pr_err("No USB3317 found\n");
		return -ENODEV;
	}
 err:
	if (ret < 0) {
		printk(KERN_ERR "ULPI read on %s port failed with error %d\n",
			ulpi_name(view), ret);
		return ret;
	}
	return 0;
}

static int usb3317_set_vbus_power(void __iomem *view, int on)
{
	int ret;

	DBG(0, "%s: Switching %s port VBUS power %s\n", __FUNCTION__,
		ulpi_name(view), on ? "on" : "off");

	if (on) {
		ret = ulpi_set(DRV_VBUS_EXT |		/* enable external Vbus */
				DRV_VBUS |		/* enable internal Vbus */
				CHRG_VBUS,		/* charge Vbus */
				ISP1504_OTGCTL, view);
	} else {
		ret = ulpi_clear(DRV_VBUS_EXT |		/* disable external Vbus */
				DRV_VBUS,		/* disable internal Vbus */
				ISP1504_OTGCTL, view);
		if (ret == 0) {
			ret = ulpi_set(DISCHRG_VBUS,	/* discharge Vbus */
					ISP1504_OTGCTL, view);
		}
	}
	if (ret < 0) {
		printk(KERN_ERR "ULPI read on %s port failed with error %d\n",
			ulpi_name(view), ret);
		return ret;
	}
	return 0;
}

static int tx27_usb_reset(void __iomem *base)
{
	int loops = 100;

	__raw_writel(__raw_readl(base + REG_USBCMD) | USBCMD_RST,
		base + REG_USBCMD);
	while (__raw_readl(base + REG_USBCMD) & USBCMD_RST) {
		udelay(1);
		loops--;
	}

	return loops > 0 ? 0 : -ETIME;
}

static int tx27_usb_init(struct platform_device *pdev, void __iomem *base)
{
	int ret;
	u32 temp;
	unsigned long flags;
	static void __iomem *otg_base = IO_ADDRESS(OTG_BASE_ADDR);

	if (!(__raw_readl(base + REG_USBSTS) & USBSTS_HCH)) {
		temp = __raw_readl(base + REG_USBCMD);
		if (temp & USBCMD_RUN) {
			temp &= ~USBCMD_RUN;
			DBG(0, "%s: Stopping USB controller\n", __FUNCTION__);
			__raw_writel(temp, base + REG_USBCMD);
		}
	}
	WARN_ON(!(__raw_readl(base + REG_USBSTS) & USBSTS_HCH) &&
		(__raw_readl(base + REG_USBCMD) & USBCMD_RUN));

	if ((__raw_readl(base + REG_USBMODE) & USBMODE_CM_MASK) !=
		USBMODE_CM_HOST) {
		ret = tx27_usb_reset(base);
		if (ret != 0) {
			DBG(0, "%s: USB reset timed out\n", __FUNCTION__);
			return ret;
		}
		/* Set to Host mode */
		temp = __raw_readl(base + REG_USBMODE);
		temp |= 3;
		__raw_writel(temp, base + REG_USBMODE);
	}

	WARN_ON((__raw_readl(base + REG_USBMODE) & 3) != 3);

	local_irq_save(flags);
	temp = __raw_readl(otg_base + REG_USBCTRL);
	if (pdev->id == 0) {
		temp &= ~(USBCTRL_OSIC_MSK | USBCTRL_OPM | USBCTRL_BPE | USBCTRL_OUIE);
		temp |= USBCTRL_OWIE | USBCTRL_OUIE;
	} else {
		temp &= ~(USBCTRL_H2SIC_MSK | USBCTRL_H2PM | USBCTRL_H2UIE);
		temp |= USBCTRL_H2DT | USBCTRL_H2WIE | USBCTRL_H2UIE;
	}
	__raw_writel(temp, otg_base + REG_USBCTRL);
	local_irq_restore(flags);

	/* select ULPI transceiver */
	temp = __raw_readl(base + REG_PORTSC1);
	temp &= ~(3 << 30);
	temp |= 2 << 30;
	temp |= 0 << 23;	/* PHCD: PHY clock disable */
	__raw_writel(temp, base + REG_PORTSC1);

	ret = tx27_usb_reset(base);
	if (ret != 0) {
		return ret;
	}
	msleep(100);
	return ret;
}

#ifdef CONFIG_ARCH_MXC_EHCI_USBH2
static int tx27_usbh2_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = IO_ADDRESS(OTG_BASE_ADDR + 0x400);
	struct clk *usbclk = clk_get(NULL, "usb_clk");

	if (IS_ERR(usbclk)) {
		printk("Failed to get usb_clk: %ld\n", PTR_ERR(usbclk));
		return PTR_ERR(usbclk);
	}

	ret = gpio_usbh2_active();
	if (ret != 0) {
		clk_put(usbclk);
		return ret;
	}

	ret = tx27_usb_init(pdev, base);
	if (ret != 0) {
		DBG(0, "%s: tx27_usb_init failed: %d\n", __FUNCTION__, ret);
		goto err;
	}

	ret = usb3317_init(base + REG_ULPIVIEW);
	if (ret != 0) {
		goto err;
	}
	ret = usb3317_set_vbus_power(base + REG_ULPIVIEW, 1);
	if (ret != 0) {
		goto err;
	}
	clk_disable(usbclk);
	clk_put(usbclk);
	return 0;

 err:
	gpio_usbh2_inactive();
	clk_put(usbclk);
	return ret;
}

static int tx27_usbh2_exit(struct platform_device *pdev)
{
	struct clk *usbclk = clk_get(NULL, "usb_clk");

	if (!IS_ERR(usbclk)) {
		/* reenable USB clock, since the driver will disable it upon removal */
		clk_enable(usbclk);
		clk_put(usbclk);
	}

	gpio_usbh2_inactive();
	return 0;
}

static struct mxc_usbh_platform_data tx27_usbh2_data = {
	.init = tx27_usbh2_init,
	.exit = tx27_usbh2_exit,
};

int tx27_usbh2_register(void)
{
	int ret;

	ret = mxc_register_device(&mxc_ehci2, &tx27_usbh2_data);
	return ret;
}
device_initcall(tx27_usbh2_register);
#endif // CONFIG_ARCH_MXC_EHCI_USBH2

#ifdef CONFIG_ARCH_MXC_EHCI_USBOTG
static int tx27_usbotg_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = IO_ADDRESS(OTG_BASE_ADDR + 0x0);
	struct clk *usbclk = clk_get(NULL, "usb_clk");

	if (IS_ERR(usbclk)) {
		printk("Failed to get usb_clk: %ld\n", PTR_ERR(usbclk));
		return PTR_ERR(usbclk);
	}

	/* disable sampling of ID pin for OTG port being used as HOST */
	__raw_writel(__raw_readl(base + REG_OTGSC) & ~OTGSC_IDPU,
		base + REG_OTGSC);

	ret = gpio_usbotg_hs_active();
	if (ret != 0) {
		return ret;
	}

	ret = tx27_usb_init(pdev, base);
	if (ret != 0) {
		DBG(0, "%s: tx27_usb_init failed: %d\n", __FUNCTION__, ret);
		goto err;
	}
	WARN_ON(__raw_readl(base + REG_OTGSC) & OTGSC_IDPU);

	ret = usb3317_init(base + REG_ULPIVIEW);
	if (ret != 0) {
		goto err;
	}

	ret = usb3317_set_vbus_power(base + REG_ULPIVIEW, 1);
	if (ret != 0) {
		goto err;
	}
	clk_disable(usbclk);
	clk_put(usbclk);
	return 0;

err:
	gpio_usbotg_hs_inactive();
	clk_put(usbclk);
	return ret;
}

static int tx27_usbotg_exit(struct platform_device *pdev)
{
	struct clk *usbclk = clk_get(NULL, "usb_clk");

	if (!IS_ERR(usbclk)) {
		/* reenable USB clock, since the driver will disable it upon removal */
		clk_enable(usbclk);
		clk_put(usbclk);
	}

	gpio_usbotg_hs_inactive();
	return 0;
}

static struct mxc_usbh_platform_data tx27_usbotg_data = {
	.init = tx27_usbotg_init,
	.exit = tx27_usbotg_exit,
};

static int tx27_usbotg_register(void)
{
	int ret;

	ret = mxc_register_device(&mxc_otg, &tx27_usbotg_data);
	return ret;
}
device_initcall(tx27_usbotg_register);
#endif // CONFIG_ARCH_MXC_EHCI_USBOTG
#endif // CONFIG_USB_EHCI_MXC

#if defined(CONFIG_USB_FSL_USB2) || defined(CONFIG_USB_FSL_USB2_MODULE)
static struct fsl_usb2_platform_data tx27_usb_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_ULPI,
};

static int __init tx27_usb_gadget_register(void)
{
	int ret;

	ret = gpio_usbotg_hs_active();
	if (ret) {
		printk(KERN_ERR "Failed to initialize USB OTG pins for ULPI interface: %d\n", ret);
		return ret;
	}
	return mxc_register_device(&mxc_otg_udc_device, &tx27_usb_pdata);
}
device_initcall(tx27_usb_gadget_register);
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
static struct resource fec_resources[] = {
	{
		.start	= FEC_BASE_ADDR,
		.end	= FEC_BASE_ADDR + 0x18f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= FEC_BASE_ADDR + 0x200,
		.end	= FEC_BASE_ADDR + 0x2e3,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MXC_INT_FEC,
		.end	= MXC_INT_FEC,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef FEC_MII_IRQ
	{
		.start	= IRQ_GPIOD(16),
		.end	= IRQ_GPIOD(16),
		.flags	= IORESOURCE_IRQ,
	},
#endif
};

static struct clk *fec_clk;
static int tx27_fec_suspend(struct platform_device *pdev)
{
	BUG_ON(fec_clk == NULL);
	DBG(1, "%s: Switching FEC PHY off\n", __FUNCTION__);
	gpio_fec_inactive();
	clk_disable(fec_clk);
	return 0;
}

static int tx27_fec_resume(struct platform_device *pdev)
{
	BUG_ON(fec_clk == NULL);
	DBG(1, "%s: Switching FEC PHY on\n", __FUNCTION__);
	clk_enable(fec_clk);
	gpio_fec_active();
	return 0;
}

static int fec_arch_init(struct platform_device *pdev)
{
	int ret;

	DBG(0, "%s: Activating FEC GPIOs\n", __FUNCTION__);
	dump_regs();
	ret = gpio_fec_active();
	if (ret) {
		printk(KERN_ERR "%s: could not enable FEC gpios: %d\n", __FUNCTION__, ret);
		return ret;
	}
	BUG_ON(fec_clk != NULL);
	fec_clk = clk_get(&pdev->dev, "fec_clk");
	if (unlikely(IS_ERR(fec_clk))) {
		printk(KERN_ERR "Failed to get fec_clk\n");
		return PTR_ERR(fec_clk);
	}
	DBG(0, "%s: Enabling FEC clock\n", __FUNCTION__);
	clk_enable(fec_clk);
	dump_regs();
	return 0;
}

static void fec_arch_exit(struct platform_device *pdev)
{
	BUG_ON(fec_clk == NULL);
	if (unlikely(IS_ERR(fec_clk))) {
		printk(KERN_ERR "Failed to get fec_clk\n");
		return;
	}
	DBG(0, "%s: Disabling FEC clock\n", __FUNCTION__);
	clk_disable(fec_clk);
	clk_put(fec_clk);
	fec_clk = NULL;
	DBG(0, "%s: Deactivating FEC GPIOs\n", __FUNCTION__);
	gpio_fec_inactive();
}

static struct fec_enet_platform_data fec_data = {
	.arch_init = fec_arch_init,
	.arch_exit = fec_arch_exit,
	.suspend = tx27_fec_suspend,
	.resume = tx27_fec_resume,
};

static struct platform_device fec_device = {
	.name		= "fec_enet",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(fec_resources),
	.resource	= fec_resources,
	.dev = {
		.platform_data = &fec_data,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
};
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
/* tx27 gpio keys driver */
struct gpio_keys_button tx27_gpio_keys[] = {
	{
		.code = KEY_POWER,
		.gpio = GPIO_PORTB + 24,
		.active_low = 0,
		.desc = "Power Button",
		.type = EV_KEY,		/* input event type (EV_KEY, EV_SW) */
		.wakeup = 1,		/* configure the button as a wake-up source */
		.debounce_interval = 1,	/* debounce ticks interval in msecs */
	},
};

struct gpio_keys_platform_data tx27_gpio_keys_pdata = {
	.buttons = tx27_gpio_keys,
	.nbuttons = ARRAY_SIZE(tx27_gpio_keys),
};

static struct platform_device tx27_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &tx27_gpio_keys_pdata,
	},
};
#endif

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led tx27_leds[] = {
	{
		.name = "GPIO-LED",
		.default_trigger = "heartbeat",
		.gpio = GPIO_PORTF + 13,
	},
};

static struct gpio_led_platform_data tx27_led_data = {
	.leds = tx27_leds,
	.num_leds = ARRAY_SIZE(tx27_leds),
};

static struct platform_device tx27_led_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &tx27_led_data,
	},
};
#endif

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
/*!
 * This array is used for mapping mx27 ADS keypad scancodes to input keyboard
 * keycodes.
 */
static u16 tx27_kpd_keycodes[] = {
	KEY_POWER,
};

static struct keypad_data tx27_keypad = {
	.rowmax = 1,
	.colmax = 1,
	.irq = MXC_INT_KPP,
	.learning = 0,
	//.delay = 2, /* unused in the driver! */
	.matrix = tx27_kpd_keycodes,
};

static struct resource tx27_kpp_resources[] = {
	{
		.start = MXC_INT_KPP,
		.end = MXC_INT_KPP,
		.flags = IORESOURCE_IRQ,
	},
};

/* tx27 keypad driver */
static struct platform_device tx27_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(tx27_kpp_resources),
	.resource = tx27_kpp_resources,
	.dev = {
		.platform_data = &tx27_keypad,
	},
};
#endif

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)
static struct mxc_nand_platform_data tx27_nand_data = {
	.hw_ecc = 1,
	.width = 1,
};

static struct resource tx27_nand_resources[] = {
	{
		.start	= NFC_BASE_ADDR,
		.end	= NFC_BASE_ADDR + 0xfff,
		.flags	= IORESOURCE_MEM
	}, {
		.start	= MXC_INT_NANDFC,
		.end	= MXC_INT_NANDFC,
		.flags	= IORESOURCE_IRQ
	},
};

static struct platform_device tx27_nand_mtd_device = {
	.name = "mxc_nand",
	.id = 0,
	.num_resources = ARRAY_SIZE(tx27_nand_resources),
	.resource = tx27_nand_resources,
	.dev = {
		.platform_data = &tx27_nand_data,
	},
};
#endif

#if defined(CONFIG_FB_IMX) || defined(CONFIG_FB_IMX_MODULE)
/*
 * Setup GPIO for LCDC device to be active
 *
 */
#define TX27_LCDC_BACKLIGHT_GPIO	(GPIO_PORTE |  5)
#define TX27_LCDC_RESET_GPIO		(GPIO_PORTF | 12)

static unsigned int mx27_lcdc_gpios[] = {
	MXC_PIN(F, 12, GPIO, GPIO_OUT | GPIO_DFLT_LOW), /* LCD reset (active LOW) */
	MXC_PIN(E,  5, GPIO, GPIO_OUT | GPIO_DFLT_HIGH), /* LCD backlight (PWM: 0: off 1: max brightness */
	MXC_PIN(A, 30, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* PA30 */
	MXC_PIN(A, 25, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* PA25 */
	MXC_PIN(A, 26, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* PA26 */
	MXC_PIN(A, 24, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* PA24 */
	MXC_PIN(A, 27, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* PA27 */
	PA5_PF_LSCLK,
	PA6_PF_LD0,
	PA7_PF_LD1,
	PA8_PF_LD2,
	PA9_PF_LD3,
	PA10_PF_LD4,
	PA11_PF_LD5,
	PA12_PF_LD6,
	PA13_PF_LD7,
	PA14_PF_LD8,
	PA15_PF_LD9,
	PA16_PF_LD10,
	PA17_PF_LD11,
	PA18_PF_LD12,
	PA19_PF_LD13,
	PA20_PF_LD14,
	PA21_PF_LD15,
	PA22_PF_LD16,
	PA23_PF_LD17,
	PA28_PF_HSYNC,
	PA29_PF_VSYNC,
	PA31_PF_OE_ACD,
};

static int tx27_gpio_lcdc_active(struct platform_device *dev)
{
	int ret;

	DBG(0, "%s: Setting up GPIO pins for LCD\n", __FUNCTION__);
	ret = mxc_gpio_setup_multiple_pins(mx27_lcdc_gpios,
					ARRAY_SIZE(mx27_lcdc_gpios), "LCD");
	if (ret) {
		DBG(0, "%s: Failed to setup GPIO pins for LCD: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
	return 0;
}

/*
 * Setup GPIO for LCDC device to be inactive
 *
 */
static int tx27_gpio_lcdc_inactive(struct platform_device *dev)
{
	mxc_gpio_release_multiple_pins(mx27_lcdc_gpios,
				ARRAY_SIZE(mx27_lcdc_gpios));
	return 0;
}

static void tx27_lcdc_power(int on)
{
	DBG(0, "%s: Switching LCD RESET %s\n", __FUNCTION__,
		on ? "off" : "on");
	if (on) {
		gpio_set_value(TX27_LCDC_RESET_GPIO, 1);
	} else {
		gpio_set_value(TX27_LCDC_RESET_GPIO, 0);
	}
}

static void tx27_lcdc_backlight(int on)
{
	DBG(0, "%s: Switching LCD BACKLIGHT %s\n", __FUNCTION__,
		on ? "on" : "off");
	if (on) {
		gpio_set_value(TX27_LCDC_BACKLIGHT_GPIO, 0);
	} else {
		gpio_set_value(TX27_LCDC_BACKLIGHT_GPIO, 1);
	}
}

static struct imx_fb_platform_data tx27_fb_data[] __initdata = {
	/* The imxfb driver erroneously interprets the '.pixclock'
	 * as pixel clock in kHz, instead of pixel clock period in ps */
	{
		.bpp	= 32,
		//.mode = {
			//.name = "G-ETV570G0DMU",
			.pixclock	= 22166,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.left_margin	= 23,
			.right_margin	= 23,

			.vsync_len	= 3,
			.upper_margin	= 5,
			.lower_margin	= 8,
		//},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,

		.dmacr		= 0x80040060,
		.init = tx27_gpio_lcdc_active,
		.exit = tx27_gpio_lcdc_inactive,
		.lcd_power = tx27_lcdc_power,
		.backlight_power = tx27_lcdc_backlight,
	},
	{
		//.fb_mode = "Xenarc_700_Y-18",
		.init = tx27_gpio_lcdc_active,
		.exit = tx27_gpio_lcdc_inactive,
		.lcd_power = NULL,
		.backlight_power = NULL,

		.pixclock	= 34576,
		.xres		= 800,
		.yres		= 480,

		.bpp		= 32,

		.hsync_len	= 64,
		.right_margin	= 138 + 1,
		.left_margin	= 118 + 3,

		.vsync_len	= 7,
		.upper_margin	= 44,
		.lower_margin	= 44,
#if 0
		/* currently not used by driver! */
		.sync		= ((0*FB_SYNC_HOR_HIGH_ACT) |
				   (0*FB_SYNC_VERT_HIGH_ACT) |
				   (1*FB_SYNC_OE_ACT_HIGH)),
#else
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
		.dmacr		= 0x80040060,
#endif
		.cmap_greyscale	= 0,
		.cmap_inverse	= 0,
		.cmap_static	= 0,

		.fixed_screen_cpu = NULL,
	},
	{
		.init = tx27_gpio_lcdc_active,
		.exit = tx27_gpio_lcdc_inactive,
		.lcd_power = NULL,
		.backlight_power = NULL,

		.pixclock	= 34576,
		.xres		= 640,
		.yres		= 480,

		.bpp		= 16,

		.hsync_len	= 64,
		.right_margin	= 138 + 1,
		.left_margin	= 118 + 3,

		.vsync_len	= 7,
		.upper_margin	= 44,
		.lower_margin	= 44,
#if 0
		/* currently not used by driver! */
		.sync		= ((0*FB_SYNC_HOR_HIGH_ACT) |
				   (0*FB_SYNC_VERT_HIGH_ACT) |
				   (1*FB_SYNC_OE_ACT_HIGH)),
#else
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
		.dmacr		= 0x80040060,
#endif
		.cmap_greyscale	= 0,
		.cmap_inverse	= 0,
		.cmap_static	= 0,

		.fixed_screen_cpu = NULL,
	},
	{
		//.fb_mode = "SHARP LQ10D42-16",
		.init = tx27_gpio_lcdc_active,
		.exit = tx27_gpio_lcdc_inactive,
		.lcd_power = NULL,
		.backlight_power = NULL,

		.pixclock	= 34576,
		.xres		= 640,
		.yres		= 480,

#ifdef USE_18BPP
		.bpp		= 32,
#else
		.bpp		= 16,
#endif
		.hsync_len	= 64,
		.right_margin	= 138 + 1,
		.left_margin	= 118 + 3,

		.vsync_len	= 7,
		.upper_margin	= 28,
		.lower_margin	= 60,
#if 0
		/* currently not used by driver! */
		.sync		= ((0*FB_SYNC_HOR_HIGH_ACT) |
				   (0*FB_SYNC_VERT_HIGH_ACT) |
				   (1*FB_SYNC_OE_ACT_HIGH)),
#else
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
#ifdef USE_18BPP
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
#else
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
#endif
		.dmacr		= 0x80040060,
#endif
		.cmap_greyscale	= 0,
		.cmap_inverse	= 0,
		.cmap_static	= 0,

		.fixed_screen_cpu = NULL,
	},
	{
		//.fb_mode = "SHARP LQ104V1DG61-16",
		.init = tx27_gpio_lcdc_active,
		.exit = tx27_gpio_lcdc_inactive,
		.lcd_power = NULL,
		.backlight_power = NULL,

		.pixclock	= 40000,
		.xres		= 640,
		.yres		= 480,

#ifdef USE_18BPP
		.bpp		= 32,
#else
		.bpp		= 16,
#endif
		.hsync_len	= 32,
		.right_margin	= 32 + 1,
		.left_margin	= 0 + 3,

		.vsync_len	= 35,
		.upper_margin	= 0,
		.lower_margin	= 0,
#if 0
		/* currently not used by driver! */
		.sync		= ((0*FB_SYNC_HOR_HIGH_ACT) |
				   (0*FB_SYNC_VERT_HIGH_ACT) |
				   (1*FB_SYNC_OE_ACT_HIGH)),
#else
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
#ifdef USE_18BPP
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
#else
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_CLKPOL | PCR_SCLK_SEL,
#endif
		.dmacr		= 0x80040060,
#endif
		.cmap_greyscale	= 0,
		.cmap_inverse	= 0,
		.cmap_static	= 0,

		.fixed_screen_cpu = NULL,
	},
	{
		//.fb_mode = "Hantronix S570-S",
		.init = tx27_gpio_lcdc_active,
		.exit = tx27_gpio_lcdc_inactive,

		.pixclock	= 156000, // CLK Period (in picoseconds)
		.xres		= 320,    // Horizontal Display Time
		.yres		= 240,    // Vertical Display Time

		.bpp		= 32,     // 18bpp on LCD -> 32bpp in memory

		.hsync_len	= 30,     // HS Pulse Width
		.right_margin	= 38,     // Horizontal Back Porch
		.left_margin	= 20,     // Horizontal Front Porch

		.vsync_len	= 3,      // VS Pulse Width
		.upper_margin	= 4,      // Vertical Front Porch
		.lower_margin	= 15,     // Vertical Back Porch

		.pcr		= PCR_TFT | PCR_COLOR | PCR_BPIX_18 |
		PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
		.dmacr		= 0x80040060,

		.cmap_greyscale	= 0,
		.cmap_inverse	= 0,
		.cmap_static	= 0,

		.fixed_screen_cpu = NULL,
	},
};

int __init karo_tx27_fb_init(void)
{
	int ret;

	ret = mxc_register_device(&mxc_fb_device, &tx27_fb_data[0]);
	if (ret != 0) {
		DBG(0, "%s: Failed to register FB device: %d\n", __FUNCTION__, ret);
	}
	return ret;
}
device_initcall(karo_tx27_fb_init);
#endif

#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)
/*!
 * Resource definition for the SDHC1
 */
static struct resource tx27_sdhc1_resources[] = {
	{
		.start = SDHC1_BASE_ADDR,
		.end = SDHC1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SDHC1,
		.end = MXC_INT_SDHC1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_GPIOC(21),
		.end = IRQ_GPIOC(21),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name   = "sdhc1",
		.start  = DMA_REQ_SDHC1,
		.end    = DMA_REQ_SDHC1,
		.flags  = IORESOURCE_DMA
	},
};

/*!
 * Resource definition for the SDHC2
 */
static struct resource tx27_sdhc2_resources[] = {
	{
		.start = SDHC2_BASE_ADDR,
		.end = SDHC2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SDHC2,
		.end = MXC_INT_SDHC2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_GPIOC(22),
		.end = IRQ_GPIOC(22),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name   = "sdhc2",
		.start  = DMA_REQ_SDHC2,
		.end    = DMA_REQ_SDHC2,
		.flags  = IORESOURCE_DMA
	},
};
static inline int tx27_mmc_get_irq(int id)
{
	int irq;

	switch (id) {
	case 0:
		irq = tx27_sdhc1_resources[2].start;
		break;
	case 1:
		irq = tx27_sdhc2_resources[2].start;
		break;
	default:
		BUG();
	}
	return irq;
}

static const char *tx27_mmc_irqdesc[] = {
	"MMC card 0 detect",
	"MMC card 1 detect",
};

static int tx27_mmc_init(struct device *dev, irqreturn_t (*mmc_detect_irq)(int, void *),
			 void *data)
{
	int err;
	int id = to_platform_device(dev)->id;
	struct mmc_host *host = data;
	int irq = tx27_mmc_get_irq(id);

	err = gpio_sdhc_active(id);
	if (err) {
		return err;
	}

	host->caps |= MMC_CAP_4_BIT_DATA;

	err = request_irq(irq, mmc_detect_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			tx27_mmc_irqdesc[id], data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD: can't request MMC card detect IRQ %d\n",
			__FUNCTION__, irq);
		return err;
	}
	device_set_wakeup_capable(dev, 1);

	return 0;
}

static void tx27_mmc_exit(struct device *dev, void *data)
{
	int id = to_platform_device(dev)->id;
	int irq = tx27_mmc_get_irq(id);

	free_irq(irq, data);
	gpio_sdhc_inactive(id);
}

static int tx27_mmc_suspend(struct device *dev, pm_message_t state)
{
	int id = to_platform_device(dev)->id;
	int irq = tx27_mmc_get_irq(id);

	if (device_may_wakeup(dev)) {
		DBG(0, "%s: Enabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return enable_irq_wake(irq);
	}
	return 0;
}

static int tx27_mmc_resume(struct device *dev)
{
	int id = to_platform_device(dev)->id;
	int irq = tx27_mmc_get_irq(id);

	if (device_may_wakeup(dev)) {
		DBG(0, "%s: Disabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return disable_irq_wake(irq);
	}
	return 0;
}

static struct imxmmc_platform_data tx27_sdhc1_data = {
	//.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	//.min_clk = 150000,
	//.max_clk = 25000000,
	//.detect_delay = 20,
	.init = tx27_mmc_init,
	.exit = tx27_mmc_exit,
	.suspend = tx27_mmc_suspend,
	.resume = tx27_mmc_resume,
};

static struct imxmmc_platform_data tx27_sdhc2_data = {
	//.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	//.min_clk = 150000,
	//.max_clk = 25000000,
	//.detect_delay = 20,
	.init = tx27_mmc_init,
	.exit = tx27_mmc_exit,
};

static struct platform_device tx27_sdhc1_device = {
	.name = "imx-mmc",
	.id = 0,
	.dev = {
		.platform_data = &tx27_sdhc1_data,
	},
	.num_resources = ARRAY_SIZE(tx27_sdhc1_resources),
	.resource = tx27_sdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device tx27_sdhc2_device = {
	.name = "imx-mmc",
	.id = 1,
	.dev = {
		.platform_data = &tx27_sdhc2_data,
	},
	.num_resources = ARRAY_SIZE(tx27_sdhc2_resources),
	.resource = tx27_sdhc2_resources,
};
#endif

#if defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)
static struct resource mxcspi1_resources[] = {
	[0] = {
	       .start = CSPI1_BASE_ADDR,
	       .end = CSPI1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	},
	[1] = {
	       .start = MXC_INT_CSPI1,
	       .end = MXC_INT_CSPI1,
	       .flags = IORESOURCE_IRQ,
	},
};

extern int gpio_spi_active(int cspi_mod);
extern int gpio_spi_inactive(int cspi_mod);

static int tx27_spi_init(struct platform_device *pdev)
{
	if (pdev->id < 0 || pdev->id > 2) {
		return -EINVAL;
	}
	return gpio_spi_active(pdev->id);
}

static int tx27_spi_exit(struct platform_device *pdev)
{
	if (pdev->id < 0 || pdev->id > 2) {
		return -EINVAL;
	}
	return gpio_spi_inactive(pdev->id);
}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 2,
	.spi_version = 0,
	.init = tx27_spi_init,
	.exit = tx27_spi_exit,
};

static struct platform_device mxcspi1_device = {
	.name = "mxc_spi",
	.id = 0,
	.dev = {
		.platform_data = &mxcspi1_data,
	},
	.num_resources = ARRAY_SIZE(mxcspi1_resources),
	.resource = mxcspi1_resources,
};
#endif // defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)

#if defined(CONFIG_AC97_BUS) || defined(CONFIG_AC97_BUS_MODULE)
static u64 tx27_dma_mask = ~0UL;

static void tx27_ac97_gpio_release(void)
{
	gpio_ac97_inactive();
}

static int tx27_ac97_init(struct platform_device *dev)
{
	int ret;
	const int irq = gpio_to_irq(GPIO_PORTC | 23);

	DBG(0, "%s: \n", __FUNCTION__);

	/*
	 * IRQs are disabled for probing whenever enable_irq() is called.
	 * Thus reenable the UCB1400 IRQ for probing in case the ucb1400-ts
	 * module has been reloaded.
	 */
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	ret = gpio_ac97_active();
	if (ret) {

		return ret;
	}
	return 0;
}

static void tx27_ac97_exit(struct platform_device *dev)
{
	DBG(0, "%s: Releasing AC97 GPIO pins\n", __FUNCTION__);
	tx27_ac97_gpio_release();
}

static struct mxc_ac97_audio_ops tx27_ac97_ops = {
	.init = tx27_ac97_init,
	.exit = tx27_ac97_exit,
	.startup = NULL,
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
	.priv = NULL,
};

static struct platform_device ac97_device = {
	.name		= "mx27-ac97",
	.id		= -1,
	.dev = {
		.dma_mask = &tx27_dma_mask,
		.coherent_dma_mask = ~0UL,
		.platform_data = &tx27_ac97_ops,
	},
};
#endif

#if defined(CONFIG_RTC_DRV_DS13XX) || defined(CONFIG_RTC_DRV_DS13XX_MODULE)
static struct ds13xx_platform_data tx27_ds1339_data = {
	.type = ds_1339,
	.ctrl = 1 << 2, /* set INTCN to disable SQW output */
	.trc = DS1339_TRC_ENABLE | DS1339_DIODE_ENABLE | DS1339_TRC_250R,
};

static struct platform_device tx27_ds1339_device = {
	.name = "rtc-ds13xx",
	.dev = {
		.platform_data = &tx27_ds1339_data,
	},
};
#endif

#if defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT) || defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT_MODULE)
static u64 mxc_emma_dmamask = 0xffffffffUL;

static struct platform_device tx27_v4l2out_device = {
	.name = "MXC Video Output",
	.id = 0,
	.dev = {
		.dma_mask = &mxc_emma_dmamask,
		.coherent_dma_mask = ~0UL,
	},
};
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int mxc_i2c0_pins[] = {
	PD17_PF_I2C_DATA,
	PD18_PF_I2C_CLK,
};

static int karo_tx27_i2c_0_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c0_pins,
					ARRAY_SIZE(mxc_i2c0_pins), "I2C0");
}

static int karo_tx27_i2c_0_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_i2c0_pins,
				ARRAY_SIZE(mxc_i2c0_pins));

	return 0;
}

static struct imx_i2c_platform_data karo_tx27_i2c_0_data = {
	.max_clk = 100000,
	.init = karo_tx27_i2c_0_init,
	.exit = karo_tx27_i2c_0_exit,
};

#if defined(CONFIG_RTC_DRV_DS13XX) || defined(CONFIG_RTC_DRV_DS13XX_MODULE)
static struct ds13xx_platform_data karo_ds1339_data = {
	.type = ds_1339,
	.ctrl = 0,
	.trc = DS1339_TRC_ENABLE | DS1339_DIODE_ENABLE | DS1339_TRC_250R,
};
#endif

static struct at24_platform_data karo_tx27_eeprom = {
	.byte_len = 2048,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16 | AT24_FLAG_TAKE8ADDR,
};

static struct i2c_board_info karo_i2c_0_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("24c16", 0x50),
		.platform_data = &karo_tx27_eeprom,
		.type = "24c16",
	},
#if defined(CONFIG_RTC_DRV_DS13XX) || defined(CONFIG_RTC_DRV_DS13XX_MODULE)
	{
		I2C_BOARD_INFO("ds1339", 0x68/*DS1339_CHIP_ID*/),
		.type = "ds1339",
		.platform_data = &karo_ds1339_data,
	},
#endif
#if defined(CONFIG_RTC_DRV_DS1307) || defined(CONFIG_RTC_DRV_DS1307_MODULE)
	{
		I2C_BOARD_INFO("ds1339", 0x68/*DS1339_CHIP_ID*/),
		.type = "ds1339",
		//.platform_data = &karo_ds1339_data,
	},
#endif
};

static int mxc_i2c_1_pins[] = {
	PC5_PF_I2C2_SDA,
	PC6_PF_I2C2_SCL,
};

static int karo_tx27_i2c_1_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_i2c_1_pins,
					ARRAY_SIZE(mxc_i2c_1_pins), "I2C_1");
}

static int karo_tx27_i2c_1_exit(struct platform_device *pdev)
{
	mxc_gpio_release_multiple_pins(mxc_i2c_1_pins,
				ARRAY_SIZE(mxc_i2c_1_pins));

	return 0;
}

static struct imx_i2c_platform_data karo_tx27_i2c_1_data = {
	.max_clk = 100000,
	.init = karo_tx27_i2c_1_init,
	.exit = karo_tx27_i2c_1_exit,
};

static struct i2c_board_info karo_i2c_1_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("lp3972", 0x34),
		.type = "lp3972",
	},
};

int __init karo_i2c_init(void)
{
	int ret;

	DBG(0, "%s: Registering I2C bus 0\n", __FUNCTION__);
	ret = mxc_register_device(&imx_i2c_device0, &karo_tx27_i2c_0_data);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C device: %d\n", ret);
		return ret;
	}
	ret = i2c_register_board_info(0, karo_i2c_0_boardinfo,
				ARRAY_SIZE(karo_i2c_0_boardinfo));
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C board info: %d\n", ret);
		platform_device_unregister(&imx_i2c_device0);
	}

	DBG(0, "%s: Registering I2C bus 1\n", __FUNCTION__);
	ret = mxc_register_device(&imx_i2c_device1, &karo_tx27_i2c_1_data);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C device: %d\n", ret);
		return ret;
	}
	ret = i2c_register_board_info(1, karo_i2c_1_boardinfo,
				ARRAY_SIZE(karo_i2c_1_boardinfo));
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C board info: %d\n", ret);
		platform_device_unregister(&imx_i2c_device1);
	}
	return ret;
}
device_initcall(karo_i2c_init);
#endif

struct platform_dev_list {
	struct platform_device *pdev;
	int flag;
} tx27_devices[] __initdata = {
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	{ .pdev = &tx27_gpio_keys_device, .flag = -1, },
#endif
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	{ .pdev = &tx27_led_device, .flag = -1, },
#endif
#if defined(CONFIG_RTC_MXC) || defined(CONFIG_RTC_MXC_MODULE)
	{ .pdev = &mxc_rtc_device, .flag = -1, },
#endif
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)
	{ .pdev = &tx27_nand_mtd_device, .flag = 1, },
#endif
#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
	{ .pdev = &tx27_keypad_device, .flag = 1, },
#endif
#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
	{ .pdev = &fec_device, .flag = 1, },
#endif
#if defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)
	{ .pdev = &mxcspi1_device, .flag = 1, },
#endif
#if defined(CONFIG_MX2_AUDMUX) || defined(CONFIG_MX2_AUDMUX_MODULE)
	{ .pdev = &mx2_audmux_device, .flag = -1, },
#endif
#if defined(CONFIG_AC97_BUS) || defined(CONFIG_AC97_BUS_MODULE)
	{ .pdev = &ac97_device, .flag = 1, },
#endif
#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)
	{ .pdev = &tx27_sdhc1_device, .flag = 1, },
	{ .pdev = &tx27_sdhc2_device, .flag = 1, },
#endif
#if defined(CONFIG_RTC_DRV_DS13XX) || defined(CONFIG_RTC_DRV_DS13XX_MODULE)
	{ .pdev = &tx27_ds1339_device, .flag = 1, },
#endif
#if defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT) || defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT_MODULE)
	{ .pdev = &tx27_v4l2out_device, .flag = 1, },
#endif
#if defined(CONFIG_MXC_VPU) || defined(CONFIG_MXC_VPU_MODULE)
	{ .pdev = &mxc_vpu_device, .flag = 1, },
#endif
};
#define TX27_NUM_DEVICES		ARRAY_SIZE(tx27_devices)

#define OSC26M_ENABLE_PIN	(GPIO_PORTB | 22)

static int _clk_26m_enable(struct clk *clk)
{
	int ret;

	DBG(0, "%s: Switching 26MHz oscillator on\n", __FUNCTION__);
	ret = gpio_request(OSC26M_ENABLE_PIN, "OSC26m");
	if (ret != 0) {
		printk(KERN_ERR "Failed to request 26MHz oscillator enable GPIO: %d\n", ret);
		return ret;
	}
	gpio_set_value(OSC26M_ENABLE_PIN, 1);
	mxc_gpio_mode(OSC26M_ENABLE_PIN | GPIO_GPIO | GPIO_OUT);
	return 0;
}

static void _clk_26m_disable(struct clk *clk)
{
	DBG(0, "%s: Switching 26MHz oscillator off\n", __FUNCTION__);
	gpio_set_value(OSC26M_ENABLE_PIN, 0);
	gpio_free(OSC26M_ENABLE_PIN);
}

static struct clk clk_26m = {
	.name = "clk_26m",
	.enable = _clk_26m_enable,
	.disable = _clk_26m_disable,
};

#ifdef CONFIG_BASE_CLK_26MHz
static __init void karo_tx27_clock_switch(struct clk *_26m_clk)
{
	int loops = 0;
	u32 cscr = __raw_readl(CCM_CSCR);
	u32 ccsr;

	if (_26m_clk != NULL) {
		DBG(0, "%s: Enabling 26MHz clock\n", __FUNCTION__);
		clk_enable(_26m_clk);

		__raw_writel(CCM_MPCTL0_PD_VAL(0) |
			CCM_MPCTL0_MFD_VAL(51) |
			CCM_MPCTL0_MFI_VAL(7) |
			CCM_MPCTL0_MFN_VAL(35), CCM_MPCTL0);

		__raw_writel(CCM_SPCTL0_PD_VAL(1) |
			CCM_SPCTL0_MFD_VAL(12) |
			CCM_SPCTL0_MFI_VAL(9) |
			CCM_SPCTL0_MFN_VAL(3), CCM_SPCTL0);

		cscr |= CCM_CSCR_MCU | CCM_CSCR_SP;
		__raw_writel(cscr, CCM_CSCR);

		cscr |= CCM_CSCR_MPLLRES | CCM_CSCR_SPLLRES;
		__raw_writel(cscr, CCM_CSCR);
		while (__raw_readl(CCM_CSCR) & (CCM_CSCR_MPLLRES | CCM_CSCR_SPLLRES)) {
			udelay(1);
			loops++;
		}
		DBG(0, "PLLs locked after %d loops: CSCR=%08x(%08x)\n", loops,
			__raw_readl(CCM_CSCR), cscr);

		cscr &= ~CCM_CSCR_FPM;
		__raw_writel(cscr, CCM_CSCR);
		DBG(9, "%s: Disabling FPM, DPLL and OSC26M\n", __FUNCTION__);
		ccsr = __raw_readl(CCM_CCSR);
		__raw_writel(ccsr & ~0x300, CCM_CCSR);
		DBG(9, "changing CCSR from %08x to %08x(%08x)\n",
			ccsr, ccsr & ~0x300, __raw_readl(CCM_CCSR));
	} else {
		printk(KERN_INFO "Changing SPCTL0 from %08x to %08x\n",
			__raw_readl(CCM_SPCTL0), CCM_SPCTL0_PD_VAL(2) |
			CCM_SPCTL0_MFD_VAL(755) |
			CCM_SPCTL0_MFI_VAL(11) |
			CCM_SPCTL0_MFN_VAL(-205));

		__raw_writel(CCM_SPCTL0_PD_VAL(2) |
			CCM_SPCTL0_MFD_VAL(755) |
			CCM_SPCTL0_MFI_VAL(11) |
			CCM_SPCTL0_MFN_VAL(-205), CCM_SPCTL0);
	}
}
#else
static inline void karo_tx27_clock_switch(struct clk *_26m_clk)
{
	printk(KERN_INFO "Changing SPCTL0 from %08x to %08x\n",
		__raw_readl(CCM_SPCTL0), CCM_SPCTL0_PD_VAL(2) |
		CCM_SPCTL0_MFD_VAL(755) |
		CCM_SPCTL0_MFI_VAL(11) |
		CCM_SPCTL0_MFN_VAL(-205));

	__raw_writel(CCM_SPCTL0_PD_VAL(2) |
		CCM_SPCTL0_MFD_VAL(755) |
		CCM_SPCTL0_MFI_VAL(11) |
		CCM_SPCTL0_MFN_VAL(-205), CCM_SPCTL0);
}
#endif

static __init void karo_tx27_clock_init(void)
{
	struct clk *cpu_clk;
	struct clk *_26m_clk = NULL;
	int ret;

	ret = clk_register(&clk_26m);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register 26MHz clock: %d\n", ret);
		goto no_26m;
	}
	_26m_clk = clk_get(NULL, "clk_26m");
	if (IS_ERR(_26m_clk)) {
		printk(KERN_ERR "Cannot request 26MHz clock: %ld\n", PTR_ERR(_26m_clk));
		_26m_clk = NULL;
	}
no_26m:
	karo_tx27_clock_switch(_26m_clk);
	mxc_clocks_init(26000000);
	cpu_clk = clk_get(NULL, "cpu_clk");
	if (!IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: Setting CPU clock to 400MHz\n", __FUNCTION__);
		if (clk_set_rate(cpu_clk, 399000000) != 0) {
			printk(KERN_ERR "Failed to set CPU clock rate\n");
		}
	} else {
		printk(KERN_ERR "Failed to get CPU clock: %ld\n",
			PTR_ERR(cpu_clk));
	}
	SHOW_REG(CCM_CSCR);
}

/*
 * preconfigure OCR for all unused GPIOs, so that GPIOs
 * configured as outputs via gpiolib can work.
 * Note, that reconfiguring a pin lateron may disturb this configuration!
 */
static __init void karo_gpio_init(void)
{
	int i;

	for (i = 0; i < 6 * 32; i++) {
		void __iomem *reg = VA_GPIO_BASE;
		int ret;
		int pin = i % 32;
		int port = i / 32;

		reg += (pin < 16) ? MXC_OCR1(port) : MXC_OCR2(port);
		ret = gpio_request(i, NULL);
		if (ret == 0) {
			DBG(0, "Configuring P%c%d (GPIO%d) OCR\n", port + 'A', pin, i);
			ret = __raw_readl(reg);
			ret |= (3 << (pin * 2));
			__raw_writel(ret, reg);
			gpio_free(i);
		} else {
			DBG(0, "Leaving P%c%d (GPIO%d) OCR=%d\n", port + 'A', pin, i,
				(__raw_readl(reg) >> 2 * pin) & 3);
		}
	}
}

static __init void karo_tx27_board_init(void)
{
	int i;

	DBG(0, "%s: \n", __FUNCTION__);
	SHOW_REG(CCM_CSCR);

	for (i = 0; i < ARRAY_SIZE(tx27_uart_devices); i++) {
		int ret;
		int port = tx27_uart_devices[i]->id;

		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, tx27_uart_devices[i],
			&tx27_uart_devices[i]->dev, tx27_uart_devices[i]->name);
		ret = mxc_register_device(tx27_uart_devices[i],
					&tx27_uart_ports[port]);
		if (ret != 0) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, tx27_uart_devices[i]->name, ret);
		}
	}

	dump_regs();

	/* enable SSI3_INT (PC23) for IRQ probing */
	set_irq_flags(gpio_to_irq(GPIO_PORTC | 23), IRQF_VALID | IRQF_PROBE);

	for (i = 0; i < TX27_NUM_DEVICES; i++) {
		int ret;

		if (tx27_devices[i].pdev == NULL) continue;
		if (!tx27_devices[i].flag) {
			DBG(0, "%s: Skipping platform device[%d] @ %p dev %p: %s\n",
				__FUNCTION__, i, tx27_devices[i].pdev, &tx27_devices[i].pdev->dev,
				tx27_devices[i].pdev->name);
			continue;
		}
		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, tx27_devices[i].pdev, &tx27_devices[i].pdev->dev,
			tx27_devices[i].pdev->name);
		ret = platform_device_register(tx27_devices[i].pdev);
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, tx27_devices[i].pdev->name, ret);
		}
	}
	device_init_wakeup(&mxc_rtc_device.dev, 1);

	karo_gpio_init();
	DBG(0, "%s: Done\n", __FUNCTION__);
}

static void __init karo_tx27_map_io(void)
{
	mxc_map_io();
}

static void __init karo_tx27_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static void __init karo_tx27_timer_init(void)
{
	DBG(0, "%s: \n", __FUNCTION__);
	karo_tx27_clock_init();
	mxc_timer_init("gpt_clk.0");
	DBG(0, "%s: Done\n", __FUNCTION__);
}

struct sys_timer karo_tx27_timer = {
	.init	= karo_tx27_timer_init,
};

MACHINE_START(TX27, "Ka-Ro electronics TX27 module (Freescale i.MX27)")
	/* Maintainer: <LW@KARO-electronics.de> */
	.phys_io	= AIPI_BASE_ADDR,
	.io_pg_offst	= ((unsigned long)AIPI_BASE_ADDR_VIRT >> 18) & 0xfffc,
	.fixup		= karo_tx27_fixup,
	.map_io		= karo_tx27_map_io,
	.init_irq	= mxc_init_irq,
	.init_machine	= karo_tx27_board_init,
	.timer		= &karo_tx27_timer,
MACHINE_END
