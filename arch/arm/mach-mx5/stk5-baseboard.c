/*
 * arch/arm/mach-mx5/stk5-baseboard.c
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
#include <linux/ipu.h>
#include <linux/dma-mapping.h>
#include <linux/pwm_backlight.h>
#include <linux/i2c/tsc2007.h>
#include <linux/fsl_devices.h>

#include <asm/clkdev.h>

#include <mach/clock.h>
#include <mach/common.h>
#include <mach/gpio.h>
#include <mach/sdhci.h>
#include <mach/mxc_ehci.h>
#include <mach/iomux-mx51.h>
#include <mach/mxc-ac97.h>
#include <mach/mxc_audmux.h>
#include <mach/mxc_audio.h>
#include <mach/mxc_ipuv3.h>

#include "devices.h"
#include "karo.h"

static int stk5_board_rev;

static struct mx51_pad_desc stk5_pinmux[] __initdata = {
	MX51_PIN_DISPB2_SER_RS__GPIO3_8,	/* SD1_CD */
	MX51_PIN_DISPB2_SER_DIO__GPIO3_6,	/* SD2_CD */
};

static struct mx51_pad_desc stk5_padctrl[] __initdata = {
	{ MX51_PIN_SD1_CMD, (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_47K_PU |
				PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE |
				PAD_CTL_DRV_HIGH), },
	{ MX51_PIN_SD2_CMD, (PAD_CTL_DRV_VOT_HIGH | PAD_CTL_47K_PU |
				PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE |
				PAD_CTL_DRV_HIGH), },
};

static struct mx51_pad_desc stk5_input_select[] __initdata = {
	{ MUX_IN_GPIO3_IPP_IND_G_IN_8_SELECT_INPUT, 1, }, /* GPIO3_8 */
	{ MUX_IN_GPIO3_IPP_IND_G_IN_6_SELECT_INPUT, 1, }, /* GPIO3_6 */
};

static int __init stk5_pad_setup(void)
{
	int ret;

	ret = mx51_iomux_request_pads(stk5_pinmux, ARRAY_SIZE(stk5_pinmux));
	if (ret) {
		return ret;
	}

	mx51_iomux_set_inputs(stk5_input_select, ARRAY_SIZE(stk5_input_select));
	mx51_iomux_set_pads(stk5_padctrl, ARRAY_SIZE(stk5_padctrl));
	mx51_iomux_release_pads(stk5_pinmux, ARRAY_SIZE(stk5_pinmux));
	return 0;
}
postcore_initcall(stk5_pad_setup);

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct mx51_pad_desc stk5_led_pads[] = {
	MX51_PIN_CSI2_D13__GPIO4_10,
};

static struct gpio_led stk5_leds[] = {
	{
		.name = "GPIO-LED",
		.default_trigger = "heartbeat",
		.gpio = IOMUX_TO_GPIO(MX51_PIN_CSI2_D13),
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
	int i;

	ret = mx51_iomux_request_pads(stk5_led_pads,
				ARRAY_SIZE(stk5_led_pads));
	if (ret)
		DBG(0, "%s: Failed to setup PADS for LED: %d\n",
			__FUNCTION__, ret);

	for (i = 0; i < ARRAY_SIZE(stk5_led_pads); i++) {
		gpio_direction_output(IOMUX_TO_GPIO(stk5_led_pads[i].pad), 0);
		gpio_free(IOMUX_TO_GPIO(stk5_led_pads[i].pad));
	}
	return ret;
}
arch_initcall(stk5_led_init);
#endif

#if defined(CONFIG_MMC_SDHCI_MXC) || defined(CONFIG_MMC_SDHCI_MXC_MODULE)
static struct mx51_pad_desc tx51_sdhci_cd_pads[] = {
	MX51_PIN_DISPB2_SER_RS__GPIO3_8,  /* GPIO3_8 */
	MX51_PIN_DISPB2_SER_DIO__GPIO3_6, /* GPIO3_6 */
};

static struct mx51_pad_desc tx51_sdhci_cd_padctl[] = {
	{ MX51_PIN_DISPB2_SER_RS, PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE, },
	{ MX51_PIN_DISPB2_SER_DIO, PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE, },
};

static struct mx51_pad_desc tx51_sdhci_cd_inputs[] = {
	{ MUX_IN_GPIO3_IPP_IND_G_IN_8_SELECT_INPUT, 1, },
	{ MUX_IN_GPIO3_IPP_IND_G_IN_6_SELECT_INPUT, 1, },
};

static struct mx51_pad_desc tx51_sdhci_pads[][6] = {
	{
		{ MX51_PIN_SD1_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION, },
		{ MX51_PIN_SD1_CLK, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD1_DATA0, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD1_DATA0, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD1_DATA0, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD1_DATA0, IOMUX_CONFIG_ALT0, },
	},
	{
		{ MX51_PIN_SD2_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION, },
		{ MX51_PIN_SD2_CLK, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD2_DATA0, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD2_DATA0, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD2_DATA0, IOMUX_CONFIG_ALT0, },
		{ MX51_PIN_SD2_DATA0, IOMUX_CONFIG_ALT0, },
	},
};

static int tx51_esdhci_init(struct device *dev,
			irq_handler_t detect_irq, void *data)
{
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	int gpio;
	int irq;

	BUG_ON(pdev->id < 0 || pdev->id >= ARRAY_SIZE(tx51_sdhci_cd_pads));

	ret = mx51_iomux_request_pads(tx51_sdhci_pads[pdev->id],
				ARRAY_SIZE(tx51_sdhci_pads[pdev->id]));

	if (ret) {
		printk(KERN_ERR "%s: Error %d configuring pads for SDHC # %d\n",
			__FUNCTION__, ret, pdev->id);
		return ret;
	}
	mx51_iomux_set_pads(&tx51_sdhci_cd_padctl[pdev->id], 1);
	mx51_iomux_set_inputs(&tx51_sdhci_cd_inputs[pdev->id], 1);

	gpio = IOMUX_TO_GPIO(tx51_sdhci_cd_pads[pdev->id].pad);
	irq = gpio_to_irq(gpio);

	DBG(0, "%s: Requesting IRQ%d GPIO%d for SDHCI # %d CD\n", __FUNCTION__,
		irq, gpio, pdev->id);
	ret = gpio_request(gpio, "sdhci-mxc-detect");
	if (ret) {
		DBG(0, "%s: Failed to request GPIO %d\n", __FUNCTION__,
			gpio);
		return ret;
	}
	gpio_direction_input(gpio);

	ret = request_irq(irq, detect_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"sdhci-mxc-detect", data);
	if (ret == 0)
		device_init_wakeup(&pdev->dev, 1);
	return ret;
}

static void tx51_esdhci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int gpio;
	int irq;

	BUG_ON(pdev->id < 0 || pdev->id >= ARRAY_SIZE(tx51_sdhci_cd_pads));

	mx51_iomux_release_pads(tx51_sdhci_pads[pdev->id],
				ARRAY_SIZE(tx51_sdhci_pads[pdev->id]));

	gpio = IOMUX_TO_GPIO(tx51_sdhci_cd_pads[pdev->id].pad);
	irq = gpio_to_irq(gpio);

	DBG(0, "%s: Freeing IRQ %d (GPIO%d) for SDHCI # %d\n", __FUNCTION__,
		irq, gpio, pdev->id);
	free_irq(irq, data);
	gpio_free(gpio);
}

#ifdef CONFIG_PM
static int tx51_esdhci_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int irq;

	BUG_ON(pdev->id < 0 || pdev->id >= ARRAY_SIZE(tx51_sdhci_cd_pads));

	if (device_may_wakeup(dev)) {
		irq = IOMUX_TO_IRQ(tx51_sdhci_cd_pads[pdev->id].pad);
		DBG(0, "%s: Enabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return enable_irq_wake(irq);
	}
	return 0;
}

static int tx51_esdhci_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int irq;

	BUG_ON(pdev->id < 0 || pdev->id >= ARRAY_SIZE(tx51_sdhci_cd_pads));

	if (device_may_wakeup(dev)) {
		irq = IOMUX_TO_IRQ(tx51_sdhci_cd_pads[pdev->id].pad);
		DBG(0, "%s: Disabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return disable_irq_wake(irq);
	}
	return 0;
}
#else
#define tx51_esdhci_suspend	NULL
#define tx51_esdhci_resume	NULL
#endif

static int tx51_esdhci_status(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int gpio;
	int status;

	BUG_ON(pdev->id < 0 || pdev->id >= ARRAY_SIZE(tx51_sdhci_cd_pads));

	gpio = IOMUX_TO_GPIO(tx51_sdhci_cd_pads[pdev->id].pad);
	status = gpio_get_value(gpio);

	DBG(0, "%s: SDHCI # %d CD=%d\n", __FUNCTION__, pdev->id, status);

	return status;
}

static struct mxc_sdhci_platform_data tx51_esdhci_pdata = {
	.get_ro = NULL,
	.init = tx51_esdhci_init,
	.exit = tx51_esdhci_exit,
	.suspend = tx51_esdhci_suspend,
	.resume = tx51_esdhci_resume,
	.setpower = NULL,
	.status = tx51_esdhci_status,
	.detect_delay = 200,
	.ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34,
	.min_clk = 150000,
	.max_clk = 25000000,
};
#endif

#if defined(CONFIG_ARCH_MXC_EHCI_USBH1) || defined(CONFIG_ARCH_MXC_EHCI_USBOTG)

/* USB register offsets */
#define REG_USBCTRL		0x800
#define REG_OTGMIRROR		0x804
#define REG_PHY_CTRL0		0x808
#define REG_PHY_CTRL1		0x80c
#define REG_USBCTRL1		0x810
#define REG_USBH2CTRL		0x814

/* USB Host/OTG register offsets */
#define REG_USBCMD		0x140
#define REG_USBSTS		0x144
#define REG_PORTSC1		0x184
#define REG_USBMODE		0x1a8

#define USBCMD_RST		(1 << 1)
#define USBCMD_RUN		(1 << 0)

#define USBSTS_HCH		(1 << 12)

/* USB_CTRL register bits */
#define USBCTRL_H1DISFSTLL	(1 << 4)
#define USBCTRL_H1HSTLL		(1 << 6)
#define USBCTRL_H1PM		(1 << 8)
#define USBCTRL_H1BPVAL_MASK	(3 << 9)
#define USBCTRL_H1BPVAL(n)	(((n) << 9) & USBCTRL_H1BPVAL_MASK)
#define USBCTRL_H1WIE		(1 << 11)
#define USBCTRL_H1UIE		(1 << 12)
#define USBCTRL_H1SIC_MASK	(3 << 13)
#define USBCTRL_H1SIC(n)	(((n) << 13) & USBCTRL_H1SIC_MASK)
#define USBCTRL_H1WIR		(1 << 15)
#define USBCTRL_H1TCKOEN	(1 << 17)
#define USBCTRL_UPBCKE		(1 << 18)
#define USBCTRL_HWIE		(1 << 19)
#define USBCTRL_OHSTLL		(1 << 7)
#define USBCTRL_OTCKOEN		(1 << 1)
#define USBCTRL_OPM		(1 << 24)
#define USBCTRL_OWIE		(1 << 27)
#define USBCTRL_OUIE		(1 << 28)
#define USBCTRL_OSIC_MASK	(3 << 29)
#define USBCTRL_OSIC_MASK	(3 << 29)
#define USBCTRL_OWIR		(1 << 31)

#ifdef DEBUG
#define usb_reg_write(v,b,r)	_usb_reg_write(v,b,r,#r)
static inline void _usb_reg_write(unsigned long val, void __iomem *base, int reg,
				const char *name)
{
	if (!dbg_lvl(1) || val != __raw_readl(base + reg)) {
		DBG(0, "%s: Writing %08lx to %s[%03x]@%p\n",
			__FUNCTION__, val, name, reg, base + reg);
		__raw_writel(val, base + reg);
	} else {
		DBG(0, "%s: %s[%03x]@%p is already %08lx\n",
			__FUNCTION__, name, reg, base + reg, val);
	}
}

#define usb_reg_read(b,r)	_usb_reg_read(b,r,#r)
static inline unsigned long _usb_reg_read(void __iomem *base, int reg, const char *name)
{
	unsigned long val;

	val = __raw_readl(base + reg);
	DBG(0, "%s: Read %08lx from %s[%03x]@%p\n",
		__FUNCTION__, val, name, reg, base + reg);
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

static struct clk *usboh3_clk;

static int stk5_usb_host_init(struct platform_device *pdev, void __iomem *base)
{
	unsigned long val;
	unsigned long flags;
	const char __maybe_unused *name = pdev->id ? "USBH1" : "USBOTG";
	unsigned int loops = 0;
	void __iomem *otg_base = MX5_IO_ADDRESS(OTG_BASE_ADDR);

	if (usboh3_clk == NULL) {
		usboh3_clk = clk_get(NULL, "usboh3");
		if (IS_ERR(usboh3_clk)) {
			int ret = PTR_ERR(usboh3_clk);
			usboh3_clk = NULL;
			return ret;
		}
	}
	clk_enable(usboh3_clk);

	if (!(usb_reg_read(base, REG_USBSTS) & USBSTS_HCH)) {
		unsigned int loops = 0;

		val = usb_reg_read(base, REG_USBCMD);
		if (val & USBCMD_RUN) {
			DBG(0, "%s: %s[%p] is busy: %08lx\n", __FUNCTION__,
				name, base + REG_USBSTS,
				usb_reg_read(base, REG_USBSTS));
			usb_reg_write(val & ~USBCMD_RUN, base, REG_USBCMD);
			while (!(usb_reg_read(base, REG_USBSTS) & USBSTS_HCH)) {
				udelay(100);
				loops++;
				if (loops > 100)
					break;
			}
			if (usb_reg_read(base, REG_USBSTS) & USBSTS_HCH) {
				DBG(0, "USB controller idle after %u loops\n",
					loops);
			} else {
				DBG(0, "USB controller NOT idle after %u loops\n",
					loops);
			}
		} else {
			DBG(0, "%s: ignoring bogus %s_HCH\n", __FUNCTION__,
				name);
		}
	}

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
		val, val | 0x3);
	usb_reg_write(val | 0x3, base, REG_USBMODE);

	local_irq_save(flags);
	/* configure for sysclk == 24MHz */
	val = usb_reg_read(otg_base, REG_PHY_CTRL1);
	val = (val & ~(3 << 0)) | (1 << 0);
	DBG(0, "%s: Changing USB_PHY_CTRL1 from %08lx to %08lx\n", __FUNCTION__,
		usb_reg_read(otg_base, REG_PHY_CTRL1), val);
	usb_reg_write(val, otg_base, REG_PHY_CTRL1);

	val = usb_reg_read(otg_base, REG_USBCTRL);
	if (pdev->id == 1) {
		val &= ~USBCTRL_H1PM;
		val |= USBCTRL_H1UIE | USBCTRL_H1WIE | USBCTRL_H1DISFSTLL;
	} else {
		val &= ~USBCTRL_OWIE;
		val |= USBCTRL_OPM;
	}
	DBG(0, "%s: Changing %s_USBCTRL from %08lx to %08lx\n", __FUNCTION__, name,
		usb_reg_read(otg_base, REG_USBCTRL), val);
	usb_reg_write(val, otg_base, REG_USBCTRL);

	if (pdev->id == 1) {
		val = usb_reg_read(otg_base, REG_USBCTRL1);
		val |= 1 << 25; /* enable ULPI clock from PHY */
		DBG(0, "%s: Changing %s_USBCTRL1 from %08lx to %08lx\n", __FUNCTION__, name,
			usb_reg_read(otg_base, REG_USBCTRL1), val);
		usb_reg_write(val, otg_base, REG_USBCTRL1);

		val = usb_reg_read(otg_base, REG_USBH2CTRL);
		val &= ~(1 << 19); /* disable H1 serial driver */
		DBG(0, "%s: Changing %s_USBH2CTRL from %08lx to %08lx\n", __FUNCTION__, name,
			usb_reg_read(otg_base, REG_USBH2CTRL), val);
		usb_reg_write(val, otg_base, REG_USBH2CTRL);
	} else {
		val = usb_reg_read(otg_base, REG_USBH2CTRL);
		val &= ~(1 << 20); /* disable OTG serial driver */
		DBG(0, "%s: Changing %s_USBH2CTRL from %08lx to %08lx\n", __FUNCTION__, name,
			usb_reg_read(otg_base, REG_USBH2CTRL), val);
		usb_reg_write(val, otg_base, REG_USBH2CTRL);
	}
	local_irq_restore(flags);

	if (pdev->id == 1) {
		val = usb_reg_read(base, REG_PORTSC1);
		/* select ULPI transceiver for USBH1 port */
		val = (val & ~(3 << 30)) | (2 << 30) | (0 << 29);
		DBG(0, "%s: Changing %s_PORTSC1 from %08lx to %08lx\n", __FUNCTION__, name,
			usb_reg_read(base, REG_PORTSC1), val);
		usb_reg_write(val, base, REG_PORTSC1);
	} else {
		val = usb_reg_read(base, REG_PORTSC1);
		/* select UTMI transceiver for USBOTG port */
		val = (val & ~(3 << 30)) | (0 << 30);
		DBG(0, "%s: Changing %s_PORTSC1 from %08lx to %08lx\n", __FUNCTION__, name,
			usb_reg_read(base, REG_PORTSC1), val);
		usb_reg_write(val, base, REG_PORTSC1);
	}

	return 0;
}

#ifdef CONFIG_ARCH_MXC_EHCI_USBH1

/* register the 26MHz clock for the USB PHY */
#define OSC26M_ENABLE_PIN	IOMUX_TO_GPIO(MX51_PIN_GPIO1_7)
#define USBH1_PHY_RESET_PIN	IOMUX_TO_GPIO(MX51_PIN_GPIO1_4)
#define USBH1_OC_PIN		IOMUX_TO_GPIO(MX51_PIN_GPIO1_6)

static int _clk_26m_enable(struct clk *clk)
{
	DBG(0, "%s: Switching 26MHz oscillator on\n", __FUNCTION__);
	gpio_set_value(OSC26M_ENABLE_PIN, 1);
	return 0;
}

static void _clk_26m_disable(struct clk *clk)
{
	DBG(0, "%s: Switching 26MHz oscillator off\n", __FUNCTION__);
	gpio_set_value(OSC26M_ENABLE_PIN, 0);
}

static struct clk clk_26m = {
	.enable = _clk_26m_enable,
	.disable = _clk_26m_disable,
};

static struct clk_lookup osc26m_clk = {
	.dev_id = NULL,
	.con_id = "osc26m",
	.clk = &clk_26m,
};

static int stk5_clk26m_register(void)
{
	DBG(0, "%s: Registering 26MHz clock\n", __FUNCTION__);
	clkdev_add(&osc26m_clk);
	return 0;
}
arch_initcall(stk5_clk26m_register);

/*
 * The USB power switch (MAX893L) used on the STK5 base board
 * produces a pulse (~100us) on the OC output whenever
 * the ON input is activated. This disturbs the USB controller.
 * As a workaround don't use USB power switching.
 * If you have a hardware that works cleanly you may
 * #define USE_USB_PWR to enable port power control for
 * the EHCI controller.
 */
static struct mx51_pad_desc karo_stk5_usbh1_pads[] = {
	MX51_PIN_USBH1_CLK__USBH1_CLK,
	MX51_PIN_USBH1_DIR__USBH1_DIR,
	MX51_PIN_USBH1_STP__USBH1_STP,
	MX51_PIN_USBH1_NXT__USBH1_NXT,
	MX51_PIN_USBH1_DATA0__USBH1_DATA0,
	MX51_PIN_USBH1_DATA1__USBH1_DATA1,
	MX51_PIN_USBH1_DATA2__USBH1_DATA2,
	MX51_PIN_USBH1_DATA3__USBH1_DATA3,
	MX51_PIN_USBH1_DATA4__USBH1_DATA4,
	MX51_PIN_USBH1_DATA5__USBH1_DATA5,
	MX51_PIN_USBH1_DATA6__USBH1_DATA6,
	MX51_PIN_USBH1_DATA7__USBH1_DATA7,
};

static struct clk *usbh1_clk;

static int stk5_usbh1_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = MX5_IO_ADDRESS(OTG_BASE_ADDR + 0x200);

	DBG(0, "%s: \n", __FUNCTION__);
	usbh1_clk = clk_get(NULL, "osc26m");
	if (IS_ERR(usbh1_clk)) {
		ret = PTR_ERR(usbh1_clk);
		printk(KERN_ERR "Cannot request 26MHz clock: %d\n", ret);
		usbh1_clk = NULL;
		return ret;
	}

	clk_enable(usbh1_clk);

	DBG(0, "%s: configuring USBH1 pads\n", __FUNCTION__);
	ret = mx51_iomux_request_pads(karo_stk5_usbh1_pads,
				ARRAY_SIZE(karo_stk5_usbh1_pads));
	if (ret) {
		goto exit;
	}
	udelay(1);

	/* release USB PHY reset */
	gpio_set_value(USBH1_PHY_RESET_PIN, 1);

	ret = stk5_usb_host_init(pdev, base);
	if (ret)
		goto release;

	return 0;

release:
	mx51_iomux_release_pads(karo_stk5_usbh1_pads,
				ARRAY_SIZE(karo_stk5_usbh1_pads));
exit:
	clk_put(usbh1_clk);
	usbh1_clk = NULL;
	return ret;
}

static int stk5_usbh1_exit(struct platform_device *pdev)
{
	if (usbh1_clk != NULL) {
		clk_disable(usbh1_clk);
		clk_put(usbh1_clk);
		usbh1_clk = NULL;
	}

	if (usboh3_clk != NULL) {
		clk_disable(usboh3_clk);
	}

	/* assert ULPI PHY reset */
	gpio_set_value(USBH1_PHY_RESET_PIN, 0);

	mx51_iomux_release_pads(karo_stk5_usbh1_pads,
				ARRAY_SIZE(karo_stk5_usbh1_pads));
	return 0;
}

static struct mxc_usbh_platform_data stk5_usbh1_data = {
	.init = stk5_usbh1_init,
	.exit = stk5_usbh1_exit,
};
#endif // CONFIG_ARCH_MXC_EHCI_USBH1

#ifdef CONFIG_ARCH_MXC_EHCI_USBOTG
static struct mx51_pad_desc karo_stk5_usbotg_pads[] = {
#ifdef USE_USB_PWR
	{ MX51_PIN_GPIO1_8, IOMUX_CONFIG_ALT1, },
#else
	{ MX51_PIN_GPIO1_8, IOMUX_CONFIG_ALT0, },
#endif
	{ MX51_PIN_GPIO1_9, IOMUX_CONFIG_ALT1, },
};

static struct clk *usb_phy_clk;

static int stk5_usbotg_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = MX5_IO_ADDRESS(OTG_BASE_ADDR + 0x000);
#ifndef USE_USB_PWR
	const int pwr_gpio = IOMUX_TO_GPIO(karo_stk5_usbotg_pads[0].pad);
#endif
	if (usb_phy_clk == NULL) {
		usb_phy_clk = clk_get(NULL, "usb_phy");
		if (IS_ERR(usb_phy_clk)) {
			ret = PTR_ERR(usb_phy_clk);
			usb_phy_clk = NULL;
			return ret;
		}
	}
	clk_enable(usb_phy_clk);

	DBG(0, "%s: configuring USBOTG pads\n", __FUNCTION__);
	ret = mx51_iomux_request_pads(karo_stk5_usbotg_pads,
				ARRAY_SIZE(karo_stk5_usbotg_pads));
	if (ret) {
		return ret;
	}
#ifndef USE_USB_PWR
	DBG(0, "%s: Switching USB power on\n", __FUNCTION__);
	gpio_direction_output(pwr_gpio, 1);
#endif
	if (ret != 0) {
		mx51_iomux_release_pads(karo_stk5_usbotg_pads,
					ARRAY_SIZE(karo_stk5_usbotg_pads));
		goto exit;
	}
	ret = stk5_usb_host_init(pdev, base);

exit:
	return ret;
}

static int stk5_usbotg_exit(struct platform_device *pdev)
{
	if (usb_phy_clk != NULL) {
		clk_disable(usb_phy_clk);
	}
	if (usboh3_clk != NULL) {
		clk_disable(usboh3_clk);
	}

	mx51_iomux_release_pads(karo_stk5_usbotg_pads,
				ARRAY_SIZE(karo_stk5_usbotg_pads));
	return 0;
}

static struct mxc_usbh_platform_data stk5_usbotg_data = {
	.init = stk5_usbotg_init,
	.exit = stk5_usbotg_exit,
};
#endif // CONFIG_ARCH_MXC_EHCI_USBOTG
#endif // CONFIG_ARCH_MXC_EHCI_USBH1 || CONFIG_ARCH_MXC_EHCI_USBOTG

#if defined(CONFIG_USB_FSL_USB2) || defined(CONFIG_USB_FSL_USB2_MODULE)
static struct fsl_usb2_platform_data tx51_usb_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI,
};
#endif

#if defined(CONFIG_AC97_BUS) || defined(CONFIG_AC97_BUS_MODULE)
static u64 stk5_dma_mask = DMA_BIT_MASK(32);

static struct mx51_pad_desc stk5_ac97_pads_on[] = {
	{ MX51_PIN_EIM_A21, IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION * 1, },	/* AC97 reset */
	{ MX51_PIN_DI1_D0_CS, IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION * 1, },	/* Codec IRQ (GPIO3[3]) */

	{ MX51_PIN_AUD3_BB_FS, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
	{ MX51_PIN_AUD3_BB_TXD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
	{ MX51_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
	{ MX51_PIN_AUD3_BB_CK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
};

static struct mx51_pad_desc stk5_ac97_pads_off[] = {
	{ MX51_PIN_EIM_A21, IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION, },	/* AC97 reset */
	{ MX51_PIN_DI1_D0_CS, IOMUX_CONFIG_ALT4 | IOMUX_CONFIG_SION, },	/* Codec IRQ (GPIO3[3]) */

	{ MX51_PIN_AUD3_BB_FS, IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION, },
	{ MX51_PIN_AUD3_BB_TXD, IOMUX_CONFIG_ALT3 | IOMUX_CONFIG_SION, },
	{ MX51_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION, },
	{ MX51_PIN_AUD3_BB_CK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION, },
};

static int stk5_ac97_gpios[] = {
	/* configure the UCB1400 strap pins to the correct values */
	IOMUX_TO_GPIO(MX51_PIN_EIM_A21),
	IOMUX_TO_GPIO(MX51_PIN_AUD3_BB_FS),
	IOMUX_TO_GPIO(MX51_PIN_AUD3_BB_TXD),
};

static int stk5_ac97_init(struct platform_device *dev, int ssi)
{
	int ret;
	const int irq = IOMUX_TO_IRQ(stk5_ac97_pads_on[1].pad);
	int i;

	/* AC97 is hardwired to AUDMUX port 3 */
	ret = mxc_audmux_configure_sync_slave(ssi + 1, 3, MXC_AUDMUX_MODE_AC97);
	if (ret) {
		DBG(0, "%s: Failed to configure AUDMUX: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
	printk(KERN_INFO "%s: AC97 IRQ: %d\n", __FUNCTION__, irq);

	/*
	 * IRQs are disabled for probing whenever enable_irq() is called.
	 * Thus reenable the UCB1400 IRQ for probing in case the ucb1400-ts
	 * module has been reloaded.
	 */
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	/* configure RESET_OUT, AUD4_TXFS, AUD4_TXD as GPIO
	 * to prevent AC97 codecs from entering test mode
	 * due to TXFS or TXD sampled HIGH when RESET is deasserted
	 */
	ret = mx51_iomux_request_pads(stk5_ac97_pads_off,
				ARRAY_SIZE(stk5_ac97_pads_off));
	if (ret) {
		DBG(0, "%s: Failed to configure AC97 pads: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
	mx51_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_3_SELECT_INPUT, 1);
	for (i = 0; i < ARRAY_SIZE(stk5_ac97_gpios); i++) {
		int gpio = stk5_ac97_gpios[i];

		DBG(0, "%s: Switching GPIO%d_%d to 0\n",
			__FUNCTION__, gpio / 32 + 1, gpio % 32);
		gpio_direction_output(gpio, 0);
	}
	udelay(1);
	/* Deassert UCB1400 RESET */
	gpio_set_value(stk5_ac97_gpios[0], 1);
	gpio_direction_input(stk5_ac97_gpios[i - 1]);

	mx51_iomux_release_pads(stk5_ac97_pads_off,
				ARRAY_SIZE(stk5_ac97_pads_off));
	ret = mx51_iomux_request_pads(stk5_ac97_pads_on,
				ARRAY_SIZE(stk5_ac97_pads_on));
	if (ret != 0) {
		DBG(0, "%s: Failed to reconfigure AC97 pads: %d\n",
			__FUNCTION__, ret);
	}
	return ret;
}

static void stk5_ac97_exit(struct platform_device *dev)
{
	DBG(0, "%s: Releasing AC97 GPIO pins\n", __FUNCTION__);

	mx51_iomux_release_pads(stk5_ac97_pads_on,
				ARRAY_SIZE(stk5_ac97_pads_on));
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

#if defined(CONFIG_FB_MXC_IPU_V3) || defined(CONFIG_FB_MXC_IPU_V3_MODULE)
#define STK5_LCD_BACKLIGHT_GPIO		IOMUX_TO_GPIO(MX51_PIN_GPIO1_2)
#define STK5_LCD_RESET_GPIO		IOMUX_TO_GPIO(MX51_PIN_CSI2_VSYNC)
#define STK5_LCD_POWER_GPIO		IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC)

/*
 * Setup GPIO for LCDC device to be active
 *
 */
static struct mx51_pad_desc tx51_lcdc_gpios[] = {
#ifdef STK5_LCD_BACKLIGHT_GPIO
	MX51_PIN_CSI2_VSYNC__GPIO4_13,	/* LCD Reset (active LOW) */
	MX51_PIN_CSI2_HSYNC__GPIO4_14,	/* LCD Power Enable (active High) */
#if !defined(CONFIG_MXC_PWM) && !defined(CONFIG_MXC_PWM_MODULE)
	MX51_PIN_GPIO1_2__GPIO1_2,	/* LCD Backlight brightness 0: full 1: off */
#endif
#endif
	/* DISP1_DAT0..5 don't have any other function */
	MX51_PIN_DISP1_DAT6__DISP1_DAT6,
	MX51_PIN_DISP1_DAT7__DISP1_DAT7,
	MX51_PIN_DISP1_DAT8__DISP1_DAT8,
	MX51_PIN_DISP1_DAT9__DISP1_DAT9,
	MX51_PIN_DISP1_DAT10__DISP1_DAT10,
	MX51_PIN_DISP1_DAT11__DISP1_DAT11,
	MX51_PIN_DISP1_DAT12__DISP1_DAT12,
	MX51_PIN_DISP1_DAT13__DISP1_DAT13,
	MX51_PIN_DISP1_DAT14__DISP1_DAT14,
	MX51_PIN_DISP1_DAT15__DISP1_DAT15,
	MX51_PIN_DISP1_DAT16__DISP1_DAT16,
	MX51_PIN_DISP1_DAT17__DISP1_DAT17,
	MX51_PIN_DISP1_DAT18__DISP1_DAT18,
	MX51_PIN_DISP1_DAT19__DISP1_DAT19,
	MX51_PIN_DISP1_DAT20__DISP1_DAT20,
	MX51_PIN_DISP1_DAT21__DISP1_DAT21,
	MX51_PIN_DISP1_DAT22__DISP1_DAT22,
	MX51_PIN_DISP1_DAT23__DISP1_DAT23,
	MX51_PIN_DI1_PIN2__HSYNC,
	MX51_PIN_DI1_PIN3__VSYNC,
};

static int stk5_gpio_lcdc_active(struct platform_device *dev)
{
	int ret;

	DBG(0, "%s: Setting up GPIO pins for LCD\n", __FUNCTION__);

	ret = mx51_iomux_request_pads(tx51_lcdc_gpios,
				ARRAY_SIZE(tx51_lcdc_gpios));
	if (ret) {
		DBG(0, "%s: Failed to setup GPIO pins for LCD: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
#ifdef STK5_LCD_BACKLIGHT_GPIO
	gpio_direction_output(STK5_LCD_POWER_GPIO, 1);
#if !defined(CONFIG_MXC_PWM) && !defined(CONFIG_MXC_PWM_MODULE)
	gpio_direction_output(STK5_LCD_BACKLIGHT_GPIO, 1);
#endif
	gpio_direction_output(STK5_LCD_RESET_GPIO, 0);
#endif
	return ret;
}

#ifdef STK5_LCD_BACKLIGHT_GPIO
#if !defined(CONFIG_BACKLIGHT_PWM) && !defined(CONFIG_BACKLIGHT_PWM_MODULE)
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

#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
static struct mx51_pad_desc stk5_pwm_pads[] = {
	MX51_PIN_GPIO1_2__PWM1,	/* LCD Backlight brightness 0: full 1: off */
};

static int stk5_backlight_init(struct device *dev)
{
	int ret;
	ret = mx51_iomux_request_pads(&stk5_pwm_pads[0], 1);
	return ret;
}

#define PWM_MAX_BRIGHTNESS 100

static int stk5_backlight_notify(int brightness)
{
	DBG(0, "%s: brightness=%d->%d\n", __FUNCTION__, brightness,
		PWM_MAX_BRIGHTNESS - brightness);
	return PWM_MAX_BRIGHTNESS - brightness;
}

static void stk5_backlight_exit(struct device *dev)
{
	mx51_iomux_release_pads(&stk5_pwm_pads[0], 1);
}

static struct platform_pwm_backlight_data stk5_backlight_data = {
	.pwm_id = 0,
	.max_brightness = PWM_MAX_BRIGHTNESS,
	.dft_brightness = 50,
	.pwm_period_ns = KHZ2PICOS(100), /* kHz -> ps is the same as Hz -> ns */
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
#endif

static int __init stk5_lcdc_init(void)
{
	int ret;

	ret = stk5_gpio_lcdc_active(NULL);
	if (ret)
		return ret;

#ifdef STK5_LCD_BACKLIGHT_GPIO
#if !defined(CONFIG_BACKLIGHT_PWM) && !defined(CONFIG_BACKLIGHT_PWM_MODULE)
	stk5_lcdc_backlight(1);
#endif
	stk5_lcdc_power(1);
#endif
	return 0;
}
arch_initcall(stk5_lcdc_init);

static struct fb_videomode stk5_fb_mode = {
	.name = "G-ETV570G0DMU",
	.pixclock	= 39722,

	.xres		= 640,
	.yres		= 480,

	.hsync_len	= 96,
	.left_margin	= 48,
	.right_margin	= 36,

	.vsync_len	= 2,
	.upper_margin	= 33,
	.lower_margin	= 11,
};

static struct mxc_fb_platform_data stk5_fb_data = {
	.mode		= &stk5_fb_mode,
/*
 * Display interface output format
 * valid formats:
 * IPU_PIX_FMT_GENERIC
 * IPU_PIX_FMT_RGB24
 * IPU_PIX_FMT_RGB666
 * IPU_PIX_FMT_YUV444
 * IPU_PIX_FMT_RGB565
 * IPU_PIX_FMT_LVDS666
 */
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
#define TSC2007_PEN_GPIO		IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS)

static struct mx51_pad_desc stk5_tsc2007_pads[] = {
	MX51_PIN_DI1_D0_CS__GPIO3_3,
};

static int stk5_tsc2007_init(void)
{
	int ret;

	ret = mx51_iomux_request_pads(stk5_tsc2007_pads,
				ARRAY_SIZE(stk5_tsc2007_pads));
	if (ret)
		return ret;
	mx51_iomux_set_input(MUX_IN_GPIO3_IPP_IND_G_IN_3_SELECT_INPUT, 1);
	gpio_direction_input(TSC2007_PEN_GPIO);
	return ret;
}

static void stk5_tsc2007_exit(void)
{
	mx51_iomux_release_pads(stk5_tsc2007_pads,
				ARRAY_SIZE(stk5_tsc2007_pads));
}

static int stk5_get_pendown(void)
{
	return !gpio_get_value(TSC2007_PEN_GPIO);
}

static struct tsc2007_platform_data stk5_tsc2007_pdata = {
	.model = 2007,
	.x_plate_ohms = 660,
	.get_pendown_state = stk5_get_pendown,
	.init_platform_hw = stk5_tsc2007_init,
	.exit_platform_hw = stk5_tsc2007_exit,
	.poll_delay_ns = 50000000,
	.poll_period_ns = 1000000,
};
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
static struct mx51_pad_desc stk5_sgtl5000_pads[] = {
	{ MX51_PIN_AUD3_BB_FS, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
	{ MX51_PIN_AUD3_BB_TXD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
	{ MX51_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
	{ MX51_PIN_AUD3_BB_CK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION * 1, },
};

static int stk5_sgtl5000_plat_init(void)
{
	int ret;

	DBG(0, "%s: \n", __FUNCTION__);
	ret = mx51_iomux_request_pads(stk5_sgtl5000_pads,
				ARRAY_SIZE(stk5_sgtl5000_pads));
	return ret;
}

static void stk5_sgtl5000_plat_finit(void)
{
	DBG(0, "%s: \n", __FUNCTION__);
	mx51_iomux_release_pads(stk5_sgtl5000_pads,
				ARRAY_SIZE(stk5_sgtl5000_pads));
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 0,
	.src_port = 1,
	.ext_port = 3,
	//.hp_irq = IOMUX_TO_IRQ(MX51_PIN_EIM_A26),
	//.hp_status = headphone_det_status,
	//.amp_enable = mxc_sgtl5000_amp_enable,
	.sysclk = 26000000,
	.init = stk5_sgtl5000_plat_init,
	.finit = stk5_sgtl5000_plat_finit,
};

static struct platform_device stk5_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
	.dev = {
		.platform_data = &sgtl5000_data,
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
#endif /* CONFIG_SND_SOC_IMX_3STACK_SGTL5000 || CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE */

#ifdef CONFIG_I2C_IMX_SELECT2
static struct mx51_pad_desc stk5_i2c_gpio_pads[] = {
	MX51_PIN_EIM_D24__I2C2_SDA,
	MX51_PIN_EIM_D27__I2C2_SCL,
};
#else
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
static struct i2c_gpio_platform_data stk5_i2c_gpio_pdata = {
	.scl_pin = IOMUX_TO_GPIO(MX51_PIN_EIM_D27),
	.sda_pin = IOMUX_TO_GPIO(MX51_PIN_EIM_D24),
	.sda_is_open_drain = 0,
};

static struct platform_device stk5_i2c_gpio_device = {
	.name = "i2c-gpio",
	.id = 1,
	.dev = {
		.platform_data = &stk5_i2c_gpio_pdata,
	},
};

static struct mx51_pad_desc stk5_i2c_gpio_pads[] = {
	MX51_PIN_EIM_D24__GPIO2_8,
	MX51_PIN_EIM_D27__GPIO2_9,
};
#endif // CONFIG_I2C_GPIO || CONFIG_I2C_GPIO_MODULE
#endif // !CONFIG_I2C_IMX_SELECT2

#if defined(CONFIG_I2C_IMX_SELECT2) ||				\
	defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
static int __init stk5_i2c_gpio_init(void)
{
	int ret;

	ret = mx51_iomux_request_pads(stk5_i2c_gpio_pads,
				ARRAY_SIZE(stk5_i2c_gpio_pads));
	if (ret) {
		DBG(0, "%s: Failed to setup I2C pins: %d\n", __FUNCTION__,
			ret);
		return ret;
	}
	mx51_iomux_release_pads(stk5_i2c_gpio_pads,
				ARRAY_SIZE(stk5_i2c_gpio_pads));
	return ret;
}
arch_initcall(stk5_i2c_gpio_init);
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static struct i2c_board_info stk5_i2c_boardinfo[] __initdata = {
#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
	{
		I2C_BOARD_INFO("sgtl5000-i2c", 0x0a),
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_TSC2007) || defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq = gpio_to_irq(TSC2007_PEN_GPIO),
		.platform_data = &stk5_tsc2007_pdata,
	},
#endif
};
#endif

static struct {
	struct platform_device *pdev;
	void *pdata;
} stk5_devices[] = {
#if defined(CONFIG_MXC_IPU_V3) || defined(CONFIG_MXC_IPU_V3_MODULE)
	{ .pdev = &mx51_ipu_device, },
#endif
#if defined(CONFIG_FB_MXC_IPU_V3) || defined(CONFIG_FB_MXC_IPU_V3_MODULE)
	{ .pdev = &mx51_fb_device, .pdata = &stk5_fb_data, },
#endif
#if defined(CONFIG_MXC_PWM) || defined(CONFIG_MXC_PWM_MODULE)
	{ .pdev = &mx51_pwm1_device, },
#endif
#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
	{ .pdev = &stk5_backlight_pwm_device, .pdata = &stk5_backlight_data, },
#endif
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	{ .pdev = &stk5_led_device, },
#endif
#if defined(CONFIG_ARCH_MXC_EHCI_USBH1) || defined(CONFIG_ARCH_MXC_EHCI_USBH1_MODULE)
	{ .pdev = &mx51_usbh1_device, .pdata = &stk5_usbh1_data, },
#endif
#if defined(CONFIG_ARCH_MXC_EHCI_USBOTG) || defined(CONFIG_ARCH_MXC_EHCI_USBOTG_MODULE)
	{ .pdev = &mx51_usbotg_device, .pdata = &stk5_usbotg_data, },
#endif
#if defined(CONFIG_USB_FSL_USB2) || defined(CONFIG_USB_FSL_USB2_MODULE)
	{ .pdev = &mx51_otg_udc_device, .pdata = &tx51_usb_pdata, },
#endif
#if defined(CONFIG_I2C_IMX_SELECT2)
	{ .pdev = &mx51_i2c2_device, },
#endif
#ifndef CONFIG_I2C_IMX_SELECT2
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	{ .pdev = &stk5_i2c_gpio_device, },
#endif
#endif
#if defined(CONFIG_MMC_SDHCI_MXC) || defined(CONFIG_MMC_SDHCI_MXC_MODULE)
	{ .pdev = &mx51_esdhc1_device, .pdata = &tx51_esdhci_pdata, },
#endif
#if defined(CONFIG_MMC_SDHCI_MXC) || defined(CONFIG_MMC_SDHCI_MXC_MODULE)
	{ .pdev = &mx51_esdhc2_device, .pdata = &tx51_esdhci_pdata, },
#endif
#if defined(CONFIG_MXC_AUDMUX_V3) || defined(CONFIG_MXC_AUDMUX_V3_MODULE)
	{ .pdev = &mxc_audmux_v3_device, },
#endif
};

/*!
 * Board specific initialization.
 */
#ifdef CONFIG_MACH_TX51_CUSTOM
extern __init int stk5_custom_board_init(void);
#else
static inline int stk5_custom_board_init(void)
{
	return 0;
}
#endif

static int __init stk5_board_init(void)
{
	int i;
	int ret;

	DBG(0, "%s: \n", __FUNCTION__);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (stk5_board_rev >= 3) {
		ret = i2c_register_board_info(1, stk5_i2c_boardinfo,
					ARRAY_SIZE(stk5_i2c_boardinfo));
		if (ret != 0)
			printk(KERN_ERR "Failed to register I2C board info: %d\n", ret);
	}
#endif
	for (i = 0; i < ARRAY_SIZE(stk5_devices); i++) {
		DBG(0, "%s: Registering device[%d] %s\n", __FUNCTION__,
			i, stk5_devices[i].pdev->name);

		if (stk5_devices[i].pdata) {
			ret = mxc_register_device(stk5_devices[i].pdev,
						stk5_devices[i].pdata);
		} else {
			ret = platform_device_register(stk5_devices[i].pdev);
		}
		if (ret) {
			DBG(0, "%s: Failed to register device[%d] %s\n",
				__FUNCTION__, i,
				stk5_devices[i].pdev->name);
		}
	}
	DBG(0, "%s: Done\n", __FUNCTION__);
	return stk5_custom_board_init();
}
subsys_initcall(stk5_board_init);

static int __init stk5_board_rev_setup(char *cmdline)
{
	get_option(&cmdline, &stk5_board_rev);
	DBG(0, "%s: Board rev set to 0x%02x\n",
		__FUNCTION__, stk5_board_rev);
	return 1;
}
__setup("stk5_board_rev=", stk5_board_rev_setup);
