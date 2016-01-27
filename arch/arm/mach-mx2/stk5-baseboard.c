/*
 * arch/arm/mach-mx27/stk5-baseboard.c
 *
 * Copyright (C) 2010  Lothar Wassmann <LW@KARO-electronics.de>
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio_keys.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2007.h>
#include <linux/pwm_backlight.h>
#include <linux/input/matrix_keypad.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/hw_irq.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/imx-uart.h>
#include <mach/iomux.h>
#include <mach/mmc.h>
#include <mach/i2c.h>
#include <mach/ulpi.h>
#include <mach/mxc_ehci.h>
#include <mach/imxfb.h>
#include <mach/mxc-ac97.h>
#include <mach/mxc_audmux.h>
#include <mach/mxc_audio.h>

#include "devices.h"
#include "karo.h"

static int stk5_board_rev;

#if defined(CONFIG_ARCH_MXC_EHCI_USBH2) || defined(CONFIG_ARCH_MXC_EHCI_USBOTG)

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

static int stk5_usb_reset(void __iomem *base)
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

static int stk5_usb_init(struct platform_device *pdev, void __iomem *base)
{
	int ret;
	u32 temp;
	unsigned long flags;
	static void __iomem *otg_base = MX2_IO_ADDRESS(OTG_BASE_ADDR);
#ifdef DEBUG
	const char *name = (((unsigned long)base) & 0x400) == 0 ? "USBOTG" : "USBH2";
#endif

	if (!(__raw_readl(base + REG_USBSTS) & USBSTS_HCH)) {
		temp = __raw_readl(base + REG_USBCMD);
		if (temp & USBCMD_RUN) {
			temp &= ~USBCMD_RUN;
			DBG(0, "%s: Stopping %s controller port\n", __FUNCTION__, name);
			__raw_writel(temp, base + REG_USBCMD);
		}
	}
	WARN_ON(!(__raw_readl(base + REG_USBSTS) & USBSTS_HCH) &&
		(__raw_readl(base + REG_USBCMD) & USBCMD_RUN));

	if ((__raw_readl(base + REG_USBMODE) & USBMODE_CM_MASK) !=
		USBMODE_CM_HOST) {
		ret = stk5_usb_reset(base);
		if (ret != 0) {
			DBG(0, "%s: %s reset timed out\n", __FUNCTION__, name);
			return ret;
		}
		/* Set to Host mode */
		temp = __raw_readl(base + REG_USBMODE);
		temp |= 3;
		DBG(0, "%s: Enabling %s host mode: %08x to %08x\n", __FUNCTION__,
			name, __raw_readl(base + REG_USBMODE), temp);
		__raw_writel(temp, base + REG_USBMODE);
	}
	DBG(1, "%s: %s_STS =%08x\n", __FUNCTION__, name, __raw_readl(base + REG_USBSTS));
	DBG(1, "%s: %s_CMD =%08x\n", __FUNCTION__, name, __raw_readl(base + REG_USBCMD));
	DBG(1, "%s: %s_MODE=%08x\n", __FUNCTION__, name, __raw_readl(base + REG_USBMODE));

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
	DBG(1, "%s: Changing USBCTRL from %08x to %08x\n", __FUNCTION__,
		__raw_readl(otg_base + REG_USBCTRL), temp);
	__raw_writel(temp, otg_base + REG_USBCTRL);
	local_irq_restore(flags);

	/* select ULPI transceiver */
	temp = __raw_readl(base + REG_PORTSC1);
	temp &= ~(3 << 30);
	temp |= 2 << 30;
	__raw_writel(temp, base + REG_PORTSC1);
	DBG(1, "%s: Changing %s_PORTSC1 from %08x to %08x\n", __FUNCTION__,
		name, __raw_readl(base + REG_PORTSC1), temp);

	ret = stk5_usb_reset(base);
	if (ret != 0) {
		return ret;
	}
	msleep(100);
	return ret;
}
#endif // CONFIG_ARCH_MXC_EHCI_USBH2 || CONFIG_ARCH_MXC_EHCI_USBOTG

#ifdef CONFIG_ARCH_MXC_EHCI_USBH2
/*
 Setup GPIO for USB

 PB22: 26MHz oscillator enable

   PIN Configuration for USBOTG:   High/Full speed OTG
	PE2,PE1,PE0,PE24,PE25 -- PRIMARY
	PC7 - PC13  -- PRIMARY
	PB23,PB24 -- PRIMARY

   PIN Configuration for USBH2:    : High/Full/Low speed host
	PA0 - PA4 -- PRIMARY
	PD19, PD20,PD21,PD22,PD23,PD24,PD26 --Alternate (SECONDARY)

   USBH1:  Full/low speed host
   not supported on TX27
 */

static unsigned int mx27_usbh2_gpios_active[] = {
	PA2_PF_USBH2_DATA7,
	PD21_AF_USBH2_DATA6,
	PD26_AF_USBH2_DATA5,
	PD19_AF_USBH2_DATA4,
	PD20_AF_USBH2_DATA3,
	PD23_AF_USBH2_DATA2,
	PD24_AF_USBH2_DATA1,
	PD22_AF_USBH2_DATA0,

	PA0_PF_USBH2_CLK,
	PA1_PF_USBH2_DIR,
	PA3_PF_USBH2_NXT,
	PA4_PF_USBH2_STP,

	MXC_PIN(B, 31, AOUT, GPIO_IN),	/* OC detect */
};

static unsigned int mx27_usbh2_gpios_inactive[] = {
	MXC_PIN(A, 4, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(A, 0, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(A, 1, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(A, 3, AOUT, GPIO_IN | GPIO_PUEN),

	MXC_PIN(A, 2, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(D, 21, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(D, 26, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(D, 19, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(D, 20, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(D, 23, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(D, 24, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(D, 22, AOUT, GPIO_IN | GPIO_PUEN),

	MXC_PIN(B, 31, AOUT, GPIO_IN),	/* OC detect */
};

static struct clk *usbh2_clk;

static int gpio_usbh2_active(void)
{
	int ret;

	DBG(0, "%s: \n", __FUNCTION__);
	usbh2_clk = clk_get(NULL, "clk_26m");
	if (IS_ERR(usbh2_clk)) {
		ret = PTR_ERR(usbh2_clk);
		printk(KERN_ERR "Cannot request 26MHz clock: %d\n", ret);
		usbh2_clk = NULL;
		return ret;
	} else {
		clk_enable(usbh2_clk);
	}

	ret = mxc_gpio_setup_multiple_pins(mx27_usbh2_gpios_active,
					ARRAY_SIZE(mx27_usbh2_gpios_active),
					"USB");
	return ret;
}

static void gpio_usbh2_inactive(void)
{
	int i;

	DBG(0, "%s: \n", __FUNCTION__);
	if (usbh2_clk != NULL) {
		clk_disable(usbh2_clk);
		clk_put(usbh2_clk);
		usbh2_clk = NULL;
	}

	for (i = ARRAY_SIZE(mx27_usbh2_gpios_inactive) - 1; i >= 0; i--) {
		mxc_gpio_mode(mx27_usbh2_gpios_inactive[i]);
	}

	mxc_gpio_release_multiple_pins(mx27_usbh2_gpios_inactive,
				ARRAY_SIZE(mx27_usbh2_gpios_inactive));
}

static int stk5_usbh2_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = MX2_IO_ADDRESS(OTG_BASE_ADDR + 0x400);
	struct clk *usbclk = clk_get(&pdev->dev, "usb");

	if (IS_ERR(usbclk)) {
		printk(KERN_ERR "%s: Failed to get usb_clk: %ld\n", __FUNCTION__, PTR_ERR(usbclk));
		return PTR_ERR(usbclk);
	}

	ret = gpio_usbh2_active();
	if (ret != 0) {
		clk_put(usbclk);
		return ret;
	}

	ret = stk5_usb_init(pdev, base);
	if (ret != 0) {
		DBG(0, "%s: stk5_usb_init failed: %d\n", __FUNCTION__, ret);
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

static int stk5_usbh2_exit(struct platform_device *pdev)
{
	struct clk *usbclk = clk_get(&pdev->dev, "usb");

	if (!IS_ERR(usbclk)) {
		/* reenable USB clock, since the driver will disable it upon removal */
		clk_enable(usbclk);
		clk_put(usbclk);
	}

	gpio_usbh2_inactive();
	return 0;
}

static struct mxc_usbh_platform_data stk5_usbh2_data = {
	.init = stk5_usbh2_init,
	.exit = stk5_usbh2_exit,
};

int stk5_usbh2_register(void)
{
	int ret;

	ret = mxc_register_device(&mx2_usbh2_device, &stk5_usbh2_data);
	return ret;
}
device_initcall(stk5_usbh2_register);
#endif // CONFIG_ARCH_MXC_EHCI_USBH2

#if defined(CONFIG_ARCH_MXC_EHCI_USBOTG) ||				\
	defined(CONFIG_USB_FSL_USB2) || defined(CONFIG_USB_FSL_USB2_MODULE)

static unsigned int mx27_usbotg_hs_gpios_active[] = {
	PE25_PF_USBOTG_DATA7,
	PC8_PF_USBOTG_DATA6,
	PC7_PF_USBOTG_DATA5,
	PC12_PF_USBOTG_DATA4,
	PC13_PF_USBOTG_DATA3,
	PC10_PF_USBOTG_DATA2,
	PC11_PF_USBOTG_DATA1,
	PC9_PF_USBOTG_DATA0,

	PE24_PF_USBOTG_CLK,
	PE2_PF_USBOTG_DIR,
	PE0_PF_USBOTG_NXT,
	PE1_PF_USBOTG_STP,

	MXC_PIN(B, 29, AOUT, GPIO_IN),	/* OC detect */
};

static unsigned int mx27_usbotg_hs_gpios_inactive[] = {
	MXC_PIN(E, 1, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(E, 24, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(E, 2, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(E, 0, AOUT, GPIO_IN | GPIO_PUEN),

	MXC_PIN(E, 25, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(C, 8, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(C, 7, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(C, 12, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(C, 13, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(C, 10, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(C, 11, AOUT, GPIO_IN | GPIO_PUEN),
	MXC_PIN(C, 9, AOUT, GPIO_IN | GPIO_PUEN),

	MXC_PIN(B, 29, AOUT, GPIO_IN),	/* OC detect */
};

static struct clk *usbotg_clk;

int gpio_usbotg_hs_active(void)
{
	int ret;

	usbotg_clk = clk_get(NULL, "clk_26m");
	if (IS_ERR(usbotg_clk)) {
		ret = PTR_ERR(usbotg_clk);
		printk(KERN_ERR "Cannot request 26MHz clock: %d\n", ret);
		clk_put(usbotg_clk);
		usbotg_clk = NULL;
		return ret;
	} else {
		clk_enable(usbotg_clk);
	}

	ret = mxc_gpio_setup_multiple_pins(mx27_usbotg_hs_gpios_active,
					ARRAY_SIZE(mx27_usbotg_hs_gpios_active),
					"USB");
	return ret;
}

void gpio_usbotg_hs_inactive(void)
{
	int i;

	if (usbotg_clk != NULL) {
		clk_disable(usbotg_clk);
		clk_put(usbotg_clk);
		usbotg_clk = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(mx27_usbotg_hs_gpios_inactive); i++) {
		DBG(1, "Enabling Pullup on P%c%d\n",
			GPIO_PORT(mx27_usbotg_hs_gpios_inactive[i]) + 'A',
			GPIO_INDEX(mx27_usbotg_hs_gpios_inactive[i]));
		mxc_gpio_mode(mx27_usbotg_hs_gpios_inactive[i]);
	}

	mxc_gpio_release_multiple_pins(mx27_usbotg_hs_gpios_inactive,
				ARRAY_SIZE(mx27_usbotg_hs_gpios_inactive));
}
#endif /* CONFIG_ARCH_MXC_EHCI_USBOTG || CONFIG_USB_FSL_USB2 || CONFIG_USB_FSL_USB2_MODULE */

#ifdef CONFIG_ARCH_MXC_EHCI_USBOTG
static int stk5_usbotg_init(struct platform_device *pdev)
{
	int ret;
	void __iomem *base = MX2_IO_ADDRESS(OTG_BASE_ADDR + 0x0);
	struct clk *usbclk = clk_get(&pdev->dev, "usb");

	if (IS_ERR(usbclk)) {
		printk(KERN_ERR "%s: Failed to get usb_clk: %ld\n", __FUNCTION__, PTR_ERR(usbclk));
		return PTR_ERR(usbclk);
	}

	/* disable sampling of ID pin for OTG port being used as HOST */
	__raw_writel(__raw_readl(base + REG_OTGSC) & ~OTGSC_IDPU,
		base + REG_OTGSC);

	ret = gpio_usbotg_hs_active();
	if (ret != 0) {
		return ret;
	}

	ret = stk5_usb_init(pdev, base);
	if (ret != 0) {
		DBG(0, "%s: stk5_usb_init failed: %d\n", __FUNCTION__, ret);
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

static int stk5_usbotg_exit(struct platform_device *pdev)
{
	struct clk *usbclk = clk_get(&pdev->dev, "usb");

	if (!IS_ERR(usbclk)) {
		/* reenable USB clock, since the driver will disable it upon removal */
		clk_enable(usbclk);
		clk_put(usbclk);
	}

	gpio_usbotg_hs_inactive();
	return 0;
}

static struct mxc_usbh_platform_data stk5_usbotg_data = {
	.init = stk5_usbotg_init,
	.exit = stk5_usbotg_exit,
};

static int stk5_usbotg_register(void)
{
	int ret;

	ret = mxc_register_device(&mx2_otg_host_device, &stk5_usbotg_data);
	return ret;
}
device_initcall(stk5_usbotg_register);
#endif // CONFIG_ARCH_MXC_EHCI_USBOTG

#if defined(CONFIG_USB_FSL_USB2) || defined(CONFIG_USB_FSL_USB2_MODULE)
static struct fsl_usb2_platform_data stk5_usb_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_ULPI,
};

static int __init stk5_usb_gadget_register(void)
{
	int ret;

	ret = gpio_usbotg_hs_active();
	if (ret) {
		printk(KERN_ERR "Failed to initialize USB OTG pins for ULPI interface: %d\n", ret);
		return ret;
	}
	return mxc_register_device(&mx2_otg_udc_device, &stk5_usb_pdata);
}
device_initcall(stk5_usb_gadget_register);
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
static unsigned int stk5_sgtl5000_pads[] = {
	PC31_PF_SSI3_CLK,
	PC29_PF_SSI3_RXD,
	PC30_PF_SSI3_TXD,
	PC28_PF_SSI3_FS,
};

static int stk5_sgtl5000_plat_init(void)
{
	int ret;

	DBG(0, "%s: \n", __FUNCTION__);
	ret = mxc_gpio_setup_multiple_pins(stk5_sgtl5000_pads,
					ARRAY_SIZE(stk5_sgtl5000_pads),
		"SSI3");
	return ret;
}

static void stk5_sgtl5000_plat_finit(void)
{
	DBG(0, "%s: \n", __FUNCTION__);

	mxc_gpio_release_multiple_pins(stk5_sgtl5000_pads,
				ARRAY_SIZE(stk5_sgtl5000_pads));
}

static struct mxc_audio_platform_data stk5_sgtl5000_data = {
	.ssi_num = 0,
	.src_port = 1,
	.ext_port = 3,
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
#define TSC2007_PEN_GPIO		(GPIO_PORTC | 23)
#define TSC2007_DBG_GPIO		(GPIO_PORTC | 20)

static int stk5_tsc2007_pins[] = {
	MXC_PIN(C, 23, AOUT, GPIO_IN),
	MXC_PIN(C, 20, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
};

static int stk5_tsc2007_init(void)
{
	int ret;

	ret = mxc_gpio_setup_multiple_pins(stk5_tsc2007_pins,
					ARRAY_SIZE(stk5_tsc2007_pins),
					"TSC2007");
	if (ret == 0) {
		gpio_direction_input(TSC2007_PEN_GPIO);

	}
	return ret;
}

static void stk5_tsc2007_exit(void)
{
	mxc_gpio_release_multiple_pins(stk5_tsc2007_pins,
				ARRAY_SIZE(stk5_tsc2007_pins));
}

static int stk5_get_pendown(void)
{
	return !gpio_get_value(TSC2007_PEN_GPIO);
}

static struct tsc2007_platform_data stk5_tsc2007_pdata = {
	.model = 2007,
	.x_plate_ohms = 660,
	.get_pendown_state = stk5_get_pendown,
	.clear_penirq = NULL,
	.init_platform_hw = stk5_tsc2007_init,
	.exit_platform_hw = stk5_tsc2007_exit,
};
#endif

#ifdef CONFIG_I2C_IMX_SELECT1
static int mxc_i2c_1_pins[] = {
	PD17_PF_I2C_DATA,
	PD18_PF_I2C_CLK,
};

static int stk5_i2c_1_init(struct device *dev)
{
	DBG(0, "%s: Initialising I2C1 pins\n", __FUNCTION__);
	return mxc_gpio_setup_multiple_pins(mxc_i2c_1_pins,
					ARRAY_SIZE(mxc_i2c_1_pins), "I2C1");
}

static void stk5_i2c_1_exit(struct device *dev)
{
	mxc_gpio_release_multiple_pins(mxc_i2c_1_pins,
				ARRAY_SIZE(mxc_i2c_1_pins));
}

static struct imxi2c_platform_data stk5_i2c_1_data = {
	.bitrate = 100000,
	.init = stk5_i2c_1_init,
	.exit = stk5_i2c_1_exit,
};

static struct i2c_board_info karo_i2c_1_boardinfo[] __initdata = {
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

int __init stk5_i2c_register(void)
{
	int ret;

	if (stk5_board_rev < 3) {
		return 0;
	}

	ret = i2c_register_board_info(0, karo_i2c_1_boardinfo,
				ARRAY_SIZE(karo_i2c_1_boardinfo));
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C board info: %d\n", ret);
	}
	return ret;
}
arch_initcall(stk5_i2c_register);
#endif /* CONFIG_I2C_IMX_SELECT1 */

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
/* stk5 gpio keys driver */
struct gpio_keys_button stk5_gpio_keys[] = {
	{
		.code = KEY_POWER,
		.gpio = GPIO_PORTB | 24,
		.active_low = 0,
		.desc = "Power Button",
		.type = EV_KEY,		/* input event type (EV_KEY, EV_SW) */
		.wakeup = 1,		/* configure the button as a wake-up source */
		.debounce_interval = 1,	/* debounce ticks interval in msecs */
	},
};

struct gpio_keys_platform_data stk5_gpio_keys_pdata = {
	.buttons = stk5_gpio_keys,
	.nbuttons = ARRAY_SIZE(stk5_gpio_keys),
};

static struct platform_device stk5_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &stk5_gpio_keys_pdata,
	},
};

static int __init stk5_gpio_keys_register(void)
{
	if (stk5_board_rev < 3) {
		int i;
		int ret;

		for (i = 0; i < ARRAY_SIZE(stk5_gpio_keys); i++) {
			ret = gpio_request(stk5_gpio_keys[i].gpio, "gpio-keys");
			if (ret) {
				printk(KERN_ERR "Failed to request GPIO%d (P%c%d): %d\n",
					stk5_gpio_keys[i].gpio,
					stk5_gpio_keys[i].gpio / 32 + 'A',
					stk5_gpio_keys[i].gpio % 32, ret);
				return ret;
			}
			mxc_gpio_mode(stk5_gpio_keys[i].gpio);
			gpio_direction_input(stk5_gpio_keys[i].gpio);
			gpio_free(stk5_gpio_keys[i].gpio);
		}
		DBG(0, "%s: Registering device %s\n", __FUNCTION__,
			stk5_gpio_keys_device.name);
		return platform_device_register(&stk5_gpio_keys_device);
	}
	return 0;
}
arch_initcall(stk5_gpio_keys_register);
#endif

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led stk5_leds[] = {
	{
		.name = "GPIO-LED",
		.default_trigger = "heartbeat",
		.gpio = GPIO_PORTF + 13,
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

#if defined(CONFIG_FB_IMX) || defined(CONFIG_FB_IMX_MODULE)
/*
 * Setup GPIO for LCDC device to be active
 *
 */
#define STK5_LCD_BACKLIGHT_GPIO		(GPIO_PORTE |  5)
#define STK5_LCD_RESET_GPIO		(GPIO_PORTF | 12)
#define STK5_LCD_POWER_GPIO		(GPIO_PORTF | 11)

static unsigned int mx27_lcdc_gpios[] = {
#ifdef STK5_LCD_BACKLIGHT_GPIO
	MXC_PIN(F, 11, GPIO, GPIO_OUT | GPIO_DFLT_LOW), /* LCD power (active HIGH) */
	MXC_PIN(F, 12, GPIO, GPIO_OUT | GPIO_DFLT_LOW), /* LCD reset (active LOW) */
#if !defined(CONFIG_BACKLIGHT_PWM) && !defined(CONFIG_BACKLIGHT_PWM_MODULE)
	MXC_PIN(E,  5, GPIO, GPIO_OUT | GPIO_DFLT_HIGH), /* LCD backlight (PWM: 0: off 1: max brightness */
#endif
#endif
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

static int stk5_gpio_lcdc_active(struct platform_device *pdev)
{
	int ret;
	struct imx_fb_platform_data *pdata = pdev->dev.platform_data;

	BUG_ON(pdev != &mxc_fb_device);

	DBG(0, "%s: Setting up GPIO pins for LCD\n", __FUNCTION__);
	ret = mxc_gpio_setup_multiple_pins(mx27_lcdc_gpios,
					ARRAY_SIZE(mx27_lcdc_gpios), "LCD");
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
	if (pdata) {
		DBG(0, "%s: Remapping LCD buffer %08x..%08x\n", __FUNCTION__,
			pdata->fixed_screen_dma,
			pdata->fixed_screen_dma + 0x00200000 - 1);

		pdata->fixed_screen_cpu = ioremap(pdata->fixed_screen_dma, 0x00200000);
		DBG(0, "%s: LCD buffer %08x..%08x remapped to %p\n",
			__FUNCTION__, pdata->fixed_screen_dma,
			pdata->fixed_screen_dma + 0x00200000 - 1, pdata->fixed_screen_cpu);
	} else {
		printk(KERN_ERR "%s: No platform_data for %s device\n",
			__FUNCTION__, pdev->name);
	}
	return 0;
}

/*
 * Setup GPIO for LCDC device to be inactive
 *
 */
static void stk5_gpio_lcdc_inactive(struct platform_device *dev)
{
	mxc_gpio_release_multiple_pins(mx27_lcdc_gpios,
				ARRAY_SIZE(mx27_lcdc_gpios));
}

static void stk5_lcdc_power(int on)
{
	DBG(0, "%s: Switching LCD RESET %s\n", __FUNCTION__,
		on ? "off" : "on");
	if (on) {
		gpio_set_value(STK5_LCD_POWER_GPIO, 1);
		gpio_set_value(STK5_LCD_RESET_GPIO, 1);
	} else {
		gpio_set_value(STK5_LCD_RESET_GPIO, 0);
		gpio_set_value(STK5_LCD_POWER_GPIO, 0);
	}
}

#if !defined(CONFIG_BACKLIGHT_PWM) && !defined(CONFIG_BACKLIGHT_PWM_MODULE)
static void stk5_lcdc_backlight(int on)
{
	DBG(0, "%s: Switching LCD BACKLIGHT %s\n", __FUNCTION__,
		on ? "on" : "off");
	if (on) {
		gpio_set_value(STK5_LCD_BACKLIGHT_GPIO, 0);
	} else {
		gpio_set_value(STK5_LCD_BACKLIGHT_GPIO, 1);
	}
}
#else
#define stk5_lcdc_backlight	NULL

static int stk5_pwm_pads[] = {
	PE5_PF_PWMO, /* LCD backlight (PWM: 0: off 1: max brightness */
};

static int stk5_backlight_init(struct device *dev)
{
	int ret;
	ret = mxc_gpio_setup_multiple_pins(&stk5_pwm_pads[0], 1,
					"Backlight PWM");
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
	mxc_gpio_release_multiple_pins(&stk5_pwm_pads[0], 1);
}

static struct platform_pwm_backlight_data stk5_backlight_data = {
	.pwm_id = 0,
	.max_brightness = PWM_MAX_BRIGHTNESS,
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
#endif

static struct imx_fb_videomode stk5_fb_modes[] = {
	{
		.bpp	= 32,
		.mode = {
			.name = "G-ETV570G0DMU-18",
			.pixclock	= 34576,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.left_margin	= 23,
			.right_margin	= 23,

			.vsync_len	= 3,
			.upper_margin	= 5,
			.lower_margin	= 8,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp	= 16,
		.mode = {
			.name = "G-ETV570G0DMU-16",
			.pixclock	= 34576,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.left_margin	= 23,
			.right_margin	= 23,

			.vsync_len	= 3,
			.upper_margin	= 5,
			.lower_margin	= 8,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 32,
		.mode = {
			.name = "Xenarc_700_Y_VGA-18",
			.pixclock	= 34576,

			.xres		= 640,
			.yres		= 480,


			.hsync_len	= 64,
			.right_margin	= 138 + 1,
			.left_margin	= 118 + 3,

			.vsync_len	= 7,
			.upper_margin	= 44,
			.lower_margin	= 44,

		},
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 16,
		.mode = {
			.name = "Xenarc_700_Y_VGA-16",
			.pixclock	= 34576,

			.xres		= 640,
			.yres		= 480,


			.hsync_len	= 64,
			.right_margin	= 138 + 1,
			.left_margin	= 118 + 3,

			.vsync_len	= 7,
			.upper_margin	= 44,
			.lower_margin	= 44,
		},
		.pcr	= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 32,
		.mode = {
			.name = "Xenarc_700_Y-18",
			.pixclock	= 34576,

			.xres		= 800,
			.yres		= 480,


			.hsync_len	= 64,
			.right_margin	= 138 + 1,
			.left_margin	= 118 + 3,

			.vsync_len	= 7,
			.upper_margin	= 44,
			.lower_margin	= 44,

		},
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 16,
		.mode = {
			.name = "SHARP_LQ10D42-16",
			.pixclock	= 34576,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.right_margin	= 138 + 1,
			.left_margin	= 118 + 3,

			.vsync_len	= 7,
			.upper_margin	= 28,
			.lower_margin	= 60,
		},
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 32,
		.mode = {
			.name = "SHARP_LQ10D42-18",
			.pixclock	= 34576,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 64,
			.right_margin	= 138 + 1,
			.left_margin	= 118 + 3,

			.vsync_len	= 7,
			.upper_margin	= 28,
			.lower_margin	= 60,
		},
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 16,
		.mode = {
			.name = "SHARP_LQ104V1DG61-16",
			.pixclock	= 40000,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 32,
			.right_margin	= 32 + 1,
			.left_margin	= 0 + 3,

			.vsync_len	= 35,
			.upper_margin	= 0,
			.lower_margin	= 0,
		},
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_16 | PCR_FLMPOL | PCR_LPPOL | PCR_CLKPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 32,
		.mode = {
			.name = "SHARP_LQ104V1DG61-18",
			.pixclock	= 40000,

			.xres		= 640,
			.yres		= 480,

			.hsync_len	= 32,
			.right_margin	= 32 + 1,
			.left_margin	= 0 + 3,

			.vsync_len	= 35,
			.upper_margin	= 0,
			.lower_margin	= 0,
		},
		.pcr		= PCR_TFT | PCR_COLOR | PCR_PBSIZ_8 |
		PCR_BPIX_18 | PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
	{
		.bpp		= 32,     // 18bpp on LCD -> 32bpp in memory
		.mode = {
			.name = "Hantronix_S570-S",
			.pixclock	= 156000, // CLK Period (in picoseconds)

			.xres		= 320,    // Horizontal Display Time
			.yres		= 240,    // Vertical Display Time

			.hsync_len	= 30,     // HS Pulse Width
			.right_margin	= 38,     // Horizontal Back Porch
			.left_margin	= 20,     // Horizontal Front Porch

			.vsync_len	= 3,      // VS Pulse Width
			.upper_margin	= 4,      // Vertical Front Porch
			.lower_margin	= 15,     // Vertical Back Porch
		},
		.pcr		= PCR_TFT | PCR_COLOR | PCR_BPIX_18 |
		PCR_END_SEL | PCR_FLMPOL | PCR_LPPOL | PCR_SCLK_SEL,
	},
};

static struct imx_fb_platform_data stk5_fb_data = {
	.init = stk5_gpio_lcdc_active,
	.exit = stk5_gpio_lcdc_inactive,
	.lcd_power = stk5_lcdc_power,
	.backlight_power = stk5_lcdc_backlight,

	.dmacr		= 0x80040060,

	.cmap_greyscale	= 0,
	.cmap_inverse	= 0,
	.cmap_static	= 0,

	.fixed_screen_dma = 0xa4000000 - 0x00200000,

	.mode		= stk5_fb_modes,
	.num_modes	= ARRAY_SIZE(stk5_fb_modes),
};
#endif

#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)
/*
 * Setup GPIO for SDHC to be active
 */
static unsigned int mx27_sdhc_gpios[][6] = {
	{
		PE23_PF_SD1_CLK,
		PE22_PF_SD1_CMD,
		PE18_PF_SD1_D0,
		PE19_PF_SD1_D1,
		PE20_PF_SD1_D2,
		PE21_PF_SD1_D3,
	},
	{
		PB9_PF_SD2_CLK,
		PB8_PF_SD2_CMD,
		PB4_PF_SD2_D0,
		PB5_PF_SD2_D1,
		PB6_PF_SD2_D2,
		PB7_PF_SD2_D3,
	},
	{
		PD1_PF_SD3_CLK,
		PD0_PF_SD3_CMD,
		PD2_AF_SD3_D0,
		PD3_AF_SD3_D1,
		PD4_AF_SD3_D2,
		PD5_AF_SD3_D3,
	},
};

static int gpio_sdhc_active(int module)
{
	int ret;
	u16 data;

	switch (module) {
	case 0:
		ret = mxc_gpio_setup_multiple_pins(mx27_sdhc_gpios[module],
						ARRAY_SIZE(mx27_sdhc_gpios[module]),
						"SDHC");
		if (ret == 0) {
			/* 22k pull up for sd1 dat3 pins */
			data = __raw_readw(MX2_IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
			data |= 0x0c;
			__raw_writew(data, MX2_IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
		}
		break;
	case 1:
		ret = mxc_gpio_setup_multiple_pins(mx27_sdhc_gpios[module],
						ARRAY_SIZE(mx27_sdhc_gpios[module]),
						"SDHC");
		if (ret == 0) {
			/* 22k pull up for sd2 pins */
			data = __raw_readw(MX2_IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
			data |= 0xfff0;
			__raw_writew(data, MX2_IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
		}
		break;
	case 2:
		ret = mxc_gpio_setup_multiple_pins(mx27_sdhc_gpios[module],
						ARRAY_SIZE(mx27_sdhc_gpios[module]),
						"SDHC");
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

/*
 * Setup GPIO for SDHC to be inactive
 */
static int gpio_sdhc_inactive(int module)
{
	switch (module) {
	case 0:
	case 1:
	case 2:
		mxc_gpio_release_multiple_pins(mx27_sdhc_gpios[module],
					ARRAY_SIZE(mx27_sdhc_gpios[module]));
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*!
 * Resource definition for the SDHC1
 */
static struct resource stk5_sdhc1_resources[] = {
	{
		.start = SDHC1_BASE_ADDR,
		.end = SDHC1_BASE_ADDR + 0x3b,
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
static struct resource stk5_sdhc2_resources[] = {
	{
		.start = SDHC2_BASE_ADDR,
		.end = SDHC2_BASE_ADDR + 0x3b,
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

static inline int stk5_mmc_get_irq(int id)
{
	int irq;

	switch (id) {
	case 0:
		irq = stk5_sdhc1_resources[2].start;
		break;
	case 1:
		irq = stk5_sdhc2_resources[2].start;
		break;
	default:
		BUG();
	}
	return irq;
}

static const char *stk5_mmc_irqdesc[] = {
	"MMC card 0 detect",
	"MMC card 1 detect",
};

static int stk5_mmc_init(struct device *dev, irqreturn_t (*mmc_detect_irq)(int, void *),
			 void *data)
{
	int err;
	int id = to_platform_device(dev)->id;
	struct mmc_host *host = data;
	int irq = stk5_mmc_get_irq(id);

	err = gpio_sdhc_active(id);
	if (err) {
		return err;
	}

	host->caps |= MMC_CAP_4_BIT_DATA;

	err = request_irq(irq, mmc_detect_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			stk5_mmc_irqdesc[id], data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD: can't request MMC card detect IRQ %d\n",
			__FUNCTION__, irq);
		return err;
	}
	device_set_wakeup_capable(dev, 1);

	return 0;
}

static void stk5_mmc_exit(struct device *dev, void *data)
{
	int id = to_platform_device(dev)->id;
	int irq = stk5_mmc_get_irq(id);

	free_irq(irq, data);
	gpio_sdhc_inactive(id);
}

#if 0
static int stk5_mmc_suspend(struct device *dev, pm_message_t state)
{
	int id = to_platform_device(dev)->id;
	int irq = stk5_mmc_get_irq(id);

	if (device_may_wakeup(dev)) {
		DBG(0, "%s: Enabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return enable_irq_wake(irq);
	}
	return 0;
}

static int stk5_mmc_resume(struct device *dev)
{
	int id = to_platform_device(dev)->id;
	int irq = stk5_mmc_get_irq(id);

	if (device_may_wakeup(dev)) {
		DBG(0, "%s: Disabling IRQ %d wakeup\n", __FUNCTION__, irq);
		return disable_irq_wake(irq);
	}
	return 0;
}
#endif

static struct imxmmc_platform_data stk5_sdhc1_data = {
	.ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34,
	//.min_clk = 150000,
	//.max_clk = 25000000,
	//.detect_delay = 20,
	.get_ro = NULL,
	.setpower = NULL,
	.init = stk5_mmc_init,
	.exit = stk5_mmc_exit,
#if 0
	.suspend = stk5_mmc_suspend,
	.resume = stk5_mmc_resume,
#endif
};

static struct imxmmc_platform_data stk5_sdhc2_data = {
	.ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34,
	//.min_clk = 150000,
	//.max_clk = 25000000,
	//.detect_delay = 20,
	.init = stk5_mmc_init,
	.exit = stk5_mmc_exit,
};

static struct platform_device stk5_sdhc1_device = {
	.name = "mxc-mmc",
	.id = 0,
	.dev = {
		.platform_data = &stk5_sdhc1_data,
	},
	.num_resources = ARRAY_SIZE(stk5_sdhc1_resources),
	.resource = stk5_sdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device stk5_sdhc2_device = {
	.name = "mxc-mmc",
	.id = 1,
	.dev = {
		.platform_data = &stk5_sdhc2_data,
	},
	.num_resources = ARRAY_SIZE(stk5_sdhc2_resources),
	.resource = stk5_sdhc2_resources,
};
#endif

#if defined(CONFIG_AC97_BUS) || defined(CONFIG_AC97_BUS_MODULE)
static unsigned int stk5_ac97_gpios_off[] = {
	MXC_PIN(E, 17, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* RESET_OUT_B */
	PC31_PF_SSI3_CLK,	/* AC97_BITCLK */
	PC29_PF_SSI3_RXD,	/* AC97_SDATAIN */
	MXC_PIN(C, 23, AOUT, GPIO_IN),		/* UCB1400 interrupt */
	/* keep SDATA_OUT and SYNC deasserted to prevent UCB1400 from entering test mode */
	MXC_PIN(C, 30, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* AC97_SDATAOUT */
	MXC_PIN(C, 28, GPIO, GPIO_OUT | GPIO_DFLT_LOW),	/* AC97_SYNC */
};

static unsigned int stk5_ac97_gpios_on[] = {
	MXC_PIN(E, 17, GPIO, GPIO_OUT | GPIO_DFLT_HIGH),	/* RESET_OUT_B */
	PC30_PF_SSI3_TXD,	/* AC97_SDATAOUT */
	PC28_PF_SSI3_FS,	/* AC97_SYNC */
};

#if 0
static int gpio_ac97_active(void)
{
	int ret;

	ret = mxc_gpio_setup_multiple_pins(stk5_ac97_gpios_off,
					ARRAY_SIZE(stk5_ac97_gpios_off),
					"AC97");
	if (ret == 0) {
		udelay(1);
		mxc_gpio_release_multiple_pins(stk5_ac97_gpios_off,
					ARRAY_SIZE(stk5_ac97_gpios_off));
		/* deassert UCB1400 reset and reconfigure SDATA_OUT and SYNC */
		ret = mxc_gpio_setup_multiple_pins(stk5_ac97_gpios_on,
						ARRAY_SIZE(stk5_ac97_gpios_on),
						"AC97");
	}
	return ret;
}
#endif

static u64 stk5_dma_mask = DMA_BIT_MASK(32);

static int stk5_ac97_init(struct platform_device *dev, int ssi)
{
	int ret;
	const int irq = gpio_to_irq(GPIO_PORTC | 23);

	DBG(0, "%s: \n", __FUNCTION__);

	/* AC97 is hardwired to AUDMUX port 3 */
	ret = mxc_audmux_configure_sync_slave(ssi + 1, 3, MXC_AUDMUX_MODE_AC97);
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
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);

	/* configure RESET_OUT, AUD4_TXFS, AUD4_TXD as GPIO
	 * to prevent AC97 codecs from entering test mode
	 * due to TXFS or TXD sampled HIGH when RESET is deasserted
	 */
	ret = mxc_gpio_setup_multiple_pins(stk5_ac97_gpios_off,
					ARRAY_SIZE(stk5_ac97_gpios_off),
					"AC97");
	if (ret) {
		DBG(0, "%s: Failed to configure AC97 gpios: %d\n",
			__FUNCTION__, ret);
		return ret;
	}

	udelay(1);

	/* Deassert UCB1400 RESET */
	DBG(1, "%s: Switching P%c%d to 1\n",
		__FUNCTION__, IOMUX_TO_GPIO(stk5_ac97_gpios_on[0]) / 32 + 'A',
		IOMUX_TO_GPIO(stk5_ac97_gpios_on[0]) % 32);

	/* reconfigure pins for actual functions */
	mxc_gpio_release_multiple_pins(stk5_ac97_gpios_off,
				ARRAY_SIZE(stk5_ac97_gpios_off));
	ret = mxc_gpio_setup_multiple_pins(stk5_ac97_gpios_on,
					ARRAY_SIZE(stk5_ac97_gpios_on),
					"AC97");
	if (ret != 0) {
		DBG(0, "%s: Failed to reconfigure AC97 gpios: %d\n",
			__FUNCTION__, ret);
	}

	return ret;
}

static void stk5_ac97_exit(struct platform_device *dev)
{
	DBG(0, "%s: Releasing AC97 GPIOS\n", __FUNCTION__);
	mxc_gpio_release_multiple_pins(stk5_ac97_gpios_off,
				ARRAY_SIZE(stk5_ac97_gpios_off));
	DBG(0, "%s: Done\n", __FUNCTION__);
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
#endif /* CONFIG_AC97_BUS || CONFIG_AC97_BUS_MODULE */

#if defined(CONFIG_KEYBOARD_IMX) || defined(CONFIG_KEYBOARD_IMX_MODULE)
/*!
 * This array is used for mapping keypad scancodes to keyboard keycodes.
 */
static const uint32_t tx27_kpd_keycodes[] = {
	/* specify your keymap with KEY(row, col, keycode), */
	KEY(0, 0, KEY_POWER),
};

static struct matrix_keymap_data tx27_keymap_data = {
	.keymap = tx27_kpd_keycodes,
	.keymap_size = ARRAY_SIZE(tx27_kpd_keycodes),
};

static struct matrix_keypad_platform_data tx27_keypad_data = {
	.keymap_data = &tx27_keymap_data,
	.wakeup = 1,
};
#endif

static struct platform_dev_list {
	struct platform_device *pdev;
	void *pdata;
} stk5_devices[] __initdata = {
#if defined(CONFIG_I2C_IMX_SELECT1)
	{ .pdev = &mxc_i2c_1_device, .pdata = &stk5_i2c_1_data, },
#endif
#if defined(CONFIG_MXC_PWM) || defined(CONFIG_MXC_PWM_MODULE)
	{ .pdev = &mxc_pwm_device, },
#endif
#if defined(CONFIG_FB_IMX) || defined(CONFIG_FB_IMX_MODULE)
	{ .pdev = &mxc_fb_device, .pdata = &stk5_fb_data, },
#endif
#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
	{ .pdev = &stk5_backlight_pwm_device, .pdata = &stk5_backlight_data, },
#endif
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	{ .pdev = &stk5_led_device, },
#endif
#if defined(CONFIG_MX2_AUDMUX) || defined(CONFIG_MX2_AUDMUX_MODULE)
	{ .pdev = &mx2_audmux_device, },
#endif
#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)
	{ .pdev = &stk5_sdhc1_device, },
	{ .pdev = &stk5_sdhc2_device, },
#endif
#if defined(CONFIG_KEYBOARD_IMX) || defined(CONFIG_KEYBOARD_IMX_MODULE)
	{ .pdev = &mx2_keypad_device, .pdata = &tx27_keypad_data, },
#endif
};
#define STK5_NUM_DEVICES		ARRAY_SIZE(stk5_devices)

#ifdef CONFIG_MACH_TX27_CUSTOM
extern __init int stk5_custom_board_init(void);
#else
static inline int stk5_custom_board_init(void)
{
	return 0;
}
#endif

static __init int stk5_board_init(void)
{
	int i;

	DBG(0, "%s: \n", __FUNCTION__);

	/* enable SSI3_INT (PC23) for IRQ probing */
	set_irq_flags(gpio_to_irq(GPIO_PORTC | 23), IRQF_VALID | IRQF_PROBE);

	for (i = 0; i < STK5_NUM_DEVICES; i++) {
		int ret;

		if (stk5_devices[i].pdev == NULL)
			continue;
		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, stk5_devices[i].pdev, &stk5_devices[i].pdev->dev,
			stk5_devices[i].pdev->name);
		if (stk5_devices[i].pdata == NULL)  {
			ret = platform_device_register(stk5_devices[i].pdev);
		} else {
			ret = mxc_register_device(stk5_devices[i].pdev,
						stk5_devices[i].pdata);
		}
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, stk5_devices[i].pdev->name, ret);
		}
	}
	DBG(0, "%s: Done\n", __FUNCTION__);
	return stk5_custom_board_init();
}
arch_initcall(stk5_board_init);

static int __init stk5_board_rev_setup(char *cmdline)
{
	get_option(&cmdline, &stk5_board_rev);
	DBG(0, "%s: Board rev set to 0x%02x\n",
		__FUNCTION__, stk5_board_rev);
	return 1;
}
__setup("stk5_board_rev=", stk5_board_rev_setup);
