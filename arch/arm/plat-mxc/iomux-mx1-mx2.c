/*
 *  arch/arm/mach-mxc/generic.c
 *
 *  author: Sascha Hauer
 *  Created: april 20th, 2004
 *  Copyright: Synertronixx GmbH
 *
 *  Common code for i.MX machines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach/map.h>
#include <mach/iomux.h>

static void __iomem *base;

void mxc_gpio_mode(int gpio_mode)
{
	unsigned int pin = gpio_mode & GPIO_PIN_MASK;
	unsigned int port = (gpio_mode & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	unsigned int ocr = (gpio_mode & GPIO_OCR_MASK) >> GPIO_OCR_SHIFT;
	int ocr_shift = (pin % 16) << 1;
	unsigned int tmp;
	unsigned long flags;

	local_irq_save(flags);

	/* Pullup enable */
	tmp = __raw_readl(base + MXC_PUEN(port));
	if (gpio_mode & GPIO_PUEN)
		__raw_writel(tmp | (1 << pin), base + MXC_PUEN(port));

	/* Primary / alternate function */
	tmp = __raw_readl(base + MXC_GPR(port));
	if (gpio_mode & GPIO_AF)
		tmp |= (1 << pin);
	else
		tmp &= ~(1 << pin);
	__raw_writel(tmp, base + MXC_GPR(port));

	if (pin < 16) {
		tmp = __raw_readl(base + MXC_OCR1(port));
		tmp &= ~(3 << ocr_shift);
		tmp |= (ocr << ocr_shift);
		__raw_writel(tmp, base + MXC_OCR1(port));

		tmp = __raw_readl(base + MXC_ICONFA1(port));
		tmp &= ~(3 << ocr_shift);
		tmp |= ((gpio_mode >> GPIO_AOUT_SHIFT) & 3) << ocr_shift;
		__raw_writel(tmp, base + MXC_ICONFA1(port));

		tmp = __raw_readl(base + MXC_ICONFB1(port));
		tmp &= ~(3 << ocr_shift);
		tmp |= ((gpio_mode >> GPIO_BOUT_SHIFT) & 3) << ocr_shift;
		__raw_writel(tmp, base + MXC_ICONFB1(port));
	} else {
		tmp = __raw_readl(base + MXC_OCR2(port));
		tmp &= ~(3 << ocr_shift);
		tmp |= (ocr << ocr_shift);
		__raw_writel(tmp, base + MXC_OCR2(port));

		tmp = __raw_readl(base + MXC_ICONFA2(port));
		tmp &= ~(3 << ocr_shift);
		tmp |= ((gpio_mode >> GPIO_AOUT_SHIFT) & 3) << ocr_shift;
		__raw_writel(tmp, base + MXC_ICONFA2(port));

		tmp = __raw_readl(base + MXC_ICONFB2(port));
		tmp &= ~(3 << ocr_shift);
		tmp |= ((gpio_mode >> GPIO_BOUT_SHIFT) & 3) << ocr_shift;
		__raw_writel(tmp, base + MXC_ICONFB2(port));
	}

	/* Data direction */
	if (gpio_mode & GPIO_OUT) {
		if (gpio_mode & GPIO_DFLT_HIGH) {
			tmp = __raw_readl(base + MXC_DR(port));
			tmp |= 1 << pin;
			__raw_writel(tmp, base + MXC_DR(port));
		} else if (gpio_mode & GPIO_DFLT_LOW) {
			tmp = __raw_readl(base + MXC_DR(port));
			tmp &= ~(1 << pin);
			__raw_writel(tmp, base + MXC_DR(port));
		}
		tmp = __raw_readl(base + MXC_DDIR(port));
		tmp |= 1 << pin;
	} else {
		tmp = __raw_readl(base + MXC_DDIR(port));
		tmp &= ~(1 << pin);
	}
	__raw_writel(tmp, base + MXC_DDIR(port));

	/* use as gpio? */
	tmp = __raw_readl(base + MXC_GIUS(port));
	if (gpio_mode & (GPIO_PF | GPIO_AF))
		tmp &= ~(1 << pin);
	else
		tmp |= (1 << pin);
	__raw_writel(tmp, base + MXC_GIUS(port));

	/* Pullup disable */
	tmp = __raw_readl(base + MXC_PUEN(port));
	if (!(gpio_mode & GPIO_PUEN))
		__raw_writel(tmp & ~(1 << pin), base + MXC_PUEN(port));

	local_irq_restore(flags);
}
EXPORT_SYMBOL(mxc_gpio_mode);

int mxc_gpio_setup_multiple_pins(const int *pin_list, unsigned count,
		const char *label)
{
	const int *p = pin_list;
	int i;
	unsigned gpio;
	unsigned mode;
	int ret = -EINVAL;

	for (i = 0; i < count; i++) {
		gpio = *p & (GPIO_PIN_MASK | GPIO_PORT_MASK);
		mode = *p & ~(GPIO_PIN_MASK | GPIO_PORT_MASK);

		if (gpio >= (GPIO_PORT_MAX + 1) * 32)
			goto setup_error;

		ret = gpio_request(gpio, label);
		if (ret)
			goto setup_error;

		mxc_gpio_mode(gpio | mode);

		p++;
	}
	return 0;

setup_error:
	mxc_gpio_release_multiple_pins(pin_list, i);
	return ret;
}
EXPORT_SYMBOL(mxc_gpio_setup_multiple_pins);

void mxc_gpio_release_multiple_pins(const int *pin_list, int count)
{
	const int *p = pin_list;
	int i;

	for (i = 0; i < count; i++) {
		unsigned gpio = *p & (GPIO_PIN_MASK | GPIO_PORT_MASK);
		gpio_free(gpio);
		p++;
	}

}
EXPORT_SYMBOL(mxc_gpio_release_multiple_pins);

void __init mxc_iomux_init(void __iomem *iomux_base)
{
	base = iomux_base;
}
