/*
 * Copyright (C) 2009 by Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/errno.h>

#include <asm/clkdev.h>

#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/mx25.h>

static inline u32 read_ccm_reg(int reg)
{
	return __raw_readl(MX25_CCM_BASE + reg);
}

static inline void write_ccm_reg(u32 val, int reg)
{
	__raw_writel(val, MX25_CCM_BASE + reg);
}

static unsigned long get_rate_mpll(struct clk *clk)
{
	ulong mpctl = read_ccm_reg(CCM_MPCTL);

	return mxc_decode_pll(mpctl, clk_get_rate(clk->parent));
}

static unsigned long get_rate_mpll_main(struct clk *clk)
{
	unsigned long rate = clk_get_rate(clk->parent);

	if (clk->id == 1)
		rate = rate * 3 / 4;
	return rate;
}

static unsigned long get_rate_upll(struct clk *clk)
{
	ulong mpctl = read_ccm_reg(CCM_UPCTL);

	return mxc_decode_pll(mpctl, clk_get_rate(clk->parent));
}

static int clk_upll_enable(struct clk *clk)
{
	u32 reg;

	reg = read_ccm_reg(CCM_CCTL);
	reg &= ~(1 << 23);
	write_ccm_reg(reg, CCM_CCTL);

	while (!(read_ccm_reg(CCM_UPCTL) & (1 << 15)))
		cpu_relax();

	return 0;
}

static void clk_upll_disable(struct clk *clk)
{
	u32 reg;

	reg = read_ccm_reg(CCM_CCTL);
	reg |= (1 << 23);
	write_ccm_reg(reg, CCM_CCTL);
}

static unsigned long get_rate_arm(struct clk *clk)
{
	u32 cctl = read_ccm_reg(CCM_CCTL);
	unsigned long rate = clk_get_rate(clk->parent);

	return rate / ((cctl >> 30) + 1);
}

static unsigned long get_rate_ahb(struct clk *clk)
{
	u32 cctl = read_ccm_reg(CCM_CCTL);
	unsigned long parent_rate = clk_get_rate(clk->parent);

	return parent_rate / (((cctl >> 28) & 0x3) + 1);
}

static int set_rate_arm(struct clk *clk, unsigned long rate)
{
	u32 cctl;
	unsigned long parent_rate = clk_get_rate(clk->parent);
	u32 div = (parent_rate + rate - 1) / rate;

	if (div == 0 || div > 4)
		return -EINVAL;

	div--;
	cctl = read_ccm_reg(CCM_CCTL);
	if (div != (cctl >> 28 & 0x3)) {
		cctl = (cctl & ~(3 << 28)) | (div << 28);
		write_ccm_reg(cctl, CCM_CCTL);
	}
	return 0;
}

static unsigned long get_rate_ipg(struct clk *clk)
{
	return clk_get_rate(clk->parent) >> 1;
}

static unsigned long get_rate_per(struct clk *clk)
{
	int per = clk->id;
	u32 ofs = (per & 0x3) * 8;
	u32 reg = per & ~0x3;
	u32 val = (read_ccm_reg(CCM_PCDR0 + reg) >> ofs) & 0x3f;
	unsigned long fref = clk_get_rate(clk->parent);

	return fref / (val + 1);
}

static int set_rate_per(struct clk *clk, unsigned long rate)
{
	int per = clk->id;
	u32 ofs = (per & 0x3) * 8;
	u32 reg = per & ~0x3;
	u32 val;
	unsigned long parent_rate = clk_get_rate(clk->parent);
	u32 div = (parent_rate + rate - 1) / rate;

	if (!div || div > 64)
		return -EINVAL;

	div--;
	val = read_ccm_reg(CCM_PCDR0 + reg);

	if (div == ((val >> ofs) & 0x3f))
		return 0;

	val = (val & ~(0x3f << ofs)) | (div << ofs);
	write_ccm_reg(val, CCM_PCDR0 + reg);

	return 0;
}

static unsigned long get_rate_otg(struct clk *clk)
{
	u32 usbdiv = (read_ccm_reg(CCM_CCTL) >> 16) & 0x3f;
	unsigned long fref = clk_get_rate(clk->parent);

	return fref / (usbdiv + 1);
}

static int set_rate_otg(struct clk *clk, unsigned long rate)
{
	u32 cctl = read_ccm_reg(CCM_CCTL) & ~(0x3f << 16);
	unsigned long fref = clk_get_rate(clk->parent);
	u32 usbdiv = (fref / rate) - 1;

	if (usbdiv > 0x3f) {
		return -EINVAL;
	}
	cctl |= usbdiv << 16;
	write_ccm_reg(cctl, CCM_CCTL);
	return 0;
}

static int clk_cgcr_enable(struct clk *clk)
{
	if (likely(clk->enable_reg)) {
		u32 reg = __raw_readl(clk->enable_reg);
		reg |= 1 << clk->enable_shift;
		__raw_writel(reg, clk->enable_reg);
	}
	return 0;
}

static void clk_cgcr_disable(struct clk *clk)
{
	if (likely(clk->enable_reg)) {
		u32 reg = __raw_readl(clk->enable_reg);
		reg &= ~(1 << clk->enable_shift);
		__raw_writel(reg, clk->enable_reg);
	}
}

static int set_rate_parent(struct clk *clk, unsigned long rate)
{
	if (clk->parent && clk->parent->set_rate)
		return clk->parent->set_rate(clk->parent, rate);
	return -EINVAL;
}

static unsigned long external_high_reference = 24000000;

static unsigned long get_rate_high_reference(struct clk *clk)
{
	return external_high_reference;
}

static struct clk ckih_clk = {
	.get_rate	= get_rate_high_reference,
};

static unsigned long external_low_reference = 32768;

static unsigned long get_rate_low_reference(struct clk *clk)
{
	return external_low_reference;
}

static struct clk ckil_clk = {
	.get_rate	= get_rate_low_reference,
};

static unsigned long get_rate_esai(struct clk *clk)
{
	return 24620000;
}

static struct clk esai_clk = {
	.get_rate	= get_rate_esai,
};

static struct clk mpll_clk = {
	.parent		= &ckih_clk,
	.get_rate	= get_rate_mpll,
};

static struct clk upll_clk = {
	.parent		= &ckih_clk,
	.get_rate	= get_rate_upll,
	//.set_rate	= _clk_pll_set_rate,
	.enable		= clk_upll_enable,
	.disable	= clk_upll_disable,
};

/* MPLL with CCTL[ARM_SRC]=0 */
static struct clk mpll_main1_clk = {
	.id		= 0,
	.parent		= &mpll_clk,
	.get_rate	= get_rate_mpll_main,
};

/* MPLL with CCTL[ARM_SRC]=1 */
static struct clk mpll_main2_clk = {
	.id		= 1,
	.parent		= &mpll_clk,
	.get_rate	= get_rate_mpll_main,
};

static int clk_per_set_parent(struct clk *clk, struct clk *parent)
{
	u32 mcr;

	if (clk->parent == parent)
		return 0;

	if (parent != &mpll_clk && parent != &upll_clk)
		return -EINVAL;

	mcr = read_ccm_reg(CCM_MCR);
	if (parent == &upll_clk)
		mcr |= (1 << clk->id);
	else
		mcr &= ~(1 << clk->id);
	write_ccm_reg(mcr, CCM_MCR);
	return 0;
}

static int clk_cpu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 cctl;

	if (clk->parent == parent)
		return 0;

	if (parent != &mpll_main1_clk && parent != &mpll_main2_clk)
		return -EINVAL;

	cctl = read_ccm_reg(CCM_CCTL);
	switch (parent->id) {
	case 0:
		cctl |= 1 << 14;
	case 1:
		cctl &= ~(1 << 14);
	default:
		return -EINVAL;
	}
	write_ccm_reg(cctl, CCM_CCTL);
	return 0;
}

static struct clk ssi1_clk;
static struct clk ssi2_clk;
static struct clk per_clk13;

static int clk_ssi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 mcr;

	if (clk->parent == parent)
		return 0;

	if (parent != &per_clk13 && parent != &esai_clk)
		return -EINVAL;

	mcr = read_ccm_reg(CCM_MCR);
	if (parent == &esai_clk)
		mcr |= 1 << (clk->id + 17);
	else
		mcr &= ~(1 << (clk->id + 17));

	return 0;
}

static unsigned long get_rate_ssi(struct clk *clk)
{
	BUG_ON(!clk->parent);
	BUG_ON(!clk->parent->get_rate);
	return clk->parent->get_rate(clk->parent);
}

static int set_rate_ssi(struct clk *clk, unsigned long rate)
{
	BUG_ON(!clk->parent);
	BUG_ON(!clk->parent->set_rate);
	return clk->parent->set_rate(clk->parent, rate);
}

static struct clk cpu_clk = {
	.parent		= &mpll_main1_clk,
	.set_parent	= clk_cpu_set_parent,
	.get_rate	= get_rate_arm,
	.set_rate	= set_rate_arm,
};

static struct clk ahb_clk = {
	.parent		= &cpu_clk,
	.get_rate	= get_rate_ahb,
};

static struct clk ipg_clk = {
	.parent		= &ahb_clk,
	.get_rate	= get_rate_ipg,
};

static struct clk usb_ahb_clk = {
	.parent		= &ahb_clk,
};

#define DEFINE_CLOCK(name, i, er, es, gr, sr, s, p)	\
	static struct clk name = {			\
		.id		= i,			\
		.enable_reg	= er,			\
		.enable_shift	= es,			\
		.get_rate	= gr,			\
		.set_rate	= sr,			\
		.enable		= clk_cgcr_enable,	\
		.disable	= clk_cgcr_disable,	\
		.parent		= p,			\
		.secondary	= s,			\
	}

#define DEFINE_CLOCK1(name, i, er, es, pfx, s, p)		\
	static struct clk name = {				\
		.id		= i,				\
		.enable_reg	= er,				\
		.enable_shift	= es,				\
		.set_parent	= clk_##pfx##_set_parent,	\
		.get_rate	= get_rate_##pfx,		\
		.set_rate	= set_rate_##pfx,		\
		.enable		= clk_cgcr_enable,		\
		.disable	= clk_cgcr_disable,		\
		.parent		= p,				\
		.secondary	= s,				\
	}

DEFINE_CLOCK(cspi1_clk,   0, MX25_CCM_BASE + CCM_CGCR1,  5, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(cspi2_clk,   0, MX25_CCM_BASE + CCM_CGCR1,  6, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(cspi3_clk,   0, MX25_CCM_BASE + CCM_CGCR1,  7, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(iim_clk,     0, MX25_CCM_BASE + CCM_CGCR1, 26, NULL, NULL, NULL, &ipg_clk);

DEFINE_CLOCK1(per_clk0,   0, MX25_CCM_BASE + CCM_CGCR0,  0, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk1,   1, MX25_CCM_BASE + CCM_CGCR0,  1, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk2,   2, MX25_CCM_BASE + CCM_CGCR0,  2, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk3,   3, MX25_CCM_BASE + CCM_CGCR0,  3, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk4,   4, MX25_CCM_BASE + CCM_CGCR0,  4, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk5,   5, MX25_CCM_BASE + CCM_CGCR0,  5, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk6,   6, MX25_CCM_BASE + CCM_CGCR0,  6, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk7,   7, MX25_CCM_BASE + CCM_CGCR0,  7, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk8,   8, MX25_CCM_BASE + CCM_CGCR0,  8, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk9,   9, MX25_CCM_BASE + CCM_CGCR0,  9, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk10, 10, MX25_CCM_BASE + CCM_CGCR0, 10, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk11, 11, MX25_CCM_BASE + CCM_CGCR0, 11, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk12, 12, MX25_CCM_BASE + CCM_CGCR0, 12, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk13, 13, MX25_CCM_BASE + CCM_CGCR0, 13, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk14, 14, MX25_CCM_BASE + CCM_CGCR0, 14, per, NULL, &ahb_clk);
DEFINE_CLOCK1(per_clk15, 15, MX25_CCM_BASE + CCM_CGCR0, 15, per, NULL, &ahb_clk);

DEFINE_CLOCK(gpt1_clk1,   0, MX25_CCM_BASE + CCM_CGCR1, 19, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(gpt1_clk,    0, MX25_CCM_BASE + CCM_CGCR0,  5, NULL, NULL, &gpt1_clk1, &per_clk5);
DEFINE_CLOCK(gpt2_clk1,   0, MX25_CCM_BASE + CCM_CGCR1, 20, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(gpt2_clk,    0, MX25_CCM_BASE + CCM_CGCR0,  5, NULL, NULL, &gpt2_clk1, &per_clk5);

DEFINE_CLOCK(wdog_clk,    0, MX25_CCM_BASE + CCM_CGCR2, 19, NULL, NULL, NULL, &ipg_clk);

DEFINE_CLOCK(uart1_clk1,  0, MX25_CCM_BASE + CCM_CGCR2, 14, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(uart1_clk,	  0,			  NULL,	 0, NULL, NULL, &uart1_clk1, &per_clk15);
DEFINE_CLOCK(uart2_clk1,  0, MX25_CCM_BASE + CCM_CGCR2, 15, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(uart2_clk,	  0,			  NULL,	 0, NULL, NULL, &uart2_clk1, &per_clk15);
DEFINE_CLOCK(uart3_clk1,  0, MX25_CCM_BASE + CCM_CGCR2, 16, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(uart3_clk,	  0,			  NULL,	 0, NULL, NULL, &uart3_clk1, &per_clk15);
DEFINE_CLOCK(uart4_clk1,  0, MX25_CCM_BASE + CCM_CGCR2, 17, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(uart4_clk,	  0,			  NULL,	 0, NULL, NULL, &uart4_clk1, &per_clk15);
DEFINE_CLOCK(uart5_clk1,  0, MX25_CCM_BASE + CCM_CGCR2, 18, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(uart5_clk,	  0,			  NULL,	 0, NULL, NULL, &uart5_clk1, &per_clk15);

DEFINE_CLOCK(nfc_clk,	  0,			  NULL,	 0, NULL, NULL, NULL, &per_clk8);

DEFINE_CLOCK(usbotg_clk,  0, MX25_CCM_BASE + CCM_CGCR0, 28, get_rate_otg, set_rate_otg, NULL, &upll_clk);

DEFINE_CLOCK(pwm1_clk1,	  0, MX25_CCM_BASE + CCM_CGCR1, 31, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(pwm1_clk,	  0,			  NULL,	 0, NULL, NULL, &pwm1_clk1, &per_clk10);
DEFINE_CLOCK(pwm2_clk1,	  0, MX25_CCM_BASE + CCM_CGCR2,	 0, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(pwm2_clk,	  0,			  NULL,	 0, NULL, NULL, &pwm2_clk1, &per_clk10);
DEFINE_CLOCK(pwm3_clk1,	  0, MX25_CCM_BASE + CCM_CGCR2,	 1, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(pwm3_clk,	  0,			  NULL,	 1, NULL, NULL, &pwm3_clk1, &per_clk10);
DEFINE_CLOCK(pwm4_clk1,	  0, MX25_CCM_BASE + CCM_CGCR2,	 2, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(pwm4_clk,	  0,			  NULL,	 2, NULL, NULL, &pwm4_clk1, &per_clk10);

DEFINE_CLOCK(kpp_clk,	  0, MX25_CCM_BASE + CCM_CGCR1, 28, NULL, NULL, NULL, &ipg_clk);

DEFINE_CLOCK(tsc_clk,	  0, MX25_CCM_BASE + CCM_CGCR2, 13, NULL, NULL, NULL, &ipg_clk);

DEFINE_CLOCK(i2c_clk,	  0, MX25_CCM_BASE + CCM_CGCR0,  6, NULL, set_rate_parent, NULL, &per_clk6);

DEFINE_CLOCK(fec_clk1,    0, MX25_CCM_BASE + CCM_CGCR0, 23, NULL, NULL, NULL, &ahb_clk);
DEFINE_CLOCK(fec_clk,     0, MX25_CCM_BASE + CCM_CGCR1, 15, NULL, NULL, &fec_clk1, &ipg_clk);

DEFINE_CLOCK(lcdc_clk2,   0, MX25_CCM_BASE + CCM_CGCR0, 24, NULL, NULL, NULL, &ahb_clk);
DEFINE_CLOCK(lcdc_clk1,   0, MX25_CCM_BASE + CCM_CGCR1, 29, NULL, NULL, &lcdc_clk2, &ipg_clk);
DEFINE_CLOCK(lcdc_clk,    0, MX25_CCM_BASE + CCM_CGCR1,  7, NULL, set_rate_parent, &lcdc_clk1, &per_clk7);

DEFINE_CLOCK(csi_clk2,   0, MX25_CCM_BASE + CCM_CGCR0, 18, NULL, NULL, NULL, &upll_clk);
DEFINE_CLOCK(csi_clk1,   0, MX25_CCM_BASE + CCM_CGCR1,  4, NULL, NULL, &csi_clk2, &ipg_clk);
DEFINE_CLOCK(csi_clk,    0, MX25_CCM_BASE + CCM_CGCR0,  0, NULL, NULL, &csi_clk1, &per_clk0);

DEFINE_CLOCK(can1_clk,    0, MX25_CCM_BASE + CCM_CGCR1,  2, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(can2_clk,    0, MX25_CCM_BASE + CCM_CGCR1,  3, NULL, NULL, NULL, &ipg_clk);

DEFINE_CLOCK(rtc_clk,     0, MX25_CCM_BASE + CCM_CGCR1,  8, NULL, NULL, NULL, &ipg_clk);

DEFINE_CLOCK(esdhc1_clk1, 0, MX25_CCM_BASE + CCM_CGCR0, 21, NULL, NULL, NULL, &ahb_clk);
DEFINE_CLOCK(esdhc1_clk,  0, MX25_CCM_BASE + CCM_CGCR1, 13, NULL, NULL, &esdhc1_clk1, &per_clk3);

DEFINE_CLOCK(esdhc2_clk1, 1, MX25_CCM_BASE + CCM_CGCR0, 22, NULL, NULL, NULL, &ahb_clk);
DEFINE_CLOCK(esdhc2_clk,  1, MX25_CCM_BASE + CCM_CGCR1, 14, NULL, NULL, &esdhc2_clk1, &per_clk4);

DEFINE_CLOCK(ssi1_ipg_clk, 0, MX25_CCM_BASE + CCM_CGCR2, 11, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK1(ssi1_clk,    0,			   NULL,  0,  ssi, &ssi1_ipg_clk, &per_clk13);
DEFINE_CLOCK(ssi2_ipg_clk, 0, MX25_CCM_BASE + CCM_CGCR2, 11, NULL, NULL, NULL, &ipg_clk);
DEFINE_CLOCK1(ssi2_clk,    0,			   NULL,  0,  ssi, &ssi2_ipg_clk, &per_clk13);

DEFINE_CLOCK(sdma_ahb_clk, 0, MX25_CCM_BASE + CCM_CGCR0, 26, NULL, NULL, NULL, &ahb_clk);
DEFINE_CLOCK(sdma_clk, 0, MX25_CCM_BASE + CCM_CGCR2,  6, NULL, NULL, &sdma_ahb_clk, &ipg_clk);

#define _REGISTER_CLOCK(d, n, c)	\
	{				\
		.dev_id = d,		\
		.con_id = n,		\
		.clk = &c,		\
	},

static struct clk_lookup lookups[] = {
	_REGISTER_CLOCK(NULL, "ckil", ckil_clk)
	_REGISTER_CLOCK(NULL, "ckih", ckih_clk)
	_REGISTER_CLOCK(NULL, "ahb", ahb_clk)
	_REGISTER_CLOCK(NULL, "iim", iim_clk)
	_REGISTER_CLOCK(NULL, "sdma", sdma_clk)
	_REGISTER_CLOCK(NULL, "usb_ahb", usb_ahb_clk)
	_REGISTER_CLOCK("imx-uart.0", NULL, uart1_clk)
	_REGISTER_CLOCK("imx-uart.1", NULL, uart2_clk)
	_REGISTER_CLOCK("imx-uart.2", NULL, uart3_clk)
	_REGISTER_CLOCK("imx-uart.3", NULL, uart4_clk)
	_REGISTER_CLOCK("imx-uart.4", NULL, uart5_clk)
	_REGISTER_CLOCK("mxc-ehci.0", "usb", usbotg_clk)
	_REGISTER_CLOCK("mxc-ehci.1", "usb", usbotg_clk)
	_REGISTER_CLOCK("mxc-ehci.2", "usb", usbotg_clk)
	_REGISTER_CLOCK("fsl-usb2-udc", "usb", usbotg_clk)
	_REGISTER_CLOCK("mxc_nand.0", NULL, nfc_clk)
	_REGISTER_CLOCK("spi_imx.0", NULL, cspi1_clk)
	_REGISTER_CLOCK("spi_imx.1", NULL, cspi2_clk)
	_REGISTER_CLOCK("spi_imx.2", NULL, cspi3_clk)
	_REGISTER_CLOCK("mxc_gpt.1", NULL, gpt2_clk)
	_REGISTER_CLOCK("mxc_pwm.0", NULL, pwm1_clk)
	_REGISTER_CLOCK("mxc_pwm.1", NULL, pwm2_clk)
	_REGISTER_CLOCK("mxc_pwm.2", NULL, pwm3_clk)
	_REGISTER_CLOCK("mxc_pwm.3", NULL, pwm4_clk)
	_REGISTER_CLOCK("imx-keypad", NULL, kpp_clk)
	_REGISTER_CLOCK("mx25-tsc", NULL, tsc_clk)
	_REGISTER_CLOCK("imx-i2c.0", NULL, i2c_clk)
	_REGISTER_CLOCK("imx-i2c.1", NULL, i2c_clk)
	_REGISTER_CLOCK("imx-i2c.2", NULL, i2c_clk)
	_REGISTER_CLOCK("fec", NULL, fec_clk)
	_REGISTER_CLOCK("mx25-camera.0", NULL, csi_clk)
	_REGISTER_CLOCK("imx-wdt.0", NULL, wdog_clk)
	_REGISTER_CLOCK("imx-fb.0", NULL, lcdc_clk)
	_REGISTER_CLOCK("mxc-flexcan.0", NULL, can1_clk)
	_REGISTER_CLOCK("mxc-flexcan.1", NULL, can2_clk)
	_REGISTER_CLOCK("rtc-mx25.0", NULL, rtc_clk)
	_REGISTER_CLOCK("sdhci.0", NULL, esdhc1_clk)
	_REGISTER_CLOCK("sdhci.1", NULL, esdhc2_clk)
	_REGISTER_CLOCK("mxc-ssi.0", NULL, ssi1_clk)
	_REGISTER_CLOCK("mxc-ssi.1", NULL, ssi2_clk)
};

static struct clk *per_clks[] __initdata = {
	&per_clk0,
	&per_clk1,
	&per_clk2,
	&per_clk3,
	&per_clk4,
	&per_clk5,
	&per_clk6,
	&per_clk7,
	&per_clk8,
	&per_clk9,
	&per_clk10,
	&per_clk11,
	&per_clk12,
	&per_clk13,
	&per_clk14,
	&per_clk15,
};

int __init mx25_clocks_init(unsigned long fref)
{
	int i;
	u32 mcr;

	/* setup parent clocks according to settings from bootloader */
	if (read_ccm_reg(CCM_CCTL) & (1 << 14))
		cpu_clk.parent = &mpll_main2_clk;
	if (read_ccm_reg(CCM_MCR) & (1 << 17))
		ssi1_clk.parent = &esai_clk;
	if (read_ccm_reg(CCM_MCR) & (1 << 18))
		ssi2_clk.parent = &esai_clk;
	mcr = read_ccm_reg(CCM_MCR);
	for (i = 0; i < ARRAY_SIZE(per_clks); i++)
		if (mcr & (1 << i))
			per_clks[i]->parent = &upll_clk;

	/* register all clocks */
	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);
#if 1
	/* switch off all unnecessary clocks */
#ifndef CONFIG_DEBUG_LL
	write_ccm_reg(0x00080000, CCM_CGCR0);
#else
	/* leave UART clock on for low level debug output */
	write_ccm_reg(0x00088000, CCM_CGCR0);
#endif
	write_ccm_reg(0x00000000, CCM_CGCR1);
	write_ccm_reg(0x00000000, CCM_CGCR2);
#endif
	pr_info("Clock input source is %ld\n", clk_get_rate(&ckih_clk));

	pr_info("CPU: %lu.%03luMHz\n",
		clk_get_rate(&cpu_clk) / 1000000,
		clk_get_rate(&cpu_clk) / 1000 % 1000);
	pr_info("AHB: %lu.%03luMHz\n",
		clk_get_rate(&ahb_clk) / 1000000,
		clk_get_rate(&ahb_clk) / 1000 % 1000);
	pr_info("MPLL: %lu.%03luMHz\n",
		clk_get_rate(&mpll_clk) / 1000000,
		clk_get_rate(&mpll_clk) / 1000 % 1000);
	pr_info("UPLL: %lu.%03luMHz\n",
		clk_get_rate(&upll_clk) / 1000000,
		clk_get_rate(&upll_clk) / 1000 % 1000);
	pr_info("IPG: %lu.%03luMHz\n",
		clk_get_rate(&ipg_clk) / 1000000,
		clk_get_rate(&ipg_clk) / 1000 % 1000);
	if ((i = clk_set_rate(&usbotg_clk, 60000000))) {
		printk("Could not set OTG clock rate: %d\n", i);
	}
	pr_info("OTG: %lu.%03luMHz\n",
		clk_get_rate(&usbotg_clk) / 1000000,
		clk_get_rate(&usbotg_clk) / 1000 % 1000);

	/* set LCDC base clock to highest possible frequency
	 * to get best resolution for pixel clock
	 */
	clk_set_parent(&per_clk7, &upll_clk);
	if ((i = clk_set_rate(&lcdc_clk, clk_get_rate(per_clk7.parent)))) {
		printk("Could not set LCD clock rate: %d\n", i);
	}
	pr_info("LCD: %lu.%03luMHz\n", clk_get_rate(&lcdc_clk) / 1000000,
		clk_get_rate(&lcdc_clk) / 1000 % 1000);
	pr_info("NFC: %lu.%03luMHz\n",
		clk_get_rate(&nfc_clk) / 1000000,
		clk_get_rate(&nfc_clk) / 1000 % 1000);

	mxc_timer_init(&gpt1_clk, MX25_IO_ADDRESS(MX25_GPT1_BASE_ADDR), 54);

	return 0;
}
