/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/clkdev.h>
#include <asm/div64.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/spba.h>

#include "crm_regs.h"

#include "karo.h"

static struct clk osc_clk;
static struct clk ckih_clk;
static struct clk fpm_clk;
static struct clk lp_apm_clk;
static struct clk pll1_main_clk;
static struct clk pll1_sw_clk;
static struct clk pll2_sw_clk;
static struct clk pll3_sw_clk;
static struct clk mcu_main_clk;
static struct clk emi_fast_clk;
static struct clk emi_slow_clk;
static struct clk emi_intr_clk;
static struct clk ipg_clk;
static struct clk ahb_clk;
static struct clk ahb_max_clk;
static struct clk ipu_clk;
static struct clk tve_clk;
static struct clk ddr_clk;
static struct clk axi_a_clk;
static struct clk axi_b_clk;
static struct clk ddr_hf_clk;
static struct clk mipi_hsp_clk;
static struct clk spba_clk;
static struct clk tmax2_clk;
static struct clk tmax3_clk;
static struct clk esdhc1_clk;
static struct clk esdhc2_clk;
static struct clk ssi_lp_apm_clk;
static struct clk vpu_clk;
static struct clk vpu_clk1;

#ifdef DEBUG
#define LEAVE_CLKS_ON
#define ENABLE_ALL_CLKS

static const char *_clk_lookup_name(struct clk *clk);
#endif

#define SPIN_DELAY	1000000 /* in nanoseconds */

static unsigned long _clk_parent_get_rate(struct clk *clk);

static void __calc_pre_post_dividers(unsigned int div, unsigned int *pre,
				unsigned int *post)
{
	unsigned int min_pre, temp_pre, old_err, err = 0;

	if (div >= 512) {
		*pre = 8;
		*post = 64;
	} else if (div >= 8) {
		min_pre = (div - 1) / 64 + 1;
		old_err = 8;
		for (temp_pre = 8; temp_pre >= min_pre; temp_pre--) {
			err = div % temp_pre;
			if (err == 0) {
				*pre = temp_pre;
				break;
			}
			err = temp_pre - err;
			if (err < old_err) {
				old_err = err;
				*pre = temp_pre;
			}
		}
		*post = (div + *pre - 1) / *pre;
	} else {
		*pre = div;
		*post = 1;
	}
}

static unsigned long _clk_pre_post_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned int pre, post;
	unsigned int div = _clk_parent_get_rate(clk) / rate;

	if (_clk_parent_get_rate(clk) % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return _clk_parent_get_rate(clk) / (pre * post);
}

/*
 * For the 4-to-1 muxed input clock
 */
static inline int _get_mux(struct clk *parent, struct clk *m0,
			struct clk *m1, struct clk *m2, struct clk *m3)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else
		BUG();
}

/*
 * For the ddr muxed input clock
 */
static inline int _get_mux_ddr(struct clk *parent, struct clk *m0,
			struct clk *m1, struct clk *m2, struct clk *m3, struct clk *m4)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else if (parent == m4)
		return 4;
	else
		BUG();
}

/* clock enable/disable functions */

static int _clk_ccgr_enable(struct clk *clk)
{
	u32 reg;

	DBG(1, "%s: Enabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);
	reg = __raw_readl(clk->enable_reg);
	reg |= MXC_CCM_CCGR_CG_MASK << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_ccgr_disable(struct clk *clk)
{
	u32 reg;

	DBG(1, "%s: Disabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);
#ifdef LEAVE_CLKS_ON
return;
#endif
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static void _clk_ccgr_disable_inwait(struct clk *clk)
{
	u32 reg;

	DBG(1, "%s: Disabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
}

static int _clk_pll_enable(struct clk *clk)
{
	u32 reg;
	int loops = 0;

	reg = __raw_readl(clk->enable_reg + MXC_PLL_DP_CTL);
	reg |= MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, clk->enable_reg + MXC_PLL_DP_CTL);

	/* Wait for lock */
	while (!(__raw_readl(clk->enable_reg + MXC_PLL_DP_CTL) & MXC_PLL_DP_CTL_LRF)) {
		udelay(1);
		loops += 1000;
		if (loops++ > SPIN_DELAY)
			panic("pll relock failed\n");
	}
	return 0;
}

static void _clk_pll_disable(struct clk *clk)
{
	u32 reg;

	DBG(1, "%s: Disabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);
#ifdef LEAVE_CLKS_ON
return;
#endif
	reg = __raw_readl(clk->enable_reg + MXC_PLL_DP_CTL) & ~MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, clk->enable_reg + MXC_PLL_DP_CTL);
}

static int _clk_ahb_max_enable(struct clk *clk)
{
	u32 reg;

	_clk_ccgr_enable(clk);

	/* Enable handshake with MAX when LPM is entered. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_ahb_max_disable(struct clk *clk)
{
	u32 reg;

	DBG(1, "%s: Disabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);
	_clk_ccgr_disable_inwait(clk);

	/* Disable handshake with MAX when LPM is entered */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static int _clk_fpm_enable(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCR);

	reg |= MXC_CCM_CCR_FPM_EN;
	__raw_writel(reg, MXC_CCM_CCR);
	return 0;
}

static void _clk_fpm_disable(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCR);

	DBG(1, "%s: Disabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);
	reg &= ~MXC_CCM_CCR_FPM_EN;
	__raw_writel(reg, MXC_CCM_CCR);
}

static int _clk_ipu_enable(struct clk *clk)
{
	u32 reg;

	/* Enable handshake with IPU when certain clock rates are changed */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg &= ~MXC_CCM_CCDR_IPU_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* Enable handshake with IPU when LPM is entered */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_ipu_disable(struct clk *clk)
{
	u32 reg;

	DBG(1, "%s: Disabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);

	/* Disable handshake with IPU whe dividers are changed */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_IPU_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* Disable handshake with IPU when LPM is entered */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static int _clk_sdma_enable(struct clk *clk)
{
	u32 reg;

	/* Enable handshake with SDMA when LPM is entered */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_sdma_disable(struct clk *clk)
{
	u32 reg;

	DBG(1, "%s: Disabling clk %s (%p)\n", __FUNCTION__,
		_clk_lookup_name(clk), clk);

	/* Disable handshake with SDMA when LPM is entered */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static int _clk_vpu_enable(struct clk *clk)
{
	/* Set VPU's parent to be axi_a or ahb depending on chip revision */
	if (mx51_revision() < MX51_CHIP_REV_2_0) {
		clk_set_parent(&vpu_clk, &ahb_clk);
		clk_set_parent(&vpu_clk1, &ahb_clk);
	} else {
		clk_set_parent(&vpu_clk, &axi_a_clk);
		clk_set_parent(&vpu_clk1, &axi_a_clk);
	}

	return 0;
}

static void _clk_vpu_disable(struct clk *clk)
{
	/* Set VPU's parent to be axi_b when its disabled. */
	clk_set_parent(&vpu_clk, &axi_b_clk);
	clk_set_parent(&vpu_clk1, &axi_b_clk);
}

/* clock get_rate/set_rate functions */

static unsigned long _clk_parent_get_rate(struct clk *clk)
{
	unsigned long rate;

	BUG_ON(!clk->parent);

	rate = clk_get_rate(clk->parent);
	DBG(2, "%s: clk %s rate is %lu\n", __FUNCTION__,
		_clk_lookup_name(clk->parent), rate);
	WARN_ON(rate == 0);
	return rate;
}

static int _clk_parent_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;

	BUG_ON(!clk->parent);

	ret = clk_set_rate(clk->parent, rate);
	if (ret == 0) {
		DBG(1, "%s: clk %s rate set to %lu\n", __FUNCTION__,
			_clk_lookup_name(clk->parent), rate);
	} else {
		DBG(0, "%s: Failed to set clk %s rate to %lu: %d\n",
			__FUNCTION__, _clk_lookup_name(clk->parent), rate, ret);
	}
	return ret;
}

static unsigned long _clk_pll_get_rate(struct clk *clk)
{
	s64 rate;
	long mfi, mfn, mfd, pdf, ref_clk, mfn_abs;
	unsigned long dp_op, dp_mfd, dp_mfn, dp_ctl, pll_hfsm, dbl;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	dp_ctl = __raw_readl(clk->enable_reg + MXC_PLL_DP_CTL);
	pll_hfsm = dp_ctl & MXC_PLL_DP_CTL_HFSM;
	dbl = dp_ctl & MXC_PLL_DP_CTL_DPDCK0_2_EN;

	if (pll_hfsm == 0) {
		dp_op = __raw_readl(clk->enable_reg + MXC_PLL_DP_OP);
		dp_mfd = __raw_readl(clk->enable_reg + MXC_PLL_DP_MFD);
		dp_mfn = __raw_readl(clk->enable_reg + MXC_PLL_DP_MFN);
	} else {
		dp_op = __raw_readl(clk->enable_reg + MXC_PLL_DP_HFS_OP);
		dp_mfd = __raw_readl(clk->enable_reg + MXC_PLL_DP_HFS_MFD);
		dp_mfn = __raw_readl(clk->enable_reg + MXC_PLL_DP_HFS_MFN);
	}
	pdf = dp_op & MXC_PLL_DP_OP_PDF_MASK;
	mfi = (dp_op & MXC_PLL_DP_OP_MFI_MASK) >> MXC_PLL_DP_OP_MFI_OFFSET;
	mfi = (mfi <= 5) ? 5 : mfi;
	mfd = dp_mfd & MXC_PLL_DP_MFD_MASK;
	mfn = mfn_abs = dp_mfn & MXC_PLL_DP_MFN_MASK;
	/* Two's complement */
	if (mfn >= 0x04000000) {
		mfn = 0x04000000 - mfn;
		mfn_abs -= 0x04000000;
	}

	ref_clk = 2 * parent_rate;
	if (dbl != 0)
		ref_clk *= 2;

	ref_clk /= (pdf + 1);
	rate = (u64)ref_clk * mfn_abs;
	do_div(rate, mfd + 1);
	if (mfn < 0)
		rate = -rate;
	rate = (ref_clk * mfi) + rate;
	return (unsigned long)rate;
}

static int _clk_pll_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	long mfi = -1, pdf = -1, mfn, mfd = 999999;
	s64 temp64;
	unsigned long quad_parent_rate;
	unsigned long pll_hfsm, dp_ctl;

	quad_parent_rate = 4 * _clk_parent_get_rate(clk);
	while (++pdf < 16 && mfi < 5)
		mfi = rate * (pdf + 1) / quad_parent_rate;
	if (mfi > 15)
		return -EINVAL;
	pdf--;

	temp64 = rate * (pdf + 1) - quad_parent_rate * mfi;
	do_div(temp64, quad_parent_rate / 1000000);
	mfn = (long)temp64;

	dp_ctl = __raw_readl(clk->enable_reg + MXC_PLL_DP_CTL);
	/* use dpdck0_2 */
	__raw_writel(dp_ctl | 0x1000L, clk->enable_reg + MXC_PLL_DP_CTL);
	pll_hfsm = dp_ctl & MXC_PLL_DP_CTL_HFSM;
	if (pll_hfsm == 0) {
		reg = (mfi << 4) | pdf;
		__raw_writel(reg, clk->enable_reg + MXC_PLL_DP_OP);
		__raw_writel(mfd, clk->enable_reg + MXC_PLL_DP_MFD);
		__raw_writel(mfn, clk->enable_reg + MXC_PLL_DP_MFN);
	} else {
		reg = (mfi << 4) | pdf;
		__raw_writel(reg, clk->enable_reg + MXC_PLL_DP_HFS_OP);
		__raw_writel(mfd, clk->enable_reg + MXC_PLL_DP_HFS_MFD);
		__raw_writel(mfn, clk->enable_reg + MXC_PLL_DP_HFS_MFN);
	}
	return 0;
}

static unsigned long _clk_pll1_sw_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div = 1;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (clk->parent == &pll2_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL2_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL2_PODF_OFFSET) + 1;
	} else if (clk->parent == &pll3_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL3_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL3_PODF_OFFSET) + 1;
	}
	rate = _clk_parent_get_rate(clk) / div;
	DBG(2, "%s: pll_sw %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static int _clk_pll1_sw_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;
	u32 reg, div = 1;

	reg = __raw_readl(MXC_CCM_CCSR);
	if (clk->parent == &pll2_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL2_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL2_PODF_OFFSET) + 1;
	} else if (clk->parent == &pll3_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL3_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL3_PODF_OFFSET) + 1;
	}
	ret = _clk_parent_set_rate(clk, rate * div);
	return ret;
}

static unsigned long _clk_axi_a_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_A_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_A_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / div;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static unsigned long _clk_axi_b_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_B_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_B_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / div;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}
static inline int my_clk_enable(struct clk *clk)
{
	return clk_enable(clk);
}

#define clk_enable(c) ({			\
	printk(KERN_DEBUG "Enabling %s\n", #c);	\
	my_clk_enable(c);			\
})

static int _clk_axi_a_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	clk_enable(&emi_fast_clk);
	clk_enable(&emi_slow_clk);
	clk_enable(&ipu_clk);
	clk_enable(&mipi_hsp_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_A_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_A_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AXI_A_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll _clk_axi_a_set_rate failed\n");
	}

	clk_disable(&emi_fast_clk);
	clk_disable(&emi_slow_clk);
	clk_disable(&ipu_clk);
	clk_disable(&mipi_hsp_clk);

	return 0;
}

static int _clk_axi_b_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	clk_enable(&emi_fast_clk);
	clk_enable(&emi_slow_clk);
	clk_enable(&ipu_clk);
	clk_enable(&mipi_hsp_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_B_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_B_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AXI_B_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll _clk_axi_b_set_rate failed\n");
	}

	clk_disable(&emi_fast_clk);
	clk_disable(&emi_slow_clk);
	clk_disable(&ipu_clk);
	clk_disable(&mipi_hsp_clk);

	return 0;
}

static unsigned long _clk_ahb_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AHB_PODF_MASK) >>
	       MXC_CCM_CBCDR_AHB_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / div;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static int _clk_ahb_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	clk_enable(&emi_fast_clk);
	clk_enable(&emi_slow_clk);
	clk_enable(&ipu_clk);
	clk_enable(&mipi_hsp_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AHB_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AHB_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AHB_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("_clk_ahb_set_rate failed\n");
	}

	clk_disable(&emi_fast_clk);
	clk_disable(&emi_slow_clk);
	clk_disable(&ipu_clk);
	clk_disable(&mipi_hsp_clk);

	return 0;
}

static unsigned long _clk_ahb_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = _clk_parent_get_rate(clk) / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return _clk_parent_get_rate(clk) / div;
}

static unsigned long _clk_fpm_get_rate(struct clk *clk)
{
	unsigned long rate = _clk_parent_get_rate(clk) * 512;

	if (__raw_readl(MXC_CCM_CCR) & MXC_CCM_CCR_FPM_MULT_MASK)
		rate *= 2;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static unsigned long _clk_fpm_div2_get_rate(struct clk *clk)
{
	unsigned long rate;
	rate = _clk_parent_get_rate(clk) / 2;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static unsigned long _clk_ipg_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_IPG_PODF_MASK) >>
	       MXC_CCM_CBCDR_IPG_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / div;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static unsigned long _clk_ipg_per_get_rate(struct clk *clk)
{
	unsigned long rate = 0;
	u32 reg, prediv1, prediv2, podf;

	if (clk->parent == &mcu_main_clk || clk->parent == &lp_apm_clk) {
		/* the mcu_main_clk is the one before the DVFS engine */
		reg = __raw_readl(MXC_CCM_CBCDR);
		prediv1 = ((reg & MXC_CCM_CBCDR_PERCLK_PRED1_MASK) >>
			   MXC_CCM_CBCDR_PERCLK_PRED1_OFFSET) + 1;
		prediv2 = ((reg & MXC_CCM_CBCDR_PERCLK_PRED2_MASK) >>
			   MXC_CCM_CBCDR_PERCLK_PRED2_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CBCDR_PERCLK_PODF_MASK) >>
			MXC_CCM_CBCDR_PERCLK_PODF_OFFSET) + 1;
		rate = _clk_parent_get_rate(clk) / (prediv1 * prediv2 * podf);
	} else if (clk->parent == &ipg_clk) {
		rate = _clk_parent_get_rate(clk);
	}
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static unsigned long _clk_ddr_hf_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_DDR_PODF_MASK) >>
	       MXC_CCM_CBCDR_DDR_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / div;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;

}

static unsigned long _clk_ddr_hf_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = _clk_parent_get_rate(clk) / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return _clk_parent_get_rate(clk) / div;
}

static int _clk_ddr_hf_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = _clk_parent_get_rate(clk) / rate;
	if (div == 0)
		div++;
	if (((_clk_parent_get_rate(clk) / div) != rate) || (div > 8))
		return -EINVAL;

	clk_enable(&emi_fast_clk);
	clk_enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_DDR_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_DDR_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_DDR_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("clk_ddr_hf_set_rate failed\n");
	}

	clk_disable(&emi_fast_clk);
	clk_disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_emi_slow_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_EMI_PODF_MASK) >>
	       MXC_CCM_CBCDR_EMI_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / div;
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static int _clk_emi_slow_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = _clk_parent_get_rate(clk) / rate;
	if (div == 0)
		div++;
	if (((_clk_parent_get_rate(clk) / div) != rate) || (div > 8))
		return -EINVAL;

	clk_enable(&emi_fast_clk);
	clk_enable(&ipu_clk);
	clk_enable(&mipi_hsp_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_EMI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_EMI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);
	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_EMI_PODF_BUSY) {
		getnstimeofday(&curtime);
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
			panic("_clk_emi_slow_set_rate failed\n");
	}

	clk_disable(&emi_fast_clk);
	clk_disable(&ipu_clk);
	clk_disable(&mipi_hsp_clk);

	return 0;
}

static unsigned long _clk_emi_slow_round_rate(struct clk *clk,
					unsigned long rate)
{
	u32 div;

	div = _clk_parent_get_rate(clk) / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return _clk_parent_get_rate(clk) / div;
}

static unsigned long _clk_tve_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	rate = _clk_parent_get_rate(clk);
	if ((reg & MXC_CCM_CSCMR1_TVE_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
		    MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
		div = (reg >> MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET) + 1;
		rate /= div;
	}
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static unsigned long _clk_tve_round_rate(struct clk *clk,
					unsigned long rate)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL)
		return -EINVAL;

	div = _clk_parent_get_rate(clk) / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return _clk_parent_get_rate(clk) / div;
}

static int _clk_tve_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL)
		return -EINVAL;

	div = _clk_parent_get_rate(clk) / rate;
	if (div == 0)
		div++;
	if (((_clk_parent_get_rate(clk) / div) != rate) || (div > 8))
		return -EINVAL;

	div--;
	reg = __raw_readl(MXC_CCM_CDCDR) & ~MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
	reg |= div << MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CDCDR);
	return 0;
}

static unsigned long _clk_esdhc1_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PODF_OFFSET) + 1;

	rate = _clk_parent_get_rate(clk) / (prediv * podf);
	DBG(0, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}
#define _clk_esdhc1_set_rate NULL
#define _clk_esdhc1_round_rate NULL

static unsigned long _clk_esdhc2_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_OFFSET) + 1;

	rate = _clk_parent_get_rate(clk) / (prediv * podf);
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}
#define _clk_esdhc2_set_rate	NULL
#define _clk_esdhc2_round_rate	NULL

#define _clk_esdhc3_get_rate	_clk_parent_get_rate
#define _clk_esdhc3_set_rate	NULL
#define _clk_esdhc3_round_rate	NULL
#define _clk_esdhc4_get_rate	_clk_parent_get_rate
#define _clk_esdhc4_set_rate	NULL
#define _clk_esdhc4_round_rate	NULL

static unsigned long _clk_hsi2c_serial_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	prediv = ((reg & MXC_CCM_CSCDR3_HSI2C_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR3_HSI2C_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR3_HSI2C_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR3_HSI2C_CLK_PODF_OFFSET) + 1;

	rate = _clk_parent_get_rate(clk) / (prediv * podf);
	DBG(1, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

#define _clk_hsi2c_serial_set_rate	NULL
#define _clk_hsi2c_serial_round_rate	NULL
#define _clk_hsi2c_serial_set_parent	NULL

static unsigned long _clk_ssi1_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);
	prediv = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK) >>
		  MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		return 0;
	podf = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK) >>
		MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET) + 1;

	rate = _clk_parent_get_rate(clk) / (prediv * podf);
	return rate;
}

#define _clk_ssi1_round_rate	_clk_pre_post_round_rate

static int _clk_ssi1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, prediv, podf;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	/* calculate the pre divider for the input clock
	 * to be <= 300MHz
	 */
	prediv = (parent_rate + 299999999) / 300000000;
	if (prediv > 8)
		return -EINVAL;

	if (prediv < 2)
		prediv = 2;

	if (rate * prediv > parent_rate)
		return -EINVAL;

	podf = parent_rate / (rate * prediv);
	DBG(-1, "%s: SSI1 clock: %lu.%03luMHz / (%u * %u) = %lu.%03luMHz\n",
		__FUNCTION__, parent_rate / 1000000,
		parent_rate / 1000 % 1000,
		prediv, podf, rate / 1000000, rate / 1000 % 1000);
	if (podf-- > 64)
		return -EINVAL;
	prediv--;

	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg = (reg & ~(MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK |
			MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK)) |
		(prediv << MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET) |
		(podf << MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET);
	__raw_writel(reg, MXC_CCM_CS1CDR);
	return 0;
}

static unsigned long _clk_ssi2_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS2CDR);
	prediv = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK) >>
		  MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		return 0;
	podf = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK) >>
		MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET) + 1;

	rate = _clk_parent_get_rate(clk) / (prediv * podf);
	return rate;
}

static int _clk_ssi2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, prediv, podf;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	/* calculate the pre divider for the input clock
	 * to be <= 300MHz
	 */
	prediv = (parent_rate + 299999999) / 300000000;
	if (prediv > 8)
		return -EINVAL;

	if (prediv < 2)
		prediv = 2;

	if (rate * prediv > parent_rate)
		return -EINVAL;

	podf = parent_rate / (rate * prediv);
	if (podf-- > 64)
		return -EINVAL;
	prediv--;

	reg = __raw_readl(MXC_CCM_CS2CDR);
	reg = (reg & ~(MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK |
			MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK)) |
		(prediv << MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET) |
		(podf << MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET);
	__raw_writel(reg, MXC_CCM_CS2CDR);
	return 0;
}

#define _clk_ssi2_round_rate	_clk_pre_post_round_rate

static unsigned long _clk_usboh3_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		return -EINVAL;
	podf = ((reg & MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET) + 1;

	rate = _clk_parent_get_rate(clk) / (prediv * podf);
	DBG(1, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	return rate;
}

#define _clk_usboh3_set_rate	NULL
#define _clk_usboh3_round_rate	NULL

static unsigned long _clk_usb_phy_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	if (clk->parent == &pll3_sw_clk) {
		DBG(0, "%s: USB PHY parent is PLL3\n", __FUNCTION__);
		reg = __raw_readl(MXC_CCM_CDCDR);
		prediv = ((reg & MXC_CCM_CDCDR_USB_PHY_PRED_MASK) >>
			  MXC_CCM_CDCDR_USB_PHY_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_USB_PHY_PODF_MASK) >>
			MXC_CCM_CDCDR_USB_PHY_PODF_OFFSET) + 1;

		rate = _clk_parent_get_rate(clk) / (prediv * podf);
	} else if (clk->parent == &osc_clk) {
		DBG(0, "%s: USB PHY parent is OSC\n", __FUNCTION__);
		rate = _clk_parent_get_rate(clk);
	} else {
		BUG();
	}
	DBG(1, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	return rate;
}

static int _clk_usb_phy_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = 0;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	if (clk->parent == &pll3_sw_clk) {
		u32 reg, prediv, podf = 1;

		prediv = parent_rate / rate;
		if (prediv * rate != parent_rate) {
			goto err;
		}

		switch (prediv) {
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			break;
		case 9:
			prediv = podf = 3;
			break;
		case 10:
			prediv = 5;
			podf = 2;
			break;
		case 12:
			prediv = 6;
			podf = 2;
			break;
		case 14:
			prediv = 5;
			podf = 2;
			break;
		case 15:
			prediv = 5;
			podf = 3;
			break;
		case 16:
			prediv = 8;
			podf = 2;
			break;
		default:
			goto err;
		}
		prediv--;
		podf--;
		reg = __raw_readl(MXC_CCM_CDCDR);
		reg &= ~(MXC_CCM_CDCDR_USB_PHY_PRED_MASK |
			MXC_CCM_CDCDR_USB_PHY_PODF_OFFSET);
		reg |= (prediv << MXC_CCM_CDCDR_USB_PHY_PRED_OFFSET) |
			(podf << MXC_CCM_CDCDR_USB_PHY_PODF_OFFSET);
		__raw_writel(reg, MXC_CCM_CDCDR);
	} else {
		ret = _clk_parent_set_rate(clk, rate);
	}
	return ret;

err:
	DBG(0, "%s: Cannot achieve %lu.%03luMHz clock rate with parent rate %lu.%03luMHz\n",
		__FUNCTION__, rate / 1000000, rate / 1000 % 1000,
		parent_rate / 1000000, parent_rate / 1000 % 1000);
	return -EINVAL;
}

#define _clk_usb_phy_round_rate	NULL

#define _clk_ipu_get_rate	NULL
#define _clk_ipu_set_rate	NULL
#define _clk_ipu_round_rate	NULL

static unsigned long _clk_nfc_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_NFC_PODF_MASK) >>
	       MXC_CCM_CBCDR_NFC_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / div;
	DBG(1, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static unsigned long _clk_nfc_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	/*
	 * Compute the divider we'd have to use to reach the target rate.
	 */

	div = _clk_parent_get_rate(clk) / rate;

	/*
	 * If there's a remainder after the division, then we have to increment
	 * the divider. There are two reasons for this:
	 *
	 * 1) The frequency we round to must be LESS THAN OR EQUAL to the
	 *    target. We aren't allowed to round to a frequency that is higher
	 *    than the target.
	 *
	 * 2) This also catches the case where target rate is less than the
	 *    parent rate, which implies a divider of zero. We can't allow a
	 *    divider of zero.
	 */

	if (_clk_parent_get_rate(clk) % rate)
		div++;

	/*
	 * The divider for this clock is 3 bits wide, so we can't possibly
	 * divide the parent by more than eight.
	 */

	if (div > 8)
		return -EINVAL;

	return _clk_parent_get_rate(clk) / div;

}

static int _clk_nfc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = _clk_parent_get_rate(clk) / rate;
	if (div == 0)
		div++;
	if (((_clk_parent_get_rate(clk) / div) != rate) || (div > 8))
		return -EINVAL;

	clk_enable(&emi_fast_clk);
	clk_enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_NFC_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_NFC_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);
	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_NFC_IPG_INT_MEM_PODF_BUSY){
		getnstimeofday(&curtime);
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
			panic("_clk_nfc_set_rate failed\n");
	}

	clk_disable(&emi_fast_clk);
	clk_disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_csi0_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR4);
	pred = ((reg & MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / (pred * podf);
	DBG(1, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

#define _clk_csi0_round_rate	_clk_pre_post_round_rate

static int _clk_csi0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = _clk_parent_get_rate(clk) / rate;

	if ((_clk_parent_get_rate(clk) / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set CSI clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR4) &
	    ~(MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_MASK |
		MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR4);

	return 0;
}

static unsigned long _clk_csi1_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR4);
	pred = ((reg & MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_OFFSET) + 1;
	rate = _clk_parent_get_rate(clk) / (pred * podf);
	DBG(1, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

#define _clk_csi1_round_rate	_clk_pre_post_round_rate

static int _clk_csi1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	unsigned int div, pre, post;

	div = _clk_parent_get_rate(clk) / rate;

	if ((_clk_parent_get_rate(clk) / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set CSI clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR4) &
	    ~(MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_MASK |
		MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR4);

	return 0;
}

#define _clk_vpu_get_rate	NULL
#define _clk_vpu_set_rate	NULL
#define _clk_vpu_round_rate	NULL

static unsigned long _clk_cspi_get_rate(struct clk *clk)
{
	unsigned long rate = _clk_parent_get_rate(clk);
	int prediv, podf;
	u32 reg = __raw_readl(MXC_CCM_CSCDR2);

	prediv = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR2_CSPI_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		return 0;
	podf = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_CSPI_CLK_PODF_OFFSET) + 1;

	return rate / (prediv * podf);
}

static int _clk_cspi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	unsigned int div, pre, post;
	unsigned long parent_rate = _clk_parent_get_rate(clk);

	div = _clk_parent_get_rate(clk) / rate;
	DBG(-1, "%s: Setting CSPI clock rate to %lu.%03luMHz div %u (base clock %lu.%03luMHz)\n",
		__FUNCTION__, rate / 1000000, rate / 1000 % 1000, div,
		parent_rate / 1000000, parent_rate / 1000 % 1000);

	if (div == 0 || (_clk_parent_get_rate(clk) / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);
	DBG(-1, "%s: CSPI clock: %lu.%03luMHz / (%u * %u) = %lu.%03luMHz\n",
		__FUNCTION__, _clk_parent_get_rate(clk) / 1000000,
		_clk_parent_get_rate(clk) / 1000 % 1000,
		pre, post, rate / 1000000, rate / 1000 % 1000);

	/* Set CSPI clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR2) &
	    ~(MXC_CCM_CSCDR2_CSPI_CLK_PRED_MASK |
		MXC_CCM_CSCDR2_CSPI_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR2_CSPI_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR2_CSPI_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

#define _clk_cspi_round_rate	_clk_pre_post_round_rate

/* set_parent functions */

static int _clk_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (parent == &osc_clk)
		reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_LP_APM_SEL;
	else if (parent == &fpm_clk)
		reg = __raw_readl(MXC_CCM_CCSR) | MXC_CCM_CCSR_LP_APM_SEL;
	else
		return -EINVAL;

	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

static int _clk_pll1_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll1_main_clk) {
		reg &= ~MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);

		/* Set the step_clk parent to be lp_apm, to save power. */
		mux = _get_mux(&lp_apm_clk, &lp_apm_clk, NULL, &pll2_sw_clk,
			       &pll3_sw_clk);
		reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
		    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
	} else {
		if (parent == &lp_apm_clk) {
			reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
			__raw_writel(reg, MXC_CCM_CCSR);

			mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
				       &pll3_sw_clk);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
			    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
		} else {
			mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
				       &pll3_sw_clk);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
			    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
			__raw_writel(reg, MXC_CCM_CCSR);

			reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		}
	}
	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

static int _clk_pll_main_set_parent(struct clk *clk, struct clk *parent)
{
	/* nothing to be done here */
	DBG(1, "%s: Changing parent of clk %s from %s to %s\n", __FUNCTION__,
		_clk_lookup_name(clk), _clk_lookup_name(clk->parent),
		_clk_lookup_name(parent));
	return 0;
}

static int _clk_pll2_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll2_sw_clk) {
		reg &= ~MXC_CCM_CCSR_PLL2_SW_CLK_SEL;
	} else {
		reg |= MXC_CCM_CCSR_PLL2_SW_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

static int _clk_pll3_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll3_sw_clk) {
		reg &= ~MXC_CCM_CCSR_PLL3_SW_CLK_SEL;
	} else {
		reg |= MXC_CCM_CCSR_PLL3_SW_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

static int _clk_ddr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, reg2;
	int mux;
	struct timespec nstimeofday;
	struct timespec curtime;

	reg = __raw_readl(MXC_CCM_CBCMR);
	reg2 = __raw_readl(MXC_CCM_CBCDR);
	mux = _get_mux_ddr(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk, &ddr_hf_clk);
	if (mux < 4) {
		reg = (reg & ~MXC_CCM_CBCMR_DDR_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET);
		__raw_writel(reg, MXC_CCM_CBCMR);
		reg2 = (reg2 & ~MXC_CCM_CBCDR_DDR_HF_SEL);
	} else {
		reg2 = (reg2 & ~MXC_CCM_CBCDR_DDR_HF_SEL) |
			(MXC_CCM_CBCDR_DDR_HF_SEL);
	}
	__raw_writel(reg2, MXC_CCM_CBCDR);
	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
		MXC_CCM_CDHIPR_DDR_HF_CLK_SEL_BUSY){
		getnstimeofday(&curtime);
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
			panic("_clk_ddr_set_parent failed\n");
	}
	return 0;
}

static int _clk_periph_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;
	struct timespec nstimeofday;
	struct timespec curtime;

	mux = _get_mux(parent, &pll1_sw_clk, &pll3_sw_clk, &lp_apm_clk, NULL);

	reg = __raw_readl(MXC_CCM_CBCMR) & ~MXC_CCM_CBCMR_PERIPH_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CBCMR_PERIPH_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCMR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll _clk_periph_apm_set_parent failed\n");
	}
	return 0;
}

static int _clk_ipg_per_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &mcu_main_clk, &lp_apm_clk, &ipg_clk, NULL);
	if (mux == 2) {
		reg |= MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL;
		if (mux == 0)
			reg &= ~MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL;
		else
			reg |= MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static int _clk_ipu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &ahb_clk,
		       &emi_slow_clk);
	reg = (reg & ~MXC_CCM_CBCMR_IPU_HSP_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_IPU_HSP_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);
	return 0;
}

static int _clk_emi_slow_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	clk_enable(&emi_fast_clk);
	clk_enable(&emi_slow_clk);
	clk_enable(&ipu_clk);
	clk_enable(&mipi_hsp_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	if (parent == &ahb_clk) {
		reg |= MXC_CCM_CBCDR_EMI_CLK_SEL;
	} else if (parent == &mcu_main_clk) {
		reg &= ~MXC_CCM_CBCDR_EMI_CLK_SEL;
	} else {
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_CBCDR);

	clk_disable(&emi_fast_clk);
	clk_disable(&emi_slow_clk);
	clk_disable(&ipu_clk);
	clk_disable(&mipi_hsp_clk);

	return 0;
}

static int _clk_tve_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);

	if (parent == &pll3_sw_clk) {
		reg &= ~(MXC_CCM_CSCMR1_TVE_CLK_SEL);
	} else if (parent == &osc_clk) {
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL;
		reg &= MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
	} else if (parent == &ckih_clk) {
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL;
		reg |= MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
	} else {
		return -EINVAL;
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static unsigned long _clk_uart_get_rate(struct clk *clk)
{
	unsigned long rate;
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_UART_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_UART_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET) + 1;

	rate = _clk_parent_get_rate(clk) / (prediv * podf);
	DBG(2, "%s: %s rate is %lu\n", __FUNCTION__, _clk_lookup_name(clk), rate);
	WARN_ON(rate == 0);
	return rate;
}

static int _clk_uart_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_UART_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_UART_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static int _clk_esdhc1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_ESDHC1_MSHC1_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC1_MSHC1_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static int _clk_esdhc2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_ESDHC2_MSHC2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC2_MSHC2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static int _clk_esdhc3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &esdhc1_clk)
		reg &= ~MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else if (parent == &esdhc2_clk)
		reg |= MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else
		return -EINVAL;
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static int _clk_esdhc4_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &esdhc1_clk)
		reg &= ~MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else if (parent == &esdhc2_clk)
		reg |= MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else
		return -EINVAL;
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static int _clk_ssi1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk,
		       &pll3_sw_clk, &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI1_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI1_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static int _clk_ssi2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk,
		       &pll3_sw_clk, &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static int _clk_usboh3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USBOH3_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_USBOH3_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static int _clk_usb_phy_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &osc_clk)
		reg &= ~MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	else if (parent == &pll3_sw_clk)
		reg |= MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	else
		return -EINVAL;

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

#define _clk_nfc_set_parent	NULL

static int _clk_csi0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CSCMR2_CSI_MCLK1_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_CSI_MCLK1_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static int _clk_csi1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CSCMR2_CSI_MCLK2_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_CSI_MCLK2_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static int _clk_vpu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static int _clk_cspi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_CSPI_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_CSPI_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}


/* clock definitions */

#define DEFINE_ROOT_CLOCK(name,rate)				\
static unsigned long name##_rate = rate;			\
static unsigned long _clk_##name##_get_rate(struct clk *clk)	\
{								\
	return name##_rate;					\
}								\
								\
static struct clk name##_clk = {				\
	.get_rate = _clk_##name##_get_rate,			\
}

DEFINE_ROOT_CLOCK(ckih, -1);
DEFINE_ROOT_CLOCK(ckih2, -1);
DEFINE_ROOT_CLOCK(osc, -1);
DEFINE_ROOT_CLOCK(ckil, -1);

//#define CLKSS

#ifdef CLKSS
#define pll_base_clk	fpm_clk
#else
#define pll_base_clk	osc_clk
#endif

static struct clk lp_apm_clk = {
	.parent = &pll_base_clk,
	.get_rate = _clk_parent_get_rate,
	.set_parent = _clk_lp_apm_set_parent,
};

static struct clk pll1_main_clk = {
	.parent = &pll_base_clk,
	.get_rate = _clk_pll_get_rate,
	.enable_reg = MXC_DPLL1_BASE,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_parent = _clk_pll_main_set_parent,
};

static struct clk pll1_sw_clk = {
	.parent = &pll1_main_clk,
	.get_rate = _clk_pll1_sw_get_rate,
	.set_rate = _clk_pll1_sw_set_rate,
	.set_parent = _clk_pll1_sw_set_parent,
};

static struct clk pll2_main_clk = {
	.parent = &pll_base_clk,
	.get_rate = _clk_pll_get_rate,
	.set_rate = _clk_pll_set_rate,
	.enable_reg = MXC_DPLL2_BASE,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_parent = _clk_pll_main_set_parent,
};

static struct clk pll2_sw_clk = {
	.parent = &pll2_main_clk,
	.get_rate = _clk_parent_get_rate,
	.set_rate = _clk_parent_set_rate,
	.set_parent = _clk_pll2_sw_set_parent,
};

static struct clk pll3_main_clk = {
	.parent = &pll_base_clk,
	.get_rate = _clk_pll_get_rate,
	.set_rate = _clk_pll_set_rate,
	.enable_reg = MXC_DPLL3_BASE,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_parent = _clk_pll_main_set_parent,
};

static struct clk pll3_sw_clk = {
	.parent = &pll3_main_clk,
	.get_rate = _clk_parent_get_rate,
	.set_rate = _clk_parent_set_rate,
	.set_parent = _clk_pll3_sw_set_parent,
};

static struct clk mcu_main_clk = {
	.parent = &pll2_sw_clk,
	.get_rate = _clk_parent_get_rate,
};

static struct clk ahb_clk = {
	.parent = &mcu_main_clk,
	.get_rate = _clk_ahb_get_rate,
	.set_rate = _clk_ahb_set_rate,
	.round_rate = _clk_ahb_round_rate,
};

static struct clk axi_a_clk = {
	.parent = &mcu_main_clk,
	.get_rate = _clk_axi_a_get_rate,
	.set_rate = _clk_axi_a_set_rate,
};

static struct clk axi_b_clk = {
	.parent = &mcu_main_clk,
	.get_rate = _clk_axi_b_get_rate,
	.set_rate = _clk_axi_b_set_rate,
};

static struct clk periph_apm_clk = {
	.parent = &pll1_sw_clk,
	.set_parent = _clk_periph_apm_set_parent,
};

static struct clk ipg_clk = {
	.parent = &ahb_clk,
	.get_rate = _clk_ipg_get_rate,
};

static struct clk ipg_per_clk = {
	.parent = &lp_apm_clk,
	.get_rate = _clk_ipg_per_get_rate,
	.set_parent = _clk_ipg_per_set_parent,
};

static struct clk aips_tz1_clk = {
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG12_OFFSET,
	.enable = _clk_ccgr_enable,
	.disable = _clk_ccgr_disable_inwait,
};

static struct clk aips_tz2_clk = {
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG13_OFFSET,
	.enable = _clk_ccgr_enable,
	.disable = _clk_ccgr_disable_inwait,
};

static struct clk ahb_max_clk = {
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG14_OFFSET,
	.enable = _clk_ahb_max_enable,
	.disable = _clk_ahb_max_disable,
};

static struct clk ahbmux1_clk = {
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG8_OFFSET,
	.enable = _clk_ccgr_enable,
	.disable = _clk_ccgr_disable_inwait,
};

static struct clk ahbmux2_clk = {
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG9_OFFSET,
	.enable = _clk_ccgr_enable,
	.disable = _clk_ccgr_disable_inwait,
};

static struct clk emi_slow_clk = {
	.parent = &mcu_main_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG8_OFFSET,
	.enable = _clk_ccgr_enable,
	.disable = _clk_ccgr_disable_inwait,
	.get_rate = _clk_emi_slow_get_rate,
	.set_rate = _clk_emi_slow_set_rate,
	.round_rate = _clk_emi_slow_round_rate,
	.set_parent = _clk_emi_slow_set_parent,
};

static struct clk ddr_hf_clk = {
	.parent = &pll1_sw_clk,
	.get_rate = _clk_ddr_hf_get_rate,
	.round_rate = _clk_ddr_hf_round_rate,
	.set_rate = _clk_ddr_hf_set_rate,
};

static struct clk ddr_clk = {
	.parent = &ddr_hf_clk,
	.set_parent = _clk_ddr_set_parent,
};

static struct clk fpm_clk = {
	.parent = &ckil_clk,
	.get_rate = _clk_fpm_get_rate,
	.enable = _clk_fpm_enable,
	.disable = _clk_fpm_disable,
};

static struct clk fpm_div2_clk = {
	.parent = &fpm_clk,
	.get_rate = _clk_fpm_div2_get_rate,
};

static struct clk ipu_sec_clk = {
	.parent = &emi_fast_clk,
	.secondary = &ahbmux1_clk,
	.enable = _clk_ipu_enable,
	.disable = _clk_ipu_disable,
};

static struct clk uart_main_clk = {
	.parent = &pll3_sw_clk,
	.get_rate = _clk_uart_get_rate,
	.set_parent = _clk_uart_set_parent,
};

static struct clk esdhc_dep_clks = {
	.parent = &spba_clk,
	.secondary = &emi_fast_clk,
};

static struct clk esdhc1_sec_clk = {
	.parent = &tmax3_clk,
	.secondary = &esdhc_dep_clks,
};

static struct clk esdhc2_sec_clk = {
	.parent = &tmax2_clk,
	.secondary = &esdhc_dep_clks,
};

static struct clk esdhc3_sec_clk = {
	.parent = &ahb_max_clk,
	.secondary = &esdhc_dep_clks,
};

static struct clk esdhc4_sec_clk = {
	.parent = &tmax3_clk,
	.secondary = &esdhc_dep_clks,
};

static struct clk fec_sec2_clk = {
	.parent = &aips_tz2_clk,
	.secondary = &emi_fast_clk,
};

static struct clk fec_sec1_clk = {
	.parent = &tmax2_clk,
	.secondary = &fec_sec2_clk,
};

static struct clk sdma_ahb_clk = {
	 .parent = &ahb_clk,
#ifdef CONFIG_SDMA_IRAM
	 .secondary = &emi_intr_clk,
#endif
	 .enable = _clk_sdma_enable,
	 .disable = _clk_sdma_disable,
};

static struct clk vpu_emi_clk = {
	.parent = &emi_fast_clk,
#ifdef CONFIG_MXC_VPU_IRAM
	.secondary = &emi_intr_clk,
#endif
	.enable = _clk_vpu_enable,
	.disable = _clk_vpu_disable,
};

static int _clk_ssi_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux(parent, &ckih_clk, &lp_apm_clk, &ckih2_clk, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi_lp_apm_clk = {
	.parent = &ckih_clk,
	.set_parent = _clk_ssi_lp_apm_set_parent,
};

static struct clk usb_sec_clk = {
	.parent = &tmax2_clk,
	.secondary = &emi_fast_clk,
};

static unsigned long _clk_usb_get_rate(struct clk *clk)
{
	return 60000000;
}

static struct clk usb_clk = {
	.get_rate = _clk_usb_get_rate,
};

#define DEFINE_CLOCK(name, i, er, es, gr, s, p)		\
	static struct clk name = {			\
		.id		= i,			\
		.enable_reg	= er,			\
		.enable_shift	= es,			\
		.get_rate	= gr,			\
		.enable		= _clk_ccgr_enable,	\
		.disable	= _clk_ccgr_disable,	\
		.secondary	= s,			\
		.parent		= p,			\
	}

#define DEFINE_CLOCK1(name, i, er, es, pfx, s, p)	\
	static struct clk name = {			\
		.id		= i,			\
		.enable_reg	= er,			\
		.enable_shift	= es,			\
		.get_rate	= pfx##_get_rate,	\
		.set_rate	= pfx##_set_rate,	\
		.round_rate	= pfx##_round_rate,	\
		.set_parent	= pfx##_set_parent,	\
		.enable		= _clk_ccgr_enable,	\
		.disable	= _clk_ccgr_disable,	\
		.secondary	= s,			\
		.parent		= p,			\
	}

DEFINE_CLOCK(gpc_dvfs_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG12_OFFSET, NULL, NULL, &aips_tz2_clk);
DEFINE_CLOCK(rtc_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG14_OFFSET, NULL, &ipg_clk, &ckil_clk);

DEFINE_CLOCK(emi_fast_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG7_OFFSET, NULL, NULL, &ddr_clk);
DEFINE_CLOCK(emi_intr_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG7_OFFSET, NULL, &ahbmux2_clk, &ahb_clk);

DEFINE_CLOCK(gpt_ipg_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG10_OFFSET, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(gpt_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG9_OFFSET, _clk_parent_get_rate, &gpt_ipg_clk, &ipg_clk);
DEFINE_CLOCK(spba_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG0_OFFSET, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(garb_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG2_OFFSET, NULL, NULL, &axi_a_clk);
DEFINE_CLOCK(emi_garb_clk, 0, MXC_CCM_CCGR6, MXC_CCM_CCGR6_CG4_OFFSET, NULL, NULL, &axi_a_clk);

DEFINE_CLOCK1(tve_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG15_OFFSET, _clk_tve, NULL, &pll3_sw_clk);

DEFINE_CLOCK(iim_clk, 0, MXC_CCM_CCGR0, MXC_CCM_CCGR0_CG15_OFFSET, NULL, &aips_tz2_clk, &ipg_clk);

DEFINE_CLOCK(tmax1_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG0_OFFSET, NULL, &ahb_max_clk, &ahb_clk);
DEFINE_CLOCK(tmax2_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG1_OFFSET, NULL, &ahb_max_clk, &ahb_clk);
DEFINE_CLOCK(tmax3_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG2_OFFSET, NULL, &ahb_max_clk, &ahb_clk);

DEFINE_CLOCK(uart1_ipg_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG3_OFFSET, NULL, &ipg_clk, &aips_tz1_clk);
DEFINE_CLOCK(uart1_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG4_OFFSET, NULL, &uart1_ipg_clk, &uart_main_clk);
DEFINE_CLOCK(uart2_ipg_clk, 1, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG5_OFFSET, NULL, &ipg_clk, &aips_tz1_clk);
DEFINE_CLOCK(uart2_clk, 1, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG6_OFFSET, NULL, &uart2_ipg_clk, &uart_main_clk);
DEFINE_CLOCK(uart3_ipg_clk, 2, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG7_OFFSET, NULL, &ipg_clk, &aips_tz1_clk);
DEFINE_CLOCK(uart3_clk, 2, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG8_OFFSET, NULL, &uart3_ipg_clk, &uart_main_clk);

DEFINE_CLOCK(i2c1_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG9_OFFSET, NULL, NULL, &ipg_per_clk);
DEFINE_CLOCK(i2c2_clk, 1, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG10_OFFSET, NULL, NULL, &ipg_per_clk);

DEFINE_CLOCK(hsi2c_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG11_OFFSET, NULL, NULL, &ipg_clk);
DEFINE_CLOCK1(hsi2c_serial_clk, 0, MXC_CCM_CCGR1, MXC_CCM_CCGR1_CG11_OFFSET, _clk_hsi2c_serial, NULL, &pll3_sw_clk);

DEFINE_CLOCK(esdhc1_ipg_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG0_OFFSET, NULL, &esdhc1_sec_clk, &ipg_clk);
DEFINE_CLOCK1(esdhc1_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG1_OFFSET, _clk_esdhc1, &esdhc1_ipg_clk, &pll3_sw_clk);
DEFINE_CLOCK(esdhc2_ipg_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG2_OFFSET, NULL, &esdhc2_sec_clk, &ipg_clk);
DEFINE_CLOCK1(esdhc2_clk, 1, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG3_OFFSET, _clk_esdhc2, &esdhc2_ipg_clk, &pll3_sw_clk);
DEFINE_CLOCK(esdhc3_ipg_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG4_OFFSET, NULL, &esdhc3_sec_clk, &ipg_clk);
DEFINE_CLOCK1(esdhc3_clk, 2, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG5_OFFSET, _clk_esdhc3, &esdhc3_ipg_clk, &esdhc1_clk);
DEFINE_CLOCK(esdhc4_ipg_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG6_OFFSET, NULL, &esdhc4_sec_clk, &ipg_clk);
DEFINE_CLOCK1(esdhc4_clk, 3, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG7_OFFSET, _clk_esdhc4, &esdhc4_ipg_clk, &esdhc1_clk);

DEFINE_CLOCK(fec_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG12_OFFSET, _clk_parent_get_rate, &fec_sec1_clk, &ipg_clk);

DEFINE_CLOCK(ssi1_dep_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG8_OFFSET, NULL, &emi_fast_clk, &aips_tz2_clk);
DEFINE_CLOCK(ssi1_ipg_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG8_OFFSET, NULL, &ssi1_dep_clk, &ipg_clk);
DEFINE_CLOCK1(ssi1_clk, 0, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG9_OFFSET, _clk_ssi1, &ssi1_ipg_clk, &pll3_sw_clk);

DEFINE_CLOCK(ssi2_dep_clk, 1, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG8_OFFSET, NULL, &emi_fast_clk, &aips_tz2_clk);
DEFINE_CLOCK(ssi2_ipg_clk, 1, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG8_OFFSET, NULL, &ssi2_dep_clk, &ipg_clk);
DEFINE_CLOCK1(ssi2_clk, 1, MXC_CCM_CCGR3, MXC_CCM_CCGR3_CG9_OFFSET, _clk_ssi2, &ssi2_ipg_clk, &pll3_sw_clk);

DEFINE_CLOCK(usb_ahb_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG13_OFFSET, NULL, NULL, &ipg_clk);
DEFINE_CLOCK1(usb_phy_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG0_OFFSET, _clk_usb_phy, &tmax3_clk, &pll3_sw_clk);
DEFINE_CLOCK1(usboh3_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG14_OFFSET, _clk_usboh3, &usb_sec_clk, &pll3_sw_clk);

DEFINE_CLOCK1(ipu_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG5_OFFSET, _clk_ipu, &ipu_sec_clk, &ahb_clk);
DEFINE_CLOCK1(csi0_clk, 0, MXC_CCM_CCGR6, MXC_CCM_CCGR6_CG2_OFFSET, _clk_csi0, NULL, &pll3_sw_clk);
DEFINE_CLOCK1(csi1_clk, 1, MXC_CCM_CCGR6, MXC_CCM_CCGR6_CG3_OFFSET, _clk_csi1, NULL, &pll3_sw_clk);

DEFINE_CLOCK1(nfc_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG10_OFFSET, _clk_nfc, NULL, &emi_slow_clk);

DEFINE_CLOCK1(vpu_clk1, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG3_OFFSET, _clk_vpu, &vpu_emi_clk, &ahb_clk);
DEFINE_CLOCK(vpu_clk, 0, MXC_CCM_CCGR5, MXC_CCM_CCGR5_CG4_OFFSET, NULL, &vpu_clk1, &ahb_clk);

DEFINE_CLOCK(pwm1_clk1, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG5_OFFSET, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(pwm1_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG6_OFFSET, NULL, &pwm1_clk1, &ipg_per_clk);
DEFINE_CLOCK(pwm2_clk1, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG7_OFFSET, NULL, NULL, &ipg_clk);
DEFINE_CLOCK(pwm2_clk, 0, MXC_CCM_CCGR2, MXC_CCM_CCGR2_CG8_OFFSET, NULL, &pwm2_clk1, &ipg_per_clk);

static struct clk cspi_main_clk = {
	.parent = &pll3_sw_clk,
	.set_parent = _clk_cspi_set_parent,
	.get_rate = _clk_cspi_get_rate,
	.set_rate = _clk_cspi_set_rate,
	.round_rate = _clk_cspi_round_rate,
};

static struct clk cspi3_clk1 = {
	.id = 2,
	.parent = &ipg_clk,
	.secondary = &aips_tz2_clk,
};

DEFINE_CLOCK(cspi1_clk1, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG9_OFFSET, NULL, &spba_clk, &ipg_clk);
DEFINE_CLOCK(cspi1_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG10_OFFSET, NULL, &cspi1_clk1, &cspi_main_clk);
DEFINE_CLOCK(cspi2_clk1, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG11_OFFSET, NULL, &aips_tz2_clk, &ipg_clk);
DEFINE_CLOCK(cspi2_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG12_OFFSET, NULL, &cspi2_clk1, &cspi_main_clk);
DEFINE_CLOCK(cspi3_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG14_OFFSET, NULL, &cspi3_clk1, &cspi_main_clk);

/* clock definitions for MIPI HSC unit which has been removed
 * from documentation, but not from hardware
 */
static int _clk_hsc_enable(struct clk *clk)
{
	u32 reg;

	_clk_ccgr_enable(clk);

	/* Handshake with IPU when certain clock rates are changed. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg &= ~MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_hsc_disable(struct clk *clk)
{
	u32 reg;

	_clk_ccgr_disable(clk);

	/* No handshake with HSC as its not enabled. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

DEFINE_CLOCK(mipi_esc_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG5_OFFSET, NULL, NULL, &pll2_sw_clk);
DEFINE_CLOCK(mipi_hsc2_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG4_OFFSET, NULL, &mipi_esc_clk, &pll2_sw_clk);
DEFINE_CLOCK(mipi_hsc1_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG3_OFFSET, NULL, &mipi_hsc2_clk, &pll2_sw_clk);

DEFINE_CLOCK(sdma_clk, 0, MXC_CCM_CCGR4, MXC_CCM_CCGR4_CG15_OFFSET, NULL, &sdma_ahb_clk, &ipg_clk);

static struct clk mipi_hsp_clk = {
	.parent = &ipu_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG6_OFFSET,
	.enable = _clk_hsc_enable,
	.disable = _clk_hsc_disable,
	.secondary = &mipi_hsc1_clk,
};

#define _REGISTER_CLOCK(d, n, c)		\
	{					\
		.dev_id = d,			\
		.con_id = n,			\
		.clk = &c,			\
	}

static struct clk_lookup lookups[] = {
	_REGISTER_CLOCK(NULL, "osc", osc_clk),
	_REGISTER_CLOCK(NULL, "ckih", ckih_clk),
	_REGISTER_CLOCK(NULL, "ckih2", ckih2_clk),
	_REGISTER_CLOCK(NULL, "ckil", ckil_clk),
	_REGISTER_CLOCK(NULL, "fpm", fpm_clk),
	_REGISTER_CLOCK(NULL, "iim", iim_clk),
	_REGISTER_CLOCK(NULL, "ahb", ahb_clk),
	_REGISTER_CLOCK(NULL, "ahb_max", ahb_max_clk),
	_REGISTER_CLOCK(NULL, "ipg", ipg_clk),
	_REGISTER_CLOCK(NULL, "ipg_per", ipg_per_clk),
	_REGISTER_CLOCK(NULL, "gpc_dvfs", gpc_dvfs_clk),
	_REGISTER_CLOCK(NULL, "gpt", gpt_clk),
	_REGISTER_CLOCK(NULL, "usb_ahb", usb_ahb_clk),
	_REGISTER_CLOCK(NULL, "tve", tve_clk),
	_REGISTER_CLOCK(NULL, "sdma", sdma_clk),
	_REGISTER_CLOCK(NULL, "usboh3", usboh3_clk),
	_REGISTER_CLOCK(NULL, "usb_phy", usb_phy_clk),
	_REGISTER_CLOCK(NULL, "csi0_mclk", csi0_clk),
	_REGISTER_CLOCK(NULL, "csi1_mclk", csi1_clk),
//	_REGISTER_CLOCK(NULL, "", ipu_di_clk),
	_REGISTER_CLOCK(NULL, "mipi_hsp", mipi_hsp_clk),

	_REGISTER_CLOCK("mxc_rtc", NULL, rtc_clk),
	_REGISTER_CLOCK("fec", NULL, fec_clk),
	_REGISTER_CLOCK("mxc_ipu", NULL, ipu_clk),
	_REGISTER_CLOCK("imx-uart.0", NULL, uart1_clk),
	_REGISTER_CLOCK("imx-uart.1", NULL, uart2_clk),
	_REGISTER_CLOCK("imx-uart.2", NULL, uart3_clk),
	_REGISTER_CLOCK("imx-i2c.0", NULL, i2c1_clk),
	_REGISTER_CLOCK("imx-i2c.1", NULL, i2c2_clk),
	_REGISTER_CLOCK("imx-i2c_hs.0", NULL, hsi2c_clk),
	_REGISTER_CLOCK("hsi2c.0", NULL, hsi2c_serial_clk),
	_REGISTER_CLOCK("mxc-ssi.0", NULL, ssi1_clk),
	_REGISTER_CLOCK("mxc-ssi.1", NULL, ssi2_clk),
	_REGISTER_CLOCK("fsl-usb2-udc", NULL, usb_clk),
	_REGISTER_CLOCK("mxc-ehci.0", NULL, usb_clk),
	_REGISTER_CLOCK("mxc-ehci.1", NULL, usb_clk),
	_REGISTER_CLOCK("sdhci.0", NULL, esdhc1_clk),
	_REGISTER_CLOCK("sdhci.1", NULL, esdhc2_clk),
	_REGISTER_CLOCK("sdhci.2", NULL, esdhc3_clk),
	_REGISTER_CLOCK("sdhci.3", NULL, esdhc4_clk),
	_REGISTER_CLOCK("mxc_nand", NULL, nfc_clk),
	_REGISTER_CLOCK("mxc_pwm.0", NULL, pwm1_clk),
	_REGISTER_CLOCK("mxc_pwm.1", NULL, pwm2_clk),
	_REGISTER_CLOCK("mxc_vpu", NULL, vpu_clk),
	_REGISTER_CLOCK("spi_imx.0", NULL, cspi1_clk),
	_REGISTER_CLOCK("spi_imx.1", NULL, cspi2_clk),
	_REGISTER_CLOCK("spi_imx.2", NULL, cspi3_clk),
#if 0
	_REGISTER_CLOCK(NULL, "", uart_main_clk),
	_REGISTER_CLOCK(NULL, "", pwm1_clk),
	_REGISTER_CLOCK(NULL, "", pwm2_clk),
	_REGISTER_CLOCK(NULL, "", cspi_main_clk),
	_REGISTER_CLOCK(NULL, "", cspi1_clk[0]),
	_REGISTER_CLOCK(NULL, "", cspi1_clk[1]),
	_REGISTER_CLOCK(NULL, "", cspi2_clk[0]),
	_REGISTER_CLOCK(NULL, "", cspi2_clk[1]),
	_REGISTER_CLOCK(NULL, "", cspi3_clk[0]),
	_REGISTER_CLOCK(NULL, "", cspi3_clk[1]),
	_REGISTER_CLOCK(NULL, "", tmax1_clk),
	_REGISTER_CLOCK(NULL, "", tmax2_clk),
	_REGISTER_CLOCK(NULL, "", tmax3_clk),
	_REGISTER_CLOCK(NULL, "", sim_clk),
	_REGISTER_CLOCK(NULL, "", spdif_xtal_clk),
	_REGISTER_CLOCK(NULL, "", spdif0_clk[0]),
	_REGISTER_CLOCK(NULL, "", spdif0_clk[1]),
	_REGISTER_CLOCK(NULL, "", spdif1_clk[0]),
	_REGISTER_CLOCK(NULL, "", spdif1_clk[1]),
	_REGISTER_CLOCK(NULL, "", arm_axi_clk),
	_REGISTER_CLOCK(NULL, "", vpu_clk[0]),
	_REGISTER_CLOCK(NULL, "", vpu_clk[1]),
	_REGISTER_CLOCK(NULL, "", vpu_clk[2]),
	_REGISTER_CLOCK(NULL, "", lpsr_clk),
	_REGISTER_CLOCK(NULL, "", pgc_clk),
	_REGISTER_CLOCK(NULL, "", ata_clk),
	_REGISTER_CLOCK(NULL, "", owire_clk),
	_REGISTER_CLOCK(NULL, "", sahara_clk),
	_REGISTER_CLOCK(NULL, "", gpu3d_clk),
	_REGISTER_CLOCK(NULL, "", gpu2d_clk),
	_REGISTER_CLOCK(NULL, "", scc_clk),
#endif
};

#ifdef DEBUG
static const char *_clk_lookup_name(struct clk *clk)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		if (clk == lookups[i].clk) {
			return lookups[i].con_id ?
				lookups[i].con_id :
				lookups[i].dev_id;
		}
	}
	return NULL;
}
#endif

int __init mx51_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1, unsigned long ckih2)
{
	unsigned int reg;
	int i;

	osc_rate = osc;
	ckil_rate = ckil;
	ckih_rate = ckih1;
	ckih2_rate = ckih2;

	/* fixup clock parents according to the selections made by the bootloader */
	reg = __raw_readl(MXC_CCM_CSCMR1);
	switch ((reg & MXC_CCM_CSCMR1_UART_CLK_SEL_MASK) >>
		MXC_CCM_CSCMR1_UART_CLK_SEL_OFFSET) {
	case 0:
		DBG(0, "%s: Setting UART main clock parent to PLL1\n", __FUNCTION__);
		clk_set_parent(&uart_main_clk, &pll1_sw_clk);
		break;
	case 1:
		DBG(0, "%s: Setting UART main clock parent to PLL2\n", __FUNCTION__);
		clk_set_parent(&uart_main_clk, &pll2_sw_clk);
		break;
	case 2:
		DBG(0, "%s: Setting UART main clock parent to PLL3\n", __FUNCTION__);
		clk_set_parent(&uart_main_clk, &pll3_sw_clk);
		break;
	case 3:
		DBG(0, "%s: Setting UART main clock parent to LP_APM\n", __FUNCTION__);
		clk_set_parent(&uart_main_clk, &lp_apm_clk);
		break;
	}

	if (__raw_readl(MXC_CCM_CCR) & MXC_CCM_CCR_FPM_EN) {
		clk_enable(&fpm_clk);
		clk_set_parent(&lp_apm_clk, &fpm_clk);
		clk_set_parent(&pll1_main_clk, &fpm_clk);
		clk_set_parent(&pll2_main_clk, &fpm_clk);
		clk_set_parent(&pll3_main_clk, &fpm_clk);
	} else {
		clk_enable(&osc_clk);
	}

	if (!(__raw_readl(MXC_CCM_CBCDR) & MXC_CCM_CBCDR_DDR_HF_SEL))
		clk_set_parent(&ddr_clk, &axi_a_clk);

	/* register all clocks */
	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		DBG(1, "%s: Registering clk %s\n", __FUNCTION__,
			lookups[i].con_id);
		clkdev_add(&lookups[i]);
	}
#ifndef LEAVE_CLKS_ON
	/* Turn off all possible clocks */
	__raw_writel((1 << MXC_CCM_CCGR0_CG0_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG1_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG2_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG3_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG4_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG8_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG9_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG12_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG13_OFFSET) |
		(1 << MXC_CCM_CCGR0_CG14_OFFSET), MXC_CCM_CCGR0);
#ifndef CONFIG_DEBUG_LL
	__raw_writel(0, MXC_CCM_CCGR1);
#else
	/* keep UART clocks active for low level debug output */
	__raw_writel(0x3f << 3, MXC_CCM_CCGR1);
#endif
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(1 << MXC_CCM_CCGR4_CG8_OFFSET, MXC_CCM_CCGR4);
	__raw_writel((1 << MXC_CCM_CCGR5_CG2_OFFSET) |
		(3 << MXC_CCM_CCGR5_CG6_OFFSET) |
		(1 << MXC_CCM_CCGR5_CG7_OFFSET) |
		(1 << MXC_CCM_CCGR5_CG8_OFFSET) |
		(3 << MXC_CCM_CCGR5_CG9_OFFSET) |
		(1 << MXC_CCM_CCGR5_CG10_OFFSET) |
		(3 << MXC_CCM_CCGR5_CG11_OFFSET), MXC_CCM_CCGR5);
	__raw_writel(1 << MXC_CCM_CCGR6_CG4_OFFSET, MXC_CCM_CCGR6);
#endif
	/* turn on essential clocks */
	clk_enable(&emi_fast_clk);
	clk_enable(&emi_intr_clk);
	//clk_enable(&usb_ahb_clk);
	//clk_enable(&usb_phy_clk);
	//clk_enable(&usboh3_clk);
#ifdef ENABLE_ALL_CLKS
	clk_enable(&ipg_clk);
	clk_enable(&ahb_clk);
	clk_enable(&spba_clk);
	clk_enable(&tmax3_clk);
	clk_enable(&pll1_main_clk);
	clk_enable(&pll2_sw_clk);
	clk_enable(&pll3_sw_clk);
	clk_enable(&lp_apm_clk);
	clk_enable(&mcu_main_clk);
	clk_enable(&ddr_clk);
#endif
	/* Setup the LPM bypass bits */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS |
		MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS |
		MXC_CCM_CLPCR_BYPASS_RTIC_LPM_HS |
		MXC_CCM_CLPCR_BYPASS_SCC_LPM_HS |
		MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	DBG(0, "%s: OSC: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&osc_clk) / 1000000,
		clk_get_rate(&osc_clk) / 1000 % 1000);
	DBG(0, "%s: CKIH: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&ckih_clk) / 1000000,
		clk_get_rate(&ckih_clk) / 1000 % 1000);
	DBG(0, "%s: CKIH2: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&ckih2_clk) / 1000000,
		clk_get_rate(&ckih2_clk) / 1000 % 1000);
	DBG(0, "%s: CKIL: %lu.%03lukHz\n", __FUNCTION__,
		clk_get_rate(&ckil_clk) / 1000,
		clk_get_rate(&ckil_clk) % 1000);
	DBG(0, "%s: PLL1 MAIN: %lu.%03lukHz\n", __FUNCTION__,
		clk_get_rate(&pll1_main_clk) / 1000,
		clk_get_rate(&pll1_main_clk) % 1000);
	DBG(0, "%s: PLL1: %lu.%03lukHz\n", __FUNCTION__,
		clk_get_rate(&pll1_sw_clk) / 1000,
		clk_get_rate(&pll1_sw_clk) % 1000);
	DBG(0, "%s: PLL2: %lu.%03lukHz\n", __FUNCTION__,
		clk_get_rate(&pll2_sw_clk) / 1000,
		clk_get_rate(&pll2_sw_clk) % 1000);
	DBG(0, "%s: PLL3: %lu.%03lukHz\n", __FUNCTION__,
		clk_get_rate(&pll3_sw_clk) / 1000,
		clk_get_rate(&pll3_sw_clk) % 1000);
	DBG(0, "%s: AHB: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&ahb_clk) / 1000000,
		clk_get_rate(&ahb_clk) / 1000 % 1000);
	DBG(0, "%s: IPG: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&ipg_clk) / 1000000,
		clk_get_rate(&ipg_clk) / 1000 % 1000);
	DBG(0, "%s: USB: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&usboh3_clk) / 1000000,
		clk_get_rate(&usboh3_clk) / 1000 % 1000);
	DBG(0, "%s: PHY: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&usb_phy_clk) / 1000000,
		clk_get_rate(&usb_phy_clk) / 1000 % 1000);
	DBG(0, "%s: EMI: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&emi_fast_clk) / 1000000,
		clk_get_rate(&emi_fast_clk) / 1000 % 1000);
	DBG(0, "%s: DDR: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&ddr_clk) / 1000000,
		clk_get_rate(&ddr_clk) / 1000 % 1000);
	DBG(0, "%s: GPT: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&gpt_clk) / 1000000,
		clk_get_rate(&gpt_clk) / 1000 % 1000);
	DBG(0, "%s: UART: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&uart_main_clk) / 1000000,
		clk_get_rate(&uart_main_clk) / 1000 % 1000);
	DBG(0, "%s: UART1: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&uart1_clk) / 1000000,
		clk_get_rate(&uart1_clk) / 1000 % 1000);
	DBG(0, "%s: NFC: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&nfc_clk) / 1000000,
		clk_get_rate(&nfc_clk) / 1000 % 1000);
	DBG(0, "%s: SSI: %lu.%03luMHz\n", __FUNCTION__,
		clk_get_rate(&ssi1_clk) / 1000000,
		clk_get_rate(&ssi1_clk) / 1000 % 1000);

	mxc_timer_init(&gpt_clk, MX5_IO_ADDRESS(GPT1_BASE_ADDR), MXC_INT_GPT);
	return 0;
}
