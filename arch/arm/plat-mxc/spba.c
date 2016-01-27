/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/spba.h>

/*!
 * @file plat-mxc/spba.c
 *
 * @brief This file contains the SPBA API implementation details.
 *
 * @ingroup SPBA
 */

static DEFINE_SPINLOCK(spba_lock);

#define SPBA_MASTER_MIN			1
#define SPBA_MASTER_MAX			7

/*!
 * the base addresses for the SPBA modules
 */
static void __iomem *spba_base;

/*!
 * SPBA clock
 */
static struct clk *spba_clk;
/*
 * This function allows the three masters (A, B, C) to take ownership of a
 * shared peripheral.
 *
 */
int spba_take_ownership(int mod, int master)
{
	unsigned long spba_flags;
	int ret = -EINVAL;

	if (master < SPBA_MASTER_MIN || master > SPBA_MASTER_MAX) {
		printk(KERN_EMERG "%s invalid master: %d\n", __FUNCTION__, master);
		return -EINVAL;
	}

	if (spba_clk == NULL) {
		spba_clk = clk_get(NULL, "spba_clk");
		if (IS_ERR(spba_clk)) {
			ret = PTR_ERR(spba_clk);
			printk(KERN_EMERG "Failed to get SPBA clock: %d\n", ret);
			spba_clk = NULL;
			return ret;
		}
	}

	clk_enable(spba_clk);

	spin_lock_irqsave(&spba_lock, spba_flags);
	__raw_writel(master, spba_base + mod);

	if ((__raw_readl(spba_base + mod) & MXC_SPBA_RAR_MASK) == master) {
		ret = 0;
	}

	spin_unlock_irqrestore(&spba_lock, spba_flags);

	clk_disable(spba_clk);
	return ret;
}
EXPORT_SYMBOL(spba_take_ownership);

/*
 * This function releases the ownership for a shared peripheral.
 */
int spba_rel_ownership(int mod, int master)
{
	int ret = 0;
	unsigned long spba_flags;
	unsigned long rar;

	if (master < SPBA_MASTER_MIN || master > SPBA_MASTER_MAX) {
		printk(KERN_EMERG "%s invalid master: %d\n", __FUNCTION__, master);
		return -EINVAL;
	}

	if (spba_clk == NULL) {
		spba_clk = clk_get(NULL, "spba_clk");
		if (IS_ERR(spba_clk)) {
			ret = PTR_ERR(spba_clk);
			printk(KERN_EMERG "Failed to get SPBA clock: %d\n", ret);
			spba_clk = NULL;
			return ret;
		}
	}

	clk_enable(spba_clk);

	if ((__raw_readl(spba_base + mod) & master) == 0) {
		clk_disable(spba_clk);
		return 0;	/* does not own it */
	}

	spin_lock_irqsave(&spba_lock, spba_flags);

	/* Since only the last 3 bits are writeable, doesn't need to mask off
	   bits 31-3 */
	rar = __raw_readl(spba_base + mod) & ~master;
	__raw_writel(rar, spba_base + mod);

	if ((__raw_readl(spba_base + mod) & master) != 0) {
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&spba_lock, spba_flags);
	clk_disable(spba_clk);

	return ret;
}
EXPORT_SYMBOL(spba_rel_ownership);

void __init spba_init(void __iomem *base_addr)
{
	spba_base = base_addr;
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SPBA");
MODULE_LICENSE("GPL");
