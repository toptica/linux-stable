/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 * Copyright 2006-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2009 Ilya Yanok, Emcraft Systems Ltd, yanok@emcraft.com
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/proc-fns.h>
#include <asm/system.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/system.h>

static void __iomem *wdog_base;
static struct clk *wdt_clk;

/*
 * Reset the system. It is called by machine_restart().
 */
void arch_reset(char mode, const char *cmd)
{
	unsigned int wcr_enable;

	if (cpu_is_mx1())
		wcr_enable = 1 << 0;
	else
		wcr_enable = 0;

	/* Assert SRS signal */
	__raw_writew(wcr_enable, wdog_base);

	/* wait for reset to assert... */
	mdelay(500);
	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}

void mxc_arch_reset_init(void __iomem *base)
{
	wdog_base = base;
}

static int __init mxc_wdt_init(void)
{
	if (cpu_is_mx1() || cpu_is_mx5())
		return 0;

	wdt_clk = clk_get_sys("imx-wdt.0", NULL);
	if (IS_ERR(wdt_clk)) {
		printk(KERN_CRIT "%s: Failed to get imx-wdt.0 clk: %ld; system reset may not work\n",
		       __FUNCTION__, PTR_ERR(wdt_clk));
		return PTR_ERR(wdt_clk);
	}
	return clk_enable(wdt_clk);
}
core_initcall(mxc_wdt_init);
