/*
 * linux/arch/arm/mach-mx25/pm.c
 *
 * i.MX25 Power Management Routines
 *
 * Original code for the SA11x0:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Modified for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Modified for the OMAP1510 by David Singleton:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Cleanup 2004 for OMAP1510/1610 by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * Modified for the MX27
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Modified for i.MX25
 * Copyright 2009 Lothar Wassmann <LW@KARO-electronics.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>

#include <mach/hardware.h>
#include <mach/system.h>
#include <asm/io.h>

/*
 * TODO: whatta save?
 */
#define LPCTL_MASK	(3 << 24)
#define LPCTL_WAIT	(1 << 24)
#define LPCTL_DOZE	(2 << 24)
#define LPCTL_STOP	(3 << 24)

/* called with IRQs disabled */
static void mx25_pm_lowpower(suspend_state_t state)
{
	u32 cctl;
	void __iomem *iobase = MX25_IO_ADDRESS(CCM_BASE_ADDR);

	WARN_ON(!irqs_disabled());
	cctl = __raw_readl(iobase + CCM_CCTL);

	/* WAIT and DOZE execute WFI only */
	switch (state) {
	case PM_SUSPEND_STANDBY:
		/* WAIT mode */
		cctl = (cctl & ~LPCTL_MASK) | LPCTL_WAIT;
		break;
	case PM_SUSPEND_MEM:
		/* DOZE mode */
		cctl = (cctl & ~LPCTL_MASK) | LPCTL_DOZE;
		break;
	case PM_SUSPEND_MAX:
		/* STOP mode */
		cctl = (cctl & ~LPCTL_MASK) | LPCTL_STOP;
		break;
	}
	__raw_writel(cctl, iobase + CCM_CCTL);

	/* Executes WFI */
	arch_idle();
}

static int mx25_pm_enter(suspend_state_t state)
{
	pr_debug("%s: Entering state %d\n", __FUNCTION__, state);
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
	case PM_SUSPEND_MAX:
		mx25_pm_lowpower(state);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

struct platform_suspend_ops mx25_pm_ops = {
	.enter = mx25_pm_enter,
	.valid = suspend_valid_only_mem,
};

static int __init mx25_pm_init(void)
{
	pr_debug("Power Management for Freescale i.MX25\n");
	suspend_set_ops(&mx25_pm_ops);

	return 0;
}
device_initcall(mx25_pm_init);
