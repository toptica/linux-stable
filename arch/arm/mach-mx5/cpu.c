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

/*!
 * @file mach-mx5/cpu.c
 *
 * @brief This file contains the CPU initialization code.
 *
 * @ingroup MSL_MX51
 */

#include <linux/module.h>
#include <linux/io.h>
#include <asm/cputype.h>
#include <mach/hardware.h>

#include "crm_regs.h"

static int cpu_silicon_rev = -1;
static int cpu_partnumber;

/* The offset of the Silicon revision register */
#define IIM_PREV		0x20
#define IIM_SREV		0x24

static void query_silicon_parameter(void)
{
	u32 prev, vt, srev;

	prev = __raw_readl(MX5_IO_ADDRESS(IIM_BASE_ADDR + IIM_PREV));
	srev = __raw_readl(MX5_IO_ADDRESS(IIM_BASE_ADDR + IIM_SREV));

	cpu_silicon_rev = srev & 0xff;
	vt = prev & 0x7;
	prev >>= 3;
	if (prev == 0x1e) {
		/* TO2 */
		cpu_silicon_rev += 0x20;
	}
	cpu_partnumber = (read_cpuid_id() >> 4) & 0xFFF;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 *	-EINVAL - not an i.MX51
 */
int mx51_revision(void)
{
	if (cpu_silicon_rev == -1)
		query_silicon_parameter();

	if (cpu_partnumber != 0xc08)
		return -EINVAL;

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx51_revision);

static int __init post_cpu_init(void)
{
	void __iomem *base;
	unsigned int reg;

	/* Set ALP bits to 000. Set ALP_EN bit in Arm Memory Controller reg. */
	reg = 0x8;
	__raw_writel(reg, MXC_CORTEXA8_PLAT_AMC);

	base = MX5_IO_ADDRESS(AIPS1_BASE_ADDR);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);

	base = MX5_IO_ADDRESS(AIPS2_BASE_ADDR);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);

	return 0;
}
postcore_initcall(post_cpu_init);
