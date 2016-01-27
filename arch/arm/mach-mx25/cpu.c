/*
 * Copyright (C) 2009  Lothar Wassmann <LW@KARO-electronics.de>
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
 * This code is based on a patch provided by Freescale support to enable
 * user mode access to the i.MX25 SOC registers.
 * Without this mmap'ed access from userspace will not work.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <asm/io.h>

/*
 * enable user mode access to SOC registers
 */
static int __init mx25_post_cpu_init(void)
{
	void __iomem *base;
	unsigned long reg;

	base = MX25_IO_ADDRESS(MX25_AIPS1_BASE_ADDR);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);

	base = MX25_IO_ADDRESS(MX25_AIPS2_BASE_ADDR);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);

	return 0;
}
postcore_initcall(mx25_post_cpu_init);
