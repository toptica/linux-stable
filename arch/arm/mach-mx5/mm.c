/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License.  You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/mm.h>
#include <linux/init.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

#include <mach/common.h>
#include <mach/hardware.h>

/*!
 * @file mach-mx5/mm.c
 *
 * @brief This file creates static mapping between physical to virtual memory.
 *
 * @ingroup Memory_MX51
 */

/*!
 * This structure defines the MX51 memory map.
 */
static struct map_desc mxc_io_desc[] __initdata = {
	{
		.virtual = (unsigned long)IIM_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(IIM_BASE_ADDR),
		.length = IIM_SIZE,
		.type = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)IRAM_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(IRAM_BASE_ADDR),
		.length = IRAM_SIZE,
		.type = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)DEBUG_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(DEBUG_BASE_ADDR),
		.length = DEBUG_SIZE,
		.type = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)AIPS1_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(AIPS1_BASE_ADDR),
		.length = AIPS1_SIZE,
		.type = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)SPBA0_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(SPBA0_BASE_ADDR),
		.length = SPBA0_SIZE,
		.type = MT_DEVICE,
	},
	{
		.virtual = (unsigned long)AIPS2_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(AIPS2_BASE_ADDR),
		.length = AIPS2_SIZE,
		.type = MT_DEVICE,
	},
#if 0
	{
		.virtual = (unsigned long)ROM_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(ROM_BASE_ADDR),
		.length = ROM_SIZE,
		.type = MT_DEVICE,
	},
#endif
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
static struct map_desc tzic_io_desc __initdata = {
	.virtual = (unsigned long)TZIC_BASE_ADDR_VIRT,
	.pfn = __phys_to_pfn(TZIC_TO2_BASE_ADDR),
	.length = TZIC_SIZE,
	.type = MT_DEVICE,
};

void __init mx51_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MX51);
	mxc_arch_reset_init(MX5_IO_ADDRESS(MX51_WDOG_BASE_ADDR));

	iotable_init(mxc_io_desc, ARRAY_SIZE(mxc_io_desc));
	/* This must be done after iotable_init(),
	 * since we need the IIM mapping for reading the CPU revision */
	if (mx51_revision() < MX51_CHIP_REV_2_0)
		tzic_io_desc.pfn = __phys_to_pfn(TZIC_TO1_BASE_ADDR);

	iotable_init(&tzic_io_desc, 1);
	spba_init(SPBA0_BASE_ADDR_VIRT);
}
