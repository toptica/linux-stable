/*
 *  Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/mach/map.h>

#include "crm_regs.h"

#include "karo.h"

static struct clk *gpc_dvfs_clk;
#if 0
extern void cpu_do_suspend_workaround(u32 sdclk_iomux_addr);
extern void cpu_cortexa8_do_idle(void __iomem *);
#endif

extern int iram_ready;
static void *suspend_iram_base;
static void (*suspend_in_iram)(void __iomem *sdclk_iomux_addr);

static int mx51_suspend_enter(suspend_state_t state)
{
	void __iomem *sdclk_iomux_addr = MX5_IO_ADDRESS(IOMUXC_BASE_ADDR + 0x4b8);

	DBG(0, "%s: \n", __FUNCTION__);
#if 0
	switch (state) {
#ifdef CONFIG_CPU_IDLE
	case PM_SUSPEND_MEM:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
		break;
#endif
	default:
		DBG(0, "%s: Invalid suspend mode: %d\n", __FUNCTION__, state);
		return -EINVAL;
	}
#endif
	if (tzic_enable_wake(0) != 0) {
		DBG(0, "%s: Aborting suspend due to interrupt\n", __FUNCTION__);
		return -EAGAIN;
	}
	if (state == PM_SUSPEND_MEM) {
		local_flush_tlb_all();
		flush_cache_all();

		DBG(0, "%s: Calling suspend code in IRAM\n", __FUNCTION__);
		/* Run the suspend code from iRAM. */
		suspend_in_iram(sdclk_iomux_addr);

		/* clear the EMPGC0/1 bits */
		__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
		__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);
	} else {
		if (mx51_revision() < MX51_CHIP_REV_2_0) {
			/* do cpu_idle_workaround */
			u32 l2_iram_addr = IDLE_IRAM_BASE_ADDR;

			if (!iram_ready)
				return 0;
			if (l2_iram_addr > 0x1FFE8000)
				cpu_cortexa8_do_idle(MX5_IO_ADDRESS(l2_iram_addr));
		} else {
			cpu_do_idle();
		}
	}

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx51_suspend_prepare(void)
{
	DBG(0, "%s: \n", __FUNCTION__);
	gpc_dvfs_clk = clk_get(NULL, "gpc_dvfs");
	if (IS_ERR(gpc_dvfs_clk)) {
		int ret = PTR_ERR(gpc_dvfs_clk);
		DBG(0, "%s: Failed to get gpc_dvfs clock\n", __FUNCTION__);
		gpc_dvfs_clk = NULL;
		return ret;
	}

	/* Copy suspend routine into iRAM */
	DBG(0, "%s: Copying suspend code from %p..%p to %p\n",
		__FUNCTION__, cpu_do_suspend_workaround.addr,
		cpu_do_suspend_workaround.addr + cpu_do_suspend_workaround.size - 1,
		suspend_iram_base);
	memcpy(suspend_iram_base, cpu_do_suspend_workaround.addr,
		cpu_do_suspend_workaround.size);

	suspend_in_iram = suspend_iram_base;

	/* gpc clock is needed for SRPG */
	return clk_enable(gpc_dvfs_clk);
}

/*
 * Called before devices are re-setup.
 */
static void mx51_suspend_finish(void)
{
	DBG(0, "%s: \n", __FUNCTION__);
	BUG_ON(gpc_dvfs_clk == NULL);
	clk_disable(gpc_dvfs_clk);
	clk_put(gpc_dvfs_clk);
	gpc_dvfs_clk = NULL;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx51_suspend_end(void)
{
	DBG(0, "%s: \n", __FUNCTION__);
}

static int mx51_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx51_suspend_ops = {
	.valid = mx51_pm_valid,
	.enter = mx51_suspend_enter,
	.prepare = mx51_suspend_prepare,
	.finish = mx51_suspend_finish,
	.end = mx51_suspend_end,
};

static int __init mx51_pm_init(void)
{
	pr_info("Static Power Management for Freescale i.MX51\n");

	suspend_set_ops(&mx51_suspend_ops);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	suspend_iram_base = __arm_ioremap(SUSPEND_IRAM_BASE_ADDR, SZ_4K,
					MT_HIGH_VECTORS);
	if (suspend_iram_base == NULL) {
		printk(KERN_ERR "Failed to map internal RAM for suspend\n");
		return -ENOMEM;
	}
#if 1
	DBG(0, "%s: Copying suspend code from %p..%p to %p\n",
		__FUNCTION__, cpu_do_suspend_workaround.addr,
		cpu_do_suspend_workaround.addr + cpu_do_suspend_workaround.size - 1,
		suspend_iram_base);
	memcpy(suspend_iram_base, cpu_do_suspend_workaround.addr,
		cpu_do_suspend_workaround.size);
#endif
	return 0;
}
device_initcall(mx51_pm_init);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("PM driver");
MODULE_LICENSE("GPL");

