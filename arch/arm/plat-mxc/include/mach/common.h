/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_COMMON_H__
#define __ASM_ARCH_MXC_COMMON_H__

struct platform_device;
struct clk;

extern void mx1_map_io(void);
extern void mx21_map_io(void);
extern void mx25_map_io(void);
extern void mx27_map_io(void);
extern void mx31_map_io(void);
extern void mx35_map_io(void);
extern void mx51_map_io(void);

extern void mxc_init_irq(void __iomem *);

extern void mx1_init_irq(void);
extern void mx21_init_irq(void);
extern void mx25_init_irq(void);
extern void mx27_init_irq(void);
extern void mx31_init_irq(void);
extern void mx35_init_irq(void);

extern void mxc_timer_init(struct clk *timer_clk, void __iomem *, int);

extern int mx1_clocks_init(unsigned long fref);
extern int mx21_clocks_init(unsigned long lref, unsigned long fref);
extern int mx25_clocks_init(unsigned long fref);
extern int mx27_clocks_init(unsigned long fref);
extern int mx31_clocks_init(unsigned long fref);
extern int mx35_clocks_init(void);
extern int __init mx51_clocks_init(unsigned long ckil, unsigned long osc,
				unsigned long ckih1, unsigned long ckih2);
extern int mxc_register_gpios(void);
extern int __init mx1_register_gpios(void);
extern int __init mx2_register_gpios(void);
extern int __init mx25_register_gpios(void);
extern int __init mx3_register_gpios(void);

extern int mxc_register_device(struct platform_device *pdev, void *data);
extern void mxc_set_cpu_type(unsigned int type);
extern void mxc_arch_reset_init(void __iomem *);

extern void __init spba_init(void __iomem *base);

extern int tzic_enable_wake(int is_idle);
//extern void cpu_do_suspend_workaround(void __iomem *sdclk_iomux_addr);
extern struct {
	void *addr;
	unsigned long size;
} cpu_do_suspend_workaround;

extern void cpu_cortexa8_do_idle(void __iomem *);
extern void mxc_cpu_lp_set(int mode);

extern void mxc_pg_enable(struct platform_device *pdev);
extern void mxc_pg_disable(struct platform_device *pdev);

#endif
