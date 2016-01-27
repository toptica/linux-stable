/*
 *  Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/errno.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/common.h>
#include <mach/hardware.h>

/*
 *****************************************
 * TZIC Registers                        *
 *****************************************
 */
void __iomem *tzic_base;

#define TZIC_INTCNTL		(tzic_base + 0x0000)	/* control register */
#define TZIC_INTTYPE		(tzic_base + 0x0004)	/* Controller type register */
#define TZIC_IMPID		(tzic_base + 0x0008)	/* Distributor Implementer Identification Register */
#define TZIC_PRIOMASK		(tzic_base + 0x000C)	/* Priority Mask Reg */
#define TZIC_SYNCCTRL		(tzic_base + 0x0010)	/* Synchronizer Control register */
#define TZIC_DSMINT		(tzic_base + 0x0014)	/* DSM interrupt Holdoffregister */
#define TZIC_INTSEC0		(tzic_base + 0x0080)	/* interrupt security register 0 */
#define TZIC_ENSET0		(tzic_base + 0x0100)	/* Enable Set Register 0 */
#define TZIC_ENCLEAR0		(tzic_base + 0x0180)	/* Enable Clear Register 0 */
#define TZIC_SRCSET0		(tzic_base + 0x0200)	/* Source Set Register 0 */
#define TZIC_SRCCLAR0		(tzic_base + 0x0280)	/* Source Clear Register 0 */
#define TZIC_PRIORITY0		(tzic_base + 0x0400)	/* Priority Register 0 */
#define TZIC_PND0		(tzic_base + 0x0D00)	/* Pending Register 0 */
#define TZIC_HIPND0		(tzic_base + 0x0D80)	/* High Priority Pending Register */
#define TZIC_WAKEUP0		(tzic_base + 0x0E00)	/* Wakeup Config Register */
#define TZIC_SWINT		(tzic_base + 0x0F00)	/* Software Interrupt Rigger Register */
#define TZIC_ID0		(tzic_base + 0x0FD0)	/* Indentification Register 0 */

/*!
 * Disable interrupt number "irq" in the TZIC
 *
 * @param  irq          interrupt source number
 */
static void mxc_mask_irq(unsigned int irq)
{
	int index, off;

	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, TZIC_ENCLEAR0 + (index << 2));
}

/*!
 * Enable interrupt number "irq" in the TZIC
 *
 * @param  irq          interrupt source number
 */
static void mxc_unmask_irq(unsigned int irq)
{
	int index, off;

	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, TZIC_ENSET0 + (index << 2));
}

#ifdef CONFIG_PM
static unsigned int wakeup_intr[4];
static unsigned int saved_wakeup[4];

/*!
 * Set interrupt number "irq" in the TZIC as a wake-up source.
 *
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 *			disble as wake-up if equal to zero
 *
 * @return       This function returns 0 on success.
 */
static int mxc_set_wake_irq(unsigned int irq, unsigned int enable)
{
	unsigned int index, off;

	index = irq >> 5;
	off = irq & 0x1F;

	if (index > 3)
		return -1;

	if (enable) {
		wakeup_intr[index] |= 1 << off;
	} else {
		wakeup_intr[index] &= ~(1 << off);
	}
	return 0;
}
#else
#define mxc_set_wake_irq	NULL
#endif

static struct irq_chip mxc_tzic_chip = {
	.name = "MXC_TZIC",
	.ack = mxc_mask_irq,
	.mask = mxc_mask_irq,
	.unmask = mxc_unmask_irq,
	.set_wake = mxc_set_wake_irq,
};

#ifdef CONFIG_PM
/* This function puts the TZIC in low-power mode/state.
 * All interrupts that are enabled are first saved.
 * Only those interrupts which are registered as a wake source by calling
 * enable_irq_wake are enabled. All other interrupts are disabled.
 */
static int mxc_tzic_suspend(struct sys_device *dev, pm_message_t mesg)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(saved_wakeup); i++) {
		saved_wakeup[i] = __raw_readl(TZIC_WAKEUP0 + i * 4);
	}
	ret = tzic_enable_wake(0);
	return ret;
}

/* This function brings the TZIC back from low-power state.
 * All interrupts that were enabled prior to suspend are re-enabled.
 */
static int mxc_tzic_resume(struct sys_device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(saved_wakeup); i++) {
		__raw_writel(saved_wakeup[i], TZIC_WAKEUP0 + i * 4);
	}
	return 0;
}

#else
#define mxc_tzic_suspend  NULL
#define mxc_tzic_resume   NULL
#endif				/* CONFIG_PM */

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct sysdev_class mxc_tzic_sysclass = {
	.name = "mxc_tzic",
	.suspend = mxc_tzic_suspend,
	.resume = mxc_tzic_resume,
};

static struct sys_device mxc_tzic_device = {
	.cls = &mxc_tzic_sysclass,
};

/*
 * This function initializes the TZIC hardware and disables all the
 * interrupts. It registers the interrupt enable and disable functions
 * to the kernel for each interrupt source.
 */
void __init mxc_init_irq(void __iomem *base)
{
	int i;

	tzic_base = base;

	i = __raw_readl(TZIC_INTCNTL);

	__raw_writel(0x80010001, TZIC_INTCNTL);
	i = __raw_readl(TZIC_INTCNTL);
	__raw_writel(0x1f, TZIC_PRIOMASK);
	i = __raw_readl(TZIC_PRIOMASK);
	__raw_writel(0x02, TZIC_SYNCCTRL);
	i = __raw_readl(TZIC_SYNCCTRL);
	for (i = 0; i < 4; i++) {
		__raw_writel(0xFFFFFFFF, TZIC_INTSEC0 + i * 4);
	}
	/* disable all interrupts */
	for (i = 0; i < 4; i++) {
		__raw_writel(0xFFFFFFFF, TZIC_ENCLEAR0 + i * 4);
	}

	/* all IRQ no FIQ Warning :: No selection */

	for (i = 0; i < MXC_MAX_INT_LINES; i++) {
		set_irq_chip(i, &mxc_tzic_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	/* init architectures chained interrupt handler */
	mxc_register_gpios();

	printk(KERN_INFO "MXC IRQ initialized\n");
}

/*
 * enable wakeup interrupt
 *
 * @param is_idle		1 if called in idle loop (enset registers);
 *				0 to be used when called from low power entry
 * @return			0 if successful; non-zero otherwise
 */
int tzic_enable_wake(int is_idle)
{
	unsigned int i, v;

	__raw_writel(1, TZIC_DSMINT);
	if (unlikely(__raw_readl(TZIC_DSMINT) == 0))
		return -EAGAIN;

	if (likely(is_idle)) {
		for (i = 0; i < 4; i++) {
			v = __raw_readl(TZIC_ENSET0 + i * 4);
			__raw_writel(v, TZIC_WAKEUP0 + i * 4);
		}
	} else {
		for (i = 0; i < 4; i++) {
			v = wakeup_intr[i];
			__raw_writel(v, TZIC_WAKEUP0 + i * 4);
		}
	}
	return 0;
}

extern int __init mxc_gpio_sysinit(void);

/* This function registers TZIC hardware as a system device */
static int __init mxc_tzic_sysinit(void)
{
	int ret;

	ret = sysdev_class_register(&mxc_tzic_sysclass);
	if (ret)
		return ret;
	ret = sysdev_register(&mxc_tzic_device);
	if (ret)
		return ret;

	ret = mxc_gpio_sysinit();
	return ret;
}
core_initcall(mxc_tzic_sysinit);
