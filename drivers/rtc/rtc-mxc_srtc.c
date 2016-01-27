/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * Implementation based on rtc-ds1553.c
 *
 * i.MX Secure Real Time Clock (SRTC) Driver
 */


#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/uaccess.h>

#include <mach/hardware.h>

#ifdef DEBUG
static int debug = 1;
#define dbg_lvl(n)	((n) < debug)
#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
module_param(debug, int, S_IWUSR | S_IRUGO);
#else
static int debug;
#define dbg_lvl(n)	0
#define DBG(lvl, fmt...)	do { } while (0)
module_param(debug, int, 0);
#endif

#define SRTC_LPPDR_INIT       0x41736166	/* init for glitch detect */

#define SRTC_LPCR_SWR_LP      (1 << 0)	/* lp software reset */
#define SRTC_LPCR_EN_LP	      (1 << 3)	/* lp enable */
#define SRTC_LPCR_WAE	      (1 << 4)	/* lp wakeup alarm enable */
#define SRTC_LPCR_SAE	      (1 << 5)	/* lp security alarm enable */
#define SRTC_LPCR_SI	      (1 << 6)	/* lp security interrupt enable */
#define SRTC_LPCR_ALP	      (1 << 7)	/* lp alarm flag */
#define SRTC_LPCR_LTC	      (1 << 8)	/* lp lock time counter */
#define SRTC_LPCR_LMC	      (1 << 9)	/* lp lock monotonic counter */
#define SRTC_LPCR_SV	      (1 << 10)	/* lp security violation */
#define SRTC_LPCR_NSA	      (1 << 11)	/* lp non secure access */
#define SRTC_LPCR_NVEIE	      (1 << 12)	/* lp non valid state exit int en */
#define SRTC_LPCR_IEIE	      (1 << 13)	/* lp init state exit int enable */
#define SRTC_LPCR_NVE	      (1 << 14)	/* lp non valid state exit bit */
#define SRTC_LPCR_IE	      (1 << 15)	/* lp init state exit bit */

#define SRTC_LPCR_ALL_INT_EN (SRTC_LPCR_WAE | SRTC_LPCR_SAE | \
			      SRTC_LPCR_SI | SRTC_LPCR_ALP | \
			      SRTC_LPCR_NVEIE | SRTC_LPCR_IEIE)

#define SRTC_LPSR_TRI	      (1 << 0)	/* lp time read invalidate */
#define SRTC_LPSR_PGD	      (1 << 1)	/* lp power supply glitc detected */
#define SRTC_LPSR_CTD	      (1 << 2)	/* lp clock tampering detected */
#define SRTC_LPSR_ALP	      (1 << 3)	/* lp alarm flag */
#define SRTC_LPSR_MR	      (1 << 4)	/* lp monotonic counter rollover */
#define SRTC_LPSR_TR	      (1 << 5)	/* lp time rollover */
#define SRTC_LPSR_EAD	      (1 << 6)	/* lp external alarm detected */
#define SRTC_LPSR_IT0	      (1 << 7)	/* lp IIM throttle */
#define SRTC_LPSR_IT1	      (1 << 8)
#define SRTC_LPSR_IT2	      (1 << 9)
#define SRTC_LPSR_SM0	      (1 << 10)	/* lp security mode */
#define SRTC_LPSR_SM1	      (1 << 11)
#define SRTC_LPSR_STATE_LP0   (1 << 12)	/* lp state */
#define SRTC_LPSR_STATE_LP1   (1 << 13)
#define SRTC_LPSR_NVES	      (1 << 14)	/* lp non-valid state exit status */
#define SRTC_LPSR_IES	      (1 << 15)	/* lp init state exit status */

#define MAX_PIE_NUM	15
#define MAX_PIE_FREQ	32768
#define MIN_PIE_FREQ	1

#define SRTC_PI0	 (1 << 0)
#define SRTC_PI1	 (1 << 1)
#define SRTC_PI2	 (1 << 2)
#define SRTC_PI3	 (1 << 3)
#define SRTC_PI4	 (1 << 4)
#define SRTC_PI5	 (1 << 5)
#define SRTC_PI6	 (1 << 6)
#define SRTC_PI7	 (1 << 7)
#define SRTC_PI8	 (1 << 8)
#define SRTC_PI9	 (1 << 9)
#define SRTC_PI10	 (1 << 10)
#define SRTC_PI11	 (1 << 11)
#define SRTC_PI12	 (1 << 12)
#define SRTC_PI13	 (1 << 13)
#define SRTC_PI14	 (1 << 14)
#define SRTC_PI15	 (1 << 15)

#define PIT_ALL_ON	(SRTC_PI1 | SRTC_PI2 | SRTC_PI3 | \
			SRTC_PI4 | SRTC_PI5 | SRTC_PI6 | SRTC_PI7 | \
			SRTC_PI8 | SRTC_PI9 | SRTC_PI10 | SRTC_PI11 | \
			SRTC_PI12 | SRTC_PI13 | SRTC_PI14 | SRTC_PI15)

#define SRTC_SWR_HP	 (1 << 0)	/* hp software reset */
#define SRTC_EN_HP	 (1 << 3)	/* hp enable */
#define SRTC_TS		 (1 << 4)	/* time syncronize hp with lp */

#define SRTC_IE_AHP	 (1 << 16)	/* Alarm HP Interrupt Enable bit */
#define SRTC_IE_WDHP	 (1 << 18)	/* Write Done HP Interrupt Enable bit */
#define SRTC_IE_WDLP	 (1 << 19)	/* Write Done LP Interrupt Enable bit */

#define SRTC_ISR_AHP	 (1 << 16)	/* interrupt status: alarm hp */
#define SRTC_ISR_WDHP	 (1 << 18)	/* interrupt status: write done hp */
#define SRTC_ISR_WDLP	 (1 << 19)	/* interrupt status: write done lp */
#define SRTC_ISR_WPHP	 (1 << 20)	/* interrupt status: write pending hp */
#define SRTC_ISR_WPLP	 (1 << 21)	/* interrupt status: write pending lp */

#define SRTC_LPSCMR	0x00	/* LP Secure Counter MSB Reg */
#define SRTC_LPSCLR	0x04	/* LP Secure Counter LSB Reg */
#define SRTC_LPSAR	0x08	/* LP Secure Alarm Reg */
#define SRTC_LPSMCR	0x0C	/* LP Secure Monotonic Counter Reg */
#define SRTC_LPCR	0x10	/* LP Control Reg */
#define SRTC_LPSR	0x14	/* LP Status Reg */
#define SRTC_LPPDR	0x18	/* LP Power Supply Glitch Detector Reg */
#define SRTC_LPGR	0x1C	/* LP General Purpose Reg */
#define SRTC_HPCMR	0x20	/* HP Counter MSB Reg */
#define SRTC_HPCLR	0x24	/* HP Counter LSB Reg */
#define SRTC_HPAMR	0x28	/* HP Alarm MSB Reg */
#define SRTC_HPALR	0x2C	/* HP Alarm LSB Reg */
#define SRTC_HPCR	0x30	/* HP Control Reg */
#define SRTC_HPISR	0x34	/* HP Interrupt Status Reg */
#define SRTC_HPIENR	0x38	/* HP Interrupt Enable Reg */

#define SRTC_SECMODE_MASK	0x3	/* the mask of SRTC security mode */
#define SRTC_SECMODE_LOW	0x0	/* Low Security */
#define SRTC_SECMODE_MED	0x1	/* Medium Security */
#define SRTC_SECMODE_HIGH	0x2	/* High Security */
#define SRTC_SECMODE_RESERVED	0x3	/* Reserved */

struct rtc_drv_data {
	struct rtc_device *rtc;
	struct clk *clk;
	spinlock_t lock;
	void __iomem *ioaddr;
	struct resource *res;
	int irq;
	unsigned long opened;
};

/*!
 * This function does write synchronization for writes to the lp srtc block.
 * To take care of the asynchronous CKIL clock, all writes from the IP domain
 * will be synchronized to the CKIL domain.
 */
static inline void rtc_write_sync_lp(void __iomem *ioaddr)
{
	unsigned long count;

	/* Wait for 3 CKIL cycles */
	count = __raw_readl(ioaddr + SRTC_LPSCLR);
	while (time_before((unsigned long)__raw_readl(ioaddr + SRTC_LPSCLR),
				count + 3));
}

/*!
 * This function updates the RTC alarm registers and then clears all the
 * interrupt status bits.
 *
 * @param  alrm         the new alarm value to be updated in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int rtc_update_alarm(struct device *dev, struct rtc_time *alrm)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;

	now = __raw_readl(ioaddr + SRTC_LPSCMR);
	rtc_time_to_tm(now, &now_tm);

	alarm_tm.tm_year = now_tm.tm_year;
	alarm_tm.tm_mon = now_tm.tm_mon;
	alarm_tm.tm_mday = now_tm.tm_mday;

	alarm_tm.tm_hour = alrm->tm_hour >= 0 ? alrm->tm_hour : now_tm.tm_hour;
	alarm_tm.tm_min = alrm->tm_min >= 0 ? alrm->tm_min : now_tm.tm_min;
	alarm_tm.tm_sec = alrm->tm_sec >= 0 ? alrm->tm_sec : now_tm.tm_sec;

	ret = rtc_tm_to_time(&now_tm, &now);
	if (ret == 0)
		ret = rtc_tm_to_time(&alarm_tm, &time);
	if (ret)
		return ret;
	if (time < now) {
		time += 60 * 60 * 24;
		rtc_time_to_tm(time, &alarm_tm);
	}
	ret = rtc_tm_to_time(&alarm_tm, &time);
	if (ret) {
		return ret;
	}
	__raw_writel(time, ioaddr + SRTC_LPSAR);
	DBG(0, "%s: Wrote %08lx to LPSAR\n", __FUNCTION__, time);

	/* clear alarm interrupt status bit */
	__raw_writel(SRTC_LPSR_ALP, ioaddr + SRTC_LPSR);
	rtc_write_sync_lp(ioaddr);

	return ret;
}

/*!
 * This function is the RTC interrupt service routine.
 *
 * @param  irq          RTC IRQ number
 * @param  dev_id       device ID which is not used
 *
 * @return IRQ_HANDLED as defined in the include/linux/interrupt.h file.
 */
static irqreturn_t mxc_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_drv_data *mxc_rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;
	u32 lp_status, lp_cr;
	u32 events = 0;

	lp_status = __raw_readl(ioaddr + SRTC_LPSR);
	lp_cr = __raw_readl(ioaddr + SRTC_LPCR);

	/* update irq data & counter */
	if (lp_status & SRTC_LPSR_ALP) {
		if (lp_cr & SRTC_LPCR_ALP)
			events |= (RTC_AF | RTC_IRQF);

		/* disable further lp alarm interrupts */
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);

		/* Update interrupt enables */
		__raw_writel(lp_cr, ioaddr + SRTC_LPCR);
	}

	/* clear interrupt status */
	__raw_writel(lp_status, ioaddr + SRTC_LPSR);

	if (events) {
		rtc_update_irq(mxc_rtc->rtc, 1, events);
	}
	return IRQ_HANDLED;
}

/*!
 * This function is used to open the RTC driver.
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_open(struct device *dev)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);

	if (test_and_set_bit(1, &mxc_rtc->opened))
		return -EBUSY;
	if (mxc_rtc->clk)
		clk_enable(mxc_rtc->clk);
	return 0;
}

/*!
 * clear all interrupts and release the IRQ
 */
static void mxc_rtc_release(struct device *dev)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;
	unsigned long flags;

	spin_lock_irqsave(&mxc_rtc->lock, flags);

	/* Disable all rtc interrupts */
	__raw_writel(__raw_readl(ioaddr + SRTC_LPCR) & ~SRTC_LPCR_ALL_INT_EN,
		     ioaddr + SRTC_LPCR);

	/* Clear all interrupt status */
	__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);

	mxc_rtc->opened = 0;

	spin_unlock_irqrestore(&mxc_rtc->lock, flags);

	if (mxc_rtc->clk)
		clk_disable(mxc_rtc->clk);
}

/*!
 * This function is used to support some ioctl calls directly.
 * Other ioctl calls are supported indirectly through the
 * arm/common/rtctime.c file.
 *
 * @param  cmd          ioctl command as defined in include/linux/rtc.h
 * @param  arg          value for the ioctl command
 *
 * @return  0 if successful or negative value otherwise.
 */
static int mxc_rtc_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;
	unsigned long flags;
	u32 lp_cr;

	switch (cmd) {
	case RTC_AIE_OFF:
		spin_lock_irqsave(&mxc_rtc->lock, flags);
		lp_cr = __raw_readl(ioaddr + SRTC_LPCR);
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);
		__raw_writel(lp_cr, ioaddr + SRTC_LPCR);
		spin_unlock_irqrestore(&mxc_rtc->lock, flags);
		return 0;

	case RTC_AIE_ON:
		spin_lock_irqsave(&mxc_rtc->lock, flags);
		lp_cr = __raw_readl(ioaddr + SRTC_LPCR);
		lp_cr |= SRTC_LPCR_ALP | SRTC_LPCR_WAE;
		__raw_writel(lp_cr, ioaddr + SRTC_LPCR);
		spin_unlock_irqrestore(&mxc_rtc->lock, flags);
		return 0;
	}

	return -ENOIOCTLCMD;
}

/*!
 * This function reads the current RTC time into tm in Gregorian date.
 *
 * @param  tm           contains the RTC time value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;

	rtc_time_to_tm(__raw_readl(ioaddr + SRTC_LPSCMR), tm);
	return 0;
}

/*!
 * This function sets the internal RTC time based on tm in Gregorian date.
 *
 * @param  tm           the time value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;
	unsigned long time;
	int ret;

	ret = rtc_tm_to_time(tm, &time);
	if (ret != 0)
		return ret;

	__raw_writel(time, ioaddr + SRTC_LPSCMR);
	rtc_write_sync_lp(ioaddr);

	return 0;
}

/*!
 * This function reads the current alarm value into the passed in \b alrm
 * argument. It updates the \b alrm's pending field value based on the whether
 * an alarm interrupt occurs or not.
 *
 * @param  alrm         contains the RTC alarm value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;
	unsigned long time;

	time = __raw_readl(ioaddr + SRTC_LPSAR);
	rtc_time_to_tm(time, &alrm->time);
	DBG(0, "%s: Read %08lx from LPSAR\n", __FUNCTION__, time);
	alrm->enabled = !!(__raw_readl(ioaddr + SRTC_LPCR) & SRTC_LPCR_WAE);
	alrm->pending = !!(__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_ALP);

	return 0;
}

/*!
 * This function sets the RTC alarm based on passed in alrm.
 *
 * @param  alrm         the alarm value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;
	unsigned long flags;
	u32 lp_cr;
	int ret;

	if (rtc_valid_tm(&alrm->time)) {
		if (alrm->time.tm_sec > 59 ||
		    alrm->time.tm_hour > 23 || alrm->time.tm_min > 59) {
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&mxc_rtc->lock, flags);
	lp_cr = __raw_readl(ioaddr + SRTC_LPCR);

	ret = rtc_update_alarm(dev, &alrm->time);
	if (ret)
		goto out;

	if (alrm->enabled)
		lp_cr |= (SRTC_LPCR_ALP | SRTC_LPCR_WAE);
	else
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);

	__raw_writel(lp_cr, ioaddr + SRTC_LPCR);

out:
	spin_unlock_irqrestore(&mxc_rtc->lock, flags);
	rtc_write_sync_lp(ioaddr);
	return ret;
}

/*!
 * This function is used to provide the content for the /proc/driver/rtc
 * file.
 *
 * @param  seq  buffer to hold the information that the driver wants to write
 *
 * @return  The number of bytes written into the rtc file.
 */
static int mxc_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);
	void __iomem *ioaddr = mxc_rtc->ioaddr;

	clk_enable(mxc_rtc->clk);
	seq_printf(seq, "alarm_IRQ\t: %s\n",
		((__raw_readl(ioaddr + SRTC_LPCR)) & SRTC_LPCR_ALP) ?
		"yes" : "no");
	clk_disable(mxc_rtc->clk);

	return 0;
}

/*!
 * The RTC driver structure
 */
static struct rtc_class_ops mxc_rtc_ops = {
	.open = mxc_rtc_open,
	.release = mxc_rtc_release,
	.ioctl = mxc_rtc_ioctl,
	.read_time = mxc_rtc_read_time,
	.set_time = mxc_rtc_set_time,
	.read_alarm = mxc_rtc_read_alarm,
	.set_alarm = mxc_rtc_set_alarm,
	.proc = mxc_rtc_proc,
};

static inline int mxc_rtc_waitfor(void __iomem *ioaddr, unsigned int reg,
				unsigned int mask, int timeout)
{
	while (!(__raw_readl(ioaddr + reg) & mask)) {
		if (timeout-- <= 0) {
			DBG(0, "%s: timeout waiting for %08x in reg %02x\n",
				__FUNCTION__, mask, reg);
			return -ETIME;
		}
		udelay(1);
	}
	return 0;
}

static int __devinit mxc_rtc_init_secmode(struct platform_device *pdev, void __iomem *ioaddr)
{
	int ret;
	void __iomem *secmode_addr;
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	struct clk *clk;

	if (res == NULL) {
		return -ENODEV;
	}
	secmode_addr = ioremap(res->start, resource_size(res));
	if (secmode_addr == NULL) {
		return -ENOMEM;
	}

	clk = clk_get(NULL, "iim");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto unmap;
	}
	ret = clk_enable(clk);
	if (ret) {
		goto put_clk;
	}
	/* Check SRTC security mode */
	if (((__raw_readl(secmode_addr) & SRTC_SECMODE_MASK) ==
			SRTC_SECMODE_LOW) &&
		(mx51_revision() == MX51_CHIP_REV_1_0)) {
		/* Workaround for MX51 TO1 due to inaccurate CKIL clock */
		__raw_writel(SRTC_LPCR_EN_LP, ioaddr + SRTC_LPCR);
		udelay(100);
	} else {
		/* move out of init state */
		__raw_writel((SRTC_LPCR_IE | SRTC_LPCR_NSA),
			     ioaddr + SRTC_LPCR);
#if 1
		ret = mxc_rtc_waitfor(ioaddr, SRTC_LPSR, SRTC_LPSR_IES, 100);
		if (ret) {
			goto clk_disable;
		}
#else
		udelay(100);
		while ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_IES) == 0);
#endif
		/* move out of non-valid state */
		__raw_writel((SRTC_LPCR_IE | SRTC_LPCR_NVE | SRTC_LPCR_NSA |
			      SRTC_LPCR_EN_LP), ioaddr + SRTC_LPCR);

#if 1
		ret = mxc_rtc_waitfor(ioaddr, SRTC_LPSR, SRTC_LPSR_NVES, 100);
		if (ret) {
			goto clk_disable;
		}
#else
		udelay(100);
		while ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_NVES) == 0);
#endif
		__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);
		udelay(100);
	}
clk_disable:
	clk_disable(clk);
put_clk:
	clk_put(clk);
unmap:
	iounmap(secmode_addr);
	return ret;
}

static int __devinit mxc_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct rtc_device *rtc;
	struct rtc_drv_data *mxc_rtc;
	void __iomem *ioaddr;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (!request_mem_region(res->start, resource_size(res), "MXC SRTC")) {
		return -EBUSY;
	}

	mxc_rtc = kzalloc(sizeof(*mxc_rtc), GFP_KERNEL);
	if (mxc_rtc == NULL) {
		ret = -ENOMEM;
		goto release;
	}

	spin_lock_init(&mxc_rtc->lock);
	mxc_rtc->res = res;
	ioaddr = ioremap(res->start, resource_size(res));
	if (ioaddr == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	mxc_rtc->ioaddr = ioaddr;
	mxc_rtc->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(mxc_rtc->clk)) {
		if (PTR_ERR(mxc_rtc->clk) == -ENOENT) {
			DBG(0, "%s: No RTC clock\n", __FUNCTION__);
			mxc_rtc->clk = NULL;
		} else {
			ret = PTR_ERR(mxc_rtc->clk);
			DBG(0, "%s: Failed to get RTC clock: %d\n",
				__FUNCTION__, ret);
			goto unmap;
		}
	} else {
		if (clk_get_rate(mxc_rtc->clk) != 32768) {
			printk(KERN_ALERT "Invalid RTC clock rate: %lu\n",
				clk_get_rate(mxc_rtc->clk));
			ret = -EINVAL;
			goto clk_put;
		}
		DBG(0, "%s: Enabling RTC clock\n", __FUNCTION__);
		clk_enable(mxc_rtc->clk);
	}

	platform_set_drvdata(pdev, mxc_rtc);

	/* Configure and enable the RTC */
	mxc_rtc->irq = platform_get_irq(pdev, 0);
	if (mxc_rtc->irq >= 0) {
		if (request_irq(mxc_rtc->irq, mxc_rtc_interrupt, 0,
				pdev->name, pdev) < 0) {
			dev_warn(&pdev->dev, "interrupt %d not available\n",
				mxc_rtc->irq);
			mxc_rtc->irq = -1;
		}
	}

	/* initialize glitch detect */
	__raw_writel(SRTC_LPPDR_INIT, ioaddr + SRTC_LPPDR);
	udelay(100);

	/* clear lp interrupt status */
	__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);
	udelay(100);;

	ret = mxc_rtc_init_secmode(pdev, ioaddr);
	if (ret) {
		DBG(0, "%s: init_secmode failed: %d\n", __FUNCTION__, ret);
		goto free_irq;
	}
	rtc = rtc_device_register(pdev->name, &pdev->dev,
				  &mxc_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		DBG(0, "%s: Failed to register RTC device: %d\n", __FUNCTION__, ret);
		goto free_irq;
	}

	mxc_rtc->rtc = rtc;
	printk(KERN_INFO "MXC SRTC driver IRQ %d\n", mxc_rtc->irq);

	/* By default, devices should wakeup if they can */
	/* So srtc is set as "should wakeup",
	 * but only if we have an IRQ
	 */
	device_init_wakeup(&pdev->dev, mxc_rtc->irq >= 0);

	if (mxc_rtc->clk)
		clk_disable(mxc_rtc->clk);
	return ret;

free_irq:
	if (mxc_rtc->irq >= 0)
		free_irq(mxc_rtc->irq, pdev);
	if (mxc_rtc->clk)
		clk_disable(mxc_rtc->clk);
clk_put:
	if (mxc_rtc->clk)
		clk_put(mxc_rtc->clk);
unmap:
	iounmap(mxc_rtc->ioaddr);
release:
	release_mem_region(res->start, resource_size(res));
free:
	kfree(mxc_rtc);
	return ret;
}

static int __devexit mxc_rtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *mxc_rtc = platform_get_drvdata(pdev);
	struct resource *res = mxc_rtc->res;

	rtc_device_unregister(mxc_rtc->rtc);
	if (mxc_rtc->irq >= 0)
		free_irq(mxc_rtc->irq, pdev);

	if (mxc_rtc->clk) {
		clk_disable(mxc_rtc->clk);
		clk_put(mxc_rtc->clk);
	}
	iounmap(mxc_rtc->ioaddr);
	release_mem_region(res->start, resource_size(res));
	kfree(mxc_rtc);
	return 0;
}

#ifdef CONFIG_PM
static int mxc_rtc_suspend(struct device *dev)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);

	if (device_can_wakeup(dev)) {
		int ret;

		DBG(0, "%s: Enabling IRQ %d wakeup\n", __FUNCTION__, mxc_rtc->irq);
		ret = enable_irq_wake(mxc_rtc->irq);
		if (ret != 0) {
			dev_warn(dev, "Failed to enable IRQ wake for IRQ %d: %d\n",
				mxc_rtc->irq, ret);
			return ret;
		}
	}
	return 0;
}

static int mxc_rtc_resume(struct device *dev)
{
	struct rtc_drv_data *mxc_rtc = dev_get_drvdata(dev);

	if (device_can_wakeup(dev)) {
		DBG(0, "%s: Disabling IRQ %d wakeup\n", __FUNCTION__, mxc_rtc->irq);
		disable_irq_wake(mxc_rtc->irq);
	}
	return 0;
}
#else
#define mxc_rtc_suspend	NULL
#define mxc_rtc_resume	NULL
#endif

static struct dev_pm_ops mxc_rtc_pm_ops = {
	.suspend = mxc_rtc_suspend,
	.resume = mxc_rtc_resume,
};

/*!
 * Contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_rtc_driver = {
	.driver = {
		.name = "mxc_rtc",
		.pm = &mxc_rtc_pm_ops,
	},
	.probe = mxc_rtc_probe,
	.remove = __devexit_p(mxc_rtc_remove),
};

static int __init mxc_rtc_init(void)
{
	return platform_driver_register(&mxc_rtc_driver);
}
module_init(mxc_rtc_init);

static void __exit mxc_rtc_exit(void)
{
	platform_driver_unregister(&mxc_rtc_driver);

}
module_exit(mxc_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
