/*
 * drivers/rtc/rtc-mx25.c
 *
 * (C) Copyright Lothar Wassmann <LW@KARO-electronics.de>
 *
 * i.MX25 DRYICE RTC driver
 * based on drivers/rtc/rtc-mxc.c Copyright 2004-2007 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/uaccess.h>

#include <mach/hardware.h>
#include <asm/io.h>

/* DRYICE RTC registers */
typedef enum {
	RTCMR	= 0x00,
	RTCLR	= 0x04,
	RCAMR	= 0x08,
	RCALR	= 0x0c,
	DCR	= 0x10,
	DSR	= 0x14,
	DIER	= 0x18,
	DGPR	= 0x3c,
} rtc_mx25_reg_t;

#define DCR_TCHL		(1 << 18)
#define DCR_TCSL		(1 << 17)
#define DCR_FSHL		(1 << 16)
#define DCR_NSA			(1 << 15)
#define DCR_OSCB		(1 << 14)
#define DCR_APE			(1 << 4)
#define DCR_TCE			(1 << 3)
#define DCR_SWR			(1 << 0)

#define DSR_EBD			(1 << 20)
#define DSR_WBF			(1 << 10)
#define DSR_WNF			(1 << 9)
#define DSR_WCF			(1 << 8)
#define DSR_WEF			(1 << 7)
#define DSR_CAF			(1 << 4)
#define DSR_TCO			(1 << 2)
#define DSR_NVF			(1 << 1)
#define DSR_SVF			(1 << 0)

#define DIER_WNIE		(1 << 9)
#define DIER_WCIE		(1 << 8)
#define DIER_WEIE		(1 << 7)
#define DIER_CAIE		(1 << 4)
#define DIER_TOIE		(1 << 2)


/* Those are the bits from a classic RTC we want to mimic */
#define RTC_IRQF		0x80	/* any of the following 3 is active */
#define RTC_PF			0x40	/* Periodic interrupt */
#define RTC_AF			0x20	/* Alarm interrupt */
#define RTC_UF			0x10	/* Update interrupt for 1Hz RTC */

#define MXC_RTC_TIME		0
#define MXC_RTC_ALARM		1

#define pdev_dbg(d, fmt...)	dev_dbg(&(d)->dev, fmt)

#ifdef DEBUG
static int debug = 0;
#define dbg_lvl(n)	((n) < debug)
#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
module_param(debug, int, S_IWUSR | S_IRUGO);
#else
static int debug;
#define dbg_lvl(n)	0
#define DBG(lvl, fmt...)	do { } while (0)
module_param(debug, int, 0);
#endif

#define USE_TIMER

struct mxc_rtc {
	struct rtc_device *rtcdev;
	void __iomem *ioaddr;
	struct resource *memres;
	int irq;
	struct clk *clk;
	spinlock_t lock;
	unsigned long ref_clk;
	struct rtc_time alarm;
	unsigned int valid_time:1;
#ifdef USE_TIMER
	struct timer_list timer;
#endif
};

#define DRV_NAME		"rtc-mx25"
#define RTC_VERSION		"1.0"

/* There is a synchronisation delay of three 32768Hz clock cycles
 * when accessing the RTC registers. Thus any write should complete
 * within about 90 microseconds.
 */
#define MX25_RTC_WRITE_TIMEOUT	100

#ifdef DEBUG

#define rtc_mx25_write(c, v, r)		_rtc_mx25_write(c, v, r, #r, __FUNCTION__)
static inline int _rtc_mx25_write(struct mxc_rtc *rtc,
				unsigned int val, rtc_mx25_reg_t reg,
				const char *name, const char *fn)
{
	void __iomem *ioaddr = rtc->ioaddr;
	unsigned int dsr = 0;

	DBG(0, "%s: Writing %08x to %s[%02x]\n", fn, val, name, reg);
	BUG_ON(reg < 0 || reg > 0x3c);
	if (reg != DIER) {
		static int max_loops;
		int loops = 0;
		int last_dsr = -1;

		while ((dsr = __raw_readl(ioaddr + DSR)) & DSR_WBF) {
			if (loops++ > MX25_RTC_WRITE_TIMEOUT) {
				DBG(-1, "%s: Previous write did not complete; status: %08x\n",
					fn, dsr);
				__raw_writel(dsr, ioaddr + DSR);
				return -ETIME;
			}
			udelay(1);
		}

		__raw_writel(val, ioaddr + reg);
		DBG(0, "%s: Waiting for write completion\n", fn);

		loops = 0;
		while (!((dsr = __raw_readl(ioaddr + DSR)) & (DSR_WCF | DSR_WEF))) {
			if (last_dsr != dsr) {
				DBG(0, "%s: DSR=%08x\n", fn, dsr);
				last_dsr = dsr;
			}
			/* There is a synchronisation delay of thre 32768Hz clock cycles,
			 * thus any write should complete within about 90 microseconds.
			 */
			if (WARN_ON(loops++ > MX25_RTC_WRITE_TIMEOUT)) {
				DBG(-1, "%s: Write to %s[%02x] did not complete; status: %08x\n",
					fn, name, reg, dsr);
				__raw_writel(dsr, ioaddr + DSR);
				return -ETIME;
			}
			udelay(1);
		}
		if (WARN_ON(dsr & DSR_WEF)) {
			DBG(-1, "%s: Write to %s[%02x] failed; status: %08x\n", fn,
				name, reg, dsr);
			return -EACCES;
		}
		if (loops > max_loops) {
			DBG(-1, "%s: Write to %s[%02x] completed after %u loops: status %08x\n", fn,
				name, reg, loops, dsr);
			max_loops = loops;
		}
	} else {
		DBG(0, "%s: Writing %08x to %s[%02x]\n", fn, val, name, reg);
		__raw_writel(val, ioaddr + reg);
		return 0;
	}
	return dsr;
}

#define rtc_mx25_read(c, r)		_rtc_mx25_read(c, r, #r, __FUNCTION__)
static inline u32 _rtc_mx25_read(struct mxc_rtc *rtc, rtc_mx25_reg_t reg,
				const char *name, const char *fn)
{
	unsigned int val;
	void __iomem *ioaddr = rtc->ioaddr;

	BUG_ON(reg < 0 || reg > 0x3c);
	val = __raw_readl(ioaddr + reg);
	DBG(0, "%s: read %08x from reg %s[%02x]\n", fn, val, name, reg);
	return val;
}
#else
static inline int rtc_mx25_write(struct mxc_rtc *rtc, unsigned int val,
				rtc_mx25_reg_t reg)
{
	void __iomem *ioaddr = rtc->ioaddr;
	unsigned int dsr = 0;

	if (reg != DIER) {
		int loops = 0;
		while (__raw_readl(ioaddr + DSR) & DSR_WBF) {
			if (loops++ > MX25_RTC_WRITE_TIMEOUT) {
				return -ETIME;
			}
			udelay(1);
		}
		__raw_writel(val, ioaddr + reg);
		loops = 0;
		while (!((dsr = __raw_readl(ioaddr + DSR)) &
				(DSR_WCF | DSR_WEF))) {
			if (loops++ > MX25_RTC_WRITE_TIMEOUT) {
				return -ETIME;
			}
			udelay(1);
		}
		if (dsr & DSR_WEF) {
			return -EACCES;
		}
	} else {
		__raw_writel(val, ioaddr + reg);
	}
	return dsr;
}

static inline u32 rtc_mx25_read(struct mxc_rtc *rtc, rtc_mx25_reg_t reg)
{
	return __raw_readl(rtc->ioaddr + reg);
}
#endif

static int rtc_mx25_hwinit(struct platform_device *pdev, struct mxc_rtc *rtc)
{
	int ret;
	u32 dcr;
	u32 dsr;
	int loops = 0;

	dcr = rtc_mx25_read(rtc, DCR);
	if (!(dcr & DCR_NSA)) {
		/* enable non-secure write access */
		dcr |= DCR_NSA;
		ret = rtc_mx25_write(rtc, dcr, DCR);
		if (ret < 0) {
			DBG(0, "%s: failed to enable non-secure access\n",
				__FUNCTION__);
			return ret;
		}
	}

	/* clear non-valid and security error flags */
	dsr = rtc_mx25_read(rtc, DSR);
	dsr &= DSR_NVF | DSR_SVF;
	rtc_mx25_write(rtc, dsr, DSR);
	rtc->valid_time = (dcr & DCR_TCE) && !(dsr & DSR_NVF);

	if (!(dcr & DCR_TCE)) {
		ret = rtc_mx25_write(rtc, rtc_mx25_read(rtc, RTCMR), RTCMR);
		if (ret < 0) {
			DBG(0, "%s: Failed to write RTC register\n",
				__FUNCTION__);
			return ret;
		}

		dcr |= DCR_TCE;
		ret = rtc_mx25_write(rtc, dcr, DCR);
		if (ret < 0) {
			DBG(0, "%s: Failed to enable RTC\n", __FUNCTION__);
			return ret;
		}

		while ((ret & (DSR_WCF | DSR_WBF)) != DSR_WCF) {
			if (loops++ >= 32) {
				DBG(-1, "%s: Wait for write completion timed out\n",
					__FUNCTION__);
				return -ETIME;
			}
			udelay(1);
			ret = rtc_mx25_read(rtc, DSR);
		}

		dcr = rtc_mx25_read(rtc, DCR);
		DBG(-1, "%s: Write completed after %u loops; status %08x\n",
			__FUNCTION__, loops, ret);
		if (!(dcr & DCR_TCE)) {
			dev_err(&pdev->dev, "DRYICE RTC cannot be enabled!\n");
			return -EACCES;
		}
	}
	DBG(0, "%s: enabling interrupts\n", __FUNCTION__);
	rtc_mx25_write(rtc, DIER_TOIE, DIER);
	return 0;
}

/* Constant to convert time between RTC epoch starting on 2000-01-01
 * and UNIX epoch starting 1970-01-01
 */
#define RTC_EPOCH_ADJUST	946684800ULL

static u32 get_alarm_or_time(struct device *dev, int time_alarm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	u64 ticks;

	if (time_alarm == MXC_RTC_TIME) {
		u32 rtcmr;

		if (!rtc->valid_time) {
			dev_warn(dev, "RTC time not valid; hwclock needs to be set\n");
			ticks = 0;
		} else do {
			rtcmr = rtc_mx25_read(rtc, RTCMR);
			ticks = (u64)rtcmr << (32 - 17);
			ticks |= rtc_mx25_read(rtc, RTCLR) >> 17;
			DBG(0, "%s: RTCMR=%08x RTCLR=%08x\n", __FUNCTION__,
				rtcmr, (u32)ticks & ~((1 << 17) - 1));
		} while (rtcmr != rtc_mx25_read(rtc, RTCMR));
	} else if (time_alarm == MXC_RTC_ALARM) {
		ticks = (u64)rtc_mx25_read(rtc, RCAMR) << (32 - 17);
		ticks |= rtc_mx25_read(rtc, RCALR) >> 17;
	} else {
		BUG();
	}
	do_div(ticks, rtc->ref_clk);
	ticks += RTC_EPOCH_ADJUST;
	DBG(0, "%s: ticks=%08x\n", __FUNCTION__, (u32)ticks);
	return (u32)ticks;
}

static void set_alarm_or_time(struct device *dev, int time_alarm, u32 time)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	u64 ticks;

	/* Convert time from UNIX epoch start: 1970-01-01
	 * to RTC epoch start: 2000-01-01
	 */
	ticks = (((u64)time - RTC_EPOCH_ADJUST) * rtc->ref_clk) << 17;
	DBG(0, "%s: time=%08x ticks=%08x:%08x\n", __FUNCTION__,
		time, (u32)(ticks >> 32), (u32)ticks);

	if (time_alarm == MXC_RTC_TIME) {
		rtc_mx25_write(rtc, ticks & ~0UL, RTCLR);
		rtc_mx25_write(rtc, ticks >> 32, RTCMR);
		rtc->valid_time = 1;
	} else if (time_alarm == MXC_RTC_ALARM) {
		rtc_mx25_write(rtc, ticks >> 32, RCAMR);
		rtc_mx25_write(rtc, ticks & ~0UL, RCALR);
	} else {
		BUG();
	}
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
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);

	now = get_alarm_or_time(dev, MXC_RTC_TIME);
	rtc_time_to_tm(now, &now_tm);
	alarm_tm.tm_year = now_tm.tm_year;
	alarm_tm.tm_mon = now_tm.tm_mon;
	alarm_tm.tm_mday = now_tm.tm_mday;
	alarm_tm.tm_hour = alrm->tm_hour;
	alarm_tm.tm_min = alrm->tm_min;
	alarm_tm.tm_sec = alrm->tm_sec;
	rtc_tm_to_time(&now_tm, &now);
	rtc_tm_to_time(&alarm_tm, &time);
	if (time < now) {
		time += 60 * 60 * 24;
		rtc_time_to_tm(time, &alarm_tm);
	}
	ret = rtc_tm_to_time(&alarm_tm, &time);

	/* clear all the interrupt status bits */
	rtc_mx25_write(rtc, rtc_mx25_read(rtc, DSR), DSR);

	set_alarm_or_time(dev, MXC_RTC_ALARM, time);

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
static irqreturn_t rtc_mx25_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	u32 status;
	u32 events = 0;

	spin_lock(&rtc->lock);

	status = rtc_mx25_read(rtc, DSR) & rtc_mx25_read(rtc, DIER);
	/* clear interrupt sources */
	rtc_mx25_write(rtc, status, DSR);

	/* update irq data & counter */
	if (status & DSR_CAF) {
		DBG(0, "%s: ALARM interrupt\n", __FUNCTION__);
		events |= (RTC_AF | RTC_IRQF);
		rtc_update_alarm(&pdev->dev, &rtc->alarm);
	}
	if (status & DSR_TCO) {
		DBG(0, "%s: RTC counter overflow\n", __FUNCTION__);
		/* restart RTC */
		rtc_mx25_write(rtc, 0, RTCMR);
	}
#if 0
	if (status & (DSR_WCF | DSR_WNF)) {
		rtc->dsr = status;
		wake_up(&rtc->wq);
	}
#endif
	spin_unlock(&rtc->lock);

	if (events) {
		rtc_update_irq(rtc->rtcdev, 1, events);
	}
	return IRQ_HANDLED;
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
static int rtc_mx25_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	unsigned int val;

	switch (cmd) {
#ifndef CONFIG_RTC_INTF_DEV_UIE_EMUL
	case RTC_UIE_ON:
		val = rtc_mx25_read(rtc, RTCLR) >> 17;

		DBG(-1, "%s: RTCLR=%08x residue=%08lx ticks=%08lx\n",
			__FUNCTION__, val, rtc->ref_clk - val,
			msecs_to_jiffies((rtc->ref_clk - val) * 1000 /
					rtc->ref_clk));
		val = (rtc->ref_clk - val) * 1000 / rtc->ref_clk;
#ifdef USE_TIMER
		if (val > 0) {
			if (mod_timer(&rtc->timer, jiffies +
					msecs_to_jiffies(val))) {
				DBG(0, "%s: Reactivated timer %p\n",
					__FUNCTION__, &rtc->timer);
			} else {
				DBG(0, "%s: Activated timer %p\n",
					__FUNCTION__, &rtc->timer);
			}
		} else {
			rtc_update_irq(rtc->rtcdev, 1, RTC_UF | RTC_IRQF);
		}
#else
		if (val > 10) {
			msleep(val - 10);
			val = rtc_mx25_read(rtc, RTCLR) >> 17;
			//val = (rtc->ref_clk - val) * 1000 / rtc->ref_clk);
		}
		DBG(-1, "%s: RTCLR=%08x residue=%08lx ticks=%08lx\n",
			__FUNCTION__, val, rtc->ref_clk - val,
			msecs_to_jiffies((rtc->ref_clk - val) * 1000 /
					rtc->ref_clk));
		rtc_update_irq(rtc->rtcdev, 1, RTC_UF | RTC_IRQF);
#endif
		return 0;

	case RTC_UIE_OFF:
		/* UIE is for the 1Hz interrupt */
#ifdef USE_TIMER
		if (timer_pending(&rtc->timer)) {
			del_timer_sync(&rtc->timer);
		}
#endif
		return 0;
#endif
	case RTC_PIE_OFF:
	case RTC_PIE_ON:
	case RTC_IRQP_SET:
	case RTC_IRQP_READ:
		DBG(0, "%s: IOCTL %08x unimplemented\n", __FUNCTION__, cmd);
		return -ENOSYS;

	case RTC_AIE_OFF:
		spin_lock_irq(&rtc->lock);
		val = rtc_mx25_read(rtc, DIER);
		DBG(0, "%s: Disabling ALARM interrupt\n", __FUNCTION__);
		rtc_mx25_write(rtc, val & ~DIER_CAIE, DIER);
		spin_unlock_irq(&rtc->lock);
		return 0;

	case RTC_AIE_ON:
		spin_lock_irq(&rtc->lock);
		val = rtc_mx25_read(rtc, DIER);
		DBG(0, "%s: Enabling ALARM interrupt\n", __FUNCTION__);
		rtc_mx25_write(rtc, val | DIER_CAIE, DIER);
		spin_unlock_irq(&rtc->lock);
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
static int rtc_mx25_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 val;

	/* Avoid roll-over from reading the different registers */
	do {
		val = get_alarm_or_time(dev, MXC_RTC_TIME);
	} while (val != get_alarm_or_time(dev, MXC_RTC_TIME));

	rtc_time_to_tm(val, tm);
	return 0;
}

/*!
 * This function sets the internal RTC time based on tm in Gregorian date.
 *
 * @param  tm           the time value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int rtc_mx25_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	int ret;

	ret = rtc_tm_to_time(tm, &time);
	if (ret != 0) {
		return ret;
	}

	/* Avoid roll-over from reading the different registers */
	do {
		set_alarm_or_time(dev, MXC_RTC_TIME, time);
	} while (time != get_alarm_or_time(dev, MXC_RTC_TIME));

	return ret;
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
static int rtc_mx25_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);

	rtc_time_to_tm(get_alarm_or_time(dev, MXC_RTC_ALARM), &alrm->time);
	alrm->enabled = !!(rtc_mx25_read(rtc, DIER) & DIER_CAIE);
	alrm->pending = !!(rtc_mx25_read(rtc, DSR) & DSR_CAF);
	DBG(0, "%s: Alarm %sabled %spending\n", __FUNCTION__,
		alrm->enabled ? "en" : "dis", alrm->pending ? "" : "not ");
	return 0;
}

/*!
 * This function sets the RTC alarm based on passed in alrm.
 *
 * @param  alrm         the alarm value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int rtc_mx25_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	int ret;

	spin_lock_irq(&rtc->lock);
	if (rtc_valid_tm(&alrm->time)) {
		if (alrm->time.tm_sec > 59 ||
		    alrm->time.tm_hour > 23 || alrm->time.tm_min > 59) {
			ret = -EINVAL;
			goto out;
		}
		ret = rtc_update_alarm(dev, &alrm->time);
	} else {
		if ((ret = rtc_valid_tm(&alrm->time)))
			goto out;
		ret = rtc_update_alarm(dev, &alrm->time);
	}

	if (ret == 0) {
		unsigned int val;

		memcpy(&rtc->alarm, &alrm->time, sizeof(struct rtc_time));

		val = rtc_mx25_read(rtc, DIER);
		if (alrm->enabled) {
			DBG(0, "%s: Enabling ALARM interrupt\n", __FUNCTION__);
			rtc_mx25_write(rtc, val | DIER_CAIE, DIER);
		} else {
			DBG(0, "%s: Disabling ALARM interrupt\n", __FUNCTION__);
			rtc_mx25_write(rtc, val & ~DIER_CAIE, DIER);
		}
		printk(KERN_DEBUG "%s: %sabling wakeup for RTC\n", __FUNCTION__,
			alrm->enabled ? "En" : "Dis");
		device_set_wakeup_enable(dev, alrm->enabled);
	}
out:
	spin_unlock_irq(&rtc->lock);

	return ret;
}

/*!
 * This function is used to provide the content for the /proc/driver/rtc
 * file.
 *
 * @param  buf          the buffer to hold the information that the driver wants to write
 *
 * @return  The number of bytes written into the rtc file.
 */
static int rtc_mx25_proc(struct device *dev, struct seq_file *sq)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	char *p = sq->buf;

	p += sprintf(p, "alarm_IRQ\t: %s\n",
		((rtc_mx25_read(rtc, DIER)) & DIER_CAIE) ? "yes" : "no");

	return p - sq->buf;
}

/*!
 * The RTC driver structure
 */
static struct rtc_class_ops rtc_mx25_ops = {
	.ioctl = rtc_mx25_ioctl,
	.read_time = rtc_mx25_read_time,
	.set_time = rtc_mx25_set_time,
	.read_alarm = rtc_mx25_read_alarm,
	.set_alarm = rtc_mx25_set_alarm,
	.proc = rtc_mx25_proc,
};

#ifdef USE_TIMER
static void rtc_mx25_timer(unsigned long data)
{
	struct mxc_rtc *rtc = (struct mxc_rtc *)data;

	DBG(0, "%s: \n", __FUNCTION__);
	rtc_update_irq(rtc->rtcdev, 1, RTC_UF | RTC_IRQF);
}
#endif

static int __devinit rtc_mx25_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct resource *res;
	struct mxc_rtc *rtc;
	int ret;
	unsigned long rtc_input_clk;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	DBG(0, "%s: Probing RTC device @ %08lx\n", __FUNCTION__, (unsigned long)res->start);

	clk = clk_get_sys(NULL, "ckil");
	if (IS_ERR(clk)) {
		pdev_dbg(pdev, "Failed to get reference clock: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}
	rtc_input_clk = clk_get_rate(clk);
	clk_put(clk);
	pdev_dbg(pdev, "reference clock is %lu.%03lukHz\n", rtc_input_clk / 1000,
		rtc_input_clk % 1000);

	if (!request_mem_region(res->start, resource_size(res), "MXC RTC")) {
		return -EBUSY;
	}

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;
	rtc->memres = res;
	rtc->ref_clk = rtc_input_clk;
	spin_lock_init(&rtc->lock);
#ifdef USE_TIMER
	init_timer(&rtc->timer);
	rtc->timer.function = rtc_mx25_timer;
	rtc->timer.data = (unsigned long)rtc;
#endif
	rtc->ioaddr = ioremap(res->start, resource_size(res));
	if (!rtc->ioaddr) {
		ret = -ENOMEM;
		goto err1;
	}

	rtc->clk = clk_get(&pdev->dev, "rtc_clk");
	if (IS_ERR(rtc->clk)) {
		if (PTR_ERR(rtc->clk) == -ENOENT) {
			DBG(0, "%s: No RTC clock\n", __FUNCTION__);
			rtc->clk = NULL;
		} else {
			ret = PTR_ERR(rtc->clk);
			goto err2;
		}
	} else {
		DBG(0, "%s: Enabling RTC clock\n", __FUNCTION__);
		clk_enable(rtc->clk);
	}

	/* Configure and enable the RTC */
	rtc->rtcdev = rtc_device_register(pdev->name, &pdev->dev, &rtc_mx25_ops,
					THIS_MODULE);
	if (IS_ERR(rtc->rtcdev)) {
		ret = PTR_ERR(rtc->rtcdev);
		goto err3;
	}
	platform_set_drvdata(pdev, rtc);

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq >= 0) {
		if (request_irq(rtc->irq, rtc_mx25_interrupt, IRQF_SHARED,
				pdev->name, pdev) < 0) {
			dev_warn(&pdev->dev, "interrupt not available\n");
			rtc->irq = -1;
		}
	}
	device_set_wakeup_capable(&pdev->dev, 1);

	ret = rtc_mx25_hwinit(pdev, rtc);
	if (ret != 0) {
		goto err4;
	}

	printk("i.MX25 Real Time clock Driver; %lukHz base clock\n",
		rtc->ref_clk);
	return 0;

err4:
	rtc_device_unregister(rtc->rtcdev);
	if (rtc->irq >= 0)
		free_irq(rtc->irq, pdev);
err3:
	if (rtc->clk) {
		clk_disable(rtc->clk);
		clk_put(rtc->clk);
	}
err2:
	iounmap(rtc->ioaddr);
err1:
	release_mem_region(res->start, resource_size(res));
	kfree(rtc);
	return ret;
}

static int __devexit rtc_mx25_remove(struct platform_device *pdev)
{
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	struct resource *res = rtc->memres;

	DBG(0, "%s: Unregistering RTC\n", __FUNCTION__);

#ifdef USE_TIMER
	del_timer_sync(&rtc->timer);
#endif
	rtc_device_unregister(rtc->rtcdev);
	if (rtc->irq >= 0) {
		DBG(0, "%s: Freeing IRQ %d\n", __FUNCTION__, rtc->irq);
		free_irq(rtc->irq, pdev);
	}
	if (rtc->clk) {
		DBG(0, "%s: Disabling clock\n", __FUNCTION__);
		clk_disable(rtc->clk);
		clk_put(rtc->clk);
	}
	DBG(0, "%s: Unmapping registers\n", __FUNCTION__);
	iounmap(rtc->ioaddr);
	DBG(0, "%s: Releasing memory region\n", __FUNCTION__);
	release_mem_region(res->start, resource_size(res));
	DBG(0, "%s: Freeing RTC private data\n", __FUNCTION__);
	kfree(rtc);
	return 0;
}

#ifdef CONFIG_PM
static int rtc_mx25_suspend(struct device *dev)
{
	struct mxc_rtc *rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		int ret;

		ret = enable_irq_wake(rtc->irq);
		if (ret != 0) {
			dev_warn(dev, "Failed to enable IRQ wake for IRQ %d: %d\n",
				rtc->irq, ret);
			return ret;
		}
	}
	return 0;
}

static int rtc_mx25_resume(struct device *dev)
{
	struct mxc_rtc *rtc = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(rtc->irq);
	return 0;
}

/*
 * pointers to the power management callback functions.
 */
static struct dev_pm_ops rtc_mx25_pm_ops = {
	.suspend = rtc_mx25_suspend,
	.resume = rtc_mx25_resume,
};
#endif

static struct platform_driver rtc_mx25_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   .pm = __dev_pm_ops_p(rtc_mx25_pm_ops),
	},
	.probe = rtc_mx25_probe,
	.remove = __devexit_p(rtc_mx25_remove),
};

static int __init rtc_mx25_init(void)
{
	return platform_driver_register(&rtc_mx25_driver);
}

static void __exit rtc_mx25_exit(void)
{
	platform_driver_unregister(&rtc_mx25_driver);

}

module_init(rtc_mx25_init);
module_exit(rtc_mx25_exit);

MODULE_AUTHOR("Lothar Wassmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("i.MX25 Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
