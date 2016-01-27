/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*
 * Implementation based on rtc-ds1553.c
 */

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-mxc.c
 * @brief Real Time Clock interface
 *
 * This file contains Real Time Clock interface for Linux.
 *
 * @ingroup RTC
 */

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
#define RTC_INPUT_CLK_32768HZ	(0x00 << 5)
#define RTC_INPUT_CLK_32000HZ	(0x01 << 5)
#define RTC_INPUT_CLK_38400HZ	(0x02 << 5)

#define RTC_SW_BIT      (1 << 0)
#define RTC_ALM_BIT     (1 << 2)
#define RTC_1HZ_BIT     (1 << 4)
#define RTC_2HZ_BIT     (1 << 7)
#define RTC_SAM0_BIT    (1 << 8)
#define RTC_SAM1_BIT    (1 << 9)
#define RTC_SAM2_BIT    (1 << 10)
#define RTC_SAM3_BIT    (1 << 11)
#define RTC_SAM4_BIT    (1 << 12)
#define RTC_SAM5_BIT    (1 << 13)
#define RTC_SAM6_BIT    (1 << 14)
#define RTC_SAM7_BIT    (1 << 15)
#define PIT_ALL_ON	(RTC_2HZ_BIT | RTC_SAM0_BIT | RTC_SAM1_BIT | \
			 RTC_SAM2_BIT | RTC_SAM3_BIT | RTC_SAM4_BIT | \
			 RTC_SAM5_BIT | RTC_SAM6_BIT | RTC_SAM7_BIT)

#define RTC_ENABLE_BIT  (1 << 7)

#define MAX_PIE_NUM     9
#define MAX_PIE_FREQ    512
const u32 PIE_BIT_DEF[MAX_PIE_NUM][2] = {
	{2, RTC_2HZ_BIT},
	{4, RTC_SAM0_BIT},
	{8, RTC_SAM1_BIT},
	{16, RTC_SAM2_BIT},
	{32, RTC_SAM3_BIT},
	{64, RTC_SAM4_BIT},
	{128, RTC_SAM5_BIT},
	{256, RTC_SAM6_BIT},
	{MAX_PIE_FREQ, RTC_SAM7_BIT},
};

/* Those are the bits from a classic RTC we want to mimic */
#define RTC_IRQF                0x80	/* any of the following 3 is active */
#define RTC_PF                  0x40	/* Periodic interrupt */
#define RTC_AF                  0x20	/* Alarm interrupt */
#define RTC_UF                  0x10	/* Update interrupt for 1Hz RTC */

#define MXC_RTC_TIME            0
#define MXC_RTC_ALARM           1

#define RTC_HOURMIN    0x00	/*  32bit rtc hour/min counter reg */
#define RTC_SECOND     0x04	/*  32bit rtc seconds counter reg */
#define RTC_ALRM_HM    0x08	/*  32bit rtc alarm hour/min reg */
#define RTC_ALRM_SEC   0x0C	/*  32bit rtc alarm seconds reg */
#define RTC_RTCCTL     0x10	/*  32bit rtc control reg */
#define RTC_RTCISR     0x14	/*  32bit rtc interrupt status reg */
#define RTC_RTCIENR    0x18	/*  32bit rtc interrupt enable reg */
#define RTC_STPWCH     0x1C	/*  32bit rtc stopwatch min reg */
#define RTC_DAYR       0x20	/*  32bit rtc days counter reg */
#define RTC_DAYALARM   0x24	/*  32bit rtc day alarm reg */
#define RTC_TEST1      0x28	/*  32bit rtc test reg 1 */
#define RTC_TEST2      0x2C	/*  32bit rtc test reg 2 */
#define RTC_TEST3      0x30	/*  32bit rtc test reg 3 */

#define pdev_dbg(d, fmt...)	dev_dbg(&(d)->dev, fmt)

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

struct mxc_rtc {
	struct rtc_device *rtcdev;
	void __iomem *ioaddr;
	struct resource *memres;
	int irq;
	struct clk *clk;
	u32 pfreq;
	struct rtc_time alarm;
	spinlock_t lock;
#if 0
	unsigned int irqen;
	int alrm_sec;
	int alrm_min;
	int alrm_hour;
	int alrm_mday;
#endif
};

#define RTC_VERSION		"1.0"

static unsigned long rtc_status;

/*!
 * This function is used to obtain the RTC time or the alarm value in
 * second.
 *
 * @param  time_alarm   use MXC_RTC_TIME for RTC time value; MXC_RTC_ALARM for alarm value
 *
 * @return The RTC time or alarm time in second.
 */
static u32 get_alarm_or_time(struct device *dev, int time_alarm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;
	u32 day, hr, min, sec, hr_min;
	if (time_alarm == MXC_RTC_TIME) {
		day = readw(ioaddr + RTC_DAYR);
		hr_min = readw(ioaddr + RTC_HOURMIN);
		sec = readw(ioaddr + RTC_SECOND);
	} else if (time_alarm == MXC_RTC_ALARM) {
		day = readw(ioaddr + RTC_DAYALARM);
		hr_min = (0x0000FFFF) & readw(ioaddr + RTC_ALRM_HM);
		sec = readw(ioaddr + RTC_ALRM_SEC);
	} else {
		panic("wrong value for time_alarm=%d\n", time_alarm);
	}

	hr = hr_min >> 8;
	min = hr_min & 0x00FF;

	return ((((day * 24 + hr) * 60) + min) * 60 + sec);
}

/*!
 * This function sets the RTC alarm value or the time value.
 *
 * @param  time_alarm   the new alarm value to be updated in the RTC
 * @param  time         use MXC_RTC_TIME for RTC time value; MXC_RTC_ALARM for alarm value
 */
static void set_alarm_or_time(struct device *dev, int time_alarm, u32 time)
{
	u32 day, hr, min, sec, temp;
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;
	day = time / 86400;
	time -= day * 86400;
	/* time is within a day now */
	hr = time / 3600;
	time -= hr * 3600;
	/* time is within an hour now */
	min = time / 60;
	sec = time - min * 60;

	temp = (hr << 8) + min;

	if (time_alarm == MXC_RTC_TIME) {
		writew(day, ioaddr + RTC_DAYR);
		writew(sec, ioaddr + RTC_SECOND);
		writew(temp, ioaddr + RTC_HOURMIN);
	} else if (time_alarm == MXC_RTC_ALARM) {
		writew(day, ioaddr + RTC_DAYALARM);
		writew(sec, ioaddr + RTC_ALRM_SEC);
		writew(temp, ioaddr + RTC_ALRM_HM);
	} else {
		panic("wrong value for time_alarm=%d\n", time_alarm);
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
	void __iomem *ioaddr = rtc->ioaddr;

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
	writew(readw(ioaddr + RTC_RTCISR), ioaddr + RTC_RTCISR);

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
static irqreturn_t mxc_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;
	u32 status;
	u32 events = 0;

	spin_lock(&rtc->lock);
	status = readw(ioaddr + RTC_RTCISR) & readw(ioaddr + RTC_RTCIENR);
	/* clear interrupt sources */
	writew(status, ioaddr + RTC_RTCISR);

	/* clear alarm interrupt if it has occurred */
	if (status & RTC_ALM_BIT) {
		status &= ~RTC_ALM_BIT;
	}

	/* update irq data & counter */
	if (status & RTC_ALM_BIT) {
		events |= (RTC_AF | RTC_IRQF);
	}
	if (status & RTC_1HZ_BIT) {
		events |= (RTC_UF | RTC_IRQF);
	}
	if (status & PIT_ALL_ON) {
		events |= (RTC_PF | RTC_IRQF);
	}

	if ((status & RTC_ALM_BIT) && rtc_valid_tm(&rtc->alarm)) {
		rtc_update_alarm(&pdev->dev, &rtc->alarm);
	}

	spin_unlock(&rtc->lock);
	rtc_update_irq(rtc->rtcdev, 1, events);
	return IRQ_HANDLED;
}

/*!
 * This function is used to open the RTC driver by registering the RTC
 * interrupt service routine.
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_open(struct device *dev)
{
	if (test_and_set_bit(1, &rtc_status))
		return -EBUSY;
	return 0;
}

/*!
 * clear all interrupts and release the IRQ
 */
static void mxc_rtc_release(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;

	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	spin_lock_irq(&rtc->lock);
	writew(0, ioaddr + RTC_RTCIENR);	/* Disable all rtc interrupts */
	writew(0xFFFFFFFF, ioaddr + RTC_RTCISR);	/* Clear all interrupt status */
	spin_unlock_irq(&rtc->lock);
	rtc_status = 0;
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
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;
	int i;

	switch (cmd) {
	case RTC_PIE_OFF:
		writew((readw(ioaddr + RTC_RTCIENR) & ~PIT_ALL_ON),
		       ioaddr + RTC_RTCIENR);
		return 0;

	case RTC_IRQP_SET:
		if (arg < 2 || arg > MAX_PIE_FREQ || (arg % 2) != 0)
			return -EINVAL;	/* Also make sure a power of 2Hz */
		if ((arg > 64) && (!capable(CAP_SYS_RESOURCE)))
			return -EACCES;
		rtc->pfreq = arg;
		return 0;

	case RTC_IRQP_READ:
		return put_user(rtc->pfreq, (u32 *)arg);

	case RTC_PIE_ON:
		for (i = 0; i < MAX_PIE_NUM; i++) {
			if (PIE_BIT_DEF[i][0] == rtc->pfreq) {
				break;
			}
		}
		if (i == MAX_PIE_NUM) {
			return -EINVAL;
		}
		spin_lock_irq(&rtc->lock);
		writew((readw(ioaddr + RTC_RTCIENR) | PIE_BIT_DEF[i][1]),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc->lock);
		return 0;

	case RTC_AIE_OFF:
		spin_lock_irq(&rtc->lock);
		writew((readw(ioaddr + RTC_RTCIENR) & ~RTC_ALM_BIT),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc->lock);
		return 0;

	case RTC_AIE_ON:
		spin_lock_irq(&rtc->lock);
		writew((readw(ioaddr + RTC_RTCIENR) | RTC_ALM_BIT),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc->lock);
		return 0;

	case RTC_UIE_OFF:	/* UIE is for the 1Hz interrupt */
		spin_lock_irq(&rtc->lock);
		writew((readw(ioaddr + RTC_RTCIENR) & ~RTC_1HZ_BIT),
		       ioaddr + RTC_RTCIENR);
		spin_unlock_irq(&rtc->lock);
		return 0;

	case RTC_UIE_ON:
		spin_lock_irq(&rtc->lock);
		writew((readw(ioaddr + RTC_RTCIENR) | RTC_1HZ_BIT),
		       ioaddr + RTC_RTCIENR);
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
static int mxc_rtc_read_time(struct device *dev, struct rtc_time *tm)
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
static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm)
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
static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;

	rtc_time_to_tm(get_alarm_or_time(dev, MXC_RTC_ALARM), &alrm->time);
	alrm->enabled = !!(readw(ioaddr + RTC_RTCIENR) & RTC_ALM_BIT);
	alrm->pending = !!(readw(ioaddr + RTC_RTCISR) & RTC_ALM_BIT);

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
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;
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
		memcpy(&rtc->alarm, &alrm->time, sizeof(struct rtc_time));

		if (alrm->enabled) {
			writew((readw(ioaddr + RTC_RTCIENR) | RTC_ALM_BIT),
			       ioaddr + RTC_RTCIENR);
		} else {
			writew((readw(ioaddr + RTC_RTCIENR) & ~RTC_ALM_BIT),
			       ioaddr + RTC_RTCIENR);
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
static int mxc_rtc_proc(struct device *dev, struct seq_file *sq)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	void __iomem *ioaddr = rtc->ioaddr;
	char *p = sq->buf;

	p += sprintf(p, "alarm_IRQ\t: %s\n",
		     (((readw(ioaddr + RTC_RTCIENR)) & RTC_ALM_BIT) !=
		      0) ? "yes" : "no");
	p += sprintf(p, "update_IRQ\t: %s\n",
		     (((readw(ioaddr + RTC_RTCIENR)) & RTC_1HZ_BIT) !=
		      0) ? "yes" : "no");
	p += sprintf(p, "periodic_IRQ\t: %s\n",
		     (((readw(ioaddr + RTC_RTCIENR)) & PIT_ALL_ON) !=
		      0) ? "yes" : "no");
	p += sprintf(p, "periodic_freq\t: %d\n", rtc->pfreq);

	return p - (sq->buf);
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

static int __devinit mxc_rtc_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct resource *res;
	struct mxc_rtc *rtc;
	u32 reg;
	int ret;
	unsigned long rtc_input_clk;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	DBG(0, "%s: Probing RTC device @ %08lx\n", __FUNCTION__, (unsigned long)res->start);

	clk = clk_get(NULL, "ckil");
	if (IS_ERR(clk)) {
		pdev_dbg(pdev, "Failed to get reference clock: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}
	rtc_input_clk = clk_get_rate(clk);
	clk_put(clk);
	if (rtc_input_clk == 32768)
		reg = RTC_INPUT_CLK_32768HZ;
	else if (rtc_input_clk == 32000)
		reg = RTC_INPUT_CLK_32000HZ;
	else if (rtc_input_clk == 38400)
		reg = RTC_INPUT_CLK_38400HZ;
	else {
		pdev_dbg(pdev, "reference clock rate %lu is not valid\n", rtc_input_clk);
		return -EINVAL;
	}
	pdev_dbg(pdev, "reference clock is %lu.%03lukHz\n", rtc_input_clk / 1000,
		rtc_input_clk % 1000);

	if (!request_mem_region(res->start, resource_size(res), "MXC RTC")) {
		return -EBUSY;
	}

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;
	spin_lock_init(&rtc->lock);

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
	rtc->rtcdev = rtc_device_register(pdev->name, &pdev->dev, &mxc_rtc_ops,
					THIS_MODULE);
	if (IS_ERR(rtc->rtcdev)) {
		ret = PTR_ERR(rtc->rtcdev);
		goto err3;
	}
	platform_set_drvdata(pdev, rtc);

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq >= 0) {
		if (request_irq(rtc->irq, mxc_rtc_interrupt, IRQF_SHARED,
				pdev->name, pdev) < 0) {
			dev_warn(&pdev->dev, "interrupt not available\n");
			rtc->irq = -1;
		}
	}
	device_set_wakeup_capable(&pdev->dev, 1);

	reg |= RTC_ENABLE_BIT;
	writew(reg, (rtc->ioaddr + RTC_RTCCTL));
	if (((readw(rtc->ioaddr + RTC_RTCCTL)) & RTC_ENABLE_BIT) == 0) {
		printk(KERN_ALERT "rtc : hardware module can't be enabled!\n");
		ret = -EIO;
		goto err4;
	}
	printk("Real Time clock Driver v%s %lukHz base clock\n", RTC_VERSION,
		rtc_input_clk);
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

static int __devexit mxc_rtc_remove(struct platform_device *pdev)
{
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);
	struct resource *res = rtc->memres;

	rtc_device_unregister(rtc->rtcdev);
	if (rtc->irq >= 0)
		free_irq(rtc->irq, pdev);
	if (rtc->clk) {
		clk_disable(rtc->clk);
		clk_put(rtc->clk);
	}
	iounmap(rtc->ioaddr);
	release_mem_region(res->start, resource_size(res));
	kfree(rtc);
	return 0;
}

static int mxc_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);

	if (device_can_wakeup(&pdev->dev)) {
		int ret;

		ret = enable_irq_wake(rtc->irq);
		if (ret != 0) {
			dev_warn(&pdev->dev, "Failed to enable IRQ wake for IRQ %d: %d\n",
				rtc->irq, ret);
			return ret;
		}
	}
	return 0;
}

static int mxc_rtc_resume(struct platform_device *pdev)
{
	struct mxc_rtc *rtc = platform_get_drvdata(pdev);

	if (device_can_wakeup(&pdev->dev))
		disable_irq_wake(rtc->irq);
	return 0;
}

/*!
 * Contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_rtc_driver = {
	.driver = {
		.name = "mxc_rtc",
	},
	.probe = mxc_rtc_probe,
	.remove = __devexit_p(mxc_rtc_remove),
	.suspend = mxc_rtc_suspend,
	.resume = mxc_rtc_resume,
};

static int __init mxc_rtc_init(void)
{
	return platform_driver_register(&mxc_rtc_driver);
}

static void __exit mxc_rtc_exit(void)
{
	platform_driver_unregister(&mxc_rtc_driver);

}

module_init(mxc_rtc_init);
module_exit(mxc_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
