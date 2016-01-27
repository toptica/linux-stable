/*
 *  General Purpose ADC driver
 *  for 8/16 single ended channels
 *
 *  Copyright (c) 2010 Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 * Based on mxc_tsc.c
 *  Copyright (c) 2009 Lothar Wassmann <LW@KARO-electronics.de>
 *
 * Based on atmel_tsadcc.c
 *  Copyright (c) 2008 ATMEL et. al.
 * and code from Freescale BSP
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG
#define ADC_MULTIPLEX

#include <linux/module.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/ctype.h>
#include <mach/mxc_tsc.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <asm/delay.h>

#include "mxc_tsc.h"

#ifdef DEBUG
static int debug = 1;
#define dbg_lvl(n)	((n) < debug)
module_param(debug, int, S_IRUGO | S_IWUSR);

#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#else
static int debug;
#define dbg_lvl(n)	0
module_param(debug, int, 0);

#define DBG(lvl, fmt...)	do { } while (0)
#endif

//#define DEFAULT_ADC_CLOCK	1666667
//#define DEFAULT_ADC_CLOCK	833333
#define DEFAULT_ADC_CLOCK	1700000

#define ADC_AVERAGING	16

#if (ADC_AVERAGING > 8)
#define ADC_SAMPLEMAX	ADC_AVERAGING
#else
#define ADC_SAMPLEMAX	8
#endif

#ifdef ADC_MULTIPLEX

#define ADC_MULTIPLEX_GPIO (GPIO_PORTB | 11)

int mxc_tsc_extmux_config(void)
{
	int ret;
	printk(KERN_INFO "%s: ADC is externally multiplexed\n",
				__FUNCTION__);
	ret = gpio_request(ADC_MULTIPLEX_GPIO, "ADC external mux");
	if (ret) {
		printk(KERN_INFO "%s: Failed to request GPIO for external ADC muxing: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
	ret = gpio_direction_output(ADC_MULTIPLEX_GPIO, 0);
	if (ret) {
		printk(KERN_INFO "%s: Failed to set GPIO direction for external ADC muxing: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
	gpio_export(ADC_MULTIPLEX_GPIO,0);
	return ret;
}

static inline void mxc_TSC_extmux_unconfig(void)
{
	gpio_unexport(ADC_MULTIPLEX_GPIO);
	gpio_free(ADC_MULTIPLEX_GPIO);
}

static inline void mxc_tsc_extmux(int mux)
{
	gpio_set_value(ADC_MULTIPLEX_GPIO,mux);
	//udelay(1000);
}
#else

static inline int mxc_tsc_extmux_config(void)	{return 0;}
static inline void mxc_TSC_extmux_unconfig(void) {}
static inline void mxc_tsc_extmux(int mux) {}

#endif



struct mxc_tsc_fifo_entry {
	unsigned int id:4,
		data:12;
};

struct mxc_tsc_adc_data {
	struct mxc_tsc_fifo_entry data[ADC_SAMPLEMAX];
};

typedef union {
	unsigned int fifo[sizeof(struct mxc_tsc_adc_data) / sizeof(int)];
	struct mxc_tsc_adc_data data;
} mxc_tsc_adc_fifo;

struct mxc_tsc_irqbuf {
	wait_queue_head_t	wq;
	unsigned int		*data;
	int			reqcount;
	int			irqcount;
	long			timeout;
	unsigned long		reg_base;
};

struct mxc_tsc {
	struct input_dev	*input;
	void __iomem		*reg_base;
	struct clk		*clk;
	int			irq;
	unsigned int	clk_enabled:1,
					attrs:1,
					intref:1;
	struct mxc_tsc_irqbuf	adc_buf;
	mxc_tsc_adc_fifo	*adc_data;
	struct mutex		adc_mutex;
	/* conversion clock rate in kHz */
	unsigned long		clkrate;
};

static inline unsigned long mxc_tsc_read(struct mxc_tsc *ts_dev, int reg)
{
	return __raw_readl(ts_dev->reg_base + reg);
}

static inline void mxc_tsc_write(struct mxc_tsc *ts_dev, int reg, unsigned long val)
{
	__raw_writel(val, ts_dev->reg_base + reg);
}

static inline void mxc_tsc_set_mask(struct mxc_tsc *ts_dev, int reg, unsigned long mask)
{
	unsigned long val = mxc_tsc_read(ts_dev, reg);
	val |= mask;
	mxc_tsc_write(ts_dev, reg, val);
}

static inline void mxc_tsc_clr_mask(struct mxc_tsc *ts_dev, int reg, unsigned long mask)
{
	unsigned long val = mxc_tsc_read(ts_dev, reg);
	val &= ~mask;
	mxc_tsc_write(ts_dev, reg, val);
}

static void tsc_clk_enable(struct mxc_tsc *ts_dev)
{
	if (!ts_dev->clk_enabled) {
		clk_enable(ts_dev->clk);
		mxc_tsc_set_mask(ts_dev, TGCR, TGCR_IPG_CLK_EN);
		ts_dev->clk_enabled = 1;
	}
}

#ifdef CONFIG_SUSPEND
static void tsc_clk_disable(struct mxc_tsc *ts_dev)
{
	if (ts_dev->clk_enabled) {
		mxc_tsc_clr_mask(ts_dev, TGCR, TGCR_IPG_CLK_EN);
		clk_disable(ts_dev->clk);
		ts_dev->clk_enabled = 0;
	}
}
#endif

static inline void mxc_tsc_read_fifo(struct mxc_tsc *ts_dev,
				     struct mxc_tsc_irqbuf *irqbuf)
{
	int start = irqbuf->irqcount;
	struct input_dev *input_dev = ts_dev->input;

	while (!(mxc_tsc_read(ts_dev, irqbuf->reg_base + CQSR) & CQSR_EMPT)) {

		unsigned long reg = mxc_tsc_read(ts_dev,
						irqbuf->reg_base + CQFIFO);

		BUG_ON(irqbuf->irqcount < 0);
		if (likely(irqbuf->irqcount < irqbuf->reqcount)) {
			BUG_ON(irqbuf->data == NULL);
			irqbuf->data[irqbuf->irqcount] = reg;
		} else {
			DBG(2, "%s: Dropping spurious data sample[%d/%d] on %s queue: %08lx\n",
				__FUNCTION__, irqbuf->irqcount,
				irqbuf->reqcount,
				irqbuf == &ts_dev->adc_buf ? "ADC" : "TSC", reg);
		}
		irqbuf->irqcount++;
	}

	if (mxc_tsc_read(ts_dev, irqbuf->reg_base + CQSR) &
		(CQSR_FOR | CQSR_FER)) {
		dev_warn(&input_dev->dev, "Fifo overrun on %s queue\n",
			irqbuf == &ts_dev->adc_buf ? "ADC" : "TSC");
		mxc_tsc_write(ts_dev, irqbuf->reg_base + CQSR,
			(CQSR_FOR | CQSR_FER));
		mxc_tsc_set_mask(ts_dev, irqbuf->reg_base + CQCR, CQCR_FRST);
		mxc_tsc_clr_mask(ts_dev, irqbuf->reg_base + CQCR, CQCR_FRST);
	}

	if (irqbuf->irqcount == irqbuf->reqcount) {
		WARN_ON(!(mxc_tsc_read(ts_dev, irqbuf->reg_base + CQSR) &
				CQSR_EMPT));
		wake_up(&irqbuf->wq);
	}

	DBG(2, "%s: Read %u samples [%d..%d] from fifo\n", __FUNCTION__,
		irqbuf->irqcount - start, start, irqbuf->irqcount - 1);
}

static int mxc_tsc_wait_data(struct mxc_tsc *ts_dev,
			     struct mxc_tsc_irqbuf *irqbuf)
{
	int ret;

	ret = wait_event_timeout(irqbuf->wq,
				irqbuf->reqcount == irqbuf->irqcount,
				irqbuf->timeout);
	if (ret == 0 && irqbuf->reqcount != irqbuf->irqcount) {
		DBG(0, "%s: Timeout waiting for %s data, reqcount %d, irqcount %d\n", __FUNCTION__,
			irqbuf == &ts_dev->adc_buf ? "ADC" : "TSC",irqbuf->reqcount,irqbuf->irqcount);
		mxc_tsc_set_mask(ts_dev, GCQCR, CQCR_FRST);
		mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_FRST);
		return -ETIME;
	}
	return 0;
}

/* parameter chan gives channel number 0..7 or -1 for reading all channels
 */
static int mxc_tsc_read_adc(struct mxc_tsc *ts_dev, int chan)
{
	int ret;
	unsigned long reg;
	int i;
	struct mxc_tsc_irqbuf *irqbuf = &ts_dev->adc_buf;
	struct mxc_tsc_adc_data *adc_data = &ts_dev->adc_data->data;

//	mutex_lock(&ts_dev->adc_mutex);

	if (chan<0) {
		if (chan==-2)
			mxc_tsc_extmux(1);
		else
			mxc_tsc_extmux(0);
		/* repare for all channels:
		 *  use 8 queue items
		 *  adjust watermark IRQ for 8 items */
		mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_LAST_ITEM_ID_MASK | CQCR_FIFOWATERMARK_MASK | CQCR_QSM_MASK);
		mxc_tsc_set_mask(ts_dev, GCQCR, (7 << CQCR_LAST_ITEM_ID_SHIFT) | (7 << CQCR_FIFOWATERMARK_SHIFT));
	} else {
		if (chan & 1) {
			mxc_tsc_extmux(1);
			chan  &= ~1;
		} else
			mxc_tsc_extmux(0);
		/* prepare for single channel:
		 * 	only use 1 queue item
		 *  adjust watermark IRQ for 1 item*/
		mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_LAST_ITEM_ID_MASK | CQCR_FIFOWATERMARK_MASK | CQCR_QSM_MASK);
		mxc_tsc_set_mask(ts_dev, GCQCR, 0 << CQCR_LAST_ITEM_ID_SHIFT | ((ADC_AVERAGING-1) << CQCR_FIFOWATERMARK_SHIFT));
	}

	while (!(mxc_tsc_read(ts_dev, irqbuf->reg_base + CQSR) & CQSR_EMPT))
		mxc_tsc_read(ts_dev, irqbuf->reg_base + CQFIFO);

	memset(adc_data, 0, sizeof(*adc_data));
	irqbuf->irqcount = 0;
	irqbuf->reqcount = (chan < 0)? 8 : ADC_AVERAGING; /* 1 ARNO */

	reg = mxc_tsc_read(ts_dev, GCC0);
	reg = (reg & ~CC_SELIN_MASK) | ((chan<0) ? 0 : chan);
	mxc_tsc_write(ts_dev, GCC0, reg);

	/* enable data ready and end of conversion interrupt */
	mxc_tsc_clr_mask(ts_dev, GCQMR,
			CQMR_EOQ_IRQ_MSK |
			CQMR_FDRY_IRQ_MSK |
			CQMR_FOR_IRQ_MSK |
			CQMR_FER_IRQ_MSK);

	/* prepare start mode FQS */
	mxc_tsc_set_mask(ts_dev, GCQCR, CQCR_QSM_FQS);

	/* start conversion */
	mxc_tsc_set_mask(ts_dev, GCQCR, CQCR_FQS);

	ret = mxc_tsc_wait_data(ts_dev, irqbuf);

	DBG(3, "%s: TGSR=%08lx\n", __FUNCTION__, mxc_tsc_read(ts_dev, TGSR));
	DBG(3, "%s: GCQSR=%08lx\n", __FUNCTION__, mxc_tsc_read(ts_dev, GCQSR));
	DBG(3, "%s: GCQCR=%08lx\n", __FUNCTION__, mxc_tsc_read(ts_dev, GCQCR));

	if (ret) {
		goto exit;
	}

	DBG(1, "%s: Read %u words from fifo\n", __FUNCTION__, irqbuf->reqcount);
	for (i = 0; i < irqbuf->reqcount; i++) {
		DBG(1, "%s: data[0x%x]=%4d\n", __FUNCTION__, i,
		    adc_data->data[i].data);
	}
exit:
	mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_QSM_MASK);
	mxc_tsc_set_mask(ts_dev, GCQMR, CQMR_EOQ_IRQ_MSK | CQMR_FDRY_IRQ_MSK);
//	mutex_unlock(&ts_dev->adc_mutex);

	return ret;
}

struct mxc_tsc_attr {
	struct device_attribute attr;
	unsigned int chan;
};

#define to_mxc_tsc_attr(a)		container_of(a, struct mxc_tsc_attr, attr)

#define MXC_TSC_DEV_ATTR(_name, _mode, _chan, _read)		\
	struct mxc_tsc_attr mxc_tsc_attr_##_name = {		\
		.attr = __ATTR(_name,_mode,_read, NULL),	\
		.chan = _chan,					\
	}

/* single channel measurements */
static ssize_t mxc_tsc_attr_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	int i, mn, mx, sum, gc, sc, ac, avg0, avg1, id, tmp;
	struct mxc_tsc *ts_dev = dev_get_drvdata(dev);
	struct mxc_tsc_attr *mxc_tsc_attr = to_mxc_tsc_attr(attr);
	struct mxc_tsc_adc_data *adc_data = &ts_dev->adc_data->data;

	mutex_lock(&ts_dev->adc_mutex);

	ret = mxc_tsc_read_adc(ts_dev, mxc_tsc_attr->chan);
	if (ret != 0) {
		dev_err(dev, "%s: Failed to read ADC%d\n", __FUNCTION__,
			(mxc_tsc_attr->chan & CC_SELIN_MASK) >>
				CC_SELIN_SHIFT);
		goto exit;
	}

	/*
	 * find minimum and maximum
	 * try to ignore zeros, assuming they are readout errors
	 */
	sum = 0;
	mn = 4096;
	mx= -1;
	for (i=0; i<ADC_AVERAGING; i++) {
		avg0 = adc_data->data[i].data;
		if (avg0) {
			if (avg0>mx) mx = avg0;
			if (avg0<mn) mn = avg0;
		}
	}
	/*
	 * if all values are zero
	 * we can't ignore them :(
	 */
	if (mx<0) {
		ret = sprintf(buf, "0\n");
		goto exit;
	}

	/*
	 * we assume that most values are ok and only
	 * a few outliers are present.
	 * a 3-bin histogram should help identifying
	 * the "good ones"
	 */
	avg0 = (3*mx + mn)/4;
	avg1 = (3*mn + mx)/4;

	gc = sc = ac = 0;
	mn = mx = 0;
	for (i=0; i<ADC_AVERAGING; i++) {
		tmp = adc_data->data[i].data;
		if (tmp) {
			if (tmp > avg0) {
				mx += 10*tmp;
				gc++;
			} else if (tmp < avg1) {
				mn += 10*tmp;
				sc++;
			} else {
				sum += 10*tmp;
				ac++;
			}
		}
	}
	/* now use the bin with the most members
	 * and return their average value ... */
	if (gc>ac && gc>sc)
		/* most values are "high values" */
		ret = sprintf(buf, "%d\n", (mx/gc + 5)/10);
	else if (sc>ac)
		/* most values are "small values" */
		ret = sprintf(buf, "%d\n", (mn/sc + 5)/10);
	else
		/* most values are "medium values" */
		ret = sprintf(buf, "%d\n", (sum/ac + 5)/10);
exit:
	mutex_unlock(&ts_dev->adc_mutex);

	return ret;
}

/* all channel measurement */
static ssize_t mxc_tsc_attr_getall(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	int i;

	struct mxc_tsc *ts_dev = dev_get_drvdata(dev);
	struct mxc_tsc_attr *mxc_tsc_attr = to_mxc_tsc_attr(attr);
	struct mxc_tsc_adc_data *adc_data = &ts_dev->adc_data->data;

	mutex_lock(&ts_dev->adc_mutex);

	ret = mxc_tsc_read_adc(ts_dev, mxc_tsc_attr->chan);
	if (ret != 0) {
		dev_err(dev, "%s: Failed to read all ADCs\n", __FUNCTION__);
		goto exit;
	}

	ret = 0;
	for (i=0; i<8; i++)
		ret += sprintf(buf + ret, "%d%c", adc_data->data[i].data,(i==7)?'\n':' ');
exit:
	mutex_unlock(&ts_dev->adc_mutex);
	return ret;
}

MXC_TSC_DEV_ATTR(adc0, S_IRUGO, CC_SELIN_XNUR, mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc1, S_IRUGO, CC_SELIN_XPUL , mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc2, S_IRUGO, CC_SELIN_YNLR , mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc3, S_IRUGO, CC_SELIN_YPLL , mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc4, S_IRUGO, CC_SELIN_WIPER, mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc5, S_IRUGO, CC_SELIN_INAUX0, mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc6, S_IRUGO, CC_SELIN_INAUX1, mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc7, S_IRUGO, CC_SELIN_INAUX2, mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(all0_7, S_IRUGO, -1, mxc_tsc_attr_getall);
#ifdef ADC_MULTIPLEX
MXC_TSC_DEV_ATTR(adc8, S_IRUGO, (CC_SELIN_XNUR   | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc9, S_IRUGO, (CC_SELIN_XPUL   | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc10, S_IRUGO, (CC_SELIN_YNLR  | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc11, S_IRUGO, (CC_SELIN_YPLL  | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc12, S_IRUGO, (CC_SELIN_WIPER | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc13, S_IRUGO, (CC_SELIN_INAUX0 | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc14, S_IRUGO, (CC_SELIN_INAUX1 | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(adc15, S_IRUGO, (CC_SELIN_INAUX2 | 1), mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(all8_15, S_IRUGO, -2, mxc_tsc_attr_getall);
#endif


static ssize_t show_reference(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxc_tsc *ts_dev = dev_get_drvdata(dev);
	int ret = ts_dev->intref;
	return sprintf(buf, "%d\n", ret);
}


static ssize_t store_reference(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t size)
{
	struct mxc_tsc *ts_dev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long reg,i;
	long value = simple_strtol(buf, &after, 10);
	size_t count = after - buf;
	if (*after && isspace(*after))
			count++;
	if (count != size) return ret;

	mutex_lock(&ts_dev->adc_mutex);
	ts_dev->intref = value ? 1:0;

	for (i=0; i < 8; i++) {
		reg = mxc_tsc_read(ts_dev, GCC0 + i*0x04);
		if (ts_dev->intref) {
			reg &= ~CC_SEL_REFP_MASK;
			reg |=  CC_SEL_REFP_INT;
		} else {
			reg &= ~CC_SEL_REFP_MASK;
			reg |=  CC_SEL_REFP_EXT;
		}
		mxc_tsc_write(ts_dev, GCC0 + i*0x04, reg);
	}

	reg = mxc_tsc_read(ts_dev, TGCR);
	if (ts_dev->intref)
		reg |=  TGCR_INTREFEN;
	else
		reg &=  ~TGCR_INTREFEN;
	mxc_tsc_write(ts_dev, TGCR, reg);

	mutex_unlock(&ts_dev->adc_mutex);
	return size;
}

static DEVICE_ATTR(internalref, S_IRUGO | S_IWUSR, show_reference, store_reference);

static struct attribute *mxc_tsc_attrs[] = {
	&dev_attr_internalref.attr,
	&mxc_tsc_attr_adc0.attr.attr,
	&mxc_tsc_attr_adc1.attr.attr,
	&mxc_tsc_attr_adc2.attr.attr,
	&mxc_tsc_attr_adc3.attr.attr,
	&mxc_tsc_attr_adc4.attr.attr,
	&mxc_tsc_attr_adc5.attr.attr,
	&mxc_tsc_attr_adc6.attr.attr,
	&mxc_tsc_attr_adc7.attr.attr,
	&mxc_tsc_attr_all0_7.attr.attr,
#ifdef ADC_MULTIPLEX
	&mxc_tsc_attr_adc8.attr.attr,
	&mxc_tsc_attr_adc9.attr.attr,
	&mxc_tsc_attr_adc10.attr.attr,
	&mxc_tsc_attr_adc11.attr.attr,
	&mxc_tsc_attr_adc12.attr.attr,
	&mxc_tsc_attr_adc13.attr.attr,
	&mxc_tsc_attr_adc14.attr.attr,
	&mxc_tsc_attr_adc15.attr.attr,
	&mxc_tsc_attr_all8_15.attr.attr,
#endif
	NULL
};

static const struct attribute_group mxc_tsc_attr_group = {
	.attrs = mxc_tsc_attrs,
};

static irqreturn_t mxc_tsc_interrupt(int irq, void *dev)
{
	struct mxc_tsc *ts_dev = dev;
	//struct input_dev *input_dev = ts_dev->input;
	unsigned long reg;
	unsigned long status = mxc_tsc_read(ts_dev, TGSR);

	DBG(3, "%s: TGSR=%08lx\n", __FUNCTION__, status);

	if (status & TGSR_TCQ_INT) {
		DBG(-1, "%s: Ooops. Wrontg type of interrupt! TCQ instead of GCQ\n",
						__FUNCTION__);
	}

	if (status & TGSR_GCQ_INT) {
		unsigned long mask = mxc_tsc_read(ts_dev, GCQMR);

		reg = mxc_tsc_read(ts_dev, GCQSR);
		DBG(1, "%s: GCQSR=%08lx GCQMR=%08lx:%08lx\n", __FUNCTION__,
			reg, mask, reg & ~mask);
		reg &= ~mask;
		mxc_tsc_write(ts_dev, GCQSR, reg);
		if (reg & CQSR_EOQ) {
			/* stop queue */
			mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_QSM_MASK);
			/* disable end of conversion interrupt */
			mxc_tsc_set_mask(ts_dev, GCQMR, CQMR_EOQ_IRQ_MSK);
		}
		if (reg & (CQSR_FOR | CQSR_FER)) {
			DBG(-1, "%s: Fifo overrun on ADC queue\n",
				__FUNCTION__);
			mxc_tsc_set_mask(ts_dev, GCQCR, CQCR_FRST);
			mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_FRST);
		} else if (reg & CQSR_FDRY) {
			struct mxc_tsc_irqbuf *irqbuf = &ts_dev->adc_buf;

			mxc_tsc_read_fifo(ts_dev, irqbuf);
		}
	}
	return IRQ_HANDLED;
}



#define SETTLE_MEAS	16


static void mxc_tsc_config(struct platform_device *pdev)
{
	struct mxc_tsc *ts_dev = platform_get_drvdata(pdev);
	unsigned int pdata = (int) pdev->dev.platform_data;
	unsigned int tgcr,i;
	unsigned int adc_clk = DEFAULT_ADC_CLOCK;
	unsigned long ipg_clk,reg;
	unsigned int clkdiv;

	int intref= (pdata) ? 1:0;
	dev_info(&pdev->dev, "using %s reference voltage\n",intref?"internal":"external");
	ts_dev->intref = intref;

	ipg_clk = clk_get_rate(ts_dev->clk);
	dev_info(&pdev->dev, "Master clock is: %lu.%06luMHz requested ADC clock: %u.%06uMHz\n",
		 ipg_clk / 1000000, ipg_clk % 1000000,
		 adc_clk / 1000000, adc_clk % 1000000);

	/*
	 * adc_clk = ipg_clk / (2 * clkdiv + 2)
	 * The exact formula for the clock divider would be:
	 * clkdiv = ipg_clk / (2 * adc_clk) - 1
	 * but we drop the '- 1' due to integer truncation
	 * and to make sure the actual clock is always less or equal
	 * to the designated clock.
	*/
	clkdiv = ipg_clk / (2 * adc_clk + 1);
	if (clkdiv > 31) {
		clkdiv = 31;
		dev_warn(&pdev->dev,
			 "cannot accomodate designated clock of %u.%06uMHz; using %lu.%06luMHz\n",
			 adc_clk / 1000000, adc_clk % 1000000,
			 ipg_clk / (2 * clkdiv + 2) / 1000000,
			 ipg_clk / (2 * clkdiv + 2) % 1000000);
	}
	/* calculate the actual ADC clock rate in kHz */
	ts_dev->clkrate = ipg_clk / (2 * clkdiv + 2) / 1000;
	dev_dbg(&pdev->dev, "clkdiv=%u actual ADC clock: %lu.%03luMHz\n",
		clkdiv, ts_dev->clkrate / 1000, ts_dev->clkrate % 1000);

	tgcr = 	TGCR_POWER_ON | /* Switch TSC on */
			(intref * TGCR_INTREFEN) | TGCR_STLC |
		((clkdiv << TGCR_ADCCLKCFG_SHIFT) & TGCR_ADCCLKCFG_MASK);

	/* reset TSC */
	mxc_tsc_write(ts_dev, TGCR, TGCR_TSC_RST);
	while (mxc_tsc_read(ts_dev, TGCR) & TGCR_TSC_RST) {
		cpu_relax();
	}
	mxc_tsc_write(ts_dev, TGCR, tgcr);
	ts_dev->clk_enabled=0;
	tsc_clk_enable(ts_dev);

	/* prepare queue for single channel but do not start ... */
	reg = (0 << CQCR_FIFOWATERMARK_SHIFT) |
		(0 << CQCR_LAST_ITEM_ID_SHIFT);
	mxc_tsc_write(ts_dev, GCQCR, reg);

	/* prepare one conversion setting for each input
	 * for single measurements only the first is used
	 * but for a complete measurement they are all used
	 */

	for (i=0; i < 8; i++) {
		reg = ((ADC_AVERAGING-1) << CC_NOS_SHIFT) |
			(SETTLE_MEAS << CC_SETTLING_TIME_SHIFT) |
			CC_YPLLSW_OFF | CC_XNURSW_OFF | CC_XPULSW |
			(intref?CC_SEL_REFP_INT:CC_SEL_REFP_EXT) | CC_SEL_REFN_AGND | //| CC_SEL_REFN_XNUR |
			(i << CC_SELIN_SHIFT);
		mxc_tsc_write(ts_dev, GCC0 + i*0x04, reg);
	}
	DBG(-1,"%s: %s measurements",__FUNCTION__,(reg & CC_SEL_REFN_MASK)>1?"single ended":"differential");

	/* prepare 8 queue items for the 8 channels */

	reg = (0x0 << 0) | (0x1 << 4) | (0x2 << 8) | (0x3 << 12) |
		  (0x4 << 16)| (0x5 << 20)| (0x6 << 24)| (0x7 << 28);
	mxc_tsc_write(ts_dev, GCQ_ITEM_7_0, reg);

	/* ADC conversion requires 14 clock cycles per samplemxc_tsc_set_mask(ts_dev, GCQCR, CQCR_FRST);
			mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_FRST);
	 * plus the settling time programmed in the TICR registers.
	 * Add 1 extra jiffy to make sure the timeout is > 0
	 *
	 * ARNOFIX: limiting factor here is not the conversion time
	 * but the kernel, spending time on other things and therefore
	 * not waking up the driver within 1 jiffy after success
	 */
	ts_dev->adc_buf.timeout = 10;
	/*msecs_to_jiffies(
		((10 * 14) +
			(SETTLE_MEAS * 8 + 1)) /
		ts_dev->clkrate / 1000) + 1;*/
	DBG(-1, "%s: ADC timeout set to %lu jiffies\n", __FUNCTION__,
		ts_dev->adc_buf.timeout);

}

static int __devinit mxc_tsc_probe(struct platform_device *pdev)
{
	int err;
	struct mxc_tsc *ts_dev;
	struct input_dev *input_dev;
	struct resource	*res;
	int irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No mmio resource defined\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ assigned\n");
		return -ENODEV;
	}

	if (!request_mem_region(res->start, resource_size(res),
				"mxc tsc regs")) {
		return -EBUSY;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct mxc_tsc), GFP_KERNEL);
	if (!ts_dev) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto err_release_mem;
	}

	/* allocate conversion buffers separately to prevent
	 * cacheline alignment issues when using DMA */
	ts_dev->adc_data = kzalloc(sizeof(mxc_tsc_adc_fifo), GFP_KERNEL);
	if (ts_dev->adc_data == NULL) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	ts_dev->irq = irq;

	mutex_init(&ts_dev->adc_mutex);
	init_waitqueue_head(&ts_dev->adc_buf.wq);

	ts_dev->adc_buf.reg_base = GCQ_REG_BASE;
	ts_dev->adc_buf.data = ts_dev->adc_data->fifo;

	platform_set_drvdata(pdev, ts_dev);

	input_dev = input_allocate_device();

	ts_dev->reg_base = ioremap(res->start, resource_size(res));
	if (!ts_dev->reg_base) {
		dev_err(&pdev->dev, "Failed to map registers\n");
		err = -ENOMEM;
		goto err_free_dev;
	}

	err = request_irq(ts_dev->irq, mxc_tsc_interrupt, 0,
			pdev->dev.driver->name, ts_dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to install irq handler: %d\n", err);
		goto err_unmap_regs;
	}

	ts_dev->clk = clk_get(&pdev->dev, "tsc_clk");
	if (IS_ERR(ts_dev->clk)) {
		dev_err(&pdev->dev, "Failed to get tsc_clk\n");
		err = PTR_ERR(ts_dev->clk);
		goto err_free_irq;
	}
	mxc_tsc_extmux_config();
	mxc_tsc_config(pdev);

	err = sysfs_create_group(&pdev->dev.kobj, &mxc_tsc_attr_group);
	if (err) {
		dev_warn(&pdev->dev, "Failed to create sysfs attributes: %d\n",
			 err);
	}
	ts_dev->attrs = !err;

	return 0;

err_free_irq:
	free_irq(ts_dev->irq, ts_dev);
err_unmap_regs:
	iounmap(ts_dev->reg_base);
err_free_dev:
	input_free_device(ts_dev->input);
err_free_mem:
	kfree(ts_dev->adc_data);
	kfree(ts_dev);
err_release_mem:
	release_mem_region(res->start, resource_size(res));
	return err;
}

static int __devexit mxc_tsc_remove(struct platform_device *pdev)
{
	struct mxc_tsc *ts_dev = platform_get_drvdata(pdev);
	struct resource *res;

	if (ts_dev->attrs) {
		DBG(1, "%s: Removing sysfs attributes\n", __FUNCTION__);
		sysfs_remove_group(&pdev->dev.kobj, &mxc_tsc_attr_group);
	}
	clk_disable(ts_dev->clk);
	clk_put(ts_dev->clk);

	free_irq(ts_dev->irq, ts_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ts_dev->reg_base);
	release_mem_region(res->start, resource_size(res));

	kfree(ts_dev->adc_data);
	kfree(ts_dev);

	mxc_TSC_extmux_unconfig();

	return 0;
}

#ifdef CONFIG_SUSPEND
static int mxc_tsc_suspend(struct device *dev)
{
	struct mxc_tsc *ts_dev = dev_get_drvdata(dev);

	if (ts_dev->clk_enabled) {
		tsc_clk_disable(ts_dev);
		ts_dev->clk_enabled = 1;
	}
	return 0;
}

static int mxc_tsc_resume(struct device *dev)
{
	struct mxc_tsc *ts_dev = dev_get_drvdata(dev);

	if (ts_dev->clk_enabled) {
		ts_dev->clk_enabled = 0;
		tsc_clk_enable(ts_dev);
	}
	return 0;
}

static struct dev_pm_ops mxc_tsc_pm_ops = {
	.suspend	= mxc_tsc_suspend,
	.resume		= mxc_tsc_resume,
};
#endif

static struct platform_driver mxc_tsc_driver = {
	.driver		= {
		.name	= "mx25-tsc",
		.pm	= __dev_pm_ops_p(mxc_tsc_pm_ops),
	},
	.probe		= mxc_tsc_probe,
	.remove		= __devexit_p(mxc_tsc_remove),
};

static int __init mxc_tsc_init(void)
{
	return platform_driver_register(&mxc_tsc_driver);
}

static void __exit mxc_tsc_exit(void)
{
	platform_driver_unregister(&mxc_tsc_driver);
}

module_init(mxc_tsc_init);
module_exit(mxc_tsc_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i.MX25 General Purpose ADC Driver");
MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
