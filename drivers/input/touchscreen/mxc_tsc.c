/*
 *  Freescale i.MX25 Touch Screen Driver
 *
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

/* FIXME: pressure values, calculated according to the formula
 * found in the i.MX25 Reference Manual, seem rather bogus
 */
#define REPORT_PRESSURE

#include <linux/module.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/mxc_tsc.h>

#include "mxc_tsc.h"

#define TSC_NUM_SAMPLES		4
#define ADC_NUM_SAMPLES		16

#if (TSC_NUM_SAMPLES <= 0) || (TSC_NUM_SAMPLES > 16)
#error Invalid value for TSC_NUM_SAMPLES
#endif

#if (ADC_NUM_SAMPLES <= 0) || (ADC_NUM_SAMPLES > 16)
#error Invalid value for ADC_NUM_SAMPLES
#endif

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

#define DEFAULT_ADC_CLOCK	1666667
#define DEFAULT_RX_VALUE	360

struct mxc_tsc_fifo_entry {
	unsigned int id:4,
		data:12;
};

/* The layout of this structure depends on the setup created by mxc_tsc_config() */
struct mxc_tsc_ts_data {
	struct mxc_tsc_fifo_entry pendown[TSC_NUM_SAMPLES];
	struct mxc_tsc_fifo_entry pos_x[TSC_NUM_SAMPLES];
	struct mxc_tsc_fifo_entry pos_y[TSC_NUM_SAMPLES];
#ifdef REPORT_PRESSURE
	struct mxc_tsc_fifo_entry yn[TSC_NUM_SAMPLES];
	struct mxc_tsc_fifo_entry xp[TSC_NUM_SAMPLES];
#endif
	struct mxc_tsc_fifo_entry pendown2[TSC_NUM_SAMPLES];
};

struct mxc_tsc_adc_data {
	struct mxc_tsc_fifo_entry data[ADC_NUM_SAMPLES];
};

typedef union {
	unsigned int fifo[sizeof(struct mxc_tsc_adc_data) / sizeof(int)];
	struct mxc_tsc_adc_data data;
} mxc_tsc_adc_fifo;

typedef union {
	unsigned int fifo[sizeof(struct mxc_tsc_ts_data) / sizeof(int)];
	struct mxc_tsc_ts_data data;
	struct mxc_tsc_fifo_entry raw[sizeof(struct mxc_tsc_ts_data) / sizeof(int)];
} mxc_tsc_ts_fifo;

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
	char			phys[32];
	void __iomem		*reg_base;
	struct clk		*clk;
	int			irq;
	struct work_struct	work;
	struct timer_list	timer;
	wait_queue_head_t	wq;
	unsigned int		pendown:1,
		clk_enabled:1,
		attrs:1;
	struct mxc_tsc_irqbuf	adc_buf;
	struct mxc_tsc_irqbuf	tsc_buf;
	mxc_tsc_ts_fifo		*tsc_data;
	mxc_tsc_adc_fifo	*adc_data;
	struct mutex		tsc_mutex;
	struct mutex		adc_mutex;
	mxc_tsc_mode		tsc_mode;
	unsigned int		r_xplate;
	/* conversion clock rate in kHz */
	unsigned long		clkrate;
	unsigned short		pressure;
	unsigned short		prev_absx;
	unsigned short		prev_absy;
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

static void tsc_clk_disable(struct mxc_tsc *ts_dev)
{
	if (ts_dev->clk_enabled) {
		mxc_tsc_clr_mask(ts_dev, TGCR, TGCR_IPG_CLK_EN);
		clk_disable(ts_dev->clk);
		ts_dev->clk_enabled = 0;
	}
}

static inline int mxc_tsc_pendown(struct mxc_tsc *ts_dev)
{
	return ts_dev->pendown;
}

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
			DBG(0, "%s: Dropping spurious data sample[%d/%d] on %s queue: %08lx\n",
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
		DBG(0, "%s: Timeout waiting for %s data\n", __FUNCTION__,
			irqbuf == &ts_dev->adc_buf ? "ADC" : "TSC");
		mxc_tsc_set_mask(ts_dev, TCQCR, CQCR_FRST);
		mxc_tsc_clr_mask(ts_dev, TCQCR, CQCR_FRST);
		return -ETIME;
	}
	return 0;
}

static int mxc_tsc_read_adc(struct mxc_tsc *ts_dev, int chan)
{
	int ret;
	unsigned long reg;
	int i;
	struct mxc_tsc_irqbuf *irqbuf = &ts_dev->adc_buf;
	struct mxc_tsc_adc_data *adc_data = &ts_dev->adc_data->data;

	mutex_lock(&ts_dev->adc_mutex);

	memset(adc_data, 0, sizeof(*adc_data));
	if (WARN_ON(irqbuf->irqcount)) {
		irqbuf->irqcount = 0;
	}
	irqbuf->reqcount = ADC_NUM_SAMPLES;

	reg = mxc_tsc_read(ts_dev, GCC0);
	reg = (reg & ~CC_SELIN_MASK) | chan;
	mxc_tsc_write(ts_dev, GCC0, reg);

	/* enable data ready and end of conversion interrupt */
	mxc_tsc_clr_mask(ts_dev, GCQMR,
			CQMR_EOQ_IRQ_MSK |
			CQMR_FDRY_IRQ_MSK |
			CQMR_FOR_IRQ_MSK |
			CQMR_FER_IRQ_MSK);
	/* start conversion */
	mxc_tsc_set_mask(ts_dev, GCQCR, CQCR_FQS);

	ret = mxc_tsc_wait_data(ts_dev, irqbuf);
	if (ret) {
		goto exit;
	}
	irqbuf->irqcount = 0;

	DBG(1, "%s: Read %u words from fifo\n", __FUNCTION__, irqbuf->reqcount);
	for (i = 0; i < irqbuf->reqcount; i++) {
		DBG(1, "%s: data[0x%x]=%4d\n", __FUNCTION__, i,
		    adc_data->data[i].data);
	}
exit:
	mxc_tsc_clr_mask(ts_dev, GCQCR, CQCR_FQS);
	mxc_tsc_set_mask(ts_dev, GCQMR, CQMR_EOQ_IRQ_MSK | CQMR_FDRY_IRQ_MSK);
	mutex_unlock(&ts_dev->adc_mutex);

	return ret;
}

static int mxc_tsc_data_valid(struct mxc_tsc_fifo_entry *data, int num_samples)
{
	int valid = 0;
	int i;

	for (i = 0; i < num_samples; i++) {
		DBG(1, "%s: data[%d]=%d:%d\n", __FUNCTION__, i,
			data[i].id, data[i].data);
		valid |= data[i].data != 0;
	}
	return valid;
}

static int mxc_tsc_get_data(struct mxc_tsc_fifo_entry *data, int num_samples)
{
	int value = 0;
	int count = 0;
	int i;

	for (i = 0; i < num_samples; i++) {
		DBG(2, "%s: data[%d]=%d:%d\n", __FUNCTION__, i,
			data[i].id, data[i].data);
		if (data[i].data == 0) {
			DBG(1, "%s: Skipping value %d\n", __FUNCTION__, i);
			continue;
		}
		if (count == 0) {
			value = data[i].data;
		} else {
			value = (value * count + data[i].data) / (count + 1);
		}
		count++;
	}
	data[0].data = value;
	return value;
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

static ssize_t mxc_tsc_attr_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct mxc_tsc *ts_dev = dev_get_drvdata(dev);
	struct mxc_tsc_attr *mxc_tsc_attr = to_mxc_tsc_attr(attr);
	struct mxc_tsc_adc_data *adc_data = &ts_dev->adc_data->data;

	ret = mxc_tsc_read_adc(ts_dev, mxc_tsc_attr->chan);
	if (ret != 0) {
		dev_err(dev, "%s: Failed to read ADC%d\n", __FUNCTION__,
			((mxc_tsc_attr->chan & CC_SELIN_MASK) >>
				CC_SELIN_SHIFT) - 5);
		return ret;
	}
	ret = sprintf(buf, "%d\n", mxc_tsc_get_data(adc_data->data,
			ADC_NUM_SAMPLES));
	return ret;
}

MXC_TSC_DEV_ATTR(inaux0, S_IRUGO, CC_SELIN_INAUX0, mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(inaux1, S_IRUGO, CC_SELIN_INAUX1, mxc_tsc_attr_get);
MXC_TSC_DEV_ATTR(inaux2, S_IRUGO, CC_SELIN_INAUX2, mxc_tsc_attr_get);

static struct attribute *mxc_tsc_attrs[] = {
	&mxc_tsc_attr_inaux0.attr.attr,
	&mxc_tsc_attr_inaux1.attr.attr,
	&mxc_tsc_attr_inaux2.attr.attr,
	NULL
};

static const struct attribute_group mxc_tsc_attr_group = {
	.attrs = mxc_tsc_attrs,
};

static void mxc_tsc_start_measure(struct mxc_tsc *ts_dev, int force)
{
	unsigned long reg;
	struct mxc_tsc_ts_data *tsc_data = &ts_dev->tsc_data->data;
	struct mxc_tsc_irqbuf *irqbuf = &ts_dev->tsc_buf;
	mxc_tsc_ts_fifo *fifo_data = ts_dev->tsc_data;

	WARN_ON(!(mxc_tsc_read(ts_dev, TCQSR) & CQSR_EMPT));

	memset(tsc_data, 0xee, sizeof(*tsc_data));
	irqbuf->irqcount = 0;
	irqbuf->reqcount = ARRAY_SIZE(fifo_data->fifo);

	reg = mxc_tsc_read(ts_dev, TCQSR);
	if (!(reg & CQSR_EMPT)) {
		DBG(0, "%s: Clearing TSC FIFO\n", __FUNCTION__);
		mxc_tsc_set_mask(ts_dev, TCQCR, CQCR_FRST);
		mxc_tsc_clr_mask(ts_dev, TCQCR, CQCR_FRST);
	}
	mxc_tsc_write(ts_dev, TCQSR, reg);

	if (force) {
		/* change configuration for FQS mode */
		reg = (0x1 << CC_YPLLSW_SHIFT) | (0x1 << CC_XNURSW_SHIFT) |
		      CC_XPULSW;
		mxc_tsc_write(ts_dev, TICR, reg);

		/* FQS */
		reg = mxc_tsc_read(ts_dev, TCQCR);
		reg &= ~CQCR_QSM_MASK;
		reg |= CQCR_QSM_FQS;
		mxc_tsc_write(ts_dev, TCQCR, reg);
		mxc_tsc_write(ts_dev, TCQCR, reg | CQCR_FQS);

		/* enable end of conversion interrupt */
		mxc_tsc_clr_mask(ts_dev, TCQMR, CQMR_EOQ_IRQ_MSK |
				CQMR_FDRY_IRQ_MSK);
	} else {
		/* Config idle for 4-wire */
		mxc_tsc_write(ts_dev, TICR, TSC_4WIRE_TOUCH_DETECT);

		/* Pen interrupt starts new conversion queue */
		reg = mxc_tsc_read(ts_dev, TCQCR);
		reg &= ~CQCR_QSM_MASK;
		reg |= CQCR_QSM_PEN;
		mxc_tsc_write(ts_dev, TCQCR, reg);

		/* PDEN and PDBEN */
		mxc_tsc_set_mask(ts_dev, TGCR, TGCR_PDB_EN | TGCR_PD_EN);

		/* enable end of conversion interrupt */
		mxc_tsc_clr_mask(ts_dev, TCQMR,
				CQMR_EOQ_IRQ_MSK |
				CQMR_FOR_IRQ_MSK |
				CQMR_FER_IRQ_MSK |
				CQMR_FDRY_IRQ_MSK);
	}
}

static int mxc_tsc_read_ts(struct mxc_tsc *ts_dev, int force)
{
	int ret;
	mxc_tsc_ts_fifo *fifo_data = ts_dev->tsc_data;
	struct mxc_tsc_irqbuf *irqbuf = &ts_dev->tsc_buf;
	struct mxc_tsc_ts_data *tsc_data = &ts_dev->tsc_data->data;

	mutex_lock(&ts_dev->tsc_mutex);

	ret = mxc_tsc_wait_data(ts_dev, irqbuf);
	if (ret) {
		goto exit;
	}

	for (ret = 0; ret < irqbuf->reqcount; ret++) {
		struct mxc_tsc_fifo_entry *data = &fifo_data->raw[ret];
		DBG(1, "%s: data[%02x]@%p=%d:%03x (%08x)\n", __FUNCTION__, ret,
			data, data->id, data->data, fifo_data->fifo[ret]);
	}

	ret = tsc_data->pendown[0].data <= 0x600 &&
		tsc_data->pendown2[0].data <= 0x600;
	if (ret) {
		int pos_x = mxc_tsc_get_data(tsc_data->pos_x, TSC_NUM_SAMPLES);
		int pos_y = mxc_tsc_get_data(tsc_data->pos_y, TSC_NUM_SAMPLES);
#ifdef REPORT_PRESSURE
		int xp = mxc_tsc_get_data(tsc_data->xp, TSC_NUM_SAMPLES);
		int yn = mxc_tsc_get_data(tsc_data->yn, TSC_NUM_SAMPLES);
#endif

		DBG(0, "%s: pos_x=%4d pos_y=%4d pd=%d\n",
			__FUNCTION__, pos_x, pos_y, ts_dev->pendown);
		if (pos_x) {
#ifdef REPORT_PRESSURE
			if (mxc_tsc_data_valid(tsc_data->xp, TSC_NUM_SAMPLES)) {
				ts_dev->pressure = ts_dev->r_xplate *
					pos_x * (yn - xp) / xp / 4096;
				DBG(1, "%s: xp=%4d yn=%4d p=%d\n", __FUNCTION__,
					xp, yn, ts_dev->pressure);
				if (ts_dev->pressure > 4095) {
					ts_dev->pressure = 4095;
				}
			} else {
				DBG(0, "%s: Invalid pressure data\n",
					__FUNCTION__);
				ret = -EINVAL;
			}
#else
			ts_dev->pressure = 4095;
#endif
			DBG(0, "%s: Detected PEN DOWN with pressure %4d\n",
			    __FUNCTION__, ts_dev->pressure);
		} else {
			DBG(-1, "%s: Discarding measurement\n", __FUNCTION__);
			ret = -EINVAL;
		}
	} else {
		DBG(0, "%s: Detected PEN UP\n", __FUNCTION__);
		ts_dev->pendown = 0;
	}
exit:
	mutex_unlock(&ts_dev->tsc_mutex);
	return ret;
}

static void mxc_tsc_work(struct work_struct *w)
{
	struct mxc_tsc *ts_dev = container_of(w, struct mxc_tsc, work);
	struct input_dev *input_dev = ts_dev->input;
	struct mxc_tsc_ts_data *tsc_data = &ts_dev->tsc_data->data;
	int ret;

	ret = mxc_tsc_read_ts(ts_dev, 0);
	DBG(0, "%s: mxc_tsc_read_ts() returned %d\n", __FUNCTION__, ret);
	if (ret > 0) {
		DBG(0, "%s: Got sample pd=%d\n", __FUNCTION__,
		    ts_dev->pendown);
		if (mxc_tsc_pendown(ts_dev)) {
			DBG(1, "%s: Reporting PD event %4d @ %4d,%4d\n",
			    __FUNCTION__, ts_dev->pressure,
			    tsc_data->pos_x[0].data,
			    tsc_data->pos_y[0].data);

			input_report_abs(input_dev, ABS_X,
					 tsc_data->pos_x[0].data);
			input_report_abs(input_dev, ABS_Y,
					 tsc_data->pos_y[0].data);
#ifdef REPORT_PRESSURE
			input_report_abs(input_dev, ABS_PRESSURE,
					ts_dev->pressure);
#endif
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_sync(input_dev);

			ts_dev->prev_absx = tsc_data->pos_x[0].data;
			ts_dev->prev_absy = tsc_data->pos_y[0].data;

			mod_timer(&ts_dev->timer, jiffies +
				  msecs_to_jiffies(10));
			return;
		}
	} else if (ret == 0) {
		DBG(1, "%s: Reporting PU event: %4d,%4d\n", __FUNCTION__,
		    ts_dev->prev_absx, ts_dev->prev_absy);
		input_report_abs(input_dev, ABS_X,
				 ts_dev->prev_absx);
		input_report_abs(input_dev, ABS_Y,
				 ts_dev->prev_absy);
#ifdef REPORT_PRESSURE
		input_report_abs(input_dev, ABS_PRESSURE, 0);
#endif
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_sync(input_dev);
	}
#if 1
	mxc_tsc_start_measure(ts_dev, 0);
#else
	mxc_tsc_enable_pendown(ts_dev);
#endif
}

static void mxc_tsc_timer(unsigned long data)
{
	struct mxc_tsc *ts_dev = (void *)data;
	struct mxc_tsc_ts_data *tsc_data = &ts_dev->tsc_data->data;
	struct mxc_tsc_irqbuf *irqbuf = &ts_dev->tsc_buf;

	/* trigger a new conversion */
	memset(tsc_data, 0xed, sizeof(*tsc_data));
	irqbuf->irqcount = 0;

	mxc_tsc_start_measure(ts_dev, 1);
}

static irqreturn_t mxc_tsc_interrupt(int irq, void *dev)
{
	struct mxc_tsc *ts_dev = dev;
	//struct input_dev *input_dev = ts_dev->input;
	unsigned long reg;
	unsigned long status = mxc_tsc_read(ts_dev, TGSR);

	DBG(3, "%s: TGSR=%08lx\n", __FUNCTION__, status);

	if (status & TGSR_TCQ_INT) {
		unsigned long mask = mxc_tsc_read(ts_dev, TCQMR);

		reg = mxc_tsc_read(ts_dev, TCQSR);
		DBG(1, "%s: TCQSR=%08lx TCQMR=%08lx:%08lx\n", __FUNCTION__,
			reg, mask, reg & ~mask);
		reg &= ~mask;
		mxc_tsc_write(ts_dev, TCQSR, reg);
		if (reg & (CQSR_FOR | CQSR_FER)) {
			DBG(-1, "%s: Fifo overrun on TSC queue\n",
				__FUNCTION__);
			mxc_tsc_set_mask(ts_dev, TCQCR, CQCR_FRST);
			mxc_tsc_clr_mask(ts_dev, TCQCR, CQCR_FRST);
		} if (reg & CQSR_FDRY) {
			struct mxc_tsc_irqbuf *irqbuf = &ts_dev->tsc_buf;

			mxc_tsc_clr_mask(ts_dev, TCQCR, CQCR_FQS);
			mxc_tsc_read_fifo(ts_dev, irqbuf);
		}
		if (reg & CQSR_PD) {
			ts_dev->pendown = 1;

			/* disable pen down detect */
			mxc_tsc_clr_mask(ts_dev, TGCR, TGCR_PD_EN);

			/* schedule new measurement */
			schedule_work(&ts_dev->work);
		}
		if (reg & CQSR_EOQ) {
			mxc_tsc_clr_mask(ts_dev, TCQCR, CQCR_FQS);

			/* disable end of conversion interrupt */
			mxc_tsc_set_mask(ts_dev, TCQMR, CQMR_EOQ_IRQ_MSK);

			DBG(1, "%s: Got EOQ interrupt\n", __FUNCTION__);
			schedule_work(&ts_dev->work);
		}
	}
	if (status & TGSR_GCQ_INT) {
		unsigned long mask = mxc_tsc_read(ts_dev, GCQMR);

		reg = mxc_tsc_read(ts_dev, GCQSR);
		DBG(1, "%s: GCQSR=%08lx GCQMR=%08lx:%08lx\n", __FUNCTION__,
			reg, mask, reg & ~mask);
		reg &= ~mask;
		mxc_tsc_write(ts_dev, GCQSR, reg);
		if (reg & (CQSR_FOR | CQSR_FER)) {
			DBG(-1, "%s: Fifo overrun on ADC queue\n",
				__FUNCTION__);
			mxc_tsc_set_mask(ts_dev, TCQCR, CQCR_FRST);
			mxc_tsc_clr_mask(ts_dev, TCQCR, CQCR_FRST);
		} else if (reg & CQSR_FDRY) {
			struct mxc_tsc_irqbuf *irqbuf = &ts_dev->adc_buf;

			mxc_tsc_read_fifo(ts_dev, irqbuf);
		}
		if (reg & CQSR_EOQ) {
			/* disable end of conversion interrupt */
			mxc_tsc_set_mask(ts_dev, GCQMR, CQMR_EOQ_IRQ_MSK);
		}
	}
	return IRQ_HANDLED;
}

#define SETTLE_PCHG	0
#define SETTLE_DET	16
#define SETTLE_MEAS	32

static void mxc_tsc_4wire_config(struct mxc_tsc *ts_dev)
{
	unsigned long reg;
	int lastitemid;

	/* Configure 4-wire */
	reg = TSC_4WIRE_PRECHARGE;
	reg |= CC_IGS;
	mxc_tsc_write(ts_dev, TCC0, reg);

	reg = TSC_4WIRE_TOUCH_DETECT;
	reg |= (TSC_NUM_SAMPLES - 1) << CC_NOS_SHIFT;
	reg |= SETTLE_DET << CC_SETTLING_TIME_SHIFT;
	mxc_tsc_write(ts_dev, TCC1, reg);

	reg = TSC_4WIRE_X_MEASURE;
	reg |= (TSC_NUM_SAMPLES - 1) << CC_NOS_SHIFT;
	reg |= SETTLE_MEAS << CC_SETTLING_TIME_SHIFT;
	mxc_tsc_write(ts_dev, TCC2, reg);

	reg = TSC_4WIRE_Y_MEASURE;
	reg |= (TSC_NUM_SAMPLES - 1) << CC_NOS_SHIFT;
	reg |= SETTLE_MEAS << CC_SETTLING_TIME_SHIFT;
	mxc_tsc_write(ts_dev, TCC3, reg);

	reg = TSC_4WIRE_YN_MEASURE;
	reg |= (TSC_NUM_SAMPLES - 1) << CC_NOS_SHIFT;
	reg |= SETTLE_MEAS << CC_SETTLING_TIME_SHIFT;
	mxc_tsc_write(ts_dev, TCC4, reg);

	reg = TSC_4WIRE_XP_MEASURE;
	reg |= (TSC_NUM_SAMPLES - 1) << CC_NOS_SHIFT;
	reg |= SETTLE_MEAS << CC_SETTLING_TIME_SHIFT;
	mxc_tsc_write(ts_dev, TCC5, reg);

#ifdef REPORT_PRESSURE
	reg = (TCQ_ITEM_TCC0 << CQ_ITEM0_SHIFT) |
		(TCQ_ITEM_TCC1 << CQ_ITEM1_SHIFT) |
		(TCQ_ITEM_TCC2 << CQ_ITEM2_SHIFT) |
		(TCQ_ITEM_TCC3 << CQ_ITEM3_SHIFT) |
		(TCQ_ITEM_TCC4 << CQ_ITEM4_SHIFT) |
		(TCQ_ITEM_TCC5 << CQ_ITEM5_SHIFT) |
		(TCQ_ITEM_TCC0 << CQ_ITEM6_SHIFT) |
		(TCQ_ITEM_TCC1 << CQ_ITEM7_SHIFT);
	lastitemid = 7;

	/* ADC conversion requires 14 clock cycles per sample
	 * plus the settling time programmed in the TICR registers.
	 * Add 1 extra jiffy to make sure the timeout is > 0
	 */
	ts_dev->tsc_buf.timeout = msecs_to_jiffies(
		((6 * TSC_NUM_SAMPLES * 14) +
			(2 * (SETTLE_PCHG * 8 + 1)) +
			(4 * (SETTLE_MEAS * 8 + 1)) +
			(2 * (SETTLE_DET * 8 + 1))) /
		ts_dev->clkrate / 1000) + 1;
#else
	reg = (TCQ_ITEM_TCC0 << CQ_ITEM0_SHIFT) |
		(TCQ_ITEM_TCC1 << CQ_ITEM1_SHIFT) |
		(TCQ_ITEM_TCC2 << CQ_ITEM2_SHIFT) |
		(TCQ_ITEM_TCC3 << CQ_ITEM3_SHIFT) |
		(TCQ_ITEM_TCC0 << CQ_ITEM4_SHIFT) |
		(TCQ_ITEM_TCC1 << CQ_ITEM5_SHIFT);
	lastitemid = 5;

	/* ADC conversion requires 14 clock cycles per sample
	 * plus the settling time programmed in the TICR registers.
	 * Add 1 extra jiffy to make sure the timeout is > 0
	 */
	ts_dev->tsc_buf.timeout = msecs_to_jiffies(
		((5 * TSC_NUM_SAMPLES * 14) +
			(2 * (SETTLE_PCHG * 8 + 1)) +
			(4 * (SETTLE_MEAS * 8 + 1)) +
			(2 * (SETTLE_DET * 8 + 1))) /
		ts_dev->clkrate / 1000) + 1;
#endif
	DBG(-1, "%s: TSC timeout set to %lu jiffies\n", __FUNCTION__,
		ts_dev->tsc_buf.timeout);
	mxc_tsc_write(ts_dev, TCQ_ITEM_7_0, reg);

	reg = mxc_tsc_read(ts_dev, TCQCR);
	reg &= ~(CQCR_FIFOWATERMARK_MASK | CQCR_LAST_ITEM_ID_MASK);
	reg |= lastitemid << CQCR_FIFOWATERMARK_SHIFT;
	reg |= lastitemid << CQCR_LAST_ITEM_ID_SHIFT;
	reg &= ~CQCR_PD_MSK;
	mxc_tsc_write(ts_dev, TCQCR, reg);

	/* clear status bits */
	reg = mxc_tsc_read(ts_dev, TCQSR);
	mxc_tsc_write(ts_dev, TCQSR, reg);

	mxc_tsc_clr_mask(ts_dev, TCQMR,
			CQMR_PD_IRQ_MSK |
			CQMR_EOQ_IRQ_MSK |
			CQMR_FDRY_IRQ_MSK |
			CQMR_FOR_IRQ_MSK |
			CQMR_FER_IRQ_MSK);

	/* Config idle for 4-wire */
	mxc_tsc_write(ts_dev, TICR, TSC_4WIRE_TOUCH_DETECT);
}

static void mxc_tsc_adc_config(struct mxc_tsc *ts_dev)
{
	unsigned long reg;

	reg = ((ADC_NUM_SAMPLES - 1) << CQCR_FIFOWATERMARK_SHIFT) |
		(0 << CQCR_LAST_ITEM_ID_SHIFT) |
		CQCR_QSM_FQS;
	mxc_tsc_write(ts_dev, GCQCR, reg);

	reg = ((ADC_NUM_SAMPLES - 1) << CC_NOS_SHIFT) |
		(SETTLE_MEAS << CC_SETTLING_TIME_SHIFT) |
		CC_YPLLSW_OFF | CC_XNURSW_OFF | CC_XPULSW |
		CC_SEL_REFP_INT | CC_SEL_REFN_AGND;
	mxc_tsc_write(ts_dev, GCC0, reg);

	/* ADC conversion requires 14 clock cycles per sample
	 * plus the settling time programmed in the TICR registers.
	 * Add 1 extra jiffy to make sure the timeout is > 0
	 */
	ts_dev->adc_buf.timeout = msecs_to_jiffies(
		((ADC_NUM_SAMPLES * 14) +
			(SETTLE_MEAS * 8 + 1)) /
		ts_dev->clkrate / 1000) + 1;
	DBG(-1, "%s: ADC timeout set to %lu jiffies\n", __FUNCTION__,
		ts_dev->adc_buf.timeout);
}

static void mxc_tsc_config(struct platform_device *pdev)
{
	struct mxc_tsc *ts_dev = platform_get_drvdata(pdev);
	struct mxc_tsc_pdata *pdata = pdev->dev.platform_data;
	unsigned int tgcr;
	unsigned int pdbt = TGCR_PDBTIME128;
	unsigned int pdben = 1;
	unsigned int intref = 1;
	unsigned int adc_clk = DEFAULT_ADC_CLOCK;
	unsigned long ipg_clk;
	unsigned int clkdiv;
	unsigned int hsync_en = 0;
	unsigned int hsync_pol = 0;

	if (pdata) {
		pdbt = pdata->pen_debounce_time - 1;
		if (pdbt > 31) {
			dev_dbg(&pdev->dev, "Pen debounce time %d out of range[0..32]; using max. value\n",
				pdata->pen_debounce_time);
		}
		pdben = pdata->pen_debounce_time > 0;
		intref = pdata->intref;
		if (pdata->adc_clk > 0) {
			adc_clk = pdata->adc_clk;
		}
		ts_dev->r_xplate = pdata->r_xplate;
		hsync_en = pdata->hsyncen;
		hsync_pol = pdata->hsyncpol;
		DBG(-1, "%s: pdbt=%d intref=%d r_xplate=%d hsync_en=%d hsync_pol=%d\n",
			__FUNCTION__, pdbt + 1, intref, ts_dev->r_xplate,
			hsync_en, hsync_pol);
	} else {
		dev_dbg(&pdev->dev, "No platform_data; using defaults\n");
	}
	if (ts_dev->r_xplate == 0) {
		ts_dev->r_xplate = DEFAULT_RX_VALUE;
		DBG(0, "%s: Assuming default Rx value of %u Ohms\n",
		    __FUNCTION__, ts_dev->r_xplate);
	}
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

	tgcr = ((pdbt << TGCR_PDBTIME_SHIFT) & TGCR_PDBTIME_MASK) | /* pen debounce time */
		(pdben * TGCR_PDB_EN) | /* pen debounce enable */
		(intref * TGCR_INTREFEN) | /* pen debounce enable */
		(hsync_en * TGCR_HSYNC_EN) | /* sync conversion with hsync */
		(hsync_pol * TGCR_HSYNC_POL) | /* HSYNC polarity */
		TGCR_POWER_SAVE | /* Switch TSC on */
		TGCR_PD_EN |	/* Enable Pen Detect */
		((clkdiv << TGCR_ADCCLKCFG_SHIFT) & TGCR_ADCCLKCFG_MASK);

	/* reset TSC */
	mxc_tsc_write(ts_dev, TGCR, TGCR_TSC_RST);
	while (mxc_tsc_read(ts_dev, TGCR) & TGCR_TSC_RST) {
		cpu_relax();
	}
	mxc_tsc_write(ts_dev, TGCR, tgcr);

	tsc_clk_enable(ts_dev);
	mxc_tsc_4wire_config(ts_dev);
	mxc_tsc_adc_config(ts_dev);
#if 1
	mxc_tsc_start_measure(ts_dev, 0);
#else
	mxc_tsc_enable_pendown(ts_dev);
#endif
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
	ts_dev->tsc_data = kzalloc(sizeof(mxc_tsc_ts_fifo), GFP_KERNEL);
	ts_dev->adc_data = kzalloc(sizeof(mxc_tsc_adc_fifo), GFP_KERNEL);
	if (ts_dev->tsc_data == NULL || ts_dev->adc_data == NULL) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	ts_dev->irq = irq;
	INIT_WORK(&ts_dev->work, mxc_tsc_work);
	mutex_init(&ts_dev->tsc_mutex);
	mutex_init(&ts_dev->adc_mutex);
	setup_timer(&ts_dev->timer, mxc_tsc_timer, (unsigned long)ts_dev);
	init_waitqueue_head(&ts_dev->wq);
	init_waitqueue_head(&ts_dev->tsc_buf.wq);
	init_waitqueue_head(&ts_dev->adc_buf.wq);

	ts_dev->tsc_buf.reg_base = TCQ_REG_BASE;
	ts_dev->adc_buf.reg_base = GCQ_REG_BASE;
	ts_dev->tsc_buf.data = ts_dev->tsc_data->fifo;
	ts_dev->adc_buf.data = ts_dev->adc_data->fifo;

	platform_set_drvdata(pdev, ts_dev);

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "Failed to allocate input device\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

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

	ts_dev->input = input_dev;

	snprintf(ts_dev->phys, sizeof(ts_dev->phys),
		 "%s/input0", dev_name(&pdev->dev));

	input_dev->name = "mxc touch screen controller";
	input_dev->phys = ts_dev->phys;
	input_dev->dev.parent = &pdev->dev;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_X, 0, 0xFFF, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 0xFFF, 0, 0);
#ifdef REPORT_PRESSURE
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 0xFFF, 0, 0);
#endif
	mxc_tsc_config(pdev);

	/* All went ok, so register to the input system */
	err = input_register_device(input_dev);
	if (err)
		goto err_fail;

	err = sysfs_create_group(&pdev->dev.kobj, &mxc_tsc_attr_group);
	if (err) {
		dev_warn(&pdev->dev, "Failed to create sysfs attributes: %d\n",
			 err);
	}
	ts_dev->attrs = !err;

	return 0;

err_fail:
	clk_disable(ts_dev->clk);
	clk_put(ts_dev->clk);
err_free_irq:
	free_irq(ts_dev->irq, ts_dev);
err_unmap_regs:
	iounmap(ts_dev->reg_base);
err_free_dev:
	input_free_device(ts_dev->input);
err_free_mem:
	kfree(ts_dev->tsc_data);
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
	del_timer_sync(&ts_dev->timer);
	input_unregister_device(ts_dev->input);

	clk_disable(ts_dev->clk);
	clk_put(ts_dev->clk);

	free_irq(ts_dev->irq, ts_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ts_dev->reg_base);
	release_mem_region(res->start, resource_size(res));

	kfree(ts_dev->tsc_data);
	kfree(ts_dev->adc_data);
	kfree(ts_dev);
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
MODULE_DESCRIPTION("i.MX25 TouchScreen Driver");
MODULE_AUTHOR("Lothar Wassmann <LW@KARO-electronics.de>");
