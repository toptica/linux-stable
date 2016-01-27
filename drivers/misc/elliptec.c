/*
 * elliptec.c: PWM Driver for Elliptec motors
 *
 * Author: Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 * The driver installs a new device class /sys/class/elliptec where
 * up to 4 i.MX25 PWMs can be used for driving Elliptec motors.
 * For each PWM used a directory is generated with several attributes
 *
 * frequency :	PMW frequency in Hz
 * duty:		duty cycle in 0.01% (i.e. 0..10000)
 * duration:	burst duration in us
 * 				writing: -1 continuous, 0 turn off, >0 start burst
 * 				reading: -1 = continuous, >=0 remaining time in us
 * arrived:		read-only, blocking if burst is running, non-blocking
 * 				for continuous mode
 * 				1 : arrived, 0: not arrived
 * ramp_us:		linear ramps at the beginning and end of the burst
 * 				are configured with this attributes, which gives
 * 				the time in us needed to ramp up from 0% to 100%
 * 				duty cycle
 *
 * Licensed under the GPL-2 or later.
 */


/* FIXME: little confusion usage of long and long long ... check necessity */


#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/sysdev.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/elliptec.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <mach/gpio.h>
#include <mach/iomux.h>


#define DRIVER_NAME			"elliptec"
#define DRIVER_VERSION			"0.1"

static struct class *elliptec_class;

DECLARE_RWSEM(elliptecs_list_lock);
LIST_HEAD(elliptecs_list);


struct elliptec_classdev {
	const char		*name;

	struct device		*dev;
	struct elliptec_pwm_data *pwm;
	struct list_head	 node;
};

struct elliptec_pwm_data {
	struct elliptec_classdev cdev;
	void __iomem *mmio_base;
	unsigned int irq;
	struct clk	*clk;
	spinlock_t	lock;	/* for protecting data below */
	struct completion completion; /* for "arrived" attribute */
	u32 cr_base;		/* PWMCR config for prescaler and clk */
	int	frequency;		/* in Hz */
	int period;			/* in ns for calculation*/
	int period_cycles;
	int	duty;			/* 0..10000 */
	int duty_cycles;	/* duration prescaler clk cycles for PWMSAR register */
	int ramp_us;		/* time in us needed to ramp up to 100% duty */
	long ramp_periods;  /* the same in terms of PWM periods*/
	long phase_periods[3]; /* duration of phases in PWM periods */
	long countdown;		/* periods to go in this state*/
	long irqs;			/* for debugging only */
	int state;			/* countdown of states/phase */
	int waiting;
#define ELLI_RAMP_UP	2
#define ELLI_CONST		1
#define ELLI_RAMP_DOWN	0
#define ELLI_FINISHED	-1
};



#define MX25_PWMCR                 0x00    /* PWM Control Register */
#define MX25_PWMSR                 0x04    /* PWM Status Register */
#define MX25_PWMIR                 0x08    /* PWM Interrupt Register */
#define MX25_PWMSAR                0x0C    /* PWM Sample Register */
#define MX25_PWMPR                 0x10    /* PWM Period Register */
#define MX25_PWMCNR                 0x14   /* PWM Counter Register */

#define MX25_PWMCR_SWR			  (1 << 3)
#define MX25_PWMCR_PRESCALER(x)    (((x - 1) & 0xFFF) << 4)
#define MX25_PWMCR_CLKSRC_IPG_HIGH (2 << 16)
#define MX25_PWMCR_CLKSRC_IPG      (1 << 16)
#define MX25_PWMCR_REPEAT(x)		  (((x) & 0x3) << 1)
#define MX25_PWMCR_WATERMARK(x)	  (((x) & 0x3) << 26)
#define MX25_PWMCR_EN              (1 << 0)

#define MX25_PWMIR_FIE			  (1 << 0)
#define MX25_PWMIR_RIE			  (1 << 1)

#define MX25_PWMSR_FE			  (1 << 3)
#define MX25_PWMSR_ROV			  (1 << 4)
#define MX25_PWMSR_CMP			  (1 << 5)
#define MX25_PWMSR_FEW			  (1 << 6)
#define MX25_PWMSR_FIFOAV_MASK	  (7 << 0)
#define MX25_PWMSR_STATUS_MASK	  (MX25_PWMSR_FE | MX25_PWMSR_ROV \
		| MX25_PWMSR_CMP | MX25_PWMSR_FEW)

#define DUTYMAX		10000		/* means 100% duty cylce */
/* ------------------------------------------------------------------------- */

int pwm_config_frequency(struct elliptec_pwm_data *pwm, int duty_10000, int frequency_hz);
int pwm_change_duty(struct elliptec_pwm_data *pwm, int duty_10000);

static ssize_t elliptec_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;

	return sprintf(buf, "%u\n", pwm->frequency);
}

static ssize_t elliptec_frequency_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long newfreq = simple_strtoul(buf, &after, 10);
	//unsigned long flags;
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		//spin_lock_irqsave(pwm->lock,flags);
		spin_lock(pwm->lock);
		ret=pwm_config_frequency(pwm,pwm->duty, newfreq);
		//spin_unlock_irqrestore(pwm->lock,flags);
		spin_unlock(pwm->lock);
		if (ret)
			return ret;
		if (pwm->countdown<0)
			writel(pwm->duty_cycles, pwm->mmio_base + MX25_PWMSAR);
		ret = count;
	}

	return ret;
}

static ssize_t elliptec_duty_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;

	unsigned long period_cycles, duty_cycles;

	if (pwm->countdown==0)
		return sprintf(buf, "%d\n", pwm->duty);

	period_cycles = readl(pwm->mmio_base + MX25_PWMPR);
	duty_cycles = readl(pwm->mmio_base + MX25_PWMSAR);

	return sprintf(buf, "%lu\n", (unsigned long) duty_cycles * DUTYMAX/period_cycles);
}

static ssize_t elliptec_duty_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long newduty = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		if ((ret = pwm_change_duty(pwm, newduty)))
			return ret;
		ret = count;
	}

	return ret;
}

static ssize_t elliptec_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	signed long long countdown = pwm->countdown;
	if (countdown>0) {
		countdown = countdown * pwm->period;
		do_div(countdown,1000);
		return sprintf(buf, "%llu\n", countdown);
	}
	return sprintf(buf, "%lld\n", countdown);
}

static ssize_t elliptec_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	char *after;
	long newduration = simple_strtol(buf, &after, 10);
	unsigned long long tmp;
	long int countdown;
	//unsigned long flags;
	unsigned long ramp;
	u32 cr,sr,i,dcr,wm,repeat;

	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count != size)
		return -EINVAL;

	//spin_lock_irqsave(pwm->lock,flags);
	spin_lock(pwm->lock);

	pwm->irqs = 0;
	/* writing  zero will stop any PWM activity */
	if (newduration == 0) {
		writel(0,pwm->mmio_base + MX25_PWMIR); /* disable all interrupts */
		writel(0,pwm->mmio_base + MX25_PWMSAR); /* zero on time */

		//writel(pwm->cr_base,pwm->mmio_base + MX25_PWMCR);
		pwm->countdown =0;
		if (pwm->waiting>0)
			complete_all(&pwm->completion);
		//spin_unlock_irqrestore(pwm->lock,flags);
		spin_unlock(pwm->lock);
		return count;
	}

	/* writing a negative value will start continuous PWM output */
	if (newduration <0) {
		pwm_config_frequency(pwm,pwm->duty,pwm->frequency);
		writel(pwm->duty_cycles, pwm->mmio_base + MX25_PWMSAR);
		cr = readl(pwm->mmio_base + MX25_PWMCR) | MX25_PWMCR_EN;
		writel(cr, pwm->mmio_base + MX25_PWMCR);
		if (pwm->countdown>0) {/* from earlier call */
			pwm->countdown =-1;
			if (pwm->waiting>0)
				complete_all(&pwm->completion);
		}
		pwm->countdown =-1;
		//spin_unlock_irqrestore(pwm->lock,flags);
		spin_unlock(pwm->lock);
		return count;
	}

	if (pwm->countdown>0 && pwm->waiting>0) /* from earlier call */
		complete_all(&pwm->completion);
	init_completion(&pwm->completion);
	pwm->waiting = 0;

	/* configure period and duty first */
	//writel(pwm->cr_base, pwm->mmio_base + MX25_PWMCR);
	pwm_config_frequency(pwm,pwm->duty,pwm->frequency);

	tmp = (unsigned long long) newduration * 1000;
	if (tmp < 2*pwm->period)
		countdown = 2;
	else {
		do_div(tmp,pwm->period);
		countdown = tmp;
	}

	/* prepare all phases */
	ramp = pwm->duty_cycles * pwm->ramp_periods / pwm->period_cycles;

	if (ramp*2 >= countdown) {
		pwm->phase_periods[2] = countdown/2; /* ramp up */
		pwm->phase_periods[1] = 0;			 /* const */
		pwm->phase_periods[0] = countdown/2; /* ramp down */
	} else {
		pwm->phase_periods[2] = ramp;
		pwm->phase_periods[1] = countdown - 2*ramp;
		pwm->phase_periods[0] = ramp;
	}

	/* evtl. skip first phases if duration is zero */
	for (pwm->state = ELLI_RAMP_UP;(pwm->state > ELLI_FINISHED) && (!pwm->phase_periods[pwm->state]); pwm->state--);
	if (pwm->state>=0)
		countdown = pwm->phase_periods[pwm->state];

	dcr=1;
	repeat=0;
	wm=3;


	if (pwm->state == ELLI_CONST) {
		/* on plateau optimize for minimum IRQ use */
		if (countdown>=8) {
			dcr=8;
			repeat = 3;
			wm=3;
		} else if (countdown>=4) {
			dcr=4;
			repeat = 2;
			wm=3;
		}
	}
	/*
	 printk(KERN_INFO "dcr = %d, repeat = %d, wm=%d, countdown = %d\n",dcr,repeat,wm,countdown);
	 */

	writel(MX25_PWMSR_STATUS_MASK, pwm->mmio_base + MX25_PWMSR);
	writel(0  ,pwm->mmio_base + MX25_PWMIR);
	/* pwm off for a moment ... */
	cr = pwm->cr_base | MX25_PWMCR_WATERMARK(wm) | MX25_PWMCR_REPEAT(repeat);
	writel(cr,pwm->mmio_base + MX25_PWMCR);
	/* now enable PWM */
	cr |= MX25_PWMCR_EN;
	writel(cr,pwm->mmio_base + MX25_PWMCR);

	/* depending on phase fill FIFO with increasing, decreasing or constant value */
	sr = readl(pwm->mmio_base + MX25_PWMSR) & MX25_PWMSR_FIFOAV_MASK;
	if (likely(pwm->state==ELLI_RAMP_DOWN))
		for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
			writel(pwm->period_cycles*countdown/pwm->ramp_periods,
				   pwm->mmio_base + MX25_PWMSAR);
			countdown -= dcr;
		}
	else if (likely(pwm->state==ELLI_RAMP_UP))
		for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
			writel(pwm->period_cycles* \
				  (pwm->phase_periods[ELLI_RAMP_UP]-countdown)/pwm->ramp_periods, \
				   pwm->mmio_base + MX25_PWMSAR);
			countdown -= dcr;
		}
	else
		for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
			writel(pwm->duty_cycles, pwm->mmio_base + MX25_PWMSAR);
			countdown -= dcr;
		}

	pwm->countdown = countdown;

	writel(MX25_PWMIR_FIE ,pwm->mmio_base + MX25_PWMIR);



	//spin_unlock_irqrestore(pwm->lock,flags);
	spin_unlock(pwm->lock);
	return count;
}

static ssize_t elliptec_arrived_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	if (pwm->countdown<0)
		return sprintf(buf, "0\n");
	if (pwm->countdown>0) {
		spin_lock(pwm->lock);
		pwm->waiting++;
		spin_unlock(pwm->lock);
		wait_for_completion(&pwm->completion);
		spin_lock(pwm->lock);
		pwm->waiting--;
		spin_unlock(pwm->lock);
	}
	return sprintf(buf, "1\n");
}

static ssize_t elliptec_ramp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;

	return sprintf(buf, "%lu\n", (unsigned long) pwm->ramp_us);
}

static ssize_t elliptec_ramp_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long newramp = simple_strtoul(buf, &after, 10);
	//unsigned long flags;
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		//spin_lock_irqsave(pwm->lock,flags);
		spin_lock(pwm->lock);
		pwm->ramp_us = newramp;
		pwm->ramp_periods = (long) newramp * 1000 / pwm->period;
		//spin_unlock_irqrestore(pwm->lock,flags);
		spin_unlock(pwm->lock);
		ret = count;
	}

	return ret;
}

static ssize_t elliptec_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	return sprintf(buf, "cntdwn: %lu, irqs: %lu, state: %d, phases:%lu %lu %lu, waiting:%d\n", \
			pwm->countdown, pwm->irqs, pwm->state, \
			pwm->phase_periods[0],pwm->phase_periods[1],pwm->phase_periods[2], \
			pwm->waiting);
}


static struct device_attribute dev_attributes[]= {
		{
			.attr = {.name = "frequency_hz", .mode = 0644 },
			.show	= elliptec_frequency_show,
			.store	= elliptec_frequency_store,
		},{
			.attr = {.name = "duty_10000", .mode = 0644 },
			.show	= elliptec_duty_show,
			.store	= elliptec_duty_store,
		},{
			.attr = {.name = "duration_us", .mode = 0644 },
			.show	= elliptec_duration_show,
			.store	= elliptec_duration_store,
		},{
			.attr = {.name = "arrived", .mode = 0644 },
			.show	= elliptec_arrived_show,
			.store	= NULL,
		},{
			.attr = {.name = "ramp_us", .mode = 0644 },
			.show	= elliptec_ramp_show,
			.store	= elliptec_ramp_store,
		},{
			.attr = {.name = "debug", .mode = 0644 },
			.show	= elliptec_debug_show,
			.store	= NULL,
		},
};

#define ATTR_NUMBER 	ARRAY_SIZE(dev_attributes)


/**
 * elliptec_classdev_register - register a new object of elliptec_classdev class.
 * @parent: The device to register.
 * @elliptec_cdev: the elliptec_classdev structure for this device.
 */
int elliptec_classdev_register(struct device *parent, struct elliptec_classdev *elliptec_cdev)
{
	int rc,i;

	elliptec_cdev->dev = device_create(elliptec_class, parent, 0, elliptec_cdev,
				      "%s", elliptec_cdev->name);
	if (IS_ERR(elliptec_cdev->dev))
		return PTR_ERR(elliptec_cdev->dev);


	/* register the attributes */
	for(i=0;i < ATTR_NUMBER;i++) {
		rc = device_create_file(elliptec_cdev->dev, &dev_attributes[i]);
		if (rc)
			goto err_out;
	}

	/* add to the list of elliptecs */
	down_write(&elliptecs_list_lock);
	list_add_tail(&elliptec_cdev->node, &elliptecs_list);
	up_write(&elliptecs_list_lock);

	printk(KERN_INFO "Registered elliptec device: %s\n",
			elliptec_cdev->name);

	return 0;
err_out:
	for(i--; i>=0;i--)
		device_remove_file(elliptec_cdev->dev, &dev_attributes[i]);
	device_unregister(elliptec_cdev->dev);
	return rc;
}

/**
 * elliptec_classdev_unregister - unregisters a object of elliptec_properties class.
 * @elliptec_cdev: the elliptec device to unregister
 *
 * Unregisters a previously registered via elliptec_classdev_register object.
 */
void elliptec_classdev_unregister(struct elliptec_classdev *elliptec_cdev)
{

	int i;
	printk(KERN_DEBUG "%s: removing attributes of %s\n",__FUNCTION__,elliptec_cdev->name);
	for(i=0;i < ATTR_NUMBER;i++)
		device_remove_file(elliptec_cdev->dev, &dev_attributes[i]);

	down_write(&elliptecs_list_lock);
	list_del(&elliptec_cdev->node);
	up_write(&elliptecs_list_lock);

	device_unregister(elliptec_cdev->dev);
}


/* -------------------------------------------------------------------------
 *
 * real pwm stuff
 *
 * ------------------------------------------------------------------------- */

irqreturn_t elliptec_fifo_irq(int irq, void *dev_id)
{
	struct elliptec_pwm_data *pwm = (struct elliptec_pwm_data *) dev_id;
	u32 sr,cr,i,repeat,wm;
	u32 dcr = 1;
	int state = -1;
	long countdown;
	//unsigned long flags;

	if (!pwm) { /* should not happen */
		printk(KERN_INFO "elliptec IRQ routine called without data (irq %d)\n",irq);
		return IRQ_HANDLED;
	}

	writel(0 ,pwm->mmio_base + MX25_PWMIR);

	//spin_lock_irqsave(pwm->lock,flags);
	spin_lock(pwm->lock);
	countdown = pwm->countdown;

	if (countdown<0) { /* should not happen */
		goto finished;
	}
	pwm->irqs++;

	if (countdown==0) {
		/* phase finished, find next applicable phase/state ... */
		for (state = pwm->state-1;(state > ELLI_FINISHED) && (!pwm->phase_periods[state]); state--);
		/* in final state loop  until FIFO is empty and then swich off ... */
		if (state <= ELLI_FINISHED) {
			/* if already final 0 is found, we are finished */
			if (readl(pwm->mmio_base + MX25_PWMSAR)==0)
				goto finished;

			pwm->state = state;
			/* enqueue final 0s, the next IRQ should then finish everything */
			sr = readl(pwm->mmio_base + MX25_PWMSR) & MX25_PWMSR_FIFOAV_MASK;
			for (i=sr ; i < 4; i++)
				writel(0,pwm->mmio_base + MX25_PWMSAR);
			writel(MX25_PWMSR_STATUS_MASK,pwm->mmio_base + MX25_PWMSR);
			writel(MX25_PWMIR_FIE ,pwm->mmio_base + MX25_PWMIR);
			//spin_unlock_irqrestore(pwm->lock,flags);
			spin_unlock(pwm->lock);
			return IRQ_HANDLED;
		}
		/* ok, there is a another phase to be executed .. */
		pwm->state = state;
		pwm->countdown = countdown = pwm->phase_periods[state];
	} else
		state = pwm->state;


	/*
	 * trying to minimize use of IRQs by making use of REPEAT
	 * and WATERMARK feature
	 */
	dcr=1;			/* number periods per FIFO enqueue */
	repeat=0;		/* REPEAT flag for PWM */
	wm=3;			/* WATERMARK level */
	if (state == ELLI_CONST) {
		if (countdown>=8) {
			dcr=8;
			repeat = 3;
			wm=3;
		} else if (countdown>=4) {
			dcr=4;
			repeat = 2;
			wm=3;
		}
	}
	cr = pwm->cr_base | MX25_PWMCR_WATERMARK(wm) | MX25_PWMCR_REPEAT(repeat) | MX25_PWMCR_EN;
	writel(cr,pwm->mmio_base + MX25_PWMCR);

	sr = readl(pwm->mmio_base + MX25_PWMSR) & MX25_PWMSR_FIFOAV_MASK;
	writel(MX25_PWMSR_STATUS_MASK,pwm->mmio_base + MX25_PWMSR);

	/* depending on phase fill FIFO with increasing, decreasing or constant values */
	if (likely(pwm->state==ELLI_RAMP_DOWN))
		for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
			writel(pwm->period_cycles*countdown/pwm->ramp_periods,
				   pwm->mmio_base + MX25_PWMSAR);
			countdown -= dcr;
		}
	else if (likely(pwm->state==ELLI_RAMP_UP))
		for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
			writel(pwm->period_cycles* \
				  (pwm->phase_periods[ELLI_RAMP_UP]-countdown)/pwm->ramp_periods, \
				   pwm->mmio_base + MX25_PWMSAR);
			countdown -= dcr;
		}
	else
		for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
			writel(pwm->duty_cycles, pwm->mmio_base + MX25_PWMSAR);
			countdown -= dcr;
		}
	pwm->countdown = countdown;

	writel(MX25_PWMIR_FIE ,pwm->mmio_base + MX25_PWMIR);

	//spin_unlock_irqrestore(pwm->lock,flags);
	spin_unlock(pwm->lock);
	return IRQ_HANDLED;

finished:
	//writel(pwm->cr_base,pwm->mmio_base + MX25_PWMCR);
	writel(0,pwm->mmio_base + MX25_PWMIR); /* disable all interrupts */
	writel(0,pwm->mmio_base + MX25_PWMSAR);

	/* printk(KERN_INFO "number of irqs: %ld\n",pwm->irqs); */
	if (pwm->waiting>0)
		complete_all(&pwm->completion);
	//spin_unlock_irqrestore(pwm->lock,flags);
	spin_unlock(pwm->lock);
	return IRQ_HANDLED;
}

int pwm_config_frequency(struct elliptec_pwm_data *pwm, int duty_10000, int frequency_hz)
{
	unsigned long long c;
	unsigned long period_cycles, duty_cycles, prescale;
	int period_ns;
	u32 cr,cr_en;
	u32 oc, nc;
	if (pwm == NULL || frequency_hz == 0 || duty_10000 > DUTYMAX || duty_10000 < 0)
			return -EINVAL;

	period_ns = 1000000000/frequency_hz;
	c = clk_get_rate(pwm->clk);
	c = c * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	prescale = period_cycles / 0x10000 + 1;

	period_cycles /= prescale;
	c = (unsigned long long)period_cycles * duty_10000;
	do_div(c, DUTYMAX);
	duty_cycles = c;

	c =
	cr_en = /* readl(pwm->mmio_base + MX25_PWMCR) & */ MX25_PWMCR_EN;

	cr = MX25_PWMCR_PRESCALER(prescale);
	cr |= MX25_PWMCR_CLKSRC_IPG;

	if (readl(pwm->mmio_base + MX25_PWMSAR)
			&& (readl(pwm->mmio_base + MX25_PWMCR) & MX25_PWMCR_EN)) {
		/* PWM is active and we need to wait until
		 * the current period is finished
		 */
		oc = readl(pwm->mmio_base + MX25_PWMCNR);
		for (;;) {
			nc = readl(pwm->mmio_base + MX25_PWMCNR);
			if (nc < oc)
				break;
			oc = nc;
		}
	}

	writel(cr | cr_en, pwm->mmio_base + MX25_PWMCR);
	writel(period_cycles, pwm->mmio_base + MX25_PWMPR);

	pwm->cr_base = cr;
	pwm->period = period_ns;
	pwm->period_cycles = period_cycles;
	pwm->frequency = frequency_hz;
	pwm->duty = duty_10000;
	pwm->duty_cycles = duty_cycles;
	pwm->ramp_periods = (long) pwm->ramp_us * 1000 / period_ns;

	return 0;
}

int pwm_change_duty(struct elliptec_pwm_data *pwm, int duty_10000)
{
	unsigned long long c;
	unsigned long period_cycles, duty_cycles;
	//unsigned long flags;

	if (pwm == NULL || duty_10000 > DUTYMAX || duty_10000 < 0)
				return -EINVAL;

	//spin_lock_irqsave(pwm->lock,flags);
	spin_lock(pwm->lock);

	period_cycles = readl(pwm->mmio_base + MX25_PWMPR);
	c = (unsigned long long)period_cycles * duty_10000;
	do_div(c, DUTYMAX);
	duty_cycles = c;
	pwm->duty_cycles = duty_cycles;
	pwm->duty = duty_10000;
	if (readl(pwm->mmio_base + MX25_PWMCR) & MX25_PWMCR_EN)
		writel(duty_cycles, pwm->mmio_base + MX25_PWMSAR);

	//spin_unlock_irqrestore(pwm->lock,flags);
	spin_unlock(pwm->lock);
	return 0;
}


static int elliptec_pwm_probe(struct platform_device *pdev)
{
	struct elliptec_pwm_platform_data *pdata = pdev->dev.platform_data;
	struct elliptec_pwm_data *elliptec_dat;
	struct resource *r;
	unsigned int irq;

	int ret = 0;

	if (!pdata)
		return -EBUSY;

	elliptec_dat = kzalloc(sizeof(struct elliptec_pwm_data),
				GFP_KERNEL);
	if (!elliptec_dat)
		return -ENOMEM;

	elliptec_dat->cdev.name = pdata->name;
	elliptec_dat->cdev.pwm = elliptec_dat;
	elliptec_dat->waiting = 0;
	ret = elliptec_classdev_register(&pdev->dev, &elliptec_dat->cdev);
	if (ret < 0) {
		goto err;
	}

	elliptec_dat->clk = clk_get(&pdev->dev, "pwm");

	if (IS_ERR(elliptec_dat->clk)) {
		ret = PTR_ERR(elliptec_dat->clk);
		goto err_free;
	}
	printk(KERN_INFO "enabling clock for Elliptec motors\n");
	clk_enable(elliptec_dat->clk);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!r || !irq) {
		dev_err(&pdev->dev, "memory or IRQ resource missing\n");
		ret = -ENODEV;
		goto err_free_clk;
	}

	r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free_clk;
	}

	elliptec_dat->mmio_base = ioremap(r->start, r->end - r->start + 1);
	if (elliptec_dat->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	spin_lock_init(&elliptec_dat->lock);
	elliptec_dat->irq = irq;
	platform_set_drvdata(pdev, elliptec_dat);

	ret = request_irq(irq, elliptec_fifo_irq, 0 , DRIVER_NAME, elliptec_dat);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", irq, ret);
		goto err_free_mem;
	}

	printk(KERN_INFO "%s: irq handler installed\n",__FUNCTION__);


	elliptec_dat->ramp_us = 0;
	pwm_config_frequency(elliptec_dat,5000,100000);

	return 0;
err_free:
	return ret;
err_free_mem:
	release_mem_region(r->start, r->end - r->start + 1);
err_free_clk:
	clk_put(elliptec_dat->clk);
err:
	elliptec_classdev_unregister(&elliptec_dat->cdev);

	kfree(elliptec_dat);

	return ret;
}

/*
 * platform device administration
 */

static int __devexit elliptec_pwm_remove(struct platform_device *pdev)
{
	struct resource *r;
	struct elliptec_pwm_data *elliptecs_data;

	elliptecs_data = platform_get_drvdata(pdev);

	iounmap(elliptecs_data->mmio_base);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, r->end - r->start + 1);

	clk_disable(elliptecs_data->clk);
	clk_put(elliptecs_data->clk);

	free_irq(elliptecs_data->irq, elliptecs_data);

	printk(KERN_DEBUG "%s: removing Elliptec drive %s\n",__FUNCTION__,elliptecs_data->cdev.name);

	elliptec_classdev_unregister(&elliptecs_data->cdev);

	kfree(elliptecs_data);

	return 0;
}

static struct platform_driver elliptec_pwm_driver = {
	.probe		= elliptec_pwm_probe,
	.remove		= __devexit_p(elliptec_pwm_remove),
	.driver		= {
		.name	= "mxc_pwm",
		.owner	= THIS_MODULE,
	},
};


/*
 * mulitplexing stuff
 *
 * + 1 output for motor enable/disable
 * + 5 outputs for 5-channel multiplexing
 * + 2 inputs for end switch detection
 */

#define MOTOR_ADDRBIT_0	0x01
#define MOTOR_ADDRBIT_1	0x02
#define MOTOR_ADDRBIT_2	0x04

#define ELLI_IO_MOTOR_ENABLE		(GPIO_PORTC | 16)
#define ELLI_IO_MOTOR_A0		(GPIO_PORTB | 23)
#define ELLI_IO_MOTOR_A1		(GPIO_PORTC | 19)
#define ELLI_IO_MOTOR_A2		(GPIO_PORTC | 4)

static int elli_current_motor = -1;
static int elli_current_enabled = 0;

/* motor muxing IOs must come first
 * other outputs follow
 */
static int elli_outputs[] = {
		ELLI_IO_MOTOR_A0,
		ELLI_IO_MOTOR_A1,
		ELLI_IO_MOTOR_A2,
		ELLI_IO_MOTOR_ENABLE,
};
#define ELLI_IO_MOTOR_NUMBER	8 /* number of motors */


static int elliptec_gpio_config(void)
{
	int ret,i,j;
	for (i=0; i< ARRAY_SIZE(elli_outputs);i++) {
		ret = gpio_request(elli_outputs[i], "Elliptec mux");
		if (ret) {
			printk(KERN_INFO "%s: Failed to request GPO %d for Elliptec muxing: %d\n",
				__FUNCTION__,i, ret);
			goto err_request_out;
		} else {
			ret = gpio_direction_output(elli_outputs[i], 0);
			if (ret) {
				printk(KERN_INFO "%s: Failed to set GPO %d direction for Elliptec muxing: %d\n",
					__FUNCTION__,i, ret);
			}
		}
	}

	if (gpio_export((GPIO_PORTC | 4), 0) != 0) {
		printk(KERN_INFO "%s: Failed to export GPIO3[4]\n", __FUNCTION__);
	}
	return ret;

err_request_out:
	for (j=0; j<i; j++)
		gpio_free(elli_outputs[j]);
	return ret;
}

void elliptec_gpio_free(void)
{
	int i;
	for (i=0; i< ARRAY_SIZE(elli_outputs);i++)
		gpio_free(elli_outputs[i]);
}

/* motor muxing attribute */
static ssize_t elliptec_mux_show(struct class *dev, char *buf)
{
	int ret = sprintf(buf,"%d\n",elli_current_motor);
	return ret;
}

static ssize_t elliptec_mux_store(struct class *dev,
		const char *buf, size_t size)
{
	//int i;
	ssize_t ret = -EINVAL;
	char *after;
	//long new_motor = simple_strtol(buf, &after, 10);
	u8 motor_no;

	motor_no = (u8) simple_strtol(buf, &after, 10);

	if(motor_no < ELLI_IO_MOTOR_NUMBER) {
		if(motor_no & MOTOR_ADDRBIT_0) {
			gpio_set_value(ELLI_IO_MOTOR_A0, 1);
		}
		else {
			gpio_set_value(ELLI_IO_MOTOR_A0, 0);
		}
		if(motor_no & MOTOR_ADDRBIT_1) {
			gpio_set_value(ELLI_IO_MOTOR_A1, 1);
		}
		else {
			gpio_set_value(ELLI_IO_MOTOR_A1, 0);
		}
		if(motor_no & MOTOR_ADDRBIT_2) {
			gpio_set_value(ELLI_IO_MOTOR_A2, 1);
		}
		else {
			gpio_set_value(ELLI_IO_MOTOR_A2, 0);
		}

		elli_current_motor = motor_no;

		ret = size;
	}
/*
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		if (new_motor<ELLI_IO_MOTOR_NUMBER) {
			for (i=0; i<ELLI_IO_MOTOR_NUMBER; i++) {
				gpio_set_value(elli_outputs[i],(i==new_motor)?1:0);
			}
			elli_current_motor = (new_motor>=0)?new_motor:-1;
			ret = size;
		}
	}*/

	

	return ret;
}

/* motor enable attribute */
static ssize_t elliptec_enable_show(struct class *dev, char *buf)
{
	int ret = sprintf(buf,"%d\n",elli_current_enabled);
	return ret;
}

static ssize_t elliptec_enable_store(struct class *dev,
		const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	long new_state = simple_strtol(buf, &after, 10);

	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		elli_current_enabled = (new_state!=0)?1:0;
		gpio_set_value(ELLI_IO_MOTOR_ENABLE,new_state);
		ret = size;
	}
	return ret;
}

/* end switch attribute */
static ssize_t elliptec_switches_show(struct class *dev, char *buf)
{
	//int ret = sprintf(buf,"%d %d\n",gpio_get_value(ELLI_IO_SWITCH1)?1:0,gpio_get_value(ELLI_IO_SWITCH2)?1:0);

	int ret = sprintf(buf,"There are no end switches :-P\n");
	return ret;
}

static CLASS_ATTR(mux, S_IWUSR | S_IRUGO, elliptec_mux_show, elliptec_mux_store);
static CLASS_ATTR(enable, S_IWUSR | S_IRUGO, elliptec_enable_show, elliptec_enable_store);
static CLASS_ATTR(switches, S_IRUGO, elliptec_switches_show, NULL);


static int __init elliptec_init(void)
{
	int ret;
	elliptec_class = class_create(THIS_MODULE, "elliptec");
	if (IS_ERR(elliptec_class))
		return PTR_ERR(elliptec_class);
	elliptec_class->suspend = NULL;//elliptec_suspend;
	elliptec_class->resume = NULL;//elliptec_resume;

	elliptec_gpio_config();
	if ((ret=class_create_file(elliptec_class, &class_attr_mux)) < 0) {
		printk(KERN_ERR "Failed to create mux config file");
		goto err_attr_mux;
	}

	if ((ret=class_create_file(elliptec_class, &class_attr_enable)) < 0) {
		printk(KERN_ERR "Failed to create enable file");
		goto err_attr_enable;
	}

	if ((ret=class_create_file(elliptec_class, &class_attr_switches)) < 0) {
		printk(KERN_ERR "Failed to create switches file");
		goto err_attr_switches;
	}

	platform_driver_register(&elliptec_pwm_driver);
	printk(KERN_INFO "Elliptec class registered\n");

	return 0;

err_attr_switches:
	class_remove_file(elliptec_class,&class_attr_enable);
err_attr_enable:
	class_remove_file(elliptec_class,&class_attr_mux);
err_attr_mux:
	return ret;
}

module_init(elliptec_init);

static void __exit elliptec_exit(void)
{

	platform_driver_unregister(&elliptec_pwm_driver);
	elliptec_gpio_free();
	class_destroy(elliptec_class);
}

module_exit(elliptec_exit);

MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
MODULE_DESCRIPTION("Elliptec motor driver class");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
