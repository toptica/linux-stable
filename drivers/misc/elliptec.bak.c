/*
 * elliptec.c: PWM Driver for Elliptec motors
 *
 * Author: Arno Euteneuer <arno.euteneuer@toptica.com>
 *
 * The driver installs a new device class /sys/class/elliptec where
 * up to
 *
 * Licensed under the GPL-2 or later.
 */

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
	int period;			/* in ns */
	int	duty;			/* 0..10000 */
	int duty_cycles;	/* for PWMSAR register */
	long  countdown;	/* periods to go */
	long irqs;			/* for debugging only */
};


/* i.MX27, i.MX31, i.MX35 share the same PWM function block: */

#define MX25_PWMCR                 0x00    /* PWM Control Register */
#define MX25_PWMSR                 0x04    /* PWM Status Register */
#define MX25_PWMIR                 0x08    /* PWM Interrupt Register */
#define MX25_PWMSAR                0x0C    /* PWM Sample Register */
#define MX25_PWMPR                 0x10    /* PWM Period Register */

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
#define MX25_PWMSR_FEW			  (1 << 6)
#define MX25_PWMSR_FIFOAV_MASK	  (7 << 0)


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
	unsigned long flags;
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		spin_lock_irqsave(pwm->lock,flags);
		ret=pwm_config_frequency(pwm,pwm->duty, newfreq);
		spin_unlock_irqrestore(pwm->lock,flags);
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

	return sprintf(buf, "%lu\n", (unsigned long) duty_cycles * 10000/period_cycles);
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
	unsigned long long countdown = pwm->countdown;
	if (countdown>0) {
		countdown = countdown * pwm->period;
		do_div(countdown,1000);
		return sprintf(buf, "%llu\n", countdown);
	}
	return sprintf(buf, "%llu\n", countdown);
}

static ssize_t elliptec_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	char *after;
	long newduration = simple_strtol(buf, &after, 10);
	unsigned long long countdown;
	unsigned long flags;
	u32 cr,sr,i,dcr,wm,repeat;

	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count != size)
		return -EINVAL;
	spin_lock_irqsave(pwm->lock,flags);

	pwm->irqs = 0;
	if (newduration == 0) {
		writel(pwm->cr_base,pwm->mmio_base + MX25_PWMCR);
		pwm->countdown =0;
		complete_all(&pwm->completion);
		spin_unlock_irqrestore(pwm->lock,flags);
		return count;
	}

	if (newduration <0) {
		pwm_config_frequency(pwm,pwm->duty,pwm->frequency);
		cr = readl(pwm->mmio_base + MX25_PWMCR) | MX25_PWMCR_EN;
		writel(cr,pwm->mmio_base + MX25_PWMCR);
		pwm->countdown =-1;
		complete_all(&pwm->completion);
		spin_unlock_irqrestore(pwm->lock,flags);
		return count;
	}
	if (pwm->countdown>0)
		complete_all(&pwm->completion);
	init_completion(&pwm->completion);

	writel(pwm->cr_base, pwm->mmio_base + MX25_PWMCR);
	pwm_config_frequency(pwm,pwm->duty,pwm->frequency);

	countdown = (unsigned long long) newduration * 1000;
	do_div(countdown,pwm->period);

	dcr=1;
	repeat=0;
	wm=1;
	if (countdown>8) {
		dcr=8;
		repeat = 3;
		wm=3;
	} else if (countdown>4) {
		dcr=4;
		repeat = 2;
		wm=3;
	} else if (countdown>2) {
		dcr=2;
		repeat = 1;
		wm=2;
	}
	writel(0xf, pwm->mmio_base + MX25_PWMSR);

	cr = pwm->cr_base | MX25_PWMCR_WATERMARK(wm) | MX25_PWMCR_REPEAT(repeat);
	writel(cr,pwm->mmio_base + MX25_PWMCR);

	sr = readl(pwm->mmio_base + MX25_PWMSR) & MX25_PWMSR_FIFOAV_MASK;
	for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
		writel(pwm->duty_cycles, pwm->mmio_base + MX25_PWMSAR);
		countdown -= dcr;
	}

	pwm->countdown = countdown;

	if (countdown == 0)
		writel(MX25_PWMIR_RIE ,pwm->mmio_base + MX25_PWMIR);
	else
		writel(MX25_PWMIR_FIE ,pwm->mmio_base + MX25_PWMIR);


	cr |= MX25_PWMCR_EN;
	writel(cr,pwm->mmio_base + MX25_PWMCR);

	spin_unlock_irqrestore(pwm->lock,flags);
	return count;
}

static ssize_t elliptec_arrived_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elliptec_classdev *elliptec_cdev = dev_get_drvdata(dev);
	struct elliptec_pwm_data *pwm = elliptec_cdev->pwm;
	if (pwm->countdown<0)
		return sprintf(buf, "0\n");
	if (pwm->countdown>0)
		wait_for_completion(&pwm->completion);
	return sprintf(buf, "1\n");
}

static DEVICE_ATTR(frequency, 0644, elliptec_frequency_show, elliptec_frequency_store);
static DEVICE_ATTR(duty, 0644, elliptec_duty_show, elliptec_duty_store);
static DEVICE_ATTR(duration, 0644, elliptec_duration_show, elliptec_duration_store);
static DEVICE_ATTR(arrived, 0644, elliptec_arrived_show, NULL);

/**
 * elliptec_classdev_register - register a new object of elliptec_classdev class.
 * @parent: The device to register.
 * @elliptec_cdev: the elliptec_classdev structure for this device.
 */
int elliptec_classdev_register(struct device *parent, struct elliptec_classdev *elliptec_cdev)
{
	int rc;

	elliptec_cdev->dev = device_create(elliptec_class, parent, 0, elliptec_cdev,
				      "%s", elliptec_cdev->name);
	if (IS_ERR(elliptec_cdev->dev))
		return PTR_ERR(elliptec_cdev->dev);


	/* register the attributes */
	rc = device_create_file(elliptec_cdev->dev, &dev_attr_frequency);
	if (rc)
		goto err_out;

	/* add to the list of elliptecs */
	down_write(&elliptecs_list_lock);
	list_add_tail(&elliptec_cdev->node, &elliptecs_list);
	up_write(&elliptecs_list_lock);

	rc = device_create_file(elliptec_cdev->dev, &dev_attr_duty);
	rc = device_create_file(elliptec_cdev->dev, &dev_attr_duration);
	rc = device_create_file(elliptec_cdev->dev, &dev_attr_arrived);
	if (rc)
		goto err_out_attr_max;


	printk(KERN_INFO "Registered elliptec device: %s\n",
			elliptec_cdev->name);

	return 0;
err_out_attr_max:
	device_remove_file(elliptec_cdev->dev, &dev_attr_frequency);
	list_del(&elliptec_cdev->node);
err_out:
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
	device_remove_file(elliptec_cdev->dev, &dev_attr_duty);
	device_remove_file(elliptec_cdev->dev, &dev_attr_frequency);
	device_remove_file(elliptec_cdev->dev, &dev_attr_duration);
	device_remove_file(elliptec_cdev->dev, &dev_attr_arrived);

	device_unregister(elliptec_cdev->dev);

	down_write(&elliptecs_list_lock);
	list_del(&elliptec_cdev->node);
	up_write(&elliptecs_list_lock);
}

static int elliptec_suspend(struct device *dev, pm_message_t state)
{
	return 0;
}

static int elliptec_resume(struct device *dev)
{
	return 0;
}


/* -------------------------------------------------------------------------
 *
 * real pwm stuff
 *
 * ------------------------------------------------------------------------- */

irqreturn_t elliptec_fifo_irq(int irq, void *dev_id)
{
	struct elliptec_pwm_data *pwm = (struct elliptec_pwm_data *) dev_id;
	u32 sr,cr,i,dcr,repeat,wm;
	long countdown;
	unsigned long flags;


	if (!pwm)
		return IRQ_HANDLED;

	spin_lock_irqsave(pwm->lock,flags);
	countdown = pwm->countdown;


	if (countdown==0) {
		sr = readl(pwm->mmio_base + MX25_PWMSR);
		if (sr & MX25_PWMSR_FIFOAV_MASK) {
			writel(0xf,pwm->mmio_base + MX25_PWMSR);
			writel(MX25_PWMIR_RIE ,pwm->mmio_base + MX25_PWMIR);
			spin_unlock_irqrestore(pwm->lock,flags);
			return IRQ_HANDLED;
		}
		goto finished;
	}

	if (countdown<0) {
		goto finished;
	}

	pwm->irqs++;
	/*
	 * trying to minimize use of IRQs by making use of REPEAT
	 * and WATERMARK feature
	 */
	dcr=1;			/* number periods per FIFO enqueue */
	repeat=0;		/* REPEAT flag for PWM */
	wm=1;			/* WATERMARK level */
	if (countdown>8) {
		dcr=8;
		repeat = 3;
		wm=3;
	} else if (countdown>4) {
		dcr=4;
		repeat = 2;
		wm=3;
	} else if (countdown>2) {
		dcr=2;
		repeat = 1;
		wm=2;
	}
	cr = pwm->cr_base | MX25_PWMCR_WATERMARK(wm) | MX25_PWMCR_REPEAT(repeat) | MX25_PWMCR_EN;
	writel(cr,pwm->mmio_base + MX25_PWMCR);

	sr = readl(pwm->mmio_base + MX25_PWMSR) & MX25_PWMSR_FIFOAV_MASK;
	for (i=sr ; (i < 4) && (countdown>=dcr); i++) {
		writel(pwm->duty_cycles, pwm->mmio_base + MX25_PWMSAR);
		countdown -= dcr;
	}
	pwm->countdown = countdown;

	writel(0xf,pwm->mmio_base + MX25_PWMSR);
	/*
	 * for last FIFO filling use rollover IRQ to be more precise
	 */
	if (countdown == 0)
		writel(MX25_PWMIR_RIE,pwm->mmio_base + MX25_PWMIR);
	else
		writel(MX25_PWMIR_FIE ,pwm->mmio_base + MX25_PWMIR);

	spin_unlock_irqrestore(pwm->lock,flags);
	return IRQ_HANDLED;

finished:
	writel(pwm->cr_base,pwm->mmio_base + MX25_PWMCR);
	writel(0,pwm->mmio_base + MX25_PWMIR); /* disable all interrupts */

	printk(KERN_INFO "number of irqs: %ld\n",pwm->irqs);
	complete_all(&pwm->completion);
	spin_unlock_irqrestore(pwm->lock,flags);
	return IRQ_HANDLED;
}

int pwm_config_frequency(struct elliptec_pwm_data *pwm, int duty_10000, int frequency_hz)
{
	unsigned long long c;
	unsigned long period_cycles, duty_cycles, prescale;
	int period_ns;
	u32 cr,cr_en;
	if (pwm == NULL || frequency_hz == 0 || duty_10000 > 10000 || duty_10000 < 0)
			return -EINVAL;

	period_ns = 1000000000/frequency_hz;
	c = clk_get_rate(pwm->clk);
	c = c * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	prescale = period_cycles / 0x10000 + 1;

	period_cycles /= prescale;
	c = (unsigned long long)period_cycles * duty_10000;
	do_div(c, 10000);
	duty_cycles = c;

	cr_en = readl(pwm->mmio_base + MX25_PWMCR) & MX25_PWMCR_EN;

	cr = MX25_PWMCR_PRESCALER(prescale);
	cr |= MX25_PWMCR_CLKSRC_IPG;

	writel(cr | cr_en, pwm->mmio_base + MX25_PWMCR);
	writel(period_cycles, pwm->mmio_base + MX25_PWMPR);

	pwm->cr_base = cr;
	pwm->period = period_ns;
	pwm->frequency = frequency_hz;
	pwm->duty = duty_10000;
	pwm->duty_cycles = duty_cycles;

	return 0;
}

int pwm_change_duty(struct elliptec_pwm_data *pwm, int duty_10000)
{
	unsigned long long c;
	unsigned long period_cycles, duty_cycles;
	unsigned long flags;

	if (pwm == NULL || duty_10000 > 10000 || duty_10000 < 0)
				return -EINVAL;

	spin_lock_irqsave(pwm->lock,flags);

	period_cycles = readl(pwm->mmio_base + MX25_PWMPR);
	c = (unsigned long long)period_cycles * duty_10000;
	do_div(c, 10000);
	duty_cycles = c;
	pwm->duty_cycles = duty_cycles;
	pwm->duty = duty_10000;
	writel(duty_cycles, pwm->mmio_base + MX25_PWMSAR);

	spin_unlock_irqrestore(pwm->lock,flags);
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


static int __init elliptec_init(void)
{
	elliptec_class = class_create(THIS_MODULE, "elliptec");
	if (IS_ERR(elliptec_class))
		return PTR_ERR(elliptec_class);
	elliptec_class->suspend = elliptec_suspend;
	elliptec_class->resume = elliptec_resume;

	platform_driver_register(&elliptec_pwm_driver);
	printk(KERN_INFO "Elliptec class registered\n");

	return 0;
}

//subsys_initcall(elliptec_init);
module_init(elliptec_init);
//arch_initcall(elliptec_init);

static void __exit elliptec_exit(void)
{
	platform_driver_unregister(&elliptec_pwm_driver);

	class_destroy(elliptec_class);
}

module_exit(elliptec_exit);

MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
MODULE_DESCRIPTION("Elliptec motor driver class");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
