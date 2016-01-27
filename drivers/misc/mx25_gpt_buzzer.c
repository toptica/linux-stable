/*
 * mx25_gpt_buzzer.c
 *
 *
 * Maintainer: Arno Euteneuer
 * 
 * This implements a driver for using one of the iMX25 general purpose timers
 * for driving a simple buzzer.
 * The device file will be /sys/class/buzzer/buzzer:X/melody.
 *
 * Writing a character to the device will produce a pulse train of a certain
 * frequency, duration and 50% duty cycle. Writing a string of more than one character
 * will produce kind of a melody with one tone for each character.
 * The frequency is given by the ASCII code of the character.
 * Capital "A" (ASCII 65) produces 440 Hz. For higher codes the frequency increases
 * by a factor of 2^(1/12) with each increment in ASCII number. After 12 steps
 * the frequency is doubled meaning a step of one octave. In the same way the
 * frequency decreases with decreasing ASCII code.
 * ASCII codes of 32 (space) and below do not produce a tone but produce a rest
 * of the same duration as the tones.
 * 
 * Reading the device returns the last melody string. 
 *
 * Copyright (C) 2007 TOPTICA Photonics AG.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/sysdev.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <mach/hardware.h>
#include <linux/mutex.h>
#include <mach/iomux.h>
#include <linux/sched.h>


#define BUF_LEN 255

#define MX25_GPTCR		0x0000
#define MX25_GPTPR		0x0004
#define MX25_GPTSR		0x0008
#define MX25_GPTIR		0x000c
#define MX25_GPTOCR1	0x0010
#define MX25_GPTOCR2	0x0014
#define MX25_GPTOCR3	0x0018
#define MX25_GPTICR1	0x001c
#define MX25_GPTICR2	0x0020
#define MX25_GPTCNT		0x0024

#define MX25_GPTCR_EN			(1 << 0)
#define MX25_GPTCR_ENMOD		(1 << 1)
#define MX25_GPTCR_WAITEN		(1 << 3)
#define MX25_GPTCR_CLKSRC_SHIFT	6
#define MX25_GPTCR_CLKSRC_IPG	(1 << 6)
#define MX25_GPTCR_CLKSRC_IPG_HF	(2 << 6)
#define MX25_GPTCR_CLKSRC_CLKIN	(3 << 6)
#define MX25_GPTCR_CLKSRC_32K	(4 << 6)
#define MX25_GPTCR_CLKSRC_MASK	(0x7 << 6)
#define MX25_GPTCR_FRR			(1 << 9)
#define MX25_GPTCR_SWR			(1 << 15)
#define MX25_GPTCR_OM1_SHIFT	20
#define MX25_GPTCR_OM2_SHIFT	23
#define MX25_GPTCR_OM3_SHIFT	26
#define MX25_GPTCR_OM_MASK		7
#define MX25_GPTCR_OM_OFF		0
#define MX25_GPTCR_OM_TOGGLE	1
#define MX25_GPTCR_OM_CLEAR		2
#define MX25_GPTCR_OM_SET		3
#define MX25_GPTCR_OM_PULSE		4
#define MX25_GPTCR_FO1			(1 << 29)
#define MX25_GPTCR_FO2			(1 << 30)
#define MX25_GPTCR_FO3			(1 << 31)

#define MX25_GPTSR_ROV			(1 << 5)
#define MX25_GPTSR_OF3			(1 << 2)
#define MX25_GPTSR_OF2			(1 << 1)
#define MX25_GPTSR_OF1			(1 << 0)


static struct class *buzzer_class;

DECLARE_RWSEM(buzzers_list_lock);
LIST_HEAD(buzzers_list);


struct buzzer_classdev {
	char		name[32];

	struct device		*dev;
	struct mx25_gpt_buzzer_data *buzzer;
	struct list_head	 node;
};

struct mx25_gpt_buzzer_data {
	struct buzzer_classdev cdev;
	void __iomem *mmio_base;
	struct clk	*clk;
	spinlock_t	lock;	/* for protecting data below */
	u32 cr_base;		/* PWMCR config for prescaler and clk */
	int	base_frequency;		/* in Hz */

	struct timer_list tl;
	char Message[BUF_LEN+1];						/* the melody */
	char *playnow;								/* "cursor in melody */
	int toplay;								/* remaining notes */
	/*
	 * How far did the process reading the message get?
	 * Useful if the message is larger than the size of the
	 * buffer we get to fill in device_read.
	 */
	char *Message_Ptr;
};


#define BUZZER_BASE_FREQ		6600000
#define BUZZER_DURATION			50


/* 
 * look-up table with powers of 2^(1/12)
 */
static unsigned int tune[12] = {
	1000,1059,1122,1189,1260,1335,1414,1498,1587,1682,1782,1888
};

void set_buzzer(int freq, struct mx25_gpt_buzzer_data *buzzer_dat )
{
	if ((freq>0) && (freq < 20000)) {
		unsigned long count = (buzzer_dat->base_frequency / freq) / 2;
		writel(count, buzzer_dat->mmio_base + MX25_GPTOCR1);
		writel(buzzer_dat->cr_base | (MX25_GPTCR_OM_TOGGLE << MX25_GPTCR_OM1_SHIFT), buzzer_dat->mmio_base + MX25_GPTCR);
	}
	else {
		writel(buzzer_dat->cr_base | (MX25_GPTCR_OM_CLEAR << MX25_GPTCR_OM1_SHIFT), buzzer_dat->mmio_base + MX25_GPTCR);
	}
}

void set_tune(char note,struct mx25_gpt_buzzer_data *buzzer_dat )
{
	char divider,factor;
	char i;
	int f=0;
	if (note > 32) {
		divider = 1;
		for (i=65; i>note; i-=12, divider *=2) ;
		factor = 1;
		for (; (i+12)<=note; i+=12, factor *=2) ;
		f = 440 * tune[note-i] * factor / divider /1000;
		/* printk(KERN_INFO "note %d, i %d, d %d, freq %d\n",note,i,note-i,f); */
	}
	set_buzzer(f,buzzer_dat);
}

void stop_buzzer(unsigned long dummy)
{
	struct mx25_gpt_buzzer_data *buzzer_dat = (struct mx25_gpt_buzzer_data *) dummy;

	int j;
	spin_lock(&buzzer_dat->lock);
	if (buzzer_dat->toplay>0) {
		set_tune(*buzzer_dat->playnow,buzzer_dat);
		buzzer_dat->playnow++;
		buzzer_dat->toplay--;
		j=jiffies;
		del_timer(&buzzer_dat->tl);
		buzzer_dat->tl.expires=j+msecs_to_jiffies(BUZZER_DURATION);
		add_timer(&buzzer_dat->tl);
		/* printk(KERN_INFO "scheduled next note %c\n",*playnow); */
	}
	else {
		del_timer(&buzzer_dat->tl);
		set_buzzer(0,buzzer_dat);
		buzzer_dat->toplay = -1; /* negative to indicate idle state */
		/* printk(KERN_INFO "tried stopping buzzer ...\n"); */
	}
	spin_unlock(&buzzer_dat->lock);
}


static ssize_t buzzer_melody_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct buzzer_classdev *buzzer_cdev = dev_get_drvdata(dev);
	struct mx25_gpt_buzzer_data *buzzer_dat = buzzer_cdev->buzzer;
	ssize_t ret;
	spin_lock(&buzzer_dat->lock);

	ret = sprintf(buf,"%s",buzzer_dat->Message);
	spin_unlock(&buzzer_dat->lock);
	return ret;
}

static ssize_t buzzer_melody_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct buzzer_classdev *buzzer_cdev = dev_get_drvdata(dev);
	struct mx25_gpt_buzzer_data *buzzer_dat = buzzer_cdev->buzzer;
	int i,j,notok;

#ifdef DEBUG
	printk(KERN_INFO "%s: %s,%d\n", __FUNCTION__, buf, size);
#endif
	i=0;
	spin_lock(&buzzer_dat->lock);
    for (notok = 1;notok;) {
		if (buzzer_dat->toplay<0) {
			/*
			 * if no melody is currently being played start first note and
			 * schedule the next one
			 */
			for (i = 0; (i < size) && (i < BUF_LEN); i++)
				buzzer_dat->Message[i] = *(buf + i);
			buzzer_dat->Message[i] = 0;	/* terminate with zero */

			buzzer_dat->Message_Ptr = buzzer_dat->Message;

			buzzer_dat->toplay = i-1;
			buzzer_dat->playnow = buzzer_dat->Message+1;

			/* play first note */
			set_tune(buzzer_dat->Message[0],buzzer_dat);
			j=jiffies;
			buzzer_dat->tl.expires=j + msecs_to_jiffies(BUZZER_DURATION);
			/* schedule next note */
			add_timer(&buzzer_dat->tl);
			notok = 0;
			spin_unlock(&buzzer_dat->lock);
		} else {
			spin_unlock(&buzzer_dat->lock);
			schedule();
			spin_lock(&buzzer_dat->lock);
		}


    }

	return i;
}


static struct device_attribute dev_attributes[]= {
		{
			.attr = {.name = "melody", .mode = 0644 },
			.show	= buzzer_melody_show,
			.store	= buzzer_melody_store,
		},
};

#define ATTR_NUMBER 	ARRAY_SIZE(dev_attributes)


/**
 * buzzer_classdev_register - register a new object of buzzer_classdev class.
 * @parent: The device to register.
 * @buzzer_cdev: the buzzer_classdev structure for this device.
 */
int buzzer_classdev_register(struct device *parent, struct buzzer_classdev *buzzer_cdev)
{
	int rc,i;

	buzzer_cdev->dev = device_create(buzzer_class, parent, 0, buzzer_cdev,
				      "%s", buzzer_cdev->name);
	if (IS_ERR(buzzer_cdev->dev))
		return PTR_ERR(buzzer_cdev->dev);


	/* register the attributes */
	for(i=0;i < ATTR_NUMBER;i++) {
		rc = device_create_file(buzzer_cdev->dev, &dev_attributes[i]);
		if (rc)
			goto err_out;
	}

	/* add to the list of buzzers */
	down_write(&buzzers_list_lock);
	list_add_tail(&buzzer_cdev->node, &buzzers_list);
	up_write(&buzzers_list_lock);

	printk(KERN_INFO "Registered buzzer device: %s\n",
			buzzer_cdev->name);

	return 0;
err_out:
	for(i--; i>=0;i--)
		device_remove_file(buzzer_cdev->dev, &dev_attributes[i]);
	device_unregister(buzzer_cdev->dev);
	return rc;
}

/**
 * buzzer_classdev_unregister - unregisters a object of buzzer_properties class.
 * @buzzer_cdev: the buzzer device to unregister
 *
 * Unregisters a previously registered via buzzer_classdev_register object.
 */
void buzzer_classdev_unregister(struct buzzer_classdev *buzzer_cdev)
{

	int i;
	printk(KERN_DEBUG "%s: removing attributes of %s\n",__FUNCTION__,buzzer_cdev->name);
	for(i=0;i < ATTR_NUMBER;i++)
		device_remove_file(buzzer_cdev->dev, &dev_attributes[i]);

	down_write(&buzzers_list_lock);
	list_del(&buzzer_cdev->node);
	up_write(&buzzers_list_lock);

	device_unregister(buzzer_cdev->dev);
}


/*
 * platform device administration
 */

static int mx25_gpt_buzzer_probe(struct platform_device *pdev)
{
	struct mx25_gpt_buzzer_data *buzzer_dat;
	struct resource *r;
	unsigned long prescale;
	unsigned long cr;
	int j;
	int ret = 0;

	buzzer_dat = kzalloc(sizeof(struct mx25_gpt_buzzer_data),
				GFP_KERNEL);
	if (!buzzer_dat)
		return -ENOMEM;

	sprintf(buzzer_dat->cdev.name,"buzzer:%d",pdev->id);
	buzzer_dat->cdev.buzzer = buzzer_dat;
	ret = buzzer_classdev_register(&pdev->dev, &buzzer_dat->cdev);
	if (ret < 0) {
		goto err;
	}
	buzzer_dat->toplay = -1;
	buzzer_dat->clk = clk_get(&pdev->dev, "gpt");
	buzzer_dat->Message_Ptr = buzzer_dat->Message;
	buzzer_dat->Message[0]='\0';

	if (IS_ERR(buzzer_dat->clk)) {
		ret = PTR_ERR(buzzer_dat->clk);
		goto err_free;
	}
	printk(KERN_INFO "enabling clock for %s\n",buzzer_dat->cdev.name);
	clk_enable(buzzer_dat->clk);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free_clk;
	}

	buzzer_dat->mmio_base = ioremap(r->start, r->end - r->start + 1);
	if (buzzer_dat->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	spin_lock_init(&buzzer_dat->lock);

	/*
	 * clock preparation
	 * adjust prescaler to get 6.6MHz clock
	 *
	 */

	writel(0, buzzer_dat->mmio_base + MX25_GPTIR);

	/* disable all input captures
	 * disable all output compares but OM1
	 * use restart mode FRR = 0
	 */
	cr = MX25_GPTCR_EN /* we enable GPT and clear output */
			| MX25_GPTCR_CLKSRC_IPG
			| MX25_GPTCR_ENMOD ;
	buzzer_dat->cr_base = cr;
	writel(cr | (MX25_GPTCR_OM_CLEAR << MX25_GPTCR_OM1_SHIFT), buzzer_dat->mmio_base + MX25_GPTCR);

	prescale = clk_get_rate(buzzer_dat->clk) / BUZZER_BASE_FREQ -1;
	buzzer_dat->base_frequency = clk_get_rate(buzzer_dat->clk) / (prescale + 1);
	writel(prescale, buzzer_dat->mmio_base + MX25_GPTPR);

	writel(0xffffffff, buzzer_dat->mmio_base + MX25_GPTSR);

	/*
	 * timer initialization
	 */
	setup_timer(&buzzer_dat->tl, stop_buzzer, (unsigned long)buzzer_dat);

	set_tune(' ',buzzer_dat);
	j=jiffies;
	buzzer_dat->tl.expires=j + msecs_to_jiffies(BUZZER_DURATION);
	add_timer(&buzzer_dat->tl);

	platform_set_drvdata(pdev, buzzer_dat);


	return 0;
err_free:
	return ret;
err_free_mem:
	release_mem_region(r->start, r->end - r->start + 1);
err_free_clk:
	clk_put(buzzer_dat->clk);
err:
	buzzer_classdev_unregister(&buzzer_dat->cdev);

	kfree(buzzer_dat);

	return ret;
}


static int __devexit mx25_gpt_buzzer_remove(struct platform_device *pdev)
{
	struct resource *r;
	struct mx25_gpt_buzzer_data *buzzers_data;

	buzzers_data = platform_get_drvdata(pdev);

	writel(buzzers_data->cr_base & ~MX25_GPTCR_EN, buzzers_data->mmio_base + MX25_GPTCR);

	iounmap(buzzers_data->mmio_base);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, r->end - r->start + 1);

	clk_disable(buzzers_data->clk);
	clk_put(buzzers_data->clk);


	printk(KERN_DEBUG "%s: removing buzzer %s\n",__FUNCTION__,buzzers_data->cdev.name);

	buzzer_classdev_unregister(&buzzers_data->cdev);

	kfree(buzzers_data);

	return 0;
}

static struct platform_driver mx25_gpt_buzzer_driver = {
	.probe		= mx25_gpt_buzzer_probe,
	.remove		= __devexit_p(mx25_gpt_buzzer_remove),
	.driver		= {
		.name	= "mxc_gpt",
		.owner	= THIS_MODULE,
	},
};


/*
 * buzzer class stuff
 */
static int __init mx25_gpt_buzzer_init(void)
{
	buzzer_class = class_create(THIS_MODULE, "buzzer");
	if (IS_ERR(buzzer_class))
		return PTR_ERR(buzzer_class);
	buzzer_class->suspend = NULL;//buzzer_suspend;
	buzzer_class->resume = NULL;//buzzer_resume;

	platform_driver_register(&mx25_gpt_buzzer_driver);
	printk(KERN_INFO "buzzer class registered\n");

	return 0;

}

module_init(mx25_gpt_buzzer_init);

static void __exit mx25_gpt_buzzer_exit(void)
{

	platform_driver_unregister(&mx25_gpt_buzzer_driver);
	class_destroy(buzzer_class);
}

module_exit(mx25_gpt_buzzer_exit);

MODULE_AUTHOR("Arno Euteneuer <arno.euteneuer@toptica.com>");
MODULE_DESCRIPTION("iMX25 GPT buzzer");
MODULE_LICENSE("GPL");
