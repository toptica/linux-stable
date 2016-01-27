/*
 * drivers/video/backlight/ili9322.c
 *
 * ILI9322 LCD controller driver core.
 *
 * (C) Copyright Lothar Wassmann <LW@KARO-electronics.de>
 *
 *   based on: drivers/video/backlight/ili9322.c
 *   Copyright 2007 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/lcd.h>
#include <linux/module.h>

#include <linux/spi/spi.h>

#include <video/ili9322.h>

#include "ili9322.h"


static inline int ili9322_write_spi(struct ili9322 *ili,
				    unsigned char reg,
				    unsigned char value)
{
	int ret;
	struct ili9322_spi *spi = &ili->spi;

	/* spi message consits of:
	 * first byte: ID and operation
	 */
	mutex_lock(&ili->mutex);

	spi->buffer.address = reg;
	spi->buffer.value = value;

	ret = spi_sync(spi->dev, &spi->message);
	mutex_unlock(&ili->mutex);
	return ret;
}

int ili9322_write(struct ili9322 *ili, unsigned char reg, unsigned char value)
{
	return ili->write(ili, reg, value);
}
EXPORT_SYMBOL_GPL(ili9322_write);

int ili9322_write_regs(struct ili9322 *ili,
		       struct ili9322_reg *values,
		       int nr_values)
{
	int index;
	int ret;

	for (index = 0; index < nr_values; index++, values++) {
		ret = ili9322_write(ili, values->address, values->value);
		if (ret != 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ili9322_write_regs);

static void ili9322_reset(struct ili9322 *ili)
{
	struct ili9322_platdata *pdata = ili->platdata;

	/* must wait at least 30ms after Power on */
	mdelay(30);

	pdata->reset(1);
	udelay(100);

	pdata->reset(0);
	mdelay(10);
}

static inline int ili9322_init_chip(struct ili9322 *ili)
{
	int ret;
	struct ili9322_platdata *pdata = ili->platdata;

	if (pdata->power) {
		pdata->power(1);
		msleep(20);
	}

	ili9322_reset(ili);

	ret = ili->client->init(ili, ili->platdata);
	if (ret != 0) {
		dev_err(ili->dev, "failed to initialise display\n");
		return ret;
	}

	ili->initialised = 1;
	return 0;
}

static inline int ili9322_power_on(struct ili9322 *ili)
{
	if (!ili->initialised)
		ili9322_init_chip(ili);

	/* turn on charge pump */
	ili9322_write(ili, ILI9322_REG_POWERCTL, 0xef);

	return 0;
}

static inline int ili9322_standby(struct ili9322 *ili)
{
	/* turn off charge pump */
	ili9322_write(ili, ILI9322_REG_POWERCTL, 0xee);

	/* Wait at least 5 frames */
	msleep(100);
	return 0;
}

static inline int ili9322_power_off(struct ili9322 *ili)
{
	struct ili9322_platdata *pdata = ili->platdata;

	/* initiate auto power down sequence */
	ili9322_write(ili, ILI9322_REG_POWERCTL, 0x80);

	if (pdata->power) {
		pdata->power(0);
	}
	return 0;
}

#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_UNBLANK)

static int ili9322_power(struct ili9322 *ili, int power)
{
	int ret = 0;

	if (!POWER_IS_ON(ili->power) && POWER_IS_ON(power)) {
		ret = ili9322_power_on(ili);
	} else if (POWER_IS_ON(ili->power) && !POWER_IS_ON(power)) {
		ret = ili9322_standby(ili);
	}
	if (ret == 0)
		ili->power = power;
	else
		dev_warn(ili->dev, "failed to set power mode %d\n", power);

	return ret;
}

static inline struct ili9322 *to_our_lcd(struct lcd_device *lcd)
{
	return lcd_get_data(lcd);
}

static int ili9322_set_power(struct lcd_device *lcd, int power)
{
	struct ili9322 *ili = to_our_lcd(lcd);

	return ili9322_power(ili, power);
}

static int ili9322_get_power(struct lcd_device *lcd)
{
	struct ili9322 *ili = to_our_lcd(lcd);

	return ili->power;
}

static struct lcd_ops ili9322_ops = {
	.get_power	= ili9322_get_power,
	.set_power	= ili9322_set_power,
};

static void __devinit ili9322_setup_spi(struct ili9322 *ili,
					struct spi_device *dev)
{
	struct ili9322_spi *spi = &ili->spi;

	ili->write = ili9322_write_spi;
	spi->dev = dev;

	spi->xfer.tx_buf = &spi->buffer;
	spi->xfer.len = 2;
	spi->xfer.bits_per_word = 8;
	dev->mode = SPI_CPOL | SPI_CPHA;

	spi_message_init(&spi->message);
	spi_message_add_tail(&spi->xfer, &spi->message);
}

static ssize_t ili9322_reg_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = -EINVAL;
	struct ili9322 *ili = dev_get_drvdata(dev);
	const char *start = buf;

	while (1) {
		struct ili9322_reg data;
		char *p;
		unsigned long val;

		/* Parse a string formatted like:
		 * "4,0;0x04,1" into address/value pairs of numbers
		 */
		val = simple_strtoul(buf, &p, 0);
		if (p == buf)
			return -EINVAL;
		if (*p != '\0' && *p != '\n' && *p != ',')
			return -EINVAL;
		if (val > 128)
			return -EINVAL;
		data.address = val;

		buf = p + 1;
		val = simple_strtoul(buf, &p, 0);
		if (p == buf)
			return -EINVAL;
		if (*p != '\0' && *p != '\n' && *p != ';')
			return -EINVAL;
		if (val > 255)
			return -EINVAL;
		data.value = val;

		ret = ili9322_write(ili, data.address, data.value);
		if (ret < 0)
			return ret;

		if (*p != ';')
			break;

		buf = p + 1;
		BUG_ON(buf > start + size);
	}
	return size;
}

static DEVICE_ATTR(reg, S_IWUSR, NULL, ili9322_reg_write);

static int ili9322_fb_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct ili9322 *ili;
	int power;

	ili = container_of(self, struct ili9322, notifier);

	if (event != FB_EVENT_PREBLANK)
		return 0;

	power = *(int *)evdata->data;
	return ili9322_power(ili, power);
}

int __devinit ili9322_probe_spi(struct spi_device *spi,
				struct ili9322_client *client)
{
	struct ili9322_platdata *pdata = spi->dev.platform_data;
	struct device *dev = &spi->dev;
	struct ili9322 *ili;
	struct lcd_device *lcd;
	int ret = 0;

	/* verify we where given some information */

	if (pdata == NULL) {
		dev_err(dev, "no platform data supplied\n");
		return -EINVAL;
	}

	if (pdata->reset == NULL) {
		dev_err(dev, "invalid platform data supplied\n");
		return -EINVAL;
	}

	/* allocate and initialise our state */
	ili = kzalloc(sizeof(struct ili9322), GFP_KERNEL);
	if (ili == NULL) {
		dev_err(dev, "no memory for device\n");
		return -ENOMEM;
	}

	mutex_init(&ili->mutex);

	ili->dev = dev;
	ili->client = client;
	ili->power = FB_BLANK_POWERDOWN;
	ili->platdata = pdata;

	dev_set_drvdata(&spi->dev, ili);

	ili9322_setup_spi(ili, spi);

	lcd = lcd_device_register("ili9322", dev, ili, &ili9322_ops);
	if (IS_ERR(lcd)) {
		dev_err(dev, "failed to register lcd device\n");
		ret = PTR_ERR(lcd);
		goto err_free;
	}

	ili->notifier.notifier_call = ili9322_fb_callback;
	ret = fb_register_client(&ili->notifier);
	if (ret)
		goto err_unregister;

	ili->lcd = lcd;

	ret = sysfs_create_file(&dev->kobj, &dev_attr_reg.attr);
	if (ret) {
		dev_err(dev, "Unable to register sysdev entry\n");
		goto err_notifier;
	}
	dev_info(dev, "initialising %s\n", client->name);

	ret = ili9322_power(ili, FB_BLANK_UNBLANK);
	if (ret != 0) {
		dev_err(dev, "failed to set lcd power state\n");
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	sysfs_remove_file(&dev->kobj, &dev_attr_reg.attr);

err_notifier:
	fb_unregister_client(&ili->notifier);

err_unregister:
	lcd_device_unregister(lcd);

err_free:
	kfree(ili);

	return ret;
}
EXPORT_SYMBOL_GPL(ili9322_probe_spi);

int __devexit ili9322_remove(struct ili9322 *ili)
{
	ili9322_power(ili, FB_BLANK_POWERDOWN);

	sysfs_remove_file(&ili->dev->kobj, &dev_attr_reg.attr);

	fb_unregister_client(&ili->notifier);
	lcd_device_unregister(ili->lcd);
	kfree(ili);

	return 0;
}
EXPORT_SYMBOL_GPL(ili9322_remove);

#ifdef CONFIG_PM
int ili9322_suspend(struct ili9322 *ili, pm_message_t state)
{
	int ret;
	struct ili9322_platdata *pdata = ili->platdata;

	if (state.event == PM_EVENT_SUSPEND) {
		if (!POWER_IS_ON(ili->power))
			return 0;

		ret = ili9322_standby(ili);

		if (pdata->suspend == ILI9322_SUSPEND_DEEP) {
			ret = ili9322_power_off(ili);
			ili->initialised = 0;
		}
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ili9322_suspend);

int ili9322_resume(struct ili9322 *ili)
{
	dev_info(ili->dev, "resuming from power state %d\n", ili->power);

	if (!POWER_IS_ON(ili->power))
		return 0;

	return ili9322_power_on(ili);
}
EXPORT_SYMBOL_GPL(ili9322_resume);
#endif

/* Power down all displays on reboot, poweroff or halt */
void ili9322_shutdown(struct ili9322 *ili)
{
	ili9322_power(ili, FB_BLANK_POWERDOWN);
}
EXPORT_SYMBOL_GPL(ili9322_shutdown);

MODULE_AUTHOR("Lothar Wassmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("ILI9322 LCD Driver");
MODULE_LICENSE("GPL v2");
