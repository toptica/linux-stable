/*
 * Copyright (c) 2012 Stefano Babic <sbabic@denx.de>
 *
 * Based on work and patch by:
 *
 * Copyright (c) 2010 Grant Erickson <marathon96@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <mach/hardware.h>
#include <plat/dmtimer.h>
#include <linux/pwm.h>
#include <plat/pwm.h>

#define DM_TIMER_LOAD_MIN		0xFFFFFFFE

struct omap_chip {
	struct platform_device	  *pdev;
	struct omap_dm_timer	  *dm_timer;
	struct omap2_pwm_platform_config   config;
	unsigned int pwm_id;

	struct pwm_chip	chip;
};

/*
 * pwm_calc_value - determines the counter value for a clock rate and period.
 * @clk_rate: The clock rate, in Hz, of the PWM's clock source to compute the
 *            counter value for.
 * @ns: The period, in nanoseconds, to computer the counter value for.
 *
 * Returns the PWM counter value for the specified clock rate and period.
 */
static inline int pwm_calc_value(unsigned long clk_rate, int ns)
{
	const unsigned long nanoseconds_per_second = 1000000000;
	int cycles;
	__u64 c;

	c = (__u64)clk_rate * ns;
	do_div(c, nanoseconds_per_second);
	cycles = c;

	return DM_TIMER_LOAD_MIN - cycles;
}

#define to_omap_chip(chip)	container_of(chip, struct omap_chip, chip)

static int omap_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
	int duty_ns, int period_ns)
{
	struct omap_chip *omap = to_omap_chip(chip);

	int status = 0;
	const bool enable = true;
	const bool autoreload = true;
	const bool toggle = true;
	const int trigger = OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE;
	int load_value, match_value;
	unsigned long clk_rate;

	dev_dbg(&omap->pdev->dev,
			"duty cycle: %d, period %d\n",
			duty_ns, period_ns);

	clk_rate = clk_get_rate(omap_dm_timer_get_fclk(omap->dm_timer));

	/*
	 * Calculate the appropriate load and match values based on the
	 * specified period and duty cycle. The load value determines the
	 * cycle time and the match value determines the duty cycle.
	 */

	load_value = pwm_calc_value(clk_rate, period_ns);
	match_value = pwm_calc_value(clk_rate, period_ns - duty_ns);

	/*
	 * We MUST enable yet stop the associated dual-mode timer before
	 * attempting to write its registers.
	 */

	omap_dm_timer_enable(omap->dm_timer);
	omap_dm_timer_stop(omap->dm_timer);

	omap_dm_timer_set_load(omap->dm_timer, autoreload, load_value);
	omap_dm_timer_set_match(omap->dm_timer, enable, match_value);

	dev_dbg(&omap->pdev->dev,
		"load value: %#08x (%d), "
		"match value: %#08x (%d)\n",
		load_value, load_value,
		match_value, match_value);

	omap_dm_timer_set_pwm(omap->dm_timer,
		  !omap->config.polarity,
		  toggle,
		  trigger);

	/* Set the counter to generate an overflow event immediately. */
	omap_dm_timer_write_counter(omap->dm_timer, DM_TIMER_LOAD_MIN);
	omap_dm_timer_start(omap->dm_timer);

	return status;
}

static int omap_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct omap_chip *omap = to_omap_chip(chip);
	int status = 0;

	dev_dbg(&omap->pdev->dev,
		"omap_pwm_enable");
	/*
	 * Enable the counter--always--before attempting to write its
	 * registers and then set the timer to its minimum load value to
	 * ensure we get an overflow event right away once we start it.
	 */

	omap_dm_timer_enable(omap->dm_timer);
	omap_dm_timer_write_counter(omap->dm_timer, DM_TIMER_LOAD_MIN);
	omap_dm_timer_start(omap->dm_timer);

	return status;
}

static void omap_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct omap_chip *omap = to_omap_chip(chip);

	dev_dbg(&omap->pdev->dev,
		"omap_pwm_disable");

	omap_dm_timer_enable(omap->dm_timer);
	omap_dm_timer_stop(omap->dm_timer);
	omap_dm_timer_disable(omap->dm_timer);
}

static struct pwm_ops omap_pwm_ops = {
	.enable = omap_pwm_enable,
	.disable = omap_pwm_disable,
	.config = omap_pwm_config,
	.owner = THIS_MODULE,
};

static int __devinit omap_pwm_probe(struct platform_device *pdev)
{
	struct omap_chip *pwm = NULL;
	struct omap2_pwm_platform_config *pdata = NULL;
	int status = 0;

	pdata = ((struct omap2_pwm_platform_config *)(pdev->dev.platform_data));

	BUG_ON(pdata == NULL);

	if (pdata == NULL) {
		dev_err(&pdev->dev, "Could not find required platform data.\n");
		return -ENOENT;
	}

	/* Allocate memory for the driver-private PWM data and state */

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);

	if (pwm == NULL) {
		dev_err(&pdev->dev, "Could not allocate memory.\n");
		return -ENOMEM;
	}

	/*
	 * Request the OMAP dual-mode timer that will be bound to and
	 * associated with this generic PWM.
	 */
	pwm->dm_timer = omap_dm_timer_request_specific(pdata->timer_id);
	if (pwm->dm_timer == NULL) {
		devm_kfree(&pdev->dev, pwm);
		return -ENOENT;
	}

	pwm->chip.ops = &omap_pwm_ops;
	pwm->chip.dev = &pdev->dev;
	pwm->chip.base = -1;
	pwm->chip.npwm = 3;
	pwm->pdev = pdev;

	printk ("OMAP PWM Timer %d\n", pdata->timer_id);

	/*
	 * Configure the source for the dual-mode timer backing this
	 * generic PWM device. The clock source will ultimately determine
	 * how small or large the PWM frequency can be.
	 *
	 * At some point, it's probably worth revisiting moving this to
	 * the configure method and choosing either the slow- or
	 * system-clock source as appropriate for the desired PWM period.
	 */
	omap_dm_timer_set_source(pwm->dm_timer, OMAP_TIMER_SRC_SYS_CLK);

	pwm->pwm_id = pdev->id;
	pwm->config = *pdata;

	status = pwmchip_add(&pwm->chip);
	if (status < 0)
		return status;

	platform_set_drvdata(pdev, pwm);

	return 0;
}

static int __devexit omap_pwm_remove(struct platform_device *pdev)
{
	struct omap_chip *pwm;

	pwm = platform_get_drvdata(pdev);

	if (pwm == NULL) {
		return -ENODEV;
	}

	omap_dm_timer_free(pwm->dm_timer);

	return pwmchip_remove(&pwm->chip);
}

static struct platform_driver omap_pwm_driver = {
	.driver		= {
		.name	= "omap-pwm",
	},
	.probe		= omap_pwm_probe,
	.remove		= __devexit_p(omap_pwm_remove)
};

static int __init omap_pwm_init(void)
{
	return platform_driver_register(&omap_pwm_driver);
}

static void __exit omap_pwm_exit(void)
{
	platform_driver_unregister(&omap_pwm_driver);
}

module_init(omap_pwm_init);
module_exit(omap_pwm_exit);

MODULE_AUTHOR("Grant Erickson <marathon96@gmail.com>");
MODULE_LICENSE("GPLv2");
