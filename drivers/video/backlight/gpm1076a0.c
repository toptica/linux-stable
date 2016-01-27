/*
 * drivers/video/backlight/gpm1076a0.c
 *
 * GiantPlus GPM1067A0 (ILI9322) LCD controller driver.
 * (C) Copyright Lothar Wassmann <LW@KARO-electronics.de>
 *
 *    based on: drivers/video/backlight/vgg2432a4.c
 *    Copyright 2007 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
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

/* Device initialisation sequences */

static struct ili9322_reg gpm_init1[] = {
	{
		.address = ILI9322_REG_GRESET,
		.value = 0, /* reset all registers to default */
	},
	{
		.address = ILI9322_REG_GRESET,
		.value = 1, /* Normal operation */
	},
	{
		.address = ILI9322_REG_VREG1OUT,
		.value	 = 0x16,
	},
	{
		.address = ILI9322_REG_VCOMH,
		.value	 = 0x3b,
	},
};

static struct ili9322_reg gpm_gamma[] = {
	{
		.address = ILI9322_REG_GAMMA1,
		.value	 = 0xa7,
	}, {
		.address = ILI9322_REG_GAMMA2,
		.value   = 0x57,
	}, {
		.address = ILI9322_REG_GAMMA3,
		.value	 = 0x73,
	}, {
		.address = ILI9322_REG_GAMMA4,
		.value	 = 0x72,
	}, {
		.address = ILI9322_REG_GAMMA5,
		.value	 = 0x73,
	}, {
		.address = ILI9322_REG_GAMMA6,
		.value	 = 0x55,
	}, {
		.address = ILI9322_REG_GAMMA7,
		.value	 = 0x17,
	}, {
		.address = ILI9322_REG_GAMMA8,
		.value	 = 0x62,
	},
};

static int gpm1076a0_lcd_init(struct ili9322 *lcd,
			      struct ili9322_platdata *pdata)
{
	int ret;
	unsigned char reg;

	ret = ili9322_write_regs(lcd, gpm_init1, ARRAY_SIZE(gpm_init1));
	if (ret)
		return ret;

	switch (pdata->intf_mode) {
	case ILI9322_HS_VS_DE_MODE:
		reg = 1 << 2;
		break;
	case ILI9322_HS_VS_MODE:
		reg = 0;
		break;
	case ILI9322_DE_MODE:
		reg = 2 << 2;
		break;
	default:
		return -EINVAL;
	}
	reg |= 1;
	ret = ili9322_write(lcd, ILI9322_REG_DISPLAY, reg);
	if (ret)
		return ret;

	ret = ili9322_write(lcd, ILI9322_REG_DCDC, 0x22);
	if (ret)
		return ret;

	reg = 0x0f;
	if (pdata->vert_flip) {
		reg &= ~1;
	}
	if (pdata->horiz_flip) {
		reg &= ~2;
	}
	switch (pdata->data_fmt) {
	case ILI9322_FMT_RGB8:
		break;
	case ILI9322_FMT_RGB24:
		reg |= 0x50;
		break;
	case ILI9322_FMT_ITU_BT601:
		reg |= 0x80;
		break;
	case ILI9322_FMT_ITU_BT656:
		reg |= 0xa0;
		break;
	}
	ret = ili9322_write(lcd, ILI9322_REG_ENTRYCTL, reg);
	if (ret)
		return ret;

	if (pdata->vert_bp) {
		ret = ili9322_write(lcd, ILI9322_REG_VERTBP, pdata->vert_bp);
		if (ret)
			return ret;
	}

	if (pdata->horiz_bp) {
		ret = ili9322_write(lcd, ILI9322_REG_HORIZBP, pdata->horiz_bp);
		if (ret)
			return ret;
	}

	ret = ili9322_write_regs(lcd, gpm_gamma, ARRAY_SIZE(gpm_gamma));
	if (ret)
		return ret;

	if (pdata->reg_data == NULL) {
		return pdata->reg_cnt > 0 ? -EINVAL : 0;
	} else if (pdata->reg_cnt > 0) {
		int i;

		for (i = 0; i < pdata->reg_cnt; i++) {
			ili9322_write(lcd, pdata->reg_data[i].address,
				pdata->reg_data[i].value);
		}
	}
	return 0;
}

#ifdef CONFIG_PM
static int gpm1076a0_suspend(struct spi_device *spi, pm_message_t state)
{
	return ili9322_suspend(dev_get_drvdata(&spi->dev), state);
}

static int gpm1076a0_resume(struct spi_device *spi)
{
	return ili9322_resume(dev_get_drvdata(&spi->dev));
}
#else
#define gpm1076a0_suspend	NULL
#define gpm1076a0_resume	NULL
#endif

static struct ili9322_client gpm1076a0_client = {
	.name	= "GPM1076A0",
	.init	= gpm1076a0_lcd_init,
};

/* Device probe */

static int __devinit gpm1076a0_probe(struct spi_device *spi)
{
	int ret;

	ret = ili9322_probe_spi(spi, &gpm1076a0_client);
	if (ret != 0) {
		dev_err(&spi->dev, "failed to initialise ili9322\n");
		return ret;
	}

	return 0;
}

static int __devexit gpm1076a0_remove(struct spi_device *spi)
{
	return ili9322_remove(dev_get_drvdata(&spi->dev));
}

static void gpm1076a0_shutdown(struct spi_device *spi)
{
	ili9322_shutdown(dev_get_drvdata(&spi->dev));
}

static struct spi_driver gpm1076a0_driver = {
	.driver = {
		.name		= "GPM1076A0",
		.owner		= THIS_MODULE,
	},
	.probe		= gpm1076a0_probe,
	.remove		= __devexit_p(gpm1076a0_remove),
	.shutdown	= gpm1076a0_shutdown,
	.suspend	= gpm1076a0_suspend,
	.resume		= gpm1076a0_resume,
};

/* Device driver initialisation */

static int __init gpm1076a0_init(void)
{
	return spi_register_driver(&gpm1076a0_driver);
}
module_init(gpm1076a0_init);

static void __exit gpm1076a0_exit(void)
{
	spi_unregister_driver(&gpm1076a0_driver);
}
module_exit(gpm1076a0_exit);

MODULE_AUTHOR("Lothar Wassmann <LW@KARO-electronics.de>");
MODULE_DESCRIPTION("GPM1076A0 LCD Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:GPM1076A0");
