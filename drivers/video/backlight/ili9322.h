/*
 * drivers/video/backlight/ili9322.h
 *
 * ILI9322 LCD controller driver core.
 *
 * (C) Copyright Lothar Wassmann <LW@KARO-electronics.de>
 *
 *   based on: drivers/video/backlight/ili9322.h
 *   Copyright 2007 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/notifier.h>

struct ili9322;

struct ili9322_client {
	const char	*name;
	int	(*init)(struct ili9322 *ili, struct ili9322_platdata *cfg);

};
/* Device attached via an SPI bus. */
struct  ili9322_spi {
	struct spi_device	*dev;
	struct spi_message	message;
	struct spi_transfer	xfer;
	struct ili9322_reg	buffer;
};

/* ILI9322 device state. */
struct ili9322 {
	struct ili9322_spi	spi;	/* SPI attached device. */

	struct device		*dev;
	struct lcd_device	*lcd;	/* LCD device we created. */
	struct ili9322_client	*client;
	struct ili9322_platdata	*platdata;
	struct notifier_block	notifier;

	int			power; /* current power state. */
	int			initialised;

	unsigned char		power1;
	struct mutex		mutex;
	int (*write)(struct ili9322 *ili, unsigned char reg, unsigned char val);
};


/* ILI9322 register access routines */

extern int ili9322_write(struct ili9322 *ili,
			 unsigned char reg, unsigned char value);

extern int ili9322_write_regs(struct ili9322 *ili,
			      struct ili9322_reg *values,
			      int nr_values);

/* Device probe */

extern int ili9322_probe_spi(struct spi_device *spi,
			     struct ili9322_client *cli);

extern int ili9322_remove(struct ili9322 *lcd);
extern void ili9322_shutdown(struct ili9322 *lcd);

/* PM */

extern int ili9322_suspend(struct ili9322 *lcd, pm_message_t state);
extern int ili9322_resume(struct ili9322 *lcd);
