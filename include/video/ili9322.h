/*
 * include/video/ili9322.h
 *
 * ILI9322 LCD controller configuration control.
 *
 * (C) Copyright Lothar Wassmann <LW@KARO-electronics.de>
 *
 *   based on: include/video/ili9320.c
 *     Copyright 2007 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* SPI interface definitions */

#define ILI9322_SPI_IDCODE		0x93
#define ILI9322_SPI_READ		(1 << 7)
#define ILI9322_SPI_WRITE		(0 << 7)

#define ILI9322_REG(addr, access)	((addr) | (access))

#define ILI9322_REG_ID			ILI9322_REG(0x00, ILI9322_SPI_READ) /* read only */
#define ILI9322_REG_VCOMA		ILI9322_REG(0x01, ILI9322_SPI_WRITE) /* write only */
#define ILI9322_REG_VCOMH		ILI9322_REG(0x02, ILI9322_SPI_WRITE) /* ... */
#define ILI9322_REG_VREG1OUT		ILI9322_REG(0x03, ILI9322_SPI_WRITE)
#define ILI9322_REG_GRESET		ILI9322_REG(0x04, ILI9322_SPI_WRITE)
#define ILI9322_REG_POWER1		ILI9322_REG(0x05, ILI9322_SPI_WRITE)
#define ILI9322_REG_ENTRYCTL		ILI9322_REG(0x06, ILI9322_SPI_WRITE)
#define ILI9322_REG_POWERCTL		ILI9322_REG(0x07, ILI9322_SPI_WRITE)
#define ILI9322_REG_VERTBP		ILI9322_REG(0x08, ILI9322_SPI_WRITE)
#define ILI9322_REG_HORIZBP		ILI9322_REG(0x09, ILI9322_SPI_WRITE)
#define ILI9322_REG_POLARITY		ILI9322_REG(0x0a, ILI9322_SPI_WRITE)
#define ILI9322_REG_DISPLAY		ILI9322_REG(0x0b, ILI9322_SPI_WRITE)

#define ILI9322_REG_DCDC		ILI9322_REG(0x0c, ILI9322_SPI_WRITE)
#define ILI9322_REG_DRIVING		ILI9322_REG(0x0d, ILI9322_SPI_WRITE)
#define ILI9322_REG_CONTRAST		ILI9322_REG(0x0e, ILI9322_SPI_WRITE)
#define ILI9322_REG_BRIGHTNESS		ILI9322_REG(0x0f, ILI9322_SPI_WRITE)

#define ILI9322_REG_GAMMA1		ILI9322_REG(0x10, ILI9322_SPI_WRITE)
#define ILI9322_REG_GAMMA2		ILI9322_REG(0x11, ILI9322_SPI_WRITE)
#define ILI9322_REG_GAMMA3		ILI9322_REG(0x12, ILI9322_SPI_WRITE)
#define ILI9322_REG_GAMMA4		ILI9322_REG(0x13, ILI9322_SPI_WRITE)
#define ILI9322_REG_GAMMA5		ILI9322_REG(0x14, ILI9322_SPI_WRITE)
#define ILI9322_REG_GAMMA6		ILI9322_REG(0x15, ILI9322_SPI_WRITE)
#define ILI9322_REG_GAMMA7		ILI9322_REG(0x16, ILI9322_SPI_WRITE)
#define ILI9322_REG_GAMMA8		ILI9322_REG(0x17, ILI9322_SPI_WRITE)

#define ILI9322_REG_POWER2		ILI9322_REG(0x30, ILI9322_SPI_WRITE)

#define ILI9322_REG_OTP_PROG		ILI9322_REG(0x42, ILI9322_SPI_WRITE)
#define ILI9322_REG_OTP_STATUS		ILI9322_REG(0x43, ILI9322_SPI_WRITE)
#define ILI9322_REG_OTP_KEY		ILI9322_REG(0x44, ILI9322_SPI_WRITE)


/* platform data to pass configuration from lcd */

enum ili9322_suspend {
	ILI9322_SUSPEND_OFF,
	ILI9322_SUSPEND_DEEP,
};

enum ili9322_intf_mode {
	ILI9322_HS_VS_DE_MODE,
	ILI9322_HS_VS_MODE,
	ILI9322_DE_MODE,
};

enum ili9322_data_fmt {
	ILI9322_FMT_RGB8,
	ILI9322_FMT_RGB24,
	ILI9322_FMT_ITU_BT601,
	ILI9322_FMT_ITU_BT656,
};

/* Holder for register and value pairs. */
struct ili9322_reg {
	unsigned char	address;
	unsigned char	value;
};

struct ili9322_platdata {
	enum ili9322_suspend suspend;

	/* set the reset line, 1 = reset asserted, 0 = reset deasserted */
	void		(*reset)(int assert);
	/* switch display power 1 = on, 0 = off */
	void		(*power)(int on);

	unsigned int	intf_mode:2,
			data_fmt:2,
			vert_flip:1,
			horiz_flip:1;
	int		vert_bp;
	int		horiz_bp;
	int		reg_cnt;
	struct ili9322_reg *reg_data;
};
