/*
 *  Freescale i.MX25 Touch Screen Driver
 *
 *  Copyright (c) 2009 Lothar Wassmann <LW@KARO-electronics.de>
 *
 * Based on code from Freescale BSP
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

typedef enum {
	MXC_TSC_4WIRE,
	MXC_TSC_5WIRE,
} mxc_tsc_mode;

struct mxc_tsc_pdata {
	int pen_debounce_time;	/* 0: disable debounce;
				 * 1..128: # of ADC clock cycles / 8 */
	unsigned int intref:1,	/* 0|1: internal reference disabled|enabled */
		hsyncen:1,	/* synchronize measurements with LCD HSYNC */
		hsyncpol:1;	/* select HSYNC polarity: 1 == active low */
	unsigned int r_xplate;	/* resistance (in Ohms) of X plate
				 * (required for pressure measurement */
	int adc_clk;		/* ADC clock frequency in Hz (max. 1750000);
				 * <= 0: use default (1666667) */
	mxc_tsc_mode tsc_mode;	/* select 4 wire or 5 wire mode */
};
