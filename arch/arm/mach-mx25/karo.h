/*
 * arch/arm/mach-mx25/karo.h
 *
 * Copyright (C) 2009  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
 *
 * This file provides platform specific definitions for the
 * Ka-Ro electronics TX25 processor modules
 */

enum {
	BOARD_KARO_STK5,
};

extern int karo_board_type;
extern int karo_mod_type;

#ifdef DEBUG
extern int tx25_debug;
#define dbg_lvl(n)	((n) < tx25_debug)
#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#else
#define dbg_lvl(n)	0
#define DBG(lvl, fmt...)	do { } while (0)
#endif

static inline int karo_get_board_type(void)
{
	return karo_board_type;
}

static inline int karo_get_module_type(void)
{
	return karo_mod_type;
}
