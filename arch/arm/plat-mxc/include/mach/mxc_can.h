/*
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
 */

#include <linux/platform_device.h>

struct flexcan_platform_data {
	char *core_reg;
	char *io_reg;
	int (*xcvr_enable)(struct platform_device *pdev, int en);
	int (*active)(struct platform_device *pdev);
	void (*inactive)(struct platform_device *pdev);
};
