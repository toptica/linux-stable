/*
 * Copyright (C) 2007  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * platform_data definitions for fec_enet device
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

struct fec_enet_platform_data {
	/* callback for platform specific initialization */
	int (*arch_init)(struct platform_device *dev);
	void (*arch_exit)(struct platform_device *dev);
	int (*set_mac_addr)(struct platform_device *dev, char *addr);
	int (*suspend)(struct platform_device *dev);
	int (*resume)(struct platform_device *dev);
};
