/*
 * include/asm-arm/arch-mxc/ssi_port.h
 *
 * Copyright (C) 2008  Lothar Wassmann <LW@KARO-electronics.de>
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

struct mxc_ssi_port {
	int num;
	struct module *owner;
	struct platform_device *parent;
	struct resource *res;
	struct clk *ssi_clk;
	int in_use;
};

extern int mxc_ssi_request_port(int num, struct platform_device *parent,
				struct mxc_ssi_port **ssi_port);
extern void mxc_ssi_release_port(struct mxc_ssi_port *ssi_port);
