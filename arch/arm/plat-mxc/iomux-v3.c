/*
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 by Sascha Hauer <kernel@pengutronix.de>
 * Copyright (C) 2009 by Jan Weitzel Phytec Messtechnik GmbH,
 *                       <armlinux@phytec.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach/map.h>
#include <mach/iomux-v3.h>

static void __iomem *base;
#ifdef CONFIG_ARCH_MX25
#define MAX_PAD_OFS	0x228
#else
#define MAX_PAD_OFS	0x200
#endif

static unsigned long iomux_v3_pad_alloc_map[(MAX_PAD_OFS + BITS_PER_LONG) / BITS_PER_LONG];

static inline int mxc_iomux_v3_pad_offset(struct pad_desc *pad)
{
	int pad_ofs;

	if (cpu_is_mx25())
		pad_ofs = pad->mux_ctrl_ofs;
	else
		pad_ofs = pad->pad_ctrl_ofs;
	BUG_ON(pad_ofs > MAX_PAD_OFS);
	return pad_ofs;
}

#ifdef DEBUG
static inline int mxc_iomux_v3_check_pad_setup(struct pad_desc *pad)
{
	u32 val;
	int ret = 0;

	val = __raw_readl(base + pad->mux_ctrl_ofs) ^ pad->mux_mode;
	if (unlikely(val)) {
		printk(KERN_ERR "%s: Invalid MUX_CTRL bits: 0x%04x for pad offset 0x%03x\n",
			__FUNCTION__, val, pad->mux_ctrl_ofs);
		ret = 1;
	}

	val = __raw_readl(base + pad->select_input_ofs) ^ pad->select_input;
	if (unlikely(val)) {
		printk(KERN_ERR "%s: Invalid SELECT_INPUT bits: 0x%04x for pad offset 0x%03x\n",
			__FUNCTION__, val, pad->select_input_ofs);
		ret = 1;
	}

	if (pad->pad_ctrl & NO_PAD_CTRL) {
		if (pad->pad_ctrl != NO_PAD_CTRL) {
			printk(KERN_ERR "%s: PAD_CTL settings masked by 'NO_PAD_CTRL' for pad offset 0x%03x\n",
			__FUNCTION__, pad->pad_ctrl_ofs);
			ret = 1;
		}
		return ret;
	}

	val = __raw_readl(base + pad->pad_ctrl_ofs) ^ pad->pad_ctrl;
	if (unlikely(val)) {
		printk(KERN_ERR "%s: Invalid PAD_CTRL bits: 0x%04x for pad offset 0x%03x\n",
			__FUNCTION__, val, pad->pad_ctrl_ofs);
		ret = 1;
	}
	return ret;
}
#else
static inline int mxc_iomux_v3_check_pad_setup(struct pad_desc *pad)
{
	return 0;
}
#endif

/*
 * setups a single pin:
 *	- reserves the pin so that it is not claimed by another driver
 *	- setups the iomux according to the configuration
 */
int mxc_iomux_v3_setup_pad(struct pad_desc *pad)
{
	unsigned int pad_ofs = mxc_iomux_v3_pad_offset(pad);

	if (test_and_set_bit(pad_ofs >> 2, iomux_v3_pad_alloc_map))
		return -EBUSY;
	if (pad->mux_ctrl_ofs)
		__raw_writel(pad->mux_mode, base + pad->mux_ctrl_ofs);

	if (pad->select_input_ofs)
		__raw_writel(pad->select_input,
				base + pad->select_input_ofs);

	if (!(pad->pad_ctrl & NO_PAD_CTRL) && pad->pad_ctrl_ofs)
		__raw_writel(pad->pad_ctrl, base + pad->pad_ctrl_ofs);
	WARN_ON(mxc_iomux_v3_check_pad_setup(pad));
	return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_setup_pad);

int mxc_iomux_v3_setup_multiple_pads(struct pad_desc *pad_list, unsigned count)
{
	struct pad_desc *p = pad_list;
	int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = mxc_iomux_v3_setup_pad(p);
		if (ret) {
			printk(KERN_ERR "Failed to setup PAD[%d]: %d\n", i, ret);
			goto setup_error;
		}
		p++;
	}
	return 0;

setup_error:
	mxc_iomux_v3_release_multiple_pads(pad_list, i);
	return ret;
}
EXPORT_SYMBOL(mxc_iomux_v3_setup_multiple_pads);

void mxc_iomux_v3_release_pad(struct pad_desc *pad)
{
	unsigned int pad_ofs = mxc_iomux_v3_pad_offset(pad);

	clear_bit(pad_ofs >> 2, iomux_v3_pad_alloc_map);
}
EXPORT_SYMBOL(mxc_iomux_v3_release_pad);

void mxc_iomux_v3_release_multiple_pads(struct pad_desc *pad_list, int count)
{
	struct pad_desc *p = pad_list;
	int i;

	for (i = 0; i < count; i++) {
		mxc_iomux_v3_release_pad(p);
		p++;
	}
}
EXPORT_SYMBOL(mxc_iomux_v3_release_multiple_pads);

void __init mxc_iomux_v3_init(void __iomem *iomux_v3_base)
{
	base = iomux_v3_base;
}
