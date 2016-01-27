/*
 * drivers/net/can/flexcan.c
 *
 * Copyright (C) 2009  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * based on: drivers/net/can/flexcan/
 *   Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

/*
 * Driver for Freescale CAN Controller FlexCAN.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/mxc_can.h>

#include "flexcan.h"

#ifdef DEBUG
static int debug = 0;
#define dbg_lvl(n)	((n) < debug)
module_param(debug, int, S_IRUGO | S_IWUSR);

#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#else
static int debug;
#define dbg_lvl(n)	0
module_param(debug, int, 0);

#define DBG(lvl, fmt...)	do { } while (0)
#endif

#define ndev_dbg(d, l, fmt...)		do { if (dbg_lvl(l)) dev_dbg(&(d)->dev, fmt); } while (0)
#define pdev_dbg(p, l, fmt...)		do { if (dbg_lvl(l)) dev_dbg(&(p)->dev, fmt); } while (0)
#define flexcan_dbg(f, l, fmt...)	do { if (dbg_lvl(l)) dev_dbg(&(f)->dev->dev, fmt); } while (0)

#define ndev_err(d, fmt...)	dev_err(&(d)->dev, fmt)
#define pdev_err(p, fmt...)	dev_err(&(p)->dev, fmt)

enum {
	FLEXCAN_ATTR_STATE = 0,
	FLEXCAN_ATTR_BITRATE,
	FLEXCAN_ATTR_BR_PRESDIV,
	FLEXCAN_ATTR_BR_RJW,
	FLEXCAN_ATTR_BR_PROPSEG,
	FLEXCAN_ATTR_BR_PSEG1,
	FLEXCAN_ATTR_BR_PSEG2,
	FLEXCAN_ATTR_BR_CLKSRC,
	FLEXCAN_ATTR_MAXMB,
	FLEXCAN_ATTR_XMIT_MAXMB,
	FLEXCAN_ATTR_FIFO,
	FLEXCAN_ATTR_WAKEUP,
	FLEXCAN_ATTR_SRX_DIS,
	FLEXCAN_ATTR_WAK_SRC,
	FLEXCAN_ATTR_BCC,
	FLEXCAN_ATTR_LOCAL_PRIORITY,
	FLEXCAN_ATTR_ABORT,
	FLEXCAN_ATTR_LOOPBACK,
	FLEXCAN_ATTR_SMP,
	FLEXCAN_ATTR_BOFF_REC,
	FLEXCAN_ATTR_TSYN,
	FLEXCAN_ATTR_LISTEN,
	FLEXCAN_ATTR_EXTEND_MSG,
	FLEXCAN_ATTR_STANDARD_MSG,
#ifdef CONFIG_CAN_DEBUG_DEVICES
	FLEXCAN_ATTR_DUMP_REG,
	FLEXCAN_ATTR_DUMP_XMIT_MB,
	FLEXCAN_ATTR_DUMP_RX_MB,
#endif
	FLEXCAN_ATTR_MAX
};

#ifdef DEBUG
#define flexcan_reg_read(f,r)	_flexcan_reg_read(f, r, #r, __FUNCTION__)
static inline unsigned long _flexcan_reg_read(struct flexcan_device *flexcan, int reg,
					      const char *name, const char *fn)
{
	unsigned long val;
	val = __raw_readl(flexcan->io_base + reg);
	DBG(2, "%s: Read %08lx from %s[%p]\n", fn, val, name,
	    flexcan->io_base + reg);
	return val;
}

#define flexcan_reg_write(f,r,v)	_flexcan_reg_write(f, r, v, #r, __FUNCTION__)
static inline void _flexcan_reg_write(struct flexcan_device *flexcan, int reg, unsigned long val,
				      const char *name, const char *fn)
{
	DBG(2, "%s: Writing %08lx to %s[%p]\n", fn, val, name, flexcan->io_base + reg);
	__raw_writel(val, flexcan->io_base + reg);
}
#else
static inline unsigned long flexcan_reg_read(struct flexcan_device *flexcan, int reg)
{
	return __raw_readl(flexcan->io_base + reg);
}

static inline void flexcan_reg_write(struct flexcan_device *flexcan, int reg, unsigned long val)
{
	__raw_writel(val, flexcan->io_base + reg);
}
#endif

static ssize_t flexcan_show_attr(struct device *dev,
				 struct device_attribute *attr, char *buf);
static ssize_t flexcan_set_attr(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count);

static struct device_attribute flexcan_dev_attr[FLEXCAN_ATTR_MAX] = {
	[FLEXCAN_ATTR_STATE] = __ATTR(state, 0444, flexcan_show_attr, NULL),
	[FLEXCAN_ATTR_BITRATE] =
	    __ATTR(bitrate, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PRESDIV] =
	    __ATTR(br_presdiv, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_RJW] =
	    __ATTR(br_rjw, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PROPSEG] =
	    __ATTR(br_propseg, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PSEG1] =
	    __ATTR(br_pseg1, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PSEG2] =
	    __ATTR(br_pseg2, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_CLKSRC] =
	    __ATTR(br_clksrc, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_MAXMB] =
	    __ATTR(maxmb, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_XMIT_MAXMB] =
	    __ATTR(xmit_maxmb, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_FIFO] =
	    __ATTR(fifo, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_WAKEUP] =
	    __ATTR(wakeup, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_SRX_DIS] =
	    __ATTR(srx_dis, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_WAK_SRC] =
	    __ATTR(wak_src, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BCC] =
	    __ATTR(bcc, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_LOCAL_PRIORITY] =
	    __ATTR(local_priority, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_ABORT] =
	    __ATTR(abort, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_LOOPBACK] =
	    __ATTR(loopback, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_SMP] =
	    __ATTR(smp, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BOFF_REC] =
	    __ATTR(boff_rec, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_TSYN] =
	    __ATTR(tsyn, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_LISTEN] =
	    __ATTR(listen, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_EXTEND_MSG] =
	    __ATTR(ext_msg, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_STANDARD_MSG] =
	    __ATTR(std_msg, 0644, flexcan_show_attr, flexcan_set_attr),
#ifdef CONFIG_CAN_DEBUG_DEVICES
	[FLEXCAN_ATTR_DUMP_REG] =
	    __ATTR(dump_reg, 0444, flexcan_show_attr, NULL),
	[FLEXCAN_ATTR_DUMP_XMIT_MB] =
	    __ATTR(dump_xmit_mb, 0444, flexcan_show_attr, NULL),
	[FLEXCAN_ATTR_DUMP_RX_MB] =
	    __ATTR(dump_rx_mb, 0444, flexcan_show_attr, NULL),
#endif
};

static void flexcan_set_bitrate(struct flexcan_device *flexcan, int bitrate)
{
	/* TODO:: implement in future
	 * based on the bitrate to get the timing of
	 * presdiv, pseg1, pseg2, propseg
	 */
}

static void flexcan_update_bitrate(struct flexcan_device *flexcan)
{
	int rate, div;

	if (flexcan->br_clksrc)
		rate = clk_get_rate(flexcan->clk);
	else {
		struct clk *clk;
		clk = clk_get(NULL, "ckih");
		if (IS_ERR(clk))
			return;
		rate = clk_get_rate(clk);
		clk_put(clk);
	}
	if (!rate)
		return;

	flexcan_dbg(flexcan, 0, "%s: master clock rate %u from %s\n",
		    __FUNCTION__, rate, flexcan->br_clksrc ? "osc" : "pll");

	div = flexcan->br_presdiv + 1;
	div *= flexcan->br_propseg + flexcan->br_pseg1 + flexcan->br_pseg2 + 4;
	flexcan->bitrate = (rate + div - 1) / div;

	flexcan_dbg(flexcan, 0, "%s: flexcan bitrate %u time quantum %uns\n",
		    __FUNCTION__, flexcan->bitrate,
		    1000000000 / (flexcan->bitrate *
				  (flexcan->br_propseg + flexcan->br_pseg1 +
				   flexcan->br_pseg2 + 4)));
}

static inline void flexcan_read_hw_mb(struct flexcan_device *flexcan, struct can_hw_mb *mb,
				      int buf_no)
{
	__raw_readsl(mb, flexcan->hwmb + buf_no * sizeof(*mb), sizeof(*mb));
}

static inline void flexcan_write_hw_mb(struct flexcan_device *flexcan, struct can_hw_mb *mb,
				       int buf_no)
{
	__raw_writesl(flexcan->hwmb + buf_no * sizeof(*mb), mb, sizeof(*mb));
}

#ifdef CONFIG_CAN_DEBUG_DEVICES
static int flexcan_dump_reg(struct flexcan_device *flexcan, char *buf)
{
	int ret = 0;
	unsigned int reg;

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	ret += sprintf(buf + ret, "MCR::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_CTRL);
	ret += sprintf(buf + ret, "CTRL::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_RXGMASK);
	ret += sprintf(buf + ret, "RXGMASK::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_RX14MASK);
	ret += sprintf(buf + ret, "RX14MASK::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_RX15MASK);
	ret += sprintf(buf + ret, "RX15MASK::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_ECR);
	ret += sprintf(buf + ret, "ECR::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_ESR);
	ret += sprintf(buf + ret, "ESR::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_IMASK2);
	ret += sprintf(buf + ret, "IMASK2::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_IMASK1);
	ret += sprintf(buf + ret, "IMASK1::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_IFLAG2);
	ret += sprintf(buf + ret, "IFLAG2::0x%08x\n", reg);
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_IFLAG1);
	ret += sprintf(buf + ret, "IFLAG1::0x%08x\n", reg);
	return ret;
}

static int flexcan_dump_xmit_mb(struct flexcan_device *flexcan, char *buf)
{
	int ret = 0, i;

	clk_enable(flexcan->clk);
	for (i = flexcan->xmit_maxmb + 1; i <= flexcan->maxmb; i++) {
		int j;

		ret += sprintf(buf + ret,
			       "mb[%d]::CS:0x%08x ID:0x%08x DATA",
			       i, flexcan->hwmb[i].mb_cs.data,
			       flexcan->hwmb[i].mb_id);
		for (j = 0; j < sizeof(flexcan->hwmb[i].mb_data); j++) {
			ret += sprintf(buf + ret, ":%02x",
				       flexcan->hwmb[i].mb_data[j]);
		}
		ret += sprintf(buf + ret, "\n");
	}
	clk_disable(flexcan->clk);
	return ret;
}

static int flexcan_dump_rx_mb(struct flexcan_device *flexcan, char *buf)
{
	int ret = 0, i;

	clk_enable(flexcan->clk);
	for (i = 0; i <= flexcan->xmit_maxmb; i++) {
		int j;

		ret += sprintf(buf + ret,
			       "mb[%d]::CS:0x%08x ID:0x%08x DATA",
			       i, flexcan->hwmb[i].mb_cs.data,
			       flexcan->hwmb[i].mb_id);
		for (j = 0; j < sizeof(flexcan->hwmb[i].mb_data); j++) {
			ret += sprintf(buf + ret, ":%02x",
				       flexcan->hwmb[i].mb_data[j]);
		}
		ret += sprintf(buf + ret, "\n");
	}
	clk_disable(flexcan->clk);
	return ret;
}
#endif

static ssize_t flexcan_show_state(struct net_device *net, char *buf)
{
	int ret, esr;
	struct flexcan_device *flexcan = netdev_priv(net);

	ret = sprintf(buf, "%s::", netif_running(net) ? "Start" : "Stop");
	if (netif_carrier_ok(net)) {
		esr = flexcan_reg_read(flexcan, CAN_HW_REG_ESR);
		switch ((esr & __ESR_FLT_CONF_MASK) >> __ESR_FLT_CONF_OFF) {
		case 0:
			ret += sprintf(buf + ret, "normal\n");
			break;
		case 1:
			ret += sprintf(buf + ret, "error passive\n");
			break;
		default:
			ret += sprintf(buf + ret, "bus off\n");
		}
	} else
		ret += sprintf(buf + ret, "bus off\n");
	return ret;
}

static ssize_t flexcan_show_attr(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int attr_id;
	struct net_device *net = dev_get_drvdata(dev);
	struct flexcan_device *flexcan = netdev_priv(net);

	attr_id = attr - flexcan_dev_attr;
	switch (attr_id) {
	case FLEXCAN_ATTR_STATE:
		return flexcan_show_state(net, buf);
	case FLEXCAN_ATTR_BITRATE:
		return sprintf(buf, "%d\n", flexcan->bitrate);
	case FLEXCAN_ATTR_BR_PRESDIV:
		return sprintf(buf, "%d\n", flexcan->br_presdiv + 1);
	case FLEXCAN_ATTR_BR_RJW:
		return sprintf(buf, "%d\n", flexcan->br_rjw);
	case FLEXCAN_ATTR_BR_PROPSEG:
		return sprintf(buf, "%d\n", flexcan->br_propseg + 1);
	case FLEXCAN_ATTR_BR_PSEG1:
		return sprintf(buf, "%d\n", flexcan->br_pseg1 + 1);
	case FLEXCAN_ATTR_BR_PSEG2:
		return sprintf(buf, "%d\n", flexcan->br_pseg2 + 1);
	case FLEXCAN_ATTR_BR_CLKSRC:
		return sprintf(buf, "%s\n", flexcan->br_clksrc ? "bus" : "osc");
	case FLEXCAN_ATTR_MAXMB:
		return sprintf(buf, "%d\n", flexcan->maxmb + 1);
	case FLEXCAN_ATTR_XMIT_MAXMB:
		return sprintf(buf, "%d\n", flexcan->xmit_maxmb + 1);
	case FLEXCAN_ATTR_FIFO:
		return sprintf(buf, "%d\n", flexcan->fifo);
	case FLEXCAN_ATTR_WAKEUP:
		return sprintf(buf, "%d\n", flexcan->wakeup);
	case FLEXCAN_ATTR_SRX_DIS:
		return sprintf(buf, "%d\n", flexcan->srx_dis);
	case FLEXCAN_ATTR_WAK_SRC:
		return sprintf(buf, "%d\n", flexcan->wak_src);
	case FLEXCAN_ATTR_BCC:
		return sprintf(buf, "%d\n", flexcan->bcc);
	case FLEXCAN_ATTR_LOCAL_PRIORITY:
		return sprintf(buf, "%d\n", flexcan->lprio);
	case FLEXCAN_ATTR_ABORT:
		return sprintf(buf, "%d\n", flexcan->abort);
	case FLEXCAN_ATTR_LOOPBACK:
		return sprintf(buf, "%d\n", flexcan->loopback);
	case FLEXCAN_ATTR_SMP:
		return sprintf(buf, "%d\n", flexcan->smp);
	case FLEXCAN_ATTR_BOFF_REC:
		return sprintf(buf, "%d\n", flexcan->boff_rec);
	case FLEXCAN_ATTR_TSYN:
		return sprintf(buf, "%d\n", flexcan->tsyn);
	case FLEXCAN_ATTR_LISTEN:
		return sprintf(buf, "%d\n", flexcan->listen);
	case FLEXCAN_ATTR_EXTEND_MSG:
		return sprintf(buf, "%d\n", flexcan->ext_msg);
	case FLEXCAN_ATTR_STANDARD_MSG:
		return sprintf(buf, "%d\n", flexcan->std_msg);
#ifdef CONFIG_CAN_DEBUG_DEVICES
	case FLEXCAN_ATTR_DUMP_REG:
		return flexcan_dump_reg(flexcan, buf);
	case FLEXCAN_ATTR_DUMP_XMIT_MB:
		return flexcan_dump_xmit_mb(flexcan, buf);
	case FLEXCAN_ATTR_DUMP_RX_MB:
		return flexcan_dump_rx_mb(flexcan, buf);
#endif
	default:
		return sprintf(buf, "%s:%p->%p\n", __func__, flexcan_dev_attr,
			       attr);
	}
}

static ssize_t flexcan_set_attr(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int attr_id, tmp;
	struct net_device *net = dev_get_drvdata(dev);
	struct flexcan_device *flexcan = netdev_priv(net);

	attr_id = attr - flexcan_dev_attr;

	mutex_lock(&flexcan->mutex);

	if (netif_running(net))
		goto set_finish;

	if (attr_id == FLEXCAN_ATTR_BR_CLKSRC) {
		if (!strcasecmp(buf, "bus"))
			flexcan->br_clksrc = 1;
		else if (!strcasecmp(buf, "osc"))
			flexcan->br_clksrc = 0;
		goto set_finish;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	switch (attr_id) {
	case FLEXCAN_ATTR_BITRATE:
		flexcan_set_bitrate(flexcan, tmp);
		break;
	case FLEXCAN_ATTR_BR_PRESDIV:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PRESDIV)) {
			flexcan->br_presdiv = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_BR_RJW:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_RJW))
			flexcan->br_rjw = tmp - 1;
		break;
	case FLEXCAN_ATTR_BR_PROPSEG:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PROPSEG)) {
			flexcan->br_propseg = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_BR_PSEG1:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PSEG1)) {
			flexcan->br_pseg1 = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_BR_PSEG2:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PSEG2)) {
			flexcan->br_pseg2 = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_MAXMB:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_MB)) {
			if (flexcan->maxmb != (tmp - 1)) {
				flexcan->maxmb = tmp - 1;
				if (flexcan->xmit_maxmb < flexcan->maxmb)
					flexcan->xmit_maxmb = flexcan->maxmb;
			}
		}
		break;
	case FLEXCAN_ATTR_XMIT_MAXMB:
		if ((tmp > 0) && (tmp <= (flexcan->maxmb + 1))) {
			if (flexcan->xmit_maxmb != (tmp - 1))
				flexcan->xmit_maxmb = tmp - 1;
		}
		break;
	case FLEXCAN_ATTR_FIFO:
		flexcan->fifo = !!tmp;
		break;
	case FLEXCAN_ATTR_WAKEUP:
		flexcan->wakeup = !!tmp;
		break;
	case FLEXCAN_ATTR_SRX_DIS:
		flexcan->srx_dis = !!tmp;
		break;
	case FLEXCAN_ATTR_WAK_SRC:
		flexcan->wak_src = !!tmp;
		break;
	case FLEXCAN_ATTR_BCC:
		flexcan->bcc = !!tmp;
		break;
	case FLEXCAN_ATTR_LOCAL_PRIORITY:
		flexcan->lprio = !!tmp;
		break;
	case FLEXCAN_ATTR_ABORT:
		flexcan->abort = !!tmp;
		break;
	case FLEXCAN_ATTR_LOOPBACK:
		flexcan->loopback = !!tmp;
		break;
	case FLEXCAN_ATTR_SMP:
		flexcan->smp = !!tmp;
		break;
	case FLEXCAN_ATTR_BOFF_REC:
		flexcan->boff_rec = !!tmp;
		break;
	case FLEXCAN_ATTR_TSYN:
		flexcan->tsyn = !!tmp;
		break;
	case FLEXCAN_ATTR_LISTEN:
		flexcan->listen = !!tmp;
		break;
	case FLEXCAN_ATTR_EXTEND_MSG:
		flexcan->ext_msg = !!tmp;
		break;
	case FLEXCAN_ATTR_STANDARD_MSG:
		flexcan->std_msg = !!tmp;
		break;
	}
 set_finish:
	mutex_unlock(&flexcan->mutex);
	return count;
}

static void flexcan_device_default(struct flexcan_device *dev)
{
	dev->br_clksrc = 1;
	dev->br_rjw = 2;
	dev->br_presdiv = 6;
	dev->br_propseg = 4;
	dev->br_pseg1 = 4;
	dev->br_pseg2 = 7;

	dev->bcc = 1;
	dev->srx_dis = 1;
	dev->smp = 1;
	dev->abort = 1;

	dev->maxmb = FLEXCAN_MAX_MB - 1;
	dev->xmit_maxmb = (FLEXCAN_MAX_MB >> 1) - 1;
	dev->xmit_mb = dev->maxmb - dev->xmit_maxmb;

	dev->ext_msg = 1;
	dev->std_msg = 1;
}

static int flexcan_device_attach(struct flexcan_device *flexcan)
{
	int ret;
	struct resource *res;
	struct platform_device *pdev = flexcan->dev;
	struct flexcan_platform_data *plat_data = pdev->dev.platform_data;
	int irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	if (!request_mem_region(res->start, resource_size(res), "flexcan")) {
		return -EBUSY;
	}

	flexcan->irq = irq;
	flexcan->io_base = ioremap_nocache(res->start, resource_size(res));
	if (!flexcan->io_base) {
		ret = -ENOMEM;
		goto release;
	}
	pdev_dbg(pdev, 0, "controller registers %08lx remapped to %p\n",
		 (unsigned long)res->start, flexcan->io_base);

	flexcan->hwmb = flexcan->io_base + CAN_MB_BASE;
	flexcan->rx_mask = flexcan->io_base + CAN_RXMASK_BASE;

	flexcan->clk = clk_get(&pdev->dev, "can");
	if (IS_ERR(flexcan->clk)) {
		ret = PTR_ERR(flexcan->clk);
		dev_err(&pdev->dev, "Failed to get clock: %d\n", ret);
		goto unmap;
	}

	if (plat_data) {
		if (plat_data->active) {
			ret = plat_data->active(pdev);
			if (ret)
				goto put_clk;
		}
		if (plat_data->core_reg) {
			flexcan->core_reg = regulator_get(&pdev->dev,
							  plat_data->core_reg);
			if (IS_ERR(flexcan->core_reg)) {
				ret = PTR_ERR(flexcan->core_reg);
				goto deactivate;
			}
		}

		if (plat_data->io_reg) {
			flexcan->io_reg = regulator_get(&pdev->dev,
							plat_data->io_reg);
			if (IS_ERR(flexcan->core_reg)) {
				ret = PTR_ERR(flexcan->io_reg);
				goto put_reg;
			}
		}
	}
	return 0;

 put_reg:
	regulator_put(flexcan->core_reg);
 deactivate:
	if (plat_data->inactive)
		plat_data->inactive(pdev);
 put_clk:
	clk_put(flexcan->clk);
 unmap:
	iounmap(flexcan->io_base);
 release:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static void flexcan_device_detach(struct flexcan_device *flexcan)
{
	struct platform_device *pdev = flexcan->dev;
	struct flexcan_platform_data *plat_data = pdev->dev.platform_data;
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	BUG_ON(!res);

	clk_put(flexcan->clk);

	if (flexcan->io_reg) {
		regulator_put(flexcan->io_reg);
	}

	if (flexcan->core_reg) {
		regulator_put(flexcan->core_reg);
	}

	if (plat_data && plat_data->inactive)
		plat_data->inactive(pdev);

	iounmap(flexcan->io_base);
	release_mem_region(res->start, resource_size(res));
}

static void flexcan_mbm_isr(struct work_struct *work);
static void flexcan_err_handler(struct work_struct *work);

static struct net_device *flexcan_device_alloc(struct platform_device *pdev,
					       void (*setup)(struct net_device *dev))
{
	struct flexcan_device *flexcan;
	struct net_device *net;
	int i, num;
	int ret;

	net = alloc_netdev(sizeof(*flexcan), "can%d", setup);
	if (net == NULL) {
		pdev_err(pdev, "Failed to allocate netdevice\n");
		return ERR_PTR(-ENOMEM);
	}
	flexcan = netdev_priv(net);

	init_timer(&flexcan->timer);
	mutex_init(&flexcan->mutex);
	INIT_WORK(&flexcan->mb_work, flexcan_mbm_isr);
	INIT_WORK(&flexcan->err_work, flexcan_err_handler);

	flexcan->dev = pdev;
	ret = flexcan_device_attach(flexcan);
	if (ret) {
		free_netdev(net);
		return ERR_PTR(ret);
	}
	flexcan_device_default(flexcan);
	flexcan_update_bitrate(flexcan);

	num = ARRAY_SIZE(flexcan_dev_attr);

	for (i = 0; i < num; i++) {
		ret = device_create_file(&pdev->dev, flexcan_dev_attr + i);
		if (ret) {
			pdev_err(pdev, "Failed to create attribute file %s: %d\n",
				(flexcan_dev_attr + i)->attr.name, ret);
			for (i--; i >= 0; i--)
				device_remove_file(&pdev->dev, flexcan_dev_attr + i);
			flexcan_device_detach(flexcan);
			free_netdev(net);
			return ERR_PTR(ret);
		}
	}
	platform_set_drvdata(pdev, net);
	return net;
}

static void flexcan_device_free(struct net_device *dev)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct platform_device *pdev = flexcan->dev;
	int i;

	ndev_dbg(dev, 0, "%s: Deleting timer\n", __FUNCTION__);
	del_timer_sync(&flexcan->timer);

	ndev_dbg(dev, 0, "%s: Removing sysfs files\n", __FUNCTION__);
	for (i = 0; i < ARRAY_SIZE(flexcan_dev_attr); i++)
		device_remove_file(&pdev->dev, flexcan_dev_attr + i);

	ndev_dbg(dev, 0, "%s: Detaching can device\n", __FUNCTION__);
	flexcan_device_detach(flexcan);
	ndev_dbg(dev, 0, "%s: Freeing net_device\n", __FUNCTION__);
	free_netdev(dev);
}

#define flexcan_swab32(x)				\
	(((x) << 24) | ((x) >> 24) |			\
		(((x) & (__u32)0x0000ff00UL) << 8) |	\
		(((x) & (__u32)0x00ff0000UL) >> 8))

static inline void flexcan_mb_write(struct can_frame *frame, struct can_hw_mb __iomem *hwmb,
				    int code)
{
	int i;
	unsigned long __iomem *s = (unsigned long *)&frame->data[0];
	unsigned long __iomem *d = (unsigned long *)&hwmb->mb_data[0];
	struct can_hw_mb mb;
	unsigned int can_id;
	int n_words = (frame->can_dlc + 3) / sizeof(unsigned int);

	mb.mb_cs.data = 0;
	mb.mb_cs.cs.code = code;
	mb.mb_cs.cs.length = frame->can_dlc;

	mb.mb_cs.cs.rtr = !!(frame->can_id & CAN_RTR_FLAG);

	if (frame->can_id & CAN_EFF_FLAG) {
		mb.mb_cs.cs.ide = 1;
		mb.mb_cs.cs.srr = 1;
		can_id = frame->can_id & CAN_EFF_MASK;
	} else {
		mb.mb_cs.cs.ide = 0;
		can_id = (frame->can_id & CAN_SFF_MASK) << 18;
	}

	DBG(0, "%s: writing can_id %08x to mb_id %p\n", __FUNCTION__, can_id, &hwmb->mb_id);
	__raw_writel(can_id, &hwmb->mb_id);
	for (i = 0; i < n_words; i++, s++, d++) {
		DBG(0, "%s: writing data %08lx to mb_data %p\n", __FUNCTION__,
		    flexcan_swab32(*s), d);
		__raw_writel(flexcan_swab32(*s), d);
	}
	DBG(0, "%s: Writing CS %08x to mb_cs %p\n", __FUNCTION__, mb.mb_cs.data, &hwmb->mb_cs);
	__raw_writel(mb.mb_cs.data, &hwmb->mb_cs.data);
}

static inline void flexcan_mb_read(struct can_frame *frame, struct can_hw_mb __iomem *hwmb)
{
	int i;
	unsigned long __iomem *s = (unsigned long *)&hwmb->mb_data[0];
	unsigned long __iomem *d = (unsigned long *)&frame->data[0];
	struct can_hw_mb mb;
	unsigned int can_id;
	int n_words;

	mb.mb_cs.data = __raw_readl(&hwmb->mb_cs);
	BUG_ON(mb.mb_cs.cs.code & CAN_MB_RX_BUSY);

	can_id = __raw_readl(&hwmb->mb_id);

	if (mb.mb_cs.cs.ide)
		frame->can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		frame->can_id = (can_id >> 18) & CAN_SFF_MASK;
	if (mb.mb_cs.cs.rtr)
		frame->can_id |= CAN_RTR_FLAG;

	frame->can_dlc = mb.mb_cs.cs.length;
	if (frame->can_dlc == 0 || frame->can_dlc > 8)
		return;

	n_words = (frame->can_dlc + 3) / sizeof(unsigned int);
	for (i = 0; i < n_words; i++, s++, d++)
		*d = flexcan_swab32(__raw_readl(s));
}

static inline void flexcan_memcpy(void *dst, void *src, int len)
{
	int i;
	unsigned int __iomem *d = dst, *s = src;

	DBG(2, "%s: Copying %u byte from %p to %p\n", __FUNCTION__, len, s, d);
	WARN_ON(len & 3);
	len = (len + 3) >> 2;
	for (i = 0; i < len; i++, s++, d++)
		__raw_writel(flexcan_swab32(*s), d);
	if (dbg_lvl(1)) {
		print_hex_dump(KERN_DEBUG, "swdat: ", DUMP_PREFIX_OFFSET, 16, 4,
			       src, len << 2, 0);
		print_hex_dump(KERN_DEBUG, "hwdat: ", DUMP_PREFIX_OFFSET, 16, 4,
			       dst, len << 2, 0);
	}
}

static inline struct can_frame *flexcan_skb_put(struct sk_buff *skb, unsigned int len)
{
	return (struct can_frame *)skb_put(skb, len);
}

static inline struct can_frame *flexcan_skb_data(struct sk_buff *skb)
{
	BUG_ON(skb == NULL);
	return (struct can_frame *)skb->data;
}

static struct net_device_stats *flexcan_get_stats(struct net_device *dev)
{
	ndev_dbg(dev, 3, "%s@%d: \n", __FUNCTION__, __LINE__);
	if (!netif_running(dev))
		return &dev->stats;
	ndev_dbg(dev, 3, "%s@%d: \n", __FUNCTION__, __LINE__);
	return &dev->stats;
}

static void flexcan_mb_bottom(struct net_device *dev, int index)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct net_device_stats *stats = flexcan_get_stats(dev);
	struct can_hw_mb __iomem *hwmb;
	struct can_frame *frame;
	struct sk_buff *skb;
	struct can_hw_mb mb;

	ndev_dbg(dev, 1, "%s: index: %d\n", __FUNCTION__, index);

	hwmb = flexcan->hwmb + index;
	mb.mb_cs.data = __raw_readl(&hwmb->mb_cs.data);
	if (flexcan->fifo ||
	    index >= flexcan->maxmb - flexcan->xmit_maxmb) {
		/* handle transmit MBs */

		if (mb.mb_cs.cs.code == CAN_MB_TX_ABORT) {
			mb.mb_cs.cs.code = CAN_MB_TX_INACTIVE;
			__raw_writel(mb.mb_cs.data, &hwmb->mb_cs.data);
		}
		if (mb.mb_cs.cs.code & CAN_MB_TX_INACTIVE) {
			if (flexcan->xmit_buffers++ == 0) {
				ndev_dbg(dev, 1, "%s: Starting netif queue\n", __FUNCTION__);
				netif_start_queue(dev);
			}
			BUG_ON(flexcan->xmit_buffers > flexcan->maxmb - flexcan->xmit_maxmb);
			return;
		}
		/* if fifo is enabled all RX MBs should be handled in the fifo_isr */
		BUG();
	}
	if (dbg_lvl(1))
		print_hex_dump(KERN_DEBUG, "rx_mb: ", DUMP_PREFIX_OFFSET, 16, 4,
			       hwmb, sizeof(*hwmb), 0);
	/* handle RX MB in case fifo is not used */
	BUG_ON(flexcan->fifo);
	if (mb.mb_cs.cs.code & CAN_MB_RX_BUSY) {
		ndev_dbg(dev, -1, "%s: MB[%02x] is busy: %x\n", __FUNCTION__,
			 index, mb.mb_cs.cs.code);
		/* unlock buffer */
		(void)flexcan_reg_read(flexcan, CAN_HW_REG_TIMER);
		return;
	}

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (skb) {
		frame = flexcan_skb_put(skb, sizeof(*frame));
		flexcan_mb_read(frame, hwmb);
		/* unlock buffer */
		(void)flexcan_reg_read(flexcan, CAN_HW_REG_TIMER);

		dev->last_rx = jiffies;
		stats->rx_packets++;
		stats->rx_bytes += frame->can_dlc;

		skb->dev = dev;
		skb->protocol = __constant_htons(ETH_P_CAN);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		netif_receive_skb(skb);
	} else {
		flexcan_dbg(flexcan, 0, "%s: Could not allocate SKB; dropping packet\n",
			    __FUNCTION__);

		stats->rx_dropped++;
	}
}

static void flexcan_fifo_isr(struct net_device *dev, unsigned int iflag1)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct net_device_stats *stats = flexcan_get_stats(dev);
	struct sk_buff *skb;
	struct can_hw_mb __iomem *hwmb = flexcan->hwmb;
	struct can_frame *frame;

	ndev_dbg(dev, 2, "%s: \n", __FUNCTION__);
	if (dbg_lvl(1))
		print_hex_dump(KERN_DEBUG, "rx_mb: ", DUMP_PREFIX_OFFSET, 16, 4,
			       hwmb, sizeof(*hwmb), 0);
	if (iflag1 & __FIFO_RDY_INT) {
		skb = dev_alloc_skb(sizeof(struct can_frame));
		if (skb) {
			frame = flexcan_skb_put(skb, sizeof(*frame));
			flexcan_mb_read(frame, hwmb);
			/* unlock mb */
			(void) flexcan_reg_read(flexcan, CAN_HW_REG_TIMER);

			dev->last_rx = jiffies;

			stats->rx_packets++;
			stats->rx_bytes += frame->can_dlc;

			skb->dev = dev;
			skb->protocol = __constant_htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			netif_receive_skb(skb);
		} else {
			(void)__raw_readl(&hwmb->mb_cs.data);
			/* unlock mb */
			(void) flexcan_reg_read(flexcan, CAN_HW_REG_TIMER);
		}
	}

	if (iflag1 & (__FIFO_OV_INT | __FIFO_WARN_INT)) {
		skb = dev_alloc_skb(sizeof(struct can_frame));
		if (skb) {
			frame = flexcan_skb_put(skb, sizeof(*frame));
			frame->can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
			frame->can_dlc = CAN_ERR_DLC;
			if (iflag1 & __FIFO_WARN_INT)
				frame->data[1] |=
				    CAN_ERR_CRTL_TX_WARNING |
				    CAN_ERR_CRTL_RX_WARNING;
			if (iflag1 & __FIFO_OV_INT)
				frame->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
			if (dbg_lvl(1))
				print_hex_dump(KERN_DEBUG, "err_mb: ", DUMP_PREFIX_OFFSET, 16, 4,
					       frame, sizeof(*frame), 0);

			skb->dev = dev;
			skb->protocol = __constant_htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			netif_receive_skb(skb);
		}
	}
}

/*
 * called by CAN ISR to handle mb events.
 */
static void flexcan_mbm_isr(struct work_struct *work)
{
	struct flexcan_device *flexcan = container_of(work, struct flexcan_device, mb_work);
	struct net_device *dev = platform_get_drvdata(flexcan->dev);
	int i, iflag1, iflag2, maxmb;

	i = 0;

	ndev_dbg(dev, 2, "%s: \n", __FUNCTION__);

	iflag1 = flexcan_reg_read(flexcan, CAN_HW_REG_IFLAG1) &
		flexcan_reg_read(flexcan, CAN_HW_REG_IMASK1);
	if (flexcan->maxmb > 31) {
		maxmb = flexcan->maxmb + 1 - 32;
		iflag2 = flexcan_reg_read(flexcan, CAN_HW_REG_IFLAG2) &
		    flexcan_reg_read(flexcan, CAN_HW_REG_IMASK2);
		iflag2 &= (1 << maxmb) - 1;
		maxmb = 32;
	} else {
		maxmb = flexcan->maxmb + 1;
		iflag1 &= (1 << maxmb) - 1;
		iflag2 = 0;
	}

	ndev_dbg(dev, 2, "%s: loop=%d iflag1=%08x\n", __FUNCTION__, i, iflag1);

	flexcan_reg_write(flexcan, CAN_HW_REG_IFLAG1, iflag1);
	flexcan_reg_write(flexcan, CAN_HW_REG_IFLAG2, iflag2);

	if (flexcan->fifo) {
		flexcan_fifo_isr(dev, iflag1);
		iflag1 &= ~0xFF;
	}
	for (i = 0; iflag1 && (i < maxmb); i++) {
		if (iflag1 & (1 << i)) {
			iflag1 &= ~(1 << i);
			flexcan_mb_bottom(dev, i);
		}
	}

	for (i = maxmb; iflag2 && (i <= flexcan->maxmb); i++) {
		if (iflag2 & (1 << (i - 32))) {
			iflag2 &= ~(1 << (i - 32));
			flexcan_mb_bottom(dev, i);
		}
	}
	enable_irq(flexcan->irq);
}

static int flexcan_mbm_xmit(struct flexcan_device *flexcan, struct can_frame *frame)
{
	int i = flexcan->xmit_mb;
	struct can_hw_mb __iomem *hwmb = flexcan->hwmb;
	static int last;

	if (flexcan->xmit_buffers != last) {
		flexcan_dbg(flexcan, 1, "%s: %d free buffers\n", __FUNCTION__,
			    flexcan->xmit_buffers);
		last = flexcan->xmit_buffers;
	}
	do {
		struct can_hw_mb mb;

		mb.mb_cs.data = __raw_readl(&hwmb[i].mb_cs);

		if (mb.mb_cs.cs.code == CAN_MB_TX_INACTIVE)
			break;
		if (++i > flexcan->maxmb) {
			if (flexcan->fifo)
				i = FLEXCAN_MAX_FIFO_MB;
			else
				i = flexcan->xmit_maxmb + 1;
		}
		if (i == flexcan->xmit_mb) {
			flexcan_dbg(flexcan, 0, "%s: no free xmit buffer\n", __FUNCTION__);
			return 0;
		}
	} while (1);

	flexcan->xmit_mb = i + 1;
	if (flexcan->xmit_mb > flexcan->maxmb) {
		if (flexcan->fifo)
			flexcan->xmit_mb = FLEXCAN_MAX_FIFO_MB;
		else
			flexcan->xmit_mb = flexcan->xmit_maxmb + 1;
	}

	flexcan_dbg(flexcan, 1, "%s: Enabling transmission of buffer %d\n", __FUNCTION__, i);
	flexcan_mb_write(frame, &hwmb[i], CAN_MB_TX_ONCE);

	if (dbg_lvl(1))
		print_hex_dump(KERN_DEBUG, "tx_mb: ", DUMP_PREFIX_OFFSET, 16, 4,
			       &hwmb[i], sizeof(*hwmb), 0);
	return 1;
}

static void flexcan_mbm_init(struct flexcan_device *flexcan)
{
	struct can_hw_mb __iomem *hwmb;
	int rx_mb, i;

	flexcan_dbg(flexcan, 0, "%s@%d: \n", __FUNCTION__, __LINE__);

	/* Set global mask to receive all messages */
	flexcan_reg_write(flexcan, CAN_HW_REG_RXGMASK, 0);
	flexcan_reg_write(flexcan, CAN_HW_REG_RX14MASK, 0);
	flexcan_reg_write(flexcan, CAN_HW_REG_RX15MASK, 0);

	for (i = 0; i < FLEXCAN_MAX_MB; i++) {
		int j;
		void __iomem *mb = &flexcan->hwmb[i];
		void __iomem *rxm = &flexcan->rx_mask[i];

		__raw_writel(0, rxm);
		for (j = 0; j < sizeof(*flexcan->hwmb); j += 4) {
			__raw_writel(0, mb + j);
		}
	}

	if (flexcan->fifo)
		rx_mb = FLEXCAN_MAX_FIFO_MB;
	else
		rx_mb = flexcan->maxmb - flexcan->xmit_maxmb;

	hwmb = flexcan->hwmb;
	if (flexcan->fifo) {
		unsigned long *id_table = flexcan->io_base + CAN_FIFO_BASE;
		for (i = 0; i < rx_mb; i++)
			__raw_writel(0, &id_table[i]);
	} else {
		for (i = 0; i < rx_mb; i++) {
			struct can_hw_mb mb;

			mb.mb_cs.data = 0;
			mb.mb_cs.cs.code = CAN_MB_RX_EMPTY;
			if (flexcan->ext_msg && flexcan->std_msg)
				mb.mb_cs.cs.ide = i & 1;
			else if (flexcan->ext_msg)
				mb.mb_cs.cs.ide = 1;

			__raw_writel(mb.mb_cs.data, &hwmb[i]);
		}
	}

	for (; i <= flexcan->maxmb; i++) {
		struct can_hw_mb mb;

		mb.mb_cs.data = 0;
		mb.mb_cs.cs.code = CAN_MB_TX_INACTIVE;
		__raw_writel(mb.mb_cs.data, &hwmb[i]);
	}

	flexcan->xmit_mb = rx_mb;
	flexcan->xmit_buffers = flexcan->maxmb - flexcan->xmit_maxmb;
}

static void flexcan_hw_start(struct flexcan_device *flexcan)
{
	unsigned int reg;

	flexcan_dbg(flexcan, 0, "%s: \n", __FUNCTION__);

	if ((flexcan->maxmb + 1) > 32) {
		flexcan_reg_write(flexcan, CAN_HW_REG_IMASK1, ~0);
		reg = (1 << (flexcan->maxmb - 31)) - 1;
		flexcan_reg_write(flexcan, CAN_HW_REG_IMASK2, reg);
	} else {
		reg = (1 << (flexcan->maxmb + 1)) - 1;
		flexcan_reg_write(flexcan, CAN_HW_REG_IMASK1, reg);
		flexcan_reg_write(flexcan, CAN_HW_REG_IMASK2, 0);
	}

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg & ~__MCR_HALT);
}

static void flexcan_hw_stop(struct flexcan_device *flexcan)
{
	unsigned int reg;

	flexcan_dbg(flexcan, 0, "%s: \n", __FUNCTION__);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg | __MCR_HALT);
}

static int flexcan_hw_reset(struct flexcan_device *flexcan)
{
	struct platform_device *pdev __attribute__((unused)) = flexcan->dev;
	unsigned int reg;
	int timeout = 100000;

	flexcan_dbg(flexcan, 0, "%s: \n", __FUNCTION__);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg | __MCR_MDIS);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_CTRL);
	if (flexcan->br_clksrc)
		reg |= __CTRL_CLK_SRC;
	else
		reg &= ~__CTRL_CLK_SRC;
	flexcan_reg_write(flexcan, CAN_HW_REG_CTRL, reg);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR) & ~__MCR_MDIS;
	flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg);
	reg |= __MCR_SOFT_RST;
	flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	while (reg & __MCR_SOFT_RST) {
		if (--timeout <= 0) {
			dev_err(&pdev->dev, "Flexcan software Reset Timeout\n");
			return -ETIME;
		}
		udelay(10);
		reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	}
	return 0;
}

static void flexcan_mcr_setup(struct flexcan_device *flexcan)
{
	unsigned int reg;

	flexcan_dbg(flexcan, 0, "%s: \n", __FUNCTION__);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	reg &= ~(__MCR_MAX_MB_MASK | __MCR_WAK_MSK | __MCR_MAX_IDAM_MASK);

	if (flexcan->fifo)
		reg |= __MCR_FEN;
	else
		reg &= ~__MCR_FEN;

	if (flexcan->wakeup)
		reg |= __MCR_SLF_WAK | __MCR_WAK_MSK;
	else
		reg &= ~(__MCR_SLF_WAK | __MCR_WAK_MSK);

	if (flexcan->wak_src)
		reg |= __MCR_WAK_SRC;
	else
		reg &= ~__MCR_WAK_SRC;

	if (flexcan->srx_dis)
		reg |= __MCR_SRX_DIS;
	else
		reg &= ~__MCR_SRX_DIS;

	if (flexcan->bcc)
		reg |= __MCR_BCC;
	else
		reg &= ~__MCR_BCC;

	if (flexcan->lprio)
		reg |= __MCR_LPRIO_EN;
	else
		reg &= ~__MCR_LPRIO_EN;

	if (flexcan->abort)
		reg |= __MCR_AEN;
	else
		reg &= ~__MCR_AEN;

	reg |= (flexcan->maxmb << __MCR_MAX_MB_OFFSET);
	reg |= __MCR_DOZE | __MCR_MAX_IDAM_C;
	flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg);
}

static void flexcan_ctrl_setup(struct flexcan_device *flexcan)
{
	unsigned int reg;

	flexcan_dbg(flexcan, 0, "%s: \n", __FUNCTION__);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_CTRL);
	reg &= ~(__CTRL_PRESDIV_MASK | __CTRL_RJW_MASK | __CTRL_PSEG1_MASK |
		 __CTRL_PSEG2_MASK | __CTRL_PROPSEG_MASK);

	if (flexcan->loopback)
		reg |= __CTRL_LPB;
	else
		reg &= ~__CTRL_LPB;

	if (flexcan->smp)
		reg |= __CTRL_SMP;
	else
		reg &= ~__CTRL_SMP;

	if (flexcan->boff_rec)
		reg |= __CTRL_BOFF_REC;
	else
		reg &= ~__CTRL_BOFF_REC;

	if (flexcan->tsyn)
		reg |= __CTRL_TSYN;
	else
		reg &= ~__CTRL_TSYN;

	if (flexcan->listen)
		reg |= __CTRL_LOM;
	else
		reg &= ~__CTRL_LOM;

	reg |= (flexcan->br_presdiv << __CTRL_PRESDIV_OFFSET) |
	    (flexcan->br_rjw << __CTRL_RJW_OFFSET) |
	    (flexcan->br_pseg1 << __CTRL_PSEG1_OFFSET) |
	    (flexcan->br_pseg2 << __CTRL_PSEG2_OFFSET) |
	    (flexcan->br_propseg << __CTRL_PROPSEG_OFFSET);

	reg &= ~__CTRL_LBUF;

	reg |= __CTRL_TWRN_MSK | __CTRL_RWRN_MSK | __CTRL_BOFF_MSK |
	    __CTRL_ERR_MSK;

	flexcan_reg_write(flexcan, CAN_HW_REG_CTRL, reg);
}

static int flexcan_hw_restart(struct net_device *dev)
{
	unsigned int reg;
	struct flexcan_device *flexcan = netdev_priv(dev);

	ndev_dbg(dev, 0, "%s: \n", __FUNCTION__);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	if (reg & __MCR_SOFT_RST)
		return 1;

	flexcan_mcr_setup(flexcan);

	flexcan_reg_write(flexcan, CAN_HW_REG_IMASK2, 0);
	flexcan_reg_write(flexcan, CAN_HW_REG_IMASK1, 0);

	flexcan_reg_write(flexcan, CAN_HW_REG_IFLAG2, 0xFFFFFFFF);
	flexcan_reg_write(flexcan, CAN_HW_REG_IFLAG1, 0xFFFFFFFF);

	flexcan_reg_write(flexcan, CAN_HW_REG_ECR, 0);

	flexcan_mbm_init(flexcan);
	netif_carrier_on(dev);
	flexcan_hw_start(flexcan);

	if (netif_queue_stopped(dev)) {
		ndev_dbg(dev, 1, "%s@%d: Starting netif queue\n",
			 __FUNCTION__, __LINE__);
		netif_start_queue(dev);
	}
	return 0;
}

static void flexcan_hw_watch(unsigned long data)
{
	unsigned int reg, ecr;
	struct net_device *dev = (struct net_device *)data;
	struct flexcan_device *flexcan = netdev_priv(dev);

	ndev_dbg(dev, 1, "%s: \n", __FUNCTION__);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	if (reg & __MCR_MDIS) {
		if (flexcan_hw_restart(dev))
			mod_timer(&flexcan->timer, HZ / 20);
		return;
	}
	ecr = flexcan_reg_read(flexcan, CAN_HW_REG_ECR);
	if (flexcan->boff_rec) {
		if (((reg & __ESR_FLT_CONF_MASK) >> __ESR_FLT_CONF_OFF) > 1) {
			reg |= __MCR_SOFT_RST;
			ndev_dbg(dev, 1, "%s: Initiating soft reset\n", __FUNCTION__);
			flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg);
			mod_timer(&flexcan->timer, HZ / 20);
			return;
		}
		netif_carrier_on(dev);
	}
	ndev_dbg(dev, 1, "%s: Done\n", __FUNCTION__);
}

static void flexcan_hw_busoff(struct net_device *dev)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	unsigned int reg;

	netif_carrier_off(dev);

	flexcan->timer.function = flexcan_hw_watch;
	flexcan->timer.data = (unsigned long)dev;

	if (flexcan->boff_rec) {
		mod_timer(&flexcan->timer, HZ / 10);
		return;
	}
	reg = flexcan_reg_read(flexcan, CAN_HW_REG_MCR);
	flexcan_reg_write(flexcan, CAN_HW_REG_MCR, reg | __MCR_SOFT_RST);
	mod_timer(&flexcan->timer, HZ / 20);
}

static int flexcan_hw_open(struct flexcan_device *flexcan)
{
	int ret;

	if ((ret = flexcan_hw_reset(flexcan)) != 0)
		return ret;

	flexcan_mcr_setup(flexcan);
	flexcan_ctrl_setup(flexcan);

	flexcan_reg_write(flexcan, CAN_HW_REG_IMASK2, 0);
	flexcan_reg_write(flexcan, CAN_HW_REG_IMASK1, 0);

	flexcan_reg_write(flexcan, CAN_HW_REG_IFLAG2, 0xFFFFFFFF);
	flexcan_reg_write(flexcan, CAN_HW_REG_IFLAG1, 0xFFFFFFFF);

	flexcan_reg_write(flexcan, CAN_HW_REG_ECR, 0);
	return 0;
}

static void flexcan_err_handler(struct work_struct *work)
{
	struct flexcan_device *flexcan = container_of(work, struct flexcan_device, err_work);
	struct net_device *dev = platform_get_drvdata(flexcan->dev);
	struct sk_buff *skb;
	struct can_frame *frame;
	unsigned int esr, ecr;

	ndev_dbg(dev, 1, "%s@%d: \n", __FUNCTION__, __LINE__);

	esr = flexcan_reg_read(flexcan, CAN_HW_REG_ESR);
	flexcan_reg_write(flexcan, CAN_HW_REG_ESR, esr & __ESR_INTERRUPTS);
	enable_irq(flexcan->irq);

	if (esr & __ESR_WAK_INT) {
		ndev_dbg(dev, 0, "%s@%d: \n", __FUNCTION__, __LINE__);
		return;
	}

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (!skb) {
		ndev_err(dev, "%s: Failed to allocate skb\n", __func__);
		return;
	}
	frame = flexcan_skb_put(skb, sizeof(*frame));
	frame->can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
	frame->can_dlc = CAN_ERR_DLC;

	if (esr & __ESR_TWRN_INT) {
		ndev_err(dev, "%s: TX_WARNING\n", __FUNCTION__);
		frame->data[1] |= CAN_ERR_CRTL_TX_WARNING;
	}
	if (esr & __ESR_RWRN_INT) {
		ndev_err(dev, "%s: RX_WARNING\n", __FUNCTION__);
		frame->data[1] |= CAN_ERR_CRTL_RX_WARNING;
	}
	if (esr & __ESR_BOFF_INT) {
		ndev_err(dev, "%s: BUS_OFF\n", __FUNCTION__);
		frame->can_id |= CAN_ERR_BUSOFF;
	}
	if (esr & __ESR_ERR_INT) {
		ndev_dbg(dev, 1, "%s@%d: \n", __FUNCTION__, __LINE__);
		if (esr & __ESR_BIT1_ERR) {
			ndev_err(dev, "%s: BIT1_ERR\n", __FUNCTION__);
			frame->data[2] |= CAN_ERR_PROT_BIT1;
		}

		if (esr & __ESR_BIT0_ERR) {
			ndev_err(dev, "%s: BIT0_ERR\n", __FUNCTION__);
			frame->data[2] |= CAN_ERR_PROT_BIT0;
		}

		if (esr & __ESR_ACK_ERR) {
			ndev_err(dev, "%s: ACK_ERR\n", __FUNCTION__);
			frame->can_id |= CAN_ERR_ACK;
		}

		/*TODO:// if (esr & __ESR_CRC_ERR) */

		if (esr & __ESR_FRM_ERR) {
			ndev_err(dev, "%s: FRM_ERR\n", __FUNCTION__);
			frame->data[2] |= CAN_ERR_PROT_FORM;
		}

		if (esr & __ESR_STF_ERR) {
			ndev_err(dev, "%s: STF_ERR\n", __FUNCTION__);
			frame->data[2] |= CAN_ERR_PROT_STUFF;
		}

		ecr = flexcan_reg_read(flexcan, CAN_HW_REG_ECR);
		switch ((esr & __ESR_FLT_CONF_MASK) >> __ESR_FLT_CONF_OFF) {
		case 0:
			ndev_dbg(dev, 0, "%s@%d: \n", __FUNCTION__, __LINE__);
			if (__ECR_TX_ERR_COUNTER(ecr) >= __ECR_ACTIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_TX_WARNING;
			if (__ECR_RX_ERR_COUNTER(ecr) >= __ECR_ACTIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_RX_WARNING;
			break;
		case 1:
			ndev_dbg(dev, 0, "%s@%d: \n", __FUNCTION__, __LINE__);
			if (__ECR_TX_ERR_COUNTER(ecr) >=
			    __ECR_PASSIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;

			if (__ECR_RX_ERR_COUNTER(ecr) >=
			    __ECR_PASSIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
			break;
		default:
			ndev_dbg(dev, 0, "%s@%d: \n", __FUNCTION__, __LINE__);
			frame->can_id |= CAN_ERR_BUSOFF;
		}
	}

	if (frame->can_id & CAN_ERR_BUSOFF) {
		ndev_dbg(dev, 0, "%s: switchung bus off\n", __FUNCTION__);
		flexcan_hw_busoff(dev);
	}
	skb->dev = dev;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	netif_receive_skb(skb);
}

static irqreturn_t flexcan_irq_handler(int irq, void *data)
{
	struct net_device *dev = data;
	struct flexcan_device *flexcan = netdev_priv(dev);
	unsigned int reg;

	ndev_dbg(dev, 1, "%s: \n", __FUNCTION__);
	disable_irq_nosync(irq);

	reg = flexcan_reg_read(flexcan, CAN_HW_REG_ESR);
	if (reg & __ESR_INTERRUPTS) {
		ndev_dbg(dev, 1, "%s: Scheduling err handler\n", __FUNCTION__);
		schedule_work(&flexcan->err_work);
		return IRQ_HANDLED;
	}

	ndev_dbg(dev, 1, "%s: Scheduling mbm handler\n", __FUNCTION__);
	schedule_work(&flexcan->mb_work);
	return IRQ_HANDLED;
}

static int flexcan_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct can_frame *frame = flexcan_skb_data(skb);
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct net_device_stats *stats = flexcan_get_stats(dev);

	ndev_dbg(dev, 1, "%s: \n", __FUNCTION__);

	if (frame->can_dlc > 8)
		return -EINVAL;

	if (flexcan_mbm_xmit(flexcan, frame)) {
		dev_kfree_skb(skb);
		stats->tx_bytes += frame->can_dlc;
		stats->tx_packets++;
		dev->trans_start = jiffies;
		if (--flexcan->xmit_buffers == 0) {
			ndev_dbg(dev, 1, "%s: Stopping netif queue\n", __FUNCTION__);
			netif_stop_queue(dev);
		}
		BUG_ON(flexcan->xmit_buffers < 0);
		return NETDEV_TX_OK;
	}
	ndev_dbg(dev, 1, "%s: could not transmit message\n", __FUNCTION__);
	return NETDEV_TX_BUSY;
}

static int flexcan_open(struct net_device *dev)
{
	int ret;
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct platform_device *pdev = flexcan->dev;
	struct flexcan_platform_data *plat_data = pdev->dev.platform_data;

	ndev_dbg(dev, 0, "%s: \n", __FUNCTION__);

	ret = clk_enable(flexcan->clk);
	if (ret)
		goto clk_err;

	if (flexcan->core_reg) {
		ret = regulator_enable(flexcan->core_reg);
		if (ret)
			goto core_reg_err;
	}

	if (flexcan->io_reg) {
		ret = regulator_enable(flexcan->io_reg);
		if (ret)
			goto io_reg_err;
	}

	if (plat_data && plat_data->xcvr_enable) {
		ret = plat_data->xcvr_enable(pdev, 1);
		if (ret)
			goto enable_err;
	}

	ret = request_irq(flexcan->irq, flexcan_irq_handler, IRQF_SAMPLE_RANDOM,
			  dev->name, dev);
	if (ret)
		goto irq_err;

	ret = flexcan_hw_open(flexcan);
	if (ret)
		goto open_err;

	flexcan_mbm_init(flexcan);
	netif_carrier_on(dev);
	flexcan_hw_start(flexcan);
	return 0;

 open_err:
	free_irq(flexcan->irq, dev);
 irq_err:
	if (plat_data && plat_data->xcvr_enable)
		plat_data->xcvr_enable(pdev, 0);
 enable_err:
	if (flexcan->io_reg)
		regulator_disable(flexcan->io_reg);
 io_reg_err:
	if (flexcan->core_reg)
		regulator_disable(flexcan->core_reg);
 core_reg_err:
	if (flexcan->clk)
		clk_disable(flexcan->clk);
 clk_err:
	return ret;
}

static int flexcan_stop(struct net_device *dev)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct platform_device *pdev = flexcan->dev;
	struct flexcan_platform_data *plat_data = pdev->dev.platform_data;

	flexcan_hw_stop(flexcan);

	free_irq(flexcan->irq, dev);

	if (plat_data && plat_data->xcvr_enable)
		plat_data->xcvr_enable(pdev, 0);

	if (flexcan->io_reg)
		regulator_disable(flexcan->io_reg);
	if (flexcan->core_reg)
		regulator_disable(flexcan->core_reg);
	clk_disable(flexcan->clk);
	return 0;
}

static const struct net_device_ops flexcan_netdev_ops = {
	.ndo_open	= flexcan_open,
	.ndo_stop	= flexcan_stop,
	.ndo_start_xmit = flexcan_start_xmit,
	.ndo_get_stats	= flexcan_get_stats,
};

static void flexcan_setup(struct net_device *dev)
{
	dev->type = ARPHRD_CAN;
	dev->mtu = sizeof(struct can_frame);
	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->tx_queue_len = FLEXCAN_MAX_MB;
	dev->flags = IFF_NOARP;
	dev->features = NETIF_F_NO_CSUM;

	dev->netdev_ops = &flexcan_netdev_ops;
	dev->destructor = flexcan_device_free;
}

static int flexcan_probe(struct platform_device *pdev)
{
	int ret;
	struct net_device *net;

	net = flexcan_device_alloc(pdev, flexcan_setup);
	if (IS_ERR(net))
		return PTR_ERR(net);

	ret = register_netdev(net);
	if (ret) {
		flexcan_device_free(net);
	}
	return ret;
}

static int flexcan_remove(struct platform_device *pdev)
{
	struct net_device *net = platform_get_drvdata(pdev);

	unregister_netdev(net);
	return 0;
}

#ifdef CONFIG_PM
static int flexcan_suspend(struct device *dev)
{
	struct net_device *net = dev_get_drvdata(dev);
	struct flexcan_device *flexcan = netdev_priv(net);

	if (!(net->flags & IFF_UP))
		return 0;

	if (flexcan->wakeup)
		set_irq_wake(flexcan->irq, 1);
	else {
		int ret;
		struct flexcan_platform_data *plat_data;
		struct platform_device *pdev = to_platform_device(dev);

		plat_data = dev->platform_data;

		if (plat_data && plat_data->xcvr_enable) {
			ret = plat_data->xcvr_enable(pdev, 0);
			if (ret)
				return ret;
		}
		if (flexcan->io_reg) {
			ret = regulator_disable(flexcan->io_reg);
			if (ret)
				return ret;
		}
		if (flexcan->core_reg) {
			ret = regulator_disable(flexcan->core_reg);
			if (ret)
				return ret;
		}
		clk_disable(flexcan->clk);
		if (plat_data && plat_data->inactive) {
			plat_data->inactive(pdev);
		}
	}
	return 0;
}

static int flexcan_resume(struct device *dev)
{
	int ret;
	struct net_device *net = dev_get_drvdata(dev);
	struct flexcan_device *flexcan = netdev_priv(net);

	if (!(net->flags & IFF_UP))
		return 0;

	if (flexcan->wakeup)
		set_irq_wake(flexcan->irq, 0);
	else {
		struct flexcan_platform_data *plat_data;
		struct platform_device *pdev = to_platform_device(dev);

		plat_data = dev->platform_data;

		if (plat_data && plat_data->active) {
			ret = plat_data->active(pdev);
			if (ret)
				printk(KERN_ERR "%s: Failed activate hardware: %d\n",
				       __func__, ret);
		}
		ret = clk_enable(flexcan->clk);
		if (ret)
			printk(KERN_ERR "%s: Failed to enable clock: %d\n",
			       __func__, ret);

		if (flexcan->core_reg) {
			ret = regulator_enable(flexcan->core_reg);
			if (ret)
				printk(KERN_ERR "%s: Failed to enable core voltage: %d\n",
				       __func__, ret);
		}
		if (flexcan->io_reg) {
			ret = regulator_enable(flexcan->io_reg);
			if (ret)
				printk(KERN_ERR "%s: Failed to enable io voltage: %d\n",
				       __func__, ret);
		}

		if (plat_data && plat_data->xcvr_enable) {
			ret = plat_data->xcvr_enable(pdev, 1);
			if (ret)
				printk(KERN_ERR "%s: Failed to enable transceiver: %d\n",
				       __func__, ret);
		}
	}
	return 0;
}

static struct dev_pm_ops flexcan_pm_ops = {
	.suspend = flexcan_suspend,
	.resume = flexcan_resume,
};
#endif

static struct platform_driver flexcan_driver = {
	.driver = {
		.name = "mxc-flexcan",
		.pm = __dev_pm_ops_p(flexcan_pm_ops),
	},
	.probe = flexcan_probe,
	.remove = flexcan_remove,
};

static __init int flexcan_init(void)
{
	pr_info("Freescale FlexCAN Driver\n");
	return platform_driver_register(&flexcan_driver);
}

static __exit void flexcan_exit(void)
{
	return platform_driver_unregister(&flexcan_driver);
}

module_init(flexcan_init);
module_exit(flexcan_exit);

MODULE_LICENSE("GPL");
