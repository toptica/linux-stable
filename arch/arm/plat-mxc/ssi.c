/*
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

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <mach/hardware.h>

#include <mach/dma.h>
#include <mach/ssi_port.h>

#ifdef DEBUG
static int debug = 1;
#define dbg_lvl(n)	((n) < debug)
module_param(debug, int, S_IRUGO | S_IWUSR);
#else
#define dbg_lvl(n)	0
static int debug;
module_param(debug, int, 0);
#endif

#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)

static DEFINE_MUTEX(mxc_ssi_lock);

static struct resource mxc_ssi_resources[][4] = {
	{
		{
			.start	= SSI1_BASE_ADDR,
			.end	= SSI1_BASE_ADDR + 0x5b,
			.flags	= IORESOURCE_MEM,
		},
		{
			.start	= MXC_INT_SSI1,
			.end	= MXC_INT_SSI1,
			.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
		},
		{
			.start	= DMA_REQ_SSI1_TX0,
			.end	= DMA_REQ_SSI1_TX0,
			.flags	= IORESOURCE_DMA,
		},
		{
			.start	= DMA_REQ_SSI1_RX0,
			.end	= DMA_REQ_SSI1_RX0,
			.flags	= IORESOURCE_DMA,
		},
	},
	{
		{
			.start	= SSI2_BASE_ADDR,
			.end	= SSI2_BASE_ADDR + 0x5b,
			.flags	= IORESOURCE_MEM,
		},
		{
			.start	= MXC_INT_SSI2,
			.end	= MXC_INT_SSI2,
			.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
		},
		{
			.start	= DMA_REQ_SSI2_TX0,
			.end	= DMA_REQ_SSI2_TX0,
			.flags	= IORESOURCE_DMA,
		},
		{
			.start	= DMA_REQ_SSI2_RX0,
			.end	= DMA_REQ_SSI2_RX0,
			.flags	= IORESOURCE_DMA,
		},
	},
};
#define NUM_SSI_PORTS	ARRAY_SIZE(mxc_ssi_ports)

static struct mxc_ssi_port mxc_ssi_ports[] = {
	{
		.num	= 0,
		.owner	= THIS_MODULE,
		.res	= mxc_ssi_resources[0],
	},
	{
		.num	= 1,
		.owner	= THIS_MODULE,
		.res	= mxc_ssi_resources[1],
	},
};

static int _mxc_ssi_init_port(int index, struct platform_device *parent,
			struct mxc_ssi_port **ssi_port)
{
	int ret = -EBUSY;
	struct mxc_ssi_port *port = &mxc_ssi_ports[index];
	struct platform_device *pdev;
	struct resource *res;

	BUG_ON(index < 0 || index >= NUM_SSI_PORTS);

	pdev = platform_device_register_simple("mxc-ssi", index,
					port->res,
					ARRAY_SIZE(mxc_ssi_resources[index]));
	if (pdev == NULL) {
		return -ENOMEM;
	}
	DBG(0, "%s: Added platform_device %s\n", __FUNCTION__, pdev->name);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto pdev_free;
	}

	DBG(0, "%s: Requesting mem region %08lx..%08lx\n", __FUNCTION__,
		(unsigned long)res->start, (unsigned long)res->end);
	if (!request_mem_region(res->start, resource_size(res), parent->name)) {
		ret = -EBUSY;
		goto pdev_free;
	}

	port->ssi_clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(port->ssi_clk)) {
		ret = PTR_ERR(port->ssi_clk);
		dev_err(&pdev->dev, "Failed to get '%s' clock: %d\n", pdev->name, ret);
		goto pdev_free;
	}
	port->in_use = 1;
	port->parent = pdev;
	*ssi_port = port;
	return 0;

pdev_free:
	DBG(0, "%s: Unregistering %s\n", __FUNCTION__, pdev->name);
	platform_device_unregister(pdev);
	return ret;
}

static inline void mxc_ssi_reserve_port(int index)
{
	mxc_ssi_ports[index].in_use++;
	BUG_ON(mxc_ssi_ports[index].in_use != 1);
}

static inline void mxc_ssi_unreserve_port(int index)
{
	mxc_ssi_ports[index].in_use--;
	BUG_ON(mxc_ssi_ports[index].in_use);
}

static inline int mxc_ssi_port_in_use(int index)
{
	return mxc_ssi_ports[index].in_use;
}

int mxc_ssi_request_port(int index, struct platform_device *parent,
			struct mxc_ssi_port **ssi_port)
{
	int ret = -EBUSY;

	DBG(0, "%s: Requesting SSI port %d\n", __FUNCTION__, index);

	if (index > 0 && index >= NUM_SSI_PORTS) {
		dev_err(&parent->dev, "Bad SSI port index %d; valid range: 0..%d or <0 for any port\n",
			index, NUM_SSI_PORTS - 1);
		return -EINVAL;
	}

	if (ssi_port == NULL) {
		dev_err(&parent->dev, "No pointer for return value\n");
		return -EINVAL;
	}

	mutex_lock(&mxc_ssi_lock);
	if (index >= 0) {
		if (!mxc_ssi_port_in_use(index))
			ret = 0;
	} else {
		for (index = 0; index < NUM_SSI_PORTS; index++) {
			if (!mxc_ssi_port_in_use(index)) {
				ret = 0;
				break;
			}
		}
	}
	if (ret != 0) {
		dev_dbg(&parent->dev, "All SSI ports are in use\n");
		goto unlock;
	}
	mxc_ssi_reserve_port(index);

	ret = _mxc_ssi_init_port(index, parent, ssi_port);
	if (ret)
		goto err;

	ret = index;
	goto unlock;

err:
	mxc_ssi_unreserve_port(index);
unlock:
	mutex_unlock(&mxc_ssi_lock);
	return ret;
}
EXPORT_SYMBOL(mxc_ssi_request_port);

void mxc_ssi_release_port(struct mxc_ssi_port *ssi_port)
{
	if (ssi_port != NULL) {
		WARN_ON(!ssi_port->in_use);
		clk_put(ssi_port->ssi_clk);
		ssi_port->in_use = 0;
		platform_device_unregister(ssi_port->parent);
	}
}
EXPORT_SYMBOL(mxc_ssi_release_port);
