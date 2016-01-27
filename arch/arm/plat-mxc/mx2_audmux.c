/*
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
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
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define DEBUG

#include <linux/platform_device.h>
#include <mach/mxc_audmux.h>
#include <linux/io.h>

#define DRV_NAME "mx2_audmux"

struct mx2_audmux_data {
	void __iomem *io;
	struct platform_device *pdev;
};

/*
 * Table of internal AUDMUX port mappings to their offchip
 * SSI pin groups. This is verified for the i.MX27 CPU.
 */
static const int mx2_audmux_routing[] = {
	3,	/* SSI1_* pin group is AUDMUX port 4 */
	4,	/* SSI2_* pin group is AUDMUX port 5 */
	5,	/* SSI3_* pin group is AUDMUX port 6 */
	2	/* SSI4_* pin group is AUDMUX port 3 */
};

/* there is only one unit in this CPU */
static struct mx2_audmux_data audmux;

/* this unit has hardly predictable register offsets */
static const unsigned int reg_offsets[] = {
	HPCR1, HPCR2, HPCR3, PPCR1, PPCR2, PPCR3
};

#define    _reg_HPCR(a)	reg_offsets[a]	/* 0..1 */
#define    _reg_PPCR(a)	reg_offsets[a]	/* 2..5 */


/*
 * Route an internal SSI channel (1 or 2) as a slave to two
 * of the four SSI pin groups (1 to 4).
 * Note: 6-wire mode and i2s data only. This means no network
 * feature on this bus.
 *
 * This means:
 *
 *  SSI unit      AUDMUX@SSI        AUDMUX@per   SSI_pin
 * ---------------------------------------------------------------------------
 *    SRCK(in)    SRCK(out)                      SSI?_CLK (2)
 *    SRFS(in)    SRFS(out)                      SSI?_FS (2)
 *    SRXD(in)                      Da           SSI?_RXD (2)
 *    STCK(in)    TCLK(out)         CLK          SSI?_CLK (1)
 *    STFS(in)    TFS(out)          FS           SSI?_FS (1)
 *    STXD(out)                     Db           SSI?_TXD (1)
 *
 * -> ssi_pin1 means the output channel, ssi_pin2 means the input channel.
 * -> RXD from ssi_pin1 is not used, TXD from ssi_pin2 is not used.
s */

/*
 * Route an internal SSI channel (1 or 2) as a slave to one
 * of the four SSI pin groups (1 to 4).
 * Note: 4-wire mode and i2s data only. This means no network
 * feature on this bus.
 *
 * This means:
 *
 *  SSI unit       AUDMUX@SSI        AUDMUX@per   SSI_pin
 * ---------------------------------------------------------------------------
 *    SRCK (in)
 *    SRFS (in)
 *    SRXD (in)                      Da           SSI?_RXD
 *    STCK (in)    TCLK(out)         CLK          SSI?_CLK
 *    STFS (in)    TFS(out)          FS           SSI?_FS
 *    STXD (out)                     Db           SSI?_TXD
 */
int mx2_audmux_configure_sync_slave(int ssi_unit, int ssi_pin, unsigned int flags)
{
	int host_port, target_port;
	u32 host_val, target_val;

	if (!audmux.io) {
		pr_err("AUDMUX: not yet initialized\n");
		return -ENODEV;
	}

	if (ssi_unit == 0 || ssi_unit > 6 || ssi_pin == 0 || ssi_pin > 6) {
		dev_err(&audmux.pdev->dev,
			"AUDMUX: source or target port not defined\n");
		return -ENODEV;
	}

	host_port = ssi_unit - 1;
	target_port = mx2_audmux_routing[ssi_pin - 1];

	pr_debug("AUDMUX: connecting SSI%d's port %d to peripheral port %d\n",
		ssi_unit, ssi_unit, target_port + 1);

	/* starting to configure the host port */
	host_val = HPCR_SYN;	/* 4wire mode */
	/* where the RxD signal comes from */
	host_val |= HPCR_RXDSEL(target_port);
	/*
	 * SSI@STFS(in) <-- AUDMUX@TFS(out)
	 * SSI@STCK(in) <-- AUDMUX@TCLK(out)
	 */
	if (flags & MX2_AUDMUX_MODE_AC97) {
		host_val |= HPCR_RCLKDIR;	/* RCLK is output at this port */
	} else {
		host_val |= HPCR_TFSDIR;	/* TFS is output at this port */
		host_val |= HPCR_RFSDIR;	/* RFS is output at this port */
	}
	host_val |= HPCR_TCLKDIR;	/* TCLK is output at this port */
	/*
	 * AUDMUX@TFS(out) select from which other port and route to
	 * port's TFS (not RFS)
	 * AUDMUX@TCLK(out) select from which other port and route to
	 * port's TCLK (not TCLK)
	 */
	host_val |= HPCR_TFCSEL(target_port);
	if (flags & MX2_AUDMUX_MODE_AC97) {
		host_val |= HPCR_RFCSEL(target_port);
	}

	/* and now configure the port to the peripheral */
	target_val = PPCR_SYN;	/* 4wire mode */
	/* where the RxD signal comes from */
	target_val |= PPCR_RXDSEL(host_port);
	if (flags & MX2_AUDMUX_MODE_AC97) {
		target_val |= PPCR_RFSDIR;
		target_val |= PPCR_TFSDIR;
	}

	__raw_writel(target_val, audmux.io + reg_offsets[target_port]);
	__raw_writel(host_val, audmux.io + reg_offsets[host_port]);

	pr_debug("H1: %08X, H2: %08X, S1: %08X, S2: %08X, S3: %08X, S4: %08X\n",
		__raw_readl(audmux.io + _reg_HPCR(0)),
		__raw_readl(audmux.io + _reg_HPCR(1)),
		__raw_readl(audmux.io + _reg_PPCR(2)),
		__raw_readl(audmux.io + _reg_PPCR(3)),
		__raw_readl(audmux.io + _reg_PPCR(4)),
		__raw_readl(audmux.io + _reg_PPCR(5)));
	return 0;
}
EXPORT_SYMBOL(mx2_audmux_configure_sync_slave);

static int __devinit mx2_audmux_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	pr_info("AUDMUX: probing\n");

	audmux.pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no resource data for AUDMUX\n");
		return -ENODEV;
	}

	if (!request_mem_region(res->start, resource_size(res), DRV_NAME)) {
		dev_err(&pdev->dev, "request_mem_region failed for AUDMUX\n");
		return -EBUSY;
	}

	audmux.io = ioremap(res->start, resource_size(res));
	if (audmux.io == NULL) {
		dev_err(&pdev->dev, "mapping region failed for AUDMUX\n");
		ret = -ENOMEM;
		goto err1;
	}

	return 0;	/* return happy */

err1:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static int __devexit mx2_audmux_remove(struct platform_device *pdev)
{
	struct resource *res;

	printk(KERN_DEBUG "%s: \n", __FUNCTION__);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	BUG_ON(!res);

	printk(KERN_DEBUG "%s: Unmapping %p\n", __FUNCTION__, audmux.io);
	iounmap(audmux.io);
	audmux.io = NULL;
	printk(KERN_DEBUG "%s: Release memregion %08lx..%08lx\n", __FUNCTION__,
		(unsigned long)res->start,
		(unsigned long)(res->start + resource_size(res) - 1));
	release_mem_region(res->start, resource_size(res));

	return 0;
}

struct platform_driver mx2_audmux_driver = {
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= mx2_audmux_probe,
	.remove		= __devexit_p(mx2_audmux_remove),
};

static int __init mx2_audmux_init(void)
{
	printk(KERN_DEBUG "%s: Registering %s\n", __FUNCTION__,
		mx2_audmux_driver.driver.name);
	return platform_driver_register(&mx2_audmux_driver);
}

static void __exit mx2_audmux_exit(void)
{
	printk(KERN_DEBUG "%s: Unregistering %s\n", __FUNCTION__,
		mx2_audmux_driver.driver.name);
	platform_driver_unregister(&mx2_audmux_driver);
}

module_init(mx2_audmux_init);
module_exit(mx2_audmux_exit);

MODULE_LICENSE("GPL");
