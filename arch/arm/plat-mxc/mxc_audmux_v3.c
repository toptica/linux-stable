/*
 * (C) Copyright Lothar Wassmann <LW@KARO-electronics.de>
 * based on: mx2_audmux.c
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
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
#include <linux/io.h>
#include <mach/mxc_audmux.h>

#define DRV_NAME "mxc_audmux_v3"

/* number of internal SSI units */
#define MXC_SSI_UNITS		2
/* number of external SSI ports */
#define MXC_SSI_PORTS		4

#define PTCR1	0x00
#define PDCR1	0x04
#define PTCR2	0x08
#define PDCR2	0x0C
#define PTCR3	0x10
#define PDCR3	0x14
#define PTCR4	0x18
#define PDCR4	0x1C
#define PTCR5	0x20
#define PDCR5	0x24
#define PTCR6	0x28
#define PDCR6	0x2C
#define PTCR7	0x30
#define PDCR7	0x34
#define CNMCR	0x38

#define PTCR(n)		((n) * 8 + PTCR1)
#define PDCR(n)		((n) * 8 + PDCR1)

/* all port timing control registers share these bits */
#define PTCR_SYN		(1 << 11)
#define PTCR_RCSEL_SHIFT	12
#define PTCR_RCSEL_MASK		(0x7 << PTCR_RCSEL_SHIFT)
#define PTCR_RCSEL(x)		(((x) & 0xf) << PTCR_RCSEL_SHIFT)
#define PTCR_RCLKDIR		(1 << 16)
#define PTCR_RFSEL_SHIFT	17
#define PTCR_RFSEL_MASK		(0x7 << PTCR_RFSEL_SHIFT)
#define PTCR_RFSEL(x)		(((x) & 0xf) << PTCR_RFSEL_SHIFT)
#define PTCR_RFSDIR		(1 << 21)
#define PTCR_TCSEL_SHIFT	22
#define PTCR_TCSEL_MASK		(0x7 << PTCR_TCSEL_SHIFT)
#define PTCR_TCSEL(x)		(((x) & 0xf) << PTCR_TCSEL_SHIFT)
#define PTCR_TCLKDIR		(1 << 26)
#define PTCR_TFSEL_SHIFT	27
#define PTCR_TFSEL_MASK		(0x7 << PTCR_TFSEL_SHIFT)
#define PTCR_TFSEL(x)		(((x) & 0xf) << PTCR_TFSEL_SHIFT)
#define PTCR_TFSDIR		(1 << 31)

/* all port data control registers share these bits */
#define PDCR_INNMASK		(0xff << 0)
#define PDCR_MODE_SHIFT		8
#define PDCR_MODE_MASK		(0x3 << PDCR_MODE_SHIFT)
#define PDCR_MODE_NORMAL	(0 << PDCR_MODE_SHIFT)
#define PDCR_MODE_INM		(1 << PDCR_MODE_SHIFT)
#define PDCR_MODE_CEBUS		(2 << PDCR_MODE_SHIFT)
#define PDCR_RXDSEL(x)		(((x) & 0x7) << 13)
#define PDCR_TXRXEN		(1 << 30)

struct mxc_audmux_v3_data {
	void __iomem *io;
	struct platform_device *pdev;
};

/* there is only one unit in this CPU */
static struct mxc_audmux_v3_data audmux;

/*
 * Route an internal SSI channel (1 or 2) as a slave to two
 * of the four SSI pin groups (1 to 4).
 * MXC_AUDMUX_MODE_SSI: 6-wire mode and i2s data only. This means no network
 * feature on this bus.
 *
 * This means:
 *
 *  SSI unit      AUDMUX@SSI        AUDMUX@per   SSI_pin
 * ---------------------------------------------------------------------------
 *    SRCK(in)    SRCK(out)                      SSI#_CLK (2)
 *    SRFS(in)    SRFS(out)                      SSI#_FS  (2)
 *    SRXD(in)                      Da           SSI#_RXD (2)
 *    STCK(in)    TCLK(out)         CLK          SSI#_CLK (1)
 *    STFS(in)    TFS(out)          FS           SSI#_FS  (1)
 *    STXD(out)                     Db           SSI#_TXD (1)
 *
 * -> ssi_pin1 means the output channel, ssi_pin2 means the input channel.
 * -> RXD from ssi_pin1 is not used, TXD from ssi_pin2 is not used.
 *
 * MXC_AUDMUX_MODE_AC97: 4-wire mode and AC97 data only.
 *
 * This means:
 *
 *  SSI unit       AUDMUX@SSI        AUDMUX@per   SSI_pin
 * ---------------------------------------------------------------------------
 *    SRCK (in)
 *    SRFS (in)
 *    SRXD (in)                      Da           SSI#_RXD
 *    STCK (in)    TCLK(out)         CLK          SSI#_CLK
 *    STFS (in)    TFS(out)          FS           SSI#_FS
 *    STXD (out)                     Db           SSI#_TXD
 */
int mxc_audmux_v3_configure_sync_slave(unsigned int ssi_unit,
				unsigned int ssi_pin, unsigned int flags)
{
	int host_port, target_port;
	u32 host_ptcr, host_pdcr, target_ptcr, target_pdcr;
	int i;

	if (!audmux.io) {
		pr_err("AUDMUX: not yet initialized\n");
		return -ENODEV;
	}

	if ((ssi_unit == 0 || ssi_unit > MXC_SSI_UNITS) ||
		(ssi_pin == 0 || ssi_pin > MXC_SSI_PORTS)) {
		dev_err(&audmux.pdev->dev,
			"AUDMUX: invalid source or target port: %d, %d\n",
			ssi_unit, ssi_pin);
		return -ENODEV;
	}

	host_port = ssi_unit - 1;
	target_port = ssi_pin - 1;

	pr_debug("AUDMUX: connecting SSI port %d to peripheral port %d\n",
		ssi_unit, ssi_pin);

	/* where the RxD signal comes from */
	host_pdcr = PDCR_RXDSEL(target_port);

	/* starting to configure the host port */
	host_ptcr = PTCR_SYN;	/* 4-wire mode */
	/*
	 * SSI@STFS(in) <-- AUDMUX@TFS(out)
	 * SSI@STCK(in) <-- AUDMUX@TCLK(out)
	 */
	if (flags & MXC_AUDMUX_MODE_AC97) {
		host_ptcr |= PTCR_RCLKDIR;	/* RCLK is output at this port */
	} else {
		host_ptcr |= PTCR_TFSDIR;	/* TFS is output at this port */
		host_ptcr |= PTCR_RFSDIR;	/* RFS is output at this port */
	}
	host_ptcr |= PTCR_TCLKDIR;	/* TCLK is output at this port */

	/*
	 * AUDMUX@TFS(out) select from which other port and route to
	 * port's TFS (not RFS)
	 * AUDMUX@TCLK(out) select from which other port and route to
	 * port's TCLK (not TCLK)
	 */
	host_ptcr |= PTCR_TCSEL(target_port);
	if (flags & MXC_AUDMUX_MODE_AC97) {
		host_ptcr |= PTCR_RCSEL(target_port);
	} else {
		host_ptcr |= PTCR_TFSEL(target_port);
	}

	/* and now configure the port to the peripheral */
	target_ptcr = PTCR_SYN;	/* 4wire mode */

	/* where the RxD signal comes from */
	target_pdcr = PDCR_RXDSEL(host_port);
	if (flags & MXC_AUDMUX_MODE_AC97) {
		target_ptcr |= PTCR_RFSDIR;
		target_ptcr |= PTCR_TFSDIR;
	}

	__raw_writel(target_ptcr, audmux.io + PTCR(target_port));
	__raw_writel(target_pdcr, audmux.io + PDCR(target_port));
	__raw_writel(host_ptcr, audmux.io + PTCR(host_port));
	__raw_writel(host_pdcr, audmux.io + PDCR(host_port));

	for (i = 0; i < MXC_SSI_PORTS + MXC_SSI_UNITS; i++) {
		pr_debug("PTCR%d: %08X, PDCR%d: %08X\n",
			i, __raw_readl(audmux.io + PTCR(i)),
			i, __raw_readl(audmux.io + PDCR(i)));
	}
	return 0;
}
EXPORT_SYMBOL(mxc_audmux_v3_configure_sync_slave);

static int __devinit mxc_audmux_v3_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	pr_info("AUDMUX: probing\n");

	audmux.pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no resource data for AUDMUX V3\n");
		return -ENODEV;
	}

	if (!request_mem_region(res->start, resource_size(res), DRV_NAME)) {
		dev_err(&pdev->dev, "request_mem_region failed for AUDMUX V3\n");
		return -EBUSY;
	}

	audmux.io = ioremap(res->start, resource_size(res));
	if (audmux.io == NULL) {
		dev_err(&pdev->dev, "mapping region failed for AUDMUX V3\n");
		ret = -ENOMEM;
		goto err1;
	}
	return 0;

err1:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static int __devexit mxc_audmux_v3_remove(struct platform_device *pdev)
{
	struct resource *res;

	dev_dbg(&pdev->dev, "%s: \n", __FUNCTION__);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	BUG_ON(!res);

	dev_dbg(&pdev->dev, "%s: Unmapping %p\n", __FUNCTION__, audmux.io);
	iounmap(audmux.io);
	audmux.io = NULL;
	dev_dbg(&pdev->dev, "%s: Release memregion %08lx..%08lx\n", __FUNCTION__,
		(unsigned long)res->start,
		(unsigned long)(res->start + resource_size(res) - 1));
	release_mem_region(res->start, resource_size(res));

	return 0;
}

struct platform_driver mxc_audmux_v3_driver = {
	.driver = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= mxc_audmux_v3_probe,
	.remove		= __devexit_p(mxc_audmux_v3_remove),
};

static int __init mxc_audmux_v3_init(void)
{
	printk(KERN_DEBUG "%s: Registering %s\n", __FUNCTION__,
		mxc_audmux_v3_driver.driver.name);
	return platform_driver_register(&mxc_audmux_v3_driver);
}
module_init(mxc_audmux_v3_init);

static void __exit mxc_audmux_v3_exit(void)
{
	printk(KERN_DEBUG "%s: Unregistering %s\n", __FUNCTION__,
		mxc_audmux_v3_driver.driver.name);
	platform_driver_unregister(&mxc_audmux_v3_driver);
}
module_exit(mxc_audmux_v3_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for the Freescale i.MX SSI port multiplexer (AUDMUX)");
MODULE_ALIAS("platform:mxc_audmux_v3");
