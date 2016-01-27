/*
 * Copyright (c) 2008 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/mxc_ehci.h>

/* called during probe() after chip reset completes */
static int ehci_mxc_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
	    HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	hcd->has_tt = 1;

	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	ehci_port_power(ehci, 0);
	return 0;
}

static const struct hc_driver ehci_mxc_hc_driver = {
	.description = hcd_name,
	.product_desc = "Freescale On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_mxc_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,
	.endpoint_reset = ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,
};

static int ehci_mxc_drv_probe(struct platform_device *pdev)
{
	struct mxc_usbh_platform_data *pdata = pdev->dev.platform_data;
	struct usb_hcd *hcd;
	struct resource *res;
	int irq, ret, temp;
	struct clk *usbclk, *ahbclk;

	dev_info(&pdev->dev, "initializing i.MX USB Controller\n");

	/* Need platform data for setup */
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", dev_name(&pdev->dev));
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);

	hcd = usb_create_hcd(&ehci_mxc_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		ret = -ENOMEM;
		goto err1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		ret = -ENODEV;
		goto err1;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		ret = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "error mapping memory\n");
		ret = -EFAULT;
		goto err2;
	}

	ahbclk = clk_get(NULL, "usb_ahb");
	if (IS_ERR(ahbclk)) {
		ret = PTR_ERR(ahbclk);
		printk(KERN_ERR "Failed to get usb_ahb_clk: %d\n", ret);
		goto err3;
	}
	clk_enable(ahbclk);

	usbclk = clk_get(&pdev->dev, "usb");
	if (IS_ERR(usbclk)) {
		ret = PTR_ERR(usbclk);
		printk(KERN_ERR "Failed to get usb_clk: %d\n", ret);
		goto err4;
	}
	clk_enable(usbclk);

	if (pdata->init) {
		ret = pdata->init(pdev);
		if (ret) {
			dev_err(&pdev->dev, "platform init failed\n");
			goto err5;
		}
	}

	/* Set to Host mode */
	temp = readl(hcd->regs + 0x1a8);
	writel(temp | 0x3, hcd->regs + 0x1a8);

	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (ret)
		goto err6;

	platform_set_drvdata(pdev, hcd);
	clk_put(usbclk);

	return 0;
err6:
	if (pdata->exit)
		pdata->exit(pdev);
err5:
	clk_disable(usbclk);
	clk_put(usbclk);
err4:
	clk_disable(ahbclk);
	clk_put(ahbclk);
err3:
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
	return ret;
}

static int ehci_mxc_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct mxc_usbh_platform_data *pdata = pdev->dev.platform_data;
	struct clk *usbclk, *ahbclk;

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	platform_set_drvdata(pdev, NULL);

	if (pdata->exit)
		pdata->exit(pdev);

	usbclk = clk_get(&pdev->dev, "usb");
	if (!IS_ERR(usbclk)) {
		clk_disable(usbclk);
		clk_put(usbclk);
	}

	ahbclk = clk_get(NULL, "usb_ahb");
	if (!IS_ERR(ahbclk)) {
		clk_disable(ahbclk);
		clk_put(ahbclk);
	}
	return 0;
}

MODULE_ALIAS("platform:mxc-ehci");

static struct platform_driver ehci_mxc_driver = {
	.probe = ehci_mxc_drv_probe,
	.remove = ehci_mxc_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "mxc-ehci",
	},
};
