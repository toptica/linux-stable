/*
 * arch/arm/mach-mx5/stk5-custom.c
 *
 * Copyright (C) 2010  Lothar Wassmann <LW@KARO-electronics.de>
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
 *
 * This file adds support for custom devices added to the Ka-Ro Starterkit-5 (STK5) baseboard
 */

#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <mach/common.h>
#include <mach/iomux-mx51.h>
#include <mach/spi.h>
#include <mach/imx_spi.h>

#include "devices.h"
#include "karo.h"

#ifdef CONFIG_SPI_IMX_SELECT1
static struct mx51_pad_desc stk5_custom_cspi1_pads[] = {
	MX51_PIN_CSPI1_MOSI__CSPI1_MOSI,
	MX51_PIN_CSPI1_MISO__CSPI1_MISO,
	MX51_PIN_CSPI1_SS0__CSPI1_SS0,
	MX51_PIN_CSPI1_SS1__CSPI1_SS1,
	MX51_PIN_CSPI1_SCLK__CSPI1_SCLK,
	MX51_PIN_CSPI1_RDY__CSPI1_RDY,
};

#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
static int stk5_custom_spi_chipselect[] = {
	MXC_SPI_CS(0),
};

static struct spi_board_info stk5_custom_spi_info[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 2000000,
		.bus_num = 0,
		.chip_select = 0,
	},
};
#else
#error No SPI chip driver configured
#endif

static struct spi_imx_master stk5_custom_spi1_data = {
	.chipselect = stk5_custom_spi_chipselect,
	.num_chipselect = ARRAY_SIZE(stk5_custom_spi_chipselect),
	.ecspi = 1,
};

static int __init stk5_spi_init(void)
{
	int ret;

	ret = mx51_iomux_request_pads(stk5_custom_cspi1_pads,
				ARRAY_SIZE(stk5_custom_cspi1_pads));
	if (ret) {
		printk(KERN_ERR "Failed to configure CSPI1 pads\n");
	}

	ret = spi_register_board_info(stk5_custom_spi_info,
				ARRAY_SIZE(stk5_custom_spi_info));
	if (ret) {
		printk(KERN_ERR "%s: Failed to register SPI board_info: %d\n",
			__FUNCTION__, ret);
		goto err;
	}
	return 0;

err:
	mx51_iomux_release_pads(stk5_custom_cspi1_pads,
				ARRAY_SIZE(stk5_custom_cspi1_pads));
	return ret;
}
device_initcall(stk5_spi_init);
#endif

static struct {
	struct platform_device *pdev;
	void *pdata;
} stk5_custom_devices[] = {
	/* add platform_devices to be registered like below: */
#ifdef CONFIG_SPI_IMX_SELECT1
	{ .pdev = &mx51_spi1_device, &stk5_custom_spi1_data, },
#endif
};

/*!
 * Board specific initialization.
 */
int __init stk5_custom_board_init(void)
{
	int i;
	int ret;

	DBG(0, "%s: \n", __FUNCTION__);

	for (i = 0; i < ARRAY_SIZE(stk5_custom_devices); i++) {
		DBG(0, "%s: Registering device[%d] %s\n", __FUNCTION__,
			i, stk5_custom_devices[i].pdev->name);

		if (stk5_custom_devices[i].pdata) {
			ret = mxc_register_device(stk5_custom_devices[i].pdev,
						stk5_custom_devices[i].pdata);
		} else {
			ret = platform_device_register(stk5_custom_devices[i].pdev);
		}
		if (ret) {
			DBG(0, "%s: Failed to register device[%d] %s\n",
				__FUNCTION__, i,
				stk5_custom_devices[i].pdev->name);
		}
	}
	DBG(0, "%s: Done\n", __FUNCTION__);
	return 0;
}
