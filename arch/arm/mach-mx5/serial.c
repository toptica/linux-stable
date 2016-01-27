/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*!
 * @file mach-mx5/serial.c
 *
 * @brief This file contains the UART initiliazation.
 *
 * @ingroup MSL_MX51
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>

#include "devices.h"

static struct resource mx51_uart0_resource[] = {
	{
		.start = UART1_BASE_ADDR,
		.end = UART1_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART1,
		.end = MXC_INT_UART1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_uart_device1 = {
	.name = "imx-uart",
	.id = 0,
	.resource = mx51_uart0_resource,
	.num_resources = ARRAY_SIZE(mx51_uart0_resource),
};

static struct resource mx51_uart1_resource[] = {
	{
		.start = UART2_BASE_ADDR,
		.end = UART2_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART2,
		.end = MXC_INT_UART2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_uart_device2 = {
	.name = "imx-uart",
	.id = 1,
	.resource = mx51_uart1_resource,
	.num_resources = ARRAY_SIZE(mx51_uart1_resource),
};

static struct resource mx51_uart2_resource[] = {
	{
		.start = UART3_BASE_ADDR,
		.end = UART3_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART3,
		.end = MXC_INT_UART3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_uart_device3 = {
	.name = "imx-uart",
	.id = 2,
	.resource = mx51_uart2_resource,
	.num_resources = ARRAY_SIZE(mx51_uart2_resource),
};
