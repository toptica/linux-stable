/*
 * arch/arm/mach-mx27/stk5-baseboard.c
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
 */

#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>

#include <mach/common.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/spi.h>

#include "devices.h"
#include "karo.h"

#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
/*
 * Setup GPIO for a CSPI device to be active
 */
static unsigned int tx27_cspi1_gpios[] = {
#ifdef USE_MXC_SPI_CS
	PD28_PF_CSPI1_SS0,
	PD27_PF_CSPI1_SS1,
#ifndef CONFIG_ARCH_MXC_EHCI_USBH2
	PD26_PF_CSPI1_SS2, /* already in use by the USB controller */
#endif
#endif
	PD31_PF_CSPI1_MOSI,
	PD30_PF_CSPI1_MISO,
	PD29_PF_CSPI1_SCLK,
	PD25_PF_CSPI1_RDY,
};

static unsigned int tx27_cspi2_gpios[] = {
#ifdef USE_MXC_SPI_CS
	PD21_PF_CSPI2_SS0,
	PD20_PF_CSPI2_SS1,
	PD19_PF_CSPI2_SS2,
#endif
	PD24_PF_CSPI2_MOSI,
	PD23_PF_CSPI2_MISO,
	PD22_PF_CSPI2_SCLK,
};

static unsigned int tx27_cspi3_gpios[] = {
#ifdef USE_MXC_SPI_CS
	PE21_AF_CSPI3_SS,
#endif
	PE18_AF_CSPI3_MISO,
	PE22_AF_CSPI3_MOSI,
	PE23_AF_CSPI3_SCLK,
};

static int __init stk5_spi_active(int cspi_mod)
{
	int ret;

	DBG(0, "%s: Configuring pins for CSPI%d\n", __FUNCTION__,
		cspi_mod + 1);

	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		ret = mxc_gpio_setup_multiple_pins(tx27_cspi1_gpios,
						ARRAY_SIZE(tx27_cspi1_gpios),
						"CSPI1");
		if (ret != 0)
			return ret;
#ifndef USE_MXC_SPI_CS
		/* release the CS gpio pins, so that the SPI driver may request them again. */
		mxc_gpio_release_multiple_pins(tx27_cspi1_gpios, 3);
#endif
		break;

	case 1:
		/* SPI2 */
		ret = mxc_gpio_setup_multiple_pins(tx27_cspi2_gpios,
						ARRAY_SIZE(tx27_cspi2_gpios),
						"CSPI2");
		if (ret != 0)
			return ret;
#ifndef USE_MXC_SPI_CS
		/* release the CS gpio pins, so that the SPI driver may request them again. */
		mxc_gpio_release_multiple_pins(tx27_cspi2_gpios, 3);
#endif
		break;

	case 2:
		/* SPI3 */
		ret = mxc_gpio_setup_multiple_pins(tx27_cspi3_gpios,
						ARRAY_SIZE(tx27_cspi3_gpios),
						"CSPI3");
		if (ret != 0)
			return ret;
#ifndef USE_MXC_SPI_CS
		/* release the CS gpio pins, so that the SPI driver may request them again. */
		mxc_gpio_release_multiple_pins(tx27_cspi3_gpios, 1);
#endif
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

/*
 * Setup GPIO for a CSPI device to be inactive
 */
static int __init stk5_spi_inactive(int cspi_mod)
{
	DBG(0, "%s: Releasing pins for CSPI%d\n", __FUNCTION__,
		cspi_mod + 1);

	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		mxc_gpio_release_multiple_pins(tx27_cspi1_gpios,
					ARRAY_SIZE(tx27_cspi1_gpios));
		break;
	case 1:
		/* SPI2 */
		mxc_gpio_release_multiple_pins(tx27_cspi2_gpios,
					ARRAY_SIZE(tx27_cspi2_gpios));
		break;
	case 2:
		/* SPI3 */
		mxc_gpio_release_multiple_pins(tx27_cspi3_gpios,
					ARRAY_SIZE(tx27_cspi3_gpios));
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static struct spi_board_info stk5_custom_spi_info[] __initdata = {
	{
		.modalias = "spidev",
		.max_speed_hz = 2000000,
		.bus_num = 0,
		.chip_select = 0,
	},
	{
		.modalias = "spidev",
		.max_speed_hz = 2000000,
		.bus_num = 0,
		.chip_select = 1,
	},
};
#else
#error No SPI chip driver configured
#endif

#ifdef CONFIG_SPI_IMX_SELECT1
static int stk5_custom_spi1_chipselect[] = {
	/*
	 * could use 'MXC_SPI_CS(0..2)' here to use the SPI controller CS logic,
	 * but this would prevent the driver working correctly with some fast
	 * devices because CS will be deasserted whenever the TX FIFO runs empty.
	 */
#ifdef USE_MXC_SPI_CS
	MXC_SPI_CS(0),
	MXC_SPI_CS(1),
#ifndef CONFIG_ARCH_MXC_EHCI_USBH2
	MXC_SPI_CS(2),
#endif
#else
	GPIO_PORTD | 28,
	GPIO_PORTD | 27,
#ifndef CONFIG_ARCH_MXC_EHCI_USBH2
	GPIO_PORTD | 26, /* Not available on STK5 when USBH2 is enabled */
#endif
#endif
};

static struct spi_imx_master stk5_custom_spi1_data = {
	.chipselect = stk5_custom_spi1_chipselect,
	.num_chipselect = ARRAY_SIZE(stk5_custom_spi1_chipselect),
};
#endif

#ifdef CONFIG_SPI_IMX_SELECT2
static int stk5_custom_spi2_chipselect[] = {
	/* see above note */
#ifdef USE_MXC_SPI_CS
	MXC_SPI_CS(0),
	MXC_SPI_CS(1),
	MXC_SPI_CS(2),
#else
	GPIO_PORTD | 21,
	GPIO_PORTD | 20,
	GPIO_PORTD | 19,
#endif
};

static struct spi_imx_master stk5_custom_spi2_data = {
	.chipselect = stk5_custom_spi2_chipselect,
	.num_chipselect = ARRAY_SIZE(stk5_custom_spi2_chipselect),
};
#endif

#ifdef CONFIG_SPI_IMX_SELECT3
static int stk5_custom_spi3_chipselect[] = {
	/* see above note */
#ifdef USE_MXC_SPI_CS
	MXC_SPI_CS(0),
#else
	GPIO_PORTE | 21,
#endif
};

static struct spi_imx_master stk5_custom_spi3_data = {
	.chipselect = stk5_custom_spi3_chipselect,
	.num_chipselect = ARRAY_SIZE(stk5_custom_spi3_chipselect),
};
#endif

static struct {
	struct platform_device *pdev;
	void *pdata;
} stk5_custom_spi_devices[] __initdata = {
#ifdef CONFIG_SPI_IMX_SELECT1
	{ .pdev = &mx2_spi1_device, .pdata = &stk5_custom_spi1_data, },
#endif
#ifdef CONFIG_SPI_IMX_SELECT2
	{ .pdev = &mx2_spi2_device, .pdata = &stk5_custom_spi2_data, },
#endif
#ifdef CONFIG_SPI_IMX_SELECT3
	{ .pdev = &mx2_spi3_device, .pdata = &stk5_custom_spi3_data, },
#endif
};

static int __init stk5_custom_spi_init(void)
{
	int ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(stk5_custom_spi_devices); i++) {
		int err;

		err = stk5_spi_active(i);
		if (err) {
			printk(KERN_ERR "Failed to configure pins for CSPI%d: %d\n",
				i + 1, err);
			if (ret == 0)
				ret = err;
			continue;
		}
		DBG(0, "%s: Registering SPI info for CSPI%d\n", __FUNCTION__,
			i + 1);

		err = spi_register_board_info(stk5_custom_spi_info,
					ARRAY_SIZE(stk5_custom_spi_info));
		if (err) {
			printk(KERN_ERR "%s: Failed to register SPI board_info: %d\n",
				__FUNCTION__, err);
			goto err;
		}
		DBG(0, "%s: Registering CSPI%d device\n", __FUNCTION__, i + 1);

		err = mxc_register_device(stk5_custom_spi_devices[i].pdev,
					stk5_custom_spi_devices[i].pdata);
		DBG(0, "%s: mxc_register_device() returned: %d\n", __FUNCTION__, err);
		if (err == 0)
			continue;
	err:
		if (ret == 0)
			ret = err;
		stk5_spi_inactive(i);
	}
	return ret;
}
arch_initcall(stk5_custom_spi_init);
#endif // defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)

#if defined(CONFIG_RTC_DRV_DS13XX) || defined(CONFIG_RTC_DRV_DS13XX_MODULE)
static struct ds13xx_platform_data stk5_ds1339_data = {
	.type = ds_1339,
	.ctrl = 1 << 2, /* set INTCN to disable SQW output */
	.trc = DS1339_TRC_ENABLE | DS1339_DIODE_ENABLE | DS1339_TRC_250R,
};

static struct platform_device stk5_ds1339_device = {
	.name = "rtc-ds13xx",
	.dev = {
		.platform_data = &stk5_ds1339_data,
	},
};
#endif

#if defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT) || defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT_MODULE)
static u64 mxc_emma_dmamask = 0xffffffffUL;

static struct platform_device stk5_v4l2out_device = {
	.name = "MXC Video Output",
	.id = 0,
	.dev = {
		.dma_mask = &mxc_emma_dmamask,
		.coherent_dma_mask = ~0UL,
	},
};
#endif

static struct platform_dev_list {
	struct platform_device *pdev;
	void *pdata;
} stk5_devices[] __initdata = {
};
#define STK5_NUM_DEVICES		ARRAY_SIZE(stk5_devices)

int __init stk5_custom_board_init(void)
{
	int i;

	DBG(0, "%s: \n", __FUNCTION__);

	/* enable SSI3_INT (PC23) for IRQ probing */
	set_irq_flags(gpio_to_irq(GPIO_PORTC | 23), IRQF_VALID | IRQF_PROBE);

	for (i = 0; i < STK5_NUM_DEVICES; i++) {
		int ret;

		if (stk5_devices[i].pdev == NULL)
			continue;
		if (stk5_devices[i].pdata == NULL)  {
			DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
				__FUNCTION__, i, stk5_devices[i].pdev, &stk5_devices[i].pdev->dev,
				stk5_devices[i].pdev->name);
			ret = platform_device_register(stk5_devices[i].pdev);
		} else {
			ret = mxc_register_device(stk5_devices[i].pdev,
						stk5_devices[i].pdata);
		}
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, stk5_devices[i].pdev->name, ret);
		}
	}
	DBG(0, "%s: Done\n", __FUNCTION__);
	return 0;
}
