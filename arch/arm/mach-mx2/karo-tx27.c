/*
 * arch/arm/mach-mx27/karo-tx27.c
 *
 * Copyright (C) 2008  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * based on: arch/arm/mach-mx27ads.c (C) Freescale Semiconductor, Inc.
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
#include <mtd/mtd-abi.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/fec_enet.h>
#include <linux/if_ether.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/clkdev.h>
#include <asm/hw_irq.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/imx-uart.h>
#include <mach/iomux.h>
#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/mmc.h>
#include <mach/mxc_nand.h>
#include <mach/i2c.h>
#include <mach/ulpi.h>
#include <mach/mxc_ehci.h>
#include <mach/imxfb.h>

#include <mach/board-tx27.h>

#include "crm_regs_mx27.h"
#include "devices.h"
#include "karo.h"

#ifdef DEBUG
int tx27_debug = 1;
module_param(tx27_debug, int, S_IRUGO | S_IWUSR);
#endif

#if defined(CONFIG_SERIAL_IMX) || defined(CONFIG_SERIAL_IMX_MODULE)
static int tx27_uart_pins[][4] = {
	{
		PE12_PF_UART1_TXD,
		PE13_PF_UART1_RXD,
		PE14_PF_UART1_CTS,
		PE15_PF_UART1_RTS,
	},{
		PE6_PF_UART2_TXD,
		PE7_PF_UART2_RXD,
		PE3_PF_UART2_CTS,
		PE4_PF_UART2_RTS,
	},{
		PE8_PF_UART3_TXD,
		PE9_PF_UART3_RXD,
		PE10_PF_UART3_CTS,
		PE11_PF_UART3_RTS,
	},{
		PB28_AF_UART4_TXD,
		PB31_AF_UART4_RXD,
		PB29_AF_UART4_CTS,
		PB26_AF_UART4_RTS,
	},{
		PB18_AF_UART5_TXD,
		PB19_AF_UART5_RXD,
		PB20_AF_UART5_CTS,
		PB21_AF_UART5_RTS,
	},{
		PB10_AF_UART6_TXD,
		PB11_AF_UART6_RXD,
		PB12_AF_UART6_CTS,
		PB13_AF_UART6_RTS,
	},
};

static int tx27_uart_init(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	return mxc_gpio_setup_multiple_pins(tx27_uart_pins[pdev->id],
					    ARRAY_SIZE(tx27_uart_pins[pdev->id]), "UART");
}

static void tx27_uart_exit(struct platform_device *pdev)
{
	DBG(0, "%s: \n", __FUNCTION__);
	mxc_gpio_release_multiple_pins(tx27_uart_pins[pdev->id],
			ARRAY_SIZE(tx27_uart_pins[pdev->id]));
}

static struct imxuart_platform_data tx27_uart_port = {
	.init = tx27_uart_init,
	.exit = tx27_uart_exit,
	.flags = IMXUART_HAVE_RTSCTS,
};
#endif

static struct platform_device *tx27_uart_devices[] __initdata = {
#if UART1_ENABLED
	&mxc_uart_device0,
#endif
#if UART2_ENABLED
	&mxc_uart_device1,
#endif
#if UART3_ENABLED
	&mxc_uart_device2,
#endif
#if UART4_ENABLED
	&mxc_uart_device3,
#endif
#if UART5_ENABLED
	&mxc_uart_device4,
#endif
#if UART6_ENABLED
	&mxc_uart_device5,
#endif
};

#if defined(CONFIG_FEC_PHYLIB) || defined(CONFIG_FEC_PHYLIB_MODULE)
#define FEC_MII_IRQ		gpio_to_irq(GPIO_PORTD | 16)
/*
 * Setup GPIO for FEC device to be active
 *
 */
static unsigned int mx27_fec_gpios_off[] = {
	/* configure the PHY strap pins to the correct values */
	MXC_PIN(D, 0, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 1, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 2, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 3, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 4, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 5, GPIO, GPIO_OUT | GPIO_DFLT_HIGH),
	MXC_PIN(D, 6, GPIO, GPIO_OUT | GPIO_DFLT_HIGH),
#ifndef FEC_MII_IRQ
	MXC_PIN(D, 7, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
#else
	MXC_PIN(D, 7, GPIO, GPIO_OUT | GPIO_DFLT_HIGH),
#endif
	MXC_PIN(D, 8, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 9, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 10, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 11, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 12, GPIO, GPIO_OUT | GPIO_DFLT_HIGH),
	MXC_PIN(D, 13, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 14, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 15, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
	MXC_PIN(D, 16, AOUT, GPIO_IN),	/* #INT */
	MXC_PIN(F, 23, GPIO, GPIO_OUT | GPIO_DFLT_LOW),
};

static unsigned int mx27_fec_pwr_gpios[] = {
	MXC_PIN(B, 27, GPIO, GPIO_OUT),		/* FEC PHY power on pin */
	MXC_PIN(B, 30, GPIO, GPIO_OUT),		/* keep FEC reset in original state */
};

static unsigned int mx27_fec_gpios_on[] = {
	PD0_AIN_FEC_TXD0,
	PD1_AIN_FEC_TXD1,
	PD2_AIN_FEC_TXD2,
	PD3_AIN_FEC_TXD3,
	PD4_AOUT_FEC_RX_ER,
	PD5_AOUT_FEC_RXD1,
	PD6_AOUT_FEC_RXD2,
	PD7_AOUT_FEC_RXD3,
	PD8_AF_FEC_MDIO,
	PD9_AIN_FEC_MDC,
	PD10_AOUT_FEC_CRS,
	PD11_AOUT_FEC_TX_CLK,
	PD12_AOUT_FEC_RXD0,
	PD13_AOUT_FEC_RX_DV,
	PD14_AOUT_FEC_RX_CLK,
	PD15_AOUT_FEC_COL,
#ifndef FEC_MII_IRQ
	PD16_AIN_FEC_TX_ER,	/* TX_ER */
#else
	MXC_PIN(D, 16, AOUT, GPIO_IN),	/* #INT */
#endif
	PF23_AIN_FEC_TX_EN,
};

#define TX27_FEC_PWR_GPIO	(GPIO_PORTB | 27)
#define TX27_FEC_RST_GPIO	(GPIO_PORTB | 30)
#define TX27_FEC_PHY_MODE0_GPIO	(GPIO_PORTD | 12)
#define TX27_FEC_PHY_MODE1_GPIO	(GPIO_PORTD | 5)
#define TX27_FEC_PHY_MODE2_GPIO	(GPIO_PORTD | 6)

int gpio_fec_active(void)
{
	int ret;

	ret = mxc_gpio_setup_multiple_pins(mx27_fec_pwr_gpios,
					ARRAY_SIZE(mx27_fec_pwr_gpios),
					"FEC");
	if (ret) {
		return ret;
	}
	/*
	 * If the PHY is already powered on, assume it has been
	 * correctly configured (by the boot loader)
	*/
	if (0 && gpio_get_value(TX27_FEC_PWR_GPIO) &&
	    gpio_get_value(TX27_FEC_RST_GPIO)) {
		ret = mxc_gpio_setup_multiple_pins(mx27_fec_gpios_on,
						ARRAY_SIZE(mx27_fec_gpios_on),
						"FEC");
		if (ret) {
			mxc_gpio_release_multiple_pins(mx27_fec_pwr_gpios,
						ARRAY_SIZE(mx27_fec_pwr_gpios));
		}
	} else {
		DBG(0, "%s: Switching FEC PHY power on\n", __FUNCTION__);
		gpio_set_value(TX27_FEC_PWR_GPIO, 1);
		DBG(0, "%s: Asserting FEC PHY reset\n", __FUNCTION__);
		gpio_set_value(TX27_FEC_RST_GPIO, 0);
		/* switch PHY strap pins into required state */
		ret = mxc_gpio_setup_multiple_pins(mx27_fec_gpios_off,
						ARRAY_SIZE(mx27_fec_gpios_off),
						"FEC");
		if (ret) {
			mxc_gpio_release_multiple_pins(mx27_fec_pwr_gpios,
						ARRAY_SIZE(mx27_fec_pwr_gpios));
			return ret;
		}
		DBG(0, "%s: Delaying for 22ms\n", __FUNCTION__);
		mdelay(22);
		DBG(0, "%s: Deasserting FEC PHY reset\n", __FUNCTION__);
		gpio_set_value(TX27_FEC_RST_GPIO, 1);
		mxc_gpio_release_multiple_pins(mx27_fec_gpios_off,
					ARRAY_SIZE(mx27_fec_gpios_off));
		ret = mxc_gpio_setup_multiple_pins(mx27_fec_gpios_on,
						ARRAY_SIZE(mx27_fec_gpios_on),
						"FEC");
		if (ret) {
			mxc_gpio_release_multiple_pins(mx27_fec_pwr_gpios,
						ARRAY_SIZE(mx27_fec_pwr_gpios));
		}
	}
	return ret;
}

/*
 * Setup GPIO for FEC device to be inactive
 *
 */
void gpio_fec_inactive(void)
{
	mxc_gpio_release_multiple_pins(mx27_fec_gpios_on,
				ARRAY_SIZE(mx27_fec_gpios_on));
	mxc_gpio_setup_multiple_pins(mx27_fec_gpios_off,
				ARRAY_SIZE(mx27_fec_gpios_off),
				"FEC");

	DBG(0, "%s: Asserting FEC PHY reset\n", __FUNCTION__);
	gpio_direction_output(TX27_FEC_RST_GPIO, 0);
	DBG(0, "%s: Configuring FEC PHY for power down mode\n", __FUNCTION__);
	gpio_direction_output(TX27_FEC_PHY_MODE0_GPIO, 0);
	gpio_direction_output(TX27_FEC_PHY_MODE1_GPIO, 1);
	DBG(0, "%s: Releasing FEC PHY reset\n", __FUNCTION__);
	gpio_direction_output(TX27_FEC_RST_GPIO, 1);

	mxc_gpio_release_multiple_pins(mx27_fec_gpios_off,
				ARRAY_SIZE(mx27_fec_gpios_off));
	mxc_gpio_release_multiple_pins(mx27_fec_pwr_gpios,
				ARRAY_SIZE(mx27_fec_pwr_gpios));
}

static struct resource fec_resources[] = {
	{
		.start	= FEC_BASE_ADDR,
		.end	= FEC_BASE_ADDR + 0x18f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= FEC_BASE_ADDR + 0x200,
		.end	= FEC_BASE_ADDR + 0x2e3,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MXC_INT_FEC,
		.end	= MXC_INT_FEC,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef FEC_MII_IRQ
	{
		.start	= FEC_MII_IRQ,
		.end	= FEC_MII_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#endif
};

static struct clk *fec_clk;
static int tx27_fec_suspend(struct platform_device *pdev)
{
	BUG_ON(fec_clk == NULL);
	DBG(1, "%s: Switching FEC PHY off\n", __FUNCTION__);
	gpio_fec_inactive();
	clk_disable(fec_clk);
	return 0;
}

static int tx27_fec_resume(struct platform_device *pdev)
{
	BUG_ON(fec_clk == NULL);
	DBG(1, "%s: Switching FEC PHY on\n", __FUNCTION__);
	clk_enable(fec_clk);
	gpio_fec_active();
	return 0;
}

static int tx27_fec_arch_init(struct platform_device *pdev)
{
	int ret;

	DBG(0, "%s: Activating FEC GPIOs\n", __FUNCTION__);
	ret = gpio_fec_active();
	if (ret) {
		printk(KERN_ERR "%s: could not enable FEC gpios: %d\n", __FUNCTION__, ret);
		return ret;
	}
	BUG_ON(fec_clk != NULL);
	fec_clk = clk_get(&pdev->dev, "fec_clk");
	if (unlikely(IS_ERR(fec_clk))) {
		printk(KERN_ERR "Failed to get fec_clk\n");
		return PTR_ERR(fec_clk);
	}
	DBG(0, "%s: Enabling FEC clock\n", __FUNCTION__);
	clk_enable(fec_clk);
	return 0;
}

static void tx27_fec_arch_exit(struct platform_device *pdev)
{
	BUG_ON(fec_clk == NULL);
	if (unlikely(IS_ERR(fec_clk))) {
		printk(KERN_ERR "Failed to get fec_clk\n");
		return;
	}
	DBG(0, "%s: Disabling FEC clock\n", __FUNCTION__);
	clk_disable(fec_clk);
	clk_put(fec_clk);
	fec_clk = NULL;
	DBG(0, "%s: Deactivating FEC GPIOs\n", __FUNCTION__);
	gpio_fec_inactive();
}

static int tx27_fec_set_mac(struct platform_device *pdev, char *addr)
{
	int i;
	void __iomem *ioaddr;
	unsigned long fec_mac_base = IIM_BASE_ADDR + 0x814;

	ioaddr = ioremap(fec_mac_base, ETH_ALEN * sizeof(long));
	if (ioaddr == NULL) {
		return -ENOMEM;
	}
	dev_info(&pdev->dev, "Copying MAC address from fuse bank %08lx\n", fec_mac_base);
	for (i = 0; i < ETH_ALEN; i++) {
		addr[i] = __raw_readl(ioaddr + i * 4);
	}
	iounmap(ioaddr);
	return 0;
}

static struct fec_enet_platform_data fec_data = {
	.arch_init = tx27_fec_arch_init,
	.arch_exit = tx27_fec_arch_exit,
	.set_mac_addr = tx27_fec_set_mac,
	.suspend = tx27_fec_suspend,
	.resume = tx27_fec_resume,
};

static struct platform_device fec_device = {
	.name		= "fec",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(fec_resources),
	.resource	= fec_resources,
	.dev = {
		.platform_data = &fec_data,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)
static unsigned int tx27_nand_pins[] __initdata = {
	PF0_PF_NFRB,
	PF1_PF_NFCLE,
	PF2_PF_NFWP,
	PF3_PF_NFCE,
	PF4_PF_NFALE,
	PF5_PF_NFRE,
	PF6_PF_NFWE,
};

static struct mxc_nand_platform_data tx27_nand_data = {
	.hw_ecc = 1,
	.width = 1,
};

static struct resource tx27_nand_resources[] = {
	{
		.start	= NFC_BASE_ADDR + 0xe00,
		.end	= NFC_BASE_ADDR + 0xe1f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= NFC_BASE_ADDR,
		.end	= NFC_BASE_ADDR + 0xdff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MXC_INT_NANDFC,
		.end	= MXC_INT_NANDFC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tx27_nand_mtd_device = {
	.name = "mxc_nand",
	.id = 0,
	.num_resources = ARRAY_SIZE(tx27_nand_resources),
	.resource = tx27_nand_resources,
	.dev = {
		.platform_data = &tx27_nand_data,
	},
};

static int __init karo_tx27_nand_init(void)
{
	DBG(0, "%s: Initialising NAND pins\n", __FUNCTION__);
	return mxc_gpio_setup_multiple_pins(tx27_nand_pins,
					ARRAY_SIZE(tx27_nand_pins), "MXC NAND");
}
arch_initcall(karo_tx27_nand_init);
#endif

#if defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT) || defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT_MODULE)
static u64 mxc_emma_dmamask = DMA_BIT_MASK(32);

static struct platform_device tx27_v4l2out_device = {
	.name = "mx27_v4l_output",
	.id = 0,
	.dev = {
		.dma_mask = &mxc_emma_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

#ifdef CONFIG_I2C_IMX_SELECT2
static int mxc_i2c_2_pins[] = {
	PC5_PF_I2C2_SDA,
	PC6_PF_I2C2_SCL,
};

static int karo_tx27_i2c_2_init(struct device *dev)
{
	DBG(0, "%s: Initialising I2C2 pins\n", __FUNCTION__);
	return mxc_gpio_setup_multiple_pins(mxc_i2c_2_pins,
					ARRAY_SIZE(mxc_i2c_2_pins), "I2C2");
}

static void karo_tx27_i2c_2_exit(struct device *dev)
{
	mxc_gpio_release_multiple_pins(mxc_i2c_2_pins,
				ARRAY_SIZE(mxc_i2c_2_pins));
}

static struct imxi2c_platform_data karo_tx27_i2c_2_data = {
	.bitrate = 100000,
	.init = karo_tx27_i2c_2_init,
	.exit = karo_tx27_i2c_2_exit,
};

static struct i2c_board_info karo_i2c_2_boardinfo[] __initdata = {
#if defined(CONFIG_RTC_DRV_DS1307) || defined(CONFIG_RTC_DRV_DS1307_MODULE)
	{
		I2C_BOARD_INFO("ds1339", 0x68 /* DS1339_CHIP_ID */),
		.type = "ds1339",
	},
#endif
#if defined(CONFIG_LP3972) || defined(CONFIG_LP3972_MODULE)
	{
		I2C_BOARD_INFO("lp3972", 0x34),
		.type = "lp3972",
	},
#endif
};

int __init karo_i2c_2_register(void)
{
	int ret;

	ret = i2c_register_board_info(1, karo_i2c_2_boardinfo,
				ARRAY_SIZE(karo_i2c_2_boardinfo));
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C_2 board info: %d\n", ret);
	}
	return ret;
}
device_initcall(karo_i2c_2_register);
#endif /* CONFIG_I2C_IMX_SELECT2 */

static struct platform_dev_list {
	struct platform_device *pdev;
	void *pdata;
} tx27_devices[] __initdata = {
#if defined(CONFIG_I2C_IMX_SELECT2)
	{ .pdev = &mxc_i2c_2_device, .pdata = &karo_tx27_i2c_2_data, },
#endif
#if defined(CONFIG_RTC_DRV_MXC) || defined(CONFIG_RTC_DRV_MXC_MODULE)
	{ .pdev = &mxc_rtc_device, },
#endif
#if defined(CONFIG_MXC_WATCHDOG) || defined(CONFIG_MXC_WATCHDOG_MODULE)
	{ .pdev = &mxc_wdt_device, },
#endif
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)
	{ .pdev = &tx27_nand_mtd_device, },
#endif
#if defined(CONFIG_FEC_PHYLIB) || defined(CONFIG_FEC_PHYLIB_MODULE)
	{ .pdev = &fec_device, },
#endif
#if defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT) || defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT_MODULE)
	{ .pdev = &tx27_v4l2out_device, },
#endif
#if defined(CONFIG_MXC_VPU) || defined(CONFIG_MXC_VPU_MODULE)
	{ .pdev = &mx2_vpu_device, },
#endif
};
#define TX27_NUM_DEVICES		ARRAY_SIZE(tx27_devices)

#define OSC26M_ENABLE_PIN	(GPIO_PORTB | 22)

static int _clk_26m_enable(struct clk *clk)
{
	int ret;

	DBG(0, "%s: Switching 26MHz oscillator on\n", __FUNCTION__);
	ret = gpio_request(OSC26M_ENABLE_PIN, "OSC26m");
	if (ret != 0) {
		printk(KERN_ERR "Failed to request 26MHz oscillator enable GPIO: %d\n", ret);
		return ret;
	}
	mxc_gpio_mode(OSC26M_ENABLE_PIN | GPIO_GPIO | GPIO_OUT | GPIO_DFLT_HIGH);
	return 0;
}

static void _clk_26m_disable(struct clk *clk)
{
	DBG(0, "%s: Switching 26MHz oscillator off\n", __FUNCTION__);
	gpio_set_value(OSC26M_ENABLE_PIN, 0);
	gpio_free(OSC26M_ENABLE_PIN);
}

static struct clk clk_26m = {
	.enable = _clk_26m_enable,
	.disable = _clk_26m_disable,
};

static struct clk_lookup osc26m_clk = {
	.dev_id = NULL,
	.con_id = "clk_26m",
	.clk = &clk_26m,
};

#define MPCTL_PD	(1 - 1)
#define MPCTL_MFI	7
#define MPCTL_MFN	35
#define MPCTL_MFD	(52 - 1)

#if 0
#define SPCTL_PD	(2 - 1)
#define SPCTL_MFI	9
#define SPCTL_MFN	3
#define SPCTL_MFD	(13 - 1)
#else
#define SPCTL_PD	(1 - 1)
#define SPCTL_MFI	14
#define SPCTL_MFN	-2
#define SPCTL_MFD	(13 - 1)
#endif

static inline void karo_tx27_spctl_config(void)
{
	u32 cscr = __raw_readl(CCM_CSCR);
	int loops = 0;
	u32 cscr2;

	if (cscr & CCM_CSCR_SP)
		return;
#if 1
	printk(KERN_INFO "Changing SPCTL0 from %08x to %08x\n",
		__raw_readl(CCM_SPCTL0), CCM_SPCTL0_CPLM |
		CCM_SPCTL0_PD_VAL(2) |
		CCM_SPCTL0_MFI_VAL(11) |
		CCM_SPCTL0_MFN_VAL(-205) |
		CCM_SPCTL0_MFD_VAL(755));

	__raw_writel(CCM_SPCTL0_CPLM |
		CCM_SPCTL0_PD_VAL(2) |
		CCM_SPCTL0_MFI_VAL(11) |
		CCM_SPCTL0_MFN_VAL(-205) |
		CCM_SPCTL0_MFD_VAL(755),
		CCM_SPCTL0);
#endif
	cscr |= CCM_CSCR_SPLLRES;
	DBG(4, "%s: Changing CSCR from %08x to %08x\n", __FUNCTION__,
		__raw_readl(CCM_CSCR), cscr);
	__raw_writel(cscr, CCM_CSCR);

	while ((cscr2 = __raw_readl(CCM_CSCR)) &
		CCM_CSCR_SPLLRES) {
		loops++;
		if (loops >= 1000000)
			break;
	}
	if (cscr2 & CCM_CSCR_SPLLRES) {
		panic("SPLL failed to lock within %u loops\n",
			loops);
		return;
	} else
		DBG(4, "PLLs locked after %d loops: CSCR=%08x(%08x:%08x)\n",
			loops, __raw_readl(CCM_CSCR), cscr2, cscr);
}

#ifdef CONFIG_BASE_CLK_26MHz
static inline void dump_cscr(u32 cscr)
{
	DBG(0, "%s: ARMSRC=%d ARMDIV=%d AHBDIV=%d FPM=%d OSC26M_DIS=%d\n", __FUNCTION__,
		!!(cscr & CCM_CSCR_ARM_SRC),
		(cscr & CCM_CSCR_ARM_MASK) >> CCM_CSCR_ARM_OFFSET,
		(cscr & CCM_CSCR_AHB_MASK) >> CCM_CSCR_AHB_OFFSET,
		!!(cscr & CCM_CSCR_FPM),
		!!(cscr & CCM_CSCR_OSC26M_DIS)
		);
}

static void __init karo_tx27_clock_switch(struct clk *_26m_clk)
{
	int loops = 0;
	u32 cscr = __raw_readl(CCM_CSCR);
	u32 ccsr;

	if (_26m_clk != NULL) {
		dump_cscr(cscr);

		DBG(0, "%s: Enabling 26MHz clock\n", __FUNCTION__);
		clk_enable(_26m_clk);
#if 1
		if (cscr & CCM_CSCR_SP) {
			DBG(0, "%s: Already running on 26MHz clock\n",
				__FUNCTION__);
			return;
		}
#endif
		cscr |= CCM_CSCR_UPDATE_DIS;
		DBG(4, "%s: Changing CSCR from %08x to %08x\n", __FUNCTION__,
			__raw_readl(CCM_CSCR), cscr);
		__raw_writel(cscr, CCM_CSCR);
		cscr &= ~CCM_CSCR_UPDATE_DIS;
#if 1
		DBG(4, "Changing MPCTL0 from %08x to %08x\n",
			__raw_readl(CCM_MPCTL0), CCM_MPCTL0_CPLM |
			CCM_MPCTL0_PD_VAL(MPCTL_PD) |
			CCM_MPCTL0_MFI_VAL(MPCTL_MFI) |
			CCM_MPCTL0_MFN_VAL(MPCTL_MFN) |
			CCM_MPCTL0_MFD_VAL(MPCTL_MFD));
		__raw_writel(CCM_MPCTL0_CPLM |
			CCM_MPCTL0_PD_VAL(MPCTL_PD) |
			CCM_MPCTL0_MFI_VAL(MPCTL_MFI) |
			CCM_MPCTL0_MFN_VAL(MPCTL_MFN) |
			CCM_MPCTL0_MFD_VAL(MPCTL_MFD),
			CCM_MPCTL0);
#endif
#if 1
		DBG(4, "Changing SPCTL0 from %08x to %08x\n",
			__raw_readl(CCM_SPCTL0), CCM_SPCTL0_CPLM |
			CCM_SPCTL0_PD_VAL(SPCTL_PD) |
			CCM_SPCTL0_MFI_VAL(SPCTL_MFI) |
			CCM_SPCTL0_MFN_VAL(SPCTL_MFN) |
			CCM_SPCTL0_MFD_VAL(SPCTL_MFD));
		__raw_writel(CCM_SPCTL0_CPLM |
			CCM_SPCTL0_PD_VAL(SPCTL_PD) |
			CCM_SPCTL0_MFI_VAL(SPCTL_MFI) |
			CCM_SPCTL0_MFN_VAL(SPCTL_MFN) |
			CCM_SPCTL0_MFD_VAL(SPCTL_MFD),
			CCM_SPCTL0);
#endif
		cscr |= CCM_CSCR_MPLLRES | CCM_CSCR_SPLLRES;
		DBG(4, "%s: Changing CSCR from %08x to %08x\n", __FUNCTION__,
			__raw_readl(CCM_CSCR), cscr);
		__raw_writel(cscr, CCM_CSCR);

		while ((cscr = __raw_readl(CCM_CSCR)) &
			(CCM_CSCR_MPLLRES | CCM_CSCR_SPLLRES)) {
			loops++;
			if (loops >= 10000)
				break;
			udelay(1);
		}
		if (cscr & (CCM_CSCR_MPLLRES | CCM_CSCR_SPLLRES)) {
			panic("PLLs failed to lock within %u loops\n",
				loops);
			return;
		} else
			DBG(2, "PLLs locked after %d loops: CSCR=%08x(%08x)\n",
				loops, __raw_readl(CCM_CSCR), cscr);

		cscr |= CCM_CSCR_MCU | CCM_CSCR_SP;
		DBG(4, "%s: Changing CSCR from %08x to %08x\n", __FUNCTION__,
			__raw_readl(CCM_CSCR), cscr);
		__raw_writel(cscr, CCM_CSCR);

		DBG(9, "%s: Enabling bypass for FPM, DPLL and OSC26M\n", __FUNCTION__);
		ccsr = __raw_readl(CCM_CCSR);
		__raw_writel(ccsr & ~0x300, CCM_CCSR);
		DBG(9, "changing CCSR from %08x to %08x(%08x)\n",
			ccsr, ccsr & ~0x300, __raw_readl(CCM_CCSR));

		DBG(9, "%s: Disabling FPM, MPLL and SPLL\n", __FUNCTION__);
		cscr &= ~CCM_CSCR_FPM;
		cscr &= ~CCM_CSCR_MPEN;
		cscr &= ~CCM_CSCR_SPEN;
#if 0
		cscr |= CCM_CSCR_OSC26M_DIS;
#endif
		DBG(9, "changing CSCR from %08x to %08x\n",
			__raw_readl(CCM_CSCR), cscr);
		__raw_writel(cscr, CCM_CSCR);
		dump_cscr(cscr);
	}
}
#else
static inline void karo_tx27_clock_switch(struct clk *_26m_clk)
{
}
#endif

static int __initdata osc26m_off;

static int __init karo_tx27_osc26m_setup(char *cmdline)
{
	int ret;

	ret = get_option(&cmdline, &osc26m_off);
	switch (ret) {
	case 0:
		DBG(0, "%s: No argument for osc26m_off\n", __FUNCTION__);
		break;
	case 1:
	case 2:
	case 3:
		break;
	}
	DBG(0, "%s: osc26m_off=%d\n", __FUNCTION__, osc26m_off);

	return 0;
}
__setup("osc26m_off=", karo_tx27_osc26m_setup);

static void __init karo_tx27_clock_init(void)
{
	struct clk *cpu_clk;
	struct clk *_26m_clk = NULL;

	if (!osc26m_off) {
		clkdev_add(&osc26m_clk);

		_26m_clk = clk_get_sys(NULL, "clk_26m");
		if (IS_ERR(_26m_clk)) {
			printk(KERN_ERR "Cannot request 26MHz clock: %ld\n", PTR_ERR(_26m_clk));
			_26m_clk = NULL;
		}
	}
	karo_tx27_clock_switch(_26m_clk);
	karo_tx27_spctl_config();
	mx27_clocks_init(26000000);

	cpu_clk = clk_get_sys(NULL, "cpu");
	if (!IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: Setting CPU clock to 400MHz\n", __FUNCTION__);
		if (clk_set_rate(cpu_clk, 399000000) != 0) {
			printk(KERN_ERR "Failed to set CPU clock rate cpu clock is: %lu.%03luMHz\n",
				clk_get_rate(cpu_clk) / 1000000,
				clk_get_rate(cpu_clk) / 1000 % 1000);
		}
	} else {
		printk(KERN_ERR "Failed to get CPU clock: %ld\n",
			PTR_ERR(cpu_clk));
	}
}

/*
 * preconfigure OCR for all unused GPIOs, so that GPIOs
 * configured as outputs via gpiolib can work.
 * Note, that reconfiguring a pin lateron may disturb this configuration!
 */
static int __init karo_tx27_gpio_init(void)
{
	int i;
	void __iomem *gpio_base = MX2_IO_ADDRESS(GPIO_BASE_ADDR);

	for (i = 0; i < 6 * 32; i++) {
		int ret;
		int pin = i % 32;
		int port = i / 32;
		void __iomem *reg = gpio_base +
			((pin < 16) ? MXC_OCR1(port) : MXC_OCR2(port));

		ret = gpio_request(i, NULL);
		if (ret == 0) {
			DBG(0, "Configuring P%c%d (GPIO%d) OCR\n",
				port + 'A', pin, i);
			ret = __raw_readl(reg);
			ret |= (3 << ((pin % 16) * 2));
			__raw_writel(ret, reg);

			ret = __raw_readl(gpio_base + MXC_GIUS(port));
			ret |= 1 << pin;
			__raw_writel(ret, gpio_base + MXC_GIUS(port));
			gpio_free(i);
		} else {
			DBG(0, "Leaving P%c%d (GPIO%d) OCR=%d\n", port + 'A', pin, i,
				(__raw_readl(reg) >> 2 * (pin % 16)) & 3);
		}
	}
	return 0;
}
late_initcall(karo_tx27_gpio_init);

#define MX27_UUID_ADDR		0x10028C04UL
#define MX27_UUID_LEN		6

static void __init karo_tx27_set_system_serial(void)
{
	void __iomem *mx27_serial_addr = ioremap(MX27_UUID_ADDR, MX27_UUID_LEN);

	if (mx27_serial_addr != NULL) {
		int i, n;
		unsigned int __iomem *p = mx27_serial_addr;

		for (i = n = 0; i < sizeof(system_serial_low) &&
			     n < MX27_UUID_LEN; i++, n++, p++) {
			system_serial_low |= readl(p) << (i * 8);
		}
		for (i = 0; i < sizeof(system_serial_high) &&
			     n < MX27_UUID_LEN; i++, n++, p++) {
			system_serial_high |= readl(p) << (i * 8);
		}
		iounmap(mx27_serial_addr);
	} else {
		printk(KERN_WARNING "Failed to map MX27_UUID_ADDR; cannot set system serial number\n");
	}
}

static void __init karo_tx27_board_init(void)
{
	int i;

	DBG(0, "%s: \n", __FUNCTION__);

	karo_tx27_set_system_serial();

	for (i = 0; i < ARRAY_SIZE(tx27_uart_devices); i++) {
		int ret;

		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, tx27_uart_devices[i],
			&tx27_uart_devices[i]->dev, tx27_uart_devices[i]->name);
		ret = mxc_register_device(tx27_uart_devices[i],
					&tx27_uart_port);
		if (ret != 0) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, tx27_uart_devices[i]->name, ret);
		}
	}

	/* enable SSI3_INT (PC23) for IRQ probing */
	set_irq_flags(gpio_to_irq(GPIO_PORTC | 23), IRQF_VALID | IRQF_PROBE);

	for (i = 0; i < TX27_NUM_DEVICES; i++) {
		int ret;

		if (tx27_devices[i].pdev == NULL)
			continue;
		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
			__FUNCTION__, i, tx27_devices[i].pdev, &tx27_devices[i].pdev->dev,
			tx27_devices[i].pdev->name);
		if (tx27_devices[i].pdata == NULL) {
			ret = platform_device_register(tx27_devices[i].pdev);
		} else {
			ret = mxc_register_device(tx27_devices[i].pdev,
				tx27_devices[i].pdata);
		}
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
				__FUNCTION__, i, tx27_devices[i].pdev->name, ret);
		}
	}
	device_init_wakeup(&mxc_rtc_device.dev, 1);

	DBG(0, "%s: Done\n", __FUNCTION__);
}

static void __init karo_tx27_map_io(void)
{
	mx27_map_io();
}

static void __init karo_tx27_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static void __init karo_tx27_timer_init(void)
{
	DBG(0, "%s: \n", __FUNCTION__);
	karo_tx27_clock_init();
	DBG(0, "%s: Done\n", __FUNCTION__);
}

struct sys_timer karo_tx27_timer = {
	.init	= karo_tx27_timer_init,
};

MACHINE_START(TX27, "Ka-Ro electronics TX27 module (Freescale i.MX27)")
	/* Maintainer: <LW@KARO-electronics.de> */
	.phys_io	= AIPI_BASE_ADDR,
	.io_pg_offst	= ((unsigned long)AIPI_BASE_ADDR_VIRT >> 18) & 0xfffc,
	.fixup		= karo_tx27_fixup,
	.map_io		= karo_tx27_map_io,
	.init_irq	= mx27_init_irq,
	.init_machine	= karo_tx27_board_init,
	.timer		= &karo_tx27_timer,
MACHINE_END
