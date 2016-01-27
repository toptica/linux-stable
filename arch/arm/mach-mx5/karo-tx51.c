/*
 * Copyright (C) 2009  Lothar Wassmann <LW@KARO-electronics.de>
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/fec_enet.h>
#include <linux/if_ether.h>
#include <linux/io.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/common.h>
#include <mach/gpio.h>
#include <mach/imx-uart.h>
#include <mach/mxc_nand.h>
#include <mach/iomux-mx51.h>

#include "devices.h"
#include "crm_regs.h"
#include "karo.h"

#ifdef DEBUG
int tx51_debug = 1;
module_param(tx51_debug, int, S_IWUSR | S_IRUGO);
#endif

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE)
static struct mx51_pad_desc karo_tx51_nand_pads[] = {
};

static struct mxc_nand_platform_data tx51_nand_data = {
	.hw_ecc = 1,
	.width = 1,
};

static int tx51_nand_init(void)
{
	int ret;

	DBG(0, "%s: Configuring NAND pins\n", __FUNCTION__);
	ret = mx51_iomux_request_pads(karo_tx51_nand_pads,
				ARRAY_SIZE(karo_tx51_nand_pads));
	if (ret) {
		return ret;
	}
	return 0;
}
arch_initcall(tx51_nand_init);

static struct resource tx51_nand_resources[] = {
	{
		/* NFC AXI register space */
		.start	= NFC_AXI_BASE_ADDR + 0x1e00,
		.end	= NFC_AXI_BASE_ADDR + 0x1e3f,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* NFC buffer space */
		.start	= NFC_AXI_BASE_ADDR,
		.end	= NFC_AXI_BASE_ADDR + 0x11ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		/* NFC IP register space */
		.start	= NFC_BASE_ADDR,
		.end	= NFC_BASE_ADDR + 0x1f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MXC_INT_NFC,
		.end	= MXC_INT_NFC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tx51_nand_device = {
	.name = "mxc_nand",
	.id = -1,
	.num_resources = ARRAY_SIZE(tx51_nand_resources),
	.resource = tx51_nand_resources,
	.dev = {
		.platform_data = &tx51_nand_data,
	},
};
#endif

#if defined(CONFIG_FEC_PHYLIB) || defined(CONFIG_FEC_PHYLIB_MODULE)
#define FEC_MII_IRQ	IOMUX_TO_IRQ(MX51_PIN_NANDF_CS2)

static struct resource tx51_fec_resources[] = {
	{
		.start	= FEC_BASE_ADDR,
		.end	= FEC_BASE_ADDR + 0x18f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= FEC_BASE_ADDR + 0x200,
		.end	= FEC_BASE_ADDR + 0x30b,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MXC_INT_FEC,
		.end	= MXC_INT_FEC,
		.flags	= IORESOURCE_IRQ,
#ifdef FEC_MII_IRQ
	}, {
		.start	= FEC_MII_IRQ,
		.end	= FEC_MII_IRQ,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	},
};

static int __init fec_resource_init(void)
{
	mx51_fec_device.resource = tx51_fec_resources;
	mx51_fec_device.num_resources = ARRAY_SIZE(tx51_fec_resources);
#ifdef FEC_MII_IRQ
	mx51_request_iomux(MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT3);
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS2));
	mx51_free_iomux(MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT3);
#endif
	return 0;
}
postcore_initcall(fec_resource_init);

static struct mx51_pad_desc tx51_fec_pads_off[] = {
	MX51_PIN_NANDF_CS2__GPIO3_18,
	MX51_PIN_NANDF_CS3__GPIO3_19,
	MX51_PIN_EIM_EB2__GPIO2_22,
	MX51_PIN_NANDF_RB3__GPIO3_11,
	MX51_PIN_NANDF_D11__GPIO3_29,
	MX51_PIN_NANDF_D9__GPIO3_31,
	MX51_PIN_EIM_EB3__GPIO2_23,
	MX51_PIN_EIM_CS2__GPIO2_27,
	MX51_PIN_EIM_CS3__GPIO2_28,
	MX51_PIN_EIM_CS4__GPIO2_29,
	MX51_PIN_NANDF_RDY_INT__GPIO3_24,
	MX51_PIN_NANDF_CS7__GPIO3_23,
	MX51_PIN_NANDF_D8__GPIO4_0,
	MX51_PIN_NANDF_CS4__GPIO3_20,
	MX51_PIN_NANDF_CS5__GPIO3_21,
	MX51_PIN_NANDF_CS6__GPIO3_22,
	MX51_PIN_NANDF_RB2__GPIO3_10,
	MX51_PIN_EIM_CS5__GPIO2_30,
};

static struct mx51_pad_desc tx51_fec_pwr_gpios[] = {
	MX51_PIN_GPIO1_3__GPIO1_3,	/* PHY Power */
	MX51_PIN_EIM_A20__GPIO2_14,	/* PHY RESET */
};

static struct mx51_pad_desc tx51_fec_pads_on[] = {
	MX51_PIN_NANDF_CS2__GPIO3_18,
	MX51_PIN_NANDF_RDY_INT__FEC_TX_CLK,
	MX51_PIN_NANDF_CS3__FEC_MDC,
	MX51_PIN_NANDF_CS4__FEC_TDATA1,
	MX51_PIN_NANDF_CS5__FEC_TDATA2,
	MX51_PIN_NANDF_CS6__FEC_TDATA3,
	MX51_PIN_NANDF_CS7__FEC_TX_EN,
	MX51_PIN_NANDF_RB2__FEC_COL,
	MX51_PIN_NANDF_RB3__FEC_RX_CLK,
	MX51_PIN_NANDF_D8__FEC_TDATA0,
	MX51_PIN_NANDF_D9__FEC__RDATA0,
	MX51_PIN_NANDF_D11__FEC_RX_DV,
	MX51_PIN_EIM_EB2__FEC_MDIO,
	MX51_PIN_EIM_EB3__FEC_RDATA1,
	MX51_PIN_EIM_CS2__FEC_RDATA2,
	MX51_PIN_EIM_CS3__FEC_RDATA3,
	MX51_PIN_EIM_CS4__FEC_RX_ER,
	MX51_PIN_EIM_CS5__FEC_CRS,
};

static unsigned int tx51_fec_strap_gpios[] = {
	IOMUX_TO_GPIO(MX51_PIN_GPIO1_3),
	IOMUX_TO_GPIO(MX51_PIN_NANDF_D9),
	IOMUX_TO_GPIO(MX51_PIN_EIM_EB3),
	IOMUX_TO_GPIO(MX51_PIN_EIM_CS2),
	IOMUX_TO_GPIO(MX51_PIN_EIM_CS3),
};

#define FEC_PWR_GPIO		IOMUX_TO_GPIO(tx51_fec_pwr_gpios[0].pad)
#define FEC_RST_GPIO		IOMUX_TO_GPIO(tx51_fec_pwr_gpios[1].pad)

static void tx51_fec_inactive(void)
{
	mx51_iomux_release_pads(tx51_fec_pads_on,
				ARRAY_SIZE(tx51_fec_pads_on));
	/* Configure PHY pins as GPIO */
	mx51_iomux_request_pads(tx51_fec_pads_off,
				ARRAY_SIZE(tx51_fec_pads_off));

	gpio_set_value(FEC_RST_GPIO, 0);
	gpio_set_value(FEC_PWR_GPIO, 0);

	mx51_iomux_release_pads(tx51_fec_pads_off,
				ARRAY_SIZE(tx51_fec_pads_off));
	mx51_iomux_release_pads(tx51_fec_pwr_gpios,
				ARRAY_SIZE(tx51_fec_pwr_gpios));
}

static int tx51_fec_active(void)
{
	int ret;

	ret = mx51_iomux_request_pads(tx51_fec_pwr_gpios,
				ARRAY_SIZE(tx51_fec_pwr_gpios));
	if (ret) {
		printk(KERN_ERR "%s: Failed to request FEC pads: %d\n",
			__FUNCTION__, ret);
		return ret;
	}
	if (!gpio_get_value(FEC_PWR_GPIO) ||
		!gpio_get_value(FEC_RST_GPIO)) {
		int i;

		DBG(0, "%s: Asserting PHY RESET\n", __FUNCTION__);
		/* make sure, PHY reset is asserted */
		gpio_direction_output(FEC_RST_GPIO, 0);

		ret = mx51_iomux_request_pads(tx51_fec_pads_off,
					ARRAY_SIZE(tx51_fec_pads_off));
		if (ret) {
			DBG(0, "%s: Failed to request pads: %d\n", __FUNCTION__, ret);
			goto release_gpio;
		}
		for (i = 0; i < ARRAY_SIZE(tx51_fec_pads_off); i++) {
			DBG(0, "%s: Deasserting GPIO%d_%d\n", __FUNCTION__,
				IOMUX_TO_GPIO(tx51_fec_pads_off[i].pad) / 32,
				IOMUX_TO_GPIO(tx51_fec_pads_off[i].pad) % 32);
			gpio_direction_input(
				IOMUX_TO_GPIO(tx51_fec_pads_off[i].pad));
		}
		for (i = 0; i < ARRAY_SIZE(tx51_fec_strap_gpios); i++) {
			DBG(0, "%s: Asserting GPIO%d_%d\n", __FUNCTION__,
				tx51_fec_strap_gpios[i] / 32,
				tx51_fec_strap_gpios[i] % 32);
			gpio_set_value(tx51_fec_strap_gpios[i], 1);
		}
		mdelay(22);

		/* make sure, PHY power is switched on */
		DBG(0, "%s: Switching PHY power on\n", __FUNCTION__);
		gpio_direction_output(FEC_PWR_GPIO, 1);

		/* deassert PHY RESET */
		DBG(0, "%s: Deasserting PHY RESET\n", __FUNCTION__);
		gpio_direction_output(FEC_RST_GPIO, 1);
		mx51_iomux_release_pads(tx51_fec_pads_off,
					ARRAY_SIZE(tx51_fec_pads_off));
	}
	/* PHY is already powered on. Assume it has been configured
	 * correctly by bootloader
	 */
	DBG(0, "%s: Configuring FEC pads\n", __FUNCTION__);
	ret = mx51_iomux_request_pads(tx51_fec_pads_on,
				ARRAY_SIZE(tx51_fec_pads_on));
	if (ret) {
		DBG(0, "%s: Failed to request pads: %d\n", __FUNCTION__, ret);
		goto release_gpio;
	}
#ifdef FEC_MII_IRQ
	gpio_direction_input(irq_to_gpio(FEC_MII_IRQ));
#endif
	/* setup pad_ctrl for NANDF_CS2 (GPIO3_18) */
	//__raw_writel(0x20f0, MX5_IO_ADDRESS(0x73fa8520));
	//__raw_writel(0x13, MX5_IO_ADDRESS(0x73fa8138));
	return ret;
release_gpio:
	mx51_iomux_release_pads(tx51_fec_pwr_gpios,
				ARRAY_SIZE(tx51_fec_pwr_gpios));
	return ret;
}

static struct clk *tx51_fec_clk;

static int fec_arch_init(struct platform_device *pdev)
{
	int ret;

	ret = tx51_fec_active();
	if (ret)
		return ret;

	BUG_ON(tx51_fec_clk != NULL);
	tx51_fec_clk = clk_get(&pdev->dev, "fec");
	if (IS_ERR(tx51_fec_clk)) {
		ret = PTR_ERR(tx51_fec_clk);
		tx51_fec_clk = NULL;
		DBG(0, "%s: Failed to get FEC clock: %d\n", __FUNCTION__, ret);
		return ret;
	}

	ret = clk_enable(tx51_fec_clk);
	if (ret)
		tx51_fec_inactive();
	return ret;
}

static void fec_arch_exit(struct platform_device *pdev)
{
	BUG_ON(!tx51_fec_clk);
	clk_disable(tx51_fec_clk);
	clk_put(tx51_fec_clk);
	tx51_fec_inactive();
}

static int fec_set_mac(struct platform_device *pdev, char *addr)
{
	int ret;
	int i;
	void __iomem *ioaddr;
	unsigned long fec_mac_base = 0;
	struct clk *iim_clk;

	iim_clk = clk_get(NULL, "iim");
	if (IS_ERR(iim_clk)) {
		printk(KERN_ERR "%s: Failed to get IIM clock\n", __FUNCTION__);
		return PTR_ERR(iim_clk);
	}
	ret = clk_enable(iim_clk);
	if (ret) {
		printk(KERN_ERR "%s: Failed to enable IIM clock: %d\n",
			__FUNCTION__, ret);
		goto put_clk;
	}
	fec_mac_base = IIM_BASE_ADDR + 0xc24;
	ioaddr = ioremap(fec_mac_base, ETH_ALEN * sizeof(long));
	if (ioaddr == NULL) {
		ret = -ENOMEM;
		goto disable_clk;
	}
	dev_info(&pdev->dev, "Copying MAC address from fuse bank %08lx\n",
		fec_mac_base);
	for (i = 0; i < ETH_ALEN; i++) {
		addr[ETH_ALEN - i - 1] = __raw_readl(ioaddr + i * 4);
	}
	iounmap(ioaddr);
disable_clk:
	clk_disable(iim_clk);
put_clk:
	clk_put(iim_clk);
	return ret;
}

#ifdef CONFIG_PM
static int tx51_fec_suspend(struct platform_device *pdev)
{
	BUG_ON(tx51_fec_clk == NULL);
	DBG(-1, "%s: Switching FEC PHY off\n", __FUNCTION__);
	tx51_fec_inactive();
	clk_disable(tx51_fec_clk);
	return 0;
}

static int tx51_fec_resume(struct platform_device *pdev)
{
	BUG_ON(tx51_fec_clk == NULL);
	DBG(-1, "%s: Switching FEC PHY on\n", __FUNCTION__);
	clk_enable(tx51_fec_clk);
	tx51_fec_active();
	return 0;
}
#else
#define tx51_fec_suspend	NULL
#define tx51_fec_resume		NULL
#endif

static struct fec_enet_platform_data tx51_fec_pdata = {
	.arch_init = fec_arch_init,
	.arch_exit = fec_arch_exit,
	.set_mac_addr = fec_set_mac,
	.suspend = tx51_fec_suspend,
	.resume = tx51_fec_resume,
};
#endif

static int __init mxc_init_srpgconfig(void)
{
	struct clk *gpcclk = clk_get(NULL, "gpc_dvfs");

	if (IS_ERR(gpcclk)) {
		printk(KERN_ERR "%s: Failed to get gpc_dvfs clock: %ld\n",
			__FUNCTION__, PTR_ERR(gpcclk));
		return PTR_ERR(gpcclk);
	}
	clk_enable(gpcclk);

	/* Setup the number of clock cycles to wait for SRPG
	 * power up and power down requests.
	 */
	__raw_writel(0x010F0201, MXC_SRPG_ARM_PUPSCR);
	__raw_writel(0x010F0201, MXC_SRPG_NEON_PUPSCR);
	__raw_writel(0x00000008, MXC_SRPG_EMPGC0_PUPSCR);
	__raw_writel(0x00000008, MXC_SRPG_EMPGC1_PUPSCR);

	__raw_writel(0x01010101, MXC_SRPG_ARM_PDNSCR);
	__raw_writel(0x01010101, MXC_SRPG_NEON_PDNSCR);
	__raw_writel(0x00000018, MXC_SRPG_EMPGC0_PDNSCR);
	__raw_writel(0x00000018, MXC_SRPG_EMPGC1_PDNSCR);

	clk_disable(gpcclk);
	clk_put(gpcclk);

	return 0;
}

static struct imxuart_platform_data tx51_uart_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

#ifdef CONFIG_I2C_IMX_SELECT3
static struct mx51_pad_desc tx51_i2c_gpio_pads[] = {
	MX51_PIN_I2C1_CLK__SCL,
	MX51_PIN_I2C1_DAT__SDA,
};
#else
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
static struct i2c_gpio_platform_data tx51_i2c_gpio_pdata = {
	.scl_pin = IOMUX_TO_GPIO(MX51_PIN_I2C1_CLK),
	.sda_pin = IOMUX_TO_GPIO(MX51_PIN_I2C1_DAT),
	.sda_is_open_drain = 0,
};

static struct platform_device tx51_i2c_gpio_device = {
	.name = "i2c-gpio",
	.id = 0,
	.dev = {
		.platform_data = &tx51_i2c_gpio_pdata,
	},
};

static struct mx51_pad_desc tx51_i2c_gpio_pads[] = {
	MX51_PIN_I2C1_CLK__GPIO3_16,
	MX51_PIN_I2C1_DAT__GPIO3_17,
};
#endif // CONFIG_I2C_GPIO || CONFIG_I2C_GPIO_MODULE
#endif // !CONFIG_I2C_IMX_SELECT3

#if defined(CONFIG_I2C_IMX_SELECT3) ||				\
	defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
static int __init tx51_i2c_gpio_init(void)
{
	int ret;

	ret = mx51_iomux_request_pads(tx51_i2c_gpio_pads,
				ARRAY_SIZE(tx51_i2c_gpio_pads));
	if (ret) {
		return ret;
	}
	mx51_iomux_release_pads(tx51_i2c_gpio_pads,
				ARRAY_SIZE(tx51_i2c_gpio_pads));
	return ret;
}
arch_initcall(tx51_i2c_gpio_init);
#endif

static struct i2c_board_info __initdata tx51_i2c_boardinfo[] = {
#if defined(CONFIG_RTC_DRV_DS1307) || defined(CONFIG_RTC_DRV_DS1307_MODULE)
	{
		I2C_BOARD_INFO("rtc-ds1307", 0x68),
		.type = "ds1339",
	},
#endif
};

static struct {
	struct platform_device *pdev;
	void *pdata;
} tx51_devices[] = {
	{ .pdev = &mx51_uart_device1, .pdata = &tx51_uart_pdata, },
	{ .pdev = &mx51_uart_device2, .pdata = &tx51_uart_pdata, },
	{ .pdev = &mx51_uart_device3, .pdata = &tx51_uart_pdata, },
#if defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE)
	{ .pdev = &tx51_nand_device, },
#endif
#if defined(CONFIG_RTC_DRV_MXC_SRTC) || defined(CONFIG_RTC_DRV_MXC_SRTC_MODULE)
	{ .pdev = &mx51_srtc_device, },
#endif
#if defined(CONFIG_MXC_WATCHDOG) || defined(CONFIG_MXC_WATCHDOG_MODULE)
	{ .pdev = &mx51_wdt_device, },
#endif
#if defined(CONFIG_FEC_PHYLIB) || defined(CONFIG_FEC_PHYLIB_MODULE)
	{ .pdev = &mx51_fec_device, .pdata = &tx51_fec_pdata, },
#endif
#ifndef CONFIG_I2C_IMX_SELECT3
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	{ .pdev = &tx51_i2c_gpio_device, },
#endif
#endif
#if defined(CONFIG_MXC_VPU) || defined(CONFIG_MXC_VPU_MODULE)
	{ .pdev = &mx51_vpu_device, },
#endif
};

static struct {
	struct mx51_pad_desc desc;
	int pad_ctl;
	int inp_sel;
	unsigned int dir:1,
		data:1;
} tx51_pinmux[] __initdata = {
	{ MX51_PIN_GPIO1_7__GPIO1_7, -1, -1, 1, 0, }, /* USB PHY clock oscillator */
	{ MX51_PIN_GPIO1_6__GPIO1_6, -1, -1, 0, -1, }, /* USBH1 overcurrent */
	{ MX51_PIN_GPIO1_4__GPIO1_4, -1, -1, 1, 0, }, /* USB PHY RESET */
};

static int __init tx51_iomux_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(tx51_pinmux); i++) {
		ret = mx51_request_iomux(tx51_pinmux[i].desc.pad,
					tx51_pinmux[i].desc.config);
		if (ret == 0) {
			int gpio = IOMUX_TO_GPIO(tx51_pinmux[i].desc.pad);

			if (tx51_pinmux[i].pad_ctl >= 0) {
				mx51_iomux_set_pad(tx51_pinmux[i].desc.pad,
					tx51_pinmux[i].pad_ctl);
			}
			if (tx51_pinmux[i].inp_sel >= 0) {
				mx51_iomux_set_input(tx51_pinmux[i].desc.pad,
					tx51_pinmux[i].inp_sel);
			}
			if (gpio >= 0) {
				if (tx51_pinmux[i].dir) {
					gpio_direction_output(gpio,
							tx51_pinmux[i].data);
				} else {
					gpio_direction_input(gpio);
				}
			}
			mx51_free_iomux(tx51_pinmux[i].desc.pad,
					tx51_pinmux[i].desc.config);
		} else {
			printk(KERN_ERR "%s: Failed to request pad[%d] %08x\n",
				__FUNCTION__, i, tx51_pinmux[i].desc.pad);
		}
	}
	return 0;
}

#define MX51_UUID_ADDR		(IIM_BASE_ADDR + 0x820)
#define MX51_UUID_LEN		8

static void __init karo_tx51_set_system_serial(void)
{
	void __iomem *mx51_serial_addr = ioremap(MX51_UUID_ADDR, MX51_UUID_LEN);
	struct clk *iim_clk;

	if (mx51_serial_addr == NULL) {
		printk(KERN_WARNING "Failed to map MX51_UUID_ADDR; cannot set system serial number\n");
		return;
	}

	iim_clk = clk_get(NULL, "iim");
	if (IS_ERR(iim_clk)) {
		printk(KERN_ERR "%s: Failed to get IIM clock: %ld\n", __FUNCTION__,
			PTR_ERR(iim_clk));
		iounmap(mx51_serial_addr);
		return;
	}

	if (clk_enable(iim_clk) == 0) {
		int i, n;
		unsigned int __iomem *p = mx51_serial_addr;

		for (i = n = 0; i < sizeof(system_serial_low) &&
			     n < MX51_UUID_LEN; i++, n++, p++) {
			system_serial_low |= readl(p) << (i * 8);
		}
		for (i = 0; i < sizeof(system_serial_high) &&
			     n < MX51_UUID_LEN; i++, n++, p++) {
			system_serial_high |= readl(p) << (i * 8);
		}
	} else {
		printk(KERN_ERR "Failed to enable IIM clock\n");
	}
	clk_disable(iim_clk);
	clk_put(iim_clk);
	iounmap(mx51_serial_addr);
}

/*!
 * Board specific initialization.
 */
static void __init tx51_board_init(void)
{
	int i;
	int ret;

	karo_tx51_set_system_serial();

	tx51_iomux_init();

	for (i = 0; i < ARRAY_SIZE(tx51_devices); i++) {
		DBG(0, "%s: Registering device[%d] %s\n", __FUNCTION__,
			i, tx51_devices[i].pdev->name);

		if (tx51_devices[i].pdata) {
			ret = mxc_register_device(tx51_devices[i].pdev,
						tx51_devices[i].pdata);
		} else {
			ret = platform_device_register(tx51_devices[i].pdev);
		}
		if (ret) {
			printk(KERN_ERR "%s: Failed to register device[%d] %s\n",
				__FUNCTION__, i, tx51_devices[i].pdev->name);
		}
	}
	mxc_init_srpgconfig();

	i2c_register_board_info(0, tx51_i2c_boardinfo,
				ARRAY_SIZE(tx51_i2c_boardinfo));
}

static void __init tx51_init_irq(void)
{
	mxc_init_irq(TZIC_BASE_ADDR_VIRT);
}

static void __init tx51_timer_init(void)
{
	mx51_clocks_init(32768, 24000000, 0, 0);
}

static void __init tx51_map_io(void)
{
	mx51_map_io();
}

static struct sys_timer tx51_timer = {
	.init	= tx51_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_TX51 data structure.
 */
MACHINE_START(TX51, "Ka-Ro TX51 module")
	/* Maintainer: Ka-Ro electronics GmbH */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((unsigned long)AIPS1_BASE_ADDR_VIRT >> 18) & 0xfffc,
	.map_io = tx51_map_io,
	.init_irq = tx51_init_irq,
	.init_machine = tx51_board_init,
	.timer = &tx51_timer,
MACHINE_END
