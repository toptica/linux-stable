/*
 * arch/arm/mach-mx2/karo-tx25.c
 *
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
 *
 * This file adds support for the Ka-Ro electronics TX25 processor modules
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/serial_8250.h>
#include <linux/fec_enet.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <mtd/mtd-abi.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#endif

#include <linux/serial.h>
#include <linux/fsl_devices.h>
#include <linux/irq.h>
#include <linux/mmc/host.h>
#include <linux/leds.h>
#include <linux/if_ether.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include <mach/iomux.h>
#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/imxfb.h>
#include <mach/mmc.h>
#include <mach/imx-uart.h>
#include <mach/mxc_nand.h>
#include <mach/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/at24.h>
#if defined(CONFIG_TOUCHSCREEN_MXC_TSC) || defined(CONFIG_TOUCHSCREEN_MXC_TSC_MODULE)
#include <mach/mxc_tsc.h>
#endif
#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)
#include <mach/mxc_can.h>
#endif

#include "devices.h"
#include "karo.h"

#ifdef DEBUG
int tx25_debug = 1;
module_param(tx25_debug, int, S_IRUGO | S_IWUSR);
#endif

int karo_board_type = 0;
int karo_mod_type = -1;

static void karo_tx25_gpio_config(struct pad_desc *pd, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		if (mxc_iomux_v3_setup_pad(&pd[i]) == 0) {
			mxc_iomux_v3_release_pad(&pd[i]);
		}
	}
}

#define MX25_INT_FEC	57
#define FEC_MII_IRQ	IRQ_GPIOD(8)

#if defined(CONFIG_FEC_PHYLIB) || defined(CONFIG_FEC_PHYLIB_MODULE)
static struct resource fec_resources[] = {
	{
		.start	= FEC_BASE_ADDR,
		.end	= FEC_BASE_ADDR + 0x18f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= FEC_BASE_ADDR + 0x200,
		.end	= FEC_BASE_ADDR + 0x30b,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MX25_INT_FEC,
		.end	= MX25_INT_FEC,
		.flags	= IORESOURCE_IRQ,
#ifdef FEC_MII_IRQ
	}, {
		.start	= FEC_MII_IRQ,
		.end	= FEC_MII_IRQ,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	},
};

/*
 * Setup GPIO for FEC device to be active
 *
 */
static struct pad_desc karo_tx25_fec_gpios_off[] = {
	MX25_PAD_FEC_MDC__GPIO_3_5,
	MX25_PAD_FEC_MDIO__GPIO_3_6,
	MX25_PAD_FEC_TDATA0__GPIO_3_7,
	MX25_PAD_FEC_TDATA1__GPIO_3_8,
	MX25_PAD_FEC_TX_EN__GPIO_3_9,
	MX25_PAD_FEC_RDATA0__GPIO_3_10,
	MX25_PAD_FEC_RDATA1__GPIO_3_11,
	MX25_PAD_FEC_RX_DV__GPIO_3_12,
	MX25_PAD_FEC_TX_CLK__GPIO_3_13,
	MX25_PAD_D12__GPIO_4_8,
	MX25_PAD_D10__GPIO_4_10,
};

static struct pad_desc karo_tx25_fec_pwr_gpios[] = {
	MX25_PAD_D11__GPIO_4_9,		/* FEC PHY power on pin */
	MX25_PAD_D13__GPIO_4_7,		/* FEC reset */
};

static struct pad_desc karo_tx25_fec_gpios_on[] = {
	MX25_PAD_FEC_MDC__FEC_MDC,
	MX25_PAD_FEC_MDIO__FEC_MDIO,
	MX25_PAD_FEC_TDATA0__FEC_TDATA0,
	MX25_PAD_FEC_TDATA1__FEC_TDATA1,
	MX25_PAD_FEC_TX_EN__FEC_TX_EN,
	MX25_PAD_FEC_RDATA0__FEC_RDATA0,
	MX25_PAD_FEC_RDATA1__FEC_RDATA1,
	MX25_PAD_FEC_RX_DV__FEC_RX_DV,
	MX25_PAD_FEC_TX_CLK__FEC_TX_CLK,
	MX25_PAD_D12__GPIO_4_8,
	MX25_PAD_D10__GPIO_4_10,
};

static struct gpio_desc {
	unsigned int gpio:7;
	unsigned int dir:1;
	unsigned int level:1;
} karo_tx25_fec_strap_gpios[] = {
	/* configure the PHY strap pins to the correct values */
	{ GPIO_PORTC |  5, 1, 0, },
	{ GPIO_PORTC |  6, 1, 0, },
	{ GPIO_PORTC |  7, 1, 0, },
	{ GPIO_PORTC |  8, 1, 0, },
	{ GPIO_PORTC |  9, 1, 0, },
	{ GPIO_PORTC | 10, 1, 1, },
	{ GPIO_PORTC | 11, 1, 1, },
	{ GPIO_PORTC | 12, 0, 1, },
	{ GPIO_PORTC | 13, 1, 0, },

	{ GPIO_PORTD |  8, 0, 0, },
	{ GPIO_PORTD | 10, 0, 0, },
	{ GPIO_PORTD |  9, 1, 1, },
	{ GPIO_PORTD |  7, 1, 0, },
};

#define TX25_FEC_PWR_GPIO	(GPIO_PORTD | 9)
#define TX25_FEC_RST_GPIO	(GPIO_PORTD | 7)

static int gpio_fec_active(void)
{
	int ret;
	int i;

	ret = mxc_iomux_v3_setup_multiple_pads(karo_tx25_fec_pwr_gpios,
					       ARRAY_SIZE(karo_tx25_fec_pwr_gpios));
	if (ret) {
		return ret;
	}
	/*
	 * If the PHY is already powered on, assume it has been
	 * correctly configured (by the boot loader)
	*/
	if (1 && gpio_get_value(TX25_FEC_PWR_GPIO) &&
	    gpio_get_value(TX25_FEC_RST_GPIO)) {
		ret = mxc_iomux_v3_setup_multiple_pads(karo_tx25_fec_gpios_on,
						       ARRAY_SIZE(karo_tx25_fec_gpios_on));
		if (ret) {
			mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_pwr_gpios,
							   ARRAY_SIZE(karo_tx25_fec_pwr_gpios));
			return ret;
		}
	} else {
		/* switch PHY strap pins into required state */
		ret = mxc_iomux_v3_setup_multiple_pads(karo_tx25_fec_gpios_off,
						       ARRAY_SIZE(karo_tx25_fec_gpios_off));
		if (ret) {
			mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_pwr_gpios,
							   ARRAY_SIZE(karo_tx25_fec_pwr_gpios));
			return ret;
		}
		DBG(0, "%s: Switching FEC PHY power on\n", __FUNCTION__);
		DBG(0, "%s: Asserting FEC PHY reset\n", __FUNCTION__);
		for (i = 0; i < ARRAY_SIZE(karo_tx25_fec_strap_gpios); i++) {
			struct gpio_desc *pd = &karo_tx25_fec_strap_gpios[i];

			ret = gpio_request(pd->gpio, "FEC");
			if (ret < 0) {
				DBG(0, "%s: Failed to request GPIO%d_%d: %d\n",
				    __FUNCTION__, pd->gpio / 32 + 1, pd->gpio % 32, ret);
				goto rel_mux;
			}
			if (pd->dir) {
				gpio_direction_output(pd->gpio,
						      pd->level);
			} else {
				gpio_direction_input(pd->gpio);
			}
		}
		DBG(0, "%s: Delaying for 22ms\n", __FUNCTION__);
		mdelay(22);
		DBG(0, "%s: Deasserting FEC PHY reset\n", __FUNCTION__);
		gpio_set_value(TX25_FEC_RST_GPIO, 1);
		mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_gpios_off,
						   ARRAY_SIZE(karo_tx25_fec_gpios_off));
		ret = mxc_iomux_v3_setup_multiple_pads(karo_tx25_fec_gpios_on,
						       ARRAY_SIZE(karo_tx25_fec_gpios_on));
		if (ret) {
			goto rel_gpio;
		}
	}
	return ret;

rel_mux:
	mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_gpios_off,
					   ARRAY_SIZE(karo_tx25_fec_gpios_off));
rel_gpio:
	while (--i >= 0) {
		struct gpio_desc *pd = &karo_tx25_fec_strap_gpios[i];
		gpio_free(pd->gpio);
	}
	mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_pwr_gpios,
					   ARRAY_SIZE(karo_tx25_fec_pwr_gpios));
	return ret;
}

/*
 * Setup GPIO for FEC device to be inactive
 *
 */
static void gpio_fec_inactive(void)
{
	int i;

	mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_gpios_on,
					   ARRAY_SIZE(karo_tx25_fec_gpios_on));
	mxc_iomux_v3_setup_multiple_pads(karo_tx25_fec_gpios_off,
					 ARRAY_SIZE(karo_tx25_fec_gpios_off));
	DBG(0, "%s: Asserting FEC PHY reset\n", __FUNCTION__);
	gpio_set_value(TX25_FEC_RST_GPIO, 0);
	DBG(0, "%s: Switching FEC PHY power off\n", __FUNCTION__);
	gpio_set_value(TX25_FEC_PWR_GPIO, 0);

	mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_gpios_off,
					   ARRAY_SIZE(karo_tx25_fec_gpios_off));
	mxc_iomux_v3_release_multiple_pads(karo_tx25_fec_pwr_gpios,
					   ARRAY_SIZE(karo_tx25_fec_pwr_gpios));
	for (i = 0; i < ARRAY_SIZE(karo_tx25_fec_strap_gpios); i++) {
		struct gpio_desc *pd = &karo_tx25_fec_strap_gpios[i];
		gpio_free(pd->gpio);
	}
}

static struct clk *tx25_fec_clk;

static int tx25_fec_suspend(struct platform_device *pdev)
{
	BUG_ON(tx25_fec_clk == NULL);
	DBG(1, "%s: Switching FEC PHY off\n", __FUNCTION__);
	gpio_fec_inactive();
	clk_disable(tx25_fec_clk);
	return 0;
}

static int tx25_fec_resume(struct platform_device *pdev)
{
	BUG_ON(tx25_fec_clk == NULL);
	DBG(1, "%s: Switching FEC PHY on\n", __FUNCTION__);
	clk_enable(tx25_fec_clk);
	gpio_fec_active();
	return 0;
}

static int fec_arch_init(struct platform_device *pdev)
{
	int ret;

	DBG(0, "%s: Activating FEC GPIOs\n", __FUNCTION__);

	ret = gpio_fec_active();
	if (ret) {
		printk(KERN_ERR "%s: could not enable FEC gpios: %d\n", __FUNCTION__, ret);
		return ret;
	}

	BUG_ON(tx25_fec_clk != NULL);
	tx25_fec_clk = clk_get(&pdev->dev, NULL);
	if (unlikely(IS_ERR(tx25_fec_clk))) {
		printk(KERN_ERR "Failed to get fec_clk\n");
		return PTR_ERR(tx25_fec_clk);
	}
	DBG(0, "%s: Enabling FEC clock\n", __FUNCTION__);
	clk_enable(tx25_fec_clk);
	return 0;
}

static void fec_arch_exit(struct platform_device *pdev)
{
	BUG_ON(tx25_fec_clk == NULL);
	if (unlikely(IS_ERR(tx25_fec_clk))) {
		printk(KERN_ERR "Failed to get fec_clk\n");
		return;
	}
	DBG(0, "%s: Disabling FEC clock\n", __FUNCTION__);
	clk_disable(tx25_fec_clk);
	clk_put(tx25_fec_clk);
	tx25_fec_clk = NULL;
	DBG(0, "%s: Deactivating FEC GPIOs\n", __FUNCTION__);
	gpio_fec_inactive();
}

static int fec_set_mac(struct platform_device *pdev, char *addr)
{
	int i;
	void __iomem *ioaddr;
	unsigned long fec_mac_base = 0;

	fec_mac_base = IIM_BASE_ADDR + 0x68;
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
	.arch_init = fec_arch_init,
	.arch_exit = fec_arch_exit,
	.set_mac_addr = fec_set_mac,
	.suspend = tx25_fec_suspend,
	.resume = tx25_fec_resume,
};

static struct platform_device fec_device = {
	.name		= "fec",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(fec_resources),
	.resource	= fec_resources,
	.dev = {
		.platform_data = &fec_data,
		.coherent_dma_mask = 0xFFFFFFFF,
	},
};
#endif

/* MTD NAND flash */
//#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)
#if 1
static struct pad_desc karo_tx25_nand_pads[] = {
	MX25_PAD_NF_CE0__NF_CE0,
	MX25_PAD_NFWE_B__NFWE_B,
	MX25_PAD_NFRE_B__NFRE_B,
	MX25_PAD_NFALE__NFALE,
	MX25_PAD_NFCLE__NFCLE,
	MX25_PAD_NFWP_B__NFWP_B,
	MX25_PAD_NFRB__NFRB,
	MX25_PAD_D7__D7,
	MX25_PAD_D6__D6,
	MX25_PAD_D5__D5,
	MX25_PAD_D4__D4,
	MX25_PAD_D3__D3,
	MX25_PAD_D2__D2,
	MX25_PAD_D1__D1,
	MX25_PAD_D0__D0,
};

static struct mxc_nand_platform_data tx25_nand_data = {
	.hw_ecc = 1,
	.width = 1,
};

static int tx25_nand_init(void)
{
	int ret;

	DBG(0, "%s: Configuring NAND pins\n", __FUNCTION__);
	ret = mxc_iomux_v3_setup_multiple_pads(karo_tx25_nand_pads,
					       ARRAY_SIZE(karo_tx25_nand_pads));
	if (ret) {
		return ret;
	}
	return 0;
}
arch_initcall(tx25_nand_init);

#define NFC_BASE_ADDR		0xBB000000UL
#define NFC_BASE_ADDR_VIRT	VA(0xFC530000)

static struct resource tx25_nand_resources[] = {
	{
		.start	= NFC_BASE_ADDR + 0x1e00,
		.end	= NFC_BASE_ADDR + 0x1e2f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= NFC_BASE_ADDR,
		.end	= NFC_BASE_ADDR + 0x11ff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MXC_INT_NANDFC,
		.end	= MXC_INT_NANDFC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tx25_nand_mtd_device = {
	.name = "mxc_nand",
	.id = 0,
	.num_resources = ARRAY_SIZE(tx25_nand_resources),
	.resource = tx25_nand_resources,
	.dev = {
		.platform_data = &tx25_nand_data,
	},
};
#endif

#if defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT) || defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT_MODULE)
static u64 mxc_emma_dmamask = 0xffffffffUL;

static struct platform_device tx25_v4l2out_device = {
	.name = "MXC Video Output",
	.id = 0,
	.dev = {
		.dma_mask = &mxc_emma_dmamask,
		.coherent_dma_mask = ~0UL,
	},
};
#endif

//#if 0
#if 1 //defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static struct pad_desc mxc_i2c0_pins[] = {
	MX25_PAD_I2C1_CLK__I2C1_CLK,
	MX25_PAD_I2C1_DAT__I2C1_DAT,
};

static int karo_tx25_i2c_0_init(struct device *dev)
{
	DBG(-1, "%s: \n", __FUNCTION__);
	return mxc_iomux_v3_setup_multiple_pads(mxc_i2c0_pins,
						ARRAY_SIZE(mxc_i2c0_pins));
}

static void karo_tx25_i2c_0_exit(struct device *dev)
{
	DBG(-1, "%s: \n", __FUNCTION__);
	mxc_iomux_v3_release_multiple_pads(mxc_i2c0_pins,
					   ARRAY_SIZE(mxc_i2c0_pins));
}

static struct imxi2c_platform_data karo_tx25_i2c_0_data = {
	.bitrate = 100000,
	.init = karo_tx25_i2c_0_init,
	.exit = karo_tx25_i2c_0_exit,
};



static struct led_info laserbox_7seg_leds[8] = {
		{.name ="7sg_0", .default_trigger = NULL, .flags = 0,},
		{.name ="7sg_1", .default_trigger = NULL, .flags = 0,},
		{.name ="7sg_2", .default_trigger = NULL, .flags = 0,},
		{.name ="7sg_3", .default_trigger = NULL, .flags = 0,},
		{.name ="7sg_4", .default_trigger = NULL, .flags = 0,},
		{.name ="7sg_5", .default_trigger = NULL, .flags = 0,},
		{.name ="7sg_6", .default_trigger = NULL, .flags = 0,},
		{.name ="7sg_7", .default_trigger = NULL, .flags = 0,},
};

static struct led_platform_data laserbox_led_7seg = {
		.num_leds = 8,
		.leds = laserbox_7seg_leds,
};

static struct led_info laserbox_front_leds[8] = {
		{.name ="frt_0", .default_trigger = NULL, .flags = 0,},
		{.name ="frt_1", .default_trigger = NULL, .flags = 0,},
		{.name ="frt_2", .default_trigger = NULL, .flags = 0,},
		{.name ="frt_3", .default_trigger = NULL, .flags = 0,},
		{.name ="frt_4", .default_trigger = NULL, .flags = 0,},
		{.name ="frt_5", .default_trigger = NULL, .flags = 0,},
		{.name ="frt_6", .default_trigger = NULL, .flags = 0,},
		{.name ="frt_7", .default_trigger = NULL, .flags = 0,},
};

static struct led_platform_data laserbox_led_front = {
		.num_leds = 8,
		.leds = laserbox_front_leds,
};

static struct led_info laserbox_extout_leds[8] =  {
		{.name ="int_0", .default_trigger = NULL, .flags = 0,},
		{.name ="int_1", .default_trigger = NULL, .flags = 0,},
		{.name ="int_2", .default_trigger = NULL, .flags = 0,},
		{.name ="int_3", .default_trigger = NULL, .flags = 0,},
		{.name ="int_4", .default_trigger = NULL, .flags = 0,},
		{.name ="int_5", .default_trigger = NULL, .flags = 0,},
		{.name ="int_6", .default_trigger = NULL, .flags = 0,},
		{.name ="int_7", .default_trigger = NULL, .flags = 0,},
};

static struct led_platform_data laserbox_led_extout = {
		.num_leds = 8,
		.leds = laserbox_extout_leds,
};

static struct pca953x_platform_data laserbox_gpio_front = {
         .gpio_base = 128,
};

static struct pca953x_platform_data laserbox_gpio_int1 = {
        .gpio_base = 136,
};

static struct pca953x_platform_data laserbox_gpio_int2 = {
        .gpio_base = 144,
};

static struct pca953x_platform_data laserbox_gpio_base = {
        .gpio_base = 152,
};

static struct at24_platform_data karo_tx25_eeprom = {
	.byte_len = 8192,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16,
};

static struct i2c_board_info karo_i2c_0_boardinfo[] __initdata = {
	{/* main EEPROM with MLe config (CPU PCB) */
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &karo_tx25_eeprom,
		.type = "24c64",
	},
	{/* EEPROM of camera PCB */
		I2C_BOARD_INFO("at24", 0x51),
		.platform_data = &karo_tx25_eeprom,
		.type = "24c64",
	},
	{/* EEPROM of connection PCB */
		I2C_BOARD_INFO("at24", 0x52),
		.platform_data = &karo_tx25_eeprom,
		.type = "24c64",
	},
	{/* EEPROM of "Steckplatine" */
		I2C_BOARD_INFO("at24", 0x53),
		.platform_data = &karo_tx25_eeprom,
		.type = "24c64",
	},
	{/* EEPROM of base PCB */
		I2C_BOARD_INFO("at24", 0x54),
		.platform_data = &karo_tx25_eeprom,
		.type = "24c64",
	},
/*
	{  temperature sensors
		I2C_BOARD_INFO("lm73", 0x48),
	},
	{
		I2C_BOARD_INFO("lm73", 0x49),
	},
	{
		I2C_BOARD_INFO("lm73", 0x4a),

	},
*/	{ /* temperature sensors */
		I2C_BOARD_INFO("lm73", 0x4c),
	},
	{
		I2C_BOARD_INFO("lm73", 0x4d),
	},
	{
		I2C_BOARD_INFO("lm73", 0x4e),
	},
	{ /* digital potentiometers */
		I2C_BOARD_INFO("isl90840", 0x28),
	},
	{
		I2C_BOARD_INFO("isl90840", 0x29),
	},
	{
		I2C_BOARD_INFO("isl90840", 0x2a),
	},
	{
		I2C_BOARD_INFO("isl90840", 0x2b),
	},
	{ /* LEDs */
		I2C_BOARD_INFO("pca9551", 0x63),
		.platform_data = &laserbox_led_7seg,
	},
	{
		I2C_BOARD_INFO("pca9551", 0x61),
		.platform_data = &laserbox_led_front,
	},
	{
		I2C_BOARD_INFO("pca9551", 0x60),
		.platform_data = &laserbox_led_extout,
	},
	{ /* GPIOs */
		I2C_BOARD_INFO("pca9557", 0x1b),
		.platform_data = &laserbox_gpio_front,
	},
	{
		I2C_BOARD_INFO("pca9557", 0x18),
		.platform_data = &laserbox_gpio_int1,
	},
	{
		I2C_BOARD_INFO("pca9557", 0x19),
		.platform_data = &laserbox_gpio_int2,
	},
	{
		I2C_BOARD_INFO("pca9557", 0x1f),
		.platform_data = &laserbox_gpio_base,
	},
	{
		I2C_BOARD_INFO("ads1112", 0x48),
	},
	{
		I2C_BOARD_INFO("ads1112", 0x4a),
	},
	{
		I2C_BOARD_INFO("mcp7941x", 0x6f),
	},
};

int __init karo_i2c_init(void)
{
	int ret;

	DBG(0, "%s: Registering I2C bus 0 (new)\n", __FUNCTION__);
	ret = mxc_register_device(&mxc_i2c_device0, &karo_tx25_i2c_0_data);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C device: %d\n", ret);
		return ret;
	}
	ret = i2c_register_board_info(0, karo_i2c_0_boardinfo,
				      ARRAY_SIZE(karo_i2c_0_boardinfo));
	if (ret != 0) {
		printk(KERN_ERR "Failed to register I2C board info: %d\n", ret);
	}
	return ret;
}
device_initcall(karo_i2c_init);
#endif
//#endif

#if defined(CONFIG_TOUCHSCREEN_MXC_TSC) || defined(CONFIG_TOUCHSCREEN_MXC_TSC_MODULE)
static struct mxc_tsc_pdata karo_tx25_tsc_pdata = {
	.pen_debounce_time = 32,
	.intref = 1,
	.adc_clk = 1666667,
	.tsc_mode = MXC_TSC_4WIRE,
	.r_xplate = 660,
	.hsyncen = 1,
	.hsyncpol = 0,
};

static int __init karo_tx25_tsc_register(void)
{
	int ret;
	printk(KERN_INFO "registering TSC device ...\n");
	ret = mxc_register_device(&mx25_tsc_device, &karo_tx25_tsc_pdata);
	if (ret) {
		printk(KERN_WARNING "Failed to register TSC device: %d\n", ret);
	}
	return ret;
}
device_initcall(karo_tx25_tsc_register);
#endif

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)

#ifdef CONFIG_CAN_FLEXCAN_CAN1
static struct pad_desc tx25_flexcan1_pads[] = {
	MX25_PAD_GPIO_A__CAN1_TX,
	MX25_PAD_GPIO_B__CAN1_RX,
};

static struct resource tx25_flexcan1_resources[] = {
	{
		.start = CAN1_BASE_ADDR,
		.end = CAN1_BASE_ADDR + 0x97f,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CAN1,
		.end = MXC_INT_CAN1,
		.flags = IORESOURCE_IRQ,
	},
};

static int tx25_flexcan1_active(struct platform_device *pdev)
{
	return mxc_iomux_v3_setup_multiple_pads(tx25_flexcan1_pads,
					       ARRAY_SIZE(tx25_flexcan1_pads));
}

static void tx25_flexcan1_inactive(struct platform_device *pdev)
{
	mxc_iomux_v3_release_multiple_pads(tx25_flexcan1_pads,
					   ARRAY_SIZE(tx25_flexcan1_pads));
	karo_tx25_gpio_config(tx25_flexcan1_pads,
			      ARRAY_SIZE(tx25_flexcan1_pads));
}

static struct flexcan_platform_data tx25_flexcan1_pdata = {
	//.core_reg = NULL;
	//.io_reg = NULL;
	//.xcvr_enable = NULL,
	.active = tx25_flexcan1_active,
	.inactive = tx25_flexcan1_inactive,
};

static struct platform_device tx25_flexcan1_device = {
	.id = 0,
	.name = "mxc-flexcan",
	.num_resources = ARRAY_SIZE(tx25_flexcan1_resources),
	.resource = tx25_flexcan1_resources,
	.dev = {
		.platform_data = &tx25_flexcan1_pdata,
	},
};
#endif // CONFIG_CAN_FLEXCAN_CAN1

#ifdef CONFIG_CAN_FLEXCAN_CAN2
static struct pad_desc tx25_flexcan2_pads[] = {
	MX25_PAD_GPIO_C__CAN2_TX,
	MX25_PAD_GPIO_D__CAN2_RX,
};

static struct resource tx25_flexcan2_resources[] = {
	{
		.start = CAN2_BASE_ADDR,
		.end = CAN2_BASE_ADDR + 0x97f,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CAN2,
		.end = MXC_INT_CAN2,
		.flags = IORESOURCE_IRQ,
	},
};

static int tx25_flexcan2_active(struct platform_device *pdev)
{
	return mxc_iomux_v3_setup_multiple_pads(tx25_flexcan2_pads,
					       ARRAY_SIZE(tx25_flexcan2_pads));
}

static void tx25_flexcan2_inactive(struct platform_device *pdev)
{
	mxc_iomux_v3_release_multiple_pads(tx25_flexcan2_pads,
					   ARRAY_SIZE(tx25_flexcan2_pads));
	karo_tx25_gpio_config(tx25_flexcan2_pads,
			      ARRAY_SIZE(tx25_flexcan2_pads));
}

static struct flexcan_platform_data tx25_flexcan2_pdata = {
	//.core_reg = NULL;
	//.io_reg = NULL;
	//.xcvr_enable = NULL,
	.active = tx25_flexcan2_active,
	.inactive = tx25_flexcan2_inactive,
};

static struct platform_device tx25_flexcan2_device = {
	.id = 1,
	.name = "mxc-flexcan",
	.num_resources = ARRAY_SIZE(tx25_flexcan2_resources),
	.resource = tx25_flexcan2_resources,
	.dev = {
		.platform_data = &tx25_flexcan2_pdata,
	},
};
#endif // CONFIG_CAN_FLEXCAN_CAN2
#endif // CONFIG_CAN_FLEXCAN || CONFIG_CAN_FLEXCAN_MODULE

struct platform_dev_list {
	struct platform_device *pdev;
	int flag;
} tx25_devices[] __initdata = {
#if defined(CONFIG_RTC_MX25) || defined(CONFIG_RTC_MX25_MODULE)
	{ .pdev = &mx25_rtc_device, .flag = -1, },
#endif
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)
	{ .pdev = &tx25_nand_mtd_device, .flag = -1, },
#endif
#if defined(CONFIG_FEC_PHYLIB) || defined(CONFIG_FEC_PHYLIB_MODULE)
	{ .pdev = &fec_device, .flag = -1, },
#endif
#if defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT) || defined(CONFIG_VIDEO_MXC_EMMA_OUTPUT_MODULE)
	{ .pdev = &tx25_v4l2out_device, .flag = -1, },
#endif
#if defined(CONFIG_MXC_VPU) || defined(CONFIG_MXC_VPU_MODULE)
	{ .pdev = &mxc_vpu_device, .flag = -1, },
#endif
#if 0
#if defined(CONFIG_TOUCHSCREEN_MXC_TSC) || defined(CONFIG_TOUCHSCREEN_MXC_TSC_MODULE)
	{ .pdev = &mxc_tsc_device, .flag = -1, },
#endif
#endif
#ifdef CONFIG_CAN_FLEXCAN_CAN1
	{ .pdev = &tx25_flexcan1_device, .flag = -1, },
#endif
#ifdef CONFIG_CAN_FLEXCAN_CAN2
	{ .pdev = &tx25_flexcan2_device, .flag = -1, },
#endif
};
#define TX25_NUM_DEVICES		ARRAY_SIZE(tx25_devices)


static __init void karo_tx25_board_init(void)
{
	int i;

	DBG(0, "%s: \n", __FUNCTION__);

	for (i = 0; i < TX25_NUM_DEVICES; i++) {
		int ret;

		if (tx25_devices[i].pdev == NULL) continue;
		if (!tx25_devices[i].flag) {
			DBG(0, "%s: Skipping platform device[%d] @ %p dev %p: %s\n",
			    __FUNCTION__, i, tx25_devices[i].pdev, &tx25_devices[i].pdev->dev,
			    tx25_devices[i].pdev->name);
			continue;
		}
		DBG(0, "%s: Registering platform device[%d] @ %p dev %p: %s\n",
		    __FUNCTION__, i, tx25_devices[i].pdev, &tx25_devices[i].pdev->dev,
		    tx25_devices[i].pdev->name);
		ret = platform_device_register(tx25_devices[i].pdev);
		if (ret) {
			printk(KERN_WARNING "%s: Failed to register platform_device[%d]: %s: %d\n",
			       __FUNCTION__, i, tx25_devices[i].pdev->name, ret);
		}
	}
#if defined(CONFIG_RTC_MX25) || defined(CONFIG_RTC_MX25_MODULE)
	device_init_wakeup(&mx25_rtc_device.dev, 1);
#endif

	DBG(0, "%s: Done\n", __FUNCTION__);
}

static struct pad_desc karo_tx25_gpios[] __initdata = {
	MX25_PAD_GPIO_A__GPIO_A,
	MX25_PAD_GPIO_B__GPIO_B,
	MX25_PAD_GPIO_C__GPIO_C,
	MX25_PAD_GPIO_D__GPIO_D,
	MX25_PAD_GPIO_E__GPIO_E,
	MX25_PAD_GPIO_F__GPIO_F,
	MX25_PAD_CSI_D7__GPIO_1_6,
	MX25_PAD_CSI_D8__GPIO_1_7,
//	MX25_PAD_CSI_MCLK__GPIO_1_8,
//	MX25_PAD_CSI_VSYNC__GPIO_1_9,
//	MX25_PAD_CSI_HSYNC__GPIO_1_10,
//	MX25_PAD_CSI_PIXCLK__GPIO_1_11,
	MX25_PAD_I2C1_CLK__GPIO_1_12,
	MX25_PAD_I2C1_DAT__GPIO_1_13,
	MX25_PAD_CSPI1_MOSI__GPIO_1_14,
	MX25_PAD_CSPI1_MISO__GPIO_1_15,
	MX25_PAD_CSPI1_SS0__GPIO_1_16,
	MX25_PAD_CSPI1_SS1__GPIO_1_17,
	MX25_PAD_CSPI1_SCLK__GPIO_1_18,
	MX25_PAD_LD5__GPIO_1_19,
	MX25_PAD_LD6__GPIO_1_20,
	MX25_PAD_LD7__GPIO_1_21,
	MX25_PAD_HSYNC__GPIO_1_22,
	MX25_PAD_VSYNC__GPIO_1_23,
	MX25_PAD_LSCLK__GPIO_1_24,
	MX25_PAD_OE_ACD__GPIO_1_25,
	MX25_PAD_PWM__GPIO_1_26,
	MX25_PAD_CSI_D2__GPIO_1_27,
	MX25_PAD_CSI_D3__GPIO_1_28,
	MX25_PAD_CSI_D4__GPIO_1_29,
	MX25_PAD_CSI_D5__GPIO_1_30,
	MX25_PAD_CSI_D6__GPIO_1_31,

	MX25_PAD_A14__GPIO_2_0,
	MX25_PAD_A15__GPIO_2_1,
	MX25_PAD_A16__GPIO_2_2,
	MX25_PAD_A17__GPIO_2_3,
	MX25_PAD_A18__GPIO_2_4,
	MX25_PAD_A19__GPIO_2_5,
	MX25_PAD_A20__GPIO_2_6,
	MX25_PAD_A21__GPIO_2_7,
	MX25_PAD_A22__GPIO_2_8,
	MX25_PAD_A23__GPIO_2_9,
	MX25_PAD_A24__GPIO_2_10,
	MX25_PAD_A25__GPIO_2_11,
	MX25_PAD_EB0__GPIO_2_12,
	MX25_PAD_EB1__GPIO_2_13,
	MX25_PAD_OE__GPIO_2_14,
	MX25_PAD_LD0__GPIO_2_15,
	MX25_PAD_LD1__GPIO_2_16,
	MX25_PAD_LD2__GPIO_2_17,
	MX25_PAD_LD3__GPIO_2_18,
	MX25_PAD_LD4__GPIO_2_19,
	MX25_PAD_DE_B__GPIO_2_20,
	MX25_PAD_CLKO__GPIO_2_21,
	MX25_PAD_CSPI1_RDY__GPIO_2_22,
	MX25_PAD_SD1_CMD__GPIO_2_23,
	MX25_PAD_SD1_CLK__GPIO_2_24,
	MX25_PAD_SD1_DATA0__GPIO_2_25,
	MX25_PAD_SD1_DATA1__GPIO_2_26,
	MX25_PAD_SD1_DATA2__GPIO_2_27,
	MX25_PAD_SD1_DATA3__GPIO_2_28,
	MX25_PAD_KPP_ROW0__GPIO_2_29,
	MX25_PAD_KPP_ROW1__GPIO_2_30,
	MX25_PAD_KPP_ROW2__GPIO_2_31,

	MX25_PAD_KPP_ROW3__GPIO_3_0,
	MX25_PAD_KPP_COL0__GPIO_3_1,
	MX25_PAD_KPP_COL1__GPIO_3_2,
	MX25_PAD_KPP_COL2__GPIO_3_3,
	MX25_PAD_KPP_COL3__GPIO_3_4,
	MX25_PAD_FEC_MDC__GPIO_3_5,
	MX25_PAD_FEC_MDIO__GPIO_3_6,
	MX25_PAD_FEC_TDATA0__GPIO_3_7,
	MX25_PAD_FEC_TDATA1__GPIO_3_8,
	MX25_PAD_FEC_TX_EN__GPIO_3_9,
	MX25_PAD_FEC_RDATA0__GPIO_3_10,
	MX25_PAD_FEC_RDATA1__GPIO_3_11,
	MX25_PAD_FEC_RX_DV__GPIO_3_12,
	MX25_PAD_FEC_TX_CLK__GPIO_3_13,
	MX25_PAD_RTCK__GPIO_3_14,
	MX25_PAD_EXT_ARMCLK__GPIO_3_15,
	MX25_PAD_UPLL_BYPCLK__GPIO_3_16,
	MX25_PAD_VSTBY_REQ__GPIO_3_17,
	MX25_PAD_VSTBY_ACK__GPIO_3_18,
	MX25_PAD_POWER_FAIL__GPIO_3_19,
	MX25_PAD_CS4__GPIO_3_20,
	MX25_PAD_CS5__GPIO_3_21,
#if 0
	MX25_PAD_NF_CE0__GPIO_3_22,
#endif
	MX25_PAD_ECB__GPIO_3_23,
	MX25_PAD_LBA__GPIO_3_24,
	MX25_PAD_RW__GPIO_3_25,
#if 0
	MX25_PAD_NFWE_B__GPIO_3_26,
	MX25_PAD_NFRE_B__GPIO_3_27,
	MX25_PAD_NFALE__GPIO_3_28,
	MX25_PAD_NFCLE__GPIO_3_29,
	MX25_PAD_NFWP_B__GPIO_3_30,
	MX25_PAD_NFRB__GPIO_3_31,
#endif
	MX25_PAD_A10__GPIO_4_0,
	MX25_PAD_A13__GPIO_4_1,
	MX25_PAD_CS0__GPIO_4_2,
	MX25_PAD_CS1__GPIO_4_3,
	MX25_PAD_BCLK__GPIO_4_4,
	MX25_PAD_D15__GPIO_4_5,
	MX25_PAD_D14__GPIO_4_6,
	MX25_PAD_D13__GPIO_4_7,
	MX25_PAD_D12__GPIO_4_8,
	MX25_PAD_D11__GPIO_4_9,
	MX25_PAD_D10__GPIO_4_10,
	MX25_PAD_D9__GPIO_4_11,
	MX25_PAD_D8__GPIO_4_12,
#if 0
	MX25_PAD_D7__GPIO_4_13,
	MX25_PAD_D6__GPIO_4_14,
	MX25_PAD_D5__GPIO_4_15,
	MX25_PAD_D4__GPIO_4_16,
	MX25_PAD_D3__GPIO_4_17,
	MX25_PAD_D2__GPIO_4_18,
	MX25_PAD_D1__GPIO_4_19,
	MX25_PAD_D0__GPIO_4_20,
#endif
	MX25_PAD_CSI_D9__GPIO_4_21,
	MX25_PAD_UART1_RXD__GPIO_4_22,
	MX25_PAD_UART1_TXD__GPIO_4_23,
	MX25_PAD_UART1_RTS__GPIO_4_24,
	MX25_PAD_UART1_CTS__GPIO_4_25,
	MX25_PAD_UART2_RXD__GPIO_4_26,
	MX25_PAD_UART2_TXD__GPIO_4_27,
	MX25_PAD_UART2_RTS__GPIO_4_28,
	MX25_PAD_UART2_CTS__GPIO_4_29,
	MX25_PAD_BOOT_MODE0__GPIO_4_30,
	MX25_PAD_BOOT_MODE1__GPIO_4_31,
};

static int __init karo_tx25_setup_gpios(void)
{
	karo_tx25_gpio_config(karo_tx25_gpios, ARRAY_SIZE(karo_tx25_gpios));
	return 0;
}
late_initcall(karo_tx25_setup_gpios);

static void __init karo_tx25_map_io(void)
{
	mx25_map_io();
}

static void __init karo_tx25_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static void __init karo_tx25_timer_init(void)
{
	mx25_clocks_init(24000000);
}

struct sys_timer karo_tx25_timer = {
	.init	= karo_tx25_timer_init,
};

static int __init karo_mod_type_setup(char *line)
{
	get_option(&line, &karo_mod_type);
	DBG(0, "%s: Module type set to 0x%02x by kernel cmd line\n",
		__FUNCTION__, karo_mod_type);
	return 1;
}
__setup("module_type=", karo_mod_type_setup);

static int __init karo_board_type_setup(char *line)
{
	get_option(&line, &karo_board_type);
	DBG(0, "%s: Board type set to 0x%02x by kernel cmd line\n",
		__FUNCTION__, karo_board_type);
	return 1;
}
__setup("board_type=", karo_board_type_setup);

MACHINE_START(TX25, "Ka-Ro electronics TX25 module (Freescale i.MX25)")
	/* Maintainer: <LW@KARO-electronics.de> */
	.phys_io	= MX25_AIPS1_BASE_ADDR,
	.io_pg_offst	= (MX25_AIPS1_BASE_ADDR_VIRT >> 18) & 0xfffc,
	.fixup		= karo_tx25_fixup,
	.map_io		= karo_tx25_map_io,
	.init_irq	= mx25_init_irq,
	.init_machine	= karo_tx25_board_init,
	.timer		= &karo_tx25_timer,
MACHINE_END
