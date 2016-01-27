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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/uio_driver.h>
#include <linux/mxc_scc2_driver.h>
#include <linux/pwm_backlight.h>
#include <mach/hardware.h>
#include <mach/spba.h>
#include <mach/gpio.h>
#include <mach/sdma.h>
#include <mach/iomux-mx51.h>
#include <mach/mxc_ipuv3.h>

#include "sdma_script_code.h"
#include "crm_regs.h"


/* Flag used to indicate when IRAM has been initialized */
int iram_ready;

void mxc_sdma_get_script_info(sdma_script_start_addrs *sdma_script_addr)
{
	/* AP<->BP */
	sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR;
	sdma_script_addr->mxc_sdma_ap_2_bp_addr = -1;
	sdma_script_addr->mxc_sdma_bp_2_ap_addr = -1;
	sdma_script_addr->mxc_sdma_ap_2_ap_fixed_addr = -1;

	/*misc */
	sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;

	/* firi */
	sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
	sdma_script_addr->mxc_sdma_firi_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_firi_addr = -1;

	/* uart */
	sdma_script_addr->mxc_sdma_uart_2_per_addr = uart_2_per_ADDR;
	sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR;

	/* UART SH */
	sdma_script_addr->mxc_sdma_uartsh_2_per_addr = uartsh_2_per_ADDR;
	sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr = uartsh_2_mcu_ADDR;

	/* SHP */
	sdma_script_addr->mxc_sdma_per_2_shp_addr = per_2_shp_ADDR;
	sdma_script_addr->mxc_sdma_shp_2_per_addr = shp_2_per_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR;
	sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR;

	/* ATA */
	sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR;
	sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR;

	/* app */
	sdma_script_addr->mxc_sdma_app_2_per_addr = app_2_per_ADDR;
	sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_per_2_app_addr = per_2_app_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR;

	/* MSHC */
	sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = -1;

	/* spdif */
	sdma_script_addr->mxc_sdma_spdif_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_spdif_addr = mcu_2_spdif_ADDR;

	/* IPU */
	sdma_script_addr->mxc_sdma_ext_mem_2_ipu_addr = ext_mem__ipu_ram_ADDR;

	/* DVFS */
	sdma_script_addr->mxc_sdma_dptc_dvfs_addr = -1;

	/* core */
	sdma_script_addr->mxc_sdma_start_addr = (unsigned short *)sdma_code;
	sdma_script_addr->mxc_sdma_ram_code_start_addr = RAM_CODE_START_ADDR;
	sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
}

#if defined(CONFIG_W1_MASTER_MXC) || defined(CONFIG_W1_MASTER_MXC_MODULE)
static struct resource w1_resources[] = {
	{
		.start = MXC_INT_OWIRE,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

struct platform_device mxc_w1_devices = {
	.name = "mxc_w1",
	.dev = {
		.platform_data = &mxc_w1_data,
	},
	.num_resources = ARRAY_SIZE(w1_resources),
	.resource = w1_resources,
};
#endif

#if defined(CONFIG_RTC_DRV_MXC_SRTC) || defined(CONFIG_RTC_DRV_MXC_SRTC_MODULE)
static struct resource mx51_srtc_resources[] = {
	{
		.start = SRTC_BASE_ADDR,
		.end = SRTC_BASE_ADDR + 0x40,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IIM_BASE_ADDR + 0x840,
		.end = IIM_BASE_ADDR + 0x843,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SRTC_NTZ,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_srtc_device = {
	.name = "mxc_rtc",
	.id = -1,
	.num_resources = ARRAY_SIZE(mx51_srtc_resources),
	.resource = mx51_srtc_resources,
};
#endif

#if defined(CONFIG_FEC_PHYLIB) || defined(CONFIG_FEC_PHYLIB_MODULE)
static struct resource mx51_fec_resources[] = {
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
	},
};

struct platform_device mx51_fec_device = {
	.name		= "fec",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(mx51_fec_resources),
	.resource	= mx51_fec_resources,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

#if defined(CONFIG_MXC_WATCHDOG) || defined(CONFIG_MXC_WATCHDOG_MODULE)
static struct resource wdt_resources[] = {
	{
		.start = WDOG1_BASE_ADDR,
		.end = WDOG1_BASE_ADDR + 0x30,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device mx51_wdt_device = {
	.name = "mxc_wdt",
	.id = 0,
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource = wdt_resources,
};
#endif

#if defined(CONFIG_ARCH_MXC_EHCI_USBOTG)
static u64 usbotg_dmamask = DMA_BIT_MASK(32);

static struct resource mx51_usbotg_resources[] = {
	{
		.start = OTG_BASE_ADDR + 0x000,
		.end = OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_OTG,
		.end = MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_usbotg_device = {
	.name = "mxc-ehci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &usbotg_dmamask,
	},
	.resource = mx51_usbotg_resources,
	.num_resources = ARRAY_SIZE(mx51_usbotg_resources),
};
#endif

#if defined(CONFIG_ARCH_MXC_EHCI_USBH1)
static u64 usbh1_dmamask = DMA_BIT_MASK(32);

static struct resource mx51_usbh1_resources[] = {
	{
		.start = OTG_BASE_ADDR + 0x200,
		.end = OTG_BASE_ADDR + 0x3ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_H1,
		.end = MXC_INT_USB_H1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_usbh1_device = {
	.name = "mxc-ehci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &usbh1_dmamask,
	},
	.resource = mx51_usbh1_resources,
	.num_resources = ARRAY_SIZE(mx51_usbh1_resources),
};
#endif

#if defined(CONFIG_ARCH_MXC_EHCI_USBH2)
static u64 usbh2_dmamask = DMA_BIT_MASK(32);

static struct resource mx51_usbh2_resources[] = {
	{
		.start = OTG_BASE_ADDR + 0x400,
		.end = OTG_BASE_ADDR + 0x5ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_H2,
		.end = MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_usbh2_device = {
	.name = "mxc-ehci",
	.id = 2,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &usbh2_dmamask,
	},
	.resource = mx51_usbh2_resources,
	.num_resources = ARRAY_SIZE(mx51_usbh2_resources),
};
#endif

#if defined(CONFIG_ARCH_MXC_EHCI_USBH3)
static u64 usbh3_dmamask = DMA_BIT_MASK(32);

static struct resource mx51_usbh3_resources[] = {
	{
		.start = OTG_BASE_ADDR + 0x600,
		.end = OTG_BASE_ADDR + 0x7ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_H3,
		.end = MXC_INT_USB_H3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_usbh3_device = {
	.name = "mxc-ehci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &usbh3_dmamask,
	},
	.resource = mx51_usbh3_resources,
	.num_resources = ARRAY_SIZE(mx51_usbh3_resources),
};
#endif

#if defined(CONFIG_USB_FSL_USB2) || defined(CONFIG_USB_FSL_USB2_MODULE)
static u64 udc_dmamask = DMA_BIT_MASK(32);

static struct resource mxc_udc_resources[] = {
	{
		.start = OTG_BASE_ADDR,
		.end = OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_OTG,
		.end = MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_otg_udc_device = {
	.name = "fsl-usb2-udc",
	.id   = -1,
	.dev  = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &udc_dmamask,
	},
	.resource = mxc_udc_resources,
	.num_resources = ARRAY_SIZE(mxc_udc_resources),
};
#endif

#if defined(CONFIG_MXC_PWM) || defined(CONFIG_MXC_PWM_MODULE)
static struct resource pwm_resources[] = {
	{
		.start = PWM1_BASE_ADDR,
		.end = PWM1_BASE_ADDR + 0x14,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device mx51_pwm1_device = {
	.name = "mxc_pwm",
	.id = 0,
	.num_resources = ARRAY_SIZE(pwm_resources),
	.resource = pwm_resources,
};
#endif

#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
struct platform_device mx5_pwm_backlight_device = {
	.name = "pwm-backlight",
	.id = -1,
};
#endif

#if defined(CONFIG_MXC_IPU_V3) || defined(CONFIG_MXC_IPU_V3_MODULE)
static struct mxc_ipu_config mx51_ipu_data = {
	.rev = 2,
};

static struct resource mx51_ipu_resources[] = {
	{
		.start = IPU_CTRL_BASE_ADDR,
		.end = IPU_CTRL_BASE_ADDR + SZ_512M,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_IPU_SYN,
		.end = MXC_INT_IPU_SYN,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MXC_INT_IPU_ERR,
		.end = MXC_INT_IPU_ERR,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_ipu_device = {
	.name = "mxc_ipu",
	.id = -1,
	.dev = {
		.platform_data = &mx51_ipu_data,
	},
	.num_resources = ARRAY_SIZE(mx51_ipu_resources),
	.resource = mx51_ipu_resources,
};

static int __init mx51_ipu_init(void)
{
	if (mx51_revision() < MX51_CHIP_REV_2_0)
		mx51_ipu_data.rev = 1;
	return 0;
}
arch_initcall(mx51_ipu_init);
#endif

#if defined(CONFIG_FB_MXC_IPU_V3) || defined(CONFIG_FB_MXC_IPU_V3_MODULE)
static struct resource mx51_fb_resources[] = {
};

struct platform_device mx51_fb_device = {
	.name = "mxc_sdc_fb",
	.id = 0,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource = mx51_fb_resources,
	.num_resources = ARRAY_SIZE(mx51_fb_resources),
};
#endif

#if defined(CONFIG_MXC_VPU) || defined(CONFIG_MXC_VPU_MODULE)
static struct resource vpu_resources[] = {
	{
		.start = VPU_BASE_ADDR,
		.end = VPU_BASE_ADDR + 0x23,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = SRC_BASE_ADDR,
		.end = SRC_BASE_ADDR + 3,
		.flags = IORESOURCE_MEM,
	},
#ifdef CONFIG_MXC_VPU_IRAM
	{
		.start = VPU_IRAM_BASE_ADDR,
		.end = VPU_IRAM_BASE_ADDR + VPU_IRAM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
#endif
	{
		.start = MXC_INT_VPU,
		.end = MXC_INT_VPU,
		.flags = IORESOURCE_IRQ,
	},
};

/*! Platform Data for MXC VPU */
struct platform_device mx51_vpu_device = {
	.name = "mxc_vpu",
	.id = -1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources = ARRAY_SIZE(vpu_resources),
	.resource = vpu_resources,
};
#endif

/*!
 * This is platform device structure for adding SCC
 */
#if defined(CONFIG_MXC_SECURITY_SCC) || defined(CONFIG_MXC_SECURITY_SCC_MODULE)
struct platform_device mxc_scc_device = {
	.name = "mxc_scc",
	.id = 0,
};
#else
static int __init mxc_init_scc(void)
{
	uint32_t reg_value;
	uint32_t reg_mask = 0;
	uint8_t *UMID_base;
	uint32_t *MAP_base;
	uint8_t i;
	uint32_t partition_no;
	uint32_t scc_partno;
	void *scm_ram_base;
	void *scc_base;
	uint8_t iram_partitions = 16;

	if (mx51_revision() < MX51_CHIP_REV_2_0)
		iram_partitions = 12;

	scc_base = ioremap((uint32_t) SCC_BASE_ADDR, 0x140);
	if (scc_base == NULL) {
		printk(KERN_ERR "FAILED TO MAP IRAM REGS\n");
		return -ENOMEM;
	}
	scm_ram_base = ioremap(IRAM_BASE_ADDR, IRAM_SIZE);
	if (scm_ram_base == NULL) {
		printk(KERN_ERR "FAILED TO MAP IRAM\n");
		return -ENOMEM;
	}

	for (partition_no = 0; partition_no < iram_partitions; partition_no++) {
		/* De-allocate a Partition */
		reg_value = ((partition_no << SCM_ZCMD_PART_SHIFT) &
			SCM_ZCMD_PART_MASK) |
			((0x03 << SCM_ZCMD_CCMD_SHIFT) & SCM_ZCMD_CCMD_MASK);
		__raw_writel(reg_value, scc_base + SCM_ZCMD_REG);
		msleep(1);
		while ((__raw_readl(scc_base + SCM_STATUS_REG) &
			SCM_STATUS_SRS_READY) != SCM_STATUS_SRS_READY);

		/* In Supervisor mode claims a partition for it's own use
		 * by writing zero to SMID register
		 */
		__raw_writel(0, scc_base + (SCM_SMID0_REG + 8 * partition_no));

		reg_mask |= (3 << (2 * (partition_no)));
	}

	msleep(1);
	reg_value = __raw_readl(scc_base + SCM_PART_OWNERS_REG);
	if ((reg_value & reg_mask) != reg_mask) {
		printk(KERN_ERR "FAILED TO ACQUIRE IRAM PARTITION\n");
		iounmap(scm_ram_base);
		iounmap(scc_base);
		return -EBUSY;
	}

	for (partition_no = 0; partition_no < iram_partitions; partition_no++) {
		MAP_base = scm_ram_base + (partition_no * 0x2000);
		UMID_base = (uint8_t *)MAP_base + 0x10;

		for (i = 0; i < 16; i++)
			UMID_base[i] = 0;

		MAP_base[0] = SCM_PERM_NO_ZEROIZE | SCM_PERM_HD_SUP_DISABLE |
		    SCM_PERM_HD_READ | SCM_PERM_HD_WRITE | SCM_PERM_HD_EXECUTE |
		    SCM_PERM_TH_READ | SCM_PERM_TH_WRITE;
	}

	/* Freeing 2 partitions for SCC2 */
	scc_partno = iram_partitions - (SCC_IRAM_SIZE / SZ_8K);
	for (partition_no = scc_partno; partition_no < iram_partitions;
	     partition_no++) {
		reg_value = ((partition_no << SCM_ZCMD_PART_SHIFT) &
			     SCM_ZCMD_PART_MASK) | ((0x03 <<
						     SCM_ZCMD_CCMD_SHIFT)
						    & SCM_ZCMD_CCMD_MASK);
		__raw_writel(reg_value, scc_base + SCM_ZCMD_REG);
		msleep(1);
		while ((__raw_readl(scc_base + SCM_STATUS_REG) &
			SCM_STATUS_SRS_READY) != SCM_STATUS_SRS_READY);
	}
	iounmap(scm_ram_base);
	iounmap(scc_base);
	printk(KERN_INFO "IRAM READY\n");
	iram_ready = 1;
	return 0;
}
arch_initcall(mxc_init_scc);
#endif

/* SPI controller and device data */
#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)

#ifdef CONFIG_SPI_IMX_SELECT1
/*!
 * Resource definition for the CSPI1
 */
static struct resource mx51_spi1_resources[] = {
	{
		.start = CSPI1_BASE_ADDR,
		.end = CSPI1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CSPI1,
		.end = MXC_INT_CSPI1,
		.flags = IORESOURCE_IRQ,
	},
};

/*! Device Definition for MXC CSPI1 */
struct platform_device mx51_spi1_device = {
	.name = "spi_imx",
	.id = 0,
	.num_resources = ARRAY_SIZE(mx51_spi1_resources),
	.resource = mx51_spi1_resources,
};
#endif				/* CONFIG_SPI_IMX_SELECT1 */

#ifdef CONFIG_SPI_IMX_SELECT2
/*!
 * Resource definition for the CSPI2
 */
static struct resource mx51_spi2_resources[] = {
	{
		.start = CSPI2_BASE_ADDR,
		.end = CSPI2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CSPI2,
		.end = MXC_INT_CSPI2,
		.flags = IORESOURCE_IRQ,
	},
};

/*! Device Definition for MXC CSPI2 */
struct platform_device mx51_spi2_device = {
	.name = "spi_imx",
	.id = 1,
	.num_resources = ARRAY_SIZE(mx51_spi2_resources),
	.resource = mx51_spi2_resources,
};
#endif				/* CONFIG_SPI_IMX_SELECT2 */

#ifdef CONFIG_SPI_IMX_SELECT3
/*!
 * Resource definition for the CSPI3
 */
static struct resource mx51_spi3_resources[] = {
	{
		.start = CSPI3_BASE_ADDR,
		.end = CSPI3_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CSPI,
		.end = MXC_INT_CSPI,
		.flags = IORESOURCE_IRQ,
	},
};

/*! Device Definition for MXC CSPI3 */
struct platform_device mx51_spi3_device = {
	.name = "spi_imx",
	.id = 2,
	.num_resources = ARRAY_SIZE(mx51_spi3_resources),
	.resource = mx51_spi3_resources,
};
#endif /* CONFIG_SPI_IMX_SELECT3 */
#endif /* CONFIG_SPI_IMX || CONFIG_SPI_IMX_MODULE */

/* I2C controller and device data */
#if defined(CONFIG_I2C_IMX) || defined(CONFIG_I2C_IMX_MODULE)

#ifdef CONFIG_I2C_IMX_SELECT1
/*!
 * Resource definition for the I2C1
 */
static struct resource mx51_i2c1_resources[] = {
	{
		.start = I2C1_BASE_ADDR,
		.end = I2C1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_I2C1,
		.end = MXC_INT_I2C1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_i2c1_device = {
	.name = "imx-i2c",
	.id = 0,
	.num_resources = ARRAY_SIZE(mx51_i2c1_resources),
	.resource = mx51_i2c1_resources,
};
#endif

#ifdef CONFIG_I2C_IMX_SELECT2
/*!
 * Resource definition for the I2C2
 */
static struct resource mx51_i2c2_resources[] = {
	{
		.start = I2C2_BASE_ADDR,
		.end = I2C2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_I2C2,
		.end = MXC_INT_I2C2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_i2c2_device = {
	.name = "imx-i2c",
	.id = 1,
	.num_resources = ARRAY_SIZE(mx51_i2c2_resources),
	.resource = mx51_i2c2_resources,
};
#endif /* CONFIG_I2C_IMX_SELECT2 */
#endif /* CONFIG_I2C_IMX || CONFIG_I2C_IMX_MODULE */

#if defined(CONFIG_I2C_IMX_HS) || defined(CONFIG_I2C_IMX_HS_MODULE)
static struct resource mx51_i2c_hs_resources[] = {
	{
		.start = HSI2C_DMA_BASE_ADDR,
		.end = HSI2C_DMA_BASE_ADDR + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_HS_I2C,
		.end = MXC_INT_HS_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx51_i2c_hs_device = {
	.name = "imx-i2c_hs",
	.id = 3,
	.num_resources = ARRAY_SIZE(mx51_i2c_hs_resources),
	.resource = mx51_i2c_hs_resources,
};
#endif

#if defined(CONFIG_FB_MXC_TVOUT_TVE) || defined(CONFIG_FB_MXC_TVOUT_TVE_MODULE)
static struct resource tve_resources[] = {
	{
		.start = TVE_BASE_ADDR,
		.end = TVE_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_TVE,
		.end = MXC_INT_TVE,
		.flags = IORESOURCE_IRQ,
	},
};
static struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
	.dig_reg = "VDIG",
};

struct platform_device mxc_tve_device = {
	.name = "tve",
	.dev = {
		.platform_data = &tve_data,
	},
	.num_resources = ARRAY_SIZE(tve_resources),
	.resource = tve_resources,
};
#endif

#if defined(CONFIG_MMC_SDHCI_MXC) || defined(CONFIG_MMC_SDHCI_MXC_MODULE)
static struct resource mx51_esdhc1_resources[] = {
	{
		.start = MMC_SDHC1_BASE_ADDR,
		.end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_MMC_SDHC1,
		.end = MXC_INT_MMC_SDHC1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
		.end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
		.flags = IORESOURCE_IRQ,
	},
};

/*!
 * Resource definition for the SDHC2
 */
static struct resource mx51_esdhc2_resources[] = {
	{
		.start = MMC_SDHC2_BASE_ADDR,
		.end = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_MMC_SDHC2,
		.end = MXC_INT_MMC_SDHC2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_IRQ,
	},
};

/*! Device Definition for MXC SDHC1 */
struct platform_device mx51_esdhc1_device = {
	.name = "sdhci",
	.id = 0,
	.num_resources = ARRAY_SIZE(mx51_esdhc1_resources),
	.resource = mx51_esdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
struct platform_device mx51_esdhc2_device = {
	.name = "sdhci",
	.id = 1,
	.num_resources = ARRAY_SIZE(mx51_esdhc2_resources),
	.resource = mx51_esdhc2_resources,
};
#endif

#if defined(CONFIG_MXC_AUDMUX_V3) || defined(CONFIG_MXC_AUDMUX_V3_MODULE)
static struct resource mxc_audmux_v3_resource = {
	.start  = AUDMUX_BASE_ADDR,
	.end    = AUDMUX_BASE_ADDR + 0x37,
	.flags  = IORESOURCE_MEM,
};

struct platform_device mxc_audmux_v3_device = {
	.name		= "mxc_audmux_v3",
	.id		= -1,
	.resource       = &mxc_audmux_v3_resource,
	.num_resources  = 1,
};
#endif

#if 0
/*!
 * Resource definition for the DVFS CORE
 */
static struct resource dvfs_core_resources[] = {
	{
		.start = MXC_DVFS_CORE_BASE,
		.end = MXC_DVFS_CORE_BASE + 4 * SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_GPC1,
		.end = MXC_INT_GPC1,
		.flags = IORESOURCE_IRQ,
	},
};

/*! Platform Data for DVFS CORE */
struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.ccm_cdcr_reg_addr = MXC_CCM_CDCR,
	.ccm_cacrr_reg_addr = MXC_CCM_CACRR,
	.ccm_cdhipr_reg_addr = MXC_CCM_CDHIPR,
	.dvfs_thrs_reg_addr = MXC_DVFSTHRS,
	.dvfs_coun_reg_addr = MXC_DVFSCOUN,
	.dvfs_emac_reg_addr = MXC_DVFSEMAC,
	.dvfs_cntr_reg_addr = MXC_DVFSCNTR,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
	.num_wp = 3,
};

/*! Device Definition for MXC DVFS core */
struct platform_device mxc_dvfs_core_device = {
	.name = "mxc_dvfs_core",
	.id = 0,
	.dev = {
		.platform_data = &dvfs_core_data,
	},
	.num_resources = ARRAY_SIZE(dvfs_core_resources),
	.resource = dvfs_core_resources,
};
#endif

struct mxc_gpio_port mxc_gpio_ports[GPIO_PORT_NUM] = {
	{
		.chip = {
			.label = "gpio-0",
		},
		.base = MX5_IO_ADDRESS(GPIO1_BASE_ADDR),
		.irq = MXC_INT_GPIO1_LOW,
		//.irq_high = MXC_INT_GPIO1_HIGH,
		.virtual_irq_start = MXC_GPIO_INT_BASE,
	},
	{
		.chip = {
			.label = "gpio-1",
		},
		.base = MX5_IO_ADDRESS(GPIO2_BASE_ADDR),
		.irq = MXC_INT_GPIO2_LOW,
		//.irq_high = MXC_INT_GPIO2_HIGH,
		.virtual_irq_start = MXC_GPIO_INT_BASE + GPIO_NUM_PIN * 1,
	},
	{
		.chip = {
			.label = "gpio-2",
		},
		.base = MX5_IO_ADDRESS(GPIO3_BASE_ADDR),
		.irq = MXC_INT_GPIO3_LOW,
		//.irq_high = MXC_INT_GPIO3_HIGH,
		.virtual_irq_start = MXC_GPIO_INT_BASE + GPIO_NUM_PIN * 2,
	},
	{
		.chip = {
			.label = "gpio-3",
		},
		.base = MX5_IO_ADDRESS(GPIO4_BASE_ADDR),
		.irq = MXC_INT_GPIO4_LOW,
		//.irq_high = MXC_INT_GPIO4_HIGH,
		.virtual_irq_start = MXC_GPIO_INT_BASE + GPIO_NUM_PIN * 3,
	},
};

int __init mx51_register_gpios(void)
{
	return mxc_gpio_init(mxc_gpio_ports, ARRAY_SIZE(mxc_gpio_ports));
}

#if 0
static struct resource spdif_resources[] = {
	{
		.start = SPDIF_BASE_ADDR,
		.end = SPDIF_BASE_ADDR + 0x50,
		.flags = IORESOURCE_MEM,
	},
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 7,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

struct platform_device mxc_alsa_spdif_device = {
	.name = "mxc_alsa_spdif",
	.id = 0,
	.dev = {
		.platform_data = &mxc_spdif_data,
	},
	.num_resources = ARRAY_SIZE(spdif_resources),
	.resource = spdif_resources,
};
#endif

struct platform_device mx51_lpmode_device = {
	.name = "mx51_lpmode",
	.id = 0,
};

struct platform_device busfreq_device = {
	.name = "busfreq",
	.id = 0,
};

#if defined(CONFIG_MXC_IIM) || defined(CONFIG_MXC_IIM_MODULE)
static struct resource mxc_iim_resources[] = {
	{
		.start = IIM_BASE_ADDR,
		.end = IIM_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device mxc_iim_device = {
	.name = "mxc_iim",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_iim_resources),
	.resource = mxc_iim_resources,
};
#endif
