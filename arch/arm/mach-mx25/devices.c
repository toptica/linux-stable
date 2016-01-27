#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <mach/mx25.h>
#include <mach/irqs.h>
#include <mach/sdma.h>

#include "sdma_script_code.h"

void mxc_sdma_get_script_info(sdma_script_start_addrs *sdma_script_addr)
{
	sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR;
	sdma_script_addr->mxc_sdma_ap_2_bp_addr = -1;
	sdma_script_addr->mxc_sdma_bp_2_ap_addr = -1;
	sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;

	sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
	sdma_script_addr->mxc_sdma_firi_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_firi_addr = -1;

	sdma_script_addr->mxc_sdma_uart_2_per_addr = uart_2_per_ADDR;
	sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_per_2_app_addr = per_2_app_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR;

	sdma_script_addr->mxc_sdma_per_2_per_addr = -1;

	sdma_script_addr->mxc_sdma_uartsh_2_per_addr = uartsh_2_per_ADDR;
	sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr = uartsh_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_per_2_shp_addr = per_2_shp_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR;

	sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR;

	sdma_script_addr->mxc_sdma_app_2_per_addr = app_2_per_ADDR;
	sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_shp_2_per_addr = shp_2_per_ADDR;
	sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR;

	sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = -1;

	sdma_script_addr->mxc_sdma_spdif_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_spdif_addr = -1;

	sdma_script_addr->mxc_sdma_asrc_2_mcu_addr = -1;

	sdma_script_addr->mxc_sdma_dptc_dvfs_addr = -1;
	sdma_script_addr->mxc_sdma_ext_mem_2_ipu_addr = ext_mem__ipu_ram_ADDR;
	sdma_script_addr->mxc_sdma_descrambler_addr = -1;

	sdma_script_addr->mxc_sdma_start_addr = (unsigned short *)sdma_code;
	sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
	sdma_script_addr->mxc_sdma_ram_code_start_addr = RAM_CODE_START_ADDR;
}

static struct resource uart0[] = {
	{
		.start = 0x43f90000,
		.end = 0x43f93fff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART1,
		.end = MXC_INT_UART1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device0 = {
	.name = "imx-uart",
	.id = 0,
	.resource = uart0,
	.num_resources = ARRAY_SIZE(uart0),
};

static struct resource uart1[] = {
	{
		.start = 0x43f94000,
		.end = 0x43f97fff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART2,
		.end = MXC_INT_UART2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device1 = {
	.name = "imx-uart",
	.id = 1,
	.resource = uart1,
	.num_resources = ARRAY_SIZE(uart1),
};

static struct resource uart2[] = {
	{
		.start = 0x5000c000,
		.end = 0x5000ffff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART3,
		.end = MXC_INT_UART3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device2 = {
	.name = "imx-uart",
	.id = 2,
	.resource = uart2,
	.num_resources = ARRAY_SIZE(uart2),
};

static struct resource uart3[] = {
	{
		.start = 0x50008000,
		.end = 0x5000bfff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART4,
		.end = MXC_INT_UART4,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device3 = {
	.name = "imx-uart",
	.id = 3,
	.resource = uart3,
	.num_resources = ARRAY_SIZE(uart3),
};

static struct resource uart4[] = {
	{
		.start = 0x5002c000,
		.end = 0x5002ffff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_UART5,
		.end = MXC_INT_UART5,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device4 = {
	.name = "imx-uart",
	.id = 4,
	.resource = uart4,
	.num_resources = ARRAY_SIZE(uart4),
};

#define MX25_OTG_BASE_ADDR 0x53FF4000

static u64 otg_dmamask = DMA_BIT_MASK(32);

static struct resource mxc_otg_resources[] = {
	{
		.start = MX25_OTG_BASE_ADDR,
		.end = MX25_OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_OTG,
		.end = MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_otg_device = {
	.name = "mxc-ehci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &otg_dmamask,
	},
	.resource = mxc_otg_resources,
	.num_resources = ARRAY_SIZE(mxc_otg_resources),
};

/* OTG gadget device */
static u64 udc_dmamask = DMA_BIT_MASK(32);

static struct resource mxc_udc_resources[] = {
	{
		.start = MX25_OTG_BASE_ADDR,
		.end = MX25_OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_OTG,
		.end = MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_otg_udc_device = {
	.name = "fsl-usb2-udc",
	.id   = -1,
	.dev  = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &udc_dmamask,
	},
	.resource = mxc_udc_resources,
	.num_resources = ARRAY_SIZE(mxc_udc_resources),
};

static u64 usbh2_dmamask = DMA_BIT_MASK(32);

static struct resource mxc_usbh2_resources[] = {
	{
		.start = MX25_OTG_BASE_ADDR + 0x400,
		.end = MX25_OTG_BASE_ADDR + 0x5ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MXC_INT_USB_H2,
		.end = MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbh2_device = {
	.name = "mxc-ehci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &usbh2_dmamask,
	},
	.resource = mxc_usbh2_resources,
	.num_resources = ARRAY_SIZE(mxc_usbh2_resources),
};

static struct resource mxc_spi_resources0[] = {
	{
	       .start = 0x43fa4000,
	       .end = 0x43fa7fff,
	       .flags = IORESOURCE_MEM,
	}, {
	       .start = MXC_INT_CSPI1,
	       .end = MXC_INT_CSPI1,
	       .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_spi_device0 = {
	.name = "spi_imx",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_spi_resources0),
	.resource = mxc_spi_resources0,
};

static struct resource mxc_spi_resources1[] = {
	{
	       .start = 0x50010000,
	       .end = 0x50013fff,
	       .flags = IORESOURCE_MEM,
	}, {
	       .start = MXC_INT_CSPI2,
	       .end = MXC_INT_CSPI2,
	       .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_spi_device1 = {
	.name = "spi_imx",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxc_spi_resources1),
	.resource = mxc_spi_resources1,
};

static struct resource mxc_spi_resources2[] = {
	{
	       .start = 0x50004000,
	       .end = 0x50007fff,
	       .flags = IORESOURCE_MEM,
	}, {
	       .start = MXC_INT_CSPI3,
	       .end = MXC_INT_CSPI3,
	       .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_spi_device2 = {
	.name = "spi_imx",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxc_spi_resources2),
	.resource = mxc_spi_resources2,
};

/* Camera stuff */

static u64 csi_dmamask = DMA_BIT_MASK(32);

static struct resource mx25_csi_resources[] = {
	{
	       .start = 0x53FF8000,
	       .end = 0x53FFBFFF,
	       .flags = IORESOURCE_MEM,
	}, {
	       .start = 17,
	       .end = 17,
	       .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx25_csi_device = {
	.name = "mx25-camera", /* ARNOFIX */
	.id = 0,
	.dev = {
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &csi_dmamask,
		},
	.num_resources = ARRAY_SIZE(mx25_csi_resources),
	.resource = mx25_csi_resources,
};

/* GPT-buzzer stuff */

static struct resource mxc_gpt_resources1[] = {
	{
		.start	= 0x53f8c000,
		.end	= 0x53f8ffff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = 53,
		.end     = 53,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_gpt_device1 = {
	.name = "mxc_gpt",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxc_gpt_resources1),
	.resource = mxc_gpt_resources1,
};

static struct resource mxc_pwm_resources0[] = {
	{
		.start	= 0x53fe0000,
		.end	= 0x53fe3fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = MXC_INT_PWM,
		.end     = MXC_INT_PWM,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device0 = {
	.name = "mxc_pwm",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources0),
	.resource = mxc_pwm_resources0,
};

static struct resource mxc_pwm_resources1[] = {
	{
		.start	= 0x53fa0000,
		.end	= 0x53fa3fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = MXC_INT_PWM2,
		.end     = MXC_INT_PWM2,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device1 = {
	.name = "mxc_pwm",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources1),
	.resource = mxc_pwm_resources1,
};

static struct resource mxc_pwm_resources2[] = {
	{
		.start	= 0x53fa8000,
		.end	= 0x53fabfff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = MXC_INT_PWM3,
		.end     = MXC_INT_PWM3,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device2 = {
	.name = "mxc_pwm",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources2),
	.resource = mxc_pwm_resources2,
};

static struct resource mxc_pwm_resources3[] = {
	{
		.start	= 0x53fc8000,
		.end	= 0x53fcbfff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = MXC_INT_PWM4,
		.end     = MXC_INT_PWM4,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device3 = {
	.name = "mxc_pwm",
	.id = 3,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources3),
	.resource = mxc_pwm_resources3,
};

static struct resource mxc_wdt_resources[] = {
	{
		.start	= WDOG_BASE_ADDR,
		.end	= WDOG_BASE_ADDR + 0x09,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device mx25_wdt_device = {
	.name = "mxc_wdt",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_wdt_resources),
	.resource = mxc_wdt_resources,
};

#if 0
static struct resource mx25_ssi1_resources[] = {
	{
		.start	= SSI1_BASE_ADDR,
		.end	= SSI1_BASE_ADDR + 0x6F,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MXC_INT_SSI1,
		.end	= MXC_INT_SSI1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "tx0",
		.start	= MXC_DMA_SSI1_16BIT_TX0,
		.end	= MXC_DMA_SSI1_16BIT_TX0,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "rx0",
		.start	= MXC_DMA_SSI1_16BIT_RX0,
		.end	= MXC_DMA_SSI1_16BIT_RX0,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "tx1",
		.start	= MXC_DMA_SSI1_16BIT_TX1,
		.end	= MXC_DMA_SSI1_16BIT_TX1,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "rx1",
		.start	= MXC_DMA_SSI1_16BIT_RX1,
		.end	= MXC_DMA_SSI1_16BIT_RX1,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource mx25_ssi2_resources[] = {
	{
		.start	= SSI2_BASE_ADDR,
		.end	= SSI2_BASE_ADDR + 0x6F,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MXC_INT_SSI2,
		.end	= MXC_INT_SSI2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "tx0",
		.start	= MXC_DMA_SSI2_16BIT_TX0,
		.end	= MXC_DMA_SSI2_16BIT_TX0,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "rx0",
		.start	= MXC_DMA_SSI2_16BIT_RX0,
		.end	= MXC_DMA_SSI2_16BIT_RX0,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "tx1",
		.start	= MXC_DMA_SSI2_16BIT_TX1,
		.end	= MXC_DMA_SSI2_16BIT_TX1,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "rx1",
		.start	= MXC_DMA_SSI2_16BIT_RX1,
		.end	= MXC_DMA_SSI2_16BIT_RX1,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device mx25_ssi1_device = {
	.name = "mxc-ssi",
	.id = 0,
	.num_resources = ARRAY_SIZE(mx25_ssi1_resources),
	.resource = mx25_ssi1_resources,
};

struct platform_device mx25_ssi2_device = {
	.name = "mxc-ssi",
	.id = 1,
	.num_resources = ARRAY_SIZE(mx25_ssi2_resources),
	.resource = mx25_ssi2_resources,
};
#endif

#if defined(CONFIG_MXC_AUDMUX_V3) || defined(CONFIG_MXC_AUDMUX_V3_MODULE)
static struct resource mxc_audmux_v3_resource = {
	.start  = AUDMUX_BASE_ADDR,
	.end    = AUDMUX_BASE_ADDR + 0x3b,
	.flags  = IORESOURCE_MEM,
};

struct platform_device mxc_audmux_v3_device = {
	.name		= "mxc_audmux_v3",
	.id		= -1,
	.resource       = &mxc_audmux_v3_resource,
	.num_resources  = 1,
};
#endif

#if defined(CONFIG_KEYBOARD_IMX) || defined(CONFIG_KEYBOARD_IMX_MODULE)
static struct resource imx_keypad_resources[] = {
	{
		.start	= 0x43fa8000,
		.end	= 0x43fa8000 + 0x7,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = 24,
		.end     = 24,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device imx_keypad_device = {
	.name = "imx-keypad",
	.id = -1,
	.num_resources = ARRAY_SIZE(imx_keypad_resources),
	.resource = imx_keypad_resources,
};
#endif

static struct resource mx25_tsc_resources[] = {
	{
		.start	= 0x50030000,
		.end	= 0x5003085f,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 46,
		.end	= 46,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mx25_tsc_device = {
	.name = "mx25-tsc",
	.id = -1,
	.num_resources = ARRAY_SIZE(mx25_tsc_resources),
	.resource = mx25_tsc_resources,
};

static struct resource mx25_i2c_1_resources[] = {
	{
		.start	= 0x43f80000,
		.end	= 0x43f80013,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MXC_INT_I2C,
		.end	= MXC_INT_I2C,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device mx25_i2c1_device = {
	.name = "imx-i2c",
	.id = 0,
	.num_resources = ARRAY_SIZE(mx25_i2c_1_resources),
	.resource = mx25_i2c_1_resources,
};

static struct resource mx25_i2c_2_resources[] = {
	{
		.start	= 0x43f98000,
		.end	= 0x43f98013,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MXC_INT_I2C2,
		.end	= MXC_INT_I2C2,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device mx25_i2c2_device = {
	.name = "imx-i2c",
	.id = 1,
	.num_resources = ARRAY_SIZE(mx25_i2c_2_resources),
	.resource = mx25_i2c_2_resources,
};

static struct resource mx25_i2c_3_resources[] = {
	{
		.start	= 0x43f84000,
		.end	= 0x43f84013,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MXC_INT_I2C3,
		.end	= MXC_INT_I2C3,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device mx25_i2c3_device = {
	.name = "imx-i2c",
	.id = 2,
	.num_resources = ARRAY_SIZE(mx25_i2c_3_resources),
	.resource = mx25_i2c_3_resources,
};

static struct resource mx25_fb_resources[] = {
	{
		.start = 0x53fbc000,
		.end   = 0x53fbc000 + 0xFFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_LCDC,
		.end   = MXC_INT_LCDC,
		.flags = IORESOURCE_IRQ,
	}
};

#if defined(CONFIG_RTC_DRV_MX25) || defined(CONFIG_RTC_DRV_MX25_MODULE)
static struct resource mx25_rtc_resources[] = {
	{
		.start = MX25_DRYICE_BASE_ADDR,
		.end = MX25_DRYICE_BASE_ADDR + 0x3f,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_DRYICE_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mx25_rtc_device = {
	.name = "rtc-mx25",
	.id = 0,
	.num_resources = ARRAY_SIZE(mx25_rtc_resources),
	.resource = mx25_rtc_resources,
};
#endif

/* mxc lcd driver */
struct platform_device mx25_fb_device = {
	.name = "imx-fb",
	.id = 0,
	.num_resources = ARRAY_SIZE(mx25_fb_resources),
	.resource = mx25_fb_resources,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct mxc_gpio_port imx_gpio_ports[] = {
	{
		.chip.label = "gpio-0",
		.base = (void __iomem *)MX25_GPIO1_BASE_ADDR_VIRT,
		.irq = MXC_INT_GPIO1,
		.virtual_irq_start = MXC_GPIO_IRQ_START,
	}, {
		.chip.label = "gpio-1",
		.base = (void __iomem *)MX25_GPIO2_BASE_ADDR_VIRT,
		.irq = MXC_INT_GPIO2,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32,
	}, {
		.chip.label = "gpio-2",
		.base = (void __iomem *)MX25_GPIO3_BASE_ADDR_VIRT,
		.irq = MXC_INT_GPIO3,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 64,
	}, {
		.chip.label = "gpio-3",
		.base = (void __iomem *)MX25_GPIO4_BASE_ADDR_VIRT,
		.irq = MXC_INT_GPIO4,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 96,
	}
};

int __init mx25_register_gpios(void)
{
	return mxc_gpio_init(imx_gpio_ports, ARRAY_SIZE(imx_gpio_ports));
}
