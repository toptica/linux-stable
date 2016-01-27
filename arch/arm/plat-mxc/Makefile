#
# Makefile for the linux kernel.
#
# Common support
obj-y := clock.o gpio.o time.o devices.o cpu.o system.o

obj-$(CONFIG_ARCH_MX1)		+= irq.o iomux-mx1-mx2.o dma-mx1-mx2.o
obj-$(CONFIG_ARCH_MX2)		+= irq.o iomux-mx1-mx2.o dma-mx1-mx2.o
obj-$(CONFIG_ARCH_MX25)		+= irq.o
obj-$(CONFIG_ARCH_MX51)		+= spba.o iomux-mx51.o
obj-$(CONFIG_ARCH_MXC_IOMUX_V3)	+= iomux-v3.o
obj-$(CONFIG_MXC_SDMA_API)	+= sdma/
obj-$(CONFIG_MXC_TZIC)		+= tzic.o

obj-$(CONFIG_MXC_PWM)		+= pwm.o
obj-$(CONFIG_MX2_AUDMUX)	+= mx2_audmux.o
obj-$(CONFIG_MXC_AUDMUX_V3)	+= mxc_audmux_v3.o

mxc-ssi-ports-objs		:= ssi.o
obj-$(CONFIG_MXC_SSI_PORTS)	+= mxc-ssi-ports.o
obj-$(CONFIG_MXC_ULPI)		+= ulpi.o

ifeq ($(CONFIG_MXC_SDMA_API),)
	obj-y			+= sdma_compat.o
endif
