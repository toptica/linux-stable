#
# Makefile for the linux kernel.
#

# Object file lists.

ccflags-$(CONFIG_KARO_DEBUG)	+= -DDEBUG

#obj-y := system.o iomux.o cpu.o mm.o clock.o devices.o serial.o dma.o lpmodes.o pm.o bus_freq.o
obj-y := system.o cpu.o mm.o devices.o serial.o

obj-$(CONFIG_ARCH_MX51)			+= clock-mx51.o
obj-$(CONFIG_MXC_SDMA_API)		+= dma.o
obj-$(CONFIG_PM)			+= pm.o lpmodes.o
obj-$(CONFIG_CPUFREQ)			+= bus_freq.o
obj-$(CONFIG_CPU_V7)			+= wfi.o suspend.o
obj-$(CONFIG_MACH_TX51)			+= karo-tx51.o
obj-$(CONFIG_MACH_TX51_BASEBOARD_STK5)	+= stk5-baseboard.o
obj-$(CONFIG_MACH_TX51_CUSTOM)		+= stk5-custom.o
