ccflags-$(CONFIG_KARO_DEBUG)		+= -DDEBUG

obj-y					:= mm.o devices.o
obj-$(CONFIG_ARCH_MX25)			+= clock.o cpu.o
obj-$(CONFIG_MACH_MX25_3DS)		+= mx25pdk.o
obj-$(CONFIG_MACH_TX25)			+= karo-tx25.o
obj-$(CONFIG_MACH_TX25_BASEBOARD_STK5)	+= stk5-baseboard.o
obj-$(CONFIG_MACH_TX25_CUSTOM)		+= stk5-custom.o

obj-$(CONFIG_PM)	 		+= pm.o
obj-$(CONFIG_MXC_SDMA_API)		+= dma.o
