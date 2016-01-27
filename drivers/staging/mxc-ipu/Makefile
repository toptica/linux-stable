# kbuild trick to avoid linker error. Can be omitted if a module is built.
#obj- := dummy.o

ccflags-$(CONFIG_FB_MXC_DEBUG)		+= -DDEBUG

obj-$(CONFIG_FB_MXC_IPU_V3)		+= mxc_ipuv3_fb.o
obj-$(CONFIG_MXC_IPU_V3)		+= ipu_v3/
