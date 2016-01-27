
#ifdef DEBUG
extern int tx27_debug;
#define dbg_lvl(n)	((n) < tx27_debug)
#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#else
#define dbg_lvl(n)	0

#define DBG(lvl, fmt...)	do { } while (0)
#endif

#define MXC_PIN(port,gpio,fn,flags)	(GPIO_PORT##port | GPIO_##fn | (flags) | (gpio))

#ifdef DEBUG
#include <mach/iomux.h>
#include "crm_regs_mx27.h"

#define MXC_VADDR_RANGE(v,n)				\
	(((v)) >= n##_BASE_ADDR_VIRT) &&		\
	(((v)) < n##_BASE_ADDR_VIRT + n##_SIZE) ?	\
	((v)-n##_BASE_ADDR_VIRT + n##_BASE_ADDR) :

#define MXC_PHYS_ADDRESS(v)			\
	(unsigned long)(MXC_VADDR_RANGE(v,AIPI)	\
	MXC_VADDR_RANGE(v,SAHB1)		\
	MXC_VADDR_RANGE(v,X_MEMC)		\
	0UL)


#define SHOW_REG(reg) DBG(0, "%s[%08lx]=%08x\n", #reg, MXC_PHYS_ADDRESS(reg), __raw_readl(reg))
#define SHOW_GPIO_REG(reg, port)						\
	DBG(0, "PT%c_%s[%08lx]=%08x\n", 'A' + port, #reg,					\
	    GPIO_BASE_ADDR + MXC_##reg(port), __raw_readl(GPIO_BASE_ADDR_VIRT + MXC_##reg(port)))

static void __maybe_unused dump_regs(void)
{
	SHOW_REG(CCM_CSCR);
	SHOW_REG(CCM_MPCTL0);
	SHOW_REG(CCM_MPCTL1);
	SHOW_REG(CCM_SPCTL0);
	SHOW_REG(CCM_SPCTL1);
	SHOW_REG(CCM_OSC26MCTL);
	SHOW_REG(CCM_PCDR0);
	SHOW_REG(CCM_PCDR1);
	SHOW_REG(CCM_PCCR0);
	SHOW_REG(CCM_PCCR1);
	SHOW_REG(CCM_CCSR);
	SHOW_REG(CCM_PMCTL);
	SHOW_REG(CCM_PMCOUNT);
	SHOW_REG(CCM_WKGDCTL);
}
#else
static inline void dump_regs(void)
{
}
#define SHOW_REG(reg) do {} while (0)
#endif
