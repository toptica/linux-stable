/*
 * Copyright 2006-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_vpu.c
 *
 * @brief VPU system initialization and file operation implementation
 *
 * @ingroup VPU
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/autoconf.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/dma-mapping.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/clock.h>

#include <mach/mxc_vpu.h>

#ifdef CONFIG_MXC_VPU_IRAM
static int use_iram = 1;
module_param(use_iram, bool, S_IRUGO | S_IWUSR);
#else
static int use_iram;
module_param(use_iram, bool, 0);
#endif

#ifdef DEBUG
static int debug = 1;
module_param(debug, int, S_IRUGO | S_IWUSR);

#define dbg_lvl(n)	((n) < debug)
#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#else
static int debug;
module_param(debug, int, 0);

#define dbg_lvl(n)	0
#define DBG(lvl, fmt...)	do { } while (0)
#endif

#ifndef cpu_is_mx32
#define cpu_is_mx32() 0
#endif

static struct vpu_priv *vpu_data;

/* To track the allocated memory buffer */
typedef struct memalloc_record {
	struct list_head list;
	struct vpu_mem_desc mem;
} memalloc_record;

struct iram_setting {
	unsigned long start;
	unsigned long end;
};

static int open_count;

struct vpu_priv {
	struct fasync_struct	*async_queue;
	struct device *dev;

	spinlock_t		lock;
	struct mutex		shmem_mutex;
	struct list_head	head;

	int clk_enabled;
	struct class *class;
	int major;
	int irq;
	struct clk *clk;
	struct vpu_mem_desc bitwork_mem;
	struct vpu_mem_desc pic_para_mem;
	struct vpu_mem_desc user_data_mem;
	struct vpu_mem_desc share_mem;

	/* IRAM setting */
	struct iram_setting iram;
	void __iomem *src_base_addr;
	void __iomem *reg_base;

	int codec_done;
	wait_queue_head_t queue;

	u32 workctrl_regsave[6];
	u32 rd_ptr_regsave[4];
	u32 wr_ptr_regsave[4];
	u32 dis_flag_regsave[4];
};

#if 1
#define	READ_REG(x)		__read_reg(vpu->reg_base, x, __FUNCTION__, __LINE__, vpu)
static inline u32 __read_reg(void __iomem *base, unsigned int reg,
			const char *fn, int ln, struct vpu_priv *vpu)
{
	u32 val;
	BUG_ON(vpu != vpu_data);
	val = __raw_readl(base + reg);
	DBG(3, "%s@%d: Read %08x from reg %04x @ %p\n", fn, ln, val, reg, base + reg);
	return val;
}

#define	WRITE_REG(val, x)	__write_reg(vpu->reg_base, val, x, __FUNCTION__, __LINE__, vpu)
static inline void __write_reg(void __iomem *base, u32 val, unsigned int reg,
			const char *fn, int ln, struct vpu_priv *vpu)
{
	DBG(3, "%s@%d: Writing %08x to reg %04x @ %p\n", fn, ln, val, reg, base + reg);
	BUG_ON(vpu != vpu_data);
	__raw_writel(val, base + reg);
}
#else
#define	READ_REG(x)		__raw_readl(vpu->reg_base + (x))
#define	WRITE_REG(val, x)	__raw_writel((val), vpu->reg_base + (x))
#endif

#define	SAVE_WORK_REGS	do {						\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(vpu->workctrl_regsave) / 2; i++)	\
		vpu->workctrl_regsave[i] = READ_REG(BIT_WORK_CTRL_BUF_REG(i)); \
	} while (0)
#define	RESTORE_WORK_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(vpu->workctrl_regsave) / 2; i++)	\
		WRITE_REG(vpu->workctrl_regsave[i], BIT_WORK_CTRL_BUF_REG(i)); \
	} while (0)
#define	SAVE_CTRL_REGS	do {						\
	int i;								\
	for (i = ARRAY_SIZE(vpu->workctrl_regsave) / 2;			\
	     i < ARRAY_SIZE(vpu->workctrl_regsave); i++)		\
		vpu->workctrl_regsave[i] = READ_REG(BIT_WORK_CTRL_BUF_REG(i)); \
	} while (0)
#define	RESTORE_CTRL_REGS	do {					\
	int i;								\
	for (i = ARRAY_SIZE(vpu->workctrl_regsave) / 2;			\
	     i < ARRAY_SIZE(vpu->workctrl_regsave); i++)		\
		WRITE_REG(vpu->workctrl_regsave[i], BIT_WORK_CTRL_BUF_REG(i)); \
} while (0)
#define	SAVE_RDWR_PTR_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(vpu->rd_ptr_regsave); i++)		\
		vpu->rd_ptr_regsave[i] = READ_REG(BIT_RD_PTR_REG(i));	\
	for (i = 0; i < ARRAY_SIZE(vpu->wr_ptr_regsave); i++)		\
		vpu->wr_ptr_regsave[i] = READ_REG(BIT_WR_PTR_REG(i));	\
} while (0)
#define	RESTORE_RDWR_PTR_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(vpu->rd_ptr_regsave); i++)		\
		WRITE_REG(vpu->rd_ptr_regsave[i], BIT_RD_PTR_REG(i));	\
	for (i = 0; i < ARRAY_SIZE(vpu->wr_ptr_regsave); i++)		\
		WRITE_REG(vpu->wr_ptr_regsave[i], BIT_WR_PTR_REG(i));	\
} while (0)
#define	SAVE_DIS_FLAG_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(vpu->dis_flag_regsave); i++)		\
		vpu->dis_flag_regsave[i] = READ_REG(BIT_FRM_DIS_FLG_REG(i));	\
} while (0)
#define	RESTORE_DIS_FLAG_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(vpu->dis_flag_regsave); i++)		\
		WRITE_REG(vpu->dis_flag_regsave[i], BIT_FRM_DIS_FLG_REG(i));	\
} while (0)

static inline int mxc_vpu_clk_enable(struct vpu_priv *vpu)
{
	int ret = 0;

	if (vpu->clk_enabled++ == 0) {
		DBG(1, "%s: Enabling VPU clock\n", __FUNCTION__);
		ret = clk_enable(vpu->clk);
	} else {
		DBG(3, "%s: VPU clock is already enabled\n", __FUNCTION__);
	}
	return ret;
}

static inline int mxc_vpu_clk_disable(struct vpu_priv *vpu)
{
	if (WARN_ON(!vpu->clk_enabled))
		return -EINVAL;
	if (--vpu->clk_enabled == 0) {
		DBG(1, "%s: Disabling VPU clock\n", __FUNCTION__);
		clk_disable(vpu->clk);
	} else {
		DBG(3, "%s: VPU clock is already disabled\n", __FUNCTION__);
	}
	return 0;
}

/*!
 * Private function to alloc dma buffer
 * @return status  0 success.
 */
static int vpu_alloc_dma_buffer(struct vpu_priv *vpu, struct vpu_mem_desc *mem)
{
	mem->cpu_addr = dma_alloc_coherent(vpu->dev, PAGE_ALIGN(mem->size),
					&mem->phy_addr,
					GFP_DMA | GFP_KERNEL);
	pr_debug("[ALLOC] mem alloc cpu_addr = 0x%p\n", mem->cpu_addr);
	if (mem->cpu_addr == NULL) {
		dev_err(vpu->dev, "could not allocate %u byte DMA buffer\n",
			PAGE_ALIGN(mem->size));
		return -ENOMEM;
	}
	return 0;
}

/*!
 * Private function to free dma buffer
 */
static void vpu_free_dma_buffer(struct vpu_priv *vpu, struct vpu_mem_desc *mem)
{
	if (mem->cpu_addr != NULL) {
		dma_free_coherent(vpu->dev, PAGE_ALIGN(mem->size),
				  mem->cpu_addr, mem->phy_addr);
	}
}

/*!
 * Private function to free buffers
 * @return status  0 success.
 */
static int vpu_free_buffers(struct vpu_priv *vpu)
{
	struct memalloc_record *rec, *n;
	struct vpu_mem_desc mem;

	list_for_each_entry_safe(rec, n, &vpu->head, list) {
		mem = rec->mem;
		if (mem.cpu_addr != NULL) {
			vpu_free_dma_buffer(vpu, &mem);
			pr_debug("[FREE] freed paddr=0x%08X\n", mem.phy_addr);
			/* delete from list */
			list_del(&rec->list);
			kfree(rec);
		}
	}

	return 0;
}

static unsigned int irq_count;

/*!
 * @brief vpu interrupt handler
 */
static irqreturn_t vpu_irq_handler(int irq, void *dev_id)
{
	struct vpu_priv *vpu = dev_id;

	BUG_ON(vpu != vpu_data);
	READ_REG(BIT_INT_STATUS);
	WRITE_REG(0x1, BIT_INT_CLEAR);

	irq_count++;

	if (vpu->async_queue)
		kill_fasync(&vpu->async_queue, SIGIO, POLL_IN);

	BUG_ON(vpu->codec_done < 0);
	vpu->codec_done++;
	BUG_ON(vpu->codec_done == 0);
	wake_up_interruptible(&vpu->queue);
	DBG(2, "%s: codec_done=%d\n", __FUNCTION__, vpu->codec_done);

	return IRQ_HANDLED;
}

/*!
 * @brief open function for vpu file operation
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_open(struct inode *inode, struct file *filp)
{
	struct vpu_priv *vpu = vpu_data;

	spin_lock(&vpu->lock);
	if ((open_count++ == 0) && cpu_is_mx32())
		vl2cc_enable();
	filp->private_data = vpu_data;
	spin_unlock(&vpu->lock);
	return 0;
}

/*!
 * @brief IO ctrl function for vpu file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */
static int vpu_ioctl(struct inode *inode, struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret;
	struct vpu_priv *vpu = filp->private_data;

	DBG(2, "%s: cmd=%08x data=%08lx\n", __FUNCTION__, cmd, arg);
	BUG_ON(vpu != vpu_data);

	switch (cmd) {
	case VPU_IOC_PHYMEM_ALLOC:
		{
			struct memalloc_record *rec;

			rec = kzalloc(sizeof(*rec), GFP_KERNEL);
			if (!rec)
				return -ENOMEM;

			if (copy_from_user(&rec->mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc))) {
				kfree(rec);
				return -EFAULT;
			}

			pr_debug("[ALLOC] mem alloc size = 0x%08x\n",
				 rec->mem.size);

			ret = vpu_alloc_dma_buffer(vpu, &rec->mem);
			if (ret) {
				kfree(rec);
				break;
			}
			if (copy_to_user((void __user *)arg, &rec->mem,
						sizeof(struct vpu_mem_desc))) {
				kfree(rec);
				ret = -EFAULT;
				break;
			}

			spin_lock(&vpu->lock);
			list_add(&rec->list, &vpu->head);
			spin_unlock(&vpu->lock);
		}
		break;

	case VPU_IOC_PHYMEM_FREE:
		{
			struct memalloc_record *rec, *n;
			struct vpu_mem_desc vpu_mem;

			if (copy_from_user(&vpu_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc)))
				return -EFAULT;

			pr_debug("[FREE] mem freed cpu_addr = %p\n",
				 vpu_mem.cpu_addr);
			if (vpu_mem.cpu_addr != NULL)
				vpu_free_dma_buffer(vpu, &vpu_mem);

			spin_lock(&vpu->lock);
			list_for_each_entry_safe(rec, n, &vpu->head, list) {
				if (rec->mem.cpu_addr == vpu_mem.cpu_addr) {
					/* delete from list */
					list_del(&rec->list);
					kfree(rec);
					break;
				}
			}
			spin_unlock(&vpu->lock);
			ret = 0;
		}
		break;

	case VPU_IOC_WAIT4INT:
	{
		unsigned long flags;
		static unsigned int loop;

		/*
		 * arg == 0: check VPU status and return immediately
		 *           => actually O_NONBLOCK should be used for this,
		 *              but userspace has to be modified
		 * arg < 0: wait for VPU ready
		 * arg > 0: wait specified nr of msecs for VPU ready
		 */
		if (!(filp->f_flags & O_NONBLOCK) && arg != 0) {
			if (arg > 0) {
				DBG(2, "%s: wait for VPU with timeout: %ld; loop: %u; irqs: %u; codec_done: %d\n",
					__FUNCTION__, arg, loop, irq_count,
					vpu->codec_done);
				ret = wait_event_interruptible_timeout(vpu->queue,
								vpu->codec_done,
								msecs_to_jiffies(arg * 100));
			} else {
				ret = wait_event_interruptible(vpu->queue,
							vpu->codec_done);
			}
		} else {
			DBG(2, "%s: Check for VPU ready; loop: %u; irqs: %u; codec_done: %d\n", __FUNCTION__,
				loop, irq_count, vpu->codec_done);
			ret = 0;
		}

		spin_lock_irqsave(&vpu->lock, flags);
		if (vpu->codec_done) {
			vpu->codec_done--;
			BUG_ON(vpu->codec_done < 0);
			ret = 0;
			DBG(1, "VPU ready loop: %u; irqs: %u; codec_done: %d\n",
				loop, irq_count, vpu->codec_done);
		} else if (ret == 0) {
			if (filp->f_flags & O_NONBLOCK || arg == 0) {
				ret = -EAGAIN;
			} else {
				dev_warn(vpu->dev, "VPU timeout: %ld codec_done: %d\n",
					arg, vpu->codec_done);
				ret = -ETIME;
			}
		} else if (signal_pending(current)) {
			dev_warn(vpu->dev, "VPU wait interrupted: %d\n",
				vpu->codec_done);
			ret = -EINTR;
		} else {
			DBG(-1, "%s: wait_event_interruptible_timeout() returned: %d\n",
				__FUNCTION__, ret);
		}
		if (vpu->codec_done > 0)
			DBG(0, "VPU codec_done: %d\n", vpu->codec_done);
		spin_unlock_irqrestore(&vpu->lock, flags);
		loop++;
	}
	break;

	case VPU_IOC_VL2CC_FLUSH:
		if (cpu_is_mx32()) {
			vl2cc_flush();
		}
		ret = 0;
		break;

	case VPU_IOC_IRAM_SETTING:
		BUG();
		if (!use_iram)
			return -EOPNOTSUPP;
		if (copy_to_user((void __user *)arg, &vpu->iram,
					sizeof(struct iram_setting)))
			return -EFAULT;
		DBG(0, "%s: IRAM: %08lx:%08lx\n", __FUNCTION__,
			vpu->iram.start, vpu->iram.end);
		ret = 0;
		break;

	case VPU_IOC_CLKGATE_SETTING:
	{
		u32 clkgate_en;

		if (get_user(clkgate_en, (u32 __user *)arg))
			return -EFAULT;

		DBG(1, "%s: %sabling VPU clock: %d codec_done: %d\n", __FUNCTION__,
			clkgate_en ? "En" : "Dis", vpu->clk_enabled, vpu->codec_done);
		if (clkgate_en)
			ret = mxc_vpu_clk_enable(vpu);
		else
			ret = mxc_vpu_clk_disable(vpu);
	}
	break;

	case VPU_IOC_GET_SHARE_MEM:
		mutex_lock(&vpu->shmem_mutex);
		if (vpu->share_mem.cpu_addr != NULL) {
			ret = copy_to_user((void __user *)arg,
					&vpu->share_mem,
					sizeof(struct vpu_mem_desc));
		} else {
			if (copy_from_user(&vpu->share_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc))) {
				mutex_unlock(&vpu->shmem_mutex);
				return -EFAULT;
			}
			ret = vpu_alloc_dma_buffer(vpu, &vpu->share_mem);
			if (ret)
				break;

			if (copy_to_user((void __user *)arg,
						&vpu->share_mem,
						sizeof(struct vpu_mem_desc)))
				ret = -EFAULT;
		}
		mutex_unlock(&vpu->shmem_mutex);
		break;

	case VPU_IOC_GET_WORK_ADDR:
		if (vpu->bitwork_mem.cpu_addr != NULL) {
			ret = copy_to_user((void __user *)arg,
					&vpu->bitwork_mem,
					sizeof(struct vpu_mem_desc));
		} else {
			if (copy_from_user(&vpu->bitwork_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc)))
				return -EFAULT;

			ret = vpu_alloc_dma_buffer(vpu, &vpu->bitwork_mem);
			if (ret)
				return ret;

			if (copy_to_user((void __user *)arg,
						&vpu->bitwork_mem,
						sizeof(struct vpu_mem_desc)))
				ret = -EFAULT;
		}
		break;

	case VPU_IOC_GET_PIC_PARA_ADDR:
		if (vpu->pic_para_mem.cpu_addr != NULL) {
			if (copy_to_user((void __user *)arg,
						&vpu->pic_para_mem,
						sizeof(struct vpu_mem_desc)))
				return -EFAULT;
			ret = 0;
		} else {
			if (copy_from_user(&vpu->pic_para_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc)))
				return -EFAULT;

			ret = vpu_alloc_dma_buffer(vpu, &vpu->pic_para_mem);
			if (ret)
				break;

			if (copy_to_user((void __user *)arg,
						&vpu->pic_para_mem,
						sizeof(struct
							vpu_mem_desc)))
				ret = -EFAULT;
		}
		break;

	case VPU_IOC_GET_USER_DATA_ADDR:
		if (vpu->user_data_mem.cpu_addr != NULL) {
			if (copy_to_user((void __user *)arg,
						&vpu->user_data_mem,
						sizeof(struct vpu_mem_desc)))
				return -EFAULT;
			ret = 0;
		} else {
			if (copy_from_user(&vpu->user_data_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc)))
				return -EFAULT;

			ret = vpu_alloc_dma_buffer(vpu, &vpu->user_data_mem);
			if (ret)
				break;

			if (copy_to_user((void __user *)arg,
						&vpu->user_data_mem,
						sizeof(struct vpu_mem_desc)))
				ret = -EFAULT;
		}
		break;

	case VPU_IOC_SYS_SW_RESET:
		{
			u32 reg;
			u32 rst_flag;
			unsigned long now = jiffies;

			if (vpu->src_base_addr == NULL)
				return -EOPNOTSUPP;

			if (cpu_is_mx37()) {
				reg = __raw_readl(vpu->src_base_addr);
				reg |= 0x02;	/* SW_VPU_RST_BIT */
				__raw_writel(reg, vpu->src_base_addr);
				rst_flag = 1 << 1;
			} else if (cpu_is_mx51()) {
				/* mask interrupt due to vpu passed reset */
				reg = __raw_readl(vpu->src_base_addr + 0x14);
				reg |= 0x02;
				__raw_writel(reg, vpu->src_base_addr + 0x14);

				reg = __raw_readl(vpu->src_base_addr);
				reg |= 0x5;    /* warm reset vpu */
				__raw_writel(reg, vpu->src_base_addr);
				rst_flag = 1 << 2;
			} else {
				return 0;
			}
			while (__raw_readl(vpu->src_base_addr) & rst_flag) {
				cpu_relax();
				if (time_after(jiffies, now + 2))
					break;
			}
			ret = (__raw_readl(vpu->src_base_addr) & rst_flag) ?
				-ETIME : 0;
		}
		break;

	case VPU_IOC_REG_DUMP:
		ret = -EOPNOTSUPP;
		break;

	case VPU_IOC_PHYMEM_DUMP:
		ret = -EOPNOTSUPP;
		break;

	default:
		dev_err(vpu->dev, "No such IOCTL, cmd is %d\n", cmd);
		ret = -EINVAL;
	}
	return ret;
}

/*!
 * @brief Release function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_release(struct inode *inode, struct file *filp)
{
	struct vpu_priv *vpu = filp->private_data;

	spin_lock(&vpu->lock);
	if (open_count > 0 && !--open_count) {
		vpu_free_buffers(vpu);

		if (cpu_is_mx32())
			vl2cc_disable();

		/* Free shared memory when vpu device is idle */
		vpu_free_dma_buffer(vpu, &vpu->share_mem);
		vpu->share_mem.cpu_addr = NULL;
	}
	spin_unlock(&vpu->lock);
	if (vpu->clk_enabled) {
		DBG(0, "%s: VPU IRQs: %d clk_enabled=%d\n", __FUNCTION__,
			irq_count, vpu->clk_enabled);
		clk_disable(vpu->clk);
		vpu->clk_enabled = 0;
	}
	return 0;
}

/*!
 * @brief fasync function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_fasync(int fd, struct file *filp, int mode)
{
	struct vpu_priv *vpu = filp->private_data;
	return fasync_helper(fd, filp, mode, &vpu->async_queue);
}

/*!
 * @brief memory map function of harware registers for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_hwregs(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = VPU_BASE_ADDR >> PAGE_SHIFT;
	pr_debug("size=0x%08lx,  page no=0x%08lx\n",
		vm->vm_end - vm->vm_start, pfn);
	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

/*!
 * @brief memory map function of memory for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_mem(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long request_size = vm->vm_end - vm->vm_start;

	pr_debug(" start=0x%08lx, pgoff=0x%08lx, size=0x%08lx\n",
		vm->vm_start, vm->vm_pgoff,
		request_size);

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			       request_size, vm->vm_page_prot) ? -EAGAIN : 0;

}

/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff)
		return vpu_map_mem(fp, vm);
	else
		return vpu_map_hwregs(fp, vm);
}

static struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.ioctl = vpu_ioctl,
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
};

/*!
 * This function is called by the driver framework to initialize the vpu device.
 * @param   dev The device structure for the vpu passed in by the framework.
 * @return   0 on success or negative error code on error
 */
static int mxc_vpu_probe(struct platform_device *pdev)
{
	int err;
	struct device *temp_class;
	struct resource *res1, *res2 = NULL, *res3 = NULL;
	struct vpu_priv *vpu;

	vpu = kzalloc(sizeof(*vpu), GFP_KERNEL);
	if (vpu == NULL)
		return -ENOMEM;

	init_waitqueue_head(&vpu->queue);
	spin_lock_init(&vpu->lock);
	INIT_LIST_HEAD(&vpu->head);
	mutex_init(&vpu->shmem_mutex);

	vpu->dev = &pdev->dev;
	DBG(0, "%s: dev=%p vpu->dev=%p\n", __FUNCTION__, &pdev->dev, vpu->dev);

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res1) {
		err = -ENODEV;
		goto err_free;
	}

	err = platform_get_irq(pdev, 0);
	if (err < 0)
		goto err_free;

	vpu->irq = err;

	if (!request_mem_region(res1->start,
					resource_size(res1), "VPU REG")) {
		err = -EBUSY;
		goto err_free;
	}

	if (cpu_is_mx32()) {
		/* Obtain VL2CC base address */
		res2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res2 == NULL) {
			dev_err(&pdev->dev, "vpu: unable to get VL2CC base\n");
			err = -ENODEV;
			goto release1;
		}

		if (!request_mem_region(res2->start, resource_size(res2),
						"VL2CC")) {
			err = -EBUSY;
			goto release1;
		}

		err = vl2cc_init(res2->start, resource_size(res2));
		if (err != 0)
			goto release2;
	}

	if (cpu_is_mx37() || cpu_is_mx51()) {
		res2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res2 == NULL) {
			dev_err(&pdev->dev, "vpu: unable to get src base addr\n");
			err = -ENODEV;
			goto release1;
		}
		if (!request_mem_region(res2->start, resource_size(res2),
						"VPU SRC")) {
			err = -EBUSY;
			goto release1;
		}
#ifdef CONFIG_MXC_VPU_IRAM
		res3 = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (res3 != NULL) {
			if (!request_mem_region(res3->start, resource_size(res3),
							"VPU IRAM")) {
				err = -EBUSY;
				goto release2;
			}
			vpu->iram.start = res3->start;
			vpu->iram.end = res3->end;
			DBG(0, "%s: Using IRAM %08lx..%08lx\n", __FUNCTION__,
				(unsigned long)vpu->iram.start,
				(unsigned long)vpu->iram.end);
		} else {
			DBG(0, "%s: Failed to get IRAM resource\n", __FUNCTION__);
		}
#endif
		vpu->src_base_addr = ioremap(res2->start, resource_size(res2));
		if (vpu->src_base_addr == NULL) {
			err = -ENOMEM;
			goto release3;
		}
		DBG(0, "%s: SRC registers[%08lx..%08lx] remapped to %p\n", __FUNCTION__,
			(unsigned long)res2->start, (unsigned long)res2->end,
			vpu->src_base_addr);
	}

	vpu->reg_base = ioremap(res1->start, resource_size(res1));
	if (vpu->reg_base == NULL) {
		err = -ENOMEM;
		goto unmap;
	}
	DBG(0, "%s: VPU registers[%08lx..%08lx] remapped to %p\n", __FUNCTION__,
		(unsigned long)res1->start, (unsigned long)res1->end,
		vpu->reg_base);

	platform_set_drvdata(pdev, vpu);
	vpu_data = vpu;

	vpu->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(vpu->clk)) {
		err = PTR_ERR(vpu->clk);
		goto err_clk;
	}

	err = request_irq(vpu->irq, vpu_irq_handler, 0, "VPU_CODEC_IRQ", vpu);
	if (err)
		goto err_irq;

	vpu->major = register_chrdev(vpu->major, "mxc_vpu", &vpu_fops);
	if (vpu->major < 0) {
		dev_err(&pdev->dev, "vpu: unable to get a major for VPU\n");
		err = vpu->major;
		goto err_chrdev;
	}

	vpu->class = class_create(THIS_MODULE, "mxc_vpu");
	if (IS_ERR(vpu->class)) {
		err = PTR_ERR(vpu->class);
		goto err_class;
	}

	temp_class = device_create(vpu->class, NULL, MKDEV(vpu->major, 0),
				   NULL, "mxc_vpu");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_device;
	}

	DBG(0, "%s: dev=%p vpu->dev=%p\n", __FUNCTION__, &pdev->dev, vpu->dev);
	dev_info(&pdev->dev, "VPU initialized\n");
	return 0;

err_device:
	class_destroy(vpu->class);
err_class:
	unregister_chrdev(vpu->major, "mxc_vpu");
err_chrdev:
	free_irq(vpu->irq, vpu);
err_irq:
	clk_put(vpu->clk);
err_clk:
	iounmap(vpu->reg_base);
unmap:
	if (vpu->src_base_addr != NULL)
		iounmap(vpu->src_base_addr);

	if (cpu_is_mx32())
		vl2cc_cleanup();
release3:
	if (res3 != NULL)
		release_mem_region(res3->start, resource_size(res3));
release2:
	if (res2 != NULL)
		release_mem_region(res2->start, resource_size(res2));
release1:
	release_mem_region(res1->start, resource_size(res1));
err_free:
	kfree(vpu);
	return err;
}

static int __exit mxc_vpu_remove(struct platform_device *pdev)
{
	struct vpu_priv *vpu = platform_get_drvdata(pdev);
	struct resource *res;

	device_destroy(vpu->class, MKDEV(vpu->major, 0));
	class_destroy(vpu->class);
	unregister_chrdev(vpu->major, "mxc_vpu");

	free_irq(vpu->irq, vpu);

	if (cpu_is_mx32()) {
		vl2cc_cleanup();
	}

	vpu_free_dma_buffer(vpu, &vpu->bitwork_mem);
	vpu_free_dma_buffer(vpu, &vpu->pic_para_mem);
	vpu_free_dma_buffer(vpu, &vpu->user_data_mem);

	iounmap(vpu->reg_base);
	if (vpu->src_base_addr)
		iounmap(vpu->src_base_addr);

	if (vpu->clk_enabled)
		clk_disable(vpu->clk);
	clk_put(vpu->clk);
	kfree(vpu);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	BUG_ON(res == NULL);
	release_mem_region(res->start, resource_size(res));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res != NULL)
		release_mem_region(res->start, resource_size(res));

#ifdef CONFIG_MXC_VPU_IRAM
	if (!cpu_is_mx32()) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (res)
			release_mem_region(res->start, resource_size(res));
	}
#endif
	return 0;
}

static inline int mxc_vpu_wait_ready(struct vpu_priv *vpu, unsigned long timeout)
{
	timeout += jiffies;
	while (READ_REG(BIT_BUSY_FLAG)) {
		if (time_after(jiffies, timeout))
			break;
		msleep(1);
	}
	return READ_REG(BIT_BUSY_FLAG) ? -ETIME : 0;
}

#ifdef CONFIG_PM
static int mxc_vpu_suspend(struct device *dev)
{
	struct vpu_priv *vpu = dev_get_drvdata(dev);

	/* Wait for vpu go to idle state, suspect vpu cannot be changed
	   to idle state after about 1 sec */

	clk_enable(vpu->clk);
	if (open_count && mxc_vpu_wait_ready(vpu, 1 * HZ))
		goto out;

	if (vpu->bitwork_mem.cpu_addr != NULL) {
		SAVE_WORK_REGS;
		SAVE_CTRL_REGS;
		SAVE_RDWR_PTR_REGS;
		SAVE_DIS_FLAG_REGS;

		WRITE_REG(0x1, BIT_BUSY_FLAG);
		WRITE_REG(VPU_SLEEP_REG_VALUE, BIT_RUN_COMMAND);
		if (mxc_vpu_wait_ready(vpu, 1 * HZ))
			goto out;
	}
	clk_disable(vpu->clk);

	/* Make sure clock is disabled before suspend */
	if (vpu->clk_enabled)
		clk_disable(vpu->clk);

	if (cpu_is_mx37() || cpu_is_mx51())
		mxc_pg_enable(to_platform_device(dev));
	return 0;

out:
	clk_disable(vpu->clk);
	return -EAGAIN;
}

static int mxc_vpu_resume(struct device *dev)
{
	int ret = 0;
	int i;
	struct vpu_priv *vpu = dev_get_drvdata(dev);

	if (cpu_is_mx37() || cpu_is_mx51())
		mxc_pg_disable(to_platform_device(dev));

	clk_enable(vpu->clk);
	if (vpu->bitwork_mem.cpu_addr != NULL) {
		u32 *p = vpu->bitwork_mem.cpu_addr;
		u32 data;
		u16 data_hi;
		u16 data_lo;

		RESTORE_WORK_REGS;

		WRITE_REG(0x0, BIT_RESET_CTRL);
		WRITE_REG(0x0, BIT_CODE_RUN);

		/*
		 * Re-load boot code, from the codebuffer in external RAM.
		 * Thankfully, we only need 4096 bytes, same for all platforms.
		 */
		if (cpu_is_mx51()) {
			for (i = 0; i < 2048; i += 4) {
				data = p[(i / 2) + 1];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG((i << 16) | data_hi, BIT_CODE_DOWN);
				WRITE_REG(((i + 1) << 16) | data_lo,
					  BIT_CODE_DOWN);

				data = p[i / 2];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG(((i + 2) << 16) | data_hi,
					  BIT_CODE_DOWN);
				WRITE_REG(((i + 3) << 16) | data_lo,
					  BIT_CODE_DOWN);
			}
		} else {
			for (i = 0; i < 2048; i += 2) {
				if (cpu_is_mx37())
					data = swab32(p[i / 2]);
				else
					data = p[i / 2];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;

				WRITE_REG((i << 16) | data_hi, BIT_CODE_DOWN);
				WRITE_REG(((i + 1) << 16) | data_lo,
					  BIT_CODE_DOWN);
			}
		}

		RESTORE_CTRL_REGS;

		WRITE_REG(BITVAL_PIC_RUN, BIT_INT_ENABLE);

		WRITE_REG(0x1, BIT_BUSY_FLAG);
		WRITE_REG(0x1, BIT_CODE_RUN);
		ret = mxc_vpu_wait_ready(vpu, 1 * HZ);
		if (ret == 0) {
			RESTORE_RDWR_PTR_REGS;
			RESTORE_DIS_FLAG_REGS;

			WRITE_REG(0x1, BIT_BUSY_FLAG);
			WRITE_REG(VPU_WAKE_REG_VALUE, BIT_RUN_COMMAND);
			ret = mxc_vpu_wait_ready(vpu, 1 * HZ);
		}
	}

	/* Restore vpu clock to its state before suspending */
	if (!vpu->clk_enabled)
		clk_disable(vpu->clk);

	return ret;
}
#else
#define	vpu_suspend	NULL
#define	vpu_resume	NULL
#endif	/* !CONFIG_PM */

/*! Driver definition
 *
 */
static struct dev_pm_ops vpu_pm_ops = {
	.suspend = mxc_vpu_suspend,
	.resume = mxc_vpu_resume,
};

static struct platform_driver mxc_vpu_driver = {
	.driver = {
		.name = "mxc_vpu",
		.pm = __dev_pm_ops_p(vpu_pm_ops),
	},
	.probe = mxc_vpu_probe,
	.remove = __exit_p(mxc_vpu_remove),
};

static int __init mxc_vpu_init(void)
{
	int ret = platform_driver_register(&mxc_vpu_driver);
	return ret;
}
module_init(mxc_vpu_init);

static void __exit mxc_vpu_exit(void)
{
	platform_driver_unregister(&mxc_vpu_driver);
	return;
}
module_exit(mxc_vpu_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxc_vpu");
