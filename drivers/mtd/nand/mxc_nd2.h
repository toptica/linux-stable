/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_nd2.h
 *
 * @brief This file contains the NAND Flash Controller register information.
 *
 *
 * @ingroup NAND_MTD
 */

#ifndef __MXC_ND2_H__
#define __MXC_ND2_H__

#include <mach/hardware.h>

struct mxc_nand_host {
	struct mtd_info mtd;
	struct nand_chip nand;
	struct mtd_partition *parts;
	struct device *dev;

	void __iomem		*axi_base;
	void __iomem		*nfc_buf;
	void __iomem		*ip_base;
	int			spare_only;
	int			status_request;
	int			pagesize_2k;
	uint16_t		col_addr;
	unsigned int		page_addr;
	struct clk		*clk;
	int			clk_act;
	int			irq;

	wait_queue_head_t	irq_waitq;
};

#define nfc_is_v1()	cpu_is_mx2()
#define nfc_is_v2()	(cpu_is_mx51() || cpu_is_mx35())
#define nfc_is_v2_1()	(cpu_is_mx25() || cpu_is_mx35())
#define nfc_is_v3()	(cpu_is_mx51() || cpu_is_mx37())
#define nfc_is_v3_1()	cpu_is_mx37()
#define nfc_is_v3_2()	cpu_is_mx51()

#define IS_2K_PAGE_NAND		(mtd->writesize == NAND_PAGESIZE_2KB)
#define IS_4K_PAGE_NAND		(mtd->writesize == NAND_PAGESIZE_4KB)
#define IS_LARGE_PAGE_NAND      (mtd->writesize > 512)

#define NAND_PAGESIZE_2KB	2048
#define NAND_PAGESIZE_4KB	4096

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3
/*
 * For V3 NFC registers Definition
 */
#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1)	/* mx37 */
#define MXC_INT_NANDFC			MXC_INT_EMI
#define NFC_FLASH_ADDR_CMD		(host->axi_base + 0x00)
#define NFC_CONFIG1			(host->axi_base + 0x04)
#define NFC_ECC_STATUS_RESULT		(host->axi_base + 0x08)
#define LAUNCH_NFC			(host->axi_base + 0x0c)
#define NFC_WRPROT			(host->ip_base + 0x00)
#define NFC_WRPROT_UNLOCK_BLK_ADD0	(host->ip_base + 0x04)
#define NFC_CONFIG2			(host->ip_base + 0x14)
#define NFC_IPC				(host->ip_base + 0x18)
#elif defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)	/* mx51 */
#define MXC_INT_NANDFC			MXC_INT_NFC
#define NFC_AUTO_MODE_ENABLE
#define NFC_FLASH_CMD			(host->axi_base + 0x00)
#define NFC_FLASH_ADDR0			(host->axi_base + 0x04)
#define NFC_FLASH_ADDR8			(host->axi_base + 0x24)
#define NFC_CONFIG1			(host->axi_base + 0x34)
#define NFC_ECC_STATUS_RESULT		(host->axi_base + 0x38)
#define NFC_ECC_STATUS_SUM		(host->axi_base + 0x3C)
#define LAUNCH_NFC			(host->axi_base + 0x40)
#define NFC_WRPROT			(host->ip_base + 0x00)
#define NFC_WRPROT_UNLOCK_BLK_ADD0	(host->ip_base + 0x04)
#define NFC_CONFIG2			(host->ip_base + 0x24)
#define NFC_CONFIG3			(host->ip_base + 0x28)
#define NFC_IPC				(host->ip_base + 0x2C)
#define NFC_DELAY_LINE			(host->ip_base + 0x34)
#else				/* skye */
#define NFC_FLASH_ADDR_CMD		(host->axi_base + 0x00)
#define NFC_CONFIG1			(host->axi_base + 0x04)
#define NFC_ECC_STATUS_RESULT		(host->axi_base + 0x08)
#define LAUNCH_NFC			(host->axi_base + 0x0C)
#define NFC_WRPROT			(host->ip_base + 0x00)
#define NFC_WRPROT_UNLOCK_BLK_ADD0	(host->ip_base + 0x04)
#define NFC_CONFIG2			(host->ip_base + 0x14)
#define NFC_IPC				(host->ip_base + 0x18)
#endif
/*!
 * Addresses for NFC RAM BUFFER Main area 0
 */
#define MAIN_AREA0			0x000
#define MAIN_AREA1			0x200

/*!
 * Addresses for NFC SPARE BUFFER Spare area 0
 */
#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1) || defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define SPARE_AREA0			0x1000
#define SPARE_LEN			64
#define SPARE_COUNT			8
#define SPARE_SIZE			(SPARE_LEN * SPARE_COUNT)
#else
#define SPARE_AREA0			0x800
#define SPARE_LEN			16
#define SPARE_COUNT			4
#define SPARE_SIZE			(SPARE_LEN * SPARE_COUNT)
#endif

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1) || defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define NFC_SPAS_WIDTH			8
#define NFC_SPAS_SHIFT			16
#define NFC_SPAS_MASK			(((1 << NFC_SPAS_WIDTH) - 1) << \
				NFC_SPAS_SHIFT)
#define NFC_ECC_MODE_4			(1 << 6)
#define NFC_ECC_MODE_MASK		(1 << 6)

#define NFC_ST_CMD_SHIFT		24
#define NFC_ST_CMD_WIDTH		8
#define NFC_ST_CMD_MASK			(((1 << NFC_ST_CMD_WIDTH) - 1) << NFC_ST_CMD_SHIFT)

#ifdef CONFIG_MACH_MX51
#define IS_4BIT_ECC (mx51_revision() > MX51_CHIP_REV_2_0 ?		\
			!(raw_read(NFC_CONFIG2) & NFC_ECC_MODE_4) :	\
			(raw_read(NFC_CONFIG2) & NFC_ECC_MODE_4))
#else
#define IS_4BIT_ECC	(raw_read(NFC_CONFIG2) & NFC_ECC_MODE_4)
#endif

#define NFC_IPC_CREQ			(1 << 0)
#define NFC_IPC_ACK			(1 << 1)

#ifdef DEBUG
#define raw_read(addr)		_raw_read(addr, __FUNCTION__, __LINE__)
static inline unsigned int _raw_read(void __iomem *addr,
			const char *fn, int ln)
{
	unsigned int val;

	val = __raw_readl(addr);
	DBG(3, "%s@%d: Read %08x from %p\n", fn, ln, val, addr);
	return val;
}

#define raw_write(val, addr)	_raw_write(val, addr, host, __FUNCTION__, __LINE__)
static inline void _raw_write(unsigned int val, void __iomem *addr,
			struct mxc_nand_host *host,
			const char *fn, int ln)
{
	DBG(3, "%s@%d: Writing %08x to %p\n", fn, ln, val, addr);
	__raw_writel(val, addr);
}

#else
#define raw_write(v,a)		__raw_writel(v,a)
#define raw_read(a)		__raw_readl(a)
#endif

static inline void NFC_SET_SPAS(struct mxc_nand_host *host, unsigned int val)
{
	raw_write((raw_read(NFC_CONFIG2) & ~NFC_SPAS_MASK) |
		(val << 16), NFC_CONFIG2);
}
#define NFC_SPAS_16			8
#define NFC_SPAS_64			32
#define NFC_SPAS_128			64
#define NFC_SPAS_112			56
#define NFC_SPAS_218			109

static inline void NFC_SET_ECC_MODE(struct mxc_nand_host *host,
				unsigned int val)
{
	if (mx51_revision() > MX51_CHIP_REV_2_0) {
		if (val == NFC_SPAS_218 || val == NFC_SPAS_112) {
			raw_write((raw_read(NFC_CONFIG2) |
					NFC_ECC_MODE_4), NFC_CONFIG2);
		} else
			raw_write((raw_read(NFC_CONFIG2) &
					~NFC_ECC_MODE_4), NFC_CONFIG2);
	} else {
#ifdef CONFIG_MACH_TX51
		printk(KERN_EMERG "Invalid processor revision: %02x",
			mx51_revision());
		BUG();
#endif
		if (val == NFC_SPAS_218 || val == NFC_SPAS_112)
			raw_write((raw_read(NFC_CONFIG2) &
					~NFC_ECC_MODE_4), NFC_CONFIG2);
		else
			raw_write((raw_read(NFC_CONFIG2) |
					NFC_ECC_MODE_4), NFC_CONFIG2);
	}
}
#else
#define IS_4BIT_ECC			1
#define NFC_SET_SPAS(v)
#define NFC_SET_ECC_MODE(v)
#define NFC_SET_NFMS(v)	(NFMS |= (v))
#endif

#define GET_NFC_ECC_STATUS() raw_read(REG_NFC_ECC_STATUS_RESULT)

/*!
 * Set 1 to specific operation bit, rest to 0 in LAUNCH_NFC Register for
 * Specific operation
 */
#define NFC_CMD				0x1
#define NFC_ADDR			0x2
#define NFC_INPUT			0x4
#define NFC_OUTPUT			0x8
#define NFC_ID				0x10
#define NFC_STATUS			0x20

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2	/* mx51 */
#define NFC_AUTO_PROG			0x40
#define NFC_AUTO_READ			0x80
#define NFC_AUTO_ERASE			0x200
#define NFC_COPY_BACK_0			0x400
#define NFC_COPY_BACK_1			0x800
#define NFC_AUTO_STATE			0x1000
#endif

/* Bit Definitions for NFC_IPC*/
#define NFC_OPS_STAT			(1 << 31)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2	/* mx51 */
#define NFC_OP_DONE			(1 << 30)
#define NFC_RB				(1 << 28)
#define NFC_PS_WIDTH			2
#define NFC_PS_SHIFT			0
#define NFC_PS_512			0
#define NFC_PS_2K			1
#define NFC_PS_4K			2
#define NFC_PS_MASK			(((1 << NFC_PS_WIDTH) - 1) << \
					 NFC_PS_SHIFT)
#else
#define NFC_RB				(1 << 29)
#endif

#define NFC_ONE_CYCLE			(1 << 2)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2	/* mx51 */
#define NFC_INT_MSK			(1 << 15)
#define NFC_AUTO_PROG_DONE_MSK		(1 << 14)
#define NFC_NUM_ADDR_PHASE1_WIDTH	2
#define NFC_NUM_ADDR_PHASE1_SHIFT	12
#define NFC_NUM_ADDR_PHASE1_MASK	(((1 << NFC_NUM_ADDR_PHASE1_WIDTH) - 1) << \
					NFC_NUM_ADDR_PHASE1_SHIFT)

#define NFC_NUM_ADDR_PHASE0_WIDTH	1
#define NFC_NUM_ADDR_PHASE0_SHIFT	5
#define NFC_NUM_ADDR_PHASE0_MASK	(((1 << NFC_NUM_ADDR_PHASE0_WIDTH) - 1) << \
					NFC_NUM_ADDR_PHASE0_SHIFT)

#define NFC_ONE_LESS_PHASE1		0
#define NFC_TWO_LESS_PHASE1		1

#define NFC_FLASH_ADDR_SHIFT		0
#else
#define NFC_INT_MSK			(1 << 4)
#define NFC_BIG				(1 << 5)
#define NFC_FLASH_ADDR_SHIFT		16
#endif

#define NFC_UNLOCK_END_ADDR_SHIFT	16

/* Bit definitions for NFC_CONFIGRATION1 */
#define NFC_SP_EN			(1 << 0)
#define NFC_CE				(1 << 1)
#define NFC_RST				(1 << 2)
#define NFC_ECC_EN			(1 << 3)

#define NFC_RBA_SHIFT			4

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_1) ||	\
    defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)	/* mx51 */
#define NFC_RBA_WIDTH			3
#else
#define NFC_RBA_WIDTH			2
#endif
#define NFC_RBA_MASK			(((1 << NFC_RBA_WIDTH) - 1) << \
					 NFC_RBA_SHIFT)

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)	/* mx51 */
#define NFC_ITERATION_SHIFT		8
#define NFC_ITERATION_WIDTH		4
#define NFC_ACTIVE_CS_SHIFT		12
#define NFC_ACTIVE_CS_WIDTH		3
#define NFC_ACTIVE_CS_MASK		(((1 << NFC_ACTIVE_CS_WIDTH) - 1) << \
				NFC_ACTIVE_CS_SHIFT)

/* Bit definitions for NFC_CONFIGRATION3 */
#define NFC_NO_SDMA			(1 << 20)
#define NFC_FMP_SHIFT			16
#define NFC_FMP_WIDTH			4
#define NFC_FMP_MASK			(((1 << NFC_FMP_WIDTH) - 1) << NFC_FMP_SHIFT)
#define NFC_RBB_MODE			(1 << 15)
#define NFC_NUM_OF_DEVICES_SHIFT	12
#define NFC_NUM_OF_DEVICES_WIDTH	3
#define NFC_NUM_OF_DEVICES_MASK		(((1 << NFC_NUM_OF_DEVICES_WIDTH) - 1) << NFC_NUM_OF_DEVICES_SHIFT)
#define NFC_DMA_MODE_SHIFT		11
#define NFC_DMA_MODE_WIDTH		1
#define NFC_SBB_SHIFT			8
#define NFC_SBB_WIDTH			3
#define NFC_SBB_MASK			(((1 << NFC_SBB_WIDTH) - 1) << NFC_SBB_SHIFT)
#define NFC_BIG				(1 << 7)
#define NFC_SB2R_SHIFT			4
#define NFC_SB2R_WIDTH			3
#define NFC_FW_SHIFT			3
#define NFC_FW_WIDTH			1
#define NFC_FW_MASK			(((1 << NFC_FW_WIDTH) - 1) << NFC_FW_SHIFT)
#define NFC_TOO				(1 << 2)
#define NFC_ADD_OP_SHIFT		0
#define NFC_ADD_OP_WIDTH		2
#define NFC_ADD_OP_MASK			(((1 << NFC_ADD_OP_WIDTH) - 1) << NFC_ADD_OP_SHIFT)
#define NFC_FW_8			1
#define NFC_FW_16			0
#endif

#define NFC_PPB_SHIFT			7
#define NFC_PPB_MASK			(3 << NFC_PPB_SHIFT)
#define NFC_PPB_32			(0 << NFC_PPB_SHIFT)
#define NFC_PPB_64			(1 << NFC_PPB_SHIFT)
#define NFC_PPB_128			(2 << NFC_PPB_SHIFT)
#define NFC_PPB_256			(3 << NFC_PPB_SHIFT)

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define NFC_V3_2_BLS_SHIFT		6
#define NFC_BLS_MASK			(3 << NFC_V3_2_BLS_SHIFT)
#define NFC_BLS_LOCKED			(0 << NFC_V3_2_BLS_SHIFT)
#define NFC_BLS_LOCKED_DEFAULT		(1 << NFC_V3_2_BLS_SHIFT)
#define NFC_BLS_UNLOCKED		(2 << NFC_V3_2_BLS_SHIFT)
#else
#define NFC_BLS_SHIFT			16
#define NFC_BLS_MASK			(3 << NFC_BLS_SHIFT)
#define NFC_BLS_LOCKED			(0 << NFC_BLS_SHIFT)
#define NFC_BLS_LOCKED_DEFAULT		(1 << NFC_BLS_SHIFT)
#define NFC_BLS_UNLOCKED		(2 << NFC_BLS_SHIFT)
#endif

#define NFC_WPC_LOCK_TIGHT		1
#define NFC_WPC_LOCK			(1 << 1)
#define NFC_WPC_UNLOCK			(1 << 2)
#define NFC_WPC_MASK			0x7

#define REG_NFC_OPS_STAT		NFC_IPC
#define REG_NFC_INTRRUPT		NFC_CONFIG2
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2
#define REG_NFC_FLASH_ADDR		NFC_FLASH_ADDR0
#define REG_NFC_FLASH_CMD		NFC_FLASH_CMD
#else
#define REG_NFC_FLASH_ADDR		NFC_FLASH_ADDR_CMD
#define REG_NFC_FLASH_CMD		NFC_FLASH_ADDR_CMD
#endif
#define REG_NFC_OPS			LAUNCH_NFC
#define REG_NFC_SET_RBA			NFC_CONFIG1
#define REG_NFC_RB			NFC_IPC
#define REG_NFC_ECC_EN			NFC_CONFIG2
#define REG_NFC_ECC_STATUS_RESULT	NFC_ECC_STATUS_RESULT
#define REG_NFC_CE			NFC_CONFIG1
#define REG_NFC_RST			NFC_CONFIG1
#define REG_NFC_PPB			NFC_CONFIG2
#define REG_NFC_SP_EN			NFC_CONFIG1
#define REG_NFC_BLS			NFC_WRPROT
#define REG_UNLOCK_BLK_ADD0		NFC_WRPROT_UNLOCK_BLK_ADD0
#define REG_UNLOCK_BLK_ADD1		NFC_WRPROT_UNLOCK_BLK_ADD1
#define REG_UNLOCK_BLK_ADD2		NFC_WRPROT_UNLOCK_BLK_ADD2
#define REG_UNLOCK_BLK_ADD3		NFC_WRPROT_UNLOCK_BLK_ADD3
#define REG_NFC_WPC			NFC_WRPROT
#define REG_NFC_ONE_CYCLE		NFC_CONFIG2

/* NFC V3 Specific MACRO functions definitions */
/* Explcit ack ops status (if any), before issue of any command  */
#define ACK_OPS							\
	raw_write((raw_read(REG_NFC_OPS_STAT) & ~NFC_OPS_STAT), \
	REG_NFC_OPS_STAT)

/* Set RBA buffer id */
#define NFC_SET_RBA(val)				\
	raw_write((raw_read(REG_NFC_SET_RBA) &		\
			~NFC_RBA_MASK) |		\
	((val) << NFC_RBA_SHIFT), REG_NFC_SET_RBA)

#define NFC_SET_PS(val)						\
	raw_write((raw_read(NFC_CONFIG2) &			\
			~NFC_PS_MASK) |				\
	((val) << NFC_PS_SHIFT), NFC_CONFIG2)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_2
#define UNLOCK_ADDR(start_addr,end_addr)		\
{							\
	int i = 0;					\
	for (; i < NAND_MAX_CHIPS; i++)			\
	raw_write(start_addr |				\
	(end_addr << NFC_UNLOCK_END_ADDR_SHIFT),	\
	REG_UNLOCK_BLK_ADD0 + (i << 2));		\
}
#define NFC_SET_NFC_ACTIVE_CS(val)					\
	raw_write((raw_read(NFC_CONFIG1) &				\
			~NFC_ACTIVE_CS_MASK) |				\
	((val) << NFC_ACTIVE_CS_SHIFT), NFC_CONFIG1)

#define NFC_GET_MAXCHIP_SP()		8

#else
#define UNLOCK_ADDR(start_addr,end_addr)				\
	raw_write(start_addr |						\
	(end_addr << NFC_UNLOCK_END_ADDR_SHIFT), REG_UNLOCK_BLK_ADD0)

#define NFC_SET_NFC_ACTIVE_CS(val)
#define NFC_GET_MAXCHIP_SP()		1
#endif

#define NFC_SET_BLS(val) ((raw_read(REG_NFC_BLS) & NFC_BLS_MASK) | (val))
#define NFC_SET_WPC(val) ((raw_read(REG_NFC_WPC) & ~NFC_WPC_MASK) | (val))

#if defined(CONFIG_ARCH_MXC_HAS_NFC_V3_2)
#define NFC_SET_NFC_NUM_ADDR_PHASE1(val)				\
	raw_write((raw_read(NFC_CONFIG2) &				\
			~NFC_NUM_ADDR_PHASE1_MASK) |			\
		((val) << NFC_NUM_ADDR_PHASE1_SHIFT), NFC_CONFIG2)

#define NFC_SET_NFC_NUM_ADDR_PHASE0(val)			\
	raw_write((raw_read(NFC_CONFIG2) &			\
			~NFC_NUM_ADDR_PHASE0_MASK) |		\
	((val) << NFC_NUM_ADDR_PHASE0_SHIFT), NFC_CONFIG2)

#define NFC_SET_NFC_ITERATION(val)					\
	raw_write((raw_read(NFC_CONFIG1) &				\
			~NFC_ITERATION_MASK) |				\
		((val) << NFC_ITERATION_SHIFT), NFC_CONFIG1)

#define NFC_SET_FW(val)						\
	raw_write((raw_read(NFC_CONFIG3) &			\
			~NFC_FW_MASK) |				\
		((val) << NFC_FW_SHIFT), NFC_CONFIG3)

#define NFC_SET_NUM_OF_DEVICE(val)					\
	raw_write((raw_read(NFC_CONFIG3) &				\
			~NFC_NUM_OF_DEVICES_MASK) |			\
		  ((val) << NFC_NUM_OF_DEVICES_SHIFT), NFC_CONFIG3)

#define NFC_SET_ADD_OP_MODE(val)				\
	raw_write((raw_read(NFC_CONFIG3) &			\
			~NFC_ADD_OP_MASK) |			\
		((val) << NFC_ADD_OP_SHIFT), NFC_CONFIG3)

#define NFC_SET_ADD_CS_MODE(val) {				\
		NFC_SET_ADD_OP_MODE(val);			\
		NFC_SET_NUM_OF_DEVICE(chip->numchips - 1);	\
	}

#define NFC_SET_ST_CMD(val) \
	raw_write((raw_read(NFC_CONFIG2) & \
			~NFC_ST_CMD_MASK) |  \
	((val) << NFC_ST_CMD_SHIFT), NFC_CONFIG2)

#define NFMS_NF_DWIDTH 0
#define NFMS_NF_PG_SZ  1
#define NFC_CMD_1_SHIFT 8

#define NUM_OF_ADDR_CYCLE (fls(chip->pagemask) >> 3)
#define SET_NFC_DELAY_LINE(val) raw_write((val), NFC_DELAY_LINE)

/* should set the fw,ps,spas,ppb*/
static inline void NFC_SET_NFMS(struct mxc_nand_host *host, unsigned int v)
{
	struct mtd_info *mtd = &host->mtd;//container_of(host, struct mxc_nand_host, mtd);
	struct nand_chip *chip = mtd->priv;

	if (!v)
		NFC_SET_FW(NFC_FW_8);
	if ((v & (1 << NFMS_NF_DWIDTH)))
		NFC_SET_FW(NFC_FW_16);
	if ((v & (1 << NFMS_NF_PG_SZ))) {
		if (IS_2K_PAGE_NAND) {
			NFC_SET_PS(NFC_PS_2K);
			NFC_SET_NFC_NUM_ADDR_PHASE1(NUM_OF_ADDR_CYCLE);
			NFC_SET_NFC_NUM_ADDR_PHASE0(NFC_TWO_LESS_PHASE1);
		} else if (IS_4K_PAGE_NAND) {
			NFC_SET_PS(NFC_PS_4K);
			NFC_SET_NFC_NUM_ADDR_PHASE1(NUM_OF_ADDR_CYCLE);
			NFC_SET_NFC_NUM_ADDR_PHASE0(NFC_TWO_LESS_PHASE1);
		} else {
			NFC_SET_PS(NFC_PS_512);
			NFC_SET_NFC_NUM_ADDR_PHASE1(NUM_OF_ADDR_CYCLE - 1);
			NFC_SET_NFC_NUM_ADDR_PHASE0(NFC_ONE_LESS_PHASE1);
		}
		NFC_SET_ADD_CS_MODE(0);
		NFC_SET_SPAS(host, mtd->oobsize >> 1);
		NFC_SET_ECC_MODE(host, mtd->oobsize >> 1);
		NFC_SET_ST_CMD(0x70);
		raw_write(raw_read(NFC_CONFIG3) | (1 << 20), NFC_CONFIG3);
		SET_NFC_DELAY_LINE(0);
	}
}
#endif

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V3_1
#define NFC_SET_NFMS(v)
#endif

#define READ_PAGE()	send_read_page(host, 0)
#define PROG_PAGE()	send_prog_page(host, 0)

#elif defined(CONFIG_ARCH_MXC_HAS_NFC_V2)

/*
 * For V1/V2 NFC registers Definition
 */

/*
 * Addresses for NFC registers
 */
#define NFC_BUF_SIZE			(host->ip_base + 0x00)
#define NFC_BUF_ADDR			(host->ip_base + 0x04)
#define NFC_FLASH_ADDR			(host->ip_base + 0x06)
#define NFC_FLASH_CMD			(host->ip_base + 0x08)
#define NFC_CONFIG			(host->ip_base + 0x0A)
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define NFC_ECC_STATUS_RESULT		(host->ip_base + 0x0C)
#define NFC_ECC_STATUS_RESULT_1		(host->ip_base + 0x0C)
#define NFC_ECC_STATUS_RESULT_2		(host->ip_base + 0x0E)
#define NFC_SPAS			(host->ip_base + 0x10)
#else
#define NFC_ECC_STATUS_RESULT		(host->ip_base + 0x0C)
#define NFC_RSLTMAIN_AREA		(host->ip_base + 0x0E)
#define NFC_RSLTSPARE_AREA		(host->ip_base + 0x10)
#endif
#define NFC_WRPROT			(host->ip_base + 0x12)
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define NFC_UNLOCKSTART_BLKADDR		(host->ip_base + 0x20)
#define NFC_UNLOCKEND_BLKADDR		(host->ip_base + 0x22)
#define NFC_UNLOCKSTART_BLKADDR1	(host->ip_base + 0x24)
#define NFC_UNLOCKEND_BLKADDR1		(host->ip_base + 0x26)
#define NFC_UNLOCKSTART_BLKADDR2	(host->ip_base + 0x28)
#define NFC_UNLOCKEND_BLKADDR2		(host->ip_base + 0x2A)
#define NFC_UNLOCKSTART_BLKADDR3	(host->ip_base + 0x2C)
#define NFC_UNLOCKEND_BLKADDR3		(host->ip_base + 0x2E)
#else
#define NFC_UNLOCKSTART_BLKADDR		(host->ip_base + 0x14)
#define NFC_UNLOCKEND_BLKADDR		(host->ip_base + 0x16)
#endif
#define NFC_NF_WRPRST			(host->ip_base + 0x18)
#define NFC_CONFIG1			(host->ip_base + 0x1A)
#define NFC_CONFIG2			(host->ip_base + 0x1C)

/*!
 * Addresses for NFC RAM BUFFER Main area 0
 */
#define MAIN_AREA0			0x000
#define MAIN_AREA1			0x200

/*!
 * Addresses for NFC SPARE BUFFER Spare area 0
 */
#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define SPARE_AREA0			0x1000
#define SPARE_LEN			64
#define SPARE_COUNT			8
#else
#define SPARE_AREA0			0x800
#define SPARE_LEN			16
#define SPARE_COUNT			4
#endif
#define SPARE_SIZE			(SPARE_LEN * SPARE_COUNT)

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define REG_NFC_ECC_MODE	NFC_CONFIG1
#define SPAS_SHIFT		0
#define REG_NFC_SPAS		NFC_SPAS
#define SPAS_MASK		0xFF00
#define IS_4BIT_ECC						\
	((raw_read(REG_NFC_ECC_MODE) & NFC_ECC_MODE_4) >> 0)

#define NFC_SET_SPAS(v)							\
	raw_write(((raw_read(REG_NFC_SPAS) & SPAS_MASK) | (((v) << SPAS_SHIFT))),
								REG_NFC_SPAS)

#define NFC_SET_ECC_MODE(v) do					\
{								\
	u32 __ecc_mode = raw_read(REG_NFC_ECC_MODE);		\
	if ((v) == NFC_SPAS_218 || (v) == NFC_SPAS_112)  {	\
		raw_write(__ecc_mode & ~NFC_ECC_MODE_4,		\
			REG_NFC_ECC_MODE);			\
	} else {						\
		raw_write(__ecc_mode | NFC_ECC_MODE_4,		\
			REG_NFC_ECC_MODE);			\
	}							\
} while (0)

#define GET_ECC_STATUS()  __raw_readl(REG_NFC_ECC_STATUS_RESULT)
#define NFC_SET_NFMS(v)						\
do {								\
	NFMS |= (v);						\
	if (((v) & (1 << NFMS_NF_PG_SZ))) {			\
		NFC_SET_SPAS(mtd->oobsize >> 1);		\
		NFC_SET_ECC_MODE(mtd->oobsize >> 1);		\
	}							\
} while (0)
#else
#define IS_4BIT_ECC			1
#define NFC_SET_SPAS(v)
#define NFC_SET_ECC_MODE(v)
#define GET_ECC_STATUS()		raw_read(REG_NFC_ECC_STATUS_RESULT)
#define NFC_SET_NFMS(v)			(NFMS |= (v))
#endif

#define GET_NFC_ECC_STATUS() raw_read(REG_NFC_ECC_STATUS_RESULT)

/*!
 * Set INT to 0, Set 1 to specific operation bit, rest to 0 in LAUNCH_NFC Register for
 * Specific operation
 */
#define NFC_CMD				0x1
#define NFC_ADDR			0x2
#define NFC_INPUT			0x4
#define NFC_OUTPUT			0x8
#define NFC_ID				0x10
#define NFC_STATUS			0x20

/* Bit Definitions */
#define NFC_OPS_STAT			(1 << 15)
#define NFC_SP_EN			(1 << 2)
#define NFC_ECC_EN			(1 << 3)
#define NFC_INT_MSK			(1 << 4)
#define NFC_BIG				(1 << 5)
#define NFC_RST				(1 << 6)
#define NFC_CE				(1 << 7)
#define NFC_ONE_CYCLE			(1 << 8)
#define NFC_BLS_LOCKED			0
#define NFC_BLS_LOCKED_DEFAULT		1
#define NFC_BLS_UNLOCKED		2
#define NFC_WPC_LOCK_TIGHT		1
#define NFC_WPC_LOCK			(1 << 1)
#define NFC_WPC_UNLOCK			(1 << 2)
#define NFC_FLASH_ADDR_SHIFT		0
#define NFC_UNLOCK_END_ADDR_SHIFT	0

#ifdef CONFIG_ARCH_MXC_HAS_NFC_V2_1
#define NFC_ECC_MODE_4			 (1 << 0)
#define NFC_SPAS_16			 8
#define NFC_SPAS_64			 32
#define NFC_SPAS_112			 56
#define NFC_SPAS_128			 64
#define NFC_SPAS_218			 109
#endif
/* NFC Register Mapping */
#define REG_NFC_OPS_STAT		NFC_CONFIG2
#define REG_NFC_INTRRUPT		NFC_CONFIG1
#define REG_NFC_FLASH_ADDR		NFC_FLASH_ADDR
#define REG_NFC_FLASH_CMD		NFC_FLASH_CMD
#define REG_NFC_OPS			NFC_CONFIG2
#define REG_NFC_SET_RBA			NFC_BUF_ADDR
#define REG_NFC_ECC_EN			NFC_CONFIG1
#define REG_NFC_ECC_STATUS_RESULT	NFC_ECC_STATUS_RESULT
#define REG_NFC_CE			NFC_CONFIG1
#define REG_NFC_SP_EN			NFC_CONFIG1
#define REG_NFC_BLS			NFC_CONFIG
#define REG_NFC_WPC			NFC_WRPROT
#define REG_START_BLKADDR		NFC_UNLOCKSTART_BLKADDR
#define REG_END_BLKADDR			NFC_UNLOCKEND_BLKADDR
#define REG_NFC_RST			NFC_CONFIG1
#define REG_NFC_ONE_CYCLE		NFC_CONFIG1

/* NFC V1/V2 Specific MACRO functions definitions */

#define raw_write(v,a)			__raw_writew(v,a)
#define raw_read(a)			__raw_readw(a)

#define NFC_SET_BLS(val)		val

#define UNLOCK_ADDR(start_addr,end_addr)		\
{							\
	raw_write(start_addr,REG_START_BLKADDR);	\
	raw_write(end_addr,REG_END_BLKADDR);		\
}

#define NFC_SET_NFC_ACTIVE_CS(val)
#define NFC_GET_MAXCHIP_SP()		1
#define NFC_SET_WPC(val)		val

/* NULL Definitions */
#define ACK_OPS
#define NFC_SET_RBA(val) raw_write(val, REG_NFC_SET_RBA)

static inline READ_PAGE(void)
{
	if (nfc_is_v2_1()) {
		send_read_page(host, 0);
	} else {
		send_read_page(host, 0);
		send_read_page(host, 1);
		send_read_page(host, 2);
		send_read_page(host, 3);
	}
}

static inline PROG_PAGE(void)
{
	if (nfc_is_v2_1()) {
		send_prog_page(host, 0);
	} else {
		send_prog_page(host, 0);
		send_prog_page(host, 1);
		send_prog_page(host, 2);
		send_prog_page(host, 3);
	}
}

#endif

#endif	/* __MXC_ND2_H__ */
