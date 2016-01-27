/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Sascha Hauer, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <mach/hardware.h>
#include <mach/mxc_nand.h>

#ifdef CONFIG_MTD_DEBUG
static int debug;
module_param(debug, int, S_IRUGO | S_IWUSR);

#define dbg_lvl(n)	((n) < debug)
#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#undef DEBUG
#define DEBUG(l, fmt...) DBG(l, fmt)
#else
static int debug;
module_param(debug, int, 0);

#define dbg_lvl(n)	0
#define DBG(lvl, fmt...)	do { } while (0)
#endif


#define DRIVER_NAME "mxc_nand"

/* Addresses for NFC registers */
#define NFC_BUF_SIZE		0x000
#define NFC_BUF_ADDR		0x004
#define NFC_FLASH_ADDR		0x006
#define NFC_FLASH_CMD		0x008
#define NFC_CONFIG		0x00A
#define NFC_ECC_STATUS_RESULT	0x00C
#define NFC_WRPROT		0x012
#ifdef CONFIG_ARCH_MX2
#define NFC_RSLTMAIN_AREA	0x00E
#define NFC_RSLTSPARE_AREA	0x010
#define NFC_UNLOCKSTART_BLKADDR	0x014
#define NFC_UNLOCKEND_BLKADDR	0x016
#else
#define NFC_ECC_STATUS_RESULT2	0x00E
#define NFC_SPAS		0x010
#endif
#define NFC_NF_WRPRST		0x018
#define NFC_CONFIG1		0x01A
#define NFC_CONFIG2		0x01C
#ifndef CONFIG_ARCH_MX2
#define NFC_UNLOCKSTART_BLKADDR 0x020
#define NFC_UNLOCKEND_BLKADDR	0x022
#define NFC_UNLOCKSTART_BLKADDR1 0x024
#define NFC_UNLOCKEND_BLKADDR1	0x026
#define NFC_UNLOCKSTART_BLKADDR2 0x028
#define NFC_UNLOCKEND_BLKADDR2	0x02a
#define NFC_UNLOCKSTART_BLKADDR3 0x02c
#define NFC_UNLOCKEND_BLKADDR3	0x02e
#endif

/* Addresses for NFC RAM BUFFER Main area 0 */
#define MAIN_AREA0		0x000
#define MAIN_AREA1		0x200
#define MAIN_AREA2		0x400
#define MAIN_AREA3		0x600

/* Addresses for NFC SPARE BUFFER Spare area 0 */
#ifdef CONFIG_ARCH_MX2
#define SPARE_AREA_SIZE		16
#define SPARE_AREA0		0x800
#define SPARE_AREA1		0x810
#define SPARE_AREA2		0x820
#define SPARE_AREA3		0x830
#else
#define SPARE_AREA_SIZE		64
#define MAIN_AREA4		0x800
#define MAIN_AREA5		0xa00
#define MAIN_AREA6		0xc00
#define MAIN_AREA7		0xe00
#define SPARE_AREA0		0x1000
#define SPARE_AREA1		0x1040
#define SPARE_AREA2		0x1080
#define SPARE_AREA3		0x10c0
#define SPARE_AREA4		0x1100
#define SPARE_AREA5		0x1140
#define SPARE_AREA6		0x1180
#define SPARE_AREA7		0x11c0
#endif

/* Set INT to 0, FCMD to 1, rest to 0 in NFC_CONFIG2 Register
 * for Command operation */
#define NFC_CMD            0x1

/* Set INT to 0, FADD to 1, rest to 0 in NFC_CONFIG2 Register
 * for Address operation */
#define NFC_ADDR           0x2

/* Set INT to 0, FDI to 1, rest to 0 in NFC_CONFIG2 Register
 * for Input operation */
#define NFC_INPUT          0x4

/* Set INT to 0, FDO to 001, rest to 0 in NFC_CONFIG2 Register
 * for Data Output operation */
#define NFC_OUTPUT         0x8

/* Set INT to 0, FD0 to 010, rest to 0 in NFC_CONFIG2 Register
 * for Read ID operation */
#define NFC_ID             0x10

/* Set INT to 0, FDO to 100, rest to 0 in NFC_CONFIG2 Register
 * for Read Status operation */
#define NFC_STATUS         0x20

/* Set INT to 1, rest to 0 in NFC_CONFIG2 Register for Read
 * Status operation */
#define NFC_INT            0x8000

#define NFC_SP_EN           (1 << 2)
#define NFC_ECC_EN          (1 << 3)
#define NFC_INT_MSK         (1 << 4)
#define NFC_BIG             (1 << 5)
#define NFC_RST             (1 << 6)
#define NFC_CE              (1 << 7)
#define NFC_ONE_CYCLE       (1 << 8)

struct mxc_nand_host {
	struct mtd_info		mtd;
	struct nand_chip	nand;
	struct mtd_partition	*parts;
	struct device		*dev;

	void __iomem		*regs;
	void __iomem		*nfc_buf;
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

/* Define delays in microsec for NAND device operations */
#define TROP_US_DELAY   2000

/* OOB placement block for use with hardware ecc generation */
static struct nand_ecclayout mx2_nand_hw_eccoob2k_8 = {
	.eccbytes = 20,
	.eccpos = {
		6, 7, 8, 9, 10,
		22, 23, 24, 25, 26,
		38, 39, 40, 41, 42,
		54, 55, 56, 57, 58,
	},
	.oobfree = {{2, 4}, {11, 11}, {27, 11}, {43, 11}, {59, 5}},
};

static struct nand_ecclayout mx2_nand_hw_eccoob_8 = {
	.eccbytes = 5,
	.eccpos = { 6, 7, 8, 9, 10 },
	.oobfree = {{0, 6}, {11, 5}}
};
#ifdef CONFIG_MTD_NAND_MXC_FLASH_BBT
static u8 bbt_pattern[] = {'B', 'b', 't', '0' };
static u8 mirror_pattern[] = {'1', 't', 'b', 'B' };

/*
 * i.MX27 flash bbt decriptors
 */
static struct nand_bbt_descr mx2_bbt_main_descr = {
	.options = (NAND_BBT_LASTBLOCK | NAND_BBT_WRITE |
		    NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP),
	.offs =	12,
	.len = 4,
	.veroffs = 11,
	.maxblocks = 4,
	.pattern = bbt_pattern,
};

static struct nand_bbt_descr mx2_bbt_mirror_descr = {
	.options = (NAND_BBT_LASTBLOCK | NAND_BBT_WRITE |
		    NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP),
	.offs =	12,
	.len = 4,
	.veroffs = 11,
	.maxblocks = 4,
	.pattern = mirror_pattern,
};

/*
 * i.MX25 flash bbt decriptors
 */
static struct nand_bbt_descr mx25_bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_WRITE |
	NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 0,
	.len = 4,
	.veroffs = 4,
	.maxblocks = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr mx25_bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_WRITE |
	NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 0,
	.len = 4,
	.veroffs = 4,
	.maxblocks = 4,
	.pattern = mirror_pattern
};
#endif

/*
 * OOB placement block for use with hardware ecc generation
 */
static struct nand_ecclayout mx25_nand_hw_eccoob2k_8 = {
	.eccbytes = 36,
	.eccpos = {
		7, 8, 9, 10, 11, 12, 13, 14, 15,
		23, 24, 25, 26, 27, 28, 29, 30, 31,
		39, 40, 41, 42, 43, 44, 45, 46, 47,
		55, 56, 57, 58, 59, 60, 61, 62, 63,
	},
	.oobfree = {{2, 5}, {16, 7}, {32, 7}, {48, 7}},
};

static struct nand_ecclayout mx25_nand_hw_eccoob_8 = {
	.eccbytes = 9,
	.eccpos = { 7, 8, 9, 10, 11, 12, 13, 14, 15, },
	.oobfree = {{0, 4}},
};

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "RedBoot", "cmdlinepart", NULL };
#endif

#ifdef CONFIG_MTD_DEBUG
#define nfc_read_reg(b, r)	__nfc_read_reg(b, r, #r, __FUNCTION__)
static inline u16 __nfc_read_reg(void __iomem *base, unsigned int reg,
				 const char *name, const char *fn)
{
	u16 val = readw(base + reg);
	DBG(3, "%s: Read %04x from %s[%02x]\n", fn, val, name, reg);
	return val;
}

#define nfc_write_reg(v, b, r)	__nfc_write_reg(v, b, r, #r, __FUNCTION__)
static inline void __nfc_write_reg(u16 val, void __iomem *base, unsigned int reg,
				   const char *name, const char *fn)
{
	DBG(3, "%s: Writing %04x to %s[%02x]\n", fn, val, name, reg);
	writew(val, base + reg);
}
#else
#define nfc_read_reg(b, r)	readw(b + r)
#define nfc_write_reg(v, b, r)	writew(v, b + r)
#endif

static irqreturn_t mxc_nfc_irq(int irq, void *dev_id)
{
	struct mxc_nand_host *host = dev_id;
	uint16_t tmp;

	DEBUG(MTD_DEBUG_LEVEL3, "%s(%d)\n", __FUNCTION__, irq);

	tmp = nfc_read_reg(host->regs, NFC_CONFIG1);
	tmp |= NFC_INT_MSK; /* Disable interrupt */
	nfc_write_reg(tmp, host->regs, NFC_CONFIG1);

	wake_up(&host->irq_waitq);

	return IRQ_HANDLED;
}

/* This function polls the NANDFC to wait for the basic operation to
 * complete by checking the INT bit of config2 register.
 */
static void wait_op_done(struct mxc_nand_host *host, int max_retries,
				uint16_t param, int useirq)
{
	if (useirq) {
		if (!(nfc_read_reg(host->regs, NFC_CONFIG2) & NFC_INT)) {
			uint32_t cfg1;
			const unsigned long timeout = max_retries;

			cfg1 = nfc_read_reg(host->regs, NFC_CONFIG1);
			cfg1 &= ~NFC_INT_MSK;	/* Enable interrupt */
			nfc_write_reg(cfg1, host->regs, NFC_CONFIG1);

			max_retries = wait_event_timeout(host->irq_waitq,
				nfc_read_reg(host->regs, NFC_CONFIG2) &
					NFC_INT, timeout);
		}
	} else {
		while (!(nfc_read_reg(host->regs, NFC_CONFIG2) & NFC_INT) &&
			max_retries-- > 0) {
			udelay(1);
		}
	}
	WARN_ON(nfc_read_reg(host->regs, NFC_CONFIG2) & ~NFC_INT);
	nfc_write_reg(0, host->regs, NFC_CONFIG2);
	if (WARN_ON(max_retries <= 0)) {
		printk(KERN_ERR "%s(%d): INT not set\n", __func__, param);
	}
}

/* This function issues the specified command to the NAND device and
 * waits for completion. */
static void send_cmd(struct mxc_nand_host *host, uint16_t cmd, int useirq)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_cmd(host, 0x%x, %d)\n", cmd, useirq);

	nfc_write_reg(cmd, host->regs, NFC_FLASH_CMD);
	WARN_ON(nfc_read_reg(host->regs, NFC_CONFIG2));
	nfc_write_reg(NFC_CMD, host->regs, NFC_CONFIG2);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, cmd, useirq);
}

/* This function sends an address (or partial address) to the
 * NAND device. The address is used to select the source/destination for
 * a NAND command. */
static void send_addr(struct mxc_nand_host *host, uint16_t addr, int islast)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_addr(host, 0x%x %d)\n", addr, islast);

	nfc_write_reg(addr, host->regs, NFC_FLASH_ADDR);
	WARN_ON(nfc_read_reg(host->regs, NFC_CONFIG2));
	nfc_write_reg(NFC_ADDR, host->regs, NFC_CONFIG2);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, addr, islast);
}

static inline void nfc_buf_read(const void __iomem *nfc, void *buf, int len)
{
	u32 *wp = buf;
	int i;

	BUG_ON((unsigned long)nfc & 3);
	BUG_ON((unsigned long)buf & 3);

	for (i = 0; i < len; i += sizeof(long)) {
		wp[i >> 2] = readl(nfc + i);
	}
}

static inline void nfc_buf_write(void __iomem *nfc, const void *buf, int len)
{
	const u32 *rp = buf;
	int i;

	BUG_ON((unsigned long)nfc & 3);
	BUG_ON((unsigned long)buf & 3);

	for (i = 0; i < len; i += sizeof(long)) {
		writel(rp[i >> 2], nfc + i);
	}
}

/* This function requests the NANDFC to initate the transfer
 * of data currently in the NANDFC RAM buffer to the NAND device. */
static void send_prog_page(struct mxc_nand_host *host, uint8_t buf_id,
			int spare_only)
{
	int i;

	if (spare_only)
		DEBUG(MTD_DEBUG_LEVEL1, "send_prog_page (%d)\n", spare_only);
	if (cpu_is_mx25()) {
		for (i = 0; i < 4; i++) {
			void __iomem *src = host->nfc_buf + SPARE_AREA0 + i * 16;
			void __iomem *dst = host->nfc_buf + SPARE_AREA0 + i * 64;

			memcpy(dst, src, 16);
		}
	}

	/* NANDFC buffer 0 is used for page read/write */
	nfc_write_reg(buf_id, host->regs, NFC_BUF_ADDR);

	/* Configure spare or page+spare access */
	if (!host->pagesize_2k) {
		uint16_t config1 = nfc_read_reg(host->regs, NFC_CONFIG1);
		if (spare_only)
			config1 |= NFC_SP_EN;
		else
			config1 &= ~NFC_SP_EN;
		nfc_write_reg(config1, host->regs, NFC_CONFIG1);
	}
	WARN_ON(nfc_read_reg(host->regs, NFC_CONFIG2));
	nfc_write_reg(NFC_INPUT, host->regs, NFC_CONFIG2);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, spare_only, true);
}

/* Requests NANDFC to initated the transfer of data from the
 * NAND device into in the NANDFC ram buffer. */
static void send_read_page(struct mxc_nand_host *host, uint8_t buf_id,
		int spare_only)
{
	int i;

	DEBUG(MTD_DEBUG_LEVEL3, "send_read_page (%d)\n", spare_only);

	/* NANDFC buffer 0 is used for page read/write */
	nfc_write_reg(buf_id, host->regs, NFC_BUF_ADDR);

	/* Configure spare or page+spare access */
	if (!host->pagesize_2k) {
		uint32_t config1 = nfc_read_reg(host->regs, NFC_CONFIG1);
		if (spare_only)
			config1 |= NFC_SP_EN;
		else
			config1 &= ~NFC_SP_EN;
		nfc_write_reg(config1, host->regs, NFC_CONFIG1);
	}

	WARN_ON(nfc_read_reg(host->regs, NFC_CONFIG2));
	nfc_write_reg(NFC_OUTPUT, host->regs, NFC_CONFIG2);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, spare_only, true);
	if (!cpu_is_mx25())
		return;

	for (i = 0; i < 4; i++) {
		void __iomem *src = host->nfc_buf + SPARE_AREA0 + i * 64;
		void __iomem *dst = host->nfc_buf + SPARE_AREA0 + i * 16;

		memcpy(dst, src, 16);
	}
}

/* Request the NANDFC to perform a read of the NAND device ID. */
static void send_read_id(struct mxc_nand_host *host)
{
	struct nand_chip *this = &host->nand;
	uint16_t tmp;

	/* NANDFC buffer 0 is used for device ID output */
	nfc_write_reg(0x0, host->regs, NFC_BUF_ADDR);

	tmp = nfc_read_reg(host->regs, NFC_CONFIG1);
	tmp &= ~NFC_SP_EN;
	nfc_write_reg(tmp, host->regs, NFC_CONFIG1);

	WARN_ON(nfc_read_reg(host->regs, NFC_CONFIG2));
	/* Read ID into main buffer */
	nfc_write_reg(NFC_ID, host->regs, NFC_CONFIG2);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, 0, true);

	if (this->options & NAND_BUSWIDTH_16) {
		/* FIXME: This cannot work, because the NFC buffer
		 * cannot be accessed with byte accesses! */
		void __iomem *main_buf = host->nfc_buf + MAIN_AREA0;
		/* compress the ID info */
		writeb(readb(main_buf + 2), main_buf + 1);
		writeb(readb(main_buf + 4), main_buf + 2);
		writeb(readb(main_buf + 6), main_buf + 3);
		writeb(readb(main_buf + 8), main_buf + 4);
		writeb(readb(main_buf + 10), main_buf + 5);
	}
}

/* This function requests the NANDFC to perform a read of the
 * NAND device status and returns the current status. */
static uint16_t get_dev_status(struct mxc_nand_host *host)
{
	void __iomem *main_buf = host->nfc_buf + MAIN_AREA1;
	uint32_t store;
	uint16_t ret, tmp;

	/* store the main area first word, later do recovery */
	store = readl(main_buf);
	/* NANDFC buffer 1 is used for device status to prevent
	 * corruption of read/write buffer on status requests. */
	nfc_write_reg(1, host->regs, NFC_BUF_ADDR);

	/* Read status into main buffer */
	tmp = nfc_read_reg(host->regs, NFC_CONFIG1);
	tmp &= ~NFC_SP_EN;
	nfc_write_reg(tmp, host->regs, NFC_CONFIG1);

	/* Issue status request to NAND device */
	WARN_ON(nfc_read_reg(host->regs, NFC_CONFIG2));
	nfc_write_reg(NFC_STATUS, host->regs, NFC_CONFIG2);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, 0, true);

	/* Status is placed in first word of main buffer */
	/* get status, then recover area 1 data */
	ret = readw(main_buf);
	writel(store, main_buf);

	DEBUG(MTD_DEBUG_LEVEL2, "%s: status=%02x\n", __FUNCTION__, ret);

	return ret;
}

/* This functions is used by upper layer to checks if device is ready */
static int mxc_nand_dev_ready(struct mtd_info *mtd)
{
	/*
	 * NFC handles R/B internally. Therefore, this function
	 * always returns status as ready.
	 */
	return 1;
}

static void mxc_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	switch (mode) {
	case NAND_ECC_WRITE:
		DBG(0, "ECC_MODE=NAND_ECC_WRITE\n");
		break;
	case NAND_ECC_READSYN:
		DBG(0, "ECC_MODE=NAND_ECC_READSYN\n");
		break;
	case NAND_ECC_READ:
		DBG(0, "ECC_MODE=NAND_ECC_READ\n");
		break;
	default:
		DBG(-1, "%s: Unknown ECC_MODE: %d\n", __FUNCTION__, mode);
	}
}

static void _mxc_nand_enable_hwecc(struct mtd_info *mtd, int on)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	uint16_t ecc = nfc_read_reg(host->regs, NFC_CONFIG1);

	if (on) {
		ecc |= NFC_ECC_EN;
	} else {
		ecc &= ~NFC_ECC_EN;
	}
	nfc_write_reg(ecc, host->regs, NFC_CONFIG1);
}

static int mxc_nand_read_oob_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				      int page, int sndcmd)
{
	struct mxc_nand_host *host = chip->priv;
	uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;
	int eccpitch = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
	uint8_t *bufpoi = buf;
	int i, toread;

	DEBUG(MTD_DEBUG_LEVEL0, "%s: Reading OOB area of page %u to oob %p\n",
	      __FUNCTION__, host->page_addr, buf);

	chip->cmdfunc(mtd, NAND_CMD_READOOB, mtd->writesize, page);
	for (i = 0; i < chip->ecc.steps; i++) {
		toread = min_t(int, length, chip->ecc.prepad);
		if (toread) {
			chip->read_buf(mtd, bufpoi, toread);
			bufpoi += toread;
			length -= toread;
		}
		bufpoi += chip->ecc.bytes;
		host->col_addr += chip->ecc.bytes;
		length -= chip->ecc.bytes;

		toread = min_t(int, length, chip->ecc.postpad);
		if (toread) {
			chip->read_buf(mtd, bufpoi, toread);
			bufpoi += toread;
			length -= toread;
		}
	}
	if (length > 0)
		chip->read_buf(mtd, bufpoi, length);

	_mxc_nand_enable_hwecc(mtd, 0);
	chip->cmdfunc(mtd, NAND_CMD_READOOB, mtd->writesize + chip->ecc.prepad, page);
	bufpoi = buf + chip->ecc.prepad;
	length = mtd->oobsize - chip->ecc.prepad;
	for (i = 0; i < chip->ecc.steps; i++) {
		toread = min_t(int, length, chip->ecc.bytes);
		chip->read_buf(mtd, bufpoi, toread);
		bufpoi += eccpitch;
		length -= eccpitch;
		host->col_addr += chip->ecc.postpad + chip->ecc.prepad;
	}
	_mxc_nand_enable_hwecc(mtd, 1);
	return 1;
}

static int mxc_nand_read_page_raw_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
					   uint8_t *buf)
{
	struct mxc_nand_host *host = chip->priv;
	int eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccpitch = eccbytes + chip->ecc.prepad + chip->ecc.postpad;
	uint8_t *oob = chip->oob_poi;
	int steps, size;
	int n;

	_mxc_nand_enable_hwecc(mtd, 0);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, host->page_addr);

	for (n = 0, steps = chip->ecc.steps; steps > 0; n++, steps--) {
		/* set col_addr to n'th slice of data area */
		host->col_addr = n * eccsize;
		chip->read_buf(mtd, buf, eccsize);
		buf += eccsize;

		/* set col_addr to n'th slice of OOB area */
		host->col_addr = mtd->writesize + n * eccpitch;
		if (chip->ecc.prepad) {
			chip->read_buf(mtd, oob, chip->ecc.prepad);
			oob += chip->ecc.prepad;
		}

		/* read ECC syndrome */
		chip->read_buf(mtd, oob, eccbytes);
		oob += eccbytes;

		if (chip->ecc.postpad) {
			chip->read_buf(mtd, oob, chip->ecc.postpad);
			oob += chip->ecc.postpad;
		}
	}

	size = mtd->oobsize - (oob - chip->oob_poi);
	if (size)
		chip->read_buf(mtd, oob, size);
	_mxc_nand_enable_hwecc(mtd, 1);

	return 0;
}

static int mxc_nand_read_page_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				       uint8_t *buf)
{
	struct mxc_nand_host *host = chip->priv;
	int n, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccpitch = eccbytes + chip->ecc.prepad + chip->ecc.postpad;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;

	DEBUG(MTD_DEBUG_LEVEL1, "%s: Reading page %u to buf %p oob %p\n", __FUNCTION__,
	      host->page_addr, buf, oob);

	/* first read out the data area and the available portion of OOB */
	for (n = 0; eccsteps; n++, eccsteps--, p += eccsize) {
		int stat;

		host->col_addr = n * eccsize;
		chip->read_buf(mtd, p, eccsize);

		host->col_addr = mtd->writesize + n * eccpitch;
		if (chip->ecc.prepad) {
			chip->read_buf(mtd, oob, chip->ecc.prepad);
			oob += chip->ecc.prepad;
		}

		stat = chip->ecc.correct(mtd, p, oob, NULL);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
		oob += eccbytes;

		if (chip->ecc.postpad) {
			chip->read_buf(mtd, oob, chip->ecc.postpad);
			oob += chip->ecc.postpad;
		}
	}

	/* Calculate remaining oob bytes */
	n = mtd->oobsize - (oob - chip->oob_poi);
	if (n)
		chip->read_buf(mtd, oob, n);

	/* Then switch ECC off and read the OOB area to get the ECC code */
	_mxc_nand_enable_hwecc(mtd, 0);
	chip->cmdfunc(mtd, NAND_CMD_READOOB, mtd->writesize, host->page_addr);
	eccsteps = chip->ecc.steps;
	oob = chip->oob_poi + chip->ecc.prepad;
	for (n = 0; eccsteps; n++, eccsteps--, p += eccsize) {
		host->col_addr = mtd->writesize + n * eccpitch + chip->ecc.prepad;
		chip->read_buf(mtd, oob, eccbytes);
		oob += eccbytes + chip->ecc.postpad;
	}
	_mxc_nand_enable_hwecc(mtd, 1);
	return 0;
}

static int mxc_nand_write_oob_syndrome(struct mtd_info *mtd,
				       struct nand_chip *chip, int page)
{
	struct mxc_nand_host *host = chip->priv;
	int eccpitch = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
	int length = mtd->oobsize;
	int i, len, status, steps = chip->ecc.steps;
	const uint8_t *bufpoi = chip->oob_poi;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	for (i = 0; i < steps; i++) {
		len = min_t(int, length, eccpitch);

		chip->write_buf(mtd, bufpoi, len);
		bufpoi += len;
		length -= len;
		host->col_addr += chip->ecc.prepad + chip->ecc.postpad;
	}
	if (length > 0)
		chip->write_buf(mtd, bufpoi, length);

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);
	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static void mxc_nand_write_page_raw_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
					     const uint8_t *buf)
{
	struct mxc_nand_host *host = chip->priv;
	int eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccpitch = eccbytes + chip->ecc.prepad + chip->ecc.postpad;
	uint8_t *oob = chip->oob_poi;
	int steps, size;
	int n;

	for (n = 0, steps = chip->ecc.steps; steps > 0; n++, steps--) {
		host->col_addr = n * eccsize;
		chip->write_buf(mtd, buf, eccsize);
		buf += eccsize;

		host->col_addr = mtd->writesize + n * eccpitch;

		if (chip->ecc.prepad) {
			chip->write_buf(mtd, oob, chip->ecc.prepad);
			oob += chip->ecc.prepad;
		}

		host->col_addr += eccbytes;
		oob += eccbytes;

		if (chip->ecc.postpad) {
			chip->write_buf(mtd, oob, chip->ecc.postpad);
			oob += chip->ecc.postpad;
		}
	}

	size = mtd->oobsize - (oob - chip->oob_poi);
	if (size)
		chip->write_buf(mtd, oob, size);
}

static void mxc_nand_write_page_syndrome(struct mtd_info *mtd,
					 struct nand_chip *chip, const uint8_t *buf)
{
	struct mxc_nand_host *host = chip->priv;
	int i, n, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccpitch = eccbytes + chip->ecc.prepad + chip->ecc.postpad;
	int eccsteps = chip->ecc.steps;
	const uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;

	chip->ecc.hwctl(mtd, NAND_ECC_WRITE);

	for (i = n = 0; eccsteps; n++, eccsteps--, i += eccbytes, p += eccsize) {
		host->col_addr = n * eccsize;

		chip->write_buf(mtd, p, eccsize);

		host->col_addr = mtd->writesize + n * eccpitch;

		if (chip->ecc.prepad) {
			chip->write_buf(mtd, oob, chip->ecc.prepad);
			oob += chip->ecc.prepad;
		}

		chip->write_buf(mtd, oob, eccbytes);
		oob += eccbytes;

		if (chip->ecc.postpad) {
			chip->write_buf(mtd, oob, chip->ecc.postpad);
			oob += chip->ecc.postpad;
		}
	}

	/* Calculate remaining oob bytes */
	i = mtd->oobsize - (oob - chip->oob_poi);
	if (i)
		chip->write_buf(mtd, oob, i);
}

static int mxc_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	uint16_t ecc_status = nfc_read_reg(host->regs, NFC_ECC_STATUS_RESULT);

	/*
	 * 1-Bit errors are automatically corrected in HW.  No need for
	 * additional correction.  2-Bit errors cannot be corrected by
	 * HW ECC, so we need to return failure
	 */
	if (!(nfc_read_reg(host->regs, NFC_CONFIG1) & NFC_ECC_EN)) {
		DEBUG(MTD_DEBUG_LEVEL1, "%s: ECC turned off\n", __FUNCTION__);
		return 0;
	}

	if (ecc_status)
		DBG(ecc_status ? 0 : 1, "%s: ECC_STATUS=%04x\n", __FUNCTION__, ecc_status);

	if (cpu_is_mx25()) {
		int subpages = mtd->writesize / nand_chip->subpagesize;
		int pg2blk_shift = nand_chip->phys_erase_shift - nand_chip->page_shift;

		do {
			if ((ecc_status & 0xf) > 4) {
				static int last_bad = -1;

				if (last_bad != host->page_addr >> pg2blk_shift) {
					last_bad = host->page_addr >> pg2blk_shift;
					printk(KERN_DEBUG
					       "MXC_NAND: HWECC uncorrectable ECC error in block %u page %u subpage %d\n",
					       last_bad, host->page_addr,
					       mtd->writesize / nand_chip->subpagesize - subpages);
				}
				return -1;
			}
			ecc_status >>= 4;
			subpages--;
		} while (subpages > 0);
	} else {
		if (((ecc_status & 0x3) == 2) || ((ecc_status >> 2) == 2)) {
			DEBUG(MTD_DEBUG_LEVEL0,
			      "MXC_NAND: HWECC uncorrectable 2-bit ECC error\n");
			return -1;
		}
	}

	return 0;
}

static int mxc_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				  u_char *ecc_code)
{
	/* HW ECC calculation is done transparently by the controller */
	return 0;
}

static u_char mxc_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	uint8_t ret = 0;
	uint16_t col, rd_word;
	uint16_t __iomem *main_buf = host->nfc_buf + MAIN_AREA0;
	uint16_t __iomem *spare_buf = host->nfc_buf + SPARE_AREA0;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s(col = %d)\n", __FUNCTION__, host->col_addr);

	BUG_ON(host->spare_only && host->col_addr >= 16);

	/* Check for status request */
	if (host->status_request)
		return get_dev_status(host) & 0xFF;

	/* Get column for 16-bit access */
	col = host->col_addr >> 1;

	/* If we are accessing the spare region */
	if (host->spare_only)
		rd_word = readw(&spare_buf[col]);
	else
		rd_word = readw(&main_buf[col]);

	/* Pick upper/lower byte of word from RAM buffer */
	if (host->col_addr & 0x1)
		ret = (rd_word >> 8) & 0xFF;
	else
		ret = rd_word & 0xFF;

	/* Update saved column address */
	host->col_addr++;

	return ret;
}

static uint16_t mxc_nand_read_word(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	uint16_t col, ret;
	uint16_t __iomem *p;

	DEBUG(MTD_DEBUG_LEVEL1,
	      "%s(col = %d)\n", __FUNCTION__, host->col_addr);

	col = host->col_addr;

	/* Adjust saved column address */
	if (col < mtd->writesize && host->spare_only)
		col += mtd->writesize;
	BUG_ON(col >= mtd->writesize + 16);

	if (col < mtd->writesize)
		p = (host->nfc_buf + MAIN_AREA0) + (col >> 1);
	else
		p = (host->nfc_buf + SPARE_AREA0) + ((col - mtd->writesize) >> 1);

	if (col & 1) {
		ret = readw(p++) >> 8;
		ret |= readw(p) << 8;
	} else
		ret = readw(p);

	/* Update saved column address */
	host->col_addr = col + 2;

	return ret;
}

/* Write data of length len to buffer buf. The data to be
 * written on NAND Flash is first copied to RAMbuffer. After the Data Input
 * Operation by the NFC, the data is written to NAND Flash */
static void mxc_nand_write_buf(struct mtd_info *mtd,
				const u_char *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	int n, col, i = 0;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s(buf=%p col=%03x, len=%03x)\n", __FUNCTION__,
	      buf, host->col_addr, len);
	col = host->col_addr;

	/* Adjust saved column address */
	if (col < mtd->writesize && host->spare_only)
		col += mtd->writesize;

	/* If more data is requested to be written than free space in
	 * the flash buffer this is clearly a BUG! */
	BUG_ON(len > mtd->writesize + mtd->oobsize - col);
	n = len;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s:%d: col=%03x, n=%03x\n", __func__, __LINE__, col, n);

	while (n) {
		void __iomem *p;

		if (col < mtd->writesize)
			p = host->nfc_buf + MAIN_AREA0 + (col & ~3);
		else
			p = host->nfc_buf + SPARE_AREA0 +
				(col & ~3) - mtd->writesize;

		DEBUG(MTD_DEBUG_LEVEL3, "%s:%d: p=%p\n", __func__,
		      __LINE__, p);

		if (((col | (int)&buf[i]) & 3) || n < 16) {
			uint32_t data = 0;

			if (col & 3 || n < 4)
				data = readl(p);

			switch (col & 3) {
			case 0:
				if (n) {
					data = (data & 0xffffff00) |
					    (buf[i++] << 0);
					n--;
					col++;
				}
			case 1:
				if (n) {
					data = (data & 0xffff00ff) |
					    (buf[i++] << 8);
					n--;
					col++;
				}
			case 2:
				if (n) {
					data = (data & 0xff00ffff) |
					    (buf[i++] << 16);
					n--;
					col++;
				}
			case 3:
				if (n) {
					data = (data & 0x00ffffff) |
					    (buf[i++] << 24);
					n--;
					col++;
				}
			}

			writel(data, p);
		} else {
			int m = mtd->writesize - col;

			if (col >= mtd->writesize)
				m += mtd->oobsize;

			m = min(n, m) & ~3;
			memcpy(p, &buf[i], m);
			col += m;
			i += m;
			n -= m;
		}
	}
	/* Update saved column address */
	host->col_addr = col;
}

/* Read the data buffer from the NAND Flash. To read the data from NAND
 * Flash first the data output cycle is initiated by the NFC, which copies
 * the data to RAMbuffer. This data of length len is then copied to buffer buf.
 */
static void mxc_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	int n, col, i = 0;

	col = host->col_addr;

	DEBUG(MTD_DEBUG_LEVEL1,
	      "%s(col=%03x len=%03x)\n", __FUNCTION__, col, len);

	/* Adjust saved column address */
	if (col < mtd->writesize && host->spare_only)
		col += mtd->writesize;

	/* If more data is requested to be read than is available in
	 * the flash buffer this is clearly a BUG! */
	BUG_ON(len > mtd->writesize + mtd->oobsize - col);
	n = len;

	while (n) {
		const void __iomem *p;

		if (col < mtd->writesize)
			p = host->nfc_buf + MAIN_AREA0 + (col & ~3);
		else
			p = host->nfc_buf + SPARE_AREA0 +
				(col & ~3) - mtd->writesize;

		if (((col | (int)&buf[i]) & 3) || n < 16) {
			uint32_t data;

			data = readl(p);
			switch (col & 3) {
			case 0:
				if (n) {
					buf[i++] = data;
					n--;
					col++;
				}
			case 1:
				if (n) {
					buf[i++] = data >> 8;
					n--;
					col++;
				}
			case 2:
				if (n) {
					buf[i++] = data >> 16;
					n--;
					col++;
				}
			case 3:
				if (n) {
					buf[i++] = data >> 24;
					n--;
					col++;
				}
			}
		} else {
			int m = mtd->writesize - col;

			if (col >= mtd->writesize)
				m += mtd->oobsize;

			m = min(n, m) & ~3;
			memcpy(&buf[i], p, m);
			col += m;
			i += m;
			n -= m;
		}
	}
	/* Update saved column address */
	host->col_addr = col;
}

/* Used by the upper layer to verify the data in NAND Flash
 * with the data in the buf. */
static int mxc_nand_verify_buf(struct mtd_info *mtd,
				const u_char *buf, int len)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	int i;
	u16 *wp = host->nfc_buf + MAIN_AREA0;
	const u_char *b = buf;

	for (i = 0; i < len >> 1; i++) {
		u16 w = *wp++;
		u16 data = *buf++;

		if (len - i > 1)
			data |= (*buf++) << 8;
		else
			data |= 0xff << 8;

		/* This is crappy! The upper layer always sends us a
		 * whole page buffer to verify, even if only a partial page
		 * was written. So, ignore all 0xff bytes in the reference buffer.
		 */
		if ((data & w) != w) {
			printk(KERN_ERR
			       "%s: verify error @ %03x: read: %02x %02x expected: %02x %02x\n",
			       __FUNCTION__, i, w & 0xff, w >> 8, data & 0xff, data >> 8);
			print_hex_dump(KERN_DEBUG, "ver: ", DUMP_PREFIX_ADDRESS,
				       16, 2, b + ((i << 1) & ~0xf), min(len - i, 64), 0);
			print_hex_dump(KERN_DEBUG, "ref: ", DUMP_PREFIX_ADDRESS,
				       16, 2, host->nfc_buf + MAIN_AREA0 + ((i << 1) & ~0xf),
				       min(len - i, 64), 0);
			return -EFAULT;
		}
	}
	return 0;
}

/* This function is used by upper layer for select and
 * deselect of the NAND chip */
static void mxc_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;

#ifdef CONFIG_MTD_NAND_MXC_FORCE_CE
	if (chip > 0) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "ERROR:  Illegal chip select (chip = %d)\n", chip);
		return;
	}

	if (chip == -1) {
		nfc_write_reg(nfc_read_reg(host->regs, NFC_CONFIG1) & ~NFC_CE,
			      host->regs, NFC_CONFIG1);
		return;
	}

	nfc_write_reg(nfc_read_reg(host->regs, NFC_CONFIG1) | NFC_CE,
		      host->regs, NFC_CONFIG1);
#endif

	switch (chip) {
	case -1:
		/* Disable the NFC clock */
		if (host->clk_act) {
			clk_disable(host->clk);
			host->clk_act = 0;
		}
		break;
	case 0:
		/* Enable the NFC clock */
		if (!host->clk_act) {
			clk_enable(host->clk);
			host->clk_act = 1;
		}
		break;
	}
}

/* Used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash */
static void mxc_nand_command(struct mtd_info *mtd, unsigned command,
				int column, int page_addr)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	int useirq = false;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s: cmd 0x%08x, col 0x%03x, page 0x%04x\n", __FUNCTION__,
	      command, column, page_addr);

	/* Reset command state information */
	host->status_request = false;

	/* Command pre-processing step */
	switch (command) {

	case NAND_CMD_STATUS:
		host->col_addr = 0;
		host->status_request = true;
		break;

	case NAND_CMD_READ0:
		host->page_addr = page_addr;
		host->col_addr = column;
		host->spare_only = false;
		break;

	case NAND_CMD_READOOB:
		host->col_addr = column;
		host->spare_only = true;
		if (host->pagesize_2k)
			command = NAND_CMD_READ0; /* only READ0 is valid */
		break;

	case NAND_CMD_SEQIN:
		if (column >= mtd->writesize) {
			/*
			 * Before sending the SEQIN command for writing OOB
			 * we must read one page out.
			 * Because K9F1GXX has no READ1 command to set current HW
			 * pointer to spare area, we must write the whole page
			 * including OOB together.
			 */
			if (host->pagesize_2k)
				/* call ourself to read a page */
				mxc_nand_command(mtd, NAND_CMD_READ0, 0,
						page_addr);

			host->col_addr = column - mtd->writesize;
			host->spare_only = true;

			/* Set program pointer to spare region */
			if (!host->pagesize_2k)
				send_cmd(host, NAND_CMD_READOOB, false);
		} else {
			host->spare_only = false;
			host->col_addr = column;

			/* Set program pointer to page start */
			if (!host->pagesize_2k)
				send_cmd(host, NAND_CMD_READ0, false);
		}
		break;

	case NAND_CMD_PAGEPROG:
		send_prog_page(host, 0, host->spare_only);
		if (host->pagesize_2k && !cpu_is_mx25()) {
			/* data in 4 areas datas */
			send_prog_page(host, 1, host->spare_only);
			send_prog_page(host, 2, host->spare_only);
			send_prog_page(host, 3, host->spare_only);
		}
		useirq = true;
		break;

	case NAND_CMD_ERASE1:
		break;
	case NAND_CMD_ERASE2:
		useirq = true;
		break;
	}

	/* Write out the command to the device. */
	send_cmd(host, command, useirq);

	/* Write out column address, if necessary */
	if (column != -1) {
		/*
		 * MXC NANDFC can only perform full page+spare or
		 * spare-only read/write.  When the upper layers
		 * layers perform a read/write buf operation,
		 * we will used the saved column adress to index into
		 * the full page.
		 */
		send_addr(host, 0, page_addr == -1);
		if (host->pagesize_2k)
			/* another col addr cycle for 2k page */
			send_addr(host, 0, false);
	}

	/* Write out page address, if necessary */
	if (page_addr != -1) {
		u32 page_mask = nand_chip->pagemask;

		do {
			send_addr(host, (page_addr & 0xff), false);
			page_mask >>= 8;
			page_addr >>= 8;
		} while (page_mask != 0);
	}

	/* Command post-processing step */
	switch (command) {

	case NAND_CMD_RESET:
		break;

	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
		if (host->pagesize_2k) {
			/* send read confirm command */
			send_cmd(host, NAND_CMD_READSTART, true);
			/* read for each AREA */
			send_read_page(host, 0, host->spare_only);
			if (!cpu_is_mx25()) {
				send_read_page(host, 1, host->spare_only);
				send_read_page(host, 2, host->spare_only);
				send_read_page(host, 3, host->spare_only);
			}
		} else
			send_read_page(host, 0, host->spare_only);
		break;

	case NAND_CMD_READID:
		host->col_addr = 0;
		send_read_id(host);
		break;

	case NAND_CMD_PAGEPROG:
		break;

	case NAND_CMD_STATUS:
		break;

	case NAND_CMD_ERASE2:
		break;
	}
}

static int __init mxcnd_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct mxc_nand_platform_data *pdata = pdev->dev.platform_data;
	struct mxc_nand_host *host;
	struct resource *res1, *res2;
	uint16_t tmp;
	int err, nr_parts;

	/* Allocate memory for MTD device structure and private data */
	host = kzalloc(sizeof(struct mxc_nand_host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->dev = &pdev->dev;
	/* structures must be linked */
	this = &host->nand;
	mtd = &host->mtd;
	mtd->priv = this;
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;

	this->priv = host;
	this->dev_ready = mxc_nand_dev_ready;
	this->cmdfunc = mxc_nand_command;
	this->select_chip = mxc_nand_select_chip;
	this->read_byte = mxc_nand_read_byte;
	this->read_word = mxc_nand_read_word;
	this->write_buf = mxc_nand_write_buf;
	this->read_buf = mxc_nand_read_buf;
	this->verify_buf = mxc_nand_verify_buf;
#ifdef CONFIG_MTD_NAND_MXC_FLASH_BBT
	if (cpu_is_mx25()) {
		this->bbt_td = &mx25_bbt_main_descr;
		this->bbt_md = &mx25_bbt_mirror_descr;
	} else {
		this->bbt_td = &mx2_bbt_main_descr;
		this->bbt_md = &mx2_bbt_mirror_descr;
	}
	this->options |= NAND_USE_FLASH_BBT;
#endif

	host->clk = clk_get(&pdev->dev, "nfc_clk");
	if (IS_ERR(host->clk)) {
		err = PTR_ERR(host->clk);
		goto eclk;
	}

	clk_enable(host->clk);
	host->clk_act = 1;

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res1 || !res2) {
		err = -ENODEV;
		goto eres;
	}

	if (!request_mem_region(res1->start, resource_size(res1), "mxc_nand regs")) {
		err = -EBUSY;
		goto ereq1;
	}

	if (!request_mem_region(res2->start, resource_size(res2), "mxc_nand buffer")) {
		err = -EBUSY;
		goto ereq2;
	}

	host->regs = ioremap(res1->start, resource_size(res1));
	if (!host->regs) {
		err = -ENOMEM;
		goto eunmap1;
	}

	host->nfc_buf = ioremap(res2->start, resource_size(res2));
	if (!host->nfc_buf) {
			err = -ENOMEM;
			goto eunmap2;
	}

	tmp = nfc_read_reg(host->regs, NFC_CONFIG1);
	tmp |= NFC_INT_MSK;
	nfc_write_reg(tmp, host->regs, NFC_CONFIG1);
	nfc_write_reg(0, host->regs, NFC_CONFIG2);

	init_waitqueue_head(&host->irq_waitq);

	host->irq = platform_get_irq(pdev, 0);

	err = request_irq(host->irq, mxc_nfc_irq, 0, "mxc_nd", host);
	if (err)
		goto eirq;

	/* Reset NAND */
	this->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* preset operation */
	/* Unlock the internal RAM Buffer */
	nfc_write_reg(0x2, host->regs, NFC_CONFIG);

	/* Blocks to be unlocked */
	nfc_write_reg(0x0, host->regs, NFC_UNLOCKSTART_BLKADDR);
	nfc_write_reg(0x4000, host->regs, NFC_UNLOCKEND_BLKADDR);

	/* Unlock Block Command for given address range */
	nfc_write_reg(0x4, host->regs, NFC_WRPROT);

	/* NAND bus width determines access funtions used by upper layer */
	if (pdata->width == 2) {
		this->options |= NAND_BUSWIDTH_16;
	}

	/* Scan to find existence of the device */
	err = nand_scan_ident(mtd, 1);
	if (err) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "MXC_ND: Unable to find any NAND device.\n");
		goto escan;
	}

	/* this is required before completing the scan */
	host->pagesize_2k = (mtd->writesize == 2048);

	tmp = nfc_read_reg(host->regs, NFC_CONFIG1);
	tmp &= ~(3 << 9); /* clear PPB mask */
	/* set PPB (pages per block) */
	tmp |= (ffs(mtd->erasesize / mtd->writesize) - 6) << 9;
	nfc_write_reg(tmp, host->regs, NFC_CONFIG1);

	if (pdata->hw_ecc) {
		this->ecc.calculate = mxc_nand_calculate_ecc;
		this->ecc.hwctl = mxc_nand_enable_hwecc;
		this->ecc.correct = mxc_nand_correct_data;
		if (cpu_is_mx25()) {
			this->ecc.mode = NAND_ECC_HW_SYNDROME;
			this->ecc.read_page = mxc_nand_read_page_syndrome;
			this->ecc.read_page_raw = mxc_nand_read_page_raw_syndrome;
			this->ecc.read_oob = mxc_nand_read_oob_syndrome;
			this->ecc.write_page = mxc_nand_write_page_syndrome;
			this->ecc.write_page_raw = mxc_nand_write_page_raw_syndrome;
			this->ecc.write_oob = mxc_nand_write_oob_syndrome;
			this->ecc.bytes = 9;
			this->ecc.prepad = 7;
			if (host->pagesize_2k) {
				this->ecc.layout = &mx25_nand_hw_eccoob2k_8;
			} else {
				this->ecc.layout = &mx25_nand_hw_eccoob_8;
			}
		} else {
			this->ecc.mode = NAND_ECC_HW;
			if (host->pagesize_2k) {
				this->ecc.layout = &mx2_nand_hw_eccoob2k_8;
			} else {
				this->ecc.layout = &mx2_nand_hw_eccoob_8;
			}
		}
		this->ecc.size = 512;
		tmp = nfc_read_reg(host->regs, NFC_CONFIG1);
		tmp |= NFC_ECC_EN;
		nfc_write_reg(tmp, host->regs, NFC_CONFIG1);
	} else {
		this->ecc.mode = NAND_ECC_SOFT;
		tmp = nfc_read_reg(host->regs, NFC_CONFIG1);
		tmp &= ~NFC_ECC_EN;
		nfc_write_reg(tmp, host->regs, NFC_CONFIG1);
	}

	err = nand_scan_tail(mtd);
	if (err) {
		goto escan;
	}
	if (cpu_is_mx25()) {
		mtd->flags &= ~MTD_OOB_WRITEABLE;
	}

	pr_info("MXC MTD nand Driver IRQ %d bus width: %u bit %s ECC IO: %08lx\n",
		host->irq, pdata->width * 8, pdata->hw_ecc ? "HW" : "SW",
		(unsigned long)res1->start);

	/* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
	nr_parts =
	    parse_mtd_partitions(mtd, part_probes, &host->parts, 0);
	if (nr_parts > 0)
		add_mtd_partitions(mtd, host->parts, nr_parts);
	else
#endif
	{
		pr_info("Registering %s as whole device\n", mtd->name);
		add_mtd_device(mtd);
	}

	platform_set_drvdata(pdev, host);

	return 0;

escan:
	free_irq(host->irq, host);
eirq:
	if (res2)
		iounmap(host->nfc_buf);
eunmap2:
	iounmap(host->regs);
eunmap1:
	release_mem_region(res2->start, resource_size(res2));
ereq2:
	release_mem_region(res1->start, resource_size(res1));
ereq1:
eres:
	clk_disable(host->clk);
	clk_put(host->clk);
eclk:
	kfree(host);

	return err;
}

static int __exit mxcnd_remove(struct platform_device *pdev)
{
	struct mxc_nand_host *host = platform_get_drvdata(pdev);
	struct resource *res;

	if (host->clk_act)
		clk_disable(host->clk);
	clk_put(host->clk);

	nand_release(&host->mtd);
	free_irq(host->irq, host);
	iounmap(host->regs);
	kfree(host);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		release_mem_region(res->start, resource_size(res));
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		release_mem_region(res->start, resource_size(res));
	}
	return 0;
}

#ifdef CONFIG_PM
static int mxcnd_suspend(struct device *dev)
{
	struct mtd_info *mtd = dev_get_drvdata(dev);
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	int ret = 0;

	DEBUG(MTD_DEBUG_LEVEL0, "MXC_ND : NAND suspend %p\n", mtd->suspend);
	ret = mtd->suspend(mtd);

	if (host->clk_act) {
		/* Disable the NFC clock */
		clk_disable(host->clk);
	}

	return ret;
}

static int mxcnd_resume(struct device *dev)
{
	struct mtd_info *mtd = dev_get_drvdata(dev);
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;
	int ret = 0;

	DEBUG(MTD_DEBUG_LEVEL0, "MXC_ND : NAND resume\n");

	if (host->clk_act) {
		/* Enable the NFC clock */
		clk_enable(host->clk);
	}
	mtd->resume(mtd);

	return ret;
}

#else
# define mxcnd_suspend   NULL
# define mxcnd_resume    NULL
#endif				/* CONFIG_PM */

static struct dev_pm_ops mxcnd_pm_ops = {
	.suspend = mxcnd_suspend,
	.resume = mxcnd_resume,
};

static struct platform_driver mxcnd_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .pm = &mxcnd_pm_ops,
	},
	.remove = __exit_p(mxcnd_remove),
};

static int __init mxc_nd_init(void)
{
	int ret;

	/* Register the device driver structure. */
	ret = platform_driver_probe(&mxcnd_driver, mxcnd_probe);
	if (ret != 0) {
		printk(KERN_ERR "Driver register failed for mxcnd_driver\n");
	}
	return ret;
}

static void __exit mxc_nd_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mxcnd_driver);
}

module_init(mxc_nd_init);
module_exit(mxc_nd_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC NAND MTD driver");
MODULE_LICENSE("GPL");
