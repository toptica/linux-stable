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

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#include <mach/mxc_nand.h>

#ifdef DEBUG
static int debug = 1;
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

#include "mxc_nd2.h"

#define DRV_VER "2.5"

//static struct mxc_nand_host *host;

/*
 * Define delays in microsec for NAND device operations
 */
#define TROP_US_DELAY   2000

#ifdef CONFIG_MTD_NAND_MXC_SWECC
static int hardware_ecc = 0;
#else
static int hardware_ecc = 1;
#endif

/*
 * OOB placement block for use with hardware ecc generation
 */
static struct nand_ecclayout nand_hw_eccoob_512 = {
	.eccbytes = 8,
	.eccpos = { 8, 9, 10, 11, 12, 13, 14, 15, },
	.oobfree = {
		{ 0, 8, },
	},
};

static struct nand_ecclayout nand_hw_eccoob_2k = {
	.eccbytes = 4 * 8,
	.eccpos = {
		8, 9, 10, 11, 12, 13, 14, 15,
		24, 25, 26, 27, 28, 29, 30, 31,
		40, 41, 42, 43, 44, 45, 46, 47,
		56, 57, 58, 59, 60, 61, 62, 63,
	},
	.oobfree = {
		{  2,  6, },
		{ 16, 8, },
		{ 32, 8, },
		{ 48, 8, },
	},
};

static struct nand_ecclayout nand_hw_eccoob_4k = {
	.eccbytes = 8 * 14,
	.eccpos = { 8,  9, 10, 11, 12, 13, 14,
		    15, 16, 17, 18, 19, 20, 21,
		    24,
	},
	.oobfree = {
		{ 2, 6, },
		{ 22, 6, },
	},
};

/*
 * @defgroup NAND_MTD NAND Flash MTD Driver for MXC processors
 */

/*
 * @file mxc_nd2.c
 *
 * @brief This file contains the hardware specific layer for NAND Flash on
 * MXC processor
 *
 * @ingroup NAND_MTD
 */

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "RedBoot", "cmdlinepart", NULL };
#endif

static irqreturn_t mxc_nfc_irq(int irq, void *dev_id)
{
	struct mxc_nand_host *host = dev_id;

	/* Disable Interrupt */
	raw_write(raw_read(REG_NFC_INTRRUPT) | NFC_INT_MSK, REG_NFC_INTRRUPT);
	wake_up(&host->irq_waitq);

	return IRQ_HANDLED;
}

#if 0
static void nfc_memcpy(void *dest, void *src, int len)
{
	u32 *wp = dest;
	u32 *rp = src;

	DBG(1, "%s: Copying %u byte from %p..%p to %p..%p\n", __FUNCTION__,
		len, src, src + len - 1, dest, dest + len - 1);
	while (len >= 4) {
		*wp++ = *rp++;
		len -= 4;
	}
	while (len >= 2) {
		u16 *wp2 = (u16 *)wp;
		u16 *rp2 = (u16 *)rp;

		*wp2++ = *rp2++;
		len -= 2;
	}
	if (len)
		BUG();
}

/*
 * Functions to transfer data to/from spare erea.
 */
static void
copy_spare(struct mtd_info *mtd, void *pbuf, void *pspare, int len, bool bfrom)
{
	u16 i, j;
	u16 m = mtd->oobsize;
	u16 n = mtd->writesize >> 9;
	u8 *d = pbuf;
	u8 *s = pspare;
	u16 t = SPARE_LEN;

	j = (m / n >> 1) << 1;

	if (bfrom) {
		for (i = 0; i < n - 1; i++)
			nfc_memcpy(&d[i * j], &s[i * t], j);

		/* the last section */
		nfc_memcpy(&d[i * j], &s[i * t], len - i * j);
	} else {
		for (i = 0; i < n - 1; i++)
			nfc_memcpy(&s[i * t], &d[i * j], j);

		/* the last section */
		nfc_memcpy(&s[i * t], &d[i * j], len - i * j);
	}
}
#endif

static void mxc_nand_copy_spare(struct mtd_info *mtd, int mode)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
	int i;

	if (mode) {
		for (i = 1; i < mtd->oobsize / 16; i++) {
			void __iomem *src = host->nfc_buf + SPARE_AREA0 + i * 16;
			void __iomem *dst = host->nfc_buf + SPARE_AREA0 + i * SPARE_LEN;

			DBG(2, "%s: Copying %p..%p to %p..%p\n", __FUNCTION__,
				src, src + 15, dst, dst + 15);
			memcpy(dst, src, 16);
		}
	} else {
		for (i = 1; i < mtd->oobsize / 16; i++) {
			void __iomem *src = host->nfc_buf + SPARE_AREA0 + i * SPARE_LEN;
			void __iomem *dst = host->nfc_buf + SPARE_AREA0 + i * 16;

			DBG(2, "%s: Copying %p..%p to %p..%p\n", __FUNCTION__,
				src, src + 15, dst, dst + 15);
			memcpy(dst, src, 16);
		}
	}
	if (dbg_lvl(1))
		print_hex_dump(KERN_DEBUG, "spare: ", DUMP_PREFIX_ADDRESS,
			16, 2, host->nfc_buf + SPARE_AREA0, 64, 0);
}

/*
 * This function polls the NFC to wait for the basic operation to complete by
 * checking the INT bit of config2 register.
 *
 * @param       maxRetries     number of retry attempts (separated by 1 us)
 * @param       useirq         True if IRQ should be used rather than polling
 */
static void wait_op_done(struct mxc_nand_host *host, int max_retries,
			int useirq)
{
	if (useirq) {
		if (!(raw_read(REG_NFC_INTRRUPT) & NFC_OPS_STAT)) {
			uint32_t cfg1;
			unsigned long timeout = msecs_to_jiffies(max_retries /
								1000) + 2;

			cfg1 = raw_read(REG_NFC_INTRRUPT);
			cfg1 &= ~NFC_INT_MSK;	/* Enable interrupt */
			raw_write(cfg1, REG_NFC_INTRRUPT);

			max_retries = wait_event_timeout(host->irq_waitq,
							raw_read(REG_NFC_OPS_STAT) & NFC_OPS_STAT,
							timeout);
		}
	} else {
		while (!(raw_read(REG_NFC_OPS_STAT) & NFC_OPS_STAT) &&
			max_retries-- > 0) {
			udelay(1);
		}
	}
	raw_write(raw_read(REG_NFC_OPS_STAT) & ~NFC_OPS_STAT,
		REG_NFC_OPS_STAT);

	if (max_retries <= 0) {
		dev_err(host->dev, "Wait for NAND operation timed out\n");
	}
}

static inline void send_atomic_cmd(struct mxc_nand_host *host,
				u16 cmd, bool useirq)
{
	/* fill command */
	raw_write(cmd, REG_NFC_FLASH_CMD);

	/* clear status */
	ACK_OPS;

	/* send out command */
	raw_write(NFC_CMD, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, useirq);
}

static void mxc_do_addr_cycle(struct mtd_info *mtd, int column, int page_addr);
static int mxc_check_ecc_status(struct mtd_info *mtd);

#ifdef NFC_AUTO_MODE_ENABLE
/*
 * This function handle the interleave related work
 * @param	mtd	mtd info
 * @param	cmd	command
 */
static void auto_cmd_interleave(struct mtd_info *mtd, u16 cmd)
{
	struct nand_chip *chip = mtd->priv;
	struct mxc_nand_host *host = chip->priv;
	u32 page_addr, ncs;
	u32 addr_low = raw_read(NFC_FLASH_ADDR0);
	u32 addr_high = raw_read(NFC_FLASH_ADDR8);

	/* adjust the addr value
	 * since ADD_OP mode is 01
	 */
	if (cmd == NAND_CMD_ERASE2)
		page_addr = addr_low;
	else
		page_addr = addr_low >> 16 | addr_high << 16;

	ncs = page_addr >> (chip->chip_shift - chip->page_shift);

	page_addr *= chip->numchips;
	page_addr += ncs;

	switch (cmd) {
		int loops;
	case NAND_CMD_PAGEPROG:
		/* reset addr cycle */
		mxc_do_addr_cycle(mtd, 0, page_addr++);
		mxc_nand_copy_spare(mtd, 1);
		NFC_SET_RBA(0);
		ACK_OPS;
		raw_write(NFC_AUTO_PROG, REG_NFC_OPS);

		/* wait auto_prog_done bit set */
		loops = 0;
		while (!(raw_read(REG_NFC_OPS_STAT) & NFC_OP_DONE)) {
			loops++;
			if (WARN_ON(loops > 1000)) {
				break;
			}
			udelay(1);
		}

		wait_op_done(host, TROP_US_DELAY, false);
		loops = 0;
		while (!(raw_read(REG_NFC_OPS_STAT) & NFC_RB)) {
			loops++;
			if (WARN_ON(loops > 1000)) {
				break;
			}
			udelay(1);
		}
		break;

	case NAND_CMD_READSTART:
		/* reset addr cycle */
		mxc_do_addr_cycle(mtd, 0, page_addr++);

		NFC_SET_RBA(0);
		ACK_OPS;
		raw_write(NFC_AUTO_READ, REG_NFC_OPS);
		wait_op_done(host, TROP_US_DELAY, false);

		/* check ecc error */
		mxc_check_ecc_status(mtd);
		mxc_nand_copy_spare(mtd, 0);
		break;

	case NAND_CMD_ERASE2:
		mxc_do_addr_cycle(mtd, -1, page_addr++);
		ACK_OPS;
		raw_write(NFC_AUTO_ERASE, REG_NFC_OPS);
		wait_op_done(host, TROP_US_DELAY, true);
		break;

	case NAND_CMD_RESET:
		send_atomic_cmd(host, cmd, false);
		break;
	}
}
#endif

static void send_addr(struct mxc_nand_host *host, u16 addr, bool useirq);

/*
 * This function issues the specified command to the NAND device and
 * waits for completion.
 *
 * @param       cmd     command for NAND Flash
 * @param       useirq  True if IRQ should be used rather than polling
 */
static void send_cmd(struct mtd_info *mtd, u16 cmd, bool useirq)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);

	DEBUG(MTD_DEBUG_LEVEL3, "send_cmd(0x%x, %d)\n", cmd, useirq);

#ifdef NFC_AUTO_MODE_ENABLE
	switch (cmd) {
	case NAND_CMD_READ0:
	case NAND_CMD_READOOB:
		raw_write(NAND_CMD_READ0, REG_NFC_FLASH_CMD);
		break;
	case NAND_CMD_SEQIN:
	case NAND_CMD_ERASE1:
		raw_write(cmd, REG_NFC_FLASH_CMD);
		break;
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE2:
	case NAND_CMD_READSTART:
		raw_write(raw_read(REG_NFC_FLASH_CMD) | cmd << NFC_CMD_1_SHIFT,
			  REG_NFC_FLASH_CMD);
		auto_cmd_interleave(mtd, cmd);
		break;
	case NAND_CMD_READID:
		send_atomic_cmd(host, cmd, useirq);
		send_addr(host, 0, false);
		break;
	case NAND_CMD_RESET:
		auto_cmd_interleave(mtd, cmd);
		break;
	case NAND_CMD_STATUS:
		send_atomic_cmd(host, cmd, useirq);
		break;
	default:
		break;
	}
#else
	send_atomic_cmd(host, cmd, useirq);
#endif
}

/*
 * This function sends an address (or partial address) to the
 * NAND device.  The address is used to select the source/destination for
 * a NAND command.
 *
 * @param       addr    address to be written to NFC.
 * @param       useirq  True if IRQ should be used rather than polling
 */
static void send_addr(struct mxc_nand_host *host, u16 addr, bool useirq)
{
	DEBUG(MTD_DEBUG_LEVEL3, "send_addr(0x%x %d)\n", addr, useirq);

	/* fill address */
	raw_write((addr << NFC_FLASH_ADDR_SHIFT), REG_NFC_FLASH_ADDR);

	/* clear status */
	ACK_OPS;

	/* send out address */
	raw_write(NFC_ADDR, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, useirq);
}

#ifndef NFC_AUTO_MODE_ENABLE
/*
 * This function requests the NFC to initate the transfer
 * of data currently in the NFC RAM buffer to the NAND device.
 *
 * @param	buf_id	      Specify Internal RAM Buffer number
 */
static void send_prog_page(struct mxc_nand_host *host, u8 buf_id)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s\n", __FUNCTION__);

	/* set ram buffer id */
	NFC_SET_RBA(buf_id);

	/* clear status */
	ACK_OPS;

	/* transfer data from NFC ram to nand */
	raw_write(NFC_INPUT, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, false);
}

/*
 * This function requests the NFC to initated the transfer
 * of data from the NAND device into in the NFC ram buffer.
 *
 * @param	buf_id		Specify Internal RAM Buffer number
 */
static void send_read_page(struct mxc_nand_host *host, u8 buf_id)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s(%d)\n", __FUNCTION__, buf_id);

	/* set ram buffer id */
	NFC_SET_RBA(buf_id);

	/* clear status */
	ACK_OPS;

	if (!host->pagesize_2k) {
		if (host->spare_only)
			raw_write(raw_read(REG_NFC_SP_EN) & ~NFC_SP_EN, REG_NFC_SP_EN);
		else
			raw_write(raw_read(REG_NFC_SP_EN) | NFC_SP_EN, REG_NFC_SP_EN);
	}

	/* transfer data from nand to NFC ram */
	raw_write(NFC_OUTPUT, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, false);
}
#else
static inline void read_dev_status(struct mxc_nand_host *host, u16 *status)
{
	/* clear status */
	ACK_OPS;

	/* use atomic mode to read status instead
	   of using auto mode, auto-mode has issues
	   and the status is not correct.
	*/
	raw_write(NFC_STATUS, REG_NFC_OPS);

	wait_op_done(host, TROP_US_DELAY, false);

	*status = (raw_read(NFC_CONFIG1) >> 16) & 0xff;
}
#endif

/*
 * This function requests the NFC to perform a read of the
 * NAND device ID.
 */
static void send_read_id(struct mxc_nand_host *host)
{
	/* Set RBA bits for BUFFER0 */
	NFC_SET_RBA(0);

	/* clear status */
	ACK_OPS;

	/* Read ID into main buffer */
	raw_write(NFC_ID, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, false);
}

/*
 * This function requests the NFC to perform a read of the
 * NAND device status and returns the current status.
 *
 * @return  device status
 */
static u16 get_dev_status(struct mxc_nand_host *host)
{
	u16 ret;
#ifdef NFC_AUTO_MODE_ENABLE
	/* set ative cs */
	NFC_SET_NFC_ACTIVE_CS(0);

	/* FIXME, NFC Auto erase may have
	 * problem, have to poll it until
	 * the nand gets idle, otherwise
	 * it may get error
	 */
	read_dev_status(host, &ret);
#else
	/* Set ram buffer id */
	NFC_SET_RBA(1);

	/* clear status */
	ACK_OPS;

	/* Read status into main buffer */
	raw_write(NFC_STATUS, REG_NFC_OPS);

	/* Wait for operation to complete */
	wait_op_done(host, TROP_US_DELAY, false);

	/* Status is placed in first word of main buffer */
	/* get status, then recovery area 1 data */
	ret = raw_read(host->nfc_buf + MAIN_AREA1);
#endif
	DBG(1, "%s: status=%04x\n", __FUNCTION__, ret);
	return ret;
}

static void mxc_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	switch (mode) {
	case NAND_ECC_WRITE:
		DBG(1, "ECC_MODE=NAND_ECC_WRITE\n");
		break;
	case NAND_ECC_READSYN:
		DBG(1, "ECC_MODE=NAND_ECC_READSYN\n");
		break;
	case NAND_ECC_READ:
		DBG(1, "ECC_MODE=NAND_ECC_READ\n");
		break;
	default:
		dev_warn(mtd->dev.parent, "Unknown ECC_MODE: %d\n", mode);
	}
}

static void _mxc_nand_enable_hwecc(struct mtd_info *mtd, int on)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
	unsigned int ecc = raw_read(REG_NFC_ECC_EN);

	if (on) {
		ecc |= NFC_ECC_EN;
	} else {
		ecc &= ~NFC_ECC_EN;
	}
	raw_write(ecc, REG_NFC_ECC_EN);
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

	DBG(1, "%s: Reading OOB area of page %u to oob %p\n",
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
		host->col_addr = n * eccsize;
		chip->read_buf(mtd, buf, eccsize);
		buf += eccsize;

		host->col_addr = mtd->writesize + n * eccpitch;
		if (chip->ecc.prepad) {
			chip->read_buf(mtd, oob, chip->ecc.prepad);
			oob += chip->ecc.prepad;
		}

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
	_mxc_nand_enable_hwecc(mtd, 0);

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

	DBG(1, "%s: Reading page %u to buf %p oob %p\n", __FUNCTION__,
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

	DBG(1, "%s: ECC: %d:%d:%d pitch: %d steps: %d\n", __FUNCTION__,
		chip->ecc.prepad, chip->ecc.bytes, chip->ecc.postpad,
		eccpitch, steps);
	DBG(1, "%s: Writing oob from %p..%p\n", __FUNCTION__,
		bufpoi, bufpoi + length - 1);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	for (i = 0; i < steps; i++) {
		len = min_t(int, length, eccpitch);

		DBG(1, "%s: write %u byte oob %p..%p at col %03x\n", __FUNCTION__,
			len, bufpoi, bufpoi + len - 1, host->col_addr);
		chip->write_buf(mtd, bufpoi, len);
		bufpoi += len;
		length -= len;
	}
	if (length > 0) {
		DBG(1, "%s: write %u byte postpad %p..%p\n", __FUNCTION__,
			length, bufpoi, bufpoi + length - 1);
		chip->write_buf(mtd, bufpoi, length);
	}
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

/*
 * Function to record the ECC corrected/uncorrected errors resulted
 * after a page read. This NFC detects and corrects upto to 4 symbols
 * of 9-bits each.
 */
static int mxc_check_ecc_status(struct mtd_info *mtd)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
	u32 ecc_stat, err;
	int no_subpages = 1;
	int ret = 0;
	u8 ecc_bit_mask, err_limit;

	ecc_bit_mask = (IS_4BIT_ECC ? 0x7 : 0xf);
	err_limit = (IS_4BIT_ECC ? 0x4 : 0x8);

	no_subpages = mtd->writesize >> 9;

	ecc_stat = GET_NFC_ECC_STATUS();
	do {
		err = ecc_stat & ecc_bit_mask;
		if (err > err_limit) {
			mtd->ecc_stats.failed++;
			printk(KERN_WARNING "UnCorrectable ECC Error\n");
			return -1;
		} else {
			ret += err;
		}
		ecc_stat >>= 4;
	} while (--no_subpages);

	mtd->ecc_stats.corrected += ret;
	if (ret)
		pr_debug("%d Symbol Correctable ECC Error\n", ret);

	return ret;
}

/*
 * Function to correct the detected errors. This NFC corrects all the errors
 * detected. So this function just return 0.
 */
static int mxc_nand_correct_data(struct mtd_info *mtd, u_char * dat,
				 u_char * read_ecc, u_char * calc_ecc)
{
	struct nand_chip *nand_chip = mtd->priv;
	struct mxc_nand_host *host = nand_chip->priv;

	/*
	 * 1-Bit errors are automatically corrected in HW.  No need for
	 * additional correction.  2-Bit errors cannot be corrected by
	 * HW ECC, so we need to return failure
	 */
	if (!(raw_read(REG_NFC_ECC_EN) & NFC_ECC_EN)) {
		DBG(1, "%s: ECC turned off\n", __FUNCTION__);
		return 0;
	} else {
		uint16_t ecc_status = GET_NFC_ECC_STATUS();
		int subpages = mtd->writesize / nand_chip->subpagesize;
		int pg2blk_shift = nand_chip->phys_erase_shift - nand_chip->page_shift;

		if (ecc_status) {
			DBG(ecc_status ? 0 : 1, "%s: ECC_STATUS=%04x\n", __FUNCTION__, ecc_status);
		}

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
	}
	return 0;
}

/*
 * Function to calculate the ECC for the data to be stored in the Nand device.
 * This NFC has a hardware RS(511,503) ECC engine together with the RS ECC
 * CONTROL blocks are responsible for detection  and correction of up to
 * 8 symbols of 9 bits each in 528 byte page.
 * So this function is just return 0.
 */

static int mxc_nand_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
				  u_char * ecc_code)
{
	return 0;
}

/*
 * This function id is used to read the data buffer from the NAND Flash. To
 * read the data from NAND Flash first the data output cycle is initiated by
 * the NFC, which copies the data to RAMbuffer. This data of length \b len is
 * then copied to buffer \b buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be read from NAND Flash
 * @param       len     number of bytes to be read
 */
static void mxc_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
	int n, i = 0, col = host->col_addr;

	DBG(1, "%s(buf=%p col=%03x, len=%03x)\n", __FUNCTION__,
		buf, host->col_addr, len);

	if (col < mtd->writesize && host->spare_only)
		col += mtd->writesize;

	/* If more data is requested to be read than is available in
	 * the flash buffer this is clearly a BUG! */
	BUG_ON(mtd->writesize && len > mtd->writesize + mtd->oobsize - col);
	n = len;

	while (n > 0) {
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
		BUG_ON(n < 0);
	}
	/* Update saved column address */
	host->col_addr = col;
}

/*
 * This function reads byte from the NAND Flash
 *
 * @param       mtd     MTD structure for the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static uint8_t mxc_nand_read_byte(struct mtd_info *mtd)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
	uint8_t ret;
	uint16_t col, rd_word;
	uint16_t __iomem *main_buf = host->nfc_buf + MAIN_AREA0;
	uint16_t __iomem *spare_buf = host->nfc_buf + SPARE_AREA0;

	DBG(1, "%s: col=%d status_request=%d\n", __FUNCTION__,
		host->col_addr, host->status_request);

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
	DBG(1, "%s: Read %02x\n", __FUNCTION__, ret);
	return ret;
}

/*
  * This function reads word from the NAND Flash
  *
  * @param     mtd     MTD structure for the NAND Flash
  *
  * @return    data read from the NAND Flash
  */
static u16 mxc_nand_read_word(struct mtd_info *mtd)
{
	union {
		u8 b[2];
		u16 w;
	} ret;

	mxc_nand_read_buf(mtd, ret.b, sizeof(ret));
	return ret.w;
}

/*
 * This function reads byte from the NAND Flash
 *
 * @param     mtd     MTD structure for the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static u_char mxc_nand_read_byte16(struct mtd_info *mtd)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);

	/* Check for status request */
	if (host->status_request) {
		return get_dev_status(host) & 0xFF;
	}

	return mxc_nand_read_word(mtd) & 0xFF;
}

/*
 * This function writes data of length \b len from buffer \b buf to the NAND
 * internal RAM buffer's MAIN area 0.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be written to NAND Flash
 * @param       len     number of bytes to be written
 */
static void mxc_nand_write_buf(struct mtd_info *mtd,
			       const u_char *buf, int len)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
	int n, i = 0, col = host->col_addr;

	DBG(1, "%s(buf=%p col=%03x, len=%03x)\n", __FUNCTION__,
		buf, host->col_addr, len);
	BUG_ON(len < 0);

	col = host->col_addr;

	/* Adjust saved column address */
	if (col < mtd->writesize && host->spare_only)
		col += mtd->writesize;

	/* If more data is requested to be written than free space in
	 * the flash buffer this is clearly a BUG! */
	BUG_ON(len > mtd->writesize + mtd->oobsize - col);
	n = len;

	DBG(3, "%s:%d: col=%03x, n=%03x\n", __func__, __LINE__, col, n);

	while (n > 0) {
		void __iomem *p;

		if (col < mtd->writesize)
			p = host->nfc_buf + MAIN_AREA0 + (col & ~3);
		else
			p = host->nfc_buf + SPARE_AREA0 +
				(col & ~3) - mtd->writesize;

		DBG(3, "%s: p=%p\n", __func__, p);

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
			if (m < 0) {
				DBG(1, "%s: m=%d n=%d writesize=%d oobsize=%d col=%d\n",
					__FUNCTION__, m, n, mtd->writesize, mtd->oobsize, col);
				BUG();
			}

			m = min(n, m) & ~3;
			memcpy(p, &buf[i], m);
			col += m;
			i += m;
			n -= m;
		}
		BUG_ON(n < 0);
	}
	/* Update saved column address */
	host->col_addr = col;
}

/*
 * This function is used by the upper layer to verify the data in NAND Flash
 * with the data in the \b buf.
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       buf     data to be verified
 * @param       len     length of the data to be verified
 *
 * @return      -EFAULT if error else 0
 *
 */
static int mxc_nand_verify_buf(struct mtd_info *mtd, const u_char *buf,
			       int len)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
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

/*
 * This function is used by upper layer for select and deselect of the NAND
 * chip
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       chip    val indicating select or deselect
 */
static void mxc_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);

	switch (chip) {
	case -1:
		/* Disable the NFC clock */
		if (host->clk_act) {
			clk_disable(host->clk);
			host->clk_act = 0;
		}
		break;
	case 0 ... 7:
		/* Enable the NFC clock */
		if (!host->clk_act) {
			clk_enable(host->clk);
			host->clk_act = 1;
		}
		NFC_SET_NFC_ACTIVE_CS(chip);
		break;
	}
}

/*
 * Function to perform the address cycles.
 */
static void mxc_do_addr_cycle(struct mtd_info *mtd, int column, int page_addr)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);

#ifdef NFC_AUTO_MODE_ENABLE
	if (page_addr != -1 && column != -1) {
		u32 mask = 0xFFFF;
		/* the column address */
		raw_write(column & mask, NFC_FLASH_ADDR0);
		raw_write((raw_read(NFC_FLASH_ADDR0) |
			   ((page_addr & mask) << 16)), NFC_FLASH_ADDR0);
		/* the row address */
		raw_write(((raw_read(NFC_FLASH_ADDR8) & (mask << 16)) |
			   ((page_addr & (mask << 16)) >> 16)),
			  NFC_FLASH_ADDR8);
	} else if (page_addr != -1) {
		raw_write(page_addr, NFC_FLASH_ADDR0);
		raw_write(0, NFC_FLASH_ADDR8);
	}

	DBG(3, "AutoMode:the ADDR REGS value is (0x%08x, 0x%08x)\n",
		raw_read(NFC_FLASH_ADDR0), raw_read(NFC_FLASH_ADDR8));
#else
	struct nand_chip *chip = mtd->priv;
	u32 page_mask = chip->pagemask;

	if (column != -1) {
		send_addr(host, column & 0xFF, false);
		if (IS_2K_PAGE_NAND) {
			/* another col addr cycle for 2k page */
			send_addr(host, (column >> 8) & 0xF, false);
		} else if (IS_4K_PAGE_NAND) {
			/* another col addr cycle for 4k page */
			send_addr(host, (column >> 8) & 0x1F, false);
		}
	}
	if (page_addr != -1) {
		do {
			send_addr(host, (page_addr & 0xff), false);
			page_mask >>= 8;
			page_addr >>= 8;
		} while (page_mask != 0);
	}
#endif
}

/*
 * This function is used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash
 *
 * @param       mtd             MTD structure for the NAND Flash
 * @param       command         command for NAND Flash
 * @param       column          column offset for the page read
 * @param       page_addr       page to be read from NAND Flash
 */
static void mxc_nand_command(struct mtd_info *mtd, unsigned int command,
			     int column, int page_addr)
{
	struct mxc_nand_host *host = container_of(mtd, struct mxc_nand_host, mtd);
	bool useirq = false;

	DBG(3, "%s: cmd=0x%08x col=0x%08x page=0x%08x\n", __FUNCTION__,
		command, column, page_addr);
	/*
	 * Reset command state information
	 */
	host->status_request = false;

	/*
	 * Command pre-processing step
	 */
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
		command = NAND_CMD_READ0;
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
				send_cmd(mtd, NAND_CMD_READOOB, false);
		} else {
			host->col_addr = column;
			host->spare_only = false;

			/* Set program pointer to page start */
			if (!host->pagesize_2k)
				send_cmd(mtd, NAND_CMD_READ0, false);
		}
		break;

	case NAND_CMD_PAGEPROG:
#ifndef NFC_AUTO_MODE_ENABLE
		/* NOTE: The NFC interal buffer
		 * access has some limitation, it
		 * does not allow byte access. To
		 * make the code simple and ease use
		 * not every time check the address
		 * alignment. Use the temp buffer
		 * to accomadate the alignment. Since we
		 * know data_buf will be at leat 4
		 * byte aligned, so we can use
		 * memcpy safely
		 */
		mxc_nand_copy_spare(mtd, 1);
		if (host->pagesize_2k)
			PROG_PAGE();
		else
			send_prog_page(host, 0);
#endif
		useirq = true;
		break;

	case NAND_CMD_ERASE1:
		break;

	case NAND_CMD_ERASE2:
		useirq = true;
		break;
	}

	/*
	 * Write out the command to the device.
	 */
	send_cmd(mtd, command, useirq);

	mxc_do_addr_cycle(mtd, column, page_addr);

	/*
	 * Command post-processing step
	 */
	switch (command) {
	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
		if (host->pagesize_2k) {
			/* send read confirm command */
			send_cmd(mtd, NAND_CMD_READSTART, false);
		}
#ifndef NFC_AUTO_MODE_ENABLE
		if (host->pagesize_2k) {
			/* read for each AREA */
			READ_PAGE();
		} else {
			send_read_page(host, 0);
		}
		/* NOTE: the NFC interal buffer
		 * access has some limitation, it
		 * does not allow byte access. To
		 * make the code simple and ease use
		 * not every time check the address
		 * alignment.Use the temp buffer
		 * to accomadate the data.since We
		 * know data_buf will be at leat 4
		 * byte alignment, so we can use
		 * memcpy safely
		 */
		mxc_nand_copy_spare(mtd, 0);
#endif
		break;

	case NAND_CMD_READID:
		send_read_id(host);
		host->col_addr = column;
		break;
	}
}

/* Generic flash bbt decriptors
*/
static uint8_t bbt_pattern[] = { 'B', 'b', 't', '0' };
static uint8_t mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | 0 * NAND_BBT_CREATE | 0 * NAND_BBT_WRITE |
	NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs = 0,
	.len = 4,
	.veroffs = 4,
	.maxblocks = 4,
	.pattern = bbt_pattern,
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | 0 * NAND_BBT_CREATE | 0 * NAND_BBT_WRITE |
	NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs = 0,
	.len = 4,
	.veroffs = 4,
	.maxblocks = 4,
	.pattern = mirror_pattern,
};

static int mxc_nand_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	/* limit to 2G size due to Kernel
	 * larger than 4G space support, need fix
	 * it later
	 */
	if (mtd->size == 0) {
		mtd->size = 1 << 31;
		chip->numchips = 1;
		chip->chipsize = mtd->size;
	}

	/* The ECC algorithm of the NFC does not allow for a writeable OOB area */
	mtd->flags &= ~MTD_OOB_WRITEABLE;

	/* use flash based bbt */
	chip->bbt_td = &bbt_main_descr;
	chip->bbt_md = &bbt_mirror_descr;

	/* update flash based bbt */
	chip->options |= NAND_USE_FLASH_BBT;

	/* Build bad block table */
	return nand_scan_bbt(mtd, chip->badblock_pattern);
}

static void mxc_nfc_init(struct mxc_nand_host *host)
{
	/* Disable interrupt */
	raw_write((raw_read(REG_NFC_INTRRUPT) | NFC_INT_MSK), REG_NFC_INTRRUPT);

	/* disable spare enable */
	raw_write(raw_read(REG_NFC_SP_EN) & ~NFC_SP_EN, REG_NFC_SP_EN);

	raw_write(NFC_IPC_CREQ, NFC_IPC);

	/* Unlock the internal RAM Buffer */
	raw_write(NFC_SET_BLS(NFC_BLS_UNLOCKED), REG_NFC_BLS);

	/* Blocks to be unlocked */
	UNLOCK_ADDR(0x0, 0xFFFF);

	/* Unlock Block Command for given address range */
	raw_write(NFC_SET_WPC(NFC_WPC_UNLOCK), REG_NFC_WPC);

	if (cpu_is_mx51()) {
		/* workaround for ENGcm09970: NFC Doesn't Work Properly when rbb_mode=0 (read status) */
		raw_write(raw_read(NFC_CONFIG3) | NFC_RBB_MODE, NFC_CONFIG3);
	}
	raw_write(0, NFC_IPC);
#ifdef CONFIG_MACH_MX37
	/* Enable symmetric mode by default except mx37TO1.0 */
	if (mx37_revision() == MX37_CHIP_REV_1_0)
		return;
#endif
	raw_write(raw_read(REG_NFC_ONE_CYCLE) |
		NFC_ONE_CYCLE, REG_NFC_ONE_CYCLE);
}

/*
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and
 *                remove functions
 *
 * @return  The function always returns 0.
 */
static int __init mxcnd_probe(struct platform_device *pdev)
{
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct mxc_nand_platform_data *pdata = pdev->dev.platform_data;
	struct mxc_nand_host *host;
	int nr_parts, err;
	struct resource *res1, *res2, *res3;
	unsigned int tmp;

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res1 || !res2) {
		return -ENODEV;
	}
	if (nfc_is_v3()) {
		res3 = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (!res3) {
			printk(KERN_ERR "Missing memory resource for IP registers\n");
			return -ENODEV;
		}
	}
	if (!request_mem_region(res1->start, resource_size(res1),
					"NFC AXI regs")) {
		return -EBUSY;
	}

	if (!request_mem_region(res2->start, resource_size(res2),
					"NFC buffer")) {
		err = -EBUSY;
		goto release1;
	}

	if (res3 && !request_mem_region(res3->start, resource_size(res3),
						"NFC IP regs")) {
		err = -EBUSY;
		goto release2;
	}

	/* Allocate memory for MTD device structure and private data */
	host = kzalloc(sizeof(struct mxc_nand_host), GFP_KERNEL);
	if (!host) {
		printk(KERN_ERR "%s: failed to allocate mtd_info\n",
			__FUNCTION__);
		err = -ENOMEM;
		goto release3;
	}

	host->axi_base = ioremap(res1->start, resource_size(res1));
	if (host->axi_base == NULL) {
		err = -ENOMEM;
		goto free_host;
	}
	DBG(0, "%s: AXI registers @ %08x mapped to %p\n", __FUNCTION__,
		res1->start, host->axi_base);

	host->nfc_buf = ioremap(res2->start, resource_size(res2));
	if (host->nfc_buf == NULL) {
		err = -ENOMEM;
		goto unmap1;
	}
	DBG(0, "%s: NFC buffers @ %08x mapped to %p\n", __FUNCTION__,
		res2->start, host->nfc_buf);

	if (res3) {
		host->ip_base = ioremap(res3->start, resource_size(res3));
		if (host->ip_base == NULL) {
			err = -ENOMEM;
			goto unmap2;
		}
		DBG(0, "%s: IP registers @ %08x mapped to %p\n", __FUNCTION__,
			res3->start, host->ip_base);
	}

	host->clk = clk_get(&pdev->dev, "nfc_clk");
	if (IS_ERR(host->clk)) {
		err = PTR_ERR(host->clk);
		goto unmap3;
	}

	/* init the nfc */
	mxc_nfc_init(host);

	host->dev = &pdev->dev;
	/* structures must be linked */
	chip = &host->nand;
	mtd = &host->mtd;
	mtd->priv = chip;
	mtd->owner = THIS_MODULE;

	chip->priv = host;
	chip->cmdfunc = mxc_nand_command;
	chip->select_chip = mxc_nand_select_chip;
	chip->read_byte = mxc_nand_read_byte;
	chip->read_word = mxc_nand_read_word;
	chip->write_buf = mxc_nand_write_buf;
	chip->read_buf = mxc_nand_read_buf;
	chip->verify_buf = mxc_nand_verify_buf;
	chip->scan_bbt = mxc_nand_scan_bbt;
#ifdef CONFIG_MTD_NAND_MXC_FLASH_BBT
	chip->bbt_td = &mxcnd_bbt_main_descr;
	chip->bbt_md = &mxcnd_bbt_mirror_descr;
	chip->options |= NAND_USE_FLASH_BBT;
#endif

	/* NAND bus width determines access funtions used by upper layer */
	if (pdata->width == 2) {
		chip->read_byte = mxc_nand_read_byte16;
		chip->options |= NAND_BUSWIDTH_16;
		NFC_SET_NFMS(host, 1 << NFMS_NF_DWIDTH);
	} else {
		NFC_SET_NFMS(host, 0);
	}

	clk_enable(host->clk);
	host->clk_act = 1;

	init_waitqueue_head(&host->irq_waitq);

	host->irq = platform_get_irq(pdev, 0);
	err = request_irq(host->irq, mxc_nfc_irq, 0, "mxc_nd", host);
	if (err) {
		goto putclk;
	}

	/* Reset NAND */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Scan to find existence of the device */
	err = nand_scan_ident(mtd, NFC_GET_MAXCHIP_SP());
	if (err) {
		printk(KERN_ERR "MXC_ND2: Unable to find any NAND device\n");
		goto free_irq;
	}
	DBG(1, "%s: Found %d NAND devices\n", __FUNCTION__, chip->numchips);

	host->pagesize_2k = (mtd->writesize == 2048);

	tmp = raw_read(REG_NFC_PPB);
	tmp &= ~NFC_PPB_MASK;
	tmp |= (ffs(mtd->erasesize / mtd->writesize) - 6) << NFC_PPB_SHIFT;
	raw_write(tmp, REG_NFC_PPB);

	if (IS_2K_PAGE_NAND) {
		NFC_SET_NFMS(host, 1 << NFMS_NF_PG_SZ);
		chip->ecc.layout = &nand_hw_eccoob_2k;
		chip->ecc.prepad = 8;
		chip->ecc.bytes = 8;
	} else if (IS_4K_PAGE_NAND) {
		NFC_SET_NFMS(host, 1 << NFMS_NF_PG_SZ);
		chip->ecc.layout = &nand_hw_eccoob_4k;
		chip->ecc.prepad = 8;
		chip->ecc.bytes = 14;
		chip->ecc.postpad = 4;
	} else {
		chip->ecc.layout = &nand_hw_eccoob_512;
		chip->ecc.prepad = 8;
		chip->ecc.bytes = 8;
	}

	/* propagate ecc.layout to mtd_info */
	mtd->ecclayout = chip->ecc.layout;

	if (hardware_ecc) {
		chip->ecc.calculate = mxc_nand_calculate_ecc;
		chip->ecc.hwctl = mxc_nand_enable_hwecc;
		chip->ecc.correct = mxc_nand_correct_data;
		chip->ecc.mode = NAND_ECC_HW_SYNDROME;
		chip->ecc.read_page = mxc_nand_read_page_syndrome;
		chip->ecc.read_page_raw = mxc_nand_read_page_raw_syndrome;
		chip->ecc.read_oob = mxc_nand_read_oob_syndrome;
		chip->ecc.write_page = mxc_nand_write_page_syndrome;
		chip->ecc.write_page_raw = mxc_nand_write_page_raw_syndrome;
		chip->ecc.write_oob = mxc_nand_write_oob_syndrome;
		chip->ecc.size = 512;
		raw_write((raw_read(REG_NFC_ECC_EN) | NFC_ECC_EN),
			  REG_NFC_ECC_EN);
	} else {
		chip->ecc.mode = NAND_ECC_SOFT;
		raw_write((raw_read(REG_NFC_ECC_EN) & ~NFC_ECC_EN),
			  REG_NFC_ECC_EN);
	}

	err = nand_scan_tail(mtd);
	if (err) {
		goto free_irq;
	}

	DBG(1, "%s: Parsing MTD partitions\n", __FUNCTION__);

	/* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
	nr_parts = parse_mtd_partitions(mtd, part_probes, &host->parts, 0);
	if (nr_parts > 0)
		add_mtd_partitions(mtd, host->parts, nr_parts);
	else
#endif
	{
		pr_info("Registering %s as whole device\n", mtd->name);
		add_mtd_device(mtd);
	}
	DBG(1, "%s: NAND scan finished\n", __FUNCTION__);

	platform_set_drvdata(pdev, mtd);
	return 0;

free_irq:
	free_irq(host->irq, host);
putclk:
	clk_put(host->clk);
unmap3:
	if (host->ip_base)
		iounmap(host->ip_base);
unmap2:
	iounmap(host->nfc_buf);
unmap1:
	iounmap(host->axi_base);
free_host:
	kfree(host);
release3:
	if (res3)
		release_mem_region(res3->start, resource_size(res3));
release2:
	release_mem_region(res2->start, resource_size(res2));
release1:
	release_mem_region(res1->start, resource_size(res1));
	return err;
}

 /*
  * Dissociates the driver from the device.
  *
  * @param   pdev  the device structure used to give information on which
  *
  * @return  The function always returns 0.
  */

static int __exit mxcnd_remove(struct platform_device *pdev)
{
	struct mxc_nand_host *host = platform_get_drvdata(pdev);
	struct resource *res;

	if (host->clk_act)
		clk_disable(host->clk);
	clk_put(host->clk);

	nand_release(&host->mtd);
	free_irq(host->irq, host);
	iounmap(host->axi_base);
	iounmap(host->nfc_buf);
	if (host->ip_base)
		iounmap(host->ip_base);
	kfree(host);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	BUG_ON(!res);
	release_mem_region(res->start, resource_size(res));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	BUG_ON(!res);
	release_mem_region(res->start, resource_size(res));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res) {
		release_mem_region(res->start, resource_size(res));
	}

	return 0;
}

/*
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcnd_driver = {
	.driver = {
		.name = "mxc_nand",
	},
	.probe = mxcnd_probe,
	.remove = __exit_p(mxcnd_remove),
};

/*
 * Main initialization routine
 * @return  0 if successful; non-zero otherwise
 */
static int __init mxc_nd_init(void)
{
	int ret;

	pr_info("MXC MTD nand Driver %s\n", DRV_VER);
	ret = platform_driver_register(&mxcnd_driver);
	if (ret) {
		printk(KERN_ERR "Driver register failed for mxc_nand driver\n");
		return ret;
	}
	return 0;
}

/*
 * Clean up routine
 */
static void __exit mxc_nd_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mxcnd_driver);
}

module_init(mxc_nd_init);
module_exit(mxc_nd_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC NAND MTD driver Version 2-5");
MODULE_LICENSE("GPL");
