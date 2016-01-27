/*
 * imx-ssi.c  --  SSI driver for Freescale IMX
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Based on mxc-alsa-mc13783 (C) 2006-2009 Freescale Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    29th Aug 2006   Initial version.
 *
 * TODO:
 *   Need to rework SSI register defs when new defs go into mainline.
 *   Add support for TDM and FIFO 1.
 *
 *    23rd Feb 2010   Adapted for 2.6.31 by <LW@KARO-electronics.de>
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/ssi_port.h>

#include "imx-ssi.h"
#include "imx-pcm.h"

/* debug */
#ifdef DEBUG
static int debug = 1;
#define dbg_lvl(n)	((n) < debug)
module_param(debug, int, S_IRUGO | S_IWUSR);
#else
static int debug;
#define dbg_lvl(n)	0
module_param(debug, int, 0);
#endif

#define DBG(lvl, fmt...)	do {			\
	if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt);	\
} while (0)

static struct imx_ssi_hw *imx_ssi_hw[2];

static inline u32 ssi_reg_read(struct imx_ssi_hw *ssi_hw, unsigned int reg)
{
	return __raw_readl(ssi_hw->mapbase + reg);
}

static inline void ssi_reg_write(struct imx_ssi_hw *ssi_hw, u32 value,
				unsigned int reg)
{
	__raw_writel(value, ssi_hw->mapbase + reg);
}

#define IMX_SSI_DUMP 0
#if IMX_SSI_DUMP
#define SSI_DUMP()						\
	do {							\
		printk(KERN_INFO "dump @ %s\n", __func__);	\
		printk(KERN_INFO "scr %08x\n",		\
			ssi_reg_read(ssi_hw, SSI_SCR));	\
		printk(KERN_INFO "sisr %08x\n",		\
			ssi_reg_read(ssi_hw, SSI_SISR));	\
		printk(KERN_INFO "stcr %08x\n",		\
			ssi_reg_read(ssi_hw, SSI_STCR));	\
		printk(KERN_INFO "srcr %08x\n",		\
			ssi_reg_read(ssi_hw, SSI_SRCR));	\
		printk(KERN_INFO "stccr %08x\n",	\
			ssi_reg_read(ssi_hw, SSI_STCCR));	\
		printk(KERN_INFO "srccr %08x\n",	\
			ssi_reg_read(ssi_hw, SSI_SRCCR));	\
		printk(KERN_INFO "sfcsr %08x\n",	\
			ssi_reg_read(ssi_hw, SSI_SFCSR));	\
		printk(KERN_INFO "stmsk %08x\n",	\
			ssi_reg_read(ssi_hw, SSI_STMSK));	\
		printk(KERN_INFO "srmsk %08x\n",	\
			ssi_reg_read(ssi_hw, SSI_SRMSK));	\
		printk(KERN_INFO "sier %08x\n",		\
			ssi_reg_read(ssi_hw, SSI_SIER));	\
	} while (0);
#else
#define SSI_DUMP()
#endif

#define SSI1_PORT	0
#define SSI2_PORT	1

/*
 * SSI system clock configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	u32 scr;
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;

	DBG(0, "%s: ID: %08x freq: %u dir: %d\n", __FUNCTION__,
		dai->id, freq, dir);

	scr = ssi_reg_read(ssi_hw, SSI_SCR);

	if (scr & SSI_SCR_SSIEN)
		return 0;

	switch (clk_id) {
	case IMX_SSP_SYS_CLK:
		if (dir == SND_SOC_CLOCK_OUT)
			scr |= SSI_SCR_SYS_CLK_EN;
		else
			scr &= ~SSI_SCR_SYS_CLK_EN;
		break;
	default:
		return -EINVAL;
	}

	ssi_reg_write(ssi_hw, scr, SSI_SCR);

	return 0;
}

/*
 * SSI Clock dividers
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_clkdiv(struct snd_soc_dai *dai,
				  int div_id, int div)
{
	u32 stccr, srccr;
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;

	DBG(0, "%s: id=%08x div=%u\n", __FUNCTION__, div_id, div);

	if (ssi_reg_read(ssi_hw, SSI_SCR) & SSI_SCR_SSIEN)
		return 0;

	srccr = ssi_reg_read(ssi_hw, SSI_SRCCR);
	stccr = ssi_reg_read(ssi_hw, SSI_STCCR);

	switch (div_id) {
	case IMX_SSI_TX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	case IMX_SSI_RX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	default:
		return -EINVAL;
	}

	ssi_reg_write(ssi_hw, stccr, SSI_STCCR);
	ssi_reg_write(ssi_hw, srccr, SSI_SRCCR);

	return 0;
}

/*
 * SSI Network Mode or TDM slots configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_tdm_slot(struct snd_soc_dai *dai,
				    unsigned int mask, int slots)
{
	u32 stmsk, srmsk, stccr;
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;

	DBG(0, "%s: Configuring %u slots\n", __FUNCTION__, slots);

	if (ssi_reg_read(ssi_hw, SSI_SCR) & SSI_SCR_SSIEN)
		return 0;
	stccr = ssi_reg_read(ssi_hw, SSI_STCCR);

	stmsk = srmsk = mask;
	stccr &= ~SSI_STCCR_DC_MASK;
	stccr |= SSI_STCCR_DC(slots - 1);

	ssi_reg_write(ssi_hw, stmsk, SSI_STMSK);
	ssi_reg_write(ssi_hw, srmsk, SSI_SRMSK);
	ssi_reg_write(ssi_hw, stccr, SSI_STCCR);
	ssi_reg_write(ssi_hw, stccr, SSI_SRCCR);

	return 0;
}

/*
 * SSI DAI format configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 * Note: We don't use the I2S modes but instead manually configure the
 * SSI for I2S.
 */
static int imx_ssi_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;
	u32 stcr = 0, srcr = 0, scr;

	scr = ssi_reg_read(ssi_hw, SSI_SCR) & ~(SSI_SCR_SYN | SSI_SCR_NET);
	if (scr & SSI_SCR_SSIEN)
		return 0;

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* data on rising edge of bclk, frame low 1clk before data */
		stcr |= SSI_STCR_TFSI | SSI_STCR_TEFS | SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_REFS | SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TFSL;
		srcr |= SSI_SRCR_RFSL;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* data on rising edge of bclk, frame high 1clk before data */
		stcr |= SSI_STCR_TFSL | SSI_STCR_TEFS;
		srcr |= SSI_SRCR_RFSL | SSI_SRCR_REFS;
		break;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		stcr &= ~(SSI_STCR_TSCKP | SSI_STCR_TFSI);
		srcr &= ~(SSI_SRCR_RSCKP | SSI_SRCR_RFSI);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		stcr |= SSI_STCR_TFSI;
		stcr &= ~SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI;
		srcr &= ~SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		stcr &= ~SSI_STCR_TFSI;
		stcr |= SSI_STCR_TSCKP;
		srcr &= ~SSI_SRCR_RFSI;
		srcr |= SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		stcr |= SSI_STCR_TFSI | SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_RSCKP;
		break;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		stcr |= SSI_STCR_TFDIR | SSI_STCR_TXDIR;
		if (((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S)
			&& ssi_mode->network_mode) {
			scr &= ~SSI_SCR_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_MSTR;
		}
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		stcr |= SSI_STCR_TFDIR;
		srcr |= SSI_SRCR_RFDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		stcr |= SSI_STCR_TXDIR;
		srcr |= SSI_SRCR_RXDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		if (((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S) &&
			ssi_mode->network_mode) {
			scr &= ~SSI_SCR_I2S_MODE_MASK;
			scr |= SSI_SCR_I2S_MODE_SLAVE;
		}
		break;
	}

	/* sync */
	if (ssi_mode->sync_mode)
		scr |= SSI_SCR_SYN;

	/* tdm - only for stereo atm */
	if (ssi_mode->network_mode)
		scr |= SSI_SCR_NET;

	ssi_reg_write(ssi_hw, stcr, SSI_STCR);
	ssi_reg_write(ssi_hw, srcr, SSI_SRCR);
	ssi_reg_write(ssi_hw, scr, SSI_SCR);

	SSI_DUMP();
	return 0;
}

static int imx_ssi_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;

	DBG(0, "%s: ssi_mode=%p ssi_hw=%p\n", __FUNCTION__,
		ssi_mode, ssi_hw);

	/* we cant really change any SSI values after SSI is enabled
	 * need to fix in software for max flexibility - lrg */
	if (dai->playback.active || dai->capture.active)
		return 0;

	/* reset the SSI port - Sect 45.4.4 */
	if (ssi_hw->active++)
		return 0;

	ssi_reg_write(ssi_hw, 0, SSI_SCR);
	clk_enable(ssi_hw->clk);

	/* BIG FAT WARNING
	 * SDMA FIFO watermark must be == SSI FIFO watermark for
	 * best results.
	 */
	ssi_reg_write(ssi_hw, (SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
				SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
				SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
				SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK)),
		SSI_SFCSR);
	ssi_reg_write(ssi_hw, 0, SSI_SIER);

	SSI_DUMP();
	return 0;
}

static int imx_ssi_hw_tx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;
	u32 stccr, stcr, sier;

	stccr = ssi_reg_read(ssi_hw, SSI_STCCR) & ~SSI_STCCR_WL_MASK;
	stcr = ssi_reg_read(ssi_hw, SSI_STCR);
	sier = ssi_reg_read(ssi_hw, SSI_SIER);

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		stccr |= SSI_STCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		stccr |= SSI_STCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		stccr |= SSI_STCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if (dai->id == IMX_DAI_SSI0 || dai->id == IMX_DAI_SSI2) {
		stcr |= SSI_STCR_TFEN0;
		sier |= SSI_SIER_TUE0_EN;
	} else {
		stcr |= SSI_STCR_TFEN1;
		sier |= SSI_SIER_TUE1_EN;
	}
	sier |= SSI_SIER_TDMAE | SSI_SIER_TIE;

	ssi_reg_write(ssi_hw, stcr, SSI_STCR);
	ssi_reg_write(ssi_hw, stccr, SSI_STCCR);
	ssi_reg_write(ssi_hw, sier, SSI_SIER);
	return 0;
}

static int imx_ssi_hw_rx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;
	u32 srccr, srcr, sier;
	bool sync_mode = ssi_mode->sync_mode;

	srccr = sync_mode ? ssi_reg_read(ssi_hw, SSI_STCCR) :
		ssi_reg_read(ssi_hw, SSI_SRCCR);
	srcr = ssi_reg_read(ssi_hw, SSI_SRCR);
	sier = ssi_reg_read(ssi_hw, SSI_SIER);
	srccr &= ~SSI_SRCCR_WL_MASK;

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		srccr |= SSI_SRCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		srccr |= SSI_SRCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		srccr |= SSI_SRCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if (dai->id == IMX_DAI_SSI0 || dai->id == IMX_DAI_SSI2)
		srcr |= SSI_SRCR_RFEN0;
	else
		srcr |= SSI_SRCR_RFEN1;
	sier |= SSI_SIER_RDMAE | SSI_SIER_RIE | SSI_SIER_ROE0_EN;
	ssi_reg_write(ssi_hw, srcr, SSI_SRCR);

	if (sync_mode)
		ssi_reg_write(ssi_hw, srccr, SSI_STCCR);
	else
		ssi_reg_write(ssi_hw, srccr, SSI_SRCCR);
	ssi_reg_write(ssi_hw, sier, SSI_SIER);
	return 0;
}

/*
 * Should only be called when port is inactive (i.e. SSIEN = 0),
 * although can be called multiple times by upper layers.
 */
static int imx_ssi_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* cant change any parameters when SSI is running */
		if ((ssi_reg_read(ssi_hw, SSI_SCR) & SSI_SCR_SSIEN) &&
			(ssi_reg_read(ssi_hw, SSI_SCR) & SSI_SCR_TE))
			return -EBUSY;
		return imx_ssi_hw_tx_params(substream, params, dai);
	} else {
		/* cant change any parameters when SSI is running */
		if ((ssi_reg_read(ssi_hw, SSI_SCR) & SSI_SCR_SSIEN) &&
			(ssi_reg_read(ssi_hw, SSI_SCR) & SSI_SCR_RE))
			return -EBUSY;
		return imx_ssi_hw_rx_params(substream, params, dai);
	}
}

static int imx_ssi_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;
	u32 scr;

	/* enable the SSI port, note that no other port config
	 * should happen after SSIEN is set */
	ssi_reg_write(ssi_hw, 0, SSI_SACNT);
	scr = ssi_reg_read(ssi_hw, SSI_SCR);
	ssi_reg_write(ssi_hw, (scr | SSI_SCR_SSIEN), SSI_SCR);
	ssi_reg_write(ssi_hw, 0, SSI_SOR);

	SSI_DUMP();
	return 0;
}

static int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;
	u32 scr;

	scr = ssi_reg_read(ssi_hw, SSI_SCR);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (scr & SSI_SCR_RE)
				ssi_reg_write(ssi_hw, 0, SSI_SCR);
			scr |= SSI_SCR_TE;
		} else
			scr |= SSI_SCR_RE;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr &= ~SSI_SCR_TE;
		else
			scr &= ~SSI_SCR_RE;
		break;
	default:
		return -EINVAL;
	}
	ssi_reg_write(ssi_hw, scr, SSI_SCR);

	SSI_DUMP();
	return 0;
}

static void imx_ssi_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;

	/* shutdown SSI if neither Tx or Rx is active */
	if (dai->playback.active || dai->capture.active)
		return;

	if (--ssi_hw->active)
		return;

	ssi_reg_write(ssi_hw, 0, SSI_SCR);

	clk_disable(ssi_hw->clk);
}

#ifdef CONFIG_PM
static int imx_ssi_suspend(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/* do we need to disable any clocks? */
	return 0;
}

static int imx_ssi_resume(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/* do we need to enable any clocks? */
	return 0;
}
#else
#define imx_ssi_suspend	NULL
#define imx_ssi_resume	NULL
#endif

static int fifo_err_counter;

static irqreturn_t ssi_irq(int irq, void *dev_id)
{
	struct imx_ssi_hw *ssi_hw = dev_id;
	u32 sier = ssi_reg_read(ssi_hw, SSI_SIER);
	u32 sisr = ssi_reg_read(ssi_hw, SSI_SISR);

	DBG(0, "%s: IRQ%d: sier=%08x sisr=%08x\n", __FUNCTION__, irq,
		sier, sisr);

	if (sisr & SSI_SISR_TUE0)
		printk(KERN_ERR "transmit fifo0 underrun\n");
	if (sisr & SSI_SISR_TUE1)
		printk(KERN_ERR "transmit fifo1 underrun\n");
	if (sisr & SSI_SISR_ROE0)
		printk(KERN_ERR "receive fifo0 underrun\n");
	if (sisr & SSI_SISR_ROE1)
		printk(KERN_ERR "receive fifo1 underrun\n");

	if (fifo_err_counter++ % 1000 == 0)
		printk(KERN_ERR "ssi1_irq SISR %08x SIER %08x fifo_errs=%d\n",
			ssi_reg_read(ssi_hw, SSI_SISR),
			ssi_reg_read(ssi_hw, SSI_SIER),
		       fifo_err_counter);
//	ssi_reg_write(ssi_hw, (SSI_SIER_TUE0_EN | SSI_SIER_ROE0_EN), SSI_SISR);
	ssi_reg_write(ssi_hw, sier & ~(SSI_SIER_TUE0_EN | SSI_SIER_ROE0_EN),
		SSI_SIER);
	return IRQ_HANDLED;
}

static struct imx_ssi_hw *imx_ssi_get_hw(unsigned int hwport,
					struct platform_device *pdev, int irq,
					struct snd_soc_dai *dai,
					unsigned long hwbase)
{
	int ret;
	struct imx_ssi_hw *ssi_hw;

	if (hwport >= ARRAY_SIZE(imx_ssi_hw))
		return ERR_PTR(-EINVAL);

	ssi_hw = imx_ssi_hw[hwport];
	if (ssi_hw != NULL) {
		ssi_hw->inuse++;
		return ssi_hw;
	}

	ssi_hw = kzalloc(sizeof(struct imx_ssi_hw), GFP_KERNEL);
	if (ssi_hw == NULL) {
		return ERR_PTR(-ENOMEM);
	}
#ifdef CONFIG_MXC_SSI_PORTS
	ret = mxc_ssi_request_port(0, pdev, &ssi_hw->ssi_port);
	if (ret)
		goto free;
	ssi_hw->clk = ssi_hw->ssi_port->ssi_clk;
#else
	DBG(0, "%s: Requesting mem region %08lx..%08lx\n", __FUNCTION__,
		hwbase, hwbase + SSI_IOSIZE - 1);
	if (!request_mem_region(hwbase, SSI_IOSIZE, dai->name)) {
		ret = -EBUSY;
		goto free;
	}
	ssi_hw->clk = clk_get(&pdev->dev, "ssi_clk");
	if (IS_ERR(ssi_hw->clk)) {
		ret = PTR_ERR(ssi_hw->clk);
		dev_dbg(&pdev->dev, "%s: failed to get clock: %d\n",
			__FUNCTION__, ret);
		goto release;
	}
#endif
	ssi_hw->dev = &pdev->dev;
	ssi_hw->irq = irq;
	ssi_hw->membase = hwbase;
	ssi_hw->mapbase = ioremap(hwbase, SSI_IOSIZE);
	if (ssi_hw->mapbase == NULL) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: failed to remap %08lx\n", __FUNCTION__,
			ssi_hw->membase);
		goto clkput;
	}

	ret = request_irq(irq, ssi_irq, 0, dai->name, ssi_hw);
	if (ret) {
		printk(KERN_ERR "%s: failure requesting %s IRQ: %d\n",
			__func__, dai->name, ret);
		goto unmap;
	}
	ssi_hw->inuse++;
	imx_ssi_hw[hwport] = ssi_hw;
	return ssi_hw;

unmap:
	iounmap(ssi_hw->mapbase);
clkput:
#ifdef CONFIG_MXC_SSI_PORTS
	mxc_ssi_release_port(ssi_hw->ssi_port);
#else
	clk_put(ssi_hw->clk);
release:
	release_mem_region(ssi_hw->membase, SSI_IOSIZE);
#endif
free:
	kfree(ssi_hw);
	dev_dbg(&pdev->dev, "%s: failed: %d\n", __FUNCTION__, ret);
	return ERR_PTR(ret);
}

static void imx_ssi_put_hw(struct imx_ssi_hw *ssi_hw)
{
	int i;

	BUG_ON(ssi_hw->inuse <= 0);
	if (--ssi_hw->inuse)
		return;

	free_irq(ssi_hw->irq, ssi_hw);
	iounmap(ssi_hw->mapbase);
#ifdef CONFIG_MXC_SSI_PORTS
	mxc_ssi_release_port(ssi_hw->ssi_port);
#else
	clk_put(ssi_hw->clk);
	DBG(0, "%s: Releasing mem region %08lx..%08lx\n", __FUNCTION__,
		ssi_hw->membase,
		ssi_hw->membase + SSI_IOSIZE - 1);
	release_mem_region(ssi_hw->membase, SSI_IOSIZE);
#endif
	for (i = 0; i < ARRAY_SIZE(imx_ssi_hw); i++) {
		if (imx_ssi_hw[i] == ssi_hw)
			imx_ssi_hw[i] = NULL;
	}
	DBG(0, "%s: Freeing %p\n", __FUNCTION__, ssi_hw);
	kfree(ssi_hw);
}

static int imx_ssi_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{
	int irq;
	struct imx_ssi *ssi_mode;
	struct imx_ssi_hw *ssi_hw;
	unsigned long hwbase;
	int hwport;

	dev_dbg(&pdev->dev, "%s: Probing DAI %s\n", __FUNCTION__, dai->name);
	switch (dai->id) {
	case IMX_DAI_SSI0:
	case IMX_DAI_SSI1:
		irq = MXC_INT_SSI1;
		hwbase = SSI1_BASE_ADDR;
		hwport = 0;
		break;
	case IMX_DAI_SSI2:
	case IMX_DAI_SSI3:
		irq = MXC_INT_SSI2;
		hwbase = SSI2_BASE_ADDR;
		hwport = 0;
		break;
	default:
		dev_err(&pdev->dev, "Invalid DAI id: %d\n", dai->id);
		return -ENODEV;
	}

	ssi_mode = kzalloc(sizeof(struct imx_ssi), GFP_KERNEL);
	if (ssi_mode == NULL) {
		return -ENOMEM;
	}

	ssi_hw = imx_ssi_get_hw(hwport, pdev, irq, dai, hwbase);
	if (IS_ERR(ssi_hw)) {
		kfree(ssi_mode);
		return PTR_ERR(ssi_hw);
	}

	DBG(0, "%s: ssi_mode=%p ssi_hw=%p\n", __FUNCTION__,
		ssi_mode, ssi_hw);
	ssi_mode->hw = ssi_hw;
	dai->private_data = ssi_mode;

	return 0;
}

static void imx_ssi_remove(struct platform_device *pdev,
			   struct snd_soc_dai *dai)
{
	struct imx_ssi *ssi_mode = dai->private_data;
	struct imx_ssi_hw *ssi_hw = ssi_mode->hw;

	DBG(0, "%s: Releasing %p\n", __FUNCTION__, ssi_hw);
	imx_ssi_put_hw(ssi_hw);
	DBG(0, "%s: Freeing %p\n", __FUNCTION__, ssi_mode);
	kfree(ssi_mode);
}

#define IMX_SSI_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
	SNDRV_PCM_RATE_96000)

#define IMX_SSI_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops imx_ssi_dai_ops = {
	.startup = imx_ssi_startup,
	.shutdown = imx_ssi_shutdown,
	.trigger = imx_ssi_trigger,
	.prepare = imx_ssi_prepare,
	.hw_params = imx_ssi_hw_params,
	.set_sysclk = imx_ssi_set_dai_sysclk,
	.set_clkdiv = imx_ssi_set_dai_clkdiv,
	.set_fmt = imx_ssi_set_dai_fmt,
	.set_tdm_slot = imx_ssi_set_dai_tdm_slot,
};

struct snd_soc_dai imx_ssi_dai[] = {
	{
		.name = "imx-ssi-1-0",
		.id = IMX_DAI_SSI0,
		.probe = imx_ssi_probe,
		.suspend = imx_ssi_suspend,
		.remove = imx_ssi_remove,
		.resume = imx_ssi_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.ops = &imx_ssi_dai_ops,
	},
	{
		.name = "imx-ssi-1-1",
		.id = IMX_DAI_SSI1,
		.probe = imx_ssi_probe,
		.suspend = imx_ssi_suspend,
		.remove = imx_ssi_remove,
		.resume = imx_ssi_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.ops = &imx_ssi_dai_ops,
	},
	{
		.name = "imx-ssi-2-0",
		.id = IMX_DAI_SSI2,
		.probe = imx_ssi_probe,
		.suspend = imx_ssi_suspend,
		.remove = imx_ssi_remove,
		.resume = imx_ssi_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.ops = &imx_ssi_dai_ops,
	},
	{
		.name = "imx-ssi-2-1",
		.id = IMX_DAI_SSI3,
		.probe = imx_ssi_probe,
		.suspend = imx_ssi_suspend,
		.remove = imx_ssi_remove,
		.resume = imx_ssi_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = IMX_SSI_RATES,
			.formats = IMX_SSI_FORMATS,
		},
		.ops = &imx_ssi_dai_ops,
	},
};

EXPORT_SYMBOL_GPL(imx_ssi_dai);

static int __init imx_ssi_init(void)
{
	return snd_soc_register_dais(imx_ssi_dai, ARRAY_SIZE(imx_ssi_dai));
}

static void __exit imx_ssi_exit(void)
{
	snd_soc_unregister_dais(imx_ssi_dai, ARRAY_SIZE(imx_ssi_dai));
}

module_init(imx_ssi_init);
module_exit(imx_ssi_exit);
MODULE_AUTHOR
    ("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("i.MX ASoC I2S driver");
MODULE_LICENSE("GPL");
