/*
 * imx-pcm.c -- ALSA SoC interface for the Freescale i.MX3 CPUs
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Based on imx31-pcm.c by Nicolas Pitre, (C) 2004 MontaVista Software, Inc.
 * and on mxc-alsa-mc13783 (C) 2006-2009 Freescale.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/mxc_audio.h>
#include <mach/clock.h>
#include <mach/hardware.h>

#include "imx-pcm.h"
#include "imx-ssi.h"
#include "imx-esai.h"

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
#include <linux/delay.h>
#include <linux/mxc_asrc.h>
#endif

#ifdef CONFIG_SND_MXC_SOC_IRAM
#define USE_IRAM	1
#else
#define USE_IRAM	0
#endif

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

static const struct snd_pcm_hardware imx_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	.buffer_bytes_max = SND_IRAM_SIZE,
	.period_bytes_max = SND_IRAM_SIZE / 4,
#else
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_max = 1 << 23,
#endif
	.period_bytes_min = 2 * SZ_1K,
	.periods_min = 2,
	.periods_max = 255,
	.fifo_size = 8,
};

#ifdef CONFIG_SND_MXC_SOC_IRAM
static uint32_t audio_iram_phys_base_addr;
static void *audio_iram_virt_base_addr;

static struct vm_operations_struct snd_mxc_audio_playback_vm_ops = {
	.open = snd_pcm_mmap_data_open,
	.close = snd_pcm_mmap_data_close,
};

/*
	enable user space access to iram buffer
*/
static int imx_iram_audio_playback_mmap(struct snd_pcm_substream *substream,
					struct vm_area_struct *area)
{
	unsigned long off;
	unsigned long phys;
	unsigned long size;
	int ret = 0;

	area->vm_ops = &snd_mxc_audio_playback_vm_ops;
	area->vm_private_data = substream;

	off = area->vm_pgoff << PAGE_SHIFT;
	phys = audio_iram_phys_base_addr + off;
	size = area->vm_end - area->vm_start;

	if (off + size > SND_IRAM_SIZE)
		return -EINVAL;

	area->vm_page_prot = pgprot_writecombine(area->vm_page_prot);
	area->vm_flags |= VM_IO;
	ret =
	    remap_pfn_range(area, area->vm_start, phys >> PAGE_SHIFT,
			    size, area->vm_page_prot);
	if (ret == 0)
		area->vm_ops->open(area);

	return ret;
}

/*
     Map nbytes in virtual space
     bytes -audio iram iram partition size
     phys_addr - physical address of iram buffer
     returns - virtual address of the iram buffer or NULL if fail
*/
static void *imx_iram_init(dma_addr_t *phys_addr, size_t bytes)
{
	void __iomem *iram_base;

	iram_base = ioremap(SND_IRAM_BASE_ADDR, bytes);

	audio_iram_virt_base_addr = iram_base;
	audio_iram_phys_base_addr = SND_IRAM_BASE_ADDR;
	*phys_addr = SND_IRAM_BASE_ADDR;

	return iram_base ? iram_base : -ENOMEM;
}

/*
     destroy the virtual mapping of the iram buffer
*/

static void imx_iram_free(void)
{
	iounmap(audio_iram_virt_base_addr);
}
#else
static int imx_iram_audio_playback_mmap(struct snd_pcm_substream *substream,
					struct vm_area_struct *area)
{
	return -EOPNOTSUPP;
}

static void *imx_iram_init(dma_addr_t *phys_addr, size_t bytes)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static void imx_iram_free(void)
{
}
#endif

static int imx_get_sdma_transfer(int format, int dai_port,
				struct snd_pcm_substream *substream)
{
	int transfer = -EINVAL;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;

	if (rtd->asrc_enable) {
		if (dai_port == IMX_DAI_SSI0) {
			if (rtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI1_TX0;
			else if (rtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI1_TX0;
		} else if (dai_port == IMX_DAI_SSI1) {
			if (rtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI1_TX1;
			else if (rtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI1_TX1;
		} else if (dai_port == IMX_DAI_SSI2) {
			if (rtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI2_TX0;
			else if (rtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI2_TX0;
		} else if (dai_port == IMX_DAI_SSI3) {
			if (rtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_SSI2_TX1;
			else if (rtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_SSI2_TX1;
		} else if (dai_port & IMX_DAI_ESAI_TX) {
			if (rtd->asrc_index == 0)
				transfer = MXC_DMA_ASRCA_ESAI;
			else if (rtd->asrc_index == 1)
				transfer = MXC_DMA_ASRCB_ESAI;
			else
				transfer = MXC_DMA_ASRCC_ESAI;
		}
	} else
#endif
		if (dai_port == IMX_DAI_SSI0) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_TX0;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_RX0;
			}
		} else if (dai_port == IMX_DAI_SSI1) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_TX1;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI1_16BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI1_24BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI1_24BIT_RX1;
			}
		} else if (dai_port == IMX_DAI_SSI2) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_TX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_TX0;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_RX0;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_RX0;
			}
		} else if (dai_port == IMX_DAI_SSI3) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_TX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_TX1;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_SSI2_16BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_SSI2_24BIT_RX1;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_SSI2_24BIT_RX1;
			}
		} else if ((dai_port & IMX_DAI_ESAI_TX) ||
			(dai_port & IMX_DAI_ESAI_RX)) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_ESAI_16BIT_TX;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_ESAI_24BIT_TX;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_ESAI_24BIT_TX;
			} else {
				if (format == SNDRV_PCM_FORMAT_S16_LE)
					transfer = MXC_DMA_ESAI_16BIT_RX;
				else if (format == SNDRV_PCM_FORMAT_S24_LE)
					transfer = MXC_DMA_ESAI_24BIT_RX;
				else if (format == SNDRV_PCM_FORMAT_S20_3LE)
					transfer = MXC_DMA_ESAI_24BIT_RX;
			}
		}

	return transfer;
}

static int dma_new_period(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;
	unsigned int dma_size = frames_to_bytes(runtime, runtime->period_size);
	unsigned int offset = dma_size * rtd->period;
	int ret = 0;
	mxc_dma_requestbuf_t sdma_request;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	DBG(1, "%s: \n", __FUNCTION__);

	if (!rtd->active)
		return 0;

	memset(&sdma_request, 0, sizeof(mxc_dma_requestbuf_t));

	DBG(1, "%s: period pos  ALSA %d DMA %d MAX %d\n", __FUNCTION__,
		runtime->periods, rtd->period, rtd->periods);
	DBG(1, "%s: period size ALSA %ld DMA %d Offset %08x dmasize %08x\n",
		__FUNCTION__, runtime->period_size, runtime->dma_bytes,
		offset, dma_size);
	DBG(1, "%s: DMA addr %08x\n", __FUNCTION__, runtime->dma_addr + offset);
#if 1
	if (playback) {
		ret = mxc_dma_sg_config(rtd->dma_wchannel,
					rtd->sg.sgl, rtd->periods,
					MXC_DMA_SIZE_UNLIMITED,
					MXC_DMA_MODE_WRITE);
	} else {
		ret = mxc_dma_sg_config(rtd->dma_wchannel,
					rtd->sg.sgl, rtd->periods,
					MXC_DMA_SIZE_UNLIMITED,
					MXC_DMA_MODE_READ);
	}
	if (ret != 0) {
		DBG(0, "%s: Failed to setup DMA: %d\n", __FUNCTION__, ret);
		return ret;
	}
	mxc_dma_enable(rtd->dma_wchannel);
	rtd->dma_active = 1;
#else
	if (playback)
		sdma_request.src_addr = runtime->dma_addr + offset;
	else
		sdma_request.dst_addr = runtime->dma_addr + offset;

	sdma_request.num_of_bytes = dma_size;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mxc_dma_config(rtd->dma_wchannel,
			       &sdma_request, 1, MXC_DMA_MODE_WRITE);
		ret = mxc_dma_enable(rtd->dma_wchannel);
	} else {
		mxc_dma_config(rtd->dma_wchannel,
			       &sdma_request, 1, MXC_DMA_MODE_READ);
		ret = mxc_dma_enable(rtd->dma_wchannel);
	}
	rtd->dma_active = 1;
	rtd->period++;
	rtd->period %= runtime->periods;
#endif
	return ret;
}

static void audio_dma_irq(void *data, int err, unsigned int count)
{
	struct snd_pcm_substream *substream = data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;

	if (err) {
		mxc_dma_disable(rtd->dma_wchannel);
		rtd->dma_active = 0;
		pr_err("%s: DMA ERROR %d after %u byte\n", __FUNCTION__,
			err, count);
		snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
	} else {
		struct scatterlist *sg = rtd->sg.sgl;
		size_t total_bytes;

		DBG(1, "%s: channel %u period %u/%u sg: %p next %p\n", __FUNCTION__,
			rtd->dma_wchannel, rtd->period, rtd->periods, sg,
			sg_next(sg));

		total_bytes += sg_dma_len(sg);
		rtd->period = (rtd->period + 1) % rtd->periods;
		snd_pcm_period_elapsed(substream);
	}
}

static int imx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;
	int ret, channel;

	if (rtd->dma_alloc) {
		mxc_dma_free(rtd->dma_wchannel);
		rtd->dma_alloc = 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
		if (rtd->asrc_enable) {
			struct dma_channel_info info;
			mxc_dma_requestbuf_t sdma_request;

			info.asrc.channs = runtime->channels;
			if (rtd->dma_asrc >= 0) {
				mxc_dma_free(rtd->dma_asrc);
			}
			memset(&sdma_request, 0, sizeof(mxc_dma_requestbuf_t));
			/* num_of_bytes can be set any value except for zero */
			sdma_request.num_of_bytes = 0x40;
			channel = mxc_dma_request_ext(rtd->dma_ch,
						"ALSA TX SDMA", &info);

			mxc_dma_config(channel, &sdma_request,
				       1, MXC_DMA_MODE_WRITE);
			rtd->dma_asrc = channel;

			if (rtd->asrc_index == 0)
				rtd->dma_ch = MXC_DMA_ASRC_A_RX;
			else if (rtd->asrc_index == 1)
				rtd->dma_ch = MXC_DMA_ASRC_B_RX;
			else
				rtd->dma_ch = MXC_DMA_ASRC_C_RX;

			channel = mxc_dma_request(rtd->dma_ch,
						"ALSA ASRC RX");
		} else
			channel = mxc_dma_request(rtd->dma_ch,
						"ALSA TX SDMA");
#else
		DBG(0, "%s: Requesting SDMA id %d\n", __FUNCTION__,
			rtd->dma_ch);
		channel = mxc_dma_request(rtd->dma_ch, "ALSA TX SDMA");
#endif
		if (channel < 0) {
			pr_err("imx-pcm: error requesting a write dma channel\n");
			return channel;
		}
	} else {
		DBG(0, "%s: Requesting DMA channel %d\n", __FUNCTION__,
			rtd->dma_ch);
		channel = mxc_dma_request(rtd->dma_ch, "ALSA RX SDMA");
		if (channel < 0) {
			pr_err("imx-pcm: error requesting a read dma channel\n");
			return channel;
		}
	}
	ret = mxc_dma_callback_set(channel, audio_dma_irq, substream);
	if (ret) {
		pr_err("imx-pcm: failed to set DMA callback for channel %d\n",
			channel);
		mxc_dma_free(channel);
		return ret;
	}
	rtd->dma_wchannel = channel;
	rtd->dma_alloc = 1;

	rtd->period = 0;
	return 0;
}

static int imx_pcm_hw_params(struct snd_pcm_substream
			     *substream, struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period_size = params_period_bytes(params);
	dma_addr_t dma_buff_phys;
	unsigned int i;
	int periods;
	struct scatterlist *sg;

	rtd->dma_ch = imx_get_sdma_transfer(params_format(params),
					srtd->dai->cpu_dai->id, substream);

	if (rtd->dma_ch < 0) {
		printk(KERN_ERR "imx-pcm: invalid sdma transfer type\n");
		return rtd->dma_ch;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	DBG(0, "%s: dma_addr=%08x size=%u period_bytes=%u periods_max=%u\n", __FUNCTION__,
		runtime->dma_addr, totsize, period_size, runtime->hw.periods_max);
	DBG(0, "%s: period_size=%u\n", __FUNCTION__,
		params_period_size(params));

	dma_buff_phys = runtime->dma_addr;

	runtime->dma_bytes = totsize;
	periods = (totsize + period_size - 1) / period_size;

	if (cpu_is_mx2())
		ret = sg_alloc_table(&rtd->sg, periods + 1, GFP_KERNEL);
	else
		ret = sg_alloc_table(&rtd->sg, periods, GFP_KERNEL);
	if (ret) {
		DBG(0, "%s: sg_alloc_table(%d) failed: %d\n",
			__FUNCTION__, periods, ret);
		return ret;
	}
	DBG(0, "%s: sg_table[%d] sg %p next %p last %p\n", __FUNCTION__,
		rtd->sg.nents, rtd->sg.sgl, sg_next(rtd->sg.sgl),
		sg_last(rtd->sg.sgl, rtd->sg.nents));

	for_each_sg(rtd->sg.sgl, sg, periods, i) {
		period_size = totsize > period_size ? period_size : totsize;

		sg_dma_len(sg) = period_size;
		sg_dma_address(sg) = dma_buff_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			DBG(0, "%s: sg[%d]@%p: src=%08x cnt=%08x next %p\n",
				__FUNCTION__, i, sg,
				sg_dma_address(sg),
				sg_dma_len(sg),
				sg_next(sg));
		} else {
			DBG(0, "%s: sg[%d]@%p: dst=%08x cnt=%08x next %p\n",
				__FUNCTION__, i, sg,
				sg_dma_address(sg),
				sg_dma_len(sg),
				sg_next(sg));
		}
		dma_buff_phys += period_size;
		totsize -= period_size;
		BUG_ON(periods > runtime->hw.periods_max);
		WARN_ON(totsize == 0 && i != periods - 1);
	}
	WARN_ON(totsize != 0);
	if (cpu_is_mx2()) {
		BUG_ON(sg == NULL);
		sg_chain(rtd->sg.sgl, rtd->sg.nents, rtd->sg.sgl);
	}
	rtd->periods = periods;
	return 0;
}

static int imx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;

	if (rtd->dma_alloc) {
		mxc_dma_free(rtd->dma_wchannel);
		rtd->dma_alloc = 0;
	}
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	if (rtd->asrc_enable && rtd->dma_asrc >= 0) {
		mxc_dma_free(rtd->dma_asrc);
		rtd->dma_asrc = -1;
	}
#endif
	return 0;
}

static int imx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mxc_runtime_data *rtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rtd->active = 1;
		ret = dma_new_period(substream);
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
		if (rtd->asrc_enable) {
			ret = mxc_dma_enable(rtd->dma_asrc);
			asrc_start_conv(rtd->asrc_index);
			/* There is underrun, if immediately enable SSI after
			   start ASRC */
			mdelay(1);
		}
#endif
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		rtd->active = 0;
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
		if (rtd->asrc_enable) {
			mxc_dma_disable(rtd->dma_asrc);
			asrc_stop_conv(rtd->asrc_index);
		}
#endif
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t imx_pcm_pointer(struct
					 snd_pcm_substream
					 *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;
	unsigned int offset = runtime->period_size * rtd->period;

	if (offset >= runtime->buffer_size)
		offset = 0;

	DBG(1, "%s: pointer offset %08x\n", __FUNCTION__, offset);
	return offset;
}

static int imx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &imx_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	rtd = kzalloc(sizeof(struct mxc_runtime_data), GFP_KERNEL);
	if (rtd == NULL)
		return -ENOMEM;

	runtime->private_data = rtd;
	return 0;
}

static int imx_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;

	if (rtd->dma_alloc) {
		mxc_dma_free(rtd->dma_wchannel);
		rtd->dma_alloc = 0;
	}

	kfree(rtd);
	return 0;
}

static int
imx_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct snd_soc_device *socdev = srtd->socdev;
	struct snd_soc_dai *cpu_dai = socdev->card->dai_link->cpu_dai;
	struct mxc_audio_platform_data *dev_data;
	int ext_ram = 0;
	int ret;

	DBG(0, "%s: UseIram=%d dma_addr=%08x dma_area=%p dma_bytes=%d\n",
		__func__, USE_IRAM, runtime->dma_addr,
		runtime->dma_area, runtime->dma_bytes);

	if (cpu_dai->dev && cpu_dai->dev->platform_data) {
		dev_data = cpu_dai->dev->platform_data;
		ext_ram = dev_data->ext_ram;
	}

	if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ||
		ext_ram || !USE_IRAM) {
		ret = dma_mmap_writecombine(substream->pcm->card->dev, vma,
					runtime->dma_area,
					runtime->dma_addr,
					runtime->dma_bytes);
		DBG(0, "%s: dma area mapped to %08x\n", __FUNCTION__,
			runtime->dma_addr);
		return ret;
	} else
		return imx_iram_audio_playback_mmap(substream, vma);
}

struct snd_pcm_ops imx_pcm_ops = {
	.open = imx_pcm_open,
	.close = imx_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = imx_pcm_hw_params,
	.hw_free = imx_pcm_hw_free,
	.prepare = imx_pcm_prepare,
	.trigger = imx_pcm_trigger,
	.pointer = imx_pcm_pointer,
	.mmap = imx_pcm_mmap,
};

static int imx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct snd_soc_pcm_runtime *srtd = pcm->private_data;
	struct snd_soc_device *socdev = srtd->socdev;
	struct snd_soc_dai *cpu_dai = socdev->card->dai_link->cpu_dai;
	struct mxc_audio_platform_data *dev_data;
	int ext_ram = 0;
	size_t size = imx_pcm_hardware.buffer_bytes_max;

	if (cpu_dai->dev && cpu_dai->dev->platform_data) {
		dev_data = cpu_dai->dev->platform_data;
		ext_ram = dev_data->ext_ram;
	}

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	if ((stream == SNDRV_PCM_STREAM_CAPTURE) || ext_ram || !USE_IRAM)
		buf->area = dma_alloc_writecombine(pcm->card->dev, size,
						&buf->addr, GFP_KERNEL);
	else
		buf->area = imx_iram_init(&buf->addr, size);

	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	printk(KERN_INFO "DMA Sound Buffers Allocated:"
	       "UseIram=%d buf->addr=%08x buf->area=%p size=%d\n",
	       USE_IRAM, buf->addr, buf->area, size);
	return 0;
}

static void imx_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	struct snd_soc_pcm_runtime *srtd = pcm->private_data;
	struct snd_soc_device *socdev = srtd->socdev;
	struct snd_soc_dai *cpu_dai = socdev->card->dai_link->cpu_dai;
	struct mxc_audio_platform_data *dev_data;
	int ext_ram = 0;
	int stream;

	if (cpu_dai->dev && cpu_dai->dev->platform_data) {
		dev_data = cpu_dai->dev->platform_data;
		ext_ram = dev_data->ext_ram;
	}

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		if ((stream == SNDRV_PCM_STREAM_CAPTURE) ||
			ext_ram || !USE_IRAM)
			dma_free_writecombine(pcm->card->dev,
					buf->bytes, buf->area, buf->addr);
		else
			imx_iram_free();
		buf->area = NULL;
	}
}

static u64 imx_pcm_dmamask = DMA_BIT_MASK(32);

static int imx_pcm_new(struct snd_card *card,
		       struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &imx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->playback.channels_min) {
		ret = imx_pcm_preallocate_dma_buffer(pcm,
						     SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = imx_pcm_preallocate_dma_buffer(pcm,
						     SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
	return 0;

out:
	DBG(0, "%s failed with %d\n", __func__, ret);
	return ret;
}

struct snd_soc_platform imx_soc_platform = {
	.name = "imx-audio",
	.pcm_ops = &imx_pcm_ops,
	.pcm_new = imx_pcm_new,
	.pcm_free = imx_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(imx_soc_platform);

static int __init imx_pcm_init(void)
{
	int ret;

	DBG(0, "%s: Registering %s\n", __FUNCTION__, imx_soc_platform.name);
	ret = snd_soc_register_platform(&imx_soc_platform);
	DBG(0, "%s: snd_soc_register_platform returned %d\n", __FUNCTION__, ret);
	return ret;
}
module_init(imx_pcm_init);

static void __exit imx_pcm_exit(void)
{
	snd_soc_unregister_platform(&imx_soc_platform);
}
module_exit(imx_pcm_exit);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("Freescale i.MX3x PCM DMA module");
MODULE_LICENSE("GPL");
