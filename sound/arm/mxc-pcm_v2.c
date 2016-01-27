/*
 * linux/sound/arm/mxc-pcm_v2.c
 * ALSA PCM interface for the Freescale i.MX chips
 *
 * Author:	Lothar Wassmann
 * Created:	Feb 11, 2008
 * Copyright:	(C) 2008 <LW@KARO-electronics.de>
 *
 * based on: sound/arm/pxa2xx-pcm.c (C) by Nicolas Pitre
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <mach/hardware.h>
#include <mach/dma.h>

#include "mxc-pcm.h"

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

static const struct snd_pcm_hardware mxc_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min		= 1,
	.channels_max		= 2,
	.period_bytes_min	= 32,
	.period_bytes_max	= 1 << 23,
	.periods_min		= 2,
	.periods_max		= SG_MAX_SINGLE_ALLOC,
	.buffer_bytes_max	= 128 * 1024,
	.fifo_size		= 32,
};

struct mxc_runtime_data {
	int dma_ch;
	struct mxc_pcm_dma_params *dma_params;
	struct sg_table sg;
	int periods;
	int period;
};

static int mxc_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct mxc_runtime_data *rtd = substream->runtime->private_data;

	DBG(1, "%s: Disabling DMA channel %d\n", __FUNCTION__, rtd->dma_ch);
	mxc_dma_disable(rtd->dma_ch);
	sg_free_table(&rtd->sg);
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int mxc_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct mxc_pcm_client *client = substream->private_data;
	return client->prepare(substream);
}

static int avg_count = -1;
static unsigned long total_bytes;

static int mxc_pcm_new_period(struct mxc_runtime_data *rtd,
			struct snd_pcm_substream *substream)
{
	int ret;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int dma_size = frames_to_bytes(runtime, runtime->period_size);
	unsigned int offset = dma_size * rtd->period;

	DBG(1, "%s: period pos  ALSA %d DMA %d\n", __FUNCTION__, runtime->periods,
		rtd->period);
	DBG(1, "%s: period size ALSA %ld DMA %d Offset %08x dmasize %08x\n",
		__FUNCTION__, runtime->period_size, runtime->dma_bytes,
		offset, dma_size);
	DBG(1, "%s: DMA addr %08x\n", __FUNCTION__, runtime->dma_addr + offset);
	if (playback) {
		ret = mxc_dma_sg_config(rtd->dma_ch,
					rtd->sg.sgl, rtd->periods,
					0,
					MXC_DMA_MODE_WRITE);
	} else {
		ret = mxc_dma_sg_config(rtd->dma_ch,
					rtd->sg.sgl, rtd->periods,
					0,
					MXC_DMA_MODE_READ);
	}
	if (ret != 0) {
		DBG(0, "%s: Failed to setup DMA: %d\n", __FUNCTION__, ret);
	} else {
		mxc_dma_enable(rtd->dma_ch);
	}
	return ret;
}

static void mxc_pcm_dma_callback(void *arg, int error, unsigned int count)
{
	struct snd_pcm_substream *substream = arg;
	struct mxc_runtime_data *rtd = substream->runtime->private_data;

	if (error) {
		mxc_dma_disable(rtd->dma_ch);
		printk(KERN_ERR "%s: DMA error %d\n", rtd->dma_params->name, error);
		debug = 4;
		snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
	} else {
		struct scatterlist *sg = rtd->sg.sgl;

		DBG(2, "%s: %s period %u/%u sg: %p next %p\n", __FUNCTION__,
			rtd->dma_params->name, rtd->period, rtd->periods, sg,
			sg_next(sg));

		total_bytes += sg_dma_len(sg);
		rtd->period = (rtd->period + 1) % rtd->periods;
		snd_pcm_period_elapsed(substream);
	}
	DBG(3, "%s: Done\n", __FUNCTION__);
}

static int mxc_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period_size = params_period_bytes(params);
	dma_addr_t dma_buff_phys;
	unsigned int i;
	int periods;
	struct scatterlist *sg;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	DBG(1, "%s: dma_addr=%08x size=%u period_bytes=%u periods_max=%u\n", __FUNCTION__,
		runtime->dma_addr, totsize, period_size, runtime->hw.periods_max);
	DBG(1, "%s: period_size=%u\n", __FUNCTION__,
		params_period_size(params));

	runtime->dma_bytes = totsize;

	dma_buff_phys = runtime->dma_addr;
	periods = (totsize + period_size - 1) / period_size;

	ret = sg_alloc_table(&rtd->sg, periods, GFP_KERNEL);
	if (ret) {
		DBG(0, "%s: sg_alloc_table(%d) failed: %d\n",
			__FUNCTION__, periods, ret);
		return ret;
	}
	DBG(1, "%s: sg_table[%d] sg %p next %p last %p\n", __FUNCTION__,
		rtd->sg.nents, rtd->sg.sgl, sg_next(rtd->sg.sgl),
		sg_last(rtd->sg.sgl, rtd->sg.nents));

	for_each_sg(rtd->sg.sgl, sg, periods, i) {
		period_size = totsize > period_size ? period_size : totsize;

		sg_dma_len(sg) = period_size;
		sg_dma_address(sg) = dma_buff_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			DBG(1, "%s: sg[%d]@%p: src=%08x dst=%08x cnt=%08x next %p\n",
				__FUNCTION__, i, sg,
				sg_dma_address(sg),
				rtd->dma_params->dev_addr,
				sg_dma_len(sg),
				sg_next(sg));
		} else {
			DBG(1, "%s: sg[%d]@%p: src=%08x dst=%08x cnt=%08x next %p\n",
				__FUNCTION__, i, sg,
				rtd->dma_params->dev_addr,
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

	rtd->periods = periods;

	return 0;
}

static int mxc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct mxc_pcm_client *client = substream->private_data;
	struct mxc_runtime_data *rtd = substream->runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		avg_count = -1;
		rtd->period = 0;
		DBG(1, "%s: Starting DMA channel %d\n", __FUNCTION__,
			rtd->dma_ch);
		ret = mxc_pcm_new_period(rtd, substream);
		if (ret != 0) {
			return ret;
		}
		ret = client->start_xfer(substream);
		if (ret) {
			DBG(0, "%s: Failed to start SSI transfer\n",
				__FUNCTION__);
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		client->stop_xfer(substream);
		mxc_dma_disable(rtd->dma_ch);
		DBG(1, "%s: Disabling DMA channel %d after %lu bytes\n",
			__FUNCTION__, rtd->dma_ch, total_bytes);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		DBG(1, "%s: Enabling DMA channel %d\n", __FUNCTION__,
			rtd->dma_ch);
		mxc_dma_enable(rtd->dma_ch);
		if (ret != 0) {
			return ret;
		}
		ret = client->start_xfer(substream);
		break;
	default:
		DBG(0, "%s: Bad request: %08x\n", __FUNCTION__, cmd);
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t mxc_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_runtime_data *rtd = runtime->private_data;
	snd_pcm_uframes_t x = rtd->period * runtime->period_size;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	DBG(3, "%s: %s period=%u/%u avail=%lu thresh=%lu\n", __FUNCTION__,
		playback ? "PLAYBACK" : "RECORD",
		rtd->period, rtd->periods, snd_pcm_playback_avail(runtime),
	    runtime->stop_threshold);
	if (x >= runtime->buffer_size) {
		x -= runtime->buffer_size;
	}
	DBG(2, "%s: %s DMA pos=%lu/%lu\n", __FUNCTION__,
		playback ? "PLAYBACK" : "RECORD", x, runtime->buffer_size);

	return x;
}

static int mxc_pcm_open(struct snd_pcm_substream *substream)
{
	struct mxc_pcm_client *client = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct mxc_runtime_data *rtd;
	int ret;

	runtime->hw = mxc_pcm_hardware;
	total_bytes = 0;

	rtd = kzalloc(sizeof(*rtd), GFP_KERNEL);
	if (rtd == NULL) {
		return -ENOMEM;
	}
	rtd->dma_params = playback ? client->playback_params :
		client->capture_params;

	DBG(1, "%s: Requesting DMA channel %d\n", __FUNCTION__,
		rtd->dma_params->channel_id);
	ret = mxc_dma_request(rtd->dma_params->channel_id, rtd->dma_params->name);
	if (ret < 0) {
		goto free_mem;
	}
	rtd->dma_ch = ret;

	ret = mxc_dma_callback_set(rtd->dma_ch,
				mxc_pcm_dma_callback,
				substream);
	if (ret < 0) {
		goto free_dma;
	}

	runtime->private_data = rtd;
	DBG(1, "%s: Starting up %s substream\n", __FUNCTION__,
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
	    "playback" : "capture");
	ret = client->startup(substream);
	if (ret != 0) {
		DBG(0, "%s: startup failed: %d\n", __FUNCTION__, ret);
		goto free_dma;
	}
	return 0;

free_dma:
	mxc_dma_free(rtd->dma_ch);

free_mem:
	kfree(rtd);

	return ret;
}

static int mxc_pcm_close(struct snd_pcm_substream *substream)
{
	struct mxc_pcm_client *client = substream->private_data;
	struct mxc_runtime_data *rtd = substream->runtime->private_data;

	DBG(1, "%s: %lu bytes transferred\n", __FUNCTION__, total_bytes);
	mxc_dma_free(rtd->dma_ch);
	client->shutdown(substream);

	kfree(rtd);
	return 0;
}

static int mxc_pcm_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *vma)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;

	ret = dma_mmap_writecombine(substream->pcm->card->dev, vma,
				runtime->dma_area,
				runtime->dma_addr,
				runtime->dma_bytes);
	if (ret == 0) {
		DBG(1, "%s: dma area mapped to %08x\n", __FUNCTION__,
			runtime->dma_addr);
	} else {
		DBG(0, "%s: Failed to map dma area\n", __FUNCTION__);
	}
	return ret;
}

static struct snd_pcm_ops mxc_pcm_ops = {
	.open		= mxc_pcm_open,
	.close		= mxc_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mxc_pcm_hw_params,
	.hw_free	= mxc_pcm_hw_free,
	.prepare	= mxc_pcm_prepare,
	.trigger	= mxc_pcm_trigger,
	.pointer	= mxc_pcm_pointer,
	.mmap		= mxc_pcm_mmap,
};

static int mxc_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = mxc_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	DBG(1, "%s: Allocating %d byte DMA buffer\n", __FUNCTION__, size);
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area) {
		return -ENOMEM;
	}
	buf->bytes = size;
	return 0;
}

static void mxc_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		DBG(1, "%s: Freeing DMA buffer %08x\n", __FUNCTION__, buf->addr);
		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 mxc_pcm_dmamask = DMA_BIT_MASK(32);

int mxc_pcm_v2_new(struct snd_card *card, struct mxc_pcm_client *client,
		   struct snd_pcm **rpcm)
{
	struct snd_pcm *pcm;
	int play = client->playback_params ? 1 : 0;
	int capt = client->capture_params ? 1 : 0;
	int ret;

	DBG(1, "%s: Creating new PCM channel\n", __FUNCTION__);
	ret = snd_pcm_new(card, "MXC-PCM", 0, play, capt, &pcm);
	if (ret) {
		goto out;
	}
	pcm->private_data = client;
	pcm->private_free = mxc_pcm_free_dma_buffers;

	if (WARN_ON(!card->dev->dma_mask)) {
		card->dev->dma_mask = &mxc_pcm_dmamask;
	}
	if (WARN_ON(!card->dev->coherent_dma_mask)) {
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	}
	if (play) {
		int stream = SNDRV_PCM_STREAM_PLAYBACK;

		snd_pcm_set_ops(pcm, stream, &mxc_pcm_ops);
		ret = mxc_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret)
			goto out;
	}
	if (capt) {
		int stream = SNDRV_PCM_STREAM_CAPTURE;

		snd_pcm_set_ops(pcm, stream, &mxc_pcm_ops);
		ret = mxc_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret) {
			goto out;
		}
	}

	if (rpcm) {
		*rpcm = pcm;
	}
	ret = 0;

 out:
	return ret;
}
EXPORT_SYMBOL(mxc_pcm_v2_new);

MODULE_AUTHOR("Lothar Wassmann");
MODULE_DESCRIPTION("Freescale i.MX PCM DMA module");
MODULE_LICENSE("GPL v2");
