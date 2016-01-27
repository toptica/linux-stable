/*
 * linux/sound/arm/mx27-pcm.c
 * ALSA PCM interface for the Freescale i.MX27 chip
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
#include <mach/dma-mx1-mx2.h>

#include "mx27-pcm.h"

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

static int streaming_dma = 1;
module_param(streaming_dma, bool, S_IRUGO | S_IWUSR);

struct mxc_dma_desc {
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	int num_of_bytes;
};

static const struct snd_pcm_hardware mx27_pcm_hardware = {
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
	.periods_max		= PAGE_SIZE / sizeof(struct mxc_dma_desc),
	.buffer_bytes_max	= 128 * 1024,
	.fifo_size		= 32,
};

struct mx27_runtime_data {
	int dma_ch;
	struct mx27_pcm_dma_params *dma_params;
	struct mxc_dma_desc *dma_desc_array;
	struct sg_table sg;
	int periods;
	int period;
};

static int mx27_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct mx27_runtime_data *rtd = substream->runtime->private_data;

	DBG(0, "%s: Disabling DMA channel %d\n", __FUNCTION__, rtd->dma_ch);
	mxc_dma_disable(rtd->dma_ch);
	sg_free_table(&rtd->sg);
	kfree(rtd->dma_desc_array);
	rtd->dma_desc_array = NULL;
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int mx27_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct mx27_pcm_client *client = substream->private_data;
	return client->prepare(substream);
}

static int avg_count = -1;
static unsigned long total_bytes;

static int mx27_pcm_new_period(struct mx27_runtime_data *rtd,
			struct snd_pcm_substream *substream)
{
	int ret;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	if (rtd->sg.sgl) {
		if (playback) {
			ret = mxc_dma_setup_sg(rtd->dma_ch,
					rtd->sg.sgl, rtd->periods,
					0,
					rtd->dma_params->dev_addr,
					DMA_MODE_WRITE);
		} else {
			ret = mxc_dma_setup_sg(rtd->dma_ch,
					rtd->sg.sgl, rtd->periods,
					0,
					rtd->dma_params->dev_addr,
					DMA_MODE_READ);
		}
	} else {
		struct mxc_dma_desc *dma_desc =
			&rtd->dma_desc_array[rtd->period];

		if (playback) {
			ret = mxc_dma_setup_single(rtd->dma_ch,
						dma_desc->src_addr,
						dma_desc->num_of_bytes,
						dma_desc->dst_addr,
						DMA_MODE_WRITE);
		} else {
			ret = mxc_dma_setup_single(rtd->dma_ch,
						dma_desc->dst_addr,
						dma_desc->num_of_bytes,
						dma_desc->src_addr,
						DMA_MODE_READ);
		}
	}
	if (ret != 0) {
		DBG(0, "%s: Failed to setup DMA\n", __FUNCTION__);
	} else {
		mxc_dma_enable(rtd->dma_ch);
	}
	return ret;
}

static void mx27_pcm_sg_callback(int dma_ch, void *arg, struct scatterlist *sg)
{
	struct snd_pcm_substream *substream = arg;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mx27_runtime_data *rtd = runtime->private_data;

	DBG(2, "%s: %s period %u/%u sg: %p next %p\n", __FUNCTION__,
		rtd->dma_params->name, rtd->period, rtd->periods, sg,
		sg_next(sg));

	total_bytes += sg_dma_len(sg);
	rtd->period = (rtd->period + 1) % rtd->periods;
	snd_pcm_period_elapsed(substream);
	DBG(3, "%s: Done\n", __FUNCTION__);
}

static void mx27_pcm_dma_callback(int dma_ch, void *arg)
{
	int ret;
	struct snd_pcm_substream *substream = arg;
	struct mx27_runtime_data *rtd = substream->runtime->private_data;

	DBG(2, "%s: %s period %u/%u streaming: %d\n", __FUNCTION__,
		rtd->dma_params->name, rtd->period, rtd->periods,
		rtd->sg.sgl != NULL);

	if (rtd->dma_desc_array) {
		total_bytes += rtd->dma_desc_array[rtd->period].num_of_bytes;
		rtd->period = (rtd->period + 1) % rtd->periods;
		snd_pcm_period_elapsed(substream);
		ret = mx27_pcm_new_period(rtd, substream);
		if (ret != 0) {
			snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
		}
	}
	DBG(3, "%s: Done\n", __FUNCTION__);
}

static void mx27_pcm_error_callback(int dma_ch, void *arg, int error)
{
	struct snd_pcm_substream *substream = arg;
	struct mx27_runtime_data *rtd = substream->runtime->private_data;

	mxc_dma_disable(rtd->dma_ch);
	printk(KERN_ERR "%s: DMA error %d\n", rtd->dma_params->name, error);
	debug = 4;
	snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
}

static int mx27_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mx27_runtime_data *rtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	dma_addr_t dma_buff_phys;
	unsigned int i;
	int periods;

	DBG(0, "%s: size=%u period_bytes=%u periods_max=%u\n", __FUNCTION__,
	    totsize, period, runtime->hw.periods_max);
	DBG(0, "%s: period_size=%u\n", __FUNCTION__,
		params_period_size(params));

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	dma_buff_phys = runtime->dma_addr;
	periods = (totsize + period - 1) / period;
	if (streaming_dma) {
		struct scatterlist *sg;

		/* allocate an sg table with one additional entry
		 * for linking the last sg back to the first
		 */
		ret = sg_alloc_table(&rtd->sg, periods + 1, GFP_KERNEL);
		if (ret) {
			DBG(0, "%s: sg_alloc_table(%d) failed: %d\n",
				__FUNCTION__, periods + 1, ret);
			return ret;
		}
		DBG(0, "%s: sg_table[%d] sg %p next %p last %p\n", __FUNCTION__,
			rtd->sg.nents, rtd->sg.sgl, sg_next(rtd->sg.sgl),
			sg_last(rtd->sg.sgl, rtd->sg.nents));

		for_each_sg(rtd->sg.sgl, sg, periods, i) {
			period = totsize > period ? period : totsize;

			sg_dma_len(sg) = period;
			sg_dma_address(sg) = dma_buff_phys;
			dma_buff_phys += period;
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				DBG(0, "%s: sg[%d]@%p: src=%08x dst=%08x cnt=%08x next %p\n",
					__FUNCTION__, i, sg,
					sg_dma_address(sg),
					rtd->dma_params->dev_addr,
					sg_dma_len(sg),
					sg_next(sg));
			} else {
				DBG(0, "%s: sg[%d]@%p: src=%08x dst=%08x cnt=%08x next %p\n",
					__FUNCTION__, i, sg,
					rtd->dma_params->dev_addr,
					sg_dma_address(sg),
					sg_dma_len(sg),
					sg_next(sg));
			}
			totsize -= period;
			BUG_ON(periods > runtime->hw.periods_max);
			WARN_ON(totsize == 0 && i != periods - 1);
		}
		BUG_ON(sg == 0);
		WARN_ON(totsize != 0);
		for_each_sg(rtd->sg.sgl, sg, periods, i) {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				DBG(0, "%s: sg[%d]@%p: src=%08x dst=%08x cnt=%08x next %p\n",
					__FUNCTION__, i, sg,
					sg_dma_address(sg),
					rtd->dma_params->dev_addr,
					sg_dma_len(sg),
					sg_next(sg));
			} else {
				DBG(0, "%s: sg[%d]@%p: src=%08x dst=%08x cnt=%08x next %p\n",
					__FUNCTION__, i, sg,
					rtd->dma_params->dev_addr,
					sg_dma_address(sg),
					sg_dma_len(sg),
					sg_next(sg));
			}
			if (sg_next(sg) == rtd->sg.sgl) {
				DBG(0, "%s: sg %p points back to %p\n",
					__FUNCTION__, sg, rtd->sg.sgl);
			}
		}
		BUG_ON(sg == 0);
	} else {
		struct mxc_dma_desc *dma_desc;

		dma_desc = kzalloc(periods * sizeof(struct mxc_dma_desc),
				GFP_KERNEL);
		if (dma_desc == NULL) {
			return -ENOMEM;
		}
		rtd->dma_desc_array = dma_desc;
		for (i = 0; i < periods; i++) {
			period = totsize > period ? period : totsize;

			dma_desc[i].num_of_bytes = period;
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				dma_desc[i].src_addr = dma_buff_phys;
				dma_desc[i].dst_addr =
					rtd->dma_params->dev_addr;
			} else {
				dma_desc[i].src_addr =
					rtd->dma_params->dev_addr;
				dma_desc[i].dst_addr = dma_buff_phys;
			}

			DBG(0, "%s: desc[%d]: src=%08x dst=%08x cnt=%08x\n",
				__FUNCTION__, i,
				dma_desc[i].src_addr,
				dma_desc[i].dst_addr,
				dma_desc[i].num_of_bytes);

			dma_buff_phys += period;
			totsize -= period;
			WARN_ON(totsize == 0 && i != periods - 1);
		}
		WARN_ON(totsize != 0);
	}
	rtd->periods = periods;

	return 0;
}

static int mx27_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct mx27_pcm_client *client = substream->private_data;
	struct mx27_runtime_data *rtd = substream->runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		avg_count = -1;
		rtd->period = 0;
		DBG(0, "%s: Starting DMA channel %d\n", __FUNCTION__,
			rtd->dma_ch);
		ret = mx27_pcm_new_period(rtd, substream);
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
		DBG(0, "%s: Enabling DMA channel %d\n", __FUNCTION__,
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

static snd_pcm_uframes_t mx27_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mx27_runtime_data *rtd = runtime->private_data;
	snd_pcm_uframes_t x = rtd->period * runtime->period_size;

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

static int mx27_pcm_open(struct snd_pcm_substream *substream)
{
	struct mx27_pcm_client *client = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct mx27_runtime_data *rtd;
	int ret;

	DBG(0, "%s: \n", __FUNCTION__);
	runtime->hw = mx27_pcm_hardware;
	total_bytes = 0;

	rtd = kzalloc(sizeof(*rtd), GFP_KERNEL);
	if (rtd == NULL) {
		return -ENOMEM;
	}
	rtd->dma_params = playback ? client->playback_params :
		client->capture_params;

	DBG(0, "%s: Requesting DMA channel %d\n", __FUNCTION__,
		rtd->dma_params->channel_id);
	ret = mxc_dma_request_by_prio(rtd->dma_params->name, DMA_PRIO_HIGH);
	if (ret < 0) {
		goto free_mem;
	}
	rtd->dma_ch = ret;
	mxc_dma_config_burstlen(rtd->dma_ch, playback ?
				TXFIFO_WATERMARK :
				RXFIFO_WATERMARK);
	ret = mxc_dma_config_channel(rtd->dma_ch,
				MXC_DMA_MEMSIZE_16 | MXC_DMA_TYPE_FIFO,
				MXC_DMA_MEMSIZE_32 | MXC_DMA_TYPE_LINEAR,
				rtd->dma_params->channel_id,
				streaming_dma);
	if (ret < 0) {
		goto free_dma;
	}

	ret = mxc_dma_setup_handlers(rtd->dma_ch,
				mx27_pcm_dma_callback,
				mx27_pcm_error_callback,
				substream);
	if (ret < 0) {
		goto free_dma;
	}

	ret = mxc_dma_setup_progression_handler(rtd->dma_ch,
						mx27_pcm_sg_callback);
	if (ret < 0) {
		DBG(0, "%s: Failed to setup DMA progression handler: %d\n",
			__FUNCTION__, ret);
		return ret;
	}

	runtime->private_data = rtd;
	DBG(0, "%s: Starting up %s substream\n", __FUNCTION__,
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
	    "playback" : "capture");
	ret = client->startup(substream);
	if (ret != 0) {
		DBG(0, "%s: startup failed: %d\n", __FUNCTION__, ret);
		goto free_dma;
	}
	return 0;

free_dma:
	DBG(0, "%s: Freeing DMA channel %d\n", __FUNCTION__, rtd->dma_ch);
	mxc_dma_free(rtd->dma_ch);

free_mem:
	DBG(0, "%s: Freeing runtime data\n", __FUNCTION__);
	kfree(rtd);

	return ret;
}

static int mx27_pcm_close(struct snd_pcm_substream *substream)
{
	struct mx27_pcm_client *client = substream->private_data;
	struct mx27_runtime_data *rtd = substream->runtime->private_data;

	DBG(0, "%s: %lu bytes transferred\n", __FUNCTION__, total_bytes);
	DBG(0, "%s: Freeing DMA channel %d\n", __FUNCTION__, rtd->dma_ch);
	mxc_dma_free(rtd->dma_ch);
	client->shutdown(substream);

	kfree(rtd);
	return 0;
}

static int mx27_pcm_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *vma)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;

	ret = dma_mmap_writecombine(substream->pcm->card->dev, vma,
				runtime->dma_area,
				runtime->dma_addr,
				runtime->dma_bytes);
	if (ret == 0) {
		DBG(0, "%s: dma area mapped to %08x\n", __FUNCTION__,
			runtime->dma_addr);
	} else {
		DBG(0, "%s: Failed to map dma area\n", __FUNCTION__);
	}
	return ret;
}

static struct snd_pcm_ops mx27_pcm_ops = {
	.open		= mx27_pcm_open,
	.close		= mx27_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mx27_pcm_hw_params,
	.hw_free	= mx27_pcm_hw_free,
	.prepare	= mx27_pcm_prepare,
	.trigger	= mx27_pcm_trigger,
	.pointer	= mx27_pcm_pointer,
	.mmap		= mx27_pcm_mmap,
};

static int mx27_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = mx27_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	DBG(0, "%s: Allocating %d byte DMA buffer\n", __FUNCTION__, size);
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area) {
		return -ENOMEM;
	}
	buf->bytes = size;
	return 0;
}

static void mx27_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	DBG(0, "%s: \n", __FUNCTION__);
	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		DBG(0, "%s: Freeing DMA buffer %08x\n", __FUNCTION__, buf->addr);
		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 mx27_pcm_dmamask = DMA_BIT_MASK(32);

int mx27_pcm_new(struct snd_card *card, struct mx27_pcm_client *client,
		 struct snd_pcm **rpcm)
{
	struct snd_pcm *pcm;
	int play = client->playback_params ? 1 : 0;
	int capt = client->capture_params ? 1 : 0;
	int ret;

	DBG(0, "%s: Creating new PCM channel\n", __FUNCTION__);
	ret = snd_pcm_new(card, "MX27-PCM", 0, play, capt, &pcm);
	if (ret) {
		goto out;
	}
	pcm->private_data = client;
	pcm->private_free = mx27_pcm_free_dma_buffers;

	if (WARN_ON(!card->dev->dma_mask)) {
		card->dev->dma_mask = &mx27_pcm_dmamask;
	}
	if (WARN_ON(!card->dev->coherent_dma_mask)) {
		card->dev->coherent_dma_mask = 0xffffffff;
	}
	if (play) {
		int stream = SNDRV_PCM_STREAM_PLAYBACK;

		snd_pcm_set_ops(pcm, stream, &mx27_pcm_ops);
		ret = mx27_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret)
			goto out;
	}
	if (capt) {
		int stream = SNDRV_PCM_STREAM_CAPTURE;

		snd_pcm_set_ops(pcm, stream, &mx27_pcm_ops);
		ret = mx27_pcm_preallocate_dma_buffer(pcm, stream);
		if (ret) {
			goto out;
		}
	}

	if (rpcm) {
		*rpcm = pcm;
	}
	ret = 0;

 out:
	DBG(0, "%s: Returning %d\n", __FUNCTION__, ret);
	return ret;
}
EXPORT_SYMBOL(mx27_pcm_new);

MODULE_AUTHOR("Lothar Wassmann");
MODULE_DESCRIPTION("Freescale i.MX27 PCM DMA module");
MODULE_LICENSE("GPL v2");
