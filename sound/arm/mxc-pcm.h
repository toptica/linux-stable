/*
 * linux/sound/arm/mx25-pcm.h
 * ALSA PCM interface for the Freescale i.MX25 chip
 *
 * Author:	Lothar Wassmann
 * Created:	Feb 11, 2008
 * Copyright:	<LW@KARO-electronics.de>
 *
 * based on: sound/arm/pxa2xx-pcm.h by Nicolas Pitre
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define TXFIFO_WATERMARK	8
#define RXFIFO_WATERMARK	8


struct mxc_pcm_dma_params {
	char *name;			/* stream identifier */
	int channel_id;			/* DMA channel id */
	u32 dev_addr;			/* device physical address for DMA */
};

struct mxc_pcm_client {
	struct mxc_pcm_dma_params *playback_params;
	struct mxc_pcm_dma_params *capture_params;
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
	int (*start_xfer)(struct snd_pcm_substream *);
	void (*stop_xfer)(struct snd_pcm_substream *);
};

extern int mx27_pcm_new(struct snd_card *card, struct mxc_pcm_client *client,
			struct snd_pcm **rpcm);
extern int mxc_pcm_v2_new(struct snd_card *card, struct mxc_pcm_client *client,
			struct snd_pcm **rpcm);
static inline int mxc_pcm_new(struct snd_card *card, struct mxc_pcm_client *client,
			struct snd_pcm **rpcm)
{
#ifdef CONFIG_MACH_MX27
	if (cpu_is_mx27())
		return mx27_pcm_new(card, client, rpcm);
#endif
#ifdef CONFIG_ARCH_MX25
	if (cpu_is_mx25())
		return mxc_pcm_v2_new(card, client, rpcm);
#endif
#ifdef CONFIG_ARCH_MX51
	if (cpu_is_mx51())
		return mxc_pcm_v2_new(card, client, rpcm);
#endif
	BUG();
}
