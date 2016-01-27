/*
 * include/asm-arm/arch-mxc/ac97_audio.h
 *
 * Copyright (C) 2008  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
 */

#include <sound/pcm.h>


typedef struct mxc_ac97_audio_ops {
	int (*init)(struct platform_device *dev, int ssi);
	void (*exit)(struct platform_device *dev);
	int (*startup)(struct snd_pcm_substream *stream, void *priv_data);
	void (*shutdown)(struct snd_pcm_substream *stream, void *priv_data);
	void (*suspend)(void *priv_data);
	void (*resume)(void *priv_data);
	void *priv;
} mxc_ac97_audio_ops_t;
