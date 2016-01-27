/*
 * sound/arm/mxc-ac97.c
 *
 * Copyright (C) 2008  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * based on pxa2xx-ac97.c by Nicolas Pitre
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
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>

#include <asm/irq.h>
#include <asm/dma.h>

#include <mach/hardware.h>
#include <mach/imx_ssi.h>
#include <mach/ssi_port.h>
#include <mach/mxc-ac97.h>
#include <mach/mxc_audmux.h>

#include "mxc-pcm.h"
#include "mxc-ac97.h"

#ifdef DEBUG
static int debug = 0;
#define dbg_lvl(n)	((n) < debug)
module_param(debug, int, S_IRUGO | S_IWUSR);
#else
#define dbg_lvl(n)	0
static int debug;
module_param(debug, int, 0);
#endif

#define DBG(lvl, fmt...)	do {			\
	if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt);	\
} while (0)

#define platform_func(p, args...)	((p) ? (p)(args) : 0)

static int variable_rate = 1;
module_param(variable_rate, bool, S_IRUGO | S_IWUSR);

static int timeout = 2;
#ifdef DEBUG
module_param(timeout, int, S_IRUGO | S_IWUSR);
#else
module_param(timeout, int, 0);
#endif

/*
 * Follow below sequence for programming the SSI to work in AC97 mode:
 * � Program the WL bits to a value corresponding to either 16 or 20 bits.
 * The WL bit setting is only
 * for the data portion of the AC97 frame (Slots #3 through #12).
 * The Tag slot (Slot #0) is always 16 bits wide and the Command Address and
 * Command Data slots (Slots #1 and #2) are always 20 bits wide.
 * � Select the number of time slots through DC bits. For AC97 operation,
 *   DC bits should be set to a value of `0xC', resulting in 13 time slots
 *   per frame.
 * � Write data to be transmitted, in Tx FIFO 0 (through Tx Data Register 0)
 * � Program the FV, TIF, RD, WR and FRDIV bits in SACNT register
 * � Update the contents of SACADD, SACDAT and (for fixed mode SATAG) registers
 * � Enable the AC97 mode of operation (AC97EN bit in SACNT register)
*/

struct mxc_ac97_priv {
	struct resource *res_mem;
	/* ioremapped base address of register area */
	void __iomem *base_addr;
	/* AC97 interrupt */
	int irq;

	spinlock_t lock;
	/* wait queue head for AC97 codec access operations */
	wait_queue_head_t wq;
	unsigned int fct;	/* frame counter for ac97 read/write operations */
	int active;
	struct mxc_ssi_port *ssi_port;
	int tx_underrun;
	u16 satag;
	u16 frdiv;
};

#ifdef DEBUG
#define mxc_reg_read(p,r)	_mxc_reg_read(p, r, __FUNCTION__, #r)
static u32 _mxc_reg_read(struct mxc_ac97_priv *priv, unsigned int reg,
			  const char *func, const char *name)
{
	u32 val = __raw_readl(priv->base_addr + reg);
	DBG(3, "%s: RD: %08x < %s(%08lx)\n", func, val, name,
	    (unsigned long)priv->res_mem->start + reg);
	return val;
}

#define mxc_reg_write(p,r,v)	_mxc_reg_write(p, r, v, __FUNCTION__, #r)
static void _mxc_reg_write(struct mxc_ac97_priv *priv, unsigned int reg,
			u32 val, const char *func, const char *name)
{
	DBG(3, "%s: WR: %08x > %s(%08lx)\n", func, val, name,
	    (unsigned long)priv->res_mem->start + reg);
	__raw_writel(val, priv->base_addr + reg);
}

#define mxc_set_mask(p,r,m)	_mxc_set_mask(p, r, m, __FUNCTION__, #r)
static void _mxc_set_mask(struct mxc_ac97_priv *priv, unsigned int reg,
			u32 mask, const char *func, const char *name)
{
	u32 val = _mxc_reg_read(priv, reg, func, name);
	_mxc_reg_write(priv, reg, val | mask, func, name);
}

#define mxc_clr_mask(p,r,m)	_mxc_clr_mask(p, r, m, __FUNCTION__, #r)
static void _mxc_clr_mask(struct mxc_ac97_priv *priv, unsigned int reg,
			u32 mask, const char *func, const char *name)
{
	u32 val = _mxc_reg_read(priv, reg, func, name);
	_mxc_reg_write(priv, reg, val & ~mask, func, name);
}

#define mxc_update_mask(p,r,c,s)	_mxc_update_mask(p, r, c, s,	\
	__FUNCTION__, #r)
static void _mxc_update_mask(struct mxc_ac97_priv *priv,
			unsigned int reg, u32 clr, u32 set,
			const char *func, const char *name)
{
	u32 val = _mxc_reg_read(priv, reg, func, name);
	_mxc_reg_write(priv, reg, (val & ~clr) | set, func, name);
}
#else
static u32 mxc_reg_read(struct mxc_ac97_priv *priv, unsigned int reg)
{
	return __raw_readl(priv->base_addr + reg);
}

static void mxc_reg_write(struct mxc_ac97_priv *priv, unsigned int reg,
			u32 val)
{
	__raw_writel(val, priv->base_addr + reg);
}

static void mxc_set_mask(struct mxc_ac97_priv *priv, unsigned int reg,
			u32 mask)
{
	mxc_reg_write(priv, reg, mxc_reg_read(priv, reg) | mask);
}

static void mxc_clr_mask(struct mxc_ac97_priv *priv, unsigned int reg,
			u32 mask)
{
	mxc_reg_write(priv, reg, mxc_reg_read(priv, reg) & ~mask);
}

static void mxc_update_mask(struct mxc_ac97_priv *priv, unsigned int reg,
			u32 clr, u32 set)
{
	u32 val = mxc_reg_read(priv, reg);
	mxc_reg_write(priv, reg, (val & ~clr) | set);
}
#endif

static void mxc_ac97_clk_enable(struct mxc_ac97_priv *priv)
{
	clk_enable(priv->ssi_port->ssi_clk);
	mxc_clr_mask(priv, SSI_SOR, SOR_CLKOFF);
}

static void mxc_ac97_clk_disable(struct mxc_ac97_priv *priv)
{
	clk_disable(priv->ssi_port->ssi_clk);
	mxc_set_mask(priv, SSI_SOR, SOR_CLKOFF);
}

static void mxc_ack_int(struct mxc_ac97_priv *priv, u32 mask)
{
	(void)mxc_reg_read(priv, SSI_SISR); /* clear last RLS/TLS status */
}

static inline int mxc_ac97_done(struct mxc_ac97_priv *priv)
{
	int ret;

	ret = wait_event_timeout(priv->wq, priv->fct == 0, timeout);
	if (ret == 0) {
		printk(KERN_WARNING "%s: Timeout waiting for data\n", __FUNCTION__);
	}
	return ret > 0;
}

/* must be called with interrupts disabled */
static void mxc_ac97_enable_slots(struct mxc_ac97_priv *priv)
{
	if (priv->active++)
		return;

	DBG(0, "%s: Enabling audio slots\n", __FUNCTION__);
	if (variable_rate) {
		mxc_reg_write(priv, SSI_SACCEN, SACC_SLOT(3) | SACC_SLOT(4));
	} else {
		priv->satag |= SATAG_SLOT(3) | SATAG_SLOT(4);
		mxc_reg_write(priv, SSI_SATAG, priv->satag);
	}
}

/* must be called with interrupts disabled */
static void mxc_ac97_disable_slots(struct mxc_ac97_priv *priv)
{
	if (--priv->active)
		return;

	DBG(0, "%s: Disabling audio slots\n", __FUNCTION__);
	if (variable_rate) {
		mxc_reg_write(priv, SSI_SACCDIS, SACC_SLOT(3) | SACC_SLOT(4));
	} else {
		priv->satag &= ~(SATAG_SLOT(3) | SATAG_SLOT(4));
		mxc_reg_write(priv, SSI_SATAG, priv->satag);
	}
}

static unsigned short mxc_ac97_read(struct snd_ac97 *ac97, unsigned short reg)
{
	struct mxc_ac97_priv *priv = ac97->bus->card->private_data;
	unsigned short val;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	WARN_ON(priv->fct != 0);
	mxc_reg_write(priv, SSI_SACADD, reg << 12);
	mxc_ack_int(priv, SISR_RLS);
	priv->fct = 2;
	mxc_set_mask(priv, SSI_SIER, SIER_RLS_EN | SIER_RIE);
	mxc_set_mask(priv, SSI_SACNT, SACNT_RD);
	spin_unlock_irqrestore(&priv->lock, flags);
	if (mxc_ac97_done(priv)) {
		val = mxc_reg_read(priv, SSI_SACDAT) >> 4;
		DBG(1, "%s: Read 0x%04x from AC97 reg 0x%02x\n", __FUNCTION__, val, reg);
	} else {
		priv->fct = 0;
		val = mxc_reg_read(priv, SSI_SACDAT) >> 4;
		printk(KERN_WARNING "Timeout reading AC97 reg 0x%02x (data: 0x%04x)\n",
			reg, val);
	}
	spin_lock_irqsave(&priv->lock, flags);
	spin_unlock_irqrestore(&priv->lock, flags);

	return val;
}

static void mxc_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
			unsigned short val)
{
	struct mxc_ac97_priv *priv = ac97->bus->card->private_data;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	WARN_ON(priv->fct != 0);
	mxc_reg_write(priv, SSI_SACADD, reg << 12);
	mxc_reg_write(priv, SSI_SACDAT, val << 4);
	mxc_ack_int(priv, SISR_TLS);
	priv->fct = 2;
	mxc_set_mask(priv, SSI_SIER, SIER_TLS_EN | SIER_TIE);
	mxc_set_mask(priv, SSI_SACNT, SACNT_WR);
	spin_unlock_irqrestore(&priv->lock, flags);
	if (mxc_ac97_done(priv)) {
		DBG(1, "%s: Wrote 0x%04x to AC97 reg 0x%02x\n", __FUNCTION__,
			val, reg);
	} else {
		priv->fct = 0;
		printk(KERN_WARNING "Timeout writing 0x%04x to AC97 reg 0x%02x\n",
			val, reg);
	}
	spin_lock_irqsave(&priv->lock, flags);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void mxc_ac97_reset(struct snd_ac97 *ac97)
{
	struct mxc_ac97_priv *priv = ac97->bus->card->private_data;

	DBG(0, "%s: \n", __FUNCTION__);
	mxc_reg_write(priv, SSI_SOR, SOR_TX_CLR | SOR_RX_CLR);
	mxc_ac97_write(ac97, 0, 0);
}

static irqreturn_t mxc_ac97_irq(int irq, void *dev_id)
{
	struct mxc_ac97_priv *priv = dev_id;
	u32 sisr = mxc_reg_read(priv, SSI_SISR);
	u32 sier = mxc_reg_read(priv, SSI_SIER);
	u32 handled = sisr & sier;
	int i;
	int need_wake = 0;

	DBG(handled ? 3 : 4,
		"%s: sisr=%08x sier=%08x handled=%08x\n",
		__FUNCTION__, sisr, sier, handled);

	if (WARN_ON(!handled)) {
		return IRQ_NONE;
	}
	for (i = 0; i < 22; i++) {
		if (handled & (1 << i)) {
			sier &= ~(1 << i);
		}
	}
	if (handled & SISR_TUE0) {
		DBG(-1, "Transmit Underrun\n");
		sier &= ~SIER_TUE0_EN;
		priv->tx_underrun++;
	}
	if (handled & SISR_TDE0) {
		DBG(1, "%s: TDE0\n", __FUNCTION__);
	}
	if (handled & SISR_RDR0) {
		DBG(1, "%s: RDR0\n", __FUNCTION__);
	}
	if (handled & SISR_TFE0) {
		DBG(1, "%s: TFE0\n", __FUNCTION__);
	}
	if (handled & SISR_ROE0) {
		DBG(1, "%s: ROE0\n", __FUNCTION__);
	}
	if (handled & (SISR_TFS | SISR_RFS)) {
		DBG(3, "%s: TFS|RFS:\n", __FUNCTION__);
		need_wake = 1;
	}
	if (handled & SISR_TLS) {
		if (priv->fct) {
			need_wake = !(mxc_reg_read(priv, SSI_SACNT) & SACNT_WR);
			DBG(1, "%s: TLS: %d:%d\n", __FUNCTION__, need_wake, priv->fct);
			if (!need_wake) {
				DBG(1, "%s: Ignoring TLS: %d\n",
					__FUNCTION__, priv->fct);
				sier |= SIER_TIE | SIER_TLS_EN;
			} else {
				priv->fct--;
				if (priv->fct > 0) {
					need_wake = 0;
					sier |= SIER_TIE | SIER_TLS_EN;
				}
			}
		} else {
			DBG(1, "%s: Ignoring TLS: %d\n",
				__FUNCTION__, priv->fct);
		}
	}
	if (handled & SISR_RLS) {
		if (priv->fct) {
			need_wake = !(mxc_reg_read(priv, SSI_SACNT) & SACNT_RD);
			DBG(1, "%s: RLS: %d:%d\n", __FUNCTION__, need_wake, priv->fct);
			if (!need_wake) {
				DBG(1, "%s: Ignoring RLS: %d\n",
					__FUNCTION__, priv->fct);
				sier |= SIER_RIE | SIER_RLS_EN;
			} else {
				priv->fct--;
				if (priv->fct > 0) {
					need_wake = 0;
					sier |= SIER_RIE | SIER_RLS_EN;
				}
			}
		} else {
			DBG(1, "%s: Ignoring RLS: %d\n",
				__FUNCTION__, priv->fct);
		}
	}
	if (need_wake && waitqueue_active(&priv->wq)) {
		DBG(3, "%s: Waking up\n", __FUNCTION__);
		wake_up(&priv->wq);
	}
	mxc_reg_write(priv, SSI_SIER, sier);

	return IRQ_RETVAL(handled);
}

/*
The correct sequence to initialize the SSI is as follows:
   1. Issue a Power-on or SSI reset (SCR[SSIEN]=0).
   2. Disable SSI clocks (ipg_clk, ccm_ssi_clk).
   3. Set all control bits for configuring the SSI (refer to Table 42-38).
   4. Enable appropriate interrupts/DMA requests through SIER.
   5. Set the SCR[SSIEN] bit (=1) to enable the SSI.
   6. Enable SSI clocks (ipg_clk, ccm_ssi_clk), as required.
   7. In case of AC97 mode, set the SACNT[AC97EN] bit after programming the
      SATAG register (if needed, for AC97 Fixed mode).
   8. Set SCR[TE/RE] bits.
*/
static int __init mxc_ac97_setup(struct mxc_ac97_priv *priv)
{
	struct mxc_ssi_port *ssi_port = priv->ssi_port;

	/* SSI Reset */
	mxc_reg_write(priv, SSI_SCR, SCR_CLK_IST);
	mxc_reg_write(priv, SSI_SOR,
		SOR_CLKOFF |
		SOR_TX_CLR |
		SOR_RX_CLR |
		SOR_INIT);
	mxc_update_mask(priv, SSI_SFCSR,
			SSI_SFCSR_TFWM0_MASK | SSI_SFCSR_RFWM0_MASK,
			SSI_SFCSR_TFWM0(TXFIFO_WATERMARK) |
			SSI_SFCSR_RFWM0(RXFIFO_WATERMARK));
	/* in AC97 mode only STCCR[WL] and STCCR[DC] need to be configured */
	mxc_update_mask(priv, SSI_STCCR,
			STCCR_WL_MASK | STCCR_DC_MASK,
			(7 << STCCR_WL_SHIFT) |
			(0x0c << STCCR_DC_SHIFT));
	if (variable_rate) {
		mxc_reg_write(priv, SSI_SACCEN, SACC_SLOT(0));
	} else {
		priv->satag |= SATAG_SLOT(0);
		mxc_reg_write(priv, SSI_SATAG, priv->satag);
	}
	mxc_reg_write(priv, SSI_SFCSR, 0x00880088);
	mxc_set_mask(priv, SSI_STCR, STCR_TFEN0);
	mxc_set_mask(priv, SSI_SRCR, SRCR_RFEN0);
	mxc_set_mask(priv, SSI_SCR, SCR_SSIEN);
	clk_enable(ssi_port->ssi_clk);
	mxc_reg_write(priv, SSI_SACNT, SACNT_AC97EN |
		(variable_rate * SACNT_FV));
	mxc_set_mask(priv, SSI_SCR, SCR_TE | SCR_RE);
	return 0;
}

static struct snd_ac97_bus_ops mxc_ac97_ops = {
	.read	= mxc_ac97_read,
	.write	= mxc_ac97_write,
	.reset	= mxc_ac97_reset,
};

static struct mxc_pcm_dma_params mxc_ac97_pcm_out = {
	.name			= "AC97 PCM out",
};

static struct mxc_pcm_dma_params mxc_ac97_pcm_in = {
	.name			= "AC97 PCM in",
};

static struct snd_pcm *mxc_ac97_pcm;
static struct snd_ac97 *mxc_ac97_ac97;

static int mxc_ac97_pcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	mxc_ac97_audio_ops_t *platform_ops;
	int r;

	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	r = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		AC97_RATES_FRONT_DAC : AC97_RATES_ADC;
	runtime->hw.rates = mxc_ac97_ac97->rates[r];
	snd_pcm_limit_hw_rates(runtime);

	platform_ops = substream->pcm->card->dev->platform_data;

	return platform_func(platform_ops->startup, substream,
			platform_ops->priv);
}

static int mxc_ac97_pcm_start(struct snd_pcm_substream *substream)
{
	struct mxc_ac97_priv *priv = substream->pcm->card->private_data;
	unsigned long flags;

	DBG(0, "%s: Starting XFER %d\n", __FUNCTION__, priv->active);
	spin_lock_irqsave(&priv->lock, flags);
	priv->tx_underrun = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mxc_set_mask(priv, SSI_SIER, SIER_TDMAE | SIER_TUE0_EN);
	} else {
		mxc_set_mask(priv, SSI_SIER, SIER_RDMAE | SIER_ROE0_EN);
	}

	mxc_ac97_enable_slots(priv);
	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

static void mxc_ac97_pcm_stop(struct snd_pcm_substream *substream)
{
	struct mxc_ac97_priv *priv = substream->pcm->card->private_data;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	mxc_ac97_disable_slots(priv);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mxc_clr_mask(priv, SSI_SIER, SIER_TDMAE | SIER_TUE0_EN);
		mxc_set_mask(priv, SSI_SOR, SOR_TX_CLR);
	} else {
		mxc_clr_mask(priv, SSI_SIER, SIER_RDMAE | SIER_ROE0_EN);
		mxc_set_mask(priv, SSI_SOR, SOR_RX_CLR);
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	DBG(priv->tx_underrun ? -1 : 0,
		"%s: XFER stopped %d; %d TX underrun errors\n", __FUNCTION__,
		priv->active, priv->tx_underrun);
}

static void mxc_ac97_pcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_card *card = substream->pcm->card;
	mxc_ac97_audio_ops_t *platform_ops = card->dev->platform_data;
	struct mxc_ac97_priv *priv = card->private_data;

	DBG(0, "%s: \n", __FUNCTION__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mxc_set_mask(priv, SSI_SOR, SOR_TX_CLR);
	} else {
		mxc_set_mask(priv, SSI_SOR, SOR_RX_CLR);
	}
	platform_func(platform_ops->shutdown, substream, platform_ops->priv);
}

static int mxc_ac97_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_ac97_priv *priv = substream->pcm->card->private_data;
	int reg = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		  AC97_PCM_FRONT_DAC_RATE : AC97_PCM_LR_ADC_RATE;
	unsigned long flags;
	unsigned int ac97_rate;

	if (runtime->rate == 0)
		return -EINVAL;
	ac97_rate = runtime->rate;

	if (variable_rate) {
		DBG(0, "%s: Setting sample rate to %u\n", __FUNCTION__,
			runtime->rate);
		if (!(mxc_ac97_read(mxc_ac97_ac97, AC97_EXTENDED_ID) &
				AC97_EI_VRA)) {
			DBG(0, "%s: Codec does not support variable rate\n",
				__FUNCTION__);
			return -EINVAL;
		}
		mxc_ac97_write(mxc_ac97_ac97, AC97_EXTENDED_STATUS,
			AC97_EA_VRA);
	} else {
		if (48000 % runtime->rate != 0)
			return -EINVAL;

		mxc_ac97_write(mxc_ac97_ac97, AC97_EXTENDED_STATUS, 0);
	}
	ret = snd_ac97_set_rate(mxc_ac97_ac97, reg, ac97_rate);
	if (ret) {
		return ret;
	}
	spin_lock_irqsave(&priv->lock, flags);
	if (variable_rate) {
		if (!(mxc_reg_read(priv, SSI_SACNT) & SACNT_FV)) {
			DBG(0, "%s: Enabling variable rate\n", __FUNCTION__);
			mxc_update_mask(priv, SSI_SACNT, SACNT_FRDIV_MASK,
					SACNT_FV);
		}
	} else {
		unsigned int frdiv = 48000 / runtime->rate;

		DBG(1, "%s: Setting frdiv to %u\n", __FUNCTION__, frdiv);
		if (frdiv == 0) {
			ret = -EINVAL;
			goto err;
		}
		if (priv->active && priv->frdiv != frdiv) {
			ret = -EBUSY;
			goto err;
		}

		priv->frdiv = frdiv;

		DBG(0, "%s: Disabling variable rate\n", __FUNCTION__);
		mxc_update_mask(priv, SSI_SACNT, SACNT_FRDIV_MASK |
				SACNT_FV,
				((frdiv - 1) << SACNT_FRDIV_SHIFT));
	}
err:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static struct mxc_pcm_client mxc_ac97_pcm_client = {
	.playback_params	= &mxc_ac97_pcm_out,
	.capture_params		= &mxc_ac97_pcm_in,
	.startup		= mxc_ac97_pcm_startup,
	.shutdown		= mxc_ac97_pcm_shutdown,
	.prepare		= mxc_ac97_pcm_prepare,
	.start_xfer		= mxc_ac97_pcm_start,
	.stop_xfer		= mxc_ac97_pcm_stop,
};

#ifdef CONFIG_PM
static int mxc_ac97_do_suspend(struct snd_card *card, pm_message_t state)
{
	mxc_ac97_audio_ops_t *platform_ops = card->dev->platform_data;
	struct mxc_ac97_priv *priv = card->private_data;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);
	snd_pcm_suspend_all(mxc_ac97_pcm);
	snd_ac97_suspend(mxc_ac97_ac97);
	platform_func(platform_ops->suspend, platform_ops->priv);
	if (priv->active)
		mxc_clr_mask(priv, SSI_SCR, SCR_SSIEN);
	mxc_ac97_clk_disable(priv);

	return 0;
}

static int mxc_ac97_do_resume(struct snd_card *card)
{
	mxc_ac97_audio_ops_t *platform_ops = card->dev->platform_data;
	struct mxc_ac97_priv *priv = card->private_data;

	mxc_ac97_clk_enable(priv);
	if (priv->active)
		mxc_set_mask(priv, SSI_SCR, SCR_SSIEN);
	platform_func(platform_ops->resume, platform_ops->priv);
	snd_ac97_resume(mxc_ac97_ac97);
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);

	return 0;
}

static int mxc_ac97_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (card != NULL) {
		ret = mxc_ac97_do_suspend(card, PMSG_SUSPEND);
	}
	return ret;
}

static int mxc_ac97_resume(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (card != NULL) {
		ret = mxc_ac97_do_resume(card);
	}
	return ret;
}

#else
#define mxc_ac97_suspend	NULL
#define mxc_ac97_resume	NULL
#endif

static int __init mxc_ac97_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res_mem;
	struct resource *res_irq;
	struct resource *res_tx_dma;
	struct resource *res_rx_dma;
	mxc_ac97_audio_ops_t *platform_ops = pdev->dev.platform_data;
	struct snd_card *card;
	struct mxc_ac97_priv *priv;
	struct snd_ac97_bus *ac97_bus;
	struct snd_ac97_template ac97_template;
	struct mxc_ssi_port *ssi_port;

	ret = mxc_ssi_request_port(-1, pdev, &ssi_port);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request SSI port; error: %d\n",
			ret);
		return ret;
	}
	DBG(0, "%s: Using SSI port %d\n", __FUNCTION__, ret);

	res_mem = &ssi_port->res[0];
	res_irq = &ssi_port->res[1];
	res_tx_dma = &ssi_port->res[2];
	res_rx_dma = &ssi_port->res[3];

	if (res_mem == NULL || res_irq == 0) {
		mxc_ssi_release_port(ssi_port);
		ret = -ENODEV;
		goto rel_port;
	}
	/* we want at least one DMA channel */
	if (res_rx_dma == NULL && res_rx_dma == NULL) {
		DBG(0, "%s: No DMA channels given\n", __FUNCTION__);
		mxc_ssi_release_port(ssi_port);
		ret = -ENODEV;
		goto rel_port;
	}

	ret = snd_card_create(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			THIS_MODULE, sizeof(struct mxc_ac97_priv), &card);
	if (ret)
		goto rel_port;

	priv = card->private_data;

	spin_lock_init(&priv->lock);
	init_waitqueue_head(&priv->wq);

	priv->ssi_port = ssi_port;
	priv->res_mem = res_mem;
	priv->base_addr = ioremap(res_mem->start, resource_size(res_mem));
	if (priv->base_addr == NULL) {
		ret = -ENOMEM;
		goto free;
	}

	priv->irq = res_irq->start;
	DBG(0, "%s: memory @ %08lx remapped to %p\n", __FUNCTION__,
	    (unsigned long)res_mem->start, priv->base_addr);

	ret = mxc_ac97_setup(priv);
	if (ret != 0) {
		goto unmap;
	}

	ret = platform_func(platform_ops->init, pdev, ssi_port->num);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to initialize AC97 hardware: %d\n",
			ret);
		goto unmap;
	}
	if (res_rx_dma != NULL) {
		mxc_ac97_pcm_in.dev_addr = res_mem->start + SSI_SRX0;
		mxc_ac97_pcm_in.channel_id = res_rx_dma->start;
	} else {
		mxc_ac97_pcm_client.capture_params = NULL;
	}

	if (res_tx_dma != NULL) {
		mxc_ac97_pcm_out.dev_addr = res_mem->start + SSI_STX0;
		mxc_ac97_pcm_out.channel_id = res_tx_dma->start;
	} else {
		mxc_ac97_pcm_client.playback_params = NULL;
	}

	card->dev = &pdev->dev;
	strncpy(card->driver, pdev->dev.driver->name, sizeof(card->driver));

	ret = mxc_pcm_new(card, &mxc_ac97_pcm_client, &mxc_ac97_pcm);
	if (ret != 0) {
		goto plf_exit;
	}

	DBG(0, "%s: Requesting IRQ %d\n", __FUNCTION__, (int)res_irq->start);
	ret = request_irq(res_irq->start, mxc_ac97_irq, 0, "AC97", priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n",
			priv->irq, ret);
		goto plf_exit;
	}

	mxc_ac97_clk_enable(priv);

	DBG(0, "%s: Registering AC97 bus\n", __FUNCTION__);
	ret = snd_ac97_bus(card, 0, &mxc_ac97_ops, NULL, &ac97_bus);
	if (ret != 0) {
		goto disable;
	}
	memset(&ac97_template, 0, sizeof(ac97_template));
	ret = snd_ac97_mixer(ac97_bus, &ac97_template, &mxc_ac97_ac97);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to initialize mixer: %d\n",
			ret);
		goto disable;
	}
	snprintf(card->shortname, sizeof(card->shortname),
		 "%s", snd_ac97_get_short_name(mxc_ac97_ac97));
	snprintf(card->longname, sizeof(card->longname),
		 "%s (%s)", pdev->dev.driver->name, card->mixername);

	DBG(0, "%s: Registering sound card %s\n", __FUNCTION__,
		card->shortname);
	ret = snd_card_register(card);
	if (ret == 0) {
		platform_set_drvdata(pdev, card);
		return 0;
	}

disable:
	mxc_ac97_clk_disable(priv);
	free_irq(priv->irq, priv);

plf_exit:
	platform_func(platform_ops->exit, pdev);

unmap:
	iounmap(priv->base_addr);

free:
	snd_card_free(card);

rel_port:
	mxc_ssi_release_port(ssi_port);
	return ret;
}

static int __exit mxc_ac97_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct mxc_ac97_priv *priv = card->private_data;
	mxc_ac97_audio_ops_t *platform_ops = card->dev->platform_data;
	void __iomem *base_addr = priv->base_addr;
	int irq = priv->irq;
	struct mxc_ssi_port *ssi_port = priv->ssi_port;

	DBG(0, "%s: Freeing sound card\n", __FUNCTION__);
	snd_card_free(card); /* requires clock running */

	DBG(0, "%s: Disabling SSI interface\n", __FUNCTION__);
	__raw_writel(0, base_addr + SSI_SCR);

	/*
	 * The data structure behind 'priv' has been freed by snd_card_free(),
	 * so mxc_ac97_clk_disable() and mxc_reg_write() cannot be used
	 * any more.
	 */
	DBG(0, "%s: Disabling ac97 clock %p\n", __FUNCTION__,
		ssi_port->ssi_clk);
	clk_disable(ssi_port->ssi_clk);

	DBG(0, "%s: Freeing IRQ %d\n", __FUNCTION__, irq);
	free_irq(irq, priv);

	platform_set_drvdata(pdev, NULL);

	DBG(0, "%s: Unmapping %p\n", __FUNCTION__, base_addr);
	iounmap(base_addr);

	DBG(0, "%s: Releasing SSI port %d\n", __FUNCTION__, ssi_port->num);
	mxc_ssi_release_port(ssi_port);

	DBG(0, "%s: Releasing GPIO pins\n", __FUNCTION__);
	platform_func(platform_ops->exit, pdev);
	return 0;
}

static struct platform_driver mxc_ac97_driver = {
	.probe		= mxc_ac97_probe,
	.remove		= __exit_p(mxc_ac97_remove),
	.suspend	= mxc_ac97_suspend,
	.resume		= mxc_ac97_resume,
	.driver		= {
		.name	= "mxc-ac97",
	},
};

static int __init mxc_ac97_init(void)
{
	return platform_driver_register(&mxc_ac97_driver);
}

static void __exit mxc_ac97_exit(void)
{
	platform_driver_unregister(&mxc_ac97_driver);
}

module_init(mxc_ac97_init);
module_exit(mxc_ac97_exit);

MODULE_AUTHOR("Lothar Wassmann");
MODULE_DESCRIPTION("AC97 driver for the Freescale i.MX27 and i.MX25 chips");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mxc-ac97");
