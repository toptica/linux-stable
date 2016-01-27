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
 * @file plat-mxc/sdma/dma_sdma.c
 * @brief Front-end to the DMA handling.  This handles the allocation/freeing
 * of DMA channels, and provides a unified interface to the machines
 * DMA facilities. This file contains functions for Smart DMA.
 *
 * @ingroup SDMA
 */

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include <epm.h>
#include "sdma.h"

#ifdef CONFIG_MXC_SDMA_API

static mxc_dma_channel_t mxc_sdma_channels[MAX_DMA_CHANNELS];
static mxc_dma_channel_private_t mxc_sdma_private[MAX_DMA_CHANNELS];

/*!
 * Tasket to handle processing the channel buffers
 *
 * @param arg channel id
 */
static void mxc_sdma_channeltasklet(unsigned long arg)
{
	dma_request_t req;
	mxc_sdma_channel_params_t *chnl;
	dma_channel_params *chnl_param;
	mxc_dma_channel_t *chnl_info;
	mxc_dma_channel_private_t *data_priv;
	int ret;

	chnl_info = &mxc_sdma_channels[arg];
	data_priv = chnl_info->private;
	chnl = mxc_sdma_get_channel_params(chnl_info->channel);
	BUG_ON(chnl == NULL);
	chnl_param = &chnl->chnl_params;

	ret = mxc_dma_get_config(arg, &req, data_priv->buf_tail);
	if (ret) {
		DBG(0, "%s: Failed to get config for channel %ld buffer %u: %d\n",
			__FUNCTION__, arg, data_priv->buf_tail, ret);
		return;
	}

	DBG(0, "%s: dma_request %u: src: %08x dst: %08x done=%d cont=%d wrap=%d\n",
		__FUNCTION__, data_priv->buf_tail, req.sourceAddr, req.destAddr,
		req.bd_done, req.bd_cont, req.bd_wrap);

	while (!req.bd_done) {
		int error = MXC_DMA_DONE;
		int bd_intr = mxc_dma_get_bd_intr(arg, data_priv->buf_tail);

		if (bd_intr < 0) {
			DBG(0, "%s: Failed to get bd_intr for channel %ld buffer %u: %d\n",
				__FUNCTION__, arg, data_priv->buf_tail, ret);
			return;
		}

		DBG(0, "%s: desc %u of %u\n", __FUNCTION__,
			data_priv->buf_tail, chnl_param->bd_number);

		if (!data_priv->circular || req.bd_error) {
			DBG(0, "%s: Deactivating channel\n", __FUNCTION__);
			chnl_info->active = 0;
		}
		if (req.bd_error) {
			error = MXC_DMA_TRANSFER_ERROR;
		}

		DBG(0, "%s: bd_intr=%d\n", __FUNCTION__, bd_intr);
		if (bd_intr) {
			chnl_info->cb_fn(chnl_info->cb_args, error,
					 req.count);
		}

		if (data_priv->buf_tail == chnl_info->curr_buf) {
			break;
		}

		ret = mxc_dma_set_config(arg, &req, data_priv->buf_tail);
		if (ret) {
			DBG(0, "%s: Failed to set config for channel %ld buffer %u: %d\n",
				__FUNCTION__, arg, data_priv->buf_tail, ret);
			return;
		}

		data_priv->buf_tail++;
		if (data_priv->buf_tail >= chnl_param->bd_number || req.bd_wrap)
			data_priv->buf_tail = 0;
		memset(&req, 0, sizeof(req));
		ret = mxc_dma_get_config(arg, &req, data_priv->buf_tail);
		if (ret) {
			DBG(0, "%s: Failed to get config for channel %ld buffer %u: %d\n",
				__FUNCTION__, arg, data_priv->buf_tail, ret);
			return;
		}
		DBG(0, "%s: dma_request %u: src: %08x dst: %08x done=%d cont=%d wrap=%d\n",
			__FUNCTION__, data_priv->buf_tail, req.sourceAddr, req.destAddr,
			req.bd_done, req.bd_cont, req.bd_wrap);
	}
}

/*!
 * This function is generally called by the driver at open time.
 * The DMA driver would do any initialization steps that is required
 * to get the channel ready for data transfer.
 *
 * @param channel_id   a pre-defined id. The peripheral driver would specify
 *                     the id associated with its peripheral. This would be
 *                     used by the DMA driver to identify the peripheral
 *                     requesting DMA and do the necessary setup on the
 *                     channel associated with the particular peripheral.
 *                     The DMA driver could use static or dynamic DMA channel
 *                     allocation.
 * @param dev_name     module name or device name
 * @return returns a negative number on error if request for a DMA channel did not
 *         succeed, returns the channel number to be used on success.
 */
int mxc_dma_request_ext(mxc_dma_device_t channel_id, const char *dev_name,
			struct dma_channel_info *info)
{
	mxc_sdma_channel_params_t *chnl;
	mxc_dma_channel_private_t *data_priv;
	int ret, i, channel_num;
	mxc_sdma_channel_ext_params_t *p;

	chnl = mxc_sdma_get_channel_params(channel_id);
	if (chnl == NULL) {
		return -EINVAL;
	}

	if (info) {
		if (!chnl->chnl_params.ext)
			return -EINVAL;
		p = (mxc_sdma_channel_ext_params_t *)chnl;
		memcpy(&p->chnl_ext_params.info, info, sizeof(info));
	}

	/* Enable the SDMA clock */
	clk_enable(mxc_sdma_clk);

	channel_num = chnl->channel_num;
	if (channel_num == MXC_DMA_DYNAMIC_CHANNEL) {
		ret = -EBUSY;
		/* Get the first free channel */
		for (i = MAX_DMA_CHANNELS - 1; i > 0; i--) {
			/* See if channel is available */
			if (!mxc_sdma_channels[i].dynamic ||
				mxc_sdma_channels[i].lock) {
				continue;
			}
			channel_num = i;
			/* Check to see if we can get this channel */
			ret = mxc_request_dma(&channel_num, dev_name);
			if (ret == 0) {
				break;
			} else {
				continue;
			}
		}
		if (ret != 0) {
			/* No free channel */
			goto err_ret;
		}
	} else {
		if (mxc_sdma_channels[channel_num].lock) {
			ret = -EBUSY;
			goto err_ret;
		}
		ret = mxc_request_dma(&channel_num, dev_name);
		if (ret != 0) {
			goto err_ret;
		}
	}

	ret = mxc_dma_setup_channel(channel_num, &chnl->chnl_params);
	if (ret != 0) {
		DBG(0, "%s: Failed to setup DMA channel %d: %d\n", __FUNCTION__,
			channel_num, ret);
		goto err_free;
	}
	if (chnl->chnl_priority != MXC_SDMA_DEFAULT_PRIORITY) {
		ret = mxc_dma_set_channel_priority(channel_num,
						chnl->chnl_priority);
		if (ret != 0) {
			pr_info("Failed to set channel prority, continuing with the existing priority\n");
		}
	}
	mxc_sdma_channels[channel_num].lock = 1;
	if ((chnl->chnl_params.transfer_type == per_2_emi) ||
		(chnl->chnl_params.transfer_type == dsp_2_emi)) {
		mxc_sdma_channels[channel_num].mode =
			MXC_DMA_MODE_READ;
	} else {
		mxc_sdma_channels[channel_num].mode =
			MXC_DMA_MODE_WRITE;
	}
	mxc_sdma_channels[channel_num].channel = channel_id;
	data_priv = mxc_sdma_channels[channel_num].private;
	tasklet_init(&data_priv->chnl_tasklet,
		mxc_sdma_channeltasklet, channel_num);
	if ((channel_id == MXC_DMA_ATA_RX) ||
		(channel_id == MXC_DMA_ATA_TX)) {
		data_priv->intr_after_every_bd = 0;
	} else {
		data_priv->intr_after_every_bd = 1;
	}
	return channel_num;

err_free:
	mxc_free_sdma(channel_num);
err_ret:
	clk_disable(mxc_sdma_clk);
	return ret;
}

/*!
 * This function is generally called by the driver at close time. The DMA
 * driver would do any cleanup associated with this channel.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_free(int channel_num)
{
	mxc_dma_channel_private_t *data_priv;

	if (channel_num < 0 || channel_num >= MAX_DMA_CHANNELS) {
		return -EINVAL;
	}

	if (!mxc_sdma_channels[channel_num].lock) {
		return -EINVAL;
	}

	mxc_free_sdma(channel_num);

	/* Disable the SDMA clock */
	clk_disable(mxc_sdma_clk);

	mxc_sdma_channels[channel_num].lock = 0;
	mxc_sdma_channels[channel_num].active = 0;
	mxc_sdma_channels[channel_num].curr_buf = 0;
	data_priv = mxc_sdma_channels[channel_num].private;
	data_priv->buf_tail = 0;
	tasklet_kill(&data_priv->chnl_tasklet);

	return 0;
}

/*!
 * Callback function called from the SDMA Interrupt routine
 *
 * @param arg driver specific argument that was registered
 */
static void mxc_dma_chnl_callback(void *arg)
{
	int channel;
	mxc_dma_channel_private_t *data_priv;

	channel = (int)arg;
	DBG(0, "%s: channel %d\n", __FUNCTION__, channel);
	data_priv = mxc_sdma_channels[channel].private;
	/* Process the buffers in a tasklet */
	tasklet_schedule(&data_priv->chnl_tasklet);
}

/*!
 * This function would just configure the buffers specified by the user into
 * dma channel. The caller must call mxc_dma_enable to start this transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param dma_buf      an array of physical addresses to the user defined
 *                     buffers. The caller must guarantee the dma_buf is
 *                     available until the transfer is completed.
 * @param num_buf      number of buffers in the array
 * @param mode         specifies whether this is READ or WRITE operation
 * @return This function returns a negative number on error if buffer could not be
 *         added with DMA for transfer. On Success, it returns 0
 */
int mxc_dma_config(int channel_num, mxc_dma_requestbuf_t *dma_buf,
		   int num_buf, mxc_dma_mode_t mode)
{
	int ret, i, prev_buf;
	mxc_dma_channel_t *chnl_info;
	mxc_dma_channel_private_t *data_priv;
	mxc_sdma_channel_params_t *chnl;
	dma_channel_params chnl_param;
	dma_request_t req;

	if (channel_num < 0 || channel_num >= MAX_DMA_CHANNELS)
		return -EINVAL;

	if (num_buf <= 0)
		return -EINVAL;

	chnl_info = &mxc_sdma_channels[channel_num];
	data_priv = chnl_info->private;
	if (chnl_info->lock != 1)
		return -ENODEV;

	/* Check to see if all buffers are taken */
	if (chnl_info->active)
		return -EBUSY;

	chnl = mxc_sdma_get_channel_params(chnl_info->channel);
	chnl_param = chnl->chnl_params;

	/* Re-setup the SDMA channel if the transfer direction or
	 * the number of assigned BDs is changed
	 */
	if (chnl_param.peripheral_type != MEMORY &&
		mode != chnl_info->mode) {

		if (chnl_param.bd_number != num_buf)
			DBG(0, "%s: Changing number of BDs for channel %u from %u to %u\n", __FUNCTION__,
				channel_num, chnl_param.bd_number, num_buf);

		chnl_param.bd_number = num_buf;
		if (chnl_param.peripheral_type == DSP) {
			if (mode == MXC_DMA_MODE_READ)
				chnl_param.transfer_type = dsp_2_emi;
			else
				chnl_param.transfer_type = emi_2_dsp;
		} else if (chnl_param.peripheral_type == FIFO_MEMORY) {
			if (mode == MXC_DMA_MODE_READ)
				chnl_param.per_address =
					MXC_FIFO_MEM_SRC_FIXED;
			else
				chnl_param.per_address =
					MXC_FIFO_MEM_DEST_FIXED;
		} else {
			if (mode == MXC_DMA_MODE_READ)
				chnl_param.transfer_type = per_2_emi;
			else
				chnl_param.transfer_type = emi_2_per;
		}
		chnl_param.callback = mxc_dma_chnl_callback;
		chnl_param.arg = (void *)channel_num;
		ret = mxc_dma_setup_channel(channel_num, &chnl_param);
		if (ret != 0)
			return ret;

		if (chnl->chnl_priority != MXC_SDMA_DEFAULT_PRIORITY) {
			ret = mxc_dma_set_channel_priority(channel_num,
							chnl->chnl_priority);
			if (ret != 0) {
				pr_info("Failed to set channel prority, continuing with the existing priority\n");
			}
		}
		chnl_info->mode = mode;
	}

	for (i = 0; i < num_buf; i++, dma_buf++) {
		memset(&req, 0, sizeof(req));
		/* Check to see if all buffers are taken */
		if (chnl_info->active) {
			DBG(0, "%s: bd %u is already active\n",
				__FUNCTION__, i);
			break;
		}
		req.destAddr = dma_buf->dst_addr;
		req.sourceAddr = dma_buf->src_addr;
		if (chnl_param.peripheral_type == ASRC)
			req.count = dma_buf->num_of_bytes / 4;
		else
			req.count = dma_buf->num_of_bytes;
		req.bd_cont = 1;
		if (data_priv->circular && i == num_buf - 1) {
			DBG(0, "%s: Setting bd_wrap for desc[%d/%d]\n",
				__FUNCTION__, i, num_buf);
			req.bd_wrap = 1;
		}
		ret = mxc_dma_set_config(channel_num, &req,
					chnl_info->curr_buf);
		if (ret != 0) {
			DBG(0, "%s: Failed to configure DMA: %d\n",
				__FUNCTION__, ret);
			return ret;
		}
		if (data_priv->intr_after_every_bd ||
			(i == num_buf - 1)) {
			mxc_dma_set_bd_intr(channel_num,
					chnl_info->curr_buf, 1);
		} else {
			mxc_dma_set_bd_intr(channel_num,
					chnl_info->curr_buf, 0);
		}

		prev_buf = chnl_info->curr_buf;
		chnl_info->curr_buf++;
		if (chnl_info->curr_buf >= chnl_param.bd_number) {
			chnl_info->curr_buf = 0;
		}
		if (chnl_info->curr_buf == data_priv->buf_tail) {
			if (!(data_priv->intr_after_every_bd) &&
				(i != num_buf - 1)) {
				/*
				 * Set the BD_INTR flag on the last BD that
				 * was queued
				 */
				mxc_dma_set_bd_intr(channel_num, prev_buf, 1);
			}
			chnl_info->active = 1;
		}
	}

	if (i == 0) {
		return -EBUSY;
	}
	return 0;
}

/*!
 * This function would just configure the scatterlist specified by the
 * user into dma channel. This is a slight variation of mxc_dma_config(),
 * it is provided for the convenience of drivers that have a scatterlist
 * passed into them. It is the calling driver's responsibility to have the
 * correct physical address filled in the "dma_address" field of the
 * scatterlist.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param sg           a scatterlist of buffers. The caller must guarantee
 *                     the dma_buf is available until the transfer is
 *                     completed.
 * @param num_buf      number of buffers in the array
 * @param num_of_bytes total number of bytes to transfer. If set to 0, this
 *                     would imply to use the length field of the scatterlist
 *                     for each DMA transfer. Else it would calculate the size
 *                     for each DMA transfer.
 * @param mode         specifies whether this is READ or WRITE operation
 * @return This function returns a negative number on error if buffer could not
 *         be added with DMA for transfer. On Success, it returns 0
 */
int mxc_dma_sg_config(int channel_num, struct scatterlist *sg,
		      int num_buf, int num_of_bytes, mxc_dma_mode_t mode)
{
	int ret, i;
	mxc_dma_requestbuf_t *dma_buf;
	gfp_t gfp_flags = in_atomic() ? GFP_ATOMIC : GFP_KERNEL;
	mxc_dma_channel_t *chnl_info;
	mxc_dma_channel_private_t *data_priv;

	DBG(0, "%s: chan %d %u buffers with %u bytes\n", __FUNCTION__,
		channel_num, num_buf, num_of_bytes);

	if (channel_num < 0 || channel_num >= MAX_DMA_CHANNELS) {
		return -EINVAL;
	}

	if (!mxc_sdma_channels[channel_num].lock) {
		return -EINVAL;
	}

	dma_buf = kmalloc(num_buf * sizeof(mxc_dma_requestbuf_t),
			gfp_flags);
	if (dma_buf == NULL) {
		return -ENOMEM;
	}

	chnl_info = &mxc_sdma_channels[channel_num];
	data_priv = chnl_info->private;
	data_priv->intr_after_every_bd = 1;
	data_priv->circular = 1;

	for (i = 0; i < num_buf; i++) {
		if (mode == MXC_DMA_MODE_READ) {
			dma_buf[i].dst_addr = sg->dma_address;
			dma_buf[i].src_addr = DMA_ADDR_INVALID;
		} else {
			dma_buf[i].dst_addr = DMA_ADDR_INVALID;
			dma_buf[i].src_addr = sg->dma_address;
		}

		if ((num_of_bytes > sg->length) || (num_of_bytes == 0)) {
			dma_buf[i].num_of_bytes = sg->length;
		} else {
			dma_buf[i].num_of_bytes = num_of_bytes;
		}
		sg = sg_next(sg);
		if (num_of_bytes != 0)
			num_of_bytes -= dma_buf[i].num_of_bytes;
		DBG(0, "%s: desc[%d/%d] src: %08x dst: %08x len: %08x\n",
			__FUNCTION__, i, num_buf, dma_buf[i].src_addr, dma_buf[i].dst_addr,
			dma_buf[i].num_of_bytes);
	}

	ret = mxc_dma_config(channel_num, dma_buf, num_buf, mode);
	kfree(dma_buf);
	return ret;
}

/*!
 * This function is provided if the driver would like to set/change its
 * callback function.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param callback     a callback function to provide notification on transfer
 *                     completion, user could specify NULL if he does not wish
 *                     to be notified
 * @param arg          an argument that gets passed in to the callback
 *                     function, used by the user to do any driver specific
 *                     operations.
 * @return this function returns a negative number on error if the callback
 *         could not be set for the channel or 0 on success
 */
int mxc_dma_callback_set(int channel_num,
			 mxc_dma_callback_t callback, void *arg)
{
	if (channel_num < 0 || channel_num >= MAX_DMA_CHANNELS) {
		return -EINVAL;
	}

	if (!mxc_sdma_channels[channel_num].lock) {
		return -EINVAL;
	}

	mxc_sdma_channels[channel_num].cb_fn = callback;
	mxc_sdma_channels[channel_num].cb_args = arg;

	mxc_dma_set_callback(channel_num, mxc_dma_chnl_callback,
			     (void *)channel_num);

	return 0;
}

/*!
 * This stops the DMA channel and any ongoing transfers. Subsequent use of
 * mxc_dma_enable() will restart the channel and restart the transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_disable(int channel_num)
{
	if (channel_num < 0 || channel_num >= MAX_DMA_CHANNELS) {
		return -EINVAL;
	}

	if (!mxc_sdma_channels[channel_num].lock) {
		return -EINVAL;
	}

	mxc_dma_stop(channel_num);
	return 0;
}

/*!
 * This starts DMA transfer. Or it restarts DMA on a stopped channel
 * previously stopped with mxc_dma_disable().
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_enable(int channel_num)
{
	if (channel_num < 0 || channel_num >= MAX_DMA_CHANNELS) {
		return -EINVAL;
	}

	if (!mxc_sdma_channels[channel_num].lock) {
		return -EINVAL;
	}

	mxc_dma_start(channel_num);
	return 0;
}

/*!
 * Initializes dma structure with dma_operations
 *
 * @param   dma           dma structure
 * @return  returns 0 on success
 */
static int __init mxc_dma_init(void)
{
	int i;

	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		mxc_sdma_channels[i].active = 0;
		mxc_sdma_channels[i].lock = 0;
		mxc_sdma_channels[i].curr_buf = 0;
		mxc_sdma_channels[i].dynamic = 1;
		mxc_sdma_private[i].buf_tail = 0;
		mxc_sdma_channels[i].private = &mxc_sdma_private[i];
	}
	/*
	 * Make statically allocated channels unavailable for dynamic channel
	 * requests
	 */
	mxc_get_static_channels(mxc_sdma_channels);

	return 0;
}

arch_initcall(mxc_dma_init);

#else
int mxc_request_dma(int *channel, const char *devicename)
{
	return -ENODEV;
}

int mxc_dma_setup_channel(int channel, dma_channel_params * p)
{
	return -ENODEV;
}

int mxc_dma_set_channel_priority(unsigned int channel, unsigned int priority)
{
	return -ENODEV;
}

int mxc_dma_set_config(int channel, dma_request_t * p, int bd_index)
{
	return -ENODEV;
}

int mxc_dma_get_config(int channel, dma_request_t * p, int bd_index)
{
	return -ENODEV;
}

int mxc_dma_start(int channel)
{
	return -ENODEV;
}

int mxc_dma_stop(int channel)
{
	return -ENODEV;
}

void mxc_free_sdma(int channel)
{
}

void mxc_dma_set_callback(int channel, dma_callback_t callback, void *arg)
{
}

void *sdma_malloc(size_t size)
{
	return NULL;
}

void sdma_free(void *buf)
{
}

void *sdma_phys_to_virt(dma_addr_t buf)
{
	return NULL;
}

dma_addr_t sdma_virt_to_phys(void *buf)
{
	return DMA_ADDR_INVALID;
}

int mxc_dma_request(mxc_dma_device_t channel_id, const char *dev_name)
{
	return -ENODEV;
}

int mxc_dma_free(int channel_num)
{
	return -ENODEV;
}

int mxc_dma_config(int channel_num, mxc_dma_requestbuf_t * dma_buf,
		   int num_buf, mxc_dma_mode_t mode)
{
	return -ENODEV;
}

int mxc_dma_sg_config(int channel_num, struct scatterlist *sg,
		      int num_buf, int num_of_bytes, mxc_dma_mode_t mode)
{
	return -ENODEV;
}

int mxc_dma_callback_set(int channel_num, mxc_dma_callback_t callback,
			 void *arg)
{
	return -ENODEV;
}

int mxc_dma_disable(int channel_num)
{
	return -ENODEV;
}

int mxc_dma_enable(int channel_num)
{
	return -ENODEV;
}

EXPORT_SYMBOL(mxc_request_dma);
EXPORT_SYMBOL(mxc_dma_setup_channel);
EXPORT_SYMBOL(mxc_dma_set_channel_priority);
EXPORT_SYMBOL(mxc_dma_set_config);
EXPORT_SYMBOL(mxc_dma_get_config);
EXPORT_SYMBOL(mxc_dma_start);
EXPORT_SYMBOL(mxc_dma_stop);
EXPORT_SYMBOL(mxc_free_dma);
EXPORT_SYMBOL(mxc_dma_set_callback);
EXPORT_SYMBOL(sdma_malloc);
EXPORT_SYMBOL(sdma_free);
EXPORT_SYMBOL(sdma_phys_to_virt);
EXPORT_SYMBOL(sdma_virt_to_phys);

#endif

EXPORT_SYMBOL(mxc_dma_request_ext);
EXPORT_SYMBOL(mxc_dma_free);
EXPORT_SYMBOL(mxc_dma_config);
EXPORT_SYMBOL(mxc_dma_sg_config);
EXPORT_SYMBOL(mxc_dma_callback_set);
EXPORT_SYMBOL(mxc_dma_disable);
EXPORT_SYMBOL(mxc_dma_enable);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Linux SDMA API");
MODULE_LICENSE("GPL");
