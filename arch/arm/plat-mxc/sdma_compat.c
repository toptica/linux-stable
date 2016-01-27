/*
 * SDMA API compatibility functions
 */
#ifndef DEBUG
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/io.h>

#include <mach/dma-mx1-mx2.h>
#include <mach/dma.h>
#include <mach/imx_ssi.h>

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


static mxc_dma_callback_t sdma_compat_callbacks[IMX_DMA_CHANNELS];
int imx_dma_get_count(int channel, int *count);

static void sdma_compat_callback(int channel, void *data)
{
	int ret;
	int count;

	if ((u_int)channel >= IMX_DMA_CHANNELS)
		return;
	if (sdma_compat_callbacks[channel] == NULL)
		return;
	ret = imx_dma_get_count(channel, &count);
	if (ret) {
		printk(KERN_ERR "Failed to get DMA count: %d\n", ret);
	} else {
		DBG(0, "%s: DMA count=%d\n", __FUNCTION__, count);
		sdma_compat_callbacks[channel](data, 0, count);
	}
}

static void sdma_compat_err_callback(int channel, void *data, int err)
{
	if ((u_int)channel >= IMX_DMA_CHANNELS)
		return;
	if (sdma_compat_callbacks[channel] == NULL)
		return;

	sdma_compat_callbacks[channel](data, err, 0);
}

static void sdma_prog_callback(int channel, void *data, struct scatterlist *sg)
{
	if ((u_int)channel >= IMX_DMA_CHANNELS)
		return;
	if (sdma_compat_callbacks[channel] == NULL)
		return;
	sdma_compat_callbacks[channel](data, 0, sg->length);
}

int mxc_dma_callback_set(int channel, mxc_dma_callback_t callback,
			void *arg)
{
	int ret;

	if ((u_int)channel >= IMX_DMA_CHANNELS) {
		printk(KERN_ERR "Invalid DMA channel %d\n", channel);
		return -EINVAL;
	}

	DBG(0, "%s: Registering SDMA callback %p for channel %d\n", __FUNCTION__,
		callback, channel);
	sdma_compat_callbacks[channel] = callback;
	ret = imx_dma_setup_handlers(channel, sdma_compat_callback,
				sdma_compat_err_callback, arg);
#if 1
	if (ret == 0) {
		ret = imx_dma_setup_progression_handler(channel, sdma_prog_callback);
	}
#endif
	return ret;
}
EXPORT_SYMBOL(mxc_dma_callback_set);

/*
  imx_dma_setup_sg(int channel,
		struct scatterlist *sg, unsigned int sgcount,
		unsigned int dma_length, unsigned int dev_addr,
		unsigned int dmamode)
*/
static unsigned int imx_dma_dev_addrs[IMX_DMA_CHANNELS];

int mxc_dma_sg_config(int channel, struct scatterlist *sg, int num_sg,
		int num_bytes, mxc_dma_mode_t mode)
{
	return imx_dma_setup_sg(channel, sg, num_sg,
				num_bytes == 0 ? MXC_DMA_SIZE_UNLIMITED : num_bytes,
				imx_dma_dev_addrs[channel],
				mode == MXC_DMA_MODE_READ ? DMA_MODE_READ : DMA_MODE_WRITE);
}
EXPORT_SYMBOL(mxc_dma_sg_config);

int mxc_dma_request_ext(mxc_dma_device_t channel_id, const char *dev_name,
			struct dma_channel_info *info)
{
	int ret;
	unsigned int config_port;
	unsigned int config_mem;
	unsigned int dmareq;
	unsigned int dev_addr;
	unsigned int burstlen;
	int channel;

	DBG(0, "%s: Requesting SDMA id %d\n", __FUNCTION__,
		channel_id);
	switch (channel_id) {
	case MXC_DMA_SSI1_16BIT_TX0:
		dmareq = DMA_REQ_SSI1_TX0;
		dev_addr = SSI1_BASE_ADDR + SSI_STX0;
		config_port = IMX_DMA_MEMSIZE_16 | IMX_DMA_TYPE_FIFO;
		config_mem = IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR;
		burstlen = 8;
		break;

	case MXC_DMA_SSI1_16BIT_RX0:
		dmareq = DMA_REQ_SSI1_RX0;
		dev_addr = SSI1_BASE_ADDR + SSI_SRX0;
		config_port = IMX_DMA_MEMSIZE_16 | IMX_DMA_TYPE_FIFO;
		config_mem = IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR;
		burstlen = 8;
		break;

	case MXC_DMA_SSI2_16BIT_TX0:
		dmareq = DMA_REQ_SSI2_TX0;
		dev_addr = SSI2_BASE_ADDR + SSI_STX0;
		config_port = IMX_DMA_MEMSIZE_16 | IMX_DMA_TYPE_FIFO;
		config_mem = IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR;
		burstlen = 8;
		break;

	case MXC_DMA_SSI2_16BIT_RX0:
		dmareq = DMA_REQ_SSI2_RX0;
		dev_addr = SSI2_BASE_ADDR + SSI_SRX0;
		config_port = IMX_DMA_MEMSIZE_16 | IMX_DMA_TYPE_FIFO;
		config_mem = IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR;
		burstlen = 8;
		break;

	default:
		return -EINVAL;
	}

	DBG(0, "%s: Requesting DMA channel\n", __FUNCTION__);
	channel = imx_dma_request_by_prio(dev_name, DMA_PRIO_HIGH);
	if (channel < 0) {
		printk(KERN_ERR "%s: Failed to request DMA channel\n", __FUNCTION__);
		return channel;
	}
	DBG(0, "%s: Got DMA channel %d\n", __FUNCTION__, channel);

	DBG(0, "%s: Configuring DMA channel %d for DMA req %d dev_addr %08x\n", __FUNCTION__,
		channel, dmareq, dev_addr);
	ret = imx_dma_config_channel(channel, config_port, config_mem, dmareq, 1);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to configure DMA channel\n", __FUNCTION__);
		imx_dma_free(channel);
		return ret;
	}
	imx_dma_config_burstlen(channel, burstlen);
	imx_dma_dev_addrs[channel] = dev_addr;
	return channel;
}
EXPORT_SYMBOL(mxc_dma_request_ext);

int mxc_dma_disable(int channel)
{
	DBG(0, "%s: Disabling DMA channel %d\n", __FUNCTION__, channel);
	imx_dma_disable(channel);
	return 0;
}
EXPORT_SYMBOL(mxc_dma_disable);

int mxc_dma_enable(int channel)
{
	DBG(0, "%s: Enabling DMA channel %d\n", __FUNCTION__, channel);
	imx_dma_enable(channel);
	return 0;
}
EXPORT_SYMBOL(mxc_dma_enable);

int mxc_dma_free(int channel)
{
	DBG(0, "%s: Freeing DMA channel %d\n", __FUNCTION__, channel);
	imx_dma_free(channel);
	return 0;
}
EXPORT_SYMBOL(mxc_dma_free);
