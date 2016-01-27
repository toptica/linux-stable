/*
 * V4L2 Driver for MX27 camera host
 *
 * Copyright (C) 2008, Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/clk.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>

#include <linux/videodev2.h>

#include <asm/arch/camera.h>

#include <asm/arch/pmic/power.h>
#include <asm/arch/imx-dma.h>
#include <asm/arch/dma.h>

#include <asm/dma.h>

#define MX27_CAM_DRV_NAME "mx27-camera"
#define MX27_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5) /* FIXME: Whats this? */

static const char *mx27_cam_driver_description = "i.MX27_Camera";

/* reset values */
#define CSICR1_RESET_VAL	0x40000800
#define CSICR2_RESET_VAL	0x0
#define CSICR3_RESET_VAL	0x0

/* csi control reg 1 */
#define CSICR1_SWAP16_EN	(1 << 31)
#define CSICR1_EXT_VSYNC	(1 << 30)
#define CSICR1_EOF_INTEN	(1 << 29)
#define CSICR1_PRP_IF_EN	(1 << 28)
#define CSICR1_CCIR_MODE	(1 << 27)
#define CSICR1_COF_INTEN	(1 << 26)
#define CSICR1_SF_OR_INTEN	(1 << 25)
#define CSICR1_RF_OR_INTEN	(1 << 24)
#define CSICR1_STATFF_LEVEL	(3 << 22)
#define CSICR1_STATFF_INTEN	(1 << 21)
#define CSICR1_RXFF_LEVEL(l)	(((l) & 3) << 19)
#define CSICR1_RXFF_INTEN	(1 << 18)
#define CSICR1_SOF_POL		(1 << 17)
#define CSICR1_SOF_INTEN	(1 << 16)
#define CSICR1_MCLKDIV(d)	(((d) & 0xF) << 12)
#define CSICR1_HSYNC_POL	(1 << 11)
#define CSICR1_CCIR_EN		(1 << 10)
#define CSICR1_MCLKEN		(1 << 9)
#define CSICR1_FCC		(1 << 8)
#define CSICR1_PACK_DIR		(1 << 7)
#define CSICR1_CLR_STATFIFO	(1 << 6)
#define CSICR1_CLR_RXFIFO	(1 << 5)
#define CSICR1_GCLK_MODE	(1 << 4)
#define CSICR1_INV_DATA		(1 << 3)
#define CSICR1_INV_PCLK		(1 << 2)
#define CSICR1_REDGE		(1 << 1)

#define SHIFT_STATFF_LEVEL	22
#define SHIFT_RXFF_LEVEL	19
#define SHIFT_MCLKDIV		12

/* control reg 3 */
#define CSICR3_FRMCNT		(0xFFFF << 16)
#define CSICR3_FRMCNT_RST	(1 << 15)
#define CSICR3_CSI_SUP		(1 << 3)
#define CSICR3_ZERO_PACK_EN	(1 << 2)
#define CSICR3_ECC_INT_EN	(1 << 1)
#define CSICR3_ECC_AUTO_EN	(1 << 0)

#define SHIFT_FRMCNT		16

/* csi status reg */
#define CSISR_SFF_OR_INT	(1 << 25)
#define CSISR_RFF_OR_INT	(1 << 24)
#define CSISR_STATFF_INT	(1 << 21)
#define CSISR_RXFF_INT		(1 << 18)
#define CSISR_EOF_INT		(1 << 17)
#define CSISR_SOF_INT		(1 << 16)
#define CSISR_F2_INT		(1 << 15)
#define CSISR_F1_INT		(1 << 14)
#define CSISR_COF_INT		(1 << 13)
#define CSISR_ECC_INT		(1 << 1)
#define CSISR_DRDY		(1 << 0)

#define CSICR1		0x00
#define CSICR2		0x04
#define CSISR		0x08
#define CSISTATFIFO	0x0c
#define CSIRFIFO	0x10
#define CSIRXCNT	0x14
#define CSICR3		0x1C

/* Currently we do not need irqs. All we need is DMA callback
 * Leave it here for reference for some time.
 */
#undef MX27_CAMERA_USE_IRQ

struct mx27_camera_dev {
	struct device		*dev;
	struct soc_camera_device *icd;
	struct clk		*clk;

	unsigned int		irq;
	void __iomem		*base;
	unsigned int		dma_chan_y;

	struct mx27_camera_platform_data *pdata;
	struct resource		*res;
	unsigned long		platform_flags;

	struct list_head	capture;

	spinlock_t		lock;

	imx_dmach_t		dma;
	struct mx27_buffer	*active;
};

/* buffer for one video frame */
struct mx27_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;

	const struct soc_camera_data_format        *fmt;
};

static DEFINE_MUTEX(camera_lock);

static int mx27_camera_set_capture_format(struct soc_camera_device *icd,
					 __u32 pixfmt, struct v4l2_rect *rect)
{
	struct soc_camera_host *ici =
		to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;
	int ret;

	icd->y_skip_top = 0;
	icd->y_current = 0;

	ret = icd->ops->set_capture_format(icd, pixfmt, rect,
		 IS_DATAWIDTH_8 | /* FIXME: only possibility, but check */
		 IS_MASTER |      /* FIXME: only possibility, but check */
		 IS_VSYNC_ACTIVE_HIGH |
		(pcdev->platform_flags & MX27_CAMERA_HSYNC_HIGH ?
		 IS_HSYNC_ACTIVE_HIGH : 0) |
		(pcdev->platform_flags & MX27_CAMERA_PCLK_SAMPLE_RISING ?
		 IS_PCLK_SAMPLE_RISING : 0));
	if (ret < 0) {
		printk("icd->ops->set_capture_format failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int mclk_get_divisor(struct mx27_camera_dev *pcdev)
{
	dev_info(pcdev->dev, "%s not implemented. Running at max speed\n",
			__FUNCTION__);

#if 0
	unsigned int mclk = pcdev->pdata->clk;
	unsigned int pclk = clk_get_rate(pcdev->clk);
	int i;
printk("%s: %ld %ld\n", __FUNCTION__, mclk, pclk);
	for (i = 0; i < 0xf; i++)
		if ((i + 1) * 2 * mclk <= pclk)
			break;
	return i;
#endif
	return 0;
}

static void mx27_camera_activate(struct mx27_camera_dev *pcdev)
{
	u32 csicr1 = 0;

	if (pcdev->platform_flags & MX27_CAMERA_SWAP16)
		csicr1 |= CSICR1_SWAP16_EN;
	if (pcdev->platform_flags & MX27_CAMERA_EXT_VSYNC)
		csicr1 |= CSICR1_EXT_VSYNC;
	if (pcdev->platform_flags & MX27_CAMERA_CCIR)
		csicr1 |= CSICR1_CCIR_EN;
	if (pcdev->platform_flags & MX27_CAMERA_CCIR_INTERLACE)
		csicr1 |= CSICR1_CCIR_MODE;
	if (pcdev->platform_flags & MX27_CAMERA_HSYNC_HIGH)
		csicr1 |= CSICR1_HSYNC_POL;
	if (pcdev->platform_flags & MX27_CAMERA_GATED_CLOCK)
		csicr1 |= CSICR1_GCLK_MODE;
	if (pcdev->platform_flags & MX27_CAMERA_INV_DATA)
		csicr1 |= CSICR1_INV_DATA;
	if (pcdev->platform_flags & MX27_CAMERA_PCLK_SAMPLE_RISING)
		csicr1 |= CSICR1_INV_PCLK;

	csicr1 |= CSICR1_MCLKDIV(mclk_get_divisor(pcdev));

	csicr1 |= CSICR1_MCLKEN | CSICR1_RXFF_LEVEL(2);

	clk_enable(pcdev->clk);
	writel(csicr1, pcdev->base + CSICR1);
}

static void mx27_camera_deactivate(struct mx27_camera_dev *pcdev)
{
	clk_disable(pcdev->clk);
	writel(0, pcdev->base + CSICR1);
}

/* The following two functions absolutely depend on the fact, that
 * there can be only one camera on PXA quick capture interface */
static int mx27_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;
	int ret;

	mutex_lock(&camera_lock);

	if (pcdev->icd) {
		ret = -EBUSY;
		goto ebusy;
	}

	dev_info(&icd->dev, "Camera driver attached to camera %d\n",
		 icd->devnum);

	mx27_camera_activate(pcdev);
	ret = icd->ops->init(icd);

	if (!ret)
		pcdev->icd = icd;

ebusy:
	mutex_unlock(&camera_lock);

	return ret;
}

static void mx27_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	dev_info(&icd->dev, "Camera driver detached from camera %d\n",
		 icd->devnum);

	icd->ops->release(icd);

	mx27_camera_deactivate(pcdev);

	pcdev->icd = NULL;
}

#ifdef MX27_CAMERA_USE_IRQ
static irqreturn_t mx27_camera_irq(int irq, void *data)
{
	struct mx27_camera_dev *pcdev = data;
	u32 status = readl(pcdev->base + CSISR);
	unsigned long flags;

	return IRQ_HANDLED;
}
#endif

static unsigned int vid_limit = 16;	/* Video memory limit, in Mb */

/*
 *  Videobuf operations
 */
static int mx27_videobuf_setup(struct videobuf_queue *vq, unsigned int *count,
			      unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;

	dev_info(&icd->dev, "count=%d, size=%d\n", *count, *size);

	*size = icd->width * icd->height *
		((icd->current_fmt->depth + 7) >> 3);

	if (0 == *count)
		*count = 32;
	while (*size * *count > vid_limit * 1024 * 1024)
		(*count)--;

	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct mx27_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct videobuf_dmabuf *dma = videobuf_to_dma(&buf->vb);

	BUG_ON(in_interrupt());

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __FUNCTION__,
		&buf->vb, buf->vb.baddr, buf->vb.bsize);

	/* This waits until this buffer is out of danger, i.e., until it is no
	 * longer in STATE_QUEUED or STATE_ACTIVE */
	videobuf_waiton(&buf->vb, 0, 0);
	videobuf_dma_unmap(vq, dma);
	videobuf_dma_free(dma);

	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static int mx27_videobuf_prepare(struct videobuf_queue *vq,
		struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct mx27_buffer *buf = container_of(vb, struct mx27_buffer, vb);
	int ret = 0;

#ifdef DEBUG
	/* This can be useful if you want to see if we actually fill
	 * the buffer with something */
	memset((void *)vb->baddr, 0xaa, vb->bsize);
#endif

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __FUNCTION__,
		vb, vb->baddr, vb->bsize);

	if (buf->fmt	!= icd->current_fmt ||
	    vb->width	!= icd->width ||
	    vb->height	!= icd->height ||
	    vb->field	!= field) {
		buf->fmt	= icd->current_fmt;
		vb->width	= icd->width;
		vb->height	= icd->height;
		vb->field	= field;
		vb->state	= VIDEOBUF_NEEDS_INIT;
	}

	vb->size = vb->width * vb->height * ((buf->fmt->depth + 7) >> 3);
	if (0 != vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;

fail:
	free_buffer(vq, buf);
out:
	return ret;
}

extern void mxc_dump_dma_register(int channel);

static void mx27_videobuf_queue(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici =
		to_soc_camera_host(icd->dev.parent);
	struct mx27_camera_dev *pcdev = ici->priv;
	struct mx27_buffer *buf = container_of(vb, struct mx27_buffer, vb);
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);
	unsigned long flags;
	int ret;
	u32 tmp;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __FUNCTION__,
		vb, vb->baddr, vb->bsize);

	spin_lock_irqsave(&pcdev->lock, flags);

	list_add_tail(&vb->queue, &pcdev->capture);

	vb->state = VIDEOBUF_ACTIVE;

	if (!pcdev->active) {
		ret = imx_dma_setup_sg(pcdev->dma, dma->sglist,
				dma->sglen,
				vb->size,
				CSI_BASE_ADDR + 0x10,
				DMA_MODE_READ);
		if (ret) {
			vb->state = VIDEOBUF_ERROR;
			wake_up(&vb->done);
			goto out;
		}
		pcdev->active = buf;
		writel(CSISR_SOF_INT, pcdev->base + CSISR);
		while(!(readl(pcdev->base + CSISR) & CSISR_SOF_INT));
		tmp = readl(pcdev->base + CSICR1) | CSICR1_CLR_RXFIFO;
		writel(tmp, pcdev->base + CSICR1);
		imx_dma_enable(pcdev->dma);
	}

	writel(0x100, pcdev->base + CSIRXCNT);

	dev_dbg(&icd->dev, "nents=%d sg=0x%p\n",
		dma->sglen, dma->sglist);
out:
	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void mx27_videobuf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	struct mx27_buffer *buf = container_of(vb, struct mx27_buffer, vb);

#ifdef DEBUG
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __FUNCTION__,
		vb, vb->baddr, vb->bsize);

	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		dev_info(&icd->dev, "%s (active)\n", __FUNCTION__);
		break;
	case VIDEOBUF_QUEUED:
		dev_info(&icd->dev, "%s (queued)\n", __FUNCTION__);
		break;
	case VIDEOBUF_PREPARED:
		dev_info(&icd->dev, "%s (prepared)\n", __FUNCTION__);
		break;
	default:
		dev_info(&icd->dev, "%s (unknown) %d\n", __FUNCTION__,
				vb->state);
		break;
	}
#endif

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops mx27_videobuf_ops = {
	.buf_setup      = mx27_videobuf_setup,
	.buf_prepare    = mx27_videobuf_prepare,
	.buf_queue      = mx27_videobuf_queue,
	.buf_release    = mx27_videobuf_release,
};

static int mx27_camera_try_fmt_cap(struct soc_camera_host *ici,
				  struct v4l2_format *f)
{
	printk("%s\n", __FUNCTION__);

	return 0;
}

static int mx27_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->card, mx27_cam_driver_description, sizeof(cap->card));
	cap->version = MX27_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int mx27_camera_reqbufs(struct soc_camera_file *icf,
			      struct v4l2_requestbuffers *p)
{
	int i;

	for (i = 0; i < p->count; i++) {
		struct mx27_buffer *buf = container_of(icf->vb_vidq.bufs[i],
						      struct mx27_buffer, vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

static void mx27_camera_frame_done(struct mx27_camera_dev *pcdev, int state)
{
	struct videobuf_buffer *vb;
	struct mx27_buffer *buf;
	struct videobuf_dmabuf *dma;
	int ret;

	if (!pcdev->active) {
		dev_err(pcdev->dev, "DMA End IRQ with no active buffer!\n");
		return;
	}

	vb = &pcdev->active->vb;
	buf = container_of(vb, struct mx27_buffer, vb);
	WARN_ON(list_empty(&vb->queue));
	dev_dbg(pcdev->dev, "%s (vb=0x%p) 0x%08lx %d\n", __FUNCTION__,
		vb, vb->baddr, vb->bsize);

	/* _init is used to debug races, see comment in pxa_camera_reqbufs() */
	list_del_init(&vb->queue);
	vb->state = state;
	do_gettimeofday(&vb->ts);
	vb->field_count++;

	wake_up(&vb->done);

	if (list_empty(&pcdev->capture)) {
		pcdev->active = NULL;
		return;
	}

	pcdev->active = list_entry(pcdev->capture.next, struct mx27_buffer,
				   vb.queue);

	vb = &pcdev->active->vb;
	vb->state = VIDEOBUF_ACTIVE;
	dma = videobuf_to_dma(vb);

	ret = imx_dma_setup_sg(pcdev->dma, dma->sglist,
			dma->sglen,
			vb->size, CSI_BASE_ADDR + 0x10,
			DMA_MODE_READ);
	if (ret) {
		vb->state = VIDEOBUF_ERROR;
		wake_up(&vb->done);
		return;
	}

	imx_dma_enable(pcdev->dma);
}

static void mx27_camera_dma_err_callback(int channel, void *data, int err)
{
	struct mx27_camera_dev *pcdev = data;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	mx27_camera_frame_done(pcdev, VIDEOBUF_ERROR);

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void mx27_camera_dma_callback(int channel, void *data)
{
	struct mx27_camera_dev *pcdev = data;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	mx27_camera_frame_done(pcdev, VIDEOBUF_DONE);

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static unsigned int mx27_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct mx27_buffer *buf;

	buf = list_entry(icf->vb_vidq.stream.next, struct mx27_buffer,
			 vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE ||
	    buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

/* Should beallocated dynamically too, but we have only one. */
static struct soc_camera_host mx27_soc_camera_host = {
	.drv_name		= MX27_CAM_DRV_NAME,
	.vbq_ops		= &mx27_videobuf_ops,
	.add			= mx27_camera_add_device,
	.remove			= mx27_camera_remove_device,
	.msize			= sizeof(struct mx27_buffer),
	.set_capture_format	= mx27_camera_set_capture_format,
	.try_fmt_cap		= mx27_camera_try_fmt_cap,
	.reqbufs		= mx27_camera_reqbufs,
	.poll			= mx27_camera_poll,
	.querycap		= mx27_camera_querycap,
};

static int mx27_camera_probe(struct platform_device *pdev)
{
	struct mx27_camera_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	unsigned int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || !irq) {
		dev_err(&pdev->dev, "No platform irq\n");
		err = -ENODEV;
		goto exit;
	}

	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	pcdev->clk = clk_get(&pdev->dev, "csi_perclk");
	if (IS_ERR(pcdev->clk)) {
		err = PTR_ERR(pcdev->clk);
		goto exit_kfree;
	}

	dev_info(&pdev->dev, "Camera clock frequency: %ld\n",
			clk_get_rate(pcdev->clk));

	/* Initialize DMA */
	err = imx_dma_request_by_prio(&pcdev->dma, "CSI RX DMA",
			DMA_PRIO_HIGH);
	if (err) {
		dev_err(&pdev->dev,
			"mxc_v4l_read failed to request DMA channel\n");
		err = -EIO;
		goto exit_clk_put;
	}

	err = imx_dma_setup_handlers(pcdev->dma, mx27_camera_dma_callback,
					mx27_camera_dma_err_callback, pcdev);
	if (err != 0) {
		dev_err(&pdev->dev,
				"mxc_v4l_read failed to set DMA callback\n");
		err = -EIO;
		goto exit_dma_free;
	}

	imx_dma_config_channel(pcdev->dma,
			IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_FIFO,
			IMX_DMA_MEMSIZE_32 | IMX_DMA_TYPE_LINEAR,
			DMA_REQ_CSI_RX, 1);

	imx_dma_config_burstlen(pcdev->dma, 64);

	dev_set_drvdata(&pdev->dev, pcdev);
	pcdev->res = res;

	pcdev->pdata = pdev->dev.platform_data;
	pcdev->platform_flags = pcdev->pdata->flags;

	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);

	/*
	 * Request the regions.
	 */
	if (!request_mem_region(res->start, res->end - res->start + 1,
				MX27_CAM_DRV_NAME)) {
		err = -EBUSY;
		goto exit_dma_free;
	}

	base = ioremap(res->start, res->end - res->start + 1);
	if (!base) {
		err = -ENOMEM;
		goto exit_release;
	}
	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->dev = &pdev->dev;

	pcdev->pdata->init(pdev);

#ifdef MX27_CAMERA_USE_IRQ
	err = request_irq(pcdev->irq, mx27_camera_irq, 0, MX27_CAM_DRV_NAME,
			  pcdev);
	if (err) {
		dev_err(pcdev->dev, "Camera interrupt register failed \n");
		goto exit_iounmap;
	}
#endif

	mx27_soc_camera_host.priv	= pcdev;
	mx27_soc_camera_host.dev.parent	= &pdev->dev;
	mx27_soc_camera_host.nr		= pdev->id;
	err = soc_camera_host_register(&mx27_soc_camera_host, THIS_MODULE);
	if (err)
		goto exit_free_irq;

	return 0;

exit_free_irq:
#ifdef MX27_CAMERA_USE_IRQ
	free_irq(pcdev->irq, pcdev);
exit_iounmap:
#endif
	iounmap(base);
exit_release:
	release_mem_region(res->start, res->end - res->start + 1);
exit_dma_free:
	imx_dma_free(pcdev->dma);
exit_clk_put:
	clk_put(pcdev->clk);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int __devexit mx27_camera_remove(struct platform_device *pdev)
{
	struct mx27_camera_dev *pcdev = platform_get_drvdata(pdev);
	struct resource *res;

	clk_put(pcdev->clk);

	imx_dma_free(pcdev->dma);
#ifdef MX27_CAMERA_USE_IRQ
	free_irq(pcdev->irq, pcdev);
#endif
	soc_camera_host_unregister(&mx27_soc_camera_host);

	iounmap(pcdev->base);

	pcdev->pdata->exit(pdev);

	res = pcdev->res;
	release_mem_region(res->start, res->end - res->start + 1);

	kfree(pcdev);

	dev_info(&pdev->dev, "MX27 Camera driver unloaded\n");

	return 0;
}

static struct platform_driver mx27_camera_driver = {
	.driver 	= {
		.name	= MX27_CAM_DRV_NAME,
	},
	.probe		= mx27_camera_probe,
	.remove		= __exit_p(mx27_camera_remove),
};


static int __devinit mx27_camera_init(void)
{
	return platform_driver_register(&mx27_camera_driver);
}

static void __exit mx27_camera_exit(void)
{
	return platform_driver_unregister(&mx27_camera_driver);
}

module_init(mx27_camera_init);
module_exit(mx27_camera_exit);

MODULE_DESCRIPTION("i.MX27 SoC Camera Host driver");
MODULE_AUTHOR("Sascha Hauer <sha@pengutronix.de>");
MODULE_LICENSE("GPL");
