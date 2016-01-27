/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * adapted for 2.6.31 by <LW@KARO-electronics.de>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * MX27 V4L2 Video Output Driver
 *
 * Video4Linux2 Output Device using MX27 eMMA Post-processing functionality.
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>

#include <media/v4l2-ioctl.h>

#include "mxc_v4l2_output.h"
#include "mx27_pp.h"

//#define SDC_FG_FB_FORMAT	V4L2_PIX_FMT_RGB565
#define SDC_FG_FB_FORMAT	V4L2_PIX_FMT_RGB32

static int video_nr = 16;
module_param(video_nr, int, S_IRUGO);

static struct v4l2_output mxc_outputs[] = {
	{
		.index = 0,
		.name = "DISP%d Video Out",
		.type = V4L2_OUTPUT_TYPE_ANALOG,	/* not really correct,
						   but no other choice */
		.audioset = 0,
		.modulator = 0,
		.std = V4L2_STD_UNKNOWN,
	},
};

#ifdef DEBUG
int debug = 1;
#define dbg_lvl(n)	((n) < debug)
module_param(debug, int, S_IRUGO | S_IWUSR);

#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#else
int debug;
#define dbg_lvl(n)	0
#define DBG(lvl, fmt...)	do { } while (0)
module_param(debug, int, 0);
#endif


/* debug counters */
static uint32_t g_irq_cnt;
static uint32_t g_buf_output_cnt;
static uint32_t g_buf_q_cnt;
static uint32_t g_buf_dq_cnt;
static uint32_t g_paused_cnt;
static uint32_t g_busy_cnt;
static uint32_t g_late_cnt;

static int dq_intr_cnt;

static int using_default_fps = -1;

static inline unsigned long diff_usec(struct timeval *t1, struct timeval *t0)
{
	return (t1->tv_sec - t0->tv_sec) * 1000000 + t1->tv_usec - t0->tv_usec;
}

#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
static int fb_event_notify(struct notifier_block *self,
			   unsigned long action, void *data)
{
	vout_data *vout = container_of(self, vout_data, fb_event_notifier);
	struct fb_event *event = data;
	struct fb_info *info = event->info;
	unsigned long flags;
	int blank, i;

	for (i = 0; i < num_registered_fb; i++)
		if (registered_fb[i] == info)
			break;

	/*
	 * Check if the event is sent by the framebuffer in which
	 * the video is displayed.
	 */
	if (i != vout->output_fb)
		return 0;

	switch (action) {
	case FB_EVENT_BLANK:
		DBG(0, "%s: BLANK\n", __FUNCTION__);
		blank = *(int *)event->data;
		down(&vout->user_lock);
		vout->fb_enabled = !blank;
		if (blank && vout->pp_ready) {
			if (pp_enable(1))
				dev_warn(dev, "unable to enable PP\n");
			vout->pp_ready = 0;
		}
		up(&vout->user_lock);
		break;

	case FB_EVENT_MXC_EOF:
		down(&vout->user_lock);
		vout->fb_enabled = 1;
		if (vout->pp_ready) {
			DBG(2, "%s: enable PP\n", __FUNCTION__);
			if (pp_enable(1))
				dev_warn(dev, "unable to enable PP\n");
			vout->pp_ready = 0;
		}
		up(&vout->user_lock);
		break;
	}

	return 0;
}
#endif

static inline struct v4l_queue *find_buffer(struct list_head *list, int index)
{
	struct v4l_queue *q;

	list_for_each_entry(q, list, head) {
		if (q->buf.index == index) {
			return q;
		}
	}
	return NULL;
}

static __inline unsigned long get_jiffies(struct timeval *t)
{
	struct timeval cur;

	do_gettimeofday(&cur);
	if ((t->tv_sec < cur.tv_sec) ||
	    ((t->tv_sec == cur.tv_sec) && (t->tv_usec < cur.tv_usec))) {
		DBG(0, "%s: timestamp is %luµs in the past\n", __FUNCTION__,
			diff_usec(&cur, t));
		g_late_cnt++;
		return jiffies;
	}

	if (t->tv_usec < cur.tv_usec) {
		cur.tv_sec = t->tv_sec - cur.tv_sec - 1;
		cur.tv_usec = t->tv_usec + 1000000 - cur.tv_usec;
	} else {
		cur.tv_sec = t->tv_sec - cur.tv_sec;
		cur.tv_usec = t->tv_usec - cur.tv_usec;
	}

	return jiffies + timeval_to_jiffies(&cur);
}

/* must be called with irq_lock held */
static void mxc_v4l2out_start_pp(vout_data *vout)
{
	struct device *dev = &vout->video_dev->dev;

	BUG_ON(vout->active == NULL);
	DBG(1, "%s: Updating HW PTR to %08x\n", __FUNCTION__,
	    vout->active->dma_desc.dma_addr);
	do_gettimeofday(&vout->frame_start);
	if (pp_ptr(vout->active->dma_desc.dma_addr, vout->offset)) {
		dev_warn(dev, "%s: unable to update buffer\n", __FUNCTION__);
		return;
	}
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	if (vout->tear_protection == TEARING_PROTECTION_ACTIVE) {
		if (vout->fb_enabled && (vout->v4l2_fb.flags != V4L2_FBUF_FLAG_OVERLAY)) {
			vout->pp_ready = 1;
		} else {
			pp_enable(1);
		}
	} else {
		pp_enable(1);
	}
#else
	pp_enable(1);
#endif
	vout->busy = 1;
}

static __inline void mxc_v4l2out_schedule_frame(vout_data *vout,
						struct timeval *tv, int init)
{
	unsigned long timeout;

	DBG(1, "%s: ts %6lu.%06lu\n", __FUNCTION__, tv->tv_sec, tv->tv_usec);
	if (tv->tv_sec == 0 && tv->tv_usec == 0) {
		if (using_default_fps != 1) {
			DBG(0, "%s: using default fps\n", __FUNCTION__);
			using_default_fps = 1;
		}
		mxc_v4l2out_start_pp(vout);
		return;
	} else {
		timeout = get_jiffies(tv);
		if (using_default_fps != 0) {
			struct timeval now;

			do_gettimeofday(&now);

			DBG(0, "%s: using timestamp %lu.%06lu (now: %lu.%06lu)\n",
				__FUNCTION__, tv->tv_sec, tv->tv_usec,
				now.tv_sec, now.tv_usec);
			using_default_fps = 0;
		}
	}
#ifdef DEBUG
	{
		unsigned long cur = jiffies;
		static unsigned long last_tv;
		static unsigned int last_fps;
		static unsigned int last_avg;
		static unsigned int fps_avg;
		static unsigned int avg_count;

		if (vout->frame_count == 0) {
			avg_count = 0;
		}
		if (avg_count++ == 0) {
			last_tv = timeout;
			fps_avg = 0;
			last_fps = 0;
		} else {
			unsigned long t = timeout;
			unsigned int fps;

			DBG(3, "%s: t=%lu last=%lu diff=%ld\n", __FUNCTION__,
				t, last_tv, t - last_tv);
			if (t - last_tv) {
				fps = HZ / (t - last_tv);
				fps_avg = ((fps_avg * (avg_count - 1)) + fps) / avg_count;
				last_tv = t;
				if (last_fps != fps || last_avg != fps_avg) {
					DBG(1, "%s: FPS: %u AVG %u\n", __FUNCTION__, fps, fps_avg);
					last_fps = fps;
					last_avg = fps_avg;
				}
			} else {
				avg_count--;
			}
		}

		DBG(1, "%s: frame %u start %lu cur %lu timeout %lu\n", __FUNCTION__,
			vout->frame_count, vout->start_jiffies, cur, timeout);
	}
#endif
	if (!time_before(jiffies, timeout)) {
		DBG(0, "%s: timeout already expired: %lu >= %lu\n", __FUNCTION__,
			jiffies, timeout);
		mxc_v4l2out_start_pp(vout);
		return;
	}
	if (init) {
		vout->output_timer.expires = timeout;
		add_timer(&vout->output_timer);
	} else {
		WARN_ON(mod_timer(&vout->output_timer, timeout));
	}
}

/*
 * Private function to free buffers
 *
 * @param bufs_paddr	Array of physical address of buffers to be freed
 *
 * @param bufs_vaddr	Array of virtual address of buffers to be freed
 *
 * @param num_buf	Number of buffers to be freed
 *
 * @param size		Size for each buffer to be free
 *
 * @return status  0 success.
 */
static void mxc_free_buffers(vout_data *vout, struct list_head *list)
{
	struct v4l_queue *q;
	struct v4l_queue *tmp;
	struct device *dev = vout->video_dev->parent;
	unsigned long flags;

	list_for_each_entry(q, list, head) {
		DBG(0, "%s: dma_addr: %08x cpu_addr: %p size %08x\n", __FUNCTION__,
		    q->dma_desc.dma_addr, q->dma_desc.cpu_addr, q->dma_desc.size);
	}
	list_for_each_entry_safe(q, tmp, list, head) {
		struct dma_buf_desc *dma = &q->dma_desc;
		DBG(0, "%s: dma_addr: %08x cpu_addr: %p size %08x\n", __FUNCTION__,
		    dma->dma_addr, dma->cpu_addr, dma->size);
		if (dma->cpu_addr != NULL) {
			dma_free_coherent(dev,
					  dma->size, dma->cpu_addr,
					  dma->dma_addr);
			DBG(2, "freed @ paddr=0x%08x\n", dma->dma_addr);
		}
		spin_lock_irqsave(&vout->irq_lock, flags);
		list_del(&q->head);
		spin_unlock_irqrestore(&vout->irq_lock, flags);
		kfree(q);
	}
}

/*
 * Private function to allocate buffers
 *
 * @param bufs_paddr	Output array of physical address of buffers allocated
 *
 * @param bufs_vaddr	Output array of virtual address of buffers allocated
 *
 * @param num_buf	Input number of buffers to allocate
 *
 * @param size		Input size for each buffer to allocate
 *
 * @return status	-0 Successfully allocated a buffer, -ENOBUFS failed.
 */
static int mxc_allocate_buffers(vout_data *vout, struct list_head *list,
				int num_buf, int size)
{
	int i;
	unsigned long flags;
	struct device *dev = vout->video_dev->parent;

	DBG(2, "Trying to allocate %u buffers of %08x byte each\n",
	    num_buf, size);

	for (i = 0; i < num_buf; i++) {
		struct v4l_queue *q = kzalloc(sizeof(*q), GFP_KERNEL);
		struct dma_buf_desc *dma;

		if (q == NULL) {
			dev_err(dev, "failed to allocate memory for queue descriptor\n");
			return i;
		}
		DBG(2, "%s: queue header allocated %p\n", __FUNCTION__, q);
		q->buf.index = i;
		dma = &q->dma_desc;
		if (size != 0) {
			DBG(2, "Trying to allocate %08x byte DMA memory for queue %p\n",
			    size, list);
			dma->size = size;
			dma->cpu_addr = dma_alloc_coherent(dev,
							   size,
							   &dma->dma_addr,
							   GFP_KERNEL);
			if (dma->cpu_addr == NULL) {
				dev_err(dev, "dma_alloc_coherent failed for buffer %u out of %u\n",
				       i, num_buf);
				kfree(q);
				return i;
			}
			DBG(2, "allocated @ paddr=0x%08x, size=%d\n",
			    dma->dma_addr, size);
		}
		spin_lock_irqsave(&vout->irq_lock, flags);
		list_add_tail(&q->head, list);
		spin_unlock_irqrestore(&vout->irq_lock, flags);
	}
	return i;
}

static void mxc_v4l2out_timer_handler(unsigned long arg)
{
	unsigned long flags;
	vout_data *vout = (vout_data *)arg;
	struct device *dev = &vout->video_dev->dev;
	struct timeval now;

	do_gettimeofday(&now);

	DBG(1, "timer handler: %lu\n", jiffies);

	spin_lock_irqsave(&vout->irq_lock, flags);

	if (vout->state == STATE_STREAM_OFF && vout->active == NULL) {
		dev_warn(dev, "stream has stopped\n");
		spin_unlock_irqrestore(&vout->irq_lock, flags);
		return;
	}

	BUG_ON(vout->active == NULL);

	if (vout->busy) {
		dev_warn(dev, "Buffer %d is still busy in frame %u\n",
			vout->active->buf.index, vout->frame_count);
		g_busy_cnt++;
	}
	vout->frame_count++;
	mxc_v4l2out_start_pp(vout);

	spin_unlock_irqrestore(&vout->irq_lock, flags);
}

irqreturn_t mxc_v4l2out_pp_in_irq_handler(int irq, void *dev_id)
{
	unsigned long flags;
	vout_data *vout = dev_id;
	struct timeval now;

	spin_lock_irqsave(&vout->irq_lock, flags);

	do_gettimeofday(&now);
	DBG(2, "%s: PP IRQ%d #%d\n", __FUNCTION__, irq, g_irq_cnt);

	g_irq_cnt++;

	if (WARN_ON(vout->state == STATE_STREAM_OFF && vout->active == NULL)) {
		spin_unlock_irqrestore(&vout->irq_lock, flags);
		return IRQ_HANDLED;
	}
#if 0
	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		struct fb_gwinfo gwinfo;

		gwinfo.enabled = 1;
		gwinfo.alpha_value = 255;
		gwinfo.ck_enabled = 0;
		gwinfo.xpos = vout->crop_current.left;
		gwinfo.ypos = vout->crop_current.top;
		gwinfo.base = vout->display_bufs[pp_num_last()];
		gwinfo.xres = vout->crop_current.width;
		gwinfo.yres = vout->crop_current.height;
		gwinfo.xres_virtual = vout->crop_current.width;
		gwinfo.vs_reversed = 0;

		mx2_gw_set(&gwinfo);
	}
#endif
	/* Process previous buffer */
	BUG_ON(vout->active == NULL);
	BUG_ON(vout->active->buf.index >= vout->buffer_cnt);
	DBG(3, "%s: queue entry %p g_irq_cnt %d\n", __FUNCTION__,
		vout->active, g_irq_cnt);
	DBG(1, "%s: frame %u finished after %lums\n", __FUNCTION__,
		g_buf_output_cnt, diff_usec(&now, &vout->frame_start) / 1000);
	g_buf_output_cnt++;
	vout->active->buf.flags |= V4L2_BUF_FLAG_DONE;
	list_add_tail(&vout->active->head, &vout->done_q);
	DBG(2, "%s: buffer[%d] %p moved to done queue %p\n", __FUNCTION__,
		vout->active->buf.index, vout->active, &vout->done_q);
	vout->active = NULL;
	vout->busy = 0;
	wake_up_interruptible(&vout->v4l_bufq);

	BUG_ON(vout->active != NULL);

	if ((vout->state == STATE_STREAM_ON ||
	     vout->state == STATE_STREAM_STOPPING ||
	     vout->state == STATE_STREAM_PAUSED) &&
	    !list_empty(&vout->ready_q)) {
		vout->active = list_first_entry(&vout->ready_q,
						struct v4l_queue, head);
		list_del_init(&vout->active->head);
		vout->queued--;
		g_buf_dq_cnt++;
		DBG(3, "next buffer[%d] %p\n", vout->active->buf.index,
			vout->active);
		if (vout->state == STATE_STREAM_PAUSED) {
			vout->state = STATE_STREAM_ON;
		}
		mxc_v4l2out_schedule_frame(vout, &vout->active->buf.timestamp, 0);
	}
	if (list_empty(&vout->ready_q)) {
		if (vout->state == STATE_STREAM_STOPPING) {
			DBG(0, "Stream idle\n");
			//vout->state = STATE_STREAM_OFF;
		} else if (vout->state == STATE_STREAM_ON) {
			DBG(1, "No more buffers ready; pausing stream; queued: %d\n",
				vout->queued);
			g_paused_cnt++;
			vout->state = STATE_STREAM_PAUSED;
		}
	}
	spin_unlock_irqrestore(&vout->irq_lock, flags);

	return IRQ_HANDLED;
}

/*
 * Start the output stream
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 *
 * called with vout->user_lock held
 */
static int mxc_v4l2out_streamon(vout_data *vout)
{
	unsigned long flags;
	struct device *dev = &vout->video_dev->dev;
	int ret;

	if (vout->state != STATE_STREAM_OFF) {
		return -EBUSY;
	}
	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		int index = 0;
		struct v4l_queue *q;

		/* Free previously allocated buffer */
		mxc_free_buffers(vout, &vout->display_q);
		/* Allocate buffers for foreground */
		ret = mxc_allocate_buffers(vout, &vout->display_q,
					   2, vout->display_buf_size);
		if (ret != 2) {
			dev_err(dev, "unable to allocate SDC FG buffers\n");
			return ret;
		}
		list_for_each_entry(q, &vout->display_q, head) {
			DBG(2, "%s: Display buffer %d @ %08x\n", __FUNCTION__,
			    index, q->dma_desc.dma_addr);
			vout->display_bufs[index++] = q->dma_desc.dma_addr;
		}
	}

	/* Configure PP */
	ret = pp_cfg(vout);
	if (ret != 0) {
		/* Free previously allocated buffer */
		mxc_free_buffers(vout, &vout->display_q);
		dev_err(dev, "failed to config PP: %d\n", ret);
		return ret;
	}

#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	if (vout->tear_protection == TEARING_PROTECTION_ACTIVE) {
		vout->output_fb = vout->output_fb_num[vout->cur_disp_output];
		vout->fb_enabled = 0;
		vout->pp_ready = 0;
		vout->fb_event_notifier.notifier_call = fb_event_notify;
		fb_register_client(&vout->fb_event_notifier);
		mx2fb_register_client(&vout->fb_event_notifier);
	}
#endif
	spin_lock_irqsave(&vout->irq_lock, flags);
	if (list_empty(&vout->ready_q)) {
		dev_warn(dev, "no buffers queued yet!\n");
		spin_unlock_irqrestore(&vout->irq_lock, flags);
		return EINVAL;
	}
	BUG_ON(vout->active);
	vout->active = list_first_entry(&vout->ready_q, struct v4l_queue, head);
	list_del_init(&vout->active->head);
	vout->queued--;
	g_buf_dq_cnt++;
	DBG(2, "%s: processing buffer[%d] %p from ready queue %p\n", __FUNCTION__,
		vout->active->buf.index, vout->active, &vout->ready_q);

	vout->frame_count = 0;
	vout->state = STATE_STREAM_ON;
	vout->start_jiffies = jiffies;
	mxc_v4l2out_schedule_frame(vout, &vout->active->buf.timestamp, 1);
	spin_unlock_irqrestore(&vout->irq_lock, flags);

	return 0;
}

/*
 * Shut down the voutera
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 *
 * called with vout->user_lock held
 */
static int mxc_v4l2out_streamoff(vout_data *vout)
{
	struct v4l_queue *q;
	struct v4l_queue *tmp;
	unsigned long flags;

	if (vout->state == STATE_STREAM_OFF) {
		return -EINVAL;
	}

	del_timer_sync(&vout->output_timer);

	pp_enable(0);		/* Disable PP */

	spin_lock_irqsave(&vout->irq_lock, flags);

	if (vout->active != NULL) {
#ifdef DEBUG
		list_for_each_entry(q, &vout->free_q, head) {
			DBG(0, "%s: dma_addr: %08x cpu_addr: %p size %08x\n", __FUNCTION__,
				q->dma_desc.dma_addr, q->dma_desc.cpu_addr,
				q->dma_desc.size);
		}
#endif
		DBG(0, "%s: Moving active buffer %p (%08x:%p) to free list\n", __FUNCTION__,
		    vout->active, vout->active->dma_desc.dma_addr,
		    vout->active->dma_desc.cpu_addr);
		list_add_tail(&vout->active->head, &vout->free_q);
#ifdef DEBUG
		list_for_each_entry(q, &vout->free_q, head) {
			DBG(0, "%s: dma_addr: %08x cpu_addr: %p size %08x\n", __FUNCTION__,
				q->dma_desc.dma_addr, q->dma_desc.cpu_addr,
				q->dma_desc.size);
		}
#endif
		vout->active = NULL;
	}
	list_for_each_entry_safe(q, tmp, &vout->ready_q, head) {
		DBG(0, "%s: Moving buffer %p (%08x:%p) from ready to free list\n", __FUNCTION__,
		    q, q->dma_desc.dma_addr, q->dma_desc.cpu_addr);
		list_move_tail(&q->head, &vout->free_q);
		vout->queued--;
	}
	list_for_each_entry_safe(q, tmp, &vout->done_q, head) {
		DBG(0, "%s: Moving buffer %p (%08x:%p) from done to free list\n", __FUNCTION__,
		    q, q->dma_desc.dma_addr, q->dma_desc.cpu_addr);
		list_move_tail(&q->head, &vout->free_q);
	}

	BUG_ON(vout->queued < 0);
	if (WARN_ON(vout->queued)) {
		DBG(0, "queued=%d\n", vout->queued);
	}
	list_for_each_entry(q, &vout->free_q, head) {
		DBG(0, "%s: Reinitializing buffer %p (%08x:%p)\n", __FUNCTION__,
		    q, q->dma_desc.dma_addr, q->dma_desc.cpu_addr);
		q->buf.flags = 0;
		q->buf.timestamp.tv_sec = 0;
		q->buf.timestamp.tv_usec = 0;
	}

	vout->state = STATE_STREAM_OFF;
#if 0
	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		struct fb_gwinfo gwinfo;

		/* Disable graphic window */
		gwinfo.enabled = 0;
		mx2_gw_set(&gwinfo);
	}
#endif
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	if (vout->tear_protection == TEARING_PROTECTION_ACTIVE) {
		vout->output_fb = -1;
		vout->fb_enabled = 0;
		vout->pp_ready = 0;
		fb_unregister_client(&vout->fb_event_notifier);
		mx2fb_unregister_client(&vout->fb_event_notifier);
	}
#endif
	spin_unlock_irqrestore(&vout->irq_lock, flags);
	mxc_free_buffers(vout, &vout->display_q);
	up(&vout->user_lock);

	return 0;
}

/*
 * Valid whether the palette is supported
 *
 * @param palette  V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_BGR24 or V4L2_PIX_FMT_BGR32
 *
 * @return 1 if supported, 0 if failed
 */
static inline int valid_mode(u32 palette)
{
	return palette == V4L2_PIX_FMT_YUV420 ||
		palette == V4L2_PIX_FMT_YUV422P;
}

/*
 * Returns bits per pixel for given pixel format
 *
 * @param pixelformat  V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_BGR24 or V4L2_PIX_FMT_BGR32
 *
 * @return bits per pixel of pixelformat
 */
static u32 fmt_to_bpp(u32 pixelformat)
{
	u32 bpp;

	switch (pixelformat) {
	case V4L2_PIX_FMT_RGB565:
		bpp = 16;
		break;
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
		bpp = 24;
		break;
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		bpp = 32;
		break;
	default:
		bpp = 8;
		break;
	}
	return bpp;
}

/*
 * V4L2 - Handles VIDIOC_G_FMT Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param v4l2_format structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2out_g_fmt(vout_data *vout, struct v4l2_format *f)
{
	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		return -EINVAL;
	}
	*f = vout->v2f;
	return 0;
}

/*
 * V4L2 - Handles VIDIOC_S_FMT Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param v4l2_format structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2out_s_fmt(vout_data *vout, struct v4l2_format *f)
{
	int retval = 0;
	u32 size = 0;
	u32 bytesperline;
	struct device *dev = &vout->video_dev->dev;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		dev_err(dev, "buffer type %d not supported\n", f->type);
		retval = -EINVAL;
		goto err0;
	}
	if (!valid_mode(f->fmt.pix.pixelformat)) {
		dev_err(dev, "pixel format %08x not supported\n", f->fmt.pix.pixelformat);
		retval = -EINVAL;
		goto err0;
	}

	bytesperline = (f->fmt.pix.width * fmt_to_bpp(f->fmt.pix.pixelformat)) / 8;
	if (f->fmt.pix.bytesperline < bytesperline) {
		f->fmt.pix.bytesperline = bytesperline;
	} else {
		bytesperline = f->fmt.pix.bytesperline;
	}
	if (bytesperline == 0) {
		return -EINVAL;
	}

	vout->v2f.fmt.pix.width = f->fmt.pix.width;
	vout->v2f.fmt.pix.height = f->fmt.pix.height;
	vout->v2f.fmt.pix.priv = f->fmt.pix.priv;
	if (vout->v2f.fmt.pix.priv) {
		if (copy_from_user(&vout->crop_rect,
				   (void __user *)vout->v2f.fmt.pix.priv,
				   sizeof(struct v4l2_rect))) {
			retval = -EFAULT;
			goto err0;
		}
	}
	DBG(0, "%s: format: %dx%d-%dbpp %u bytes per line\n", __FUNCTION__,
		f->fmt.pix.width, f->fmt.pix.height,
		fmt_to_bpp(f->fmt.pix.pixelformat), bytesperline);

	if ((vout->crop_rect.top > f->fmt.pix.height) ||
	    (vout->crop_rect.left > f->fmt.pix.width)) {
		dev_warn(dev, "invalid crop rect %dx%d (%dx%d)\n",
			vout->crop_rect.left, vout->crop_rect.top,
			f->fmt.pix.width, f->fmt.pix.height);
		retval = -EINVAL;
		goto err0;
	}

	if ((vout->crop_rect.top + vout->crop_rect.height) > f->fmt.pix.height)
		vout->crop_rect.height =
		    f->fmt.pix.height - vout->crop_rect.top;
	if ((vout->crop_rect.left + vout->crop_rect.width) > f->fmt.pix.width)
		vout->crop_rect.width = f->fmt.pix.width - vout->crop_rect.left;

	vout->crop_rect.width -= vout->crop_rect.width % 8;
	vout->crop_rect.height -= vout->crop_rect.height % 8;
	vout->crop_rect.top -= vout->crop_rect.top % 2;
	vout->crop_rect.left += vout->crop_rect.left % 2;

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
		/* bytesperline for YUV planar formats is for Y plane only */
		size = bytesperline * f->fmt.pix.height * 2;
		if ((vout->crop_rect.width != 0)
		    && (vout->crop_rect.height != 0)) {
			vout->offset.y_offset =
			    (vout->v2f.fmt.pix.width * vout->crop_rect.top) +
			    (vout->crop_rect.left);
			vout->offset.u_offset =
			    (vout->v2f.fmt.pix.width *
			     vout->v2f.fmt.pix.height) +
			    ((vout->v2f.fmt.pix.width *
			      vout->crop_rect.top) >> 1) +
			    (vout->crop_rect.left >> 1);
			vout->offset.v_offset =
			    (vout->v2f.fmt.pix.width *
			     vout->v2f.fmt.pix.height) +
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) >> 1) +
			    ((vout->v2f.fmt.pix.width *
			      vout->crop_rect.top) >> 1) +
			    (vout->crop_rect.left >> 1);
			vout->offset.qp_offset =
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) << 1) +
			    (((vout->v2f.fmt.pix.width * vout->crop_rect.top) +
			      vout->crop_rect.left) >> 10);
		} else {
			vout->offset.y_offset = 0;
			vout->offset.u_offset =
			    (vout->v2f.fmt.pix.width *
			     vout->v2f.fmt.pix.height);
			vout->offset.v_offset =
			    vout->offset.u_offset +
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) >> 1);
			vout->offset.qp_offset =
			    vout->offset.v_offset +
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) >> 1);
		}
		break;

	case V4L2_PIX_FMT_YUV420:
		size = (bytesperline * f->fmt.pix.height * 3) / 2;
		if ((vout->crop_rect.width != 0)
		    && (vout->crop_rect.height != 0)) {
			vout->offset.y_offset =
			    (vout->v2f.fmt.pix.width * vout->crop_rect.top) +
			    (vout->crop_rect.left);
			vout->offset.u_offset =
			    (vout->v2f.fmt.pix.width *
			     vout->v2f.fmt.pix.height) +
			    ((vout->v2f.fmt.pix.width *
			      vout->crop_rect.top) >> 2) +
			    (vout->crop_rect.left >> 1);
			vout->offset.v_offset =
			    (vout->v2f.fmt.pix.width *
			     vout->v2f.fmt.pix.height) +
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) >> 2) +
			    ((vout->v2f.fmt.pix.width *
			      vout->crop_rect.top) >> 2) +
			    (vout->crop_rect.left >> 1);
			vout->offset.qp_offset =
			    (vout->v2f.fmt.pix.width *
			     vout->v2f.fmt.pix.height) +
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) >> 1) +
			    (((vout->v2f.fmt.pix.width * vout->crop_rect.top) +
			      vout->crop_rect.left) >> 10);
		} else {
			vout->offset.y_offset = 0;
			vout->offset.u_offset =
			    (vout->v2f.fmt.pix.width *
			     vout->v2f.fmt.pix.height);
			vout->offset.v_offset =
			    vout->offset.u_offset +
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) >> 2);
			vout->offset.qp_offset =
			    vout->offset.v_offset +
			    ((vout->v2f.fmt.pix.width *
			      vout->v2f.fmt.pix.height) >> 2);
		}
		break;

	default:
		size = bytesperline * f->fmt.pix.height;
		vout->offset.y_offset = 0;
		vout->offset.u_offset =
		    (vout->v2f.fmt.pix.width * vout->v2f.fmt.pix.height);
		vout->offset.v_offset =
		    vout->offset.u_offset +
		    ((vout->v2f.fmt.pix.width * vout->v2f.fmt.pix.height) >> 2);
		vout->offset.qp_offset =
		    vout->offset.v_offset +
		    ((vout->v2f.fmt.pix.width * vout->v2f.fmt.pix.height) >> 2);
	}

	/* Return the actual size of the image to the app */
	f->fmt.pix.sizeimage = size;
	vout->v2f.fmt.pix.sizeimage = size;
	vout->v2f.fmt.pix.pixelformat = f->fmt.pix.pixelformat;
	vout->v2f.fmt.pix.bytesperline = f->fmt.pix.bytesperline;

	DBG(0, "%s: Y: %08x..%08x\n", __FUNCTION__,
		vout->offset.y_offset, vout->offset.y_offset +
		f->fmt.pix.height * bytesperline);
	DBG(0, "%s: U: %08x..%08x\n", __FUNCTION__,
		vout->offset.u_offset, vout->offset.u_offset +
		f->fmt.pix.height * bytesperline / 2);
	DBG(0, "%s: V: %08x..%08x\n", __FUNCTION__,
		vout->offset.v_offset, vout->offset.v_offset +
		f->fmt.pix.height * bytesperline / 2);

	return 0;

 err0:
	dev_warn(dev, "%s: failed with error %d\n", __FUNCTION__, retval);
	return retval;
}

/*
 * V4L2 - Handles VIDIOC_G_CTRL Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param c           structure v4l2_control *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_get_v4l2out_control(vout_data *vout, struct v4l2_control *c)
{
	switch (c->id) {
	case V4L2_CID_HFLIP:
		c->value = (vout->rotate & IPU_ROTATE_HORIZ_FLIP) ? 1 : 0;
		break;
	case V4L2_CID_VFLIP:
		c->value = (vout->rotate & IPU_ROTATE_VERT_FLIP) ? 1 : 0;
		break;
	case (V4L2_CID_PRIVATE_BASE + 1):
		c->value = vout->rotate;
		break;
	case V4L2_CID_MXC_TEAR_PROTECT:
		c->value = vout->tear_protection;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * V4L2 - Handles VIDIOC_S_CTRL Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param c           structure v4l2_control *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_set_v4l2out_control(vout_data *vout, struct v4l2_control *c)
{
	switch (c->id) {
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_MXC_ROT:
		if (c->value < 0 || c->value > 7) {
			return -EINVAL;
		}
		vout->rotate = c->value;
		break;
	case V4L2_CID_MXC_TEAR_PROTECT:
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
		if (c->value == TEARING_PROTECTION_ACTIVE)
			vout->tear_protection = TEARING_PROTECTION_ACTIVE;
		else
			vout->tear_protection = TEARING_PROTECTION_INACTIVE;;
#else
		vout->tear_protection = TEARING_PROTECTION_UNSUPPORTED;
#endif
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * V4L2 interface - open function
 *
 * @param file         structure file *
 *
 * @return  status    0 success, ENODEV invalid device instance,
 *                    ENODEV timeout, ERESTARTSYS interrupted by user
 */
static int mxc_v4l2out_open(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	vout_data *vout = video_get_drvdata(dev);
	int err;

	dq_intr_cnt = 0;
	if (!vout) {
		dev_info(&dev->dev, "Internal error, vout_data not found!\n");
		return -ENODEV;
	}

	down(&vout->user_lock);
	if (vout->open_count == 0) {
		err = pp_init(vout);
		if (err) {
			goto out;
		}
		init_timer(&vout->output_timer);
		vout->output_timer.function = mxc_v4l2out_timer_handler;
		vout->output_timer.data = (unsigned long)vout;

		vout->state = STATE_STREAM_OFF;
		vout->queued = 0;
		g_irq_cnt = g_buf_output_cnt = g_buf_q_cnt = g_buf_dq_cnt = 0;
		g_busy_cnt = 0;
		g_paused_cnt = 0;
		g_late_cnt = 0;
		using_default_fps = -1;
		vout->open_count++;
	}
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	vout->tear_protection = TEARING_PROTECTION_ACTIVE;
#else
	vout->tear_protection = TEARING_PROTECTION_UNSUPPORTED;
#endif

	file->private_data = dev;
 out:
	up(&vout->user_lock);
	return err;
}

/*
 * V4L2 interface - close function
 *
 * @param file     struct file *
 *
 * @return         0 success
 */
static int mxc_v4l2out_close(struct file *file)
{
	struct video_device *dev = file->private_data;
	vout_data *vout = video_get_drvdata(dev);

	down(&vout->user_lock);
	if (vout->open_count) {
		DBG(0, "%s: interrupts handled: %u frames output: %u queued: %u dequeued: %u paused: %u busy: %u late: %u\n",
			__FUNCTION__, g_irq_cnt, g_buf_output_cnt, g_buf_q_cnt,
			g_buf_dq_cnt, g_paused_cnt, g_busy_cnt, g_late_cnt);

		pp_exit(vout);
		mxc_v4l2out_streamoff(vout);
		file->private_data = NULL;
		WARN_ON(!list_empty(&vout->done_q));
		if (WARN_ON(vout->active)) {
			DBG(0, "%s: Moving active buffer %p (%08x:%p) to done list\n",
				__FUNCTION__, vout->active,
				vout->active->dma_desc.dma_addr,
				vout->active->dma_desc.cpu_addr);
			list_add_tail(&vout->active->head, &vout->done_q);
			vout->active = NULL;
		}
		WARN_ON(!list_empty(&vout->ready_q));
		DBG(0, "%s: Releasing buffers from ready queue\n", __FUNCTION__);
		mxc_free_buffers(vout, &vout->ready_q);
		DBG(0, "%s: Releasing buffers from done queue\n", __FUNCTION__);
		mxc_free_buffers(vout, &vout->done_q);
		DBG(0, "%s: Releasing buffers from free queue\n", __FUNCTION__);
		mxc_free_buffers(vout, &vout->free_q);
		vout->buffer_cnt = 0;
		DBG(0, "%s: Releasing display buffers\n", __FUNCTION__);
		mxc_free_buffers(vout, &vout->display_q);

		/* capture off */
		wake_up_interruptible(&vout->v4l_bufq);
		WARN_ON(--vout->open_count);
	}
	up(&vout->user_lock);

	return 0;
}

static int do_dequeue(vout_data *vout, struct v4l2_buffer *user_buf)
{
	int retval;
	struct device *dev = &vout->video_dev->dev;
	unsigned long flags;

	if (vout->active == NULL &&
	    list_empty(&vout->done_q) &&
	    list_empty(&vout->ready_q)) {
		return -EINVAL;
	}
	up(&vout->user_lock);
	retval = wait_event_interruptible_timeout(vout->v4l_bufq,
						  !list_empty(&vout->done_q),
						  10 * HZ);
	down(&vout->user_lock);
	spin_lock_irqsave(&vout->irq_lock, flags);
	if (retval >= 0 && !list_empty(&vout->done_q)) {
		struct v4l_queue *q;
		struct v4l2_buffer *buf;

		q = list_first_entry(&vout->done_q, struct v4l_queue, head);
		DBG(2, "%s: processing buffer[%d] %p from done queue %p\n",
		    __FUNCTION__, q->buf.index, q, &vout->done_q);
		WARN_ON(!(q->buf.flags & V4L2_BUF_FLAG_DONE));
		q->buf.flags &= ~(V4L2_BUF_FLAG_DONE | V4L2_BUF_FLAG_QUEUED);
		list_move_tail(&q->head, &vout->free_q);
		buf = &q->buf;
		DBG(2, "%s: buffer[%d] %p moved to free queue %p\n", __FUNCTION__,
		    buf->index, q, &vout->free_q);
		BUG_ON(buf->index >= vout->buffer_cnt);
		if (user_buf != NULL) {
			memcpy(user_buf, buf, sizeof(*user_buf));
		}
		DBG(1, "VIDIOC_DQBUF: buffer %d: %d\n", buf->index, retval);
		retval = 0;
	} else if (retval == 0) {
		dev_warn(dev, "VIDIOC_DQBUF: timeout\n");
		retval = -ETIME;
		DBG(3, "VIDIOC_DQBUF: ret: %d\n", retval);
	}
	spin_unlock_irqrestore(&vout->irq_lock, flags);
	return retval;
}
/*
 * V4L2 interface - ioctl function
 *
 * @param file       struct file *
 *
 * @param ioctlnr    unsigned int
 *
 * @param arg        void *
 *
 * @return           0 success, ENODEV for invalid device instance,
 *                   -1 for other errors.
 */
static long mxc_v4l2out_do_ioctl(struct file *file,
				unsigned int ioctlnr, void *arg)
{
	struct video_device *vdev = file->private_data;
	struct device *dev = &vdev->dev;
	vout_data *vout = video_get_drvdata(vdev);
	int retval = 0;
	int i = 0;

	if (dbg_lvl(3)) {
		v4l_printk_ioctl(ioctlnr);
		printk(" arg=%p\n", arg);
	}

	down(&vout->user_lock);
	switch (ioctlnr) {
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap = arg;
			strcpy(cap->driver, "mxc_v4l2_output");
			cap->version = 0;
			cap->capabilities = V4L2_CAP_VIDEO_OUTPUT |
				V4L2_CAP_STREAMING;
			strlcpy(cap->card, "MX27 eMMA", sizeof(cap->card));
			cap->bus_info[0] = '\0';
			retval = 0;
		}
		break;

	case VIDIOC_G_FMT:
		{
			struct v4l2_format *gf = arg;
			retval = mxc_v4l2out_g_fmt(vout, gf);
		}
		break;

	case VIDIOC_S_FMT:
		{
			struct v4l2_format *sf = arg;
			if (vout->state != STATE_STREAM_OFF) {
				retval = -EBUSY;
				break;
			}
			retval = mxc_v4l2out_s_fmt(vout, sf);
		}
		break;

	case VIDIOC_REQBUFS:
		{
			struct v4l2_requestbuffers *req = arg;
			struct v4l_queue *q;
			size_t bufsize = PAGE_ALIGN(vout->v2f.fmt.pix.sizeimage);
			unsigned long flags;

			if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) ||
			    (req->memory != V4L2_MEMORY_MMAP)) {
				dev_warn(&vdev->dev,
					"VIDIOC_REQBUFS: incorrect buffer type: %d\n",
					req->type);
				retval = -EINVAL;
				break;
			}

			DBG(0, "%s: VIDIOC_REQBUFS: %d buffers of %u byte requested\n",
			    __FUNCTION__, req->count, bufsize);

			if (req->count == 0) {
				spin_lock_irqsave(&vout->irq_lock, flags);
				if (vout->state == STATE_STREAM_ON) {
					vout->state = STATE_STREAM_STOPPING;
				}
				spin_unlock_irqrestore(&vout->irq_lock, flags);
				while (vout->active ||
					!list_empty(&vout->ready_q)) {
					DBG(0, "%s: Waiting for stream to drain; queued: %d\n",
					    __FUNCTION__, vout->queued);
					retval = do_dequeue(vout, NULL);
					if (retval != 0) {
						DBG(0, "do_dequeue() returned %d\n",
							retval);
					}
					if (retval == -ERESTARTSYS) {
						break;
					}
				}
#if 1
				retval = mxc_v4l2out_streamoff(vout);
#else
				pp_enable(0);		/* Disable PP */
				WARN_ON(!list_empty(&vout->ready_q));
				WARN_ON(vout->active);
				mxc_free_buffers(vout, &vout->done_q);
				mxc_free_buffers(vout, &vout->free_q);
#endif
				break;
			}
			if (vout->state != STATE_STREAM_OFF) {
				dev_warn(dev, "VIDIOC_REQBUFS: streaming active\n");
				retval = -EBUSY;
				break;
			}
#if 0
			if (!list_empty(&vout->done_q) ||
				!list_empty(&vout->ready_q)) {
				dev_warn(dev, "VIDIOC_REQBUFS: busy buffers exist\n");
				retval = -EBUSY;
				break;
			}
#endif
			/* This needs no interrupt locking, since we have the busy_lock
			 * and are in STATE_STREAM_OFF where there cannot be any interrupts
			 */
			if (!list_empty(&vout->free_q)) {
				q = list_first_entry(&vout->free_q,
						struct v4l_queue, head);
				if (q->buf.length != bufsize ||
				    q->buf.type != req->type ||
				    q->buf.memory != req->memory) {
					dev_err(dev, "VIDIOC_REQBUFS: Mismatch between old and new buffer parameters\n");
					retval = -EINVAL;
					break;
				}
			}

			if (req->count > MAX_FRAME_NUM) {
				req->count = MAX_FRAME_NUM;
			}
			if (req->count == vout->buffer_cnt) {
				break;
			} else if (req->count < vout->buffer_cnt) {
				struct list_head release_q;
				struct list_head *tmp;
				struct list_head *x;

				list_for_each_prev_safe(x, tmp, &vout->free_q) {
					if (vout->buffer_cnt-- > req->count) {
						q = container_of(x, struct v4l_queue, head);
						DBG(0, "%s: Moving buffer[%d:%d] %p (%08x:%p) from free to release list\n",
							__FUNCTION__, vout->buffer_cnt,
							q->buf.index, q, q->dma_desc.dma_addr,
							q->dma_desc.cpu_addr);
						list_move(x, &release_q);
					}
				}
				BUG_ON(vout->buffer_cnt != req->count);
				mxc_free_buffers(vout, &release_q);
				break;
			}
			retval = mxc_allocate_buffers(vout, &vout->free_q,
						req->count - vout->buffer_cnt,
						bufsize);
			if (retval + vout->buffer_cnt < req->count) {
				dev_warn(dev, "Could only allocate %u of %u buffers\n",
					retval + vout->buffer_cnt, req->count);
				req->count = retval + vout->buffer_cnt;
			}
			vout->buffer_cnt = req->count;
			i = 0;
			list_for_each_entry(q, &vout->free_q, head) {
				q->buf.memory = V4L2_MEMORY_MMAP;
				q->buf.index = i;
				q->buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
				q->buf.length = bufsize;
				q->buf.m.offset = q->dma_desc.dma_addr;
				i++;
			}
			BUG_ON(i != vout->buffer_cnt);
		}
		break;

	case VIDIOC_QUERYBUF:
		{
			struct v4l2_buffer *buf = arg;
			struct v4l_queue *q;
			unsigned long flags;

			if (buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				dev_warn(dev, "VIDIOC_QUERYBUF: incorrect buffer type: %d expected %d\n",
					buf->type, V4L2_BUF_TYPE_VIDEO_OUTPUT);
				retval = -EINVAL;
				break;
			}
			if (buf->index >= vout->buffer_cnt) {
				dev_warn(dev, "VIDIOC_QUERYBUF: buffer index out of range: %d [0..%d]\n",
					buf->index, vout->buffer_cnt);
				retval = -EINVAL;
				break;
			}
			spin_lock_irqsave(&vout->irq_lock, flags);
			q = find_buffer(&vout->free_q, buf->index);
			if (q == NULL) {
				q = find_buffer(&vout->ready_q, buf->index);
				if (q == NULL) {
					q = find_buffer(&vout->done_q, buf->index);
					if (q != NULL) {
						DBG(2, "VIDIOC_QUERYBUF: buffer[%d] %p found in done list\n",
						    q->buf.index, q);
					}
				} else {
					DBG(2, "VIDIOC_QUERYBUF: buffer[%d] %p found in ready list\n",
					    q->buf.index, q);
				}
			} else {
				DBG(2, "VIDIOC_QUERYBUF: buffer[%d] %p found in free list\n",
				    q->buf.index, q);
			}
			spin_unlock_irqrestore(&vout->irq_lock, flags);
			if (q == NULL) {
				dev_warn(dev, "VIDIOC_QUERYBUF: buffer %d not found in any list\n",
					buf->index);
				retval = -ENOENT;
				break;
			}
			memcpy(buf, &q->buf, sizeof(q->buf));
		}
		break;

	case VIDIOC_QBUF:
		{
			struct v4l2_buffer *buf = arg;
			struct v4l_queue *q;
			unsigned long flags;

			if (buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				dev_err(dev, "VIDIOC_QBUF: invalid buffer type: %u\n",
				       buf->type);
				retval = -EINVAL;
				break;
			}

			if (buf->index >= vout->buffer_cnt) {
				dev_err(dev, "VIDIOC_QBUF: invalid buffer index: %u/%u\n",
				       buf->index, vout->buffer_cnt);
				retval = -EINVAL;
				break;
			}

			if (vout->state == STATE_STREAM_STOPPING) {
				dev_err(dev, "VIDIOC_QBUF: stream is draining\n");
				retval = -EINVAL;
				break;
			}
			DBG(1, "VIDIOC_QBUF: %d/%d ts %6lu.%06lu\n", buf->index,
				vout->queued, buf->timestamp.tv_sec,
				buf->timestamp.tv_usec);

			spin_lock_irqsave(&vout->irq_lock, flags);
			q = find_buffer(&vout->free_q, buf->index);
			if (q == NULL || (q->buf.flags & V4L2_BUF_FLAG_QUEUED)) {
				spin_unlock_irqrestore(&vout->irq_lock, flags);
				if (q == NULL) {
					printk(KERN_ERR "buffer %d not on free queue %p\n",
					       buf->index, &vout->free_q);
					retval = -ENOENT;
				} else {
					retval = -EBUSY;
				}
				break;
			}
			q->buf.timestamp = buf->timestamp;
			buf = &q->buf;
			WARN_ON(buf->flags & V4L2_BUF_FLAG_DONE);
			buf->flags |= V4L2_BUF_FLAG_QUEUED;

			BUG_ON(buf->index >= vout->buffer_cnt);

			if (vout->state == STATE_STREAM_PAUSED &&
			    vout->active == NULL) {
				DBG(1, "%s: Restarting stream\n", __FUNCTION__);
				vout->active = q;
				list_del_init(&vout->active->head);
				vout->state = STATE_STREAM_ON;

				mxc_v4l2out_schedule_frame(vout,
							&buf->timestamp, 0);
			} else {
				list_move_tail(&q->head, &vout->ready_q);
				vout->queued++;
				g_buf_q_cnt++;
				DBG(2, "%s: buffer[%d] %p moved to ready queue %p\n",
					__FUNCTION__, buf->index, q, &vout->ready_q);
			}
			if (vout->state == STATE_STREAM_PAUSED) {
				vout->state = STATE_STREAM_ON;
			}
			spin_unlock_irqrestore(&vout->irq_lock, flags);
		}
		break;

	case VIDIOC_DQBUF:
		{
			struct v4l2_buffer *buf = arg;

			DBG(1, "VIDIOC_DQBUF: queued: %d\n", vout->queued);

			if (list_empty(&vout->done_q) &&
			    (file->f_flags & O_NONBLOCK)) {
				retval = -EAGAIN;
				DBG(3, "VIDIOC_DQBUF: ret: %d\n", retval);
				break;
			}

			retval = do_dequeue(vout, buf);
			if (retval == -ETIME) {
				dev_warn(dev, "VIDIOC_DQBUF: timeout\n");
				DBG(3, "VIDIOC_DQBUF: ret: %d\n", retval);
			} else if (retval == -ERESTARTSYS) {
				if (dq_intr_cnt == 0)
					DBG(0, "VIDIOC_DQBUF: interrupted\n");
				dq_intr_cnt++;
				DBG(0, "VIDIOC_DQBUF: ret: %d\n", retval);
			} else {
				DBG(1, "VIDIOC_DQBUF: ret: %d\n", retval);
			}
		}
		break;

	case VIDIOC_STREAMON:
		retval = mxc_v4l2out_streamon(vout);
		break;

	case VIDIOC_STREAMOFF:
		retval = mxc_v4l2out_streamoff(vout);
		break;

	case VIDIOC_G_CTRL:
		retval = mxc_get_v4l2out_control(vout, arg);
		break;

	case VIDIOC_S_CTRL:
		retval = mxc_set_v4l2out_control(vout, arg);
		break;

	case VIDIOC_CROPCAP:
		{
			struct v4l2_cropcap *cap = arg;

			if (cap->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				retval = -EINVAL;
				break;
			}
			if (vout->cur_disp_output < 0) {
				retval = -ENXIO;
				break;
			}
			cap->bounds = vout->crop_bounds[vout->cur_disp_output];
			cap->defrect = vout->crop_bounds[vout->cur_disp_output];
			retval = 0;
		}
		break;

	case VIDIOC_G_CROP:
		{
			struct v4l2_crop *crop = arg;

			if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				retval = -EINVAL;
				break;
			}
			crop->c = vout->crop_current;
		}
		break;

	case VIDIOC_S_CROP:
		{
			struct v4l2_crop *crop = arg;
			struct v4l2_rect *b = &vout->crop_bounds[vout->cur_disp_output];

			if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				retval = -EINVAL;
				break;
			}
			if (crop->c.height < 0) {
				retval = -EINVAL;
				break;
			}
			if (crop->c.width < 0) {
				retval = -EINVAL;
				break;
			}

			if (vout->cur_disp_output < 0) {
				retval = -ENXIO;
				break;
			}
			if (crop->c.top < b->top)
				crop->c.top = b->top;
			if (crop->c.top > b->top + b->height)
				crop->c.top = b->top + b->height;
			if (crop->c.height > b->top - crop->c.top + b->height)
				crop->c.height =
				    b->top - crop->c.top + b->height;

			if (crop->c.left < b->left)
				crop->c.top = b->left;
			if (crop->c.left > b->left + b->width)
				crop->c.top = b->left + b->width;
			if (crop->c.width > b->left - crop->c.left + b->width)
				crop->c.width =
				    b->left - crop->c.left + b->width;

			/* stride line limitation */
			crop->c.height -= crop->c.height % 8;
			crop->c.width -= crop->c.width % 8;

			vout->crop_current = crop->c;
			DBG(0, "%s: crop rect: %dx%d%+d%+d\n", __FUNCTION__,
			    crop->c.width, crop->c.height,
			    crop->c.left, crop->c.top);

			vout->display_buf_size = vout->crop_current.width *
			    vout->crop_current.height;
			vout->display_buf_size *= fmt_to_bpp(SDC_FG_FB_FORMAT) / 8;
		}
		break;

	case VIDIOC_ENUMOUTPUT:
		{
			struct v4l2_output *output = arg;

			if ((output->index >= MXC_V4L2_OUT_NUM_OUTPUTS) ||
				!vout->output_enabled[output->index]) {
				retval = -EINVAL;
				break;
			}
			memcpy(output, &mxc_outputs[output->index], sizeof(*output));
			snprintf(output->name, sizeof(output->name),
				mxc_outputs[output->index].name, output->index);
		}
		break;

	case VIDIOC_G_OUTPUT:
		{
			u32 *index = arg;

			*index = vout->cur_disp_output;
		}
		break;

	case VIDIOC_S_OUTPUT:
		{
			u32 *index = arg;

			if ((*index >= MXC_V4L2_OUT_NUM_OUTPUTS) ||
				!vout->output_enabled[*index]) {
				retval = -EINVAL;
				break;
			}

			if (vout->state != STATE_STREAM_OFF) {
				retval = -EBUSY;
				break;
			}

			vout->cur_disp_output = *index;
		}
		break;

	case VIDIOC_G_FBUF:
		{
			struct v4l2_framebuffer *fb = arg;

			memcpy(fb, &vout->v4l2_fb, sizeof(*fb));
		}
		break;

	case VIDIOC_S_FBUF:
		{
			struct v4l2_framebuffer *fb = arg;

			memcpy(&vout->v4l2_fb, fb, sizeof(vout->v4l2_fb));
			vout->v4l2_fb.capability = V4L2_FBUF_CAP_EXTERNOVERLAY;
		}
		break;

	case VIDIOC_ENUM_FMT:
	case VIDIOC_TRY_FMT:
	case VIDIOC_QUERYCTRL:
	case VIDIOC_G_PARM:
	case VIDIOC_ENUMSTD:
	case VIDIOC_G_STD:
	case VIDIOC_S_STD:
	case VIDIOC_G_TUNER:
	case VIDIOC_S_TUNER:
	case VIDIOC_G_FREQUENCY:
	case VIDIOC_S_FREQUENCY:
	default:
		retval = -EINVAL;
	}

	up(&vout->user_lock);
	return retval;
}

/*
 * V4L2 interface - ioctl function
 *
 * @return  None
 */
static long mxc_v4l2out_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, mxc_v4l2out_do_ioctl);
}

/*
 * V4L2 interface - mmap function
 *
 * @param file          structure file *
 *
 * @param vma           structure vm_area_struct *
 *
 * @return status       0 Success, EINTR busy lock error,
 *                      ENOBUFS remap_page error
 */
static int mxc_v4l2out_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *vdev = file->private_data;
	struct device *dev = &vdev->dev;
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	int res = 0;
	vout_data *vout = video_get_drvdata(vdev);

	down(&vout->user_lock);

	/* make buffers write-thru cacheable */
	vma->vm_page_prot = __pgprot(pgprot_val(vma->vm_page_prot) &
				     ~L_PTE_BUFFERABLE);

	if (remap_pfn_range(vma, start, vma->vm_pgoff, size, vma->vm_page_prot)) {
		dev_err(dev, "%s: - remap_pfn_range failed\n", __FUNCTION__);
		res = -ENOBUFS;
	}

	up(&vout->user_lock);
	return res;
}

/*
 * V4L2 interface - poll function
 *
 * @param file       structure file *
 *
 * @param wait       structure poll_table *
 *
 * @return  status   POLLIN | POLLRDNORM
 */
static unsigned int mxc_v4l2out_poll(struct file *file, poll_table *wait)
{
	struct video_device *vdev = file->private_data;
	vout_data *vout = video_get_drvdata(vdev);

	wait_queue_head_t *queue = NULL;
	int res = POLLIN | POLLRDNORM;

	down(&vout->user_lock);

	queue = &vout->v4l_bufq;
	poll_wait(file, queue, wait);

	up(&vout->user_lock);
	return res;
}

static struct v4l2_file_operations mxc_v4l2out_fops = {
	.owner = THIS_MODULE,
	.open = mxc_v4l2out_open,
	.release = mxc_v4l2out_close,
	.ioctl = mxc_v4l2out_ioctl,
	.mmap = mxc_v4l2out_mmap,
	.poll = mxc_v4l2out_poll,
};

static struct video_device mxc_v4l2out_template = {
	.name = "MXC Video Output",
	.fops = &mxc_v4l2out_fops,
	.release = video_device_release,
};

/*
 * Probe routine for the framebuffer driver. It is called during the
 * driver binding process.      The following functions are performed in
 * this routine: Framebuffer initialization, Memory allocation and
 * mapping, Framebuffer registration, IPU initialization.
 *
 * @return      Appropriate error code to the kernel common code
 */
static int mxc_v4l2out_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	vout_data *vout;

	/*
	 * Allocate sufficient memory for the fb structure
	 */
	vout = kzalloc(sizeof(vout_data), GFP_KERNEL);
	if (!vout)
		return -ENOMEM;

	vout->video_dev = video_device_alloc();
	if (vout->video_dev == NULL) {
		kfree(vout);
		return -ENOMEM;
	}
	*vout->video_dev = mxc_v4l2out_template;

	vout->video_dev->parent = &pdev->dev;
	vout->video_dev->minor = -1;

	init_waitqueue_head(&vout->v4l_bufq);

	INIT_LIST_HEAD(&vout->free_q);
	INIT_LIST_HEAD(&vout->ready_q);
	INIT_LIST_HEAD(&vout->done_q);

	INIT_LIST_HEAD(&vout->display_q);

	spin_lock_init(&vout->irq_lock);

	init_MUTEX(&vout->user_lock);

	/* register v4l device */
	ret = video_register_device(vout->video_dev,
				    VFL_TYPE_GRABBER, video_nr);
	if (ret != 0) {
		dev_err(&pdev->dev, "video_register_device failed: %d\n", ret);
		kfree(vout->video_dev);
		kfree(vout);
		return ret;
	}
	DBG(0, "mxc_v4l2out: registered device video%d\n",
		vout->video_dev->minor & 0x1f);

	video_set_drvdata(vout->video_dev, vout);
	platform_set_drvdata(pdev, vout);

	/* setup outputs and cropping */
	vout->cur_disp_output = -1;
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	vout->output_fb = -1;
#endif
	/* set default pixfmt to 16bpp */
	vout->v4l2_fb.fmt.pixelformat = V4L2_PIX_FMT_RGB565;

	for (i = 0; i < num_registered_fb && i < MXC_V4L2_OUT_NUM_OUTPUTS; i++) {
		struct fb_info *fbi = registered_fb[i];
		char *idstr = fbi->fix.id;

		DBG(0, "Checking FB '%s'\n", idstr);
		if (strncmp(idstr, "IMX", 3) == 0) {
			int disp_num = i;

			vout->crop_bounds[disp_num].left = 0;
			vout->crop_bounds[disp_num].top = 0;
			vout->crop_bounds[disp_num].width = fbi->var.xres;
			vout->crop_bounds[disp_num].height = fbi->var.yres;
			vout->output_enabled[disp_num] = true;
			vout->output_fb_num[disp_num] = i;
			DBG(0, "crop bounds: %dx%d%+d%+d\n",
			    vout->crop_bounds[disp_num].left,
			    vout->crop_bounds[disp_num].top,
			    vout->crop_bounds[disp_num].width,
			    vout->crop_bounds[disp_num].height);
			if (vout->cur_disp_output == -1) {
				vout->cur_disp_output = disp_num;
				switch (fbi->var.bits_per_pixel) {
				case 16:
					vout->v4l2_fb.fmt.pixelformat = V4L2_PIX_FMT_RGB565;
					break;

				case 32:
					vout->v4l2_fb.fmt.pixelformat = V4L2_PIX_FMT_RGB32;
					break;

				default:
					dev_warn(&pdev->dev, "Unsupported color depth %d on frame buffer %s\n",
						fbi->var.bits_per_pixel, fbi->fix.id);
				}
			}
		}
	}
	if (vout->cur_disp_output >= 0) {
		vout->crop_current = vout->crop_bounds[vout->cur_disp_output];
	}

	/* Setup framebuffer parameters */
	vout->v4l2_fb.capability = V4L2_FBUF_CAP_EXTERNOVERLAY;
	vout->v4l2_fb.flags = V4L2_FBUF_FLAG_PRIMARY;

	return 0;
}

static int mxc_v4l2out_remove(struct platform_device *pdev)
{
	vout_data *vout = platform_get_drvdata(pdev);

	video_unregister_device(vout->video_dev);
	kfree(vout);
	return 0;
}

/*
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_v4l2out_driver = {
	.driver = {
		.name = "mx27_v4l_output",
	},
	.probe = mxc_v4l2out_probe,
	.remove = mxc_v4l2out_remove,
};

/*
 * mxc v4l2 init function
 *
 */
static int mxc_v4l2out_init(void)
{
	return platform_driver_register(&mxc_v4l2out_driver);
}
module_init(mxc_v4l2out_init);

/*
 * mxc v4l2 cleanup function
 *
 */
static void mxc_v4l2out_clean(void)
{
	platform_driver_unregister(&mxc_v4l2out_driver);
}
module_exit(mxc_v4l2out_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("V4L2-driver for MXC video output");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");
MODULE_ALIAS("platform:mx27_v4l_output");
