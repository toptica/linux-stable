/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup MXC_V4L2_OUTPUT MXC V4L2 Video Output Driver
 */
/*!
 * @file mxc_v4l2_output.h
 *
 * @brief MXC V4L2 Video Output Driver Header file
 *
 * Video4Linux2 Output Device using MXC IPU Post-processing functionality.
 *
 * @ingroup MXC_V4L2_OUTPUT
 */
#ifndef __MXC_V4L2_OUTPUT_H__
#define __MXC_V4L2_OUTPUT_H__

#include <linux/list.h>
#include <linux/spinlock.h>
#include <media/v4l2-dev.h>

#include <mach/ipu.h>
#include <mach/mxc_v4l2.h>

#define MIN_FRAME_NUM 2
#define MAX_FRAME_NUM 30

#define MXC_V4L2_OUT_NUM_OUTPUTS	5
#define MXC_V4L2_OUT_2_SDC		0
#define MXC_V4L2_OUT_2_ADC		1

struct dma_buf_desc {
	dma_addr_t dma_addr;
	void *cpu_addr;
	size_t size;
};

typedef enum {
	BUF_IDLE,
	BUF_DONE,
	BUF_CANCELED,
	BUF_ERROR,
	BUF_BUSY,
} v4l_buf_state_t;

typedef struct v4l_queue {
	struct v4l2_buffer buf;
	struct list_head head;
	v4l_buf_state_t state;
	struct dma_buf_desc dma_desc;
} v4l_queue_t;

/*!
 * States for the video stream
 */
typedef enum {
	STATE_STREAM_OFF,
	STATE_STREAM_ON,
	STATE_STREAM_PAUSED,
	STATE_STREAM_STOPPING,
} v4lout_state;

/*
 * States for tearing protection
 */
typedef enum {
	TEARING_PROTECTION_INACTIVE,
	TEARING_PROTECTION_ACTIVE,
	TEARING_PROTECTION_UNSUPPORTED
} v4l_tear_protect;

/*
 * common v4l2 driver structure.
 */
typedef struct _vout_data {
	struct video_device *video_dev;
	/*!
	 * semaphore guard against SMP multithreading
	 */
	struct semaphore user_lock;
	spinlock_t irq_lock;

	struct notifier_block fb_event_notifier;

	/*!
	 * number of process that have device open
	 */
	int open_count;

	v4l_tear_protect tear_protection;

	/*!
	 * params lock for this camera
	 */
	struct semaphore param_lock;

	struct timer_list output_timer;
	unsigned long start_jiffies;
	u32 frame_count;
	struct list_head ready_q;
	struct list_head done_q;
	struct list_head free_q;
	struct v4l_queue *active;

	s8 next_rdy_ipu_buf;
	s8 next_done_ipu_buf;
	s8 ipu_buf[2];
	v4lout_state state;
	int busy;

	int cur_disp_output;
	int output_fb_num[MXC_V4L2_OUT_NUM_OUTPUTS];
	int output_enabled[MXC_V4L2_OUT_NUM_OUTPUTS];
	struct v4l2_framebuffer v4l2_fb;
#if defined(CONFIG_VIDEO_MXC_IPU_OUTPUT) || defined(CONFIG_VIDEO_MXC_IPU_OUTPUT_MODULE)
	ipu_channel_t display_ch;
	ipu_channel_t post_proc_ch;
#endif
	int buffer_cnt;
	struct list_head display_q;
	u32 display_buf_size;
	dma_addr_t display_bufs[2];

	/*!
	 * Poll wait queue
	 */
	wait_queue_head_t v4l_bufq;

	/*!
	 * v4l2 format
	 */
	struct v4l2_format v2f;
	struct v4l2_mxc_offset offset;
	struct v4l2_rect crop_rect;
	ipu_rotate_mode_t rotate;

	/* crop */
	struct v4l2_rect crop_bounds[MXC_V4L2_OUT_NUM_OUTPUTS];
	struct v4l2_rect crop_current;
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	int output_fb;
	int fb_enabled;
	int pp_ready;
#endif
#if 1
	int queued;
	struct timeval frame_start;
#endif
} vout_data;

#endif	/* __MXC_V4L2_OUTPUT_H__ */
