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

/*
 * file plat-mxc/sdma/sdma.c
 * This file contains functions for Smart DMA  API
 *
 * SDMA (Smart DMA) is used for transferring data between MCU and peripherals
 *
 */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include <iapi.h>
#include <epm.h>
#include "sdma.h"

#define M3_BASE_ADDRESS CSD0_BASE_ADDR
#define CHAD(ch) sdma_data[0].cd->ccb_ptr[ch].channelDescriptor

void __iomem *sdma_base_addr = NULL;

/*
 * SDMA status mutex
 */
static struct semaphore sdma_status_mutex;

/*
 * SDMA channel sleep queues
 */
//static struct semaphore sdma_sleep_mutex[MAX_DMA_CHANNELS];
static wait_queue_head_t sdma_sleep_queue[MAX_DMA_CHANNELS];

/*
 * SDMA channel synchronization
 */
static struct semaphore sdma_synch_mutex[MAX_DMA_CHANNELS];

struct clk *mxc_sdma_clk;

/*
 * Structure containing sdma channels information.
 */
typedef struct {
	/* Channel number */
	int channel;
	/* Channel usage name */
	int in_use;
	/* Name of device using the channel */
	char devicename[MAX_DEVNAME_LENGTH];
	/* Transfer type. Needed for setting SDMA script */
	sdma_transferT transfer_type;
	/* Peripheral type. Needed for setting SDMA script */
	sdma_periphT peripheral_type;
	/* Watermark level of device's fifo */
	__u32 watermark_level;
	/* Peripheral event id */
	int event_id;
	/* Peripheral event id2 (for channels that use 2 events) */
	int event_id2;
	/* Running status (boolean)  */
	int running;
	/* buffer descriptors number */
	int bd_number;
	/*   callback function       */
	dma_callback_t callback;
	/*   callback argument       */
	void *arg;
	/* SDMA data access word size */
	unsigned long word_size:8;
	/* channel descriptor pointer */
	channelDescriptor *cd;
} sdma_struct;

/*
 * Used to save the status of channels.
 */
static sdma_struct sdma_data[MAX_DMA_CHANNELS];

/*
 * Stores the start address of the SDMA scripts
 */
static sdma_script_start_addrs sdma_script_addrs;

extern void mxc_sdma_get_script_info(sdma_script_start_addrs *sdma_script_add);

/*
 * Init sleep mutex of the channel
 *
 * @param  channel  channel number
 */
static void sdma_init_sleep(int channel)
{
	init_waitqueue_head(&sdma_sleep_queue[channel]);
}

#ifdef DEBUG
#define SHOW_REG(b, p, r)		__show_reg(b, p, r, #r)

static inline void __show_reg(void __iomem *base, unsigned long phys,
			unsigned int reg, const char *name)
{
	DBG(0, "%-12s[%08lx]=%08x\n", name, phys + reg, __raw_readl(base + reg));
}

void dump_sdma_regs(void)
{
	unsigned int reg;

	for (reg = 0; reg < 0x180; reg += 4) {
		SHOW_REG(sdma_base_addr, SDMA_BASE_ADDR, reg);
	}
}
#endif

/*
 * Puts channel to sleep
 *
 * @param  channel  channel number
 */
#include <linux/delay.h>

static void sdma_sleep_channel(int channel)
{
	int ret;

	DBG(1, "%s: Waiting for channel %d to drain\n",
		__FUNCTION__, channel);
	BUG_ON(irqs_disabled());

	ret = wait_event_interruptible_timeout(sdma_sleep_queue[channel],
					iapi_SDMAIntr & (1 << channel), HZ);
	if (ret == 0 && !(iapi_SDMAIntr & (1 << channel))) {
		DBG(-1, "%s: Wait for channel %d done timed out: %08lx\n",
			__FUNCTION__, channel, iapi_SDMAIntr);
	} else if (ret < 0) {
		DBG(-1, "%s: Wait for channel %d aborted: %08lx, %d\n",
			__FUNCTION__, channel, iapi_SDMAIntr, ret);
	} else {
		DBG(1, "%s: Wait for channel %d done finished after %u ticks: %08lx\n",
			__FUNCTION__, channel, HZ - ret, iapi_SDMAIntr);
	}
}

/*
 * Wake up channel from sleep
 *
 * @param  channel  channel number
 */
static void sdma_wakeup_channel(int channel)
{
	DBG(1, "%s: Waking up channel %d\n", __FUNCTION__, channel);
	wake_up_interruptible(&sdma_sleep_queue[channel]);
}

/*
 * Sdma interrupt handler routine.
 * Calls channels callback function
 *
 * @param   irq    the interrupt number
 * @param   dev_id driver private data
 * @return the function returns \b IRQ_RETVAL(1) -  interrupt was handled
 */
static irqreturn_t sdma_int_handler(int irq, void *dev_id)
{
	DBG(2, "%s: IRQ %d\n", __FUNCTION__, irq);
	IRQ_Handler();
	DBG(2, "%s: Done\n", __FUNCTION__);
	return IRQ_HANDLED;
}

/*
 * I.API channel callback function
 *
 * @param   cd            channel descriptor structure
 * @param   channel_data  SDMA struct of the current channel
 */
static void iapi_interrupt_callback(channelDescriptor *cd,
				    sdma_struct *channel_data)
{
	int channel;
	dma_callback_t callback;
	void *arg;

	DBG(2, "%s: cd=%p\n", __FUNCTION__, cd);
	channel = channel_data->channel;

	channel_data->running = 0;

	arg = channel_data->arg;

	if (arg == NULL) {
		arg = &channel;
	}

	callback = channel_data->callback;

	if (callback != NULL) {
		DBG(2, "%s: Calling %p\n", __FUNCTION__, callback);
		callback(arg);
	}
}

/*
 * Returns pc of SDMA script according to peripheral and transfer type
 *
 * @param   peripheral_type   peripheral type
 * @param   transfer_type     transfer type
 *
 * @return  PC of SDMA script
*/
static unsigned short sdma_get_pc(sdma_periphT peripheral_type,
				  sdma_transferT transfer_type)
{
	int res = 0;

	if (peripheral_type == MEMORY) {
		switch (transfer_type) {
		case emi_2_int:
			res = sdma_script_addrs.mxc_sdma_ap_2_ap_addr;
			break;
		case emi_2_emi:
			res = sdma_script_addrs.mxc_sdma_ap_2_ap_addr;
			break;
		case int_2_emi:
			res = sdma_script_addrs.mxc_sdma_ap_2_ap_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == DSP) {
		switch (transfer_type) {
		case emi_2_dsp:
			res = sdma_script_addrs.mxc_sdma_ap_2_bp_addr;
			break;
		case dsp_2_emi:
			res = sdma_script_addrs.mxc_sdma_bp_2_ap_addr;
			break;
		case dsp_2_emi_loop:
			res =
			    sdma_script_addrs.
			    mxc_sdma_loopback_on_dsp_side_addr;
			break;
		case emi_2_dsp_loop:
			res =
			    sdma_script_addrs.mxc_sdma_mcu_interrupt_only_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == FIRI) {
		switch (transfer_type) {
		case per_2_int:
			res = sdma_script_addrs.mxc_sdma_firi_2_per_addr;
			break;
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_firi_2_mcu_addr;
			break;
		case int_2_per:
			res = sdma_script_addrs.mxc_sdma_per_2_firi_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_firi_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == UART) {
		switch (transfer_type) {
		case per_2_int:
			res = sdma_script_addrs.mxc_sdma_uart_2_per_addr;
			break;
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_uart_2_mcu_addr;
			break;
		case int_2_per:
			res = sdma_script_addrs.mxc_sdma_per_2_app_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_app_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == UART_SP) {
		switch (transfer_type) {
		case per_2_int:
			res = sdma_script_addrs.mxc_sdma_uartsh_2_per_addr;
			break;
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_uartsh_2_mcu_addr;
			break;
		case int_2_per:
			res = sdma_script_addrs.mxc_sdma_per_2_shp_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_shp_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == ATA) {
		switch (transfer_type) {
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_ata_2_mcu_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_ata_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == CSPI || peripheral_type == EXT ||
		   peripheral_type == SSI) {
		switch (transfer_type) {
		case per_2_int:
			res = sdma_script_addrs.mxc_sdma_app_2_per_addr;
			break;
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_app_2_mcu_addr;
			break;
		case int_2_per:
			res = sdma_script_addrs.mxc_sdma_per_2_app_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_app_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == SSI_SP || peripheral_type == MMC ||
		   peripheral_type == SDHC || peripheral_type == CSPI_SP ||
		   peripheral_type == ESAI || peripheral_type == MSHC_SP) {
		switch (transfer_type) {
		case per_2_int:
			res = sdma_script_addrs.mxc_sdma_shp_2_per_addr;
			break;
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_shp_2_mcu_addr;
			break;
		case int_2_per:
			res = sdma_script_addrs.mxc_sdma_per_2_shp_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_shp_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == ASRC) {
		switch (transfer_type) {
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_asrc_2_mcu_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_asrc_2_mcu_addr;
			break;
		case per_2_per:
			res = sdma_script_addrs.mxc_sdma_per_2_per_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == MSHC) {
		switch (transfer_type) {
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_mshc_2_mcu_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_mshc_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == CCM) {
		switch (transfer_type) {
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_dptc_dvfs_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == FIFO_MEMORY) {
		res = sdma_script_addrs.mxc_sdma_ap_2_ap_fixed_addr;
	} else if (peripheral_type == SPDIF) {
		switch (transfer_type) {
		case per_2_emi:
			res = sdma_script_addrs.mxc_sdma_spdif_2_mcu_addr;
			break;
		case emi_2_per:
			res = sdma_script_addrs.mxc_sdma_mcu_2_spdif_addr;
			break;
		default:
			res = -EINVAL;
		}
	} else if (peripheral_type == IPU_MEMORY) {
		if (transfer_type == emi_2_per) {
			res = sdma_script_addrs.mxc_sdma_ext_mem_2_ipu_addr;
		} else {
			res = -EINVAL;
		}
	}

	if (res < 0) {
		printk(KERN_ERR "SDMA script not found\n");
	} else {
		DBG(1, "%s: PC[%d,%d]=%04x\n", __FUNCTION__,
			peripheral_type, transfer_type, res);
	}
	return res;

}

static inline int sdma_asrc_set_info(dma_channel_params *p,
				     script_data *pcontext, int eflags)
{
	dma_channel_ext_params *ep = (dma_channel_ext_params *)p;
	unsigned int wml, tmp, wml1, wml2;
	struct dma_channel_asrc_info *info = &ep->info.asrc;

	wml = 0;
	if (p->transfer_type == per_2_per) {
		if (!p->ext)
			return wml;
		wml1 = p->watermark_level;
		wml2 = ep->watermark_level2;
		if (info->channs) {
			wml |= (info->channs & SDMA_ASRC_INFO_N_MASK) <<
			    SDMA_ASRC_INFO_N_OFF;
			if (ep->p2p_dir)
				wml2 *= info->channs & SDMA_ASRC_INFO_N_MASK;
			else
				wml1 *= info->channs & SDMA_ASRC_INFO_N_MASK;
		}
		if (info->channs & 1) {
			if (ep->p2p_dir)
				wml |= SDMA_ASRC_P2P_INFO_PS;
			else
				wml |= SDMA_ASRC_P2P_INFO_PA;
		}
		if (wml1 > wml2) {
			tmp = wml2 & SDMA_ASRC_P2P_INFO_LWML_MASK;
			wml |= tmp << SDMA_ASRC_P2P_INFO_LWML_OFF;
			tmp = wml1 & SDMA_ASRC_P2P_INFO_HWML_MASK;
			wml |= tmp << SDMA_ASRC_P2P_INFO_HWML_OFF;
			if (eflags & (1 << 31))
				wml |= SDMA_ASRC_P2P_INFO_LWE;
			if (eflags & (1 << 30))
				wml |= SDMA_ASRC_P2P_INFO_HWE;
		} else {
			tmp = wml1 & SDMA_ASRC_P2P_INFO_LWML_MASK;
			wml |= tmp << SDMA_ASRC_P2P_INFO_LWML_OFF;
			tmp = wml2 & SDMA_ASRC_P2P_INFO_HWML_MASK;
			wml |= tmp << SDMA_ASRC_P2P_INFO_HWML_OFF;
			wml |= eflags >> 2;
			tmp = pcontext->event_mask2;
			pcontext->event_mask2 = pcontext->event_mask1;
			pcontext->event_mask1 = tmp;
		}
	} else {
		if (p->ext && info->channs) {
			wml |= (info->channs & SDMA_ASRC_INFO_N_MASK) <<
			    SDMA_ASRC_INFO_N_OFF;
			tmp = (info->channs * p->watermark_level) &
			    SDMA_ASRC_INFO_WML_MASK;
			wml |= tmp << SDMA_ASRC_INFO_WML_OFF;
		} else {
			tmp = (p->watermark_level & SDMA_ASRC_INFO_WML_MASK);
			wml |= tmp << SDMA_ASRC_INFO_WML_OFF;
		}

		if (p->transfer_type == per_2_emi)
			wml |= SDMA_ASRC_INFO_TXFR_DIR;

		if (p->ext && (info->channs & 1)) {
			if (p->transfer_type == per_2_emi)
				wml |= SDMA_ASRC_INFO_PS;
			else
				wml |= SDMA_ASRC_INFO_PA;
		}
		wml |= eflags;
	}
	return wml;
}

/*
 * Downloads channel context according to channel parameters
 *
 * @param   channel           channel number
 * @param   p                 channel parameters
 */
static int sdma_load_context(int channel, dma_channel_params *p)
{
	script_data context;
	int res;
	int event1_greater_than_32;
	int event2_greater_than_32;
	dma_channel_ext_params *ep = (dma_channel_ext_params *)p;

	res = 0;

	memset(&context, 0, sizeof(script_data));
	context.load_address = sdma_get_pc(p->peripheral_type,
					   p->transfer_type);

	if (context.load_address > 0) {
		if ((p->peripheral_type != MEMORY) &&
			(p->peripheral_type != DSP)) {
			/* Handle multiple event channels differently */
			if (p->event_id2) {
				if (p->event_id2 < 32) {
					context.event_mask2 =
					    0x1 << p->event_id2;
					event2_greater_than_32 = 0;
				} else {
					context.event_mask2 =
					    0x1 << (p->event_id2 - 32);
					event2_greater_than_32 = 1 << 31;
				}
				if (p->event_id < 32) {
					context.event_mask1 =
					    0x1 << p->event_id;
					event1_greater_than_32 = 0;
				} else {
					context.event_mask1 =
					    0x1 << (p->event_id - 32);
					event1_greater_than_32 = 1 << 30;
				}
			} else {
				event1_greater_than_32 = 0;
				event2_greater_than_32 = 0;
				if (p->event_id < 32) {
					context.event_mask1 =
					    0x1 << p->event_id;
					context.event_mask2 = 0;
				} else {
					context.event_mask1 = 0;
					context.event_mask2 =
					    0x1 << (p->event_id - 32);
				}
			}

			if (p->ext)
				context.wml = ep->info_bits;
			/* Watermark Level */
			if (p->peripheral_type == ASRC) {
				context.wml |= sdma_asrc_set_info(p,
								  &context,
								  event2_greater_than_32
								  |
								  event1_greater_than_32);
			} else
				context.wml |= event2_greater_than_32 |
					event1_greater_than_32 | p->watermark_level;

			/* Address */
			context.shp_addr = p->per_address;
			iapi_IoCtl(sdma_data[channel].cd,
				   IAPI_CHANGE_PERIPHADDR, p->per_address);
		} else {
			context.wml = M3_BASE_ADDRESS;
		}

		sdma_data[channel].transfer_type = p->transfer_type;
		sdma_data[channel].peripheral_type = p->peripheral_type;
		sdma_data[channel].watermark_level = p->watermark_level;
		iapi_AssignScript(sdma_data[channel].cd, &context);
	} else {
		res = context.load_address;
	}

	return res;
}

/*
 * Setup channel according to parameters. Must be called once after mxc_request_dma()
 *
 * @param   channel           channel number
 * @param   p                 channel parameters pointer
 * @return  0 on success, error code on fail
 */
int mxc_dma_setup_channel(int channel, dma_channel_params *p)
{
	int err = 0;
	int i;

	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	mxc_dma_stop(channel);

	for (i = 0; i < sdma_data[channel].bd_number; i++) {
		iapi_IoCtl(sdma_data[channel].cd,
			   (i << BD_NUM_OFFSET) |
			   IAPI_CHANGE_SET_STATUS, 0);
	}

	DBG(0, "%s: Changing number of BDs for channel %d from %u to %u\n",
		__FUNCTION__, channel, sdma_data[channel].bd_number,
		p->bd_number <= 0 ? 1 : p->bd_number);
	sdma_data[channel].bd_number = (p->bd_number <= 0) ? 1 : p->bd_number;

	sdma_data[channel].word_size = p->word_size;

	sdma_data[channel].event_id = p->event_id;
	sdma_data[channel].event_id2 = p->event_id2;

	sdma_data[channel].callback = p->callback;

	sdma_data[channel].arg = p->arg;

	err = iapi_IoCtl(sdma_data[channel].cd,
			 IAPI_CHANGE_BDNUM, sdma_data[channel].bd_number);
	if (err < 0) {
		printk(KERN_ERR "Failed to allocate buffer descriptors: %d\n",
			err);
		err = -ENOMEM;
		goto setup_channel_fail;
	}

	if (channel != 0) {
		switch (p->transfer_type) {
		case dsp_2_per:
			break;
		case emi_2_per:
		case int_2_per:
		case per_2_int:
		case per_2_emi:
		case per_2_per:
			/*
			 * Peripheral <------> Memory
			 * evtOvr = 0 dspOvr = 1
			 */
			err = iapi_IoCtl(sdma_data[channel].cd,
				IAPI_CHANGE_OWNERSHIP,
				(OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT) |
				(OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU) |
				(DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP));
			if (err)
				goto setup_channel_fail;
			if (p->event_id) {
				err = iapi_SetChannelEventMapping(p->event_id,
								1 << channel);
				if (err)
					goto setup_channel_fail;
			}
			if (p->event_id2) {
				err = iapi_SetChannelEventMapping(p->event_id2,
								1 << channel);
			}
			break;
		case emi_2_dsp:
		case int_2_dsp:
		case dsp_2_int:
		case dsp_2_emi:
		case dsp_2_dsp:
			/*
			 * DSP <-----------> Memory
			 * evtOvr = 1 dspOvr = 0
			 */
			err = iapi_IoCtl(sdma_data[channel].cd,
					IAPI_CHANGE_OWNERSHIP,
					(DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT) |
					(OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU) |
					(OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP));
			break;
		case emi_2_int:
		case emi_2_emi:
		case int_2_int:
		case int_2_emi:
		case emi_2_dsp_loop:
		case dsp_2_emi_loop:
			/* evtOvr = 1 dspOvr = 1 */
			err = iapi_IoCtl(sdma_data[channel].cd,
				IAPI_CHANGE_OWNERSHIP,
				(DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT) |
				(OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU) |
				(DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP));
			break;
		case per_2_dsp:
			/* evtOvr = 0 dspOvr = 0 */
			err = iapi_IoCtl(sdma_data[channel].cd,
				IAPI_CHANGE_OWNERSHIP,
				(OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT) |
				(DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU) |
				(OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP));
			if (err)
				goto setup_channel_fail;
			err = iapi_SetChannelEventMapping(p->event_id,
							1 << channel);
			break;
		default:
			break;
			printk(KERN_ERR "Wrong SDMA transfer type\n");
			err = -EINVAL;
		}
		if (err)
			goto setup_channel_fail;

		err = sdma_load_context(channel, p);
		if (err)
			goto setup_channel_fail;

		err = iapi_IoCtl(sdma_data[channel].cd, IAPI_CHANGE_PRIORITY,
				MXC_SDMA_DEFAULT_PRIORITY);
	}
setup_channel_fail:
	return err;
}
EXPORT_SYMBOL(mxc_dma_setup_channel);

/*
 * Setup the channel priority. This can be used to change the default priority
 * for the channel.
 *
 * @param   channel           channel number
 * @param   priority          priority to be set for the channel
 *
 * @return  0 on success, error code on failure
 */
int mxc_dma_set_channel_priority(unsigned int channel, unsigned int priority)
{
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	if (priority < MXC_SDMA_MIN_PRIORITY
	    || priority > MXC_SDMA_MAX_PRIORITY) {
		return -EINVAL;
	}
	return iapi_IoCtl(sdma_data[channel].cd, IAPI_CHANGE_PRIORITY,
			  priority);
}
EXPORT_SYMBOL(mxc_dma_set_channel_priority);

/*
 * Allocates dma channel.
 * If channel's value is 0, then the function allocates a free channel
 * dynamically and sets its value to channel.
 * Else allocates requested channel if it is free.
 * If the channel is busy or no free channels (in dynamic allocation) -EBUSY returned.
 *
 * @param   channel           pointer to channel number
 * @param   devicename        device name
 * @return  0 on success, error code on fail
 */
int mxc_request_dma(int *channel, const char *devicename)
{
	int i, res = -EBUSY;

	if (sdma_base_addr == NULL)
		return -ENOSYS;

	if (channel == NULL || *channel < 0 || *channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	down(&sdma_status_mutex);

	/* Dynamic allocation */
	if (*channel == 0) {
		for (i = MAX_DMA_CHANNELS - 1; i > 0; i--) {
#ifdef CONFIG_SDMA_IRAM
			/* TODO: It will be removed after DPTC uses UDMA interface */
			if (i >= MXC_DMA_CHANNEL_IRAM)
				continue;
#endif /* CONFIG_SDMA_IRAM */
			if (!sdma_data[i].in_use) {
				*channel = i;
				res = 0;
				break;
			}
		}
	} else {
		if (sdma_data[*channel].in_use)
			return -EBUSY;
		res = 0;
	}
	if (res)
		return res;

	DBG(1, "%s: Opening channel %d\n", __FUNCTION__, *channel);

	res = iapi_Open(sdma_data[0].cd, *channel);
	if (res < 0) {
		printk(KERN_ERR "Failed iapi_Open channel %d, 0x%x\n",
			*channel, res);
	} else {
		sdma_data[*channel].in_use = 1;
		strlcpy(sdma_data[*channel].devicename, devicename,
			sizeof(sdma_data[*channel].devicename));
		sdma_data[*channel].cd = CHAD(*channel);

		iapi_IoCtl(sdma_data[*channel].cd, IAPI_CHANGE_SYNCH,
			CALLBACK_ISR);
		iapi_IoCtl(sdma_data[*channel].cd,
			IAPI_CHANGE_CALLBACKFUNC,
			(unsigned long)iapi_interrupt_callback);
		iapi_IoCtl(sdma_data[*channel].cd,
			IAPI_CHANGE_USER_ARG,
			(unsigned long)&sdma_data[*channel]);
	}

	up(&sdma_status_mutex);

	return res;
}
EXPORT_SYMBOL(mxc_request_dma);

/*
 * Configures request parameters. Can be called multiple times after
 * mxc_request_dma() and mxc_dma_setup_channel().
 *
 *
 * @param   channel           channel number
 * @param   p                 request parameters pointer
 * @param   bd_index          index of buffer descriptor to set
 * @return  0 on success, error code on fail
 */
int mxc_dma_set_config(int channel, dma_request_t *p, int bd_index)
{
	int ret;
	unsigned long param;

	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	if (!sdma_data[channel].in_use) {
		return -EINVAL;
	}
	DBG(0, "%s: dma_request: src: %08x dst: %08x cont=%d wrap=%d\n",
		__FUNCTION__, p->sourceAddr, p->destAddr,
		p->bd_cont, p->bd_wrap);

	ret = iapi_IoCtl(sdma_data[channel].cd,
			(bd_index << BD_NUM_OFFSET) |
			IAPI_CHANGE_SET_TRANSFER_CD, sdma_data[channel].word_size);
	if (ret < 0)
		return ret;

	param = BD_DONE | BD_INTR | BD_EXTD;

	if (sdma_data[channel].bd_number > 1 && p->bd_cont) {
		param |= BD_CONT;
	}

	if (bd_index == sdma_data[channel].bd_number - 1 || p->bd_wrap) {
		param |= BD_WRAP;
	}

	switch (sdma_data[channel].transfer_type) {
	case emi_2_per:
	case dsp_2_per:
	case int_2_per:
	case emi_2_dsp:
	case int_2_dsp:
	case emi_2_dsp_loop:
		ret = iapi_IoCtl(sdma_data[channel].cd,
				(bd_index << BD_NUM_OFFSET) |
				IAPI_CHANGE_SET_BUFFERADDR,
				(unsigned long)p->sourceAddr);
		break;
	case per_2_int:
	case per_2_emi:
	case per_2_dsp:
	case dsp_2_int:
	case dsp_2_emi:
	case dsp_2_dsp:
	case dsp_2_emi_loop:
		ret = iapi_IoCtl(sdma_data[channel].cd,
				(bd_index << BD_NUM_OFFSET) |
				IAPI_CHANGE_SET_BUFFERADDR,
				(unsigned long)p->destAddr);
		break;
	case emi_2_int:
	case emi_2_emi:
	case int_2_int:
	case int_2_emi:
		ret = iapi_IoCtl(sdma_data[channel].cd,
			   (bd_index << BD_NUM_OFFSET) |
			   IAPI_CHANGE_SET_BUFFERADDR,
			   (unsigned long)p->sourceAddr);
		if (ret < 0)
			return ret;
		ret = iapi_IoCtl(sdma_data[channel].cd,
			   (bd_index << BD_NUM_OFFSET) |
			   IAPI_CHANGE_SET_EXTDBUFFERADDR,
			   (unsigned long)p->destAddr);
		break;
	case per_2_per:
	case dvfs_pll:
	case dvfs_pdr:
		return -EINVAL;
	}
	if (ret < 0)
		return ret;

	/* Change the endianness for DSP to MCU Data transfers */
	if (sdma_data[channel].transfer_type == dsp_2_emi ||
		sdma_data[channel].transfer_type == emi_2_dsp) {
		ret = iapi_IoCtl(sdma_data[channel].cd,
				IAPI_CHANGE_SET_ENDIANNESS,
				SET_BIT_ALL);
		if (ret < 0)
			return ret;
	}

	ret = iapi_IoCtl(sdma_data[channel].cd,
			(bd_index << BD_NUM_OFFSET) |
			IAPI_CHANGE_SET_COUNT, p->count);
	if (ret < 0)
		return ret;

	ret = iapi_IoCtl(sdma_data[channel].cd,
			(bd_index << BD_NUM_OFFSET) | IAPI_CHANGE_SET_STATUS,
			param);
	return ret;
}
EXPORT_SYMBOL(mxc_dma_set_config);

/*
 * Configures the BD_INTR bit on a buffer descriptor parameters.
 *
 *
 * @param   channel           channel number
 * @param   bd_index          index of buffer descriptor to set
 * @param   bd_intr           flag to set or clear the BD_INTR bit
 * @return  0 on success, error code on fail
 */
void mxc_dma_set_bd_intr(int channel, int bd_index, int bd_intr)
{
	unsigned long param;

	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return;

	iapi_IoCtl(sdma_data[channel].cd,
		   (bd_index << BD_NUM_OFFSET) |
		   IAPI_CHANGE_GET_STATUS, (unsigned long)&param);

	if (bd_intr) {
		param |= BD_INTR;
	} else {
		param &= ~BD_INTR;
	}
	iapi_IoCtl(sdma_data[channel].cd,
		   (bd_index << BD_NUM_OFFSET) | IAPI_CHANGE_SET_STATUS, param);
}
EXPORT_SYMBOL(mxc_dma_set_bd_intr);

/*
 * Gets the BD_INTR bit on a buffer descriptor.
 *
 *
 * @param   channel           channel number
 * @param   bd_index          index of buffer descriptor to set
 *
 * @return returns the BD_INTR bit status
 */
int mxc_dma_get_bd_intr(int channel, int bd_index)
{
	int ret;
	unsigned long bd_status = 0;

	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	ret = iapi_IoCtl(sdma_data[channel].cd,
			(bd_index << BD_NUM_OFFSET) |
			IAPI_CHANGE_GET_STATUS, (unsigned long)&bd_status);

	return ret == 0 ? !!(bd_status & BD_INTR) : ret;
}
EXPORT_SYMBOL(mxc_dma_get_bd_intr);

/*
 * Stop the current transfer
 *
 * @param   channel           channel number
 * @param   buffer_number     number of buffers (beginning with 0),
 *                            whose done bits should be reset to 0
 */
int mxc_dma_reset(int channel, int buffer_number)
{
	unsigned char param = 0;
	int i = 0;

	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	if (!sdma_data[channel].in_use) {
		return -EINVAL;
	}

	/* clear the BD_DONE bits for all the necessary buffers */
	for (i = 0; i < buffer_number; i++) {

		iapi_IoCtl(sdma_data[channel].cd, (i << BD_NUM_OFFSET) |
			   IAPI_CHANGE_GET_STATUS, (unsigned long)&param);

		/* clear the BD_DONE bit of the buffer */
		param &= ~BD_DONE;

		iapi_IoCtl(sdma_data[channel].cd, (i << BD_NUM_OFFSET) |
			   IAPI_CHANGE_SET_STATUS, param);
	}

	return 0;
}
EXPORT_SYMBOL(mxc_dma_reset);

/*
 * Returns request parameters.
 *
 * @param   channel           channel number
 * @param   p                 request parameters pointer
 * @param   bd_index          index of buffer descriptor to get
 * @return  0 on success, error code on fail
 */
int mxc_dma_get_config(int channel, dma_request_t *p, int bd_index)
{
	int err = 0;
	unsigned long bd_status;
	unsigned long bd_count;
	dma_addr_t sourceAddr;
	dma_addr_t destAddr;

	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	memset(p, 0, sizeof(*p));

	err = iapi_IoCtl(sdma_data[channel].cd,
			(bd_index << BD_NUM_OFFSET) |
			IAPI_CHANGE_GET_STATUS, (unsigned long)&bd_status);
	if (err)
		return err;
	err = iapi_IoCtl(sdma_data[channel].cd,
			(bd_index << BD_NUM_OFFSET) |
			IAPI_CHANGE_GET_COUNT, (unsigned long)&bd_count);
	if (err)
		return err;
	err = iapi_IoCtl(sdma_data[channel].cd,
			(bd_index << BD_NUM_OFFSET) |
			IAPI_CHANGE_GET_BUFFERADDR, (unsigned long)&sourceAddr);
	if (err)
		return err;

	switch (sdma_data[channel].transfer_type) {
	case emi_2_per:
	case dsp_2_per:
	case int_2_per:
	case emi_2_dsp:
	case int_2_dsp:
	case emi_2_dsp_loop:
		p->sourceAddr = sourceAddr;
		break;
	case per_2_int:
	case per_2_emi:
	case per_2_dsp:
	case dsp_2_int:
	case dsp_2_emi:
	case dsp_2_dsp:
	case dsp_2_emi_loop:
		p->destAddr = sourceAddr;
		break;
	case emi_2_int:
	case emi_2_emi:
	case int_2_int:
	case int_2_emi:
		p->sourceAddr = sourceAddr;
		err = iapi_IoCtl(sdma_data[channel].cd,
				(bd_index << BD_NUM_OFFSET) |
				IAPI_CHANGE_GET_EXTDBUFFERADDR,
				(unsigned long)&destAddr);
		if (err)
			return err;
		p->destAddr = destAddr;
		break;
	default:
		break;
	}

	p->count = bd_count;
	p->bd_done = !!(bd_status & BD_DONE);
	p->bd_cont = !!(bd_status & BD_CONT);
	p->bd_error = !!(bd_status & BD_RROR);
	p->bd_wrap = !!(bd_status & BD_WRAP);

	return err;
}
EXPORT_SYMBOL(mxc_dma_get_config);

/*
 * This function is used by MXC IPC's write_ex2. It passes the pointer to the
 * data control structure to iapi_write_ipcv2()
 *
 * @param channel  SDMA channel number
 * @param ctrl_ptr Data Control structure pointer
 */
int mxc_sdma_write_ipcv2(int channel, void *ctrl_ptr)
{
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;
	return iapi_Write_ipcv2(sdma_data[channel].cd, ctrl_ptr);
}
EXPORT_SYMBOL(mxc_sdma_write_ipcv2);

/*
 * This function is used by MXC IPC's read_ex2. It passes the pointer to the
 * data control structure to iapi_read_ipcv2()
 *
 * @param channel   SDMA channel number
 * @param ctrl_ptr  Data Control structure pointer
 */
int mxc_sdma_read_ipcv2(int channel, void *ctrl_ptr)
{
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;
	return iapi_Read_ipcv2(sdma_data[channel].cd, ctrl_ptr);
}
EXPORT_SYMBOL(mxc_sdma_read_ipcv2);

/*
 * Starts dma channel.
 *
 * @param   channel           channel number
 */
int mxc_dma_start(int channel)
{
	DBG(1, "%s: Starting DMA channel %d\n", __FUNCTION__, channel);
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	if (!sdma_data[channel].running) {
		sdma_data[channel].running = 1;
		iapi_StartChannel(channel);
	}

	return 0;
}
EXPORT_SYMBOL(mxc_dma_start);

/*
 * Stops dma channel.
 *
 * @param   channel           channel number
 */
int mxc_dma_stop(int channel)
{
	DBG(1, "%s: Stopping DMA channel %d\n", __FUNCTION__, channel);
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	iapi_StopChannel(channel);
	sdma_data[channel].running = 0;

	return 0;
}
EXPORT_SYMBOL(mxc_dma_stop);

/*
 * Frees dma channel.
 *
 * @param   channel           channel number
 */
void mxc_free_sdma(int channel)
{
	int i;

	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return;

	mxc_dma_stop(channel);

	if (sdma_data[channel].event_id != 0) {
		iapi_SetChannelEventMapping(sdma_data[channel].event_id, 0x0);
	}
	if (sdma_data[channel].event_id2 != 0) {
		iapi_SetChannelEventMapping(sdma_data[channel].event_id2, 0x0);
	}

	sdma_data[channel].event_id = 0;

	iapi_IoCtl(sdma_data[channel].cd, IAPI_CHANGE_PRIORITY, 0x0);
	iapi_IoCtl(sdma_data[channel].cd, IAPI_CHANGE_OWNERSHIP,
		   (OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT) |
		   (OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU) |
		   (OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP));

	for (i = 0; i < sdma_data[channel].bd_number; i++) {
		iapi_IoCtl(sdma_data[channel].cd,
			   (i << BD_NUM_OFFSET) |
			   IAPI_CHANGE_SET_STATUS, 0);
	}

	iapi_Close(sdma_data[channel].cd);

	strlcpy(sdma_data[channel].devicename, "not used",
		sizeof(sdma_data[channel].devicename));

	sdma_data[channel].in_use = 0;
}
EXPORT_SYMBOL(mxc_free_sdma);

/*
 * Initializes channel's priorities
 *
 */
static void __init init_priorities(void)
{
	iapi_IoCtl(sdma_data[0].cd, IAPI_CHANGE_PRIORITY, 0x7);
}

/*
 * Initializes events table
 */
static void __init init_event_table(void)
{
	int channel;

	for (channel = 0; channel < MAX_DMA_CHANNELS; channel++) {
		iapi_SetChannelEventMapping(channel, 0);
	}
}

/*
 * Sets callback function. Used with standard dma api
 * for supporting interrupts
 *
 * @param   channel           channel number
 * @param   callback          callback function pointer
 * @param   arg               argument for callback function
 */
void mxc_dma_set_callback(int channel, dma_callback_t callback, void *arg)
{
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return;

	sdma_data[channel].callback = callback;
	sdma_data[channel].arg = arg;
}
EXPORT_SYMBOL(mxc_dma_set_callback);

/*
 * Synchronization function used by I.API
 *
 * @param channel        channel number
 */
static int getChannel(int channel)
{
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;

	if (irqs_disabled() || in_atomic()) {
		if (down_trylock(&sdma_synch_mutex[channel])) {
			return -EBUSY;
		}
	} else {
		if (down_interruptible(&sdma_synch_mutex[channel])) {
			return -EBUSY;
		}
	}

	return 0;
}

/*
 * Synchronization function used by I.API
 *
 * @param channel        channel number
 */
static int releaseChannel(int channel)
{
	if (channel < 0 || channel >= MAX_DMA_CHANNELS)
		return -EINVAL;
	up(&sdma_synch_mutex[channel]);
	return 0;
}

/*
 * Unmask interrupt function. Used by I.API
 *
 */
//static unsigned long flags;
static void unmask_sdma_interrupt(void)
{
	/* Commented out to take care of the PREEMPT_RT option
	 * local_irq_restore(flags);
	 */
}

/*
 * Mask interrupt function. Used by I.API
 *
 */
static void mask_sdma_interrupt(void)
{
	/* Commented to take of the PREEMPT_RT option
	 * local_irq_save(flags);
	 */
}

#ifdef DEBUG
static void *sdma_memcpy(void *dst, const void *src, size_t len)
{
	DBG(1, "%s: Copying %u byte from %p..%p to %p..%p\n", __FUNCTION__,
		len, src, src + len - 1, dst, dst + len - 1);
	memcpy(dst, src, len);
	return dst;
}
#endif

/*
 * Initializes I.API
 */
static int __init init_iapi_struct(void)
{
	printk(KERN_INFO "Using SDMA I.API\n");

	iapi_Malloc = &sdma_malloc;
#ifdef CONFIG_SDMA_IRAM
	iapi_iram_Malloc = &sdma_iram_malloc;
#endif	/* CONFIG_SDMA_IRAM */

	iapi_Free = &sdma_free;
	iapi_Virt2Phys = sdma_virt_to_phys;
	iapi_Phys2Virt = sdma_phys_to_virt;
	iapi_memset = &memset;
#ifndef DEBUG
	iapi_memcpy = &memcpy;
#else
	iapi_memcpy = &sdma_memcpy;
#endif
	iapi_GotoSleep = &sdma_sleep_channel;
	iapi_WakeUp = &sdma_wakeup_channel;
	iapi_InitSleep = &sdma_init_sleep;
	iapi_ReleaseChannel = &releaseChannel;
	iapi_GetChannel = &getChannel;

	iapi_EnableInterrupts = &unmask_sdma_interrupt;
	iapi_DisableInterrupts = &mask_sdma_interrupt;

	sdma_data[0].cd = kzalloc(sizeof(channelDescriptor), GFP_KERNEL);
	if (sdma_data[0].cd == NULL)
		return -ENOMEM;
	return 0;
}

/*
 * Initializes channel synchronization mutexes
 */
static void __init sdma_init_mutexes(void)
{
	int i;

	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		init_MUTEX(&sdma_synch_mutex[i]);
	}

	init_MUTEX(&sdma_status_mutex);
}

/*
 * Channels status read proc file system function
 *
 * @param    buf	pointer to the buffer the data shuld be written to.
 * @param    start	pointer to the pointer where the new data is
 *                      written to.
 *			procedure should update the start pointer to point to
 *			where in the buffer the data was written.
 * @param    offset	offset from start of the file
 * @param    count	number of bytes to read.
 * @param    eof	pointer to eof flag. sould be set to 1 when
 *                      reaching eof.
 * @param    data	driver specific data pointer.
 *
 * @return   number byte read from the log buffer.
 */
static int proc_read_channels(char *buf, char **start, off_t offset, int count,
			      int *eof, void *data)
{
	char *log;
	char *log_ptr;
	char tmp[48];
	int i;
//#define LOG_BUF_SIZE 4096

#ifdef LOG_BUF_SIZE
	log = kzalloc(LOG_BUF_SIZE, GFP_KERNEL);
	if (log == NULL) {
		return -ENOMEM;
	}
#else
	log = buf;
#endif
	log_ptr = log;

	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		if (sdma_data[i].in_use == 0) {
			continue;
		}

		memset(tmp, 0, sizeof(tmp));
		snprintf(tmp, sizeof(tmp), "Channel %d: %s\n", i,
			sdma_data[i].devicename);
#ifndef LOG_BUF_SIZE
		strlcpy(log_ptr, tmp, PAGE_SIZE - (log_ptr - log));
#else
		strlcpy(log_ptr, tmp, LOG_BUF_SIZE - (log_ptr - log));
#endif
		log_ptr += strlen(tmp);
		if (log_ptr - log >= PAGE_SIZE)
			break;
	}

	if (offset > (log_ptr - log)) {
		*eof = 1;
		count = 0;
	} else {
		if (offset + count > (log_ptr - log)) {
			count = (log_ptr - log) - offset;
			*eof = 1;
		} else {
			*eof = 0;
		}

		memcpy(buf, log, count);
		*start = buf;
	}
#ifdef LOG_BUF_SIZE
	kfree(log);
#endif
	return count;
}

/*
 * SDMA proc file system read function
 */
static int __init init_proc_fs(void)
{
	struct proc_dir_entry *sdma_proc_dir;
	int res;

	res = 0;

	sdma_proc_dir = proc_mkdir("sdma", NULL);
	create_proc_read_entry("channels", 0, sdma_proc_dir,
			       proc_read_channels, NULL);

	if (res < 0) {
		printk(KERN_WARNING "Failed create SDMA proc entry\n");
	}

	return res;
}

/*
 * Initializes SDMA private data
 */
static void __init init_sdma_data(void)
{
	int i;

	sdma_data[0].in_use = 1;
	strcpy(sdma_data[0].devicename, "MCU");

	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		sdma_data[i].channel = i;
	}
}

#if defined(CONFIG_MXC_SUPER_GEM)
/*
 * Initialize the Super GEM SDMA channel
 *
 * @return returns NO FUCKING -1 on error, 0 on success.
 */
static int __init init_super_gem(void)
{
	channelDescriptor *cd;
	script_data context;
	int res = 0;

	res = iapi_Open(sdma_data[0].cd, MXC_DMA_CHANNEL_GEM);
	if (res < 0) {
		return res;
	}
	sdma_data[MXC_DMA_CHANNEL_GEM].in_use = 1;
	cd = CHAD(MXC_DMA_CHANNEL_GEM);
	memset(&context, 0, sizeof(script_data));
	context.load_address = sdma_script_addrs.mxc_sdma_utra_addr;
	context.wml = M3_BASE_ADDRESS;
	res = iapi_AssignScript(cd, &context);
	if (res < 0) {
		iapi_Close(cd);
		sdma_data[MXC_DMA_CHANNEL_GEM].in_use = 0;
		return res;
	}
	res = iapi_IoCtl(cd, IAPI_CHANGE_OWNERSHIP,
			(OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT) |
			(DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU) |
			(OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP));
	if (res < 0) {
		iapi_Close(cd);
		sdma_data[MXC_DMA_CHANNEL_GEM].in_use = 0;
		return res;
	}
	/* Set EP=1, which is required to start SuperGem script the first time */
	/* This can be done only on the AP side */
	SDMA_H_EVTPEND |= 1 << MXC_DMA_CHANNEL_GEM;

	res = iapi_SetChannelEventMapping(DMA_REQ_GEM,
					1 << MXC_DMA_CHANNEL_GEM);
	if (res < 0) {
		iapi_Close(cd);
		sdma_data[MXC_DMA_CHANNEL_GEM].in_use = 0;
		return res;
	}

	return 0;
}
#endif

/*
 * Initializes dma
 */
int __init sdma_init(void)
{
	int res;
	configs_data confreg_data;
	struct clk *ahb_clk;

	sdma_base_addr = ioremap(SDMA_BASE_ADDR, SZ_16K);
	if (sdma_base_addr == NULL)
		return -ENOMEM;

	/* Initialize to the default values */
	confreg_data = iapi_ConfigDefaults;

#ifdef MXC_SDMA_DSPDMA
	confreg_data.dspdma = MXC_SDMA_DSPDMA;
#endif
	/* Set ACR bit */
	mxc_sdma_clk = clk_get(NULL, "sdma");
	if (IS_ERR(mxc_sdma_clk)) {
		res = PTR_ERR(mxc_sdma_clk);
		printk(KERN_EMERG "Failed to get SDMA clock: %d\n", res);
		goto clk_fail;
	}

	ahb_clk = clk_get(NULL, "ahb");
	if (IS_ERR(ahb_clk)) {
		res = PTR_ERR(ahb_clk);
		printk(KERN_EMERG "Failed to get SDMA clock: %d\n", res);
		goto clk_fail;
	}
	clk_enable(mxc_sdma_clk);
	clk_enable(ahb_clk);
	printk(KERN_INFO "AHB clock rate: %lu.%03luMHz SDMA clock rate: %lu.%03luMHz\n",
		clk_get_rate(ahb_clk) / 1000000,
		clk_get_rate(ahb_clk) / 1000 % 1000,
		clk_get_rate(mxc_sdma_clk) / 1000000,
		clk_get_rate(mxc_sdma_clk) / 1000 % 1000);
	if (clk_get_rate(ahb_clk) / clk_get_rate(mxc_sdma_clk) != 2) {
		printk(KERN_INFO "Setting SDMA ACR\n");
		confreg_data.acr = 1;
	}
	clk_disable(ahb_clk);
	clk_put(ahb_clk);

	init_sdma_data();

	init_sdma_pool();

	res = request_irq(MXC_INT_SDMA, sdma_int_handler, 0, "mxc-sdma", NULL);
	if (res) {
		printk(KERN_EMERG "Failed to get SDMA clock: %d\n", res);
		goto sdma_init_fail;
	}

	sdma_init_mutexes();

	res = init_iapi_struct();
	if (res)
		goto free_irq;

	mxc_sdma_get_script_info(&sdma_script_addrs);

	res = iapi_Init(sdma_data[0].cd, &confreg_data,
			sdma_script_addrs.mxc_sdma_start_addr,
			sdma_script_addrs.mxc_sdma_ram_code_size * 2,
			sdma_script_addrs.mxc_sdma_ram_code_start_addr, 0x50);
	if (res < 0) {
		printk(KERN_EMERG "Failed to init SDMA API: %d\n", res);
		goto free_mem;
	}

	init_priorities();

	init_event_table();

#if defined(CONFIG_MXC_SUPER_GEM)
	res = init_super_gem();
	if (res < 0) {
		goto free_mem;
	}
#endif

	init_proc_fs();

	printk(KERN_INFO "MXC SDMA API initialized\n");

	clk_disable(mxc_sdma_clk);
	return res;
free_mem:
	kfree(sdma_data[0].cd);
free_irq:
	free_irq(MXC_INT_SDMA, NULL);
sdma_init_fail:
	clk_disable(mxc_sdma_clk);
	clk_put(mxc_sdma_clk);
clk_fail:
	iounmap(sdma_base_addr);
	sdma_base_addr = NULL;
	printk(KERN_ERR "Error %d in sdma_init\n", res);
	return res;
}
arch_initcall(sdma_init);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Linux SDMA API");
MODULE_LICENSE("GPL");
