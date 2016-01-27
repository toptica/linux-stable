/******************************************************************************
 *
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 ******************************************************************************
 *
 * File: iapiMiddle.c
 *
 * $Id iapiMiddle.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the MIDDLE level functions of the I.API.
 *
 *
 *
 *
 * $Log iapiMiddle.c $
 *
 *****************************************************************************/


/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include <string.h>
#include <linux/io.h>

#include <epm.h>
#include <iapiLow.h>
#include <iapiMiddle.h>

/* ****************************************************************************
 * Global Variable Section
 *****************************************************************************/


/* ****************************************************************************
 * Function Section
 *****************************************************************************/

/* ***************************************************************************/
/**Allocates one Buffer Descriptor structure using information present in the
 * channel descriptor.
 *
 * @param *ccb_p channel control block used to get the channel descriptor
 *
 * @return
 *       - pointer on the new Buffer Descriptor
 *       - NULL if allocation failed
 *
 */
bufferDescriptor *
iapi_AllocBD(channelControlBlock *ccb_p)
{
	bufferDescriptor *ptrBD = NULL;
	int num_desc = ccb_p->channelDescriptor->bufferDescNumber;

	if (num_desc == 0)
		return NULL;

	ptrBD = MALLOC(num_desc * sizeof(bufferDescriptor));
	if (ptrBD == NULL) {
		DBG(0, "%s: Failed to allocate %u byte for %u BDs\n",
			__FUNCTION__,
			num_desc * sizeof(bufferDescriptor),
			num_desc);
		return NULL;
	}
	ptrBD->mode.command = 0;
	ptrBD->mode.status = 0;
	ptrBD->mode.count = 0;
	ptrBD->bufferAddr = DMA_ADDR_INVALID;

	return ptrBD;
}

/* ***************************************************************************/
/**Allocate one channel context data structure.
 *
 * @param **ctxd_p  pointer to context data to be allocated
 * @param channel channel number of context data structure
 *
 * @return
 *     - IAPI_SUCCESS
 *     - -iapi_errno if allocation failed
 */
int
iapi_AllocContext(contextData **ctxd_p, unsigned char channel)
{
	if (*ctxd_p != NULL) {
		iapi_errno = IAPI_ERR_CC_ALREADY_DEFINED |
			IAPI_ERR_CH_AVAILABLE | channel;
		return -iapi_errno;
	}

	*ctxd_p = MALLOC(sizeof(contextData));
	if (*ctxd_p == NULL) {
		iapi_errno = IAPI_ERR_CC_ALLOC_FAILED |
			IAPI_ERR_CH_AVAILABLE | channel;
		return -iapi_errno;
	}
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**Allocates channel description and fill in with default values.
 *
 * <b>Algorithm:</b>\n
 *  - Check channel properties.
 *  - Then modifies the properties of the channel description with default
 *
 * @param **cd_p  pointer to channel descriptor to be allocated
 * @param channel channel number of channel descriptor
 *
 * @return
 *     - IAPI_SUCCESS
 *     - -iapi_errno if allocation failed
 *
 */
int
iapi_AllocChannelDesc(channelDescriptor **cd_p, unsigned char channel)
{
	unsigned int pri;

	if (*cd_p != NULL) {
		iapi_errno = IAPI_ERR_CD_ALREADY_DEFINED |
			IAPI_ERR_CH_AVAILABLE | channel;
		return -iapi_errno;
	}

	*cd_p = MALLOC(sizeof(channelDescriptor));
	if (*cd_p == NULL) {
		iapi_errno = IAPI_ERR_CD_ALLOC_FAILED |
			IAPI_ERR_CH_AVAILABLE | channel;
		return -iapi_errno;
	}

	iapi_memcpy(*cd_p, &iapi_ChannelDefaults, sizeof(channelDescriptor));
	(*cd_p)->channelNumber = channel;
#ifdef MCU
	pri = __raw_readl(SDMA_CHNPRI(channel));
	if (pri != 0) {
		(*cd_p)->priority = pri;
	} else {
		__raw_writel((*cd_p)->priority, SDMA_CHNPRI(channel));
	}
#endif
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**Changes channel description information after performing sanity checks.
 *
 * <b>Algorithm:</b>\n
 *    - Check channel properties.
 *    - Then modifies the properties of the channel description.
 *
 * @param *cd_p channel descriptor of the channel to change
 * @param whatToChange control code indicating the desired change
 * @param newval new value
 *
 * @return
 *     - IAPI_SUCCESS
 *     - IAPI_FAILURE if change failed
 *
 */
int
iapi_ChangeChannelDesc(channelDescriptor *cd_p, unsigned char whatToChange,
		unsigned long newval)
{
	bufferDescriptor *tmpBDptr;
	unsigned int index;

	DBG(0, "%s: channel %d what=%08x newval=%08lx\n", __FUNCTION__,
		cd_p->channelNumber, whatToChange, newval);
	/* verify parameter validity */
	if (cd_p == NULL) {
		iapi_errno = IAPI_ERR_CD_UNINITIALIZED;
		return -iapi_errno;
	}

	/* verify channel descriptor initialization */
	if (cd_p->ccb_ptr == NULL) {
		iapi_errno = IAPI_ERR_CCB_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE |
			cd_p->channelNumber;
		return -iapi_errno;
	}

	/* verify channel is not in use */
	if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
		tmpBDptr = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
		for (index = cd_p->bufferDescNumber; index > 0; index--) {
			if (tmpBDptr->mode.status & BD_DONE) {
				iapi_errno = IAPI_ERR_CH_IN_USE | IAPI_ERR_CH_AVAILABLE |
					cd_p->channelNumber;
				return -iapi_errno;
			}
			tmpBDptr++;
		}
	}

	/* Select the change accorded to the selector given in parameter */
	switch (whatToChange) {
	case IAPI_CHANNELNUMBER:
		/*
		 * Channel Number
		 */
		/* Channel number can not be changed (description remains attached) */
		iapi_errno = IAPI_ERR_CD_CHANGE_CH_NUMBER |
			IAPI_ERR_CH_AVAILABLE |
			cd_p->channelNumber;
		return -iapi_errno;

	case IAPI_BUFFERDESCNUMBER:
		/*
		 * Buffer Descriptor Number
		 */
		if (newval >= MAX_BD_NUM) {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER |
				IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}
		if (newval == cd_p->bufferDescNumber) {
			break;
		}
		/* Free memory used for previous data */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			tmpBDptr = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			for (index = 0; index < cd_p->bufferDescNumber; index++) {
				if (tmpBDptr->bufferAddr != DMA_ADDR_INVALID) {
					if (cd_p->trust == FALSE) {
						FREE(iapi_Phys2Virt(tmpBDptr->bufferAddr));
					}
				}
				tmpBDptr++;
			}
			FREE(iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr));
			cd_p->ccb_ptr->baseBDptr = DMA_ADDR_INVALID;
			cd_p->ccb_ptr->currentBDptr = DMA_ADDR_INVALID;
		}
		/* Allocate and initialize structures */
		cd_p->bufferDescNumber = newval;
		cd_p->ccb_ptr->status.openedInit = FALSE;
		if (IAPI_SUCCESS != iapi_InitializeMemory(cd_p->ccb_ptr)) {
			iapi_errno = IAPI_ERR_BD_ALLOCATION | cd_p->channelNumber;
			return -iapi_errno;
		}
		cd_p->ccb_ptr->status.openedInit = TRUE;
		break;

	case IAPI_BUFFERSIZE:
		/*
		 * Buffer size
		 */
		if (newval < MAX_BD_SIZE) {
			if (newval != cd_p->bufferSize) {
				/* Free memory used for previous old data */
				if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
					tmpBDptr = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
					for (index = 0; index < cd_p->bufferDescNumber; index++) {
						if (cd_p->trust == FALSE) {
							FREE(iapi_Phys2Virt(tmpBDptr->bufferAddr));
						}
						tmpBDptr++;
					}
					FREE(iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr));
				}
				cd_p->ccb_ptr->baseBDptr = DMA_ADDR_INVALID;
				cd_p->ccb_ptr->currentBDptr = DMA_ADDR_INVALID;
				/* Allocate and initialize structures */
				cd_p->bufferSize = newval;
				cd_p->ccb_ptr->status.openedInit = FALSE;
				if (IAPI_SUCCESS != iapi_InitializeMemory(cd_p->ccb_ptr)) {
					iapi_errno = IAPI_ERR_BD_ALLOCATION | cd_p->channelNumber;
					return -iapi_errno;
				}
				cd_p->ccb_ptr->status.openedInit = TRUE;
			}
			break;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}

	case IAPI_BLOCKING:
		/*
		 * Blocking / non blocking feature
		 */
		if (newval < MAX_BLOCKING) {
			cd_p->blocking = newval;
			break;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}

	case IAPI_CALLBACKSYNCH:
		/*
		 * Synchronization method
		 */
		if (newval < MAX_SYNCH) {
			cd_p->callbackSynch = newval;
			iapi_ChangeCallbackISR( cd_p, cd_p->callbackISR_ptr);
			break;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}

	case IAPI_OWNERSHIP:
		/*
		 *  Ownership of the channel
		 */
#ifdef DSP
		iapi_errno = IAPI_ERR_NOT_ALLOWED | cd_p->channelNumber;
		return -iapi_errno;
#endif /* DSP */
#ifdef MCU
		if (newval < MAX_OWNERSHIP) {
			cd_p->ownership = newval;
			iapi_ChannelConfig(cd_p->channelNumber,
					(newval >> CH_OWNSHP_OFFSET_EVT) & 1,
					(newval >> CH_OWNSHP_OFFSET_MCU) & 1,
					(newval >> CH_OWNSHP_OFFSET_DSP) & 1);
			break;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}
#endif /* MCU */
	case IAPI_PRIORITY:
		/*
		 * Priority
		 */
#ifdef DSP
		iapi_errno = IAPI_ERR_NOT_ALLOWED | cd_p->channelNumber;
		return -iapi_errno;
#endif /* DSP */

#ifdef MCU
		if (newval < MAX_CH_PRIORITY) {
#if 1
			__raw_writel(newval, SDMA_CHNPRI(cd_p->channelNumber));
#else
			volatile unsigned long *ChannelPriorities = &SDMA_CHNPRI_0;
			ChannelPriorities[cd_p->channelNumber] = newval;
#endif
			break;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}
#endif /* MCU */
	case IAPI_TRUST:
		/*
		 * "Trust" property
		 */
		if (newval < MAX_TRUST) {
			if (cd_p->trust != newval) {
				cd_p->trust = newval;
				if (newval == FALSE) {
					if (IAPI_SUCCESS !=iapi_InitializeMemory(cd_p->ccb_ptr)) {
						iapi_errno = IAPI_ERR_BD_ALLOCATION | cd_p->channelNumber;
						return -iapi_errno;
					}
				}
			}
			break;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER | IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}

	case IAPI_CALLBACKISR_PTR:
		/*
		 * Callback function pointer
		 */
		if ((void *)newval != NULL) {
			cd_p->callbackISR_ptr = (void *)newval;
			iapi_ChangeCallbackISR( cd_p, cd_p->callbackISR_ptr);
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER |
				IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}
		break;

	case IAPI_CCB_PTR:
		/*
		 * Channel Control Block pointer
		 */
		cd_p->ccb_ptr = (channelControlBlock *)newval;
		cd_p->ccb_ptr->channelDescriptor = cd_p;
		break;

	case IAPI_BDWRAP:
		/*
		 * WRAP/UNWRAP
		 */
		/* pointer to first BD */
		tmpBDptr = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
		/* pointer to last BD */
		tmpBDptr += cd_p->bufferDescNumber - 1;
		DBG(0, "%s: newval=%08lx\n", __FUNCTION__, newval);
		if (newval == TRUE) {
			/* wrap last BD */
			DBG(0, "%s: Setting BD_WRAP for channel %d BD[%d/%d]\n",
				__FUNCTION__, cd_p->channelNumber,
				cd_p->bufferDescNumber - 1,
				cd_p->bufferDescNumber);
			tmpBDptr->mode.status |= BD_WRAP;
		} else if (newval == FALSE) {
			/* unwrap last BD */
			DBG(0, "%s: Clearing BD_WRAP for channel %d BD[%d/%d]\n",
				__FUNCTION__, cd_p->channelNumber,
				cd_p->bufferDescNumber - 1,
				cd_p->bufferDescNumber);
			tmpBDptr->mode.status &= ~BD_WRAP;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER |
				IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}
		break;

	case IAPI_WML:
		/*
		 * Watermark level
		 */
#ifdef DSP
		iapi_errno = IAPI_ERR_NOT_ALLOWED | cd_p->channelNumber;
		return -iapi_errno;
#endif /* DSP */
#ifdef MCU
		if (newval >= MAX_WML) {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER |
				IAPI_ERR_CH_AVAILABLE |
				cd_p->channelNumber;
			return -iapi_errno;
		}
		if (cd_p->watermarkLevel != newval) {
			cd_p->watermarkLevel = newval;
		}
		break;
#endif /* MCU */
	default:
		/*
		 * Detect errors
		 */
		iapi_errno = IAPI_ERR_CD_CHANGE_UNKNOWN |
			IAPI_ERR_CH_AVAILABLE |
			cd_p->channelNumber;
		return -iapi_errno;
	}
	return IAPI_SUCCESS;
}


/* ***************************************************************************/
/**Initialize a table of function pointers that contain the interrupt Service
 * Routine callback pointers for the SDMA channels with a default value
 *
 * <b>Algorithm:</b>\n
 *    - Loop on each element of the global IAPI variable callbackIsrTable
 *
 * @param *func_p default callback functon for all SDMA channels
 *
 * @return none
 */
void
iapi_InitializeCallbackISR(void(* func_p)(channelDescriptor *cd_p, void *arg))
{
	unsigned long chCnt;

	for (chCnt = 0; chCnt < CH_NUM; chCnt++) {
		callbackIsrTable[chCnt] = func_p;
	}
}

/* ***************************************************************************/
/**For the specified  channel control block, attach the array of buffer
 * descriptors, the channel description structure and initialize channel's
 * status using information in the channel descriptor.
 *
 * @param *ccb_p pointer to channel control block
 *
 * @return none
 *
 */
int
iapi_InitializeMemory(channelControlBlock *ccb_p)
{
	bufferDescriptor *bd_p;
	unsigned int index;
	channelDescriptor *cd_p = ccb_p->channelDescriptor;

	/* Attach the array of Buffer descriptors */
	bd_p = iapi_AllocBD(ccb_p);
	if (bd_p == NULL)
		return -IAPI_ERR_BD_ALLOCATION;

	ccb_p->baseBDptr = iapi_Virt2Phys(bd_p);
	ccb_p->currentBDptr = ccb_p->baseBDptr;
	DBG(0, "%s: BDptr[%p] of ccb %p set to %08x\n", __FUNCTION__,
		&ccb_p->baseBDptr, ccb_p, ccb_p->baseBDptr);
	for (index = 0; index < cd_p->bufferDescNumber - 1; index++) {
		if (cd_p->trust == TRUE) {
			iapi_SetBufferDescriptor(bd_p,
						cd_p->dataSize,
						BD_CONT | BD_EXTD, cd_p->bufferSize,
						NULL, DMA_ADDR_INVALID);
		} else {
			if (cd_p->bufferSize != 0) {
				void *buf = MALLOC(cd_p->bufferSize);

				if (buf == NULL)
					goto cleanup;

				iapi_SetBufferDescriptor(bd_p,
							cd_p->dataSize,
							BD_CONT | BD_EXTD,
							cd_p->bufferSize,
							buf, DMA_ADDR_INVALID);
			}
		}
		bd_p++;
	}

	if (cd_p->trust == TRUE) {
		iapi_SetBufferDescriptor(bd_p,
					cd_p->dataSize,
					BD_EXTD | BD_WRAP | BD_INTR,
					cd_p->bufferSize,
					NULL, DMA_ADDR_INVALID);
	} else {
		if (cd_p->bufferSize != 0) {
			void *buf = MALLOC(cd_p->bufferSize);

			if (buf == NULL)
				goto cleanup;

			iapi_SetBufferDescriptor(bd_p,
						cd_p->dataSize,
						BD_EXTD | BD_WRAP | BD_INTR,
						cd_p->bufferSize,
						buf,
						DMA_ADDR_INVALID);
		}
	}
	return IAPI_SUCCESS;
cleanup:
	WARN_ON(1);
	while (--index >= 0) {
		
	}
}
