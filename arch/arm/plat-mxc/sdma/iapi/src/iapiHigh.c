/******************************************************************************
 *
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * File: iapiHigh.c
 *
 * $Id iapiHigh.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the HIGH level functions of the I.API.
 *
 *
 * /
 *
 * $Log iapiHigh.c $
 *
 *****************************************************************************/

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include <stdarg.h>
#include <string.h>
#include <linux/err.h>
#include <linux/io.h>

#include <epm.h>
#include <iapi.h>

#ifdef DEBUG
#include <mach/sdma.h>
#if CH_NUM != MAX_DMA_CHANNELS
#error BAD number of DMA channels!
#endif
#endif

/* ****************************************************************************
 * External Reference Section (for compatibility with already developed code)
 *****************************************************************************/
static void iapi_read_ipcv2_callback(struct iapi_channelDescriptor *cd_p, void *data);

/* ****************************************************************************
 * Global Variable Section
 *****************************************************************************/
#define		MAX_CHANNEL	    32

static dataNodeDescriptor *dnd_read_control_struct[MAX_CHANNEL];

/* MASK to nullify all the bits of Status in Data Node descriptor apart from L, E and D */
#define	    GET_LED_MASK	    0xE0

/* Table defines mapping of Data Node Descriptor to Buffer Descriptor status */
static unsigned char dnd_2_bd_status[] = {
	[0x00] = 0x85, /* L = 0, E = 0,  D = 0 */
	[0x20] = 0x84, /* L = 0, E = 0,  D = 1 */
	[0x40] = 0xAB, /* L = 0, E = 1,  D = 0 */
	[0x60] = 0xAA, /* L = 0, E = 1,  D = 1 */
	[0x80] = 0xC5, /* L = 1, E = 0,  D = 0 */
	[0xA0] = 0xC4, /* L = 1, E = 0,  D = 1 */
	[0xC0] = 0xEB, /* L = 1, E = 1,  D = 0 */
	[0xE0] = 0xEA, /* L = 1, E = 1,  D = 1 */
};
/* ****************************************************************************
 * Function Section
 *****************************************************************************/

/* ***************************************************************************/
/* Opens an SDMA channel to be used by the library.
 *
 * <b>Algorithm:</b>\n
 *
 * - Check if initialization is necessary.
 *   - Check that user initialized OS dependant functions.
 *   - Test validity of input parameters
 *   - Check whole channel control block data structure
 *   - Finish initializations (tables with default values)
 *   - Initialize channel 0 is dedicated to communications with SDMA
 * - Check channel control block definition
 *   - if the channel descriptor is not initialized, initialize it with
 * the default value
 * - If buffer descriptor already allocated, exit with iapi_errno filled
 * complete the lowest bits with the number of 'D' bits set
 * - Buffer Descriptors allocation
 * - Channel's configuration properties (mcu side only)
 * - read/write direction => enable/disable channel setting
 *
 * @param  *cd_p If channelNumber is 0, it is pointer to channel descriptor
 *		for the channnel 0 to be opened and
 *		has default values.
 *		For other channels, this function should be called after
 *		channel 0 has been opened, and it's channel descriptor
 *		has been allocated.
 * @param  channelNumber channel to be opened
 *
 * @return
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : close failed, return negated value of iapi_errno
 */
int
iapi_Open(channelDescriptor *cd_p, unsigned char channelNumber)
{
	channelControlBlock *ccb_p;
	channelControlBlock *local_ccb_p;
	channelDescriptor   *local_cd_p;
	bufferDescriptor    *bd_p;
	int index;

	DBG(0, "%s: cd_p=%p channel=%d\n", __FUNCTION__, cd_p, channelNumber);

	/*
	 * 1. Check if initialization is necessary
	 */
	if (cd_p == NULL) {
		iapi_errno = IAPI_ERR_CD_UNINITIALIZED |
			IAPI_ERR_CH_AVAILABLE | channelNumber;
		return -iapi_errno;
	}

	/* Verify these functions every time */
	if ((iapi_GetChannel == NULL) || (iapi_ReleaseChannel == NULL)) {
		iapi_errno = IAPI_ERR_NO_OS_FN | channelNumber;
		return -iapi_errno;
	}

	/* Try to aquire channel */
	if (iapi_GetChannel(channelNumber) != 0) {
		iapi_errno = IAPI_ERR_CH_IN_USE | channelNumber;
		return -iapi_errno;
	}

	DBG(0, "%s@%d: cd_p=%p ccb_ptr=%p\n", __FUNCTION__, __LINE__, cd_p,
		cd_p->ccb_ptr);
	if (channelNumber == 0 && cd_p->ccb_ptr == NULL) {
		int i;

		/* Verify that the user initialized all OS dependant functions required
		 * by the library.
		 */
		if ((iapi_Malloc == NULL) ||
			(iapi_Free == NULL) ||
			(iapi_Virt2Phys == NULL) ||
			(iapi_Phys2Virt == NULL) ||
			(iapi_GotoSleep == NULL) ||
			(iapi_WakeUp == NULL) ||
			(iapi_InitSleep == NULL) ||
			(iapi_memset == NULL) ||
			(iapi_memcpy == NULL)) {
			iapi_errno = IAPI_ERR_NO_OS_FN | channelNumber;
			iapi_ReleaseChannel(channelNumber);
			return -iapi_errno;
		}
		DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
		/* Whole channel control block data structure */
		ccb_p = MALLOC(CH_NUM * sizeof(channelControlBlock));
		if (ccb_p == NULL) {
			iapi_errno = IAPI_ERR_CCB_ALLOC_FAILED |
				IAPI_ERR_CH_AVAILABLE | channelNumber;
			iapi_ReleaseChannel(channelNumber);
			return -iapi_errno;
		}
		DBG(0, "%s: ccb allocated at %p..%p\n", __FUNCTION__, ccb_p,
			(unsigned char *)(ccb_p + CH_NUM) - 1);

		/* Zero-out the CCB structures array just allocated */
		iapi_memset(ccb_p, 0x00, CH_NUM * sizeof(channelControlBlock));
		for (i = 0; i < CH_NUM; i++) {
			ccb_p[i].baseBDptr = DMA_ADDR_INVALID;
			ccb_p[i].currentBDptr = DMA_ADDR_INVALID;
		}
		/* Save the address of the CCB structures array */
		iapi_CCBHead = ccb_p;

		cd_p->ccb_ptr = ccb_p;
		ccb_p->channelDescriptor = cd_p;
		DBG(0, "%s: channelDescriptor %p=%p\n", __FUNCTION__,
			&ccb_p->channelDescriptor, cd_p);
#ifdef MCU
		/* finish initializations */
		iapi_InitChannelTables();
#endif /* MCU */
		/* Channel 0 is dedicated to communications with SDMA */
		cd_p->ownership = ((DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT) |
				(OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU) |
				(DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP));
		cd_p->bufferDescNumber = 1;
	}

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	/*
	 * 2. Check channel control block
	 */
	ccb_p = cd_p->ccb_ptr;
	if (ccb_p == NULL) {
		iapi_errno = IAPI_ERR_NO_CCB_DEFINED |
			IAPI_ERR_CH_AVAILABLE |
			channelNumber;
		iapi_ReleaseChannel(channelNumber);
		return -iapi_errno;
	}

	DBG(0, "%s@%d: ccb_p=%p\n", __FUNCTION__, __LINE__, ccb_p);
	/* Control block & Descriptor associated with the channel being worked on */
	local_ccb_p = &ccb_p[channelNumber];
	local_cd_p  =  ccb_p[channelNumber].channelDescriptor;

	DBG(0, "%s@%d: local_ccb_p[%d]=%p local_cd_p[%p]=%p\n", __FUNCTION__, __LINE__,
		channelNumber, local_ccb_p,
		&ccb_p[channelNumber].channelDescriptor, local_cd_p);
	/* If the channel is not initialized, initialize it with the default value */
	if (local_cd_p == NULL) {
		int result = iapi_AllocChannelDesc(&local_cd_p, channelNumber);
		if (result!= IAPI_SUCCESS) {
			iapi_ReleaseChannel(channelNumber);
			return result; //is already negated from iapi_AllocChannelDesc
		}

		local_cd_p->ccb_ptr = (struct iapi_channelControlBlock *)local_ccb_p;
		local_ccb_p->channelDescriptor = local_cd_p;
	}

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	/*
	 * 3. If buffer descriptor already allocated, exit with iapi_errno filled
	 */
	if (local_ccb_p->baseBDptr != DMA_ADDR_INVALID) {
		int result = IAPI_ERR_BD_ALLOCATED;

		bd_p = iapi_Phys2Virt(local_ccb_p->baseBDptr);
		if (bd_p == NULL) {
			iapi_errno = IAPI_ERR_BD_ALLOCATION;
			return -iapi_errno;
		}
		DBG(0, "%s: bd_p=%p phys=%08x\n", __FUNCTION__, bd_p, local_ccb_p->baseBDptr);
		for (index = 1; index < local_cd_p->bufferDescNumber; index++) {
			if ((bd_p->mode.status & BD_DONE) == BD_DONE) {
				/* complete the lowest bits with the number of 'D' bits set */
				result++;
			}
			bd_p++;
		}
		iapi_errno = result;
		iapi_ReleaseChannel(channelNumber);
		return -iapi_errno;
	}

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	/*
	 * 4. Buffer Descriptors allocation
	 */
	iapi_InitializeMemory(local_ccb_p);

#ifdef MCU
	/*
	 * 5. Channel's configuration properties (mcu side only)
	 */
	iapi_ChannelConfig(channelNumber,
			(local_cd_p->ownership >> CH_OWNSHP_OFFSET_EVT) & 1,
			(local_cd_p->ownership >> CH_OWNSHP_OFFSET_MCU) & 1,
			(local_cd_p->ownership >> CH_OWNSHP_OFFSET_DSP) & 1);
#endif /* MCU */

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	/* Setting interrupt handling */
	iapi_ChangeCallbackISR(local_cd_p, local_cd_p->callbackISR_ptr);

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	/* Call initialization fn for polling synch on this channel */
	INIT_SLEEP(channelNumber);

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	/* No user arg pointer yet */
	userArgTable[cd_p->channelNumber] = NULL;

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	/*
	 * 6. read/write direction => enable/disable channel
	 */
#ifdef MCU
	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
#if 1
	__raw_writel(1, SDMA_CHNPRI(channelNumber));
#else
	channelPriorityMatx = &SDMA_CHNPRI_0;
	channelPriorityMatx[channelNumber] = 1;
#endif
#endif /* MCU */

	DBG(0, "%s@%d: \n", __FUNCTION__, __LINE__);
	local_ccb_p->status.openedInit = TRUE;
	iapi_ReleaseChannel(channelNumber);
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/** Attempts to read nbyte from the data buffer descriptor associated with the
 * channel channelNumber, into the user's data buffer pointed to by buf.
 *
 * <b>Algorithm:</b>\n
 * - Check data structures are properly initialized:
 *   - Channel descriptor validity
 *   - Control block & Descriptor associated with the channel being worked on
 *   - Check initialization has been done for trusted channels
 *   - If transfer data size is used, check validity of combination transfer
 *     size/requested bytes
 * - Set the 'D' done bits on all buffer descriptors
 * - Starting of the channel
 * - Synchronization mechanism handling:
 *   - for callback: just exit function
 *   - for polling: call the synchronization function then read data from
 *     buffer until either nbyte parameter is reached or all buffer descriptors
 *     have been processed.
 *
 * <b>Notes:</b>\n
 *   1) Virtual DMA SDMA channels are unidirectional, an iapi_Read authorized
 *       on a channel means that we are expecting to receive from the SDMA. The
 *       meaning of an interrupt received from the SDMA is therefore that the
 *       data has been copied from the SDMA to the host's data buffers and is
 *       already passed on upper layers of the application.\n
 *
 * @param *cd_p chanenl descriptor for the channel to read from
 * @param *buf buffer to receive the data
 * @param nbyte number of bytes to read from channel
 *
 * @return
 *       - number of bytes read
 *       - -iapi_errno : in case of failure return negated value of iapi_errno
 */
int
iapi_Read(channelDescriptor *cd_p, void *buf, unsigned short nbyte)
{
	int index;
	int readBytes;
	int toRead;
	unsigned int copyFinished;
	unsigned int bufsize;
	bufferDescriptor *bd_p;
	channelControlBlock *ccb_p;
	unsigned char *local_buf;
	unsigned char chNum;
	unsigned char div;

	iapi_errno = IAPI_ERR_NO_ERROR;

	/*
	 * 1. Check data structures are properly initialized
	 */
	/* Channel descriptor validity */
	if (cd_p == NULL) {
		iapi_errno = IAPI_ERR_CD_UNINITIALIZED;
		return -iapi_errno;
	}

	/* Channel control block validity */
	if (cd_p->ccb_ptr == NULL) {
		iapi_errno = IAPI_ERR_CCB_UNINITIALIZED;
		return -iapi_errno;
	}

	/* Control block & Descriptor associated with the channel being worked on */
	chNum = cd_p->channelNumber;
	ccb_p = cd_p->ccb_ptr;

	/* Try to aquire channel */
	if (iapi_GetChannel(chNum) != 0) {
		iapi_errno = IAPI_ERR_CH_IN_USE | chNum;
		return -iapi_errno;
	}

	/* Check if channel is already opened/initialized */
	if (ccb_p->status.openedInit == FALSE) {
		iapi_errno = IAPI_ERR_CHANNEL_UNINITIALIZED |
			IAPI_ERR_CH_AVAILABLE | chNum;
		iapi_ReleaseChannel(chNum);
		return -iapi_errno;
	}

	/* Buffer descriptor validity */
	bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
	if (bd_p == NULL) {
		iapi_errno = IAPI_ERR_BD_UNINITIALIZED |
			IAPI_ERR_CH_AVAILABLE | chNum;
		iapi_ReleaseChannel(chNum);
		return -iapi_errno;
	}


	/* Check initialization has been done for trusted channels */
	if (cd_p->trust == TRUE) {
		bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
		for (index = 0; index < cd_p->bufferDescNumber; index++) {
			if ((bd_p->bufferAddr == DMA_ADDR_INVALID) ||
				(bd_p->mode.count == 0)) {
				iapi_errno = IAPI_ERR_BD_UNINITIALIZED |
					IAPI_ERR_CH_AVAILABLE | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			bd_p++;
		}
	}

	bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
	/* If transfer data size is used, check that the required read length is
	 * divisible by transfer data size expressed in bytes
	 */
	if (cd_p->useDataSize) {
		/* Check for divisibility only if data size different then 8bit */
		if (cd_p->dataSize != TRANSFER_8BIT) {
			switch(cd_p->dataSize) {
			case TRANSFER_32BIT:
				div = 4;
				break;
			case TRANSFER_16BIT:
				div = 2;
				break;
			case TRANSFER_24BIT:
				div = 3;
				break;
				/* we should not get to default */
			default:
				iapi_errno = IAPI_ERR_INVALID_PARAMETER | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			/* check the total number of bytes requested */
			if ((nbyte % div) != 0) {
				iapi_errno = IAPI_ERR_INVALID_PARAMETER | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			/* now check the length of every BD */
			for (index = 0; index < cd_p->bufferDescNumber; index++) {
				if ((bd_p->mode.count % div) != 0) {
					iapi_errno = IAPI_ERR_INVALID_PARAMETER | chNum;
					iapi_ReleaseChannel(chNum);
					return -iapi_errno;
				}
				bd_p++;
			}
		}
	}

	/*
	 * 2. Set the 'D' done bits on all buffer descriptors
	 */
	bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
	for (index = 0; index < cd_p->bufferDescNumber; index++) {
		bd_p->mode.status |= BD_DONE;
		bd_p++;
	}

	/*
	 * 3. Starting of the channel
	 */
	iapi_lowStartChannel(chNum);
	ccb_p->status.execute = TRUE;
	readBytes = 0;

	/*
	 * 4. Synchronization mechanism handling
	 */
	if (cd_p->callbackSynch == DEFAULT_POLL) {
		iapi_SynchChannel(chNum);

		bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
		toRead = nbyte;
		copyFinished = FALSE;
		local_buf = buf;

		/*
		 * Check the 'RROR' bit on all buffer descriptors, set error number
		 *    and return IAPI_FAILURE if set.
		 */
		for (index = 0; index < cd_p->bufferDescNumber; index++) {
			if (bd_p->mode.status & BD_RROR) {
				iapi_errno = IAPI_ERR_RROR_BIT_READ | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			bd_p++;
		}


		/*
		 * 5. Read loop
		 */

		bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
		while (!copyFinished) {
			if (!(bd_p->mode.status & BD_DONE)) {
				if (cd_p->trust == FALSE) {
					bufsize = cd_p->bufferSize;
				} else {
					bufsize = bd_p->mode.count;
				}
				/* if L bit is set, read only "count" bytes and exit the loop */
				if (bd_p->mode.status & BD_LAST) {
					bufsize = bd_p->mode.count;
					copyFinished = TRUE;
				}
				if (toRead > bufsize) {
					if (cd_p->trust == FALSE) {
						iapi_memcpy(local_buf, iapi_Phys2Virt(bd_p->bufferAddr), bufsize);
						local_buf += bufsize;
					}
					readBytes += bufsize;
					toRead -= bufsize;
					/* advance bd_p only if bit L is not set. The loop will exit anyway. */
					if (!(bd_p->mode.status & BD_LAST)) {
						if (bd_p->mode.status & BD_WRAP) {
							bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
						} else if ((iapi_Phys2Virt(ccb_p->baseBDptr +
											(cd_p->bufferDescNumber - 1) *
											sizeof(bufferDescriptor))) != bd_p) {
							bd_p++;
						} else {
							/* finished here : end of buffer descriptors */
							copyFinished = TRUE;
						}
					}
				} else {
					if (cd_p->trust == FALSE) {
						iapi_memcpy(local_buf, iapi_Phys2Virt(bd_p->bufferAddr), toRead);
						local_buf += toRead;
					}
					readBytes += toRead;
					toRead = 0;
					/* finished successfully : readBytes = nbytes */
					copyFinished = TRUE;
				}
			} else {
				/* finished here : buffer not already done */
				copyFinished = TRUE;
			}
		}
		iapi_ReleaseChannel(chNum);
	}

	/*
	 *If synchronization type is callback, the user of I.API must
	 *release the channel
	 */
	return readBytes;
}

/****************************************************************************
 * Attempts to write nbyte from the buffer pointed to by buf to the channel
 * data buffers associated with the opened channel number channelNumber
 *
 * <b>Algorithm:</b>\n
 *
 * - Check data structures are properly initialized:
 *   - Channel descriptor validity
 *   - Channel control block validity
 *   - Buffer descriptor validity
 *   - If transfer data size is used, check validity of combination transfer
 *     size/requested bytes
 * - Write loop\n
 *  Write occurs in the buffer acceded form buffer descriptor and continues
 *  to the "next" buffer which can be:\n
 *    -# the last BD of the ring so re-start from beginning\n
 *    -# the last BD  of the BD array but no ring so finish\n
 *    -# (general case) the next BD in the BD array\n
 *  And copy continues until data fit in the current buffer or the nbyte
 *  parameter is reached.
 * - Starting of the channel
 *
 * <b>Notes:</b>\n
 *   1) Virtual DMA SDMA channels are unidirectionnal, an iapi_Write authorized
 *      on a channel means that we are expecting to send to the SDMA. The
 *      meaning of an interrupt received from the SDMA is therfore that the
 *      data has been delivered to the SDMA.
 *
 * @param *cd_p chanenl descriptor for the channel to write to
 * @param *buf buffer with data to be written
 * @param nbyte number of bytes to write to channel
 *
 * @return
 *       - number of bytes written
 *       - -iapi_errno if failure
 */
int
iapi_Write(channelDescriptor *cd_p, void *buf, unsigned short nbyte)
{
	int writtenBytes = 0;
	unsigned int toWrite;
	unsigned int copyFinished;
	unsigned int buffsize;
	unsigned int index = 0;
	bufferDescriptor *bd_p;
	channelControlBlock *ccb_p;
	unsigned char *local_buf;
	unsigned char chNum;
	unsigned char div;

	iapi_errno = IAPI_ERR_NO_ERROR;

	/*
	 * 1. Check data structures are properly initialized
	 */
	/* Channel descriptor validity */
	if (cd_p == NULL) {
		iapi_errno = IAPI_ERR_CD_UNINITIALIZED;
		return -iapi_errno;
	}

	/* Channel control block validity */
	if (cd_p->ccb_ptr == NULL) {
		iapi_errno = IAPI_ERR_CCB_UNINITIALIZED;
		return -iapi_errno;
	}

	/* Control block & Descriptpor associated with the channel being worked on */
	chNum = cd_p->channelNumber;
	ccb_p = cd_p->ccb_ptr;

	/* Try to aquire channel */
	if (iapi_GetChannel(chNum) != 0) {
		iapi_errno = IAPI_ERR_CH_IN_USE | chNum;
		return -iapi_errno;
	}

	/* Buffer descriptor validity */
	bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
	if (bd_p == NULL) {
		iapi_errno = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
		iapi_ReleaseChannel(chNum);
		return -iapi_errno;
	}

	/* Check initialization has been done for trusted channels */
	if (cd_p->trust == TRUE) {
		bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
		for (index = 0; index < cd_p->bufferDescNumber; index++) {
			if ((bd_p->bufferAddr == DMA_ADDR_INVALID) || (bd_p->mode.count == 0)) {
				iapi_errno = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			bd_p++;
		}
	}


	bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
	/* If transfer data size is used, check that the required write length is
	 * divisible by transfer data size expressed in bytes
	 */
	if (cd_p->useDataSize) {
		/* Check for divisibility only if data size different then 8bit */
		if (cd_p->dataSize != TRANSFER_8BIT) {
			switch(cd_p->dataSize) {
			case TRANSFER_32BIT:
				div = 4;
				break;
			case TRANSFER_16BIT:
				div = 2;
				break;
			case TRANSFER_24BIT:
				div = 3;
				break;
			default:
				iapi_errno = IAPI_ERR_INVALID_PARAMETER | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			/* check the total number of bytes requested */
			if ((nbyte % div) != 0) {
				iapi_errno = IAPI_ERR_INVALID_PARAMETER | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			/* now check the length of every BD */
			for (index = 0; index < cd_p->bufferDescNumber; index++) {
				if ((bd_p->mode.count % div) != 0) {
					iapi_errno = IAPI_ERR_INVALID_PARAMETER | chNum;
					iapi_ReleaseChannel(chNum);
					return -iapi_errno;
				}
				bd_p++;
			}
		}
	}

	/*
	 * 2. Write loop
	 */
	local_buf = buf;
	toWrite = nbyte;
	copyFinished = FALSE;
	bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);

	while (!copyFinished) {
		/* variable buffsize contains the number of bytes that the SDMA
		 * will transfer at each pass of the while loop */
		/* in NON trusted mode, buffsize is copied from Channel
		 * descriptor bufferSize (same size for all transfers) */

		if (cd_p->trust == FALSE) {
			buffsize = cd_p->bufferSize;
		} else {
			/* in TRUSTED mode, it's up to the user to specify the size of each buffer thru an IoCtl call */
			/* This IoCtl has directly modified the bd_p->mode.count     */
			/* therefore, buffersize is copied from the bd_p->mode.count */
			buffsize = bd_p->mode.count;
		}
		DBG(-1, "%s: nbyte=%u towrite=%u buffsize=%u\n", __FUNCTION__,
			nbyte, toWrite, buffsize);

		/* in any mode (trusted or non trusted), the transfer size must be overridden by */
		/* "toWrite" when there is less remaining bytes to transfer than the current buffer size */
		if (toWrite < buffsize) {
			buffsize = toWrite;
		}

		if (!(bd_p->mode.status & BD_DONE)) {
			/* More data to write than a single buffer can contain */
			if (cd_p->trust == FALSE) {
				iapi_memcpy(iapi_Phys2Virt(bd_p->bufferAddr),
					local_buf, buffsize);
				local_buf += buffsize;
			}

			/* update the BD count that will be used by the SDMA to transfer the proper nb of bytes */
			bd_p->mode.count = buffsize;

			bd_p->mode.status |= BD_DONE;
			writtenBytes += buffsize;
			toWrite -= buffsize;
			/* Prepares access to the "next" buffer */
			if (toWrite == 0) {
				/* - case 1 - finished successfully : writtenBytes = nbytes */
				copyFinished = TRUE;
			} else if ((bd_p->mode.status & BD_WRAP)) {
				/* - case 2 - Last BD and WRAP bit set so re-start from beginning */
				bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
			} else if ((iapi_Phys2Virt(
						ccb_p->baseBDptr +
						(cd_p->bufferDescNumber - 1) *
						sizeof(bufferDescriptor))) ==
				bd_p) {
				/* - case 3 - Last BD of the BD but not ring */
				copyFinished = TRUE;
			} else {
				/* - case 4 - general : next BD in the BD array */
				bd_p++;
			}
		} else {
			/* finished here : buffer not already done */
			copyFinished = TRUE;
		}
	}

	ccb_p->currentBDptr = ccb_p->baseBDptr;

	/*
	 * 3. Starting of the channel
	 */
	iapi_lowStartChannel(chNum);
	ccb_p->status.execute = TRUE;

	if (cd_p->callbackSynch == DEFAULT_POLL) {
		iapi_SynchChannel(chNum);
		/*
		 * Check the 'RROR' bit on all buffer descriptors, set error number
		 *    and return IAPI_FAILURE if set.
		 */
		bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);
		for (index = 0; index < cd_p->bufferDescNumber; index++) {
			if (bd_p->mode.status & BD_RROR) {
				iapi_errno = IAPI_ERR_RROR_BIT_WRITE | chNum;
				iapi_ReleaseChannel(chNum);
				return -iapi_errno;
			}
			bd_p++;
		}
		iapi_ReleaseChannel(chNum);
	}

	/*
	 * If synchronization type is callback, the user of I.API must
	 * release the channel
	 */
	return writtenBytes;
}

/* ***************************************************************************/
/* This function is used to receive data from the SDMA.
 *
 * <b>Algorithm:</b>\n
 *
 * The data control structure would be copied to IPCv1 complied Buffer
 * Descriptor Array. This array shall be allocated from non cacheable memory.
 * It would then provide this buffer descriptor array as an input to SDMA using
 * channel control block and then configure the Host Enable (HE) or
 * DSP enable (DE) bit of SDMA for the channel used for this transfer depending
 * on the source.
 *
 * <b>Notes:</b>\n
 * Virtual DMA channels are unidirectional, an iapi_Write_ipcv2 authorized
 * on a channel means that source processor is expecting to send to the destination
 * processor. The meaning of an interrupt received from the SDMA notifies that the
 * data has been delivered to the destination processor.
 *
 * @param *cd_p chanenl descriptor for the channel to receive from
 * @param *data_control_struct_ipcv2

 *   Data Control structure:
 *   -------------------------
 *   | Data Node Descriptor 1|
 *   -------------------------
 *   | Data Node Descriptor 2|
 *   -------------------------
 *   |           :           |
 *   |           :           |
 *   -------------------------
 *   |Data Node Descriptor n |
 *   -------------------------
 *
 *   Data Node Descriptor (Buffer Descriptor):
 *------------------------------------------------------------------------------
 *| 31	30	29	28	27	26	25	24	23	22	21	20	19	18	17	16	15	 …	  0|
 *------------------------------------------------------------------------------
 *| L	E	D	R	R	R	R	R	|<---- Reserved          ---->  |<- Length-> |
 *------------------------------------------------------------------------------
 *| <---------------------------- Data Ptr ----------------------------------->|
 *------------------------------------------------------------------------------
 *
 * L bit (LAST): If set, means that this buffer of data is the last buffer of the frame
 * E bit (END): If set, we reached the end of the buffers passed to the function
 * D bit (DONE): Only valid on the read callback. When set, means that the buffer has been
 * filled by the SDMA.
 * Length: Length of data pointed by this node in bytes
 * Data Ptr: Pointer to the data pointed to by this node.
 * The Function Shall not be called for the same channel unless the Read callback has been
 * received for channel for which it has been called already.
 *
 * @return
 *       - IAPI_SUCCESS on success, IAPI_ERROR otherwise
 *
 *- -iapi_errno if failure
 */

int iapi_Read_ipcv2(channelDescriptor *cd_p, void *data_control_struct_ipcv2)
{
	channelControlBlock *ccb_p;
/* The Parameters passed are considered to be validated by the upper layers */
	bufferDescriptor_ipcv1_v2 *bd_ipcv2_p;
	dataNodeDescriptor    *dnd_p = data_control_struct_ipcv2;

	ccb_p = cd_p->ccb_ptr;
	iapi_errno = IAPI_ERR_NO_ERROR;

	if (ccb_p->baseBDptr == DMA_ADDR_INVALID) {
		iapi_errno = IAPI_ERR_BD_UNINITIALIZED;
		return -iapi_errno;
	}

	ccb_p->currentBDptr = ccb_p->baseBDptr;

	/* Copy the data Node descriptor information to new BDs */
	bd_ipcv2_p = iapi_Phys2Virt(ccb_p->baseBDptr);

	while (1) {
		bd_ipcv2_p->bufferAddr = dnd_p->bufferAddr;
		bd_ipcv2_p->mode.count = dnd_p->mode.count;
#ifdef MCU
		bd_ipcv2_p->mode.endianness = 1;
#endif
#ifdef DSP
		bd_ipcv2_p->mode.endianness = 0;
#endif

		bd_ipcv2_p->mode.status = dnd_2_bd_status[dnd_p->mode.status & GET_LED_MASK];

		if ((dnd_p->mode.status & DND_END_OF_XFER) != 0) {
			/* Break the loop at End of Transfer */
			break;
		}
		bd_ipcv2_p++;
		dnd_p++;
	}
	/*
	 * Store the buffer address
	 */
	dnd_read_control_struct[cd_p->channelNumber] = data_control_struct_ipcv2;
	/*
	 *  Register the Call Back
	 */

	iapi_AttachCallbackISR(cd_p, iapi_read_ipcv2_callback);

	/*
	 *  Starting of the channel
	 */
	iapi_lowStartChannel(cd_p->channelNumber);
	ccb_p->status.execute = TRUE;

	return IAPI_SUCCESS;
}


/* ***************************************************************************/
/*
 * The function is used send a group of buffers to SDMA.
 * <b>Algorithm:</b>\n
 *
 * The data control structure would be copied to IPCv1 complied Buffer
 * Descriptor Array. This array shall be allocated from non cacheable memory.
 * It would then provide this buffer descriptor array as an input to SDMA using
 * channel control block and then configure the Host Enable (HE) or
 * DSP enable (DE) bit of SDMA for the channel used for this transfer depending
 * on the source.
 * The Function Shall not be called for the same channel unless the Read callback has been
 * received for channel for which it has been called already.
 *
 * <b>Notes:</b>\n
 * Virtual DMA channels are unidirectional, an iapi_Write_ipcv2 authorized
 * on a channel means that source processor is expecting to send to the destination
 * processor. The meaning of an interrupt received from the SDMA notifies that the
 * data has been delivered to the destination processor.
 *
 * @param *cd_p chanenl descriptor for the channel to write to
 * @param *data_control_struct_ipcv2

 *   Data Control structure:
 *   -------------------------
 *   | Data Node Descriptor 1|
 *   -------------------------
 *   | Data Node Descriptor 2|
 *   -------------------------
 *   |           :           |
 *   |           :           |
 *   -------------------------
 *   |Data Node Descriptor n |
 *   -------------------------
 *
 *   Data Node Descriptor (Buffer Descriptor):
 *------------------------------------------------------------------------------
 *| 31	30	29	28	27	26	25	24	23	22	21	20	19	18	17	16	15	 …	  0|
 *------------------------------------------------------------------------------
 *| L	E	D	R	R	R	R	R	|<---- Reserved          ---->  |<- Length-> |
 *------------------------------------------------------------------------------
 *| <---------------------------- Data Ptr ----------------------------------->|
 *------------------------------------------------------------------------------
 *
 * L bit (LAST): If set, means that this buffer of data is the last buffer of the frame
 * E bit (END): If set, we reached the end of the buffers passed to the function
 * D bit (DONE): Only valid on the read callback. When set, means that the buffer has been
 * filled by the SDMA.
 * Length: Length of data pointed by this node in bytes
 * Data Ptr: Pointer to the data pointed to by this node.
 *
 *
 * @return
 *       - iapi sucess on success.
 *       - -iapi_errno if failure
 */

int iapi_Write_ipcv2(channelDescriptor *cd_p, void *data_control_struct_ipcv2)
{
	channelControlBlock *ccb_p;
/* The Parameters passed are considered to be validated by the upper layers */
	bufferDescriptor_ipcv1_v2 *bd_ipcv2_p;
	dataNodeDescriptor    *dnd_p = data_control_struct_ipcv2;
	ccb_p = cd_p->ccb_ptr;
	iapi_errno = IAPI_ERR_NO_ERROR;

	if (ccb_p->baseBDptr == DMA_ADDR_INVALID) {
		iapi_errno = IAPI_ERR_BD_UNINITIALIZED;
		return -iapi_errno;
	}

	ccb_p->currentBDptr = ccb_p->baseBDptr;

	bd_ipcv2_p = iapi_Phys2Virt(ccb_p->currentBDptr);
	/* Copy the data Node descriptor information to new BDs */
	while (1) {
		bd_ipcv2_p->bufferAddr = dnd_p->bufferAddr;
		bd_ipcv2_p->mode.count = dnd_p->mode.count;

#ifdef MCU
		bd_ipcv2_p->mode.endianness = 1;
#endif
#ifdef DSP
		bd_ipcv2_p->mode.endianness = 0;
#endif

		bd_ipcv2_p->mode.status = dnd_2_bd_status[dnd_p->mode.status & GET_LED_MASK];

		if ((dnd_p->mode.status & DND_END_OF_XFER) != 0) {
			/* Break the loop at End of Transfer */
			break;
		}
		bd_ipcv2_p++;
		dnd_p++;
	}

	/*
	 *  Starting of the channel
	 */
	iapi_lowStartChannel(cd_p->channelNumber);
	ccb_p->status.execute = TRUE;

	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/** Call back ISR for the IPCv2 Receive.
 *
 * <b>Algorithm:</b>\n
 *    - This would copy back the informationfrom IPCv1 BD to IPCv2 BD on
 * the receiving processor
 *
 * @return
 *     - void
 */

static void iapi_read_ipcv2_callback(struct iapi_channelDescriptor *cd_p, void *data)
{
	dataNodeDescriptor    *dnd_p = dnd_read_control_struct[cd_p->channelNumber];//cd_p->ccb_ptr->channelDNDBuffer;
	bufferDescriptor_ipcv1_v2 *bd_ipcv2_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
	int index;

	for (index = MAX_BD_NUM - 1; index >= 0; index--) {
		dnd_p->mode.status = 0;
		dnd_p->mode.count = bd_ipcv2_p->mode.count;

		dnd_p->mode.status |= bd_ipcv2_p->mode.status & BD_DONE ? 0x00 : DND_DONE;
		dnd_p->mode.status |= bd_ipcv2_p->mode.status & BD_IPCV2_END_OF_FRAME ? DND_END_OF_FRAME : 0x00;
		dnd_p->mode.status |= bd_ipcv2_p->mode.status & BD_LAST ? DND_END_OF_XFER : 0x00;
		cd_p->ccb_ptr->currentBDptr = iapi_Virt2Phys(bd_ipcv2_p);

		if ((bd_ipcv2_p->mode.status & BD_LAST) ||
			!(bd_ipcv2_p->mode.status & BD_CONT))
			break;
		dnd_p++;
		bd_ipcv2_p++;
	}

	/* Call back the Original ISR */
	cd_p->callbackISR_ptr(cd_p, data);
}

/* ***************************************************************************/
/**Terminates a channel.
 *
 * <b>Algorithm:</b>\n
 *   - Check input parameters ans data structures
 *   - Check that all buffes have been processed (test all 'D' bits)
 *   - Stop the channel execution
 *   - Free alocated memory structures
 *   - Re-instantiate default interrupt handling
 *
 * @param *cd_p chanenl descriptor for the channel to close
 *
 * @return
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : close failed
 */
int
iapi_Close(channelDescriptor *cd_p)
{
	int index = 0;
	unsigned char chNum;
	channelControlBlock *ccb_p;

	/*
	 * 1. Check input parameters ans data structures
	 */
	if (cd_p != NULL) {
		if (cd_p->ccb_ptr != NULL) {
			chNum = cd_p->channelNumber;
			ccb_p = cd_p->ccb_ptr;
		} else {
			iapi_errno = IAPI_ERR_NO_CCB_DEFINED | IAPI_ERR_CH_AVAILABLE;
			return -iapi_errno;
		}
	} else {
		iapi_errno = IAPI_ERR_CD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE;
		return -iapi_errno;
	}
	/* Try to aquire channel */
	if (iapi_GetChannel(chNum) != 0) {
		iapi_errno = IAPI_ERR_CH_IN_USE | chNum;
		return -iapi_errno;
	}

	/*
	 * 2. Check that all buffes have been processed (test all 'D' bits),
	 * only if the forceClose bit in channel descriptor is set to FALSE
	 */
	if (ccb_p->baseBDptr != DMA_ADDR_INVALID) {
		bufferDescriptor *bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);

		if (bd_p == NULL) {
			iapi_errno = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
			return -iapi_errno;
		}
		if (cd_p->forceClose == FALSE) {
			for (index = cd_p->bufferDescNumber; index > 0; index--) {
				if (bd_p->mode.status & BD_DONE) {
					iapi_errno = IAPI_ERR_CLOSE | IAPI_ERR_CH_AVAILABLE | chNum;
					iapi_ReleaseChannel(chNum);
					return -iapi_errno;
				}
				bd_p++;
			}
		} else {
			/* if the closing is forced, mark channel unused and
			 * set BD ownership to processor
			 */
			ccb_p->status.execute = FALSE;
			for (index = cd_p->bufferDescNumber; index > 0; index--) {
				bd_p->mode.status &= ~BD_DONE;
				bd_p++;
			}
		}
	}

	/*
	 * 3. Stop the channel execution
	 */
	iapi_lowStopChannel(chNum);

	/*
	 * 4. Free alocated memory structures
	 */
	if (cd_p->trust == FALSE && ccb_p->baseBDptr != DMA_ADDR_INVALID) {
		bufferDescriptor *bd_p = iapi_Phys2Virt(ccb_p->baseBDptr);

		for (index = cd_p->bufferDescNumber; index > 0; index--) {
			FREE(iapi_Phys2Virt(bd_p->bufferAddr));
			bd_p++;
		}
	}

	/*
	 * 5. Re-instantiate default interrupt handling
	 */
	iapi_DetachCallbackISR(cd_p);
	if (ccb_p->baseBDptr != DMA_ADDR_INVALID) {
		FREE(iapi_Phys2Virt(ccb_p->baseBDptr));
		ccb_p->baseBDptr = DMA_ADDR_INVALID;
		ccb_p->currentBDptr = DMA_ADDR_INVALID;
	}
	FREE(cd_p);
	ccb_p->channelDescriptor = NULL;
	ccb_p->status.openedInit = FALSE;

	iapi_ReleaseChannel(chNum);

	return IAPI_SUCCESS;
}

static inline int iapi_clr_status(channelDescriptor *cd_p, unsigned long param,
				unsigned long mask)
{
	int retvalue = IAPI_SUCCESS;
	bufferDescriptor *bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	if (param == SET_BIT_ALL) {
		int j;

		for (j = 0; j < cd_p->bufferDescNumber; j++) {
			bde_p->mode.status &= ~mask;
			bde_p++;
		}
	} else if (param < cd_p->bufferDescNumber) {
		bde_p[param].mode.status &= ~mask;
	} else {
		retvalue = IAPI_FAILURE;
	}
	return retvalue;
}

static inline int iapi_set_status(channelDescriptor *cd_p, unsigned long param,
				unsigned long mask)
{
	int retvalue = IAPI_SUCCESS;
	bufferDescriptor *bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	if (param == SET_BIT_ALL) {
		int j;

		for (j = 0; j < cd_p->bufferDescNumber; j++) {
			bde_p->mode.status |= mask;
			bde_p++;
		}
	} else if (param < cd_p->bufferDescNumber) {
		bde_p[param].mode.status |= mask;
	} else {
		retvalue = IAPI_FAILURE;
	}
	return retvalue;
}

static inline int iapi_set_command(channelDescriptor *cd_p, unsigned long param,
				unsigned long cmd)
{
	int retvalue = IAPI_SUCCESS;
	bufferDescriptor *bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	if (param == SET_BIT_ALL) {
		int j;

		for (j = 0; j < cd_p->bufferDescNumber; j++) {
			bde_p->mode.command = cmd;
			bde_p++;
		}
	} else if (param < cd_p->bufferDescNumber) {
		bde_p[param].mode.command = cmd;
	} else {
		retvalue = IAPI_FAILURE;
	}
	return retvalue;
}

/* ***************************************************************************/
/**The request argument selects the control function to be performed.
 *
 * <b>Algorithm:</b>\n
 *
 * - Check data structures are properly initialized:
 *   - Channel descriptor validity
 *   - Channel control block validity
 * - The ctlRequest parameter contains in the lower 16 bits the control code of
 *   the change to be performed, and in the upper 16 bits, the BD to be
 *   modified if the change affects a BD od the channel.
 * - Selection of the parameter to change and appropriate sanity checks:
 *   - Channel Descriptor: changes the pointer to the channel descriptor
 * structure, the pointer to the new channel descriptor is given in the third
 * argument call
 *   - Buffer Descriptor Number: changes the number of buffer descriptor for the
 * channel
 *   - Buffer size: changes the size of the data buffers pointed to by the
 * buffer descriptor; note that all buffer descriptors are assumed to have the
 * same size for a given buffer descripotr chain
 *   - Blocking policy: changes the blocking policy for the read and write calls
 *   - Ownership: changes direction: turnaround
 *   - Synchronization method: changes the callback type, default or user. The *
 * callback function table is set accordingly
 *   - Trust property: trust can only be changed through ChangeChannelDesc first
 * request, this guarantees the close/open sequence for the channel
 *   - Callback Interrupt service routine pointer: changes the callback function
 * pointer, when this method is used, to replace it with a new one
 *   - Channel control block pointer: not available
 *   - Priority: changes the channel priority directly in SDMA register
 *   - Watermark level: changes the value of the peripheral watermark level that
 * passed to the script. The new value is passed in the third parameter call.
 *   - Wrap bit: changes to set to 1 the Wrap bit of the last buffer descriptor
 *
 * @param *cd_p channel descriptor for the channel to modify
 * @param ctlRequest request control code and, if tha case, number of BD to be
 *                   changed
 * @param param parameter for the modification
 *
 * @return
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : operation failed
 */
int
iapi_IoCtl(channelDescriptor *cd_p, unsigned long ctlRequest,
	unsigned long param)
{
	int result = IAPI_SUCCESS;
	unsigned char chNum;
	unsigned long clean_ctlRequest; /* lower 16 bits of the ctlRequest */
	unsigned long bd_num; /* upper 16 bits of the ctlRequest */

	DBG(0, "%s: cd=%p req=%08lx param=%08lx\n", __FUNCTION__,
		cd_p, ctlRequest, param);
	/*
	 * 1. Check data structures are properly initialized
	 */
	/* Channel descriptor validity */
	if (cd_p == NULL) {
		iapi_errno = IAPI_ERR_CD_UNINITIALIZED;
		return -iapi_errno;
	}

	/* Channel control block validity */
	if (cd_p->ccb_ptr == NULL) {
		iapi_errno = IAPI_ERR_CCB_UNINITIALIZED;
		return -iapi_errno;
	}

	/* Control block & Descriptor associated with the channel being worked on */
	chNum = cd_p->channelNumber;

	/* Remove, if exists, BD number specified in upper bits of ctlRequest */
	clean_ctlRequest = ctlRequest & ~BD_NUM_MASK;

	/* Extract, if exists, BD number specified in upper bits of ctlRequest */
	bd_num = (ctlRequest & BD_NUM_MASK) >> BD_NUM_OFFSET;

	/* Check that the bd_num is valid */
	if (bd_num >= cd_p->bufferDescNumber) {
		DBG(0, "%s: BD number %lu out of range: %u\n", __FUNCTION__,
			bd_num, cd_p->bufferDescNumber);
		iapi_errno = IAPI_ERR_INVALID_PARAMETER | chNum;
		return -iapi_errno;
	}

	/* All checks OK, try to aquire channel */
	if (iapi_GetChannel(chNum) != 0) {
		iapi_errno = IAPI_ERR_CH_IN_USE | chNum;
		return -iapi_errno;
	}

	/*
	 * 2. Selection of the parameter to change and appropriate sanity checks
	 */
	switch (clean_ctlRequest) {
	case IAPI_CHANGE_CHANDESC:
		/*
		 * Channel Descriptor
		 * --- Changes the pointer to the channel descriptor structure: the pointer
		 * to the new channel descriptor is given in the third argument call.
		 */
		if ((void *)param == NULL) {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER;
			result = -iapi_errno;
		} else {
			channelDescriptor *chParam = (channelDescriptor *)param;

			if (chParam->channelNumber != chNum) {
				/* Release ch so it can be aquired by the Close fn */
				iapi_ReleaseChannel(chNum);
				result = iapi_Close(cd_p);
				if (result == IAPI_SUCCESS) {
					FREE(cd_p);
					iapi_AllocChannelDesc(&cd_p,
							chParam->channelNumber);
					iapi_memcpy(cd_p, chParam,
						sizeof(channelDescriptor));
					/* Channel is released allready, so Open can get the channel */
					result = iapi_Open(cd_p,
							chParam->channelNumber);
					if (result != IAPI_SUCCESS) {
						return result; /* error code already set in iapi_Open */
					}
				} else {
					return result; /* error code already set in iapi_Close */
				}
			} else {
				iapi_errno = IAPI_ERR_CD_CHANGE |
					IAPI_ERR_CH_AVAILABLE |
					cd_p->channelNumber;
				result = -iapi_errno;
				break;
			}
		}
		break;

	case IAPI_CHANGE_BDNUM:
		/*
		 * Buffer Descriptor Number
		 * --- Changes the number of buffer descriptor for the channel.
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERDESCNUMBER, param);
		break;

	case IAPI_CHANGE_BUFFSIZE:
		/*
		 * Buffer size
		 * --- Changes the size of the data buffers pointed to by the buffer
		 * descriptor; note that all buffer descriptors are assumed to have the
		 * same size for a given buffer descripotr chain.
		 */
		result =  iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERSIZE, param);
		break;

	case IAPI_CHANGE_CHANBLOCK:
		/*
		 * Blocking policy
		 * --- Changes the blocking policy for the read and write calls.
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_BLOCKING, param);
		break;

	case IAPI_CHANGE_OWNERSHIP:
		/*
		 * Ownership
		 * --- Changes direction: turnaround
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_OWNERSHIP, param);
		break;

	case IAPI_CHANGE_SYNCH:
		/*
		 * Synchronization method
		 * --- Changes the callback type, default or user. The callback function
		 * table is set accordingly.
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_CALLBACKSYNCH, param);
		break;

	case IAPI_CHANGE_TRUST:
		/*
		 * Trust property
		 * --- trust can only be changed through ChangeChannelDesc first request,
		 * this guarantees the close/open sequence for the channel.
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_TRUST, param);
		break;

	case IAPI_CHANGE_CALLBACKFUNC:
		/*
		 * Callback Interrupt service routine pointer
		 * --- Cahnges the callback function pointer, when this method is used, to
		 * replace it with a new one.
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_CALLBACKISR_PTR, param);
		break;

	case IAPI_CHANGE_CHANCCB:
		/*
		 * Channel control block pointer
		 * --- NA
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_CCB_PTR, param);
		break;
#ifdef MCU
	case IAPI_CHANGE_PRIORITY:
		/*
		 * Priority
		 * --- Changes the channel priority directly in SDMA register
		 */
		if (param < MAX_CH_PRIORITY) {
			__raw_writel(param, SDMA_CHNPRI(cd_p->channelNumber));
		} else {
			result = IAPI_FAILURE;
		}
		break;
#endif /* MCU */
	case IAPI_CHANGE_BDWRAP:
		/*
		 * Wrap
		 * --- Set to 1 the wrap bit of the last buffer descriptor of the array.
		 * it provides the possibility to have a circular buffer structure.
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_BDWRAP, param);
		break;

	case IAPI_CHANGE_WATERMARK:
		/*
		 * Watermark
		 * --- Changes the value of the peripheral watermark level that triggers
		 * a DMA request. It impacts context of the channel, therefore channel 0
		 * must be started to update the context with this new value.
		 */
		result = iapi_ChangeChannelDesc(cd_p, IAPI_WML, param);
		break;

	case IAPI_CHANGE_SET_BDINTR:
		/*
		 * INTR
		 * --- Set the INTR bit on specified BD or on all BD's if SET_BIT_ALL
		 * is passed as parameter.
		 */
		result = iapi_set_status(cd_p, param, BD_INTR);
		break;

	case IAPI_CHANGE_UNSET_BDINTR:
		/*
		 * INTR
		 * --- Unset the INTR bit on specified BD or on all BD's if SET_BIT_ALL
		 * is passed as parameter.
		 */
		result = iapi_clr_status(cd_p, param, BD_INTR);
		break;

	case IAPI_CHANGE_EVTMASK1:
		/*
		 * EventMask1
		 * --- Changes the value of the eventMask1
		 */
		cd_p->eventMask1 = param;
		break;

	case IAPI_CHANGE_EVTMASK2:
		/*
		 * EventMask2
		 * --- Changes the value of the eventMask2
		 */
		cd_p->eventMask2 = param;
		break;

	case IAPI_CHANGE_PERIPHADDR:
		/*
		 * Peripheral Address
		 * --- Changes the value of the peripheralAddr
		 */
		cd_p->peripheralAddr = param;
		break;

	case IAPI_CHANGE_SET_BDCONT:
		/*
		 * Cont
		 * --- Set the CONT bit on specified BD on all BD's if SET_BIT_ALL
		 * is passed as parameter.
		 */
		result = iapi_set_status(cd_p, param, BD_CONT);
		break;

	case IAPI_CHANGE_UNSET_BDCONT:
		/*
		 * Cont
		 * --- Unset the CONT bit on specified BD or on all BD's if SET_BIT_ALL
		 * is passed as parameter.
		 */
		result = iapi_clr_status(cd_p, param, BD_CONT);
		break;

	case IAPI_CHANGE_SET_BDEXTD:
		/*
		 * EXTD
		 * --- Set the EXTD bit on specified BD or on all BD's if SET_BIT_ALL
		 * is passed as parameter.
		 */
		result = iapi_set_status(cd_p, param, BD_EXTD);
		break;

	case IAPI_CHANGE_UNSET_BDEXTD:
		/*
		 * EXTD
		 * --- Unset the EXTD bit on specified BD or on all BD's if SET_BIT_ALL
		 * is passed as parameter.
		 */
		result = iapi_clr_status(cd_p, param, BD_EXTD);
		break;

	case IAPI_CHANGE_SET_TRANSFER_CD:
		/*
		 * TRANSFER SIZE to be used for this channel
		 * --- Set the transfer size used indicator and code for transfer size in
		 * the CD
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;
			int j;

			if ((param == TRANSFER_8BIT) || (param == TRANSFER_16BIT) ||
				(param == TRANSFER_24BIT) || (param == TRANSFER_32BIT)) {
				cd_p->useDataSize = TRUE;
				cd_p->dataSize = param;
				bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
				for (j = 0; j < cd_p->bufferDescNumber; j++) {
					bde_p->mode.command = param;
					bde_p++;
				}
			} else {
				result = IAPI_FAILURE;
			}
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_USER_ARG:
		/*
		 * USER_ARG
		 * --- Set the user selectable pointer to be received by the callback
		 * function, if IRQ synch is used
		 */
		userArgTable[cd_p->channelNumber]= (void *)param;
		break;

	case IAPI_CHANGE_FORCE_CLOSE:
		/*
		 * FORCE_CLOSE
		 * --- Set the forceClose bit in channelDescriptor to value passed in param.
		 * If this bit is TRUE, the channel in closed even if some BD are still
		 * owned by the SDMA.
		 */
		if ((param == TRUE) || (param == FALSE)) {
			cd_p->forceClose = param;
		} else {
			iapi_errno = IAPI_ERR_INVALID_PARAMETER | cd_p->channelNumber;
			result = -iapi_errno;
		}
		break;

	case IAPI_CHANGE_SET_TRANSFER:
		/*
		 * TRANSFER type
		 * --- Set the last 2 bits in the command field of the BD to specify the
		 * transfer type 8, 16, 24, or 32 bits on all BD's, allready set in the CD
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			if ((param == TRANSFER_8BIT) || (param == TRANSFER_16BIT) ||
				(param == TRANSFER_24BIT) || (param == TRANSFER_32BIT)) {
				int j;

				bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
				for (j = 0; j < cd_p->bufferDescNumber; j++) {
					bde_p->mode.command = param;
					bde_p++;
				}
			} else {
				result = IAPI_FAILURE;
			}
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_SET_BUFFERADDR:
		/*
		 * BUFFER address
		 * --- Change buffer address in BD specified in the upper 16 bits of the
		 * ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;

			/* DO NOT translate address to physical */
			bde_p->bufferAddr = (dma_addr_t)param;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_GET_BUFFERADDR:
		/*
		 * BUFFER address
		 * --- Get the buffer address from the BD specified in the upper 16 bits of the
		 * ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;
			dma_addr_t *retval = (dma_addr_t *)param;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			/* DO NOT Translate to virtual */
			*retval = bde_p->bufferAddr;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_SET_EXTDBUFFERADDR:
		/*
		 * EXTENDED BUFFER address
		 * --- Change extended buffer address in BD specified in the upper 16 bits
		 * of the ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;

			/* DO NOT translate address to physical. The user might want something else
			 * here
			 */
			bde_p->extBufferAddr = (dma_addr_t)param;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_GET_EXTDBUFFERADDR:
		/*
		 * EXTENDED BUFFER address
		 * --- Get extended buffer address from the BD specified in the upper 16 bits
		 * of the ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;
			dma_addr_t *retval = (dma_addr_t *)param;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;

			/* DO NOT translate address to vitual - user knows what is here.
			 */
			*retval = bde_p->extBufferAddr;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_SET_COMMAND:
		/*
		 * COMMAND field
		 * --- Change command field in BD specified in the upper 16 bits of the
		 * ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			/* Update command field */
			bde_p->mode.command = param;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_GET_COMMAND:
		/*
		 * COMMAND field
		 * --- Get the command field from the BD specified in the upper 16 bits
		 * of the ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			/* Get the command field */
			*((unsigned long *)param) = bde_p->mode.command;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_SET_COUNT:
		/*
		 * COUNT field
		 * --- Change count field in BD specified in the upper 16 bits of the
		 * ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			/* Update count field */
			bde_p->mode.count = param;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_GET_COUNT:
		/*
		 * COUNT field
		 * --- Get the count field of the BD specified in the upper 16 bits of the
		 * ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			/* Update count field */
			*((unsigned long *)param) = bde_p->mode.count;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_SET_STATUS:
		/*
		 * STATUS field
		 * --- Change status field in BD specified in the upper 16 bits of the
		 * ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			/* Update status field */
			DBG(1, "%s: BD[%ld] %08x->%08lx\n", __FUNCTION__,
				bd_num, bde_p->mode.status, param);
			bde_p->mode.status = param;
		} else {
			result = IAPI_FAILURE;
		}
		break;

	case IAPI_CHANGE_GET_STATUS:
		/*
		 * STATUS field
		 * --- Get the status field of the BD specified in the upper 16 bits
		 * of the ctlRequest.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			/* Get pointer to the BD structure to change */
			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			/* Update status field */
			*((unsigned long *)param) = bde_p->mode.status;
		} else {
			result = IAPI_FAILURE;
		}
		break;
#ifdef MCU
	case IAPI_CHANGE_SET_ENDIANNESS:
		/*
		 * Endianness
		 * --- Set the ENDIANNESS indicator in the command filed of the specified BD
		 * or on all BD's if SET_BIT_ALL is passed as parameter.
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

			if (param == SET_BIT_ALL) {
				int j;

				for (j = 0; j < cd_p->bufferDescNumber; j++, bde_p++) {
					/* Clear the respective bits in the command field
					 * and set the new parameter value
					 */
					bde_p->mode.command &= ~ENDIANNESS_MASK;
					bde_p->mode.command |= CHANGE_ENDIANNESS &
						ENDIANNESS_MASK;
				}
			} else if (param < cd_p->bufferDescNumber) {
				bde_p[param].mode.command &= ~ENDIANNESS_MASK;
				bde_p[param].mode.command |= CHANGE_ENDIANNESS &
					ENDIANNESS_MASK;
			} else {
				result = IAPI_FAILURE;
			}
		} else {
			result = IAPI_FAILURE;
		}
		break;
#ifdef SDMA_V2
	case IAPI_ENTER_LOCK_MODE:
		/*
		 * SDMA State
		 * --- Enter the SDMA into LOCK Mode. No RAM update allowed except same Context
		 * update with same PC Value.
		 */
		if (param == RESET_CLEAR_LOCK) {
			__raw_writel(1 << RESET_CLR_BIT_OFFSET, SDMA_SDMA_LOCK);
			__raw_writel(1 << LOCK_BIT_OFFSET, SDMA_SDMA_LOCK);
			iapi_SdmaState = LOCK;
		} else if (param == RESET_NOCLEAR_LOCK) {
			__raw_writel(1 << LOCK_BIT_OFFSET, SDMA_SDMA_LOCK);
			iapi_SdmaState = LOCK;
		}
		break;
#endif
#endif
	case IAPI_CHANGE_SET_INTR_MASK:
		/*
		 * Interrupt Mask
		 * --- Sets the Interrupt Mask directly in SDMA register. Can be used  to set
		 * mask per channel or for all channels(SET_BIT_ALL)
		 * In case of error, the error reflects the channel number the error is received for.
		 */
		if (param == SET_BIT_ALL) {
			result = iapi_lowChangeIntrMask(SET_BIT_ALL, OR_OP);
		} else {
			/* chnum is Extracted earlier in iapi_Ioctl() and checked for validity */
			result = iapi_lowChangeIntrMask(1 << chNum, OR_OP);
		}
		/* iapi_errno has been set by iapi_lowChangeIntrMask() */
		if (result != IAPI_SUCCESS)
			result |= IAPI_ERR_CH_AVAILABLE | chNum;
		break;

	case IAPI_CHANGE_UNSET_INTR_MASK:
		/*
		 * Interrupt Mask
		 * --- Clears the Interrupt Mask directly in SDMA register. Can be used  to clear
		 * mask per channel or for all channels(SET_BIT_ALL)
		 * In case of error, the error reflects the channel number the error is received for.
		 */
		if (param == SET_BIT_ALL) {
			result = iapi_lowChangeIntrMask(~SET_BIT_ALL, AND_OP);
		} else {
			result = iapi_lowChangeIntrMask(~(1 << chNum), AND_OP);
		}
		/* iapi_errno has been set by iapi_lowChangeIntrMask() */
		if (result != IAPI_SUCCESS)
			result |= IAPI_ERR_CH_AVAILABLE | chNum;
		break;

	case IAPI_CHANGE_BUFFER_LOCATION:
		/* Buffer Location
		 * Set whether Buffer is located in External Memory or Internal Memory
		 */
		if (cd_p->ccb_ptr->baseBDptr != DMA_ADDR_INVALID) {
			bufferDescriptor *bde_p;

			bde_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
			bde_p += bd_num;
			if ((param == EXTERNAL_MEM) || (param == INTERNAL_MEM)) {
				/* Clear the respective bits in the command field
				 * and set the new parameter value
				 */
				bde_p->mode.command &= ~BUFFER_LOC_MASK;
				bde_p->mode.command |= param & BUFFER_LOC_MASK;
			} else {
				result = IAPI_FAILURE;
			}
		} else {
			result = IAPI_FAILURE;
		}
		break;

	default:
		iapi_errno = IAPI_ERR_CD_CHANGE_UNKNOWN |
			IAPI_ERR_CH_AVAILABLE | chNum;
		result = -iapi_errno;
	}

	iapi_ReleaseChannel(chNum);
	return result;
}

/* ***************************************************************************/
/**Initialization of the SDMA - opening of channel 0, download RAM image.
 *
 * <b>Algorithm:</b>\n
 *     - open channel 0
 *     - if ram_image pointer passed is not NULL, download RAM image to SDMA
 *
 * @param
 *     - cd_p channel descriptor pointer for channel 0
 *     - ram_image pointer to RAM image to download, or NULL if this operation
 *                 is not required
 *     - code_size size of the RAM image, in bytes
 *     - start_addr start address for the RAM image
 *
 * @return
 *     - IAPI_SUCCESS if all operations were successful
 *     - negated I.API error code if any operation failed
 */
#ifdef MCU
int
iapi_Init(channelDescriptor *cd_p, configs_data *config_p, unsigned short *ram_image,
	unsigned short code_size, dma_addr_t start_addr, unsigned short channel0_addr)
{
#endif
#ifdef DSP
int
iapi_Init(channelDescriptor *cd_p)
{
#endif
	int retvalue;  /* Variable to store the results from I.API calls */

	/* Check initialization not allredy done */
	if (iapi_CCBHead != NULL) {
		iapi_errno = IAPI_ERR_NOT_ALLOWED;
		return -iapi_errno;
	}
	/* Be sure SDMA has not started yet */
#ifdef MCU
	__raw_writel(0, SDMA_H_C0PTR);
#endif
#ifdef DSP
	__raw_writel(0, SDMA_D_C0PTR);
#endif

	/* Try to open channel 0 */
	retvalue = iapi_Open(cd_p, 0);
	if (retvalue != IAPI_SUCCESS) {
		return retvalue;
	}
#if 0
	print_hex_dump(KERN_DEBUG, "sdma: ", DUMP_PREFIX_ADDRESS, 4, 4,
		cd_p->ccb_ptr, CH_NUM * sizeof(channelControlBlock), 0);
#endif
#ifdef MCU
	/* Set Command Channel (Channel Zero) */
	__raw_writel(0x4000 | (channel0_addr & 0x3FFF), SDMA_CHN0ADDR);

	/* Set bits of CONFIG register but with static context switching */
	__raw_writel((config_p->dspdma << 12) | (config_p->rtdobs << 11) |
		     (config_p->acr << 4), SDMA_H_CONFIG);

	/* Send the address for the host channel table to the SDMA */
	__raw_writel(iapi_Virt2Phys(iapi_CCBHead), SDMA_H_C0PTR);
	/* If required, download the RAM image for SDMA */
	if (ram_image != NULL) {
		retvalue = iapi_SetScript(cd_p, ram_image, code_size,
					start_addr);
	}

	/* Set bits of CONFIG register with given context switching mode */
	__raw_writel((config_p->dspdma << 12) | (config_p->rtdobs << 11) |
		     (config_p->acr << 4) | config_p->csm, SDMA_H_CONFIG);

#endif
#ifdef DSP
	/* Send the address for the host channel table to the SDMA */
	__raw_writel(iapi_Virt2Phys(iapi_CCBHead), SDMA_D_C0PTR);
#endif

#ifdef SDMA_V2
	iapi_SdmaState = OPEN;
#endif
	return retvalue;
}


/* ***************************************************************************/
/**High layer interface for starting a channel
 *
 * <b>Algorithm:</b>\n
 *    - call low layer function for starting a channel
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_StartChannel(unsigned char channel)
{
	iapi_lowStartChannel(channel);
	return IAPI_SUCCESS;
}
/* ***************************************************************************/
/**High layer interface for stopping a channel
 *
 * <b>Algorithm:</b>\n
 *    - call low layer function for stopping a channel
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_StopChannel(unsigned char channel)
{
	iapi_lowStopChannel(channel);
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for synchronising a channel
 *
 * <b>Algorithm:</b>\n
 *    - call low layer function for stopping a channel
 *
 * @return
 *     - IAPI_SUCCESS
 */
int iapi_SynchChannel(unsigned char channel)
{
	iapi_lowSynchChannel(channel);
	return IAPI_SUCCESS;
}

#ifdef MCU
/* ***************************************************************************/
/**High layer interface for getting program memory data from SDMA
 *
 * <b>Algorithm:</b>\n
 *    - call coresponding low layer function
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_GetScript(channelDescriptor *cd_p, void *buf, unsigned short size,
	dma_addr_t address)
{
	iapi_lowGetScript(cd_p, buf, size, address);
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for getting data memory from SDMA
 *
 * <b>Algorithm:</b>\n
 *    - call coresponding low layer function
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_GetContext(channelDescriptor *cd_p, void *buf, unsigned char channel)
{
	iapi_lowGetContext(cd_p, buf, channel);
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for set program memory data to SDMA - e.g. scripts
 *
 * <b>Algorithm:</b>\n
 *    - call coresponding low layer function
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_SetScript(channelDescriptor *cd_p, void *buf, unsigned short nbyte,
	dma_addr_t destAddr)
{
	iapi_lowSetScript(cd_p, buf, nbyte, destAddr);
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for set data memory to SDMA - e.g. contexts.
 *
 * <b>Algorithm:</b>\n
 *    - call coresponding low layer function
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_SetContext(channelDescriptor *cd_p, void *buf, unsigned char channel)
{
	iapi_lowSetContext(cd_p, buf, channel);
	return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface used to associate specified channel with a script.
 *
 * <b>Algorithm:</b>\n
 *    - call coresponding low layer function
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_AssignScript(channelDescriptor *cd_p, script_data *data_p)
{
	/* VERIFY THAT THE CHANNEL IT IS OPENED !!!! */
	return iapi_lowAssignScript(cd_p, data_p);
}

/* ***************************************************************************/
/**High layer interface used to associate specified channel with a script.
 *
 * <b>Algorithm:</b>\n
 *    - call coresponding low layer function
 *
 * @return
 *     - IAPI_SUCCESS
 */
int
iapi_SetChannelEventMapping(unsigned char event, unsigned long channel_map)
{
	return iapi_lowSetChannelEventMapping(event, channel_map);
}
#endif



#ifdef DSP
#define SDMA_DI  SDMA_D_INTR
void IRQ_Handler();
#pragma interrupt IRQ_Handler
#endif

#ifdef MCU
#define SDMA_DI  SDMA_H_INTR
#endif

#ifndef IRQ_KEYWORD
#define IRQ_KEYWORD
#endif /* IRQ_KEYWORD */

/* ***************************************************************************/
/**
 *@brief  Find the first set bit in data parameter.
 *
 *       Find the first set bit in unsigned integer parameter data. Data is scanned
 *  from MSB to LSB, searching for the set bit. The value returned is the
 *  offset from the most significant bit of data. If bit 31 is set, the value
 *  returned is zero. If no bits are set, a value of 32 is returned. This is compliant
 *  with the MCore FF1 instruction.
 *
 *
 *
 * @param
 *   - data: variable to check
 *
 * @return
 *   - the offset of the most significant bit set from the MSB
 */
static unsigned int
quartz_FF1(unsigned int data)
{
	register unsigned int result = 0;
	while ((result <= 31) && !(data & 0x80000000U)) {
		data <<= 1U;
		result++;
	}

	return result;
}

static void dump_chan(channelControlBlock *ccb, int chan)
{
	dma_addr_t bd_phys = ccb[chan].baseBDptr;
	int i;

	if (bd_phys == DMA_ADDR_INVALID)
		return;

	while (1) {
		bufferDescriptor *bd = iapi_Phys2Virt(bd_phys);

		DBG(1, "BD[%2d]@%08x count=%d CONT=%d DONE=%d WRAP=%d LAST=%d INTR=%d ERR=%d EXTD=%d\n",
			i, iapi_Virt2Phys(bd), bd->mode.count,
			!!(bd->mode.status & BD_CONT),
			!!(bd->mode.status & BD_DONE),
			!!(bd->mode.status & BD_WRAP),
			!!(bd->mode.status & BD_LAST),
			!!(bd->mode.status & BD_INTR),
			!!(bd->mode.status & BD_RROR),
			!!(bd->mode.status & BD_EXTD));
		if (bd->mode.status & (BD_LAST | BD_WRAP)) {
			break;
		}
		bd_phys += ((bd->mode.status & BD_EXTD) ?
			SDMA_EXTENDED_BD_SIZE : SDMA_BD_SIZE);
		i++;
	}
}

static void dump_dma(void)
{
	dma_addr_t reg;

	reg = __raw_readl(SDMA_H_C0PTR);
	if (reg != DMA_ADDR_INVALID) {
		channelControlBlock *ccb = iapi_Phys2Virt(reg);
		int chan;

		for (chan = 0; chan < CH_NUM; chan++) {
			dump_chan(ccb, chan);
		}
	}
}

IRQ_KEYWORD
void
IRQ_Handler(void)
{
	unsigned int intrReg;/* interrupt register mask for clearing the interrupt bit */
	unsigned char chNum; /* SDMA channel number generating the IRQ */

	/* Disable interrupts */
	iapi_DisableInterrupts();
	/*
	 * Clear interrupt in SDMA DI register => ACK to the SDMA the IT request.
	 * Get each interrupt number, clear them one after the other.
	 */
	if (__raw_readl(SDMA_DI) != 0) {
		chNum = CH_NUM - 1 - quartz_FF1(__raw_readl(SDMA_DI));
		intrReg = 1 << chNum;
	} else {
		chNum = 32;
		intrReg = 0;
	}
	DBG(0, "%s: SDMA_DI=%08x\n", __FUNCTION__, __raw_readl(SDMA_DI));
#ifdef DEBUG
	dump_dma();
#endif
	while (intrReg != 0) {
		DBG(0, "%s: ACK %08x\n", __FUNCTION__, intrReg);
		__raw_writel(intrReg, SDMA_DI);
		iapi_SDMAIntr |= intrReg;
		iapi_WakeUp(chNum);
		if (callbackIsrTable[chNum] != NULL) {
			/* release channel before callback, so IoCtl's are available */
			iapi_ReleaseChannel(chNum);
			callbackIsrTable[chNum](iapi_CCBHead[chNum].channelDescriptor,
						userArgTable[chNum]);
		}

		chNum = CH_NUM - 1 - quartz_FF1(__raw_readl(SDMA_DI));
		intrReg = 1 << chNum;
	}

	/* Enable interrupts */
	iapi_EnableInterrupts();
	DBG(0, "%s: Done\n", __FUNCTION__);
}

/* ***************************************************************************/
/**
 *@brief  Perform a memory copy operation, in the memory of the same processor
 *
 *       Size bytes are copied from the src address to dest address. It is used
 *    the channel pointed by cd_p, which must be configured prior to this call:
 *    opened, associated with the script to perform the operation - DSP_2_DSP,
 *    or MCU_2_MCU - and have the synchronization option set.
 *
 *
 *
 * @param
 *   - cd_p: channel configured to perform DSP_2_DSP or MCU_2_MCU transfers
 *   - dest: destination memory address
 *   - src : source memory address
 *   - size: number of bytes to copy from src to dest
 *
 * @return
 *   - the offset of the most significant bit set from the MSB
 */

int iapi_MemCopy(channelDescriptor *cd_p, void *dest, void *src, unsigned long size)
{
	int result = IAPI_SUCCESS;
	bufferDescriptor *bd_p;

	/* Channel descriptor validity */
	if (cd_p == NULL) {
		iapi_errno = IAPI_ERR_CD_UNINITIALIZED;
		return -iapi_errno;
	}

	/* Check and set correct parameter */
	if (cd_p->trust != TRUE) {
		result = iapi_ChangeChannelDesc(cd_p, IAPI_TRUST, TRUE);
	}

	if (cd_p->bufferDescNumber != 1) {
		result = iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERDESCNUMBER, 1);
		if (result != IAPI_SUCCESS) {
			return result;
		}
	}

	if (cd_p->bufferSize != size) {
		result = iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERSIZE, size);
		if (result != IAPI_SUCCESS) {
			return result;
		}
	}
	/* Set addresses */
	bd_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
	bd_p->bufferAddr = iapi_Virt2Phys(src);
	bd_p->extBufferAddr = iapi_Virt2Phys(dest);

	/* Set mode */
	bd_p->mode.count = size;
	bd_p->mode.command = 0x00;
	bd_p->mode.status = BD_INTR | BD_EXTD | BD_DONE | BD_WRAP;

	/* Decide if we sleep or not */
	if (cd_p->callbackSynch == DEFAULT_POLL) {
		iapi_StartChannel(cd_p->channelNumber);
		/* Call synchronization routine */
		iapi_SynchChannel(cd_p->channelNumber);
	} else {
		/* Just start the channel */
		iapi_StartChannel(cd_p->channelNumber);
	}

	return result;
}

/* ***************************************************************************/
/**Return the channel number from the channel descriptor
 *
 * @param cd_p  pointer to channel descriptor to obtain the channel number
 *
 * @return
 *     - the channel number
 *
 */
int iapi_GetChannelNumber(channelDescriptor *cd_p)
{
	return cd_p->channelNumber;
}

/* ***************************************************************************/
/**Return the error bit from the current BD of the channel
 *
 *
 * @param cd_p  pointer to channel descriptor
 *
 * @return
 *     - 0 if no error detected
 *     - BD_RROR | DATA_ERROR if error detected
 *
 */
unsigned long iapi_GetError(channelDescriptor *cd_p)
{
	bufferDescriptor *bd_p = iapi_Phys2Virt(cd_p->ccb_ptr->currentBDptr);

	if (bd_p == NULL)
		return -EINVAL;

	return (bd_p->mode.status & BD_RROR) |
		cd_p->ccb_ptr->status.data_error;
}

/* ***************************************************************************/
/**Return the count from the current BD of the channel
 *
 *
 * @param cd_p  pointer to channel descriptor
 *
 * @return
 *     - count field of the current BD for the channel
 *
 */
int iapi_GetCount(channelDescriptor *cd_p)
{
	bufferDescriptor *bd_p = iapi_Phys2Virt(cd_p->ccb_ptr->currentBDptr);

	if (bd_p == NULL)
		return -EINVAL;

	return bd_p->mode.count;
}

/* ***************************************************************************/
/**Return the sum of counts for all the BD's owned by the processor for
 * the channel specified by the received parameter.
 *
 *
 * @param cd_p  pointer to channel descriptor
 *
 * @return
 *     - sum of count fields
 *
 */
int iapi_GetCountAll(channelDescriptor *cd_p)
{
	int retval = 0;
	int i;
	bufferDescriptor *bd_p;

	bd_p = iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);

	for (i = 0; i < cd_p->bufferDescNumber &&
		     !(bd_p->mode.status & BD_DONE); i++, bd_p++) {
		retval += bd_p->mode.count;
	}
	return retval;
}
