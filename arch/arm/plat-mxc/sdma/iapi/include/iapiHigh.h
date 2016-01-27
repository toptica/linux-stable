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
 * File: iapiHigh.h
 *
 * $Id iapiHigh.h $
 *
 * Description:
 *  prototypes for high level function of I.API
 *
 *
 *  http://venerque.sps.mot.com/pjt/sfs/www/iapi/softsim_api.pdf
 *
 * $Log iapiHigh.h
 *
 * ***************************************************************************/

#ifndef _iapiHigh_h
#define _iapiHigh_h

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include "sdmaStruct.h"

/* ****************************************************************************
 * Macro-command Section
 *****************************************************************************/
enum {
	IAPI_CHANGE_CHANDESC,		/* 0x00 */
	IAPI_CHANGE_BDNUM,		/* 0x01 */
	IAPI_CHANGE_BUFFSIZE,		/* 0x02 */
	IAPI_CHANGE_CHANBLOCK,		/* 0x03 */
	IAPI_CHANGE_INSTANCE,		/* 0x04 */
	IAPI_CHANGE_OWNERSHIP,		/* 0x05 */
	IAPI_CHANGE_SYNCH,		/* 0x06 */
	IAPI_CHANGE_TRUST,		/* 0x07 */
	IAPI_CHANGE_CALLBACKFUNC,	/* 0x08 */
	IAPI_CHANGE_CHANCCB,		/* 0x09 */
	IAPI_CHANGE_PRIORITY,		/* 0x0a */
	IAPI_CHANGE_BDWRAP,		/* 0x0b */
	IAPI_CHANGE_WATERMARK,		/* 0x0c */
	IAPI_CHANGE_SET_BDCONT,		/* 0x0d */
	IAPI_CHANGE_UNSET_BDCONT,	/* 0x0e */
	IAPI_CHANGE_SET_BDEXTD,		/* 0x0f */
	IAPI_CHANGE_UNSET_BDEXTD,	/* 0x10 */
	IAPI_CHANGE_EVTMASK1,		/* 0x11 */
	IAPI_CHANGE_EVTMASK2,		/* 0x12 */
	IAPI_CHANGE_PERIPHADDR,		/* 0x13 */
	IAPI_CHANGE_SET_BDINTR,		/* 0x14 */
	IAPI_CHANGE_UNSET_BDINTR,	/* 0x15 */
	IAPI_CHANGE_SET_TRANSFER_CD,	/* 0x16 */
	IAPI_CHANGE_FORCE_CLOSE,	/* 0x17 */
	IAPI_CHANGE_SET_TRANSFER,	/* 0x18 */
	IAPI_CHANGE_USER_ARG,		/* 0x19 */
	IAPI_CHANGE_SET_BUFFERADDR,	/* 0x1a */
	IAPI_CHANGE_SET_EXTDBUFFERADDR,	/* 0x1b */
	IAPI_CHANGE_SET_COMMAND,	/* 0x1c */
	IAPI_CHANGE_SET_COUNT,		/* 0x1d */
	IAPI_CHANGE_SET_STATUS,		/* 0x1e */
	IAPI_CHANGE_GET_BUFFERADDR,	/* 0x1f */
	IAPI_CHANGE_GET_EXTDBUFFERADDR,	/* 0x20 */
	IAPI_CHANGE_GET_COMMAND,	/* 0x21 */
	IAPI_CHANGE_GET_COUNT,		/* 0x22 */
	IAPI_CHANGE_GET_STATUS,		/* 0x23 */
	IAPI_CHANGE_BUFFER_LOCATION,	/* 0x24 */
	IAPI_CHANGE_SET_ENDIANNESS,	/* 0x25 */
#ifdef SDMA_V2
	IAPI_ENTER_LOCK_MODE,		/* 0x26 */
#endif
	IAPI_CHANGE_SET_INTR_MASK,	/* 0x27 */
	IAPI_CHANGE_UNSET_INTR_MASK,	/* 0x28 */
};


/*
 * Public Function Prototype Section
 */
int iapi_Open(channelDescriptor *cd_p, unsigned char channelNumber);
int iapi_Close(channelDescriptor *cd_p);
int iapi_Read(channelDescriptor *cd_p, void *buf, unsigned short nbyte);
int iapi_Write(channelDescriptor *cd_p, void *buf, unsigned short nbyte);
int iapi_MemCopy(channelDescriptor *cd_p, void* dest, void* src,
		unsigned long size);
int iapi_IoCtl(channelDescriptor *cd_p, unsigned long ctlRequest,
				unsigned long param);


int iapi_Read_ipcv2(channelDescriptor *cd_p, void *data_control_struct_ipcv2);

int iapi_Write_ipcv2(channelDescriptor *cd_p, void *data_control_struct_ipcv2);

#ifdef MCU
int iapi_Init(channelDescriptor *cd_p, configs_data *config_p,
	unsigned short *ram_image, unsigned short code_size,
	dma_addr_t start_addr, unsigned short channel0_addr);
#endif /* MCU */
#ifdef DSP
int iapi_Init(channelDescriptor *cd_p);
#endif /* DSP */

int iapi_StartChannel(unsigned char channel);
int iapi_StopChannel(unsigned char channel);
int iapi_SynchChannel(unsigned char channel);

int iapi_GetChannelNumber(channelDescriptor *cd_p);
unsigned long iapi_GetError(channelDescriptor *cd_p);
int iapi_GetCount(channelDescriptor *cd_p);
int iapi_GetCountAll(channelDescriptor *cd_p);

#ifndef IRQ_KEYWORD
#define IRQ_KEYWORD
#endif /* IRQ_KEYWORD */

IRQ_KEYWORD void IRQ_Handler(void);

#ifdef MCU
int iapi_GetScript(channelDescriptor *cd_p, void *buf, unsigned short size,
		dma_addr_t address);
int iapi_GetContext(channelDescriptor *cd_p, void *buf,
		unsigned char channel);
int iapi_SetScript(channelDescriptor *cd_p, void *buf, unsigned short nbyte,
		dma_addr_t destAddr);
int iapi_SetContext(channelDescriptor *cd_p, void *buf,
		unsigned char channel);
int iapi_AssignScript(channelDescriptor *cd_p, script_data *data_p);

int iapi_SetChannelEventMapping(unsigned char event, unsigned long channel_map);
#endif /* MCU */

#endif /* _iapiHigh_h */
