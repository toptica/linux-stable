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
 * File: iapiLow.h
 *
 * $Id iapiLow.h $
 *
 * Description:
 *  prototypes for low level function of I.API
 *
 *
 *
 *
 * $Log iapiLow.h
 *
 * ***************************************************************************/

#ifndef _iapiLow_h
#define _iapiLow_h

/* ****************************************************************************
 * Boolean identifiers
 *****************************************************************************/

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include "iapiOS.h"
#include <linux/types.h>
#include <mach/dma.h>

/* ****************************************************************************
 * Macro-command Section
 *****************************************************************************/
enum
{
	OR_OP,
	AND_OP
};

/* ****************************************************************************
 * Public Function Prototype Section
 *****************************************************************************/
typedef void (*CallbackISR)(channelDescriptor *cd_p, void *arg);

void iapi_lowStartChannel(unsigned char channel);
void iapi_lowStopChannel(unsigned char channel);
int iapi_lowChangeIntrMask(unsigned int param, unsigned char op);
void iapi_AttachCallbackISR(channelDescriptor *cd_p,
			    CallbackISR func_p);
void iapi_DetachCallbackISR(channelDescriptor *cd_p);
void iapi_ChangeCallbackISR(channelDescriptor *cd_p,
			    CallbackISR func_p);
void iapi_lowSynchChannel(unsigned char channel);
void iapi_SetBufferDescriptor(bufferDescriptor *bd_p, unsigned char command,
			      unsigned char status, unsigned short count,
			      void *buffAddr, dma_addr_t extBufferAddr);

#endif /* _iapiLow_h */
