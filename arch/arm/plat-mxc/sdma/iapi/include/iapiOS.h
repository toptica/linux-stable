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
 * File: iapiOS.h
 *
 * $Id iapiOS.h $
 *
 * Description:
 *  prototypes for OS level function of I.API
 *
 *
 *
 *
 * $Log iapiOS.h
 *
 * ***************************************************************************/

#ifndef _iapiOS_h
#define _iapiOS_h

/* ****************************************************************************
 * Boolean identifiers
 *****************************************************************************/
#define NO_OS     0
#define LINUX     1
#define SYMBIAN   2
#define WINCE     3

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#if OS == NO_OS
#include <stdlib.h>
#elif OS == LINUX
#include <linux/types.h>
#endif

#include "sdmaStruct.h"
#include "iapiDefaults.h"
#ifdef MCU
#include "iapiLowMcu.h"
#endif /* MCU */

/* ****************************************************************************
 * Macro-command Section
 *****************************************************************************/
#ifdef CONFIG_SDMA_IRAM
#define IRAM_MALLOC(x) (*iapi_iram_Malloc)(x)
#else
#define IRAM_MALLOC(x) (*iapi_Malloc)(x)
#endif
#define MALLOC(x) (*iapi_Malloc)(x)
#define FREE(x)   if ((x) != NULL) (*iapi_Free)(x)

#define GOTO_SLEEP(x) (iapi_GotoSleep)(x)
#define INIT_SLEEP(x) (iapi_InitSleep)(x)

/* ****************************************************************************
 * Public Function Prototype Section
 *****************************************************************************/

#ifdef CONFIG_SDMA_IRAM
extern void *(*iapi_iram_Malloc)(size_t size);
#endif
extern void *(*iapi_Malloc)(size_t size);
extern void (*iapi_Free)(void *ptr);

extern dma_addr_t (*iapi_Virt2Phys)(void *ptr);
extern void *(*iapi_Phys2Virt)(dma_addr_t);

extern void (*iapi_WakeUp)(int);
extern void (*iapi_GotoSleep)(int);
extern void (*iapi_InitSleep)(int);

extern void *(*iapi_memcpy)(void *dest, const void *src, size_t count);
extern void *(*iapi_memset)(void *dest, int c, size_t count);

extern void (*iapi_EnableInterrupts)(void);
extern void (*iapi_DisableInterrupts)(void);

extern int (*iapi_GetChannel)(int);
extern int (*iapi_ReleaseChannel)(int);

#endif /* _iapiOS_h */
