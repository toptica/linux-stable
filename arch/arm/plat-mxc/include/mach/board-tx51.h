/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_BOARD_TX51_H__
#define __ASM_ARCH_MXC_BOARD_TX51_H__

/*
 * plat-mxc/include/mach/board-tx51.h
 *
 * This file contains all the board level configuration options.
 *
 * It currently hold the options defined for Ka-Ro TX51 Platform.
 *
 */

/*
 * Include Files
 */

#define UART1_ENABLED		1
/* UART 2 configuration */
#define UART2_MODE		MODE_DCE
#define UART2_IR		NO_IRDA
#define UART2_ENABLED		1
/* UART 3 configuration */
#define UART3_MODE		MODE_DCE
#define UART3_IR		NO_IRDA
#define UART3_ENABLED		1

#endif /* __ASM_ARCH_MXC_BOARD_TX51_H__ */
