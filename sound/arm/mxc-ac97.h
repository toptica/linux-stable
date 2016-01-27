/*
 * Copyright (C) 2009  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
 */

#define SSI_SACCEN	0x54
#define SSI_SACCDIS	0x58

#define SACC_SLOT(n)	(1 << (9 - ((n) - 3)))
#define SATAG_SLOT(n)	(1 << (15 - (n)))

/*
 * SCR Register bit shift definitions
 */
#define STCCR_WL_SHIFT	13
#define STCCR_WL_MASK	(0x0f << STCCR_WL_SHIFT)
#define STCCR_DC_SHIFT	8
#define STCCR_DC_MASK	(0x1f << STCCR_DC_SHIFT)
#define STCCR_DIV2	(1 << SSI_DIVIDE_BY_TWO_SHIFT)

#define SRCCR_WL_SHIFT	13
#define SRCCR_WL_MASK	(0x0f << SRCCR_WL_SHIFT)
#define SRCCR_DC_SHIFT	8
#define SRCCR_DC_MASK	(0x1f << STCCR_DC_SHIFT)
#define SRCCR_DIV2	(1 << 18)

/*
 * SCR Register bit definitions
 */
#define SCR_CLK_IST	(1 << 9)
#define SCR_SYN		(1 << 4)
#define SCR_RE		(1 << 2)
#define SCR_TE		(1 << 1)
#define SCR_SSIEN	(1 << 0)

/*
 * SSI Option Register (SOR) bit definitions
 */
#define SOR_CLKOFF	(1 << 6)
#define SOR_RX_CLR	(1 << 5)
#define SOR_TX_CLR	(1 << 4)
#define SOR_INIT	(1 << 3)
#define SOR_WAIT_SHIFT	1
#define SOR_WAIT_MASK	(3 << SOR_WAIT_SHIFT)

/*
 * SSI Status Register (SISR) bit definitions
 */
#define SISR_RFRC	(1 << 24) /* receive frame complete flag (on some CPUs) */
#define SISR_TFRC	(1 << 23) /* transmit frame complete flag (on some CPUs) */
#define SISR_CMDAU	(1 << 18)
#define SISR_CMDDU	(1 << 17)
#define SISR_RXT	(1 << 16)
#define SISR_RDR1	(1 << 15)
#define SISR_RDR0	(1 << 14)
#define SISR_TDE1	(1 << 13)
#define SISR_TDE0	(1 << 12)
#define SISR_ROE1	(1 << 11)
#define SISR_ROE0	(1 << 10)
#define SISR_TUE1	(1 << 9)
#define SISR_TUE0	(1 << 8)
#define SISR_TFS	(1 << 7)
#define SISR_RFS	(1 << 6)
#define SISR_TLS	(1 << 5)
#define SISR_RLS	(1 << 4)
#define SISR_RFF1	(1 << 3)
#define SISR_RFF0	(1 << 2)
#define SISR_TFE1	(1 << 1)
#define SISR_TFE0	(1 << 0)

/*
 * SSI Interrupt Enable Register (SIER) bit definitions
 */
#define SIER_RFRC_EN	(1 << 24) /* receive frame complete enable (on some CPUs) */
#define SIER_TFRC_EN	(1 << 23) /* transmit frame complete enable (on some CPUs) */
#define SIER_RDMAE	(1 << 22)
#define SIER_RIE	(1 << 21)
#define SIER_TDMAE	(1 << 20)
#define SIER_TIE	(1 << 19)
#define SIER_CMDAU_EN	(1 << 18)
#define SIER_CMDDU_EN	(1 << 17)
#define SIER_RXT_EN	(1 << 16)
#define SIER_RDR1_EN	(1 << 15)
#define SIER_RDR0_EN	(1 << 14)
#define SIER_TDE1_EN	(1 << 13)
#define SIER_TDE0_EN	(1 << 12)
#define SIER_ROE1_EN	(1 << 11)
#define SIER_ROE0_EN	(1 << 10)
#define SIER_TUE1_EN	(1 << 9)
#define SIER_TUE0_EN	(1 << 8)
#define SIER_TFS_EN	(1 << 7)
#define SIER_RFS_EN	(1 << 6)
#define SIER_TLS_EN	(1 << 5)
#define SIER_RLS_EN	(1 << 4)
#define SIER_RFF1_EN	(1 << 3)
#define SIER_RFF0_EN	(1 << 2)
#define SIER_TFE1_EN	(1 << 1)
#define SIER_TFE0_EN	(1 << 0)

/*
 * SSI AC97 Control Register (SACNT) bit definitions
 */
#define SACNT_FRDIV_SHIFT	5
#define SACNT_FRDIV_BITS	6
#define SACNT_FRDIV_MASK	(((1 << SACNT_FRDIV_BITS) - 1) << SACNT_FRDIV_SHIFT)
#define SACNT_WR	(1 << 4)
#define SACNT_RD	(1 << 3)
#define SACNT_FV	(1 << 1)
#define SACNT_AC97EN	(1 << 0)

/*
 * SSI Transmit Control Register (STCR) bit definitions
 */
#define STCR_TXBIT0	(1 << 9)
#define STCR_TFEN1	(1 << 8)
#define STCR_TFEN0	(1 << 7)
#define STCR_TSHFD	(1 << 4)
#define STCR_TSCKP	(1 << 3)

/*
 * SSI Receive Control Register (SRCR) bit definitions
 */
#define SRCR_RXEXT	(1 << 10)
#define SRCR_RXBIT0	(1 << 9)
#define SRCR_RFEN1	(1 << 8)
#define SRCR_RFEN0	(1 << 7)
#define SRCR_RSHFD	(1 << 4)
#define SRCR_RSCKP	(1 << 3)
