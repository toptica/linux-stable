/*
 *  Freescale i.MX25 Touch Screen Driver
 *
 *  Copyright (c) 2009 Lothar Wassmann <LW@KARO-electronics.de>
 *
 * Based on code from Freescale BSP
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/* TSC General Config Register */
#define TGCR			0x000
#define TGCR_IPG_CLK_EN		(1 << 0)
#define TGCR_TSC_RST		(1 << 1)
#define TGCR_FUNC_RST		(1 << 2)
#define TGCR_SLPC		(1 << 4)
#define TGCR_STLC		(1 << 5)
#define TGCR_HSYNC_EN		(1 << 6)
#define TGCR_HSYNC_POL		(1 << 7)
#define TGCR_POWERMODE_SHIFT	8
#define TGCR_POWER_OFF		(0x0 << TGCR_POWERMODE_SHIFT)
#define TGCR_POWER_SAVE		(0x1 << TGCR_POWERMODE_SHIFT)
#define TGCR_POWER_ON		(0x3 << TGCR_POWERMODE_SHIFT)
#define TGCR_POWER_MASK		(0x3 << TGCR_POWERMODE_SHIFT)
#define TGCR_INTREFEN		(1 << 10)
#define TGCR_ADCCLKCFG_SHIFT	16
#define TGCR_ADCCLKCFG_MASK	(0x1f << TGCR_ADCCLKCFG_SHIFT)
#define TGCR_PD_EN		(1 << 23)
#define TGCR_PDB_EN		(1 << 24)
#define TGCR_PDBTIME_SHIFT	25
#define TGCR_PDBTIME128		(0x3f << TGCR_PDBTIME_SHIFT)
#define TGCR_PDBTIME_MASK	(0x7f << TGCR_PDBTIME_SHIFT)

/* TSC General Status Register */
#define TGSR			0x004
#define TGSR_TCQ_INT		(1 << 0)
#define TGSR_GCQ_INT		(1 << 1)
#define TGSR_SLP_INT		(1 << 2)
#define TGSR_TCQ_DMA		(1 << 16)
#define TGSR_GCQ_DMA		(1 << 17)

/* TSC IDLE Config Register */
#define TICR			0x008

/* queue dependent register offsets wrt *_REG_BASE */
#define CQFIFO			0x00
#define CQCR			0x04
#define CQSR			0x08
#define CQMR			0x0c
#define CQ_ITEM_7_0		0x20
#define CQ_ITEM_15_8		0x24

/* TouchScreen Convert Queue FIFO Register */
#define TCQ_REG_BASE		0x400
#define TCQFIFO			(TCQ_REG_BASE + CQFIFO)
/* TouchScreen Convert Queue Control Register */
#define TCQCR			(TCQ_REG_BASE + CQCR)
#define CQCR_QSM_SHIFT		0
#define CQCR_QSM_STOP		(0x0 << CQCR_QSM_SHIFT)
#define CQCR_QSM_PEN		(0x1 << CQCR_QSM_SHIFT)
#define CQCR_QSM_FQS		(0x2 << CQCR_QSM_SHIFT)
#define CQCR_QSM_FQS_PEN	(0x3 << CQCR_QSM_SHIFT)
#define CQCR_QSM_MASK		(0x3 << CQCR_QSM_SHIFT)
#define CQCR_FQS		(1 << 2)
#define CQCR_RPT		(1 << 3)
#define CQCR_LAST_ITEM_ID_SHIFT 4
#define CQCR_LAST_ITEM_ID_MASK	(0xf << CQCR_LAST_ITEM_ID_SHIFT)
#define CQCR_FIFOWATERMARK_SHIFT 8
#define CQCR_FIFOWATERMARK_MASK (0xf << CQCR_FIFOWATERMARK_SHIFT)
#define CQCR_REPEATWAIT_SHIFT	12
#define CQCR_REPEATWAIT_MASK	(0xf << CQCR_REPEATWAIT_SHIFT)
#define CQCR_QRST		(1 << 16)
#define CQCR_FRST		(1 << 17)
#define CQCR_PD_MSK		(1 << 18)
#define CQCR_PD_CFG		(1 << 19)

/* TouchScreen Convert Queue Status Register */
#define TCQSR			(TCQ_REG_BASE + CQSR)
#define CQSR_PD			(1 << 0)
#define CQSR_EOQ		(1 << 1)
#define CQSR_FOR		(1 << 4)
#define CQSR_FUR		(1 << 5)
#define CQSR_FER		(1 << 6)
#define CQSR_EMPT		(1 << 13)
#define CQSR_FULL		(1 << 14)
#define CQSR_FDRY		(1 << 15)

/* TouchScreen Convert Queue Mask Register */
#define TCQMR			(TCQ_REG_BASE + CQMR)
#define CQMR_PD_IRQ_MSK		(1 << 0)
#define CQMR_EOQ_IRQ_MSK	(1 << 1)
#define CQMR_FOR_IRQ_MSK	(1 << 4)
#define CQMR_FUR_IRQ_MSK	(1 << 5)
#define CQMR_FER_IRQ_MSK	(1 << 6)
#define CQMR_FDRY_IRQ_MSK	(1 << 15)
#define CQMR_PD_DMA_MSK		(1 << 16)
#define CQMR_EOQ_DMA_MSK	(1 << 17)
#define CQMR_FOR_DMA_MSK	(1 << 20)
#define CQMR_FUR_DMA_MSK	(1 << 21)
#define CQMR_FER_DMA_MSK	(1 << 22)
#define CQMR_FDRY_DMA_MSK	(1 << 31)

/* TouchScreen Convert Queue ITEM 7~0 */
#define TCQ_ITEM_7_0		(TCQ_REG_BASE + CQ_ITEM_7_0)

/* TouchScreen Convert Queue ITEM 15~8 */
#define TCQ_ITEM_15_8		(TCQ_REG_BASE + CQ_ITEM_15_8)

#define TCQ_ITEM_TCC0		0x0
#define TCQ_ITEM_TCC1		0x1
#define TCQ_ITEM_TCC2		0x2
#define TCQ_ITEM_TCC3		0x3
#define TCQ_ITEM_TCC4		0x4
#define TCQ_ITEM_TCC5		0x5
#define TCQ_ITEM_TCC6		0x6
#define TCQ_ITEM_TCC7		0x7
#define TCQ_ITEM_GCC7		0x8
#define TCQ_ITEM_GCC6		0x9
#define TCQ_ITEM_GCC5		0xa
#define TCQ_ITEM_GCC4		0xb
#define TCQ_ITEM_GCC3		0xc
#define TCQ_ITEM_GCC2		0xd
#define TCQ_ITEM_GCC1		0xe
#define TCQ_ITEM_GCC0		0xf

/* TouchScreen Convert Config 0-7 */
#define TCC0			(TCQ_REG_BASE + 0x40)
#define TCC1			(TCQ_REG_BASE + 0x44)
#define TCC2			(TCQ_REG_BASE + 0x48)
#define TCC3			(TCQ_REG_BASE + 0x4c)
#define TCC4			(TCQ_REG_BASE + 0x50)
#define TCC5			(TCQ_REG_BASE + 0x54)
#define TCC6			(TCQ_REG_BASE + 0x58)
#define TCC7			(TCQ_REG_BASE + 0x5c)
#define CC_PEN_IACK		(1 << 1)
#define CC_SEL_REFN_SHIFT	2
#define CC_SEL_REFN_XNUR	(0x0 << CC_SEL_REFN_SHIFT)
#define CC_SEL_REFN_YNLR	(0x1 << CC_SEL_REFN_SHIFT)
#define CC_SEL_REFN_AGND	(0x3 << CC_SEL_REFN_SHIFT)
#define CC_SEL_REFN_MASK	(0x3 << CC_SEL_REFN_SHIFT)
#define CC_SELIN_SHIFT		4
#define CC_SELIN_XPUL		(0x0 << CC_SELIN_SHIFT)
#define CC_SELIN_YPLL		(0x1 << CC_SELIN_SHIFT)
#define CC_SELIN_XNUR		(0x2 << CC_SELIN_SHIFT)
#define CC_SELIN_YNLR		(0x3 << CC_SELIN_SHIFT)
#define CC_SELIN_WIPER		(0x4 << CC_SELIN_SHIFT)
#define CC_SELIN_INAUX0		(0x5 << CC_SELIN_SHIFT)
#define CC_SELIN_INAUX1		(0x6 << CC_SELIN_SHIFT)
#define CC_SELIN_INAUX2		(0x7 << CC_SELIN_SHIFT)
#define CC_SELIN_MASK		(0x7 << CC_SELIN_SHIFT)
#define CC_SEL_REFP_SHIFT	7
#define CC_SEL_REFP_YPLL	(0x0 << CC_SEL_REFP_SHIFT)
#define CC_SEL_REFP_XPUL	(0x1 << CC_SEL_REFP_SHIFT)
#define CC_SEL_REFP_EXT		(0x2 << CC_SEL_REFP_SHIFT)
#define CC_SEL_REFP_INT		(0x3 << CC_SEL_REFP_SHIFT)
#define CC_SEL_REFP_MASK		(0x3 << CC_SEL_REFP_SHIFT)
#define CC_XPULSW		(1 << 9)
#define CC_XNURSW_SHIFT		10
#define CC_XNURSW_HIGH		(0x0 << CC_XNURSW_SHIFT)
#define CC_XNURSW_OFF		(0x1 << CC_XNURSW_SHIFT)
#define CC_XNURSW_LOW		(0x3 << CC_XNURSW_SHIFT)
#define CC_XNURSW_MASK		(0x3 << CC_XNURSW_SHIFT)
#define CC_YPLLSW_SHIFT		12
#define CC_YPLLSW_HIGH		(0x0 << CC_YPLLSW_SHIFT)
#define CC_YPLLSW_OFF		(0x1 << CC_YPLLSW_SHIFT)
#define CC_YPLLSW_LOW		(0x3 << CC_YPLLSW_SHIFT)
#define CC_YPLLSW_MASK		(0x3 << CC_YPLLSW_SHIFT)
#define CC_YNLRSW		(1 << 14)
#define CC_WIPERSW		(1 << 15)
#define CC_NOS_SHIFT		16
#define CC_NOS_MASK		(0xf << CC_NOS_SHIFT)
#define CC_IGS			(1 << 20)
#define CC_SETTLING_TIME_SHIFT	24
#define CC_SETTLING_TIME_MASK	(0xff << CC_SETTLING_TIME_SHIFT)

#define TSC_4WIRE_PRECHARGE	(		\
		CC_SEL_REFN_AGND |		\
		CC_SELIN_XPUL |			\
		CC_SEL_REFP_INT |		\
		CC_XNURSW_OFF |			\
		CC_YPLLSW_OFF |			\
		0)				\
/*0x158c*/
#define TSC_4WIRE_TOUCH_DETECT	(		\
		CC_PEN_IACK |			\
		CC_SEL_REFN_AGND |		\
		CC_SELIN_XPUL |			\
		CC_SEL_REFP_INT |		\
		CC_XPULSW |			\
		CC_XNURSW_OFF |			\
		CC_YPLLSW_OFF |			\
		CC_YNLRSW |			\
		0)				\
/*0x578e*/

#define TSC_4WIRE_X_MEASURE	(		\
		CC_SEL_REFN_XNUR |		\
		CC_SELIN_YPLL |			\
		CC_SEL_REFP_XPUL |		\
		CC_XNURSW_LOW |			\
		CC_YPLLSW_OFF |			\
		0)				\
/*0x1c90*/
#define TSC_4WIRE_Y_MEASURE	(		\
		CC_SEL_REFN_YNLR |		\
		CC_SELIN_XPUL |			\
		CC_SEL_REFP_YPLL |		\
		CC_XPULSW |			\
		CC_XNURSW_OFF |			\
		CC_YNLRSW |			\
		0)				\
/*0x4604*/
#define TSC_4WIRE_XP_MEASURE	(		\
		CC_SEL_REFN_AGND |		\
		CC_SELIN_XPUL |			\
		CC_SEL_REFP_INT |		\
		CC_XPULSW |			\
		CC_YPLLSW_HIGH |		\
		CC_XNURSW_LOW |			\
		0)				\
/*0x0f8c*/
#define TSC_4WIRE_YN_MEASURE	(		\
		CC_SEL_REFN_AGND |		\
		CC_SELIN_YNLR |			\
		CC_SEL_REFP_INT |		\
		CC_XPULSW |			\
		CC_YPLLSW_HIGH |		\
		CC_XNURSW_LOW |			\
		0)				\
/*0x0fbc*/

#define TSC_GENERAL_ADC_GCC0	(		\
		CC_SEL_REFN_AGND |		\
		CC_SELIN_INAUX0 |		\
		CC_SEL_REFP_INT |		\
		CC_XPULSW |			\
		CC_XNURSW_OFF |			\
		CC_YPLLSW_OFF |			\
		0)				\
/*0x17dc*/
#define TSC_GENERAL_ADC_GCC1	(		\
		CC_SEL_REFN_AGND |		\
		CC_SELIN_INAUX0 |		\
		CC_SEL_REFP_INT |		\
		CC_XPULSW |			\
		CC_XNURSW_OFF |			\
		CC_YPLLSW_OFF |			\
		0)				\
/*0x17ec*/
#define TSC_GENERAL_ADC_GCC2	(		\
		CC_SEL_REFN_AGND |		\
		CC_SELIN_INAUX0 |		\
		CC_SEL_REFP_INT |		\
		CC_XPULSW |			\
		CC_XNURSW_OFF |			\
		CC_YPLLSW_OFF |			\
		0)				\
/*0x17fc*/

/* GeneralADC Convert Queue FIFO Register */
#define GCQ_REG_BASE		0x800
#define GCQFIFO			(GCQ_REG_BASE + CQFIFO)
#define GCQFIFO_ADCOUT_SHIFT	4
#define GCQFIFO_ADCOUT_MASK	(0xfff << GCQFIFO_ADCOUT_SHIFT)

/* GeneralADC Convert Queue Control Register */
#define GCQCR			(GCQ_REG_BASE + CQCR)

/* GeneralADC Convert Queue Status Register */
#define GCQSR			(GCQ_REG_BASE + CQSR)

/* GeneralADC Convert Queue Mask Register */
#define GCQMR			(GCQ_REG_BASE + CQMR)

/* GeneralADC Convert Queue ITEM 7~0 */
#define GCQ_ITEM_7_0		(GCQ_REG_BASE + CQ_ITEM_7_0)

/* GeneralADC Convert Queue ITEM 15~8 */
#define GCQ_ITEM_15_8		(GCQ_REG_BASE + CQ_ITEM_15_8)

#define CQ_ITEM7_SHIFT		28
#define CQ_ITEM6_SHIFT		24
#define CQ_ITEM5_SHIFT		20
#define CQ_ITEM4_SHIFT		16
#define CQ_ITEM3_SHIFT		12
#define CQ_ITEM2_SHIFT		 8
#define CQ_ITEM1_SHIFT		 4
#define CQ_ITEM0_SHIFT		 0

#define CQ_ITEM8_SHIFT		28
#define CQ_ITEM9_SHIFT		24
#define CQ_ITEM10_SHIFT		20
#define CQ_ITEM11_SHIFT		16
#define CQ_ITEM12_SHIFT		12
#define CQ_ITEM13_SHIFT		 8
#define CQ_ITEM14_SHIFT		 4
#define CQ_ITEM15_SHIFT		 0

#define GCQ_ITEM_GCC0		0x0
#define GCQ_ITEM_GCC1		0x1
#define GCQ_ITEM_GCC2		0x2
#define GCQ_ITEM_GCC3		0x3

/* GeneralADC Convert Config 0-7 */
#define GCC0			(GCQ_REG_BASE + 0x40)
#define GCC1			(GCQ_REG_BASE + 0x44)
#define GCC2			(GCQ_REG_BASE + 0x48)
#define GCC3			(GCQ_REG_BASE + 0x4c)
#define GCC4			(GCQ_REG_BASE + 0x50)
#define GCC5			(GCQ_REG_BASE + 0x54)
#define GCC6			(GCQ_REG_BASE + 0x58)
#define GCC7			(GCQ_REG_BASE + 0x5c)

/* TSC Test Register R/W */
#define TTR			0xc00
/* TSC Monitor Register 1, 2 */
#define MNT1			0xc04
#define MNT2			0xc04

#define DETECT_ITEM_ID_1	1
#define DETECT_ITEM_ID_2	5
#define TS_X_ITEM_ID		2
#define TS_Y_ITEM_ID		3
#define TSI_DATA		1
#define FQS_DATA		0
