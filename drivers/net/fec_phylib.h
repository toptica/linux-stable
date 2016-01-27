/****************************************************************************/

/*
 *	fec.h  --  Fast Ethernet Controller for Motorola ColdFire SoC
 *		   processors.
 *
 *	(C) Copyright 2000-2005, Greg Ungerer (gerg@snapgear.com)
 *	(C) Copyright 2000-2001, Lineo (www.lineo.com)
 */

/****************************************************************************/
#ifndef FEC_H
#define	FEC_H
/****************************************************************************/

/*
 * dummy value to write into RDAR,TDAR. FEC hardware will scan the TX/RX
 * descriptors in memory upon any write access to those registers.
 * The actual value written to those registers does not matter.
*/
#define DONT_CARE		0
#define RDAR_BUSY		(1 << 24)
#define TDAR_BUSY		(1 << 24)

#if defined(CONFIG_M523x) || defined(CONFIG_M527x) || defined(CONFIG_M528x) || \
    defined(CONFIG_M520x) || defined(CONFIG_M532x) || defined(CONFIG_ARCH_MXC)
/*
 *	Just figures, Motorola would have to change the offsets for
 *	registers in the same peripheral device on different models
 *	of the ColdFire!
 */
#define FEC_EIR			0x004
#define FEC_EIMR		0x008
#define	FEC_RDAR		0x010
#define	FEC_TDAR		0x014
#define	FEC_ECR			0x024
#define	FEC_MMFR		0x040
#define	FEC_MSCR		0x044
#define	FEC_MIBC		0x064
#define	FEC_RCR			0x084
#define	FEC_TCR			0x0c4
#define	FEC_PALR		0x0e4
#define	FEC_PAUR		0x0e8
#define	FEC_OPD			0x0ec
#define	FEC_IAUR		0x118
#define	FEC_IALR		0x11c
#define	FEC_GAUR		0x120
#define	FEC_GALR		0x124
#define	FEC_TFWR		0x144
#define	FEC_FRBR		0x14c
#define	FEC_FRSR		0x150
#define	FEC_ERDSR		0x180
#define	FEC_ETDSR		0x184
#define	FEC_EMRBR		0x188

#define FEC_ECR_RESET		(1 << 0)
#define FEC_ECR_ETHER_EN	(1 << 1)
#else

#define FEC_ECNTRL		0x000 /* Ethernet control reg */
#define FEC_IEVENT		0x004 /* Interrupt even reg */
#define FEC_IMASK		0x008 /* Interrupt mask reg */
#define FEC_IVEC		0x00c /* Interrupt vec status reg */
#define FEC_R_DES_ACTIVE	0x010 /* Receive descriptor reg */
#define FEC_X_DES_ACTIVE	0x014 /* Transmit descriptor reg */
#define FEC_MII_DATA		0x040 /* MII manage frame reg */
#define FEC_MII_SPEED		0x044 /* MII speed control reg */
#define FEC_R_BOUND		0x08c /* FIFO receive bound reg */
#define FEC_R_FSTART		0x090 /* FIFO receive start reg */
#define FEC_X_WMRK		0x0a4 /* FIFO transmit water mark */
#define FEC_X_FSTART		0x0ac /* FIFO transmit start reg */
#define FEC_R_CNTRL		0x104 /* Receive control reg */
#define FEC_MAX_FRM_LEN		0x108 /* Maximum frame length reg */
#define FEC_X_CNTRL		0x144 /* Transmit Control reg */
#define FEC_ADDR_LOW		0x3c0 /* Low 32bits MAC address */
#define FEC_ADDR_HIGH		0x3c4 /* High 16bits MAC address */
#define FEC_GRP_HASH_TABLE_HIGH	0x3c8 /* High 32bits hash table */
#define FEC_GRP_HASH_TABLE_LOW	0x3cc /* Low 32bits hash table */
#define FEC_R_DES_START		0x3d0 /* Receive descriptor ring */
#define FEC_X_DES_START		0x3d4 /* Transmit descriptor ring */
#define FEC_R_BUFF_SIZE		0x3d8 /* Maximum receive buff size */
#define FEC_FIFO_RAM		0x400 /* FIFO RAM buffer */

#endif /* CONFIG_M5272 */


/*
 *	Define the buffer descriptor structure.
 */
/* Please see "Receive Buffer Descriptor Field Definitions" in Specification.
 * It's LE.
 */
#if defined(CONFIG_ARCH_MXC)
typedef struct bufdesc {
	unsigned short	cbd_datlen;		/* Data length */
	unsigned short	cbd_sc;			/* Control and status info */
	dma_addr_t	cbd_bufaddr;		/* Buffer address as seen by FEC Hardware */
} cbd_t;
#else
typedef struct bufdesc {
	unsigned short	cbd_sc;			/* Control and status info */
	unsigned short	cbd_datlen;		/* Data length */
	dma_addr_t	cbd_bufaddr;		/* Buffer address */
} cbd_t;
#endif

/*
 *	The following definitions courtesy of commproc.h, which where
 *	Copyright (c) 1997 Dan Malek (dmalek@jlc.net).
 */
#define BD_SC_EMPTY	((ushort)0x8000)	/* Receive is empty */
#define BD_SC_READY	((ushort)0x8000)	/* Transmit is ready */
#define BD_SC_WRAP	((ushort)0x2000)	/* Last buffer descriptor */
#define BD_SC_INTRPT	((ushort)0x1000)	/* Interrupt on change */
#define BD_SC_CM	((ushort)0x0200)	/* Continous mode */
#define BD_SC_ID	((ushort)0x0100)	/* Rec'd too many idles */
#define BD_SC_P		((ushort)0x0100)	/* xmt preamble */
#define BD_SC_BR	((ushort)0x0020)	/* Break received */
#define BD_SC_FR	((ushort)0x0010)	/* Framing error */
#define BD_SC_PR	((ushort)0x0008)	/* Parity error */
#define BD_SC_OV	((ushort)0x0002)	/* Overrun */
#define BD_SC_CD	((ushort)0x0001)	/* ?? */

/* Buffer descriptor control/status used by Ethernet receive.
*/
#define BD_ENET_RX_EMPTY	((ushort)0x8000)
#define BD_ENET_RX_WRAP		((ushort)0x2000)
#define BD_ENET_RX_INTR		((ushort)0x1000)
#define BD_ENET_RX_LAST		((ushort)0x0800)
#define BD_ENET_RX_FIRST	((ushort)0x0400)
#define BD_ENET_RX_MISS		((ushort)0x0100)
#define BD_ENET_RX_LG		((ushort)0x0020)
#define BD_ENET_RX_NO		((ushort)0x0010)
#define BD_ENET_RX_SH		((ushort)0x0008)
#define BD_ENET_RX_CR		((ushort)0x0004)
#define BD_ENET_RX_OV		((ushort)0x0002)
#define BD_ENET_RX_CL		((ushort)0x0001)
#define BD_ENET_RX_STATS	((ushort)0x013f)	/* All status bits */

/* Buffer descriptor control/status used by Ethernet transmit.
*/
#define BD_ENET_TX_READY	((ushort)0x8000)
#define BD_ENET_TX_PAD		((ushort)0x4000)
#define BD_ENET_TX_WRAP		((ushort)0x2000)
#define BD_ENET_TX_INTR		((ushort)0x1000)
#define BD_ENET_TX_LAST		((ushort)0x0800)
#define BD_ENET_TX_TC		((ushort)0x0400)
#define BD_ENET_TX_DEF		((ushort)0x0200)
#define BD_ENET_TX_HB		((ushort)0x0100)
#define BD_ENET_TX_LC		((ushort)0x0080)
#define BD_ENET_TX_RL		((ushort)0x0040)
#define BD_ENET_TX_RCMASK	((ushort)0x003c)
#define BD_ENET_TX_UN		((ushort)0x0002)
#define BD_ENET_TX_CSL		((ushort)0x0001)
#define BD_ENET_TX_STATS	((ushort)0x03ff)	/* All status bits */


#define RCR_LOOP		(1 << 0)
#define RCR_DRT			(1 << 1)
#define RCR_MII_MODE		(1 << 2)
#define RCR_PROM		(1 << 3)
#define RCR_BC_REJ		(1 << 4)
#define RCR_FCE			(1 << 5)
#define RCR_MAX_FL_SHIFT	16
#define RCR_MAX_FL_MASK		(0x7ff << (RCR_MAX_FL_SHIFT))
#define RCR_MAX_FL_set(n)	(((n) << (RCR_MAX_FL_SHIFT)) & (RCR_MAX_FL_MASK))
#define RCR_MAX_FL_get(n)	(((n) & (RCR_MAX_FL_MASK)) >> (RCR_MAX_FL_SHIFT))

#define TCR_GTS			(1 << 0)
#define TCR_HBC			(1 << 1)
#define TCR_FDEN		(1 << 2)
#define TCR_TFCPAUSE		(1 << 3)
#define TCR_RFCPAUSE		(1 << 4)

/****************************************************************************/
#endif /* FEC_H */
