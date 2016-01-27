/*
 * Fast Ethernet Controller (FEC) driver for Motorola MPC8xx.
 * Copyright (c) 1997 Dan Malek (dmalek@jlc.net)
 *
 * This version of the driver is specific to the FADS implementation,
 * since the board contains control registers external to the processor
 * for the control of the LevelOne LXT970 transceiver.  The MPC860T manual
 * describes connections using the internal parallel port I/O, which
 * is basically all of Port D.
 *
 * Right now, I am very wasteful with the buffers.  I allocate memory
 * pages and then divide them into 2K frame buffers.  This way I know I
 * have buffers large enough to hold one frame within one buffer descriptor.
 * Once I get this working, I will use 64 or 128 byte CPM buffers, which
 * will be much more memory efficient and will easily handle lots of
 * small packets.
 *
 * Much better multiple PHY support by Magnus Damm.
 * Copyright (c) 2000 Ericsson Radio Systems AB.
 *
 * Support for FEC controller of ColdFire processors.
 * Copyright (c) 2001-2005 Greg Ungerer (gerg@snapgear.com)
 *
 * Bug fixes and cleanup by Philippe De Muyter (phdm@macqel.be)
 * Copyright (c) 2004-2006 Macq Electronique SA.
 */
/*
 * Copyright 2006-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/resource.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/fec_enet.h>
#include <linux/phy.h>

#include <asm/irq.h>
#include <asm/io.h>

#define DRV_NAME		"fec"

#ifdef DEBUG
static int debug = 3;
#define dbg_lvl(n)	((n) < debug)
module_param(debug, int, S_IRUGO | S_IWUSR);

#define DBG(lvl, fmt...)	do { if (dbg_lvl(lvl)) printk(KERN_DEBUG fmt); } while (0)
#else
static int debug;
#define dbg_lvl(n)	0
module_param(debug, int, 0);

#define DBG(lvl, fmt...)	do { } while (0)
#endif

#if defined(CONFIG_M523x) || defined(CONFIG_M527x) || \
    defined(CONFIG_M5272) || defined(CONFIG_M528x) || \
    defined(CONFIG_M520x) || defined(CONFIG_M532x)
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include "fec_phylib.h"
#define FEC_ALIGNMENT  (0x03)	       /* FEC needs 4bytes alignment*/
#elif defined(CONFIG_ARCH_MXC)
#include <mach/hardware.h>
#include "fec_phylib.h"
#define FEC_ALIGNMENT  (0x0F)	       /* FEC needs 128bits(16bytes) alignment*/
#else
#include <asm/8xx_immap.h>
#include <asm/mpc8xx.h>
#include "commproc.h"
#define FEC_ALIGNMENT  (0x03)	       /* FEC needs 4bytes alignment */
#endif

#define FEC_ADDR_ALIGNMENT(x) ((unsigned char *)(((unsigned long)(x) + (FEC_ALIGNMENT)) & (~FEC_ALIGNMENT)))

#define platform_func(p, args...)	((p) ? (p)(args) : 0)

/* The number of Tx and Rx buffers.  These are allocated from the page
 * pool.  The code may assume these are power of two, so it it best
 * to keep them that size.
 * We don't need to allocate pages for the transmitter.  We just use
 * the skbuffer directly.
 */
#define FEC_ENET_RX_PAGES	8
#define FEC_ENET_RX_FRSIZE	2048
#define FEC_ENET_RX_FRPPG	(PAGE_SIZE / FEC_ENET_RX_FRSIZE)
#define RX_RING_SIZE		(FEC_ENET_RX_FRPPG * FEC_ENET_RX_PAGES)
#define FEC_ENET_TX_FRSIZE	2048
#define FEC_ENET_TX_FRPPG	(PAGE_SIZE / FEC_ENET_TX_FRSIZE)
#define TX_RING_SIZE		16			/* Must be power of two */
#define TX_RING_MOD_MASK	(TX_RING_SIZE - 1)	/*   for this to work */

#if (((RX_RING_SIZE + TX_RING_SIZE) * 8) > PAGE_SIZE)
#error "FEC: descriptor ring size constants too large"
#endif
#define CBD_BUF_SIZE		((RX_RING_SIZE + TX_RING_SIZE) * sizeof(cbd_t))

/* Interrupt events/masks.
*/
#define FEC_ENET_HBERR	((uint)0x80000000)	/* Heartbeat error */
#define FEC_ENET_BABR	((uint)0x40000000)	/* Babbling receiver */
#define FEC_ENET_BABT	((uint)0x20000000)	/* Babbling transmitter */
#define FEC_ENET_GRA	((uint)0x10000000)	/* Graceful stop complete */
#define FEC_ENET_TXF	((uint)0x08000000)	/* Full frame transmitted */
#define FEC_ENET_TXB	((uint)0x04000000)	/* A buffer was transmitted */
#define FEC_ENET_RXF	((uint)0x02000000)	/* Full frame received */
#define FEC_ENET_RXB	((uint)0x01000000)	/* A buffer was received */
#define FEC_ENET_MII	((uint)0x00800000)	/* MII interrupt */
#define FEC_ENET_EBERR	((uint)0x00400000)	/* SDMA bus error */

/* MXC arch interrupt bits */
#define FEC_ENET_LC	((uint)0x00200000)	/* Late collision */
#define FEC_ENET_RL	((uint)0x00100000)	/* Collision retry limit exceeded */
#define FEC_ENET_UN	((uint)0x00080000)	/* TX Fifo underrun */

#ifndef CONFIG_ARCH_MXC
#define FEC_ENET_MASK   ((uint)0xffc00000)
#else
#define FEC_ENET_MASK   ((uint)0xfff80000)
#endif

/* The FEC stores dest/src/type, data, and checksum for receive packets.
 */
#define PKT_MAXBUF_SIZE		1518
#define PKT_MINBUF_SIZE		64
#define PKT_MAXBLR_SIZE		1520


/*
 * The 5270/5271/5280/5282/532x RX control register also contains maximum frame
 * size bits. Other FEC hardware does not, so we need to take that into
 * account when setting it.
 */
#if defined(CONFIG_M523x) || defined(CONFIG_M527x) || defined(CONFIG_M528x) || \
    defined(CONFIG_M520x) || defined(CONFIG_M532x) || defined(CONFIG_ARCH_MXC)
#define	OPT_FRAME_SIZE	(RCR_MAX_FL_set(PKT_MAXBUF_SIZE))
#else
#define	OPT_FRAME_SIZE	0
#endif

/* The FEC buffer descriptors track the ring buffers.  The rx_bd_base and
 * tx_bd_base always point to the base of the buffer descriptors.  The
 * cur_rx and cur_tx point to the currently available buffer.
 * The dirty_tx tracks the current buffer that is being sent by the
 * controller.  The cur_tx and dirty_tx are equal under both completely
 * empty and completely full conditions.  The empty/ready indicator in
 * the buffer descriptor determines the actual condition.
 */
struct fec_enet_private {
	/* Hardware registers of the FEC device */
	void __iomem *reg_base;
	void __iomem *mib_base;
	struct resource *res_mem1;
	struct resource *res_mem2;
	int	etn_irq;
	int	mii_irq;
	struct mii_bus *mii;
	int mii_complete;

	u32 msg_enable;

	/* The saved address of a sent-in-place packet/buffer, for skfree(). */
	void	*tx_bounce[TX_RING_SIZE];
	struct	sk_buff* tx_skbuff[TX_RING_SIZE];
	struct  sk_buff* rx_skbuff[RX_RING_SIZE];
	ushort	skb_cur;
	ushort	skb_dirty;

	/* CPM dual port RAM relative addresses.
	*/
	struct device *dma_dev;		/* pointer to (platform_)device for dma_sync*() functions */
	void	*cbd_mem_base;		/* save the virtual base address of rx&tx buffer descriptor */
	dma_addr_t cbd_phys_base;	/* physical address of buffer descriptor memory for access by FEC HW */

	cbd_t	*rx_bd_base;		/* Address of Rx and Tx buffers. */
	cbd_t	*tx_bd_base;
	cbd_t	*cur_rx, *cur_tx;	/* The next free ring entry */
	cbd_t	*dirty_tx;		/* The ring entries to be free()ed. */
	struct	net_device_stats stats;
	uint	tx_full;
	spinlock_t lock;

	struct	phy_device *phy;
	uint	phy_speed;
	uint	phy_addr;

	unsigned int opened:1;
	unsigned int phy_int_enabled:1;
	unsigned int linkstatus:1;
	unsigned int full_duplex:1;

	struct clk *clk;
};

static int fec_connect_phy(struct net_device *dev, struct fec_enet_private *fep);
static void fec_restart(struct net_device *dev, int duplex);
static void fec_enet_tx(struct net_device *dev);
static void fec_enet_rx(struct net_device *dev);
static void fec_enet_mii(struct net_device *dev);
static void fec_stop(struct net_device *dev);
static void _fec_set_mac_address(struct net_device *dev);

/*
 *  fec_copy_threshold controls the copy when receiving ethernet frame.
 *     If ethernet header is aligned on a 4byte boundary, the ip header and
 *     higher level header will not be aligned.
 *     The reason is, that an ethernet header is 14bytes long.
 *     And the max size of tcp & ip header is 128bytes. Normally it is 40bytes.
 *     So I set the default value between 128 to 256.
 */
static int fec_copy_threshold = 192;

/* MII processing.  We keep this as simple as possible.  Requests are
 * placed on the list (if there is room).  When the request is finished
 * by the MII, an optional function may be called.
 */
typedef struct mii_list {
	uint	mii_regval;
	void	(*mii_func)(uint val, struct net_device *dev);
	struct	mii_list *mii_next;
} mii_list_t;

/* Make MII read/write commands for the FEC.
*/
#define mk_mii_read(REG)	(0x60020000 | ((REG & 0x1f) << 18))
#define mk_mii_write(REG, VAL)	(0x50020000 | ((REG & 0x1f) << 18) | \
						(VAL & 0xffff))
#define mk_mii_end	0

/* Transmitter timeout.
*/
#define TX_TIMEOUT (2 * HZ)

/* Register definitions for the PHY.
*/

#define MII_REG_CR	    0  /* Control Register			   */
#define MII_REG_SR	    1  /* Status Register			   */
#define MII_REG_PHYIR1	    2  /* PHY Identification Register 1		   */
#define MII_REG_PHYIR2	    3  /* PHY Identification Register 2		   */
#define MII_REG_ANAR	    4  /* A-N Advertisement Register		   */
#define MII_REG_ANLPAR	    5  /* A-N Link Partner Ability Register	   */
#define MII_REG_ANER	    6  /* A-N Expansion Register		   */
#define MII_REG_ANNPTR	    7  /* A-N Next Page Transmit Register	   */
#define MII_REG_ANLPRNPR    8  /* A-N Link Partner Received Next Page Reg. */

/* values for phy_status */

#define PHY_CONF_ANE	0x0001  /* 1 auto-negotiation enabled */
#define PHY_CONF_LOOP	0x0002  /* 1 loopback mode enabled */
#define PHY_CONF_SPMASK	0x00f0  /* mask for speed */
#define PHY_CONF_10HDX	0x0010  /* 10 Mbit half duplex supported */
#define PHY_CONF_10FDX	0x0020  /* 10 Mbit full duplex supported */
#define PHY_CONF_100HDX	0x0040  /* 100 Mbit half duplex supported */
#define PHY_CONF_100FDX	0x0080  /* 100 Mbit full duplex supported */

#define PHY_STAT_LINK	0x0100  /* 1 up - 0 down */
#define PHY_STAT_FAULT	0x0200  /* 1 remote fault */
#define PHY_STAT_ANC	0x0400  /* 1 auto-negotiation complete	*/
#define PHY_STAT_SPMASK	0xf000  /* mask for speed */
#define PHY_STAT_10HDX	0x1000  /* 10 Mbit half duplex selected	*/
#define PHY_STAT_10FDX	0x2000  /* 10 Mbit full duplex selected	*/
#define PHY_STAT_100HDX	0x4000  /* 100 Mbit half duplex selected */
#define PHY_STAT_100FDX	0x8000  /* 100 Mbit full duplex selected */

static inline unsigned long fec_reg_read(struct fec_enet_private *fep,
					unsigned int reg)
{
	return readl(fep->reg_base + reg);
}

static inline void fec_reg_write(struct fec_enet_private *fep,
				unsigned int reg, unsigned long val)
{
	writel(val, fep->reg_base + reg);
}

static inline void fec_enet_cbd_get(struct fec_enet_private *fep)
{
	dma_sync_single_for_cpu(fep->dma_dev, fep->cbd_phys_base,
				CBD_BUF_SIZE, DMA_BIDIRECTIONAL);
}

static inline void fec_enet_cbd_put(struct fec_enet_private *fep)
{
	dma_sync_single_for_device(fep->dma_dev, fep->cbd_phys_base,
				   CBD_BUF_SIZE, DMA_BIDIRECTIONAL);
}

static inline void fec_enet_rxbuf_get(struct fec_enet_private *fep,
				cbd_t *bdp, ushort len)
{
#ifdef USE_DMASYNC
	dma_sync_single_for_cpu(fep->dma_dev, bdp->cbd_bufaddr,
				len, DMA_FROM_DEVICE);
#endif
}

static inline void fec_enet_rxbuf_put(struct fec_enet_private *fep,
				cbd_t *bdp, ushort len)
{
#ifdef USE_DMASYNC
	dma_sync_single_for_device(fep->dma_dev, bdp->cbd_bufaddr, len,
				DMA_FROM_DEVICE);
#endif
}

static inline void fec_enet_rxbuf_map(struct fec_enet_private *fep,
				cbd_t *bdp, void *buf, ushort len)
{
	bdp->cbd_bufaddr = dma_map_single(fep->dma_dev, buf,
					  len, DMA_FROM_DEVICE);
}

static inline void fec_enet_rxbuf_unmap(struct fec_enet_private *fep,
					cbd_t *bdp, ushort len)
{
	dma_unmap_single(fep->dma_dev, bdp->cbd_bufaddr,
			 len, DMA_FROM_DEVICE);
	bdp->cbd_bufaddr = ~0;
}

static inline void fec_enet_txbuf_map(struct fec_enet_private *fep,
				cbd_t *bdp, void *buf, ushort len)
{
	bdp->cbd_bufaddr = dma_map_single(fep->dma_dev, buf,
					  len, DMA_TO_DEVICE);
}

static inline void fec_enet_txbuf_unmap(struct fec_enet_private *fep,
					cbd_t *bdp, ushort len)
{
	dma_unmap_single(fep->dma_dev, bdp->cbd_bufaddr,
			 len, DMA_TO_DEVICE);
	bdp->cbd_bufaddr = ~0;
}

static inline void fec_enet_txbuf_get(struct fec_enet_private *fep,
				cbd_t *bdp, ushort len)
{
#ifdef USE_DMASYNC
	dma_sync_single_for_cpu(fep->dma_dev, bdp->cbd_bufaddr,
				len, DMA_TO_DEVICE);
#endif
}

static inline void fec_enet_txbuf_put(struct fec_enet_private *fep,
				cbd_t *bdp, ushort len)
{
#ifdef USE_DMASYNC
	dma_sync_single_for_device(fep->dma_dev, bdp->cbd_bufaddr,
				   len, DMA_TO_DEVICE);
#endif
}

static int
fec_enet_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	cbd_t *bdp;
	unsigned short status;
	unsigned long flags;

	if (!fep->linkstatus) {
		/* Link is down or autonegotiation is in progress. */
		return NETDEV_TX_BUSY;
	}

	spin_lock_irqsave(&fep->lock, flags);

	fec_enet_cbd_get(fep);

	/* Fill in a Tx ring entry */
	bdp = fep->cur_tx;

	status = bdp->cbd_sc;
#ifdef DEBUG
	if (status & BD_ENET_TX_READY) {
		/* Ooops.  All transmit buffers are full.  Bail out.
		 * This should not happen, since dev->tbusy should be set.
		 */
		printk("%s: tx queue full!.\n", dev->name);
		fec_enet_cbd_put(fep);
		spin_unlock_irqrestore(&fep->lock, flags);
		return NETDEV_TX_BUSY;
	}
#endif
	/* Clear all of the status flags.
	 */
	status &= ~BD_ENET_TX_STATS;

	/* Set buffer length and buffer pointer.
	*/
	bdp->cbd_datlen = skb->len;

	/*
	 *	On some FEC implementations data must be aligned on
	 *	4-byte boundaries. Use bounce buffers to copy data
	 *	and get it aligned. Ugh.
	 */
	if (unlikely((bdp->cbd_bufaddr) & FEC_ALIGNMENT)) {
		unsigned int index;
		index = bdp - fep->tx_bd_base;
		memcpy(fep->tx_bounce[index], skb->data, skb->len);
		fec_enet_txbuf_map(fep, bdp, fep->tx_bounce[index], skb->len);
	} else {
		fec_enet_txbuf_map(fep, bdp, skb->data, skb->len);
	}

	/* Save skb pointer.
	*/
	fep->tx_skbuff[fep->skb_cur] = skb;

	fep->stats.tx_bytes += skb->len;
	fep->skb_cur = (fep->skb_cur + 1) & TX_RING_MOD_MASK;

	/* Send it on its way.  Tell FEC it's ready, interrupt when done,
	 * it's the last BD of the frame, and to put the CRC on the end.
	 */
	status |= (BD_ENET_TX_READY | BD_ENET_TX_INTR |
		BD_ENET_TX_LAST | BD_ENET_TX_TC);
	bdp->cbd_sc = status;

	dev->trans_start = jiffies;

	/* If this was the last BD in the ring, start at the beginning again.
	*/
	if (status & BD_ENET_TX_WRAP) {
		bdp = fep->tx_bd_base;
	} else {
		bdp++;
	}

	if (bdp == fep->dirty_tx) {
		fep->tx_full = 1;
		netif_stop_queue(dev);
	}

	fep->cur_tx = bdp;
	fec_enet_cbd_put(fep);

	/* Trigger transmission start */
	fec_reg_write(fep, FEC_TDAR, DONT_CARE);

	spin_unlock_irqrestore(&fep->lock, flags);

	return NETDEV_TX_OK;
}

static void
fec_timeout(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);

	dev_warn(&dev->dev, "transmit timed out\n");
	fep->stats.tx_errors++;
#ifdef DEBUG
	{
		int i;
		cbd_t *bdp;

		fec_enet_cbd_get(fep);

		printk(KERN_DEBUG "%s: Ring data dump: cur_tx %p%s, dirty_tx %p cur_rx: %p\n",
		       __FUNCTION__,
		       fep->cur_tx, fep->tx_full ? " (full)" : "",
		       fep->dirty_tx,
		       fep->cur_rx);

		bdp = fep->tx_bd_base;
		printk(" tx: %u buffers\n", TX_RING_SIZE);
		for (i = 0; i < TX_RING_SIZE; i++) {
			printk("  %p: %04x %04x %08x\n",
			       bdp,
			       bdp->cbd_sc,
			       bdp->cbd_datlen,
			       bdp->cbd_bufaddr);
			bdp++;
		}

		bdp = fep->rx_bd_base;
		printk(" rx: %lu buffers\n", RX_RING_SIZE);
		for (i = 0; i < RX_RING_SIZE; i++) {
			printk("  %p: %04x %04x %08x\n",
			       bdp,
			       bdp->cbd_sc,
			       bdp->cbd_datlen,
			       bdp->cbd_bufaddr);
			bdp++;
		}
		fec_enet_cbd_put(fep);
	}
#endif
	fec_restart(dev, fep->full_duplex);
	//netif_schedule(dev);
}

/* The interrupt handler.
 * This is called from the MPC core interrupt.
 */
static irqreturn_t
fec_enet_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct fec_enet_private *fep = netdev_priv(dev);
	uint int_events;
	int handled = 0;

	/* Get the interrupt events that caused us to be here.
	*/
	while ((int_events = fec_reg_read(fep, FEC_EIR) & FEC_ENET_MASK) != 0) {
		fec_reg_write(fep, FEC_EIR, int_events);

		/* Handle receive event in its own function.
		 */
		if (int_events & (FEC_ENET_RXF | FEC_ENET_RXB)) {
			handled = 1;
			fec_enet_rx(dev);
		}

		if (int_events & FEC_ENET_UN) {
			printk(KERN_WARNING "TX fifo underrun");
		}
		/* Transmit OK, or non-fatal error. Update the buffer
		   descriptors. FEC handles all errors, we just discover
		   them as part of the transmit process.
		*/
		if (int_events & (FEC_ENET_TXF | FEC_ENET_TXB)) {
			handled = 1;
			fec_enet_tx(dev);
		}

		if (int_events & (FEC_ENET_MII | FEC_ENET_HBERR)) {
			handled = 1;
			fec_enet_mii(dev);
		}
	}
	return IRQ_RETVAL(handled);
}

static void fec_free_skb(struct fec_enet_private *fep, cbd_t *bdp,
			struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	if (!dma_mapping_error(fep->dma_dev, bdp->cbd_bufaddr)) {
		fec_enet_txbuf_unmap(fep, bdp, skb->len);
	}
	dev_kfree_skb_any(skb);
	*pskb = NULL;
}

static void
fec_enet_tx(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	cbd_t *bdp;
	unsigned short status;
	struct sk_buff *skb;

	spin_lock(&fep->lock);

	fec_enet_cbd_get(fep);
	bdp = fep->dirty_tx;

	while (((status = bdp->cbd_sc) & BD_ENET_TX_READY) == 0) {
		if (bdp == fep->cur_tx && fep->tx_full == 0) break;

		skb = fep->tx_skbuff[fep->skb_dirty];
		/* Check for errors. */
		if (status & (BD_ENET_TX_HB | BD_ENET_TX_LC |
				   BD_ENET_TX_RL | BD_ENET_TX_UN |
				   BD_ENET_TX_CSL)) {
			fep->stats.tx_errors++;
			if (status & BD_ENET_TX_HB)  /* No heartbeat */
				fep->stats.tx_heartbeat_errors++;
			if (status & BD_ENET_TX_LC)  /* Late collision */
				fep->stats.tx_window_errors++;
			if (status & BD_ENET_TX_RL)  /* Retrans limit */
				fep->stats.tx_aborted_errors++;
			if (status & BD_ENET_TX_UN)  /* Underrun */
				fep->stats.tx_fifo_errors++;
			if (status & BD_ENET_TX_CSL) /* Carrier lost */
				fep->stats.tx_carrier_errors++;
		} else {
			fep->stats.tx_packets++;
		}

#ifdef DEBUG
		if (status & BD_ENET_TX_READY)
			printk("HEY! Enet xmit interrupt and TX_READY.\n");
#endif
		/* Deferred means some collisions occurred during transmit,
		 * but we eventually sent the packet OK.
		 */
		if (status & BD_ENET_TX_DEF)
			fep->stats.collisions++;

		/* Free the sk buffer associated with this last transmit.
		 */
		fec_free_skb(fep, bdp, &fep->tx_skbuff[fep->skb_dirty]);
		fep->skb_dirty = (fep->skb_dirty + 1) & TX_RING_MOD_MASK;

		/* Update pointer to next buffer descriptor to be transmitted.
		 */
		if (status & BD_ENET_TX_WRAP)
			bdp = fep->tx_bd_base;
		else
			bdp++;

		/* Since we have freed up a buffer, the ring is no longer
		 * full.
		 */
		if (fep->tx_full) {
			fep->tx_full = 0;
			if (netif_queue_stopped(dev)) {
				netif_wake_queue(dev);
			}
		}
	}
	fec_enet_cbd_put(fep);
	fep->dirty_tx = bdp;
	spin_unlock(&fep->lock);
}


/* During a receive, the cur_rx points to the current incoming buffer.
 * When we update through the ring, if the next incoming buffer has
 * not been given to the system, we just set the empty indicator,
 * effectively tossing the packet.
 */
static void
fec_enet_rx(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	cbd_t *bdp;
	unsigned short status;
	struct sk_buff *skb;
	ushort pkt_len;
	int rx_index;

#ifdef CONFIG_M532x
	/* This is probably nonsense
	   Proper use of dma-mapping functions should make this obsolete
	*/
	flush_cache_all();
#endif
	/* reserve the dual port memory area for our use */
	fec_enet_cbd_get(fep);

	/* First, grab all of the stats for the incoming packet.
	 * These get messed up if we get called due to a busy condition.
	 */
	bdp = fep->cur_rx;

while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) {
	rx_index = bdp - fep->rx_bd_base;
#ifdef DEBUG
	/* Since we have allocated space to hold a complete frame,
	 * the last indicator should be set.
	 */
	WARN_ON(!(status & BD_ENET_RX_LAST));
#endif

	if (!fep->opened) {
		goto rx_processing_done;
	}
	/* Check for errors. */
	if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO |
			   BD_ENET_RX_CR | BD_ENET_RX_OV)) {
		fep->stats.rx_errors++;
		if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH)) {
		/* Frame too long or too short. */
			fep->stats.rx_length_errors++;
		}
		if (status & BD_ENET_RX_NO)	/* Frame alignment */
			fep->stats.rx_frame_errors++;
		if (status & BD_ENET_RX_CR)	/* CRC Error */
			fep->stats.rx_crc_errors++;
		if (status & BD_ENET_RX_OV)	/* FIFO overrun */
			fep->stats.rx_fifo_errors++;
	}

	/* Report late collisions as a frame error.
	 * On this error, the BD is closed, but we don't know what we
	 * have in the buffer.  So, just drop this frame on the floor.
	 */
	if (status & BD_ENET_RX_CL) {
		fep->stats.rx_errors++;
		fep->stats.rx_frame_errors++;
		dev_warn(&dev->dev, "%s: Collision detected; dropping packet\n",
			__FUNCTION__);
		goto rx_processing_done;
	}

	/* Process the incoming frame.
	 */
	fep->stats.rx_packets++;
	pkt_len = bdp->cbd_datlen;
	fep->stats.rx_bytes += pkt_len;

	/* This does 16 byte alignment, exactly what we need.
	 * The packet length includes FCS, but we don't want to
	 * include that when passing upstream as it messes up
	 * bridging applications.
	 */
	if ((pkt_len - 4) < fec_copy_threshold) {
		skb = dev_alloc_skb(pkt_len);
	} else {
		skb = dev_alloc_skb(FEC_ENET_RX_FRSIZE);
	}

	if (skb == NULL) {
		printk("%s: Memory squeeze, dropping packet.\n", dev->name);
		fep->stats.rx_dropped++;
	} else {
		if ((pkt_len - 4) < fec_copy_threshold) {
			/* skip 2 bytes, so IP header is on a 4 bytes boundary */
			skb_reserve(skb, 2);
			skb_put(skb, pkt_len - 4); /* Make room */
			fec_enet_rxbuf_get(fep, bdp, pkt_len - 4);
			skb_copy_to_linear_data(skb,
						fep->rx_skbuff[rx_index]->data,
						pkt_len - 4);
			fec_enet_rxbuf_put(fep, bdp, pkt_len - 4);
		} else {
			struct sk_buff *pskb = fep->rx_skbuff[rx_index];

			/* unmap the skb we are going to hand down to the network layer */
			fec_enet_rxbuf_unmap(fep, bdp, FEC_ENET_RX_FRSIZE);

			/* init the newly allocated skb */
			fep->rx_skbuff[rx_index] = skb;
			skb->data = FEC_ADDR_ALIGNMENT(skb->data);
			/* map the newly allocated skb's data buffer for DMA */
			fec_enet_rxbuf_map(fep, bdp, skb->data,
					FEC_ENET_RX_FRSIZE);

			skb_put(pskb, pkt_len - 4);	/* Make room */
			skb = pskb;
		}
		skb->dev = dev;
		skb->protocol = eth_type_trans(skb, dev);
		netif_rx(skb);
	}
  rx_processing_done:
	/* Clear the status flags for this buffer.
	*/
	status &= ~BD_ENET_RX_STATS;

	/* Mark the buffer empty.
	*/
	status |= BD_ENET_RX_EMPTY;
	bdp->cbd_sc = status;

	/* release the dual port memory area for use by the FEC hardware */
	fec_enet_cbd_put(fep);

	/* Update BD pointer to next entry.
	*/
	if (status & BD_ENET_RX_WRAP)
		bdp = fep->rx_bd_base;
	else
		bdp++;

#if 1
	/* Doing this here will keep the FEC running while we process
	 * incoming frames.  On a heavily loaded network, we should be
	 * able to keep up at the expense of system resources.
	 */
	fec_reg_write(fep, FEC_RDAR, DONT_CARE);
#endif
   } /* while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) */
	fep->cur_rx = bdp;

#if 0
	/* Doing this here will allow us to process all frames in the
	 * ring before the FEC is allowed to put more there.  On a heavily
	 * loaded network, some frames may be lost.  Unfortunately, this
	 * increases the interrupt overhead since we can potentially work
	 * our way back to the interrupt return only to come right back
	 * here.
	 */
	fec_reg_write(fep, FEC_RDAR, DONT_CARE);
#endif
}

/* called from interrupt context */
static void fec_enet_mii(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	fep->mii_complete = 1;
}

/*
 * do some initializtion based architecture of this chip
 * MOVED to platform_data hooks!
 */
static void fec_link_change(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	struct phy_device *phydev = fep->phy;

	if (phydev->link != fep->linkstatus ||
	    phydev->duplex != fep->full_duplex) {
		DBG(0, "%s: link status changed from %d to %d %s -> %s duplex\n", __FUNCTION__,
		    fep->linkstatus, phydev->link, fep->full_duplex ? "full" : "half",
		    phydev->duplex ? "full" : "half");
		if (phydev->link) {
			fec_restart(dev, phydev->duplex);
		} else {
			fec_stop(dev);
		}
		if (fep->linkstatus != phydev->link && netif_msg_link(fep)) {
			phy_print_status(phydev);
		}
		fep->linkstatus = phydev->link;
	}
}

/*
 * Code specific to Freescale i.MXC
 */
static int fec_request_intrs(struct platform_device *pdev, struct net_device *dev)
{
	int ret;
	struct fec_enet_private *fep = netdev_priv(dev);

	fep->etn_irq = platform_get_irq(pdev, 0);
	fep->mii_irq = platform_get_irq(pdev, 1);

	/* Setup interrupt handlers. */
	ret = request_irq(fep->etn_irq, fec_enet_interrupt, 0, "fec", dev);
	if (ret != 0) {
		printk(KERN_ERR "FEC: Could not allocate FEC IRQ(%d)!\n", fep->etn_irq);
		return ret;
	}
	return 0;
}

static void fec_release_intrs(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);

	free_irq(fep->etn_irq, dev);
}

#ifdef CONFIG_ARCH_MX25
/*
 * i.MX25 allows RMII mode to be configured via a gasket
 */
#define FEC_MIIGSK_CFGR			0x300
#define FEC_MIIGSK_ENR			0x308

#define FEC_MIIGSK_CFGR_FRCONT		(1 << 6)
#define FEC_MIIGSK_CFGR_LBMODE		(1 << 4)
#define FEC_MIIGSK_CFGR_EMODE		(1 << 3)
#define FEC_MIIGSK_CFGR_IF_MODE_MASK	(3 << 0)
#define FEC_MIIGSK_CFGR_IF_MODE_MII	(0 << 0)
#define FEC_MIIGSK_CFGR_IF_MODE_RMII	(1 << 0)

#define FEC_MIIGSK_ENR_READY		(1 << 2)
#define FEC_MIIGSK_ENR_EN		(1 << 1)

static inline unsigned long fec_reg_read16(struct fec_enet_private *fep, unsigned int reg)
{
	return readw(fep->reg_base + reg);
}

static inline void fec_reg_write16(struct fec_enet_private *fep, unsigned int reg, unsigned long val)
{
	writew(val, fep->reg_base + reg);
}

static int fec_localhw_setup(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	int loops = 0;
	const int max_loops = 10000;
	u16 enr;

	/*
	 * Set up the MII gasket for RMII mode
	 */
	dev_dbg(fep->dma_dev, "enable RMII gasket\n");

	/* disable the gasket and wait */
	fec_reg_write16(fep, FEC_MIIGSK_ENR, 0);
	while (fec_reg_read16(fep, FEC_MIIGSK_ENR) & FEC_MIIGSK_ENR_READY) {
		if (loops++ >= max_loops)
			return -ETIME;
		udelay(1);
	}

	/* configure the gasket for RMII, 50 MHz, no loopback, no echo */
	fec_reg_write16(fep, FEC_MIIGSK_CFGR, FEC_MIIGSK_CFGR_IF_MODE_RMII);

	/* re-enable the gasket */
	fec_reg_write16(fep, FEC_MIIGSK_ENR, FEC_MIIGSK_ENR_EN);
	fec_reg_read16(fep, FEC_MIIGSK_CFGR);
	fec_reg_read16(fep, FEC_MIIGSK_ENR);

	for (loops = 0; loops < max_loops; loops++) {
		enr = readw(fep->reg_base + FEC_MIIGSK_ENR);
		if (enr & FEC_MIIGSK_ENR_READY) {
			break;
		}
		udelay(1);
	}
	if (enr & FEC_MIIGSK_ENR_READY) {
		dev_dbg(fep->dma_dev, "RMII gasket ready after %u loops\n",
			loops);
		return 0;
	}
	dev_err(fep->dma_dev, "RMII gasket NOT ready\n");
	return -ETIME;
}
#else
static inline int fec_localhw_setup(struct net_device *dev)
{
	return 0;
}
#endif

static int fec_set_mii(struct net_device *dev, struct fec_enet_private *fep)
{
	unsigned long rate;
	struct clk *clk;

	fec_reg_write(fep, FEC_RCR, OPT_FRAME_SIZE | RCR_MII_MODE);
	fec_reg_write(fep, FEC_TCR, 0x00);

	/*
	 * Set MII speed to 2.5 MHz
	 */
	clk = clk_get(fep->dma_dev, "fec");
	if (!IS_ERR(clk)) {
		rate = clk_get_rate(clk);
		clk_put(clk);
	} else {
		printk(KERN_ERR "Failed to get fec clock: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}
#if 1
	fep->phy_speed = ((((rate + 4999999) / 2500000) / 2) & 0x3F) << 1;
#else
	fep->phy_speed = ((((rate + 2499999) / 2500000) / 2) & 0x3F) << 1;
#endif
	fec_reg_write(fep, FEC_MSCR, fep->phy_speed);
	dev_dbg(fep->dma_dev, "%s: clkdiv set to %u for MII clock %u at base clock %lu\n",
		__FUNCTION__, fep->phy_speed >> 1, 2500000, rate);
	dev_dbg(fep->dma_dev, "%s: actual MII clock is: %lu\n", __FUNCTION__, rate / (fep->phy_speed));

	return 0;
}

static void fec_get_mac(struct platform_device *pdev, struct net_device *dev)
{
	// keep bootloader assigned MAC address
	struct fec_enet_platform_data *pdata = pdev->dev.platform_data;
	struct fec_enet_private *fep = netdev_priv(dev);
	unsigned long eth_addr = fec_reg_read(fep, FEC_PALR);

	dev->dev_addr[0] = eth_addr >> 24;
	dev->dev_addr[1] = eth_addr >> 16;
	dev->dev_addr[2] = eth_addr >> 8;
	dev->dev_addr[3] = eth_addr >> 0;
	eth_addr = fec_reg_read(fep, FEC_PAUR);
	dev->dev_addr[5] = eth_addr >> 16;
	dev->dev_addr[4] = eth_addr >> 24;

	if (!is_valid_ether_addr(dev->dev_addr)) {
		/* let platform code set up the MAC address */
		platform_func(pdata->set_mac_addr, pdev, dev->dev_addr);
	}
}

/* ------------------------------------------------------------------------- */

static int
fec_enet_open(struct net_device *dev)
{
	int ret = 0;
	struct fec_enet_private *fep = netdev_priv(dev);

	/* I should reset the ring buffers here, but I don't yet know
	 * a simple way to do that.
	 */
	DBG(0, "%s: \n", __FUNCTION__);
	_fec_set_mac_address(dev);

	fec_restart(dev, 0);

	ret = fec_connect_phy(dev, fep);
	if (ret != 0) {
		DBG(0, "%s: Failed to connect to PHY: %d\n", __FUNCTION__, ret);
		return ret;
	}
	phy_start(fep->phy);

	fep->linkstatus = fep->phy->link;
	//fec_restart(dev, 0);
	DBG(0, "%s: Link status is: %d\n", __FUNCTION__, fep->linkstatus);

	fep->opened = 1;

	/* enable receiver */
	fec_reg_write(fep, FEC_RDAR, DONT_CARE);

	return ret;
}

static int
fec_enet_close(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);

	DBG(0, "%s: \n", __FUNCTION__);

	fep->opened = 0;
	if (fep->linkstatus) {
		fec_stop(dev);
	}
	if (fep->phy) {
		DBG(0, "%s: Stopping PHY %p\n", __FUNCTION__, fep->phy);
		phy_stop(fep->phy);

		DBG(0, "%s: Disconnecting PHY %p\n", __FUNCTION__, fep->phy);
		phy_disconnect(fep->phy);
		fep->phy = NULL;
	}
#if 1
	/* Whack a reset.  We should wait for this.
	*/
	fec_reg_write(fep, FEC_ECR, FEC_ECR_RESET);
	udelay(10);

	/* Mask and clear outstanding MII command interrupts.
	*/
	fec_reg_write(fep, FEC_EIMR, 0);
	fec_reg_write(fep, FEC_EIR, FEC_ENET_MII);
	/* Switch off MII */
	fec_reg_write(fep, FEC_MSCR, 0);
#endif
	return 0;
}

/* Set or clear the multicast filter for this adaptor.
 * Skeleton taken from sunlance driver.
 * The CPM Ethernet implementation allows Multicast as well as individual
 * MAC address filtering.  Some of the drivers check to make sure it is
 * a group multicast address, and discard those that are not.  I guess I
 * will do the same for now, but just remove the test if you want
 * individual filtering as well (do the upper net layers want or support
 * this kind of feature?).
 */

#define HASH_BITS	6		/* #bits in hash */
#define CRC32_POLY	0xEDB88320

static void set_multicast_list(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	struct dev_mc_list *dmi;
	unsigned int i, j, bit, data, crc;
	unsigned char hash;

	if (dev->flags & IFF_PROMISC) {
		fec_reg_write(fep, FEC_RCR, fec_reg_read(fep, FEC_RCR) | RCR_PROM);
	} else {

		fec_reg_write(fep, FEC_RCR, fec_reg_read(fep, FEC_RCR) & ~RCR_PROM);

		if (dev->flags & IFF_ALLMULTI) {
			/* Catch all multicast addresses, so set the
			 * filter to all 1's.
			 */
			fec_reg_write(fep, FEC_IAUR, ~0);
			fec_reg_write(fep, FEC_IALR, ~0);
		} else {
			/* Clear filter and add the addresses in hash register.
			*/
			fec_reg_write(fep, FEC_IAUR, 0);
			fec_reg_write(fep, FEC_IALR, 0);

			dmi = dev->mc_list;

			for (j = 0; j < dev->mc_count; j++, dmi = dmi->next) {
				/* Only support group multicast for now.
				*/
				if (!(dmi->dmi_addr[0] & 1))
					continue;

				/* calculate crc32 value of mac address
				*/
				crc = ~0;

				for (i = 0; i < dmi->dmi_addrlen; i++) {
					data = dmi->dmi_addr[i];
					for (bit = 0; bit < 8; bit++, data >>= 1) {
						crc = (crc >> 1) ^
						(((crc ^ data) & 1) ? CRC32_POLY : 0);
					}
				}

				/* only upper 6 bits (HASH_BITS) are used
				   which point to specific bit in he hash registers
				*/
				hash = (crc >> (32 - HASH_BITS)) & 0x3f;

				if (hash > 31)
					fec_reg_write(fep, FEC_IAUR,
						      fec_reg_read(fep, FEC_IAUR) |
						      (1 << (hash - 32)));
				else
					fec_reg_write(fep, FEC_IALR,
						      fec_reg_read(fep, FEC_IALR) |
						      (1 << hash));
			}
		}
	}
}

/* Set a MAC change in hardware.
 */
static void
_fec_set_mac_address(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);

	/* Set station address. */
	fec_reg_write(fep, FEC_PALR, dev->dev_addr[3] | (dev->dev_addr[2] << 8) |
		(dev->dev_addr[1] << 16) | (dev->dev_addr[0] << 24));
	fec_reg_write(fep, FEC_PAUR, (dev->dev_addr[5] << 16) |
		(dev->dev_addr[4] << 24));
}

static int
fec_set_mac_address(struct net_device *dev, void *_addr)
{
	struct sockaddr *addr = _addr;

	if (!is_valid_ether_addr((const char *)&addr->sa_data)) {
		dev_err(&dev->dev, "Bad ethernet address: %02x:%02x:%02x:%02x:%02x:%02x\n",
		       addr->sa_data[0], addr->sa_data[1], addr->sa_data[2], addr->sa_data[3],
		       addr->sa_data[4], addr->sa_data[5]);
		return -EINVAL;
	}
	dev_dbg(&dev->dev, "Setting MAC address to %02x:%02x:%02x:%02x:%02x:%02x\n",
	       addr->sa_data[0], addr->sa_data[1], addr->sa_data[2], addr->sa_data[3],
	       addr->sa_data[4], addr->sa_data[5]);

	memcpy(&dev->dev_addr, &addr->sa_data, ETH_ALEN);

	_fec_set_mac_address(dev);

	return 0;
}

static void fec_enet_free_buffers(struct fec_enet_private *fep)
{
	cbd_t *bdp = fep->rx_bd_base;
	int i;

	kfree(fep->tx_bounce[0]);
	memset(fep->tx_bounce, 0, TX_RING_SIZE * sizeof(void*));
	for (i = 0; i < RX_RING_SIZE; i++, bdp++) {
		if (fep->rx_skbuff[i] != NULL) {
			fec_enet_rxbuf_unmap(fep, bdp, FEC_ENET_RX_FRSIZE);
			kfree_skb(fep->rx_skbuff[i]);
			fep->rx_skbuff[i] = NULL;
		}
	}
}

/* called by the generic PHY layer in interrupt context */
static int fec_mii_read(struct mii_bus *bus, int phy_id, int regnum)
{
	int ret;
	struct net_device *dev = bus->priv;
	struct fec_enet_private *fep = netdev_priv(dev);
	unsigned long regval = mk_mii_read(regnum) | phy_id << 23;
	unsigned long flags;
	int loops = 0;
	static int max_loops;

	spin_lock_irqsave(&fep->lock, flags);
	fep->mii_complete = 0;
	fec_reg_write(fep, FEC_MMFR, regval);
	spin_unlock_irqrestore(&fep->lock, flags);

	while (!fep->mii_complete) {
		if (loops++ >= 1000) {
			//return -ETIME;
		}
		udelay(1);
	}
	if (loops > max_loops) {
		max_loops = loops;
		DBG(0, "%s: mii read finished after %u loops\n", __FUNCTION__, loops);
	}
	ret = fec_reg_read(fep, FEC_MMFR);
	return ret & 0xffff;
}

static int fec_mii_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
	struct net_device *dev = bus->priv;
	struct fec_enet_private *fep = netdev_priv(dev);
	unsigned long regval = mk_mii_write(regnum, val) | phy_id << 23;
	unsigned long flags;
	int loops = 0;

	spin_lock_irqsave(&fep->lock, flags);
	fep->mii_complete = 0;
	fec_reg_write(fep, FEC_MMFR, regval);
	spin_unlock_irqrestore(&fep->lock, flags);

	while (!fep->mii_complete) {
		if (loops++ >= 100) {
			return -ETIME;
		}
		udelay(1);
	}
	return 0;
}

static int fec_init_phy(struct net_device *dev, struct fec_enet_private *fep)
{
	int ret;
	int i;
	struct mii_bus *mii;

	mii = mdiobus_alloc();
	if (mii == NULL) {
		return -ENOMEM;
	}
	mii->name = "fec mii";
	mii->read = fec_mii_read;
	mii->write = fec_mii_write;
	mii->priv = dev;
	snprintf(mii->id, MII_BUS_ID_SIZE, "%x", 0);
	mii->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		mii->irq[i] = fep->mii_irq >= 0 ? fep->mii_irq : PHY_POLL;
	}

	ret = mdiobus_register(mii);
	if (ret != 0) {
		dev_err(fep->dma_dev, "Failed to register MII bus: %d\n", ret);
		kfree(mii->irq);
		mdiobus_free(mii);
		return ret;
	}
	fep->phy_addr = -1;
	dev_dbg(fep->dma_dev, "MII bus registered\n");
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		if (mii->phy_map[i] != NULL) {
			fep->phy_addr = i;
			break;
		}
	}
	if (fep->phy_addr == -1) {
		dev_err(fep->dma_dev, "No PHY found\n");
		return -ENODEV;
	}
	dev_info(fep->dma_dev, "Using PHY at addr 0x%02x\n", fep->phy_addr);
	fep->mii = mii;

	return 0;
}

static int fec_connect_phy(struct net_device *dev, struct fec_enet_private *fep)
{
	struct mii_bus *mii = fep->mii;

	DBG(0, "%s: Connecting PHY at addr %02x\n", __FUNCTION__,
	    fep->phy_addr);

	fep->phy = phy_connect(dev, dev_name(&mii->phy_map[fep->phy_addr]->dev),
			       fec_link_change, 0, mii->phy_map[fep->phy_addr]->interface);
	if (IS_ERR(fep->phy)) {
		int ret = PTR_ERR(fep->phy);
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		fep->phy = NULL;
		return ret;
	}
	dev_dbg(&dev->dev, "%s: Registered PHY %s[%02x] IRQ %d with %s\n", __FUNCTION__,
		dev_name(&fep->phy->dev), fep->phy_addr, fep->phy->irq, dev->name);

	return 0;
}

/* Initialize the FEC Ethernet on 860T (or ColdFire 5272).
 */
 /*
  * XXX:  We need to clean up on failure exits here.
  */

static const struct net_device_ops fec_netdev_ops = {
	.ndo_open		= fec_enet_open,
	.ndo_stop		= fec_enet_close,
	.ndo_start_xmit		= fec_enet_start_xmit,
	.ndo_set_multicast_list = set_multicast_list,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= fec_timeout,
	.ndo_set_mac_address	= fec_set_mac_address,
};

int __devinit fec_enet_init(struct platform_device *pdev, struct net_device *dev)
{
	int ret;
	struct fec_enet_private *fep = netdev_priv(dev);
	cbd_t *bdp;
	struct sk_buff *pskb;
	int i;
	void *mem;

	spin_lock_init(&fep->lock);

	/* Whack a reset.  We should wait for this.
	*/
	fec_reg_write(fep, FEC_ECR, FEC_ECR_RESET);
	udelay(10);

	/* Set the Ethernet address.  If using multiple Enets on the 8xx,
	 * this needs some work to get unique addresses.
	 *
	 * This is our default MAC address unless the user changes
	 * it via eth_mac_addr (our dev->set_mac_addr handler).
	 */
	fec_get_mac(pdev, dev);

	fep->dirty_tx = fep->cur_tx = fep->tx_bd_base;
	fep->cur_rx = fep->rx_bd_base;

	fep->skb_cur = fep->skb_dirty = 0;

	/* allocate memory for TX bounce buffers */
	mem = kzalloc(TX_RING_SIZE * FEC_ENET_TX_FRSIZE, GFP_KERNEL);
	if (mem == NULL) {
		return -ENOMEM;
	}

	fec_enet_cbd_get(fep);

	/* Initialize the transmit buffer descriptors.
	*/
	bdp = fep->tx_bd_base;

	for (i = 0; i < TX_RING_SIZE; i++) {
		fep->tx_bounce[i] = mem;
		mem = (void *)((unsigned long)(mem + FEC_ENET_TX_FRSIZE));

		/* Initialize the BD for every fragment in the page.
		*/
		bdp->cbd_bufaddr = ~0;
		bdp++;
	}

	/* Set the last buffer to wrap.
	*/
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* ...and the same for receive.
	*/
	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++, bdp++) {
		pskb = __dev_alloc_skb(FEC_ENET_RX_FRSIZE, GFP_KERNEL);
		if (pskb == NULL) {
			DBG(0, "%s: Failed to allocate RX skb; cleaning up\n", __FUNCTION__);
			ret = -ENOMEM;
			goto cleanup;
		}
		DBG(0, "%s: RX skb allocated @ %p\n", __FUNCTION__, pskb);
		fep->rx_skbuff[i] = pskb;
		pskb->data = FEC_ADDR_ALIGNMENT(pskb->data);
		bdp->cbd_sc = BD_ENET_RX_EMPTY;
		bdp->cbd_bufaddr = ~0;
		fec_enet_rxbuf_map(fep, bdp, pskb->data, FEC_ENET_RX_FRSIZE);
	}
	/* Set the last buffer to wrap.
	*/
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;
	fec_enet_cbd_put(fep);

	/* Set receive and transmit descriptor base.
	*/
	fec_reg_write(fep, FEC_ERDSR, fep->cbd_phys_base);
	fec_reg_write(fep, FEC_ETDSR, fep->cbd_phys_base + RX_RING_SIZE * sizeof(cbd_t));

	/* Install our interrupt handlers. This varies depending on
	 * the architecture.
	*/
	ret = fec_request_intrs(pdev, dev);
	if (ret != 0) {
		goto cleanup;
	}
	/* Clear and enable interrupts */
	fec_reg_write(fep, FEC_EIR, fec_reg_read(fep, FEC_EIR));
	fec_reg_write(fep, FEC_EIMR, FEC_ENET_TXF | FEC_ENET_TXB |
		      FEC_ENET_RXF | FEC_ENET_RXB | FEC_ENET_MII);

	fec_reg_write(fep, FEC_IAUR, 0);
	fec_reg_write(fep, FEC_IALR, 0);
	fec_reg_write(fep, FEC_EMRBR, PKT_MAXBLR_SIZE);
	fec_reg_write(fep, FEC_ECR, FEC_ECR_ETHER_EN);

	ret = fec_localhw_setup(dev);
	if (ret != 0) {
		goto cleanup;
	}

	/* The FEC Ethernet specific entries in the device structure. */
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->netdev_ops = &fec_netdev_ops;

	/* setup MII interface */
	ret = fec_set_mii(dev, fep);
	if (ret) {
		DBG(0, "%s: Failed to initialize MII interface: %d\n", __FUNCTION__, ret);
		goto cleanup;
	}

	ret = fec_init_phy(dev, fep);
	if (ret) {
		DBG(0, "%s: Failed to initialize PHY: %d\n", __FUNCTION__, ret);
		goto cleanup;
	}
	return 0;

cleanup:
	fec_enet_free_buffers(fep);
	fec_enet_cbd_put(fep);
	return ret;
}

/* This function is called to start or restart the FEC during a link
 * change.  This only happens when switching between half and full
 * duplex.
 */
static void
fec_restart(struct net_device *dev, int duplex)
{
	struct fec_enet_private *fep = netdev_priv(dev);
	cbd_t *bdp;
	int i;
	u32 rcr = OPT_FRAME_SIZE | RCR_MII_MODE;	/* MII enable */
	u32 tcr = TCR_HBC;

	DBG(0, "%s: Restarting FEC in %s-duplex mode\n", __FUNCTION__,
	    duplex ? "full" : "half");
	/* Whack a reset.  We should wait for this.
	 */
	fec_reg_write(fep, FEC_ECR, FEC_ECR_RESET);
	udelay(10);

	/* Enable interrupts we wish to service.
	 */
	fec_reg_write(fep, FEC_EIMR, FEC_ENET_TXF | FEC_ENET_TXB |
		      FEC_ENET_RXF | FEC_ENET_RXB | FEC_ENET_MII);

	/* Clear any outstanding interrupt.
	 *
	 */
	fec_reg_write(fep, FEC_EIR, FEC_ENET_MASK);

	/* Set station address.
	 */
	_fec_set_mac_address(dev);

	/* Reset all multicast.
	 */
	fec_reg_write(fep, FEC_IAUR, 0);
	fec_reg_write(fep, FEC_IALR, 0);

	/* Set maximum receive buffer size.
	 */
	fec_reg_write(fep, FEC_EMRBR, PKT_MAXBLR_SIZE);

	/* Set receive and transmit descriptor base.
	 */
	fec_reg_write(fep, FEC_ERDSR, fep->cbd_phys_base);
	fec_reg_write(fep, FEC_ETDSR, fep->cbd_phys_base + RX_RING_SIZE * sizeof(cbd_t));

	fep->dirty_tx = fep->cur_tx = fep->tx_bd_base;
	fep->cur_rx = fep->rx_bd_base;

	/* Reset SKB transmit buffers.
	 */
	fep->skb_cur = fep->skb_dirty = 0;
	bdp = fep->tx_bd_base;
	for (i = 0; i <= TX_RING_MOD_MASK; i++) {
		if (fep->tx_skbuff[i] != NULL) {
			fec_free_skb(fep, bdp, &fep->tx_skbuff[i]);
			bdp++;
		}
	}

	/* Initialize the receive buffer descriptors.
	 */
	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		/* Initialize the BD for every fragment in the page.
		*/
		bdp->cbd_sc = BD_ENET_RX_EMPTY;
		bdp++;
	}

	/* Set the last buffer to wrap.
	 */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* ...and the same for transmmit.
	 */
	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		/* Initialize the BD for every fragment in the page.
		*/
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = ~0;
		bdp++;
	}

	/* Set the last buffer to wrap.
	 */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* Enable MII mode.
	 */
	if (duplex) {
		tcr |= TCR_FDEN;	/* FD enable */
	} else {
		rcr |= RCR_DRT;		/* No Rcv on Xmit */
	}
	fec_reg_write(fep, FEC_RCR, rcr);
	fec_reg_write(fep, FEC_TCR, tcr);
	fep->full_duplex = duplex;

	/* Set MII speed.
	 */
	fec_reg_write(fep, FEC_MSCR, fep->phy_speed);

	/* And last, enable the transmit and receive processing.
	 */
	fec_reg_write(fep, FEC_ECR, FEC_ECR_ETHER_EN);
	fec_localhw_setup(dev);
	fec_reg_write(fep, FEC_RDAR, DONT_CARE);

	DBG(0, "%s: Starting netif queue\n", __FUNCTION__);
	netif_start_queue(dev);
}

static void
fec_stop(struct net_device *dev)
{
	struct fec_enet_private *fep = netdev_priv(dev);

	netif_stop_queue(dev);

	/*
	 * We cannot expect a graceful transmit stop without link!
	 */
	if (fep->linkstatus) {
		fec_reg_write(fep, FEC_TCR, 0x01);	/* Graceful transmit stop */
		udelay(10);
		if (!(fec_reg_read(fep, FEC_EIR) & FEC_ENET_GRA))
			dev_warn(&dev->dev, "Graceful transmit stop did not complete!\n");
	}
#if 0
	/* Whack a reset.  We should wait for this.
	 */
	fec_reg_write(fep, FEC_ECR, FEC_ECR_RESET);
	udelay(10);
	/* Mask and clear outstanding MII command interrupts.
	 */
	fec_reg_write(fep, FEC_EIMR, 0);
	fec_reg_write(fep, FEC_EIR, FEC_ENET_MII);
	fec_reg_write(fep, FEC_MSCR, fep->phy_speed);
#endif
}

static int __devinit fec_enet_probe(struct platform_device *pdev)
{
	int ret;
	struct fec_enet_private *fep;
	struct net_device *dev;
	struct fec_enet_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res_mem1;
	struct resource *res_mem2;

	res_mem1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res_mem1 == NULL) {
		return -ENODEV;
	}

	res_mem1 = request_mem_region(res_mem1->start,
				      resource_size(res_mem1),
				      DRV_NAME);
	if (res_mem1 == NULL) {
		return -EBUSY;
	}
	res_mem2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res_mem2 != NULL) {
		res_mem2 = request_mem_region(res_mem2->start,
					      resource_size(res_mem2),
					      DRV_NAME);
		if (res_mem2 == NULL) {
			ret = -EBUSY;
			goto release1;
		}
	}

	dev = alloc_etherdev(sizeof(struct fec_enet_private));
	if (dev == NULL) {
		ret = -ENOMEM;
		goto release2;
	}
	platform_set_drvdata(pdev, dev);
	fep = netdev_priv(dev);
	fep->res_mem1 = res_mem1;
	fep->res_mem2 = res_mem2;
	fep->dma_dev = &pdev->dev;

	fep->reg_base = ioremap(res_mem1->start, resource_size(res_mem1));
	if (fep->reg_base == NULL) {
		ret = -ENOMEM;
		goto free_netdev;
	}

	fep->mib_base = ioremap(res_mem2->start, resource_size(res_mem2));
	if (fep->mib_base == NULL) {
		ret = -ENOMEM;
		goto unmap1;
	}

	/* Allocate memory for buffer descriptors. */
	fep->cbd_mem_base = dma_alloc_coherent(&pdev->dev, CBD_BUF_SIZE,
					       &fep->cbd_phys_base,
					       GFP_KERNEL);
	if (fep->cbd_mem_base == NULL) {
		ret = -ENOMEM;
		goto unmap2;
	}

	/* Set receive and transmit descriptor base.
	*/
	fep->rx_bd_base = fep->cbd_mem_base;
	fep->tx_bd_base = fep->rx_bd_base + RX_RING_SIZE;

	ret = platform_func(pdata->arch_init, pdev);
	if (ret != 0) {
		dev_err(&pdev->dev, "platform init failed: %d\n", ret);
		goto free_dma;
	}

	ret = fec_enet_init(pdev, dev);
	if (ret != 0) {
		goto fec_disable;
	}

	/* Enable most messages by default */
	fep->msg_enable = (NETIF_MSG_IFUP << 1) - 1;
	ret = register_netdev(dev);
	if (ret != 0) {
		/* XXX: missing cleanup here */
		goto free_buffers;
	}

	dev_info(&dev->dev, "%s: ethernet %02x:%02x:%02x:%02x:%02x:%02x\n", dev->name,
	       dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
	       dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	return 0;

 free_buffers:
	fec_enet_free_buffers(fep);

 fec_disable:
	platform_func(pdata->arch_exit, pdev);

 free_dma:
	dma_free_coherent(&pdev->dev, CBD_BUF_SIZE, fep->cbd_mem_base, fep->cbd_phys_base);

 unmap2:
	if (fep->mib_base)
		iounmap(fep->mib_base);

 unmap1:
	iounmap(fep->reg_base);

 free_netdev:
	free_netdev(dev);

 release2:
	if (res_mem2 != NULL) {
		release_resource(res_mem2);
	}

 release1:
	release_resource(res_mem1);

	return ret;
}

static int __devexit fec_enet_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(dev);

	unregister_netdev(dev);

	if (fep->mii != NULL) {
		kfree(fep->mii->irq);
		mdiobus_unregister(fep->mii);
	}
	mdiobus_free(fep->mii);

	fec_release_intrs(dev);

	iounmap(fep->reg_base);
	if (fep->mib_base)
		iounmap(fep->mib_base);

	fec_enet_free_buffers(fep);

	dma_free_coherent(&pdev->dev, CBD_BUF_SIZE, fep->cbd_mem_base, fep->cbd_phys_base);

	release_resource(fep->res_mem1);
	if (fep->res_mem2 != NULL) {
		release_resource(fep->res_mem2);
	}
	free_netdev(dev);
	return 0;
}

static void fec_enet_shutdown(struct platform_device *pdev)
{
	struct fec_enet_platform_data *pdata = pdev->dev.platform_data;

	platform_func(pdata->arch_exit, pdev);
}

#ifdef CONFIG_PM
static int fec_enet_suspend(struct device *dev)
{
	int ret;
	struct fec_enet_platform_data *pdata = dev->platform_data;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct fec_enet_private *fep = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_device_detach(ndev);
		phy_disconnect(fep->phy);
		fep->phy = NULL;
	}

	ret = platform_func(pdata->suspend, to_platform_device(dev));
	if (ret != 0 && netif_running(ndev)) {
		/* Undo suspend */

		if (fec_connect_phy(ndev, fep) != 0) {
			dev_err(dev, "Failed to connect to PHY\n");
			return ret;
		}
		phy_start(fep->phy);

		fec_link_change(ndev);
		netif_device_attach(ndev);
	}
	return ret;
}

static int fec_enet_resume(struct device *dev)
{
	int ret;
	struct fec_enet_platform_data *pdata = dev->platform_data;
	struct net_device *ndev = dev_get_drvdata(dev);

	ret = platform_func(pdata->resume, to_platform_device(dev));
	if (ret != 0) {
		return ret;
	}
	if (netif_running(ndev)) {
		struct fec_enet_private *fep = netdev_priv(ndev);

		ret = fec_connect_phy(ndev, fep);
		if (ret != 0) {
			dev_err(dev, "Failed to connect to PHY: %d\n", ret);
			return ret;
		}
		phy_start(fep->phy);

		fec_link_change(ndev);
		netif_device_attach(ndev);
	}
	return 0;
}
#else
#define fec_enet_suspend	NULL
#define fec_enet_resume		NULL
#endif

static struct dev_pm_ops fec_enet_pm_ops = {
	.suspend = fec_enet_suspend,
	.resume = fec_enet_resume,
};

static struct platform_driver fec_enet_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &fec_enet_pm_ops,
	},
	.probe = fec_enet_probe,
	.remove = __devexit_p(fec_enet_remove),
	.shutdown = fec_enet_shutdown,
};

static int __init fec_enet_module_init(void)
{
	int ret;

	ret = platform_driver_register(&fec_enet_driver);

	return ret;
}
module_init(fec_enet_module_init);

static void __exit fec_enet_module_cleanup(void)
{
	platform_driver_unregister(&fec_enet_driver);
}
module_exit(fec_enet_module_cleanup);

MODULE_LICENSE("GPL");
