/*
 * s3c2440 DMA handling
 *
 * Copyright (c) 2013 Heiko Stuebner <heiko@sntech.de>
 *
 * based on amba-pl08x.c
 *
 * Copyright (c) 2006 ARM Ltd.
 * Copyright (c) 2010 ST-Ericsson SA
 *
 * Author: Peter Pearse <peter.pearse@arm.com>
 * Author: Linus Walleij <linus.walleij@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * The DMA controllers in s3c2440 SoCs have a varying number of DMA signals
 * that can be routed to any of the 4 to 8 hardware-channels.
 *
 * Therefore on these DMA controllers the number of channels
 * and the number of incoming DMA signals are two totally different things.
 * It is usually not possible to theoretically handle all physical signals,
 * so a multiplexing scheme with possible denial of use is necessary.
 *
 * Open items:
 * - bursts
 */

#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-s3c24xx.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>

#include "dmaengine.h"
#include "virt-dma.h"

#define MAX_DMA_CHANNELS	8

#define s3c2440_DISRC			0x00
#define s3c2440_DISRCC			0x04
#define s3c2440_DISRCC_INC_INCREMENT	0
#define s3c2440_DISRCC_INC_FIXED	BIT(0)
#define s3c2440_DISRCC_LOC_AHB		0
#define s3c2440_DISRCC_LOC_APB		BIT(1)

#define s3c2440_DIDST			0x08
#define s3c2440_DIDSTC			0x0c
#define s3c2440_DIDSTC_INC_INCREMENT	0
#define s3c2440_DIDSTC_INC_FIXED	BIT(0)
#define s3c2440_DIDSTC_LOC_AHB		0
#define s3c2440_DIDSTC_LOC_APB		BIT(1)
#define s3c2440_DIDSTC_INT_TC0		0
#define s3c2440_DIDSTC_INT_RELOAD	BIT(2)

#define s3c2440_DCON			0x10

#define s3c2440_DCON_TC_MASK		0xfffff
#define s3c2440_DCON_DSZ_BYTE		(0 << 20)
#define s3c2440_DCON_DSZ_HALFWORD	(1 << 20)
#define s3c2440_DCON_DSZ_WORD		(2 << 20)
#define s3c2440_DCON_DSZ_MASK		(3 << 20)
#define s3c2440_DCON_DSZ_SHIFT		20
#define s3c2440_DCON_AUTORELOAD		0
#define s3c2440_DCON_NORELOAD		BIT(22)
#define s3c2440_DCON_HWTRIG		BIT(23)
#define s3c2440_DCON_HWSRC_SHIFT	24
#define s3c2440_DCON_SERV_SINGLE	0
#define s3c2440_DCON_SERV_WHOLE		BIT(27)
#define s3c2440_DCON_TSZ_UNIT		0
#define s3c2440_DCON_TSZ_BURST4		BIT(28)
#define s3c2440_DCON_INT		BIT(29)
#define s3c2440_DCON_SYNC_PCLK		0
#define s3c2440_DCON_SYNC_HCLK		BIT(30)
#define s3c2440_DCON_DEMAND		0
#define s3c2440_DCON_HANDSHAKE		BIT(31)

#define s3c2440_DSTAT			0x14
#define s3c2440_DSTAT_STAT_BUSY		BIT(20)
#define s3c2440_DSTAT_CURRTC_MASK	0xfffff

#define s3c2440_DMASKTRIG		0x20
#define s3c2440_DMASKTRIG_SWTRIG	BIT(0)
#define s3c2440_DMASKTRIG_ON		BIT(1)
#define s3c2440_DMASKTRIG_STOP		BIT(2)

/*
 * S3C2440 SoCs cannot select any physical channel
 * for a DMA source. Instead only specific channels are valid.
 * All of these SoCs have 4 physical channels and the number of request
 * source bits is 3. Additionally we also need 1 bit to mark the channel
 * as valid.
 * Therefore we separate the chansel element of the channel data into 4
 * parts of 4 bits each, to hold the information if the channel is valid
 * and the hw request source to use.
 *
 * Example:
 * SDI is valid on channels 0, 2 and 3 - with varying hw request sources.
 * For it the chansel field would look like
 *
 * ((BIT(3) | 1) << 3 * 4) | // channel 3, with request source 1
 * ((BIT(3) | 2) << 2 * 4) | // channel 2, with request source 2
 * ((BIT(3) | 2) << 0 * 4)   // channel 0, with request source 2
 */
#define s3c2440_CHANSEL_WIDTH		4
#define s3c2440_CHANSEL_VALID		BIT(3)
#define s3c2440_CHANSEL_REQ_MASK	7

/*
 * enum s3c2440_dma_chan_state - holds the virtual channel states
 * @s3c2440_DMA_CHAN_IDLE: the channel is idle
 * @s3c2440_DMA_CHAN_RUNNING: the channel has allocated a physical transport
 * channel and is running a transfer on it
 * @s3c2440_DMA_CHAN_WAITING: the channel is waiting for a physical transport
 * channel to become available
 */
enum s3c2440_dma_chan_state {
	s3c2440_DMA_CHAN_IDLE,
	s3c2440_DMA_CHAN_RUNNING,
	s3c2440_DMA_CHAN_WAITING,
};

/*
 * struct s3c2440_sg - structure containing data per sg
 * @src_addr: src address of sg
 * @dst_addr: dst address of sg
 * @len: transfer len in bytes
 * @node: node for txd's dsg_list
 */
struct s3c2440_sg {
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	size_t len;
	struct list_head node;
};

/*
 * struct s3c2440_txd - wrapper for struct dma_async_tx_descriptor
 * @vd: virtual DMA descriptor
 * @dsg_list: list of children sg's
 * @at: sg currently being transfered
 * @width: transfer width
 * @disrcc: value for source control register
 * @didstc: value for destination control register
 * @dcon: base value for dcon register
 * @cyclic: indicate cyclic transfer
 */
struct s3c2440_txd {
	struct virt_dma_desc vd;
	struct list_head dsg_list;
	struct list_head *at;
	u8 width;
	u32 disrcc;
	u32 didstc;
	u32 dcon;
	bool cyclic;
};

/*
 * struct s3c2440_dma_phy - holder for the physical channels
 * @id: physical index to this channel
 * @valid: does the channel have all required elements
 * @base: virtual memory base (remapped) for the this channel
 * @irq: interrupt for this channel
 * @clk: clock for this channel
 * @lock: a lock to use when altering an instance of this struct
 * @serving: virtual channel currently being served by this physicalchannel
 * @host: a pointer to the host (internal use)
 */
struct s3c2440_dma_phy {
	unsigned int			id;
	bool				valid;
	void __iomem			*base;
	int				irq;
	spinlock_t			lock;
	struct s3c2440_dma_chan		*serving;
	struct s3c2440_dma_engine	*host;
};

/*
 * struct s3c2440_dma_chan - this structure wraps a DMA ENGINE channel
 * @id: the id of the channel
 * @name: name of the channel
 * @vc: wrappped virtual channel
 * @phy: the physical channel utilized by this channel, if there is one
 * @runtime_addr: address for RX/TX according to the runtime config
 * @at: active transaction on this channel
 * @lock: a lock for this channel data
 * @host: a pointer to the host (internal use)
 * @state: whether the channel is idle, running etc
 */
struct s3c2440_dma_chan {
	int id;
	const char *name;
	struct virt_dma_chan vc;
	struct s3c2440_dma_phy *phy;
	struct dma_slave_config cfg;
	struct s3c2440_txd *at;
	struct s3c2440_dma_engine *host;
	enum s3c2440_dma_chan_state state;
};

/*
 * struct s3c2440_dma_engine - the local state holder for the s3c2440
 * @pdev: the corresponding platform device
 * @pdata: platform data passed in from the platform/machine
 * @base: virtual memory base (remapped)
 * @slave: slave engine for this instance
 * @memcpy: memcpy engine for this instance
 * @phy_chans: array of data for the physical channels
 */
struct s3c2440_dma_engine {
	struct platform_device			*pdev;
	const struct s3c24xx_dma_platdata	*pdata;
	void __iomem				*base;
	struct dma_device			slave;
	struct s3c2440_dma_phy			*phy_chans;
};

/*
 * Physical channel handling
 */

/*
 * Check whether a certain channel is busy or not.
 */
static int s3c2440_dma_phy_busy(struct s3c2440_dma_phy *phy)
{
	unsigned int val = readl(phy->base + s3c2440_DSTAT);
	return val & s3c2440_DSTAT_STAT_BUSY;
}

static bool s3c2440_dma_phy_valid(struct s3c2440_dma_chan *s3cchan,
				  struct s3c2440_dma_phy *phy)
{
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	const struct s3c24xx_dma_platdata *pdata = s3cdma->pdata;
	struct s3c24xx_dma_channel *cdata = &pdata->channels[s3cchan->id];
	int phyvalid;

	phyvalid = (cdata->chansel >> (phy->id * s3c2440_CHANSEL_WIDTH));
	return (phyvalid & s3c2440_CHANSEL_VALID) ? true : false;
}

/*
 * Allocate a physical channel for a virtual channel
 *
 * Try to locate a physical channel to be used for this transfer. If all
 * are taken return NULL and the requester will have to cope by using
 * some fallback PIO mode or retrying later.
 */
static
struct s3c2440_dma_phy *s3c2440_dma_get_phy(struct s3c2440_dma_chan *s3cchan)
{
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	const struct s3c24xx_dma_platdata *pdata = s3cdma->pdata;
	struct s3c2440_dma_phy *phy = NULL;
	unsigned long flags;
	int i;

	for (i = 0; i < pdata->num_phy_channels; i++) {
		phy = &s3cdma->phy_chans[i];

		if (!phy->valid)
			continue;

		if (!s3c2440_dma_phy_valid(s3cchan, phy))
			continue;

		spin_lock_irqsave(&phy->lock, flags);

		if (!phy->serving) {
			phy->serving = s3cchan;
			spin_unlock_irqrestore(&phy->lock, flags);
			break;
		}

		spin_unlock_irqrestore(&phy->lock, flags);
	}

	/* No physical channel available, cope with it */
	if (i == pdata->num_phy_channels) {
		dev_warn(&s3cdma->pdev->dev, "no phy channel available\n");
		return NULL;
	}

	return phy;
}

/*
 * Mark the physical channel as free.
 *
 * This drops the link between the physical and virtual channel.
 */
static inline void s3c2440_dma_put_phy(struct s3c2440_dma_phy *phy)
{
	phy->serving = NULL;
}

/*
 * Stops the channel by writing the stop bit.
 * This should not be used for an on-going transfer, but as a method of
 * shutting down a channel (eg, when it's no longer used) or terminating a
 * transfer.
 */
static void s3c2440_dma_terminate_phy(struct s3c2440_dma_phy *phy)
{
	writel(s3c2440_DMASKTRIG_STOP, phy->base + s3c2440_DMASKTRIG);
}

/*
 * Virtual channel handling
 */

static inline
struct s3c2440_dma_chan *to_s3c2440_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct s3c2440_dma_chan, vc.chan);
}

static u32 s3c2440_dma_getbytes_chan(struct s3c2440_dma_chan *s3cchan)
{
	struct s3c2440_dma_phy *phy = s3cchan->phy;
	struct s3c2440_txd *txd = s3cchan->at;
	u32 tc = readl(phy->base + s3c2440_DSTAT) & s3c2440_DSTAT_CURRTC_MASK;

	return tc * txd->width;
}

static int s3c2440_dma_set_runtime_config(struct dma_chan *chan,
				  struct dma_slave_config *config)
{
	struct s3c2440_dma_chan *s3cchan = to_s3c2440_dma_chan(chan);
	unsigned long flags;
	int ret = 0;

	/* Reject definitely invalid configurations */
	if (config->src_addr_width == DMA_SLAVE_BUSWIDTH_8_BYTES ||
	    config->dst_addr_width == DMA_SLAVE_BUSWIDTH_8_BYTES)
		return -EINVAL;

	spin_lock_irqsave(&s3cchan->vc.lock, flags);
	s3cchan->cfg = *config;
	spin_unlock_irqrestore(&s3cchan->vc.lock, flags);

	return ret;
}

/*
 * Transfer handling
 */

static inline
struct s3c2440_txd *to_s3c2440_txd(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct s3c2440_txd, vd.tx);
}

static struct s3c2440_txd *s3c2440_dma_get_txd(void)
{
	struct s3c2440_txd *txd = kzalloc(sizeof(*txd), GFP_NOWAIT);

	if (txd) {
		INIT_LIST_HEAD(&txd->dsg_list);
		txd->dcon = s3c2440_DCON_INT | s3c2440_DCON_NORELOAD;
	}

	return txd;
}

static void s3c2440_dma_free_txd(struct s3c2440_txd *txd)
{
	struct s3c2440_sg *dsg, *_dsg;

	list_for_each_entry_safe(dsg, _dsg, &txd->dsg_list, node) {
		list_del(&dsg->node);
		kfree(dsg);
	}

	kfree(txd);
}

static void s3c2440_dma_start_next_sg(struct s3c2440_dma_chan *s3cchan,
				       struct s3c2440_txd *txd)
{
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	struct s3c2440_dma_phy *phy = s3cchan->phy;
	const struct s3c24xx_dma_platdata *pdata = s3cdma->pdata;
	struct s3c2440_sg *dsg = list_entry(txd->at, struct s3c2440_sg, node);
	struct s3c24xx_dma_channel *cdata;
	struct dma_slave_config *cfg = &s3cchan->cfg;
	int direction = cfg->direction;
	int csel;
	u32 dcon = txd->dcon;
	u32 val;

	/* transfer-size and -count from len and width */
	switch (txd->width) {
	case 1:
		dcon |= s3c2440_DCON_DSZ_BYTE | dsg->len;
		break;
	case 2:
		dcon |= s3c2440_DCON_DSZ_HALFWORD | (dsg->len / 2);
		break;
	case 4:
		dcon |= s3c2440_DCON_DSZ_WORD | (dsg->len / 4);
		break;
	}

	cdata = &pdata->channels[s3cchan->id];
	csel = cdata->chansel >> (phy->id * s3c2440_CHANSEL_WIDTH);

	csel &= s3c2440_CHANSEL_REQ_MASK;
	dcon |= csel << s3c2440_DCON_HWSRC_SHIFT;
	if (direction == DMA_MEM_TO_MEM) // mem to mem use soft trigger
		dcon &= ~s3c2440_DCON_HWTRIG;
	else // mem to dev or dev to mem use hw trigger
		dcon |= s3c2440_DCON_HWTRIG;

	writel_relaxed(dsg->src_addr, phy->base + s3c2440_DISRC);
	writel_relaxed(txd->disrcc, phy->base + s3c2440_DISRCC);
	writel_relaxed(dsg->dst_addr, phy->base + s3c2440_DIDST);
	writel_relaxed(txd->didstc, phy->base + s3c2440_DIDSTC);
	writel_relaxed(dcon, phy->base + s3c2440_DCON);

	val = readl_relaxed(phy->base + s3c2440_DMASKTRIG);
	val &= ~s3c2440_DMASKTRIG_STOP;
	val |= s3c2440_DMASKTRIG_ON;

	if (direction == DMA_MEM_TO_MEM)
		val |= s3c2440_DMASKTRIG_SWTRIG;

	writel(val, phy->base + s3c2440_DMASKTRIG);
}

/*
 * Set the initial DMA register values and start first sg.
 */
static void s3c2440_dma_start_next_txd(struct s3c2440_dma_chan *s3cchan)
{
	struct s3c2440_dma_phy *phy = s3cchan->phy;
	struct virt_dma_desc *vd = vchan_next_desc(&s3cchan->vc);
	struct s3c2440_txd *txd = to_s3c2440_txd(&vd->tx);

	list_del(&txd->vd.node);

	s3cchan->at = txd;

	/* Wait for channel inactive */
	while (s3c2440_dma_phy_busy(phy))
		cpu_relax();

	/* point to the first element of the sg list */
	txd->at = txd->dsg_list.next;
	s3c2440_dma_start_next_sg(s3cchan, txd);
}

static void s3c2440_dma_free_txd_list(struct s3c2440_dma_engine *s3cdma,
				struct s3c2440_dma_chan *s3cchan)
{
	LIST_HEAD(head);

	vchan_get_all_descriptors(&s3cchan->vc, &head);
	vchan_dma_desc_free_list(&s3cchan->vc, &head);
}

/*
 * Try to allocate a physical channel.  When successful, assign it to
 * this virtual channel, and initiate the next descriptor.  The
 * virtual channel lock must be held at this point.
 */
static void s3c2440_dma_phy_alloc_and_start(struct s3c2440_dma_chan *s3cchan)
{
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	struct s3c2440_dma_phy *phy;

	phy = s3c2440_dma_get_phy(s3cchan);
	if (!phy) {
		dev_dbg(&s3cdma->pdev->dev, "no physical channel available for xfer on %s\n",
			s3cchan->name);
		s3cchan->state = s3c2440_DMA_CHAN_WAITING;
		return;
	}

	dev_dbg(&s3cdma->pdev->dev, "allocated physical channel %d for xfer on %s\n",
		phy->id, s3cchan->name);

	s3cchan->phy = phy;
	s3cchan->state = s3c2440_DMA_CHAN_RUNNING;

	s3c2440_dma_start_next_txd(s3cchan);
}

static void s3c2440_dma_phy_reassign_start(struct s3c2440_dma_phy *phy,
	struct s3c2440_dma_chan *s3cchan)
{
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;

	dev_dbg(&s3cdma->pdev->dev, "reassigned physical channel %d for xfer on %s\n",
		phy->id, s3cchan->name);

	/*
	 * We do this without taking the lock; we're really only concerned
	 * about whether this pointer is NULL or not, and we're guaranteed
	 * that this will only be called when it _already_ is non-NULL.
	 */
	phy->serving = s3cchan;
	s3cchan->phy = phy;
	s3cchan->state = s3c2440_DMA_CHAN_RUNNING;
	s3c2440_dma_start_next_txd(s3cchan);
}

/*
 * Free a physical DMA channel, potentially reallocating it to another
 * virtual channel if we have any pending.
 */
static void s3c2440_dma_phy_free(struct s3c2440_dma_chan *s3cchan)
{
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	struct s3c2440_dma_chan *p, *next = NULL;

retry:

	list_for_each_entry(p, &s3cdma->slave.channels,
		vc.chan.device_node)
		if (p->state == s3c2440_DMA_CHAN_WAITING &&
			s3c2440_dma_phy_valid(p, s3cchan->phy)) {
			next = p;
			break;
		}

	/* Ensure that the physical channel is stopped */
	s3c2440_dma_terminate_phy(s3cchan->phy);

	if (next) {
		bool success;

		/*
		 * Eww.  We know this isn't going to deadlock
		 * but lockdep probably doesn't.
		 */
		spin_lock(&next->vc.lock);
		/* Re-check the state now that we have the lock */
		success = next->state == s3c2440_DMA_CHAN_WAITING;
		if (success)
			s3c2440_dma_phy_reassign_start(s3cchan->phy, next);
		spin_unlock(&next->vc.lock);

		/* If the state changed, try to find another channel */
		if (!success)
			goto retry;
	} else {
		/* No more jobs, so free up the physical channel */
		s3c2440_dma_put_phy(s3cchan->phy);
	}

	s3cchan->phy = NULL;
	s3cchan->state = s3c2440_DMA_CHAN_IDLE;
}

static void s3c2440_dma_desc_free(struct virt_dma_desc *vd)
{
	struct s3c2440_txd *txd = to_s3c2440_txd(&vd->tx);

	s3c2440_dma_free_txd(txd);
}

static irqreturn_t s3c2440_dma_irq(int irq, void *data)
{
	struct s3c2440_dma_phy *phy = data;
	struct s3c2440_dma_chan *s3cchan = phy->serving;
	struct s3c2440_txd *txd;

	dev_dbg(&phy->host->pdev->dev, "interrupt on channel %d\n", phy->id);

	/*
	 * Interrupts happen to notify the completion of a transfer and the
	 * channel should have moved into its stop state already on its own.
	 * Therefore interrupts on channels not bound to a virtual channel
	 * should never happen. Nevertheless send a terminate command to the
	 * channel if the unlikely case happens.
	 */
	if (unlikely(!s3cchan)) {
		dev_err(&phy->host->pdev->dev, "interrupt on unused channel %d\n",
			phy->id);

		s3c2440_dma_terminate_phy(phy);

		return IRQ_HANDLED;
	}

	spin_lock(&s3cchan->vc.lock);
	txd = s3cchan->at;
	if (txd) {
		/* when more sg's are in this txd, start the next one */
		if (!list_is_last(txd->at, &txd->dsg_list)) {
			txd->at = txd->at->next;
			if (txd->cyclic)
				vchan_cyclic_callback(&txd->vd);
			s3c2440_dma_start_next_sg(s3cchan, txd);
		} else if (!txd->cyclic) {
			s3cchan->at = NULL;
			vchan_cookie_complete(&txd->vd);

			/*
			 * And start the next descriptor (if any),
			 * otherwise free this channel.
			 */
			if (vchan_next_desc(&s3cchan->vc))
				s3c2440_dma_start_next_txd(s3cchan);
			else
				s3c2440_dma_phy_free(s3cchan);
		} else {
			vchan_cyclic_callback(&txd->vd);

			/* Cyclic: reset at beginning */
			txd->at = txd->dsg_list.next;
			s3c2440_dma_start_next_sg(s3cchan, txd);
		}
	}
	spin_unlock(&s3cchan->vc.lock);

	return IRQ_HANDLED;
}

/*
 * The DMA ENGINE API
 */

static int s3c2440_dma_terminate_all(struct dma_chan *chan)
{
	struct s3c2440_dma_chan *s3cchan = to_s3c2440_dma_chan(chan);
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&s3cchan->vc.lock, flags);

	if (!s3cchan->phy && !s3cchan->at) {
		dev_err(&s3cdma->pdev->dev, "trying to terminate already stopped channel %d\n",
			s3cchan->id);
		ret = -EINVAL;
		goto unlock;
	}

	s3cchan->state = s3c2440_DMA_CHAN_IDLE;

	/* Mark physical channel as free */
	if (s3cchan->phy)
		s3c2440_dma_phy_free(s3cchan);

	/* Dequeue current job */
	if (s3cchan->at) {
		s3c2440_dma_desc_free(&s3cchan->at->vd);
		s3cchan->at = NULL;
	}

	/* Dequeue jobs not yet fired as well */
	s3c2440_dma_free_txd_list(s3cdma, s3cchan);
unlock:
	spin_unlock_irqrestore(&s3cchan->vc.lock, flags);

	return ret;
}

static void s3c2440_dma_free_chan_resources(struct dma_chan *chan)
{
	/* Ensure all queued descriptors are freed */
	vchan_free_chan_resources(to_virt_chan(chan));
}

static enum dma_status s3c2440_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct s3c2440_dma_chan *s3cchan = to_s3c2440_dma_chan(chan);
	struct s3c2440_txd *txd;
	struct s3c2440_sg *dsg;
	struct virt_dma_desc *vd;
	unsigned long flags;
	enum dma_status ret;
	size_t bytes = 0;

	spin_lock_irqsave(&s3cchan->vc.lock, flags);
	ret = dma_cookie_status(chan, cookie, txstate);

	/*
	 * There's no point calculating the residue if there's
	 * no txstate to store the value.
	 */
	if (ret == DMA_COMPLETE || !txstate) {
		spin_unlock_irqrestore(&s3cchan->vc.lock, flags);
		return ret;
	}

	vd = vchan_find_desc(&s3cchan->vc, cookie);
	if (vd) {
		/* On the issued list, so hasn't been processed yet */
		txd = to_s3c2440_txd(&vd->tx);

		list_for_each_entry(dsg, &txd->dsg_list, node)
			bytes += dsg->len;
	} else {
		/*
		 * Currently running, so sum over the pending sg's and
		 * the currently active one.
		 */
		txd = s3cchan->at;

		dsg = list_entry(txd->at, struct s3c2440_sg, node);
		list_for_each_entry_from(dsg, &txd->dsg_list, node)
			bytes += dsg->len;

		bytes += s3c2440_dma_getbytes_chan(s3cchan);
	}
	spin_unlock_irqrestore(&s3cchan->vc.lock, flags);

	/*
	 * This cookie not complete yet
	 * Get number of bytes left in the active transactions and queue
	 */
	dma_set_residue(txstate, bytes);

	/* Whether waiting or running, we're in progress */
	return ret;
}

/*
 * Initialize a descriptor to be used by memcpy submit
 */
static struct dma_async_tx_descriptor *s3c2440_dma_prep_memcpy(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	struct s3c2440_dma_chan *s3cchan = to_s3c2440_dma_chan(chan);
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	struct s3c2440_txd *txd;
	struct s3c2440_sg *dsg;
	int src_mod, dest_mod;

	dev_dbg(&s3cdma->pdev->dev, "prepare memcpy of %zu bytes from %s\n",
			len, s3cchan->name);

	if ((len & s3c2440_DCON_TC_MASK) != len) {
		dev_err(&s3cdma->pdev->dev, "memcpy size %zu to large\n", len);
		return NULL;
	}

	txd = s3c2440_dma_get_txd();
	if (!txd)
		return NULL;

	dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
	if (!dsg) {
		s3c2440_dma_free_txd(txd);
		return NULL;
	}
	list_add_tail(&dsg->node, &txd->dsg_list);

	dsg->src_addr = src;
	dsg->dst_addr = dest;
	dsg->len = len;

	/*
	 * Determine a suitable transfer width.
	 * The DMA controller cannot fetch/store information which is not
	 * naturally aligned on the bus, i.e., a 4 byte fetch must start at
	 * an address divisible by 4 - more generally addr % width must be 0.
	 */
	src_mod = src % 4;
	dest_mod = dest % 4;
	switch (len % 4) {
	case 0:
		txd->width = (src_mod == 0 && dest_mod == 0) ? 4 : 1;
		break;
	case 2:
		txd->width = ((src_mod == 2 || src_mod == 0) &&
			      (dest_mod == 2 || dest_mod == 0)) ? 2 : 1;
		break;
	default:
		txd->width = 1;
		break;
	}

	txd->disrcc = s3c2440_DISRCC_LOC_AHB | s3c2440_DISRCC_INC_INCREMENT;
	txd->didstc = s3c2440_DIDSTC_LOC_AHB | s3c2440_DIDSTC_INC_INCREMENT;
	txd->dcon |= s3c2440_DCON_DEMAND | s3c2440_DCON_SYNC_HCLK |
		     s3c2440_DCON_SERV_WHOLE;

	return vchan_tx_prep(&s3cchan->vc, &txd->vd, flags);
}

static struct dma_async_tx_descriptor *s3c2440_dma_prep_dma_cyclic(
	struct dma_chan *chan, dma_addr_t addr, size_t size, size_t period,
	enum dma_transfer_direction direction, unsigned long flags)
{
	struct s3c2440_dma_chan *s3cchan = to_s3c2440_dma_chan(chan);
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	const struct s3c24xx_dma_platdata *pdata = s3cdma->pdata;
	struct s3c24xx_dma_channel *cdata = &pdata->channels[s3cchan->id];
	struct s3c2440_txd *txd;
	struct s3c2440_sg *dsg;
	unsigned sg_len;
	dma_addr_t slave_addr;
	u32 hwcfg = 0;
	int i;

	dev_dbg(&s3cdma->pdev->dev,
		"prepare cyclic transaction of %zu bytes with period %zu from %s\n",
		size, period, s3cchan->name);

	if (!is_slave_direction(direction)) {
		dev_err(&s3cdma->pdev->dev,
			"direction %d unsupported\n", direction);
		return NULL;
	}

	txd = s3c2440_dma_get_txd();
	if (!txd)
		return NULL;

	txd->cyclic = 1;

	if (cdata->handshake)
		txd->dcon |= s3c2440_DCON_HANDSHAKE;

	switch (cdata->bus) {
	case S3C24XX_DMA_APB:
		txd->dcon |= s3c2440_DCON_SYNC_PCLK;
		hwcfg |= s3c2440_DISRCC_LOC_APB;
		break;
	case S3C24XX_DMA_AHB:
		txd->dcon |= s3c2440_DCON_SYNC_HCLK;
		hwcfg |= s3c2440_DISRCC_LOC_AHB;
		break;
	}

	/*
	 * Always assume our peripheral desintation is a fixed
	 * address in memory.
	 */
	hwcfg |= s3c2440_DISRCC_INC_FIXED;

	/*
	 * Individual dma operations are requested by the slave,
	 * so serve only single atomic operations (s3c2440_DCON_SERV_SINGLE).
	 */
	txd->dcon |= s3c2440_DCON_SERV_SINGLE;

	if (direction == DMA_MEM_TO_DEV) {
		txd->disrcc = s3c2440_DISRCC_LOC_AHB |
			      s3c2440_DISRCC_INC_INCREMENT;
		txd->didstc = hwcfg;
		slave_addr = s3cchan->cfg.dst_addr;
		txd->width = s3cchan->cfg.dst_addr_width;
	} else {
		txd->disrcc = hwcfg;
		txd->didstc = s3c2440_DIDSTC_LOC_AHB |
			      s3c2440_DIDSTC_INC_INCREMENT;
		slave_addr = s3cchan->cfg.src_addr;
		txd->width = s3cchan->cfg.src_addr_width;
	}

	sg_len = size / period;

	for (i = 0; i < sg_len; i++) {
		dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
		if (!dsg) {
			s3c2440_dma_free_txd(txd);
			return NULL;
		}
		list_add_tail(&dsg->node, &txd->dsg_list);

		dsg->len = period;
		/* Check last period length */
		if (i == sg_len - 1)
			dsg->len = size - period * i;
		if (direction == DMA_MEM_TO_DEV) {
			dsg->src_addr = addr + period * i;
			dsg->dst_addr = slave_addr;
		} else { /* DMA_DEV_TO_MEM */
			dsg->src_addr = slave_addr;
			dsg->dst_addr = addr + period * i;
		}
	}

	return vchan_tx_prep(&s3cchan->vc, &txd->vd, flags);
}

static struct dma_async_tx_descriptor *s3c2440_dma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct s3c2440_dma_chan *s3cchan = to_s3c2440_dma_chan(chan);
	struct s3c2440_dma_engine *s3cdma = s3cchan->host;
	const struct s3c24xx_dma_platdata *pdata = s3cdma->pdata;
	struct s3c24xx_dma_channel *cdata = &pdata->channels[s3cchan->id];
	struct s3c2440_txd *txd;
	struct s3c2440_sg *dsg;
	struct scatterlist *sg;
	dma_addr_t slave_addr;
	u32 hwcfg = 0;
	int tmp;

	dev_dbg(&s3cdma->pdev->dev, "prepare transaction of %d bytes from %s\n",
			sg_dma_len(sgl), s3cchan->name);

	if (!is_slave_direction(direction)) {
		dev_err(&s3cdma->pdev->dev,
			"direction %d unsupported\n", direction);
		return NULL;
	}

	txd = s3c2440_dma_get_txd();
	if (!txd)
		return NULL;

	if (cdata->handshake)
		txd->dcon |= s3c2440_DCON_HANDSHAKE;

	switch (cdata->bus) {
	case S3C24XX_DMA_APB:
		txd->dcon |= s3c2440_DCON_SYNC_PCLK;
		hwcfg |= s3c2440_DISRCC_LOC_APB;
		break;
	case S3C24XX_DMA_AHB:
		txd->dcon |= s3c2440_DCON_SYNC_HCLK;
		hwcfg |= s3c2440_DISRCC_LOC_AHB;
		break;
	}

	/*
	 * Always assume our peripheral desintation is a fixed
	 * address in memory.
	 */
	hwcfg |= s3c2440_DISRCC_INC_FIXED;

	/*
	 * Individual dma operations are requested by the slave,
	 * so serve only single atomic operations (s3c2440_DCON_SERV_SINGLE).
	 */
	txd->dcon |= s3c2440_DCON_SERV_SINGLE;

	if (direction == DMA_MEM_TO_DEV) {
		txd->disrcc = s3c2440_DISRCC_LOC_AHB |
			      s3c2440_DISRCC_INC_INCREMENT;
		txd->didstc = hwcfg;
		slave_addr = s3cchan->cfg.dst_addr;
		txd->width = s3cchan->cfg.dst_addr_width;
	} else if (direction == DMA_DEV_TO_MEM) {
		txd->disrcc = hwcfg;
		txd->didstc = s3c2440_DIDSTC_LOC_AHB |
			      s3c2440_DIDSTC_INC_INCREMENT;
		slave_addr = s3cchan->cfg.src_addr;
		txd->width = s3cchan->cfg.src_addr_width;
	}

	for_each_sg(sgl, sg, sg_len, tmp) {
		dsg = kzalloc(sizeof(*dsg), GFP_NOWAIT);
		if (!dsg) {
			s3c2440_dma_free_txd(txd);
			return NULL;
		}
		list_add_tail(&dsg->node, &txd->dsg_list);

		dsg->len = sg_dma_len(sg);
		if (direction == DMA_MEM_TO_DEV) {
			dsg->src_addr = sg_dma_address(sg);
			dsg->dst_addr = slave_addr;
		} else { /* DMA_DEV_TO_MEM */
			dsg->src_addr = slave_addr;
			dsg->dst_addr = sg_dma_address(sg);
		}
	}

	return vchan_tx_prep(&s3cchan->vc, &txd->vd, flags);
}

/*
 * Slave transactions callback to the slave device to allow
 * synchronization of slave DMA signals with the DMAC enable
 */
static void s3c2440_dma_issue_pending(struct dma_chan *chan)
{
	struct s3c2440_dma_chan *s3cchan = to_s3c2440_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&s3cchan->vc.lock, flags);
	if (vchan_issue_pending(&s3cchan->vc)) {
		if (!s3cchan->phy && s3cchan->state != s3c2440_DMA_CHAN_WAITING)
			s3c2440_dma_phy_alloc_and_start(s3cchan);
	}
	spin_unlock_irqrestore(&s3cchan->vc.lock, flags);
}

/*
 * Bringup and teardown
 */

/*
 * Initialise the DMAC slave channels.
 * Make a local wrapper to hold required data
 */
static int s3c2440_dma_init_virtual_channels(struct s3c2440_dma_engine *s3cdma,
		struct dma_device *dmadev, unsigned int channels)
{
	struct s3c2440_dma_chan *chan;
	int i;

	INIT_LIST_HEAD(&dmadev->channels);

	/*
	 * Register as many many memcpy as we have physical channels,
	 * we won't always be able to use all but the code will have
	 * to cope with that situation.
	 */
	for (i = 0; i < channels; i++) {
		chan = devm_kzalloc(dmadev->dev, sizeof(*chan), GFP_KERNEL);
		if (!chan)
			return -ENOMEM;

		chan->id = i;
		chan->host = s3cdma;
		chan->state = s3c2440_DMA_CHAN_IDLE;

		chan->name = kasprintf(GFP_KERNEL, "slave%d", i);
		if (!chan->name)
			return -ENOMEM;

		dev_dbg(dmadev->dev,
			 "initialize virtual channel \"%s\"\n",
			 chan->name);

		chan->vc.desc_free = s3c2440_dma_desc_free;
		vchan_init(&chan->vc, dmadev);
	}
	dev_info(dmadev->dev, "initialized %d virtual slave channels\n",
		 i);
	return i;
}

static void s3c2440_dma_free_virtual_channels(struct dma_device *dmadev)
{
	struct s3c2440_dma_chan *chan = NULL;
	struct s3c2440_dma_chan *next;

	list_for_each_entry_safe(chan,
				 next, &dmadev->channels, vc.chan.device_node) {
		list_del(&chan->vc.chan.device_node);
		tasklet_kill(&chan->vc.task);
	}
}

static bool s3c2440_dma_filter(struct dma_chan *chan, void *param)
{
	struct s3c2440_dma_chan *s3cchan;
	int ch = *(uintptr_t *)param;

	s3cchan = to_s3c2440_dma_chan(chan);

	return s3cchan->id == ch;
}

static struct of_dma_filter_info s3c2440_dma_info = {
	.filter_fn = s3c2440_dma_filter,
};

#define S3C2440_DMA_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
				 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

extern int s3c2440_dma_pdev_fix(struct platform_device *pdev);
static int s3c2440_dma_probe(struct platform_device *pdev)
{
	const struct s3c24xx_dma_platdata *pdata;
	struct s3c2440_dma_engine *s3cdma;
	struct resource *res;
	int ret;
	int i;

	s3c2440_dma_pdev_fix(pdev);
	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	/* Basic sanity check */
	if (pdata->num_phy_channels > MAX_DMA_CHANNELS) {
		dev_err(&pdev->dev, "to many dma channels %d, max %d\n",
			pdata->num_phy_channels, MAX_DMA_CHANNELS);
		return -EINVAL;
	}

	s3cdma = devm_kzalloc(&pdev->dev, sizeof(*s3cdma), GFP_KERNEL);
	if (!s3cdma)
		return -ENOMEM;

	s3cdma->pdev = pdev;
	s3cdma->pdata = pdata;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	s3cdma->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(s3cdma->base))
		return PTR_ERR(s3cdma->base);

	s3cdma->phy_chans = devm_kzalloc(&pdev->dev,
					      sizeof(struct s3c2440_dma_phy) *
							pdata->num_phy_channels,
					      GFP_KERNEL);
	if (!s3cdma->phy_chans)
		return -ENOMEM;

	/* acquire irqs and clocks for all physical channels */
	for (i = 0; i < pdata->num_phy_channels; i++) {
		struct s3c2440_dma_phy *phy = &s3cdma->phy_chans[i];

		phy->id = i;
		phy->base = s3cdma->base + (i * 0x40);  // stride is 0x40
		phy->host = s3cdma;

		phy->irq = platform_get_irq(pdev, i);
		if (phy->irq < 0) {
			dev_err(&pdev->dev, "failed to get irq %d, err %d\n",
				i, phy->irq);
			continue;
		}

		ret = devm_request_irq(&pdev->dev, phy->irq, s3c2440_dma_irq,
				       0, pdev->name, phy);
		if (ret) {
			dev_err(&pdev->dev, "Unable to request irq for channel %d, error %d\n",
				i, ret);
			continue;
		}

		spin_lock_init(&phy->lock);
		phy->valid = true;

		dev_dbg(&pdev->dev, "physical channel %d is %s\n",
			i, s3c2440_dma_phy_busy(phy) ? "BUSY" : "FREE");
	}

	/* Initialize slave engine for SoC internal dedicated peripherals */
	dma_cap_set(DMA_SLAVE, s3cdma->slave.cap_mask);
	dma_cap_set(DMA_MEMCPY, s3cdma->slave.cap_mask);
	dma_cap_set(DMA_CYCLIC, s3cdma->slave.cap_mask);
	dma_cap_set(DMA_PRIVATE, s3cdma->slave.cap_mask);
	s3cdma->slave.dev = &pdev->dev;
	s3cdma->slave.device_free_chan_resources =
					s3c2440_dma_free_chan_resources;
	s3cdma->slave.device_tx_status = s3c2440_dma_tx_status;
	s3cdma->slave.device_issue_pending = s3c2440_dma_issue_pending;
	s3cdma->slave.device_prep_slave_sg = s3c2440_dma_prep_slave_sg;
	s3cdma->slave.device_prep_dma_memcpy = s3c2440_dma_prep_memcpy;
	s3cdma->slave.device_prep_dma_cyclic = s3c2440_dma_prep_dma_cyclic;
	s3cdma->slave.device_config = s3c2440_dma_set_runtime_config;
	s3cdma->slave.device_terminate_all = s3c2440_dma_terminate_all;
	s3cdma->slave.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) | (BIT(DMA_MEM_TO_MEM));
	s3cdma->slave.src_addr_widths = S3C2440_DMA_BUSWIDTHS;
	s3cdma->slave.dst_addr_widths = S3C2440_DMA_BUSWIDTHS;

	s3cdma->slave.filter.map = pdata->slave_map;
	s3cdma->slave.filter.mapcnt = pdata->slavecnt;
	s3cdma->slave.filter.fn = s3c2440_dma_filter;

	/* Register slave channels */
	ret = s3c2440_dma_init_virtual_channels(s3cdma, &s3cdma->slave,
				pdata->num_channels);
	if (ret <= 0) {
		dev_warn(&pdev->dev,
			"%s failed to enumerate slave channels - %d\n",
				__func__, ret);
		return ret;
	}

	ret = dma_async_device_register(&s3cdma->slave);
	if (ret) {
		dev_warn(&pdev->dev,
			"%s failed to register slave as an async device - %d\n",
			__func__, ret);
		goto err_slave_reg;
	}

	s3c2440_dma_info.dma_cap = s3cdma->slave.cap_mask;
	ret = of_dma_controller_register(pdev->dev.of_node,
				of_dma_simple_xlate, &s3c2440_dma_info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register DMA controller\n");
		goto err_of_dma_reg;
	}

	platform_set_drvdata(pdev, s3cdma);
	dev_info(&pdev->dev, "Loaded dma driver with %d physical channels\n",
		 pdata->num_phy_channels);

	return 0;

err_of_dma_reg:
	dma_async_device_unregister(&s3cdma->slave);
err_slave_reg:
	s3c2440_dma_free_virtual_channels(&s3cdma->slave);

	return ret;
}

static void s3c2440_dma_free_irq(struct platform_device *pdev,
				struct s3c2440_dma_engine *s3cdma)
{
	int i;

	for (i = 0; i < s3cdma->pdata->num_phy_channels; i++) {
		struct s3c2440_dma_phy *phy = &s3cdma->phy_chans[i];

		devm_free_irq(&pdev->dev, phy->irq, phy);
	}
}

static int s3c2440_dma_remove(struct platform_device *pdev)
{
	struct s3c2440_dma_engine *s3cdma = platform_get_drvdata(pdev);

	dma_async_device_unregister(&s3cdma->slave);

	s3c2440_dma_free_irq(pdev, s3cdma);

	s3c2440_dma_free_virtual_channels(&s3cdma->slave);

	return 0;
}

static const struct of_device_id s3c2440_dma_dt_ids[] = {
	{
		.compatible = "tq2440, s3c2440-dma",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, s3c2440_dma_dt_ids);

static struct platform_driver s3c2440_dma_driver = {
	.driver		= {
		.name	= "s3c2440-dma",
		.of_match_table = of_match_ptr(s3c2440_dma_dt_ids),
	},
	.probe		= s3c2440_dma_probe,
	.remove		= s3c2440_dma_remove,
};

module_platform_driver(s3c2440_dma_driver);

MODULE_DESCRIPTION("s3c2440 DMA Driver");
MODULE_LICENSE("GPL v2");
