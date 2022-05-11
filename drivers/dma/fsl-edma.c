// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * drivers/dma/fsl-edma.c
 *
 * Copyright 2013-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018, 2020-2021 NXP
 *
 * Driver for the Freescale eDMA engine with flexible channel multiplexing
 * capability for DMA request sources. The eDMA block can be found on some
 * Vybrid, Layerscape and S32GEN1 SoCs.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>

#include "fsl-edma-common.h"

#define EDMA_TCD(ch)	(0x1000 + 32 * (ch))

static int is_vf610_edma(struct fsl_edma_engine *data);
static int is_s32gen1_edma(struct fsl_edma_engine *data);

static void fsl_edma_synchronize(struct dma_chan *chan)
{
	struct fsl_edma_chan *fsl_chan = to_fsl_edma_chan(chan);

	vchan_synchronize(&fsl_chan->vchan);
}

static void fsl_edma3_enable_request(struct fsl_edma_chan *fsl_chan)
{
	void __iomem *addr = fsl_chan->edma->membase;
	u32 ch = fsl_chan->vchan.chan.chan_id;

	edma_writel(fsl_chan->edma, EDMA3_CHn_CSR_ERQ | EDMA3_CHn_CSR_EEI,
			addr + EDMA3_CHn_CSR(ch));
}

static void fsl_edma3_disable_request(struct fsl_edma_chan *fsl_chan)
{
	void __iomem *addr = fsl_chan->edma->membase;
	u32 ch = fsl_chan->vchan.chan.chan_id;

	/* Make sure there is no hardware request in progress. */
	while (edma_readl(fsl_chan->edma, addr + EDMA3_MP_HRS)
		& EDMA3_MP_HRS_CH(ch))
		;
	edma_writel(fsl_chan->edma, 0, addr + EDMA3_CHn_CSR(ch));
}

static irqreturn_t fsl_edma_tx_handler(int irq, void *dev_id)
{
	struct fsl_edma_engine *fsl_edma = dev_id;
	unsigned int intr, ch;
	struct edma_regs *regs = &fsl_edma->regs;
	struct fsl_edma_chan *fsl_chan;

	intr = edma_readl(fsl_edma, regs->intl);
	if (!intr)
		return IRQ_NONE;

	for (ch = 0; ch < fsl_edma->n_chans; ch++) {
		if (intr & (0x1 << ch)) {
			edma_writeb(fsl_edma, EDMA_CINT_CINT(ch), regs->cint);

			fsl_chan = &fsl_edma->chans[ch];

			spin_lock(&fsl_chan->vchan.lock);

			if (!fsl_chan->edesc) {
				/* terminate_all called before */
				spin_unlock(&fsl_chan->vchan.lock);
				continue;
			}

			if (!fsl_chan->edesc->iscyclic) {
				list_del(&fsl_chan->edesc->vdesc.node);
				vchan_cookie_complete(&fsl_chan->edesc->vdesc);
				fsl_chan->edesc = NULL;
				fsl_chan->status = DMA_COMPLETE;
				fsl_chan->idle = true;
			} else {
				vchan_cyclic_callback(&fsl_chan->edesc->vdesc);
			}

			if (!fsl_chan->edesc)
				fsl_edma_xfer_desc(fsl_chan);

			spin_unlock(&fsl_chan->vchan.lock);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t fsl_edma3_tx_handler(int irq, void *dev_id)
{
	struct fsl_edma_engine *fsl_edma = dev_id;
	unsigned int ch, ch_int;
	void __iomem *base_addr;
	struct fsl_edma_chan *fsl_chan;
	bool handled = false;

	base_addr = fsl_edma->membase;

	for (ch = 0; ch < fsl_edma->n_chans; ch++) {
		ch_int = edma_readl(fsl_edma,
				fsl_edma->membase + EDMA3_CHn_INT(ch));
		if (ch_int & EDMA3_CHn_INT_INT) {
			handled = true;
			edma_writel(fsl_edma, EDMA3_CHn_INT_INT,
				base_addr + EDMA3_CHn_INT(ch));

			fsl_chan = &fsl_edma->chans[ch];

			spin_lock(&fsl_chan->vchan.lock);
			/* It is possible that fsl_edma_terminate_all() has
			 * canceled transfers on this channel
			 */
			if (!fsl_chan->edesc) {
				spin_unlock(&fsl_chan->vchan.lock);
				continue;
			}

			if (!fsl_chan->edesc->iscyclic) {
				list_del(&fsl_chan->edesc->vdesc.node);
				vchan_cookie_complete(&fsl_chan->edesc->vdesc);
				fsl_chan->edesc = NULL;
				fsl_chan->status = DMA_COMPLETE;
			} else {
				vchan_cyclic_callback(&fsl_chan->edesc->vdesc);
			}

			if (!fsl_chan->edesc)
				fsl_edma_xfer_desc(fsl_chan);

			spin_unlock(&fsl_chan->vchan.lock);
		}
	}
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t fsl_edma_err_handler(int irq, void *dev_id)
{
	struct fsl_edma_engine *fsl_edma = dev_id;
	unsigned int err, ch;
	struct edma_regs *regs = &fsl_edma->regs;

	err = edma_readl(fsl_edma, regs->errl);
	if (!err)
		return IRQ_NONE;

	for (ch = 0; ch < fsl_edma->n_chans; ch++) {
		if (err & (0x1 << ch)) {
			fsl_edma_disable_request(&fsl_edma->chans[ch]);
			edma_writeb(fsl_edma, EDMA_CERR_CERR(ch), regs->cerr);
			fsl_edma->chans[ch].status = DMA_ERROR;
			fsl_edma->chans[ch].idle = true;
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t fsl_edma3_err_handler(int irq, void *dev_id)
{
	struct fsl_edma_engine *fsl_edma = dev_id;
	unsigned int err, ch, ch_es;

	err = edma_readl(fsl_edma, fsl_edma->membase + EDMA3_MP_ES);
	if (!EDMA3_MP_ES_VLD(err))
		return IRQ_NONE;

	for (ch = 0; ch < fsl_edma->n_chans; ch++) {
		ch_es = edma_readl(fsl_edma,
				fsl_edma->membase + EDMA3_CHn_ES(ch));
		if (ch_es & EDMA3_CHn_ES_ERR) {
			fsl_edma3_disable_request(&fsl_edma->chans[ch]);
			edma_writel(fsl_edma, EDMA3_CHn_ES_ERR,
				fsl_edma->membase + EDMA3_CHn_ES(ch));
			fsl_edma->chans[ch].status = DMA_ERROR;
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t fsl_edma_irq_handler(int irq, void *dev_id)
{
	if (fsl_edma_tx_handler(irq, dev_id) == IRQ_HANDLED)
		return IRQ_HANDLED;

	return fsl_edma_err_handler(irq, dev_id);
}

static irqreturn_t fsl_edma3_irq_handler(int irq, void *dev_id)
{
	if (fsl_edma3_tx_handler(irq, dev_id) == IRQ_HANDLED)
		return IRQ_HANDLED;

	return fsl_edma3_err_handler(irq, dev_id);
}

static struct dma_chan *fsl_edma_xlate(struct of_phandle_args *dma_spec,
		struct of_dma *ofdma)
{
	struct fsl_edma_engine *fsl_edma = ofdma->of_dma_data;
	struct dma_chan *chan, *_chan;
	struct fsl_edma_chan *fsl_chan;
	u32 dmamux_nr = fsl_edma->drvdata->dmamuxs;
	unsigned long chans_per_mux = fsl_edma->n_chans / dmamux_nr;

	if (dma_spec->args_count != 2)
		return NULL;

	mutex_lock(&fsl_edma->fsl_edma_mutex);
	list_for_each_entry_safe(chan, _chan, &fsl_edma->dma_dev.channels, device_node) {
		if (chan->client_count)
			continue;
		if ((chan->chan_id / chans_per_mux) == dma_spec->args[0]) {
			chan = dma_get_slave_channel(chan);
			if (chan) {
				chan->device->privatecnt++;
				fsl_chan = to_fsl_edma_chan(chan);
				fsl_chan->slave_id = dma_spec->args[1];
				fsl_edma_chan_mux(fsl_chan, fsl_chan->slave_id,
						true);
				mutex_unlock(&fsl_edma->fsl_edma_mutex);
				return chan;
			}
		}
	}
	mutex_unlock(&fsl_edma->fsl_edma_mutex);
	return NULL;
}

static int
fsl_edma_irq_init(struct platform_device *pdev, struct fsl_edma_engine *fsl_edma)
{
	int ret;
	unsigned int i, j;
	const struct fsl_edma_drvdata *drvdata = fsl_edma->drvdata;

	fsl_edma->irq_nos = devm_kzalloc(&pdev->dev,
			drvdata->n_irqs * sizeof(*fsl_edma->irq_nos), GFP_KERNEL);
	if (!fsl_edma->irq_nos)
		return -ENOMEM;

	for (i = 0; i < drvdata->n_irqs; i++) {
		fsl_edma->irq_nos[i] = platform_get_irq_byname(pdev,
				drvdata->irqs[i].name);
		if (fsl_edma->irq_nos[i] < 0) {
			dev_err(&pdev->dev, "Can't get %s irq.\n",
					drvdata->irqs[i].name);
			return fsl_edma->irq_nos[i];
		}

		for (j = 0; j < i; j++) {
			if (fsl_edma->irq_nos[i] == fsl_edma->irq_nos[j])
				break;
		}

		/* Check there is a irq with multiple functionalities */
		if (is_vf610_edma(fsl_edma))
			if (j < i) {
				fsl_edma->irq_nos[i] = -1;
				drvdata->irqs[j].name = "eDma";
			}
	}

	for (i = 0; i < drvdata->n_irqs; i++) {
		if (fsl_edma->irq_nos[i] >= 0) {
			ret = devm_request_irq(&pdev->dev,
					fsl_edma->irq_nos[i],
					drvdata->irqs[i].irqhandler,
					0,
					drvdata->irqs[i].name,
					fsl_edma);
			if (ret) {
				dev_err(&pdev->dev,
						"Can't register %s IRQ.\n",
						drvdata->irqs[i].name);
				return ret;
			}
		}
	}

	return 0;
}

static int
fsl_edma2_irq_init(struct platform_device *pdev,
		   struct fsl_edma_engine *fsl_edma)
{
	int i, ret, irq;
	int count;

	count = platform_irq_count(pdev);
	dev_dbg(&pdev->dev, "%s Found %d interrupts\r\n", __func__, count);
	if (count <= 2) {
		dev_err(&pdev->dev, "Interrupts in DTS not correct.\n");
		return -EINVAL;
	}
	/*
	 * 16 channel independent interrupts + 1 error interrupt on i.mx7ulp.
	 * 2 channel share one interrupt, for example, ch0/ch16, ch1/ch17...
	 * For now, just simply request irq without IRQF_SHARED flag, since 16
	 * channels are enough on i.mx7ulp whose M4 domain own some peripherals.
	 */
	for (i = 0; i < count; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return -ENXIO;

		sprintf(fsl_edma->chans[i].chan_name, "eDMA2-CH%02d", i);

		/* The last IRQ is for eDMA err */
		if (i == count - 1)
			ret = devm_request_irq(&pdev->dev, irq,
						fsl_edma_err_handler,
						0, "eDMA2-ERR", fsl_edma);
		else
			ret = devm_request_irq(&pdev->dev, irq,
						fsl_edma_tx_handler, 0,
						fsl_edma->chans[i].chan_name,
						fsl_edma);
		if (ret)
			return ret;
	}

	return 0;
}

static unsigned int s32gen1_mux_channel_mapping(u32 channel_id)
{
	return 4 * (channel_id/4) + ((4 - channel_id % 4) - 1);
}

static unsigned int vf610_mux_channel_mapping(u32 channel_id)
{
	return channel_id;
}

static void fsl_edma_irq_exit(
		struct platform_device *pdev, struct fsl_edma_engine *fsl_edma)
{
	unsigned int i;
	const struct fsl_edma_drvdata *drvdata = fsl_edma->drvdata;

	for (i = 0; i < drvdata->n_irqs; i++) {
		if (fsl_edma->irq_nos[i] >= 0)
			devm_free_irq(&pdev->dev,
					fsl_edma->irq_nos[i], fsl_edma);
	}
}

static void fsl_disable_clocks(struct fsl_edma_engine *fsl_edma, int nr_clocks)
{
	int i;

	for (i = 0; i < nr_clocks; i++)
		clk_disable_unprepare(fsl_edma->muxclk[i]);
}

static void fsl_edma_enable_arbitration(struct fsl_edma_engine *fsl_edma)
{
	void __iomem *addr = fsl_edma->membase;

	edma_writel(fsl_edma, EDMA_CR_ERGA | EDMA_CR_ERCA, addr + EDMA_CR);
}

static void fsl_edma3_enable_arbitration(struct fsl_edma_engine *fsl_edma)
{
	void __iomem *addr = fsl_edma->membase;

	edma_writel(fsl_edma, EDMA3_MP_CSR_ERCA, addr + EDMA3_MP_CSR);
}

static void __iomem *fsl_edma_get_tcd_addr(struct fsl_edma_chan *fsl_chan)
{
	void __iomem *membase = fsl_chan->edma->membase;
	u32 ch = fsl_chan->vchan.chan.chan_id;

	return (EDMA_TCD(ch) + membase);
}

static void __iomem *fsl_edma3_get_tcd_addr(struct fsl_edma_chan *fsl_chan)
{
	void __iomem *membase = fsl_chan->edma->membase;
	u32 ch = fsl_chan->vchan.chan.chan_id;

	return (EDMA3_TCD(ch) + membase);
}

static struct fsl_edma_irq s32gen1_edma_irqs[] = {
	{"edma-err", fsl_edma3_irq_handler, },
	{"edma-tx_0-15", fsl_edma3_tx_handler, },
	{"edma-tx_16-31", fsl_edma3_tx_handler, },
};

static struct fsl_edma_irq vf610_edma_irqs[] = {
	{"edma-err", fsl_edma_irq_handler, },
	{"edma-tx", fsl_edma_tx_handler, },
};

static struct fsl_edma_ops fsl_edma_ops = {
	.edma_enable_request = fsl_edma_enable_request,
	.edma_disable_request = fsl_edma_disable_request,
	.edma_enable_arbitration = fsl_edma_enable_arbitration,
	.edma_get_tcd_addr = fsl_edma_get_tcd_addr,
};

static struct fsl_edma_ops fsl_edma3_ops = {
	.edma_enable_request = fsl_edma3_enable_request,
	.edma_disable_request = fsl_edma3_disable_request,
	.edma_enable_arbitration = fsl_edma3_enable_arbitration,
	.edma_get_tcd_addr = fsl_edma3_get_tcd_addr,
};

static struct fsl_edma_drvdata s32gen1_data = {
	.version = v1,
	.dmamuxs = DMAMUX_NR,
	.n_irqs = ARRAY_SIZE(s32gen1_edma_irqs),
	.irqs = s32gen1_edma_irqs,
	.mux_channel_mapping = s32gen1_mux_channel_mapping,
	.ops = &fsl_edma3_ops,
};

static struct fsl_edma_drvdata vf610_data = {
	.version = v1,
	.dmamuxs = DMAMUX_NR,
	.n_irqs = ARRAY_SIZE(vf610_edma_irqs),
	.irqs = vf610_edma_irqs,
	.mux_channel_mapping = vf610_mux_channel_mapping,
	.ops = &fsl_edma_ops,
};

static struct fsl_edma_drvdata ls1028a_data = {
	.version = v1,
	.dmamuxs = DMAMUX_NR,
	.mux_swap = true,
	.setup_irq = fsl_edma_irq_init,
};

static struct fsl_edma_drvdata imx7ulp_data = {
	.version = v3,
	.dmamuxs = 1,
	.has_dmaclk = true,
	.setup_irq = fsl_edma2_irq_init,
};

static const struct of_device_id fsl_edma_dt_ids[] = {
	{ .compatible = "nxp,s32cc-edma", .data = &s32gen1_data},
	{ .compatible = "fsl,vf610-edma", .data = &vf610_data},
	{ .compatible = "fsl,ls1028a-edma", .data = &ls1028a_data},
	{ .compatible = "fsl,imx7ulp-edma", .data = &imx7ulp_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_edma_dt_ids);

static inline int is_s32gen1_edma(struct fsl_edma_engine *data)
{
	return data->drvdata == &s32gen1_data;
}

static inline int is_vf610_edma(struct fsl_edma_engine *data)
{
	return data->drvdata == &vf610_data;
}

static int fsl_edma_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(fsl_edma_dt_ids, &pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct fsl_edma_engine *fsl_edma;
	const struct fsl_edma_drvdata *drvdata = NULL;
	struct fsl_edma_chan *fsl_chan;
	struct fsl_edma_hw_tcd __iomem *hw_tcd;
	struct edma_regs *regs;
	struct resource *res;
	unsigned int ch;
	int len, chans;
	int ret, i;

	if (of_id)
		drvdata = of_id->data;
	if (!drvdata) {
		dev_err(&pdev->dev, "unable to find driver data\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "dma-channels", &chans);
	if (ret) {
		dev_err(&pdev->dev, "Can't get dma-channels.\n");
		return ret;
	}

	len = sizeof(*fsl_edma) + sizeof(*fsl_chan) * chans;
	fsl_edma = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!fsl_edma)
		return -ENOMEM;

	fsl_edma->drvdata = drvdata;
	fsl_edma->n_chans = chans;
	mutex_init(&fsl_edma->fsl_edma_mutex);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fsl_edma->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fsl_edma->membase))
		return PTR_ERR(fsl_edma->membase);

	fsl_edma_setup_regs(fsl_edma);
	regs = &fsl_edma->regs;

	if (drvdata->has_dmaclk) {
		fsl_edma->dmaclk = devm_clk_get(&pdev->dev, "dma");
		if (IS_ERR(fsl_edma->dmaclk)) {
			dev_err(&pdev->dev, "Missing DMA block clock.\n");
			return PTR_ERR(fsl_edma->dmaclk);
		}

		ret = clk_prepare_enable(fsl_edma->dmaclk);
		if (ret) {
			dev_err(&pdev->dev, "DMA clk block failed.\n");
			return ret;
		}
	}

	for (i = 0; i < fsl_edma->drvdata->dmamuxs; i++) {
		char clkname[32];

		res = platform_get_resource(pdev, IORESOURCE_MEM, 1 + i);
		fsl_edma->muxbase[i] = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(fsl_edma->muxbase[i])) {
			/* on error: disable all previously enabled clks */
			fsl_disable_clocks(fsl_edma, i);
			return PTR_ERR(fsl_edma->muxbase[i]);
		}

		sprintf(clkname, "dmamux%d", i);
		fsl_edma->muxclk[i] = devm_clk_get(&pdev->dev, clkname);
		if (IS_ERR(fsl_edma->muxclk[i])) {
			dev_err(&pdev->dev, "Missing DMAMUX block clock.\n");
			/* on error: disable all previously enabled clks */
			fsl_disable_clocks(fsl_edma, i);
			return PTR_ERR(fsl_edma->muxclk[i]);
		}

		ret = clk_prepare_enable(fsl_edma->muxclk[i]);
		if (ret)
			/* on error: disable all previously enabled clks */
			fsl_disable_clocks(fsl_edma, i);

	}

	fsl_edma->big_endian = of_property_read_bool(np, "big-endian");

	INIT_LIST_HEAD(&fsl_edma->dma_dev.channels);
	for (i = 0; i < fsl_edma->n_chans; i++) {
		struct fsl_edma_chan *fsl_chan = &fsl_edma->chans[i];

		fsl_chan->edma = fsl_edma;
		fsl_chan->pm_state = RUNNING;
		fsl_chan->slave_id = 0;
		fsl_chan->idle = true;
		fsl_chan->dma_dir = DMA_NONE;
		fsl_chan->vchan.desc_free = fsl_edma_free_desc;
		vchan_init(&fsl_chan->vchan, &fsl_edma->dma_dev);
	}

	dma_cap_set(DMA_PRIVATE, fsl_edma->dma_dev.cap_mask);
	dma_cap_set(DMA_SLAVE, fsl_edma->dma_dev.cap_mask);
	dma_cap_set(DMA_CYCLIC, fsl_edma->dma_dev.cap_mask);

	fsl_edma->dma_dev.dev = &pdev->dev;
	fsl_edma->dma_dev.device_alloc_chan_resources
		= fsl_edma_alloc_chan_resources;
	fsl_edma->dma_dev.device_free_chan_resources
		= fsl_edma_free_chan_resources;
	fsl_edma->dma_dev.device_tx_status = fsl_edma_tx_status;
	fsl_edma->dma_dev.device_prep_slave_sg = fsl_edma_prep_slave_sg;
	fsl_edma->dma_dev.device_prep_dma_cyclic = fsl_edma_prep_dma_cyclic;
	fsl_edma->dma_dev.device_config = fsl_edma_slave_config;
	fsl_edma->dma_dev.device_pause = fsl_edma_pause;
	fsl_edma->dma_dev.device_resume = fsl_edma_resume;
	fsl_edma->dma_dev.device_terminate_all = fsl_edma_terminate_all;
	fsl_edma->dma_dev.device_synchronize = fsl_edma_synchronize;
	fsl_edma->dma_dev.device_issue_pending = fsl_edma_issue_pending;

	fsl_edma->dma_dev.src_addr_widths = FSL_EDMA_BUSWIDTHS;
	fsl_edma->dma_dev.dst_addr_widths = FSL_EDMA_BUSWIDTHS;
	fsl_edma->dma_dev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);

	platform_set_drvdata(pdev, fsl_edma);

	ret = dma_async_device_register(&fsl_edma->dma_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't register Freescale eDMA engine. (%d)\n", ret);
		fsl_disable_clocks(fsl_edma, fsl_edma->drvdata->dmamuxs);
		return ret;
	}

	for (i = 0; i < fsl_edma->n_chans; i++) {
		struct fsl_edma_chan *fsl_chan = &fsl_edma->chans[i];

		hw_tcd = (struct fsl_edma_hw_tcd __iomem *)
			fsl_edma->drvdata->ops->edma_get_tcd_addr(fsl_chan);

		edma_writew(fsl_edma, 0x0, &hw_tcd->csr);
		fsl_edma_chan_mux(fsl_chan, 0, false);
	}

	if (is_s32gen1_edma(fsl_edma))
		for (ch = 0; ch < fsl_edma->n_chans; ch++)
			edma_writel(fsl_edma, EDMA3_CHn_INT_INT,
					fsl_edma->membase + EDMA3_CHn_INT(ch));
	else
		edma_writel(fsl_edma, ~0, fsl_edma->membase + EDMA_INTR);

	ret = fsl_edma_irq_init(pdev, fsl_edma);
	if (ret)
		return ret;

	ret = of_dma_controller_register(np, fsl_edma_xlate, fsl_edma);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't register Freescale eDMA of_dma. (%d)\n", ret);
		dma_async_device_unregister(&fsl_edma->dma_dev);
		fsl_disable_clocks(fsl_edma, fsl_edma->drvdata->dmamuxs);
		return ret;
	}

	/* enable round robin arbitration */
	fsl_edma->drvdata->ops->edma_enable_arbitration(fsl_edma);

	return 0;
}

static int fsl_edma_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_edma_engine *fsl_edma = platform_get_drvdata(pdev);

	fsl_edma_irq_exit(pdev, fsl_edma);
	fsl_edma_cleanup_vchan(&fsl_edma->dma_dev);
	of_dma_controller_free(np);
	dma_async_device_unregister(&fsl_edma->dma_dev);
	fsl_disable_clocks(fsl_edma, fsl_edma->drvdata->dmamuxs);

	return 0;
}

static int fsl_edma_suspend_late(struct device *dev)
{
	struct fsl_edma_engine *fsl_edma = dev_get_drvdata(dev);
	struct fsl_edma_chan *fsl_chan;
	unsigned long flags;
	int i;

	for (i = 0; i < fsl_edma->n_chans; i++) {
		fsl_chan = &fsl_edma->chans[i];
		spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
		/* Make sure chan is idle or will force disable. */
		if (unlikely(!fsl_chan->idle)) {
			dev_warn(dev, "WARN: There is non-idle channel.");
			fsl_edma->drvdata->ops->edma_disable_request(fsl_chan);
			fsl_edma_chan_mux(fsl_chan, 0, false);
		}

		fsl_chan->pm_state = SUSPENDED;
		spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
	}

	return 0;
}

static int fsl_edma_resume_early(struct device *dev)
{
	struct fsl_edma_engine *fsl_edma = dev_get_drvdata(dev);
	struct fsl_edma_chan *fsl_chan;
	int i;

	for (i = 0; i < fsl_edma->n_chans; i++) {
		struct fsl_edma_hw_tcd __iomem *hw_tcd;

		fsl_chan = &fsl_edma->chans[i];
		hw_tcd = (struct fsl_edma_hw_tcd __iomem *)
			fsl_edma->drvdata->ops->edma_get_tcd_addr(fsl_chan);

		fsl_chan->pm_state = RUNNING;
		edma_writew(fsl_edma, 0x0, &hw_tcd->csr);
		if (fsl_chan->slave_id != 0)
			fsl_edma_chan_mux(fsl_chan, fsl_chan->slave_id, true);
	}

	fsl_edma->drvdata->ops->edma_enable_arbitration(fsl_edma);

	return 0;
}

/*
 * eDMA provides the service to others, so it should be suspend late
 * and resume early. When eDMA suspend, all of the clients should stop
 * the DMA data transmission and let the channel idle.
 */
static const struct dev_pm_ops fsl_edma_pm_ops = {
	.suspend_late   = fsl_edma_suspend_late,
	.resume_early   = fsl_edma_resume_early,
};

static struct platform_driver fsl_edma_driver = {
	.driver		= {
		.name	= "fsl-edma",
		.of_match_table = fsl_edma_dt_ids,
		.pm     = &fsl_edma_pm_ops,
	},
	.probe          = fsl_edma_probe,
	.remove		= fsl_edma_remove,
};

static int __init fsl_edma_init(void)
{
	return platform_driver_register(&fsl_edma_driver);
}
subsys_initcall(fsl_edma_init);

static void __exit fsl_edma_exit(void)
{
	platform_driver_unregister(&fsl_edma_driver);
}
module_exit(fsl_edma_exit);

MODULE_ALIAS("platform:fsl-edma");
MODULE_DESCRIPTION("Freescale eDMA engine driver");
MODULE_LICENSE("GPL v2");
