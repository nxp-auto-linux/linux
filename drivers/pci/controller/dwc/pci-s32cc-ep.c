// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe EndPoint controller driver for NXP S32CC SoCs
 *
 * Copyright 2023-2024 NXP
 */

#if (IS_ENABLED(CONFIG_PCI_S32CC_DEBUG))
#ifndef DEBUG
#define DEBUG
#endif
#endif

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/of_platform.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/phy.h>
#include <linux/nvmem-consumer.h>
#include <linux/ioport.h>
#include <soc/s32cc/revision.h>

#include "pci-s32cc.h"

#define PCIE_EP_DEFAULT_BAR_SIZE	SZ_1M
#define PCI_BASE_ADDRESS_MEM_NON_PREFETCH	0x00	/* non-prefetchable */
#define PCIE_EP_BAR_DEFAULT_INIT_FLAGS	(PCI_BASE_ADDRESS_SPACE_MEMORY | \
			PCI_BASE_ADDRESS_MEM_TYPE_32 | \
			PCI_BASE_ADDRESS_MEM_NON_PREFETCH)

#define PCI_INTERRUPT_PINS_MASK		0xffff00ff
#define PCI_INTERRUPT_PINS_VAL		0x00000100

/* List for EndPoint devices to allow accessing them from user space (by ioctls) */
static LIST_HEAD(s32cc_pcie_ep_list);
/* Mutex for the End Point list, for safe management of the probed EndPoints */
static DEFINE_MUTEX(s32cc_pcie_ep_list_mutex);

struct s32cc_pcie_ep_node {
	struct list_head list;
	struct s32cc_pcie *ep;
	struct pci_epf_bar ep_bars[PCI_STD_NUM_BARS];
};

static const struct pci_epc_features s32cc_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = true,
	.reserved_bar = BIT(BAR_1) | BIT(BAR_5),
	.bar_fixed_64bit = BIT(BAR_0),
	.bar_fixed_size[0] = SZ_2M,
	.bar_fixed_size[2] = SZ_1M,
	.bar_fixed_size[3] = SZ_64K,
	.bar_fixed_size[4] = 256,
};

static const struct pci_epc_features *
s32cc_pcie_ep_get_features(struct dw_pcie_ep *ep)
{
	return &s32cc_pcie_epc_features;
}

/* msi IRQ handler
 * irq - interrupt number
 * arg - pointer to the "struct pcie_port" object
 */
static irqreturn_t s32cc_pcie_msi_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}

/* Chained MSI interrupt service routine, for EP */
static void dw_ep_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pcie_port *pp;

	chained_irq_enter(chip, desc);

	pp = irq_desc_get_handler_data(desc);
	dw_handle_msi_irq(pp);

	chained_irq_exit(chip, desc);
}

/* Return the s32cc_pcie object for the EP with the given PCIe ID,
 * as specified in the device tree
 */
struct s32cc_pcie *s32cc_get_dw_pcie(int pcie_ep_id)
{
	struct s32cc_pcie_ep_node *pci_node, *n;
	struct s32cc_pcie *res = ERR_PTR(-ENODEV);

	mutex_lock(&s32cc_pcie_ep_list_mutex);
	list_for_each_entry_safe(pci_node, n, &s32cc_pcie_ep_list, list) {
#if (!IS_ENABLED(CONFIG_PCI_S32CC_IOCTL_LIMIT_ONE_ENDPOINT))
		if (pci_node->ep->id == pcie_ep_id) {
			res = pci_node->ep;
			break;
		}
#else
		/* Take the first entry */
		res = pci_node->ep;
		break;
#endif
	}
	mutex_unlock(&s32cc_pcie_ep_list_mutex);

	return res;
}
EXPORT_SYMBOL(s32cc_get_dw_pcie);

/* Return the array of BAR configuration for the EP with the given PCIe ID,
 * as specified in the device tree
 */
static struct pci_epf_bar *s32cc_get_bars(int pcie_ep_id)
{
	struct s32cc_pcie_ep_node *pci_node;
	struct pci_epf_bar  *res = ERR_PTR(-ENODEV);

	mutex_lock(&s32cc_pcie_ep_list_mutex);
	list_for_each_entry(pci_node, &s32cc_pcie_ep_list, list) {
#if (!IS_ENABLED(CONFIG_PCI_S32CC_IOCTL_LIMIT_ONE_ENDPOINT))
		if (pci_node->ep->id == pcie_ep_id) {
			res = pci_node->ep_bars;
			break;
		}
#else
		/* Take the first entry */
		res = pci_node->ep_bars;
		break;
#endif
	}
	mutex_unlock(&s32cc_pcie_ep_list_mutex);

	return res;
}

static int s32cc_add_pcie_ep_to_list(struct s32cc_pcie *s32cc_ep)
{
	struct s32cc_pcie_ep_node *ep_entry;

	if (!s32cc_ep)
		return -EINVAL;

	ep_entry = kzalloc(sizeof(*ep_entry), GFP_KERNEL);
	if (!ep_entry)
		return -ENOMEM;

	ep_entry->ep = s32cc_ep;
	mutex_lock(&s32cc_pcie_ep_list_mutex);
	list_add(&ep_entry->list, &s32cc_pcie_ep_list);
	mutex_unlock(&s32cc_pcie_ep_list_mutex);

	return 0;
}

static void s32cc_del_pcie_ep_from_list(int pcie_id)
{
	struct s32cc_pcie_ep_node *ep_entry, *n;

	mutex_lock(&s32cc_pcie_ep_list_mutex);
	list_for_each_entry_safe(ep_entry, n, &s32cc_pcie_ep_list, list) {
		if (ep_entry->ep->id == pcie_id) {
			list_del(&ep_entry->list);
			kfree(ep_entry);
			break;
		}
	}
	mutex_unlock(&s32cc_pcie_ep_list_mutex);
}

static int s32cc_pcie_deinit_controller_ep(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = &s32cc_pp->pcie;

	dw_pcie_ep_exit(&pcie->ep);
	s32cc_del_pcie_ep_from_list(s32cc_pp->id);

	return deinit_controller(s32cc_pp);
}

static void s32cc_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pcie;
	u32 tmp = 0;

	if (!ep) {
		pr_err("%s: No S32CC EP configuration found\n", __func__);
		return;
	}
	pcie = to_dw_pcie_from_ep(ep);

	pcie->iatu_unroll_enabled = dw_pcie_iatu_unroll_enabled(pcie);
	dev_dbg(pcie->dev, "iATU unroll: %s\n",
		pcie->iatu_unroll_enabled ? "enabled" : "disabled");

	dw_pcie_dbi_ro_wr_en(pcie);

	/*
	 * Configure the class and revision for the EP device,
	 * to enable human friendly enumeration by the RC (e.g. by lspci)
	 * EPF will set its own IDs.
	 */
	tmp = dw_pcie_readl_dbi(pcie, PCI_CLASS_REVISION) |
			((PCI_BASE_CLASS_PROCESSOR << PCI_BASE_CLASS_OFF) |
			(PCI_SUBCLASS_OTHER << PCI_SUBCLASS_OFF));
	dw_pcie_writel_dbi(pcie, PCI_CLASS_REVISION, tmp);

	dev_dbg(pcie->dev, "%s: Enable MSI/MSI-X capabilities\n", __func__);

	/* Enable MSIs by setting the capability bit */
	tmp = dw_pcie_readl_dbi(pcie, PCI_MSI_CAP) | MSI_EN;
	dw_pcie_writel_dbi(pcie, PCI_MSI_CAP, tmp);

	/* Enable MSI-Xs by setting the capability bit */
	tmp = dw_pcie_readl_dbi(pcie, PCI_MSIX_CAP) | MSIX_EN;
	dw_pcie_writel_dbi(pcie, PCI_MSIX_CAP, tmp);

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	tmp = dw_pcie_readl_dbi(pcie, PCI_COMMAND) |
			PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, tmp);

	dw_pcie_dbi_ro_wr_dis(pcie);
}

int s32cc_pcie_setup_outbound(struct s32cc_outbound_region *ptr_outb)
{
	int ret = 0;
	struct pci_epc *epc;
	struct s32cc_pcie *s32cc_pcie_ep;

	if (!ptr_outb) {
		pr_err("%s: Invalid Outbound data\n", __func__);
		return -EINVAL;
	}

#if (!IS_ENABLED(CONFIG_PCI_S32CC_IOCTL_LIMIT_ONE_ENDPOINT))
	s32cc_pcie_ep = s32cc_get_dw_pcie(ptr_outb->pcie_id);
#else
	/* No ID is needed in this case; use 0 to satisfy the API */
	s32cc_pcie_ep = s32cc_get_dw_pcie(0);
#endif
	if (IS_ERR(s32cc_pcie_ep)) {
		pr_err("%s: No valid S32CC EP configuration found\n",
			__func__);
		return -ENODEV;
	}

	epc = s32cc_pcie_ep->pcie.ep.epc;
	if (!epc || !epc->ops) {
		dev_err(s32cc_pcie_ep->pcie.dev,
			"Invalid S32CC EP configuration\n");
		return -ENODEV;
	}

	/* Setup outbound region */
	ret = epc->ops->map_addr(epc, 0, 0, ptr_outb->base_addr,
			ptr_outb->target_addr, ptr_outb->size);

	return ret;
}
EXPORT_SYMBOL(s32cc_pcie_setup_outbound);

int s32cc_pcie_setup_inbound(struct s32cc_inbound_region *ptr_inb)
{
	int ret = 0;
	struct pci_epc *epc;
	enum pci_barno bar_num = BAR_0, idx;
	struct pci_epf_bar *s32cc_ep_bars;
	struct s32cc_pcie *s32cc_pcie_ep;
	const struct pci_epc_features *epc_features;
	dma_addr_t next_phys_addr = 0;
	int add;

	if (!ptr_inb) {
		pr_err("%s: Invalid Inbound data\n", __func__);
		return -EINVAL;
	}

#if (!IS_ENABLED(CONFIG_PCI_S32CC_IOCTL_LIMIT_ONE_ENDPOINT))
	s32cc_pcie_ep = s32cc_get_dw_pcie(ptr_inb->pcie_id);
	s32cc_ep_bars = s32cc_get_bars(ptr_inb->pcie_id);
#else
	/* In this case we don't need the ID, so use 0 to comply with the API */
	s32cc_pcie_ep = s32cc_get_dw_pcie(0);
	s32cc_ep_bars = s32cc_get_bars(0);
#endif

	if (IS_ERR(s32cc_pcie_ep) || IS_ERR(s32cc_ep_bars)) {
		pr_err("%s: No valid S32CC EP configuration found\n",
			__func__);
		return -ENODEV;
	}

	epc = s32cc_pcie_ep->pcie.ep.epc;

	if (!epc || !epc->ops) {
		dev_err(s32cc_pcie_ep->pcie.dev,
			"Invalid S32CC EP configuration\n");
		return -ENODEV;
	}
	epc_features = s32cc_pcie_ep_get_features(&s32cc_pcie_ep->pcie.ep);
	if (!epc_features) {
		dev_err(s32cc_pcie_ep->pcie.dev,
			"Invalid S32CC EP controller features\n");
		return -ENODEV;
	}

	/* Setup BARs, starting from the one received as argument */
	if (ptr_inb->bar_nr < PCI_STD_NUM_BARS)
		bar_num = (enum pci_barno)ptr_inb->bar_nr;
	else
		dev_warn(s32cc_pcie_ep->pcie.dev,
			"Invalid BAR (%d); using default BAR0\n", bar_num);

	s32cc_ep_bars[bar_num].phys_addr = ptr_inb->target_addr;
	if (!s32cc_ep_bars[bar_num].size)
		s32cc_ep_bars[bar_num].size = PCIE_EP_DEFAULT_BAR_SIZE;

	/* Setup BARs and inbound regions, up to BAR5 inclusivelly */
	for (idx = bar_num; idx < PCI_STD_NUM_BARS; idx += add) {
		struct pci_epf_bar *epf_bar = &s32cc_ep_bars[idx];

		/* EPC features takes precedence over existing info
		 * in BARs array
		 */
		if (epc_features->bar_fixed_64bit & BIT(idx)) {
			epf_bar->flags &= ~PCI_BASE_ADDRESS_MEM_TYPE_MASK;
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		}
		if (epc_features->bar_fixed_size[idx])
			epf_bar->size = epc_features->bar_fixed_size[idx];

		epf_bar->barno = idx;
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epc_features->reserved_bar & BIT(idx)) ||
		    !epf_bar->size) {
			epf_bar->size = 0;
			continue;
		}

		/* shift BAR physical address if it interferes with
		 * previous BAR
		 */
		if (next_phys_addr > epf_bar->phys_addr)
			epf_bar->phys_addr = next_phys_addr;
		ret = epc->ops->set_bar(epc, 0, 0, epf_bar);
		if (ret)
			dev_err(s32cc_pcie_ep->pcie.dev,
					"%s: Unable to init BAR%d\n",
					__func__, idx);

		next_phys_addr = epf_bar->phys_addr + epf_bar->size;
	}

	return ret;
}
EXPORT_SYMBOL(s32cc_pcie_setup_inbound);

static int s32cc_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				 enum pci_epc_irq_type type, u16 interrupt_num)
{
	struct dw_pcie *pci;

	if (!ep) {
		pr_err("%s: No S32CC EP configuration found\n", __func__);
		return -ENODEV;
	}

	pci = to_dw_pcie_from_ep(ep);
	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		dev_dbg(pci->dev, "%s: func %d: legacy int\n",
			__func__, func_no);
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		dev_dbg(pci->dev, "%s: func %d: MSI %d\n",
			__func__, func_no, interrupt_num);
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		dev_dbg(pci->dev, "%s: func %d: MSI-X %d\n",
			__func__, func_no, interrupt_num);
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "%s: UNKNOWN IRQ type\n", __func__);
	}

	return -EINVAL;
}

#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
static int s32cc_pcie_ep_start_dma(struct dw_pcie_ep *ep, bool read,
				 dma_addr_t src, dma_addr_t dst, u32 len,
				 struct completion *complete)
{
	if (IS_ENABLED(CONFIG_PCI_DW_DMA))
		return dw_pcie_ep_start_dma(ep, read, src, dst, len, complete);

	pr_info("%s: DMA not enabled\n", __func__);
	return -EINVAL;
}
#endif

static struct dw_pcie_ep_ops s32cc_pcie_ep_ops = {
	.ep_init = s32cc_pcie_ep_init,
	.raise_irq = s32cc_pcie_ep_raise_irq,
	.get_features = s32cc_pcie_ep_get_features,
#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
	.start_dma = s32cc_pcie_ep_start_dma,
#endif
};

int s32cc_send_msi(struct dw_pcie *pcie)
{
	/* Trigger the first MSI, since all MSIs are routed through the same
	 * physical interrupt anyway
	 */
	return s32cc_pcie_ep_raise_irq(&pcie->ep, 1,
		PCI_EPC_IRQ_MSI, 1);
}

static int __init s32cc_add_pcie_ep(struct s32cc_pcie *s32cc_pp)
{
	int ret;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct dw_pcie_ep *ep = &pcie->ep;
	struct device *dev = pcie->dev;

	ep->ops = &s32cc_pcie_ep_ops;

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize endpoint\n");
		return ret;
	}

	ret = s32cc_add_pcie_ep_to_list(s32cc_pp);
	if (ret)
		dev_warn(s32cc_pp->pcie.dev, "can't populate EP list\n");

	return 0;
}

static int s32cc_pcie_dt_init_ep(struct platform_device *pdev,
				struct s32cc_pcie *s32cc_pp)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct dw_pcie_ep *ep = &pcie->ep;
	int ret;

	ret = s32cc_pcie_dt_init_common(pdev, s32cc_pp);
	if (ret)
		return ret;

	/* This is for EP only */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	dev_dbg(dev, "addr_space: %pR\n", res);
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	s32cc_pp->auto_config_bars = of_property_read_bool(np, "auto-config-bars");

	return 0;
}

static int s32cc_pcie_config_ep(struct s32cc_pcie *s32cc_pp,
					struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	int ret = 0;

	ret = s32cc_pcie_config_common(s32cc_pp, pdev);
	if (ret)
		return ret;

	/* MSI configuration for EP */
	if (IS_ENABLED(CONFIG_PCI_S32CC_EP_MSI)) {
		u32 val, ctrl, num_ctrls;
		struct pcie_port *pp = &pcie->pp;

		ret = s32cc_pcie_config_irq(&pcie->pp.msi_irq, "msi", pdev,
					    s32cc_pcie_msi_handler, &pcie->pp);
		if (ret) {
			dev_err(dev, "failed to request msi irq\n");
			return ret;
		}

		pp->num_vectors = MSI_DEF_NUM_VECTORS;
		ret = dw_pcie_allocate_domains(pp);
		if (ret)
			dev_err(dev, "Unable to setup MSI domain for EP\n");

		if (pp->msi_irq)
			irq_set_chained_handler_and_data(pp->msi_irq,
				dw_ep_chained_msi_isr,
				pp);

		if (pp->has_msi_ctrl) {
			num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

			/* Initialize IRQ Status array.
			 * See void dw_pcie_setup_rc(struct pcie_port *pp)
			 */
			for (ctrl = 0; ctrl < num_ctrls; ctrl++) {
				pp->irq_mask[ctrl] = ~0;
				dw_pcie_writel_dbi(pcie, PCIE_MSI_INTR0_MASK +
						(ctrl * MSI_REG_CTRL_BLOCK_SIZE),
						pp->irq_mask[ctrl]);
				dw_pcie_writel_dbi(pcie, PCIE_MSI_INTR0_ENABLE +
						(ctrl * MSI_REG_CTRL_BLOCK_SIZE),
						~0);
			}
		}

		/* Setup interrupt pins */
		val = dw_pcie_readl_dbi(pcie, PCI_INTERRUPT_LINE);
		val &= PCI_INTERRUPT_PINS_MASK;
		val |= PCI_INTERRUPT_PINS_VAL;
		dw_pcie_writel_dbi(pcie, PCI_INTERRUPT_LINE, val);

		dw_pcie_msi_init(&pcie->pp);
	}

	if (IS_ENABLED(CONFIG_PCI_DW_DMA)) {
		s32cc_config_dma_data(&s32cc_pp->dma, pcie);
		ret = s32cc_pcie_config_irq(&s32cc_pp->dma_irq,
			"dma", pdev,
			s32cc_pcie_dma_handler, pcie);
		if (ret) {
			dev_err(dev, "failed to request dma irq\n");
			goto fail_ep_init;
		}
	}

	ret = s32cc_add_pcie_ep(s32cc_pp);
	if (ret)
		goto fail_ep_init;

	s32cc_config_user_space_data(&s32cc_pp->uinfo, pcie);

	return ret;

fail_ep_init:
	s32cc_pcie_deinit_controller_ep(s32cc_pp);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	return ret;
}

static struct dw_pcie_ops s32cc_pcie_ops = {
	.link_up = s32cc_pcie_link_is_up,
	.start_link = s32cc_pcie_start_link,
	.stop_link = s32cc_pcie_stop_link,
	.write_dbi = s32cc_pcie_write,
};

static const struct of_device_id s32cc_pcie_of_match_ep[];

static int s32cc_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s32cc_pcie *s32cc_pp;
	struct dw_pcie *pcie;
	const struct of_device_id *match;
	const struct s32cc_pcie_data *data;
	int ret = 0;

	ret = s32cc_check_serdes(dev);
	if (ret)
		return ret;

	match = of_match_device(s32cc_pcie_of_match_ep, dev);
	if (!match)
		return -EINVAL;

	s32cc_pp = devm_kzalloc(dev, sizeof(*s32cc_pp), GFP_KERNEL);
	if (!s32cc_pp)
		return -ENOMEM;

	pcie = &s32cc_pp->pcie;
	pcie->dev = dev;
	pcie->ops = &s32cc_pcie_ops;

	platform_set_drvdata(pdev, s32cc_pp);

	data = match->data;
	s32cc_pp->mode = data->mode;

	ret = s32cc_pcie_dt_init_ep(pdev, s32cc_pp);
	if (ret)
		goto err;

	ret = s32cc_pcie_config_ep(s32cc_pp, pdev);
	if (ret) {
		dev_err(dev, "Failed to set common PCIe settings\n");
		goto err;
	}

	/* Configure BARs, if necessary. Only warn if it fails. */
	if (s32cc_pp->auto_config_bars) {
		int result = -EINVAL;
		int bar = 0;

		if (s32cc_pp->shared_mem.start > 0 &&
		    s32cc_pp->shared_mem.end > 0) {
			struct s32cc_inbound_region ptr_inb = {
#if (!IS_ENABLED(CONFIG_PCI_S32CC_IOCTL_LIMIT_ONE_ENDPOINT))
				s32cc_pp->id,
#endif
				bar,
				s32cc_pp->shared_mem.start,
			};

			result = s32cc_pcie_setup_inbound(&ptr_inb);
			if (!result) {
				/* Verify BARs fit the reserved region */
				struct pci_epf_bar *epf_bar, *s32cc_ep_bars =
					s32cc_get_bars(s32cc_pp->id);
				size_t sum = 0;

				/* s32cc_ep_bars was checked in
				 * s32cc_pcie_setup_inbound
				 */
				for (; bar < PCI_STD_NUM_BARS; bar++) {
					epf_bar = &s32cc_ep_bars[bar];
					sum += epf_bar->size;
				}
				if (sum > (s32cc_pp->shared_mem.end -
				    s32cc_pp->shared_mem.start)) {
					dev_warn(dev, "EP BARs too large\n");
					result = -EINVAL;
				}
			}
		}
		if (result)
			dev_warn(dev, "Failed to configure EP BARs\n");
	}

err:
	if (ret)
		platform_set_drvdata(pdev, NULL);

	return ret;
}

#if (IS_ENABLED(CONFIG_PM_SLEEP))
static const struct dev_pm_ops s32cc_pcie_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s32cc_pcie_suspend,
				s32cc_pcie_resume)
};
#endif

static const struct s32cc_pcie_data ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id s32cc_pcie_of_match_ep[] = {
	{ .compatible = "nxp,s32cc-pcie-ep", .data = &ep_of_data },
	{},
};
MODULE_DEVICE_TABLE(of, s32cc_pcie_of_match_ep);

static struct platform_driver s32cc_pcie_driver_ep = {
	.driver = {
		.name	= "s32cc-pcie-ep",
		.owner	= THIS_MODULE,
		.of_match_table = s32cc_pcie_of_match_ep,
#if (IS_ENABLED(CONFIG_PM_SLEEP))
		.pm = &s32cc_pcie_pm_ops,
#endif
	},
	.probe = s32cc_pcie_probe,
	.shutdown = s32cc_pcie_shutdown,
};

module_platform_driver(s32cc_pcie_driver_ep);

MODULE_AUTHOR("Ionut Vicovan <Ionut.Vicovan@nxp.com>");
MODULE_DESCRIPTION("NXP S32CC PCIe EndPoint controller driver");
MODULE_LICENSE("GPL v2");
