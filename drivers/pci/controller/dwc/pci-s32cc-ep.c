// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe EndPoint controller driver for NXP S32CC SoCs
 *
 * Copyright 2023 NXP
 */

#if IS_ENABLED(CONFIG_PCI_S32CC_DEBUG)
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
};

/* msi IRQ handler
 * irq - interrupt number
 * arg - pointer to the "struct dw_pcie_rp" object
 */
static irqreturn_t s32cc_pcie_msi_handler(int irq, void *arg)
{
	struct dw_pcie_rp *pp = arg;

	return dw_handle_msi_irq(pp);
}

/* Chained MSI interrupt service routine, for EP */
static void dw_ep_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct dw_pcie_rp *pp;

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
		if (pci_node->ep->id == pcie_ep_id) {
			res = pci_node->ep;
			break;
		}
	}
	mutex_unlock(&s32cc_pcie_ep_list_mutex);

	return res;
}
EXPORT_SYMBOL(s32cc_get_dw_pcie);

static int s32cc_add_pcie_ep_to_list(struct s32cc_pcie *s32cc_ep)
{
	struct s32cc_pcie_ep_node *ep_entry;

	if (!s32cc_ep)
		return -EINVAL;

	ep_entry = kmalloc(sizeof(*ep_entry), GFP_KERNEL);
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

	/*
	 * Configure the class and revision for the EP device,
	 * to enable human friendly enumeration by the RC (e.g. by lspci)
	 * EPF will set its own IDs.
	 */
	dw_pcie_dbi_ro_wr_en(pcie);
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

	s32cc_pcie_ep = s32cc_get_dw_pcie(ptr_outb->pcie_id);
	if (IS_ERR(s32cc_pcie_ep)) {
		pr_err("%s: No S32CC EP configuration found for PCIe%d\n",
		       __func__, ptr_outb->pcie_id);
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
	int bar_num;
	struct pci_epf_bar bar = {
		.size = PCIE_EP_DEFAULT_BAR_SIZE,
		.flags = PCIE_EP_BAR_DEFAULT_INIT_FLAGS,
	};
	struct s32cc_pcie *s32cc_pcie_ep;

	if (!ptr_inb) {
		pr_err("%s: Invalid Inbound data\n", __func__);
		return -EINVAL;
	}

	s32cc_pcie_ep = s32cc_get_dw_pcie(ptr_inb->pcie_id);
	if (IS_ERR(s32cc_pcie_ep)) {
		pr_err("%s: No S32CC EP configuration found for PCIe%d\n",
		       __func__, ptr_inb->pcie_id);
		return -ENODEV;
	}

	epc = s32cc_pcie_ep->pcie.ep.epc;

	if (!epc || !epc->ops) {
		dev_err(s32cc_pcie_ep->pcie.dev,
			"Invalid S32CC EP configuration\n");
		return -ENODEV;
	}

	/* Setup inbound region */
	bar_num = ptr_inb->bar_nr;
	if (bar_num < BAR_0 || bar_num >= BAR_5) {
		dev_err(s32cc_pcie_ep->pcie.dev,
			"Invalid BAR number (%d)\n", bar_num);
		return -EINVAL;
	}

	bar.barno = bar_num;
	bar.phys_addr = ptr_inb->target_addr;
	ret = epc->ops->set_bar(epc, 0, 0, &bar);

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

static const struct pci_epc_features s32cc_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = false,
	.msix_capable = true,
	.reserved_bar = BIT(BAR_1) | BIT(BAR_5),
	.bar_fixed_64bit = BIT(BAR_0),
	.bar_fixed_size[0] = SZ_1M,
	.bar_fixed_size[2] = (4 * SZ_1M),
	.bar_fixed_size[3] = SZ_64K,
	.bar_fixed_size[4] = 256,
};

static const struct pci_epc_features*
s32cc_pcie_ep_get_features(struct dw_pcie_ep *ep)
{
	return &s32cc_pcie_epc_features;
}

static struct dw_pcie_ep_ops s32cc_pcie_ep_ops = {
	.ep_init = s32cc_pcie_ep_init,
	.raise_irq = s32cc_pcie_ep_raise_irq,
	.get_features = s32cc_pcie_ep_get_features,
};

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
	struct resource *res;
	struct dw_pcie_ep *ep = &pcie->ep;
	int ret;

	ret = s32cc_pcie_dt_init_common(pdev, s32cc_pp);
	if (ret)
		return ret;

	/* This is for EP only */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;
	dev_dbg(dev, "addr_space: %pR\n", res);

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	return 0;
}

static int s32cc_pcie_config_ep(struct s32cc_pcie *s32cc_pp,
				struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	u32 val, ctrl, num_ctrls;
	struct dw_pcie_rp *pp = &pcie->pp;
	int ret = 0;

	ret = s32cc_pcie_config_common(s32cc_pp, pdev);
	if (ret)
		return ret;

	/* MSI configuration for EP */
	if (IS_ENABLED(CONFIG_PCI_S32CC_EP_MSI)) {
		ret = s32cc_pcie_config_irq(&pcie->pp.msi_irq[0], "msi", pdev,
					    s32cc_pcie_msi_handler, &pcie->pp);
		if (ret) {
			dev_err(dev, "Failed to request msi irq\n");
			return ret;
		}

		pp->num_vectors = MSI_DEF_NUM_VECTORS;
		ret = dw_pcie_allocate_domains(pp);
		if (ret)
			dev_err(dev, "Unable to setup MSI domain for EP\n");

		if (pp->msi_irq)
			irq_set_chained_handler_and_data(pp->msi_irq[0],
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

	ret = s32cc_add_pcie_ep(s32cc_pp);
	if (ret)
		goto fail_ep_init;

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

static int s32cc_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s32cc_pcie *s32cc_pp;
	struct dw_pcie *pcie;
	int ret = 0;

	ret = s32cc_check_serdes(dev);
	if (ret)
		return ret;

	s32cc_pp = devm_kzalloc(dev, sizeof(*s32cc_pp), GFP_KERNEL);
	if (!s32cc_pp)
		return -ENOMEM;

	pcie = &s32cc_pp->pcie;
	pcie->dev = dev;
	pcie->ops = &s32cc_pcie_ops;

	platform_set_drvdata(pdev, s32cc_pp);

	ret = s32cc_pcie_dt_init_ep(pdev, s32cc_pp);
	if (ret)
		goto err;

	ret = s32cc_pcie_config_ep(s32cc_pp, pdev);
	if (ret) {
		dev_err(dev, "Failed to set common PCIe settings\n");
		goto err;
	}

err:
	if (ret)
		platform_set_drvdata(pdev, NULL);

	return ret;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
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
#if IS_ENABLED(CONFIG_PM_SLEEP)
		.pm = &s32cc_pcie_pm_ops,
#endif
	},
	.probe = s32cc_pcie_probe,
	.shutdown = s32cc_pcie_shutdown,
};

module_platform_driver(s32cc_pcie_driver_ep);

MODULE_AUTHOR("Ionut Vicovan <Ionut.Vicovan@nxp.com>");
MODULE_DESCRIPTION("NXP S32CC PCIe EndPoint controller driver");
MODULE_LICENSE("GPL");
