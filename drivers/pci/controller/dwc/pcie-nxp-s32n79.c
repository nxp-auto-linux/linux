// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for NXP S32N79 platform
 *
 * Copyright 2026 NXP
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/property.h>

#include "../../pci.h"
#include "pcie-designware.h"

/* PCM_CMN Core Control 3 register */
#define S32N_PCM_CMN_CORE_CTRL3		0xC
#define S32N_PCM_CMN_CORE_CTRL3_LE	BIT(0)

struct s32n_pcie {
	struct dw_pcie pci;
	void __iomem *pcm;
};

#define to_s32n_from_dw_pcie(x) container_of(x, struct s32n_pcie, pci)

static int s32n_start_link(struct dw_pcie *pci)
{
	struct s32n_pcie *priv = to_s32n_from_dw_pcie(pci);
	u32 reg = readl(priv->pcm + S32N_PCM_CMN_CORE_CTRL3);

	reg |= S32N_PCM_CMN_CORE_CTRL3_LE;
	writel(reg, priv->pcm + S32N_PCM_CMN_CORE_CTRL3);

	return 0;
}

static const struct dw_pcie_ops s32n_pcie_ops = {
	.start_link = s32n_start_link,
};

/* Hide a standard PCI capability by unlinking it from the list */
static void s32n_pcie_remove_capability(struct dw_pcie *pci, u8 cap)
{
	u8 pos, prev = 0;
	int ttl = PCI_FIND_CAP_TTL;

	/* Capability list head pointer (offset 0x34) */
	pos = dw_pcie_readb_dbi(pci, PCI_CAPABILITY_LIST);

	while (pos && ttl--) {
		u8 id = dw_pcie_readb_dbi(pci, pos + PCI_CAP_LIST_ID);
		u8 next = dw_pcie_readb_dbi(pci, pos + PCI_CAP_LIST_NEXT);

		if (id == cap) {
			dw_pcie_dbi_ro_wr_en(pci);
			if (prev)
				dw_pcie_writeb_dbi(pci,
						   prev + PCI_CAP_LIST_NEXT, next);
			else
				dw_pcie_writeb_dbi(pci, PCI_CAPABILITY_LIST,
						   next);
			dw_pcie_dbi_ro_wr_dis(pci);
			return;
		}

		prev = pos;
		pos = next;
	}

	dev_dbg(pci->dev, "cap 0x%02x not found; nothing to remove\n", cap);
}

/* Configure cache coherency boundary */
static int s32n_pcie_config_coherency(struct dw_pcie *pci)
{
	u64 boundary = 0;
	int err;

	err = device_property_read_u64(pci->dev, "nxp,coherency-boundary", &boundary);
	if (err) {
		dev_err(pci->dev, "Invalid coherency boundary property: %d\n", err);
		return err;
	}

	/*
	 * Add a hard boundary in the PCIe controller coherency control
	 * registers to separate coherent memory space from non-coherent
	 * peripheral space.
	 *
	 * This is required to ensure coherent inbound PCIe traffic (e.g., MSIs
	 * with NoSnoop=0) targeting peripherals (e.g., GIC) is handled correctly,
	 * i.e., not dropped by interconnect module, as peripheral config space
	 * on S32N79 is non-coherent.
	 */
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writel_dbi(pci, COHERENCY_CONTROL_1_OFF,
			   (lower_32_bits(boundary) & CFG_MEMTYPE_BOUNDARY_LOW_ADDR_MASK));
	dw_pcie_writel_dbi(pci, COHERENCY_CONTROL_2_OFF, upper_32_bits(boundary));
	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int s32n_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u16 cap, devctl;
	int err;

	/* Set Max payload size to 256B */
	cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	devctl = dw_pcie_readw_dbi(pci, cap + PCI_EXP_DEVCTL);
	devctl = (u16)(devctl & ~PCI_EXP_DEVCTL_PAYLOAD);
	devctl |= PCI_EXP_DEVCTL_PAYLOAD_256B;
	dw_pcie_writew_dbi(pci, cap + PCI_EXP_DEVCTL, devctl);

	/* Configure cache coherency boundary */
	err = s32n_pcie_config_coherency(pci);
	if (err)
		return err;

	/* In RC mode, MSI-X for the Root Port is not implemented */
	s32n_pcie_remove_capability(pci, PCI_CAP_ID_MSIX);

	return 0;
}

static const struct dw_pcie_host_ops s32n_pcie_host_ops = {
	.init = s32n_host_init,
};

static int s32n_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s32n_pcie *priv;
	struct dw_pcie *pci;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	pci = &priv->pci;
	pci->dev = dev;
	pci->ops = &s32n_pcie_ops;
	pci->pp.ops = &s32n_pcie_host_ops;

	priv->pcm = devm_platform_ioremap_resource_byname(pdev, "pcm");
	if (IS_ERR(priv->pcm))
		return dev_err_probe(dev, PTR_ERR(priv->pcm), "Failed to map PCM registers\n");

	platform_set_drvdata(pdev, priv);

	return dw_pcie_host_init(&pci->pp);
}

static const struct of_device_id s32n_pcie_of_match[] = {
	{ .compatible = "nxp,s32n79-pcie" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s32n_pcie_of_match);

static struct platform_driver s32n_pcie_driver = {
	.probe  = s32n_pcie_probe,
	.driver = {
		.name = "s32n79-pcie",
		.of_match_table = s32n_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(s32n_pcie_driver);

MODULE_DESCRIPTION("NXP S32N79 PCIe host controller driver");
MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL");
