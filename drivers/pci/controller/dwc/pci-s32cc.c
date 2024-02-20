// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for NXP S32CC SoCs
 *
 * Copyright 2019-2024 NXP
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

#include "pci-s32cc-regs.h"
#include "pci-s32cc.h"
#include "../../pci.h"

#if (IS_ENABLED(CONFIG_PCI_S32CC_DEBUG_WRITES))
#define dev_dbg_w dev_dbg
#define PTR_FMT "%px"
#else
#define dev_dbg_w(fmt, ...)
#define PTR_FMT "%px"
#endif

#define PCIE_LINKUP			(BIT(PCIE_SS_SMLH_LINK_UP_BIT) | \
					BIT(PCIE_SS_RDLH_LINK_UP_BIT))

/* Default timeout (ms) */
#define PCIE_CX_CPL_BASE_TIMER_VALUE	100

/* Use 200ms for PHY link timeout (slightly larger than 100ms, which PCIe standard requests
 * to wait "before sending a Configuration Request to the device")
 */
#define PCIE_LINK_TIMEOUT_MS		200
#define PCIE_LINK_TIMEOUT_US		(PCIE_LINK_TIMEOUT_MS * USEC_PER_MSEC)
#define PCIE_LINK_WAIT_US		1000

enum pcie_dev_type_val {
	PCIE_EP_VAL = 0x0,
	PCIE_RC_VAL = 0x4
};

void s32cc_pcie_write(struct dw_pcie *pci,
		void __iomem *base, u32 reg, size_t size, u32 val)
{
	int ret;
	struct s32cc_pcie *s32cc_pci = to_s32cc_from_dw_pcie(pci);

	if (IS_ENABLED(CONFIG_PCI_S32CC_DEBUG_WRITES)) {
		if ((uintptr_t)base == (uintptr_t)(s32cc_pci->ctrl_base))
			dev_dbg_w(pci->dev, "W%d(ctrl+0x%x, 0x%x)\n",
				(int)size * 8, (u32)(reg), (u32)(val));
		else if ((uintptr_t)base == (uintptr_t)(pci->atu_base))
			dev_dbg_w(pci->dev, "W%d(atu+0x%x, 0x%x)\n",
				(int)size * 8, (u32)(reg), (u32)(val));
		else if ((uintptr_t)base == (uintptr_t)(pci->dbi_base))
			dev_dbg_w(pci->dev, "W%d(dbi+0x%x, 0x%x)\n",
				(int)size * 8, (u32)(reg), (u32)(val));
		else if ((uintptr_t)base == (uintptr_t)(pci->dbi_base2))
			dev_dbg_w(pci->dev, "W%d(dbi2+0x%x, 0x%x)\n",
				(int)size * 8, (u32)(reg), (u32)(val));
		else if (IS_ENABLED(CONFIG_PCI_DW_DMA) &&
			(uintptr_t)base == (uintptr_t)(s32cc_pci->dma.dma_base))
			dev_dbg_w(pci->dev, "W%d(dma+0x%x, 0x%x)\n",
				(int)size * 8, (u32)(reg), (u32)(val));
		else
			dev_dbg_w(pci->dev, "W%d(%lx+0x%x, 0x%x)\n",
				(int)size * 8, (uintptr_t)(base), (u32)(reg), (u32)(val));
	}

	ret = dw_pcie_write(base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "(pcie%d): Write to address 0x%lx failed\n",
			s32cc_pci->id, (uintptr_t)(base + reg));
}

/* Allow printing DMA writes */
static inline void s32cc_pcie_write_dma(struct dma_info *di,
		void __iomem *base, u32 reg, size_t size, u32 val)
{
	struct s32cc_pcie *s32cc_pci = to_s32cc_from_dma_info(di);

	s32cc_pcie_write(&s32cc_pci->pcie, base, reg, size, val);
}

void dw_pcie_writel_ctrl(struct s32cc_pcie *pci, u32 reg, u32 val)
{
	s32cc_pcie_write(&pci->pcie, pci->ctrl_base, reg, 0x4, val);
}

u32 dw_pcie_readl_ctrl(struct s32cc_pcie *pci, u32 reg)
{
	u32 val = 0;

	if (dw_pcie_read(pci->ctrl_base + reg, 0x4, &val))
		dev_err(pci->pcie.dev, "Read ctrl address failed\n");

	return val;
}

struct dma_info *dw_get_dma_info(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pp =
		to_s32cc_from_dw_pcie(pcie);
	return &s32cc_pp->dma;
}

struct s32cc_userspace_info *dw_get_userspace_info(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pci = to_s32cc_from_dw_pcie(pcie);

	return &s32cc_pci->uinfo;
}

static bool s32cc_has_msi_parent(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct s32cc_pcie *s32cc_pci = to_s32cc_from_dw_pcie(pcie);

	return s32cc_pci->has_msi_parent;
}

u8 dw_pcie_iatu_unroll_enabled(struct dw_pcie *pci)
{
	u32 val;

	val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	if (val == 0xffffffff)
		return 1;

	return 0;
}

int s32cc_check_serdes(struct device *dev)
{
	struct nvmem_cell *serdes_cell;
	size_t read_len = 0;
	u8 *serdes = NULL;
	int ret = 0;

	serdes_cell = devm_nvmem_cell_get(dev, "serdes_presence");
	if (IS_ERR(serdes_cell)) {
		if (PTR_ERR(serdes_cell) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get serdes cell\n");
		return PTR_ERR(serdes_cell);
	}

	serdes = nvmem_cell_read(serdes_cell, &read_len);
	devm_nvmem_cell_put(dev, serdes_cell);
	if (IS_ERR(serdes)) {
		dev_err(dev, "Failed to read serdes cell\n");
		return PTR_ERR(serdes);
	} else if (read_len != SERDES_CELL_SIZE) {
		dev_err(dev, "Invalid read size of serdes cell\n");
		ret = -EINVAL;
		goto out;
	}

	if (!(*serdes)) {
		dev_err(dev, "SerDes subsystem is not present, skipping configuring PCIe\n");
		ret = -ENODEV;
	}

out:
	kfree(serdes);
	return ret;
}

static void s32cc_pcie_enable_hot_unplug_irq(struct s32cc_pcie *pci)
{
	u32 tmp = dw_pcie_readl_ctrl(pci, LINK_INT_CTRL_STS) |
			LINK_REQ_RST_NOT_INT_EN;

	dw_pcie_writel_ctrl(pci, LINK_INT_CTRL_STS, tmp);
}

static void s32cc_pcie_disable_hot_unplug_irq(struct s32cc_pcie *pci)
{
	u32 tmp = dw_pcie_readl_ctrl(pci, LINK_INT_CTRL_STS)
			& ~(LINK_REQ_RST_NOT_INT_EN);

	dw_pcie_writel_ctrl(pci, LINK_INT_CTRL_STS, tmp);
}

static void s32cc_pcie_disable_hot_plug_irq(struct dw_pcie *pcie)
{
	u32 reg;

	dw_pcie_dbi_ro_wr_en(pcie);
	reg = dw_pcie_readl_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS);
	reg &= ~(BIT(PCIE_CAP_PRESENCE_DETECT_CHANGE_EN_BIT) |
		BIT(PCIE_CAP_HOT_PLUG_INT_EN_BIT) |
		BIT(PCIE_CAP_DLL_STATE_CHANGED_EN_BIT));
	dw_pcie_writel_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS, reg);
	dw_pcie_dbi_ro_wr_dis(pcie);
}

static void s32cc_pcie_enable_hot_plug_irq(struct dw_pcie *pcie)
{
	u32 reg;

	dw_pcie_dbi_ro_wr_en(pcie);
	reg = dw_pcie_readl_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS);
	reg |= (BIT(PCIE_CAP_PRESENCE_DETECT_CHANGE_EN_BIT) |
		BIT(PCIE_CAP_HOT_PLUG_INT_EN_BIT) |
		BIT(PCIE_CAP_DLL_STATE_CHANGED_EN_BIT));
	dw_pcie_writel_dbi(pcie, PCIE_SLOT_CONTROL_SLOT_STATUS, reg);
	dw_pcie_dbi_ro_wr_dis(pcie);
}

static void s32cc_pcie_disable_ltssm(struct s32cc_pcie *pci)
{
	u32 gen_ctrl_3 = dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_3)
			& ~(LTSSM_EN_MASK);

	dw_pcie_dbi_ro_wr_en(&pci->pcie);
	dw_pcie_writel_ctrl(pci, PE0_GEN_CTRL_3, gen_ctrl_3);
	dw_pcie_dbi_ro_wr_dis(&pci->pcie);
}

static void s32cc_pcie_enable_ltssm(struct s32cc_pcie *pci)
{
	u32 gen_ctrl_3 = dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_3) |
				LTSSM_EN_MASK;

	dw_pcie_dbi_ro_wr_en(&pci->pcie);
	dw_pcie_writel_ctrl(pci, PE0_GEN_CTRL_3, gen_ctrl_3);
	dw_pcie_dbi_ro_wr_dis(&pci->pcie);
}

static bool is_s32cc_pcie_ltssm_enabled(struct s32cc_pcie *pci)
{
	return (dw_pcie_readl_ctrl(pci, PE0_GEN_CTRL_3) & LTSSM_EN_MASK);
}

static bool has_data_phy_link(struct s32cc_pcie *s32cc_pp)
{
	u32 val = dw_pcie_readl_ctrl(s32cc_pp, PCIE_SS_PE0_LINK_DBG_2);

	if ((val & PCIE_LINKUP) == PCIE_LINKUP) {
		switch (val & GET_MASK_VALUE(PCIE_SS_SMLH_LTSSM_STATE)) {
		case LTSSM_STATE_L0:
		case LTSSM_STATE_L0S:
		case LTSSM_STATE_L1_IDLE:
			return true;
		default:
			return false;
		}
	}

	return false;
}

int s32cc_pcie_link_is_up(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pp = to_s32cc_from_dw_pcie(pcie);

	if (!is_s32cc_pcie_ltssm_enabled(s32cc_pp))
		return 0;

	return has_data_phy_link(s32cc_pp);
}

static int s32cc_pcie_get_link_speed(struct dw_pcie *pcie)
{
	u32 cap_offset = dw_pcie_find_capability(pcie, PCI_CAP_ID_EXP);
	u32 link_sta = dw_pcie_readw_dbi(pcie, cap_offset + PCI_EXP_LNKSTA);

	/* return link speed based on negotiated link status */
	return link_sta & PCI_EXP_LNKSTA_CLS;
}

static u32 s32cc_pcie_get_link_width(struct dw_pcie *pcie)
{
	u32 cap_offset = dw_pcie_find_capability(pcie, PCI_CAP_ID_EXP);
	u32 link_sta = dw_pcie_readw_dbi(pcie, cap_offset + PCI_EXP_LNKSTA);

	return (link_sta & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
}

static struct pci_bus *s32cc_get_child_downstream_bus(struct pci_bus *bus)
{
	struct pci_bus *child, *root_bus = NULL;

	list_for_each_entry(child, &bus->children, node) {
		if (child->parent == bus) {
			root_bus = child;
			break;
		}
	}

	if (!root_bus)
		return ERR_PTR(-ENODEV);

	return root_bus;
}

static int s32cc_enable_hotplug_cap(struct dw_pcie *pcie)
{
	struct pcie_port *pp = &pcie->pp;
	struct pci_bus *bus = pp->bridge->bus;
	struct pci_bus *root_bus;
	struct pci_dev *dev;
	int pos;
	u32 reg32;
	u16 reg16;

	root_bus = s32cc_get_child_downstream_bus(bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		return PTR_ERR(root_bus);
	}
	dev = root_bus->self;

	pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (!pos) {
		dev_err(pcie->dev, "Unable to find PCI_CAP_ID_EXP capability\n");
		return -ENXIO;
	}

	pci_read_config_word(dev, pos + PCI_EXP_FLAGS, &reg16);
	reg16 |= PCI_EXP_FLAGS_SLOT;
	pci_write_config_word(dev, pos + PCI_EXP_FLAGS, reg16);

	pcie_capability_read_dword(dev, PCI_EXP_SLTCAP, &reg32);
	reg32 |= (PCI_EXP_SLTCAP_HPC | PCI_EXP_SLTCAP_HPS);
	pcie_capability_write_dword(dev, PCI_EXP_SLTCAP, reg32);

	s32cc_pcie_enable_hot_plug_irq(pcie);

	return 0;
}

int s32cc_pcie_start_link(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pp = to_s32cc_from_dw_pcie(pcie);
	u32 tmp, cap_offset;
	int ret = 0;

	/* Don't do anything if not Root Complex */
	if (!is_s32cc_pcie_rc(s32cc_pp->mode))
		return 0;

	/* Try to (re)establish the link, starting with Gen1 */
	s32cc_pcie_disable_ltssm(s32cc_pp);

	dw_pcie_dbi_ro_wr_en(pcie);
	cap_offset = dw_pcie_find_capability(pcie, PCI_CAP_ID_EXP);
	tmp = (dw_pcie_readl_dbi(pcie, cap_offset + PCI_EXP_LNKCAP) &
			~(PCI_EXP_LNKCAP_SLS)) | PCI_EXP_LNKCAP_SLS_2_5GB;
	dw_pcie_writel_dbi(pcie, cap_offset + PCI_EXP_LNKCAP, tmp);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Start LTSSM. */
	s32cc_pcie_enable_ltssm(s32cc_pp);
	ret = dw_pcie_wait_for_link(pcie);

	if (ret) {
		/* We do not exit with error if link up was unsuccessful
		 * EndPoint may be connected.
		 */
		ret = 0;
		goto out;
	}

	dw_pcie_dbi_ro_wr_en(pcie);
	/* Allow Gen2 or Gen3 mode after the link is up.
	 * s32cc_pcie.linkspeed is one of the speeds defined in pci_regs.h:
	 * PCI_EXP_LNKCAP_SLS_2_5GB for Gen1
	 * PCI_EXP_LNKCAP_SLS_5_0GB for Gen2
	 * PCI_EXP_LNKCAP_SLS_8_0GB for Gen3
	 */
	tmp = (dw_pcie_readl_dbi(pcie, cap_offset + PCI_EXP_LNKCAP) &
			~(PCI_EXP_LNKCAP_SLS)) | s32cc_pp->linkspeed;
	dw_pcie_writel_dbi(pcie, cap_offset + PCI_EXP_LNKCAP, tmp);

	/*
	 * Start Directed Speed Change so the best possible speed both link
	 * partners support can be negotiated.
	 * The manual says:
	 * When you set the default of the Directed Speed Change field of the
	 * Link Width and Speed Change Control register
	 * (GEN2_CTRL_OFF.DIRECT_SPEED_CHANGE) using the
	 * DEFAULT_GEN2_SPEED_CHANGE configuration parameter to 1, then
	 * the speed change is initiated automatically after link up, and the
	 * controller clears the contents of GEN2_CTRL_OFF.DIRECT_SPEED_CHANGE.
	 */
	tmp = dw_pcie_readl_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL);
	/* Deassert and re-assert GEN2_CTRL_OFF.DIRECT_SPEED_CHANGE, as per S32G3 SerDes
	 * manual rev. 2.0
	 */
	dw_pcie_writel_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL,
			   tmp & (~(u32)PORT_LOGIC_SPEED_CHANGE));
	dw_pcie_writel_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL,
			   tmp | PORT_LOGIC_SPEED_CHANGE);
	dw_pcie_dbi_ro_wr_dis(pcie);

	tmp = dw_pcie_readl_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL);
	ret = read_poll_timeout(dw_pcie_readl_dbi, tmp,
				!(tmp & PORT_LOGIC_SPEED_CHANGE),
				PCIE_LINK_WAIT_US,
				PCIE_LINK_TIMEOUT_US, false,
				pcie, PCIE_LINK_WIDTH_SPEED_CONTROL);

	if (ret)
		dev_info(pcie->dev, "Speed change timeout\n");

	/* Make sure link training is finished as well! */
	ret = dw_pcie_wait_for_link(pcie);

	if (!ret)
		dev_info(pcie->dev, "X%d, Gen%d\n",
			 s32cc_pcie_get_link_width(pcie),
			 s32cc_pcie_get_link_speed(pcie));
out:
	return ret;
}

void s32cc_pcie_stop_link(struct dw_pcie *pcie)
{
	struct s32cc_pcie *s32cc_pp = to_s32cc_from_dw_pcie(pcie);

	s32cc_pcie_disable_ltssm(s32cc_pp);
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

static int s32cc_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct s32cc_pcie *s32cc_pci = to_s32cc_from_dw_pcie(pcie);
	int ret;

	dw_pcie_setup_rc(pp);

	ret = s32cc_pcie_start_link(pcie);
	if (ret) {
		if (!phy_validate(s32cc_pci->phy0, PHY_MODE_PCIE, 0, NULL)) {
			dev_err(pcie->dev, "Failed to get link up with EP connected\n");
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_PCI_MSI) && !s32cc_has_msi_parent(pp))
		dw_pcie_msi_init(pp);

	return 0;
}

static struct dw_pcie_ops s32cc_pcie_ops = {
	.link_up = s32cc_pcie_link_is_up,
	.start_link = s32cc_pcie_start_link,
	.stop_link = s32cc_pcie_stop_link,
	.write_dbi = s32cc_pcie_write,
};

static int s32cc_pcie_msi_host_init(struct pcie_port *pp)
{
	return 0;
}

static struct dw_pcie_host_ops s32cc_pcie_host_ops = {
	.host_init = s32cc_pcie_host_init,
};

static struct dw_pcie_host_ops s32cc_pcie_host_ops2 = {
	.host_init = s32cc_pcie_host_init,
	.msi_host_init = s32cc_pcie_msi_host_init,
};

static irqreturn_t s32cc_pcie_hot_unplug_irq(int irq, void *arg)
{
	struct s32cc_pcie *s32cc_pci = arg;
	struct dw_pcie *pcie = &s32cc_pci->pcie;
	u32 tmp = dw_pcie_readl_ctrl(s32cc_pci, LINK_INT_CTRL_STS) |
				LINK_REQ_RST_NOT_CLR;

	dw_pcie_writel_ctrl(s32cc_pci, LINK_INT_CTRL_STS, tmp);

	if (s32cc_pcie_link_is_up(pcie))
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t s32cc_pcie_hot_unplug_thread(int irq, void *arg)
{
	struct s32cc_pcie *s32cc_pci = arg;
	struct dw_pcie *pcie = &s32cc_pci->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct pci_bus *bus = pp->bridge->bus;
	struct pci_bus *root_bus;
	struct pci_dev *pdev, *temp;
	int ret;

	pci_lock_rescan_remove();

	root_bus = s32cc_get_child_downstream_bus(bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		goto out_unlock_rescan;
	}

	/* if EP is not connected -- Hot-Unplug Surprise event */
	if (phy_validate(s32cc_pci->phy0, PHY_MODE_PCIE, 0, NULL))
		pci_walk_bus(root_bus, pci_dev_set_disconnected, NULL);

	list_for_each_entry_safe_reverse(pdev, temp,
					 &root_bus->devices, bus_list) {
		pci_dev_get(pdev);
		pci_stop_and_remove_bus_device(pdev);
		pci_dev_put(pdev);
	}

	ret = s32cc_enable_hotplug_cap(pcie);
	if (ret)
		dev_err(pcie->dev, "Failed to enable hotplug capability\n");

out_unlock_rescan:
	pci_unlock_rescan_remove();
	return IRQ_HANDLED;
}

static irqreturn_t s32cc_pcie_hot_plug_irq(int irq, void *arg)
{
	struct s32cc_pcie *s32cc_pci = arg;
	u32 tmp = dw_pcie_readl_ctrl(s32cc_pci, PE0_INT_STS) |
				HP_INT_STS;

	dw_pcie_writel_ctrl(s32cc_pci, PE0_INT_STS, tmp);

	/* if EP is not connected, we exit */
	if (phy_validate(s32cc_pci->phy0, PHY_MODE_PCIE, 0, NULL))
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t s32cc_pcie_hot_plug_thread(int irq, void *arg)
{
	struct s32cc_pcie *s32cc_pci = arg;
	struct dw_pcie *pcie = &s32cc_pci->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct pci_bus *bus = pp->bridge->bus;
	struct pci_dev *dev;
	struct pci_bus *root_bus;
	int num, ret;

	pci_lock_rescan_remove();

	root_bus = s32cc_get_child_downstream_bus(bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		goto out_unlock_rescan;
	}

	num = pci_scan_slot(root_bus, PCI_DEVFN(0, 0));
	if (!num) {
		dev_err(pcie->dev, "No new device found\n");
		goto out_unlock_rescan;
	}

	for_each_pci_bridge(dev, root_bus) {
		ret = pci_hp_add_bridge(dev);
		if (ret)
			dev_warn(pcie->dev, "Failed to add hp bridge\n");
	}

	pci_assign_unassigned_bridge_resources(root_bus->self);
	pcie_bus_configure_settings(root_bus);
	pci_bus_add_devices(root_bus);

out_unlock_rescan:
	pci_unlock_rescan_remove();
	return IRQ_HANDLED;
}

#define MAX_IRQ_NAME_SIZE 32
int s32cc_pcie_config_irq(int *irq_id, char *irq_name,
		struct platform_device *pdev,
		irq_handler_t irq_handler, void *irq_arg)
{
	int ret = 0;
	char irq_display_name[MAX_IRQ_NAME_SIZE];

	*(irq_id) = platform_get_irq_byname(pdev, irq_name);
	if (*(irq_id) <= 0) {
		dev_err(&pdev->dev, "failed to get %s irq\n", irq_name);
		return -ENODEV;
	}
	snprintf(irq_display_name, MAX_IRQ_NAME_SIZE, "s32cc-pcie-%s",
			irq_name);
	ret = devm_request_irq(&pdev->dev, *(irq_id), irq_handler,
			IRQF_SHARED,  irq_name, irq_arg);
	if (ret) {
		dev_info(&pdev->dev, "Register interrupt %d (%s) failed (%d)\n",
				*irq_id, irq_name, ret);
		return ret;
	}

	dev_info(&pdev->dev, "Allocated line %d for interrupt %d (%s)",
			ret, *irq_id, irq_name);

	return 0;
}

static int s32cc_pcie_config_hp_irq(struct s32cc_pcie *s32cc_pp,
				    struct platform_device *pdev)
{
	int irq_id, ret;

	irq_id = platform_get_irq_byname(pdev, "link_req_stat");
	if (irq_id <= 0) {
		dev_err(&pdev->dev, "Failed to get link_req_stat irq\n");
		return -ENODEV;
	}

	ret = request_threaded_irq(irq_id, s32cc_pcie_hot_unplug_irq,
				   s32cc_pcie_hot_unplug_thread, IRQF_SHARED,
				   "s32cc-pcie-hot-unplug", s32cc_pp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request link_reg_stat irq\n");
		return ret;
	}

	irq_id = platform_get_irq_byname(pdev, "misc");
	if (irq_id <= 0) {
		dev_err(&pdev->dev, "Failed to get misc irq\n");
		return -ENODEV;
	}

	ret = request_threaded_irq(irq_id, s32cc_pcie_hot_plug_irq,
				   s32cc_pcie_hot_plug_thread, IRQF_SHARED,
				   "s32cc-pcie-hotplug", s32cc_pp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request misc irq\n");
		return ret;
	}

	return ret;
}

static int __init s32cc_add_pcie_port(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	int ret;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(pcie->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

void s32cc_pcie_shutdown(struct platform_device *pdev)
{
	struct s32cc_pcie *s32cc_pp = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (is_s32cc_pcie_rc(s32cc_pp->mode)) {
		/* bring down link, so bootloader gets clean state
		 * in case of reboot
		 */
		s32cc_pcie_disable_ltssm(s32cc_pp);
	}

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	mdelay(PCIE_CX_CPL_BASE_TIMER_VALUE);
}

static const struct of_device_id s32cc_pcie_of_match[];

static void s32cc_pcie_set_phy_mode(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct device *dev = pcie->dev;
	struct device_node *np = dev->of_node;
	const char *pcie_phy_mode = NULL;
	int ret;

	ret = of_property_read_string(np, "nxp,phy-mode", &pcie_phy_mode);
	if (ret || !pcie_phy_mode) {
		dev_info(dev, "Missing 'nxp,phy-mode' property, using default CRNS\n");
		s32cc_pp->phy_mode = CRNS;
	} else if (!strcmp(pcie_phy_mode, "crns")) {
		s32cc_pp->phy_mode = CRNS;
	} else if (!strcmp(pcie_phy_mode, "crss")) {
		s32cc_pp->phy_mode = CRSS;
	} else if (!strcmp(pcie_phy_mode, "srns")) {
		s32cc_pp->phy_mode = SRNS;
	} else if (!strcmp(pcie_phy_mode, "sris")) {
		s32cc_pp->phy_mode = SRIS;
	} else {
		dev_warn(dev, "Unsupported 'nxp,phy-mode' specified, using default CRNS\n");
		s32cc_pp->phy_mode = CRNS;
	}
}

int s32cc_pcie_dt_init_common(struct platform_device *pdev,
			      struct s32cc_pcie *s32cc_pp)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct resource *res;
	u32 pcie_vendor_id = PCI_VENDOR_ID_FREESCALE, pcie_variant_bits = 0;
	int ret;
	struct device_node *shmn;

	ret = of_property_read_u32(np, "device_id", &s32cc_pp->id);
	if (ret) {
		dev_err(dev, "Missing 'device_id' property\n");
		return ret;
	}

	s32cc_pcie_set_phy_mode(s32cc_pp);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pcie->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->dbi_base))
		return PTR_ERR(pcie->dbi_base);
	dev_dbg(dev, "dbi: %pR\n", res);
	dev_dbg(dev, "dbi virt: 0x" PTR_FMT "\n", pcie->dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi2");
	pcie->dbi_base2 = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->dbi_base2))
		return PTR_ERR(pcie->dbi_base2);
	dev_dbg(dev, "dbi2: %pR\n", res);
	dev_dbg(dev, "dbi2 virt: 0x" PTR_FMT "\n", pcie->dbi_base2);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "atu");
	dev_dbg(dev, "atu: %pR\n", res);
	pcie->atu_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->atu_base))
		return PTR_ERR(pcie->atu_base);
	dev_dbg(dev, "atu virt: 0x" PTR_FMT "\n", pcie->atu_base);

	if (IS_ENABLED(CONFIG_PCI_DW_DMA)) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
		dev_dbg(dev, "dma: %pR\n", res);
		s32cc_pp->dma.dma_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(s32cc_pp->dma.dma_base))
			return PTR_ERR(s32cc_pp->dma.dma_base);
		dev_dbg(dev, "dma virt: 0x" PTR_FMT "\n", s32cc_pp->dma.dma_base);
		s32cc_pp->dma.iatu_unroll_enabled = dw_pcie_iatu_unroll_enabled(pcie);
		if (IS_ENABLED(CONFIG_PCI_S32CC_DEBUG_WRITES))
			s32cc_pp->dma.write_dma = s32cc_pcie_write_dma;
	} else {
		s32cc_pp->dma.dma_base = NULL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrl");
	s32cc_pp->ctrl_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(s32cc_pp->ctrl_base))
		return PTR_ERR(s32cc_pp->ctrl_base);
	dev_dbg(dev, "ctrl: %pR\n", res);
	dev_dbg(dev, "ctrl virt: 0x" PTR_FMT "\n", s32cc_pp->ctrl_base);

	s32cc_pp->linkspeed = (enum pcie_link_speed)of_pci_get_max_link_speed(np);
	if (s32cc_pp->linkspeed < GEN1 || s32cc_pp->linkspeed > GEN3) {
		dev_warn(dev, "Invalid PCIe speed; setting to GEN1\n");
		s32cc_pp->linkspeed = GEN1;
	}

	/* Reserved memory */
	/* Get pointer to shared mem region device node from "memory-region" phandle.
	 * Don't throw errors if not available, just warn and go on without.
	 */
	shmn = of_parse_phandle(np, "shared-mem", 0);
	if (shmn) {
		/* Convert memory region to a struct resource */
		ret = of_address_to_resource(shmn, 0, &s32cc_pp->shared_mem);
		of_node_put(shmn);
		if (ret) {
			dev_warn(dev, "Failed to translate shared-mem to a resource\n");
			s32cc_pp->shared_mem.start = 0;
			s32cc_pp->shared_mem.end = 0;
		}
	} else {
		dev_warn(dev, "No shared-mem node\n");
	}

	/* If "msi-parent" property is present in device tree and the PCIe
	 * is RC, MSIs will not be handled by iMSI-RX (default mechanism
	 * implemented in DesignWare core).
	 * The MSIs will be forwarded through AXI bus to the msi parent,
	 * which should be the GIC, which will generate MSIs as SPIs.
	 */
	if (is_s32cc_pcie_rc(s32cc_pp->mode) && of_parse_phandle(np, "msi-parent", 0))
		s32cc_pp->has_msi_parent = true;

	ret = of_property_read_u32(np, "pcie_device_id", &pcie_variant_bits);
	if (ret) {
		ret = s32cc_nvmem_get_pcie_dev_id(dev, &pcie_variant_bits);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_info(dev, "Error reading PCIe Device ID from NVMEM\n");
			return ret;
		}
	}

	if (!pcie_variant_bits)
		return 0;

	/* Write PCI Vendor and Device ID. */
	pcie_vendor_id |= pcie_variant_bits << PCI_DEVICE_ID_SHIFT;
	dev_dbg(dev, "Setting PCI Device and Vendor IDs to 0x%x:0x%x\n",
		(u32)(pcie_vendor_id >> PCI_DEVICE_ID_SHIFT),
		(u32)(pcie_vendor_id & GENMASK(15, 0)));
	dw_pcie_dbi_ro_wr_en(pcie);
	dw_pcie_writel_dbi(pcie, PCI_VENDOR_ID, pcie_vendor_id);

	if (pcie_vendor_id != dw_pcie_readl_dbi(pcie, PCI_VENDOR_ID))
		dev_warn(dev, "PCI Device and Vendor IDs could not be set\n");

	dw_pcie_dbi_ro_wr_dis(pcie);

	return 0;
}

static void disable_equalization(struct dw_pcie *pcie)
{
	u32 val;

	dw_pcie_dbi_ro_wr_en(pcie);

	val = dw_pcie_readl_dbi(pcie, PORT_LOGIC_GEN3_EQ_CONTROL);
	val &= ~(GET_MASK_VALUE(PCIE_GEN3_EQ_FB_MODE) |
		GET_MASK_VALUE(PCIE_GEN3_EQ_PSET_REQ_VEC));
	val |= BUILD_MASK_VALUE(PCIE_GEN3_EQ_FB_MODE, 1) |
		BUILD_MASK_VALUE(PCIE_GEN3_EQ_PSET_REQ_VEC, 0x84);
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_GEN3_EQ_CONTROL, val);

	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Test value */
	dev_dbg(pcie->dev, "PCIE_PORT_LOGIC_GEN3_EQ_CONTROL: 0x%08x\n",
		dw_pcie_readl_dbi(pcie, PORT_LOGIC_GEN3_EQ_CONTROL));
}

static int find_first_system_ram_res(struct resource *res, void *arg)
{
	u64 *prev_min = arg;

	if (!arg || !res)
		return 0;
	if (res->start < *prev_min)
		*prev_min = res->start;

	/* Continue iteration in walk_system_ram_res() */
	return 0;
}

static u64 get_system_ram_base(void)
{
	u64 prev_min = GENMASK_ULL(40, 40);

	walk_system_ram_res(0x0, GENMASK_ULL(40, 40) - 1, &prev_min,
			    find_first_system_ram_res);
	return prev_min;
}

static void s32cc_pcie_reset_mstr_ace(struct dw_pcie *pcie)
{
	u64 ddr_base_addr = get_system_ram_base();
	u32 ddr_base_low = lower_32_bits(ddr_base_addr);
	u32 ddr_base_high = upper_32_bits(ddr_base_addr);

	dw_pcie_dbi_ro_wr_en(pcie);
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_3, 0x0);
	/* Transactions to peripheral targets should be non-coherent,
	 * or Ncore might drop them. Define the start of DDR as seen by Linux
	 * as the boundary between "memory" and "peripherals", with peripherals
	 * being below this boundary, and memory addresses being above it.
	 * One example where this is needed are PCIe MSIs, which use NoSnoop=0
	 * and might end up routed to Ncore.
	 */
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_1,
		(ddr_base_low & CC_1_MEMTYPE_BOUNDARY_MASK) |
		(CC_1_MEMTYPE_LOWER_PERIPH & CC_1_MEMTYPE_VALUE_MASK));
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_2, ddr_base_high);
	dw_pcie_dbi_ro_wr_dis(pcie);
}

static int init_pcie(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct device *dev = pcie->dev;
	u32 val;

	if (is_s32cc_pcie_rc(s32cc_pp->mode))
		val = dw_pcie_readl_ctrl(s32cc_pp, PE0_GEN_CTRL_1) |
				BUILD_MASK_VALUE(DEVICE_TYPE, PCIE_RC_VAL);
	else
		val = dw_pcie_readl_ctrl(s32cc_pp, PE0_GEN_CTRL_1) |
				BUILD_MASK_VALUE(DEVICE_TYPE, PCIE_EP_VAL);

	dw_pcie_writel_ctrl(s32cc_pp, PE0_GEN_CTRL_1, val);

	if (s32cc_pp->phy_mode == SRIS) {
		val = dw_pcie_readl_ctrl(s32cc_pp, PE0_GEN_CTRL_1) |
				SRIS_MODE_MASK;
		dw_pcie_writel_ctrl(s32cc_pp, PE0_GEN_CTRL_1, val);
	}

	/* Enable writing dbi registers */
	dw_pcie_dbi_ro_wr_en(pcie);

	/* Enable direct speed change */
	val = dw_pcie_readl_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pcie, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Disable phase 2,3 equalization */
	disable_equalization(pcie);

	/* Make sure DBI registers are R/W - see dw_pcie_setup_rc */
	dw_pcie_dbi_ro_wr_en(pcie);
	dw_pcie_setup(pcie);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Make sure we use the coherency defaults (just in case the settings
	 * have been changed from their reset values
	 */
	s32cc_pcie_reset_mstr_ace(pcie);

	/* Test value for coherency control reg */
	dev_dbg(dev, "COHERENCY_CONTROL_3_OFF: 0x%08x\n",
		dw_pcie_readl_dbi(pcie, PORT_LOGIC_COHERENCY_CONTROL_3));

	/* Make sure DBI registers are R/W */
	dw_pcie_dbi_ro_wr_en(pcie);

	val = dw_pcie_readl_dbi(pcie, PORT_LOGIC_PORT_FORCE_OFF);
	val |= BIT(PCIE_DO_DESKEW_FOR_SRIS_BIT);
	dw_pcie_writel_dbi(pcie, PORT_LOGIC_PORT_FORCE_OFF, val);

	if (is_s32cc_pcie_rc(s32cc_pp->mode)) {
		/* Set max payload supported, 256 bytes and
		 * relaxed ordering.
		 */
		val = dw_pcie_readl_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS);
		val &= ~(BIT(CAP_EN_REL_ORDER_BIT) |
			GET_MASK_VALUE(CAP_MAX_PAYLOAD_SIZE_CS) |
			GET_MASK_VALUE(CAP_MAX_READ_REQ_SIZE));
		val |= BIT(CAP_EN_REL_ORDER_BIT) |
			BUILD_MASK_VALUE(CAP_MAX_PAYLOAD_SIZE_CS, 1) |
			BUILD_MASK_VALUE(CAP_MAX_READ_REQ_SIZE, 1),
		dw_pcie_writel_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS, val);

		/* Enable the IO space, Memory space, Bus master,
		 * Parity error, Serr and disable INTx generation
		 */
		dw_pcie_writel_dbi(pcie, PCIE_CTRL_TYPE1_STATUS_COMMAND_REG,
				   BIT(PCIE_SERREN_BIT) | BIT(PCIE_PERREN_BIT) |
				   BIT(PCIE_INT_EN_BIT) | BIT(PCIE_IO_EN_BIT) |
				   BIT(PCIE_MSE_BIT) | BIT(PCIE_BME_BIT));
		/* Test value */
		dev_dbg(dev, "PCIE_CTRL_TYPE1_STATUS_COMMAND_REG reg: 0x%08x\n",
			dw_pcie_readl_dbi(pcie,
					  PCIE_CTRL_TYPE1_STATUS_COMMAND_REG));

		/* Enable errors */
		val = dw_pcie_readl_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS);
		val |=  BIT(CAP_CORR_ERR_REPORT_EN_BIT) |
			BIT(CAP_NON_FATAL_ERR_REPORT_EN_BIT) |
			BIT(CAP_FATAL_ERR_REPORT_EN_BIT) |
			BIT(CAP_UNSUPPORT_REQ_REP_EN_BIT);
		dw_pcie_writel_dbi(pcie, CAP_DEVICE_CONTROL_DEVICE_STATUS, val);
	}

	val = dw_pcie_readl_dbi(pcie, PORT_GEN3_RELATED_OFF);
	val |= PCIE_EQ_PHASE_2_3;
	dw_pcie_writel_dbi(pcie, PORT_GEN3_RELATED_OFF, val);

	/* Disable writing dbi registers */
	dw_pcie_dbi_ro_wr_dis(pcie);

	s32cc_pcie_enable_ltssm(s32cc_pp);

	return 0;
}

static int init_pcie_phy(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct device *dev = pcie->dev;
	int ret;

	ret = phy_init(s32cc_pp->phy0);
	if (ret) {
		dev_err(dev, "Failed to init 'serdes_lane0' PHY\n");
		return ret;
	}

	ret = phy_set_mode_ext(s32cc_pp->phy0, PHY_MODE_PCIE,
			       s32cc_pp->phy_mode);
	if (ret) {
		dev_err(dev, "Failed to set mode on 'serdes_lane0' PHY\n");
		return ret;
	}

	ret = phy_power_on(s32cc_pp->phy0);
	if (ret) {
		dev_err(dev, "Failed to power on 'serdes_lane0' PHY\n");
		return ret;
	}

	/*
	 *	It is safe to consider that a null phy1 means phy1 uninitialized
	 *	and non-zero initialized.
	 *	So if phy1 is non-zero then reuse the handler.
	 */
	if (!s32cc_pp->phy1)
		s32cc_pp->phy1 = devm_phy_optional_get(dev, "serdes_lane1");

	ret = phy_init(s32cc_pp->phy1);
	if (ret) {
		dev_err(dev, "Failed to init 'serdes_lane1' PHY\n");
		return ret;
	}

	ret = phy_set_mode_ext(s32cc_pp->phy1, PHY_MODE_PCIE,
			       s32cc_pp->phy_mode);
	if (ret) {
		dev_err(dev, "Failed to set mode on 'serdes_lane1' PHY\n");
		return ret;
	}

	ret = phy_power_on(s32cc_pp->phy1);
	if (ret) {
		dev_err(dev, "Failed to power on 'serdes_lane1' PHY\n");
		return ret;
	}

	return 0;
}

static int deinit_pcie_phy(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct device *dev = pcie->dev;
	int ret;

	if (s32cc_pp->phy0) {
		ret = phy_power_off(s32cc_pp->phy0);
		if (ret) {
			dev_err(dev, "Failed to power off 'serdes_lane0' PHY\n");
			return ret;
		}

		ret = phy_exit(s32cc_pp->phy0);
		if (ret) {
			dev_err(dev, "Failed to exit 'serdes_lane0' PHY\n");
			return ret;
		}
	}

	if (s32cc_pp->phy1) {
		ret = phy_power_off(s32cc_pp->phy1);
		if (ret) {
			dev_err(dev, "Failed to power off 'serdes_lane1' PHY\n");
			return ret;
		}

		ret = phy_exit(s32cc_pp->phy1);
		if (ret) {
			dev_err(dev, "Failed to exit 'serdes_lane1' PHY\n");
			return ret;
		}
	}

	return 0;
}

static void s32cc_pcie_pme_turnoff(struct s32cc_pcie *s32cc_pp)
{
	/* TODO: We may want to transition to L2 low power state here, after
	 *	removal of power and clocks.
	 *	This needs more investigation (see SerDes manual, chapter
	 *	"L2 and L3 power down entry and exit conditions").
	 *	We're able to disable and power off the PHYs from this driver,
	 *	however clocks are controlled from the SerDes driver.
	 *
	 *	For now, just disable LTSSM.
	 */

	s32cc_pcie_disable_ltssm(s32cc_pp);
}

int deinit_controller(struct s32cc_pcie *s32cc_pp)
{
	/* Other drivers assert controller reset, then disable phys,
	 *	then de-assert reset and disable clocks. On our platform
	 *	reset and clocks management is done in the SerDes driver,
	 *	so we need to investigate how and whether we should do
	 *	that here.
	 */

	s32cc_pcie_pme_turnoff(s32cc_pp);
	return deinit_pcie_phy(s32cc_pp);
}

static int wait_phy_data_link(struct s32cc_pcie *s32cc_pp)
{
	bool has_link;
	int ret = read_poll_timeout(has_data_phy_link, has_link, has_link,
			PCIE_LINK_WAIT_US, PCIE_LINK_TIMEOUT_US, 0, s32cc_pp);

	if (ret)
		dev_info(s32cc_pp->pcie.dev, "Failed to stabilize PHY link\n");

	return ret;
}

static void s32cc_pcie_downstream_dev_to_D0(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = NULL;
	struct pcie_port *pp = NULL;
	struct pci_bus *root_bus = NULL;
	struct pci_dev *pdev;

	if (s32cc_pp)
		pcie = &s32cc_pp->pcie;
	if (pcie)
		pp = &pcie->pp;

	/* Check if we did manage to initialize the host */
	if (!pp || IS_ERR(pp) || IS_ERR(pp->bridge) || IS_ERR(pp->bridge->bus))
		return;

	/*
	 * link doesn't go into L2 state with some of the EndPoints
	 * if they are not in D0 state. So, we need to make sure that immediate
	 * downstream devices are in D0 state before sending PME_TurnOff to put
	 * link into L2 state.
	 */

	root_bus = s32cc_get_child_downstream_bus(pp->bridge->bus);
	if (IS_ERR(root_bus)) {
		dev_err(pcie->dev, "Failed to find downstream devices\n");
		return;
	}

	list_for_each_entry(pdev, &root_bus->devices, bus_list) {
		if (PCI_SLOT(pdev->devfn) == 0) {
			if (pci_set_power_state(pdev, PCI_D0))
				dev_err(pcie->dev,
					"Failed to transition %s to D0 state\n",
					dev_name(&pdev->dev));
		}
	}
}

static int s32cc_pcie_deinit_controller_host(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = NULL;
	struct pcie_port *pp = NULL;

	if (s32cc_pp) {
		pcie = &s32cc_pp->pcie;

		/* TODO: investigate if this is really necessary*/
		s32cc_pcie_downstream_dev_to_D0(s32cc_pp);

		if (pcie)
			pp = &pcie->pp;
		if (pp)
			dw_pcie_host_deinit(pp);

		return deinit_controller(s32cc_pp);
	}

	return 0;
}

static int s32cc_pcie_init_controller(struct s32cc_pcie *s32cc_pp)
{
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct pcie_port *pp = &pcie->pp;
	int ret = 0;

	s32cc_pcie_disable_ltssm(s32cc_pp);

	ret = init_pcie_phy(s32cc_pp);
	if (ret)
		return ret;

	ret = init_pcie(s32cc_pp);
	if (ret)
		return ret;

	/* Only wait for link if RC */
	if (is_s32cc_pcie_rc(s32cc_pp->mode)) {
		ret = wait_phy_data_link(s32cc_pp);
		if (ret) {
			if (!phy_validate(s32cc_pp->phy0, PHY_MODE_PCIE, 0, NULL)) {
				dev_err(pcie->dev, "Failed to get link up with EP connected\n");
				return ret;
			}
		}
	}

	dev_info(pcie->dev, "Configuring as %s\n",
		 s32cc_pcie_ep_rc_mode_str(s32cc_pp->mode));

	if (s32cc_pp->has_msi_parent)
		pp->ops = &s32cc_pcie_host_ops2;
	else
		pp->ops = &s32cc_pcie_host_ops;

	return 0;
}

int s32cc_pcie_config_common(struct s32cc_pcie *s32cc_pp,
					struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *phy;
	int ret = 0;

	phy = devm_phy_get(dev, "serdes_lane0");
	if (IS_ERR(phy)) {
		if (PTR_ERR(phy) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get 'serdes_lane0' PHY\n");

		return PTR_ERR(phy);
	}

	s32cc_pp->phy0 = phy;

	ret = s32cc_pcie_init_controller(s32cc_pp);
	if (ret)
		return ret;

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to get runtime sync for PCIe dev: %d\n",
			ret);
		goto fail_pm_get_sync;
	}

	return ret;

fail_pm_get_sync:
	pm_runtime_disable(dev);
	return ret;
}

static int s32cc_pcie_config_host(struct s32cc_pcie *s32cc_pp,
					struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct pcie_port *pp = &pcie->pp;
	int ret = 0;

	ret = s32cc_pcie_config_common(s32cc_pp, pdev);
	if (ret)
		return ret;

	/* MSI configuration for RC */
	if (IS_ENABLED(CONFIG_PCI_MSI) && !s32cc_has_msi_parent(pp)) {
		ret = s32cc_pcie_config_irq(&pp->msi_irq, "msi", pdev,
					      s32cc_pcie_msi_handler, pp);
		if (ret) {
			dev_err(dev, "failed to request msi irq\n");
			return ret;
		}
	}

	ret = s32cc_add_pcie_port(pp);
	if (ret)
		goto fail_host_init;

	ret = s32cc_pcie_config_hp_irq(s32cc_pp, pdev);
	if (ret)
		goto fail_host_init;
	ret = s32cc_enable_hotplug_cap(pcie);
	if (ret) {
		dev_err(dev, "Failed to enable hotplug capability\n");
		goto fail_host_init;
	}

	s32cc_pcie_enable_hot_unplug_irq(s32cc_pp);

	/* TODO: Init debugfs here */

	return ret;

fail_host_init:
	s32cc_pcie_deinit_controller_host(s32cc_pp);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	return ret;
}

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

	match = of_match_device(s32cc_pcie_of_match, dev);
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

	ret = s32cc_pcie_dt_init_common(pdev, s32cc_pp);
	if (ret)
		goto err;

	if (is_s32cc_pcie_rc(s32cc_pp->mode)) {
		ret = s32cc_pcie_config_host(s32cc_pp, pdev);
	} else {
		dev_err(dev, "Invalid PCIe host operating mode\n");
		ret = -EINVAL;
	}

err:
	if (ret) {
		dev_err(dev, "Failed to set PCIe host settings\n");
		platform_set_drvdata(pdev, NULL);
	}

	dw_pcie_dbi_ro_wr_dis(pcie);
	return ret;
}

#if (IS_ENABLED(CONFIG_PM_SLEEP))

int s32cc_pcie_suspend(struct device *dev)
{
	struct s32cc_pcie *s32cc_pp = dev_get_drvdata(dev);
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct pci_bus *bus, *root_bus;

	/* Save MSI interrupt vector */
	s32cc_pp->msi_ctrl_int = dw_pcie_readl_dbi(pcie,
			PORT_MSI_CTRL_INT_0_EN_OFF);

	if (is_s32cc_pcie_rc(s32cc_pp->mode)) {
		/* Disable Hot-Plug irq */
		s32cc_pcie_disable_hot_plug_irq(pcie);
		/* Disable Hot-Unplug irq */
		s32cc_pcie_disable_hot_unplug_irq(s32cc_pp);

		s32cc_pcie_downstream_dev_to_D0(s32cc_pp);

		bus = pp->bridge->bus;
		root_bus = s32cc_get_child_downstream_bus(bus);
		if (!IS_ERR(root_bus))
			pci_walk_bus(root_bus, pci_dev_set_disconnected, NULL);

		pci_stop_root_bus(bus);
		pci_remove_root_bus(bus);
	}

	return deinit_controller(s32cc_pp);
}

int s32cc_pcie_resume(struct device *dev)
{
	struct s32cc_pcie *s32cc_pp = dev_get_drvdata(dev);
	struct dw_pcie *pcie = &s32cc_pp->pcie;
	struct pcie_port *pp = &pcie->pp;
	int ret = 0;

	/* TODO: we should keep link state (bool) in struct s32cc_pcie
	 *	and not do anything on resume if there was no link.
	 *	Otherwise resuming is slow since it will get link timeouts.
	 */

	ret = s32cc_pcie_init_controller(s32cc_pp);
	if (ret < 0)
		return ret;

	if (is_s32cc_pcie_rc(s32cc_pp->mode)) {
		ret = s32cc_pcie_host_init(pp);
		if (ret < 0) {
			dev_err(dev, "Failed to init host: %d\n", ret);
			goto fail_host_init;
		}

		ret = pci_host_probe(pp->bridge);
		if (ret)
			return ret;
	}

	/* Restore MSI interrupt vector */
	dw_pcie_writel_dbi(pcie, PORT_MSI_CTRL_INT_0_EN_OFF,
			   s32cc_pp->msi_ctrl_int);

	if (is_s32cc_pcie_rc(s32cc_pp->mode)) {
		/* Enable Hot-Plug capability */
		ret = s32cc_enable_hotplug_cap(pcie);
		if (ret) {
			dev_err(dev, "Failed to enable hotplug capability\n");
			goto fail_host_init;
		}

		/* Enable Hot-Unplug irq */
		s32cc_pcie_enable_hot_unplug_irq(s32cc_pp);
	}

	return 0;

fail_host_init:
	return deinit_controller(s32cc_pp);
}

static const struct dev_pm_ops s32cc_pcie_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s32cc_pcie_suspend,
				s32cc_pcie_resume)
};
#endif

static const struct s32cc_pcie_data rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct of_device_id s32cc_pcie_of_match[] = {
	{ .compatible = "nxp,s32cc-pcie", .data = &rc_of_data },
	{},
};
MODULE_DEVICE_TABLE(of, s32cc_pcie_of_match);

static struct platform_driver s32cc_pcie_driver = {
	.driver = {
		.name	= "s32cc-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = s32cc_pcie_of_match,
#if (IS_ENABLED(CONFIG_PM_SLEEP))
		.pm = &s32cc_pcie_pm_ops,
#endif
	},
	.probe = s32cc_pcie_probe,
	.shutdown = s32cc_pcie_shutdown,
};

module_platform_driver(s32cc_pcie_driver);

MODULE_AUTHOR("Ionut Vicovan <Ionut.Vicovan@nxp.com>");
MODULE_DESCRIPTION("NXP S32CC PCIe Host controller driver");
MODULE_LICENSE("GPL v2");
