// SPDX-License-Identifier: GPL-2.0
/**
 * SERDES driver for S32GEN1 SoCs
 *
 * Copyright 2021 NXP
 */
#include <dt-bindings/phy/phy.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/pcs/fsl-s32gen1-xpcs.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/processor.h>
#include <linux/reset.h>
#include <linux/stringify.h>

#define SERDES_MAX_LANES 2

#define SERDES_LINE(TYPE, INSTANCE)\
	{ \
		.mode = (TYPE), \
		.instance = (INSTANCE), \
	}

#define PCIE_LANE(N) SERDES_LINE(PHY_MODE_PCIE, N)
#define XPCS_LANE(N) SERDES_LINE(PHY_MODE_ETHERNET, N)

#define PCIE_PHY_GEN_CTRL	(0x0)
#define  REF_USE_PAD_MASK	BIT(17)
#define PCIE_PHY_MPLLA_CTRL	(0x10)
#define  MPLLA_STATE_MASK	BIT(31)
#define  MPLL_STATE_MASK	BIT(30)
#define SS_RW_REG_0		(0xf0)
#define  SUBMODE_MASK		(0x7)
#define  CLKEN_MASK		BIT(23)
#define  PHY0_CR_PARA_SEL_MASK	BIT(9)

#define PHY_REG_ADDR		(0x0)
#define  PHY_REG_EN		BIT(31)
#define PHY_REG_DATA		(0x4)

#define RAWLANE0_DIG_PCS_XF_RX_EQ_DELTA_IQ_OVRD_IN	(0x3019)
#define RAWLANE1_DIG_PCS_XF_RX_EQ_DELTA_IQ_OVRD_IN	(0x3119)

#define SERDES_LOCK_TIMEOUT_MS	1

#define EXTERNAL_CLK_NAME	"ext"
#define INTERNAL_CLK_NAME	"ref"

#define SERDES_PCIE_FREQ	100000000

struct serdes_lane_conf {
	enum phy_mode mode;
	u8 instance; /** Instance ID (e.g PCIE0, XPCS1) */
};

struct serdes_conf {
	struct serdes_lane_conf lanes[SERDES_MAX_LANES];
};

struct pcie_ctrl {
	struct reset_control *rst;
	void __iomem *phy_base;
	bool initialized_phy;
};

struct serdes_ctrl {
	struct reset_control *rst;
	void __iomem *ss_base;
	struct clk_bulk_data *clks;
	int nclks;
	u32 ss_mode;
	bool ext_clk;
};

struct xpcs_ctrl {
	struct s32gen1_xpcs *phys[2];
	const struct s32gen1_xpcs_ops *ops;
	void __iomem *base0, *base1;
	bool powered_on[2];
	bool initialized_clks;
};

struct serdes {
	struct pcie_ctrl pcie;
	struct serdes_ctrl ctrl;
	struct xpcs_ctrl xpcs;
	struct device *dev;
	struct phy *phys[SERDES_MAX_LANES];
	u8 lanes_status;
};

static void mark_configured_lane(struct serdes *serdes, u32 lane)
{
	serdes->lanes_status |= BIT(lane);
}

static bool is_lane_configured(struct serdes *serdes, u32 lane)
{
	return !!(serdes->lanes_status & BIT(lane));
}

static int serdes_phy_reset(struct phy *p)
{
	return 0;
}

static bool pcie_phy_is_locked(struct serdes *serdes)
{
	u32 mplla = readl(serdes->ctrl.ss_base + PCIE_PHY_MPLLA_CTRL);
	const u32 mask = MPLLA_STATE_MASK | MPLL_STATE_MASK;

	return (mplla & mask) == mask;
}

static bool locked_phy_or_timeout(struct serdes *serdes, ktime_t timeout)
{
	ktime_t cur = ktime_get();

	return pcie_phy_is_locked(serdes) || ktime_after(cur, timeout);
}

static void pcie_phy_write(struct serdes *serdes, u32 reg, u32 val)
{
	writel(PHY_REG_EN, serdes->pcie.phy_base + PHY_REG_ADDR);
	writel(reg | PHY_REG_EN, serdes->pcie.phy_base + PHY_REG_ADDR);
	udelay(100);
	writel(val, serdes->pcie.phy_base + PHY_REG_DATA);
	udelay(100);
}

static struct clk *get_serdes_clk(struct serdes *serdes, const char *name)
{
	int i;

	for (i = 0; i < serdes->ctrl.nclks; i++) {
		if (!strcmp(serdes->ctrl.clks[i].id, name))
			return serdes->ctrl.clks[i].clk;
	}

	return NULL;
}

static int get_clk_rate(struct serdes *serdes, unsigned long *rate)
{
	struct device *dev = serdes->dev;
	struct clk *clk;

	if (serdes->ctrl.ext_clk)
		clk = get_serdes_clk(serdes, EXTERNAL_CLK_NAME);
	else
		clk = get_serdes_clk(serdes, INTERNAL_CLK_NAME);

	if (!clk) {
		dev_err(dev, "Failed to determine SerDes clock\n");
		return -EINVAL;
	}

	*rate = clk_get_rate(clk);
	return 0;
}

static int check_pcie_clk(struct serdes *serdes)
{
	struct device *dev = serdes->dev;
	unsigned long rate;
	int ret;

	ret = get_clk_rate(serdes, &rate);
	if (ret)
		return ret;

	if (rate != SERDES_PCIE_FREQ) {
		dev_err(dev, "PCIe PHY cannot operate at %lu HZ\n", rate);
		return -EINVAL;
	}

	return 0;
}

static int pci_phy_power_on_common(struct phy *p)
{
	ktime_t timeout = ktime_add_ms(ktime_get(), SERDES_LOCK_TIMEOUT_MS);
	struct serdes *serdes = phy_get_drvdata(p);
	struct serdes_ctrl *sctrl = &serdes->ctrl;
	struct pcie_ctrl *pcie = &serdes->pcie;
	u32 ctrl, reg0;
	int ret;

	if (pcie->initialized_phy)
		return 0;

	ret = check_pcie_clk(serdes);
	if (ret)
		return ret;

	ctrl = readl(sctrl->ss_base + PCIE_PHY_GEN_CTRL);

	if (sctrl->ext_clk)
		ctrl |= REF_USE_PAD_MASK;
	else
		ctrl &= ~REF_USE_PAD_MASK;

	/* Monitor Serdes MPLL state */
	writel(ctrl, sctrl->ss_base + PCIE_PHY_GEN_CTRL);

	spin_until_cond(locked_phy_or_timeout(serdes, timeout));
	if (!pcie_phy_is_locked(serdes)) {
		dev_err(serdes->dev, "Failed to lock PCIE phy\n");
		return -ETIMEDOUT;
	}

	/* Set PHY register access to CR interface */
	reg0 = readl(sctrl->ss_base + SS_RW_REG_0);
	reg0 |=  PHY0_CR_PARA_SEL_MASK;
	writel(reg0, sctrl->ss_base + SS_RW_REG_0);

	pcie->initialized_phy = true;
	return 0;
}

static int pcie_phy_power_on(struct phy *p)
{
	struct serdes *serdes = phy_get_drvdata(p);
	u32 iq_ovrd_in;
	int ret;

	ret = pci_phy_power_on_common(p);
	if (ret)
		return ret;

	/* RX_EQ_DELTA_IQ_OVRD enable and override value for PCIe lanes */
	if (p->id == 0)
		iq_ovrd_in = RAWLANE0_DIG_PCS_XF_RX_EQ_DELTA_IQ_OVRD_IN;
	else
		iq_ovrd_in = RAWLANE1_DIG_PCS_XF_RX_EQ_DELTA_IQ_OVRD_IN;

	pcie_phy_write(serdes, iq_ovrd_in, 0x3);
	pcie_phy_write(serdes, iq_ovrd_in, 0x13);

	return 0;
}

static int xpcs_phy_init(struct serdes *serdes, int id)
{
	struct serdes_ctrl *ctrl = &serdes->ctrl;
	struct xpcs_ctrl *xpcs = &serdes->xpcs;
	struct device *dev = serdes->dev;
	void __iomem *base;
	unsigned long rate;
	int ret;

	if (xpcs->phys[id])
		return 0;

	if (!id)
		base = xpcs->base0;
	else
		base = xpcs->base1;

	ret = get_clk_rate(serdes, &rate);
	if (ret)
		return ret;

	return xpcs->ops->init(&xpcs->phys[id], dev, id, base,
			       ctrl->ext_clk, rate);
}

static int xpcs_phy_power_on(struct serdes *serdes, int id)
{
	struct xpcs_ctrl *xpcs = &serdes->xpcs;
	int ret;

	if (xpcs->powered_on[id])
		return 0;

	ret = xpcs->ops->power_on(xpcs->phys[id]);
	if (ret)
		dev_err(serdes->dev, "Failed to power on XPCS%d\n", id);
	else
		xpcs->powered_on[id] = true;

	return ret;
}

static bool is_xpcs_rx_stable(struct serdes *serdes, int id)
{
	struct xpcs_ctrl *xpcs = &serdes->xpcs;

	return xpcs->ops->has_valid_rx(xpcs->phys[id]);
}

static int xpcs_init_clks(struct serdes *serdes)
{
	struct serdes_ctrl *ctrl = &serdes->ctrl;
	struct xpcs_ctrl *xpcs = &serdes->xpcs;
	int ret, order[2], i, xpcs_id;

	if (xpcs->initialized_clks)
		return 0;

	switch (ctrl->ss_mode) {
	case 1:
	case 2:
		return 0;
	case 3:
		order[0] = 1;
		order[1] = 0;
		break;
	case 4:
		order[0] = 0;
		order[1] = 1;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(order); i++) {
		xpcs_id = order[i];

		ret = xpcs_phy_init(serdes, xpcs_id);
		if (ret)
			return ret;

		ret = xpcs_phy_power_on(serdes, xpcs_id);
		if (ret)
			return ret;

		if (!is_xpcs_rx_stable(serdes, xpcs_id)) {
			dev_info(serdes->dev, "Unstable RX detected on XPCS%d\n",
				 xpcs_id);
			return 0;
		}

		ret = xpcs->ops->init_mplla(xpcs->phys[xpcs_id]);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(order); i++) {
		ret = xpcs->ops->vreset(xpcs->phys[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(order); i++) {
		ret = xpcs->ops->wait_vreset(xpcs->phys[i]);
		if (ret)
			return ret;
	}

	xpcs->initialized_clks = true;
	return 0;
}

static void xpcs_phy_release(struct phy *p)
{
	struct serdes *serdes = phy_get_drvdata(p);
	struct xpcs_ctrl *xpcs = &serdes->xpcs;
	int id = p->id;

	xpcs->ops->release(xpcs->phys[id]);
	xpcs->phys[id] = NULL;
}

static int serdes_phy_init(struct phy *p)
{
	struct serdes *serdes = phy_get_drvdata(p);

	if (p->attrs.mode == PHY_MODE_PCIE)
		return 0;

	if (p->attrs.mode == PHY_MODE_ETHERNET)
		return xpcs_phy_init(serdes, p->id);

	return -EINVAL;
}

static void serdes_phy_release(struct phy *p)
{
	if (p->attrs.mode == PHY_MODE_ETHERNET)
		xpcs_phy_release(p);
}

static int serdes_phy_power_on(struct phy *p)
{
	struct serdes *serdes = phy_get_drvdata(p);

	if (p->attrs.mode == PHY_MODE_PCIE)
		return pcie_phy_power_on(p);

	if (p->attrs.mode == PHY_MODE_ETHERNET)
		return xpcs_phy_power_on(serdes, p->id);

	return 0;
}

static int serdes_phy_power_off(struct phy *p)
{
	return 0;
}

struct s32gen1_xpcs *s32gen1_phy2xpcs(struct phy *phy)
{
	struct serdes *serdes = phy_get_drvdata(phy);
	struct xpcs_ctrl *xpcs = &serdes->xpcs;

	return xpcs->phys[phy->id];
}
EXPORT_SYMBOL_GPL(s32gen1_phy2xpcs);

static int xpcs_phy_configure(struct phy *phy, struct phylink_link_state *state)
{
	struct serdes *serdes = phy_get_drvdata(phy);
	struct device *dev = serdes->dev;
	struct xpcs_ctrl *xpcs = &serdes->xpcs;
	int id = phy->id;
	int ret;

	if (state->interface != PHY_INTERFACE_MODE_2500BASEX) {
		ret = xpcs_init_clks(serdes);
		if (ret) {
			dev_err(dev, "Failed to initialize XPCS clocks\n");
			return ret;
		}
	}

	ret = xpcs->ops->config(xpcs->phys[id], state);
	if (!ret)
		xpcs->ops->reset_rx(xpcs->phys[id]);

	return ret;

}

static int serdes_phy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	int ret = -EINVAL;

	if (phy->attrs.mode == PHY_MODE_ETHERNET)
		ret = xpcs_phy_configure(phy,
					 (struct phylink_link_state *)opts);

	return ret;

}

static const struct phy_ops serdes_ops = {
	.reset		= serdes_phy_reset,
	.init		= serdes_phy_init,
	.power_on	= serdes_phy_power_on,
	.power_off	= serdes_phy_power_off,
	.release	= serdes_phy_release,
	.configure	= serdes_phy_configure,
	.owner		= THIS_MODULE,
};

static const struct serdes_conf serdes_mux_table[] = {
	/* Mode 0 */
	{ .lanes = { [0] = PCIE_LANE(0), [1] = PCIE_LANE(1) }, },
	/* Mode 1 */
	{ .lanes = { [0] = PCIE_LANE(0), [1] = XPCS_LANE(0), }, },
	/* Mode 2 */
	{ .lanes = { [0] = PCIE_LANE(0), [1] = XPCS_LANE(1), }, },
	/* Mode 3 */
	{ .lanes = { [0] = XPCS_LANE(0), [1] = XPCS_LANE(1), }, },
	/* Mode 4 */
	{ .lanes = { [0] = XPCS_LANE(0), [1] = XPCS_LANE(1), }, },
};


static int check_lane_selection(struct serdes *serdes,
				u32 phy_type, u32 instance,
				u32 lane_id, enum phy_mode *mode)
{
	struct serdes_ctrl *ctrl = &serdes->ctrl;
	const struct serdes_conf *conf = &serdes_mux_table[ctrl->ss_mode];
	const struct serdes_lane_conf *lane_conf;
	struct device *dev = serdes->dev;
	const char *phy_name;

	if (lane_id >= SERDES_MAX_LANES) {
		dev_err(dev, "Invalid lane : %u\n", lane_id);
		return -EINVAL;
	}

	switch (phy_type) {
	case PHY_TYPE_PCIE:
		*mode = PHY_MODE_PCIE;
		phy_name = __stringify_1(PHY_MODE_PCIE);
		break;
	case PHY_TYPE_XPCS:
		*mode = PHY_MODE_ETHERNET;
		phy_name = __stringify_1(PHY_MODE_ETHERNET);
		break;
	default:
		dev_err(dev, "Invalid PHY type : %u\n", phy_type);
		return -EINVAL;
	}

	if (is_lane_configured(serdes, lane_id)) {
		dev_err(dev, "Lane %u is already configured\n", lane_id);
		return -EINVAL;
	}

	lane_conf = &conf->lanes[lane_id];

	if (lane_conf->mode != *mode) {
		dev_err(dev, "Invalid %u mode applied on SerDes lane %d. Expected mode %u\n",
			*mode, lane_id, lane_conf->mode);
		return -EINVAL;
	}

	if (lane_conf->mode != PHY_MODE_PCIE &&
	    lane_conf->instance != instance) {
		dev_err(dev, "PHY %s instance %u cannot be applied on lane %u using SerDes mode %u)\n",
			phy_name, instance, lane_id, serdes->ctrl.ss_mode);
		return -EINVAL;
	}

	mark_configured_lane(serdes, lane_id);
	return 0;
}

static struct phy *serdes_xlate(struct device *dev,
				struct of_phandle_args *args)
{
	int ret;
	struct phy *phy;
	struct serdes *serdes;
	u32 phy_type = args->args[0];
	u32 instance = args->args[1];
	uint32_t lane_id = args->args[2];
	enum phy_mode mode;

	serdes = dev_get_drvdata(dev);
	if (!serdes)
		return ERR_PTR(-EINVAL);

	ret = check_lane_selection(serdes, phy_type, instance, lane_id, &mode);
	if (ret)
		return ERR_PTR(ret);

	phy = serdes->phys[lane_id];
	phy->id = instance;
	phy->attrs.mode = mode;

	return phy;
}

static int assert_reset(struct serdes *serdes)
{
	struct device *dev = serdes->dev;
	int ret;

	ret = reset_control_assert(serdes->pcie.rst);
	if (ret) {
		dev_err(dev, "Failed to assert PCIE reset: %d\n", ret);
		return ret;
	}

	ret = reset_control_assert(serdes->ctrl.rst);
	if (ret) {
		dev_err(dev, "Failed to assert SERDES reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int deassert_reset(struct serdes *serdes)
{
	struct device *dev = serdes->dev;
	int ret;

	ret = reset_control_deassert(serdes->pcie.rst);
	if (ret) {
		dev_err(dev, "Failed to assert PCIE reset: %d\n", ret);
		return ret;
	}

	ret = reset_control_deassert(serdes->ctrl.rst);
	if (ret) {
		dev_err(dev, "Failed to assert SERDES reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int init_serdes(struct serdes *serdes)
{
	struct serdes_ctrl *ctrl = &serdes->ctrl;
	u32 reg0;
	int ret;

	ret = assert_reset(serdes);
	if (ret)
		return ret;

	reg0 = readl(ctrl->ss_base + SS_RW_REG_0);
	reg0 &= ~SUBMODE_MASK;
	reg0 |= ctrl->ss_mode;
	writel(reg0, ctrl->ss_base + SS_RW_REG_0);

	reg0 = readl(ctrl->ss_base + SS_RW_REG_0);
	if (ctrl->ext_clk)
		reg0 &= ~CLKEN_MASK;
	else
		reg0 |= CLKEN_MASK;

	writel(reg0, ctrl->ss_base + SS_RW_REG_0);

	udelay(100);

	ret = deassert_reset(serdes);
	if (ret)
		return ret;

	dev_info(serdes->dev, "Using mode %d for SerDes subsystem\n",
		 ctrl->ss_mode);

	return 0;
}

static int ss_dt_init(struct platform_device *pdev, struct serdes *serdes)
{
	struct serdes_ctrl *ctrl = &serdes->ctrl;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	ret = of_property_read_u32(dev->of_node, "fsl,sys-mode",
				   &ctrl->ss_mode);
	if (ret) {
		dev_err(dev, "Failed to get SerDes subsystem mode\n");
		return -EINVAL;
	}

	if (ctrl->ss_mode >= ARRAY_SIZE(serdes_mux_table)) {
		dev_err(dev, "Invalid SerDes subsystem mode %u\n",
			ctrl->ss_mode);
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ss_pcie");
	if (!res) {
		dev_err(dev, "Missing 'ss_pcie' reg region.\n");
		return -EIO;
	}

	ctrl->ss_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!ctrl->ss_base) {
		dev_err(dev, "Failed to map 'ss_pcie'\n");
		return -ENOMEM;
	}

	ctrl->rst = devm_reset_control_get(dev, "serdes");
	if (IS_ERR(ctrl->rst)) {
		if (PTR_ERR(ctrl->rst) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get 'serdes' reset control\n");
		return PTR_ERR(ctrl->rst);
	}

	ctrl->nclks = devm_clk_bulk_get_all(dev, &ctrl->clks);
	if (ctrl->nclks < 1) {
		dev_err(dev, "Failed to get SerDes clocks\n");
		return ctrl->nclks;
	}

	ret = clk_bulk_prepare_enable(ctrl->nclks, ctrl->clks);
	if (ret) {
		dev_err(dev, "Failed to enable SerDes clocks\n");
		return ctrl->nclks;
	}

	if (get_serdes_clk(serdes, EXTERNAL_CLK_NAME))
		ctrl->ext_clk = true;

	return ret;
}

static int pcie_dt_init(struct platform_device *pdev, struct serdes *serdes)
{
	struct pcie_ctrl *pcie = &serdes->pcie;
	struct device *dev = &pdev->dev;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pcie_phy");
	if (!res) {
		dev_err(dev, "Missing 'pcie_phy' reg region.\n");
		return -EIO;
	}

	pcie->phy_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!pcie->phy_base) {
		dev_err(dev, "Failed to map 'ss_pcie'\n");
		return -ENOMEM;
	}

	pcie->rst = devm_reset_control_get(dev, "pcie");
	if (IS_ERR(pcie->rst)) {
		dev_err(dev, "Failed to get 'pcie' reset control\n");
		return PTR_ERR(pcie->rst);
	}

	return 0;
}

static int xpcs_dt_init(struct platform_device *pdev, struct serdes *serdes)
{
	struct xpcs_ctrl *xpcs = &serdes->xpcs;
	struct device *dev = &pdev->dev;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "xpcs0");
	if (!res) {
		dev_err(dev, "Missing 'xpcs0' reg region.\n");
		return -EIO;
	}

	xpcs->base0 = devm_ioremap(dev, res->start, resource_size(res));
	if (!xpcs->base0) {
		dev_err(dev, "Failed to map 'xpcs0'\n");
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "xpcs1");
	if (!res) {
		dev_err(dev, "Missing 'xpcs1' reg region.\n");
		return -EIO;
	}

	xpcs->base1 = devm_ioremap(dev, res->start, resource_size(res));
	if (!xpcs->base1) {
		dev_err(dev, "Failed to map 'xpcs1'\n");
		return -ENOMEM;
	}

	xpcs->ops = s32gen1_xpcs_get_ops();

	return 0;
}

static int serdes_probe(struct platform_device *pdev)
{
	struct serdes *serdes;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	int ret;
	size_t i;

	serdes = devm_kzalloc(dev, sizeof(*serdes), GFP_KERNEL);
	if (!serdes)
		return -ENOMEM;

	platform_set_drvdata(pdev, serdes);
	serdes->dev = dev;

	for (i = 0; i < ARRAY_SIZE(serdes->phys); i++) {
		serdes->phys[i] = devm_phy_create(dev, NULL, &serdes_ops);
		if (IS_ERR(serdes->phys[i]))
			return PTR_ERR(serdes->phys[i]);
		phy_set_drvdata(serdes->phys[i], serdes);
		serdes->phys[i]->id = i;
	}

	ret = ss_dt_init(pdev, serdes);
	if (ret)
		return ret;

	ret = pcie_dt_init(pdev, serdes);
	if (ret)
		goto disable_clks;

	ret = xpcs_dt_init(pdev, serdes);
	if (ret)
		goto disable_clks;

	ret = init_serdes(serdes);
	if (ret)
		goto disable_clks;

	phy_provider = devm_of_phy_provider_register(dev, serdes_xlate);
	if (IS_ERR(phy_provider))
		ret = PTR_ERR(phy_provider);

disable_clks:
	if (ret)
		clk_bulk_disable_unprepare(serdes->ctrl.nclks,
					   serdes->ctrl.clks);

	return 0;
}

static int serdes_remove(struct platform_device *pdev)
{
	struct serdes *serdes = platform_get_drvdata(pdev);

	clk_bulk_disable_unprepare(serdes->ctrl.nclks, serdes->ctrl.clks);
	return 0;
}

static const struct of_device_id serdes_match[] = {
	{
		.compatible = "fsl,s32gen1-serdes",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, serdes_match);

static struct platform_driver serdes_driver = {
	.probe		= serdes_probe,
	.remove		= serdes_remove,
	.driver		= {
		.name	= "phy-s32gen1-serdes",
		.of_match_table = serdes_match,
	},
};
module_platform_driver(serdes_driver);

MODULE_AUTHOR("Ghennadi Procopciuc <ghennadi.procopciuc@nxp.com>");
MODULE_DESCRIPTION("S32GEN1 SERDES driver");
MODULE_LICENSE("GPL v2");
