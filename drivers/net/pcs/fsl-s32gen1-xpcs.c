// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright 2021 NXP
 */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/pcs/fsl-s32gen1-xpcs.h>
#include <linux/processor.h>
#include <linux/regmap.h>

#define XPCS_TIMEOUT_MS		300

#define SR_MII_CTRL				0x1F0000U
#define   AN_ENABLE				BIT(12)
#define   SR_RST				BIT(15)
#define   SS13					BIT(13)
#define   DUPLEX_MODE				BIT(8)
#define   SS6					BIT(6)
#define SR_MII_STS				0x1F0001U
#define   LINK_STS				BIT(2)
#define   AN_ABL				BIT(3)
#define   AN_CMPL				BIT(5)
#define SR_MII_DEV_ID1				0x1F0002U
#define SR_MII_DEV_ID2				0x1F0003U
#define SR_MII_EXT_STS				0x1F000FU
#define   CAP_1G_T_FD				BIT(13)
#define   CAP_1G_T_HD				BIT(12)
#define VR_MII_DIG_CTRL1			0x1F8000U
#define   BYP_PWRUP				BIT(1)
#define   EN_2_5G_MODE				BIT(2)
#define   MAC_AUTO_SW				BIT(9)
#define   CS_EN					BIT(10)
#define   PWRSV					BIT(11)
#define   EN_VSMMD1				BIT(13)
#define   R2TLBE				BIT(14)
#define   VR_RST				BIT(15)
#define VR_MII_AN_CTRL				0x1F8001U
#define VR_MII_AN_INTR_STS			0x1F8002U
#define VR_MII_DBG_CTRL				0x1F8005U
#define   SUPPRESS_LOS_DET			BIT(4)
#define   RX_DT_EN_CTL				BIT(6)
#define VR_MII_LINK_TIMER_CTRL			0x1F800AU
#define VR_MII_DIG_STS				0x1F8010U
#define   PSEQ_STATE_OFF			(2)
#define   PSEQ_STATE_MASK			(0x7 << PSEQ_STATE_OFF)
#define   PSEQ_STATE(val)			(((val) & PSEQ_STATE_MASK) >> \
						 PSEQ_STATE_OFF)
#define     POWER_GOOD_STATE			0x4
#define VR_MII_GEN5_12G_16G_TX_RATE_CTRL	0x1F8034U
#define   TX0_RATE_OFF				0
#define   TX0_RATE_MASK				0x7
#define     TX0_BAUD_DIV_1			0
#define     TX0_BAUD_DIV_4			2
#define VR_MII_GEN5_12G_16G_TX_EQ_CTRL0		0x1F8036U
#define   TX_EQ_MAIN_OFF			8
#define   TX_EQ_MAIN_MASK			(0x3F << TX_EQ_MAIN_OFF)
#define VR_MII_CONSUMER_10G_TX_TERM_CTRL	0x1F803CU
#define   TX0_TERM_OFF				0
#define   TX0_TERM_MASK				0x7
#define VR_MII_GEN5_12G_16G_RX_GENCTRL1		0x1F8051U
#define   RX_RST_0				BIT(4)
#define VR_MII_GEN5_12G_16G_RX_RATE_CTRL	0x1F8054U
#define   RX0_RATE_OFF				0
#define   RX0_RATE_MASK				0x3
#define     RX0_BAUD_DIV_2			0x1
#define     RX0_BAUD_DIV_8			0x3
#define VR_MII_GEN5_12G_16G_CDR_CTRL		0x1F8056U
#define   VCO_LOW_FREQ_0			BIT(8)
#define VR_MII_GEN5_12G_16G_MPLL_CMN_CTRL	0x1F8070U
#define   MPLLB_SEL_0				BIT(4)
#define VR_MII_GEN5_12G_16G_MPLLA_CTRL0		0x1F8071U
#define   MPLLA_CAL_DISABLE			BIT(15)
#define   MLLA_MULTIPLIER_OFF			0
#define   MLLA_MULTIPLIER_MASK			(0xFF << MLLA_MULTIPLIER_OFF)
#define VR_MII_GEN5_12G_MPLLA_CTRL1		0x1F8072U
#define   MPLLA_FRACN_CTRL_OFF			5
#define   MPLLA_FRACN_CTRL_MASK			(0x7FF << MPLLA_FRACN_CTRL_OFF)
#define VR_MII_GEN5_12G_16G_MPLLA_CTRL2		0x1F8073U
#define   MPLLA_TX_CLK_DIV_OFF			11
#define   MPLLA_TX_CLK_DIV_MASK			(0x7 << MPLLA_TX_CLK_DIV_OFF)
#define   MPLLA_DIV10_CLK_EN			BIT(9)
#define VR_MII_GEN5_12G_16G_MPLLB_CTRL0		0x1F8074U
#define   MPLLB_CAL_DISABLE			BIT(15)
#define   MLLB_MULTIPLIER_OFF			0
#define   MLLB_MULTIPLIER_MASK			0xFF
#define VR_MII_GEN5_12G_MPLLB_CTRL1		0x1F8075U
#define   MPLLB_FRACN_CTRL_OFF			5
#define   MPLLB_FRACN_CTRL_MASK			(0x7FF << MPLLB_FRACN_CTRL_OFF)
#define VR_MII_GEN5_12G_16G_MPLLB_CTRL2		0x1F8076U
#define   MPLLB_TX_CLK_DIV_OFF			11
#define   MPLLB_TX_CLK_DIV_MASK			(0x7 << MPLLA_TX_CLK_DIV_OFF)
#define   MPLLB_DIV10_CLK_EN			BIT(9)
#define VR_MII_RX_LSTS				0x1F8020U
#define   RX_VALID_0				BIT(12)
#define VR_MII_GEN5_12G_MPLLA_CTRL3		0x1F8077U
#define   MPLLA_BANDWIDTH_OFF			0x0
#define   MPLLA_BANDWIDTH_MASK			0xFFFF
#define VR_MII_GEN5_12G_MPLLB_CTRL3		0x1F8078U
#define   MPLLB_BANDWIDTH_OFF			0x0
#define   MPLLB_BANDWIDTH_MASK			0xFFFF
#define VR_MII_GEN5_12G_16G_REF_CLK_CTRL	0x1F8091U
#define   REF_CLK_EN				BIT(0)
#define   REF_USE_PAD				BIT(1)
#define   REF_CLK_DIV2				BIT(2)
#define   REF_RANGE_OFF				3
#define   REF_RANGE_MASK			(0x7 << REF_RANGE_OFF)
#define     RANGE_26_53_MHZ			0x1
#define     RANGE_52_78_MHZ			0x2
#define     RANGE_78_104_MHZ			0x3
#define   REF_RANGE(x)				(((x) & 0x7U) << 3)
#define   REF_MPLLA_DIV2			BIT(6)
#define   REF_MPLLB_DIV2			BIT(7)
#define   REF_RPT_CLK_EN			BIT(8)

#define VR_MII_GEN5_12G_16G_VCO_CAL_LD0		0x1F8092U
#define   VCO_LD_VAL_0_OFF			0
#define   VCO_LD_VAL_0_MASK			0x1FFF
#define VR_MII_GEN5_12G_VCO_CAL_REF0		0x1F8096U
#define   VCO_REF_LD_0_OFF			0
#define   VCO_REF_LD_0_MASK			0x3F

#define KHZ		(1000)
#define MHZ(X)		(1000 * KHZ * (X))

#define XPCS_WRITE_BITS(xpcs, REG, mask, value) \
	xpcs_write_bits(xpcs, #REG, REG, mask, value)

#define XPCS_WRITE(xpcs, REG, value) \
	xpcs_write(xpcs, #REG, REG, value)

#define XPCS_READ(xpcs, REG) \
	xpcs_read(xpcs, #REG, REG)

struct s32gen1_xpcs_params {
	u32 addr1;
	u32 addr2;
};

struct s32gen1_xpcs {
	struct s32gen1_xpcs_params params;
	void __iomem *base;
	struct device *dev;
	unsigned char id;
	struct regmap *regmap;
	bool ext_clk;
	bool mhz125;
};

typedef bool (*xpcs_poll_func_t)(struct s32gen1_xpcs *);

static int get_xpcs_id(struct s32gen1_xpcs *xpcs)
{
	return xpcs->id;
}

static struct device *get_xpcs_device(struct s32gen1_xpcs *xpcs)
{
	return xpcs->dev;
}

static void init_params(u32 reg, struct s32gen1_xpcs *xpcs,
			struct s32gen1_xpcs_params *params, u32 *data)
{
	u32 ofsleft = (reg >> 8) & 0xffffU;
	u32 ofsright = (reg & 0xffU);

	*data = ofsleft;

	params->addr1 = xpcs->params.addr1;
	params->addr2 = xpcs->params.addr2 + (ofsright * 4);

	params->addr1 -= xpcs->params.addr2;
	params->addr2 -= xpcs->params.addr2;
}

static int xpcs_regmap_reg_read(void *context, unsigned int reg,
				unsigned int *result)
{
	struct s32gen1_xpcs *xpcs = context;
	struct s32gen1_xpcs_params params;
	u32 data;

	init_params(reg, xpcs, &params, &data);

	writel(data, xpcs->base + params.addr1);
	*result = readl(xpcs->base + params.addr2);

	return 0;
}

static int xpcs_regmap_reg_write(void *context, unsigned int reg,
				 unsigned int val)
{
	struct s32gen1_xpcs *xpcs = context;
	struct s32gen1_xpcs_params params;
	u32 data;

	init_params(reg, xpcs, &params, &data);

	writel(data, xpcs->base + params.addr1);
	writel(val, xpcs->base + params.addr2);

	return 0;
}

static void xpcs_write_bits(struct s32gen1_xpcs *xpcs, const char *name,
			    unsigned int reg, unsigned int mask,
			    unsigned int value)
{
	struct device *dev = get_xpcs_device(xpcs);
	int ret = regmap_write_bits(xpcs->regmap, reg, mask, value);

	if (ret)
		dev_err(dev, "Failed to write bits of XPCS reg: %s\n", name);
}

static void xpcs_write(struct s32gen1_xpcs *xpcs, const char *name,
		       unsigned int reg, unsigned int value)
{
	struct device *dev = get_xpcs_device(xpcs);
	int ret = regmap_write(xpcs->regmap, reg, value);

	if (ret)
		dev_err(dev, "Failed to write XPCS reg: %s\n", name);
}

static unsigned int xpcs_read(struct s32gen1_xpcs *xpcs, const char *name,
			      unsigned int reg)
{
	struct device *dev = get_xpcs_device(xpcs);
	unsigned int val = 0;
	int ret;

	ret = regmap_read(xpcs->regmap, reg, &val);
	if (ret)
		dev_err(dev, "Failed to read XPCS reg: %s\n", name);

	return val;
}

const struct regmap_range xpcs_wr_ranges[] = {
	regmap_reg_range(0x1F0000, 0x1F0000),
	regmap_reg_range(0x1F0004, 0x1F0004),
	regmap_reg_range(0x1F8000, 0x1F8003),
	regmap_reg_range(0x1F8005, 0x1F8005),
	regmap_reg_range(0x1F800A, 0x1F800A),
	regmap_reg_range(0x1F8012, 0x1F8012),
	regmap_reg_range(0x1F8015, 0x1F8015),
	regmap_reg_range(0x1F8030, 0x1F8037),
	regmap_reg_range(0x1F803C, 0x1F803C),
	regmap_reg_range(0x1F8050, 0x1F8058),
	regmap_reg_range(0x1F805C, 0x1F805E),
	regmap_reg_range(0x1F8064, 0x1F8064),
	regmap_reg_range(0x1F806B, 0x1F806B),
	regmap_reg_range(0x1F8070, 0x1F8078),
	regmap_reg_range(0x1F8090, 0x1F8092),
	regmap_reg_range(0x1F8096, 0x1F8096),
	regmap_reg_range(0x1F8099, 0x1F80A2),
	regmap_reg_range(0x1F80E1, 0x1F80E1),
};

const struct regmap_range xpcs_rd_ranges[] = {
	regmap_reg_range(0x1F0001, 0x1F0003),
	regmap_reg_range(0x1F0005, 0x1F0006),
	regmap_reg_range(0x1F000F, 0x1F000F),
	regmap_reg_range(0x1F0708, 0x1F0710),
	regmap_reg_range(0x1F8010, 0x1F8011),
	regmap_reg_range(0x1F8018, 0x1F8018),
	regmap_reg_range(0x1F8020, 0x1F8020),
	regmap_reg_range(0x1F8040, 0x1F8040),
	regmap_reg_range(0x1F8060, 0x1F8060),
	regmap_reg_range(0x1F8098, 0x1F8098),
};

static bool xpcs_writeable_reg(struct device *dev, unsigned int reg)
{
	return regmap_reg_in_ranges(reg, xpcs_wr_ranges,
				    ARRAY_SIZE(xpcs_wr_ranges));
}

static bool xpcs_readable_reg(struct device *dev, unsigned int reg)
{
	if (!xpcs_writeable_reg(dev, reg))
		return regmap_reg_in_ranges(reg, xpcs_rd_ranges,
					    ARRAY_SIZE(xpcs_rd_ranges));

	return true;
}

static const struct regmap_config xpcs_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_read = xpcs_regmap_reg_read,
	.reg_write = xpcs_regmap_reg_write,
	.writeable_reg = xpcs_writeable_reg,
	.readable_reg = xpcs_readable_reg,
	.max_register = 0x1F80E1,
};

static int xpcs_init(struct s32gen1_xpcs **xpcs, struct device *dev,
		     unsigned char id, void __iomem *base, bool ext_clk,
		     unsigned long rate)
{
	struct s32gen1_xpcs *xpcsp;
	struct regmap_config conf;
	int ret;

	if (rate != MHZ(125) && rate != MHZ(100)) {
		dev_err(dev, "XPCS cannot operate @%lu HZ\n", rate);
		return -EINVAL;
	}

	*xpcs = devm_kmalloc(dev, sizeof(**xpcs), GFP_KERNEL);
	if (!*xpcs) {
		dev_err(dev, "Failed to allocate xpcs\n");
		return -ENOMEM;
	}

	xpcsp = *xpcs;

	xpcsp->base = base;
	xpcsp->ext_clk = ext_clk;
	xpcsp->id = id;
	xpcsp->dev = dev;

	if (rate == MHZ(125))
		xpcsp->mhz125 = true;
	else
		xpcsp->mhz125 = false;

	conf = xpcs_regmap_config;

	if (!get_xpcs_id(xpcsp)) {
		/**
		 * XPCS parameters based on Serdes Reference Manual,
		 * chapter 5.2
		 */
		xpcsp->params = (struct s32gen1_xpcs_params) {
			.addr1 = 0x823FCU,
			.addr2 = 0x82000U,
		};
		conf.name = "xpcs1";
	} else {
		xpcsp->params = (struct s32gen1_xpcs_params) {
			.addr1 = 0X82BFCU,
			.addr2 = 0x82800U,
		};
		conf.name = "xpcs0";
	}

	xpcsp->regmap = devm_regmap_init(dev, NULL, xpcsp, &conf);
	if (IS_ERR(xpcsp->regmap)) {
		ret = PTR_ERR(xpcsp->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	return 0;
}

static bool is_pgood_state(struct s32gen1_xpcs *xpcs)
{
	unsigned int val;

	/* Not in reset state */
	val = XPCS_READ(xpcs, VR_MII_DIG_CTRL1);
	if (val & VR_RST)
		return false;

	val = XPCS_READ(xpcs, VR_MII_DIG_STS);

	return PSEQ_STATE(val) == POWER_GOOD_STATE;
}

static bool is_not_in_reset(struct s32gen1_xpcs *xpcs)
{
	unsigned int val;

	val = XPCS_READ(xpcs, SR_MII_CTRL);

	return !(val & SR_RST);
}

static bool xpcs_poll_timeout(struct s32gen1_xpcs *xpcs, xpcs_poll_func_t func,
			      ktime_t timeout)
{
	ktime_t cur = ktime_get();

	return func(xpcs) || ktime_after(cur, timeout);
}

static int xpcs_wait(struct s32gen1_xpcs *xpcs, xpcs_poll_func_t func)
{
	ktime_t timeout = ktime_add_ms(ktime_get(), XPCS_TIMEOUT_MS);

	spin_until_cond(xpcs_poll_timeout(xpcs, func, timeout));
	if (!func(xpcs))
		return -ETIMEDOUT;

	return 0;
}

static int wait_power_good_state(struct s32gen1_xpcs *xpcs)
{
	int ret;

	ret = xpcs_wait(xpcs, is_pgood_state);
	if (ret == -ETIMEDOUT)
		dev_err(get_xpcs_device(xpcs), "XPCS%d power good timeout\n",
			get_xpcs_id(xpcs));

	return ret;
}

static int wait_reset(struct s32gen1_xpcs *xpcs)
{
	int ret;

	ret = xpcs_wait(xpcs, is_not_in_reset);
	if (ret == -ETIMEDOUT)
		dev_err(get_xpcs_device(xpcs), "XPCS%d is in reset\n",
			get_xpcs_id(xpcs));

	return ret;
}

static int xpcs_power_on(struct s32gen1_xpcs *xpcs)
{
	if (!xpcs->ext_clk)
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1,
				BYP_PWRUP, BYP_PWRUP);

	/* Power stabilization */
	return wait_power_good_state(xpcs);
}

static bool xpcs_has_valid_rx(struct s32gen1_xpcs *xpcs)
{
	unsigned int val;

	val = XPCS_READ(xpcs, VR_MII_RX_LSTS);
	return !!(val & RX_VALID_0);
}

static int xpcs_vreset(struct s32gen1_xpcs *xpcs)
{
	int ret = 0;

	if (!xpcs)
		return -EINVAL;

	XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, VR_RST, VR_RST);

	return ret;
}

static int xpcs_wait_vreset(struct s32gen1_xpcs *xpcs)
{
	if (!xpcs)
		return -EINVAL;

	return wait_reset(xpcs);
}

static int xpcs_reset_rx(struct s32gen1_xpcs *xpcs)
{
	struct device *dev = get_xpcs_device(xpcs);
	int ret;

	ret = wait_power_good_state(xpcs);
	if (ret) {
		dev_err(dev, "Failed to enter in PGOOD state after vendor reset\n");
		return ret;
	}

	/* Step 21 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_RX_GENCTRL1,
			RX_RST_0, RX_RST_0);

	/* Step 22 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_RX_GENCTRL1,
			RX_RST_0, 0);

	return 0;
}

static int xpcs_init_mplla(struct s32gen1_xpcs *xpcs)
{
	struct device *dev;
	unsigned int val;

	if (!xpcs)
		return -EINVAL;

	dev = get_xpcs_device(xpcs);

	/* Step 7 */
	val = 0;
	if (xpcs->ext_clk)
		val |= REF_USE_PAD;

	if (xpcs->mhz125) {
		val |= REF_MPLLA_DIV2;
		val |= REF_CLK_DIV2;
		val |= (RANGE_52_78_MHZ << REF_RANGE_OFF);
	} else {
		val |= (RANGE_78_104_MHZ << REF_RANGE_OFF);
	}

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_REF_CLK_CTRL,
			REF_MPLLA_DIV2 | REF_USE_PAD | REF_RANGE_MASK |
			REF_CLK_DIV2, val);

	/* Step 8 */
	if (xpcs->mhz125)
		val = (80 << MLLA_MULTIPLIER_OFF);
	else
		val = (25 << MLLA_MULTIPLIER_OFF);

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLLA_CTRL0,
			MPLLA_CAL_DISABLE | MLLA_MULTIPLIER_MASK,
			val);

	/* Step 9 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_MPLLA_CTRL1,
			MPLLA_FRACN_CTRL_MASK, 0);

	/* Step 10 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLLA_CTRL2,
			MPLLA_TX_CLK_DIV_MASK | MPLLA_DIV10_CLK_EN,
			(1 << MPLLA_TX_CLK_DIV_OFF) | MPLLA_DIV10_CLK_EN);

	/* Step 11 */
	if (xpcs->mhz125)
		val = 43 << MPLLA_BANDWIDTH_OFF;
	else
		val = 357 << MPLLA_BANDWIDTH_OFF;

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_MPLLA_CTRL3,
			MPLLA_BANDWIDTH_MASK, val);

	/* Step 18 */
	if (!xpcs->ext_clk)
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, BYP_PWRUP, 0);

	return 0;
}

static void xpcs_prepare_link_state(struct s32gen1_xpcs *xpcs,
				    const struct phylink_link_state *state)
{
	if (state->an_enabled)
		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, AN_ENABLE, AN_ENABLE);
	else
		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, AN_ENABLE, 0);

	if (state->duplex == DUPLEX_FULL)
		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, DUPLEX_MODE, DUPLEX_MODE);

	if (state->duplex == DUPLEX_HALF)
		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, DUPLEX_MODE, 0);
}

static int xpcs_set_1g_mode(struct s32gen1_xpcs *xpcs,
			    const struct phylink_link_state *state)
{
	unsigned int val;
	int ret;

	xpcs_prepare_link_state(xpcs, state);

	/* Step 2 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_EQ_CTRL0,
			TX_EQ_MAIN_MASK, 0xC << TX_EQ_MAIN_OFF);

	/* Step 3 */
	XPCS_WRITE_BITS(xpcs, VR_MII_CONSUMER_10G_TX_TERM_CTRL,
			TX0_TERM_MASK, 0x4 << TX0_TERM_OFF);

	/* Step 4 */
	XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, EN_2_5G_MODE, 0);

	/* Step 5 */
	XPCS_WRITE_BITS(xpcs, VR_MII_DBG_CTRL,
			SUPPRESS_LOS_DET | RX_DT_EN_CTL, 0);

	/* Step 6 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLL_CMN_CTRL,
			MPLLB_SEL_0, 0);

	ret = xpcs_init_mplla(xpcs);
	if (ret)
		return ret;

	/* Step 12 */
	if (xpcs->mhz125)
		val = 1360 << VCO_LD_VAL_0_OFF;
	else
		val = 1350 << VCO_LD_VAL_0_OFF;

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_VCO_CAL_LD0,
			VCO_LD_VAL_0_MASK, val);

	/* Step 13 */
	if (xpcs->mhz125)
		val = 17 << VCO_REF_LD_0_OFF;
	else
		val = 27 << VCO_REF_LD_0_OFF;

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_VCO_CAL_REF0,
			VCO_REF_LD_0_MASK, val);

	/* Step 14 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_RATE_CTRL,
			TX0_RATE_MASK, TX0_BAUD_DIV_4 << TX0_RATE_OFF);

	/* Step 15 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_RX_RATE_CTRL,
			RX0_RATE_MASK, RX0_BAUD_DIV_8 << RX0_RATE_OFF);

	/* Step 16 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_CDR_CTRL,
			VCO_LOW_FREQ_0, 0);

	/* Step 17 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLLB_CTRL0,
			MPLLB_CAL_DISABLE, MPLLB_CAL_DISABLE);

	/* Step 18 */
	if (!xpcs->ext_clk)
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, BYP_PWRUP, 0);

	return 0;
}

static int xpcs_set_2g5_mode(struct s32gen1_xpcs *xpcs,
			     const struct phylink_link_state *state)
{
	unsigned int val;
	int ret;

	xpcs_prepare_link_state(xpcs, state);

	/* Step 2 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_EQ_CTRL0,
			TX_EQ_MAIN_MASK, 0xC << TX_EQ_MAIN_OFF);

	/* Step 3 */
	XPCS_WRITE_BITS(xpcs, VR_MII_CONSUMER_10G_TX_TERM_CTRL,
			TX0_TERM_MASK, 0x4 << TX0_TERM_OFF);

	/* Step 4 */
	XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, EN_2_5G_MODE, EN_2_5G_MODE);

	/* Step 5 */
	XPCS_WRITE_BITS(xpcs, VR_MII_DBG_CTRL, SUPPRESS_LOS_DET | RX_DT_EN_CTL,
			SUPPRESS_LOS_DET | RX_DT_EN_CTL);

	/* Step 6 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLL_CMN_CTRL,
			MPLLB_SEL_0, MPLLB_SEL_0);

	/* Step 7 */
	val = 0;
	if (xpcs->ext_clk)
		val |= REF_USE_PAD;

	val |= REF_MPLLB_DIV2;
	val |= REF_CLK_DIV2;

	if (xpcs->mhz125)
		val |= (RANGE_52_78_MHZ << REF_RANGE_OFF);
	else
		val |= (RANGE_26_53_MHZ << REF_RANGE_OFF);

	XPCS_WRITE(xpcs, VR_MII_GEN5_12G_16G_REF_CLK_CTRL, val);

	/* Step 8 */
	if (xpcs->mhz125)
		val = 125 << MLLB_MULTIPLIER_OFF;
	else
		val = 156 << MLLB_MULTIPLIER_OFF;

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLLB_CTRL0,
			MPLLB_CAL_DISABLE | MLLA_MULTIPLIER_MASK,
			val);

	/* Step 9 */
	if (xpcs->mhz125)
		val = (0 << MPLLB_FRACN_CTRL_OFF);
	else
		val = (1044 << MPLLB_FRACN_CTRL_OFF);

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_MPLLB_CTRL1,
			MPLLB_FRACN_CTRL_MASK, val);

	/* Step 10 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLLB_CTRL2,
			MPLLB_TX_CLK_DIV_MASK | MPLLB_DIV10_CLK_EN,
			(5 << MPLLA_TX_CLK_DIV_OFF) | MPLLA_DIV10_CLK_EN);

	/* Step 11 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_MPLLB_CTRL3,
			MPLLB_BANDWIDTH_MASK, 68 << MPLLB_BANDWIDTH_OFF);

	/* Step 12 */
	if (xpcs->mhz125)
		val = 1350 << VCO_LD_VAL_0_OFF;
	else
		val = 1375 << VCO_LD_VAL_0_OFF;

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_VCO_CAL_LD0,
			VCO_LD_VAL_0_MASK, val);

	/* Step 13 */
	if (xpcs->mhz125)
		val = 27 << VCO_REF_LD_0_OFF;
	else
		val = 22 << VCO_REF_LD_0_OFF;

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_VCO_CAL_REF0,
			VCO_REF_LD_0_MASK, val);

	/* Step 14 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_RATE_CTRL,
			TX0_RATE_MASK, TX0_BAUD_DIV_1 << TX0_RATE_OFF);

	/* Step 15 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_RX_RATE_CTRL,
			RX0_RATE_MASK, RX0_BAUD_DIV_2 << RX0_RATE_OFF);

	/* Step 16 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_CDR_CTRL,
			VCO_LOW_FREQ_0, VCO_LOW_FREQ_0);

	/* Step 17 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLLA_CTRL0,
			MPLLB_CAL_DISABLE, MPLLB_CAL_DISABLE);

	/* Step 18 */
	if (!xpcs->ext_clk)
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, BYP_PWRUP, 0);

	ret = xpcs_vreset(xpcs);
	if (ret)
		return ret;

	ret = xpcs_wait_vreset(xpcs);
	if (ret)
		return ret;

	return 0;
}

static int xpcs_config(struct s32gen1_xpcs *xpcs,
		       const struct phylink_link_state *state)
{
	int ret = -EINVAL;

	if (state->interface == PHY_INTERFACE_MODE_1000BASEX)
		ret = xpcs_set_1g_mode(xpcs, state);

	if (state->interface == PHY_INTERFACE_MODE_2500BASEX)
		ret = xpcs_set_2g5_mode(xpcs, state);

	if (ret)
		return ret;

	return ret;
}

static int xpcs_get_state(struct s32gen1_xpcs *xpcs,
			  struct phylink_link_state *state)
{
	struct device *dev = get_xpcs_device(xpcs);
	unsigned int val;
	bool ss6, ss13;

	linkmode_zero(state->lp_advertising);

	val = XPCS_READ(xpcs, SR_MII_STS);
	if (val & AN_ABL)
		linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
				 state->advertising);

	state->link = !!(val & LINK_STS);
	state->an_complete = !!(val & AN_CMPL);

	val = XPCS_READ(xpcs, SR_MII_CTRL);

	state->an_enabled = !!(val & AN_ENABLE);
	ss6 = !!(val & SS6);
	ss13 = !!(val & SS13);

	switch (ss6 << 1 | ss13) {
	case 0:
		state->speed = SPEED_10;
		break;
	case 1:
		state->speed = SPEED_100;
		break;
	case 2:
		state->speed = SPEED_1000;
		break;
	default:
		dev_err(dev, "Failed to interpret the value of SR_MII_CTRL\n");
		break;
	}

	val = XPCS_READ(xpcs, VR_MII_DIG_CTRL1);
	if ((val & EN_2_5G_MODE) && state->speed == SPEED_1000)
		state->speed = SPEED_2500;

	val = XPCS_READ(xpcs, SR_MII_EXT_STS);

	if (val & CAP_1G_T_FD)
		linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				 state->advertising);
	if (val & CAP_1G_T_HD)
		linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				 state->advertising);

	return 0;
}

static void xpcs_release(struct s32gen1_xpcs *xpcs)
{
	regmap_exit(xpcs->regmap);
	devm_kfree(get_xpcs_device(xpcs), xpcs);
}

static const struct s32gen1_xpcs_ops s32gen1_xpcs_ops = {
	.get_state = xpcs_get_state,
	.init = xpcs_init,
	.power_on = xpcs_power_on,
	.config = xpcs_config,
	.vreset = xpcs_vreset,
	.wait_vreset = xpcs_wait_vreset,
	.init_mplla = xpcs_init_mplla,
	.reset_rx = xpcs_reset_rx,
	.release = xpcs_release,
	.has_valid_rx = xpcs_has_valid_rx,
};

const struct s32gen1_xpcs_ops *s32gen1_xpcs_get_ops(void)
{
	return &s32gen1_xpcs_ops;
}
EXPORT_SYMBOL_GPL(s32gen1_xpcs_get_ops);
MODULE_LICENSE("GPL v2");
