// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright 2021-2022,2024 NXP
 */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/pcs/nxp-s32cc-xpcs.h>
#include <linux/processor.h>
#include <linux/regmap.h>

#define XPCS_TIMEOUT_MS		300

#define SR_MII_CTRL				0x1F0000U
#define   AN_ENABLE				BIT(12)
#define   SR_RST				BIT(15)
#define   SS13					BIT(13)
#define   RESTART_AN				BIT(9)
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
#define   CL37_TMR_OVRRIDE		BIT(3)
#define   INIT					BIT(8)
#define   MAC_AUTO_SW				BIT(9)
#define   CS_EN					BIT(10)
#define   PWRSV					BIT(11)
#define   EN_VSMMD1				BIT(13)
#define   R2TLBE				BIT(14)
#define   VR_RST				BIT(15)
#define VR_MII_AN_CTRL				0x1F8001U
#define   MII_AN_INTR_EN		BIT(0)
#define   PCS_MODE_OFF			(1)
#define   PCS_MODE_MASK			(0x3 << PCS_MODE_OFF)
#define   PCS_MODE_SET(x)		(((x) << PCS_MODE_OFF) & PCS_MODE_MASK)
#define    PCS_MODE_SGMII		(2)
#define   MII_CTRL				BIT(8)
#define VR_MII_AN_INTR_STS			0x1F8002U
#define  CL37_ANCMPLT_INTR		BIT(0)
#define  CL37_ANSGM_STS_DUPLEX		BIT(1)
#define  CL37_ANSGM_STS_SPEED_OFF	(2)
#define  CL37_ANSGM_STS_SPEED_MASK	(0x3 << CL37_ANSGM_STS_SPEED_OFF)
#define  CL37_ANSGM_STS_LINK		BIT(4)
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
#define VR_MII_GEN5_12G_16G_TX_GENCTRL1 0x1F8031U
#define   VBOOST_EN_0				BIT(4)
#define   TX_CLK_RDY_0				BIT(12)
#define	VR_MII_GEN5_12G_16G_TX_GENCTRL2 0x1F8032U
#define	  TX_REQ_0					BIT(0)
#define VR_MII_GEN5_12G_16G_TX_RATE_CTRL	0x1F8034U
#define   TX0_RATE_OFF				0
#define   TX0_RATE_MASK				0x7
#define     TX0_BAUD_DIV_1			0
#define     TX0_BAUD_DIV_4			2
#define VR_MII_GEN5_12G_16G_TX_EQ_CTRL0		0x1F8036U
#define   TX_EQ_MAIN_OFF			8
#define   TX_EQ_MAIN_MASK			(0x3F << TX_EQ_MAIN_OFF)
#define VR_MII_GEN5_12G_16G_TX_EQ_CTRL1		0x1F8037U
#define   TX_EQ_OVR_RIDE			BIT(6)
#define VR_MII_CONSUMER_10G_TX_TERM_CTRL	0x1F803CU
#define   TX0_TERM_OFF				0
#define   TX0_TERM_MASK				0x7
#define VR_MII_GEN5_12G_16G_RX_GENCTRL1		0x1F8051U
#define   RX_RST_0				BIT(4)
#define VR_MII_GEN5_12G_16G_RX_GENCTRL2 0x1F8052U
#define   RX_REQ_0				BIT(0)
#define VR_MII_GEN5_12G_16G_RX_RATE_CTRL	0x1F8054U
#define   RX0_RATE_OFF				0
#define   RX0_RATE_MASK				0x3
#define     RX0_BAUD_DIV_2			0x1
#define     RX0_BAUD_DIV_8			0x3
#define VR_MII_GEN5_12G_16G_CDR_CTRL		0x1F8056U
#define   CDR_SSC_EN_0				BIT(4)
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
#define VR_MII_GEN5_12G_16G_MISC_CTRL0		0x1F8090U
#define   PLL_CTRL				BIT(15)
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

struct s32cc_xpcs_params {
	u32 addr1;
	u32 addr2;
};

enum s32cc_xpc_pll {
	XPCS_PLLA,	/* Slow PLL */
	XPCS_PLLB,	/* Fast PLL */
};

struct s32cc_xpcs {
	struct s32cc_xpcs_params params;
	enum s32cc_xpc_pll ref;
	void __iomem *base;
	struct device *dev;
	unsigned char id;
	struct regmap *regmap;
	bool ext_clk;
	bool mhz125;
	enum pcie_xpcs_mode pcie_shared;
	struct phylink_pcs pcs;
};

typedef bool (*xpcs_poll_func_t)(struct s32cc_xpcs *);

static const struct regmap_range xpcs_wr_ranges[] = {
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

static const struct regmap_range xpcs_rd_ranges[] = {
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

static int get_xpcs_id(struct s32cc_xpcs *xpcs)
{
	return xpcs->id;
}

static struct device *get_xpcs_device(struct s32cc_xpcs *xpcs)
{
	return xpcs->dev;
}

static void init_params(u32 reg, struct s32cc_xpcs *xpcs,
			struct s32cc_xpcs_params *params, u32 *data)
{
	u32 ofsleft = (reg >> 8) & 0xffffU;
	u32 ofsright = (reg & 0xffU);

	*data = ofsleft;

	params->addr1 = xpcs->params.addr1;
	params->addr2 = xpcs->params.addr2 + (ofsright * 4);

	params->addr1 -= xpcs->params.addr2;
	params->addr2 -= xpcs->params.addr2;
}

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

static int xpcs_regmap_reg_read(void *context, unsigned int reg,
				unsigned int *result)
{
	struct s32cc_xpcs *xpcs = context;
	struct device *dev = get_xpcs_device(xpcs);
	struct s32cc_xpcs_params params;
	u32 data;

	if (!xpcs_readable_reg(dev, reg)) {
		dev_err(dev, "The register 0x%x isn't readable\n", reg);
		return -EPERM;
	}

	init_params(reg, xpcs, &params, &data);

	writew(data, xpcs->base + params.addr1);
	*result = readw(xpcs->base + params.addr2);

	return 0;
}

static int xpcs_regmap_reg_write(void *context, unsigned int reg,
				 unsigned int val)
{
	struct s32cc_xpcs *xpcs = context;
	struct device *dev = get_xpcs_device(xpcs);
	struct s32cc_xpcs_params params;
	u32 data;

	if (!xpcs_writeable_reg(dev, reg)) {
		dev_err(dev, "The register 0x%x isn't writable\n", reg);
		return -EPERM;
	}

	init_params(reg, xpcs, &params, &data);

	writew(data, xpcs->base + params.addr1);
	writew(val, xpcs->base + params.addr2);

	return 0;
}

static void xpcs_write_bits(struct s32cc_xpcs *xpcs, const char *name,
			    unsigned int reg, unsigned int mask,
			    unsigned int value)
{
	struct device *dev = get_xpcs_device(xpcs);
	int ret = regmap_write_bits(xpcs->regmap, reg, mask, value);

	if (ret)
		dev_err(dev, "Failed to write bits of XPCS reg: %s\n", name);
}

static void xpcs_write(struct s32cc_xpcs *xpcs, const char *name,
		       unsigned int reg, unsigned int value)
{
	struct device *dev = get_xpcs_device(xpcs);
	int ret = regmap_write(xpcs->regmap, reg, value);

	if (ret)
		dev_err(dev, "Failed to write XPCS reg: %s\n", name);
}

static unsigned int xpcs_read(struct s32cc_xpcs *xpcs, const char *name,
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

static const struct regmap_config xpcs_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_read = xpcs_regmap_reg_read,
	.reg_write = xpcs_regmap_reg_write,
	.writeable_reg = xpcs_writeable_reg,
	.readable_reg = xpcs_readable_reg,
	.max_register = 0x1F80E1,
};

static bool is_pgood_state(struct s32cc_xpcs *xpcs)
{
	unsigned int val;

	/* Not in reset state */
	val = XPCS_READ(xpcs, VR_MII_DIG_CTRL1);
	if (val & VR_RST)
		return false;

	val = XPCS_READ(xpcs, VR_MII_DIG_STS);

	return PSEQ_STATE(val) == POWER_GOOD_STATE;
}

static bool is_not_in_reset(struct s32cc_xpcs *xpcs)
{
	unsigned int val;

	val = XPCS_READ(xpcs, SR_MII_CTRL);

	return !(val & SR_RST);
}

static bool xpcs_poll_timeout(struct s32cc_xpcs *xpcs, xpcs_poll_func_t func,
			      ktime_t timeout)
{
	ktime_t cur = ktime_get();

	return func(xpcs) || ktime_after(cur, timeout);
}

static int xpcs_wait(struct s32cc_xpcs *xpcs, xpcs_poll_func_t func)
{
	ktime_t timeout = ktime_add_ms(ktime_get(), XPCS_TIMEOUT_MS);

	spin_until_cond(xpcs_poll_timeout(xpcs, func, timeout));
	if (!func(xpcs))
		return -ETIMEDOUT;

	return 0;
}

static int xpcs_wait_bits(struct s32cc_xpcs *xpcs, unsigned int reg,
			  unsigned int mask, unsigned int bits)
{
	ktime_t cur;
	ktime_t timeout = ktime_add_ms(ktime_get(), XPCS_TIMEOUT_MS);

	spin_until_cond((cur = ktime_get(),
			 (XPCS_READ(xpcs, reg) & mask) == bits ||
			 ktime_after(cur, timeout)));
	if ((XPCS_READ(xpcs, reg) & mask) != bits)
		return -ETIMEDOUT;

	return 0;
}

static int wait_power_good_state(struct s32cc_xpcs *xpcs)
{
	int ret;

	ret = xpcs_wait(xpcs, is_pgood_state);
	if (ret == -ETIMEDOUT)
		dev_err(get_xpcs_device(xpcs), "XPCS%d power good timeout\n",
			get_xpcs_id(xpcs));

	return ret;
}

static int wait_reset(struct s32cc_xpcs *xpcs)
{
	int ret;

	ret = xpcs_wait(xpcs, is_not_in_reset);
	if (ret == -ETIMEDOUT)
		dev_err(get_xpcs_device(xpcs), "XPCS%d is in reset\n",
			get_xpcs_id(xpcs));

	return ret;
}

static int xpcs_power_on(struct s32cc_xpcs *xpcs)
{
	/*Nothing for now*/
	return 0;
}

static bool xpcs_has_valid_rx(struct s32cc_xpcs *xpcs)
{
	unsigned int val;

	val = XPCS_READ(xpcs, VR_MII_RX_LSTS);
	return !!(val & RX_VALID_0);
}

static int xpcs_vreset(struct s32cc_xpcs *xpcs)
{
	int ret = 0;

	if (!xpcs)
		return -EINVAL;

	XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, VR_RST, VR_RST);

	return ret;
}

static int xpcs_wait_vreset(struct s32cc_xpcs *xpcs)
{
	if (!xpcs)
		return -EINVAL;

	return wait_reset(xpcs);
}

static int xpcs_reset_rx(struct s32cc_xpcs *xpcs)
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

static int xpcs_ref_clk_sel(struct s32cc_xpcs *xpcs,
			    enum s32cc_xpc_pll ref_pll)
{
	switch (ref_pll) {
	case XPCS_PLLA:
		XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLL_CMN_CTRL,
				MPLLB_SEL_0, 0);
		xpcs->ref = XPCS_PLLA;
		break;
	case XPCS_PLLB:
		XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLL_CMN_CTRL,
				MPLLB_SEL_0, MPLLB_SEL_0);
		xpcs->ref = XPCS_PLLB;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void xpcs_electrical_configure(struct s32cc_xpcs *xpcs)
{
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_EQ_CTRL0,
			TX_EQ_MAIN_MASK, 0xC << TX_EQ_MAIN_OFF);

	XPCS_WRITE_BITS(xpcs, VR_MII_CONSUMER_10G_TX_TERM_CTRL,
			TX0_TERM_MASK, 0x4 << TX0_TERM_OFF);
}

static int xpcs_vco_cfg(struct s32cc_xpcs *xpcs, enum s32cc_xpc_pll vco_pll)
{
	unsigned int vco_ld = 0;
	unsigned int vco_ref = 0;
	unsigned int rx_baud = 0;
	unsigned int tx_baud = 0;

	switch (vco_pll) {
	case XPCS_PLLA:
		if (xpcs->mhz125) {
			vco_ld = 1360 << VCO_LD_VAL_0_OFF;
			vco_ref = 17 << VCO_REF_LD_0_OFF;
		} else {
			vco_ld = 1350 << VCO_LD_VAL_0_OFF;
			vco_ref = 27 << VCO_REF_LD_0_OFF;
		}

		rx_baud = RX0_BAUD_DIV_8 << RX0_RATE_OFF;
		tx_baud = TX0_BAUD_DIV_4 << TX0_RATE_OFF;
		break;
	case XPCS_PLLB:
		if (xpcs->mhz125) {
			vco_ld = 1350 << VCO_LD_VAL_0_OFF;
			vco_ref = 27 << VCO_REF_LD_0_OFF;
		} else {
			vco_ld = 1344 << VCO_LD_VAL_0_OFF;
			vco_ref = 43 << VCO_REF_LD_0_OFF;
		}

		rx_baud = RX0_BAUD_DIV_2 << RX0_RATE_OFF;
		tx_baud = TX0_BAUD_DIV_1 << TX0_RATE_OFF;
		break;
	default:
		return -EINVAL;
	}

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_VCO_CAL_LD0,
			VCO_LD_VAL_0_MASK, vco_ld);

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_VCO_CAL_REF0,
			VCO_REF_LD_0_MASK, vco_ref);

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_RATE_CTRL,
			TX0_RATE_MASK, tx_baud);
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_RX_RATE_CTRL,
			RX0_RATE_MASK, rx_baud);

	if (vco_pll == XPCS_PLLB) {
		XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_CDR_CTRL,
				VCO_LOW_FREQ_0, VCO_LOW_FREQ_0);
	} else {
		XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_CDR_CTRL,
				VCO_LOW_FREQ_0, 0);
	}

	return 0;
}

static int xpcs_init_mplla(struct s32cc_xpcs *xpcs)
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
		val |= (RANGE_26_53_MHZ << REF_RANGE_OFF);
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

	return 0;
}

static int xpcs_init_mpllb(struct s32cc_xpcs *xpcs)
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
		val |= REF_MPLLB_DIV2;
		val |= REF_CLK_DIV2;
		val |= (RANGE_52_78_MHZ << REF_RANGE_OFF);
	} else {
		val |= (RANGE_26_53_MHZ << REF_RANGE_OFF);
	}

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_REF_CLK_CTRL,
			REF_MPLLB_DIV2 | REF_USE_PAD | REF_RANGE_MASK |
			REF_CLK_DIV2, val);

	/* Step 8 */
	if (xpcs->mhz125)
		val = 125 << MLLB_MULTIPLIER_OFF;
	else
		val = 39 << MLLB_MULTIPLIER_OFF;

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MPLLB_CTRL0,
			MPLLB_CAL_DISABLE | MLLB_MULTIPLIER_MASK,
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
			(5 << MPLLA_TX_CLK_DIV_OFF) | MPLLB_DIV10_CLK_EN);

	/* Step 11 */
	if (xpcs->mhz125)
		val = (68 << MPLLB_BANDWIDTH_OFF);
	else
		val = (102 << MPLLB_BANDWIDTH_OFF);

	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_MPLLB_CTRL3,
			MPLLB_BANDWIDTH_MASK, val);

	return 0;
}

static void serdes_pma_high_freq_recovery(struct s32cc_xpcs *xpcs)
{
	/* PCS signal protection, PLL railout recovery */
	XPCS_WRITE_BITS(xpcs, VR_MII_DBG_CTRL, SUPPRESS_LOS_DET | RX_DT_EN_CTL,
			SUPPRESS_LOS_DET | RX_DT_EN_CTL);
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_MISC_CTRL0,
			PLL_CTRL, PLL_CTRL);
}

static void serdes_pma_configure_tx_eq_post(struct s32cc_xpcs *xpcs)
{
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_EQ_CTRL1,
			TX_EQ_OVR_RIDE, TX_EQ_OVR_RIDE);
}

static int xpcs_init_plls(struct s32cc_xpcs *xpcs)
{
	int ret;
	struct device *dev = get_xpcs_device(xpcs);

	if (!xpcs->ext_clk) {
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, BYP_PWRUP, BYP_PWRUP);
	} else if (xpcs->pcie_shared == NOT_SHARED) {
		wait_power_good_state(xpcs);
	} else if (xpcs->pcie_shared == PCIE_XPCS_2G5) {
		wait_power_good_state(xpcs);
		/* Configure equalization */
		serdes_pma_configure_tx_eq_post(xpcs);
		xpcs_electrical_configure(xpcs);

		/* Enable receiver recover */
		serdes_pma_high_freq_recovery(xpcs);
		return 0;
	}

	xpcs_electrical_configure(xpcs);

	xpcs_ref_clk_sel(xpcs, XPCS_PLLA);
	ret = xpcs_init_mplla(xpcs);
	if (ret) {
		dev_err(dev, "Failed to initialize PLLA\n");
		return ret;
	}
	ret = xpcs_init_mpllb(xpcs);
	if (ret) {
		dev_err(dev, "Failed to initialize PLLB\n");
		return ret;
	}
	xpcs_vco_cfg(xpcs, XPCS_PLLA);

	if (!xpcs->ext_clk)
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, BYP_PWRUP, 0);

	XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, AN_ENABLE, 0);
	XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, DUPLEX_MODE, DUPLEX_MODE);

	return 0;
}

static int serdes_bifurcation_pll_transit(struct s32cc_xpcs *xpcs,
					  enum s32cc_xpc_pll target_pll)
{
	int ret = 0;
	struct device *dev = get_xpcs_device(xpcs);

	/* Configure XPCS speed and VCO */
	if (target_pll == XPCS_PLLA) {
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, EN_2_5G_MODE, 0);
		xpcs_vco_cfg(xpcs, XPCS_PLLA);
	} else {
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1,
				EN_2_5G_MODE, EN_2_5G_MODE);
		xpcs_vco_cfg(xpcs, XPCS_PLLB);
	}

	/* Signal that clock are not available */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_GENCTRL1,
			TX_CLK_RDY_0, 0);

	/* Select PLL reference */
	if (target_pll == XPCS_PLLA)
		xpcs_ref_clk_sel(xpcs, XPCS_PLLA);
	else
		xpcs_ref_clk_sel(xpcs, XPCS_PLLB);

	/* Initiate transmitter TX reconfiguration request */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_GENCTRL2,
			TX_REQ_0, TX_REQ_0);

	/* Wait for transmitter to reconfigure */
	ret = xpcs_wait_bits(xpcs, VR_MII_GEN5_12G_16G_TX_GENCTRL2,
			     TX_REQ_0, 0);
	if (ret) {
		dev_err(dev, "Switch to TX_REQ_0 failed\n");
		return ret;
	}

	/* Initiate transmitter RX reconfiguration request */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_RX_GENCTRL2,
			RX_REQ_0, RX_REQ_0);

	/* Wait for receiver to reconfigure */
	ret = xpcs_wait_bits(xpcs, VR_MII_GEN5_12G_16G_RX_GENCTRL2,
			     RX_REQ_0, 0);
	if (ret) {
		dev_err(dev, "Switch to RX_REQ_0 failed\n");
		return ret;
	}

	/* Signal that clock are available */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_GENCTRL1,
			TX_CLK_RDY_0, TX_CLK_RDY_0);

	/* Flush internal logic */
	XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, INIT, INIT);

	/* Wait for init */
	ret = xpcs_wait_bits(xpcs, VR_MII_DIG_CTRL1, INIT, 0);
	if (ret) {
		dev_err(dev, "XPCS INIT failed\n");
		return ret;
	}

	return ret;
}

static int xpcs_configure(struct s32cc_xpcs *xpcs,
			  const struct phylink_link_state *state)
{
	XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, AN_ENABLE, 0);
	XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, DUPLEX_MODE, DUPLEX_MODE);

	return 0;
}

/* Note: This function should be compatible with phylink.
 * That means it should only modify link, duplex, speed
 * an_complete, pause.
 */
static int xpcs_get_state(struct s32cc_xpcs *xpcs,
			  struct phylink_link_state *state)
{
	struct device *dev = get_xpcs_device(xpcs);
	unsigned int mii_ctrl, val, ss;
	bool ss6, ss13, an_enabled, intr_en;


	mii_ctrl = XPCS_READ(xpcs, SR_MII_CTRL);
	an_enabled = !!(mii_ctrl & AN_ENABLE);
	intr_en = !!(XPCS_READ(xpcs, VR_MII_AN_CTRL) & MII_AN_INTR_EN);

	/* Check this important condition */
	if (an_enabled && !intr_en) {
		dev_err(dev, "Invalid SGMII AN config interrupt is disabled\n");
		return -EINVAL;
	}

	if (an_enabled) {
		/* MLO_AN_INBAND */
		state->speed = SPEED_UNKNOWN;
		state->link = 0;
		state->duplex =  DUPLEX_UNKNOWN;
		state->an_complete = 0;
		state->pause = MLO_PAUSE_NONE;
		val = XPCS_READ(xpcs, VR_MII_AN_INTR_STS);

		/* Interrupt is raised with each SGMII AN that is in cases
		 * Link down - Every SGMII link timer expire
		 * Link up - Once before link goes up
		 * So either linkup or raised interrupt mean AN was completed
		 */
		if ((val & CL37_ANCMPLT_INTR) || (val & CL37_ANSGM_STS_LINK)) {
			state->an_complete = 1;
			if (val & CL37_ANSGM_STS_LINK)
				state->link = 1;
			else
				return 0;
			if (val & CL37_ANSGM_STS_DUPLEX)
				state->duplex = DUPLEX_FULL;
			else
				state->duplex = DUPLEX_HALF;
			ss = ((val & CL37_ANSGM_STS_SPEED_MASK) >>
			      CL37_ANSGM_STS_SPEED_OFF);
		} else {
			return 0;
		}

		/* Clear the interrupt */
		if (val & CL37_ANCMPLT_INTR)
			XPCS_WRITE_BITS(xpcs, VR_MII_AN_INTR_STS,
					CL37_ANCMPLT_INTR, 0);
	} else {
		/* MLO_AN_FIXED, MLO_AN_PHY */
		val = XPCS_READ(xpcs, SR_MII_STS);
		state->link = !!(val & LINK_STS);
		state->an_complete = 0;
		state->pause = MLO_PAUSE_NONE;

		if (mii_ctrl & DUPLEX_MODE)
			state->duplex = DUPLEX_FULL;
		else
			state->duplex = DUPLEX_HALF;

		ss6 = !!(mii_ctrl & SS6);
		ss13 = !!(mii_ctrl & SS13);
		ss = ss6 << 1 | ss13;
	}

	switch (ss) {
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
	if ((mii_ctrl & EN_2_5G_MODE) && state->speed == SPEED_1000)
		state->speed = SPEED_2500;

	/* Cover SGMII AN inability to distigunish between 1G and 2.5G */
	if ((mii_ctrl & EN_2_5G_MODE) &&
	    state->speed != SPEED_2500 && an_enabled) {
		dev_err(dev, "Speed not supported in SGMII AN mode\n");
		return -EINVAL;
	}

	return 0;
}

static void s32cc_phylink_pcs_get_state(struct phylink_pcs *pcs,
					struct phylink_link_state *state)
{
	struct s32cc_xpcs *xpcs = container_of(pcs, struct s32cc_xpcs, pcs);

	xpcs_get_state(xpcs, state);
}

static int xpcs_pre_pcie_2g5(struct s32cc_xpcs *xpcs)
{
	struct device *dev = get_xpcs_device(xpcs);
	int ret;

	/* Enable volatge boost */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_GENCTRL1, VBOOST_EN_0,
			VBOOST_EN_0);

	/* TX rate baud  */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_TX_RATE_CTRL, 0x7, 0x0U);

	/* Rx rate baud/2 */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_RX_RATE_CTRL, 0x3U, 0x1U);

	/* Set low-frequency operating band */
	XPCS_WRITE_BITS(xpcs, VR_MII_GEN5_12G_16G_CDR_CTRL, CDR_SSC_EN_0,
			VCO_LOW_FREQ_0);

	ret = serdes_bifurcation_pll_transit(xpcs, XPCS_PLLB);
	if (ret)
		dev_err(dev, "Switch to PLLB failed\n");

	return ret;
}

static int xpcs_config(struct s32cc_xpcs *xpcs,
		       const struct phylink_link_state *state)
{
	struct device *dev = get_xpcs_device(xpcs);
	unsigned int val = 0, duplex = 0;
	int ret = 0;
	int speed = state->speed;
	bool sgmi_osc = false;

	/* Configure adaptive MII width */
	XPCS_WRITE_BITS(xpcs, VR_MII_AN_CTRL, MII_CTRL, 0);

	if (phylink_test(state->advertising, 2500baseT_Full))
		sgmi_osc = true;

	if (phylink_test(state->advertising, 10baseT_Half) ||
	    phylink_test(state->advertising, 10baseT_Full) ||
	    phylink_test(state->advertising, 100baseT_Half) ||
	    phylink_test(state->advertising, 100baseT_Full) ||
	    phylink_test(state->advertising, 100baseT1_Full) ||
	    phylink_test(state->advertising, 1000baseT_Half) ||
	    phylink_test(state->advertising, 1000baseT_Full) ||
	    phylink_test(state->advertising, 1000baseX_Full))
		if (state->an_enabled && sgmi_osc)
			dev_err(dev, "Invalid advertising configuration for SGMII AN\n");

	if (state->an_enabled && !state->an_complete) {
		if (sgmi_osc) {
			XPCS_WRITE(xpcs, VR_MII_LINK_TIMER_CTRL, 0x30e);
			speed = SPEED_2500;
		} else {
			XPCS_WRITE(xpcs, VR_MII_LINK_TIMER_CTRL, 0x7a1);
			speed = SPEED_1000;
		}
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, CL37_TMR_OVRRIDE, 0);
		XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1,
				CL37_TMR_OVRRIDE, CL37_TMR_OVRRIDE);
	} else if (!state->an_enabled) {
		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, AN_ENABLE, 0);
		XPCS_WRITE_BITS(xpcs, VR_MII_AN_CTRL, MII_AN_INTR_EN, 0);
	}

	if (!state->an_enabled || !state->an_complete) {
		switch (speed) {
		case SPEED_10:
			break;
		case SPEED_100:
			val = SS13;
			break;
		case SPEED_1000:
			val = SS6;
			break;
		case SPEED_2500:
			val = SS6;
			break;
		default:
			dev_err(dev, "Speed not supported\n");
			break;
		}

		if (state->duplex == DUPLEX_FULL)
			duplex = DUPLEX_MODE;
		else
			duplex = 0;

		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, DUPLEX_MODE, duplex);

		if (speed == SPEED_2500) {
			ret = serdes_bifurcation_pll_transit(xpcs, XPCS_PLLB);
			if (ret)
				dev_err(dev, "Switch to PLLB failed\n");
		} else {
			ret = serdes_bifurcation_pll_transit(xpcs, XPCS_PLLA);
			if (ret)
				dev_err(dev, "Switch to PLLA failed\n");
		}

		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, SS6 | SS13, val);
	}

	if (state->an_enabled && !state->an_complete) {
		/* Select SGMII type AN, enable interrupt */
		XPCS_WRITE_BITS(xpcs, VR_MII_AN_CTRL,
				PCS_MODE_MASK | MII_AN_INTR_EN,
				PCS_MODE_SET(PCS_MODE_SGMII) | MII_AN_INTR_EN);
		/* Enable SGMII AN */
		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, AN_ENABLE, AN_ENABLE);
		/* Enable SGMII AUTO SW */
		if (sgmi_osc)
			XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1, MAC_AUTO_SW, 0);
		else
			XPCS_WRITE_BITS(xpcs, VR_MII_DIG_CTRL1,
					MAC_AUTO_SW, MAC_AUTO_SW);

		XPCS_WRITE_BITS(xpcs, SR_MII_CTRL, RESTART_AN, RESTART_AN);
	}

	return 0;
}

static int s32cc_phylink_pcs_config(struct phylink_pcs *pcs,
				    unsigned int neg_mode,
				    phy_interface_t interface,
				    const unsigned long *advertising,
				    bool permit_pause_to_mac)
{
	/* Configuration done in link_up() */
	return 0;
}

static void s32cc_phylink_pcs_restart_an(struct phylink_pcs *pcs)
{
	/* Not yet */
}

static void s32cc_phylink_pcs_link_up(struct phylink_pcs *pcs,
				      unsigned int an_mode,
				      phy_interface_t interface, int speed,
				      int duplex)
{
	struct s32cc_xpcs *xpcs = container_of(pcs, struct s32cc_xpcs, pcs);
	struct device *dev = get_xpcs_device(xpcs);
	struct phylink_link_state state = { 0 };

	if (xpcs_get_state(xpcs, &state)) {
		dev_err(dev, "Get XPCS state failed\n");
		return;
	}

	state.speed = speed;
	state.duplex = duplex;
	/* We support serdes autoneg only on speed <= 1G */
	if (an_mode == MLO_AN_INBAND && speed <= SPEED_1000 &&
	    interface == PHY_INTERFACE_MODE_SGMII)
		state.an_enabled = true;

	dev_info(dev, "xpcs_%d: speed=%u duplex=%d an=%d\n", xpcs->id, speed, duplex, an_mode);
	xpcs_config(xpcs, &state);
}

static const struct phylink_pcs_ops s32cc_phylink_pcs_ops = {
	.pcs_get_state = s32cc_phylink_pcs_get_state,
	.pcs_config = s32cc_phylink_pcs_config,
	.pcs_an_restart = s32cc_phylink_pcs_restart_an,
	.pcs_link_up = s32cc_phylink_pcs_link_up,
};

static void xpcs_release(struct s32cc_xpcs *xpcs)
{
	if (xpcs) {
		regmap_exit(xpcs->regmap);
		devm_kfree(get_xpcs_device(xpcs), xpcs);
	}
}

static int xpcs_init(struct s32cc_xpcs **xpcs, struct device *dev,
		     unsigned char id, void __iomem *base, bool ext_clk,
		     unsigned long rate, enum pcie_xpcs_mode pcie_shared)
{
	struct s32cc_xpcs *xpcsp;
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
	xpcsp->pcie_shared = pcie_shared;

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
		xpcsp->params = (struct s32cc_xpcs_params) {
			.addr1 = 0x823FCU,
			.addr2 = 0x82000U,
		};
		conf.name = "xpcs0";
	} else {
		xpcsp->params = (struct s32cc_xpcs_params) {
			.addr1 = 0X82BFCU,
			.addr2 = 0x82800U,
		};
		conf.name = "xpcs1";
	}

	xpcsp->regmap = devm_regmap_init(dev, NULL, xpcsp, &conf);
	if (IS_ERR(xpcsp->regmap)) {
		ret = PTR_ERR(xpcsp->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	/* Phylink PCS */
	xpcsp->pcs.ops = &s32cc_phylink_pcs_ops;
	xpcsp->pcs.poll = true;

	return 0;
}

static const struct s32cc_xpcs_ops s32cc_xpcs_ops = {
	.init = xpcs_init,
	.power_on = xpcs_power_on,
	.config = xpcs_configure,
	.vreset = xpcs_vreset,
	.wait_vreset = xpcs_wait_vreset,
	.init_plls = xpcs_init_plls,
	.reset_rx = xpcs_reset_rx,
	.release = xpcs_release,
	.has_valid_rx = xpcs_has_valid_rx,
	.pre_pcie_2g5 = xpcs_pre_pcie_2g5,
	.xpcs_get_state = xpcs_get_state,
	.xpcs_config = xpcs_config,
};

const struct s32cc_xpcs_ops *s32cc_xpcs_get_ops(void)
{
	return &s32cc_xpcs_ops;
}
EXPORT_SYMBOL_GPL(s32cc_xpcs_get_ops);

struct phylink_pcs *s32cc_xpcs_get_pcs(struct s32cc_xpcs *xpcs)
{
	return &xpcs->pcs;
}
EXPORT_SYMBOL_GPL(s32cc_xpcs_get_pcs);

MODULE_LICENSE("GPL v2");
