// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2024-2026 NXP
 *
 */

#include <linux/cache.h>
#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <ufs/ufshci.h>

#include "ufshcd-pltfrm.h"
#include "ufshcd-dwc.h"
#include "ufshci-dwc.h"

/* MPHY register definitions */
#define MPHY_REG_LSB_MASK			0x00FF
#define MPHY_REG_MSB_MASK			0xFF00

/* M-PHY per-lane CR register addresses. */
#define RX_DAC_CTRL(n)		(0x10AF + ((n) * 0x100))
#define RX_DAC_CTRL_OVRD(n)	(0x10B0 + ((n) * 0x100))
#define RX_DAC_CTRL_SEL(n)	(0x10B1 + ((n) * 0x100))
#define RX_DAC_CTRL_EN(n)	(0x10B8 + ((n) * 0x100))
#define RX_OVRD_IN_1(n)		(0x3006 + ((n) * 0x100))
#define FAST_FLAGS(n)		(0x401C + ((n) * 0x100))
#define FW_CALIB_CCFG(n)	(0x404D + ((n) * 0x100))

/* Vendor specific attributes */
#define RXSQCONTROL				0x8009
#define RXC10DIRECTCONF7			0x8010
#define RXRHOLDCTRLOPT				0x8013
#define CBRATESEL				0x8114
#define CBCREGADDRLSB				0x8116
#define CBCREGADDRMSB				0x8117
#define CBCREGWRLSB				0x8118
#define CBCREGWRMSB				0x8119
#define CBCREGRDLSB				0x811A
#define CBCREGRDMSB				0x811B
#define CBCREGRDWRSEL				0x811C
#define CBCRCTRL				0x811F
#define CBREFCLKCTRL2				0x8132
#define MPLLBFRACNDENLSB			0x8141
#define V2IDIV5DIV2MPLLB			0x8160

/* TX Equalization Main/Post values for various de-emphasis and amplitudes. */
#define TX_EQ_MAIN_DE00_LA			0x8014
#define TX_EQ_MAIN_DE03_LA			0x801A
#define TX_EQ_MAIN_DE06_SA			0x801D
#define TX_EQ_POST_DE06_SA			0x801E
#define TX_EQ_MAIN_DE06_LA			0x8020
#define TX_EQ_POST_DE06_LA			0x8021

/* TX PHY direct iboost/rboost configuration. */
#define TXC10_IBOOST_RBOOST_CONF		0x8024

/* Vendor specific MPHY disable */
#define VS_MPHYDISABLE				0xD0C1

/* CPort definitions */
#define CPORT_0					0

/* SCM Register Offsets. */
#define SCM_ONE_US_TICK			0x2CU
#define SCM_STATUS			0x30

/* SCM_ONE_US_TICK Register. */
#define SCM_ONE_US_TICK_MASK		GENMASK(8, 0)

/* SCM_MPHY_RAM_CONFIG_STATUS Register Fields. */
#define SCM_STATUS_PHY_RESET_MASK		GENMASK(0, 0)
#define SCM_STATUS_SRAM_BYPASS_MASK		GENMASK(1, 1)
#define SCM_STATUS_SRAM_EXT_LD_DONE_MASK	GENMASK(2, 2)
#define SCM_STATUS_SRAM_INIT_DONE_MASK		GENMASK(16, 16)

#define SCM_STATUS_PHY_RESET(val)		\
	FIELD_PREP(SCM_STATUS_PHY_RESET_MASK, val)

#define SCM_STATUS_SRAM_BYPASS(val)		\
	FIELD_PREP(SCM_STATUS_SRAM_BYPASS_MASK, val)

/* Timeout values. */
#define CFG_MPHY_INIT_TIMEOUT_VALUE_US	3000000

/* Clock validation limits. */
#define MAX_VALID_ONE_US_TICK		SCM_ONE_US_TICK_MASK

/* Use reserved DMA pool for coherent DMA allocations on non-coherent devices. */
#define UFS_S32N79_QUIRK_ERR053438	BIT(0)

/* Chase-read DMA descriptors so queued Normal-NC writes drain before doorbell. */
#define UFS_S32N79_QUIRK_ERR053440	BIT(1)

/**
 * struct s32n_ufs - S32N79 UFS host controller data
 * @hba: UFS host controller instance
 * @reg_scm: SCM register base address
 * @quirks: driver-private quirks from match data
 * @mphy_boot_mode: MPHY boot mode configuration
 */
struct s32n_ufs {
	struct ufs_hba *hba;
	void __iomem *reg_scm;
	struct clk *core_clk;
	u32 quirks;
	u8 mphy_boot_mode;
};

/**
 * enum mphy_boot_mode - MPHY boot mode options
 * @MPHY_BOOT_NONE: Skip MPHY initialization
 * @MPHY_BOOT_ROM: Running MPHY from internal FW ROM
 * @MPHY_BOOT_COPY_SRAM: Running MPHY from a (modified) FW ROM copy in external SRAM
 */
enum mphy_boot_mode {
	MPHY_BOOT_NONE = 0,
	MPHY_BOOT_ROM = 1,
	MPHY_BOOT_COPY_SRAM = 2,
};

/**
 * struct ufs_s32n_phy_cfg - one MPHY CR (config-register) write
 * @addr: CR address (the per-lane offset is encoded in the address)
 * @val: value to program
 */
struct ufs_s32n_phy_cfg {
	u16 addr;
	u16 val;
};

static unsigned long ufs_s32n_calculate_us_tick(unsigned long clk_rate)
{
	return clk_rate / USEC_PER_SEC;
}

static int ufs_s32n_configure_clocks(struct ufs_hba *hba)
{
	struct s32n_ufs *ufs = ufshcd_get_variant(hba);
	struct device *dev = hba->dev;
	unsigned long clk_rate = 0;
	unsigned long one_us_tick;

	if (!ufs->core_clk) {
		dev_err(dev, "Invalid core_clk.\n");
		return -EINVAL;
	}

	clk_rate = clk_get_rate(ufs->core_clk);
	if (!clk_rate) {
		dev_err(dev, "Failed to get valid clock rate.\n");
		return -EINVAL;
	}

	one_us_tick = ufs_s32n_calculate_us_tick(clk_rate);
	if (one_us_tick == 0 || one_us_tick > MAX_VALID_ONE_US_TICK) {
		dev_err(dev, "Invalid one_us_tick value: %lu (clk_rate: %lu Hz).\n",
			one_us_tick, clk_rate);
		return -EINVAL;
	}

	dev_info(dev, "Core clock rate: %lu Hz, one_us_tick = %lu.\n",
		 clk_rate, one_us_tick);

	/* Configure the micro-second tick rate generator based on core
	 * clock rate.
	 */
	writel(one_us_tick, ufs->reg_scm + SCM_ONE_US_TICK);
	ufshcd_dwc_program_clk_div(hba, one_us_tick);

	return 0;
}

static int ufs_s32n_phy_read_cr(struct ufs_hba *hba, u16 addr, u16 *value)
{
	unsigned int uic_arg3 = 0;
	int ret = 0;

	/* CR LSB parallel address - 0x8116. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGADDRLSB, 0), addr & 0x00FF);
	/* CR MSB parallel address - 0x8117. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGADDRMSB, 0), addr >> 8);
	/* CR parallel control value: 1- write, 0 - read - 0x811c. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGRDWRSEL, 0), 0x0);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);
	/* CR LSB parallel read value - 0x811a. */
	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(CBCREGRDLSB, 0), &uic_arg3);
	*value = (u16)((u16)uic_arg3 & MPHY_REG_LSB_MASK);
	/* CR MSB parallel read value - 0x811b. */
	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(CBCREGRDMSB, 0), &uic_arg3);
	*value |= (u16)((u16)(uic_arg3 << 8) & MPHY_REG_MSB_MASK);

	return ret;
}

static int ufs_s32n_phy_write_cr(struct ufs_hba *hba, u16 addr, u16 value)
{
	int ret = 0;

	/* CR LSB parallel address - 0x8116. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGADDRLSB, 0), addr & 0x00FF);
	/* CR MSB parallel address - 0x8117. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGADDRMSB, 0), addr >> 8);
	/* CR LSB parallel write value - 0x8118. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGWRLSB, 0), value & 0x00FF);
	/* CR MSB parallel write value - 0x8119. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGWRMSB, 0), value >> 8);
	/* CR parallel control value: 1- write, 0 - read - 0x811c. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCREGRDWRSEL, 0), 0x1);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);

	return ret;
}

static int ufs_s32n_phy_sram_ext_loading(struct ufs_hba *hba)
{
	int ret = 0;
	u16 reg;

	ret |= ufs_s32n_phy_read_cr(hba, 0xC002, &reg);
	ret |= ufs_s32n_phy_write_cr(hba, 0xC002, reg);

	return ret;
}

static int ufs_s32n_phy_pre_link_calib(struct ufs_hba *hba)
{
	/* Calibrations that must run while the MPHY is disabled. */
	static const struct ufs_s32n_phy_cfg calib_mphy_disabled[] = {
		/* Bypass RX-AFE calibrations. */
		{ FAST_FLAGS(0), 0x4 },
		{ FAST_FLAGS(1), 0x4 },
	};
	/* Calibrations that must run while the MPHY is enabled. */
	static const struct ufs_s32n_phy_cfg calib_mphy_enabled[] = {
		/* Write ATT offset compensation code to internal DAC. */
		{ RX_DAC_CTRL_OVRD(0), 0x1 },
		{ RX_DAC_CTRL_OVRD(1), 0x1 },
		{ RX_DAC_CTRL(0), 0x8E },
		{ RX_DAC_CTRL(1), 0x91 },
		{ RX_DAC_CTRL_SEL(0), 0x1 },
		{ RX_DAC_CTRL_SEL(1), 0x1 },
		{ RX_DAC_CTRL_EN(0), 0x1 },
		{ RX_DAC_CTRL_EN(1), 0x1 },
		{ RX_DAC_CTRL_OVRD(0), 0x0 },
		{ RX_DAC_CTRL_OVRD(1), 0x0 },
		/* Write CTLE offset compensation code to internal DAC. */
		{ RX_DAC_CTRL_OVRD(0), 0x1 },
		{ RX_DAC_CTRL_OVRD(1), 0x1 },
		{ RX_DAC_CTRL(0), 0x71 },
		{ RX_DAC_CTRL(1), 0x7F },
		{ RX_DAC_CTRL_SEL(0), 0x2 },
		{ RX_DAC_CTRL_SEL(1), 0x2 },
		{ RX_DAC_CTRL_EN(0), 0x1 },
		{ RX_DAC_CTRL_EN(1), 0x1 },
		{ RX_DAC_CTRL_OVRD(0), 0x0 },
		{ RX_DAC_CTRL_OVRD(1), 0x0 },
		/* Generic FW calibration configurations. */
		{ FW_CALIB_CCFG(0), 0x100 },
		{ FW_CALIB_CCFG(1), 0x100 },
	};
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(calib_mphy_disabled); i++)
		ret |= ufs_s32n_phy_write_cr(hba, calib_mphy_disabled[i].addr,
					     calib_mphy_disabled[i].val);

	/* Enable MPHY before the DAC/FW calibration writes. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYDISABLE, 0), 0x0);

	for (i = 0; i < ARRAY_SIZE(calib_mphy_enabled); i++)
		ret |= ufs_s32n_phy_write_cr(hba, calib_mphy_enabled[i].addr,
					     calib_mphy_enabled[i].val);

	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);

	return ret;
}

static int ufs_s32n_phy_reset_config(struct ufs_hba *hba)
{
	/*
	 * MPHY reset configuration: RX/TX vendor attributes programmed while
	 * the PHY is held in reset.
	 */
	static const struct ufshcd_dme_attr_val mphy_rst_cfg[] = {
		/* Clock control. */
		{ UIC_ARG_MIB_SEL(CBREFCLKCTRL2, 0), 0x80, DME_LOCAL },
		/* RX squelch control. */
		{ UIC_ARG_MIB_SEL(RXSQCONTROL, SELIND_LN0_RX), 0x1, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(RXSQCONTROL, SELIND_LN1_RX), 0x1, DME_LOCAL },
		/* RX Rhold control option. */
		{ UIC_ARG_MIB_SEL(RXRHOLDCTRLOPT, SELIND_LN0_RX), 0x2, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(RXRHOLDCTRLOPT, SELIND_LN1_RX), 0x2, DME_LOCAL },
		/* CR parallel selection. */
		{ UIC_ARG_MIB_SEL(CBCRCTRL, 0), 0x1, DME_LOCAL },
		/* TX Equalization Main de-emphasis 0db, large amplitude. */
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE00_LA, SELIND_LN0_TX), 0x27, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE00_LA, SELIND_LN1_TX), 0x27, DME_LOCAL },
		/* TX Equalization Main de-emphasis 3.5db, large amplitude. */
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE03_LA, SELIND_LN0_TX), 0x21, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE03_LA, SELIND_LN1_TX), 0x21, DME_LOCAL },
		/* TX Equalization Main de-emphasis 6db, small amplitude. */
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE06_SA, SELIND_LN0_TX), 0x1D, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE06_SA, SELIND_LN1_TX), 0x1D, DME_LOCAL },
		/* TX Equalization Main de-emphasis 6db, large amplitude. */
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE06_LA, SELIND_LN0_TX), 0x1D, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(TX_EQ_MAIN_DE06_LA, SELIND_LN1_TX), 0x1D, DME_LOCAL },
		/* TX Equalization Post de-emphasis 6db, small amplitude. */
		{ UIC_ARG_MIB_SEL(TX_EQ_POST_DE06_SA, SELIND_LN0_TX), 0x0A, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(TX_EQ_POST_DE06_SA, SELIND_LN1_TX), 0x0A, DME_LOCAL },
		/* TX Equalization Post de-emphasis 6db, large amplitude. */
		{ UIC_ARG_MIB_SEL(TX_EQ_POST_DE06_LA, SELIND_LN0_TX), 0x0A, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(TX_EQ_POST_DE06_LA, SELIND_LN1_TX), 0x0A, DME_LOCAL },
		/* TX PHY direct iboost/rboost configuration. */
		{ UIC_ARG_MIB_SEL(TXC10_IBOOST_RBOOST_CONF, SELIND_LN0_TX), 0x0, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(TXC10_IBOOST_RBOOST_CONF, SELIND_LN1_TX), 0x0, DME_LOCAL },
		/* RX PHY direct configuration 7. */
		{ UIC_ARG_MIB_SEL(RXC10DIRECTCONF7, SELIND_LN0_RX), 0x3, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(RXC10DIRECTCONF7, SELIND_LN1_RX), 0x3, DME_LOCAL },
		/* CB PHY direct MPLLB configuration. */
		{ UIC_ARG_MIB_SEL(MPLLBFRACNDENLSB, 0), 0x1, DME_LOCAL },
		{ UIC_ARG_MIB_SEL(V2IDIV5DIV2MPLLB, 0), 0x1, DME_LOCAL },
	};
	int ret = 0;

	ret |= ufshcd_dwc_dme_set_attrs(hba, mphy_rst_cfg,
					ARRAY_SIZE(mphy_rst_cfg));

	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);

	return ret;
}

static int ufs_s32n_link_startup_pre_change(struct ufs_hba *hba)
{
	struct s32n_ufs *ufs = ufshcd_get_variant(hba);
	struct device *dev = hba->dev;
	u32 reg;
	int ret = 0;

	/* There are three supported MPHY boot options implemented:
	 * - Running MPHY from internal FW ROM:
	 *  MPHY_RAM_CONFIG_STATUS.SRAM_BYPASS = 1
	 *  MPHY_RAM_CONFIG_STATUS.SRAM_EXT_LD_DONE = 0
	 * - Running MPHY from a (modified) FW ROM copy in external SRAM:
	 *  MPHY_RAM_CONFIG_STATUS.SRAM_BYPASS = 0
	 *  MPHY_RAM_CONFIG_STATUS.SRAM_EXT_LD_DONE = 0 (-> 1 after external loading is done).
	 * - MPHY initialization skipped used when running in a simulated
	 * environment or when another component is doing it.
	 */
	if (ufs->mphy_boot_mode == MPHY_BOOT_NONE) {
		dev_info(dev, "Skipping UFS MPHY init.\n");
		return 0;
	}

	ret = ufs_s32n_configure_clocks(hba);
	if (ret)
		return ret;

	/* Reset SCM.MPHY_RAM_CONFIG_STATUS to default value; keep MPHY in reset. */
	writel(SCM_STATUS_PHY_RESET(1), ufs->reg_scm + SCM_STATUS);
	if (ufs->mphy_boot_mode == MPHY_BOOT_ROM) {
		/* ROM Mode. */
		writel(SCM_STATUS_PHY_RESET(1) | SCM_STATUS_SRAM_BYPASS(1),
		       ufs->reg_scm + SCM_STATUS);
	}

	ret |= ufs_s32n_phy_reset_config(hba);
	if (ret)
		return ret;

	/* Release PHY_RESET. */
	writel(readl(ufs->reg_scm + SCM_STATUS) & ~SCM_STATUS_PHY_RESET(1),
	       ufs->reg_scm + SCM_STATUS);

	/* Wait until SRAM_INIT_DONE = 1.
	 * In case of MPHY FW running from SRAM max can be as high as 15ms@100MHz
	 * reference clock (FW is copied from MPHY ROM to SRAM).
	 */
	ret = readl_poll_timeout(ufs->reg_scm + SCM_STATUS, reg,
			reg & SCM_STATUS_SRAM_INIT_DONE_MASK,
			1000, CFG_MPHY_INIT_TIMEOUT_VALUE_US);
	if (ret) {
		dev_err(dev, "UFS MPHY init not done!\n");
		return ret;
	}

	if (ufs->mphy_boot_mode == MPHY_BOOT_COPY_SRAM) {
		/* SRAM External Loading phase - MPHY control registers can be
		 * overwritten using CREG interface.
		 */
		ret |= ufs_s32n_phy_sram_ext_loading(hba);
		if (ret) {
			dev_err(dev, "SRAM external loading failed!\n");
			return ret;
		}

		/* Set MPHY_RAM_CONFIG_STATUS.SRAM_EXT_LD_DONE. */
		writel(readl(ufs->reg_scm + SCM_STATUS) |
		       SCM_STATUS_SRAM_EXT_LD_DONE_MASK,
		       ufs->reg_scm + SCM_STATUS);
	}

	/* Pre-link MPHY calibration. */
	ret = ufs_s32n_phy_pre_link_calib(hba);
	if (ret) {
		dev_err(dev, "UFS MPHY pre-link calibration failed!\n");
		return ret;
	}

	/* Set Local DeviceID. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(N_DEVICEID, 0), 0x0);

	/* End of Gear1 settings */

	if (ret)
		return ret;

	return ufshcd_check_hibern8(hba, 2);
}

static int ufs_s32n_link_startup_post_change(struct ufs_hba *hba)
{
	/* Post startup link calibration. */
	static const struct ufs_s32n_phy_cfg post_link_cfg[] = {
		{ RX_OVRD_IN_1(0), 0xc }, { RX_OVRD_IN_1(1), 0xc },
		{ RX_OVRD_IN_1(0), 0x8 }, { RX_OVRD_IN_1(1), 0x8 },
		{ RX_OVRD_IN_1(0), 0x0 }, { RX_OVRD_IN_1(1), 0x0 },
	};
	struct s32n_ufs *ufs = ufshcd_get_variant(hba);
	unsigned int data = 0;
	unsigned int i;
	int ret = 0;

	/* If MPHY initialization is not handled by TF-A,
	 * invoke the generic DW power mode update procedure.
	 */
	if (ufs->mphy_boot_mode == MPHY_BOOT_NONE)
		return 0;

	/* Set Connection State to IDLE (it allows CPort Attributes to be set). */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(T_CONNECTIONSTATE, CPORT_0), CPORT_IDLE);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(T_CPORTFLAGS, CPORT_0), CPORT_DEF_FLAGS);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(T_CONNECTIONSTATE, CPORT_0), CPORT_CONNECTED);
	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(T_CONNECTIONSTATE, CPORT_0), &data);
	if ((ret != 0) || (data != CPORT_CONNECTED))
		return ret;

	for (i = 0; i < ARRAY_SIZE(post_link_cfg); i++)
		ret |= ufs_s32n_phy_write_cr(hba, post_link_cfg[i].addr,
					     post_link_cfg[i].val);

	/* Performing MPHY configuration for rate change:
	 * CB rate selection: 0 - rate A, 1 - rate B;
	 */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBRATESEL, 0), 0x1);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);

	/* End of Gear4 calibration */
	return ret;
}

static int ufs_s32n_link_startup_notify(struct ufs_hba *hba,
			     enum ufs_notify_change_status status)
{
	int err = 0;

	if (status == PRE_CHANGE) {
		err = ufs_s32n_link_startup_pre_change(hba);
		if (err) {
			dev_err(hba->dev, "MPHY setup failed (%d).\n", err);
			goto out;
		}
	} else { /* POST_CHANGE */
		err = ufshcd_dwc_link_is_up(hba);
		if (err) {
			dev_err(hba->dev, "Link is not up.\n");
			goto out;
		}

		err = ufs_s32n_link_startup_post_change(hba);
		if (err)
			dev_err(hba->dev, "Connection setup failed (%d).\n",
				err);
	}

out:
	return err;
}

/*
 * S32N79 interconnect can keep Normal-NC writes (UTRD/UCD/UTMRD updates)
 * in a per-destination, per-cache-line write buffer past the point at
 * which the doorbell Device write drains, so the controller may fetch
 * stale descriptor words. Flush by chasing a read on every cache line
 * of each region just written.
 */
static __always_inline void ufs_s32n_chase_read(const void *addr)
{
	u32 v = *(const volatile u32 *)addr;

	OPTIMIZER_HIDE_VAR(v);
}

static void ufs_s32n_chase_region(const void *base, size_t size)
{
	const u8 *p = base;
	const u8 *end = p + size;

	for (; p < end; p += L1_CACHE_BYTES)
		ufs_s32n_chase_read(p);
}

/*
 * ERR053440 is worked around only on hosts flagged in match data, and only
 * when non-coherent.
 */
static bool ufs_s32n_err053440(struct ufs_hba *hba)
{
	struct s32n_ufs *ufs = ufshcd_get_variant(hba);

	return (ufs->quirks & UFS_S32N79_QUIRK_ERR053440) &&
	       !dev_is_dma_coherent(hba->dev);
}

static void ufs_s32n_setup_xfer_req(struct ufs_hba *hba, int tag,
				    bool is_scsi_cmd)
{
	struct ufshcd_lrb *lrbp = &hba->lrb[tag];
	struct utp_transfer_req_desc *utrd = lrbp->utr_descriptor_ptr;
	size_t prdt_bytes = 0;
	size_t cmd_upiu_len;
	u16 prdt_len;
	u16 dsl;

	if (!ufs_s32n_err053440(hba))
		return;

	dma_wmb();

	/*
	 * The controller fetches the 32-byte transaction header plus the Data
	 * Segment that follows it in the Command UPIU.
	 */
	dsl = be16_to_cpu(lrbp->ucd_req_ptr->header.data_segment_length);
	cmd_upiu_len = sizeof(struct utp_upiu_req) + dsl;
	ufs_s32n_chase_region(lrbp->ucd_req_ptr, cmd_upiu_len);

	ufs_s32n_chase_region(lrbp->ucd_rsp_ptr, sizeof(struct utp_upiu_rsp));

	/*
	 * Read prd_table_length from UTRD with a single volatile load: it
	 * doubles as the chase of UTRD's one cache line (UFSHCI 3.0 UTRD is
	 * 32 B) and yields the count used to scope the PRDT chase below.
	 */
	prdt_len = le16_to_cpu(*(const volatile __le16 *)&utrd->prd_table_length);

	if (prdt_len) {
		prdt_bytes = (hba->quirks & UFSHCD_QUIRK_PRDT_BYTE_GRAN) ?
			     prdt_len : prdt_len * ufshcd_sg_entry_size(hba);
		ufs_s32n_chase_region(lrbp->ucd_prdt_ptr, prdt_bytes);
	}
}

static void ufs_s32n_setup_task_mgmt(struct ufs_hba *hba, int tag, u8 tm_func)
{
	struct utp_task_req_desc *utmrd = hba->utmrdl_base_addr + tag;

	if (!ufs_s32n_err053440(hba))
		return;

	dma_wmb();
	ufs_s32n_chase_region(utmrd, sizeof(*utmrd));
}

struct s32n79_match_data {
	const struct ufs_hba_variant_ops *vops;
	unsigned short sg_tablesize;
	u32 quirks;
};

static int ufs_s32n_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	const struct s32n79_match_data *data =
				of_device_get_match_data(hba->dev);
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	const char *mphy_boot_mode;
	struct s32n_ufs *ufs;
	int ret;

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;

	ufs->hba = hba;
	ufs->mphy_boot_mode = MPHY_BOOT_NONE;
	ufs->quirks = data ? data->quirks : 0;

	hba->quirks |= UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8 | UFSHCD_QUIRK_PERFORM_LINK_STARTUP_ONCE;
	hba->spm_lvl = UFS_PM_LVL_5;

	if (data && hba->host && data->sg_tablesize)
		hba->host->sg_tablesize = data->sg_tablesize;

	ret = of_property_read_string(np, "nxp,mphy-boot-mode", &mphy_boot_mode);
	if (ret || !mphy_boot_mode) {
		dev_warn(dev,
			"nxp,mphy-boot-mode property not found. UFS MPHY init will be skipped.\n");
		goto init_out;
	}

	if (!strcmp(mphy_boot_mode, "rom")) {
		ufs->mphy_boot_mode = MPHY_BOOT_ROM;
	} else if (!strcmp(mphy_boot_mode, "copy-sram")) {
		ufs->mphy_boot_mode = MPHY_BOOT_COPY_SRAM;
	} else {
		dev_warn(dev,
			 "Unrecognized nxp,mphy-boot-mode property. UFS MPHY init will be skipped.\n");
		goto init_out;
	}

	ufs->reg_scm = devm_platform_ioremap_resource_byname(pdev, "scm");
	if (IS_ERR(ufs->reg_scm)) {
		dev_err(dev, "ioremap failed for SCM registers.\n");
		return PTR_ERR(ufs->reg_scm);
	}

	ufs->core_clk = devm_clk_get(dev, "core_clk");
	if (IS_ERR(ufs->core_clk)) {
		dev_err(dev, "Failed to get core clock.\n");
		return PTR_ERR(ufs->core_clk);
	}

init_out:
	ufshcd_set_variant(hba, ufs);

	return 0;
}

static struct ufs_hba_variant_ops ufs_hba_s32n79_vops = {
	.name = "s32n79",
	.init = ufs_s32n_init,
	.link_startup_notify = ufs_s32n_link_startup_notify,
	/* ERR053440 */
	.setup_xfer_req = ufs_s32n_setup_xfer_req,
	.setup_task_mgmt = ufs_s32n_setup_task_mgmt,
};

static const struct s32n79_match_data ufs_s32n79_data = {
	.vops = &ufs_hba_s32n79_vops,
	.sg_tablesize = SG_ALL,
	.quirks = UFS_S32N79_QUIRK_ERR053438 | UFS_S32N79_QUIRK_ERR053440,
};

static const struct s32n79_match_data ufs_s32n79_virt_data = {
	.vops = &ufs_hba_s32n79_vops,
	.sg_tablesize = 120,
	.quirks = UFS_S32N79_QUIRK_ERR053438 | UFS_S32N79_QUIRK_ERR053440,
};

static const struct of_device_id ufs_s32n79_match[] = {
	{
		.compatible = "nxp,s32n79-ufshc",
		.data = &ufs_s32n79_data,
	},
	{
		.compatible = "nxp,s32n79-ufshc-virt",
		.data = &ufs_s32n79_virt_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ufs_s32n79_match);

static void ufs_s32n79_release_dma_reserved_mem(void *data)
{
	of_reserved_mem_device_release(data);
}

static int ufs_s32n79_init_dma_reserved_mem(struct device *dev,
					    const struct s32n79_match_data *data)
{
	int err;

	if (!(data->quirks & UFS_S32N79_QUIRK_ERR053438))
		return 0;

	/* DMA coherent devices are not affected by ERR053438. */
	if (dev_is_dma_coherent(dev))
		return 0;

	/*
	 * S32N79 ERR053438 workaround: attach the reserved DMA pool
	 * before the UFS core allocates coherent descriptor memory.
	 */
	err = of_reserved_mem_device_init(dev);
	if (err)
		return dev_err_probe(dev, err,
				     "failed to attach reserved memory\n");

	return devm_add_action_or_reset(dev,
					ufs_s32n79_release_dma_reserved_mem,
					dev);
}

static int ufs_s32n79_probe(struct platform_device *pdev)
{
	const struct s32n79_match_data *data;
	struct device *dev = &pdev->dev;
	int err;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	err = ufs_s32n79_init_dma_reserved_mem(dev, data);
	if (err)
		return err;

	return ufshcd_pltfrm_init(pdev, data->vops);
}

static void ufs_s32n79_remove(struct platform_device *pdev)
{
	struct ufs_hba *hba =  platform_get_drvdata(pdev);

	pm_runtime_get_sync(&(pdev)->dev);
	ufshcd_remove(hba);
}

static const struct dev_pm_ops ufs_s32n79_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
};

static struct platform_driver ufs_s32n79_driver = {
	.probe = ufs_s32n79_probe,
	.remove_new = ufs_s32n79_remove,
	.driver	= {
		.name	= "ufs-s32n79",
		.pm	= &ufs_s32n79_pm_ops,
		.of_match_table	= ufs_s32n79_match,
	},
};

module_platform_driver(ufs_s32n79_driver);

MODULE_AUTHOR("Larisa Grigore <larisa.grigore@nxp.com>");
MODULE_DESCRIPTION("NXP S32N79 UFS Host Controller platform driver");
MODULE_LICENSE("GPL");
