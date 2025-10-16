// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2024-2025 NXP
 *
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "ufs-nxp-s32n79.h"
#include "ufshcd-pltfrm.h"
#include "ufshcd-dwc.h"
#include "ufshci-dwc.h"

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

/**
 * struct s32n_ufs - S32N79 UFS host controller data
 * @hba: UFS host controller instance
 * @reg_scm: SCM register base address
 * @mphy_boot_mode: MPHY boot mode configuration
 */
struct s32n_ufs {
	struct ufs_hba *hba;
	void __iomem *reg_scm;
	struct clk *core_clk;
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

	ret |= ufs_s32n_phy_write_cr(hba, MPHY_REG_UNKNOWN_1, 0x080f);
	ret |= ufs_s32n_phy_write_cr(hba, SUP_DIG_MPLLB_OVRD_IN_5, 0x4004);
	ret |= ufs_s32n_phy_write_cr(hba, SUP_DIG_MPLLB_OVRD_IN_6, 0x4004);

	return ret;
}

static int ufs_s32n_phy_configure(struct ufs_hba *hba)
{
	int ret = 0;

	/* RX squelch control. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXSQCONTROL, SELIND_LN0_RX), 0x1);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXSQCONTROL, SELIND_LN1_RX), 0x1);
	/* RX Rhold control option. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXRHOLDCTRLOPT, SELIND_LN0_RX), 0x2);
	/* CR parallel selection. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBCRCTRL, 0), 0x1);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);
	/* RX squelch control. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXSQCONTROL, 0), 0x2);
	/* RX override control 1. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXOVRCTRL1, 0), 0x58);
	/* RX squelch control. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXSQCONTROL, 1), 0x2);
	/* RX override control 1. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXOVRCTRL1, 1), 0x58);
	/* RX line reset configuration MSByte. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXLINERSTCFGMSB, SELIND_LN0_RX), 0x1);
	/* RX line reset configuration LSByte. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXLINERSTCFGLSB, SELIND_LN0_RX), 0xf4);
	/* RX line reset configuration MSByte. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXLINERSTCFGMSB, SELIND_LN1_RX), 0x1);
	/* RX line reset configuration LSByte. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RXLINERSTCFGLSB, SELIND_LN1_RX), 0xf4);
	/* CB PHY override capability. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBENBLCPBATTRWR, 0), 0x1);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);
	/* Specifies support for the refresh adaptation used for RX. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_HS_ADAPT_REFRESH_CAP, SELIND_LN0_RX), 0x8f);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_HS_ADAPT_REFRESH_CAP, SELIND_LN1_RX), 0x8f);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_HS_ADAPT_INITIAL_CAP, SELIND_LN0_RX), 0x8f);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_HS_ADAPT_INITIAL_CAP, SELIND_LN1_RX), 0x8f);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);
	/* CB PHY override capability. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBENBLCPBATTRWR, 0), 0x0);
	/* ULP control. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBUPLH8, 0), 0x1);
	/* CB Override core_clk control 2. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBREFCLKCTRL2, 0), 0x90);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);

	return ret;
}

static int ufs_s32n_link_startup_pre_change(struct ufs_hba *hba)
{
	struct s32n_ufs *ufs = ufshcd_get_variant(hba);
	struct device *dev = hba->dev;
	u16 data = 0;
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

	ret = ufs_s32n_phy_configure(hba);
	if (ret) {
		dev_err(dev, "UFS MPHY configuration failed!\n");
		return ret;
	}

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

	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);
	ret |= ufs_s32n_phy_read_cr(hba, MPHY_REG_UNKNOWN_1, &data);

	if (ufs->mphy_boot_mode == MPHY_BOOT_COPY_SRAM) {
		/* SRAM Mode. */
		if ((ret != 0) || data != 0x080f) {
			dev_err(dev, "Unexpected data read from MPHY register!\n");
			return ret ? ret : -EIO;
		}

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

	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x1);
	/* Enable MPHY. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYDISABLE, 0), 0x0);

	if (ret)
		return ret;

	return ufshcd_check_hibern8(hba, 2);
}

static int ufs_s32n_link_startup_post_change(struct ufs_hba *hba)
{
	struct s32n_ufs *ufs = ufshcd_get_variant(hba);
	unsigned int data = 0;
	u16 reg_val = 0;
	int ret = 0;

	/* If MPHY initialization is not handled by TF-A,
	 * invoke the generic DW power mode update procedure.
	 */
	if (ufs->mphy_boot_mode == MPHY_BOOT_NONE)
		return 0;

	/* Set Local DeviceID. */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(N_DEVICEID, 0), 0x1);

	/* Set Connection State to IDLE (it allows CPort Attributes to be set). */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(T_CONNECTIONSTATE, CPORT_0), CPORT_IDLE);

	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(T_CPORTFLAGS, CPORT_0),
			     CPORT_FLAGS(CSV_N_ON, CSD_N_ON, E2EFC_OFF));
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(T_CONNECTIONSTATE, CPORT_0), CPORT_CONNECTED);
	if (ret)
		return ret;

	ret = ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(T_CONNECTIONSTATE, CPORT_0), &data);
	if ((ret != 0) || (data != CPORT_CONNECTED))
		return ret;

	/* SlowAuto_Mode for both Tx & Rx. */
	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(PA_PWRMODE, 0), &data);
	if ((ret != 0) || (data != 0x55))
		return ret;

	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(PA_HSSERIES, 0), &data);
	if ((ret != 0) || (data != PA_HS_MODE_A))
		return ret;

	/* Performing MPHY configuration for rate change:
	 * CB rate selection: 0 - rate A, 1 - rate B;
	 */
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(CBRATESEL, 0), 0x1);
	ret |= ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VS_MPHYCFGUPDT, 0), 0x01);

	ret |= ufs_s32n_phy_write_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_0, 0xc);
	ret |= ufs_s32n_phy_read_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_0, &reg_val);

	ret |= ufs_s32n_phy_write_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_1, 0xc);
	ret |= ufs_s32n_phy_read_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_1, &reg_val);

	ret |= ufs_s32n_phy_write_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_0, 0x8);
	ret |= ufs_s32n_phy_read_cr(hba, RAWLANEN_DIG_PCS_XF_RX_PCS_OUT_0, &reg_val);
	if ((ret != 0) || ((reg_val & MPHY_REG_LSB_MASK) != 0))
		return ret;

	ret |= ufs_s32n_phy_write_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_1, 0x8);
	ret |= ufs_s32n_phy_read_cr(hba, RAWLANEN_DIG_PCS_XF_RX_PCS_OUT, &reg_val);
	if ((ret != 0) || ((reg_val & MPHY_REG_LSB_MASK) != 0))
		return ret;

	ret |= ufs_s32n_phy_write_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_0, 0x0);
	ret |= ufs_s32n_phy_write_cr(hba, RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_1, 0x0);

	/* Completed host PHY programming before rate change PMC
	 * REQUESTING POWER MODE CHANGE, POWERMODE REQUEST 0x44.
	 */
	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(TX_HS_UNTERMINATED_LINE_DRIVE_CAP, 0), &data);
	if ((ret != 0) || (data != 0))
		return ret;

	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(TX_LS_TERMINATED_LINE_DRIVE_CAP, 0), &data);
	if ((ret != 0) || (data != 0))
		return ret;

	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(RX_HS_UNTERMINATED_CAP, 0), &data);
	if ((ret != 0) || (data != 0))
		return ret;

	ret |= ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(RX_LS_TERMINATED_CAP, 0), &data);
	if ((ret != 0) || (data != 0))
		return ret;

	return 0;
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

static int ufs_s32n_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
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
};

static const struct of_device_id ufs_s32n79_match[] = {
	{
		.compatible = "nxp,s32n79-ufshc",
		.data = &ufs_hba_s32n79_vops,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ufs_s32n79_match);

static int ufs_s32n79_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;

	of_id = of_match_node(ufs_s32n79_match, pdev->dev.of_node);

	return ufshcd_pltfrm_init(pdev, of_id->data);
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
