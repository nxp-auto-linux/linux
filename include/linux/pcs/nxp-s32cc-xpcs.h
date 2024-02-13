/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright 2021-2022,2024 NXP
 */
#ifndef NXP_S32CC_XPCS_H
#define NXP_S32CC_XPCS_H

#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/types.h>
#include <linux/phylink.h>

struct s32cc_xpcs;

enum pcie_xpcs_mode {
	NOT_SHARED,
	PCIE_XPCS_1G,
	PCIE_XPCS_2G5,
};

struct s32cc_xpcs_ops {
	int (*init)(struct s32cc_xpcs **xpcs, struct device *dev,
		    unsigned char id, void __iomem *base, bool ext_clk,
		    unsigned long rate, enum pcie_xpcs_mode pcie_shared);
	int (*power_on)(struct s32cc_xpcs *xpcs);
	int (*config)(struct s32cc_xpcs *xpcs,
		      const struct phylink_link_state *state);
	int (*vreset)(struct s32cc_xpcs *xpcs);
	int (*wait_vreset)(struct s32cc_xpcs *xpcs);
	int (*init_plls)(struct s32cc_xpcs *xpcs);
	int (*reset_rx)(struct s32cc_xpcs *xpcs);
	void (*release)(struct s32cc_xpcs *xpcs);
	bool (*has_valid_rx)(struct s32cc_xpcs *xpcs);
	int (*pre_pcie_2g5)(struct s32cc_xpcs *xpcs);

	/* These function are planned to be used directly
	 * by phylink in newer kernels (starting from 5.10).
	 */
	int (*xpcs_config)(struct s32cc_xpcs *xpcs,
			   const struct phylink_link_state *state);
	int (*xpcs_get_state)(struct s32cc_xpcs *xpcs,
			      struct phylink_link_state *state);
};

const struct s32cc_xpcs_ops *s32cc_xpcs_get_ops(void);

struct phylink_pcs *s32cc_xpcs_get_pcs(struct s32cc_xpcs *xpcs);

/**
 * s32cc_phy2xpcs() - Get XPCS instance associated with a PHY
 *
 * @phy: A generic PHY obtained from S32CC SerDes driver.
 *
 * The return value will be the XPCS instance associate with the
 * passed SerDes PHY.
 */
struct s32cc_xpcs *s32cc_phy2xpcs(struct phy *phy);

#endif

