/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright 2021 NXP
 */
#ifndef NXP_S32CC_XPCS_H
#define NXP_S32CC_XPCS_H

#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/types.h>
#include <linux/phylink.h>

struct s32cc_xpcs;

struct s32cc_xpcs_ops {
	int (*get_state)(struct s32cc_xpcs *xpcs,
			 struct phylink_link_state *state);
	int (*init)(struct s32cc_xpcs **xpcs, struct device *dev,
		    unsigned char id, void __iomem *base, bool ext_clk,
		    unsigned long rate);
	int (*power_on)(struct s32cc_xpcs *xpcs);
	int (*config)(struct s32cc_xpcs *xpcs,
		      const struct phylink_link_state *state);
	int (*vreset)(struct s32cc_xpcs *xpcs);
	int (*wait_vreset)(struct s32cc_xpcs *xpcs);
	int (*init_mplla)(struct s32cc_xpcs *xpcs);
	int (*reset_rx)(struct s32cc_xpcs *xpcs);
	void (*release)(struct s32cc_xpcs *xpcs);
	bool (*has_valid_rx)(struct s32cc_xpcs *xpcs);
};

const struct s32cc_xpcs_ops *s32cc_xpcs_get_ops(void);

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

