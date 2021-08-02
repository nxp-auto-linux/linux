/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Copyright 2021 NXP
 */
#ifndef NXP_S32CC_PCIE_PHY_SUBMODE_H
#define NXP_S32CC_PCIE_PHY_SUBMODE_H

enum pcie_phy_mode {
	CRNS = 0, /* Common Reference Clock, No Spread Spectrum */
	CRSS = 1, /* Common Reference Clock, Spread Spectrum */
	SRIS = 2  /* Separate Reference Clock, Spread Spectrum */
};

#endif
