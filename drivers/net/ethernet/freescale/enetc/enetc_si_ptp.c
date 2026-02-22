// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
//
// ENETC SI (Station Interface) PTP clock driver
//
// This driver exposes a read‑only 64‑bit hardware timestamp to Linux PTP
// framework so that phc2sys can sync system time from this hardware clock.
//
/* Copyright 2026 NXP */

#include <linux/ptp_clock_kernel.h>
#include "enetc.h"

#define ENETC_PTP_TIME_LO   0x000	/* offset for low 32 bits */
#define ENETC_PTP_TIME_HI   0x004	/* offset for high 32 bits */

struct enetc_si_ptp {
	struct ptp_clock_info caps;
	void __iomem *regs;		/* base pointer to ENETC PTP regs */
};

static int enetc_si_ptp_gettime(struct ptp_clock_info *ptp,
				struct timespec64 *ts)
{
	struct enetc_si_ptp *priv = container_of(ptp, struct enetc_si_ptp,
						 caps);

	u32 hi, lo;
	u64 ns;

	/*
	 * reading the low order 32-bits from SICTR0 will capture the time
	 * such that a subsequent read from SICTR1 will provide the correct
	 * 64-bit time value
	 */
	lo = enetc_rd_reg(priv->regs + ENETC_PTP_TIME_LO);
	hi = enetc_rd_reg(priv->regs + ENETC_PTP_TIME_HI);

	ns = ((u64)hi << 32) | lo;

	/* assume counter in nanoseconds */
	*ts = ns_to_timespec64(ns);

	return 0;
}

static const struct ptp_clock_info enetc_si_ptp_caps = {
	.owner      = THIS_MODULE,
	.name       = "ENETC SI read-only PTP",
	.gettime64  = enetc_si_ptp_gettime,
	/* no setters / adjustments supported */
};

int enetc_si_ptp_create(struct enetc_si *si)
{
	struct device *dev = &si->pdev->dev;
	struct enetc_si_ptp *priv;
	struct ptp_clock *ptp;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regs = si->hw.reg + ENETC_SICTR0;
	priv->caps = enetc_si_ptp_caps;

	ptp = ptp_clock_register(&priv->caps, dev);
	if (IS_ERR(ptp))
		return PTR_ERR(ptp);

	si->si_ptp = ptp;

	dev_info(dev, "ENETC SI read-only PTP clock registered\n");

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_si_ptp_create);

void enetc_si_ptp_destroy(struct enetc_si *si)
{
	if (si->si_ptp)
		ptp_clock_unregister(si->si_ptp);

	si->si_ptp = NULL;
}
EXPORT_SYMBOL_GPL(enetc_si_ptp_destroy);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("ENETC SI read-only PTP clock driver");
