// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * NXP S32V reboot driver
 * Copyright 2015-2016 Freescale Semiconductors, Inc.
 * Copyright 2017 NXP
 * Author: Stoica Cosmin <cosmin.stoica@nxp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <asm/system_misc.h>

/* MC_ME_GS */
#define MC_ME_GS(mc_me)			((mc_me) + 0x00000000)
#define MC_ME_GS_S_MTRANS		(1 << 27)

/* MC_ME_MCTL */
#define MC_ME_MCTL(mc_me)		((mc_me) + 0x00000004)
#define MC_ME_MCTL_RESET		(0x0 << 28)
#define MC_ME_MCTL_KEY			(0x00005AF0)
#define MC_ME_MCTL_INVERTEDKEY		(0x0000A50F)

/* MC_RGM_FRET */
#define MC_RGM_FRET(mc_rgm)		((mc_rgm) + 0x607)

#define MC_RGM_FRET_VALUE		(0xF)

struct	s32v_reboot_priv {
	struct device *dev;
	void __iomem *mc_me;
	void __iomem *mc_rgm;
};

static struct s32v_reboot_priv	*s32v_reboot_priv;

static void s32v_reboot(enum reboot_mode reboot_mode, const char *cmd)
{
	struct s32v_reboot_priv	*priv = s32v_reboot_priv;
	unsigned long timeout;

	if (s32v_reboot_priv) {
		writeb(MC_RGM_FRET_VALUE,
			MC_RGM_FRET(s32v_reboot_priv->mc_rgm));
		writel_relaxed(MC_ME_MCTL_RESET | MC_ME_MCTL_KEY,
				MC_ME_MCTL(priv->mc_me));
		writel_relaxed(MC_ME_MCTL_RESET | MC_ME_MCTL_INVERTEDKEY,
				MC_ME_MCTL(priv->mc_me));
		while ((readl_relaxed(MC_ME_GS(priv->mc_me))
			& MC_ME_GS_S_MTRANS) != 0x00000000)
			;
	}

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout))
		cpu_relax();

}

static int s32v_reboot_probe(struct platform_device *pdev)
{
	struct device_node *np;
	s32v_reboot_priv = devm_kzalloc(&pdev->dev,
			sizeof(*s32v_reboot_priv), GFP_KERNEL);

	if (!s32v_reboot_priv)
		return -ENOMEM;

	s32v_reboot_priv->mc_me = of_iomap(pdev->dev.of_node, 0);
	if (!s32v_reboot_priv->mc_me) {
		devm_kfree(&pdev->dev, s32v_reboot_priv);
		dev_err(&pdev->dev, "can not map resource\n");
		return -ENODEV;
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_rgm");
	if (!np) {
		iounmap(s32v_reboot_priv->mc_me);
		devm_kfree(&pdev->dev, s32v_reboot_priv);
		dev_err(&pdev->dev, "unable to find MC_RGM node\n");
		return -ENODEV;
	}

	s32v_reboot_priv->mc_rgm = of_iomap(np, 0);
	if (!s32v_reboot_priv->mc_rgm) {
		iounmap(s32v_reboot_priv->mc_me);
		devm_kfree(&pdev->dev, s32v_reboot_priv);
		dev_err(&pdev->dev, "can not map resource\n");
		return -ENODEV;
	}

	s32v_reboot_priv->dev = &pdev->dev;

	arm_pm_restart = s32v_reboot;

	return 0;
}

static const struct of_device_id s32v_reboot_of_match[] = {
	{ .compatible = "fsl,s32v-reset" },
	{}
};

static struct platform_driver s32v_reboot_driver = {
	.probe = s32v_reboot_probe,
	.driver = {
		.name = "s32v-reset",
		.of_match_table = s32v_reboot_of_match,
	},
};

static int __init s32v_reboot_init(void)
{
	return platform_driver_register(&s32v_reboot_driver);
}
device_initcall(s32v_reboot_init);
