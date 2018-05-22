// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * NXP S32GEN1 reboot driver
 * Copyright 2018 NXP
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
#include <asm/system_misc.h>

/* MC_ME_CTL */
#define MC_ME_CTL_KEY(mc_me)		((mc_me) + 0x00000000)
#define MC_ME_CTL_KEY_KEY		(0x00005AF0)
#define MC_ME_CTL_KEY_INVERTEDKEY	(0x0000A50F)

/* MC_ME_MODE_CONF */
#define MC_ME_MODE_CONF(mc_me)		((mc_me) + 0x00000004)
#define MC_ME_MODE_CONF_FUNC_RST	(0x1 << 1)

/* MC_ME_MODE_UPD */
#define MC_ME_MODE_UPD(mc_me)		((mc_me) + 0x00000008)
#define MC_ME_MODE_UPD_UPD		(0x1 << 0)

struct	s32gen1_reboot_priv {
	struct device *dev;
	void __iomem *mc_me;
};

static struct s32gen1_reboot_priv	*s32gen1_reboot_priv;

static void s32gen1_reboot(enum reboot_mode reboot_mode, const char *cmd)
{
	struct s32gen1_reboot_priv	*priv = s32gen1_reboot_priv;
	unsigned long timeout;

	if (priv) {
		writel_relaxed(MC_ME_MODE_CONF_FUNC_RST,
				MC_ME_MODE_CONF(priv->mc_me));

		writel_relaxed(MC_ME_MODE_UPD_UPD,
				MC_ME_MODE_UPD(priv->mc_me));

		writel_relaxed(MC_ME_CTL_KEY_KEY,
				MC_ME_CTL_KEY(priv->mc_me));
		writel_relaxed(MC_ME_CTL_KEY_INVERTEDKEY,
				MC_ME_CTL_KEY(priv->mc_me));
	}

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout))
		cpu_relax();
}

static int s32gen1_reboot_probe(struct platform_device *pdev)
{
	s32gen1_reboot_priv = devm_kzalloc(&pdev->dev,
		sizeof(*s32gen1_reboot_priv), GFP_KERNEL);
	if (!s32gen1_reboot_priv)
		return -ENOMEM;

	s32gen1_reboot_priv->mc_me = of_iomap(pdev->dev.of_node, 0);
	if (!s32gen1_reboot_priv->mc_me) {
		devm_kfree(&pdev->dev, s32gen1_reboot_priv);
		dev_err(&pdev->dev, "Can not map resource\n");
		return -ENODEV;
	}

	s32gen1_reboot_priv->dev = &pdev->dev;

	arm_pm_restart = s32gen1_reboot;

	return 0;
}

static const struct of_device_id s32gen1_reboot_of_match[] = {
	{ .compatible = "fsl,s32gen1-reset" },
	{}
};

static struct platform_driver s32gen1_reboot_driver = {
	.probe = s32gen1_reboot_probe,
	.driver = {
		.name = "s32gen1-reset",
		.of_match_table = s32gen1_reboot_of_match,
	},
};

static int __init s32gen1_reboot_init(void)
{
	return platform_driver_register(&s32gen1_reboot_driver);
}
device_initcall(s32gen1_reboot_init);
