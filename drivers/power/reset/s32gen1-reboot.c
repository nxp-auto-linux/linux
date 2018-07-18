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

/* MC_RGM_FRET */
#define MC_RGM_FRET(mc_rgm)		((mc_rgm) + 0x18)

#define MC_RGM_FRET_VALUE		(0xF)

#define OF_MATCH_MC_RGM		0
#define OF_MATCH_MC_ME		1

static const struct of_device_id s32gen1_reboot_of_match[] = {
	{ .compatible = "fsl,s32gen1-reset" },
	{}
};

struct	s32gen1_reboot_priv {
	void __iomem *mc_me;
	void __iomem *mc_rgm;
};

static struct s32gen1_reboot_priv s32gen1_reboot_priv;

static int s32gen1_reboot(struct notifier_block *this, unsigned long mode,
			  void *cmd)
{
	unsigned long timeout;

	if (s32gen1_reboot_priv.mc_rgm) {
		writeb(MC_RGM_FRET_VALUE,
			MC_RGM_FRET(s32gen1_reboot_priv.mc_rgm));
	}

	if (s32gen1_reboot_priv.mc_me) {
		writel_relaxed(MC_ME_MODE_CONF_FUNC_RST,
				MC_ME_MODE_CONF(s32gen1_reboot_priv.mc_me));

		writel_relaxed(MC_ME_MODE_UPD_UPD,
				MC_ME_MODE_UPD(s32gen1_reboot_priv.mc_me));

		writel_relaxed(MC_ME_CTL_KEY_KEY,
				MC_ME_CTL_KEY(s32gen1_reboot_priv.mc_me));
		writel_relaxed(MC_ME_CTL_KEY_INVERTEDKEY,
				MC_ME_CTL_KEY(s32gen1_reboot_priv.mc_me));
	}

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout))
		cpu_relax();

	return 0;
}

static struct notifier_block s32gen1_reboot_nb = {
	.notifier_call = s32gen1_reboot,
	.priority = 192,
};

static int s32gen1_reboot_probe(struct platform_device *pdev)
{
	const struct of_device_id *of;
	int err;

	of = of_match_device(s32gen1_reboot_of_match, &pdev->dev);
	if (of == NULL)
		return -ENODEV;

	s32gen1_reboot_priv.mc_me = of_iomap(pdev->dev.of_node, OF_MATCH_MC_ME);
	if (!s32gen1_reboot_priv.mc_me) {
		dev_err(&pdev->dev, "Can not map resource\n");
		return -ENODEV;
	}

	s32gen1_reboot_priv.mc_rgm = of_iomap(pdev->dev.of_node,
					      OF_MATCH_MC_RGM);
	if (!s32gen1_reboot_priv.mc_rgm) {
		iounmap(s32gen1_reboot_priv.mc_me);
		dev_err(&pdev->dev, "Can not map resource\n");
		return -ENODEV;
	}

	err = register_restart_handler(&s32gen1_reboot_nb);
	if (err) {
		iounmap(s32gen1_reboot_priv.mc_rgm);
		iounmap(s32gen1_reboot_priv.mc_me);
		dev_err(&pdev->dev, "Failed to register handler\n");
		return err;
	}

	return 0;
}

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
