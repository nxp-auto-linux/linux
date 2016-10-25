// SPDX-License-Identifier: GPL-2.0+
/*
 *	Driver for Broadcom 89xxx SOCs integrated PHYs
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Copyright 2015 Freescale Semiconductor, Inc.
 *
 */
#include <linux/module.h>
#include <linux/phy.h>

#define MII_BCM89XXX_IR		0x1b	/* interrupt register */
#define MII_BCM89XXX_IR_DUPLEX	0x0003	/* duplex changed */
#define MII_BCM89XXX_IR_SPEED	0x0002	/* speed changed */
#define MII_BCM89XXX_IR_LINK	0x0001	/* link changed */

MODULE_DESCRIPTION("Broadcom 89xxx internal PHY driver");
MODULE_AUTHOR("Stoica Cosmin-Stefan <cosminstefan.stoica@freescale.com>");
MODULE_LICENSE("GPL");

static int bcm89xxx_config_init(struct phy_device *phydev)
{
	int reg;

	reg = phy_read(phydev, MII_BCM89XXX_IR);
	if (reg < 0)
		return reg;

	/* Unmask events we are interested in  */
	reg = ~(MII_BCM89XXX_IR_DUPLEX |
		MII_BCM89XXX_IR_SPEED |
		MII_BCM89XXX_IR_LINK);

	return phy_write(phydev, MII_BCM89XXX_IR, reg);
}

static int bcm89xxx_ack_interrupt(struct phy_device *phydev)
{
	int reg;

	/* Clear pending interrupts.  */
	reg = phy_read(phydev, MII_BCM89XXX_IR);
	if (reg < 0)
		return reg;

	return 0;
}

static int bcm89xxx_config_intr(struct phy_device *phydev)
{
	int reg, err;

	reg = phy_read(phydev, MII_BCM89XXX_IR);
	if (reg < 0)
		return reg;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		reg &= ~0xFFFF;

	err = phy_write(phydev, MII_BCM89XXX_IR, reg);
	return err;
}

static struct phy_driver bcm89xxx_driver[] = {
	{
		.phy_id		= 0x03625cde,
		.phy_id_mask	= 0xfffffff0,
		.name		= "Broadcom BCM89XXX (1)",
		/* ASYM_PAUSE bit is marked RO in datasheet, so don't cheat */
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_IS_INTERNAL,
		.config_init	= bcm89xxx_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.ack_interrupt	= bcm89xxx_ack_interrupt,
		.config_intr	= bcm89xxx_config_intr,
	}
};

static int __init bcm89xxx_phy_init(void)
{
	return phy_drivers_register(bcm89xxx_driver,
				    ARRAY_SIZE(bcm89xxx_driver),
				    THIS_MODULE);
}

static void __exit bcm89xxx_phy_exit(void)
{
	phy_drivers_unregister(bcm89xxx_driver,
			       ARRAY_SIZE(bcm89xxx_driver));
}

module_init(bcm89xxx_phy_init);
module_exit(bcm89xxx_phy_exit);

static struct mdio_device_id __maybe_unused bcm89xxx_tbl[] = {
	{ 0x03625cde, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, bcm89xxx_tbl);
