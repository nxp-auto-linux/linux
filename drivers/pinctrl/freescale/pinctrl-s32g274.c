/*
 * s32g274 pinctrl driver based on imx pinmux and pinconf core
 *
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-s32.h"

enum s32_pins {
	S32G274_MSCR_PA_00 =  0,
};

/* Pad names for the pinmux subsystem */
static const struct pinctrl_pin_desc s32_pinctrl_pads[] = {

};


static struct s32_pinctrl_soc_info s32_pinctrl_info = {
	.pins = s32_pinctrl_pads,
	.npins = ARRAY_SIZE(s32_pinctrl_pads),
};


static const struct of_device_id s32_pinctrl_of_match[] = {
	{
		.compatible = "fsl,s32g274-siul2",
		.data = (void *) PINCTRL_V1,
	},
	{
		.compatible = "fsl,s32g274-siul2-pinctrl",
		.data = (void *) PINCTRL_V2,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s32_pinctrl_of_match);

static int s32g274_pinctrl_probe(struct platform_device *pdev)
{
	enum s32_pinctrl_version vers;
	const struct of_device_id *of_id =
		of_match_device(s32_pinctrl_of_match, &pdev->dev);

	if (!of_id)
		return -ENODEV;

	vers = (enum s32_pinctrl_version) of_id->data;

	return s32_pinctrl_probe(pdev, &s32_pinctrl_info, vers);
}

static struct platform_driver s32g274_pinctrl_driver = {
	.driver = {
		.name = "s32g274-siul2-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = s32_pinctrl_of_match,
	},
	.probe = s32g274_pinctrl_probe,
	.remove = s32_pinctrl_remove,
};

module_platform_driver(s32g274_pinctrl_driver);

MODULE_AUTHOR("Matthew Nunez <matthew.nunez@nxp.com>");
MODULE_DESCRIPTION("NXP S32G274 pinctrl driver");
MODULE_LICENSE("GPL v2");
