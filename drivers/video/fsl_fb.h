/*
 * Copyright 2012-2014,2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Freescale fsl-FB device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef FSL_FB_H_
#define FSL_FB_H_

/****************************************
 * framebuffer DCU info
 ****************************************/
struct dcu_fb_data {
	/* dynamic size */
	struct fb_info **fsl_dcu_info;
	struct device *dev;
	void __iomem *reg_base;
	unsigned int irq;
	struct clk *pxl_clk;
	struct clk *dcu_clk;
};

/****************************************
 * DCU layer info
 ****************************************/
struct mfb_info {
	int index;
	unsigned long pseudo_palette[16];
	unsigned char alpha;
	unsigned char blend;
	unsigned int count;
	int x_layer_d;	/* layer display x offset to physical screen */
	int y_layer_d;	/* layer display y offset to physical screen */
	int tiled;	/* layer in tiled format */
	struct dcu_fb_data *parent;
};

#endif /* FSL_FB_H_ */
