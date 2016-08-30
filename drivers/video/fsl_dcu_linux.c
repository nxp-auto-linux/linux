/*
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
 *
 * Freescale fsl-DCU device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>
#include <linux/cdev.h>
#include <linux/videodev2.h>
#include <linux/uaccess.h>

#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <asm/current.h>

#include <uapi/video/fsl_dcu_ioctl.h>

#include "fsl_dcu_regs.h"
#include "fsl_dcu_regmacros.h"
#include "fsl_dcu.h"
#include "fsl_dcu_linux.h"
#include "fsl_fb.h"

/**********************************************************
 * DCU disable writeback, EXPERIMENTAL code
 **********************************************************/
#define DCU_DISABLE_WRITEBACK 1

/**********************************************************
 * DCU regbase, found in agnostic DCU driver layer
 **********************************************************/
/* number of DCU units */
#define DCU_DEV_COUNT	1

/* wb_data */
long wb_len;
char *wb_vir_ptr;
unsigned long wb_phys_ptr;

/**********************************************************
 * DCU defines
 **********************************************************/
#define MAJOR_NUM			100
#define DEVICE_NAME			"dcu0"

#define DRIVER_NAME			"fsl_dcu"
#define DCU_LAYER_NUM_MAX	8

#define TCON_CTRL			0x0
#define TCON_BYPASS_ENABLE	(1 << 29)
#define TCON_ENABLE			(1 << 31)

#define LDB_CTRL			0x0
#define LDB_DI0_VSYNC_LOW	(1 << 9)
#define LDB_DI0_VSYNC_HIGH	(0 << 9)
#define LDB_DWHC0			(1 << 5)
#define LDB_BITMAPCH0_SPWG	(0 << 6)
#define LDB_BITMAPCH0_JEIDA	(1 << 6)
#define LDB_CH0MOD_EN		(1 << 0)

/**********************************************************
 * Macros for tracing
 **********************************************************/
/* #define __LOG_TRACE__ 1 */

#ifdef __LOG_TRACE__
	#define __TRACE__ dev_info(&dcu_pdev->dev, "DCU: %s\n", __func__)
	#define __MSG_TRACE__(string, args...) dev_info(&dcu_pdev->dev, \
		"DCU: %s : %d : " string, __func__, __LINE__, ##args)
#else
	#define __TRACE__
	#define __MSG_TRACE__(string, args...)
	#define __HERE__ dev_info(&dcu_pdev->dev, " HERE %s\n", __func__)
#endif

/**********************************************************
 * GLOBAL DCU configuration registers
 **********************************************************/
uint64_t *DCU_BASE_ADDRESS;
void __iomem *dcu_reg_base;
struct clk *dcu_clk;
struct dcu_fb_data *dcu_fb_data;
struct platform_device *dcu_pdev;
struct cdev *dcu_cdev;
struct class *dcu_class;
dev_t dcu_devno;
uint32_t dcu_clk_val;

#define DCU_INIT_TRUE		0
#define DCU_INIT_ERR_PROBE	1
#define DCU_INIT_ERR_CFG	2
int dcu_init_status = DCU_INIT_ERR_PROBE;

/* The state of a process which wants to wait for VSYNC */
#define EVENT_STATUS_CLEAR	0
#define DCU_STATUS_WAITING	1
#define DCU_STATUS_WAITED	2

/* We use events and wait queues because completions are unsuitable */
int event_condition_list[DCU_LAYERS_NUM_MAX];
wait_queue_head_t dcu_event_queue;

/* A global CLUT written by a process and read during VBLANK */
uint32_t dcu_clut[256];
uint32_t dcu_clut_size;
uint32_t dcu_clut_offset;
Dcu_Layer_t dcu_clut_layer;
spinlock_t dcu_clut_lock;

/* The most recent display configuration */
struct IOCTL_DISPLAY_CFG current_display_cfg;

/* The FB objects which listen for changes in display timings */
struct fb_info *timings_listener_list[DCU_LAYERS_NUM_MAX];
int timings_listener_list_size;

/**********************************************************
 * GLOBAL DCU & FB supported color formats
 **********************************************************/

/* The color formats supported by the DCU & FB drivers */
const struct dcu_fb_color_format DCU_FB_COLOR_FORMATS[] = {
	{"ARGB8888",  { {16, 8, 0}, {8, 8, 0}, {0, 8, 0}, {24, 8, 0} }, 32},
	{"RGB888",    { {16, 8, 0}, {8, 8, 0}, {0, 8, 0}, { 0, 0, 0} }, 24},
	{"RGB565",    { {11, 5, 0}, {5, 6, 0}, {0, 5, 0}, { 0, 0, 0} }, 16},
	{"ARGB1555",  { {10, 5, 0}, {5, 5, 0}, {0, 5, 0}, {15, 1, 0} }, 16},
	{"ARGB4444",  { { 8, 4, 0}, {4, 4, 0}, {0, 4, 0}, {12, 4, 0} }, 16},

	{"GRAY-8BPP", { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0}, { 0, 8, 0} },  8},
	{"GRAY-4BPP", { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0}, { 0, 4, 0} },  4},

	{"CLUT-8BPP", { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0}, { 0, 0, 0} },  8},
	{"CLUT-4BPP", { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0}, { 0, 0, 0} },  4},
	{"CLUT-2BPP", { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0}, { 0, 0, 0} },  2},
	{"CLUT-1BPP", { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0}, { 0, 0, 0} },  1},
};

const Dcu_BPP_t DCU_FB_COLOR_BPP[] = {
	DCU_BPP_32,
	DCU_BPP_24,
	DCU_BPP_16,
	DCU_BPP_16_ARGB1555,
	DCU_BPP_16_ARGB4444,
	DCU_BPP_TRANS_8,
	DCU_BPP_TRANS_4,
	DCU_BPP_8,
	DCU_BPP_4,
	DCU_BPP_2,
	DCU_BPP_1
};

const int dcu_fb_color_format_count =
	sizeof(DCU_FB_COLOR_FORMATS) /
	sizeof(DCU_FB_COLOR_FORMATS[0]);

/**********************************************************
 * External HDMI function definitions
 **********************************************************/

#ifndef CONFIG_FB_MXS_SII902X
/* FIXME: avoid direct communication with HDMI driver */
struct fb_monspecs sii902x_get_monspecs(void)
{
	struct fb_monspecs monspecs;

	dev_info(&dcu_pdev->dev,
		"No HDMI SII9022 support. Recompile with FB_MXS_SII902X activated.\n");
	memset(&monspecs, 0, sizeof(struct fb_monspecs));

	return monspecs;
}
#endif

/**********************************************************
 * FUNCTION: fsl_dcu_get_dcufb
 **********************************************************/
struct dcu_fb_data *fsl_dcu_get_dcufb(void)
{
	__TRACE__;
	return dcu_fb_data;
}
EXPORT_SYMBOL_GPL(fsl_dcu_get_dcufb);

/**********************************************************
 * FUNCTION: fsl_dcu_get_pdev
 **********************************************************/
struct platform_device *fsl_dcu_get_pdev(void)
{
	__TRACE__;
	return dcu_pdev;
}
EXPORT_SYMBOL_GPL(fsl_dcu_get_pdev);

/**********************************************************
 * FUNCTION: fsl_dcu_init_status
 **********************************************************/
int fsl_dcu_init_status(void)
{
	__TRACE__;
	return dcu_init_status;
}
EXPORT_SYMBOL_GPL(fsl_dcu_init_status);

/**********************************************************
 * FUNCTION: fsl_dcu_num_layers
 * INFO: number of layers is based on max blending layers
 **********************************************************/
int fsl_dcu_num_layers(void)
{
	__TRACE__;
	return DCU_LAYER_NUM_MAX;
}
EXPORT_SYMBOL_GPL(fsl_dcu_num_layers);

/**********************************************************
 * FUNCTION: fsl_dcu_registers
 **********************************************************/
void fsl_dcu_registers(void)
{
	dev_info(&dcu_pdev->dev, "-----------DCU REGS ----------\n");
	dev_info(&dcu_pdev->dev, "[REG : DCU_DCU_MODE]\t : %02x => %08x\n",
			0x10,
			readl(dcu_fb_data->reg_base + 0x10));
	dev_info(&dcu_pdev->dev, "[REG : DCU_BGND  ]\t : %02x => %08x\n",
			0x14,
			readl(dcu_fb_data->reg_base + 0x14));
	dev_info(&dcu_pdev->dev, "[REG : DCU_DISP_SIZE]\t : %02x => %08x\n",
			0x18,
			readl(dcu_fb_data->reg_base + 0x18));
	dev_info(&dcu_pdev->dev, "[REG : DCU_HSYN_PARA]\t : %02x => %08x\n",
			0x1C,
			readl(dcu_fb_data->reg_base + 0x1C));
	dev_info(&dcu_pdev->dev, "[REG : DCU_VSYN_PARA]\t : %02x => %08x\n",
			0x20,
			readl(dcu_fb_data->reg_base + 0x20));
	dev_info(&dcu_pdev->dev, "[REG : DCU_SYN_POL]\t : %02x => %08x\n",
			0x24,
			readl(dcu_fb_data->reg_base + 0x24));
	dev_info(&dcu_pdev->dev, "[REG : DCU_THRESHOLD]\t : %02x => %08x\n",
			0x28,
			readl(dcu_fb_data->reg_base + 0x28));
	dev_info(&dcu_pdev->dev, "[REG : DCU_INT_STATUS]\t : %02x => %08x\n",
			0x2C,
			readl(dcu_fb_data->reg_base + 0x2C));
	dev_info(&dcu_pdev->dev, "[REG : DCU_INT_MASK]\t : %02x => %08x\n",
			0x30,
			readl(dcu_fb_data->reg_base + 0x30));
	dev_info(&dcu_pdev->dev, "[REG : DCU_DIV_RATIO]\t : %02x => %08x\n",
			0x54,
			readl(dcu_fb_data->reg_base + 0x54));
	dev_info(&dcu_pdev->dev, "[REG : DCU_UPDATE_MODE]\t : %02x => %08x\n",
			0xCC,
			readl(dcu_fb_data->reg_base + 0xCC));
	dev_info(&dcu_pdev->dev, "-------------------------------\n");
}
EXPORT_SYMBOL_GPL(fsl_dcu_registers);

/**********************************************************
 * FUNCTION: fsl_dcu_get_color_format_match
 **********************************************************/
int fsl_dcu_get_color_format_match(const struct fb_var_screeninfo *var)
{
	struct dcu_fb_color_format user_format = {
		"",
		{
		{var->red.offset, var->red.length, var->red.msb_right},
		{var->green.offset, var->green.length, var->green.msb_right},
		{var->blue.offset, var->blue.length, var->blue.msb_right},
		{var->transp.offset, var->transp.length, var->transp.msb_right}
		},
		var->bits_per_pixel};
	int i;

	/* Search for an exact match for the color format */
	for (i = 0; i < dcu_fb_color_format_count; ++i)
		if ((memcmp(DCU_FB_COLOR_FORMATS[i].channels,
				user_format.channels,
				sizeof(user_format.channels)) == 0) &&
				(user_format.bpp ==
					DCU_FB_COLOR_FORMATS[i].bpp))
			break;

	if (i < dcu_fb_color_format_count)
		return i;

	/* Search for a default format using only the BPP value */
	for (i = 0; i < dcu_fb_color_format_count; ++i)
		if (user_format.bpp == DCU_FB_COLOR_FORMATS[i].bpp)
			return i;

	/* At this point, we could not identify any supported format match */
	dev_err(&dcu_pdev->dev,
			"Unsupported color format!\n");

	return dcu_fb_color_format_count;
}
EXPORT_SYMBOL_GPL(fsl_dcu_get_color_format_match);

/**********************************************************
 * FUNCTION: fsl_dcu_get_color_format_byname
 **********************************************************/
int fsl_dcu_get_color_format_byname(const char *format_name)
{
	int i;

	for (i = 0; i < dcu_fb_color_format_count; ++i)
		if (strcasecmp(DCU_FB_COLOR_FORMATS[i].format_name,
				format_name) == 0)
			return i;

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(fsl_dcu_get_color_format_byname);

/**********************************************************
 * FUNCTION: fsl_dcu_set_clut
 **********************************************************/
int fsl_dcu_set_clut(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct fb_cmap *cmap = &info->cmap;
	uint32_t clut[256], i;
	uint16_t clut_offset = 256 * mfbi->index;
	unsigned long flags;

	/* Convert the FB color-map to a CLUT */
	for (i = 0; i < cmap->len; ++i) {
		if (cmap->transp)
			clut[i] = (cmap->transp[i] & 0xFF) << 24;
		else
			clut[i] = 0xFF000000;

		clut[i] |= (cmap->red[i] & 0xFF) << 16;
		clut[i] |= (cmap->green[i] & 0xFF) << 8;
		clut[i] |= (cmap->blue[i] & 0xFF);
	}

	spin_lock_irqsave(&dcu_clut_lock, flags);

	dcu_clut_size = cmap->len;
	dcu_clut_layer = mfbi->index;
	dcu_clut_offset = clut_offset;
	memcpy(dcu_clut, clut, cmap->len * sizeof(uint32_t));

	spin_unlock_irqrestore(&dcu_clut_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_set_clut);

/**********************************************************
 * FUNCTION: fsl_dcu_config_layer
 **********************************************************/
int fsl_dcu_config_layer(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = info->par;
	Dcu_Size_t	layer_size;
	Dcu_Position_t	layer_pos;
	Dcu_Colour_t	layer_chroma_max;
	Dcu_Colour_t	layer_chroma_min;
	int color_format_idx;

	__TRACE__;

	layer_size.mHeight = var->yres;
	layer_size.mWidth = var->xres_virtual;
	DCU_SetLayerSize(0, mfbi->index, &layer_size);

	layer_pos.mX = mfbi->x_layer_d;
	layer_pos.mY = mfbi->y_layer_d;
	DCU_SetLayerPosition(0, mfbi->index, &layer_pos);

	DCU_SetLayerBuffAddr(0, mfbi->index, info->fix.smem_start);

	DCU_SetLayerForeground(0, mfbi->index, 0x0);
	DCU_SetLayerBackground(0, mfbi->index, 0x0);

	if ((info->fix.capabilities & FB_CAP_FOURCC) &&
		(var->grayscale == V4L2_PIX_FMT_UYVY)) {
		DCU_SetLayerBPP(0, mfbi->index, DCU_BPP_YCbCr422);
	} else {
		color_format_idx = fsl_dcu_get_color_format_match(var);

		if (color_format_idx >= dcu_fb_color_format_count)
			return -EINVAL;

		switch (DCU_FB_COLOR_BPP[color_format_idx]) {
		case DCU_BPP_TRANS_8:
		case DCU_BPP_TRANS_4:
			/* 8-BPP or 4-BPP grayscale */
			DCU_SetLayerForeground(0, mfbi->index, 0x00FFFFFF);
			DCU_SetLayerBackground(0, mfbi->index, 0x00000000);
			break;

		case DCU_BPP_8:
		case DCU_BPP_4:
		case DCU_BPP_2:
		case DCU_BPP_1:
			if (mfbi->index >= 8)
				/* Only first 8 layers can use the CLUT */
				return -EINVAL;
			break;

		default:
			break;
		}

		DCU_SetLayerBPP(0, mfbi->index,
			DCU_FB_COLOR_BPP[color_format_idx]);
	}

	DCU_SetLayerAlphaVal(0, mfbi->index, mfbi->alpha);
	DCU_SetLayerAlphaMode(0, mfbi->index, DCU_ALPHAKEY_WHOLEFRAME);
	DCU_LayerEnable(0, mfbi->index);

	layer_chroma_max.Blue_Value  = 0xFF;
	layer_chroma_max.Red_Value   = 0xFF;
	layer_chroma_max.Green_Value = 0xFF;

	layer_chroma_min.Blue_Value  = 0x0;
	layer_chroma_min.Red_Value   = 0x0;
	layer_chroma_min.Green_Value = 0x0;

	DCU_SetLayerChroma(0, mfbi->index,
		&layer_chroma_max, &layer_chroma_min);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_config_layer);

/**********************************************************
 * FUNCTION: fsl_dcu_reset_layer
 **********************************************************/
int fsl_dcu_reset_layer(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;

	Dcu_Size_t	layer_size;
	Dcu_Position_t	layer_pos;

	Dcu_Colour_t	layer_chroma_max;
	Dcu_Colour_t	layer_chroma_min;

	__TRACE__;

	layer_size.mHeight = 0;
	layer_size.mWidth = 0;
	DCU_SetLayerSize(0, mfbi->index, &layer_size);

	layer_pos.mX = 0;
	layer_pos.mY = 0;
	DCU_SetLayerPosition(0, mfbi->index, &layer_pos);

	DCU_SetLayerBuffAddr(0, mfbi->index, 0);
	DCU_SetLayerBPP(0, mfbi->index, DCU_BPP_1);
	DCU_SetLayerAlphaVal(0, mfbi->index, 0);
	DCU_SetLayerAlphaMode(0, mfbi->index, DCU_ALPHAKEY_OFF);
	DCU_LayerDisable(0, mfbi->index);

	layer_chroma_max.Blue_Value =  0x0;
	layer_chroma_max.Red_Value =  0x0;
	layer_chroma_max.Green_Value =  0x0;

	layer_chroma_min.Blue_Value =  0x0;
	layer_chroma_min.Red_Value =  0x0;
	layer_chroma_min.Green_Value =  0x0;

	DCU_SetLayerChroma(0, mfbi->index,
			&layer_chroma_max, &layer_chroma_min);

	DCU_SetLayerForeground(0, mfbi->index, 0);
	DCU_SetLayerBackground(0, mfbi->index, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_reset_layer);

/**********************************************************
 * FUNCTION: fsl_dcu_map_vram
 **********************************************************/
int fsl_dcu_map_vram(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;
	u32 smem_len = info->fix.line_length * info->var.yres_virtual;

	__TRACE__;

	info->fix.smem_len = smem_len;

	info->screen_base = dma_alloc_wc(info->device,
		info->fix.smem_len, (dma_addr_t *)&info->fix.smem_start,
		GFP_KERNEL);
	if (!info->screen_base) {
		dev_err(dcufb->dev,
			"DCU: unable to allocate memory for <fb%d> surface.\n",
			mfbi->index);
		return -ENOMEM;
	}

	memset(info->screen_base, 0, info->fix.smem_len);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_map_vram);

/**********************************************************
 * FUNCTION: fsl_dcu_unmap_vram
 **********************************************************/
void fsl_dcu_unmap_vram(struct fb_info *info)
{
	__TRACE__;
	if (!info->screen_base)
		return;

	dma_free_wc(info->device, info->fix.smem_len,
		info->screen_base, info->fix.smem_start);

	info->screen_base = NULL;
	info->fix.smem_start = 0;
	info->fix.smem_len = 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_unmap_vram);

/**********************************************************
 * FUNCTION: fsl_dcu_set_layer
 **********************************************************/
int fsl_dcu_set_layer(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	Dcu_Position_t layer_pos;
	uint32_t addr;

	__TRACE__;

	addr = info->fix.smem_start +
		(var->yoffset * var->xres_virtual *
		var->bits_per_pixel >> 3);

	DCU_SetLayerHorizontalSkip(0, mfbi->index,
		var->xoffset,
		var->xres_virtual - var->xres - var->xoffset);

	layer_pos.mX = mfbi->x_layer_d - var->xoffset;
	layer_pos.mY = mfbi->y_layer_d;

	DCU_SetLayerBuffAddr(0, mfbi->index, addr);
	DCU_SetLayerPosition(0, mfbi->index, &layer_pos);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_set_layer);

/**********************************************************
 * FUNCTION: fsl_dcu_irq
 **********************************************************/
irqreturn_t fsl_dcu_irq(int irq, void *dev_id)
{
	__TRACE__;
	return IRQ_HANDLED;
}

/**********************************************************
 * FUNCTION: fsl_init_ldb
 **********************************************************/
int fsl_init_ldb(struct device_node *np, DCU_DISPLAY_TYPE display_type)
{
	struct device_node *ldb_np;
	struct platform_device *ldb_pdev;
	struct resource *ldb_res;
	void __iomem *ldb_reg;
	struct clk *ldb_clk;

	resource_size_t size;
	const char *name;

	ldb_np = of_parse_phandle(np, "ldb-controller", 0);
	if (!ldb_np)
		return -EINVAL;

	ldb_pdev = of_find_device_by_node(ldb_np);
	if (!ldb_pdev)
		return -EINVAL;

	ldb_res = platform_get_resource(ldb_pdev, IORESOURCE_MEM, 0);
	if (!ldb_res || resource_type(ldb_res) != IORESOURCE_MEM) {
		dev_err(&ldb_pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	ldb_clk = devm_clk_get(&ldb_pdev->dev, "ldb");
	if (IS_ERR(ldb_clk))
		dev_err(&ldb_pdev->dev, "could not get clock\n");
	else
		clk_prepare_enable(ldb_clk);

	size = resource_size(ldb_res);
	name = ldb_res->name ?: dev_name(&ldb_pdev->dev);

	if (!devm_request_mem_region(&ldb_pdev->dev,
			ldb_res->start, size, name)){
		dev_err(&ldb_pdev->dev,
			"can't request region for resource %pR\n", ldb_res);
		return -EBUSY;
	}

	if (ldb_res->flags & IORESOURCE_CACHEABLE)
		ldb_reg = devm_ioremap(&ldb_pdev->dev, ldb_res->start, size);
	else
		ldb_reg = devm_ioremap(&ldb_pdev->dev,
				ldb_res->start, size);

	if (!ldb_reg) {
		dev_err(&ldb_pdev->dev,
				"ioremap failed for resource %pR\n", ldb_res);
		devm_release_mem_region(&ldb_pdev->dev, ldb_res->start, size);
		ldb_reg = ERR_PTR(-ENOMEM);
	}

	/* LDB settings according to display type HDMI/LVDS */
	if (display_type == DCU_DISPLAY_LVDS)
		writel(LDB_DI0_VSYNC_LOW | LDB_DWHC0
		| LDB_CH0MOD_EN | LDB_BITMAPCH0_JEIDA,
			ldb_reg + LDB_CTRL);
	else if (display_type == DCU_DISPLAY_HDMI)
		writel(0, ldb_reg + LDB_CTRL);

	devm_release_mem_region(&ldb_pdev->dev, ldb_res->start, size);

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_turn_panel_on
 **********************************************************/
int fsl_turn_panel_on(struct device_node *np,
	DCU_DISPLAY_TYPE display_type)
{
	int err = 0;
	int panel_data_gpio;
	int panel_backlight_gpio;

	panel_data_gpio = of_get_named_gpio(np, "panel-data-gpio", 0);
	if (!gpio_is_valid(panel_data_gpio)) {
		dev_err(&dcu_pdev->dev, "DCU: failed to get panel data GPIO\n");
		return err;
	}

	err = gpio_request_one(panel_data_gpio, GPIOF_OUT_INIT_HIGH,
			"panel_data_gpio");

	if (err) {
		dev_err(&dcu_pdev->dev,
			"DCU: failed to set panel data GPIO: %d\n", err);
		return err;
	}

	panel_backlight_gpio = of_get_named_gpio(np,
			"panel-backlight-gpio", 0);
	if (!gpio_is_valid(panel_backlight_gpio))
		dev_warn(&dcu_pdev->dev,
			"DCU: failed to get panel backlight GPIO\n");

	err = gpio_request_one(panel_backlight_gpio, GPIOF_OUT_INIT_HIGH,
			"panel_backlight_gpio");
	if (err)
		dev_warn(&dcu_pdev->dev,
			"DCU: failed to set panel backlight GPIO: %d\n", err);

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_dcu_wait_for_vsync
 **********************************************************/
int fsl_dcu_wait_for_vsync(void)
{
	unsigned long flags;
	int cond_idx;

	/* Try to get an entry in the waiting PID list */
	spin_lock_irqsave(&dcu_event_queue.lock, flags);

	for (cond_idx = 0; cond_idx < DCU_LAYERS_NUM_MAX; ++cond_idx) {
		if (event_condition_list[cond_idx] == EVENT_STATUS_CLEAR) {
			event_condition_list[cond_idx] = DCU_STATUS_WAITING;
			break;
		}
	}

	spin_unlock_irqrestore(&dcu_event_queue.lock, flags);

	/* If no index was available, return unsuccessfully */
	if (cond_idx >= DCU_LAYERS_NUM_MAX)
		return -EBUSY;

	/* Wait until the DCU event occurs */
	wait_event(dcu_event_queue,
		event_condition_list[cond_idx] == DCU_STATUS_WAITED);

	/* Release the entry in the waiting PID list */
	spin_lock_irqsave(&dcu_event_queue.lock, flags);
	event_condition_list[cond_idx] = EVENT_STATUS_CLEAR;
	spin_unlock_irqrestore(&dcu_event_queue.lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_wait_for_vsync);

/**********************************************************
 * FUNCTION: fsl_dcu_event_VSYNC
 **********************************************************/
void fsl_dcu_event_VSYNC(void)
{
	unsigned long flags;
	int i;

	/* Notify all currently-waiting processes to resume */
	spin_lock_irqsave(&dcu_event_queue.lock, flags);

	for (i = 0; i < DCU_LAYERS_NUM_MAX; ++i) {
		if (event_condition_list[i] == DCU_STATUS_WAITING)
			event_condition_list[i] = DCU_STATUS_WAITED;
	}

	spin_unlock_irqrestore(&dcu_event_queue.lock, flags);

	wake_up_all(&dcu_event_queue);
}

/**********************************************************
 * FUNCTION: fsl_dcu_event_VBLANK
 **********************************************************/
void fsl_dcu_event_VBLANK(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dcu_clut_lock, flags);

	if (dcu_clut_size > 0) {
		DCU_CLUTLoad(0, dcu_clut_layer, dcu_clut_offset,
			dcu_clut_size, dcu_clut);
		dcu_clut_size = 0;
	}

	spin_unlock_irqrestore(&dcu_clut_lock, flags);
}

/**********************************************************
 * FUNCTION: fsl_dcu_display
 * DCU configure display
 **********************************************************/
void fsl_dcu_display(DCU_DISPLAY_TYPE display_type,
		Dcu_LCD_Para_t *dcu_lcd_timings)
{
	Dcu_Colour_t bkgr_color;

	__TRACE__;

	/* DCU set configuration, LVDS has fixed div according to RM - TODO */
	DCU_Init(0, 150, dcu_lcd_timings, DCU_FREQDIV_NORMAL);

	/* Initialize DCU background color */
	bkgr_color.Red_Value	= 0x0;
	bkgr_color.Green_Value	= 0x0;
	bkgr_color.Blue_Value	= 0x0;
	DCU_BGNDColorSet(0, &bkgr_color);

	/* Register and enable the callbacks for VSYNC and VBLANK */
	DCU_RegisterCallbackVSYNC(0, fsl_dcu_event_VSYNC);
	DCU_RegisterCallbackVBLANK(0, fsl_dcu_event_VBLANK);

	DCU_EnableDisplayTimingIrq(0,
		DCU_INT_VSYNC_MASK | DCU_INT_VS_BLANK_MASK);
}

/**********************************************************
 * FUNCTION: print_display_modes
 * DCU print display modes
 **********************************************************/
void print_display_modes(struct fb_monspecs monspecs)
{
	int i;

	for (i = 0; i < monspecs.modedb_len; i++) {
		dev_info(&dcu_pdev->dev, "hdmi %d %d %d %d %d %d %d %d %d\n",
			monspecs.modedb[i].pixclock * 1000,
			monspecs.modedb[i].xres,
			monspecs.modedb[i].yres,
			monspecs.modedb[i].upper_margin,
			monspecs.modedb[i].lower_margin,
			monspecs.modedb[i].left_margin,
			monspecs.modedb[i].right_margin,
			monspecs.modedb[i].hsync_len,
			monspecs.modedb[i].vsync_len
		);
	}
}

/*******************************************************************
 * FUNCTION: fsl_dcu_register_timings_listener
 * Register a FB object for notifications of display timings changes
 *******************************************************************/
int fsl_dcu_register_timings_listener(struct fb_info *info)
{
	if (timings_listener_list_size >= DCU_LAYERS_NUM_MAX)
		return -EINVAL;

	timings_listener_list[timings_listener_list_size++] = info;

	return 0;
}
EXPORT_SYMBOL_GPL(fsl_dcu_register_timings_listener);

/*************************************************************************
 * FUNCTION: update_display_timings
 * Update the display timings stored by a FB object
 ************************************************************************/
void update_display_timings(struct IOCTL_DISPLAY_CFG *ioctl_display_cfg,
	struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	var->pixclock = ioctl_display_cfg->clock_freq / 1000;
	var->left_margin = ioctl_display_cfg->hback_porch;
	var->right_margin = ioctl_display_cfg->hfront_porch;
	var->upper_margin = ioctl_display_cfg->vback_porch;
	var->lower_margin = ioctl_display_cfg->vfront_porch;
	var->hsync_len = ioctl_display_cfg->hsync_len;
	var->vsync_len = ioctl_display_cfg->vsync_len;
}

/**********************************************************
 * FUNCTION: fsl_dcu_compare_display_timings
 **********************************************************/
int fsl_dcu_display_configs_match(
	struct IOCTL_DISPLAY_CFG *display_cfg1,
	struct IOCTL_DISPLAY_CFG *display_cfg2)
{
	struct IOCTL_DISPLAY_CFG copy1 = *display_cfg1;
	struct IOCTL_DISPLAY_CFG copy2 = *display_cfg2;

	/* we only care about timings and the display type */
	copy1.hactive = copy2.hactive = 0;
	copy1.vactive = copy2.vactive = 0;

	return (memcmp(&copy1, &copy2, sizeof(struct IOCTL_DISPLAY_CFG)) == 0);
}

/**********************************************************
 * FUNCTION: fsl_dcu_configure_display
 * Set the parameters of a DCU-managed display
 **********************************************************/
void fsl_dcu_configure_display(struct IOCTL_DISPLAY_CFG *display_cfg)
{
	Dcu_LCD_Para_t dcu_lcd_timings;
	struct fb_monspecs monspecs;
	int i;

	/* Don't configure the display to the same parameters */
	if (fsl_dcu_display_configs_match(display_cfg, &current_display_cfg))
		return;

	/* Update the current display parameters */
	memcpy(&current_display_cfg, display_cfg,
		sizeof(struct IOCTL_DISPLAY_CFG));

	/* Set specific parameters for the DCU to drive the display */
	dcu_lcd_timings.mDeltaX = display_cfg->hactive;
	dcu_lcd_timings.mDeltaY = display_cfg->vactive;
	dcu_lcd_timings.mHorzBP = display_cfg->hback_porch;
	dcu_lcd_timings.mHorzFP = display_cfg->hfront_porch;
	dcu_lcd_timings.mHorzPW = display_cfg->hsync_len;
	dcu_lcd_timings.mVertBP = display_cfg->vback_porch;
	dcu_lcd_timings.mVertFP = display_cfg->vfront_porch;
	dcu_lcd_timings.mVertPW = display_cfg->vsync_len;
	dcu_lcd_timings.mSyncPol = 3;
	dcu_lcd_timings.mVertFq = 60;
	dcu_lcd_timings.mDivFactor = dcu_clk_val / display_cfg->clock_freq;

	if (((display_cfg->hsync_len + display_cfg->vsync_len) == 0) &&
		(display_cfg->disp_type == IOCTL_DISPLAY_HDMI)) {
		/* query HDMI IP for monitor specs through DDC/EDID */
		/* FIXME: avoid getting EDID info through HDMI direct call*/
		monspecs = sii902x_get_monspecs();

		/* search mode and set */
		for (i = 0; i < monspecs.modedb_len; i++) {
			struct fb_videomode *mode = &monspecs.modedb[i];

			if ((mode->xres == dcu_lcd_timings.mDeltaX) &&
			    (mode->yres == dcu_lcd_timings.mDeltaY)) {
				dcu_lcd_timings.mHorzBP  = mode->left_margin;
				dcu_lcd_timings.mHorzFP  = mode->right_margin;
				dcu_lcd_timings.mHorzPW  = mode->hsync_len;
				dcu_lcd_timings.mVertBP  = mode->upper_margin;
				dcu_lcd_timings.mVertFP  = mode->lower_margin;
				dcu_lcd_timings.mVertPW  = mode->vsync_len;
				dcu_lcd_timings.mSyncPol = 3;
				dcu_lcd_timings.mVertFq  = mode->refresh;
				break;
			}
		}

		if ((dcu_lcd_timings.mHorzPW + dcu_lcd_timings.mVertPW) == 0)
			dev_warn(&dcu_pdev->dev,
				"DCU: Requested resolution not in EDID\n");
	}

	/* set display configuration */
	fsl_dcu_display(display_cfg->disp_type, &dcu_lcd_timings);

	/* notify all listeners of changes in display timings */
	for (i = 0; i < timings_listener_list_size; ++i)
		update_display_timings(display_cfg,
			timings_listener_list[i]);
}
EXPORT_SYMBOL_GPL(fsl_dcu_configure_display);

/**********************************************************
 * FUNCTION: device_ioctl
 * DCU Linux IOCTL operations
 **********************************************************/
long device_ioctl(struct file *filp,
		unsigned int ioctl_cmd,
		unsigned long arg)
{
	int ret;
	struct fb_monspecs monspecs;

	struct IOCTL_LAYER_POS layer_pos;
	struct IOCTL_LAYER_ALFA_VAL layer_alpha_val;
	struct IOCTL_LAYER_ALFA_KEY layer_alpha_key;
	struct IOCTL_LAYER_CHROMA layer_chroma;
	struct IOCTL_DISPLAY_CFG display_cfg;

	__TRACE__;

	switch (ioctl_cmd) {
	case IOCTL_GET_LAYER_POS:
	{
		/* copy from user space */
		ret = copy_from_user(&layer_pos,
			(struct IOCTL_LAYER_POS *)arg,
				sizeof(layer_pos));

		DCU_GetLayerPosition(0, layer_pos.id,
				(Dcu_Position_t *)&layer_pos.pos);

		/* copy back to user space */
		ret = copy_to_user((struct IOCTL_LAYER_POS *)arg,
				&layer_pos, sizeof(layer_pos));
	}
	break;

	case IOCTL_SET_LAYER_POS:
	{
		/* copy from user space */
		ret = copy_from_user(&layer_pos,
			(struct IOCTL_LAYER_POS *)arg,
				sizeof(layer_pos));

		DCU_SetLayerPosition(0, layer_pos.id,
				(Dcu_Position_t *)&layer_pos.pos);
	}
	break;

	case IOCTL_GET_LAYER_ALPHA_VAL:
	{
		/* copy from user space */
		ret = copy_from_user(&layer_alpha_val,
			(struct IOCTL_LAYER_ALFA_VAL *)arg,
				sizeof(layer_alpha_val));

		DCU_GetLayerAlphaVal(0, layer_alpha_val.id,
				&layer_alpha_val.val);

		/* copy back to user space */
		ret = copy_to_user((struct IOCTL_LAYER_ALFA_VAL *)arg,
			&layer_alpha_val, sizeof(layer_alpha_val));
	}
	break;

	case IOCTL_SET_LAYER_ALPHA_VAL:
	{
		/* copy from user space */
		ret = copy_from_user(&layer_alpha_val,
			(struct IOCTL_LAYER_ALFA_VAL *)arg,
				sizeof(layer_alpha_val));

		DCU_SetLayerAlphaVal(0, layer_alpha_val.id,
				layer_alpha_val.val);
	}
	break;

	case IOCTL_GET_LAYER_ALPHA_MODE:
	{
		/* copy from user space */
		ret = copy_from_user(&layer_alpha_key,
			(struct DCU_IOCTL_LAYER_ALFA_KEY *)arg,
				sizeof(layer_alpha_key));

		DCU_GetLayerAlphaMode(0, layer_alpha_key.id,
				(Dcu_AlphaKey_t *)&layer_alpha_key.key);

		/* copy back to user space */
		ret = copy_to_user((struct IOCTL_LAYER_ALFA_VAL *)arg,
			&layer_alpha_key, sizeof(layer_alpha_key));
	}
	break;

	case IOCTL_SET_LAYER_ALPHA_MODE:
	{
		/* copy from user space */
		ret = copy_from_user(&layer_alpha_key,
			(struct DCU_IOCTL_LAYER_ALFA_KEY *)arg,
				sizeof(layer_alpha_key));

		DCU_SetLayerAlphaMode(0, layer_alpha_key.id,
				layer_alpha_key.key);
	}
	break;

	case IOCTL_SET_LAYER_CHROMA_KEY:
	{
		/* copy from user space */
		ret = copy_from_user(&layer_chroma,
			(struct IOCTL_LAYER_CHROMA *)arg,
				sizeof(layer_chroma));

		DCU_SetLayerChroma(0, layer_chroma.id,
			(Dcu_Colour_t *)&layer_chroma.max,
			(Dcu_Colour_t *)&layer_chroma.min);

		if (layer_chroma.state == IOCTL_DCU_CHROMA_ON)
			DCU_LayerChromaEnable(0, layer_chroma.id);
		else
			DCU_LayerChromaDisable(0, layer_chroma.id);
	}
	break;

	case IOCTL_GET_LAYER_CHROMA_KEY:
	{
		DCU_GetLayerChromaMax(0, 0,
				(Dcu_Colour_t *)&layer_chroma.max);
		DCU_GetLayerChromaMin(0, 0,
				(Dcu_Colour_t *)&layer_chroma.min);

		/* copy to user space */
		ret = copy_to_user(&layer_chroma,
			(struct IOCTL_LAYER_CHROMA *)arg,
				sizeof(layer_chroma));
	}
	break;

	case IOCTL_PRINT_DISPLAY_INFO:
	{
		/* query HDMI IP for monitor specs through DDC/EDID */
		monspecs = sii902x_get_monspecs();

		/* output HDMI display modes */
		print_display_modes(monspecs);
	}
	break;

	case IOCTL_SET_DISPLAY_CFG:
	{
		/* copy from user space */
		ret = copy_from_user(&display_cfg,
			(struct IOCTL_DISPLAY_CFG *)arg,
			sizeof(display_cfg));

		fsl_dcu_configure_display(&display_cfg);
	}
	break;
	}
	return 0;
}

/**********************************************************
 * FUNCTION: fsl_dcu_open
 **********************************************************/
static int fsl_dcu_open(struct inode *inod, struct file *fil)
{
	__TRACE__;
	return 0;
}


static int fsl_dcu_close(struct inode *inode, struct file *filp)
{
	__TRACE__;
	return 0;
}

/**********************************************************
 * FUNCTION: fsl_dcu_read
 **********************************************************/
static ssize_t fsl_dcu_read(struct file *fil, char *buff,
		size_t len, loff_t *off)
{

	int max_bytes;
	int bytes_read = 0;

	__TRACE__;

	max_bytes = wb_len - *off;

	if (max_bytes > len)
		bytes_read = len;
	else
		bytes_read = max_bytes;

	bytes_read = bytes_read -
		     copy_to_user(buff, wb_vir_ptr + (*off), bytes_read);
	*off = *off + bytes_read;

	return bytes_read;
}

/**********************************************************
 * STRUCT operations
 **********************************************************/
const struct file_operations dcu_fops = {
	.owner			= THIS_MODULE,
	.read			= fsl_dcu_read,
	.open			= fsl_dcu_open,
	.release		= fsl_dcu_close,
	.unlocked_ioctl	= device_ioctl,	/*DCU IOCTL */
};

/**********************************************************
 * FUNCTION: fsl_dcu_create
 **********************************************************/
int fsl_dcu_dev_create(struct platform_device *pdev)
{
	int ret;

	dcu_pdev = pdev;
	__TRACE__;

	/* Alloc MAJOR number for the character device  */
	ret = alloc_chrdev_region(&dcu_devno, 0, 1, DEVICE_NAME);
	if (ret < 0) {
		dev_err(&dcu_pdev->dev,
			"DCU: alloc_chrdev_region error %d\n", ret);
		return ret;
	}

	dcu_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (!dcu_class) {
		dev_err(&dcu_pdev->dev,
			"DCU: class_create error\n");
		return -1;
	}

	if (!(device_create(dcu_class, NULL,
			dcu_devno, NULL, DEVICE_NAME))) {
		dev_err(&dcu_pdev->dev,
			"DCU: device_create error\n");
		return -1;
	}

	/* setup file operations */
	dcu_cdev = cdev_alloc();
	dcu_cdev->ops = &dcu_fops;

	/* add file operations */
	ret = cdev_add(dcu_cdev, dcu_devno, 1);
	if (ret < 0) {
		dev_err(&dcu_pdev->dev,
			"DCU: cdev_add error %d\n", ret);
		return ret;
	}

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_dcu_irq_handler_wrapper
 **********************************************************/
irqreturn_t fsl_dcu_irq_handler_wrapper(int irq, void *dev_id)
{
	DCU0_Timing_Isr();
	return IRQ_HANDLED;
}

/**********************************************************
 * FUNCTION: fsl_dcu_init_memory_pool
 **********************************************************/
void fsl_dcu_init_memory_pool(struct platform_device *pdev)
{
	struct device_node *dcu_mem_node;
	__be32 *dcu_mem_region;
	u64 dcu_mem_start, dcu_mem_len;
	int prop_len, i;

	/* get the memory region for DCU-managed surfaces */
	dcu_mem_node = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);

	if (!dcu_mem_node)
		goto dcu_use_default_pool;

	dcu_mem_region =
		(__be32 *)of_get_property(dcu_mem_node, "reg", &prop_len);

	if (!dcu_mem_region || (prop_len <= 0))
		goto dcu_use_default_pool;

	/* get number of 32-bit words per value */
	i = prop_len / sizeof(u64);

	/* get the memory region for DCU surface allocations */
	dcu_mem_start = of_read_number(&dcu_mem_region[0], i);
	dcu_mem_len = of_read_number(&dcu_mem_region[i], i);

	if (!devm_request_mem_region(&pdev->dev,
			dcu_mem_start, dcu_mem_len, dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "DCU: request memory region error\n");
		goto dcu_use_default_pool;
	}

	dma_release_declared_memory(&pdev->dev);
	if (dma_declare_coherent_memory(
			&pdev->dev, dcu_mem_start,
			dcu_mem_start, dcu_mem_len) == 0) {
		dev_err(&dcu_pdev->dev, "DCU: memory pool creation error\n");
		devm_release_mem_region(&pdev->dev, dcu_mem_start,
			dcu_mem_len);
		goto dcu_use_default_pool;
	}

	dev_info(&pdev->dev,
		"DCU: surface memory space is [0x%08llX, 0x%08llX].\n",
		dcu_mem_start, dcu_mem_start + dcu_mem_len);

	return;

dcu_use_default_pool:
	dev_info(&pdev->dev,
		"DCU: using default surface memory space.\n");
}

/**********************************************************
 * FUNCTION: fsl_dcu_probe
 **********************************************************/
int fsl_dcu_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret, irq_num, i;

	__TRACE__;

	dcu_pdev = pdev;
	ret = 0;

	memset(&current_display_cfg, 0, sizeof(current_display_cfg));
	memset(timings_listener_list, 0, sizeof(timings_listener_list));
	timings_listener_list_size = 0;

	/* initialize the DCU memory pool if configured in DTB */
	fsl_dcu_init_memory_pool(pdev);

	/* create device and register it in /dev through sysfs */
	fsl_dcu_dev_create(pdev);

	/* map register space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dcu_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "could not get memory IO resource\n");
		return -ENODEV;
	}

	dcu_fb_data = devm_kzalloc(&pdev->dev,
			sizeof(struct dcu_fb_data), GFP_KERNEL);
	dev_set_drvdata(&pdev->dev, dcu_fb_data);
	dcu_fb_data->dev = &pdev->dev;

	dcu_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dcu_reg_base)) {
		dcu_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "DCU: could not ioremap resource\n");
		return PTR_ERR(dcu_fb_data->reg_base);
	}
	dcu_fb_data->reg_base = dcu_reg_base;

	/* allocate memory for base reg */
	DCU_BASE_ADDRESS = kmalloc(sizeof(uint64_t) * DCU_DEV_COUNT,
							GFP_KERNEL);
	if (!DCU_BASE_ADDRESS) {
		dcu_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "DCU: could not allocate memory for reg_base\n");
		goto failed_alloc_base;
	}

	/* save DCU0 register map to global variable for DCU agnostic layer */
	DCU_BASE_ADDRESS[0] = (uint64_t)dcu_reg_base;

	/* enable clocks for DCU */
	dcu_clk = devm_clk_get(&pdev->dev, "dcu");
	if (IS_ERR(dcu_clk)) {
		dcu_init_status = DCU_INIT_ERR_CFG;
		ret = PTR_ERR(dcu_clk);
		dev_err(&pdev->dev, "DCU: could not get clock\n");
		goto failed_getclock;
	}
	clk_prepare_enable(dcu_clk);

	/* get DCU clock in Hz */
	dcu_clk_val = clk_get_rate(dcu_clk);
	dcu_fb_data->clk = dcu_clk;
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* initialize the global CLUT */
	spin_lock_init(&dcu_clut_lock);
	dcu_clut_size = 0;

	/* insert DCU interrupt handler */
	init_waitqueue_head(&dcu_event_queue);

	for (i = 0; i < DCU_LAYERS_NUM_MAX; ++i)
		event_condition_list[i] = EVENT_STATUS_CLEAR;

	irq_num = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq_num,
		fsl_dcu_irq_handler_wrapper, 0, "2d-ace", NULL);

	if (ret != 0) {
		dev_err(&pdev->dev,
			"DCU: could not register interrupt handler\n");
		return ret;
	}

	/* prebare write back */
#ifndef DCU_DISABLE_WRITEBACK
	fsl_dcu_wb_prepare(pdev);
	fsl_dcu_wb_enable();
#endif

	/* init has finalized */
	dcu_init_status = DCU_INIT_TRUE;

	__TRACE__;

	return 0;

failed_alloc_base:
failed_getclock:
	return ret;
}

/**********************************************************
 * FUNCTION: fsl_dcu_remove
 **********************************************************/
int fsl_dcu_remove(struct platform_device *pdev)
{
	__TRACE__;

	cdev_del(dcu_cdev);
	device_destroy(dcu_class, dcu_devno);
	class_destroy(dcu_class);
	unregister_chrdev_region(dcu_devno, 1);

	DCU_Disable(0);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

/**********************************************************
 * DCU Linux DTB query related operations
 **********************************************************/
static const struct of_device_id fsl_dcu_dt_ids[] = {
	{
		.compatible = "fsl,s32v234-dcu",
	},
	{}
};

static int fsl_dcu_runtime_suspend(struct device *dev)
{
	struct dcu_fb_data *dcufb = dev_get_drvdata(dev);

	clk_disable_unprepare(dcufb->clk);
	return 0;
}
static int fsl_dcu_runtime_resume(struct device *dev)
{
	struct dcu_fb_data *dcufb = dev_get_drvdata(dev);

	clk_prepare_enable(dcufb->clk);
	return 0;
}

static const struct dev_pm_ops fsl_dcu_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_dcu_runtime_suspend,
			fsl_dcu_runtime_resume, NULL)
};

static struct platform_driver fsl_dcu_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fsl_dcu_dt_ids,
		.pm = &fsl_dcu_pm_ops,
	},
	.probe = fsl_dcu_probe,
	.remove = fsl_dcu_remove,
};

module_platform_driver(fsl_dcu_driver);

MODULE_AUTHOR("Lupescu Grigore");
MODULE_DESCRIPTION("Freescale fsl-DCU driver");
MODULE_LICENSE("GPL");
