/*
 * Copyright 2012-2014 Freescale Semiconductor, Inc.
 *
 * Freescale fsl-FB device driver
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
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>

#include "fsl_dcu.h"
#include "fsl_fb.h"

#define DRIVER_NAME	"fsl_fb"

#ifndef __linux__
	#error "Error! Not a Linux platform!".
#endif

/**********************************************************
 * External routines defined in fsl-DCU driver
 **********************************************************/
extern int fsl_dcu_config_layer(struct fb_info *info);
extern int fsl_dcu_reset_layer(struct fb_info *info);
extern int fsl_dcu_map_vram(struct fb_info *info);
extern void fsl_dcu_unmap_vram(struct fb_info *info);
extern int fsl_dcu_set_layer(struct fb_info *info);
extern struct dcu_fb_data * fsl_dcu_get_dcufb(void);
extern struct platform_device* fsl_dcu_get_pdev(void);
extern int fsl_dcu_num_layers(void);
extern int fsl_dcu_init_status(void);

/**********************************************************
 * Various color formats
 **********************************************************/
#define BPP_16_RGB565		4
#define BPP_24_RGB888		5
#define BPP_32_ARGB8888		6

#define MFB_SET_ALPHA	_IOW('M', 0, __u8)
#define MFB_GET_ALPHA	_IOR('M', 0, __u8)
#define MFB_SET_LAYER	_IOW('M', 4, struct layer_display_offset)
#define MFB_GET_LAYER	_IOR('M', 4, struct layer_display_offset)

/**********************************************************
 * Macros for tracing
 **********************************************************/
/* #define __LOG_TRACE__ 1 */

#ifdef __LOG_TRACE__
#define __TRACE__ printk(KERN_INFO "[ fsl-FB ] %s\n", __func__);
#define __MSG_TRACE__(string, args...) printk(KERN_INFO "[ fsl-FB ] "\
		" %s : %d : " string, __FUNCTION__, __LINE__, ##args)
#else
	#define __TRACE__ ;
	#define __MSG_TRACE__(string, args...) ;
#endif

struct layer_display_offset {
	int x_layer_d;
	int y_layer_d;
};

/**********************************************************
 * FUNCTION: fsl_fb_check_var
 **********************************************************/
static int fsl_fb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;

	__TRACE__;

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if (var->xoffset + info->var.xres > info->var.xres_virtual)
		var->xoffset = info->var.xres_virtual - info->var.xres;

	if (var->yoffset + info->var.yres > info->var.yres_virtual)
		var->yoffset = info->var.yres_virtual - info->var.yres;

	/* Check FOURCC */
	if ((info->fix.capabilities & FB_CAP_FOURCC) &&
		(var->grayscale > 1)) {
		switch (var->grayscale) {
		case V4L2_PIX_FMT_UYVY:
			dev_info(dcufb->dev, "switch to UYVY format\n");
			break;

		default:
			dev_err(dcufb->dev,
				"unsupported FOURCC color format: %u\n",
				var->grayscale);
			return -EINVAL;
		}
	} else {
		switch (var->bits_per_pixel) {
		case 16:
			var->red.length = 5;
			var->red.offset = 11;
			var->red.msb_right = 0;

			var->green.length = 6;
			var->green.offset = 5;
			var->green.msb_right = 0;

			var->blue.length = 5;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
			break;

		case 24:
			var->red.length = 8;
			var->red.offset = 16;
			var->red.msb_right = 0;

			var->green.length = 8;
			var->green.offset = 8;
			var->green.msb_right = 0;

			var->blue.length = 8;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
			break;

		case 32:
			var->red.length = 8;
			var->red.offset = 16;
			var->red.msb_right = 0;

			var->green.length = 8;
			var->green.offset = 8;
			var->green.msb_right = 0;

			var->blue.length = 8;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 8;
			var->transp.offset = 24;
			var->transp.msb_right = 0;
			break;

		default:
			dev_err(dcufb->dev,
				"unsupported color depth: %u\n",
				var->bits_per_pixel);
			return -EINVAL;
		}
	}

	return 0;
}


/**********************************************************
 * FUNCTION: fsl_fb_check_var
 * INFO: map VRAM
 **********************************************************/
static int fsl_fb_set_par(struct fb_info *info)
{
	unsigned long len;
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;

	__TRACE__;

	if ((fix->capabilities & FB_CAP_FOURCC) &&
		(var->grayscale == V4L2_PIX_FMT_UYVY))
		var->bits_per_pixel = 16;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	len = info->var.yres_virtual * info->fix.line_length;

	if (len != info->fix.smem_len) {
		if (info->fix.smem_start){
			__MSG_TRACE__("unmap_video_memory\n");
			fsl_dcu_unmap_vram(info);
		}

		__MSG_TRACE__("map_video_memory\n");
		if (fsl_dcu_map_vram(info)) {
			dev_err(dcufb->dev, "unable to allocate fb memory\n");
			return -ENOMEM;
		}
	}

	fsl_dcu_config_layer(info);
	return 0;
}

static inline __u32 CNVT_TOHW(__u32 val, __u32 width)
{
	__TRACE__;
	return ((val<<width) + 0x7FFF - val) >> 16;
}

/**********************************************************
 * FUNCTION: fsl_fb_check_var
 * INFO: map VRAM
 **********************************************************/
static int fsl_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			unsigned blue, unsigned transp, struct fb_info *info)
{
	unsigned int val;
	int ret = -EINVAL;

	__TRACE__;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour. We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			val = (red << info->var.red.offset) |
				(green << info->var.green.offset) |
				(blue << info->var.blue.offset) |
				(transp << info->var.transp.offset);

			pal[regno] = val;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

/**********************************************************
 * FUNCTION: fsl_fb_pan_display
 * INFO: select view/region to display
 **********************************************************/
static int fsl_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	__TRACE__;
	if ((info->var.xoffset == var->xoffset) &&
		(info->var.yoffset == var->yoffset))
		return 0;

	if ((var->xoffset + info->var.xres) > info->var.xres_virtual
		|| (var->yoffset + info->var.yres) > info->var.yres_virtual)
		return -EINVAL;

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	fsl_dcu_set_layer(info);
	__MSG_TRACE__("fsl_dcu_set_layer\n");

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_fb_blank
 **********************************************************/
static int fsl_fb_blank(int blank_mode, struct fb_info *info)
{
	__TRACE__;

	switch (blank_mode) {
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		fsl_dcu_reset_layer(info);
		break;
	case FB_BLANK_UNBLANK:
		fsl_dcu_config_layer(info);
		break;
	}

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_fb_ioctl
 * INFO: user-kernel basic communication
 **********************************************************/
static int fsl_fb_ioctl(struct fb_info *info, unsigned int cmd,
		unsigned long arg)
{
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;
	struct layer_display_offset layer_d;
	void __user *buf = (void __user *)arg;
	unsigned char alpha;

	__TRACE__;

	switch (cmd) {
	case MFB_SET_LAYER:
		if (copy_from_user(&layer_d, buf, sizeof(layer_d)))
			return -EFAULT;
		mfbi->x_layer_d = layer_d.x_layer_d;
		mfbi->y_layer_d = layer_d.y_layer_d;
		fsl_fb_set_par(info);
		break;
	case MFB_GET_LAYER:
		layer_d.x_layer_d = mfbi->x_layer_d;
		layer_d.y_layer_d = mfbi->y_layer_d;
		if (copy_to_user(buf, &layer_d, sizeof(layer_d)))
			return -EFAULT;
		break;
	case MFB_GET_ALPHA:
		alpha = mfbi->alpha;
		if (copy_to_user(buf, &alpha, sizeof(alpha)))
			return -EFAULT;
		break;
	case MFB_SET_ALPHA:
		if (copy_from_user(&alpha, buf, sizeof(alpha)))
			return -EFAULT;
		mfbi->blend = 1;
		mfbi->alpha = alpha;
		fsl_fb_set_par(info);
		break;
	default:
		dev_err(dcufb->dev, "unknown ioctl command (0x%08X)\n", cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_fb_ioctl
 * INFO: user-kernel communication
 **********************************************************/
static int fsl_fb_open(struct fb_info *info, int user)
{
	struct mfb_info *mfbi = info->par;
	int ret = 0;

	__TRACE__;

	mfbi->index = info->node;
	fsl_fb_set_par(info);
	mfbi->count++;

	if (mfbi->count == 1) {
		fsl_fb_check_var(&info->var, info);
		ret = fsl_fb_set_par(info);
	}
	return ret;
}

/**********************************************************
 * FUNCTION: fsl_fb_release
 **********************************************************/
static int fsl_fb_release(struct fb_info *info, int user)
{
	struct mfb_info *mfbi = info->par;

	__TRACE__;
	mfbi->count--;

	return 0;
}

/**********************************************************
 * Structure Linux FB module
 **********************************************************/
static struct fb_ops fsl_dcu_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = fsl_fb_check_var,
	.fb_set_par = fsl_fb_set_par,
	.fb_setcolreg = fsl_fb_setcolreg,
	.fb_blank = fsl_fb_blank,
	.fb_pan_display = fsl_fb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = fsl_fb_ioctl,
	.fb_open = fsl_fb_open,
	.fb_release = fsl_fb_release,
};

/**********************************************************
 * FUNCTION: fsl_fb_init
 **********************************************************/
static int fsl_fb_init(struct fb_info *info)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct fb_var_screeninfo *var;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct display_timings *timings;
	int ret;
	int i;

	__TRACE__;

	ret = 0;
	pdev = fsl_dcu_get_pdev();
	np = pdev->dev.of_node;
	var = &info->var;

	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		dev_err(&pdev->dev, "failed to find display phandle\n");
		return -ENOENT;
	}

	ret = of_property_read_u32(display_np, "bits-per-pixel",
				&var->bits_per_pixel);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get property bits-per-pixel\n");
		goto put_display_node;
	}

	timings = of_get_display_timings(display_np);
	if (!timings) {
		dev_err(&pdev->dev, "failed to get display timings\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	timings_np = of_find_node_by_name(display_np,
					"display-timings");
	if (!timings_np) {
		dev_err(&pdev->dev, "failed to find display-timings node\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	for (i = 0; i < of_get_child_count(timings_np); i++) {
		struct videomode vm;
		struct fb_videomode fb_vm;

		ret = videomode_from_timings(timings, &vm, i);
		if (ret < 0)
			goto put_timings_node;

		ret = fb_videomode_from_videomode(&vm, &fb_vm);
		if (ret < 0)
			goto put_timings_node;

		fb_add_videomode(&fb_vm, &info->modelist);
	}

	return 0;
put_timings_node:
	of_node_put(timings_np);
put_display_node:
	of_node_put(display_np);
	return ret;
}

/**********************************************************
 * FUNCTION: fsl_fb_install
 **********************************************************/
static int fsl_fb_install(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;
	struct fb_modelist *modelist;
	int ret;

	__TRACE__;

	info->var.activate = FB_ACTIVATE_NOW;
	info->fbops = &fsl_dcu_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette = &mfbi->pseudo_palette;

	fb_alloc_cmap(&info->cmap, 16, 0);
	INIT_LIST_HEAD(&info->modelist);

	ret = fsl_fb_init(info);
	if (ret)
		return ret;

	modelist = list_first_entry(&info->modelist,
			struct fb_modelist, list);
	fb_videomode_to_var(&info->var, &modelist->mode);

	fsl_fb_check_var(&info->var, info);
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dcufb->dev, "failed to register framebuffer device\n");
		return ret;
	}
	return 0;
}

/**********************************************************
 * FUNCTION: fsl_fb_uninstall
 **********************************************************/
static void fsl_fb_uninstall(struct fb_info *info)
{
	__TRACE__;
	unregister_framebuffer(info);

	__MSG_TRACE__("unmap video memory");
	fsl_dcu_unmap_vram(info);

	if (&info->cmap)
		fb_dealloc_cmap(&info->cmap);
}

/**********************************************************
 * INFO: DCU Linux driver configuration
 **********************************************************/
static int r_init(void);
static void r_cleanup(void);

module_init(r_init);
module_exit(r_cleanup);

/**********************************************************
 * FUNCTION: r_init
 **********************************************************/
static int r_init(void)
{
	struct mfb_info *mfbi;
	int dcu_num_layers = 0;
	struct platform_device *pdev;
	struct dcu_fb_data *dcufb;
	int ret = 0;
	int i;

	__TRACE__;
	if(fsl_dcu_init_status() != 0)
		return fsl_dcu_init_status();

	pdev = fsl_dcu_get_pdev();
	dcufb = fsl_dcu_get_dcufb();

	dcu_num_layers = fsl_dcu_num_layers();
	if(dcu_num_layers < 1){
		dev_err(&pdev->dev, "invalid number layers %d", dcu_num_layers);
		goto failed_invalid_layers;
	}

	/* probe and init DCU */
	dcufb = fsl_dcu_get_dcufb();

	/* allocate fsl_dcu_info layers info */
	dcufb->fsl_dcu_info = (struct fb_info**)
			kmalloc(sizeof(struct fb_info*) * dcu_num_layers,
					GFP_KERNEL);

	/* if alloc failed, clean up */
	if(dcufb->fsl_dcu_info == NULL){
		dev_err(&pdev->dev, "could not alloc dcufb->fsl_dcu_info");
		goto fatal_alloc_fb_info;
	}

	/* dynamic number of layers */
	for (i = 0; i < dcu_num_layers; i++) {
		dcufb->fsl_dcu_info[i] =
			framebuffer_alloc(sizeof(struct mfb_info), &pdev->dev);
		if (!dcufb->fsl_dcu_info[i]) {
			ret = -ENOMEM;
			goto failed_alloc_framebuffer;
		}

		dcufb->fsl_dcu_info[i]->fix.smem_start = 0;
		dcufb->fsl_dcu_info[i]->fix.capabilities |= FB_CAP_FOURCC;

		mfbi = dcufb->fsl_dcu_info[i]->par;

		/* set layer info */
		mfbi->index = i;
		mfbi->alpha = 0xFF;
		mfbi->blend = 0;
		mfbi->count = 0;
		mfbi->x_layer_d = 0;
		mfbi->y_layer_d = 0;
		mfbi->parent = dcufb;

		ret = fsl_fb_install(dcufb->fsl_dcu_info[i]);
		if (ret) {
			dev_err(&pdev->dev,
				"could not register framebuffer %d\n", i);
			goto failed_register_framebuffer;
		}
	}
	return 0;

failed_register_framebuffer:
	for (i = 0; i < dcu_num_layers; i++) {
		if (dcufb->fsl_dcu_info[i])
			framebuffer_release(dcufb->fsl_dcu_info[i]);
	}

failed_alloc_framebuffer:
fatal_alloc_fb_info:
failed_invalid_layers:
	return ret;
}

/**********************************************************
 * FUNCTION: r_cleanup
 **********************************************************/
static void r_cleanup(void)
{
	struct platform_device *pdev;
	struct dcu_fb_data *dcufb;
	int dcu_num_layers = 0;
	int i;

	__TRACE__;

	pdev = fsl_dcu_get_pdev();
	dcufb = fsl_dcu_get_dcufb();
	dcu_num_layers = fsl_dcu_num_layers();

	if(dcu_num_layers < 1){
		dev_err(&pdev->dev, "invalid number layers %d",
				dcu_num_layers);
		return;
	}

	pm_runtime_get_sync(&pdev->dev);
	clk_disable_unprepare(dcufb->clk);

	/* dynamic number of layers */
	for (i = 0; i < dcu_num_layers; i++) {
		fsl_fb_uninstall(dcufb->fsl_dcu_info[i]);
		framebuffer_release(dcufb->fsl_dcu_info[i]);
	}

	return;
}

/**********************************************************
 * INFO: Module info
 **********************************************************/
MODULE_AUTHOR("Lupescu Grigore");
MODULE_DESCRIPTION("Freescale fsl-FB driver");
MODULE_LICENSE("GPL");

