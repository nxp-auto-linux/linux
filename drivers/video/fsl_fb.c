/*
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
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
#include <linux/string.h>
#include <linux/ctype.h>

#include "fsl_dcu.h"
#include "fsl_dcu_linux.h"
#include "fsl_fb.h"

#define DRIVER_NAME	"fsl_fb"

#ifndef __linux__
	#error "Error! Not a Linux platform!".
#endif

/**********************************************************
 * Various support macros
 **********************************************************/

#define MFB_SET_ALPHA	_IOW('M', 0, __u8)
#define MFB_GET_ALPHA	_IOR('M', 0, __u8)
#define MFB_SET_LAYER	_IOW('M', 4, struct layer_display_offset)
#define MFB_GET_LAYER	_IOR('M', 4, struct layer_display_offset)

/**
 * iMX6 IPU tiled formats and IOCTL calls used by Vivante driver
 */
#define MXCFB_SET_PREFETCH	_IOW('F', 0x30, int)
#define MXCFB_GET_PREFETCH	_IOR('F', 0x31, int)

/* 32bit single buf 4x4 standard */
#define IPU_PIX_FMT_GPU32_ST     v4l2_fourcc('5', 'I', '4', 'S')

/**********************************************************
 * Macros for tracing
 **********************************************************/
/* #define __LOG_TRACE__ 1 */

#ifdef __LOG_TRACE__
#define __TRACE__ dev_info(fsl_dcu_get_dcufb()->dev, \
	"[fsl-FB] %s\n", __func__)
#define __MSG_TRACE__(string, args...) dev_info(fsl_dcu_get_dcufb()->dev, \
	"[fsl-FB] %s : %d : " string, __func__, __LINE__, ##args)
#else
	#define __TRACE__
	#define __MSG_TRACE__(string, args...)
#endif

struct layer_display_offset {
	int x_layer_d;
	int y_layer_d;
};

/*********************************************************************
 * The video format as read from the kernel command arguments at boot
 ********************************************************************/

struct fb_video_format {
	int use_hdmi;
	int fb_idx;
	int res_x;
	int res_y;
	int refresh_rate;
	int surf_format_idx;
};

struct fb_video_format video_format;

/**********************************************************
 * FUNCTION: fsl_fb_video_format_match
 **********************************************************/
int fsl_fb_video_format_match(struct fb_videomode *fb_mode,
	struct fb_video_format *fb_format)
{
	/* Check for corresponding HDMI / LVDS format */
	if ((fb_format->use_hdmi && !strstr(fb_mode->name, "HDMI"))
		|| (!fb_format->use_hdmi && !strstr(fb_mode->name, "LVDS")))
		return 0;

	if ((fb_mode->xres != fb_format->res_x) ||
		(fb_mode->yres != fb_format->res_y))
		return 0;

	return 1;
}

/**********************************************************
 * FUNCTION: fsl_fb_init_color_format
 **********************************************************/
void fsl_fb_init_color_format(struct fb_var_screeninfo *var,
	int color_fmt_idx)
{
	struct fb_bitfield *var_channels[] = {
		&var->red, &var->green, &var->blue, &var->transp};
	int i;

	for (i = 0; i < 4; ++i)
		memcpy(var_channels[i],
			&DCU_FB_COLOR_FORMATS[color_fmt_idx].channels[i],
			sizeof(struct fb_bitfield));

	var->bits_per_pixel =
		DCU_FB_COLOR_FORMATS[color_fmt_idx].bpp;
}

/**********************************************************
 * FUNCTION: fsl_fb_check_var
 **********************************************************/
static int fsl_fb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;
	int color_fmt_idx;

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
		/* Handle grayscale and FOURCC color formats */
		switch (var->grayscale) {
		case V4L2_PIX_FMT_UYVY:
			break;

		default:
			dev_err(dcufb->dev,
				"Unsupported FOURCC color format: %u\n",
				var->grayscale);
			return -EINVAL;
		}
	} else {
		/* Find the correct color format */
		color_fmt_idx = fsl_dcu_get_color_format_match(var);

		if (color_fmt_idx >= dcu_fb_color_format_count) {
			dev_info(dcufb->dev, "No valid color format found.\n");
			return -EINVAL;
		}

		/* Ensure color format description is updated */
		fsl_fb_init_color_format(var, color_fmt_idx);
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
	struct mfb_info *mfbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	struct IOCTL_DISPLAY_CFG ioctl_display_cfg;

	__TRACE__;

	if ((fix->capabilities & FB_CAP_FOURCC) &&
		(var->grayscale == V4L2_PIX_FMT_UYVY))
		var->bits_per_pixel = 16;

	var->nonstd = mfbi->tiled ? IPU_PIX_FMT_GPU32_ST : 0;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	len = info->var.yres_virtual * info->fix.line_length;

	if (len != info->fix.smem_len) {
		if (info->fix.smem_start)
			fsl_dcu_unmap_vram(info);

		__MSG_TRACE__("map_video_memory [xv=%d,yv=%d;bpp=%d]\n",
				var->xres_virtual, var->yres_virtual,
				var->bits_per_pixel);

		if (fsl_dcu_map_vram(info))
			return -ENOMEM;
	}

	/* Configure display properties */
	memset(&ioctl_display_cfg, 0, sizeof(ioctl_display_cfg));
	ioctl_display_cfg.disp_type = video_format.use_hdmi ?
		IOCTL_DISPLAY_HDMI : IOCTL_DISPLAY_LVDS;
	ioctl_display_cfg.clock_freq = var->pixclock * 1000;
	ioctl_display_cfg.hactive = var->xres;
	ioctl_display_cfg.vactive = var->yres;
	ioctl_display_cfg.hback_porch = var->left_margin;
	ioctl_display_cfg.hfront_porch = var->right_margin;
	ioctl_display_cfg.vback_porch = var->upper_margin;
	ioctl_display_cfg.vfront_porch = var->lower_margin;
	ioctl_display_cfg.hsync_len = var->hsync_len;
	ioctl_display_cfg.vsync_len = var->vsync_len;

	fsl_dcu_configure_display(&ioctl_display_cfg);
	fsl_dcu_config_layer(info);

	return 0;
}

static inline __u32 CNVT_TOHW(__u32 val, __u32 width)
{
	__TRACE__;
	return ((val<<width) + 0x7FFF - val) >> 16;
}

/**********************************************************
 * FUNCTION: fsl_fb_setcolreg
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
 * FUNCTION: fsl_fb_setcmap
 * INFO: set an entire color map
 **********************************************************/
int fsl_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;
	int ret = 0;

	__TRACE__;

	if (!cmap)
		return -EINVAL;

	ret = fb_alloc_cmap(&info->cmap, cmap->len, cmap->transp != NULL);

	if (ret == 0)
		ret = fb_copy_cmap(cmap, &info->cmap);

	/* CLUT layout will be 256 entries per layer for up to 8 layers */
	if (mfbi->index >= 8) {
		dev_err(dcufb->dev, "CLUT not available for layer %d.\n",
			mfbi->index);
		return -EINVAL;
	}

	if (ret == 0)
		ret = fsl_dcu_set_clut(info);

	return ret;
}

/**********************************************************
 * FUNCTION: fsl_fb_pan_display
 * INFO: select view/region to display
 **********************************************************/
static int fsl_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
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

	__MSG_TRACE__("layer(%d):xoff=%d;yoff=%d\n",
			((struct mfb_info *)info->par)->index,
			info->var.xoffset, info->var.yoffset);

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

	case FBIO_WAITFORVSYNC:
		return fsl_dcu_wait_for_vsync();

	case MXCFB_GET_PREFETCH:
	{
		if (copy_to_user(buf, &mfbi->tiled, sizeof(mfbi->tiled)))
			return -EFAULT;
		break;
	}
	case MXCFB_SET_PREFETCH:
	{
		int enable = 0;

		if (copy_from_user(&enable, buf, sizeof(enable)))
			return -EFAULT;

		/* enable tile */
		mfbi->tiled = enable;
		fsl_fb_set_par(info);
		break;
	}
	default:
		dev_err(dcufb->dev, "Unknown ioctl command (0x%08X).\n", cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_fb_open
 **********************************************************/
static int fsl_fb_open(struct fb_info *info, int user)
{
	struct mfb_info *mfbi = info->par;
	int ret = 0;

	__TRACE__;

	mfbi->index = info->node;
	fsl_fb_check_var(&info->var, info);
	fsl_fb_set_par(info);
	mfbi->count++;

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
	.fb_setcmap = fsl_fb_setcmap,
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
	struct dcu_fb_data *dcufb;
	struct device_node *np;
	struct fb_var_screeninfo *var;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct device_node *timing;
	struct display_timings *timings;
	char *name;
	int i, j, name_len, ret;

	__TRACE__;

	ret = 0;
	pdev = fsl_dcu_get_pdev();
	dcufb = fsl_dcu_get_dcufb();

	np = pdev->dev.of_node;
	var = &info->var;

	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		dev_err(dcufb->dev, "Failed to find display phandle.\n");
		return -ENOENT;
	}

	ret = of_property_read_u32(display_np, "bits-per-pixel",
				&var->bits_per_pixel);
	if (ret < 0) {
		dev_err(dcufb->dev, "Failed to get property 'bits-per-pixel'.\n");
		goto put_display_node;
	}

	timings = of_get_display_timings(display_np);
	if (!timings) {
		dev_err(dcufb->dev, "Failed to get display timings.\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	timings_np = of_find_node_by_name(display_np,
					"display-timings");
	if (!timings_np) {
		dev_err(dcufb->dev, "Failed to find display-timings node.\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	INIT_LIST_HEAD(&info->modelist);
	timing = of_get_next_child(timings_np, NULL);

	for (i = 0; i < of_get_child_count(timings_np); i++) {
		struct videomode vm;
		struct fb_videomode fb_vm;

		memset(&vm, 0, sizeof(vm));
		memset(&fb_vm, 0, sizeof(fb_vm));

		/* Initialize the FB videomode timings */
		ret = videomode_from_timings(timings, &vm, i);
		if (ret < 0)
			goto put_timings_node;

		ret = fb_videomode_from_videomode(&vm, &fb_vm);
		if (ret < 0)
			goto put_timings_node;

		/* We need the original pixclock */
		fb_vm.pixclock = vm.pixelclock / 1000;

		/* Set the FB videomode name, in uppercase */
		name_len = strlen(timing->name);
		name = kmalloc(name_len + 1, GFP_KERNEL);
		if (!name)
			goto put_timings_node;

		strcpy(name, timing->name);
		fb_vm.name = name;

		for (j = 0; j < name_len; ++j)
			name[j] = toupper(name[j]);

		fb_add_videomode(&fb_vm, &info->modelist);
		timing = of_get_next_child(timings_np, timing);
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
static int fsl_fb_install(struct fb_info *info,
	struct fb_video_format *video_format)
{
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;
	struct fb_modelist *modelist = NULL;
	struct list_head *pos;
	int ret, found = 0;

	__TRACE__;

	info->var.activate = FB_ACTIVATE_NOW;
	info->fbops = &fsl_dcu_ops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette = &mfbi->pseudo_palette;

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dcufb->dev, "Failed to register framebuffer device.\n");
		goto fb_install_failed;
	}

	fb_alloc_cmap(&info->cmap, 16, 0);

	ret = fsl_fb_init(info);
	if (ret)
		goto fb_install_failed;

	list_for_each(pos, &info->modelist) {
		modelist = list_entry(pos, struct fb_modelist, list);

		if (!fsl_fb_video_format_match(&modelist->mode, video_format))
			continue;

		found = 1;
		break;
	}

	if (!found) {
		dev_warn(dcufb->dev,
			"Invalid display mode for <fb %d> (using defaults).\n",
			mfbi->index);
		modelist = list_first_entry(&info->modelist,
			struct fb_modelist, list);
	}

	/* Set video mode and color format */
	fb_videomode_to_var(&info->var, &modelist->mode);
	fsl_fb_init_color_format(&info->var, video_format->surf_format_idx);
	fsl_fb_check_var(&info->var, info);

	return 0;

fb_install_failed:
	unregister_framebuffer(info);
	return ret;
}

/**********************************************************
 * FUNCTION: fsl_fb_uninstall
 **********************************************************/
static void fsl_fb_uninstall(struct fb_info *info)
{
	struct list_head *pos;
	struct fb_modelist *modelist;

	__TRACE__;

	if (!info)
		return;

	list_for_each(pos, &info->modelist) {
		modelist = list_entry(pos, struct fb_modelist, list);
		kfree(modelist->mode.name);
	}

	__MSG_TRACE__("unmap video memory");
	fsl_dcu_unmap_vram(info);

	fb_dealloc_cmap(&info->cmap);
	unregister_framebuffer(info);
}

/**********************************************************
 * INFO: DCU Linux driver configuration
 **********************************************************/
char *video;
core_param(video, video, charp, 0444);

static int r_init(void);
static void r_cleanup(void);

module_init(r_init);
module_exit(r_cleanup);

/**********************************************************
 * FUNCTION: fsl_fb_parse_video_format
 * INFO: Parse the video format as given at kernel boot
 **********************************************************/
int fsl_fb_parse_video_format(char *video_str,
	struct fb_video_format *video_format)
{
	const char *split = ",;|";
	char *video_copy, *token, *rest;
	int len = strlen(video_str);

	if (len == 0)
		return -EINVAL;

	video_copy = kmalloc(len + 1, GFP_KERNEL);
	if (!video_copy)
		return -ENOMEM;

	/* Work on a duplicate of the video argument */
	strcpy(video_copy, video_str);
	memset(video_format, 0, sizeof(struct fb_video_format));
	rest = video_copy;

	/* Parse the video mode (HDMI or LVDS) */
	token = strsep(&rest, split);
	if (strcasecmp(token, "HDMI") == 0)
		video_format->use_hdmi = 1;
	else
		if (strcasecmp(token, "LVDS") == 0)
			video_format->use_hdmi = 0;
		else
			return -EINVAL;

	/* Parse the FB device to use */
	token = strsep(&rest, split);
	if (sscanf(token, "fb%d", &video_format->fb_idx) < 1)
		return -EINVAL;

	if ((video_format->fb_idx < 0) ||
		(video_format->fb_idx >= fsl_dcu_num_layers()))
		return -EINVAL;

	/* Parse the resolution and refresh rate */
	token = strsep(&rest, split);
	if (sscanf(token, "%dx%d-%d", &video_format->res_x,
			&video_format->res_y, &video_format->refresh_rate) < 3)
		return -EINVAL;

	if ((video_format->res_x <= 0) || (video_format->res_y <= 0)
		|| (video_format->refresh_rate <= 0))
		return -EINVAL;

	/* Parse the color format */
	token = strsep(&rest, split);
	video_format->surf_format_idx = fsl_dcu_get_color_format_byname(token);

	if (video_format->surf_format_idx < 0)
		return -EINVAL;

	dev_info(fsl_dcu_get_dcufb()->dev,
		"Video mode: <%s> on </dev/fb%d>, <%d x %d @ %d Hz>, <%s>\n",
		video_format->use_hdmi ? "HDMI" : "LVDS", video_format->fb_idx,
		video_format->res_x, video_format->res_y,
		video_format->refresh_rate, token);

	kfree(video_copy);

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_fb_init_first_layer
 * INFO: initialize the first layer after kernel boot
 **********************************************************/
int fsl_fb_init_first_layer(struct fb_info *info)
{
	__u32 fb_activate_mask = info->var.activate;
	int ret;

	info->var.activate |= (FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW);
	ret = fb_set_var(info, &info->var);
	info->var.activate = fb_activate_mask;

	return ret;
}

/**********************************************************
 * FUNCTION: r_init
 **********************************************************/
static int r_init(void)
{
	struct mfb_info *mfbi;
	int dcu_num_layers = 0;
	struct dcu_fb_data *dcufb;
	int ret = 0, byte_len, i, j;

	__TRACE__;
	if (fsl_dcu_init_status() != 0)
		return fsl_dcu_init_status();

	/* Parse the video format as given in the kernel boot arguments */
	ret = fsl_fb_parse_video_format(video, &video_format);
	if (ret != 0) {
		/* On invalid / missing format use a default display mode */
		memset(&video_format, 0, sizeof(video_format));
	}

	dcufb = fsl_dcu_get_dcufb();

	dcu_num_layers = fsl_dcu_num_layers();
	if (dcu_num_layers < 1) {
		dev_err(dcufb->dev,
			"Invalid number of layers (%d).", dcu_num_layers);
		return -EINVAL;
	}

	/* allocate fsl_dcu_info layers info */
	dcufb->fsl_dcu_info = (struct fb_info **)
			kmalloc_array(dcu_num_layers, sizeof(struct fb_info *),
					GFP_KERNEL);

	/* if alloc failed, clean up */
	if (dcufb->fsl_dcu_info == NULL) {
		dev_err(dcufb->dev, "Could not allocate framebuffer list.\n");
		return -ENOMEM;
	}

	memset(dcufb->fsl_dcu_info, 0, byte_len);

	/* Allocate and initialize individual layers */
	for (i = 0; i < dcu_num_layers; i++) {
		dcufb->fsl_dcu_info[i] =
			framebuffer_alloc(sizeof(struct mfb_info), dcufb->dev);
		if (!dcufb->fsl_dcu_info[i]) {
			ret = -ENOMEM;
			dev_err(dcufb->dev,
				"Could not allocate framebuffer %d.\n", i);
			break;
		}

		dcufb->fsl_dcu_info[i]->fix.smem_start = 0;
		dcufb->fsl_dcu_info[i]->fix.capabilities |= FB_CAP_FOURCC;

		/* Set layer information */
		mfbi = dcufb->fsl_dcu_info[i]->par;
		mfbi->index = i;
		mfbi->alpha = 0xFF;
		mfbi->blend = 0;
		mfbi->count = 0;
		mfbi->x_layer_d = 0;
		mfbi->y_layer_d = 0;
		mfbi->parent = dcufb;

		ret = fsl_fb_install(dcufb->fsl_dcu_info[i], &video_format);
		if (ret) {
			dev_err(dcufb->dev,
				"Could not register framebuffer %d.\n", i);
			framebuffer_release(dcufb->fsl_dcu_info[i]);
			break;
		}

		fsl_dcu_register_timings_listener(dcufb->fsl_dcu_info[i]);
	}

	/* Initialize the first layer if everything is OK so far */
	if (i >= dcu_num_layers) {
		ret = fsl_fb_init_first_layer(
			dcufb->fsl_dcu_info[video_format.fb_idx]);

		if (!ret)
			return 0;
	}

	/* An error occurred, so all framebuffers get released */
	dev_err(dcufb->dev, "Rolling back framebuffer initialization.\n");

	for (j = 0; j < i; j++) {
		fsl_fb_uninstall(dcufb->fsl_dcu_info[j]);
		framebuffer_release(dcufb->fsl_dcu_info[j]);
	}
	kfree(dcufb->fsl_dcu_info);

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

	if (dcu_num_layers < 1) {
		dev_err(dcufb->dev,
			"invalid number of layers (%d)", dcu_num_layers);
		return;
	}

	pm_runtime_get_sync(&pdev->dev);
	clk_disable_unprepare(dcufb->clk);

	/* dynamic number of layers */
	for (i = 0; i < dcu_num_layers; i++) {
		fsl_fb_uninstall(dcufb->fsl_dcu_info[i]);
		framebuffer_release(dcufb->fsl_dcu_info[i]);
	}
}

/**********************************************************
 * INFO: Module info
 **********************************************************/
MODULE_AUTHOR("Lupescu Grigore, Trandafir Andrei");
MODULE_DESCRIPTION("Freescale FB driver");
MODULE_LICENSE("GPL");
