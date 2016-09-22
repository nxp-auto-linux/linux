/*
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
 *
 * Freescale DCU driver header for Linux
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef FSL_DCU_LINUX_H_
#define FSL_DCU_LINUX_H_

#include <uapi/video/fsl_dcu_ioctl.h>

struct dcu_fb_color_format {
	char format_name[16];
	struct fb_bitfield channels[4];
	int bpp;
};

extern const struct dcu_fb_color_format DCU_FB_COLOR_FORMATS[];
extern const int dcu_fb_color_format_count;

int fsl_dcu_config_layer(struct fb_info *info);
void fsl_dcu_configure_display(struct IOCTL_DISPLAY_CFG *display_cfg);
int fsl_dcu_reset_layer(struct fb_info *info);
int fsl_dcu_map_vram(struct fb_info *info);
void fsl_dcu_unmap_vram(struct fb_info *info);
int fsl_dcu_set_layer(struct fb_info *info);
int fsl_dcu_register_timings_listener(struct fb_info *info);
struct dcu_fb_data *fsl_dcu_get_dcufb(void);
struct platform_device *fsl_dcu_get_pdev(void);
int fsl_dcu_num_layers(void);
int fsl_dcu_init_status(void);
int fsl_dcu_wait_for_vsync(void);
int fsl_dcu_wait_for_vblank(void);
int fsl_dcu_get_color_format_match(const struct fb_var_screeninfo *var);
int fsl_dcu_get_color_format_byname(const char *format_name);
int fsl_dcu_set_clut(struct fb_info *info);
struct fb_monspecs sii902x_get_monspecs(void);

#endif /* FSL_DCU_LINUX_H_ */
