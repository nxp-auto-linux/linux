/*
 * Copyright 2012-2014 Freescale Semiconductor, Inc.
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
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
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
#include <asm/current.h>
#include <linux/uaccess.h>

#include "fsl_dcu_regs.h"
#include "fsl_dcu_regmacros.h"
#include "fsl_dcu.h"
#include "fsl_fb.h"

#include <uapi/video/fsl_dcu_ioctl.h>

/**********************************************************
 * DCU disable writeback, EXPERIMENTAL code
 **********************************************************/
#define DCU_DISABLE_WRITEBACK 1

/**********************************************************
 * DCU regbase, found in agnostic DCU driver layer
 **********************************************************/
/* number of DCU units */
#define DCU_DEV_COUNT	1

/* list of DCU base addr */
extern uint64_t* DCU_BASE_ADDR64;

/* wb_data */
long wb_len;
char* wb_vir_ptr;
unsigned long wb_phys_ptr;

/**********************************************************
 * DCU defines
 **********************************************************/
#define MAJOR_NUM 100
#define DEVICE_NAME "dcu0"

#define DRIVER_NAME		"fsl_dcu"
#define DCU_LAYER_NUM_MAX	8

#define TCON_CTRL		0x0
#define TCON_BYPASS_ENABLE	(1 << 29)
#define TCON_ENABLE		(1 << 31)

#define LDB_CTRL		0x0
#define LDB_DI0_VSYNC_LOW	(1 << 9)
#define LDB_DI0_VSYNC_HIGH	(0 << 9)
#define LDB_DWHC0		(1 << 5)
#define LDB_BITMAPCH0_SPWG	(0 << 6)
#define LDB_BITMAPCH0_JEIDA	(1 << 6)
#define LDB_CH0MOD_EN		(1 << 0)

/**********************************************************
 * External function definitions
 **********************************************************/
extern void __iomem *devm_ioremap(struct device *dev, resource_size_t offset,
		resource_size_t size);
extern void __iomem *devm_ioremap_nocache(struct device *dev,
		resource_size_t offset, resource_size_t size);

void fsl_dcu_wb_enable(void);
void fsl_dcu_wb_disable(void);
void fsl_dcu_wb_prepare(struct platform_device *pdev);

/**********************************************************
 * Macros for tracing
 **********************************************************/
//#define __LOG_TRACE__ 1

#ifdef __LOG_TRACE__
	#define __TRACE__ printk(KERN_INFO "[ fsl-DCU ] %s\n", __func__);
	#define __MSG_TRACE__(string, args...) printk(KERN_INFO "[ fsl-DCU ] "\
			" %s : %d : " string, __FUNCTION__, __LINE__, ##args)
#else
	#define __TRACE__ ;
	#define __MSG_TRACE__(string, args...) ;
	#define __HERE__ printk(KERN_INFO " HERE %s\n", __func__);
#endif

/**********************************************************
 * GLOBAL DCU configuration registers
 **********************************************************/
void __iomem * dcu_reg_base;
struct clk * dcu_clk;
struct dcu_fb_data * dcu_fb_data;
struct platform_device *dcu_pdev;
struct cdev *dcu_cdev;
struct class *dcu_class;
dev_t dcu_devno;
uint32_t dcu_clk_val;

#define DCU_INIT_TRUE		0
#define DCU_INIT_ERR_PROBE	1
#define DCU_INIT_ERR_CFG	2
int dcu_init_status = DCU_INIT_ERR_PROBE;

/**********************************************************
 * GLOBAL DCU display output type
 **********************************************************/

static bool g_enable_hdmi = false;

/**********************************************************
 * HDMI/LVDS selection bootargs
 **********************************************************/
static int __init enable_hdmi_display(char *str)
{
	__TRACE__;
	g_enable_hdmi = true;

	return 1;
}
__setup("hdmi", enable_hdmi_display);

/**********************************************************
 * External HDMI function definitions
 **********************************************************/
struct fb_monspecs sii902x_get_monspecs(void);

#ifndef CONFIG_FB_MXS_SII902X
/* FIXME: avoid direct communication with HDMI driver */
struct fb_monspecs sii902x_get_monspecs(void)
{
	struct fb_monspecs monspecs;
	printk(KERN_INFO "Module compiled without HDMI SII9022 support."
			"Recompile with FB_MXS_SII902X activated");
	memset(&monspecs, 0, sizeof(struct fb_monspecs));

	return monspecs;
}
#endif

/**********************************************************
 * FUNCTION: fsl_dcu_get_dcufb
 **********************************************************/
struct dcu_fb_data* fsl_dcu_get_dcufb(void)
{
	__TRACE__;
	return dcu_fb_data;
}
EXPORT_SYMBOL_GPL(fsl_dcu_get_dcufb);

/**********************************************************
 * FUNCTION: fsl_dcu_get_pdev
 **********************************************************/
struct platform_device* fsl_dcu_get_pdev(void)
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
	printk(KERN_INFO "-----------DCU REGS ---------- \n");
	printk(KERN_INFO "[REG : DCU_DCU_MODE]\t : %02x => %08x\n",
			0x10,
			readl(dcu_fb_data->reg_base + 0x10));
	printk(KERN_INFO "[REG : DCU_BGND  ]\t : %02x => %08x\n",
			0x14,
			readl(dcu_fb_data->reg_base + 0x14));
	printk(KERN_INFO "[REG : DCU_DISP_SIZE]\t : %02x => %08x\n",
			0x18,
			readl(dcu_fb_data->reg_base + 0x18));
	printk(KERN_INFO "[REG : DCU_HSYN_PARA]\t : %02x => %08x\n",
			0x1C,
			readl(dcu_fb_data->reg_base + 0x1C));
	printk(KERN_INFO "[REG : DCU_VSYN_PARA]\t : %02x => %08x\n",
			0x20,
			readl(dcu_fb_data->reg_base + 0x20));
	printk(KERN_INFO "[REG : DCU_SYN_POL]\t : %02x => %08x\n",
			0x24,
			readl(dcu_fb_data->reg_base + 0x24));
	printk(KERN_INFO "[REG : DCU_THRESHOLD]\t : %02x => %08x\n",
			0x28,
			readl(dcu_fb_data->reg_base + 0x28));
	printk(KERN_INFO "[REG : DCU_INT_STATUS]\t : %02x => %08x\n",
			0x2C,
			readl(dcu_fb_data->reg_base + 0x2C));
	printk(KERN_INFO "[REG : DCU_INT_MASK]\t : %02x => %08x\n",
			0x30,
			readl(dcu_fb_data->reg_base + 0x30));
	printk(KERN_INFO "[REG : DCU_DIV_RATIO]\t : %02x => %08x\n",
			0x54,
			readl(dcu_fb_data->reg_base + 0x54));
	printk(KERN_INFO "[REG : DCU_UPDATE_MODE]\t : %02x => %08x\n",
			0xCC,
			readl(dcu_fb_data->reg_base + 0xCC));
	printk(KERN_INFO "------------------------------- \n");
}
EXPORT_SYMBOL_GPL(fsl_dcu_registers);

/**********************************************************
 * FUNCTION: fsl_dcu_config_layer
 **********************************************************/
int fsl_dcu_config_layer(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = info->par;
	struct dcu_fb_data *dcufb = mfbi->parent;

	Dcu_Size_t	layer_size;
	Dcu_Position_t	layer_pos;
	Dcu_Colour_t	layer_chroma_max;
	Dcu_Colour_t	layer_chroma_min;

	__TRACE__;

	layer_size.mHeight = var->yres;
	layer_size.mWidth = var->xres;
	DCU_SetLayerSize(0, mfbi->index, &layer_size);

	layer_pos.mX = mfbi->x_layer_d;
	layer_pos.mY = mfbi->y_layer_d;

	DCU_SetLayerPosition(0, mfbi->index, &layer_pos);
	DCU_SetLayerBuffAddr(0, mfbi->index, info->fix.smem_start);

	switch (var->bits_per_pixel) {
	case 16:
		DCU_SetLayerBPP(0, mfbi->index, DCU_BPP_16);
		break;
	case 24:
		DCU_SetLayerBPP(0, mfbi->index, DCU_BPP_24);
		break;
	case 32:
		DCU_SetLayerBPP(0, mfbi->index, DCU_BPP_32);
		break;
	default:
		dev_err(dcufb->dev, "unsupported color depth: %u\n",
			var->bits_per_pixel);
		return -EINVAL;
	}

	DCU_SetLayerAlphaVal(0, mfbi->index, mfbi->alpha);
	DCU_SetLayerAlphaMode(0, mfbi->index, DCU_ALPHAKEY_WHOLEFRAME);
	DCU_LayerEnable(0, mfbi->index);

	layer_chroma_max.Blue_Value =  0xFF;
	layer_chroma_max.Red_Value =  0xFF;
	layer_chroma_max.Green_Value =  0xFF;

	layer_chroma_min.Blue_Value =  0x0;
	layer_chroma_min.Red_Value =  0x0;
	layer_chroma_min.Green_Value =  0x0;

	DCU_SetLayerChroma(0, mfbi->index,
			&layer_chroma_max, &layer_chroma_min);

	DCU_SetLayerForeground(0, mfbi->index, 0);
	DCU_SetLayerBackground(0, mfbi->index, 0);

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
		dev_err(dcufb->dev, "unable to allocate fb memory\n");
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
	int pixel_offset;
	unsigned long addr;

	__TRACE__;

	pixel_offset = (var->yoffset * var->xres_virtual) + var->xoffset;
	addr = info->fix.smem_start +
		(pixel_offset * (var->bits_per_pixel >> 3));

	DCU_SetLayerBuffAddr(0, mfbi->index, addr);

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

int fsl_setup_siul2(struct device_node *np, DCU_DISPLAY_TYPE display_type)
{
	struct device_node *siul2_np;
	struct platform_device *siul2_pdev;
	struct resource *siul2_res;
	void __iomem *siul2_reg;
	resource_size_t size;
	const char *name;
	int i;

	siul2_np = of_parse_phandle(np, "siul2-controller", 0);
	if (!siul2_np)
		return -EINVAL;

	siul2_pdev = of_find_device_by_node(siul2_np);
	if (!siul2_pdev)
		return -EINVAL;

	siul2_res = platform_get_resource(siul2_pdev, IORESOURCE_MEM, 0);
	if (!siul2_res || resource_type(siul2_res) != IORESOURCE_MEM) {
		dev_err(&siul2_pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	size = resource_size(siul2_res);
	name = siul2_res->name ?: dev_name(&siul2_pdev->dev);

	if (!devm_request_mem_region(&siul2_pdev->dev,
			siul2_res->start, size, name)){
		dev_err(&siul2_pdev->dev,
			"can't request region for resource %pR\n", siul2_res);
		return -EBUSY;
	}

	if (siul2_res->flags & IORESOURCE_CACHEABLE)
		siul2_reg = devm_ioremap(&siul2_pdev->dev,
				siul2_res->start, size);
	else
		siul2_reg = devm_ioremap_nocache(&siul2_pdev->dev,
				siul2_res->start, size);

	/* select IOMUX based on dcu display type */
	switch(display_type)
	{
		case DCU_DISPLAY_LVDS:
		{
			for(i=0x420; i<0x498; i+=0x4)
				writel(0x0020C101, siul2_reg + i);
				
			/*for(i=0x484; i<0x494; i+=0x4)
				writel(0x0020C101, siul2_reg + i); */
		}
		break;

		case DCU_DISPLAY_HDMI:
		{
			for(i=0x180; i<0x1F0; i+=0x4)
				writel(0x00102100, siul2_reg + i);
			writel(0x00000001, siul2_reg + 0x1F0);
		}
		break;

		case DCU_DISPLAY_TCON:
		{
		}
		break;
	}

	devm_release_mem_region(&siul2_pdev->dev, siul2_res->start, size);

	return 0;
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
	struct clk * ldb_clk;

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
		ldb_reg = devm_ioremap_nocache(&ldb_pdev->dev,
				ldb_res->start, size);

	if (!ldb_reg) {
		dev_err(&ldb_pdev->dev,
				"ioremap failed for resource %pR\n", ldb_res);
		devm_release_mem_region(&ldb_pdev->dev, ldb_res->start, size);
		ldb_reg = ERR_PTR(-ENOMEM);
	}

	/* LDB settings according to display type HDMI/LVDS */
	if(display_type == DCU_DISPLAY_LVDS)
		writel(LDB_DI0_VSYNC_LOW | LDB_DWHC0
		| LDB_CH0MOD_EN | LDB_BITMAPCH0_JEIDA,
			ldb_reg + LDB_CTRL);
	else if(display_type == DCU_DISPLAY_HDMI)
		writel(0, ldb_reg + LDB_CTRL);


	devm_release_mem_region(&ldb_pdev->dev, ldb_res->start, size);

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_turn_panel_on
 **********************************************************/
int fsl_turn_panel_on(struct device_node *np, DCU_DISPLAY_TYPE display_type)
{
	int err = 0;
	int panel_data_gpio;
	int panel_backlight_gpio;

	panel_data_gpio = of_get_named_gpio(np, "panel-data-gpio", 0);
	if (!gpio_is_valid(panel_data_gpio)){
		dev_err(&dcu_pdev->dev,
                        "failed to get panel data GPIO\n");
		return err;
	}

	err = gpio_request_one(panel_data_gpio, GPIOF_OUT_INIT_HIGH,
			"panel_data_gpio");

	if (err){
		dev_err(&dcu_pdev->dev,
			"failed to set panel data GPIO: %d\n", err);
		return err;
	}

	panel_backlight_gpio = of_get_named_gpio(np,
			"panel-backlight-gpio", 0);
	if (!gpio_is_valid(panel_backlight_gpio))
		dev_warn(&dcu_pdev->dev,
                        "failed to get panel backlight GPIO\n");

	err = gpio_request_one(panel_backlight_gpio, GPIOF_OUT_INIT_HIGH,
			"panel_backlight_gpio");
	if (err)
		dev_warn(&dcu_pdev->dev,
			"failed to set panel backlight GPIO: %d\n", err);

	return 0;
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
 * FUNCTION: fsl_dcu_display
 * DCU configure display
 **********************************************************/
int fsl_dcu_display(DCU_DISPLAY_TYPE display_type,
		Dcu_LCD_Para_t dcu_lcd_timings)
{
	int ret;
	
	__TRACE__;

	/* SIUL2 config, LCD LVDS */
	ret = fsl_setup_siul2(dcu_pdev->dev.of_node, display_type);
	if (ret)
		dev_err(&dcu_pdev->dev, "could not set IOMUX\n");

	/* DCU set configuration, LVDS has fixed div according to RM - TODO
	if(display_type == DCU_DISPLAY_LVDS)
		DCU_Init(0, (dcu_clk_val / 1000000), // clock in mhz 
				&dcu_lcd_timings, DCU_FREQDIV_LVDS);
	else
	*/
	DCU_Init(0, 150, &dcu_lcd_timings, DCU_FREQDIV_NORMAL);

	return ret;
}

/**********************************************************
 * FUNCTION: print_display_modes
 * DCU print display modes
 **********************************************************/
void print_display_modes(struct fb_monspecs monspecs)
{
	int i;
	for(i=0; i<monspecs.modedb_len; i++){
		printk(KERN_INFO "hdmi %d %d %d %d %d %d %d %d %d\n",
			monspecs.modedb[i].pixclock* 1000,
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

/**********************************************************
 * FUNCTION: device_ioctl
 * DCU Linux IOCTL operations
 **********************************************************/
long device_ioctl(struct file *filp,
		unsigned int ioctl_cmd,
		unsigned long arg)
{
	int ret, i;
	Dcu_LCD_Para_t dcu_lcd_timings;
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
				(struct IOCTL_LAYER_POS*)arg,
					sizeof(layer_pos));

			DCU_GetLayerPosition(0, layer_pos.id,
					(Dcu_Position_t*)&layer_pos.pos);

			/* copy back to user space */
			ret = copy_to_user((struct IOCTL_LAYER_POS*)arg,
					&layer_pos, sizeof(layer_pos));
		}
		break;

		case IOCTL_SET_LAYER_POS:
		{
			/* copy from user space */
			ret = copy_from_user(&layer_pos,
				(struct IOCTL_LAYER_POS*)arg,
					sizeof(layer_pos));

			DCU_SetLayerPosition(0, layer_pos.id,
					(Dcu_Position_t*)&layer_pos.pos);
		}
		break;

		case IOCTL_GET_LAYER_ALPHA_VAL:
		{
			/* copy from user space */
			ret = copy_from_user(&layer_alpha_val,
				(struct IOCTL_LAYER_ALFA_VAL*)arg,
					sizeof(layer_alpha_val));

			DCU_GetLayerAlphaVal(0, layer_alpha_val.id,
					&layer_alpha_val.val);

			/* copy back to user space */
			ret = copy_to_user((struct IOCTL_LAYER_ALFA_VAL*)arg,
				&layer_alpha_val, sizeof(layer_alpha_val));
		}
		break;

		case IOCTL_SET_LAYER_ALPHA_VAL:
		{
			/* copy from user space */
			ret = copy_from_user(&layer_alpha_val,
				(struct IOCTL_LAYER_ALFA_VAL*)arg,
					sizeof(layer_alpha_val));

			DCU_SetLayerAlphaVal(0, layer_alpha_val.id,
					layer_alpha_val.val);
		}
		break;

		case IOCTL_GET_LAYER_ALPHA_MODE:
		{
			/* copy from user space */
			ret = copy_from_user(&layer_alpha_key,
				(struct DCU_IOCTL_LAYER_ALFA_KEY*)arg,
					sizeof(layer_alpha_key));

			DCU_GetLayerAlphaMode(0, layer_alpha_key.id,
					(Dcu_AlphaKey_t*)&layer_alpha_key.key);

			/* copy back to user space */
			ret = copy_to_user((struct IOCTL_LAYER_ALFA_VAL*)arg,
				&layer_alpha_key, sizeof(layer_alpha_key));
		}
		break;

		case IOCTL_SET_LAYER_ALPHA_MODE:
		{
			/* copy from user space */
			ret = copy_from_user(&layer_alpha_key,
				(struct DCU_IOCTL_LAYER_ALFA_KEY*)arg,
					sizeof(layer_alpha_key));

			DCU_SetLayerAlphaMode(0, layer_alpha_key.id,
					layer_alpha_key.key);
		}
		break;

		case IOCTL_SET_LAYER_CHROMA_KEY:
		{
			/* copy from user space */
			ret = copy_from_user(&layer_chroma,
				(struct IOCTL_LAYER_CHROMA*)arg,
					sizeof(layer_chroma));

			DCU_SetLayerChroma(0, layer_chroma.id,
				(Dcu_Colour_t*)&layer_chroma.max,
				(Dcu_Colour_t*)&layer_chroma.min);

			if(layer_chroma.state == IOCTL_DCU_CHROMA_ON)
				DCU_LayerChromaEnable(0, layer_chroma.id);
			else
				DCU_LayerChromaDisable(0, layer_chroma.id);
		}
		break;

		case IOCTL_GET_LAYER_CHROMA_KEY:
		{
			DCU_GetLayerChromaMax(0, 0,
					(Dcu_Colour_t*)&layer_chroma.max);
			DCU_GetLayerChromaMin(0, 0,
					(Dcu_Colour_t*)&layer_chroma.min);

			/* copy to user space */
			ret = copy_to_user(&layer_chroma,
				(struct IOCTL_LAYER_CHROMA*)arg,
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
				(struct IOCTL_DISPLAY_CFG*)arg,
				sizeof(display_cfg));

			/* set configuration specific settings */
			dcu_lcd_timings.mDeltaX = display_cfg.hactive;
			dcu_lcd_timings.mDeltaY = display_cfg.vactive;
			dcu_lcd_timings.mHorzBP = display_cfg.hback_porch;
			dcu_lcd_timings.mHorzFP = display_cfg.hfront_porch;
			dcu_lcd_timings.mHorzPW = display_cfg.hsync_len;
			dcu_lcd_timings.mVertBP = display_cfg.vback_porch;
			dcu_lcd_timings.mVertPW = display_cfg.vsync_len;
			dcu_lcd_timings.mVertFP = display_cfg.vfront_porch;
			dcu_lcd_timings.mSyncPol= 3;
			dcu_lcd_timings.mVertFq = 60;
			dcu_lcd_timings.mDivFactor = dcu_clk_val / display_cfg.clock_freq;

			if(((display_cfg.hsync_len + display_cfg.vsync_len) == 0) &&
					(display_cfg.disp_type == IOCTL_DISPLAY_HDMI))
			{
				/* set hdmi state */
				enable_hdmi_display("");

				/* query HDMI IP for monitor specs through DDC/EDID */
				/* FIXME: avoid getting EDID info through HDMI direct call*/ 
				monspecs = sii902x_get_monspecs();

				/* search mode and set */
				for(i=0; i<monspecs.modedb_len; i++)
					if((monspecs.modedb[i].xres == dcu_lcd_timings.mDeltaX) &&
							(monspecs.modedb[i].yres == dcu_lcd_timings.mDeltaY))
					{
						dcu_lcd_timings.mHorzBP = monspecs.modedb[i].left_margin;
						dcu_lcd_timings.mHorzFP = monspecs.modedb[i].right_margin;
						dcu_lcd_timings.mHorzPW = monspecs.modedb[i].hsync_len;
						dcu_lcd_timings.mVertBP = monspecs.modedb[i].upper_margin;
						dcu_lcd_timings.mVertPW = monspecs.modedb[i].vsync_len;
						dcu_lcd_timings.mVertFP = monspecs.modedb[i].lower_margin;
						dcu_lcd_timings.mSyncPol= 3;
						dcu_lcd_timings.mVertFq = monspecs.modedb[i].refresh;
						break;
					}
				if((dcu_lcd_timings.mHorzPW + dcu_lcd_timings.mVertPW) == 0)
					printk(KERN_INFO "no match for requested resolution in EDID\n");

			}

			/* set display configuration */
			fsl_dcu_display(display_cfg.disp_type, dcu_lcd_timings);
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
static ssize_t fsl_dcu_read(struct file *fil, char* buff,
		size_t len, loff_t *off){

	int max_bytes;
	int bytes_read = 0;
	__TRACE__;

	max_bytes = wb_len - *off;
	if(max_bytes > len)
		bytes_read = len;
	else
		bytes_read = max_bytes;

	bytes_read = bytes_read - copy_to_user(buff, wb_vir_ptr + (*off), bytes_read);
	*off = *off + bytes_read;

	return bytes_read;
}

/**********************************************************
 * STRUCT operations
 **********************************************************/
struct file_operations dcu_fops = {
	.owner		= THIS_MODULE,
	.read		= fsl_dcu_read,
	.open		= fsl_dcu_open,
	.release	= fsl_dcu_close,
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
		printk ("alloc_chrdev_region failed with %d \n", ret);
		return ret;
	}

	if (!(dcu_class = class_create(THIS_MODULE, DEVICE_NAME))) {
		printk ("class_create failed with %d \n", ret);
		return -1;
	}

	if (!(device_create(dcu_class, NULL,
			dcu_devno, NULL, DEVICE_NAME))) {
		printk ("device_create failed with %d \n", ret);
		return -1;
	}

	/* setup file operations */
	dcu_cdev = cdev_alloc();
	dcu_cdev->ops = &dcu_fops;

	/* add file operations */
	ret = cdev_add(dcu_cdev, dcu_devno, 1);
	if (ret < 0) {
		printk ("cdev_add failed with %d \n", ret);
		return ret;
	}

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_dcu_lcd
 **********************************************************/
int fsl_dcu_lcd_timings(struct platform_device *pdev,
		Dcu_LCD_Para_t* dcu_lcd_timings)
{
	int i, ret;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct display_timings *timings;

	__TRACE__;

	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		printk ("of_parse_phandle failed \n");
		return -ENOENT;
	}

	timings = of_get_display_timings(display_np);
	if (!timings) {
		printk ("of_get_display_timings failed with \n");
		return -ENOENT;
	}

	timings_np = of_find_node_by_name(display_np,
					"display-timings");
	if (!timings_np) {
		printk ("of_find_node_by_name failed with \n");
		return -ENOENT;
	}

	for (i = 0; i < of_get_child_count(timings_np); i++) {
		struct videomode vm;

		ret = videomode_from_timings(timings, &vm, i);
		if(ret < 0){
			printk ("videomode_from_timings failed with \n");
			return ret;
		}

		dcu_lcd_timings->mDeltaX = vm.hactive;
		dcu_lcd_timings->mDeltaY = vm.vactive;
		dcu_lcd_timings->mHorzBP = vm.hback_porch;
		dcu_lcd_timings->mHorzFP = vm.hfront_porch;
		dcu_lcd_timings->mHorzPW = vm.hsync_len;
		dcu_lcd_timings->mVertBP = vm.vback_porch;
		dcu_lcd_timings->mVertPW = vm.vsync_len;
		dcu_lcd_timings->mVertFP = vm.vfront_porch;
		dcu_lcd_timings->mSyncPol= 3;
		dcu_lcd_timings->mVertFq = 60;
		dcu_lcd_timings->mDivFactor = dcu_clk_val / vm.pixelclock;
	}

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_dcu_wb_enable, EXPERIMENTAL code
 **********************************************************/
void fsl_dcu_wb_enable()
{
	Dcu_WriteBack_t dcu_wb_addr;
	dcu_wb_addr.WBAlphaVal = 128;
	dcu_wb_addr.WBType = 0;
	dcu_wb_addr.WBMode = 2;
	dcu_wb_addr.WBAddress = wb_phys_ptr;
	DCU_SetWriteBackConfig(0, &dcu_wb_addr);
}

/**********************************************************
 * FUNCTION: fsl_dcu_wb_disable, EXPERIMENTAL code
 **********************************************************/
void fsl_dcu_wb_disable()
{
	Dcu_WriteBack_t dcu_wb_addr;
	dcu_wb_addr.WBAlphaVal = 128;
	dcu_wb_addr.WBType = 0;
	dcu_wb_addr.WBMode = 0;
	dcu_wb_addr.WBAddress = wb_phys_ptr;
	DCU_SetWriteBackConfig(0, &dcu_wb_addr);
}

/**********************************************************
 * FUNCTION: fsl_dcu_wb_prepare, EXPERIMENTAL code
 **********************************************************/
void fsl_dcu_wb_prepare(struct platform_device *pdev)
{
	wb_len = 1024* 768* 3;
	wb_vir_ptr = dma_alloc_wc(&pdev->dev,
			wb_len * sizeof(char), (dma_addr_t *)&wb_phys_ptr,
			GFP_KERNEL);
}


/**********************************************************
 * FUNCTION: fsl_dcu_probe
 **********************************************************/
int fsl_dcu_probe(struct platform_device *pdev)
{
	struct resource *res;
	Dcu_LCD_Para_t dcu_lcd_timings;
	int ret;

	__TRACE__;

	dcu_pdev = pdev;
	ret = 0;

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

	dcu_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dcu_reg_base)) {
		dcu_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "could not ioremap resource\n");
		return PTR_ERR(dcu_fb_data->reg_base);
	}
	dcu_fb_data->reg_base = dcu_reg_base;

	/* allocate memory for base reg */
	DCU_BASE_ADDR64 = kmalloc(sizeof(uint64_t) * DCU_DEV_COUNT, GFP_KERNEL);
	if(!DCU_BASE_ADDR64){
		dcu_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "could allocate memory for reg_base\n");
		goto failed_alloc_base;
	}

	/* save DCU0 register map to global variable for DCU agnostic layer */
	DCU_BASE_ADDR64[0] = (uint64_t)dcu_reg_base;

	/* enable clocks for DCU */
	dcu_clk = devm_clk_get(&pdev->dev, "dcu");
	if (IS_ERR(dcu_clk)) {
		dcu_init_status = DCU_INIT_ERR_CFG;
		ret = PTR_ERR(dcu_clk);
		dev_err(&pdev->dev, "could not get clock\n");
		goto failed_getclock;
	}
	clk_prepare_enable(dcu_clk);

	/* get DCU clock in Hz*/
	dcu_clk_val = clk_get_rate(dcu_clk);
	dcu_fb_data->clk = dcu_clk;
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* get lcd timings from device tree */
	fsl_dcu_lcd_timings(pdev, &dcu_lcd_timings);

	/* setup display type HDMI/LVDS, by adjusting IOMUX/LDB/TIMINGS */
	/*if(g_enable_hdmi)
		fsl_dcu_display(DCU_DISPLAY_HDMI, dcu_lcd_timings);
	else
	*/
	fsl_dcu_display(DCU_DISPLAY_LVDS, dcu_lcd_timings);

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
 * DCU Linux DTB query related operations
 **********************************************************/
static struct of_device_id fsl_dcu_dt_ids[] = {
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


