# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_VGASTATE)		  += vgastate.o
obj-$(CONFIG_HDMI)		  += hdmi.o
fb-objs			  := $(fb-y)

obj-$(CONFIG_VT)		  += console/
obj-$(CONFIG_FB_STI)		  += console/
obj-$(CONFIG_LOGO)		  += logo/
obj-y				  += backlight/
obj-y				  += fbdev/

obj-$(CONFIG_FB_FSL)		  += fsl_fb.o

obj-$(CONFIG_VIDEOMODE_HELPERS) += display_timing.o videomode.o
ifeq ($(CONFIG_OF),y)
obj-$(CONFIG_VIDEOMODE_HELPERS) += of_display_timing.o of_videomode.o
endif
