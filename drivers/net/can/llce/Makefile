# SPDX-License-Identifier: GPL-2.0
#
#  Makefile for the NXP LLCE CAN drivers
#
obj-$(CONFIG_CAN_LLCE)			+= llce_can_common.o
obj-$(CONFIG_CAN_LLCE_CORE)		+= llce_can_core.o
ifneq ($(CONFIG_DEBUG_FS),)
obj-$(CONFIG_CAN_LLCE_CORE)		+= llce_can_core_debugfs.o
endif
obj-$(CONFIG_CAN_LLCE_CONTROLLER)	+= llce_can.o
obj-$(CONFIG_CAN_LLCE_LOGGER)		+= llce_logger.o

