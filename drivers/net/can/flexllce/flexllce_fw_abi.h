/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * FlexLLCE (RCU) firmware ABI - host-side mirror of the shared-memory structures.
 *
 * These structures mirror the FlexLLCE firmware Flexllce_InterfaceTypes.h layout
 * 1:1: field widths, packing and offsets are the firmware-imposed ABI and must
 * match the running firmware exactly.
 *
 * The host-side conventions built on top of this ABI - command/return/event
 * enumerations, bit-field masks, shared-memory offsets and helper macros - are
 * hand-maintained in flexllce_if.h, which includes this file.
 *
 * Copyright 2026 NXP
 */
#ifndef _FLEXLLCE_FW_ABI_H_
#define _FLEXLLCE_FW_ABI_H_

#include <linux/build_bug.h>
#include <linux/types.h>

#endif /* _FLEXLLCE_FW_ABI_H_ */
