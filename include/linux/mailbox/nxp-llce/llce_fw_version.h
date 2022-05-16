/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2022 NXP */
#ifndef LLCE_FW_VERSION_H
#define LLCE_FW_VERSION_H

#define LLCE_VERSION_MAX_LENGTH 50

/**
 * Firmware version datatype.
 * It contains the version_string which encodes the
 * target derivative and the versions of each feature
 **/
struct llce_fw_version {
	/** OUTPUT: LLCE FW version string actual length. */
	u8 string_length;
	/** OUTPUT: LLCE FW version string. */
	u8 version_string[LLCE_VERSION_MAX_LENGTH];
} __aligned(4) __packed;

#endif /* LLCE_FW_VERSION_H*/
