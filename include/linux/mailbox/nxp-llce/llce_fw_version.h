/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020 NXP */
#ifndef LLCE_FW_VERSION_H
#define LLCE_FW_VERSION_H

#define LLCE_VERSION_MAX_LENGTH 50

/**
 * CAN functionality list type.
 *
 * Enumeration containing the list of features.
 */
enum llce_component_version {
	/** Label of the version value regarding interface feature. */
	LLCE_VERSION_INTERFACE = 0,
	/** Label of the version value regarding interface basic features. */
	LLCE_VERSION_BASIC_FUNC,
	/**
	 * Label of the version value regarding can2can internal
	 * routing feature.
	 */
	LLCE_VERSION_ROUTING_CAN2CAN,
	/**
	 * Label of the version value regarding can2eth internal
	 * routing feature.
	 */
	LLCE_VERSION_ROUTING_CAN2ETH,
	/** Label of the version value regarding logging feature. */
	LLCE_VERSION_LOGGING
};

#endif /* LLCE_FW_VERSION_H*/
