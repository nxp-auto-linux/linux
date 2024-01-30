/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * This header provides constants for qoriq thermal bindings.
 *
 * Copyright 2024 NXP
 */

#ifndef _DT_BINDINGS_THERMAL_QORIQ_H
#define _DT_BINDINGS_THERMAL_QORIQ_H

#define QORIQ_SENSOR_0_IMMEDIATE_TEMP	0
#define QORIQ_SENSOR_1_IMMEDIATE_TEMP	1
#define QORIQ_SENSOR_2_IMMEDIATE_TEMP	2
#define QORIQ_SENSOR_0_AVERAGE_TEMP	16
#define QORIQ_SENSOR_1_AVERAGE_TEMP	17
#define QORIQ_SENSOR_2_AVERAGE_TEMP	18

/* The average temperature is calculated using this formula:
 * ALPF x Current_Temp + (1 - ALPF) x Average_Temp.
 */
#define QORIQ_TMR_ALPF_1_0	0 /* ALPF = 1.0 */
#define QORIQ_TMR_ALPF_0_5	1 /* ALPF = 0.5 */
#define QORIQ_TMR_ALPF_0_25	2 /* ALPF = 0.25 */
#define QORIQ_TMR_ALPF_0_125	3 /* ALPF = 0.125 */

#endif

