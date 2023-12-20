/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/*
 * Copyright 2023 NXP
 */

#ifndef S32CC_SCMI_NVMEM_H
#define S32CC_SCMI_NVMEM_H

/*
 * Commands: Read
 * Register(s): SIUL2_0 - MIDR1[31:26]
 * Description: Returns the ASCII code for the SoC Letter (e.g. 'G' -> 71).
 */
#define S32CC_SCMI_NVMEM_SOC_LETTER			(0x0)

/*
 * Commands: Read
 * Register(s): SIUL2_0 - MIDR1[25:16]
 * Description: Returns the SoC Part Number (e.g.: 399).
 */
#define S32CC_SCMI_NVMEM_SOC_PART_NO		(0x1)

/*
 * Commands: Read
 * Register(s): SIUL2_0 - MIDR1[7:4]
 * Description: Returns the MAJOR field of the SoC Revision (e.g.: 1).
 */
#define S32CC_SCMI_NVMEM_SOC_MAJOR			(0x2)

/*
 * Commands: Read
 * Register(s): SIUL2_0 - MIDR1[3:0]
 * Description: Returns the MINOR field of the SoC Revision (e.g.: 1).
 */
#define S32CC_SCMI_NVMEM_SOC_MINOR			(0x3)

/*
 * Commands: Read
 * Register(s): SIUL2_0 - MIDR2[19:16]
 * Description: Returns the FREQUENCY field from the reg, translated according
   to the SoC's Reference Manual (e.g.: 1000MHz -> 0b1011).
 */
#define S32CC_SCMI_NVMEM_CORE_MAX_FREQ		(0x4)

/*
 * Commands: Read
 * Register(s): SIUL2_0 - MIDR1[25:16]
 * Description: Returns the PCIe Device ID based on the SoC Part Number (e.g.:
   for S32G399 -> 0x0).
 */
#define S32CC_SCMI_NVMEM_PCIE_DEV_ID		(0x5)

/*
 * Commands: Read
 * Register(s): SIUL2_1 - MIDR1[15]
 * Description: Returns the SerDes presence (SerDes not present -> 0,
   SerDes present -> 1).
 */
#define S32CC_SCMI_NVMEM_SERDES_PRESENCE	(0x6)

/*
 * Commands: Read
 * Register(s): SIUL2_1 - MIDR2[27:26]
 * Description: Returns the SUBMINOR field of the SoC Revision, if applicable
   (e.g.: 0).
 */
#define S32CC_SCMI_NVMEM_SOC_SUBMINOR		(0x7)

/*
 * Commands: Read
 * Register(s): MC_RGM - DES[31:0], FES[31:0], RDSS[0], MC_ME - MODE_STAT[0]
 * Description: Returns the reset cause determined based on the values of
   mentioned registers, and also clears it by writing a non-zero value to
   MC_RGM - DES[31:0].
 */
#define S32CC_SCMI_NVMEM_RESET_CAUSE		(0x8)

#define S32CC_SCMI_NVMEM_MAX				(0x9)

#define S32CC_SCMI_NVMEM_CELL_SIZE			(0x4)

#endif
