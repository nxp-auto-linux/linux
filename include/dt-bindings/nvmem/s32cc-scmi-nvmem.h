/* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
/*
 * Copyright 2023-2024 NXP
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

/*
 * Commands: Read
 * Register(s): A53_GPR - GPR06[0]
 * Description: Returns the value of the lockstep enable bit for the A53
   clusters (lockstep disabled -> 0, lockstep enabled -> 1).
 */
#define S32CC_SCMI_NVMEM_LOCKSTEP_ENABLED	(0x9)

/*
 * Commands: Write
 * Register(s): DDR_GPR - DDR_Config_0[31]
 * Description: Writes 0/1 to the mentioned register, which disables/enables
   the DDR PMU (Monitor Counter Full) IRQ.
 */
#define S32CC_SCMI_NVMEM_DDR_PMU_IRQ		(0xA)

/*
 * Commands: Write
 * Register(s): SRC_0 - GMAC_0_CTRL_STS[3:0]
 * Description: Writes a 4-bit value to the mentioned register, which selects
   the GMAC0 PHY Interface Mode (e.g.: RGMII -> 0b0010).
 */
#define S32CC_SCMI_NVMEM_GMAC0_PHY_INTF_SEL	(0xB)

/*
 * Commands: Write
 * Register(s): SRC_1 - PFE_EMACX_INTF_SEL[11:0]
 * Description: Writes a 12-bit value to the mentioned register, which selects
   the PFE EMACs PHY Interface Mode, 4 bits for each EMAC.
 */
#define S32CC_SCMI_NVMEM_PFE_EMACS_INTF_SEL	(0xC)

/*
 * Commands: Read/Write
 * Register(s): SRC_1 - PFE_COH_EN[5:0]
 * Description: Reads or Writes a 6-bit value from/to the mentioned register,
   which selects the PFE Coherency Mode. Values explained in the SoC Reference
   Manual.
 */
#define S32CC_SCMI_NVMEM_PFE_COH_EN			(0xD)

/*
 * Commands: Write
 * Register(s): SRC_1 - PFE_PWR_CTRL[8:0]
 * Description: Writes a 9-bit value to the mentioned register, which drives
   the PFE Subsystem signals. Values explained in the SoC Reference Manual.
 */
#define S32CC_SCMI_NVMEM_PFE_PWR_CTRL		(0xE)

/*
 * Commands: Write
 * Register(s): SRC_1 - GENCTRL1[2:0]
 * Description: Writes a 3-bit value to the mentioned register, where each bit
   selects if corresponding EMAC runs or not in SGMII mode.
 */
#define S32CC_SCMI_NVMEM_PFE_EMACS_GENCTRL1	(0xF)

/*
 * Commands: Read/Write
 * Register(s): SRC_1 - GENCTRL3[16]
 * Descripition: Reads/Writes a 1-bit value from/to the mentioned register, only
   used for communication between PFE Master and Slave instances.
 */
#define S32CC_SCMI_NVMEM_PFE_GENCTRL3		(0x10)

/*
 * Commands: Write
 * Register(s): DDR_GPR - DDR_RET_CONTROL[0]
 * Description: Writes a 1-bit value to the mentioned register, for enabling/
   disabling the DDR IO retention (0 -> enabled, 1 -> disabled).
 */
#define S32CC_SCMI_NVMEM_DDR_RET_CTRL		(0x11)

/*
 * Commands: Write
 * Register(s): DDR_GPR - DDR_Config_0[14]
 * Description: Writes a 1-bit value to the mentioned register, for enabling/
   disabling the Memory Data retention while in Standby mode
   (0 -> disabled, 1 -> enabled).
 */
#define S32CC_SCMI_NVMEM_DDR_MEM_RET		(0x12)

#define S32CC_SCMI_NVMEM_MAX				(0x13)

#define S32CC_SCMI_NVMEM_CELL_SIZE			(0x4)

#endif
