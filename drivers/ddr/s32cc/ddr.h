/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright 2020-2023 NXP
 *
 */

#ifndef _NXP_DDR_ERR050543_H
#define _NXP_DDR_ERR050543_H

/* ERR050543 related defines */
#define MR4_IDX			4
#define MR4_MASK		0xFFU
#define REF_RATE_MASK		0x7U
#define BYTE_SHIFT		8
#define TUF_THRESHOLD		3
#define REQUIRED_OK_CHECKS	3

/* DDRC Related */
#define OFFSET_DDRC_DERATEEN		0x20
#define DDRC_DERATEEN_ENABLE		0x1
#define DDRC_DERATEEN_DERATE_BYTE_SHIFT	4
#define DDRC_DERATEEN_DERATE_BYTE_MASK	0xfU
#define DDRC_DERATE_BYTE_1		0x1U
#define OFFSET_DDRC_RFSHTMG		0x64
#define DDRC_RFSHTMG_VAL_SHIFT		16
#define DDRC_RFSHTMG_VAL		0xfff
#define DDRC_RFSHTMG_MASK		(DDRC_RFSHTMG_VAL \
		<< DDRC_RFSHTMG_VAL_SHIFT)
#define DDRC_RFSHTMG_UPDATE_SHIFT	2
#define OFFSET_DDRC_RFSHCTL3		0x60
#define DDRC_RFSHCTL3_UPDATE_SHIFT	1
#define DDRC_RFSHCTL3_AUTO_REFRESH_VAL	0x1
#define DDRC_RFSHCTL3_MASK		(DDRC_RFSHCTL3_AUTO_REFRESH_VAL \
		<< DDRC_RFSHCTL3_UPDATE_SHIFT)
#define DDRC_RFSHCTL3_UPDATE_LEVEL_TOGGLE	0x1U
#define OFFSET_DDRC_MRSTAT		0x18
#define DDRC_MRSTAT_MR_WR_FLAG		0x1
#define OFFSET_DDRC_MRCTRL0		0x10
#define DDRC_MRCTRL0_MR_TYPE_READ	0x1
#define DDRC_MRCTRL0_MR_RANK_SHIFT	4
#define DDRC_MRCTRL0_MR_WR_SHIFT	31
#define DDRC_MRCTRL0_MR_WR_VAL		0x3
#define DDRC_MRCTRL0_MR_WR_MASK		BIT(DDRC_MRCTRL0_MR_WR_SHIFT)
#define DDRC_MRCTRL0_MR_RANK_OFF	BIT(DDRC_MRCTRL0_MR_RANK_SHIFT)
#define DDRC_MRCTRL0_MR_RANK_MASK	(DDRC_MRCTRL0_MR_WR_VAL \
		<< DDRC_MRCTRL0_MR_RANK_SHIFT)
#define DDRC_MRCTRL0_RANK_ACCESS_POS	4
#define DDRC_MRCTRL0_RANK_ACCESS_FIELD	(0xfU)
#define DDRC_MRCTRL0_RANK_0		0x1U
#define DDRC_MRCTRL0_WR_ENGAGE		(0x1U)
#define DDRC_MRCTRL0_WR_ENGAGE_POS	31

#define OFFSET_DDRC_MRCTRL1		0x14
#define DDRC_MRCTRL1_MR_VAL		0xff
#define DDRC_MRCTRL1_MR_ADDR_SHIFT	8
#define DDRC_MRCTRL1_MR_ADDR_MASK	(DDRC_MRCTRL1_MR_VAL \
		<< DDRC_MRCTRL1_MR_ADDR_SHIFT)
#define DDRC_MRCTRL1_MR_ADDRESS_FIELD	(0xffU)
#define DDRC_MRCTRL1_MR_ADDRESS_POS	8

/* Performance monitoring registers */
#define OFFSET_MRR_0_DATA_REG_ADDR	0x40
#define MRR_0_DDR_SEL_REG_MASK          0x1U
#define OFFSET_MRR_1_DATA_REG_ADDR	0x44

#define DDRC_MRSTAT_MR_BUSY                  0x1U
#define DDRC_MRSTAT_MR_NOT_BUSY              0x0U
#define DDRC_REQUIRED_MRSTAT_READS		0x2U


#define SUCCESSIVE_READ			2

#define ERRATA_CHANGES_REVERTED		1
#define ERRATA_CHANGES_UNMODIFIED	0

/* Read lpddr4 mode register with given rank and index */
u32 read_lpddr4_mr(u8 mr_rank, u8 mr_index, void __iomem *ddrc_base,
		   void __iomem *perf_base);

/*
 * Read Temperature Update Flag from lpddr4 MR4 register.
 * This method actually reads the first 3 bits of MR4 (MR4[2:0])
 * instead of the TUF flag.
 * The return value is being used in order to determine if the
 * timing parameters need to be adjusted or not.
 */
uint8_t read_TUF(void __iomem *ddrc_base,
		 void __iomem *perf_base);

/*
 * Periodically read Temperature Update Flag in MR4 and undo changes made by
 * ERR050543 workaround if no longer needed. Refresh rate is updated and auto
 * derating is turned on.
 * @param traffic_halted - if ddr traffic was halted, restore also timing
 * parameters
 */
int poll_derating_temp_errata(void __iomem *ddrc_base,
			      void __iomem *perf_base);

#endif /* _NXP_DDR_ERR050543_H */
