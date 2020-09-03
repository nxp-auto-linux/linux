// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 *
 */

#include <linux/io.h>
#include "ddr.h"

/*
 * Periodically read Temperature Update Flag in MR4 and undo changes made by
 * ERR050543 workaround if no longer needed. Refresh rate is updated and auto
 * derating is turned on.
 * @param ddrc_base - Base address of DDRC controller
 * @param perf_base - Base address of Perf monitor unit
 * @return - Returns 1, if the errata changes are reverted, 0 otherwise
 */
int poll_derating_temp_errata(void __iomem *ddrc_base,
		void __iomem *perf_base)
{
	int nominal_temp_flag = 0;
	uint8_t val_1, val_2;
	uint32_t reg, bf_val;

	if (read_TUF(ddrc_base, perf_base) <= TUF_THRESHOLD) {
		nominal_temp_flag++;

		val_1 = read_TUF(ddrc_base, perf_base);
		val_2 = read_TUF(ddrc_base, perf_base);

		if (val_1 <= TUF_THRESHOLD && val_2 <= TUF_THRESHOLD)
			nominal_temp_flag += SUCCESSIVE_READ;
	}

	if (nominal_temp_flag != REQUIRED_OK_CHECKS)
		return 0;

	/*
	 * Update average time interval between refreshes per rank:
	 * RFSHTMG.T_RFC_NOM_X1_X32 = RFSHTMG.T_RFC_NOM_X1_X32 * 4
	 */
	reg = readl(ddrc_base + OFFSET_DDRC_RFSHTMG);
	bf_val = (reg >> DDRC_RFSHTMG_VAL_SHIFT) & DDRC_RFSHTMG_VAL;
	bf_val = bf_val << 2;
	reg = (reg & ~DDRC_RFSHTMG_MASK) |
		(bf_val << DDRC_RFSHTMG_VAL_SHIFT);
	writel(reg, ddrc_base + OFFSET_DDRC_RFSHTMG);

	/*
	 * Toggle RFSHCTL3.REFRESH_UPDATE_LEVEL to indicate that
	 * refresh registers have been updated
	 */
	reg = readl(ddrc_base + OFFSET_DDRC_RFSHCTL3);
	bf_val = (reg >> DDRC_RFSHCTL3_UPDATE_SHIFT) &
		DDRC_RFSHCTL3_AUTO_REFRESH_VAL;
	bf_val = bf_val ^ 1;
	writel((reg & ~DDRC_RFSHCTL3_MASK) |
			(bf_val << DDRC_RFSHCTL3_UPDATE_SHIFT),
		   ddrc_base + OFFSET_DDRC_RFSHCTL3);

	/* Enable timing parameter derating: DERATEEN.DERATE_EN = 1 */
	reg = readl(ddrc_base + OFFSET_DDRC_DERATEEN);
	writel(reg | DDRC_DERATEEN_ENABLE, ddrc_base +
			OFFSET_DDRC_DERATEEN);

	return 1;
}
