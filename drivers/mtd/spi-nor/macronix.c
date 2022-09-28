// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 * Copyright 2022-2025 NXP
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

static int
mx25l25635_post_bfpt_fixups(struct spi_nor *nor,
			    const struct sfdp_parameter_header *bfpt_header,
			    const struct sfdp_bfpt *bfpt)
{
	/*
	 * MX25L25635F supports 4B opcodes but MX25L25635E does not.
	 * Unfortunately, Macronix has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * We need a way to differentiate MX25L25635E and MX25L25635F, and it
	 * seems that the F version advertises support for Fast Read 4-4-4 in
	 * its BFPT table.
	 */
	if (bfpt->dwords[SFDP_DWORD(5)] & BFPT_DWORD5_FAST_READ_4_4_4)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static int
macronix_qpp4b_post_sfdp_fixups(struct spi_nor *nor)
{
	/* PP_1_1_4_4B is supported but missing in 4BAIT. */
	struct spi_nor_flash_parameter *params = nor->params;

	params->hwcaps.mask |= SNOR_HWCAPS_PP_1_1_4;
	spi_nor_set_pp_settings(&params->page_programs[SNOR_CMD_PP_1_1_4],
				SPINOR_OP_PP_1_1_4_4B, SNOR_PROTO_1_1_4);

	return 0;
}

static const struct spi_nor_fixups mx25l25635_fixups = {
	.post_bfpt = mx25l25635_post_bfpt_fixups,
	.post_sfdp = macronix_qpp4b_post_sfdp_fixups,
};

static const struct spi_nor_fixups macronix_qpp4b_fixups = {
	.post_sfdp = macronix_qpp4b_post_sfdp_fixups,
};

static int spi_nor_macronix_set_octal_dtr(struct spi_nor *nor, bool enable)
{
	struct spi_mem_op op;
	u8 *buf = nor->bouncebuf;
	*buf = 0xff;
	u8 addr_width = 4;
	int ret;

	/* The assumption is that the memory is in SPI 1-1-1 mode by default */
	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDCR2, 1),
			   SPI_MEM_OP_ADDR(addr_width, 0x0, 1),
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_DATA_IN(1, buf, 1));

	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret) {
		dev_err(nor->dev, "Failed to read CR2\n");
		return ret;
	}

	ret = spi_nor_write_enable(nor);
	if (ret) {
		dev_err(nor->dev, "Failed to enable write\n");
		return ret;
	}

	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;

	*buf &= ~CR2_STR_OPI_EN;
	*buf |= CR2_DTR_OPI_EN;

	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WRCR2, 1),
			   SPI_MEM_OP_ADDR(addr_width, 0x0, 1),
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_DATA_OUT(1, buf, 1));
	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret) {
		dev_err(nor->dev, "Failed to enable octal DTR mode\n");
		return ret;
	}

	return 0;
}

static void mx25uw51245g_default_init(struct spi_nor *nor)
{
	nor->params->set_octal_dtr = spi_nor_macronix_set_octal_dtr;
}

static int mx25uw51245g_late_init(struct spi_nor *nor)
{
	/* erase size in case it is set to 4K from BFPT */
	nor->erase_opcode = SPINOR_OP_SE_4B;
	nor->mtd.erasesize = nor->info->sector_size;

	nor->params->hwcaps.mask |= SNOR_HWCAPS_PP_8_8_8_DTR;

	spi_nor_set_pp_settings(&nor->params->page_programs[SNOR_CMD_PP_8_8_8_DTR],
				SPINOR_OP_PP_4B, SNOR_PROTO_8_8_8_DTR);

	/* The spi-nor core will always double the amount of dummy cycles if the
	 * protocol is DTR, but this memory's SFDP already takes it into account.
	 * Maximum 20 cycles are needed and using too many can cause the device
	 * to run incorrectly.
	 */
	spi_nor_set_read_settings(&nor->params->reads[SNOR_CMD_READ_8_8_8_DTR],
				0, 10, SPINOR_OP_READ_1_4_4_DTR_4B,
				SNOR_PROTO_8_8_8_DTR);

	/* Identify (free) samples with empty SFDP table */
	if (nor->cmd_ext_type == SPI_NOR_EXT_NONE) {
		nor->cmd_ext_type = SPI_NOR_EXT_INVERT;

		nor->params->rdsr_addr_nbytes = 4;
		nor->params->rdsr_dummy = 4;
	}

	return 0;
}

static struct spi_nor_fixups mx25uw51245g_fixups = {
	.default_init = mx25uw51245g_default_init,
	.late_init = mx25uw51245g_late_init,
};

static const struct flash_info macronix_nor_parts[] = {
	{
		.id = SNOR_ID(0xc2, 0x20, 0x10),
		.name = "mx25l512e",
		.size = SZ_64K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x12),
		.name = "mx25l2005a",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x13),
		.name = "mx25l4005a",
		.size = SZ_512K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x14),
		.name = "mx25l8005",
		.size = SZ_1M,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x15),
		.name = "mx25l1606e",
		.size = SZ_2M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x16),
		.name = "mx25l3205d",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x17),
		.name = "mx25l6405d",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x18),
		.name = "mx25l12805d",
		.size = SZ_16M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_4BIT_BP,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x19),
		.name = "mx25l25635e",
		.size = SZ_32M,
		.no_sfdp_flags = SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixups = &mx25l25635_fixups
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x1a),
		.name = "mx66l51235f",
		.size = SZ_64M,
		.no_sfdp_flags = SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
		.fixups = &macronix_qpp4b_fixups,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x1b),
		.name = "mx66l1g45g",
		.size = SZ_128M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixups = &macronix_qpp4b_fixups,
	}, {
		/* MX66L2G45G */
		.id = SNOR_ID(0xc2, 0x20, 0x1c),
		.fixups = &macronix_qpp4b_fixups,
	}, {
		.id = SNOR_ID(0xc2, 0x23, 0x14),
		.name = "mx25v8035f",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x32),
		.name = "mx25u2033e",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x33),
		.name = "mx25u4035",
		.size = SZ_512K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x34),
		.name = "mx25u8035",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x36),
		.name = "mx25u3235f",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x37),
		.name = "mx25u6435f",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x38),
		.name = "mx25u12835f",
		.size = SZ_16M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x39),
		.name = "mx25u25635f",
		.size = SZ_32M,
		.no_sfdp_flags = SECT_4K,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3a),
		.name = "mx25u51245g",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
		.fixups = &macronix_qpp4b_fixups,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3a),
		.name = "mx66u51235f",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
		.fixups = &macronix_qpp4b_fixups,
	}, {
		/* MX66U1G45G */
		.id = SNOR_ID(0xc2, 0x25, 0x3b),
		.fixups = &macronix_qpp4b_fixups,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3c),
		.name = "mx66u2g45g",
		.size = SZ_256M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
		.fixups = &macronix_qpp4b_fixups,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x18),
		.name = "mx25l12855e",
		.size = SZ_16M,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x19),
		.name = "mx25l25655e",
		.size = SZ_32M,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x1b),
		.name = "mx66l1g55g",
		.size = SZ_128M,
		.no_sfdp_flags = SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x28, 0x15),
		.name = "mx25r1635f",
		.size = SZ_2M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x28, 0x16),
		.name = "mx25r3235f",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x81, 0x3a),
		.name = "mx25uw51245g",
		.size = SZ_64M,
		.n_banks = 4,
		.no_sfdp_flags = SPI_NOR_OCTAL_READ | SPI_NOR_OCTAL_DTR_READ,
		.flags = SPI_NOR_RWW,
		.fixups = &mx25uw51245g_fixups,

	}, {
		.id = SNOR_ID(0xc2, 0x9e, 0x16),
		.name = "mx25l3255e",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}
};

static void macronix_nor_default_init(struct spi_nor *nor)
{
	nor->params->quad_enable = spi_nor_sr1_bit6_quad_enable;
}

static int macronix_nor_late_init(struct spi_nor *nor)
{
	if (!nor->params->set_4byte_addr_mode)
		nor->params->set_4byte_addr_mode = spi_nor_set_4byte_addr_mode_en4b_ex4b;

	return 0;
}

static const struct spi_nor_fixups macronix_nor_fixups = {
	.default_init = macronix_nor_default_init,
	.late_init = macronix_nor_late_init,
};

const struct spi_nor_manufacturer spi_nor_macronix = {
	.name = "macronix",
	.parts = macronix_nor_parts,
	.nparts = ARRAY_SIZE(macronix_nor_parts),
	.fixups = &macronix_nor_fixups,
};
