/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2023 NXP */
#ifndef LLCE_CORE_H
#define LLCE_CORE_H

#define STATUS_REGS_OFFSET	(0x3C8A0U)
#define LIN_MEM_OFFSET		(0x3C800U)

static inline void __iomem *llce_get_status_regs_addr(void __iomem *shmem)
{
	return shmem + STATUS_REGS_OFFSET;
}

#endif
