// SPDX-License-Identifier: GPL-2.0
/*
 * DMA support for the Synopsys DesignWare
 * PCIe host controller driver, customized
 * for the NXP S32CC PCIe driver
 *
 * Copyright 2017-2023 NXP
 */

#include <linux/iopoll.h>
#include "pci-dma-s32cc.h"
#include "pci-ioctl-s32cc.h"

/* Reset timeout */
#define DMA_RESET_TIMEOUT_MS	1000
#define DMA_RESET_TIMEOUT_US	(DMA_RESET_TIMEOUT_MS * USEC_PER_MSEC)
#define DMA_RESET_WAIT_US	100

/* Error flags for channel 0 */
#define PCIE_DMA_APP_ERR_CH0			BIT(0)
#define PCIE_DMA_LL_FETCH_ERR_CH0		BIT(16)
#define PCIE_DMA_UNSUPPORTED_REQ_ERR_CH0	BIT(0)
#define PCIE_DMA_CPL_ABORT_ERR_CH0		BIT(8)
#define PCIE_DMA_CPL_TIMEOUT_ERR_CH0		BIT(16)
#define PCIE_DMA_DATA_POISIONING_ERR_CH0	BIT(24)

/* Interrupt masks */
#define PCIE_ABORT_INT_STATUS_MASK		GENMASK(23, 16)
#define PCIE_ABORT_INT_CLEAR_MASK		GENMASK(23, 16)
#define PCIE_DONE_INT_CLEAR_MASK		GENMASK(7, 0)

u32 dw_pcie_read_dma(struct dma_info *di,
		u32 reg, size_t size)
{
	int ret;
	u32 val;

	if (di->read_dma)
		return di->read_dma(di, di->dma_base, reg, size);

	ret = dw_pcie_read(di->dma_base + reg, size, &val);
	if (ret)
		pr_err("%s: Read DMA address failed\n", __func__);

	return val;
}

void dw_pcie_write_dma(struct dma_info *di,
		u32 reg, size_t size, u32 val)
{
	int ret;

	if (di->write_dma) {
		di->write_dma(di, di->dma_base, reg, size, val);
		return;
	}

	ret = dw_pcie_write(di->dma_base + reg, size, val);
	if (ret)
		pr_err("%s: Write DMA address failed\n", __func__);
}

static inline u32 dw_pcie_get_dma_channel_base(struct dma_info *di,
		u8 ch_nr, u8 dir)
{
	if (di->iatu_unroll_enabled)
		return PCIE_DMA_CH_BASE_UNROLL * (ch_nr + 1) +
				PCIE_DMA_IATU_INBOUND_OFF * dir;

	return PCIE_DMA_CH_BASE;
}

int dw_pcie_dma_write_en(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, BIT(0));

	return 0;
}

int dw_pcie_dma_write_soft_reset(struct dma_info *di)
{
	u32 dma_en_reg;
	int ret;

	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, 0x0);
	/* Wait for the enable bit to flip to 0.
	 * In case we get timeout, do not exit,
	 * try to set it anyway, but return -ETIMEDOUT.
	 */
	ret = read_poll_timeout(dw_pcie_readl_dma, dma_en_reg, (dma_en_reg == 0),
				DMA_RESET_WAIT_US, DMA_RESET_TIMEOUT_US, 0,
				di, PCIE_DMA_WRITE_ENGINE_EN);

	di->wr_ch.status = DMA_CH_STOPPED;
	/* Pull back the enable bit to 1 */
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ENGINE_EN, BIT(0));

	return ret;
}

int dw_pcie_dma_read_en(struct dma_info *di)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, BIT(0));

	return 0;
}

int dw_pcie_dma_read_soft_reset(struct dma_info *di)
{
	u32 dma_en_reg;
	int ret;

	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, 0x0);
	/* Wait for the enable bit to flip to 0.
	 * In case we get timeout, do not exit,
	 * try to set it anyway, but return -ETIMEDOUT.
	 */
	ret = read_poll_timeout(dw_pcie_readl_dma, dma_en_reg, (dma_en_reg == 0),
				DMA_RESET_WAIT_US, DMA_RESET_TIMEOUT_US, 0,
				di, PCIE_DMA_READ_ENGINE_EN);

	di->rd_ch.status = DMA_CH_STOPPED;
	/* Pull back the enable bit to 1 */
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ENGINE_EN, BIT(0));

	return ret;
}

static void dw_pcie_dma_set_wr_remote_done_int(struct dma_info *di,
		u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_WRITE);

	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_set_wr_remote_abort_int(struct dma_info *di,
		u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_WRITE);

	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_set_rd_remote_done_int(struct dma_info *di,
		u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_READ);

	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_set_rd_remote_abort_int(struct dma_info *di,
		u64 val, u8 ch_nr)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, DMA_CH_READ);

	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_LOW,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_HIGH,
		upper_32_bits(val));
	/* Set RIE in CTRL1 reg */
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x10);
}

static void dw_pcie_dma_en_local_int(struct dma_info *di,
		u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF,
		dw_pcie_readl_dma(di,
			ch_base + PCIE_DMA_CH_CONTROL1_OFF) | 0x8);
}

/* Interrupts mask and clear functions */
static void dw_pcie_dma_clear_wr_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) & (~val));
}

static void dw_pcie_dma_clear_wr_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_MASK) &
				((~val) << 16));
}

static void dw_pcie_dma_clear_rd_done_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_MASK) & (~val));
}

static void dw_pcie_dma_clear_rd_abort_int_mask(struct dma_info *di, u64 val)
{
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK,
		dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_MASK) &
			((~val) << 16));
}

void dw_pcie_dma_set_sar(struct dma_info *di, u64 val,
		u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_LOW_OFF,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_HIGH_OFF,
		upper_32_bits(val));
}

void dw_pcie_dma_set_dar(struct dma_info *di,
		u64 val, u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_LOW_OFF,
		lower_32_bits(val));
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_HIGH_OFF,
		upper_32_bits(val));
}

void dw_pcie_dma_set_transfer_size(struct dma_info *di,
		u32 val, u8 ch_nr, u8 dir)
{
	u32 ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);

	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_TRANSFER_SIZE_OFF, val);
}

void dw_pcie_dma_set_viewport(struct dma_info *di, u8 ch_nr, u8 dir)
{
	if (!di->iatu_unroll_enabled)
		dw_pcie_writel_dma(di, PCIE_DMA_VIEWPORT_SEL,
				(dir << 31) | ch_nr);
}

void dw_pcie_dma_clear_regs(struct dma_info *di)
{
	u8 ch = 0;
	u8 dir = DMA_CH_WRITE;

	di->wr_ch.status = DMA_CH_STOPPED;
	di->rd_ch.status = DMA_CH_STOPPED;

	for (dir = DMA_CH_WRITE; dir <= DMA_CH_READ; dir++) {
		u32 ch_base = dw_pcie_get_dma_channel_base(di, ch, dir);

		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL2_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_TRANSFER_SIZE_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_LOW_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_SAR_HIGH_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_LOW_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_DAR_HIGH_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_LLP_LOW_OFF, 0);
		dw_pcie_writel_dma(di, ch_base + PCIE_DMA_LLP_HIGH_OFF, 0);
	}

	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_MASK, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_INT_CLEAR, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DONE_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_ABORT_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH01_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH23_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH45_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_WRITE_CH67_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_MASK, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_CLEAR, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_DONE_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_LOW, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_ABORT_IMWR_HIGH, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH01_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH23_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH45_IMWR_DATA, 0);
	dw_pcie_writel_dma(di, PCIE_DMA_READ_CH67_IMWR_DATA, 0);
}

int dw_pcie_dma_single_rw(struct dma_info *di,
		struct dma_data_elem *dma_single_rw)
{
	u32 flags;
	struct dma_ch_info *ptr_ch;
	u8 ch_nr = dma_single_rw->ch_num;
	const u32 dma_nr_ch = dw_pcie_dma_get_nr_chan(di);
	u8 dir;
	u32 ch_base;

	/* Invalid channel number */
	if (ch_nr > dma_nr_ch - 1)
		return -EINVAL;

	/* Invalid transfer size */
	if (dma_single_rw->size > CONFIG_PCIE_DMA_MAX_SIZE)
		return -EINVAL;

	flags = dma_single_rw->flags;
	ptr_ch = (flags & DMA_FLAG_WRITE_ELEM) ?
		&di->wr_ch : &di->rd_ch;

	if (flags & DMA_FLAG_WRITE_ELEM) {
		/* Invalid channel number */
		if (dma_single_rw->ch_num > dma_nr_ch - 1)
			return -EINVAL;

		if (di->wr_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->wr_ch.status = DMA_CH_RUNNING;
		di->wr_ch.errors = 0;
		dw_pcie_dma_write_en(di);
		dir = DMA_CH_WRITE;
	} else {
		/* Invalid channel number */
		if (dma_single_rw->ch_num > dma_nr_ch - 1)
			return -EINVAL;

		if (di->rd_ch.status == DMA_CH_RUNNING)
			return -EBUSY;

		di->rd_ch.status = DMA_CH_RUNNING;
		di->rd_ch.errors = 0;
		dw_pcie_dma_read_en(di);
		dir = DMA_CH_READ;
	}

	if (!di->iatu_unroll_enabled)
		dw_pcie_dma_set_viewport(di, ch_nr, dir);
	/* Clear CR1 for proper init */
	ch_base = dw_pcie_get_dma_channel_base(di, ch_nr, dir);
	dw_pcie_writel_dma(di, ch_base + PCIE_DMA_CH_CONTROL1_OFF, 0);

	if (flags & (DMA_FLAG_EN_DONE_INT | DMA_FLAG_EN_ABORT_INT)) {
		dw_pcie_dma_en_local_int(di, ch_nr, dir);

		if (flags & (DMA_FLAG_RIE | DMA_FLAG_LIE)) {
			if (flags & DMA_FLAG_WRITE_ELEM) {
				dw_pcie_dma_set_wr_remote_abort_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_set_wr_remote_done_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_clear_wr_done_int_mask(di,
					(1 << ch_nr));
				dw_pcie_dma_clear_wr_abort_int_mask(di,
					(1 << ch_nr));
			} else if (flags & DMA_FLAG_READ_ELEM) {
				dw_pcie_dma_set_rd_remote_abort_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_set_rd_remote_done_int(di,
					dma_single_rw->imwr, ch_nr);
				dw_pcie_dma_clear_rd_done_int_mask(di,
					(1 << ch_nr));
				dw_pcie_dma_clear_rd_abort_int_mask(di,
					(1 << ch_nr));
			}
		}
	}
	/* Set transfer size */
	dw_pcie_dma_set_transfer_size(di, dma_single_rw->size, ch_nr, dir);
	/* Set SAR & DAR */
	dw_pcie_dma_set_sar(di, dma_single_rw->sar, ch_nr, dir);
	dw_pcie_dma_set_dar(di, dma_single_rw->dar, ch_nr, dir);

	if (flags & DMA_FLAG_WRITE_ELEM)
		dw_pcie_writel_dma(di, PCIE_DMA_WRITE_DOORBELL, 0);
	else
		dw_pcie_writel_dma(di, PCIE_DMA_READ_DOORBELL, 0);

	return 0;
}

static void dw_pcie_dma_check_errors(struct dma_info *di,
		u32 direction, u32 *error)
{
	u32 val = 0;
	*error = DMA_ERR_NONE;

	if (direction == DMA_CH_WRITE) {
		val = dw_pcie_readl_dma(di, PCIE_DMA_WRITE_ERR_STATUS);
		if (val & PCIE_DMA_APP_ERR_CH0)
			*error |= DMA_ERR_WR;
		if (val & PCIE_DMA_LL_FETCH_ERR_CH0)
			*error |= DMA_ERR_FETCH_LL;
	} else {
		/* Get error status low */
		val = dw_pcie_readl_dma(di, PCIE_DMA_READ_ERR_STATUS_LOW);
		if (val & PCIE_DMA_APP_ERR_CH0)
			*error |= DMA_ERR_RD;
		if (val & PCIE_DMA_LL_FETCH_ERR_CH0)
			*error |= DMA_ERR_FETCH_LL;
		/* Get error status high */
		val = dw_pcie_readl_dma(di, PCIE_DMA_READ_ERR_STATUS_HIGH);
		if (val & PCIE_DMA_UNSUPPORTED_REQ_ERR_CH0)
			*error |= DMA_ERR_UNSUPPORTED_REQ;
		if (val & PCIE_DMA_CPL_ABORT_ERR_CH0)
			*error |= DMA_ERR_CPL_ABORT;
		if (val & PCIE_DMA_CPL_TIMEOUT_ERR_CH0)
			*error |= DMA_ERR_CPL_TIMEOUT;
		if (val & PCIE_DMA_DATA_POISIONING_ERR_CH0)
			*error |= DMA_ERR_DATA_POISIONING;
	}
}

/* Generic int handlers for DMA read and write (separate).
 * They should be called from the platform specific interrupt handler.
 */
static u32 dw_handle_dma_irq_write(struct dma_info *di, u32 val_write)
{
	u32 err_type = DMA_ERR_NONE;

	if (val_write) {
		if (di->wr_ch.status == DMA_CH_RUNNING) {
			if (val_write & PCIE_ABORT_INT_STATUS_MASK) {
				/* Abort interrupt */
				/* Get error type */
				dw_pcie_dma_check_errors(di,
					DMA_FLAG_WRITE_ELEM,
					&di->wr_ch.errors);
				/* Clear interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_WRITE_INT_CLEAR,
					PCIE_ABORT_INT_CLEAR_MASK);
				err_type = di->wr_ch.errors;
			} else { /* Done interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_WRITE_INT_CLEAR,
					PCIE_DONE_INT_CLEAR_MASK);
			}
			di->wr_ch.status = DMA_CH_STOPPED;
		} else {
			/* Clear all interrupts */
			dw_pcie_writel_dma(di,
				PCIE_DMA_WRITE_INT_CLEAR,
				PCIE_ABORT_INT_CLEAR_MASK |
				PCIE_DONE_INT_CLEAR_MASK);
		}
	}

	return err_type;
}

static u32 dw_handle_dma_irq_read(struct dma_info *di, u32 val_read)
{
	u32 err_type = DMA_ERR_NONE;

	if (val_read) {
		if (di->rd_ch.status == DMA_CH_RUNNING) {
			/* Search interrupt type, abort or done */
			if (val_read & PCIE_ABORT_INT_STATUS_MASK) {
				/* Abort interrupt */
				/* Get error type */
				dw_pcie_dma_check_errors(di,
					DMA_FLAG_READ_ELEM,
					&di->rd_ch.errors);
				/* Clear interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_READ_INT_CLEAR,
					PCIE_ABORT_INT_CLEAR_MASK);
				err_type = di->rd_ch.errors;
			} else { /* Done interrupt */
				dw_pcie_writel_dma(di,
					PCIE_DMA_READ_INT_CLEAR,
					PCIE_DONE_INT_CLEAR_MASK);
			}
			di->rd_ch.status = DMA_CH_STOPPED;
		} else {
			/* Clear all interrupts */
			dw_pcie_writel_dma(di, PCIE_DMA_READ_INT_CLEAR,
				PCIE_ABORT_INT_CLEAR_MASK |
				PCIE_DONE_INT_CLEAR_MASK);
		}
	}

	return err_type;
}

void s32cc_register_callback(struct dw_pcie *pcie,
		void (*call_back)(u32 arg))
{
	struct dma_info *di = dw_get_dma_info(pcie);

	di->call_back = call_back;
}

irqreturn_t s32cc_pcie_dma_handler(int irq, void *arg)
{
	struct dw_pcie *pcie = arg;
	struct dma_info *di = dw_get_dma_info(pcie);
	struct s32cc_userspace_info *uinfo =
			dw_get_userspace_info(pcie);
	u32 dma_error = DMA_ERR_NONE;
	bool signal;

	u32 val_write = dw_pcie_readl_dma(di, PCIE_DMA_WRITE_INT_STATUS);
	u32 val_read = dw_pcie_readl_dma(di, PCIE_DMA_READ_INT_STATUS);

	if (val_write) {
		signal = (di->wr_ch.status == DMA_CH_RUNNING);

		dma_error = dw_handle_dma_irq_write(di, val_write);
		if (dma_error != DMA_ERR_NONE)
			dev_info(pcie->dev, "dma write error 0x%0x.\n",
					dma_error);
#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
		else if (di->complete)
			complete(di->complete);
#endif

		if (signal && uinfo->send_signal_to_user)
			uinfo->send_signal_to_user(uinfo);
		if (di->call_back)
			di->call_back(val_write);
	}
	if (val_read) {
		signal = (di->rd_ch.status == DMA_CH_RUNNING);

		dma_error = dw_handle_dma_irq_read(di, val_read);
		if (dma_error != DMA_ERR_NONE)
			dev_info(pcie->dev, "dma read error 0x%0x.\n",
					dma_error);
#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
		else if (di->complete)
			complete(di->complete);
#endif
		if (signal && uinfo->send_signal_to_user)
			uinfo->send_signal_to_user(uinfo);
	}

	return IRQ_HANDLED;
}

void s32cc_config_dma_data(struct dma_info *di, struct dw_pcie *pcie)
{
	if (di) {
		di->call_back = NULL;
		dw_pcie_dma_clear_regs(di);
	}
}

#if (IS_ENABLED(CONFIG_PCI_EPF_TEST))
int dw_pcie_ep_start_dma(struct dw_pcie_ep *ep, bool read,
				 dma_addr_t src, dma_addr_t dst, u32 len,
				 struct completion *complete)
{
	struct dw_pcie *pcie = to_dw_pcie_from_ep(ep);
	struct dma_info *di = dw_get_dma_info(pcie);

	int ret = 0;
	struct dma_data_elem dma_single = { 0, };

	if (read)
		dma_single.flags = (DMA_FLAG_READ_ELEM | DMA_FLAG_EN_DONE_INT |
				DMA_FLAG_LIE);
	else
		dma_single.flags = (DMA_FLAG_WRITE_ELEM | DMA_FLAG_EN_DONE_INT |
				DMA_FLAG_LIE);

	dma_single.size = len;
	dma_single.sar = src;
	dma_single.dar = dst;

	di->complete = complete;

	/* Test the DMA benchmark */
	ret = dw_pcie_dma_single_rw(di, &dma_single);
	return ret;
}
#endif
