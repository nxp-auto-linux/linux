// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * NXP S32N79 PCIe NTB host-side endpoint driver
 *
 * Copyright 2026 NXP
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ntb.h>
#include <linux/pci.h>
#include <linux/sizes.h>

#define NXP_NTB_DEVICE_ID	0x0f80

enum nxp_ntb_bar {
	NXP_NTB_BAR_DB	= 0,	/* Doorbell registers */
	NXP_NTB_BAR_CFG	= 1,	/* NTB Config Structure */
	NXP_NTB_BAR_MW1	= 2,	/* 32-bit MW1 or 64-bit MW1 lower half */
	NXP_NTB_BAR_MW2	= 3,	/* 32-bit MW2 or 64-bit MW1 upper half */
	NXP_NTB_BAR_MW3	= 4,	/* 32-bit MW3 or 64-bit MW2 lower half */
	NXP_NTB_BAR_MW4	= 5,	/* 32-bit MW4 or 64-bit MW2 upper half */
};

/* Doorbell register block offset within BAR0 */
#define NXP_NTB_DB_BLOCK_OFFSET		0x30000

/* Config structure (BAR1) register map */
#define NXP_NTB_CFG_VERSION		0x00	/* 32-bit */
#define NXP_NTB_CFG_CMD_ID		0x04	/* 32-bit */
#define NXP_NTB_CFG_ARGUMENT		0x08	/* 32-bit */
#define NXP_NTB_CFG_CMD_STATUS		0x0C	/* 32-bit */
#define NXP_NTB_CFG_LINK_STATUS		0x10	/* 32-bit */
#define NXP_NTB_CFG_MW_ADDRESS		0x18	/* 64-bit */
#define NXP_NTB_CFG_MW_SIZE		0x20	/* 64-bit */

#define NXP_NTB_CFG_VERSION_V01		1

/* CMD_ID values (BAR1 + 0x04) */
#define NXP_NTB_CMD_CONFIGURE_MW	0x1
#define NXP_NTB_CMD_TEARDOWN_MW		0x2
#define NXP_NTB_CMD_LINK_UP		0x3
#define NXP_NTB_CMD_LINK_DOWN		0x4

/* CMD_STATUS values (BAR1 + 0x0C) */
#define NXP_NTB_CMD_STATUS_NEW		0x0
#define NXP_NTB_CMD_STATUS_OK		0x1
#define NXP_NTB_CMD_STATUS_FAIL		0x2
#define NXP_NTB_CMD_STATUS_BAD_CMD	0x3
#define NXP_NTB_CMD_STATUS_BAD_ARGS	0x4
#define NXP_NTB_CMD_STATUS_BAD_STATE	0x5

/* LINK_STATUS bits (BAR1 + 0x10) */
#define NXP_NTB_LINK_STATUS_UP		BIT(0)

/* Doorbell register block (BAR0 + NXP_NTB_DB_BLOCK_OFFSET) */
#define NXP_NTB_DB_DBISTAT		0x00	/* W1C interrupt status */
#define NXP_NTB_DB_DB_CR0		0x04	/* MSI-X vector map DB0..DB3 */
#define NXP_NTB_DB_DB_CR1		0x08	/* MSI-X vector map DB4..DB7 */
#define NXP_NTB_DB_OMR(n)		(0x40 + (n) * 0x10)
#define NXP_NTB_DB_IMR(n)		(0x48 + (n) * 0x10)

/*
 * Each VEC field in DB_CR0/DB_CR1 is 5 bits wide at bit positions
 * [7:3], [15:11], [23:19], [31:27] — i.e. (slot % 4) * 8 + 3.
 */
#define NXP_NTB_DB_VEC_SHIFT(slot)	(((slot) & 3) * 8 + 3)
#define NXP_NTB_DB_VEC(slot, vec)	((u32)(vec) << NXP_NTB_DB_VEC_SHIFT(slot))

/* DBn -> MSI-X vector n (DB0..DB3 in CR0, DB4..DB7 in CR1) */
#define NXP_NTB_DB_CR0_DEFAULT \
	(NXP_NTB_DB_VEC(0, 0) | NXP_NTB_DB_VEC(1, 1) | \
	 NXP_NTB_DB_VEC(2, 2) | NXP_NTB_DB_VEC(3, 3))
#define NXP_NTB_DB_CR1_DEFAULT \
	(NXP_NTB_DB_VEC(0, 4) | NXP_NTB_DB_VEC(1, 5) | \
	 NXP_NTB_DB_VEC(2, 6) | NXP_NTB_DB_VEC(3, 7))

#define NXP_NTB_NUM_IRQS		8	/* number of HW doorbells */

/*
 * HW Doorbells allocation:
 *   DB0..DB4 : 5 NTB message registers (msg API)
 *   DB5..DB6 : 2 NTB doorbells (db API)
 *   DB7      : reserved for host to endpoint commands + link-event signaling
 */
#define NXP_NTB_NUM_MSGS		5
#define NXP_NTB_MSG_VALID_MASK		(BIT(NXP_NTB_NUM_MSGS) - 1)
#define NXP_NTB_NUM_DBS			2
#define NXP_NTB_DB_SLOT_BASE		NXP_NTB_NUM_MSGS	/* DB5 */
#define NXP_NTB_DB_VALID_MASK		(BIT(NXP_NTB_NUM_DBS) - 1)
#define NXP_NTB_CMD_DB			(NXP_NTB_NUM_IRQS - 1)	/* DB7 */

/* Maximum memory windows the host can ever expose (BAR2..BAR5) */
#define NXP_NTB_MAX_MWS			4

#define NXP_NTB_CMD_TIMEOUT_MS		1000
#define NXP_NTB_CMD_POLL_US		10

/**
 * struct nxp_ntb_dev - NXP S32N79 NTB device state
 * @ntb:      embedded NTB subsystem device
 * @dev:      backing struct device pointer
 * @cmd_lock: serialises commands issued via the BAR1 config structure
 * @cfg_reg:  iomap of BAR1 (NTB config structure)
 * @db_reg:   iomap of BAR0 doorbell block (BAR0 + NXP_NTB_DB_BLOCK_OFFSET)
 * @irq_base: Linux IRQ number of MSI-X vector 0, used to derive the vector
 *            index in the ISR
 * @mw_count: number of memory windows discovered from BAR2..BAR5 scan
 * @mw_bars:  mw_idx to BAR number map populated at probe time
 */
struct nxp_ntb_dev {
	struct ntb_dev	ntb;
	struct device	*dev;
	struct mutex	cmd_lock;

	void __iomem	*cfg_reg;
	void __iomem	*db_reg;

	int		irq_base;
	int		mw_count;
	u8		mw_bars[NXP_NTB_MAX_MWS];
};

#define ntb_to_nxp(__ntb) container_of(__ntb, struct nxp_ntb_dev, ntb)

/*
 * Submit a command to the S32N79 NTB endpoint and wait for completion. The
 * caller must already hold @cmd_lock; commands with additional arguments
 * (CMD_CONFIGURE_MW: MW_ADDRESS, MW_SIZE) must write those fields under the
 * same lock before invoking this helper.
 */
static int nxp_ntb_send_command_locked(struct nxp_ntb_dev *ndev, u32 cmd, u32 arg)
{
	struct device *dev = ndev->dev;
	u32 status;
	int ret;

	writel(NXP_NTB_CMD_STATUS_NEW, ndev->cfg_reg + NXP_NTB_CFG_CMD_STATUS);
	writel(arg, ndev->cfg_reg + NXP_NTB_CFG_ARGUMENT);
	writel(cmd, ndev->cfg_reg + NXP_NTB_CFG_CMD_ID);
	/* ensure config writes land in endpoint before the CMD_DB signal */
	(void)readl(ndev->cfg_reg + NXP_NTB_CFG_CMD_ID);
	writel(cmd, ndev->db_reg + NXP_NTB_DB_OMR(NXP_NTB_CMD_DB));

	ret = readl_poll_timeout(ndev->cfg_reg + NXP_NTB_CFG_CMD_STATUS,
				 status,
				 status != NXP_NTB_CMD_STATUS_NEW,
				 NXP_NTB_CMD_POLL_US,
				 NXP_NTB_CMD_TIMEOUT_MS * USEC_PER_MSEC);
	if (ret) {
		dev_err(dev, "command 0x%x timed out\n", cmd);
		return ret;
	}

	switch (status) {
	case NXP_NTB_CMD_STATUS_OK:
		ret = 0;
		break;
	case NXP_NTB_CMD_STATUS_FAIL:
		dev_err(dev, "command 0x%x: failed\n", cmd);
		ret = -EIO;
		break;
	case NXP_NTB_CMD_STATUS_BAD_CMD:
		dev_err(dev, "command 0x%x: rejected (bad cmd)\n", cmd);
		ret = -EINVAL;
		break;
	case NXP_NTB_CMD_STATUS_BAD_ARGS:
		dev_err(dev, "command 0x%x: rejected (bad args)\n", cmd);
		ret = -EINVAL;
		break;
	case NXP_NTB_CMD_STATUS_BAD_STATE:
		dev_err(dev, "command 0x%x: rejected (bad state)\n", cmd);
		ret = -EBUSY;
		break;
	default:
		dev_err(dev, "command 0x%x: unknown status 0x%x\n", cmd, status);
		ret = -EPROTO;
		break;
	}

	return ret;
}

static int nxp_ntb_send_command(struct nxp_ntb_dev *ndev, u32 cmd, u32 arg)
{
	int ret;

	mutex_lock(&ndev->cmd_lock);
	ret = nxp_ntb_send_command_locked(ndev, cmd, arg);
	mutex_unlock(&ndev->cmd_lock);
	return ret;
}

static u64 nxp_ntb_link_is_up(struct ntb_dev *ntb, enum ntb_speed *speed,
			      enum ntb_width *width)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);

	if (speed)
		*speed = NTB_SPEED_AUTO;
	if (width)
		*width = NTB_WIDTH_AUTO;

	return readl(ndev->cfg_reg + NXP_NTB_CFG_LINK_STATUS) &
		NXP_NTB_LINK_STATUS_UP;
}

static int nxp_ntb_link_enable(struct ntb_dev *ntb, enum ntb_speed max_speed,
			       enum ntb_width max_width)
{
	return nxp_ntb_send_command(ntb_to_nxp(ntb), NXP_NTB_CMD_LINK_UP, 0);
}

static int nxp_ntb_link_disable(struct ntb_dev *ntb)
{
	return nxp_ntb_send_command(ntb_to_nxp(ntb), NXP_NTB_CMD_LINK_DOWN, 0);
}

static u64 nxp_ntb_db_valid_mask(struct ntb_dev *ntb)
{
	return NXP_NTB_DB_VALID_MASK;
}

static int nxp_ntb_db_vector_count(struct ntb_dev *ntb)
{
	return NXP_NTB_NUM_DBS;
}

static u64 nxp_ntb_db_vector_mask(struct ntb_dev *ntb, int db_vector)
{
	if (db_vector < 0 || db_vector >= NXP_NTB_NUM_DBS)
		return 0;

	return BIT_ULL(db_vector);
}

static u64 nxp_ntb_db_read(struct ntb_dev *ntb)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);
	u32 sts = readl(ndev->db_reg + NXP_NTB_DB_DBISTAT);

	return (sts >> NXP_NTB_DB_SLOT_BASE) & NXP_NTB_DB_VALID_MASK;
}

static int nxp_ntb_db_clear(struct ntb_dev *ntb, u64 db_bits)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);

	writel((u32)((db_bits & NXP_NTB_DB_VALID_MASK) << NXP_NTB_DB_SLOT_BASE),
	       ndev->db_reg + NXP_NTB_DB_DBISTAT);
	return 0;
}

/* Doorbell masking is not supported in S32N79 hardware */
static int nxp_ntb_db_set_mask(struct ntb_dev *ntb, u64 db_bits)
{
	return 0;
}

static int nxp_ntb_db_clear_mask(struct ntb_dev *ntb, u64 db_bits)
{
	return 0;
}

/*
 * Ring one or more peer doorbells. Each set bit in db_bits triggers a
 * write to a distinct OMR in the doorbell range (DB5..DB6)
 */
static int nxp_ntb_peer_db_set(struct ntb_dev *ntb, u64 db_bits)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);
	int bit;

	if (db_bits & ~NXP_NTB_DB_VALID_MASK)
		return -EINVAL;

	for (bit = 0; bit < NXP_NTB_NUM_DBS; bit++) {
		if (!(db_bits & BIT(bit)))
			continue;

		writel(1, ndev->db_reg +
			NXP_NTB_DB_OMR(NXP_NTB_DB_SLOT_BASE + bit));
		/* flush posted write */
		(void)readl(ndev->db_reg +
				NXP_NTB_DB_OMR(NXP_NTB_DB_SLOT_BASE + bit));
	}

	return 0;
}

static int nxp_ntb_msg_count(struct ntb_dev *ntb)
{
	return NXP_NTB_NUM_MSGS;
}

static u64 nxp_ntb_msg_inbits(struct ntb_dev *ntb)
{
	return NXP_NTB_MSG_VALID_MASK;
}

/* Outbound message registers status is not supported in HW */
static u64 nxp_ntb_msg_outbits(struct ntb_dev *ntb)
{
	return 0;
}

static u64 nxp_ntb_msg_read_sts(struct ntb_dev *ntb)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);

	return readl(ndev->db_reg + NXP_NTB_DB_DBISTAT) & NXP_NTB_MSG_VALID_MASK;
}

static int nxp_ntb_msg_clear_sts(struct ntb_dev *ntb, u64 sts_bits)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);

	writel((u32)(sts_bits & NXP_NTB_MSG_VALID_MASK),
	       ndev->db_reg + NXP_NTB_DB_DBISTAT);
	return 0;
}

/* Message-status masking is not supported in HW */
static int nxp_ntb_msg_set_mask(struct ntb_dev *ntb, u64 mask_bits)
{
	return 0;
}

static int nxp_ntb_msg_clear_mask(struct ntb_dev *ntb, u64 mask_bits)
{
	return 0;
}

static u32 nxp_ntb_msg_read(struct ntb_dev *ntb, int *pidx, int midx)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);

	if (midx < 0 || midx >= NXP_NTB_NUM_MSGS)
		return ~(u32)0;

	if (pidx)
		*pidx = NTB_DEF_PEER_IDX;

	return readl(ndev->db_reg + NXP_NTB_DB_IMR(midx));
}

static int nxp_ntb_peer_msg_write(struct ntb_dev *ntb, int pidx, int midx,
				  u32 msg)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);

	if (pidx != NTB_DEF_PEER_IDX || midx < 0 || midx >= NXP_NTB_NUM_MSGS)
		return -EINVAL;

	writel(msg, ndev->db_reg + NXP_NTB_DB_OMR(midx));

	/* flush posted write */
	(void)readl(ndev->db_reg + NXP_NTB_DB_OMR(midx));
	return 0;
}

static int nxp_ntb_mw_to_bar(struct nxp_ntb_dev *ndev, int widx)
{
	if (widx < 0 || widx >= ndev->mw_count)
		return -EINVAL;
	return ndev->mw_bars[widx];
}

static int nxp_ntb_mw_count(struct ntb_dev *ntb, int pidx)
{
	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;
	return ntb_to_nxp(ntb)->mw_count;
}

static int nxp_ntb_peer_mw_count(struct ntb_dev *ntb)
{
	return ntb_to_nxp(ntb)->mw_count;
}

static int nxp_ntb_mw_get_align(struct ntb_dev *ntb, int pidx, int widx,
				resource_size_t *addr_align,
				resource_size_t *size_align,
				resource_size_t *size_max)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);
	int bar;

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	bar = nxp_ntb_mw_to_bar(ndev, widx);
	if (bar < 0)
		return bar;

	if (addr_align)
		*addr_align = SZ_4K;
	if (size_align)
		*size_align = SZ_4K;
	if (size_max)
		*size_max = pci_resource_len(ndev->ntb.pdev, bar);

	return 0;
}

/*
 * Program the endpoint's inbound translation for MW widx so peer writes into
 * BAR(widx) land at host DMA address addr (up to size bytes).
 */
static int nxp_ntb_mw_set_trans(struct ntb_dev *ntb, int pidx, int widx,
				dma_addr_t addr, resource_size_t size)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);
	resource_size_t bar_size;
	int bar, ret;

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	bar = nxp_ntb_mw_to_bar(ndev, widx);
	if (bar < 0)
		return bar;

	if (!IS_ALIGNED(addr, SZ_4K) || !IS_ALIGNED(size, SZ_4K)) {
		dev_err(ndev->dev,
			"MW%d: addr %pad and size %pap must be 4K aligned\n",
			widx, &addr, &size);
		return -EINVAL;
	}

	bar_size = pci_resource_len(ndev->ntb.pdev, bar);
	if (size > bar_size) {
		dev_err(ndev->dev,
			"MW%d size %pap exceeds BAR%d size %pap\n",
			widx, &size, bar, &bar_size);
		return -EINVAL;
	}

	mutex_lock(&ndev->cmd_lock);
	writeq(addr, ndev->cfg_reg + NXP_NTB_CFG_MW_ADDRESS);
	writeq(size, ndev->cfg_reg + NXP_NTB_CFG_MW_SIZE);
	ret = nxp_ntb_send_command_locked(ndev, NXP_NTB_CMD_CONFIGURE_MW, widx);
	mutex_unlock(&ndev->cmd_lock);

	return ret;
}

/*
 * Invalidate the endpoint's inbound translation for MW widx so the peer can
 * no longer DMA into the host buffer previously programmed with
 * mw_set_trans().
 */
static int nxp_ntb_mw_clear_trans(struct ntb_dev *ntb, int pidx, int widx)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	if (nxp_ntb_mw_to_bar(ndev, widx) < 0)
		return -EINVAL;

	return nxp_ntb_send_command(ndev, NXP_NTB_CMD_TEARDOWN_MW, widx);
}

static int nxp_ntb_peer_mw_get_addr(struct ntb_dev *ntb, int widx,
				    phys_addr_t *base, resource_size_t *size)
{
	struct nxp_ntb_dev *ndev = ntb_to_nxp(ntb);
	int bar;

	bar = nxp_ntb_mw_to_bar(ndev, widx);
	if (bar < 0)
		return bar;

	if (base)
		*base = pci_resource_start(ndev->ntb.pdev, bar);
	if (size)
		*size = pci_resource_len(ndev->ntb.pdev, bar);

	return 0;
}

static const struct ntb_dev_ops nxp_ntb_ops = {
	.link_is_up		= nxp_ntb_link_is_up,
	.link_enable		= nxp_ntb_link_enable,
	.link_disable		= nxp_ntb_link_disable,
	.db_valid_mask		= nxp_ntb_db_valid_mask,
	.db_vector_count	= nxp_ntb_db_vector_count,
	.db_vector_mask		= nxp_ntb_db_vector_mask,
	.db_read		= nxp_ntb_db_read,
	.db_clear		= nxp_ntb_db_clear,
	.db_set_mask		= nxp_ntb_db_set_mask,
	.db_clear_mask		= nxp_ntb_db_clear_mask,
	.peer_db_set		= nxp_ntb_peer_db_set,
	.msg_count		= nxp_ntb_msg_count,
	.msg_inbits		= nxp_ntb_msg_inbits,
	.msg_outbits		= nxp_ntb_msg_outbits,
	.msg_read_sts		= nxp_ntb_msg_read_sts,
	.msg_clear_sts		= nxp_ntb_msg_clear_sts,
	.msg_set_mask		= nxp_ntb_msg_set_mask,
	.msg_clear_mask		= nxp_ntb_msg_clear_mask,
	.msg_read		= nxp_ntb_msg_read,
	.peer_msg_write		= nxp_ntb_peer_msg_write,
	.mw_count		= nxp_ntb_mw_count,
	.peer_mw_count		= nxp_ntb_peer_mw_count,
	.mw_get_align		= nxp_ntb_mw_get_align,
	.mw_set_trans		= nxp_ntb_mw_set_trans,
	.mw_clear_trans		= nxp_ntb_mw_clear_trans,
	.peer_mw_get_addr	= nxp_ntb_peer_mw_get_addr,
};

static int nxp_ntb_init_pci(struct nxp_ntb_dev *ndev, struct pci_dev *pdev)
{
	struct device *dev = ndev->dev;
	void __iomem *bar0;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(dev, "failed to enable PCI device: %d\n", ret);
		return ret;
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(dev, "failed to set DMA mask: %d\n", ret);
			return ret;
		}
		dev_warn(dev, "DMA limited to 32-bit addressing\n");
	}

	bar0 = pcim_iomap_region(pdev, NXP_NTB_BAR_DB, KBUILD_MODNAME);
	if (IS_ERR(bar0)) {
		dev_err(dev, "failed to map BAR0 (doorbells): %ld\n",
			PTR_ERR(bar0));
		return PTR_ERR(bar0);
	}
	ndev->db_reg = bar0 + NXP_NTB_DB_BLOCK_OFFSET;

	ndev->cfg_reg = pcim_iomap_region(pdev, NXP_NTB_BAR_CFG, KBUILD_MODNAME);
	if (IS_ERR(ndev->cfg_reg)) {
		dev_err(dev, "failed to map BAR1 (config): %ld\n",
			PTR_ERR(ndev->cfg_reg));
		return PTR_ERR(ndev->cfg_reg);
	}

	pci_set_master(pdev);

	return 0;
}

static int nxp_ntb_init_dev(struct nxp_ntb_dev *ndev)
{
	struct pci_dev *pdev = ndev->ntb.pdev;
	struct device *dev = ndev->dev;
	u32 version;
	int bar;

	version = readl(ndev->cfg_reg + NXP_NTB_CFG_VERSION);
	dev_info(dev, "NTB config v%u\n", version);

	if (version != NXP_NTB_CFG_VERSION_V01)
		dev_warn(dev, "unexpected config version %u (expected %u)\n",
			 version, NXP_NTB_CFG_VERSION_V01);

	/* Build the MW index to BAR map by walking BAR2..BAR5 */
	for (bar = NXP_NTB_BAR_MW1; bar <= NXP_NTB_BAR_MW4; bar++) {
		if (!pci_resource_len(pdev, bar))
			continue;
		ndev->mw_bars[ndev->mw_count++] = bar;
	}

	if (!ndev->mw_count) {
		dev_err(dev, "no memory windows: BAR2..BAR5 all zero-length\n");
		return -ENODEV;
	}

	dev_info(dev, "discovered %d memory window(s)\n", ndev->mw_count);

	return 0;
}

/*
 * MSI-X layout (8 vectors total):
 *   Vector 0..4 : NTB message-register slots (DB0..DB4 OMR/IMR)
 *   Vector 5..6 : NTB doorbells (DB5..DB6 OMR/IMR)
 *   Vector 7    : Link events (LINK_UP / LINK_DOWN via DB7_IMR)
 */
static irqreturn_t nxp_ntb_vec_isr(int irq, void *data)
{
	struct nxp_ntb_dev *ndev = data;
	int vec;

	vec = irq - ndev->irq_base;
	if (vec < 0 || vec >= NXP_NTB_NUM_IRQS) {
		dev_warn_ratelimited(ndev->dev, "spurious IRQ on vector %d\n", vec);
		return IRQ_NONE;
	}

	if (vec == NXP_NTB_CMD_DB) {
		u32 evt = readl(ndev->db_reg + NXP_NTB_DB_IMR(NXP_NTB_CMD_DB));

		dev_dbg(ndev->dev, "link event: 0x%x\n", evt);

		/* W1C since there is no NTB consumer for this vector */
		writel(BIT(vec), ndev->db_reg + NXP_NTB_DB_DBISTAT);
		ntb_link_event(&ndev->ntb);
	} else if (vec >= NXP_NTB_DB_SLOT_BASE) {
		ntb_db_event(&ndev->ntb, vec - NXP_NTB_DB_SLOT_BASE);
	} else {
		ntb_msg_event(&ndev->ntb);
	}

	return IRQ_HANDLED;
}

static void nxp_ntb_free_irq_vectors(void *data)
{
	pci_free_irq_vectors(data);
}

static int nxp_ntb_init_isr(struct nxp_ntb_dev *ndev)
{
	struct pci_dev *pdev = ndev->ntb.pdev;
	struct device *dev = ndev->dev;
	int i, irq, ret;

	irq = pci_alloc_irq_vectors(pdev, NXP_NTB_NUM_IRQS, NXP_NTB_NUM_IRQS,
				    PCI_IRQ_MSIX);
	if (irq < 0) {
		dev_err(dev, "failed to allocate %d MSI-X vectors: %d\n",
			NXP_NTB_NUM_IRQS, irq);
		return irq;
	}

	/* Manage pci_free_irq_vectors() via devres */
	ret = devm_add_action_or_reset(dev, nxp_ntb_free_irq_vectors, pdev);
	if (ret)
		return ret;

	/* Cache the base vector so the ISR can derive its index cheaply */
	ndev->irq_base = pci_irq_vector(pdev, 0);

	/* Route DBn to MSI-X vector n via the CR0/CR1 vector map */
	writel(NXP_NTB_DB_CR0_DEFAULT, ndev->db_reg + NXP_NTB_DB_DB_CR0);
	writel(NXP_NTB_DB_CR1_DEFAULT, ndev->db_reg + NXP_NTB_DB_DB_CR1);

	/* clear DB status bits */
	writel(GENMASK(NXP_NTB_NUM_IRQS - 1, 0), ndev->db_reg + NXP_NTB_DB_DBISTAT);

	for (i = 0; i < NXP_NTB_NUM_IRQS; i++) {
		ret = devm_request_irq(dev, pci_irq_vector(pdev, i),
				       nxp_ntb_vec_isr, 0, KBUILD_MODNAME, ndev);
		if (ret) {
			dev_err(dev, "request irq for vec %d failed: %d\n", i, ret);
			return ret;
		}
	}

	return 0;
}

static int nxp_ntb_pci_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct nxp_ntb_dev *ndev;
	int ret;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	ndev = devm_kzalloc(dev, sizeof(*ndev), GFP_KERNEL);
	if (!ndev)
		return -ENOMEM;

	ndev->dev = dev;
	ndev->ntb.pdev = pdev;
	ndev->ntb.topo = NTB_TOPO_NONE;
	ndev->ntb.ops = &nxp_ntb_ops;
	pci_set_drvdata(pdev, ndev);

	ret = devm_mutex_init(dev, &ndev->cmd_lock);
	if (ret)
		return ret;

	ret = nxp_ntb_init_pci(ndev, pdev);
	if (ret)
		return ret;

	ret = nxp_ntb_init_dev(ndev);
	if (ret)
		return ret;

	ret = nxp_ntb_init_isr(ndev);
	if (ret)
		return ret;

	ret = ntb_register_device(&ndev->ntb);
	if (ret) {
		dev_err(dev, "failed to register NTB device: %d\n", ret);
		return ret;
	}

	return 0;
}

static void nxp_ntb_pci_remove(struct pci_dev *pdev)
{
	struct nxp_ntb_dev *ndev = pci_get_drvdata(pdev);

	nxp_ntb_link_disable(&ndev->ntb);

	ntb_unregister_device(&ndev->ntb);
}

static const struct pci_device_id nxp_ntb_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_PHILIPS, NXP_NTB_DEVICE_ID) },
	{ }
};
MODULE_DEVICE_TABLE(pci, nxp_ntb_pci_tbl);

static struct pci_driver nxp_ntb_pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= nxp_ntb_pci_tbl,
	.probe		= nxp_ntb_pci_probe,
	.remove		= nxp_ntb_pci_remove,
};
module_pci_driver(nxp_ntb_pci_driver);

MODULE_DESCRIPTION("NXP S32N79 PCIe NTB host driver");
MODULE_AUTHOR("Valentin Ciocoi Radulescu <valentin.ciocoi@nxp.com>");
MODULE_LICENSE("Dual BSD/GPL");
