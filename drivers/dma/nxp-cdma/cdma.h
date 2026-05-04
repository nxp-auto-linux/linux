/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * NXP S32N79 Central DMA (CDMA) Controller Driver
 *
 * Copyright 2026 NXP
 */

#ifndef NXP_CDMA_H
#define NXP_CDMA_H

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/scatterlist.h>
#include <linux/types.h>
#include <linux/wait.h>

/* PCI Identification */
#define CDMA_PCI_VENDOR_ID	PCI_VENDOR_ID_PHILIPS	/* 0x1131, NXP */
#define CDMA_PCI_DEVICE_ID	0xe200

/* Station Interface register offsets (within 128 KB EA aperture) */
#define CDMA_SIMR		0x0000	/* SI Mode Register */
#define CDMA_SISR		0x0004	/* SI Status Register */
#define CDMA_SICAPR0		0x0100	/* SI Capability Register 0 */
#define CDMA_SICAPR1		0x0104	/* SI Capability Register 1 */
#define CDMA_SITCR		0x0200	/* SI Throttle Control */
#define CDMA_SICAR0		0x0300	/* SI Cache Attribute 0 */
#define CDMA_SICAR1		0x0304	/* SI Cache Attribute 1 */
#define CDMA_SIQOSR		0x030C	/* SI QoS Register */
#define CDMA_SICASR(n)		(0x0310 + (n) * 8) /* Cache Attr Select 0-7 */

#define CDMA_SIMR_EN		BIT(31)
#define CDMA_SISR_BUSY		BIT(0)

#define CDMA_SICAPR1_NUM_BDR_MASK	GENMASK(4, 0)
#define CDMA_SICAPR1_NUM_MSIX_MASK	GENMASK(12, 8)
#define CDMA_SICAPR1_NUM_MSIX_SHIFT	8

/* BD Ring register offsets (within SI, per ring) */
#define CDMA_BDR_BASE		0x8000
#define CDMA_BDR_STRIDE		0x0100	/* 256 bytes per ring */

/* Offsets within one ring's register block */
#define CDMA_BaMR		0x00	/* BD Ring Mode Register */
#define CDMA_BaSR		0x04	/* BD Ring Status Register */
#define CDMA_BaBAR0		0x10	/* BD Ring Base Address Low */
#define CDMA_BaBAR1		0x14	/* BD Ring Base Address High */
#define CDMA_BaPIR		0x18	/* BD Ring Producer Index */
#define CDMA_BaCIR		0x1C	/* BD Ring Consumer Index */
#define CDMA_BaLENR		0x20	/* BD Ring Length */
#define CDMA_BaIER		0x40	/* BD Ring Interrupt Enable */
#define CDMA_BaIDR		0x44	/* BD Ring Interrupt Detect */
#define CDMA_BaIVR		0x48	/* BD Ring Interrupt Vector */
#define CDMA_BaICR0		0x50	/* BD Ring Interrupt Coalescing 0 */
#define CDMA_BaICR1		0x54	/* BD Ring Interrupt Coalescing 1 */

/*
 * LENGTH field is bits [16:3], in units of 8 BDs.
 * To program N BD entries: write ((N / 8) << 3) to BaLENR.
 * Ring can hold LENGTH-1 usable BDs (PIR==CIR means empty).
 */
#define CDMA_BaLENR_LENGTH_SHIFT	3
#define CDMA_BaLENR_LENGTH(n)		(((n) / 8) << CDMA_BaLENR_LENGTH_SHIFT)

#define CDMA_BaMR_EN		BIT(31)	/* Ring enable */
#define CDMA_BaMR_SM_MASK	GENMASK(15, 14)	/* Start mode */
#define CDMA_BaMR_EM		BIT(13)	/* Execution mode */
#define CDMA_BaMR_WBM		BIT(12)	/* Writeback mode */
#define CDMA_BaMR_WWC_MASK	GENMASK(11, 10)	/* Writeback write control */
#define CDMA_BaMR_WRR_MASK	GENMASK(6, 4)	/* WRR weight */
#define CDMA_BaMR_IPV_MASK	GENMASK(3, 0)	/* Internal priority value */

#define CDMA_BaSR_BUSY		BIT(0)
#define CDMA_BaSR_OP_XOFF	BIT(1)
#define CDMA_BaSR_PE		BIT(16)	/* Programming error */
#define CDMA_BaSR_SBE		BIT(17)	/* System bus error */
#define CDMA_BaSR_ME		BIT(18)	/* Memory error */
#define CDMA_BaSR_ERR_MASK	(CDMA_BaSR_PE | CDMA_BaSR_SBE | CDMA_BaSR_ME)

#define CDMA_BaIER_CIE		BIT(0)	/* Completion interrupt enable */
#define CDMA_BaIER_TIE		BIT(1)	/* Threshold interrupt enable */

#define CDMA_SIIDR		0xD000
/* Bits [15:0] = completion for ring 0-15 (CIn), [31:16] = threshold (TIn) */
#define CDMA_SIIDR_C(ring)	BIT(ring)	/* Completion for ring */
#define CDMA_SIIDR_T(ring)	BIT(16 + (ring))/* Threshold for ring */

#define CDMA_SIPESR		0xE004	/* SI Programming Error Status */
#define CDMA_SICMECR		0xE010	/* Correctable Memory Error Config */
#define CDMA_SICMESR		0xE014	/* Correctable Memory Error Status */
#define CDMA_SIUNSBECR		0xE020	/* Uncorr Non-Fatal SBE Config */
#define CDMA_SIUNSBESR0		0xE024	/* Uncorr Non-Fatal SBE Status 0 */

/* Global (shared read-only) */
#define CDMA_CCLKCR		0xF000	/* Clock Configuration Register */

/* Hardware descriptor format (16 bytes each, little-endian) */

/* Source Descriptor (SD) */
struct cdma_sd {
	__le64	addr;		/* DATA_ADDR / SG_ADDR / RCV_PIPE_ID_VALUE */
	__le32	length;		/* Transfer length */
	__le32	config;		/* TYPE | FMT | CCS | TSIZE | flags */
} __packed;

/* Destination Descriptor (DD) */
struct cdma_dd {
	__le64	addr;		/* DATA_ADDR / SG_ADDR / pipe IDs + tag */
	__le32	length;		/* Transfer length */
	__le32	config;		/* TYPE | FMT | CCS | TSIZE | flags */
} __packed;

/* Writeback Descriptor (overwrites SD on completion when WBM=1) */
struct cdma_wb {
	__le32	rsvd;
	__le32	rcv_tag;	/* Pipe receiver tag (from sender) */
	__le32	length;		/* Actual bytes transferred */
	__le32	status;		/* W | DE | SPE | DPE */
} __packed;

/* S/G Entry Descriptor */
struct cdma_sge {
	__le64	addr;		/* Data buffer or next SG table address */
	__le32	length;		/* Buffer length */
	__le32	config;		/* TYPE | FMT | CCS | F */
} __packed;

/* SD/DD config word field macros */
#define CDMA_DESC_TYPE_SHIFT	28
#define CDMA_DESC_TYPE_MASK	GENMASK(31, 28)
#define CDMA_DESC_FMT_EXT	BIT(27)
#define CDMA_DESC_CCS_SHIFT	24
#define CDMA_DESC_CCS_MASK	GENMASK(26, 24)
#define CDMA_DESC_TSIZE_SHIFT	20
#define CDMA_DESC_TSIZE_MASK	GENMASK(23, 20)
/* 2^8 = 256-byte AXI beats (CAPR1 MAX_TSIZE) */
#define CDMA_DESC_TSIZE_DEFAULT	8
#define CDMA_DESC_THROTTLE_SHIFT 16
#define CDMA_DESC_THROTTLE_MASK	GENMASK(19, 16)
#define CDMA_DESC_FINAL		BIT(11)
#define CDMA_DESC_CI		BIT(10)	/* Completion interrupt */
#define CDMA_DESC_SO		BIT(9)	/* Strict ordering */
#define CDMA_DESC_WB		BIT(3)	/* Writeback (inverted on WB) */

/* DD-specific fields */
#define CDMA_DD_NPR_SHIFT	24	/* Number of pipe receivers (NPR) */
#define CDMA_DD_NPR_MASK	GENMASK(25, 24)
#define CDMA_DD_LWC_SHIFT	9	/* Last write control */
#define CDMA_DD_LWC_MASK	GENMASK(10, 9)

#define CDMA_BaCIR_INDEX_MASK	GENMASK(15, 0)	/* BD ring consumer index */
#define CDMA_BaCIR_ERR_STAT	BIT(16)		/* Error status (read-to-clear) */

/* Writeback status bits */
#define CDMA_WB_W		BIT(3)	/* Writeback indicator (inverted) */
#define CDMA_WB_DE		BIT(2)	/* Data error */
#define CDMA_WB_SPE		BIT(1)	/* Source programming error */
#define CDMA_WB_DPE		BIT(0)	/* Destination programming error */

/* Descriptor TYPE encodings */
#define CDMA_TYPE_DATA		0x0	/* Data buffer, singlecast */
#define CDMA_TYPE_SG_FULL	0x2	/* S/G table link, full length */
#define CDMA_TYPE_PIPE		0x3	/* Pipe send (DD) / receive (SD) */
#define CDMA_TYPE_SG_SHORT	0x6	/* S/G table link, short length */
#define CDMA_TYPE_DATA_MC	0x8	/* Data buffer, multicast */
#define CDMA_TYPE_SG_MC_FULL	0xA	/* S/G table link, multicast, full */
#define CDMA_TYPE_SG_MC_SHORT	0xE	/* S/G table link, multicast, short */

/* Driver constants */
#define CDMA_MAX_SIS		48
#define CDMA_MAX_RINGS_PER_SI	16
#define CDMA_BD_ENTRIES		256	/* BD entries per ring (must be multiple of 8) */
#define CDMA_BD_ENTRY_SIZE	16	/* Each BD entry is 16 bytes */
#define CDMA_BD_RING_SIZE	(CDMA_BD_ENTRIES * CDMA_BD_ENTRY_SIZE)
#define CDMA_BD_ALIGN		64	/* HW requires 64-byte alignment */
/*
 * Max zero-copy transfer size. The bound (CDMA_SG_MAX_ENTRIES - 1) * PAGE_SIZE
 * guarantees that any user buffer of this size — regardless of in-page offset —
 * fits within CDMA_SG_MAX_ENTRIES SGEs (worst case: offset = PAGE_SIZE - 1
 * splits the last byte into an extra page).
 *
 * On 4 KiB pages: 1023 * 4096 = 4190208 bytes (= 4 MiB - 4 KiB).
 */
#define CDMA_PIPE_BUF_SIZE	((CDMA_SG_MAX_ENTRIES - 1) * PAGE_SIZE)
#define CDMA_BD_PAIR		2	/* Every transfer is one SD + one DD */
#define CDMA_POLL_TIMEOUT_US	1000000	/* 1s for register polling (wait_idle) */
#define CDMA_COMP_TIMEOUT_US	5000000	/* 5s for IRQ completion (threaded handler) */
#define CDMA_DRV_NAME		"nxp-cdma"

/* S/G table constants */
#define CDMA_SG_MAX_ENTRIES	1024	/* Max SGE per table (HW spec ceiling, range 4..1024) */
#define CDMA_SGE_SIZE		16	/* Each SGE is 16 bytes */
#define CDMA_SGT_SIZE		(CDMA_SG_MAX_ENTRIES * CDMA_SGE_SIZE) /* 16 KiB */
#define CDMA_SGT_ALIGN		16	/* HW requires 16-byte alignment */

/* BD Ring */
struct cdma_ring {
	struct cdma_si *si;
	void __iomem *regs;	/* Ring register base */
	u32 idx;	/* Ring index within SI */

	/* DMA-coherent BD ring memory */
	void *bd_base;	/* CPU virtual address */
	dma_addr_t bd_dma;	/* DMA/bus address */

	/* Producer-consumer ring state */
	/* Next producer index (where to write BDs) */
	u16 pir;
	/* SD index of last submitted transfer (writeback location) */
	u16 wb_idx;

	/* Completion tracking */
	struct completion done;
	int result;

	/* Pipe association */
	struct cdma_pipe *pipe;
};

/* Station Interface */
struct cdma_si {
	struct cdma_device	*cdev;
	void __iomem *regs; /* SI register base */
	u32 num_rings;	/* From SICAPR1 */
	u32 num_msix;	/* From SICAPR1 */
	int irq;		/* MSI-X IRQ (shared) */
	struct cdma_ring rings[]; /* Flex array */
};

/*
 * Each SI provides both TX (ring 0, sender BDs) and RX (ring 1,
 * receiver BDs). TX and RX operate fully independently — no shared
 * state, no reader-first coordination. Cross-SI rendezvous is done
 * by hardware via IERB-configured pipe IDs (TYPE=PIPE descriptors).
 */
struct cdma_pipe {
	struct cdma_device *cdev;
	u8 hw_pipe_id;			/* IERB pipe ID, from DT */
	u8 pipe_idx;			/* For char-device naming only */
	char name_tx[32];		/* "cdma-pipe0-tx" */
	char name_rx[32];		/* "cdma-pipe0-rx" */

	struct cdma_ring *tx_ring;	/* Local SI ring 0 — sender BDs */
	struct cdma_ring *rx_ring;	/* Local SI ring 1 — receiver BDs */
	size_t max_xfer;

	/* TX state (independent of RX) */
	struct mutex tx_lock;
	struct miscdevice miscdev_tx;	/* /dev/cdma-pipeN-tx */

	/* RX state (independent of TX) */
	struct mutex rx_lock;
	struct miscdevice miscdev_rx;	/* /dev/cdma-pipeN-rx */
};

/* Top-level device */
struct cdma_device {
	struct pci_dev *pdev;
	struct cdma_si *si;	/* Single SI for now */

	/* S/G table DMA pool */
	struct dma_pool *sgt_pool;

	/* DT-configured pipe ID */
	u32 hw_pipe_id;		/* IERB pipe ID for this SI */
	bool has_pipe_id;	/* true if nxp,pipe-id was found in DT */

	/* Pipe devices */
	struct cdma_pipe *pipes;
	u32 num_pipes;
};

/* Register access helpers */
static inline u32 cdma_si_read(struct cdma_si *si, u32 offset)
{
	return ioread32(si->regs + offset);
}

static inline void cdma_si_write(struct cdma_si *si, u32 offset, u32 val)
{
	iowrite32(val, si->regs + offset);
}

static inline void __iomem *cdma_ring_reg_base(struct cdma_si *si, u32 ring)
{
	return si->regs + CDMA_BDR_BASE + ring * CDMA_BDR_STRIDE;
}

static inline u32 cdma_ring_read(struct cdma_ring *ring, u32 offset)
{
	return ioread32(ring->regs + offset);
}

static inline void cdma_ring_write(struct cdma_ring *ring, u32 offset, u32 val)
{
	iowrite32(val, ring->regs + offset);
}

/* cdma-ring.c */
int cdma_ring_alloc(struct cdma_ring *ring);
void cdma_ring_free(struct cdma_ring *ring);
int cdma_ring_init(struct cdma_ring *ring);
void cdma_ring_reset(struct cdma_ring *ring);
void cdma_ring_cancel(struct cdma_ring *ring);
int cdma_ring_submit(struct cdma_ring *ring, u32 num_bds);
int cdma_build_sg_table(struct cdma_device *cdev, struct sg_table *sgt,
			dma_addr_t *sgt_dma_out, void **sgt_cpu_out);
void cdma_free_sg_table(struct cdma_device *cdev, void *sgt_cpu,
			dma_addr_t sgt_dma);
void cdma_build_sg_pipe_sender(struct cdma_ring *ring,
			       dma_addr_t src_sgt, u32 src_len,
			       u8 pipe_id, u32 tag);
void cdma_build_sg_pipe_receiver(struct cdma_ring *ring,
				 dma_addr_t dst_sgt, u32 dst_len,
				 u8 pipe_id);

/* cdma-cdev.c */
int cdma_cdev_init(struct cdma_device *cdev);
void cdma_cdev_fini(struct cdma_device *cdev);

#endif /* NXP_CDMA_H */
