// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Management Interface (SCMI) Message SMC/HVC
 * Transport driver
 *
 * Copyright 2020,2022-2023 NXP
 */

#include <linux/arm-smccc.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/processor.h>
#include <linux/slab.h>

#include "common.h"

/**
 * struct scmi_smc - Structure representing a SCMI smc transport
 *
 * @cinfo: SCMI channel info
 * @notif_cinfo: SCMI notification channel info
 * @shmem: Transmit/Receive shared memory area
 * @notif_shmem: Notification shared memory area
 * @shmem_lock: Lock to protect access to Tx/Rx shared memory area.
 *		Used when NOT operating in atomic mode.
 * @spinlock: Lock to protect access to Tx/Rx shared memory area.
 *	      Used when operating in atomic mode.
 * @spin_flags: Lock flags
 * @func_id: smc/hvc call function id
 * @node: Linked list pointers
 * @tx_no_completion_irq: Tx channel has no completion interrupt
 *                        mechanism for synchronous commands.
 */

struct scmi_smc {
	struct scmi_chan_info *cinfo;
	struct scmi_chan_info *notif_cinfo;
	struct scmi_shared_mem __iomem *shmem;
	struct scmi_shared_mem __iomem *notif_shmem;
	/* Protect access to shmem area */
	struct mutex shmem_lock;
	struct list_head node;
	/* Protect access to shmem area in atomic context */
	spinlock_t spinlock;
	unsigned long spin_flags;
	u32 func_id;
	bool tx_no_completion_irq;
};

static LIST_HEAD(scmi_smc_devices);
static DEFINE_MUTEX(smc_devices_lock);

static irqreturn_t smc_msg_done_isr(int irq, void *data)
{
	struct scmi_smc *scmi_info = data;

	scmi_rx_callback(scmi_info->cinfo,
			 shmem_read_header(scmi_info->shmem), NULL);

	return IRQ_HANDLED;
}

static irqreturn_t smc_platform_notif_isr(int irq, void *data)
{
	struct scmi_smc *scmi_info = data;

	scmi_rx_callback(scmi_info->notif_cinfo,
			 shmem_read_header(scmi_info->notif_shmem), NULL);

	return IRQ_HANDLED;
}

static bool smc_chan_available(struct device *dev, int idx)
{
	struct device_node *np = of_parse_phandle(dev->of_node, "shmem", 0);
	if (!np)
		return false;

	of_node_put(np);
	return true;
}

static inline void smc_channel_lock_init(struct scmi_smc *scmi_info)
{
	if (IS_ENABLED(CONFIG_ARM_SCMI_TRANSPORT_SMC_ATOMIC_ENABLE))
		spin_lock_init(&scmi_info->spinlock);
	else
		mutex_init(&scmi_info->shmem_lock);
}

static inline int
smc_channel_try_lock_acquire(struct scmi_smc *scmi_info)
{
	unsigned long flags;
	int ret;

	if (IS_ENABLED(CONFIG_ARM_SCMI_TRANSPORT_SMC_ATOMIC_ENABLE)) {
		ret = spin_trylock_irqsave(&scmi_info->spinlock,
					   flags);
		if (ret)
			scmi_info->spin_flags = flags;
		return ret;
	}

	return mutex_trylock(&scmi_info->shmem_lock);
}

static inline void smc_channel_lock_release(struct scmi_smc *scmi_info)
{
	if (IS_ENABLED(CONFIG_ARM_SCMI_TRANSPORT_SMC_ATOMIC_ENABLE))
		spin_unlock_irqrestore(&scmi_info->spinlock,
				       scmi_info->spin_flags);
	else
		mutex_unlock(&scmi_info->shmem_lock);
}

static struct scmi_smc *create_scmi_smc_dev(struct device *cdev,
					    struct device *dev)
{
	struct scmi_smc *scmi_info;
	u32 func_id;
	int ret, irq;

	scmi_info = devm_kzalloc(dev, sizeof(*scmi_info), GFP_KERNEL);
	if (!scmi_info)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(dev->of_node, "arm,smc-id", &func_id);
	if (ret < 0)
		return ERR_PTR(ret);

	/*
	 * If there is an interrupt named "a2p", then the service and
	 * completion of a message is signaled by an interrupt rather than by
	 * the return of the SMC call.
	 */
	irq = of_irq_get_byname(cdev->of_node, "a2p");
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, smc_msg_done_isr,
				       IRQF_NO_SUSPEND,
				       dev_name(dev), scmi_info);
		if (ret) {
			dev_err(dev, "failed to setup SCMI smc irq\n");
			return ERR_PTR(ret);
		}
	} else {
		scmi_info->tx_no_completion_irq = true;
	}

	irq = of_irq_get_byname(cdev->of_node, "p2a_notif");
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, smc_platform_notif_isr,
				       IRQF_NO_SUSPEND,
				       dev_name(dev), scmi_info);
		if (ret) {
			dev_err(dev, "failed to setup SCMI smc notification irq\n");
			return ERR_PTR(ret);
		}
	}

	scmi_info->func_id = func_id;
	smc_channel_lock_init(scmi_info);

	return scmi_info;
}

static struct scmi_smc *get_scmi_smc_dev(struct scmi_chan_info *cinfo,
					 struct device *dev)
{
	struct scmi_smc *smc_dev = NULL;
	bool found = false;

	mutex_lock(&smc_devices_lock);
	list_for_each_entry(smc_dev, &scmi_smc_devices, node) {
		if (smc_dev->cinfo->dev == cinfo->dev) {
			found = true;
			break;
		}
	}

	if (found)
		goto release_lock;

	smc_dev = create_scmi_smc_dev(cinfo->dev, dev);
	if (!IS_ERR(smc_dev))
		list_add(&smc_dev->node, &scmi_smc_devices);

release_lock:
	mutex_unlock(&smc_devices_lock);
	return smc_dev;
}

static int smc_chan_setup(struct scmi_chan_info *cinfo, struct device *dev,
			  bool tx)
{
	struct device *cdev = cinfo->dev;
	struct scmi_smc *scmi_info;
	resource_size_t size;
	struct resource res;
	struct device_node *np;
	int ret, shmem_idx;
	const char *ch_name = tx ? "tx" : "rx";
	struct scmi_shared_mem __iomem **shmem;

	shmem_idx = of_property_match_string(cdev->of_node,
					     "mbox-names",
					     ch_name);
	if (shmem_idx < 0)
		return -ENXIO;

	scmi_info = get_scmi_smc_dev(cinfo, dev);
	if (IS_ERR(scmi_info))
		return PTR_ERR(scmi_info);

	np = of_parse_phandle(cdev->of_node, "shmem", shmem_idx);
	if (!of_device_is_compatible(np, "arm,scmi-shmem")) {
		of_node_put(np);
		return -ENXIO;
	}

	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret) {
		dev_err(cdev, "failed to get SCMI %s shared memory\n", ch_name);
		return ret;
	}

	if (tx)
		shmem = &scmi_info->shmem;
	else
		shmem = &scmi_info->notif_shmem;

	size = resource_size(&res);
	*shmem = devm_ioremap(dev, res.start, size);
	if (!*shmem) {
		dev_err(dev, "failed to ioremap SCMI %s shared memory\n",
			ch_name);
		return -EADDRNOTAVAIL;
	}

	if (tx) {
		scmi_info->cinfo = cinfo;
		cinfo->no_completion_irq = scmi_info->tx_no_completion_irq;
	} else {
		scmi_info->notif_cinfo = cinfo;
	}

	cinfo->transport_info = scmi_info;

	return 0;
}

static int smc_chan_free(int id, void *p, void *data)
{
	struct scmi_chan_info *cinfo = p;
	struct scmi_smc *scmi_info = cinfo->transport_info;

	cinfo->transport_info = NULL;
	scmi_info->cinfo = NULL;

	scmi_free_channel(cinfo, data, id);

	return 0;
}

static int smc_send_message(struct scmi_chan_info *cinfo,
			    struct scmi_xfer *xfer)
{
	struct scmi_smc *scmi_info = cinfo->transport_info;
	struct arm_smccc_res res;
	int ret = 0;

	/*
	 * Channel will be released only once response has been
	 * surely fully retrieved, so after .mark_txdone()
	 */
	while (!ret) {
		spin_until_cond(shmem_is_free(scmi_info->shmem));
		ret = smc_channel_try_lock_acquire(scmi_info);
	}

	shmem_tx_prepare(scmi_info->shmem, xfer);

	arm_smccc_1_1_invoke(scmi_info->func_id, 0, 0, 0, 0, 0, 0, 0, &res);

	/* Only SMCCC_RET_NOT_SUPPORTED is valid error code */
	if (res.a0) {
		smc_channel_lock_release(scmi_info);
		return -EOPNOTSUPP;
	}

	return 0;
}

static void smc_fetch_response(struct scmi_chan_info *cinfo,
			       struct scmi_xfer *xfer)
{
	struct scmi_smc *scmi_info = cinfo->transport_info;

	shmem_fetch_response(scmi_info->shmem, xfer);
}

static void smc_mark_txdone(struct scmi_chan_info *cinfo, int ret)
{
	struct scmi_smc *scmi_info = cinfo->transport_info;

	smc_channel_lock_release(scmi_info);
}

static void smc_fetch_notification(struct scmi_chan_info *cinfo,
				   size_t max_len, struct scmi_xfer *xfer)
{
	struct scmi_smc *scmi_info = cinfo->transport_info;

	shmem_fetch_notification(scmi_info->notif_shmem, max_len, xfer);
}

static const struct scmi_transport_ops scmi_smc_ops = {
	.chan_available = smc_chan_available,
	.chan_setup = smc_chan_setup,
	.chan_free = smc_chan_free,
	.send_message = smc_send_message,
	.mark_txdone = smc_mark_txdone,
	.fetch_response = smc_fetch_response,
	.fetch_notification = smc_fetch_notification,
};

const struct scmi_desc scmi_smc_desc = {
	.ops = &scmi_smc_ops,
	.max_rx_timeout_ms = 30,
	.max_msg = 20,
	.max_msg_size = 128,
	/*
	 * Setting .sync_cmds_atomic_replies to true for SMC assumes that,
	 * once the SMC instruction has completed successfully, the issued
	 * SCMI command would have been already fully processed by the SCMI
	 * platform firmware and so any possible response value expected
	 * for the issued command will be immmediately ready to be fetched
	 * from the shared memory area.
	 */
	.sync_cmds_completed_on_ret = true,
	.atomic_enabled = IS_ENABLED(CONFIG_ARM_SCMI_TRANSPORT_SMC_ATOMIC_ENABLE),
};
