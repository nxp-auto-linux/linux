/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - Messaging Unit Interface
 *
 * This file defines the interface specification for the Messaging Unit
 * instance used by host application cores to communicate with HSE.
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_MU_H
#define HSE_MU_H

/**
 * enum hse_irq_type - HSE MU interrupt type
 * @HSE_INT_ACK_REQUEST: TX Interrupt, triggered when HSE acknowledged the
 *                       service request and released the service channel
 * @HSE_INT_RESPONSE: RX Interrupt, triggered when HSE wrote the response
 * @HSE_INT_SYS_EVENT: General Purpose Interrupt, triggered when HSE sends
 *                     a system event, generally an error notification
 */
enum hse_irq_type {
	HSE_INT_ACK_REQUEST = 0x00u,
	HSE_INT_RESPONSE    = 0x01u,
	HSE_INT_SYS_EVENT   = 0x02u,
};

#define HSE_NUM_CHANNELS    16u
#define HSE_ALL_CHANNELS    0x0000FFFFul

#define HSE_INVALID_CHANNEL    0xFFu

/* MU Access Functions */

void *hse_mu_init(struct device *dev);

u8 hse_mu_next_free_channel(void *mu_inst);

int hse_mu_send_request(void *mu_inst, u8 channel, u32 srv_desc, void *ctx,
			void (*rx_cbk)(void *mu_inst, u8 channel, void *ctx));

u32 hse_mu_recv_response(void *mu_inst, u8 channel);

void hse_mu_free(void *mu_inst);

#endif /* HSE_MU_H */
