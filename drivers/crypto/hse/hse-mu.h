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

#define HSE_ANY_CHANNEL        0xACu
#define HSE_INVALID_CHANNEL    0xFFu

void *hse_mu_init(struct device *dev);

int hse_mu_reserve_channel(void *mu_inst, u8 *channel);

int hse_mu_send_request(void *mu_inst, u8 channel, u32 srv_desc, void *ctx,
			void (*rx_cbk)(void *mu_inst, u8 channel, void *ctx));

int hse_mu_recv_response(void *mu_inst, u8 channel, u32 *srv_rsp);

int hse_mu_release_channel(void *mu_inst, u8 channel);

void hse_mu_free(void *mu_inst);

#endif /* HSE_MU_H */
