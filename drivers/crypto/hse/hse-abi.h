/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - HSE Binary Interface
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_ABI_H
#define HSE_ABI_H

/**
 * enum hse_srv_id - HSE service ID
 * @HSE_SRV_ID_HASH: perform a hash operation
 */
enum hse_srv_id {
	HSE_SRV_ID_HASH = 0x00000200ul,
};

/**
 * enum hse_srv_response - HSE service response
 * @HSE_SRV_RSP_OK: HSE service successfully executed with no error
 * @HSE_SRV_RSP_INVALID_ADDR: address parameters are invalid
 * @HSE_SRV_RSP_INVALID_PARAM: HSE request parameters are invalid
 * @HSE_SRV_RSP_NOT_SUPPORTED: operation or feature is not supported
 * @HSE_SRV_RSP_NOT_ALLOWED: operation not allowed because of some restrictions
 *                           (in attributes, life-cycle dependent operations,
 *                           key-management, etc.)
 * @HSE_SRV_RSP_NOT_ENOUGH_SPACE: not enough space to perform the service
 * @HSE_SRV_RSP_READ_FAILURE: service request failed, read access denied
 * @HSE_SRV_RSP_WRITE_FAILURE: service request failed, write access denied
 * @HSE_SRV_RSP_STREAMING_MODE_FAILURE: service request in streaming mode failed
 * @HSE_SRV_RSP_MEMORY_FAILURE: physical errors (e.g. flipped bits) detected
 *                              during memory read or write
 * @HSE_SRV_RSP_CANCEL_FAILURE: service cannot be canceled
 * @HSE_SRV_RSP_CANCELED: service has been canceled
 * @HSE_SRV_RSP_GENERAL_ERROR: returned if an error not covered by the error
 *                             codes above is detected inside HSE
 *
 * The Service response is provided by MUB_RRx register after the service
 * execution.
 */
enum hse_srv_response {
	HSE_SRV_RSP_OK = 0x55A5AA33ul,
	HSE_SRV_RSP_INVALID_ADDR = 0x55A5AA55ul,
	HSE_SRV_RSP_INVALID_PARAM = 0x55A5AA56ul,
	HSE_SRV_RSP_NOT_SUPPORTED = 0xAA55A569ul,
	HSE_SRV_RSP_NOT_ALLOWED = 0xAA55A536ul,
	HSE_SRV_RSP_NOT_ENOUGH_SPACE = 0xAA55A563ul,
	HSE_SRV_RSP_READ_FAILURE = 0xAA55A599ul,
	HSE_SRV_RSP_WRITE_FAILURE = 0xAA55A5B1ul,
	HSE_SRV_RSP_STREAMING_MODE_FAILURE = 0xAA55A5C1ul,
	HSE_SRV_RSP_MEMORY_FAILURE = 0x55A5AA36ul,
	HSE_SRV_RSP_CANCEL_FAILURE = 0x55A5C461ul,
	HSE_SRV_RSP_CANCELED = 0x55A5C596ul,
	HSE_SRV_RSP_GENERAL_ERROR = 0x55A5C565ul,
};

/**
 * enum hse_srv_prio - HSE service request priority
 * @HSE_SRV_PRIO_LOW: low priority
 * @HSE_SRV_PRIO_MED: medium priority
 * @HSE_SRV_PRIO_HIGH: high priority
 */
enum hse_srv_prio {
	HSE_SRV_PRIO_LOW = 0u,
	HSE_SRV_PRIO_MED = 1u,
	HSE_SRV_PRIO_HIGH = 2u,
};

/**
 * enum hse_srv_access_modes - HSE access modes
 * @HSE_ACCESS_MODE_ONE_PASS: ONE-PASS access mode
 * @HSE_ACCESS_MODE_START: START access mode
 * @HSE_ACCESS_MODE_UPDATE: UPDATE access mode
 * @HSE_ACCESS_MODE_FINISH: FINISH access mode
 */
enum hse_srv_access_modes {
	HSE_ACCESS_MODE_ONE_PASS = 0u,
	HSE_ACCESS_MODE_START = 1u,
	HSE_ACCESS_MODE_UPDATE = 2u,
	HSE_ACCESS_MODE_FINISH = 3u,
};

/**
 * enum hse_hash_algorithms - supported HASH algorithm types
 * @HSE_HASH_ALGO_NULL: none
 * @HSE_HASH_ALGO_MD5: MD5 hash
 * @HSE_HASH_ALGO_SHA_1: SHA1 hash
 */
enum hse_hash_algorithms {
	HSE_HASH_ALGO_NULL = 0u,
	HSE_HASH_ALGO_MD5 = 1u,
	HSE_HASH_ALGO_SHA_1 = 2u,
};

/**
 * struct hse_hash_srv - HASH service
 * @access_mode: ONE-PASS, START, UPDATE, FINISH
 * @stream_id: stream to use for START, UPDATE, FINISH access modes - there is a
 *             limited number of streams per interface, up to HSE_STREAM_COUNT.
 * @hash_algo: hash algorithm to be used
 * @input_len: length of the input message - must be an integer multiple of
 *             algorithm block size for START and UPDATE, can be zero for START,
 *             and FINISH step
 * @p_input: address of the input message - mandatory for UPDATE, can be NULL
 *           for START and FINISH step
 * @p_hash_len: holds the address to a u32 location in which the hash length
 *              in bytes is stored. On calling this service, this parameter
 *              shall contain the size of the buffer provided by host. When the
 *              request has finished, the actual length of the returned value
 *              shall be stored. If the buffer is smaller than the size of the
 *              hash, the hash will be truncated. If the buffer is larger,
 *              *p_hash_len is adjusted to the size of the hash. The input
 *              hash length shall not be zero for ONE-PASS or FINISH steps
 * @p_hash: the address where output hash will be stored
 *
 * The HASH service can be accessible in one-pass or streaming (SUF) mode. In
 * case of streaming mode, three steps (calls) are needed: START, UPDATE,
 * FINISH. For each streaming step, the fields that are not mandatory shall be
 * set NULL or 0.
 *
 * The table below summarizes which fields are used by each access mode.
 * Unless stated otherwise, the unused fields shall be NULL or 0.
 *
 * | Field \ Mode | One-pass | Start | Update | Finish |
 * |--------------+----------+-------+--------+--------|
 * | access_mode  |     *    |   *   |    *   |    *   |
 * | stream_id    |          |   *   |    *   |    *   |
 * | hash_algo    |     *    |   *   |    *   |    *   |
 * | input_len    |     *    |   *   |    *   |    *   |
 * | p_input      |     *    |   *   |    *   |    *   |
 * | p_hash_len   |     *    |       |        |    *   |
 * | p_hash       |     *    |       |        |    *   |
 */
struct hse_hash_srv {
	u8 access_mode;
	u8 stream_id;
	u8 hash_algo;
	u8 reserved;
	u32 input_len;
	u64 p_input;
	u64 p_hash_len;
	u64 p_hash;
};

/**
 * struct hse_cancel_srv - cancel a HSE service
 * @channel: the MU channel index on which the service was sent
 *
 * Cancel a HSE one-pass or streaming service sent on a specific channel,
 * which must belong to the same MU instance used for the cancel request.
 */
struct hse_cancel_srv {
	u8 channel;
};

/**
 * struct hse_srv_meta_data - HSE service meta data
 * @prio: priority of the HSE message
 */
struct hse_srv_meta_data {
	u8 prio;
	u8 reserved[3];
};

/**
 * struct hse_srv_desc - HSE service descriptor
 * @srv_id: service ID of the HSE message
 * @srv_meta_data: service meta data (e.g. priority)
 * @hse_srv: service identified by the service ID
 */
struct hse_srv_desc {
	u32 srv_id;
	struct hse_srv_meta_data srv_meta_data;
	union {
		struct hse_hash_srv hash_req;
		struct hse_cancel_srv cancel_srv_req;
	};
};

#endif /* HSE_ABI_H */
