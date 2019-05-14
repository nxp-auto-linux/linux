/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - HSE Binary Interface
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_ABI_H
#define HSE_ABI_H

#define HSE_KEY_CATALOG_ID_RAM    2u /* RAM key catalog ID */
#define HSE_INVALID_KEY_HANDLE    0xFFFFFFFFul /* invalid key handle */

#define HSE_KEY_HANDLE(group, slot)    ((HSE_KEY_CATALOG_ID_RAM << 16u) |      \
					((group) << 8u) | (slot))

/**
 * enum hse_srv_id - HSE service ID
 * @HSE_SRV_ID_IMPORT_KEY: import/update key into a key store
 * @HSE_SRV_ID_HASH: perform a hash operation
 * @HSE_SRV_ID_SYM_CIPHER: symmetric cipher encryption/decryption
 */
enum hse_srv_id {
	HSE_SRV_ID_IMPORT_KEY = 0x00000104ul,
	HSE_SRV_ID_HASH = 0x00000200ul,
	HSE_SRV_ID_SYM_CIPHER = 0x00000203ul,
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
 * @HSE_SRV_RSP_KEY_NOT_AVAILABLE: key locked due to failed boot measurement or
 *                                 an active debugger
 * @HSE_SRV_RSP_KEY_INVALID: the key flags don't match the crypto operation
 * @HSE_SRV_RSP_KEY_EMPTY: specified key slot is empty
 * @HSE_SRV_RSP_KEY_WRITE_PROTECTED: key slot write protected
 * @HSE_SRV_RSP_KEY_UPDATE_ERROR: specified key slot cannot be updated due to
 *                                errors in verification of the parameters
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
	HSE_SRV_RSP_KEY_NOT_AVAILABLE = 0xA5AA5571ul,
	HSE_SRV_RSP_KEY_INVALID = 0xA5AA5527ul,
	HSE_SRV_RSP_KEY_EMPTY = 0xA5AA5517ul,
	HSE_SRV_RSP_KEY_WRITE_PROTECTED = 0xA5AA5537ul,
	HSE_SRV_RSP_KEY_UPDATE_ERROR = 0xA5AA5573ul,
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
 * enum hse_hash_algorithms - supported hash algorithm types
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
 * enum hse_cipher_algorithms - supported cipher algorithm types
 * @HSE_CIPHER_ALGO_NULL: none
 * @HSE_CIPHER_ALGO_AES: AES cipher
 */
enum hse_cipher_algorithms {
	HSE_CIPHER_ALGO_NULL = 0u,
	HSE_CIPHER_ALGO_AES = 16u,
};

/**
 * enum hse_block_modes - supported symmetric cipher block modes
 * @HSE_CIPHER_BLOCK_MODE_NULL: none
 * @HSE_CIPHER_BLOCK_MODE_CBC: cipher block chaining mode
 */
enum hse_block_modes {
	HSE_CIPHER_BLOCK_MODE_NULL = 0u,
	HSE_CIPHER_BLOCK_MODE_CBC = 2u,
};

/**
 * enum hse_cipher_dir - symmetric cipher direction
 * @HSE_CIPHER_DIR_ENCRYPT: encrypt
 * @HSE_CIPHER_DIR_DECRYPT: decrypt
 */
enum hse_cipher_dir {
	HSE_CIPHER_DIR_ENCRYPT = 0u,
	HSE_CIPHER_DIR_DECRYPT = 1u,
};

/**
 * enum hse_key_flags - key properties
 * @HSE_KF_MU_INST: key used on current MU instance
 * @HSE_KF_USAGE_ENCRYPT: key used for encryption
 * @HSE_KF_USAGE_DECRYPT: key used for decryption
 */
enum hse_key_flags {
	HSE_KF_MU_INST = (1 << CONFIG_CRYPTO_DEV_NXP_HSE_MU_ID),
	HSE_KF_USAGE_ENCRYPT = (1 << 4u),
	HSE_KF_USAGE_DECRYPT = (1 << 5u),
};

/**
 * enum hse_key_types - key types used by HSE
 * @HSE_KEY_TYPE_AES: AES 128, 192 or 256-bit key
 */
enum hse_key_types {
	HSE_KEY_TYPE_AES = 0x12u,
};

/**
 * struct hse_hash_srv - perform a hash operation
 * @access_mode: ONE-PASS, START, UPDATE, FINISH
 * @stream_id: ID for START, UPDATE, FINISH access modes - only a limited number
 *             of channels per MU instance are available for streaming use
 * @hash_algo: hash algorithm to be used
 * @input_len: length of the input message - must be an integer multiple of
 *             algorithm block size for START and UPDATE access modes, can be
 *             zero for START and FINISH, there are no restrictions for ONE-PASS
 * @p_input: address of the input message - mandatory for ONE-PASS and UPDATE
 *           access modes, can be zero for START and FINISH
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
 * The table below summarizes which fields are used by each access mode.
 * Unless stated otherwise, the unused fields shall be NULL or 0.
 *
 * | Field \ Mode | ONE-PASS | START | UPDATE | FINISH |
 * |--------------+----------+-------+--------+--------|
 * | access_mode  |     *    |   *   |    *   |    *   |
 * | stream_id    |          |   *   |    *   |    *   |
 * | hash_algo    |     *    |   *   |    *   |    *   |
 * | input_len    |     *    |   *   |    *   |    *   |
 * | p_input      |     *    |   *   |    *   |    *   |
 * | p_hash_len   |     *    |       |        |    *   |
 * | p_hash       |     *    |       |        |    *   |
 *
 * The hash service can be accessible in ONE-PASS or streaming (SUF) mode. In
 * case of streaming mode, three steps (calls) are needed: START, UPDATE,
 * FINISH. For each streaming step, the fields that are not mandatory shall be
 * set NULL or 0.
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
} __packed;

/**
 * struct hse_skcipher_srv - symmetric cipher encryption/decryption
 * @access_mode: ONE-PASS, START, UPDATE, FINISH.
 * @cipher_algo: cipher algorithm
 * @block_mode: cipher block mode
 * @cipher_dir: direction - encrypt/decrypt.
 * @key_handle: key handle from RAM catalog
 * @iv_len: initialization vector length. Ignored for NULL and ECB block
 *          modes. Must be the block length corresponding to the block mode
 * @p_iv: address of the initialization vector/nonce. Ignored for NULL and ECB
 *        block modes
 * @input_len: the plaintext and ciphertext length. For ECB, CBC and CFB
 *             cipher block modes, it must be a multiple of block length
 * @p_input: address of the plaintext for encryption, or ciphertext for
 *           decryption
 * @p_output: address of the plaintext for decryption, or ciphertext for
 *            encryption
 *
 * To perform encryption/decryption with a block cipher in ECB or CBC mode, the
 * length of the input must be an exact multiple of the block size. For all AES
 * variants it is 16 bytes (128 bits). If the input plaintext is not an exact
 * multiple of block size, it must be padded by application For other modes,
 * such as counter mode (CTR) or OFB or CFB, padding is not required. In these
 * cases, the ciphertext is always the same length as the plaintext.
 */
struct hse_skcipher_srv {
	u8 access_mode;
	u8 reserved0[1];
	u8 cipher_algo;
	u8 block_mode;
	u8 cipher_dir;
	u8 reserved1[3];
	u32 key_handle;
	u32 iv_len;
	u64 p_iv;
	u32 input_len;
	u64 p_input;
	u64 p_output;
} __packed;

/**
 * struct hse_import_key_srv - import/update key into a key store
 * @target_key_handle: slot in which to add or update a key
 * @p_key_info: pointer to hse_key_info struct, specifying usage flags,
 *              restriction access, key length in bits, etc.
 * @p_key: pointer to key values. Contains the symmetric key
 * @key_len: key length in bytes
 * @cipher_key: unused, must be HSE_INVALID_KEY_HANDLE
 * @auth_key: unused, must be HSE_INVALID_KEY_HANDLE
 */
struct hse_import_key_srv {
	u32 target_key_handle;
	u64 p_key_info;
	u8 reserved0[16];
	u64 p_key;
	u8 reserved1[4];
	u16 key_len;
	u8 reserved2[2];
	u32 cipher_key;
	u8 reserved3[32];
	u32 auth_key;
	u8 reserved4[28];
} __packed;

/**
 * struct hse_srv_desc - HSE service descriptor
 * @srv_id: service ID of the HSE request
 * @priority: priority of the HSE request
 * @hse_srv: service identified by the service ID
 */
struct hse_srv_desc {
	u32 srv_id;
	u8 priority;
	u8 reserved[3];
	union {
		struct hse_hash_srv hash_req;
		struct hse_skcipher_srv skcipher_req;
		struct hse_import_key_srv import_key_req;
	};
};

/**
 * struct hse_key_info - key properties
 * @key_flags: the targeted key flags
 * @key_bit_len: length of the key in bits
 * @key_type: targeted key type
 */
struct hse_key_info {
	u16 key_flags;
	u16 key_bit_len;
	u8 reserved0[8];
	u8 key_type;
	u8 reserved1[3];
};

#endif /* HSE_ABI_H */
