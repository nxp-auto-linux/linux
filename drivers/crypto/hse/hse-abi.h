/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - HSE Firmware Binary Interface
 *
 * This file defines the firmware binary interface of HSE cryptographic engine.
 *
 * Copyright 2019-2020 NXP
 */

#ifndef HSE_ABI_H
#define HSE_ABI_H

#define HSE_KEY_CATALOG_ID_RAM    2u /* RAM key catalog ID */
#define HSE_INVALID_KEY_HANDLE    0xFFFFFFFFul /* invalid key handle */

#define HSE_KEY_HANDLE(group, slot)    ((HSE_KEY_CATALOG_ID_RAM << 16u) |      \
					((group) << 8u) | (slot))

#define HSE_KEY_HMAC_MIN_SIZE    16u /* minimum key size admitted for HMAC */

/**
 * enum hse_status - HSE status
 * @HSE_STATUS_INIT_OK: HSE initialization successfully completed
 * @HSE_STATUS_RNG_INIT_OK: RNG initialization successfully completed
 * @HSE_STATUS_INSTALL_OK: HSE installation phase successfully completed,
 *                         key stores have been formatted and can be used
 * @HSE_STATUS_BOOT_OK: HSE secure booting phase successfully completed
 *
 * Note that if no secure boot is configured (i.e. SMR, SHE, BSB), HSE signals
 * that the booting phase completed successfully.
 */
enum hse_status {
	HSE_STATUS_INIT_OK = BIT(0),
	HSE_STATUS_RNG_INIT_OK = BIT(1),
	HSE_STATUS_INSTALL_OK = BIT(2),
	HSE_STATUS_BOOT_OK = BIT(3),
};

/**
 * enum hse_event - HSE system event
 * @HSE_ERR_NON_FATAL_INTRUSION: non-fatal intrusion detected by HSE
 * @HSE_ERR_FATAL_INTRUSION: fatal intrusion detected by HSE, can only be
 *                           recovered by resetting the entire system
 */
enum hse_event {
	HSE_ERR_NON_FATAL_INTRUSION = BIT(0),
	HSE_ERR_FATAL_INTRUSION = BIT(1),
};

/**
 * enum hse_srv_id - HSE service ID
 * @HSE_SRV_ID_IMPORT_KEY: import/update key into a key store
 * @HSE_SRV_ID_HASH: perform a hash operation
 * @HSE_SRV_ID_MAC: generate a message authentication code
 * @HSE_SRV_ID_SYM_CIPHER: symmetric key encryption/decryption
 * @HSE_SRV_ID_AEAD: AEAD encryption/decryption
 * @HSE_SRV_ID_GET_RANDOM_NUM: hardware random number generator
 */
enum hse_srv_id {
	HSE_SRV_ID_IMPORT_KEY = 0x00000104ul,
	HSE_SRV_ID_HASH = 0x00A50200ul,
	HSE_SRV_ID_MAC = 0x00A50201ul,
	HSE_SRV_ID_SYM_CIPHER = 0x00A50203ul,
	HSE_SRV_ID_AEAD = 0x00A50204ul,
	HSE_SRV_ID_GET_RANDOM_NUM = 0x00000300ul,
};

/**
 * enum hse_srv_response - HSE service response
 * @HSE_SRV_RSP_OK: service successfully executed with no error
 * @HSE_SRV_RSP_VERIFY_FAILED: authentication tag/signature verification failed
 * @HSE_SRV_RSP_INVALID_ADDR: invalid service descriptor address parameters
 * @HSE_SRV_RSP_INVALID_PARAM: invalid service descriptor request parameters
 * @HSE_SRV_RSP_NOT_SUPPORTED: operation or feature not supported
 * @HSE_SRV_RSP_NOT_ALLOWED: operation subject to restrictions (in attributes,
 *                           life-cycle dependent operations, key-management)
 * @HSE_SRV_RSP_NOT_ENOUGH_SPACE: not enough space to perform the service
 * @HSE_SRV_RSP_READ_FAILURE: service request failed, read access denied
 * @HSE_SRV_RSP_WRITE_FAILURE: service request failed, write access denied
 * @HSE_SRV_RSP_STREAMING_MODE_FAILURE: service request in streaming mode failed
 * @HSE_SRV_RSP_KEY_NOT_AVAILABLE: key locked due to failed boot measurement or
 *                                 an active debugger
 * @HSE_SRV_RSP_KEY_INVALID: the key flags don't match the crypto operation
 * @HSE_SRV_RSP_KEY_EMPTY: specified key slot empty
 * @HSE_SRV_RSP_KEY_WRITE_PROTECTED: key slot write protected
 * @HSE_SRV_RSP_KEY_UPDATE_ERROR: specified key slot cannot be updated due to
 *                                errors in verification of the parameters
 * @HSE_SRV_RSP_MEMORY_FAILURE: physical errors (e.g. flipped bits) detected
 *                              during memory read or write
 * @HSE_SRV_RSP_CANCEL_FAILURE: service cannot be canceled
 * @HSE_SRV_RSP_CANCELED: service has been canceled
 * @HSE_SRV_RSP_GENERAL_ERROR: error not covered by the error codes above
 */
enum hse_srv_response {
	HSE_SRV_RSP_OK = 0x55A5AA33ul,
	HSE_SRV_RSP_VERIFY_FAILED = 0x55A5AA35ul,
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
 * enum hse_srv_access_mode - HSE access modes
 * @HSE_ACCESS_MODE_ONE_PASS: ONE-PASS access mode
 * @HSE_ACCESS_MODE_START: START access mode
 * @HSE_ACCESS_MODE_UPDATE: UPDATE access mode
 * @HSE_ACCESS_MODE_FINISH: FINISH access mode
 */
enum hse_srv_access_mode {
	HSE_ACCESS_MODE_ONE_PASS = 0u,
	HSE_ACCESS_MODE_START = 1u,
	HSE_ACCESS_MODE_UPDATE = 2u,
	HSE_ACCESS_MODE_FINISH = 3u,
};

/**
 * enum hse_hash_algorithm - supported hash algorithm types
 * @HSE_HASH_ALGO_MD5: MD5 hash
 * @HSE_HASH_ALGO_SHA1: SHA1 hash
 * @HSE_HASH_ALGO_SHA2_224: SHA2-224 hash
 * @HSE_HASH_ALGO_SHA2_256: SHA2-256 hash
 * @HSE_HASH_ALGO_SHA2_384: SHA2-384 hash
 * @HSE_HASH_ALGO_SHA2_512: SHA2-512 hash
 */
enum hse_hash_algorithm {
	HSE_HASH_ALGO_MD5 = 1u,
	HSE_HASH_ALGO_SHA1 = 2u,
	HSE_HASH_ALGO_SHA2_224 = 3u,
	HSE_HASH_ALGO_SHA2_256 = 4u,
	HSE_HASH_ALGO_SHA2_384 = 5u,
	HSE_HASH_ALGO_SHA2_512 = 6u,
};

/**
 * enum hse_mac_algorithm - supported MAC algorithm types
 * @HSE_HASH_ALGO_HMAC: HMAC
 */
enum hse_mac_algorithm {
	HSE_MAC_ALGO_HMAC = 0x20u,
};

/**
 * enum hse_cipher_algorithm - supported cipher algorithm types
 * @HSE_CIPHER_ALGO_AES: AES cipher
 */
enum hse_cipher_algorithm {
	HSE_CIPHER_ALGO_AES = 0x10u,
};

/**
 * enum hse_block_mode - supported symmetric cipher block modes
 * @HSE_CIPHER_BLOCK_MODE_CTR: counter mode
 * @HSE_CIPHER_BLOCK_MODE_CBC: cipher block chaining mode
 * @HSE_CIPHER_BLOCK_MODE_ECB: electronic codebook mode
 * @HSE_CIPHER_BLOCK_MODE_CFB: cipher feedback mode
 */
enum hse_block_mode {
	HSE_CIPHER_BLOCK_MODE_CTR = 1u,
	HSE_CIPHER_BLOCK_MODE_CBC = 2u,
	HSE_CIPHER_BLOCK_MODE_ECB = 3u,
	HSE_CIPHER_BLOCK_MODE_CFB = 4u,
};

/**
 * enum hse_cipher_dir - symmetric cipher direction
 * @HSE_CIPHER_DIR_DECRYPT: decrypt
 * @HSE_CIPHER_DIR_ENCRYPT: encrypt
 */
enum hse_cipher_dir {
	HSE_CIPHER_DIR_DECRYPT = 0u,
	HSE_CIPHER_DIR_ENCRYPT = 1u,
};

/**
 * enum hse_auth_cipher_mode - authenticated encryption mode
 * @HSE_AUTH_CIPHER_MODE_GCM: Galois/counter mode
 */
enum hse_auth_cipher_mode {
	HSE_AUTH_CIPHER_MODE_GCM = 0x12u,
};

/**
 * enum hse_auth_dir - HSE authentication direction
 * @HSE_AUTH_DIR_GENERATE: generate authentication tag
 */
enum hse_auth_dir {
	HSE_AUTH_DIR_GENERATE = 1u,
};

/**
 * enum hse_key_flags - key properties
 * @HSE_KF_USAGE_ENCRYPT: key used for encryption (and AEAD tag computation)
 * @HSE_KF_USAGE_DECRYPT: key used for decryption (and AEAD tag verification)
 * @HSE_KF_USAGE_SIGN: key used for message authentication code/tag generation
 */
enum hse_key_flags {
	HSE_KF_USAGE_ENCRYPT = BIT(0),
	HSE_KF_USAGE_DECRYPT = BIT(1),
	HSE_KF_USAGE_SIGN = BIT(2),
};

/**
 * enum hse_key_type - key types used by HSE
 * @HSE_KEY_TYPE_AES: AES 128, 192 or 256-bit key
 * @HSE_KEY_TYPE_HMAC: HMAC key, 16-byte or more
 */
enum hse_key_type {
	HSE_KEY_TYPE_AES = 0x12u,
	HSE_KEY_TYPE_HMAC = 0x20u,
};

/**
 * enum hse_rng_class - rng generation method
 * @HSE_RNG_CLASS_PTG3: prediction resistance, reseed every 16 bytes
 */
enum hse_rng_class {
	HSE_RNG_CLASS_PTG3 = 2u,
};

/**
 * enum hse_sgt_opt - scatter-gather table option
 * @HSE_SGT_OPT_NONE: scatter-gather tables are not used
 * @HSE_SGT_OPT_INPUT: input provided as scatter-gather table
 * @HSE_SGT_OPT_OUTPUT: output provided as scatter-gather table
 */
enum hse_sgt_opt {
	HSE_SGT_OPT_NONE = 0u,
	HSE_SGT_OPT_INPUT = BIT(0),
	HSE_SGT_OPT_OUTPUT = BIT(1),
};

/**
 * struct hse_hash_srv - perform a hash operation
 * @access_mode: ONE-PASS, START, UPDATE, FINISH
 * @stream_id: ID for START, UPDATE, FINISH access modes - only a limited number
 *             of channels per MU instance are available for streaming use
 * @hash_algo: hash algorithm to be used
 * @sgt_opt: specify whether input/output is provided as scatter-gather table
 * @input_len: length of the input message - must be an integer multiple of
 *             algorithm block size for START and UPDATE access modes, can be
 *             zero for START and FINISH, there are no restrictions for ONE-PASS
 * @input: address of the input message - mandatory for ONE-PASS and UPDATE
 *         access modes, can be zero for START and FINISH
 * @hash_len: holds the address to a u32 location in which the hash length
 *            in bytes is stored. On calling this service, this parameter
 *            shall contain the size of the buffer provided by host. When the
 *            request has finished, the actual length of the returned value
 *            shall be stored. If the buffer is smaller than the size of the
 *            hash, the hash will be truncated. If the buffer is larger,
 *            *hash_len is adjusted to the size of the hash. The input
 *            hash length shall not be zero for ONE-PASS or FINISH steps
 * @hash: the address where output hash will be stored
 *
 * This service is accessible in ONE-PASS or streaming (SUF) mode. In case of
 * streaming mode, three steps (calls) are needed: START, UPDATE, FINISH. For
 * each streaming step, any fields that aren't mandatory shall be set NULL or 0.
 * The table below summarizes which fields are required for each access mode.
 *
 * | Field \ Mode | ONE-PASS | START | UPDATE | FINISH |
 * |--------------+----------+-------+--------+--------|
 * | access_mode  |     *    |   *   |    *   |    *   |
 * | stream_id    |          |   *   |    *   |    *   |
 * | hash_algo    |     *    |   *   |    *   |    *   |
 * | input_len    |     *    |   *   |    *   |    *   |
 * | input        |     *    |   *   |    *   |    *   |
 * | hash_len     |     *    |       |        |    *   |
 * | hash         |     *    |       |        |    *   |
 *
 */
struct hse_hash_srv {
	u8 access_mode;
	u8 stream_id;
	u8 hash_algo;
	u8 reserved0;
	u8 sgt_opt;
	u8 reserved1[3];
	u32 input_len;
	u64 input;
	u64 hash_len;
	u64 hash;
} __packed;

/**
 * struct hse_mac_srv - generate a message authentication code
 * @access_mode: ONE-PASS, START, UPDATE, FINISH; see &enum hse_srv_access_mode
 * @stream_id: ID for START, UPDATE, FINISH access modes - only a limited number
 *             of channels per MU instance are available for streaming use
 * @auth_dir: direction - generate MAC
 * @scheme: MAC scheme to be used
 * @key_handle: key handle from RAM catalog
 * @sgt_opt: specify whether input/output is provided as scatter-gather table
 * @input_len: length of the input message - must be an integer multiple of
 *             algorithm block size for START and UPDATE access modes, cannot
 *             be zero for any SUF access mode, no restrictions for ONE-PASS
 * @input: address of the input message - mandatory for ONE-PASS and UPDATE
 *         access modes, cannot be zero for any SUF access mode
 * @tag_len: holds the address to a u32 location in which the tag length
 *           in bytes is stored. On calling this service, this parameter
 *           shall contain the size of the buffer provided by host. When the
 *           request has finished, the actual length of the returned value
 *           shall be stored. If the buffer is smaller than the size of the
 *           tag, the tag will be truncated. If the buffer is larger,
 *           *tag_len is adjusted to the size of the tag. The input
 *           tag length shall not be zero for ONE-PASS or FINISH steps
 * @tag: the address where output tag will be stored
 *
 * This service is accessible in ONE-PASS or streaming (SUF) mode. In case of
 * streaming mode, three steps (calls) are needed: START, UPDATE, FINISH. For
 * each streaming step, any fields that aren't mandatory shall be set NULL or 0.
 * The table below summarizes which fields are required for each access mode.
 *
 * | Field \ Mode | ONE-PASS | START | UPDATE | FINISH |
 * |--------------+----------+-------+--------+--------|
 * | access_mode  |     *    |   *   |    *   |    *   |
 * | stream_id    |          |   *   |    *   |    *   |
 * | auth_dir     |     *    |   *   |    *   |    *   |
 * | scheme       |     *    |   *   |    *   |    *   |
 * | key_handle   |     *    |   *   |        |        |
 * | input_len    |     *    |   *   |    *   |    *   |
 * | input        |     *    |   *   |    *   |    *   |
 * | tag_len      |     *    |       |        |    *   |
 * | tag          |     *    |       |        |    *   |
 *
 */
struct hse_mac_srv {
	u8 access_mode;
	u8 stream_id;
	u8 auth_dir;
	u8 reserved0;
	struct hse_mac_scheme {
		u8 mac_algo;
		u8 reserved1[3];
		union {
			struct {
				u8 hash_algo;
			} hmac;
			u8 reserved2[12];
		};
	} scheme;
	u32 key_handle;
	u8 sgt_opt;
	u8 reserved3[3];
	u32 input_len;
	u64 input;
	u64 tag_len;
	u64 tag;
} __packed;

/**
 * struct hse_skcipher_srv - symmetric key cipher encryption/decryption
 * @access_mode: only ONE-PASS mode supported
 * @cipher_algo: cipher algorithm
 * @block_mode: cipher block mode
 * @cipher_dir: direction - encrypt/decrypt from &enum hse_cipher_dir
 * @key_handle: RAM catalog key handle
 * @iv_len: initialization vector/nonce length. Ignored for NULL and ECB block
 *          modes. Must be the appropriate block size for the block mode
 * @iv: address of the initialization vector/nonce. Ignored for NULL and ECB
 *      block modes
 * @sgt_opt: specify whether input/output is provided as scatter-gather table
 * @input_len: plaintext/ciphertext length, in bytes. For ECB, CBC and CFB
 *             cipher block modes, must be a multiple of block length
 * @input: address of the plaintext for encryption, or ciphertext for decryption
 * @output: address of ciphertext for encryption or plaintext for decryption
 *
 * To perform encryption/decryption with a block cipher in ECB or CBC mode, the
 * length of the input must be an exact multiple of the block size. For all AES
 * variants it is 16 bytes (128 bits). If the input plaintext is not an exact
 * multiple of block size, it must be padded by application. For other modes,
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
	u64 iv;
	u8 sgt_opt;
	u8 reserved2[3];
	u32 input_len;
	u64 input;
	u64 output;
} __packed;

/**
 * struct hse_aead_srv - authenticated encryption with additional data
 * @access_mode: only ONE-PASS mode supported
 * @auth_cipher_mode: authenticated cipher mode from &enum hse_auth_cipher_mode
 * @cipher_dir: direction - encrypt/decrypt from &enum hse_cipher_dir
 * @key_handle: RAM catalog key handle
 * @iv_len: initialization vector/nonce length
 *          GCM recommended sizes: 12 bytes or less, zero not allowed
 * @iv: address of the initialization vector/nonce
 * @aad_len: length of AAD header data (in bytes), can be zero
 * @aad: address of AAD header data, ignored if aad_len is zero
 * @sgt_opt: specify whether input/output is provided as scatter-gather table
 * @input_len: plaintext/ciphertext length, in bytes.
 *             Can be zero (compute/verify the tag without input message).
 * @input: address of plaintext for encryption or ciphertext for decryption
 * @tag_len: length of tag (in bytes)
 *           GCM valid Tag sizes 16, 15, 14, 13, 12, 8 or 4
 * @tag: address of output/input tag for authenticated encryption/decryption
 * @output: address of ciphertext for encryption or plaintext for decryption
 *
 * Authenticated Encryption with Associated Data (AEAD) is a block cipher mode
 * of operation which also allows integrity checks (e.g. AES-GCM). Additional
 * authenticated data (AAD) is an optional input header which is authenticated,
 * but not encrypted. Both message integrity and confidentiality are assured.
 */
struct hse_aead_srv {
	u8 access_mode;
	u8 reserved0;
	u8 auth_cipher_mode;
	u8 cipher_dir;
	u32 key_handle;
	u32 iv_len;
	u64 iv;
	u32 aad_len;
	u64 aad;
	u8 sgt_opt;
	u8 reserved1[3];
	u32 input_len;
	u64 input;
	u32 tag_len;
	u64 tag;
	u64 output;
} __packed;

/**
 * struct hse_import_key_srv - import/update key into a key store
 * @key_handle: key slot to update
 * @key_info: address of associated hse_key_info struct (specifying usage
 *            flags, access restrictions, key length in bits, etc.)
 * @sym.key: address of symmetric key
 * @sym.keylen: symmetric key length in bytes
 * @cipher_key: unused, must be set to HSE_INVALID_KEY_HANDLE
 * @auth_key: unused, must be set to HSE_INVALID_KEY_HANDLE
 */
struct hse_import_key_srv {
	u32 key_handle;
	u64 key_info;
	struct {
		u8 reserved0[16];
		u64 key;
		u8 reserved1[4];
		u16 keylen;
		u8 reserved2[2];
	} sym;
	u32 cipher_key;
	u8 reserved3[40];
	u32 auth_key;
	u8 reserved4[36];
} __packed;

/**
 * struct hse_rng_srv - random number generation
 * @rng_class: random number generation method
 * @random_num_len: length of the generated number in bytes
 * @random_num: the address where the generated number will be stored
 */
struct hse_rng_srv {
	u8 rng_class;
	u8 reserved[3];
	u32 random_num_len;
	u64 random_num;
} __packed;

/**
 * struct hse_srv_desc - HSE service descriptor
 * @srv_id: service ID of the HSE request
 * @hash_req: hash service request
 * @mac_req: MAC service request
 * @skcipher_req: symmetric key cipher service request
 * @aead_req: AEAD service request
 * @import_key_req: import key service request
 * @rng_req: RNG service request
 */
struct hse_srv_desc {
	u32 srv_id;
	u8 reserved[4];
	union {
		struct hse_hash_srv hash_req;
		struct hse_mac_srv mac_req;
		struct hse_skcipher_srv skcipher_req;
		struct hse_aead_srv aead_req;
		struct hse_import_key_srv import_key_req;
		struct hse_rng_srv rng_req;
	};
} __packed;

/**
 * struct hse_key_info - key properties
 * @key_flags: the targeted key flags; see &enum hse_key_flags
 * @key_bit_len: length of the key in bits
 * @key_type: targeted key type; see &enum hse_key_type
 */
struct hse_key_info {
	u16 key_flags;
	u16 key_bit_len;
	u8 reserved0[8];
	u8 key_type;
	u8 reserved1[3];
} __packed;

#endif /* HSE_ABI_H */
