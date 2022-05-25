.. SPDX-License-Identifier: BSD-3-Clause

===================================
HSE crypto offloading engine driver
===================================

:Copyright: 2019-2022 NXP

Overview
========
The NXP Hardware Security Engine is a security subsystem aimed at running
relevant security functions for applications with stringent confidentiality
and authenticity requirements. This file contains general information about
the HSE crypto driver, which provides support for offloading cryptographic
operations to HSE's dedicated coprocessors through the kernel crypto API.

Supported Platforms
-------------------
This driver provides cryptographic offloading support for the
following NXP processors:

- S32G274A
- S32G399A
- S32R45

Supported Algorithms
--------------------
This driver currently supports the following crypto operations:

- Hashing: MD5, SHA1, SHA2
- Symmetric Key Ciphering: AES-CTR, AES-CBC, AES-ECB, AES-CFB
- Message Authentication Codes: HMAC(MD5), HMAC(SHA1), HMAC(SHA2)
- Authenticated Encryption with Associated Data: AES-GCM
- Hardware True Random Number Generation: PTG.3 class

Configuration
=============
The following Kconfig options are available:

- Messaging Unit Interface (CONFIG_CRYPTO_DEV_NXP_HSE_MU):
  There are 4 Messaging Unit instances available for interfacing application
  processor subsystems with HSE and the user can configure which one is used
  by the Linux driver for sending service requests and receiving replies.
  The MU instance indicated here shall be used in interrupt mode and therefore
  should be entirely reserved for the Linux crypto driver. Sharing an instance
  with another driver or application shall result in requests being dropped.

- Message Digest Support (CONFIG_CRYPTO_DEV_NXP_HSE_AHASH):
  Enables hash and hash-based MAC offloading to HSE.

- Symmetric Key Cipher Support (CONFIG_CRYPTO_DEV_NXP_HSE_SKCIPHER):
  Enables symmetric key cipher offloading to HSE.

- AuthEnc and AEAD Support (CONFIG_CRYPTO_DEV_NXP_HSE_AEAD):
  Enables authenticated encryption and AEAD offloading to HSE.

- Hardware RNG support (CONFIG_CRYPTO_DEV_NXP_HSE_RNG):
  Enables hardware true random number generation via HSE.

- Entropy cache maximum size (CONFIG_CRYPTO_DEV_NXP_HSE_RNG_CACHE)
  Total size of driver entropy cache. Used to improve request latency.

- RAM Key Catalog AES Group Configuration:
	- AES 256-bit Key Group ID within RAM Key Catalog
	  (CRYPTO_DEV_NXP_HSE_AES_KEY_GROUP_ID):
	  This option specifies which key group is used by driver for
	  programming AES 256-bit keys into HSE, depending on how the
	  RAM catalog was initialized by firmware.
	- Number of key slots in the AES 256-bit Key Group
	  (CRYPTO_DEV_NXP_HSE_AES_KEY_GROUP_SIZE):
	  This option specifies the maximum number of keys that can be
	  stored in the AES 256-bit key group.

- RAM Key Catalog HMAC Group Configuration:
	- HMAC 1024-bit Key Group ID within RAM Key Catalog
	  (CRYPTO_DEV_NXP_HSE_HMAC_KEY_GROUP_ID):
	  This option specifies which key group is used by driver for
	  programming HMAC 1024-bit keys into HSE, depending on how the
	  RAM catalog was initialized by firmware.
	- Number of key slots in the HMAC 1024-bit Key Group
	  (CRYPTO_DEV_NXP_HSE_HMAC_KEY_GROUP_SIZE):
	  This option specifies the maximum number of keys that can be
	  stored in the HMAC 1024-bit key group.

- Debug information for HSE crypto driver (CRYPTO_DEV_NXP_HSE_DEBUG):
  Enables printing driver debug messages to the kernel log.

Known Limitations
=================
This driver is affected by the following known issues:

- With standard firmware, hmac(sha384) and hmac(sha512) are not supported with
  keys between 64 and 128 bytes in length due to the firmware-imposed limits.
