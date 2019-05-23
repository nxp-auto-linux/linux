.. SPDX-License-Identifier: BSD-3-Clause

===================================
HSE crypto offloading engine driver
===================================

:Copyright: 2019-2020 NXP

Overview
========
The NXP Hardware Security Engine is a security subsystem aimed ar running
relevant security functions for applications with stringent confidentiality
and authenticity requirements. This file contains general information about
the HSE crypto driver, which provides support for offloading cryptographic
operations to HSE's dedicated coprocessors through the kernel crypto API.

Supported Platforms
-------------------
This driver provides crypto offloading support for NXP S32G274A processor.

Supported Algorithms
--------------------
This driver currently supports the following crypto operations:

- Hashing: MD5, SHA1, SHA2
- Symmetric Key Ciphering: AES-CTR, AES-CBC, AES-ECB, AES-CFB
- Message Authentication Codes: HMAC(MD5), HMAC(SHA1), HMAC(SHA2)
- Authenticated Encryption with Associated Data: AES-GCM
- True Random Number Generator: PTG.3 class

Configuration
=============
The following Kconfig options are available:

- Messaging Unit (MU) Instance (CONFIG_CRYPTO_DEV_NXP_HSE_MU_ID):
  There are 4 Messaging Unit instances available for interfacing application
  processor subsystems with HSE and the user can configure which one is used
  by the Linux driver for sending service requests and receiving replies.

- HSE hardware True RNG support (CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG):
  Enable/disable HSE True RNG support. Default value is yes.

- AES Key Group ID within HSE Key Catalog (CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GID):
  There can be up to 256 key groups within HSE RAM Key Catalog, each one storing
  a different key type. This option specifies which key group is used by driver
  for programming AES 256-bit keys into HSE, depending on how the RAM catalog was
  initialized by firmware. Default value is 1.

- Number of AES Key Slots in Key Group (CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GSIZE):
  Each Key Group can store up to 256 keys and this option configures the number
  of keys that can be stored in the AES 256-bit key group.

- HMAC Key Group ID within HSE Key Catalog (CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GID):
  There can be up to 256 key groups within HSE RAM Key Catalog, each one storing
  a different key type. This option specifies which key group is used by driver
  for programming HMAC keys into HSE, depending on how catalog was initialized
  by firmware. Default value is 2.

- Number of HMAC Key Slots in Key Group (CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GSIZE):
  Each Key Group can store up to 256 keys and this option configures the number
  of keys that can be stored in the HMAC key group.

Known Limitations
=================
This driver is affected by the following known issues:

- HSE cannot access the last 512M of DDR, therefore any service descriptors
  or key buffers allocated by the driver in this range are going to cause
  a system reset when the respective requests are being sent to HSE firmware.
  The default DDR size used by Linux on S32G274A has been temporarily reduced
  in order to circumvent this limitation.

- The crypto driver does not currently support the RNG non-blocking mode of
  operation (the wait parameter from hwrng_read is ignored).
