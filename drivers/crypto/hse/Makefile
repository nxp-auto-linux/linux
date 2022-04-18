# SPDX-License-Identifier: BSD-3-Clause

obj-$(CONFIG_CRYPTO_DEV_NXP_HSE) += hse.o

hse-y := hse-core.o hse-mu.o

hse-$(CONFIG_CRYPTO_DEV_NXP_HSE_AHASH) += hse-ahash.o
hse-$(CONFIG_CRYPTO_DEV_NXP_HSE_SKCIPHER) += hse-skcipher.o
hse-$(CONFIG_CRYPTO_DEV_NXP_HSE_AEAD) += hse-aead.o
hse-$(CONFIG_CRYPTO_DEV_NXP_HSE_RNG) += hse-rng.o

ccflags-$(CONFIG_CRYPTO_DEV_NXP_HSE_DEBUG) += -DDEBUG
