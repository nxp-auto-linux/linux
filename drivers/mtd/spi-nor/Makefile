# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_MTD_SPI_NOR)	+= spi-nor.o
obj-$(CONFIG_SPI_ASPEED_SMC)	+= aspeed-smc.o
obj-$(CONFIG_SPI_CADENCE_QUADSPI)	+= cadence-quadspi.o
obj-$(CONFIG_SPI_FSL_QUADSPI) += fsl-quadspi.o
ifdef CONFIG_SPI_FSL_QUADSPI
obj-$(CONFIG_SOC_S32GEN1) += s32gen1_qspi.o
obj-$(CONFIG_SOC_S32GEN1) += s32gen1_qspi_macronix.o
obj-$(CONFIG_SOC_S32GEN1) += s32gen1_qspi_micron.o
obj-$(CONFIG_SOC_S32GEN1) += s32gen1_qspi_mem_common.o
endif
obj-$(CONFIG_SPI_HISI_SFC)	+= hisi-sfc.o
obj-$(CONFIG_SPI_MTK_QUADSPI)    += mtk-quadspi.o
obj-$(CONFIG_SPI_NXP_SPIFI)	+= nxp-spifi.o
obj-$(CONFIG_SPI_INTEL_SPI)	+= intel-spi.o
obj-$(CONFIG_SPI_INTEL_SPI_PCI)	+= intel-spi-pci.o
obj-$(CONFIG_SPI_INTEL_SPI_PLATFORM)	+= intel-spi-platform.o
