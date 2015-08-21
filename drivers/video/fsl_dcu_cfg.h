/**
*   @file    dcu_cfg.h
 *
*   @brief   The file is the 2D-ACE header file.
*   @details It contains the version of the UIP used for this platform.
 */
/*==================================================================================================
*   Copyright (c) 2014 Freescale Semiconductor, Inc
*   All Rights Reserved.
==================================================================================================*/
/*==================================================================================================
Revision History:
                             Modification     Tracking
Author (core ID)              Date D/M/Y       Number     Description of Changes
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     04/06/2014    ENGR00316549  First version of the platorm config file.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     04/07/2014    ENGR00321529  Adding the HUD support.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     28/07/2014    ENGR00324499  Implementing changes for frbuff interface.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     19/08/2014    ENGR00327882  Porting the driver on the RAYLEIGH platform.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     27/05/2015    ENGR00355821  Porting on Treerunner. 
==================================================================================================*/
/**
 * \file    dcu_cfg.h
 * \brief   This is the DCU Driver Header File
 * \author
 * \author
 * \version 0.1
 * \date    <dd-month-20xx>
 * \note    <optional some notes>
 ****************************************************************************/
#ifndef DCU_CFG_H_
#define DCU_CFG_H_

#ifdef  __cplusplus
extern "C" {
#endif

/*****************************************************************************
* platform config defines
*****************************************************************************/

/**
* @brief DCU IP Version: DCU v00.00.02.01
* @details M4_SRC_USED_PERIPHERAL IP Versions
*/
#define IPV_DCU                     (IPV_DCU_TREERUNNER)
#define DCU_DRV_VARIANT             (DRV_LINUX_OS)

#define DCU_IRQ_SUPPORT             (0)
#define DCU_IRQ_STATEMACHINE        (0)


#define IPV_DCU_HALO                (0x00000201UL)
#define IPV_DCU_VYBRID              (0x00000101UL)
#define IPV_DCU_RAYLEIGH            (0x00000301UL)
#define IPV_DCU_TREERUNNER          (0x00000401UL)

#define DRV_BARE_METAL              (0)
#define DRV_LINUX_OS                (1)

#define TILE_FIX_SIZE               (0)
#define TILE_VAR_SIZE               (1)

#if (IPV_DCU_TREERUNNER == IPV_DCU)

  /* 64 bits platform architecture */
  #define ARCHITECTURE_64BITS
  /* Number of DCU units */
  #define DCU_NUMBER                      (1U)
  /*** define DCU base address ***/
  #define DCU0_BASE       (const uint32_t)0x40028000
  /* Number of layers */
  #define DCU_LAYERS_NUM_MAX              (8U)
  /* Maximum number of blend layers */
  #define DCU_LAYERS_BLEND_MAX            (2U)

  #define DCU_HUD_FUNCTIONALITY           (1)
  #define DCU_WRITEBACK_FUNCTIONALITY     (1)
  #define DCU_SAFETY_FUNCTIONALITY        (1)
  #define DCU_PDI_FUNCTIONALITY           (0)
  #define DCU_RLE_FUNCTIONALITY           (1)
  #define DCU_TILE_FUNCTIONALITY          (1)
  #define DCU_TILE_MODE                   (TILE_FIX_SIZE)

typedef enum
{
  HUD_DCU = 1                    /**< second 2D_ACE unit */
}Dcu_HUDUnit_t;

#endif


#if (IPV_DCU_RAYLEIGH == IPV_DCU)

  /* Number of DCU units */
  #define DCU_NUMBER                      (1U)
  /*** define DCU base address ***/
  #define DCU0_BASE       (const uint32_t)0x40140000
  /* Number of layers */
  #define DCU_LAYERS_NUM_MAX              (16U)
  /* Maximum number of blend layers */
  #define DCU_LAYERS_BLEND_MAX            (4U)

  #define DCU_HUD_FUNCTIONALITY           (0)
  #define DCU_WRITEBACK_FUNCTIONALITY     (1)
  #define DCU_SAFETY_FUNCTIONALITY        (1)
  #define DCU_RLE_FUNCTIONALITY           (1)
  #define DCU_TILE_FUNCTIONALITY          (1)
  #define DCU_TILE_MODE                   (TILE_VAR_SIZE)

#endif

#if (IPV_DCU_HALO == IPV_DCU)

  /* Number of DCU units */
  #define DCU_NUMBER                      (2U)
  /*** define DCU base address ***/
  #define DCU0_BASE       (const uint32_t)0x40028000
  #define DCU1_BASE       (const uint32_t)0x40038000
  /* Number of layers */
  #define DCU_LAYERS_NUM_MAX              (32U)
  /* Maximum number of blend layers */
  #define DCU_LAYERS_BLEND_MAX            (6U)

  #define DCU_HUD_FUNCTIONALITY           (1)
  #define DCU_WRITEBACK_FUNCTIONALITY     (1)
  #define DCU_SAFETY_FUNCTIONALITY        (1)
  #define DCU_TILE_FUNCTIONALITY          (1)
  #define DCU_TILE_MODE                   (TILE_VAR_SIZE)

typedef enum
{
  HUD_DCU = 1                    /**< second 2D_ACE unit */
}Dcu_HUDUnit_t;

#endif

#if (IPV_DCU_VYBRID == IPV_DCU)

  /* Number of DCU units */
  #define DCU_NUMBER                      (2U)
  /*** define DCU base address ***/
  #define DCU0_BASE       (const uint32_t)0x40058000
  #define DCU1_BASE       (const uint32_t)0x400D8000
  /* Number of layer*/
  #define DCU_LAYERS_NUM_MAX              (64U)
  /* Maximum number of blend layers */
  #define DCU_LAYERS_BLEND_MAX            (6U)

  /* Paralel Data Interface */
  #define DCU_HUD_FUNCTIONALITY           (0)
  #define DCU_WRITEBACK_FUNCTIONALITY     (0)
  #define DCU_SAFETY_FUNCTIONALITY        (1)
  #define DCU_TILE_FUNCTIONALITY          (1)
  #define DCU_TILE_MODE                   (TILE_VAR_SIZE)

#endif

#ifdef  __cplusplus
}
#endif

#endif /* DCU_CFG_H_ */
