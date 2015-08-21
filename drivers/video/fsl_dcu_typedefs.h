/**
*   @file    typedefs.h
 *
*   @brief   This files defines data types used in the header files.
*   @details Types definitions used in the driver.
 */
/*==================================================================================================
*   (c) Copyright 2014 Freescale Semiconductor, Inc
*   All Rights Reserved.
==================================================================================================*/
/*==================================================================================================
Revision History:
                             Modification     Tracking
Author (core ID)              Date D/M/Y       Number     Description of Changes
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     21/03/2014    ENGR00298226  First version of the updated 2D-ACE driver.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     20/05/2014    ENGR00307868  Review and update the 2D-ACE driver.
---------------------------   ----------    ------------  ------------------------------------------
==================================================================================================*/
#ifndef DCU_TYPEDEFS_H_
#define DCU_TYPEDEFS_H_

#include "fsl_dcu_cfg.h"

#if ( DRV_BARE_METAL == DCU_DRV_VARIANT )
  #include <stdint.h>
#endif /* DCU_DRV_VARIANT == DRV_BARE_METAL */

#if ( DRV_LINUX_OS == DCU_DRV_VARIANT )
  #include <linux/module.h>
  #include <linux/kernel.h>
#endif /* DCU_DRV_VARIANT == DRV_LINUX_OS */
    
    /* Standard typedefs used by header files, based on ISO C standard */
    typedef volatile int8_t vint8_t;
    typedef volatile uint8_t vuint8_t;

    typedef volatile int16_t vint16_t;
    typedef volatile uint16_t vuint16_t;

    typedef volatile int32_t vint32_t;
    typedef volatile uint32_t vuint32_t;
    
#endif
