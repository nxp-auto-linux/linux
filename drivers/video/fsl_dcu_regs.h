/**
*   @file    dcu4_regs.h
 *
*   @brief   The file defines the 2D-ACE registers' addresses.
*   @details Definitions of addresses of the memory mapped registers.
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
Cristian Tomescu (B13031)     10/04/2014    ENGR00307868  Review and update the 2D-ACE driver.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     04/06/2014    ENGR00316549  Porting the 2D-ACE driver on HALO platform.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     04/07/2014    ENGR00321529  Adding the HUD support.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     29/07/2014    ENGR00324499  Implementing changes for frbuff interface.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     06/08/2014    ENGR00324522  Implementing the write-back support.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     20/08/2014    ENGR00327882  Porting the driver on the RAYLEIGH platform.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     31/10/2014    ENGR00337995  Improvement of some functions.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     07/01/2015    ENGR00344709  Changes about the errors interrupts.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     27/05/2015    ENGR00355821  Porting on Treerunner. 
==================================================================================================*/
#ifndef DCU_REGS_H
#define DCU_REGS_H

#include "fsl_dcu_typedefs.h"
#include "fsl_dcu_cfg.h"

/* DCU - Internal RAM Offsets */
#define DCU_CLUT_OFFSET                 0x00002000
#define DCU_GAMMARED_OFFSET             0x00004000
#define DCU_GAMMAGREEN_OFFSET           0x00004400
#define DCU_GAMMABLUE_OFFSET            0x00004800
#define DCU_CURSOR_OFFSET               0x00004C00

/* CLUT memory size */
#define CLUT_SIZE                       (0x6000)

/* Offset of the layer registers bank */
#define	DCU_LYRDESC_ADDR_OFFSET         (0x40)

/* DCU RLE config flag */
#define NO_RLE_LAYER                    (Dcu_Layer_t)0xFFUL

/* DCU base address mask */
#define DCU_ADDR_MASK	                (0xFFFFFFF8UL)

#if (DRV_BARE_METAL == DCU_DRV_VARIANT)
#if (1 == DCU_NUMBER)
const uint32_t DCU_BASE_ADDRESS[DCU_NUMBER] = {
  DCU0_BASE
};
#endif /* DCU_NUMBER == 1 */
#if (2 == DCU_NUMBER)
const uint32_t DCU_BASE_ADDRESS[DCU_NUMBER] = {
  DCU0_BASE,
  DCU1_BASE
};
#endif /* DCU_NUMBER == 2 */
#endif /* DCU_DRV_VARIANT == DRV_BARE_METAL */

/* DCU - Peripheral register structure */
typedef struct DCU_MemMap {
  vuint32_t CTRLDESCCURSOR1;            /* offset: 0x00000000*/
  vuint32_t CTRLDESCCURSOR2;            /* offset: 0x00000004*/
  vuint32_t CTRLDESCCURSOR3;            /* offset: 0x00000008*/
  vuint32_t CTRLDESCCURSOR4;            /* offset: 0x0000000C*/
  vuint32_t DCU_MODE;                   /* offset: 0x00000010*/
  vuint32_t BGND;                       /* offset: 0x00000014*/
  vuint32_t DISP_SIZE;                  /* offset: 0x00000018*/
  vuint32_t HSYN_PARA;                  /* offset: 0x0000001C*/
  vuint32_t VSYN_PARA;                  /* offset: 0x00000020*/
  vuint32_t SYNPOL;                     /* offset: 0x00000024*/
  vuint32_t THRESHOLD;                  /* offset: 0x00000028*/
  vuint32_t INT_STATUS;                 /* offset: 0x0000002C*/
  vuint32_t INT_MASK;                   /* offset: 0x00000030*/
  vuint32_t COLBAR_1;                   /* offset: 0x00000034*/
  vuint32_t COLBAR_2;                   /* offset: 0x00000038*/
  vuint32_t COLBAR_3;                   /* offset: 0x0000003C*/
  vuint32_t COLBAR_4;                   /* offset: 0x00000040*/
  vuint32_t COLBAR_5;                   /* offset: 0x00000044*/
  vuint32_t COLBAR_6;                   /* offset: 0x00000048*/
  vuint32_t COLBAR_7;                   /* offset: 0x0000004C*/
  vuint32_t COLBAR_8;                   /* offset: 0x00000050*/
  vuint32_t DIV_RATIO;                  /* offset: 0x00000054*/
  vuint32_t SIGN_CALC_1;                /* offset: 0x00000058*/
  vuint32_t SIGN_CALC_2;                /* offset: 0x0000005C*/
  vuint32_t CRC_VAL;                    /* offset: 0x00000060*/
  vuint32_t PDI_STATUS;                 /* offset: 0x00000064*/
  vuint32_t MASK_PDI_STATUS;            /* offset: 0x00000068*/
  vuint32_t PARR_ERR_STATUS1;           /* offset: 0x0000006C*/
  vuint32_t PARR_ERR_STATUS2;           /* offset: 0x00000070*/
  vuint8_t RESERVED0[8];
  vuint32_t PARR_ERR_STATUS3;           /* offset: 0x0000007C*/
  vuint32_t MASK_PARR_ERR_STATUS1;      /* offset: 0x00000080*/
  vuint32_t MASK_PARR_ERR_STATUS2;      /* offset: 0x00000084*/
  vuint8_t RESERVED1[8];
  vuint32_t MASK_PARR_ERR_STATUS3;      /* offset: 0x00000090*/
  vuint32_t THRESHOLD_INP_BUF_1;        /* offset: 0x00000094*/
  vuint32_t THRESHOLD_INP_BUF_2;        /* offset: 0x00000098*/
  vuint32_t THRESHOLD_INP_BUF_3;        /* offset: 0x0000009C*/
  vuint32_t LUMA_COMP;                  /* offset: 0x000000A0*/
  vuint32_t CHROMA_RED;                 /* offset: 0x000000A4*/
  vuint32_t CHROMA_GREEN;               /* offset: 0x000000A8*/
  vuint32_t CHROMA_BLUE;                /* offset: 0x000000AC*/
  vuint32_t CRC_POS;                    /* offset: 0x000000B0*/
  vuint32_t LYR_INTPOL_EN;              /* offset: 0x000000B4*/
  vuint32_t LYR_LUMA_COMP;              /* offset: 0x000000B8*/
  vuint32_t LYR_CHROMA_RED;             /* offset: 0x000000BC*/
  vuint32_t LYR_CHROMA_GREEN;           /* offset: 0x000000C0*/
  vuint32_t LYR_CHROMA_BLUE;            /* offset: 0x000000C4*/
  vuint32_t COMP_IMSIZE;                /* offset: 0x000000C8*/
  vuint32_t UPDATE_MODE;                /* offset: 0x000000CC*/
  vuint32_t UNDERRUN;                   /* offset: 0x000000D0*/
  vuint8_t RESERVED2[44];
  vuint32_t GPR;                        /* offset: 0x00000100*/
  vuint32_t SLR_L0;        	        /* offset: 0x00000104*/
  vuint32_t SLR_L1;                     /* offset: 0x00000108*/
  vuint32_t SLR_DISP_SIZE;              /* offset: 0x0000010C*/
  vuint32_t SLR_HVSYNC_PARA;            /* offset: 0x00000110*/
  vuint32_t SLR_POL;                    /* offset: 0x00000114*/
  vuint32_t SLR_L0TRANSP;               /* offset: 0x00000118*/
  vuint32_t SLR_L1TRANSP;               /* offset: 0x0000011C*/
  vuint8_t RESERVED3[224];
  vuint32_t CTRLDESCL0_1;               /* offset: 0x00000200*/
  vuint32_t CTRLDESCL0_2;               /* offset: 0x00000204*/
  vuint32_t CTRLDESCL0_3;               /* offset: 0x00000208*/
  vuint32_t CTRLDESCL0_4;               /* offset: 0x0000020C*/
  vuint32_t CTRLDESCL0_5;               /* offset: 0x00000210*/
  vuint32_t CTRLDESCL0_6;               /* offset: 0x00000214*/
  vuint32_t CTRLDESCL0_7;               /* offset: 0x00000218*/
  vuint32_t CTRLDESCL0_8;               /* offset: 0x0000021C*/
  vuint32_t CTRLDESCL0_9;               /* offset: 0x00000220*/
  vuint8_t RESERVED4[28];
  vuint32_t CTRLDESCL1_1;               /* offset: 0x00000240*/
  vuint32_t CTRLDESCL1_2;               /* offset: 0x00000244*/
  vuint32_t CTRLDESCL1_3;               /* offset: 0x00000248*/
  vuint32_t CTRLDESCL1_4;               /* offset: 0x0000024C*/
  vuint32_t CTRLDESCL1_5;               /* offset: 0x00000250*/
  vuint32_t CTRLDESCL1_6;               /* offset: 0x00000254*/
  vuint32_t CTRLDESCL1_7;               /* offset: 0x00000258*/
  vuint32_t CTRLDESCL1_8;               /* offset: 0x0000025C*/
  vuint32_t CTRLDESCL1_9;               /* offset: 0x00000260*/
  vuint8_t RESERVED5[28];
  vuint32_t CTRLDESCL2_1;               /* offset: 0x00000280*/
  vuint32_t CTRLDESCL2_2;               /* offset: 0x00000284*/
  vuint32_t CTRLDESCL2_3;               /* offset: 0x00000288*/
  vuint32_t CTRLDESCL2_4;               /* offset: 0x0000028C*/
  vuint32_t CTRLDESCL2_5;               /* offset: 0x00000290*/
  vuint32_t CTRLDESCL2_6;               /* offset: 0x00000294*/
  vuint32_t CTRLDESCL2_7;               /* offset: 0x00000298*/
  vuint32_t CTRLDESCL2_8;               /* offset: 0x0000029C*/
  vuint32_t CTRLDESCL2_9;               /* offset: 0x000002A0*/
  vuint8_t RESERVED6[28];
  vuint32_t CTRLDESCL3_1;               /* offset: 0x000002C0*/
  vuint32_t CTRLDESCL3_2;               /* offset: 0x000002C4*/
  vuint32_t CTRLDESCL3_3;               /* offset: 0x000002C8*/
  vuint32_t CTRLDESCL3_4;               /* offset: 0x000002CC*/
  vuint32_t CTRLDESCL3_5;               /* offset: 0x000002D0*/
  vuint32_t CTRLDESCL3_6;               /* offset: 0x000002D4*/
  vuint32_t CTRLDESCL3_7;               /* offset: 0x000002D8*/
  vuint32_t CTRLDESCL3_8;               /* offset: 0x000002DC*/
  vuint32_t CTRLDESCL3_9;               /* offset: 0x000002E0*/
  vuint8_t RESERVED7[28];
  vuint32_t CTRLDESCL4_1;               /* offset: 0x00000300*/
  vuint32_t CTRLDESCL4_2;               /* offset: 0x00000304*/
  vuint32_t CTRLDESCL4_3;               /* offset: 0x00000308*/
  vuint32_t CTRLDESCL4_4;               /* offset: 0x0000030C*/
  vuint32_t CTRLDESCL4_5;               /* offset: 0x00000310*/
  vuint32_t CTRLDESCL4_6;               /* offset: 0x00000314*/
  vuint32_t CTRLDESCL4_7;               /* offset: 0x00000318*/
  vuint32_t CTRLDESCL4_8;               /* offset: 0x0000031C*/
  vuint32_t CTRLDESCL4_9;               /* offset: 0x00000320*/
  vuint8_t RESERVED8[28];
  vuint32_t CTRLDESCL5_1;               /* offset: 0x00000340*/
  vuint32_t CTRLDESCL5_2;               /* offset: 0x00000344*/
  vuint32_t CTRLDESCL5_3;               /* offset: 0x00000348*/
  vuint32_t CTRLDESCL5_4;               /* offset: 0x0000034C*/
  vuint32_t CTRLDESCL5_5;               /* offset: 0x00000350*/
  vuint32_t CTRLDESCL5_6;               /* offset: 0x00000354*/
  vuint32_t CTRLDESCL5_7;               /* offset: 0x00000358*/
  vuint32_t CTRLDESCL5_8;               /* offset: 0x0000035C*/
  vuint32_t CTRLDESCL5_9;               /* offset: 0x00000360*/
  vuint8_t RESERVED9[28];
  vuint32_t CTRLDESCL6_1;       /* offset: 0x00000380*/
  vuint32_t CTRLDESCL6_2;       /* offset: 0x00000384*/
  vuint32_t CTRLDESCL6_3;       /* offset: 0x00000388*/
  vuint32_t CTRLDESCL6_4;       /* offset: 0x0000038C*/
  vuint32_t CTRLDESCL6_5;       /* offset: 0x00000390*/
  vuint32_t CTRLDESCL6_6;       /* offset: 0x00000394*/
  vuint32_t CTRLDESCL6_7;       /* offset: 0x00000398*/
  vuint32_t CTRLDESCL6_8;       /* offset: 0x0000039C*/
  vuint32_t CTRLDESCL6_9;       /* offset: 0x000003A0*/
  vuint8_t RESERVED10[28];
  vuint32_t CTRLDESCL7_1;       /* offset: 0x000003C0*/
  vuint32_t CTRLDESCL7_2;       /* offset: 0x000003C4*/
  vuint32_t CTRLDESCL7_3;       /* offset: 0x000003C8*/
  vuint32_t CTRLDESCL7_4;       /* offset: 0x000003CC*/
  vuint32_t CTRLDESCL7_5;       /* offset: 0x000003D0*/
  vuint32_t CTRLDESCL7_6;       /* offset: 0x000003D4*/
  vuint32_t CTRLDESCL7_7;       /* offset: 0x000003D8*/
  vuint32_t CTRLDESCL7_8;       /* offset: 0x000003DC*/
  vuint32_t CTRLDESCL7_9;       /* offset: 0x000003E0*/
  vuint8_t RESERVED11[28];
  vuint32_t CTRLDESCL8_1;       /* offset: 0x00000400*/
  vuint32_t CTRLDESCL8_2;       /* offset: 0x00000404*/
  vuint32_t CTRLDESCL8_3;       /* offset: 0x00000408*/
  vuint32_t CTRLDESCL8_4;       /* offset: 0x0000040C*/
  vuint32_t CTRLDESCL8_5;       /* offset: 0x00000410*/
  vuint32_t CTRLDESCL8_6;       /* offset: 0x00000414*/
  vuint32_t CTRLDESCL8_7;       /* offset: 0x00000418*/
  vuint32_t CTRLDESCL8_8;       /* offset: 0x0000041C*/
  vuint32_t CTRLDESCL8_9;       /* offset: 0x00000420*/
  vuint8_t RESERVED12[28];
  vuint32_t CTRLDESCL9_1;       /* offset: 0x00000440*/
  vuint32_t CTRLDESCL9_2;       /* offset: 0x00000444*/
  vuint32_t CTRLDESCL9_3;       /* offset: 0x00000448*/
  vuint32_t CTRLDESCL9_4;       /* offset: 0x0000044C*/
  vuint32_t CTRLDESCL9_5;       /* offset: 0x00000450*/
  vuint32_t CTRLDESCL9_6;       /* offset: 0x00000454*/
  vuint32_t CTRLDESCL9_7;       /* offset: 0x00000458*/
  vuint32_t CTRLDESCL9_8;       /* offset: 0x0000045C*/
  vuint32_t CTRLDESCL9_9;       /* offset: 0x00000460*/
  vuint8_t RESERVED13[28];
  vuint32_t CTRLDESCL10_1;       /* offset: 0x00000480*/
  vuint32_t CTRLDESCL10_2;       /* offset: 0x00000484*/
  vuint32_t CTRLDESCL10_3;       /* offset: 0x00000488*/
  vuint32_t CTRLDESCL10_4;       /* offset: 0x0000048C*/
  vuint32_t CTRLDESCL10_5;       /* offset: 0x00000490*/
  vuint32_t CTRLDESCL10_6;       /* offset: 0x00000494*/
  vuint32_t CTRLDESCL10_7;       /* offset: 0x00000498*/
  vuint32_t CTRLDESCL10_8;       /* offset: 0x0000049C*/
  vuint32_t CTRLDESCL10_9;       /* offset: 0x000004A0*/
  vuint8_t RESERVED14[28];
  vuint32_t CTRLDESCL11_1;       /* offset: 0x000004C0*/
  vuint32_t CTRLDESCL11_2;       /* offset: 0x000004C4*/
  vuint32_t CTRLDESCL11_3;       /* offset: 0x000004C8*/
  vuint32_t CTRLDESCL11_4;       /* offset: 0x000004CC*/
  vuint32_t CTRLDESCL11_5;       /* offset: 0x000004D0*/
  vuint32_t CTRLDESCL11_6;       /* offset: 0x000004D4*/
  vuint32_t CTRLDESCL11_7;       /* offset: 0x000004D8*/
  vuint32_t CTRLDESCL11_8;       /* offset: 0x000004DC*/
  vuint32_t CTRLDESCL11_9;       /* offset: 0x000004E0*/
  vuint8_t RESERVED15[28];
  vuint32_t CTRLDESCL12_1;       /* offset: 0x00000500*/
  vuint32_t CTRLDESCL12_2;       /* offset: 0x00000504*/
  vuint32_t CTRLDESCL12_3;       /* offset: 0x00000508*/
  vuint32_t CTRLDESCL12_4;       /* offset: 0x0000050C*/
  vuint32_t CTRLDESCL12_5;       /* offset: 0x00000510*/
  vuint32_t CTRLDESCL12_6;       /* offset: 0x00000514*/
  vuint32_t CTRLDESCL12_7;       /* offset: 0x00000518*/
  vuint32_t CTRLDESCL12_8;       /* offset: 0x0000051C*/
  vuint32_t CTRLDESCL12_9;       /* offset: 0x00000520*/
  vuint8_t RESERVED16[28];
  vuint32_t CTRLDESCL13_1;       /* offset: 0x00000540*/
  vuint32_t CTRLDESCL13_2;       /* offset: 0x00000544*/
  vuint32_t CTRLDESCL13_3;       /* offset: 0x00000548*/
  vuint32_t CTRLDESCL13_4;       /* offset: 0x0000054C*/
  vuint32_t CTRLDESCL13_5;       /* offset: 0x00000550*/
  vuint32_t CTRLDESCL13_6;       /* offset: 0x00000554*/
  vuint32_t CTRLDESCL13_7;       /* offset: 0x00000558*/
  vuint32_t CTRLDESCL13_8;       /* offset: 0x0000055C*/
  vuint32_t CTRLDESCL13_9;       /* offset: 0x00000560*/
  vuint8_t RESERVED17[28];
  vuint32_t CTRLDESCL14_1;       /* offset: 0x00000580*/
  vuint32_t CTRLDESCL14_2;       /* offset: 0x00000584*/
  vuint32_t CTRLDESCL14_3;       /* offset: 0x00000588*/
  vuint32_t CTRLDESCL14_4;       /* offset: 0x0000058C*/
  vuint32_t CTRLDESCL14_5;       /* offset: 0x00000590*/
  vuint32_t CTRLDESCL14_6;       /* offset: 0x00000594*/
  vuint32_t CTRLDESCL14_7;       /* offset: 0x00000598*/
  vuint32_t CTRLDESCL14_8;       /* offset: 0x0000059C*/
  vuint32_t CTRLDESCL14_9;       /* offset: 0x000005A0*/
  vuint8_t RESERVED18[28];
  vuint32_t CTRLDESCL15_1;       /* offset: 0x000005C0*/
  vuint32_t CTRLDESCL15_2;       /* offset: 0x000005C4*/
  vuint32_t CTRLDESCL15_3;       /* offset: 0x000005C8*/
  vuint32_t CTRLDESCL15_4;       /* offset: 0x000005CC*/
  vuint32_t CTRLDESCL15_5;       /* offset: 0x000005D0*/
  vuint32_t CTRLDESCL15_6;       /* offset: 0x000005D4*/
  vuint32_t CTRLDESCL15_7;       /* offset: 0x000005D8*/
  vuint32_t CTRLDESCL15_8;       /* offset: 0x000005DC*/
  vuint32_t CTRLDESCL15_9;       /* offset: 0x000005E0*/
  vuint8_t RESERVED19[28];
  vuint32_t CTRLDESCL16_1;       /* offset: 0x00000600*/
  vuint32_t CTRLDESCL16_2;       /* offset: 0x00000604*/
  vuint32_t CTRLDESCL16_3;       /* offset: 0x00000608*/
  vuint32_t CTRLDESCL16_4;       /* offset: 0x0000060C*/
  vuint32_t CTRLDESCL16_5;       /* offset: 0x00000610*/
  vuint32_t CTRLDESCL16_6;       /* offset: 0x00000614*/
  vuint32_t CTRLDESCL16_7;       /* offset: 0x00000618*/
  vuint32_t CTRLDESCL16_8;       /* offset: 0x0000061C*/
  vuint32_t CTRLDESCL16_9;       /* offset: 0x00000620*/
  vuint8_t RESERVED20[28];
  vuint32_t CTRLDESCL17_1;       /* offset: 0x00000640*/
  vuint32_t CTRLDESCL17_2;       /* offset: 0x00000644*/
  vuint32_t CTRLDESCL17_3;       /* offset: 0x00000648*/
  vuint32_t CTRLDESCL17_4;       /* offset: 0x0000064C*/
  vuint32_t CTRLDESCL17_5;       /* offset: 0x00000650*/
  vuint32_t CTRLDESCL17_6;       /* offset: 0x00000654*/
  vuint32_t CTRLDESCL17_7;       /* offset: 0x00000658*/
  vuint32_t CTRLDESCL17_8;       /* offset: 0x0000065C*/
  vuint32_t CTRLDESCL17_9;       /* offset: 0x00000660*/
  vuint8_t RESERVED21[28];
  vuint32_t CTRLDESCL18_1;       /* offset: 0x00000680*/
  vuint32_t CTRLDESCL18_2;       /* offset: 0x00000684*/
  vuint32_t CTRLDESCL18_3;       /* offset: 0x00000688*/
  vuint32_t CTRLDESCL18_4;       /* offset: 0x0000068C*/
  vuint32_t CTRLDESCL18_5;       /* offset: 0x00000690*/
  vuint32_t CTRLDESCL18_6;       /* offset: 0x00000694*/
  vuint32_t CTRLDESCL18_7;       /* offset: 0x00000698*/
  vuint32_t CTRLDESCL18_8;       /* offset: 0x0000069C*/
  vuint32_t CTRLDESCL18_9;       /* offset: 0x000006A0*/
  vuint8_t RESERVED22[28];
  vuint32_t CTRLDESCL19_1;       /* offset: 0x000006C0*/
  vuint32_t CTRLDESCL19_2;       /* offset: 0x000006C4*/
  vuint32_t CTRLDESCL19_3;       /* offset: 0x000006C8*/
  vuint32_t CTRLDESCL19_4;       /* offset: 0x000006CC*/
  vuint32_t CTRLDESCL19_5;       /* offset: 0x000006D0*/
  vuint32_t CTRLDESCL19_6;       /* offset: 0x000006D4*/
  vuint32_t CTRLDESCL19_7;       /* offset: 0x000006D8*/
  vuint32_t CTRLDESCL19_8;       /* offset: 0x000006DC*/
  vuint32_t CTRLDESCL19_9;       /* offset: 0x000006E0*/
  vuint8_t RESERVED23[28];
  vuint32_t CTRLDESCL20_1;       /* offset: 0x00000700*/
  vuint32_t CTRLDESCL20_2;       /* offset: 0x00000704*/
  vuint32_t CTRLDESCL20_3;       /* offset: 0x00000708*/
  vuint32_t CTRLDESCL20_4;       /* offset: 0x0000070C*/
  vuint32_t CTRLDESCL20_5;       /* offset: 0x00000710*/
  vuint32_t CTRLDESCL20_6;       /* offset: 0x00000714*/
  vuint32_t CTRLDESCL20_7;       /* offset: 0x00000718*/
  vuint32_t CTRLDESCL20_8;       /* offset: 0x0000071C*/
  vuint32_t CTRLDESCL20_9;       /* offset: 0x00000720*/
  vuint8_t RESERVED24[28];
  vuint32_t CTRLDESCL21_1;       /* offset: 0x00000740*/
  vuint32_t CTRLDESCL21_2;       /* offset: 0x00000744*/
  vuint32_t CTRLDESCL21_3;       /* offset: 0x00000748*/
  vuint32_t CTRLDESCL21_4;       /* offset: 0x0000074C*/
  vuint32_t CTRLDESCL21_5;       /* offset: 0x00000750*/
  vuint32_t CTRLDESCL21_6;       /* offset: 0x00000754*/
  vuint32_t CTRLDESCL21_7;       /* offset: 0x00000758*/
  vuint32_t CTRLDESCL21_8;       /* offset: 0x0000075C*/
  vuint32_t CTRLDESCL21_9;       /* offset: 0x00000760*/
  vuint8_t RESERVED25[28];
  vuint32_t CTRLDESCL22_1;       /* offset: 0x00000780*/
  vuint32_t CTRLDESCL22_2;       /* offset: 0x00000784*/
  vuint32_t CTRLDESCL22_3;       /* offset: 0x00000788*/
  vuint32_t CTRLDESCL22_4;       /* offset: 0x0000078C*/
  vuint32_t CTRLDESCL22_5;       /* offset: 0x00000790*/
  vuint32_t CTRLDESCL22_6;       /* offset: 0x00000794*/
  vuint32_t CTRLDESCL22_7;       /* offset: 0x00000798*/
  vuint32_t CTRLDESCL22_8;       /* offset: 0x0000079C*/
  vuint32_t CTRLDESCL22_9;       /* offset: 0x000007A0*/
  vuint8_t RESERVED26[28];
  vuint32_t CTRLDESCL23_1;       /* offset: 0x000007C0*/
  vuint32_t CTRLDESCL23_2;       /* offset: 0x000007C4*/
  vuint32_t CTRLDESCL23_3;       /* offset: 0x000007C8*/
  vuint32_t CTRLDESCL23_4;       /* offset: 0x000007CC*/
  vuint32_t CTRLDESCL23_5;       /* offset: 0x000007D0*/
  vuint32_t CTRLDESCL23_6;       /* offset: 0x000007D4*/
  vuint32_t CTRLDESCL23_7;       /* offset: 0x000007D8*/
  vuint32_t CTRLDESCL23_8;       /* offset: 0x000007DC*/
  vuint32_t CTRLDESCL23_9;       /* offset: 0x000007E0*/
  vuint8_t RESERVED27[28];
  vuint32_t CTRLDESCL24_1;       /* offset: 0x00000800*/
  vuint32_t CTRLDESCL24_2;       /* offset: 0x00000804*/
  vuint32_t CTRLDESCL24_3;       /* offset: 0x00000808*/
  vuint32_t CTRLDESCL24_4;       /* offset: 0x0000080C*/
  vuint32_t CTRLDESCL24_5;       /* offset: 0x00000810*/
  vuint32_t CTRLDESCL24_6;       /* offset: 0x00000814*/
  vuint32_t CTRLDESCL24_7;       /* offset: 0x00000818*/
  vuint32_t CTRLDESCL24_8;       /* offset: 0x0000081C*/
  vuint32_t CTRLDESCL24_9;       /* offset: 0x00000820*/
  vuint8_t RESERVED28[28];
  vuint32_t CTRLDESCL25_1;       /* offset: 0x00000840*/
  vuint32_t CTRLDESCL25_2;       /* offset: 0x00000844*/
  vuint32_t CTRLDESCL25_3;       /* offset: 0x00000848*/
  vuint32_t CTRLDESCL25_4;       /* offset: 0x0000084C*/
  vuint32_t CTRLDESCL25_5;       /* offset: 0x00000850*/
  vuint32_t CTRLDESCL25_6;       /* offset: 0x00000854*/
  vuint32_t CTRLDESCL25_7;       /* offset: 0x00000858*/
  vuint32_t CTRLDESCL25_8;       /* offset: 0x0000085C*/
  vuint32_t CTRLDESCL25_9;       /* offset: 0x00000860*/
  vuint8_t RESERVED29[28];
  vuint32_t CTRLDESCL26_1;       /* offset: 0x00000880*/
  vuint32_t CTRLDESCL26_2;       /* offset: 0x00000884*/
  vuint32_t CTRLDESCL26_3;       /* offset: 0x00000888*/
  vuint32_t CTRLDESCL26_4;       /* offset: 0x0000088C*/
  vuint32_t CTRLDESCL26_5;       /* offset: 0x00000890*/
  vuint32_t CTRLDESCL26_6;       /* offset: 0x00000894*/
  vuint32_t CTRLDESCL26_7;       /* offset: 0x00000898*/
  vuint32_t CTRLDESCL26_8;       /* offset: 0x0000089C*/
  vuint32_t CTRLDESCL26_9;       /* offset: 0x000008A0*/
  vuint8_t RESERVED30[28];
  vuint32_t CTRLDESCL27_1;       /* offset: 0x000008C0*/
  vuint32_t CTRLDESCL27_2;       /* offset: 0x000008C4*/
  vuint32_t CTRLDESCL27_3;       /* offset: 0x000008C8*/
  vuint32_t CTRLDESCL27_4;       /* offset: 0x000008CC*/
  vuint32_t CTRLDESCL27_5;       /* offset: 0x000008D0*/
  vuint32_t CTRLDESCL27_6;       /* offset: 0x000008D4*/
  vuint32_t CTRLDESCL27_7;       /* offset: 0x000008D8*/
  vuint32_t CTRLDESCL27_8;       /* offset: 0x000008DC*/
  vuint32_t CTRLDESCL27_9;       /* offset: 0x000008E0*/
  vuint8_t RESERVED31[28];
  vuint32_t CTRLDESCL28_1;       /* offset: 0x00000900*/
  vuint32_t CTRLDESCL28_2;       /* offset: 0x00000904*/
  vuint32_t CTRLDESCL28_3;       /* offset: 0x00000908*/
  vuint32_t CTRLDESCL28_4;       /* offset: 0x0000090C*/
  vuint32_t CTRLDESCL28_5;       /* offset: 0x00000910*/
  vuint32_t CTRLDESCL28_6;       /* offset: 0x00000914*/
  vuint32_t CTRLDESCL28_7;       /* offset: 0x00000918*/
  vuint32_t CTRLDESCL28_8;       /* offset: 0x0000091C*/
  vuint32_t CTRLDESCL28_9;       /* offset: 0x00000920*/
  vuint8_t RESERVED32[28];
  vuint32_t CTRLDESCL29_1;       /* offset: 0x00000940*/
  vuint32_t CTRLDESCL29_2;       /* offset: 0x00000944*/
  vuint32_t CTRLDESCL29_3;       /* offset: 0x00000948*/
  vuint32_t CTRLDESCL29_4;       /* offset: 0x0000094C*/
  vuint32_t CTRLDESCL29_5;       /* offset: 0x00000950*/
  vuint32_t CTRLDESCL29_6;       /* offset: 0x00000954*/
  vuint32_t CTRLDESCL29_7;       /* offset: 0x00000958*/
  vuint32_t CTRLDESCL29_8;       /* offset: 0x0000095C*/
  vuint32_t CTRLDESCL29_9;       /* offset: 0x00000960*/
  vuint8_t RESERVED33[28];
  vuint32_t CTRLDESCL30_1;       /* offset: 0x00000980*/
  vuint32_t CTRLDESCL30_2;       /* offset: 0x00000984*/
  vuint32_t CTRLDESCL30_3;       /* offset: 0x00000988*/
  vuint32_t CTRLDESCL30_4;       /* offset: 0x0000098C*/
  vuint32_t CTRLDESCL30_5;       /* offset: 0x00000990*/
  vuint32_t CTRLDESCL30_6;       /* offset: 0x00000994*/
  vuint32_t CTRLDESCL30_7;       /* offset: 0x00000998*/
  vuint32_t CTRLDESCL30_8;       /* offset: 0x0000099C*/
  vuint32_t CTRLDESCL30_9;       /* offset: 0x000009A0*/
  vuint8_t RESERVED34[28];
  vuint32_t CTRLDESCL31_1;       /* offset: 0x000009C0*/
  vuint32_t CTRLDESCL31_2;       /* offset: 0x000009C4*/
  vuint32_t CTRLDESCL31_3;       /* offset: 0x000009C8*/
  vuint32_t CTRLDESCL31_4;       /* offset: 0x000009CC*/
  vuint32_t CTRLDESCL31_5;       /* offset: 0x000009D0*/
  vuint32_t CTRLDESCL31_6;       /* offset: 0x000009D4*/
  vuint32_t CTRLDESCL31_7;       /* offset: 0x000009D8*/
  vuint32_t CTRLDESCL31_8;       /* offset: 0x000009DC*/
  vuint32_t CTRLDESCL31_9;       /* offset: 0x000009E0*/
  vuint8_t RESERVED36[28];
  vuint32_t CTRLDESCL32_1;       /* offset: 0x00000A00*/
  vuint32_t CTRLDESCL32_2;       /* offset: 0x00000A04*/
  vuint32_t CTRLDESCL32_3;       /* offset: 0x00000A08*/
  vuint32_t CTRLDESCL32_4;       /* offset: 0x00000A0C*/
  vuint32_t CTRLDESCL32_5;       /* offset: 0x00000A10*/
  vuint32_t CTRLDESCL32_6;       /* offset: 0x00000A14*/
  vuint32_t CTRLDESCL32_7;       /* offset: 0x00000A18*/
  vuint32_t CTRLDESCL32_8;       /* offset: 0x00000A1C*/
  vuint32_t CTRLDESCL32_9;       /* offset: 0x00000A20*/
  vuint8_t RESERVED37[28];
  vuint32_t CTRLDESCL33_1;       /* offset: 0x00000A40*/
  vuint32_t CTRLDESCL33_2;       /* offset: 0x00000A44*/
  vuint32_t CTRLDESCL33_3;       /* offset: 0x00000A48*/
  vuint32_t CTRLDESCL33_4;       /* offset: 0x00000A4C*/
  vuint32_t CTRLDESCL33_5;       /* offset: 0x00000A50*/
  vuint32_t CTRLDESCL33_6;       /* offset: 0x00000A54*/
  vuint32_t CTRLDESCL33_7;       /* offset: 0x00000A58*/
  vuint32_t CTRLDESCL33_8;       /* offset: 0x00000A5C*/
  vuint32_t CTRLDESCL33_9;       /* offset: 0x00000A60*/
  vuint8_t RESERVED38[28];
  vuint32_t CTRLDESCL34_1;       /* offset: 0x00000A80*/
  vuint32_t CTRLDESCL34_2;       /* offset: 0x00000A84*/
  vuint32_t CTRLDESCL34_3;       /* offset: 0x00000A88*/
  vuint32_t CTRLDESCL34_4;       /* offset: 0x00000A8C*/
  vuint32_t CTRLDESCL34_5;       /* offset: 0x00000A90*/
  vuint32_t CTRLDESCL34_6;       /* offset: 0x00000A94*/
  vuint32_t CTRLDESCL34_7;       /* offset: 0x00000A98*/
  vuint32_t CTRLDESCL34_8;       /* offset: 0x00000A9C*/
  vuint32_t CTRLDESCL34_9;       /* offset: 0x00000AA0*/
  vuint8_t RESERVED39[28];
  vuint32_t CTRLDESCL35_1;       /* offset: 0x00000AC0*/
  vuint32_t CTRLDESCL35_2;       /* offset: 0x00000AC4*/
  vuint32_t CTRLDESCL35_3;       /* offset: 0x00000AC8*/
  vuint32_t CTRLDESCL35_4;       /* offset: 0x00000ACC*/
  vuint32_t CTRLDESCL35_5;       /* offset: 0x00000AD0*/
  vuint32_t CTRLDESCL35_6;       /* offset: 0x00000AD4*/
  vuint32_t CTRLDESCL35_7;       /* offset: 0x00000AD8*/
  vuint32_t CTRLDESCL35_8;       /* offset: 0x00000ADC*/
  vuint32_t CTRLDESCL35_9;       /* offset: 0x00000AE0*/
  vuint8_t RESERVED40[28];
  vuint32_t CTRLDESCL36_1;       /* offset: 0x00000B00*/
  vuint32_t CTRLDESCL36_2;       /* offset: 0x00000B04*/
  vuint32_t CTRLDESCL36_3;       /* offset: 0x00000B08*/
  vuint32_t CTRLDESCL36_4;       /* offset: 0x00000B0C*/
  vuint32_t CTRLDESCL36_5;       /* offset: 0x00000B10*/
  vuint32_t CTRLDESCL36_6;       /* offset: 0x00000B14*/
  vuint32_t CTRLDESCL36_7;       /* offset: 0x00000B18*/
  vuint32_t CTRLDESCL36_8;       /* offset: 0x00000B1C*/
  vuint32_t CTRLDESCL36_9;       /* offset: 0x00000B20*/
  vuint8_t RESERVED41[28];
  vuint32_t CTRLDESCL37_1;       /* offset: 0x00000B40*/
  vuint32_t CTRLDESCL37_2;       /* offset: 0x00000B44*/
  vuint32_t CTRLDESCL37_3;       /* offset: 0x00000B48*/
  vuint32_t CTRLDESCL37_4;       /* offset: 0x00000B4C*/
  vuint32_t CTRLDESCL37_5;       /* offset: 0x00000B50*/
  vuint32_t CTRLDESCL37_6;       /* offset: 0x00000B54*/
  vuint32_t CTRLDESCL37_7;       /* offset: 0x00000B58*/
  vuint32_t CTRLDESCL37_8;       /* offset: 0x00000B5C*/
  vuint32_t CTRLDESCL37_9;       /* offset: 0x00000B60*/
  vuint8_t RESERVED42[28];
  vuint32_t CTRLDESCL38_1;       /* offset: 0x00000B80*/
  vuint32_t CTRLDESCL38_2;       /* offset: 0x00000B84*/
  vuint32_t CTRLDESCL38_3;       /* offset: 0x00000B88*/
  vuint32_t CTRLDESCL38_4;       /* offset: 0x00000B8C*/
  vuint32_t CTRLDESCL38_5;       /* offset: 0x00000B90*/
  vuint32_t CTRLDESCL38_6;       /* offset: 0x00000B94*/
  vuint32_t CTRLDESCL38_7;       /* offset: 0x00000B98*/
  vuint32_t CTRLDESCL38_8;       /* offset: 0x00000B9C*/
  vuint32_t CTRLDESCL38_9;       /* offset: 0x00000BA0*/
  vuint8_t RESERVED43[28];
  vuint32_t CTRLDESCL39_1;       /* offset: 0x00000BC0*/
  vuint32_t CTRLDESCL39_2;       /* offset: 0x00000BC4*/
  vuint32_t CTRLDESCL39_3;       /* offset: 0x00000BC8*/
  vuint32_t CTRLDESCL39_4;       /* offset: 0x00000BCC*/
  vuint32_t CTRLDESCL39_5;       /* offset: 0x00000BD0*/
  vuint32_t CTRLDESCL39_6;       /* offset: 0x00000BD4*/
  vuint32_t CTRLDESCL39_7;       /* offset: 0x00000BD8*/
  vuint32_t CTRLDESCL39_8;       /* offset: 0x00000BDC*/
  vuint32_t CTRLDESCL39_9;       /* offset: 0x00000BE0*/
  vuint8_t RESERVED44[28];
  vuint32_t CTRLDESCL40_1;       /* offset: 0x00000C00*/
  vuint32_t CTRLDESCL40_2;       /* offset: 0x00000C04*/
  vuint32_t CTRLDESCL40_3;       /* offset: 0x00000C08*/
  vuint32_t CTRLDESCL40_4;       /* offset: 0x00000C0C*/
  vuint32_t CTRLDESCL40_5;       /* offset: 0x00000C10*/
  vuint32_t CTRLDESCL40_6;       /* offset: 0x00000C14*/
  vuint32_t CTRLDESCL40_7;       /* offset: 0x00000C18*/
  vuint32_t CTRLDESCL40_8;       /* offset: 0x00000C1C*/
  vuint32_t CTRLDESCL40_9;       /* offset: 0x00000C20*/
  vuint8_t RESERVED45[28];
  vuint32_t CTRLDESCL41_1;       /* offset: 0x00000C40*/
  vuint32_t CTRLDESCL41_2;       /* offset: 0x00000C44*/
  vuint32_t CTRLDESCL41_3;       /* offset: 0x00000C48*/
  vuint32_t CTRLDESCL41_4;       /* offset: 0x00000C4C*/
  vuint32_t CTRLDESCL41_5;       /* offset: 0x00000C50*/
  vuint32_t CTRLDESCL41_6;       /* offset: 0x00000C54*/
  vuint32_t CTRLDESCL41_7;       /* offset: 0x00000C58*/
  vuint32_t CTRLDESCL41_8;       /* offset: 0x00000C5C*/
  vuint32_t CTRLDESCL41_9;       /* offset: 0x00000C60*/
  vuint8_t RESERVED46[28];
  vuint32_t CTRLDESCL42_1;       /* offset: 0x00000C80*/
  vuint32_t CTRLDESCL42_2;       /* offset: 0x00000C84*/
  vuint32_t CTRLDESCL42_3;       /* offset: 0x00000C88*/
  vuint32_t CTRLDESCL42_4;       /* offset: 0x00000C8C*/
  vuint32_t CTRLDESCL42_5;       /* offset: 0x00000C90*/
  vuint32_t CTRLDESCL42_6;       /* offset: 0x00000C94*/
  vuint32_t CTRLDESCL42_7;       /* offset: 0x00000C98*/
  vuint32_t CTRLDESCL42_8;       /* offset: 0x00000C9C*/
  vuint32_t CTRLDESCL42_9;       /* offset: 0x00000CA0*/
  vuint8_t RESERVED47[28];
  vuint32_t CTRLDESCL43_1;       /* offset: 0x00000CC0*/
  vuint32_t CTRLDESCL43_2;       /* offset: 0x00000CC4*/
  vuint32_t CTRLDESCL43_3;       /* offset: 0x00000CC8*/
  vuint32_t CTRLDESCL43_4;       /* offset: 0x00000CCC*/
  vuint32_t CTRLDESCL43_5;       /* offset: 0x00000CD0*/
  vuint32_t CTRLDESCL43_6;       /* offset: 0x00000CD4*/
  vuint32_t CTRLDESCL43_7;       /* offset: 0x00000CD8*/
  vuint32_t CTRLDESCL43_8;       /* offset: 0x00000CDC*/
  vuint32_t CTRLDESCL43_9;       /* offset: 0x00000CE0*/
  vuint8_t RESERVED48[28];
  vuint32_t CTRLDESCL44_1;       /* offset: 0x00000D00*/
  vuint32_t CTRLDESCL44_2;       /* offset: 0x00000D04*/
  vuint32_t CTRLDESCL44_3;       /* offset: 0x00000D08*/
  vuint32_t CTRLDESCL44_4;       /* offset: 0x00000D0C*/
  vuint32_t CTRLDESCL44_5;       /* offset: 0x00000D10*/
  vuint32_t CTRLDESCL44_6;       /* offset: 0x00000D14*/
  vuint32_t CTRLDESCL44_7;       /* offset: 0x00000D18*/
  vuint32_t CTRLDESCL44_8;       /* offset: 0x00000D1C*/
  vuint32_t CTRLDESCL44_9;       /* offset: 0x00000D20*/
  vuint8_t RESERVED49[28];
  vuint32_t CTRLDESCL45_1;       /* offset: 0x00000D40*/
  vuint32_t CTRLDESCL45_2;       /* offset: 0x00000D44*/
  vuint32_t CTRLDESCL45_3;       /* offset: 0x00000D48*/
  vuint32_t CTRLDESCL45_4;       /* offset: 0x00000D4C*/
  vuint32_t CTRLDESCL45_5;       /* offset: 0x00000D50*/
  vuint32_t CTRLDESCL45_6;       /* offset: 0x00000D54*/
  vuint32_t CTRLDESCL45_7;       /* offset: 0x00000D58*/
  vuint32_t CTRLDESCL45_8;       /* offset: 0x00000D5C*/
  vuint32_t CTRLDESCL45_9;       /* offset: 0x00000D60*/
  vuint8_t RESERVED50[28];
  vuint32_t CTRLDESCL46_1;       /* offset: 0x00000D80*/
  vuint32_t CTRLDESCL46_2;       /* offset: 0x00000D84*/
  vuint32_t CTRLDESCL46_3;       /* offset: 0x00000D88*/
  vuint32_t CTRLDESCL46_4;       /* offset: 0x00000D8C*/
  vuint32_t CTRLDESCL46_5;       /* offset: 0x00000D90*/
  vuint32_t CTRLDESCL46_6;       /* offset: 0x00000D94*/
  vuint32_t CTRLDESCL46_7;       /* offset: 0x00000D98*/
  vuint32_t CTRLDESCL46_8;       /* offset: 0x00000D9C*/
  vuint32_t CTRLDESCL46_9;       /* offset: 0x00000DA0*/
  vuint8_t RESERVED51[28];
  vuint32_t CTRLDESCL47_1;       /* offset: 0x00000DC0*/
  vuint32_t CTRLDESCL47_2;       /* offset: 0x00000DC4*/
  vuint32_t CTRLDESCL47_3;       /* offset: 0x00000DC8*/
  vuint32_t CTRLDESCL47_4;       /* offset: 0x00000DCC*/
  vuint32_t CTRLDESCL47_5;       /* offset: 0x00000DD0*/
  vuint32_t CTRLDESCL47_6;       /* offset: 0x00000DD4*/
  vuint32_t CTRLDESCL47_7;       /* offset: 0x00000DD8*/
  vuint32_t CTRLDESCL47_8;       /* offset: 0x00000DDC*/
  vuint32_t CTRLDESCL47_9;       /* offset: 0x00000DE0*/
  vuint8_t RESERVED52[28];
  vuint32_t CTRLDESCL48_1;       /* offset: 0x00000E00*/
  vuint32_t CTRLDESCL48_2;       /* offset: 0x00000E04*/
  vuint32_t CTRLDESCL48_3;       /* offset: 0x00000E08*/
  vuint32_t CTRLDESCL48_4;       /* offset: 0x00000E0C*/
  vuint32_t CTRLDESCL48_5;       /* offset: 0x00000E10*/
  vuint32_t CTRLDESCL48_6;       /* offset: 0x00000E14*/
  vuint32_t CTRLDESCL48_7;       /* offset: 0x00000E18*/
  vuint32_t CTRLDESCL48_8;       /* offset: 0x00000E1C*/
  vuint32_t CTRLDESCL48_9;       /* offset: 0x00000E20*/
  vuint8_t RESERVED53[28];
  vuint32_t CTRLDESCL49_1;       /* offset: 0x00000E40*/
  vuint32_t CTRLDESCL49_2;       /* offset: 0x00000E44*/
  vuint32_t CTRLDESCL49_3;       /* offset: 0x00000E48*/
  vuint32_t CTRLDESCL49_4;       /* offset: 0x00000E4C*/
  vuint32_t CTRLDESCL49_5;       /* offset: 0x00000E50*/
  vuint32_t CTRLDESCL49_6;       /* offset: 0x00000E54*/
  vuint32_t CTRLDESCL49_7;       /* offset: 0x00000E58*/
  vuint32_t CTRLDESCL49_8;       /* offset: 0x00000E5C*/
  vuint32_t CTRLDESCL49_9;       /* offset: 0x00000E60*/
  vuint8_t RESERVED54[28];
  vuint32_t CTRLDESCL50_1;       /* offset: 0x00000E80*/
  vuint32_t CTRLDESCL50_2;       /* offset: 0x00000E84*/
  vuint32_t CTRLDESCL50_3;       /* offset: 0x00000E88*/
  vuint32_t CTRLDESCL50_4;       /* offset: 0x00000E8C*/
  vuint32_t CTRLDESCL50_5;       /* offset: 0x00000E90*/
  vuint32_t CTRLDESCL50_6;       /* offset: 0x00000E94*/
  vuint32_t CTRLDESCL50_7;       /* offset: 0x00000E98*/
  vuint32_t CTRLDESCL50_8;       /* offset: 0x00000E9C*/
  vuint32_t CTRLDESCL50_9;       /* offset: 0x00000EA0*/
  vuint8_t RESERVED55[28];
  vuint32_t CTRLDESCL51_1;       /* offset: 0x00000EC0*/
  vuint32_t CTRLDESCL51_2;       /* offset: 0x00000EC4*/
  vuint32_t CTRLDESCL51_3;       /* offset: 0x00000EC8*/
  vuint32_t CTRLDESCL51_4;       /* offset: 0x00000ECC*/
  vuint32_t CTRLDESCL51_5;       /* offset: 0x00000ED0*/
  vuint32_t CTRLDESCL51_6;       /* offset: 0x00000ED4*/
  vuint32_t CTRLDESCL51_7;       /* offset: 0x00000ED8*/
  vuint32_t CTRLDESCL51_8;       /* offset: 0x00000EDC*/
  vuint32_t CTRLDESCL51_9;       /* offset: 0x00000EE0*/
  vuint8_t RESERVED56[28];
  vuint32_t CTRLDESCL52_1;       /* offset: 0x00000F00*/
  vuint32_t CTRLDESCL52_2;       /* offset: 0x00000F04*/
  vuint32_t CTRLDESCL52_3;       /* offset: 0x00000F08*/
  vuint32_t CTRLDESCL52_4;       /* offset: 0x00000F0C*/
  vuint32_t CTRLDESCL52_5;       /* offset: 0x00000F10*/
  vuint32_t CTRLDESCL52_6;       /* offset: 0x00000F14*/
  vuint32_t CTRLDESCL52_7;       /* offset: 0x00000F18*/
  vuint32_t CTRLDESCL52_8;       /* offset: 0x00000F1C*/
  vuint32_t CTRLDESCL52_9;       /* offset: 0x00000F20*/
  vuint8_t RESERVED57[28];
  vuint32_t CTRLDESCL53_1;       /* offset: 0x00000F40*/
  vuint32_t CTRLDESCL53_2;       /* offset: 0x00000F44*/
  vuint32_t CTRLDESCL53_3;       /* offset: 0x00000F48*/
  vuint32_t CTRLDESCL53_4;       /* offset: 0x00000F4C*/
  vuint32_t CTRLDESCL53_5;       /* offset: 0x00000F50*/
  vuint32_t CTRLDESCL53_6;       /* offset: 0x00000F54*/
  vuint32_t CTRLDESCL53_7;       /* offset: 0x00000F58*/
  vuint32_t CTRLDESCL53_8;       /* offset: 0x00000F5C*/
  vuint32_t CTRLDESCL53_9;       /* offset: 0x00000F60*/
  vuint8_t RESERVED58[28];
  vuint32_t CTRLDESCL54_1;       /* offset: 0x00000F80*/
  vuint32_t CTRLDESCL54_2;       /* offset: 0x00000F84*/
  vuint32_t CTRLDESCL54_3;       /* offset: 0x00000F88*/
  vuint32_t CTRLDESCL54_4;       /* offset: 0x00000F8C*/
  vuint32_t CTRLDESCL54_5;       /* offset: 0x00000F90*/
  vuint32_t CTRLDESCL54_6;       /* offset: 0x00000F94*/
  vuint32_t CTRLDESCL54_7;       /* offset: 0x00000F98*/
  vuint32_t CTRLDESCL54_8;       /* offset: 0x00000F9C*/
  vuint32_t CTRLDESCL54_9;       /* offset: 0x00000FA0*/
  vuint8_t RESERVED59[28];
  vuint32_t CTRLDESCL55_1;       /* offset: 0x00000FC0*/
  vuint32_t CTRLDESCL55_2;       /* offset: 0x00000FC4*/
  vuint32_t CTRLDESCL55_3;       /* offset: 0x00000FC8*/
  vuint32_t CTRLDESCL55_4;       /* offset: 0x00000FCC*/
  vuint32_t CTRLDESCL55_5;       /* offset: 0x00000FD0*/
  vuint32_t CTRLDESCL55_6;       /* offset: 0x00000FD4*/
  vuint32_t CTRLDESCL55_7;       /* offset: 0x00000FD8*/
  vuint32_t CTRLDESCL55_8;       /* offset: 0x00000FDC*/
  vuint32_t CTRLDESCL55_9;       /* offset: 0x00000FE0*/
  vuint8_t RESERVED60[28];
  vuint32_t CTRLDESCL56_1;       /* offset: 0x00001000*/
  vuint32_t CTRLDESCL56_2;       /* offset: 0x00001004*/
  vuint32_t CTRLDESCL56_3;       /* offset: 0x00001008*/
  vuint32_t CTRLDESCL56_4;       /* offset: 0x0000100C*/
  vuint32_t CTRLDESCL56_5;       /* offset: 0x00001010*/
  vuint32_t CTRLDESCL56_6;       /* offset: 0x00001014*/
  vuint32_t CTRLDESCL56_7;       /* offset: 0x00001018*/
  vuint32_t CTRLDESCL56_8;       /* offset: 0x0000101C*/
  vuint32_t CTRLDESCL56_9;       /* offset: 0x00001020*/
  vuint8_t RESERVED61[28];
  vuint32_t CTRLDESCL57_1;       /* offset: 0x00001040*/
  vuint32_t CTRLDESCL57_2;       /* offset: 0x00001044*/
  vuint32_t CTRLDESCL57_3;       /* offset: 0x00001048*/
  vuint32_t CTRLDESCL57_4;       /* offset: 0x0000104C*/
  vuint32_t CTRLDESCL57_5;       /* offset: 0x00001050*/
  vuint32_t CTRLDESCL57_6;       /* offset: 0x00001054*/
  vuint32_t CTRLDESCL57_7;       /* offset: 0x00001058*/
  vuint32_t CTRLDESCL57_8;       /* offset: 0x0000105C*/
  vuint32_t CTRLDESCL57_9;       /* offset: 0x00001060*/
  vuint8_t RESERVED62[28];
  vuint32_t CTRLDESCL58_1;       /* offset: 0x00001080*/
  vuint32_t CTRLDESCL58_2;       /* offset: 0x00001084*/
  vuint32_t CTRLDESCL58_3;       /* offset: 0x00001088*/
  vuint32_t CTRLDESCL58_4;       /* offset: 0x0000108C*/
  vuint32_t CTRLDESCL58_5;       /* offset: 0x00001090*/
  vuint32_t CTRLDESCL58_6;       /* offset: 0x00001094*/
  vuint32_t CTRLDESCL58_7;       /* offset: 0x00001098*/
  vuint32_t CTRLDESCL58_8;       /* offset: 0x0000109C*/
  vuint32_t CTRLDESCL58_9;       /* offset: 0x000010A0*/
  vuint8_t RESERVED63[28];
  vuint32_t CTRLDESCL59_1;       /* offset: 0x000010C0*/
  vuint32_t CTRLDESCL59_2;       /* offset: 0x000010C4*/
  vuint32_t CTRLDESCL59_3;       /* offset: 0x000010C8*/
  vuint32_t CTRLDESCL59_4;       /* offset: 0x000010CC*/
  vuint32_t CTRLDESCL59_5;       /* offset: 0x000010D0*/
  vuint32_t CTRLDESCL59_6;       /* offset: 0x000010D4*/
  vuint32_t CTRLDESCL59_7;       /* offset: 0x000010D8*/
  vuint32_t CTRLDESCL59_8;       /* offset: 0x000010DC*/
  vuint32_t CTRLDESCL59_9;       /* offset: 0x000010E0*/
  vuint8_t RESERVED64[28];
  vuint32_t CTRLDESCL60_1;       /* offset: 0x00001100*/
  vuint32_t CTRLDESCL60_2;       /* offset: 0x00001104*/
  vuint32_t CTRLDESCL60_3;       /* offset: 0x00001108*/
  vuint32_t CTRLDESCL60_4;       /* offset: 0x0000110C*/
  vuint32_t CTRLDESCL60_5;       /* offset: 0x00001110*/
  vuint32_t CTRLDESCL60_6;       /* offset: 0x00001114*/
  vuint32_t CTRLDESCL60_7;       /* offset: 0x00001118*/
  vuint32_t CTRLDESCL60_8;       /* offset: 0x0000111C*/
  vuint32_t CTRLDESCL60_9;       /* offset: 0x00001120*/
  vuint8_t RESERVED65[28];
  vuint32_t CTRLDESCL61_1;       /* offset: 0x00001140*/
  vuint32_t CTRLDESCL61_2;       /* offset: 0x00001144*/
  vuint32_t CTRLDESCL61_3;       /* offset: 0x00001148*/
  vuint32_t CTRLDESCL61_4;       /* offset: 0x0000114C*/
  vuint32_t CTRLDESCL61_5;       /* offset: 0x00001150*/
  vuint32_t CTRLDESCL61_6;       /* offset: 0x00001154*/
  vuint32_t CTRLDESCL61_7;       /* offset: 0x00001158*/
  vuint32_t CTRLDESCL61_8;       /* offset: 0x0000115C*/
  vuint32_t CTRLDESCL61_9;       /* offset: 0x00001160*/
  vuint8_t RESERVED66[28];
  vuint32_t CTRLDESCL62_1;       /* offset: 0x00001180*/
  vuint32_t CTRLDESCL62_2;       /* offset: 0x00001184*/
  vuint32_t CTRLDESCL62_3;       /* offset: 0x00001188*/
  vuint32_t CTRLDESCL62_4;       /* offset: 0x0000118C*/
  vuint32_t CTRLDESCL62_5;       /* offset: 0x00001190*/
  vuint32_t CTRLDESCL62_6;       /* offset: 0x00001194*/
  vuint32_t CTRLDESCL62_7;       /* offset: 0x00001198*/
  vuint32_t CTRLDESCL62_8;       /* offset: 0x0000119C*/
  vuint32_t CTRLDESCL62_9;       /* offset: 0x000011A0*/
  vuint8_t RESERVED67[28];
  vuint32_t CTRLDESCL63_1;       /* offset: 0x000011C0*/
  vuint32_t CTRLDESCL63_2;       /* offset: 0x000011C4*/
  vuint32_t CTRLDESCL63_3;       /* offset: 0x000011C8*/
  vuint32_t CTRLDESCL63_4;       /* offset: 0x000011CC*/
  vuint32_t CTRLDESCL63_5;       /* offset: 0x000011D0*/
  vuint32_t CTRLDESCL63_6;       /* offset: 0x000011D4*/
  vuint32_t CTRLDESCL63_7;       /* offset: 0x000011D8*/
  vuint32_t CTRLDESCL63_8;       /* offset: 0x000011DC*/
  vuint32_t CTRLDESCL63_9;       /* offset: 0x000011E0*/
  vuint8_t RESERVED68[3612];
  vuint32_t CLUT[0x2000];      	 /* offset: 0x00002000*/
  vuint32_t GAMMARED[0x400];     /* offset: 0x00004000*/
  vuint32_t GAMMAGREEN[0x400];   /* offset: 0x00004400*/
  vuint32_t GAMMABLUE[0x400];    /* offset: 0x00004800*/
  vuint32_t GAMMACURSOR[0x400];  /* offset: 0x00004C00*/
} volatile *DCU_MemMapPtr;

/* DCU Internal RAM per module */

#define DCU_GAMMARED_ADDR32(dcu_id)     (DCU_BASE_ADDRESS[(dcu_id)] + DCU_GAMMARED_OFFSET)
#define DCU0_GAMMARED          	        (DCU0_BASE + DCU_GAMMARED_OFFSET)
#define DCU1_GAMMARED          	        (DCU1_BASE + DCU_GAMMARED_OFFSET)

#define DCU_GAMMAGREEN_ADDR32(dcu_id)   (DCU_BASE_ADDRESS[(dcu_id)] + DCU_GAMMAGREEN_OFFSET)
#define DCU0_GAMMAGREEN                 (DCU0_BASE + DCU_GAMMAGREEN_OFFSET)
#define DCU1_GAMMAGREEN                 (DCU1_BASE + DCU_GAMMAGREEN_OFFSET)

#define DCU_GAMMABLUE_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_GAMMABLUE_OFFSET)
#define DCU0_GAMMABLUE                  (DCU0_BASE + DCU_GAMMABLUE_OFFSET)
#define DCU1_GAMMABLUE                  (DCU1_BASE + DCU_GAMMABLUE_OFFSET)

#define DCU_CURSOR_ADDR32(dcu_id)       (DCU_BASE_ADDRESS[(dcu_id)] + DCU_CURSOR_OFFSET)
#define DCU0_CURSOR                     (DCU0_BASE + DCU_CURSOR_OFFSET)
#define DCU1_CURSOR                     (DCU1_BASE + DCU_CURSOR_OFFSET)

#define DCU_CLUT_ADDR32(dcu_id)         (DCU_BASE_ADDRESS[(dcu_id)] + DCU_CLUT_OFFSET)
#define DCU0_CLUT                       (DCU0_BASE + DCU_CLUT_OFFSET)
#define DCU1_CLUT                       (DCU1_BASE + DCU_CLUT_OFFSET)
#define DCU_CLUT_MEMSIZE                (0x800)


/* DCU - Register offsets */
#define DCU_CTRLDESCCURSOR1_OFFSET      0x00000000
#define DCU_CTRLDESCCURSOR2_OFFSET      0x00000004
#define DCU_CTRLDESCCURSOR3_OFFSET      0x00000008
#define DCU_CTRLDESCCURSOR4_OFFSET      0x0000000C
#define DCU_MODE_OFFSET                 0x00000010
#define DCU_BGND_OFFSET                 0x00000014
#define DCU_DISP_SIZE_OFFSET            0x00000018
#define DCU_HSYN_PARA_OFFSET            0x0000001C
#define DCU_VSYN_PARA_OFFSET            0x00000020
#define DCU_SYNPOL_OFFSET               0x00000024
#define DCU_THRESHOLD_OFFSET            0x00000028
#define DCU_INT_STATUS_OFFSET           0x0000002C
#define DCU_INT_MASK_OFFSET             0x00000030
#define DCU_COLBAR_1_OFFSET             0x00000034
#define DCU_COLBAR_2_OFFSET             0x00000038
#define DCU_COLBAR_3_OFFSET             0x0000003C
#define DCU_COLBAR_4_OFFSET             0x00000040
#define DCU_COLBAR_5_OFFSET             0x00000044
#define DCU_COLBAR_6_OFFSET             0x00000048
#define DCU_COLBAR_7_OFFSET             0x0000004C
#define DCU_COLBAR_8_OFFSET             0x00000050
#define DCU_DIV_RATIO_OFFSET            0x00000054
#if (1 == DCU_SAFETY_FUNCTIONALITY)
#define DCU_SIGN_CALC1_OFFSET           0x00000058
#define DCU_SIGN_CALC2_OFFSET           0x0000005C
#define DCU_CRC_VAL_OFFSET              0x00000060
#endif /*(DCU_SAFETY_FUNCTIONALITY)*/
#define DCU_PDI_STATUS_OFFSET           0x00000064
#define DCU_MASK_PDI_STATUS_OFFSET      0x00000068
#define DCU_PARR_ERR_STATUS1_OFFSET     0x0000006C
#define DCU_PARR_ERR_STATUS2_OFFSET     0x00000070
#define DCU_PARR_ERR_STATUS3_OFFSET     0x0000007C
#define DCU_MASK_PARR_ERR_STATUS1_OFFSET 0x00000080
#define DCU_MASK_PARR_ERR_STATUS2_OFFSET 0x00000084
#define DCU_MASK_PARR_ERR_STATUS3_OFFSET 0x00000090
#define DCU_THRESHOLD_INP_BUF_1_OFFSET  0x00000094
#define DCU_THRESHOLD_INP_BUF_2_OFFSET  0x00000098
#define DCU_THRESHOLD_INP_BUF_3_OFFSET  0x0000009C
#define DCU_LUMA_COMP_OFFSET            0x000000A0
#define DCU_CHROMA_RED_OFFSET           0x000000A4
#define DCU_CHROMA_GREEN_OFFSET         0x000000A8
#define DCU_CHROMA_BLUE_OFFSET          0x000000AC
#if (1 == DCU_SAFETY_FUNCTIONALITY)
#define DCU_CRC_POS_OFFSET              0x000000B0
#endif /*(DCU_SAFETY_FUNCTIONALITY)*/
#define DCU_LYR_INTPOL_EN_OFFSET        0x000000B4
#define DCU_LYR_LUMA_COMP_OFFSET        0x000000B8
#define DCU_LYR_CHROMA_RED_OFFSET       0x000000BC
#define DCU_LYR_CHROMA_GREEN_OFFSET     0x000000C0
#define DCU_LYR_CHROMA_BLUE_OFFSET      0x000000C4
#define DCU_COMP_IMSIZE_OFFSET          0x000000C8
#define DCU_UPDATE_MODE_OFFSET          0x000000CC
#define DCU_UNDERRUN_OFFSET             0x000000D0
#if (1 == DCU_WRITEBACK_FUNCTIONALITY)
  #define DCU_WRITEBACK_ADDR_OFFSET     0x000000D4
  #define DCU_WRITEBACK_CTRL_OFFSET     0x000000D8
  #define DCU_WRITEBACK_STAT_OFFSET     0x000000DC
#endif /* DCU_WRITEBACK_FUNCTIONALITY */
#if (1 == DCU_SAFETY_FUNCTIONALITY)
  #define DCU_CRC_FRM_CTRL_OFFSET         0x000000E0
  #define DCU_CRC_FRM_VAL_OFFSET          0x000000E4
#endif /*(DCU_SAFETY_FUNCTIONALITY)*/
#define DCU_GPR_OFFSET                  0x00000100
#define DCU_SLR_L0_OFFSET               0x00000104
#define DCU_SLR_L1_OFFSET               0x00000108
#define DCU_SLR_DISP_SIZE_OFFSET        0x0000010C
#define DCU_SLR_HVSYNC_PARA_OFFSET      0x00000110
#define DCU_SLR_POL_OFFSET              0x00000114
#define DCU_SLR_L0TRANSP_OFFSET         0x00000118
#define DCU_SLR_L1TRANSP_OFFSET         0x0000011C
#ifdef DCU_HUD_FUNCTIONALITY
  #define DCU_SLR_HUD_OFFSET            0x00000120
#endif /* DCU_HUD_FUNCTIONALITY */

#define DCU_CTRLDESCL0_1_OFFSET         0x00000200
#define DCU_CTRLDESCL0_2_OFFSET         0x00000204
#define DCU_CTRLDESCL0_3_OFFSET         0x00000208
#define DCU_CTRLDESCL0_4_OFFSET         0x0000020C
#define DCU_CTRLDESCL0_5_OFFSET         0x00000210
#define DCU_CTRLDESCL0_6_OFFSET         0x00000214
#define DCU_CTRLDESCL0_7_OFFSET         0x00000218
#define DCU_CTRLDESCL0_8_OFFSET         0x0000021C
#define DCU_CTRLDESCL0_9_OFFSET         0x00000220
#define DCU_CTRLDESCL0_10_OFFSET        0x00000224
#define DCU_CTRLDESCL0_11_OFFSET        0x00000228

#define DCU_CTRLDESCL1_1_OFFSET         0x00000240
#define DCU_CTRLDESCL1_2_OFFSET         0x00000244
#define DCU_CTRLDESCL1_3_OFFSET         0x00000248
#define DCU_CTRLDESCL1_4_OFFSET         0x0000024C
#define DCU_CTRLDESCL1_5_OFFSET         0x00000250
#define DCU_CTRLDESCL1_6_OFFSET         0x00000254
#define DCU_CTRLDESCL1_7_OFFSET         0x00000258
#define DCU_CTRLDESCL1_8_OFFSET         0x0000025C
#define DCU_CTRLDESCL1_9_OFFSET         0x00000260
#define DCU_CTRLDESCL1_10_OFFSET        0x00000264
#define DCU_CTRLDESCL1_11_OFFSET        0x00000268

#define DCU_CTRLDESCL2_1_OFFSET         0x00000280
#define DCU_CTRLDESCL2_2_OFFSET         0x00000284
#define DCU_CTRLDESCL2_3_OFFSET         0x00000288
#define DCU_CTRLDESCL2_4_OFFSET         0x0000028C
#define DCU_CTRLDESCL2_5_OFFSET         0x00000290
#define DCU_CTRLDESCL2_6_OFFSET         0x00000294
#define DCU_CTRLDESCL2_7_OFFSET         0x00000298
#define DCU_CTRLDESCL2_8_OFFSET         0x0000029C
#define DCU_CTRLDESCL2_9_OFFSET         0x000002A0
#define DCU_CTRLDESCL2_10_OFFSET        0x000002A4
#define DCU_CTRLDESCL2_11_OFFSET        0x000002A8

#define DCU_CTRLDESCL3_1_OFFSET         0x000002C0
#define DCU_CTRLDESCL3_2_OFFSET         0x000002C4
#define DCU_CTRLDESCL3_3_OFFSET         0x000002C8
#define DCU_CTRLDESCL3_4_OFFSET         0x000002CC
#define DCU_CTRLDESCL3_5_OFFSET         0x000002D0
#define DCU_CTRLDESCL3_6_OFFSET         0x000002D4
#define DCU_CTRLDESCL3_7_OFFSET         0x000002D8
#define DCU_CTRLDESCL3_8_OFFSET         0x000002DC
#define DCU_CTRLDESCL3_9_OFFSET         0x000002E0
#define DCU_CTRLDESCL3_10_OFFSET        0x000002E4
#define DCU_CTRLDESCL3_11_OFFSET        0x000002E8

#define DCU_CTRLDESCL4_1_OFFSET         0x00000300
#define DCU_CTRLDESCL4_2_OFFSET         0x00000304
#define DCU_CTRLDESCL4_3_OFFSET         0x00000308
#define DCU_CTRLDESCL4_4_OFFSET         0x0000030C
#define DCU_CTRLDESCL4_5_OFFSET         0x00000310
#define DCU_CTRLDESCL4_6_OFFSET         0x00000314
#define DCU_CTRLDESCL4_7_OFFSET         0x00000318
#define DCU_CTRLDESCL4_8_OFFSET         0x0000031C
#define DCU_CTRLDESCL4_9_OFFSET         0x00000320
#define DCU_CTRLDESCL4_10_OFFSET        0x00000324
#define DCU_CTRLDESCL4_11_OFFSET        0x00000328

#define DCU_CTRLDESCL5_1_OFFSET         0x00000340
#define DCU_CTRLDESCL5_2_OFFSET         0x00000344
#define DCU_CTRLDESCL5_3_OFFSET         0x00000348
#define DCU_CTRLDESCL5_4_OFFSET         0x0000034C
#define DCU_CTRLDESCL5_5_OFFSET         0x00000350
#define DCU_CTRLDESCL5_6_OFFSET         0x00000354
#define DCU_CTRLDESCL5_7_OFFSET         0x00000358
#define DCU_CTRLDESCL5_8_OFFSET         0x0000035C
#define DCU_CTRLDESCL5_9_OFFSET         0x00000360
#define DCU_CTRLDESCL5_10_OFFSET        0x00000364
#define DCU_CTRLDESCL5_11_OFFSET        0x00000368

#define DCU_CTRLDESCL6_1_OFFSET         0x00000380
#define DCU_CTRLDESCL6_2_OFFSET         0x00000384
#define DCU_CTRLDESCL6_3_OFFSET         0x00000388
#define DCU_CTRLDESCL6_4_OFFSET         0x0000038C
#define DCU_CTRLDESCL6_5_OFFSET         0x00000390
#define DCU_CTRLDESCL6_6_OFFSET         0x00000394
#define DCU_CTRLDESCL6_7_OFFSET         0x00000398
#define DCU_CTRLDESCL6_8_OFFSET         0x0000039C
#define DCU_CTRLDESCL6_9_OFFSET         0x000003A0
#define DCU_CTRLDESCL6_10_OFFSET        0x000003A4
#define DCU_CTRLDESCL6_11_OFFSET        0x000003A8

#define DCU_CTRLDESCL7_1_OFFSET         0x000003C0
#define DCU_CTRLDESCL7_2_OFFSET         0x000003C4
#define DCU_CTRLDESCL7_3_OFFSET         0x000003C8
#define DCU_CTRLDESCL7_4_OFFSET         0x000003CC
#define DCU_CTRLDESCL7_5_OFFSET         0x000003D0
#define DCU_CTRLDESCL7_6_OFFSET         0x000003D4
#define DCU_CTRLDESCL7_7_OFFSET         0x000003D8
#define DCU_CTRLDESCL7_8_OFFSET         0x000003DC
#define DCU_CTRLDESCL7_9_OFFSET         0x000003E0
#define DCU_CTRLDESCL7_10_OFFSET        0x000003E4
#define DCU_CTRLDESCL7_11_OFFSET        0x000003E8

#define DCU_CTRLDESCL8_1_OFFSET         0x00000400
#define DCU_CTRLDESCL8_2_OFFSET         0x00000404
#define DCU_CTRLDESCL8_3_OFFSET         0x00000408
#define DCU_CTRLDESCL8_4_OFFSET         0x0000040C
#define DCU_CTRLDESCL8_5_OFFSET         0x00000410
#define DCU_CTRLDESCL8_6_OFFSET         0x00000414
#define DCU_CTRLDESCL8_7_OFFSET         0x00000418
#define DCU_CTRLDESCL8_8_OFFSET         0x0000041C
#define DCU_CTRLDESCL8_9_OFFSET         0x00000420
#define DCU_CTRLDESCL8_10_OFFSET        0x00000424
#define DCU_CTRLDESCL8_11_OFFSET        0x00000428

#define DCU_CTRLDESCL9_1_OFFSET         0x00000440
#define DCU_CTRLDESCL9_2_OFFSET         0x00000444
#define DCU_CTRLDESCL9_3_OFFSET         0x00000448
#define DCU_CTRLDESCL9_4_OFFSET         0x0000044C
#define DCU_CTRLDESCL9_5_OFFSET         0x00000450
#define DCU_CTRLDESCL9_6_OFFSET         0x00000454
#define DCU_CTRLDESCL9_7_OFFSET         0x00000458
#define DCU_CTRLDESCL9_8_OFFSET         0x0000045C
#define DCU_CTRLDESCL9_9_OFFSET         0x00000460
#define DCU_CTRLDESCL9_10_OFFSET        0x00000464
#define DCU_CTRLDESCL9_11_OFFSET        0x00000468

#define DCU_CTRLDESCL10_1_OFFSET        0x00000480
#define DCU_CTRLDESCL10_2_OFFSET        0x00000484
#define DCU_CTRLDESCL10_3_OFFSET        0x00000488
#define DCU_CTRLDESCL10_4_OFFSET        0x0000048C
#define DCU_CTRLDESCL10_5_OFFSET        0x00000490
#define DCU_CTRLDESCL10_6_OFFSET        0x00000494
#define DCU_CTRLDESCL10_7_OFFSET        0x00000498
#define DCU_CTRLDESCL10_8_OFFSET        0x0000049C
#define DCU_CTRLDESCL10_9_OFFSET        0x000004A0
#define DCU_CTRLDESCL10_10_OFFSET       0x000004A4
#define DCU_CTRLDESCL10_11_OFFSET       0x000004A8

#define DCU_CTRLDESCL11_1_OFFSET        0x000004C0
#define DCU_CTRLDESCL11_2_OFFSET        0x000004C4
#define DCU_CTRLDESCL11_3_OFFSET        0x000004C8
#define DCU_CTRLDESCL11_4_OFFSET        0x000004CC
#define DCU_CTRLDESCL11_5_OFFSET        0x000004D0
#define DCU_CTRLDESCL11_6_OFFSET        0x000004D4
#define DCU_CTRLDESCL11_7_OFFSET        0x000004D8
#define DCU_CTRLDESCL11_8_OFFSET        0x000004DC
#define DCU_CTRLDESCL11_9_OFFSET        0x000004E0
#define DCU_CTRLDESCL11_10_OFFSET       0x000004E4
#define DCU_CTRLDESCL11_11_OFFSET       0x000004E8

#define DCU_CTRLDESCL12_1_OFFSET        0x00000500
#define DCU_CTRLDESCL12_2_OFFSET        0x00000504
#define DCU_CTRLDESCL12_3_OFFSET        0x00000508
#define DCU_CTRLDESCL12_4_OFFSET        0x0000050C
#define DCU_CTRLDESCL12_5_OFFSET        0x00000510
#define DCU_CTRLDESCL12_6_OFFSET        0x00000514
#define DCU_CTRLDESCL12_7_OFFSET        0x00000518
#define DCU_CTRLDESCL12_8_OFFSET        0x0000051C
#define DCU_CTRLDESCL12_9_OFFSET        0x00000520
#define DCU_CTRLDESCL12_10_OFFSET       0x00000524
#define DCU_CTRLDESCL12_11_OFFSET       0x00000528

#define DCU_CTRLDESCL13_1_OFFSET        0x00000540
#define DCU_CTRLDESCL13_2_OFFSET        0x00000544
#define DCU_CTRLDESCL13_3_OFFSET        0x00000548
#define DCU_CTRLDESCL13_4_OFFSET        0x0000054C
#define DCU_CTRLDESCL13_5_OFFSET        0x00000550
#define DCU_CTRLDESCL13_6_OFFSET        0x00000554
#define DCU_CTRLDESCL13_7_OFFSET        0x00000558
#define DCU_CTRLDESCL13_8_OFFSET        0x0000055C
#define DCU_CTRLDESCL13_9_OFFSET        0x00000560
#define DCU_CTRLDESCL13_10_OFFSET       0x00000564
#define DCU_CTRLDESCL13_11_OFFSET       0x00000568

#define DCU_CTRLDESCL14_1_OFFSET        0x00000580
#define DCU_CTRLDESCL14_2_OFFSET        0x00000584
#define DCU_CTRLDESCL14_3_OFFSET        0x00000588
#define DCU_CTRLDESCL14_4_OFFSET        0x0000058C
#define DCU_CTRLDESCL14_5_OFFSET        0x00000590
#define DCU_CTRLDESCL14_6_OFFSET        0x00000594
#define DCU_CTRLDESCL14_7_OFFSET        0x00000598
#define DCU_CTRLDESCL14_8_OFFSET        0x0000059C
#define DCU_CTRLDESCL14_9_OFFSET        0x000005A0
#define DCU_CTRLDESCL14_10_OFFSET       0x000005A4
#define DCU_CTRLDESCL14_11_OFFSET       0x000005A8

#define DCU_CTRLDESCL15_1_OFFSET        0x000005C0
#define DCU_CTRLDESCL15_2_OFFSET        0x000005C4
#define DCU_CTRLDESCL15_3_OFFSET        0x000005C8
#define DCU_CTRLDESCL15_4_OFFSET        0x000005CC
#define DCU_CTRLDESCL15_5_OFFSET        0x000005D0
#define DCU_CTRLDESCL15_6_OFFSET        0x000005D4
#define DCU_CTRLDESCL15_7_OFFSET        0x000005D8
#define DCU_CTRLDESCL15_8_OFFSET        0x000005DC
#define DCU_CTRLDESCL15_9_OFFSET        0x000005E0
#define DCU_CTRLDESCL15_10_OFFSET       0x000005E4
#define DCU_CTRLDESCL15_11_OFFSET       0x000005E8

#define DCU_CTRLDESCL16_1_OFFSET        0x00000600
#define DCU_CTRLDESCL16_2_OFFSET        0x00000604
#define DCU_CTRLDESCL16_3_OFFSET        0x00000608
#define DCU_CTRLDESCL16_4_OFFSET        0x0000060C
#define DCU_CTRLDESCL16_5_OFFSET        0x00000610
#define DCU_CTRLDESCL16_6_OFFSET        0x00000614
#define DCU_CTRLDESCL16_7_OFFSET        0x00000618
#define DCU_CTRLDESCL16_8_OFFSET        0x0000061C
#define DCU_CTRLDESCL16_9_OFFSET        0x00000620
#define DCU_CTRLDESCL16_10_OFFSET       0x00000624
#define DCU_CTRLDESCL16_11_OFFSET       0x00000628

#define DCU_CTRLDESCL17_1_OFFSET        0x00000640
#define DCU_CTRLDESCL17_2_OFFSET        0x00000644
#define DCU_CTRLDESCL17_3_OFFSET        0x00000648
#define DCU_CTRLDESCL17_4_OFFSET        0x0000064C
#define DCU_CTRLDESCL17_5_OFFSET        0x00000650
#define DCU_CTRLDESCL17_6_OFFSET        0x00000654
#define DCU_CTRLDESCL17_7_OFFSET        0x00000658
#define DCU_CTRLDESCL17_8_OFFSET        0x0000065C
#define DCU_CTRLDESCL17_9_OFFSET        0x00000660
#define DCU_CTRLDESCL17_10_OFFSET       0x00000664
#define DCU_CTRLDESCL17_11_OFFSET       0x00000668

#define DCU_CTRLDESCL18_1_OFFSET        0x00000680
#define DCU_CTRLDESCL18_2_OFFSET        0x00000684
#define DCU_CTRLDESCL18_3_OFFSET        0x00000688
#define DCU_CTRLDESCL18_4_OFFSET        0x0000068C
#define DCU_CTRLDESCL18_5_OFFSET        0x00000690
#define DCU_CTRLDESCL18_6_OFFSET        0x00000694
#define DCU_CTRLDESCL18_7_OFFSET        0x00000698
#define DCU_CTRLDESCL18_8_OFFSET        0x0000069C
#define DCU_CTRLDESCL18_9_OFFSET        0x000006A0
#define DCU_CTRLDESCL18_10_OFFSET       0x000006A4
#define DCU_CTRLDESCL18_11_OFFSET       0x000006A8

#define DCU_CTRLDESCL19_1_OFFSET        0x000006C0
#define DCU_CTRLDESCL19_2_OFFSET        0x000006C4
#define DCU_CTRLDESCL19_3_OFFSET        0x000006C8
#define DCU_CTRLDESCL19_4_OFFSET        0x000006CC
#define DCU_CTRLDESCL19_5_OFFSET        0x000006D0
#define DCU_CTRLDESCL19_6_OFFSET        0x000006D4
#define DCU_CTRLDESCL19_7_OFFSET        0x000006D8
#define DCU_CTRLDESCL19_8_OFFSET        0x000006DC
#define DCU_CTRLDESCL19_9_OFFSET        0x000006E0
#define DCU_CTRLDESCL19_10_OFFSET       0x000006E4
#define DCU_CTRLDESCL19_11_OFFSET       0x000006E8

#define DCU_CTRLDESCL20_1_OFFSET        0x00000700
#define DCU_CTRLDESCL20_2_OFFSET        0x00000704
#define DCU_CTRLDESCL20_3_OFFSET        0x00000708
#define DCU_CTRLDESCL20_4_OFFSET        0x0000070C
#define DCU_CTRLDESCL20_5_OFFSET        0x00000710
#define DCU_CTRLDESCL20_6_OFFSET        0x00000714
#define DCU_CTRLDESCL20_7_OFFSET        0x00000718
#define DCU_CTRLDESCL20_8_OFFSET        0x0000071C
#define DCU_CTRLDESCL20_9_OFFSET        0x00000720
#define DCU_CTRLDESCL20_10_OFFSET       0x00000724
#define DCU_CTRLDESCL20_11_OFFSET       0x00000728

#define DCU_CTRLDESCL21_1_OFFSET        0x00000740
#define DCU_CTRLDESCL21_2_OFFSET        0x00000744
#define DCU_CTRLDESCL21_3_OFFSET        0x00000748
#define DCU_CTRLDESCL21_4_OFFSET        0x0000074C
#define DCU_CTRLDESCL21_5_OFFSET        0x00000750
#define DCU_CTRLDESCL21_6_OFFSET        0x00000754
#define DCU_CTRLDESCL21_7_OFFSET        0x00000758
#define DCU_CTRLDESCL21_8_OFFSET        0x0000075C
#define DCU_CTRLDESCL21_9_OFFSET        0x00000760
#define DCU_CTRLDESCL21_10_OFFSET       0x00000764
#define DCU_CTRLDESCL21_11_OFFSET       0x00000768

#define DCU_CTRLDESCL22_1_OFFSET        0x00000780
#define DCU_CTRLDESCL22_2_OFFSET        0x00000784
#define DCU_CTRLDESCL22_3_OFFSET        0x00000788
#define DCU_CTRLDESCL22_4_OFFSET        0x0000078C
#define DCU_CTRLDESCL22_5_OFFSET        0x00000790
#define DCU_CTRLDESCL22_6_OFFSET        0x00000794
#define DCU_CTRLDESCL22_7_OFFSET        0x00000798
#define DCU_CTRLDESCL22_8_OFFSET        0x0000079C
#define DCU_CTRLDESCL22_9_OFFSET        0x000007A0
#define DCU_CTRLDESCL22_10_OFFSET       0x000007A4
#define DCU_CTRLDESCL22_11_OFFSET       0x000007A8

#define DCU_CTRLDESCL23_1_OFFSET        0x000007C0
#define DCU_CTRLDESCL23_2_OFFSET        0x000007C4
#define DCU_CTRLDESCL23_3_OFFSET        0x000007C8
#define DCU_CTRLDESCL23_4_OFFSET        0x000007CC
#define DCU_CTRLDESCL23_5_OFFSET        0x000007D0
#define DCU_CTRLDESCL23_6_OFFSET        0x000007D4
#define DCU_CTRLDESCL23_7_OFFSET        0x000007D8
#define DCU_CTRLDESCL23_8_OFFSET        0x000007DC
#define DCU_CTRLDESCL23_9_OFFSET        0x000007E0
#define DCU_CTRLDESCL23_10_OFFSET       0x000007E4
#define DCU_CTRLDESCL23_11_OFFSET       0x000007E8

#define DCU_CTRLDESCL24_1_OFFSET        0x00000800
#define DCU_CTRLDESCL24_2_OFFSET        0x00000804
#define DCU_CTRLDESCL24_3_OFFSET        0x00000808
#define DCU_CTRLDESCL24_4_OFFSET        0x0000080C
#define DCU_CTRLDESCL24_5_OFFSET        0x00000810
#define DCU_CTRLDESCL24_6_OFFSET        0x00000814
#define DCU_CTRLDESCL24_7_OFFSET        0x00000818
#define DCU_CTRLDESCL24_8_OFFSET        0x0000081C
#define DCU_CTRLDESCL24_9_OFFSET        0x00000820
#define DCU_CTRLDESCL24_10_OFFSET       0x00000824
#define DCU_CTRLDESCL24_11_OFFSET       0x00000828

#define DCU_CTRLDESCL25_1_OFFSET        0x00000840
#define DCU_CTRLDESCL25_2_OFFSET        0x00000844
#define DCU_CTRLDESCL25_3_OFFSET        0x00000848
#define DCU_CTRLDESCL25_4_OFFSET        0x0000084C
#define DCU_CTRLDESCL25_5_OFFSET        0x00000850
#define DCU_CTRLDESCL25_6_OFFSET        0x00000854
#define DCU_CTRLDESCL25_7_OFFSET        0x00000858
#define DCU_CTRLDESCL25_8_OFFSET        0x0000085C
#define DCU_CTRLDESCL25_9_OFFSET        0x00000860
#define DCU_CTRLDESCL25_10_OFFSET       0x00000864
#define DCU_CTRLDESCL25_11_OFFSET       0x00000868

#define DCU_CTRLDESCL26_1_OFFSET        0x00000880
#define DCU_CTRLDESCL26_2_OFFSET        0x00000884
#define DCU_CTRLDESCL26_3_OFFSET        0x00000888
#define DCU_CTRLDESCL26_4_OFFSET        0x0000088C
#define DCU_CTRLDESCL26_5_OFFSET        0x00000890
#define DCU_CTRLDESCL26_6_OFFSET        0x00000894
#define DCU_CTRLDESCL26_7_OFFSET        0x00000898
#define DCU_CTRLDESCL26_8_OFFSET        0x0000089C
#define DCU_CTRLDESCL26_9_OFFSET        0x000008A0
#define DCU_CTRLDESCL26_10_OFFSET       0x000008A4
#define DCU_CTRLDESCL26_11_OFFSET       0x000008A8

#define DCU_CTRLDESCL27_1_OFFSET        0x000008C0
#define DCU_CTRLDESCL27_2_OFFSET        0x000008C4
#define DCU_CTRLDESCL27_3_OFFSET        0x000008C8
#define DCU_CTRLDESCL27_4_OFFSET        0x000008CC
#define DCU_CTRLDESCL27_5_OFFSET        0x000008D0
#define DCU_CTRLDESCL27_6_OFFSET        0x000008D4
#define DCU_CTRLDESCL27_7_OFFSET        0x000008D8
#define DCU_CTRLDESCL27_8_OFFSET        0x000008DC
#define DCU_CTRLDESCL27_9_OFFSET        0x000008E0
#define DCU_CTRLDESCL27_10_OFFSET       0x000008E4
#define DCU_CTRLDESCL27_11_OFFSET       0x000008E8

#define DCU_CTRLDESCL28_1_OFFSET        0x00000900
#define DCU_CTRLDESCL28_2_OFFSET        0x00000904
#define DCU_CTRLDESCL28_3_OFFSET        0x00000908
#define DCU_CTRLDESCL28_4_OFFSET        0x0000090C
#define DCU_CTRLDESCL28_5_OFFSET        0x00000910
#define DCU_CTRLDESCL28_6_OFFSET        0x00000914
#define DCU_CTRLDESCL28_7_OFFSET        0x00000918
#define DCU_CTRLDESCL28_8_OFFSET        0x0000091C
#define DCU_CTRLDESCL28_9_OFFSET        0x00000920
#define DCU_CTRLDESCL28_10_OFFSET       0x00000924
#define DCU_CTRLDESCL28_11_OFFSET       0x00000928

#define DCU_CTRLDESCL29_1_OFFSET        0x00000940
#define DCU_CTRLDESCL29_2_OFFSET        0x00000944
#define DCU_CTRLDESCL29_3_OFFSET        0x00000948
#define DCU_CTRLDESCL29_4_OFFSET        0x0000094C
#define DCU_CTRLDESCL29_5_OFFSET        0x00000950
#define DCU_CTRLDESCL29_6_OFFSET        0x00000954
#define DCU_CTRLDESCL29_7_OFFSET        0x00000958
#define DCU_CTRLDESCL29_8_OFFSET        0x0000095C
#define DCU_CTRLDESCL29_9_OFFSET        0x00000960
#define DCU_CTRLDESCL29_10_OFFSET       0x00000964
#define DCU_CTRLDESCL29_11_OFFSET       0x00000968

#define DCU_CTRLDESCL30_1_OFFSET        0x00000980
#define DCU_CTRLDESCL30_2_OFFSET        0x00000984
#define DCU_CTRLDESCL30_3_OFFSET        0x00000988
#define DCU_CTRLDESCL30_4_OFFSET        0x0000098C
#define DCU_CTRLDESCL30_5_OFFSET        0x00000990
#define DCU_CTRLDESCL30_6_OFFSET        0x00000994
#define DCU_CTRLDESCL30_7_OFFSET        0x00000998
#define DCU_CTRLDESCL30_8_OFFSET        0x0000099C
#define DCU_CTRLDESCL30_9_OFFSET        0x000009A0
#define DCU_CTRLDESCL30_10_OFFSET       0x000009A4
#define DCU_CTRLDESCL30_11_OFFSET       0x000009A8

#define DCU_CTRLDESCL31_1_OFFSET        0x000009C0
#define DCU_CTRLDESCL31_2_OFFSET        0x000009C4
#define DCU_CTRLDESCL31_3_OFFSET        0x000009C8
#define DCU_CTRLDESCL31_4_OFFSET        0x000009CC
#define DCU_CTRLDESCL31_5_OFFSET        0x000009D0
#define DCU_CTRLDESCL31_6_OFFSET        0x000009D4
#define DCU_CTRLDESCL31_7_OFFSET        0x000009D8
#define DCU_CTRLDESCL31_8_OFFSET        0x000009DC
#define DCU_CTRLDESCL31_9_OFFSET        0x000009E0
#define DCU_CTRLDESCL31_10_OFFSET       0x000009E4
#define DCU_CTRLDESCL31_11_OFFSET       0x000009E8

#if (32 != DCU_LAYERS_NUM_MAX)
#define DCU_CTRLDESCL32_1_OFFSET        0x00000A00
#define DCU_CTRLDESCL32_2_OFFSET        0x00000A04
#define DCU_CTRLDESCL32_3_OFFSET        0x00000A08
#define DCU_CTRLDESCL32_4_OFFSET        0x00000A0C
#define DCU_CTRLDESCL32_5_OFFSET        0x00000A10
#define DCU_CTRLDESCL32_6_OFFSET        0x00000A14
#define DCU_CTRLDESCL32_7_OFFSET        0x00000A18
#define DCU_CTRLDESCL32_8_OFFSET        0x00000A1C
#define DCU_CTRLDESCL32_9_OFFSET        0x00000A20
#define DCU_CTRLDESCL33_1_OFFSET        0x00000A40
#define DCU_CTRLDESCL33_2_OFFSET        0x00000A44
#define DCU_CTRLDESCL33_3_OFFSET        0x00000A48
#define DCU_CTRLDESCL33_4_OFFSET        0x00000A4C
#define DCU_CTRLDESCL33_5_OFFSET        0x00000A50
#define DCU_CTRLDESCL33_6_OFFSET        0x00000A54
#define DCU_CTRLDESCL33_7_OFFSET        0x00000A58
#define DCU_CTRLDESCL33_8_OFFSET        0x00000A5C
#define DCU_CTRLDESCL33_9_OFFSET        0x00000A60
#define DCU_CTRLDESCL34_1_OFFSET        0x00000A80
#define DCU_CTRLDESCL34_2_OFFSET        0x00000A84
#define DCU_CTRLDESCL34_3_OFFSET        0x00000A88
#define DCU_CTRLDESCL34_4_OFFSET        0x00000A8C
#define DCU_CTRLDESCL34_5_OFFSET        0x00000A90
#define DCU_CTRLDESCL34_6_OFFSET        0x00000A94
#define DCU_CTRLDESCL34_7_OFFSET        0x00000A98
#define DCU_CTRLDESCL34_8_OFFSET        0x00000A9C
#define DCU_CTRLDESCL34_9_OFFSET        0x00000AA0
#define DCU_CTRLDESCL35_1_OFFSET        0x00000AC0
#define DCU_CTRLDESCL35_2_OFFSET        0x00000AC4
#define DCU_CTRLDESCL35_3_OFFSET        0x00000AC8
#define DCU_CTRLDESCL35_4_OFFSET        0x00000ACC
#define DCU_CTRLDESCL35_5_OFFSET        0x00000AD0
#define DCU_CTRLDESCL35_6_OFFSET        0x00000AD4
#define DCU_CTRLDESCL35_7_OFFSET        0x00000AD8
#define DCU_CTRLDESCL35_8_OFFSET        0x00000ADC
#define DCU_CTRLDESCL35_9_OFFSET        0x00000AE0
#define DCU_CTRLDESCL36_1_OFFSET        0x00000B00
#define DCU_CTRLDESCL36_2_OFFSET        0x00000B04
#define DCU_CTRLDESCL36_3_OFFSET        0x00000B08
#define DCU_CTRLDESCL36_4_OFFSET        0x00000B0C
#define DCU_CTRLDESCL36_5_OFFSET        0x00000B10
#define DCU_CTRLDESCL36_6_OFFSET        0x00000B14
#define DCU_CTRLDESCL36_7_OFFSET        0x00000B18
#define DCU_CTRLDESCL36_8_OFFSET        0x00000B1C
#define DCU_CTRLDESCL36_9_OFFSET        0x00000B20
#define DCU_CTRLDESCL37_1_OFFSET        0x00000B40
#define DCU_CTRLDESCL37_2_OFFSET        0x00000B44
#define DCU_CTRLDESCL37_3_OFFSET        0x00000B48
#define DCU_CTRLDESCL37_4_OFFSET        0x00000B4C
#define DCU_CTRLDESCL37_5_OFFSET        0x00000B50
#define DCU_CTRLDESCL37_6_OFFSET        0x00000B54
#define DCU_CTRLDESCL37_7_OFFSET        0x00000B58
#define DCU_CTRLDESCL37_8_OFFSET        0x00000B5C
#define DCU_CTRLDESCL37_9_OFFSET        0x00000B60
#define DCU_CTRLDESCL38_1_OFFSET        0x00000B80
#define DCU_CTRLDESCL38_2_OFFSET        0x00000B84
#define DCU_CTRLDESCL38_3_OFFSET        0x00000B88
#define DCU_CTRLDESCL38_4_OFFSET        0x00000B8C
#define DCU_CTRLDESCL38_5_OFFSET        0x00000B90
#define DCU_CTRLDESCL38_6_OFFSET        0x00000B94
#define DCU_CTRLDESCL38_7_OFFSET        0x00000B98
#define DCU_CTRLDESCL38_8_OFFSET        0x00000B9C
#define DCU_CTRLDESCL38_9_OFFSET        0x00000BA0
#define DCU_CTRLDESCL39_1_OFFSET        0x00000BC0
#define DCU_CTRLDESCL39_2_OFFSET        0x00000BC4
#define DCU_CTRLDESCL39_3_OFFSET        0x00000BC8
#define DCU_CTRLDESCL39_4_OFFSET        0x00000BCC
#define DCU_CTRLDESCL39_5_OFFSET        0x00000BD0
#define DCU_CTRLDESCL39_6_OFFSET        0x00000BD4
#define DCU_CTRLDESCL39_7_OFFSET        0x00000BD8
#define DCU_CTRLDESCL39_8_OFFSET        0x00000BDC
#define DCU_CTRLDESCL39_9_OFFSET        0x00000BE0
#define DCU_CTRLDESCL40_1_OFFSET        0x00000C00
#define DCU_CTRLDESCL40_2_OFFSET        0x00000C04
#define DCU_CTRLDESCL40_3_OFFSET        0x00000C08
#define DCU_CTRLDESCL40_4_OFFSET        0x00000C0C
#define DCU_CTRLDESCL40_5_OFFSET        0x00000C10
#define DCU_CTRLDESCL40_6_OFFSET        0x00000C14
#define DCU_CTRLDESCL40_7_OFFSET        0x00000C18
#define DCU_CTRLDESCL40_8_OFFSET        0x00000C1C
#define DCU_CTRLDESCL40_9_OFFSET        0x00000C20
#define DCU_CTRLDESCL41_1_OFFSET        0x00000C40
#define DCU_CTRLDESCL41_2_OFFSET        0x00000C44
#define DCU_CTRLDESCL41_3_OFFSET        0x00000C48
#define DCU_CTRLDESCL41_4_OFFSET        0x00000C4C
#define DCU_CTRLDESCL41_5_OFFSET        0x00000C50
#define DCU_CTRLDESCL41_6_OFFSET        0x00000C54
#define DCU_CTRLDESCL41_7_OFFSET        0x00000C58
#define DCU_CTRLDESCL41_8_OFFSET        0x00000C5C
#define DCU_CTRLDESCL41_9_OFFSET        0x00000C60
#define DCU_CTRLDESCL42_1_OFFSET        0x00000C80
#define DCU_CTRLDESCL42_2_OFFSET        0x00000C84
#define DCU_CTRLDESCL42_3_OFFSET        0x00000C88
#define DCU_CTRLDESCL42_4_OFFSET        0x00000C8C
#define DCU_CTRLDESCL42_5_OFFSET        0x00000C90
#define DCU_CTRLDESCL42_6_OFFSET        0x00000C94
#define DCU_CTRLDESCL42_7_OFFSET        0x00000C98
#define DCU_CTRLDESCL42_8_OFFSET        0x00000C9C
#define DCU_CTRLDESCL42_9_OFFSET        0x00000CA0
#define DCU_CTRLDESCL43_1_OFFSET        0x00000CC0
#define DCU_CTRLDESCL43_2_OFFSET        0x00000CC4
#define DCU_CTRLDESCL43_3_OFFSET        0x00000CC8
#define DCU_CTRLDESCL43_4_OFFSET        0x00000CCC
#define DCU_CTRLDESCL43_5_OFFSET        0x00000CD0
#define DCU_CTRLDESCL43_6_OFFSET        0x00000CD4
#define DCU_CTRLDESCL43_7_OFFSET        0x00000CD8
#define DCU_CTRLDESCL43_8_OFFSET        0x00000CDC
#define DCU_CTRLDESCL43_9_OFFSET        0x00000CE0
#define DCU_CTRLDESCL44_1_OFFSET        0x00000D00
#define DCU_CTRLDESCL44_2_OFFSET        0x00000D04
#define DCU_CTRLDESCL44_3_OFFSET        0x00000D08
#define DCU_CTRLDESCL44_4_OFFSET        0x00000D0C
#define DCU_CTRLDESCL44_5_OFFSET        0x00000D10
#define DCU_CTRLDESCL44_6_OFFSET        0x00000D14
#define DCU_CTRLDESCL44_7_OFFSET        0x00000D18
#define DCU_CTRLDESCL44_8_OFFSET        0x00000D1C
#define DCU_CTRLDESCL44_9_OFFSET        0x00000D20
#define DCU_CTRLDESCL45_1_OFFSET        0x00000D40
#define DCU_CTRLDESCL45_2_OFFSET        0x00000D44
#define DCU_CTRLDESCL45_3_OFFSET        0x00000D48
#define DCU_CTRLDESCL45_4_OFFSET        0x00000D4C
#define DCU_CTRLDESCL45_5_OFFSET        0x00000D50
#define DCU_CTRLDESCL45_6_OFFSET        0x00000D54
#define DCU_CTRLDESCL45_7_OFFSET        0x00000D58
#define DCU_CTRLDESCL45_8_OFFSET        0x00000D5C
#define DCU_CTRLDESCL45_9_OFFSET        0x00000D60
#define DCU_CTRLDESCL46_1_OFFSET        0x00000D80
#define DCU_CTRLDESCL46_2_OFFSET        0x00000D84
#define DCU_CTRLDESCL46_3_OFFSET        0x00000D88
#define DCU_CTRLDESCL46_4_OFFSET        0x00000D8C
#define DCU_CTRLDESCL46_5_OFFSET        0x00000D90
#define DCU_CTRLDESCL46_6_OFFSET        0x00000D94
#define DCU_CTRLDESCL46_7_OFFSET        0x00000D98
#define DCU_CTRLDESCL46_8_OFFSET        0x00000D9C
#define DCU_CTRLDESCL46_9_OFFSET        0x00000DA0
#define DCU_CTRLDESCL47_1_OFFSET        0x00000DC0
#define DCU_CTRLDESCL47_2_OFFSET        0x00000DC4
#define DCU_CTRLDESCL47_3_OFFSET        0x00000DC8
#define DCU_CTRLDESCL47_4_OFFSET        0x00000DCC
#define DCU_CTRLDESCL47_5_OFFSET        0x00000DD0
#define DCU_CTRLDESCL47_6_OFFSET        0x00000DD4
#define DCU_CTRLDESCL47_7_OFFSET        0x00000DD8
#define DCU_CTRLDESCL47_8_OFFSET        0x00000DDC
#define DCU_CTRLDESCL47_9_OFFSET        0x00000DE0
#define DCU_CTRLDESCL48_1_OFFSET        0x00000E00
#define DCU_CTRLDESCL48_2_OFFSET        0x00000E04
#define DCU_CTRLDESCL48_3_OFFSET        0x00000E08
#define DCU_CTRLDESCL48_4_OFFSET        0x00000E0C
#define DCU_CTRLDESCL48_5_OFFSET        0x00000E10
#define DCU_CTRLDESCL48_6_OFFSET        0x00000E14
#define DCU_CTRLDESCL48_7_OFFSET        0x00000E18
#define DCU_CTRLDESCL48_8_OFFSET        0x00000E1C
#define DCU_CTRLDESCL48_9_OFFSET        0x00000E20
#define DCU_CTRLDESCL49_1_OFFSET        0x00000E40
#define DCU_CTRLDESCL49_2_OFFSET        0x00000E44
#define DCU_CTRLDESCL49_3_OFFSET        0x00000E48
#define DCU_CTRLDESCL49_4_OFFSET        0x00000E4C
#define DCU_CTRLDESCL49_5_OFFSET        0x00000E50
#define DCU_CTRLDESCL49_6_OFFSET        0x00000E54
#define DCU_CTRLDESCL49_7_OFFSET        0x00000E58
#define DCU_CTRLDESCL49_8_OFFSET        0x00000E5C
#define DCU_CTRLDESCL49_9_OFFSET        0x00000E60
#define DCU_CTRLDESCL50_1_OFFSET        0x00000E80
#define DCU_CTRLDESCL50_2_OFFSET        0x00000E84
#define DCU_CTRLDESCL50_3_OFFSET        0x00000E88
#define DCU_CTRLDESCL50_4_OFFSET        0x00000E8C
#define DCU_CTRLDESCL50_5_OFFSET        0x00000E90
#define DCU_CTRLDESCL50_6_OFFSET        0x00000E94
#define DCU_CTRLDESCL50_7_OFFSET        0x00000E98
#define DCU_CTRLDESCL50_8_OFFSET        0x00000E9C
#define DCU_CTRLDESCL50_9_OFFSET        0x00000EA0
#define DCU_CTRLDESCL51_1_OFFSET        0x00000EC0
#define DCU_CTRLDESCL51_2_OFFSET        0x00000EC4
#define DCU_CTRLDESCL51_3_OFFSET        0x00000EC8
#define DCU_CTRLDESCL51_4_OFFSET        0x00000ECC
#define DCU_CTRLDESCL51_5_OFFSET        0x00000ED0
#define DCU_CTRLDESCL51_6_OFFSET        0x00000ED4
#define DCU_CTRLDESCL51_7_OFFSET        0x00000ED8
#define DCU_CTRLDESCL51_8_OFFSET        0x00000EDC
#define DCU_CTRLDESCL51_9_OFFSET        0x00000EE0
#define DCU_CTRLDESCL52_1_OFFSET        0x00000F00
#define DCU_CTRLDESCL52_2_OFFSET        0x00000F04
#define DCU_CTRLDESCL52_3_OFFSET        0x00000F08
#define DCU_CTRLDESCL52_4_OFFSET        0x00000F0C
#define DCU_CTRLDESCL52_5_OFFSET        0x00000F10
#define DCU_CTRLDESCL52_6_OFFSET        0x00000F14
#define DCU_CTRLDESCL52_7_OFFSET        0x00000F18
#define DCU_CTRLDESCL52_8_OFFSET        0x00000F1C
#define DCU_CTRLDESCL52_9_OFFSET        0x00000F20
#define DCU_CTRLDESCL53_1_OFFSET        0x00000F40
#define DCU_CTRLDESCL53_2_OFFSET        0x00000F44
#define DCU_CTRLDESCL53_3_OFFSET        0x00000F48
#define DCU_CTRLDESCL53_4_OFFSET        0x00000F4C
#define DCU_CTRLDESCL53_5_OFFSET        0x00000F50
#define DCU_CTRLDESCL53_6_OFFSET        0x00000F54
#define DCU_CTRLDESCL53_7_OFFSET        0x00000F58
#define DCU_CTRLDESCL53_8_OFFSET        0x00000F5C
#define DCU_CTRLDESCL53_9_OFFSET        0x00000F60
#define DCU_CTRLDESCL54_1_OFFSET        0x00000F80
#define DCU_CTRLDESCL54_2_OFFSET        0x00000F84
#define DCU_CTRLDESCL54_3_OFFSET        0x00000F88
#define DCU_CTRLDESCL54_4_OFFSET        0x00000F8C
#define DCU_CTRLDESCL54_5_OFFSET        0x00000F90
#define DCU_CTRLDESCL54_6_OFFSET        0x00000F94
#define DCU_CTRLDESCL54_7_OFFSET        0x00000F98
#define DCU_CTRLDESCL54_8_OFFSET        0x00000F9C
#define DCU_CTRLDESCL54_9_OFFSET        0x00000FA0
#define DCU_CTRLDESCL55_1_OFFSET        0x00000FC0
#define DCU_CTRLDESCL55_2_OFFSET        0x00000FC4
#define DCU_CTRLDESCL55_3_OFFSET        0x00000FC8
#define DCU_CTRLDESCL55_4_OFFSET        0x00000FCC
#define DCU_CTRLDESCL55_5_OFFSET        0x00000FD0
#define DCU_CTRLDESCL55_6_OFFSET        0x00000FD4
#define DCU_CTRLDESCL55_7_OFFSET        0x00000FD8
#define DCU_CTRLDESCL55_8_OFFSET        0x00000FDC
#define DCU_CTRLDESCL55_9_OFFSET        0x00000FE0
#define DCU_CTRLDESCL56_1_OFFSET        0x00001000
#define DCU_CTRLDESCL56_2_OFFSET        0x00001004
#define DCU_CTRLDESCL56_3_OFFSET        0x00001008
#define DCU_CTRLDESCL56_4_OFFSET        0x0000100C
#define DCU_CTRLDESCL56_5_OFFSET        0x00001010
#define DCU_CTRLDESCL56_6_OFFSET        0x00001014
#define DCU_CTRLDESCL56_7_OFFSET        0x00001018
#define DCU_CTRLDESCL56_8_OFFSET        0x0000101C
#define DCU_CTRLDESCL56_9_OFFSET        0x00001020
#define DCU_CTRLDESCL57_1_OFFSET        0x00001040
#define DCU_CTRLDESCL57_2_OFFSET        0x00001044
#define DCU_CTRLDESCL57_3_OFFSET        0x00001048
#define DCU_CTRLDESCL57_4_OFFSET        0x0000104C
#define DCU_CTRLDESCL57_5_OFFSET        0x00001050
#define DCU_CTRLDESCL57_6_OFFSET        0x00001054
#define DCU_CTRLDESCL57_7_OFFSET        0x00001058
#define DCU_CTRLDESCL57_8_OFFSET        0x0000105C
#define DCU_CTRLDESCL57_9_OFFSET        0x00001060
#define DCU_CTRLDESCL58_1_OFFSET        0x00001080
#define DCU_CTRLDESCL58_2_OFFSET        0x00001084
#define DCU_CTRLDESCL58_3_OFFSET        0x00001088
#define DCU_CTRLDESCL58_4_OFFSET        0x0000108C
#define DCU_CTRLDESCL58_5_OFFSET        0x00001090
#define DCU_CTRLDESCL58_6_OFFSET        0x00001094
#define DCU_CTRLDESCL58_7_OFFSET        0x00001098
#define DCU_CTRLDESCL58_8_OFFSET        0x0000109C
#define DCU_CTRLDESCL58_9_OFFSET        0x000010A0
#define DCU_CTRLDESCL59_1_OFFSET        0x000010C0
#define DCU_CTRLDESCL59_2_OFFSET        0x000010C4
#define DCU_CTRLDESCL59_3_OFFSET        0x000010C8
#define DCU_CTRLDESCL59_4_OFFSET        0x000010CC
#define DCU_CTRLDESCL59_5_OFFSET        0x000010D0
#define DCU_CTRLDESCL59_6_OFFSET        0x000010D4
#define DCU_CTRLDESCL59_7_OFFSET        0x000010D8
#define DCU_CTRLDESCL59_8_OFFSET        0x000010DC
#define DCU_CTRLDESCL59_9_OFFSET        0x000010E0
#define DCU_CTRLDESCL60_1_OFFSET        0x00001100
#define DCU_CTRLDESCL60_2_OFFSET        0x00001104
#define DCU_CTRLDESCL60_3_OFFSET        0x00001108
#define DCU_CTRLDESCL60_4_OFFSET        0x0000110C
#define DCU_CTRLDESCL60_5_OFFSET        0x00001110
#define DCU_CTRLDESCL60_6_OFFSET        0x00001114
#define DCU_CTRLDESCL60_7_OFFSET        0x00001118
#define DCU_CTRLDESCL60_8_OFFSET        0x0000111C
#define DCU_CTRLDESCL60_9_OFFSET        0x00001120
#define DCU_CTRLDESCL61_1_OFFSET        0x00001140
#define DCU_CTRLDESCL61_2_OFFSET        0x00001144
#define DCU_CTRLDESCL61_3_OFFSET        0x00001148
#define DCU_CTRLDESCL61_4_OFFSET        0x0000114C
#define DCU_CTRLDESCL61_5_OFFSET        0x00001150
#define DCU_CTRLDESCL61_6_OFFSET        0x00001154
#define DCU_CTRLDESCL61_7_OFFSET        0x00001158
#define DCU_CTRLDESCL61_8_OFFSET        0x0000115C
#define DCU_CTRLDESCL61_9_OFFSET        0x00001160
#define DCU_CTRLDESCL62_1_OFFSET        0x00001180
#define DCU_CTRLDESCL62_2_OFFSET        0x00001184
#define DCU_CTRLDESCL62_3_OFFSET        0x00001188
#define DCU_CTRLDESCL62_4_OFFSET        0x0000118C
#define DCU_CTRLDESCL62_5_OFFSET        0x00001190
#define DCU_CTRLDESCL62_6_OFFSET        0x00001194
#define DCU_CTRLDESCL62_7_OFFSET        0x00001198
#define DCU_CTRLDESCL62_8_OFFSET        0x0000119C
#define DCU_CTRLDESCL62_9_OFFSET        0x000011A0
#define DCU_CTRLDESCL63_1_OFFSET        0x000011C0
#define DCU_CTRLDESCL63_2_OFFSET        0x000011C4
#define DCU_CTRLDESCL63_3_OFFSET        0x000011C8
#define DCU_CTRLDESCL63_4_OFFSET        0x000011CC
#define DCU_CTRLDESCL63_5_OFFSET        0x000011D0
#define DCU_CTRLDESCL63_6_OFFSET        0x000011D4
#define DCU_CTRLDESCL63_7_OFFSET        0x000011D8
#define DCU_CTRLDESCL63_8_OFFSET        0x000011DC
#define DCU_CTRLDESCL63_9_OFFSET        0x000011E0
#endif /*(32 != DCU_LAYERS_NUM_MAX)*/

#if (1 == DCU_HUD_FUNCTIONALITY)
  #define DCU_HUD_WIDTH_OFFSET          0x00005000
  #define DCU_HUD_HEIGHT_OFFSET         0x00005004
  #define DCU_WARP_DESC_ADDR_OFFSET     0x00005008
  #define DCU_WARP_IRQ_CTRL_OFFSET      0x0000500C
  #define DCU_WARP_IRQ_STAT_OFFSET      0x00005010
  #define DCU_WARP_CTRL_OFFSET          0x00005014
  #define DCU_WARP_XOVR_STAT_OFFSET     0x00005018
  #define DCU_WARP_YOVR_STAT_OFFSET     0x0000501C
  #define DCU_WARP_DESC_TBSZ_OFFSET     0x00005020
#endif /* DCU_HUD_FUNCTIONALITY */



/**********          DCU_CTRLDESCCURSOR1 REGISTER         **********/
#define DCU_CTRLDESCCURSOR1_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_CTRLDESCCURSOR1_OFFSET)

/* Field definitions for CTRLDESCCURSOR1 */
#define DCU_CTRLDESCCURSOR1_HEIGHT_SHIFT        (16)
#define DCU_CTRLDESCCURSOR1_HEIGHT_MASK         ((0x000007FF) << (DCU_CTRLDESCCURSOR1_HEIGHT_SHIFT))

#define DCU_CTRLDESCCURSOR1_WIDTH_SHIFT         (0)
#define DCU_CTRLDESCCURSOR1_WIDTH_MASK          ((0x000007FF) << (DCU_CTRLDESCCURSOR1_WIDTH_SHIFT))


/**********          DCU_CTRLDESCCURSOR2 REGISTER         **********/
#define DCU_CTRLDESCCURSOR2_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_CTRLDESCCURSOR2_OFFSET)

/* Field definitions for CTRLDESCCURSOR2 */
#define DCU_CTRLDESCCURSOR2_POSY_SHIFT          (16)
#define DCU_CTRLDESCCURSOR2_POSY_MASK           ((0x000007FF) << (DCU_CTRLDESCCURSOR2_POSY_SHIFT))

#define DCU_CTRLDESCCURSOR2_POSX_SHIFT          (0)
#define DCU_CTRLDESCCURSOR2_POSX_MASK           ((0x000007FF) << (DCU_CTRLDESCCURSOR2_POSX_SHIFT))


/**********          DCU_CTRLDESCCURSOR3 REGISTER         **********/
#define DCU_CTRLDESCCURSOR3_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_CTRLDESCCURSOR3_OFFSET)

/* Field definitions for CTRLDESCCURSOR3 */
#define DCU_CTRLDESCCURSOR3_CUR_EN_SHIFT               (31)
#define DCU_CTRLDESCCURSOR3_CUR_EN_MASK                ((1) << (DCU_CTRLDESCCURSOR3_CUR_EN_SHIFT))

#define DCU_CTRLDESCCURSOR3_DEFAULT_CURSOR_COLOR_SHIFT (0)
#define DCU_CTRLDESCCURSOR3_DEFAULT_CURSOR_COLOR_MASK  ((0x00FFFFFF) << (DCU_CTRLDESCCURSOR3_DEFAULT_CURSOR_COLOR_SHIFT))


/**********          DCU_CTRLDESCCURSOR4 REGISTER         **********/
#define DCU_CTRLDESCCURSOR4_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_CTRLDESCCURSOR4_OFFSET)

/* Field definitions for CTRLDESCCURSOR4 */
#define DCU_CTRLDESCCURSOR4_HWC_BLINK_OFF_SHIFT        (16)
#define DCU_CTRLDESCCURSOR4_HWC_BLINK_OFF_MASK         ((0x000000FF) << (DCU_CTRLDESCCURSOR4_HWC_BLINK_OFF_SHIFT))

#define DCU_CTRLDESCCURSOR4_EN_BLINK_SHIFT             (8)
#define DCU_CTRLDESCCURSOR4_EN_BLINK_MASK              ((1) << (DCU_CTRLDESCCURSOR4_EN_BLINK_SHIFT))

#define DCU_CTRLDESCCURSOR4_HWC_BLINK_ON_SHIFT         (0)
#define DCU_CTRLDESCCURSOR4_HWC_BLINK_ON_MASK          ((0x000000FF) << (DCU_CTRLDESCCURSOR4_HWC_BLINK_ON_SHIFT))


/**********          DCU_MODE REGISTER         **********/
#define DCU_MODE_ADDR32(dcu_id)         (DCU_BASE_ADDRESS[(dcu_id)] + DCU_MODE_OFFSET)

/* Field definitions for DCU_MODE */
#define DCU_DCU_MODE_DCU_SW_RESET_SHIFT         (31)
#define DCU_DCU_MODE_DCU_SW_RESET_MASK          ((1) << (DCU_DCU_MODE_DCU_SW_RESET_SHIFT))

#define DCU_DCU_MODE_EN_DITHER_SHIFT            (30)
#define DCU_DCU_MODE_EN_DITHER_MASK             ((1) << (DCU_DCU_MODE_EN_DITHER_SHIFT))

#define DCU_DCU_MODE_ADDB_SHIFT                 (28)
#define DCU_DCU_MODE_ADDB_MASK                  ((0x00000003) << (DCU_DCU_MODE_ADDB_SHIFT))

#define DCU_DCU_MODE_ADDG_SHIFT                 (26)
#define DCU_DCU_MODE_ADDG_MASK                  ((0x00000003) << (DCU_DCU_MODE_ADDG_SHIFT))

#define DCU_DCU_MODE_ADDR_SHIFT                 (24)
#define DCU_DCU_MODE_ADDR_MASK                  ((0x00000003) << (DCU_DCU_MODE_ADDR_SHIFT))

#if (IPV_DCU_VYBRID == IPV_DCU)
  #define DCU_DCU_MODE_DDR_MODE_SHIFT           (23)
  #define DCU_DCU_MODE_DDR_MODE_MASK            ((1) << (DCU_DCU_MODE_DDR_MODE_SHIFT))
#endif

#define DCU_MODE_BLEND_ITER_SHIFT               (20)
#define DCU_MODE_BLEND_ITER_MASK                ((0x00000007) << (DCU_MODE_BLEND_ITER_SHIFT))

#if (1 == DCU_PDI_FUNCTIONALITY)
  #define DCU_MODE_PDI_SYNC_LOCK_SHIFT          (16)
  #define DCU_MODE_PDI_SYNC_LOCK_MASK           ((0x0000000F) << (DCU_MODE_PDI_SYNC_LOCK_SHIFT))

  #define DCU_MODE_PDI_INTERPOL_EN_SHIFT        (15)
  #define DCU_MODE_PDI_INTERPOL_EN_MASK         ((1) << (DCU_MODE_PDI_INTERPOL_EN_SHIFT))
#endif /*(DCU_PDI_FUNCTIONALITY)*/

#define DCU_MODE_RASTER_EN_SHIFT                (14)
#define DCU_MODE_RASTER_EN_MASK                 ((1) << (DCU_MODE_RASTER_EN_SHIFT))

#if (1 == DCU_PDI_FUNCTIONALITY)
  #define DCU_MODE_PDI_EN_SHIFT                 (13)
  #define DCU_MODE_PDI_EN_MASK                  ((1) << (DCU_MODE_PDI_EN_SHIFT))

  #define DCU_MODE_PDI_BYTE_REV_SHIFT           (12)
  #define DCU_MODE_PDI_BYTE_REV_MASK            ((1) << (DCU_MODE_PDI_BYTE_REV_SHIFT))

  #define DCU_MODE_PDI_DE_MODE_SHIFT            (11)
  #define DCU_MODE_PDI_DE_MODE_MASK             ((1) << (DCU_MODE_PDI_DE_MODE_SHIFT))

  #define DCU_MODE_PDI_NARROW_MODE_SHIFT        (10)
  #define DCU_MODE_PDI_NARROW_MODE_MASK         ((1) << (DCU_MODE_PDI_NARROW_MODE_SHIFT))

  #define DCU_MODE_PDI_MODE_SHIFT               (9)
  #define DCU_MODE_PDI_MODE_MASK                ((0x00000003) << (DCU_MODE_PDI_MODE_SHIFT))

  #define DCU_MODE_PDI_SLAVE_MODE_SHIFT         (7)
  #define DCU_MODE_PDI_SLAVE_MODE_MASK          ((1) << (DCU_MODE_PDI_SLAVE_MODE_SHIFT))
#endif /*(DCU_PDI_FUNCTIONALITY)*/

#define DCU_MODE_TAG_EN_SHIFT                   (6)
#define DCU_MODE_TAG_EN_MASK                    ((1) << (DCU_MODE_TAG_EN_SHIFT))

#define DCU_MODE_SIG_EN_SHIFT                   (5)
#define DCU_MODE_SIG_EN_MASK                    ((1) << (DCU_MODE_SIG_EN_SHIFT))

#define DCU_EN_GAMMA_SHIFT                      (2)
#define DCU_EN_GAMMA_MASK                       ((1) << (DCU_EN_GAMMA_SHIFT))

#define DCU_MODE_SHIFT                          (0)
#define DCU_MODE_MASK                           ((0x00000003) << (DCU_MODE_SHIFT))


/**********          DCU_BACKGROUND REGISTER         **********/
#define DCU_BGND_ADDR32(dcu_id) (DCU_BASE_ADDRESS[(dcu_id)] + DCU_BGND_OFFSET)

/* Field definitions for BGND */
#define DCU_BGND_BGND_R_SHIFT               (16)
#define DCU_BGND_BGND_R_MASK                ((0x000000FF) << (DCU_BGND_BGND_R_SHIFT))

#define DCU_BGND_BGND_G_SHIFT               (8)
#define DCU_BGND_BGND_G_MASK                ((0x000000FF) << (DCU_BGND_BGND_G_SHIFT))

#define DCU_BGND_BGND_B_SHIFT               (0)
#define DCU_BGND_BGND_B_MASK                ((0x000000FF) << (DCU_BGND_BGND_B_SHIFT))


/**********          DCU_DISPLAY_SIZE REGISTER         **********/
#define DCU_DISP_SIZE_ADDR32(dcu_id)  (DCU_BASE_ADDRESS[(dcu_id)] + DCU_DISP_SIZE_OFFSET)

/* Field definitions for DISP_SIZE */
#define DCU_DISP_SIZE_DELTA_Y_SHIFT              (16)
#define DCU_DISP_SIZE_DELTA_Y_MASK               ((0x000007FF) << (DCU_DISP_SIZE_DELTA_Y_SHIFT))

#define DCU_DISP_SIZE_DELTA_X_SHIFT              (0)
#define DCU_DISP_SIZE_DELTA_X_MASK               ((0x0000007F) << (DCU_DISP_SIZE_DELTA_X_SHIFT))


/**********          HSYN_PARA REGISTER         **********/
#define DCU_HSYN_PARA_ADDR32(dcu_id)  (DCU_BASE_ADDRESS[(dcu_id)] + DCU_HSYN_PARA_OFFSET)

/* Field definitions for HSYN_PARA */
#define DCU_HSYN_PARA_BP_H_SHIFT                 (22)
#define DCU_HSYN_PARA_BP_H_MASK                  ((0x000001FF) << (DCU_HSYN_PARA_BP_H_SHIFT))

#define DCU_HSYN_PARA_PW_H_SHIFT                 (11)
#define DCU_HSYN_PARA_PW_H_MASK                  ((0x000001FF) << (DCU_HSYN_PARA_PW_H_SHIFT))

#define DCU_HSYN_PARA_FP_H_SHIFT                 (0)
#define DCU_HSYN_PARA_FP_H_MASK                  ((0x000001FF) << (DCU_HSYN_PARA_FP_H_SHIFT))


/**********          VSYN_PARA REGISTER         **********/
#define DCU_VSYN_PARA_ADDR32(dcu_id)  (DCU_BASE_ADDRESS[(dcu_id)] + DCU_VSYN_PARA_OFFSET)

/* Field definitions for VSYN_PARA */
#define DCU_VSYN_PARA_BP_V_SHIFT                 (22)
#define DCU_VSYN_PARA_BP_V_MASK                  ((0x000001FF) << (DCU_VSYN_PARA_BP_V_SHIFT))

#define DCU_VSYN_PARA_PW_V_SHIFT                 (11)
#define DCU_VSYN_PARA_PW_V_MASK                  ((0x000001FF) << (DCU_VSYN_PARA_PW_V_SHIFT))

#define DCU_VSYN_PARA_FP_V_SHIFT                 (0)
#define DCU_VSYN_PARA_FP_V_MASK                  ((0x000001FF) << (DCU_VSYN_PARA_FP_V_SHIFT))


/**********          VSYN_PARA REGISTER         **********/
#define DCU_SYNPOL_ADDR32(dcu_id)       (DCU_BASE_ADDRESS[(dcu_id)] + DCU_SYNPOL_OFFSET)

/* Field definitions for SYNPOL */
#if (IPV_DCU_HALO == IPV_DCU)
  #define DCU_SYNPOL_INV_DE_SHIFT                (11)
  #define DCU_SYNPOL_INV_DE_MASK                 ((1) << (DCU_SYNPOL_INV_DE_SHIFT))
#endif /*(IPV_DCU_HALO == IPV_DCU)*/

#if (1 == DCU_PDI_FUNCTIONALITY)
  #define DCU_SYNPOL_INV_PDI_DE_SHIFT            (10)
  #define DCU_SYNPOL_INV_PDI_DE_MASK             ((1) << (DCU_SYNPOL_INV_PDI_DE_SHIFT))

  #define DCU_SYNPOL_INV_PDI_HS_SHIFT            (9)
  #define DCU_SYNPOL_INV_PDI_HS_MASK             ((1) << (DCU_SYNPOL_INV_PDI_HS_SHIFT))

  #define DCU_SYNPOL_INV_PDI_VS_SHIFT            (8)
  #define DCU_SYNPOL_INV_PDI_VS_MASK             ((1) << (DCU_SYNPOL_INV_PDI_VS_SHIFT))

  #define DCU_SYNPOL_INV_PDI_CLK_SHIFT           (7)
  #define DCU_SYNPOL_INV_PDI_CLK_MASK            ((1) << (DCU_SYNPOL_INV_PDI_CLK_SHIFT))
#endif /*(DCU_PDI_FUNCTIONALITY)*/

#define DCU_SYNPOL_INV_PXCK_SHIFT                (6)
#define DCU_SYNPOL_INV_PXCK_MASK                 ((1) << (DCU_SYNPOL_INV_PXCK_SHIFT))

#define DCU_SYNPOL_NEG_SHIFT                     (5)
#define DCU_SYNPOL_NEG_MASK                      ((1) << (DCU_SYNPOL_NEG_SHIFT))

#define DCU_SYNPOL_INV_VS_SHIFT                  (1)
#define DCU_SYNPOL_INV_VS_MASK                   ((1) << (DCU_SYNPOL_INV_VS_SHIFT))

#define DCU_SYNPOL_INV_HS_SHIFT                  (0)
#define DCU_SYNPOL_INV_HS_MASK                   ((1) << (DCU_SYNPOL_INV_HS_SHIFT))


/************ INTERRUPT STATUS REGISTER ************/
#define DCU_THRESHOLD_ADDR32(dcu_id)  (DCU_BASE_ADDRESS[(dcu_id)] + DCU_THRESHOLD_OFFSET)

/* Field definitions for THRESHOLD */
#define DCU_THRESHOLD_LS_BF_VS_SHIFT             (16)
#define DCU_THRESHOLD_LS_BF_VS_MASK              ((0x000003FF) << (DCU_THRESHOLD_LS_BF_VS_SHIFT))

#define DCU_THRESHOLD_OUT_BUF_HIGH_SHIFT         (8)
#define DCU_THRESHOLD_OUT_BUF_HIGH_MASK          ((0x000000FF) << (DCU_THRESHOLD_OUT_BUF_HIGH_SHIFT))

#define DCU_THRESHOLD_OUT_BUF_LOW_SHIFT          (0)
#define DCU_THRESHOLD_OUT_BUF_LOW_MASK           ((0x000000FF) << (DCU_THRESHOLD_OUT_BUF_LOW_SHIFT))


/************ INTERRUPT STATUS REGISTER ************/
#define DCU_INT_STATUS_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_INT_STATUS_OFFSET)

/* Field definitions for INT_STATUS */
#if (6 == DCU_LAYERS_BLEND_MAX)
#define DCU_INT_STATUS_P6_EMPTY_SHIFT            (31)
#define DCU_INT_STATUS_P6_EMPTY_MASK             ((1) << (DCU_INT_STATUS_P6_EMPTY_SHIFT))

#define DCU_INT_STATUS_P5_EMPTY_SHIFT            (30)
#define DCU_INT_STATUS_P5_EMPTY_MASK             ((1) << (DCU_INT_STATUS_P5_EMPTY_SHIFT))
#endif

#if (4 <= DCU_LAYERS_BLEND_MAX)
#define DCU_INT_STATUS_P4_EMPTY_SHIFT            (29)
#define DCU_INT_STATUS_P4_EMPTY_MASK             ((1) << (DCU_INT_STATUS_P4_EMPTY_SHIFT))

#define DCU_INT_STATUS_P3_EMPTY_SHIFT            (28)
#define DCU_INT_STATUS_P3_EMPTY_MASK             ((1) << (DCU_INT_STATUS_P3_EMPTY_SHIFT))
#endif

#define DCU_INT_STATUS_P2_EMPTY_SHIFT            (27)
#define DCU_INT_STATUS_P2_EMPTY_MASK             ((1) << (DCU_INT_STATUS_P2_EMPTY_SHIFT))

#define DCU_INT_STATUS_P1_EMPTY_SHIFT            (26)
#define DCU_INT_STATUS_P1_EMPTY_MASK             ((1) << (DCU_INT_STATUS_P1_EMPTY_SHIFT))

#if (6 == DCU_LAYERS_BLEND_MAX)
#define DCU_INT_STATUS_P6_FIFO_HI_FLAG_SHIFT      (23)
#define DCU_INT_STATUS_P6_FIFO_HI_FLAG_MASK       ((1) << (DCU_INT_STATUS_P6_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_STATUS_P6_FIFO_LO_FLAG_SHIFT      (22)
#define DCU_INT_STATUS_P6_FIFO_LO_FLAG_MASK       ((1) << (DCU_INT_STATUS_P6_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_STATUS_P5_FIFO_HI_FLAG_SHIFT      (21)
#define DCU_INT_STATUS_P5_FIFO_HI_FLAG_MASK       ((1) << (DCU_INT_STATUS_P5_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_STATUS_P5_FIFO_LO_FLAG_SHIFT      (20)
#define DCU_INT_STATUS_P5_FIFO_LO_FLAG_MASK       ((1) << (DCU_INT_STATUS_P5_FIFO_LO_FLAG_SHIFT))
#endif

#if (4 <= DCU_LAYERS_BLEND_MAX)
#define DCU_INT_STATUS_P4_FIFO_HI_FLAG_SHIFT      (19)
#define DCU_INT_STATUS_P4_FIFO_HI_FLAG_MASK       ((1) << (DCU_INT_STATUS_P4_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_STATUS_P4_FIFO_LO_FLAG_SHIFT      (18)
#define DCU_INT_STATUS_P4_FIFO_LO_FLAG_MASK       ((1) << (DCU_INT_STATUS_P4_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_STATUS_P3_FIFO_HI_FLAG_SHIFT      (17)
#define DCU_INT_STATUS_P3_FIFO_HI_FLAG_MASK       ((1) << (DCU_INT_STATUS_P3_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_STATUS_P3_FIFO_LO_FLAG_SHIFT      (16)
#define DCU_INT_STATUS_P3_FIFO_LO_FLAG_MASK       ((1) << (DCU_INT_STATUS_P3_FIFO_LO_FLAG_SHIFT))
#endif

#define DCU_INT_STATUS_DMA_TRANS_FINISH_SHIFT     (14)
#define DCU_INT_STATUS_DMA_TRANS_FINISH_MASK      ((1) << (DCU_INT_STATUS_DMA_TRANS_FINISH_SHIFT))

#define DCU_INT_STATUS_LYR_TRANS_FINISH_SHIFT     (12)
#define DCU_INT_STATUS_LYR_TRANS_FINISH_MASK      ((1) << (DCU_INT_STATUS_LYR_TRANS_FINISH_SHIFT))

#define DCU_INT_STATUS_IPM_ERROR_SHIFT            (11)
#define DCU_INT_STATUS_IPM_ERROR_MASK             ((1) << (DCU_INT_STATUS_IPM_ERROR_SHIFT))

#define DCU_INT_STATUS_PROG_END_SHIFT             (10)
#define DCU_INT_STATUS_PROG_END_MASK              ((1) << (DCU_INT_STATUS_PROG_END_SHIFT))

#define DCU_INT_STATUS_P2_FIFO_HI_FLAG_SHIFT      (9)
#define DCU_INT_STATUS_P2_FIFO_HI_FLAG_MASK       ((1) << (DCU_INT_STATUS_P2_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_STATUS_P2_FIFO_LO_FLAG_SHIFT      (8)
#define DCU_INT_STATUS_P2_FIFO_LO_FLAG_MASK       ((1) << (DCU_INT_STATUS_P2_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_STATUS_P1_FIFO_HI_FLAG_SHIFT      (7)
#define DCU_INT_STATUS_P1_FIFO_HI_FLAG_MASK       ((1) << (DCU_INT_STATUS_P1_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_STATUS_P1_FIFO_LO_FLAG_SHIFT      (6)
#define DCU_INT_STATUS_P1_FIFO_LO_FLAG_MASK       ((1) << (DCU_INT_STATUS_P1_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_STATUS_CRC_OVERFLOW_SHIFT         (5)
#define DCU_INT_STATUS_CRC_OVERFLOW_MASK          ((1) << (DCU_INT_STATUS_CRC_OVERFLOW_SHIFT))

#define DCU_INT_STATUS_CRC_READY_SHIFT            (4)
#define DCU_INT_STATUS_CRC_READY_MASK             ((1) << (DCU_INT_STATUS_CRC_READY_SHIFT))

#define DCU_INT_STATUS_VS_BLANK_SHIFT             (3)
#define DCU_INT_STATUS_VS_BLANK_MASK              ((1) << (DCU_INT_STATUS_VS_BLANK_SHIFT))

#define DCU_INT_STATUS_LS_BF_VS_SHIFT             (2)
#define DCU_INT_STATUS_LS_BF_VS_MASK              ((1) << (DCU_INT_STATUS_LS_BF_VS_SHIFT))

#define DCU_INT_STATUS_UNDRUN_SHIFT               (1)
#define DCU_INT_STATUS_UNDRUN_MASK                ((1) << (DCU_INT_STATUS_UNDRUN_SHIFT))

#define DCU_INT_STATUS_VSYNC_SHIFT                (0)
#define DCU_INT_STATUS_VSYNC_MASK                 ((1) << (DCU_INT_STATUS_VSYNC_SHIFT))



/************ INTERRUPT MASK REGISTER ************/
#define DCU_INT_MASK_ADDR32(dcu_id)       (DCU_BASE_ADDRESS[(dcu_id)] + DCU_INT_MASK_OFFSET)

/* Field definitions for INT_MASK */
#if (6 == DCU_LAYERS_BLEND_MAX)
#define DCU_INT_MASK_M_P6_EMPTY_SHIFT      (31)
  #define DCU_INT_MASK_M_P6_EMPTY_MASK       ((1UL) << (DCU_INT_MASK_M_P6_EMPTY_SHIFT))

#define DCU_INT_MASK_M_P5_EMPTY_SHIFT      (30)
  #define DCU_INT_MASK_M_P5_EMPTY_MASK       ((1UL) << (DCU_INT_MASK_M_P5_EMPTY_SHIFT))
#endif

#if (4 <= DCU_LAYERS_BLEND_MAX)
#define DCU_INT_MASK_M_P4_EMPTY_SHIFT      (29)
  #define DCU_INT_MASK_M_P4_EMPTY_MASK       ((1UL) << (DCU_INT_MASK_M_P4_EMPTY_SHIFT))

#define DCU_INT_MASK_M_P3_EMPTY_SHIFT      (28)
  #define DCU_INT_MASK_M_P3_EMPTY_MASK       ((1UL) << (DCU_INT_MASK_M_P3_EMPTY_SHIFT))
#endif

#define DCU_INT_MASK_M_P2_EMPTY_SHIFT      (27)
#define DCU_INT_MASK_M_P2_EMPTY_MASK       ((1UL) << (DCU_INT_MASK_M_P2_EMPTY_SHIFT))

#define DCU_INT_MASK_M_P1_EMPTY_SHIFT      (26)
#define DCU_INT_MASK_M_P1_EMPTY_MASK       ((1UL) << (DCU_INT_MASK_M_P1_EMPTY_SHIFT))

#if (6 == DCU_LAYERS_BLEND_MAX)
#define DCU_INT_MASK_M_P6_FIFO_HI_FLAG_SHIFT      (23)
  #define DCU_INT_MASK_M_P6_FIFO_HI_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P6_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_MASK_M_P6_FIFO_LO_FLAG_SHIFT      (22)
  #define DCU_INT_MASK_M_P6_FIFO_LO_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P6_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_MASK_M_P5_FIFO_HI_FLAG_SHIFT      (21)
  #define DCU_INT_MASK_M_P5_FIFO_HI_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P5_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_MASK_M_P5_FIFO_LO_FLAG_SHIFT      (20)
  #define DCU_INT_MASK_M_P5_FIFO_LO_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P5_FIFO_LO_FLAG_SHIFT))
#endif

#if (4 <= DCU_LAYERS_BLEND_MAX)
#define DCU_INT_MASK_M_P4_FIFO_HI_FLAG_SHIFT    (19)
  #define DCU_INT_MASK_M_P4_FIFO_HI_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P4_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_MASK_M_P4_FIFO_LO_FLAG_SHIFT    (18)
  #define DCU_INT_MASK_M_P4_FIFO_LO_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P4_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_MASK_M_P3_FIFO_HI_FLAG_SHIFT    (17)
  #define DCU_INT_MASK_M_P3_FIFO_HI_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P3_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_MASK_M_P3_FIFO_LO_FLAG_SHIFT    (16)
  #define DCU_INT_MASK_M_P3_FIFO_LO_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P3_FIFO_LO_FLAG_SHIFT))
#endif

#define DCU_INT_DATA_TRANS_END_SHIFT            (14)
#define DCU_INT_DATA_TRANS_END_MASK             ((1UL) << (DCU_INT_DATA_TRANS_END_SHIFT))

#define DCU_INT_LYRCFG_TRANS_END_SHIFT          (12)
#define DCU_INT_LYRCFG_TRANS_END_MASK           ((1UL) << (DCU_INT_LYRCFG_TRANS_END_SHIFT))

#define DCU_INT_MASK_M_IPM_ERROR_SHIFT          (11)
#define DCU_INT_MASK_M_IPM_ERROR_MASK           ((1UL) << (DCU_INT_MASK_M_IPM_ERROR_SHIFT))

#define DCU_INT_PROG_END_SHIFT                  (10)
#define DCU_INT_PROG_END_MASK                   ((1UL) << (DCU_INT_PROG_END_SHIFT))

#define DCU_INT_MASK_M_P2_FIFO_HI_FLAG_SHIFT    (9)
#define DCU_INT_MASK_M_P2_FIFO_HI_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P2_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_MASK_M_P2_FIFO_LO_FLAG_SHIFT    (8)
#define DCU_INT_MASK_M_P2_FIFO_LO_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P2_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_MASK_M_P1_FIFO_HI_FLAG_SHIFT    (7)
#define DCU_INT_MASK_M_P1_FIFO_HI_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P1_FIFO_HI_FLAG_SHIFT))

#define DCU_INT_MASK_M_P1_FIFO_LO_FLAG_SHIFT    (6)
#define DCU_INT_MASK_M_P1_FIFO_LO_FLAG_MASK     ((1UL) << (DCU_INT_MASK_M_P1_FIFO_LO_FLAG_SHIFT))

#define DCU_INT_CRC_OVERFLOW_SHIFT              (5)
#define DCU_INT_CRC_OVERFLOW_MASK               ((1UL) << (DCU_INT_CRC_OVERFLOW_SHIFT))

#define DCU_INT_CRC_READY_SHIFT                 (4)
#define DCU_INT_CRC_READY_MASK                  ((1UL) << (DCU_INT_CRC_READY_SHIFT))

#define DCU_INT_VS_BLANK_SHIFT                  (3)
#define DCU_INT_VS_BLANK_MASK                   ((1UL) << (DCU_INT_VS_BLANK_SHIFT))

#define DCU_INT_LS_BF_VS_SHIFT                  (2)
#define DCU_INT_LS_BF_VS_MASK                   ((1UL) << (DCU_INT_LS_BF_VS_SHIFT))

#define DCU_INT_MASK_M_UNDRUN_SHIFT             (1)
#define DCU_INT_MASK_M_UNDRUN_MASK              ((1UL) << (DCU_INT_MASK_M_UNDRUN_SHIFT))

#define DCU_INT_VSYNC_SHIFT                     (0)
#define DCU_INT_VSYNC_MASK                      ((1UL) << (DCU_INT_VSYNC_SHIFT))

#define DCU_INT_TIMING_MASK     ((uint32_t)((DCU_INT_VS_BLANK_MASK) | (DCU_INT_LS_BF_VS_MASK) | (DCU_INT_VSYNC_MASK) | (DCU_INT_PROG_END_MASK) | (DCU_INT_DATA_TRANS_END_MASK) | (DCU_INT_LYRCFG_TRANS_END_MASK)))
#define DCU_INT_CRC_MASK        ((uint32_t)((DCU_INT_CRC_READY_MASK) | (DCU_INT_CRC_OVERFLOW_MASK))) 


/************ COLBAR n REGISTER ************/
#define DCU_COLBAR1_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_1_OFFSET)
#define DCU_COLBAR2_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_2_OFFSET)
#define DCU_COLBAR3_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_3_OFFSET)
#define DCU_COLBAR4_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_4_OFFSET)
#define DCU_COLBAR5_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_5_OFFSET)
#define DCU_COLBAR6_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_6_OFFSET)
#define DCU_COLBAR7_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_7_OFFSET)
#define DCU_COLBAR8_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_COLBAR_8_OFFSET)

/* Field definitions for COLBAR_n */
#define DCU_COLBAR_n_R_SHIFT           (16)
#define DCU_COLBAR_n_R_MASK            ((0x000000FF) << (DCU_COLBAR_n_R_SHIFT))

#define DCU_COLBAR_n_G_SHIFT           (8)
#define DCU_COLBAR_n_G_MASK            ((0x000000FF) << (DCU_COLBAR_n_G_SHIFT))

#define DCU_COLBAR_n_B_SHIFT           (0)
#define DCU_COLBAR_n_B_MASK            ((0x000000FF) << (DCU_COLBAR_n_B_SHIFT))


/************ DIVISION RATIO REGISTER ************/
#define DCU_DIV_RATIO_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_DIV_RATIO_OFFSET)

/* Field definitions for DIV_RATIO */
#if (IPV_DCU_TREERUNNER == IPV_DCU)
  #define DCU_DIV_DUALEDGE_EN_SHIFT     (31)
  #define DCU_DIV_DUALEDGE_EN_MASK      ((0x1) << (DCU_DIV_DUALEDGE_EN_SHIFT))
#endif

#define DCU_DIV_RATIO_SHIFT             (0)
#define DCU_DIV_RATIO_MASK              ((0x000000FF) << (DCU_DIV_RATIO_SHIFT))

#if (1 == DCU_SAFETY_FUNCTIONALITY)
/************ SIGNED AREA SIZE REGISTER ************/
#define DCU_SIGN_CALC1_ADDR32(dcu_id) (DCU_BASE_ADDRESS[(dcu_id)] + DCU_SIGN_CALC1_OFFSET)

/* Field definitions for SIGN_CALC_1 */
#define DCU_SIGN_CALC1_VER_SIZE_SHIFT            (16)
#define DCU_SIGN_CALC1_VER_SIZE_MASK             ((0x000007FF) << (DCU_SIGN_CALC1_VER_SIZE_SHIFT))

#define DCU_SIGN_CALC1_HOR_SIZE_SHIFT            (0)
#define DCU_SIGN_CALC1_HOR_SIZE_MASK             ((0x000007FF) << (DCU_SIGN_CALC1_HOR_SIZE_SHIFT))


/************ SIGNED AREA POSITION REGISTER ************/
#define DCU_SIGN_CALC2_ADDR32(dcu_id) (DCU_BASE_ADDRESS[(dcu_id)] + DCU_SIGN_CALC2_OFFSET)

/* Field definitions for SIGN_CALC_2 */
#define DCU_SIGN_CALC2_VERT_POS_SHIFT            (16)
#define DCU_SIGN_CALC2_VERT_POS_MASK             ((0x000007FF) << (DCU_SIGN_CALC2_VERT_POS_SHIFT))

#define DCU_SIGN_CALC2_HOR_POS_SHIFT             (0)
#define DCU_SIGN_CALC2_HOR_POS_MASK              ((0x000007FF) << (DCU_SIGN_CALC2_HOR_POS_SHIFT))


/************ CRC VAL REGISTERS ************/
#define DCU_CRC_VAL_ADDR32(dcu_id)    (DCU_BASE_ADDRESS[(dcu_id)] + DCU_CRC_VAL_OFFSET)
#endif
  
#if (1 == DCU_PDI_FUNCTIONALITY)
/************ PDI_STATUS REGISTERS ************/
#define DCU_PDI_STATUS_ADDR32(dcu_id)	(DCU_BASE_ADDRESS[(dcu_id)] + DCU_PDI_STATUS_OFFSET)

/* Field definitions for PDI_STATUS */
#define DCU_PDI_STATUS_PDI_BLANKING_ERR_SHIFT     (9)
#define DCU_PDI_STATUS_PDI_BLANKING_ERR_MASK      ((1) << (DCU_PDI_STATUS_PDI_BLANKING_ERR_SHIFT))

#define DCU_PDI_STATUS_PDI_ECC_ERR2_SHIFT         (8)
#define DCU_PDI_STATUS_PDI_ECC_ERR2_MASK          ((1) << (DCU_PDI_STATUS_PDI_ECC_ERR2_SHIFT))

#define DCU_PDI_STATUS_PDI_ECC_ERR1_SHIFT         (7)
#define DCU_PDI_STATUS_PDI_ECC_ERR1_MASK          ((1) << (DCU_PDI_STATUS_PDI_ECC_ERR1_SHIFT))

#define DCU_PDI_STATUS_PDI_LOCK_LOST_SHIFT        (6)
#define DCU_PDI_STATUS_PDI_LOCK_LOST_MASK         ((1) << (DCU_PDI_STATUS_PDI_LOCK_LOST_SHIFT))

#define DCU_PDI_STATUS_PDI_LOCK_DET_SHIFT         (5)
#define DCU_PDI_STATUS_PDI_LOCK_DET_MASK          ((1) << (DCU_PDI_STATUS_PDI_LOCK_DET_SHIFT))

#define DCU_PDI_STATUS_PDI_VSYNC_DET_SHIFT        (4)
#define DCU_PDI_STATUS_PDI_VSYNC_DET_MASK         ((1) << (DCU_PDI_STATUS_PDI_VSYNC_DET_SHIFT))

#define DCU_PDI_STATUS_PDI_HSYNC_DET_SHIFT        (3)
#define DCU_PDI_STATUS_PDI_HSYNC_DET_MASK         ((1) << (DCU_PDI_STATUS_PDI_HSYNC_DET_SHIFT))

#define DCU_PDI_STATUS_PDI_DE_DET_SHIFT           (2)
#define DCU_PDI_STATUS_PDI_DE_DET_MASK            ((1) << (DCU_PDI_STATUS_PDI_DE_DET_SHIFT))

#define DCU_PDI_STATUS_PDI_CLK_LOST_SHIFT         (1)
#define DCU_PDI_STATUS_PDI_CLK_LOST_MASK          ((1) << (DCU_PDI_STATUS_PDI_CLK_LOST_SHIFT))

#define DCU_PDI_STATUS_PDI_CLK_DET_SHIFT          (0)
#define DCU_PDI_STATUS_PDI_CLK_DET_MASK           ((1) << (DCU_PDI_STATUS_PDI_CLK_DET_SHIFT))


/************ MASK_PDI_STATUS REGISTERS ************/
#define DCU_MASK_PDI_STATUS_ADDR32(dcu_id)	(DCU_BASE_ADDRESS[(dcu_id)] + DCU_MASK_PDI_STATUS_OFFSET)

/* Field definitions for Mask_PDI_STATUS */
#define DCU_MASK_PDI_STATUS_M_PDI_BLANKING_ERR_SHIFT   (9)
#define DCU_MASK_PDI_STATUS_M_PDI_BLANKING_ERR_MASK    ((1) << (DCU_MASK_PDI_STATUS_M_PDI_BLANKING_ERR_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_ECC_ERR2_SHIFT       (8)
#define DCU_MASK_PDI_STATUS_M_PDI_ECC_ERR2_MASK        ((1) << (DCU_MASK_PDI_STATUS_M_PDI_ECC_ERR2_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_ECC_ERR1_SHIFT       (7)
#define DCU_MASK_PDI_STATUS_M_PDI_ECC_ERR1_MASK        ((1) << (DCU_MASK_PDI_STATUS_M_PDI_ECC_ERR1_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_LOCK_LOST_SHIFT      (6)
#define DCU_MASK_PDI_STATUS_M_PDI_LOCK_LOST_MASK       ((1) << (DCU_MASK_PDI_STATUS_M_PDI_LOCK_LOST_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_LOCK_DET_SHIFT       (5)
#define DCU_MASK_PDI_STATUS_M_PDI_LOCK_DET_MASK        ((1) << (DCU_MASK_PDI_STATUS_M_PDI_LOCK_DET_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_VSYNC_DET_SHIFT      (4)
#define DCU_MASK_PDI_STATUS_M_PDI_VSYNC_DET_MASK       ((1) << (DCU_MASK_PDI_STATUS_M_PDI_VSYNC_DET_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_HSYNC_DET_SHIFT      (3)
#define DCU_MASK_PDI_STATUS_M_PDI_HSYNC_DET_MASK       ((1) << (DCU_MASK_PDI_STATUS_M_PDI_HSYNC_DET_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_DE_DET_SHIFT         (2)
#define DCU_MASK_PDI_STATUS_M_PDI_DE_DET_MASK          ((1) << (DCU_MASK_PDI_STATUS_M_PDI_DE_DET_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_CLK_LOST_SHIFT       (1)
#define DCU_MASK_PDI_STATUS_M_PDI_CLK_LOST_MASK        ((1) << (DCU_MASK_PDI_STATUS_M_PDI_CLK_LOST_SHIFT))

#define DCU_MASK_PDI_STATUS_M_PDI_CLK_DET_SHIFT        (0)
#define DCU_MASK_PDI_STATUS_M_PDI_CLK_DET_MASK         ((1) << (DCU_MASK_PDI_STATUS_M_PDI_CLK_DET_SHIFT))
#endif /*(DCU_PDI_FUNCTIONALITY)*/


/************ HW ERROR INTERRUPTS STATUS REGISTER1 ************/
#define DCU_HWERR_STATUS1_ADDR32(dcu_id)        (DCU_BASE_ADDRESS[(dcu_id)] + DCU_PARR_ERR_STATUS1_OFFSET)

/* Field definitions for PARR_ERR_STATUS1 */
#if (32 <= DCU_LAYERS_NUM_MAX)
#define DCU_PARR_ERR_STATUS1_L31_SHIFT         (31)
#define DCU_PARR_ERR_STATUS1_L31_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L31_SHIFT))

#define DCU_PARR_ERR_STATUS1_L30_SHIFT         (30)
#define DCU_PARR_ERR_STATUS1_L30_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L30_SHIFT))

#define DCU_PARR_ERR_STATUS1_L29_SHIFT         (29)
#define DCU_PARR_ERR_STATUS1_L29_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L29_SHIFT))

#define DCU_PARR_ERR_STATUS1_L28_SHIFT         (28)
#define DCU_PARR_ERR_STATUS1_L28_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L28_SHIFT))

#define DCU_PARR_ERR_STATUS1_L27_SHIFT         (27)
#define DCU_PARR_ERR_STATUS1_L27_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L27_SHIFT))

#define DCU_PARR_ERR_STATUS1_L26_SHIFT         (26)
#define DCU_PARR_ERR_STATUS1_L26_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L26_SHIFT))

#define DCU_PARR_ERR_STATUS1_L25_SHIFT         (25)
#define DCU_PARR_ERR_STATUS1_L25_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L25_SHIFT))

#define DCU_PARR_ERR_STATUS1_L24_SHIFT         (24)
#define DCU_PARR_ERR_STATUS1_L24_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L24_SHIFT))

#define DCU_PARR_ERR_STATUS1_L23_SHIFT         (23)
#define DCU_PARR_ERR_STATUS1_L23_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L23_SHIFT))

#define DCU_PARR_ERR_STATUS1_L22_SHIFT         (22)
#define DCU_PARR_ERR_STATUS1_L22_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L22_SHIFT))

#define DCU_PARR_ERR_STATUS1_L21_SHIFT         (21)
#define DCU_PARR_ERR_STATUS1_L21_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L21_SHIFT))

#define DCU_PARR_ERR_STATUS1_L20_SHIFT         (20)
#define DCU_PARR_ERR_STATUS1_L20_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L20_SHIFT))

#define DCU_PARR_ERR_STATUS1_L19_SHIFT         (19)
#define DCU_PARR_ERR_STATUS1_L19_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L19_SHIFT))

#define DCU_PARR_ERR_STATUS1_L18_SHIFT         (18)
#define DCU_PARR_ERR_STATUS1_L18_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L18_SHIFT))

#define DCU_PARR_ERR_STATUS1_L17_SHIFT         (17)
#define DCU_PARR_ERR_STATUS1_L17_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L17_SHIFT))

#define DCU_PARR_ERR_STATUS1_L16_SHIFT         (16)
#define DCU_PARR_ERR_STATUS1_L16_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L16_SHIFT))
#endif /*(32 <= DCU_LAYERS_NUM_MAX)*/
#if (16 <= DCU_LAYERS_NUM_MAX)
#define DCU_PARR_ERR_STATUS1_L15_SHIFT         (15)
#define DCU_PARR_ERR_STATUS1_L15_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L15_SHIFT))

#define DCU_PARR_ERR_STATUS1_L14_SHIFT         (14)
#define DCU_PARR_ERR_STATUS1_L14_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L14_SHIFT))

#define DCU_PARR_ERR_STATUS1_L13_SHIFT         (13)
#define DCU_PARR_ERR_STATUS1_L13_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L13_SHIFT))

#define DCU_PARR_ERR_STATUS1_L12_SHIFT         (12)
#define DCU_PARR_ERR_STATUS1_L12_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L12_SHIFT))

#define DCU_PARR_ERR_STATUS1_L11_SHIFT         (11)
#define DCU_PARR_ERR_STATUS1_L11_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L11_SHIFT))

#define DCU_PARR_ERR_STATUS1_L10_SHIFT         (10)
#define DCU_PARR_ERR_STATUS1_L10_MASK          ((1) << (DCU_PARR_ERR_STATUS1_L10_SHIFT))

#define DCU_PARR_ERR_STATUS1_L9_SHIFT          (9)
#define DCU_PARR_ERR_STATUS1_L9_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L9_SHIFT))

#define DCU_PARR_ERR_STATUS1_L8_SHIFT          (8)
#define DCU_PARR_ERR_STATUS1_L8_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L8_SHIFT))
#endif /*(16 <= DCU_LAYERS_NUM_MAX)*/
#define DCU_PARR_ERR_STATUS1_L7_SHIFT          (7)
#define DCU_PARR_ERR_STATUS1_L7_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L7_SHIFT))

#define DCU_PARR_ERR_STATUS1_L6_SHIFT          (6)
#define DCU_PARR_ERR_STATUS1_L6_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L6_SHIFT))

#define DCU_PARR_ERR_STATUS1_L5_SHIFT          (5)
#define DCU_PARR_ERR_STATUS1_L5_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L5_SHIFT))

#define DCU_PARR_ERR_STATUS1_L4_SHIFT          (4)
#define DCU_PARR_ERR_STATUS1_L4_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L4_SHIFT))

#define DCU_PARR_ERR_STATUS1_L3_SHIFT          (3)
#define DCU_PARR_ERR_STATUS1_L3_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L3_SHIFT))

#define DCU_PARR_ERR_STATUS1_L2_SHIFT          (2)
#define DCU_PARR_ERR_STATUS1_L2_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L2_SHIFT))

#define DCU_PARR_ERR_STATUS1_L1_SHIFT          (1)
#define DCU_PARR_ERR_STATUS1_L1_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L1_SHIFT))

#define DCU_PARR_ERR_STATUS1_L0_SHIFT          (0)
#define DCU_PARR_ERR_STATUS1_L0_MASK           ((1) << (DCU_PARR_ERR_STATUS1_L0_SHIFT))

#if (64 == DCU_LAYERS_NUM_MAX)
/************ HW ERROR INTERRUPTS STATUS REGISTER2 ************/
#define DCU_HWERR_STATUS2_ADDR32(dcu_id)        (DCU_BASE_ADDRESS[(dcu_id)] + DCU_PARR_ERR_STATUS2_OFFSET)

/* Field definitions for PARR_ERR_STATUS2 */
#define DCU_PARR_ERR_STATUS2_L63_SHIFT         (31)
#define DCU_PARR_ERR_STATUS2_L63_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L63_SHIFT))

#define DCU_PARR_ERR_STATUS2_L62_SHIFT         (30)
#define DCU_PARR_ERR_STATUS2_L62_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L62_SHIFT))

#define DCU_PARR_ERR_STATUS2_L61_SHIFT         (29)
#define DCU_PARR_ERR_STATUS2_L61_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L61_SHIFT))

#define DCU_PARR_ERR_STATUS2_L60_SHIFT         (28)
#define DCU_PARR_ERR_STATUS2_L60_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L60_SHIFT))

#define DCU_PARR_ERR_STATUS2_L59_SHIFT         (27)
#define DCU_PARR_ERR_STATUS2_L59_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L59_SHIFT))

#define DCU_PARR_ERR_STATUS2_L58_SHIFT         (26)
#define DCU_PARR_ERR_STATUS2_L58_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L58_SHIFT))

#define DCU_PARR_ERR_STATUS2_L57_SHIFT         (25)
#define DCU_PARR_ERR_STATUS2_L57_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L57_SHIFT))

#define DCU_PARR_ERR_STATUS2_L56_SHIFT         (24)
#define DCU_PARR_ERR_STATUS2_L56_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L56_SHIFT))

#define DCU_PARR_ERR_STATUS2_L55_SHIFT         (23)
#define DCU_PARR_ERR_STATUS2_L55_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L55_SHIFT))

#define DCU_PARR_ERR_STATUS2_L54_SHIFT         (22)
#define DCU_PARR_ERR_STATUS2_L54_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L54_SHIFT))

#define DCU_PARR_ERR_STATUS2_L53_SHIFT         (21)
#define DCU_PARR_ERR_STATUS2_L53_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L53_SHIFT))

#define DCU_PARR_ERR_STATUS2_L52_SHIFT         (20)
#define DCU_PARR_ERR_STATUS2_L52_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L52_SHIFT))

#define DCU_PARR_ERR_STATUS2_L51_SHIFT         (19)
#define DCU_PARR_ERR_STATUS2_L51_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L51_SHIFT))

#define DCU_PARR_ERR_STATUS2_L50_SHIFT         (18)
#define DCU_PARR_ERR_STATUS2_L50_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L50_SHIFT))

#define DCU_PARR_ERR_STATUS2_L49_SHIFT         (17)
#define DCU_PARR_ERR_STATUS2_L49_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L49_SHIFT))

#define DCU_PARR_ERR_STATUS2_L48_SHIFT         (16)
#define DCU_PARR_ERR_STATUS2_L48_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L48_SHIFT))

#define DCU_PARR_ERR_STATUS2_L47_SHIFT         (15)
#define DCU_PARR_ERR_STATUS2_L47_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L47_SHIFT))

#define DCU_PARR_ERR_STATUS2_L46_SHIFT         (14)
#define DCU_PARR_ERR_STATUS2_L46_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L46_SHIFT))

#define DCU_PARR_ERR_STATUS2_L45_SHIFT         (13)
#define DCU_PARR_ERR_STATUS2_L45_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L45_SHIFT))

#define DCU_PARR_ERR_STATUS2_L44_SHIFT         (12)
#define DCU_PARR_ERR_STATUS2_L44_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L44_SHIFT))

#define DCU_PARR_ERR_STATUS2_L43_SHIFT         (11)
#define DCU_PARR_ERR_STATUS2_L43_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L43_SHIFT))

#define DCU_PARR_ERR_STATUS2_L42_SHIFT         (10)
#define DCU_PARR_ERR_STATUS2_L42_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L42_SHIFT))

#define DCU_PARR_ERR_STATUS2_L41_SHIFT         (9)
#define DCU_PARR_ERR_STATUS2_L41_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L41_SHIFT))

#define DCU_PARR_ERR_STATUS2_L40_SHIFT         (8)
#define DCU_PARR_ERR_STATUS2_L40_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L40_SHIFT))

#define DCU_PARR_ERR_STATUS2_L39_SHIFT         (7)
#define DCU_PARR_ERR_STATUS2_L39_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L39_SHIFT))

#define DCU_PARR_ERR_STATUS2_L38_SHIFT         (6)
#define DCU_PARR_ERR_STATUS2_L38_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L38_SHIFT))

#define DCU_PARR_ERR_STATUS2_L37_SHIFT         (5)
#define DCU_PARR_ERR_STATUS2_L37_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L37_SHIFT))

#define DCU_PARR_ERR_STATUS2_L36_SHIFT         (4)
#define DCU_PARR_ERR_STATUS2_L36_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L36_SHIFT))

#define DCU_PARR_ERR_STATUS2_L35_SHIFT         (3)
#define DCU_PARR_ERR_STATUS2_L35_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L35_SHIFT))

#define DCU_PARR_ERR_STATUS2_L34_SHIFT         (2)
#define DCU_PARR_ERR_STATUS2_L34_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L34_SHIFT))

#define DCU_PARR_ERR_STATUS2_L33_SHIFT         (1)
#define DCU_PARR_ERR_STATUS2_L33_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L33_SHIFT))

#define DCU_PARR_ERR_STATUS2_L32_SHIFT         (0)
#define DCU_PARR_ERR_STATUS2_L32_MASK          ((1) << (DCU_PARR_ERR_STATUS2_L32_SHIFT))
#endif /*(64 == DCU_LAYERS_NUM_MAX)*/


/************ HW ERROR INTERRUPTS STATUS REGISTER3 ************/
#define DCU_HWERR_STATUS3_ADDR32(dcu_id)        (DCU_BASE_ADDRESS[(dcu_id)] + DCU_PARR_ERR_STATUS3_OFFSET)
#define DCU0_PARR_ERR_STATUS3                   (DCU0_BASE + DCU_PARR_ERR_STATUS3_OFFSET)
#define DCU1_PARR_ERR_STATUS3                   (DCU1_BASE + DCU_PARR_ERR_STATUS3_OFFSET)

/* Field definitions for PARR_ERR_STATUS3 */
#if (1 == DCU_RLE_FUNCTIONALITY)
#define DCU_PARR_ERR_STATUS3_RLE_ERR_SHIFT              (3)
#define DCU_PARR_ERR_STATUS3_RLE_ERR_MASK               ((1) << (DCU_PARR_ERR_STATUS3_RLE_ERR_SHIFT))
#endif /*(DCU_RLE_FUNCTIONALITY)*/

#define DCU_PARR_ERR_STATUS3_HWC_ERR_SHIFT              (2)
#define DCU_PARR_ERR_STATUS3_HWC_ERR_MASK               ((1) << (DCU_PARR_ERR_STATUS3_HWC_ERR_SHIFT))

#define DCU_PARR_ERR_STATUS3_SIG_ERR_SHIFT              (1)
#define DCU_PARR_ERR_STATUS3_SIG_ERR_MASK               ((1) << (DCU_PARR_ERR_STATUS3_SIG_ERR_SHIFT))

#define DCU_PARR_ERR_STATUS3_DISP_ERR_SHIFT             (0)
#define DCU_PARR_ERR_STATUS3_DISP_ERR_MASK              ((1) << (DCU_PARR_ERR_STATUS3_DISP_ERR_SHIFT))


/************ HW ERROR INTERRUPTS MASK REGISTER1 ************/
#define DCU_MASK_HWERR_INT1_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_MASK_PARR_ERR_STATUS1_OFFSET)

/* Field definitions for MASK_PARR_ERR_STATUS1 */
#if (32 <= DCU_LAYERS_NUM_MAX)
#define DCU_MASK_PARR_ERR_STATUS1_M_L31_SHIFT       (31)
#define DCU_MASK_PARR_ERR_STATUS1_M_L31_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L31_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L30_SHIFT       (30)
#define DCU_MASK_PARR_ERR_STATUS1_M_L30_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L30_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L29_SHIFT       (29)
#define DCU_MASK_PARR_ERR_STATUS1_M_L29_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L29_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L28_SHIFT       (28)
#define DCU_MASK_PARR_ERR_STATUS1_M_L28_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L28_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L27_SHIFT       (27)
#define DCU_MASK_PARR_ERR_STATUS1_M_L27_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L27_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L26_SHIFT       (26)
#define DCU_MASK_PARR_ERR_STATUS1_M_L26_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L26_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L25_SHIFT       (25)
#define DCU_MASK_PARR_ERR_STATUS1_M_L25_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L25_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L24_SHIFT       (24)
#define DCU_MASK_PARR_ERR_STATUS1_M_L24_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L24_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L23_SHIFT       (23)
#define DCU_MASK_PARR_ERR_STATUS1_M_L23_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L23_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L22_SHIFT       (22)
#define DCU_MASK_PARR_ERR_STATUS1_M_L22_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L22_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L21_SHIFT       (21)
#define DCU_MASK_PARR_ERR_STATUS1_M_L21_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L21_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L20_SHIFT       (20)
#define DCU_MASK_PARR_ERR_STATUS1_M_L20_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L20_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L19_SHIFT       (19)
#define DCU_MASK_PARR_ERR_STATUS1_M_L19_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L19_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L18_SHIFT       (18)
#define DCU_MASK_PARR_ERR_STATUS1_M_L18_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L18_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L17_SHIFT       (17)
#define DCU_MASK_PARR_ERR_STATUS1_M_L17_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L17_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L16_SHIFT       (16)
#define DCU_MASK_PARR_ERR_STATUS1_M_L16_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L16_SHIFT))
#endif /*(32 <= DCU_LAYERS_NUM_MAX)*/
#if (16 <= DCU_LAYERS_NUM_MAX)
#define DCU_MASK_PARR_ERR_STATUS1_M_L15_SHIFT       (15)
#define DCU_MASK_PARR_ERR_STATUS1_M_L15_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L15_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L14_SHIFT       (14)
#define DCU_MASK_PARR_ERR_STATUS1_M_L14_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L14_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L13_SHIFT       (13)
#define DCU_MASK_PARR_ERR_STATUS1_M_L13_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L13_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L12_SHIFT       (12)
#define DCU_MASK_PARR_ERR_STATUS1_M_L12_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L12_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L11_SHIFT       (11)
#define DCU_MASK_PARR_ERR_STATUS1_M_L11_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L11_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L10_SHIFT       (10)
#define DCU_MASK_PARR_ERR_STATUS1_M_L10_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L10_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L9_SHIFT        (9)
#define DCU_MASK_PARR_ERR_STATUS1_M_L9_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L9_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L8_SHIFT        (8)
#define DCU_MASK_PARR_ERR_STATUS1_M_L8_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L8_SHIFT))
#endif /*(16 <= DCU_LAYERS_NUM_MAX)*/
#define DCU_MASK_PARR_ERR_STATUS1_M_L7_SHIFT        (7)
#define DCU_MASK_PARR_ERR_STATUS1_M_L7_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L7_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L6_SHIFT        (6)
#define DCU_MASK_PARR_ERR_STATUS1_M_L6_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L6_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L5_SHIFT        (5)
#define DCU_MASK_PARR_ERR_STATUS1_M_L5_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L5_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L4_SHIFT        (4)
#define DCU_MASK_PARR_ERR_STATUS1_M_L4_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L4_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L3_SHIFT        (3)
#define DCU_MASK_PARR_ERR_STATUS1_M_L3_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L3_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L2_SHIFT        (2)
#define DCU_MASK_PARR_ERR_STATUS1_M_L2_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L2_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L1_SHIFT        (1)
#define DCU_MASK_PARR_ERR_STATUS1_M_L1_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L1_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS1_M_L0_SHIFT        (0)
#define DCU_MASK_PARR_ERR_STATUS1_M_L0_MASK         ((1) << (DCU_MASK_PARR_ERR_STATUS1_M_L0_SHIFT))

#if (64 == DCU_LAYERS_NUM_MAX)
/************ HW ERROR INTERRUPTS MASK REGISTER2 ************/
#define DCU_MASK_HWERR_INT2_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_MASK_PARR_ERR_STATUS2_OFFSET)

/* Field definitions for MASK_PARR_ERR_STATUS2 */
#define DCU_MASK_PARR_ERR_STATUS2_M_L63_SHIFT       (31)
#define DCU_MASK_PARR_ERR_STATUS2_M_L63_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L63_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L62_SHIFT       (30)
#define DCU_MASK_PARR_ERR_STATUS2_M_L62_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L62_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L61_SHIFT       (29)
#define DCU_MASK_PARR_ERR_STATUS2_M_L61_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L61_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L60_SHIFT       (28)
#define DCU_MASK_PARR_ERR_STATUS2_M_L60_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L60_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L59_SHIFT       (27)
#define DCU_MASK_PARR_ERR_STATUS2_M_L59_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L59_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L58_SHIFT       (26)
#define DCU_MASK_PARR_ERR_STATUS2_M_L58_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L58_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L57_SHIFT       (25)
#define DCU_MASK_PARR_ERR_STATUS2_M_L57_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L57_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L56_SHIFT       (24)
#define DCU_MASK_PARR_ERR_STATUS2_M_L56_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L56_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L55_SHIFT       (23)
#define DCU_MASK_PARR_ERR_STATUS2_M_L55_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L55_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L54_SHIFT       (22)
#define DCU_MASK_PARR_ERR_STATUS2_M_L54_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L54_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L53_SHIFT       (21)
#define DCU_MASK_PARR_ERR_STATUS2_M_L53_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L53_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L52_SHIFT       (20)
#define DCU_MASK_PARR_ERR_STATUS2_M_L52_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L52_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L51_SHIFT       (19)
#define DCU_MASK_PARR_ERR_STATUS2_M_L51_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L51_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L50_SHIFT       (18)
#define DCU_MASK_PARR_ERR_STATUS2_M_L50_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L50_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L49_SHIFT       (17)
#define DCU_MASK_PARR_ERR_STATUS2_M_L49_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L49_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L48_SHIFT       (16)
#define DCU_MASK_PARR_ERR_STATUS2_M_L48_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L48_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L47_SHIFT       (15)
#define DCU_MASK_PARR_ERR_STATUS2_M_L47_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L47_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L46_SHIFT       (14)
#define DCU_MASK_PARR_ERR_STATUS2_M_L46_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L46_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L45_SHIFT       (13)
#define DCU_MASK_PARR_ERR_STATUS2_M_L45_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L45_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L44_SHIFT       (12)
#define DCU_MASK_PARR_ERR_STATUS2_M_L44_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L44_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L43_SHIFT       (11)
#define DCU_MASK_PARR_ERR_STATUS2_M_L43_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L43_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L42_SHIFT       (10)
#define DCU_MASK_PARR_ERR_STATUS2_M_L42_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L42_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L41_SHIFT       (9)
#define DCU_MASK_PARR_ERR_STATUS2_M_L41_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L41_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L40_SHIFT       (8)
#define DCU_MASK_PARR_ERR_STATUS2_M_L40_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L40_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L39_SHIFT       (7)
#define DCU_MASK_PARR_ERR_STATUS2_M_L39_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L39_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L38_SHIFT       (6)
#define DCU_MASK_PARR_ERR_STATUS2_M_L38_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L38_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L37_SHIFT       (5)
#define DCU_MASK_PARR_ERR_STATUS2_M_L37_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L37_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L36_SHIFT       (4)
#define DCU_MASK_PARR_ERR_STATUS2_M_L36_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L36_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L35_SHIFT       (3)
#define DCU_MASK_PARR_ERR_STATUS2_M_L35_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L35_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L34_SHIFT       (2)
#define DCU_MASK_PARR_ERR_STATUS2_M_L34_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L34_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L33_SHIFT       (1)
#define DCU_MASK_PARR_ERR_STATUS2_M_L33_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L33_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS2_M_L32_SHIFT       (0)
#define DCU_MASK_PARR_ERR_STATUS2_M_L32_MASK        ((1) << (DCU_MASK_PARR_ERR_STATUS2_M_L32_SHIFT))
#endif /*(64 == DCU_LAYERS_NUM_MAX)*/


/************ HW ERROR INTERRUPTS MASK REGISTER3 ************/
#define DCU_MASK_HWERR_INT3_ADDR32(dcu_id)      (DCU_BASE_ADDRESS[(dcu_id)] + DCU_MASK_PARR_ERR_STATUS3_OFFSET)

/* Field definitions for MASK_PARR_ERR_STATUS3 */
#if (1 == DCU_RLE_FUNCTIONALITY)
#define DCU_MASK_PARR_ERR_STATUS3_M_RLE_ERR_SHIFT            (3)
#define DCU_MASK_PARR_ERR_STATUS3_M_RLE_ERR_MASK             ((1) << (DCU_MASK_PARR_ERR_STATUS3_M_RLE_ERR_SHIFT))
#endif /*(DCU_RLE_FUNCTIONALITY)*/

#define DCU_MASK_PARR_ERR_STATUS3_M_HWC_ERR_SHIFT            (2)
#define DCU_MASK_PARR_ERR_STATUS3_M_HWC_ERR_MASK             ((1) << (DCU_MASK_PARR_ERR_STATUS3_M_HWC_ERR_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS3_M_SIG_ERR_SHIFT            (1)
#define DCU_MASK_PARR_ERR_STATUS3_M_SIG_ERR_MASK             ((1) << (DCU_MASK_PARR_ERR_STATUS3_M_SIG_ERR_SHIFT))

#define DCU_MASK_PARR_ERR_STATUS3_M_DISP_ERR_SHIFT           (0)
#define DCU_MASK_PARR_ERR_STATUS3_M_DISP_ERR_MASK            ((1) << (DCU_MASK_PARR_ERR_STATUS3_M_DISP_ERR_SHIFT))


/* Field definitions for THRESHOLD_INP_BUF_1 */
#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P2_HI_SHIFT        (24)
#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P2_HI_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_1_INP_BUF_P2_HI_SHIFT))

#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P2_LO_SHIFT        (16)
#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P2_LO_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_1_INP_BUF_P2_LO_SHIFT))

#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P1_HI_SHIFT        (8)
#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P1_HI_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_1_INP_BUF_P1_HI_SHIFT))

#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P1_LO_SHIFT        (0)
#define DCU_THRESHOLD_INP_BUF_1_INP_BUF_P1_LO_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_1_INP_BUF_P1_LO_SHIFT))



/* Field definitions for THRESHOLD_INP_BUF_2 */
#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P4_HI_SHIFT        (24)
#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P4_HI_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_2_INP_BUF_P4_HI_SHIFT))

#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P4_LO_SHIFT        (16)
#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P4_LO_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_2_INP_BUF_P4_LO_SHIFT))

#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P3_HI_SHIFT        (8)
#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P3_HI_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_2_INP_BUF_P3_HI_SHIFT))

#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P3_LO_SHIFT        (0)
#define DCU_THRESHOLD_INP_BUF_2_INP_BUF_P3_LO_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_2_INP_BUF_P3_LO_SHIFT))



/* Field definitions for THRESHOLD_INP_BUF_3 */
#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P6_HI_SHIFT        (24)
#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P6_HI_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_3_INP_BUF_P6_HI_SHIFT))

#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P6_LO_SHIFT        (16)
#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P6_LO_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_3_INP_BUF_P6_LO_SHIFT))

#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P5_HI_SHIFT        (8)
#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P5_HI_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_3_INP_BUF_P5_HI_SHIFT))

#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P5_LO_SHIFT        (0)
#define DCU_THRESHOLD_INP_BUF_3_INP_BUF_P5_LO_MASK         ((0x0000007F) << (DCU_THRESHOLD_INP_BUF_3_INP_BUF_P5_LO_SHIFT))



/* Field definitions for LUMA_COMP */
#define DCU_LUMA_COMP_Y_RED_SHIFT                (22)
#define DCU_LUMA_COMP_Y_RED_MASK                 ((0x000003FF) << (DCU_LUMA_COMP_Y_RED_SHIFT))

#define DCU_LUMA_COMP_Y_GREEN_SHIFT              (11)
#define DCU_LUMA_COMP_Y_GREEN_MASK               ((0x000003FF) << (DCU_LUMA_COMP_Y_GREEN_SHIFT))

#define DCU_LUMA_COMP_Y_BLUE_SHIFT               (0)
#define DCU_LUMA_COMP_Y_BLUE_MASK                ((0x000003FF) << (DCU_LUMA_COMP_Y_BLUE_SHIFT))



/* Field definitions for CHROMA_RED */
#define DCU_CHROMA_RED_CR_RED_SHIFT               (16)
#define DCU_CHROMA_RED_CR_RED_MASK                ((0x000007FF) << (DCU_CHROMA_RED_CR_RED_SHIFT))

#define DCU_CHROMA_RED_CB_GREEN_SHIFT             (0)
#define DCU_CHROMA_RED_CB_GREEN_MASK              ((0x00000FFF) << (DCU_CHROMA_RED_CB_GREEN_SHIFT))



/* Field definitions for CHROMA_GREEN */
#define DCU_CHROMA_GREEN_CR_GREEN_SHIFT            (16)
#define DCU_CHROMA_GREEN_CR_GREEN_MASK             ((0x000007FF) << (DCU_CHROMA_GREEN_CR_GREEN_SHIFT))

#define DCU_CHROMA_GREEN_CB_GREEN_SHIFT            (0)
#define DCU_CHROMA_GREEN_CB_GREEN_MASK             ((0x00000FFF) << (DCU_CHROMA_GREEN_CB_GREEN_SHIFT))



/* Field definitions for CHROMA_BLUE */
#define DCU_CHROMA_BLUE_CR_BLUE_SHIFT              (16)
#define DCU_CHROMA_BLUE_CR_BLUE_MASK               ((0x000007FF) << (DCU_CHROMA_BLUE_CR_BLUE_SHIFT))

#define DCU_CHROMA_BLUE_CB_BLUE_SHIFT              (0)
#define DCU_CHROMA_BLUE_CB_BLUE_MASK               ((0x00000FFF) << (DCU_CHROMA_BLUE_CB_BLUE_SHIFT))


/************ CRC_POS REGISTER ************/
#if (1 == DCU_SAFETY_FUNCTIONALITY)
  #define DCU_CRC_POS_ADDR32(dcu)             (DCU_BASE_ADDRESS[(dcu)] + DCU_CRC_POS_OFFSET)
#endif /*(DCU_SAFETY_FUNCTIONALITY)*/

/* Field definitions for LYR_INTPOL_EN */
#define DCU_LYR_INTPOL_EN_EN_SHIFT                 (0)
#define DCU_LYR_INTPOL_EN_EN_MASK                  ((1) << (DCU_LYR_INTPOL_EN_EN_SHIFT))


/* Field definitions for LYR_LUMA_COMP */
#define DCU_LYR_LUMA_COMP_Y_RED_SHIFT              (22)
#define DCU_LYR_LUMA_COMP_Y_RED_MASK               ((0x000003FF) << (DCU_LYR_LUMA_COMP_Y_RED_SHIFT))

#define DCU_LYR_LUMA_COMP_Y_GREEN_SHIFT            (11)
#define DCU_LYR_LUMA_COMP_Y_GREEN_MASK             ((0x000003FF) << (DCU_LYR_LUMA_COMP_Y_GREEN_SHIFT))

#define DCU_LYR_LUMA_COMP_Y_BLUE_SHIFT             (0)
#define DCU_LYR_LUMA_COMP_Y_BLUE_MASK              ((0x000003FF) << (DCU_LYR_LUMA_COMP_Y_BLUE_SHIFT))


/* Field definitions for LYR_CHROMA_RED */
#define DCU_LYR_CHROMA_RED_CR_RED_SHIFT            (16)
#define DCU_LYR_CHROMA_RED_CR_RED_MASK             ((0x000007FF) << (DCU_LYR_CHROMA_RED_CR_RED_SHIFT))

#define DCU_LYR_CHROMA_RED_CB_RED_SHIFT            (0)
#define DCU_LYR_CHROMA_RED_CB_RED_MASK             ((0x00000FFF) << (DCU_LYR_CHROMA_RED_CB_RED_SHIFT))



/* Field definitions for LYR_CHROMA_GREEN */
#define DCU_LYR_CHROMA_GREEN_CR_GREEN_SHIFT        (16)
#define DCU_LYR_CHROMA_GREEN_CR_GREEN_MASK         ((0x000007FF) << (DCU_LYR_CHROMA_GREEN_CR_GREEN_SHIFT))

#define DCU_LYR_CHROMA_GREEN_CB_GREEN_SHIFT        (0)
#define DCU_LYR_CHROMA_GREEN_CB_GREEN_MASK         ((0x00000FFF) << (DCU_LYR_CHROMA_GREEN_CB_GREEN_SHIFT))


/* Field definitions for LYR_CHROMA_BLUE */
#define DCU_LYR_CHROMA_BLUE_CR_BLUE_SHIFT          (16)
#define DCU_LYR_CHROMA_BLUE_CR_BLUE_MASK           ((0x000007FF) << (DCU_LYR_CHROMA_BLUE_CR_BLUE_SHIFT))

#define DCU_LYR_CHROMA_BLUE_CB_BLUE_SHIFT          (0)
#define DCU_LYR_CHROMA_BLUE_CB_BLUE_MASK           ((0x00000FFF) << (DCU_LYR_CHROMA_BLUE_CB_BLUE_SHIFT))

#if (IPV_DCU_VYBRID == IPV_DCU)
/************ UPDATE MODE REGISTER ************/
#define DCU_COMP_IMSIZE_ADDR32(dcu)         (DCU_BASE_ADDRESS[(dcu)] + DCU_COMP_IMSIZE_OFFSET)

/* Field definitions for COMP_IMSIZE */
#define DCU_COMP_IMSIZE_SIZE_SHIFT                 (0)
#define DCU_COMP_IMSIZE_SIZE_MASK                  ((0x003FFFFF) << (DCU_COMP_IMSIZE_SIZE_SHIFT))

#endif /*(IPV_DCU_VYBRID == IPV_DCU)*/


/************ UPDATE MODE REGISTER ************/
#define DCU_UPDATE_MODE_ADDR32(dcu)         (DCU_BASE_ADDRESS[(dcu)] + DCU_UPDATE_MODE_OFFSET)

/* Field definitions for UPDATE_MODE */
#define DCU_UPDATE_MODE_MODE_SHIFT              (31)
#define DCU_UPDATE_MODE_MODE_MASK               ((1) << (DCU_UPDATE_MODE_MODE_SHIFT))

#define DCU_UPDATE_MODE_READREG_SHIFT           (30)
#define DCU_UPDATE_MODE_READREG_MASK            ((1) << (DCU_UPDATE_MODE_READREG_SHIFT))

#if (IPV_DCU_HALO == IPV_DCU)
  #define DCU_UPDATE_MODE_TRIGMODEND_SHIFT      (0)
  #define DCU_UPDATE_MODE_TRIGMODEND_MASK       (0x00FF) << (DCU_UPDATE_MODE_TRIGMODEND_SHIFT))
#endif /*(IPV_DCU_HALO == IPV_DCU)*/


/* Field definitions for UNDERRUN */
#define DCU_UNDERRUN_LINE_SHIFT                 (16)
#define DCU_UNDERRUN_LINE_MASK                  ((0x000007FF) << (DCU_UNDERRUN_LINE_SHIFT))

#define DCU_UNDERRUN_PIXEL_SHIFT                (0)
#define DCU_UNDERRUN_PIXEL_MASK                 ((0x000007FF) << (DCU_UNDERRUN_PIXEL_SHIFT))

#if (1 == DCU_WRITEBACK_FUNCTIONALITY)
/************ WRITEBACK ADDR REGISTER ************/
  #define DCU_WRITEBACK_ADDR_ADDR32(dcu)    (DCU_BASE_ADDRESS[(uint8_t)(dcu)] + DCU_WRITEBACK_ADDR_OFFSET)

  /* Field definitions for WRITEBACK ADDR */
  #define DCU_WRITEBACK_ADDR_SHIFT              (0)
  #define DCU_WRITEBACK_ADDR_MASK               ((0xFFFFFFF8) << (DCU_WRITEBACK_ADDR_SHIFT))


/************ WRITEBACK CTRL REGISTER ************/
  #define DCU_WRITEBACK_CTRL_ADDR32(dcu)    (DCU_BASE_ADDRESS[(uint8_t)(dcu)] + DCU_WRITEBACK_CTRL_OFFSET)

  /* Field definitions for WRITEBACK CTRL  */
  #define DCU_WRITEBACK_CTRL_WALPHA_SHIFT       (16)
  #define DCU_WRITEBACK_CTRL_WALPHA_MASK        ((0xFF) << (DCU_WRITEBACK_CTRL_WALPHA_SHIFT))

  #define DCU_WRITEBACK_CTRL_TYPE_SHIFT         (2)
  #define DCU_WRITEBACK_CTRL_TYPE_MASK          ((1UL) << (DCU_WRITEBACK_CTRL_TYPE_SHIFT))

  #define DCU_WRITEBACK_CTRL_MODE_SHIFT         (0)
  #define DCU_WRITEBACK_CTRL_MODE_MASK          ((3UL) << (DCU_WRITEBACK_CTRL_MODE_SHIFT))


/************ WRITEBACK STAT REGISTER ************/
  #define DCU_WRITEBACK_STAT_ADDR32(dcu)    (DCU_BASE_ADDRESS[(uint8_t)(dcu)] + DCU_WRITEBACK_STAT_OFFSET)

  /* Field definitions for WRITEBACK STAT */
  #define DCU_WRITEBACK_STAT_WUNDERRUN_SHIFT      (3)
  #define DCU_WRITEBACK_STAT_WUNDERRUN_MASK       ((1UL) << (DCU_WRITEBACK_STAT_WUNDERRUN_SHIFT))

  #define DCU_WRITEBACK_STAT_WOVERRUN_SHIFT       (2)
  #define DCU_WRITEBACK_STAT_WOVERRUN_MASK        ((1UL) << (DCU_WRITEBACK_STAT_WOVERRUN_SHIFT))

  #define DCU_WRITEBACK_STAT_WDONE_SHIFT          (1)
  #define DCU_WRITEBACK_STAT_WDONE_MASK           ((1UL) << (DCU_WRITEBACK_STAT_WDONE_SHIFT))

  #define DCU_WRITEBACK_STAT_WERR_SHIFT           (0)
  #define DCU_WRITEBACK_STAT_WERR_MASK            ((1UL) << DCU_WRITEBACK_STAT_WERR_SHIFT)

  #define DCU_WRITEBACK_STAT_ALLERR_MASK         DCU_WRITEBACK_STAT_WUNDERRUN_MASK | DCU_WRITEBACK_STAT_WOVERRUN_MASK | DCU_WRITEBACK_STAT_WERR_MASK
#endif /* DCU_WRITEBACK_FUNCTIONALITY */

#if (1 == DCU_SAFETY_FUNCTIONALITY)
/************ FRAME CRC CONTROL REGISTER ************/
  #define DCU_CRC_FRM_CTRL_ADDR32(dcu)    (DCU_BASE_ADDRESS[(uint8_t)(dcu)] + DCU_CRC_FRM_CTRL_OFFSET)

  /* Field definitions for FRAME CRC CONTROL */
  #define DCU_CRC_FRMCTRL_EN_SHIFT                (0)
  #define DCU_CRC_FRMCTRL_EN_MASK                 ((1UL) << DCU_CRC_FRMCTRL_EN_SHIFT)


/************ FRAME CRC CONTROL REGISTER ************/
  #define DCU_CRC_FRM_VAL_ADDR32(dcu)    (DCU_BASE_ADDRESS[(uint8_t)(dcu)] + DCU_CRC_FRM_VAL_OFFSET)
#endif /*(DCU_SAFETY_FUNCTIONALITY)*/


/* Field definitions for GPR */
#define DCU_GPR_HLB_SHIFT                         (31)
#define DCU_GPR_HLB_MASK                          ((1) << (DCU_GPR_HLB_SHIFT))


/************ LAYER0 SOFTLOCK REGISTER ************/
#define DCU_SLR_L0_ADDR32(dcu)              (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_L0_OFFSET)

/* Field definitions for SLR_L0 */
#if (IPV_DCU_VYBRID == IPV_DCU)
  #define DCU_SLR_L0_WREN_SHIFT                   (16)
  #define DCU_SLR_L0_WREN_MASK                    ((0x0000F0E0) << (DCU_SLR_L0_WREN_SHIFT))

  #define DCU_SLR_L0_SLOCK_SHIFT                  (16)
  #define DCU_SLR_L0_SLOCK_MASK                   ((0x00000F0E) << (DCU_SLR_L0_SLOCK_SHIFT))
#endif /*(IPV_DCU_VYBRID == IPV_DCU)*/
#if ((IPV_DCU_HALO == IPV_DCU) || (IPV_DCU_RAYLEIGH == IPV_DCU) || (IPV_DCU_TREERUNNER == IPV_DCU))
  #define DCU_SLR_L0_WREN_SHIFT                   (15)
  #define DCU_SLR_L0_WREN_MASK                    ((0x0001E1E1) << (DCU_SLR_L0_WREN_SHIFT))

  #define DCU_SLR_L0_SLOCK_SHIFT                  (11)
  #define DCU_SLR_L0_SLOCK_MASK                   ((0x0001E1E1) << (DCU_SLR_L0_SLOCK_SHIFT))
#endif /*(IPV_DCU_HALO == IPV_DCU)||(IPV_DCU_RAYLEIGH == IPV_DCU)*/


/************ LAYER1 SOFTLOCK REGISTER ************/
#define DCU_SLR_L1_ADDR32(dcu)              (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_L1_OFFSET)

/* Field definitions for SLR_L1 */
#if (IPV_DCU_VYBRID == IPV_DCU)
  #define DCU_SLR_L1_WREN_SHIFT                   (16)
  #define DCU_SLR_L1_WREN_MASK                    ((0x0000F0E0) << (DCU_SLR_L1_WREN_SHIFT))

  #define DCU_SLR_L1_SLOCK_SHIFT                  (16)
  #define DCU_SLR_L1_SLOCK_MASK                   ((0x00000F0E) << (DCU_SLR_L1_SLOCK_SHIFT))
#endif /*(IPV_DCU_VYBRID == IPV_DCU)*/
#if ((IPV_DCU_HALO == IPV_DCU) || (IPV_DCU_RAYLEIGH == IPV_DCU) || (IPV_DCU_TREERUNNER == IPV_DCU))
  #define DCU_SLR_L1_WREN_SHIFT                   (15)
  #define DCU_SLR_L1_WREN_MASK                    ((0x0001E1E1) << (DCU_SLR_L1_WREN_SHIFT))

  #define DCU_SLR_L1_SLOCK_SHIFT                  (11)
  #define DCU_SLR_L1_SLOCK_MASK                   ((0x0001E1E1) << (DCU_SLR_L1_SLOCK_SHIFT))
#endif /*(IPV_DCU_HALO == IPV_DCU)||(IPV_DCU_RAYLEIGH == IPV_DCU)*/


/********* DISPLAY SIZE SOFTLOCK REGISTER *********/
#define DCU_SLR_DISP_SIZE_ADDR32(dcu)       (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_DISP_SIZE_OFFSET)

/* Field definitions for SLR_DISP_SIZE */
#define DCU_SLR_DISP_SIZE_WREN_SHIFT            (31)
#define DCU_SLR_DISP_SIZE_WREN_DISP_MASK        ((1) << (DCU_SLR_DISP_SIZE_WREN_SHIFT))

#define DCU_SLR_DISP_SIZE_SLOCK_SHIFT           (27)
#define DCU_SLR_DISP_SIZE_SLOCK_MASK            ((1) << (DCU_SLR_DISP_SIZE_SLOCK_SHIFT))


/******** HVSYNC PARAMETERS SOFTLOCK REGISTER ********/
#define DCU_SLR_HVSYNC_PARA_ADDR32(dcu)     (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_HVSYNC_PARA_OFFSET)

/* Field definitions for SLR_HVSYNC_PARA */
#define DCU_SLR_HVSYNC_PARA_WREN_SHIFT          (30)
#define DCU_SLR_HVSYNC_PARA_WREN_MASK           ((3) << (DCU_SLR_HVSYNC_PARA_WREN_SHIFT))

#define DCU_SLR_HVSYNC_PARA_SLOCK_SHIFT         (26)
#define DCU_SLR_HVSYNC_PARA_SLOCK_MASK          ((3) << (DCU_SLR_HVSYNC_PARA_SLOCK_SHIFT))


/******** HVSYNC POLARITY SOFTLOCK REGISTER ********/
#define DCU_SLR_HVSYNC_POL_ADDR32(dcu)      (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_POL_OFFSET)

/* Field definitions for SLR_POL */
#define DCU_SLR_POL_WREN_SHIFT                  (31)
#define DCU_SLR_POL_WREN_MASK                   ((1) << (DCU_SLR_POL_WREN_SHIFT))

#define DCU_SLR_POL_SLOCK_SHIFT                 (27)
#define DCU_SLR_POL_SLOCK_MASK                  ((1) << (DCU_SLR_POL_SLOCK_SHIFT))


/****** LAYER0 TRANSPARENCY SOFTLOCK REGISTER ******/
#define DCU_SLR_L0_TRANSP_ADDR32(dcu)       (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_L0TRANSP_OFFSET)

/* Field definitions for SLR_L0TRANSP */
#define DCU_SLR_L0TRANSP_WREN_SHIFT             (30)
#define DCU_SLR_L0TRANSP_WREN_MASK              ((3) << (DCU_SLR_L0TRANSP_WREN_SHIFT))

#define DCU_SLR_L0TRANSP_SLOCK_SHIFT            (26)
#define DCU_SLR_L0TRANSP_SLOCK_MASK             ((3) << (DCU_SLR_L0TRANSP_SLOCK_SHIFT))


/****** LAYER1 TRANSPARENCY SOFTLOCK REGISTER ******/
#define DCU_SLR_L1_TRANSP_ADDR32(dcu)       (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_L1TRANSP_OFFSET)

/* Field definitions for SLR_L1TRANSP */
#define DCU_SLR_L1TRANSP_WREN_SHIFT             (30)
#define DCU_SLR_L1TRANSP_WREN_MASK              ((0x3) << (DCU_SLR_L1TRANSP_WREN_SHIFT))

#define DCU_SLR_L1TRANSP_SLOCK_SHIFT            (26)
#define DCU_SLR_L1TRANSP_SLOCK_MASK             ((0x3) << (DCU_SLR_L1TRANSP_SLOCK_SHIFT))


#if (1 == DCU_HUD_FUNCTIONALITY)
/************ HUD SOFTLOCK REGISTER ************/
  #define DCU_SLR_HUD_ADDR32(dcu)           (DCU_BASE_ADDRESS[(dcu)] + DCU_SLR_HUD_OFFSET)

  /* Field definitions for SLR_HUD */
 #if (IPV_DCU_TREERUNNER == IPV_DCU)
  #define DCU_SLR_HUD_WREN_SHIFT                (30)
  #define DCU_SLR_HUD_WREN_MASK                 ((0x3) << (DCU_SLR_HUD_WREN_SHIFT))

  #define DCU_SLR_HUD_SLOCK_SHIFT               (26)
  #define DCU_SLR_HUD_SLOCK_MASK                ((0x3) << (DCU_SLR_L1TRANSP_SLOCK_SHIFT))
 #endif /*(IPV_DCU_TREERUNNER == IPV_DCU)*/

 #if (IPV_DCU_HALO == IPV_DCU)
  #define DCU_SLR_HUD_SLOCK_SHIFT               (4)
  #define DCU_SLR_HUD_SLOCK_MASK                ((3) << (DCU_SLR_HUD_SHIFT))
 #endif /*(IPV_DCU_HALO == IPV_DCU)*/


/************ HUD WIDTH REGISTER ************/
  #define DCU_HUD_WIDTH_ADDR32(dcu)         (DCU_BASE_ADDRESS[(dcu)] + DCU_HUD_WIDTH_OFFSET)

/************ HUD HEIGTH REGISTER ************/
  #define DCU_HUD_HEIGHT_ADDR32(dcu)        (DCU_BASE_ADDRESS[(dcu)] + DCU_HUD_HEIGHT_OFFSET)

  /* Field definitions for HUD_WIDTH & HUD_HEIGHT */
  #define DCU_HUD_SIZE_MASK                   	(0x07FF)


/************ WARP DESCRIPTOR ADDRESS REGISTER ************/
  #define DCU_WARP_DESC_ADDR_ADDR32(dcu)   (DCU_BASE_ADDRESS[(dcu)] + DCU_WARP_DESC_ADDR_OFFSET)

  /* Field definitions for WARP_DESC_ADDR */
  #define DCU_WARP_DESC_ADDR_SHIFT              (0)
  #define DCU_WARP_DESC_ADDR_MASK               ((0x0FFFFFFFF) << (DCU_WARP_DESC_ADDR_SHIFT))


/************ WARP INTERRUPT CONTROL REGISTER ************/
  #define DCU_WARP_IRQ_CTRL_ADDR32(dcu)     (DCU_BASE_ADDRESS[(dcu)] + DCU_WARP_IRQ_CTRL_OFFSET)

  /* Field definitions for WARP_IRQ_CTRL */
  #define DCU_WARP_IRQCTRL_LDDONE_SHIFT         (15)
  #define DCU_WARP_IRQCTRL_LDDONE_MASK          ((1) << (DCU_WARP_IRQCTRL_LDDONE_SHIFT))

  #define DCU_WARP_IRQCTRL_YOVFLW_SHIFT         (2)
  #define DCU_WARP_IRQCTRL_YOVFLW_MASK          ((1) << (DCU_WARP_IRQCTRL_YOVFLW_SHIFT))

  #define DCU_WARP_IRQCTRL_XOVFLW_SHIFT         (1)
  #define DCU_WARP_IRQCTRL_XOVFLW_MASK          ((1) << (DCU_WARP_IRQCTRL_XOVFLW_SHIFT))

  
/************ WARP INTERRUPT STATUS REGISTER ************/
  #define DCU_WARP_IRQ_STAT_ADDR32(dcu)     (DCU_BASE_ADDRESS[(dcu)] + DCU_WARP_IRQ_STAT_OFFSET)

  /* Field definitions for WARP_IRQ_STAT */
  #define DCU_WARP_IRQSTAT_LDDONE_SHIFT         (15)
  #define DCU_WARP_IRQSTAT_LDDONE_MASK          ((1) << (DCU_WARP_IRQSTAT_LDDONE_SHIFT))

  #define DCU_WARP_IRQSTAT_YOVFLW_SHIFT         (2)
  #define DCU_WARP_IRQSTAT_YOVFLW_MASK          ((1) << (DCU_WARP_IRQSTAT_YOVFLW_SHIFT))

  #define DCU_WARP_IRQSTAT_XOVFLW_SHIFT         (1)
  #define DCU_WARP_IRQSTAT_XOVFLW_MASK          ((1) << (DCU_WARP_IRQSTAT_XOVFLW_SHIFT))


/************ WARP CONTROL REGISTER ************/
  #define DCU_WARP_CTRL_ADDR32(dcu)         (DCU_BASE_ADDRESS[(dcu)] + DCU_WARP_CTRL_OFFSET)

  /* Field definitions for WARP_CTRL */
  #define DCU_WARP_CTRL_LINESLB_SHIFT           (16)
  #define DCU_WARP_CTRL_LINESLB_MASK            ((0x3F) << (DCU_WARP_CTRL_LINESLB_SHIFT))

  #define DCU_WARP_CTRL_AXIXFRS_SHIFT           (1)
  #define DCU_WARP_CTRL_AXIXFRS_MASK            ((7) << (DCU_WARP_CTRL_AXIXFRS_SHIFT))

  #define DCU_WARP_CTRL_HUDEN_SHIFT             (0)
  #define DCU_WARP_CTRL_HUDEN_MASK              ((1) << (DCU_WARP_CTRL_HUDEN_SHIFT))


/****** WARP HORIZONTAL OVERFLOW REGISTER ******/
  #define DCU_WARP_XOVR_STAT_ADDR32(dcu)    (DCU_BASE_ADDRESS[(dcu)] + DCU_WARP_XOVR_STAT_OFFSET)

/****** WARP VERTICAL OVERFLOW REGISTER ******/
  #define DCU_WARP_YOVR_STAT_ADDR32(dcu)    (DCU_BASE_ADDRESS[(dcu)] + DCU_WARP_YOVR_STAT_OFFSET)

  /* Field definitions for WARP_XOVR and WARP_YOVR */
  #define DCU_WARP_OVR_PIXEL_SHIFT             (16)
  #define DCU_WARP_OVR_PIXEL_MASK              ((0x0FFF) << (DCU_WARP_OVR_PIXEL_SHIFT))

  #define DCU_WARP_OVR_LINE_SHIFT              (0)
  #define DCU_WARP_OVR_LINE_MASK               ((0x0FFF) << (DCU_WARP_OVR_LINE_SHIFT))



/****** WARP DESCRIPTOR TABLE SIZE REGISTER ******/
  #define DCU_WARP_DESC_TBSZ_ADDR32(dcu)    (DCU_BASE_ADDRESS[(dcu)] + DCU_WARP_DESC_TBSZ_OFFSET)

  /* Field definitions for WARP_YOVR */
  #define DCU_WARP_DESC_TBSZ_SHIFT              (0)
#if (IPV_DCU_TREERUNNER == IPV_DCU)
  #define DCU_WARP_DESC_TBSZ_MASK               ((0x0FFFFFFFF) << (DCU_WARP_DESC_TBSZ_SHIFT))
#endif /*(IPV_DCU_TREERUNNER == IPV_DCU)*/
#if (IPV_DCU_HALO == IPV_DCU)
  #define DCU_WARP_DESC_TBSZ_MASK               ((0x0FFFF) << (DCU_WARP_DESC_TBSZ_SHIFT))
#endif /*(IPV_DCU_HALO == IPV_DCU)*/
#endif /*(DCU_HUD_FUNCTIONALITY)*/


/***********************************************************************************/
/*                            LAYER REGISTERS BLOCK                                */
/***********************************************************************************/

#define DCU_LAYER_BLOCK_OFFSET              (uint32_t)0x00000200
#define DCU_LAYER_BASE_ADDR32(dcu, layer)   (DCU_BASE_ADDRESS[(dcu)] + DCU_LAYER_BLOCK_OFFSET + (uint32_t)((layer)*0x40UL))


/************ LAYER CONTROL DESCRIPTOR 1 ************/
#define DCU_CTRLDESCL1_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer))

/* Field definitions for CTRLDESCLn_1 */
#define DCU_CTRLDESCLn_1_HEIGHT_SHIFT       (16)
#define DCU_CTRLDESCLn_1_HEIGHT_MASK        ((0x000007FF) << (DCU_CTRLDESCLn_1_HEIGHT_SHIFT))

#define DCU_CTRLDESCLn_1_WIDTH_SHIFT        (0)
#define DCU_CTRLDESCLn_1_WIDTH_MASK         ((0x000007FF) << (DCU_CTRLDESCLn_1_WIDTH_SHIFT))


/************ LAYER CONTROL DESCRIPTOR 2 ************/
#define DCU_CTRLDESCL2_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x04UL)

/* Field definitions for CTRLDESCLn_2 */
#define DCU_CTRLDESCLn_2_POSY_SHIFT         (16)
#define DCU_CTRLDESCLn_2_POSY_MASK          ((0x00000FFF) << (DCU_CTRLDESCLn_2_POSY_SHIFT))

#define DCU_CTRLDESCLn_2_POSX_SHIFT         (0)
#define DCU_CTRLDESCLn_2_POSX_MASK          ((0x00000FFF) << (DCU_CTRLDESCLn_2_POSX_SHIFT))


/************ LAYER CONTROL DESCRIPTOR 3 ************/
#define DCU_CTRLDESCL3_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x08UL)


/************ LAYER CONTROL DESCRIPTOR 4 ************/
#define DCU_CTRLDESCL4_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x0CUL)

/* Field definitions for CTRLDESCLn_4 */
#define DCU_CTRLDESCLn_4_EN_SHIFT           (31)
#define DCU_CTRLDESCLn_4_EN_MASK            ((1) << (DCU_CTRLDESCLn_4_EN_SHIFT))

#if (TILE_VAR_SIZE == DCU_TILE_MODE)
#define DCU_CTRLDESCLn_4_TILE_EN_SHIFT      (30)
#define DCU_CTRLDESCLn_4_TILE_EN_MASK       ((1) << (DCU_CTRLDESCLn_4_TILE_EN_SHIFT))

#define DCU_CTRLDESCLn_4_DATA_SEL_SHIFT     (29)
#define DCU_CTRLDESCLn_4_DATA_SEL_MASK      ((1) << (DCU_CTRLDESCLn_4_DATA_SEL_SHIFT))
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */

#define DCU_CTRLDESCLn_4_SAFETY_EN_SHIFT    (28)
#define DCU_CTRLDESCLn_4_SAFETY_EN_MASK     ((1) << (DCU_CTRLDESCLn_4_SAFETY_EN_SHIFT))

#define DCU_CTRLDESCLn_4_TRANS_SHIFT        (20)
#define DCU_CTRLDESCLn_4_TRANS_MASK         ((0x000000FF) << (DCU_CTRLDESCLn_4_TRANS_SHIFT))

#define DCU_CTRLDESCLn_4_BPP_SHIFT          (16)
#define DCU_CTRLDESCLn_4_BPP_MASK           ((0x0000000F) << (DCU_CTRLDESCLn_4_BPP_SHIFT))

#if (1 == DCU_RLE_FUNCTIONALITY)
#define DCU_CTRLDESCLn_4_EN_RLE_SHIFT       (15)
#define DCU_CTRLDESCLn_4_EN_RLE_MASK        ((1) << (DCU_CTRLDESCLn_4_EN_RLE_SHIFT))

#define DCU_CTRLDESCLn_4_EN_GRLE_SHIFT      (3)
#define DCU_CTRLDESCLn_4_EN_GRLE_MASK       ((1) << (DCU_CTRLDESCLn_4_EN_GRLE_SHIFT))
#endif /*(1 == DCU_RLE_FUNCTIONALITY)*/

#define DCU_CTRLDESCLn_4_LUOFFS_SHIFT       (4)
#define DCU_CTRLDESCLn_4_LUOFFS_MASK        ((0x000007FF) << (DCU_CTRLDESCLn_4_LUOFFS_SHIFT))

#define DCU_CTRLDESCLn_4_BB_SHIFT           (2)
#define DCU_CTRLDESCLn_4_BB_MASK            ((1) << (DCU_CTRLDESCLn_4_BB_SHIFT))

#define DCU_CTRLDESCLn_4_AB_SHIFT           (0)
#define DCU_CTRLDESCLn_4_AB_MASK            ((0x00000003) << (DCU_CTRLDESCLn_4_AB_SHIFT))


/************ LAYER CONTROL DESCRIPTOR 5 ************/
#define DCU_CTRLDESCL5_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x10UL)

/* Field definitions for CTRLDESCLn_5 */
#define DCU_CTRLDESCLn_5_CKMAX_R_SHIFT      (16)
#define DCU_CTRLDESCLn_5_CKMAX_R_MASK       ((0x000000FF) << (DCU_CTRLDESCLn_5_CKMAX_R_SHIFT))

#define DCU_CTRLDESCLn_5_CKMAX_G_SHIFT      (8)
#define DCU_CTRLDESCLn_5_CKMAX_G_MASK       ((0x000000FF) << (DCU_CTRLDESCLn_5_CKMAX_G_SHIFT))

#define DCU_CTRLDESCLn_5_CKMAX_B_SHIFT      (0)
#define DCU_CTRLDESCLn_5_CKMAX_B_MASK       ((0x000000FF) << (DCU_CTRLDESCLn_5_CKMAX_B_SHIFT))


/************ LAYER CONTROL DESCRIPTOR 6 ************/
#define DCU_CTRLDESCL6_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x14UL)

/* Field definitions for CTRLDESCLn_6 */
#define DCU_CTRLDESCLn_6_CKMIN_R_SHIFT      (16)
#define DCU_CTRLDESCLn_6_CKMIN_R_MASK       ((0x000000FF) << (DCU_CTRLDESCLn_6_CKMIN_R_SHIFT))

#define DCU_CTRLDESCLn_6_CKMIN_G_SHIFT      (8)
#define DCU_CTRLDESCLn_6_CKMIN_G_MASK       ((0x000000FF) << (DCU_CTRLDESCLn_6_CKMIN_G_SHIFT))

#define DCU_CTRLDESCLn_6_CKMIN_B_SHIFT      (0)
#define DCU_CTRLDESCLn_6_CKMIN_B_MASK       ((0x000000FF) << (DCU_CTRLDESCLn_6_CKMIN_B_SHIFT))


#if (TILE_VAR_SIZE == DCU_TILE_MODE)
/************ LAYER CONTROL DESCRIPTOR 7 ************/
#define DCU_CTRLDESCL7_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x18UL)

/* Field definitions for CTRLDESCLn_7 */
#define DCU_CTRLDESCLn_7_TILE_VER_SIZE_SHIFT    (16)
#define DCU_CTRLDESCLn_7_TILE_VER_SIZE_MASK     ((0x000007FF) << (DCU_CTRLDESCLn_7_TILE_VER_SIZE_SHIFT))

#define DCU_CTRLDESCLn_7_TILE_HOR_SIZE_SHIFT    (0)
#define DCU_CTRLDESCLn_7_TILE_HOR_SIZE_MASK     ((0x0000007F) << (DCU_CTRLDESCLn_7_TILE_HOR_SIZE_SHIFT))
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */


/************ LAYER CONTROL DESCRIPTOR 8 ************/
#define DCU_CTRLDESCL8_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x1CUL)

/* Field definitions for CTRLDESCLn_8 */
#define DCU_CTRLDESCLn_8_COLOR_SHIFT            (0)
#define DCU_CTRLDESCLn_8_COLOR_MASK             ((0x00FFFFFF) << (DCU_CTRLDESCLn_8_COLOR_SHIFT))


/************ LAYER CONTROL DESCRIPTOR 9 ************/
#define DCU_CTRLDESCL9_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x20UL)

/* Field definitions for CTRLDESCLn_9 */
#define DCU_CTRLDESCLn_9_COLOR_SHIFT            (0)
#define DCU_CTRLDESCLn_9_COLOR_MASK             ((0x00FFFFFF) << (DCU_CTRLDESCLn_9_COLOR_SHIFT))


#if ((IPV_DCU_HALO == IPV_DCU)||(IPV_DCU_RAYLEIGH == IPV_DCU)||(IPV_DCU_TREERUNNER == IPV_DCU))
/************ LAYER CONTROL DESCRIPTOR 10 ************/
#define DCU_CTRLDESCL10_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x24UL)

/* Field definitions for CTRLDESCLn_10 */
#if (TILE_FIX_SIZE == DCU_TILE_MODE)
 #define DCU_CTRLDESCLn_10_GPUTILE_EN_SHIFT     (31)
 #define DCU_CTRLDESCLn_10_GPUTILE_EN_MASK      ((0x07FF) << (DCU_CTRLDESCLn_10_GPUTILE_EN_SHIFT))
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */

#define DCU_CTRLDESCLn_10_POST_SKIP_SHIFT       (16)
#define DCU_CTRLDESCLn_10_POST_SKIP_COLOR_MASK  ((0x07FF) << (DCU_CTRLDESCLn_10_POST_SKIP_SHIFT))

#define DCU_CTRLDESCLn_10_PRE_SKIP_SHIFT        (0)
#define DCU_CTRLDESCLn_10_PRE_SKIP_COLOR_MASK   ((0x07FF) << (DCU_CTRLDESCLn_10_PRE_SKIP_SHIFT))
#endif

#if (1 == DCU_RLE_FUNCTIONALITY)
/************ LAYER CONTROL DESCRIPTOR 11 ************/
#define DCU_CTRLDESCL11_ADDR32(dcu, layer)   (DCU_LAYER_BASE_ADDR32(dcu, layer) + 0x28UL)

/* Field definitions for CTRLDESCLn_11 */
#define DCU_CTRLDESCLn_11_COMP_IMSZ_SHIFT       (0)
#define DCU_CTRLDESCLn_11_COMP_IMSZ_MASK        ((0xFFFFFFFF) << (DCU_CTRLDESCLn_10_PRE_SKIP_SHIFT))
#endif /*(1 == DCU_RLE_FUNCTIONALITY)*/
/*** End of bit definitions ***/

#endif
