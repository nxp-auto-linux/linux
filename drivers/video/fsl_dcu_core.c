/**
*	 @file		dcu.c
*
*	 @brief	 The file is the 2D-ACE source file.
*	 @details It contains the code of the driver.
*/
/*==================================================================================================
*	 Copyright (c) 2014 Freescale Semiconductor, Inc
*	 All Rights Reserved.
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
Cristian Tomescu (B13031)     04/06/2014    ENGR00316549  Porting the 2DACE driver on HALO platform.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     04/07/2014    ENGR00321529  Adding the HUD support.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     28/07/2014    ENGR00324499  Implementing changes for frbuff interface.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     06/08/2014    ENGR00324522  Implementing the writeback support.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     20/08/2014    ENGR00327882  Porting the driver on the RAYLEIGH platform.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     15/09/2014    ENGR00330491  Small improvements.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     02/09/2014    ENGR00334168  Adding the DCU_Disable function.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     31/10/2014    ENGR00337995  Improvement of some functions.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     24/11/2014    ENGR00340864  Change in div factor calculation.API also.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     07/01/2015    ENGR00344709  Changes about the errors interrupts.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     23/02/2015    ENGR00351101  Integration of the driver in ASR OS. 
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     27/05/2015    ENGR00355821  Porting on Treerunner. 
==================================================================================================*/

#include "fsl_dcu_regs.h"
#include "fsl_dcu.h"
#include "fsl_dcu_regmacros.h"

/*****************************************************************************
* local types
*****************************************************************************/

/*****************************************************************************
* local functions prototype
*****************************************************************************/
#if (1 == DCU_IRQ_SUPPORT)
static void DCU_Timing_Isr(Dcu_Unit_t dcu_id);
#endif

/*****************************************************************************
* local variables
*****************************************************************************/
#if (1 == DCU_IRQ_SUPPORT)
static Dcu_Callback_t		gpCallbackVSYNC[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackVSBLANK[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackLSBFVS[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackDMATRANSFIN[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackLYRTRANSFIN[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackPROGEND[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackCRCREADY[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackCRCOFV[DCU_NUMBER];
#if (1 == DCU_HUD_FUNCTIONALITY)
static Dcu_Callback_t		gpCallbackWARPLDDONE[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackWARPXOVFLW[DCU_NUMBER];
static Dcu_Callback_t		gpCallbackWARPYOVFLW[DCU_NUMBER];
#endif /* DCU_HUD_FUNCTIONALITY */

static volatile Dcu_DisplayIStatus_t gDCU_DisplayIStatus[DCU_NUMBER];
#endif /* DCU_IRQ_SUPPORT */

static Dcu_Layer_t RLE_layer[DCU_NUMBER];

static uint16_t Display_SizeX[DCU_NUMBER];
static uint16_t Display_SizeY[DCU_NUMBER];

/*==================================================================================================
*                                       GLOBAL VARIABLES
==================================================================================================*/
#if ( DRV_LINUX_OS == DCU_DRV_VARIANT )
uint64_t* DCU_BASE_ADDR64;
#endif /* DCU_DRV_VARIANT == DRV_LINUX_OS */

/*==================================================================================================
*				 GLOBAL FUNCTIONS
==================================================================================================*/
/*================================================================================================*/
/**
* @brief	The function initializes the DCU driver module.
* @details This is the first function of the DCU driver to be called before accessing
*          anything else, it initializes all needed registers and data structures.
*
* @param[in]	dcu_id	 Selects the DCU to be accessed
*		 aFreq	The DCU clock frequency in MHz
*		 apcLCD	Pointer to LCD parameter struct
*
* @return	error ode
*
* @api
*/
Dcu_Err_t DCU_Init(Dcu_Unit_t dcu_id, uint32_t aDCUclk, const Dcu_LCD_Para_t* apcLCD, Dcu_LCD_Connection_t aDivCalcType)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint8_t div = 0;
	uint32_t div32 = 0;
	uint32_t rezpart = 0;

	REG_WRITE32(DCU_MODE_ADDR32(dcu_id),0U);

	/* Background colour = black */
	REG_WRITE32(DCU_BGND_ADDR32(dcu_id),0U);

  /* Configuration of the LCD interface */
  REG_RMW32(DCU_DISP_SIZE_ADDR32(dcu_id), DCU_DISP_SIZE_DELTA_X_MASK, (apcLCD->mDeltaX >> 4)<<DCU_DISP_SIZE_DELTA_X_SHIFT);
	Display_SizeX[dcu_id] = apcLCD->mDeltaX;

  REG_RMW32(DCU_DISP_SIZE_ADDR32(dcu_id), DCU_DISP_SIZE_DELTA_Y_MASK, (apcLCD->mDeltaY)<<DCU_DISP_SIZE_DELTA_Y_SHIFT);
	Display_SizeY[dcu_id] = apcLCD->mDeltaY;

  REG_RMW32(DCU_HSYN_PARA_ADDR32(dcu_id), DCU_HSYN_PARA_BP_H_MASK, (apcLCD->mHorzBP)<<DCU_HSYN_PARA_BP_H_SHIFT);
  REG_RMW32(DCU_HSYN_PARA_ADDR32(dcu_id), DCU_HSYN_PARA_PW_H_MASK, (apcLCD->mHorzPW)<<DCU_HSYN_PARA_PW_H_SHIFT);
  REG_RMW32(DCU_HSYN_PARA_ADDR32(dcu_id), DCU_HSYN_PARA_FP_H_MASK, (apcLCD->mHorzFP)<<DCU_HSYN_PARA_FP_H_SHIFT);

  REG_RMW32(DCU_VSYN_PARA_ADDR32(dcu_id), DCU_VSYN_PARA_BP_V_MASK, (apcLCD->mVertBP)<<DCU_VSYN_PARA_BP_V_SHIFT);
  REG_RMW32(DCU_VSYN_PARA_ADDR32(dcu_id), DCU_VSYN_PARA_PW_V_MASK, (apcLCD->mVertPW)<<DCU_VSYN_PARA_PW_V_SHIFT);
  REG_RMW32(DCU_VSYN_PARA_ADDR32(dcu_id), DCU_VSYN_PARA_FP_V_MASK, (apcLCD->mVertFP)<<DCU_VSYN_PARA_FP_V_SHIFT);

	REG_WRITE32(DCU_SYNPOL_ADDR32(dcu_id),apcLCD->mSyncPol);

	/* Calculation of the frequency division factor */
	if (DCU_FREQDIV_NORMAL == aDivCalcType)
	{
    rezpart = ((uint32_t)(apcLCD->mDeltaX)+(uint32_t)(apcLCD->mHorzBP)+(uint32_t)(apcLCD->mHorzPW)+(uint32_t)(apcLCD->mHorzFP))*((uint32_t)(apcLCD->mDeltaY)+(uint32_t)(apcLCD->mVertBP)+(uint32_t)(apcLCD->mVertPW)+(uint32_t)(apcLCD->mVertFP));
    div32 = (aDCUclk*1000000)/(((uint32_t)(apcLCD->mVertFq)) * rezpart) - 1;
	}

	if (DCU_FREQDIV_HDMI == aDivCalcType)
	{
    div32 = (((aDCUclk * 1000)/(uint32_t)(apcLCD->mVertFq)) - 1);
	}

	if (DCU_FREQDIV_LVDS == aDivCalcType)
	{
		div32 = (apcLCD->mDivFactor) - 1;
	}

  if( 0UL != (0xFFFFFF00 & div32))
	{
		err = DCU_ERR_RANGE;
  }
		div = (uint8_t)div32;

	if(div32 < 1)
	{
		div = 1;
	}
  REG_RMW32(DCU_DIV_RATIO_ADDR32(dcu_id), DCU_DIV_RATIO_MASK, div << DCU_DIV_RATIO_SHIFT);

	/* Init global variables for DCU */
#if (1 == DCU_IRQ_SUPPORT)
	gpCallbackVSYNC[dcu_id]			 = NULL_PTR;
	gpCallbackVSBLANK[dcu_id]		 = NULL_PTR;
	gpCallbackLSBFVS[dcu_id]			= NULL_PTR;
	gpCallbackDMATRANSFIN[dcu_id] = NULL_PTR;
	gpCallbackLYRTRANSFIN[dcu_id] = NULL_PTR;
	gpCallbackPROGEND[dcu_id]		 = NULL_PTR;
	gpCallbackCRCREADY[dcu_id]		= NULL_PTR;
	gpCallbackCRCOFV[dcu_id]			= NULL_PTR;
#if (1 == DCU_HUD_FUNCTIONALITY)
	if ((uint8_t)HUD_DCU == (uint8_t)dcu_id)
	{
		gpCallbackWARPLDDONE[HUD_DCU] = NULL_PTR;
		gpCallbackWARPXOVFLW[HUD_DCU] = NULL_PTR;
		gpCallbackWARPYOVFLW[HUD_DCU] = NULL_PTR;
	}
#endif /* DCU_HUD_FUNCTIONALITY */

	gDCU_DisplayIStatus[dcu_id]	= DCU_INACTIVE;
#endif /* DCU_IRQ_SUPPORT */

	RLE_layer[dcu_id] = NO_RLE_LAYER;

	/* Gamma adjust disable */
	REG_BIT_CLEAR32(DCU_MODE_ADDR32(dcu_id), DCU_EN_GAMMA_MASK);

	/* Set lines before Vertical sync value */
  REG_BIT_CLEAR32(DCU_THRESHOLD_ADDR32(dcu_id), DCU_THRESHOLD_LS_BF_VS_MASK);

	/* Standard mode */
  REG_RMW32(DCU_MODE_ADDR32(dcu_id), DCU_MODE_MASK, DCU_MODE_NORMAL << DCU_MODE_SHIFT);
	REG_BIT_SET32(DCU_MODE_ADDR32(dcu_id), DCU_MODE_RASTER_EN_MASK);

	/* Set max number of blending layer */
  REG_RMW32(DCU_MODE_ADDR32(dcu_id), DCU_MODE_BLEND_ITER_MASK, (DCU_LAYERS_BLEND_MAX) << DCU_MODE_BLEND_ITER_SHIFT);

  /* Disables cursor */
  REG_WRITE32(DCU_CTRLDESCCURSOR1_ADDR32(dcu_id),0UL);
  REG_WRITE32(DCU_CTRLDESCCURSOR2_ADDR32(dcu_id),0UL);
  REG_WRITE32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id),0UL);
  REG_BIT_CLEAR32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_HWC_BLINK_ON_MASK << DCU_CTRLDESCCURSOR4_HWC_BLINK_ON_SHIFT);
  REG_BIT_CLEAR32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_HWC_BLINK_OFF_MASK << DCU_CTRLDESCCURSOR4_HWC_BLINK_OFF_SHIFT);

	/* Disables all layers */
	for (div=0; div < DCU_LAYERS_NUM_MAX; div++)
	{
		REG_WRITE32(DCU_CTRLDESCL4_ADDR32(dcu_id, div), 0U);
		REG_WRITE32(DCU_CTRLDESCL5_ADDR32(dcu_id, div), 0U);
		REG_WRITE32(DCU_CTRLDESCL6_ADDR32(dcu_id, div), 0U);
#if ((IPV_DCU_HALO == IPV_DCU) || (IPV_DCU_RAYLEIGH == IPV_DCU) || (IPV_DCU_TREERUNNER == IPV_DCU))
		REG_WRITE32(DCU_CTRLDESCL10_ADDR32(dcu_id, div), 0U);
#endif
	}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
  DCU_EnableDisplayTimingIrq(dcu_id, DCU_INT_VS_BLANK_MASK | DCU_INT_LS_BF_VS_MASK | DCU_INT_VSYNC_MASK | DCU_INT_PROG_END_MASK | DCU_INT_DATA_TRANS_END_MASK | DCU_INT_LYRCFG_TRANS_END_MASK);
#endif /* DCU_IRQ_STATEMACHINE && DCU_IRQ_SUPPORT */

	/* Set the automatic update mode */
  REG_WRITE32(DCU_UPDATE_MODE_ADDR32(dcu_id), DCU_UPDATE_MODE_READREG_MASK);
  while ((REG_READ32(DCU_UPDATE_MODE_ADDR32(dcu_id)) & DCU_UPDATE_MODE_READREG_MASK)) {}
	REG_WRITE32(DCU_UPDATE_MODE_ADDR32(dcu_id), DCU_UPDATE_MODE_MODE_MASK);

	return(err);
} /* DCU_Init() */


void DCU_Disable(Dcu_Unit_t dcu_id)
{
  REG_RMW32(DCU_MODE_ADDR32(dcu_id), DCU_MODE_MASK, DCU_MODE_OFF << DCU_MODE_SHIFT);
}


void DCU_SwReset(Dcu_Unit_t dcu_id)
{
	REG_BIT_SET32(DCU_MODE_ADDR32(dcu_id), DCU_DCU_MODE_DCU_SW_RESET_MASK);
  REG_BIT_CLEAR32(DCU_MODE_ADDR32(dcu_id), DCU_DCU_MODE_DCU_SW_RESET_MASK);
}


#if (1 == DCU_IRQ_SUPPORT)
Dcu_Err_t DCU_GetDisplayIStatus(Dcu_Unit_t dcu_id, Dcu_DisplayIStatus_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
		*pValue = gDCU_DisplayIStatus[dcu_id];
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}
#endif /* DCU_IRQ_SUPPORT */

Dcu_Err_t DCU_GetHwErrStatus(Dcu_Unit_t dcu_id, Dcu_HwErrRet_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
		/* Get value of error status register 1 */
    pValue->HW_Error1 = REG_READ32(DCU_HWERR_STATUS1_ADDR32(dcu_id));
	REG_WRITE32(DCU_HWERR_STATUS1_ADDR32(dcu_id),pValue->HW_Error1);
#if (64 == DCU_LAYERS_NUM_MAX)
		/* Get value of error status register 2 */
    pValue->HW_Error2 = REG_READ32(DCU_HWERR_STATUS2_ADDR32(dcu_id));
	REG_WRITE32(DCU_HWERR_STATUS2_ADDR32(dcu_id),pValue->HW_Error2);
#endif
		/* Get value of error status register 3 */
    pValue->HW_Error3 = REG_READ32(DCU_HWERR_STATUS3_ADDR32(dcu_id));
	REG_WRITE32(DCU_HWERR_STATUS3_ADDR32(dcu_id),pValue->HW_Error3);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

/**
* @brief   This function is used to set the mainly parameters of a DCUs layer.
* @details The function configures a layer for its main functionality. The
*	configured parameters are the size, the position, the colour coding
*	schema, the foreground and the background colours and the image
*	buffer address.
*
* @param[in]	dcu_id		- selects the DCU to be accessed.
*		layer_id	- selects of the layer to be accessed.
*		 pLayer_Params	- pointer to the config data structure.
*
* @return	 error code	- DCU_ERR_NULL_PTR if the buffer pointer is NULL
*                               - DCU_ERR_RANGE if the size or the position are too large
*
* @api
*/
Dcu_Err_t DCU_SetLayerConfig(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_LayerParams_t* pLayer_Params)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t temp_reg = 0;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_STATEMACHINE && DCU_IRQ_SUPPORT */

  if (((Display_SizeY[dcu_id] >= pLayer_Params->LayerHeight) && (Display_SizeX[dcu_id] >= pLayer_Params->LayerWidth))
      && ((Display_SizeY[dcu_id] >= pLayer_Params->LayerPosY) && (Display_SizeX[dcu_id] >= pLayer_Params->LayerPosX)))
  {
    if(0 != pLayer_Params->LayerBuffAdd)
    {
      temp_reg  = ((pLayer_Params->LayerWidth) << DCU_CTRLDESCLn_1_WIDTH_SHIFT) & DCU_CTRLDESCLn_1_WIDTH_MASK;
      temp_reg |= ((pLayer_Params->LayerHeight)<< DCU_CTRLDESCLn_1_HEIGHT_SHIFT)& DCU_CTRLDESCLn_1_HEIGHT_MASK;
      REG_WRITE32(DCU_CTRLDESCL1_ADDR32(dcu_id, layer_id), temp_reg);

      temp_reg  = ((pLayer_Params->LayerPosX) << DCU_CTRLDESCLn_2_POSX_SHIFT) & DCU_CTRLDESCLn_2_POSX_MASK;
      temp_reg |= ((pLayer_Params->LayerPosY) << DCU_CTRLDESCLn_2_POSY_SHIFT) & DCU_CTRLDESCLn_2_POSY_MASK;
      REG_WRITE32(DCU_CTRLDESCL2_ADDR32(dcu_id, layer_id), temp_reg);

      REG_WRITE32(DCU_CTRLDESCL3_ADDR32(dcu_id, layer_id), pLayer_Params->LayerBuffAdd);

      temp_reg  = ( (pLayer_Params->LayerColCode)<<DCU_CTRLDESCLn_4_BPP_SHIFT );
      temp_reg |= ( (pLayer_Params->LayerChromaKey)<<DCU_CTRLDESCLn_4_BB_SHIFT );
      temp_reg |= ( (pLayer_Params->LayerAlphaBlend)<<DCU_CTRLDESCLn_4_AB_SHIFT);
      temp_reg |= (((pLayer_Params->LayerTransp)<<DCU_CTRLDESCLn_4_TRANS_SHIFT) & DCU_CTRLDESCLn_4_TRANS_MASK);
      REG_WRITE32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), temp_reg);

      REG_WRITE32(DCU_CTRLDESCL8_ADDR32(dcu_id, layer_id), ((pLayer_Params->LayerFGNDCol) & DCU_CTRLDESCLn_8_COLOR_MASK));
      REG_WRITE32(DCU_CTRLDESCL9_ADDR32(dcu_id, layer_id), ((pLayer_Params->LayerBGNDCol) & DCU_CTRLDESCLn_9_COLOR_MASK));

#if ((IPV_DCU_HALO == IPV_DCU) || (IPV_DCU_RAYLEIGH == IPV_DCU) || (IPV_DCU_TREERUNNER == IPV_DCU))
      temp_reg  = ( (pLayer_Params->LayerPreSkip)<<DCU_CTRLDESCLn_10_PRE_SKIP_SHIFT ) & DCU_CTRLDESCLn_10_PRE_SKIP_COLOR_MASK;
      temp_reg |= ( (pLayer_Params->LayerPostSkip)<<DCU_CTRLDESCLn_10_POST_SKIP_SHIFT ) & DCU_CTRLDESCLn_10_POST_SKIP_COLOR_MASK;
#if (TILE_FIX_SIZE == DCU_TILE_MODE)
      temp_reg |= ( (pLayer_Params->LayerTileEn)<<DCU_CTRLDESCLn_10_GPUTILE_EN_SHIFT ) & DCU_CTRLDESCLn_10_GPUTILE_EN_MASK;
#endif /* TILE_FIX_SIZE == DCU_TILE_MODE */
      REG_WRITE32(DCU_CTRLDESCL10_ADDR32(dcu_id, layer_id), temp_reg);
#endif
		}
		else
		{
			err = DCU_ERR_NULL_PTR;
		}
	}
	else
	{
		err = DCU_ERR_RANGE;
	}

	return(err);
}


/**
* @brief	 This function enables a DCUs layer.
* @details This function enables a DCUs layer for the blending engine.
*
* @param[in]	dcu_id		- selects the DCU to be accessed.
*		 layer_id	- selects of the layer to be accessed.
*
* @return		 none
*
* @api
*/
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#else
void DCU_LayerEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#endif
{
	uint32_t temp_reg = 0;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_STATEMACHINE && DCU_IRQ_SUPPORT */

	temp_reg = REG_READ32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id));
	temp_reg |= (uint32_t)DCU_CTRLDESCLn_4_EN_MASK;
	REG_WRITE32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), temp_reg);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_STATEMACHINE && DCU_IRQ_SUPPORT */
}


/**
* @brief	 This function disables a DCUs layer.
* @details This function disables a DCUs layer for the blending engine.
*
* @param[in]	dcu_id		- selects the DCU to be accessed.
*		layer_id	- selects of the layer to be accessed.
*
* @return		 none
*
* @api
*/
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#else
void DCU_LayerDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
	uint32_t temp_reg = 0;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END != gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	temp_reg = REG_READ32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id));
	temp_reg &= ~DCU_CTRLDESCLn_4_EN_MASK;
	REG_WRITE32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), temp_reg);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

}

Dcu_Err_t DCU_IsLayerEnabled(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Enable_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
    *pValue=(Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_EN_MASK)>>DCU_CTRLDESCLn_4_EN_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

Dcu_Err_t DCU_SetLayerSize(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Size_t* pSize)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t temp_reg = 0;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

  if ((Display_SizeY[dcu_id] >= pSize->mHeight) && (Display_SizeX[dcu_id] >= pSize->mWidth))
	{
      temp_reg  = ((pSize->mWidth) << DCU_CTRLDESCLn_1_WIDTH_SHIFT) & DCU_CTRLDESCLn_1_WIDTH_MASK;
      temp_reg |= ((pSize->mHeight)<< DCU_CTRLDESCLn_1_HEIGHT_SHIFT)& DCU_CTRLDESCLn_1_HEIGHT_MASK;
		REG_WRITE32(DCU_CTRLDESCL1_ADDR32(dcu_id, layer_id), temp_reg);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

Dcu_Err_t DCU_GetLayerSize(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Size_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
    pValue->mWidth=(uint16_t)(REG_BIT_GET32(DCU_CTRLDESCL1_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_1_WIDTH_MASK)>>DCU_CTRLDESCLn_1_WIDTH_SHIFT);
    pValue->mHeight=(uint16_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_1_HEIGHT_MASK)>>DCU_CTRLDESCLn_1_HEIGHT_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

Dcu_Err_t DCU_SetLayerPosition(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Position_t* pPos)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t temp_reg = 0;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

  if ((Display_SizeY[dcu_id] >= pPos->mY) && (Display_SizeX[dcu_id] >= pPos->mX))
	{
    temp_reg  = ((pPos->mX) << DCU_CTRLDESCLn_2_POSX_SHIFT) & DCU_CTRLDESCLn_2_POSX_MASK;
    temp_reg |= ((pPos->mY) << DCU_CTRLDESCLn_2_POSY_SHIFT)& DCU_CTRLDESCLn_2_POSY_MASK;
		REG_WRITE32(DCU_CTRLDESCL2_ADDR32(dcu_id, layer_id), temp_reg);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

Dcu_Err_t DCU_GetLayerPosition(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Position_t* pPos)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pPos)
	{
    pPos->mX=(uint16_t)(REG_BIT_GET32(DCU_CTRLDESCL2_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_2_POSX_MASK)>>DCU_CTRLDESCLn_2_POSY_SHIFT);
    pPos->mY=(uint16_t)(REG_BIT_GET32(DCU_CTRLDESCL2_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_2_POSY_MASK)>>DCU_CTRLDESCLn_2_POSY_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_SetLayerBuffAddr(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t addr)
#else
void DCU_SetLayerBuffAddr(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t addr)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	REG_WRITE32(DCU_CTRLDESCL3_ADDR32(dcu_id, layer_id), addr);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

Dcu_Err_t DCU_GetLayerBuffAddr(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
		*pValue = REG_READ32(DCU_CTRLDESCL3_ADDR32(dcu_id, layer_id));
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_SetLayerBPP(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_BPP_t col_format)
#else
void DCU_SetLayerBPP(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_BPP_t col_format)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

  REG_RMW32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_BPP_MASK, col_format<<DCU_CTRLDESCLn_4_BPP_SHIFT);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

Dcu_Err_t DCU_GetLayerBPP(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_BPP_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
    *pValue=(uint8_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_BPP_MASK)>>DCU_CTRLDESCLn_4_BPP_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_SetLayerAlphaMode(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id,Dcu_AlphaKey_t alpha_mode)
#else
void DCU_SetLayerAlphaMode(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id,Dcu_AlphaKey_t alpha_mode)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

  REG_RMW32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_AB_MASK, alpha_mode<<DCU_CTRLDESCLn_4_AB_SHIFT);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_SetLayerAlphaVal(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint8_t value)
#else
void DCU_SetLayerAlphaVal(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint8_t value)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

  REG_RMW32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_TRANS_MASK, value<<DCU_CTRLDESCLn_4_TRANS_SHIFT);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

Dcu_Err_t DCU_GetLayerAlphaMode(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_AlphaKey_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
    *pValue=(Dcu_AlphaKey_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_AB_MASK)>>DCU_CTRLDESCLn_4_AB_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

Dcu_Err_t DCU_GetLayerAlphaVal(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint8_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
    *pValue=(uint8_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_TRANS_MASK)>>DCU_CTRLDESCLn_4_TRANS_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerChromaEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#else
void DCU_LayerChromaEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

  REG_BIT_SET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_BB_MASK);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerChromaDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#else
void DCU_LayerChromaDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

  REG_BIT_CLEAR32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_BB_MASK);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

Dcu_Err_t DCU_IsLayerChromaEnabled(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Enable_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
    *pValue=(Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_BB_MASK)>>DCU_CTRLDESCLn_4_BB_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

Dcu_Err_t DCU_SetLayerLUTOffset(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint16_t aValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if(0x800 > aValue)
	{
    REG_RMW32(DCU_CTRLDESCL4_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_4_LUOFFS_MASK, aValue<<DCU_CTRLDESCLn_4_LUOFFS_SHIFT);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return (err);
}

#if (1 == DCU_RLE_FUNCTIONALITY)
Dcu_Err_t DCU_LayerRLEEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
{
	Dcu_Err_t err = DCU_ERR_OK;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if(RLE_layer[dcu_id] != NO_RLE_LAYER)
	{
    REG_BIT_SET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_EN_RLE_MASK);
		RLE_layer[dcu_id] = layer_id;
	}
	else
	{
		err = DCU_ERR_RLE;
	}
	return(err);
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerRLEDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#else
void DCU_LayerRLEDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if(RLE_layer[dcu_id] == layer_id)
	{
    REG_BIT_CLEAR32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_EN_RLE_MASK);
		RLE_layer[dcu_id] = NO_RLE_LAYER;
	}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

Dcu_Err_t DCU_IsLayerRLEEnabled(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Enable_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
    *pValue=(Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_EN_RLE_MASK)>>DCU_CTRLDESCLn_4_EN_RLE_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

#if (IPV_DCU_VYBRID == IPV_DCU)
Dcu_Err_t DCU_SetRLECompSize(Dcu_Unit_t dcu_id, uint32_t aValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if (0x00400000 > aValue)
	{
		/* Set value */
    REG_RMW32(DCU_COMP_IMSIZE_ADDR32(dcu_id), DCU_COMP_IMSIZE_SIZE_MASK, aValue<<DCU_COMP_IMSIZE_SIZE_SHIFT);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

Dcu_Err_t DCU_GetRLELayer(Dcu_Unit_t dcu_id, Dcu_Layer_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
		*pValue = RLE_layer[dcu_id];
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}
#endif
#endif /*(1 == DCU_RLE_FUNCTIONALITY)*/

Dcu_Err_t DCU_SetLayerChroma(Dcu_Unit_t dcu_id , Dcu_Layer_t layer_id, Dcu_Colour_t* pLyrChromaMax, Dcu_Colour_t* pLyrChromaMin)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t Chroma_M = 0;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if ((NULL_PTR != pLyrChromaMax) && (NULL_PTR != pLyrChromaMin))
	{
		/* the layers chroma maximum */
    Chroma_M  = (((uint8_t)pLyrChromaMax->Red_Value)  <<DCU_CTRLDESCLn_5_CKMAX_R_SHIFT) & DCU_CTRLDESCLn_5_CKMAX_R_MASK;
    Chroma_M |= (((uint8_t)pLyrChromaMax->Green_Value)<<DCU_CTRLDESCLn_5_CKMAX_G_SHIFT) & DCU_CTRLDESCLn_5_CKMAX_G_MASK;
    Chroma_M |= (((uint8_t)pLyrChromaMax->Blue_Value) <<DCU_CTRLDESCLn_5_CKMAX_B_SHIFT) & DCU_CTRLDESCLn_5_CKMAX_B_MASK;

		REG_WRITE32(DCU_CTRLDESCL5_ADDR32(dcu_id, layer_id), Chroma_M);

		/* the layers chroma minimum value */
    Chroma_M  = (((uint8_t)pLyrChromaMin->Red_Value)  <<DCU_CTRLDESCLn_6_CKMIN_R_SHIFT) & DCU_CTRLDESCLn_6_CKMIN_R_MASK;
    Chroma_M |= (((uint8_t)pLyrChromaMin->Green_Value)<<DCU_CTRLDESCLn_6_CKMIN_G_SHIFT) & DCU_CTRLDESCLn_6_CKMIN_G_MASK;
    Chroma_M |= (((uint8_t)pLyrChromaMin->Blue_Value) <<DCU_CTRLDESCLn_6_CKMIN_B_SHIFT) & DCU_CTRLDESCLn_6_CKMIN_B_MASK;

		REG_WRITE32(DCU_CTRLDESCL6_ADDR32(dcu_id, layer_id), Chroma_M);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_GetLayerChromaMax(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Colour_t* pLyrChromaMax)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pLyrChromaMax)
	{
    pLyrChromaMax->Red_Value = (REG_BIT_GET32(DCU_CTRLDESCL5_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_5_CKMAX_R_MASK))>>DCU_CTRLDESCLn_5_CKMAX_R_SHIFT;
    pLyrChromaMax->Green_Value=(REG_BIT_GET32(DCU_CTRLDESCL5_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_5_CKMAX_G_MASK))>>DCU_CTRLDESCLn_5_CKMAX_G_SHIFT;
    pLyrChromaMax->Blue_Value =(REG_BIT_GET32(DCU_CTRLDESCL5_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_5_CKMAX_B_MASK))>>DCU_CTRLDESCLn_5_CKMAX_B_SHIFT;
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

Dcu_Err_t DCU_GetLayerChromaMin(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Colour_t* pLyrChromaMin)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pLyrChromaMin)
	{
    pLyrChromaMin->Red_Value = (REG_BIT_GET32(DCU_CTRLDESCL6_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_6_CKMIN_R_MASK))>>DCU_CTRLDESCLn_6_CKMIN_R_SHIFT;
    pLyrChromaMin->Green_Value=(REG_BIT_GET32(DCU_CTRLDESCL6_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_6_CKMIN_G_MASK))>>DCU_CTRLDESCLn_6_CKMIN_G_SHIFT;
    pLyrChromaMin->Blue_Value =(REG_BIT_GET32(DCU_CTRLDESCL6_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_6_CKMIN_B_MASK))>>DCU_CTRLDESCLn_6_CKMIN_B_SHIFT;
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

Dcu_Err_t DCU_SetLayerForeground(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id, uint32_t value)
{
	Dcu_Err_t err = DCU_ERR_OK;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if (0x00FFFFFF > value)
	{
		REG_WRITE32(DCU_CTRLDESCL8_ADDR32(dcu_id, layer_id), value);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

Dcu_Err_t DCU_GetLayerForeground(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
		*pValue = REG_READ32(DCU_CTRLDESCL8_ADDR32(dcu_id, layer_id));
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

Dcu_Err_t DCU_SetLayerBackground(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t value)
{
	Dcu_Err_t err = DCU_ERR_OK;

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if (0x00FFFFFF > value)
	{
		REG_WRITE32(DCU_CTRLDESCL9_ADDR32(dcu_id, layer_id), value);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

Dcu_Err_t DCU_GetLayerBackground(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pValue)
	{
		*pValue = REG_READ32(DCU_CTRLDESCL9_ADDR32(dcu_id, layer_id));
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
}

#if (1 == DCU_TILE_FUNCTIONALITY)
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerTileEnable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id)
#else
void DCU_LayerTileEnable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

#if (TILE_VAR_SIZE == DCU_TILE_MODE)
  REG_BIT_SET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_TILE_EN_MASK);
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */

#if (TILE_FIX_SIZE == DCU_TILE_MODE)
  REG_BIT_SET32(DCU_CTRLDESCL10_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_10_GPUTILE_EN_MASK);
#endif /* TILE_FIX_SIZE == DCU_TILE_MODE */

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerTileDisable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id)
#else
void DCU_LayerTileDisable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id)
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
{
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

#if (TILE_VAR_SIZE == DCU_TILE_MODE)
  REG_BIT_CLEAR32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_TILE_EN_MASK);
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */

#if (TILE_FIX_SIZE == DCU_TILE_MODE)
  REG_BIT_CLEAR32(DCU_CTRLDESCL10_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_10_GPUTILE_EN_MASK);
#endif /* TILE_FIX_SIZE == DCU_TILE_MODE */

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	return(DCU_ERR_OK);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
}

Dcu_Err_t DCU_IsLayerTileEnabled(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id, Dcu_Enable_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
#if (TILE_VAR_SIZE == DCU_TILE_MODE)
    *pValue = (Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id,layer_id),DCU_CTRLDESCLn_4_TILE_EN_MASK)>>DCU_CTRLDESCLn_4_TILE_EN_SHIFT);
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */

#if (TILE_FIX_SIZE == DCU_TILE_MODE)
    *pValue = (Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCL10_ADDR32(dcu_id,layer_id),DCU_CTRLDESCLn_10_GPUTILE_EN_MASK)>>DCU_CTRLDESCLn_10_GPUTILE_EN_SHIFT);
#endif /* TILE_FIX_SIZE == DCU_TILE_MODE */
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_SetLayerTile(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id, Dcu_Tile_t* pTileData)
{
	Dcu_Err_t err = DCU_ERR_OK;

	uint32_t tile_addr = pTileData->pTileData;
#if (TILE_VAR_SIZE == DCU_TILE_MODE)
	uint32_t hor_size = (pTileData->mWidth) >> 4;
	uint32_t vert_size = pTileData->mHeight;
#endif
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_PROG_END!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

#if (TILE_VAR_SIZE == DCU_TILE_MODE)
	if ((0x0800 > pTileData->mWidth) && (0x0800 > pTileData->mHeight))
	{
		/* Set the tile dimentions */
    REG_RMW32(DCU_CTRLDESCL7_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_7_TILE_HOR_SIZE_MASK, hor_size<<DCU_CTRLDESCLn_7_TILE_HOR_SIZE_SHIFT);
    REG_RMW32(DCU_CTRLDESCL7_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_7_TILE_VER_SIZE_MASK, vert_size<<DCU_CTRLDESCLn_7_TILE_VER_SIZE_SHIFT);

		/* Check if bitmap address is in the system or in the CLUT memory*/
		if ((tile_addr > (DCU_BASE_ADDR64[dcu_id] + DCU_CLUT_OFFSET)) &&
        (tile_addr < (DCU_BASE_ADDR64[dcu_id] + DCU_CLUT_OFFSET + CLUT_SIZE)))
		{
			/* Set data select bit */
      REG_BIT_SET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_DATA_SEL_MASK);
		}
		else
		{
			/* Set data select bit */
      REG_BIT_CLEAR32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_DATA_SEL_MASK);
		}
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */

		/* Set the tile bitmap address */
		REG_WRITE32(DCU_CTRLDESCL3_ADDR32(dcu_id, layer_id), tile_addr);

		/* Enable/Disable the tile mode */
		if (DCU_ENABLE == pTileData->TileEnable)
		{
#if (TILE_VAR_SIZE == DCU_TILE_MODE)
      REG_BIT_SET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_TILE_EN_MASK);
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */

#if (TILE_FIX_SIZE == DCU_TILE_MODE)
      REG_BIT_SET32(DCU_CTRLDESCL10_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_10_GPUTILE_EN_MASK);
#endif /* TILE_FIX_SIZE == DCU_TILE_MODE */
		}
		else
		{
#if (TILE_VAR_SIZE == DCU_TILE_MODE)
      REG_BIT_CLEAR32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_TILE_EN_MASK);
#endif /*(TILE_VAR_SIZE == DCU_TILE_MODE)*/

#if (TILE_FIX_SIZE == DCU_TILE_MODE)
      REG_BIT_CLEAR32(DCU_CTRLDESCL10_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_10_GPUTILE_EN_MASK);
#endif /*(TILE_FIX_SIZE == DCU_TILE_MODE)*/
		}
#if (TILE_VAR_SIZE == DCU_TILE_MODE)
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */
	return(err);
}

#if (TILE_VAR_SIZE == DCU_TILE_MODE)
Dcu_Err_t DCU_GetLayerTile(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id, Dcu_Tile_t* pTileConfig)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pTileConfig)
	{
		/* Get the tile size */
    pTileConfig->mWidth = 16*(REG_BIT_GET32(DCU_CTRLDESCL7_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_7_TILE_HOR_SIZE_MASK)>>DCU_CTRLDESCLn_7_TILE_HOR_SIZE_SHIFT);
    pTileConfig->mHeight = (REG_BIT_GET32(DCU_CTRLDESCL7_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_7_TILE_VER_SIZE_MASK))>>DCU_CTRLDESCLn_7_TILE_VER_SIZE_SHIFT;

		/* Get the tile bitmap address */
    pTileConfig->pTileData = REG_READ32(DCU_CTRLDESCL3_ADDR32(dcu_id, layer_id));

		/* Get the tile mode status */
    pTileConfig->TileEnable = (Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id, layer_id), DCU_CTRLDESCLn_4_TILE_EN_MASK)>>DCU_CTRLDESCLn_4_TILE_EN_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}
#endif /*(TILE_VAR_SIZE == DCU_TILE_MODE)*/
#endif /*(DCU_TILE_FUNCTIONALITY)*/


/**
* @brief   This function is used to read back the main parameters of a DCUs layer.
* @details The function returns the main functionality configuration of a layer.
*	The returned parameters are the size, the position, the colour coding
*	schema, the foreground and the background colours and the image
*	buffer address.
*
* @param[in]	dcu_id	- selects the DCU to be accessed.
*		layer_id - selects of the layer to be accessed.
*		pLayer_Params- pointer to the returned config data structure.
*
* @return	error code- DCU_ERR_NULL_PTR if the buffer pointer is NULL
*
* @api
*/
Dcu_Err_t DCU_GetLayerConfig(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_LayerParams_t* pLayer_Params)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if(NULL_PTR != pLayer_Params)
	{
    pLayer_Params->LayerWidth = (REG_BIT_GET32(DCU_CTRLDESCL1_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_1_WIDTH_MASK))>>DCU_CTRLDESCLn_1_WIDTH_SHIFT;
    pLayer_Params->LayerHeight= (REG_BIT_GET32(DCU_CTRLDESCL1_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_1_HEIGHT_MASK))>>DCU_CTRLDESCLn_1_HEIGHT_SHIFT;

    pLayer_Params->LayerPosX = (REG_BIT_GET32(DCU_CTRLDESCL2_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_2_POSX_MASK))>>DCU_CTRLDESCLn_2_POSX_SHIFT;
    pLayer_Params->LayerPosY = (REG_BIT_GET32(DCU_CTRLDESCL2_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_2_POSY_MASK))>>DCU_CTRLDESCLn_2_POSY_SHIFT;

    pLayer_Params->LayerBuffAdd = REG_READ32(DCU_CTRLDESCL3_ADDR32(dcu_id, layer_id));

    pLayer_Params->LayerColCode = (REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_4_BPP_MASK))>>DCU_CTRLDESCLn_4_BPP_SHIFT;
    pLayer_Params->LayerChromaKey = (REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_4_BB_MASK))>>DCU_CTRLDESCLn_4_BB_SHIFT;
    pLayer_Params->LayerAlphaBlend = (REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_4_AB_MASK))>>DCU_CTRLDESCLn_4_AB_SHIFT;
    pLayer_Params->LayerTransp = (REG_BIT_GET32(DCU_CTRLDESCL4_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_4_TRANS_MASK))>>DCU_CTRLDESCLn_4_TRANS_SHIFT;

    pLayer_Params->LayerFGNDCol = REG_READ32(DCU_CTRLDESCL8_ADDR32(dcu_id, layer_id)) & DCU_CTRLDESCLn_8_COLOR_MASK;
    pLayer_Params->LayerBGNDCol = REG_READ32(DCU_CTRLDESCL9_ADDR32(dcu_id, layer_id)) & DCU_CTRLDESCLn_9_COLOR_MASK;
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}


/****************************************************************************/
/** This function is used to write a single value to the DCUs CLUT
*
*
* \param dcu_id			 Selects the DCU to be accessed
* \param aIndex		 Index (offset) to the CLUT value
* \param aValue		 CLUT value as 00rrggbb
*
* \return Execution result
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
Dcu_Err_t DCU_CLUTSet(Dcu_Unit_t dcu_id, uint16_t aIndex, uint32_t aValue)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t mem_index = ((uint32_t)aIndex << 2);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_VBLANK!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if (DCU_CLUT_MEMSIZE > aIndex)
	{
		/* Get address of CLUT */
    vuint32_t* pCLUT = (vuint32_t*)REG_ADDR(DCU_BASE_ADDR64[dcu_id] + DCU_CLUT_OFFSET + mem_index);
		/* Set CLUT value */
		*(pCLUT) = aValue;
	}
	else
	{
		err = DCU_ERR_RANGE;
	}

	return (err);
} /* DCU_ClutSet() */

/****************************************************************************/
/** This function is used to read a single value to the DCUs CLUT
*
*
* \param dcu_id			 Selects the DCU to be accessed
* \param aIndex		 Index (offset) to the CLUT value
* \param apValue		Pointer to a variable to be filled with the value
*
* \return Execution result
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
Dcu_Err_t DCU_CLUTGet(Dcu_Unit_t dcu_id, uint16_t aIndex, uint32_t* apValue)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t mem_index = ((uint32_t)aIndex << 2);

	if (DCU_CLUT_MEMSIZE > aIndex)
	{
		if (NULL_PTR != apValue)
		{
      vuint32_t* pCLUT = (vuint32_t*)REG_ADDR(DCU_BASE_ADDR64[dcu_id] + DCU_CLUT_OFFSET + mem_index);
			/* Get CLUT value */
			*apValue = *(pCLUT);
		}
		else
		{
			err = DCU_ERR_NULL_PTR;
		}
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return (err);
} /* DCU_ClutGet() */

/****************************************************************************/
/** This function loads a part of the DCU-CLUT with the given value array and associates this CLUT
*   part to a given layer
*
*
* \param dcu_id		Selects the DCU to be accessed
* \param aBase          Base index (offset) where to start writing the values to the CLUT
* \param aCount		Number of CLUT values to write
* \param apcValues	Pointer to the CLUT value array
* \param layer_id         DCU-layer number to be associated to the new values (-1 if no)
*
* \return Execution result
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
Dcu_Err_t DCU_CLUTLoad(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint16_t LUTOffset, uint16_t LUTSize, const uint32_t* apValue)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t i;
  vuint32_t* pCLUT = (vuint32_t*)REG_ADDR(DCU_CLUT_ADDR32(dcu_id) + (LUTOffset<<2));

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_VBLANK != gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if (NULL_PTR != apValue)
	{
    if ((256 >= LUTSize) && (DCU_CLUT_MEMSIZE >= (LUTSize + LUTOffset)))
		{
			/* Copy data to CLUT memory */
			for(i=0; i < LUTSize; i++)
			{
				*(pCLUT + i) = *(apValue + i);
			} /* for() */
			/* Set CLUT offset */
      REG_RMW32(DCU_CTRLDESCL4_ADDR32(dcu_id,layer_id), DCU_CTRLDESCLn_4_LUOFFS_MASK, LUTOffset<<DCU_CTRLDESCLn_4_LUOFFS_SHIFT);
		}
		else
		{
			err = DCU_ERR_RANGE;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}

	return (err);
} /* DCU_ClutLoad() */

/****************************************************************************/
/** This function writes data to the dedicated cursor RAM
*
*
* \param dcu_id		Selects the DCU to be accessed
* \param aCount		Number of cursor values to be written (32 bit values!)
* \param apcValues	Pointer to the cursor data array
*
* \return Execution result
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
Dcu_Err_t DCU_CursorLoad(Dcu_Unit_t dcu_id, uint16_t aCount, const uint32_t* apcValues)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t i;
  vuint32_t* pCursor = (vuint32_t*)REG_ADDR(DCU_BASE_ADDR64[dcu_id] + DCU_CURSOR_OFFSET);

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_VBLANK!=gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if(NULL_PTR != apcValues)
	{/* Copy data to Cursor RAM memory */
		for(i=0;i<aCount;i++)
		{
			*(pCursor	+ i) = *(apcValues+i);
		} /* for() */
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return (err);
} /* DCU_CursorLoad() */

/****************************************************************************/
/** This function loads the gamma correction tables
*
*
* \param dcu_id			Selects the DCU to be accessed
* \param apcR	Pointer to the "red" correction table (must contain 256 values)
* \param apcG	Pointer to the "green"correction table (must contain 256 values)
* \param apcB	Pointer to the "blue" correction table (must contain 256 values)
*
* \return Execution result
*
* \bug
* \todo
* \note
* \sa	 DCU_FNC_GAMMA_ENABLE
*****************************************************************************/
Dcu_Err_t DCU_GammaLoad(Dcu_Unit_t dcu_id, Dcu_Gamma_t *pGammaCorrectionData)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t i;

  uint32_t* pGammaRed   = (uint32_t*)REG_ADDR(DCU_GAMMARED_ADDR32(dcu_id));
  uint32_t* pGammaGreen = (uint32_t*)REG_ADDR(DCU_GAMMAGREEN_ADDR32(dcu_id));
  uint32_t* pGammaBlue  = (uint32_t*)REG_ADDR(DCU_GAMMABLUE_ADDR32(dcu_id));

#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
	if(DCU_VBLANK != gDCU_DisplayIStatus[dcu_id])
	{
		return(DCU_ERR_CALLTIME);
	}
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */

	if(NULL_PTR != pGammaCorrectionData)
	{
		uint32_t *pRed	 = pGammaCorrectionData->pRedGamma;
		uint32_t *pGreen = pGammaCorrectionData->pGreenGamma;
		uint32_t *pBlue	= pGammaCorrectionData->pBlueGamma;

    if((NULL_PTR != pRed) && (NULL_PTR != pGreen) && (NULL_PTR != pBlue))
		{/* Copy data to RAM memory */
			/* Load to gamma "red" table */
			for (i=0;i<256;i++)
			{
				pGammaRed[i] = pRed[i];
			} /* for() */

			/* Load to gamma "green" table */
			for (i=0;i<256;i++)
			{
				pGammaGreen[i] = pGreen[i];
			} /* for() */

			/* Load to gamma "blue" table */
			for (i=0;i<256;i++)
			{
				pGammaBlue[i] = pBlue[i];
			} /* for() */
		}
		else
		{
			err = DCU_ERR_NULL_PTR;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
} /* Dcu_GammaLoad() */

/****************************************************************************/
/** This function sets the DCUs default background color register (no pixel blended)
*
*
* \param dcu_id		 Selects the DCU to be accessed
* \param aRed		 The red part of the background color
* \param aGreen	 The green part of the background color
* \param aBlue		The blue part of the background color
*
* \return void
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
Dcu_Err_t DCU_BGNDColorSet(Dcu_Unit_t dcu_id, Dcu_Colour_t* pBGNDColour)
{
	Dcu_Err_t err = DCU_ERR_OK;
	vuint32_t vBGND;

	if (NULL_PTR != pBGNDColour)
	{
		vBGND	= (pBGNDColour->Red_Value << DCU_BGND_BGND_R_SHIFT);
		vBGND |= (pBGNDColour->Green_Value<< DCU_BGND_BGND_G_SHIFT);
		vBGND |= (pBGNDColour->Blue_Value << DCU_BGND_BGND_B_SHIFT);

		/* Set BGND color */
		REG_WRITE32(DCU_BGND_ADDR32(dcu_id),vBGND);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return err;
} /* Dcu_BGNDColorSet() */

#if (1 == DCU_IRQ_SUPPORT)
/****************************************************************************/
/** This function handle interrupt for DCU
*
*
* \param dcu_id		 Selects the DCU to be accessed
* \return void
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
static void DCU_Timing_Isr(Dcu_Unit_t dcu_id)
{
	uint32_t int_status;
#if (64 == DCU_LAYERS_NUM_MAX)
	uint32_t int_hwstatus1;
#endif

	/* Check LS_BF_VS interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_LS_BF_VS_MASK);
	if(0 != int_status)
	{
    gDCU_DisplayIStatus[dcu_id] = DCU_LSBFVS;

		/* Call callback function */
		if (gpCallbackLSBFVS[dcu_id] != NULL_PTR)
		{
			gpCallbackLSBFVS[dcu_id]();
    }

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_LS_BF_VS_MASK);
  }

	/* Check VS_BLANK interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_VS_BLANK_MASK);
	if(0 != int_status)
	{
    gDCU_DisplayIStatus[dcu_id] = DCU_VBLANK;

		/* Call callback function */
		if (gpCallbackVSBLANK[dcu_id] != NULL_PTR)
		{
			gpCallbackVSBLANK[dcu_id]();
    }

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_VS_BLANK_MASK);
  }

	/* Check VSYNC interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_VSYNC_MASK);
	if(0 != int_status)
	{
    gDCU_DisplayIStatus[dcu_id] = DCU_VSYNC;

		/* Call callback function */
		if (gpCallbackVSYNC[dcu_id] != NULL_PTR)
		{
			gpCallbackVSYNC[dcu_id]();
    }

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_VSYNC_MASK);
  }

	/* Check DMA trans finish interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_DMA_TRANS_FINISH_MASK);
	if(0 != int_status)
	{
    gDCU_DisplayIStatus[dcu_id] = DCU_DATA_DONE;

		/* Call callback function */
		if (gpCallbackDMATRANSFIN[dcu_id] != NULL_PTR)
		{
			gpCallbackDMATRANSFIN[dcu_id]();
    }

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_DMA_TRANS_FINISH_MASK);
  }

  /* Check programm end interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_PROG_END_MASK);
	if(0 != int_status)
	{
    gDCU_DisplayIStatus[dcu_id] = DCU_PROG_END;

		/* Call callback function */
    if (gpCallbackPROGEND[dcu_id] != NULL_PTR)
		{
      gpCallbackPROGEND[dcu_id]();
    }

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_PROG_END_MASK);
  }

  /* Check layer transfer finish interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_LYR_TRANS_FINISH_MASK);
	if(0 != int_status)
	{
    gDCU_DisplayIStatus[dcu_id] = DCU_LYRCFG_DONE;

		/* Call callback function */
    if (gpCallbackLYRTRANSFIN[dcu_id] != NULL_PTR)
		{
      gpCallbackLYRTRANSFIN[dcu_id]();
    }

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_LYR_TRANS_FINISH_MASK);
  }

#if (1 == DCU_SAFETY_FUNCTIONALITY)
	/* Check CRC ready interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_CRC_READY_MASK);
	if(0 != int_status)
	{
		if (gpCallbackCRCREADY[dcu_id] != NULL_PTR)
		{
			/* Call callback function */
			gpCallbackCRCREADY[dcu_id]();
		}

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_CRC_READY_MASK);
  }

	/* Check CRC overflow interrupt */
  int_status = REG_BIT_GET32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_CRC_OVERFLOW_MASK);
	if(0 != int_status)
	{
		if (gpCallbackCRCOFV[dcu_id] != NULL_PTR)
		{
			/* Call callback function */
			gpCallbackCRCOFV[dcu_id]();
		}

		/* Clear the interrupt flag */
    REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), DCU_INT_STATUS_CRC_OVERFLOW_MASK);
		}
#endif /* DCU_SAFETY_FUNCTIONALITY */

}/* Dcu_Timing_Isr() */

/****************************************************************************/
/** This function is interrupt service routine for DCU0
*
*
* \param
* \return void
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
void DCU0_Timing_Isr(void)
{
	DCU_Timing_Isr(0);
}/* DCU0_Timing_Isr() */

/****************************************************************************/
/** This function is interrupt service routine for DCU1
*
*
* \param
* \return void
*
* \bug
* \todo
* \note
* \sa
*****************************************************************************/
void DCU1_Timing_Isr(void)
{
	DCU_Timing_Isr(1);
}/* DCU1_Timing_Isr() */

#endif /* DCU_IRQ_SUPPORT */

/*================================================================================================*/
/** 
* @brief	 Dcu driver update mode function.
* @details The function sets the periodic or the async layer config update mode and
*          in case of asynch commands the update at the next vertical blanking period.
*
* @param[in]	dcu_id	The DCU unit to be accessed.
*		 mode	The update mode.
*
* @return
*/
void DCU_SetUpdateMode(Dcu_Unit_t dcu_id, Dcu_UpdateMode_t mode)
{
	if(DCU_PERIODIC_MODE == mode)
	{
		/* Set the automatic update mode */
    REG_WRITE32(DCU_UPDATE_MODE_ADDR32(dcu_id),DCU_UPDATE_MODE_MODE_MASK);
	}

	if(DCU_ASYNC_MODE == mode)
	{
		/* Set the async update mode */
    REG_BIT_CLEAR32(DCU_UPDATE_MODE_ADDR32(dcu_id),DCU_UPDATE_MODE_MODE_MASK);
		/* Command the next update */
    REG_BIT_SET32(DCU_UPDATE_MODE_ADDR32(dcu_id),DCU_UPDATE_MODE_READREG_MASK);
    while ((REG_READ32(DCU_UPDATE_MODE_ADDR32(dcu_id)) & DCU_UPDATE_MODE_READREG_MASK))
		{
		}
	}
}

Dcu_Err_t DCU_GetUpdateMode(Dcu_Unit_t dcu_id, Dcu_UpdateMode_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue=(Dcu_UpdateMode_t)((REG_BIT_GET32(DCU_UPDATE_MODE_ADDR32(dcu_id),DCU_UPDATE_MODE_MODE_MASK))>>DCU_UPDATE_MODE_MODE_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

void DCU_SetLinesBeforeVSYNC(Dcu_Unit_t dcu_id, uint8_t uLines)
{
	/* Set value */
  REG_RMW32(DCU_THRESHOLD_ADDR32(dcu_id), DCU_THRESHOLD_LS_BF_VS_MASK, uLines << DCU_THRESHOLD_LS_BF_VS_SHIFT);
}

Dcu_Err_t DCU_SetCursor(Dcu_Unit_t dcu_id, Dcu_Cursor_t *pCursorSettings)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pCursorSettings)
	{
    if ((0x0800 > pCursorSettings->CursorHeight) && (0x0800 > pCursorSettings->CursorWidth))
		{
      REG_BIT_CLEAR32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id), DCU_CTRLDESCCURSOR3_CUR_EN_MASK);

			/* Set cursor size */
      REG_RMW32(DCU_CTRLDESCCURSOR1_ADDR32(dcu_id), DCU_CTRLDESCCURSOR1_HEIGHT_MASK, (pCursorSettings->CursorHeight) << DCU_CTRLDESCCURSOR1_HEIGHT_SHIFT);
      REG_RMW32(DCU_CTRLDESCCURSOR1_ADDR32(dcu_id), DCU_CTRLDESCCURSOR1_WIDTH_MASK, (pCursorSettings->CursorWidth) << DCU_CTRLDESCCURSOR1_WIDTH_SHIFT);

      if ((0x0800 > pCursorSettings->CursorPosX)&&(0x0800 > pCursorSettings->CursorPosY)&&(0x01000000 > pCursorSettings->CursorColour))
			{
				/* Set cursor position */
        REG_RMW32(DCU_CTRLDESCCURSOR2_ADDR32(dcu_id), DCU_CTRLDESCCURSOR2_POSX_MASK, (pCursorSettings->CursorPosX) << DCU_CTRLDESCCURSOR2_POSX_SHIFT);
        REG_RMW32(DCU_CTRLDESCCURSOR2_ADDR32(dcu_id), DCU_CTRLDESCCURSOR2_POSY_MASK, (pCursorSettings->CursorPosY) << DCU_CTRLDESCCURSOR2_POSY_SHIFT);
				/* Set cursor colour */
        REG_RMW32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id), DCU_CTRLDESCCURSOR3_DEFAULT_CURSOR_COLOR_MASK, pCursorSettings->CursorColour << DCU_CTRLDESCCURSOR3_DEFAULT_CURSOR_COLOR_SHIFT);

				if (DCU_ENABLE == pCursorSettings->CursorEnable)
				{
          REG_BIT_SET32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id), DCU_CTRLDESCCURSOR3_CUR_EN_MASK);
				}
				else
				{
				}
			}
			else
			{
				err = DCU_ERR_RANGE;
			}
		}
		else
		{
			err = DCU_ERR_RANGE;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_SetCursorPosition(Dcu_Unit_t dcu_id, Dcu_Position_t *pPosition)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pPosition)
	{
		if ((0x0800 > pPosition->mX)&&(0x0800 > pPosition->mY))
		{
			/* Set cursor position */
      REG_RMW32(DCU_CTRLDESCCURSOR2_ADDR32(dcu_id), DCU_CTRLDESCCURSOR2_POSX_MASK, (pPosition->mX) << DCU_CTRLDESCCURSOR2_POSX_SHIFT);
      REG_RMW32(DCU_CTRLDESCCURSOR2_ADDR32(dcu_id), DCU_CTRLDESCCURSOR2_POSY_MASK, (pPosition->mY) << DCU_CTRLDESCCURSOR2_POSY_SHIFT);
		}
		else
		{
			err = DCU_ERR_RANGE;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}


Dcu_Err_t DCU_SetCursorColour(Dcu_Unit_t dcu_id, uint32_t uColour)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (0x01000000 > uColour)
	{
		/* Set cursor colour */
    REG_RMW32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id), DCU_CTRLDESCCURSOR3_DEFAULT_CURSOR_COLOR_MASK, uColour << DCU_CTRLDESCCURSOR3_DEFAULT_CURSOR_COLOR_SHIFT);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

void DCU_CursorEnable(Dcu_Unit_t dcu_id)
{
  REG_BIT_SET32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id), DCU_CTRLDESCCURSOR3_CUR_EN_MASK);
}

void DCU_CursorDisable(Dcu_Unit_t dcu_id)
{
  REG_BIT_CLEAR32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id), DCU_CTRLDESCCURSOR3_CUR_EN_MASK);
}

Dcu_Err_t DCU_IsCursorEnabled(Dcu_Unit_t dcu_id, Dcu_Enable_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue=(Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCCURSOR3_ADDR32(dcu_id),DCU_CTRLDESCCURSOR3_CUR_EN_MASK)>>DCU_CTRLDESCCURSOR3_CUR_EN_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_SetCursorBlinking(Dcu_Unit_t dcu_id, Dcu_CursorBlink_t* pBlinkingData)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pBlinkingData)
	{
    if ((0x100 > pBlinkingData->CursorBlinkOn)&&(0x100 > pBlinkingData->CursorBlinkOff))
		{
      REG_BIT_CLEAR32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_EN_BLINK_MASK);

			/* Set blinking timing */
      REG_RMW32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_HWC_BLINK_ON_MASK, (pBlinkingData->CursorBlinkOn) << DCU_CTRLDESCCURSOR4_HWC_BLINK_ON_SHIFT);
      REG_RMW32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_HWC_BLINK_OFF_MASK, (pBlinkingData->CursorBlinkOff) << DCU_CTRLDESCCURSOR4_HWC_BLINK_OFF_SHIFT);

			if (DCU_ENABLE == pBlinkingData->CursorBlinkEn)
			{
        REG_BIT_SET32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_EN_BLINK_MASK);
			}
			else
			{
			}
		}
		else
		{
			err = DCU_ERR_RANGE;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

void DCU_CursorBlinkingEnable(Dcu_Unit_t dcu_id)
{
  REG_BIT_SET32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_EN_BLINK_MASK);
}

void DCU_CursorBlinkingDisable(Dcu_Unit_t dcu_id)
{
  REG_BIT_CLEAR32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id), DCU_CTRLDESCCURSOR4_EN_BLINK_MASK);
}

Dcu_Err_t DCU_IsCursorBlinkingEnabled(Dcu_Unit_t dcu_id, Dcu_Enable_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue=(Dcu_Enable_t)(REG_BIT_GET32(DCU_CTRLDESCCURSOR4_ADDR32(dcu_id),DCU_CTRLDESCCURSOR4_EN_BLINK_MASK)>>DCU_CTRLDESCCURSOR4_EN_BLINK_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

void DCU_GammaCorrectionEnable(Dcu_Unit_t dcu_id)
{
	REG_BIT_SET32(DCU_MODE_ADDR32(dcu_id), DCU_EN_GAMMA_MASK);
}

void DCU_GammaCorrectionDisable(Dcu_Unit_t dcu_id)
{
	REG_BIT_CLEAR32(DCU_MODE_ADDR32(dcu_id), DCU_EN_GAMMA_MASK);
}

Dcu_Err_t DCU_IsGammaCorrectionEnabled(Dcu_Unit_t dcu_id, Dcu_Enable_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue=(Dcu_Enable_t)(REG_BIT_GET32(DCU_MODE_ADDR32(dcu_id), DCU_EN_GAMMA_MASK)>>DCU_EN_GAMMA_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

#if (1 == DCU_SAFETY_FUNCTIONALITY)
/*================================================================================================*/
/**
* @brief	 Dcu driver CRC area setting function.
* @details This is the function of the DCU driver to be called to set the display area for
*          which the signature is calculated.
*
* @param[in]	dcu_id	 Selects the DCU unit to be accessed.
*		pSAPosition	Pointer to area position structure.
*		pSASize		Pointer to area size structure.
*
* @return     error code = DCU_ERR_RANGE  For range or position larger than the display size.
*/
Dcu_Err_t DCU_SetSignedArea(Dcu_Unit_t dcu_id, Dcu_Position_t* pSAPosition, Dcu_Size_t* pSASize)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t temp_reg;

  if (((Display_SizeY[dcu_id] >= pSASize->mHeight) && (Display_SizeX[dcu_id] >= pSASize->mWidth)) 
      && ((Display_SizeY[dcu_id] >= pSAPosition->mY) && (Display_SizeX[dcu_id] >= pSAPosition->mX)))
  {
    temp_reg  = (((pSASize->mWidth)<<(DCU_SIGN_CALC1_HOR_SIZE_SHIFT)) & DCU_SIGN_CALC1_HOR_SIZE_MASK);
    temp_reg |= (((pSASize->mHeight)<<(DCU_SIGN_CALC1_VER_SIZE_SHIFT)) & DCU_SIGN_CALC1_VER_SIZE_MASK);
		REG_WRITE32(DCU_SIGN_CALC1_ADDR32(dcu_id), temp_reg);

    temp_reg  = (((pSAPosition->mX)<<(DCU_SIGN_CALC2_HOR_POS_SHIFT)) & DCU_SIGN_CALC2_HOR_POS_MASK);
    temp_reg |= (((pSAPosition->mY)<<(DCU_SIGN_CALC2_VERT_POS_SHIFT)) & DCU_SIGN_CALC2_VERT_POS_MASK);
		REG_WRITE32(DCU_SIGN_CALC2_ADDR32(dcu_id), temp_reg);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}

	return (err);
}

/*================================================================================================*/
/**
* @brief	 Dcu driver CRC seting and enable function.
* @details This function sets the CRC calculation mode
*		and enables the CRC engine.
*
* @param[in]	dcu_id	- selects the DCU unit to be accessed.
*		 xCalcMode	 - the calculation mode.
*
* @return		 void
*/
void DCU_SignatureEnable(Dcu_Unit_t dcu_id, Dcu_CRCmode_t xCalcMode)
{
  REG_RMW32(DCU_MODE_ADDR32(dcu_id), DCU_MODE_TAG_EN_MASK, (xCalcMode<<DCU_MODE_TAG_EN_SHIFT));
	REG_BIT_SET32(DCU_MODE_ADDR32(dcu_id), DCU_MODE_SIG_EN_MASK);
}

/*================================================================================================*/
/**
* @brief	 Dcu driver CRC block disable function.
* @details The function sets the CRC calculation mode
*		and to enable the CRC engine.
*
* @param[in]	dcu_id	 Selects the DCU unit to be accessed.
*		xCalcMode		The caculation mode.
*
* @return	 void
*/
void DCU_SignatureDisable(Dcu_Unit_t dcu_id)
{
	REG_BIT_CLEAR32(DCU_MODE_ADDR32(dcu_id), DCU_MODE_SIG_EN_MASK);
}

/*================================================================================================*/
/**
* @brief   Dcu driver CRC block and sets the CRC calculation function enables the  mode.
* @details This function of the DCU driver to be called to set the CRC calculation mode
*          and to enable the CRC engine.
*
* @param[in]	dcu_id	Selects the DCU unit to be accessed.
*             pValue       The pointer to the structure where the signature is load.
*
* @return	void
*/
Dcu_Err_t DCU_GetSignature(Dcu_Unit_t dcu_id, Dcu_Signature_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
		pValue->mCRCVal = REG_READ32(DCU_CRC_VAL_ADDR32(dcu_id));
		pValue->mCRCPos = REG_READ32(DCU_CRC_POS_ADDR32(dcu_id));
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}

	return (err);
}

/*================================================================================================*/
/**
* @brief	Dcu driver layer safe enable function.
* @details This function enables the safe functionality
*					for the selected layer(0 or 1).
*
* @param[in]	dcu_id	selects the DCU unit to be accessed.
*		s_layer	selects the layer to be safe(0 or 1).
*
* @return	void
*/
void DCU_LayerSafeEnable(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer)
{
  REG_BIT_SET32(DCU_CTRLDESCL4_ADDR32(dcu_id, s_layer), DCU_CTRLDESCLn_4_SAFETY_EN_MASK);
}

/*================================================================================================*/
/**
* @brief	 Dcu driver layer safe disable function.
* @details This function disables the safe functionality
*					for the selected layer(0 or 1).
*
* @param[in]	dcu_id	selects the DCU unit to be accessed.
*		s_layer	selects the layer(0 or 1).
*
* @return	void
*/
void DCU_LayerSafeDisable(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer)
{
  REG_BIT_CLEAR32(DCU_CTRLDESCL4_ADDR32(dcu_id, s_layer), DCU_CTRLDESCLn_4_SAFETY_EN_MASK);
}

void DCU_LayerRegsLock(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer)
{
	if(DCU_SAFE_LAYER0==s_layer)
	{
    REG_WRITE32(DCU_SLR_L0_ADDR32(dcu_id), (DCU_SLR_L0_WREN_MASK | DCU_SLR_L0_SLOCK_MASK));
    REG_WRITE32(DCU_SLR_L0_TRANSP_ADDR32(dcu_id), (DCU_SLR_L0TRANSP_WREN_MASK | DCU_SLR_L0TRANSP_SLOCK_MASK));

		REG_BIT_CLEAR32(DCU_SLR_L0_ADDR32(dcu_id), DCU_SLR_L0_WREN_MASK);
    REG_BIT_CLEAR32(DCU_SLR_L0_TRANSP_ADDR32(dcu_id), DCU_SLR_L0TRANSP_WREN_MASK);
	}
	else
	{
    REG_WRITE32(DCU_SLR_L1_ADDR32(dcu_id), (DCU_SLR_L1_WREN_MASK | DCU_SLR_L1_SLOCK_MASK));
    REG_WRITE32(DCU_SLR_L1_TRANSP_ADDR32(dcu_id), (DCU_SLR_L1TRANSP_WREN_MASK | DCU_SLR_L1TRANSP_SLOCK_MASK));

    REG_BIT_CLEAR32(DCU_SLR_L1_ADDR32(dcu_id), DCU_SLR_L1_WREN_MASK);
    REG_BIT_CLEAR32(DCU_SLR_L1_TRANSP_ADDR32(dcu_id), DCU_SLR_L1TRANSP_WREN_MASK);
	}
}

void DCU_LayerRegsUnlock(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer)
{
	if(DCU_SAFE_LAYER0==s_layer)
	{
		REG_WRITE32(DCU_SLR_L0_ADDR32(dcu_id), DCU_SLR_L0_WREN_MASK);
    REG_WRITE32(DCU_SLR_L0_TRANSP_ADDR32(dcu_id), DCU_SLR_L0TRANSP_WREN_MASK);

		REG_BIT_CLEAR32(DCU_SLR_L0_ADDR32(dcu_id), DCU_SLR_L0_WREN_MASK);
    REG_BIT_CLEAR32(DCU_SLR_L0_TRANSP_ADDR32(dcu_id), DCU_SLR_L0TRANSP_WREN_MASK);
	}
	else
	{
		REG_WRITE32(DCU_SLR_L1_ADDR32(dcu_id), DCU_SLR_L1_WREN_MASK);
    REG_WRITE32(DCU_SLR_L1_TRANSP_ADDR32(dcu_id), DCU_SLR_L1TRANSP_WREN_MASK);

		REG_BIT_CLEAR32(DCU_SLR_L1_ADDR32(dcu_id), DCU_SLR_L1_WREN_MASK);
    REG_BIT_CLEAR32(DCU_SLR_L1_TRANSP_ADDR32(dcu_id), DCU_SLR_L1TRANSP_WREN_MASK);
	}
}

void DCU_DisplayInterfaceLock(Dcu_Unit_t dcu_id)
{
  REG_WRITE32(DCU_SLR_DISP_SIZE_ADDR32(dcu_id), (DCU_SLR_DISP_SIZE_WREN_DISP_MASK | DCU_SLR_DISP_SIZE_SLOCK_MASK));
  REG_WRITE32(DCU_SLR_HVSYNC_PARA_ADDR32(dcu_id), (DCU_SLR_HVSYNC_PARA_WREN_MASK | DCU_SLR_HVSYNC_PARA_SLOCK_MASK));
  REG_WRITE32(DCU_SLR_HVSYNC_POL_ADDR32(dcu_id), (DCU_SLR_POL_WREN_MASK | DCU_SLR_POL_SLOCK_MASK));

  REG_BIT_CLEAR32(DCU_SLR_DISP_SIZE_ADDR32(dcu_id), DCU_SLR_DISP_SIZE_WREN_DISP_MASK);
  REG_BIT_CLEAR32(DCU_SLR_HVSYNC_PARA_ADDR32(dcu_id), DCU_SLR_HVSYNC_PARA_WREN_MASK);
  REG_BIT_CLEAR32(DCU_SLR_HVSYNC_POL_ADDR32(dcu_id), DCU_SLR_POL_WREN_MASK);
}

void DCU_DisplayInterfaceUnlock(Dcu_Unit_t dcu_id)
{
  REG_WRITE32(DCU_SLR_DISP_SIZE_ADDR32(dcu_id), DCU_SLR_DISP_SIZE_WREN_DISP_MASK);
  REG_WRITE32(DCU_SLR_HVSYNC_PARA_ADDR32(dcu_id), DCU_SLR_HVSYNC_PARA_WREN_MASK);
	REG_WRITE32(DCU_SLR_HVSYNC_POL_ADDR32(dcu_id), DCU_SLR_POL_WREN_MASK);

  REG_BIT_CLEAR32(DCU_SLR_DISP_SIZE_ADDR32(dcu_id), DCU_SLR_DISP_SIZE_WREN_DISP_MASK);
  REG_BIT_CLEAR32(DCU_SLR_HVSYNC_PARA_ADDR32(dcu_id), DCU_SLR_HVSYNC_PARA_WREN_MASK);
  REG_BIT_CLEAR32(DCU_SLR_HVSYNC_POL_ADDR32(dcu_id), DCU_SLR_POL_WREN_MASK);
}
#endif /* DCU_SAFETY_FUNCTIONALITY */

#if (1 == DCU_IRQ_SUPPORT)
/**
* @brief	It registers the VS_BLANK callback.
* @details This function registers the gpCallbackVSBLANK callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id	selects the DCU unit to be accessed.
*		aCallback the calback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.
*/
Dcu_Err_t DCU_RegisterCallbackVBLANK(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackVSBLANK[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/**
* @brief	 It registers the VSYNK callback.
* @details This function registers the gpCallbackVSYNC callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the calback function.
*
* @return	error code -DCU_ERR_NOCALLBACK for NULL pointer to the callback.
*/
Dcu_Err_t DCU_RegisterCallbackVSYNC(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackVSYNC[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/**
* @brief	 It registers the LSBVFS callback.
* @details This function registers the gpCallbackLSBFVS callback function
*			if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the calback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.
*
*/
Dcu_Err_t DCU_RegisterCallbackLSBFVS(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackLSBFVS[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/**
* @brief	 The function enables the display interface timing interrupts.
* @details The function enables one of, more or all display interface interrupts:
*	VS_BLANK, LS_BF_VS, VSYNC, LYR_TRANS_FINISH, DATA_TRANS_FINISH
*	and/or PROG_END. The interrupts which are not in the mask remain
*	as previous this function call(enabled or disabled).
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*             int_mask    the interrupt mask: VS_BLANK, LS_BF_VS, VSYNC,
*		LYR_TRANS_FINISH, DATA_TRANS_FINISH, PROG_END,
*		or an OR between any of them.
*
* @return	err code	- DCU_ERR_MASK	for a wrong interrupt mask.
*/
Dcu_Err_t DCU_EnableDisplayTimingIrq(Dcu_Unit_t dcu_id, uint32_t int_mask)
{
	Dcu_Err_t err = DCU_ERR_OK;
  uint32_t mask = int_mask & (~DCU_INT_TIMING_MASK);

  if (0 != int_mask)
  {
	if (0 == mask)
	{
      REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), int_mask);
      REG_BIT_CLEAR32(DCU_INT_MASK_ADDR32(dcu_id), int_mask);
    }
    else
    {
		err = DCU_ERR_MASK;
	}
  }
	else
	{
      err = DCU_ERR_MASK;
	}
	return(err);
}

/**
* @brief	 The function disables the display interface timing interrupts.
* @details The function disables one, more or all the display interface interrupts:
*		VS_BLANK, LS_BF_VS, VSYNC, LYR_TRANS_FINISH, DATA_TRANS_FINISH
*		and/or PROG_END. The interrupts which are not in the mask remain
*		as previous this function call(enabled or disabled).
*
* @param[in]  dcu_id      selects the DCU unit to be accessed.
*             int_mask    the interrupt mask: VS_BLANK, LS_BF_VS and/or VSYNC
*			LYR_TRANS_FINISH, DATA_TRANS_FINISH, PROG_END,
*			or an OR between any of them.
*
* @return	 err code	- DCU_ERR_MASK	for a wrong interrupt mask.
*/
Dcu_Err_t DCU_DisableDisplayTimingIrq(Dcu_Unit_t dcu_id, uint32_t int_mask)
{
	Dcu_Err_t err = DCU_ERR_OK;
  uint32_t mask = int_mask & (~DCU_INT_TIMING_MASK);

  if (0 != int_mask)
  {
    if ((uint32_t)0 == mask)
    {
      REG_BIT_SET32(DCU_INT_MASK_ADDR32(dcu_id), int_mask);
      REG_WRITE32(DCU_INT_STATUS_ADDR32(dcu_id), int_mask);
    }
    else
	{
		err = DCU_ERR_MASK;
	}
  }
	else
	{
      err = DCU_ERR_MASK;
	}
	return(err);
}

/**
* @brief	 It registers the PROG_END callback.
* @details This function registers the gpCallbackPROGEND callback function
*			if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the calback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.
*/
Dcu_Err_t DCU_RegisterCallbackProgDone(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackPROGEND[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/**
* @brief	 It registers the DATA_TRANS_FINISH callback.
* @details This function registers the gpCallbackDMATRANSFIN callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the calback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.
*/
Dcu_Err_t DCU_RegisterCallbackDataTransferDone(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackDMATRANSFIN[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/**
* @brief	 It registers the LYR_TRANS_FINISH callback.
* @details This function registers the gpCallbackLYRTRANSFIN callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the calback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.
*/
Dcu_Err_t DCU_RegisterCallbackLyrCfgTransferDone(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackLYRTRANSFIN[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

#if (1 == DCU_SAFETY_FUNCTIONALITY)
/** 
* @brief	 It registers the CRC READY callback.
* @details This function registers the gpCallbackCRCREADY callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id		- selects the DCU unit to be accessed.
*		aCallback	- the calback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for a NULL pointer to the callback.
*/
Dcu_Err_t DCU_RegisterCallbackCRCReady(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackCRCREADY[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/**
* @brief	 It registers the CRC overflow callback.
* @details This function registers the gpCallbackCRCOFV callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the calback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.
*/
Dcu_Err_t DCU_RegisterCallbackCRCOverflow(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackCRCOFV[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/**
* @brief	 The function enables the safety interrupts.
* @details The function enables one of or both CRC interrupts: CRCREADY and/or
*		 CRCOVF.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		int_mask	the interrupt mask: DCU_INT_CRC_READY_MASK,
*			 DCU_INT_CRC_OVERFLOW_MASK or an or between them.
*
* @return	 err code	- DCU_ERR_MASK	for a wrong interrupt mask.
*                     - DCU_ERR_NOCALLBACK for a int without a registered callback.
*/
Dcu_Err_t DCU_EnableSafetyIrq(Dcu_Unit_t dcu_id, uint32_t int_mask)
{
	Dcu_Err_t err = DCU_ERR_OK;

  if ((0 == (int_mask & DCU_INT_CRC_MASK)) || (0 == int_mask))
	{
		err = DCU_ERR_MASK;
	}
  else
  {
    if (DCU_INT_CRC_OVERFLOW_MASK == (int_mask & DCU_INT_CRC_OVERFLOW_MASK))
	{
		if (NULL_PTR != gpCallbackCRCOFV[dcu_id])
		{
        REG_BIT_CLEAR32(DCU_INT_MASK_ADDR32(dcu_id), DCU_INT_CRC_OVERFLOW_MASK);
		}
		else
		{
			err = DCU_ERR_NOCALLBACK;
		}
	}
    if (DCU_INT_CRC_READY_MASK == (int_mask & DCU_INT_CRC_READY_MASK))
	{
		if(NULL_PTR != gpCallbackCRCREADY[dcu_id])
		{
        REG_BIT_CLEAR32(DCU_INT_MASK_ADDR32(dcu_id), DCU_INT_CRC_READY_MASK);
		}
		else
		{
			err = DCU_ERR_NOCALLBACK;
		}
	}
  }
	return(err);
}

/**
* @brief	 The function disables the safety interrupts.
* @details The function disables one of or both safety interrupts: CRCREADY
*						 and/or CRCOVF.
*
* @param[in]	dcu_id		 - selects the DCU unit to be accessed.
*		int_mask	 - the interrupt mask: DCU_INT_CRC_READY_MASK,
*		DCU_INT_CRC_OVERFLOW_MASK or an or between them.
*
* @return	 err code	- DCU_ERR_MASK	for a wrong interrupt mask.
*/
Dcu_Err_t DCU_DisableSafetyIrq(Dcu_Unit_t dcu_id, uint32_t int_mask)
{
	Dcu_Err_t err = DCU_ERR_OK;

  if ((0 == (int_mask & DCU_INT_CRC_MASK)) || (0 == int_mask))
	{
		err = DCU_ERR_MASK;
	}
	else
	{
    REG_BIT_SET32(DCU_INT_MASK_ADDR32(dcu_id), int_mask);
	}
	return(err);
}
#endif /* DCU_SAFETY_FUNCTIONALITY */
#endif /* DCU_IRQ_SUPPORT */

#if (1 == DCU_HUD_FUNCTIONALITY)
/***************		HUD SUPPORT		***************/
Dcu_Err_t DCU_SetHUDConfig(Dcu_HUDUnit_t dcu_id, Dcu_Warp_t* pHUDConfig)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pHUDConfig)
	{
		uint32_t LocalVal = 0UL;

    if ((0 == ((pHUDConfig->HUDWidth) & ~DCU_HUD_SIZE_MASK)) || (0 == ((pHUDConfig->HUDHeight) & ~DCU_HUD_SIZE_MASK)))
    {
      REG_WRITE32(DCU_HUD_WIDTH_ADDR32(dcu_id), (pHUDConfig->HUDWidth) & DCU_HUD_SIZE_MASK);
      REG_WRITE32(DCU_HUD_HEIGHT_ADDR32(dcu_id), (pHUDConfig->HUDHeight) & DCU_HUD_SIZE_MASK);

      REG_WRITE32(DCU_WARP_DESC_ADDR_ADDR32(dcu_id), pHUDConfig->HUDLDAddress);

      if ( (0 == (((pHUDConfig->HUDLBSize)<<DCU_WARP_CTRL_LINESLB_SHIFT) & ~DCU_WARP_CTRL_LINESLB_MASK)) ||
           (0 == (((pHUDConfig->HUDAXISize)<<DCU_WARP_CTRL_AXIXFRS_SHIFT) & ~DCU_WARP_CTRL_AXIXFRS_MASK)) )
      {
        LocalVal |= ((pHUDConfig->HUDLBSize << DCU_WARP_CTRL_LINESLB_SHIFT) & DCU_WARP_CTRL_LINESLB_MASK);  
        LocalVal |= ((pHUDConfig->HUDAXISize << DCU_WARP_CTRL_AXIXFRS_SHIFT) & DCU_WARP_CTRL_AXIXFRS_MASK);  
				if (DCU_ENABLE == pHUDConfig->HUDEnable)
				{
					LocalVal |= 1UL;
				}
        REG_WRITE32(DCU_WARP_CTRL_ADDR32(dcu_id), LocalVal);
			}
			else
			{
				err = DCU_ERR_RANGE;
			}
		}
		else
		{
			err = DCU_ERR_RANGE;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);

}

void DCU_HUDEnable(Dcu_HUDUnit_t dcu_id) 
{
	REG_BIT_SET32(DCU_WARP_CTRL_ADDR32(dcu_id), DCU_WARP_CTRL_HUDEN_MASK);
}

void DCU_HUDDisable(Dcu_HUDUnit_t dcu_id)
{
	REG_BIT_CLEAR32(DCU_WARP_CTRL_ADDR32(dcu_id), DCU_WARP_CTRL_HUDEN_MASK);
}

Dcu_Err_t DCU_SetHUDSize(Dcu_HUDUnit_t dcu_id, Dcu_Size_t* pSize)
{
	Dcu_Err_t err = DCU_ERR_OK;

  if ((0 == ((pSize->mWidth) & ~DCU_HUD_SIZE_MASK)) || (0 == ((pSize->mHeight) & ~DCU_HUD_SIZE_MASK)))
	{
    REG_WRITE32(DCU_HUD_WIDTH_ADDR32(dcu_id), (pSize->mWidth) & DCU_HUD_SIZE_MASK);
    REG_WRITE32(DCU_HUD_HEIGHT_ADDR32(dcu_id), (pSize->mHeight) & DCU_HUD_SIZE_MASK);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

void DCU_SetHUDLineDescrAddress(Dcu_HUDUnit_t dcu_id, uint32_t addr)
{
	REG_WRITE32(DCU_WARP_DESC_ADDR_ADDR32(dcu_id), addr);
}

Dcu_Err_t DCU_GetHUDLineDescrAddress(Dcu_HUDUnit_t dcu_id, uint32_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
		*pValue = REG_READ32(DCU_WARP_DESC_ADDR_ADDR32(dcu_id));
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

void DCU_SetHUDLineDescrSize(Dcu_HUDUnit_t dcu_id, uint32_t TableSize)
{
	REG_WRITE32(DCU_WARP_DESC_TBSZ_ADDR32(dcu_id), TableSize);
}

Dcu_Err_t DCU_GetHUDLineDescrSize(Dcu_HUDUnit_t dcu_id, uint32_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
		*pValue = REG_READ32(DCU_WARP_DESC_TBSZ_ADDR32(dcu_id));
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

void DCU_SetHUDLBLinesNum(Dcu_HUDUnit_t dcu_id, uint8_t NumLines)
{
  REG_RMW32(DCU_WARP_CTRL_ADDR32(dcu_id), DCU_WARP_CTRL_LINESLB_MASK, (NumLines << DCU_WARP_CTRL_LINESLB_SHIFT));
}

Dcu_Err_t DCU_GetHUDLBLinesNum(Dcu_HUDUnit_t dcu_id, uint8_t* pNumLines)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pNumLines)
	{
    *pNumLines = ((REG_READ32(DCU_WARP_CTRL_ADDR32(dcu_id)) & DCU_WARP_CTRL_LINESLB_MASK) >> DCU_WARP_CTRL_LINESLB_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

void DCU_SetHUDTransferSize(Dcu_HUDUnit_t dcu_id, uint8_t NumBytes)
{
  REG_RMW32(DCU_WARP_CTRL_ADDR32(dcu_id), DCU_WARP_CTRL_AXIXFRS_MASK, (NumBytes << DCU_WARP_CTRL_AXIXFRS_SHIFT));
}

Dcu_Err_t DCU_GetHUDTransferSize(Dcu_HUDUnit_t dcu_id, uint8_t* pNumBytes)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pNumBytes)
	{
    *pNumBytes = ((REG_READ32(DCU_WARP_CTRL_ADDR32(dcu_id)) & DCU_WARP_CTRL_AXIXFRS_MASK) >> DCU_WARP_CTRL_AXIXFRS_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_GetHUDXoverflowPos(Dcu_HUDUnit_t dcu_id, Dcu_HUDErrPos_t* pXErrPixel)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pXErrPixel)
	{
    pXErrPixel->WARPPixelNum = ((REG_READ32(DCU_WARP_XOVR_STAT_ADDR32(dcu_id)) & DCU_WARP_OVR_PIXEL_MASK) >> DCU_WARP_OVR_PIXEL_SHIFT);
    pXErrPixel->WARPLineNum = ((REG_READ32(DCU_WARP_XOVR_STAT_ADDR32(dcu_id)) & DCU_WARP_OVR_LINE_MASK) >> DCU_WARP_OVR_LINE_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_GetHUDYoverflowPos(Dcu_HUDUnit_t dcu_id, Dcu_HUDErrPos_t* pYErrPixel)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pYErrPixel)
	{
    pYErrPixel->WARPPixelNum = ((REG_READ32(DCU_WARP_YOVR_STAT_ADDR32(dcu_id)) & DCU_WARP_OVR_PIXEL_MASK) >> DCU_WARP_OVR_PIXEL_SHIFT);
    pYErrPixel->WARPLineNum = ((REG_READ32(DCU_WARP_YOVR_STAT_ADDR32(dcu_id)) & DCU_WARP_OVR_LINE_MASK) >> DCU_WARP_OVR_LINE_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

#if (1 == DCU_IRQ_SUPPORT)
Dcu_Err_t DCU_EnableHUDIrq(Dcu_HUDUnit_t dcu_id, uint32_t int_mask)
{
	Dcu_Err_t err = DCU_ERR_OK;

  if ((0 == (int_mask & (DCU_WARP_IRQCTRL_LDDONE_MASK | DCU_WARP_IRQCTRL_YOVFLW_MASK | DCU_WARP_IRQCTRL_XOVFLW_MASK))) || (0 == int_mask))
	{
		err = DCU_ERR_MASK;
	}
  else if (DCU_WARP_IRQCTRL_XOVFLW_MASK == (int_mask & DCU_WARP_IRQCTRL_XOVFLW_MASK))
	{
		if (NULL_PTR != gpCallbackWARPXOVFLW[dcu_id])
		{
      REG_BIT_SET32(DCU_WARP_IRQ_CTRL_ADDR32(dcu_id), DCU_WARP_IRQCTRL_XOVFLW_MASK);
		}
		else
		{
			err = DCU_ERR_NOCALLBACK;
		}
	}
  else if (DCU_WARP_IRQCTRL_YOVFLW_MASK == (int_mask & DCU_WARP_IRQCTRL_YOVFLW_MASK))
	{
		if (NULL_PTR != gpCallbackWARPYOVFLW[dcu_id])
		{
      REG_BIT_SET32(DCU_WARP_IRQ_CTRL_ADDR32(dcu_id), DCU_WARP_IRQCTRL_YOVFLW_MASK);
		}
		else
		{
			err = DCU_ERR_NOCALLBACK;
		}
	}
	else
	{
		if (NULL_PTR != gpCallbackWARPLDDONE[dcu_id])
		{
      REG_BIT_SET32(DCU_WARP_IRQ_CTRL_ADDR32(dcu_id), DCU_WARP_IRQCTRL_LDDONE_MASK);
		}
		else
		{
			err = DCU_ERR_NOCALLBACK;
		}
	}
	return(err);
}

Dcu_Err_t DCU_DisableHUDIrq(Dcu_HUDUnit_t dcu_id, uint32_t int_mask)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t mask;

  mask = int_mask & (DCU_WARP_IRQCTRL_LDDONE_MASK | DCU_WARP_IRQCTRL_YOVFLW_MASK | DCU_WARP_IRQCTRL_XOVFLW_MASK);
  if ((0 == mask) || (0 == int_mask))
	{
		err = DCU_ERR_MASK;
	}
	else
	{
		REG_BIT_CLEAR32(DCU_WARP_IRQ_CTRL_ADDR32(dcu_id), mask);
	}
	return(err);
}

/** 
* @brief	 It registers the WARP Line Descriptor Done callback.
* @details This function registers the gpCallbackLDDone callback function
*	if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the callback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.  
*/
Dcu_Err_t DCU_RegisterCallbackLDDONE(Dcu_HUDUnit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackWARPLDDONE[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/** 
* @brief	 It registers the WARP X Overflow callback.
* @details This function registers the gpCallbackWARPXOVFLW callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the callback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.  
*/
Dcu_Err_t DCU_RegisterCallbackXOVFLW(Dcu_HUDUnit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackWARPXOVFLW[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}

/** 
* @brief	 It registers the WARP Y Overflow callback.
* @details This function registers the gpCallbackWARPYOVFLW callback function
*		if the pointer is not NULL.
*
* @param[in]	dcu_id		selects the DCU unit to be accessed.
*		aCallback	the callback function.
*
* @return   error code  - DCU_ERR_NOCALLBACK for NULL pointer to the callback.  
*/
Dcu_Err_t DCU_RegisterCallbackYOVFLW (Dcu_HUDUnit_t dcu_id, Dcu_Callback_t aCallback)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != aCallback)
	{
		gpCallbackWARPYOVFLW[dcu_id] = aCallback;
	}
	else
	{
		err = DCU_ERR_NOCALLBACK;
	}
	return(err);
}
#endif /* DCU_IRQ_SUPPORT */

#endif /*(DCU_HUD_FUNCTIONALITY)*/

#if (1 == DCU_WRITEBACK_FUNCTIONALITY)
Dcu_Err_t DCU_SetWriteBackConfig(Dcu_Unit_t dcu_id, Dcu_WriteBack_t* pWBConfigData)
{
	Dcu_Err_t err = DCU_ERR_OK;
	uint32_t reg_temp;

	if (NULL_PTR != pWBConfigData)
	{
    if ((0 == (pWBConfigData->WBAddress & ~DCU_WRITEBACK_ADDR_MASK)) &&
       (0 == (pWBConfigData->WBAlphaVal & ~(DCU_WRITEBACK_CTRL_WALPHA_MASK >> DCU_WRITEBACK_CTRL_WALPHA_SHIFT))))
    {
      REG_WRITE32(DCU_WRITEBACK_ADDR_ADDR32(dcu_id), pWBConfigData->WBAddress);

      reg_temp = (pWBConfigData->WBAlphaVal << DCU_WRITEBACK_CTRL_WALPHA_SHIFT);
      reg_temp|= (pWBConfigData->WBType << DCU_WRITEBACK_CTRL_TYPE_SHIFT);
      reg_temp|= (pWBConfigData->WBMode << DCU_WRITEBACK_CTRL_MODE_SHIFT);
      REG_WRITE32(DCU_WRITEBACK_CTRL_ADDR32(dcu_id), reg_temp);
		}
		else
		{
			err = DCU_ERR_RANGE;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_SetWriteBackBuffAddr(Dcu_Unit_t dcu_id, uint32_t addr)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (0 == (addr & ~DCU_WRITEBACK_ADDR_MASK))
	{
		REG_WRITE32(DCU_WRITEBACK_ADDR_ADDR32(dcu_id), addr);
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

Dcu_Err_t DCU_SetWriteBackAlphaVal(Dcu_Unit_t dcu_id, uint8_t value)
{
	Dcu_Err_t err = DCU_ERR_OK;

  if (0 == (value & ~(DCU_WRITEBACK_CTRL_WALPHA_MASK >> DCU_WRITEBACK_CTRL_WALPHA_SHIFT)))
	{
    REG_RMW32(DCU_WRITEBACK_CTRL_ADDR32(dcu_id), DCU_WRITEBACK_CTRL_WALPHA_MASK, (value << DCU_WRITEBACK_CTRL_WALPHA_SHIFT));
	}
	else
	{
		err = DCU_ERR_RANGE;
	}
	return(err);
}

void DCU_SetWriteBackPixelSize(Dcu_Unit_t dcu_id, Dcu_WBtype_t pSize)
{
  REG_RMW32(DCU_WRITEBACK_CTRL_ADDR32(dcu_id), DCU_WRITEBACK_CTRL_TYPE_MASK, pSize << DCU_WRITEBACK_CTRL_TYPE_SHIFT);
}

void DCU_SetWriteBackMode(Dcu_Unit_t dcu_id, Dcu_WBmode_t wb_mode)
{
  REG_RMW32(DCU_WRITEBACK_CTRL_ADDR32(dcu_id), DCU_WRITEBACK_CTRL_MODE_MASK, (uint32_t)wb_mode << DCU_WRITEBACK_CTRL_MODE_SHIFT);
}

Dcu_Err_t DCU_GetWriteBackBuffAddr(Dcu_Unit_t dcu_id, uint32_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue = ((REG_READ32(DCU_WRITEBACK_ADDR_ADDR32(dcu_id)) & DCU_WRITEBACK_ADDR_MASK) >> DCU_WRITEBACK_ADDR_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_GetWriteBackPixelSize(Dcu_Unit_t dcu_id, Dcu_WBtype_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue = ((REG_READ32(DCU_WRITEBACK_CTRL_ADDR32(dcu_id)) & DCU_WRITEBACK_CTRL_TYPE_MASK) >> DCU_WRITEBACK_CTRL_TYPE_SHIFT);
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_GetWriteBackAlphaVal(Dcu_Unit_t dcu_id, uint8_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue = (REG_READ32(DCU_WRITEBACK_CTRL_ADDR32(dcu_id)) & DCU_WRITEBACK_CTRL_WALPHA_MASK) >> DCU_WRITEBACK_CTRL_WALPHA_SHIFT;
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_GetWriteBackMode(Dcu_Unit_t dcu_id, Dcu_WBmode_t* pValue)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    *pValue = REG_READ32(DCU_WRITEBACK_CTRL_ADDR32(dcu_id)) & DCU_WRITEBACK_CTRL_MODE_MASK;
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_WriteBackErrCheck(Dcu_Unit_t dcu_id, uint32_t* pValue, uint32_t uiMask)
{
	Dcu_Err_t err = DCU_ERR_OK;

	if (NULL_PTR != pValue)
	{
    if ((0 != (uiMask & (DCU_WRITEBACK_STAT_ALLERR_MASK)))||(0 == (uiMask & ~(DCU_WRITEBACK_STAT_ALLERR_MASK))))
		{
      *pValue = REG_READ32(DCU_WRITEBACK_STAT_ADDR32(dcu_id)) & (DCU_WRITEBACK_STAT_ALLERR_MASK);
			if (0 != *pValue)
			{
				err = DCU_ERR_WRITEBACK;
			}
		}
		else
		{
			err = DCU_ERR_MASK;
		}
	}
	else
	{
		err = DCU_ERR_NULL_PTR;
	}
	return(err);
}

Dcu_Err_t DCU_WriteBackReadyCheck(Dcu_Unit_t dcu_id)
{
	uint32_t readyflag;
	Dcu_Err_t err = DCU_ERR_OK;

  readyflag = (REG_READ32(DCU_WRITEBACK_STAT_ADDR32(dcu_id)) & DCU_WRITEBACK_STAT_WDONE_MASK);
	if (0 != readyflag)
	{
		err = DCU_ERR_OK;
    REG_BIT_SET32(DCU_WRITEBACK_STAT_ADDR32(dcu_id), DCU_WRITEBACK_STAT_WDONE_MASK);
	}
	else
	{
		err = DCU_ERR_WRITEBACK;
	}

	return(err);
}
#endif /* DCU_WRITEBACK_FUNCTIONALITY */
