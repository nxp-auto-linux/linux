/**
*	 @file		dcu.h
*
*	 @brief	 The file is the 2D-ACE header file.
*   @details It contains the prototypes of the function and typedefs of parameters.
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
Cristian Tomescu (B13031)     04/06/2014    ENGR00316549  Porting the 2D-ACE driver on HALO platform.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     04/07/2014    ENGR00321529  Adding the HUD support.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     28/07/2014    ENGR00324499  Implementing changes for frbuff interface.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     06/08/2014    ENGR00324522  Implementing the writeback support.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     20/08/2014    ENGR00327882  Porting the driver on the RAYLEIGH platform.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     02/09/2014    ENGR00334168  Adding the DCU_Disable function.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     31/10/2014    ENGR00337995  Improvement of some functions.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     24/11/2014    ENGR00340864  Change in div factor calculation.API also.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     07/01/2015    ENGR00344709  Changes about the errors interrupts.
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     23/03/2015    ENGR00351101  Integration of the driver in ASR OS. 
---------------------------   ----------    ------------  ------------------------------------------
Cristian Tomescu (B13031)     27/05/2015    ENGR00355821  Porting on Treerunner. 
==================================================================================================*/
/**
 * \file		dcu.h
 * \brief	 This is the DCU Driver Header File
 * \author
 * \author
 * \version 0.1
 * \date		<dd-month-20xx>
 * \note		<optional some notes>
 ****************************************************************************/
#ifndef DCU_H_
#define DCU_H_

#ifdef	__cplusplus
extern "C" {
#endif

#include "fsl_dcu_typedefs.h"
#include "fsl_dcu_cfg.h"

/*****************************************************************************
* local defines
*****************************************************************************/


/*****************************************************************************
* local types
*****************************************************************************/

typedef enum
{
	DCU_DISPLAY_LVDS=0,
	DCU_DISPLAY_HDMI=1,
	DCU_DISPLAY_TCON=2
}DCU_DISPLAY_TYPE;

typedef enum
{
	DCU_MODE_OFF=0,	 /**< DCU off (pixel clock active if enabled by I/O) */
	DCU_MODE_NORMAL,/**< Normal mode. Panel content controlled by layer */
	DCU_MODE_TEST,	/**< Test mode */
	DCU_MODE_COLBAR	 /**< Color Bar Generation */
}Dcu_Mode_t;

typedef enum
{
	DCU_DISABLE=0,
	DCU_ENABLE=1
}Dcu_Enable_t;

typedef enum
{
	DCU_ALPHAKEY_OFF=0,
	DCU_ALPHAKEY_CHROMASEL=1,
	DCU_ALPHAKEY_WHOLEFRAME=2,
	DCU_ALPHAKEY_OFF2=3,
}Dcu_AlphaKey_t;

typedef enum
{
	DCU_BPP_1=0,
	DCU_BPP_2,
	DCU_BPP_4,
	DCU_BPP_8,		/**< CLUT */
	DCU_BPP_16,		 /**< 565 */
	DCU_BPP_24,
	DCU_BPP_32,		 /**< ARGB8888 */
	DCU_BPP_TRANS_4,
	DCU_BPP_TRANS_8,
	DCU_BPP_LUM_OFFS_4,
	DCU_BPP_LUM_OFFS_8,
	DCU_BPP_16_ARGB1555,
	DCU_BPP_16_ARGB4444,
	DCU_BPP_16_APAL8,
	DCU_BPP_YCbCr422,
	DCU_BPP_32_ARGB8888
}Dcu_BPP_t;

typedef enum
{
	DCU_ERR_OK=0,		/** OK (no error) */
	DCU_ERR_RANGE,		/** value range violation */
	DCU_ERR_NULL_PTR,	/** null pointer set as parameter */
	DCU_ERR_NOCALLBACK,	/** no callback registered */
	DCU_ERR_MASK,		/** wrong mask */
	DCU_ERR_CALLTIME,	/** wrong call moment */
	DCU_ERR_RLE,		/** second RLE layer */
#ifdef DCU_WRITEBACK_FUNCTIONALITY
	DCU_ERR_WRITEBACK,	/** write back error */
#endif /* DCU_WRITEBACK_FUNCTIONALITY */
	DCU_ERR_TIMEOUT
}Dcu_Err_t;

typedef enum
{
	DCU_INACTIVE = 0x0,	/**< no interrupt */
	DCU_VSYNC,		/**< vertical sync interrupt */
	DCU_VBLANK,		/**< vertical blanking interrupt */
	DCU_LSBFVS,		/**< lines before vertical sync interrupt */
  DCU_PROG_END,               /**< programming of all buffered register is done */
  DCU_LYRCFG_DONE,            /**< layer transfer finished */
  DCU_DATA_DONE               /**< DMA transfer finished */
}Dcu_DisplayIStatus_t;

#if (1 == DCU_IRQ_SUPPORT)
typedef void (* Dcu_Callback_t)(void);
#endif

typedef struct
{
	uint16_t mDeltaX;	/**< delta X */
	uint16_t mDeltaY;	/**< delta Y */
	uint16_t mHorzBP;	/**< Horizontal back porch */
	uint16_t mHorzPW;	/**< Horizontal sync pulse */
	uint16_t mHorzFP;	/**< Horizontal front porch */
	uint16_t mVertBP;	/**< Vertical back porch */
	uint16_t mVertPW;	/**< Vertical sync pulse */
	uint16_t mVertFP;	/**< Vertical front porch */
  uint16_t mVertFq;       /**< Vertical frequency/Pixell frequency/Division factor */
	uint32_t mSyncPol;	/**< Sync signal polarity */
  uint16_t mDivFactor;    /**< division factor */
}Dcu_LCD_Para_t;

typedef enum
{
  DCU_FREQDIV_NORMAL,     /**< div calculation using frame rate(in frames/second) and pixell number */
  DCU_FREQDIV_HDMI,       /**< div calculation using pixell clock(in kHz)*/
  DCU_FREQDIV_LVDS        /**< no div calculation, it is used a constant value(uint8_t) */
}Dcu_LCD_Connection_t;

typedef struct
{
	uint16_t mWidth;	/**< image width->X */
	uint16_t mHeight;	/**< image height->Y */
}Dcu_Size_t;

typedef struct
{
	int16_t mX;		/**< left upper corner X pos */
	int16_t mY;		/**< left upper corner Y pos */
}Dcu_Position_t;

typedef struct
{
	uint32_t Red_Value;
	uint32_t Green_Value;
	uint32_t Blue_Value;
}Dcu_Colour_t;

typedef struct
{
	uint16_t	mWidth;		/** tile width->X */
	uint16_t	mHeight;	/** tile height->Y */
	uint32_t	pTileData;	/** tile image address */
	Dcu_Enable_t	TileEnable;	/** tile mode enable */
}Dcu_Tile_t;

typedef struct
{
	uint16_t	CursorWidth;	/** cursor width->X */
	uint16_t	CursorHeight;	/** cursor height->Y */
	uint16_t	CursorPosX;	/** cursor X position */
	uint16_t	CursorPosY;	/** cursor Y position */
	uint32_t	CursorColour;	/** cursor color */
	Dcu_Enable_t	CursorEnable;	/** cursor enable */
}Dcu_Cursor_t;

typedef struct
{
	uint8_t		CursorBlinkOn;	/** blinking ON period */
	uint8_t		CursorBlinkOff;	/** blinking OFF period */
	Dcu_Enable_t	CursorBlinkEn;	/** blinking enable */
}Dcu_CursorBlink_t;

typedef struct
{
	uint32_t	HW_Error1;
#if (64 == DCU_LAYERS_NUM_MAX)
	uint32_t	HW_Error2;
#endif
	uint32_t	HW_Error3;
}Dcu_HwErrRet_t;

typedef struct
{
	uint32_t*	pRedGamma;
	uint32_t*	pGreenGamma;
	uint32_t*	pBlueGamma;
}Dcu_Gamma_t;

typedef struct
{
	uint32_t	mCRCVal;	/**< CRC for pixells' value */
	uint32_t	mCRCPos;	/**< CRC for pixells' position */
}Dcu_Signature_t;

typedef struct
{
	uint16_t	LayerWidth;
	uint16_t	LayerHeight;
	uint16_t	LayerPosX;
	uint16_t	LayerPosY;
	uint32_t	LayerBuffAdd;
	Dcu_BPP_t	LayerColCode;
	Dcu_AlphaKey_t	LayerAlphaBlend;
	Dcu_Enable_t	LayerChromaKey;
	uint32_t	LayerTransp;
	uint32_t	LayerFGNDCol;
	uint32_t	LayerBGNDCol;
  uint16_t              LayerTileEn;
  uint16_t              LayerPreSkip;
  uint16_t              LayerPostSkip;
}Dcu_LayerParams_t;

typedef enum
{
	DCU_CRC_GLOBAL = 0x0,	/**< crc for the safety area */
	DCU_CRC_LAYER		/**< crc for the safety layers */
}Dcu_CRCmode_t;

typedef enum
{
	DCU_ASYNC_MODE = 0x0,	/**<	*/
	DCU_PERIODIC_MODE	/**<	*/
}Dcu_UpdateMode_t;

typedef enum
{
	DCU0 = 0x0,	/**< first 2D_ACE unit */
	DCU1		/**< second 2D_ACE unit */
}Dcu_Unit_t;


#if (8 == DCU_LAYERS_NUM_MAX)
typedef enum
{
  DCU_LAYER_0=0,
  DCU_LAYER_1,
  DCU_LAYER_2,
  DCU_LAYER_3,
  DCU_LAYER_4,
  DCU_LAYER_5,
  DCU_LAYER_6,
  DCU_LAYER_7
}Dcu_Layer_t;
#endif
#if (16 == DCU_LAYERS_NUM_MAX)
typedef enum
{
	DCU_LAYER_0=0,
	DCU_LAYER_1,
	DCU_LAYER_2,
	DCU_LAYER_3,
	DCU_LAYER_4,
	DCU_LAYER_5,
	DCU_LAYER_6,
	DCU_LAYER_7,
	DCU_LAYER_8,
	DCU_LAYER_9,
	DCU_LAYER_10,
	DCU_LAYER_11,
	DCU_LAYER_12,
	DCU_LAYER_13,
	DCU_LAYER_14,
	DCU_LAYER_15
}Dcu_Layer_t;
#endif
#if (32 == DCU_LAYERS_NUM_MAX)
typedef enum
{
	DCU_LAYER_0=0,
	DCU_LAYER_1,
	DCU_LAYER_2,
	DCU_LAYER_3,
	DCU_LAYER_4,
	DCU_LAYER_5,
	DCU_LAYER_6,
	DCU_LAYER_7,
	DCU_LAYER_8,
	DCU_LAYER_9,
	DCU_LAYER_10,
	DCU_LAYER_11,
	DCU_LAYER_12,
	DCU_LAYER_13,
	DCU_LAYER_14,
	DCU_LAYER_15,
	DCU_LAYER_16,
	DCU_LAYER_17,
	DCU_LAYER_18,
	DCU_LAYER_19,
	DCU_LAYER_20,
	DCU_LAYER_21,
	DCU_LAYER_22,
	DCU_LAYER_23,
	DCU_LAYER_24,
	DCU_LAYER_25,
	DCU_LAYER_26,
	DCU_LAYER_27,
	DCU_LAYER_28,
	DCU_LAYER_29,
	DCU_LAYER_30,
	DCU_LAYER_31
}Dcu_Layer_t;
#endif

#if (64 == DCU_LAYERS_NUM_MAX)
typedef enum
{
	DCU_LAYER_0=0,
	DCU_LAYER_1,
	DCU_LAYER_2,
	DCU_LAYER_3,
	DCU_LAYER_4,
	DCU_LAYER_5,
	DCU_LAYER_6,
	DCU_LAYER_7,
	DCU_LAYER_8,
	DCU_LAYER_9,
	DCU_LAYER_10,
	DCU_LAYER_11,
	DCU_LAYER_12,
	DCU_LAYER_13,
	DCU_LAYER_14,
	DCU_LAYER_15,
	DCU_LAYER_16,
	DCU_LAYER_17,
	DCU_LAYER_18,
	DCU_LAYER_19,
	DCU_LAYER_20,
	DCU_LAYER_21,
	DCU_LAYER_22,
	DCU_LAYER_23,
	DCU_LAYER_24,
	DCU_LAYER_25,
	DCU_LAYER_26,
	DCU_LAYER_27,
	DCU_LAYER_28,
	DCU_LAYER_29,
	DCU_LAYER_30,
	DCU_LAYER_31,
	DCU_LAYER_32,
	DCU_LAYER_33,
	DCU_LAYER_34,
	DCU_LAYER_35,
	DCU_LAYER_36,
	DCU_LAYER_37,
	DCU_LAYER_38,
	DCU_LAYER_39,
	DCU_LAYER_40,
	DCU_LAYER_41,
	DCU_LAYER_42,
	DCU_LAYER_43,
	DCU_LAYER_44,
	DCU_LAYER_45,
	DCU_LAYER_46,
	DCU_LAYER_47,
	DCU_LAYER_48,
	DCU_LAYER_49,
	DCU_LAYER_50,
	DCU_LAYER_51,
	DCU_LAYER_52,
	DCU_LAYER_53,
	DCU_LAYER_54,
	DCU_LAYER_55,
	DCU_LAYER_56,
	DCU_LAYER_57,
	DCU_LAYER_58,
	DCU_LAYER_59,
	DCU_LAYER_60,
	DCU_LAYER_61,
	DCU_LAYER_62,
	DCU_LAYER_63
}Dcu_Layer_t;
#endif

typedef enum
{
	DCU_SAFE_LAYER0=0,
	DCU_SAFE_LAYER1
}Dcu_SafeLayer_t;

#if (1 == DCU_TILE_FUNCTIONALITY)
#if (TILE_VAR_SIZE == DCU_TILE_MODE)
typedef enum
{
	DCU_TILE_LAYER0=0,
	DCU_TILE_LAYER1,
	DCU_TILE_LAYER2,
	DCU_TILE_LAYER3,
#if (32 == DCU_LAYERS_NUM_MAX)
	DCU_TILE_LAYER28 =28,
	DCU_TILE_LAYER29,
	DCU_TILE_LAYER30,
	DCU_TILE_LAYER31
#endif
#if (64 == DCU_LAYERS_NUM_MAX)
	DCU_TILE_LAYER4,
	DCU_TILE_LAYER5,
	DCU_TILE_LAYER6,
	DCU_TILE_LAYER7,
	DCU_TILE_LAYER56 =56,
	DCU_TILE_LAYER57,
	DCU_TILE_LAYER58,
	DCU_TILE_LAYER59,
	DCU_TILE_LAYER60,
	DCU_TILE_LAYER61,
	DCU_TILE_LAYER62,
	DCU_TILE_LAYER63
#endif
}Dcu_TileLayer_t;
#endif /* TILE_VAR_SIZE == DCU_TILE_MODE */
#if (TILE_FIX_SIZE == DCU_TILE_MODE)
typedef enum
{
  DCU_TILE_LAYER0=0,
  DCU_TILE_LAYER1,
  DCU_TILE_LAYER2,
  DCU_TILE_LAYER3,
  DCU_TILE_LAYER4,
  DCU_TILE_LAYER5,
  DCU_TILE_LAYER6,
  DCU_TILE_LAYER7
}Dcu_TileLayer_t;
#endif /* TILE_FIX_SIZE == DCU_TILE_MODE */
#endif /* DCU_TILE_FUNCTIONALITY */
#if (1 == DCU_HUD_FUNCTIONALITY)
typedef struct
{
	uint16_t	HUDWidth;	/** HUD width->X */
	uint16_t	HUDHeight;	/** HUD height->Y */
	uint32_t	HUDLDAddress;	/** line descriptor table address */
  uint32_t      HUDLDSize;        /** line descriptor size = number of bytes */
	uint8_t		HUDLBSize;	/** line buffer size = number of lines */
	uint8_t		HUDAXISize;	/** AXI transfer data chunk */
	Dcu_Enable_t	HUDEnable;	/** HUD enable */
}Dcu_Warp_t;

typedef struct
{
	uint16_t	WARPPixelNum;	/** WARP error pixell number*/
	uint16_t	WARPLineNum;	/** WARP error line number */
}Dcu_HUDErrPos_t;
#endif /*(DCU_HUD_FUNCTIONALITY)*/
#ifdef DCU_WRITEBACK_FUNCTIONALITY
typedef enum
{
	DCU_WB_WBOFF=0,
	DCU_WB_DISPOFF,
	DCU_WB_ALL_ON
}Dcu_WBmode_t;

typedef enum
{
	DCU_WB_RGB888=0,
	DCU_WB_ARGB8888
}Dcu_WBtype_t;

typedef struct
{
	uint32_t	WBAddress;	/** buffer address */
	Dcu_WBmode_t	WBMode;		/** writeback mode */
	Dcu_WBtype_t	WBType;		/** writeback pixel size */
	uint8_t			 WBAlphaVal;	/** writeback alpha value */
}Dcu_WriteBack_t;
#endif /* DCU_WRITEBACK_FUNCTIONALITY */
/*****************************************************************************
* publics functions (prototypes)
*****************************************************************************/
Dcu_Err_t DCU_Init(Dcu_Unit_t dcu_id, uint32_t aDCUclk, const Dcu_LCD_Para_t* apcLCD, Dcu_LCD_Connection_t aDivCalcType);
void DCU_Disable(Dcu_Unit_t dcu_id);
void DCU_SwReset(Dcu_Unit_t dcu_id);
Dcu_Err_t DCU_CLUTSet(Dcu_Unit_t dcu_id, uint16_t aIndex, uint32_t aValue);
Dcu_Err_t DCU_CLUTGet(Dcu_Unit_t dcu_id, uint16_t aIndex, uint32_t* apValue);
Dcu_Err_t DCU_CLUTLoad(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint16_t aLUTOffset, uint16_t aLUTSize, const uint32_t* apValue);
Dcu_Err_t DCU_CursorLoad(Dcu_Unit_t dcu_id, uint16_t aCount, const uint32_t* apcValues);
#if (1 == DCU_IRQ_SUPPORT)
void DCU0_Timing_Isr(void);
void DCU1_Timing_Isr(void);
#endif /* DCU_IRQ_SUPPORT */

void DCU_SetUpdateMode(Dcu_Unit_t dcu_id, Dcu_UpdateMode_t mode);
void DCU_SetLinesBeforeVSYNC(Dcu_Unit_t dcu_id, uint8_t uLines);
Dcu_Err_t DCU_BGNDColorSet(Dcu_Unit_t dcu_id, Dcu_Colour_t* pBGNDColour);
#if ((IPV_DCU_VYBRID == IPV_DCU) && (1 == DCU_RLE_FUNCTIONALITY))
	Dcu_Err_t DCU_SetRLECompSize(Dcu_Unit_t dcu_id, uint32_t aValue);
#endif
Dcu_Err_t DCU_SetCursor(Dcu_Unit_t dcu_id, Dcu_Cursor_t *pCursorSettings);
Dcu_Err_t DCU_SetCursorPosition(Dcu_Unit_t dcu_id, Dcu_Position_t *pPosition);
Dcu_Err_t DCU_SetCursorColour(Dcu_Unit_t dcu_id, uint32_t uColour);
void DCU_CursorEnable(Dcu_Unit_t dcu_id);
void DCU_CursorDisable(Dcu_Unit_t dcu_id);
Dcu_Err_t DCU_SetCursorBlinking(Dcu_Unit_t dcu_id, Dcu_CursorBlink_t* pBlinkingDuration);
void DCU_CursorBlinkingEnable(Dcu_Unit_t dcu_id);
void DCU_CursorBlinkingDisable(Dcu_Unit_t dcu_id);
void DCU_GammaCorrectionEnable(Dcu_Unit_t dcu_id);
void DCU_GammaCorrectionDisable(Dcu_Unit_t dcu_id);
Dcu_Err_t DCU_GammaLoad(Dcu_Unit_t dcu_id, Dcu_Gamma_t *pGammaCorrectionData);
#if (1 == DCU_IRQ_SUPPORT)
Dcu_Err_t DCU_GetDisplayIStatus(Dcu_Unit_t dcu_id, Dcu_DisplayIStatus_t* pValue);
#endif /* DCU_IRQ_SUPPORT */
Dcu_Err_t DCU_GetHwErrStatus(Dcu_Unit_t dcu_id, Dcu_HwErrRet_t* pValue);


Dcu_Err_t DCU_IsCursorEnabled(Dcu_Unit_t dcu_id, Dcu_Enable_t* pValue);
Dcu_Err_t DCU_IsCursorBlinkingEnabled(Dcu_Unit_t dcu_id, Dcu_Enable_t* pValue);
Dcu_Err_t DCU_GetUpdateMode(Dcu_Unit_t dcu_id, Dcu_UpdateMode_t* pValue);
#if ((IPV_DCU_VYBRID == IPV_DCU) && (1 == DCU_RLE_FUNCTIONALITY))
	Dcu_Err_t DCU_GetRLELayer(Dcu_Unit_t dcu_id, Dcu_Layer_t* pValue);
#endif
Dcu_Err_t DCU_IsGammaCorrectionEnabled(Dcu_Unit_t dcu_id, Dcu_Enable_t* pValue);

/***************	 LAYER SETTINGS SECTION	 	***************/
Dcu_Err_t DCU_SetLayerConfig(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_LayerParams_t* pLayer_Params);
Dcu_Err_t DCU_SetLayerSize(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Size_t* pSize);
Dcu_Err_t DCU_SetLayerPosition(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Position_t* pPos);
Dcu_Err_t DCU_SetLayerLUTOffset(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint16_t aValue);
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
Dcu_Err_t DCU_LayerEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
Dcu_Err_t DCU_LayerDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
Dcu_Err_t DCU_SetLayerBuffAddr(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t addr);
Dcu_Err_t DCU_SetLayerBPP(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_BPP_t col_format);
Dcu_Err_t DCU_SetLayerAlphaMode(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id, Dcu_AlphaKey_t alpha_mode);
Dcu_Err_t DCU_SetLayerAlphaVal(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint8_t value);
Dcu_Err_t DCU_LayerChromaEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
Dcu_Err_t DCU_LayerChromaDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
#else
void DCU_LayerEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
void DCU_LayerDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
void DCU_SetLayerBuffAddr(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t addr);
void DCU_SetLayerBPP(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_BPP_t col_format);
void DCU_SetLayerAlphaMode(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id, Dcu_AlphaKey_t alpha_mode);
void DCU_SetLayerAlphaVal(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id, uint8_t value);
void DCU_LayerChromaEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
void DCU_LayerChromaDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
Dcu_Err_t DCU_SetLayerChroma(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Colour_t* pLyrChromaMax, Dcu_Colour_t* pLyrChromaMin);
Dcu_Err_t DCU_SetLayerForeground(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id, uint32_t value);
Dcu_Err_t DCU_SetLayerBackground(Dcu_Unit_t dcu_id,Dcu_Layer_t layer_id, uint32_t value);
Dcu_Err_t DCU_LayerRLEEnable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
#if ((1 == DCU_IRQ_SUPPORT) && (1 == DCU_IRQ_STATEMACHINE))
 #if (1 == DCU_RLE_FUNCTIONALITY)
Dcu_Err_t DCU_LayerRLEDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
 #endif */(1 == DCU_RLE_FUNCTIONALITY)*/
 #if (1 == DCU_TILE_FUNCTIONALITY)
Dcu_Err_t DCU_LayerTileEnable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id);
Dcu_Err_t DCU_LayerTileDisable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id);
 #endif /*(DCU_TILE_FUNCTIONALITY)*/
#else
 #if (1 == DCU_RLE_FUNCTIONALITY)
void DCU_LayerRLEDisable(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id);
 #endif /*(1 == DCU_RLE_FUNCTIONALITY)*/
 #if (1 == DCU_TILE_FUNCTIONALITY)
void DCU_LayerTileEnable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id);
void DCU_LayerTileDisable(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id);
 #endif /*(DCU_TILE_FUNCTIONALITY)*/
#endif /* DCU_IRQ_SUPPORT && DCU_IRQ_STATEMACHINE */
#if (1 == DCU_TILE_FUNCTIONALITY)
Dcu_Err_t DCU_SetLayerTile(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id, Dcu_Tile_t* pTileData);
#endif /*(DCU_TILE_FUNCTIONALITY)*/

Dcu_Err_t DCU_IsLayerEnabled(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Enable_t* pValue);
Dcu_Err_t DCU_GetLayerSize(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Size_t* pValue);
Dcu_Err_t DCU_GetLayerPosition(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Position_t* pPos);
Dcu_Err_t DCU_GetLayerBuffAddr(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t* pValue);
Dcu_Err_t DCU_GetLayerBPP(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_BPP_t* pValue);
Dcu_Err_t DCU_GetLayerAlphaMode(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_AlphaKey_t* pValue);
Dcu_Err_t DCU_GetLayerAlphaVal(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint8_t* pValue);
Dcu_Err_t DCU_GetLayerForeground(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t* pValue);
Dcu_Err_t DCU_GetLayerBackground(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, uint32_t* pValue);
Dcu_Err_t DCU_IsLayerChromaEnabled(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Enable_t* pValue);
Dcu_Err_t DCU_GetLayerChromaMax(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Colour_t* pLyrChromaMax);
Dcu_Err_t DCU_GetLayerChromaMin(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Colour_t* pLyrChromaMin);
#if (1 == DCU_RLE_FUNCTIONALITY)
Dcu_Err_t DCU_IsLayerRLEEnabled(Dcu_Unit_t dcu_id, Dcu_Layer_t layer_id, Dcu_Enable_t* pValue);
#endif /*(1 == DCU_RLE_FUNCTIONALITY)*/
#if (1 == DCU_TILE_FUNCTIONALITY)
Dcu_Err_t DCU_IsLayerTileEnabled(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id, Dcu_Enable_t* pValue);
#if (TILE_VAR_SIZE == DCU_TILE_MODE)
Dcu_Err_t DCU_GetLayerTile(Dcu_Unit_t dcu_id, Dcu_TileLayer_t layer_id, Dcu_Tile_t* pTileConfig);
#endif /*(TILE_VAR_SIZE == DCU_TILE_MODE)*/
#endif /*(DCU_TILE_FUNCTIONALITY)*/

/***************	SAFETY SECTION		***************/
#if (1 == DCU_SAFETY_FUNCTIONALITY)
Dcu_Err_t DCU_SetSignedArea(Dcu_Unit_t dcu_id, Dcu_Position_t* pSAPosition, Dcu_Size_t* pSASize);
void DCU_SignatureEnable(Dcu_Unit_t dcu_id, Dcu_CRCmode_t xCalcMode);
void DCU_SignatureDisable(Dcu_Unit_t dcu_id);
Dcu_Err_t DCU_GetSignature(Dcu_Unit_t dcu_id, Dcu_Signature_t* pValue);
void DCU_LayerSafeEnable(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer);
void DCU_LayerSafeDisable(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer);
void DCU_LayerRegsLock(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer);
void DCU_LayerRegsUnlock(Dcu_Unit_t dcu_id, Dcu_SafeLayer_t s_layer);
void DCU_DisplayInterfaceLock(Dcu_Unit_t dcu_id);
void DCU_DisplayInterfaceUnlock(Dcu_Unit_t dcu_id);
#endif /* DCU_SAFETY_FUNCTIONALITY */

/***************	INTERRUPTS MANAGEMENT	***************/
#if (1 == DCU_IRQ_SUPPORT)
Dcu_Err_t DCU_EnableDisplayTimingIrq(Dcu_Unit_t dcu_id, uint32_t int_mask);
Dcu_Err_t DCU_DisableDisplayTimingIrq(Dcu_Unit_t dcu_id, uint32_t int_mask);
Dcu_Err_t DCU_RegisterCallbackVBLANK(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackLSBFVS(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackVSYNC(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackProgDone(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackDataTransferDone(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackLyrCfgTransferDone(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);

#if (1 == DCU_SAFETY_FUNCTIONALITY)
Dcu_Err_t DCU_EnableSafetyIrq(Dcu_Unit_t dcu_id, uint32_t int_mask);
Dcu_Err_t DCU_DisableSafetyIrq(Dcu_Unit_t dcu_id, uint32_t int_mask);
Dcu_Err_t DCU_RegisterCallbackCRCReady(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackCRCOverflow(Dcu_Unit_t dcu_id, Dcu_Callback_t aCallback);
#endif /* DCU_SAFETY_FUNCTIONALITY */

#if (1 == DCU_HUD_FUNCTIONALITY)
Dcu_Err_t DCU_EnableHUDIrq(Dcu_HUDUnit_t dcu_id, uint32_t int_mask);
Dcu_Err_t DCU_DisableHUDIrq(Dcu_HUDUnit_t dcu_id, uint32_t int_mask);
Dcu_Err_t DCU_RegisterCallbackLDDONE(Dcu_HUDUnit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackXOVFLW(Dcu_HUDUnit_t dcu_id, Dcu_Callback_t aCallback);
Dcu_Err_t DCU_RegisterCallbackYOVFLW (Dcu_HUDUnit_t dcu_id, Dcu_Callback_t aCallback);
#endif /* DCU_HUD_FUNCTIONALITY */
#endif /* DCU_IRQ_SUPPORT */

/***************	HUD SUPPORT	***************/
#if (1 == DCU_HUD_FUNCTIONALITY)
Dcu_Err_t DCU_SetHUDConfig(Dcu_HUDUnit_t dcu_id, Dcu_Warp_t* pHUDConfig);
void DCU_HUDEnable(Dcu_HUDUnit_t dcu_id); 
void DCU_HUDDisable(Dcu_HUDUnit_t dcu_id);
Dcu_Err_t DCU_SetHUDSize(Dcu_HUDUnit_t dcu_id, Dcu_Size_t* pSize);
void DCU_SetHUDLineDescrAddress(Dcu_HUDUnit_t dcu_id, uint32_t addr);
void DCU_SetHUDLineDescrSize(Dcu_HUDUnit_t dcu_id, uint32_t TableSize);
void DCU_SetHUDLBLinesNum(Dcu_HUDUnit_t dcu_id, uint8_t NumLines);
void DCU_SetHUDTransferSize(Dcu_HUDUnit_t dcu_id, uint8_t NumBytes);
Dcu_Err_t DCU_GetHUDLineDescrAddress(Dcu_HUDUnit_t dcu_id, uint32_t* pValue);
Dcu_Err_t DCU_GetHUDLineDescrSize(Dcu_HUDUnit_t dcu_id, uint32_t* pValue);
Dcu_Err_t DCU_GetHUDLBLinesNum(Dcu_HUDUnit_t dcu_id, uint8_t* pNumLines);
Dcu_Err_t DCU_GetHUDTransferSize(Dcu_HUDUnit_t dcu_id, uint8_t* pNumBytes);
Dcu_Err_t DCU_GetHUDXoverflowPos(Dcu_HUDUnit_t dcu_id, Dcu_HUDErrPos_t* pXErrPixel);
Dcu_Err_t DCU_GetHUDYoverflowPos (Dcu_HUDUnit_t dcu_id, Dcu_HUDErrPos_t* pYErrPixel);
#endif /* DCU_HUD_FUNCTIONALITY */
#if (1 == DCU_WRITEBACK_FUNCTIONALITY)
Dcu_Err_t DCU_SetWriteBackConfig(Dcu_Unit_t dcu_id, Dcu_WriteBack_t* pWBConfigData);
Dcu_Err_t DCU_SetWriteBackBuffAddr(Dcu_Unit_t dcu_id, uint32_t addr);
Dcu_Err_t DCU_SetWriteBackAlphaVal(Dcu_Unit_t dcu_id, uint8_t value);
void DCU_SetWriteBackPixelSize(Dcu_Unit_t dcu_id, Dcu_WBtype_t pSize);
void DCU_SetWriteBackMode(Dcu_Unit_t dcu_id, Dcu_WBmode_t wb_mode);
Dcu_Err_t DCU_GetWriteBackBuffAddr(Dcu_Unit_t dcu_id, uint32_t* pValue);
Dcu_Err_t DCU_GetWriteBackPixelSize(Dcu_Unit_t dcu_id, Dcu_WBtype_t* pValue);
Dcu_Err_t DCU_GetWriteBackAlphaVal(Dcu_Unit_t dcu_id, uint8_t* pValue);
Dcu_Err_t DCU_GetWriteBackMode(Dcu_Unit_t dcu_id, Dcu_WBmode_t* pValue);
Dcu_Err_t DCU_WriteBackErrCheck(Dcu_Unit_t dcu_id, uint32_t* pValue, uint32_t uiMask);
Dcu_Err_t DCU_WriteBackReadyCheck(Dcu_Unit_t dcu_id);
#endif /* DCU_WRITEBACK_FUNCTIONALITY */
#ifdef	__cplusplus
}
#endif

#endif /* DCU_H_ */
