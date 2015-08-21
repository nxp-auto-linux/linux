#ifndef FSL_DCU_IOCTL_
#define FSL_DCU_IOCTL_

/************************************************
	VARIOUS TYPES
*************************************************/
typedef struct
{
  int16_t mX;
  int16_t mY;
} IOCTL_DCU_POSITION;

typedef enum
{
	IOCTL_DCU_ALPHAKEY_OFF=0,
	IOCTL_DCU_ALPHAKEY_CHROMASEL=1,
	IOCTL_DCU_ALPHAKEY_WHOLEFRAME=2,
	IOCTL_DCU_ALPHAKEY_OFF2=3,
} IOCTL_DCU_ALPHAKEY;

typedef enum
{
	IOCTL_DCU_CHROMA_OFF=0,
	IOCTL_DCU_CHROMA_ON=1
} IOCTL_LAYER_CHROMA_STATE;

typedef struct
{
  uint32_t red;
  uint32_t green;
  uint32_t blue;
} IOCTL_LAYER_CHROMA_KEY;

typedef enum
{
	IOCTL_DISPLAY_LVDS=0,
	IOCTL_DISPLAY_HDMI=1,
	IOCTL_DISPLAY_TCON=2
} IOCTL_DISPLAY_TYPE;

/************************************************
	LALYER POSITION
*************************************************/
#define IOCTL_SET_LAYER_POS 10
#define IOCTL_GET_LAYER_POS 11

struct IOCTL_LAYER_POS
{
	uint16_t 		id;	/* layer index */
	IOCTL_DCU_POSITION	pos; 	/* layer position */
};

/************************************************
	LALYER ALFA VALUE
*************************************************/
#define IOCTL_SET_LAYER_ALPHA_VAL	20
#define IOCTL_GET_LAYER_ALPHA_VAL	21

struct IOCTL_LAYER_ALFA_VAL
{
	uint16_t 	id;	/* layer index */
	uint8_t		val; 	/* layer alpha value */
};

/************************************************
	LALYER ALFA MODE
*************************************************/
#define IOCTL_SET_LAYER_ALPHA_MODE	22
#define IOCTL_GET_LAYER_ALPHA_MODE	23

struct IOCTL_LAYER_ALFA_KEY
{
	uint16_t 		id;	/* layer index */
	IOCTL_DCU_ALPHAKEY 	key; 	/* layer alpha key */
};

/************************************************
	LALYER CHROMA SET/GET
*************************************************/
#define IOCTL_SET_LAYER_CHROMA_KEY	24
#define IOCTL_GET_LAYER_CHROMA_KEY	25

struct IOCTL_LAYER_CHROMA
{
	uint16_t 			id;	/* layer index */
	IOCTL_LAYER_CHROMA_STATE	state;	/* layer chroma state */
	IOCTL_LAYER_CHROMA_KEY 		min;	/* min color */
	IOCTL_LAYER_CHROMA_KEY 		max;	/* max color */
};

/************************************************
	DISPLAY SETTINGS SET/GET
*************************************************/
#define IOCTL_SET_DISPLAY_CFG		26
#define IOCTL_PRINT_DISPLAY_INFO	27

struct IOCTL_DISPLAY_CFG
{
	IOCTL_DISPLAY_TYPE disp_type;	/* display type */
	uint32_t clock_freq;		/* display clock frequency */
	uint16_t hactive;		/* horizontal resolution */
	uint16_t vactive;		/* vertical resolution */
	uint16_t hback_porch;		/* horizontal back porch */
	uint16_t hfront_porch;		/* horizontal front porch */
	uint16_t vback_porch;		/* vertical back porch */
	uint16_t vfront_porch;		/* vertical front porch */
	uint16_t hsync_len;		/* horizontal len */
	uint16_t vsync_len;		/* vertical len */
};

#endif

