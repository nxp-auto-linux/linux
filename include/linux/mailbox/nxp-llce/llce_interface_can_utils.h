/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2020-2023 NXP */
#ifndef LLCE_INTERFACE_CAN_UTILS_H
#define LLCE_INTERFACE_CAN_UTILS_H

#include "llce_can.h"

/* Masks used for un-shifted values, for checking user input
 */
/** Frame RTR field mask unpacked. */
#define LLCE_CAN_MB_RTR_UNPACKED (0x00000001U)
/** Frame IDE field mask unpacked. */
#define LLCE_CAN_MB_IDE_UNPACKED (0x00000001U)
/** Frame standard ID field mask unpacked. */
#define LLCE_CAN_MB_IDSTD_MASK_UNPACKED (0x0000007FF)
/** Frame BRS field mask unpacked. */
#define LLCE_CAN_MB_BRS_UNPACKED (0x00000001U)
/** Frame FDF field mask unpacked. */
#define LLCE_CAN_MB_FDF_UNPACKED (0x00000001U)
/** Frame ESI field mask unpacked. */
#define LLCE_CAN_MB_ESI_UNPACKED (0x00000001U)
/**
 * word0 of a CAN frame.
 * It contains the message ID and related configurations.
 **/
struct llce_can_word0 {
	/**
	 * INPUT/OUTPUT: CAN frame Standard or Extended ID field.
	 * In base frame format, only the 11 most significant bits (28-18)
	 * are used for frame identification. The 18 least significant bits are
	 * ignored.
	 *
	 * In extended frame format, all bits (0-28) are used for frame
	 * identification.
	 */
	u32 id;
	/**
	 * INPUT/OUTPUT: CAN frame Remote Transmission Request field.
	 *
	 * If BCAN transmits this bit as one (recessive) and receives it as zero
	 * (dominant), it is interpreted as an arbitration
	 * loss. If this bit is transmitted as zero (dominant), then if it
	 * is received as one (recessive), BCAN treats it as a bit error.
	 * If the value received matches the value transmitted, it is considered
	 * a successful bit transmission.
	 */
	u8 rtr;
	/**
	 * INPUT/OUTPUT: CAN frame Identifier Extension field.
	 *
	 * This field identifies whether the frame format is base (standard ID)
	 * or extended.
	 */
	u8 ide;
} __aligned(4) __packed;

/**
 * word1 of a CAN frame.
 * It contains CAN frame format and bit rate related configurations.
 **/
struct llce_can_word1 {
	/**
	 * INPUT/OUTPUT: CAN frame Data Length Code field.
	 *
	 * This 4-bit field defines the number of bytes in the data field
	 * of a CAN frame (DATA BYTE 0 to DATA BYTE 63).
	 * When RTR = 1, the frame is a remote request and does not include
	 * the data field, regardless of the DLC field.
	 */
	u8 dlc;
	/**
	 * INPUT/OUTPUT: CAN frame FD Format Indicator field.
	 *
	 * This bit distinguishes between classical CAN format and CAN FD format
	 * frames.
	 */
	u8 fdf;
	/**
	 * INPUT/OUTPUT: CAN frame Bit Rate Switch field.
	 *
	 * This bit defines whether the bit rate is switched inside a CAN FD
	 * frame. This bit is meaningful only if FDF = 1, otherwise it is not
	 * used by BCAN.
	 */
	u8 brs;
	/**
	 * INPUT/OUTPUT: CAN frame Error State Indicator field.
	 *
	 * This bit indicates if the transmitting node is error active or error
	 * passive. This bit is meaningful only if FDF = 1, otherwise it is not
	 * used by BCAN.
	 */
	u8 esi;
} __aligned(4) __packed;

/**
 * Pack members of struct llce_can_word0 into CAN frame
 * format.
 * This is a helper function abstracts the CAN frame
 * layout away from the user when building a CAN frame.
 * @param llce_word0* - pointer to struct holding fields of word0 part from CAN
 * frame. The result is to be copied into word0 field of llce_can_mb_type
 * struct.
 */
static inline u32 llce_can_pack_word0(const struct llce_can_word0 *llce_word0)
{
	u32 word0_temp = 0U;

	word0_temp |= ((llce_word0->rtr & LLCE_CAN_MB_RTR_UNPACKED)
			   << LLCE_CAN_MB_RTR_SHIFT);
	word0_temp |= ((llce_word0->ide & LLCE_CAN_MB_IDE_UNPACKED)
			   << LLCE_CAN_MB_IDE_SHIFT);
	if (LLCE_CAN_MB_IDE_UNPACKED ==
	    (llce_word0->ide & LLCE_CAN_MB_IDE_UNPACKED)) {
		/* Extended Id
		 */
		word0_temp |= (llce_word0->id & LLCE_CAN_MB_ID_MASK);
	} else {
		/* Standard Id
		 */
		word0_temp |=
			((llce_word0->id & LLCE_CAN_MB_IDSTD_MASK_UNPACKED)
			 << LLCE_CAN_MB_IDSTD_SHIFT);
	}

	return word0_temp;
}

/**
 * Pack members of struct llce_can_word1 into CAN frame
 * format.
 * This is a helper function abstracts the CAN frame
 * layout away from the user when building a CAN frame.
 * @param llce_word1* - pointer to struct holding fields of word1 part from CAN
 * frame. The result is to be copied into word1 field of llce_can_mb_type
 * struct.
 */
static inline u32 llce_can_pack_word1(const struct llce_can_word1 *llce_word1)
{
	u32 word1_temp = 0U;

	word1_temp |= ((llce_word1->brs & LLCE_CAN_MB_BRS_UNPACKED)
		       << LLCE_CAN_MB_BRS_SHIFT);
	word1_temp |= ((llce_word1->fdf & LLCE_CAN_MB_FDF_UNPACKED)
		       << LLCE_CAN_MB_FDF_SHIFT);
	word1_temp |= ((llce_word1->esi & LLCE_CAN_MB_ESI_UNPACKED)
		       << LLCE_CAN_MB_ESI_SHIFT);
	word1_temp |= (llce_word1->dlc & LLCE_CAN_MB_DLC_MASK);

	return word1_temp;
}

/**
 * Unpack fields from CAN frame into struct llce_can_word0
 * struct.
 * This is a helper function which extracts the fields of
 * word0 from a CAN frame and populates the fields of struct llce_can_word0.
 * @param word0 - word0 part of CAN frame with fields that match CAN layout.
 * The input parameter is word0 field of llce_can_mb_type struct.
 */
static inline struct llce_can_word0 llce_can_unpack_word0(u32 word0)
{
	struct llce_can_word0 llce_word0;

	llce_word0.rtr = (u8)((word0 & LLCE_CAN_MB_RTR) >>
			 LLCE_CAN_MB_RTR_SHIFT);
	llce_word0.ide = (u8)((word0 & LLCE_CAN_MB_IDE) >>
			 LLCE_CAN_MB_IDE_SHIFT);
	if (LLCE_CAN_MB_IDE_UNPACKED ==
	    (llce_word0.ide & LLCE_CAN_MB_IDE_UNPACKED)) {
		/* Extended Id
		 */
		llce_word0.id = word0 & LLCE_CAN_MB_ID_MASK;
	} else {
		/* Standard Id
		 */
		llce_word0.id = ((word0 & LLCE_CAN_MB_IDSTD_MASK) >>
			    LLCE_CAN_MB_IDSTD_SHIFT);
	}

	return llce_word0;
}

/**
 * Unpack fields from CAN frame into struct llce_can_word1
 * struct.
 * This is a helper function which extracts the fields of
 * word1 from a CAN frame and populates the fields of struct llce_can_word1.
 * @param word1 - word1 part of CAN frame with fields that match CAN layout.
 * The input parameter is word1 field of llce_can_mb_type struct.
 */
static inline struct llce_can_word1 llce_can_unpack_word1(u32 word1)
{
	struct llce_can_word1 llce_word1;

	llce_word1.esi = (u8)((word1 & LLCE_CAN_MB_ESI) >>
			 LLCE_CAN_MB_ESI_SHIFT);
	llce_word1.brs = (u8)((word1 & LLCE_CAN_MB_BRS) >>
			 LLCE_CAN_MB_BRS_SHIFT);
	llce_word1.fdf = (u8)((word1 & LLCE_CAN_MB_FDF) >>
			 LLCE_CAN_MB_FDF_SHIFT);
	llce_word1.dlc = (u8)(word1 & LLCE_CAN_MB_DLC_MASK);

	return llce_word1;
}

/**
 * Retrieve message buffer from shared memory based
on rx descriptor index and
 * unpack fields from the message buffer.
 * This is a helper function which takes a
descriptor as input
 * and returns the corresponding message buffer, which can be either a long (64
bytes) or short (8bytes) one.
 * It also retrieves the relevant fields from the message buffer.
 * The lower indexes correspond to long message buffers, followed by the short
ones
 * in the rx descriptor array, llce_can_rx_mb_desc_type can_rx_mb_desc.
 * @param p_can_shared_memory   - Pointer to shared memory.
.* @param rx_mb_desc_idx	  - Rx message buffer descriptor index.
 * @param word0			- word0 field of message buffer.
 * @param word1			- word1 field of message buffer.
 * @param payload		   - Payload - either long or short.
 * @param timestamp		- Timestamp from a clock source.
 */
static inline void
llce_can_get_mb_data(struct llce_can_shared_memory *p_can_shared_memory,
		     u16 rx_mb_desc_idx, u32 *word0, u32 *word1,
		     u8 **payload, u32 *timestamp)
{
	/* Decide whether it's a 64B frame or a 8B frame (Short frame)
	 */
	if (rx_mb_desc_idx < LLCE_CAN_CONFIG_MAXRXMB) {
		*word0 = p_can_shared_memory->can_mb[rx_mb_desc_idx].word0;
		*word1 = p_can_shared_memory->can_mb[rx_mb_desc_idx].word1;
		*payload = (u8 *)p_can_shared_memory->can_mb[rx_mb_desc_idx]
				   .payload;
		*timestamp = p_can_shared_memory->can_mb[rx_mb_desc_idx].timestamp;
	} else {
		*word0 = p_can_shared_memory
				     ->can_short_mb[rx_mb_desc_idx -
						    LLCE_CAN_CONFIG_MAXRXMB]
				     .word0;
		*word1 = p_can_shared_memory
				     ->can_short_mb[rx_mb_desc_idx -
						    LLCE_CAN_CONFIG_MAXRXMB]
				     .word1;
		*payload = (u8 *)p_can_shared_memory
				   ->can_short_mb[rx_mb_desc_idx -
						  LLCE_CAN_CONFIG_MAXRXMB]
				   .payload;
		*timestamp = p_can_shared_memory
				     ->can_short_mb[rx_mb_desc_idx -
						    LLCE_CAN_CONFIG_MAXRXMB]
				     .timestamp;
	}
}

#endif /*LLCE_INTERFACE_CAN_UTILS_H*/
