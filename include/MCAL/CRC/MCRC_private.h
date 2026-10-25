/*
 * MCRC_private.h
 *
 *  PRIVATE header of the MCRC driver. Hardware facts only.
 *
 *  Reference: RM0368 Rev 6, Ch 4 (CRC), §4.4 — offsets verified
 *
 *  Revision history:
 *    Rev 1 — first release
 *
 *  Created on: Sep 15, 2026
 *      Author: dell
 */

#ifndef MCRC_PRIVATE_H_
#define MCRC_PRIVATE_H_

#include "STD_TYPES.h"

/* CRC unit base: AHB1, 0x40023000 (AHB1PERIPH + 0x3000) */
#define MCRC_CRC_BASE_ADDRESS       0x40023000UL

typedef struct
{
    volatile u32 DR;        /* 0x00 : Data register — write word to feed,
                                             read for the running/final CRC */
    volatile u32 IDR;       /* 0x04 : Independent data (8-bit scratchpad,
                                      UNRELATED to the checksum — unused)  */
    volatile u32 CR;        /* 0x08 : Control — bit 0 RESET only. Writing 1
                                      resets DR to 0xFFFFFFFF; the bit reads
                                      back 0 (self-clearing).               */
} MCRC_Registers_t;

#define MCRC_CRC_REG   ((volatile MCRC_Registers_t *) MCRC_CRC_BASE_ADDRESS)

/* CR bits */
#define MCRC_CR_RESET              0u

/* DR reset value (after RESET): the fixed init of the computation */
#define MCRC_DR_RESET_VALUE        0xFFFFFFFFUL

#endif /* MCRC_PRIVATE_H_ */
