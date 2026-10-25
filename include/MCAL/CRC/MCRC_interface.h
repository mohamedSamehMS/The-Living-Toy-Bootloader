/*
 * MCRC_interface.h
 *
 *  PUBLIC interface of the MCRC driver.
 *
 *  Reference: RM0368 Rev 6, Ch 4 (CRC)
 *
 *  BOOT-ORDER CONTRACT: MRCC_Init(), then MRCC_EnablePeripheralClock
 *  (MRCC_CRC) — or MCRC_Init(), which performs the enable itself
 *  (house pattern: the driver owns its clock bit). Writes to the
 *  un-clocked unit are silently ignored.
 *
 *  =====================================================================
 *  THE CROSS-SEAT CONTRACT — the variant this driver computes
 *  (FOTA-03 coordination point: the ESP8266 seat and the PC sender
 *  implement THIS variant, byte order, and padding — or verification
 *  falsely fails on good data):
 *
 *      Polynomial   : 0x04C11DB7 (fixed in silicon, MSB-first)
 *      Initial value: 0xFFFFFFFF (fixed)
 *      Reflection   : NONE — input and output, both sides
 *      Final XOR    : NONE
 *      Variant name : "CRC-32/MPEG-2"-style, NON-REFLECTED — this is
 *                     NOT zlib/PKZIP CRC-32 (which reflects). Same
 *                     polynomial, different arithmetic. The silicon
 *                     cannot be configured to match zlib.
 *      Feeding      : byte stream in ADDRESS ORDER. The unit consumes
 *                     the MSB of each written word FIRST, so the driver
 *                     byte-reverses each word (__builtin_bswap32) before
 *                     writing DR — the canonical feeding mode; everything
 *                     (CalcBuffer, CalcWord, SelfTest) goes through it.
 *      Padding      : buffers not a multiple of 4 bytes are padded with
 *                     0x00 to the next word boundary — (variant, order,
 *                     padding) is ONE matched triple the sender side
 *                     must replicate in full.
 *      Word width   : one DR write = one 32-bit word; the running CRC
 *                     is readable from DR at any time (streaming).
 *  =====================================================================
 *
 *  NO INTERRUPTS: the CRC unit has no IRQ line on the F401 (not in
 *  the MNVIC enum — correctly: nothing to enable). Zero storm surface
 *  by construction. Pure register polling.
 *
 *  Revision history:
 *    Rev 1 — first release (FOTA-03 spec; CalcWord streaming for the
 *            bootloader's boot-time region validation; SelfTest KAT)
 *
 *
 *
 *      Wire encoding: when a computed CRC is serialized onto the
 *                     wire or into a trailer (per-chunk 4-byte CRC,
 *                     image trailer fields), the 32-bit result is
 *                     written BIG-ENDIAN (MSB first). The triple above
 *                     governs how bytes ENTER the computation; this
 *                     line governs how the result LEAVES it.
 *  Created on: Sep 15, 2026
 */

#ifndef MCRC_INTERFACE_H_
#define MCRC_INTERFACE_H_

#include "STD_TYPES.h"

typedef enum
{
    MCRC_OK = 0,
    MCRC_NOT_INITIALIZED,       /* MCRC_Init has not run               */
    MCRC_NULL_POINTER,          /* NULL buffer / result pointer        */
    MCRC_INVALID_PARAM,         /* zero length                         */
    MCRC_SELFTEST_FAILED       /* KAT mismatch — see MCRC_KAT_EXPECTED */
} MCRC_ReturnType_t;

/* Enable the AHB1 clock (MRCC_CRC) and reset the unit. Idempotent. */
MCRC_ReturnType_t MCRC_Init(void);

/* Reset the running computation (CR.RESET). Each new computation MUST
 * begin here — DR retains state across calls; forgetting RESET mixes
 * the previous computation into the new one, silently. Ungated (a
 * reset is always safe). */
MCRC_ReturnType_t MCRC_Reset(void);

/* One-shot CRC of a byte buffer, in the canonical mode: address-order
 * bytes, 0x00-padded to the word boundary. Resets, feeds, returns the
 * final value through the pointer. The FOTA per-chunk and whole-image
 * checks both call this. Length is u32 — the boot-time app-region
 * validation spans 128+ KB. */
MCRC_ReturnType_t MCRC_CalcBuffer(const u8 *Copy_pu8Data, u32 Copy_u32Length,
                                  u32 *Copy_pu32Crc);

/* Streaming primitive: feed ONE word (already in the canonical
 * byte-reversed form — the bootloader walks flash word-by-word in
 * address order, so it passes each word through bswap itself, or use
 * CalcBuffer on the whole region). Does NOT reset — pair with
 * MCRC_Reset at the start of each computation. Returns the running
 * CRC through the pointer. */
MCRC_ReturnType_t MCRC_CalcWord(u32 Copy_u32Word, u32 *Copy_pu32Crc);

/* Known-answer test: CRC of "123456789" (padded per the rule) must
 * equal MCRC_KAT_EXPECTED. Verifies the variant AND the endianness
 * handling in one call. THE bench verification of the paper constant —
 * run once on hardware; on mismatch, correct the config constant to
 * the measured value and re-run (the ground-truth rule). */
MCRC_ReturnType_t MCRC_SelfTest(void);

#endif /* MCRC_INTERFACE_H_ */
