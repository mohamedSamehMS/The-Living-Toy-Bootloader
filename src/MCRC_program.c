/*
 * MCRC_program.c
 *
 *  Implementation of the MCRC driver — the smallest driver in the
 *  library: three registers, no interrupts, no timing, no config.
 *
 *  Reference: RM0368 Rev 6, Ch 4 (CRC), §4.4
 *
 *  DESIGN PRINCIPLES:
 *   1. ONE CANONICAL FEEDING MODE (FOTA-03 TRAP 2): byte stream in
 *      address order, each word byte-reversed before the DR write
 *      (the unit consumes MSB-first). Every entry point goes through
 *      it — an un-reversed feed computes the CRC of a reversed stream,
 *      plausible-looking and wrong.
 *   2. RESET DISCIPLINE (FOTA-03 correctness note 1): DR retains
 *      state; CalcBuffer resets internally, CalcWord is deliberately
 *      streaming — the caller pairs it with MCRC_Reset.
 *   3. NO STORM SURFACE: no IRQ line exists; nothing to arm, nothing
 *      to service.
 *
 *  Revision history:
 *    Rev 1 — first release
 *
 *  Created on: Sep 15, 2026
 */
#include "STD_TYPES.h"
#include <MCAL/RCC/MRCC_interface.h>

#include <MCAL/CRC/MCRC_private.h>
#include <MCAL/CRC/MCRC_interface.h>
#include <MCAL/CRC/MCRC_config.h>





static u8 Global_u8IsInitialized = 0;

MCRC_ReturnType_t MCRC_Init(void)
{
    if (MRCC_EnablePeripheralClock(MRCC_CRC) != MRCC_OK)
    {
        return MCRC_NOT_INITIALIZED;    /* unreachable with our enum — belt
                                           and braces */
    }
    MCRC_CRC_REG->CR = (1uL << MCRC_CR_RESET);   /* clean state        */
    Global_u8IsInitialized = 1u;
    return MCRC_OK;
}

MCRC_ReturnType_t MCRC_Reset(void)
{
    /* Ungated: a reset is always safe (the disarm-always-succeeds rule) */
    MCRC_CRC_REG->CR = (1uL << MCRC_CR_RESET);
    return MCRC_OK;
}

MCRC_ReturnType_t MCRC_CalcBuffer(const u8 *Copy_pu8Data, u32 Copy_u32Length,
                                  u32 *Copy_pu32Crc)
{
    u32 Local_u32WordCount = Copy_u32Length / 4u;
    u32 Local_u32TailBytes  = Copy_u32Length % 4u;
    u32 Local_u32Index;
    u32 Local_u32Word;

    if (Global_u8IsInitialized == 0u)  { return MCRC_NOT_INITIALIZED; }
    if (Copy_pu8Data == (const u8 *)0) { return MCRC_NULL_POINTER; }
    if (Copy_pu32Crc == (u32 *)0)      { return MCRC_NULL_POINTER; }
    if (Copy_u32Length == 0u)          { return MCRC_INVALID_PARAM; }

    (void)MCRC_Reset();

    /* Full words — canonical mode: assemble each word in address order,
       bswap it (unit eats MSB first), write DR. */
    for (Local_u32Index = 0u; Local_u32Index < Local_u32WordCount; Local_u32Index++)
    {
        const u8 *Local_pu8Word = &Copy_pu8Data[Local_u32Index * 4u];
        Local_u32Word = ((u32)Local_pu8Word[0] << 24) |
                        ((u32)Local_pu8Word[1] << 16) |
                        ((u32)Local_pu8Word[2] << 8)  |
                        ((u32)Local_pu8Word[3]);
        /* Word is already big-endian-assembled = the bswap of the native
           little-endian load. Writing it directly IS the canonical feed. */
        MCRC_CRC_REG->DR = Local_u32Word;
    }

    /* Tail: pad with 0x00 to the word boundary (the documented rule —
       part of the cross-seat triple) */
    if (Local_u32TailBytes != 0u)
    {
        Local_u32Word = 0u;
        if (Local_u32TailBytes >= 1u) { Local_u32Word |= ((u32)Copy_pu8Data[Local_u32WordCount * 4u + 0u] << 24); }
        if (Local_u32TailBytes >= 2u) { Local_u32Word |= ((u32)Copy_pu8Data[Local_u32WordCount * 4u + 1u] << 16); }
        if (Local_u32TailBytes >= 3u) { Local_u32Word |= ((u32)Copy_pu8Data[Local_u32WordCount * 4u + 2u] << 8); }
        MCRC_CRC_REG->DR = Local_u32Word;
    }

    *Copy_pu32Crc = MCRC_CRC_REG->DR;
    return MCRC_OK;
}

MCRC_ReturnType_t MCRC_CalcWord(u32 Copy_u32Word, u32 *Copy_pu32Crc)
{
    if (Global_u8IsInitialized == 0u) { return MCRC_NOT_INITIALIZED; }
    if (Copy_pu32Crc == (u32 *)0)     { return MCRC_NULL_POINTER; }

    /* Streaming primitive: the word arrives CANONICAL (byte-reversed).
       The bootloader reads flash words natively (little-endian) and
       passes __builtin_bswap32(word) — or uses CalcBuffer. */
    MCRC_CRC_REG->DR = Copy_u32Word;
    *Copy_pu32Crc = MCRC_CRC_REG->DR;
    return MCRC_OK;
}

MCRC_ReturnType_t MCRC_SelfTest(void)
{
    u32 Local_u32Crc;

    if (Global_u8IsInitialized == 0u) { return MCRC_NOT_INITIALIZED; }

    if (MCRC_CalcBuffer((const u8 *)MCRC_KAT_STRING, MCRC_KAT_LENGTH, &Local_u32Crc) != MCRC_OK)
    {
        return MCRC_SELFTEST_FAILED;
    }
    if (Local_u32Crc != MCRC_KAT_EXPECTED)
    {
        return MCRC_SELFTEST_FAILED;   /* correct the config constant to the
                                          MEASURED value, then re-run — the
                                          ground-truth rule, applied to
                                          ourselves */
    }
    return MCRC_OK;
}
