/*
 * MFLS_private.h
 *
 *  PRIVATE header of the MFLS driver — never include from HAL or
 *  Application code. Hardware facts only: the FLASH interface base
 *  address, the register map, bit positions, the unlock key
 *  sequence, and the F401CC sector map.
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Ch. 3: FLASH
 *
 *  GROUND-TRUTH NOTE (before the first bench run): the SR and CR bit
 *  positions below were written from the RM0368 programming model and
 *  should be diffed ONCE against the CMSIS device header
 *  (stm32f401xc.h, FLASH_SR_* / FLASH_CR_* defines) or the manual
 *  itself — 2 minutes, and pinning register facts is the house rule.
 *
 *  Revision history:
 *    Rev 1 — initial four-file set (FOTA seat)
 *
 *  Created on: (set on first commit)
 *      Author: Mohamed Sameh (drafted with the FOTA Seat AI session)
 */

#ifndef MFLS_PRIVATE_H_
#define MFLS_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  FLASH interface base address (RM0368 §2.3 memory map)
 *
 *  NOTE — deliberate duplicate, flagged for the cleanup MRCC itself
 *  requested: MRCC_private.h defines FLASH_ACR_ADDRESS with the SAME
 *  value (flash latency must be programmed during clock init, before
 *  this driver existed). When MRCC is next revised, both files should
 *  reference one shared "FLASH interface facts" definition. Until
 *  then this driver keeps its own map — and NEVER writes ACR: MRCC
 *  owns LATENCY/PRFTEN (one owner per register; latency is a
 *  clock-tree concern, not a storage concern).
 * ===================================================================== */
#define MFLS_FLASH_BASE_ADDRESS        (0x40023C00UL)

/* =====================================================================
 *  Register map (RM0368 §3.7 — offsets verified)
 *
 *  ACR     : access control — OWNED BY MRCC, never written here
 *  KEYR    : unlock key port
 *  OPTKEYR : option-byte unlock — UNUSED in v1 (option bytes deferred)
 *  SR      : status — BSY, EOP, error flags (W1C semantics)
 *  CR      : control — PG/SER/MER/SNB/PSIZE/STRT/LOCK (EOPIE/ERRIE
 *            exist but stay untouched: v1 arms no interrupt source)
 *  OPTCR   : option control — UNUSED in v1
 * ===================================================================== */
typedef struct
{
    volatile u32 ACR;        /* 0x00 : Access control register         */
    volatile u32 KEYR;       /* 0x04 : Key register (unlock)           */
    volatile u32 OPTKEYR;    /* 0x08 : Option key register (unused)    */
    volatile u32 SR;         /* 0x0C : Status register                 */
    volatile u32 CR;         /* 0x10 : Control register                */
    volatile u32 OPTCR;      /* 0x14 : Option control (unused v1)      */
    volatile u32 OPTCR1;     /* 0x18 : Option control 1 (unused v1 —
                                    present in the CMSIS device map; the
                                    struct models the FULL hardware map,
                                    the RCC-struct house rule).            */
} MFLS_Registers_t;

#define MFLS_FLASH                   ((volatile MFLS_Registers_t *) MFLS_FLASH_BASE_ADDRESS)

/* =====================================================================
 *  Unlock keys (RM0368 §3.5.2)
 *
 *  Write KEY1 then KEY2 to KEYR while LOCK is set. A WRONG key at
 *  any point locks the controller until the next RESET — which is
 *  why MFLS_privateUnlock writes keys ONLY after reading LOCK == 1,
 *  in the exact order, and verifies success by read-back. Keys are
 *  never written speculatively.
 * ===================================================================== */
#define MFLS_KEY1                     (0x45670123UL)
#define MFLS_KEY2                     (0xCDEF89ABUL)

/* =====================================================================
 *  FLASH_CR bits (RM0368 §3.7.5)
 * ===================================================================== */
#define MFLS_CR_PG                    0      /* Bit  0    : programming enable      */
#define MFLS_CR_SER                   1      /* Bit  1    : sector erase enable     */
#define MFLS_CR_MER                   2      /* Bit  2    : mass erase (unused v1)  */
#define MFLS_CR_SNB                   3      /* Bits 3:7  : sector number field —
                                                CMSIS mask 0xF8. Bits 3:6 select
                                                sectors 0..5 on this single-bank
                                                part; bit 7 is the family's
                                                dual-bank sector selector,
                                                unimplemented on the F401 — the
                                                full-width mask keeps the field
                                                write honest and matches the
                                                pinned stm32f401xc.h record.     */
#define MFLS_CR_SNB_MASK              (0x1Fu << MFLS_CR_SNB)   /* 0xF8 = FLASH_CR_SNB */
#define MFLS_CR_PSIZE                 8      /* Bits 8:9  : programming parallelism */
#define MFLS_CR_PSIZE_MASK            (0x3u << MFLS_CR_PSIZE)
#define MFLS_CR_PSIZE_X32             2u     /* 00=x8, 01=x16, 10=x32 — v1 is x32  */
#define MFLS_CR_STRT                  16     /* Bit 16    : start erase             */
#define MFLS_CR_LOCK                  31     /* Bit 31    : lock CR                 */

/* =====================================================================
 *  FLASH_SR bits — names and positions pinned against stm32f401xc.h
 *  (the project's CMSIS device header): bit 1 is SOP ("wrong
 *  operation sequence"), bit 7 is PGSERR ("programming sequence
 *  error") in that record — earlier F4 manual revisions use other
 *  names (OPERR / PERR) for the same positions. The driver's error
 *  policy is name-agnostic: ANY set error bit aborts the operation
 *  as MFLS_WRITE_ERROR and the whole board is cleared W1C.
 *
 *  EOP is set by hardware at the end of a successful program/erase;
 *  every flag is cleared by writing 1. Error flags STAY SET until
 *  cleared — so every operation clears stale flags BEFORE starting
 *  and again at the end, otherwise the previous operation's error
 *  leaks into this one's result.
 * ===================================================================== */
#define MFLS_SR_EOP                   0      /* Bit  0 : end of operation           */
#define MFLS_SR_SOP                   1      /* Bit  1 : wrong operation sequence   */
#define MFLS_SR_WRPERR                4      /* Bit  4 : write-protection error     */
#define MFLS_SR_PGAERR                5      /* Bit  5 : alignment error            */
#define MFLS_SR_PGPERR                6      /* Bit  6 : parallelism error          */
#define MFLS_SR_PGSERR                7      /* Bit  7 : programming sequence error */
#define MFLS_SR_BSY                   16     /* Bit 16 : busy (read-only)           */

#define MFLS_SR_ERROR_MASK            ((1uL << MFLS_SR_SOP)    | \
                                       (1uL << MFLS_SR_WRPERR) | \
                                       (1uL << MFLS_SR_PGAERR) | \
                                       (1uL << MFLS_SR_PGPERR) | \
                                       (1uL << MFLS_SR_PGSERR))
#define MFLS_SR_CLEAR_MASK            (MFLS_SR_ERROR_MASK | (1uL << MFLS_SR_EOP))
/* =====================================================================
 *  F401CC sector map (RM0368 §3.3 / F401xC datasheet)
 *
 *  256 KB, SINGLE BANK, non-uniform sectors — erase is by SECTOR
 *  (no page erase) and there is NO read-while-write: while any
 *  erase/program runs, every FLASH-resident instruction fetch stalls
 *  the core until the operation finishes. That stall is why a flash
 *  erase visibly "freezes" the whole board for the erase duration —
 *  expected bench behavior, not a hang (stall contract in the
 *  interface header).
 * ===================================================================== */
#define MFLS_SECTOR_COUNT             6u

#define MFLS_SECTOR_BASE_0            (0x08000000UL)
#define MFLS_SECTOR_SIZE_0            (16UL  * 1024UL)
#define MFLS_SECTOR_BASE_1            (0x08004000UL)
#define MFLS_SECTOR_SIZE_1            (16UL  * 1024UL)
#define MFLS_SECTOR_BASE_2            (0x08008000UL)
#define MFLS_SECTOR_SIZE_2            (16UL  * 1024UL)
#define MFLS_SECTOR_BASE_3            (0x0800C000UL)
#define MFLS_SECTOR_SIZE_3            (16UL  * 1024UL)
#define MFLS_SECTOR_BASE_4            (0x08010000UL)
#define MFLS_SECTOR_SIZE_4            (64UL  * 1024UL)
#define MFLS_SECTOR_BASE_5            (0x08020000UL)
#define MFLS_SECTOR_SIZE_5            (128UL * 1024UL)

#define MFLS_FLASH_END_ADDRESS        (MFLS_SECTOR_BASE_5 + MFLS_SECTOR_SIZE_5)

/* Sector descriptor — entry type of the internal table */
typedef struct
{
    u32 u32Base;
    u32 u32Size;
} MFLS_SectorInfo_t;

#endif /* MFLS_PRIVATE_H_ */
