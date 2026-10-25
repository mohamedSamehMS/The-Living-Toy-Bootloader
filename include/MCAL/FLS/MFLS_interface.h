/*
 * MFLS_interface.h
 *
 *  PUBLIC interface of the MFLS driver. HAL and Application code
 *  must include ONLY this file.
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Ch. 3: FLASH
 *
 *  SCOPE (v1, deliberate):
 *    - WORD programming only (x32, 4-byte aligned). Byte/half-word
 *      variants can be added without breaking this API.
 *    - SECTOR erase only. No mass erase, no option bytes (write-
 *      protecting the future bootloader sector via OPTCR is a later
 *      hardening step).
 *    - Polling only — this driver arms NO interrupt source.
 *      MNVIC_FLASH (position 4) exists in the MNVIC enum and stays
 *      reserved for a possible future EOP-interrupt variant.
 *    - No Init function: the driver is stateless (MGPIO precedent).
 *      The FLASH interface registers need no RCC clock enable and no
 *      one-time setup; lock state lives in hardware (CR.LOCK) and is
 *      managed per operation.
 *    - ACR is never written: MRCC owns LATENCY/PRFTEN.
 *
 *  CALLING CONTRACTS:
 *    - Thread context ONLY — never call MFLS from an ISR (the
 *      unlock -> program -> wait sequence must not interleave).
 *    - Single context: concurrent MFLS calls from two loops are not
 *      supported (v1 has no scheduler).
 *    - Erase the sector before programming it. With the config
 *      check enabled, writing non-erased words is rejected loudly
 *      (MFLS_NOT_ERASED) — programming only clears bits (1->0).
 *
 *  THE STALL CONTRACT (single bank, no read-while-write):
 *    While any erase/program runs, every FLASH-resident instruction
 *    fetch stalls the core until it completes. Consequences — all
 *    expected, all documented:
 *      - a sector erase visibly freezes the whole board for the
 *        erase duration (~0.5 s for 16 KB, ~2 s for the 128 KB
 *        sector, datasheet typ): no SysTick, no interrupts, no UI.
 *        Callers plan around it (FOTA does: the "Preparing..." state).
 *      - interrupts pending during a stall fire late (coalesced),
 *        they are not lost.
 *      - the polling loops in this driver are themselves stalled
 *        during a legitimate operation — the timeout bound therefore
 *        guards post-operation pathological states (BSY stuck,
 *        error-locked controller), not the operation itself.
 *
 *  Revision history:
 *    Rev 1 — initial four-file set (FOTA seat)
 *
 *  Created on: (set on first commit)
 *      Author: Mohamed Sameh (drafted with the FOTA Seat AI session)
 */

#ifndef MFLS_INTERFACE_H_
#define MFLS_INTERFACE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Driver status codes — same convention as MRCC/MGPIO/MNVIC
 * ===================================================================== */
typedef enum
{
    MFLS_OK = 0,              /* Operation completed                       */
    MFLS_INVALID_SECTOR,      /* Sector id outside the supported set       */
    MFLS_PROTECTED_SECTOR,    /* Sector exists but is outside this build's
                                 writable window (MFLS_config.h) — the
                                 bootloader sector cannot be erased by an
                                 app-build binary, by design              */
    MFLS_INVALID_PARAM,       /* NULL pointer, zero word count, or an
                                 address that is not 4-byte aligned       */
    MFLS_INVALID_ADDRESS,     /* Address outside the 256 KB flash array    */
    MFLS_NOT_ERASED,          /* Target word(s) do not read 0xFFFFFFFF —
                                 programming would corrupt silently;
                                 nothing was written (config-gated)       */
    MFLS_BUSY,                /* Controller BSY already set on entry       */
    MFLS_TIMEOUT,             /* BSY never cleared within the derived
                                 bound (MFLS_config.h)                    */
    MFLS_LOCKED,              /* Unlock failed — a wrong key was written
                                 somewhere earlier: the controller stays
                                 locked until the next RESET. Power-cycle
                                 and investigate before retrying          */
    MFLS_WRITE_ERROR          /* SR error flags were set (WRPERR/PGAERR/
                                 PGPERR/OPERR...) — flags are cleared
                                 before returning so the next operation
                                 starts from a clean SR                   */
} MFLS_ReturnType_t;

/* =====================================================================
 *  Sector identifier — values are indices into the internal sector
 *  table and MUST stay 0..5 in order (MGPIO_Pin_t pattern).
 * ===================================================================== */
typedef enum
{
    MFLS_SECTOR_0 = 0,        /* 16 KB  : 0x08000000 — future bootloader  */
    MFLS_SECTOR_1,            /* 16 KB  : 0x08004000 — app region start   */
    MFLS_SECTOR_2,            /* 16 KB  : 0x08008000                      */
    MFLS_SECTOR_3,            /* 16 KB  : 0x0800C000                      */
    MFLS_SECTOR_4,            /* 64 KB  : 0x08010000 — app region end     */
    MFLS_SECTOR_5             /* 128 KB : 0x08020000 — FOTA staging       */
} MFLS_Sector_t;

/* =====================================================================
 *  API
 * ===================================================================== */

/* Erase one sector. Validates the sector id AND the build's writable
 * window BEFORE anything is unlocked or written. Sequence: unlock ->
 * clear stale flags -> SER+SNB -> STRT -> poll BSY -> check EOP and
 * error flags -> clear flags -> disarm SER -> relock.
 * Expect the board to freeze for the erase duration (stall contract). */
MFLS_ReturnType_t MFLS_EraseSector(MFLS_Sector_t Copy_Sector);

/* Program a run of 32-bit words at Copy_u32Address.
 *   - address must be 4-byte aligned; the whole run must lie inside
 *     the build's writable window
 *   - with the config check on, every target word must read
 *     0xFFFFFFFF beforehand — the check runs BEFORE the controller
 *     is unlocked, so a rejected call writes nothing at all
 *   - PG + PSIZE(x32) are set once per call; each word is one store,
 *     one BSY wait, one EOP/error check-and-clear
 * Reading programmed data afterwards is a plain memory access — no
 * read API exists because flash is memory-mapped (see example). */
MFLS_ReturnType_t MFLS_ProgramWords(u32 Copy_u32Address,
                                    const u32 *Copy_pu32Data,
                                    u16 Copy_u16WordCount);

/* Sector id -> base address and size (the non-uniform F401CC map).
 * The FOTA receiver and bootloader use this instead of hardcoding
 * 0x08020000-style constants. */
MFLS_ReturnType_t MFLS_GetSectorBounds(MFLS_Sector_t Copy_Sector,
                                       u32 *Copy_pu32Base,
                                       u32 *Copy_pu32Size);

/* Flash address -> owning sector id. Fails for addresses outside the
 * 256 KB array. */
MFLS_ReturnType_t MFLS_GetSectorOfAddress(u32 Copy_u32Address,
                                          MFLS_Sector_t *Copy_pSector);


/* Debug/verification aid: raw FLASH_SR value captured just before the
   last status clear — the flags the most recent completed operation
   left behind. Read IMMEDIATELY after an MFLS call returns; the next
   operation's pre-clear overwrites it. Bit map: 0=EOP, 1=SOP,
   4=WRPERR, 5=PGAERR, 6=PGPERR, 7=PGSERR, 16=BSY. */
MFLS_ReturnType_t MFLS_GetLastStatusSnapshot(u32 *Copy_pu32Snapshot);

/* =====================================================================
 *  Usage example (Phase-0 harness pattern):
 *
 *      u32  Local_au32Pattern[4] = {0xDEADBEEFUL, 0x12345678UL, 0, 1};
 *      u32  Local_u32Base;
 *      u32  Local_u32Size;
 *
 *      if (MFLS_GetSectorBounds(MFLS_SECTOR_5, &Local_u32Base, &Local_u32Size) == MFLS_OK)
 *      {
 *          if (MFLS_EraseSector(MFLS_SECTOR_5) == MFLS_OK)
 *          {
 *              (void)MFLS_ProgramWords(Local_u32Base, Local_au32Pattern, 4u);
 *          }
 *      }
 *      / * read-back: plain memory access, no API:
 *        if (*(volatile u32 *)Local_u32Base == 0xDEADBEEFUL) { ... }
 *      * /
 * ===================================================================== */

#endif /* MFLS_INTERFACE_H_ */
