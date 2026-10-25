/*
 * MFLS_program.c
 *
 *  Implementation of the MFLS driver.
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Ch. 3: FLASH
 *
 *  DESIGN PRINCIPLES (documented, deliberate):
 *
 *  1. VALIDATE FIRST, UNLOCK LAST. Parameters, the writable-window
 *     policy, alignment, and the erased-state of the target words
 *     are ALL checked BEFORE the KEYR sequence runs. A rejected call
 *     leaves the controller locked and the flash untouched — there
 *     is no partially-applied operation.
 *
 *  2. THE WRITABLE WINDOW IS A BUILD FACT, NOT A CONVENTION.
 *     MFLS_config.h states what THIS binary may ever erase/program.
 *     The app build compiles with window = staging only, so the
 *     running application cannot erase its own region even by bug.
 *
 *  3. POLLING ONLY, SYSTICK NEVER. The waits are bounded loop
 *     counters derived from the MRCC published clock model. Two
 *     reasons: the bootloader runs before any SysTick exists, and a
 *     SysTick-based timeout hangs when SysTick is not initialized —
 *     the MSPI bug class, avoided here by construction.
 *
 *  4. STATUS HYGIENE. Stale SR flags are cleared BEFORE every
 *     operation (an uncleared WRPERR from a previous failure would
 *     leak into this operation's result) and again after — errors
 *     are reported once, loudly, then cleared.
 *
 *  5. LOCK DISCIPLINE. Keys are written only after reading
 *     LOCK == 1, only in the exact KEY1/KEY2 order, verified by
 *     read-back (a wrong key locks the controller until reset —
 *     reported as MFLS_LOCKED, never ignored). Every public function
 *     re-locks the controller before returning.
 *
 *  Revision history:
 *    Rev 1 — initial four-file set (FOTA seat)
 *
 *  Created on: (set on first commit)
 *      Author: Mohamed Sameh (drafted with the FOTA Seat AI session)
 */
#include "BIT_MATH.h"
#include "STD_TYPES.h"

#include <MCAL/FLS/MFLS_interface.h>
#include <MCAL/FLS/MFLS_private.h>
#include <MCAL/FLS/MFLS_config.h>



/* =====================================================================
 *  Compile-time consistency guards
 *
 *  The sector table is the ground truth every FOTA region derives
 *  from — a typo in one base address would be catastrophic AND
 *  silent. The adjacency chain below turns any inconsistency into a
 *  BUILD BREAK: each sector must begin exactly where the previous
 *  ends, and the map must cover exactly 256 KB ending at 0x08040000.
 * ===================================================================== */
_Static_assert(MFLS_SECTOR_COUNT == 6u, "F401CC: exactly 6 sectors (4x16K + 64K + 128K)");
_Static_assert(MFLS_SECTOR_BASE_0 + MFLS_SECTOR_SIZE_0 == MFLS_SECTOR_BASE_1, "sector map: S0 must end at S1");
_Static_assert(MFLS_SECTOR_BASE_1 + MFLS_SECTOR_SIZE_1 == MFLS_SECTOR_BASE_2, "sector map: S1 must end at S2");
_Static_assert(MFLS_SECTOR_BASE_2 + MFLS_SECTOR_SIZE_2 == MFLS_SECTOR_BASE_3, "sector map: S2 must end at S3");
_Static_assert(MFLS_SECTOR_BASE_3 + MFLS_SECTOR_SIZE_3 == MFLS_SECTOR_BASE_4, "sector map: S3 must end at S4");
_Static_assert(MFLS_SECTOR_BASE_4 + MFLS_SECTOR_SIZE_4 == MFLS_SECTOR_BASE_5, "sector map: S4 must end at S5");
_Static_assert(MFLS_SECTOR_BASE_5 + MFLS_SECTOR_SIZE_5 == 0x08040000UL, "sector map: flash must end at 0x08040000 (256 KB)");
_Static_assert(MFLS_SECTOR_4 == 4 && MFLS_SECTOR_5 == 5, "sector enum must equal table index");

/* =====================================================================
 *  Private lookup table — sector id -> base/size
 *  Designated initializers (MGPIO pattern): a missing entry breaks
 *  the build instead of indexing past the array.
 * ===================================================================== */
static const MFLS_SectorInfo_t MFLS_AstSectors[MFLS_SECTOR_COUNT] =
{
    [MFLS_SECTOR_0] = { MFLS_SECTOR_BASE_0, MFLS_SECTOR_SIZE_0 },
    [MFLS_SECTOR_1] = { MFLS_SECTOR_BASE_1, MFLS_SECTOR_SIZE_1 },
    [MFLS_SECTOR_2] = { MFLS_SECTOR_BASE_2, MFLS_SECTOR_SIZE_2 },
    [MFLS_SECTOR_3] = { MFLS_SECTOR_BASE_3, MFLS_SECTOR_SIZE_3 },
    [MFLS_SECTOR_4] = { MFLS_SECTOR_BASE_4, MFLS_SECTOR_SIZE_4 },
    [MFLS_SECTOR_5] = { MFLS_SECTOR_BASE_5, MFLS_SECTOR_SIZE_5 }
};

/* Raw FLASH_SR captured by MFLS_privateClearStatus immediately BEFORE
   its W1C clear — the flags the most recently COMPLETED operation left.
   Read it right after an MFLS call returns (the next call's pre-clear
   overwrites it). Bench-diagnostics aid. */
static u32 MFLS_u32LastStatusSnapshot = 0u;


/* =====================================================================
 *  Private helper prototypes
 * ===================================================================== */

/* Unlock per RM0368: keys only when LOCK reads 1, exact order,
   verified by read-back. */
static MFLS_ReturnType_t MFLS_privateUnlock(void);

/* SET_BIT(CR, LOCK) — the controller never stays unlocked between
   public calls. */
static void MFLS_privateLock(void);

/* Poll SR.BSY with the config-derived bounded counter. */
static MFLS_ReturnType_t MFLS_privateWaitNotBusy(void);

/* Snapshot + clear every SR status flag (W1C). Returns
   MFLS_WRITE_ERROR if any error flag was set, MFLS_OK otherwise. */
static MFLS_ReturnType_t MFLS_privateClearStatus(void);

/* Flash address -> owning sector id (no policy check here). */
static MFLS_ReturnType_t MFLS_privateAddressToSector(u32 Copy_u32Address,
                                                     MFLS_Sector_t *Copy_pSector);

/* Sector id validation + writable-window policy check. */
static MFLS_ReturnType_t MFLS_privateValidateWritableSector(MFLS_Sector_t Copy_Sector);

/* =====================================================================
 *  Private helpers
 * ===================================================================== */

static MFLS_ReturnType_t MFLS_privateValidateWritableSector(MFLS_Sector_t Copy_Sector)
{
    /* (u32) cast BEFORE the comparison — C enums are not type-safe
       (MGPIO pattern): garbage that passes a signed test fails here. */
    if ((u32)Copy_Sector > (u32)MFLS_SECTOR_5)
    {
        return MFLS_INVALID_SECTOR;
    }
    if (  ((u32)Copy_Sector < (u32)MFLS_WRITABLE_SECTOR_MIN)
       || ((u32)Copy_Sector > (u32)MFLS_WRITABLE_SECTOR_MAX) )
    {
        return MFLS_PROTECTED_SECTOR;
    }
    return MFLS_OK;
}

static MFLS_ReturnType_t MFLS_privateAddressToSector(u32 Copy_u32Address,
                                                     MFLS_Sector_t *Copy_pSector)
{
    u8 Local_u8Sector;

    if (  (Copy_u32Address <  MFLS_SECTOR_BASE_0)
       || (Copy_u32Address >= MFLS_FLASH_END_ADDRESS) )
    {
        return MFLS_INVALID_ADDRESS;
    }

    for (Local_u8Sector = 0u; Local_u8Sector < MFLS_SECTOR_COUNT; Local_u8Sector++)
    {
        if (  (Copy_u32Address >= MFLS_AstSectors[Local_u8Sector].u32Base)
           && (Copy_u32Address <  (MFLS_AstSectors[Local_u8Sector].u32Base
                                   + MFLS_AstSectors[Local_u8Sector].u32Size)) )
        {
            *Copy_pSector = (MFLS_Sector_t)Local_u8Sector;
            return MFLS_OK;
        }
    }

    return MFLS_INVALID_ADDRESS;   /* unreachable — the adjacency
                                      _Static_asserts pin the table */
}

static MFLS_ReturnType_t MFLS_privateUnlock(void)
{
    /* Never write keys speculatively: a wrong key locks the
       controller until reset. Write only when LOCK is actually set,
       then verify the result. */
    if (GET_BIT(MFLS_FLASH->CR, MFLS_CR_LOCK) == 0u)
    {
        return MFLS_OK;            /* already unlocked */
    }

    MFLS_FLASH->KEYR = MFLS_KEY1;
    MFLS_FLASH->KEYR = MFLS_KEY2;

    if (GET_BIT(MFLS_FLASH->CR, MFLS_CR_LOCK) != 0u)
    {
        /* Either the sequence raced, or a previous wrong-key write
           locked the controller until reset. Report loudly. */
        return MFLS_LOCKED;
    }
    return MFLS_OK;
}

static void MFLS_privateLock(void)
{
    SET_BIT(MFLS_FLASH->CR, MFLS_CR_LOCK);
}

static MFLS_ReturnType_t MFLS_privateWaitNotBusy(void)
{
    u32 Local_u32Timeout = MFLS_TIMEOUT_COUNT;

    /* SR is volatile — re-read every iteration. During a legitimate
       erase the FLASH-RESIDENT loop body is itself stalled with the
       rest of the code (single bank), so this bound guards
       post-operation pathological states — generous by construction
       (see the derivation in MFLS_config.h). */
    while (GET_BIT(MFLS_FLASH->SR, MFLS_SR_BSY) != 0u)
    {
        Local_u32Timeout--;
        if (Local_u32Timeout == 0u)
        {
            return MFLS_TIMEOUT;
        }
    }
    return MFLS_OK;
}

static MFLS_ReturnType_t MFLS_privateClearStatus(void)
{
	MFLS_u32LastStatusSnapshot = MFLS_FLASH->SR;
    MFLS_ReturnType_t Local_ReturnStatus = MFLS_OK;

    if ((MFLS_FLASH->SR & MFLS_SR_ERROR_MASK) != 0u)
    {
        Local_ReturnStatus = MFLS_WRITE_ERROR;
    }

    /* W1C: writing 1 clears EOP and every error flag. Always clear
       everything, so the next operation starts from a clean SR
       whatever happened before. (Unimplemented bits ignore writes.) */
    MFLS_FLASH->SR = MFLS_SR_CLEAR_MASK;

    return Local_ReturnStatus;
}

/* =====================================================================
 *  MFLS_EraseSector
 * ===================================================================== */
MFLS_ReturnType_t MFLS_EraseSector(MFLS_Sector_t Copy_Sector)
{
    MFLS_ReturnType_t Local_ReturnStatus;

    /* 1. Validate everything BEFORE unlocking (principle 1). */
    Local_ReturnStatus = MFLS_privateValidateWritableSector(Copy_Sector);
    if (Local_ReturnStatus != MFLS_OK)
    {
        return Local_ReturnStatus;
    }

    /* 2. Refuse to start on top of a running operation. */
    if (GET_BIT(MFLS_FLASH->SR, MFLS_SR_BSY) != 0u)
    {
        return MFLS_BUSY;
    }

    /* 3. Unlock (keys only when actually locked). */
    Local_ReturnStatus = MFLS_privateUnlock();
    if (Local_ReturnStatus != MFLS_OK)
    {
        return Local_ReturnStatus;
    }

    /* 4. Clear stale status — this operation reports only its own
          outcome. */
    (void)MFLS_privateClearStatus();

    /* 5. Arm the erase: SER + SNB. SNB is only writable while
          BSY == 0 — guaranteed by step 2. Masked field write, never
          a blind RMW: untouched bits keep their values. */
    MFLS_FLASH->CR = (MFLS_FLASH->CR & ~MFLS_CR_SNB_MASK)
                   | (((u32)Copy_Sector << MFLS_CR_SNB) & MFLS_CR_SNB_MASK)
                   | (1uL << MFLS_CR_SER);

    /* 6. START. The erase runs to completion with no CPU help. From
          this store until BSY clears, every flash-resident code
          fetch STALLS — including this very polling loop. The board
          freezes for the erase duration: expected, by design (the
          stall contract in the interface header). */
    SET_BIT(MFLS_FLASH->CR, MFLS_CR_STRT);

    /* 7. Bounded wait. */
    Local_ReturnStatus = MFLS_privateWaitNotBusy();

    if (Local_ReturnStatus == MFLS_OK)
    {
        /* 8. Completion criterion (Rev 3): BSY cleared + no error
              flag set. EOP is NOT required — BENCH FINDING (runs
              1-2, V2 harness, recorded in the revision history):
              on this part EOP never asserts in polled mode — SR
              read 0x00000000 after BOTH a verified-correct sector
              erase AND verified-correct word programming. ST's own
              F4 busy-poll path also trusts BSY + error flags only.
              The EOP bit stays in the W1C clear mask inside
              MFLS_privateClearStatus — harmless if it ever does
              assert (e.g. a future EOPIE-enabled variant). */
        Local_ReturnStatus = MFLS_privateClearStatus();
    }
    else
    {
        /* Timeout path: clear flags too — they may explain the hang. */
        (void)MFLS_privateClearStatus();
    }

    /* 9. Disarm SER — leave CR as we found it, apart from LOCK. */
    CLR_BIT(MFLS_FLASH->CR, MFLS_CR_SER);

    /* 10. Always relock before returning (principle 5). */
    MFLS_privateLock();

    return Local_ReturnStatus;
}

/* =====================================================================
 *  MFLS_ProgramWords
 * ===================================================================== */
MFLS_ReturnType_t MFLS_ProgramWords(u32 Copy_u32Address,
                                    const u32 *Copy_pu32Data,
                                    u16 Copy_u16WordCount)
{
    MFLS_ReturnType_t Local_ReturnStatus;
    MFLS_Sector_t     Local_FirstSector;
    MFLS_Sector_t     Local_LastSector;
    volatile u32     *Local_pu32Target;
    u32               Local_u32LastAddress;
    u32               Local_u32Index;

    /* ---- 1. Parameter validation — nothing touched yet ---- */
    if (Copy_pu32Data == NULL)
    {
        return MFLS_INVALID_PARAM;
    }
    if (Copy_u16WordCount == 0u)
    {
        return MFLS_INVALID_PARAM;
    }
    if ((Copy_u32Address & 0x3u) != 0u)
    {
        /* x32 programming requires word alignment — hardware would
           raise PGAERR; refuse first, loudly (validate-first rule). */
        return MFLS_INVALID_PARAM;
    }

    if (Copy_u32Address < MFLS_SECTOR_BASE_0)
    {
        return MFLS_INVALID_ADDRESS;
    }
    /* u16 word count x 4 cannot overflow; the end-address check below
       keeps the whole run inside the array. */
    Local_u32LastAddress = Copy_u32Address + ((u32)Copy_u16WordCount * 4u) - 1u;
    if (Local_u32LastAddress >= MFLS_FLASH_END_ADDRESS)
    {
        return MFLS_INVALID_ADDRESS;
    }

    /* ---- 2. Writable-window policy — first AND last sector ---- */
    Local_ReturnStatus = MFLS_privateAddressToSector(Copy_u32Address, &Local_FirstSector);
    if (Local_ReturnStatus != MFLS_OK)
    {
        return Local_ReturnStatus;
    }
    Local_ReturnStatus = MFLS_privateAddressToSector(Local_u32LastAddress, &Local_LastSector);
    if (Local_ReturnStatus != MFLS_OK)
    {
        return Local_ReturnStatus;
    }
    if (  ((u32)Local_FirstSector < (u32)MFLS_WRITABLE_SECTOR_MIN)
       || ((u32)Local_FirstSector > (u32)MFLS_WRITABLE_SECTOR_MAX)
       || ((u32)Local_LastSector  < (u32)MFLS_WRITABLE_SECTOR_MIN)
       || ((u32)Local_LastSector  > (u32)MFLS_WRITABLE_SECTOR_MAX) )
    {
        return MFLS_PROTECTED_SECTOR;
    }

    /* ---- 3. Erased-state check (config-gated) ----
       Programming only clears bits: writing over stale data ANDs the
       old and new values SILENTLY. The check runs BEFORE the unlock,
       so a rejected call writes nothing at all. */
#if MFLS_VERIFY_ERASED_BEFORE_PROGRAM
    Local_pu32Target = (volatile u32 *)Copy_u32Address;
    for (Local_u32Index = 0u; Local_u32Index < (u32)Copy_u16WordCount; Local_u32Index++)
    {
        if (Local_pu32Target[Local_u32Index] != 0xFFFFFFFFUL)
        {
            return MFLS_NOT_ERASED;
        }
    }
#endif

    /* ---- 4. Controller sequence ---- */
    if (GET_BIT(MFLS_FLASH->SR, MFLS_SR_BSY) != 0u)
    {
        return MFLS_BUSY;
    }

    Local_ReturnStatus = MFLS_privateUnlock();
    if (Local_ReturnStatus != MFLS_OK)
    {
        return Local_ReturnStatus;
    }

    (void)MFLS_privateClearStatus();

    /* PG + PSIZE(x32) set once for the whole call. PSIZE must match
       the actual store width (u32) or hardware raises PGPERR. */
    MFLS_FLASH->CR = (MFLS_FLASH->CR & ~MFLS_CR_PSIZE_MASK)
                   | (MFLS_CR_PSIZE_X32 << MFLS_CR_PSIZE)
                   | (1uL << MFLS_CR_PG);

    Local_pu32Target  = (volatile u32 *)Copy_u32Address;
    Local_ReturnStatus = MFLS_OK;

    for (Local_u32Index = 0u;
         (Local_ReturnStatus == MFLS_OK) && (Local_u32Index < (u32)Copy_u16WordCount);
         Local_u32Index++)
    {
        /* The store itself starts the programming cycle (BSY sets on
           the write). Expect a short core stall per word — flash
           instruction fetches wait for the program cycle to end. */
        Local_pu32Target[Local_u32Index] = Copy_pu32Data[Local_u32Index];

        Local_ReturnStatus = MFLS_privateWaitNotBusy();

        if (Local_ReturnStatus == MFLS_OK)
        {
            /* Per-word hygiene: an error flag aborts the run (flags
               cleared, status returned); EOP cleared so the next
               word starts from a clean SR. */
            Local_ReturnStatus = MFLS_privateClearStatus();
        }
        else
        {
            (void)MFLS_privateClearStatus();   /* timeout: clear + report */
        }
    }

    /* Disarm PG regardless of outcome. */
    CLR_BIT(MFLS_FLASH->CR, MFLS_CR_PG);

    /* Always relock before returning. */
    MFLS_privateLock();

    return Local_ReturnStatus;
}

/* =====================================================================
 *  MFLS_GetSectorBounds
 * ===================================================================== */
MFLS_ReturnType_t MFLS_GetSectorBounds(MFLS_Sector_t Copy_Sector,
                                       u32 *Copy_pu32Base,
                                       u32 *Copy_pu32Size)
{
    if ((u32)Copy_Sector > (u32)MFLS_SECTOR_5)
    {
        return MFLS_INVALID_SECTOR;
    }
    if ((Copy_pu32Base == NULL) || (Copy_pu32Size == NULL))
    {
        return MFLS_INVALID_PARAM;
    }

    *Copy_pu32Base = MFLS_AstSectors[Copy_Sector].u32Base;
    *Copy_pu32Size = MFLS_AstSectors[Copy_Sector].u32Size;

    return MFLS_OK;
}

/* =====================================================================
 *  MFLS_GetSectorOfAddress
 * ===================================================================== */
MFLS_ReturnType_t MFLS_GetSectorOfAddress(u32 Copy_u32Address,
                                          MFLS_Sector_t *Copy_pSector)
{
    if (Copy_pSector == NULL)
    {
        return MFLS_INVALID_PARAM;
    }

    return MFLS_privateAddressToSector(Copy_u32Address, Copy_pSector);
}

MFLS_ReturnType_t MFLS_GetLastStatusSnapshot(u32 *Copy_pu32Snapshot)
{
    if (Copy_pu32Snapshot == NULL)
    {
        return MFLS_INVALID_PARAM;
    }
    *Copy_pu32Snapshot = MFLS_u32LastStatusSnapshot;
    return MFLS_OK;
}
