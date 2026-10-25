/*
 * main.c — FOTA BOOTLOADER, V1 (FOTA seat, Project 4)
 *
 *  HOME: sector 0, 0x08000000..0x08003FFF (16 KB) — enforced by
 *  mem.ld (FLASH LENGTH = 16K): the LINKER cannot place BL code
 *  outside S0. If this ever outgrows 16 KB the BUILD breaks loudly
 *  instead of the BL silently spilling into the app region.
 *
 *  THE WHOLE JOB:
 *      reset -> is a complete, verified image waiting in staging
 *      (S5, magic word planted)? -> install it into the app region
 *      (S1..S4) -> boot it.  Otherwise -> boot the app already there.
 *
 *  DELIBERATELY ABSENT (each absence is a safety choice):
 *    - No SysTick, no interrupts of any kind. Nothing to init,
 *      nothing to storm; the NVIC is all-disabled from reset and
 *      this code never arms a single source.
 *    - No UART. Silent by design: PC13 blink codes are the only
 *      output; SWD is always available for recovery.
 *    - No MRCC_Init: the reset default (16 MHz HSI) is exactly what
 *      MFLS's compile-time timeout derivation assumes.
 *    - No staging erase on the normal path — the ~2.3 s sector erase
 *      is paid once per UPDATE (and once, self-healing, on bad
 *      staging), never on an ordinary boot.
 *
 *  BOOT FLOW:
 *      1. PC13 output, OFF (happy path: the LED never lights)
 *      2. magic at the staging trailer?  no  -> jump to app
 *      3. trailer checks: version, length bounds, trailer self-CRC
 *      4. staged-image end-to-end CRC — re-verified HERE (trust
 *         nothing, including our own receiver)
 *      5. staged SP/PC sanity — checked BEFORE the old app is
 *         erased: a CRC-valid file that is not an app must be
 *         DECLINED, not installed over a working app.
 *         any failure of 3..5 -> erase S5 (clears the bad magic)
 *                                -> jump to the intact old app
 *      6. LED ON: erase S1..S4 -> program the image -> CRC the
 *         newly-written app region. Any failure -> fast-blink HALT.
 *      7. erase S5 — the commit's LAST act: the new app is written
 *         and verified; clearing the magic makes it permanent.
 *      8. LED OFF -> jump to the new app.
 *
 *  DESIGN CORRECTION vs the earlier sketch (recorded): the magic is
 *  cleared by ERASING S5, not by programming 0x00000000 over it —
 *  MFLS's erased-state guard rejects programming a non-erased word.
 *  The F401 has no sub-sector erase, so the whole sector goes.
 *
 *  POWER-CUT TABLE (one failure, any instant — every row ends in a
 *  bootable app or an automatic re-commit on the next boot):
 *      during download            no magic yet     -> old app boots
 *      after magic, before erase  magic intact     -> re-commit next
 *                                                     boot (idempotent)
 *      during erase/program       magic intact,    -> re-commit
 *                                 staging untouched
 *      after program, pre-verify  magic intact     -> re-commit
 *      during the S5 erase        new app verified -> boots (worst
 *                                                     case: one more
 *                                                     re-commit)
 *      after S5 erase             no magic         -> new app boots
 *  The only unrecoverable states need TWO independent failures.
 *
 *  MFLS window in this build: sectors 1..5 — this binary cannot
 *  erase or program its own S0, by construction.
 *
 *  PC13 meaning: OFF = normal; ON solid = committing (~2..4 s,
 *  board frozen — the single-bank stall contract); fast blink =
 *  commit failed: erase, program or verify (power-cycling retries
 *  the commit; SWD recovery if it repeats); slow blink = the app's
 *  reset vectors are invalid (SP or PC out of range) and no update
 *  is pending; only the vectors are checked, not an app CRC.
 */

#include "STD_TYPES.h"

#include <MCAL/RCC/MRCC_interface.h>
#include <MCAL/GPIO/MGPIO_interface.h>
#include <MCAL/FLS/MFLS_interface.h>
#include <MCAL/CRC/MCRC_interface.h>
#include "MNVIC_interface.h"   /* VTOR setter only (ungated — by design) */

/* ---- layout facts (S5 geometry comes from MFLS at runtime) ---- */
#define BL_APP_BASE             (0x08004000UL)
#define BL_APP_SIZE             (112UL * 1024UL)   /* S1..S4            */
#define BL_APP_END              (BL_APP_BASE + BL_APP_SIZE)
#define BL_RAM_BASE             (0x20000000UL)
#define BL_RAM_END              (0x20010000UL)      /* 64 KB SRAM top    */
#define BL_TRAILER_SIZE         32u
#define BL_MAGIC                (0x464F5441UL)      /* "FOTA"            */
#define BL_TRAILER_VERSION      2u
#define BL_MIN_IMAGE_BYTES      8u                  /* SP + PC, no less  */
#define BL_HALT_BLINK_FAST_MS   80u
#define BL_HALT_BLINK_SLOW_MS   300u

/* =====================================================================
 *  PC13 helpers + crude delay (NO SysTick in this binary, ever)
 * ===================================================================== */
static void BL_privateLedOn(void)
{
    (void)MGPIO_SetPinValue(MGPIO_PORTC, MGPIO_PIN_13, MGPIO_PIN_LOW);
}

static void BL_privateLedOff(void)
{
    (void)MGPIO_SetPinValue(MGPIO_PORTC, MGPIO_PIN_13, MGPIO_PIN_HIGH);
}

/* ~4-5 cycles per iteration at 16 MHz HSI, 0 flash wait states ->
   ~3200 iterations per ms. Rough BY DESIGN: blink codes need no
   precision. Never use this for timing logic. */
static void BL_privateDelayMs(u32 Copy_u32Ms)
{
    volatile u32 Local_u32Count = Copy_u32Ms * 3200u;
    while (Local_u32Count != 0u)
    {
        Local_u32Count--;
    }
}

/* Never returns. */
static void BL_privateHaltBlink(u32 Copy_u32PeriodMs)
{
    while (1)
    {
        BL_privateLedOn();   BL_privateDelayMs(Copy_u32PeriodMs);
        BL_privateLedOff();  BL_privateDelayMs(Copy_u32PeriodMs);
    }
}

/* =====================================================================
 *  Jump to the app at 0x08004000
 *
 *  Sanity gates first: an app region of 0xFFFFFFFF (nothing flashed)
 *  or a trashed vector table must NOT be jumped into — slow-blink
 *  halt instead ("no valid app": flash one via SWD).
 *
 *  The BL never enables a single interrupt, so the app starts with a
 *  clean NVIC (system reset cleared it) and PRIMASK untouched (=0).
 *  VTOR is set HERE so any fault during app startup vectors to the
 *  app's own handlers; the app's main sets it again as its first
 *  statement (redundant, and covers standalone debugging).
 * ===================================================================== */
static void BL_privateJumpToApp(void)
{
    u32 Local_u32Sp = *(volatile u32 *)BL_APP_BASE;
    u32 Local_u32Pc = *(volatile u32 *)(BL_APP_BASE + 4u);

    if (  (Local_u32Sp < BL_RAM_BASE) || (Local_u32Sp > BL_RAM_END)
       || ((Local_u32Sp & 0x7u) != 0u) )                 /* 8-aligned */
    {
        BL_privateHaltBlink(BL_HALT_BLINK_SLOW_MS);
    }
    if (  (Local_u32Pc < BL_APP_BASE) || (Local_u32Pc >= BL_APP_END)
       || ((Local_u32Pc & 1u) == 0u) )                   /* Thumb bit */
    {
        BL_privateHaltBlink(BL_HALT_BLINK_SLOW_MS);
    }

    (void)MNVIC_SetVectorTableBase(BL_APP_BASE);

    /* Set the app's initial stack pointer, then branch to its reset
       handler — in ONE asm block so nothing the compiler generates
       can touch the (already-replaced) stack in between. */
    asm volatile
    (
        "msr msp, %0        \n"
        "bx   %1            \n"
        :
        : "r" (Local_u32Sp), "r" (Local_u32Pc)
    );

    while (1) { }   /* unreachable — keeps the compiler honest */
}

/* =====================================================================
 *  The commit. Called ONLY after staging passed every check.
 *  Halts (fast blink) on hard failure; returns only on success.
 * ===================================================================== */
static void BL_privateCommit(u32 Copy_u32StagingBase,
                             u32 Copy_u32Length,
                             u32 Copy_u32E2eCrc)
{
    u32 Local_u32Padded = (Copy_u32Length + 3u) & ~3u;
    u32 Local_u32Words  = Local_u32Padded / 4u;
    u32 Local_u32Crc = 0u;
    u32 Local_u32Sector;

    BL_privateLedOn();   /* visible "working" — freezes during stalls */

    /* 1. Erase the app region, S1..S4, in order. From the first STRT
          until the last verify, the old app is being replaced —
          power cuts here are covered by the magic still being set. */
    for (Local_u32Sector = 1u; Local_u32Sector <= 4u; Local_u32Sector++)
    {
        if (MFLS_EraseSector((MFLS_Sector_t)Local_u32Sector) != MFLS_OK)
        {
            BL_privateHaltBlink(BL_HALT_BLINK_FAST_MS);
        }
    }

    /* 2. Program the image: staging -> app region (flash-to-flash;
          MFLS reads source words between the per-word program
          cycles — reads are legal whenever BSY is low). */
    if (MFLS_ProgramWords(BL_APP_BASE,
                          (const u32 *)Copy_u32StagingBase,
                          (u16)Local_u32Words) != MFLS_OK)
    {
        BL_privateHaltBlink(BL_HALT_BLINK_FAST_MS);
    }

    /* 3. Verify what is NOW in the app region — the same CRC the
          sender computed, over the same padded footprint. */
    if (  (MCRC_CalcBuffer((const u8 *)(void *)BL_APP_BASE,
                           Local_u32Padded, &Local_u32Crc) != MCRC_OK)
       || (Local_u32Crc != Copy_u32E2eCrc) )
    {
        BL_privateHaltBlink(BL_HALT_BLINK_FAST_MS);
    }

    /* 4. THE LAST ACT: erase staging — clears the magic. The new
         app is written and verified; from this instant it IS the
         app. If this erase fails (a near-impossible controller
         state): NOT fatal — the magic survives and the next boot
         simply re-commits the identical image. Self-healing beats
         halting. */
    (void)MFLS_EraseSector(MFLS_SECTOR_5);

    BL_privateLedOff();
}

/* =====================================================================
 *  main — the whole bootloader decision tree
 * ===================================================================== */
void main(void)
{
    u32 Local_u32StagingBase = 0u;
    u32 Local_u32StagingSize = 0u;
    u32 Local_u32TrailerAddr;
    u32 Local_u32Calc = 0u;
    u8  Local_u8StagingGood = 1u;

    /* ---- 1. minimal bring-up: PC13 only ---- */
    (void)MRCC_EnablePeripheralClock(MRCC_GPIOC);
    (void)MGPIO_SetPinMode        (MGPIO_PORTC, MGPIO_PIN_13, MGPIO_MODE_OUTPUT);
    (void)MGPIO_SetPinOutputConfig(MGPIO_PORTC, MGPIO_PIN_13, MGPIO_OUTPUT_PUSH_PULL, MGPIO_SPEED_LOW);
    BL_privateLedOff();

    /* ---- 2. staging geometry from the driver (no hardcodes) ---- */
    if (MFLS_GetSectorBounds(MFLS_SECTOR_5, &Local_u32StagingBase, &Local_u32StagingSize) != MFLS_OK)
    {
        BL_privateHaltBlink(BL_HALT_BLINK_SLOW_MS);  /* impossible: pinned by asserts */
    }
    Local_u32TrailerAddr = Local_u32StagingBase + Local_u32StagingSize - BL_TRAILER_SIZE;

    /* ---- 3. pending update? ---- */
    if (*(volatile u32 *)Local_u32TrailerAddr == BL_MAGIC)
    {
        u32 Local_u32TrailerVersion = *(volatile u32 *)(Local_u32TrailerAddr +  4u);
        u32 Local_u32AppVersion     = *(volatile u32 *)(Local_u32TrailerAddr +  8u);
        u32 Local_u32Length         = *(volatile u32 *)(Local_u32TrailerAddr + 12u);
        u32 Local_u32E2eCrc         = *(volatile u32 *)(Local_u32TrailerAddr + 16u);
        u32 Local_u32TrailerCrc     = *(volatile u32 *)(Local_u32TrailerAddr + 20u);

        (void)Local_u32AppVersion;    /* future: display / downgrade policy */
        (void)MCRC_Init();            /* enables the CRC unit clock */

        /* Check 1: trailer self-CRC (words 1..4) */
        if (  (MCRC_CalcBuffer((const u8 *)(void *)(Local_u32TrailerAddr + 4u),
                               16u, &Local_u32Calc) != MCRC_OK)
           || (Local_u32Calc != Local_u32TrailerCrc) )
        {
            Local_u8StagingGood = 0u;
        }

        /* Check 2: field values */
        if (Local_u32TrailerVersion != BL_TRAILER_VERSION)
        {
            Local_u8StagingGood = 0u;
        }
        if (  (Local_u32Length < BL_MIN_IMAGE_BYTES)
           || (Local_u32Length > BL_APP_SIZE) )
        {
            Local_u8StagingGood = 0u;
        }

        /* Check 3: staged image end-to-end CRC (re-verified here) */
        if (Local_u8StagingGood != 0u)
        {
            if (  (MCRC_CalcBuffer((const u8 *)(void *)Local_u32StagingBase,
                                   (Local_u32Length + 3u) & ~3u,
                                   &Local_u32Calc) != MCRC_OK)
               || (Local_u32Calc != Local_u32E2eCrc) )
            {
                Local_u8StagingGood = 0u;
            }
        }

        /* Check 4: staged image must LOOK like an app (SP/PC sane)
           BEFORE the old app is destroyed */
        if (Local_u8StagingGood != 0u)
        {
            u32 Local_u32Sp = *(volatile u32 *)Local_u32StagingBase;
            u32 Local_u32Pc = *(volatile u32 *)(Local_u32StagingBase + 4u);

            if (  (Local_u32Sp < BL_RAM_BASE) || (Local_u32Sp > BL_RAM_END)
               || ((Local_u32Sp & 0x7u) != 0u)
               || (Local_u32Pc < BL_APP_BASE) || (Local_u32Pc >= BL_APP_END)
               || ((Local_u32Pc & 1u) == 0u) )
            {
                Local_u8StagingGood = 0u;
            }
        }

        if (Local_u8StagingGood != 0u)
        {
            BL_privateCommit(Local_u32StagingBase, Local_u32Length, Local_u32E2eCrc);
            /* falls through to the jump — into the NEW app */
        }
        else
        {
            /* Bad staging: wipe it (clears the magic) and boot the
               intact old app. One-time ~2.3 s cost, self-healing. */
            (void)MFLS_EraseSector(MFLS_SECTOR_5);
        }
    }

    /* ---- 4. boot (the 99% path: this ran within microseconds) ---- */
    BL_privateJumpToApp();
}
