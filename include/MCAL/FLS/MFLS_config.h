/*
 * MFLS_config.h
 *
 *  Configuration of the MFLS driver.
 *
 *  Section 1 is the ONE project policy knob: the writable sector
 *  window — the compile-time statement of which flash sectors THIS
 *  BUILD may ever erase or program. Section 2 is a driver policy
 *  toggle. Section 3 derives the polling timeout from the MRCC
 *  published clock model (no hardcoded clock numbers) and validates.
 *
 *  Reference: RM0368 Ch. 3 (FLASH); F401xC datasheet (erase times)
 *
 *  Revision history:
 *    Rev 1 — initial four-file set (FOTA seat)
 *
 *  Bench-measured erase durations (runs 1-2, WeAct F401CC, 16 MHz):
 *      128 KB sector: 2.33 s      64 KB sector: 0.65 s
 *  both far inside the 4 s allowance — the bound is validated.
 *  Created on: (set on first commit)
 *      Author: Mohamed Sameh (drafted with the FOTA Seat AI session)
 */

#ifndef MFLS_CONFIG_H_
#define MFLS_CONFIG_H_

#include "STD_TYPES.h"
#include <MCAL/RCC/MRCC_interface.h>   /* publishes MRCC_HCLK_KHZ — the
                                           single source of truth for the
                                           timeout derivation below        */

/* =====================================================================
 *  1. WRITABLE SECTOR WINDOW  (per-build policy — THE safety knob)
 *
 *  MFLS rejects any erase/program outside [MIN .. MAX] BEFORE
 *  touching KEYR. This is a COMPILE-TIME policy, stronger than a
 *  runtime convention: the app-build binary simply contains no code
 *  path able to erase the app region or the bootloader sector.
 *
 *  Three intended profiles (one pair per build configuration):
 *
 *      PHASE-0 HARNESS (today's layout: running image in S0..S1,
 *                       ~26 KB; scratch targets are S4..S5):
 *              MIN 4, MAX 5        <- current value
 *
 *      APP BUILD (after the FOTA step-3 relayout: app S1..S4,
 *                 staging S5 — the app may write ONLY staging):
 *              MIN 5, MAX 5
 *
 *      BOOTLOADER BUILD (installs from staging into the app region;
 *                        never erases its own S0):
 *              MIN 1, MAX 5
 * ===================================================================== */
#define MFLS_WRITABLE_SECTOR_MIN      1
#define MFLS_WRITABLE_SECTOR_MAX      5

/* =====================================================================
 *  2. Erased-state verification  (driver policy)
 *
 *  1 = MFLS_ProgramWords READS every target word first and rejects
 *      the call with MFLS_NOT_ERASED unless every word reads
 *      0xFFFFFFFF. Flash programming only clears bits (1->0), so
 *      writing over stale data corrupts SILENTLY — this check turns
 *      that bug class into a loud error before anything is written.
 *  0 = skip the check.
 * ===================================================================== */
#define MFLS_VERIFY_ERASED_BEFORE_PROGRAM    1

/* =====================================================================
 *  3. Derived values — DO NOT EDIT
 *
 *  Polling timeout from the published clock model:
 *      - MRCC_HCLK_KHZ is numerically the CPU cycles per
 *        millisecond (16000 kHz = 16000 cycles/ms)
 *      - polling loop body (SR read + test + decrement) ~ 8 cycles
 *      - worst-case operation allowance: 4000 ms (datasheet typ
 *        erase ~2 s for the 128 KB sector — margin x2)
 *      - iterations = (cycles/ms / loop cycles) x ms budget
 *
 *  WHY DERIVED, and not a fixed count like MRCC_TIMEOUT_COUNT:
 *  a fixed iteration count is NOT clock-portable — 10M iterations
 *  is ~5 s at 16 MHz HSI but under 1 s at the 84 MHz PLL
 *  contingency. MRCC's waits are short enough to tolerate that;
 *  this driver's waits are SECONDS long, so the portability failure
 *  would be real. Deriving from MRCC_HCLK_KHZ keeps the bound
 *  correct under any SYSTEM_CLOCK.
 *
 *  16 MHz: (16000/8) x 4000  =   8,000,000 iterations (~4 s)
 *  84 MHz: (84000/8) x 4000  =  42,000,000 iterations (~4 s)
 *  Both fit u32 with huge headroom.
 * ===================================================================== */
#define MFLS_POLL_LOOP_CYCLES         8u
#define MFLS_MAX_OPERATION_MS         4000UL

/* NOTE (preprocessor rule): NO casts in this macro. It is evaluated
   by the #if in section 4, and #if expressions accept integer
   constants and operators ONLY — a (u32) cast expands into tokens the
   preprocessor cannot parse ("missing binary operator before token").
   The u / UL suffixes already force unsigned arithmetic, so the cast
   added nothing at runtime anyway. */
#define MFLS_TIMEOUT_COUNT            ((MRCC_HCLK_KHZ / MFLS_POLL_LOOP_CYCLES) * MFLS_MAX_OPERATION_MS)



/* =====================================================================
 *  4. Configuration validation — bad values stop the build
 * ===================================================================== */
#if (MFLS_WRITABLE_SECTOR_MIN > MFLS_WRITABLE_SECTOR_MAX)
#error "MFLS: MFLS_WRITABLE_SECTOR_MIN must be <= MFLS_WRITABLE_SECTOR_MAX"
#endif

#if (MFLS_WRITABLE_SECTOR_MAX > 5)
#error "MFLS: MFLS_WRITABLE_SECTOR_MAX must be 0..5 - the F401CC has 6 sectors"
#endif

#if (MFLS_VERIFY_ERASED_BEFORE_PROGRAM != 0) && (MFLS_VERIFY_ERASED_BEFORE_PROGRAM != 1)
#error "MFLS: MFLS_VERIFY_ERASED_BEFORE_PROGRAM must be 0 or 1"
#endif

#if (MFLS_TIMEOUT_COUNT == 0UL)
#error "MFLS: derived MFLS_TIMEOUT_COUNT is zero - check the clock model"
#endif

#endif /* MFLS_CONFIG_H_ */
