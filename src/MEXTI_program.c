/*
 * MEXTI_program.c
 *
 *  Implementation of the MEXTI driver.
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 10: EXTI / SYSCFG
 *
 *  DESIGN PRINCIPLES (documented, deliberate):
 *
 *  1. THE DRIVER ACTUALLY FIRES. Rev 1 configured lines, unmasks IMR,
 *     stores callbacks — and defines NO vector-table handlers and
 *     enables NO NVIC line. Nothing ever called HandleLine: a button
 *     press pended a flag and the system sailed on. Every callback
 *     was dead code. Rev 2 owns the seven handlers (EXTI0..4 own IRQs,
 *     EXTI9_5 and EXTI15_10 shared) and the line->IRQ NVIC mapping.
 *
 *  2. PR SNAPSHOT DISPATCH. The shared handler reads PR ONCE at entry
 *     and services exactly the bits the snapshot shows (direct W1C
 *     store per line, then callback). Same discipline as MUSART Rev 3's
 *     SR snapshot: detection atomic, clear-by-sequence preserved.
 *
 *  3. W1C DISCIPLINE. PR is written with one-hot direct stores only.
 *     `PR |= x` (read-modify-write) would write 1 to every pending bit
 *     it read — silently eating other lines' events on shared vectors.
 *     Rev 1 already had the right idiom; Rev 2 keeps it and documents
 *     it at every PR touch.
 *
 *  4. VALIDATE FIRST, WRITE LAST. Whole-table validation (incl.
 *     duplicate-line detection via a seen-mask) before ANY register
 *     write in Init; u32-cast-before-compare on every enum (a negative
 *     enum value passes a signed ">=" range check and then produces a
 *     NEGATIVE SHIFT — undefined behavior); garbage port/trigger/state
 *     values rejected loudly.
 *
 *  5. MGPIO_Port_t REUSE. The port type's values ARE the EXTICR
 *     encodings (0/1/2) — pinned by _Static_assert, no translation
 *     layer, one port type across the library.
 *
 *  6. ASYMMETRIC ENABLE/DISABLE. Enable raises the line's NVIC IRQ too
 *     (mapped 0..4 / 5..9 / 10..15); Disable masks IMR only — the
 *     shared vectors make NVIC shutdown a shared-resource decision,
 *     not a per-line one.
 *
 *  Revision history:
 *    Rev 1 — original
 *    Rev 2 — this rewrite (see principles above; also: MEXTI_int.h ->
 *            MEXTI_interface.h rename, naming/enum sweep, SWIER trigger
 *            added, Init becomes validating + status)
 *
 *  Created on: Aug 12, 2026
 *      Author: dell
 */
#include <MCAL/EXTI/MEXTI_private.h>
#include <MCAL/EXTI/MEXTI_interface.h>
#include <MCAL/EXTI/MEXTI_config.h>
#include "MNVIC_interface.h"
#include "BIT_MATH.h"
#include "STD_TYPES.h"

/* ========================================================================= *
 *  Compile-time consistency guards
 * ========================================================================= */

/* Line enum == line number (used directly as bit position/shift count) */
_Static_assert(MEXTI_LINE0 == 0,  "line enum must equal line number");
_Static_assert(MEXTI_LINE15 == 15, "line enum must equal line number");

/* MGPIO_Port_t values ARE the EXTICR encodings — direct mapping, no
   translation layer (design principle 5) */
_Static_assert(MGPIO_PORTA == 0, "MGPIO_Port_t must match EXTICR encoding");
_Static_assert(MGPIO_PORTB == 1, "MGPIO_Port_t must match EXTICR encoding");
_Static_assert(MGPIO_PORTC == 2, "MGPIO_Port_t must match EXTICR encoding");

/* EXTI line -> NVIC IRQ mapping (ground-truthed against the startup
   vector table; these pins make the mapper arithmetic below safe) */
_Static_assert(MNVIC_EXTI0  == 6,  "vector layout: EXTI0 is IRQ 6");
_Static_assert(MNVIC_EXTI4  == 10, "vector layout: EXTI4 is IRQ 10");
_Static_assert(MNVIC_EXTI9_5  == 23, "vector layout: EXTI9_5 is IRQ 23");
_Static_assert(MNVIC_EXTI15_10 == 40, "vector layout: EXTI15_10 is IRQ 40");

/* ========================================================================= *
 *  Private state & prototypes
 * ========================================================================= */

/* Set by Init after whole-table validation — gates EnableInterrupt
   (the ARMING function; config/inspection functions stay ungated). */
static u8 Global_u8IsInitialized = 0;

/* Per-line callbacks. NULL = unregistered (events ignored). Written
   by SetCallBack (any context, aligned pointer store = atomic); read
   in ISR context — re-read every event. */
static void (*MEXTI_Callbacks[MEXTI_NUM_LINES])(void) = {NULL};

static MNVIC_IRQ_t         MEXTI_privateLineToIrq (u32 Copy_u32Line);
static void                MEXTI_privateSetTrigger(MEXTI_LineNumber_t Copy_Line,
                                                   MEXTI_Trigger_t Copy_Trigger);
static void                MEXTI_privateHandler   (u32 Copy_u32StartLine, u32 Copy_u32EndLine);

/* ========================================================================= *
 *                        INITIALIZATION                                     *
 * ========================================================================= */

MEXTI_ReturnType_t MEXTI_Init(void)
{
    u8   Local_u8Index;
    u16  Local_u16SeenLines = 0u;   /* duplicate-line detection mask */

    /* ---- PHASE 1: validate the WHOLE table before touching anything.
            Either every row is clean or NO register is written (the
            validate-first principle — a half-applied table is exactly
            the partial-config failure mode MGPIO Rev 2 eliminated). */

    for (Local_u8Index = 0u; Local_u8Index < (u8)MEXTI_NUM_CONFIGURED_LINES; Local_u8Index++)
    {
        const MEXTI_ConfigEntry_t *Local_CfgRow = &MEXTI_LinesConfig[Local_u8Index];

        /* Line range — u32 cast BEFORE compare (a negative enum value
           passes a signed >= test, then becomes a negative shift = UB) */
        if ((u32)Local_CfgRow->Line > (u32)MEXTI_LINE15)
        {
            return MEXTI_INVALID_CONFIG;
        }

        /* Port range — only A..C exist in this driver's world (MGPIO's
           port set; EXTICR has wider encodings but no port to route) */
        if ((u32)Local_CfgRow->PortSource > (u32)MGPIO_PORTC)
        {
            return MEXTI_INVALID_CONFIG;
        }

        /* Trigger range — a garbage trigger with Rev 1's silent
           `default: break` configured NO edge: a line that could
           never trigger, silently. Now the whole Init fails loudly. */
        if ((u32)Local_CfgRow->Trigger > (u32)MEXTI_TRIGGER_ON_CHANGE)
        {
            return MEXTI_INVALID_CONFIG;
        }

        /* Init state must be the defined binary */
        if ((Local_CfgRow->InitState != MEXTI_ENABLED) &&
            (Local_CfgRow->InitState != MEXTI_DISABLED))
        {
            return MEXTI_INVALID_CONFIG;
        }

        /* Duplicate line — the second row would silently OVERWRITE the
           first's routing/trigger. Detected, rejected. */
        if ((Local_u16SeenLines & MEXTI_LINE_MASK(Local_CfgRow->Line)) != 0u)
        {
            return MEXTI_INVALID_CONFIG;
        }
        Local_u16SeenLines |= (u16)MEXTI_LINE_MASK(Local_CfgRow->Line);
    }

    /* ---- PHASE 2: apply. Only after the table is proven clean. ---- */
    /* The init gate goes up before the rows are applied so Enable
       (below) passes its own check. If a row-level NVIC enable ever
       failed (unreachable with validated mapping — belt and braces),
       the error propagates with partial state — documented trade-off;
       table re-runs are idempotent, so re-Init after fixing is safe. */
    Global_u8IsInitialized = 1u;

    for (Local_u8Index = 0u; Local_u8Index < (u8)MEXTI_NUM_CONFIGURED_LINES; Local_u8Index++)
    {
        const MEXTI_ConfigEntry_t *Local_CfgRow = &MEXTI_LinesConfig[Local_u8Index];
        MEXTI_ReturnType_t Local_ReturnStatus;

        /* 1. SYSCFG EXTICR routing — the four-line formula that
              replaces Rev 1's 16-case copy-paste switch:
              register = line >> 2, field = (line & 3) * 4.
              MGPIO_Port_t values ARE the encodings (asserted above). */
        {
            u32 Local_u32ExticrIndex = MEXTI_EXTICR_INDEX(Local_CfgRow->Line);
            u32 Local_u32Field = MEXTI_EXTICR_MASK(Local_CfgRow->Line);

            MSYSCFG->EXTICR[Local_u32ExticrIndex] =
                (MSYSCFG->EXTICR[Local_u32ExticrIndex] & ~Local_u32Field)
              | ((u32)Local_CfgRow->PortSource << MEXTI_EXTICR_SHIFT(Local_CfgRow->Line));
        }

        /* 2. Trigger edges (shared helper — Rev 1 duplicated this
              switch between Init and SetTrigger; one copy now) */
        MEXTI_privateSetTrigger(Local_CfgRow->Line, Local_CfgRow->Trigger);

        /* 3. Stale-pending wipe — a PR bit set before boot (line
              floated during power-up, debugger poking) would fire the
              INSTANT the line is unmasked. Clear first. Direct W1C
              store, always. */
        (void)MEXTI_ClearFlag(Local_CfgRow->Line);

        /* 4. Enable or leave disabled per the row */
        if (Local_CfgRow->InitState == MEXTI_ENABLED)
        {
            Local_ReturnStatus = MEXTI_EnableInterrupt(Local_CfgRow->Line);
            if (Local_ReturnStatus != MEXTI_OK)
            {
                return Local_ReturnStatus;   /* unreachable path — see above */
            }
        }
        else
        {
            (void)MEXTI_DisableInterrupt(Local_CfgRow->Line);
        }
    }

    return MEXTI_OK;
}

/* ========================================================================= *
 *                        ENABLE / DISABLE                                    *
 * ========================================================================= */

MEXTI_ReturnType_t MEXTI_EnableInterrupt(MEXTI_LineNumber_t Copy_Line)
{
    if ((u32)Copy_Line > (u32)MEXTI_LINE15)
    {
        return MEXTI_INVALID_LINE;
    }

    /* Init gate — the ARMING function. Before Init, EXTICR holds its
       reset all-port-A default and no trigger is configured: arming a
       line then would fire on whatever the silicon happens to be. */
    if (Global_u8IsInitialized == 0u)
    {
        return MEXTI_NOT_INITIALIZED;
    }

    /* ORDER: clear stale pending -> unmask IMR -> enable NVIC.
       An IMR unmask over a set PR bit pends the IRQ immediately (the
       request is generated when pending AND unmasked) — so the wipe
       must come first. (This is the spurious-first-interrupt trap,
       handled here the same way Init handles it.) */
    (void)MEXTI_ClearFlag(Copy_Line);

    /* IMR is a plain RW register (no W1S alias exists) — read-modify-
       write is the correct idiom. Safe: the ISR never writes IMR; this
       is a boot/config-time function. */
    MEXTI->IMR |= MEXTI_LINE_MASK(Copy_Line);

    /* NVIC enable via the mapping. Re-enabling an enabled line is
       harmless (ISER is W1S). Status checked; with the validated
       mapping the enable cannot actually fail — belt and braces. */
    {
        MNVIC_ReturnType_t Local_EnumStatus = MNVIC_EnableInterrupt(MEXTI_privateLineToIrq((u32)Copy_Line));
        if (Local_EnumStatus != MNVIC_OK)
        {
            return MEXTI_INVALID_CONFIG;   /* unreachable — documented */
        }
    }

    return MEXTI_OK;
}

MEXTI_ReturnType_t MEXTI_DisableInterrupt(MEXTI_LineNumber_t Copy_Line)
{
    if ((u32)Copy_Line > (u32)MEXTI_LINE15)
    {
        return MEXTI_INVALID_LINE;
    }

    /* Ungated: disarming always succeeds (the MNVIC_ClearPending
       principle). IMR only — NEVER the NVIC line: for lines 5..15 the
       vector is SHARED with other lines; killing it here would
       silently murder them. NVIC shutdown is the application's
       shared-resource call (MNVIC_DisableInterrupt directly). */
    MEXTI->IMR &= ~MEXTI_LINE_MASK(Copy_Line);

    return MEXTI_OK;
}

/* ========================================================================= *
 *                        TRIGGER / CALLBACK / FLAG                          *
 * ========================================================================= */

MEXTI_ReturnType_t MEXTI_SetTrigger(MEXTI_LineNumber_t Copy_Line,
                                    MEXTI_Trigger_t    Copy_Trigger)
{
    if ((u32)Copy_Line > (u32)MEXTI_LINE15)
    {
        return MEXTI_INVALID_LINE;
    }
    if ((u32)Copy_Trigger > (u32)MEXTI_TRIGGER_ON_CHANGE)
    {
        return MEXTI_INVALID_PARAM;
    }

    /* Not init-gated: pure configuration write, safe in any state.
       A line with BOTH trigger bits clear can never pend — which is
       exactly why garbage values are rejected instead of written. */
    MEXTI_privateSetTrigger(Copy_Line, Copy_Trigger);
    return MEXTI_OK;
}

MEXTI_ReturnType_t MEXTI_SetCallBack(MEXTI_LineNumber_t Copy_Line,
                                     void (*Copy_CallBack)(void))
{
    if ((u32)Copy_Line > (u32)MEXTI_LINE15)
    {
        return MEXTI_INVALID_LINE;
    }

    /* NULL is a VALID value: UNREGISTER (library convention — an
       event with no callback is ignored, never a crash. Rev 1
       rejected NULL here, contradicting MSYSTICK/MUSART). The aligned
       pointer store is atomic; the ISR re-reads it every event. */
    MEXTI_Callbacks[Copy_Line] = Copy_CallBack;
    return MEXTI_OK;
}

MEXTI_ReturnType_t MEXTI_ClearFlag(MEXTI_LineNumber_t Copy_Line)
{
    if ((u32)Copy_Line > (u32)MEXTI_LINE15)
    {
        return MEXTI_INVALID_LINE;
    }

    /* THE W1C IDIOM — direct one-hot store, never read-modify-write.
       PR |= mask would also write 1 to every OTHER pending bit the
       read saw — eating sibling lines' events on shared vectors
       before their handler dispatches them. This single line of code
       is the whole reason Rev 1's ClearFlag was correct where most
       hand-rolled EXTI drivers are not. */
    MEXTI->PR = MEXTI_LINE_MASK(Copy_Line);

    return MEXTI_OK;
}

MEXTI_ReturnType_t MEXTI_TriggerLine(MEXTI_LineNumber_t Copy_Line)
{
    if ((u32)Copy_Line > (u32)MEXTI_LINE15)
    {
        return MEXTI_INVALID_LINE;
    }

    /* SWIER write -> sets PR for the line. If IMR + NVIC are enabled,
       the FULL dispatch path fires (IRQ -> handler -> clear ->
       callback) — the zero-hardware button test. If not enabled, only
       PR sets (no exception; visible to a later ClearFlag consumer).
       The SWIER bit itself clears when PR is cleared — nothing to
       clean up manually. Not init-gated: it arms nothing. */
    MEXTI->SWIER = MEXTI_LINE_MASK(Copy_Line);

    return MEXTI_OK;
}

/* ========================================================================= *
 *                        INTERRUPT SERVICE ROUTINES                         *
 * ========================================================================= *
 * The seven vector-table entries (strong symbols override the weak
 * Default_Handler aliases in startup_stm32f401xc.s — same linkage
 * pattern as MSPI/MUSART). Names MUST match the startup file exactly:
 * EXTI0..4 own one IRQ each; EXTI9_5 covers lines 5..9; EXTI15_10
 * covers lines 10..15. The two shared handlers dispatch per-line via
 * the PR snapshot. These are THE missing half of Rev 1 — without them
 * (and without the NVIC enable), no button press ever executed a
 * single instruction of user code.
 * ========================================================================= */

/* ---- Shared dispatch body --------------------------------------------------
 * PR SNAPSHOT: read PR once at entry; service exactly the snapshot's
 * set bits within the line range. Per set bit: direct W1C store, then
 * callback. Consequences:
 *   - an edge arriving AFTER the snapshot on a not-yet-serviced line
 *     simply sets PR again -> the IRQ re-pends -> handled next entry
 *     (flags pending after entry are never lost — the conservative
 *     snapshot property, same as MUSART Rev 3);
 *   - DOCUMENTED RESIDUAL (the few-cycle race): an edge arriving on
 *     an ALREADY-snapshot-set line between snapshot and its W1C store
 *     is consumed by that store (PR was already 1; the new edge does
 *     nothing, and the clear wipes it). Window = a few CPU cycles;
 *     button edges are human-timescale. Accepted, bounded, documented.
 *   - callbacks may safely call back into MEXTI (SetCallBack,
 *     TriggerLine) — per-line independence holds throughout.
 * -------------------------------------------------------------------------- */
static void MEXTI_privateHandler(u32 Copy_u32StartLine, u32 Copy_u32EndLine)
{
    /* ONE read of PR — the detection snapshot (reading PR does NOT
       clear it; only writes do — so the snapshot is side-effect-free,
       unlike MUSART's SR where the read itself primes a sequence) */
    u32 Local_u32Pending = MEXTI->PR;
    u32 Local_u32Line;

    for (Local_u32Line = Copy_u32StartLine; Local_u32Line <= Copy_u32EndLine; Local_u32Line++)
    {
        u32 Local_u32Mask = MEXTI_LINE_MASK(Local_u32Line);

        if ((Local_u32Pending & Local_u32Mask) != 0u)
        {
            /* Direct W1C store — the only PR write idiom in the file.
               Clearing BEFORE the callback: the callback may take
               arbitrary (ISR-budget) time, and the line must already
               be re-armed to catch a new edge during it.            */
            MEXTI->PR = Local_u32Mask;

            if (MEXTI_Callbacks[Local_u32Line] != NULL)
            {
                MEXTI_Callbacks[Local_u32Line]();
            }
        }
    }
}

/* ---- Public vector-table entries (linkage only — not API) ---- */

void EXTI0_IRQHandler(void)     { MEXTI_privateHandler(0u, 0u); }
void EXTI1_IRQHandler(void)     { MEXTI_privateHandler(1u, 1u); }
void EXTI2_IRQHandler(void)     { MEXTI_privateHandler(2u, 2u); }
void EXTI3_IRQHandler(void)     { MEXTI_privateHandler(3u, 3u); }
void EXTI4_IRQHandler(void)     { MEXTI_privateHandler(4u, 4u); }
void EXTI9_5_IRQHandler(void)   { MEXTI_privateHandler(5u, 9u); }
void EXTI15_10_IRQHandler(void) { MEXTI_privateHandler(10u, 15u); }

/* ========================================================================= *
 *                        PRIVATE HELPERS                                     *
 * ========================================================================= */

/* ---- Line -> NVIC IRQ mapping --------------------------------------------
 * Structure of EXTI's NVIC wiring (RM0368 §10.1, pinned by the
 * _Static_asserts at the top of this file):
 *      lines 0..4   : one IRQ each, positions 6..10, CONTIGUOUS —
 *                     arithmetic mapping (MNVIC_EXTI0 + line)
 *      lines 5..9   : ONE shared IRQ, position 23 (EXTI9_5)
 *      lines 10..15 : ONE shared IRQ, position 40 (EXTI15_10)
 * The mapping's failure mode if gotten wrong is quiet: enabling line
 * 7's IMR and calling MNVIC_EnableInterrupt((MNVIC_IRQ_t)7) enables
 * EXTI1's IRQ instead (position 7!) — compiles clean, does nothing
 * for line 7. The static asserts + this one function make that bug
 * unreachable. Only call with a VALIDATED line (0..15). */
static MNVIC_IRQ_t MEXTI_privateLineToIrq(u32 Copy_u32Line)
{
    if (Copy_u32Line <= 4u)
    {
        /* Contiguity asserted above: EXTI0..EXTI4 == 6..10 */
        return (MNVIC_IRQ_t)((u32)MNVIC_EXTI0 + Copy_u32Line);
    }
    else if (Copy_u32Line <= 9u)
    {
        return MNVIC_EXTI9_5;
    }
    else
    {
        return MNVIC_EXTI15_10;
    }
}

/* ---- Trigger edges (shared by Init and SetTrigger) -----------------------
 * RISING : RTSR set, FTSR clear
 * FALLING: RTSR clear, FTSR set
 * ON_CHANGE: both set (legal hardware usage — pending on either edge)
 * RTSR/FTSR are plain RW — read-modify-write is the correct idiom
 * (config-time function; the ISR never touches them). */
static void MEXTI_privateSetTrigger(MEXTI_LineNumber_t Copy_Line, MEXTI_Trigger_t Copy_Trigger)
{
    switch (Copy_Trigger)
    {
        case MEXTI_TRIGGER_RISING:
            MEXTI->RTSR |=  MEXTI_LINE_MASK(Copy_Line);
            MEXTI->FTSR &= ~MEXTI_LINE_MASK(Copy_Line);
            break;

        case MEXTI_TRIGGER_FALLING:
            MEXTI->RTSR &= ~MEXTI_LINE_MASK(Copy_Line);
            MEXTI->FTSR |=  MEXTI_LINE_MASK(Copy_Line);
            break;

        case MEXTI_TRIGGER_ON_CHANGE:
            MEXTI->RTSR |= MEXTI_LINE_MASK(Copy_Line);
            MEXTI->FTSR |= MEXTI_LINE_MASK(Copy_Line);
            break;

        default:
            /* Unreachable: every caller range-checks the enum first
               (validate-first). Kept defensive. */
            break;
    }
}
