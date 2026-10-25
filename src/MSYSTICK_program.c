/*
 * MSYSTICK_program.c
 *
 *  Implementation of the MSYSTICK driver.
 *
 *  Reference: ARM PM0214 (Cortex-M4 Devices Generic User Guide) §4.4
 *
 *  DESIGN PRINCIPLES (documented, deliberate):
 *
 *  1. THE TIMEBASE IS FREE-RUNNING. LOAD is programmed once for 1 ms
 *     and hardware auto-reloads forever. No function ever reprograms
 *     LOAD/VAL, so delays can never disturb the scheduler tick — the
 *     old-style "reprogram SysTick per delay" bug class cannot occur.
 *
 *  2. DELAYS MEASURE, NEVER ASSUME — and the us path measures TRUE
 *     time. Every wait is a delta against the running counters,
 *     computed with unsigned/64-bit arithmetic. Interrupt load can
 *     stretch a delay. The us path reads through a wrap-aware reader
 *     (MSYSTICK_privateReadTotalTicks) so wraps the handler has not
 *     yet counted are COMPENSATED rather than papered over: the Rev 2
 *     "monotonic guard" bounded, but did not eliminate, an early exit
 *     of up to one wrap period (1 ms) — a wait whose start sample
 *     landed in a pending-wrap window banked one phantom wrap period
 *     and could exit 1 ms early. Readings are now true, so plain
 *     subtraction is correct.
 *
 *  3. ONE CTRL WRITE. Every CTRL modification is a full-word store
 *     with the value pre-built (every READ of CTRL clears COUNTFLAG —
 *     read-modify-write on CTRL is a flag thief; see HasCounted).
 *
 *  4. SINGLE WRITER. Global_u32MilliTicks has exactly one writer —
 *     the exception handler — and is file-static: the getter and the
 *     callback are the only public interfaces to the timebase.
 *
 *  Revision history:
 *    Rev 1 — original implementation
 *    Rev 2 — TICKINT fix; single CTRL write; SHPR3 priority; the
 *            monotonic-guard us delay (since found insufficient)
 *    Rev 3 — wrap-aware total-tick reader replaces the seqlock +
 *            monotonic guard; us delays now never early; see
 *            MSYSTICK_privateReadTotalTicks / MSYSTICK_SetBusyWait_us
 *
 *  Created on: Jul 29, 2026
 *      Author: dell
 */

#include <MCAL/SYSTICK/MSYSTICK_config.h>
#include <MCAL/SYSTICK/MSYSTICK_interface.h>
#include <MCAL/SYSTICK/MSYSTICK_private.h>
#include "BIT_MATH.h"
#include "STD_TYPES.h"

/* =====================================================================
 *  Private helper prototypes
 * ===================================================================== */

/* One seqlock-consistent (ms counter, VAL) raw sample. */
static void MSYSTICK_privateReadRaw(u32 *Copy_pu32Ms, u32 *Copy_pu32Val);

/* TRUE total ticks — including wraps the handler has not counted. */
static u64 MSYSTICK_privateReadTotalTicks(u32 *Copy_pu32LastMs,
                                          u32 *Copy_pu32LastVal,
                                          u32 *Copy_pu32ExtraWraps);

/* =====================================================================
 *  Compile-time consistency guards
 * ===================================================================== */

/* The 1 ms reload must fit the 24-bit LOAD register. Uses the model
   from config, so this self-checks for every future clock config. */
_Static_assert((MSYSTICK_SYSTICK_INPUT_FREQ_HZ / 1000UL) <= (MSYSTICK_MAX_LOAD_VALUE + 1UL),
               "MSYSTICK: 1 ms reload exceeds the 24-bit SysTick LOAD register");

/* =====================================================================
 *  Module state (all file-private)
 * ===================================================================== */

/* Free-running millisecond count. THE timebase of the whole project.
   One writer: SysTick_Handler. Readers: the getter, the delay loops,
   the seqlock in ReadRaw. Wraps every ~49.7 days — use subtraction. */
static volatile u32 Global_u32MilliTicks = 0;

/* Input ticks per 1 ms — written by Init, read-only afterwards.
   Non-zero is the precondition behind every timing computation, which
   is why Global_u8IsInitialized gates every public timing function. */
static u32 Global_u32TicksPerMs = 0;

/* Set LAST in Init — the contract that lets other functions trust
   Global_u32TicksPerMs and the running counter. */
static u8 Global_u8IsInitialized = 0;

/* Single 1 ms callback slot. NULL = none registered. Read from ISR
   context; written at init time — the assignment is atomic (aligned
   pointer store), so no disable-interrupt dance is needed. */
static void (*Global_pfTickCallback)(void) = NULL;

/* =====================================================================
 *  MSYSTICK_Init
 * ===================================================================== */
MSYSTICK_ReturnType_t MSYSTICK_Init(void)
{
    u32 Local_u32Ctrl;

    /* Reject re-init: a second pass would rewind Global_u32MilliTicks
       to 0 — every (Now - Last) in the system would instantly see a
       ~49.7-day jump. Nothing is touched on rejection. */
    if (Global_u8IsInitialized != 0u)
    {
        return MSYSTICK_ALREADY_INITIALIZED;
    }

    Global_u32TicksPerMs = MSYSTICK_SYSTICK_INPUT_FREQ_HZ / 1000UL;

    /* Build the complete control word FIRST, write it ONCE at the end:
       no intermediate half-configured states, and no read-modify-write
       on CTRL (any CTRL read clears COUNTFLAG — the flag thief). */
    Local_u32Ctrl = (1u << MSYSTICK_CTRL_ENABLE)    /* start the counter   */
                  | (1u << MSYSTICK_CTRL_TICKINT);  /* THE missing bit: the
                                                     exception that makes
                                                     the ms counter run  */
#if MSYSTICK_CLOCK_SOURCE == MSYSTICK_CLK_HCLK
    Local_u32Ctrl |= (1u << MSYSTICK_CTRL_CLKSOURCE);
#endif

    /* 1. Stop the counter first — a full-word store, so COUNTFLAG is
          not consumed (writes to CTRL do not clear it; reads do). */
    SYSTICK->CTRL = 0u;

    /* 2. Periodic 1 ms reload: LOAD = ticks - 1 (the register holds
          the RELOAD value; the counter counts LOAD..1..0 then reloads
          — the classic off-by-one, handled). Hardware reloads forever:
          the timer never needs re-arming. */
    SYSTICK->LOAD = Global_u32TicksPerMs - 1UL;

    /* 3. Zero the current value. Side effect (PM0214): writing VAL
          also clears CTRL.COUNTFLAG — deliberate flag hygiene so no
          stale wrap from a previous boot state is visible. */
    SYSTICK->VAL = 0u;

    /* 4. Exception priority — BEFORE enabling. SysTick is system
          exception 15: its priority byte is SHPR3[31:24] in the SCB,
          not an NVIC register. RMW preserves PendSV's field (bits
          23:16). Priority 15 = lowest by default: EXTI/USART/timer
          ISRs preempt the scheduler tick, never the reverse.
          The logical 0..15 value is shifted into the implemented
          upper nibble (F401: 4 priority bits).
          (When MNVIC grows system-handler support, this write moves
          there — all priorities from one place.) */
    {
        u32 Local_u32Shpr3 = MSYSTICK_SCB_SHPR3;
        Local_u32Shpr3 &= ~(0xFFu << MSYSTICK_SHPR3_SYSTICK_BYTE_POS);
        Local_u32Shpr3 |= (((u32)MSYSTICK_INTERRUPT_PRIORITY
                            << MSYSTICK_PRIORITY_REGISTER_SHIFT)
                           << MSYSTICK_SHPR3_SYSTICK_BYTE_POS);
        MSYSTICK_SCB_SHPR3 = Local_u32Shpr3;
    }

    /* 5. Single store: counter + exception + clock source all live. */
    SYSTICK->CTRL = Local_u32Ctrl;

    /* Last — only now may any other function trust the timebase. */
    Global_u8IsInitialized = 1u;

    return MSYSTICK_OK;
}

/* =====================================================================
 *  MSYSTICK_InterruptEnable / Disable
 *
 *  TICKINT only — the counter itself keeps running. See the interface
 *  header for the timebase-loss contract while disabled.
 *
 *  These are the ONLY CTRL read-modify-writes in the driver: each one
 *  reads CTRL, and any CTRL read clears COUNTFLAG — a side effect on
 *  MSYSTICK_HasCounted(). Documented, accepted: these functions are
 *  for occasional critical sections, not routine use.
 * ===================================================================== */
MSYSTICK_ReturnType_t MSYSTICK_InterruptEnable(void)
{
    if (Global_u8IsInitialized == 0u)
    {
        return MSYSTICK_NOT_INITIALIZED;
    }
    SET_BIT(SYSTICK->CTRL, MSYSTICK_CTRL_TICKINT);
    return MSYSTICK_OK;
}

MSYSTICK_ReturnType_t MSYSTICK_InterruptDisable(void)
{
    if (Global_u8IsInitialized == 0u)
    {
        return MSYSTICK_NOT_INITIALIZED;
    }
    CLR_BIT(SYSTICK->CTRL, MSYSTICK_CTRL_TICKINT);
    return MSYSTICK_OK;
}

/* =====================================================================
 *  MSYSTICK_privateReadRaw  (private helper)
 *
 *  One seqlock-consistent (ms counter, VAL) sample: read ms, read
 *  VAL, re-read ms; retry if the handler ran mid-sequence (VAL would
 *  belong to the wrong millisecond — a torn sample). The retry loop
 *  is bounded by interrupt latency, not by the tick rate.
 *
 *  What this does NOT catch (and cannot): a wrap that happened while
 *  the handler has not run yet. In that window ms and VAL are both
 *  self-consistent — they are merely STALE TOGETHER, and the sample
 *  reads one wrap period low. That gap is closed by the caller below,
 *  not by the seqlock.
 * ===================================================================== */
static void MSYSTICK_privateReadRaw(u32 *Copy_pu32Ms, u32 *Copy_pu32Val)
{
    u32 Local_u32Ms1;
    u32 Local_u32Ms2;

    do
    {
        Local_u32Ms1  = Global_u32MilliTicks;
        *Copy_pu32Val = SYSTICK->VAL;               /* volatile: re-read */
        Local_u32Ms2  = Global_u32MilliTicks;
    } while (Local_u32Ms1 != Local_u32Ms2);

    *Copy_pu32Ms = Local_u32Ms1;
}

/* =====================================================================
 *  MSYSTICK_privateReadTotalTicks  (private helper — the Rev 3 core)
 *
 *  TRUE total ticks — including wraps the handler has not yet
 *  counted. Called repeatedly by the busy-wait loop with its call-
 *  local history threaded through by pointer (no shared state, no
 *  cross-context races).
 *
 *  A reload is directly observable in the raw state: VAL only ever
 *  INCREASES across a reload (it counts down otherwise). Between
 *  consecutive samples (the loop samples every few us — far below
 *  the 1 ms wrap period):
 *    - ms advanced          -> the handler ran and counted that wrap
 *    - ms static, VAL up    -> wrap happened, handler has NOT counted
 *                              it -> compensate (ExtraWraps++)
 *    - ms static, VAL down  -> normal progress within the ms
 *
 *  The ms-advanced branch is checked FIRST (the else-if is
 *  load-bearing): if one interval contains BOTH the wrap and its
 *  handler run, VAL may still read as "up" — but the wrap is already
 *  counted in ms, and compensating it again would read one wrap HIGH.
 *  Conversely, ExtraWraps is decremented (clamped at zero) whenever
 *  ms advances, so a compensated wrap is never counted twice: one
 *  pending wrap becomes one counted wrap, exactly once.
 *
 *  Consequences of TRUE readings:
 *    - the us busy-wait needs only naive subtraction (Rev 2's
 *      accumulate/skip guard is gone)
 *    - us delays keep correct time even while the handler is starved
 *      by higher-priority ISRs or masked (PRIMASK/BASEPRI critical
 *      sections) — each wrap in the blackout is observed and
 *      compensated. The GLOBAL ms counter still loses that time
 *      (pending exceptions coalesce into one firing) — that loss is
 *      documented in the interface; this reader simply does not
 *      depend on the global.
 *
 *  Degenerate mode (documented bound): the compensation is exact
 *  while the caller samples at least once per wrap period —
 *  guaranteed here, since the loop samples every few us. If a single
 *  interval ever spans a full wrap period or more (debugger stall,
 *  >1 ms lockout), one wrap can go uncompensated — an error bounded
 *  by one wrap period, and always in the direction that makes a wait
 *  run LONG, never short. Normal operation cannot produce this.
 * ===================================================================== */
static u64 MSYSTICK_privateReadTotalTicks(u32 *Copy_pu32LastMs,
                                          u32 *Copy_pu32LastVal,
                                          u32 *Copy_pu32ExtraWraps)
{
    u32 Local_u32Ms;
    u32 Local_u32Val;
    u32 Local_u32SubMsTicks;

    MSYSTICK_privateReadRaw(&Local_u32Ms, &Local_u32Val);

    if (Local_u32Ms != *Copy_pu32LastMs)
    {
        /* Handler ran since the last sample: one pending wrap (if
           any) became counted — consume it. Clamp at zero. */
        if (*Copy_pu32ExtraWraps != 0u)
        {
            (*Copy_pu32ExtraWraps)--;
        }
    }
    else if (Local_u32Val > *Copy_pu32LastVal)
    {
        /* Reload happened, handler has not counted it — compensate */
        (*Copy_pu32ExtraWraps)++;
    }
    /* else: VAL descended within the same ms — normal progress */

    /* History for the next call — every sample in the chain */
    *Copy_pu32LastMs  = Local_u32Ms;
    *Copy_pu32LastVal = Local_u32Val;

    /* VAL counts DOWN from TicksPerMs-1: sub-ms progress is how far
       it has descended. VAL <= LOAD always (RM0368/PM0214), so this
       cannot underflow. */
    Local_u32SubMsTicks = (Global_u32TicksPerMs - 1UL) - Local_u32Val;

    return ((u64)(Local_u32Ms + *Copy_pu32ExtraWraps) * (u64)Global_u32TicksPerMs)
           + (u64)Local_u32SubMsTicks;
}

/* =====================================================================
 *  MSYSTICK_SetBusyWait_us
 *
 *  64-bit, multiply-before-divide: (us * Freq) / 1e6 — exact for every
 *  input frequency (a precomputed 32-bit ticks-per-us would truncate
 *  for fractional-MHz inputs).
 *
 *  REV 3 REDESIGN: the readings are TRUE (wrap-aware), so plain
 *  subtraction is correct — no accumulate/skip guard. One residual
 *  corner needs one cheap check:
 *
 *  STALE-START CHECK: the history seed (ReadRaw below) can itself
 *  land inside a pending-wrap window — before any wrap transition
 *  has been observed — so the FIRST reading can be one wrap period
 *  low. That error is exactly one wrap period (the ms count is stale
 *  by one; sub-ms progress is live), so Start-read-one-low makes
 *        Now - Start = true_progress + TPM  >=  TPM
 *  between two samples taken microseconds apart — which true
 *  progress can never be. A delta of a full wrap period between
 *  adjacent samples can only mean a stale start (or a debugger stall
 *  between them, which re-baselines in the LONG direction too).
 *  Detect, discard, re-baseline.
 *
 *  Contract (normal operation): never EARLY; late by at most one
 *  polling iteration (~2-4 us at 16 MHz) plus interrupt latency.
 * ===================================================================== */
MSYSTICK_ReturnType_t MSYSTICK_SetBusyWait_us(u32 Copy_u32Us)
{
    u32 Local_u32LastMs;
    u32 Local_u32LastVal;
    u32 Local_u32ExtraWraps = 0u;
    u64 Local_u64TargetTicks;
    u64 Local_u64Start;
    u64 Local_u64Now;

    if (Global_u8IsInitialized == 0u)
    {
        return MSYSTICK_NOT_INITIALIZED;   /* TicksPerMs would be 0 — no waiting */
    }
    if (Copy_u32Us == 0UL)
    {
        return MSYSTICK_OK;
    }

    Local_u64TargetTicks = ((u64)Copy_u32Us * (u64)MSYSTICK_SYSTICK_INPUT_FREQ_HZ) / 1000000ULL;

    /* Seed the wrap-detection history with one raw consistent read.
       If the seed lands inside a pending-wrap window, the first
       ReadTotalTicks cannot know (no transition observed yet) —
       that is exactly the stale-start case the check below catches. */
    MSYSTICK_privateReadRaw(&Local_u32LastMs, &Local_u32LastVal);

    Local_u64Start = MSYSTICK_privateReadTotalTicks(&Local_u32LastMs, &Local_u32LastVal,
                                                    &Local_u32ExtraWraps);
    Local_u64Now   = MSYSTICK_privateReadTotalTicks(&Local_u32LastMs, &Local_u32LastVal,
                                                    &Local_u32ExtraWraps);

    /* Two samples microseconds apart cannot differ by a full wrap
       period through true progress — only through a stale start.
       Discard it; the wait then measures from now (runs slightly
       LONG, never short). */
    if ((Local_u64Now - Local_u64Start) >= (u64)Global_u32TicksPerMs)
    {
        Local_u64Start = Local_u64Now;
    }

    /* Readings are TRUE and non-decreasing — plain subtraction */
    while ((Local_u64Now - Local_u64Start) < Local_u64TargetTicks)
    {
        Local_u64Now = MSYSTICK_privateReadTotalTicks(&Local_u32LastMs, &Local_u32LastVal,
                                                      &Local_u32ExtraWraps);
        asm("NOP");     /* keep the loop body non-empty for the optimizer */
    }

    return MSYSTICK_OK;
}

/* =====================================================================
 *  MSYSTICK_SetBusyWait_ms
 *
 *  Waits on the ms counter alone — immune to every wrap race by
 *  construction (Global_u32MilliTicks only ever increases, one writer).
 *  Unsigned subtraction handles the u32 wrap: if the counter rolls
 *  over mid-wait, (Now - Start) is still the true elapsed count.
 *  Not affected by the Rev 3 redesign — it never needed it.
 * ===================================================================== */
MSYSTICK_ReturnType_t MSYSTICK_SetBusyWait_ms(u32 Copy_u32Ms)
{
    u32 Local_u32Start;

    if (Global_u8IsInitialized == 0u)
    {
        return MSYSTICK_NOT_INITIALIZED;
    }
    if (Copy_u32Ms == 0UL)
    {
        return MSYSTICK_OK;
    }

    Local_u32Start = Global_u32MilliTicks;
    while ((Global_u32MilliTicks - Local_u32Start) < Copy_u32Ms)
    {
        asm("NOP");
    }

    return MSYSTICK_OK;
}

/* =====================================================================
 *  MSYSTICK_GetElapsed_ms
 * ===================================================================== */
u32 MSYSTICK_GetElapsed_ms(void)
{
    /* Aligned volatile u32 read — atomic against the handler's
       increment; no torn reads possible. 0 before Init is the true
       value at t = 0, hence no status return for this getter. */
    return Global_u32MilliTicks;
}

/* =====================================================================
 *  MSYSTICK_SetTickCallback
 * ===================================================================== */
MSYSTICK_ReturnType_t MSYSTICK_SetTickCallback(void (*Copy_pfCallback)(void))
{
    /* Single slot, overwrite semantics — see the interface contract.
       NULL (disable) is a valid value, not an error. The aligned
       pointer store is atomic; the handler re-reads it every tick. */
    Global_pfTickCallback = Copy_pfCallback;

    if (Global_u8IsInitialized == 0u)
    {
        return MSYSTICK_NOT_INITIALIZED;   /* stored, fires after Init */
    }
    return MSYSTICK_OK;
}

/* =====================================================================
 *  MSYSTICK_HasCounted
 * ===================================================================== */
u8 MSYSTICK_HasCounted(void)
{
    /* COUNTFLAG: set on counter reach-0, cleared by ANY read of CTRL
       (this read) or any write to VAL. One consumption per wrap —
       whoever reads first wins. Reading CTRL of a stopped SysTick is
       harmless (flag simply 0), hence no init check here.
       The Rev 3 busy-wait loop reads only VAL and the ms counter —
       it never touches CTRL, so it never steals this flag. */
    return (u8)GET_BIT(SYSTICK->CTRL, MSYSTICK_CTRL_COUNTFLAG);
}

/* =====================================================================
 *  SysTick_Handler
 *
 *  The symbol name MUST match the vector-table entry in the startup
 *  file exactly — the weak default in startup_stm32f401xc.s is
 *  overridden by this definition.
 *
 *  Runs at the configured priority (default 15 = lowest): any
 *  peripheral ISR may preempt it. Total budget for this handler:
 *  increment + callback. Everything heavier belongs in the scheduler,
 *  at thread context.
 *
 *  Pending-coalescing note: if the handler is starved across several
 *  wraps (higher-priority ISR, PRIMASK/BASEPRI), only ONE exception
 *  fires on un-block — ms advances by 1, not by the number of wraps.
 *  The global ms counter loses that time (documented); the us-delay
 *  reader compensates wraps independently and is unaffected.
 * ===================================================================== */
void SysTick_Handler(void)
{
    /* The single writer of the global timebase. Increment FIRST so
       the seqlock in ReadRaw sees the new value before any callback
       code could read the timebase. */
    Global_u32MilliTicks++;

    if (Global_pfTickCallback != NULL)
    {
        Global_pfTickCallback();
    }
}
