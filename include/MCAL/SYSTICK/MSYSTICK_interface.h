/*
 * MSYSTICK_interface.h
 *
 *  PUBLIC interface of the MSYSTICK driver. HAL and Application code
 *  must include ONLY this file.
 *
 *  Reference: ARM PM0214 (Cortex-M4 Devices Generic User Guide) §4.4
 *
 *  INITIALIZATION ORDER (contract):
 *      1. MRCC_Init()          — the clock tree must match the model
 *      2. MSYSTICK_Init()      — this driver
 *  With the current HSI default, HCLK is 16 MHz either way, but the
 *  order matters the day SYSTEM_CLOCK becomes PLL_HSE.
 *
 *  CONCURRENCY CONTRACT: the 1 ms handler runs at the LOWEST interrupt
 *  priority by default (config), so every peripheral ISR may preempt
 *  it. All delay/getter functions are therefore safe to call from any
 *  context EXCEPT the SysTick handler itself. Delays measure elapsed
 *  time against a wrap-aware reader (Rev 3): interrupt load can
 *  stretch a delay — it cannot shorten one. (An earlier revision
 *  overclaimed this for the us path; see SetBusyWait_us.)
 *
 *  Revision history:
 *    Rev 1 — original API (void prefixes, u8 error codes)
 *    Rev 2 — status enum aligned with MRCC/MGPIO; TICKINT fix;
 *            names de-voided; granularity floor documented at 500 ns
 *    Rev 3 — us-delay exactness contract corrected for the wrap-aware
 *            reader (never early; see SetBusyWait_us)
 *
 *  Created on: Jul 29, 2026
 *      Author: dell
 */

#ifndef MSYSTICK_INTERFACE_H_
#define MSYSTICK_INTERFACE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Driver status codes — same convention as MRCC_ReturnType_t /
 *  MGPIO_ReturnType_t. (MSYSTICK_NOK was deleted: it was never
 *  returned anywhere.)
 * ===================================================================== */
typedef enum
{
    MSYSTICK_OK = 0,            /* Operation completed                  */
    MSYSTICK_NOT_INITIALIZED,   /* Init has not run yet — nothing was
                                   touched, nothing was waited           */
    MSYSTICK_ALREADY_INITIALIZED/* Init called twice — rejected, state
                                   untouched (a re-init would rewind the
                                   timebase to 0 and make every elapsed-
                                   time comparison see a ~49.7-day jump) */
} MSYSTICK_ReturnType_t;

/* =====================================================================
 *  API
 * ===================================================================== */

/* Configure SysTick and START the free-running 1 ms timebase:
 *   - clock source per config (HCLK or HCLK/8)
 *   - LOAD sized for exactly 1 ms, hardware auto-reloads forever
 *   - exception priority written to SCB SHPR3 (SysTick is system
 *     exception 15 — SCB, not NVIC)
 *   - ENABLE and TICKINT written together in ONE CTRL store
 *
 * THE FIX this function carries (Rev 2): the original implementation
 * never set TICKINT, contradicting this very contract — the ms
 * counter never advanced, ms delays hung forever, and us delays
 * returned early (the sub-ms term sawed 0..1999..0 while the ms term
 * stayed 0, so the unsigned delta underflowed and the loop exited
 * instantly on the first counter wrap). Init now genuinely starts
 * the tick.
 *
 * A second call returns MSYSTICK_ALREADY_INITIALIZED and changes
 * nothing. (After a real reset the statics are zero again, so init
 * works normally — "twice" means twice in one power cycle.) */
MSYSTICK_ReturnType_t MSYSTICK_Init(void);

/* Enable/disable ONLY the exception (TICKINT bit). The counter keeps
 * running either way.
 *
 * TIMEBASE-LOSS CONTRACT: while disabled, wraps are NOT counted. On
 * re-enable at most ONE pending exception fires, representing the
 * whole blackout — Global_u32MilliTicks loses the elapsed ms it did
 * not see (SysTick exceptions do not stack; pending coalesces into
 * one). Fine for a short critical section; corrupting if held long.
 * Note: the us delay path is exempt — its wrap-aware reader counts
 * pending wraps itself, so a us delay taken during a blackout still
 * measures TRUE time (Rev 3). The global ms counter is what loses.
 *
 * Also note: any read of CTRL clears COUNTFLAG, and these two
 * functions read CTRL as part of their read-modify-write — one
 * MSYSTICK_HasCounted() poll can be consumed as a side effect. These
 * functions are for occasional critical sections, not routine use. */
MSYSTICK_ReturnType_t MSYSTICK_InterruptEnable (void);
MSYSTICK_ReturnType_t MSYSTICK_InterruptDisable(void);

/* Blocking delays, measured against the free-running timebase —
 * LOAD/VAL are never reprogrammed, so delays never disturb the 1 ms
 * counter or the scheduler tick.
 *
 *   _ms : waits on the ms counter alone; unsigned subtraction makes
 *         it correct across the u32 wrap (~49.7 days). IMMUNE to
 *         wrap races by construction — the counter is monotonic.
 *   _us : waits on TRUE total ticks (ms count + sub-ms VAL position
 *         + wraps the handler has not counted yet) in 64-bit math,
 *         multiply-before-divide so fractional-MHz clocks are not
 *         truncated.
 *
 * EXACTNESS CONTRACT for _us (Rev 3): measured against true time, so
 * a pending-but-unhandled wrap can never corrupt the reading. In
 * normal operation the wait is never EARLY and runs late by at most
 * one polling iteration (~2-4 us at 16 MHz) plus any interrupt
 * latency. It stays correct even while the SysTick handler is
 * starved by higher-priority ISRs or masked by PRIMASK/BASEPRI (the
 * reader compensates uncounted wraps itself).
 *
 * For ms-accurate waits prefer _ms (exact by construction); for
 * precise short waits, the Phase-1 timer driver is the right tool.
 *
 * Granularity floor: hardware tick is 500 ns, but the polling loop
 * body (~2-4 us at 16 MHz) dominates — see the granularity note in
 * MSYSTICK_config.h BEFORE changing CLKSOURCE: at 16 MHz it does not
 * help.
 *
 * Both return MSYSTICK_NOT_INITIALIZED and DO NOT WAIT when Init has
 * not run (Global_u32TicksPerMs would be 0 and the math meaningless). */
MSYSTICK_ReturnType_t MSYSTICK_SetBusyWait_ms(u32 Copy_u32Ms);
MSYSTICK_ReturnType_t MSYSTICK_SetBusyWait_us(u32 Copy_u32Us);

/* Free-running milliseconds since Init(). Monotonic except for the
 * documented TICKINT-blackout/coalescing loss; wraps at 2^32 ms
 * (~49.7 days). ALWAYS compute elapsed time by subtraction (Now -
 * Last) — never direct comparison — and wraparound is handled
 * correctly. Returns 0 before Init — harmless, since 0 is also the
 * true value at t = 0; no status needed for this getter. */
u32 MSYSTICK_GetElapsed_ms(void);

/* Register a callback to run from the SysTick handler every 1 ms.
 *
 * ONE SLOT, overwrite semantics — calling again replaces the previous
 * registration; pass NULL to disable. This is deliberate (config
 * explains why): the cooperative scheduler is the tick multiplexer.
 * It should own this slot and dispatch periodic work at thread
 * context; modules must not each grab a direct 1 ms ISR hook.
 *
 * The callback runs in ISR context at the configured (lowest)
 * priority: keep it extremely short, and do not call any MSYSTICK
 * delay function from it.
 *
 * Returns MSYSTICK_NOT_INITIALIZED if Init has not run — the pointer
 * is still stored and will start firing once Init enables the
 * exception. */
MSYSTICK_ReturnType_t MSYSTICK_SetTickCallback(void (*Copy_pfCallback)(void));

/* 1 if the counter reached 0 since the last CTRL read by ANYONE, else
 * 0. COUNTFLAG is read-to-clear (PM0214 §4.4.2): the flag can be
 * consumed exactly once per wrap, by whichever code reads CTRL first.
 * Do not read SYSTICK->CTRL anywhere else in the codebase, or this
 * function's readings are silently stolen. (The driver itself reads
 * CTRL only in InterruptEnable/Disable — documented there — and
 * clears the flag deliberately in Init via the VAL write. The Rev 3
 * busy-wait loop does NOT read CTRL, so it never steals flags.)
 * Returns 0 (not an error) before Init. */
u8 MSYSTICK_HasCounted(void);

#endif /* MSYSTICK_INTERFACE_H_ */
