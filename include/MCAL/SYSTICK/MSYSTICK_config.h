/*
 * MSYSTICK_config.h
 *
 *  Configuration of the MSYSTICK driver.
 *
 *  Sections 1-3 are USER CHOICES. Section 4 derives the effective input
 *  frequency from the MRCC PUBLISHED CLOCK MODEL — the hand-written
 *  HCLK constant that used to live here is gone. Section 5 validates.
 *
 *  Reference: ARM PM0214 (Cortex-M4 Devices Generic User Guide) §4.4
 *
 *  Revision history:
 *    Rev 1 — original implementation (hard-coded HCLK_FREQ_HZ)
 *    Rev 2 — HCLK derived from MRCC model; granularity note corrected
 *            (500 ns, previously misstated as 4 us)
 *    Rev 3 — revision-history block introduced; no content change
 *            (the us-delay redesign of Rev 3 lives in program.c)
 *
 *  Created on: Jul 29, 2026
 *      Author: dell
 */

#ifndef MSYSTICK_CONFIG_H_
#define MSYSTICK_CONFIG_H_

#include "STD_TYPES.h"
#include <MCAL/RCC/MRCC_interface.h>   /* publishes MRCC_HCLK_KHZ (derived
                                           clock model — single source of
                                           truth for every clock number) */

/* =====================================================================
 *  1. Clock source  (user choice)
 *
 *  The CLKSOURCE bit selects between (PM0214 §4.4.2):
 *      MSYSTICK_CLK_HCLK_DIV8 : HCLK / 8  — the default; coarser tick
 *      MSYSTICK_CLK_HCLK      : HCLK      — 8x finer tick granularity
 *
 *  NOTE the semantics: it is HCLK vs HCLK/8 — NOT SYSCLK vs SYSCLK/8.
 *  If MRCC's AHB prescaler is not /1, HCLK and SYSCLK differ.
 * ===================================================================== */
#define MSYSTICK_CLK_HCLK_DIV8     0
#define MSYSTICK_CLK_HCLK          1

#define MSYSTICK_CLOCK_SOURCE      MSYSTICK_CLK_HCLK_DIV8

/* =====================================================================
 *  2. Processor clock — DERIVED, not hand-set
 *
 *  The old version hard-coded HCLK_FREQ_HZ = 16000000 with a comment
 *  "must match MRCC's configured HCLK" — a human-enforced invariant,
 *  i.e. exactly the bug class the MRCC rewrite eliminated. The failure
 *  mode was concrete: flip SYSTEM_CLOCK to PLL_HSE (84 MHz) and this
 *  driver would still believe 16 MHz; HCLK/8 would really be 10.5 MHz
 *  while LOAD stayed 1999, so the "1 ms" reload would actually be
 *  190 us — every delay in the system 5.25x short, zero diagnostics.
 *
 *  Consuming the published model means flipping the MRCC config
 *  updates SysTick automatically. RUNTIME dependency that goes with
 *  it: MSYSTICK_Init() must run AFTER MRCC_Init() whenever the clock
 *  tree differs from the reset default.
 * ===================================================================== */
#define MSYSTICK_HCLK_FREQ_HZ          (MRCC_HCLK_KHZ * 1000UL)

/* =====================================================================
 *  3. Exception priority  (user choice)
 *
 *  SysTick is system exception 15 — its priority lives in the SCB
 *  SHPR3 register, NOT in the NVIC (PM0214 §4.2.5, §4.4.2).
 *
 *  The F401 implements 4 priority bits (16 levels, 0 = highest).
 *  Default 15 (LOWEST) — the same convention FreeRTOS uses for its
 *  kernel tick — because:
 *    - this handler drives the 1 ms scheduler tick, which tolerates
 *      tens-of-us latency;
 *    - the Project 4 roadmap prioritizes EXTI (buttons), USART
 *      (ESP8266) and timers ABOVE SysTick — those ISRs must be able
 *      to preempt a scheduler dispatch in progress;
 *    - delay functions measure elapsed ticks, never assume they ran
 *      uninterrupted, so ISR preemption only ever stretches a delay
 *      slightly — it can never shorten one.
 * ===================================================================== */
#define MSYSTICK_INTERRUPT_PRIORITY    15

/* =====================================================================
 *  4. Derived values — DO NOT EDIT
 * ===================================================================== */

/* Effective SysTick input frequency after the CLKSOURCE choice */
#if MSYSTICK_CLOCK_SOURCE == MSYSTICK_CLK_HCLK_DIV8
    #define MSYSTICK_SYSTICK_INPUT_FREQ_HZ   (MSYSTICK_HCLK_FREQ_HZ / 8UL)
#else
    #define MSYSTICK_SYSTICK_INPUT_FREQ_HZ   (MSYSTICK_HCLK_FREQ_HZ)
#endif

/* One hardware tick in nanoseconds.
   Current config: 16 MHz / 8 = 2 MHz input -> 500 ns per tick.
   (CORRECTED in Rev 2: an earlier revision of this comment said
   4000 ns — the /8 divisor apparently applied twice, 0.5 us x 8. The
   macro formula was always right, nothing consumed the constant
   numerically, and no code decision was made from the wrong figure —
   the error was purely in this prose.) */
#define MSYSTICK_TICK_PERIOD_NS         (1000000000ULL / MSYSTICK_SYSTICK_INPUT_FREQ_HZ)

/* =====================================================================
 *  GRANULARITY NOTE for MSYSTICK_SetBusyWait_us
 *
 *  Two distinct limits — do not confuse them:
 *    1. HARDWARE limit: the tick period — 500 ns at the current
 *       config. This is the resolution of the MEASUREMENT.
 *    2. PRACTICAL limit: the polling loop body itself. Each sample
 *       costs the seqlock reads (ms, VAL, ms — one of them on the
 *       slow PPB), the wrap check, and 64-bit arithmetic — call it
 *       40-70 CPU cycles, roughly 2-4 us at 16 MHz. The loop
 *       cannot observe time passing faster than it can sample it.
 *
 *  The PRACTICAL limit dominates at 16 MHz: every request below
 *  ~3 us costs one loop iteration no matter what was asked, and the
 *  overshoot on longer requests is also one iteration, not one tick.
 *
 *  Decision guidance (the reason this note exists):
 *    - Switching CLKSOURCE to MSYSTICK_CLK_HCLK (62.5 ns ticks) does
 *      NOT help at 16 MHz: the measurement gets 8x finer, but the
 *      loop still samples only every 2-4 us. Do not flip CLKSOURCE
 *      for delay accuracy — it buys nothing until the CPU clock
 *      rises.
 *    - Raising SYSCLK in MRCC (84 MHz target) improves BOTH limits
 *      ~5x together, because the loop body shrinks with the CPU
 *      clock while the tick shrinks with HCLK.
 *    - Precise short waits belong to HARDWARE timing, not polling:
 *      the Phase-1 timer driver (wait on an update/compare flag), or
 *      the DWT cycle counter (CYCCNT — 62.5 ns resolution at 16 MHz,
 *      core-clocked) if something is needed before Phase 1.
 *
 *  Measuring the real floor on hardware: loop 1000 iterations of
 *  SetBusyWait_us(1) and read the GetElapsed_ms delta — that is the
 *  per-call cost in us. (A single call is below the ms counter's
 *  resolution, so one measurement alone proves nothing.)
 * ===================================================================== */

/* =====================================================================
 *  5. Configuration validation — bad values stop the build
 * ===================================================================== */
#if (MSYSTICK_CLOCK_SOURCE != MSYSTICK_CLK_HCLK_DIV8) && \
    (MSYSTICK_CLOCK_SOURCE != MSYSTICK_CLK_HCLK)
#error "MSYSTICK: MSYSTICK_CLOCK_SOURCE must be MSYSTICK_CLK_HCLK_DIV8 or MSYSTICK_CLK_HCLK"
#endif

#if (MSYSTICK_INTERRUPT_PRIORITY > 15)
#error "MSYSTICK: priority exceeds 15 - the F401 implements 4 priority bits"
#endif

/* A 1 ms reload needs at least 1000 input ticks per millisecond */
#if (MSYSTICK_SYSTICK_INPUT_FREQ_HZ < 1000UL)
#error "MSYSTICK: input frequency too low for a 1 ms tick"
#endif

/* NOTE: MSYSTICK_MAX_CALLBACKS was removed deliberately. One callback
 * slot is a design decision, not a config knob: the cooperative
 * scheduler IS the tick multiplexer — modules with periodic work
 * subscribe to the scheduler, they do not each grab a 1 ms ISR hook.
 * If you ever feel the need to raise this number, build the registry
 * into the scheduler instead, at thread context — not here, in
 * interrupt context.
 */

#endif /* MSYSTICK_CONFIG_H_ */
