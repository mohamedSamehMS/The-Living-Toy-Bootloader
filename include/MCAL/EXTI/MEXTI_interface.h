/*
 * MEXTI_interface.h
 *
 *  PUBLIC interface of the MEXTI driver. HAL and Application code
 *  must include ONLY this file.
 *
 *  NOTE — replaces MEXTI_int.h (file renamed, guard renamed; delete
 *  the old file from the tree).
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 10: EXTI / SYSCFG
 *
 *  BOOT-ORDER CONTRACT (all steps are HARD prerequisites):
 *      1. MRCC_Init()
 *      2. MRCC_EnablePeripheralClock(MRCC_GPIOx)   — the button port
 *      3. MGPIO pin config (this driver does NOT own pins):
 *              MGPIO_SetPinMode (port, pin, MGPIO_MODE_INPUT);
 *              MGPIO_SetPinPull (port, pin, MGPIO_PULL_UP);
 *         (active-low buttons: pull-up, falling trigger — the pattern
 *          the config table's ENABLED rows assume)
 *      4. MRCC_EnablePeripheralClock(MRCC_SYSCFG) — CRITICAL: writes
 *         to SYSCFG EXTICR are SILENTLY ignored without it (the #1
 *         "nothing works, no error anywhere" trap — lines would stay
 *         routed to their reset port-A default)
 *      5. MNVIC_Init() and PRIORITIES (before first enable — the
 *         MNVIC boot-order contract). Per the Project 4 plan, buttons
 *         sit at priority 1. NOTE the shared vectors:
 *              lines 0..4  : one IRQ each    -> MNVIC_EXTI0..EXTI4
 *              lines 5..9  : ONE shared IRQ  -> MNVIC_EXTI9_5
 *              lines 10..15: ONE shared IRQ  -> MNVIC_EXTI15_10
 *         Setting MNVIC_EXTI15_10's priority sets it for ALL of
 *         lines 10..15 — there is no per-line NVIC priority.
 *      6. MEXTI_Init()   — applies the config table
 *      7. MEXTI_SetCallBack() per line (any time before events)
 *
 *  SCOPE BOUNDARY: GPIO lines 0..15 only. Internal EXTI lines
 *  16 (PVD), 17 (RTC alarm), 18 (OTG FS wake), 21 (tamper),
 *  22 (RTC wakeup) belong to their owning drivers (MPWR, MRTC,
 *  OTG) — they are hardwired internally and do not route through
 *  SYSCFG EXTICR.
 *
 *  CALLBACK CONTRACT: callbacks run in ISR context (EXTI priority —
 *  set high per the plan so buttons preempts SysTick and USART).
 *  ISR-minimal rule: set a flag / timestamp with
 *  MSYSTICK_GetElapsed_ms() and return — ALL processing happens in
 *  the super-loop. DEBOUNCING IS NOT THIS DRIVER'S JOB: mechanical
 *  buttons bounce, one press = several edges = several callback
 *  invocations. Debounce belongs at the HAL/app layer: ignore edges
 *  within ~20-30 ms of the last accepted one.
 *
 *  Revision history:
 *    Rev 1 — original API (MEXTI_int.h; no handlers, no NVIC wiring)
 *    Rev 2 — this rewrite: handlers + NVIC mapping live in the
 *            driver; MGPIO_Port_t reuse; NULL-means-unregister;
 *            MEXTI_TriggerLine (SWIER) added; naming sweep
 *
 *  Created on: Sep 5, 2026
 *      Author: dell
 */

#ifndef MEXTI_INTERFACE_H_
#define MEXTI_INTERFACE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Driver status codes — the library convention
 * ===================================================================== */
typedef enum
{
    MEXTI_OK = 0,            /* Operation completed                       */
    MEXTI_NOT_INITIALIZED,   /* Enable called before MEXTI_Init — the
                                config table (routing/trigger) has not
                                been applied, so the line would fire on
                                whatever the reset defaults are. Nothing
                                was armed.                                */
    MEXTI_INVALID_LINE,      /* Line id outside 0..15                     */
    MEXTI_INVALID_PARAM,     /* Garbage trigger / port / InitState value
                                (C enums are not type-safe)              */
    MEXTI_INVALID_CONFIG     /* A config-table row failed validation in
                                Init: bad field, or DUPLICATE line (the
                                second row would silently overwrite the
                                first's routing)                          */
} MEXTI_ReturnType_t;

/* =====================================================================
 *  EXTI line identifiers
 *
 *  Values equal the line number and are used directly as bit
 *  positions / loop indices — keep 0..15 in order.
 * ===================================================================== */
typedef enum
{
    MEXTI_LINE0 = 0, MEXTI_LINE1,  MEXTI_LINE2,  MEXTI_LINE3,
    MEXTI_LINE4,     MEXTI_LINE5,  MEXTI_LINE6,  MEXTI_LINE7,
    MEXTI_LINE8,     MEXTI_LINE9,  MEXTI_LINE10, MEXTI_LINE11,
    MEXTI_LINE12,    MEXTI_LINE13, MEXTI_LINE14, MEXTI_LINE15
} MEXTI_LineNumber_t;

/* NOTE: no MEXTI_Port_t — the port type is MGPIO_Port_t, reused
 * (values 0/1/2 = A/B/C are also the EXTICR encodings). The config
 * table's PortSource field uses it; one port type across the library. */

/* =====================================================================
 *  Trigger edge selection
 *
 *  Selector values only — the driver maps them to RTSR/FTSR bits
 *  (ON_CHANGE = both bits set, which is legal hardware usage).
 * ===================================================================== */
typedef enum
{
    MEXTI_TRIGGER_RISING = 0,
    MEXTI_TRIGGER_FALLING,
    MEXTI_TRIGGER_ON_CHANGE
} MEXTI_Trigger_t;

/* ========================================================================= *
 *                           PUBLIC API                                      *
 * ========================================================================= */

/**
 * @brief  Applies the static config table: validates every row (line,
 *         port, trigger, state, NO duplicate lines — validation runs
 *         for the WHOLE table before any register is touched: either
 *         the entire table lands or nothing is written), then per row
 *         sets SYSCFG routing, the trigger edges, clears any stale
 *         pending bit, and enables/disables the line.
 * @note   Re-Init is legal and idempotent (const table -> identical
 *         register writes). Registered callbacks are PRESERVED across
 *         re-Init — they are runtime state, not configuration.
 *         Prerequisites: steps 1-4 of the boot-order contract.
 */
MEXTI_ReturnType_t MEXTI_Init(void);

/**
 * @brief  Arm one line: clear any stale pending, unmask IMR, and
 *         enable the line's NVIC IRQ. THE ARMING FUNCTION — the only
 *         one gated on MEXTI_Init (enabling a line whose routing and
 *         trigger have not been configured is a config-order error).
 *
 *         Line -> IRQ mapping (internal, asserted in program.c):
 *              0..4  -> MNVIC_EXTI0 .. MNVIC_EXTI4   (own IRQ each)
 *              5..9  -> MNVIC_EXTI9_5                (shared)
 *              10..15-> MNVIC_EXTI15_10              (shared)
 * @note   Priority is the APPLICATION's job, BEFORE first enable
 *         (MNVIC boot-order contract; see the shared-vector note in
 *         the file header — one priority per IRQ, not per line).
 *         Re-enabling an already-active line is legal (IMR is plain
 *         RW; NVIC re-enable is W1S) but the stale-pending clear can
 *         consume one not-yet-serviced event — Enable is a
 *         boot/config-time function, not a per-event one.
 */
MEXTI_ReturnType_t MEXTI_EnableInterrupt (MEXTI_LineNumber_t Copy_Line);

/**
 * @brief  Disarm one line: mask IMR. DELIBERATELY does NOT disable the
 *         NVIC IRQ — for lines 5..15 the vector is SHARED: disabling
 *         it would silently kill other live lines on the same IRQ.
 *         NVIC-line shutdown is a shared-resource decision the
 *         application can make via MNVIC_DisableInterrupt itself.
 *         Ungated (the disarm-always-succeeds principle).
 */
MEXTI_ReturnType_t MEXTI_DisableInterrupt(MEXTI_LineNumber_t Copy_Line);

/**
 * @brief  Set the trigger edge(s) of one line (RTSR/FTSR). Not
 *         init-gated (pure configuration write), fully validated.
 *         A line enabled with BOTH trigger bits clear never pends —
 *         garbage trigger values are rejected instead of silently
 *         producing that state.
 */
MEXTI_ReturnType_t MEXTI_SetTrigger(MEXTI_LineNumber_t Copy_Line,
                                    MEXTI_Trigger_t    Copy_Trigger);

/**
 * @brief  Register the callback for one line. Called from the line's
 *         ISR after its pending bit is cleared. NULL is a VALID value:
 *         it UNREGISTERS the callback (library-wide convention —
 *         events with no callback are simply ignored, never a crash).
 *         Not init-gated: register any time before events.
 */
MEXTI_ReturnType_t MEXTI_SetCallBack(MEXTI_LineNumber_t Copy_Line,
                                     void (*Copy_CallBack)(void));

/**
 * @brief  Clear the pending flag of one line — direct W1C store (the
 *         ONLY correct PR idiom; see private.h). Ungated: clearing is
 *         always safe, like MNVIC_ClearPending. Use before manual
 *         re-enables to discard latched spurious events.
 */
MEXTI_ReturnType_t MEXTI_ClearFlag(MEXTI_LineNumber_t Copy_Line);

/**
 * @brief  Software-trigger one line: writes SWIER, which sets the
 *         line's PR pending bit. If the line is ENABLED (IMR + NVIC),
 *         the FULL dispatch path runs: IRQ fires -> handler -> pending
 *         cleared -> callback executed. THE zero-hardware button test.
 *         If the line is not enabled, only PR sets (no exception) —
 *         visible via a later ClearFlag/consumer.
 * @note   Distinct from MNVIC_TriggerIRQ(MNVIC_EXTI0): that pends
 *         ONLY the NVIC side (PR stays 0 — the handler runs, finds
 *         nothing, returns). It proves VECTOR LINKAGE; SWIER proves
 *         the whole path INCLUDING your callback.
 */
MEXTI_ReturnType_t MEXTI_TriggerLine(MEXTI_LineNumber_t Copy_Line);

/* IRQ handlers are NOT part of the public contract — the startup
 * file's weak Default_Handler aliases link to this driver's strong
 * EXTI0..4/EXTI9_5/EXTI15_10 definitions directly. */

/* =====================================================================
 *  Usage example (button on PA0, active low):
 *      (void)MRCC_EnablePeripheralClock(MRCC_GPIOA);
 *      (void)MGPIO_SetPinMode(MGPIO_PORTA, MGPIO_PIN_0, MGPIO_MODE_INPUT);
 *      (void)MGPIO_SetPinPull (MGPIO_PORTA, MGPIO_PIN_0, MGPIO_PULL_UP);
 *      (void)MRCC_EnablePeripheralClock(MRCC_SYSCFG);
 *      (void)MNVIC_Init();
 *      (void)MNVIC_SetPriority(MNVIC_EXTI0, 1, 0);   // plan: buttons = 1
 *      (void)MEXTI_Init();                            // table row LINE0
 *      (void)MEXTI_SetCallBack(MEXTI_LINE0, Btn_Callback);
 *      (void)MEXTI_TriggerLine(MEXTI_LINE0);          // no button needed!
 * ===================================================================== */

#endif /* MEXTI_INTERFACE_H_ */
