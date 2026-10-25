/*
 * MTIM_interface.h
 *
 *  PUBLIC interface of the MTIM driver. HAL and Application code must
 *  include ONLY this file.
 *
 *  Reference: RM0368 Rev 6, Ch 12/13/14; PM0214 (NVIC, indirectly)
 *
 *  BOOT-ORDER CONTRACT (per used timer):
 *      1. MRCC_Init()
 *      2. MRCC_EnablePeripheralClock(MRCC_TIMx)   - CRITICAL: writes to
 *         an un-clocked timer are silently ignored (the #1 trap)
 *      3. For pin-using modes (PWM / capture), MGPIO AF wiring FIRST -
 *         this driver does NOT own pins. Representative map (AF table,
 *         RM0368 -8.3):
 *              TIM5_CH1 = PA0  AF2  (the IR pin - also TIM2_CH1, AF2)
 *              TIM3     = PA6/PA7/PB0/PB1, all AF2 (backlight pool)
 *              TIM1_CH1 = PA8  AF1  (MOE path)
 *              TIM9     = PA2/PA3 AF3 - CONFLICT: debug console. Audio
 *                         Option (b) uses NO pin (UDE only); Option (a)
 *                         would sacrifice the console.
 *              TIM4     = pinless in this project (pure tick)
 *      4. MNVIC_Init() + priorities BEFORE arming interrupts
 *         (plan: TIM = 4-5, below DMA/USART, above SysTick). Note the
 *         deliberate demotion: TIM5 capture replaces the EXTI line-0
 *         button-class IRQ (priority 1) - correct, because capture
 *         fires only on decoder-relevant edges.
 *      5. MTIM_Init()  - applies the config table (PSC/ARR/mode)
 *      6. Consumer: Start / EnableUpdateInterrupt / PWMStart /
 *         CaptureStart...
 *
 *  THE ONE-FUNCTION-PER-TIMER RULE (silicon, enforced by the driver):
 *  one PSC/ARR pair per counter -> one frequency per timer. A second
 *  mode on an occupied timer, or a second PWM channel at a different
 *  frequency, returns MTIM_INVALID_STATE. Multiple PWM channels at the
 *  SAME frequency on one timer are legal (that is what channels are).
 *
 *  ISR CONTRACT: callbacks run in ISR context at the configured TIM
 *  priority. ISR-minimal rule applies (budget <= ~150 us per the NEC
 *  tolerance analysis). Capture callbacks receive the CCR timestamp.
 *
 *  Revision history:
 *    Rev 1 - first release (family driver: 8 instances, time-base /
 *            periodic / PWM / capture / UDE-DMA-trigger modes)
 *
 *  Created on: Sep 12, 2026
 *      Author: dell
 */

#ifndef MTIM_INTERFACE_H_
#define MTIM_INTERFACE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Driver status codes - the library convention
 * ===================================================================== */
typedef enum
{
    MTIM_OK = 0,            /* Operation completed                       */
    MTIM_INVALID_TIMER,     /* Timer id outside the 8-instance set      */
    MTIM_INVALID_CHANNEL,   /* Channel absent on this timer (e.g. CH3
                               on TIM10 - one-channel silicon)          */
    MTIM_INVALID_PARAM,     /* Frequency impossible / duty > 1000 /
                             * NULL output pointer / bad enum value     */
    MTIM_INVALID_STATE,     /* Mode conflict - the one-function-per-
                               timer rule; see the file header          */
    MTIM_NOT_INITIALIZED,   /* MTIM_Init has not run                    */
    MTIM_INVALID_CONFIG     /* A config-table row failed validation
                               (bad mode/frequency, or duplicate timer) */
} MTIM_ReturnType_t;

/* =====================================================================
 *  Timer instances - all eight on the F401
 *
 *  Values index the internal instance table; keep 0..7 in order.
 *  Width/pin annotations per the definitive allocation document.
 * ===================================================================== */
typedef enum
{
    MTIM_TIM1 = 0,      /* 16-bit advanced, APB2 - reserved; supported
                           basic (time-base/PWM incl. the MOE path);
                           complementary/dead-time/break OUT OF SCOPE   */
    MTIM_TIM2,          /* 32-bit, APB1 - reserve (Attract-Mode ?)     */
    MTIM_TIM3,          /* 16-bit, APB1 - backlight PWM (runtime)      */
    MTIM_TIM4,          /* 16-bit, APB1 - matrix POV tick (400 Hz)     */
    MTIM_TIM5,          /* 32-bit, APB1 - NEC IR capture (1 MHz base)  */
    MTIM_TIM9,          /* 16-bit, APB2 - audio DMA trigger (runtime)  */
    MTIM_TIM10,         /* 16-bit, APB2 - conditional buzzer PWM       */
    MTIM_TIM11,         /* 16-bit, APB2 - conditional buzzer PWM       */
    MTIM_INSTANCE_COUNT
} MTIM_Timer_t;

/* =====================================================================
 *  Capture/compare channels
 *
 *  Values are channel indices used directly as shift bases - keep
 *  0..3. Validity is per-instance (TIM2-5 & TIM1: all 4; TIM9: 2;
 *  TIM10/11: 1) - enforced by MTIM_INVALID_CHANNEL.
 * ===================================================================== */
typedef enum
{
    MTIM_CH1 = 0,
    MTIM_CH2,
    MTIM_CH3,
    MTIM_CH4
} MTIM_Channel_t;

/* =====================================================================
 *  PWM output polarity (CCER CCxP, output meaning)
 * ===================================================================== */
typedef enum
{
    MTIM_PWM_ACTIVE_HIGH = 0,   /* CCxP = 0 - the normal LED/backlight case */
    MTIM_PWM_ACTIVE_LOW         /* CCxP = 1 - inverted duty                  */
} MTIM_PWMPolarity_t;

/* =====================================================================
 *  Capture edge (CCER CCxP, input meaning)
 * ===================================================================== */
typedef enum
{
    MTIM_CAPTURE_RISING = 0,    /* CCxP = 0 */
    MTIM_CAPTURE_FALLING        /* CCxP = 1 */
} MTIM_CaptureEdge_t;

/* =====================================================================
 *  Callback types
 *
 *  Update callback: no argument (the period elapsed).
 *  Capture callback: the CCR timestamp - the free-running CNT value
 *  latched at the edge. Differencing consecutive timestamps (any
 *  channel, any edge) yields durations; unsigned subtraction handles
 *  the wrap (32-bit at 1 MHz wraps at ~71.6 minutes - no concern for
 *  68 ms NEC frames; 16-bit timers wrap every 65536 ticks, deltas
 *  still correct in u32 arithmetic within one wrap).
 * ===================================================================== */
typedef void (*MTIM_UpdateCallback_t)(void);
typedef void (*MTIM_CaptureCallback_t)(u32 Copy_u32Timestamp);

/* ========================================================================= *
 *                           PUBLIC API                                      *
 * ========================================================================= */

/**
 * @brief  Validate the whole config table (modes, frequencies against
 *         the published clock, duplicate timers), then apply each row:
 *         stop the timer, solve+write PSC/ARR, force UG to latch, clear
 *         UIF. Mode state is recorded. Counters are NOT started and no
 *         interrupts are armed - start/arming is the consumer's
 *         explicit act (config-vs-arm separation: game pause exists).
 * @note   Whole-table-first: a bad row leaves NO register touched.
 *         Re-Init re-applies the (const) table; registered callbacks
 *         and armed interrupt bits are preserved (MEXTI convention).
 *         Re-Init of a RUNNING timer re-latches its time-base by force
 *         (UG zeroes CNT) - that is what "reconfigure" means here.
 */
MTIM_ReturnType_t MTIM_Init(void);

/* --------------------- Time-base --------------------- */

/**
 * @brief  Configure a timer as a FREE-RUNNING measurement base at
 *         Copy_u32TickHz resolution: PSC = CLK/tick - 1, ARR = maximum,
 *         counter wraps forever. The input-capture time-base (TIM5 @
 *         1 MHz is the design case). Update interrupts may be armed on
 *         it (wrap tracking) via EnableUpdateInterrupt.
 * @note   Does not start the counter (MTIM_Start does). Re-configuring
 *         the SAME mode is allowed (rate change); a different active
 *         mode returns MTIM_INVALID_STATE (DeInit first).
 */
MTIM_ReturnType_t MTIM_TimeBaseConfig(MTIM_Timer_t Copy_Timer, u32 Copy_u32TickHz);

/**
 * @brief  Configure an EXACT periodic time-base at Copy_u32FrequencyHz:
 *         PSC/ARR solved to maximize ARR (duty resolution), force-UG
 *         latched, UIF cleared. The update-interrupt/DMA-request
 *         period (TIM4's 400 Hz tick; TIM9's future sample rate).
 * @note   Integer division floors: actual = CLK / ((PSC+1)*(ARR+1)),
 *         error at most one count per period (ppm-scale at these
 *         rates). Impossible targets (too high/low for the width)
 *         return MTIM_INVALID_PARAM - never a silent wrong frequency.
 */
MTIM_ReturnType_t MTIM_PeriodicConfig(MTIM_Timer_t Copy_Timer, u32 Copy_u32FrequencyHz);

/** Start / stop the counter (CR1.CEN). Stop freezes CNT; state kept. */
MTIM_ReturnType_t MTIM_Start(MTIM_Timer_t Copy_Timer);
MTIM_ReturnType_t MTIM_Stop (MTIM_Timer_t Copy_Timer);

/** Full teardown: stop, disarm UIE/UDE/CCxIE, mode -> NONE. The
 *  escape hatch of the mode state machine ("give the timer back"). */
MTIM_ReturnType_t MTIM_DeInit(MTIM_Timer_t Copy_Timer);

/** Read CNT (u32; 16-bit timers naturally read 0..65535).
 *  Wrap-safe by unsigned subtraction at the CALLER - deltas, never
 *  direct comparison. */
MTIM_ReturnType_t MTIM_GetCounter(MTIM_Timer_t Copy_Timer, u32 *Copy_pu32Counter);

/* --------------------- Update events --------------------- */

/**
 * @brief  Arm the update interrupt: DIER.UIE + the instance's NVIC IRQ
 *         + callback registration. NULL callback = flag-only operation
 *         (poll UIF via... not exposed - consumer reads SR? No: NULL
 *         is allowed and the event is simply ignored, library rule).
 *
 *  DELIBERATE CONTRAST with MEXTI's Enable (which WIPES stale pending
 *  first): a pending UIF at arm time represents a REAL elapsed period
 *  - periodic semantics honor it (the first callback may fire
 *  immediately, correctly). EXTI's edge semantics wipe instead. Both
 *  are right for their own event class; this is why the two drivers
 *  differ on purpose.  MTIM_Init already cleared UIF after its UG.
 */
MTIM_ReturnType_t MTIM_EnableUpdateInterrupt (MTIM_Timer_t Copy_Timer, MTIM_UpdateCallback_t Copy_pfCallback);
MTIM_ReturnType_t MTIM_DisableUpdateInterrupt(MTIM_Timer_t Copy_Timer);

/**
 * @brief  Arm the update DMA request (DIER.UDE) - NO NVIC, NO ISR: the
 *         audio-path hook. Option (b) full sequence (TIM9 case):
 *              MTIM_PeriodicConfig(MTIM_TIM9, 8000);      /* 8 kHz
 *              MTIM_EnableUpdateDMARequest(MTIM_TIM9);    /* UDE on
 *              MTIM_Start(MTIM_TIM9);
 *              /* MDMA (future): stream with PAR = &GPIOx->BSRR,
 *                 memory = precomputed BSRR words, request = TIM9 up
 *         Each update event paces one DMA word - zero CPU per sample.
 *         (The corrected Option-b mechanism: request-paced
 *         memory-to-peripheral with BSRR as the "peripheral", NOT
 *         software-initiated M2M. MDMA owns the stream configuration.)
 */
MTIM_ReturnType_t MTIM_EnableUpdateDMARequest (MTIM_Timer_t Copy_Timer);
MTIM_ReturnType_t MTIM_DisableUpdateDMARequest(MTIM_Timer_t Copy_Timer);

/* --------------------- PWM --------------------- */

/**
 * @brief  Start PWM on one channel: solves PSC/ARR for the frequency,
 *         configures the channel (PWM mode 1 + CCR preload - glitch-
 *         free duty updates), CCER enable + polarity, ARPE on, and for
 *         TIM1 ONLY additionally sets BDTR.MOE (without which TIM1
 *         outputs NOTHING, silently - the trap this path exists to
 *         kill). Starts the counter.
 *
 *  Mode rules: timer must be mode NONE (first use) or PWM at the SAME
 *  frequency (adding a channel - the multi-channel same-frequency
 *  case channels exist for). PWM at a DIFFERENT frequency, or any
 *  other active mode, returns MTIM_INVALID_STATE - the ARR is shared,
 *  the rule is silicon.
 *
 *  Duty is PERMILLE (0..1000) - integer, 0.1% steps: fine enough for
 *  backlight, no float in an integer project.
 *  Pin wiring (AF) is the caller's - see the boot-order contract.
 */
MTIM_ReturnType_t MTIM_PWMStart(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                u32 Copy_u32FrequencyHz, u16 Copy_u16DutyPermille,
                                MTIM_PWMPolarity_t Copy_Polarity);

/** Change duty at runtime. CCR is preloaded: takes effect at the next
 *  period boundary - no glitch mid-cycle. Timer must be in PWM mode. */
MTIM_ReturnType_t MTIM_PWMSetDuty(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                  u16 Copy_u16DutyPermille);

/** Stop one PWM channel (CCER disable). The timer keeps running while
 *  any channel remains; the mode stays PWM until DeInit. */
MTIM_ReturnType_t MTIM_PWMStop(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel);

/* --------------------- Input capture --------------------- */

/**
 * @brief  Arm input capture on one channel, on that channel's own pin
 *         (CH1->TI1, CH2->TI2...), with the given edge and callback.
 *         Requires mode FREE_RUN (the capture time-base must already
 *         be configured - MTIM_TimeBaseConfig first; loud ordering).
 *         WIPES any stale CCxIF before arming (an edge that arrived
 *         before capture was configured is not the consumer's event -
 *         EXTI semantics here, contrasting the update-event decision
 *         documented above). Enables the instance's capture NVIC IRQ.
 *
 *  ISR behavior: on each edge the hardware latches CNT into CCR; the
 *  handler reads CCR, clears CCxIF (+CCxOF if an overcapture occurred
 *  - the delivered timestamp is then the OLDER edge; impossible at NEC
 *  rates but handled), then calls the callback with the timestamp.
 */
MTIM_ReturnType_t MTIM_CaptureStart(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                    MTIM_CaptureEdge_t Copy_Edge,
                                    MTIM_CaptureCallback_t Copy_pfCallback);

/**
 * @brief  THE NEC PATTERN - both edges of ONE pin, two channels:
 *         CH1 captures rising (TI1), CH2 ALSO captures TI1 (CC2S=10
 *         maps CH2's input onto TI1) with falling polarity. Requires
 *         FREE_RUN mode + a 2-channel timer + both channels free.
 *         Rising edges -> Copy_pfRising(ts); falling -> Copy_pfFalling
 *         (ts). The IR decoder differences consecutive timestamps
 *         across BOTH callbacks to measure marks AND spaces - the
 *         268 ms busy-wait engine's hardware replacement.
 */
MTIM_ReturnType_t MTIM_CaptureStartBothEdges(MTIM_Timer_t Copy_Timer,
                                             MTIM_CaptureCallback_t Copy_pfRising,
                                             MTIM_CaptureCallback_t Copy_pfFalling);

/** Disarm one capture channel (CCxE + CCxIE off, callback cleared).
 *  NVIC line deliberately NOT disabled - TIM1/9/10/11 vectors are
 *  shared (the MEXTI shared-vector principle). */
MTIM_ReturnType_t MTIM_CaptureStop(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel);

/** Poll the last captured timestamp of a channel (CCR read; the flag
 *  was already cleared by the ISR - this is a stale-view read). */
MTIM_ReturnType_t MTIM_GetCapture(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                  u32 *Copy_pu32Timestamp);

/* IRQ handlers are NOT public API - startup-file weak-alias linkage,
 * same as MSPI/MUSART/MEXTI. Eight entries: TIM2..TIM5 dedicated;
 * TIM1's four vectors shared with TIM9/10/11 per the family layout
 * (positions 24-27; the small timers ride TIM1's break/update/trg
 * vectors - each entirely, TIM1's CC vector is TIM1-only). */

/* =====================================================================
 *  Usage example - the two Phase-2 migrations:
 *
 *  Matrix tick (TIM4):   MRCC_EnablePeripheralClock(MRCC_TIM4);
 *                        MNVIC_SetPriority(MNVIC_TIM4, 4, 0);
 *                        MTIM_Init();                       // 400 Hz row
 *                        MTIM_EnableUpdateInterrupt(MTIM_TIM4, Matrix_Tick);
 *                        MTIM_Start(MTIM_TIM4);
 *
 *  IR capture (TIM5):    MGPIO_SetPinMode(PORTA, PIN_0, ALT_FUNC); AF2...
 *                        MNVIC_SetPriority(MNVIC_TIM5, 5, 0);
 *                        MTIM_Init();                       // 1 MHz row
 *                        MTIM_CaptureStartBothEdges(MTIM_TIM5,
 *                                                   IR_Rise, IR_Fall);
 *                        MTIM_Start(MTIM_TIM5);
 * ===================================================================== */

#endif /* MTIM_INTERFACE_H_ */
