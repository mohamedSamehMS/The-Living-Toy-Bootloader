/*
 * MTIM_program.c
 *
 *  Implementation of the MTIM driver - the first NEW silicon driver of
 *  the project (everything before this was revision of existing code).
 *
 *  Reference: RM0368 Rev 6, Ch 12/13/14
 *
 *  DESIGN PRINCIPLES (documented, deliberate):
 *
 *  1. TABLE-DRIVEN FAMILY. One static table carries per instance: the
 *     register pointer, NVIC ids (update-IRQ and capture-IRQ differ on
 *     TIM1!), width, channel count, advanced flag, clock Hz - plus the
 *     mutable mode/callback state. The three register-map shapes are
 *     one superset struct + capability flags validated before access
 *     (absent registers on TIM9/10/11 are unreachable by construction).
 *
 *  2. THE MODE STATE MACHINE IS THE ONE-FUNCTION-PER-TIMER RULE MADE
 *     LOUD. One PSC/ARR pair per counter = one frequency per timer.
 *     Mode conflicts and cross-frequency PWM return MTIM_INVALID_STATE
 *     instead of silently compromising the third consumer.
 *
 *  3. SOLVER, NOT CONSTANTS. PSC/ARR are computed from the published
 *     clock model and a runtime target - the 16->84 MHz flip is
 *     compile-time automatic, and impossible frequencies are rejected
 *     loudly (never a silently-wrong period).
 *
 *  4. THE UG LATCH SEQUENCE. PSC written to the register goes live only
 *     at the next update event: write PSC/ARR -> force EGR.UG (latches
 *     immediately, zeroes CNT) -> clear the UIF that UG just set.
 *     Skipping the clear = an instant spurious interrupt on arming.
 *
 *  5. SR SNAPSHOT + WRITE-0-TO-CLEAR. One SR read at ISR entry (read
 *     is side-effect-free - no coupled sequences in this peripheral);
 *     per serviced flag: `SR = ~mask` (clears ONLY that bit - the
 *     MUSART-TC idiom, the OPPOSITE of EXTI's PR W1C).
 *
 *  6. NVIC PER THE LAYERING RULE. The driver enables its own IRQ lines
 *     (typed MNVIC calls, status-checked) and nothing else; priorities
 *     are the application's.
 *
 *  Revision history:
 *    Rev 1 - first release
 *
 *  Created on: Sep 12, 2026
 *      Author: dell
 */
#include "BIT_MATH.h"
#include "STD_TYPES.h"

#include <MCAL/TIM/MTIM_interface.h>
#include <MCAL/TIM/MTIM_private.h>
#include <MCAL/TIM/MTIM_config.h>

#include "MNVIC_interface.h"


/* ========================================================================= *
 *  Compile-time consistency guards
 * ========================================================================= */
_Static_assert(MTIM_TIM1 == 0 && MTIM_TIM11 == 7, "timer enum must equal table index");
_Static_assert(MTIM_CH1 == 0 && MTIM_CH4 == 3,    "channel enum must equal channel index");

/* Shared-vector facts, pinned (startup-vector ground truth):
 * TIM9 entirely -> 24; TIM10 entirely -> 25; TIM11 entirely -> 26;
 * TIM1 update -> 25, TIM1 CC -> 27 (TIM1 break/trg never armed here). */
_Static_assert(MNVIC_TIM1_BRK_TIM9      == 24, "vector layout");
_Static_assert(MNVIC_TIM1_UP_TIM10      == 25, "vector layout");
_Static_assert(MNVIC_TIM1_TRG_COM_TIM11 == 26, "vector layout");
_Static_assert(MNVIC_TIM1_CC            == 27, "vector layout");
_Static_assert(MNVIC_TIM2 == 28 && MNVIC_TIM3 == 29, "vector layout");
_Static_assert(MNVIC_TIM4 == 30 && MNVIC_TIM5 == 50, "vector layout");

/* ========================================================================= *
 *  Private types & the instance table
 * ========================================================================= */

typedef enum
{
    MTIM_PRIVATE_EV_UPDATE   = 1u,   /* service UIF only                     */
    MTIM_PRIVATE_EV_CAPTURE  = 2u,   /* service CCxIF only                   */
    MTIM_PRIVATE_EV_ALL      = 3u    /* both                                  */
} MTIM_PrivateEventMask_t;

typedef struct
{
    /* ---- immutable wiring ---- */
    volatile MTIM_Registers_t *Reg;
    MNVIC_IRQ_t                UpdateIrq;    /* where THIS timer's update pends */
    MNVIC_IRQ_t                CaptureIrq;   /* where its CC events pend        */
    u8                         Is32Bit;      /* TIM2/TIM5 only                   */
    u8                         ChannelCount; /* 4 / 2 / 1                        */
    u8                         IsAdvanced;   /* TIM1: BDTR.MOE path              */
    u32                        ClockHz;      /* published model, per bus         */
    /* ---- mutable state ---- */
    MTIM_Mode_t                ActiveMode;
    u32                        ActiveFrequencyHz;   /* PWM same-freq check       */
    MTIM_UpdateCallback_t      UpdateCallback;
    MTIM_CaptureCallback_t     CaptureCallback[4]; /* per channel; NULL = none  */
} MTIM_Instance_t;

static MTIM_Instance_t MTIM_Instances[MTIM_INSTANCE_COUNT] =
{
    /*   Reg                UpdateIrq                CaptureIrq               32b ch adv clock                    mode0 */
    [MTIM_TIM1]  = { MTIM_TIM1_REG,  MNVIC_TIM1_UP_TIM10,      MNVIC_TIM1_CC,            0u, 4u, 1u, MTIM_APB2_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} },
    [MTIM_TIM2]  = { MTIM_TIM2_REG,  MNVIC_TIM2,               MNVIC_TIM2,               1u, 4u, 0u, MTIM_APB1_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} },
    [MTIM_TIM3]  = { MTIM_TIM3_REG,  MNVIC_TIM3,               MNVIC_TIM3,               0u, 4u, 0u, MTIM_APB1_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} },
    [MTIM_TIM4]  = { MTIM_TIM4_REG,  MNVIC_TIM4,               MNVIC_TIM4,               0u, 4u, 0u, MTIM_APB1_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} },
    [MTIM_TIM5]  = { MTIM_TIM5_REG,  MNVIC_TIM5,               MNVIC_TIM5,               1u, 4u, 0u, MTIM_APB1_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} },
    [MTIM_TIM9]  = { MTIM_TIM9_REG,  MNVIC_TIM1_BRK_TIM9,      MNVIC_TIM1_BRK_TIM9,      0u, 2u, 0u, MTIM_APB2_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} },
    [MTIM_TIM10] = { MTIM_TIM10_REG, MNVIC_TIM1_UP_TIM10,      MNVIC_TIM1_UP_TIM10,      0u, 1u, 0u, MTIM_APB2_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} },
    [MTIM_TIM11] = { MTIM_TIM11_REG, MNVIC_TIM1_TRG_COM_TIM11, MNVIC_TIM1_TRG_COM_TIM11, 0u, 1u, 0u, MTIM_APB2_TIMER_CLK_HZ, MTIM_MODE_NONE, 0u, 0u, {NULL,NULL,NULL,NULL} }
};

static u8 Global_u8IsInitialized = 0;

/* ========================================================================= *
 *  Private prototypes
 * ========================================================================= */
static MTIM_ReturnType_t MTIM_privateValidateTimer (MTIM_Timer_t Copy_Timer);
static MTIM_ReturnType_t MTIM_privateValidateChannel(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel);
static MTIM_ReturnType_t MTIM_privateSolvePeriodic   (const MTIM_Instance_t *Copy_pInst, u32 Copy_u32FrequencyHz,
                                                      u16 *Copy_pu16Psc, u32 *Copy_pu32Arr);
static MTIM_ReturnType_t MTIM_privateSolveFreeRun   (const MTIM_Instance_t *Copy_pInst, u32 Copy_u32TickHz,
                                                     u16 *Copy_pu16Psc, u32 *Copy_pu32Arr);
static void              MTIM_privateLatchTimeBase  (volatile MTIM_Registers_t *Copy_pReg,
                                                     u16 Copy_u16Psc, u32 Copy_u32Arr);
static void              MTIM_privateDispatch       (MTIM_Timer_t Copy_Timer, u8 Copy_u8EventMask);

/* ========================================================================= *
 *  INITIALIZATION
 * ========================================================================= */

MTIM_ReturnType_t MTIM_Init(void)
{
    u8  Local_u8Index;
    u8  Local_u8SeenTimers = 0u;      /* duplicate detection mask (8 bits) */
    u16 Local_u16Psc;
    u32 Local_u32Arr;

    /* ---- PHASE 1: validate the WHOLE table before touching silicon ---- */
    for (Local_u8Index = 0u; Local_u8Index < (u8)MTIM_NUM_CONFIGURED_TIMERS; Local_u8Index++)
    {
        const MTIM_ConfigEntry_t *Local_CfgRow = &MTIM_TimersConfig[Local_u8Index];
        MTIM_ReturnType_t Local_ReturnStatus = MTIM_OK;

        if (MTIM_privateValidateTimer(Local_CfgRow->Timer) != MTIM_OK)
        {
            return MTIM_INVALID_CONFIG;
        }
        if ((u32)Local_CfgRow->Mode > (u32)MTIM_MODE_PWM)
        {
            return MTIM_INVALID_CONFIG;
        }
        if (Local_CfgRow->Mode == MTIM_MODE_PWM)     /* runtime-only mode */
        {
            return MTIM_INVALID_CONFIG;
        }
        if ((Local_u8SeenTimers & (u8)(1uL << (u32)Local_CfgRow->Timer)) != 0u)
        {
            return MTIM_INVALID_CONFIG;              /* duplicate row */
        }
        Local_u8SeenTimers |= (u8)(1uL << (u32)Local_CfgRow->Timer);

        /* Frequency feasibility pre-check against the compiled clock -
           a future edit that sets an impossible rate fails HERE, with
           the whole table still un-applied */
        if (Local_CfgRow->Mode == MTIM_MODE_PERIODIC)
        {
            Local_ReturnStatus = MTIM_privateSolvePeriodic(&MTIM_Instances[Local_CfgRow->Timer],
                                                           Local_CfgRow->FrequencyHz, &Local_u16Psc, &Local_u32Arr);
        }
        else if (Local_CfgRow->Mode == MTIM_MODE_FREE_RUN)
        {
            Local_ReturnStatus = MTIM_privateSolveFreeRun(&MTIM_Instances[Local_CfgRow->Timer],
                                                          Local_CfgRow->FrequencyHz, &Local_u16Psc, &Local_u32Arr);
        }
        else { /* NONE - nothing to solve */ }

        if (Local_ReturnStatus != MTIM_OK)
        {
            return MTIM_INVALID_CONFIG;
        }
    }

    /* ---- PHASE 2: apply (only after the table is proven clean) ---- */
    Global_u8IsInitialized = 1u;

    for (Local_u8Index = 0u; Local_u8Index < (u8)MTIM_NUM_CONFIGURED_TIMERS; Local_u8Index++)
    {
        const MTIM_ConfigEntry_t *Local_CfgRow = &MTIM_TimersConfig[Local_u8Index];
        MTIM_Instance_t *Local_pInst = &MTIM_Instances[Local_CfgRow->Timer];

        if (Local_CfgRow->Mode == MTIM_MODE_NONE)
        {
            Local_pInst->ActiveMode         = MTIM_MODE_NONE;   /* state only */
            Local_pInst->ActiveFrequencyHz  = 0u;
            continue;
        }

        if (Local_CfgRow->Mode == MTIM_MODE_PERIODIC)
        {
            (void)MTIM_privateSolvePeriodic(Local_pInst, Local_CfgRow->FrequencyHz, &Local_u16Psc, &Local_u32Arr);
        }
        else
        {
            (void)MTIM_privateSolveFreeRun(Local_pInst, Local_CfgRow->FrequencyHz, &Local_u16Psc, &Local_u32Arr);
        }

        CLR_BIT(Local_pInst->Reg->CR1, MTIM_CR1_CEN);        /* stop          */
        MTIM_privateLatchTimeBase(Local_pInst->Reg, Local_u16Psc, Local_u32Arr);

        Local_pInst->ActiveMode        = Local_CfgRow->Mode;
        Local_pInst->ActiveFrequencyHz = Local_CfgRow->FrequencyHz;
        /* Callbacks/DIER preserved on re-Init (MEXTI convention). */
    }

    return MTIM_OK;
}

/* ========================================================================= *
 *  TIME-BASE CONFIGURATION
 * ========================================================================= */

MTIM_ReturnType_t MTIM_TimeBaseConfig(MTIM_Timer_t Copy_Timer, u32 Copy_u32TickHz)
{
    MTIM_Instance_t *Local_pInst;
    u16 Local_u16Psc;
    u32 Local_u32Arr;
    MTIM_ReturnType_t Local_ReturnStatus;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    if (Global_u8IsInitialized == 0u)                     { return MTIM_NOT_INITIALIZED; }

    Local_pInst = &MTIM_Instances[Copy_Timer];

    /* Mode state machine: same-mode re-config (rate change) is legal;
       any other active mode is the one-function-per-timer violation */
    if ((Local_pInst->ActiveMode != MTIM_MODE_NONE) &&
        (Local_pInst->ActiveMode != MTIM_MODE_FREE_RUN))
    {
        return MTIM_INVALID_STATE;
    }

    Local_ReturnStatus = MTIM_privateSolveFreeRun(Local_pInst, Copy_u32TickHz, &Local_u16Psc, &Local_u32Arr);
    if (Local_ReturnStatus != MTIM_OK) { return Local_ReturnStatus; }

    CLR_BIT(Local_pInst->Reg->CR1, MTIM_CR1_CEN);
    MTIM_privateLatchTimeBase(Local_pInst->Reg, Local_u16Psc, Local_u32Arr);

    Local_pInst->ActiveMode        = MTIM_MODE_FREE_RUN;
    Local_pInst->ActiveFrequencyHz = Copy_u32TickHz;
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_PeriodicConfig(MTIM_Timer_t Copy_Timer, u32 Copy_u32FrequencyHz)
{
    MTIM_Instance_t *Local_pInst;
    u16 Local_u16Psc;
    u32 Local_u32Arr;
    MTIM_ReturnType_t Local_ReturnStatus;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    if (Global_u8IsInitialized == 0u)                     { return MTIM_NOT_INITIALIZED; }

    Local_pInst = &MTIM_Instances[Copy_Timer];

    if ((Local_pInst->ActiveMode != MTIM_MODE_NONE) &&
        (Local_pInst->ActiveMode != MTIM_MODE_PERIODIC))
    {
        return MTIM_INVALID_STATE;
    }

    Local_ReturnStatus = MTIM_privateSolvePeriodic(Local_pInst, Copy_u32FrequencyHz, &Local_u16Psc, &Local_u32Arr);
    if (Local_ReturnStatus != MTIM_OK) { return Local_ReturnStatus; }

    CLR_BIT(Local_pInst->Reg->CR1, MTIM_CR1_CEN);
    MTIM_privateLatchTimeBase(Local_pInst->Reg, Local_u16Psc, Local_u32Arr);

    Local_pInst->ActiveMode        = MTIM_MODE_PERIODIC;
    Local_pInst->ActiveFrequencyHz = Copy_u32FrequencyHz;
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_Start(MTIM_Timer_t Copy_Timer)
{
    MTIM_Instance_t *Local_pInst;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    if (Global_u8IsInitialized == 0u)                     { return MTIM_NOT_INITIALIZED; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    if (Local_pInst->ActiveMode == MTIM_MODE_NONE)
    {
        return MTIM_INVALID_STATE;      /* nothing configured to start */
    }
    SET_BIT(Local_pInst->Reg->CR1, MTIM_CR1_CEN);
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_Stop(MTIM_Timer_t Copy_Timer)
{
    MTIM_Instance_t *Local_pInst;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    CLR_BIT(Local_pInst->Reg->CR1, MTIM_CR1_CEN);    /* CNT freezes; state kept */
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_DeInit(MTIM_Timer_t Copy_Timer)
{
    MTIM_Instance_t *Local_pInst;
    u8 Local_u8Channel;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }

    Local_pInst = &MTIM_Instances[Copy_Timer];

    CLR_BIT(Local_pInst->Reg->CR1, MTIM_CR1_CEN);
    Local_pInst->Reg->DIER = 0u;                       /* disarm everything    */
    for (Local_u8Channel = 0u; Local_u8Channel < 4u; Local_u8Channel++)
    {
        Local_pInst->CaptureCallback[Local_u8Channel] = NULL;
    }
    Local_pInst->UpdateCallback       = NULL;
    Local_pInst->ActiveMode           = MTIM_MODE_NONE;
    Local_pInst->ActiveFrequencyHz    = 0u;
    /* NVIC lines deliberately NOT disabled (shared vectors) */
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_GetCounter(MTIM_Timer_t Copy_Timer, u32 *Copy_pu32Counter)
{
    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    if (Copy_pu32Counter == (u32 *)0)                    { return MTIM_INVALID_PARAM; }

    *Copy_pu32Counter = MTIM_Instances[Copy_Timer].Reg->CNT;
    return MTIM_OK;
}

/* ========================================================================= *
 *  UPDATE EVENTS (interrupt + DMA request)
 * ========================================================================= */

MTIM_ReturnType_t MTIM_EnableUpdateInterrupt(MTIM_Timer_t Copy_Timer, MTIM_UpdateCallback_t Copy_pfCallback)
{
    MTIM_Instance_t *Local_pInst;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    if (Global_u8IsInitialized == 0u)                     { return MTIM_NOT_INITIALIZED; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    if (Local_pInst->ActiveMode == MTIM_MODE_NONE)
    {
        return MTIM_INVALID_STATE;        /* no time-base configured yet */
    }

    /* NO stale-UIF wipe - deliberate, documented contrast with MEXTI:
       a pending UIF at arm time is a REAL elapsed period (periodic
       semantics honor it; EXTI's edge semantics wipe instead). Init
       already cleared the UIF its UG produced. */
    Local_pInst->UpdateCallback = Copy_pfCallback;    /* NULL = ignored events */
    SET_BIT(Local_pInst->Reg->DIER, MTIM_DIER_UIE);

    {
        MNVIC_ReturnType_t Local_EnumStatus = MNVIC_EnableInterrupt(Local_pInst->UpdateIrq);
        if (Local_EnumStatus != MNVIC_OK) { return MTIM_INVALID_STATE; }
    }
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_DisableUpdateInterrupt(MTIM_Timer_t Copy_Timer)
{
    MTIM_Instance_t *Local_pInst;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    CLR_BIT(Local_pInst->Reg->DIER, MTIM_DIER_UIE);
    Local_pInst->UpdateCallback = NULL;
    /* Ungated disarm (the library principle); NVIC line NOT disabled
       (shared vectors for the TIM1/9/10/11 family). */
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_EnableUpdateDMARequest(MTIM_Timer_t Copy_Timer)
{
    MTIM_Instance_t *Local_pInst;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    if (Global_u8IsInitialized == 0u)                     { return MTIM_NOT_INITIALIZED; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    if (Local_pInst->ActiveMode != MTIM_MODE_PERIODIC)
    {
        return MTIM_INVALID_STATE;    /* the audio pattern is periodic-paced;
                                         free-run UDE would fire per wrap */
    }

    /* UDE: a DMA request, NOT a CPU interrupt - no NVIC, no ISR. Each
       update event paces one MDMA word (the corrected Option-b
       mechanism; MDMA owns the stream setup). */
    SET_BIT(Local_pInst->Reg->DIER, MTIM_DIER_UDE);
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_DisableUpdateDMARequest(MTIM_Timer_t Copy_Timer)
{
    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    CLR_BIT(MTIM_Instances[Copy_Timer].Reg->DIER, MTIM_DIER_UDE);
    return MTIM_OK;
}

/* ========================================================================= *
 *  PWM
 * ========================================================================= */

MTIM_ReturnType_t MTIM_PWMStart(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                u32 Copy_u32FrequencyHz, u16 Copy_u16DutyPermille,
                                MTIM_PWMPolarity_t Copy_Polarity)
{
    MTIM_Instance_t          *Local_pInst;
    volatile MTIM_Registers_t *Local_pReg;
    u16 Local_u16Psc;
    u32 Local_u32Arr;
    u32 Local_u32Ccr;
    MTIM_ReturnType_t Local_ReturnStatus;
    u32 Local_u32CcmrShift;    /* channel's byte within its CCMR register */
    u32 Local_u32CcmrMask;
    u8  Local_u8FirstActivation = 0u;  /* NONE->PWM this call, vs. adding a
                                           channel to an already-running PWM
                                           timer - only the former may force
                                           EGR.UG (see the guarded re-latch
                                           below)                            */

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK)          { return MTIM_INVALID_TIMER; }
    if (MTIM_privateValidateChannel(Copy_Timer, Copy_Channel) != MTIM_OK) { return MTIM_INVALID_CHANNEL; }
    if (Global_u8IsInitialized == 0u)                              { return MTIM_NOT_INITIALIZED; }
    if (Copy_u16DutyPermille > 1000u)                              { return MTIM_INVALID_PARAM; }
    if ((u32)Copy_Polarity > (u32)MTIM_PWM_ACTIVE_LOW)             { return MTIM_INVALID_PARAM; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    Local_pReg  = Local_pInst->Reg;

    /* Mode state machine: NONE -> become PWM; PWM same frequency ->
       add a channel; anything else (incl. PWM at a DIFFERENT rate) ->
       INVALID_STATE. One PSC/ARR pair per counter: the rule is silicon. */
    if (Local_pInst->ActiveMode == MTIM_MODE_NONE)
    {
        Local_ReturnStatus = MTIM_privateSolvePeriodic(Local_pInst, Copy_u32FrequencyHz, &Local_u16Psc, &Local_u32Arr);
        if (Local_ReturnStatus != MTIM_OK) { return Local_ReturnStatus; }

        CLR_BIT(Local_pReg->CR1, MTIM_CR1_CEN);
        MTIM_privateLatchTimeBase(Local_pReg, Local_u16Psc, Local_u32Arr);
        SET_BIT(Local_pReg->CR1, MTIM_CR1_ARPE);      /* ARR preload on  */

        Local_pInst->ActiveMode        = MTIM_MODE_PWM;
        Local_pInst->ActiveFrequencyHz = Copy_u32FrequencyHz;
        Local_u8FirstActivation        = 1u;
    }
    else if (Local_pInst->ActiveMode == MTIM_MODE_PWM)
    {
        if (Local_pInst->ActiveFrequencyHz != Copy_u32FrequencyHz)
        {
            return MTIM_INVALID_STATE;    /* second frequency on one ARR */
        }
        /* same frequency: fall through, add the channel */
    }
    else
    {
        return MTIM_INVALID_STATE;        /* FREE_RUN/PERIODIC occupied */
    }

    /* ---- channel configuration (register-shape aware, no copy-paste) ----
       CH1/CH3 live in CCMR1/CCMR2 low byte, CH2/CH4 in the high byte. */
    Local_u32CcmrShift = ((u32)Copy_Channel & 1u) * 8u;
    Local_u32CcmrMask  = 0xFFu << Local_u32CcmrShift;

    if ((u32)Copy_Channel < 2u)
    {
        Local_pReg->CCMR1 = (Local_pReg->CCMR1 & ~Local_u32CcmrMask)
                          | (MTIM_CCMR_OC_PWM1_PE << Local_u32CcmrShift);
    }
    else
    {
        Local_pReg->CCMR2 = (Local_pReg->CCMR2 & ~Local_u32CcmrMask)
                          | (MTIM_CCMR_OC_PWM1_PE << Local_u32CcmrShift);
    }

    /* CCER: enable + polarity (CCxE at ch*4, CCxP at ch*4+1) */
    Local_pReg->CCER = (Local_pReg->CCER & ~MTIM_CCER_CCX_MASK(Copy_Channel))
                     | (MTIM_CCER_CCXE(Copy_Channel)
                        | ((u32)Copy_Polarity * MTIM_CCER_CCXP(Copy_Channel)));

    /* TIM1 ONLY: BDTR.MOE - without it, ALL TIM1 outputs are gated off
       (reset 0) and PWM produces SILENTLY nothing. The classic TIM1
       trap; this driver cannot be bitten by it. AOE stays 0: software
       re-arms after a break (we never enable break, so MOE persists). */
    if (Local_pInst->IsAdvanced != 0u)
    {
        SET_BIT(Local_pReg->BDTR, MTIM_BDTR_MOE);
    }

    /* Duty -> CCR (64-bit math: duty * (ARR+1) can exceed u32 at
       32-bit ARR). CCR preloaded (OCxPE) -> takes effect next period,
       glitch-free. duty=1000 -> CCR=ARR+1 -> permanently active. */
    Local_u32Arr = (Local_pInst->Is32Bit != 0u) ? 0xFFFFFFFFul : 0xFFFFu;
    (void)MTIM_privateSolvePeriodic(Local_pInst, Copy_u32FrequencyHz, &Local_u16Psc, &Local_u32Arr);
    Local_u32Ccr = (u32)(((u64)Copy_u16DutyPermille * ((u64)Local_u32Arr + 1u)) / 1000u);
    (&Local_pReg->CCR1)[Copy_Channel] = Local_u32Ccr;

    /* Re-latch ONLY on first activation (NONE -> PWM this call): this
       forces the just-written CCR live immediately, so the very first
       PWM cycle already reflects the requested duty instead of waiting
       up to one period for a natural update event.
       DELIBERATELY SKIPPED on the add-a-channel path (ActiveMode was
       already PWM): EGR.UG zeroes CNT unconditionally, which would
       reset the phase of every channel ALREADY running on this timer -
       an audible/visible glitch on an active channel every time a new
       channel is added. Interrupts are still not armed for PWM either
       way, so this guard only changes the output-glitch behavior, not
       interrupt safety. */
    if (Local_u8FirstActivation != 0u)
    {
        Local_pReg->EGR = MTIM_FLAG(MTIM_EGR_UG);
        Local_pReg->SR  = ~MTIM_FLAG(MTIM_SR_UIF);
    }

    SET_BIT(Local_pReg->CR1, MTIM_CR1_CEN);            /* running        */
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_PWMSetDuty(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                  u16 Copy_u16DutyPermille)
{
    MTIM_Instance_t           *Local_pInst;
    volatile MTIM_Registers_t *Local_pReg;
    u16 Local_u16Psc;
    u32 Local_u32Arr;
    u32 Local_u32Ccr;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK)          { return MTIM_INVALID_TIMER; }
    if (MTIM_privateValidateChannel(Copy_Timer, Copy_Channel) != MTIM_OK) { return MTIM_INVALID_CHANNEL; }
    if (Copy_u16DutyPermille > 1000u)                              { return MTIM_INVALID_PARAM; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    if (Local_pInst->ActiveMode != MTIM_MODE_PWM)
    {
        return MTIM_INVALID_STATE;
    }
    Local_pReg = Local_pInst->Reg;

    /* Re-solve the ARR from the active frequency (deterministic - same
       solver, same clock model) instead of caching it. */
    Local_u32Arr = (Local_pInst->Is32Bit != 0u) ? 0xFFFFFFFFul : 0xFFFFu;
    (void)MTIM_privateSolvePeriodic(Local_pInst, Local_pInst->ActiveFrequencyHz, &Local_u16Psc, &Local_u32Arr);

    Local_u32Ccr = (u32)(((u64)Copy_u16DutyPermille * ((u64)Local_u32Arr + 1u)) / 1000u);
    (&Local_pReg->CCR1)[Copy_Channel] = Local_u32Ccr;   /* preloaded: takes
                                                           effect next period */
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_PWMStop(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel)
{
    volatile MTIM_Registers_t *Local_pReg;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK)                 { return MTIM_INVALID_TIMER; }
    if (MTIM_privateValidateChannel(Copy_Timer, Copy_Channel) != MTIM_OK) { return MTIM_INVALID_CHANNEL; }

    Local_pReg = MTIM_Instances[Copy_Timer].Reg;
    Local_pReg->CCER &= ~MTIM_CCER_CCX_MASK(Copy_Channel);   /* CCxE off; the
                                   timer keeps running for other channels */
    return MTIM_OK;
}

/* ========================================================================= *
 *  INPUT CAPTURE
 * ========================================================================= */

/* Shared channel-arming core: configures one capture channel with an
   explicit input SELECT (own TIx, or TI1-shared for the both-edges
   pattern), edge, callback. Callers validate mode/availability. */
static MTIM_ReturnType_t MTIM_privateArmCapture(MTIM_Instance_t *Copy_pInst,
                                                MTIM_Channel_t Copy_Channel,
                                                u32 Copy_u32InputSelect,
                                                MTIM_CaptureEdge_t Copy_Edge,
                                                MTIM_CaptureCallback_t Copy_pfCallback)
{
    volatile MTIM_Registers_t *Local_pReg = Copy_pInst->Reg;
    u32 Local_u32CcmrShift = ((u32)Copy_Channel & 1u) * 8u;
    u32 Local_u32CcmrMask  = 0xFFu << Local_u32CcmrShift;
    u32 Local_u32InputField = Copy_u32InputSelect;      /* CCxS raw encoding   */

    /* CCMR byte: input select in CCxS [1:0]; ICxPSC/ICxF stay 0
       (no prescaler, no filter - NEC edges are ms-scale; the filter
       exists for noisy lines, add a knob if ever needed) */
    if ((u32)Copy_Channel < 2u)
    {
        Local_pReg->CCMR1 = (Local_pReg->CCMR1 & ~Local_u32CcmrMask)
                          | (Local_u32InputField << Local_u32CcmrShift);
    }
    else
    {
        Local_pReg->CCMR2 = (Local_pReg->CCMR2 & ~Local_u32CcmrMask)
                          | (Local_u32InputField << Local_u32CcmrShift);
    }

    /* CCER: CCxE enable + edge (CCxP: 0=rising, 1=falling for inputs) */
    Local_pReg->CCER = (Local_pReg->CCER & ~MTIM_CCER_CCX_MASK(Copy_Channel))
                     | (MTIM_CCER_CCXE(Copy_Channel)
                        | ((u32)Copy_Edge * MTIM_CCER_CCXP(Copy_Channel)));

    /* EXTI-semantics wipe: an edge that arrived BEFORE capture was
       configured is not the consumer's event (contrast the update-event
       no-wipe decision - different event classes, different rules). */
    Local_pReg->SR = ~MTIM_FLAG(MTIM_CC_IF_BIT(Copy_Channel));

    Copy_pInst->CaptureCallback[Copy_Channel] = Copy_pfCallback;
    SET_BIT(Local_pReg->DIER, 1u + (u32)Copy_Channel);
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_CaptureStart(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                    MTIM_CaptureEdge_t Copy_Edge,
                                    MTIM_CaptureCallback_t Copy_pfCallback)
{
    MTIM_Instance_t *Local_pInst;
    MNVIC_ReturnType_t Local_EnumStatus;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK)                  { return MTIM_INVALID_TIMER; }
    if (MTIM_privateValidateChannel(Copy_Timer, Copy_Channel) != MTIM_OK)  { return MTIM_INVALID_CHANNEL; }
    if (Global_u8IsInitialized == 0u)                                      { return MTIM_NOT_INITIALIZED; }
    if ((u32)Copy_Edge > (u32)MTIM_CAPTURE_FALLING)                        { return MTIM_INVALID_PARAM; }

    Local_pInst = &MTIM_Instances[Copy_Timer];

    /* Capture rides on the FREE-RUN time-base - loud ordering: the
       measurement base must exist before edges mean anything */
    if (Local_pInst->ActiveMode != MTIM_MODE_FREE_RUN)
    {
        return MTIM_INVALID_STATE;
    }

    (void)MTIM_privateArmCapture(Local_pInst, Copy_Channel, MTIM_CCMR_IN_TIx,
                                 Copy_Edge, Copy_pfCallback);

    Local_EnumStatus = MNVIC_EnableInterrupt(Local_pInst->CaptureIrq);
    if (Local_EnumStatus != MNVIC_OK) { return MTIM_INVALID_STATE; }
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_CaptureStartBothEdges(MTIM_Timer_t Copy_Timer,
                                             MTIM_CaptureCallback_t Copy_pfRising,
                                             MTIM_CaptureCallback_t Copy_pfFalling)
{
    MTIM_Instance_t *Local_pInst;
    MNVIC_ReturnType_t Local_EnumStatus;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK) { return MTIM_INVALID_TIMER; }
    if (Global_u8IsInitialized == 0u)                     { return MTIM_NOT_INITIALIZED; }

    Local_pInst = &MTIM_Instances[Copy_Timer];

    if (Local_pInst->ActiveMode != MTIM_MODE_FREE_RUN)
    {
        return MTIM_INVALID_STATE;
    }
    if (Local_pInst->ChannelCount < 2u)
    {
        return MTIM_INVALID_CHANNEL;      /* TIM10/11: single-channel silicon */
    }
    if ((Local_pInst->CaptureCallback[MTIM_CH1] != NULL) ||
        (Local_pInst->CaptureCallback[MTIM_CH2] != NULL))
    {
        return MTIM_INVALID_STATE;        /* channels already capturing     */
    }

    /* THE NEC PATTERN: CH1 on TI1 rising; CH2 ALSO on TI1 (CC2S=10 -
       the one-pin/two-channel trick), falling. Marks AND spaces from
       one pin, no slave-mode machinery, counter stays free-running. */
    (void)MTIM_privateArmCapture(Local_pInst, MTIM_CH1, MTIM_CCMR_IN_TIx,
                                 MTIM_CAPTURE_RISING,  Copy_pfRising);
    (void)MTIM_privateArmCapture(Local_pInst, MTIM_CH2, MTIM_CCMR_IN_TI1_SHARED,
                                 MTIM_CAPTURE_FALLING, Copy_pfFalling);

    Local_EnumStatus = MNVIC_EnableInterrupt(Local_pInst->CaptureIrq);
    if (Local_EnumStatus != MNVIC_OK) { return MTIM_INVALID_STATE; }
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_CaptureStop(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel)
{
    MTIM_Instance_t *Local_pInst;

    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK)                 { return MTIM_INVALID_TIMER; }
    if (MTIM_privateValidateChannel(Copy_Timer, Copy_Channel) != MTIM_OK) { return MTIM_INVALID_CHANNEL; }

    Local_pInst = &MTIM_Instances[Copy_Timer];
    Local_pInst->Reg->CCER &= ~MTIM_CCER_CCX_MASK(Copy_Channel);
    CLR_BIT(Local_pInst->Reg->DIER, 1u + (u32)Copy_Channel);
    Local_pInst->CaptureCallback[Copy_Channel] = NULL;
    /* NVIC NOT disabled - shared vectors (TIM1/9/10/11), MEXTI principle */
    return MTIM_OK;
}

MTIM_ReturnType_t MTIM_GetCapture(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel,
                                  u32 *Copy_pu32Timestamp)
{
    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK)                 { return MTIM_INVALID_TIMER; }
    if (MTIM_privateValidateChannel(Copy_Timer, Copy_Channel) != MTIM_OK) { return MTIM_INVALID_CHANNEL; }
    if (Copy_pu32Timestamp == (u32 *)0)                                   { return MTIM_INVALID_PARAM; }

    *Copy_pu32Timestamp = (&MTIM_Instances[Copy_Timer].Reg->CCR1)[Copy_Channel];
    return MTIM_OK;
}

/* ========================================================================= *
 *  INTERRUPT SERVICE ROUTINES
 * ========================================================================= *
 *  Eight vector entries (strong symbols overriding the startup weak
 *  aliases). TIM2..TIM5 have dedicated vectors; the TIM1/9/10/11 family
 *  shares four: TIM9 rides 24 ENTIRELY, TIM10 rides 25, TIM11 rides 26
 *  (their whole interrupt sets, one vector each), while TIM1's update
 *  pends 25 and its CC events pend 27 - hence the event-mask parameter:
 *  vector 27 services TIM1's CAPTURE events only, vector 25 its UPDATE
 *  only, so a coincidence of both flags can never double-callback.
 *
 *  STORM-SAFETY (the library invariant, timer edition): this driver can
 *  arm exactly UIE / CC1-4IE / UDE(no IRQ). Break and trigger/com
 *  interrupts are NEVER armable - BIE/TIE have no enable path in this
 *  code - so vectors 24/26 fire only for their small timer, and no
 *  unserviced flag is reachable. Every armable flag has a guaranteed
 *  write-0 clear inside its dispatch branch.
 * ========================================================================= */

static void MTIM_privateDispatch(MTIM_Timer_t Copy_Timer, u8 Copy_u8EventMask)
{
    MTIM_Instance_t           *Local_pInst = &MTIM_Instances[Copy_Timer];
    volatile MTIM_Registers_t *Local_pReg  = Local_pInst->Reg;

    /* SR/DIER snapshots at entry - SR read is side-effect-free (no
       coupled clears in this peripheral), so this is detection-stability
       hygiene, not a bug fix: flags set after the snapshot re-pend and
       are serviced on the next entry. Conservative, never lossy. */
    u32 Local_u32Sr   = Local_pReg->SR;
    u32 Local_u32Dier = Local_pReg->DIER;
    u8  Local_u8Channel;

    /* ---- Update event ---- */
    if (((Copy_u8EventMask & (u8)MTIM_PRIVATE_EV_UPDATE) != 0u) &&
        ((Local_u32Sr & MTIM_FLAG(MTIM_SR_UIF)) != 0u) &&
        ((Local_u32Dier & MTIM_FLAG(MTIM_DIER_UIE)) != 0u))
    {
        Local_pReg->SR = ~MTIM_FLAG(MTIM_SR_UIF);   /* W0C, one bit only */

        if (Local_pInst->UpdateCallback != NULL)
        {
            Local_pInst->UpdateCallback();
        }
    }

    /* ---- Capture channels (per-instance channel count gates absent
            registers - TIM10/11's nonexistent CCR3/CCMR2 are structurally
            unreachable here) ---- */
    if ((Copy_u8EventMask & (u8)MTIM_PRIVATE_EV_CAPTURE) != 0u)
    {
        for (Local_u8Channel = 0u; Local_u8Channel < Local_pInst->ChannelCount; Local_u8Channel++)
        {
            if (((Local_u32Sr   >> MTIM_CC_IF_BIT(Local_u8Channel)) & 1u) != 0u)
            if (((Local_u32Dier >> MTIM_CC_IF_BIT(Local_u8Channel)) & 1u) != 0u)
            {
                u32 Local_u32Timestamp = (&Local_pReg->CCR1)[Local_u8Channel];
                u32 Local_u32ClearMask = MTIM_FLAG(MTIM_CC_IF_BIT(Local_u8Channel));

                /* Overcapture (a second edge before CCR was read): clear
                   it in the same store; the delivered timestamp is the
                   OLDER edge - impossible at NEC rates, handled anyway. */
                if (((Local_u32Sr >> MTIM_CC_OF_BIT(Local_u8Channel)) & 1u) != 0u)
                {
                    Local_u32ClearMask |= MTIM_FLAG(MTIM_CC_OF_BIT(Local_u8Channel));
                }

                Local_pReg->SR = ~Local_u32ClearMask;    /* W0C */

                if (Local_pInst->CaptureCallback[Local_u8Channel] != NULL)
                {
                    Local_pInst->CaptureCallback[Local_u8Channel](Local_u32Timestamp);
                }
            }
        }
    }
}

/* ---- Public vector-table entries (linkage only - not API) ---- */

void TIM2_IRQHandler(void)               { MTIM_privateDispatch(MTIM_TIM2, (u8)MTIM_PRIVATE_EV_ALL); }
void TIM3_IRQHandler(void)               { MTIM_privateDispatch(MTIM_TIM3, (u8)MTIM_PRIVATE_EV_ALL); }
void TIM4_IRQHandler(void)               { MTIM_privateDispatch(MTIM_TIM4, (u8)MTIM_PRIVATE_EV_ALL); }
void TIM5_IRQHandler(void)               { MTIM_privateDispatch(MTIM_TIM5, (u8)MTIM_PRIVATE_EV_ALL); }

/* TIM1 break vector carries TIM9 entirely; TIM1 break is never armed
   by this driver (BIE has no enable path) - dispatch TIM9 only. */
void TIM1_BRK_TIM9_IRQHandler(void)      { MTIM_privateDispatch(MTIM_TIM9,  (u8)MTIM_PRIVATE_EV_ALL); }

/* TIM1 update vector carries TIM10 entirely; TIM1's OWN update is
   serviced here too - UPDATE events only (its CC events pend 27). */
void TIM1_UP_TIM10_IRQHandler(void)
{
    MTIM_privateDispatch(MTIM_TIM1,  (u8)MTIM_PRIVATE_EV_UPDATE);
    MTIM_privateDispatch(MTIM_TIM10, (u8)MTIM_PRIVATE_EV_ALL);
}

/* TIM11's vector; TIM1 trigger/commutation never armed - TIM11 only. */
void TIM1_TRG_COM_TIM11_IRQHandler(void) { MTIM_privateDispatch(MTIM_TIM11, (u8)MTIM_PRIVATE_EV_ALL); }

/* TIM1 capture/compare ONLY - the event mask is what prevents a
   UIF+CCxIF coincidence from double-callbacking across vectors 25/27. */
void TIM1_CC_IRQHandler(void)            { MTIM_privateDispatch(MTIM_TIM1,  (u8)MTIM_PRIVATE_EV_CAPTURE); }

/* ========================================================================= *
 *  PRIVATE HELPERS
 * ========================================================================= */

static MTIM_ReturnType_t MTIM_privateValidateTimer(MTIM_Timer_t Copy_Timer)
{
    if ((u32)Copy_Timer >= (u32)MTIM_INSTANCE_COUNT)   /* u32 cast first:
                                                          the enum-garbage rule */
    {
        return MTIM_INVALID_TIMER;
    }
    return MTIM_OK;
}

static MTIM_ReturnType_t MTIM_privateValidateChannel(MTIM_Timer_t Copy_Timer, MTIM_Channel_t Copy_Channel)
{
    /* Self-contained bounds check (does not trust the caller to have
       already validated Copy_Timer): every current call site does
       call MTIM_privateValidateTimer first, but that's caller
       discipline, not something this function can rely on to stay
       memory-safe if a future call site ever skips it. */
    if (MTIM_privateValidateTimer(Copy_Timer) != MTIM_OK)
    {
        return MTIM_INVALID_TIMER;
    }
    if ((u32)Copy_Channel >= (u32)MTIM_Instances[Copy_Timer].ChannelCount)
    {
        return MTIM_INVALID_CHANNEL;    /* absent on this timer's shape */
    }
    return MTIM_OK;
}

/* ---- Periodic solver: maximize ARR (duty resolution), smallest PSC.
        Total counts D = CLK / f. pscDiv = ceil(D / (maxArr+1)) is the
        smallest divider keeping ARR in range; PSC = pscDiv-1 (16-bit,
        checked), ARR = D/pscDiv - 1. Integer floors: error <= one
        count per period (ppm-scale). u64 math throughout - the 16-bit
        corner (65536 * 65536) overflows u32. ---------------------------- */
static MTIM_ReturnType_t MTIM_privateSolvePeriodic(const MTIM_Instance_t *Copy_pInst, u32 Copy_u32FrequencyHz,
                                                   u16 *Copy_pu16Psc, u32 *Copy_pu32Arr)
{
    u64 Local_u64Divider;
    u64 Local_u64MaxArr = (Copy_pInst->Is32Bit != 0u) ? 0xFFFFFFFFull : 0xFFFFull;

    if (Copy_u32FrequencyHz == 0u)
    {
        return MTIM_INVALID_PARAM;             /* division by zero guard */
    }
    Local_u64Divider = (u64)Copy_pInst->ClockHz / (u64)Copy_u32FrequencyHz;
    if (Local_u64Divider < 2u)
    {
        return MTIM_INVALID_PARAM;             /* target >= clock: impossible */
    }

    Local_u64Divider = (Local_u64Divider + Local_u64MaxArr) / (Local_u64MaxArr + 1u);  /* ceil */
    if (Local_u64Divider > 65536u)
    {
        return MTIM_INVALID_PARAM;             /* too slow even at PSC max */
    }

    *Copy_pu16Psc = (u16)(Local_u64Divider - 1u);
    *Copy_pu32Arr = (u32)(((u64)Copy_pInst->ClockHz / (u64)Copy_u32FrequencyHz) / Local_u64Divider - 1u);
    return MTIM_OK;
}

/* ---- Free-run solver: PSC = CLK/tick - 1 (exact when divisible, else
        floored with ppm error), ARR = maximum. The 16->84 MHz flip is
        PSC-only by construction (the timer doc's property). ---------- */
static MTIM_ReturnType_t MTIM_privateSolveFreeRun(const MTIM_Instance_t *Copy_pInst, u32 Copy_u32TickHz,
                                                  u16 *Copy_pu16Psc, u32 *Copy_pu32Arr)
{
    if (Copy_u32TickHz == 0u)
    {
        return MTIM_INVALID_PARAM;
    }
    if ((u64)Copy_u32TickHz > (u64)Copy_pInst->ClockHz)
    {
        return MTIM_INVALID_PARAM;             /* tick faster than the clock */
    }

    *Copy_pu16Psc = (u16)(((u64)Copy_pInst->ClockHz / (u64)Copy_u32TickHz) - 1u);
    *Copy_pu32Arr = (Copy_pInst->Is32Bit != 0u) ? 0xFFFFFFFFul : 0xFFFFul;
    return MTIM_OK;
}

/* ---- The UG latch sequence (design principle 4): PSC written to the
        register goes live only at an update event - write, then force
        EGR.UG (immediate latch + CNT zeroed), then clear the UIF the
        UG just SET (skip the clear = instant spurious interrupt on
        the first arming; the MEXTI stale-pending trap, timer edition). */
static void MTIM_privateLatchTimeBase(volatile MTIM_Registers_t *Copy_pReg, u16 Copy_u16Psc, u32 Copy_u32Arr)
{
    Copy_pReg->PSC = (u32)Copy_u16Psc;
    Copy_pReg->ARR = Copy_u32Arr;
    Copy_pReg->EGR = MTIM_FLAG(MTIM_EGR_UG);
    Copy_pReg->SR  = ~MTIM_FLAG(MTIM_SR_UIF);
}
