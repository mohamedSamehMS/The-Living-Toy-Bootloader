/*
 * MTIM_config.h
 *
 *  Configuration of the MTIM driver: the per-timer static assignment
 *  table - the timer-allocation decision record, made executable.
 *
 *  Each row locks ONE timer's boot mode and frequency (the mode
 *  state machine's starting point). Channel-level work (PWM duty,
 *  capture arming) is runtime API - it needs pins, callbacks and
 *  duty values the table cannot know.
 *
 *  Frequencies are consumed through the solver against the PUBLISHED
 *  CLOCK MODEL, so the 16 -> 84 MHz PLL flip recomputes every PSC/ARR
 *  here at COMPILE time - the timer doc's "PSC-only" property.
 *
 *  Reference: RM0368 Rev 6, Ch 13 (TIM2-5, pp 316-375);
 *             Ch 12 (TIM1, pp 243-315); Ch 14 (TIM9-11, pp 376-421)
 *
 *  Revision history:
 *    Rev 1 - first release (family driver, 8 instances, 4 modes)
 *
 *  Created on: Sep 12, 2026
 *      Author: dell
 */

#ifndef MTIM_CONFIG_H_
#define MTIM_CONFIG_H_

#include "STD_TYPES.h"
#include <MCAL/RCC/MRCC_interface.h>   /* published clock model: timer
                                           clocks with the x2 APB rule
                                           already applied                */

/* =====================================================================
 *  1. Timer-clock constants - DERIVED from the published model
 *
 *  TIM2..TIM5 sit on APB1; TIM1/TIM9/10/11 on APB2. Never hardcode.
 * ===================================================================== */
#define MTIM_APB1_TIMER_CLK_HZ     (MRCC_APB1_TIMER_CLK_KHZ * 1000UL)
#define MTIM_APB2_TIMER_CLK_HZ     (MRCC_APB2_TIMER_CLK_KHZ * 1000UL)

/* =====================================================================
 *  2. Modes  (the one-function-per-timer rule, as data)
 *
 *      NONE     - not configured at boot; runtime API still usable
 *      FREE_RUN - PSC solved for the tick rate, ARR = maximum, counter
 *                 counts and wraps forever; the capture time-base
 *      PERIODIC - PSC + ARR solved for an exact interrupt/DMA period
 *      PWM      - set at runtime by MTIM_PWMStart (never a boot row:
 *                 PWM needs a channel, a frequency AND a duty)
 * ===================================================================== */
typedef enum
{
    MTIM_MODE_NONE = 0,
    MTIM_MODE_FREE_RUN,
    MTIM_MODE_PERIODIC,
    MTIM_MODE_PWM          /* runtime-only - reject in table validation */
} MTIM_Mode_t;

/** One row per timer - this table IS the project's timer allocation */
typedef struct
{
    MTIM_Timer_t   Timer;        /* which timer                             */
    MTIM_Mode_t    Mode;         /* boot mode (NONE = reserved, runtime)   */
    u32            FrequencyHz;  /* FREE_RUN: tick rate | PERIODIC: period */
} MTIM_ConfigEntry_t;

/* ---- Project 4 allocation (per the definitive timer document) -------
 *  TIM4 : matrix POV tick    - 400 Hz periodic (2.5 ms line dwell)
 *  TIM5 : NEC IR capture     - 1 MHz free-run (hardware timestamps)
 *  TIM3 : backlight PWM      - runtime/channel-level (carrier TBD)
 *  TIM9 : audio DMA trigger  - runtime; frequency comes with the
 *                              audio-path decision (Option b leaning)
 *  TIM10/11: conditional passive-buzzer PWM - pending buzzer check
 *  TIM2 : reserve (Attract-Mode open question - see its doc note)
 *  TIM1 : reserved, supported-basic (MOE handled, advanced out of scope)
 * -------------------------------------------------------------------- */
static const MTIM_ConfigEntry_t MTIM_TimersConfig[] =
{
    { MTIM_TIM1,  MTIM_MODE_NONE,     0u        },
    { MTIM_TIM2,  MTIM_MODE_NONE,     0u        },
    { MTIM_TIM3,  MTIM_MODE_NONE,     0u        },
    { MTIM_TIM4,  MTIM_MODE_PERIODIC, 400u      },
    { MTIM_TIM5,  MTIM_MODE_FREE_RUN, 1000000u  },
    { MTIM_TIM9,  MTIM_MODE_NONE,     0u        },
    { MTIM_TIM10, MTIM_MODE_NONE,     0u        },
    { MTIM_TIM11, MTIM_MODE_NONE,     0u        }
};

#define MTIM_NUM_CONFIGURED_TIMERS   (sizeof(MTIM_TimersConfig) / sizeof(MTIM_TimersConfig[0]))

/* This header defines a struct with MTIM types - include the interface
   (member TYPES are not lazily expandable; MEXTI's forced include). */
#include <MCAL/TIM/MTIM_interface.h>

#endif /* MTIM_CONFIG_H_ */
