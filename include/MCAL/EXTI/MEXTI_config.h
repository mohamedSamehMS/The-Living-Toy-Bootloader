/*
 * MEXTI_config.h
 *
 *  Configuration of the MEXTI driver: the per-line static init table.
 *
 *  Each row wires ONE EXTI line: its GPIO port source (SYSCFG EXTICR
 *  routing), its trigger edge, and whether it starts enabled or
 *  disabled. MEXTI_Init() applies the whole table at boot.
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 10: EXTI / SYSCFG
 *
 *  INCLUDE NOTE: this header includes MEXTI_interface.h — a genuine
 *  (and forced) dependency, unlike MUSART's removed include: the
 *  table's STRUCT MEMBER TYPES (MEXTI_LineNumber_t, MGPIO_Port_t,
 *  MEXTI_Trigger_t) are not lazily expandable, they must resolve at
 *  parse time. Macro enum NAMES resolve at use time; member TYPES
 *  do not.
 *
 *  The table is `static` in a header — this file must be included by
 *  MEXTI_program.c ONLY (the MEXTI-style pattern MSPI's config also
 *  follows). Including it anywhere else silently duplicates it.
 *
 *  Revision history:
 *    Rev 1 — original (not self-contained; silent default paths)
 *    Rev 2 — self-contained includes; MGPIO_Port_t replaces the
 *            duplicate MEXTI_Port_t; rows are runtime-validated by
 *            Init (bad line/port/trigger/state now fail loudly)
 *
 *  Created on: Aug 13, 2026
 *      Author: dell
 */

#ifndef MEXTI_CONFIG_H_
#define MEXTI_CONFIG_H_

#include "STD_TYPES.h"
#include <MCAL/GPIO/MGPIO_interface.h>   /* MGPIO_Port_t — the port field
                                            REUSES the MGPIO type (values
                                            0/1/2 = A/B/C = EXTICR
                                            encodings — asserted in
                                            MEXTI_program.c)             */
#include <MCAL/EXTI/MEXTI_interface.h>   /* line + trigger enum types     */

/* Line start states */
#define MEXTI_ENABLED    1u
#define MEXTI_DISABLED   0u

/** One row per EXTI line this project uses — edit to match wiring */
typedef struct
{
    MEXTI_LineNumber_t   Line;        /* EXTI line 0..15                       */
    MGPIO_Port_t         PortSource;  /* which port drives this line (A/B/C)   */
    MEXTI_Trigger_t      Trigger;     /* edge(s) that pend the line            */
    u8                   InitState;   /* MEXTI_ENABLED / MEXTI_DISABLED        */
} MEXTI_ConfigEntry_t;

/* Current Project 4 wiring:
 *   Lines 0..3  : Port A, falling — buttons to GND, internal pull-ups
 *                 (each has its OWN NVIC IRQ — positions 6..10)
 *   Line 10/13  : Port B, falling — additional buttons, SHARING the
 *                 EXTI15_10 IRQ (position 40)
 *   Everything else: configured but disabled, reserved for expansion
 *   PC13 users beware: line 13 routed to PORTB here, NOT port C — the
 *   onboard LED pin is PC13, and the PC13/PC15 pins carry LSE duties. */
static const MEXTI_ConfigEntry_t MEXTI_LinesConfig[] =
{
    { MEXTI_LINE0,  MGPIO_PORTA, MEXTI_TRIGGER_FALLING, MEXTI_ENABLED },
    { MEXTI_LINE1,  MGPIO_PORTA, MEXTI_TRIGGER_FALLING, MEXTI_DISABLED },
    { MEXTI_LINE2,  MGPIO_PORTA, MEXTI_TRIGGER_FALLING, MEXTI_DISABLED },
    { MEXTI_LINE3,  MGPIO_PORTA, MEXTI_TRIGGER_FALLING, MEXTI_DISABLED },
    { MEXTI_LINE4,  MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED },
    { MEXTI_LINE5,  MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED },
    { MEXTI_LINE6,  MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED },
    { MEXTI_LINE7,  MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED },
    { MEXTI_LINE8,  MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED },
    { MEXTI_LINE9,  MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED },
    { MEXTI_LINE10, MGPIO_PORTB, MEXTI_TRIGGER_FALLING, MEXTI_DISABLED },
    { MEXTI_LINE11, MGPIO_PORTB, MEXTI_TRIGGER_FALLING, MEXTI_DISABLED },
    { MEXTI_LINE12, MGPIO_PORTB, MEXTI_TRIGGER_FALLING, MEXTI_DISABLED },
    { MEXTI_LINE13, MGPIO_PORTB, MEXTI_TRIGGER_FALLING, MEXTI_DISABLED },
    { MEXTI_LINE14, MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED },
    { MEXTI_LINE15, MGPIO_PORTA, MEXTI_TRIGGER_RISING,  MEXTI_DISABLED }
};

#define MEXTI_NUM_CONFIGURED_LINES   (sizeof(MEXTI_LinesConfig) / sizeof(MEXTI_LinesConfig[0]))

#endif /* MEXTI_CONFIG_H_ */
