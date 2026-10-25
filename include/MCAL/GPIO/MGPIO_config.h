/*
 * MGPIO_config.h
 *
 *  Configuration of the MGPIO driver: board/package facts and driver
 *  policy. No register definitions belong here (those live in private).
 *
 *  Reference: STM32F401CCU6 datasheet — LQFP48 pin availability
 *
 *  NOTE — replaces GPIO_config.h (renamed for prefix consistency).
 *  This file was empty before; the pin masks give it its first real
 *  job, which is making the driver portable between packages.
 *
 *  Created on: Jul 24, 2026
 *      Author: dell
 */

#ifndef MGPIO_CONFIG_H_
#define MGPIO_CONFIG_H_

/* =====================================================================
 *  1. Valid-pin masks per port — PACKAGE FACTS
 *
 *  Bit N set   = pin N exists on this package and may be used.
 *  Bit N clear = every MGPIO call for pin N fails fast with
 *                MGPIO_INVALID_PIN.
 *
 *  Values below are for the STM32F401CCU6 in LQFP48 (WeAct Black
 *  Pill). These masks are a POLICY guard, not silicon: hardware would
 *  let you write MODER for an absent pin's field — the driver refuses.
 *  Porting to a 64-pin F401 means updating exactly this file (plus
 *  adding ports, see MGPIO_private.h).
 * ===================================================================== */

/* PA0..PA12 usable.
   PA13 = SWDIO, PA14 = SWCLK — excluded so no MGPIO call can
   reconfigure them and silently kill the debug connection.
   PA15 = JTDI — excluded by the same debug policy; add 0x8000 to the
   mask if you accept SWD-only debugging and want the pin back.
   PA11/PA12 = USB OTG_FS DM/DP (Phase 2) — included: ordinary
   GPIO/AF pins whenever OTG is off. */
#define MGPIO_PORTA_AVAILABLE_PINS    (0x1FFFu)

/* PB0..PB15 all usable on LQFP48.
   Debug note: PB3 = JTDO/SWO and PB4 = NJTRST default to their debug
   alternate functions after reset; claiming them as GPIO works fine
   but disables SWO tracing / full JTAG — that is a per-project
   decision, so they stay available here. */
#define MGPIO_PORTB_AVAILABLE_PINS    (0xFFFFu)

/* PC13..PC15 only.
   PC13 = onboard user LED on the WeAct Black Pill (ACTIVE LOW).
   PC14/PC15 = OSC32_IN/OSC32_OUT — free as GPIO while no LSE crystal
   is fitted, but they become the 32.768 kHz LSE pins if a crystal is
   added for the Phase-1 RTC. Remove them from this mask at that point
   (mask 0xC000u) so nothing reconfigures the RTC's crystal pins. */
#define MGPIO_PORTC_AVAILABLE_PINS    (0xE000u)

/* =====================================================================
 *  2. Driver policy options
 * ===================================================================== */

/* 1 = MGPIO_SetPinMode(MGPIO_MODE_ANALOG) also clears the pin's PUPDR
   field to no-pull. A pull resistor on an analog pin only burns
   current and can distort readings; RM0368 recommends no-pull for
   analog pins. 0 = PUPDR left untouched by SetPinMode. */
#define MGPIO_ANALOG_MODE_CLEARS_PULL   1

#endif /* MGPIO_CONFIG_H_ */
