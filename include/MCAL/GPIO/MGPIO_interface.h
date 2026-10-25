/*
 * MGPIO_interface.h
 *
 *  PUBLIC interface of the MGPIO driver. HAL and Application code must
 *  include ONLY this file (never MGPIO_private.h, never the config).
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Ch. 8: GPIO
 *
 *  DEPENDENCY — clock enable is MANDATORY before any MGPIO call:
 *      MRCC_EnablePeripheralClock(MRCC_GPIOA);   // or GPIOB / GPIOC
 *  Register writes to an un-clocked port are silently ignored by
 *  hardware: no bus error, no fault — just nothing happens. That is
 *  the classic "no error anywhere, nothing works" trap.
 *
 *  CONCURRENCY CONTRACT — every function that changes pin state does
 *  so with a single register store (BSRR, or a direct ODR/AFR word
 *  write). All functions are therefore safe to call from ISRs and safe
 *  against ISRs touching the same port. There is deliberately NO
 *  read-modify-write pin API.
 *
 *  Pins used as interrupts (buttons) are configured HERE (mode +
 *  pull); the interrupt side is owned by MEXTI, not this driver.
 *
 *  NOTE — replaces MGPIO_interfacee.h (the double-'e' filename typo,
 *  which had leaked into the include guard and every #include line).
 *  Rename the include in every consumer and delete the old file.
 *
 *  Created on: Aug 8, 2026
 *      Author: dell
 */

#ifndef MGPIO_INTERFACE_H_
#define MGPIO_INTERFACE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Driver status codes — same convention as MRCC_ReturnType_t
 *
 *  C enums are NOT type-safe: garbage values compile cleanly. So every
 *  function range-checks EVERY enum parameter and returns a loud error
 *  instead of the old behavior — silently doing nothing (or worse,
 *  half-applying a configuration).
 *
 *  Pin validity is a separate error from parameter validity: a pin can
 *  be a perfectly legal 0..15 that this package simply doesn't have.
 * ===================================================================== */
typedef enum
{
    MGPIO_OK = 0,           /* Operation completed successfully          */
    MGPIO_INVALID_PORT,     /* Port id outside the supported set         */
    MGPIO_INVALID_PIN,      /* Pin not available on this package/port
                               (see the masks in MGPIO_config.h)         */
    MGPIO_INVALID_PARAM     /* Out-of-range mode/type/speed/pull/value/
                               alt-function, or a NULL pointer argument  */
} MGPIO_ReturnType_t;

/* =====================================================================
 *  Port identifier
 *
 *  Values are indices into the internal port table in MGPIO_program.c
 *  and MUST stay 0,1,2 in this order. GPIOH is intentionally absent:
 *  PH0/PH1 are the HSE crystal pins on the WeAct Black Pill (see
 *  MGPIO_private.h for how to add a port).
 * ===================================================================== */
typedef enum
{
    MGPIO_PORTA = 0,        /* index 0 */
    MGPIO_PORTB,            /* index 1 */
    MGPIO_PORTC             /* index 2 */
} MGPIO_Port_t;

/* =====================================================================
 *  Pin identifier
 *
 *  Values equal the pin NUMBER and are used directly as bit positions
 *  and shift counts by the driver — keep them 0..15 in order.
 * ===================================================================== */
typedef enum
{
    MGPIO_PIN_0 = 0, MGPIO_PIN_1,  MGPIO_PIN_2,  MGPIO_PIN_3,
    MGPIO_PIN_4,     MGPIO_PIN_5,  MGPIO_PIN_6,  MGPIO_PIN_7,
    MGPIO_PIN_8,     MGPIO_PIN_9,  MGPIO_PIN_10, MGPIO_PIN_11,
    MGPIO_PIN_12,    MGPIO_PIN_13, MGPIO_PIN_14, MGPIO_PIN_15
} MGPIO_Pin_t;

/* =====================================================================
 *  Pin mode (MODER encodings — values written to hardware as-is)
 *
 *  ANALOG disables the digital input/output cell (also the correct
 *  state for unused pins: lowest power, no floating-input noise).
 * ===================================================================== */
typedef enum
{
    MGPIO_MODE_INPUT = 0,           /* MODER 00 — digital input        */
    MGPIO_MODE_OUTPUT,              /* MODER 01 — general purpose out  */
    MGPIO_MODE_ALT_FUNC,            /* MODER 10 — peripheral mapped    */
    MGPIO_MODE_ANALOG               /* MODER 11 — analog / unused      */
} MGPIO_Mode_t;

/* =====================================================================
 *  Output type (OTYPER encoding) — applies in OUTPUT and ALT_FUNC modes
 * ===================================================================== */
typedef enum
{
    MGPIO_OUTPUT_PUSH_PULL = 0,     /* OTYPER 0 — driven both ways     */
    MGPIO_OUTPUT_OPEN_DRAIN         /* OTYPER 1 — driven low, floats
                                       high (I2C-style lines; needs a
                                       pull-up, often external)       */
} MGPIO_OutputType_t;

/* =====================================================================
 *  Output speed (OSPEEDR encodings)
 *
 *  Slew/drive strength vs. toggle rate — see the datasheet's "output
 *  speed vs. toggle frequency" guidance. Practical rule of thumb:
 *      LEDs, slow control lines      -> MGPIO_SPEED_LOW   (less EMI)
 *      SPI to the TFT later at high
 *      clocks, USART pins            -> MGPIO_SPEED_HIGH / VERY_HIGH
 * ===================================================================== */
typedef enum
{
    MGPIO_SPEED_LOW = 0,            /* OSPEEDR 00 */
    MGPIO_SPEED_MEDIUM,             /* OSPEEDR 01 */
    MGPIO_SPEED_HIGH,               /* OSPEEDR 10 */
    MGPIO_SPEED_VERY_HIGH           /* OSPEEDR 11 */
} MGPIO_Speed_t;

/* =====================================================================
 *  Pull configuration (PUPDR encodings) — valid in ANY pin mode
 *
 *  Inputs: defines the idle level of floating lines.
 *  Outputs / AF: internal pull usable on open-drain-style lines.
 *  Analog: a pull only wastes current — see
 *  MGPIO_ANALOG_MODE_CLEARS_PULL in MGPIO_config.h.
 * ===================================================================== */
typedef enum
{
    MGPIO_NO_PULL = 0,              /* PUPDR 00 */
    MGPIO_PULL_UP,                  /* PUPDR 01 — ~40 kOhm typical     */
    MGPIO_PULL_DOWN                 /* PUPDR 10 — ~40 kOhm typical     */
} MGPIO_Pull_t;

/* =====================================================================
 *  Pin output value
 * ===================================================================== */
typedef enum
{
    MGPIO_PIN_LOW = 0,              /* logic 0 */
    MGPIO_PIN_HIGH                  /* logic 1 */
} MGPIO_PinValue_t;

/* =====================================================================
 *  Alternate function select (AFR encodings, AF0..AF15)
 *
 *  The 4-bit AFR field can select AF0..AF15. Which functions exist on
 *  WHICH pins is per-pin — always confirm in RM0368's alternate
 *  function table. Typical F401 assignments:
 *      AF0  : SWD/JTAG debug (reset default on PA13/PA14), MCO
 *      AF1/2/3 : TIM1/TIM2, TIM3/TIM4/TIM5, TIM9/TIM10/TIM11
 *      AF4  : I2C1/2/3
 *      AF5  : SPI1/SPI2          AF6 : SPI3
 *      AF7  : USART1/USART2      AF8 : USART6
 *      AF10 : OTG_FS             AF12 : SDIO
 *      AF15 : EVENTOUT
 *  AF9/AF11/AF13/AF14 are not used on the F401.
 * ===================================================================== */
typedef enum
{
    MGPIO_AF0 = 0,  MGPIO_AF1,  MGPIO_AF2,  MGPIO_AF3,
    MGPIO_AF4,      MGPIO_AF5,  MGPIO_AF6,  MGPIO_AF7,
    MGPIO_AF8,      MGPIO_AF9,  MGPIO_AF10, MGPIO_AF11,
    MGPIO_AF12,     MGPIO_AF13, MGPIO_AF14, MGPIO_AF15
} MGPIO_AltFunc_t;

/* =====================================================================
 *  API
 * ===================================================================== */

/* ---- Mode ------------------------------------------------------------ */
/* Write the pin's MODER field. Call this FIRST for every pin, before
   output/pull/AF configuration.
   When MGPIO_ANALOG_MODE_CLEARS_PULL is 1 in config, selecting ANALOG
   also clears the pin's PUPDR to no-pull. */
MGPIO_ReturnType_t MGPIO_SetPinMode(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                    MGPIO_Mode_t  Copy_Mode);

/* ---- Output stage ----------------------------------------------------- */
/* Write OTYPER + OSPEEDR for the pin. Meaningful in OUTPUT mode AND in
   ALT_FUNC mode — alternate functions drive the pin through this same
   output stage (SPI MOSI wants push-pull, I2C AF pins want
   open-drain). No effect while the pin is INPUT/ANALOG.
   Both parameters are validated BEFORE anything is written, so a bad
   value can never half-apply a configuration (the old driver wrote
   OSPEEDR even when the type was invalid). */
MGPIO_ReturnType_t MGPIO_SetPinOutputConfig(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                            MGPIO_OutputType_t Copy_OutputType,
                                            MGPIO_Speed_t       Copy_Speed);

/* ---- Pulls ------------------------------------------------------------ */
/* Write the pin's PUPDR field. Valid in ANY mode — the old name
   (SetInputCONFIG) implied input-only, which was wrong. */
MGPIO_ReturnType_t MGPIO_SetPinPull(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                    MGPIO_Pull_t Copy_Pull);

/* ---- Alternate function ------------------------------------------------ */
/* Write the pin's AFR field (AFRL for pins 0..7, AFRH for 8..15).
   Takes effect only while the pin's mode is MGPIO_MODE_ALT_FUNC.
   PA13/PA14 are rejected by the pin-validity mask, so this function
   can no longer silently disconnect SWD and brick debugging. */
MGPIO_ReturnType_t MGPIO_SetPinAltFunction(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                           MGPIO_AltFunc_t Copy_AltFunc);

/* ---- Writing pin state --------------------------------------------------- */

/* Set or reset ONE pin, atomically, via a single BSRR write.
   Replaces BOTH the old ODR read-modify-write version (racy against
   ISRs on the same port) and the old separate "Atomic" variant — one
   API, always safe. The value reaches the physical pin when the pin is
   in OUTPUT (or open-drain AF) mode; otherwise it just updates the ODR
   latch. */
MGPIO_ReturnType_t MGPIO_SetPinValue(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                     MGPIO_PinValue_t Copy_PinValue);

/* Atomically write EVERY pin selected by Copy_u16Mask to the matching
   bit of Copy_u16Value — ONE BSRR write total:
        Mask bit N = 1 -> pin N is driven to (Value >> N) & 1
        Mask bit N = 0 -> pin N is completely untouched
   The set field (Value & Mask) and reset field (~Value & Mask) are
   disjoint by construction, so BSRR's "reset wins over set" rule can
   never fire. This is the primitive for the Music Maker's 8-bit R2R
   DAC (mask 0x00FF on one port): all bits change in a single bus
   write, so no intermediate glitch codes hit the audio output.
   Replaces the old racy SetPortByte. */
MGPIO_ReturnType_t MGPIO_WritePins(MGPIO_Port_t Copy_Port,
                                   u16 Copy_u16Mask, u16 Copy_u16Value);

/* Write the full 16-bit ODR word in one atomic store. ALL pins of the
   port update — use when there are no "untouched" pins to preserve. */
MGPIO_ReturnType_t MGPIO_SetPortValue(MGPIO_Port_t Copy_Port, u16 Copy_u16Value);

/* ---- Reading pin state ---------------------------------------------------- */

/* Read ONE pin from IDR. IDR returns the ACTUAL pin level — for output
   pins this is the real state on the pin (short/overload detection),
   not merely the ODR latch. The level is delivered through the pointer
   and the function returns only status: an invalid port/pin can no
   longer masquerade as a pin level (the old version returned -1,
   which arrived as 255 in a u8). */
MGPIO_ReturnType_t MGPIO_GetPinValue(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                     u8 *Copy_pu8Value);

/* Read the full 16-bit IDR word — whole-port input snapshot in one
   read (button-matrix row/column scanning). */
MGPIO_ReturnType_t MGPIO_GetPortValue(MGPIO_Port_t Copy_Port, u16 *Copy_pu16Value);

/* =====================================================================
 *  Usage examples
 *
 *  LED on the WeAct onboard LED (PC13, ACTIVE LOW):
 *      MRCC_EnablePeripheralClock(MRCC_GPIOC);
 *      MGPIO_SetPinMode        (MGPIO_PORTC, MGPIO_PIN_13, MGPIO_MODE_OUTPUT);
 *      MGPIO_SetPinOutputConfig(MGPIO_PORTC, MGPIO_PIN_13,
 *                               MGPIO_OUTPUT_PUSH_PULL, MGPIO_SPEED_LOW);
 *      MGPIO_SetPinValue       (MGPIO_PORTC, MGPIO_PIN_13, MGPIO_PIN_LOW);
 *
 *  Button on PA0 (input, internal pull-up; MEXTI owns the interrupt):
 *      MRCC_EnablePeripheralClock(MRCC_GPIOA);
 *      MGPIO_SetPinMode (MGPIO_PORTA, MGPIO_PIN_0, MGPIO_MODE_INPUT);
 *      MGPIO_SetPinPull (MGPIO_PORTA, MGPIO_PIN_0, MGPIO_PULL_UP);
 *      MGPIO_GetPinValue(MGPIO_PORTA, MGPIO_PIN_0, &Local_u8Level);
 *
 *  USART1 TX on PA9 (AF7):
 *      MGPIO_SetPinMode       (MGPIO_PORTA, MGPIO_PIN_9, MGPIO_MODE_ALT_FUNC);
 *      MGPIO_SetPinOutputConfig(MGPIO_PORTA, MGPIO_PIN_9,
 *                               MGPIO_OUTPUT_PUSH_PULL, MGPIO_SPEED_HIGH);
 *      MGPIO_SetPinAltFunction(MGPIO_PORTA, MGPIO_PIN_9, MGPIO_AF7);
 *
 *  8-bit R2R DAC on PB0..PB7 — glitch-free single-write update:
 *      MGPIO_WritePins(MGPIO_PORTB, 0x00FFu, Local_u8Sample);
 * ===================================================================== */

#endif /* MGPIO_INTERFACE_H_ */
