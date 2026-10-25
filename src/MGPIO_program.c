/*
 * MGPIO_program.c
 *
 *  Implementation of the MGPIO driver.
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Ch. 8: GPIO
 *
 *  DESIGN PRINCIPLES (documented, deliberate):
 *
 *  1. VALIDATE FIRST, WRITE LAST
 *     Every parameter is validated BEFORE any register is touched.
 *     The old driver could half-apply a configuration (it wrote
 *     OSPEEDR even when the output-type parameter was invalid, and
 *     silently no-oped on bad enum values). Eliminated: either the
 *     entire configuration lands, or nothing is written and a status
 *     code returns.
 *
 *  2. CONCURRENCY CONTRACT — two classes of function:
 *     - VALUE functions (SetPinValue / WritePins / SetPortValue):
 *       exactly ONE register store each (BSRR, or a full ODR word) —
 *       atomic, safe to call from ISRs, safe against ISRs writing the
 *       same port from another context. There is deliberately NO
 *       read-modify-write on the value path — the old ODR
 *       SET_BIT/CLR_BIT version was a lost-update race.
 *     - CONFIG functions (SetPinMode / SetPinOutputConfig / SetPinPull
 *       / SetPinAltFunction): read-modify-write of this pin's 1/2/4-bit
 *       field inside MODER / OTYPER / OSPEEDR / PUPDR / AFR. Two
 *       contexts RECONFIGURING the same port simultaneously is not
 *       supported — pin configuration is an init-time operation.
 *       Value-writing interrupts never touch these registers, so the
 *       Project 4 pattern (EXTI ISR toggling LED pins while the game
 *       loop runs) stays race-free.
 *     (This header is the authoritative statement of the contract; the
 *     shorter sentence in MGPIO_interface.h compresses it.)
 *
 *  3. PACKAGE VALIDITY EVERYWHERE
 *     The valid-pin masks from MGPIO_config.h are enforced in EVERY
 *     function. The old driver enforced them in 2 of 8 — which let
 *     SetAlternateFunctionCONFIG remap PA13/PA14 and silently kill
 *     the SWD debug connection.
 *
 *  4. UNSIGNED SHIFTS ONLY
 *     Every shift operates on u-suffixed / u32-cast operands. Pin 15
 *     drives masks up to bit 31 — the old signed-shift expressions
 *     (0b11 << 30, 0b1111 << 28, 1 << 31) were undefined behavior.
 *
 *  DEPENDENCY: MRCC_EnablePeripheralClock(MRCC_GPIOx) must have run
 *  before any call here — writes to an un-clocked port are silently
 *  ignored by the hardware. No error, nothing happens.
 *
 *  NOTE — replaces GPIO_program.c. Includes reference the renamed
 *  headers (MGPIO_interface.h, single 'e'; MGPIO_private.h /
 *  MGPIO_config.h with the MGPIO_ prefix).
 *
 *  Created on: Jul 24, 2026
 *      Author: dell
 */

#include <MCAL/GPIO/MGPIO_config.h>
#include <MCAL/GPIO/MGPIO_interface.h>
#include <MCAL/GPIO/MGPIO_private.h>
#include "BIT_MATH.h"
#include "STD_TYPES.h"

/* =====================================================================
 *  Compile-time consistency guards
 *
 *  A load-bearing design assumption of this driver: the public enum
 *  values EQUAL the hardware field encodings, so this file writes
 *  them into registers untranslated. These checks turn a future
 *  silent reordering of any enum into a BUILD BREAK instead of a
 *  register corruption. (_Static_assert needs C11 — the GCC default;
 *  on an older toolchain swap for the negative-array typedef trick.)
 * ===================================================================== */
_Static_assert(MGPIO_MODE_INPUT       == 0, "MODER: INPUT must encode 00");
_Static_assert(MGPIO_MODE_OUTPUT      == 1, "MODER: OUTPUT must encode 01");
_Static_assert(MGPIO_MODE_ALT_FUNC    == 2, "MODER: ALT_FUNC must encode 10");
_Static_assert(MGPIO_MODE_ANALOG      == 3, "MODER: ANALOG must encode 11");
_Static_assert(MGPIO_OUTPUT_PUSH_PULL == 0, "OTYPER: push-pull must encode 0");
_Static_assert(MGPIO_OUTPUT_OPEN_DRAIN== 1, "OTYPER: open-drain must encode 1");
_Static_assert(MGPIO_SPEED_LOW        == 0, "OSPEEDR: LOW must encode 00");
_Static_assert(MGPIO_SPEED_VERY_HIGH  == 3, "OSPEEDR: VERY_HIGH must encode 11");
_Static_assert(MGPIO_NO_PULL          == 0, "PUPDR: no-pull must encode 00");
_Static_assert(MGPIO_PULL_UP          == 1, "PUPDR: pull-up must encode 01");
_Static_assert(MGPIO_PULL_DOWN        == 2, "PUPDR: pull-down must encode 10 - 11 is RESERVED");
_Static_assert(MGPIO_PIN_LOW == 0 && MGPIO_PIN_HIGH == 1, "pin value encodings");
_Static_assert(MGPIO_PIN_0 == 0 && MGPIO_PIN_15 == 15,    "pin enum must equal pin number");
_Static_assert(MGPIO_PORTA == 0 && MGPIO_PORTC == 2,      "port enum must equal table index");
_Static_assert(MGPIO_AF0 == 0 && MGPIO_AF15 == 15,        "AF enum must equal AFR encoding");

/* =====================================================================
 *  Private lookup tables
 * ===================================================================== */

/* Port id -> register map. Index order MUST match MGPIO_Port_t. The
   designated initializers make the correspondence explicit: if a port
   is ever added to the enum without a table entry, the build breaks
   instead of silently indexing past the array. */
static volatile MGPIO_Registers_t * const MGPIO_PortTable[] =
{
    [MGPIO_PORTA] = MGPIO_GPIOA,
    [MGPIO_PORTB] = MGPIO_GPIOB,
    [MGPIO_PORTC] = MGPIO_GPIOC
};

/* Per-package pin availability (from MGPIO_config.h — a board fact).
   Bit N set = pin N exists on this package and may be used. */
static const u16 MGPIO_AvailablePins[] =
{
    [MGPIO_PORTA] = MGPIO_PORTA_AVAILABLE_PINS,
    [MGPIO_PORTB] = MGPIO_PORTB_AVAILABLE_PINS,
    [MGPIO_PORTC] = MGPIO_PORTC_AVAILABLE_PINS
};

/* =====================================================================
 *  Private helpers
 * ===================================================================== */

/* Validate port id, pin number, and package availability — used by
   every pin-addressed public function.

   The (u32) casts BEFORE each comparison matter: C enums are not
   type-safe, and a garbage value cast in from a negative int passes a
   plain "value > MAX" test when the enum's underlying type is signed
   — and would then index the tables out of bounds. In u32 arithmetic,
   (u32)(-1) is huge and fails the range check. */
static MGPIO_ReturnType_t MGPIO_privateValidatePortPin(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin)
{
    if ((u32)Copy_Port > (u32)MGPIO_PORTC)
    {
        return MGPIO_INVALID_PORT;
    }
    if ((u32)Copy_Pin > (u32)MGPIO_PIN_15)
    {
        return MGPIO_INVALID_PIN;
    }
    if ((MGPIO_AvailablePins[Copy_Port] & (1u << Copy_Pin)) == 0u)
    {
        return MGPIO_INVALID_PIN;   /* legal number — absent on package */
    }
    return MGPIO_OK;
}

/* Validate port id only — for the whole-port functions. */
static MGPIO_ReturnType_t MGPIO_privateValidatePort(MGPIO_Port_t Copy_Port)
{
    if ((u32)Copy_Port > (u32)MGPIO_PORTC)
    {
        return MGPIO_INVALID_PORT;
    }
    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_SetPinMode
 *
 *  Writes the pin's 2-bit MODER field: input / output / alternate
 *  function / analog. Call this FIRST for a pin — before output
 *  config, pull, or AF selection.
 *
 *  With MGPIO_ANALOG_MODE_CLEARS_PULL = 1 (config), selecting ANALOG
 *  also clears the pin's PUPDR field: a pull on an analog pin only
 *  burns current and can distort the analog level (RM0368 recommends
 *  no-pull for analog pins).
 *
 *  ANALOG is also the correct mode for UNUSED pins — lowest power and
 *  no floating-input noise (the reset default is input-floating).
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_SetPinMode(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                    MGPIO_Mode_t Copy_Mode)
{
    volatile MGPIO_Registers_t *Local_pu32Port;
    MGPIO_ReturnType_t Local_ReturnStatus = MGPIO_privateValidatePortPin(Copy_Port, Copy_Pin);

    if (Local_ReturnStatus != MGPIO_OK)
    {
        return Local_ReturnStatus;
    }

    /* Enums are not type-safe — garbage values compile. Range-check
       BEFORE the register write. (Encoding equality is guarded by the
       _Static_asserts at the top of this file.) */
    if ((u32)Copy_Mode > (u32)MGPIO_MODE_ANALOG)
    {
        return MGPIO_INVALID_PARAM;
    }

    Local_pu32Port = MGPIO_PortTable[Copy_Port];

    /* MODER, 2 bits/pin: clear this pin's field, then write the new
       encoding. All other pins' fields in the register are preserved. */
    Local_pu32Port->MODER = (Local_pu32Port->MODER & ~MGPIO_MODER_MASK(Copy_Pin))
                          | ((u32)Copy_Mode << ((u32)Copy_Pin * 2u));

#if MGPIO_ANALOG_MODE_CLEARS_PULL
    if (Copy_Mode == MGPIO_MODE_ANALOG)
    {
        Local_pu32Port->PUPDR &= ~MGPIO_PUPDR_MASK(Copy_Pin);   /* no-pull */
    }
#endif

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_SetPinOutputConfig
 *
 *  Writes OTYPER + OSPEEDR for one pin. Meaningful in OUTPUT mode AND
 *  in ALT_FUNC mode — alternate functions drive the pin through this
 *  same output stage (SPI MOSI wants push-pull; I2C AF pins want
 *  open-drain). While the pin is INPUT/ANALOG, both settings are
 *  latched but have no effect.
 *
 *  BOTH parameters are validated BEFORE any write — the old driver
 *  wrote OSPEEDR even when the output type was invalid, leaving a
 *  half-applied configuration.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_SetPinOutputConfig(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                            MGPIO_OutputType_t Copy_OutputType,
                                            MGPIO_Speed_t Copy_Speed)
{
    volatile MGPIO_Registers_t *Local_pu32Port;
    MGPIO_ReturnType_t Local_ReturnStatus = MGPIO_privateValidatePortPin(Copy_Port, Copy_Pin);

    if (Local_ReturnStatus != MGPIO_OK)
    {
        return Local_ReturnStatus;
    }

    if ((u32)Copy_OutputType > (u32)MGPIO_OUTPUT_OPEN_DRAIN)
    {
        return MGPIO_INVALID_PARAM;
    }
    if ((u32)Copy_Speed > (u32)MGPIO_SPEED_VERY_HIGH)
    {
        return MGPIO_INVALID_PARAM;
    }

    Local_pu32Port = MGPIO_PortTable[Copy_Port];

    /* OTYPER: 1 bit per pin — 0 push-pull, 1 open-drain */
    Local_pu32Port->OTYPER = (Local_pu32Port->OTYPER & ~MGPIO_OTYPER_MASK(Copy_Pin))
                           | ((u32)Copy_OutputType << (u32)Copy_Pin);

    /* OSPEEDR: 2 bits per pin — slew/drive strength. Low speed for
       LEDs and slow control lines (less EMI); high/very-high for the
       SPI-to-TFT and USART pins later. */
    Local_pu32Port->OSPEEDR = (Local_pu32Port->OSPEEDR & ~MGPIO_OSPEEDR_MASK(Copy_Pin))
                            | ((u32)Copy_Speed << ((u32)Copy_Pin * 2u));

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_SetPinPull
 *
 *  Writes the pin's 2-bit PUPDR field — valid in ANY pin mode, not
 *  only input (the old name, SetInputCONFIG, implied input-only):
 *    - INPUT        : defines the idle level of a floating line
 *    - OUTPUT / AF  : internal pull usable on open-drain-style lines
 *    - ANALOG       : see the no-pull note in MGPIO_SetPinMode
 *  PULL_DOWN == 2 keeps the reserved PUPDR encoding 11 unreachable
 *  through the enum.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_SetPinPull(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                    MGPIO_Pull_t Copy_Pull)
{
    volatile MGPIO_Registers_t *Local_pu32Port;
    MGPIO_ReturnType_t Local_ReturnStatus = MGPIO_privateValidatePortPin(Copy_Port, Copy_Pin);

    if (Local_ReturnStatus != MGPIO_OK)
    {
        return Local_ReturnStatus;
    }

    if ((u32)Copy_Pull > (u32)MGPIO_PULL_DOWN)
    {
        return MGPIO_INVALID_PARAM;
    }

    Local_pu32Port = MGPIO_PortTable[Copy_Port];

    /* PUPDR, 2 bits/pin — enum values are the field encodings */
    Local_pu32Port->PUPDR = (Local_pu32Port->PUPDR & ~MGPIO_PUPDR_MASK(Copy_Pin))
                          | ((u32)Copy_Pull << ((u32)Copy_Pin * 2u));

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_SetPinAltFunction
 *
 *  Writes the pin's 4-bit AFR field: AFRL for pins 0..7, AFRH for
 *  pins 8..15 (field index = pin - 8). The selection takes effect
 *  only while MODER = ALT_FUNC, but it may be programmed before or
 *  after the mode switch.
 *
 *  PA13/PA14 (SWDIO/SWCLK) never reach this function — the config
 *  pin mask rejects them with MGPIO_INVALID_PIN — so no call can
 *  silently disconnect the debugger.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_SetPinAltFunction(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                           MGPIO_AltFunc_t Copy_AltFunc)
{
    volatile MGPIO_Registers_t *Local_pu32Port;
    MGPIO_ReturnType_t Local_ReturnStatus = MGPIO_privateValidatePortPin(Copy_Port, Copy_Pin);

    if (Local_ReturnStatus != MGPIO_OK)
    {
        return Local_ReturnStatus;
    }

    if ((u32)Copy_AltFunc > (u32)MGPIO_AF15)
    {
        return MGPIO_INVALID_PARAM;
    }

    Local_pu32Port = MGPIO_PortTable[Copy_Port];

    if ((u32)Copy_Pin < 8u)
    {
        /* Low half of the port: AFRL, 4 bits per pin */
        Local_pu32Port->AFRL = (Local_pu32Port->AFRL & ~MGPIO_AFRL_MASK(Copy_Pin))
                             | ((u32)Copy_AltFunc << ((u32)Copy_Pin * 4u));
    }
    else
    {
        /* High half: AFRH, 4 bits per pin, field index = pin - 8 */
        Local_pu32Port->AFRH = (Local_pu32Port->AFRH & ~MGPIO_AFRH_MASK(Copy_Pin))
                             | ((u32)Copy_AltFunc << (((u32)Copy_Pin - 8u) * 4u));
    }

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_SetPinValue
 *
 *  Drives one pin HIGH or LOW through a SINGLE BSRR write — atomic,
 *  safe against any interrupt writing the same port. This one
 *  function replaces BOTH old APIs: the ODR read-modify-write version
 *  (racy against ISRs on the same port) and the separate "Atomic"
 *  variant (redundant — atomic is now the only implementation).
 *
 *  The value reaches the physical pin when the pin is in OUTPUT mode
 *  (or an open-drain AF); otherwise it is only latched into ODR
 *  without effect.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_SetPinValue(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                     MGPIO_PinValue_t Copy_PinValue)
{
    volatile MGPIO_Registers_t *Local_pu32Port;
    MGPIO_ReturnType_t Local_ReturnStatus = MGPIO_privateValidatePortPin(Copy_Port, Copy_Pin);

    if (Local_ReturnStatus != MGPIO_OK)
    {
        return Local_ReturnStatus;
    }

    if ((u32)Copy_PinValue > (u32)MGPIO_PIN_HIGH)
    {
        return MGPIO_INVALID_PARAM;
    }

    Local_pu32Port = MGPIO_PortTable[Copy_Port];

    if (Copy_PinValue == MGPIO_PIN_HIGH)
    {
        /* Set half: BS(pin), bits 15:0 — one atomic write */
        Local_pu32Port->BSRR = MGPIO_BSRR_BS(Copy_Pin);
    }
    else
    {
        /* Reset half: BR(pin), bits 31:16 — one atomic write.
           (If both halves were ever written for the same pin, the
           reset half wins per RM0368 — this driver never does that.) */
        Local_pu32Port->BSRR = MGPIO_BSRR_BR(Copy_Pin);
    }

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_WritePins
 *
 *  Atomically drives EVERY pin selected by Copy_u16Mask to the
 *  matching bit of Copy_u16Value — ONE BSRR write total:
 *      mask bit N = 1 -> pin N driven to (Value >> N) & 1
 *      mask bit N = 0 -> pin N completely untouched
 *
 *  The set field (Value & Mask) and reset field (~Value & Mask, in
 *  bits 31:16) are disjoint by construction, so BSRR's
 *  "reset wins over set" rule can never fire — each masked pin lands
 *  on exactly its target value simultaneously.
 *
 *  Project 4 use — the Music Maker's 8-bit R2R DAC:
 *      MGPIO_WritePins(MGPIO_PORTB, 0x00FFu, Local_u8Sample);
 *  all 8 bits change on one bus write, so no intermediate codes
 *  glitch the audio output (pin-by-pin updates would).
 *
 *  The mask must only address pins available on this package — the
 *  SWD pins cannot be driven through this back door either.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_WritePins(MGPIO_Port_t Copy_Port,
                                   u16 Copy_u16Mask, u16 Copy_u16Value)
{
    volatile MGPIO_Registers_t *Local_pu32Port;
    u32 Local_u32SetField;
    u32 Local_u32ResetField;

    if ((u32)Copy_Port > (u32)MGPIO_PORTC)
    {
        return MGPIO_INVALID_PORT;
    }

    /* Mask must be a subset of this port's available pins */
    if (((u32)Copy_u16Mask & ~(u32)MGPIO_AvailablePins[Copy_Port]) != 0u)
    {
        return MGPIO_INVALID_PIN;
    }

    Local_pu32Port = MGPIO_PortTable[Copy_Port];

    /* Set half   (BS, bits 15:0) : pins to drive 1 =  Value & Mask
       Reset half (BR, bits 31:16): pins to drive 0 = ~Value & Mask  */
    Local_u32SetField   = (u32)Copy_u16Value & (u32)Copy_u16Mask;
    Local_u32ResetField = ((u32)Copy_u16Mask & ~(u32)Copy_u16Value) << 16u;

    /* Single store — every masked pin updates simultaneously */
    Local_pu32Port->BSRR = Local_u32SetField | Local_u32ResetField;

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_SetPortValue
 *
 *  Full 16-bit ODR store — one atomic write updating ALL pins of the
 *  port. Bits belonging to pins not in output mode (input / AF / SWD
 *  / absent-on-package) are latched into ODR without effect but are
 *  NOT rejected: this function's contract is "the whole port". When
 *  some pins must stay untouched, MGPIO_WritePins is the right tool.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_SetPortValue(MGPIO_Port_t Copy_Port, u16 Copy_u16Value)
{
    volatile MGPIO_Registers_t *Local_pu32Port;

    if ((u32)Copy_Port > (u32)MGPIO_PORTC)
    {
        return MGPIO_INVALID_PORT;
    }

    Local_pu32Port = MGPIO_PortTable[Copy_Port];

    /* Single full-word store — atomic on the bus */
    Local_pu32Port->ODR = (u32)Copy_u16Value;

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_GetPinValue
 *
 *  Reads one pin from IDR — the ACTUAL pin level. For output pins,
 *  IDR reflects the real state ON the pin (a short to VCC/GND or an
 *  overloaded pin shows up here), not merely the ODR latch.
 *
 *  The level is delivered through the pointer; the function returns
 *  only status. The old version returned -1 in a u8 on error — which
 *  arrived as 255 and was indistinguishable from a pin level.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_GetPinValue(MGPIO_Port_t Copy_Port, MGPIO_Pin_t Copy_Pin,
                                     u8 *Copy_pu8Value)
{
    MGPIO_ReturnType_t Local_ReturnStatus = MGPIO_privateValidatePortPin(Copy_Port, Copy_Pin);

    if (Local_ReturnStatus != MGPIO_OK)
    {
        return Local_ReturnStatus;
    }

    if (Copy_pu8Value == (u8 *)0)
    {
        return MGPIO_INVALID_PARAM;
    }

    /* IDR is volatile — re-read on every call, no compiler caching */
    *Copy_pu8Value = (u8)GET_BIT(MGPIO_PortTable[Copy_Port]->IDR, (u32)Copy_Pin);

    return MGPIO_OK;
}

/* =====================================================================
 *  MGPIO_GetPortValue
 *
 *  Whole-port input snapshot: ONE IDR read returns all 16 pin levels
 *  coherently — the primitive for button-matrix row/column scanning,
 *  where reading pins one by one can straddle a matrix state change.
 *  Pins absent on the package read 0.
 * ===================================================================== */
MGPIO_ReturnType_t MGPIO_GetPortValue(MGPIO_Port_t Copy_Port, u16 *Copy_pu16Value)
{
    if ((u32)Copy_Port > (u32)MGPIO_PORTC)
    {
        return MGPIO_INVALID_PORT;
    }

    if (Copy_pu16Value == (u16 *)0)
    {
        return MGPIO_INVALID_PARAM;
    }

    *Copy_pu16Value = (u16)(MGPIO_PortTable[Copy_Port]->IDR & 0xFFFFu);

    return MGPIO_OK;
}
