/*
 * MGPIO_private.h
 *
 *  PRIVATE header of the MGPIO driver — never include this from HAL or
 *  Application code. Contains ONLY hardware facts:
 *      - GPIO port base addresses
 *      - the per-port register map (offsets verified vs RM0368 §8.4)
 *      - per-pin field masks and BSRR bit encoders
 *
 *  All user-tunable options (per-package valid-pin masks, driver
 *  policy) live in MGPIO_config.h. All public types and the API live
 *  in MGPIO_interface.h.
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Ch. 8: GPIO
 *             STM32F401xC datasheet — pin availability per package
 *
 *  NOTE — replaces GPIO_private.h (renamed for the MGPIO_ prefix
 *  convention). Delete the old file from the tree.
 *
 *  Created on: Jul 24, 2026
 *      Author: dell
 */

#ifndef MGPIO_PRIVATE_H_
#define MGPIO_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  GPIO port base addresses (RM0368 §2.3, memory map)
 *
 *  All GPIO ports live on AHB1, spaced 0x400 apart. The F401 die has
 *  no GPIOF/GPIOG. GPIOD/E and GPIOH exist on larger packages:
 *      GPIOD : 0x40020C00   (64-pin+ packages)
 *      GPIOE : 0x40021000   (64-pin+ packages)
 *      GPIOH : 0x40021C00   (PH0/PH1 only)
 *  On the WeAct Black Pill (LQFP48) PH0/PH1 carry the 25 MHz HSE
 *  crystal, so GPIOH is deliberately NOT part of this driver's port
 *  enum. To port to a bigger package: add the base address here, an
 *  MGPIO_PORTx member in the interface enum, a port-table entry and a
 *  config pin-mask in MGPIO_program.c / MGPIO_config.h.
 * ===================================================================== */
#define MGPIO_GPIOA_BASE_ADDRESS      (0x40020000UL)
#define MGPIO_GPIOB_BASE_ADDRESS      (0x40020400UL)
#define MGPIO_GPIOC_BASE_ADDRESS      (0x40020800UL)

/* =====================================================================
 *  Per-port register map (RM0368 §8.4 — offsets verified)
 *
 *  The GPIO register block is contiguous from 0x00 to 0x24 — unlike
 *  RCC, no reserved gaps need declaring. Field meanings:
 *      MODER   : 2 bits per pin — 00 input, 01 output, 10 alternate
 *                function, 11 analog
 *      OTYPER  : 1 bit  per pin — 0 push-pull, 1 open-drain
 *      OSPEEDR : 2 bits per pin — 00 low .. 11 very high
 *      PUPDR   : 2 bits per pin — 00 none, 01 up, 10 down, 11 reserved
 *      IDR     : read-only ACTUAL pin level (real pin state, even for
 *                output pins — useful to detect shorts/overload)
 *      ODR     : output data latch. A full-word store is atomic; a
 *                per-bit read-modify-write is NOT (see BSRR below)
 *      BSRR    : atomic bit set/reset port — see BSRR section below
 *      LCKR    : configuration lock (unused by this driver)
 *      AFRL/H  : 4 bits per pin alternate-function select, low/high
 *                half of the port
 * ===================================================================== */
typedef struct
{
    volatile u32 MODER;      /* 0x00 : Mode register                             */
    volatile u32 OTYPER;     /* 0x04 : Output type register                       */
    volatile u32 OSPEEDR;    /* 0x08 : Output speed register                      */
    volatile u32 PUPDR;      /* 0x0C : Pull-up / pull-down register               */
    volatile u32 IDR;        /* 0x10 : Input data register (read-only)            */
    volatile u32 ODR;        /* 0x14 : Output data register                       */
    volatile u32 BSRR;       /* 0x18 : Bit set / reset register (atomic writes)   */
    volatile u32 LCKR;       /* 0x1C : Lock register (unused)                     */
    volatile u32 AFRL;       /* 0x20 : Alternate function low  (pins 0..7)        */
    volatile u32 AFRH;       /* 0x24 : Alternate function high (pins 8..15)       */
} MGPIO_Registers_t;

/* Register map access points — consumed by the port lookup table in
   MGPIO_program.c. Prefixed: the old bare GPIOA/GPIOB/GPIOC macros
   were unprefixed global namespace hazards. */
#define MGPIO_GPIOA   ((volatile MGPIO_Registers_t *) MGPIO_GPIOA_BASE_ADDRESS)
#define MGPIO_GPIOB   ((volatile MGPIO_Registers_t *) MGPIO_GPIOB_BASE_ADDRESS)
#define MGPIO_GPIOC   ((volatile MGPIO_Registers_t *) MGPIO_GPIOC_BASE_ADDRESS)

/* =====================================================================
 *  Enum values <-> hardware encodings
 *
 *  The public enums in MGPIO_interface.h are DELIBERATELY equal to the
 *  hardware field encodings, so MGPIO_program.c can write enum values
 *  straight into the register fields with no translation layer:
 *      MGPIO_MODE_INPUT / OUTPUT / ALT_FUNC / ANALOG -> MODER 00/01/10/11
 *      MGPIO_OUTPUT_PUSH_PULL / OPEN_DRAIN           -> OTYPER  0 / 1
 *      MGPIO_SPEED_LOW .. VERY_HIGH                  -> OSPEEDR 00/01/10/11
 *      MGPIO_NO_PULL / PULL_UP / PULL_DOWN           -> PUPDR  00/01/10
 *  PUPDR encoding 11 is reserved in hardware and unreachable through
 *  the enum — the enum and the register can never disagree.
 * ===================================================================== */

/* =====================================================================
 *  Per-pin field masks
 *
 *  Every mask is u-suffixed and casts the pin to u32 BEFORE shifting.
 *  Why: for pin 15, the two-bit masks reach bit 31 and the AFRH mask
 *  reaches bit 31 — an int (the enum's underlying type) left-shifted
 *  past its sign bit is undefined behavior in C. Unsigned shifts are
 *  always defined. This rule killed the old driver's signed-shift UB.
 *
 *  Usage pattern in MGPIO_program.c:
 *      reg->MODER &= ~MGPIO_MODER_MASK(pin);    clear the field
 *      reg->MODER |= ((u32)mode << (pin * 2u)); write the encoding
 * ===================================================================== */
#define MGPIO_MODER_MASK(pin)     (0x3u  << ((u32)(pin) * 2u))   /* 2 bits / pin  */
#define MGPIO_OSPEEDR_MASK(pin)   (0x3u  << ((u32)(pin) * 2u))   /* 2 bits / pin  */
#define MGPIO_PUPDR_MASK(pin)     (0x3u  << ((u32)(pin) * 2u))   /* 2 bits / pin  */
#define MGPIO_OTYPER_MASK(pin)    (0x1u  <<  (u32)(pin))         /* 1 bit  / pin  */
#define MGPIO_AFRL_MASK(pin)      (0xFu  << ((u32)(pin) * 4u))          /* pins 0..7  */
#define MGPIO_AFRH_MASK(pin)      (0xFu  << (((u32)(pin) - 8u) * 4u))   /* pins 8..15 */

/* =====================================================================
 *  BSRR — atomic bit set / reset (RM0368 §8.4.7)
 *
 *  Writing 1 to BS(x) (bits 15:0)  sets   pin x — a single bus write.
 *  Writing 1 to BR(x) (bits 31:16) resets pin x — a single bus write.
 *  Writing 0 anywhere has NO effect on that pin.
 *  If BS(x) and BR(x) are both written 1 for the same pin, BR wins.
 *
 *  This is why MGPIO has no ODR read-modify-write API: SET_BIT/CLR_BIT
 *  on ODR is a lost-update race the moment any ISR writes the same
 *  port — which Project 4 WILL have (EXTI button handler touching LED
 *  pins while the game loop writes TFT pins on the same port). BSRR
 *  does the same job in one atomic write.
 * ===================================================================== */
#define MGPIO_BSRR_BS(pin)        (1u <<  (u32)(pin))
#define MGPIO_BSRR_BR(pin)        (1u << ((u32)(pin) + 16u))

/* LCKR (0x1C): LCK[15:0] lock a pin's configuration; LCKK (bit 16)
   arms the lock through a specific read/write sequence (RM0368 §8.4.9)
   — once armed, it only clears on a port reset. Unused by this driver;
   defined here for documentation completeness. */
#define MGPIO_LCKR_LCKK           16

#endif /* MGPIO_PRIVATE_H_ */
