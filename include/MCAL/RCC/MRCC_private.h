/*
 * MRCC_private.h
 *
 *  PRIVATE header of the MRCC driver — never include this from HAL or
 *  Application code. Contains ONLY hardware facts: base addresses, the
 *  register map struct, bit positions, field masks/encoders, and
 *  internal driver constants.
 *
 *  Everything user-tunable (clock source, HSE type, prescalers, PLL
 *  factors) lives in MRCC_config.h. The original version violated this
 *  by keeping HSI/HSE/PLL_* option macros here — that inversion is
 *  what allowed the SYSTEMCLOCK/SYSTEM_CLOCK mismatch to hide.
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Chapter 7: RCC
 *
 *  Created on: Jul 17, 2026
 *      Author: dell
 */

#ifndef MRCC_PRIVATE_H_
#define MRCC_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Base addresses
 * ===================================================================== */

/* RCC peripheral base address (RM0368 §7.3, Table 26) */
#define RCC_BASE_ADDRESS              (0x40023800UL)

/*
 * FLASH Access Control Register (FLASH_ACR) address.
 *
 * The RCC driver touches FLASH_ACR for exactly ONE reason: RM0368
 * requires flash LATENCY (wait states) to be in place BEFORE SYSCLK
 * rises above 30 MHz — i.e. during clock init, before the MFLS (Flash)
 * driver of Phase 1 even exists. This is a documented, deliberate
 * layering exception. When MFLS is built, move this address to a
 * shared location so both drivers reference one definition.
 */
#define FLASH_ACR_ADDRESS             (0x40023C00UL)

#define FLASH_ACR                     (*(volatile u32 *) FLASH_ACR_ADDRESS)

/* =====================================================================
 *  RCC register map (RM0368 §7.3 — offsets verified)
 *
 *  Explicitly-declared reserved gaps keep the C struct layout identical
 *  to the hardware register map. Do NOT reorder, rename, or remove the
 *  reserved fields — every field below sits at its exact byte offset.
 * ===================================================================== */
typedef struct
{
    volatile u32 CR;            /* 0x00 : Clock control register            */
    volatile u32 PLLCFGR;       /* 0x04 : PLL configuration register        */
    volatile u32 CFGR;          /* 0x08 : Clock configuration register      */
    volatile u32 CIR;           /* 0x0C : Clock interrupt register          */
    volatile u32 AHB1RSTR;      /* 0x10 : AHB1 peripheral reset register    */
    volatile u32 AHB2RSTR;      /* 0x14 : AHB2 peripheral reset register    */
    volatile u32 RESERVED1;     /* 0x18 : Reserved                          */
    volatile u32 RESERVED2;     /* 0x1C : Reserved                          */
    volatile u32 APB1RSTR;      /* 0x20 : APB1 peripheral reset register    */
    volatile u32 APB2RSTR;      /* 0x24 : APB2 peripheral reset register    */
    volatile u32 RESERVED3;     /* 0x28 : Reserved                          */
    volatile u32 RESERVED4;     /* 0x2C : Reserved                          */
    volatile u32 AHB1ENR;       /* 0x30 : AHB1 peripheral clock enable      */
    volatile u32 AHB2ENR;       /* 0x34 : AHB2 peripheral clock enable      */
    volatile u32 RESERVED5;     /* 0x38 : Reserved                          */
    volatile u32 RESERVED6;     /* 0x3C : Reserved                          */
    volatile u32 APB1ENR;       /* 0x40 : APB1 peripheral clock enable      */
    volatile u32 APB2ENR;       /* 0x44 : APB2 peripheral clock enable      */
    volatile u32 RESERVED7;     /* 0x48 : Reserved                          */
    volatile u32 RESERVED8;     /* 0x4C : Reserved                          */
    volatile u32 AHB1LPENR;     /* 0x50 : AHB1 clock enable in low-power    */
    volatile u32 AHB2LPENR;     /* 0x54 : AHB2 clock enable in low-power    */
    volatile u32 RESERVED9;     /* 0x58 : Reserved                          */
    volatile u32 RESERVED10;    /* 0x5C : Reserved                          */
    volatile u32 APB1LPENR;     /* 0x60 : APB1 clock enable in low-power    */
    volatile u32 APB2LPENR;     /* 0x64 : APB2 clock enable in low-power    */
    volatile u32 RESERVED11;    /* 0x68 : Reserved                          */
    volatile u32 RESERVED12;    /* 0x6C : Reserved                          */
    volatile u32 BDCR;          /* 0x70 : Backup domain control register    */
    volatile u32 CSR;           /* 0x74 : Control/status register           */
    volatile u32 RESERVED13;    /* 0x78 : Reserved                          */
    volatile u32 RESERVED14;    /* 0x7C : Reserved                          */
    volatile u32 SSCGR;         /* 0x80 : Spread spectrum clock config      */
    volatile u32 PLLI2SCFGR;    /* 0x84 : PLLI2S configuration register     */
    volatile u32 RESERVED15;    /* 0x88 : Reserved                          */
    volatile u32 DCKCFGR;       /* 0x8C : Dedicated clocks config register  */
} RCC_t;

/* Single global access point to the RCC register map */
#define RCC                           ((volatile RCC_t *) RCC_BASE_ADDRESS)

/* =====================================================================
 *  RCC_CR — clock control register (RM0368 §7.3.1)
 * ===================================================================== */
#define RCC_CR_HSION                  0      /* Bit 0  : HSI 16 MHz enable          */
#define RCC_CR_HSIRDY                 1      /* Bit 1  : HSI ready (read-only)      */
#define RCC_CR_HSEON                  16     /* Bit 16 : HSE enable                 */
#define RCC_CR_HSERDY                 17     /* Bit 17 : HSE ready (read-only)      */
#define RCC_CR_HSEBYP                 18     /* Bit 18 : 0 = crystal, 1 = ext. clk  */
#define RCC_CR_CSSON                  19     /* Bit 19 : Clock security system      */
#define RCC_CR_PLLON                  24     /* Bit 24 : Main PLL enable            */
#define RCC_CR_PLLRDY                 25     /* Bit 25 : Main PLL ready (read-only) */

/* =====================================================================
 *  RCC_PLLCFGR — main PLL configuration (RM0368 §7.3.2)
 * ===================================================================== */
#define RCC_PLLCFGR_PLLM              0      /* Bits  5:0  : /M input divider
                                                — SIX bits wide, range 2..63
                                                (an earlier comment wrongly
                                                said "Bits 4:0"; the shift
                                                position 0 was correct)      */
#define RCC_PLLCFGR_PLLN              6      /* Bits 14:6  : VCO multiplier N       */
#define RCC_PLLCFGR_PLLP              16     /* Bits 17:16: /P output divider       */
#define RCC_PLLCFGR_PLLSRC            22     /* Bit  22    : 0 = HSI/2, 1 = HSE     */
#define RCC_PLLCFGR_PLLQ              24     /* Bits 27:24: /Q (USB OTG FS, SDIO)   */

/* Encode a REAL PLL output divider (2, 4, 6 or 8) into the 2-bit PLLP
   register field (0, 1, 2, 3). MRCC_config.h states the DIVIDER; this
   macro is the ONLY place it is translated into the field encoding. */
#define RCC_PLLCFGR_PLLP_FIELD(div)   ((((u32)(div)) / 2u) - 1u)

/* =====================================================================
 *  RCC_CFGR — clock configuration (RM0368 §7.3.7)
 * ===================================================================== */
#define RCC_CFGR_SW                   0      /* Bits  1:0  : system clock switch (write)  */
#define RCC_CFGR_SWS                  2      /* Bits  3:2  : system clock status (read)  */
#define RCC_CFGR_HPRE                 4      /* Bits  7:4  : AHB  prescaler              */
#define RCC_CFGR_PPRE1                10     /* Bits 12:10 : APB1 prescaler              */
#define RCC_CFGR_PPRE2                13     /* Bits 15:13 : APB2 prescaler              */

/* Field masks — for read-modify-write on CFGR. Safe because MRCC_Init
 * runs once at boot, before any other driver or interrupt is active. */
#define RCC_CFGR_SW_MASK              (0x3u  << RCC_CFGR_SW)
#define RCC_CFGR_SWS_MASK             (0x3u  << RCC_CFGR_SWS)
#define RCC_CFGR_HPRE_MASK            (0xFu  << RCC_CFGR_HPRE)
#define RCC_CFGR_PPRE1_MASK           (0x7u  << RCC_CFGR_PPRE1)
#define RCC_CFGR_PPRE2_MASK           (0x7u  << RCC_CFGR_PPRE2)

/* SW / SWS encodings — both fields share these values (RM0368 §7.3.7):
 * 00 = HSI, 01 = HSE, 10 = PLL, 11 = illegal. Writing SW only REQUESTS
 * a switch; the switch is complete only once SWS mirrors it.          */
#define RCC_SW_HSI                    0x00u
#define RCC_SW_HSE                    0x01u
#define RCC_SW_PLL                    0x02u

/* =====================================================================
 *  RCC_CIR — clock interrupt register (RM0368 §7.3.4)
 *
 *  Ready flags are cleared by writing '1' to the corresponding clear
 *  bit. RCC_CIR_CLEAR_ALL clears every flag at once:
 *      bits 16..21 : LSIRDYC, LSERDYC, HSIRDYC, HSERDYC, PLLRDYC,
 *                    PLLI2SRDYC
 *      bit  23     : CSSC (clock security system clear)
 *  Bit 22 is reserved on the F401 and is therefore excluded.
 * ===================================================================== */
#define RCC_CIR_CLEAR_ALL             (0x00BF0000UL)

/* =====================================================================
 *  FLASH_ACR bits — only what this driver programs (RM0368, Flash ch.)
 *
 *  FLASH_LATENCY_WAIT_STATES itself is NOT defined here: it is DERIVED
 *  by MRCC_config.h from the computed SYSCLK (0 WS <= 30 MHz,
 *  1 WS <= 60 MHz, 2 WS otherwise). There is no legitimate user choice,
 *  so no knob exists — only bit definitions.
 * ===================================================================== */
#define FLASH_ACR_LATENCY             0      /* Bits 2:0 : flash wait states */
#define FLASH_ACR_LATENCY_MASK        0x7u
#define FLASH_ACR_PRFTEN              8      /* Bit 8 : prefetch enable      */

/* =====================================================================
 *  Ready-flag polling timeout
 *
 *  Init runs before SysTick exists, so a plain loop counter is the
 *  right tool. 500,000 iterations at 16 MHz takes well over 100 ms —
 *  far longer than any legitimate oscillator or PLL lock time, but
 *  finite, so a missing or broken crystal returns MRCC_TIMEOUT
 *  instead of hanging the boot forever.
 * ===================================================================== */
#define MRCC_TIMEOUT_COUNT            (500000UL)

#endif /* MRCC_PRIVATE_H_ */
