/*
 * MRCC_config.h
 *
 *  Configuration file for the MRCC driver. Split into two halves:
 *
 *    Sections 1-5 : USER CHOICES (oscillator, prescalers, PLL factors)
 *    Sections 6-7 : DERIVED MODEL + VALIDATION — do not edit
 *
 *  The derived half exists so that mutually-dependent values (flash
 *  latency vs. SYSCLK, bus limits vs. prescalers, PLL operating
 *  windows) can never silently disagree: the preprocessor computes the
 *  actual frequencies and every guard checks the COMPUTED number.
 *
 *  Reference: RM0368 (STM32F401xB/xC), Chapter 7: RCC + datasheet
 *             PLL characteristics / operating conditions
 *
 *  Created on: Jul 17, 2026
 *      Author: dell
 */

#ifndef MRCC_CONFIG_H_
#define MRCC_CONFIG_H_

/* =====================================================================
 *  1. SYSTEM CLOCK SOURCE  (user choice)
 *
 *      MRCC_CLK_HSI      16 MHz internal RC oscillator (reset default)
 *      MRCC_CLK_HSE      External crystal/oscillator used directly
 *      MRCC_CLK_PLL_HSI  PLL fed from HSI/2 (8 MHz input to the PLL)
 *      MRCC_CLK_PLL_HSE  PLL fed from HSE crystal (Project 4 target)
 * ===================================================================== */
#define MRCC_CLK_HSI      0
#define MRCC_CLK_HSE      1
#define MRCC_CLK_PLL_HSI  2
#define MRCC_CLK_PLL_HSE  3

/* Currently 16 MHz HSI. Flip to MRCC_CLK_PLL_HSE for 84 MHz — the
 * validation in section 7 will then force every dependent value
 * (APB1 prescaler, PLL factors) to be consistent before it compiles. */
#define SYSTEM_CLOCK      MRCC_CLK_HSI

/* =====================================================================
 *  2. HSE SOURCE TYPE  (user choice)
 *
 *      HSE_CRYSTAL  Crystal between OSC_IN/OSC_OUT (WeAct Black Pill:
 *                   verify the crystal marking — typically 25 MHz)
 *      HSE_BYPASS   External clock signal driven onto OSC_IN
 * ===================================================================== */
#define HSE_CRYSTAL       0
#define HSE_BYPASS        1

#define HSE_SOURCE_TYPE   HSE_CRYSTAL

/* =====================================================================
 *  3. OSCILLATOR FREQUENCIES  (user choice — must match the board)
 *
 *  Integer kHz so the whole preprocessor model stays integer math.
 *  HSI is factory-trimmed to a nominal 16 MHz and is NOT a choice.
 * ===================================================================== */
#define HSE_CRYSTAL_FREQ_KHZ    25000   /* crystal / ext. clock, in kHz */
#define HSI_FREQ_KHZ            16000   /* nominal, factory trimmed     */

/* =====================================================================
 *  4. BUS PRESCALERS  (user choice — PLAIN DIVIDER VALUES)
 *
 *      AHB options : 1, 2, 4, 8, 16, 64, 128, 256, 512
 *      APB options : 1, 2, 4, 8, 16
 *
 *  F401 limits (enforced against the COMPUTED frequencies in sec. 7):
 *      HCLK : max 84 MHz     PCLK1 : max 42 MHz (!)     PCLK2 : max 84 MHz
 *
 *  The HPRE/PPRE field encodings live in the DRIVER (program.c), not
 *  here — config states intent ("divide by 2"), the driver translates.
 *
 *  Timer note (RM0368): APB prescaler = 1  ->  timer clock = PCLK;
 *                        APB prescaler > 1  ->  timer clock = 2 x PCLK.
 *  The derived timer clocks are published below for MTIMER (Phase 1).
 * ===================================================================== */
#define RCC_AHB_PRESCALER_DIV    1      /* 16 MHz today; 1 at 84 MHz  */
#define RCC_APB1_PRESCALER_DIV   1      /* 16 MHz today; 2 at 84 MHz  */
#define RCC_APB2_PRESCALER_DIV   1      /* 16 MHz today; 1 at 84 MHz  */

/* =====================================================================
 *  5. PLL COEFFICIENTS  (user choice — validated against sec. 6 model)
 *
 *      VCO input  = PLL input / M      (must be 1-2 MHz)
 *      VCO output = VCO input x N      (must be 100-432 MHz)
 *      SYSCLK     = VCO output / P     (P = 2, 4, 6 or 8)
 *      PLL48CLK   = VCO output / Q     (48 MHz target for USB/SDIO)
 *
 *  Project 4 target (25 MHz HSE -> 84 MHz):
 *      25/25 = 1 MHz, x336 = 336 MHz, /4 = 84 MHz SYSCLK, /7 = 48 MHz.
 *
 *  PLL_P is the REAL divider (2/4/6/8), not the register field value;
 *  the driver encodes it into PLLP[17:16].
 *
 *  HSI-PLL note: PLL input is HSI/2 = 8 MHz, so M must be <= 8 for a
 *  1 MHz VCO input; 168/3.5 shows HSI-PLL cannot reach 48 MHz with an
 *  integer Q — set PLL_48MHZ_REQUIRED to 0 if running HSI-PLL without
 *  USB OTG FS / SDIO.
 * ===================================================================== */
#define PLL_M                25     /* 2..63  : input divider          */
#define PLL_N                336    /* VCO multiplier (see VCO window) */
#define PLL_P                4      /* 2/4/6/8: SYSCLK output divider  */
#define PLL_Q                7      /* 2..15  : PLL48CLK divider       */

/* Set to 0 only if USB OTG FS / SDIO will never be used — lets an
   HSI-PLL build pass the 48 MHz check with a clear conscience.        */
#define PLL_48MHZ_REQUIRED   1

/* =====================================================================
 *  6. DERIVED CLOCK MODEL — DO NOT EDIT
 *
 *  Computed from sections 1-5 by the preprocessor. Purposes:
 *    - flash latency is DERIVED from SYSCLK (no user choice exists)
 *    - bus limits are checked against actual computed frequencies
 *    - one definition of every clock number in the whole library
 *
 *  Integer division may truncate by a few kHz on awkward M values —
 *  negligible against the MHz-scale guard thresholds below (and the
 *  hardware divides the real oscillator, not this model).
 * ===================================================================== */

/* ---- PLL model. Computed from the PLL input implied by SYSTEM_CLOCK,
        so coefficients stay validated even while running HSI/HSE.  ---- */
#if (SYSTEM_CLOCK == MRCC_CLK_PLL_HSI)
    #define PLL_INPUT_KHZ      (HSI_FREQ_KHZ / 2)    /* PLLSRC=0: HSI/2 */
#else
    #define PLL_INPUT_KHZ      HSE_CRYSTAL_FREQ_KHZ  /* PLLSRC=1: HSE   */
#endif

#define PLL_VCO_IN_KHZ      (PLL_INPUT_KHZ / PLL_M)    /* 1-2 MHz window */
#define PLL_VCO_OUT_KHZ     (PLL_VCO_IN_KHZ * PLL_N)  /* 100-432 MHz    */
#define PLL_48CLK_KHZ       (PLL_VCO_OUT_KHZ / PLL_Q) /* 48 MHz target  */

/* ---- System and bus clocks (published to other drivers) ------------ */
#if   (SYSTEM_CLOCK == MRCC_CLK_HSI)
    #define MRCC_SYSCLK_KHZ   HSI_FREQ_KHZ
#elif (SYSTEM_CLOCK == MRCC_CLK_HSE)
    #define MRCC_SYSCLK_KHZ   HSE_CRYSTAL_FREQ_KHZ
#else
    #define MRCC_SYSCLK_KHZ   (PLL_VCO_OUT_KHZ / PLL_P)
#endif

#define MRCC_HCLK_KHZ       (MRCC_SYSCLK_KHZ / RCC_AHB_PRESCALER_DIV)
#define MRCC_PCLK1_KHZ      (MRCC_HCLK_KHZ / RCC_APB1_PRESCALER_DIV)
#define MRCC_PCLK2_KHZ      (MRCC_HCLK_KHZ / RCC_APB2_PRESCALER_DIV)

/* ---- Timer clocks (RM0368 x2 rule) — consumed by MTIMER (Phase 1) -- */
#if (RCC_APB1_PRESCALER_DIV == 1)
    #define MRCC_APB1_TIMER_CLK_KHZ   MRCC_PCLK1_KHZ
#else
    #define MRCC_APB1_TIMER_CLK_KHZ   (MRCC_PCLK1_KHZ * 2)
#endif
#if (RCC_APB2_PRESCALER_DIV == 1)
    #define MRCC_APB2_TIMER_CLK_KHZ   MRCC_PCLK2_KHZ
#else
    #define MRCC_APB2_TIMER_CLK_KHZ   (MRCC_PCLK2_KHZ * 2)
#endif

/* ---- Flash wait states: DERIVED from SYSCLK (2.7-3.6 V, RM0368):
        0 WS <= 30 MHz, 1 WS <= 60 MHz, 2 WS otherwise.
        SYSCLK fully determines this — it is not a user choice.   ------ */
#if   (MRCC_SYSCLK_KHZ <= 30000)
    #define FLASH_LATENCY_WAIT_STATES   0
#elif (MRCC_SYSCLK_KHZ <= 60000)
    #define FLASH_LATENCY_WAIT_STATES   1
#else
    #define FLASH_LATENCY_WAIT_STATES   2
#endif

/* =====================================================================
 *  7. CONFIGURATION VALIDATION — the build STOPS on any inconsistency
 *
 *  This is where the file's own philosophy is enforced for EVERY
 *  dependent value: prescaler vs. bus limit, latency vs. SYSCLK,
 *  PLL windows, 48 MHz clock — all against the computed model above.
 * ===================================================================== */

/* ---- Clock source / HSE type ---- */
#if (SYSTEM_CLOCK != MRCC_CLK_HSI) && (SYSTEM_CLOCK != MRCC_CLK_HSE) && \
    (SYSTEM_CLOCK != MRCC_CLK_PLL_HSI) && (SYSTEM_CLOCK != MRCC_CLK_PLL_HSE)
#error "MRCC: SYSTEM_CLOCK must be one of MRCC_CLK_HSI/HSE/PLL_HSI/PLL_HSE"
#endif

#if (HSE_SOURCE_TYPE != HSE_CRYSTAL) && (HSE_SOURCE_TYPE != HSE_BYPASS)
#error "MRCC: HSE_SOURCE_TYPE must be HSE_CRYSTAL or HSE_BYPASS"
#endif

/* ---- PLL coefficient field widths ---- */
#if (PLL_M < 2) || (PLL_M > 63)
#error "MRCC: PLL_M must be 2..63"
#endif

#if (PLL_P != 2) && (PLL_P != 4) && (PLL_P != 6) && (PLL_P != 8)
#error "MRCC: PLL_P must be one of 2, 4, 6, 8"
#endif

#if (PLL_Q < 2) || (PLL_Q > 15)
#error "MRCC: PLL_Q must be 2..15"
#endif

/* ---- PLL operating windows (datasheet) ---- */
#if (PLL_VCO_IN_KHZ < 1000) || (PLL_VCO_IN_KHZ > 2000)
#error "MRCC: PLL VCO input outside the recommended 1-2 MHz window - resize PLL_M for the selected PLL input (HSI-PLL: M <= 8)"
#endif

#if (PLL_VCO_OUT_KHZ < 100000) || (PLL_VCO_OUT_KHZ > 432000)
#error "MRCC: PLL VCO output must be 100-432 MHz - fix PLL_N"
#endif

/* ---- 48 MHz clock for USB OTG FS / SDIO ---- */
#if PLL_48MHZ_REQUIRED && (PLL_48CLK_KHZ != 48000)
#error "MRCC: PLL48CLK is not 48 MHz - USB OTG FS / SDIO will not work. Fix PLL_Q, or set PLL_48MHZ_REQUIRED to 0 if unused (HSI-PLL cannot reach 48 MHz)"
#endif

/* ---- SYSCLK device limit ---- */
#if (MRCC_SYSCLK_KHZ > 84000)
#error "MRCC: SYSCLK exceeds the 84 MHz device maximum - fix PLL_N / PLL_P"
#endif

/* ---- Prescaler divider values ---- */
#if (RCC_AHB_PRESCALER_DIV != 1) && (RCC_AHB_PRESCALER_DIV != 2) && \
    (RCC_AHB_PRESCALER_DIV != 4) && (RCC_AHB_PRESCALER_DIV != 8) && \
    (RCC_AHB_PRESCALER_DIV != 16) && (RCC_AHB_PRESCALER_DIV != 64) && \
    (RCC_AHB_PRESCALER_DIV != 128) && (RCC_AHB_PRESCALER_DIV != 256) && \
    (RCC_AHB_PRESCALER_DIV != 512)
#error "MRCC: RCC_AHB_PRESCALER_DIV must be 1, 2, 4, 8, 16, 64, 128, 256 or 512"
#endif

#if (RCC_APB1_PRESCALER_DIV != 1) && (RCC_APB1_PRESCALER_DIV != 2) && \
    (RCC_APB1_PRESCALER_DIV != 4) && (RCC_APB1_PRESCALER_DIV != 8) && \
    (RCC_APB1_PRESCALER_DIV != 16)
#error "MRCC: RCC_APB1_PRESCALER_DIV must be 1, 2, 4, 8 or 16"
#endif

#if (RCC_APB2_PRESCALER_DIV != 1) && (RCC_APB2_PRESCALER_DIV != 2) && \
    (RCC_APB2_PRESCALER_DIV != 4) && (RCC_APB2_PRESCALER_DIV != 8) && \
    (RCC_APB2_PRESCALER_DIV != 16)
#error "MRCC: RCC_APB2_PRESCALER_DIV must be 1, 2, 4, 8 or 16"
#endif

/* ---- Bus frequency limits vs. COMPUTED clocks (the reviewer's
        scenario: flip to PLL_HSE and forget APB1 -> stopped HERE) ---- */
#if (MRCC_PCLK1_KHZ > 42000)
#error "MRCC: APB1 exceeds its 42 MHz maximum - increase RCC_APB1_PRESCALER_DIV (2 is correct for 84 MHz SYSCLK)"
#endif

#if (MRCC_PCLK2_KHZ > 84000)
#error "MRCC: APB2 exceeds its 84 MHz maximum - increase RCC_APB2_PRESCALER_DIV"
#endif

#endif /* MRCC_CONFIG_H_ */
