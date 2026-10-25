/*
 * MRCC_program.c
 *
 *  Implementation of the MRCC driver.
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Chapter 7: RCC
 *
 *  Design assumptions (documented, deliberate):
 *   - MRCC_Init() runs ONCE, first thing at boot, on a freshly reset
 *     device: PLL off, SW = HSI, CFGR at reset defaults. Runtime clock
 *     re-configuration is NOT supported — changing to the PLL later
 *     at runtime would require returning to HSI and disabling the PLL
 *     first. Change MRCC_config.h and reflash instead.
 *   - All read-modify-write operations here are safe because init runs
 *     before any other driver or any interrupt is active.
 *   - Every ready/switch poll is bounded by MRCC_TIMEOUT_COUNT, so a
 *     dead crystal or failing PLL returns MRCC_TIMEOUT instead of
 *     hanging the boot.
 *   - Flash latency, bus frequency limits, and PLL operating windows
 *     are validated at COMPILE time by the derived model in
 *     MRCC_config.h — the runtime code below never re-checks them.
 *
 *  Created on: Jul 17, 2026
 *      Author: dell
 */
#include "BIT_MATH.h"
#include "STD_TYPES.h"
#include <MCAL/RCC/MRCC_config.h>
#include <MCAL/RCC/MRCC_interface.h>
#include <MCAL/RCC/MRCC_private.h>


/* =====================================================================
 *  Private helper prototypes (internal to this file)
 * ===================================================================== */

/* Poll a read-only "ready" bit with a bounded loop counter.
   Returns MRCC_TIMEOUT instead of hanging forever on a dead source. */
static MRCC_ReturnType_t MRCC_privateWaitReadyFlag     (volatile u32 *Copy_pu32Reg, u8 Copy_u8Bit);

/* After writing SW, poll SWS until hardware confirms the switch.
   SW only REQUESTS the switch; SWS reports the CURRENT source. */
static MRCC_ReturnType_t MRCC_privateWaitSysClockSwitch(u8 Copy_u8ExpectedSWS);

/* Program flash wait states + prefetch. MUST run while SYSCLK is still
   low (before the switch) — RM0368 requires correct latency in place
   before SYSCLK exceeds 30 MHz. Wait states are derived in config. */
static void MRCC_privateSetFlashLatency(void);

/* Program AHB/APB1/APB2 prescalers from the config dividers.
   Also runs while SYSCLK is still low, so bus limits are never even
   transiently exceeded after the switch. */
static void MRCC_privateSetBusPrescalers(void);

/* Plain divider (1,2,4,8,16,64,128,256,512) -> HPRE field encoding. */
static u32 MRCC_privateEncodeAHBDivider(u32 Copy_u32Divider);

/* Plain divider (1,2,4,8,16) -> PPRE1/PPRE2 field encoding. */
static u32 MRCC_privateEncodeAPBDivider(u32 Copy_u32Divider);

/* Map a peripheral identifier to its clock-enable register + bit.
   Returns a null pointer for an unknown peripheral. */
static volatile u32 * MRCC_privateGetEnableRegister(MRCC_Peripheral_t Copy_Peripheral, u8 *Copy_pu8Bit);

/* =====================================================================
 *  MRCC_Init — configure the system clock per MRCC_config.h
 * ===================================================================== */
MRCC_ReturnType_t MRCC_Init(void)
{
    MRCC_ReturnType_t Local_ReturnStatus = MRCC_OK;

    /* Clear every RCC ready flag so no stale flag can fire later if RCC
       interrupts are ever enabled (write '1' to the clear bits; bit 22
       is reserved and excluded from the composite value). */
    RCC->CIR = RCC_CIR_CLEAR_ALL;

#if SYSTEM_CLOCK == MRCC_CLK_HSI

    /* -----------------------------------------------------------------
     *  HSI: internal 16 MHz RC oscillator — the reset configuration.
     *
     *  Derived model guarantees (MRCC_config.h):
     *    FLASH_LATENCY_WAIT_STATES = 0   (16 MHz <= 30 MHz)
     *    bus frequencies legal          (compile-time checked)
     *  HSION is already set after reset — writing it again is harmless
     *  and HSIRDY asserts immediately.
     * ----------------------------------------------------------------- */
    SET_BIT(RCC->CR, RCC_CR_HSION);
    Local_ReturnStatus = MRCC_privateWaitReadyFlag(&RCC->CR, RCC_CR_HSIRDY);

    if (Local_ReturnStatus == MRCC_OK)
    {
        /* Latency + prescalers while the (slow) old clock still runs */
        MRCC_privateSetFlashLatency();
        MRCC_privateSetBusPrescalers();

        /* Select HSI as system clock source: SW = 00 */
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_MASK) | (RCC_SW_HSI << RCC_CFGR_SW);

        /* Confirm hardware completed the switch: SWS must read 00 */
        Local_ReturnStatus = MRCC_privateWaitSysClockSwitch(RCC_SW_HSI);
    }

#elif SYSTEM_CLOCK == MRCC_CLK_HSE

    /* -----------------------------------------------------------------
     *  HSE: external crystal / oscillator used directly as SYSCLK.
     *  Flash latency is DERIVED from the HSE frequency — a 25 MHz
     *  crystal yields 0 WS automatically; anything above 30 MHz gets
     *  more wait states with no manual step.
     * ----------------------------------------------------------------- */

    /* HSEBYP may only be written while HSE is disabled (reset state) */
#if HSE_SOURCE_TYPE == HSE_CRYSTAL
    CLR_BIT(RCC->CR, RCC_CR_HSEBYP);     /* crystal on OSC_IN/OSC_OUT     */
#elif HSE_SOURCE_TYPE == HSE_BYPASS
    SET_BIT(RCC->CR, RCC_CR_HSEBYP);     /* external clock on OSC_IN      */
#else
#error "MRCC: HSE_SOURCE_TYPE must be HSE_CRYSTAL or HSE_BYPASS"
#endif

    SET_BIT(RCC->CR, RCC_CR_HSEON);
    Local_ReturnStatus = MRCC_privateWaitReadyFlag(&RCC->CR, RCC_CR_HSERDY);

    if (Local_ReturnStatus == MRCC_OK)
    {
        MRCC_privateSetFlashLatency();
        MRCC_privateSetBusPrescalers();

        /* Select HSE as system clock source: SW = 01 */
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_MASK) | (RCC_SW_HSE << RCC_CFGR_SW);
        Local_ReturnStatus = MRCC_privateWaitSysClockSwitch(RCC_SW_HSE);
    }

#elif SYSTEM_CLOCK == MRCC_CLK_PLL_HSI

    /* -----------------------------------------------------------------
     *  PLL fed from HSI: with PLLSRC = 0 the PLL input is HSI/2
     *  = 8 MHz (RM0368 §7.3.2), so PLL_M must be <= 8 to land inside
     *  the 1–2 MHz VCO input window — the config model ENFORCES this
     *  at compile time. Note HSI-PLL cannot produce an exact 48 MHz
     *  PLL48CLK (see the section 5 note in MRCC_config.h).
     * ----------------------------------------------------------------- */
    SET_BIT(RCC->CR, RCC_CR_HSION);
    Local_ReturnStatus = MRCC_privateWaitReadyFlag(&RCC->CR, RCC_CR_HSIRDY);

    if (Local_ReturnStatus == MRCC_OK)
    {
        /* Latency + prescalers while still on the (slow) old clock */
        MRCC_privateSetFlashLatency();
        MRCC_privateSetBusPrescalers();

        /* PLLCFGR may only be written while the PLL is OFF. On a
           freshly reset device it is off — see file-header assumption. */
        RCC->PLLCFGR = ((u32)PLL_M << RCC_PLLCFGR_PLLM)                     |
                       ((u32)PLL_N << RCC_PLLCFGR_PLLN)                     |
                       (RCC_PLLCFGR_PLLP_FIELD(PLL_P) << RCC_PLLCFGR_PLLP)  |
                       (0u                     << RCC_PLLCFGR_PLLSRC)       |  /* 0: HSI/2 */
                       ((u32)PLL_Q << RCC_PLLCFGR_PLLQ);

        SET_BIT(RCC->CR, RCC_CR_PLLON);
        Local_ReturnStatus = MRCC_privateWaitReadyFlag(&RCC->CR, RCC_CR_PLLRDY);

        if (Local_ReturnStatus == MRCC_OK)
        {
            /* Select PLL as system clock source: SW = 10 */
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_MASK) | (RCC_SW_PLL << RCC_CFGR_SW);
            Local_ReturnStatus = MRCC_privateWaitSysClockSwitch(RCC_SW_PLL);
        }
    }

#elif SYSTEM_CLOCK == MRCC_CLK_PLL_HSE

    /* -----------------------------------------------------------------
     *  PLL fed from HSE — the Project 4 target configuration:
     *  25 MHz crystal -> 84 MHz SYSCLK, 48 MHz PLL48CLK.
     *
     *  Sequence (RM0368 §7.2.1):
     *    1. HSE on + HSERDY
     *    2. flash latency raised while still on the old (slow) clock
     *    3. bus prescalers set while still on the old clock
     *    4. PLLCFGR written (PLL must be off)
     *    5. PLL on + PLLRDY
     *    6. SW = PLL, then SWS confirmed
     *
     *  Before enabling this branch, verify:
     *    - the board crystal really is 25 MHz (HSE_CRYSTAL_FREQ_KHZ)
     *    - PWR voltage scaling (VOS) per RM0368 PWR ch. + datasheet
     *
     *  Optional hardening once running at 84 MHz: enable the Clock
     *  Security System (RCC_CR_CSSON) so an HSE failure triggers a
     *  reset back to HSI instead of a stalled PLL.
     * ----------------------------------------------------------------- */

    /* HSEBYP may only be written while HSE is disabled (reset state) */
#if HSE_SOURCE_TYPE == HSE_CRYSTAL
    CLR_BIT(RCC->CR, RCC_CR_HSEBYP);
#elif HSE_SOURCE_TYPE == HSE_BYPASS
    SET_BIT(RCC->CR, RCC_CR_HSEBYP);
#else
#error "MRCC: HSE_SOURCE_TYPE must be HSE_CRYSTAL or HSE_BYPASS"
#endif

    /* Step 1 */
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    Local_ReturnStatus = MRCC_privateWaitReadyFlag(&RCC->CR, RCC_CR_HSERDY);

    if (Local_ReturnStatus == MRCC_OK)
    {
        /* Steps 2–3: while SYSCLK is still the (slow) old clock */
        MRCC_privateSetFlashLatency();
        MRCC_privateSetBusPrescalers();

        /* Step 4: full PLL configuration, PLL source = HSE.
           PLL_P arrives as the real divider (2/4/6/8) and is encoded
           into the 2-bit field by RCC_PLLCFGR_PLLP_FIELD. */
        RCC->PLLCFGR = ((u32)PLL_M << RCC_PLLCFGR_PLLM)                     |
                       ((u32)PLL_N << RCC_PLLCFGR_PLLN)                     |
                       (RCC_PLLCFGR_PLLP_FIELD(PLL_P) << RCC_PLLCFGR_PLLP)  |
                       (1u                     << RCC_PLLCFGR_PLLSRC)       |  /* 1: HSE */
                       ((u32)PLL_Q << RCC_PLLCFGR_PLLQ);

        /* Step 5 */
        SET_BIT(RCC->CR, RCC_CR_PLLON);
        Local_ReturnStatus = MRCC_privateWaitReadyFlag(&RCC->CR, RCC_CR_PLLRDY);

        if (Local_ReturnStatus == MRCC_OK)
        {
            /* Step 6 */
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_MASK) | (RCC_SW_PLL << RCC_CFGR_SW);
            Local_ReturnStatus = MRCC_privateWaitSysClockSwitch(RCC_SW_PLL);
        }
    }

#else
#error "MRCC: SYSTEM_CLOCK not recognised - check MRCC_config.h"
#endif

    return Local_ReturnStatus;
}

/* =====================================================================
 *  MRCC_EnablePeripheralClock
 * ===================================================================== */
MRCC_ReturnType_t MRCC_EnablePeripheralClock(MRCC_Peripheral_t Copy_Peripheral)
{
    u8 Local_u8Bit = 0;
    volatile u32 *Local_pu32EnableReg = MRCC_privateGetEnableRegister(Copy_Peripheral, &Local_u8Bit);

    if (Local_pu32EnableReg == (volatile u32 *)0)
    {
        return MRCC_INVALID_PERIPHERAL;
    }

    SET_BIT(*Local_pu32EnableReg, Local_u8Bit);

    /* Read-back: the enable write takes a few cycles to propagate before
       the peripheral's own registers may be accessed. Reading the enable
       register back guarantees those cycles have elapsed. */
    (void)(*Local_pu32EnableReg);

    return MRCC_OK;
}

/* =====================================================================
 *  MRCC_DisablePeripheralClock
 *  (Disable only after the peripheral is no longer in use — its
 *   registers become inaccessible the moment the clock is gated off.)
 * ===================================================================== */
MRCC_ReturnType_t MRCC_DisablePeripheralClock(MRCC_Peripheral_t Copy_Peripheral)
{
    u8 Local_u8Bit = 0;
    volatile u32 *Local_pu32EnableReg = MRCC_privateGetEnableRegister(Copy_Peripheral, &Local_u8Bit);

    if (Local_pu32EnableReg == (volatile u32 *)0)
    {
        return MRCC_INVALID_PERIPHERAL;
    }

    CLR_BIT(*Local_pu32EnableReg, Local_u8Bit);
    (void)(*Local_pu32EnableReg);     /* Read-back, same rationale */

    return MRCC_OK;
}

/* =====================================================================
 *  Private helpers
 * ===================================================================== */

/* ---- Bounded wait for a ready flag ---------------------------------- */
static MRCC_ReturnType_t MRCC_privateWaitReadyFlag(volatile u32 *Copy_pu32Reg, u8 Copy_u8Bit)
{
    u32 Local_u32Timeout = MRCC_TIMEOUT_COUNT;

    /* Register is volatile → re-read every iteration */
    while (GET_BIT(*Copy_pu32Reg, Copy_u8Bit) == 0)
    {
        Local_u32Timeout--;
        if (Local_u32Timeout == 0)
        {
            return MRCC_TIMEOUT;   /* Oscillator/PLL never became ready */
        }
    }
    return MRCC_OK;
}

/* ---- Bounded wait for SW -> SWS confirmation ------------------------- */
static MRCC_ReturnType_t MRCC_privateWaitSysClockSwitch(u8 Copy_u8ExpectedSWS)
{
    u32 Local_u32Timeout = MRCC_TIMEOUT_COUNT;

    /* SWS[3:2] always reports the CURRENT system clock source. Writing
       SW only requests a switch; hardware completes it once the target
       source is stable. Returning only after SWS matches guarantees the
       core is genuinely running from the new clock (RM0368 §7.2.1).   */
    while (((RCC->CFGR & RCC_CFGR_SWS_MASK) >> RCC_CFGR_SWS) != Copy_u8ExpectedSWS)
    {
        Local_u32Timeout--;
        if (Local_u32Timeout == 0)
        {
            return MRCC_TIMEOUT;   /* Switch never completed */
        }
    }
    return MRCC_OK;
}

/* ---- Flash wait states + prefetch ------------------------------------ */
static void MRCC_privateSetFlashLatency(void)
{
    u32 Local_u32ACR = FLASH_ACR;

    /* FLASH_LATENCY_WAIT_STATES is DERIVED by MRCC_config.h from the
       computed SYSCLK — there is no user knob to get wrong. This runs
       while SYSCLK is still low, BEFORE the clock switch — never after.
       Other ACR bits are preserved untouched. */
    Local_u32ACR &= ~FLASH_ACR_LATENCY_MASK;
    Local_u32ACR |= (FLASH_LATENCY_WAIT_STATES & FLASH_ACR_LATENCY_MASK);

    /* Prefetch is worthwhile whenever any wait state is configured */
    if (FLASH_LATENCY_WAIT_STATES > 0)
    {
        Local_u32ACR |= (1u << FLASH_ACR_PRFTEN);
    }

    FLASH_ACR = Local_u32ACR;
}

/* ---- Plain divider -> HPRE field encoding (RM0368 §7.3.7) ------------
 * 0000=/1, 1000=/2, 1001=/4, 1010=/8, 1011=/16, 1100=/64, 1101=/128,
 * 1110=/256, 1111=/512. Encodings 001–111 read as /1 but are reserved,
 * so they are never emitted. The divider value itself is validated at
 * COMPILE time in MRCC_config.h (section 7); the default case below is
 * unreachable and kept purely defensive. */
static u32 MRCC_privateEncodeAHBDivider(u32 Copy_u32Divider)
{
    switch (Copy_u32Divider)
    {
        case 1:   return 0x0u;
        case 2:   return 0x8u;
        case 4:   return 0x9u;
        case 8:   return 0xAu;
        case 16:  return 0xBu;
        case 64:  return 0xCu;
        case 128: return 0xDu;
        case 256: return 0xEu;
        case 512: return 0xFu;
        default:  return 0x0u;    /* unreachable — compile-time validated */
    }
}

/* ---- Plain divider -> PPRE field encoding (RM0368 §7.3.7) ------------
 * 0xx=/1 (000 emitted), 100=/2, 101=/4, 110=/8, 111=/16 */
static u32 MRCC_privateEncodeAPBDivider(u32 Copy_u32Divider)
{
    switch (Copy_u32Divider)
    {
        case 1:  return 0x0u;
        case 2:  return 0x4u;
        case 4:  return 0x5u;
        case 8:  return 0x6u;
        case 16: return 0x7u;
        default: return 0x0u;    /* unreachable — compile-time validated */
    }
}

/* ---- Program AHB/APB1/APB2 prescalers from the config dividers ------- */
static void MRCC_privateSetBusPrescalers(void)
{
    u32 Local_u32CFGR = RCC->CFGR;

    /* AHB (HPRE) — HCLK limit enforced at compile time against the
       computed HCLK. Config states a PLAIN divider; the driver owns
       the register field encoding. */
    Local_u32CFGR &= ~RCC_CFGR_HPRE_MASK;
    Local_u32CFGR |= (MRCC_privateEncodeAHBDivider(RCC_AHB_PRESCALER_DIV) << RCC_CFGR_HPRE);

    /* APB1 (PPRE1) — 42 MHz max, compile-time checked against the
       computed PCLK1. Timer note: with PPRE1 > 1, APB1 timers run at
       2 x PCLK1 — MRCC_APB1_TIMER_CLK_KHZ already models this. */
    Local_u32CFGR &= ~RCC_CFGR_PPRE1_MASK;
    Local_u32CFGR |= (MRCC_privateEncodeAPBDivider(RCC_APB1_PRESCALER_DIV) << RCC_CFGR_PPRE1);

    /* APB2 (PPRE2) — 84 MHz max, compile-time checked */
    Local_u32CFGR &= ~RCC_CFGR_PPRE2_MASK;
    Local_u32CFGR |= (MRCC_privateEncodeAPBDivider(RCC_APB2_PRESCALER_DIV) << RCC_CFGR_PPRE2);

    RCC->CFGR = Local_u32CFGR;
}

/* ---- Peripheral id -> (enable register, bit position) ---------------- */
static volatile u32 * MRCC_privateGetEnableRegister(MRCC_Peripheral_t Copy_Peripheral, u8 *Copy_pu8Bit)
{
    volatile u32 *Local_pu32Reg = (volatile u32 *)0;

    switch (Copy_Peripheral)
    {
        /* ============ AHB1 peripherals (AHB1ENR) ============ */
        case MRCC_GPIOA:    Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 0;  break;
        case MRCC_GPIOB:    Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 1;  break;
        case MRCC_GPIOC:    Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 2;  break;
        case MRCC_GPIOD:    Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 3;  break;
        case MRCC_GPIOE:    Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 4;  break;
        case MRCC_GPIOH:    Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 7;  break;
        case MRCC_CRC:      Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 12; break;
        case MRCC_BKPSRAM:  Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 18; break;
        case MRCC_DMA1:     Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 21; break;
        case MRCC_DMA2:     Local_pu32Reg = &RCC->AHB1ENR; *Copy_pu8Bit = 22; break;

        /* ============ AHB2 peripherals (AHB2ENR) ============ */
        case MRCC_OTG_FS:   Local_pu32Reg = &RCC->AHB2ENR; *Copy_pu8Bit = 7;  break;

        /* ============ APB1 peripherals (APB1ENR) ============ */
        case MRCC_TIM2:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 0;  break;
        case MRCC_TIM3:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 1;  break;
        case MRCC_TIM4:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 2;  break;
        case MRCC_TIM5:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 3;  break;
        case MRCC_WWDG:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 11; break;
        case MRCC_SPI2:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 14; break;
        case MRCC_SPI3:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 15; break;
        case MRCC_USART2:   Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 17; break;
        case MRCC_I2C1:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 21; break;
        case MRCC_I2C2:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 22; break;
        case MRCC_I2C3:     Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 23; break;
        case MRCC_PWR:      Local_pu32Reg = &RCC->APB1ENR; *Copy_pu8Bit = 28; break;

        /* ============ APB2 peripherals (APB2ENR) ============ */
        case MRCC_TIM1:     Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 0;  break;
        case MRCC_USART1:   Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 4;  break;
        case MRCC_USART6:   Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 5;  break;
        case MRCC_ADC1:     Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 8;  break;
        case MRCC_SDIO:     Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 11; break;
        case MRCC_SPI1:     Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 12; break;
        case MRCC_SPI4:     Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 13; break;
        case MRCC_SYSCFG:   Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 14; break;
        case MRCC_TIM9:     Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 16; break;
        case MRCC_TIM10:    Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 17; break;
        case MRCC_TIM11:    Local_pu32Reg = &RCC->APB2ENR; *Copy_pu8Bit = 18; break;

        default:
            /* Unknown peripheral → null pointer → caller reports error */
            break;
    }

    return Local_pu32Reg;
}
