/*
 * MRCC_interface.h
 *
 *  PUBLIC interface of the MRCC driver. HAL and Application code must
 *  include ONLY this file (never MRCC_private.h).
 *
 *  Exposes:
 *    1. The driver status type     — the uniform convention for all MCAL
 *    2. The published clock model  — compile-time kHz constants computed
 *                                    by MRCC_config.h's derived section
 *    3. The flat peripheral enum   — bus is resolved INSIDE the driver
 *    4. The API
 *
 *  Reference: RM0368 (STM32F401xB/xC Reference Manual), Chapter 7: RCC
 *
 *  Created on: Jul 17, 2026
 *      Author: dell
 */

#ifndef MRCC_INTERFACE_H_
#define MRCC_INTERFACE_H_

#include "STD_TYPES.h"
#include <MCAL/RCC/MRCC_config.h>   /* brings in the derived clock model
                                       published in the section below    */

/* =====================================================================
 *  1. Driver status codes
 *
 *  Convention: every MCAL driver returns a status enum of this shape,
 *  prefixed with the driver name. The remaining drivers (MGPIO, MEXTI,
 *  MSPI, MUSART, ...) should adopt the identical pattern when revised,
 *  so the whole library ends up with ONE uniform error convention.
 * ===================================================================== */
typedef enum
{
    MRCC_OK = 0,                /* Operation completed successfully          */
    MRCC_TIMEOUT,               /* Ready/switch flag did not assert in time  */
    MRCC_INVALID_PERIPHERAL     /* Peripheral id unknown on this device     */
} MRCC_ReturnType_t;

/* =====================================================================
 *  2. Published clock model — compile-time constants, in kHz
 *
 *  Single source of truth for every driver that needs a clock number:
 *    - MTIMER (Phase 1) : tick / PWM math from MRCC_APB1_TIMER_CLK_KHZ
 *                         and MRCC_APB2_TIMER_CLK_KHZ (the RM0368 x2
 *                         rule for APB prescalers is already applied)
 *    - MUSART           : baud-rate divisors from MRCC_PCLK1/PCLK2_KHZ
 *
 *  All values are COMPUTED by the preprocessor from the user choices in
 *  MRCC_config.h, so they follow any clock-source change automatically.
 *  A driver that hardcodes its own clock value instead of using these
 *  is a bug: it will silently break the day SYSTEM_CLOCK changes.
 * ===================================================================== */
/*  MRCC_SYSCLK_KHZ           system clock (core)                      */
/*  MRCC_HCLK_KHZ             AHB bus clock (after HPRE)               */
/*  MRCC_PCLK1_KHZ            APB1 bus clock (after PPRE1)             */
/*  MRCC_PCLK2_KHZ            APB2 bus clock (after PPRE2)             */
/*  MRCC_APB1_TIMER_CLK_KHZ   TIM2..TIM5 timer clock                   */
/*  MRCC_APB2_TIMER_CLK_KHZ   TIM1, TIM9..TIM11 timer clock            */

/* =====================================================================
 *  3. Peripheral identifiers — ONE flat enum for the STM32F401CCU6
 *
 *  Each value names exactly one clock-enable bit on exactly one bus
 *  (RM0368 §7.3.10–§7.3.15). The bus is resolved INTERNALLY by the
 *  driver, so a caller can no longer mismatch peripheral and bus —
 *  the old two-argument API allowed e.g. (AHB1, APB2_SPI1_Enable),
 *  which silently enabled CRC (bit 12 of AHB1ENR) instead of SPI1.
 *
 *  Per entry: name, bus, ENR bit position.
 *  Note: RTC has no clock-enable bit — it lives in the backup domain
 *  (BDCR) and is configured through PWR + BDCR by the Phase-1 RTC
 *  driver, which will need MRCC_PWR (see APB1 section below).
 * ===================================================================== */
typedef enum
{
    /* ---------- AHB1 bus (AHB1ENR) ---------- */
    MRCC_GPIOA = 0,     /* Bit 0  : GPIO port A                                  */
    MRCC_GPIOB,         /* Bit 1  : GPIO port B                                  */
    MRCC_GPIOC,         /* Bit 2  : GPIO port C                                  */
    MRCC_GPIOD,         /* Bit 3  : GPIO port D                                  */
    MRCC_GPIOE,         /* Bit 4  : GPIO port E                                  */
    MRCC_GPIOH,         /* Bit 7  : GPIO port H                                  */
    MRCC_CRC,           /* Bit 12 : CRC calculation unit (Phase 1 — FOTA)        */
    MRCC_BKPSRAM,       /* Bit 18 : 4 KB backup SRAM (needs VBAT supply —
                               interesting with a coin cell for RTC/parental
                               settings persistence)                            */
    MRCC_DMA1,          /* Bit 21 : DMA1 (Phase 1 — audio / TFT transfers)      */
    MRCC_DMA2,          /* Bit 22 : DMA2 (Phase 1 — audio / TFT transfers)      */

    /* ---------- AHB2 bus (AHB2ENR) ---------- */
    MRCC_OTG_FS,        /* Bit 7  : USB OTG full-speed (Phase 2 — CDC / MSC)    */

    /* ---------- APB1 bus (APB1ENR) ---------- */
    MRCC_TIM2,          /* Bit 0  : Timer 2 (32-bit) — Phase 1                  */
    MRCC_TIM3,          /* Bit 1  : Timer 3                                     */
    MRCC_TIM4,          /* Bit 2  : Timer 4                                     */
    MRCC_TIM5,          /* Bit 3  : Timer 5 (32-bit)                            */
    MRCC_WWDG,          /* Bit 11 : Window watchdog (Phase 2, low priority)     */
    MRCC_SPI2,          /* Bit 14 : SPI 2                                       */
    MRCC_SPI3,          /* Bit 15 : SPI 3                                       */
    MRCC_USART2,        /* Bit 17 : USART 2                                     */
    MRCC_I2C1,          /* Bit 21 : I2C 1                                       */
    MRCC_I2C2,          /* Bit 22 : I2C 2                                       */
    MRCC_I2C3,          /* Bit 23 : I2C 3                                       */
    MRCC_PWR,           /* Bit 28 : Power interface — REQUIRED for backup-domain
                               access (RTC, BKPSRAM); the Phase-1 RTC driver
                               enables this before touching BDCR               */

    /* ---------- APB2 bus (APB2ENR) ---------- */
    MRCC_TIM1,          /* Bit 0  : Timer 1 (advanced) — Phase 1                */
    MRCC_USART1,        /* Bit 4  : USART 1 (ESP8266 link)                      */
    MRCC_USART6,        /* Bit 5  : USART 6                                     */
    MRCC_ADC1,          /* Bit 8  : ADC 1                                       */
    MRCC_SDIO,          /* Bit 11 : SDIO (Phase 2)                              */
    MRCC_SPI1,          /* Bit 12 : SPI 1 (TFT / 74HC595 path)                  */
    MRCC_SPI4,          /* Bit 13 : SPI 4                                       */
    MRCC_SYSCFG,        /* Bit 14 : System configuration (EXTI routing)         */
    MRCC_TIM9,          /* Bit 16 : Timer 9                                     */
    MRCC_TIM10,         /* Bit 17 : Timer 10                                    */
    MRCC_TIM11          /* Bit 18 : Timer 11                                    */
} MRCC_Peripheral_t;

/* =====================================================================
 *  4. API
 *
 *  Naming: the "void" infix was dropped now that every function
 *  returns a status. Apply the same convention to the other drivers
 *  as they are revised.
 *
 *  MRCC_Init() MUST be the first MCAL call at boot: every other driver
 *  assumes the clock tree is configured and its peripheral clock can
 *  be enabled.
 *
 *  Usage example:
 *      if (MRCC_Init() == MRCC_OK)
 *      {
 *          (void)MRCC_EnablePeripheralClock(MRCC_GPIOA);
 *          (void)MRCC_EnablePeripheralClock(MRCC_SYSCFG);
 *      }
 * ===================================================================== */

MRCC_ReturnType_t MRCC_Init(void);

MRCC_ReturnType_t MRCC_EnablePeripheralClock (MRCC_Peripheral_t Copy_Peripheral);
MRCC_ReturnType_t MRCC_DisablePeripheralClock(MRCC_Peripheral_t Copy_Peripheral);

#endif /* MRCC_INTERFACE_H_ */
