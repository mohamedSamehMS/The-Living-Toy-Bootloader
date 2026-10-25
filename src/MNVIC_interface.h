/*
 * MNVIC_interface.h
 *
 *  PUBLIC interface of the MNVIC driver. HAL and Application code
 *  must include ONLY this file.
 *
 *  Reference: ARM PM0214 §4.3 (NVIC), §4.2 (SCB/AIRCR)
 *             Verified against startup_stm32f401xc.s (xPacks) vector
 *             table — the IRQ enum below is ground-truthed, not
 *             guessed: positions counted directly from that file.
 *
 *  BOOT-ORDER CONTRACT:
 *      1. MRCC_Init()           (clock tree)
 *      2. MNVIC_Init()          — programs AIRCR.PRIGROUP from config
 *      3. MNVIC_SetPriority()   for each interrupt in use
 *      4. ... driver inits that enable their IRQs (MEXTI, MUSART...)
 *      5. MNVIC_EnableInterrupt() — ARM's recommended order: priority
 *         BEFORE enable. (Re-prioritizing an already-active IRQ is
 *         legal — the new value governs subsequent preemption — but
 *         init-order avoids the question entirely.)
 *
 *  CONCURRENCY CONTRACT: every register access here is either a
 *  single-word store (ISER/ICER/ISPR/ICPR/STIR) or a single-byte
 *  store/read (IPR) — all atomic, safe to call from ISR context.
 *
 *  SCOPE (deliberate): this driver is PERIPHERAL-IRQ only. System
 *  exceptions (SysTick/PendSV/...) keep their owners — MSYSTICK
 *  programs its own SHPR3 byte. One owner per register, no dual
 *  writers. When Phase 3 (FreeRTOS) needs VTOR/PendSV work, MNVIC
 *  grows SHPR1-3 support and the SysTick write migrates here.
 *
 *  Revision history:
 *    Rev 1 — original (raw u8 positions, runtime group static)
 *    Rev 2 — typed IRQ enum ground-truthed to the vector table;
 *            compile-time grouping; status returns; hole validation
 *
 *  Created on: Aug 1, 2026
 *      Author: dell
 */

#ifndef MNVIC_INTERFACE_H_
#define MNVIC_INTERFACE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Driver status codes — same convention as MRCC/MGPIO/MSYSTICK
 * ===================================================================== */
typedef enum
{
    MNVIC_OK = 0,           /* Operation completed                        */
    MNVIC_NOT_INITIALIZED,  /* SetPriority called before MNVIC_Init —
                               the AIRCR grouping is not yet programmed,
                               so the byte would encode against the WRONG
                               split; nothing was written                */
    MNVIC_INIT_FAILED,      /* Init: AIRCR read-back mismatch — the write
                               was silently ignored (wrong-key class)    */
    MNVIC_INVALID_IRQ,      /* Position outside 0..84, or a family-layout
                               hole (interrupt absent on the F401)       */
    MNVIC_INVALID_PARAM     /* Priority value beyond the grouping's
                               field width, or a NULL pointer            */
} MNVIC_ReturnType_t;

/* =====================================================================
 *  IRQ positions — STM32F401, ground-truthed against the startup file
 *
 *  The F4 family keeps a FIXED vector layout: peripherals absent on
 *  the F401 (CAN, USART3, TIM6/7/8/12-14, FSMC, Ethernet-class...)
 *  leave their slots EMPTY — positions do not compact. 29 of the 85
 *  positions (0..84) are holes; the driver rejects them with
 *  MNVIC_INVALID_IRQ (see the validity mask in program.c).
 *
 *  Positions verified by counting the startup_stm32f401xc.s vector
 *  table: EXTI9_5 = 23, USART1 = 37, EXTI15_10 = 40, DMA2_Stream0 =
 *  56, SPI4 = 84 (last). _Static_asserts in program.c pin these.
 *
 *  TIM notes for the Phase-1 timer driver: TIM1's break/update/
 *  trigger-commutation/CC positions are SHARED with TIM9/TIM10/
 *  TIM11 (24/25/26) — enabling TIM9's update interrupt and TIM1's
 *  CC interrupt goes through DIFFERENT positions, while TIM9's
 *  update lives at 24 with TIM1 break.
 *
 *  EXTI notes for the MEXTI revision: PVD/TAMP_STAMP/RTC_WKUP/
 *  RTC_Alarm/OTG_FS_WKUP are non-GPIO EXTI lines (16, 21, 22, 17,
 *  18) delivered through their OWN NVIC positions below.
 *
 *  Porting to another STM32: regenerate this enum from that part's
 *  startup vector table — the positions are family facts, not
 *  choices.
 * ===================================================================== */
typedef enum
{
    /* ---- 0..18 : first block ---- */
    MNVIC_WWDG              = 0,    /* Window watchdog                          */
    MNVIC_PVD               = 1,    /* PVD through EXTI line 16                 */
    MNVIC_TAMP_STAMP        = 2,    /* Tamper/timestamp through EXTI line 21    */
    MNVIC_RTC_WKUP          = 3,    /* RTC wakeup through EXTI line 22          */
    MNVIC_FLASH             = 4,    /* Flash (Phase 1 driver)                   */
    MNVIC_RCC               = 5,    /* RCC (CSS-on if ever enabled)             */
    MNVIC_EXTI0             = 6,    /* EXTI line 0 — buttons                    */
    MNVIC_EXTI1             = 7,    /* EXTI line 1                              */
    MNVIC_EXTI2             = 8,    /* EXTI line 2                              */
    MNVIC_EXTI3             = 9,    /* EXTI line 3                              */
    MNVIC_EXTI4             = 10,   /* EXTI line 4                              */
    MNVIC_DMA1_STREAM0      = 11,   /* DMA1 stream 0 (Phase 1)                  */
    MNVIC_DMA1_STREAM1      = 12,   /* DMA1 stream 1                            */
    MNVIC_DMA1_STREAM2      = 13,   /* DMA1 stream 2                            */
    MNVIC_DMA1_STREAM3      = 14,   /* DMA1 stream 3                            */
    MNVIC_DMA1_STREAM4      = 15,   /* DMA1 stream 4                            */
    MNVIC_DMA1_STREAM5      = 16,   /* DMA1 stream 5                            */
    MNVIC_DMA1_STREAM6      = 17,   /* DMA1 stream 6                            */
    MNVIC_ADC               = 18,   /* ADC1 (only ADC on F401)                  */
    /* 19..22 : CAN1-family slots — absent on F401, NOT valid IRQs     */

    MNVIC_EXTI9_5           = 23,   /* EXTI lines 5..9 (shared handler)        */
    MNVIC_TIM1_BRK_TIM9     = 24,   /* TIM1 break AND TIM9 (shared position)   */
    MNVIC_TIM1_UP_TIM10     = 25,   /* TIM1 update AND TIM10 (shared)          */
    MNVIC_TIM1_TRG_COM_TIM11= 26,   /* TIM1 trig/comm AND TIM11 (shared)       */
    MNVIC_TIM1_CC           = 27,   /* TIM1 capture/compare                    */
    MNVIC_TIM2              = 28,   /* TIM2 (32-bit) — Phase 1                 */
    MNVIC_TIM3              = 29,   /* TIM3 — Phase 1                          */
    MNVIC_TIM4              = 30,   /* TIM4 — Phase 1                          */
    MNVIC_I2C1_EV           = 31,   /* I2C1 event                              */
    MNVIC_I2C1_ER           = 32,   /* I2C1 error                              */
    MNVIC_I2C2_EV           = 33,   /* I2C2 event                              */
    MNVIC_I2C2_ER           = 34,   /* I2C2 error                              */
    MNVIC_SPI1              = 35,   /* SPI1 (TFT / 74HC595)                    */
    MNVIC_SPI2              = 36,   /* SPI2                                    */
    MNVIC_USART1            = 37,   /* USART1 — ESP8266 link                   */
    MNVIC_USART2            = 38,   /* USART2                                  */
    /* 39 : USART3 slot — absent on F401                                */

    MNVIC_EXTI15_10         = 40,   /* EXTI lines 10..15 (shared handler)      */
    MNVIC_RTC_ALARM         = 41,   /* RTC alarm A/B through EXTI line 17      */
    MNVIC_OTG_FS_WKUP       = 42,   /* USB OTG FS wakeup through EXTI line 18  */
    /* 43..46 : TIM8/TIM12-14 slots — absent on F401                    */
    MNVIC_DMA1_STREAM7      = 47,   /* DMA1 stream 7                           */
    /* 48 : FSMC slot — absent on F401                                  */
    MNVIC_SDIO              = 49,   /* SDIO (Phase 2)                          */
    MNVIC_TIM5              = 50,   /* TIM5 (32-bit) — Phase 1                 */
    MNVIC_SPI3              = 51,   /* SPI3                                    */
    /* 52..55 : UART4/UART5/TIM6/TIM7 slots — absent on F401
               (note for the audio plan: the F401 has no TIM6/TIM7 —
               the classic DAC-trigger timers; see MRCC review)       */
    MNVIC_DMA2_STREAM0      = 56,   /* DMA2 stream 0 (Phase 1)                 */
    MNVIC_DMA2_STREAM1      = 57,   /* DMA2 stream 1                           */
    MNVIC_DMA2_STREAM2      = 58,   /* DMA2 stream 2                           */
    MNVIC_DMA2_STREAM3      = 59,   /* DMA2 stream 3                           */
    MNVIC_DMA2_STREAM4      = 60,   /* DMA2 stream 4                           */
    /* 61..66 : Ethernet/CAN2-class family slots — absent on F401      */
    MNVIC_OTG_FS            = 67,   /* USB OTG FS (Phase 2)                    */
    MNVIC_DMA2_STREAM5      = 68,   /* DMA2 stream 5                           */
    MNVIC_DMA2_STREAM6      = 69,   /* DMA2 stream 6                           */
    MNVIC_DMA2_STREAM7      = 70,   /* DMA2 stream 7                           */
    MNVIC_USART6            = 71,   /* USART6                                  */
    MNVIC_I2C3_EV           = 72,   /* I2C3 event                              */
    MNVIC_I2C3_ER           = 73,   /* I2C3 error                              */
    /* 74..80 : OTG_HS-class family slots — absent on F401             */
    MNVIC_FPU               = 81,   /* FPU (roadmap: low relevance)            */
    /* 82..83 : reserved                                                */
    MNVIC_SPI4              = 84    /* SPI4 — LAST F401 IRQ (85 total)         */
} MNVIC_IRQ_t;

/* =====================================================================
 *  API
 * ===================================================================== */

/* Program AIRCR.PRIGROUP from MNVIC_config.h. Idempotent: rewriting
 * the same value changes nothing (no timebase to rewind, unlike
 * MSYSTICK), so a double call simply returns OK. Includes a read-back
 * check that converts a silently-ignored AIRCR write (wrong-key
 * failure class) into MNVIC_INIT_FAILED. Must run before
 * SetPriority; see the boot-order contract in the file header. */
MNVIC_ReturnType_t MNVIC_Init(void);

/* Enable / disable one interrupt. Write-1-to-set / write-1-to-clear
 * direct stores — single transactions. Disabled interrupts keep
 * their pending state: a disabled-but-pending IRQ fires as soon as
 * it is re-enabled (use ClearPending first if that is unwanted). */
MNVIC_ReturnType_t MNVIC_EnableInterrupt (MNVIC_IRQ_t Copy_Irq);
MNVIC_ReturnType_t MNVIC_DisableInterrupt(MNVIC_IRQ_t Copy_Irq);

/* Set / clear / read the PENDING state of one interrupt.
 * Setting pending by software is equivalent to the peripheral having
 * asserted the line: the handler runs once the IRQ is enabled.
 * ClearPending is the correct way to discard a latched spurious
 * event (e.g. before enabling, to skip one stale EXTI hit). */
MNVIC_ReturnType_t MNVIC_SetPending  (MNVIC_IRQ_t Copy_Irq);
MNVIC_ReturnType_t MNVIC_ClearPending(MNVIC_IRQ_t Copy_Irq);
MNVIC_ReturnType_t MNVIC_GetPending  (MNVIC_IRQ_t Copy_Irq, u8 *Copy_pu8Pending);

/* 1 while the IRQ's handler is executing (IABR is set by hardware on
 * exception entry, cleared on exit). Useful for ISR self-diagnostics
 * and for detecting re-entrancy. */
MNVIC_ReturnType_t MNVIC_GetActive   (MNVIC_IRQ_t Copy_Irq, u8 *Copy_pu8Active);

/* Program the priority byte of one interrupt.
 *
 *   Copy_u8GroupPriority : preemption priority, 0 = highest.
 *                           Valid range is derived from the config's
 *                           grouping — e.g. 0..15 at 16/0. A value one
 *                           past the field width used to silently
 *                           truncate toward priority 0 (the MOST
 *                           urgent) — now it returns INVALID_PARAM.
 *   Copy_u8SubPriority   : ordering within the same group (no
 *                           nesting). Always 0 at the default 16/0
 *                           grouping.
 *
 * The full-byte direct store replaces the old read-modify-write
 * (whose &= 0xF preserved only unimplemented bits — a no-op).
 * Returns NOT_INITIALIZED if MNVIC_Init has not run. */
MNVIC_ReturnType_t MNVIC_SetPriority(MNVIC_IRQ_t Copy_Irq,
                                     u8 Copy_u8GroupPriority, u8 Copy_u8SubPriority);

/* Read back the programmed priority, decoded through the same
 * compile-time grouping (debug/verification aid). */
MNVIC_ReturnType_t MNVIC_GetPriority(MNVIC_IRQ_t Copy_Irq,
                                     u8 *Copy_pu8GroupPriority, u8 *Copy_pu8SubPriority);

/* Software-trigger an interrupt: writes STIR, which sets the IRQ
 * pending — the handler runs when enabled. PRIMARY USE: testing
 * interrupt handlers with ZERO hardware attached (an MEXTI handler
 * can be exercised before any button is wired). Note: privileged
 * code only — unprivileged STIR access faults unless SCB CCR's
 * USERSETMPEND bit is set (not touched by this driver; bare-metal
 * super-loop code runs privileged, so this is informational). */
MNVIC_ReturnType_t MNVIC_TriggerIRQ(MNVIC_IRQ_t Copy_Irq);

/* =====================================================================
 *  Suggested Project 4 priority plan (16/0 grouping, 0 = highest)
 *
 *      1  : EXTI (buttons)        — very short, time-critical UI
 *      2  : DMA streams           — data movers must not overrun
 *      3  : USART1 (ESP8266)      — reception integrity
 *      4-5: TIM (game tick, PWM, LED refresh)
 *      15 : SysTick — owned by MSYSTICK (its config, not this file)
 *  0 stays reserved so "an IRQ is accidentally at maximum priority"
 *  is always visible as a misconfiguration.
 * ===================================================================== */

/* =====================================================================
 *  Usage example (button EXTI on line 0):
 *      MNVIC_Init();
 *      MNVIC_SetPriority(MNVIC_EXTI0, 1, 0);
 *      ... MGPIO + MEXTI line configuration ...
 *      MNVIC_EnableInterrupt(MNVIC_EXTI0);
 * ===================================================================== */


/* Set the vector table base address (SCB VTOR).
 *
 * WHO CALLS THIS: the FOTA bootloader just before jumping into the
 * application (app linked at 0x08004000), and the application's own
 * startup — so the app also runs when flashed/debugged standalone.
 * Both call it THROUGH this driver: MNVIC already owns SCB (AIRCR);
 * one owner per register. Address must be >= 128-byte aligned;
 * power-of-two sizing vs. the table size is the caller's job. */
MNVIC_ReturnType_t MNVIC_SetVectorTableBase(u32 Copy_u32Address);

/* Request a full system reset via AIRCR.SYSRESETREQ.
 *
 * WHO CALLS THIS: the FOTA receiver, after a complete image is
 * staged — the bootloader installs it at next boot.
 *
 * DELIBERATELY void — a documented exception to the status-enum
 * convention: this function NEVER RETURNS, and a status return
 * would imply execution continues past the call. SYSRESETREQ (not
 * VECTRESET): resets the whole system except the debug domain. */
void MNVIC_SystemReset(void);

#endif /* MNVIC_INTERFACE_H_ */
