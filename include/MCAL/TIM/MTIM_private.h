/*
 * MTIM_private.h
 *
 *  PRIVATE header of the MTIM driver - never include from HAL or
 *  Application code. Hardware facts only: base addresses, the register
 *  map (one superset struct for all three timer shapes), bit
 *  positions, and field masks.
 *
 *  Reference: RM0368 Rev 6, Ch 12/13/14 (register sections verified)
 *
 *  Revision history:
 *    Rev 1 - first release
 *
 *  Created on: Sep 12, 2026
 *      Author: dell
 */

#ifndef MTIM_PRIVATE_H_
#define MTIM_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Timer base addresses (RM0368 -2.3 memory map)
 *
 *  APB1: TIM2 0x40000000, TIM3 0x40000400, TIM4 0x40000800,
 *        TIM5 0x40000C00
 *  APB2: TIM1 0x40010000, TIM9 0x40014000, TIM10 0x40014400,
 *        TIM11 0x40014800
 * ===================================================================== */
#define MTIM_TIM1_BASE_ADDRESS     0x40010000UL
#define MTIM_TIM2_BASE_ADDRESS     0x40000000UL
#define MTIM_TIM3_BASE_ADDRESS     0x40000400UL
#define MTIM_TIM4_BASE_ADDRESS     0x40000800UL
#define MTIM_TIM5_BASE_ADDRESS     0x40000C00UL
#define MTIM_TIM9_BASE_ADDRESS     0x40014000UL
#define MTIM_TIM10_BASE_ADDRESS    0x40014400UL
#define MTIM_TIM11_BASE_ADDRESS    0x40014800UL

/* =====================================================================
 *  Register map - ONE SUPERSET STRUCT for all three timer shapes
 *
 *  The three shapes (advanced TIM1 / general TIM2-5 / limited
 *  TIM9-11) share the offsets of every register that exists on them.
 *  Absent registers are protected by per-instance capability flags in
 *  the program.c table (channel count, advanced), validated before
 *  every access - the MGPIO mask pattern applied to registers. The
 *  struct fields for absent registers are never touched on those
 *  instances; they exist for offset layout only.
 *
 *  Width facts (load-bearing for the API):
 *    - PSC is 16-bit on ALL timers, even the 32-bit ones
 *    - CNT/ARR/CCR are 32-bit on TIM2/TIM5, 16-bit elsewhere
 *      (writes to the high half of a 16-bit register are ignored by
 *      hardware; the solver clamps ARR to the instance width anyway)
 *
 *  TIM2_OR / TIM5_OR (RM0368 -13.4.19/20) exist but are DELIBERATELY
 *  NOT mapped: neither affects CH1 (our only capture use, PA0). Map
 *  and verify against the manual before any future channel-remap use.
 * ===================================================================== */
typedef struct
{
    volatile u32 CR1;      /* 0x00 : control 1 (CEN/ARPE)                        */
    volatile u32 CR2;      /* 0x04 : control 2 - absent TIM9/10/11 (unused)      */
    volatile u32 SMCR;     /* 0x08 : slave mode - NOT USED (no sync in Rev 1)    */
    volatile u32 DIER;     /* 0x0C : DMA/interrupt enables                       */
    volatile u32 SR;       /* 0x10 : status - WRITE-0-TO-CLEAR                   */
    volatile u32 EGR;      /* 0x14 : event generation (UG force-update)          */
    volatile u32 CCMR1;    /* 0x18 : ch1 (low byte) + ch2 (high byte) modes      */
    volatile u32 CCMR2;    /* 0x1C : ch3 + ch4 - absent TIM9/10/11               */
    volatile u32 CCER;     /* 0x20 : capture/compare enables + polarities        */
    volatile u32 CNT;      /* 0x24 : the counter                                 */
    volatile u32 PSC;      /* 0x28 : prescaler (16-bit; latched at update!)      */
    volatile u32 ARR;      /* 0x2C : auto-reload (the period)                    */
    volatile u32 RCR;      /* 0x30 : repetition counter - TIM1 ONLY, kept 0      */
    volatile u32 CCR1;     /* 0x34 : capture/compare 1                           */
    volatile u32 CCR2;     /* 0x38 : capture/compare 2 - absent TIM10/11         */
    volatile u32 CCR3;     /* 0x3C : capture/compare 3 - absent TIM9/10/11       */
    volatile u32 CCR4;     /* 0x40 : capture/compare 4 - absent TIM9/10/11       */
    volatile u32 BDTR;     /* 0x44 : break/dead-time - TIM1 ONLY (MOE!)          */
    volatile u32 DCR;      /* 0x48 : DMA control - absent TIM9/10/11 (unused)    */
    volatile u32 DMAR;     /* 0x4C : DMA address  - absent TIM9/10/11 (unused)   */
} MTIM_Registers_t;

/* Register map access points - the _REG SUFFIX IS LOAD-BEARING (Rev 1a).
   The bare names MTIM_TIM1..MTIM_TIM11 are the INSTANCE ENUM in
   MTIM_interface.h. Rev 1 defined these macros with the same bare names,
   so every enum use in the project expanded to a pointer cast - the
   instance table, the static asserts and the ISR dispatch calls all
   broke at once. Macro-vs-enum collision: the MEXTI Rev 2a lesson,
   now retired library-wide - register macros always carry a suffix
   distinct from the instance enum (MGPIO: GPIOA vs PORTA;
   MUSART: MUSART1 vs USART_1; here: MTIM_TIMx_REG vs MTIM_TIMx). */
#define MTIM_TIM1_REG   ((volatile MTIM_Registers_t *) MTIM_TIM1_BASE_ADDRESS)
#define MTIM_TIM2_REG   ((volatile MTIM_Registers_t *) MTIM_TIM2_BASE_ADDRESS)
#define MTIM_TIM3_REG   ((volatile MTIM_Registers_t *) MTIM_TIM3_BASE_ADDRESS)
#define MTIM_TIM4_REG   ((volatile MTIM_Registers_t *) MTIM_TIM4_BASE_ADDRESS)
#define MTIM_TIM5_REG   ((volatile MTIM_Registers_t *) MTIM_TIM5_BASE_ADDRESS)
#define MTIM_TIM9_REG   ((volatile MTIM_Registers_t *) MTIM_TIM9_BASE_ADDRESS)
#define MTIM_TIM10_REG  ((volatile MTIM_Registers_t *) MTIM_TIM10_BASE_ADDRESS)
#define MTIM_TIM11_REG  ((volatile MTIM_Registers_t *) MTIM_TIM11_BASE_ADDRESS)

/* ===================== CR1 bits ===================== */
#define MTIM_CR1_CEN           0u    /* Bit 0 : counter enable                */
#define MTIM_CR1_URS           2u    /* Bit 2 : update request source (unused)*/
#define MTIM_CR1_ARPE          7u    /* Bit 7 : ARR preload enable            */

/* ===================== DIER bits ===================== */
#define MTIM_DIER_UIE          0u    /* Bit 0 : update interrupt enable       */
#define MTIM_DIER_CC1IE        1u    /* Bits 1-4 : capture/compare x IE       */
#define MTIM_DIER_CC2IE        2u
#define MTIM_DIER_CC3IE        3u
#define MTIM_DIER_CC4IE        4u
#define MTIM_DIER_UDE          8u    /* Bit 8 : update DMA request - the
                                        AUDIO HOOK (Option b sample pacing)  */

/* ===================== SR bits =====================
 *  WRITE-0-TO-CLEAR: `SR = ~mask` clears only `mask` - 1s elsewhere
 *  leave flags unchanged. This is the MUSART-TC idiom and the OPPOSITE
 *  of EXTI's PR (write-1). Do not port the wrong idiom between the
 *  two drivers. Reading SR has NO side effects (no coupled clear
 *  sequences anywhere in this peripheral - unlike USART SR).
 *  CCxOF = overcapture: a second edge arrived before CCR was read.
 *  Cleared alongside CCxIF in the ISR; the delivered timestamp is the
 *  OLDER capture (documented for the IR decoder).                     */
#define MTIM_SR_UIF            0u
#define MTIM_SR_CC1IF          1u
#define MTIM_SR_CC2IF          2u
#define MTIM_SR_CC3IF          3u
#define MTIM_SR_CC4IF          4u
#define MTIM_SR_CC1OF          9u
#define MTIM_SR_CC2OF          10u
#define MTIM_SR_CC3OF          11u
#define MTIM_SR_CC4OF          12u

/* ===================== EGR bits ===================== */
#define MTIM_EGR_UG            0u    /* Bit 0 : force update - reloads the
                                        shadowed PSC/ARR and zeroes CNT.
                                        LOAD-BEARING: PSC written to the
                                        register is NOT active until the
                                        next update event; the config
                                        sequence forces UG to latch it
                                        immediately. Side effect: UG SETS
                                        UIF - always clear UIF after.     */

/* ===================== CCMR input/output fields ===================== */
/* Input capture selects: CC1S=01 maps TI1->CH1; CC2S=01 maps TI2->CH2;
   CC2S=10 maps TI1->CH2 - the BOTH-EDGES trick: one pin captured on
   two channels with independent polarity (the NEC pattern).          */
#define MTIM_CCMR_CCxS_MASK          0x3u
#define MTIM_CCMR_IN_TIx             0x1u   /* channel's own pin            */
#define MTIM_CCMR_IN_TI1_SHARED      0x2u   /* CH2 mapped onto TI1          */

/* Output compare: PWM mode 1 (110) + preload enable - one byte value
   replicated into the channel's CCMR byte by the driver.              */
#define MTIM_CCMR_OCXM_PWM1          0x6u   /* bits [6:4] -> 110            */
#define MTIM_CCMR_OCXPE              0x1u   /* bit 3                        */
#define MTIM_CCMR_OC_PWM1_PE         ((MTIM_CCMR_OCXM_PWM1 << 4) | (MTIM_CCMR_OCXPE << 3))

/* ===================== CCER bits =====================
 *  Per channel: CCxE at ch*4, CCxP at ch*4+1.
 *  Output:  CCxP=0 active-high, 1 active-low (inverted).
 *  Input:   CCxP=0 rising,     1 falling.                              */
#define MTIM_CCER_CCXE(ch)     (0x1u << ((u32)(ch) * 4u))
#define MTIM_CCER_CCXP(ch)     (0x1u << ((u32)(ch) * 4u + 1u))
#define MTIM_CCER_CCX_MASK(ch) (0x3u << ((u32)(ch) * 4u))

/* ===================== BDTR bits (TIM1 ONLY) =====================
 *  MOE (bit 15) - THE TIM1 TRAP: all four outputs are gated by MOE,
 *  reset value 0. PWM configured but MOE unset produces SILENTLY
 *  NOTHING - this session's favorite bug class wearing a timer costume.
 *  AOE=0 (reset, kept): a break event clears MOE and only software
 *  can restore it. We never enable break, so MOE stays once set.      */
#define MTIM_BDTR_AOE          14u
#define MTIM_BDTR_MOE          15u

/* ===================== derived helpers ===================== */
#define MTIM_FLAG(bit)         (1uL << (u32)(bit))
#define MTIM_CC_IF_BIT(ch)     (1u + (u32)(ch))
#define MTIM_CC_OF_BIT(ch)     (9u + (u32)(ch))

#endif /* MTIM_PRIVATE_H_ */
