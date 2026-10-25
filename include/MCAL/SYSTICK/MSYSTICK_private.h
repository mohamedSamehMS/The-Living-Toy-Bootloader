/*
 * MSYSTICK_private.h
 *
 *  PRIVATE header of the MSYSTICK driver — never include from HAL or
 *  Application code. Hardware facts only: base addresses, the register
 *  map, bit positions, and internal constants.
 *
 *  Reference: ARM PM0214 (Cortex-M4 Devices Generic User Guide) §4.4
 *
 *  Revision history:
 *    Rev 1 — original (SYSTIC spelling, extern-ed ms counter)
 *    Rev 2 — renamed macros/struct; ms counter made file-static;
 *            SHPR3 definitions added for SysTick priority
 *    Rev 3 — revision-history block introduced; no content change
 *            (the us-delay redesign of Rev 3 adds no registers)
 *
 *  NOTE — the old version extern-ed Global_u32MilliTicks here. It is
 *  now file-static in MSYSTICK_program.c: the getter and the callback
 *  are the ONLY sanctioned interfaces. Any file that fails to compile
 *  after this change was committing a layering violation — unwind it
 *  through the getter, do not re-add the extern.
 *
 *  Created on: Jul 29, 2026
 *      Author: dell
 */

#ifndef MSYSTICK_PRIVATE_H_
#define MSYSTICK_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  SysTick register map (PM0214 §4.4 — offsets verified)
 *
 *  SysTick is a CORE peripheral (part of the Cortex-M4 itself, not
 *  ST silicon), which is why it sits in the PPB region 0xE000E010 —
 *  and why it needs no RCC clock enable: it is always clocked, and
 *  the RCC "SYSCFGEN"-style gating does not apply to it.
 * ===================================================================== */
#define MSYSTICK_SYSTICK_BASE_ADDRESS      0xE000E010UL

typedef struct
{
    volatile u32 CTRL;      /* 0x00 : Control and status register       */
    volatile u32 LOAD;      /* 0x04 : Reload value register            */
    volatile u32 VAL;       /* 0x08 : Current value register           */
    volatile u32 CALIB;     /* 0x0C : Calibration value register       */
} MSYSTICK_Registers_t;

/* Access point — correctly spelled now (the old macro, SYSTIC, was
   missing the K). Consumed by program.c only. */
#define SYSTICK                             ((volatile MSYSTICK_Registers_t *) MSYSTICK_SYSTICK_BASE_ADDRESS)

/* 24-bit maximum the LOAD register can hold (PM0214: VAL/LOAD are
   24 bits wide — bits 31:24 read zero, writes ignored) */
#define MSYSTICK_MAX_LOAD_VALUE             0xFFFFFFUL

/* =====================================================================
 *  CTRL bits (PM0214 §4.4.2)
 *
 *  COUNTFLAG (bit 16): set by hardware when the counter reaches 0.
 *  Cleared by: ANY read of CTRL, or ANY write to VAL. Both facts are
 *  load-bearing in this driver — see MSYSTICK_HasCounted (read-to-
 *  clear) and MSYSTICK_Init (VAL write conveniently clears it).
 * ===================================================================== */
#define MSYSTICK_CTRL_ENABLE                0     /* Bit  0 : counter enable          */
#define MSYSTICK_CTRL_TICKINT               1     /* Bit  1 : exception on reach-0    */
#define MSYSTICK_CTRL_CLKSOURCE             2     /* Bit  2 : 0 = HCLK/8, 1 = HCLK    */
#define MSYSTICK_CTRL_COUNTFLAG             16    /* Bit 16 : read-to-clear wrap flag */

/* =====================================================================
 *  SysTick exception priority — SCB, not NVIC
 *
 *  SysTick is system exception 15. Its priority byte is SHPR3[31:24]
 *  (PM0214 §4.2.5). RMW on SHPR3 preserves PendSV's field (bits
 *  23:16, exception 14) — safe because Init is a single-caller,
 *  boot-time operation.
 *
 *  The F401 implements 4 of the 8 priority bits (__NVIC_PRIO_BITS=4,
 *  an ST device fact — ARM leaves it implementation-defined). A
 *  logical priority 0..15 must therefore be shifted left by 4 to land
 *  in the implemented (upper) nibble of the byte.
 *
 *  FUTURE: when the MNVIC driver grows system-handler support
 *  (SHPR1-3), this definition and the write in MSYSTICK_Init should
 *  move there, so ALL interrupt priorities in the project are set
 *  from one place. Until then this driver owns its own byte.
 * ===================================================================== */
#define MSYSTICK_SCB_SHPR3_ADDRESS          0xE000ED20UL
#define MSYSTICK_SCB_SHPR3                  (*(volatile u32 *) MSYSTICK_SCB_SHPR3_ADDRESS)

#define MSYSTICK_SHPR3_SYSTICK_BYTE_POS     24    /* SHPR3[31:24] — exception 15     */
#define MSYSTICK_PRIORITY_BITS              4     /* F401 implements 4 priority bits */
#define MSYSTICK_PRIORITY_REGISTER_SHIFT    (8u - MSYSTICK_PRIORITY_BITS)   /* = 4   */

/* CALIB / TENMS: on ST devices the calibration value reads 0 —
   ST does not implement the calibration feature. The struct field
   keeps the register offset correct; never trust its contents. */

#endif /* MSYSTICK_PRIVATE_H_ */
