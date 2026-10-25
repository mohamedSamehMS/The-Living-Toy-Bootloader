/*
 * MEXTI_private.h
 *
 *  PRIVATE header of the MEXTI driver — never include from HAL or
 *  Application code. Hardware facts only.
 *
 *  Contains BOTH register maps: EXTI and SYSCFG.
 *
 *  THE SYSCFG FOLD (a documented, deliberate architecture decision):
 *  SYSCFG is NOT a standalone driver in this library — it is folded
 *  into MEXTI, because on the F401 SYSCFG's ONLY Project 4 function
 *  is EXTI line routing (EXTICR). Its other registers are dead weight
 *  for this project:
 *    - MEMRMP (memory remap): runtime aliasing of address 0x00000000.
 *      Boot source on the F401 is chosen by the BOOT pins at reset;
 *      the FOTA "bootloader/app vector jump" the roadmap mentions is
 *      SCB VTOR work (Phase 2), NOT MEMRMP. Zero consumers.
 *    - PMC (peripheral mode config): FULLY RESERVED on the F401 —
 *      its Ethernet MII/RMII and FMC remap bits exist only on larger
 *      F4 siblings. Reads 0, writes ignored.
 *  Roadmap amendment: the "SYSCFG ✅ Built" row is now "absorbed into
 *  MEXTI (EXTI routing — its only F401 function)".
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 10: EXTI / SYSCFG
 *
 *  Revision history:
 *    Rev 1 — original (correct maps; unprefixed macros)
 *    Rev 2 — prefix sweep (EXTI -> MEXTI, SYSCFG -> MSYSCFG); the
 *            fold made explicit; W1C discipline documented at PR
 *
 *  Created on: Aug 11, 2026
 *      Author: dell
 */

#ifndef MEXTI_PRIVATE_H_
#define MEXTI_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  EXTI register map (RM0368 §10 — offsets verified)
 *
 *  EXTI is clocked via... nothing peripheral-specific: it lives on the
 *  APB2 bus and only needs the SYSCFG clock for EXTICR (the register
 *  physically lives in SYSCFG anyway — the EXTI block itself is always
 *  accessible).
 *
 *  Field semantics:
 *    IMR   : interrupt mask — bit N = 1 UNMASKS line N (plain RW; the
 *            set/unset idiom here IS read-modify-write — there is no
 *            set/clear alias pair for IMR; safe because the ISR never
 *            writes IMR, only boot/config code does)
 *    EMR   : event mask — event-mode line output. NEVER touched by
 *            this driver (no use for event/wakeup mode) — kept for
 *            map completeness only
 *    RTSR  : rising trigger enable per line (RW)
 *    FTSR  : falling trigger enable per line (RW). BOTH bits set =
 *            pending on either edge (legal, is MEXTI_TRIGGER_ON_CHANGE)
 *    SWIER : software interrupt event — writing 1 sets the line's PR
 *            pending bit; cleared when PR is cleared. THE zero-hardware
 *            full-path test mechanism (see MEXTI_TriggerLine)
 *    PR    : pending register — set by hardware on a selected edge OR
 *            by SWIER. W1C: write 1 to clear, writing 0 has no effect.
 *            CRITICAL DISCIPLINE (the trap this driver must never
 *            fall into): PR is NEVER read-modify-written. `PR |= x`
 *            would write 1 to every pending bit read as 1 — silently
 *            clearing OTHER lines' events before their shared handler
 *            has run. The only idiom is the DIRECT STORE of the one
 *            bit being serviced.
 * ===================================================================== */
#define MEXTI_EXTI_BASE_ADDRESS   0x40013C00UL

typedef struct
{
    volatile u32 IMR;      /* 0x00 : Interrupt mask register     */
    volatile u32 EMR;      /* 0x04 : Event mask register (unused) */
    volatile u32 RTSR;     /* 0x08 : Rising trigger selection    */
    volatile u32 FTSR;     /* 0x0C : Falling trigger selection   */
    volatile u32 SWIER;    /* 0x10 : Software interrupt event    */
    volatile u32 PR;       /* 0x14 : Pending register (W1C!)     */
} MEXTI_Registers_t;

#define MEXTI   ((volatile MEXTI_Registers_t *) MEXTI_EXTI_BASE_ADDRESS)

/* =====================================================================
 *  SYSCFG register map (RM0368 §10.2 — offsets verified, folded here)
 *
 *  EXTICR[0..3]: 4 bits per EXTI line — line N's field lives at
 *      EXTICR[N >> 2], bits [(N & 3) * 4]. Encodings: 0000=PA,
 *      0001=PB, 0010=PC (also 0011=PD, 0100=PE, 0111=PH on the F401 —
 *      this driver's port set is A..C per MGPIO). RESET VALUE 0 = PA
 *      on every line: un-routed lines default to port A.
 *  MEMRMP / PMC: present in the map for offset correctness, UNUSED
 *      (see the fold note in the file header).
 * ===================================================================== */
#define MEXTI_SYSCFG_BASE_ADDRESS  0x40013800UL

typedef struct
{
    volatile u32 MEMRMP;     /* 0x00 : memory remap (unused — see fold note) */
    volatile u32 PMC;        /* 0x04 : RESERVED on F401 (unused)            */
    volatile u32 EXTICR[4];  /* 0x08..0x14 : EXTI routing (the fold's point) */
} MSYSCFG_Registers_t;

#define MSYSCFG   ((volatile MSYSCFG_Registers_t *) MEXTI_SYSCFG_BASE_ADDRESS)

/* EXTICR port-source encodings — values ARE the register encodings.
   MGPIO_Port_t is reused as the config/API type; these asserts in
   program.c pin the equality.                                        */
#define MEXTI_EXTICR_PORTA   0u
#define MEXTI_EXTICR_PORTB   1u
#define MEXTI_EXTICR_PORTC   2u

/* Total GPIO EXTI lines (0..15). Lines 16..22 exist on the part but
   are INTERNAL (PVD/RTC-alarm/OTG-wake/tamper/RTC-wake) — hardwired
   to their owning peripherals, NOT routed through EXTICR, and owned
   by future MPWR/MRTC drivers, not this one.                        */
#define MEXTI_NUM_LINES      16u

/* One-hot line mask — u-suffixed, line cast to u32 BEFORE shifting
   (the library-wide unsigned-shift discipline).                    */
#define MEXTI_LINE_MASK(line)   (1uL << (u32)(line))

/* EXTICR field access for line N: register index and 4-bit field    */
#define MEXTI_EXTICR_INDEX(line)   ((u32)(line) >> 2u)
#define MEXTI_EXTICR_SHIFT(line)   (((u32)(line) & 3u) * 4u)
#define MEXTI_EXTICR_MASK(line)    (0xFu << MEXTI_EXTICR_SHIFT(line))

#endif /* MEXTI_PRIVATE_H_ */
