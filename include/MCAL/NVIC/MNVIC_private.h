/*
 * MNVIC_private.h
 *
 *  PRIVATE header of the MNVIC driver — never include from HAL or
 *  Application code. Hardware facts only: base addresses, the register
 *  map (offsets verified), AIRCR definitions, bit positions.
 *
 *  Reference: ARM PM0214 (Cortex-M4 Devices Generic User Guide)
 *             §4.3 (NVIC), §4.2 (SCB)
 *
 *  Revision history:
 *    Rev 1 — original (verified register map; bare AIRCR pointer)
 *    Rev 2 — AIRCR bit definitions; naming sweep (MNVIC_t ->
 *            MNVIC_Registers_t); SCB_AIRCR -> MNVIC_SCB_AIRCR
 *
 *  Created on: Aug 1, 2026
 *      Author: dell
 */

#ifndef MNVIC_PRIVATE_H_
#define MNVIC_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Base addresses
 *
 *  The NVIC is a CORE peripheral (Cortex-M4 silicon, not ST), living
 *  in the PPB region — like SysTick, it needs no RCC clock enable.
 *
 *  The whole region used by this driver:
 *      NVIC registers : 0xE000E100 .. 0xE000E4FF
 *      IPR (byte map) : 0xE000E400 .. 0xE000E4EF (240 bytes)
 *      STIR           : 0xE000EF00
 *  AIRCR lives over in the SCB: 0xE000ED0C.
 * ===================================================================== */
#define MNVIC_BASE_ADDRESS              (0xE000E100UL)

/* =====================================================================
 *  NVIC register map (PM0214 §4.3 — offsets verified)
 *
 *  The struct covers the full 240-IRQ NVIC architecture (F401 uses
 *  only IRQs 0..84; the rest of the arrays exists but is
 *  unimplemented). Field semantics:
 *      ISER : write-1-to-SET   enable, per IRQ bit. Reading returns
 *                            the current enable state.
 *      ICER : write-1-to-CLEAR enable ("disable"). The write pattern
 *                            matters: write only the bit(s) to change.
 *      ISPR : write-1-to-SET   pending
 *      ICPR : write-1-to-CLEAR pending
 *      IABR : READ-ONLY — 1 while the IRQ is actively executing
 *                            (set on entry, cleared on exit)
 *      IPR  : 8-bit priority per IRQ, only bits [7:4] implemented
 *                            on the F401 ([3:0] read 0, writes ignored)
 *      STIR : software interrupt trigger — writing an IRQ number
 *                            sets that IRQ pending (see TriggerIRQ)
 *
 *  W1S/W1C discipline: because writes only affect bits written as 1,
 *  the idiom for all of ISER/ICER/ISPR/ICPR is the DIRECT store of
 *  (1 << bit) — one transaction, no read-modify-write. RMW on these
 *  registers works (they are readable) but implies register state
 *  that is simply not needed and is not the architectural pattern.
 * ===================================================================== */
typedef struct
{
    volatile u32 NVIC_ISER[8];      /* 0xE000E100 : enable set           */
    volatile u32 RESERVED0[24];     /* 0xE000E120 : (gap to 0x180)       */
    volatile u32 NVIC_ICER[8];      /* 0xE000E180 : enable clear         */
    volatile u32 RESERVED1[24];     /* 0xE000E1A0 : (gap to 0x200)       */
    volatile u32 NVIC_ISPR[8];      /* 0xE000E200 : pending set          */
    volatile u32 RESERVED2[24];     /* 0xE000E220 : (gap to 0x280)       */
    volatile u32 NVIC_ICPR[8];      /* 0xE000E280 : pending clear        */
    volatile u32 RESERVED3[24];     /* 0xE000E2A0 : (gap to 0x300)       */
    volatile u32 NVIC_IABR[8];      /* 0xE000E300 : active (read-only)   */
    volatile u32 RESERVED4[56];     /* 0xE000E320 : (gap to 0x400)       */
    volatile u8  NVIC_IPR[240];     /* 0xE000E400 : priorities (byte map)*/
    volatile u32 RESERVED5[644];    /* 0xE000E4F0 : (gap to 0xF00)       */
    volatile u32 NVIC_STIR;         /* 0xE000EF00 : software trigger     */
} MNVIC_Registers_t;

/* Register map access point (program.c only) */
#define MNVIC                           ((volatile MNVIC_Registers_t *) MNVIC_BASE_ADDRESS)

/* =====================================================================
 *  SCB AIRCR — Application Interrupt and Reset Control Register
 *
 *  This driver touches AIRCR for exactly ONE field: PRIGROUP
 *  (bits 10:8), written once by MNVIC_Init from the config value.
 *
 *  THE WRITE-KEY TRAP (why this register gets special treatment):
 *  every write must carry 0x05FA in bits 31:16 or the ENTIRE write
 *  is silently ignored. But on READ those bits return VECTKEYSTAT
 *  = 0xFA05 — NOT the write key. A naive read-modify-write would
 *  write 0xFA05 back as the "key", the write would be silently
 *  dropped, and the grouping would never change. The correct
 *  pattern (mask low 16 bits, then OR in the real key) is used and
 *  documented in MNVIC_Init.
 *
 *  RMW safety of the low half: the preserved bits read as —
 *  ENDIANNESS (bit 15) = 0 on STM32 (little-endian), VECTRESET,
 *  VECTCLRACTIVE, SYSRESETREQ (bits 0..2) all read 0 on a healthy
 *  part — so nothing dangerous is ever written back. A read-back of
 *  PRIGROUP after the write converts "write silently ignored" into
 *  a returned MNVIC_INIT_FAILED.
 * ===================================================================== */
#define MNVIC_SCB_AIRCR_ADDRESS         (0xE000ED0CUL)
#define MNVIC_SCB_AIRCR                 (*(volatile u32 *) MNVIC_SCB_AIRCR_ADDRESS)

#define MNVIC_AIRCR_VECTKEY_POS         16      /* Bits 31:16 : key field          */
#define MNVIC_AIRCR_VECTKEY_WRITE       (0x05FAUL)  /* the value writes require    */
#define MNVIC_AIRCR_VECTKEYSTAT_READ    (0xFA05UL)  /* what reads return (trap!)   */
#define MNVIC_AIRCR_ENDIANNESS_POS      15      /* Bit  15 : 0 = little-endian      */
#define MNVIC_AIRCR_PRIGROUP_POS        8       /* Bits 10:8 : priority grouping    */
#define MNVIC_AIRCR_PRIGROUP_MASK       (0x7u << MNVIC_AIRCR_PRIGROUP_POS)
#define MNVIC_AIRCR_LOW16_MASK          (0x0000FFFFUL)  /* key-free preservation   */

/* =====================================================================
 *  Interrupt position math (PM0214: ISER/ICER/ISPR/ICPR/IABR are
 *  bit-per-IRQ across 32-bit words):
 *      word index = IRQ >> 5        bit within word = IRQ & 0x1F
 *  Kept as private helpers next to their only consumer.
 * ===================================================================== */
#define MNVIC_IRQ_WORD(irq)             ((u32)(irq) >> 5u)
#define MNVIC_IRQ_BIT(irq)              (1uL << ((u32)(irq) & 0x1Fu))


/* =====================================================================
 *  SCB VTOR — vector table offset register (Rev 3, FOTA)
 *
 *  VTOR tells the core where the vector table lives once code runs
 *  from a base other than 0x08000000. Bits [31:7] are implemented:
 *  the base must be at least 128-byte aligned (ARMv7-M floor). The
 *  FOTA layout's regions are 16 KB aligned — far above it.
 * ===================================================================== */
#define MNVIC_SCB_VTOR_ADDRESS          (0xE000ED08UL)
#define MNVIC_SCB_VTOR                  (*(volatile u32 *) MNVIC_SCB_VTOR_ADDRESS)
#define MNVIC_VTOR_ALIGNMENT_MASK       (0x7Fu)

/* AIRCR reset-control bits (Rev 3) — only SYSRESETREQ is used;
   VECTRESET/VECTCLRACTIVE are DEBUG-ONLY (ARM reserves VECTRESET). */
#define MNVIC_AIRCR_VECTRESET           0
#define MNVIC_AIRCR_VECTCLRACTIVE       1
#define MNVIC_AIRCR_SYSRESETREQ         2



#endif /* MNVIC_PRIVATE_H_ */
