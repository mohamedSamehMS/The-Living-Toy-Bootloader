/*
 * MNVIC_program.c
 *
 *  Implementation of the MNVIC driver.
 *
 *  Reference: ARM PM0214 §4.3 (NVIC), §4.2 (SCB/AIRCR)
 *
 *  DESIGN PRINCIPLES (documented, deliberate):
 *
 *  1. COMPILE-TIME GROUPING. The old runtime static (initialized to
 *     0 — an invalid case of the grouping switch) made every
 *     SetGroupPriority call before SetGroupsConfig a SILENT NO-OP,
 *     leaving the IRQ at reset priority 0 = the HIGHEST priority —
 *     the exact opposite of the caller's intent. Grouping now lives
 *     in MNVIC_config.h; the priority-encoding formula is constant.
 *
 *  2. VALIDATE FIRST, WRITE LAST. Every IRQ position is checked
 *     against the vector-table-derived validity mask (29 of 85
 *     positions are family holes); every priority value is checked
 *     against the grouping's field width — an out-of-range value
 *     used to silently truncate toward priority 0 (most urgent).
 *     Either the whole operation lands, or a status returns.
 *
 *  3. W1S/W1C DISCIPLINE. ISER/ICER/ISPR/ICPR/STIR are written with
 *     direct single-word stores of the one-hot bit — the
 *     architectural idiom for write-1-to-* registers. No RMW, no
 *     implied register state.
 *
 *  4. AIRCR GETS THE WRITE-KEY TREATMENT. VECTKEY reads as 0xFA05
 *     but must be written as 0x05FA — a naive RMW writes the wrong
 *     key and the whole register write is silently ignored. The
 *     mask-then-OR pattern plus a PRIGROUP read-back converts that
 *     silent failure into MNVIC_INIT_FAILED.
 *
 *  Revision history:
 *    Rev 1 — original
 *    Rev 2 — this rewrite: typed IRQ enum, validity mask, compile-
 *            time grouping, status returns, Get TriggerIRQ added
 *
 *  Created on: Aug 1, 2026
 *      Author: dell
 */

#include "BIT_MATH.h"
#include "STD_TYPES.h"

#include <MCAL/NVIC/MNVIC_config.h>
#include <MCAL/NVIC/MNVIC_private.h>
#include "MNVIC_interface.h"

/* =====================================================================
 *  Compile-time consistency guards
 *
 *  These pin the enum to the startup_stm32f401xc.s vector table
 *  (ground-truthed by counting the DCD entries). Any future edit
 *  that inserts/removes an entry breaks the BUILD, not the runtime.
 * ===================================================================== */
_Static_assert(MNVIC_EXTI9_5    == 23, "vector layout: EXTI9_5 must be position 23");
_Static_assert(MNVIC_USART1     == 37, "vector layout: USART1 must be position 37");
_Static_assert(MNVIC_EXTI15_10  == 40, "vector layout: EXTI15_10 must be position 40");
_Static_assert(MNVIC_DMA2_STREAM0 == 56, "vector layout: DMA2_Stream0 must be position 56");
_Static_assert(MNVIC_SPI4       == 84, "vector layout: SPI4 is the LAST F401 IRQ (85 total)");
_Static_assert(MNVIC_TIM1_UP_TIM10 == 25, "vector layout: TIM1_UP/TIM10 shared position");

/* The single priority-encoding formula must reproduce all five
   hand-derived encodings of the old five-case switch (group shift
   = 1 + PRIGROUP, sub shift = 4 — for every legal grouping): */
_Static_assert((1u + MNVIC_PRIGROUP_16_0) == 4u, "16/0 encoding: group shift must be 4");
_Static_assert((1u + MNVIC_PRIGROUP_8_2)  == 5u, "8/2  encoding: group shift must be 5");
_Static_assert((1u + MNVIC_PRIGROUP_4_4)  == 6u, "4/4  encoding: group shift must be 6");
_Static_assert((1u + MNVIC_PRIGROUP_2_8)  == 7u, "2/8  encoding: group shift must be 7");
_Static_assert((1u + MNVIC_PRIGROUP_1_16) == 8u, "1/16 encoding: group field is empty");

/* =====================================================================
 *  Module state
 * ===================================================================== */

/* Set by Init — gates SetPriority only (the ONE function whose
   correctness depends on AIRCR.PRIGROUP actually being programmed:
   before Init, the hardware grouping is the reset default, which
   may not match the config's encoding split). Enable/pending/
   trigger functions are deliberately NOT gated: none of them depend
   on the grouping. */
static u8 Global_u8IsInitialized = 0;

/* =====================================================================
 *  IRQ validity mask — derived from the startup vector table
 *
 *  Bit N set = position N is a real F401 interrupt. The 29 holes
 *  (family slots for peripherals the F401 lacks) are masked off, so
 *  a garbage enum value that passes the range check and happens to
 *  land on a hole is still rejected. Same pattern as MGPIO's
 *  per-package pin masks.
 *      word 0 : positions  0..31   (holes 19..22)
 *      word 1 : positions 32..63   (holes 39, 43..46, 48, 52..55, 61..63)
 *      word 2 : positions 64..84   (holes 64..66, 74..80, 82..83)
 * ===================================================================== */
static const u32 MNVIC_ValidIrqMask[3] =
{
    0xFF87FFFFu,    /* 0..31   */
    0x1F0E877Fu,    /* 32..63  */
    0x001203F8u     /* 64..84  */
};

/* =====================================================================
 *  Private helper — validate an IRQ position
 * ===================================================================== */
static MNVIC_ReturnType_t MNVIC_privateValidateIrq(MNVIC_IRQ_t Copy_Irq)
{
    /* (u32) cast BEFORE the comparison: C enums are not type-safe and
       a negative int cast in from garbage passes a signed "> MAX"
       test — but becomes huge in u32 arithmetic and fails here. */
    if ((u32)Copy_Irq > (u32)MNVIC_SPI4)
    {
        return MNVIC_INVALID_IRQ;
    }
    if ((MNVIC_ValidIrqMask[(u32)Copy_Irq >> 5u]
         & (1uL << ((u32)Copy_Irq & 0x1Fu))) == 0u)
    {
        return MNVIC_INVALID_IRQ;   /* family-layout hole — no such IRQ */
    }
    return MNVIC_OK;
}

/* =====================================================================
 *  MNVIC_Init
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_Init(void)
{
    u32 Local_u32Aircr;

    /* ---- Program the priority grouping (AIRCR.PRIGROUP) ----
     *
     * AIRCR is WRITE-KEYED: bits 31:16 must carry 0x05FA on every
     * write or the whole write is SILENTLY ignored. The trap: on
     * READ those bits return VECTKEYSTAT = 0xFA05 — NOT the write
     * key. A naive read-modify-write would write 0xFA05 back, the
     * write would be silently dropped, and the grouping would never
     * change. The mask-then-OR sequence is the correct pattern:
     *   1. read AIRCR (read-key 0xFA05 lands in [31:16])
     *   2. keep only the low 16 bits — drops the read-key, preserves
     *      ENDIANNESS and the reset-control bits (all read 0 on a
     *      healthy part, so nothing dangerous is ever written back)
     *   3. OR in the WRITE key 0x05FA
     *   4. rewrite PRIGROUP with the compile-time config value
     */
    Local_u32Aircr = MNVIC_SCB_AIRCR;                       /* read (0xFA05 key)  */
    Local_u32Aircr &= MNVIC_AIRCR_LOW16_MASK;               /* drop read-key      */
    Local_u32Aircr |= (MNVIC_AIRCR_VECTKEY_WRITE << MNVIC_AIRCR_VECTKEY_POS);
    Local_u32Aircr &= ~MNVIC_AIRCR_PRIGROUP_MASK;           /* clear grouping     */
    Local_u32Aircr |= ((u32)MNVIC_PRIORITY_GROUPING << MNVIC_AIRCR_PRIGROUP_POS);
    MNVIC_SCB_AIRCR = Local_u32Aircr;                       /* single keyed store */

    /* Read-back verification: PRIGROUP must now read back as the
       config value. A write with a wrong key (the silent-ignore
       failure class) leaves the old value — which becomes a loud
       MNVIC_INIT_FAILED instead of a silently wrong grouping. */
    if (((MNVIC_SCB_AIRCR & MNVIC_AIRCR_PRIGROUP_MASK) >> MNVIC_AIRCR_PRIGROUP_POS)
        != (u32)MNVIC_PRIORITY_GROUPING)
    {
        return MNVIC_INIT_FAILED;
    }

    Global_u8IsInitialized = 1u;
    return MNVIC_OK;
    /* Note: this write is functionally idempotent — with the default
       16/0 config, reset PRIGROUP (0..2 all mean 16/0 on a 4-bit
       part) and the programmed value (3) give the identical split.
       Init exists to make the register EXPLICITLY match the model,
       and to verify the write actually landed. */
}

/* =====================================================================
 *  Enable / disable — ISER / ICER
 *
 *  Write-1-to-set / write-1-to-clear, one-hot direct store. A single
 *  transaction; bits written 0 are untouched. Note the semantics on
 *  disable: the pending state is NOT cleared — a disabled-but-pending
 *  IRQ fires on re-enable (ClearPending first if that is unwanted).
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_EnableInterrupt(MNVIC_IRQ_t Copy_Irq)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    MNVIC->NVIC_ISER[MNVIC_IRQ_WORD(Copy_Irq)] = MNVIC_IRQ_BIT(Copy_Irq);
    return MNVIC_OK;
}

MNVIC_ReturnType_t MNVIC_DisableInterrupt(MNVIC_IRQ_t Copy_Irq)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    MNVIC->NVIC_ICER[MNVIC_IRQ_WORD(Copy_Irq)] = MNVIC_IRQ_BIT(Copy_Irq);
    return MNVIC_OK;
}

/* =====================================================================
 *  Pending state — ISPR (set) / ICPR (clear)
 *
 *  Software-set pending is indistinguishable to the NVIC from the
 *  peripheral asserting its line: the handler runs once enabled.
 *  ClearPending discards a latched event — the right call before
 *  enabling an IRQ whose peripheral flag may already be set (the
 *  MEXTI "stale flag fires immediately on enable" trap).
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_SetPending(MNVIC_IRQ_t Copy_Irq)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    MNVIC->NVIC_ISPR[MNVIC_IRQ_WORD(Copy_Irq)] = MNVIC_IRQ_BIT(Copy_Irq);
    return MNVIC_OK;
}

MNVIC_ReturnType_t MNVIC_ClearPending(MNVIC_IRQ_t Copy_Irq)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    MNVIC->NVIC_ICPR[MNVIC_IRQ_WORD(Copy_Irq)] = MNVIC_IRQ_BIT(Copy_Irq);
    return MNVIC_OK;
}

MNVIC_ReturnType_t MNVIC_GetPending(MNVIC_IRQ_t Copy_Irq, u8 *Copy_pu8Pending)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    if (Copy_pu8Pending == (u8 *)0)
    {
        return MNVIC_INVALID_PARAM;
    }
    *Copy_pu8Pending = (u8)((MNVIC->NVIC_ISPR[MNVIC_IRQ_WORD(Copy_Irq)]
                             >> ((u32)Copy_Irq & 0x1Fu)) & 1u);
    return MNVIC_OK;
}

/* =====================================================================
 *  Active state — IABR (read-only)
 *
 *  Hardware sets the bit on exception ENTRY and clears it on EXIT:
 *  reading it from inside the handler itself returns 1 (the classic
 *  self-check), reading from another context tests "is that ISR
 *  running right now" (re-entrancy diagnostics).
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_GetActive(MNVIC_IRQ_t Copy_Irq, u8 *Copy_pu8Active)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    if (Copy_pu8Active == (u8 *)0)
    {
        return MNVIC_INVALID_PARAM;
    }
    *Copy_pu8Active = (u8)((MNVIC->NVIC_IABR[MNVIC_IRQ_WORD(Copy_Irq)]
                            >> ((u32)Copy_Irq & 0x1Fu)) & 1u);
    return MNVIC_OK;
}

/* =====================================================================
 *  MNVIC_SetPriority
 *
 *  The REV 2 collapse: with grouping fixed at compile time, the whole
 *  five-case switch becomes ONE formula —
 *      byte = (group << (1 + PRIGROUP)) | (sub << 4)
 *  because the group field's start bit is 8 - GROUP_BITS = 1 + the
 *  PRIGROUP encoding itself. Verified by _Static_asserts above.
 *
 *  The direct full-byte store replaces the old RMW: the &= 0xF only
 *  preserved unimplemented bits (which read as zero), so the RMW
 *  accomplished nothing. A single byte store is atomic — safe even
 *  if called from an ISR.
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_SetPriority(MNVIC_IRQ_t Copy_Irq,
                                     u8 Copy_u8GroupPriority, u8 Copy_u8SubPriority)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }

    /* Gated: before Init, AIRCR still holds the reset grouping, which
       may not match the config split — the byte would encode against
       the WRONG field geometry. Nothing is written. */
    if (Global_u8IsInitialized == 0u)
    {
        return MNVIC_NOT_INITIALIZED;
    }

    /* Range checks — the silent-truncation fix. The old code let an
       out-of-range group wrap into the neighboring field (or into
       nothing), which could land the IRQ at priority 0 — the MOST
       urgent — with no diagnostic. Now: loud rejection. */
    if (Copy_u8GroupPriority > MNVIC_GROUP_PRIORITY_MAX)
    {
        return MNVIC_INVALID_PARAM;
    }
    if (Copy_u8SubPriority > MNVIC_SUB_PRIORITY_MAX)
    {
        return MNVIC_INVALID_PARAM;
    }

    MNVIC->NVIC_IPR[Copy_Irq] = (u8)(((u32)Copy_u8GroupPriority << MNVIC_GROUP_PRIORITY_SHIFT)
                                     | ((u32)Copy_u8SubPriority   << MNVIC_SUB_PRIORITY_SHIFT));
    return MNVIC_OK;
}

/* =====================================================================
 *  MNVIC_GetPriority — decode through the same compile-time split
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_GetPriority(MNVIC_IRQ_t Copy_Irq,
                                     u8 *Copy_pu8GroupPriority, u8 *Copy_pu8SubPriority)
{
    u8 Local_u8Byte;

    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    if ((Copy_pu8GroupPriority == (u8 *)0) || (Copy_pu8SubPriority == (u8 *)0))
    {
        return MNVIC_INVALID_PARAM;
    }

    Local_u8Byte = MNVIC->NVIC_IPR[Copy_Irq];

    *Copy_pu8GroupPriority = (u8)((Local_u8Byte >> MNVIC_GROUP_PRIORITY_SHIFT)
                                  & MNVIC_GROUP_PRIORITY_MAX);
    *Copy_pu8SubPriority   = (u8)((Local_u8Byte >> MNVIC_SUB_PRIORITY_SHIFT)
                                  & MNVIC_SUB_PRIORITY_MAX);
    return MNVIC_OK;
}

/* =====================================================================
 *  MNVIC_TriggerIRQ — STIR
 *
 *  Writing the IRQ position to STIR sets that IRQ pending — the
 *  software equivalent of the peripheral asserting its line. This is
 *  how interrupt handlers get tested with ZERO hardware: enable
 *  MNVIC_EXTI0 with a test handler, call TriggerIRQ(MNVIC_EXTI0),
 *  and the handler executes without any button existing. (The
 *  peripheral's own mask is bypassed entirely — NVIC enable +
 *  pending is sufficient for exception activation.)
 *
 *  STIR is privileged-writable by default (unprivileged access
 *  faults unless SCB CCR.USERSETMPEND is set — this driver never
 *  touches CCR; bare-metal super-loop code runs privileged).
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_TriggerIRQ(MNVIC_IRQ_t Copy_Irq)
{
    if (MNVIC_privateValidateIrq(Copy_Irq) != MNVIC_OK)
    {
        return MNVIC_INVALID_IRQ;
    }
    MNVIC->NVIC_STIR = (u32)Copy_Irq;
    return MNVIC_OK;
}





/* =====================================================================
 *  MNVIC_SetVectorTableBase  (Rev 3, FOTA)
 * ===================================================================== */
MNVIC_ReturnType_t MNVIC_SetVectorTableBase(u32 Copy_u32Address)
{
    /* VTOR implements bits [31:7]: 128-byte alignment minimum. Reject
       misalignment BEFORE the write rather than letting hardware drop
       the low bits silently. */
    if ((Copy_u32Address & MNVIC_VTOR_ALIGNMENT_MASK) != 0u)
    {
        return MNVIC_INVALID_PARAM;
    }

    MNVIC_SCB_VTOR = Copy_u32Address;

    /* Belt-and-braces read-back (VTOR carries no write key; bits
       [6:0] read 0 and the argument's are 0). */
    if (MNVIC_SCB_VTOR != Copy_u32Address)
    {
        return MNVIC_INIT_FAILED;
    }
    return MNVIC_OK;
}

/* =====================================================================
 *  MNVIC_SystemReset  (Rev 3, FOTA) — NEVER RETURNS
 * ===================================================================== */
void MNVIC_SystemReset(void)
{
    u32 Local_u32Aircr;

    /* The MNVIC_Init keyed-write pattern, verbatim: read (key reads
       0xFA05), keep ONLY the low 16 bits (drops the read-key, keeps
       ENDIANNESS; VECTRESET/VECTCLRACTIVE/SYSRESETREQ read 0 on a
       healthy part), OR in the WRITE key, set SYSRESETREQ, store. */
    Local_u32Aircr = MNVIC_SCB_AIRCR;
    Local_u32Aircr &= MNVIC_AIRCR_LOW16_MASK;
    Local_u32Aircr |= (MNVIC_AIRCR_VECTKEY_WRITE << MNVIC_AIRCR_VECTKEY_POS);
    Local_u32Aircr |= (1uL << MNVIC_AIRCR_SYSRESETREQ);
    MNVIC_SCB_AIRCR = Local_u32Aircr;

    /* Reset arrives a few cycles later. DSB guarantees the store
       completed; then wait forever — if the reset is somehow blocked
       (debugger holding reset), this loop is where we stay. */
    asm("DSB");
    while (1)
    {
        asm("WFI");
    }
}
