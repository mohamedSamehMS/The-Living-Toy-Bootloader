/*
 * MUSART_program.c
 *
 *  Implementation of the MUSART driver.
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 19: USART
 *
 *  DESIGN PRINCIPLES (documented, deliberate):
 *
 *  1. STORM-SAFE BY CONSTRUCTION. Every interrupt this driver can arm
 *     (TXE/RXNE/TC/IDLE) has a guaranteed clear path inside its ISR;
 *     everything else is rejected at enable (MUSART_NOT_SUPPORTED).
 *     The Rev 1 API could arm 7 unserviced sources — each one an
 *     immediate IRQ storm at priority 0, freezing the whole system.
 *     IDLE was the worst: it fires on a QUIET line, and nothing in
 *     the Rev 1 ISR ever executed the SR-read/DR-read sequence that
 *     clears it.
 *
 *  2. ONE SHARED ISR BODY. USART1/2/6 handlers are three one-line
 *     wrappers around one table-indexed function. The Rev 1
 *     USART2/USART6 RX handlers had silently diverged from USART1
 *     (missing the RXNEIE disable) — copy-paste divergence between
 *     instance blocks is now structurally impossible.
 *
 *  3. CLOCKS FROM THE PUBLISHED MODEL. BRR math consumes
 *     MUSART_PCLK1/2_HZ derived from MRCC — no hand-copied 16 MHz.
 *
 *  4. INTEGER BRR MATH. mantissa/fraction from shifts and masks —
 *     bit-exact with the Rev 1 float version (truncation behavior
 *     provably identical), deterministic, no FPU/soft-float pulled
 *     into a project whose roadmap declared the FPU low-relevance.
 *     Runtime-validated: baud 0 and 12-bit mantissa overflow are
 *     returned as errors instead of dividing by zero / silently
 *     programming a wrong baud.
 *
 *  5. TABLE-DRIVEN. Registers, NVIC id, config, and callbacks for all
 *     three instances live in ONE static table — the pattern that
 *     killed MGPIO's per-function port switches.
 *
 *  Revision history:
 *    Rev 1 — original
 *    Rev 2 — rewrite (see principles above; also: typed MNVIC,
 *            MSYSTICK_GetElapsed_ms rename, UE checks on blocking ops,
 *            NVIC-enable-after-validation, NULL-means-unregister)
 *    Rev 3 — SR snapshot at handler entry: RXNE/IDLE detection made
 *            atomic against the IDLE clear-sequence — the combined
 *            byte+idle case no longer loses the frame-complete event;
 *            TXE/TC keep live reads (post-callback state is the true
 *            state)
 *    Rev 4 — UE checks extended to the arming functions
 *            (SendByteNonBlocking / ReceiveEnable / InterruptEnable);
 *            InterruptDisable stays ungated by design. IDLE-alone
 *            data-loss footgun documented at MUSART_INT_IDLE and in
 *            the handler's branch 2.
 *
 *  Created on: Aug 23, 2026
 *      Author: dell
 */

#include <MCAL/USART/MUSART_config.h>
#include <MCAL/USART/MUSART_interface.h>
#include <MCAL/USART/MUSART_private.h>
#include <MCAL/SYSTICK/MSYSTICK_interface.h>
#include "MNVIC_interface.h"
#include "BIT_MATH.h"
#include "STD_TYPES.h"

/* ========================================================================= *
 *                        PRIVATE TYPES & TABLES                             *
 * ========================================================================= */

/** Per-instance compile-time configuration, built from config.h */
typedef struct
{
    u8                     Enabled;
    u32                    BaudRate;
    MUSART_WordLength_t    WordLength;
    MUSART_Parity_t        Parity;
    MUSART_StopBits_t      StopBits;
    MUSART_USARTMode_t     Mode;
    MUSART_Oversampling_t  Oversampling;
    MUSART_SyncMode_t      SyncMode;
    u8                     DmaTx;
    u8                     DmaRx;
    u8                     Smartcard;
    u32                    ClockHz;      /* APB clock of THIS instance */
} MUSART_InstanceConfig_t;

/** One row per USART instance: immutable wiring + mutable callbacks */
typedef struct
{
    /* ---- immutable after table initialization ---- */
    volatile MUSART_Registers_t         *Reg;
    MNVIC_IRQ_t                          Irq;
    const MUSART_InstanceConfig_t       *Cfg;
    /* ---- mutable: registered callbacks (NULL = none) ---- */
    MUSART_TxCallback_t                  TxCallback;        /* TXE   */
    MUSART_TxCompleteCallback_t          TxCompleteCallback;/* TC    */
    MUSART_RxCallback_t                  RxCallback;        /* RXNE  */
    MUSART_IdleCallback_t                IdleCallback;      /* IDLE  */
    volatile u32                         FeDroppedCount;    /* Rev 5: FE/NF bytes dropped by the ISR */
} MUSART_Instance_t;

/* Instance count — indexes into the table (see MUSART_USART_t) */
#define MUSART_INSTANCE_COUNT   3u

/* ---- the three per-instance configs (from MUSART_config.h) ---- */
static const MUSART_InstanceConfig_t MUSART1_Cfg =
{
    MUSART1_ENABLED, MUSART1_BAUDRATE, MUSART1_WORDLENGTH, MUSART1_PARITY,
    MUSART1_STOPBITS, MUSART1_MODE, MUSART1_OVERSAMPLING, MUSART1_SYNC_MODE,
    MUSART1_DMA_TX, MUSART1_DMA_RX, MUSART1_SMARTCARD_MODE,
    MUSART_PCLK2_HZ                 /* USART1 sits on APB2 */
};
static const MUSART_InstanceConfig_t MUSART2_Cfg =
{
    MUSART2_ENABLED, MUSART2_BAUDRATE, MUSART2_WORDLENGTH, MUSART2_PARITY,
    MUSART2_STOPBITS, MUSART2_MODE, MUSART2_OVERSAMPLING, MUSART2_SYNC_MODE,
    MUSART2_DMA_TX, MUSART2_DMA_RX, MUSART2_SMARTCARD_MODE,
    MUSART_PCLK1_HZ                 /* USART2 sits on APB1 */
};
static const MUSART_InstanceConfig_t MUSART6_Cfg =
{
    MUSART6_ENABLED, MUSART6_BAUDRATE, MUSART6_WORDLENGTH, MUSART6_PARITY,
    MUSART6_STOPBITS, MUSART6_MODE, MUSART6_OVERSAMPLING, MUSART6_SYNC_MODE,
    MUSART6_DMA_TX, MUSART6_DMA_RX, MUSART6_SMARTCARD_MODE,
    MUSART_PCLK2_HZ                 /* USART6 sits on APB2 */
};

/* ---- THE instance table (design principle 5) ---- */
static MUSART_Instance_t MUSART_Instances[MUSART_INSTANCE_COUNT] =
{
    /*        Reg        IRQ              Config        callbacks (all unregistered) */
    [MUSART_USART1] = { MUSART1, MNVIC_USART1, &MUSART1_Cfg, NULL, NULL, NULL, NULL, 0u },
    [MUSART_USART2] = { MUSART2, MNVIC_USART2, &MUSART2_Cfg, NULL, NULL, NULL, NULL, 0u },
    [MUSART_USART6] = { MUSART6, MNVIC_USART6, &MUSART6_Cfg, NULL, NULL, NULL, NULL, 0u }
};

/* ========================================================================= *
 *                        PRIVATE FUNCTION PROTOTYPES                        *
 * ========================================================================= */
static MUSART_ReturnType_t MUSART_privateValidateInstance(MUSART_USART_t Copy_USART);
static u8                   MUSART_privateWaitForFlagTimeout(volatile MUSART_Registers_t *Copy_USART, u8 Copy_u8Flag);
static void                 MUSART_privateHandler(u8 Copy_u8Index);

/* ========================================================================= *
 *                        INITIALIZATION & CONTROL                           *
 * ========================================================================= */

MUSART_ReturnType_t MUSART_Init(MUSART_USART_t Copy_USART)
{
    volatile MUSART_Registers_t   *Local_Reg;
    const MUSART_InstanceConfig_t *Local_Cfg;
    u32  Local_u32X;         /* 16*USARTDIV (or 8* in OVER8) — integer */
    u16  Local_u16Mantissa;
    u8   Local_u8Fraction;
    u8   Local_u8Over8;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    Local_Reg = MUSART_Instances[Copy_USART].Reg;
    Local_Cfg = MUSART_Instances[Copy_USART].Cfg;

    /* 1. Disable before configuring — RM0368: CR1/CR2/CR3/BRR writes
          are only guaranteed with UE=0. (Rev 1 did this; kept.) */
    CLR_BIT(Local_Reg->CR1, MUSART_CR1_UE);

    /* 2. CR2: stop bits (enum values ARE the encodings) + sync mode */
    Local_Reg->CR2 &= ~MUSART_CR2_STOP_MASK;
    Local_Reg->CR2 |= ((u32)Local_Cfg->StopBits << MUSART_CR2_STOP);

    if (Local_Cfg->SyncMode == MUSART_SYNC)
    {
        SET_BIT(Local_Reg->CR2, MUSART_CR2_CLKEN);
    }
    else
    {
        CLR_BIT(Local_Reg->CR2, MUSART_CR2_CLKEN);
    }

    /* 3. CR3: cleared wholesale, then DMA request lines / smartcard.
          DMA flags default to 0 until the Phase-1 DMA driver exists. */
    Local_Reg->CR3 = 0;
    if (Local_Cfg->DmaTx) { SET_BIT(Local_Reg->CR3, MUSART_CR3_DMAT); }
    if (Local_Cfg->DmaRx) { SET_BIT(Local_Reg->CR3, MUSART_CR3_DMAR); }
    if (Local_Cfg->Smartcard)
    {
        SET_BIT(Local_Reg->CR3, MUSART_CR3_SCEN);   /* STUB — see config guide */
    }

    /* 4. CR1: oversampling (smartcard forces 16 — hardware would
          anyway, made explicit), word length, parity, direction */
    Local_u8Over8 = (Local_Cfg->Oversampling == MUSART_OVERSAMPLING_8) ? 1u : 0u;
    if (Local_Cfg->Smartcard) { Local_u8Over8 = 0u; }

    if (Local_u8Over8) { SET_BIT(Local_Reg->CR1, MUSART_CR1_OVER8); }
    else               { CLR_BIT(Local_Reg->CR1, MUSART_CR1_OVER8); }

    if (Local_Cfg->WordLength == MUSART_WORDLEN_9BITS) { SET_BIT(Local_Reg->CR1, MUSART_CR1_M); }
    else                                               { CLR_BIT(Local_Reg->CR1, MUSART_CR1_M); }

    if (Local_Cfg->Parity != MUSART_PARITY_NONE)
    {
        SET_BIT(Local_Reg->CR1, MUSART_CR1_PCE);
        if (Local_Cfg->Parity == MUSART_PARITY_ODD) { SET_BIT(Local_Reg->CR1, MUSART_CR1_PS); }
        else                                        { CLR_BIT(Local_Reg->CR1, MUSART_CR1_PS); }
    }
    else
    {
        CLR_BIT(Local_Reg->CR1, MUSART_CR1_PCE);
    }

    if ((Local_Cfg->Mode == MUSART_MODE_TX_ONLY) || (Local_Cfg->Mode == MUSART_MODE_TX_RX))
    {
        SET_BIT(Local_Reg->CR1, MUSART_CR1_TE);
    }
    if ((Local_Cfg->Mode == MUSART_MODE_RX_ONLY) || (Local_Cfg->Mode == MUSART_MODE_TX_RX))
    {
        SET_BIT(Local_Reg->CR1, MUSART_CR1_RE);
    }

    /* 5. BRR — INTEGER math (design principle 4).
          USARTDIV = fCK / ((8 or 16) * baud);  X = fCK / baud gives
          16*USARTDIV (or 8* in OVER8) directly:
            OVER8=0: mantissa = X >> 4, fraction = X & 0xF  (4 bits)
            OVER8=1: mantissa = X >> 3, fraction = X & 0x7  (3 bits —
                     BRR[3] stays 0, the detail everyone misses)
          Truncation is bit-exact with the old float version. */
    if (Local_Cfg->BaudRate == 0u)
    {
        return MUSART_INVALID_CONFIG;   /* guarded at compile time too — defense in depth */
    }
    Local_u32X = Local_Cfg->ClockHz / Local_Cfg->BaudRate;

    if (Local_u8Over8)
    {
        Local_u16Mantissa = (u16)(Local_u32X >> 3);
        Local_u8Fraction  = (u8)(Local_u32X & 0x07u);
    }
    else
    {
        Local_u16Mantissa = (u16)(Local_u32X >> 4);
        Local_u8Fraction  = (u8)(Local_u32X & 0x0Fu);
    }

    /* Mantissa must fit its 12-bit field, else the register masks it
       away and the link silently runs the WRONG baud. Loud instead. */
    if (Local_u16Mantissa > 0x0FFFu)
    {
        return MUSART_INVALID_CONFIG;   /* baud too low for this clock */
    }

    Local_Reg->BRR = (((u32)Local_u16Mantissa & 0x0FFFu) << 4) | Local_u8Fraction;

    /* 6. Enable if configured (all functions gate on UE — see Rev 4) */
    if (Local_Cfg->Enabled) { SET_BIT(Local_Reg->CR1, MUSART_CR1_UE); }
    else                    { CLR_BIT(Local_Reg->CR1, MUSART_CR1_UE); }

    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_Enable(MUSART_USART_t Copy_USART)
{
    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    SET_BIT(MUSART_Instances[Copy_USART].Reg->CR1, MUSART_CR1_UE);
    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_Disable(MUSART_USART_t Copy_USART)
{
    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    CLR_BIT(MUSART_Instances[Copy_USART].Reg->CR1, MUSART_CR1_UE);
    return MUSART_OK;
}

/* ========================================================================= *
 *                        INTERRUPT MANAGEMENT                               *
 * ========================================================================= */

MUSART_ReturnType_t MUSART_InterruptEnable(MUSART_USART_t Copy_USART, MUSART_Interrupt_t Copy_Interrupt)
{
    volatile MUSART_Registers_t *Local_Reg;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }

    /* Enum garbage check BEFORE the switch (enums are not type-safe) */
    if ((u32)Copy_Interrupt > (u32)MUSART_INT_ERR)
    {
        return MUSART_INVALID_CONFIG;
    }

    Local_Reg = MUSART_Instances[Copy_USART].Reg;

    /* UE check (Rev 4): arming sources on a disabled USART is a
       configuration-order error (worst corner: TXE on a never-written
       DR fires the callback immediately — true but meaningless).
       InterruptDisable deliberately stays UNGATED: disarming must
       always succeed, like MNVIC_ClearPending. */
    if (GET_BIT(Local_Reg->CR1, MUSART_CR1_UE) == 0)
    {
        return MUSART_NOT_ENABLED;
    }

    /* ---- Validate AND set the peripheral bit FIRST; the NVIC line is
            enabled only afterwards (Rev 1 order was reversed — a bad
            argument still armed the IRQ line). ---- */
    switch (Copy_Interrupt)
    {
        case MUSART_INT_TXE:  SET_BIT(Local_Reg->CR1, MUSART_CR1_TXEIE);  break;
        case MUSART_INT_RXNE: SET_BIT(Local_Reg->CR1, MUSART_CR1_RXNEIE); break;
        case MUSART_INT_TC:   SET_BIT(Local_Reg->CR1, MUSART_CR1_TCIE);   break;
        case MUSART_INT_IDLE: SET_BIT(Local_Reg->CR1, MUSART_CR1_IDLEIE); break;

        /* THE STORM GUARDS: these sources have NO ISR service. Arming
           one would pend the IRQ forever (flag never clears -> the
           handler returns -> flag re-pends -> ...) at priority 0.
           IDLE through this exact API was the Rev 1 system-freezer. */
        case MUSART_INT_PE:
        case MUSART_INT_LBD:
        case MUSART_INT_CTS:
        case MUSART_INT_ERR:
        default:
            return MUSART_NOT_SUPPORTED;
    }

    /* ---- NVIC enable (typed enum; status propagated) ----
            Re-enabling an enabled line is harmless: ISER is
            write-1-to-set. Priority: the APPLICATION sets it via
            MNVIC_SetPriority before first use — the driver never
            touches priorities (one owner per concern). */
    {
        MNVIC_ReturnType_t Local_EnumStatus = MNVIC_EnableInterrupt(MUSART_Instances[Copy_USART].Irq);
        if (Local_EnumStatus != MNVIC_OK)
        {
            return Local_EnumStatus;   /* impossible with our enum values — belt and braces */
        }
    }

    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_InterruptDisable(MUSART_USART_t Copy_USART, MUSART_Interrupt_t Copy_Interrupt)
{
    volatile MUSART_Registers_t *Local_Reg;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    if ((u32)Copy_Interrupt > (u32)MUSART_INT_ERR)
    {
        return MUSART_INVALID_CONFIG;
    }

    Local_Reg = MUSART_Instances[Copy_USART].Reg;

    /* Deliberately NOT UE-gated (contrast the Rev 4 arming gate in
       InterruptEnable): disarming must always succeed, whatever state
       the peripheral is in. */
    /* Disable accepts the unserviced ids too (idempotent no-op bits):
       disarming can never create a storm, so no rejection here. */
    switch (Copy_Interrupt)
    {
        case MUSART_INT_TXE:  CLR_BIT(Local_Reg->CR1, MUSART_CR1_TXEIE);  break;
        case MUSART_INT_RXNE: CLR_BIT(Local_Reg->CR1, MUSART_CR1_RXNEIE); break;
        case MUSART_INT_TC:   CLR_BIT(Local_Reg->CR1, MUSART_CR1_TCIE);   break;
        case MUSART_INT_IDLE: CLR_BIT(Local_Reg->CR1, MUSART_CR1_IDLEIE); break;
        case MUSART_INT_PE:   CLR_BIT(Local_Reg->CR1, MUSART_CR1_PEIE);   break;
        case MUSART_INT_LBD:  CLR_BIT(Local_Reg->CR2, MUSART_CR2_LBDIE);  break;
        case MUSART_INT_CTS:  CLR_BIT(Local_Reg->CR3, MUSART_CR3_CTSIE);  break;
        case MUSART_INT_ERR:  CLR_BIT(Local_Reg->CR3, MUSART_CR3_EIE);    break;
        default: return MUSART_INVALID_CONFIG;
    }
    return MUSART_OK;
}

/* ========================================================================= *
 *                        BLOCKING FUNCTIONS                                 *
 * ========================================================================= */

MUSART_ReturnType_t MUSART_SendByteBlocking(MUSART_USART_t Copy_USART, u8 Copy_u8Data)
{
    volatile MUSART_Registers_t *Local_Reg;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    Local_Reg = MUSART_Instances[Copy_USART].Reg;

    /* UE check — the MSPI SPE lesson: a disabled peripheral's flags
       never assert, so the timeout would burn a full second per call.
       Refuse immediately and loudly instead. */
    if (GET_BIT(Local_Reg->CR1, MUSART_CR1_UE) == 0)
    {
        return MUSART_NOT_ENABLED;
    }

    if (MUSART_privateWaitForFlagTimeout(Local_Reg, MUSART_SR_TXE) == 0)
    {
        return MUSART_TIMEOUT;
    }

    Local_Reg->DR = Copy_u8Data;   /* TXE reset value is 1, so first
                                      call after enable proceeds at once */
    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_ReceiveByteBlocking(MUSART_USART_t Copy_USART, u8 *Copy_pu8Data)
{
    volatile MUSART_Registers_t *Local_Reg;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    if (Copy_pu8Data == (u8 *)0)
    {
        return MUSART_NULL_POINTER;
    }
    Local_Reg = MUSART_Instances[Copy_USART].Reg;

    if (GET_BIT(Local_Reg->CR1, MUSART_CR1_UE) == 0)
    {
        return MUSART_NOT_ENABLED;
    }

    if (MUSART_privateWaitForFlagTimeout(Local_Reg, MUSART_SR_RXNE) == 0)
    {
        return MUSART_TIMEOUT;
    }

    /* DR read clears RXNE — and (with the SR reads above) any latched
       ORE, recovering the link after an overrun without hanging. The
       byte delivered is the OLDER one; the overrun byte was lost by
       hardware before software ever saw it. */
    *Copy_pu8Data = (u8)Local_Reg->DR;
    return MUSART_OK;
}

/* ========================================================================= *
 *                  NON-BLOCKING / INTERRUPT-DRIVEN FUNCTIONS                *
 * ========================================================================= */

MUSART_ReturnType_t MUSART_SendByteNonBlocking(MUSART_USART_t Copy_USART, u8 Copy_u8Data)
{
    volatile MUSART_Registers_t *Local_Reg;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    Local_Reg = MUSART_Instances[Copy_USART].Reg;

    /* UE check (Rev 4): on a disabled USART this used to return OK —
       but the DR write clears TXE and only the TDR->shift-register
       transfer (which needs UE) can set it again. The byte sits in TDR
       as a DEFERRED action: if the app enables the USART later, the
       stale byte transmits and the callback fires for a write made
       while disabled. Refuse loudly, matching SendByteBlocking.
       Must run BEFORE the TXE/BUSY check: TXE reads 1 at reset, so
       without this gate the busy check passes and the harmful DR
       write happens. */
    if (GET_BIT(Local_Reg->CR1, MUSART_CR1_UE) == 0)
    {
        return MUSART_NOT_ENABLED;
    }

    /* DR must be empty RIGHT NOW — a non-blocking API must not wait.
       BUSY tells the caller to retry or use the callback chain. */
    if (GET_BIT(Local_Reg->SR, MUSART_SR_TXE) == 0)
    {
        return MUSART_BUSY;
    }

    Local_Reg->DR = Copy_u8Data;

    /* Arm TXE (peripheral bit first, then NVIC — same order contract
       as InterruptEnable). The ISR disables TXEIE BEFORE the callback:
       one-shot per byte, storm-proof even if the callback writes
       nothing. Stream pattern: the TxCallback writes the next byte
       through this same function, re-arming for the following byte. */
    SET_BIT(Local_Reg->CR1, MUSART_CR1_TXEIE);
    {
        MNVIC_ReturnType_t Local_EnumStatus = MNVIC_EnableInterrupt(MUSART_Instances[Copy_USART].Irq);
        if (Local_EnumStatus != MNVIC_OK)
        {
            return Local_EnumStatus;
        }
    }
    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_ReceiveEnable(MUSART_USART_t Copy_USART)
{
    volatile MUSART_Registers_t *Local_Reg;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    Local_Reg = MUSART_Instances[Copy_USART].Reg;

    /* UE check (Rev 4): the receiver can never set RXNE/IDLE while
       UE=0 — armed-but-inert is exactly the silent "nothing works, no
       error anywhere" trap. Refuse loudly. */
    if (GET_BIT(Local_Reg->CR1, MUSART_CR1_UE) == 0)
    {
        return MUSART_NOT_ENABLED;
    }

    /* Arm the framing pair: RXNE (every byte) + IDLE (frame end).
       If a byte is already sitting in DR, the IRQ fires immediately
       with it — arming never discards buffered data. */
    SET_BIT(Local_Reg->CR1, MUSART_CR1_RXNEIE);
    SET_BIT(Local_Reg->CR1, MUSART_CR1_IDLEIE);

    {
        MNVIC_ReturnType_t Local_EnumStatus = MNVIC_EnableInterrupt(MUSART_Instances[Copy_USART].Irq);
        if (Local_EnumStatus != MNVIC_OK)
        {
            return Local_EnumStatus;
        }
    }
    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_ReceiveDisable(MUSART_USART_t Copy_USART)
{
    volatile MUSART_Registers_t *Local_Reg;

    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    Local_Reg = MUSART_Instances[Copy_USART].Reg;

    /* Deliberately NOT UE-gated (contrast the Rev 4 arming gate in
       ReceiveEnable): disarming must always succeed. */
    CLR_BIT(Local_Reg->CR1, MUSART_CR1_RXNEIE);
    CLR_BIT(Local_Reg->CR1, MUSART_CR1_IDLEIE);
    /* NVIC line intentionally left enabled — harmless, and ReceiveEnable
       is then a two-CR1-bit operation away. */
    return MUSART_OK;
}

/* ========================================================================= *
 *                        CALLBACK SETTERS                                    *
 * ========================================================================= */

/* NULL is a valid "unregister" value for every setter — one table row
   per instance means every instance gets identical semantics (the
   Rev 1 divergence class cannot recur). Aligned pointer stores are
   atomic; ISRs re-read them every event. */

MUSART_ReturnType_t MUSART_SetTxCallback(MUSART_USART_t Copy_USART, MUSART_TxCallback_t Copy_pfCallback)
{
    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    MUSART_Instances[Copy_USART].TxCallback = Copy_pfCallback;
    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_SetTxCompleteCallback(MUSART_USART_t Copy_USART, MUSART_TxCompleteCallback_t Copy_pfCallback)
{
    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    MUSART_Instances[Copy_USART].TxCompleteCallback = Copy_pfCallback;
    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_SetRxCallback(MUSART_USART_t Copy_USART, MUSART_RxCallback_t Copy_pfCallback)
{
    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    MUSART_Instances[Copy_USART].RxCallback = Copy_pfCallback;
    return MUSART_OK;
}

MUSART_ReturnType_t MUSART_SetIdleCallback(MUSART_USART_t Copy_USART, MUSART_IdleCallback_t Copy_pfCallback)
{
    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    MUSART_Instances[Copy_USART].IdleCallback = Copy_pfCallback;
    return MUSART_OK;
}

/* ========================================================================= *
 *                        INTERRUPT SERVICE ROUTINES                         *
 * ========================================================================= *
 * One shared body (design principle 2) — the three public handlers are
 * one-line wrappers. Everything below is the storm-safety map made
 * executable; read it together with the map comment in private.h.
 * ========================================================================= */

/* ---- The shared handler --------------------------------------------------
 * REV 3 HARDENING — the SR SNAPSHOT. SR is read exactly ONCE, at entry,
 * into Local_u32Sr; branches 1 and 2 detect their flags from that copy.
 *
 * WHY: IDLE is cleared by the read-SR + read-DR sequence. In the
 * combined case (a byte AND its follow-up idle both pending when the
 * handler runs — ISR latency beyond ~one character time), branch 1's
 * own SR read + DR read completes that sequence, and hardware silently
 * drops IDLE before branch 2's live check could see it — the
 * frame-complete event is lost. The snapshot captures IDLE=1 before any
 * branch can consume it: DETECTION becomes atomic, while the hardware
 * clear still happens through the normal sequences.
 *
 * Properties of the snapshot:
 *   - flags that set AFTER the snapshot are handled on the NEXT IRQ
 *     invocation (flag stays set, IRQ re-pends) — conservative, never
 *     lossy. RXNE=1 in the snapshot implies live RXNE=1 at branch 1:
 *     only a DR read clears RXNE, and none has happened yet.
 *   - TXE / TC (branches 3/4) deliberately keep LIVE reads — see the
 *     notes at those branches; do not "harmonize" them to the snapshot.
 * -------------------------------------------------------------------------- */
static void MUSART_privateHandler(u8 Copy_u8Index)
{
    volatile MUSART_Registers_t *Local_Reg  = MUSART_Instances[Copy_u8Index].Reg;
    MUSART_Instance_t           *Local_Inst = &MUSART_Instances[Copy_u8Index];

    /* ONE SR read at entry. This read is itself the "read SR" half of
       the IDLE/ORE clear sequences: if IDLE is set now, branch 1's DR
       read will complete its clear in hardware — and branch 2 still
       detects the event from this saved copy. That is the whole fix. */
    u32 Local_u32Sr = Local_Reg->SR;

    /* ============ 1. RXNE — continuous byte reception ============
     * Snapshot-detected (equivalent to live here — see header note).
     * The DR read delivers the byte, clears RXNE — and, in the
     * combined case, completes the IDLE clear whose detection branch 2
     * already banked in the snapshot. RXNE sets only on hardware byte
     * arrival, so leaving RXNEIE armed cannot storm.
     * Order note: RXNE before IDLE, so byte + idle both pending
     * delivers the byte BEFORE the frame-complete callback. */
    if ((GET_BIT(Local_u32Sr, MUSART_SR_RXNE) == 1) &&
        (GET_BIT(Local_Reg->CR1, MUSART_CR1_RXNEIE) == 1))
    {
        u8 Local_u8Data = (u8)Local_Reg->DR;   /* clears RXNE (and ORE) */

        /* REV 5b — ERROR-FLAG INSTRUMENTATION, POLICY-SEPARATED.
           FE/NF events are COUNTED always — the counter is the link-
           health instrument. DELIVERY follows two config knobs, both
           defaulting to DELIVER: bench runs 5-7 show this link sets FE
           on a substantial fraction of REAL bytes (leading theory: a
           few-percent baud mismatch on the adapter's TX — the stop bit
           is single-sampled and takes the worst drift, while the
           majority-voted data bits stay correct; that is why headers
           parsed and transfers completed pre-patch). Dropping FE bytes
           stalls any count-based protocol on this link while the bytes
           themselves are usually good. The integrity consequences of
           delivering them are handled UP the stack: the receiver's
           count gate and the v2 CRC. DR is read FIRST regardless, so
           every flag-clear sequence and the Rev 3 snapshot are
           unchanged. */
        {
            u8 Local_u8Drop = 0u;

            if (GET_BIT(Local_u32Sr, MUSART_SR_FE) == 1)
            {
                Local_Inst->FeDroppedCount++;
#if MUSART_DROP_FRAMING_ERRORS
                Local_u8Drop = 1u;
#endif
            }
#if MUSART_DROP_NOISY_BYTES
            else if (GET_BIT(Local_u32Sr, MUSART_SR_NF) == 1)
            {
                Local_Inst->FeDroppedCount++;
                Local_u8Drop = 1u;
            }
#endif

            if ((Local_u8Drop == 0u) && (Local_Inst->RxCallback != NULL))
            {
                Local_Inst->RxCallback(Local_u8Data);
            }
        }
    }

    /* ============ 2. IDLE — frame complete ============
     * SNAPSHOT-detected — THE FIX. In the combined case the hardware
     * flag was already cleared by branch 1's DR read; this snapshot is
     * the only place it still exists.
     * Hardware clear: the snapshot read primed the sequence; this DR
     * read completes it for the pure-idle case (branch 1 never ran, no
     * DR read yet). In the combined case branch 1's read already
     * completed it and this extra read is a harmless no-op (RXNE=0,
     * stale value, discarded via (void)).
     * IDLE-ALONE FOOTGUN (Rev 4, documented at MUSART_INT_IDLE): with
     * RXNEIE=0 this DR read discards the pending byte unread —
     * unavoidable while clearing IDLE, and over a multi-byte frame it
     * is the FIRST byte (the only one that reached RDR; later bytes
     * were dropped by hardware overrun). IDLE must be armed with RXNE;
     * MUSART_ReceiveEnable always arms the pair.
     * Theoretical window, documented: a byte landing between the
     * snapshot and this DR read (next frame starting within ~1-2 char
     * times of the last) would be consumed and discarded here. Real
     * ESP8266 traffic never frames that tightly; the production
     * protocol's ring buffer (see interface framing note) removes the
     * window entirely. */
    if ((GET_BIT(Local_u32Sr, MUSART_SR_IDLE) == 1) &&
        (GET_BIT(Local_Reg->CR1, MUSART_CR1_IDLEIE) == 1))
    {
        (void)Local_Reg->DR;                   /* completes the IDLE clear */

        if (Local_Inst->IdleCallback != NULL)
        {
            Local_Inst->IdleCallback();
        }
    }

    /* ============ 3. TXE — one-shot transmit-ready ============
     * LIVE read, deliberately NOT the snapshot: a DR write clears TXE,
     * and user callbacks in branches 1-2 may have transmitted a byte
     * before this branch runs. The live read then correctly reports
     * "DR full"; a snapshot TXE=1 would fire the callback for a DR
     * that is no longer empty (SendByteNonBlocking's TXE guard would
     * catch it with BUSY — safe, but pointlessly noisy). */
    if ((GET_BIT(Local_Reg->SR, MUSART_SR_TXE) == 1) &&
        (GET_BIT(Local_Reg->CR1, MUSART_CR1_TXEIE) == 1))
    {
        CLR_BIT(Local_Reg->CR1, MUSART_CR1_TXEIE);

        if (Local_Inst->TxCallback != NULL)
        {
            Local_Inst->TxCallback();
        }
    }

    /* ============ 4. TC — transmission truly complete ============
     * LIVE read, deliberately NOT the snapshot: a DR write also clears
     * TC. If the TXE callback (branch 3) chained the next byte, a
     * snapshot TC=1 would report "transmission complete" for a
     * transmission just superseded — a lie for direction-switching
     * callers. The live read drops the superseded event: exactly the
     * right semantics. */
    if ((GET_BIT(Local_Reg->SR, MUSART_SR_TC) == 1) &&
        (GET_BIT(Local_Reg->CR1, MUSART_CR1_TCIE) == 1))
    {
        Local_Reg->SR = ~(1uL << MUSART_SR_TC);

        if (Local_Inst->TxCompleteCallback != NULL)
        {
            Local_Inst->TxCompleteCallback();
        }
    }

    /* ORE final note: with RXNEIE armed, ORE implies RXNE=1 (so the
       snapshot saw it too) and was cleared by branch 1's DR read. With
       RXNEIE off, ORE cannot pend the IRQ at all (its interrupt gate
       is RXNEIE or EIE — and EIE is rejected at enable). No ORE storm
       is reachable in any state this driver can create. */
}

/* ---- Public vector-table entries (strong symbols override the weak
        Default_Handler aliases in startup_stm32f401xc.s — same linkage
        pattern as MSPI's handlers; NOT part of the API contract) ---- */

void USART1_IRQHandler(void) { MUSART_privateHandler(MUSART_USART1); }
void USART2_IRQHandler(void) { MUSART_privateHandler(MUSART_USART2); }
void USART6_IRQHandler(void) { MUSART_privateHandler(MUSART_USART6); }

/* ========================================================================= *
 *                        PRIVATE HELPERS                                     *
 * ========================================================================= */

/* ---- Instance validation (u32 cast BEFORE compare — the enum-garbage
        rule from MGPIO/MNVIC: a negative value passes a signed "> MAX"
        test but becomes huge as u32 and fails here) ---------------------- */
static MUSART_ReturnType_t MUSART_privateValidateInstance(MUSART_USART_t Copy_USART)
{
    if ((u32)Copy_USART >= (u32)MUSART_INSTANCE_COUNT)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    return MUSART_OK;
}

/* ---- Bounded flag poll, timed against the SysTick timebase.
        CONTRACT: MSYSTICK_Init must have run — GetElapsed_ms returns
        0 forever before init, and this loop would never exit. Same
        dependency note MSPI carries. ------------------------------------ */
static u8 MUSART_privateWaitForFlagTimeout(volatile MUSART_Registers_t *Copy_USART, u8 Copy_u8Flag)
{
    u32 Local_u32StartTime = MSYSTICK_GetElapsed_ms();   /* Rev 2 API name */

    while (GET_BIT(Copy_USART->SR, Copy_u8Flag) == 0)
    {
        if ((MSYSTICK_GetElapsed_ms() - Local_u32StartTime) >= MUSART_TIMEOUT_MS)
        {
            return 0;   /* timeout */
        }
    }
    return 1;
}

/* =====================================================================
 *  MUSART_GetFramingErrorCount  (Rev 5)
 * ===================================================================== */
MUSART_ReturnType_t MUSART_GetFramingErrorCount(MUSART_USART_t Copy_USART, u32 *Copy_pu32Count)
{
    if (MUSART_privateValidateInstance(Copy_USART) != MUSART_OK)
    {
        return MUSART_INVALID_PERIPHERAL;
    }
    if (Copy_pu32Count == (u32 *)0)
    {
        return MUSART_NULL_POINTER;
    }
    *Copy_pu32Count = MUSART_Instances[Copy_USART].FeDroppedCount;
    return MUSART_OK;
}
