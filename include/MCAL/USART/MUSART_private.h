/*
 * MUSART_private.h
 *
 *  PRIVATE header of the MUSART driver — never include from HAL or
 *  Application code. Hardware facts only: base addresses, the register
 *  map, and every bit position used or referenced.
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 19: USART
 *
 *  Revision history:
 *    Rev 1 — original (register map + bits, all verified correct)
 *    Rev 2 — naming sweep: USART_DefReg_t -> MUSART_Registers_t,
 *            USART1/2/6 macros -> MUSART1/2/6 (unprefixed instance
 *            macros were global collision hazards, same class as the
 *            old MGPIO GPIOA/GPIOB/GPIOC); "not used yet" notes
 *            updated for the serviced-interrupt set
 *
 *  Created on: Aug 23, 2026
 *      Author: dell
 */

#ifndef MUSART_PRIVATE_H_
#define MUSART_PRIVATE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Base addresses (RM0368 §2.3 memory map)
 *
 *  The F401 has exactly three USARTs: 1, 2, 6. There is no USART3/4/5
 *  on this part — the family keeps the numbering, the silicon does
 *  not follow it. USART1/6 are APB2 peripherals; USART2 is APB1.
 * ===================================================================== */
#define MUSART_USART1_BASE_ADDRESS   ((u32)0x40011000)
#define MUSART_USART2_BASE_ADDRESS   ((u32)0x40004400)
#define MUSART_USART6_BASE_ADDRESS   ((u32)0x40011400)

#define MUSART1   ((volatile MUSART_Registers_t *) MUSART_USART1_BASE_ADDRESS)
#define MUSART2   ((volatile MUSART_Registers_t *) MUSART_USART2_BASE_ADDRESS)
#define MUSART6   ((volatile MUSART_Registers_t *) MUSART_USART6_BASE_ADDRESS)

/* =====================================================================
 *  Register map (RM0368 §19.6 — offsets 0x00-0x18, verified)
 *
 *  SR stays fully declared (not a reduced struct) because several of
 *  its flags are clear-by-software-sequence and the ISR reasoning
 *  depends on knowing them all:
 *    RXNE  : cleared by reading DR
 *    IDLE  : cleared by the sequence read-SR then read-DR
 *    ORE   : cleared by the sequence read-SR then read-DR
 *    TC    : cleared by writing 0 to the TC bit (idiom used in the
 *            ISR: SR = ~(1<<TC) — 1s elsewhere are no-ops)
 *    LBD/CTS: cleared by writing 0 (never armed by this driver)
 *    TXE   : hardware-set when DR empties; cannot be software-cleared
 * ===================================================================== */
typedef struct
{
    volatile u32 SR;    /* 0x00 : Status register                        */
    volatile u32 DR;    /* 0x04 : Data register (read clears RXNE)       */
    volatile u32 BRR;   /* 0x08 : Baud rate register                     */
    volatile u32 CR1;   /* 0x0C : Control register 1                     */
    volatile u32 CR2;   /* 0x10 : Control register 2                     */
    volatile u32 CR3;   /* 0x14 : Control register 3                     */
    volatile u32 GTPR;  /* 0x18 : Guard time and prescaler register     */
} MUSART_Registers_t;

/* ===================== SR bit positions ===================== */
#define MUSART_SR_PE     0u   /* Parity error — never armed (rejected)   */
#define MUSART_SR_FE     1u   /* Framing error — info only               */
#define MUSART_SR_NF     2u   /* Noise detected flag — info only         */
#define MUSART_SR_ORE    3u   /* Overrun: cleared with the RXNE DR read  */
#define MUSART_SR_IDLE   4u   /* Idle line detected (FRAME COMPLETE)     */
#define MUSART_SR_RXNE   5u   /* Receive data register not empty         */
#define MUSART_SR_TC     6u   /* Transmission complete                   */
#define MUSART_SR_TXE    7u   /* Transmit data register empty            */
#define MUSART_SR_LBD    8u   /* LIN break detection — never armed       */
#define MUSART_SR_CTS    9u   /* CTS flag — never armed                  */

/* ===================== CR1 bit positions ===================== */
#define MUSART_CR1_SBK     0u   /* Send break                             */
#define MUSART_CR1_RWU     1u   /* Receiver wakeup                        */
#define MUSART_CR1_RE      2u   /* Receiver enable                        */
#define MUSART_CR1_TE      3u   /* Transmitter enable                     */
#define MUSART_CR1_IDLEIE  4u   /* IDLE interrupt enable — SERVICED       */
#define MUSART_CR1_RXNEIE  5u   /* RXNE interrupt enable — SERVICED       */
#define MUSART_CR1_TCIE    6u   /* TC interrupt enable — SERVICED         */
#define MUSART_CR1_TXEIE   7u   /* TXE interrupt enable — SERVICED        */
#define MUSART_CR1_PEIE    8u   /* PE interrupt enable — never armed      */
#define MUSART_CR1_PS      9u   /* Parity selection (0=even, 1=odd)       */
#define MUSART_CR1_PCE     10u  /* Parity control enable                  */
#define MUSART_CR1_WAKE    11u  /* Wakeup method                          */
#define MUSART_CR1_M       12u  /* Word length (0=8 bits, 1=9 bits)       */
#define MUSART_CR1_UE      13u  /* USART enable                           */
#define MUSART_CR1_OVER8   15u  /* Oversampling mode (0=16, 1=8)          */

/* ===================== CR2 bit positions ===================== */
#define MUSART_CR2_ADD     0u   /* Node address — multiprocessor, unused  */
#define MUSART_CR2_LBDL    5u   /* LIN break length — unused              */
#define MUSART_CR2_LBDIE   6u   /* LIN break int enable — never armed     */
#define MUSART_CR2_LBCL    8u   /* Last bit clock pulse — sync, stub      */
#define MUSART_CR2_CPHA    9u   /* Clock phase — sync, stub               */
#define MUSART_CR2_CPOL    10u  /* Clock polarity — sync, stub            */
#define MUSART_CR2_CLKEN   11u  /* SCK pin enable — sync mode             */
#define MUSART_CR2_STOP        12u  /* STOP[1:0]: 00=1, 01=0.5, 10=2, 11=1.5 */
#define MUSART_CR2_STOP_MASK   (0x03u << MUSART_CR2_STOP)
#define MUSART_CR2_LINEN   14u  /* LIN mode enable — unused               */

/* ===================== CR3 bit positions ===================== */
#define MUSART_CR3_EIE     0u   /* Error int enable (FE/ORE/NF) — REJECTED at enable */
#define MUSART_CR3_IREN    1u   /* IrDA mode — unused                     */
#define MUSART_CR3_IRLP    2u   /* IrDA low-power — unused                */
#define MUSART_CR3_HDSEL   3u   /* Half-duplex selection — unused         */
#define MUSART_CR3_NACK    4u   /* Smartcard NACK — stub                  */
#define MUSART_CR3_SCEN    5u   /* Smartcard mode — STUB (see config)     */
#define MUSART_CR3_DMAR    6u   /* DMA enable receiver (Phase 1)          */
#define MUSART_CR3_DMAT    7u   /* DMA enable transmitter (Phase 1)       */
#define MUSART_CR3_RTSE    8u   /* RTS enable — no flow control used      */
#define MUSART_CR3_CTSE    9u   /* CTS enable — no flow control used      */
#define MUSART_CR3_CTSIE   10u  /* CTS int enable — never armed           */
#define MUSART_CR3_ONEBIT  11u  /* One sample bit method                  */

/* ===================== GTPR bits (smartcard stub) ===================== */
#define MUSART_GTPR_PSC        0u
#define MUSART_GTPR_PSC_MASK   (0xFFu << MUSART_GTPR_PSC)
#define MUSART_GTPR_GT         8u
#define MUSART_GTPR_GT_MASK    (0xFFu << MUSART_GTPR_GT)

/* =====================================================================
 *  Storm-safety map (the load-bearing invariant of this driver):
 *
 *  Only these four sources can EVER be enabled, and each has a
 *  guaranteed clear inside its ISR:
 *      TXE  -> TXEIE disabled in-ISR before the callback (one-shot)
 *      RXNE -> DR read in-ISR (clears RXNE and ORE)
 *      IDLE -> SR-read (the flag GET_BITs) + DR read in-ISR
 *      TC   -> write-0-to-clear in-ISR
 *  PE / LBD / CTS / ERR are rejected at ENABLE time (MUSART_
 *  NOT_SUPPORTED) — they can never be armed, so no unserviced flag
 *  can pend the IRQ line forever. This is what "safe by construction"
 *  means here: the Rev 1 API could arm 7 storm sources; this one
 *  can arm zero.
 * ===================================================================== */

#endif /* MUSART_PRIVATE_H_ */
