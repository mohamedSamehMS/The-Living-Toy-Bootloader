/*
 * MUSART_config.h
 *
 *  Configuration of the MUSART driver.
 *
 *  Section 1 consumes the MRCC PUBLISHED CLOCK MODEL (the hard-coded
 *  16 MHz constants that used to live here are gone — same bug class
 *  as the one removed from MSYSTICK). Sections 2-4 are per-instance
 *  user choices. Section 5 validates.
 *
 *  NOTE (layering, fixed): this file no longer includes the interface
 *  header — config is pure data; program.c consumes both.
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 19: USART
 *
 *  Revision history:
 *    Rev 1 — original (hard-coded APB clocks; DMA flags on with no
 *            DMA driver behind them)
 *    Rev 2 — clocks from MRCC model; DMA flags defaulted to 0 until
 *            the Phase-1 DMA driver exists; numeric validation added
 *
 *  Created on: Aug 28, 2026
 *      Author: dell
 */

#ifndef MUSART_CONFIG_H_
#define MUSART_CONFIG_H_

#include <MCAL/RCC/MRCC_interface.h>   /* publishes MRCC_PCLK1/2_KHZ */

/* =====================================================================
 *  1. Bus clocks & timeout — DERIVED, not hand-set
 *
 *  USART1/USART6 sit on APB2; USART2 on APB1. The old hard-coded
 *  16000000UL values would have survived an MRCC flip to PLL_HSE:
 *  USART1's BRR would still assume 16 MHz while APB2 ran at 84 MHz —
 *  a "9600 baud" link actually running at 50,400 baud, silently.
 *  Consuming the model means BRR math follows any clock change.
 * ===================================================================== */
#define MUSART_PCLK1_HZ              (MRCC_PCLK1_KHZ * 1000UL)
#define MUSART_PCLK2_HZ              (MRCC_PCLK2_KHZ * 1000UL)

/* Blocking-operation timeout (TXE/RXNE waits). SysTick-driven. */
#define MUSART_TIMEOUT_MS            1000UL

/* Rev 5b: 1 = FE-flagged bytes are DROPPED in addition to being
   counted. Default 0 = counted and DELIVERED — bench finding (runs
   5-7): FE on this link flags mostly-valid bytes (probable few-percent
   adapter baud mismatch; single-sampled stop bit vs majority-voted
   data bits), so dropping them stalls count-based transfer while the
   data is usually correct. Flip to 1 only on a link whose FE means
   real garbage. */
#define MUSART_DROP_FRAMING_ERRORS    0

#if (MUSART_DROP_FRAMING_ERRORS != 0) && (MUSART_DROP_FRAMING_ERRORS != 1)
#error "MUSART: MUSART_DROP_FRAMING_ERRORS must be 0 or 1"
#endif

#define MUSART_DROP_NOISY_BYTES     0
#if (MUSART_DROP_NOISY_BYTES != 0) && (MUSART_DROP_NOISY_BYTES != 1)
#error "MUSART: MUSART_DROP_NOISY_BYTES must be 0 or 1"
#endif
/* =====================================================================
 *  2. Configuration options guide
 *
 *  ENABLED:        1 (UE set at init) | 0 (configured but left disabled)
 *  BAUDRATE:       any standard rate — ESP8266 AT firmware commonly
 *                  defaults to 115200; match it here
 *  WORDLENGTH:     MUSART_WORDLEN_8BITS | MUSART_WORDLEN_9BITS
 *                  (API is byte-oriented — see interface notes on the
 *                  9-bit/parity interactions before using 9BITS)
 *  PARITY:         MUSART_PARITY_NONE | _EVEN | _ODD
 *  STOPBITS:       MUSART_STOPBITS_1 | _0_5 | _2 | _1_5
 *  MODE:           MUSART_MODE_TX_ONLY | _RX_ONLY | _TX_RX
 *  OVERSAMPLING:   MUSART_OVERSAMPLING_16 | _8
 *  SYNC_MODE:      MUSART_ASYNC | MUSART_SYNC (CLKEN stub — CPOL/CPHA/
 *                  LBCL stay at reset defaults; complete only if sync
 *                  mode is ever actually needed)
 *  DMA_TX / DMA_RX: 1 = set CR3 DMAT/DMAR. NOTE: the request lines are
 *                  enabled but NOTHING services them until the Phase-1
 *                  DMA driver exists — leave 0 until then (a set DMAT
 *                  with no stream is inert, but also pointless).
 *  SMARTCARD_MODE: 1 = SCEN (STUB — guard time/stop-bit requirements
 *                  are the application's job; not used in Project 4)
 * ===================================================================== */

/* ===================== USART1 (ESP8266 link) ===================== */
#define MUSART1_ENABLED          1
#define MUSART1_BAUDRATE         115200u
#define MUSART1_WORDLENGTH       MUSART_WORDLEN_8BITS
#define MUSART1_PARITY           MUSART_PARITY_NONE
#define MUSART1_STOPBITS         MUSART_STOPBITS_1
#define MUSART1_MODE             MUSART_MODE_TX_RX
#define MUSART1_OVERSAMPLING     MUSART_OVERSAMPLING_16
#define MUSART1_SYNC_MODE        MUSART_ASYNC
#define MUSART1_DMA_TX           0
#define MUSART1_DMA_RX           0
#define MUSART1_SMARTCARD_MODE   0

/* ===================== USART2 ===================== */
#define MUSART2_ENABLED          1
#define MUSART2_BAUDRATE         115200u
#define MUSART2_WORDLENGTH       MUSART_WORDLEN_8BITS
#define MUSART2_PARITY           MUSART_PARITY_NONE
#define MUSART2_STOPBITS         MUSART_STOPBITS_1
#define MUSART2_MODE             MUSART_MODE_TX_RX
#define MUSART2_OVERSAMPLING     MUSART_OVERSAMPLING_16
#define MUSART2_SYNC_MODE        MUSART_ASYNC
#define MUSART2_DMA_TX           0
#define MUSART2_DMA_RX           0
#define MUSART2_SMARTCARD_MODE   0

/* ===================== USART6 ===================== */
#define MUSART6_ENABLED          0
#define MUSART6_BAUDRATE         9600u
#define MUSART6_WORDLENGTH       MUSART_WORDLEN_8BITS
#define MUSART6_PARITY           MUSART_PARITY_NONE
#define MUSART6_STOPBITS         MUSART_STOPBITS_1
#define MUSART6_MODE             MUSART_MODE_TX_RX
#define MUSART6_OVERSAMPLING     MUSART_OVERSAMPLING_16
#define MUSART6_SYNC_MODE        MUSART_ASYNC
#define MUSART6_DMA_TX           0
#define MUSART6_DMA_RX           0
#define MUSART6_SMARTCARD_MODE   0

/* =====================================================================
 *  3. Validity note for enum-valued macros
 *
 *  The WORDLENGTH/PARITY/... macros hold ENUM names, which the
 *  preprocessor cannot test — their validation happens at runtime
 *  in every MUSART function (the enum-not-type-safe rule). Only the
 *  NUMERIC macros can be checked here, and are, below.
 * ===================================================================== */

/* =====================================================================
 *  4. Configuration validation — bad numeric values stop the build
 * ===================================================================== */
#if (MUSART1_BAUDRATE == 0) || (MUSART2_BAUDRATE == 0) || (MUSART6_BAUDRATE == 0)
#error "MUSART: baud rate must be non-zero (division in BRR math)"
#endif

#if (MUSART1_ENABLED != 0) && (MUSART1_ENABLED != 1)
#error "MUSART: MUSART1_ENABLED must be 0 or 1"
#endif
#if (MUSART2_ENABLED != 0) && (MUSART2_ENABLED != 1)
#error "MUSART: MUSART2_ENABLED must be 0 or 1"
#endif
#if (MUSART6_ENABLED != 0) && (MUSART6_ENABLED != 1)
#error "MUSART: MUSART6_ENABLED must be 0 or 1"
#endif

#if (MUSART1_DMA_TX != 0) && (MUSART1_DMA_TX != 1)
#error "MUSART: MUSART1_DMA_TX must be 0 or 1"
#endif
#if (MUSART1_DMA_RX != 0) && (MUSART1_DMA_RX != 1)
#error "MUSART: MUSART1_DMA_RX must be 0 or 1"
#endif
#if (MUSART2_DMA_TX != 0) && (MUSART2_DMA_TX != 1)
#error "MUSART: MUSART2_DMA_TX must be 0 or 1"
#endif
#if (MUSART2_DMA_RX != 0) && (MUSART2_DMA_RX != 1)
#error "MUSART: MUSART2_DMA_RX must be 0 or 1"
#endif
#if (MUSART6_DMA_TX != 0) && (MUSART6_DMA_TX != 1)
#error "MUSART: MUSART6_DMA_TX must be 0 or 1"
#endif
#if (MUSART6_DMA_RX != 0) && (MUSART6_DMA_RX != 1)
#error "MUSART: MUSART6_DMA_RX must be 0 or 1"
#endif

#if (MUSART1_SMARTCARD_MODE != 0) && (MUSART1_SMARTCARD_MODE != 1)
#error "MUSART: MUSART1_SMARTCARD_MODE must be 0 or 1"
#endif
#if (MUSART2_SMARTCARD_MODE != 0) && (MUSART2_SMARTCARD_MODE != 1)
#error "MUSART: MUSART2_SMARTCARD_MODE must be 0 or 1"
#endif
#if (MUSART6_SMARTCARD_MODE != 0) && (MUSART6_SMARTCARD_MODE != 1)
#error "MUSART: MUSART6_SMARTCARD_MODE must be 0 or 1"
#endif

#endif /* MUSART_CONFIG_H_ */
