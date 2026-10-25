/*
 * MUSART_interface.h
 *
 *  PUBLIC interface of the MUSART driver. HAL and Application code
 *  must include ONLY this file.
 *
 *  Reference: RM0368 (STM32F401xB/xC), Ch. 19: USART
 *
 *  BOOT-ORDER CONTRACT (all four are HARD prerequisites):
 *      1. MRCC_Init()  — clock tree matches the published model the
 *         BRR math consumes
 *      2. MRCC_EnablePeripheralClock(MRCC_USART1) — writes to an
 *         un-clocked USART are SILENTLY ignored (the classic
 *         "nothing works, no error anywhere" trap)
 *      3. GPIO AF wiring (this driver does NOT own pins):
 *              MGPIO_SetPinMode        (MGPIO_PORTA, MGPIO_PIN_9,  MGPIO_MODE_ALT_FUNC)
 *              MGPIO_SetPinOutputConfig(MGPIO_PORTA, MGPIO_PIN_9,  MGPIO_OUTPUT_PUSH_PULL, HIGH)
 *              MGPIO_SetPinAltFunction (MGPIO_PORTA, MGPIO_PIN_9,  MGPIO_AF7)
 *              MGPIO_SetPinMode        (MGPIO_PORTA, MGPIO_PIN_10, MGPIO_MODE_ALT_FUNC)
 *              MGPIO_SetPinOutputConfig(MGPIO_PORTA, MGPIO_PIN_10, MGPIO_OUTPUT_PUSH_PULL, HIGH)
 *              MGPIO_SetPinAltFunction (MGPIO_PORTA, MGPIO_PIN_10, MGPIO_AF7)
 *      4. MSYSTICK_Init() — blocking functions time out against the
 *         SysTick timebase; without it they hang
 *  Also recommended: MNVIC_Init() + MNVIC_SetPriority(MNVIC_USART1,
 *  3, 0) BEFORE first use — otherwise the USART IRQ sits at reset
 *  priority 0 (HIGHEST) and preempts EXTI/SysTick.
 *
 *  FRAMING CONTRACT (the ESP8266 pattern this driver is built for):
 *  RX is CONTINUOUS once armed. Every byte -> RxCallback(byte); the
 *  application accumulates into a frame buffer. When the line goes
 *  idle (one character-time of silence after data) -> IdleCallback:
 *  the frame is complete, parse it now. No per-byte re-arming exists
 *  — the Rev 1 one-shot model could not survive 115200 baud (one
 *  byte every 86.8 us; re-arm latency during a TFT render exceeds
 *  that and silently overruns the link).
 *
 *  Revision history:
 *    Rev 1 — original API (one-shot RX; 9 arm-able interrupt sources,
 *            2 serviced; handler prototypes in the header)
 *    Rev 2 — continuous RX + IDLE framing; unserviced sources
 *            rejected at enable; TX one-shot kept (storm-proof);
 *            TC callback added; handlers removed from the public
 *            contract; MUSART_ namespace sweep
 *
 *  Created on: Aug 23, 2026
 *      Author: dell
 */

#ifndef MUSART_INTERFACE_H_
#define MUSART_INTERFACE_H_

#include "STD_TYPES.h"

/* =====================================================================
 *  Driver status codes — MUSART_ convention, matching the library
 * ===================================================================== */
typedef enum
{
    MUSART_OK = 0,             /* Operation completed                    */
    MUSART_NULL_POINTER,       /* NULL output pointer passed             */
    MUSART_TIMEOUT,            /* Blocking wait exceeded MUSART_TIMEOUT_MS*/
    MUSART_INVALID_PERIPHERAL, /* Instance outside USART1/2/6            */
    MUSART_INVALID_CONFIG,     /* Bad interrupt id, baud 0, or BRR
                                  mantissa overflow (baud too low for
                                  the clock)                             */
    MUSART_BUSY,               /* Non-blocking TX while DR still full     */
    MUSART_NOT_SUPPORTED,      /* Interrupt source exists in hardware but
                                  is NOT serviced — enabling it would
                                  storm the IRQ (see storm-safety map)   */
    MUSART_NOT_ENABLED         /* Peripheral UE=0 (blocking op on a
                                  disabled USART — refuse, don't hang)   */
} MUSART_ReturnType_t;

/* =====================================================================
 *  Hardware instance ids
 *
 *  Values index the internal instance table — keep 0,1,2 in order.
 *  (The F401 has no USART3/4/5; the family numbering jumps 2 -> 6.)
 * ===================================================================== */
typedef enum
{
    MUSART_USART1 = 0,         /* APB2 — the ESP8266 link                */
    MUSART_USART2,             /* APB1                                    */
    MUSART_USART6              /* APB2                                    */
} MUSART_USART_t;

/* ===================== Frame-format enums ===================== */

/** Word length (CR1 M). API is byte-oriented: with 9BITS + parity the
 *  frame is 8 data + parity; with 9BITS and no parity the 9th bit is
 *  not accessible through the u8 API. With 8BITS + parity the frame
 *  is 7 data + parity (bit 7 of received bytes IS the parity bit). */
typedef enum
{
    MUSART_WORDLEN_8BITS = 0,
    MUSART_WORDLEN_9BITS
} MUSART_WordLength_t;

/** Parity control (CR1 PCE & PS) */
typedef enum
{
    MUSART_PARITY_NONE = 0,
    MUSART_PARITY_EVEN,
    MUSART_PARITY_ODD
} MUSART_Parity_t;

/** Stop bits (CR2 STOP[1:0]) — values ARE the register encodings */
typedef enum
{
    MUSART_STOPBITS_1 = 0,     /* 00 */
    MUSART_STOPBITS_0_5,       /* 01 */
    MUSART_STOPBITS_2,         /* 10 */
    MUSART_STOPBITS_1_5        /* 11 — mainly smartcard */
} MUSART_StopBits_t;

/** TX/RX direction (CR1 TE & RE) */
typedef enum
{
    MUSART_MODE_TX_ONLY = 0,
    MUSART_MODE_RX_ONLY,
    MUSART_MODE_TX_RX
} MUSART_USARTMode_t;

/** Oversampling (CR1 OVER8) — 8 halves the BRR fraction resolution
 *  but allows higher baud at high PCLK; 16 is the default */
typedef enum
{
    MUSART_OVERSAMPLING_16 = 0,
    MUSART_OVERSAMPLING_8
} MUSART_Oversampling_t;

/** Synchronous mode (CR2 CLKEN) — STUB: CPOL/CPHA/LBCL stay at reset
 *  defaults; complete this only if sync mode is ever needed */
typedef enum
{
    MUSART_ASYNC = 0,
    MUSART_SYNC
} MUSART_SyncMode_t;

/* =====================================================================
 *  Interrupt sources (RM0368 USART interrupt map)
 *
 *  Only TXE / RXNE / TC / IDLE can be enabled — the other five
 *  return MUSART_NOT_SUPPORTED at enable time. They remain in the
 *  enum (they are real hardware sources) so a caller gets a LOUD
 *  rejection instead of a silent lockup.
 * ===================================================================== */
typedef enum
{
    MUSART_INT_TXE = 0,        /* SERVICED — TxCallback                  */
    MUSART_INT_RXNE,           /* SERVICED — RxCallback(byte), continuous*/
    MUSART_INT_TC,             /* SERVICED — TxCompleteCallback          */
	MUSART_INT_IDLE,           /* SERVICED — IdleCallback (frame done).
	                                   FOOTGUN: arm IDLE only via
	                                   MUSART_ReceiveEnable (arms RXNE+IDLE
	                                   together). Arming IDLE ALONE through
	                                   this low-level API yields frame-END
	                                   events whose data is systematically
	                                   lost: with RXNEIE=0 no byte is ever
	                                   delivered, later bytes are dropped by
	                                   hardware overrun, and the clear
	                                   sequence's DR read discards the one
	                                   byte (the frame's first) that reached
	                                   RDR. */
    MUSART_INT_PE,             /* NOT SUPPORTED — rejected at enable     */
    MUSART_INT_LBD,            /* NOT SUPPORTED                          */
    MUSART_INT_CTS,            /* NOT SUPPORTED                          */
    MUSART_INT_ERR             /* NOT SUPPORTED (FE/ORE/NF via EIE)      */
} MUSART_Interrupt_t;

/* ===================== Callback types ===================== */

/** TX byte handed to the shift register (TXE): DR is empty, the byte
 *  is moving — "ready for the next write", NOT "on the wire". */
typedef void (*MUSART_TxCallback_t)(void);

/** Transmission truly complete (TC): the LAST stop bit is on the wire
 *  and the line is quiet. Use for direction-switching links. */
typedef void (*MUSART_TxCompleteCallback_t)(void);

/** One received byte. Runs in ISR context: append to a buffer, do
 *  nothing slower. */
typedef void (*MUSART_RxCallback_t)(u8 Copy_u8Data);

/** Line idle after data — the accumulated frame is complete. Parse
 *  here. Runs in ISR context. */
typedef void (*MUSART_IdleCallback_t)(void);

/* ========================================================================= *
 *                           PUBLIC API                                      *
 * ========================================================================= */

/**
 * @brief  Configures one USART from MUSART_config.h and enables it if
 *         the instance config says so. Must run after MRCC clock enable
 *         and GPIO AF wiring (see boot-order contract).
 * @note   Re-Init at runtime: legal (UE is cleared first — RM0368
 *         requires configuration with UE=0), but drops in-flight data.
 */
MUSART_ReturnType_t MUSART_Init(MUSART_USART_t Copy_USART);

/** Runtime UE enable/disable (after a config that started disabled). */
MUSART_ReturnType_t MUSART_Enable (MUSART_USART_t Copy_USART);
MUSART_ReturnType_t MUSART_Disable(MUSART_USART_t Copy_USART);

/* --------------------- Interrupt management --------------------- */

/**
 * @brief  Arm one USART interrupt source + its NVIC line.
 *
 * TXE: one-shot per byte — the ISR disables TXEIE before calling
 *      TxCallback; for a stream, the callback writes the next byte
 *      via MUSART_SendByteNonBlocking (which re-arms).
 * RXNE: CONTINUOUS — every byte fires RxCallback until disabled.
 *      This is the framing-workhorse; prefer MUSART_ReceiveEnable.
 * TC:  fires TxCompleteCallback once per completed transmission
 *      (flag cleared in-ISR by the write-0 idiom).
 * IDLE: fires IdleCallback on every idle-after-data detection
 *      (flag cleared in-ISR by the SR-read + DR-read sequence).
 * PE/LBD/CTS/ERR: MUSART_NOT_SUPPORTED — never arm an unserviced
 *      source; that is an interrupt-storm lockup by construction.
 *
 * @note   NVIC enable happens ONLY after parameter validation (Rev 1
 *         enabled the line first — a bad argument still armed it).
 */
MUSART_ReturnType_t MUSART_InterruptEnable (MUSART_USART_t Copy_USART, MUSART_Interrupt_t Copy_Interrupt);
MUSART_ReturnType_t MUSART_InterruptDisable(MUSART_USART_t Copy_USART, MUSART_Interrupt_t Copy_Interrupt);

/* --------------------- Blocking functions --------------------- */

/**
 * @brief  Send one byte, waiting (bounded by MUSART_TIMEOUT_MS) for
 *         TXE, then writing DR.
 * @retval MUSART_NOT_ENABLED if UE=0 (refuse instead of hanging),
 *         MUSART_TIMEOUT if TXE never asserts.
 * @note   Requires MSYSTICK_Init (timeout timebase).
 */
MUSART_ReturnType_t MUSART_SendByteBlocking(MUSART_USART_t Copy_USART, u8 Copy_u8Data);

/**
 * @brief  Receive one byte, waiting (bounded) for RXNE, then reading
 *         DR (which also clears a latched ORE).
 * @note   If an overrun already occurred, DR holds the OLDER byte —
 *         the newer one was lost by hardware; the read just recovers
 *         the link without hanging.
 */
MUSART_ReturnType_t MUSART_ReceiveByteBlocking(MUSART_USART_t Copy_USART, u8 *Copy_pu8Data);

/* --------------------- Non-blocking functions --------------------- */

/**
 * @brief  Start a one-byte non-blocking transmission: writes DR (must
 *         already be empty — else MUSART_BUSY) and arms the TXE
 *         interrupt. TxCallback fires once, when the byte reaches the
 *         shift register; the ISR disarms TXEIE itself (storm-proof).
 *         Chain bytes from the callback for streams.
 */
MUSART_ReturnType_t MUSART_SendByteNonBlocking(MUSART_USART_t Copy_USART, u8 Copy_u8Data);

/**
 * @brief  Arm CONTINUOUS reception + idle detection (RXNEIE + IDLEIE
 *         + NVIC): every arriving byte -> RxCallback, each idle-after-
 *         data -> IdleCallback. THE ESP8266 FRAMING PRIMITIVE.
 * @note   If a byte is already pending in DR when armed, the IRQ fires
 *         immediately with it — arming never discards buffered data.
 */
MUSART_ReturnType_t MUSART_ReceiveEnable (MUSART_USART_t Copy_USART);

/** Disarm continuous reception (RXNEIE + IDLEIE). NVIC line stays
 *  enabled — harmless; re-arm with ReceiveEnable. */
MUSART_ReturnType_t MUSART_ReceiveDisable(MUSART_USART_t Copy_USART);

/* --------------------- Callback setters --------------------- */
/* NULL is a VALID value for every setter: it UNREGISTERS the callback
 * (matching the MSYSTICK convention). An event with no callback set
 * is simply ignored — never a crash. */
MUSART_ReturnType_t MUSART_SetTxCallback         (MUSART_USART_t Copy_USART, MUSART_TxCallback_t         Copy_pfCallback);
MUSART_ReturnType_t MUSART_SetTxCompleteCallback (MUSART_USART_t Copy_USART, MUSART_TxCompleteCallback_t Copy_pfCallback);
MUSART_ReturnType_t MUSART_SetRxCallback         (MUSART_USART_t Copy_USART, MUSART_RxCallback_t         Copy_pfCallback);
MUSART_ReturnType_t MUSART_SetIdleCallback       (MUSART_USART_t Copy_USART, MUSART_IdleCallback_t       Copy_pfCallback);

/* Rev 5: number of bytes this instance's ISR has DROPPED due to framing
 * error (always) or the noise flag (MUSART_DROP_NOISY_BYTES). A
 * link-health instrument — read it after transfers. */
MUSART_ReturnType_t MUSART_GetFramingErrorCount(MUSART_USART_t Copy_USART, u32 *Copy_pu32Count);

/* =====================================================================
 *  ESP8266 framing example (the pattern the RX/IDLE design serves):
 *
 *      static u8  EspRxBuffer[256];
 *      static u16 EspRxLength = 0;
 *
 *      static void Esp_RxByte(u8 Copy_u8Byte)
 *      {
 *          if (EspRxLength < sizeof(EspRxBuffer))
 *          {
 *              EspRxBuffer[EspRxLength++] = Copy_u8Byte;
 *          }
 *      }
 *
 *      static void Esp_FrameComplete(void)
 *      {
 *          / * full AT response in EspRxBuffer[0..EspRxLength-1]:
 *             parse it here, then reset for the next frame * /
 *          Esp_Parse(EspRxBuffer, EspRxLength);
 *          EspRxLength = 0;
 *      }
 *
 *      MUSART_Init(MUSART_USART1);
 *      MUSART_SetRxCallback   (MUSART_USART1, Esp_RxByte);
 *      MUSART_SetIdleCallback (MUSART_USART1, Esp_FrameComplete);
 *      MUSART_ReceiveEnable   (MUSART_USART1);
 *
 *  (IRQ handlers are NOT part of the public contract — the startup
 *  file's weak aliases link to this driver's definitions directly.)
 * ===================================================================== */

#endif /* MUSART_INTERFACE_H_ */
