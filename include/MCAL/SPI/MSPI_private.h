/*
 * MSPI_private.h
 *
 *  Created on: Aug 15, 2026
 *      Author: dell
 *  Description: Hardware register definitions and memory map for the SPI peripheral.
 *               This file is strictly for MCAL internal use. The application layer
 *               should never include this file directly.
 */

#ifndef MSPI_PRIVATE_H_
#define MSPI_PRIVATE_H_

#include "STD_TYPES.h"

/* ========================================================================= *
 *                        BASE MEMORY ADDRESSES                              *
 * ========================================================================= *
 *  Defined in RM0368 Section 28.5 (SPI register map).
 *  SPI1 is on APB2, SPI2/SPI3 are on APB1.
 * ========================================================================= */
#define SPI1_BASE_ADDRESS   (0x40013000)
#define SPI2_BASE_ADDRESS   (0x40003800)
#define SPI3_BASE_ADDRESS   (0x40003C00)

/* ========================================================================= *
 *                        SPI REGISTER MAP STRUCTURE                         *
 * ========================================================================= *
 *  Note on 'const volatile':
 *  Read-only hardware registers (SR, RXCRCR, TXCRCR) are declared 'const volatile'.
 *  - 'volatile' tells the compiler not to optimize away reads/writes because
 *    the hardware can change the value at any time without the CPU knowing.
 *  - 'const' enforces memory protection at the C compiler level. If a developer
 *    accidentally tries to write to the Status Register (SR), the compiler will
 *    throw an error at compile-time, preventing a silent hardware fault.
 * ========================================================================= */
typedef struct {
    volatile u32 CR1;             // 0x00 SPI Control Register 1 (Read/Write)
    volatile u32 CR2;             // 0x04 SPI Control Register 2 (Read/Write)
    const volatile u32 SR;        // 0x08 SPI Status Register (Read-Only)
    volatile u32 DR;              // 0x0C SPI Data Register (Read/Write - Special hardware behavior)
    volatile u32 CRCPR;           // 0x10 SPI CRC Polynomial Register (Read/Write)
    const volatile u32 RXCRCR;    // 0x14 SPI RX CRC Register (Read-Only)
    const volatile u32 TXCRCR;    // 0x18 SPI TX CRC Register (Read-Only)
    volatile u32 I2SCFGR;         // 0x1C SPI I2S Configuration Register (Read/Write)
    volatile u32 I2SPR;           // 0x20 SPI I2S Prescaler Register (Read/Write)
} SPI_t;

/* Pointers to all hardware instances */
#define MSPI1   ((volatile SPI_t*) SPI1_BASE_ADDRESS)
#define MSPI2   ((volatile SPI_t*) SPI2_BASE_ADDRESS)
#define MSPI3   ((volatile SPI_t*) SPI3_BASE_ADDRESS)

/* ========================================================================= *
 *                        CR1 REGISTER BITS                                  *
 * ========================================================================= */
#define MSPI_CR1_CPHA       0   // Clock Phase: 0=First transition capture, 1=Second
#define MSPI_CR1_CPOL       1   // Clock Polarity: 0=Idle Low, 1=Idle High
#define MSPI_CR1_MSTR       2   // Master Selection: 0=Slave, 1=Master
#define MSPI_CR1_BR         3   // Baud Rate Control (Bits 5:3). Divides the APB clock.
#define MSPI_CR1_SPE        6   // SPI Enable: 0=Disabled, 1=Enabled. MUST be 0 when configuring!
#define MSPI_CR1_LSBFIRST   7   // Frame Format: 0=MSB First, 1=LSB First
#define MSPI_CR1_SSI        8   // Internal Slave Select. Must be 1 for Master to avoid Mode Fault.
#define MSPI_CR1_SSM        9   // Software Slave Management: 1=NSS pin is free for GPIO.
#define MSPI_CR1_RXONLY     10  // Receive Only Mode: 0=Full Duplex, 1=Output disabled (MISO only)
#define MSPI_CR1_DFF        11  // Data Frame Format: 0=8-bit, 1=16-bit
#define MSPI_CR1_BIDIOE     14  // Bidirectional Output Enable (used in 1-wire mode)
#define MSPI_CR1_BIDIMODE   15  // Bidirectional Data Mode Enable (1-wire mode)

/* ========================================================================= *
 *                        CR2 REGISTER BITS                                  *
 * ========================================================================= */
#define MSPI_CR2_SSOE       2   // SS Output Enable: 1=NSS pin acts as output to select slaves
#define MSPI_CR2_ERRIE      5   // Error Interrupt Enable
#define MSPI_CR2_RXNEIE     6   // RX buffer Not Empty Interrupt Enable (Triggers ISR when data arrives)
#define MSPI_CR2_TXEIE      7   // Tx buffer Empty Interrupt Enable (Triggers ISR when ready to send)

/* ========================================================================= *
 *                        SR REGISTER BITS                                   *
 * ========================================================================= */
#define MSPI_SR_RXNE        0   // Receive buffer Not Empty (Read-only)
#define MSPI_SR_TXE         1   // Transmit buffer Empty (Read-only)
#define MSPI_SR_OVR         6   // Overrun flag (Read-only). 1=Data lost due to overrun.
#define MSPI_SR_BSY         7   // Busy flag (Read-only)
#endif /* MSPI_PRIVATE_H_ */
