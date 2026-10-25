/*
 * MSPI_config.h
 *
 *  Created on: Aug 15, 2026
 *      Author: dell
 *  Description: Static configuration definitions for SPI instances.
 *               This file is included ONLY by MSPI_program.c.
 *               Using 'static const' makes the configs private to the driver
 *               AND forces them into Flash memory (the .rodata section) to save RAM.
 */

#ifndef MSPI_CONFIG_H_
#define MSPI_CONFIG_H_

#include "MSPI_interface.h"

/* ========================================================================= *
 *                          SPI1 CONFIGURATION                               *
 * ========================================================================= *
 *  Hardware Pins (Default Routing):
 *  - PA5: SPI1_SCK  (Alternate Function 5)
 *  - PA6: SPI1_MISO (Alternate Function 5)
 *  - PA7: SPI1_MOSI (Alternate Function 5)
 *
 *  Clock Source: APB2 (Usually 16MHz on HSI or up to 84MHz on PLL)
 *  Baud Rate:    DIV_8 -> 16MHz / 8 = 2 MHz SPI Clock.
 *                2 MHz is a very safe speed for most general-purpose SPI sensors.
 *  State:        Enabled immediately at initialization.
 *  Mode:         Mode 0 (CPOL=Low, CPHA=First) - The most common SPI mode.
 * ========================================================================= */
static const MSPI_Config_t MSPI1_Config = {
    .State     = MSPI_STATE_ENABLE,
    .Mode      = MSPI_MASTER_MODE,
    .CPOL      = MSPI_CPOL_LOW,
    .CPHA      = MSPI_CPHA_FIRST,
    .DFF       = MSPI_DFF_8BIT,
    .BaudRate  = MSPI_BR_FPCLCK_DIV_2,
    .LSBFirst  = 0
};


/* ========================================================================= *
 *                          SPI2 CONFIGURATION                               *
 * ========================================================================= *
 *  Hardware Pins (Default Routing):
 *  - PB10: SPI2_SCK  (Alternate Function 5)
 *  - PC2 : SPI2_MISO (Alternate Function 5)
 *  - PC3 : SPI2_MOSI (Alternate Function 5)
 *
 *  Clock Source: APB1 (Usually 16MHz on HSI or up to 42MHz on PLL)
 *  State:        Disabled at init. Application can enable via MSPI_voidEnable().
 *  Mode:         Mode 0 (Must match the Master's CPOL/CPHA).
 * ========================================================================= */
static const MSPI_Config_t MSPI2_Config = {
    .State     = MSPI_STATE_DISABLE,     // Keep SPI2 off until the application specifically turns it on
    .Mode      = MSPI_SLAVE_MODE,        // SPI2 will act as a Slave
    .CPOL      = MSPI_CPOL_LOW,          // Must match the Master's CPOL
    .CPHA      = MSPI_CPHA_FIRST,        // Must match the Master's CPHA
    .DFF       = MSPI_DFF_8BIT,          // 8-bit data frame
    .BaudRate  = MSPI_BR_FPCLCK_DIV_2,   // Ignored in Slave mode, but defined for completeness
    .LSBFirst  = 0                       // MSB First
};


/* ========================================================================= *
 *                          SPI3 CONFIGURATION                               *
 * ========================================================================= *
 *  Hardware Pins (Default Routing):
 *  - PB3 : SPI3_SCK  (Alternate Function 6)
 *  - PB4 : SPI3_MISO (Alternate Function 6)
 *  - PB5 : SPI3_MOSI (Alternate Function 6)
 *
 *  Clock Source: APB1 (Usually 16MHz on HSI or up to 42MHz on PLL)
 *  State:        Enabled immediately at init.
 *  Mode:         Mode 3 (CPOL=High, CPHA=Second).
 *                This mode is very common for SD cards, LCD displays, and some flash memory.
 *  Baud Rate:    DIV_16 -> 16MHz / 16 = 1 MHz SCK frequency.
 * ========================================================================= */
static const MSPI_Config_t MSPI3_Config = {
    .State     = MSPI_STATE_ENABLE,      // Turn on SPI3 immediately after init
    .Mode      = MSPI_MASTER_MODE,       // SPI3 will act as the Master
    .CPOL      = MSPI_CPOL_HIGH,         // Clock idle state is High (Mode 3)
    .CPHA      = MSPI_CPHA_SECOND,       // Data captured on second clock edge (Mode 3)
    .DFF       = MSPI_DFF_8BIT,          // 8-bit data frame
    .BaudRate  = MSPI_BR_FPCLCK_DIV_16,  // 16MHz / 16 = 1 MHz SCK frequency
    .LSBFirst  = 0                       // MSB First
};

#endif /* MSPI_CONFIG_H_ */
