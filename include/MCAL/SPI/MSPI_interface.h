/*
 * MSPI_interface.h
 *
 *  Created on: Aug 15, 2026
 *      Author: dell
 *  Description: Public API for the MCAL SPI driver.
 *               Supports Blocking (with timeouts), Non-Blocking (interrupt-driven
 *               buffer streaming), and runtime configuration.
 */

#ifndef MSPI_INTERFACE_H_
#define MSPI_INTERFACE_H_

#include "STD_TYPES.h"
#include "MSPI_private.h" // Required to expose the SPI_t hardware struct

/* ========================================================================= *
 *                           ENUMERATIONS                                    *
 * ========================================================================= */

/** @brief SPI Peripheral State. Used to enable or disable the peripheral at init. */
typedef enum {
    MSPI_STATE_DISABLE = 0, /**< Peripheral disabled (SPE=0). Saves power. */
    MSPI_STATE_ENABLE       /**< Peripheral enabled (SPE=1). Ready for use. */
} MSPI_State_t;

/** @brief SPI Mode (Master/Slave). Determines if the peripheral generates the clock or receives it. */
typedef enum {
    MSPI_SLAVE_MODE = 0,    /**< Waits for external clock from a Master. */
    MSPI_MASTER_MODE        /**< Generates its own clock to drive Slaves. */
} MSPI_Mode_t;

/** @brief Clock Polarity (CPOL). Determines the idle state of the SCK pin. */
typedef enum {
    MSPI_CPOL_LOW = 0,      /**< Clock idle state is Low (0V). */
    MSPI_CPOL_HIGH          /**< Clock idle state is High (3.3V/5V). */
} MSPI_CPOL_t;

/** @brief Clock Phase (CPHA). Determines which clock transition is used to capture (read) data. */
typedef enum {
    MSPI_CPHA_FIRST = 0,    /**< First clock transition is the first data capture edge. */
    MSPI_CPHA_SECOND        /**< Second clock transition is the first data capture edge. */
} MSPI_CPHA_t;

/** @brief Data Frame Format (DFF). Selects the length of the data transmitted/received. */
typedef enum {
    MSPI_DFF_8BIT = 0,      /**< 8-bit data frame format (Standard for most devices). */
    MSPI_DFF_16BIT          /**< 16-bit data frame format (Used by some high-res sensors). */
} MSPI_DFF_t;

/** @brief Baud Rate Prescaler. Divides the APB bus clock (PCLK) to generate the SPI Master clock (SCK). */
typedef enum {
    MSPI_BR_FPCLCK_DIV_2 = 0, MSPI_BR_FPCLCK_DIV_4, MSPI_BR_FPCLCK_DIV_8,
    MSPI_BR_FPCLCK_DIV_16, MSPI_BR_FPCLCK_DIV_32, MSPI_BR_FPCLCK_DIV_64,
    MSPI_BR_FPCLCK_DIV_128, MSPI_BR_FPCLCK_DIV_256
} MSPI_BaudRate_t;

/* ========================================================================= *
 *                           CONFIGURATION STRUCT                            *
 * ========================================================================= */

/**
 * @brief SPI Configuration Struct
 * @note  Members are declared 'const' to enforce immutability. Once initialized,
 *        the configuration cannot be accidentally modified at runtime. This is a
 *        critical safety feature in embedded systems.
 */
typedef struct {
    const MSPI_State_t      State;      /**< Enable/Disable at init */
    const MSPI_Mode_t       Mode;       /**< Master or Slave */
    const MSPI_CPOL_t       CPOL;       /**< Clock Polarity setting */
    const MSPI_CPHA_t       CPHA;       /**< Clock Phase setting */
    const MSPI_DFF_t        DFF;        /**< 8-bit or 16-bit data frame */
    const MSPI_BaudRate_t   BaudRate;   /**< Clock prescaler (ignored in Slave mode) */
    const u8                LSBFirst;   /**< 0 = MSB First, 1 = LSB First */
} MSPI_Config_t;

/* ========================================================================= *
 *                           FUNCTION PROTOTYPES                             *
 * ========================================================================= */

/**
 * @brief  Initializes a specific SPI peripheral using the hidden static config.
 * @param  Copy_SPIx: Pointer to the SPI instance (MSPI1, MSPI2, MSPI3).
 * @note   The Application layer MUST enable RCC clocks and configure GPIO AF
 *         pins BEFORE calling this function. Strict layer separation.
 */
void MSPI_voidInit(volatile SPI_t *Copy_SPIx);

/** @brief Enables the SPI peripheral at runtime. */
void MSPI_voidEnable(volatile SPI_t *Copy_SPIx);

/** @brief Disables the SPI peripheral at runtime. */
void MSPI_voidDisable(volatile SPI_t *Copy_SPIx);

/**
 * @brief  Modifies the SPI Mode (Master/Slave) at runtime.
 * @note   Safely disables SPI before changing MSTR bit, then re-enables.
 */
void MSPI_voidSetMode(volatile SPI_t *Copy_SPIx, MSPI_Mode_t Copy_Mode);

/**
 * @brief  Sends/Receives an 8-bit byte simultaneously (Blocking mode).
 * @note   WARNING: MSYSTICK_voidInit() MUST be called before using this function.
 *         Includes an RXNE flush to prevent reading stale/aborted data.
 *         Includes SPE safety check to prevent hanging if SPI is disabled.
 * @retval The received 8-bit byte.
 */
u8 MSPI_u8SendReceiveByteBlocking(volatile SPI_t *Copy_SPIx, u8 Copy_u8Data);

/**
 * @brief  Sends/Receives a 16-bit word simultaneously (Blocking mode).
 * @note   Ensure DFF is set to MSPI_DFF_16BIT in the config before using.
 * @retval The received 16-bit word.
 */
u16 MSPI_u16SendReceiveByteBlocking(volatile SPI_t *Copy_SPIx, u16 Copy_u16Data);

/**
 * @brief  Sends a buffer of data using interrupts (Non-Blocking mode).
 * @param  Copy_pvoidBuffer: Pointer to the data buffer (void* to support both u8 and u16 arrays).
 * @param  Copy_u16Length: Number of elements to send.
 * @param  Copy_pfNotification: Callback executed when the entire buffer is transmitted.
 */
void MSPI_voidSendBufferNonBlocking(volatile SPI_t *Copy_SPIx, void *Copy_pvoidBuffer, u16 Copy_u16Length, void (*Copy_pfNotification)(void));

/**
 * @brief  Receives a buffer of data using interrupts (Non-Blocking mode).
 * @param  Copy_pvoidBuffer: Pointer to the buffer where data will be stored.
 * @param  Copy_u16Length: Number of elements to receive.
 * @param  Copy_pfNotification: Callback executed when the entire buffer is received.
 */
void MSPI_voidReceiveBufferNonBlocking(volatile SPI_t *Copy_SPIx, void *Copy_pvoidBuffer, u16 Copy_u16Length, void (*Copy_pfNotification)(void));

/* Interrupt Service Routines (Linked from startup file) */
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void SPI3_IRQHandler(void);

#endif /* MSPI_INTERFACE_H_ */
