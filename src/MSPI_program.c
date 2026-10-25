/*
 * MSPI_program.c
 *
 *  Created on: Aug 15, 2026
 *      Author: dell
 *  Description: Implementation of the MCAL SPI driver.
 *               Incorporates architectural fixes:
 *               1. Hidden static configs (MEXTI-style architecture).
 *               2. RXNE flush using (void)Copy_SPIx->DR for MISRA compliance.
 *               3. Fixed ISR Trap by checking CR2 interrupt enable bits.
 *               4. Fixed Pointer Arithmetic Trap by using void* and DataSize.
 *               5. Added SPE safety check in blocking functions.
 *               6. Adjusted to the typed MNVIC API (MNVIC_EnableInterrupt +
 *                  status check; raw positions 35/36/51 retired).
 */

#include "BIT_MATH.h"
#include "STD_TYPES.h"

#include <MCAL/SYSTICK/MSYSTICK_interface.h>

#include "MCAL/SPI/MSPI_interface.h"
#include "MCAL/SPI/MSPI_private.h"
#include "MCAL/SPI/MSPI_config.h"  // Include the static configs (ONLY included here!)

#include "MNVIC_interface.h"

/* ========================================================================= *
 *                        PRIVATE MACROS & TYPES                             *
 * ========================================================================= */

#define MSPI_TIMEOUT_MS   1000UL

/**
 * @brief Non-Blocking Transfer State Struct
 *        Tracks the progress of an interrupt-driven buffer transfer.
 *        FIX: Uses void* and DataSize to prevent pointer arithmetic corruption.
 */
typedef struct {
    void *Buffer;           /**< Pointer to the data buffer (void* supports u8 and u16) */
    u16 Length;             /**< Total number of elements to transfer */
    u16 Index;              /**< Current element index */
    u8  DataSize;           /**< Element size in bytes: 1 for 8-bit, 2 for 16-bit */
    void (*Callback)(void); /**< Function to call when transfer is complete */
} MSPI_TransferState_t;

/* ========================================================================= *
 *                        PRIVATE GLOBAL VARIABLES                           *
 * ========================================================================= */
// One state struct per SPI instance for TX and RX
static MSPI_TransferState_t MSPI1_TxState = {NULL, 0, 0, 0, NULL};
static MSPI_TransferState_t MSPI1_RxState = {NULL, 0, 0, 0, NULL};
static MSPI_TransferState_t MSPI2_TxState = {NULL, 0, 0, 0, NULL};
static MSPI_TransferState_t MSPI2_RxState = {NULL, 0, 0, 0, NULL};
static MSPI_TransferState_t MSPI3_TxState = {NULL, 0, 0, 0, NULL};
static MSPI_TransferState_t MSPI3_RxState = {NULL, 0, 0, 0, NULL};

/* ========================================================================= *
 *                        PRIVATE FUNCTION PROTOTYPES                        *
 * ========================================================================= */
static void MSPI_voidEnableTXInterrupt(volatile SPI_t *Copy_SPIx);
static void MSPI_voidDisableTXInterrupt(volatile SPI_t *Copy_SPIx);
static void MSPI_voidEnableRXInterrupt(volatile SPI_t *Copy_SPIx);
static void MSPI_voidDisableRXInterrupt(volatile SPI_t *Copy_SPIx);
static u8   MSPI_u8WaitForFlagTimeout(volatile SPI_t *Copy_SPIx, u8 Copy_u8Flag);


/* ========================================================================= *
 *                        PUBLIC FUNCTIONS                                   *
 * ========================================================================= */

/**
 * @brief  Initializes a specific SPI peripheral using the hidden static config.
 */
void MSPI_voidInit(volatile SPI_t *Copy_SPIx) {
    if (Copy_SPIx == NULL) return;

    // Pointer to hold the address of the correct static config
    const MSPI_Config_t *Local_Config = NULL;

    // Select the static config based on the passed instance
    if (Copy_SPIx == MSPI1) {
        Local_Config = &MSPI1_Config;
    } else if (Copy_SPIx == MSPI2) {
        Local_Config = &MSPI2_Config;
    } else if (Copy_SPIx == MSPI3) {
        Local_Config = &MSPI3_Config;
    } else {
        return; // Invalid instance
    }

    // 1. Disable SPI before configuration (SPE = 0) - Mandatory!
    // Writing to CR1 while SPE=1 is ignored by hardware.
    CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE);

    // 2. Configure Baud Rate (Bits 5:3)
    Copy_SPIx->CR1 &= ~(0b111 << MSPI_CR1_BR);
    Copy_SPIx->CR1 |= (Local_Config->BaudRate << MSPI_CR1_BR);

    // 3. Clock Polarity (CPOL)
    if (Local_Config->CPOL == MSPI_CPOL_HIGH) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_CPOL);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_CPOL);

    // 4. Clock Phase (CPHA)
    if (Local_Config->CPHA == MSPI_CPHA_SECOND) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_CPHA);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_CPHA);

    // 5. Data Frame Format (DFF)
    if (Local_Config->DFF == MSPI_DFF_16BIT) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_DFF);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_DFF);

    // 6. Bit Order (LSBFIRST)
    if (Local_Config->LSBFirst == 1) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_LSBFIRST);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_LSBFIRST);

    // 7. Software Slave Management (SSM=1)
    // Prevents Master Mode Fault without needing hardware NSS pin.
    SET_BIT(Copy_SPIx->CR1, MSPI_CR1_SSM);

    // Set SSI based on mode: Master needs 1, Slave needs 0 to listen.
    if (Local_Config->Mode == MSPI_MASTER_MODE) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_SSI);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_SSI);

    // 8. Master/Slave Mode (MSTR)
    if (Local_Config->Mode == MSPI_MASTER_MODE) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_MSTR);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_MSTR);

    // 9. Enable or Disable
    if (Local_Config->State == MSPI_STATE_ENABLE) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE);
}

void MSPI_voidEnable(volatile SPI_t *Copy_SPIx)  { SET_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE); }
void MSPI_voidDisable(volatile SPI_t *Copy_SPIx) { CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE); }

void MSPI_voidSetMode(volatile SPI_t *Copy_SPIx, MSPI_Mode_t Copy_Mode) {
    // Must disable SPI before changing MSTR bit, then re-enable.
    CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE);
    if (Copy_Mode == MSPI_MASTER_MODE) SET_BIT(Copy_SPIx->CR1, MSPI_CR1_MSTR);
    else CLR_BIT(Copy_SPIx->CR1, MSPI_CR1_MSTR);
    SET_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE);
}


/* ========================================================================= *
 *                        BLOCKING FUNCTIONS                                 *
 * ========================================================================= */

u8 MSPI_u8SendReceiveByteBlocking(volatile SPI_t *Copy_SPIx, u8 Copy_u8Data) {
    // ROBUSTNESS TWEAK: Safety check. If SPI is disabled, don't hang waiting for flags.
    if (GET_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE) == 0) return 0;

    u8 Local_u8ReceivedData = 0;

    if (MSPI_u8WaitForFlagTimeout(Copy_SPIx, MSPI_SR_TXE) == 1) {

        // ROBUSTNESS TWEAK: MISRA-compliant dummy read to flush RXNE.
        // If a previous transfer was aborted, RXNE might be stuck at 1.
        // Casting to void prevents aggressive optimizers (-O2/-O3) from deleting
        // the read operation because the resulting value is not used.
        if (GET_BIT(Copy_SPIx->SR, MSPI_SR_RXNE) == 1) {
            (void)Copy_SPIx->DR;
        }

        Copy_SPIx->DR = Copy_u8Data; // Write data to start transmission

        if (MSPI_u8WaitForFlagTimeout(Copy_SPIx, MSPI_SR_RXNE) == 1) {
            Local_u8ReceivedData = (u8)Copy_SPIx->DR; // Read received data
        } else {
            Local_u8ReceivedData = 0xFF; // RX Timeout Error
        }
    } else {
        Local_u8ReceivedData = 0xFE; // TX Timeout Error
    }
    return Local_u8ReceivedData;
}

u16 MSPI_u16SendReceiveByteBlocking(volatile SPI_t *Copy_SPIx, u16 Copy_u16Data) {
    // ROBUSTNESS TWEAK: Safety check.
    if (GET_BIT(Copy_SPIx->CR1, MSPI_CR1_SPE) == 0) return 0;

    u16 Local_u16ReceivedData = 0;

    if (MSPI_u8WaitForFlagTimeout(Copy_SPIx, MSPI_SR_TXE) == 1) {

        // ROBUSTNESS TWEAK: MISRA-compliant flush
        if (GET_BIT(Copy_SPIx->SR, MSPI_SR_RXNE) == 1) {
            (void)Copy_SPIx->DR;
        }

        Copy_SPIx->DR = Copy_u16Data;

        if (MSPI_u8WaitForFlagTimeout(Copy_SPIx, MSPI_SR_RXNE) == 1) {
            Local_u16ReceivedData = (u16)Copy_SPIx->DR;
        } else {
            Local_u16ReceivedData = 0xFFFF; // RX Timeout
        }
    } else {
        Local_u16ReceivedData = 0xFFFE; // TX Timeout
    }
    return Local_u16ReceivedData;
}


/* ========================================================================= *
 *                        NON-BLOCKING FUNCTIONS                             *
 * ========================================================================= */

/* NVIC NOTE (new MNVIC driver):
 * This driver enables its own NVIC IRQ internally (below) — but it does NOT
 * set a priority. Until the application sets one, the SPI IRQ sits at the
 * IPR reset value 0 = HIGHEST priority, and will pre-empt the EXTI button
 * and USART1 (ESP8266) handlers mid-transfer. Per MNVIC's boot-order
 * contract (priority BEFORE enable), set it once at boot:
 *      MNVIC_Init();
 *      MNVIC_SetPriority(MNVIC_SPI1, 4, 0);   // suggested plan: SPI = 4
 * IPR is writable while the IRQ is still disabled, so app-init ordering
 * works: the MNVIC_EnableInterrupt() calls below only ever turn the line
 * on — they never need to know the priority.
 * (Re-arming a transfer re-enables an already-enabled IRQ: ISER is
 * write-1-to-set, so the repeat write is harmless.)
 * ========================================================================= */

void MSPI_voidSendBufferNonBlocking(volatile SPI_t *Copy_SPIx, void *Copy_pvoidBuffer, u16 Copy_u16Length, void (*Copy_pfNotification)(void)) {
    if (Copy_pvoidBuffer == NULL || Copy_u16Length == 0) return;

    MSPI_TransferState_t *Local_State = NULL;
    MNVIC_ReturnType_t Local_EnumNVICStatus = MNVIC_OK;

    // NVIC ADJUSTMENT: typed MNVIC_IRQ_t enum replaces the raw position
    // numbers (35 / 36 / 51) — the "is SPI1 35 or 36?" bug class is now
    // impossible to write, and the enum values are _Static_assert-pinned
    // to startup_stm32f401xc.s in MNVIC_program.c.
    if (Copy_SPIx == MSPI1)        { Local_State = &MSPI1_TxState; Local_EnumNVICStatus = MNVIC_EnableInterrupt(MNVIC_SPI1); }
    else if (Copy_SPIx == MSPI2)   { Local_State = &MSPI2_TxState; Local_EnumNVICStatus = MNVIC_EnableInterrupt(MNVIC_SPI2); }
    else if (Copy_SPIx == MSPI3)   { Local_State = &MSPI3_TxState; Local_EnumNVICStatus = MNVIC_EnableInterrupt(MNVIC_SPI3); }
    else return;

    // NVIC ADJUSTMENT: if the enable failed, the ISR can never run and this
    // transfer would hang forever with zero diagnostics — abort BEFORE
    // arming any transfer state.
    if (Local_EnumNVICStatus != MNVIC_OK) return;

    // Setup transfer state
    Local_State->Buffer = Copy_pvoidBuffer;
    Local_State->Length = Copy_u16Length;
    Local_State->Index = 0;
    Local_State->Callback = Copy_pfNotification;

    // CRITICAL FIX: Determine DataSize based on DFF bit to prevent pointer arithmetic corruption.
    // If DFF is 1 (16-bit), DataSize is 2. If DFF is 0 (8-bit), DataSize is 1.
    Local_State->DataSize = (GET_BIT(Copy_SPIx->CR1, MSPI_CR1_DFF) == 1) ? 2 : 1;

    // Write first byte to shift register. TXE flag will trigger the ISR.
    // Cast the void pointer correctly based on DataSize.
    if (Local_State->DataSize == 1) {
        Copy_SPIx->DR = ((u8*)Local_State->Buffer)[Local_State->Index++];
    } else {
        Copy_SPIx->DR = ((u16*)Local_State->Buffer)[Local_State->Index++];
    }

    MSPI_voidEnableTXInterrupt(Copy_SPIx);
}

void MSPI_voidReceiveBufferNonBlocking(volatile SPI_t *Copy_SPIx, void *Copy_pvoidBuffer, u16 Copy_u16Length, void (*Copy_pfNotification)(void)) {
    if (Copy_pvoidBuffer == NULL || Copy_u16Length == 0) return;

    MSPI_TransferState_t *Local_State = NULL;
    MNVIC_ReturnType_t Local_EnumNVICStatus = MNVIC_OK;

    // NVIC ADJUSTMENT: typed enum, same as the TX path above.
    if (Copy_SPIx == MSPI1)        { Local_State = &MSPI1_RxState; Local_EnumNVICStatus = MNVIC_EnableInterrupt(MNVIC_SPI1); }
    else if (Copy_SPIx == MSPI2)   { Local_State = &MSPI2_RxState; Local_EnumNVICStatus = MNVIC_EnableInterrupt(MNVIC_SPI2); }
    else if (Copy_SPIx == MSPI3)   { Local_State = &MSPI3_RxState; Local_EnumNVICStatus = MNVIC_EnableInterrupt(MNVIC_SPI3); }
    else return;

    // NVIC ADJUSTMENT: abort before arming state — see TX path rationale.
    if (Local_EnumNVICStatus != MNVIC_OK) return;

    Local_State->Buffer = Copy_pvoidBuffer;
    Local_State->Length = Copy_u16Length;
    Local_State->Index = 0;
    Local_State->Callback = Copy_pfNotification;

    Local_State->DataSize = (GET_BIT(Copy_SPIx->CR1, MSPI_CR1_DFF) == 1) ? 2 : 1;

    MSPI_voidEnableRXInterrupt(Copy_SPIx);

    // Master must generate the clock. Send dummy byte to start reception.
    if (GET_BIT(Copy_SPIx->CR1, MSPI_CR1_MSTR) == 1) {
        Copy_SPIx->DR = 0xFF;
    }
}


/* ========================================================================= *
 *                        INTERRUPT SERVICE ROUTINES                         *
 * ========================================================================= */

void SPI1_IRQHandler(void) {
    // BUG FIX: The ISR Trap.
    // TXE is almost always 1 when idle. If an RX interrupt fires, checking only
    // SR would falsely trigger the TX logic. We MUST verify the CR2 enable bit.

    // Handle Transmit Complete
    if ((GET_BIT(MSPI1->SR, MSPI_SR_TXE) == 1) && (GET_BIT(MSPI1->CR2, MSPI_CR2_TXEIE) == 1)) {
        if (MSPI1_TxState.Index < MSPI1_TxState.Length) {
            // CRITICAL FIX: Cast pointer based on DataSize to prevent skipping bytes.
            if (MSPI1_TxState.DataSize == 1) {
                MSPI1->DR = ((u8*)MSPI1_TxState.Buffer)[MSPI1_TxState.Index++];
            } else {
                MSPI1->DR = ((u16*)MSPI1_TxState.Buffer)[MSPI1_TxState.Index++];
            }
        } else {
            // Buffer finished. Disable interrupt and notify application.
            MSPI_voidDisableTXInterrupt(MSPI1);
            if (MSPI1_TxState.Callback != NULL) { MSPI1_TxState.Callback(); }
        }
    }

    // Handle Receive Complete
    if ((GET_BIT(MSPI1->SR, MSPI_SR_RXNE) == 1) && (GET_BIT(MSPI1->CR2, MSPI_CR2_RXNEIE) == 1)) {
        // CRITICAL FIX: Cast pointer based on DataSize.
        if (MSPI1_RxState.DataSize == 1) {
            ((u8*)MSPI1_RxState.Buffer)[MSPI1_RxState.Index++] = MSPI1->DR;
        } else {
            ((u16*)MSPI1_RxState.Buffer)[MSPI1_RxState.Index++] = MSPI1->DR;
        }

        if (MSPI1_RxState.Index < MSPI1_RxState.Length) {
            // Need more data. If Master, send dummy byte to generate next clock.
            if (GET_BIT(MSPI1->CR1, MSPI_CR1_MSTR) == 1) { MSPI1->DR = 0xFF; }
        } else {
            // Buffer finished. Disable interrupt and notify application.
            MSPI_voidDisableRXInterrupt(MSPI1);
            if (MSPI1_RxState.Callback != NULL) { MSPI1_RxState.Callback(); }
        }
    }
}

void SPI2_IRQHandler(void) {
    if ((GET_BIT(MSPI2->SR, MSPI_SR_TXE) == 1) && (GET_BIT(MSPI2->CR2, MSPI_CR2_TXEIE) == 1)) {
        if (MSPI2_TxState.Index < MSPI2_TxState.Length) {
            if (MSPI2_TxState.DataSize == 1) { MSPI2->DR = ((u8*)MSPI2_TxState.Buffer)[MSPI2_TxState.Index++]; }
            else { MSPI2->DR = ((u16*)MSPI2_TxState.Buffer)[MSPI2_TxState.Index++]; }
        } else { MSPI_voidDisableTXInterrupt(MSPI2); if (MSPI2_TxState.Callback != NULL) { MSPI2_TxState.Callback(); } }
    }
    if ((GET_BIT(MSPI2->SR, MSPI_SR_RXNE) == 1) && (GET_BIT(MSPI2->CR2, MSPI_CR2_RXNEIE) == 1)) {
        if (MSPI2_RxState.DataSize == 1) { ((u8*)MSPI2_RxState.Buffer)[MSPI2_RxState.Index++] = MSPI2->DR; }
        else { ((u16*)MSPI2_RxState.Buffer)[MSPI2_RxState.Index++] = MSPI2->DR; }

        if (MSPI2_RxState.Index < MSPI2_RxState.Length) { if (GET_BIT(MSPI2->CR1, MSPI_CR1_MSTR) == 1) { MSPI2->DR = 0xFF; } }
        else { MSPI_voidDisableRXInterrupt(MSPI2); if (MSPI2_RxState.Callback != NULL) { MSPI2_RxState.Callback(); } }
    }
}

void SPI3_IRQHandler(void) {
    if ((GET_BIT(MSPI3->SR, MSPI_SR_TXE) == 1) && (GET_BIT(MSPI3->CR2, MSPI_CR2_TXEIE) == 1)) {
        if (MSPI3_TxState.Index < MSPI3_TxState.Length) {
            if (MSPI3_TxState.DataSize == 1) { MSPI3->DR = ((u8*)MSPI3_TxState.Buffer)[MSPI3_TxState.Index++]; }
            else { MSPI3->DR = ((u16*)MSPI3_TxState.Buffer)[MSPI3_TxState.Index++]; }
        } else { MSPI_voidDisableTXInterrupt(MSPI3); if (MSPI3_TxState.Callback != NULL) { MSPI3_TxState.Callback(); } }
    }
    if ((GET_BIT(MSPI3->SR, MSPI_SR_RXNE) == 1) && (GET_BIT(MSPI3->CR2, MSPI_CR2_RXNEIE) == 1)) {
        if (MSPI3_RxState.DataSize == 1) { ((u8*)MSPI3_RxState.Buffer)[MSPI3_RxState.Index++] = MSPI3->DR; }
        else { ((u16*)MSPI3_RxState.Buffer)[MSPI3_RxState.Index++] = MSPI3->DR; }

        if (MSPI3_RxState.Index < MSPI3_RxState.Length) { if (GET_BIT(MSPI3->CR1, MSPI_CR1_MSTR) == 1) { MSPI3->DR = 0xFF; } }
        else { MSPI_voidDisableRXInterrupt(MSPI3); if (MSPI3_RxState.Callback != NULL) { MSPI3_RxState.Callback(); } }
    }
}


/* ========================================================================= *
 *                        PRIVATE FUNCTIONS                                  *
 * ========================================================================= */

static void MSPI_voidEnableTXInterrupt(volatile SPI_t *Copy_SPIx)  { SET_BIT(Copy_SPIx->CR2, MSPI_CR2_TXEIE); }
static void MSPI_voidDisableTXInterrupt(volatile SPI_t *Copy_SPIx) { CLR_BIT(Copy_SPIx->CR2, MSPI_CR2_TXEIE); }
static void MSPI_voidEnableRXInterrupt(volatile SPI_t *Copy_SPIx)  { SET_BIT(Copy_SPIx->CR2, MSPI_CR2_RXNEIE); }
static void MSPI_voidDisableRXInterrupt(volatile SPI_t *Copy_SPIx) { CLR_BIT(Copy_SPIx->CR2, MSPI_CR2_RXNEIE); }

/**
 * @brief Private function to wait for an SPI flag with a timeout.
 * @note  Depends on MSYSTICK driver. If SysTick is not running, this will hang.
 */
static u8 MSPI_u8WaitForFlagTimeout(volatile SPI_t *Copy_SPIx, u8 Copy_u8Flag) {
    u32 Local_u32StartTime = MSYSTICK_GetElapsed_ms();
    while (GET_BIT(Copy_SPIx->SR, Copy_u8Flag) == 0) {
        if ((MSYSTICK_GetElapsed_ms() - Local_u32StartTime) >= MSPI_TIMEOUT_MS) {
            return 0; // Timeout occurred
        }
    }
    return 1; // Flag set successfully
}
