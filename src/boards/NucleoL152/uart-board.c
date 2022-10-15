/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "stm32l1xx.h"
#include "utilities.h"
#include "board.h"
#include "sysIrqHandlers.h"
#include "uart-board.h"

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT                       10

#define USART_RX_FIFO_SIZE  2048
#define USART_TX_FIFO_SIZE  2048

typedef struct {
    UART_HandleTypeDef UartHandle;
    uint8_t RxData;
    uint8_t TxData;
    uint8_t rxBuffer[USART_RX_FIFO_SIZE];
    uint8_t txBuffer[USART_TX_FIFO_SIZE];
} UartContext_t;

UartContext_t UartContext[UART_2 + 1]; // indexed by UART_n

extern Uart_t ConsoleUart;  // UART_2
extern Uart_t GpsUart;     // UART_1

void UartMcuInit(Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx)
{

    obj->UartId = uartId;

    if (obj->UartId == UART_1) {
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_ENABLE();

        GpioInit(&obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART1);
        GpioInit(&obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART1);
    } else if (obj->UartId == UART_2) {
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        __HAL_RCC_USART2_CLK_ENABLE();

        GpioInit(&obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART2);
        GpioInit(&obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF7_USART2);
    }
}

void
UartMcuConfig(Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength,
  StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl)
{

    switch (obj->UartId) {
    case UART_1:
        UartContext[obj->UartId].UartHandle.Instance = USART1;
        break;
    case UART_2:
        UartContext[obj->UartId].UartHandle.Instance = USART2;
        break;
    default:
        assert_param( LMN_STATUS_ERROR );
        break;
    }

    UartContext[obj->UartId].UartHandle.Init.BaudRate = baudrate;

    switch (mode) {
    case TX_ONLY:
        if (obj->FifoTx.Data == NULL) {
            assert_param(LMN_STATUS_ERROR);
        }
        UartContext[obj->UartId].UartHandle.Init.Mode = UART_MODE_TX;
        break;
    case RX_ONLY:
        if (obj->FifoRx.Data == NULL) {
            assert_param(LMN_STATUS_ERROR);
        }
        UartContext[obj->UartId].UartHandle.Init.Mode = UART_MODE_RX;
        break;
    case RX_TX:
        if ((obj->FifoTx.Data == NULL) || (obj->FifoRx.Data == NULL)) {
            assert_param(LMN_STATUS_ERROR);
        }
        UartContext[obj->UartId].UartHandle.Init.Mode = UART_MODE_TX_RX;
        break;
    default:
        assert_param( LMN_STATUS_ERROR );
        break;
    }

    switch(wordLength) {
    case UART_8_BIT:
        UartContext[obj->UartId].UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case UART_9_BIT:
        UartContext[obj->UartId].UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        assert_param(LMN_STATUS_ERROR);
        break;        
    }

    switch (stopBits) {
    case UART_2_STOP_BIT:
        UartContext[obj->UartId].UartHandle.Init.StopBits = UART_STOPBITS_2;
        break;
    case UART_1_STOP_BIT:
        UartContext[obj->UartId].UartHandle.Init.StopBits = UART_STOPBITS_1;
        break;
    default:
        assert_param(LMN_STATUS_ERROR);
        break;
    }

    switch (parity) {
    case NO_PARITY:
        UartContext[obj->UartId].UartHandle.Init.Parity = UART_PARITY_NONE;
        break;
    case EVEN_PARITY:
        UartContext[obj->UartId].UartHandle.Init.Parity = UART_PARITY_EVEN;
        break;
    default:
        UartContext[obj->UartId].UartHandle.Init.Parity = UART_PARITY_ODD;
        break;
    }

    switch (flowCtrl) {
    case NO_FLOW_CTRL:
        UartContext[obj->UartId].UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        break;
    case RTS_FLOW_CTRL:
        UartContext[obj->UartId].UartHandle.Init.HwFlowCtl = UART_HWCONTROL_RTS;    
        break;
    case CTS_FLOW_CTRL:
        UartContext[obj->UartId].UartHandle.Init.HwFlowCtl = UART_HWCONTROL_CTS;    
        break;
    case RTS_CTS_FLOW_CTRL:
        UartContext[obj->UartId].UartHandle.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;    
        break;
    default:
        assert_param(LMN_STATUS_ERROR);
        break;
    }

    UartContext[obj->UartId].UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    FifoInit(&obj->FifoRx, (uint8_t *) &UartContext[obj->UartId].rxBuffer, USART_RX_FIFO_SIZE);
    FifoInit(&obj->FifoTx, (uint8_t *) &UartContext[obj->UartId].txBuffer, USART_TX_FIFO_SIZE);

    if (HAL_UART_Init(&UartContext[obj->UartId].UartHandle) != HAL_OK) {
        assert_param(FAIL);
    }

    switch (obj->UartId) {
    case UART_1:
        HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        break;
    case UART_2:
        HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        break;
    default:
        assert_param(LMN_STATUS_ERROR);
        break;
    }

    /* Enable the UART Data Register not empty Interrupt */
    HAL_UART_Receive_IT(&UartContext[obj->UartId].UartHandle, &UartContext[obj->UartId].RxData, 1);
}

void UartMcuDeInit(Uart_t *obj)
{

    switch (obj->UartId) {
    case UART_1:
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_DISABLE();
        break;

    case UART_2:
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        __HAL_RCC_USART2_CLK_DISABLE();
        break;

    default:
        assert_param(LMN_STATUS_ERROR);
        break;
    }

    GpioInit(&obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0);
}

uint8_t
UartMcuPutChar(Uart_t *obj, uint8_t data)
{
    CRITICAL_SECTION_BEGIN();
    UartContext[obj->UartId].TxData = data;

    if (IsFifoFull(&obj->FifoTx) == false) {
        FifoPush(&obj->FifoTx, UartContext[obj->UartId].TxData);

        // Trig UART Tx interrupt to start sending the FIFO contents.
        __HAL_UART_ENABLE_IT(&UartContext[obj->UartId].UartHandle, UART_IT_TC);

        CRITICAL_SECTION_END();
        return 0; // OK
    }
    CRITICAL_SECTION_END();
    return 1; // Busy
}

uint8_t
UartMcuGetChar(Uart_t *obj, uint8_t *data)
{

    CRITICAL_SECTION_BEGIN();

    if (IsFifoEmpty(&obj->FifoRx) == false) {
        *data = FifoPop(&obj->FifoRx);
        CRITICAL_SECTION_END();
        return 0;
    }
    CRITICAL_SECTION_END();
    return 1;
}

uint8_t
UartMcuPutBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size)
{
    uint8_t retryCount;
    uint16_t i;

    for (i = 0; i < size; i++) {
        retryCount = 0;
        while (UartPutChar(obj, buffer[i]) != 0) {
            retryCount++;

            // Exit if something goes terribly wrong
            if (retryCount > TX_BUFFER_RETRY_COUNT) {
                return 1; // Error
            }
        }
    }
    return 0; // OK
}

uint8_t
UartMcuGetBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes)
{
    uint16_t localSize = 0;

    while (localSize < size) {
        if (UartGetChar(obj, buffer + localSize) == 0) {
            localSize++;
        } else {
            break;
        }
    }

    *nbReadBytes = localSize;

    if (localSize == 0) {
        return 1; // Empty
    }
    return 0; // OK
}

void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
    Uart_t *uart;
    UartId_t uartId;

    if (handle == &UartContext[UART_1].UartHandle) {
        uart = &GpsUart;
        uartId = UART_1;
    } else if (handle == &UartContext[UART_2].UartHandle) {
        uart = &ConsoleUart;
        uartId = UART_2;
    } else {
        // Unknown UART peripheral skip processing
        return;
    }

    if (IsFifoEmpty(&uart->FifoTx) == false) {
        UartContext[uartId].TxData = FifoPop(&uart->FifoTx);
        //  Write one byte to the transmit data register
        HAL_UART_Transmit_IT(&UartContext[uartId].UartHandle, &UartContext[uartId].TxData, 1);
    }

    if (uart->IrqNotify != NULL) {
        uart->IrqNotify(UART_NOTIFY_TX);
    }
}

void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
    Uart_t *uart;
    UartId_t uartId;

    if (handle == &UartContext[UART_1].UartHandle) {
        uart = &GpsUart;
        uartId = UART_1;
    } else if (handle == &UartContext[UART_2].UartHandle) {
        uart = &ConsoleUart;
        uartId = UART_2;
    } else {
        // Unknown UART peripheral skip processing
        return;
    }

    if (IsFifoFull(&uart->FifoRx) == false) {
        // Read one byte from the receive data register
        FifoPush(&uart->FifoRx, UartContext[uartId].RxData);
    }

    if (uart->IrqNotify != NULL) {
        uart->IrqNotify(UART_NOTIFY_RX);
    }

    HAL_UART_Receive_IT(&UartContext[uartId].UartHandle, &UartContext[uartId].RxData, 1);
}

void
HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    UartId_t uartId;

    if (handle == &UartContext[UART_1].UartHandle) {
        uartId = UART_1;
    } else if (handle == &UartContext[UART_2].UartHandle) {
        uartId = UART_2;
    } else {
        // Unknown UART peripheral skip processing
        return;
    }

    HAL_UART_Receive_IT(&UartContext[uartId].UartHandle, &UartContext[uartId].RxData, 1);
}

void
USART1_IRQHandler(void)
{

    HAL_UART_IRQHandler(&UartContext[UART_1].UartHandle);
}

void
USART2_IRQHandler(void)
{

    HAL_UART_IRQHandler(&UartContext[UART_2].UartHandle);
}
