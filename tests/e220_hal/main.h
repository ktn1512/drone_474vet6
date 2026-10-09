#ifndef E220_HOST_HAL_MAIN_H
#define E220_HOST_HAL_MAIN_H

#include <stdint.h>

/* Minimal STM32 HAL surface used by the E220 transport. */
typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum {
    HAL_UART_STATE_RESET = 0x00U,
    HAL_UART_STATE_READY = 0x20U,
    HAL_UART_STATE_BUSY = 0x24U,
    HAL_UART_STATE_BUSY_TX = 0x21U,
    HAL_UART_STATE_BUSY_RX = 0x22U,
    HAL_UART_STATE_BUSY_TX_RX = 0x23U,
    HAL_UART_STATE_TIMEOUT = 0xA0U,
    HAL_UART_STATE_ERROR = 0xE0U
} HAL_UART_StateTypeDef;

typedef struct {
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
    uint32_t HwFlowCtl;
    uint32_t OverSampling;
    uint32_t OneBitSampling;
    uint32_t ClockPrescaler;
} UART_InitTypeDef;

typedef struct {
    void *Instance;
    UART_InitTypeDef Init;
    volatile HAL_UART_StateTypeDef gState;
    volatile HAL_UART_StateTypeDef RxState;
    volatile uint32_t ErrorCode;
} UART_HandleTypeDef;

#define UART_WORDLENGTH_8B 0x00000000U
#define UART_WORDLENGTH_9B 0x00001000U
#define UART_STOPBITS_1 0x00000000U
#define UART_STOPBITS_2 0x00002000U
#define UART_PARITY_NONE 0x00000000U
#define UART_PARITY_EVEN 0x00000400U
#define UART_PARITY_ODD 0x00000600U
#define UART_HWCONTROL_NONE 0x00000000U
#define UART_HWCONTROL_RTS_CTS 0x00000300U
#define UART_MODE_TX_RX 0x0000000CU
#define HAL_UART_ERROR_NONE 0x00000000U
#define HAL_UART_ERROR_ORE 0x00000008U
#define __DMB() ((void)0)

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                    uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                  const uint8_t *data, uint16_t len,
                                  uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart,
                                     const uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart);

#endif
