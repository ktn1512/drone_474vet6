#ifndef E220_H
#define E220_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* E220 transparent mode: M0=M1=0, RX by UART 1-byte interrupt,
 * every TX uses USART2 DMA normal mode (never polling or UART TX-IT).
 * HAL_OK means accepted by STM32 HAL, NOT delivered over LoRa.
 */
#define E220_RX_BUFFER_SIZE       256U
#define E220_MAX_PACKET_SIZE      200U

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    uint8_t rx_buffer[E220_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_overflow_count;
    volatile uint32_t rx_total_count;
    volatile uint32_t uart_error_count;
    volatile HAL_StatusTypeDef rx_status;

    /* DMA reads from THIS persistent buffer; caller may use stack memory. */
    uint8_t tx_dma_buffer[E220_MAX_PACKET_SIZE];
    volatile uint8_t tx_active;
    volatile uint32_t tx_start_count;
    volatile uint32_t tx_done_count;
    volatile uint32_t tx_busy_count;
    volatile uint32_t tx_error_count;
} E220_t;

/* USART must already be initialized, and huart->hdmatx linked to TX DMA. */
HAL_StatusTypeDef E220_Init(E220_t *e220, UART_HandleTypeDef *huart);

/* Safe for stack or temporary caller buffers: driver copies into tx_dma_buffer. */
HAL_StatusTypeDef E220_Send_DMA(E220_t *e220, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef E220_SendString_DMA(E220_t *e220, const char *str);

/* Compatibility with older main/drone_link modules: ALWAYS use TX DMA. */
HAL_StatusTypeDef E220_Send(E220_t *e220, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef E220_SendString(E220_t *e220, const char *str);
HAL_StatusTypeDef E220_Send_IT(E220_t *e220, const uint8_t *data, uint16_t len);

/* Call from main.c HAL_UART_* callbacks for the E220 USART only. */
void E220_RxCallback(E220_t *e220);
void E220_TxCpltCallback(E220_t *e220);
void E220_ErrorCallback(E220_t *e220);

uint16_t E220_Available(E220_t *e220);
uint8_t E220_ReadByte(E220_t *e220);
uint16_t E220_Read(E220_t *e220, uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif
#endif
