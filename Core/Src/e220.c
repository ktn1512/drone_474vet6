#include "e220.h"
#include <stddef.h>
#include <string.h>

static int valid_handle(const E220_t *e) {
    return e != NULL && e->huart != NULL && e->huart->Instance != NULL &&
           e->huart->Init.BaudRate > 0U;
}

HAL_StatusTypeDef E220_Init(E220_t *e, UART_HandleTypeDef *uart) {
    if (e == NULL || uart == NULL || uart->Instance == NULL ||
        uart->hdmatx == NULL ||
        uart->Init.BaudRate == 0U ||
        uart->Init.WordLength != UART_WORDLENGTH_8B ||
        uart->Init.StopBits != UART_STOPBITS_1 ||
        uart->Init.Parity != UART_PARITY_NONE ||
        uart->Init.HwFlowCtl != UART_HWCONTROL_NONE) return HAL_ERROR;

    memset(e, 0, sizeof(*e));
    e->huart = uart;
    e->rx_status = HAL_UART_Receive_IT(uart, &e->rx_byte, 1U);
    return e->rx_status;
}

HAL_StatusTypeDef E220_Send_DMA(E220_t *e, const uint8_t *data, uint16_t len) {
    if (!valid_handle(e) || e->huart->hdmatx == NULL || len == 0U ||
        len > E220_MAX_PACKET_SIZE || data == NULL) return HAL_ERROR;

    /* Protect the entire DMA buffer until the UART TC callback (NOT merely
     * until DMA channel has finished copying data to UART TDR). */
    if (e->tx_active || e->huart->gState != HAL_UART_STATE_READY) {
        ++e->tx_busy_count;
        return HAL_BUSY;
    }

    e->tx_active = 1U;
    memcpy(e->tx_dma_buffer, data, len);
    HAL_StatusTypeDef rc = HAL_UART_Transmit_DMA(e->huart, e->tx_dma_buffer, len);
    if (rc == HAL_OK) {
        ++e->tx_start_count;
    } else {
        e->tx_active = 0U;
        if (rc == HAL_BUSY) ++e->tx_busy_count;
        else ++e->tx_error_count;
    }
    return rc;
}

HAL_StatusTypeDef E220_SendString_DMA(E220_t *e, const char *str) {
    if (!str) return HAL_ERROR;
    size_t n = 0U;
    while (n <= E220_MAX_PACKET_SIZE && str[n] != '\0') ++n;
    if (n == 0U || n > E220_MAX_PACKET_SIZE) return HAL_ERROR;
    return E220_Send_DMA(e, (const uint8_t *)str, (uint16_t)n);
}

/* One DMA-only implementation for ALL sending APIs. */
HAL_StatusTypeDef E220_Send(E220_t *e, const uint8_t *data, uint16_t n) {
    return E220_Send_DMA(e, data, n);
}
HAL_StatusTypeDef E220_Send_IT(E220_t *e, const uint8_t *data, uint16_t n) {
    return E220_Send_DMA(e, data, n);
}
HAL_StatusTypeDef E220_SendString(E220_t *e, const char *s) {
    return E220_SendString_DMA(e, s);
}

/* Called at the end of physical USART shift-register transmission. */
void E220_TxCpltCallback(E220_t *e) {
    if (!valid_handle(e)) return;
    e->tx_active = 0U;
    ++e->tx_done_count;
}

void E220_RxCallback(E220_t *e) {
    if (!valid_handle(e)) return;
    ++e->rx_total_count;
    uint16_t next = (uint16_t)((e->rx_head + 1U) % E220_RX_BUFFER_SIZE);
    if (next != e->rx_tail) {
        e->rx_buffer[e->rx_head] = e->rx_byte;
        __DMB();
        e->rx_head = next;
    } else {
        ++e->rx_overflow_count;
    }
    e->rx_status = HAL_UART_Receive_IT(e->huart, &e->rx_byte, 1U);
}

void E220_ErrorCallback(E220_t *e) {
    if (!valid_handle(e)) return;
    ++e->uart_error_count;
    /* TX DMA is managed by the HAL. Do not abort TX merely on an RX error. */
    if (e->huart->gState == HAL_UART_STATE_READY && e->tx_active) {
        /* HAL may have stopped an erroneous DMA transfer without calling
         * TxCplt. Recover the guard only once USART TX is idle. */
        e->tx_active = 0U;
        ++e->tx_error_count;
    }
    e->rx_status = HAL_UART_AbortReceive(e->huart);
    if (e->rx_status == HAL_OK)
        e->rx_status = HAL_UART_Receive_IT(e->huart, &e->rx_byte, 1U);
}

uint16_t E220_Available(E220_t *e) {
    if (!valid_handle(e)) return 0U;
    return (uint16_t)((e->rx_head + E220_RX_BUFFER_SIZE - e->rx_tail) % E220_RX_BUFFER_SIZE);
}
uint8_t E220_ReadByte(E220_t *e) {
    if (!valid_handle(e) || e->rx_tail == e->rx_head) return 0U;
    uint16_t index = e->rx_tail;
    __DMB();
    uint8_t data = e->rx_buffer[index];
    __DMB();
    e->rx_tail = (uint16_t)((index + 1U) % E220_RX_BUFFER_SIZE);
    return data;
}
uint16_t E220_Read(E220_t *e, uint8_t *data, uint16_t max_len) {
    if (data == NULL) return 0U;
    uint16_t n = 0U;
    while (n < max_len && E220_Available(e) != 0U)
        data[n++] = E220_ReadByte(e);
    return n;
}
