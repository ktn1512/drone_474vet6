#include "e220.h"
#include <stddef.h>
#include <string.h>

static int valid_handle(const E220_t *e220)
{
    return e220 != NULL && e220->huart != NULL &&
           e220->huart->Instance != NULL && e220->huart->Init.BaudRate != 0U;
}

static int supported_baud(uint32_t baud)
{
    switch (baud) {
    case 1200U:
    case 2400U:
    case 4800U:
    case 9600U:
    case 19200U:
    case 38400U:
    case 57600U:
    case 115200U:
        return 1;
    default:
        return 0;
    }
}

HAL_StatusTypeDef E220_Init(E220_t *e220, UART_HandleTypeDef *huart)
{
    if (e220 == NULL || huart == NULL || huart->Instance == NULL ||
        !supported_baud(huart->Init.BaudRate) ||
        huart->Init.WordLength != UART_WORDLENGTH_8B ||
        huart->Init.StopBits != UART_STOPBITS_1 ||
        huart->Init.Parity != UART_PARITY_NONE ||
        huart->Init.HwFlowCtl != UART_HWCONTROL_NONE) {
        return HAL_ERROR;
    }

    memset(e220, 0, sizeof(*e220));
    e220->huart = huart;
    e220->rx_status = HAL_UART_Receive_IT(huart, &e220->rx_byte, 1U);
    return e220->rx_status;
}

static HAL_StatusTypeDef validate_send(E220_t *e220, const uint8_t *data,
                                       uint16_t len)
{
    if (!valid_handle(e220) || len > E220_MAX_PACKET_SIZE ||
        (data == NULL && len != 0U)) {
        return HAL_ERROR;
    }
    if (len != 0U && e220->huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }
    return HAL_OK;
}

HAL_StatusTypeDef E220_Send(E220_t *e220, const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = validate_send(e220, data, len);
    if (status != HAL_OK || len == 0U) {
        return status;
    }

    /* 8N1 = 10 UART bits/byte. A fixed 100 ms times out above 96 bytes
     * at 9600 baud; round up the wire time and add a HAL polling margin.
     */
    uint32_t baud = e220->huart->Init.BaudRate;
    uint32_t timeout = ((uint32_t)len * 10000U + baud - 1U) / baud +
                       E220_UART_TIMEOUT_MARGIN_MS;
    return HAL_UART_Transmit(e220->huart, data, len, timeout);
}

HAL_StatusTypeDef E220_SendString(E220_t *e220, const char *str)
{
    if (str == NULL) {
        return HAL_ERROR;
    }

    uint16_t len = 0U;
    while (len <= E220_MAX_PACKET_SIZE && str[len] != '\0') {
        ++len;
    }
    return E220_Send(e220, (const uint8_t *)str, len);
}

HAL_StatusTypeDef E220_Send_IT(E220_t *e220, const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = validate_send(e220, data, len);
    if (status != HAL_OK || len == 0U) {
        return status;
    }
    return HAL_UART_Transmit_IT(e220->huart, data, len);
}

void E220_RxCallback(E220_t *e220)
{
    if (!valid_handle(e220)) {
        return;
    }

    uint16_t head = e220->rx_head;
    uint16_t next = (uint16_t)((head + 1U) % E220_RX_BUFFER_SIZE);
    if (next != e220->rx_tail) {
        e220->rx_buffer[head] = e220->rx_byte;
        /* Publish the byte before the ISR advances the producer index. */
        __DMB();
        e220->rx_head = next;
    } else {
        ++e220->rx_overflow_count;
    }
    e220->rx_status = HAL_UART_Receive_IT(e220->huart, &e220->rx_byte, 1U);
}

void E220_ErrorCallback(E220_t *e220)
{
    if (!valid_handle(e220)) {
        return;
    }

    ++e220->uart_error_count;
    /* Abort RX only: clear UART errors and flush stale data, preserving TX. */
    e220->rx_status = HAL_UART_AbortReceive(e220->huart);
    if (e220->rx_status == HAL_OK) {
        e220->rx_status = HAL_UART_Receive_IT(e220->huart, &e220->rx_byte, 1U);
    }
}

uint16_t E220_Available(E220_t *e220)
{
    if (!valid_handle(e220)) {
        return 0U;
    }
    uint16_t head = e220->rx_head;
    uint16_t tail = e220->rx_tail;
    return (uint16_t)((head + E220_RX_BUFFER_SIZE - tail) % E220_RX_BUFFER_SIZE);
}

uint8_t E220_ReadByte(E220_t *e220)
{
    if (!valid_handle(e220)) {
        return 0U;
    }
    uint16_t tail = e220->rx_tail;
    if (tail == e220->rx_head) {
        return 0U;
    }
    __DMB();
    uint8_t data = e220->rx_buffer[tail];
    /* Finish reading before making this slot available to the RX ISR. */
    __DMB();
    e220->rx_tail = (uint16_t)((tail + 1U) % E220_RX_BUFFER_SIZE);
    return data;
}

uint16_t E220_Read(E220_t *e220, uint8_t *buf, uint16_t max_len)
{
    if (!valid_handle(e220) || buf == NULL) {
        return 0U;
    }

    uint16_t count = 0U;
    while (count < max_len && E220_Available(e220) != 0U) {
        buf[count++] = E220_ReadByte(e220);
    }
    return count;
}
