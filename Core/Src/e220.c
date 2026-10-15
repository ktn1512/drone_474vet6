/**
 * @file e220.c
 * @brief FIFO RX một byte/ngắt và TX DMA dùng bộ đệm tồn tại lâu dài.
 * @details Callback từ main.c cần được điều phối cho đúng UART E220.
 * TX USART hoàn tất là bằng chứng cục bộ, không xác nhận nhận RF.
 */
#include "e220.h"
#include <stddef.h>
#include <string.h>

/**
 * @brief Kiểm tra driver, UART, Instance và baud rate cơ bản trước thao tác.
 * @param[in] e Driver cần kiểm tra, có thể là NULL.
 * @retval 1 Các con trỏ cần thiết hợp lệ và baud rate khác 0.
 * @retval 0 Thiếu một điều kiện trên.
 * @note Không kiểm tra DMA TX hay toàn bộ định dạng UART.
 */
static int valid_handle(const E220_t *e) {
    return e != NULL && e->huart != NULL && e->huart->Instance != NULL &&
           e->huart->Init.BaudRate > 0U;
}

/** @brief Kiểm tra UART 8N1 và DMA TX rồi bắt đầu RX bằng ngắt. */
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

/** @brief Khóa bộ đệm, sao chép gói và yêu cầu HAL khởi động TX DMA. */
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
    HAL_StatusTypeDef rc = HAL_UART_Transmit_DMA(e->huart, e->tx_dma_buffer, len); /**< Kết quả khởi động DMA, dùng cập nhật cờ và bộ đếm TX. */
    if (rc == HAL_OK) {
        ++e->tx_start_count;
    } else {
        e->tx_active = 0U;
        if (rc == HAL_BUSY) ++e->tx_busy_count;
        else ++e->tx_error_count;
    }
    return rc;
}

/** @brief Đo độ dài chuỗi có giới hạn trước khi chuyển sang API DMA. */
HAL_StatusTypeDef E220_SendString_DMA(E220_t *e, const char *str) {
    if (!str) return HAL_ERROR;
    size_t n = 0U; /**< Độ dài chuỗi theo byte, chưa gồm NUL. */
    while (n <= E220_MAX_PACKET_SIZE && str[n] != '\0') ++n;
    if (n == 0U || n > E220_MAX_PACKET_SIZE) return HAL_ERROR;
    return E220_Send_DMA(e, (const uint8_t *)str, (uint16_t)n);
}

/* One DMA-only implementation for ALL sending APIs. */
/** @brief Chuyển API gửi tương thích sang cùng cài đặt TX DMA. */
HAL_StatusTypeDef E220_Send(E220_t *e, const uint8_t *data, uint16_t n) {
    return E220_Send_DMA(e, data, n);
}
/** @brief Giữ tên API IT để tương thích, đường truyền thực tế vẫn là DMA. */
HAL_StatusTypeDef E220_Send_IT(E220_t *e, const uint8_t *data, uint16_t n) {
    return E220_Send_DMA(e, data, n);
}
/** @brief Chuyển chuỗi sang API đo độ dài và TX DMA. */
HAL_StatusTypeDef E220_SendString(E220_t *e, const char *s) {
    return E220_SendString_DMA(e, s);
}

/* Called at the end of physical USART shift-register transmission. */
/** @brief Mở khóa bộ đệm TX tại callback kết thúc truyền USART. */
void E220_TxCpltCallback(E220_t *e) {
    if (!valid_handle(e)) return;
    e->tx_active = 0U;
    ++e->tx_done_count;
}

/** @brief Ghi byte RX hoặc ghi nhận tràn FIFO, rồi đăng ký RX tiếp theo. */
void E220_RxCallback(E220_t *e) {
    if (!valid_handle(e)) return;
    ++e->rx_total_count;
    uint16_t next = (uint16_t)((e->rx_head + 1U) % E220_RX_BUFFER_SIZE); /**< Ô FIFO kế tiếp; trùng rx_tail nghĩa là FIFO đầy. */
    if (next != e->rx_tail) {
        e->rx_buffer[e->rx_head] = e->rx_byte;
        __DMB();
        e->rx_head = next;
    } else {
        ++e->rx_overflow_count;
    }
    e->rx_status = HAL_UART_Receive_IT(e->huart, &e->rx_byte, 1U);
}

/** @brief Ghi nhận lỗi, mở khóa TX khi UART rảnh và khởi động lại RX. */
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

/** @brief Tính số byte chưa đọc bằng chênh lệch chỉ số FIFO theo modulo. */
uint16_t E220_Available(E220_t *e) {
    if (!valid_handle(e)) return 0U;
    return (uint16_t)((e->rx_head + E220_RX_BUFFER_SIZE - e->rx_tail) % E220_RX_BUFFER_SIZE);
}
/** @brief Đọc FIFO với hàng rào bộ nhớ trước khi công bố rx_tail mới. */
uint8_t E220_ReadByte(E220_t *e) {
    if (!valid_handle(e) || e->rx_tail == e->rx_head) return 0U;
    uint16_t index = e->rx_tail; /**< Ảnh chụp chỉ số byte cũ nhất trước khi đọc. */
    __DMB();
    uint8_t data = e->rx_buffer[index]; /**< Byte lấy khỏi FIFO; có thể bằng 0. */
    __DMB();
    e->rx_tail = (uint16_t)((index + 1U) % E220_RX_BUFFER_SIZE);
    return data;
}
/** @brief Tiêu thụ byte FIFO đến khi hết dữ liệu hoặc đạt max_len. */
uint16_t E220_Read(E220_t *e, uint8_t *data, uint16_t max_len) {
    if (data == NULL) return 0U;
    uint16_t n = 0U; /**< Số byte đã chép vào bộ đệm đích. */
    while (n < max_len && E220_Available(e) != 0U)
        data[n++] = E220_ReadByte(e);
    return n;
}
