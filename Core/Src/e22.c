/*
 * e22.c
 *
 *  Created on: 23 thg 8, 2026
 *      Author: khanh
 */

/**
 * @file e22.c
 * @brief Driver E22 cũ với TX UART đồng bộ và FIFO RX qua ngắt.
 * @details API yêu cầu con trỏ hợp lệ, không kiểm tra NULL và không xác nhận RF.
 */
#include "e22.h"
#include <string.h>

/** @brief Gán UART, đặt FIFO rỗng và đăng ký RX một byte. */
void E22_Init(E22_t *e22, UART_HandleTypeDef *huart) {
	e22->huart = huart;

	e22->rx_head = 0;
	e22->rx_tail = 0;

	HAL_UART_Receive_IT(e22->huart, &e22->rx_byte, 1);
}

/** @brief Chờ HAL truyền UART với timeout cố định 100 ms. */
HAL_StatusTypeDef E22_Send(E22_t *e22, const uint8_t *data, uint16_t len) {
	return HAL_UART_Transmit(e22->huart, (uint8_t*) data, len, 100);
}

/** @brief Gửi số byte strlen của chuỗi, không gửi NUL kết thúc. */
HAL_StatusTypeDef E22_SendString(E22_t *e22, const char *str) {
	return E22_Send(e22, (uint8_t*) str, strlen(str));
}

/** @brief Ghi byte nếu FIFO còn chỗ và đăng ký lần nhận tiếp theo. */
void E22_RxCallback(E22_t *e22) {
	uint16_t next = (e22->rx_head + 1) % E22_RX_BUFFER_SIZE; /**< Chỉ số ghi kế tiếp; bằng rx_tail nghĩa là hết ô trống. */

	if (next != e22->rx_tail) {
		e22->rx_buffer[e22->rx_head] = e22->rx_byte;

		e22->rx_head = next;
	}

	HAL_UART_Receive_IT(e22->huart, &e22->rx_byte, 1);
}

/** @brief Tính lượng byte chưa đọc, kể cả khi chỉ số FIFO đã vòng về đầu. */
uint16_t E22_Available(E22_t *e22) {
	if (e22->rx_head >= e22->rx_tail) {
		return e22->rx_head - e22->rx_tail;
	}

	return E22_RX_BUFFER_SIZE - e22->rx_tail + e22->rx_head;
}

/** @brief Tiêu thụ một byte FIFO, giữ giá trị mặc định 0 khi rỗng. */
uint8_t E22_ReadByte(E22_t *e22) {
	uint8_t data = 0; /**< Byte trả về, giữ 0 khi FIFO rỗng. */

	if (e22->rx_head != e22->rx_tail) {
		data = e22->rx_buffer[e22->rx_tail];

		e22->rx_tail = (e22->rx_tail + 1) % E22_RX_BUFFER_SIZE;
	}

	return data;
}

/** @brief Chép byte FIFO đang có theo giới hạn của người gọi. */
uint16_t E22_Read(E22_t *e22, uint8_t *buf, uint16_t max_len) {
	uint16_t count = 0; /**< Số byte đã đọc và chép vào buf. */

	while (E22_Available(e22) && count < max_len) {
		buf[count++] = E22_ReadByte(e22);
	}

	return count;
}
