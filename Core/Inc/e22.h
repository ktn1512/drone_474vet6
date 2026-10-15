/*
 * e22.h
 *
 *  Created on: 23 thg 8, 2026
 *      Author: khanh
 */

/**
 * @file e22.h
 * @brief API E22 cũ: UART TX chờ đồng bộ, RX bằng ngắt một byte.
 * @details Driver không kiểm tra NULL, không cấu hình chân mode hay thông số RF.
 * Con trỏ phải hợp lệ và UART phải được khởi tạo trước khi gọi.
 */
#ifndef __E22_H
#define __E22_H

#include "main.h"
#include <stdint.h>

/** @brief Số ô FIFO RX; chừa một ô phân biệt đầy/rỗng, chứa tối đa 255 byte. */
#define E22_RX_BUFFER_SIZE 256

/**
 * @brief Trạng thái UART và FIFO nhận E22.
 * @details Callback ghi qua rx_head, vòng lặp chính đọc qua rx_tail.
 * FIFO đầy thì bỏ byte mới; driver này không có bộ đếm overflow.
 */
typedef struct {
	UART_HandleTypeDef *huart; /**< UART được khởi tạo trước E22_Init và chia sẻ với HAL. */

	uint8_t rx_byte; /**< Ô một byte HAL đang nhận bằng ngắt. */

	uint8_t rx_buffer[E22_RX_BUFFER_SIZE]; /**< FIFO nhận UART, tối đa 255 byte chưa đọc. */
	volatile uint16_t rx_head; /**< Chỉ số ô ghi tiếp theo, callback RX cập nhật. */
	volatile uint16_t rx_tail; /**< Chỉ số byte đọc tiếp theo, hàm đọc cập nhật. */

} E22_t;

/**
 * @brief Gán UART, đặt FIFO rỗng và yêu cầu HAL nhận một byte bằng ngắt.
 * @param[out] e22 Trạng thái driver tồn tại suốt thời gian dùng RX ngắt.
 * @param[in] huart UART đã khởi tạo, khác NULL.
 * @note Kết quả HAL_UART_Receive_IT không được trả về hay lưu lại.
 */
void E22_Init(E22_t *e22, UART_HandleTypeDef *huart);

/**
 * @brief Gửi bằng HAL_UART_Transmit đồng bộ với timeout 100 ms.
 * @param[in,out] e22 Driver có UART hợp lệ; HAL cập nhật trạng thái UART.
 * @param[in] data Bộ đệm hợp lệ chứa ít nhất len byte.
 * @param[in] len Số byte yêu cầu gửi.
 * @return Trạng thái HAL_UART_Transmit: HAL_OK, HAL_BUSY, HAL_ERROR hoặc HAL_TIMEOUT.
 * @note HAL_OK chỉ chứng minh hoàn tất UART cục bộ, không xác nhận nhận RF.
 */
HAL_StatusTypeDef E22_Send(E22_t *e22, const uint8_t *data, uint16_t len);

/**
 * @brief Gửi chuỗi có NUL kết thúc bằng API TX đồng bộ, không gửi NUL.
 * @param[in,out] e22 Driver có UART hợp lệ.
 * @param[in] str Chuỗi hợp lệ, độ dài phải vừa uint16_t để gửi đủ.
 * @return Trạng thái E22_Send với độ dài strlen(str).
 * @note Không tự bổ sung newline hay checksum.
 */
HAL_StatusTypeDef E22_SendString(E22_t *e22, const char *str);

/**
 * @brief Đưa byte RX vào FIFO nếu còn chỗ và đăng ký byte tiếp theo.
 * @param[in,out] e22 Driver của UART vừa phát sinh HAL_UART_RxCpltCallback.
 * @note FIFO đầy thì bỏ byte mới; không lưu kết quả đăng ký RX tiếp.
 */
void E22_RxCallback(E22_t *e22);

/**
 * @brief Lấy số byte hiện có trong FIFO, không tiêu thụ dữ liệu.
 * @param[in] e22 Driver hợp lệ cần kiểm tra.
 * @return Số byte có thể đọc, 0..E22_RX_BUFFER_SIZE - 1.
 */
uint16_t E22_Available(E22_t *e22);

/**
 * @brief Lấy và loại byte cũ nhất khỏi FIFO nếu có.
 * @param[in,out] e22 Driver hợp lệ; rx_tail tăng khi đọc thành công.
 * @return Byte đọc được; 0 khi FIFO rỗng.
 * @note Dữ liệu thật cũng có thể bằng 0; kiểm tra E22_Available trước khi đọc.
 */
uint8_t E22_ReadByte(E22_t *e22);

/**
 * @brief Đọc tối đa max_len byte đang có trong FIFO, không chờ byte mới.
 * @param[in,out] e22 Driver hợp lệ có FIFO cần tiêu thụ.
 * @param[out] buf Bộ đệm hợp lệ có dung lượng ít nhất max_len byte.
 * @param[in] max_len Giới hạn số byte lấy ra.
 * @return Số byte thực tế đã chép vào buf.
 * @note Không bổ sung NUL vào buf.
 */
uint16_t E22_Read(E22_t *e22, uint8_t *buf, uint16_t max_len);

#endif /* INC_E22_H_ */
