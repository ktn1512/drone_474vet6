/**
 * @file e220.h
 * @brief Driver E220 transparent: nhận UART bằng ngắt, gửi UART bằng DMA.
 * @details UART phải được khởi tạo 8N1, không điều khiển luồng và có DMA TX.
 * M0/M1 và cấu hình RF được thiết lập bên ngoài driver. HAL_OK chỉ báo HAL
 * chấp nhận thao tác UART, không xác nhận thiết bị đầu xa đã nhận qua RF.
 */
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
/** @brief Số ô FIFO RX; chừa một ô nên dung lượng hữu dụng là 255 byte. */
#define E220_RX_BUFFER_SIZE       256U
/** @brief Số byte tối đa cho một lần TX DMA. */
#define E220_MAX_PACKET_SIZE      256U

/**
 * @brief Trạng thái UART, FIFO RX và vùng nhớ TX tồn tại suốt phiên sử dụng.
 * @details ISR ghi qua rx_head, vòng lặp chính đọc qua rx_tail; FIFO đầy bỏ byte mới.
 * Bộ đếm uint32_t có thể tràn tự nhiên. Chỉ dùng một luồng gọi TX; tx_active
 * bảo vệ bộ đệm DMA đến khi callback kết thúc truyền USART được gọi.
 */
typedef struct {
    UART_HandleTypeDef *huart; /**< UART đã khởi tạo cho RX ngắt và TX DMA. */
    uint8_t rx_byte; /**< Ô một byte HAL đang nhận bằng ngắt. */
    uint8_t rx_buffer[E220_RX_BUFFER_SIZE]; /**< FIFO byte UART RX, dung lượng hữu dụng E220_RX_BUFFER_SIZE - 1. */
    volatile uint16_t rx_head; /**< Chỉ số ô ghi tiếp theo, callback RX cập nhật. */
    volatile uint16_t rx_tail; /**< Chỉ số byte đọc tiếp theo, hàm đọc cập nhật. */
    volatile uint32_t rx_overflow_count; /**< Số byte bị bỏ vì FIFO RX đầy. */
    volatile uint32_t rx_total_count; /**< Số byte qua callback RX, kể cả byte bị bỏ vì FIFO đầy. */
    volatile uint32_t uart_error_count; /**< Số lần callback lỗi UART hợp lệ được gọi. */
    volatile HAL_StatusTypeDef rx_status; /**< Trạng thái HAL gần nhất khi bắt đầu hoặc phục hồi RX. */

    /* DMA reads from THIS persistent buffer; caller may use stack memory. */
    uint8_t tx_dma_buffer[E220_MAX_PACKET_SIZE]; /**< Bản sao TX phải tồn tại và không đổi khi tx_active khác 0. */
    volatile uint8_t tx_active; /**< Cờ 1 khi bộ đệm TX được giữ đến callback USART TC; 0 khi rảnh. */
    volatile uint32_t tx_start_count; /**< Số lần HAL_UART_Transmit_DMA trả HAL_OK. */
    volatile uint32_t tx_done_count; /**< Số callback TX USART hoàn tất; không phải số ACK RF. */
    volatile uint32_t tx_busy_count; /**< Số yêu cầu bị chặn vì driver/UART bận hoặc HAL trả HAL_BUSY. */
    volatile uint32_t tx_error_count; /**< Số lỗi khởi động TX khác HAL_BUSY và lần phục hồi TX đã ngừng do lỗi. */
} E220_t;

/* USART must already be initialized, and huart->hdmatx linked to TX DMA. */
/**
 * @brief Kiểm tra UART, xóa trạng thái driver và bắt đầu RX một byte bằng ngắt.
 * @param[out] e220 Vùng trạng thái tồn tại lâu dài, khác NULL.
 * @param[in] huart UART đã khởi tạo 8N1, không flow control, có hdmatx hợp lệ.
 * @retval HAL_ERROR Con trỏ hoặc cấu hình không đáp ứng điều kiện.
 * @return Trạng thái HAL_UART_Receive_IT khi cấu hình hợp lệ; lưu vào rx_status.
 * @note Không gọi lại trong lúc TX đang sử dụng bộ đệm driver.
 */
HAL_StatusTypeDef E220_Init(E220_t *e220, UART_HandleTypeDef *huart);

/* Safe for stack or temporary caller buffers: driver copies into tx_dma_buffer. */
/**
 * @brief Sao chép gói và bắt đầu TX UART bằng DMA, không chờ hoàn tất.
 * @param[in,out] e220 Driver hợp lệ có DMA TX; cập nhật cờ và bộ đếm TX.
 * @param[in] data Bộ đệm ít nhất len byte; có thể là vùng nhớ tạm trên stack.
 * @param[in] len Số byte cần gửi, từ 1 đến E220_MAX_PACKET_SIZE.
 * @retval HAL_OK HAL đã bắt đầu TX DMA; chưa xác nhận hoàn tất UART hay nhận RF.
 * @retval HAL_BUSY Driver/UART đang bận hoặc HAL báo bận.
 * @retval HAL_ERROR Tham số không hợp lệ hoặc HAL báo lỗi khởi động TX.
 * @return Trạng thái HAL_UART_Transmit_DMA nếu đã gọi HAL.
 */
HAL_StatusTypeDef E220_Send_DMA(E220_t *e220, const uint8_t *data, uint16_t len);
/**
 * @brief Gửi chuỗi kết thúc bằng NUL qua DMA, không gửi NUL hay tự thêm newline.
 * @param[in,out] e220 Driver dùng để truyền và cập nhật cờ, bộ đếm TX.
 * @param[in] str Chuỗi hợp lệ có NUL kết thúc, dài 1..E220_MAX_PACKET_SIZE byte.
 * @retval HAL_ERROR str là NULL, chuỗi rỗng hoặc quá dài.
 * @return Trạng thái E220_Send_DMA: HAL_OK, HAL_BUSY hoặc lỗi HAL.
 */
HAL_StatusTypeDef E220_SendString_DMA(E220_t *e220, const char *str);

/* Compatibility with older main/drone_link modules: ALWAYS use TX DMA. */
/**
 * @brief API tương thích chuyển toàn bộ việc gửi sang E220_Send_DMA.
 * @param[in,out] e220 Driver hợp lệ, có DMA TX.
 * @param[in] data Bộ đệm ít nhất len byte; được sao chép trước khi hàm trả về.
 * @param[in] len Số byte, 1..E220_MAX_PACKET_SIZE.
 * @return Trạng thái E220_Send_DMA; HAL_OK chỉ báo bắt đầu TX UART.
 */
HAL_StatusTypeDef E220_Send(E220_t *e220, const uint8_t *data, uint16_t len);
/**
 * @brief API tương thích gửi chuỗi qua E220_SendString_DMA.
 * @param[in,out] e220 Driver hợp lệ, có DMA TX.
 * @param[in] str Chuỗi có NUL kết thúc, dài 1..E220_MAX_PACKET_SIZE byte.
 * @return Trạng thái E220_SendString_DMA; không gửi NUL, không thêm newline.
 */
HAL_StatusTypeDef E220_SendString(E220_t *e220, const char *str);
/**
 * @brief API tương thích có tên IT nhưng luôn gửi bằng DMA.
 * @param[in,out] e220 Driver hợp lệ, có DMA TX.
 * @param[in] data Bộ đệm ít nhất len byte, được sao chép vào driver.
 * @param[in] len Số byte, 1..E220_MAX_PACKET_SIZE.
 * @return Trạng thái E220_Send_DMA; HAL_OK không xác nhận chuyển giao RF.
 */
HAL_StatusTypeDef E220_Send_IT(E220_t *e220, const uint8_t *data, uint16_t len);

/* Call from main.c HAL_UART_* callbacks for the E220 USART only. */
/**
 * @brief Đưa byte RX vào FIFO nếu còn chỗ và đăng ký nhận byte kế tiếp.
 * @param[in,out] e220 Driver của UART vừa gọi HAL_UART_RxCpltCallback.
 * @note Mỗi byte tăng rx_total_count; byte bị bỏ do FIFO đầy tăng rx_overflow_count.
 */
void E220_RxCallback(E220_t *e220);
/**
 * @brief Giải phóng bộ đệm TX tại callback hoàn tất USART và tăng tx_done_count.
 * @param[in,out] e220 Driver của UART vừa gọi HAL_UART_TxCpltCallback.
 * @note Hoàn tất USART không xác nhận radio đầu xa nhận được gói qua RF.
 */
void E220_TxCpltCallback(E220_t *e220);
/**
 * @brief Ghi nhận lỗi UART, phục hồi RX và mở khóa TX nếu HAL đã về READY.
 * @param[in,out] e220 Driver của UART vừa gọi HAL_UART_ErrorCallback.
 * @note Chỉ abort RX; không chủ động hủy TX do lỗi RX.
 */
void E220_ErrorCallback(E220_t *e220);

/**
 * @brief Lấy số byte đang có trong FIFO RX mà không tiêu thụ dữ liệu.
 * @param[in] e220 Driver cần kiểm tra.
 * @return Số byte 0..255; trả 0 nếu driver không hợp lệ.
 */
uint16_t E220_Available(E220_t *e220);
/**
 * @brief Lấy và loại byte cũ nhất khỏi FIFO RX.
 * @param[in,out] e220 Driver; rx_tail tăng khi đọc thành công.
 * @return Byte lấy được; 0 nếu FIFO rỗng hoặc driver không hợp lệ.
 * @note Dữ liệu thật cũng có thể bằng 0; kiểm tra E220_Available trước khi đọc.
 */
uint8_t E220_ReadByte(E220_t *e220);
/**
 * @brief Đọc tối đa max_len byte đang có trong FIFO, không chờ dữ liệu mới.
 * @param[in,out] e220 Driver có FIFO RX cần tiêu thụ.
 * @param[out] buf Bộ đệm đích dung lượng ít nhất max_len byte.
 * @param[in] max_len Số byte tối đa muốn lấy.
 * @return Số byte đã chép; 0 nếu buf là NULL hoặc không có dữ liệu.
 * @note Không bổ sung NUL vào buf.
 */
uint16_t E220_Read(E220_t *e220, uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif
#endif
