/**
 * @file gps.h
 * @brief Giao diện gom byte UART và phân tích các trường GPS trong câu GNRMC.
 */

/*
 * gps.h
 *
 *  Created on: 22 thg 8, 2026
 *      Author: khanh
 */

#ifndef GPS_H
#define GPS_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* Dữ liệu GPS */
/**
 * @brief Các trường GPS được bộ phân tích RMC cập nhật, không chứa thời gian hay độ cao.
 */
typedef struct
{
	/**
	 * @brief Vĩ độ sau chuyển đổi, đơn vị độ thập phân; Nam mang dấu âm.
	 */
	double latitude;  // Vĩ độ (degree)
	/**
	 * @brief Kinh độ sau chuyển đổi, đơn vị độ thập phân; Tây mang dấu âm.
	 */
	double longitude; // Kinh độ (degree)

	/**
	 * @brief Tốc độ suy từ trường knot trong RMC, đơn vị km/h.
	 */
	float speed_kmh; // Tốc độ (km/h)
	/**
	 * @brief Tốc độ suy từ speed_kmh, đơn vị m/giây.
	 */
	float speed_ms;	 // Tốc độ (m/s)

	/**
	 * @brief Course Over Ground lấy từ RMC, đơn vị độ; không được chuẩn hóa phạm vi.
	 */
	float course; // Hướng di chuyển COG (0-360 degree)

	/**
	 * @brief Bằng 1 khi token trạng thái RMC bắt đầu bằng A, ngược lại bằng 0.
	 */
	uint8_t fix;		// 1: GPS có fix, 0: chưa có fix
	/**
	 * @brief Số vệ tinh; khởi tạo bằng 0 và chưa được parser RMC cập nhật.
	 */
	uint8_t satellites; // Số vệ tinh (chưa sử dụng trong RMC)

} GPS_Data_t;

/**
 * @brief  Khởi tạo thư viện GPS
 * @param  huart: UART dùng để giao tiếp với GPS
 * @retval HAL_OK nếu khởi tạo thành công
 */
/**
 * @brief Lưu handle UART và xóa dữ liệu GPS đã phân tích.
 * @param[in] huart Handle UART được lưu nội bộ; hàm không kiểm tra NULL.
 * @retval HAL_OK Luôn được trả về sau khi lưu handle và xóa gps_data.
 * @note Hàm không khởi động nhận UART và không đặt lại bộ đệm/chỉ số nhận câu.
 */
HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Nhận từng byte dữ liệu GPS từ UART
 * @param  data: byte dữ liệu vừa nhận
 * @note   Gọi hàm này trong HAL_UART_RxCpltCallback()
 */
/**
 * @brief Gom một byte vào bộ đệm và xử lý câu khi nhận ký tự xuống dòng.
 * @param[in] data Byte vừa nhận từ UART GPS.
 * @note Người gọi tự tổ chức việc nhận UART và truyền từng byte vào hàm.
 * @note Giữ tối đa 127 byte trước ký tự kết thúc chuỗi; byte vượt giới hạn bị bỏ qua.
 * @note Gặp LF thì thêm ký tự kết thúc, gọi GPS_Process và đặt lại chỉ số nhận.
 */
void GPS_RxCallback(uint8_t data);

/**
 * @brief  Phân tích câu NMEA vừa nhận
 * @note   Hàm này tự kiểm tra loại câu và cập nhật dữ liệu GPS
 */
/**
 * @brief Phân tích bộ đệm nếu bắt đầu bằng tiền tố $GNRMC.
 * @note Bộ đệm phải chứa chuỗi kết thúc bằng NUL; GPS_RxCallback thực hiện khi nhận LF.
 * @note Chưa kiểm checksum, trường rỗng hay đầy đủ câu; strtok bỏ qua token rỗng.
 * @note Dữ liệu được cập nhật theo token nhận được, kể cả khi trạng thái fix bằng 0.
 */
void GPS_Process(void);

/**
 * @brief  Lấy dữ liệu GPS hiện tại
 * @retval Con trỏ tới cấu trúc GPS_Data_t
 */
/**
 * @brief Lấy địa chỉ vùng dữ liệu GPS dùng chung của thư viện.
 * @return Con trỏ tới gps_data tĩnh; dữ liệu tồn tại suốt thời gian chương trình chạy.
 * @note Con trỏ cho phép sửa dữ liệu; thư viện không tạo bản sao hoặc khóa đồng bộ.
 */
GPS_Data_t *GPS_GetData(void);

#endif