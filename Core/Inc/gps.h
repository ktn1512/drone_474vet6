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
typedef struct
{
	double latitude;  // Vĩ độ (degree)
	double longitude; // Kinh độ (degree)

	float speed_kmh; // Tốc độ (km/h)
	float speed_ms;	 // Tốc độ (m/s)

	float course; // Hướng di chuyển COG (0-360 degree)

	uint8_t fix;		// 1: GPS có fix, 0: chưa có fix
	uint8_t satellites; // Số vệ tinh (chưa sử dụng trong RMC)

} GPS_Data_t;

/**
 * @brief  Khởi tạo thư viện GPS
 * @param  huart: UART dùng để giao tiếp với GPS
 * @retval HAL_OK nếu khởi tạo thành công
 */
HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Nhận từng byte dữ liệu GPS từ UART
 * @param  data: byte dữ liệu vừa nhận
 * @note   Gọi hàm này trong HAL_UART_RxCpltCallback()
 */
void GPS_RxCallback(uint8_t data);

/**
 * @brief  Phân tích câu NMEA vừa nhận
 * @note   Hàm này tự kiểm tra loại câu và cập nhật dữ liệu GPS
 */
void GPS_Process(void);

/**
 * @brief  Lấy dữ liệu GPS hiện tại
 * @retval Con trỏ tới cấu trúc GPS_Data_t
 */
GPS_Data_t *GPS_GetData(void);

#endif