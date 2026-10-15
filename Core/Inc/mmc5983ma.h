/**
 * @file mmc5983ma.h
 * @brief Giao diện đo từ trường MMC5983MA qua I2C.
 */

/*
 * mmc5983.h
 *
 *  Created on: 22 thg 8, 2026
 *      Author: khanh
 */

#pragma once
#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* Địa chỉ I2C 7-bit của MMC5983MA */
/**
 * @brief Địa chỉ I2C 0x30 dạng đã dịch trái một bit để truyền vào HAL.
 */
#define MMC5983_I2C_ADDR (0x30 << 1)

/* Giá trị Product ID */
/**
 * @brief Product ID được MMC5983_Init chấp nhận.
 */
#define MMC5983_WHOAMI 0x30

/* Cấu trúc quản lý MMC5983MA */
/**
 * @brief Handle I2C và mẫu từ trường gần nhất của cảm biến MMC5983MA.
 */
typedef struct
{
	/**
	 * @brief Handle I2C phải được người gọi gán trước khi dùng driver.
	 */
	I2C_HandleTypeDef *hi2c; // I2C dùng để giao tiếp

	/**
	 * @brief Dữ liệu thô trục X 18 bit không dấu, lưu trong int32_t; 0..262143 LSB.
	 */
	int32_t raw_x; // Giá trị X thô 18-bit
	/**
	 * @brief Dữ liệu thô trục Y 18 bit không dấu, lưu trong int32_t; 0..262143 LSB.
	 */
	int32_t raw_y; // Giá trị Y thô 18-bit
	/**
	 * @brief Dữ liệu thô trục Z 18 bit không dấu, lưu trong int32_t; 0..262143 LSB.
	 */
	int32_t raw_z; // Giá trị Z thô 18-bit

	/**
	 * @brief Từ trường trục X, Gauss; cập nhật bởi ReadMag.
	 */
	float mag_x; // Từ trường trục X (Gauss)
	/**
	 * @brief Từ trường trục Y, Gauss; cập nhật bởi ReadMag.
	 */
	float mag_y; // Từ trường trục Y (Gauss)
	/**
	 * @brief Từ trường trục Z, Gauss; cập nhật bởi ReadMag.
	 */
	float mag_z; // Từ trường trục Z (Gauss)

} MMC5983MA_t;

/*
 * Khởi tạo MMC5983MA
 *
 * dev : con trỏ tới cấu trúc MMC5983MA_t
 *
 * Trả về:
 * HAL_OK    : khởi tạo thành công
 * HAL_ERROR : cảm biến không phản hồi / sai ID
 */
/**
 * @brief Xóa mẫu, kiểm tra Product ID và cấu hình chế độ đo từng lần.
 * @param[in,out] dev Trạng thái cảm biến; dev và dev->hi2c phải hợp lệ.
 * @retval HAL_OK Hoàn tất kiểm tra ID và ghi các thanh ghi cấu hình.
 * @retval HAL_ERROR Con trỏ không hợp lệ, ID sai hoặc một giao dịch I2C thất bại.
 * @note Giữ nguyên hi2c; xóa các trường mẫu trước khi kiểm tra Product ID.
 */
HAL_StatusTypeDef MMC5983_Init(MMC5983MA_t *dev);

/*
 * Đọc Product ID của MMC5983MA
 *
 * dev : con trỏ tới cấu trúc cảm biến
 *
 * Trả về:
 * 0x30 : đúng MMC5983MA
 */
/**
 * @brief Đọc một byte Product ID của cảm biến.
 * @param[in] dev Trạng thái chứa handle I2C đã khởi tạo.
 * @return Byte Product ID đọc được; trả về 0 nếu tham số/giao dịch đọc thất bại.
 */
uint8_t MMC5983_WhoAmI(MMC5983MA_t *dev);

/*
 * Đọc dữ liệu từ trường dạng raw
 *
 * dev : con trỏ tới cấu trúc cảm biến
 * x   : nơi lưu giá trị X raw
 * y   : nơi lưu giá trị Y raw
 * z   : nơi lưu giá trị Z raw
 *
 * Raw có giá trị khoảng 0 -> 262143
 */
/**
 * @brief Kích hoạt một phép đo, chờ hoàn tất và ghép dữ liệu từ trường 18 bit.
 * @param[in,out] dev Trạng thái có I2C hợp lệ; nhận raw_x/raw_y/raw_z khi thành công.
 * @param[out] x Nơi nhận dữ liệu X thô, 0..262143 LSB; không được NULL.
 * @param[out] y Nơi nhận dữ liệu Y thô, 0..262143 LSB; không được NULL.
 * @param[out] z Nơi nhận dữ liệu Z thô, 0..262143 LSB; không được NULL.
 * @retval HAL_OK Dữ liệu đầu ra và raw_* đã cập nhật.
 * @retval HAL_TIMEOUT Cờ hoàn tất phép đo chưa xuất hiện trong thời gian chờ.
 * @retval HAL_ERROR Tham số không hợp lệ hoặc giao dịch I2C thất bại.
 * @note Hàm chờ bằng HAL_Delay; các trường mag_* giữ nguyên.
 */
HAL_StatusTypeDef MMC5983_ReadRaw(
	MMC5983MA_t *dev,
	int32_t *x,
	int32_t *y,
	int32_t *z);

/*
 * Đọc dữ liệu từ trường và đổi sang Gauss
 *
 * dev : con trỏ tới cấu trúc cảm biến
 * x   : nơi lưu từ trường X
 * y   : nơi lưu từ trường Y
 * z   : nơi lưu từ trường Z
 */
/**
 * @brief Đọc một mẫu thô rồi đổi từ trường sang Gauss.
 * @param[in,out] dev Trạng thái cảm biến; nhận cả raw_* và mag_* khi thành công.
 * @param[out] x Nơi nhận từ trường X, Gauss; không được NULL.
 * @param[out] y Nơi nhận từ trường Y, Gauss; không được NULL.
 * @param[out] z Nơi nhận từ trường Z, Gauss; không được NULL.
 * @retval HAL_OK Đã cập nhật dữ liệu từ trường.
 * @retval HAL_TIMEOUT Phép đo thô hết thời gian chờ.
 * @retval HAL_ERROR Tham số không hợp lệ hoặc giao dịch I2C thất bại.
 * @note Mỗi trục được đổi theo (raw - 131072) / 16384; không áp dụng hiệu chuẩn hard/soft iron.
 */
HAL_StatusTypeDef MMC5983_ReadMag(
	MMC5983MA_t *dev,
	float *x,
	float *y,
	float *z);

#endif