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
#define MMC5983_I2C_ADDR (0x30 << 1)

/* Giá trị Product ID */
#define MMC5983_WHOAMI 0x30

/* Cấu trúc quản lý MMC5983MA */
typedef struct
{
	I2C_HandleTypeDef *hi2c; // I2C dùng để giao tiếp

	int32_t raw_x; // Giá trị X thô 18-bit
	int32_t raw_y; // Giá trị Y thô 18-bit
	int32_t raw_z; // Giá trị Z thô 18-bit

	float mag_x; // Từ trường trục X (Gauss)
	float mag_y; // Từ trường trục Y (Gauss)
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
HAL_StatusTypeDef MMC5983_Init(MMC5983MA_t *dev);

/*
 * Đọc Product ID của MMC5983MA
 *
 * dev : con trỏ tới cấu trúc cảm biến
 *
 * Trả về:
 * 0x30 : đúng MMC5983MA
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
HAL_StatusTypeDef MMC5983_ReadMag(
	MMC5983MA_t *dev,
	float *x,
	float *y,
	float *z);

#endif