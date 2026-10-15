/**
 * @file dshot.h
 * @brief Giao diện xuất DShot300 cho bốn động cơ qua PWM và DMA của TIM5.
 */
/*
 * dshot.h
 *
 *  Created on: 30 thg 8, 2026
 *      Author: khanh
 */

#ifndef DSHOT_H
#define DSHOT_H

#include "main.h"

/**
 * @brief Xóa trạng thái truyền và khởi động PWM trên bốn kênh TIM5.
 * @pre TIM5 và DMA đã được cấu hình; implementation giả định clock bộ đếm
 *      144 MHz, prescaler = 0 và ARR = 479 để phát DShot300.
 * @note Hàm không kiểm tra giá trị trả về của HAL_TIM_PWM_Start.
 */
void DSHOT_Init(void);

/**
 * @brief Phát bất đồng bộ một khung DShot cho mỗi động cơ bằng PWM/DMA TIM5.
 * @param[in] m1 Lệnh M1 tương đương độ rộng xung PWM, µs.
 * @param[in] m2 Lệnh M2 tương đương độ rộng xung PWM, µs.
 * @param[in] m3 Lệnh M3 tương đương độ rộng xung PWM, µs.
 * @param[in] m4 Lệnh M4 tương đương độ rộng xung PWM, µs.
 * @pre Gọi DSHOT_Init sau khi cấu hình TIM5/DMA.
 * @note Đầu vào được chặn trong 1000–2000 µs rồi ánh xạ thành giá trị DShot
 *       48–2047; 1000 µs tạo giá trị 48, không tạo lệnh dừng DShot 0.
 *       Bit yêu cầu telemetry luôn bằng 0. Khi khung trước còn bận DMA,
 *       lời gọi mới được bỏ qua. Khi khởi động DMA lỗi, hàm dừng cả bốn
 *       kênh DMA và xóa trạng thái bận; không trả trạng thái lỗi cho bên gọi.
 */
void DSHOT_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif
/* INC_DSHOT_H_ */
