/**
 * @file rate_controller.h
 * @brief Giao diện vòng điều khiển tốc độ góc roll/pitch/yaw.
 */
/*
 * rate_controller.h
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_RATE_CONTROLLER_H_
#define INC_RATE_CONTROLLER_H_

#include "pid.h"

/**
 * @brief Trạng thái ba vòng PID tốc độ góc tạo hiệu chỉnh cho mixer.
 */
typedef struct {
	/**
	 * @brief PID tốc độ roll: đầu vào độ/s, đầu ra hiệu chỉnh độ rộng xung µs.
	 */
	PID_t roll;
	/**
	 * @brief PID tốc độ pitch: đầu vào độ/s, đầu ra hiệu chỉnh độ rộng xung µs.
	 */
	PID_t pitch;
	/**
	 * @brief PID tốc độ yaw: đầu vào độ/s, đầu ra hiệu chỉnh độ rộng xung µs.
	 */
	PID_t yaw;

} RateController_t;

/**
 * @brief Khởi tạo ba PID tốc độ góc với hệ số hiện có trong implementation.
 * @param[out] rate Bộ điều khiển tốc độ được khởi tạo; con trỏ phải hợp lệ.
 * @note Roll/pitch giới hạn I ở 10 µs và đầu ra ở 180 µs;
 *       yaw giới hạn I ở 0 µs và đầu ra ở 100 µs. Không kiểm tra NULL.
 */
void RateController_Init(RateController_t *rate);

/**
 * @brief Chuyển sai số tốc độ góc thành ba giá trị hiệu chỉnh cho Mixer_QuadX.
 * @param[in,out] rate Bộ điều khiển tốc độ đã khởi tạo; con trỏ phải hợp lệ.
 * @param[in] roll_rate_sp Tốc độ góc roll đặt, độ/s.
 * @param[in] pitch_rate_sp Tốc độ góc pitch đặt, độ/s.
 * @param[in] yaw_rate_sp Tốc độ góc yaw đặt, độ/s.
 * @param[in] gyro_x Tốc độ góc đo trên trục X dùng cho roll, độ/s.
 * @param[in] gyro_y Tốc độ góc đo trên trục Y dùng cho pitch, độ/s.
 * @param[in] gyro_z Tốc độ góc đo trên trục Z dùng cho yaw, độ/s.
 * @param[in] dt Khoảng thời gian cập nhật, giây; PID chấp nhận 0,0005 đến 0,01 s.
 * @param[out] roll_out Nơi nhận hiệu chỉnh roll, µs; con trỏ phải hợp lệ.
 * @param[out] pitch_out Nơi nhận hiệu chỉnh pitch, µs; con trỏ phải hợp lệ.
 * @param[out] yaw_out Nơi nhận hiệu chỉnh yaw, µs; con trỏ phải hợp lệ.
 * @note Mỗi trục áp dụng chính sách dữ liệu/dt không hợp lệ của PID_Update;
 *       không đổi dấu/đơn vị gyroscope và không tự kiểm tra NULL.
 */
void RateController_Update(RateController_t *rate,

float roll_rate_sp, float pitch_rate_sp, float yaw_rate_sp,

float gyro_x, float gyro_y, float gyro_z,

float dt,

float *roll_out, float *pitch_out, float *yaw_out);

#endif /* INC_RATE_CONTROLLER_H_ */
