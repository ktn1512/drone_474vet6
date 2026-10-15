/**
 * @file angle_controller.h
 * @brief Giao diện vòng điều khiển góc roll/pitch tạo tốc độ góc đặt.
 */
/*
 * angle_controller.h
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_ANGLE_CONTROLLER_H_
#define INC_ANGLE_CONTROLLER_H_

#include "pid.h"

/**
 * @brief Trạng thái hai vòng PID góc roll/pitch sinh tốc độ góc đặt.
 */
typedef struct {
	/**
	 * @brief PID góc roll: đầu vào độ, đầu ra tốc độ góc đặt độ/s.
	 */
	PID_t roll;
	/**
	 * @brief PID góc pitch: đầu vào độ, đầu ra tốc độ góc đặt độ/s.
	 */
	PID_t pitch;

} AngleController_t;

/**
 * @brief Khởi tạo hai PID góc với hệ số hiện có trong implementation.
 * @param[out] angle Bộ điều khiển góc được khởi tạo; con trỏ phải hợp lệ.
 * @note Giới hạn I là 10 độ/s, giới hạn đầu ra là 200 độ/s; không kiểm tra NULL.
 */
void AngleController_Init(AngleController_t *angle);

/**
 * @brief Chuyển sai số góc roll/pitch thành tốc độ góc đặt cho vòng rate.
 * @param[in,out] angle Bộ điều khiển góc đã khởi tạo; con trỏ phải hợp lệ.
 * @param[in] roll_sp_deg Góc roll đặt, độ.
 * @param[in] pitch_sp_deg Góc pitch đặt, độ.
 * @param[in] roll_deg Góc roll đo, độ.
 * @param[in] pitch_deg Góc pitch đo, độ.
 * @param[in] dt Khoảng thời gian cập nhật, giây; PID chấp nhận 0,0005 đến 0,01 s.
 * @param[out] roll_rate_sp Nơi nhận tốc độ góc roll đặt, độ/s; con trỏ phải hợp lệ.
 * @param[out] pitch_rate_sp Nơi nhận tốc độ góc pitch đặt, độ/s; con trỏ phải hợp lệ.
 * @note Mỗi trục áp dụng chính sách dữ liệu/dt không hợp lệ của PID_Update;
 *       hàm không tự kiểm tra NULL.
 */
void AngleController_Update(AngleController_t *angle,

float roll_sp_deg, float pitch_sp_deg,

float roll_deg, float pitch_deg,

float dt,

float *roll_rate_sp, float *pitch_rate_sp);

#endif /* INC_ANGLE_CONTROLLER_H_ */
