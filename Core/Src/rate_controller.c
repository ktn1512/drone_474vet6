/**
 * @file rate_controller.c
 * @brief Cài đặt các vòng PID tốc độ góc tạo hiệu chỉnh động cơ.
 */
/*
 * rate_controller.c
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#include "rate_controller.h"

/**
 * @brief Khởi tạo PID tốc độ roll/pitch/yaw với các hệ số hiện tại.
 */
void RateController_Init(RateController_t *rate)
{
	// Gia tri khoi dau de kiem tra tren gia test
	PID_Init(&rate->roll,
			 0.8f, 0.02f, 0.007f,
			 10.0f, 180.0f);

	PID_Init(&rate->pitch,
			 0.8f, 0.002f, 0.007f,
			 10.0f, 180.0f);

	PID_Init(&rate->yaw,
			 0.3f, 0.0f, 0.0f,
			 0.0f, 100.0f);
}

/**
 * @brief Cập nhật ba PID tốc độ góc và ghi hiệu chỉnh tương ứng cho mixer.
 */
void RateController_Update(RateController_t *rate,

						   float roll_rate_sp, float pitch_rate_sp, float yaw_rate_sp,

						   float gyro_x, float gyro_y, float gyro_z,

						   float dt,

						   float *roll_out, float *pitch_out, float *yaw_out)
{
	*roll_out = PID_Update(&rate->roll, roll_rate_sp, gyro_x, dt);

	*pitch_out = PID_Update(&rate->pitch, pitch_rate_sp, gyro_y, dt);

	*yaw_out = PID_Update(&rate->yaw, yaw_rate_sp, gyro_z, dt);
}
