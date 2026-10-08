/*
 * rate_controller.c
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#include "rate_controller.h"

void RateController_Init(RateController_t *rate)
{
	// Gia tri khoi dau de kiem tra tren gia test
	PID_Init(&rate->roll,
			 0.5f, 1.4f, 0.0f,
			 0.0f, 180.0f);

	PID_Init(&rate->pitch,
			 0.5f, 1.4f, 0.0f,
			 0.0f, 180.0f);

	PID_Init(&rate->yaw,
			 0.3f, 0.0f, 0.0f,
			 0.0f, 100.0f);
}

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
