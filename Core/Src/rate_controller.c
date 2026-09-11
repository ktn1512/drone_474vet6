/*
 * rate_controller.c
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#include "rate_controller.h"

void RateController_Init(RateController_t *rate) {
	PID_Init(&rate->roll, 0.18f, 0.08f, 0.004f, 100.0f, 500.0f);

	PID_Init(&rate->pitch, 0.18f, 0.08f, 0.004f, 100.0f, 500.0f);

	PID_Init(&rate->yaw, 0.25f, 0.10f, 0.000f, 100.0f, 500.0f);
}

void RateController_Update(RateController_t *rate,

float roll_rate_sp, float pitch_rate_sp, float yaw_rate_sp,

float gyro_x, float gyro_y, float gyro_z,

float dt,

float *roll_out, float *pitch_out, float *yaw_out) {
	*roll_out = PID_Update(&rate->roll, roll_rate_sp, gyro_x, dt);

	*pitch_out = PID_Update(&rate->pitch, pitch_rate_sp, gyro_y, dt);

	*yaw_out = PID_Update(&rate->yaw, yaw_rate_sp, gyro_z, dt);
}
