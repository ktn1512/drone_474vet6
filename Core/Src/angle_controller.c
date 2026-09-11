/*
 * angle_controller.c
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#include "angle_controller.h"

void AngleController_Init(AngleController_t *angle) {
	PID_Init(&angle->roll, 4.0f, 0.0f, 0.0f, 0.0f, 300.0f);

	PID_Init(&angle->pitch, 4.0f, 0.0f, 0.0f, 0.0f, 300.0f);
}

void AngleController_Update(AngleController_t *angle,

float roll_sp_deg, float pitch_sp_deg,

float roll_deg, float pitch_deg,

float dt,

float *roll_rate_sp, float *pitch_rate_sp) {
	*roll_rate_sp = PID_Update(&angle->roll, roll_sp_deg, roll_deg, dt);

	*pitch_rate_sp = PID_Update(&angle->pitch, pitch_sp_deg, pitch_deg, dt);
}
