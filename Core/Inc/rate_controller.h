/*
 * rate_controller.h
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_RATE_CONTROLLER_H_
#define INC_RATE_CONTROLLER_H_

#include "pid.h"

typedef struct {
	PID_t roll;
	PID_t pitch;
	PID_t yaw;

} RateController_t;

void RateController_Init(RateController_t *rate);

void RateController_Update(RateController_t *rate,

float roll_rate_sp, float pitch_rate_sp, float yaw_rate_sp,

float gyro_x, float gyro_y, float gyro_z,

float dt,

float *roll_out, float *pitch_out, float *yaw_out);

#endif /* INC_RATE_CONTROLLER_H_ */
