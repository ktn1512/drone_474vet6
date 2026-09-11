/*
 * angle_controller.h
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_ANGLE_CONTROLLER_H_
#define INC_ANGLE_CONTROLLER_H_

#include "pid.h"

typedef struct {
	PID_t roll;
	PID_t pitch;

} AngleController_t;

void AngleController_Init(AngleController_t *angle);

void AngleController_Update(AngleController_t *angle,

float roll_sp_deg, float pitch_sp_deg,

float roll_deg, float pitch_deg,

float dt,

float *roll_rate_sp, float *pitch_rate_sp);

#endif /* INC_ANGLE_CONTROLLER_H_ */
