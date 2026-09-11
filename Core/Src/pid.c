/*
 * pid.c
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#include "pid.h"

static float constrainf(float x, float min, float max) {
	if (x > max)
		return max;
	if (x < min)
		return min;

	return x;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit,
		float output_limit) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->integral = 0.0f;
	pid->prev_error = 0.0f;

	pid->integral_limit = integral_limit;
	pid->output_limit = output_limit;
}

void PID_Reset(PID_t *pid) {
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float feedback, float dt) {
	float error = setpoint - feedback;

	pid->integral += error * dt;

	pid->integral = constrainf(pid->integral, -pid->integral_limit,
			pid->integral_limit);

	float derivative = (error - pid->prev_error) / dt;

	pid->prev_error = error;

	float output = pid->kp * error + pid->ki * pid->integral
			+ pid->kd * derivative;

	output = constrainf(output, -pid->output_limit, pid->output_limit);

	return output;
}
