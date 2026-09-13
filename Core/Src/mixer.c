/*
 * mixer.c
 *
 *  Created on: 13 thg 9, 2026
 *      Author: khanh
 */

#include "mixer.h"

static uint16_t LimitMotor(float value) {
	if (value < 1000.0f)
		value = 1000.0f;

	if (value > 2000.0f)
		value = 2000.0f;

	return (uint16_t) value;
}

void Mixer_QuadX(float throttle,

float roll_pid, float pitch_pid, float yaw_pid,

MotorMixer_t *motor) {
	float m1;
	float m2;
	float m3;
	float m4;

	m1 = throttle - roll_pid + pitch_pid - yaw_pid;
	m2 = throttle + roll_pid - pitch_pid + yaw_pid;
	m3 = throttle - roll_pid - pitch_pid + yaw_pid;
	m4 = throttle + roll_pid + pitch_pid - yaw_pid;

	motor->m1 = LimitMotor(m1);
	motor->m2 = LimitMotor(m2);
	motor->m3 = LimitMotor(m3);
	motor->m4 = LimitMotor(m4);
}
