/*
 * mixer.h
 *
 *  Created on: 13 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_MIXER_H_
#define INC_MIXER_H_

#include <stdint.h>

typedef struct {
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;

} MotorMixer_t;

void Mixer_QuadX(float throttle,

float roll_pid, float pitch_pid, float yaw_pid,

MotorMixer_t *motor);
#endif /* INC_MIXER_H_ */
