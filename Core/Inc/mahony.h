/*
 * mahony.h
 *
 *  Created on: 13 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_MAHONY_H_
#define INC_MAHONY_H_

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;

	float twoKp;
	float twoKi;

	float integralFBx;
	float integralFBy;
	float integralFBz;

} Mahony_t;

void Mahony_Init(Mahony_t *mahony);

/* Initialize tilt from stationary accelerometer data; yaw starts at zero.
 * Sensor axes must match the gyro axes, with +Z reading +1 g when level.
 * Returns 1 on success, 0 for invalid acceleration (state unchanged).
 */
int Mahony_InitFromAccel(Mahony_t *mahony, float ax, float ay, float az);

/* Gyroscope inputs: degrees/s; accelerometer inputs: g; dt: seconds. */
void Mahony_Update(Mahony_t *mahony,

float gx, float gy, float gz,

float ax, float ay, float az,

float dt);

void Mahony_GetEuler(Mahony_t *mahony,

float *roll, float *pitch, float *yaw);

#endif /* INC_MAHONY_H_ */
