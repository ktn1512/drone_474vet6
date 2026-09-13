/*
 * mahony.c
 *
 *  Created on: 13 thg 9, 2026
 *      Author: khanh
 */

#include "mahony.h"
#include <math.h>

static float invSqrt(float x) {
	return 1.0f / sqrtf(x);
}

void Mahony_Init(Mahony_t *m) {
	m->q0 = 1.0f;
	m->q1 = 0.0f;
	m->q2 = 0.0f;
	m->q3 = 0.0f;

	m->twoKp = 4.0f;
	m->twoKi = 0.05f;

	m->integralFBx = 0;
	m->integralFBy = 0;
	m->integralFBz = 0;
}

void Mahony_Update(Mahony_t *m,

float gx, float gy, float gz,

float ax, float ay, float az,

float dt) {
	float recipNorm;

	float halfvx;
	float halfvy;
	float halfvz;

	float halfex;
	float halfey;
	float halfez;

	float qa;
	float qb;
	float qc;

	if ((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f)) {
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);

		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		halfvx = m->q1 * m->q3 - m->q0 * m->q2;

		halfvy = m->q0 * m->q1 + m->q2 * m->q3;

		halfvz = m->q0 * m->q0 - 0.5f + m->q3 * m->q3;

		halfex = ay * halfvz - az * halfvy;
		halfey = az * halfvx - ax * halfvz;
		halfez = ax * halfvy - ay * halfvx;

		m->integralFBx += m->twoKi * halfex * dt;
		m->integralFBy += m->twoKi * halfey * dt;
		m->integralFBz += m->twoKi * halfez * dt;

		gx += m->integralFBx;
		gy += m->integralFBy;
		gz += m->integralFBz;

		gx += m->twoKp * halfex;
		gy += m->twoKp * halfey;
		gz += m->twoKp * halfez;
	}

	gx *= 0.5f * dt * 0.0174533f;
	gy *= 0.5f * dt * 0.0174533f;
	gz *= 0.5f * dt * 0.0174533f;

	qa = m->q0;
	qb = m->q1;
	qc = m->q2;

	m->q0 += (-qb * gx - qc * gy - m->q3 * gz);
	m->q1 += (qa * gx + qc * gz - m->q3 * gy);
	m->q2 += (qa * gy - qb * gz + m->q3 * gx);
	m->q3 += (qa * gz + qb * gy - qc * gx);

	recipNorm = invSqrt(
			m->q0 * m->q0 + m->q1 * m->q1 + m->q2 * m->q2 + m->q3 * m->q3);

	m->q0 *= recipNorm;
	m->q1 *= recipNorm;
	m->q2 *= recipNorm;
	m->q3 *= recipNorm;
}

void Mahony_GetEuler(Mahony_t *m,

float *roll, float *pitch, float *yaw) {
	*roll = atan2f(2.0f * (m->q0 * m->q1 + m->q2 * m->q3),
			1.0f - 2.0f * (m->q1 * m->q1 + m->q2 * m->q2)) * 57.29578f;

	*pitch = asinf(2.0f * (m->q0 * m->q2 - m->q3 * m->q1)) * 57.29578f;

	*yaw = atan2f(2.0f * (m->q0 * m->q3 + m->q1 * m->q2),
			1.0f - 2.0f * (m->q2 * m->q2 + m->q3 * m->q3)) * 57.29578f;
}
