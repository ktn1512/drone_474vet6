/*
 * pid.h
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float kp;
	float ki;
	float kd;

	float integral;
	float prev_error;

	float integral_limit;
	float output_limit;

} PID_t;

/**
 * @brief Khởi tạo PID
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float integral_limit,
		float output_limit);

/**
 * @brief Reset PID
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Tính PID
 */
float PID_Update(PID_t *pid, float setpoint, float feedback, float dt);

#endif
/* INC_PID_H_ */
