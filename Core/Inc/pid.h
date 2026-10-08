
#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct
{
	float kp;
	float ki;
	float kd;

	float integral;
	float prev_error;
	float prev_feedback;
	float derivative_filtered;

	float integral_limit;
	float output_limit;

	uint8_t initialized;
} PID_t;

void PID_Init(PID_t *pid,
			  float kp, float ki, float kd,
			  float integral_limit,
			  float output_limit);

void PID_Reset(PID_t *pid);

float PID_Update(PID_t *pid,
				 float setpoint,
				 float feedback,
				 float dt);

#endif
