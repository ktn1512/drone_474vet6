
#include "pid.h"
#include <math.h>

static float constrainf(float x, float lo, float hi)
{
	if (x > hi)
		return hi;
	if (x < lo)
		return lo;
	return x;
}

void PID_Init(PID_t *pid,
			  float kp, float ki, float kd,
			  float integral_limit,
			  float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->integral_limit = fabsf(integral_limit);
	pid->output_limit = fabsf(output_limit);

	PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_feedback = 0.0f;
	pid->derivative_filtered = 0.0f;
	pid->initialized = 0U;
}

float PID_Update(PID_t *pid,
				 float setpoint,
				 float feedback,
				 float dt)
{
	if (!isfinite(setpoint) ||
		!isfinite(feedback) ||
		!isfinite(dt) ||
		dt < 0.0005f ||
		dt > 0.01f)
	{
		PID_Reset(pid);
		return 0.0f;
	}

	float error = setpoint - feedback;

	// P term
	float P = pid->kp * error;

	// D term: derivative on measurement
	if (!pid->initialized)
	{
		pid->prev_feedback = feedback;
		pid->initialized = 1U;
	}

	float derivative =
		(feedback - pid->prev_feedback) / dt;

	pid->prev_feedback = feedback;

	// Low-pass filter, time constant about 5.3 ms
	const float tau = 0.0053f;
	float alpha = dt / (tau + dt);

	pid->derivative_filtered +=
		alpha * (derivative - pid->derivative_filtered);

	float D = -pid->kd * pid->derivative_filtered;

	// Candidate integral
	float new_integral =
		pid->integral + pid->ki * error * dt;

	new_integral = constrainf(
		new_integral,
		-pid->integral_limit,
		pid->integral_limit);

	// Conditional integration anti-windup
	float candidate_output = P + new_integral + D;

	if (!((candidate_output > pid->output_limit && error > 0.0f) ||
		  (candidate_output < -pid->output_limit && error < 0.0f)))
	{
		pid->integral = new_integral;
	}

	pid->prev_error = error;

	// Final PID output
	float output = P + pid->integral + D;

	return constrainf(output,
					  -pid->output_limit,
					  pid->output_limit);
}
