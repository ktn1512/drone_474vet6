/**
 * @file pid.c
 * @brief Cài đặt PID, kiểm tra dt, lọc đạo hàm và chống tích phân bão hòa.
 */

#include "pid.h"
#include <math.h>

/**
 * @brief Giới hạn một giá trị vào khoảng đóng cho trước.
 * @param[in] x Giá trị cần giới hạn, cùng đơn vị với lo và hi.
 * @param[in] lo Cận dưới của khoảng.
 * @param[in] hi Cận trên của khoảng; bên gọi bảo đảm lo không lớn hơn hi.
 * @return hi nếu x vượt cận trên, lo nếu x dưới cận dưới, ngược lại trả x.
 * @note Không xử lý riêng NaN; khi x là NaN, các so sánh không đổi x.
 */
static float constrainf(float x, float lo, float hi)
{
	if (x > hi)
		return hi;
	if (x < lo)
		return lo;
	return x;
}

/**
 * @brief Khởi tạo hệ số/giới hạn và gọi PID_Reset để xóa lịch sử.
 */
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

/**
 * @brief Xóa trạng thái động, giữ nguyên hệ số và giới hạn.
 */
void PID_Reset(PID_t *pid)
{
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_feedback = 0.0f;
	pid->derivative_filtered = 0.0f;
	pid->initialized = 0U;
}

/**
 * @brief Cập nhật P, I có chống bão hòa và D lọc trên feedback.
 */
float PID_Update(PID_t *pid, float setpoint, float feedback, float dt)
{
	if (!isfinite(setpoint) || !isfinite(feedback) || !isfinite(dt) || dt < 0.0005f || dt > 0.01f)
	{
		PID_Reset(pid);
		return 0.0f;
	}

	/**
	 * @brief Sai số giá trị đặt trừ phép đo, đơn vị E.
	 */
	float error = setpoint - feedback;

	// P term
	/**
	 * @brief Thành phần tỉ lệ, đơn vị đầu ra U.
	 */
	float P = pid->kp * error;

	// D term: derivative on measurement
	if (!pid->initialized)
	{
		pid->prev_feedback = feedback;
		pid->initialized = 1U;
	}

	/**
	 * @brief Đạo hàm phép đo chưa lọc, đơn vị E/s.
	 */
	float derivative =
		(feedback - pid->prev_feedback) / dt;

	pid->prev_feedback = feedback;

	// Low-pass filter, time constant about 5.3 ms
	/**
	 * @brief Hằng số thời gian bộ lọc D, giây.
	 */
	const float tau = 0.0053f;
	/**
	 * @brief Trọng số cập nhật bộ lọc D, không có đơn vị.
	 */
	float alpha = dt / (tau + dt);

	pid->derivative_filtered +=
		alpha * (derivative - pid->derivative_filtered);

	/**
	 * @brief Thành phần D đối dấu đạo hàm feedback, đơn vị U.
	 */
	float D = -pid->kd * pid->derivative_filtered;

	// Candidate integral
	/**
	 * @brief Thành phần I ứng viên sau tích phân và trước kiểm tra chống bão hòa, đơn vị U.
	 */
	float new_integral =
		pid->integral + pid->ki * error * dt;

	new_integral = constrainf(
		new_integral,
		-pid->integral_limit,
		pid->integral_limit);

	// Conditional integration anti-windup
	/**
	 * @brief Tổng P + I ứng viên + D để xét tích phân có làm bão hòa tăng thêm hay không.
	 */
	float candidate_output = P + new_integral + D;

	if (!((candidate_output > pid->output_limit && error > 0.0f) ||
		  (candidate_output < -pid->output_limit && error < 0.0f)))
	{
		pid->integral = new_integral;
	}

	pid->prev_error = error;

	// Final PID output
	/**
	 * @brief Tổng các thành phần sau khi quyết định cập nhật I, đơn vị U.
	 */
	float output = P + pid->integral + D;

	return constrainf(output,
					  -pid->output_limit,
					  pid->output_limit);
}
