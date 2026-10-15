/**
 * @file mixer.c
 * @brief Trộn lệnh Quad X theo quy ước IMU và giới hạn từng đầu ra động cơ.
 */

#include "mixer.h"
#include <math.h>

/**
 * @brief Cận dưới lệnh động cơ sau trộn, µs.
 */
#define MOTOR_MIN_US 1000.0f
/**
 * @brief Cận trên lệnh động cơ sau trộn, µs.
 */
#define MOTOR_MAX_US 1700.0f

/**
 * @brief Chuẩn hóa lệnh động cơ về độ rộng xung nguyên trong giới hạn.
 * @param[in] value Lệnh động cơ chưa giới hạn, µs.
 * @return Lệnh được chặn trong 1000–1700 µs và làm tròn gần nhất.
 * @retval 1000U Khi value không hữu hạn hoặc nằm dưới cận nhỏ nhất.
 */
static uint16_t LimitMotor(float value)
{
	if (!isfinite(value))
		return 1000U;

	if (value < MOTOR_MIN_US)
		value = MOTOR_MIN_US;

	if (value > MOTOR_MAX_US)
		value = MOTOR_MAX_US;

	return (uint16_t)(value + 0.5f);
}

/**
 * @brief Tính bốn lệnh theo sơ đồ Quad X rồi giới hạn từng động cơ.
 */
void Mixer_QuadX(float throttle,
				 float roll_pid,
				 float pitch_pid,
				 float yaw_pid,
				 MotorMixer_t *motor)
{
	if (motor == NULL)
		return;

	/*
	 *          FRONT
	 *
	 *     M4 CW     M2 CCW
	 *
	 *           FC
	 *
	 *     M3 CCW    M1 CW
	 *
	 *           REAR
	 *
	 * IMU convention:
	 * roll+  = rear down
	 * pitch+ = right down
	 * yaw+   = CCW
	 *
	 * PID output = target - feedback
	 */

	/**
	 * @brief Lệnh M1 trước giới hạn, µs; dấu trộn (-roll, -pitch, +yaw).
	 */
	float m1 = throttle - roll_pid - pitch_pid + yaw_pid;

	/**
	 * @brief Lệnh M2 trước giới hạn, µs; dấu trộn (+roll, -pitch, -yaw).
	 */
	float m2 = throttle + roll_pid - pitch_pid - yaw_pid;

	/**
	 * @brief Lệnh M3 trước giới hạn, µs; dấu trộn (-roll, +pitch, -yaw).
	 */
	float m3 = throttle - roll_pid + pitch_pid - yaw_pid;

	/**
	 * @brief Lệnh M4 trước giới hạn, µs; dấu trộn (+roll, +pitch, +yaw).
	 */
	float m4 = throttle + roll_pid + pitch_pid + yaw_pid;

	motor->m1 = LimitMotor(m1);
	motor->m2 = LimitMotor(m2);
	motor->m3 = LimitMotor(m3);
	motor->m4 = LimitMotor(m4);
}
