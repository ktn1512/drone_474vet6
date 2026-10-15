/**
 * @file mixer.h
 * @brief Khai báo phép trộn Quad X và các lệnh động cơ.
 */
/*
 * mixer.h
 *
 *  Created on: 13 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_MIXER_H_
#define INC_MIXER_H_

#include <stdint.h>

/**
 * @brief Bốn lệnh động cơ sau trộn và giới hạn, biểu diễn độ rộng xung µs.
 */
typedef struct {
	/**
	 * @brief Lệnh M1 phía sau bên phải, quay CW theo sơ đồ mixer, µs.
	 */
	uint16_t m1;
	/**
	 * @brief Lệnh M2 phía trước bên phải, quay CCW theo sơ đồ mixer, µs.
	 */
	uint16_t m2;
	/**
	 * @brief Lệnh M3 phía sau bên trái, quay CCW theo sơ đồ mixer, µs.
	 */
	uint16_t m3;
	/**
	 * @brief Lệnh M4 phía trước bên trái, quay CW theo sơ đồ mixer, µs.
	 */
	uint16_t m4;

} MotorMixer_t;

/**
 * @brief Trộn throttle và hiệu chỉnh ba trục theo sơ đồ Quad X hiện tại.
 * @param[in] throttle Mức nền chung của bốn động cơ, µs.
 * @param[in] roll_pid Hiệu chỉnh roll từ vòng tốc độ, µs.
 * @param[in] pitch_pid Hiệu chỉnh pitch từ vòng tốc độ, µs.
 * @param[in] yaw_pid Hiệu chỉnh yaw từ vòng tốc độ, µs.
 * @param[out] motor Nơi nhận bốn lệnh đã giới hạn; nếu NULL, hàm trả về ngay.
 * @note Từng lệnh được chặn trong 1000–1700 µs và làm tròn về số nguyên;
 *       lệnh không hữu hạn trở thành 1000 µs. Dấu trộn tuân theo sơ đồ
 *       và quy ước IMU ghi trong mixer.c, không tự hoán đổi trục.
 */
void Mixer_QuadX(float throttle,

float roll_pid, float pitch_pid, float yaw_pid,

MotorMixer_t *motor);
#endif /* INC_MIXER_H_ */
