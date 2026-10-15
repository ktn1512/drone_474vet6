/**
 * @file mahony.c
 * @brief Ước lượng quaternion và góc Euler bằng phản hồi Mahony gyro/accel.
 */

/*
 * mahony.c
 *
 *  Created on: 13 thg 9, 2026
 *      Author: khanh
 */

#include "mahony.h"
#include <math.h>

/**
 * @brief Tính nghịch đảo căn bậc hai của một số.
 * @param[in] x Giá trị cần tính căn; phải dương để nhận kết quả hữu hạn.
 * @return 1 / sqrtf(x); không có bước kiểm tra miền giá trị.
 */
static float invSqrt(float x) {
	return 1.0f / sqrtf(x);
}

/**
 * @brief Khởi tạo quaternion đơn vị, các hệ số và tích phân phản hồi.
 * @see Mahony_Init trong mahony.h để biết hợp đồng tham số.
 */
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

/**
 * @brief Tạo quaternion từ roll/pitch của accel với yaw bằng 0.
 * @see Mahony_InitFromAccel trong mahony.h để biết hợp đồng tham số.
 */
int Mahony_InitFromAccel(Mahony_t *m, float ax, float ay, float az) {
	/**
	 * @brief Bình phương độ lớn vector gia tốc, g², để loại mẫu nhỏ hoặc không hữu hạn.
	 */
	float normSq = ax * ax + ay * ay + az * az;
	if (!isfinite(normSq) || normSq < 1.0e-6f) {
		return 0;
	}

	/**
	 * @brief Góc roll suy từ accel, radian.
	 */
	float roll = atan2f(ay, az);
	/**
	 * @brief Góc pitch suy từ accel, radian.
	 */
	float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
	/**
	 * @brief Cosine nửa góc roll, không đơn vị.
	 */
	float cr = cosf(0.5f * roll);
	/**
	 * @brief Sine nửa góc roll, không đơn vị.
	 */
	float sr = sinf(0.5f * roll);
	/**
	 * @brief Cosine nửa góc pitch, không đơn vị.
	 */
	float cp = cosf(0.5f * pitch);
	/**
	 * @brief Sine nửa góc pitch, không đơn vị.
	 */
	float sp = sinf(0.5f * pitch);

	Mahony_Init(m);
	/* Euler roll/pitch, yaw = 0, using the same convention as GetEuler. */
	m->q0 = cr * cp;
	m->q1 = sr * cp;
	m->q2 = cr * sp;
	m->q3 = -sr * sp;
	return 1;
}

/**
 * @brief Đổi gyro từ độ/giây sang rad/giây rồi cập nhật quaternion.
 * @see Mahony_Update trong mahony.h để biết hợp đồng tham số.
 */
void Mahony_Update(Mahony_t *m,

float gx, float gy, float gz,

float ax, float ay, float az,

float dt) {
	/**
	 * @brief Nghịch đảo độ lớn vector dùng chuẩn hóa accel rồi quaternion.
	 */
	float recipNorm;

	/**
	 * @brief Nửa thành phần X của hướng trọng lực ước lượng từ quaternion, không đơn vị.
	 */
	float halfvx;
	/**
	 * @brief Nửa thành phần Y của hướng trọng lực ước lượng từ quaternion, không đơn vị.
	 */
	float halfvy;
	/**
	 * @brief Nửa thành phần Z của hướng trọng lực ước lượng từ quaternion, không đơn vị.
	 */
	float halfvz;

	/**
	 * @brief Thành phần X của tích có hướng giữa accel chuẩn hóa và nửa hướng trọng lực.
	 */
	float halfex;
	/**
	 * @brief Thành phần Y của tích có hướng giữa accel chuẩn hóa và nửa hướng trọng lực.
	 */
	float halfey;
	/**
	 * @brief Thành phần Z của tích có hướng giữa accel chuẩn hóa và nửa hướng trọng lực.
	 */
	float halfez;

	/**
	 * @brief Bản sao q0 trước bước tích phân để các phép tính dùng cùng trạng thái cũ.
	 */
	float qa;
	/**
	 * @brief Bản sao q1 trước bước tích phân, không đơn vị.
	 */
	float qb;
	/**
	 * @brief Bản sao q2 trước bước tích phân, không đơn vị.
	 */
	float qc;

	/* Feedback is in radians/s: convert gyro before adding feedback. */
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

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

	gx *= 0.5f * dt;
	gy *= 0.5f * dt;
	gz *= 0.5f * dt;

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

/**
 * @brief Xuất góc Euler theo độ từ quaternion hiện tại.
 * @see Mahony_GetEuler trong mahony.h để biết hợp đồng tham số.
 */
void Mahony_GetEuler(Mahony_t *m,

float *roll, float *pitch, float *yaw) {
	*roll = atan2f(2.0f * (m->q0 * m->q1 + m->q2 * m->q3),
			1.0f - 2.0f * (m->q1 * m->q1 + m->q2 * m->q2)) * 57.29578f;

	/* Rounding can put sin(pitch) slightly outside [-1, 1] near 90 deg. */
	/**
	 * @brief Sine góc pitch, không đơn vị; được giới hạn về [-1, 1] trước asinf.
	 */
	float sinPitch = 2.0f * (m->q0 * m->q2 - m->q3 * m->q1);
	sinPitch = fmaxf(-1.0f, fminf(1.0f, sinPitch));
	*pitch = asinf(sinPitch) * 57.29578f;

	*yaw = atan2f(2.0f * (m->q0 * m->q3 + m->q1 * m->q2),
			1.0f - 2.0f * (m->q2 * m->q2 + m->q3 * m->q3)) * 57.29578f;
}
