/**
 * @file mahony.h
 * @brief Giao diện ước lượng tư thế bằng bộ lọc Mahony gyro/accel.
 */

/*
 * mahony.h
 *
 *  Created on: 13 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_MAHONY_H_
#define INC_MAHONY_H_

/**
 * @brief Trạng thái quaternion và phản hồi của bộ lọc Mahony dùng gyro/accel.
 */
typedef struct {
	/**
	 * @brief Thành phần vô hướng quaternion định hướng, không đơn vị.
	 */
	float q0;
	/**
	 * @brief Thành phần X của phần vector quaternion, không đơn vị.
	 */
	float q1;
	/**
	 * @brief Thành phần Y của phần vector quaternion, không đơn vị.
	 */
	float q2;
	/**
	 * @brief Thành phần Z của phần vector quaternion, không đơn vị.
	 */
	float q3;

	/**
	 * @brief Hai lần hệ số phản hồi tỉ lệ; nhân sai số hướng trọng lực để hiệu chỉnh gyro.
	 */
	float twoKp;
	/**
	 * @brief Hai lần hệ số phản hồi tích phân; tích lũy sai số theo dt.
	 */
	float twoKi;

	/**
	 * @brief Phản hồi tích phân trục X cộng vào gyro nội bộ, rad/giây.
	 */
	float integralFBx;
	/**
	 * @brief Phản hồi tích phân trục Y cộng vào gyro nội bộ, rad/giây.
	 */
	float integralFBy;
	/**
	 * @brief Phản hồi tích phân trục Z cộng vào gyro nội bộ, rad/giây.
	 */
	float integralFBz;

} Mahony_t;

/**
 * @brief Đặt quaternion đơn vị, hệ số phản hồi mặc định và xóa tích phân.
 * @param[out] mahony Trạng thái bộ lọc cần khởi tạo; con trỏ phải hợp lệ.
 */
void Mahony_Init(Mahony_t *mahony);

/* Initialize tilt from stationary accelerometer data; yaw starts at zero.
 * Sensor axes must match the gyro axes, with +Z reading +1 g when level.
 * Returns 1 on success, 0 for invalid acceleration (state unchanged).
 */
/**
 * @brief Khởi tạo roll/pitch từ vector gia tốc và đặt yaw bằng 0.
 * @param[in,out] mahony Trạng thái được khởi tạo lại nếu vector gia tốc hợp lệ.
 * @param[in] ax Gia tốc trục X, đơn vị g.
 * @param[in] ay Gia tốc trục Y, đơn vị g.
 * @param[in] az Gia tốc trục Z, đơn vị g.
 * @retval 1 Đã khởi tạo quaternion và các hệ số/phản hồi qua Mahony_Init.
 * @retval 0 Bình phương độ lớn không hữu hạn hoặc nhỏ hơn 1e-6; trạng thái giữ nguyên.
 * @note Dùng mẫu đứng yên; trục accel phải trùng gyro và khi cân bằng +Z đo +1 g.
 * @note Hàm không kiểm tra con trỏ mahony.
 */
int Mahony_InitFromAccel(Mahony_t *mahony, float ax, float ay, float az);

/* Gyroscope inputs: degrees/s; accelerometer inputs: g; dt: seconds. */
/**
 * @brief Cập nhật quaternion bằng gyro và phản hồi hướng trọng lực từ accel.
 * @param[in,out] mahony Trạng thái quaternion và tích phân phản hồi.
 * @param[in] gx Vận tốc góc trục X, độ/giây; hàm đổi sang rad/giây nội bộ.
 * @param[in] gy Vận tốc góc trục Y, độ/giây; hàm đổi sang rad/giây nội bộ.
 * @param[in] gz Vận tốc góc trục Z, độ/giây; hàm đổi sang rad/giây nội bộ.
 * @param[in] ax Gia tốc trục X, g; vector được chuẩn hóa khi khác vector 0.
 * @param[in] ay Gia tốc trục Y, g.
 * @param[in] az Gia tốc trục Z, g.
 * @param[in] dt Khoảng thời gian tích phân của lần cập nhật, đơn vị giây.
 * @note Vector accel bằng 0 bỏ qua phản hồi accel; vẫn tích phân gyro và chuẩn hóa quaternion.
 * @note Người gọi cung cấp con trỏ và số liệu hữu hạn hợp lệ; hàm không kiểm tra dt.
 */
void Mahony_Update(Mahony_t *mahony,

float gx, float gy, float gz,

float ax, float ay, float az,

float dt);

/**
 * @brief Tính roll, pitch, yaw từ quaternion hiện tại.
 * @param[in] mahony Trạng thái bộ lọc chứa quaternion đã chuẩn hóa.
 * @param[out] roll Nơi nhận góc roll, đơn vị độ.
 * @param[out] pitch Nơi nhận góc pitch, đơn vị độ.
 * @param[out] yaw Nơi nhận góc yaw, đơn vị độ.
 * @note Các con trỏ phải hợp lệ; yaw không được tham chiếu Bắc bằng từ kế.
 */
void Mahony_GetEuler(Mahony_t *mahony,

float *roll, float *pitch, float *yaw);

#endif /* INC_MAHONY_H_ */
