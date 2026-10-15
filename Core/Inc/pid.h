/**
 * @file pid.h
 * @brief Khai báo bộ PID dùng cho các vòng điều khiển, với đơn vị do từng vòng quy định.
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

/**
 * @brief Hệ số, giới hạn và trạng thái của bộ điều khiển PID.
 * @details Gọi E là đơn vị của setpoint/feedback và U là đơn vị đầu ra;
 *          các hệ số phải được chọn theo E và U của vòng điều khiển.
 */
typedef struct
{
	/**
	 * @brief Hệ số tỉ lệ, đơn vị U/E.
	 */
	float kp;
	/**
	 * @brief Hệ số tích phân, đơn vị U/(E*s).
	 */
	float ki;
	/**
	 * @brief Hệ số vi phân, đơn vị U*s/E.
	 */
	float kd;

	/**
	 * @brief Thành phần I đã bao gồm ki, đơn vị U; giới hạn bởi integral_limit.
	 */
	float integral;
	/**
	 * @brief Sai số lần cập nhật hợp lệ trước, đơn vị E; hiện chỉ được lưu, không dùng tính D.
	 */
	float prev_error;
	/**
	 * @brief Feedback lần trước, đơn vị E, dùng tính đạo hàm trên phép đo.
	 */
	float prev_feedback;
	/**
	 * @brief Đạo hàm feedback sau lọc thông thấp, đơn vị E/s.
	 */
	float derivative_filtered;

	/**
	 * @brief Biên độ giới hạn thành phần I, đơn vị U; PID_Init lưu trị tuyệt đối.
	 */
	float integral_limit;
	/**
	 * @brief Biên độ giới hạn đầu ra, đơn vị U; PID_Init lưu trị tuyệt đối.
	 */
	float output_limit;

	/**
	 * @brief Cờ đã có feedback đầu tiên: 0 sau reset, 1 sau cập nhật hợp lệ.
	 */
	uint8_t initialized;
} PID_t;

/**
 * @brief Gán hệ số, lấy trị tuyệt đối các giới hạn và xóa trạng thái PID.
 * @param[out] pid Bộ PID được khởi tạo; phải là con trỏ hợp lệ.
 * @param[in] kp Hệ số tỉ lệ, đơn vị U/E.
 * @param[in] ki Hệ số tích phân, đơn vị U/(E*s).
 * @param[in] kd Hệ số vi phân, đơn vị U*s/E.
 * @param[in] integral_limit Biên độ giới hạn thành phần I, đơn vị U.
 * @param[in] output_limit Biên độ giới hạn đầu ra, đơn vị U.
 * @note Hàm không kiểm tra NULL hoặc tính hữu hạn của hệ số/giới hạn.
 */
void PID_Init(PID_t *pid,
			  float kp, float ki, float kd,
			  float integral_limit,
			  float output_limit);

/**
 * @brief Xóa tích phân, lịch sử sai số/phép đo, bộ lọc D và cờ khởi tạo.
 * @param[in,out] pid Bộ PID cần reset; phải là con trỏ hợp lệ.
 * @note Giữ nguyên hệ số và các giới hạn; không kiểm tra NULL.
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Tính PID với đạo hàm trên feedback, lọc D và chống tích phân bão hòa.
 * @param[in,out] pid Bộ PID đã khởi tạo; phải là con trỏ hợp lệ.
 * @param[in] setpoint Giá trị đặt, cùng đơn vị E với feedback.
 * @param[in] feedback Giá trị đo, đơn vị E.
 * @param[in] dt Khoảng thời gian cập nhật, giây; chấp nhận 0,0005 đến 0,01 s.
 * @return Đầu ra đơn vị U, được giới hạn trong [-output_limit, output_limit]
 *         khi các hệ số và trạng thái hữu hạn.
 * @retval 0.0f Khi setpoint/feedback/dt không hữu hạn hoặc dt ngoài khoảng;
 *              trạng thái PID được reset trước khi trả về.
 * @note Thành phần D có dấu âm, dùng lọc thông thấp với tau = 0,0053 s.
 *       Lần cập nhật hợp lệ đầu tiên sau reset có đạo hàm đo bằng 0.
 *       Thành phần I chỉ nhận giá trị mới nếu sai số không đẩy đầu ra ứng viên
 *       vượt giới hạn xa hơn. Không kiểm tra NULL hay tính hữu hạn của hệ số.
 */
float PID_Update(PID_t *pid,
				 float setpoint,
				 float feedback,
				 float dt);

#endif
