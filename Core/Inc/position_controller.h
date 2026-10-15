/**
 * @file position_controller.h
 * @brief Khai báo cấu trúc trạng thái cho vòng điều khiển vị trí chưa có API.
 */
/*
 * position_controller.h
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_POSITION_CONTROLLER_H_
#define INC_POSITION_CONTROLLER_H_

#include "pid.h"

/**
 * @brief Cấu trúc dự phòng chứa hai PID theo phương Bắc và Đông cho vòng vị trí.
 * @note Header hiện chỉ khai báo trạng thái; chưa có API cập nhật,
 *       nên đơn vị đầu vào/đầu ra và cấu hình PID chưa được quy định ở đây.
 */
typedef struct {
	/**
	 * @brief Trạng thái PID phương Bắc; đơn vị theo cấu hình vòng vị trí của bên sử dụng.
	 */
	PID_t north;
	/**
	 * @brief Trạng thái PID phương Đông; đơn vị theo cấu hình vòng vị trí của bên sử dụng.
	 */
	PID_t east;

} PositionController_t;

#endif /* INC_POSITION_CONTROLLER_H_ */
