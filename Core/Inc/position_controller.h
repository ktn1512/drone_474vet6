/*
 * position_controller.h
 *
 *  Created on: 6 thg 9, 2026
 *      Author: khanh
 */

#ifndef INC_POSITION_CONTROLLER_H_
#define INC_POSITION_CONTROLLER_H_

#include "pid.h"

typedef struct {
	PID_t north;
	PID_t east;

} PositionController_t;

#endif /* INC_POSITION_CONTROLLER_H_ */
