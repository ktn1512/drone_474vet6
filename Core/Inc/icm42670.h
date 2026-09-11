/*
 * icm42670.c
 *
 *  Created on: 22 thg 8, 2026
 *      Author: khanh
 */
#pragma once

#ifndef __ICM42670_H
#define __ICM42670_H

#include "stm32g4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define ICM42670_SPI        hspi1

#define ICM_CS_PORT         GPIOA
#define ICM_CS_PIN          GPIO_PIN_4

#define ICM_WHO_AM_I        0x75
#define ICM_PWR_MGMT0       0x1F

#define ICM_ACCEL_DATA_X1   0x0B
#define ICM_GYRO_DATA_X1    0x11

typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t gx;
	int16_t gy;
	int16_t gz;

} ICM42670_Data_t;

void ICM42670_Init(void);

uint8_t ICM42670_ReadReg(uint8_t reg);

void ICM42670_WriteReg(uint8_t reg, uint8_t data);

void ICM42670_ReadAccel(ICM42670_Data_t *imu);

void ICM42670_ReadGyro(ICM42670_Data_t *imu);

void ICM42670_ReadAll(ICM42670_Data_t *imu);

#endif /* INC_ICM42670_C_ */
