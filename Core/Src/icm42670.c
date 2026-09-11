/*
 * icm42670.c
 *
 *  Created on: 22 thg 8, 2026
 *      Author: khanh
 */

#include "icm42670.h"

static void CS_Low(void) {
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

static void CS_High(void) {
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}

uint8_t ICM42670_ReadReg(uint8_t reg) {
	uint8_t tx = reg | 0x80;
	uint8_t rx = 0;

	CS_Low();

	HAL_SPI_Transmit(&ICM42670_SPI, &tx, 1, 100);
	HAL_SPI_Receive(&ICM42670_SPI, &rx, 1, 100);

	CS_High();

	return rx;
}

void ICM42670_WriteReg(uint8_t reg, uint8_t data) {
	uint8_t buf[2];

	buf[0] = reg & 0x7F;
	buf[1] = data;

	CS_Low();

	HAL_SPI_Transmit(&ICM42670_SPI, buf, 2, 100);

	CS_High();
}

static void ICM42670_ReadBytes(uint8_t reg, uint8_t *data, uint16_t len) {
	uint8_t addr = reg | 0x80;

	CS_Low();

	HAL_SPI_Transmit(&ICM42670_SPI, &addr, 1, 100);
	HAL_SPI_Receive(&ICM42670_SPI, data, len, 100);

	CS_High();
}

void ICM42670_Init(void) {
	HAL_Delay(100);

	ICM42670_WriteReg(ICM_PWR_MGMT0, 0x0F);

	HAL_Delay(50);
}

void ICM42670_ReadAccel(ICM42670_Data_t *imu) {
	uint8_t buf[6];

	ICM42670_ReadBytes(ICM_ACCEL_DATA_X1, buf, 6);

	imu->ax = (int16_t) ((buf[0] << 8) | buf[1]);
	imu->ay = (int16_t) ((buf[2] << 8) | buf[3]);
	imu->az = (int16_t) ((buf[4] << 8) | buf[5]);
}

void ICM42670_ReadGyro(ICM42670_Data_t *imu) {
	uint8_t buf[6];

	ICM42670_ReadBytes(ICM_GYRO_DATA_X1, buf, 6);

	imu->gx = (int16_t) ((buf[0] << 8) | buf[1]);
	imu->gy = (int16_t) ((buf[2] << 8) | buf[3]);
	imu->gz = (int16_t) ((buf[4] << 8) | buf[5]);
}

void ICM42670_ReadAll(ICM42670_Data_t *imu) {
	uint8_t buf[12];

	ICM42670_ReadBytes(ICM_ACCEL_DATA_X1, buf, 12);

	imu->ax = (int16_t) ((buf[0] << 8) | buf[1]);
	imu->ay = (int16_t) ((buf[2] << 8) | buf[3]);
	imu->az = (int16_t) ((buf[4] << 8) | buf[5]);

	imu->gx = (int16_t) ((buf[6] << 8) | buf[7]);
	imu->gy = (int16_t) ((buf[8] << 8) | buf[9]);
	imu->gz = (int16_t) ((buf[10] << 8) | buf[11]);
}
