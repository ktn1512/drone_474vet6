/*
 * icm42670.h
 * ICM-42670-P IMU driver
 */

#ifndef INC_ICM42670_H_
#define INC_ICM42670_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

#define ICM42670_SPI        hspi1
#define ICM_CS_PORT         GPIOA
#define ICM_CS_PIN          GPIO_PIN_4

/* Device registers */
#define ICM_WHO_AM_I        0x75U
#define ICM_PWR_MGMT0       0x1FU
#define ICM_GYRO_CONFIG0    0x20U
#define ICM_ACCEL_CONFIG0   0x21U
#define ICM_GYRO_CONFIG1    0x23U
#define ICM_ACCEL_CONFIG1   0x24U
#define ICM_TEMP_DATA1      0x09U
#define ICM_ACCEL_DATA_X1   0x0BU
#define ICM_GYRO_DATA_X1    0x11U

#define ICM_WHO_AM_I_VALUE  0x67U

/* Configuration used by the driver */
#define ICM_GYRO_FS_DPS     2000.0f
#define ICM_ACCEL_FS_G      8.0f
#define ICM_GYRO_LSB_PER_DPS 16.4f
#define ICM_ACCEL_LSB_PER_G  4096.0f

/* Init/configuration error codes */
typedef enum
{
    ICM42670_OK = 0,
    ICM42670_ERROR = 1,
    ICM42670_TIMEOUT = 2,
    ICM42670_WHOAMI_ERROR = 3
} ICM42670_Status_t;

typedef struct
{
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;

    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    int16_t temp_raw;

    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float temperature_c;

} ICM42670_Data_t;

/* Driver state */
typedef struct
{
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;

    uint8_t initialized;
} ICM42670_t;

ICM42670_Status_t ICM42670_Init(ICM42670_t *dev);
ICM42670_Status_t ICM42670_ReadReg(uint8_t reg, uint8_t *data);
ICM42670_Status_t ICM42670_WriteReg(uint8_t reg, uint8_t data);
ICM42670_Status_t ICM42670_ReadRaw(ICM42670_Data_t *imu);
ICM42670_Status_t ICM42670_ReadAll(ICM42670_Data_t *imu);

void ICM42670_ApplyCalibration(ICM42670_t *dev, ICM42670_Data_t *imu);
ICM42670_Status_t ICM42670_CalibrateGyro(ICM42670_t *dev, uint16_t samples, uint16_t sample_delay_ms);

#endif /* INC_ICM42670_H_ */
